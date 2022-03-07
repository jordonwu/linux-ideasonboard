// SPDX-License-Identifier: GPL-2.0
/*
 * V4L2 Capture ISI subdev driver for i.MX8QXP/QM platform
 *
 * ISI is a Image Sensor Interface of i.MX8QXP/QM platform, which
 * used to process image from camera sensor to memory or DC
 *
 * Copyright (c) 2019 NXP Semiconductor
 */

#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/media-bus-format.h>
#include <linux/minmax.h>
#include <linux/pm_runtime.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/videodev2.h>

#include <media/media-entity.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-subdev.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-dma-contig.h>
#include <media/videobuf2-v4l2.h>

#include "imx8-isi-core.h"
#include "imx8-isi-regs.h"

/* Keep the first entry matching MXC_ISI_DEF_PIXEL_FORMAT */
static const struct mxc_isi_format_info mxc_isi_out_formats[] = {
	/* YUV formats */
	{
		.mbus_code	= MEDIA_BUS_FMT_YUV8_1X24,
		.fourcc		= V4L2_PIX_FMT_YUYV,
		.isi_format	= CHNL_IMG_CTRL_FORMAT_YUV422_1P8P,
		.memplanes	= 1,
		.depth		= { 16 },
		.encoding	= MXC_ISI_ENC_YUV,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_YUV8_1X24,
		.fourcc		= V4L2_PIX_FMT_YUVA32,
		.isi_format	= CHNL_IMG_CTRL_FORMAT_YUV444_1P8,
		.memplanes	= 1,
		.depth		= { 32 },
		.encoding	= MXC_ISI_ENC_YUV,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_YUV8_1X24,
		.fourcc		= V4L2_PIX_FMT_NV12M,
		.isi_format	= CHNL_IMG_CTRL_FORMAT_YUV420_2P8P,
		.memplanes	= 2,
		.depth		= { 8, 16 },
		.hsub		= 2,
		.vsub		= 2,
		.encoding	= MXC_ISI_ENC_YUV,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_YUV8_1X24,
		.fourcc		= V4L2_PIX_FMT_NV16M,
		.isi_format	= CHNL_IMG_CTRL_FORMAT_YUV422_2P8P,
		.memplanes	= 2,
		.depth		= { 8, 16 },
		.hsub		= 2,
		.vsub		= 1,
		.encoding	= MXC_ISI_ENC_YUV,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_YUV8_1X24,
		.fourcc		= V4L2_PIX_FMT_YUV444M,
		.isi_format	= CHNL_IMG_CTRL_FORMAT_YUV444_3P8P,
		.memplanes	= 3,
		.depth		= { 8, 8, 8 },
		.hsub		= 1,
		.vsub		= 1,
		.encoding	= MXC_ISI_ENC_YUV,
	},
	/* RGB formats */
	{
		.mbus_code	= MEDIA_BUS_FMT_RGB888_1X24,
		.fourcc		= V4L2_PIX_FMT_RGB565,
		.isi_format	= CHNL_IMG_CTRL_FORMAT_RGB565,
		.memplanes	= 1,
		.depth		= { 16 },
		.encoding	= MXC_ISI_ENC_RGB,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_RGB888_1X24,
		.fourcc		= V4L2_PIX_FMT_RGB24,
		.isi_format	= CHNL_IMG_CTRL_FORMAT_BGR888P,
		.memplanes	= 1,
		.depth		= { 24 },
		.encoding	= MXC_ISI_ENC_RGB,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_RGB888_1X24,
		.fourcc		= V4L2_PIX_FMT_BGR24,
		.isi_format	= CHNL_IMG_CTRL_FORMAT_RGB888P,
		.memplanes	= 1,
		.depth		= { 24 },
		.encoding	= MXC_ISI_ENC_RGB,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_RGB888_1X24,
		.fourcc		= V4L2_PIX_FMT_XBGR32,
		.isi_format	= CHNL_IMG_CTRL_FORMAT_XRGB888,
		.memplanes	= 1,
		.depth		= { 32 },
		.encoding	= MXC_ISI_ENC_RGB,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_RGB888_1X24,
		.fourcc		= V4L2_PIX_FMT_ABGR32,
		.isi_format	= CHNL_IMG_CTRL_FORMAT_ARGB8888,
		.memplanes	= 1,
		.depth		= { 32 },
		.encoding	= MXC_ISI_ENC_RGB,
	},
	/*
	 * RAW formats
	 *
	 * The ISI shifts the 10-bit and 12-bit formats left by 6 and 4 bits
	 * when using CHNL_IMG_CTRL_FORMAT_RAW10 or MXC_ISI_OUT_FMT_RAW12
	 * respectively, to align the bits to the left and pad with zeros in
	 * the LSBs. The corresponding V4L2 formats are however right-aligned,
	 * we have to use CHNL_IMG_CTRL_FORMAT_RAW16 to avoid the left shift.
	 */
	{
		.mbus_code	= MEDIA_BUS_FMT_Y8_1X8,
		.fourcc		= V4L2_PIX_FMT_GREY,
		.isi_format	= CHNL_IMG_CTRL_FORMAT_RAW8,
		.memplanes	= 1,
		.depth		= { 8 },
		.encoding	= MXC_ISI_ENC_RAW,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_Y10_1X10,
		.fourcc		= V4L2_PIX_FMT_Y10,
		.isi_format	= CHNL_IMG_CTRL_FORMAT_RAW16,
		.memplanes	= 1,
		.depth		= { 16 },
		.encoding	= MXC_ISI_ENC_RAW,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_Y12_1X12,
		.fourcc		= V4L2_PIX_FMT_Y12,
		.isi_format	= CHNL_IMG_CTRL_FORMAT_RAW16,
		.memplanes	= 1,
		.depth		= { 16 },
		.encoding	= MXC_ISI_ENC_RAW,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_Y14_1X14,
		.fourcc		= V4L2_PIX_FMT_Y14,
		.isi_format	= CHNL_IMG_CTRL_FORMAT_RAW16,
		.memplanes	= 1,
		.depth		= { 16 },
		.encoding	= MXC_ISI_ENC_RAW,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SBGGR8_1X8,
		.fourcc		= V4L2_PIX_FMT_SBGGR8,
		.isi_format	= CHNL_IMG_CTRL_FORMAT_RAW8,
		.memplanes	= 1,
		.depth		= { 8 },
		.encoding	= MXC_ISI_ENC_RAW,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SGBRG8_1X8,
		.fourcc		= V4L2_PIX_FMT_SGBRG8,
		.isi_format	= CHNL_IMG_CTRL_FORMAT_RAW8,
		.memplanes	= 1,
		.depth		= { 8 },
		.encoding	= MXC_ISI_ENC_RAW,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SGRBG8_1X8,
		.fourcc		= V4L2_PIX_FMT_SGRBG8,
		.isi_format	= CHNL_IMG_CTRL_FORMAT_RAW8,
		.memplanes	= 1,
		.depth		= { 8 },
		.encoding	= MXC_ISI_ENC_RAW,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SRGGB8_1X8,
		.fourcc		= V4L2_PIX_FMT_SRGGB8,
		.isi_format	= CHNL_IMG_CTRL_FORMAT_RAW8,
		.memplanes	= 1,
		.depth		= { 8 },
		.encoding	= MXC_ISI_ENC_RAW,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SBGGR10_1X10,
		.fourcc		= V4L2_PIX_FMT_SBGGR10,
		.isi_format	= CHNL_IMG_CTRL_FORMAT_RAW16,
		.memplanes	= 1,
		.depth		= { 16 },
		.encoding	= MXC_ISI_ENC_RAW,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SGBRG10_1X10,
		.fourcc		= V4L2_PIX_FMT_SGBRG10,
		.isi_format	= CHNL_IMG_CTRL_FORMAT_RAW16,
		.memplanes	= 1,
		.depth		= { 16 },
		.encoding	= MXC_ISI_ENC_RAW,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SGRBG10_1X10,
		.fourcc		= V4L2_PIX_FMT_SGRBG10,
		.isi_format	= CHNL_IMG_CTRL_FORMAT_RAW16,
		.memplanes	= 1,
		.depth		= { 16 },
		.encoding	= MXC_ISI_ENC_RAW,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SRGGB10_1X10,
		.fourcc		= V4L2_PIX_FMT_SRGGB10,
		.isi_format	= CHNL_IMG_CTRL_FORMAT_RAW16,
		.memplanes	= 1,
		.depth		= { 16 },
		.encoding	= MXC_ISI_ENC_RAW,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SBGGR12_1X12,
		.fourcc		= V4L2_PIX_FMT_SBGGR12,
		.isi_format	= CHNL_IMG_CTRL_FORMAT_RAW16,
		.memplanes	= 1,
		.depth		= { 16 },
		.encoding	= MXC_ISI_ENC_RAW,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SGBRG12_1X12,
		.fourcc		= V4L2_PIX_FMT_SGBRG12,
		.isi_format	= CHNL_IMG_CTRL_FORMAT_RAW16,
		.memplanes	= 1,
		.depth		= { 16 },
		.encoding	= MXC_ISI_ENC_RAW,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SGRBG12_1X12,
		.fourcc		= V4L2_PIX_FMT_SGRBG12,
		.isi_format	= CHNL_IMG_CTRL_FORMAT_RAW16,
		.memplanes	= 1,
		.depth		= { 16 },
		.encoding	= MXC_ISI_ENC_RAW,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SRGGB12_1X12,
		.fourcc		= V4L2_PIX_FMT_SRGGB12,
		.isi_format	= CHNL_IMG_CTRL_FORMAT_RAW16,
		.memplanes	= 1,
		.depth		= { 16 },
		.encoding	= MXC_ISI_ENC_RAW,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SBGGR14_1X14,
		.fourcc		= V4L2_PIX_FMT_SBGGR14,
		.isi_format	= CHNL_IMG_CTRL_FORMAT_RAW16,
		.memplanes	= 1,
		.depth		= { 16 },
		.encoding	= MXC_ISI_ENC_RAW,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SGBRG14_1X14,
		.fourcc		= V4L2_PIX_FMT_SGBRG14,
		.isi_format	= CHNL_IMG_CTRL_FORMAT_RAW16,
		.memplanes	= 1,
		.depth		= { 16 },
		.encoding	= MXC_ISI_ENC_RAW,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SGRBG14_1X14,
		.fourcc		= V4L2_PIX_FMT_SGRBG14,
		.isi_format	= CHNL_IMG_CTRL_FORMAT_RAW16,
		.memplanes	= 1,
		.depth		= { 16 },
		.encoding	= MXC_ISI_ENC_RAW,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SRGGB14_1X14,
		.fourcc		= V4L2_PIX_FMT_SRGGB14,
		.isi_format	= CHNL_IMG_CTRL_FORMAT_RAW16,
		.memplanes	= 1,
		.depth		= { 16 },
		.encoding	= MXC_ISI_ENC_RAW,
	},
	/* JPEG */
	{
		.mbus_code	= MEDIA_BUS_FMT_JPEG_1X8,
		.fourcc		= V4L2_PIX_FMT_MJPEG,
		.isi_format	= CHNL_IMG_CTRL_FORMAT_RAW8,
		.memplanes	= 1,
		.depth		= { 8 },
		.encoding	= MXC_ISI_ENC_RAW,
	}
};

static const struct mxc_isi_format_info *mxc_isi_format_by_fourcc(u32 fourcc)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(mxc_isi_out_formats); i++) {
		const struct mxc_isi_format_info *fmt = &mxc_isi_out_formats[i];

		if (fmt->fourcc == fourcc)
			return fmt;
	}

	return NULL;
}

/* -----------------------------------------------------------------------------
 * videobuf2 queue operations
 */

void mxc_isi_video_frame_write_done(struct mxc_isi_pipe *pipe, u32 status)
{
	struct mxc_isi_video *video = &pipe->video;
	struct device *dev = pipe->isi->dev;
	struct mxc_isi_buffer *next_buf;
	struct mxc_isi_buffer *buf;
	enum mxc_isi_buf_id buf_id;

	spin_lock(&video->buf_lock);

	/*
	 * The ISI hardware handles buffers using a ping-pong mechanism with
	 * two sets of destination addresses (with shadow registers to allow
	 * programming addresses for all planes atomically) named BUF1 and
	 * BUF2. Addresses can be loaded and copied to shadow registers at any
	 * at any time.
	 *
	 * The hardware keeps track of which buffer is being written to and
	 * automatically switches to the other buffer at frame end, copying the
	 * corresponding address to another set of shadow registers that track
	 * the address being written to. The active buffer tracking bits are
	 * accessible through the CHNL_STS register.
	 *
	 *  BUF1        BUF2  |   Event   | Action
	 *                    |           |
	 *                    |           | Program initial buffers
	 *                    |           | B0 in BUF1, B1 in BUF2
	 *                    | Start ISI |
	 * +----+             |           |
	 * | B0 |             |           |
	 * +----+             |           |
	 *             +----+ | FRM IRQ 0 | B0 complete, BUF2 now active
	 *             | B1 | |           | Program B2 in BUF1
	 *             +----+ |           |
	 * +----+             | FRM IRQ 1 | B1 complete, BUF1 now active
	 * | B2 |             |           | Program B3 in BUF2
	 * +----+             |           |
	 *             +----+ | FRM IRQ 2 | B2 complete, BUF2 now active
	 *             | B3 | |           | Program B4 in BUF1
	 *             +----+ |           |
	 * +----+             | FRM IRQ 3 | B3 complete, BUF1 now active
	 * | B4 |             |           | Program B5 in BUF2
	 * +----+             |           |
	 *        ...         |           |
	 *
	 * Races between address programming and buffer switching can be
	 * detected by checking if a frame end interrupt occured after
	 * programming the addresses.
	 *
	 * As none of the shadow registers are accessible, races can occur
	 * between address programming and buffer switching. It is possible to
	 * detect the race condition by checking if a frame end interrupt
	 * occurred after programming the addresses, but impossible to
	 * determine if the race has been won or lost.
	 *
	 * In addition to this, we need to use discard buffers if no pending
	 * buffers are available. To simplify handling of discard buffer, we
	 * need to allocate three of them, as two can be active concurrently
	 * and we need to still be able to get hold of a next buffer. The logic
	 * could be improved to use two buffers only, but as all discard
	 * buffers share the same memory, an additional buffer is cheap.
	 */

	/* Check which buffer has just completed. */
	buf_id = pipe->isi->pdata->buf_active_reverse
	       ? (status & CHNL_STS_BUF1_ACTIVE ? MXC_ISI_BUF2 : MXC_ISI_BUF1)
	       : (status & CHNL_STS_BUF1_ACTIVE ? MXC_ISI_BUF1 : MXC_ISI_BUF2);

	buf = list_first_entry_or_null(&video->out_active,
				       struct mxc_isi_buffer, list);

	/* Safety check, this should really never happen. */
	if (!buf) {
		dev_warn(dev, "trying to access empty active list\n");
		goto done;
	}

	/*
	 * If the buffer that has completed doesn't match the buffer on the
	 * front of the active list, it means we have lost one frame end
	 * interrupt (or possibly a large odd number of interrupts, although
	 * quite unlikely).
	 *
	 * For instance, if IRQ1 is lost and we handle IRQ2, both B1 and B2
	 * have been completed, but B3 hasn't been programmed, BUF2 still
	 * addresses B1 and the ISI is now writting in B1 instead of B3. We
	 * can't complete B2 as that would result in out-of-order completion.
	 *
	 * The only option is to ignore this interrupt and try again. When IRQ3
	 * will be handled, we will complete B1 and be in sync again.
	 */
	if (buf->id != buf_id) {
		dev_dbg(dev, "buffer ID mismatch (expected %u, got %u), skipping\n",
			buf->id, buf_id);

		/*
		 * Increment the frame count by two to account for the missed
		 * and the ignored interrupts.
		 */
		video->frame_count += 2;
		goto done;
	}

	/* Pick the next buffer and queue it to the hardware. */
	next_buf = list_first_entry_or_null(&video->out_pending,
					    struct mxc_isi_buffer, list);
	if (!next_buf) {
		next_buf = list_first_entry_or_null(&video->out_discard,
					 	    struct mxc_isi_buffer, list);

		/* Safety check, this should never happen. */
		if (!next_buf) {
			dev_warn(dev, "trying to access empty discard list\n");
			goto done;
		}
	}

	mxc_isi_channel_set_outbuf(pipe, next_buf, buf_id);

	/*
	 * Check if we have raced with the end of frame interrupt. If so, we
	 * can't tell if the ISI has recorded the new address, or is still
	 * using the previous buffer. We must assume the latter as that is the
	 * worst case.
	 *
	 * For instance, if we are handling IRQ1 and now detect the FRM
	 * interrupt, assume B2 has completed and the ISI has switched to BUF2
	 * using B1 just before we programmed B3. Unlike in the previous race
	 * condition, B3 has been programmed and will be written to the next
	 * time the ISI switches to BUF2. We can however handle this exactly as
	 * the first race condition, as we'll program B3 (still at the head of
	 * the pending list) when handling IRQ3.
	 */
	status = mxc_isi_channel_irq_status(pipe, false);
	if (status & CHNL_STS_FRM_STRD) {
		dev_dbg(dev, "raced with frame end interrupt\n");
		video->frame_count += 2;
		goto done;
	}

	/*
	 * The next buffer has been queued successfully, move it to the active
	 * list, and complete the current buffer.
	 */
	next_buf->v4l2_buf.vb2_buf.state = VB2_BUF_STATE_ACTIVE;
	list_move_tail(&next_buf->list, &video->out_active);

	if (!buf->discard) {
		list_del_init(&buf->list);
		buf->v4l2_buf.sequence = video->frame_count;
		buf->v4l2_buf.vb2_buf.timestamp = ktime_get_ns();
		vb2_buffer_done(&buf->v4l2_buf.vb2_buf, VB2_BUF_STATE_DONE);
	} else {
		list_move_tail(&buf->list, &video->out_discard);
	}

	video->frame_count++;

done:
	spin_unlock(&video->buf_lock);
}

static void mxc_isi_video_free_discard_buffers(struct mxc_isi_video *video)
{
	unsigned int i;

	for (i = 0; i < video->pix.num_planes; i++) {
		struct mxc_isi_dma_buffer *buf = &video->discard_buffer[i];

		if (!buf->addr)
			continue;

		dma_free_coherent(video->pipe->isi->dev, buf->size, buf->addr,
				  buf->dma);
		buf->addr = NULL;
	}
}

static int mxc_isi_video_alloc_discard_buffers(struct mxc_isi_video *video)
{
	unsigned int i, j;

	/* Allocate memory for each plane. */
	for (i = 0; i < video->pix.num_planes; i++) {
		struct mxc_isi_dma_buffer *buf = &video->discard_buffer[i];

		buf->size = PAGE_ALIGN(video->pix.plane_fmt[i].sizeimage);
		buf->addr = dma_alloc_coherent(video->pipe->isi->dev, buf->size,
					       &buf->dma, GFP_DMA | GFP_KERNEL);
		if (!buf->addr) {
			mxc_isi_video_free_discard_buffers(video);
			return -ENOMEM;
		}

		dev_dbg(video->pipe->isi->dev,
			"%s: num_plane=%d discard_size=%zu discard_buffer=%p\n",
			__func__, i, buf->size, buf->addr);
	}

	/* Fill the DMA addresses in the discard buffers. */
	for (i = 0; i < ARRAY_SIZE(video->buf_discard); ++i) {
		struct mxc_isi_buffer *buf = &video->buf_discard[i];

		buf->discard = true;

		for (j = 0; j < video->pix.num_planes; ++j)
			buf->dma_addrs[j] = video->discard_buffer[j].dma;
	}

	return 0;
}

static int mxc_isi_video_validate_format(struct mxc_isi_video *video)
{
	const struct v4l2_mbus_framefmt *format;
	const struct mxc_isi_format_info *info;
	struct v4l2_subdev_state *state;
	struct v4l2_subdev *sd = &video->pipe->sd;
	int ret = 0;

	state = v4l2_subdev_lock_active_state(sd);

	info = mxc_isi_format_by_fourcc(video->pix.pixelformat);
	format = v4l2_subdev_get_try_format(sd, state, MXC_ISI_PIPE_PAD_SOURCE);

	if (format->code != info->mbus_code ||
	    format->width != video->pix.width ||
	    format->height != video->pix.height) {
		dev_dbg(video->pipe->isi->dev,
			"%s: configuration mismatch, 0x%04x/%ux%u != 0x%04x/%ux%u\n",
			__func__, format->code, format->width, format->height,
			info->mbus_code, video->pix.width, video->pix.height);
		ret = -EINVAL;
	}

	v4l2_subdev_unlock_state(state);

	return ret;
}

static void mxc_isi_video_return_buffers(struct mxc_isi_video *video,
					 enum vb2_buffer_state state)
{
	struct mxc_isi_buffer *buf;
	unsigned long flags;

	spin_lock_irqsave(&video->buf_lock, flags);

	while (!list_empty(&video->out_active)) {
		buf = list_entry(video->out_active.next,
				 struct mxc_isi_buffer, list);
		list_del_init(&buf->list);
		if (buf->discard)
			continue;

		vb2_buffer_done(&buf->v4l2_buf.vb2_buf, state);
	}

	while (!list_empty(&video->out_pending)) {
		buf = list_entry(video->out_pending.next,
				 struct mxc_isi_buffer, list);
		list_del_init(&buf->list);
		vb2_buffer_done(&buf->v4l2_buf.vb2_buf, state);
	}

	while (!list_empty(&video->out_discard)) {
		buf = list_entry(video->out_discard.next,
				 struct mxc_isi_buffer, list);
		list_del_init(&buf->list);
	}

	INIT_LIST_HEAD(&video->out_active);
	INIT_LIST_HEAD(&video->out_pending);
	INIT_LIST_HEAD(&video->out_discard);

	spin_unlock_irqrestore(&video->buf_lock, flags);
}

static inline struct mxc_isi_buffer *to_isi_buffer(struct vb2_v4l2_buffer *v4l2_buf)
{
	return container_of(v4l2_buf, struct mxc_isi_buffer, v4l2_buf);
}

static int mxc_isi_vb2_queue_setup(struct vb2_queue *q,
				   unsigned int *num_buffers,
				   unsigned int *num_planes,
				   unsigned int sizes[],
				   struct device *alloc_devs[])
{
	struct mxc_isi_video *video = vb2_get_drv_priv(q);
	const struct mxc_isi_format_info *fmt = video->fmtinfo;
	unsigned long wh;
	int i;

	for (i = 0; i < fmt->memplanes; i++)
		alloc_devs[i] = video->pipe->isi->dev;

	wh = video->pix.width * video->pix.height;

	*num_planes = fmt->memplanes;

	for (i = 0; i < fmt->memplanes; i++) {
		unsigned int size = wh * fmt->depth[i] / 8;

		if (i > 1)
			size /= fmt->hsub * fmt->vsub;

		sizes[i] = max_t(u32, size, video->pix.plane_fmt[i].sizeimage);
	}
	dev_dbg(video->pipe->isi->dev, "%s, buf_n=%d, size=%d\n",
		__func__, *num_buffers, sizes[0]);

	return 0;
}

static int mxc_isi_vb2_buffer_init(struct vb2_buffer *vb2)
{
	struct mxc_isi_buffer *buf = to_isi_buffer(to_vb2_v4l2_buffer(vb2));
	struct mxc_isi_video *video = vb2_get_drv_priv(vb2->vb2_queue);
	const struct mxc_isi_format_info *fmt = video->fmtinfo;
	unsigned int i;

	for (i = 0; i < fmt->memplanes; ++i)
		buf->dma_addrs[i] = vb2_dma_contig_plane_dma_addr(vb2, i);

	return 0;
}

static int mxc_isi_vb2_buffer_prepare(struct vb2_buffer *vb2)
{
	struct mxc_isi_video *video = vb2_get_drv_priv(vb2->vb2_queue);
	const struct mxc_isi_format_info *fmt = video->fmtinfo;
	unsigned int i;

	for (i = 0; i < fmt->memplanes; i++) {
		unsigned long size = video->pix.plane_fmt[i].sizeimage;

		if (vb2_plane_size(vb2, i) < size) {
			dev_err(video->pipe->isi->dev,
				"User buffer too small (%ld < %ld)\n",
				vb2_plane_size(vb2, i), size);
			return -EINVAL;
		}

		vb2_set_plane_payload(vb2, i, size);
	}

	return 0;
}

static void mxc_isi_vb2_buffer_queue(struct vb2_buffer *vb2)
{
	struct vb2_v4l2_buffer *v4l2_buf = to_vb2_v4l2_buffer(vb2);
	struct mxc_isi_buffer *buf = to_isi_buffer(v4l2_buf);
	struct mxc_isi_video *video = vb2_get_drv_priv(vb2->vb2_queue);
	unsigned long flags;

	spin_lock_irqsave(&video->buf_lock, flags);
	list_add_tail(&buf->list, &video->out_pending);
	spin_unlock_irqrestore(&video->buf_lock, flags);
}

static int mxc_isi_vb2_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct mxc_isi_video *video = vb2_get_drv_priv(q);
	unsigned long flags;
	unsigned int i;
	int ret;

	ret = media_pipeline_start(video->vdev.entity.pads, &video->pipe->pipe);
	if (ret < 0)
		goto err_bufs;

	/*
	 * Verify that the configured format matches the output of the
	 * subdev.
	 */
	ret = mxc_isi_video_validate_format(video);
	if (ret)
		goto err_stop;

	/* Create buffers for discard operation. */
	ret = mxc_isi_video_alloc_discard_buffers(video);
	if (ret)
		goto err_stop;

	/* Initialize the ISI channel. */
	mxc_isi_channel_init(video->pipe);
	mxc_isi_channel_set_output_format(video->pipe, video->fmtinfo,
					  &video->pix);

	spin_lock_irqsave(&video->buf_lock, flags);

	/* Add the discard buffers to the out_discard list. */
	for (i = 0; i < ARRAY_SIZE(video->buf_discard); ++i) {
		struct mxc_isi_buffer *buf = &video->buf_discard[i];

		list_add_tail(&buf->list, &video->out_discard);
	}

	/* Queue two ISI channel output buffers. */
	for (i = 0; i < 2; ++i) {
		enum mxc_isi_buf_id buf_id = i == 0 ? MXC_ISI_BUF1
					   : MXC_ISI_BUF2;
		struct mxc_isi_buffer *buf;
		struct list_head *list;

		/*
		 * We are guaranteed to have at least one buffer in the pending
		 * list. If there is a second one, queue two pending buffers,
		 * otherwise use a discard buffer for the second buffer.
		 */
		list = i == 1 && list_is_singular(&video->out_pending) 
		     ? &video->out_discard : &video->out_pending;

		buf = list_first_entry(list, struct mxc_isi_buffer, list);
		buf->v4l2_buf.vb2_buf.state = VB2_BUF_STATE_ACTIVE;

		mxc_isi_channel_set_outbuf(video->pipe, buf, buf_id);
		list_move_tail(&buf->list, &video->out_active);
	}

	/* Clear frame count */
	video->frame_count = 0;
	spin_unlock_irqrestore(&video->buf_lock, flags);

	ret = mxc_isi_pipe_enable(video->pipe);
	if (ret)
		goto err_free;

	video->is_streaming = true;

	return 0;

err_free:
	mxc_isi_video_free_discard_buffers(video);
err_stop:
	media_pipeline_stop(video->vdev.entity.pads);
err_bufs:
	mxc_isi_video_return_buffers(video, VB2_BUF_STATE_QUEUED);
	return ret;
}

static void mxc_isi_vb2_stop_streaming(struct vb2_queue *q)
{
	struct mxc_isi_video *video = vb2_get_drv_priv(q);

	mxc_isi_pipe_disable(video->pipe);

	mxc_isi_video_return_buffers(video, VB2_BUF_STATE_ERROR);
	mxc_isi_video_free_discard_buffers(video);

	media_pipeline_stop(video->vdev.entity.pads);

	video->is_streaming = false;
}

static const struct vb2_ops mxc_isi_vb2_qops = {
	.queue_setup		= mxc_isi_vb2_queue_setup,
	.buf_init		= mxc_isi_vb2_buffer_init,
	.buf_prepare		= mxc_isi_vb2_buffer_prepare,
	.buf_queue		= mxc_isi_vb2_buffer_queue,
	.wait_prepare		= vb2_ops_wait_prepare,
	.wait_finish		= vb2_ops_wait_finish,
	.start_streaming	= mxc_isi_vb2_start_streaming,
	.stop_streaming		= mxc_isi_vb2_stop_streaming,
};

/* -----------------------------------------------------------------------------
 * V4L2 controls
 */

static inline struct mxc_isi_video *ctrl_to_isi_video(struct v4l2_ctrl *ctrl)
{
	return container_of(ctrl->handler, struct mxc_isi_video, ctrls.handler);
}

static int mxc_isi_video_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct mxc_isi_video *video = ctrl_to_isi_video(ctrl);

	switch (ctrl->id) {
	case V4L2_CID_ALPHA_COMPONENT:
		video->pipe->alpha = ctrl->val;
		break;
	}

	return 0;
}

static const struct v4l2_ctrl_ops mxc_isi_video_ctrl_ops = {
	.s_ctrl = mxc_isi_video_s_ctrl,
};

static int mxc_isi_video_ctrls_create(struct mxc_isi_video *video)
{
	struct v4l2_ctrl_handler *handler = &video->ctrls.handler;
	int ret;

	v4l2_ctrl_handler_init(handler, 1);

	video->ctrls.alpha = v4l2_ctrl_new_std(handler, &mxc_isi_video_ctrl_ops,
					       V4L2_CID_ALPHA_COMPONENT,
					       0, 255, 1, 0);

	if (handler->error) {
		ret = handler->error;
		v4l2_ctrl_handler_free(&video->ctrls.handler);
		return ret;
	}

	video->vdev.ctrl_handler = handler;

	return 0;
}

static void mxc_isi_video_ctrls_delete(struct mxc_isi_video *video)
{
	v4l2_ctrl_handler_free(&video->ctrls.handler);
	video->ctrls.alpha = NULL;
}

/* -----------------------------------------------------------------------------
 * V4L2 ioctls
 */

static int mxc_isi_video_querycap(struct file *file, void *priv,
				  struct v4l2_capability *cap)
{
	struct mxc_isi_video *video = video_drvdata(file);

	strlcpy(cap->driver, MXC_ISI_CAPTURE, sizeof(cap->driver));
	strlcpy(cap->card, MXC_ISI_CAPTURE, sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:%s.%d",
		 dev_name(video->pipe->isi->dev), video->pipe->id);

	return 0;
}

static int mxc_isi_video_enum_fmt(struct file *file, void *priv,
				  struct v4l2_fmtdesc *f)
{
	const struct mxc_isi_format_info *fmt;
	unsigned int index = f->index;
	unsigned int i;

	if (f->mbus_code) {
		/*
		 * If a media bus code is specified, only enumerate formats
		 * compatible with it.
		 */
		for (i = 0; i < ARRAY_SIZE(mxc_isi_out_formats); i++) {
			fmt = &mxc_isi_out_formats[i];
			if (fmt->mbus_code != f->mbus_code)
				continue;

			if (index == 0)
				break;

			index--;
		}

		if (i == ARRAY_SIZE(mxc_isi_out_formats))
			return -EINVAL;
	} else {
		/* Otherwise, enumerate all formatS. */
		if (f->index >= ARRAY_SIZE(mxc_isi_out_formats))
			return -EINVAL;

		fmt = &mxc_isi_out_formats[f->index];
	}

	f->pixelformat = fmt->fourcc;

	return 0;
}

static int mxc_isi_video_g_fmt(struct file *file, void *fh,
			       struct v4l2_format *f)
{
	struct mxc_isi_video *video = video_drvdata(file);

	f->fmt.pix_mp = video->pix;

	return 0;
}

static void __mxc_isi_video_try_fmt(struct v4l2_pix_format_mplane *pix,
				    const struct mxc_isi_format_info **info)
{
	const struct mxc_isi_format_info *fmt;
	unsigned int i;

	fmt = mxc_isi_format_by_fourcc(pix->pixelformat);
	if (!fmt)
		fmt = &mxc_isi_out_formats[0];

	pix->width = clamp(pix->width, MXC_ISI_MIN_WIDTH, MXC_ISI_MAX_WIDTH);
	pix->height = clamp(pix->height, MXC_ISI_MIN_HEIGHT, MXC_ISI_MAX_HEIGHT);
	pix->pixelformat = fmt->fourcc;
	pix->field = V4L2_FIELD_NONE;
	pix->colorspace = V4L2_COLORSPACE_JPEG;
	pix->num_planes = fmt->memplanes;

	for (i = 0; i < pix->num_planes; i++) {
		struct v4l2_plane_pix_format *plane = &pix->plane_fmt[i];
		unsigned int bpl;

		/* The pitch must be identical for all planes. */
		if (i == 0)
			bpl = clamp(plane->bytesperline,
				    pix->width * fmt->depth[0] / 8,
				    65535U);
		else
			bpl = pix->plane_fmt[0].bytesperline;

		plane->bytesperline = bpl;

		plane->sizeimage = plane->bytesperline * pix->height;
		if (i >= 1)
			plane->sizeimage /= fmt->vsub;
	}

	pix->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(pix->colorspace);
	pix->quantization =
		V4L2_MAP_QUANTIZATION_DEFAULT(fmt->encoding == MXC_ISI_ENC_RGB,
					      pix->colorspace, pix->ycbcr_enc);
	pix->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(pix->colorspace);

	if (info)
		*info = fmt;
}

static int mxc_isi_video_try_fmt(struct file *file, void *fh,
				 struct v4l2_format *f)
{
	__mxc_isi_video_try_fmt(&f->fmt.pix_mp, NULL);
	return 0;
}

static int mxc_isi_video_s_fmt(struct file *file, void *priv,
			       struct v4l2_format *f)
{
	struct mxc_isi_video *video = video_drvdata(file);
	struct v4l2_pix_format_mplane *pix = &f->fmt.pix_mp;
	const struct mxc_isi_format_info *fmt;

	if (vb2_is_busy(&video->vb2_q))
		return -EBUSY;

	__mxc_isi_video_try_fmt(pix, &fmt);

	video->pix = *pix;
	video->fmtinfo = fmt;

	return 0;
}

static int mxc_isi_video_enum_framesizes(struct file *file, void *priv,
					 struct v4l2_frmsizeenum *fsize)
{
	struct mxc_isi_video *video = video_drvdata(file);
	const struct mxc_isi_format_info *fmt;

	if (fsize->index)
		return -EINVAL;

	fmt = mxc_isi_format_by_fourcc(fsize->pixel_format);
	if (!fmt)
		return -EINVAL;

	fsize->type = V4L2_FRMSIZE_TYPE_CONTINUOUS;
	fsize->stepwise.min_width = 4;
	fsize->stepwise.max_width = 4;
	fsize->stepwise.min_height = 4096;
	fsize->stepwise.max_height = 4096;
	fsize->stepwise.step_width = 1;
	fsize->stepwise.step_height = 1;

	if (video->pipe->isi->pdata->model == MXC_ISI_IMX8MP &&
	    video->pipe->id == 1)
		fsize->stepwise.min_height /= 2;

	return 0;
}

static const struct v4l2_ioctl_ops mxc_isi_video_ioctl_ops = {
	.vidioc_querycap		= mxc_isi_video_querycap,

	.vidioc_enum_fmt_vid_cap	= mxc_isi_video_enum_fmt,
	.vidioc_try_fmt_vid_cap_mplane	= mxc_isi_video_try_fmt,
	.vidioc_s_fmt_vid_cap_mplane	= mxc_isi_video_s_fmt,
	.vidioc_g_fmt_vid_cap_mplane	= mxc_isi_video_g_fmt,

	.vidioc_reqbufs			= vb2_ioctl_reqbufs,
	.vidioc_querybuf		= vb2_ioctl_querybuf,
	.vidioc_qbuf			= vb2_ioctl_qbuf,
	.vidioc_dqbuf			= vb2_ioctl_dqbuf,
	.vidioc_expbuf			= vb2_ioctl_expbuf,
	.vidioc_prepare_buf		= vb2_ioctl_prepare_buf,
	.vidioc_create_bufs		= vb2_ioctl_create_bufs,

	.vidioc_streamon		= vb2_ioctl_streamon,
	.vidioc_streamoff		= vb2_ioctl_streamoff,

	.vidioc_enum_framesizes		= mxc_isi_video_enum_framesizes,
};

/* -----------------------------------------------------------------------------
 * Video device file operations
 */

static int mxc_isi_video_open(struct file *file)
{
	struct mxc_isi_video *video = video_drvdata(file);
	int ret;

	ret = v4l2_fh_open(file);
	if (ret)
		return ret;

	ret = pm_runtime_resume_and_get(video->pipe->isi->dev);
	if (ret) {
		v4l2_fh_release(file);
		return ret;
	}

	/* increase usage count for ISI channel */
	mutex_lock(&video->lock);
	atomic_inc(&video->usage_count);
	mutex_unlock(&video->lock);

	return 0;
}

static int mxc_isi_video_release(struct file *file)
{
	struct mxc_isi_video *video = video_drvdata(file);
	int ret;

	ret = vb2_fop_release(file);
	if (ret) {
		dev_err(video->pipe->isi->dev, "%s fail\n", __func__);
		goto done;
	}

	if (atomic_read(&video->usage_count) > 0 &&
	    atomic_dec_and_test(&video->usage_count))
		mxc_isi_channel_deinit(video->pipe);

done:
	pm_runtime_put(video->pipe->isi->dev);
	return ret;
}

static const struct v4l2_file_operations mxc_isi_video_fops = {
	.owner		= THIS_MODULE,
	.open		= mxc_isi_video_open,
	.release	= mxc_isi_video_release,
	.poll		= vb2_fop_poll,
	.unlocked_ioctl	= video_ioctl2,
	.mmap		= vb2_fop_mmap,
};

int mxc_isi_video_register(struct mxc_isi_pipe *pipe,
			   struct v4l2_device *v4l2_dev)
{
	struct mxc_isi_video *video = &pipe->video;
	struct v4l2_pix_format_mplane *pix = &video->pix;
	struct video_device *vdev = &video->vdev;
	struct vb2_queue *q = &video->vb2_q;
	const struct mxc_isi_format_info *fmt;
	int ret = -ENOMEM;

	video->pipe = pipe;

	mutex_init(&video->lock);
	spin_lock_init(&video->buf_lock);

	atomic_set(&video->usage_count, 0);

	pix->width = MXC_ISI_DEF_WIDTH;
	pix->height = MXC_ISI_DEF_HEIGHT;
	pix->pixelformat = MXC_ISI_DEF_PIXEL_FORMAT;
	__mxc_isi_video_try_fmt(pix, &fmt);

	video->fmtinfo = fmt;

	memset(vdev, 0, sizeof(*vdev));
	snprintf(vdev->name, sizeof(vdev->name), "mxc_isi.%d.capture", pipe->id);

	vdev->fops	= &mxc_isi_video_fops;
	vdev->ioctl_ops	= &mxc_isi_video_ioctl_ops;
	vdev->v4l2_dev	= v4l2_dev;
	vdev->minor	= -1;
	vdev->release	= video_device_release_empty;
	vdev->queue	= q;
	vdev->lock	= &video->lock;

	vdev->device_caps = V4L2_CAP_STREAMING | V4L2_CAP_VIDEO_CAPTURE_MPLANE
			  | V4L2_CAP_IO_MC;
	video_set_drvdata(vdev, video);

	INIT_LIST_HEAD(&video->out_pending);
	INIT_LIST_HEAD(&video->out_active);
	INIT_LIST_HEAD(&video->out_discard);

	memset(q, 0, sizeof(*q));
	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	q->io_modes = VB2_MMAP | VB2_DMABUF;
	q->drv_priv = video;
	q->ops = &mxc_isi_vb2_qops;
	q->mem_ops = &vb2_dma_contig_memops;
	q->buf_struct_size = sizeof(struct mxc_isi_buffer);
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->min_buffers_needed = 2;
	q->lock = &video->lock;

	ret = vb2_queue_init(q);
	if (ret)
		goto err_free_ctx;

	video->pad.flags = MEDIA_PAD_FL_SINK;
	vdev->entity.function = MEDIA_ENT_F_PROC_VIDEO_SCALER;
	ret = media_entity_pads_init(&vdev->entity, 1, &video->pad);
	if (ret)
		goto err_free_ctx;

	ret = mxc_isi_video_ctrls_create(video);
	if (ret)
		goto err_me_cleanup;

	ret = video_register_device(vdev, VFL_TYPE_VIDEO, -1);
	if (ret)
		goto err_ctrl_free;

	ret = media_create_pad_link(&pipe->sd.entity,
				    MXC_ISI_PIPE_PAD_SOURCE,
				    &vdev->entity, 0,
				    MEDIA_LNK_FL_IMMUTABLE |
				    MEDIA_LNK_FL_ENABLED);
	if (ret < 0)
		goto err_ctrl_free;

	return 0;

err_ctrl_free:
	mxc_isi_video_ctrls_delete(video);
err_me_cleanup:
	media_entity_cleanup(&vdev->entity);
err_free_ctx:
	return ret;
}

void mxc_isi_video_unregister(struct mxc_isi_pipe *pipe)
{
	struct mxc_isi_video *video = &pipe->video;
	struct video_device *vdev = &video->vdev;

	mutex_lock(&video->lock);

	if (video_is_registered(vdev)) {
		video_unregister_device(vdev);
		mxc_isi_video_ctrls_delete(video);
		media_entity_cleanup(&vdev->entity);
	}

	mutex_unlock(&video->lock);
}
