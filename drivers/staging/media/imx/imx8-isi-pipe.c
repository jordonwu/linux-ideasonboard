// SPDX-License-Identifier: GPL-2.0
/*
 * V4L2 Capture ISI subdev driver for i.MX8QXP/QM platform
 *
 * ISI is a Image Sensor Interface of i.MX8QXP/QM platform, which
 * used to process image from camera sensor to memory or DC
 *
 * Copyright (c) 2019 NXP Semiconductor
 */

#include <linux/bug.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/of_graph.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/videodev2.h>

#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-dma-contig.h>

#include "imx8-isi-hw.h"

#define sd_to_cap_dev(ptr)	container_of(ptr, struct mxc_isi_pipe, sd)

static const struct mxc_isi_format_info mxc_isi_out_formats[] = {
	{
		.fourcc		= V4L2_PIX_FMT_GREY,
		.depth		= { 8 },
		.color		= MXC_ISI_OUT_FMT_RAW8,
		.memplanes	= 1,
		.colorspace	= MXC_ISI_CS_NONE,
		.mbus_code	= MEDIA_BUS_FMT_Y8_1X8,
	}, {
		.fourcc		= V4L2_PIX_FMT_Y10,
		.depth		= { 16 },
		.color		= MXC_ISI_OUT_FMT_RAW10,
		.memplanes	= 1,
		.colorspace	= MXC_ISI_CS_NONE,
		.mbus_code	= MEDIA_BUS_FMT_Y10_1X10,
	}, {
		.fourcc		= V4L2_PIX_FMT_Y12,
		.depth		= { 16 },
		.color		= MXC_ISI_OUT_FMT_RAW12,
		.memplanes	= 1,
		.colorspace	= MXC_ISI_CS_NONE,
		.mbus_code	= MEDIA_BUS_FMT_Y12_1X12,
	}, {
		.fourcc		= V4L2_PIX_FMT_SBGGR8,
		.depth		= { 8 },
		.color		= MXC_ISI_OUT_FMT_RAW8,
		.memplanes	= 1,
		.colorspace	= MXC_ISI_CS_NONE,
		.mbus_code	= MEDIA_BUS_FMT_SBGGR8_1X8,
	}, {
		.fourcc		= V4L2_PIX_FMT_SGBRG8,
		.depth		= { 8 },
		.color		= MXC_ISI_OUT_FMT_RAW8,
		.memplanes	= 1,
		.colorspace	= MXC_ISI_CS_NONE,
		.mbus_code	= MEDIA_BUS_FMT_SGBRG8_1X8,
	}, {
		.fourcc		= V4L2_PIX_FMT_SGRBG8,
		.depth		= { 8 },
		.color		= MXC_ISI_OUT_FMT_RAW8,
		.memplanes	= 1,
		.colorspace	= MXC_ISI_CS_NONE,
		.mbus_code	= MEDIA_BUS_FMT_SGRBG8_1X8,
	}, {
		.fourcc		= V4L2_PIX_FMT_SRGGB8,
		.depth		= { 8 },
		.color		= MXC_ISI_OUT_FMT_RAW8,
		.memplanes	= 1,
		.colorspace	= MXC_ISI_CS_NONE,
		.mbus_code	= MEDIA_BUS_FMT_SRGGB8_1X8,
	}, {
		.fourcc		= V4L2_PIX_FMT_SBGGR10,
		.depth		= { 16 },
		.color		= MXC_ISI_OUT_FMT_RAW10,
		.memplanes	= 1,
		.colorspace	= MXC_ISI_CS_NONE,
		.mbus_code	= MEDIA_BUS_FMT_SBGGR10_1X10,
	}, {
		.fourcc		= V4L2_PIX_FMT_SGBRG10,
		.depth		= { 16 },
		.color		= MXC_ISI_OUT_FMT_RAW10,
		.memplanes	= 1,
		.colorspace	= MXC_ISI_CS_NONE,
		.mbus_code	= MEDIA_BUS_FMT_SGBRG10_1X10,
	}, {
		.fourcc		= V4L2_PIX_FMT_SGRBG10,
		.depth		= { 16 },
		.color		= MXC_ISI_OUT_FMT_RAW10,
		.memplanes	= 1,
		.colorspace	= MXC_ISI_CS_NONE,
		.mbus_code	= MEDIA_BUS_FMT_SGRBG10_1X10,
	}, {
		.fourcc		= V4L2_PIX_FMT_SRGGB10,
		.depth		= { 16 },
		.color		= MXC_ISI_OUT_FMT_RAW10,
		.memplanes	= 1,
		.colorspace	= MXC_ISI_CS_NONE,
		.mbus_code	= MEDIA_BUS_FMT_SRGGB10_1X10,
	}, {
		.fourcc		= V4L2_PIX_FMT_SBGGR12,
		.depth		= { 16 },
		.color		= MXC_ISI_OUT_FMT_RAW12,
		.memplanes	= 1,
		.colorspace	= MXC_ISI_CS_NONE,
		.mbus_code	= MEDIA_BUS_FMT_SBGGR12_1X12,
	}, {
		.fourcc		= V4L2_PIX_FMT_SGBRG12,
		.depth		= { 16 },
		.color		= MXC_ISI_OUT_FMT_RAW12,
		.memplanes	= 1,
		.colorspace	= MXC_ISI_CS_NONE,
		.mbus_code	= MEDIA_BUS_FMT_SGBRG12_1X12,
	}, {
		.fourcc		= V4L2_PIX_FMT_SGRBG12,
		.depth		= { 16 },
		.color		= MXC_ISI_OUT_FMT_RAW12,
		.memplanes	= 1,
		.colorspace	= MXC_ISI_CS_NONE,
		.mbus_code	= MEDIA_BUS_FMT_SGRBG12_1X12,
	}, {
		.fourcc		= V4L2_PIX_FMT_SRGGB12,
		.depth		= { 16 },
		.color		= MXC_ISI_OUT_FMT_RAW12,
		.memplanes	= 1,
		.colorspace	= MXC_ISI_CS_NONE,
		.mbus_code	= MEDIA_BUS_FMT_SRGGB12_1X12,
	}, {
		.fourcc		= V4L2_PIX_FMT_RGB565,
		.depth		= { 16 },
		.color		= MXC_ISI_OUT_FMT_RGB565,
		.memplanes	= 1,
		.colorspace	= MXC_ISI_CS_RGB,
		.mbus_code	= MEDIA_BUS_FMT_RGB565_1X16,
	}, {
		.fourcc		= V4L2_PIX_FMT_RGB24,
		.depth		= { 24 },
		.color		= MXC_ISI_OUT_FMT_BGR32P,
		.memplanes	= 1,
		.colorspace	= MXC_ISI_CS_RGB,
		.mbus_code	= MEDIA_BUS_FMT_RGB888_1X24,
	}, {
		.fourcc		= V4L2_PIX_FMT_BGR24,
		.depth		= { 24 },
		.color		= MXC_ISI_OUT_FMT_RGB32P,
		.memplanes	= 1,
		.colorspace	= MXC_ISI_CS_RGB,
		.mbus_code	= MEDIA_BUS_FMT_BGR888_1X24,
	}, {
		.fourcc		= V4L2_PIX_FMT_YUYV,
		.depth		= { 16 },
		.color		= MXC_ISI_OUT_FMT_YUV422_1P8P,
		.memplanes	= 1,
		.colorspace	= MXC_ISI_CS_YUV,
		.mbus_code	= MEDIA_BUS_FMT_YUYV8_1X16,
	}, {
		.fourcc		= V4L2_PIX_FMT_YUV32,
		.depth		= { 32 },
		.color		= MXC_ISI_OUT_FMT_YUV444_1P8,
		.memplanes	= 1,
		.colorspace	= MXC_ISI_CS_YUV,
		.mbus_code	= MEDIA_BUS_FMT_AYUV8_1X32,
	}, {
		.fourcc		= V4L2_PIX_FMT_NV12,
		.depth		= { 8, 8 },
		.color		= MXC_ISI_OUT_FMT_YUV420_2P8P,
		.memplanes	= 2,
		.colorspace	= MXC_ISI_CS_YUV,
		.mbus_code	= MEDIA_BUS_FMT_YUYV8_1X16,
	}, {
		.fourcc		= V4L2_PIX_FMT_YUV444M,
		.depth		= { 8, 8, 8 },
		.color		= MXC_ISI_OUT_FMT_YUV444_3P8P,
		.memplanes	= 3,
		.colorspace	= MXC_ISI_CS_YUV,
		.mbus_code	= MEDIA_BUS_FMT_YUV8_1X24,
	}, {
		.fourcc		= V4L2_PIX_FMT_XBGR32,
		.depth		= { 32 },
		.color		= MXC_ISI_OUT_FMT_XRGB32,
		.memplanes	= 1,
		.colorspace	= MXC_ISI_CS_RGB,
		.mbus_code	= MEDIA_BUS_FMT_RGB888_1X24,
	}, {
		.fourcc		= V4L2_PIX_FMT_ABGR32,
		.depth		= { 32 },
		.color		= MXC_ISI_OUT_FMT_ARGB32,
		.memplanes	= 1,
		.colorspace	= MXC_ISI_CS_RGB,
		.mbus_code	= MEDIA_BUS_FMT_RGB888_1X24,
	}
};

static const struct mxc_isi_format_info *mxc_isi_format_by_code(u32 code)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(mxc_isi_out_formats); i++) {
		const struct mxc_isi_format_info *fmt = &mxc_isi_out_formats[i];

		if (fmt->mbus_code == code)
			return fmt;
	}

	return NULL;
}

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

static struct v4l2_subdev *mxc_get_source_subdev(struct v4l2_subdev *subdev,
						 u32 *pad,
						 const char * const label)
{
	struct media_pad *source_pad = NULL;
	struct v4l2_subdev *sen_sd;
	unsigned int i;

	/* Get remote source pad */
	for (i = 0; i < subdev->entity.num_pads; i++) {
		struct media_pad *pad = &subdev->entity.pads[i];

		if (!(pad->flags & MEDIA_PAD_FL_SINK))
			continue;

		pad = media_entity_remote_pad(pad);
		if (pad) {
			source_pad = pad;
			break;
		}
	}

	if (!source_pad) {
		v4l2_err(subdev, "%s, No remote pad found!\n", label);
		return NULL;
	}

	/* Get remote source pad subdev */
	sen_sd = media_entity_to_v4l2_subdev(source_pad->entity);
	if (!sen_sd) {
		v4l2_err(subdev, "%s, No remote subdev found!\n", label);
		return NULL;
	}

	if (pad)
		*pad = source_pad->index;

	return sen_sd;
}

/*
 * mxc_isi_pipeline_enable() - Enable streaming on a pipeline
 */
static int mxc_isi_pipeline_enable(struct mxc_isi_pipe *pipe, bool enable)
{
	struct device *dev = pipe->isi->dev;
	struct v4l2_subdev *src_sd;
	int ret;

	src_sd = mxc_get_source_subdev(&pipe->sd, NULL, __func__);
	if (!src_sd)
		return -EPIPE;

	ret = v4l2_subdev_call(src_sd, video, s_stream, enable);
	if (ret < 0 && ret != -ENOIOCTLCMD) {
		dev_err(dev, "subdev %s s_stream failed\n", src_sd->name);
		return ret;
	}

	return 0;
}

static void mxc_isi_cap_frame_write_done(struct mxc_isi_dev *isi)
{
	struct mxc_isi_pipe *pipe = &isi->pipe;
	struct device *dev = pipe->isi->dev;
	struct mxc_isi_buffer *buf;
	struct vb2_buffer *vb2;

	if (list_empty(&pipe->video.out_active)) {
		dev_warn(dev, "trying to access empty active list\n");
		return;
	}

	buf = list_first_entry(&pipe->video.out_active, struct mxc_isi_buffer, list);

	/*
	 * Skip frame when buffer number is not match ISI trigger
	 * buffer
	 */
	if ((is_buf_active(isi, 1) && buf->id == MXC_ISI_BUF1) ||
	    (is_buf_active(isi, 2) && buf->id == MXC_ISI_BUF2)) {
		dev_dbg(dev, "status=0x%x id=%d\n", isi->status, buf->id);
		return;
	}

	if (buf->discard) {
		list_move_tail(pipe->video.out_active.next, &pipe->video.out_discard);
	} else {
		vb2 = &buf->v4l2_buf.vb2_buf;
		list_del_init(&buf->list);
		buf->v4l2_buf.vb2_buf.timestamp = ktime_get_ns();
		vb2_buffer_done(&buf->v4l2_buf.vb2_buf, VB2_BUF_STATE_DONE);
	}

	pipe->video.frame_count++;

	if (list_empty(&pipe->video.out_pending)) {
		if (list_empty(&pipe->video.out_discard)) {
			dev_warn(dev, "trying to access empty discard list\n");
			return;
		}

		buf = list_first_entry(&pipe->video.out_discard,
				       struct mxc_isi_buffer, list);
		buf->v4l2_buf.sequence = pipe->video.frame_count;
		mxc_isi_channel_set_outbuf(isi, buf);
		list_move_tail(pipe->video.out_discard.next, &pipe->video.out_active);
		return;
	}

	/* ISI channel output buffer */
	buf = list_first_entry(&pipe->video.out_pending, struct mxc_isi_buffer, list);
	buf->v4l2_buf.sequence = pipe->video.frame_count;
	mxc_isi_channel_set_outbuf(isi, buf);
	vb2 = &buf->v4l2_buf.vb2_buf;
	vb2->state = VB2_BUF_STATE_ACTIVE;
	list_move_tail(pipe->video.out_pending.next, &pipe->video.out_active);
}

/* -----------------------------------------------------------------------------
 * videobuf2 queue operations
 */

static inline struct mxc_isi_buffer *to_isi_buffer(struct vb2_v4l2_buffer *v4l2_buf)
{
	return container_of(v4l2_buf, struct mxc_isi_buffer, v4l2_buf);
}

static int cap_vb2_queue_setup(struct vb2_queue *q,
			       unsigned int *num_buffers,
			       unsigned int *num_planes,
			       unsigned int sizes[],
			       struct device *alloc_devs[])
{
	struct mxc_isi_pipe *pipe = vb2_get_drv_priv(q);
	struct mxc_isi_frame *dst_f = &pipe->formats[MXC_ISI_SD_PAD_SOURCE];
	const struct mxc_isi_format_info *fmt = dst_f->info;
	unsigned long wh;
	int i;

	if (!fmt)
		return -EINVAL;

	for (i = 0; i < fmt->memplanes; i++)
		alloc_devs[i] = pipe->isi->dev;

	wh = dst_f->format.width * dst_f->format.height;

	*num_planes = fmt->memplanes;

	for (i = 0; i < fmt->memplanes; i++) {
		unsigned int size = (wh * fmt->depth[i]) / 8;

		if (i == 1 && fmt->fourcc == V4L2_PIX_FMT_NV12)
			size >>= 1;
		sizes[i] = max_t(u32, size, pipe->video.pix.plane_fmt[i].sizeimage);
	}
	dev_dbg(pipe->isi->dev, "%s, buf_n=%d, size=%d\n",
		__func__, *num_buffers, sizes[0]);

	return 0;
}

static int cap_vb2_buffer_prepare(struct vb2_buffer *vb2)
{
	struct vb2_queue *q = vb2->vb2_queue;
	struct mxc_isi_pipe *pipe = vb2_get_drv_priv(q);
	struct mxc_isi_frame *dst_f = &pipe->formats[MXC_ISI_SD_PAD_SOURCE];
	int i;

	if (!dst_f->info)
		return -EINVAL;

	for (i = 0; i < dst_f->info->memplanes; i++) {
		unsigned long size = pipe->video.pix.plane_fmt[i].sizeimage;

		if (vb2_plane_size(vb2, i) < size) {
			v4l2_err(&pipe->video.vdev,
				 "User buffer too small (%ld < %ld)\n",
				 vb2_plane_size(vb2, i), size);
			return -EINVAL;
		}

		vb2_set_plane_payload(vb2, i, size);
	}

	return 0;
}

static void cap_vb2_buffer_queue(struct vb2_buffer *vb2)
{
	struct vb2_v4l2_buffer *v4l2_buf = to_vb2_v4l2_buffer(vb2);
	struct mxc_isi_buffer *buf = to_isi_buffer(v4l2_buf);
	struct mxc_isi_pipe *pipe = vb2_get_drv_priv(vb2->vb2_queue);
	unsigned long flags;

	spin_lock_irqsave(&pipe->slock, flags);
	list_add_tail(&buf->list, &pipe->video.out_pending);
	spin_unlock_irqrestore(&pipe->slock, flags);
}

static int cap_vb2_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct mxc_isi_pipe *pipe = vb2_get_drv_priv(q);
	const struct mxc_isi_format_info *info;
	struct mxc_isi_dev *isi = pipe->isi;
	struct mxc_isi_buffer *buf;
	struct mxc_isi_frame *fmt;
	struct vb2_buffer *vb2;
	unsigned long flags;
	int i, j;
	int ret;

	ret = media_pipeline_start(&pipe->video.vdev.entity, &pipe->pipe);
	if (ret < 0)
		return ret;

	/*
	 * Verify that the configured format matches the output of the
	 * subdev.
	 */
	fmt = &pipe->formats[MXC_ISI_SD_PAD_SOURCE];
	info = mxc_isi_format_by_fourcc(pipe->video.pix.pixelformat);

        if (fmt->format.code != info->mbus_code ||
	    fmt->format.width != pipe->video.pix.width ||
            fmt->format.height != pipe->video.pix.height) {
		dev_dbg(pipe->isi->dev,
			"%s: configuration mismatch, 0x%04x/%ux%u != 0x%04x/%ux%u\n",
			__func__, fmt->format.code, fmt->format.width,
			fmt->format.height, info->mbus_code,
			pipe->video.pix.width, pipe->video.pix.height);
                ret = -EINVAL;
		goto error;
	}

	mxc_isi_channel_init(isi);
	mxc_isi_channel_config(isi, &pipe->formats[MXC_ISI_SD_PAD_SINK],
			       &pipe->formats[MXC_ISI_SD_PAD_SOURCE],
			       pipe->video.pix.plane_fmt[0].bytesperline);

	/* Create a buffer for discard operation */
	for (i = 0; i < pipe->video.pix.num_planes; i++) {
		pipe->video.discard_size[i] = pipe->video.pix.plane_fmt[i].sizeimage;
		pipe->video.discard_buffer[i] =
			dma_alloc_coherent(pipe->isi->dev,
					   PAGE_ALIGN(pipe->video.discard_size[i]),
					   &pipe->video.discard_buffer_dma[i],
					   GFP_DMA | GFP_KERNEL);
		if (!pipe->video.discard_buffer[i]) {
			for (j = 0; j < i; j++) {
				dma_free_coherent(pipe->isi->dev,
						  PAGE_ALIGN(pipe->video.discard_size[j]),
						  pipe->video.discard_buffer[j],
						  pipe->video.discard_buffer_dma[j]);
				dev_err(pipe->isi->dev,
					"alloc dma buffer(%d) fail\n", j);
			}

			ret = -ENOMEM;
			goto error;
		}
		dev_dbg(pipe->isi->dev,
			"%s: num_plane=%d discard_size=%d discard_buffer=%p\n"
			, __func__, i,
			PAGE_ALIGN((int)pipe->video.discard_size[i]),
			pipe->video.discard_buffer[i]);
	}

	spin_lock_irqsave(&pipe->slock, flags);

	/* add two list member to out_discard list head */
	pipe->video.buf_discard[0].discard = true;
	list_add_tail(&pipe->video.buf_discard[0].list, &pipe->video.out_discard);

	pipe->video.buf_discard[1].discard = true;
	list_add_tail(&pipe->video.buf_discard[1].list, &pipe->video.out_discard);

	/* ISI channel output buffer 1 */
	buf = list_first_entry(&pipe->video.out_discard, struct mxc_isi_buffer, list);
	buf->v4l2_buf.sequence = 0;
	vb2 = &buf->v4l2_buf.vb2_buf;
	vb2->state = VB2_BUF_STATE_ACTIVE;
	mxc_isi_channel_set_outbuf(isi, buf);
	list_move_tail(pipe->video.out_discard.next, &pipe->video.out_active);

	/* ISI channel output buffer 2 */
	buf = list_first_entry(&pipe->video.out_pending, struct mxc_isi_buffer, list);
	buf->v4l2_buf.sequence = 1;
	vb2 = &buf->v4l2_buf.vb2_buf;
	vb2->state = VB2_BUF_STATE_ACTIVE;
	mxc_isi_channel_set_outbuf(isi, buf);
	list_move_tail(pipe->video.out_pending.next, &pipe->video.out_active);

	/* Clear frame count */
	pipe->video.frame_count = 1;
	spin_unlock_irqrestore(&pipe->slock, flags);

	mxc_isi_channel_enable(isi, false);
	ret = mxc_isi_pipeline_enable(pipe, 1);
	if (ret < 0 && ret != -ENOIOCTLCMD)
		goto error;

	isi->is_streaming = 1;

	return 0;

error:
	/* FIXME: Free discard buffer, return vb2 buffers to vb2 */
	media_pipeline_stop(&pipe->video.vdev.entity);
	return ret;
}

static void cap_vb2_stop_streaming(struct vb2_queue *q)
{
	struct mxc_isi_pipe *pipe = vb2_get_drv_priv(q);
	struct mxc_isi_dev *isi = pipe->isi;
	struct mxc_isi_buffer *buf;
	unsigned long flags;
	int i;

	mxc_isi_pipeline_enable(pipe, 0);
	mxc_isi_channel_disable(isi);

	spin_lock_irqsave(&pipe->slock, flags);

	while (!list_empty(&pipe->video.out_active)) {
		buf = list_entry(pipe->video.out_active.next,
				 struct mxc_isi_buffer, list);
		list_del_init(&buf->list);
		if (buf->discard)
			continue;

		vb2_buffer_done(&buf->v4l2_buf.vb2_buf, VB2_BUF_STATE_ERROR);
	}

	while (!list_empty(&pipe->video.out_pending)) {
		buf = list_entry(pipe->video.out_pending.next,
				 struct mxc_isi_buffer, list);
		list_del_init(&buf->list);
		vb2_buffer_done(&buf->v4l2_buf.vb2_buf, VB2_BUF_STATE_ERROR);
	}

	while (!list_empty(&pipe->video.out_discard)) {
		buf = list_entry(pipe->video.out_discard.next,
				 struct mxc_isi_buffer, list);
		list_del_init(&buf->list);
	}

	INIT_LIST_HEAD(&pipe->video.out_active);
	INIT_LIST_HEAD(&pipe->video.out_pending);
	INIT_LIST_HEAD(&pipe->video.out_discard);

	spin_unlock_irqrestore(&pipe->slock, flags);

	for (i = 0; i < pipe->video.pix.num_planes; i++)
		dma_free_coherent(pipe->isi->dev,
				  PAGE_ALIGN(pipe->video.discard_size[i]),
				  pipe->video.discard_buffer[i],
				  pipe->video.discard_buffer_dma[i]);

	media_pipeline_stop(&pipe->video.vdev.entity);

	isi->is_streaming = 0;
}

static const struct vb2_ops mxc_cap_vb2_qops = {
	.queue_setup		= cap_vb2_queue_setup,
	.buf_prepare		= cap_vb2_buffer_prepare,
	.buf_queue		= cap_vb2_buffer_queue,
	.wait_prepare		= vb2_ops_wait_prepare,
	.wait_finish		= vb2_ops_wait_finish,
	.start_streaming	= cap_vb2_start_streaming,
	.stop_streaming		= cap_vb2_stop_streaming,
};

/* -----------------------------------------------------------------------------
 * V4L2 controls
 */

static inline struct mxc_isi_pipe *ctrl_to_isi_cap(struct v4l2_ctrl *ctrl)
{
	return container_of(ctrl->handler, struct mxc_isi_pipe,
			    video.ctrls.handler);
}

static int mxc_isi_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct mxc_isi_pipe *pipe = ctrl_to_isi_cap(ctrl);
	struct mxc_isi_dev *isi = pipe->isi;
	unsigned long flags;

	if (ctrl->flags & V4L2_CTRL_FLAG_INACTIVE)
		return 0;

	spin_lock_irqsave(&isi->slock, flags);

	switch (ctrl->id) {
	case V4L2_CID_ALPHA_COMPONENT:
		if (ctrl->val < 0 || ctrl->val > 255)
			return -EINVAL;
		isi->alpha = ctrl->val;
		isi->alphaen = 1;
		break;

	default:
		dev_err(pipe->isi->dev,
			"%s: Not support %d CID\n", __func__, ctrl->id);
		return -EINVAL;
	}

	spin_unlock_irqrestore(&isi->slock, flags);
	return 0;
}

static const struct v4l2_ctrl_ops mxc_isi_ctrl_ops = {
	.s_ctrl = mxc_isi_s_ctrl,
};

static int mxc_isi_ctrls_create(struct mxc_isi_pipe *pipe)
{
	struct mxc_isi_ctrls *ctrls = &pipe->video.ctrls;
	struct v4l2_ctrl_handler *handler = &ctrls->handler;

	if (pipe->video.ctrls.ready)
		return 0;

	v4l2_ctrl_handler_init(handler, 1);

	ctrls->alpha = v4l2_ctrl_new_std(handler, &mxc_isi_ctrl_ops,
					 V4L2_CID_ALPHA_COMPONENT,
					 0, 0xff, 1, 0);

	if (!handler->error)
		ctrls->ready = true;

	return handler->error;
}

static void mxc_isi_ctrls_delete(struct mxc_isi_pipe *pipe)
{
	struct mxc_isi_ctrls *ctrls = &pipe->video.ctrls;

	if (ctrls->ready) {
		v4l2_ctrl_handler_free(&ctrls->handler);
		ctrls->ready = false;
		ctrls->alpha = NULL;
	}
}

/* -----------------------------------------------------------------------------
 * V4L2 ioctls
 */

static int mxc_isi_cap_querycap(struct file *file, void *priv,
				struct v4l2_capability *cap)
{
	struct mxc_isi_pipe *pipe = video_drvdata(file);

	strlcpy(cap->driver, MXC_ISI_CAPTURE, sizeof(cap->driver));
	strlcpy(cap->card, MXC_ISI_CAPTURE, sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:%s.%d",
		 dev_name(pipe->isi->dev), pipe->id);

	cap->device_caps = V4L2_CAP_STREAMING | V4L2_CAP_VIDEO_CAPTURE_MPLANE;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;

	return 0;
}

static int mxc_isi_cap_enum_fmt(struct file *file, void *priv,
				struct v4l2_fmtdesc *f)
{
	const struct mxc_isi_format_info *fmt;

	if (f->index >= ARRAY_SIZE(mxc_isi_out_formats))
		return -EINVAL;

	fmt = &mxc_isi_out_formats[f->index];
	f->pixelformat = fmt->fourcc;

	return 0;
}

static int mxc_isi_cap_g_fmt_mplane(struct file *file, void *fh,
				    struct v4l2_format *f)
{
	struct mxc_isi_pipe *pipe = video_drvdata(file);

	f->fmt.pix_mp = pipe->video.pix;

	return 0;
}

static void __mxc_isi_cap_try_fmt_mplane(struct v4l2_pix_format_mplane *pix,
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
				    pix->width * fmt->depth[i] / 8,
				    65535U);
		else
			bpl = pix->plane_fmt[0].bytesperline;

		plane->bytesperline = bpl;

		plane->sizeimage = plane->bytesperline * pix->height;
		if (pix->pixelformat == V4L2_PIX_FMT_NV12 && i == 1)
			plane->sizeimage /= 2;
	}

	pix->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(pix->colorspace);
	pix->quantization =
		V4L2_MAP_QUANTIZATION_DEFAULT(fmt->color == MXC_ISI_CS_RGB,
					      pix->colorspace, pix->ycbcr_enc);
	pix->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(pix->colorspace);

	if (info)
		*info = fmt;
}

static int mxc_isi_cap_try_fmt_mplane(struct file *file, void *fh,
				      struct v4l2_format *f)
{
	__mxc_isi_cap_try_fmt_mplane(&f->fmt.pix_mp, NULL);
	return 0;
}

static int mxc_isi_cap_s_fmt_mplane(struct file *file, void *priv,
				    struct v4l2_format *f)
{
	struct mxc_isi_pipe *pipe = video_drvdata(file);
	struct v4l2_pix_format_mplane *pix = &f->fmt.pix_mp;
	const struct mxc_isi_format_info *fmt;

	if (vb2_is_busy(&pipe->video.vb2_q))
		return -EBUSY;

	__mxc_isi_cap_try_fmt_mplane(pix, &fmt);

	pipe->video.pix = *pix;

	return 0;
}

static int mxc_isi_cap_enum_framesizes(struct file *file, void *priv,
				       struct v4l2_frmsizeenum *fsize)
{
	struct mxc_isi_pipe *pipe = video_drvdata(file);
	const struct mxc_isi_format_info *fmt;

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

	if (pipe->isi->pdata->model == MXC_ISI_IMX8MP && pipe->id == 1)
		fsize->stepwise.min_height /= 2;

	return 0;
}

static const struct v4l2_ioctl_ops mxc_isi_capture_ioctl_ops = {
	.vidioc_querycap		= mxc_isi_cap_querycap,

	.vidioc_enum_fmt_vid_cap	= mxc_isi_cap_enum_fmt,
	.vidioc_try_fmt_vid_cap_mplane	= mxc_isi_cap_try_fmt_mplane,
	.vidioc_s_fmt_vid_cap_mplane	= mxc_isi_cap_s_fmt_mplane,
	.vidioc_g_fmt_vid_cap_mplane	= mxc_isi_cap_g_fmt_mplane,

	.vidioc_reqbufs			= vb2_ioctl_reqbufs,
	.vidioc_querybuf		= vb2_ioctl_querybuf,
	.vidioc_qbuf			= vb2_ioctl_qbuf,
	.vidioc_dqbuf			= vb2_ioctl_dqbuf,
	.vidioc_expbuf			= vb2_ioctl_expbuf,
	.vidioc_prepare_buf		= vb2_ioctl_prepare_buf,
	.vidioc_create_bufs		= vb2_ioctl_create_bufs,

	.vidioc_streamon		= vb2_ioctl_streamon,
	.vidioc_streamoff		= vb2_ioctl_streamoff,

	.vidioc_enum_framesizes		= mxc_isi_cap_enum_framesizes,
};

/* -----------------------------------------------------------------------------
 * Video device file operations
 */

static bool is_entity_link_setup(struct mxc_isi_pipe *pipe)
{
	struct video_device *vdev = &pipe->video.vdev;
	struct v4l2_subdev *csi_sd, *sen_sd;

	if (!vdev->entity.num_links || !pipe->sd.entity.num_links)
		return false;

	csi_sd = mxc_get_source_subdev(&pipe->sd, NULL, __func__);
	if (!csi_sd || !csi_sd->entity.num_links)
		return false;

	sen_sd = mxc_get_source_subdev(csi_sd, NULL, __func__);
	if (!sen_sd || !sen_sd->entity.num_links)
		return false;

	return true;
}

static int mxc_isi_capture_open(struct file *file)
{
	struct mxc_isi_pipe *pipe = video_drvdata(file);
	struct mxc_isi_dev *isi = pipe->isi;
	struct device *dev = pipe->isi->dev;
	int ret = -EBUSY;

	mutex_lock(&pipe->lock);
	pipe->video.is_link_setup = is_entity_link_setup(pipe);
	if (!pipe->video.is_link_setup) {
		mutex_unlock(&pipe->lock);
		return 0;
	}
	mutex_unlock(&pipe->lock);

	if (isi->frame_write_done) {
		dev_err(dev, "ISI channel[%d] is busy\n", pipe->id);
		return ret;
	}

	mutex_lock(&pipe->lock);
	ret = v4l2_fh_open(file);
	if (ret) {
		mutex_unlock(&pipe->lock);
		return ret;
	}
	mutex_unlock(&pipe->lock);

	pm_runtime_get_sync(dev);

	/* increase usage count for ISI channel */
	mutex_lock(&isi->lock);
	atomic_inc(&isi->usage_count);
	isi->frame_write_done = mxc_isi_cap_frame_write_done;
	mutex_unlock(&isi->lock);

	return 0;
}

static int mxc_isi_capture_release(struct file *file)
{
	struct mxc_isi_pipe *pipe = video_drvdata(file);
	struct mxc_isi_dev *isi = pipe->isi;
	struct device *dev = pipe->isi->dev;
	int ret = -1;

	if (!pipe->video.is_link_setup)
		return 0;

	mutex_lock(&pipe->lock);
	ret = _vb2_fop_release(file, NULL);
	if (ret) {
		dev_err(dev, "%s fail\n", __func__);
		mutex_unlock(&pipe->lock);
		goto label;
	}
	mutex_unlock(&pipe->lock);

	if (atomic_read(&isi->usage_count) > 0 &&
	    atomic_dec_and_test(&isi->usage_count))
		mxc_isi_channel_deinit(isi);

	mutex_lock(&isi->lock);
	isi->frame_write_done = NULL;
	mutex_unlock(&isi->lock);

label:
	pm_runtime_put(dev);
	return (ret) ? ret : 0;
}

static const struct v4l2_file_operations mxc_isi_capture_fops = {
	.owner		= THIS_MODULE,
	.open		= mxc_isi_capture_open,
	.release	= mxc_isi_capture_release,
	.poll		= vb2_fop_poll,
	.unlocked_ioctl	= video_ioctl2,
	.mmap		= vb2_fop_mmap,
};

static int mxc_isi_video_register(struct mxc_isi_pipe *pipe,
				  struct v4l2_device *v4l2_dev)
{
	struct v4l2_pix_format_mplane *pix = &pipe->video.pix;
	struct video_device *vdev = &pipe->video.vdev;
	struct vb2_queue *q = &pipe->video.vb2_q;
	int ret = -ENOMEM;

	pix->width = MXC_ISI_DEF_WIDTH;
	pix->height = MXC_ISI_DEF_HEIGHT;
	pix->pixelformat = V4L2_PIX_FMT_YUYV;
	__mxc_isi_cap_try_fmt_mplane(pix, NULL);

	memset(vdev, 0, sizeof(*vdev));
	snprintf(vdev->name, sizeof(vdev->name), "mxc_isi.%d.capture", pipe->id);

	vdev->fops	= &mxc_isi_capture_fops;
	vdev->ioctl_ops	= &mxc_isi_capture_ioctl_ops;
	vdev->v4l2_dev	= v4l2_dev;
	vdev->minor	= -1;
	vdev->release	= video_device_release_empty;
	vdev->queue	= q;
	vdev->lock	= &pipe->lock;

	vdev->device_caps = V4L2_CAP_STREAMING | V4L2_CAP_VIDEO_CAPTURE_MPLANE;
	video_set_drvdata(vdev, pipe);

	INIT_LIST_HEAD(&pipe->video.out_pending);
	INIT_LIST_HEAD(&pipe->video.out_active);
	INIT_LIST_HEAD(&pipe->video.out_discard);

	memset(q, 0, sizeof(*q));
	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	q->io_modes = VB2_MMAP | VB2_USERPTR | VB2_DMABUF;
	q->drv_priv = pipe;
	q->ops = &mxc_cap_vb2_qops;
	q->mem_ops = &vb2_dma_contig_memops;
	q->buf_struct_size = sizeof(struct mxc_isi_buffer);
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->min_buffers_needed = 2;
	q->lock = &pipe->lock;

	ret = vb2_queue_init(q);
	if (ret)
		goto err_free_ctx;

	pipe->video.pad.flags = MEDIA_PAD_FL_SINK;
	vdev->entity.function = MEDIA_ENT_F_PROC_VIDEO_SCALER;
	ret = media_entity_pads_init(&vdev->entity, 1, &pipe->video.pad);
	if (ret)
		goto err_free_ctx;

	ret = mxc_isi_ctrls_create(pipe);
	if (ret)
		goto err_me_cleanup;

	ret = video_register_device(vdev, VFL_TYPE_VIDEO, -1);
	if (ret)
		goto err_ctrl_free;

	ret = media_create_pad_link(&pipe->sd.entity,
				    MXC_ISI_SD_PAD_SOURCE,
				    &vdev->entity, 0,
				    MEDIA_LNK_FL_IMMUTABLE |
				    MEDIA_LNK_FL_ENABLED);
	if (ret < 0)
		goto err_ctrl_free;

	vdev->ctrl_handler = &pipe->video.ctrls.handler;
	v4l2_dev->ctrl_handler = &pipe->video.ctrls.handler;
	v4l2_info(v4l2_dev, "Registered %s as /dev/%s\n",
		  vdev->name, video_device_node_name(vdev));

	return 0;

err_ctrl_free:
	mxc_isi_ctrls_delete(pipe);
err_me_cleanup:
	media_entity_cleanup(&vdev->entity);
err_free_ctx:
	return ret;
}

static void mxc_isi_video_unregister(struct mxc_isi_pipe *pipe)
{
	struct video_device *vdev = &pipe->video.vdev;

	mutex_lock(&pipe->lock);

	if (video_is_registered(vdev)) {
		video_unregister_device(vdev);
		mxc_isi_ctrls_delete(pipe);
		media_entity_cleanup(&vdev->entity);
	}

	mutex_unlock(&pipe->lock);
}

/* -----------------------------------------------------------------------------
 * V4L2 subdev operations
 */

static struct v4l2_mbus_framefmt *
mxc_isi_pipe_get_pad_format(struct mxc_isi_pipe *pipe,
			    struct v4l2_subdev_pad_config *cfg,
			    enum v4l2_subdev_format_whence which,
			    unsigned int pad)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &pipe->formats[pad].format;
	case V4L2_SUBDEV_FORMAT_TRY:
	default:
		return v4l2_subdev_get_try_format(&pipe->sd, cfg, pad);
	}
}

static struct v4l2_rect *
mxc_isi_pipe_get_pad_crop(struct mxc_isi_pipe *pipe,
			  struct v4l2_subdev_pad_config *cfg,
			  enum v4l2_subdev_format_whence which,
			  unsigned int pad)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &pipe->formats[pad].crop;
	case V4L2_SUBDEV_FORMAT_TRY:
	default:
		return v4l2_subdev_get_try_crop(&pipe->sd, cfg, pad);
	}
}

static struct v4l2_rect *
mxc_isi_pipe_get_pad_compose(struct mxc_isi_pipe *pipe,
			     struct v4l2_subdev_pad_config *cfg,
			     enum v4l2_subdev_format_whence which,
			     unsigned int pad)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &pipe->formats[pad].compose;
	case V4L2_SUBDEV_FORMAT_TRY:
	default:
		return v4l2_subdev_get_try_compose(&pipe->sd, cfg, pad);
	}
}

static int mxc_isi_pipe_init_cfg(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg)
{
	enum v4l2_subdev_format_whence which = cfg ? V4L2_SUBDEV_FORMAT_TRY
					     : V4L2_SUBDEV_FORMAT_ACTIVE;
	struct mxc_isi_pipe *pipe = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *fmt_source;
	struct v4l2_mbus_framefmt *fmt_sink;
	struct v4l2_rect *compose;
	struct v4l2_rect *crop;

	fmt_sink = mxc_isi_pipe_get_pad_format(pipe, cfg, which,
					       MXC_ISI_SD_PAD_SINK);
	fmt_source = mxc_isi_pipe_get_pad_format(pipe, cfg, which,
						 MXC_ISI_SD_PAD_SOURCE);

	fmt_sink->width = MXC_ISI_DEF_WIDTH;
	fmt_sink->height = MXC_ISI_DEF_HEIGHT;
	fmt_sink->code = MEDIA_BUS_FMT_YUYV8_1X16;
	fmt_sink->field = V4L2_FIELD_NONE;
	fmt_sink->colorspace = V4L2_COLORSPACE_JPEG;
	fmt_sink->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt_sink->colorspace);
	fmt_sink->quantization =
		V4L2_MAP_QUANTIZATION_DEFAULT(false, fmt_sink->colorspace,
					      fmt_sink->ycbcr_enc);
	fmt_sink->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(fmt_sink->colorspace);

	*fmt_source = *fmt_sink;

	compose = mxc_isi_pipe_get_pad_compose(pipe, cfg, which,
					       MXC_ISI_SD_PAD_SINK);
	crop = mxc_isi_pipe_get_pad_crop(pipe, cfg, which,
					 MXC_ISI_SD_PAD_SOURCE);

	compose->left = 0;
	compose->top = 0;
	compose->width = MXC_ISI_DEF_WIDTH;
	compose->height = MXC_ISI_DEF_HEIGHT;

	*crop = *compose;

	return 0;
}

static int mxc_isi_pipe_enum_mbus_code(struct v4l2_subdev *sd,
				       struct v4l2_subdev_pad_config *cfg,
				       struct v4l2_subdev_mbus_code_enum *code)
{
	return 0;
}

static int mxc_isi_pipe_get_fmt(struct v4l2_subdev *sd,
				struct v4l2_subdev_pad_config *cfg,
				struct v4l2_subdev_format *fmt)
{
	struct mxc_isi_pipe *pipe = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *format;

	format = mxc_isi_pipe_get_pad_format(pipe, cfg, fmt->which, fmt->pad);

	mutex_lock(&pipe->lock);
	fmt->format = *format;
	mutex_unlock(&pipe->lock);

	return 0;
}

static int mxc_isi_pipe_set_fmt(struct v4l2_subdev *sd,
				struct v4l2_subdev_pad_config *cfg,
				struct v4l2_subdev_format *fmt)
{
	struct mxc_isi_pipe *pipe = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *mf = &fmt->format;
	const struct mxc_isi_format_info *info;
	struct v4l2_mbus_framefmt *format;
	struct v4l2_rect *rect;

	if (vb2_is_busy(&pipe->video.vb2_q))
		return -EBUSY;

	info = mxc_isi_format_by_code(mf->code);
	if (!info)
		info = mxc_isi_format_by_code(MEDIA_BUS_FMT_YUYV8_1X16);

	mutex_lock(&pipe->lock);

	if (fmt->pad == MXC_ISI_SD_PAD_SINK) {
		unsigned int max_width;

		/*
		 * FIXME: This needs to handled more dynamically, larger line
		 * lengths are possible when bypassing the scaler.
		 */
		max_width = pipe->isi->pdata->model == MXC_ISI_IMX8MN
			  ? 2048 : 4096;

		mf->width = clamp(mf->width, MXC_ISI_MIN_WIDTH, max_width);
		mf->height = clamp(mf->height, MXC_ISI_MIN_HEIGHT,
				   MXC_ISI_MAX_HEIGHT);

		/* Propagate the format to the source pad. */
		rect = mxc_isi_pipe_get_pad_compose(pipe, cfg, fmt->which,
						    MXC_ISI_SD_PAD_SINK);
		rect->width = mf->width;
		rect->height = mf->height;

		rect = mxc_isi_pipe_get_pad_crop(pipe, cfg, fmt->which,
						 MXC_ISI_SD_PAD_SOURCE);
		rect->left = 0;
		rect->top = 0;
		rect->width = mf->width;
		rect->height = mf->height;

		format = mxc_isi_pipe_get_pad_format(pipe, cfg, fmt->which,
						     MXC_ISI_SD_PAD_SOURCE);
		format->width = mf->width;
		format->height = mf->height;
	} else {
		/*
		 * The width and height on the source can't be changed, they
		 * must match the crop rectangle size.
		 */
		rect = mxc_isi_pipe_get_pad_crop(pipe, cfg, fmt->which,
						 MXC_ISI_SD_PAD_SOURCE);

		mf->width = rect->width;
		mf->height = rect->height;
	}

	format = mxc_isi_pipe_get_pad_format(pipe, cfg, fmt->which, fmt->pad);
	*format = *mf;

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE)
		pipe->formats[fmt->pad].info = info;

	mutex_unlock(&pipe->lock);

	dev_dbg(pipe->isi->dev, "pad%d: code: 0x%x, %dx%d",
		fmt->pad, mf->code, mf->width, mf->height);

	return 0;
}

static int mxc_isi_pipe_get_selection(struct v4l2_subdev *sd,
				      struct v4l2_subdev_pad_config *cfg,
				      struct v4l2_subdev_selection *sel)
{
	struct mxc_isi_pipe *pipe = v4l2_get_subdevdata(sd);
	const struct v4l2_mbus_framefmt *format;
	const struct v4l2_rect *rect;
	int ret = 0;

	mutex_lock(&pipe->lock);

	switch (sel->target) {
	case V4L2_SEL_TGT_COMPOSE_BOUNDS:
		if (sel->pad != MXC_ISI_SD_PAD_SINK) {
			/* No compose rectangle on source pad. */
			ret = -EINVAL;
			break;
		}

		/* The sink compose is bound by the sink format. */
		format = mxc_isi_pipe_get_pad_format(pipe, cfg, sel->which,
						     MXC_ISI_SD_PAD_SINK);
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = format->width;
		sel->r.height = format->height;
		break;

	case V4L2_SEL_TGT_CROP_BOUNDS:
		if (sel->pad != MXC_ISI_SD_PAD_SOURCE) {
			/* No crop rectangle on sink pad. */
			ret = -EINVAL;
			break;
		}

		/* The source crop is bound by the sink compose. */
		rect = mxc_isi_pipe_get_pad_compose(pipe, cfg, sel->which,
						    MXC_ISI_SD_PAD_SINK);
		sel->r = *rect;
		break;

	case V4L2_SEL_TGT_CROP:
		if (sel->pad != MXC_ISI_SD_PAD_SOURCE) {
			/* No crop rectangle on sink pad. */
			ret = -EINVAL;
			break;
		}

		rect = mxc_isi_pipe_get_pad_crop(pipe, cfg, sel->which,
						 sel->pad);
		sel->r = *rect;
		break;

	case V4L2_SEL_TGT_COMPOSE:
		if (sel->pad != MXC_ISI_SD_PAD_SINK) {
			/* No compose rectangle on source pad. */
			ret = -EINVAL;
			break;
		}

		rect = mxc_isi_pipe_get_pad_compose(pipe, cfg, sel->which,
						    sel->pad);
		sel->r = *rect;
		break;

	default:
		ret = -EINVAL;
		break;
	}

	mutex_unlock(&pipe->lock);
	return ret;
}

static int mxc_isi_pipe_set_selection(struct v4l2_subdev *sd,
				      struct v4l2_subdev_pad_config *cfg,
				      struct v4l2_subdev_selection *sel)
{
	struct mxc_isi_pipe *pipe = v4l2_get_subdevdata(sd);
	struct mxc_isi_frame *f = &pipe->formats[MXC_ISI_SD_PAD_SINK];
	struct v4l2_mbus_framefmt *format;
	struct v4l2_rect *rect;
	unsigned long flags;
	int ret = 0;

	mutex_lock(&pipe->lock);

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
		if (sel->pad != MXC_ISI_SD_PAD_SOURCE) {
			/* The pipeline support cropping on the source only. */
			ret = -EINVAL;
			break;
		}

		/* The source crop is bound by the sink compose. */
		rect = mxc_isi_pipe_get_pad_compose(pipe, cfg, sel->which,
						    MXC_ISI_SD_PAD_SINK);
		sel->r.left = clamp_t(s32, sel->r.left, 0, rect->width - 1);
		sel->r.top = clamp_t(s32, sel->r.top, 0, rect->height - 1);
		sel->r.width = clamp(sel->r.width, MXC_ISI_MIN_WIDTH,
				     rect->width - sel->r.left);
		sel->r.height = clamp(sel->r.height, MXC_ISI_MIN_HEIGHT,
				      rect->height - sel->r.top);

		rect = mxc_isi_pipe_get_pad_crop(pipe, cfg, sel->which,
						 MXC_ISI_SD_PAD_SOURCE);
		*rect = sel->r;

		/* Propagate the crop rectangle to the source pad. */
		format = mxc_isi_pipe_get_pad_format(pipe, cfg, sel->which,
						     MXC_ISI_SD_PAD_SOURCE);
		format->width = sel->r.width;
		format->height = sel->r.height;
		break;

	case V4L2_SEL_TGT_COMPOSE:
		if (sel->pad != MXC_ISI_SD_PAD_SINK) {
			/* Composing is supported on the sink only. */
			ret = -EINVAL;
			break;
		}

		f = &pipe->formats[MXC_ISI_SD_PAD_SOURCE];

		/* The sink crop is bound by the sink format downscaling only). */
		format = mxc_isi_pipe_get_pad_format(pipe, cfg, sel->which,
						     MXC_ISI_SD_PAD_SINK);

		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = clamp(sel->r.width, MXC_ISI_MIN_WIDTH,
				     format->width);
		sel->r.height = clamp(sel->r.height, MXC_ISI_MIN_HEIGHT,
				      format->height);

		rect = mxc_isi_pipe_get_pad_compose(pipe, cfg, sel->which,
						    MXC_ISI_SD_PAD_SINK);
		*rect = sel->r;

		/* Propagate the compose rectangle to the source pad. */
		rect = mxc_isi_pipe_get_pad_crop(pipe, cfg, sel->which,
						 MXC_ISI_SD_PAD_SOURCE);
		rect->left = 0;
		rect->top = 0;
		rect->width = sel->r.width;
		rect->height = sel->r.height;

		format = mxc_isi_pipe_get_pad_format(pipe, cfg, sel->which,
						     MXC_ISI_SD_PAD_SOURCE);
		format->width = sel->r.width;
		format->height = sel->r.height;
		break;

	default:
		ret = -EINVAL;
		break;
	}

	mutex_unlock(&pipe->lock);

	if (ret < 0)
		return ret;

	if (sel->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		spin_lock_irqsave(&pipe->slock, flags);
		/*
		 * FIXME: Support moving the crop rectangle when the pipeline
		 * is streaming.
		 */
		spin_unlock_irqrestore(&pipe->slock, flags);
	}

	dev_dbg(pipe->isi->dev, "%s, target %#x: (%d,%d)/%dx%d", __func__,
		sel->target, sel->r.left, sel->r.top, sel->r.width,
		sel->r.height);

	return 0;
}

static const struct v4l2_subdev_pad_ops mxc_isi_subdev_pad_ops = {
	.init_cfg = mxc_isi_pipe_init_cfg,
	.enum_mbus_code = mxc_isi_pipe_enum_mbus_code,
	.get_fmt = mxc_isi_pipe_get_fmt,
	.set_fmt = mxc_isi_pipe_set_fmt,
	.get_selection = mxc_isi_pipe_get_selection,
	.set_selection = mxc_isi_pipe_set_selection,
};

static const struct v4l2_subdev_ops mxc_isi_subdev_ops = {
	.pad = &mxc_isi_subdev_pad_ops,
};

static int mxc_isi_subdev_registered(struct v4l2_subdev *sd)
{
	struct mxc_isi_pipe *pipe = sd_to_cap_dev(sd);
	int ret;

	if (!pipe)
		return -ENXIO;

	ret = mxc_isi_video_register(pipe, sd->v4l2_dev);
	if (ret < 0)
		return ret;

	return 0;
}

static void mxc_isi_subdev_unregistered(struct v4l2_subdev *sd)
{
	struct mxc_isi_pipe *pipe = v4l2_get_subdevdata(sd);

	if (!pipe)
		return;

	mxc_isi_video_unregister(pipe);
}

static const struct v4l2_subdev_internal_ops mxc_isi_capture_sd_internal_ops = {
	.registered = mxc_isi_subdev_registered,
	.unregistered = mxc_isi_subdev_unregistered,
};

/* -----------------------------------------------------------------------------
 * Init & cleanup
 */

int mxc_isi_pipe_init(struct mxc_isi_dev *isi)
{
	struct mxc_isi_pipe *pipe = &isi->pipe;
	struct v4l2_subdev *sd;
	unsigned int i;
	int ret;

	pipe->id = isi->id;
	pipe->isi = isi;

	spin_lock_init(&pipe->slock);
	mutex_init(&pipe->lock);

	sd = &pipe->sd;
	v4l2_subdev_init(sd, &mxc_isi_subdev_ops);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	snprintf(sd->name, sizeof(sd->name), "mxc_isi.%d", pipe->id);

	sd->entity.function = MEDIA_ENT_F_PROC_VIDEO_PIXEL_FORMATTER;

	pipe->pads[MXC_ISI_SD_PAD_SINK].flags = MEDIA_PAD_FL_SINK;
	pipe->pads[MXC_ISI_SD_PAD_SOURCE].flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&sd->entity, MXC_ISI_SD_PADS_NUM, pipe->pads);
	if (ret)
		return ret;

	sd->internal_ops = &mxc_isi_capture_sd_internal_ops;

	v4l2_set_subdevdata(sd, pipe);

	sd->fwnode = of_fwnode_handle(pipe->isi->dev->of_node);

	/* Default configuration. */
	mxc_isi_pipe_init_cfg(sd, NULL);

	for (i = 0; i < ARRAY_SIZE(pipe->formats); ++i) {
		struct mxc_isi_frame *frame = &pipe->formats[i];

		frame->info = mxc_isi_format_by_code(frame->format.code);
	}

	return 0;
}

void mxc_isi_pipe_cleanup(struct mxc_isi_dev *isi)
{
	struct mxc_isi_pipe *pipe = &isi->pipe;
	struct v4l2_subdev *sd = &pipe->sd;

	media_entity_cleanup(&sd->entity);
	v4l2_set_subdevdata(sd, NULL);
}
