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

static const struct mxc_isi_fmt mxc_isi_out_formats[] = {
	{
		.fourcc		= V4L2_PIX_FMT_GREY,
		.depth		= { 8 },
		.color		= MXC_ISI_OUT_FMT_RAW8,
		.memplanes	= 1,
		.mbus_code  = MEDIA_BUS_FMT_Y8_1X8,
	}, {
		.fourcc		= V4L2_PIX_FMT_Y10,
		.depth		= { 16 },
		.color		= MXC_ISI_OUT_FMT_RAW10,
		.memplanes	= 1,
		.mbus_code  = MEDIA_BUS_FMT_Y10_1X10,
	}, {
		.fourcc		= V4L2_PIX_FMT_Y12,
		.depth		= { 16 },
		.color		= MXC_ISI_OUT_FMT_RAW12,
		.memplanes	= 1,
		.mbus_code  = MEDIA_BUS_FMT_Y12_1X12,
	}, {
		.fourcc		= V4L2_PIX_FMT_SBGGR8,
		.depth		= { 8 },
		.color		= MXC_ISI_OUT_FMT_RAW8,
		.memplanes	= 1,
		.mbus_code  = MEDIA_BUS_FMT_SBGGR8_1X8,
	}, {
		.fourcc		= V4L2_PIX_FMT_SGBRG8,
		.depth		= { 8 },
		.color		= MXC_ISI_OUT_FMT_RAW8,
		.memplanes	= 1,
		.mbus_code  = MEDIA_BUS_FMT_SGBRG8_1X8,
	}, {
		.fourcc		= V4L2_PIX_FMT_SGRBG8,
		.depth		= { 8 },
		.color		= MXC_ISI_OUT_FMT_RAW8,
		.memplanes	= 1,
		.mbus_code  = MEDIA_BUS_FMT_SGRBG8_1X8,
	}, {
		.fourcc		= V4L2_PIX_FMT_SRGGB8,
		.depth		= { 8 },
		.color		= MXC_ISI_OUT_FMT_RAW8,
		.memplanes	= 1,
		.mbus_code  = MEDIA_BUS_FMT_SRGGB8_1X8,
	}, {
		.fourcc		= V4L2_PIX_FMT_SBGGR10,
		.depth		= { 16 },
		.color		= MXC_ISI_OUT_FMT_RAW10,
		.memplanes	= 1,
		.mbus_code  = MEDIA_BUS_FMT_SBGGR10_1X10,
	}, {
		.fourcc		= V4L2_PIX_FMT_SGBRG10,
		.depth		= { 16 },
		.color		= MXC_ISI_OUT_FMT_RAW10,
		.memplanes	= 1,
		.mbus_code  = MEDIA_BUS_FMT_SGBRG10_1X10,
	}, {
		.fourcc		= V4L2_PIX_FMT_SGRBG10,
		.depth		= { 16 },
		.color		= MXC_ISI_OUT_FMT_RAW10,
		.memplanes	= 1,
		.mbus_code  = MEDIA_BUS_FMT_SGRBG10_1X10,
	}, {
		.fourcc		= V4L2_PIX_FMT_SRGGB10,
		.depth		= { 16 },
		.color		= MXC_ISI_OUT_FMT_RAW10,
		.memplanes	= 1,
		.mbus_code  = MEDIA_BUS_FMT_SRGGB10_1X10,
	}, {
		.fourcc		= V4L2_PIX_FMT_SBGGR12,
		.depth		= { 16 },
		.color		= MXC_ISI_OUT_FMT_RAW12,
		.memplanes	= 1,
		.mbus_code  = MEDIA_BUS_FMT_SBGGR12_1X12,
	}, {
		.fourcc		= V4L2_PIX_FMT_SGBRG12,
		.depth		= { 16 },
		.color		= MXC_ISI_OUT_FMT_RAW12,
		.memplanes	= 1,
		.mbus_code  = MEDIA_BUS_FMT_SGBRG12_1X12,
	}, {
		.fourcc		= V4L2_PIX_FMT_SGRBG12,
		.depth		= { 16 },
		.color		= MXC_ISI_OUT_FMT_RAW12,
		.memplanes	= 1,
		.mbus_code  = MEDIA_BUS_FMT_SGRBG12_1X12,
	}, {
		.fourcc		= V4L2_PIX_FMT_SRGGB12,
		.depth		= { 16 },
		.color		= MXC_ISI_OUT_FMT_RAW12,
		.memplanes	= 1,
		.mbus_code  = MEDIA_BUS_FMT_SRGGB12_1X12,
	}, {
		.fourcc		= V4L2_PIX_FMT_RGB565,
		.depth		= { 16 },
		.color		= MXC_ISI_OUT_FMT_RGB565,
		.memplanes	= 1,
		.mbus_code  = MEDIA_BUS_FMT_RGB565_1X16,
	}, {
		.fourcc		= V4L2_PIX_FMT_RGB24,
		.depth		= { 24 },
		.color		= MXC_ISI_OUT_FMT_BGR32P,
		.memplanes	= 1,
		.mbus_code  = MEDIA_BUS_FMT_RGB888_1X24,
	}, {
		.fourcc		= V4L2_PIX_FMT_BGR24,
		.depth		= { 24 },
		.color		= MXC_ISI_OUT_FMT_RGB32P,
		.memplanes	= 1,
		.mbus_code  = MEDIA_BUS_FMT_BGR888_1X24,
	}, {
		.fourcc		= V4L2_PIX_FMT_YUYV,
		.depth		= { 16 },
		.color		= MXC_ISI_OUT_FMT_YUV422_1P8P,
		.memplanes	= 1,
		.mbus_code	= MEDIA_BUS_FMT_YUYV8_1X16,
	}, {
		.fourcc		= V4L2_PIX_FMT_YUV32,
		.depth		= { 32 },
		.color		= MXC_ISI_OUT_FMT_YUV444_1P8,
		.memplanes	= 1,
		.mbus_code	= MEDIA_BUS_FMT_AYUV8_1X32,
	}, {
		.fourcc		= V4L2_PIX_FMT_NV12,
		.depth		= { 8, 8 },
		.color		= MXC_ISI_OUT_FMT_YUV420_2P8P,
		.memplanes	= 2,
		.mbus_code	= MEDIA_BUS_FMT_YUYV8_1X16,
	}, {
		.fourcc		= V4L2_PIX_FMT_YUV444M,
		.depth		= { 8, 8, 8 },
		.color		= MXC_ISI_OUT_FMT_YUV444_3P8P,
		.memplanes	= 3,
		.mbus_code	= MEDIA_BUS_FMT_YUV8_1X24,
	}, {
		.fourcc		= V4L2_PIX_FMT_XBGR32,
		.depth		= { 32 },
		.color		= MXC_ISI_OUT_FMT_XRGB32,
		.memplanes	= 1,
		.mbus_code	= MEDIA_BUS_FMT_RGB888_1X24,
	}, {
		.fourcc		= V4L2_PIX_FMT_ABGR32,
		.depth		= { 32 },
		.color		= MXC_ISI_OUT_FMT_ARGB32,
		.memplanes	= 1,
		.mbus_code	= MEDIA_BUS_FMT_RGB888_1X24,
	}
};

/*
 * Pixel link input format
 */
static const struct mxc_isi_fmt mxc_isi_src_formats[] = {
	{
		.fourcc		= V4L2_PIX_FMT_RGB32,
		.depth		= { 32 },
		.memplanes	= 1,
	}, {
		.fourcc		= V4L2_PIX_FMT_YUV32,
		.depth		= { 32 },
		.memplanes	= 1,
	}
};

static const struct mxc_isi_fmt *mxc_isi_format_by_code(u32 code)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(mxc_isi_out_formats); i++) {
		const struct mxc_isi_fmt *fmt = &mxc_isi_out_formats[i];

		if (fmt->mbus_code == code)
			return fmt;
	}

	return NULL;
}

static const struct mxc_isi_fmt *mxc_isi_format_by_fourcc(u32 fourcc)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(mxc_isi_out_formats); i++) {
		const struct mxc_isi_fmt *fmt = &mxc_isi_out_formats[i];

		if (fmt->fourcc == fourcc)
			return fmt;
	}

	return NULL;
}

static const struct mxc_isi_fmt *mxc_isi_get_src_fmt(u32 code)
{
	/* two fmt RGB32 and YUV444 from pixellink */
	switch (code) {
	case MEDIA_BUS_FMT_YUYV8_1X16:
	case MEDIA_BUS_FMT_YVYU8_2X8:
	case MEDIA_BUS_FMT_AYUV8_1X32:
	case MEDIA_BUS_FMT_UYVY8_2X8:
	case MEDIA_BUS_FMT_YUYV8_2X8:
		return &mxc_isi_src_formats[1];

	default:
		return &mxc_isi_src_formats[0];
	}
}

static struct media_pad
*mxc_isi_get_remote_source_pad(struct v4l2_subdev *subdev)
{
	unsigned int i;

	for (i = 0; i < subdev->entity.num_pads; i++) {
		struct media_pad *pad = &subdev->entity.pads[i];

		if (!(pad->flags & MEDIA_PAD_FL_SINK))
			continue;

		pad = media_entity_remote_pad(pad);
		if (pad)
			return pad;
	}

	return NULL;
}

static struct v4l2_subdev *mxc_get_source_subdev(struct v4l2_subdev *subdev,
						 const char * const label)
{
	struct media_pad *source_pad;
	struct v4l2_subdev *sen_sd;

	/* Get remote source pad */
	source_pad = mxc_isi_get_remote_source_pad(subdev);
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

	src_sd = mxc_get_source_subdev(&pipe->sd, __func__);
	if (!src_sd)
		return -EPIPE;

	ret = v4l2_subdev_call(src_sd, video, s_stream, enable);
	if (ret < 0 && ret != -ENOIOCTLCMD) {
		dev_err(dev, "subdev %s s_stream failed\n", src_sd->name);
		return ret;
	}

	return 0;
}

static int mxc_isi_update_buf_paddr(struct mxc_isi_buffer *buf, int memplanes)
{
	struct frame_addr *paddr = &buf->paddr;
	struct vb2_buffer *vb2 = &buf->v4l2_buf.vb2_buf;

	paddr->cb = 0;
	paddr->cr = 0;

	switch (memplanes) {
	case 3:
		paddr->cr = vb2_dma_contig_plane_dma_addr(vb2, 2);
		/* fall through */
	case 2:
		paddr->cb = vb2_dma_contig_plane_dma_addr(vb2, 1);
		/* fall through */
	case 1:
		paddr->y = vb2_dma_contig_plane_dma_addr(vb2, 0);
		break;
	default:
		return -EINVAL;
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
	struct mxc_isi_frame *dst_f = &pipe->dst_f;
	const struct mxc_isi_fmt *fmt = dst_f->fmt;
	unsigned long wh;
	int i;

	if (!fmt)
		return -EINVAL;

	for (i = 0; i < fmt->memplanes; i++)
		alloc_devs[i] = pipe->isi->dev;

	wh = dst_f->width * dst_f->height;

	*num_planes = fmt->memplanes;

	for (i = 0; i < fmt->memplanes; i++) {
		unsigned int size = (wh * fmt->depth[i]) / 8;

		if (i == 1 && fmt->fourcc == V4L2_PIX_FMT_NV12)
			size >>= 1;
		sizes[i] = max_t(u32, size, dst_f->sizeimage[i]);
	}
	dev_dbg(pipe->isi->dev, "%s, buf_n=%d, size=%d\n",
		__func__, *num_buffers, sizes[0]);

	return 0;
}

static int cap_vb2_buffer_prepare(struct vb2_buffer *vb2)
{
	struct vb2_queue *q = vb2->vb2_queue;
	struct mxc_isi_pipe *pipe = vb2_get_drv_priv(q);
	struct mxc_isi_frame *dst_f = &pipe->dst_f;
	int i;

	dev_dbg(pipe->isi->dev, "%s\n", __func__);

	if (!pipe->dst_f.fmt)
		return -EINVAL;

	for (i = 0; i < dst_f->fmt->memplanes; i++) {
		unsigned long size = dst_f->sizeimage[i];

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

	mxc_isi_update_buf_paddr(buf, pipe->dst_f.fmt->mdataplanes);
	list_add_tail(&buf->list, &pipe->video.out_pending);

	spin_unlock_irqrestore(&pipe->slock, flags);
}

static int cap_vb2_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct mxc_isi_pipe *pipe = vb2_get_drv_priv(q);
	struct mxc_isi_dev *isi = pipe->isi;
	struct mxc_isi_buffer *buf;
	struct vb2_buffer *vb2;
	unsigned long flags;
	int i, j;

	dev_dbg(pipe->isi->dev, "%s\n", __func__);

	if (count < 2)
		return -ENOBUFS;

	if (!isi)
		return -EINVAL;

	/* Create a buffer for discard operation */
	for (i = 0; i < pipe->video.pix.num_planes; i++) {
		pipe->video.discard_size[i] = pipe->dst_f.sizeimage[i];
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
			return -ENOMEM;
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

	return 0;
}

static void cap_vb2_stop_streaming(struct vb2_queue *q)
{
	struct mxc_isi_pipe *pipe = vb2_get_drv_priv(q);
	struct mxc_isi_dev *isi = pipe->isi;
	struct mxc_isi_buffer *buf;
	unsigned long flags;
	int i;

	dev_dbg(pipe->isi->dev, "%s\n", __func__);

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

	dev_dbg(pipe->isi->dev, "%s\n", __func__);

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

	v4l2_ctrl_handler_init(handler, 4);

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
	struct mxc_isi_pipe *pipe = video_drvdata(file);
	const struct mxc_isi_fmt *fmt;

	dev_dbg(pipe->isi->dev, "%s\n", __func__);

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
	struct v4l2_pix_format_mplane *pix = &f->fmt.pix_mp;
	struct mxc_isi_frame *dst_f = &pipe->dst_f;
	int i;

	dev_dbg(pipe->isi->dev, "%s\n", __func__);

	pix->width = dst_f->o_width;
	pix->height = dst_f->o_height;
	pix->field = V4L2_FIELD_NONE;
	pix->pixelformat = dst_f->fmt->fourcc;
	pix->colorspace = V4L2_COLORSPACE_JPEG;
	pix->num_planes = dst_f->fmt->memplanes;

	for (i = 0; i < pix->num_planes; ++i) {
		pix->plane_fmt[i].bytesperline = dst_f->bytesperline[i];
		pix->plane_fmt[i].sizeimage = dst_f->sizeimage[i];
	}

	return 0;
}

static int mxc_isi_cap_try_fmt_mplane(struct file *file, void *fh,
				      struct v4l2_format *f)
{
	struct mxc_isi_pipe *pipe = video_drvdata(file);
	struct v4l2_pix_format_mplane *pix = &f->fmt.pix_mp;
	const struct mxc_isi_fmt *fmt;

	dev_dbg(pipe->isi->dev, "%s\n", __func__);

	fmt = mxc_isi_format_by_fourcc(pix->pixelformat);
	if (!fmt)
		return -EINVAL;

	if (pix->width <= 0 || pix->height <= 0) {
		v4l2_err(&pipe->sd, "%s, W/H=(%d, %d) is not valid\n"
			, __func__, pix->width, pix->height);
		return -EINVAL;
	}

	return 0;
}

/* Update input frame size and formate  */
static int mxc_isi_source_fmt_init(struct mxc_isi_pipe *pipe)
{
	struct mxc_isi_frame *src_f = &pipe->src_f;
	struct mxc_isi_frame *dst_f = &pipe->dst_f;
	struct v4l2_subdev_format src_fmt;
	struct media_pad *source_pad;
	struct v4l2_subdev *src_sd;
	int ret;

	source_pad = mxc_isi_get_remote_source_pad(&pipe->sd);
	if (!source_pad) {
		v4l2_err(&pipe->sd,
			 "%s, No remote pad found!\n", __func__);
		return -EINVAL;
	}

	src_sd = mxc_get_source_subdev(&pipe->sd, __func__);
	if (!src_sd)
		return -EINVAL;

	src_fmt.pad = source_pad->index;
	src_fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	src_fmt.format.code = dst_f->fmt->mbus_code;
	src_fmt.format.width = dst_f->width;
	src_fmt.format.height = dst_f->height;
	ret = v4l2_subdev_call(src_sd, pad, set_fmt, NULL, &src_fmt);
	if (ret < 0 && ret != -ENOIOCTLCMD) {
		v4l2_err(&pipe->sd, "set remote fmt fail!\n");
		return ret;
	}

	memset(&src_fmt, 0, sizeof(src_fmt));
	src_fmt.pad = source_pad->index;
	src_fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	ret = v4l2_subdev_call(src_sd, pad, get_fmt, NULL, &src_fmt);
	if (ret < 0 && ret != -ENOIOCTLCMD) {
		v4l2_err(&pipe->sd, "get remote fmt fail!\n");
		return ret;
	}

	/* Pixel link master will transfer format to RGB32 or YUV32 */
	src_f->fmt = mxc_isi_get_src_fmt(src_fmt.format.code);

	set_frame_bounds(src_f, src_fmt.format.width, src_fmt.format.height);

	if (dst_f->width > src_f->width || dst_f->height > src_f->height) {
		dev_err(pipe->isi->dev,
			"%s: src:(%d,%d), dst:(%d,%d) Not support upscale\n",
			__func__,
			src_f->width, src_f->height,
			dst_f->width, dst_f->height);
		return -EINVAL;
	}

	return 0;
}

static int mxc_isi_cap_s_fmt_mplane(struct file *file, void *priv,
				    struct v4l2_format *f)
{
	struct mxc_isi_pipe *pipe = video_drvdata(file);
	struct v4l2_pix_format_mplane *pix = &f->fmt.pix_mp;
	struct mxc_isi_frame *dst_f = &pipe->dst_f;
	const struct mxc_isi_fmt *fmt;
	int bpl;
	int i;

	/* Step1: Check format with output support format list.
	 * Step2: Update output frame information.
	 * Step3: Checkout the format whether is supported by remote subdev
	 *	 Step3.1: If Yes, call remote subdev set_fmt.
	 *	 Step3.2: If NO, call remote subdev get_fmt.
	 * Step4: Update input frame information.
	 * Step5: Update mxc isi channel configuration.
	 */

	dev_dbg(pipe->isi->dev, "%s, fmt=0x%X\n", __func__, pix->pixelformat);
	if (vb2_is_busy(&pipe->video.vb2_q))
		return -EBUSY;

	/* Check out put format */
	fmt = mxc_isi_format_by_fourcc(pix->pixelformat);
	if (!fmt)
		return -EINVAL;

	/* update out put frame size and formate */
	if (pix->height <= 0 || pix->width <= 0)
		return -EINVAL;

	dst_f->fmt = fmt;
	dst_f->height = pix->height;
	dst_f->width = pix->width;

	pix->num_planes = fmt->memplanes;

	for (i = 0; i < pix->num_planes; i++) {
		bpl = pix->plane_fmt[i].bytesperline;

		if ((bpl == 0) || (bpl / (fmt->depth[i] >> 3)) < pix->width)
			pix->plane_fmt[i].bytesperline =
					(pix->width * fmt->depth[i]) >> 3;

		if ((i == 1) && (pix->pixelformat == V4L2_PIX_FMT_NV12))
			pix->plane_fmt[i].sizeimage =
				(pix->width * (pix->height >> 1) *
				 fmt->depth[i] >> 3);
		else
			pix->plane_fmt[i].sizeimage =
				(pix->width * pix->height *
				 fmt->depth[i] >> 3);
	}

	if (pix->num_planes > 1) {
		for (i = 0; i < pix->num_planes; i++) {
			dst_f->bytesperline[i] = pix->plane_fmt[i].bytesperline;
			dst_f->sizeimage[i]    = pix->plane_fmt[i].sizeimage;
		}
	} else {
		dst_f->bytesperline[0] = dst_f->width * dst_f->fmt->depth[0] / 8;
		dst_f->sizeimage[0]    = dst_f->height * dst_f->bytesperline[0];
	}

	memcpy(&pipe->video.pix, pix, sizeof(*pix));
	set_frame_bounds(dst_f, pix->width, pix->height);

	return 0;
}

static int mxc_isi_config_parm(struct mxc_isi_pipe *pipe)
{
	struct mxc_isi_dev *isi = pipe->isi;
	int ret;

	ret = mxc_isi_source_fmt_init(pipe);
	if (ret < 0)
		return -EINVAL;

	mxc_isi_channel_init(isi);
	mxc_isi_channel_config(isi, &pipe->src_f, &pipe->dst_f);

	return 0;
}

static int mxc_isi_cap_streamon(struct file *file, void *priv,
				enum v4l2_buf_type type)
{
	struct mxc_isi_pipe *pipe = video_drvdata(file);
	struct mxc_isi_dev *isi = pipe->isi;
	int ret;

	dev_dbg(pipe->isi->dev, "%s\n", __func__);

	ret = mxc_isi_config_parm(pipe);
	if (ret < 0)
		return ret;

	ret = vb2_ioctl_streamon(file, priv, type);
	mxc_isi_channel_enable(isi, false);
	ret = mxc_isi_pipeline_enable(pipe, 1);
	if (ret < 0 && ret != -ENOIOCTLCMD)
		return ret;

	isi->is_streaming = 1;

	return 0;
}

static int mxc_isi_cap_streamoff(struct file *file, void *priv,
				 enum v4l2_buf_type type)
{
	struct mxc_isi_pipe *pipe = video_drvdata(file);
	struct mxc_isi_dev *isi = pipe->isi;
	int ret;

	dev_dbg(pipe->isi->dev, "%s\n", __func__);

	mxc_isi_pipeline_enable(pipe, 0);
	mxc_isi_channel_disable(isi);
	ret = vb2_ioctl_streamoff(file, priv, type);

	isi->is_streaming = 0;

	return ret;
}

static int mxc_isi_cap_g_selection(struct file *file, void *fh,
				   struct v4l2_selection *s)
{
	struct mxc_isi_pipe *pipe = video_drvdata(file);
	struct mxc_isi_frame *f = &pipe->src_f;

	dev_dbg(pipe->isi->dev, "%s\n", __func__);

	if (s->type != V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		return -EINVAL;

	switch (s->target) {
	case V4L2_SEL_TGT_COMPOSE_DEFAULT:
	case V4L2_SEL_TGT_COMPOSE_BOUNDS:
		f = &pipe->dst_f;
		/* fall through */
	case V4L2_SEL_TGT_CROP_BOUNDS:
	case V4L2_SEL_TGT_CROP_DEFAULT:
		s->r.left = 0;
		s->r.top = 0;
		s->r.width = f->o_width;
		s->r.height = f->o_height;
		return 0;

	case V4L2_SEL_TGT_COMPOSE:
		f = &pipe->dst_f;
		/* fall through */
	case V4L2_SEL_TGT_CROP:
		s->r.left = f->h_off;
		s->r.top = f->v_off;
		s->r.width = f->width;
		s->r.height = f->height;
		return 0;
	}

	return -EINVAL;
}

static int enclosed_rectangle(struct v4l2_rect *a, struct v4l2_rect *b)
{
	if (a->left < b->left || a->top < b->top)
		return 0;

	if (a->left + a->width > b->left + b->width)
		return 0;

	if (a->top + a->height > b->top + b->height)
		return 0;

	return 1;
}

static int mxc_isi_cap_s_selection(struct file *file, void *fh,
				   struct v4l2_selection *s)
{
	struct mxc_isi_pipe *pipe = video_drvdata(file);
	struct mxc_isi_frame *f;
	struct v4l2_rect rect = s->r;
	unsigned long flags;

	dev_dbg(pipe->isi->dev, "%s\n", __func__);
	if (s->type != V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		return -EINVAL;

	if (s->target == V4L2_SEL_TGT_COMPOSE)
		f = &pipe->dst_f;
	else if (s->target == V4L2_SEL_TGT_CROP)
		f = &pipe->src_f;
	else
		return -EINVAL;

	if (s->flags & V4L2_SEL_FLAG_LE &&
	    !enclosed_rectangle(&rect, &s->r))
		return -ERANGE;

	if (s->flags & V4L2_SEL_FLAG_GE &&
	    !enclosed_rectangle(&s->r, &rect))
		return -ERANGE;

	s->r = rect;
	spin_lock_irqsave(&pipe->slock, flags);
	set_frame_crop(f, s->r.left, s->r.top, s->r.width,
		       s->r.height);
	spin_unlock_irqrestore(&pipe->slock, flags);

	return 0;
}

static int mxc_isi_cap_enum_framesizes(struct file *file, void *priv,
				       struct v4l2_frmsizeenum *fsize)
{
	struct mxc_isi_pipe *pipe = video_drvdata(file);
	const struct mxc_isi_fmt *fmt;

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

	.vidioc_streamon		= mxc_isi_cap_streamon,
	.vidioc_streamoff		= mxc_isi_cap_streamoff,

	.vidioc_g_selection		= mxc_isi_cap_g_selection,
	.vidioc_s_selection		= mxc_isi_cap_s_selection,

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

	csi_sd = mxc_get_source_subdev(&pipe->sd, __func__);
	if (!csi_sd || !csi_sd->entity.num_links)
		return false;

	sen_sd = mxc_get_source_subdev(csi_sd, __func__);
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
	struct video_device *vdev = &pipe->video.vdev;
	struct vb2_queue *q = &pipe->video.vb2_q;
	int ret = -ENOMEM;

	dev_dbg(pipe->isi->dev, "%s\n", __func__);
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
				    MXC_ISI_SD_PAD_SOURCE_MEM,
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

static int mxc_isi_subdev_enum_mbus_code(struct v4l2_subdev *sd,
					 struct v4l2_subdev_pad_config *cfg,
					 struct v4l2_subdev_mbus_code_enum *code)
{
	return 0;
}

static int mxc_isi_subdev_get_fmt(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_format *fmt)
{
	struct mxc_isi_pipe *pipe = v4l2_get_subdevdata(sd);
	struct mxc_isi_frame *f;
	struct v4l2_mbus_framefmt *mf = &fmt->format;

	mutex_lock(&pipe->lock);

	switch (fmt->pad) {
	case MXC_ISI_SD_PAD_SOURCE_MEM:
	case MXC_ISI_SD_PAD_SOURCE_DC0:
	case MXC_ISI_SD_PAD_SOURCE_DC1:
		f = &pipe->dst_f;
		break;
	case MXC_ISI_SD_PAD_SINK_MIPI0:
	case MXC_ISI_SD_PAD_SINK_MIPI1:
	case MXC_ISI_SD_PAD_SINK_HDMI:
	case MXC_ISI_SD_PAD_SINK_DC0:
	case MXC_ISI_SD_PAD_SINK_DC1:
	case MXC_ISI_SD_PAD_SINK_MEM:
		f = &pipe->src_f;
		break;
	default:
		mutex_unlock(&pipe->lock);
		v4l2_err(&pipe->sd,
			 "%s, Pad is not support now!\n", __func__);
		return -1;
	}

	if (!WARN_ON(!f->fmt))
		mf->code = f->fmt->mbus_code;

	/* Source/Sink pads crop rectangle size */
	mf->width = f->width;
	mf->height = f->height;
	mf->colorspace = V4L2_COLORSPACE_JPEG;

	mutex_unlock(&pipe->lock);

	return 0;
}

static int mxc_isi_subdev_set_fmt(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_format *fmt)
{
	struct mxc_isi_pipe *pipe = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *mf = &fmt->format;
	struct mxc_isi_frame *dst_f = &pipe->dst_f;
	const struct mxc_isi_fmt *out_fmt;
	int i;

	if (fmt->pad < MXC_ISI_SD_PAD_SOURCE_MEM &&
	    vb2_is_busy(&pipe->video.vb2_q))
		return -EBUSY;

	out_fmt = mxc_isi_format_by_code(mf->code);
	if (!out_fmt)
		return -EINVAL;

	if (pipe->isi->pdata->model == MXC_ISI_IMX8MN && mf->width > ISI_2K)
		return -EINVAL;

	mutex_lock(&pipe->lock);
	/* update out put frame size and formate */
	dst_f->fmt = out_fmt;

	if (dst_f->fmt->memplanes > 1) {
		for (i = 0; i < dst_f->fmt->memplanes; i++) {
			if ((i == 1) &&
			    (dst_f->fmt->fourcc == V4L2_PIX_FMT_NV12))
				dst_f->sizeimage[i] = (mf->width *
						      (mf->height >> 1) *
						      dst_f->fmt->depth[i] >> 3);
			else
				dst_f->sizeimage[i] = (mf->width *
						      mf->height *
						      dst_f->fmt->depth[i] >> 3);
		}
		dst_f->bytesperline[i] = (mf->width *
					 dst_f->fmt->depth[i] >> 3);
	} else {
		dst_f->bytesperline[0] = mf->width * dst_f->fmt->depth[0] / 8;
		dst_f->sizeimage[0]    = mf->height * dst_f->bytesperline[0];
	}

	set_frame_bounds(dst_f, mf->width, mf->height);
	mutex_unlock(&pipe->lock);

	dev_dbg(pipe->isi->dev, "pad%d: code: 0x%x, %dx%d",
		fmt->pad, mf->code, mf->width, mf->height);

	return 0;
}

static int mxc_isi_subdev_get_selection(struct v4l2_subdev *sd,
					struct v4l2_subdev_pad_config *cfg,
					struct v4l2_subdev_selection *sel)
{
	struct mxc_isi_pipe *pipe = v4l2_get_subdevdata(sd);
	struct mxc_isi_frame *f = &pipe->src_f;
	struct v4l2_rect *r = &sel->r;
	struct v4l2_rect *try_sel;

	mutex_lock(&pipe->lock);

	switch (sel->target) {
	case V4L2_SEL_TGT_COMPOSE_BOUNDS:
		f = &pipe->dst_f;
		/* fall through */
	case V4L2_SEL_TGT_CROP_BOUNDS:
		r->width = f->o_width;
		r->height = f->o_height;
		r->left = 0;
		r->top = 0;
		mutex_unlock(&pipe->lock);
		return 0;

	case V4L2_SEL_TGT_CROP:
		try_sel = v4l2_subdev_get_try_crop(sd, cfg, sel->pad);
		break;
	case V4L2_SEL_TGT_COMPOSE:
		try_sel = v4l2_subdev_get_try_compose(sd, cfg, sel->pad);
		f = &pipe->dst_f;
		break;
	default:
		mutex_unlock(&pipe->lock);
		return -EINVAL;
	}

	if (sel->which == V4L2_SUBDEV_FORMAT_TRY) {
		sel->r = *try_sel;
	} else {
		r->left = f->h_off;
		r->top = f->v_off;
		r->width = f->width;
		r->height = f->height;
	}

	dev_dbg(pipe->isi->dev,
		"%s, target %#x: l:%d, t:%d, %dx%d, f_w: %d, f_h: %d",
		__func__, sel->pad, r->left, r->top, r->width, r->height,
		f->c_width, f->c_height);

	mutex_unlock(&pipe->lock);
	return 0;
}

static int mxc_isi_subdev_set_selection(struct v4l2_subdev *sd,
					struct v4l2_subdev_pad_config *cfg,
					struct v4l2_subdev_selection *sel)
{
	struct mxc_isi_pipe *pipe = v4l2_get_subdevdata(sd);
	struct mxc_isi_frame *f = &pipe->src_f;
	struct v4l2_rect *r = &sel->r;
	struct v4l2_rect *try_sel;
	unsigned long flags;

	mutex_lock(&pipe->lock);

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
		try_sel = v4l2_subdev_get_try_crop(sd, cfg, sel->pad);
		break;
	case V4L2_SEL_TGT_COMPOSE:
		try_sel = v4l2_subdev_get_try_compose(sd, cfg, sel->pad);
		f = &pipe->dst_f;
		break;
	default:
		mutex_unlock(&pipe->lock);
		return -EINVAL;
	}

	if (sel->which == V4L2_SUBDEV_FORMAT_TRY) {
		*try_sel = sel->r;
	} else {
		spin_lock_irqsave(&pipe->slock, flags);
		set_frame_crop(f, r->left, r->top, r->width, r->height);
		spin_unlock_irqrestore(&pipe->slock, flags);
	}

	dev_dbg(pipe->isi->dev, "%s, target %#x: (%d,%d)/%dx%d", __func__,
		sel->target, r->left, r->top, r->width, r->height);

	mutex_unlock(&pipe->lock);

	return 0;
}

static const struct v4l2_subdev_pad_ops mxc_isi_subdev_pad_ops = {
	.enum_mbus_code = mxc_isi_subdev_enum_mbus_code,
	.get_selection  = mxc_isi_subdev_get_selection,
	.set_selection  = mxc_isi_subdev_set_selection,
	.get_fmt = mxc_isi_subdev_get_fmt,
	.set_fmt = mxc_isi_subdev_set_fmt,
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

	dev_dbg(pipe->isi->dev, "%s\n", __func__);

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

	dev_dbg(pipe->isi->dev, "%s\n", __func__);

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

	/* ISI Sink pads */
	pipe->pads[MXC_ISI_SD_PAD_SINK_MIPI0].flags = MEDIA_PAD_FL_SINK;
	pipe->pads[MXC_ISI_SD_PAD_SINK_MIPI1].flags = MEDIA_PAD_FL_SINK;
	pipe->pads[MXC_ISI_SD_PAD_SINK_DC0].flags = MEDIA_PAD_FL_SINK;
	pipe->pads[MXC_ISI_SD_PAD_SINK_DC1].flags = MEDIA_PAD_FL_SINK;
	pipe->pads[MXC_ISI_SD_PAD_SINK_HDMI].flags = MEDIA_PAD_FL_SINK;
	pipe->pads[MXC_ISI_SD_PAD_SINK_MEM].flags = MEDIA_PAD_FL_SINK;
	pipe->pads[MXC_ISI_SD_PAD_SINK_PARALLEL_CSI].flags = MEDIA_PAD_FL_SINK;

	/* ISI source pads */
	pipe->pads[MXC_ISI_SD_PAD_SOURCE_MEM].flags = MEDIA_PAD_FL_SOURCE;
	pipe->pads[MXC_ISI_SD_PAD_SOURCE_DC0].flags = MEDIA_PAD_FL_SOURCE;
	pipe->pads[MXC_ISI_SD_PAD_SOURCE_DC1].flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&sd->entity, MXC_ISI_SD_PADS_NUM, pipe->pads);
	if (ret)
		return ret;

	sd->internal_ops = &mxc_isi_capture_sd_internal_ops;

	v4l2_set_subdevdata(sd, pipe);

	sd->fwnode = of_fwnode_handle(pipe->isi->dev->of_node);

	/* Default configuration  */
	pipe->dst_f.width = 1280;
	pipe->dst_f.height = 800;
	pipe->dst_f.fmt = &mxc_isi_out_formats[0];
	pipe->src_f.fmt = pipe->dst_f.fmt;

	return 0;
}

void mxc_isi_pipe_cleanup(struct mxc_isi_dev *isi)
{
	struct mxc_isi_pipe *pipe = &isi->pipe;
	struct v4l2_subdev *sd = &pipe->sd;

	media_entity_cleanup(&sd->entity);
	v4l2_set_subdevdata(sd, NULL);
}
