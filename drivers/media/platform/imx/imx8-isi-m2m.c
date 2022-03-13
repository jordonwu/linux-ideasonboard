// SPDX-License-Identifier: GPL-2.0
/*
 * ISI V4L2 memory to memory driver for i.MX8QXP/QM platform
 *
 * ISI is a Image Sensor Interface of i.MX8QXP/QM platform, which
 * used to process image from camera sensor or memory to memory or DC
 *
 * Copyright (c) 2019 NXP Semiconductor
 */

#include <linux/container_of.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/limits.h>
#include <linux/minmax.h>
#include <linux/mutex.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/videodev2.h>

#include <media/media-entity.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-dma-contig.h>

#include "imx8-isi-core.h"

struct mxc_isi_m2m_buffer {
	struct v4l2_m2m_buffer buf;
	u32 dma_addrs[3];
};

struct mxc_isi_m2m_ctx_queue_data {
	struct v4l2_pix_format_mplane format;
	const struct mxc_isi_format_info *info;
	u32 sequence;
};

struct mxc_isi_m2m_ctx {
	struct v4l2_fh fh;
	struct mxc_isi_m2m *m2m;

	struct {
		struct mxc_isi_m2m_ctx_queue_data out;
		struct mxc_isi_m2m_ctx_queue_data cap;
	} queues;
};

static inline struct mxc_isi_m2m_buffer *
to_isi_m2m_buffer(struct vb2_v4l2_buffer *buf)
{
	return container_of(buf, struct mxc_isi_m2m_buffer, buf.vb);
}

static inline struct mxc_isi_m2m_ctx *to_isi_m2m_ctx(struct v4l2_fh *fh)
{
	return container_of(fh, struct mxc_isi_m2m_ctx, fh);
}

static inline struct mxc_isi_m2m_ctx_queue_data *
mxc_isi_m2m_ctx_qdata(struct mxc_isi_m2m_ctx *ctx, enum v4l2_buf_type type)
{
	if (V4L2_TYPE_IS_OUTPUT(type))
		return &ctx->queues.out;
	else
		return &ctx->queues.cap;
}

/* -----------------------------------------------------------------------------
 * V4L2 M2M device operations
 */

static void mxc_isi_m2m_frame_write_done(struct mxc_isi_dev *isi)
{
	struct mxc_isi_m2m *m2m = &isi->m2m;
	struct vb2_v4l2_buffer *src_vbuf, *dst_vbuf;
	struct mxc_isi_m2m_ctx *ctx;

	ctx = v4l2_m2m_get_curr_priv(m2m->m2m_dev);
	if (!ctx) {
		dev_err(m2m->isi->dev,
			"Instance released before the end of transaction\n");
		return;
	}

	src_vbuf = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx);
	dst_vbuf = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);

	v4l2_m2m_buf_copy_metadata(src_vbuf, dst_vbuf, false);

	src_vbuf->sequence = ctx->queues.out.sequence++;
	dst_vbuf->sequence = ctx->queues.cap.sequence++;

	v4l2_m2m_buf_done(src_vbuf, VB2_BUF_STATE_DONE);
	v4l2_m2m_buf_done(dst_vbuf, VB2_BUF_STATE_DONE);

	v4l2_m2m_job_finish(m2m->m2m_dev, ctx->fh.m2m_ctx);
}

static void mxc_isi_m2m_device_run(void *priv)
{
	struct mxc_isi_m2m_ctx *ctx = priv;
	struct mxc_isi_m2m *m2m = ctx->m2m;
	struct vb2_v4l2_buffer *src_vbuf, *dst_vbuf;
	struct mxc_isi_m2m_buffer *src_buf, *dst_buf;
	unsigned long flags;

	mutex_lock(&m2m->lock);

	/* If the context has changed, reconfigure the channel. */
	if (m2m->last_ctx != ctx) {
		const struct v4l2_area in_size = {
			.width = ctx->queues.out.format.width,
			.height = ctx->queues.out.format.height,
		};
		const struct v4l2_area scale = {
			.width = ctx->queues.cap.format.width,
			.height = ctx->queues.cap.format.height,
		};
		const struct v4l2_rect crop = {
			.width = ctx->queues.cap.format.width,
			.height = ctx->queues.cap.format.height,
		};

		mxc_isi_channel_disable(m2m->pipe);

		mxc_isi_channel_config(m2m->pipe, MXC_ISI_INPUT_MEM,
				       &in_size, &scale, &crop,
				       ctx->queues.out.info->encoding,
				       ctx->queues.cap.info->encoding);
		mxc_isi_channel_set_input_format(m2m->pipe,
						 ctx->queues.out.info,
						 &ctx->queues.out.format);
		mxc_isi_channel_set_output_format(m2m->pipe,
						  ctx->queues.cap.info,
						  &ctx->queues.cap.format);

		mxc_isi_channel_enable(m2m->pipe);

		m2m->last_ctx = ctx;
	}

	mutex_unlock(&m2m->lock);

	spin_lock_irqsave(&m2m->slock, flags);

	src_vbuf = v4l2_m2m_next_src_buf(ctx->fh.m2m_ctx);
	dst_vbuf = v4l2_m2m_next_dst_buf(ctx->fh.m2m_ctx);

	src_buf = to_isi_m2m_buffer(src_vbuf);
	dst_buf = to_isi_m2m_buffer(dst_vbuf);

	mxc_isi_channel_set_inbuf(m2m->pipe, src_buf->dma_addrs[0]);
	mxc_isi_channel_set_outbuf(m2m->pipe, dst_buf->dma_addrs, MXC_ISI_BUF1);
	mxc_isi_channel_set_outbuf(m2m->pipe, dst_buf->dma_addrs, MXC_ISI_BUF2);

	mxc_isi_channel_m2m_start(m2m->pipe);

	spin_unlock_irqrestore(&m2m->slock, flags);
}

static const struct v4l2_m2m_ops mxc_isi_m2m_ops = {
	.device_run = mxc_isi_m2m_device_run,
};

/* -----------------------------------------------------------------------------
 * videobuf2 queue operations
 */

static int mxc_isi_m2m_vb2_queue_setup(struct vb2_queue *q,
				       unsigned int *num_buffers,
				       unsigned int *num_planes,
				       unsigned int sizes[],
				       struct device *alloc_devs[])
{
	struct mxc_isi_m2m_ctx *ctx = vb2_get_drv_priv(q);
	struct mxc_isi_m2m *m2m = ctx->m2m;
	const struct mxc_isi_m2m_ctx_queue_data *qdata =
		mxc_isi_m2m_ctx_qdata(ctx, q->type);
	unsigned long wh;
	unsigned int i;

	*num_planes = qdata->info->memplanes;

	wh = qdata->format.width * qdata->format.height;

	for (i = 0; i < qdata->info->memplanes; ++i) {
		unsigned int size = wh * qdata->info->depth[i] / 8;

		alloc_devs[i] = m2m->isi->dev;

		if (i > 1)
			size /= qdata->info->hsub * qdata->info->vsub;

		sizes[i] = max_t(u32, size, qdata->format.plane_fmt[i].sizeimage);
	}

	return 0;
}

static int mxc_isi_m2m_vb2_buffer_init(struct vb2_buffer *vb2)
{
	struct vb2_queue *vq = vb2->vb2_queue;
	struct mxc_isi_m2m_buffer *buf = to_isi_m2m_buffer(to_vb2_v4l2_buffer(vb2));
	struct mxc_isi_m2m_ctx *ctx = vb2_get_drv_priv(vb2->vb2_queue);
	const struct mxc_isi_m2m_ctx_queue_data *qdata =
		mxc_isi_m2m_ctx_qdata(ctx, vq->type);
	unsigned int i;

	for (i = 0; i < qdata->info->memplanes; ++i)
		buf->dma_addrs[i] = vb2_dma_contig_plane_dma_addr(vb2, i);

	return 0;
}

static int mxc_isi_m2m_vb2_buffer_prepare(struct vb2_buffer *vb2)
{
	struct vb2_queue *vq = vb2->vb2_queue;
	struct mxc_isi_m2m_ctx *ctx = vb2_get_drv_priv(vq);
	struct mxc_isi_m2m *m2m = ctx->m2m;
	const struct mxc_isi_m2m_ctx_queue_data *qdata =
		mxc_isi_m2m_ctx_qdata(ctx, vq->type);
	unsigned int i;

	for (i = 0; i < qdata->info->memplanes; ++i) {
		unsigned long size = qdata->format.plane_fmt[i].sizeimage;

		if (vb2_plane_size(vb2, i) < size) {
			dev_err(m2m->isi->dev,
				 "User buffer too small (%ld < %ld)\n",
				 vb2_plane_size(vb2, i), size);
			return -EINVAL;
		}

		vb2_set_plane_payload(vb2, i, size);
	}

	return 0;
}

static void mxc_isi_m2m_vb2_buffer_queue(struct vb2_buffer *vb2)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb2);
	struct mxc_isi_m2m_ctx *ctx = vb2_get_drv_priv(vb2->vb2_queue);

	v4l2_m2m_buf_queue(ctx->fh.m2m_ctx, vbuf);
}

static int mxc_isi_m2m_vb2_start_streaming(struct vb2_queue *q,
					   unsigned int count)
{
	struct mxc_isi_m2m_ctx *ctx = vb2_get_drv_priv(q);
	struct mxc_isi_m2m_ctx_queue_data *qdata =
		mxc_isi_m2m_ctx_qdata(ctx, q->type);

	qdata->sequence = 0;

	return 0;
}

static void mxc_isi_m2m_vb2_stop_streaming(struct vb2_queue *q)
{
	struct mxc_isi_m2m_ctx *ctx = vb2_get_drv_priv(q);
	struct mxc_isi_m2m *m2m = ctx->m2m;
	struct vb2_v4l2_buffer *vbuf;
	unsigned long flags;

	spin_lock_irqsave(&m2m->slock, flags);

	for (;;) {
		if (q->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
			vbuf = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx);
		else
			vbuf = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);
		if (!vbuf)
			break;

		v4l2_m2m_buf_done(vbuf, VB2_BUF_STATE_ERROR);
	}

	spin_unlock_irqrestore(&m2m->slock, flags);
}

static const struct vb2_ops mxc_isi_m2m_vb2_qops = {
	.queue_setup		= mxc_isi_m2m_vb2_queue_setup,
	.buf_init		= mxc_isi_m2m_vb2_buffer_init,
	.buf_prepare		= mxc_isi_m2m_vb2_buffer_prepare,
	.buf_queue		= mxc_isi_m2m_vb2_buffer_queue,
	.wait_prepare		= vb2_ops_wait_prepare,
	.wait_finish		= vb2_ops_wait_finish,
	.start_streaming	= mxc_isi_m2m_vb2_start_streaming,
	.stop_streaming		= mxc_isi_m2m_vb2_stop_streaming,
};

static int mxc_isi_m2m_queue_init(void *priv, struct vb2_queue *src_vq,
				  struct vb2_queue *dst_vq)
{
	struct mxc_isi_m2m_ctx *ctx = priv;
	struct mxc_isi_m2m *m2m = ctx->m2m;
	int ret;

	src_vq->type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	src_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	src_vq->drv_priv = ctx;
	src_vq->buf_struct_size = sizeof(struct mxc_isi_m2m_buffer);
	src_vq->ops = &mxc_isi_m2m_vb2_qops;
	src_vq->mem_ops = &vb2_dma_contig_memops;
	src_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	src_vq->lock = &m2m->lock;
	src_vq->dev = m2m->isi->dev;

	ret = vb2_queue_init(src_vq);
	if (ret)
		return ret;

	dst_vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	dst_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	dst_vq->drv_priv = ctx;
	dst_vq->buf_struct_size = sizeof(struct mxc_isi_m2m_buffer);
	dst_vq->ops = &mxc_isi_m2m_vb2_qops;
	dst_vq->mem_ops = &vb2_dma_contig_memops;
	dst_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	dst_vq->lock = &m2m->lock;
	dst_vq->dev = m2m->isi->dev;

	return vb2_queue_init(dst_vq);
}

/* -----------------------------------------------------------------------------
 * V4L2 controls
 */

static inline struct mxc_isi_m2m *ctrl_to_mxc_isi_m2m(struct v4l2_ctrl *ctrl)
{
	return container_of(ctrl->handler, struct mxc_isi_m2m, ctrls.handler);
}

static int mxc_isi_m2m_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct mxc_isi_m2m *m2m = ctrl_to_mxc_isi_m2m(ctrl);
	unsigned long flags;

	spin_lock_irqsave(&m2m->slock, flags);

	switch (ctrl->id) {
	case V4L2_CID_HFLIP:
		m2m->pipe->hflip = ctrl->val;
		break;

	case V4L2_CID_VFLIP:
		m2m->pipe->vflip = ctrl->val;
		break;

	case V4L2_CID_ALPHA_COMPONENT:
		m2m->pipe->alpha = ctrl->val;
		break;
	}

	spin_unlock_irqrestore(&m2m->slock, flags);
	return 0;
}

static const struct v4l2_ctrl_ops mxc_isi_m2m_ctrl_ops = {
	.s_ctrl = mxc_isi_m2m_s_ctrl,
};

static int mxc_isi_m2m_ctrls_create(struct mxc_isi_m2m *m2m)
{
	struct v4l2_ctrl_handler *handler = &m2m->ctrls.handler;
	int ret;

	v4l2_ctrl_handler_init(handler, 3);

	m2m->ctrls.alpha = v4l2_ctrl_new_std(handler, &mxc_isi_m2m_ctrl_ops,
					     V4L2_CID_ALPHA_COMPONENT,
					     0, 255, 1, 0);
	m2m->ctrls.hflip = v4l2_ctrl_new_std(handler, &mxc_isi_m2m_ctrl_ops,
					     V4L2_CID_HFLIP, 0, 1, 1, 0);
	m2m->ctrls.vflip = v4l2_ctrl_new_std(handler, &mxc_isi_m2m_ctrl_ops,
					     V4L2_CID_VFLIP, 0, 1, 1, 0);

	if (handler->error) {
		ret = handler->error;
		v4l2_ctrl_handler_free(handler);
		return ret;
	}

	m2m->vdev.ctrl_handler = handler;

	return 0;
}

static void mxc_isi_m2m_ctrls_delete(struct mxc_isi_m2m *m2m)
{
	v4l2_ctrl_handler_free(&m2m->ctrls.handler);
}

/* -----------------------------------------------------------------------------
 * V4L2 ioctls
 */

static int mxc_isi_m2m_querycap(struct file *file, void *fh,
				struct v4l2_capability *cap)
{
	struct mxc_isi_m2m *m2m = video_drvdata(file);

	strlcpy(cap->driver, MXC_ISI_DRIVER_NAME, sizeof(cap->driver));
	strlcpy(cap->card, MXC_ISI_M2M, sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:%s",
		 dev_name(m2m->isi->dev));
	cap->device_caps = V4L2_CAP_STREAMING | V4L2_CAP_VIDEO_M2M_MPLANE;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;

	return 0;
}

static int mxc_isi_m2m_enum_fmt_vid(struct file *file, void *fh,
				    struct v4l2_fmtdesc *f)
{
	const enum mxc_isi_video_type type =
		f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE ?
		MXC_ISI_VIDEO_OUT : MXC_ISI_VIDEO_CAP;
	const struct mxc_isi_format_info *info;

	info = mxc_isi_format_enum(f->index, type);
	if (!info)
		return -EINVAL;

	f->pixelformat = info->fourcc;

	return 0;
}

static const struct mxc_isi_format_info *
__mxc_isi_m2m_try_fmt_vid(struct mxc_isi_m2m_ctx *ctx,
			  struct v4l2_pix_format_mplane *pix,
			  const enum mxc_isi_video_type type)
{
	if (type == MXC_ISI_VIDEO_OUT) {
		/* TODO: Support larger input widths */
		pix->width = min(pix->width, 2048U);
	} else {
		/* Downscaling only */
		pix->width = min(pix->width, ctx->queues.out.format.width);
		pix->height = min(pix->height, ctx->queues.out.format.height);
	}

	return mxc_isi_format_try(pix, type);
}

static int mxc_isi_m2m_try_fmt_vid(struct file *file, void *fh,
				   struct v4l2_format *f)
{
	const enum mxc_isi_video_type type =
		f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE ?
		MXC_ISI_VIDEO_OUT : MXC_ISI_VIDEO_CAP;
	struct mxc_isi_m2m_ctx *ctx = to_isi_m2m_ctx(fh);

	__mxc_isi_m2m_try_fmt_vid(ctx, &f->fmt.pix_mp, type);

	return 0;
}

static int mxc_isi_m2m_g_fmt_vid(struct file *file, void *fh,
				 struct v4l2_format *f)
{
	struct mxc_isi_m2m_ctx *ctx = to_isi_m2m_ctx(fh);
	const struct mxc_isi_m2m_ctx_queue_data *qdata =
		mxc_isi_m2m_ctx_qdata(ctx, f->type);

	f->fmt.pix_mp = qdata->format;

	return 0;
}

static int mxc_isi_m2m_s_fmt_vid(struct file *file, void *fh,
				 struct v4l2_format *f)
{
	const enum mxc_isi_video_type type =
		f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE ?
		MXC_ISI_VIDEO_OUT : MXC_ISI_VIDEO_CAP;
	struct mxc_isi_m2m_ctx *ctx = to_isi_m2m_ctx(fh);
	struct v4l2_pix_format_mplane *pix = &f->fmt.pix_mp;
	const struct mxc_isi_format_info *info;
	struct vb2_queue *vq;

	vq = v4l2_m2m_get_vq(ctx->fh.m2m_ctx, f->type);
	if (!vq)
		return -EINVAL;

	if (vb2_is_busy(vq))
		return -EBUSY;

	info = __mxc_isi_m2m_try_fmt_vid(ctx, pix, type);

	if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		ctx->queues.out.format = *pix;
		ctx->queues.out.info = info;
	}

	/*
	 * Always set the format on the capture side, due to either format
	 * propagation or direct setting.
	 */
	ctx->queues.cap.format = *pix;
	ctx->queues.cap.info = info;

	return 0;
}

static int mxc_isi_m2m_streamon(struct file *file, void *fh,
				enum v4l2_buf_type type)
{
	struct mxc_isi_m2m_ctx *ctx = to_isi_m2m_ctx(fh);
	struct mxc_isi_m2m *m2m = ctx->m2m;
	int ret;

	mutex_lock(&m2m->lock);

	if (m2m->usage_count == INT_MAX) {
		ret = -EOVERFLOW;
		goto unlock;
	}

	/* Initialize the channel with the first user of the M2M device. */
	if (m2m->usage_count == 0)
		mxc_isi_channel_init(m2m->pipe);

	ret = v4l2_m2m_ioctl_streamon(file, fh, type);
	if (ret) {
		if (m2m->usage_count == 0)
			mxc_isi_channel_deinit(m2m->pipe);
		goto unlock;
	}

	m2m->usage_count++;

unlock:
	mutex_unlock(&m2m->lock);
	return ret;
}

static int mxc_isi_m2m_streamoff(struct file *file, void *fh,
				 enum v4l2_buf_type type)
{
	struct mxc_isi_m2m_ctx *ctx = to_isi_m2m_ctx(fh);
	struct mxc_isi_m2m *m2m = ctx->m2m;

	mutex_lock(&m2m->lock);

	v4l2_m2m_ioctl_streamoff(file, fh, type);

	/*
	 * If the last context is this one, reset it to make sure the device
	 * will be reconfigured when streaming is restarted.
	 */
	if (m2m->last_ctx == ctx)
		m2m->last_ctx = NULL;

	/* Turn off the light with the last user. */
	if (--m2m->usage_count == 0) {
		mxc_isi_channel_disable(m2m->pipe);
		mxc_isi_channel_deinit(m2m->pipe);
	}

	WARN_ON(m2m->usage_count < 0);

	mutex_unlock(&m2m->lock);

	return 0;
}

static const struct v4l2_ioctl_ops mxc_isi_m2m_ioctl_ops = {
	.vidioc_querycap		= mxc_isi_m2m_querycap,

	.vidioc_enum_fmt_vid_cap	= mxc_isi_m2m_enum_fmt_vid,
	.vidioc_enum_fmt_vid_out	= mxc_isi_m2m_enum_fmt_vid,
	.vidioc_g_fmt_vid_cap_mplane	= mxc_isi_m2m_g_fmt_vid,
	.vidioc_g_fmt_vid_out_mplane	= mxc_isi_m2m_g_fmt_vid,
	.vidioc_s_fmt_vid_cap_mplane	= mxc_isi_m2m_s_fmt_vid,
	.vidioc_s_fmt_vid_out_mplane	= mxc_isi_m2m_s_fmt_vid,
	.vidioc_try_fmt_vid_cap_mplane	= mxc_isi_m2m_try_fmt_vid,
	.vidioc_try_fmt_vid_out_mplane	= mxc_isi_m2m_try_fmt_vid,

	.vidioc_reqbufs			= v4l2_m2m_ioctl_reqbufs,
	.vidioc_querybuf		= v4l2_m2m_ioctl_querybuf,
	.vidioc_qbuf			= v4l2_m2m_ioctl_qbuf,
	.vidioc_dqbuf			= v4l2_m2m_ioctl_dqbuf,
	.vidioc_expbuf			= v4l2_m2m_ioctl_expbuf,
	.vidioc_prepare_buf		= v4l2_m2m_ioctl_prepare_buf,
	.vidioc_create_bufs		= v4l2_m2m_ioctl_create_bufs,

	.vidioc_streamon		= mxc_isi_m2m_streamon,
	.vidioc_streamoff		= mxc_isi_m2m_streamoff,
};

/* -----------------------------------------------------------------------------
 * Video device file operations
 */

static void mxc_isi_m2m_init_format(struct mxc_isi_m2m_ctx_queue_data *qdata,
				    enum mxc_isi_video_type type)
{
	qdata->format.width = MXC_ISI_DEF_WIDTH;
	qdata->format.height = MXC_ISI_DEF_HEIGHT;
	qdata->format.pixelformat = MXC_ISI_DEF_PIXEL_FORMAT;

	qdata->info = mxc_isi_format_try(&qdata->format, type);
}

static int mxc_isi_m2m_open(struct file *file)
{
	struct video_device *vdev = video_devdata(file);
	struct mxc_isi_m2m *m2m = video_drvdata(file);
	struct mxc_isi_m2m_ctx *ctx;
	int ret;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->m2m = m2m;
	v4l2_fh_init(&ctx->fh, vdev);
	file->private_data = &ctx->fh;

	ctx->fh.m2m_ctx = v4l2_m2m_ctx_init(m2m->m2m_dev, ctx,
					    &mxc_isi_m2m_queue_init);
	if (IS_ERR(ctx->fh.m2m_ctx)) {
		ret = PTR_ERR(ctx->fh.m2m_ctx);
		ctx->fh.m2m_ctx = NULL;
		goto error;
	}

	mxc_isi_m2m_init_format(&ctx->queues.out, MXC_ISI_VIDEO_OUT);
	mxc_isi_m2m_init_format(&ctx->queues.cap, MXC_ISI_VIDEO_CAP);

	v4l2_fh_add(&ctx->fh);

	ret = pm_runtime_resume_and_get(m2m->isi->dev);
	if (ret)
		goto error;

	return 0;

error:
	if (ctx->fh.m2m_ctx)
		v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
	v4l2_fh_exit(&ctx->fh);
	kfree(ctx);
	return ret;
}

static int mxc_isi_m2m_release(struct file *file)
{
	struct mxc_isi_m2m *m2m = video_drvdata(file);
	struct mxc_isi_m2m_ctx *ctx = to_isi_m2m_ctx(file->private_data);

	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);

	mutex_lock(&m2m->lock);
	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
	mutex_unlock(&m2m->lock);

	kfree(ctx);

	pm_runtime_put(m2m->isi->dev);

	return 0;
}

static const struct v4l2_file_operations mxc_isi_m2m_fops = {
	.owner		= THIS_MODULE,
	.open		= mxc_isi_m2m_open,
	.release	= mxc_isi_m2m_release,
	.poll		= v4l2_m2m_fop_poll,
	.unlocked_ioctl	= video_ioctl2,
	.mmap		= v4l2_m2m_fop_mmap,
};

/* -----------------------------------------------------------------------------
 * Registration
 */

int mxc_isi_m2m_register(struct mxc_isi_dev *isi, struct v4l2_device *v4l2_dev)
{
	struct mxc_isi_m2m *m2m = &isi->m2m;
	struct video_device *vdev = &m2m->vdev;
	int ret;

	m2m->isi = isi;
	m2m->pipe = &isi->pipes[0];

	spin_lock_init(&m2m->slock);
	mutex_init(&m2m->lock);

	/* Initialize the media entity. */
	m2m->pads[0].flags = MEDIA_PAD_FL_SINK;
	m2m->pads[1].flags = MEDIA_PAD_FL_SOURCE;
	vdev->entity.function = MEDIA_ENT_F_PROC_VIDEO_SCALER;
	ret = media_entity_pads_init(&vdev->entity, 2, m2m->pads);
	if (ret)
		goto err_mutex;

	/* Initialize the video device and create controls. */
	snprintf(vdev->name, sizeof(vdev->name), "mxc_isi.m2m");

	vdev->fops	= &mxc_isi_m2m_fops;
	vdev->ioctl_ops	= &mxc_isi_m2m_ioctl_ops;
	vdev->v4l2_dev	= v4l2_dev;
	vdev->minor	= -1;
	vdev->release	= video_device_release_empty;
	vdev->vfl_dir	= VFL_DIR_M2M;

	vdev->device_caps = V4L2_CAP_STREAMING | V4L2_CAP_VIDEO_M2M_MPLANE;
	video_set_drvdata(vdev, m2m);

	ret = mxc_isi_m2m_ctrls_create(m2m);
	if (ret)
		goto err_entity;

	/* Create the M2M device. */
	m2m->m2m_dev = v4l2_m2m_init(&mxc_isi_m2m_ops);
	if (IS_ERR(m2m->m2m_dev)) {
		dev_err(isi->dev, "failed to initialize m2m device\n");
		ret = PTR_ERR(m2m->m2m_dev);
		goto err_ctrls;
	}

	/* Register the video device. */
	ret = video_register_device(vdev, VFL_TYPE_VIDEO, -1);
	if (ret < 0) {
		dev_err(isi->dev, "failed to register m2m device\n");
		goto err_m2m;
	}

	/* Create links. */
	ret = media_create_pad_link(&m2m->pipe->sd.entity,
				    MXC_ISI_PIPE_PAD_SOURCE,
				    &vdev->entity, 0,
				    MEDIA_LNK_FL_IMMUTABLE);
	if (ret)
		goto err_video;

	ret = media_create_pad_link(&vdev->entity, 0,
				    &m2m->isi->crossbar.sd.entity,
				    m2m->isi->crossbar.num_sinks - 1,
				    MEDIA_LNK_FL_IMMUTABLE |
				    MEDIA_LNK_FL_ENABLED);
	if (ret)
		goto err_video;

	return 0;

err_video:
	video_unregister_device(vdev);
err_m2m:
	v4l2_m2m_release(m2m->m2m_dev);
err_ctrls:
	mxc_isi_m2m_ctrls_delete(m2m);
err_entity:
	media_entity_cleanup(&vdev->entity);
err_mutex:
	mutex_destroy(&m2m->lock);
	return ret;
}

int mxc_isi_m2m_unregister(struct mxc_isi_dev *isi)
{
	struct mxc_isi_m2m *m2m = &isi->m2m;
	struct video_device *vdev = &m2m->vdev;

	video_unregister_device(vdev);

	v4l2_m2m_release(m2m->m2m_dev);
	mxc_isi_m2m_ctrls_delete(m2m);
	media_entity_cleanup(&vdev->entity);
	mutex_destroy(&m2m->lock);

	return 0;
}
