// SPDX-License-Identifier: GPL-2.0
/*
 * ISI V4L2 memory to memory driver for i.MX8QXP/QM platform
 *
 * ISI is a Image Sensor Interface of i.MX8QXP/QM platform, which
 * used to process image from camera sensor or memory to memory or DC
 *
 * Copyright (c) 2019 NXP Semiconductor
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/bug.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/pm_runtime.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/of_graph.h>
#include <linux/videodev2.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-dma-contig.h>

#include "imx8-isi-hw.h"
#include "imx8-common.h"

struct mxc_isi_m2m_dev {
	struct mxc_isi_dev *isi;

	struct video_device vdev;
	struct v4l2_m2m_dev *m2m_dev;

	struct list_head	out_active;
	struct mxc_isi_ctrls	ctrls;

	struct mutex lock;
	spinlock_t   slock;

	unsigned int aborting;
	unsigned int frame_count;

	u8 id;
};

struct mxc_isi_ctx_format {
	struct v4l2_pix_format_mplane format;
	const struct mxc_isi_format_info *info;
};

struct mxc_isi_ctx {
	struct v4l2_fh fh;
	struct mxc_isi_m2m_dev *m2m;

	struct {
		struct mxc_isi_ctx_format out;
		struct mxc_isi_ctx_format cap;
	} formats;
};

static inline struct mxc_isi_ctx *to_isi_ctx(struct v4l2_fh *fh)
{
	return container_of(fh, struct mxc_isi_ctx, fh);
}

static void mxc_isi_channel_set_m2m_src_addr(struct mxc_isi_dev *mxc_isi,
					     struct mxc_isi_buffer *buf)
{
	struct vb2_buffer *vb2_buf = &buf->v4l2_buf.vb2_buf;
	struct frame_addr *paddr = &buf->paddr;

	/* Only support one plane */
	paddr->y = vb2_dma_contig_plane_dma_addr(vb2_buf, 0);
	writel(paddr->y, mxc_isi->regs + CHNL_IN_BUF_ADDR);
}

static void mxc_isi_m2m_config_src(struct mxc_isi_dev *mxc_isi,
				   struct mxc_isi_frame *src_f)
{
	u32 val;

	/* source format */
	val = readl(mxc_isi->regs + CHNL_MEM_RD_CTRL);
	val &= ~CHNL_MEM_RD_CTRL_IMG_TYPE_MASK;
	val |= src_f->fmt->color << CHNL_MEM_RD_CTRL_IMG_TYPE_OFFSET;
	writel(val, mxc_isi->regs + CHNL_MEM_RD_CTRL);

	/* source image width and height */
	val = (src_f->width << CHNL_IMG_CFG_WIDTH_OFFSET |
	       src_f->height << CHNL_IMG_CFG_HEIGHT_OFFSET);
	writel(val, mxc_isi->regs + CHNL_IMG_CFG);

	/* source pitch */
	val = src_f->bytesperline[0] << CHNL_IN_BUF_PITCH_LINE_PITCH_OFFSET;
	writel(val, mxc_isi->regs + CHNL_IN_BUF_PITCH);
}

static void mxc_isi_m2m_config_dst(struct mxc_isi_dev *mxc_isi,
				   struct mxc_isi_frame *dst_f)
{
	u32 val;

	/* out format */
	val = readl(mxc_isi->regs + CHNL_IMG_CTRL);
	val &= ~CHNL_IMG_CTRL_FORMAT_MASK;
	val |= dst_f->fmt->color << CHNL_IMG_CTRL_FORMAT_OFFSET;
	writel(val, mxc_isi->regs + CHNL_IMG_CTRL);

	/* out pitch */
	val = readl(mxc_isi->regs + CHNL_OUT_BUF_PITCH);
	val &= ~CHNL_IN_BUF_PITCH_LINE_PITCH_MASK;
	val |= dst_f->bytesperline[0] << CHNL_OUT_BUF_PITCH_LINE_PITCH_OFFSET;
	writel(val, mxc_isi->regs + CHNL_OUT_BUF_PITCH);
}

static void mxc_isi_m2m_start_read(struct mxc_isi_dev *mxc_isi)
{
	u32 val;

	val = readl(mxc_isi->regs + CHNL_MEM_RD_CTRL);
	val &= ~ CHNL_MEM_RD_CTRL_READ_MEM_MASK;
	writel(val, mxc_isi->regs + CHNL_MEM_RD_CTRL);
	udelay(300);

	val |= CHNL_MEM_RD_CTRL_READ_MEM_ENABLE << CHNL_MEM_RD_CTRL_READ_MEM_OFFSET;
	writel(val, mxc_isi->regs + CHNL_MEM_RD_CTRL);
}

#define to_isi_buffer(x)	\
	container_of((x), struct mxc_isi_buffer, v4l2_buf)

static struct v4l2_m2m_buffer *to_v4l2_m2m_buffer(struct vb2_v4l2_buffer *vbuf)
{
	struct v4l2_m2m_buffer *b;

	b = container_of(vbuf, struct v4l2_m2m_buffer, vb);
	return b;
}

/* -----------------------------------------------------------------------------
 * V4L2 M2M device operations
 */

static void mxc_isi_m2m_frame_write_done(struct mxc_isi_dev *mxc_isi)
{
	struct mxc_isi_m2m_dev *m2m = mxc_isi->m2m;
	struct v4l2_fh *fh;
	struct mxc_isi_ctx *curr_ctx;
	struct vb2_v4l2_buffer *src_vbuf, *dst_vbuf;
	struct mxc_isi_buffer *src_buf, *dst_buf;
	struct v4l2_m2m_buffer *b;

	dev_dbg(m2m->isi->dev, "%s\n", __func__);

	curr_ctx = v4l2_m2m_get_curr_priv(m2m->m2m_dev);
	if (!curr_ctx) {
		dev_err(m2m->isi->dev,
			"Instance released before the end of transaction\n");
		return;
	}
	fh = &curr_ctx->fh;

	if (m2m->aborting) {
		mxc_isi_channel_disable(mxc_isi);
		dev_warn(m2m->isi->dev, "Aborting current job\n");
		goto job_finish;
	}

	src_vbuf = v4l2_m2m_next_src_buf(fh->m2m_ctx);
	if (!src_vbuf) {
		dev_err(m2m->isi->dev, "No enought source buffers\n");
		goto job_finish;
	}
	src_buf = to_isi_buffer(src_vbuf);
	v4l2_m2m_src_buf_remove(fh->m2m_ctx);
	v4l2_m2m_buf_done(src_vbuf, VB2_BUF_STATE_DONE);

	if (!list_empty(&m2m->out_active)) {
		dst_buf = list_first_entry(&m2m->out_active,
					   struct mxc_isi_buffer, list);
		dst_vbuf = &dst_buf->v4l2_buf;
		list_del_init(&dst_buf->list);
		dst_buf->v4l2_buf.vb2_buf.timestamp = ktime_get_ns();
		v4l2_m2m_buf_done(dst_vbuf, VB2_BUF_STATE_DONE);

	}
	m2m->frame_count++;

	dst_vbuf = v4l2_m2m_next_dst_buf(fh->m2m_ctx);
	if (dst_vbuf) {
		dst_vbuf->vb2_buf.state = VB2_BUF_STATE_ACTIVE;
		dst_buf = to_isi_buffer(dst_vbuf);
		dst_buf->v4l2_buf.sequence = m2m->frame_count;
		mxc_isi_channel_set_outbuf(mxc_isi, dst_buf);
		v4l2_m2m_dst_buf_remove(fh->m2m_ctx);
		b = to_v4l2_m2m_buffer(dst_vbuf);
		list_add_tail(&b->list, &m2m->out_active);
	}

job_finish:
	v4l2_m2m_job_finish(m2m->m2m_dev, fh->m2m_ctx);
}

static void mxc_isi_m2m_device_run(void *priv)
{
	struct mxc_isi_ctx *ctx = priv;
	struct mxc_isi_m2m_dev *m2m = ctx->m2m;
	struct mxc_isi_dev *mxc_isi = m2m->isi;
	struct v4l2_fh *fh = &ctx->fh;
	struct vb2_v4l2_buffer *vbuf;
	struct mxc_isi_buffer *src_buf;
	unsigned long flags;

	dev_dbg(m2m->isi->dev, "%s enter\n", __func__);

	spin_lock_irqsave(&m2m->slock, flags);

	/* SRC */
	vbuf = v4l2_m2m_next_src_buf(fh->m2m_ctx);
	if (!vbuf) {
		dev_err(m2m->isi->dev, "Null src buf\n");
		goto unlock;
	}

	src_buf = to_isi_buffer(vbuf);
	mxc_isi_channel_set_m2m_src_addr(mxc_isi, src_buf);
	mxc_isi_channel_enable(mxc_isi, true);

unlock:
	spin_unlock_irqrestore(&m2m->slock, flags);
}

static int mxc_isi_m2m_job_ready(void *priv)
{
	struct mxc_isi_ctx *ctx = priv;
	struct mxc_isi_m2m_dev *m2m = ctx->m2m;
	struct v4l2_fh *fh = &ctx->fh;
	unsigned int num_src_bufs_ready;
	unsigned int num_dst_bufs_ready;
	unsigned long flags;

	dev_dbg(m2m->isi->dev, "%s\n", __func__);

	spin_lock_irqsave(&m2m->slock, flags);
	num_src_bufs_ready = v4l2_m2m_num_src_bufs_ready(fh->m2m_ctx);
	num_dst_bufs_ready = v4l2_m2m_num_dst_bufs_ready(fh->m2m_ctx);
	spin_unlock_irqrestore(&m2m->slock, flags);

	if (num_src_bufs_ready >= 1 && num_dst_bufs_ready >= 1)
		return 1;
	return 0;
}

static void mxc_isi_m2m_job_abort(void *priv)
{
	struct mxc_isi_ctx *ctx = priv;
	struct mxc_isi_m2m_dev *m2m = ctx->m2m;

	m2m->aborting = 1;
	dev_dbg(m2m->isi->dev, "Abort requested\n");
}

static struct v4l2_m2m_ops mxc_isi_m2m_ops = {
	.device_run = mxc_isi_m2m_device_run,
	.job_ready  = mxc_isi_m2m_job_ready,
	.job_abort  = mxc_isi_m2m_job_abort,
};

/* -----------------------------------------------------------------------------
 * videobuf2 queue operations
 */

static int m2m_vb2_queue_setup(struct vb2_queue *q,
		unsigned int *num_buffers, unsigned int *num_planes,
		unsigned int sizes[], struct device *alloc_devs[])
{
	struct mxc_isi_ctx *ctx = vb2_get_drv_priv(q);
	struct mxc_isi_m2m_dev *m2m = ctx->m2m;
	struct device *dev = m2m->isi->dev;
	struct mxc_isi_frame *frame;
	struct mxc_isi_fmt *fmt;
	unsigned long wh;
	int i;

	dev_dbg(m2m->isi->dev, "%s\n", __func__);

	if (q->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		if (*num_buffers < 3) {
			dev_err(dev, "%s at least need 3 buffer\n", __func__);
			return -EINVAL;
		}
		frame = &m2m->dst_f;
	} else {
		if (*num_buffers < 1) {
			dev_err(dev, "%s at least need one buffer\n", __func__);
			return -EINVAL;
		}
		frame = &m2m->src_f;
	}

	fmt = frame->fmt;
	if (fmt == NULL)
		return -EINVAL;

	for (i = 0; i < fmt->memplanes; i++)
		alloc_devs[i] = m2m->isi->dev;

	*num_planes = fmt->memplanes;
	wh = frame->width * frame->height;

	for (i = 0; i < fmt->memplanes; i++) {
		unsigned int size = (wh * fmt->depth[i]) >> 3;

		if (i == 1 && fmt->fourcc == V4L2_PIX_FMT_NV12)
			size >>= 1;
		sizes[i] = max_t(u32, size, frame->sizeimage[i]);

		dev_dbg(m2m->isi->dev, "%s, buf_n=%d, planes[%d]->size=%d\n",
					__func__,  *num_buffers, i, sizes[i]);
	}

	return 0;
}

static int m2m_vb2_buffer_prepare(struct vb2_buffer *vb2)
{
	struct vb2_queue *vq = vb2->vb2_queue;
	struct mxc_isi_ctx *ctx = vb2_get_drv_priv(vq);
	struct mxc_isi_m2m_dev *m2m = ctx->m2m;
	struct mxc_isi_frame *frame;
	int i;

	if (vq->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		frame = &m2m->dst_f;
	else
		frame = &m2m->src_f;

	if (frame == NULL)
		return -EINVAL;

	for (i = 0; i < frame->fmt->memplanes; i++) {
		unsigned long size = frame->sizeimage[i];

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

static void m2m_vb2_buffer_queue(struct vb2_buffer *vb2)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb2);
	struct mxc_isi_ctx *ctx = vb2_get_drv_priv(vb2->vb2_queue);
	struct v4l2_fh *fh = &ctx->fh;

	v4l2_m2m_buf_queue(fh->m2m_ctx, vbuf);
}

static int m2m_vb2_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct mxc_isi_ctx *ctx = vb2_get_drv_priv(q);
	struct mxc_isi_m2m_dev *m2m = ctx->m2m;
	struct mxc_isi_dev *mxc_isi = m2m->isi;
	struct v4l2_fh *fh = &ctx->fh;
	struct vb2_v4l2_buffer *dst_vbuf;
	struct  v4l2_m2m_buffer *b;
	struct mxc_isi_buffer *dst_buf;
	unsigned long flags;

	if (V4L2_TYPE_IS_OUTPUT(q->type))
		return 0;

	if (count < 2) {
		dev_err(m2m->isi->dev, "Need to at leas 2 buffers\n");
		return -EINVAL;
	}

	spin_lock_irqsave(&m2m->slock, flags);

	/* BUF1 */
	dst_vbuf = v4l2_m2m_next_dst_buf(fh->m2m_ctx);
	if (!dst_vbuf) {
		dev_err(m2m->isi->dev, "%d: Null dst buf\n", __LINE__);
		goto unlock;
	}
	dst_vbuf->vb2_buf.state = VB2_BUF_STATE_ACTIVE;
	dst_buf = to_isi_buffer(dst_vbuf);
	dst_buf->v4l2_buf.sequence = 0;
	mxc_isi_channel_set_outbuf(mxc_isi, dst_buf);
	v4l2_m2m_dst_buf_remove(fh->m2m_ctx);
	b = to_v4l2_m2m_buffer(dst_vbuf);
	list_add_tail(&b->list, &m2m->out_active);

	/* BUF2 */
	dst_vbuf = v4l2_m2m_next_dst_buf(fh->m2m_ctx);
	if (!dst_vbuf) {
		dev_err(m2m->isi->dev, "%d: Null dst buf\n", __LINE__);
		goto unlock;
	}
	dst_vbuf->vb2_buf.state = VB2_BUF_STATE_ACTIVE;
	dst_buf = to_isi_buffer(dst_vbuf);
	dst_buf->v4l2_buf.sequence = 1;
	mxc_isi_channel_set_outbuf(mxc_isi, dst_buf);
	v4l2_m2m_dst_buf_remove(fh->m2m_ctx);
	b = to_v4l2_m2m_buffer(dst_vbuf);
	list_add_tail(&b->list, &m2m->out_active);

	m2m->frame_count = 1;
	m2m->aborting = 0;
unlock:
	spin_unlock_irqrestore(&m2m->slock, flags);

	return 0;
}

static void m2m_vb2_stop_streaming(struct vb2_queue *q)
{
	struct mxc_isi_ctx *ctx = vb2_get_drv_priv(q);
	struct mxc_isi_m2m_dev *m2m = ctx->m2m;
	struct vb2_v4l2_buffer *vb2;
	struct mxc_isi_buffer *buf;
	unsigned long flags;

	spin_lock_irqsave(&m2m->slock, flags);

	while ((vb2 = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx)) != NULL)
		v4l2_m2m_buf_done(vb2, VB2_BUF_STATE_ERROR);

	while ((vb2 = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx)) != NULL)
		v4l2_m2m_buf_done(vb2, VB2_BUF_STATE_ERROR);

	while (!list_empty(&m2m->out_active)) {
		buf = list_entry(m2m->out_active.next, struct mxc_isi_buffer, list);
		list_del(&buf->list);
		vb2_buffer_done(&buf->v4l2_buf.vb2_buf, VB2_BUF_STATE_ERROR);
	}

	INIT_LIST_HEAD(&m2m->out_active);

	spin_unlock_irqrestore(&m2m->slock, flags);
}

static struct vb2_ops mxc_m2m_vb2_qops = {
	.queue_setup		= m2m_vb2_queue_setup,
	.buf_prepare		= m2m_vb2_buffer_prepare,
	.buf_queue		= m2m_vb2_buffer_queue,
	.wait_prepare		= vb2_ops_wait_prepare,
	.wait_finish		= vb2_ops_wait_finish,
	.start_streaming	= m2m_vb2_start_streaming,
	.stop_streaming		= m2m_vb2_stop_streaming,
};

static int mxc_m2m_queue_init(void *priv, struct vb2_queue *src_vq,
			      struct vb2_queue *dst_vq)
{
	struct mxc_isi_ctx *ctx = priv;
	struct mxc_isi_m2m_dev *m2m = ctx->m2m;
	int ret;

	src_vq->type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	src_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	src_vq->drv_priv = ctx;
	src_vq->buf_struct_size = sizeof(struct mxc_isi_buffer);
	src_vq->ops = &mxc_m2m_vb2_qops;
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
	dst_vq->buf_struct_size = sizeof(struct mxc_isi_buffer);
	dst_vq->ops = &mxc_m2m_vb2_qops;
	dst_vq->mem_ops = &vb2_dma_contig_memops;
	dst_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	dst_vq->lock = &m2m->lock;
	dst_vq->dev = m2m->isi->dev;

	return vb2_queue_init(dst_vq);
}

/* -----------------------------------------------------------------------------
 * V4L2 controls
 */

#define ctrl_to_mxc_isi_m2m(__ctrl) \
	container_of((__ctrl)->handler, struct mxc_isi_m2m_dev, ctrls.handler)

static int mxc_isi_m2m_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct mxc_isi_m2m_dev *m2m = ctrl_to_mxc_isi_m2m(ctrl);
	struct mxc_isi_dev *mxc_isi = m2m->isi;
	unsigned long flags;

	spin_lock_irqsave(&mxc_isi->slock, flags);

	switch (ctrl->id) {
	case V4L2_CID_HFLIP:
		mxc_isi->hflip = ctrl->val;
		break;

	case V4L2_CID_VFLIP:
		mxc_isi->vflip = ctrl->val;
		break;

	case V4L2_CID_ALPHA_COMPONENT:
		mxc_isi->alpha = ctrl->val;
		break;
	}

	spin_unlock_irqrestore(&mxc_isi->slock, flags);
	return 0;
}

static const struct v4l2_ctrl_ops mxc_isi_m2m_ctrl_ops = {
	.s_ctrl = mxc_isi_m2m_s_ctrl,
};

static int mxc_isi_m2m_ctrls_create(struct mxc_isi_m2m_dev *m2m)
{
	struct mxc_isi_ctrls *ctrls = &m2m->ctrls;
	struct v4l2_ctrl_handler *handler = &ctrls->handler;

	if (m2m->ctrls.ready)
		return 0;

	v4l2_ctrl_handler_init(handler, 3);

	ctrls->hflip = v4l2_ctrl_new_std(handler, &mxc_isi_m2m_ctrl_ops,
					V4L2_CID_HFLIP, 0, 1, 1, 0);
	ctrls->vflip = v4l2_ctrl_new_std(handler, &mxc_isi_m2m_ctrl_ops,
					V4L2_CID_VFLIP, 0, 1, 1, 0);
	ctrls->alpha = v4l2_ctrl_new_std(handler, &mxc_isi_m2m_ctrl_ops,
					V4L2_CID_ALPHA_COMPONENT, 0, 0xff, 1, 0);

	if (!handler->error)
		ctrls->ready = true;

	return handler->error;
}

static void mxc_isi_m2m_ctrls_delete(struct mxc_isi_m2m_dev *m2m)
{
	struct mxc_isi_ctrls *ctrls = &m2m->ctrls;

	if (ctrls->ready) {
		v4l2_ctrl_handler_free(&ctrls->handler);
		ctrls->ready = false;
		ctrls->alpha = NULL;
	}
}

/* -----------------------------------------------------------------------------
 * V4L2 ioctls
 */

static int mxc_isi_m2m_querycap(struct file *file, void *priv,
					struct v4l2_capability *cap)
{
	struct mxc_isi_m2m_dev *m2m = video_drvdata(file);

	strlcpy(cap->driver, MXC_ISI_M2M, sizeof(cap->driver));
	strlcpy(cap->card, MXC_ISI_M2M, sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:%s.%d",
		 dev_name(m2m->isi->dev), m2m->id);
	cap->device_caps = V4L2_CAP_STREAMING | V4L2_CAP_VIDEO_M2M_MPLANE;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;

	return 0;
}

static int mxc_isi_m2m_enum_fmt_vid(struct file *file, void *priv,
				    struct v4l2_fmtdesc *f)
{
	const enum mxc_isi_video_type type =
		f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE ?
		MXC_ISI_VIDEO_OUT : MXC_ISI_VIDEO_CAP;
	struct mxc_isi_format_info *info;

	info = mxc_isi_format_enum(f->index, type);
	if (!info)
		return -EINVAL;

	f->pixelformat = info->fourcc;

	return 0;
}

static int mxc_isi_m2m_try_fmt_vid_out(struct file *file, void *fh,
				       struct v4l2_format *f)
{
	struct v4l2_pix_format_mplane *pix = &f->fmt.pix_mp;

	/* TODO: Support larger input widths */
	pix->width = min(pix->width, 2048U);

	mxc_isi_format_try(pix, MXC_ISI_VIDEO_OUT);

	return 0;
}

static int mxc_isi_m2m_try_fmt_vid_cap(struct file *file, void *fh,
				   struct v4l2_format *f)
{
	struct mxc_isi_m2m_dev *m2m = video_drvdata(file);
	struct device *dev = m2m->isi->dev;
	struct v4l2_pix_format_mplane *pix = &f->fmt.pix_mp;
	struct mxc_isi_format_info *info;

	/* Downscaling only */
	pix->width = min(pix->width, ctx->formats.out.width);
	pix->height = min(pix->height, ctx->formats.out.height);

	mxc_isi_format_try(pix, MXC_ISI_VIDEO_CAP);

	return 0;
}

static int mxc_isi_m2m_g_fmt_vid(struct file *file, void *fh,
				 struct v4l2_format *f)
{
	struct mxc_isi_ctx *ctx = to_isi_ctx(fh);

	if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		f->fmt.pix_fmt = ctx->formats.out;
	else
		f->fmt.pix_fmt = ctx->formats.cap;

	return 0;
}

static int mxc_isi_m2m_s_fmt_vid(struct file *file, void *fh,
				 struct v4l2_format *f)
{
	struct mxc_isi_ctx *ctx = to_isi_ctx(fh);
	struct v4l2_pix_format_mplane *pix = &f->fmt.pix_mp;
	struct vb2_queue *vq;

	vq = v4l2_m2m_get_vq(fh->m2m_ctx, f->type);
	if (!vq)
		return -EINVAL;

	if (vb2_is_busy(vq))
		return -EBUSY;

	mxc_isi_m2m_try_fmt_vid_out(file, fh, f);

	if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		ctx->formats.out = *pix;

	/*
	 * Always set the format on the capture side, due to either format
	 * propagation or direct setting.
	 */
	ctx->formats.cap = *pix;

	return 0;
}

static int mxc_isi_m2m_streamon(struct file *file, void *priv,
			     enum v4l2_buf_type type)
{
	const enum mxc_isi_video_type type =
		f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE ?
		MXC_ISI_VIDEO_OUT : MXC_ISI_VIDEO_CAP;
	struct mxc_isi_m2m_dev *m2m = video_drvdata(file);
	struct mxc_isi_dev *mxc_isi = m2m->isi;
	int ret;

	if (type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		m2m->frame_count = 0;
		mxc_isi_channel_config(mxc_isi, src_f, dst_f);
	}

	ret = v4l2_m2m_ioctl_streamon(file, priv, type);

	return ret;
}

static int mxc_isi_m2m_streamoff(struct file *file, void *priv,
			    enum v4l2_buf_type type)
{
	struct mxc_isi_m2m_dev *m2m = video_drvdata(file);
	struct mxc_isi_dev *mxc_isi =  m2m->isi;
	int ret;

	if (type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		mxc_isi_channel_disable(mxc_isi);

	ret = v4l2_m2m_ioctl_streamoff(file, priv, type);

	return ret;
}

static const struct v4l2_ioctl_ops mxc_isi_m2m_ioctl_ops = {
	.vidioc_querycap		= mxc_isi_m2m_querycap,

	.vidioc_enum_fmt_vid_cap	= mxc_isi_m2m_enum_fmt_vid,
	.vidioc_enum_fmt_vid_out	= mxc_isi_m2m_enum_fmt_vid,
	.vidioc_g_fmt_vid_cap_mplane	= mxc_isi_m2m_g_fmt_vid,
	.vidioc_g_fmt_vid_out_mplane	= mxc_isi_m2m_g_fmt_vid,
	.vidioc_s_fmt_vid_cap_mplane	= mxc_isi_m2m_s_fmt_vid,
	.vidioc_s_fmt_vid_out_mplane	= mxc_isi_m2m_s_fmt_vid,
	.vidioc_try_fmt_vid_cap_mplane	= mxc_isi_m2m_try_fmt_vid_cap,
	.vidioc_try_fmt_vid_out_mplane	= mxc_isi_m2m_try_fmt_vid_out,

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

static void mxc_isi_m2m_init_format(struct mxc_isi_ctx_format *format,
				    enum mxc_isi_video_type type)
{
	format->format.width = MXC_ISI_DEF_WIDTH;
	format->format.height = MXC_ISI_DEF_HEIGHT;
	format->format.pixelformat = MXC_ISI_DEF_PIXEL_FORMAT;

	format->info = mxc_isi_format_try(&format->format, type);
}

static int mxc_isi_m2m_open(struct file *file)
{
	struct video_device *vdev = video_devdata(file);
	struct mxc_isi_m2m_dev *m2m = video_drvdata(file);
	struct mxc_isi_dev *isi = m2m->isi;
	struct device *dev = isi->dev;
	struct mxc_isi_ctx *ctx;
	int ret;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->m2m = m2m;
	v4l2_fh_init(&ctx->fh, vdev);
	file->private_data = &ctx->fh;

	ctx->fh.m2m_ctx = v4l2_m2m_ctx_init(m2m->m2m_dev, ctx,
					    &mxc_m2m_queue_init);
	if (IS_ERR(ctx->fh.m2m_ctx)) {
		ret = PTR_ERR(ctx->fh.m2m_ctx);
		ctx->fh.m2m_ctx = NULL;
		goto error;
	}

	mxc_isi_m2m_init_format(ctx->formats.out, MXC_ISI_VIDEO_OUT);
	mxc_isi_m2m_init_format(ctx->formats.cap, MXC_ISI_VIDEO_CAP);

	v4l2_fh_add(&ctx->fh);

	ret = pm_runtime_resume_and_get(isi->dev);
	if (ret)
		goto error;

	if (atomic_inc_return(&isi->usage_count) == 1)
		mxc_isi_channel_init(isi);

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
	struct mxc_isi_m2m_dev *m2m = video_drvdata(file);
	struct mxc_isi_dev *mxc_isi = m2m->isi;
	struct device *dev = m2m->isi->dev;
	struct mxc_isi_ctx *ctx = to_isi_ctx(file->private_data);

	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);

	mutex_lock(&m2m->lock);
	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
	mutex_unlock(&m2m->lock);

	kfree(ctx);
	if (atomic_dec_and_test(&mxc_isi->usage_count))
		mxc_isi_channel_deinit(mxc_isi);

	pm_runtime_put(dev);

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

static int isi_m2m_probe(struct platform_device *pdev)
{
	struct mxc_isi_m2m_dev *m2m;
	struct video_device *vdev;
	int ret = -ENOMEM;

	m2m = devm_kzalloc(&pdev->dev, sizeof(*m2m), GFP_KERNEL);
	if (!m2m)
		return -ENOMEM;
	m2m->pdev = pdev;

	spin_lock_init(&m2m->slock);
	mutex_init(&m2m->lock);

	/* m2m */
	m2m->m2m_dev = v4l2_m2m_init(&mxc_isi_m2m_ops);
	if (IS_ERR(m2m->m2m_dev)) {
		dev_err(&pdev->dev, "%s fail to get m2m device\n", __func__);
		return PTR_ERR(m2m->m2m_dev);
	}

	INIT_LIST_HEAD(&m2m->out_active);

	/* Video device */
	vdev = &m2m->vdev;
	memset(vdev, 0, sizeof(*vdev));
	snprintf(vdev->name, sizeof(vdev->name), "mxc_isi.%d.m2m", m2m->id);

	vdev->fops	= &mxc_isi_m2m_fops;
	vdev->ioctl_ops	= &mxc_isi_m2m_ioctl_ops;
	vdev->v4l2_dev	= v4l2_dev;
	vdev->minor	= -1;
	vdev->release	= video_device_release_empty;
	vdev->vfl_dir = VFL_DIR_M2M;
	vdev->device_caps = V4L2_CAP_STREAMING | V4L2_CAP_VIDEO_M2M_MPLANE;

	ret = mxc_isi_m2m_ctrls_create(m2m);
	if (ret)
		goto free_m2m;

	ret = video_register_device(vdev, VFL_TYPE_VIDEO, -1);
	if (ret < 0) {
		dev_err(&pdev->dev, "%s fail to register video device\n", __func__);
		goto ctrl_free;
	}

	vdev->ctrl_handler = &m2m->ctrls.handler;
	video_set_drvdata(vdev, m2m);

	dev_info(&pdev->dev, "Register m2m success for ISI.%d\n", m2m->id);

	return 0;

ctrl_free:
	mxc_isi_m2m_ctrls_delete(m2m);
free_m2m:
	v4l2_m2m_release(m2m->m2m_dev);
	return ret;
}

static int isi_m2m_remove(struct platform_device *pdev)
{
	struct mxc_isi_m2m_dev *m2m = platform_get_drvdata(pdev);
	struct video_device *vdev = &m2m->vdev;

	if (video_is_registered(vdev)) {
		video_unregister_device(vdev);
		mxc_isi_m2m_ctrls_delete(m2m);
		media_entity_cleanup(&vdev->entity);
	}
	v4l2_m2m_release(m2m->m2m_dev);

	return 0;
}
