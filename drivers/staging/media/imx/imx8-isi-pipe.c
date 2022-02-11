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
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/minmax.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/videodev2.h>

#include <media/media-entity.h>
#include <media/v4l2-subdev.h>
#include <media/videobuf2-v4l2.h>

#include "imx8-isi-core.h"
#include "imx8-isi-regs.h"

static struct v4l2_subdev *mxc_isi_get_source_subdev(struct mxc_isi_pipe *pipe,
						     u32 *pad,
						     const char * const label)
{
	struct v4l2_subdev *subdev = &pipe->sd;
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
		dev_err(pipe->isi->dev, "%s, No remote pad found!\n", label);
		return NULL;
	}

	/* Get remote source pad subdev */
	sen_sd = media_entity_to_v4l2_subdev(source_pad->entity);
	if (!sen_sd) {
		dev_err(pipe->isi->dev, "%s, No remote subdev found!\n", label);
		return NULL;
	}

	if (pad)
		*pad = source_pad->index;

	return sen_sd;
}

/*
 * mxc_isi_pipeline_enable() - Enable streaming on a pipeline
 */
int mxc_isi_pipeline_enable(struct mxc_isi_pipe *pipe, bool enable)
{
	struct v4l2_subdev *src_sd;
	int ret;

	src_sd = mxc_isi_get_source_subdev(pipe, NULL, __func__);
	if (!src_sd)
		return -EPIPE;

	ret = v4l2_subdev_call(src_sd, video, s_stream, enable);
	if (ret < 0 && ret != -ENOIOCTLCMD) {
		dev_err(pipe->isi->dev, "subdev %s s_stream failed\n",
			src_sd->name);
		return ret;
	}

	return 0;
}

static void mxc_isi_cap_frame_write_done(struct mxc_isi_pipe *pipe)
{
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
	if ((mxc_isi_is_buf_active(pipe, 1) && buf->id == MXC_ISI_BUF1) ||
	    (mxc_isi_is_buf_active(pipe, 2) && buf->id == MXC_ISI_BUF2)) {
		dev_dbg(dev, "status=0x%x id=%d\n", pipe->status, buf->id);
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
		mxc_isi_channel_set_outbuf(pipe, buf);
		list_move_tail(pipe->video.out_discard.next, &pipe->video.out_active);
		return;
	}

	/* ISI channel output buffer */
	buf = list_first_entry(&pipe->video.out_pending, struct mxc_isi_buffer, list);
	buf->v4l2_buf.sequence = pipe->video.frame_count;
	mxc_isi_channel_set_outbuf(pipe, buf);
	vb2 = &buf->v4l2_buf.vb2_buf;
	vb2->state = VB2_BUF_STATE_ACTIVE;
	list_move_tail(pipe->video.out_pending.next, &pipe->video.out_active);
}

/* -----------------------------------------------------------------------------
 * V4L2 subdev operations
 */

static struct v4l2_mbus_framefmt *
mxc_isi_pipe_get_pad_format(struct mxc_isi_pipe *pipe,
			    struct v4l2_subdev_state *state,
			    enum v4l2_subdev_format_whence which,
			    unsigned int pad)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &pipe->formats[pad].format;
	case V4L2_SUBDEV_FORMAT_TRY:
	default:
		return v4l2_subdev_get_try_format(&pipe->sd, state, pad);
	}
}

static struct v4l2_rect *
mxc_isi_pipe_get_pad_crop(struct mxc_isi_pipe *pipe,
			  struct v4l2_subdev_state *state,
			  enum v4l2_subdev_format_whence which,
			  unsigned int pad)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &pipe->formats[pad].crop;
	case V4L2_SUBDEV_FORMAT_TRY:
	default:
		return v4l2_subdev_get_try_crop(&pipe->sd, state, pad);
	}
}

static struct v4l2_rect *
mxc_isi_pipe_get_pad_compose(struct mxc_isi_pipe *pipe,
			     struct v4l2_subdev_state *state,
			     enum v4l2_subdev_format_whence which,
			     unsigned int pad)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &pipe->formats[pad].compose;
	case V4L2_SUBDEV_FORMAT_TRY:
	default:
		return v4l2_subdev_get_try_compose(&pipe->sd, state, pad);
	}
}

static int mxc_isi_pipe_s_stream(struct v4l2_subdev *sd, int enable)
{
	return 0;
}

static int mxc_isi_pipe_init_cfg(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state)
{
	enum v4l2_subdev_format_whence which = state ? V4L2_SUBDEV_FORMAT_TRY
					     : V4L2_SUBDEV_FORMAT_ACTIVE;
	struct mxc_isi_pipe *pipe = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *fmt_source;
	struct v4l2_mbus_framefmt *fmt_sink;
	struct v4l2_rect *compose;
	struct v4l2_rect *crop;

	fmt_sink = mxc_isi_pipe_get_pad_format(pipe, state, which,
					       MXC_ISI_SD_PAD_SINK);
	fmt_source = mxc_isi_pipe_get_pad_format(pipe, state, which,
						 MXC_ISI_SD_PAD_SOURCE);

	fmt_sink->width = MXC_ISI_DEF_WIDTH;
	fmt_sink->height = MXC_ISI_DEF_HEIGHT;
	fmt_sink->code = MEDIA_BUS_FMT_UYVY8_1X16;
	fmt_sink->field = V4L2_FIELD_NONE;
	fmt_sink->colorspace = V4L2_COLORSPACE_JPEG;
	fmt_sink->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt_sink->colorspace);
	fmt_sink->quantization =
		V4L2_MAP_QUANTIZATION_DEFAULT(false, fmt_sink->colorspace,
					      fmt_sink->ycbcr_enc);
	fmt_sink->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(fmt_sink->colorspace);

	*fmt_source = *fmt_sink;

	compose = mxc_isi_pipe_get_pad_compose(pipe, state, which,
					       MXC_ISI_SD_PAD_SINK);
	crop = mxc_isi_pipe_get_pad_crop(pipe, state, which,
					 MXC_ISI_SD_PAD_SOURCE);

	compose->left = 0;
	compose->top = 0;
	compose->width = MXC_ISI_DEF_WIDTH;
	compose->height = MXC_ISI_DEF_HEIGHT;

	*crop = *compose;

	return 0;
}

static int mxc_isi_pipe_enum_mbus_code(struct v4l2_subdev *sd,
				       struct v4l2_subdev_state *state,
				       struct v4l2_subdev_mbus_code_enum *code)
{
	return 0;
}

static int mxc_isi_pipe_get_fmt(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *state,
				struct v4l2_subdev_format *fmt)
{
	struct mxc_isi_pipe *pipe = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *format;

	format = mxc_isi_pipe_get_pad_format(pipe, state, fmt->which, fmt->pad);

	mutex_lock(&pipe->lock);
	fmt->format = *format;
	mutex_unlock(&pipe->lock);

	return 0;
}

static int mxc_isi_pipe_set_fmt(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *state,
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
		info = mxc_isi_format_by_code(MEDIA_BUS_FMT_UYVY8_1X16);

	mutex_lock(&pipe->lock);

	if (fmt->pad == MXC_ISI_SD_PAD_SINK) {
		unsigned int max_width;

		/*
		 * FIXME: This needs to handled more dynamically, larger line
		 * lengths are possible when bypassing the scaler.
		 */
		max_width = pipe->isi->pdata->model == MXC_ISI_IMX8MN
			  ? 2048 : 4096;

		mf->code = info->mbus_code;
		mf->width = clamp(mf->width, MXC_ISI_MIN_WIDTH, max_width);
		mf->height = clamp(mf->height, MXC_ISI_MIN_HEIGHT,
				   MXC_ISI_MAX_HEIGHT);

		/* Propagate the format to the source pad. */
		rect = mxc_isi_pipe_get_pad_compose(pipe, state, fmt->which,
						    MXC_ISI_SD_PAD_SINK);
		rect->width = mf->width;
		rect->height = mf->height;

		rect = mxc_isi_pipe_get_pad_crop(pipe, state, fmt->which,
						 MXC_ISI_SD_PAD_SOURCE);
		rect->left = 0;
		rect->top = 0;
		rect->width = mf->width;
		rect->height = mf->height;

		format = mxc_isi_pipe_get_pad_format(pipe, state, fmt->which,
						     MXC_ISI_SD_PAD_SOURCE);
		format->code = info->mbus_code;
		format->width = mf->width;
		format->height = mf->height;
	} else {
		/*
		 * The width and height on the source can't be changed, they
		 * must match the crop rectangle size.
		 */
		rect = mxc_isi_pipe_get_pad_crop(pipe, state, fmt->which,
						 MXC_ISI_SD_PAD_SOURCE);

		mf->width = rect->width;
		mf->height = rect->height;
	}

	format = mxc_isi_pipe_get_pad_format(pipe, state, fmt->which, fmt->pad);
	*format = *mf;

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE)
		pipe->formats[fmt->pad].info = info;

	mutex_unlock(&pipe->lock);

	dev_dbg(pipe->isi->dev, "pad%d: code: 0x%x, %dx%d",
		fmt->pad, mf->code, mf->width, mf->height);

	return 0;
}

static int mxc_isi_pipe_get_selection(struct v4l2_subdev *sd,
				      struct v4l2_subdev_state *state,
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
		format = mxc_isi_pipe_get_pad_format(pipe, state, sel->which,
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
		rect = mxc_isi_pipe_get_pad_compose(pipe, state, sel->which,
						    MXC_ISI_SD_PAD_SINK);
		sel->r = *rect;
		break;

	case V4L2_SEL_TGT_CROP:
		if (sel->pad != MXC_ISI_SD_PAD_SOURCE) {
			/* No crop rectangle on sink pad. */
			ret = -EINVAL;
			break;
		}

		rect = mxc_isi_pipe_get_pad_crop(pipe, state, sel->which,
						 sel->pad);
		sel->r = *rect;
		break;

	case V4L2_SEL_TGT_COMPOSE:
		if (sel->pad != MXC_ISI_SD_PAD_SINK) {
			/* No compose rectangle on source pad. */
			ret = -EINVAL;
			break;
		}

		rect = mxc_isi_pipe_get_pad_compose(pipe, state, sel->which,
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
				      struct v4l2_subdev_state *state,
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
		rect = mxc_isi_pipe_get_pad_compose(pipe, state, sel->which,
						    MXC_ISI_SD_PAD_SINK);
		sel->r.left = clamp_t(s32, sel->r.left, 0, rect->width - 1);
		sel->r.top = clamp_t(s32, sel->r.top, 0, rect->height - 1);
		sel->r.width = clamp(sel->r.width, MXC_ISI_MIN_WIDTH,
				     rect->width - sel->r.left);
		sel->r.height = clamp(sel->r.height, MXC_ISI_MIN_HEIGHT,
				      rect->height - sel->r.top);

		rect = mxc_isi_pipe_get_pad_crop(pipe, state, sel->which,
						 MXC_ISI_SD_PAD_SOURCE);
		*rect = sel->r;

		/* Propagate the crop rectangle to the source pad. */
		format = mxc_isi_pipe_get_pad_format(pipe, state, sel->which,
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
		format = mxc_isi_pipe_get_pad_format(pipe, state, sel->which,
						     MXC_ISI_SD_PAD_SINK);

		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = clamp(sel->r.width, MXC_ISI_MIN_WIDTH,
				     format->width);
		sel->r.height = clamp(sel->r.height, MXC_ISI_MIN_HEIGHT,
				      format->height);

		rect = mxc_isi_pipe_get_pad_compose(pipe, state, sel->which,
						    MXC_ISI_SD_PAD_SINK);
		*rect = sel->r;

		/* Propagate the compose rectangle to the source pad. */
		rect = mxc_isi_pipe_get_pad_crop(pipe, state, sel->which,
						 MXC_ISI_SD_PAD_SOURCE);
		rect->left = 0;
		rect->top = 0;
		rect->width = sel->r.width;
		rect->height = sel->r.height;

		format = mxc_isi_pipe_get_pad_format(pipe, state, sel->which,
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

static const struct v4l2_subdev_video_ops mxc_isi_subdev_video_ops = {
	.s_stream = mxc_isi_pipe_s_stream,
};

static const struct v4l2_subdev_pad_ops mxc_isi_subdev_pad_ops = {
	.init_cfg = mxc_isi_pipe_init_cfg,
	.enum_mbus_code = mxc_isi_pipe_enum_mbus_code,
	.get_fmt = mxc_isi_pipe_get_fmt,
	.set_fmt = mxc_isi_pipe_set_fmt,
	.get_selection = mxc_isi_pipe_get_selection,
	.set_selection = mxc_isi_pipe_set_selection,
};

static const struct v4l2_subdev_ops mxc_isi_subdev_ops = {
	.video = &mxc_isi_subdev_video_ops,
	.pad = &mxc_isi_subdev_pad_ops,
};

/* -----------------------------------------------------------------------------
 * IRQ handling
 */

static irqreturn_t mxc_isi_pipe_irq_handler(int irq, void *priv)
{
	struct mxc_isi_pipe *pipe = priv;
	const struct mxc_isi_ier_reg *ier_reg = pipe->isi->pdata->ier_reg;
	unsigned long flags;
	u32 status;

	spin_lock_irqsave(&pipe->slock, flags);

	status = mxc_isi_get_irq_status(pipe);
	pipe->status = status;

	if (status & CHNL_STS_FRM_STRD_MASK)
		mxc_isi_cap_frame_write_done(pipe);

	spin_unlock_irqrestore(&pipe->slock, flags);

	if (status & (CHNL_STS_AXI_WR_ERR_Y_MASK |
		      CHNL_STS_AXI_WR_ERR_U_MASK |
		      CHNL_STS_AXI_WR_ERR_V_MASK))
		dev_dbg(pipe->isi->dev, "%s: IRQ AXI Error stat=0x%X\n",
			__func__, status);

	if (status & (ier_reg->panic_y_buf_en.mask |
		      ier_reg->panic_u_buf_en.mask |
		      ier_reg->panic_v_buf_en.mask))
		dev_dbg(pipe->isi->dev, "%s: IRQ Panic OFLW Error stat=0x%X\n",
			__func__, status);

	if (status & (ier_reg->oflw_y_buf_en.mask |
		      ier_reg->oflw_u_buf_en.mask |
		      ier_reg->oflw_v_buf_en.mask))
		dev_dbg(pipe->isi->dev, "%s: IRQ OFLW Error stat=0x%X\n",
			__func__, status);

	if (status & (ier_reg->excs_oflw_y_buf_en.mask |
		      ier_reg->excs_oflw_u_buf_en.mask |
		      ier_reg->excs_oflw_v_buf_en.mask))
		dev_dbg(pipe->isi->dev, "%s: IRQ EXCS OFLW Error stat=0x%X\n",
			__func__, status);

	return IRQ_HANDLED;
}

/* -----------------------------------------------------------------------------
 * Init & cleanup
 */

int mxc_isi_pipe_init(struct mxc_isi_dev *isi, unsigned int id)
{
	struct mxc_isi_pipe *pipe = &isi->pipes[id];
	struct v4l2_subdev *sd;
	unsigned int i;
	int irq;
	int ret;

	pipe->id = id;
	pipe->isi = isi;
	pipe->regs = isi->regs + id * isi->pdata->reg_offset;

	atomic_set(&pipe->usage_count, 0);

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

	v4l2_set_subdevdata(sd, pipe);

	sd->fwnode = of_fwnode_handle(pipe->isi->dev->of_node);

	/* Default configuration. */
	mxc_isi_pipe_init_cfg(sd, NULL);

	for (i = 0; i < ARRAY_SIZE(pipe->formats); ++i) {
		struct mxc_isi_frame *frame = &pipe->formats[i];

		frame->info = mxc_isi_format_by_code(frame->format.code);
	}

	/* Register IRQ handler. */
	mxc_isi_clean_registers(pipe);
	mxc_isi_channel_set_chain_buf(pipe);

	irq = platform_get_irq(to_platform_device(isi->dev), id);
	if (irq < 0) {
		dev_err(pipe->isi->dev, "Failed to get IRQ (%d)\n", irq);
		ret = irq;
		goto error;
	}

	ret = devm_request_irq(isi->dev, irq, mxc_isi_pipe_irq_handler,
			       0, dev_name(isi->dev), pipe);
	if (ret < 0) {
		dev_err(isi->dev, "failed to request IRQ (%d)\n", ret);
		goto error;
	}

	return 0;

error:
	media_entity_cleanup(&sd->entity);
	v4l2_set_subdevdata(sd, NULL);

	return ret;
}

void mxc_isi_pipe_cleanup(struct mxc_isi_pipe *pipe)
{
	struct v4l2_subdev *sd = &pipe->sd;

	media_entity_cleanup(&sd->entity);
	v4l2_set_subdevdata(sd, NULL);
}

int mxc_isi_pipe_register(struct mxc_isi_pipe *pipe)
{
	int ret;

	ret = v4l2_device_register_subdev(&pipe->isi->v4l2_dev, &pipe->sd);
	if (ret < 0)
		return ret;

	return mxc_isi_video_register(pipe, &pipe->isi->v4l2_dev);
}

void mxc_isi_pipe_unregister(struct mxc_isi_pipe *pipe)
{
	mxc_isi_video_unregister(pipe);
}
