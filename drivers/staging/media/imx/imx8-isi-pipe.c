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

/*
 * TODO: Add comment to explain that the pipeline subdev covers the gasket, and
 * thus can have inputs narrower than 24 bit.
 */
static const struct mxc_isi_bus_format_info mxc_isi_bus_formats[] = {
	/* YUV formats */
	{
		.mbus_code	= MEDIA_BUS_FMT_UYVY8_1X16,
		.output		= MEDIA_BUS_FMT_YUV8_1X24,
		.pads		= BIT(MXC_ISI_PIPE_PAD_SINK),
		.encoding	= MXC_ISI_ENC_YUV,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_YUV8_1X24,
		.output		= MEDIA_BUS_FMT_YUV8_1X24,
		.pads		= BIT(MXC_ISI_PIPE_PAD_SOURCE),
		.encoding	= MXC_ISI_ENC_YUV,
	},
	/* RGB formats */
	{
		.mbus_code	= MEDIA_BUS_FMT_RGB565_1X16,
		.output		= MEDIA_BUS_FMT_RGB888_1X24,
		.pads		= BIT(MXC_ISI_PIPE_PAD_SINK),
		.encoding	= MXC_ISI_ENC_RGB,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_RGB888_1X24,
		.output		= MEDIA_BUS_FMT_RGB888_1X24,
		.pads		= BIT(MXC_ISI_PIPE_PAD_SINK)
				| BIT(MXC_ISI_PIPE_PAD_SOURCE),
		.encoding	= MXC_ISI_ENC_RGB,
	},
	/* RAW formats */
	{
		.mbus_code	= MEDIA_BUS_FMT_Y8_1X8,
		.output		= MEDIA_BUS_FMT_Y8_1X8,
		.pads		= BIT(MXC_ISI_PIPE_PAD_SINK)
				| BIT(MXC_ISI_PIPE_PAD_SOURCE),
		.encoding	= MXC_ISI_ENC_RAW,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_Y10_1X10,
		.output		= MEDIA_BUS_FMT_Y10_1X10,
		.pads		= BIT(MXC_ISI_PIPE_PAD_SINK)
				| BIT(MXC_ISI_PIPE_PAD_SOURCE),
		.encoding	= MXC_ISI_ENC_RAW,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_Y12_1X12,
		.output		= MEDIA_BUS_FMT_Y12_1X12,
		.pads		= BIT(MXC_ISI_PIPE_PAD_SINK)
				| BIT(MXC_ISI_PIPE_PAD_SOURCE),
		.encoding	= MXC_ISI_ENC_RAW,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_Y14_1X14,
		.output		= MEDIA_BUS_FMT_Y14_1X14,
		.pads		= BIT(MXC_ISI_PIPE_PAD_SINK)
				| BIT(MXC_ISI_PIPE_PAD_SOURCE),
		.encoding	= MXC_ISI_ENC_RAW,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SBGGR8_1X8,
		.output		= MEDIA_BUS_FMT_SBGGR8_1X8,
		.pads		= BIT(MXC_ISI_PIPE_PAD_SINK)
				| BIT(MXC_ISI_PIPE_PAD_SOURCE),
		.encoding	= MXC_ISI_ENC_RAW,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SGBRG8_1X8,
		.output		= MEDIA_BUS_FMT_SGBRG8_1X8,
		.pads		= BIT(MXC_ISI_PIPE_PAD_SINK)
				| BIT(MXC_ISI_PIPE_PAD_SOURCE),
		.encoding	= MXC_ISI_ENC_RAW,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SGRBG8_1X8,
		.output		= MEDIA_BUS_FMT_SGRBG8_1X8,
		.pads		= BIT(MXC_ISI_PIPE_PAD_SINK)
				| BIT(MXC_ISI_PIPE_PAD_SOURCE),
		.encoding	= MXC_ISI_ENC_RAW,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SRGGB8_1X8,
		.output		= MEDIA_BUS_FMT_SRGGB8_1X8,
		.pads		= BIT(MXC_ISI_PIPE_PAD_SINK)
				| BIT(MXC_ISI_PIPE_PAD_SOURCE),
		.encoding	= MXC_ISI_ENC_RAW,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SBGGR10_1X10,
		.output		= MEDIA_BUS_FMT_SBGGR10_1X10,
		.pads		= BIT(MXC_ISI_PIPE_PAD_SINK)
				| BIT(MXC_ISI_PIPE_PAD_SOURCE),
		.encoding	= MXC_ISI_ENC_RAW,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SGBRG10_1X10,
		.output		= MEDIA_BUS_FMT_SGBRG10_1X10,
		.pads		= BIT(MXC_ISI_PIPE_PAD_SINK)
				| BIT(MXC_ISI_PIPE_PAD_SOURCE),
		.encoding	= MXC_ISI_ENC_RAW,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SGRBG10_1X10,
		.output		= MEDIA_BUS_FMT_SGRBG10_1X10,
		.pads		= BIT(MXC_ISI_PIPE_PAD_SINK)
				| BIT(MXC_ISI_PIPE_PAD_SOURCE),
		.encoding	= MXC_ISI_ENC_RAW,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SRGGB10_1X10,
		.output		= MEDIA_BUS_FMT_SRGGB10_1X10,
		.pads		= BIT(MXC_ISI_PIPE_PAD_SINK)
				| BIT(MXC_ISI_PIPE_PAD_SOURCE),
		.encoding	= MXC_ISI_ENC_RAW,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SBGGR12_1X12,
		.output		= MEDIA_BUS_FMT_SBGGR12_1X12,
		.pads		= BIT(MXC_ISI_PIPE_PAD_SINK)
				| BIT(MXC_ISI_PIPE_PAD_SOURCE),
		.encoding	= MXC_ISI_ENC_RAW,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SGBRG12_1X12,
		.output		= MEDIA_BUS_FMT_SGBRG12_1X12,
		.pads		= BIT(MXC_ISI_PIPE_PAD_SINK)
				| BIT(MXC_ISI_PIPE_PAD_SOURCE),
		.encoding	= MXC_ISI_ENC_RAW,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SGRBG12_1X12,
		.output		= MEDIA_BUS_FMT_SGRBG12_1X12,
		.pads		= BIT(MXC_ISI_PIPE_PAD_SINK)
				| BIT(MXC_ISI_PIPE_PAD_SOURCE),
		.encoding	= MXC_ISI_ENC_RAW,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SRGGB12_1X12,
		.output		= MEDIA_BUS_FMT_SRGGB12_1X12,
		.pads		= BIT(MXC_ISI_PIPE_PAD_SINK)
				| BIT(MXC_ISI_PIPE_PAD_SOURCE),
		.encoding	= MXC_ISI_ENC_RAW,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SBGGR14_1X14,
		.output		= MEDIA_BUS_FMT_SBGGR14_1X14,
		.pads		= BIT(MXC_ISI_PIPE_PAD_SINK)
				| BIT(MXC_ISI_PIPE_PAD_SOURCE),
		.encoding	= MXC_ISI_ENC_RAW,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SGBRG14_1X14,
		.output		= MEDIA_BUS_FMT_SGBRG14_1X14,
		.pads		= BIT(MXC_ISI_PIPE_PAD_SINK)
				| BIT(MXC_ISI_PIPE_PAD_SOURCE),
		.encoding	= MXC_ISI_ENC_RAW,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SGRBG14_1X14,
		.output		= MEDIA_BUS_FMT_SGRBG14_1X14,
		.pads		= BIT(MXC_ISI_PIPE_PAD_SINK)
				| BIT(MXC_ISI_PIPE_PAD_SOURCE),
		.encoding	= MXC_ISI_ENC_RAW,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SRGGB14_1X14,
		.output		= MEDIA_BUS_FMT_SRGGB14_1X14,
		.pads		= BIT(MXC_ISI_PIPE_PAD_SINK)
				| BIT(MXC_ISI_PIPE_PAD_SOURCE),
		.encoding	= MXC_ISI_ENC_RAW,
	}
};

static const struct mxc_isi_bus_format_info *
mxc_isi_bus_format_by_code(u32 code, unsigned int pad)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(mxc_isi_bus_formats); i++) {
		const struct mxc_isi_bus_format_info *info =
			&mxc_isi_bus_formats[i];

		if (info->mbus_code == code && info->pads & BIT(pad))
			return info;
	}

	return NULL;
}

const struct mxc_isi_bus_format_info *
mxc_isi_bus_format_by_index(unsigned int index, unsigned int pad)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(mxc_isi_bus_formats); i++) {
		const struct mxc_isi_bus_format_info *info =
			&mxc_isi_bus_formats[i];

		if (!(info->pads & BIT(pad)))
			continue;

		if (!index)
			return info;

		index--;
	}

	return NULL;
}

static inline struct mxc_isi_pipe *to_isi_pipe(struct v4l2_subdev *sd)
{
	return container_of(sd, struct mxc_isi_pipe, sd);
}

int mxc_isi_pipe_enable(struct mxc_isi_pipe *pipe)
{
	struct mxc_isi_crossbar *xbar = &pipe->isi->crossbar;
	const struct mxc_isi_bus_format_info *sink_info;
	const struct mxc_isi_bus_format_info *src_info;
	const struct v4l2_mbus_framefmt *sink_fmt;
	const struct v4l2_mbus_framefmt *src_fmt;
	const struct v4l2_rect *compose;
	struct v4l2_subdev_state *state;
	struct v4l2_subdev *sd = &pipe->sd;
	int ret;

	state = v4l2_subdev_lock_active_state(sd);

	sink_fmt = v4l2_subdev_get_try_format(sd, state, MXC_ISI_PIPE_PAD_SINK);
	src_fmt = v4l2_subdev_get_try_format(sd, state, MXC_ISI_PIPE_PAD_SOURCE);
	compose = v4l2_subdev_get_try_compose(sd, state, MXC_ISI_PIPE_PAD_SINK);

	sink_info = mxc_isi_bus_format_by_code(sink_fmt->code,
					       MXC_ISI_PIPE_PAD_SINK);
	src_info = mxc_isi_bus_format_by_code(src_fmt->code,
					      MXC_ISI_PIPE_PAD_SOURCE);

	mxc_isi_channel_config(pipe, sink_fmt, compose, sink_info->encoding,
			       src_info->encoding);

	v4l2_subdev_unlock_state(state);

	mxc_isi_channel_enable(pipe);

	ret = v4l2_subdev_enable_streams(&xbar->sd, xbar->num_sinks + pipe->id,
					 BIT(0));
	if (ret) {
		mxc_isi_channel_disable(pipe);
		dev_err(pipe->isi->dev, "Failed to enable pipe %u\n",
			pipe->id);
		return ret;
	}

	return 0;
}

void mxc_isi_pipe_disable(struct mxc_isi_pipe *pipe)
{
	struct mxc_isi_crossbar *xbar = &pipe->isi->crossbar;
	int ret;

	ret = v4l2_subdev_disable_streams(&xbar->sd, xbar->num_sinks + pipe->id,
					  BIT(0));
	if (ret)
		dev_err(pipe->isi->dev, "Failed to disable pipe %u\n",
			pipe->id);

	mxc_isi_channel_disable(pipe);
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
			    unsigned int pad)
{
	return v4l2_subdev_get_try_format(&pipe->sd, state, pad);
}

static struct v4l2_rect *
mxc_isi_pipe_get_pad_crop(struct mxc_isi_pipe *pipe,
			  struct v4l2_subdev_state *state,
			  unsigned int pad)
{
	return v4l2_subdev_get_try_crop(&pipe->sd, state, pad);
}

static struct v4l2_rect *
mxc_isi_pipe_get_pad_compose(struct mxc_isi_pipe *pipe,
			     struct v4l2_subdev_state *state,
			     unsigned int pad)
{
	return v4l2_subdev_get_try_compose(&pipe->sd, state, pad);
}

static int mxc_isi_pipe_init_cfg(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state)
{
	struct mxc_isi_pipe *pipe = to_isi_pipe(sd);
	struct v4l2_mbus_framefmt *fmt_source;
	struct v4l2_mbus_framefmt *fmt_sink;
	struct v4l2_rect *compose;
	struct v4l2_rect *crop;

	fmt_sink = mxc_isi_pipe_get_pad_format(pipe, state,
					       MXC_ISI_PIPE_PAD_SINK);
	fmt_source = mxc_isi_pipe_get_pad_format(pipe, state,
						 MXC_ISI_PIPE_PAD_SOURCE);

	fmt_sink->width = MXC_ISI_DEF_WIDTH;
	fmt_sink->height = MXC_ISI_DEF_HEIGHT;
	fmt_sink->code = MXC_ISI_DEF_MBUS_CODE_SINK;
	fmt_sink->field = V4L2_FIELD_NONE;
	fmt_sink->colorspace = V4L2_COLORSPACE_JPEG;
	fmt_sink->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt_sink->colorspace);
	fmt_sink->quantization =
		V4L2_MAP_QUANTIZATION_DEFAULT(false, fmt_sink->colorspace,
					      fmt_sink->ycbcr_enc);
	fmt_sink->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(fmt_sink->colorspace);

	*fmt_source = *fmt_sink;
	fmt_source->code = MXC_ISI_DEF_MBUS_CODE_SOURCE;

	compose = mxc_isi_pipe_get_pad_compose(pipe, state,
					       MXC_ISI_PIPE_PAD_SINK);
	crop = mxc_isi_pipe_get_pad_crop(pipe, state, MXC_ISI_PIPE_PAD_SOURCE);

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
	static const u32 output_codes[] = {
		MEDIA_BUS_FMT_YUV8_1X24,
		MEDIA_BUS_FMT_RGB888_1X24,
	};
	struct mxc_isi_pipe *pipe = to_isi_pipe(sd);
	const struct mxc_isi_bus_format_info *info;
	unsigned int index;
	unsigned int i;

	if (code->pad == MXC_ISI_PIPE_PAD_SOURCE) {
		const struct v4l2_mbus_framefmt *format;

		format = mxc_isi_pipe_get_pad_format(pipe, state,
						     MXC_ISI_PIPE_PAD_SINK);
		info = mxc_isi_bus_format_by_code(format->code,
						  MXC_ISI_PIPE_PAD_SINK);

		if (info->encoding == MXC_ISI_ENC_RAW) {
			/*
			 * For RAW formats, the sink and source media bus codes
			 * must match.
			 */
			if (code->index)
				return -EINVAL;

			code->code = info->output;
		} else {
			/*
			 * For RGB or YUV formats, the ISI supports format
			 * conversion. Either of the two output formats can be
			 * used regardless of the input.
			 */
			if (code->index > 1)
				return -EINVAL;

			code->code = output_codes[code->index];
		}

		return 0;
	}

	index = code->index;

	for (i = 0; i < ARRAY_SIZE(mxc_isi_bus_formats); ++i) {
		info = &mxc_isi_bus_formats[i];

		if (!(info->pads & BIT(MXC_ISI_PIPE_PAD_SINK)))
			continue;

		if (index == 0) {
			code->code = info->mbus_code;
			return 0;
		}

		index--;
	}

	return -EINVAL;
}

static int mxc_isi_pipe_set_fmt(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *state,
				struct v4l2_subdev_format *fmt)
{
	struct mxc_isi_pipe *pipe = to_isi_pipe(sd);
	struct v4l2_mbus_framefmt *mf = &fmt->format;
	const struct mxc_isi_bus_format_info *info;
	struct v4l2_mbus_framefmt *format;
	struct v4l2_rect *rect;

	if (vb2_is_busy(&pipe->video.vb2_q))
		return -EBUSY;

	if (fmt->pad == MXC_ISI_PIPE_PAD_SINK) {
		unsigned int max_width;

		info = mxc_isi_bus_format_by_code(mf->code,
						  MXC_ISI_PIPE_PAD_SINK);
		if (!info)
			info = mxc_isi_bus_format_by_code(MXC_ISI_DEF_MBUS_CODE_SINK,
							  MXC_ISI_PIPE_PAD_SINK);

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
		rect = mxc_isi_pipe_get_pad_compose(pipe, state,
						    MXC_ISI_PIPE_PAD_SINK);
		rect->width = mf->width;
		rect->height = mf->height;

		rect = mxc_isi_pipe_get_pad_crop(pipe, state,
						 MXC_ISI_PIPE_PAD_SOURCE);
		rect->left = 0;
		rect->top = 0;
		rect->width = mf->width;
		rect->height = mf->height;

		format = mxc_isi_pipe_get_pad_format(pipe, state,
						     MXC_ISI_PIPE_PAD_SOURCE);
		format->code = info->output;
		format->width = mf->width;
		format->height = mf->height;
	} else {
		/*
		 * For RGB or YUV formats, the ISI supports RGB <-> YUV format
		 * conversion. For RAW formats, the sink and source media bus
		 * codes must match.
		 */
		format = mxc_isi_pipe_get_pad_format(pipe, state,
						     MXC_ISI_PIPE_PAD_SINK);
		info = mxc_isi_bus_format_by_code(format->code,
						  MXC_ISI_PIPE_PAD_SINK);

		if (info->encoding != MXC_ISI_ENC_RAW) {
			if (mf->code != MEDIA_BUS_FMT_YUV8_1X24 &&
			    mf->code != MEDIA_BUS_FMT_RGB888_1X24)
				mf->code = info->output;

			info = mxc_isi_bus_format_by_code(mf->code,
							  MXC_ISI_PIPE_PAD_SOURCE);
		}

		mf->code = info->output;

		/*
		 * The width and height on the source can't be changed, they
		 * must match the crop rectangle size.
		 */
		rect = mxc_isi_pipe_get_pad_crop(pipe, state,
						 MXC_ISI_PIPE_PAD_SOURCE);

		mf->width = rect->width;
		mf->height = rect->height;
	}

	format = mxc_isi_pipe_get_pad_format(pipe, state, fmt->pad);
	*format = *mf;

	dev_dbg(pipe->isi->dev, "pad%u: code: 0x%04x, %ux%u",
		fmt->pad, mf->code, mf->width, mf->height);

	return 0;
}

static int mxc_isi_pipe_get_selection(struct v4l2_subdev *sd,
				      struct v4l2_subdev_state *state,
				      struct v4l2_subdev_selection *sel)
{
	struct mxc_isi_pipe *pipe = to_isi_pipe(sd);
	const struct v4l2_mbus_framefmt *format;
	const struct v4l2_rect *rect;

	switch (sel->target) {
	case V4L2_SEL_TGT_COMPOSE_BOUNDS:
		if (sel->pad != MXC_ISI_PIPE_PAD_SINK)
			/* No compose rectangle on source pad. */
			return -EINVAL;

		/* The sink compose is bound by the sink format. */
		format = mxc_isi_pipe_get_pad_format(pipe, state,
						     MXC_ISI_PIPE_PAD_SINK);
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = format->width;
		sel->r.height = format->height;
		break;

	case V4L2_SEL_TGT_CROP_BOUNDS:
		if (sel->pad != MXC_ISI_PIPE_PAD_SOURCE)
			/* No crop rectangle on sink pad. */
			return -EINVAL;

		/* The source crop is bound by the sink compose. */
		rect = mxc_isi_pipe_get_pad_compose(pipe, state,
						    MXC_ISI_PIPE_PAD_SINK);
		sel->r = *rect;
		break;

	case V4L2_SEL_TGT_CROP:
		if (sel->pad != MXC_ISI_PIPE_PAD_SOURCE)
			/* No crop rectangle on sink pad. */
			return -EINVAL;

		rect = mxc_isi_pipe_get_pad_crop(pipe, state, sel->pad);
		sel->r = *rect;
		break;

	case V4L2_SEL_TGT_COMPOSE:
		if (sel->pad != MXC_ISI_PIPE_PAD_SINK)
			/* No compose rectangle on source pad. */
			return -EINVAL;

		rect = mxc_isi_pipe_get_pad_compose(pipe, state, sel->pad);
		sel->r = *rect;
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static int mxc_isi_pipe_set_selection(struct v4l2_subdev *sd,
				      struct v4l2_subdev_state *state,
				      struct v4l2_subdev_selection *sel)
{
	struct mxc_isi_pipe *pipe = to_isi_pipe(sd);
	struct v4l2_mbus_framefmt *format;
	struct v4l2_rect *rect;
	unsigned long flags;

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
		if (sel->pad != MXC_ISI_PIPE_PAD_SOURCE)
			/* The pipeline support cropping on the source only. */
			return -EINVAL;

		/* The source crop is bound by the sink compose. */
		rect = mxc_isi_pipe_get_pad_compose(pipe, state,
						    MXC_ISI_PIPE_PAD_SINK);
		sel->r.left = clamp_t(s32, sel->r.left, 0, rect->width - 1);
		sel->r.top = clamp_t(s32, sel->r.top, 0, rect->height - 1);
		sel->r.width = clamp(sel->r.width, MXC_ISI_MIN_WIDTH,
				     rect->width - sel->r.left);
		sel->r.height = clamp(sel->r.height, MXC_ISI_MIN_HEIGHT,
				      rect->height - sel->r.top);

		rect = mxc_isi_pipe_get_pad_crop(pipe, state,
						 MXC_ISI_PIPE_PAD_SOURCE);
		*rect = sel->r;

		/* Propagate the crop rectangle to the source pad. */
		format = mxc_isi_pipe_get_pad_format(pipe, state,
						     MXC_ISI_PIPE_PAD_SOURCE);
		format->width = sel->r.width;
		format->height = sel->r.height;
		break;

	case V4L2_SEL_TGT_COMPOSE:
		if (sel->pad != MXC_ISI_PIPE_PAD_SINK)
			/* Composing is supported on the sink only. */
			return -EINVAL;

		/* The sink crop is bound by the sink format downscaling only). */
		format = mxc_isi_pipe_get_pad_format(pipe, state,
						     MXC_ISI_PIPE_PAD_SINK);

		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = clamp(sel->r.width, MXC_ISI_MIN_WIDTH,
				     format->width);
		sel->r.height = clamp(sel->r.height, MXC_ISI_MIN_HEIGHT,
				      format->height);

		rect = mxc_isi_pipe_get_pad_compose(pipe, state,
						    MXC_ISI_PIPE_PAD_SINK);
		*rect = sel->r;

		/* Propagate the compose rectangle to the source pad. */
		rect = mxc_isi_pipe_get_pad_crop(pipe, state,
						 MXC_ISI_PIPE_PAD_SOURCE);
		rect->left = 0;
		rect->top = 0;
		rect->width = sel->r.width;
		rect->height = sel->r.height;

		format = mxc_isi_pipe_get_pad_format(pipe, state,
						     MXC_ISI_PIPE_PAD_SOURCE);
		format->width = sel->r.width;
		format->height = sel->r.height;
		break;

	default:
		return -EINVAL;
	}

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

static const struct v4l2_subdev_pad_ops mxc_isi_pipe_subdev_pad_ops = {
	.init_cfg = mxc_isi_pipe_init_cfg,
	.enum_mbus_code = mxc_isi_pipe_enum_mbus_code,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = mxc_isi_pipe_set_fmt,
	.get_selection = mxc_isi_pipe_get_selection,
	.set_selection = mxc_isi_pipe_set_selection,
};

static const struct v4l2_subdev_ops mxc_isi_pipe_subdev_ops = {
	.pad = &mxc_isi_pipe_subdev_pad_ops,
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

	if (status & CHNL_STS_FRM_STRD)
		mxc_isi_cap_frame_write_done(pipe);

	spin_unlock_irqrestore(&pipe->slock, flags);

	if (status & (CHNL_STS_AXI_WR_ERR_Y |
		      CHNL_STS_AXI_WR_ERR_U |
		      CHNL_STS_AXI_WR_ERR_V))
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
	int irq;
	int ret;

	pipe->id = id;
	pipe->isi = isi;
	pipe->regs = isi->regs + id * isi->pdata->reg_offset;

	atomic_set(&pipe->usage_count, 0);

	spin_lock_init(&pipe->slock);

	sd = &pipe->sd;
	v4l2_subdev_init(sd, &mxc_isi_pipe_subdev_ops);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	snprintf(sd->name, sizeof(sd->name), "mxc_isi.%d", pipe->id);
	sd->dev = isi->dev;

	sd->entity.function = MEDIA_ENT_F_PROC_VIDEO_PIXEL_FORMATTER;

	pipe->pads[MXC_ISI_PIPE_PAD_SINK].flags = MEDIA_PAD_FL_SINK;
	pipe->pads[MXC_ISI_PIPE_PAD_SOURCE].flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&sd->entity, MXC_ISI_PIPE_PADS_NUM,
				     pipe->pads);
	if (ret)
		return ret;

	ret = v4l2_subdev_init_finalize(sd);
	if (ret < 0)
		goto error;

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

	return ret;
}

void mxc_isi_pipe_cleanup(struct mxc_isi_pipe *pipe)
{
	struct v4l2_subdev *sd = &pipe->sd;

	media_entity_cleanup(&sd->entity);
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
