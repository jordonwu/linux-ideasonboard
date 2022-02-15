// SPDX-License-Identifier: GPL-2.0-only
/*
 * i.MX8 ISI - Input crossbar switch
 *
 * Copyright (c) 2022 Laurent Pinchart <laurent.pinchart@ideasonboard.com>
 */

#include <linux/device.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/minmax.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/types.h>

#include <media/media-entity.h>
#include <media/v4l2-subdev.h>

#include "imx8-isi-core.h"

static inline struct mxc_isi_crossbar *to_isi_crossbar(struct v4l2_subdev *sd)
{
	return container_of(sd, struct mxc_isi_crossbar, sd);
}

/* -----------------------------------------------------------------------------
 * V4L2 subdev operations
 */

static const struct v4l2_mbus_framefmt mxc_isi_crossbar_default_format = {
	.code = MXC_ISI_DEF_MBUS_CODE_SINK,
	.width = MXC_ISI_DEF_WIDTH,
	.height = MXC_ISI_DEF_HEIGHT,
	.field = V4L2_FIELD_NONE,
	.colorspace = V4L2_COLORSPACE_SRGB,
};

static int __mxc_isi_crossbar_set_routing(struct v4l2_subdev *sd,
					  struct v4l2_subdev_state *state,
					  struct v4l2_subdev_krouting *routing)
{
	int ret;

	ret = v4l2_subdev_routing_validate(sd, routing,
					   V4L2_SUBDEV_ROUTING_NO_N_TO_1 |
					   V4L2_SUBDEV_ROUTING_NO_STREAM_MIX);
	if (ret)
		return ret;

	return v4l2_subdev_set_routing_with_fmt(sd, state, routing,
						&mxc_isi_crossbar_default_format);
}

static int __mxc_isi_crossbar_enable_streams(struct v4l2_subdev *sd,
					     struct v4l2_subdev_state *state,
					     u32 pad, u64 streams_mask,
					     bool enable)
{
	struct mxc_isi_crossbar *xbar = to_isi_crossbar(sd);
	struct v4l2_subdev_route *route;
	struct v4l2_subdev *remote_sd;
	struct media_pad *remote_pad;
	u64 sink_streams = 0;
	int sink_pad = -1;

	/*
	 * Translate the source pad and streams to the sink side. The routing
	 * validation forbids stream merging, so all matching entries in the
	 * routing table are guaranteed to have the same sink pad.
	 *
	 * TODO: This is likely worth a helper function, it could perhaps be
	 * supported by v4l2_subdev_state_xlate_streams() with pad1 set to -1.
	 */
	for_each_active_route(&state->routing, route) {
		if (route->source_pad != pad ||
		    !(streams_mask & BIT(route->source_stream)))
			continue;

		sink_streams |= BIT(route->sink_stream);
		sink_pad = route->sink_pad;
	}

	if (sink_pad < 0) {
		dev_dbg(xbar->isi->dev,
			"no stream connected to pipeline %u\n",
			pad - xbar->num_sinks);
		return -EPIPE;
	}

	remote_pad = media_entity_remote_pad(&xbar->pads[sink_pad]);
	remote_sd = media_entity_to_v4l2_subdev(remote_pad->entity);

	if (!remote_sd) {
		dev_dbg(xbar->isi->dev,
			"no entity connected to crossbar input %u\n",
			sink_pad);
		return -EPIPE;
	}

	/*
	 * TODO: Avoid multiple enable or disable of the same inputs when the
	 * routing table duplicates streams (1-to-N).
	 */
	if (enable)
		return v4l2_subdev_enable_streams(remote_sd, remote_pad->index,
						  sink_streams);
	else
		return v4l2_subdev_disable_streams(remote_sd, remote_pad->index,
						   sink_streams);
}

static int mxc_isi_crossbar_init_cfg(struct v4l2_subdev *sd,
				     struct v4l2_subdev_state *state)
{
	struct mxc_isi_crossbar *xbar = to_isi_crossbar(sd);
	struct v4l2_subdev_krouting routing = { };
	struct v4l2_subdev_route *routes;
	unsigned int i;
	int ret;

	/*
	 * Create a 1:1 mapping between pixel link inputs and outputs to
	 * pipelines by default.
	 */
	routes = kcalloc(xbar->num_sources, sizeof(*routes), GFP_KERNEL);
	if (!routes)
		return -ENOMEM;

	for (i = 0; i < xbar->num_sources; ++i) {
		struct v4l2_subdev_route *route = &routes[i];

		route->sink_pad = i;
		route->source_pad = i + xbar->num_sinks;
		route->flags = V4L2_SUBDEV_ROUTE_FL_ACTIVE;
	};

	routing.num_routes = xbar->num_sources;
	routing.routes = routes;

	ret = __mxc_isi_crossbar_set_routing(sd, state, &routing);

	kfree(routes);

	return ret;
}

static int mxc_isi_crossbar_enum_mbus_code(struct v4l2_subdev *sd,
				       struct v4l2_subdev_state *state,
				       struct v4l2_subdev_mbus_code_enum *code)
{
	struct mxc_isi_crossbar *xbar = to_isi_crossbar(sd);
	const struct mxc_isi_bus_format_info *info;

	if (code->pad >= xbar->num_sinks) {
		const struct v4l2_mbus_framefmt *format;

		/*
		 * The media bus code on source pads is identical to the
		 * connected sink pad.
		 */
		if (code->index > 0)
			return -EINVAL;

		format = v4l2_subdev_state_get_opposite_stream_format(state,
								      code->pad,
								      code->stream);
		if (!format)
			return -EINVAL;

		code->code = format->code;;

		return 0;
	}

	info = mxc_isi_bus_format_by_index(code->index, MXC_ISI_PIPE_PAD_SINK);
	if (!info)
		return -EINVAL;

	code->code = info->mbus_code;

	return 0;
}

static int mxc_isi_crossbar_set_fmt(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *state,
				struct v4l2_subdev_format *fmt)
{
	struct mxc_isi_crossbar *xbar = to_isi_crossbar(sd);
	struct v4l2_mbus_framefmt *sink_fmt;
	struct v4l2_mbus_framefmt *source_fmt;

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE &&
	    media_entity_is_streaming(&sd->entity))
		return -EBUSY;

	/*
	 * The source pad format is always identical to the sink pad format and
	 * can't be modified.
	 */
	if (fmt->pad >= xbar->num_sinks)
		return v4l2_subdev_get_fmt(sd, state, fmt);

	/* Validate the requested format. */
	fmt->format.width = clamp_t(unsigned int, fmt->format.width,
				    MXC_ISI_MIN_WIDTH, MXC_ISI_MAX_WIDTH);
	fmt->format.height = clamp_t(unsigned int, fmt->format.height,
				     MXC_ISI_MIN_HEIGHT, MXC_ISI_MAX_HEIGHT);
	fmt->format.field = V4L2_FIELD_NONE;

	/*
	 * Set the format on the sink stream and propagate it to the source
	 * stream.
	 */
	sink_fmt = v4l2_subdev_state_get_stream_format(state, fmt->pad,
						       fmt->stream);
	source_fmt = v4l2_subdev_state_get_opposite_stream_format(state,
								  fmt->pad,
								  fmt->stream);
	if (!sink_fmt || !source_fmt)
		return -EINVAL;

	*sink_fmt = fmt->format;
	*source_fmt = fmt->format;

	return 0;
}

static int mxc_isi_crossbar_set_routing(struct v4l2_subdev *sd,
					struct v4l2_subdev_state *state,
					enum v4l2_subdev_format_whence which,
					struct v4l2_subdev_krouting *routing)
{
	if (which == V4L2_SUBDEV_FORMAT_ACTIVE &&
	    media_entity_is_streaming(&sd->entity))
		return -EBUSY;

	return __mxc_isi_crossbar_set_routing(sd, state, routing);
}

static int mxc_isi_crossbar_enable_streams(struct v4l2_subdev *sd,
					   struct v4l2_subdev_state *state,
					   u32 pad, u64 streams_mask)
{
	return __mxc_isi_crossbar_enable_streams(sd, state, pad, streams_mask,
						 true);
}

static int mxc_isi_crossbar_disable_streams(struct v4l2_subdev *sd,
					    struct v4l2_subdev_state *state,
					    u32 pad, u64 streams_mask)
{
	return __mxc_isi_crossbar_enable_streams(sd, state, pad, streams_mask,
						 false);
}

static const struct v4l2_subdev_pad_ops mxc_isi_crossbar_subdev_pad_ops = {
	.init_cfg = mxc_isi_crossbar_init_cfg,
	.enum_mbus_code = mxc_isi_crossbar_enum_mbus_code,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = mxc_isi_crossbar_set_fmt,
	.set_routing = mxc_isi_crossbar_set_routing,
	.enable_streams = mxc_isi_crossbar_enable_streams,
	.disable_streams = mxc_isi_crossbar_disable_streams,
};

static const struct v4l2_subdev_ops mxc_isi_crossbar_subdev_ops = {
	.pad = &mxc_isi_crossbar_subdev_pad_ops,
};

static const struct media_entity_operations mxc_isi_cross_entity_ops = {
	.get_fwnode_pad = v4l2_subdev_get_fwnode_pad_1_to_1,
	.link_validate	= v4l2_subdev_link_validate,
	.has_route = v4l2_subdev_has_route,
};

/* -----------------------------------------------------------------------------
 * Init & cleanup
 */

int mxc_isi_crossbar_init(struct mxc_isi_dev *isi)
{
	struct mxc_isi_crossbar *xbar = &isi->crossbar;
	struct v4l2_subdev *sd = &xbar->sd;
	unsigned int num_pads;
	unsigned int i;
	int ret;

	xbar->isi = isi;

	v4l2_subdev_init(sd, &mxc_isi_crossbar_subdev_ops);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_MULTIPLEXED;
	strlcpy(sd->name, "crossbar", sizeof(sd->name));
	sd->dev = isi->dev;

	sd->entity.function = MEDIA_ENT_F_VID_MUX;
	sd->entity.ops = &mxc_isi_cross_entity_ops;

	/*
	 * The subdev has one sink and one source per port, plus one sink for
	 * the memory input.
	 */
	xbar->num_sinks = isi->pdata->num_ports + 1;
	xbar->num_sources = isi->pdata->num_ports;
	num_pads = xbar->num_sinks + xbar->num_sources;

	xbar->pads = kcalloc(num_pads, sizeof(*xbar->pads), GFP_KERNEL);
	if (!xbar->pads)
		return -ENOMEM;

	for (i = 0; i < xbar->num_sinks; ++i)
		xbar->pads[i].flags = MEDIA_PAD_FL_SINK;
	for (i = 0; i < xbar->num_sources; ++i)
		xbar->pads[i + xbar->num_sinks].flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&sd->entity, num_pads, xbar->pads);
	if (ret)
		return ret;

	ret = v4l2_subdev_init_finalize(sd);
	if (ret < 0)
		goto error;

	return 0;

error:
	media_entity_cleanup(&sd->entity);

	return ret;
}

void mxc_isi_crossbar_cleanup(struct mxc_isi_crossbar *xbar)
{
	media_entity_cleanup(&xbar->sd.entity);
	kfree(xbar->pads);
}

int mxc_isi_crossbar_register(struct mxc_isi_crossbar *xbar)
{
	return v4l2_device_register_subdev(&xbar->isi->v4l2_dev, &xbar->sd);
}

void mxc_isi_crossbar_unregister(struct mxc_isi_crossbar *xbar)
{
}
