// SPDX-License-Identifier: (GPL-2.0+ OR MIT)
/*
 * Rockchip ISP1 Driver - CSI-2 Receiver
 *
 * Copyright (C) 2019 Collabora, Ltd.
 * Copyright (C) 2022 Ideas on Board
 *
 * Based on Rockchip ISP1 driver by Rockchip Electronics Co., Ltd.
 * Copyright (C) 2017 Rockchip Electronics Co., Ltd.
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/phy/phy.h>
#include <linux/phy/phy-mipi-dphy.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>

#include "rkisp1-common.h"
#include "rkisp1-csi.h"

#define RKISP1_CSI_DEV_NAME	RKISP1_DRIVER_NAME "_csi"

#define RKISP1_CSI_DEF_FMT	MEDIA_BUS_FMT_SRGGB10_1X10

static inline struct rkisp1_csi *to_rkisp1_csi(struct v4l2_subdev *sd)
{
	return container_of(sd, struct rkisp1_csi, sd);
}

static struct v4l2_mbus_framefmt *
rkisp1_csi_get_pad_fmt(struct rkisp1_csi *csi,
		       struct v4l2_subdev_state *sd_state,
		       unsigned int pad, u32 which)
{
	struct v4l2_subdev_state state = {
		.pads = csi->pad_cfg
	};
	if (which == V4L2_SUBDEV_FORMAT_TRY)
		return v4l2_subdev_get_try_format(&csi->sd, sd_state, pad);
	else
		return v4l2_subdev_get_try_format(&csi->sd, &state, pad);
}

static int rkisp1_csi_config(struct rkisp1_csi *csi,
			     const struct rkisp1_sensor_async *sensor)
{
	struct rkisp1_device *rkisp1 = csi->rkisp1;
	const struct rkisp1_mbus_info *sink_fmt = rkisp1->isp.sink_fmt;
	unsigned int lanes = sensor->lanes;
	u32 mipi_ctrl;

	if (lanes < 1 || lanes > 4)
		return -EINVAL;

	mipi_ctrl = RKISP1_CIF_MIPI_CTRL_NUM_LANES(lanes - 1) |
		    RKISP1_CIF_MIPI_CTRL_SHUTDOWNLANES(0xf) |
		    RKISP1_CIF_MIPI_CTRL_ERR_SOT_SYNC_HS_SKIP |
		    RKISP1_CIF_MIPI_CTRL_CLOCKLANE_ENA;

	rkisp1_write(rkisp1, RKISP1_CIF_MIPI_CTRL, mipi_ctrl);

	/* V12 could also use a newer csi2-host, but we don't want that yet */
	if (rkisp1->info->isp_ver == RKISP1_V12)
		rkisp1_write(rkisp1, RKISP1_CIF_ISP_CSI0_CTRL0, 0);

	/* Configure Data Type and Virtual Channel */
	rkisp1_write(rkisp1, RKISP1_CIF_MIPI_IMG_DATA_SEL,
		     RKISP1_CIF_MIPI_DATA_SEL_DT(sink_fmt->mipi_dt) |
		     RKISP1_CIF_MIPI_DATA_SEL_VC(0));

	/* Clear MIPI interrupts */
	rkisp1_write(rkisp1, RKISP1_CIF_MIPI_ICR, ~0);
	/*
	 * Disable RKISP1_CIF_MIPI_ERR_DPHY interrupt here temporary for
	 * isp bus may be dead when switch isp.
	 */
	rkisp1_write(rkisp1, RKISP1_CIF_MIPI_IMSC,
		     RKISP1_CIF_MIPI_FRAME_END | RKISP1_CIF_MIPI_ERR_CSI |
		     RKISP1_CIF_MIPI_ERR_DPHY |
		     RKISP1_CIF_MIPI_SYNC_FIFO_OVFLW(0x03) |
		     RKISP1_CIF_MIPI_ADD_DATA_OVFLW);

	dev_dbg(rkisp1->dev, "\n  MIPI_CTRL 0x%08x\n"
		"  MIPI_IMG_DATA_SEL 0x%08x\n"
		"  MIPI_STATUS 0x%08x\n"
		"  MIPI_IMSC 0x%08x\n",
		rkisp1_read(rkisp1, RKISP1_CIF_MIPI_CTRL),
		rkisp1_read(rkisp1, RKISP1_CIF_MIPI_IMG_DATA_SEL),
		rkisp1_read(rkisp1, RKISP1_CIF_MIPI_STATUS),
		rkisp1_read(rkisp1, RKISP1_CIF_MIPI_IMSC));

	return 0;
}

static void rkisp1_csi_enable(struct rkisp1_csi *csi)
{
	struct rkisp1_device *rkisp1 = csi->rkisp1;
	u32 val;

	val = rkisp1_read(rkisp1, RKISP1_CIF_MIPI_CTRL);
	rkisp1_write(rkisp1, RKISP1_CIF_MIPI_CTRL,
		     val | RKISP1_CIF_MIPI_CTRL_OUTPUT_ENA);
}

static void rkisp1_csi_disable(struct rkisp1_csi *csi)
{
	struct rkisp1_device *rkisp1 = csi->rkisp1;
	u32 val;

	/* Mask and clear interrupts. */
	rkisp1_write(rkisp1, RKISP1_CIF_MIPI_IMSC, 0);
	rkisp1_write(rkisp1, RKISP1_CIF_MIPI_ICR, ~0);

	val = rkisp1_read(rkisp1, RKISP1_CIF_MIPI_CTRL);
	rkisp1_write(rkisp1, RKISP1_CIF_MIPI_CTRL,
		     val & (~RKISP1_CIF_MIPI_CTRL_OUTPUT_ENA));
}

int rkisp1_csi_start(struct rkisp1_csi *csi,
		     const struct rkisp1_sensor_async *sensor)
{
	struct rkisp1_device *rkisp1 = csi->rkisp1;
	union phy_configure_opts opts;
	struct phy_configure_opts_mipi_dphy *cfg = &opts.mipi_dphy;
	s64 pixel_clock;
	int ret;

	ret = rkisp1_csi_config(csi, sensor);
	if (ret)
		return ret;

	pixel_clock = v4l2_ctrl_g_ctrl_int64(sensor->pixel_rate_ctrl);
	if (!pixel_clock) {
		dev_err(rkisp1->dev, "Invalid pixel rate value\n");
		return -EINVAL;
	}

	phy_mipi_dphy_get_default_config(pixel_clock,
					 rkisp1->isp.sink_fmt->bus_width,
					 sensor->lanes, cfg);
	phy_set_mode(csi->dphy, PHY_MODE_MIPI_DPHY);
	phy_configure(csi->dphy, &opts);
	phy_power_on(csi->dphy);

	rkisp1_csi_enable(csi);

	/*
	 * CIF spec says to wait for sufficient time after enabling
	 * the MIPI interface and before starting the sensor output.
	 */
	usleep_range(1000, 1200);

	return 0;
}

void rkisp1_csi_stop(struct rkisp1_csi *csi)
{
	rkisp1_csi_disable(csi);

	phy_power_off(csi->dphy);
}

irqreturn_t rkisp1_csi_isr(int irq, void *ctx)
{
	struct device *dev = ctx;
	struct rkisp1_device *rkisp1 = dev_get_drvdata(dev);
	u32 val, status;

	status = rkisp1_read(rkisp1, RKISP1_CIF_MIPI_MIS);
	if (!status)
		return IRQ_NONE;

	rkisp1_write(rkisp1, RKISP1_CIF_MIPI_ICR, status);

	/*
	 * Disable DPHY errctrl interrupt, because this dphy
	 * erctrl signal is asserted until the next changes
	 * of line state. This time is may be too long and cpu
	 * is hold in this interrupt.
	 */
	if (status & RKISP1_CIF_MIPI_ERR_CTRL(0x0f)) {
		val = rkisp1_read(rkisp1, RKISP1_CIF_MIPI_IMSC);
		rkisp1_write(rkisp1, RKISP1_CIF_MIPI_IMSC,
			     val & ~RKISP1_CIF_MIPI_ERR_CTRL(0x0f));
		rkisp1->csi.is_dphy_errctrl_disabled = true;
	}

	/*
	 * Enable DPHY errctrl interrupt again, if mipi have receive
	 * the whole frame without any error.
	 */
	if (status == RKISP1_CIF_MIPI_FRAME_END) {
		/*
		 * Enable DPHY errctrl interrupt again, if mipi have receive
		 * the whole frame without any error.
		 */
		if (rkisp1->csi.is_dphy_errctrl_disabled) {
			val = rkisp1_read(rkisp1, RKISP1_CIF_MIPI_IMSC);
			val |= RKISP1_CIF_MIPI_ERR_CTRL(0x0f);
			rkisp1_write(rkisp1, RKISP1_CIF_MIPI_IMSC, val);
			rkisp1->csi.is_dphy_errctrl_disabled = false;
		}
	} else {
		rkisp1->debug.mipi_error++;
	}

	return IRQ_HANDLED;
}

/* ----------------------------------------------------------------------------
 * Subdev pad operations
 */

static void rkisp1_csi_set_src_fmt(struct rkisp1_csi *csi,
				   struct v4l2_subdev_state *sd_state,
				   struct v4l2_mbus_framefmt *format,
				   unsigned int which)
{
	struct v4l2_mbus_framefmt *sink_fmt;

	/* We don't set the src format directly; take it from the sink format */
	sink_fmt = rkisp1_csi_get_pad_fmt(csi, sd_state, RKISP1_CSI_PAD_SINK,
					  which);

	*format = *sink_fmt;
}

static void rkisp1_csi_set_sink_fmt(struct rkisp1_csi *csi,
				    struct v4l2_subdev_state *sd_state,
				    struct v4l2_mbus_framefmt *format,
				    unsigned int which)
{
	const struct rkisp1_mbus_info *mbus_info;
	struct v4l2_mbus_framefmt *sink_fmt, *src_fmt;

	sink_fmt = rkisp1_csi_get_pad_fmt(csi, sd_state, RKISP1_CSI_PAD_SINK,
					  which);
	src_fmt = rkisp1_csi_get_pad_fmt(csi, sd_state, RKISP1_CSI_PAD_SRC,
					 which);

	sink_fmt->code = format->code;

	mbus_info = rkisp1_mbus_info_get_by_code(sink_fmt->code);
	if (!mbus_info || !(mbus_info->direction & RKISP1_ISP_SD_SINK)) {
		sink_fmt->code = RKISP1_CSI_DEF_FMT;
		mbus_info = rkisp1_mbus_info_get_by_code(sink_fmt->code);
	}

	sink_fmt->width = clamp_t(u32, format->width,
				  RKISP1_ISP_MIN_WIDTH,
				  RKISP1_ISP_MAX_WIDTH);
	sink_fmt->height = clamp_t(u32, format->height,
				   RKISP1_ISP_MIN_HEIGHT,
				   RKISP1_ISP_MAX_HEIGHT);

	/* Propagate to source pad */
	src_fmt->code = sink_fmt->code;
	src_fmt->width = sink_fmt->width;
	src_fmt->height = sink_fmt->height;

	*format = *sink_fmt;
}

static int rkisp1_csi_enum_mbus_code(struct v4l2_subdev *sd,
				     struct v4l2_subdev_state *sd_state,
				     struct v4l2_subdev_mbus_code_enum *code)
{
	unsigned int i;
	int pos = 0;

	if (code->index >= rkisp1_mbus_info_length())
		return -EINVAL;

	for (i = 0; i < rkisp1_mbus_info_length(); i++) {
		const struct rkisp1_mbus_info *fmt =
			rkisp1_mbus_info_get_by_index(i);

		if (fmt->direction & RKISP1_ISP_SD_SINK)
			pos++;

		if (code->index == pos - 1) {
			code->code = fmt->mbus_code;
			return 0;
		}
	}

	return -EINVAL;
}

static int rkisp1_csi_init_config(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state)
{
	struct v4l2_mbus_framefmt *sink_fmt, *src_fmt;

	sink_fmt = v4l2_subdev_get_try_format(sd, sd_state,
					      RKISP1_CSI_PAD_SRC);
	sink_fmt->width = RKISP1_DEFAULT_WIDTH;
	sink_fmt->height = RKISP1_DEFAULT_HEIGHT;
	sink_fmt->field = V4L2_FIELD_NONE;
	sink_fmt->code = RKISP1_CSI_DEF_FMT;

	src_fmt = v4l2_subdev_get_try_format(sd, sd_state,
					     RKISP1_CSI_PAD_SINK);
	*src_fmt = *sink_fmt;

	return 0;
}

static int rkisp1_csi_get_fmt(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *sd_state,
			      struct v4l2_subdev_format *fmt)
{
	struct rkisp1_csi *csi = to_rkisp1_csi(sd);

	mutex_lock(&csi->ops_lock);
	fmt->format = *rkisp1_csi_get_pad_fmt(csi, sd_state, fmt->pad,
					      fmt->which);
	mutex_unlock(&csi->ops_lock);
	return 0;
}

static int rkisp1_csi_set_fmt(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *sd_state,
			      struct v4l2_subdev_format *fmt)
{
	struct rkisp1_csi *csi = to_rkisp1_csi(sd);

	mutex_lock(&csi->ops_lock);
	if (fmt->pad == RKISP1_CSI_PAD_SINK)
		rkisp1_csi_set_sink_fmt(csi, sd_state, &fmt->format,
					fmt->which);
	else
		rkisp1_csi_set_src_fmt(csi, sd_state, &fmt->format,
				       fmt->which);

	mutex_unlock(&csi->ops_lock);
	return 0;
}

/* ----------------------------------------------------------------------------
 * Subdev video operations
 */

static int rkisp1_csi_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct rkisp1_csi *csi = to_rkisp1_csi(sd);
	struct rkisp1_device *rkisp1 = csi->rkisp1;
	struct media_pad *source_pad;
	struct v4l2_subdev *source;
	int ret;

	if (!enable) {
		v4l2_subdev_call(csi->source->sd, video, s_stream,
				 false);

		rkisp1_csi_stop(csi);

		return ret;
	}

	source_pad = media_entity_remote_source_pad(&sd->entity);
	if (IS_ERR(source_pad)) {
		dev_dbg(rkisp1->dev, "Failed to get source for CSI: %d\n", ret);
		return -EPIPE;
	}

	source = media_entity_to_v4l2_subdev(source_pad->entity);
	if (!source) {
		/* This should really not happen, so is not worth a message. */
		return -EPIPE;
	}

	csi->source = container_of(source->asd, struct rkisp1_sensor_async,
				   asd);

	if (csi->source->mbus_type != V4L2_MBUS_CSI2_DPHY)
		return -EINVAL;

	mutex_lock(&csi->ops_lock);
	ret = rkisp1_csi_start(csi, csi->source);
	mutex_unlock(&csi->ops_lock);
	if (ret)
		return ret;

	v4l2_subdev_call(csi->source->sd, video, s_stream, true);

	return 0;
}

/* ----------------------------------------------------------------------------
 * Registration
 */

static const struct media_entity_operations rkisp1_csi_media_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static const struct v4l2_subdev_video_ops rkisp1_csi_video_ops = {
	.s_stream = rkisp1_csi_s_stream,
};

static const struct v4l2_subdev_pad_ops rkisp1_csi_pad_ops = {
	.enum_mbus_code = rkisp1_csi_enum_mbus_code,
	.init_cfg = rkisp1_csi_init_config,
	.get_fmt = rkisp1_csi_get_fmt,
	.set_fmt = rkisp1_csi_set_fmt,
};

static const struct v4l2_subdev_ops rkisp1_csi_ops = {
	.video = &rkisp1_csi_video_ops,
	.pad = &rkisp1_csi_pad_ops,
};

int rkisp1_csi_register(struct rkisp1_device *rkisp1)
{
	struct rkisp1_csi *csi = &rkisp1->csi;
	struct v4l2_subdev_state state = {};
	struct media_pad *pads;
	struct v4l2_subdev *sd;
	int ret;

	csi->rkisp1 = rkisp1;
	mutex_init(&csi->ops_lock);

	sd = &csi->sd;
	v4l2_subdev_init(sd, &rkisp1_csi_ops);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sd->entity.ops = &rkisp1_csi_media_ops;
	sd->entity.function = MEDIA_ENT_F_VID_IF_BRIDGE;
	sd->owner = THIS_MODULE;
	strscpy(sd->name, RKISP1_CSI_DEV_NAME, sizeof(sd->name));

	pads = csi->pads;
	pads[RKISP1_CSI_PAD_SINK].flags = MEDIA_PAD_FL_SINK |
					  MEDIA_PAD_FL_MUST_CONNECT;
	pads[RKISP1_CSI_PAD_SRC].flags = MEDIA_PAD_FL_SOURCE |
					 MEDIA_PAD_FL_MUST_CONNECT;

	ret = media_entity_pads_init(&sd->entity, RKISP1_CSI_PAD_MAX, pads);
	if (ret)
		goto error;

	state.pads = csi->pad_cfg;
	rkisp1_csi_init_config(sd, &state);

	ret = v4l2_device_register_subdev(&csi->rkisp1->v4l2_dev, sd);
	if (ret) {
		dev_err(sd->dev, "Failed to register csi receiver subdev\n");
		goto error;
	}

	return 0;

error:
	media_entity_cleanup(&sd->entity);
	mutex_destroy(&csi->ops_lock);
	csi->rkisp1 = NULL;
	return ret;
}

void rkisp1_csi_unregister(struct rkisp1_device *rkisp1)
{
	struct rkisp1_csi *csi = &rkisp1->csi;

	if (!csi->rkisp1)
		return;

	v4l2_device_unregister_subdev(&csi->sd);
	media_entity_cleanup(&csi->sd.entity);
	mutex_destroy(&csi->ops_lock);
}

int rkisp1_csi_init(struct rkisp1_device *rkisp1)
{
	struct rkisp1_csi *csi = &rkisp1->csi;

	csi->dphy = devm_phy_get(rkisp1->dev, "dphy");
	if (IS_ERR(csi->dphy)) {
		if (PTR_ERR(csi->dphy) != -EPROBE_DEFER)
		dev_err_probe(rkisp1->dev, PTR_ERR(csi->dphy),
			      "Couldn't get the MIPI D-PHY\n");
		return PTR_ERR(csi->dphy);
	}

	phy_init(csi->dphy);

	return 0;
}

void rkisp1_csi_cleanup(struct rkisp1_device *rkisp1)
{
	struct rkisp1_csi *csi = &rkisp1->csi;

	phy_exit(csi->dphy);
}
