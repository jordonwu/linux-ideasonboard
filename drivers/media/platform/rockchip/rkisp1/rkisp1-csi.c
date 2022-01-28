// SPDX-License-Identifier: (GPL-2.0+ OR MIT)
/*
 * Rockchip ISP1 Driver - CSI Subdevice
 *
 * Copyright (C) 2022 Ideas on Board
 *
 * Based on Rockchip ISP1 driver by Rockchip Electronics Co., Ltd.
 * Copyright (C) 2017 Rockchip Electronics Co., Ltd.
 */

#include <linux/phy/phy.h>
#include <media/v4l2-fwnode.h>

#include "rkisp1-common.h"

#define RKISP1_CSI_DEV_NAME	RKISP1_DRIVER_NAME "_csi"

#define RKISP1_CSI_DEF_FMT MEDIA_BUS_FMT_SRGGB10_1X10

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

int rkisp1_csi_link_sensor(struct rkisp1_device *rkisp1, struct v4l2_subdev *sd,
			   struct rkisp1_sensor_async *s_asd)
{
	struct rkisp1_csi *csi =
		container_of(rkisp1->csi_subdev, struct rkisp1_csi, sd);
	struct media_entity *source, *sink;
	unsigned int flags, source_pad;
	int ret;		

	s_asd->pixel_rate_ctrl = v4l2_ctrl_find(sd->ctrl_handler,
						V4L2_CID_PIXEL_RATE);
	s_asd->sd = sd;
	s_asd->dphy = devm_phy_get(rkisp1->dev, "dphy");
	if (IS_ERR(s_asd->dphy)) {
		if (PTR_ERR(s_asd->dphy) != -EPROBE_DEFER)
			dev_err(rkisp1->dev, "Couldn't get the MIPI D-PHY\n");
		return PTR_ERR(s_asd->dphy);
	}

	phy_init(s_asd->dphy);

	/* link the sensor */
	flags = MEDIA_LNK_FL_ENABLED;

	ret = media_entity_get_fwnode_pad(&sd->entity, sd->fwnode,
					  MEDIA_PAD_FL_SOURCE);
	if (ret < 0) {
		dev_err(rkisp1->dev, "failed to find src pad for %s\n",
			sd->name);
		return ret;
	}
	source_pad = ret;

	ret = media_create_pad_link(&sd->entity, source_pad,
				    &csi->sd.entity, RKISP1_CSI_PAD_SINK,
				    flags);
	if (ret) {
		dev_err(csi->rkisp1->dev, "failed to link src pad of %s\n",
			sd->name);
		return ret;
	}

	csi->active_sensor = s_asd;

	/* link the isp */
	source = &rkisp1->csi_subdev->entity;
	sink = &rkisp1->isp.sd.entity;
	ret = media_create_pad_link(source, RKISP1_CSI_PAD_SRC,
				    sink, RKISP1_ISP_PAD_SINK_VIDEO,
				    MEDIA_LNK_FL_ENABLED);
	if (ret)
		return ret;

	return 0;
}

static int rkisp1_config_mipi(struct rkisp1_csi *csi)
{
	struct rkisp1_device *rkisp1 = csi->rkisp1;
	const struct rkisp1_mbus_info *sink_fmt = rkisp1->isp.sink_fmt;
	unsigned int lanes = csi->active_sensor->lanes;
	u32 mipi_ctrl;

	if (lanes < 1 || lanes > 4)
		return -EINVAL;

	mipi_ctrl = RKISP1_CIF_MIPI_CTRL_NUM_LANES(lanes - 1) |
		    RKISP1_CIF_MIPI_CTRL_SHUTDOWNLANES(0xf) |
		    RKISP1_CIF_MIPI_CTRL_ERR_SOT_SYNC_HS_SKIP |
		    RKISP1_CIF_MIPI_CTRL_CLOCKLANE_ENA;

	rkisp1_write(rkisp1, mipi_ctrl, RKISP1_CIF_MIPI_CTRL);

	/* V12 could also use a newer csi2-host, but we don't want that yet */
	if (rkisp1->info->isp_ver == RKISP1_V12)
		rkisp1_write(rkisp1, 0, RKISP1_CIF_ISP_CSI0_CTRL0);

	/* Configure Data Type and Virtual Channel */
	rkisp1_write(rkisp1,
		     RKISP1_CIF_MIPI_DATA_SEL_DT(sink_fmt->mipi_dt) |
		     RKISP1_CIF_MIPI_DATA_SEL_VC(0),
		     RKISP1_CIF_MIPI_IMG_DATA_SEL);

	/* Clear MIPI interrupts */
	rkisp1_write(rkisp1, ~0, RKISP1_CIF_MIPI_ICR);
	/*
	 * Disable RKISP1_CIF_MIPI_ERR_DPHY interrupt here temporary for
	 * isp bus may be dead when switch isp.
	 */
	rkisp1_write(rkisp1,
		     RKISP1_CIF_MIPI_FRAME_END | RKISP1_CIF_MIPI_ERR_CSI |
		     RKISP1_CIF_MIPI_ERR_DPHY |
		     RKISP1_CIF_MIPI_SYNC_FIFO_OVFLW(0x03) |
		     RKISP1_CIF_MIPI_ADD_DATA_OVFLW,
		     RKISP1_CIF_MIPI_IMSC);

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

static void rkisp1_mipi_start(struct rkisp1_device *rkisp1)
{
	u32 val;

	val = rkisp1_read(rkisp1, RKISP1_CIF_MIPI_CTRL);
	rkisp1_write(rkisp1, val | RKISP1_CIF_MIPI_CTRL_OUTPUT_ENA,
			RKISP1_CIF_MIPI_CTRL);
}

static void rkisp1_mipi_clear_interrupts(struct rkisp1_device *rkisp1)
{
	rkisp1_write(rkisp1, 0, RKISP1_CIF_MIPI_IMSC);
	rkisp1_write(rkisp1, ~0, RKISP1_CIF_MIPI_ICR);
}

static void rkisp1_mipi_stop(struct rkisp1_device *rkisp1)
{
	u32 val;

	rkisp1_mipi_clear_interrupts(rkisp1);

	val = rkisp1_read(rkisp1, RKISP1_CIF_MIPI_CTRL);
	rkisp1_write(rkisp1, val & (~RKISP1_CIF_MIPI_CTRL_OUTPUT_ENA),
		     RKISP1_CIF_MIPI_CTRL);
}

static irqreturn_t rkisp1_mipi_isr(int irq, struct rkisp1_device *rkisp1)
{
	u32 val, status;

	status = rkisp1_read(rkisp1, RKISP1_CIF_MIPI_MIS);
	if (!status)
		return IRQ_NONE;

	rkisp1_write(rkisp1, status, RKISP1_CIF_MIPI_ICR);

	/*
	 * Disable DPHY errctrl interrupt, because this dphy
	 * erctrl signal is asserted until the next changes
	 * of line state. This time is may be too long and cpu
	 * is hold in this interrupt.
	 */
	if (status & RKISP1_CIF_MIPI_ERR_CTRL(0x0f)) {
		val = rkisp1_read(rkisp1, RKISP1_CIF_MIPI_IMSC);
		rkisp1_write(rkisp1, val & ~RKISP1_CIF_MIPI_ERR_CTRL(0x0f),
			     RKISP1_CIF_MIPI_IMSC);
		rkisp1->isp.is_dphy_errctrl_disabled = true;
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
		if (rkisp1->isp.is_dphy_errctrl_disabled) {
			val = rkisp1_read(rkisp1, RKISP1_CIF_MIPI_IMSC);
			val |= RKISP1_CIF_MIPI_ERR_CTRL(0x0f);
			rkisp1_write(rkisp1, val, RKISP1_CIF_MIPI_IMSC);
			rkisp1->isp.is_dphy_errctrl_disabled = false;
		}
	} else {
		rkisp1->debug.mipi_error++;
	}

	return IRQ_HANDLED;
}

static int rkisp1_csi_isr(struct v4l2_subdev *sd, u32 status, bool *handled)
{
	struct rkisp1_csi *csi = container_of(sd, struct rkisp1_csi, sd);
	struct rkisp1_device *rkisp1 = csi->rkisp1;

	*handled = rkisp1_mipi_isr(status, rkisp1) == IRQ_HANDLED;

	return 0;
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

	mbus_info = rkisp1_mbus_info_get(sink_fmt->code);
	if (!mbus_info || !(mbus_info->direction & RKISP1_ISP_SD_SINK)) {
		sink_fmt->code = RKISP1_CSI_DEF_FMT;
		mbus_info = rkisp1_mbus_info_get(sink_fmt->code);
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
	struct rkisp1_csi *csi = container_of(sd, struct rkisp1_csi, sd);

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
	struct rkisp1_csi *csi = container_of(sd, struct rkisp1_csi, sd);

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
 * Stream operations
 */

static int rkisp1_mipi_csi2_start(struct rkisp1_isp *isp,
				  struct rkisp1_sensor_async *sensor)
{
	struct rkisp1_device *rkisp1 =
		container_of(isp->sd.v4l2_dev, struct rkisp1_device, v4l2_dev);
	union phy_configure_opts opts;
	struct phy_configure_opts_mipi_dphy *cfg = &opts.mipi_dphy;
	s64 pixel_clock;

	if (!sensor->pixel_rate_ctrl) {
		dev_warn(rkisp1->dev, "No pixel rate control in sensor subdev\n");
		return -EPIPE;
	}

	pixel_clock = v4l2_ctrl_g_ctrl_int64(sensor->pixel_rate_ctrl);
	if (!pixel_clock) {
		dev_err(rkisp1->dev, "Invalid pixel rate value\n");
		return -EINVAL;
	}

	phy_mipi_dphy_get_default_config(pixel_clock, isp->sink_fmt->bus_width,
					 sensor->lanes, cfg);
	phy_set_mode(sensor->dphy, PHY_MODE_MIPI_DPHY);
	phy_configure(sensor->dphy, &opts);
	phy_power_on(sensor->dphy);

	return 0;
}

static void rkisp1_mipi_csi2_stop(struct rkisp1_sensor_async *sensor)
{
	phy_power_off(sensor->dphy);
}

static int rkisp1_csi_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct rkisp1_csi *csi = container_of(sd, struct rkisp1_csi, sd);
	struct rkisp1_device *rkisp1 = csi->rkisp1;
	int ret;

	if (!enable) {
		v4l2_subdev_call(csi->active_sensor->sd, video, s_stream, 0);
		rkisp1_mipi_stop(rkisp1);

		ret = v4l2_subdev_call(csi->active_sensor->sd, core, s_power, 0);
		if (ret == -ENOIOCTLCMD)
			ret = 0;

		rkisp1_mipi_csi2_stop(csi->active_sensor);

		return ret;
	}

	ret = rkisp1_config_mipi(csi);
	if (ret)
		return ret;

	ret = rkisp1_mipi_csi2_start(&rkisp1->isp, csi->active_sensor);
	if (ret)
		return ret;

	ret = v4l2_subdev_call(csi->active_sensor->sd, core, s_power, 1);
	if (ret < 0 && ret != -ENOIOCTLCMD)
		return ret;

	mutex_lock(&csi->ops_lock);
	rkisp1_mipi_start(rkisp1);
	v4l2_subdev_call(csi->active_sensor->sd, video, s_stream, 1);
	mutex_unlock(&csi->ops_lock);

	return 0;
}

/* ----------------------------------------------------------------------------
 * Registration
 */

static const struct media_entity_operations rkisp1_csi_media_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static const struct v4l2_subdev_core_ops rkisp1_csi_core_ops = {
	.interrupt_service_routine = rkisp1_csi_isr,
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
	.core = &rkisp1_csi_core_ops,
	.video = &rkisp1_csi_video_ops,
	.pad = &rkisp1_csi_pad_ops,
};

int rkisp1_csi_register(struct rkisp1_device *rkisp1)
{
	struct rkisp1_csi *csi = &rkisp1->csi;
	struct v4l2_subdev_state state = {};
	static const char *dev_name = RKISP1_CSI_DEV_NAME;
	struct media_pad *pads;
	struct v4l2_subdev *sd;
	int ret;

	csi->rkisp1 = rkisp1;
	state.pads = csi->pad_cfg;
	pads = csi->pads;
	sd = &csi->sd;

	v4l2_subdev_init(sd, &rkisp1_csi_ops);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sd->entity.ops = &rkisp1_csi_media_ops;
	sd->entity.function = MEDIA_ENT_F_VID_IF_BRIDGE;
	sd->owner = THIS_MODULE;
	strscpy(sd->name, dev_name, sizeof(sd->name));

	pads[RKISP1_CSI_PAD_SINK].flags = MEDIA_PAD_FL_SINK |
					  MEDIA_PAD_FL_MUST_CONNECT;
	pads[RKISP1_CSI_PAD_SRC].flags = MEDIA_PAD_FL_SOURCE |
					 MEDIA_PAD_FL_MUST_CONNECT;

	mutex_init(&csi->ops_lock);
	ret = media_entity_pads_init(&sd->entity, RKISP1_CSI_PAD_MAX, pads);
	if (ret)
		goto error;

	rkisp1_csi_init_config(sd, &state);

	ret = v4l2_device_register_subdev(&csi->rkisp1->v4l2_dev, sd);
	if (ret) {
		dev_err(sd->dev, "Failed to register csi receiver subdev\n");
		goto error;
	}

	rkisp1->csi_subdev = sd;

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
