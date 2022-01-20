// SPDX-License-Identifier: (GPL-2.0+ OR MIT)
/*
 * Rockchip ISP1 Driver - CSI Subdevice
 *
 * Copyright (C) 2022 Ideas on Board
 *
 * Based on Rockchip ISP1 driver by Rockchip Electronics Co., Ltd.
 * Copyright (C) 2017 Rockchip Electronics Co., Ltd.
 */

#include "rkisp1-common.h"

int rkisp1_config_mipi(struct rkisp1_device *rkisp1)
{
	const struct rkisp1_isp_mbus_info *sink_fmt = rkisp1->isp.sink_fmt;
	unsigned int lanes = rkisp1->active_sensor->lanes;
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

void rkisp1_mipi_start(struct rkisp1_device *rkisp1)
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

void rkisp1_mipi_stop(struct rkisp1_device *rkisp1)
{
	u32 val;

	rkisp1_mipi_clear_interrupts(rkisp1);

	val = rkisp1_read(rkisp1, RKISP1_CIF_MIPI_CTRL);
	rkisp1_write(rkisp1, val & (~RKISP1_CIF_MIPI_CTRL_OUTPUT_ENA),
		     RKISP1_CIF_MIPI_CTRL);
}

irqreturn_t rkisp1_mipi_isr(int irq, void *ctx)
{
	struct device *dev = ctx;
	struct rkisp1_device *rkisp1 = dev_get_drvdata(dev);
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
