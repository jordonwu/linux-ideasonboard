// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2019-2020 NXP
 *
 */
#include <dt-bindings/pinctrl/pads-imx8qxp.h>

#include <linux/module.h>

#include "imx8-isi-hw.h"

#define	ISI_DOWNSCALE_THRESHOLD		0x4000

static void dump_isi_regs(struct mxc_isi_dev *isi)
{
#ifdef DEBUG
	struct device *dev = isi->dev;
	struct {
		u32 offset;
		const char *const name;
	} registers[] = {
		{ 0x00, "CHNL_CTRL" },
		{ 0x04, "CHNL_IMG_CTRL" },
		{ 0x08, "CHNL_OUT_BUF_CTRL" },
		{ 0x0C, "CHNL_IMG_CFG" },
		{ 0x10, "CHNL_IER" },
		{ 0x14, "CHNL_STS" },
		{ 0x18, "CHNL_SCALE_FACTOR" },
		{ 0x1C, "CHNL_SCALE_OFFSET" },
		{ 0x20, "CHNL_CROP_ULC" },
		{ 0x24, "CHNL_CROP_LRC" },
		{ 0x28, "CHNL_CSC_COEFF0" },
		{ 0x2C, "CHNL_CSC_COEFF1" },
		{ 0x30, "CHNL_CSC_COEFF2" },
		{ 0x34, "CHNL_CSC_COEFF3" },
		{ 0x38, "CHNL_CSC_COEFF4" },
		{ 0x3C, "CHNL_CSC_COEFF5" },
		{ 0x40, "CHNL_ROI_0_ALPHA" },
		{ 0x44, "CHNL_ROI_0_ULC" },
		{ 0x48, "CHNL_ROI_0_LRC" },
		{ 0x4C, "CHNL_ROI_1_ALPHA" },
		{ 0x50, "CHNL_ROI_1_ULC" },
		{ 0x54, "CHNL_ROI_1_LRC" },
		{ 0x58, "CHNL_ROI_2_ALPHA" },
		{ 0x5C, "CHNL_ROI_2_ULC" },
		{ 0x60, "CHNL_ROI_2_LRC" },
		{ 0x64, "CHNL_ROI_3_ALPHA" },
		{ 0x68, "CHNL_ROI_3_ULC" },
		{ 0x6C, "CHNL_ROI_3_LRC" },
		{ 0x70, "CHNL_OUT_BUF1_ADDR_Y" },
		{ 0x74, "CHNL_OUT_BUF1_ADDR_U" },
		{ 0x78, "CHNL_OUT_BUF1_ADDR_V" },
		{ 0x7C, "CHNL_OUT_BUF_PITCH" },
		{ 0x80, "CHNL_IN_BUF_ADDR" },
		{ 0x84, "CHNL_IN_BUF_PITCH" },
		{ 0x88, "CHNL_MEM_RD_CTRL" },
		{ 0x8C, "CHNL_OUT_BUF2_ADDR_Y" },
		{ 0x90, "CHNL_OUT_BUF2_ADDR_U" },
		{ 0x94, "CHNL_OUT_BUF2_ADDR_V" },
		{ 0x98, "CHNL_SCL_IMG_CFG" },
		{ 0x9C, "CHNL_FLOW_CTRL" },
	};
	u32 i;

	dev_dbg(dev, "ISI CHNLC register dump, isi%d\n", isi->id);
	for (i = 0; i < ARRAY_SIZE(registers); i++) {
		u32 reg = readl(isi->regs + registers[i].offset);
		dev_dbg(dev, "%20s[0x%.2x]: %.2x\n",
			registers[i].name, registers[i].offset, reg);
	}
#endif
}

/* 
 * A2,A1,      B1, A3,     B3, B2,
 * C2, C1,     D1, C3,     D3, D2
 */
static const u32 coeffs[2][6] = {
	/* YUV2RGB */
	{ 0x0000012A, 0x012A0198, 0x0730079C,
	  0x0204012A, 0x01F00000, 0x01800180 },

	/* RGB->YUV */
	{ 0x00810041, 0x07db0019, 0x007007b6,
	  0x07a20070, 0x001007ee, 0x00800080 },
};

static void printk_pixelformat(char *prefix, int val)
{
	pr_info("%s %c%c%c%c\n", prefix ? prefix : "pixelformat",
		val & 0xff,
		(val >> 8)  & 0xff,
		(val >> 16) & 0xff,
		(val >> 24) & 0xff);
}

bool is_buf_active(struct mxc_isi_dev *isi, int buf_id)
{
	u32 status = isi->status;
	bool reverse = isi->pdata->buf_active_reverse;

	return (buf_id == 1) ? ((reverse) ? (status & 0x100) : (status & 0x200)) :
			       ((reverse) ? (status & 0x200) : (status & 0x100));
}

static void chain_buf(struct mxc_isi_dev *isi, const struct mxc_isi_frame *frm)
{
	u32 val;

	if (frm->format.width > ISI_2K) {
		val = readl(isi->regs + CHNL_CTRL);
		val &= ~CHNL_CTRL_CHAIN_BUF_MASK;
		val |= (CHNL_CTRL_CHAIN_BUF_2_CHAIN << CHNL_CTRL_CHAIN_BUF_OFFSET);
		writel(val, isi->regs + CHNL_CTRL);
		if (isi->chain)
			regmap_write(isi->chain, CHNL_CTRL, CHNL_CTRL_CLK_EN_MASK);
		isi->chain_buf = 1;
	} else {
		val = readl(isi->regs + CHNL_CTRL);
		val &= ~CHNL_CTRL_CHAIN_BUF_MASK;
		writel(val, isi->regs + CHNL_CTRL);
		isi->chain_buf = 0;
	}
}

void mxc_isi_channel_set_outbuf(struct mxc_isi_dev *isi,
				struct mxc_isi_buffer *buf)
{
	struct vb2_buffer *vb2_buf = &buf->v4l2_buf.vb2_buf;
	u32 framecount = buf->v4l2_buf.sequence;
	struct frame_addr *paddr = &buf->paddr;
	struct mxc_isi_pipe *pipe;
	struct v4l2_pix_format_mplane *pix;
	int val = 0;

	if (buf->discard) {
		pipe = &isi->pipe;
		pix = &pipe->video.pix;
		paddr->y = pipe->video.discard_buffer_dma[0];
		if (pix->num_planes == 2)
			paddr->cb = pipe->video.discard_buffer_dma[1];
		if (pix->num_planes == 3) {
			paddr->cb = pipe->video.discard_buffer_dma[1];
			paddr->cr = pipe->video.discard_buffer_dma[2];
		}
	} else {
		paddr->y = vb2_dma_contig_plane_dma_addr(vb2_buf, 0);

		if (vb2_buf->num_planes == 2)
			paddr->cb = vb2_dma_contig_plane_dma_addr(vb2_buf, 1);
		if (vb2_buf->num_planes == 3) {
			paddr->cb = vb2_dma_contig_plane_dma_addr(vb2_buf, 1);
			paddr->cr = vb2_dma_contig_plane_dma_addr(vb2_buf, 2);
		}
	}

	val = readl(isi->regs + CHNL_OUT_BUF_CTRL);

	if (framecount == 0 || ((is_buf_active(isi, 2)) && (framecount != 1))) {
		writel(paddr->y, isi->regs + CHNL_OUT_BUF1_ADDR_Y);
		writel(paddr->cb, isi->regs + CHNL_OUT_BUF1_ADDR_U);
		writel(paddr->cr, isi->regs + CHNL_OUT_BUF1_ADDR_V);
		val ^= CHNL_OUT_BUF_CTRL_LOAD_BUF1_ADDR_MASK;
		buf->id = MXC_ISI_BUF1;
	} else if (framecount == 1 || is_buf_active(isi, 1)) {
		writel(paddr->y, isi->regs + CHNL_OUT_BUF2_ADDR_Y);
		writel(paddr->cb, isi->regs + CHNL_OUT_BUF2_ADDR_U);
		writel(paddr->cr, isi->regs + CHNL_OUT_BUF2_ADDR_V);
		val ^= CHNL_OUT_BUF_CTRL_LOAD_BUF2_ADDR_MASK;
		buf->id = MXC_ISI_BUF2;
	}
	writel(val, isi->regs + CHNL_OUT_BUF_CTRL);
}

static void mxc_isi_channel_sw_reset(struct mxc_isi_dev *isi)
{
	u32 val;

	val = readl(isi->regs + CHNL_CTRL);
	val |= CHNL_CTRL_SW_RST;
	writel(val, isi->regs + CHNL_CTRL);
	mdelay(5);
	val &= ~CHNL_CTRL_SW_RST;
	writel(val, isi->regs + CHNL_CTRL);
}

static void mxc_isi_channel_source_config(struct mxc_isi_dev *isi)
{
	u32 val;

	val = readl(isi->regs + CHNL_CTRL);
	val &= ~(CHNL_CTRL_MIPI_VC_ID_MASK |
		 CHNL_CTRL_SRC_INPUT_MASK | CHNL_CTRL_SRC_TYPE_MASK);

	switch (isi->interface[IN_PORT]) {
	case ISI_INPUT_INTERFACE_MIPI0_CSI2:
		val |= isi->pdata->chan_src->src_mipi0;
		val |= 0 << CHNL_CTRL_MIPI_VC_ID_OFFSET;
		break;
	case ISI_INPUT_INTERFACE_MIPI1_CSI2:
		val |= isi->pdata->chan_src->src_mipi1;
		val |= 0 << CHNL_CTRL_MIPI_VC_ID_OFFSET;
		break;
	case ISI_INPUT_INTERFACE_DC0:
		val |= isi->pdata->chan_src->src_dc0;
		break;
	case ISI_INPUT_INTERFACE_DC1:
		val |= isi->pdata->chan_src->src_dc1;
		break;
	case ISI_INPUT_INTERFACE_HDMI:
		val |= isi->pdata->chan_src->src_hdmi;
		break;
	case ISI_INPUT_INTERFACE_PARALLEL_CSI:
		val |= isi->pdata->chan_src->src_csi;
		break;
	case ISI_INPUT_INTERFACE_MEM:
		val |= isi->pdata->chan_src->src_mem;
		val |= (CHNL_CTRL_SRC_TYPE_MEMORY << CHNL_CTRL_SRC_TYPE_OFFSET);
		break;
	default:
		dev_err(isi->dev, "invalid interface\n");
		break;
	}

	writel(val, isi->regs + CHNL_CTRL);
}

void mxc_isi_channel_set_flip(struct mxc_isi_dev *isi)
{
	u32 val;

	val = readl(isi->regs + CHNL_IMG_CTRL);
	val &= ~(CHNL_IMG_CTRL_VFLIP_EN_MASK | CHNL_IMG_CTRL_HFLIP_EN_MASK);

	if (isi->vflip)
		val |= (CHNL_IMG_CTRL_VFLIP_EN_ENABLE << CHNL_IMG_CTRL_VFLIP_EN_OFFSET);
	if (isi->hflip)
		val |= (CHNL_IMG_CTRL_HFLIP_EN_ENABLE << CHNL_IMG_CTRL_HFLIP_EN_OFFSET);

	writel(val, isi->regs + CHNL_IMG_CTRL);
}

static void mxc_isi_channel_set_csc(struct mxc_isi_dev *isi,
				    const struct mxc_isi_frame *src_f,
				    const struct mxc_isi_frame *dst_f)
{
	const struct mxc_isi_format_info *src_fmt = src_f->info;
	const struct mxc_isi_format_info *dst_fmt = dst_f->info;
	u32 val, csc = 0;

	val = readl(isi->regs + CHNL_IMG_CTRL);
	val &= ~(CHNL_IMG_CTRL_FORMAT_MASK |
		 CHNL_IMG_CTRL_YCBCR_MODE_MASK |
		 CHNL_IMG_CTRL_CSC_BYPASS_MASK |
		 CHNL_IMG_CTRL_CSC_MODE_MASK);

	/* set outbuf format */
	val |= dst_fmt->color << CHNL_IMG_CTRL_FORMAT_OFFSET;

	isi->cscen = 1;

	if (src_fmt->colorspace == MXC_ISI_CS_YUV &&
	    dst_fmt->colorspace == MXC_ISI_CS_RGB) {
		/* YUV2RGB */
		csc = YUV2RGB;
		/* YCbCr enable???  */
		val |= (CHNL_IMG_CTRL_CSC_MODE_YCBCR2RGB << CHNL_IMG_CTRL_CSC_MODE_OFFSET);
		val |= (CHNL_IMG_CTRL_YCBCR_MODE_ENABLE << CHNL_IMG_CTRL_YCBCR_MODE_OFFSET);
	} else if (src_fmt->colorspace == MXC_ISI_CS_RGB &&
		   dst_fmt->colorspace == MXC_ISI_CS_YUV) {
		/* RGB2YUV */
		csc = RGB2YUV;
		val |= (CHNL_IMG_CTRL_CSC_MODE_RGB2YCBCR << CHNL_IMG_CTRL_CSC_MODE_OFFSET);
	} else {
		/* Bypass CSC */
		pr_info("bypass csc\n");
		isi->cscen = 0;
		val |= CHNL_IMG_CTRL_CSC_BYPASS_ENABLE;
	}

	pr_info("input colorspace %u", src_fmt->colorspace);
	printk_pixelformat("output fmt", dst_fmt->fourcc);

	if (isi->cscen) {
		writel(coeffs[csc][0], isi->regs + CHNL_CSC_COEFF0);
		writel(coeffs[csc][1], isi->regs + CHNL_CSC_COEFF1);
		writel(coeffs[csc][2], isi->regs + CHNL_CSC_COEFF2);
		writel(coeffs[csc][3], isi->regs + CHNL_CSC_COEFF3);
		writel(coeffs[csc][4], isi->regs + CHNL_CSC_COEFF4);
		writel(coeffs[csc][5], isi->regs + CHNL_CSC_COEFF5);
	}

	writel(val, isi->regs + CHNL_IMG_CTRL);
}

void mxc_isi_channel_set_alpha(struct mxc_isi_dev *isi)
{
	u32 val;

	val = readl(isi->regs + CHNL_IMG_CTRL);
	val &= ~(CHNL_IMG_CTRL_GBL_ALPHA_VAL_MASK | CHNL_IMG_CTRL_GBL_ALPHA_EN_MASK);

	if (isi->alphaen)
		val |= ((isi->alpha << CHNL_IMG_CTRL_GBL_ALPHA_VAL_OFFSET) |
			(CHNL_IMG_CTRL_GBL_ALPHA_EN_ENABLE << CHNL_IMG_CTRL_GBL_ALPHA_EN_OFFSET));

	writel(val, isi->regs + CHNL_IMG_CTRL);
}

static void mxc_isi_channel_set_panic_threshold(struct mxc_isi_dev *isi)
{
	const struct mxc_isi_set_thd *set_thd = isi->pdata->set_thd;
	u32 val;

	val = readl(isi->regs + CHNL_OUT_BUF_CTRL);

	val &= ~(set_thd->panic_set_thd_y.mask);
	val |= set_thd->panic_set_thd_y.threshold << set_thd->panic_set_thd_y.offset;

	val &= ~(set_thd->panic_set_thd_u.mask);
	val |= set_thd->panic_set_thd_u.threshold << set_thd->panic_set_thd_u.offset;

	val &= ~(set_thd->panic_set_thd_v.mask);
	val |= set_thd->panic_set_thd_v.threshold << set_thd->panic_set_thd_v.offset;

	writel(val, isi->regs + CHNL_OUT_BUF_CTRL);
}

void mxc_isi_channel_set_chain_buf(struct mxc_isi_dev *isi)
{
	u32 val;

	if (isi->chain_buf) {
		val = readl(isi->regs + CHNL_CTRL);
		val &= ~CHNL_CTRL_CHAIN_BUF_MASK;
		val |= (CHNL_CTRL_CHAIN_BUF_2_CHAIN << CHNL_CTRL_CHAIN_BUF_OFFSET);

		writel(val, isi->regs + CHNL_CTRL);
	}
}

void mxc_isi_channel_set_crop(struct mxc_isi_dev *isi)
{
	const struct mxc_isi_frame *src_f = &isi->pipe.formats[MXC_ISI_SD_PAD_SINK];
	const struct mxc_isi_frame *dst_f = &isi->pipe.formats[MXC_ISI_SD_PAD_SOURCE];
	u32 val, val0, val1;

	val = readl(isi->regs + CHNL_IMG_CTRL);
	val &= ~CHNL_IMG_CTRL_CROP_EN_MASK;

	/*
	 * FIXME: To take advantage of scaler phase configuration, and to allow
	 * digital zoom use cases, we should expose a crop rectangle on the
	 * sink pad and convert it to the post-scaler crop rectangle
	 * internally.
	 */
	if ((src_f->compose.height == dst_f->crop.height) &&
	    (src_f->compose.width == dst_f->crop.width)) {
		isi->crop = 0;
		writel(val, isi->regs + CHNL_IMG_CTRL);
		return;
	}

	isi->crop = 1;
	val |= (CHNL_IMG_CTRL_CROP_EN_ENABLE << CHNL_IMG_CTRL_CROP_EN_OFFSET);
	val0 = dst_f->crop.top | (dst_f->crop.left << CHNL_CROP_ULC_X_OFFSET);
	val1 = dst_f->crop.height | (dst_f->crop.width << CHNL_CROP_LRC_X_OFFSET);

	writel(val0, isi->regs + CHNL_CROP_ULC);
	writel((val1 + val0), isi->regs + CHNL_CROP_LRC);
	writel(val, isi->regs + CHNL_IMG_CTRL);
}

static void mxc_isi_channel_clear_scaling(struct mxc_isi_dev *isi)
{
	u32 val0;

	writel(0x10001000, isi->regs + CHNL_SCALE_FACTOR);

	val0 = readl(isi->regs + CHNL_IMG_CTRL);
	val0 &= ~(CHNL_IMG_CTRL_DEC_X_MASK | CHNL_IMG_CTRL_DEC_Y_MASK);
	writel(val0, isi->regs + CHNL_IMG_CTRL);
}

static void mxc_isi_channel_set_scaling(struct mxc_isi_dev *isi,
					const struct mxc_isi_frame *src_f,
					const struct mxc_isi_frame *dst_f)
{
	u32 decx, decy;
	u32 xscale, yscale;
	u32 xdec = 0, ydec = 0;
	u32 val0, val1;

	dev_dbg(isi->dev, "input_size %ux%u, output_size %ux%u\n",
		src_f->format.width, src_f->format.height,
		src_f->compose.width, src_f->compose.height);

	if (src_f->format.height == src_f->compose.height &&
	    src_f->format.width == src_f->compose.width) {
		isi->scale = 0;
		mxc_isi_channel_clear_scaling(isi);
		dev_dbg(isi->dev, "%s: no scale\n", __func__);
		return;
	}

	isi->scale = 1;

	decx = src_f->format.width / src_f->compose.width;
	decy = src_f->format.height / src_f->compose.height;

	if (decx > 1) {
		/* Down */
		if (decx >= 2 && decx < 4) {
			decx = 2;
			xdec = 1;
		} else if (decx >= 4 && decx < 8) {
			decx = 4;
			xdec = 2;
		} else if (decx >= 8) {
			decx = 8;
			xdec = 3;
		}
		xscale = src_f->format.width * 0x1000
		       / (src_f->compose.width * decx);
	} else {
		/* Up  */
		xscale = src_f->format.width * 0x1000 / src_f->compose.width;
	}

	if (decy > 1) {
		if (decy >= 2 && decy < 4) {
			decy = 2;
			ydec = 1;
		} else if (decy >= 4 && decy < 8) {
			decy = 4;
			ydec = 2;
		} else if (decy >= 8) {
			decy = 8;
			ydec = 3;
		}
		yscale = src_f->format.height * 0x1000
		       / (src_f->compose.height * decy);
	} else {
		yscale = src_f->format.height * 0x1000 / src_f->compose.height;
	}

	val0 = readl(isi->regs + CHNL_IMG_CTRL);
	val0 |= CHNL_IMG_CTRL_YCBCR_MODE_MASK;//YCbCr  Sandor???
	val0 &= ~(CHNL_IMG_CTRL_DEC_X_MASK | CHNL_IMG_CTRL_DEC_Y_MASK);
	val0 |= (xdec << CHNL_IMG_CTRL_DEC_X_OFFSET) |
			(ydec << CHNL_IMG_CTRL_DEC_Y_OFFSET);
	writel(val0, isi->regs + CHNL_IMG_CTRL);

	if (xscale > ISI_DOWNSCALE_THRESHOLD)
		xscale = ISI_DOWNSCALE_THRESHOLD;
	if (yscale > ISI_DOWNSCALE_THRESHOLD)
		yscale = ISI_DOWNSCALE_THRESHOLD;

	val1 = xscale | (yscale << CHNL_SCALE_FACTOR_Y_SCALE_OFFSET);

	writel(val1, isi->regs + CHNL_SCALE_FACTOR);

	/* Update scale config if scaling enabled */
	val1 = (src_f->compose.height << CHNL_SCL_IMG_CFG_HEIGHT_OFFSET)
	     | src_f->compose.width;
	writel(val1, isi->regs + CHNL_SCL_IMG_CFG);

	writel(0, isi->regs + CHNL_SCALE_OFFSET);

	return;
}

void mxc_isi_channel_init(struct mxc_isi_dev *isi)
{
	u32 val;

	/* sw reset */
	mxc_isi_channel_sw_reset(isi);

	/* Init channel clk first */
	val = readl(isi->regs + CHNL_CTRL);
	val |= (CHNL_CTRL_CLK_EN_ENABLE << CHNL_CTRL_CLK_EN_OFFSET);
	writel(val, isi->regs + CHNL_CTRL);
}

void mxc_isi_channel_deinit(struct mxc_isi_dev *isi)
{
	u32 val;

	/* sw reset */
	mxc_isi_channel_sw_reset(isi);

	/* deinit channel clk first */
	val = (CHNL_CTRL_CLK_EN_DISABLE << CHNL_CTRL_CLK_EN_OFFSET);
	writel(val, isi->regs + CHNL_CTRL);

	if (isi->chain_buf && isi->chain)
		regmap_write(isi->chain, CHNL_CTRL, 0x0);
}

void mxc_isi_channel_config(struct mxc_isi_dev *isi,
			    const struct mxc_isi_frame *src_f,
			    const struct mxc_isi_frame *dst_f,
			    unsigned int pitch)
{
	u32 val;

	/* images having higher than 2048 horizontal resolution */
	chain_buf(isi, src_f);

	/* config output frame size and format */
	val = (src_f->format.height << CHNL_IMG_CFG_HEIGHT_OFFSET)
	    | src_f->format.width;
	writel(val, isi->regs + CHNL_IMG_CFG);

	/* scale size need to equal input size when scaling disabled*/
	writel(val, isi->regs + CHNL_SCL_IMG_CFG);

	/* check csc and scaling  */
	mxc_isi_channel_set_csc(isi, src_f, dst_f);

	mxc_isi_channel_set_scaling(isi, src_f, dst_f);

	/* select the source input / src type / virtual channel for mipi*/
	mxc_isi_channel_source_config(isi);

	/* line pitch */
	writel(pitch, isi->regs + CHNL_OUT_BUF_PITCH);

	/* TODO */
	mxc_isi_channel_set_flip(isi);

	mxc_isi_channel_set_alpha(isi);

	mxc_isi_channel_set_panic_threshold(isi);

	val = readl(isi->regs + CHNL_CTRL);
	val &= ~CHNL_CTRL_CHNL_BYPASS_MASK;

	/*  Bypass channel */
	if (!isi->cscen && !isi->scale)
		val |= (CHNL_CTRL_CHNL_BYPASS_ENABLE << CHNL_CTRL_CHNL_BYPASS_OFFSET);

	writel(val, isi->regs + CHNL_CTRL);
}

void mxc_isi_clean_registers(struct mxc_isi_dev *isi)
{
	mxc_isi_get_irq_status(isi);
}

static void mxc_isi_enable_irq(struct mxc_isi_dev *isi)
{
	const struct mxc_isi_ier_reg *ier_reg = isi->pdata->ier_reg;
	u32 val;

	val = CHNL_IER_FRM_RCVD_EN_MASK |
		CHNL_IER_AXI_WR_ERR_U_EN_MASK |
		CHNL_IER_AXI_WR_ERR_V_EN_MASK |
		CHNL_IER_AXI_WR_ERR_Y_EN_MASK;

	/* Y/U/V overflow enable */
	val |= ier_reg->oflw_y_buf_en.mask |
	       ier_reg->oflw_u_buf_en.mask |
	       ier_reg->oflw_v_buf_en.mask;

	/* Y/U/V excess overflow enable */
	val |= ier_reg->excs_oflw_y_buf_en.mask |
	       ier_reg->excs_oflw_u_buf_en.mask |
	       ier_reg->excs_oflw_v_buf_en.mask;

	/* Y/U/V panic enable */
	val |= ier_reg->panic_y_buf_en.mask |
	       ier_reg->panic_u_buf_en.mask |
	       ier_reg->panic_v_buf_en.mask;

	writel(val, isi->regs + CHNL_IER);
}

static void mxc_isi_disable_irq(struct mxc_isi_dev *isi)
{
	writel(0, isi->regs + CHNL_IER);
}

void mxc_isi_channel_enable(struct mxc_isi_dev *isi, bool m2m_enabled)
{
	u32 val;

	val = readl(isi->regs + CHNL_CTRL);
	val |= 0xff << CHNL_CTRL_BLANK_PXL_OFFSET;

	if (m2m_enabled) {
		val &= ~(CHNL_CTRL_SRC_TYPE_MASK | CHNL_CTRL_SRC_INPUT_MASK);
		val |= (isi->pdata->chan_src->src_mem << CHNL_CTRL_SRC_INPUT_OFFSET |
			CHNL_CTRL_SRC_TYPE_MEMORY << CHNL_CTRL_SRC_TYPE_OFFSET);
	}

	val &= ~CHNL_CTRL_CHNL_EN_MASK;
	val |= CHNL_CTRL_CHNL_EN_ENABLE << CHNL_CTRL_CHNL_EN_OFFSET;
	writel(val, isi->regs + CHNL_CTRL);

	mxc_isi_clean_registers(isi);
	mxc_isi_enable_irq(isi);

#if 0
	if (m2m_enabled) {
		mxc_isi_m2m_start_read(isi);
		return;
	}
#endif

	dump_isi_regs(isi);
	msleep(300);
}

void mxc_isi_channel_disable(struct mxc_isi_dev *isi)
{
	u32 val;

	mxc_isi_disable_irq(isi);

	val = readl(isi->regs + CHNL_CTRL);
	val &= ~(CHNL_CTRL_CHNL_EN_MASK | CHNL_CTRL_CLK_EN_MASK);
	val |= (CHNL_CTRL_CHNL_EN_DISABLE << CHNL_CTRL_CHNL_EN_OFFSET);
	val |= (CHNL_CTRL_CLK_EN_DISABLE << CHNL_CTRL_CLK_EN_OFFSET);
	writel(val, isi->regs + CHNL_CTRL);
}

u32 mxc_isi_get_irq_status(struct mxc_isi_dev *isi)
{
	u32 status;

	status = readl(isi->regs + CHNL_STS);
	writel(status, isi->regs + CHNL_STS);
	return status;
}
