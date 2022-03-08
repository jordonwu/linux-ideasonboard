// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2019-2020 NXP
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/types.h>

#include "imx8-isi-core.h"
#include "imx8-isi-regs.h"

#define	ISI_DOWNSCALE_THRESHOLD		0x4000

static inline u32 mxc_isi_read(struct mxc_isi_pipe *pipe, u32 reg)
{
	return readl(pipe->regs + reg);
}

static inline void mxc_isi_write(struct mxc_isi_pipe *pipe, u32 reg,
				      u32 val)
{
	writel(val, pipe->regs + reg);
}

static void mxc_isi_pipe_dump_regs(struct mxc_isi_pipe *pipe)
{
#ifdef DEBUG
	struct device *dev = pipe->isi->dev;
	struct {
		u32 offset;
		const char *const name;
	} registers[] = {
		{ 0x00, "CHNL_CTRL" },
		{ 0x04, "CHNL_IMG_CTRL" },
		{ 0x08, "CHNL_OUT_BUF_CTRL" },
		{ 0x0c, "CHNL_IMG_CFG" },
		{ 0x10, "CHNL_IER" },
		{ 0x14, "CHNL_STS" },
		{ 0x18, "CHNL_SCALE_FACTOR" },
		{ 0x1c, "CHNL_SCALE_OFFSET" },
		{ 0x20, "CHNL_CROP_ULC" },
		{ 0x24, "CHNL_CROP_LRC" },
		{ 0x28, "CHNL_CSC_COEFF0" },
		{ 0x2c, "CHNL_CSC_COEFF1" },
		{ 0x30, "CHNL_CSC_COEFF2" },
		{ 0x34, "CHNL_CSC_COEFF3" },
		{ 0x38, "CHNL_CSC_COEFF4" },
		{ 0x3c, "CHNL_CSC_COEFF5" },
		{ 0x40, "CHNL_ROI_0_ALPHA" },
		{ 0x44, "CHNL_ROI_0_ULC" },
		{ 0x48, "CHNL_ROI_0_LRC" },
		{ 0x4c, "CHNL_ROI_1_ALPHA" },
		{ 0x50, "CHNL_ROI_1_ULC" },
		{ 0x54, "CHNL_ROI_1_LRC" },
		{ 0x58, "CHNL_ROI_2_ALPHA" },
		{ 0x5c, "CHNL_ROI_2_ULC" },
		{ 0x60, "CHNL_ROI_2_LRC" },
		{ 0x64, "CHNL_ROI_3_ALPHA" },
		{ 0x68, "CHNL_ROI_3_ULC" },
		{ 0x6c, "CHNL_ROI_3_LRC" },
		{ 0x70, "CHNL_OUT_BUF1_ADDR_Y" },
		{ 0x74, "CHNL_OUT_BUF1_ADDR_U" },
		{ 0x78, "CHNL_OUT_BUF1_ADDR_V" },
		{ 0x7c, "CHNL_OUT_BUF_PITCH" },
		{ 0x80, "CHNL_IN_BUF_ADDR" },
		{ 0x84, "CHNL_IN_BUF_PITCH" },
		{ 0x88, "CHNL_MEM_RD_CTRL" },
		{ 0x8c, "CHNL_OUT_BUF2_ADDR_Y" },
		{ 0x90, "CHNL_OUT_BUF2_ADDR_U" },
		{ 0x94, "CHNL_OUT_BUF2_ADDR_V" },
		{ 0x98, "CHNL_SCL_IMG_CFG" },
		{ 0x9c, "CHNL_FLOW_CTRL" },
	};
	u32 i;

	dev_dbg(dev, "ISI CHNLC register dump, pipe%d\n", pipe->id);
	for (i = 0; i < ARRAY_SIZE(registers); i++) {
		u32 reg = mxc_isi_pip_read(pipe, registers[i].offset);
		dev_dbg(dev, "%20s[0x%.2x]: %.2x\n",
			registers[i].name, registers[i].offset, reg);
	}
#endif
}

/* -----------------------------------------------------------------------------
 * Buffers
 */

void mxc_isi_channel_set_outbuf(struct mxc_isi_pipe *pipe,
				struct mxc_isi_buffer *buf,
				enum mxc_isi_buf_id buf_id)
{
	int val;

	buf->id = buf_id;

	val = mxc_isi_read(pipe, CHNL_OUT_BUF_CTRL);

	if (buf_id == MXC_ISI_BUF1) {
		mxc_isi_write(pipe, CHNL_OUT_BUF1_ADDR_Y, buf->dma_addrs[0]);
		mxc_isi_write(pipe, CHNL_OUT_BUF1_ADDR_U, buf->dma_addrs[1]);
		mxc_isi_write(pipe, CHNL_OUT_BUF1_ADDR_V, buf->dma_addrs[2]);
		val ^= CHNL_OUT_BUF_CTRL_LOAD_BUF1_ADDR;
	} else  {
		mxc_isi_write(pipe, CHNL_OUT_BUF2_ADDR_Y, buf->dma_addrs[0]);
		mxc_isi_write(pipe, CHNL_OUT_BUF2_ADDR_U, buf->dma_addrs[1]);
		mxc_isi_write(pipe, CHNL_OUT_BUF2_ADDR_V, buf->dma_addrs[2]);
		val ^= CHNL_OUT_BUF_CTRL_LOAD_BUF2_ADDR;
	}

	mxc_isi_write(pipe, CHNL_OUT_BUF_CTRL, val);
}

/* -----------------------------------------------------------------------------
 * Pipeline configuration
 */

static void mxc_isi_channel_source_config(struct mxc_isi_pipe *pipe,
					  unsigned int input)
{
	u32 val;

	val = mxc_isi_read(pipe, CHNL_CTRL);
	val &= ~(CHNL_CTRL_MIPI_VC_ID_MASK | CHNL_CTRL_SRC_TYPE_MASK |
		  CHNL_CTRL_SRC_INPUT_MASK);

	val |= CHNL_CTRL_SRC_INPUT(input);
	val |= CHNL_CTRL_MIPI_VC_ID(0); /* FIXME: For CSI-2 only */

	/*
	 * FIXME: Support memory input
	 * val |= CHNL_CTRL_SRC_TYPE(CHNL_CTRL_SRC_TYPE_MEMORY);
	 */

	mxc_isi_write(pipe, CHNL_CTRL, val);
}

static void mxc_isi_chain_buf(struct mxc_isi_pipe *pipe,
			      const struct v4l2_mbus_framefmt *format)
{
	u32 val;

	if (format->width > ISI_2K) {
		val = mxc_isi_read(pipe, CHNL_CTRL);
		val &= ~CHNL_CTRL_CHAIN_BUF_MASK;
		val |= CHNL_CTRL_CHAIN_BUF(CHNL_CTRL_CHAIN_BUF_2_CHAIN);
		mxc_isi_write(pipe, CHNL_CTRL, val);
		mxc_isi_write(pipe + 1, CHNL_CTRL, CHNL_CTRL_CLK_EN);
		pipe->chain_buf = 1;
	} else {
		val = mxc_isi_read(pipe, CHNL_CTRL);
		val &= ~CHNL_CTRL_CHAIN_BUF_MASK;
		mxc_isi_write(pipe, CHNL_CTRL, val);
		pipe->chain_buf = 0;
	}
}

static u32 mxc_isi_channel_scaling_ratio(unsigned int from, unsigned int to,
					 u32 *dec)
{
	unsigned int ratio = from / to;

	if (ratio < 2)
		*dec = 1;
	else if (ratio < 4)
		*dec = 2;
	else if (ratio < 8)
		*dec = 4;
	else
		*dec = 8;

	return min_t(u32, from * 0x1000 / (to * *dec), ISI_DOWNSCALE_THRESHOLD);
}

static void mxc_isi_channel_set_scaling(struct mxc_isi_pipe *pipe,
					enum mxc_isi_encoding src_encoding,
					const struct v4l2_mbus_framefmt *format,
					const struct v4l2_rect *compose,
					bool *bypass)
{
	u32 xscale, yscale;
	u32 decx, decy;
	u32 val;

	dev_dbg(pipe->isi->dev, "input_size %ux%u, output_size %ux%u\n",
		format->width, format->height, compose->width, compose->height);

	xscale = mxc_isi_channel_scaling_ratio(format->width, compose->width,
					       &decx);
	yscale = mxc_isi_channel_scaling_ratio(format->height, compose->height,
					       &decy);

	val = mxc_isi_read(pipe, CHNL_IMG_CTRL);
	val &= ~(CHNL_IMG_CTRL_DEC_X_MASK | CHNL_IMG_CTRL_DEC_Y_MASK |
		 CHNL_IMG_CTRL_YCBCR_MODE);

	val |= CHNL_IMG_CTRL_DEC_X(ilog2(decx))
	    |  CHNL_IMG_CTRL_DEC_Y(ilog2(decy));

	/*
	 * Contrary to what the documentation states, YCBCR_MODE does not
	 * control conversion between YCbCr and RGB, but whether the scaler
	 * operates in YUV mode or in RGB mode. It must be set when the scaler
	 * input is YUV.
	 */
	if (src_encoding == MXC_ISI_ENC_YUV)
		val |= CHNL_IMG_CTRL_YCBCR_MODE;

	mxc_isi_write(pipe, CHNL_IMG_CTRL, val);

	mxc_isi_write(pipe, CHNL_SCALE_FACTOR,
		      CHNL_SCALE_FACTOR_Y_SCALE(yscale) |
		      CHNL_SCALE_FACTOR_X_SCALE(xscale));

	mxc_isi_write(pipe, CHNL_SCALE_OFFSET, 0);

	mxc_isi_write(pipe, CHNL_SCL_IMG_CFG,
		      CHNL_SCL_IMG_CFG_HEIGHT(compose->height) |
		      CHNL_SCL_IMG_CFG_WIDTH(compose->width));

	*bypass = format->height == compose->height &&
		  format->width == compose->width;
}

static void mxc_isi_channel_set_crop(struct mxc_isi_pipe *pipe,
				     const struct v4l2_rect *src,
				     const struct v4l2_rect *dst)
{
	u32 val, val0, val1;

	val = mxc_isi_read(pipe, CHNL_IMG_CTRL);
	val &= ~CHNL_IMG_CTRL_CROP_EN;

	if (src->height == dst->height && src->width == dst->width) {
		mxc_isi_write(pipe, CHNL_IMG_CTRL, val);
		return;
	}

	val |= CHNL_IMG_CTRL_CROP_EN;
	val0 = CHNL_CROP_ULC_X(dst->left) | CHNL_CROP_ULC_Y(dst->top);
	val1 = CHNL_CROP_LRC_X(dst->width) | CHNL_CROP_LRC_Y(dst->height);

	mxc_isi_write(pipe, CHNL_CROP_ULC, val0);
	mxc_isi_write(pipe, CHNL_CROP_LRC, val1 + val0);
	mxc_isi_write(pipe, CHNL_IMG_CTRL, val);
}

/* 
 * A2,A1,      B1, A3,     B3, B2,
 * C2, C1,     D1, C3,     D3, D2
 */
static const u32 mxc_isi_coeffs[2][6] = {
	/* YUV2RGB */
	{ 0x0000012a, 0x012A0198, 0x0730079C,
	  0x0204012a, 0x01F00000, 0x01800180 },

	/* RGB->YUV */
	{ 0x00810041, 0x07db0019, 0x007007b6,
	  0x07a20070, 0x001007ee, 0x00800080 },
};

static void mxc_isi_channel_set_csc(struct mxc_isi_pipe *pipe,
				    enum mxc_isi_encoding src_encoding,
				    enum mxc_isi_encoding dst_encoding,
				    bool *bypass)
{
	static const char * const encodings[] = {
		[MXC_ISI_ENC_RAW] = "RAW",
		[MXC_ISI_ENC_RGB] = "RGB",
		[MXC_ISI_ENC_YUV] = "YUV",
	};
	bool cscen = true;
	u32 val, csc = 0;

	val = mxc_isi_read(pipe, CHNL_IMG_CTRL);
	val &= ~(CHNL_IMG_CTRL_CSC_BYPASS | CHNL_IMG_CTRL_CSC_MODE_MASK);

	if (src_encoding == MXC_ISI_ENC_YUV &&
	    dst_encoding == MXC_ISI_ENC_RGB) {
		/* YUV2RGB */
		csc = YUV2RGB;
		/* YCbCr enable???  */
		val |= CHNL_IMG_CTRL_CSC_MODE(CHNL_IMG_CTRL_CSC_MODE_YCBCR2RGB);
	} else if (src_encoding == MXC_ISI_ENC_RGB &&
		   dst_encoding == MXC_ISI_ENC_YUV) {
		/* RGB2YUV */
		csc = RGB2YUV;
		val |= CHNL_IMG_CTRL_CSC_MODE(CHNL_IMG_CTRL_CSC_MODE_RGB2YCBCR);
	} else {
		/* Bypass CSC */
		cscen = false;
		val |= CHNL_IMG_CTRL_CSC_BYPASS;
	}

	dev_dbg(pipe->isi->dev, "CSC: %s -> %s\n",
		encodings[src_encoding], encodings[dst_encoding]);

	if (cscen) {
		mxc_isi_write(pipe, CHNL_CSC_COEFF0, mxc_isi_coeffs[csc][0]);
		mxc_isi_write(pipe, CHNL_CSC_COEFF1, mxc_isi_coeffs[csc][1]);
		mxc_isi_write(pipe, CHNL_CSC_COEFF2, mxc_isi_coeffs[csc][2]);
		mxc_isi_write(pipe, CHNL_CSC_COEFF3, mxc_isi_coeffs[csc][3]);
		mxc_isi_write(pipe, CHNL_CSC_COEFF4, mxc_isi_coeffs[csc][4]);
		mxc_isi_write(pipe, CHNL_CSC_COEFF5, mxc_isi_coeffs[csc][5]);
	}

	mxc_isi_write(pipe, CHNL_IMG_CTRL, val);

	*bypass = !cscen;
}

static void mxc_isi_channel_set_alpha(struct mxc_isi_pipe *pipe)
{
	u32 val;

	val = mxc_isi_read(pipe, CHNL_IMG_CTRL);
	val &= ~CHNL_IMG_CTRL_GBL_ALPHA_VAL_MASK;
	val |= CHNL_IMG_CTRL_GBL_ALPHA_VAL(pipe->alpha) |
	       CHNL_IMG_CTRL_GBL_ALPHA_EN;
	mxc_isi_write(pipe, CHNL_IMG_CTRL, val);
}

static void mxc_isi_channel_set_flip(struct mxc_isi_pipe *pipe)
{
	u32 val;

	val = mxc_isi_read(pipe, CHNL_IMG_CTRL);
	val &= ~(CHNL_IMG_CTRL_VFLIP_EN | CHNL_IMG_CTRL_HFLIP_EN);

	if (pipe->vflip)
		val |= CHNL_IMG_CTRL_VFLIP_EN;
	if (pipe->hflip)
		val |= CHNL_IMG_CTRL_HFLIP_EN;

	mxc_isi_write(pipe, CHNL_IMG_CTRL, val);
}

static void mxc_isi_channel_set_panic_threshold(struct mxc_isi_pipe *pipe)
{
	const struct mxc_isi_set_thd *set_thd = pipe->isi->pdata->set_thd;
	u32 val;

	val = mxc_isi_read(pipe, CHNL_OUT_BUF_CTRL);

	val &= ~(set_thd->panic_set_thd_y.mask);
	val |= set_thd->panic_set_thd_y.threshold << set_thd->panic_set_thd_y.offset;

	val &= ~(set_thd->panic_set_thd_u.mask);
	val |= set_thd->panic_set_thd_u.threshold << set_thd->panic_set_thd_u.offset;

	val &= ~(set_thd->panic_set_thd_v.mask);
	val |= set_thd->panic_set_thd_v.threshold << set_thd->panic_set_thd_v.offset;

	mxc_isi_write(pipe, CHNL_OUT_BUF_CTRL, val);
}

void mxc_isi_channel_config(struct mxc_isi_pipe *pipe, unsigned int input,
			    const struct v4l2_mbus_framefmt *src_format,
			    const struct v4l2_rect *scale,
			    const struct v4l2_rect *crop,
			    enum mxc_isi_encoding src_encoding,
			    enum mxc_isi_encoding dst_encoding)
{
	bool csc_bypass;
	bool scaler_bypass;
	u32 val;

	/* Input source (including VC configuration for CSI-2) */
	mxc_isi_channel_source_config(pipe, input);

	/* Input frame size */
	mxc_isi_write(pipe, CHNL_IMG_CFG,
		      CHNL_IMG_CFG_HEIGHT(src_format->height) |
		      CHNL_IMG_CFG_WIDTH(src_format->width));

	/* Scaling */
	mxc_isi_chain_buf(pipe, src_format);
	mxc_isi_channel_set_scaling(pipe, src_encoding, src_format, scale,
				    &scaler_bypass);
	mxc_isi_channel_set_crop(pipe, scale, crop);

	/* CSC */
	mxc_isi_channel_set_csc(pipe, src_encoding, dst_encoding, &csc_bypass);

	/* Output buffer management, including global alpha and flipping */
	mxc_isi_channel_set_alpha(pipe);
	mxc_isi_channel_set_flip(pipe);

	mxc_isi_channel_set_panic_threshold(pipe);

	val = mxc_isi_read(pipe, CHNL_CTRL);
	val &= ~CHNL_CTRL_CHNL_BYPASS;

	/*
	 * If no scaling or color space conversion is needed, bypass the
	 * channel.
	 */
	if (csc_bypass && scaler_bypass)
		val |= CHNL_CTRL_CHNL_BYPASS;

	mxc_isi_write(pipe, CHNL_CTRL, val);
}

void mxc_isi_channel_set_output_format(struct mxc_isi_pipe *pipe,
				       const struct mxc_isi_format_info *info,
				       struct v4l2_pix_format_mplane *format)
{
	u32 val;

	/* set outbuf format */
	dev_dbg(pipe->isi->dev, "output format %p4cc", &format->pixelformat);

	val = mxc_isi_read(pipe, CHNL_IMG_CTRL);
	val &= ~CHNL_IMG_CTRL_FORMAT_MASK;
	val |= CHNL_IMG_CTRL_FORMAT(info->isi_format);
	mxc_isi_write(pipe, CHNL_IMG_CTRL, val);

	/* line pitch */
	mxc_isi_write(pipe, CHNL_OUT_BUF_PITCH,
		      format->plane_fmt[0].bytesperline);
}

/* -----------------------------------------------------------------------------
 * IRQ
 */

u32 mxc_isi_channel_irq_status(struct mxc_isi_pipe *pipe, bool clear)
{
	u32 status;

	status = mxc_isi_read(pipe, CHNL_STS);
	if (clear)
		mxc_isi_write(pipe, CHNL_STS, status);

	return status;
}

void mxc_isi_channel_irq_clear(struct mxc_isi_pipe *pipe)
{
	mxc_isi_write(pipe, CHNL_STS, 0xffffffff);
}

static void mxc_isi_channel_irq_enable(struct mxc_isi_pipe *pipe)
{
	const struct mxc_isi_ier_reg *ier_reg = pipe->isi->pdata->ier_reg;
	u32 val;

	val = CHNL_IER_FRM_RCVD_EN |
		CHNL_IER_AXI_WR_ERR_U_EN |
		CHNL_IER_AXI_WR_ERR_V_EN |
		CHNL_IER_AXI_WR_ERR_Y_EN;

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

	mxc_isi_channel_irq_clear(pipe);
	mxc_isi_write(pipe, CHNL_IER, val);
}

static void mxc_isi_channel_irq_disable(struct mxc_isi_pipe *pipe)
{
	mxc_isi_write(pipe, CHNL_IER, 0);
}

/* -----------------------------------------------------------------------------
 * Init, deinit, enable, disable
 */

static void mxc_isi_channel_sw_reset(struct mxc_isi_pipe *pipe)
{
	u32 val;

	val = mxc_isi_read(pipe, CHNL_CTRL);
	val |= CHNL_CTRL_SW_RST;
	mxc_isi_write(pipe, CHNL_CTRL, val);
	mdelay(5);
	val &= ~CHNL_CTRL_SW_RST;
	mxc_isi_write(pipe, CHNL_CTRL, val);
}

void mxc_isi_channel_init(struct mxc_isi_pipe *pipe)
{
	u32 val;

	/* sw reset */
	mxc_isi_channel_sw_reset(pipe);

	/* Init channel clk first */
	val = mxc_isi_read(pipe, CHNL_CTRL);
	val |= CHNL_CTRL_CLK_EN;
	mxc_isi_write(pipe, CHNL_CTRL, val);
}

void mxc_isi_channel_deinit(struct mxc_isi_pipe *pipe)
{
	/* sw reset */
	mxc_isi_channel_sw_reset(pipe);

	/* deinit channel clk first */
	mxc_isi_write(pipe, CHNL_CTRL, 0);

	if (pipe->chain_buf)
		mxc_isi_write(pipe + 1, CHNL_CTRL, 0);
}

void mxc_isi_channel_enable(struct mxc_isi_pipe *pipe)
{
	u32 val;

	mxc_isi_channel_irq_enable(pipe);

	val = mxc_isi_read(pipe, CHNL_CTRL);
	val |= CHNL_CTRL_BLANK_PXL(0xff);

	val |= CHNL_CTRL_CHNL_EN;
	mxc_isi_write(pipe, CHNL_CTRL, val);

	mxc_isi_pipe_dump_regs(pipe);
	msleep(300);
}

void mxc_isi_channel_disable(struct mxc_isi_pipe *pipe)
{
	u32 val;

	mxc_isi_channel_irq_disable(pipe);

	val = mxc_isi_read(pipe, CHNL_CTRL);
	val &= ~CHNL_CTRL_CHNL_EN;
	val &= ~CHNL_CTRL_CLK_EN;
	mxc_isi_write(pipe, CHNL_CTRL, val);
}
