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

static inline void mxc_isi_write(struct mxc_isi_pipe *pipe, u32 reg, u32 val)
{
	writel(val, pipe->regs + reg);
}

/* -----------------------------------------------------------------------------
 * Buffers
 */

void mxc_isi_channel_set_inbuf(struct mxc_isi_pipe *pipe, u32 dma_addr)
{
	mxc_isi_write(pipe, CHNL_IN_BUF_ADDR, dma_addr);
}

void mxc_isi_channel_set_outbuf(struct mxc_isi_pipe *pipe,
				const u32 dma_addrs[3],
				enum mxc_isi_buf_id buf_id)
{
	int val;

	val = mxc_isi_read(pipe, CHNL_OUT_BUF_CTRL);

	if (buf_id == MXC_ISI_BUF1) {
		mxc_isi_write(pipe, CHNL_OUT_BUF1_ADDR_Y, dma_addrs[0]);
		mxc_isi_write(pipe, CHNL_OUT_BUF1_ADDR_U, dma_addrs[1]);
		mxc_isi_write(pipe, CHNL_OUT_BUF1_ADDR_V, dma_addrs[2]);
		val ^= CHNL_OUT_BUF_CTRL_LOAD_BUF1_ADDR;
	} else  {
		mxc_isi_write(pipe, CHNL_OUT_BUF2_ADDR_Y, dma_addrs[0]);
		mxc_isi_write(pipe, CHNL_OUT_BUF2_ADDR_U, dma_addrs[1]);
		mxc_isi_write(pipe, CHNL_OUT_BUF2_ADDR_V, dma_addrs[2]);
		val ^= CHNL_OUT_BUF_CTRL_LOAD_BUF2_ADDR;
	}

	mxc_isi_write(pipe, CHNL_OUT_BUF_CTRL, val);
}

/* -----------------------------------------------------------------------------
 * Pipeline configuration
 */

static void mxc_isi_channel_source_config(struct mxc_isi_pipe *pipe,
					  enum mxc_isi_input input)
{
	u32 val;

	val = mxc_isi_read(pipe, CHNL_CTRL);
	val &= ~(CHNL_CTRL_MIPI_VC_ID_MASK | CHNL_CTRL_SRC_TYPE_MASK |
		  CHNL_CTRL_SRC_INPUT_MASK);

	if (input == MXC_ISI_INPUT_MEM) {
		/*
		 * The memory input is connected to the last port of the
		 * crossbar switch, after all pixel link inputs. The SRC_INPUT
		 * field controls the input selection and must be set
		 * accordingly, despite being documented as ignored when using
		 * the memory input in the i.MX8MP reference manual, and
		 * reserved in the i.MX8MN reference manual.
		 */
		val |= CHNL_CTRL_SRC_TYPE(CHNL_CTRL_SRC_TYPE_MEMORY);
		val |= CHNL_CTRL_SRC_INPUT(pipe->isi->pdata->num_ports);
	} else {
		val |= CHNL_CTRL_SRC_TYPE(CHNL_CTRL_SRC_TYPE_DEVICE);
		val |= CHNL_CTRL_SRC_INPUT(input);
		val |= CHNL_CTRL_MIPI_VC_ID(0); /* FIXME: For CSI-2 only */
	}

	mxc_isi_write(pipe, CHNL_CTRL, val);
}

static void mxc_isi_chain_buf(struct mxc_isi_pipe *pipe,
			      const struct v4l2_area *in_size)
{
	u32 val;

	if (in_size->width > 2048) {
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
					enum mxc_isi_encoding encoding,
					const struct v4l2_area *in_size,
					const struct v4l2_area *out_size,
					bool *bypass)
{
	u32 xscale, yscale;
	u32 decx, decy;
	u32 val;

	dev_dbg(pipe->isi->dev, "input %ux%u, output %ux%u\n",
		in_size->width, in_size->height,
		out_size->width, out_size->height);

	xscale = mxc_isi_channel_scaling_ratio(in_size->width, out_size->width,
					       &decx);
	yscale = mxc_isi_channel_scaling_ratio(in_size->height, out_size->height,
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
	if (encoding == MXC_ISI_ENC_YUV)
		val |= CHNL_IMG_CTRL_YCBCR_MODE;

	mxc_isi_write(pipe, CHNL_IMG_CTRL, val);

	mxc_isi_write(pipe, CHNL_SCALE_FACTOR,
		      CHNL_SCALE_FACTOR_Y_SCALE(yscale) |
		      CHNL_SCALE_FACTOR_X_SCALE(xscale));

	mxc_isi_write(pipe, CHNL_SCALE_OFFSET, 0);

	mxc_isi_write(pipe, CHNL_SCL_IMG_CFG,
		      CHNL_SCL_IMG_CFG_HEIGHT(out_size->height) |
		      CHNL_SCL_IMG_CFG_WIDTH(out_size->width));

	*bypass = in_size->height == out_size->height &&
		  in_size->width == out_size->width;
}

static void mxc_isi_channel_set_crop(struct mxc_isi_pipe *pipe,
				     const struct v4l2_area *src,
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
static const u32 mxc_isi_yuv2rgb_coeffs[6] = {
	/* YUV -> RGB */
	0x0000012a, 0x012a0198, 0x0730079c,
	0x0204012a, 0x01f00000, 0x01800180
};

static const u32 mxc_isi_rgb2yuv_coeffs[6] = {
	/* RGB->YUV */
	0x00810041, 0x07db0019, 0x007007b6,
	0x07a20070, 0x001007ee, 0x00800080
};

static void mxc_isi_channel_set_csc(struct mxc_isi_pipe *pipe,
				    enum mxc_isi_encoding in_encoding,
				    enum mxc_isi_encoding out_encoding,
				    bool *bypass)
{
	static const char * const encodings[] = {
		[MXC_ISI_ENC_RAW] = "RAW",
		[MXC_ISI_ENC_RGB] = "RGB",
		[MXC_ISI_ENC_YUV] = "YUV",
	};
	const u32 *coeffs;
	bool cscen = true;
	u32 val;

	val = mxc_isi_read(pipe, CHNL_IMG_CTRL);
	val &= ~(CHNL_IMG_CTRL_CSC_BYPASS | CHNL_IMG_CTRL_CSC_MODE_MASK);

	if (in_encoding == MXC_ISI_ENC_YUV &&
	    out_encoding == MXC_ISI_ENC_RGB) {
		/* YUV2RGB */
		coeffs = mxc_isi_yuv2rgb_coeffs;
		/* YCbCr enable???  */
		val |= CHNL_IMG_CTRL_CSC_MODE(CHNL_IMG_CTRL_CSC_MODE_YCBCR2RGB);
	} else if (in_encoding == MXC_ISI_ENC_RGB &&
		   out_encoding == MXC_ISI_ENC_YUV) {
		/* RGB2YUV */
		coeffs = mxc_isi_rgb2yuv_coeffs;
		val |= CHNL_IMG_CTRL_CSC_MODE(CHNL_IMG_CTRL_CSC_MODE_RGB2YCBCR);
	} else {
		/* Bypass CSC */
		cscen = false;
		val |= CHNL_IMG_CTRL_CSC_BYPASS;
	}

	dev_dbg(pipe->isi->dev, "CSC: %s -> %s\n",
		encodings[in_encoding], encodings[out_encoding]);

	if (cscen) {
		mxc_isi_write(pipe, CHNL_CSC_COEFF0, coeffs[0]);
		mxc_isi_write(pipe, CHNL_CSC_COEFF1, coeffs[1]);
		mxc_isi_write(pipe, CHNL_CSC_COEFF2, coeffs[2]);
		mxc_isi_write(pipe, CHNL_CSC_COEFF3, coeffs[3]);
		mxc_isi_write(pipe, CHNL_CSC_COEFF4, coeffs[4]);
		mxc_isi_write(pipe, CHNL_CSC_COEFF5, coeffs[5]);
	}

	mxc_isi_write(pipe, CHNL_IMG_CTRL, val);

	*bypass = !cscen;
}

void mxc_isi_channel_set_alpha(struct mxc_isi_pipe *pipe, u8 alpha)
{
	u32 val;

	val = mxc_isi_read(pipe, CHNL_IMG_CTRL);
	val &= ~CHNL_IMG_CTRL_GBL_ALPHA_VAL_MASK;
	val |= CHNL_IMG_CTRL_GBL_ALPHA_VAL(alpha) |
	       CHNL_IMG_CTRL_GBL_ALPHA_EN;
	mxc_isi_write(pipe, CHNL_IMG_CTRL, val);
}

void mxc_isi_channel_set_flip(struct mxc_isi_pipe *pipe, bool hflip, bool vflip)
{
	u32 val;

	val = mxc_isi_read(pipe, CHNL_IMG_CTRL);
	val &= ~(CHNL_IMG_CTRL_VFLIP_EN | CHNL_IMG_CTRL_HFLIP_EN);

	if (vflip)
		val |= CHNL_IMG_CTRL_VFLIP_EN;
	if (hflip)
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

void mxc_isi_channel_config(struct mxc_isi_pipe *pipe, enum mxc_isi_input input,
			    const struct v4l2_area *in_size,
			    const struct v4l2_area *scale,
			    const struct v4l2_rect *crop,
			    enum mxc_isi_encoding in_encoding,
			    enum mxc_isi_encoding out_encoding)
{
	bool csc_bypass;
	bool scaler_bypass;
	u32 val;

	/* Input source (including VC configuration for CSI-2) */
	mxc_isi_channel_source_config(pipe, input);

	/* Input frame size */
	mxc_isi_write(pipe, CHNL_IMG_CFG,
		      CHNL_IMG_CFG_HEIGHT(in_size->height) |
		      CHNL_IMG_CFG_WIDTH(in_size->width));

	/* Scaling */
	mxc_isi_chain_buf(pipe, in_size);
	mxc_isi_channel_set_scaling(pipe, in_encoding, in_size, scale,
				    &scaler_bypass);
	mxc_isi_channel_set_crop(pipe, scale, crop);

	/* CSC */
	mxc_isi_channel_set_csc(pipe, in_encoding, out_encoding, &csc_bypass);

	/* Output buffer management */
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

void mxc_isi_channel_set_input_format(struct mxc_isi_pipe *pipe,
				      const struct mxc_isi_format_info *info,
				      const struct v4l2_pix_format_mplane *format)
{
	unsigned int bpl = format->plane_fmt[0].bytesperline;

	mxc_isi_write(pipe, CHNL_MEM_RD_CTRL,
		      CHNL_MEM_RD_CTRL_IMG_TYPE(info->isi_in_format));
	mxc_isi_write(pipe, CHNL_IN_BUF_PITCH,
		      CHNL_IN_BUF_PITCH_LINE_PITCH(bpl));
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
	val |= CHNL_IMG_CTRL_FORMAT(info->isi_out_format);
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

static void mxc_isi_channel_sw_reset(struct mxc_isi_pipe *pipe, bool enable_clk)
{
	mxc_isi_write(pipe, CHNL_CTRL, CHNL_CTRL_SW_RST);
	mdelay(5);
	mxc_isi_write(pipe, CHNL_CTRL, enable_clk ? CHNL_CTRL_CLK_EN : 0);
}

void mxc_isi_channel_init(struct mxc_isi_pipe *pipe)
{
	mxc_isi_channel_sw_reset(pipe, true);
}

void mxc_isi_channel_deinit(struct mxc_isi_pipe *pipe)
{
	mxc_isi_channel_sw_reset(pipe, false);

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

	msleep(300);
}

void mxc_isi_channel_disable(struct mxc_isi_pipe *pipe)
{
	u32 val;

	mxc_isi_channel_irq_disable(pipe);

	val = mxc_isi_read(pipe, CHNL_CTRL);
	val &= ~CHNL_CTRL_CHNL_EN;
	mxc_isi_write(pipe, CHNL_CTRL, val);
}

void mxc_isi_channel_m2m_start(struct mxc_isi_pipe *pipe)
{
	u32 val;

	val = mxc_isi_read(pipe, CHNL_MEM_RD_CTRL);
	val &= ~CHNL_MEM_RD_CTRL_READ_MEM;
	mxc_isi_write(pipe, CHNL_MEM_RD_CTRL, val);
	udelay(300);

	val |= CHNL_MEM_RD_CTRL_READ_MEM;
	mxc_isi_write(pipe, CHNL_MEM_RD_CTRL, val);
}
