/* SPDX-License-Identifier: GPL-2.0 */
/*
 * V4L2 Capture ISI subdev for i.MX8QXP/QM platform
 *
 * ISI is a Image Sensor Interface of i.MX8QXP/QM platform, which
 * used to process image from camera sensor to memory or DC
 * Copyright 2019-2020 NXP
 */

#ifndef __MXC_ISI_CORE_H__
#define __MXC_ISI_CORE_H__

#include <linux/bug.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <linux/slab.h>
#include <linux/sys_soc.h>
#include <linux/types.h>

#include <media/media-device.h>
#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-subdev.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-dma-contig.h>

#define ISI_OF_NODE_NAME	"isi"
#define MIPI_CSI2_OF_NODE_NAME  "csi"
#define PARALLEL_OF_NODE_NAME   "pcsi"

#define MXC_ISI_MAX_DEVS	8
#define MXC_ISI_NUM_PORTS	1
#define MXC_MIPI_CSI2_MAX_DEVS	2
#define MXC_MAX_SENSORS		3

/* ISI PADS */
#define MXC_ISI_SD_PAD_SINK	0
#define MXC_ISI_SD_PAD_SOURCE	1
#define MXC_ISI_SD_PADS_NUM	2

#define ISI_2K			2048

#define MXC_ISI_DRIVER_NAME	"mxc-isi"
#define MXC_ISI_CAPTURE		"mxc-isi-cap"
#define MXC_MAX_PLANES		3

struct mxc_isi_dev;

enum {
	IN_PORT,
	OUT_PORT,
	MAX_PORTS,
};

enum isi_input_interface {
	ISI_INPUT_INTERFACE_DC0 = 0,
	ISI_INPUT_INTERFACE_DC1,
	ISI_INPUT_INTERFACE_MIPI0_CSI2,
	ISI_INPUT_INTERFACE_MIPI1_CSI2,
	ISI_INPUT_INTERFACE_HDMI,
	ISI_INPUT_INTERFACE_MEM,
	ISI_INPUT_INTERFACE_PARALLEL_CSI,
	ISI_INPUT_INTERFACE_MAX,
};

enum isi_output_interface {
	ISI_OUTPUT_INTERFACE_DC0 = 0,
	ISI_OUTPUT_INTERFACE_DC1,
	ISI_OUTPUT_INTERFACE_MEM,
	ISI_OUTPUT_INTERFACE_MAX,
};

enum mxc_isi_buf_id {
	MXC_ISI_BUF1 = 0x0,
	MXC_ISI_BUF2,
};

enum mxc_isi_out_fmt {
	MXC_ISI_OUT_FMT_RGBA32	= 0x0,
	MXC_ISI_OUT_FMT_ABGR32,
	MXC_ISI_OUT_FMT_ARGB32,
	MXC_ISI_OUT_FMT_RGBX32,
	MXC_ISI_OUT_FMT_XBGR32,
	MXC_ISI_OUT_FMT_XRGB32,
	MXC_ISI_OUT_FMT_RGB32P,
	MXC_ISI_OUT_FMT_BGR32P,
	MXC_ISI_OUT_FMT_A2BGR10,
	MXC_ISI_OUT_FMT_A2RGB10,
	MXC_ISI_OUT_FMT_RGB565,
	MXC_ISI_OUT_FMT_RAW8,
	MXC_ISI_OUT_FMT_RAW10,
	MXC_ISI_OUT_FMT_RAW10P,
	MXC_ISI_OUT_FMT_RAW12,
	MXC_ISI_OUT_FMT_RAW16,
	MXC_ISI_OUT_FMT_YUV444_1P8P,
	MXC_ISI_OUT_FMT_YUV444_2P8P,
	MXC_ISI_OUT_FMT_YUV444_3P8P,
	MXC_ISI_OUT_FMT_YUV444_1P8,
	MXC_ISI_OUT_FMT_YUV444_1P10,
	MXC_ISI_OUT_FMT_YUV444_2P10,
	MXC_ISI_OUT_FMT_YUV444_3P10,
	MXC_ISI_OUT_FMT_YUV444_1P10P = 0x18,
	MXC_ISI_OUT_FMT_YUV444_2P10P,
	MXC_ISI_OUT_FMT_YUV444_3P10P,
	MXC_ISI_OUT_FMT_YUV444_1P12 = 0x1C,
	MXC_ISI_OUT_FMT_YUV444_2P12,
	MXC_ISI_OUT_FMT_YUV444_3P12,
	MXC_ISI_OUT_FMT_YUV422_1P8P = 0x20,
	MXC_ISI_OUT_FMT_YUV422_2P8P,
	MXC_ISI_OUT_FMT_YUV422_3P8P,
	MXC_ISI_OUT_FMT_YUV422_1P10 = 0x24,
	MXC_ISI_OUT_FMT_YUV422_2P10,
	MXC_ISI_OUT_FMT_YUV422_3P10,
	MXC_ISI_OUT_FMT_YUV422_1P10P = 0x28,
	MXC_ISI_OUT_FMT_YUV422_2P10P,
	MXC_ISI_OUT_FMT_YUV422_3P10P,
	MXC_ISI_OUT_FMT_YUV422_1P12 = 0x2C,
	MXC_ISI_OUT_FMT_YUV422_2P12,
	MXC_ISI_OUT_FMT_YUV422_3P12,
	MXC_ISI_OUT_FMT_YUV420_2P8P = 0x31,
	MXC_ISI_OUT_FMT_YUV420_3P8P,
	MXC_ISI_OUT_FMT_YUV420_2P10 = 0x35,
	MXC_ISI_OUT_FMT_YUV420_3P10,
	MXC_ISI_OUT_FMT_YUV420_2P10P = 0x39,
	MXC_ISI_OUT_FMT_YUV420_3P10P,
	MXC_ISI_OUT_FMT_YUV420_2P12 = 0x3D,
	MXC_ISI_OUT_FMT_YUV420_3P12,
};

enum mxc_isi_in_fmt {
	MXC_ISI_IN_FMT_BGR8P	= 0x0,
};

struct mxc_isi_format_info {
	u32	mbus_code;
	u32	fourcc;
	u32	color;
	u16	memplanes;
	u8	colorspace;
	u8	depth[MXC_MAX_PLANES];
	u16	mdataplanes;
	u16	flags;
};

struct mxc_isi_ctrls {
	struct v4l2_ctrl_handler handler;
	struct v4l2_ctrl *alpha;
	bool ready;
};

/**
 * struct addr -  physical address set for DMA
 * @y:	 luminance plane physical address
 * @cb:	 Cb plane physical address
 * @cr:	 Cr plane physical address
 */
struct frame_addr {
	u32	y;
	u32	cb;
	u32	cr;
};

/**
 * struct mxc_isi_frame - source/target frame properties
 * o_width:	 original image width from sensor
 * o_height: original image height from sensor
 * c_width:	 crop image width set by g_selection
 * c_height: crop image height set by g_selection
 * h_off:	crop horizontal pixel offset
 * v_off:	crop vertical pixel offset
 * width:	out image pixel width
 * height:	out image pixel weight
 * bytesperline: bytesperline value for each plane
 * paddr:	image frame buffer physical addresses
 * fmt:	color format pointer
 */
struct mxc_isi_frame {
	u32	o_width;
	u32	o_height;
	u32	c_width;
	u32	c_height;
	u32	h_off;
	u32	v_off;
	u32	width;
	u32	height;
	unsigned int	sizeimage[MXC_MAX_PLANES];
	unsigned int	bytesperline[MXC_MAX_PLANES];
	const struct mxc_isi_format_info *info;
};

struct mxc_isi_roi_alpha {
	u8 alpha;
	struct v4l2_rect rect;
};

struct mxc_isi_buffer {
	struct vb2_v4l2_buffer  v4l2_buf;
	struct list_head	list;
	struct frame_addr	paddr;
	enum mxc_isi_buf_id	id;
	bool discard;
};

struct mxc_isi_chan_src {
	u32 src_dc0;
	u32 src_dc1;
	u32 src_mipi0;
	u32 src_mipi1;
	u32 src_hdmi;
	u32 src_csi;
	u32 src_mem;
};

struct mxc_isi_reg {
	u32 offset;
	u32 mask;
};

struct mxc_isi_ier_reg {
	/* Overflow Y/U/V triggier enable*/
	struct mxc_isi_reg oflw_y_buf_en;
	struct mxc_isi_reg oflw_u_buf_en;
	struct mxc_isi_reg oflw_v_buf_en;

	/* Excess overflow Y/U/V triggier enable*/
	struct mxc_isi_reg excs_oflw_y_buf_en;
	struct mxc_isi_reg excs_oflw_u_buf_en;
	struct mxc_isi_reg excs_oflw_v_buf_en;

	/* Panic Y/U/V triggier enable*/
	struct mxc_isi_reg panic_y_buf_en;
	struct mxc_isi_reg panic_v_buf_en;
	struct mxc_isi_reg panic_u_buf_en;
};

struct mxc_isi_panic_thd {
	u32 mask;
	u32 offset;
	u32 threshold;
};

struct mxc_isi_set_thd {
	struct mxc_isi_panic_thd panic_set_thd_y;
	struct mxc_isi_panic_thd panic_set_thd_u;
	struct mxc_isi_panic_thd panic_set_thd_v;
};

enum model {
	MXC_ISI_IMX8,
	MXC_ISI_IMX8MN,
	MXC_ISI_IMX8MP,
};

struct mxc_isi_plat_data {
	enum model model;
	const struct mxc_isi_chan_src *chan_src;
	const struct mxc_isi_ier_reg  *ier_reg;
	const struct mxc_isi_set_thd *set_thd;
	const struct clk_bulk_data *clks;
	unsigned int num_clks;
	bool buf_active_reverse;
};

struct mxc_isi_video {
	struct video_device		vdev;
	struct media_pad		pad;

	bool				is_link_setup;
	struct v4l2_pix_format_mplane	pix;
	struct mxc_isi_ctrls		ctrls;

	struct vb2_queue		vb2_q;
	struct mxc_isi_buffer		buf_discard[2];
	struct list_head		out_pending;
	struct list_head		out_active;
	struct list_head		out_discard;
	u32				frame_count;

	/* dirty buffer */
	size_t				discard_size[MXC_MAX_PLANES];
	void				*discard_buffer[MXC_MAX_PLANES];
	dma_addr_t			discard_buffer_dma[MXC_MAX_PLANES];
};

struct mxc_isi_pipe {
	struct mxc_isi_dev		*isi;
	u32				id;

	struct v4l2_subdev		sd;
	struct media_pad		pads[MXC_ISI_SD_PADS_NUM];
	struct mxc_isi_frame		formats[MXC_ISI_SD_PADS_NUM];

	struct mxc_isi_video		video;

	struct mutex			lock;
	spinlock_t			slock;
};

struct mxc_isi_dev {
	struct mxc_isi_pipe pipe;

	struct device *dev;

	const struct mxc_isi_plat_data *pdata;

	struct clk_bulk_data *clks;

	struct reset_control *soft_resetn;
	struct reset_control *clk_enable;

	struct regmap *chain;

	struct mutex lock;
	spinlock_t   slock;

	void __iomem *regs;

	u8 chain_buf;
	u8 alpha;

	void (*frame_write_done)(struct mxc_isi_dev *isi);

	/* manage share ISI channel resource */
	atomic_t usage_count;

	/* scale factor */
	u32 xfactor;
	u32 yfactor;
	u32 pre_dec_x;
	u32 pre_dec_y;

	u32 status;

	u32 interface[MAX_PORTS];
	int id;

	struct media_device media_dev;
	struct v4l2_device v4l2_dev;
	struct v4l2_async_notifier notifier;
	struct list_head asds;

	unsigned int hflip:1;
	unsigned int vflip:1;
	unsigned int cscen:1;
	unsigned int scale:1;
	unsigned int alphaen:1;
	unsigned int crop:1;
	unsigned int deinterlace:1;
	unsigned int is_streaming:1;
};

int mxc_isi_pipe_init(struct mxc_isi_dev *isi);
void mxc_isi_pipe_cleanup(struct mxc_isi_dev *isi);

static inline void set_frame_bounds(struct mxc_isi_frame *f,
				    u32 width, u32 height)
{
	f->o_width  = width;
	f->o_height = height;
	f->c_width  = width;
	f->c_height = height;
	f->width  = width;
	f->height = height;
}

static inline void set_frame_out(struct mxc_isi_frame *f,
				 u32 width, u32 height)
{
	f->c_width  = width;
	f->c_height = height;
	f->width  = width;
	f->height = height;
}

static inline void set_frame_crop(struct mxc_isi_frame *f,
				  u32 left, u32 top, u32 width, u32 height)
{
	f->h_off = left;
	f->v_off = top;
	f->c_width  = width;
	f->c_height = height;
}
#endif /* __MXC_ISI_CORE_H__ */
