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

#include <linux/atomic.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/videodev2.h>

#include <media/media-device.h>
#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-v4l2.h>

struct clk_bulk_data;
struct device;
struct regmap;

/* Pipeline pads */
#define MXC_ISI_PIPE_PAD_SINK		0
#define MXC_ISI_PIPE_PAD_SOURCE		1
#define MXC_ISI_PIPE_PADS_NUM		2

#define ISI_2K				2048

/*
 * Absolute limits based on register field widths, not taking the scaler and
 * line buffer constraints into account. The hardware may have additional
 * constraints not documented in the reference manual.
 */
#define MXC_ISI_MIN_WIDTH		1U
#define MXC_ISI_MIN_HEIGHT		1U
#define MXC_ISI_MAX_WIDTH		8191U
#define MXC_ISI_MAX_HEIGHT		8191U

#define MXC_ISI_DEF_WIDTH		1920U
#define MXC_ISI_DEF_HEIGHT		1080U
#define MXC_ISI_DEF_MBUS_CODE_SINK	MEDIA_BUS_FMT_UYVY8_1X16
#define MXC_ISI_DEF_MBUS_CODE_SOURCE	MEDIA_BUS_FMT_YUV8_1X24
#define MXC_ISI_DEF_PIXEL_FORMAT	V4L2_PIX_FMT_YUYV

#define MXC_ISI_DRIVER_NAME		"mxc-isi"
#define MXC_ISI_CAPTURE			"mxc-isi-cap"
#define MXC_MAX_PLANES			3

struct mxc_isi_dev;

enum mxc_isi_buf_id {
	MXC_ISI_BUF1 = 0x0,
	MXC_ISI_BUF2,
};

enum mxc_isi_encoding {
	MXC_ISI_ENC_RAW,
	MXC_ISI_ENC_RGB,
	MXC_ISI_ENC_YUV,
};

enum isi_csi_coeff {
	YUV2RGB = 0,
	RGB2YUV,
};

struct mxc_isi_format_info {
	u32	mbus_code;
	u32	fourcc;
	u32	isi_format;
	u16	memplanes;
	u8	depth[MXC_MAX_PLANES];
	u8	hsub;
	u8	vsub;
	enum mxc_isi_encoding encoding;
};

struct mxc_isi_bus_format_info {
	u32	mbus_code;
	u32	output;
	u32	pads;
	enum mxc_isi_encoding encoding;
};

struct mxc_isi_buffer {
	struct vb2_v4l2_buffer  v4l2_buf;
	struct list_head	list;
	u32			dma_addrs[3];
	enum mxc_isi_buf_id	id;
	bool discard;
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
	unsigned int num_ports;
	unsigned int num_channels;
	unsigned int reg_offset;
	const struct mxc_isi_ier_reg  *ier_reg;
	const struct mxc_isi_set_thd *set_thd;
	const struct clk_bulk_data *clks;
	unsigned int num_clks;
	bool buf_active_reverse;
	bool has_gasket;
};

struct mxc_isi_dma_buffer {
	size_t				size;
	void				*addr;
	dma_addr_t			dma;
};

struct mxc_isi_crossbar {
	struct mxc_isi_dev		*isi;

	unsigned int			num_sinks;
	unsigned int			num_sources;

	struct v4l2_subdev		sd;
	struct media_pad		*pads;
};

struct mxc_isi_video {
	struct mxc_isi_pipe		*pipe;

	struct video_device		vdev;
	struct media_pad		pad;

	struct mutex			lock;
	atomic_t			usage_count;
	bool				is_streaming;

	struct v4l2_pix_format_mplane	pix;
	const struct mxc_isi_format_info *fmtinfo;

	struct {
		struct v4l2_ctrl_handler handler;
		struct v4l2_ctrl *alpha;
	} ctrls;

	struct vb2_queue		vb2_q;
	struct mxc_isi_buffer		buf_discard[3];
	struct list_head		out_pending;
	struct list_head		out_active;
	struct list_head		out_discard;
	u32				frame_count;
	/* Protects out_pending, out_active, out_discard and frame_count */
	spinlock_t			buf_lock;


	struct mxc_isi_dma_buffer	discard_buffer[MXC_MAX_PLANES];
};

struct mxc_isi_pipe {
	struct mxc_isi_dev		*isi;
	u32				id;
	void __iomem			*regs;

	struct media_pipeline		pipe;

	struct v4l2_subdev		sd;
	struct media_pad		pads[MXC_ISI_PIPE_PADS_NUM];

	struct mxc_isi_video		video;

	u8				chain_buf;
	u8				alpha;

	unsigned int			hflip:1;
	unsigned int			vflip:1;
};

struct mxc_isi_dev {
	struct device			*dev;

	const struct mxc_isi_plat_data	*pdata;

	void __iomem			*regs;
	struct clk_bulk_data		*clks;
	struct regmap			*gasket;

	struct mxc_isi_crossbar		crossbar;
	struct mxc_isi_pipe		*pipes;

	struct media_device		media_dev;
	struct v4l2_device		v4l2_dev;
	struct v4l2_async_notifier	notifier;
};

int mxc_isi_crossbar_init(struct mxc_isi_dev *isi);
void mxc_isi_crossbar_cleanup(struct mxc_isi_crossbar *xbar);
int mxc_isi_crossbar_register(struct mxc_isi_crossbar *xbar);
void mxc_isi_crossbar_unregister(struct mxc_isi_crossbar *xbar);

const struct mxc_isi_bus_format_info *
mxc_isi_bus_format_by_index(unsigned int index, unsigned int pad);

int mxc_isi_pipe_init(struct mxc_isi_dev *isi, unsigned int id);
void mxc_isi_pipe_cleanup(struct mxc_isi_pipe *pipe);
int mxc_isi_pipe_register(struct mxc_isi_pipe *pipe);
void mxc_isi_pipe_unregister(struct mxc_isi_pipe *pipe);
int mxc_isi_pipe_enable(struct mxc_isi_pipe *pipe);
void mxc_isi_pipe_disable(struct mxc_isi_pipe *pipe);

int mxc_isi_video_register(struct mxc_isi_pipe *pipe,
			   struct v4l2_device *v4l2_dev);
void mxc_isi_video_unregister(struct mxc_isi_pipe *pipe);
void mxc_isi_video_frame_write_done(struct mxc_isi_pipe *pipe, u32 status);

void mxc_isi_channel_init(struct mxc_isi_pipe *pipe);
void mxc_isi_channel_deinit(struct mxc_isi_pipe *pipe);
void mxc_isi_channel_enable(struct mxc_isi_pipe *pipe);
void mxc_isi_channel_disable(struct mxc_isi_pipe *pipe);
void mxc_isi_channel_set_output_format(struct mxc_isi_pipe *pipe,
				       const struct mxc_isi_format_info *info,
				       struct v4l2_pix_format_mplane *format);
void mxc_isi_channel_set_flip(struct mxc_isi_pipe *pipe);
void mxc_isi_channel_set_alpha(struct mxc_isi_pipe *pipe);
void mxc_isi_channel_set_chain_buf(struct mxc_isi_pipe *pipe);
void mxc_isi_channel_set_crop(struct mxc_isi_pipe *pipe,
			      const struct v4l2_rect *src,
			      const struct v4l2_rect *dst);

void mxc_isi_channel_set_outbuf(struct mxc_isi_pipe *pipe,
				struct mxc_isi_buffer *buf,
				enum mxc_isi_buf_id buf_id);

void mxc_isi_channel_config(struct mxc_isi_pipe *pipe, unsigned int input,
			    const struct v4l2_mbus_framefmt *src_format,
			    const struct v4l2_rect *src_compose,
			    enum mxc_isi_encoding src_encoding,
			    enum mxc_isi_encoding dst_encoding);

void mxc_isi_clear_irqs(struct mxc_isi_pipe *pipe);

u32 mxc_isi_get_irq_status(struct mxc_isi_pipe *pipe, bool clear);

#endif /* __MXC_ISI_CORE_H__ */
