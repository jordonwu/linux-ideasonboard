// SPDX-License-Identifier: (GPL-2.0+ OR MIT)
/*
 * Rockchip ISP1 Driver - Base driver
 *
 * Copyright (C) 2019 Collabora, Ltd.
 *
 * Based on Rockchip ISP1 driver by Rockchip Electronics Co., Ltd.
 * Copyright (C) 2017 Rockchip Electronics Co., Ltd.
 */

#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/of_platform.h>
#include <linux/pinctrl/consumer.h>
#include <linux/phy/phy.h>
#include <linux/phy/phy-mipi-dphy.h>
#include <linux/mfd/syscon.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-mc.h>

#include "rkisp1-common.h"
#include "rkisp1-csi.h"

/*
 * ISP Details
 * -----------
 *
 * ISP Comprises with:
 *	MIPI serial camera interface
 *	Image Signal Processing
 *	Many Image Enhancement Blocks
 *	Crop
 *	Resizer
 *	RBG display ready image
 *	Image Rotation
 *
 * ISP Block Diagram
 * -----------------
 *                                                             rkisp1-resizer.c          rkisp1-capture.c
 *                                                          |====================|  |=======================|
 *                                rkisp1-isp.c                              Main Picture Path
 *                        |==========================|      |===============================================|
 *                        +-----------+  +--+--+--+--+      +--------+  +--------+              +-----------+
 *                        |           |  |  |  |  |  |      |        |  |        |              |           |
 * +--------+    |\       |           |  |  |  |  |  |   -->|  Crop  |->|  RSZ   |------------->|           |
 * |  MIPI  |--->|  \     |           |  |  |  |  |  |   |  |        |  |        |              |           |
 * +--------+    |   |    |           |  |IE|IE|IE|IE|   |  +--------+  +--------+              |  Memory   |
 *               |MUX|--->|    ISP    |->|0 |1 |2 |3 |---+                                      | Interface |
 * +--------+    |   |    |           |  |  |  |  |  |   |  +--------+  +--------+  +--------+  |           |
 * |Parallel|--->|  /     |           |  |  |  |  |  |   |  |        |  |        |  |        |  |           |
 * +--------+    |/       |           |  |  |  |  |  |   -->|  Crop  |->|  RSZ   |->|  RGB   |->|           |
 *                        |           |  |  |  |  |  |      |        |  |        |  | Rotate |  |           |
 *                        +-----------+  +--+--+--+--+      +--------+  +--------+  +--------+  +-----------+
 *                                               ^
 * +--------+                                    |          |===============================================|
 * |  DMA   |------------------------------------+                          Self Picture Path
 * +--------+
 *
 *         rkisp1-stats.c        rkisp1-params.c
 *       |===============|      |===============|
 *       +---------------+      +---------------+
 *       |               |      |               |
 *       |      ISP      |      |      ISP      |
 *       |               |      |               |
 *       +---------------+      +---------------+
 *
 *
 * Media Topology
 * --------------
 *      +----------+     +----------+
 *      | Sensor 2 |     | Sensor X |
 *      ------------ ... ------------
 *      |    0     |     |    0     |
 *      +----------+     +----------+      +-----------+
 *                  \      |               |  params   |
 *                   \     |               | (output)  |
 *    +----------+    \    |               +-----------+
 *    | Sensor 1 |     v   v                     |
 *    ------------      +------+------+          |
 *    |    0     |----->|  0   |  1   |<---------+
 *    +----------+      |------+------|
 *                      |     ISP     |
 *                      |------+------|
 *        +-------------|  2   |  3   |----------+
 *        |             +------+------+          |
 *        |                |                     |
 *        v                v                     v
 *  +- ---------+    +-----------+         +-----------+
 *  |     0     |    |     0     |         |   stats   |
 *  -------------    -------------         | (capture) |
 *  |  Resizer  |    |  Resizer  |         +-----------+
 *  ------------|    ------------|
 *  |     1     |    |     1     |
 *  +-----------+    +-----------+
 *        |                |
 *        v                v
 *  +-----------+    +-----------+
 *  | selfpath  |    | mainpath  |
 *  | (capture) |    | (capture) |
 *  +-----------+    +-----------+
 */

struct rkisp1_isr_data {
	const char *name;
	irqreturn_t (*isr)(int irq, void *ctx);
};

/* ----------------------------------------------------------------------------
 * Sensor DT bindings
 */

static int rkisp1_create_links(struct rkisp1_device *rkisp1)
{
	struct media_entity *source, *sink;
	unsigned int flags = MEDIA_LNK_FL_ENABLED | MEDIA_LNK_FL_IMMUTABLE;
	unsigned int i;
	int ret;

	/* create ISP->RSZ->CAP links */
	for (i = 0; i < 2; i++) {
		source = &rkisp1->isp.sd.entity;
		sink = &rkisp1->resizer_devs[i].sd.entity;
		ret = media_create_pad_link(source, RKISP1_ISP_PAD_SOURCE_VIDEO,
					    sink, RKISP1_RSZ_PAD_SINK,
					    MEDIA_LNK_FL_ENABLED);
		if (ret)
			return ret;

		source = sink;
		sink = &rkisp1->capture_devs[i].vnode.vdev.entity;
		ret = media_create_pad_link(source, RKISP1_RSZ_PAD_SRC,
					    sink, 0, flags);
		if (ret)
			return ret;
	}

	/* params links */
	source = &rkisp1->params.vnode.vdev.entity;
	sink = &rkisp1->isp.sd.entity;
	ret = media_create_pad_link(source, 0, sink,
				    RKISP1_ISP_PAD_SINK_PARAMS, flags);
	if (ret)
		return ret;

	/* 3A stats links */
	source = &rkisp1->isp.sd.entity;
	sink = &rkisp1->stats.vnode.vdev.entity;
	return media_create_pad_link(source, RKISP1_ISP_PAD_SOURCE_STATS,
				     sink, 0, flags);
}

static int
rkisp1_subdev_notifier_bound(struct v4l2_async_notifier *notifier,
			     struct v4l2_subdev *sd,
			     struct v4l2_async_subdev *asd)
{
	struct rkisp1_device *rkisp1 =
		container_of(notifier, struct rkisp1_device, notifier);
	struct rkisp1_sensor_async *s_asd =
		container_of(asd, struct rkisp1_sensor_async, asd);
	int ret;

	if (s_asd->port == 0)
		return rkisp1_csi_link_sensor(rkisp1, sd, s_asd);

	ret = v4l2_create_fwnode_links_to_pad(sd,
		&rkisp1->isp.pads[RKISP1_ISP_PAD_SINK_VIDEO],
		MEDIA_LNK_FL_ENABLED);
	if (ret < 0) {
		dev_err(rkisp1->dev, "failed to create fwnode links: %d\n", ret);
		return ret;
	}

	return 0;
}

static void
rkisp1_subdev_notifier_unbind(struct v4l2_async_notifier *notifier,
			      struct v4l2_subdev *sd,
			      struct v4l2_async_subdev *asd)
{
	struct rkisp1_device *rkisp1 =
		container_of(notifier, struct rkisp1_device, notifier);
	struct rkisp1_sensor_async *s_asd =
		container_of(asd, struct rkisp1_sensor_async, asd);

	if (rkisp1->info->features & RKISP1_FEATURE_MIPI_CSI2)
		phy_exit(s_asd->dphy);
}

static int rkisp1_subdev_notifier_complete(struct v4l2_async_notifier *notifier)
{
	struct rkisp1_device *rkisp1 =
		container_of(notifier, struct rkisp1_device, notifier);

	return v4l2_device_register_subdev_nodes(&rkisp1->v4l2_dev);
}

static const struct v4l2_async_notifier_operations
rkisp1_subdev_notifier_ops = {
	.bound = rkisp1_subdev_notifier_bound,
	.unbind = rkisp1_subdev_notifier_unbind,
	.complete = rkisp1_subdev_notifier_complete,
};

static int rkisp1_subdev_notifier(struct rkisp1_device *rkisp1)
{
	struct v4l2_async_notifier *ntf = &rkisp1->notifier;
	struct fwnode_handle *fwnode = dev_fwnode(rkisp1->dev);
	struct fwnode_handle *ep;
	int ret = 0;

	v4l2_async_nf_init(ntf);

	fwnode_graph_for_each_endpoint(fwnode, ep) {
		struct fwnode_handle *port;
		struct v4l2_fwnode_endpoint vep = { };
		struct rkisp1_sensor_async *rk_asd;
		u32 reg = 0;

		port = fwnode_get_parent(ep);
		fwnode_property_read_u32(port, "reg", &reg);
		fwnode_handle_put(port);

		/* Assume port 0 if not specified */
		switch (reg) {
		case 0:
			vep.bus_type = V4L2_MBUS_CSI2_DPHY;
			break;
		case 1:
			/*
			 * bus-type property in DT is mandatory; this will be
			 * used to determine if it's PARALLEL or BT656
			 */
			vep.bus_type = V4L2_MBUS_UNKNOWN;
			break;
		}

		ret = v4l2_fwnode_endpoint_parse(ep, &vep);
		if (ret)
			break;

		rk_asd = v4l2_async_nf_add_fwnode_remote(ntf, ep,
							 struct rkisp1_sensor_async);
		if (IS_ERR(rk_asd)) {
			ret = PTR_ERR(rk_asd);
			break;
		}

		if (reg == 0) {
			if (!rkisp1_internal_csi(rkisp1) ||
			    vep.bus_type != V4L2_MBUS_CSI2_DPHY) {
				dev_err(rkisp1->dev,
					"internal CSI must be available for port 0\n");
				ret = -EINVAL;
				break;
			}
		} else {
			/* reg == 1 */
			if (vep.bus_type != V4L2_MBUS_PARALLEL &&
			    vep.bus_type != V4L2_MBUS_BT656) {
				dev_err(rkisp1->dev,
					"external CSI must be parallel or BT656\n");
				ret = -EINVAL;
				break;
			}
		}

		rk_asd->mbus_type = vep.bus_type;
		rk_asd->port = reg;

		if (vep.bus_type == V4L2_MBUS_CSI2_DPHY) {
			rk_asd->mbus_flags = vep.bus.mipi_csi2.flags;
			rk_asd->lanes = vep.bus.mipi_csi2.num_data_lanes;
		} else {
			rk_asd->mbus_flags = vep.bus.parallel.flags;
		}

		dev_dbg(rkisp1->dev, "registered ep id %d, bus type %u, %u lanes\n",
			vep.base.id, rk_asd->mbus_type, rk_asd->lanes);
	}

	if (ret) {
		fwnode_handle_put(ep);
		v4l2_async_nf_cleanup(ntf);
		return ret;
	}

	if (!ep)
		dev_dbg(rkisp1->dev, "no remote subdevice found\n");

	ntf->ops = &rkisp1_subdev_notifier_ops;
	ret = v4l2_async_nf_register(&rkisp1->v4l2_dev, ntf);
	if (ret) {
		v4l2_async_nf_cleanup(ntf);
		return ret;
	}

	return 0;
}

/* ----------------------------------------------------------------------------
 * Power
 */

static int __maybe_unused rkisp1_runtime_suspend(struct device *dev)
{
	struct rkisp1_device *rkisp1 = dev_get_drvdata(dev);

	clk_bulk_disable_unprepare(rkisp1->clk_size, rkisp1->clks);
	return pinctrl_pm_select_sleep_state(dev);
}

static int __maybe_unused rkisp1_runtime_resume(struct device *dev)
{
	struct rkisp1_device *rkisp1 = dev_get_drvdata(dev);
	int ret;

	ret = pinctrl_pm_select_default_state(dev);
	if (ret)
		return ret;
	ret = clk_bulk_prepare_enable(rkisp1->clk_size, rkisp1->clks);
	if (ret)
		return ret;

	return 0;
}

static const struct dev_pm_ops rkisp1_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend,
				pm_runtime_force_resume)
	SET_RUNTIME_PM_OPS(rkisp1_runtime_suspend, rkisp1_runtime_resume, NULL)
};

/* ----------------------------------------------------------------------------
 * Core
 */

static void rkisp1_entities_unregister(struct rkisp1_device *rkisp1)
{
	if (rkisp1->info->features & RKISP1_FEATURE_MIPI_CSI2)
		rkisp1_csi_unregister(rkisp1);
	rkisp1_params_unregister(rkisp1);
	rkisp1_stats_unregister(rkisp1);
	rkisp1_capture_devs_unregister(rkisp1);
	rkisp1_resizer_devs_unregister(rkisp1);
	rkisp1_isp_unregister(rkisp1);
}

static int rkisp1_entities_register(struct rkisp1_device *rkisp1)
{
	int ret;

	ret = rkisp1_isp_register(rkisp1);
	if (ret)
		goto error;

	ret = rkisp1_resizer_devs_register(rkisp1);
	if (ret)
		goto error;

	ret = rkisp1_capture_devs_register(rkisp1);
	if (ret)
		goto error;

	ret = rkisp1_stats_register(rkisp1);
	if (ret)
		goto error;

	ret = rkisp1_params_register(rkisp1);
	if (ret)
		goto error;

	if (rkisp1->info->features & RKISP1_FEATURE_MIPI_CSI2) {
		ret = rkisp1_csi_register(rkisp1);
		if (ret)
			goto error;
	}

	return 0;

error:
	rkisp1_entities_unregister(rkisp1);
	return ret;
}

static irqreturn_t rkisp1_mipi_isr(int irq, void *ctx)
{
	struct device *dev = ctx;
	struct rkisp1_device *rkisp1 = dev_get_drvdata(dev);
	bool handled;

	v4l2_subdev_call(rkisp1->csi_subdev, core, interrupt_service_routine,
			 irq, &handled);

	return IRQ_HANDLED;
}

static irqreturn_t rkisp1_isr(int irq, void *ctx)
{
	/*
	 * Call rkisp1_capture_isr() first to handle the frame that
	 * potentially completed using the current frame_sequence number before
	 * it is potentially incremented by rkisp1_isp_isr() in the vertical
	 * sync.
	 */
	rkisp1_capture_isr(irq, ctx);
	rkisp1_isp_isr(irq, ctx);
	rkisp1_mipi_isr(irq, ctx);

	return IRQ_HANDLED;
}

static const char * const px30_isp_clks[] = {
	"isp",
	"aclk",
	"hclk",
	"pclk",
};

static const struct rkisp1_isr_data px30_isp_isrs[] = {
	{ "isp", rkisp1_isp_isr },
	{ "mi", rkisp1_capture_isr },
	{ "mipi", rkisp1_mipi_isr },
};

static const struct rkisp1_info px30_isp_info = {
	.clks = px30_isp_clks,
	.clk_size = ARRAY_SIZE(px30_isp_clks),
	.isrs = px30_isp_isrs,
	.isr_size = ARRAY_SIZE(px30_isp_isrs),
	.isp_ver = RKISP1_V12,
	.features = RKISP1_FEATURE_DUAL_CROP
		  | RKISP1_FEATURE_MIPI_CSI2,
};

static const char * const rk3399_isp_clks[] = {
	"isp",
	"aclk",
	"hclk",
};

static const struct rkisp1_isr_data rk3399_isp_isrs[] = {
	{ NULL, rkisp1_isr },
};

static const struct rkisp1_info rk3399_isp_info = {
	.clks = rk3399_isp_clks,
	.clk_size = ARRAY_SIZE(rk3399_isp_clks),
	.isrs = rk3399_isp_isrs,
	.isr_size = ARRAY_SIZE(rk3399_isp_isrs),
	.isp_ver = RKISP1_V10,
	.features = RKISP1_FEATURE_DUAL_CROP
		  | RKISP1_FEATURE_MIPI_CSI2,
};

static const char * const imx8mp_isp_clks[] = {
	"isp",
	"hclk",
	"aclk",
};

static const struct rkisp1_isr_data imx8mp_isp_isrs[] = {
	{ NULL, rkisp1_isr },
};

static const struct rkisp1_info imx8mp_isp_info = {
	.clks = imx8mp_isp_clks,
	.clk_size = ARRAY_SIZE(imx8mp_isp_clks),
	.isrs = imx8mp_isp_isrs,
	.isr_size = ARRAY_SIZE(imx8mp_isp_isrs),
	.isp_ver = IMX8MP_V10,
	.features = RKISP1_FEATURE_RSZ_CROP
		  | RKISP1_FEATURE_MAIN_STRIDE,
};

static const struct of_device_id rkisp1_of_match[] = {
	{
		.compatible = "rockchip,px30-cif-isp",
		.data = &px30_isp_info,
	},
	{
		.compatible = "rockchip,rk3399-cif-isp",
		.data = &rk3399_isp_info,
	},
	{
		.compatible = "fsl,imx8mp-isp",
		.data = &imx8mp_isp_info,
	},
	{},
};
MODULE_DEVICE_TABLE(of, rkisp1_of_match);

static int rkisp1_probe(struct platform_device *pdev)
{
	const struct rkisp1_info *info;
	struct device *dev = &pdev->dev;
	struct rkisp1_device *rkisp1;
	struct v4l2_device *v4l2_dev;
	unsigned int i;
	int ret, irq;
	u32 cif_id;

	rkisp1 = devm_kzalloc(dev, sizeof(*rkisp1), GFP_KERNEL);
	if (!rkisp1)
		return -ENOMEM;

	info = of_device_get_match_data(dev);
	rkisp1->info = info;

	dev_set_drvdata(dev, rkisp1);
	rkisp1->dev = dev;

	mutex_init(&rkisp1->stream_lock);

	rkisp1->base_addr = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(rkisp1->base_addr))
		return PTR_ERR(rkisp1->base_addr);

	for (i = 0; i < info->isr_size; i++) {
		irq = info->isrs[i].name
		    ? platform_get_irq_byname(pdev, info->isrs[i].name)
		    : platform_get_irq(pdev, i);
		if (irq < 0)
			return irq;

		ret = devm_request_irq(dev, irq, info->isrs[i].isr, IRQF_SHARED,
				       dev_driver_string(dev), dev);
		if (ret) {
			dev_err(dev, "request irq failed: %d\n", ret);
			return ret;
		}
	}

	for (i = 0; i < info->clk_size; i++)
		rkisp1->clks[i].id = info->clks[i];
	ret = devm_clk_bulk_get(dev, info->clk_size, rkisp1->clks);
	if (ret)
		return ret;
	rkisp1->clk_size = info->clk_size;

	if (info->isp_ver == IMX8MP_V10) {
		unsigned int id;

		rkisp1->gasket = syscon_regmap_lookup_by_phandle_args(dev->of_node,
								      "fsl,blk-ctrl",
								      1, &id);
		if (IS_ERR(rkisp1->gasket)) {
			ret = PTR_ERR(rkisp1->gasket);
			dev_err(dev, "failed to get gasket: %d\n", ret);
			return ret;
		}

		rkisp1->gasket_id = id;
	}

	pm_runtime_enable(&pdev->dev);

	rkisp1->media_dev.hw_revision = info->isp_ver;
	strscpy(rkisp1->media_dev.model, RKISP1_DRIVER_NAME,
		sizeof(rkisp1->media_dev.model));
	rkisp1->media_dev.dev = &pdev->dev;
	strscpy(rkisp1->media_dev.bus_info, RKISP1_BUS_INFO,
		sizeof(rkisp1->media_dev.bus_info));
	media_device_init(&rkisp1->media_dev);

	v4l2_dev = &rkisp1->v4l2_dev;
	v4l2_dev->mdev = &rkisp1->media_dev;
	strscpy(v4l2_dev->name, RKISP1_DRIVER_NAME, sizeof(v4l2_dev->name));

	ret = v4l2_device_register(rkisp1->dev, &rkisp1->v4l2_dev);
	if (ret)
		return ret;

	ret = media_device_register(&rkisp1->media_dev);
	if (ret) {
		dev_err(dev, "Failed to register media device: %d\n", ret);
		goto err_unreg_v4l2_dev;
	}

	ret = rkisp1_entities_register(rkisp1);
	if (ret)
		goto err_unreg_media_dev;

	ret = rkisp1_subdev_notifier(rkisp1);
	if (ret)
		goto err_unreg_entities;

	ret = rkisp1_create_links(rkisp1);
	if (ret)
		goto err_unreg_notifier;

	rkisp1_debug_init(rkisp1);

	// todo pm runtime get
	ret = pm_runtime_resume_and_get(rkisp1->dev);
	if (ret < 0) {
		dev_err(rkisp1->dev, "failed to power on\n");
		goto err_unreg_notifier;
	}

	cif_id = rkisp1_read(rkisp1, RKISP1_CIF_VI_ID);
	dev_info(rkisp1->dev, "CIF_ID 0x%08x\n", cif_id);

	// todo pm runtime put

	return 0;

err_unreg_notifier:
	v4l2_async_nf_unregister(&rkisp1->notifier);
	v4l2_async_nf_cleanup(&rkisp1->notifier);
err_unreg_entities:
	rkisp1_entities_unregister(rkisp1);
err_unreg_media_dev:
	media_device_unregister(&rkisp1->media_dev);
err_unreg_v4l2_dev:
	v4l2_device_unregister(&rkisp1->v4l2_dev);
	pm_runtime_disable(&pdev->dev);
	return ret;
}

static int rkisp1_remove(struct platform_device *pdev)
{
	struct rkisp1_device *rkisp1 = platform_get_drvdata(pdev);

	v4l2_async_nf_unregister(&rkisp1->notifier);
	v4l2_async_nf_cleanup(&rkisp1->notifier);

	rkisp1_entities_unregister(rkisp1);
	rkisp1_debug_cleanup(rkisp1);

	media_device_unregister(&rkisp1->media_dev);
	v4l2_device_unregister(&rkisp1->v4l2_dev);

	pm_runtime_disable(&pdev->dev);

	return 0;
}

static struct platform_driver rkisp1_drv = {
	.driver = {
		.name = RKISP1_DRIVER_NAME,
		.of_match_table = of_match_ptr(rkisp1_of_match),
		.pm = &rkisp1_pm_ops,
	},
	.probe = rkisp1_probe,
	.remove = rkisp1_remove,
};

module_platform_driver(rkisp1_drv);
MODULE_DESCRIPTION("Rockchip ISP1 platform driver");
MODULE_LICENSE("Dual MIT/GPL");
