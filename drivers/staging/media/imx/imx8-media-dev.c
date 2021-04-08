// SPDX-License-Identifier: GPL-2.0
/*
 * V4L2 Media Controller Driver for NXP IMX8QXP/QM SOC
 *
 * Copyright (c) 2019 NXP Semiconductor
 *
 */

#include <linux/device.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>

#include <media/media-device.h>
#include <media/v4l2-async.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#include "imx8-common.h"

#define MXC_MD_DRIVER_NAME	"mxc-md"

enum mxc_md_entity_type {
	MXC_MD_ENTITY_ISI,
	MXC_MD_ENTITY_MIPI_CSI2,
	MXC_MD_ENTITY_PARALLEL_CSI,
};

struct mxc_md_async_subdev {
	struct v4l2_async_subdev asd;
	struct list_head list;
	struct v4l2_subdev *sd;

	enum mxc_md_entity_type type;
	u32 interface[MAX_PORTS];
	int id;
};

struct mxc_md {
	struct media_device media_dev;
	struct v4l2_device v4l2_dev;
	struct platform_device *pdev;

	struct v4l2_async_notifier notifier;
	struct list_head asds;
};

static inline struct mxc_md_async_subdev *
asd_to_mxc_md_async_subdev(struct v4l2_async_subdev *asd)
{
	return container_of(asd, struct mxc_md_async_subdev, asd);
};

static inline struct mxc_md *notifier_to_mxc_md(struct v4l2_async_notifier *n)
{
	return container_of(n, struct mxc_md, notifier);
};

static int mxc_md_create_links(struct mxc_md *mxc_md,
			       struct mxc_md_async_subdev *masd_isi)
{
	const unsigned int link_flags = MEDIA_LNK_FL_IMMUTABLE
				      | MEDIA_LNK_FL_ENABLED;
	struct mxc_md_async_subdev *masd;
	enum mxc_md_entity_type type;
	struct media_entity *source = NULL;
	struct media_entity *sink = &masd_isi->sd->entity;
	unsigned int source_pad;
	unsigned int sink_pad;
	int ret;
	int id;

	/* Create the link between the ISI and its input. */
	switch (masd_isi->interface[IN_PORT]) {
	case ISI_INPUT_INTERFACE_MIPI0_CSI2:
		source_pad = MXC_MIPI_CSI2_PAD_SOURCE;
		sink_pad = MXC_ISI_SD_PAD_SINK_MIPI0;
		type = MXC_MD_ENTITY_MIPI_CSI2;
		id = 0;
		break;

	case ISI_INPUT_INTERFACE_MIPI1_CSI2:
		source_pad = MXC_MIPI_CSI2_PAD_SOURCE;
		sink_pad = MXC_ISI_SD_PAD_SINK_MIPI1;
		type = MXC_MD_ENTITY_MIPI_CSI2;
		id = 1;
		break;

	case ISI_INPUT_INTERFACE_PARALLEL_CSI:
		source_pad = MXC_PARALLEL_CSI_PAD_SOURCE;
		sink_pad = MXC_ISI_SD_PAD_SINK_PARALLEL_CSI;
		type = MXC_MD_ENTITY_PARALLEL_CSI;
		id = 0;
		break;
	}

	list_for_each_entry(masd, &mxc_md->asds, list) {
		if (masd->type == type && masd->id == id) {
			source = &masd->sd->entity;
			break;
		}
	}

	if (!source) {
		dev_err(&mxc_md->pdev->dev, "Failed to find source for %s\n",
			sink->name);
		return -ENODEV;
	}

	ret = media_create_pad_link(source, source_pad, sink, sink_pad,
				    link_flags);
	if (ret < 0) {
		dev_err(&mxc_md->pdev->dev,
			"Failed to create link [%s] -> [%s]\n",
			source->name, sink->name);
		return ret;
	}

	/* Notify the source and sink (ISI). */
	media_entity_call(source, link_setup, &source->pads[source_pad],
			  &sink->pads[sink_pad], link_flags);
	media_entity_call(sink, link_setup, &sink->pads[sink_pad],
			  &source->pads[source_pad], link_flags);

	dev_dbg(&mxc_md->pdev->dev, "Created link [%s] -> [%s]\n",
		source->name, sink->name);

	return 0;
}

static int mxc_md_async_notifier_bound(struct v4l2_async_notifier *notifier,
				       struct v4l2_subdev *sd,
				       struct v4l2_async_subdev *asd)
{
	struct mxc_md *mxc_md = notifier_to_mxc_md(notifier);
	struct mxc_md_async_subdev *masd = asd_to_mxc_md_async_subdev(asd);

	dev_dbg(&mxc_md->pdev->dev, "Bound subdev %s\n", sd->name);

	masd->sd = sd;

	return 0;
}

static int mxc_md_async_notifier_complete(struct v4l2_async_notifier *notifier)
{
	struct mxc_md *mxc_md = notifier_to_mxc_md(notifier);
	struct mxc_md_async_subdev *masd;
	int ret;

	dev_dbg(&mxc_md->pdev->dev, "%s\n", __func__);

	list_for_each_entry(masd, &mxc_md->asds, list) {
		if (masd->type != MXC_MD_ENTITY_ISI)
			continue;

		ret = mxc_md_create_links(mxc_md, masd);
		if (ret < 0) {
			dev_err(&mxc_md->pdev->dev,
				"Failed to create link for %s: %d\n",
				masd->sd->name, ret);
			return ret;
		}
	}

	ret = v4l2_device_register_subdev_nodes(&mxc_md->v4l2_dev);
	if (ret < 0) {
		dev_err(&mxc_md->pdev->dev,
			"Failed to register subdev nodes: %d\n", ret);
		return ret;
	}

	return media_device_register(&mxc_md->media_dev);
}

static const struct v4l2_async_notifier_operations mxc_md_async_notifier_ops = {
	.bound = mxc_md_async_notifier_bound,
	.complete = mxc_md_async_notifier_complete,
};

static int mxc_md_register_async_subdevs(struct mxc_md *mxc_md)
{
	const struct mxc_md_subdev_info {
		enum mxc_md_entity_type type;
		const char *node;
	} subdevs[] = {
		{ MXC_MD_ENTITY_ISI, "isi" },
		{ MXC_MD_ENTITY_MIPI_CSI2, "csi" },
		{ MXC_MD_ENTITY_PARALLEL_CSI, "pcsi" },
	};

	struct device_node *node;
	int ret;

	for_each_available_child_of_node(mxc_md->pdev->dev.of_node, node) {
		const struct mxc_md_subdev_info *info = NULL;
		struct mxc_md_async_subdev *masd;
		unsigned int i;

		/* Skip disabled or unknown nodes. */
		if (!of_device_is_available(node))
			continue;

		for (i = 0; i < ARRAY_SIZE(subdevs); ++i) {
			if (!strcmp(node->name, subdevs[i].node)) {
				info = &subdevs[i];
				break;
			}
		}

		if (!info)
			continue;

		/*
		 * Add an entry to the notifier and fill it with the type, the
		 * interface (for ISI only) and the instance ID (for CSI-2
		 * only).
		 */
		masd = v4l2_async_notifier_add_fwnode_subdev(&mxc_md->notifier,
							     of_fwnode_handle(node),
							     struct mxc_md_async_subdev);
		if (IS_ERR(masd)) {
			dev_err(&mxc_md->pdev->dev,
				"Failed to add node %pOF to subdev notifier: %d\n",
				node, ret);
			of_node_put(node);
			return PTR_ERR(masd);
		}

		masd->type = info->type;

		if (info->type == MXC_MD_ENTITY_ISI) {
			ret = of_property_read_u32_array(node, "interface",
							 masd->interface,
							 ARRAY_SIZE(masd->interface));
			if (ret < 0) {
				dev_err(&mxc_md->pdev->dev,
					"Failed to get %pOF interface: %d\n",
					node, ret);
				return -ENODEV;
			}

			switch (masd->interface[IN_PORT]) {
			case ISI_INPUT_INTERFACE_MIPI0_CSI2:
			case ISI_INPUT_INTERFACE_MIPI1_CSI2:
			case ISI_INPUT_INTERFACE_PARALLEL_CSI:
				break;

			case ISI_INPUT_INTERFACE_HDMI:
			case ISI_INPUT_INTERFACE_DC0:
			case ISI_INPUT_INTERFACE_DC1:
			case ISI_INPUT_INTERFACE_MEM:
			default:
				dev_err(&mxc_md->pdev->dev,
					"Unsupported input interface %u for %pOF\n",
					masd->interface[IN_PORT], node);
				return -EINVAL;
			}
		} else if (info->type == MXC_MD_ENTITY_MIPI_CSI2) {
			masd->id = of_alias_get_id(node, info->node);
			if (masd->id < 0) {
				dev_err(&mxc_md->pdev->dev,
					"Failed to get %pOF instance ID: %d\n",
					node, masd->id);
				return -EINVAL;
			}
		}

		list_add_tail(&masd->list, &mxc_md->asds);
	}

	return 0;
}

static int mxc_md_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct v4l2_device *v4l2_dev;
	struct mxc_md *mxc_md;
	int ret;

	mxc_md = devm_kzalloc(dev, sizeof(*mxc_md), GFP_KERNEL);
	if (!mxc_md)
		return -ENOMEM;

	INIT_LIST_HEAD(&mxc_md->asds);
	mxc_md->pdev = pdev;
	platform_set_drvdata(pdev, mxc_md);

	/* register media device  */
	strlcpy(mxc_md->media_dev.model, "FSL Capture Media Device",
		sizeof(mxc_md->media_dev.model));
	mxc_md->media_dev.dev = dev;

	media_device_init(&mxc_md->media_dev);

	/* register v4l2 device */
	v4l2_dev = &mxc_md->v4l2_dev;
	v4l2_dev->mdev = &mxc_md->media_dev;
	strlcpy(v4l2_dev->name, "mx8-img-md", sizeof(v4l2_dev->name));

	ret = v4l2_device_register(dev, &mxc_md->v4l2_dev);
	if (ret < 0) {
		v4l2_err(v4l2_dev, "Failed to register v4l2_device (%d)\n", ret);
		goto err_media;
	}

	/* Initialize, fill and register the async notifier. */
	v4l2_async_notifier_init(&mxc_md->notifier);
	mxc_md->notifier.ops = &mxc_md_async_notifier_ops;

	ret = mxc_md_register_async_subdevs(mxc_md);
	if (ret < 0) {
		dev_err(&mxc_md->pdev->dev,
			"Failed to register async subdevs: %d\n", ret);
		goto err_v4l2;
	}

	ret = v4l2_async_notifier_register(&mxc_md->v4l2_dev,
					   &mxc_md->notifier);
	if (ret < 0) {
		dev_err(&mxc_md->pdev->dev,
			"Failed to register async notifier: %d\n", ret);
		goto err_v4l2;
	}

	return 0;

err_v4l2:
	v4l2_device_unregister(&mxc_md->v4l2_dev);
err_media:
	media_device_cleanup(&mxc_md->media_dev);
	return ret;
}

static int mxc_md_remove(struct platform_device *pdev)
{
	struct mxc_md *mxc_md = platform_get_drvdata(pdev);

	v4l2_async_notifier_unregister(&mxc_md->notifier);

	v4l2_device_unregister(&mxc_md->v4l2_dev);
	media_device_unregister(&mxc_md->media_dev);
	media_device_cleanup(&mxc_md->media_dev);

	return 0;
}

static const struct of_device_id mxc_md_of_match[] = {
	{ .compatible = "fsl,mxc-md" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, mxc_md_of_match);

static struct platform_driver mxc_md_driver = {
	.driver = {
		.name = MXC_MD_DRIVER_NAME,
		.of_match_table	= mxc_md_of_match,
	},
	.probe = mxc_md_probe,
	.remove = mxc_md_remove,
};

module_platform_driver(mxc_md_driver);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("MXC Media Device driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" MXC_MD_DRIVER_NAME);
