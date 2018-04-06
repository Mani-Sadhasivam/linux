// SPDX-License-Identifier: GPL-2.0+
/*
 * Overlay manager for applying list of overlays specified using command line
 *
 * Copyright (C) 2018 Linaro Ltd.
 * Author: Manivannan Sadhasivam <manivannan.sadhasivam@linaro.org>
 *
 * Derived from the following works:
 *
 * 1. ConfigFS interface
 *    Pantelis Antoniou <panto@antoniou-consulting.com>
 *
 * 2. Overlay Manager
 *    Dmitry Shmidt <dimitrysh@google.com>
 */

#define pr_fmt(fmt) "OF: overlay_mgr: " fmt

#include <linux/firmware.h>
#include <linux/kernel.h>
#include <linux/libfdt.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include "of_private.h"

struct of_overlay_data {
	const struct firmware *fw;
	struct device_node *overlay;
};

static char *of_overlay_dt_entry;
module_param_named(overlay_dt_entry, of_overlay_dt_entry, charp, 0);

static int of_overlay_mgr_apply_overlay(void *blob, char *entry)
{
	int ret, ovcs_id;

	ret = of_overlay_fdt_apply(blob, fdt_totalsize(blob), &ovcs_id);
	if (ret < 0) {
		pr_err("failed to create overlay: %s\n", entry);
		return ret;
	}

	pr_info("overlay applied: %s\n", entry);

	return 0;
}

static int of_overlay_mgr_apply_dt(struct device *dev, char *dt_entry)
{
	struct device_node *enp = dev->of_node;
	struct device_node *next;
	struct device_node *prev = NULL;
	int ret = 0;

	enp = of_get_child_by_name(enp, dt_entry);
	if (!enp) {
		pr_err("dt entry not found: %s\n", dt_entry);
		return -ENODEV;
	}

	pr_info("applying dt entry: %s\n", enp->name);
	while ((next = of_get_next_available_child(enp, prev)) != NULL) {
		if (strncmp(next->name, "overlay", 7) == 0) {
			ret = of_overlay_mgr_apply_overlay(next, dt_entry);
			if (ret < 0)
				break;
		}
		prev = next;
	}

	return ret;
}

static int of_overlay_mgr_probe(struct platform_device *pdev)
{
	struct of_overlay_data *data;
	char *cur_entry, *next_entry;
	int ret;

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	if (!of_overlay_dt_entry) {
		pr_debug("no overlay entry found\n");
		return 0;
	}

	next_entry = of_overlay_dt_entry;
	do {
		cur_entry = next_entry;
		next_entry = strchr(cur_entry, ',');
		if (next_entry)
			*next_entry++ = '\0';

		ret = of_overlay_mgr_apply_dt(&pdev->dev, cur_entry);
		if (ret == 0)
			continue;

		ret = request_firmware(&data->fw, cur_entry, NULL);
		if (ret < 0) {
			release_firmware(data->fw);
			continue;
		}

		ret = of_overlay_mgr_apply_overlay((void *)data->fw->data,
							cur_entry);
		if (ret < 0)
			release_firmware(data->fw);

	} while (next_entry);

	return 0;
}

static const struct of_device_id of_overlay_mgr_match[] = {
	{ .compatible = "linux,overlay_manager", },
	{ /* sentinel */ }
};

static struct platform_driver of_overlay_mgr_driver = {
	.probe	= of_overlay_mgr_probe,
	.driver	= {
		.name = "overlay_manager",
		.of_match_table = of_match_ptr(of_overlay_mgr_match),
	},
};

static int __init of_overlay_mgr_init(void)
{
	return platform_driver_register(&of_overlay_mgr_driver);
}

late_initcall(of_overlay_mgr_init);
