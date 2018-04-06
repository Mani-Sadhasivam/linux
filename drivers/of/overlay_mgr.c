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
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/slab.h>

#include "of_private.h"

#define MAX_OVERLAY 10

struct of_overlay_data {
	const struct firmware *fw;
	struct device_node *overlay;
	int ovcs_id[MAX_OVERLAY];
	char *entry;
};

static char *of_overlay_entry[MAX_OVERLAY];
static unsigned int of_overlay_count;

module_param_array_named(overlays, of_overlay_entry, charp,
				&of_overlay_count, 0);

static int create_overlay(struct of_overlay_data *data, void *blob, int idx)
{
	int ret;

	/* Unflatten the overlay */
	of_fdt_unflatten_tree(blob, NULL, &data->overlay);
	if (data->overlay == NULL) {
		pr_err("Failed to unflatten tree\n");
		return -EINVAL;
	}

	/* Mark overlay as detached */
	of_node_set_flag(data->overlay, OF_DETACHED);

	/* Perform a local resolution before applying overlay */
	ret = of_resolve_phandles(data->overlay);
	if (ret < 0) {
		pr_err("Failed to resolve phandles\n");
		return ret;
	}

	/* Apply overlay */
	ret = of_overlay_apply(data->overlay, &data->ovcs_id[idx]);
	if (ret < 0) {
		pr_err("Failed to apply overlay: %s\n", data->entry);
		return -EINVAL;
	}
	pr_debug("Overlay applied: %s\n", data->entry);

	return ret;
}

static int of_overlay_mgr_apply(struct of_overlay_data *data)
{
	int ret = 0, idx;

	if (!of_overlay_count) {
		pr_debug("No overlay entry found\n");
		return ret;
	}

	for (idx = 0; idx < of_overlay_count; idx++) {
		data->entry = kasprintf(GFP_KERNEL, "%s.dtbo",
					of_overlay_entry[idx]);

		/*
		 * Request dtbo for the overlay name(s) provided
		 * via kernel command line
		 */
		ret = request_firmware(&data->fw, data->entry, NULL);
		if (ret < 0)
			goto err;

		ret = create_overlay(data, (void *)data->fw->data, idx);
		if (ret < 0)
			goto err;
	}

	return ret;

err:
	release_firmware(data->fw);

	return ret;
}

static int __init of_overlay_mgr_init(void)
{
	struct of_overlay_data *data;

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	return of_overlay_mgr_apply(data);
}

late_initcall(of_overlay_mgr_init);
