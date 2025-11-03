// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * Author: Manivannan Sadhasivam <manivannan.sadhasivam@oss.qualcomm.com>
 */

#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/of_platform.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/pwrseq/provider.h>
#include <linux/regulator/consumer.h>
#include <linux/serdev.h>
#include <linux/slab.h>

struct pwrseq_pci_m2_pdata {
	const char *const *vregs;
	size_t num_vregs;
	const struct pwrseq_target_data **targets;
};

struct pwrseq_pci_m2_ctx {
	struct pwrseq_device *pwrseq;
	const struct pwrseq_pci_m2_pdata *pdata;
	struct regulator_bulk_data *regs;
	struct gpio_desc *bt_gpio;
	struct gpio_desc *wlan_gpio;
	struct notifier_block nb;
	struct device *dev;
};

static int pwrseq_pci_m2_vregs_enable(struct pwrseq_device *pwrseq)
{
	struct pwrseq_pci_m2_ctx *ctx = pwrseq_device_get_drvdata(pwrseq);

	return regulator_bulk_enable(ctx->pdata->num_vregs, ctx->regs);
}

static int pwrseq_pci_m2_vregs_disable(struct pwrseq_device *pwrseq)
{
	struct pwrseq_pci_m2_ctx *ctx = pwrseq_device_get_drvdata(pwrseq);

	return regulator_bulk_disable(ctx->pdata->num_vregs, ctx->regs);
}

static const struct pwrseq_unit_data pwrseq_pci_m2_vregs_unit_data = {
	.name = "regulators-enable",
	.enable = pwrseq_pci_m2_vregs_enable,
	.disable = pwrseq_pci_m2_vregs_disable,
};

static const struct pwrseq_unit_data *pwrseq_pci_m2_unit_deps[] = {
	&pwrseq_pci_m2_vregs_unit_data,
	NULL
};

static int pwrseq_pci_m2_wlan_enable(struct pwrseq_device *pwrseq)
{
	struct pwrseq_pci_m2_ctx *ctx = pwrseq_device_get_drvdata(pwrseq);

	gpiod_set_value_cansleep(ctx->wlan_gpio, 1);

	return 0;
}

static int pwrseq_pci_m2_wlan_disable(struct pwrseq_device *pwrseq)
{
	struct pwrseq_pci_m2_ctx *ctx = pwrseq_device_get_drvdata(pwrseq);

	pr_info("#### %s: %d", __func__, __LINE__);

	gpiod_set_value_cansleep(ctx->wlan_gpio, 0);

	return 0;
}

static const struct pwrseq_unit_data pwrseq_pci_m2_wlan_unit_data = {
	.name = "wlan-enable",
	.deps = pwrseq_pci_m2_unit_deps,
	.enable = pwrseq_pci_m2_wlan_enable,
	.disable = pwrseq_pci_m2_wlan_disable,
};

static int pwrseq_pci_m2_bt_enable(struct pwrseq_device *pwrseq)
{
	struct pwrseq_pci_m2_ctx *ctx = pwrseq_device_get_drvdata(pwrseq);

	pr_info("#### %s: %d", __func__, __LINE__);

	gpiod_set_value_cansleep(ctx->bt_gpio, 1);

	return 0;
}

static int pwrseq_pci_m2_bt_disable(struct pwrseq_device *pwrseq)
{
	struct pwrseq_pci_m2_ctx *ctx = pwrseq_device_get_drvdata(pwrseq);

	gpiod_set_value_cansleep(ctx->bt_gpio, 0);

	return 0;
}

static const struct pwrseq_unit_data pwrseq_pci_m2_bt_unit_data = {
	.name = "bluetooth-enable",
	.deps = pwrseq_pci_m2_unit_deps,
	.enable = pwrseq_pci_m2_bt_enable,
	.disable = pwrseq_pci_m2_bt_disable,
};

static const struct pwrseq_target_data pwrseq_pci_m2_bt_target_data = {
	.name = "bluetooth",
	.unit = &pwrseq_pci_m2_bt_unit_data,
};

static const struct pwrseq_target_data pwrseq_pci_m2_wlan_target_data = {
	.name = "wlan",
	.unit = &pwrseq_pci_m2_wlan_unit_data,
};

static const struct pwrseq_target_data *pwrseq_pci_m2_targets[] = {
	&pwrseq_pci_m2_bt_target_data,
	&pwrseq_pci_m2_wlan_target_data,
	NULL
};

static const char *const pwrseq_pci_m2_a_vregs[] = {
	"vpcie3v3",
	"vio1v8",
};

static const struct pwrseq_pci_m2_pdata pwrseq_pci_m2_a_of_data = {
	.vregs = pwrseq_pci_m2_a_vregs,
	.num_vregs = ARRAY_SIZE(pwrseq_pci_m2_a_vregs),
	.targets = pwrseq_pci_m2_targets,
};

static int pwrseq_pci_m2_match(struct pwrseq_device *pwrseq,
				 struct device *dev)
{
	//struct pwrseq_pci_m2_ctx *ctx = pwrseq_device_get_drvdata(pwrseq);

	pr_info("#### %s: %d", __func__, __LINE__);

	return PWRSEQ_MATCH_OK;
}

static int pci_connector_m2_notify(struct notifier_block *nb, unsigned long action,
			      void *data)
{
	struct pwrseq_pci_m2_ctx *ctx = container_of(nb, struct pwrseq_pci_m2_ctx, nb);
	struct pci_dev *pdev = to_pci_dev(data);
	struct device_node *remote;
	struct serdev_controller *serdev_ctrl;
	struct serdev_device *serdev;
	struct device *dev = ctx->dev;
	int ret;

	pci_info(pdev, "#### %s: %d", __func__, __LINE__);
	/* Check whether the PCI device is associated with this M.2 connector */
	remote = of_graph_get_remote_node(dev_of_node(ctx->dev), 0, -1);
	if (!remote || (remote != pdev->dev.parent->of_node)) {
		of_node_put(remote);
		return NOTIFY_DONE;
	}
	of_node_put(remote);

	switch (action) {
	case BUS_NOTIFY_ADD_DEVICE:
		/* Create BT device for WCN7850 */
		if (pdev->vendor == PCI_VENDOR_ID_QCOM && pdev->device == 0x1107) {
			pci_info(pdev, "#### %s: %d", __func__, __LINE__);

			remote = of_graph_get_remote_node(dev_of_node(ctx->dev), 1, -1);
			if (!remote) {
				of_node_put(remote);
				return NOTIFY_DONE;
			}

			pci_info(pdev, "#### %s: %d", __func__, __LINE__);
			serdev_ctrl = of_find_serdev_controller_by_node(remote);
			of_node_put(remote);
			if (!serdev_ctrl)
				return NOTIFY_DONE;

			pci_info(pdev, "#### %s: %d", __func__, __LINE__);
			serdev = serdev_device_alloc(serdev_ctrl);
			if (!serdev)
				return NOTIFY_DONE;

			pci_info(pdev, "#### %s: %d", __func__, __LINE__);
			ret = serdev_device_add(serdev, "WCN7850");
			if (ret) {
				dev_err(dev, "Failed to add serdev for WCN7850: %d\n", ret);
				serdev_device_put(serdev);
				return NOTIFY_DONE;
			}
		}
		break;
	}

	return NOTIFY_DONE;
}
static int pwrseq_pci_m2_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct pwrseq_pci_m2_ctx *ctx;
	struct pwrseq_config config;
	int ret;
	u32 i;

	pr_info("#### %s: %d", __func__, __LINE__);

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	pr_info("#### %s: %d", __func__, __LINE__);

	ctx->dev = dev;
	ctx->pdata = of_device_get_match_data(dev);
	if (!ctx->pdata)
		return dev_err_probe(dev, -ENODEV,
				     "Failed to obtain platform data\n");

	ctx->regs = devm_kcalloc(dev, ctx->pdata->num_vregs,
				 sizeof(*ctx->regs), GFP_KERNEL);
	if (!ctx->regs)
		return -ENOMEM;

	for (i = 0; i < ctx->pdata->num_vregs; i++)
		ctx->regs[i].supply = ctx->pdata->vregs[i];

	ret = devm_regulator_bulk_get(dev, ctx->pdata->num_vregs, ctx->regs);
	if (ret < 0)
		return dev_err_probe(dev, ret,
				     "Failed to get all regulators\n");

	pr_info("#### %s: %d", __func__, __LINE__);

	ctx->bt_gpio = devm_gpiod_get_optional(dev, "w-disable2", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->bt_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->bt_gpio),
				     "Failed to get the Bluetooth enable GPIO\n");

	/*
	 * FIXME: This should actually be GPIOD_OUT_LOW, but doing so would
	 * cause the WLAN power to be toggled, resulting in PCIe link down.
	 * Since the PCIe controller driver is not handling link down currently,
	 * the device becomes unusable. So we need to keep this workaround until
	 * the link down handling is implemented in the controller driver.
	 */
	ctx->wlan_gpio = devm_gpiod_get_optional(dev, "w-disable1",
						 GPIOD_ASIS);
	if (IS_ERR(ctx->wlan_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->wlan_gpio),
				     "Failed to get the WLAN enable GPIO\n");

	pr_info("#### %s: %d", __func__, __LINE__);

	memset(&config, 0, sizeof(config));

	config.parent = dev;
	config.owner = THIS_MODULE;
	config.drvdata = ctx;
	config.match = pwrseq_pci_m2_match;
	config.targets = ctx->pdata->targets;

	ctx->pwrseq = devm_pwrseq_device_register(dev, &config);
	if (IS_ERR(ctx->pwrseq))
		return dev_err_probe(dev, PTR_ERR(ctx->pwrseq),
				     "Failed to register the power sequencer\n");

	ctx->nb.notifier_call = pci_connector_m2_notify;
	ret = bus_register_notifier(&pci_bus_type, &ctx->nb);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to register notifier\n");

	return 0;
}

static const struct of_device_id pwrseq_pci_m2_of_match[] = {
	{
		.compatible = "pci-m2-a-connector",
		.data = &pwrseq_pci_m2_a_of_data,
	},
	{ }
};
MODULE_DEVICE_TABLE(of, pwrseq_pci_m2_of_match);

static struct platform_driver pwrseq_pci_m2_driver = {
	.driver = {
		.name = "pwrseq-pci-m2",
		.of_match_table = pwrseq_pci_m2_of_match,
	},
	.probe = pwrseq_pci_m2_probe,
};
module_platform_driver(pwrseq_pci_m2_driver);

MODULE_AUTHOR("Manivannan Sadhasivam <manivannan.sadhasivam@oss.qualcomm.com>");
MODULE_DESCRIPTION("Generic PCI Power Control driver for M.2 device");
MODULE_LICENSE("GPL");
