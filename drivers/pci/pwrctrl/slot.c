// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2024 Linaro Ltd.
 * Author: Manivannan Sadhasivam <manivannan.sadhasivam@linaro.org>
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/of_platform.h>
#include <linux/pci-pwrctrl.h>
#include <linux/platform_device.h>
#include <linux/pwrseq/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>

struct pci_pwrctrl_slot_data {
	struct pci_pwrctrl ctx;
	struct regulator_bulk_data *supplies;
	int num_supplies;
	struct gpio_desc *w_dis1_gpio;
	struct pwrseq_desc *pwrseq;
};

static void devm_pci_pwrctrl_slot_power_off(void *data)
{
	struct pci_pwrctrl_slot_data *slot = data;

	gpiod_set_value_cansleep(slot->w_dis1_gpio, 0);
	regulator_bulk_disable(slot->num_supplies, slot->supplies);
	regulator_bulk_free(slot->num_supplies, slot->supplies);
}

static int pci_pwrctrl_slot_probe(struct platform_device *pdev)
{
	struct platform_device *remote_pdev;
	struct pci_pwrctrl_slot_data *slot;
	struct device *dev = &pdev->dev;
	struct device_node *remote, *endpoint;
	struct clk *clk;
	int ret;

#if 0
	/* FIXME: Assuming port 0 is PCIe interface */
	endpoint = of_graph_get_endpoint_by_regs(dev_of_node(dev), 0, -1);
	if (endpoint) {
		remote = of_graph_get_remote_port_parent(endpoint);
		if (remote) {
			remote_pdev = of_find_device_by_node(remote);
			if (!remote_pdev || !remote_pdev->dev.driver)
				return -EPROBE_DEFER;

			/* Now add devlink to the connector node */
			if (!device_link_add(dev, &remote_pdev->dev,
			       DL_FLAG_PM_RUNTIME | DL_FLAG_STATELESS))
				dev_err(dev, "Failed to link %s\n",
					dev_name(&remote_pdev->dev));
		}
	}
#endif

	slot = devm_kzalloc(dev, sizeof(*slot), GFP_KERNEL);
	if (!slot)
		return -ENOMEM;

	pci_pwrctrl_init(&slot->ctx, dev);

	if (of_graph_is_present(dev_of_node(dev))) {
		dev_info(dev, "#### %s: %d POWERING ON", __func__, __LINE__);

		slot->pwrseq = devm_pwrseq_get(dev, "wlan");
		if (IS_ERR(slot->pwrseq))
			return dev_err_probe(dev, PTR_ERR(slot->pwrseq),
				     "Failed to get the power sequencer\n");

		ret = pwrseq_power_on(slot->pwrseq);
		if (ret)
			return dev_err_probe(dev, ret,
				     "Failed to power-on the device\n");

		goto set_ready;
	}

	ret = of_regulator_bulk_get_all(dev, dev_of_node(dev),
					&slot->supplies);
	if (ret < 0) {
		dev_err_probe(dev, ret, "Failed to get slot regulators\n");
		return ret;
	}

	slot->num_supplies = ret;
	ret = regulator_bulk_enable(slot->num_supplies, slot->supplies);
	if (ret < 0) {
		dev_err_probe(dev, ret, "Failed to enable slot regulators\n");
		goto err_regulator_free;
	}

	/* FIXME */
	ret = devm_add_action_or_reset(dev, devm_pci_pwrctrl_slot_power_off,
				       slot);
	if (ret)
		goto err_regulator_disable;

	clk = devm_clk_get_optional_enabled(dev, NULL);
	if (IS_ERR(clk)) {
		return dev_err_probe(dev, PTR_ERR(clk),
				     "Failed to enable slot clock\n");
	}

	slot->w_dis1_gpio = devm_gpiod_get_optional(dev, "w-disable1", GPIOD_OUT_LOW);
	if (IS_ERR(slot->w_dis1_gpio))
		return dev_err_probe(dev, PTR_ERR(slot->w_dis1_gpio),
				     "Failed to get \"W_DISABLE1#\" GPIO\n");

	gpiod_set_value_cansleep(slot->w_dis1_gpio, 1);

set_ready:
	ret = devm_pci_pwrctrl_device_set_ready(dev, &slot->ctx);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to register pwrctrl driver\n");

	return 0;

err_regulator_disable:
	regulator_bulk_disable(slot->num_supplies, slot->supplies);
err_regulator_free:
	regulator_bulk_free(slot->num_supplies, slot->supplies);

	return ret;
}

static const struct of_device_id pci_pwrctrl_slot_of_match[] = {
	{
		.compatible = "pciclass,0604",
	},
	{ }
};
MODULE_DEVICE_TABLE(of, pci_pwrctrl_slot_of_match);

static struct platform_driver pci_pwrctrl_slot_driver = {
	.driver = {
		.name = "pci-pwrctrl-slot",
		.of_match_table = pci_pwrctrl_slot_of_match,
	},
	.probe = pci_pwrctrl_slot_probe,
};
module_platform_driver(pci_pwrctrl_slot_driver);

MODULE_AUTHOR("Manivannan Sadhasivam <manivannan.sadhasivam@linaro.org>");
MODULE_DESCRIPTION("Generic PCI Power Control driver for PCI Slots");
MODULE_LICENSE("GPL");
