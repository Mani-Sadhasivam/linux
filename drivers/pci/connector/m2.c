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
#include <linux/regulator/consumer.h>
#include <linux/serdev.h>
#include <linux/slab.h>

struct m2_con_data {
	struct regulator_bulk_data *supplies;
	int num_supplies;
	struct gpio_desc *w_dis1_gpio;
	struct gpio_desc *w_dis2_gpio;
	struct device *dev;
	struct notifier_block nb;
};

static int pci_connector_m2_notify(struct notifier_block *nb, unsigned long action,
			      void *data)
{
	struct m2_con_data *m2_data = container_of(nb, struct m2_con_data, nb);
	struct pci_dev *pdev = to_pci_dev(data);
	struct device_node *remote;
	struct serdev_controller *serdev_ctrl;
	struct serdev_device *serdev;
	struct device *dev = m2_data->dev;
	int ret;

	/* Check whether the PCI device is associated with this M.2 connector */
	remote = of_graph_get_remote_node(dev_of_node(m2_data->dev), 0, -1);
	if (!remote || (remote != pdev->dev.parent->of_node)) {
		of_node_put(remote);
		return NOTIFY_DONE;
	}
	of_node_put(remote);

	switch (action) {
	case BUS_NOTIFY_ADD_DEVICE:
		/* Create BT device for WCN7850 */
		if (pdev->vendor == PCI_VENDOR_ID_QCOM && pdev->device == 0x1107) {
			remote = of_graph_get_remote_node(dev_of_node(m2_data->dev), 1, -1);
			if (!remote) {
				of_node_put(remote);
				return NOTIFY_DONE;
			}

			serdev_ctrl = of_find_serdev_controller_by_node(remote);
			of_node_put(remote);
			if (!serdev_ctrl)
				return NOTIFY_DONE;

			serdev = serdev_device_alloc(serdev_ctrl);
			if (!serdev)
				return NOTIFY_DONE;

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

static int pci_connector_m2_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct m2_con_data *m2_data;
	int ret;

	m2_data = devm_kzalloc(dev, sizeof(*m2_data), GFP_KERNEL);
	if (!m2_data)
		return -ENOMEM;

	platform_set_drvdata(pdev, m2_data);

	ret = of_regulator_bulk_get_all(dev, dev_of_node(dev),
					&m2_data->supplies);
	if (ret < 0)
		return dev_err_probe(dev, ret, "Failed to get M.2 regulators\n");

	m2_data->num_supplies = ret;

	ret = regulator_bulk_enable(m2_data->num_supplies, m2_data->supplies);
	if (ret < 0) {
		dev_err_probe(dev, ret, "Failed to enable M.2 regulators\n");
		goto err_regulator_free;
	}

	m2_data->w_dis1_gpio = devm_gpiod_get_optional(dev, "w-disable1", GPIOD_OUT_LOW);
	if (IS_ERR(m2_data->w_dis1_gpio)) {
		ret = PTR_ERR(m2_data->w_dis1_gpio);
		dev_err_probe(dev, ret, "Failed to get \"W_DISABLE1#\" GPIO\n");
		goto err_regulator_disable;
	}

	m2_data->w_dis2_gpio = devm_gpiod_get_optional(dev, "w-disable2", GPIOD_OUT_LOW);
	if (IS_ERR(m2_data->w_dis2_gpio)) {
		ret = PTR_ERR(m2_data->w_dis2_gpio);
		dev_err_probe(dev, ret, "Failed to get \"W_DISABLE2#\" GPIO\n");
		goto err_regulator_disable;
	}

	/* TODO: Check */
	gpiod_set_value_cansleep(m2_data->w_dis1_gpio, 1);
	gpiod_set_value_cansleep(m2_data->w_dis2_gpio, 1);

	m2_data->dev = dev;
	m2_data->nb.notifier_call = pci_connector_m2_notify;
	ret = bus_register_notifier(&pci_bus_type, &m2_data->nb);
	if (ret) {
		dev_err_probe(dev, ret, "Failed to register notifier\n");
		goto err_gpio_disable;
	}

	return 0;

err_gpio_disable:
	gpiod_set_value_cansleep(m2_data->w_dis1_gpio, 0);
	gpiod_set_value_cansleep(m2_data->w_dis2_gpio, 0);
err_regulator_disable:
	regulator_bulk_disable(m2_data->num_supplies, m2_data->supplies);
err_regulator_free:
	regulator_bulk_free(m2_data->num_supplies, m2_data->supplies);

	return ret;
}

static void pci_connector_m2_remove(struct platform_device *pdev)
{
	struct m2_con_data *m2_data = platform_get_drvdata(pdev);

	gpiod_set_value_cansleep(m2_data->w_dis1_gpio, 0);
	gpiod_set_value_cansleep(m2_data->w_dis2_gpio, 0);
	regulator_bulk_disable(m2_data->num_supplies, m2_data->supplies);
	regulator_bulk_free(m2_data->num_supplies, m2_data->supplies);
}

static const struct of_device_id pci_connector_m2_of_match[] = {
	{
		.compatible = "pci-m2-connector",
	},
	{ }
};
MODULE_DEVICE_TABLE(of, pci_connector_m2_of_match);

static struct platform_driver pci_connector_m2_driver = {
	.driver = {
		.name = "pci-m2-connector",
		.of_match_table = pci_connector_m2_of_match,
	},
	.probe = pci_connector_m2_probe,
	.remove = pci_connector_m2_remove,
};
module_platform_driver(pci_connector_m2_driver);

MODULE_AUTHOR("Manivannan Sadhasivam <manivannan.sadhasivam@oss.qualcomm.com>");
MODULE_DESCRIPTION("Generic PCI Power Control driver for M.2 device");
MODULE_LICENSE("GPL");
