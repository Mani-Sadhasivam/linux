// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * Author: Manivannan Sadhasivam <manivannan.sadhasivam@oss.qualcomm.com>
 */

#include <linux/auxiliary_bus.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/idr.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/pwrseq/pcie-m2-bt.h>
#include <linux/pwrseq/provider.h>
#include <linux/regulator/consumer.h>
#include <linux/serdev.h>
#include <linux/slab.h>

struct pwrseq_pci_dev {
	struct serdev_device *serdev;
	struct pcie_m2_bt_auxdev *bt;
	struct pci_dev *pdev;
	struct list_head list;
};

struct pwrseq_pcie_m2_pdata {
	const struct pwrseq_target_data **targets;
};

struct pwrseq_pcie_m2_ctx {
	struct pwrseq_device *pwrseq;
	struct device_node *of_node;
	const struct pwrseq_pcie_m2_pdata *pdata;
	struct regulator_bulk_data *regs;
	size_t num_vregs;
	struct notifier_block nb;
	struct gpio_desc *w_disable1_gpio;
	struct gpio_desc *w_disable2_gpio;
	struct device *dev;
	struct list_head pci_devices;
	struct mutex list_lock;
};

static int pwrseq_pcie_m2_vregs_enable(struct pwrseq_device *pwrseq)
{
	struct pwrseq_pcie_m2_ctx *ctx = pwrseq_device_get_drvdata(pwrseq);

	return regulator_bulk_enable(ctx->num_vregs, ctx->regs);
}

static int pwrseq_pcie_m2_vregs_disable(struct pwrseq_device *pwrseq)
{
	struct pwrseq_pcie_m2_ctx *ctx = pwrseq_device_get_drvdata(pwrseq);

	return regulator_bulk_disable(ctx->num_vregs, ctx->regs);
}

static const struct pwrseq_unit_data pwrseq_pcie_m2_vregs_unit_data = {
	.name = "regulators-enable",
	.enable = pwrseq_pcie_m2_vregs_enable,
	.disable = pwrseq_pcie_m2_vregs_disable,
};

static const struct pwrseq_unit_data *pwrseq_pcie_m2_unit_deps[] = {
	&pwrseq_pcie_m2_vregs_unit_data,
	NULL
};

static int pwrseq_pci_m2_e_uart_enable(struct pwrseq_device *pwrseq)
{
	struct pwrseq_pcie_m2_ctx *ctx = pwrseq_device_get_drvdata(pwrseq);

	return gpiod_set_value_cansleep(ctx->w_disable2_gpio, 0);
}

static int pwrseq_pci_m2_e_uart_disable(struct pwrseq_device *pwrseq)
{
	struct pwrseq_pcie_m2_ctx *ctx = pwrseq_device_get_drvdata(pwrseq);

	return gpiod_set_value_cansleep(ctx->w_disable2_gpio, 1);
}

static bool pwrseq_pci_m2_e_uart_is_controllable(struct pwrseq_device *pwrseq)
{
	struct pwrseq_pcie_m2_ctx *ctx = pwrseq_device_get_drvdata(pwrseq);

	/*
	 * The UART enable is driven through the W_DISABLE2# line. When it is not
	 * wired up on this connector the enable/disable callbacks are no-ops, so
	 * the host cannot gate the Bluetooth function on its own.
	 */
	return !!ctx->w_disable2_gpio;
}

static const struct pwrseq_unit_data pwrseq_pcie_m2_e_uart_unit_data = {
	.name = "uart-enable",
	.deps = pwrseq_pcie_m2_unit_deps,
	.enable = pwrseq_pci_m2_e_uart_enable,
	.disable = pwrseq_pci_m2_e_uart_disable,
};

static int pwrseq_pci_m2_e_pcie_enable(struct pwrseq_device *pwrseq)
{
	struct pwrseq_pcie_m2_ctx *ctx = pwrseq_device_get_drvdata(pwrseq);

	return gpiod_set_value_cansleep(ctx->w_disable1_gpio, 0);
}

static int pwrseq_pci_m2_e_pcie_disable(struct pwrseq_device *pwrseq)
{
	struct pwrseq_pcie_m2_ctx *ctx = pwrseq_device_get_drvdata(pwrseq);

	return gpiod_set_value_cansleep(ctx->w_disable1_gpio, 1);
}

static bool pwrseq_pci_m2_e_pcie_is_controllable(struct pwrseq_device *pwrseq)
{
	struct pwrseq_pcie_m2_ctx *ctx = pwrseq_device_get_drvdata(pwrseq);

	/*
	 * The PCIe/WiFi enable is driven through the W_DISABLE1# line. When it
	 * is not wired up on this connector the enable/disable callbacks are
	 * no-ops, so the host cannot gate the PCIe/WiFi function on its own.
	 */
	return !!ctx->w_disable1_gpio;
}

static const struct pwrseq_unit_data pwrseq_pcie_m2_e_pcie_unit_data = {
	.name = "pcie-enable",
	.deps = pwrseq_pcie_m2_unit_deps,
	.enable = pwrseq_pci_m2_e_pcie_enable,
	.disable = pwrseq_pci_m2_e_pcie_disable,
};

static const struct pwrseq_unit_data pwrseq_pcie_m2_m_pcie_unit_data = {
	.name = "pcie-enable",
	.deps = pwrseq_pcie_m2_unit_deps,
};

static int pwrseq_pcie_m2_e_pwup_delay(struct pwrseq_device *pwrseq)
{
	/*
	 * FIXME: This delay is only required for some Qcom WLAN/BT cards like
	 * WCN7850 and not for all devices. But currently, there is no way to
	 * identify the device model before enumeration.
	 */
	msleep(50);

	return 0;
}

static const struct pwrseq_target_data pwrseq_pcie_m2_e_uart_target_data = {
	.name = "uart",
	.unit = &pwrseq_pcie_m2_e_uart_unit_data,
	.post_enable = pwrseq_pcie_m2_e_pwup_delay,
	.is_controllable = pwrseq_pci_m2_e_uart_is_controllable,
};

static const struct pwrseq_target_data pwrseq_pcie_m2_e_pcie_target_data = {
	.name = "pcie",
	.unit = &pwrseq_pcie_m2_e_pcie_unit_data,
	.post_enable = pwrseq_pcie_m2_e_pwup_delay,
	.is_controllable = pwrseq_pci_m2_e_pcie_is_controllable,
};

static const struct pwrseq_target_data pwrseq_pcie_m2_m_pcie_target_data = {
	.name = "pcie",
	.unit = &pwrseq_pcie_m2_m_pcie_unit_data,
};

static const struct pwrseq_target_data *pwrseq_pcie_m2_e_targets[] = {
	&pwrseq_pcie_m2_e_pcie_target_data,
	&pwrseq_pcie_m2_e_uart_target_data,
	NULL
};

static const struct pwrseq_target_data *pwrseq_pcie_m2_m_targets[] = {
	&pwrseq_pcie_m2_m_pcie_target_data,
	NULL
};

static const struct pwrseq_pcie_m2_pdata pwrseq_pcie_m2_e_of_data = {
	.targets = pwrseq_pcie_m2_e_targets,
};

static const struct pwrseq_pcie_m2_pdata pwrseq_pcie_m2_m_of_data = {
	.targets = pwrseq_pcie_m2_m_targets,
};

static int pwrseq_pcie_m2_match(struct pwrseq_device *pwrseq,
				 struct device *dev)
{
	struct pwrseq_pcie_m2_ctx *ctx = pwrseq_device_get_drvdata(pwrseq);
	struct device_node *endpoint __free(device_node) = NULL;

	/*
	 * The Bluetooth function is represented by an auxiliary device created
	 * by this driver. It has no OF node, so match it by verifying that it
	 * is a child of this connector.
	 */
	if (dev_is_auxiliary(dev) && dev->parent == ctx->dev)
		return PWRSEQ_MATCH_OK;

	/*
	 * Traverse the 'remote-endpoint' nodes and check if the remote node's
	 * parent matches the OF node of 'dev'.
	 */
	for_each_endpoint_of_node(ctx->of_node, endpoint) {
		struct device_node *remote __free(device_node) =
				of_graph_get_remote_port_parent(endpoint);
		if (remote && (remote == dev_of_node(dev)))
			return PWRSEQ_MATCH_OK;
	}

	return PWRSEQ_NO_MATCH;
}

/*
 * The Bluetooth interface of a supported module is exposed as an auxiliary
 * device. Its name (combined with this driver's module name) is matched by
 * the Bluetooth driver. Store that name as the PCI ID driver data.
 */
static const struct pci_device_id pwrseq_m2_pci_ids[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_MARVELL_EXT, 0x2b43),
	  .driver_data = (kernel_ulong_t)"88w8987-bt" },
	{ PCI_DEVICE(PCI_VENDOR_ID_PHILIPS, 0x3003),
	  .driver_data = (kernel_ulong_t)"88w8987-bt" },
	{ PCI_DEVICE_SUB(PCI_VENDOR_ID_QCOM, 0x1103, PCI_VENDOR_ID_QCOM, 0x0108),
	  .driver_data = (kernel_ulong_t)"qca2066-bt" },
	{ PCI_DEVICE_SUB(PCI_VENDOR_ID_QCOM, 0x1103, PCI_VENDOR_ID_FOXCONN, 0xe105),
	  .driver_data = (kernel_ulong_t)"wcn6855-bt" },
	{ PCI_DEVICE_SUB(PCI_VENDOR_ID_QCOM, 0x1103, PCI_VENDOR_ID_QCOM, 0x337e),
	  .driver_data = (kernel_ulong_t)"wcn6855-bt" },
	{ PCI_DEVICE_SUB(PCI_VENDOR_ID_QCOM, 0x1107, PCI_VENDOR_ID_QCOM, 0x337c),
	  .driver_data = (kernel_ulong_t)"wcn7850-bt" },
	{ } /* Sentinel */
};

static DEFINE_IDA(pwrseq_pcie_m2_bt_ida);

static void pwrseq_pcie_m2_bt_release(struct device *dev)
{
	struct pcie_m2_bt_auxdev *bt =
		to_pcie_m2_bt_auxdev(to_auxiliary_dev(dev));

	ida_free(&pwrseq_pcie_m2_bt_ida, bt->adev.id);
	kfree(bt);
}

static int pwrseq_pcie_m2_create_bt_aux(struct pwrseq_pcie_m2_ctx *ctx,
					struct pwrseq_pci_dev *pci_dev,
					struct pci_dev *pdev)
{
	const struct pci_device_id *id;
	struct device *dev = ctx->dev;
	struct pcie_m2_bt_auxdev *bt;
	int ret;

	id = pci_match_id(pwrseq_m2_pci_ids, pdev);
	if (WARN_ON_ONCE(!id)) /* Shouldn't happen */
		return -ENODEV;

	bt = kzalloc_obj(*bt);
	if (!bt)
		return -ENOMEM;

	ret = ida_alloc(&pwrseq_pcie_m2_bt_ida, GFP_KERNEL);
	if (ret < 0)
		goto err_free_bt;

	bt->serdev = pci_dev->serdev;
	bt->pwrseq_target = "uart";
	bt->adev.id = ret;
	bt->adev.name = (const char *)id->driver_data;
	bt->adev.dev.parent = dev;
	bt->adev.dev.release = pwrseq_pcie_m2_bt_release;

	ret = auxiliary_device_init(&bt->adev);
	if (ret)
		goto err_free_ida;

	ret = auxiliary_device_add(&bt->adev);
	if (ret) {
		dev_err(dev, "Failed to add bluetooth auxiliary device: %d\n",
			ret);
		auxiliary_device_uninit(&bt->adev);
		return ret;
	}

	pci_dev->bt = bt;

	return 0;

err_free_ida:
	ida_free(&pwrseq_pcie_m2_bt_ida, bt->adev.id);
err_free_bt:
	kfree(bt);

	return ret;
}

static int pwrseq_pcie_m2_create_serdev_one(struct pwrseq_pcie_m2_ctx *ctx,
					struct pci_dev *pdev)
{
	struct serdev_controller *serdev_ctrl;
	struct device *dev = ctx->dev;
	struct pwrseq_pci_dev *pci_dev;
	int ret;

	struct device_node *serdev_parent __free(device_node) =
		of_graph_get_remote_node(dev_of_node(ctx->dev), 3, 0);
	if (!serdev_parent)
		return 0;

	serdev_ctrl = of_find_serdev_controller_by_node(serdev_parent);
	if (!serdev_ctrl)
		return 0;

	/* Bail out if the device was already attached to this controller */
	if (serdev_ctrl->serdev) {
		serdev_controller_put(serdev_ctrl);
		return 0;
	}

	/* Bail out if the serdev device was already created for the PCI dev */
	scoped_guard(mutex, &ctx->list_lock) {
		list_for_each_entry(pci_dev, &ctx->pci_devices, list) {
			if (pci_dev->pdev == pdev)
				return 0;
		}
	}

	pci_dev = kzalloc(sizeof(*pci_dev), GFP_KERNEL);
	if (!pci_dev) {
		ret = -ENOMEM;
		goto err_put_ctrl;
	}

	pci_dev->serdev = serdev_device_alloc(serdev_ctrl);
	if (!pci_dev->serdev) {
		ret = -ENOMEM;
		goto err_free_pci_dev;
	}

	ret = serdev_device_add(pci_dev->serdev);
	if (ret) {
		dev_err(dev, "Failed to add serdev for PCI device (%s): %d\n",
			pci_name(pdev), ret);
		goto err_free_serdev;
	}

	ret = pwrseq_pcie_m2_create_bt_aux(ctx, pci_dev, pdev);
	if (ret)
		goto err_remove_serdev;

	serdev_controller_put(serdev_ctrl);

	pci_dev->pdev = pci_dev_get(pdev);

	mutex_lock(&ctx->list_lock);
	list_add_tail(&pci_dev->list, &ctx->pci_devices);
	mutex_unlock(&ctx->list_lock);

	return 0;

err_remove_serdev:
	serdev_device_remove(pci_dev->serdev);
	goto err_free_pci_dev;
err_free_serdev:
	serdev_device_put(pci_dev->serdev);
err_free_pci_dev:
	kfree(pci_dev);
err_put_ctrl:
	serdev_controller_put(serdev_ctrl);

	return ret;
}

static void __pwrseq_pcie_m2_remove_serdev(struct pwrseq_pcie_m2_ctx *ctx,
					   struct pwrseq_pci_dev *pci_dev)
{
	if (pci_dev->bt) {
		auxiliary_device_delete(&pci_dev->bt->adev);
		auxiliary_device_uninit(&pci_dev->bt->adev);
	}

	if (pci_dev->serdev)
		serdev_device_remove(pci_dev->serdev);

	pci_dev_put(pci_dev->pdev);
	list_del(&pci_dev->list);
	kfree(pci_dev);
}

static void pwrseq_pcie_m2_remove_serdev(struct pwrseq_pcie_m2_ctx *ctx,
					 struct pci_dev *pdev)
{
	struct pwrseq_pci_dev *pci_dev, *tmp;

	mutex_lock(&ctx->list_lock);
	list_for_each_entry_safe(pci_dev, tmp, &ctx->pci_devices, list) {
		if (!pdev || pci_dev->pdev == pdev) {
			__pwrseq_pcie_m2_remove_serdev(ctx, pci_dev);
			if (pdev)
				break;
		}
	}
	mutex_unlock(&ctx->list_lock);
}

static int pwrseq_pcie_m2_notify(struct notifier_block *nb, unsigned long action,
			      void *data)
{
	struct pwrseq_pcie_m2_ctx *ctx = container_of(nb, struct pwrseq_pcie_m2_ctx, nb);
	struct pci_dev *pdev = to_pci_dev(data);
	int ret;

	/*
	 * Check whether the PCI device is associated with this M.2 connector or
	 * not, by comparing the OF node of the PCI device parent and the Port 0
	 * (PCIe) remote node parent OF node.
	 */
	struct device_node *pci_parent __free(device_node) =
			of_graph_get_remote_node(dev_of_node(ctx->dev), 0, 0);
	if (!pci_parent || (pci_parent != pdev->dev.parent->of_node))
		return NOTIFY_DONE;

	switch (action) {
	case BUS_NOTIFY_ADD_DEVICE:
		if (pci_match_id(pwrseq_m2_pci_ids, pdev)) {
			ret = pwrseq_pcie_m2_create_serdev_one(ctx, pdev);
			if (ret)
				return notifier_from_errno(ret);
		}
		break;
	case BUS_NOTIFY_REMOVED_DEVICE:
		if (pci_match_id(pwrseq_m2_pci_ids, pdev))
			pwrseq_pcie_m2_remove_serdev(ctx, pdev);

		break;
	}

	return NOTIFY_OK;
}

static bool pwrseq_pcie_m2_check_remote_node(struct device *dev, u8 port, u8 endpoint,
					     const char *node)
{
	struct device_node *remote __free(device_node) =
			of_graph_get_remote_node(dev_of_node(dev), port, endpoint);

	if (remote && of_node_name_eq(remote, node))
		return true;

	return false;
}

/*
 * If the connector exposes a non-discoverable bus like UART, the respective
 * protocol device needs to be created manually with the help of the notifier
 * of the discoverable bus like PCIe.
 */
static int pwrseq_pcie_m2_register_notifier(struct pwrseq_pcie_m2_ctx *ctx)
{
	int ret;

	/*
	 * Register a PCI notifier for Key E connector that has PCIe as Port
	 * 0/Endpoint 0 interface and Serial as Port 3/Endpoint 0 interface.
	 */
	if (!pwrseq_pcie_m2_check_remote_node(ctx->dev, 3, 0, "serial") ||
	    !pwrseq_pcie_m2_check_remote_node(ctx->dev, 0, 0, "pcie"))
		return 0;

	ctx->nb.notifier_call = pwrseq_pcie_m2_notify;
	ret = bus_register_notifier(&pci_bus_type, &ctx->nb);
	if (ret)
		return dev_err_probe(ctx->dev, ret,
				     "Failed to register notifier for serdev\n");
	return 0;
}

static int pwrseq_pcie_m2_create_serdev(struct pwrseq_pcie_m2_ctx *ctx)
{
	struct pci_dev *pdev = NULL;
	int ret;

	if (!pwrseq_pcie_m2_check_remote_node(ctx->dev, 3, 0, "serial") ||
	    !pwrseq_pcie_m2_check_remote_node(ctx->dev, 0, 0, "pcie"))
		return 0;

	struct device_node *pci_parent __free(device_node) =
				of_graph_get_remote_node(dev_of_node(ctx->dev), 0, 0);
	if (!pci_parent)
		return 0;

	/* Create serdev for existing PCI devices if required */
	for_each_pci_dev(pdev) {
		if (!pdev->dev.parent || pci_parent != pdev->dev.parent->of_node)
			continue;

		if (!pci_match_id(pwrseq_m2_pci_ids, pdev))
			continue;

		ret = pwrseq_pcie_m2_create_serdev_one(ctx, pdev);
		if (ret) {
			dev_err_probe(ctx->dev, ret,
				      "Failed to create serdev for PCI device (%s)\n",
				      pci_name(pdev));
			pci_dev_put(pdev);
			goto err_remove_serdev;
		}
	}

	return 0;

err_remove_serdev:
	pwrseq_pcie_m2_remove_serdev(ctx, NULL);

	return ret;
}

static int pwrseq_pcie_m2_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct pwrseq_pcie_m2_ctx *ctx;
	struct pwrseq_config config = {};
	int ret;

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	platform_set_drvdata(pdev, ctx);
	ctx->dev = dev;
	ctx->of_node = dev_of_node(dev);
	ctx->pdata = device_get_match_data(dev);
	if (!ctx->pdata)
		return dev_err_probe(dev, -ENODEV,
				     "Failed to obtain platform data\n");

	/*
	 * Currently, of_regulator_bulk_get_all() is the only regulator API that
	 * allows to get all supplies in the devicetree node without manually
	 * specifying them.
	 */
	ret = of_regulator_bulk_get_all(dev, dev_of_node(dev), &ctx->regs);
	if (ret < 0)
		return dev_err_probe(dev, ret,
				     "Failed to get all regulators\n");

	ctx->num_vregs = ret;

	ctx->w_disable1_gpio = devm_gpiod_get_optional(dev, "w-disable1", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->w_disable1_gpio)) {
		ret = dev_err_probe(dev, PTR_ERR(ctx->w_disable1_gpio),
				     "Failed to get the W_DISABLE_1# GPIO\n");
		goto err_free_regulators;
	}

	ctx->w_disable2_gpio = devm_gpiod_get_optional(dev, "w-disable2", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->w_disable2_gpio)) {
		ret = dev_err_probe(dev, PTR_ERR(ctx->w_disable2_gpio),
				     "Failed to get the W_DISABLE_2# GPIO\n");
		goto err_free_regulators;
	}

	config.parent = dev;
	config.owner = THIS_MODULE;
	config.drvdata = ctx;
	config.match = pwrseq_pcie_m2_match;
	config.targets = ctx->pdata->targets;

	ctx->pwrseq = devm_pwrseq_device_register(dev, &config);
	if (IS_ERR(ctx->pwrseq)) {
		ret = dev_err_probe(dev, PTR_ERR(ctx->pwrseq),
				     "Failed to register the power sequencer\n");
		goto err_free_regulators;
	}

	mutex_init(&ctx->list_lock);
	INIT_LIST_HEAD(&ctx->pci_devices);

	/* Create serdev for available PCI devices (if required) */
	ret = pwrseq_pcie_m2_create_serdev(ctx);
	if (ret)
		goto err_destroy_mutex;

	/*
	 * Register a notifier for creating protocol devices for
	 * non-discoverable busses like UART.
	 */
	ret = pwrseq_pcie_m2_register_notifier(ctx);
	if (ret)
		goto err_remove_serdev;

	return 0;

err_remove_serdev:
	pwrseq_pcie_m2_remove_serdev(ctx, NULL);
err_destroy_mutex:
	mutex_destroy(&ctx->list_lock);
err_free_regulators:
	regulator_bulk_free(ctx->num_vregs, ctx->regs);

	return ret;
}

static void pwrseq_pcie_m2_remove(struct platform_device *pdev)
{
	struct pwrseq_pcie_m2_ctx *ctx = platform_get_drvdata(pdev);

	bus_unregister_notifier(&pci_bus_type, &ctx->nb);
	pwrseq_pcie_m2_remove_serdev(ctx, NULL);
	mutex_destroy(&ctx->list_lock);

	regulator_bulk_free(ctx->num_vregs, ctx->regs);
}

static const struct of_device_id pwrseq_pcie_m2_of_match[] = {
	{
		.compatible = "pcie-m2-m-connector",
		.data = &pwrseq_pcie_m2_m_of_data,
	},
	{
		.compatible = "pcie-m2-e-connector",
		.data = &pwrseq_pcie_m2_e_of_data,
	},
	{ }
};
MODULE_DEVICE_TABLE(of, pwrseq_pcie_m2_of_match);

static struct platform_driver pwrseq_pcie_m2_driver = {
	.driver = {
		.name = "pwrseq-pcie-m2",
		.of_match_table = pwrseq_pcie_m2_of_match,
	},
	.probe = pwrseq_pcie_m2_probe,
	.remove = pwrseq_pcie_m2_remove,
};
module_platform_driver(pwrseq_pcie_m2_driver);

MODULE_AUTHOR("Manivannan Sadhasivam <manivannan.sadhasivam@oss.qualcomm.com>");
MODULE_DESCRIPTION("Power Sequencing driver for PCIe M.2 connector");
MODULE_LICENSE("GPL");
