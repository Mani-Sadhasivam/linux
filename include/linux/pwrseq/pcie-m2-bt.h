/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * Author: Manivannan Sadhasivam <manivannan.sadhasivam@oss.qualcomm.com>
 */

#ifndef __POWER_SEQUENCING_PCIE_M2_BT_H__
#define __POWER_SEQUENCING_PCIE_M2_BT_H__

#include <linux/auxiliary_bus.h>
#include <linux/container_of.h>

struct serdev_device;

/**
 * struct pcie_m2_bt_auxdev - Auxiliary device for the Bluetooth function of a
 *                            PCIe M.2 module.
 * @adev: Auxiliary device.
 * @serdev: Serdev device representing the UART transport. Allocated and owned
 *          by the producer (power sequencing driver).
 * @pwrseq_target: Power sequencing target the consumer requests to power up the
 *                 Bluetooth function.
 *
 * On M.2 modules exposing Bluetooth over UART, the controller is not described
 * in firmware. The power sequencing driver detects the module over PCIe and
 * publishes this auxiliary device so that the Bluetooth driver can bind to the
 * pre-allocated serdev transport and drive power sequencing.
 */
struct pcie_m2_bt_auxdev {
	struct auxiliary_device adev;
	struct serdev_device *serdev;
	const char *pwrseq_target;
};

#define to_pcie_m2_bt_auxdev(_adev) \
	container_of(_adev, struct pcie_m2_bt_auxdev, adev)

#endif /* __POWER_SEQUENCING_PCIE_M2_BT_H__ */
