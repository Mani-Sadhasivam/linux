.. SPDX-License-Identifier: GPL-2.0

===================================
Writing PCI Host Controller Drivers
===================================

:Author: Manivannan Sadhasivam <manivannan.sadhasivam@oss.qualcomm.com>

1. Introduction
===============

A PCI Host Controller driver is used to control a PCI Root Complex (RC) hardware
inside an SoC. The Root Complex hardware comprises of a single PCI Host Bridge
and one or more Root Port or Root Complex Integrated Endpoint (RCiEP) devices:
 
                     +------------------+
                     |       CPU        |
                     +------------------+
                              |
        +--------------------------------------------+
        |                     |               Root   |
        |            +------------------+   Complex  |
        |            |   Host Bridge    |            |
        |            +------------------+            |
        |                     |                      |
        |               Bus 0 |                      |
        |        +------------|----------+           |
        |        |            |          |           |
        |  +----------+ +----------+ +-------+       |
        |  |  Root    | |  Root    | | RCiEP |       |
        |  |  Port    | |  Port    | +-------+       |
        |  +----------+ +----------+                 |
        |       |             |                      |
        | Bus 1 |       Bus 2 |                      |
        |       |             |                      |
        +--------------------------------------------+

Host Bridge: Used to connect CPU(s) to the PCI hierarchy.
Root Port: Virtual PCI-PCI bridge to connect Host Bridge to one or more PCI bus.
RCiEP: Embedded PCIe endpoint inside Root Complex connected to the Host bridge.

2. Enumeration
==============

The Host Bridge device is not discoverable, so it is typically enumerated with
the help of the firmware interface like ACPI or Devicetree. But the Root Port
and RCiEP devices are discoverable through standard enumeration process defined
in the PCIe spec.

A Host Controller driver usually configures both Host Bridge and Root Port(s)
based on the platform requirement. In the case of ACPI on standardized platforms
(e.g. x86), no platform specific host controller driver is required as the
firmware configures the Root Complex before OS boot and exposes the resource
information ACPI tables. For more info, refer
https://docs.kernel.org/PCI/acpi-info.html # TODO FIX

But for the Devicetree platforms, a dedicated host controller driver is often
required because the Root Complex hardware typically needs vendor specific
initialization like PHY, clocks, power domains and there is no standard
mechanism equivalent to ACPI/MCFG to convey resource information to the OS. So
on these platforms, Root Complex hardware is enumerated through Devicetree
nodes as below:

        pcie@1c20000 {
            compatible = "qcom,pcie-sc8280xp";
            reg = <0x0 0x01c20000 0x0 0x3000>,
                  <0x0 0x3c000000 0x0 0xf1d>,
                  <0x0 0x3c000f20 0x0 0xa8>,
                  <0x0 0x3c001000 0x0 0x1000>,
                  <0x0 0x3c100000 0x0 0x100000>,
                  <0x0 0x01c23000 0x0 0x1000>;
            reg-names = "parf", "dbi", "elbi", "atu", "config", "mhi";
            ranges = <0x01000000 0x0 0x00000000 0x0 0x3c200000 0x0 0x100000>,
                     <0x02000000 0x0 0x3c300000 0x0 0x3c300000 0x0 0x1d00000>;

            bus-range = <0x00 0xff>;
            device_type = "pci";
            linux,pci-domain = <0>;
            num-lanes = <4>;

            #address-cells = <3>;
            #size-cells = <2>;

            clocks = <&gcc GCC_PCIE_2A_AUX_CLK>,
                     <&gcc GCC_PCIE_2A_CFG_AHB_CLK>,
                     <&gcc GCC_PCIE_2A_MSTR_AXI_CLK>,
                     <&gcc GCC_PCIE_2A_SLV_AXI_CLK>,
                     <&gcc GCC_PCIE_2A_SLV_Q2A_AXI_CLK>,
                     <&gcc GCC_DDRSS_PCIE_SF_TBU_CLK>,
                     <&gcc GCC_AGGRE_NOC_PCIE_4_AXI_CLK>,
                     <&gcc GCC_AGGRE_NOC_PCIE_SOUTH_SF_AXI_CLK>;
            interrupts = <GIC_SPI 86 IRQ_TYPE_LEVEL_HIGH>,
                         <GIC_SPI 523 IRQ_TYPE_LEVEL_HIGH>,
                         <GIC_SPI 524 IRQ_TYPE_LEVEL_HIGH>,
                         <GIC_SPI 525 IRQ_TYPE_LEVEL_HIGH>;
            interrupt-names = "msi0", "msi1", "msi2", "msi3";
            #interrupt-cells = <1>;
            interrupt-map-mask = <0 0 0 0x7>;
            interrupt-map = <0 0 0 1 &intc 0 0 GIC_SPI 530 IRQ_TYPE_LEVEL_HIGH>,
                            <0 0 0 2 &intc 0 0 GIC_SPI 531 IRQ_TYPE_LEVEL_HIGH>,
            ...

            pcieport0: pcie@0 {
                device_type = "pci";
                reg = <0x0 0x0 0x0 0x0 0x0>;
                bus-range = <0x01 0xff>;

                #address-cells = <3>;
                #size-cells = <2>;
                ranges;
                phys = <&pcie0_phy>;
                reset-gpios = <&tlmm 143 GPIO_ACTIVE_LOW>;
                wake-gpios = <&tlmm 145 GPIO_ACTIVE_LOW>;
            };
        };


Note the presence of two nodes in the above example. `pcie@1c20000` node
represents a PCI Host Bridge device and `pcie@0` represents a single Root Port
device. The Host Bridge node should contain the properties associated with the
Host Bridge device such as ranges, interrupts, clocks, power-domains etc... and
the Root Port node should contain the port specific properties such as phys,
reset-gpios, wake-gpios etc...

NOTE: Legacy Devicetrees used a single node to describe both Host Bridge and
Root Port devices. But that design is now deprecated.

3. Driver Design
================

3.1 Prerequisites
-----------------

Before starting to write a new Host Controller driver, check if any of the
existing drivers can be reused. For example, if the Root Complex supports
Enhanced Configuration Access Mechanism (ECAM) and the bootloader has configured
the ECAM mapping before OS boot, `CONFIG_PCI_HOST_GENERIC` driver can be used.

If the Root Complex hardware (IP) is from a known IP vendors such as Synopsys or
Cadence, then the existing CONFIG_PCIE_DW_PLAT and PCIE_CADENCE_PLAT_HOST
drivers can be reused. If not, then check if any of the existing glue drivers
available for these IPs could be reused.

Or if the Root Complex hardware is designed in-house by the SoC vendor, then
check if there is an existing driver from the vendor for their previous
generation Root Complex hardware. Often, the existing driver could be reused
with minimal modifications.

Only if the Root Complex doesn't satisfy above prerequisites, a new Host
Controller driver should be written.

3.2 probe()
-----------

3.2.1 Initialize Resources
--------------------------

At the start of the probe, initialize all the Host Bridge specific resources
such as Clocks, PHY, regulators, INT-X/MSI/MSI-X etc... Then setup address
translations for the Inbound, Outbound, DMA translation regions defined in the
Host Bridge Devicetree node. 

NOTE: If the hardware supports ECAM, it is strongly recommended to setup ECAM to
avoid doing individual addresss translations for each outbound access.

Power ON any Slots or endpoints connected to the bus with the help of PWRCTRL
subsystem APIs such as pci_pwrctrl_create_devices() and
pci_pwrctrl_power_on_devices().

Finally, allocate the Host Bridge device with devm_pci_alloc_host_bridge() and
start the bus scan by calling pci_host_probe(). pci_host_probe() will create the
Root bus for the Host Bridge and scan/enumerate all the Root Port, RCiEPs and
endpoint devices connected to the bus.

If the Root Complex IP is from a known IP vendor, the IP specific helpers should
be reused for above operations whereever applicable.

