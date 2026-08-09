// SPDX-License-Identifier: GPL-2.0
/*
 * Generic PCI Endpoint Controller driver for firmware-initialised controllers.
 *
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * Author: Manivannan Sadhasivam <manivannan.sadhasivam@oss.qualcomm.com>
 */

#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pci-epc.h>
#include <linux/pci-epf.h>
#include <linux/platform_device.h>
#include <linux/dma-map-ops.h>

/*
 * Register map — relative to the "ctrl" MMIO region.
 */

/* Read-only capability registers */
#define PCI_EPC_GEN_CAPS			0x00
#define   PCI_EPC_GEN_CAP_INTX			BIT(0)
#define   PCI_EPC_GEN_CAP_MSI			BIT(1)
#define   PCI_EPC_GEN_CAP_MSIX			BIT(2)
#define   PCI_EPC_GEN_CAP_LINKUP_NOTIFY		BIT(3)
#define   PCI_EPC_GEN_CAP_DYN_INBOUND_MAP	BIT(4)
#define   PCI_EPC_GEN_CAP_SUBRANGE		BIT(5)
#define PCI_EPC_GEN_MAX_FUNC			0x04

/* Link state (read-only status, driver polls after issuing CMD_START/STOP) */
#define PCI_EPC_GEN_LINKUP			0x10

#define PCI_EPC_GEN_FUNC_NO			0x18
#define PCI_EPC_GEN_BAR_NO			0x1c
#define PCI_EPC_GEN_IRQ_TYPE			0x20
#define PCI_EPC_GEN_IRQ_NUM			0x24

/* PCI configuration space header fields */
#define PCI_EPC_GEN_HDR_VID			0x30
#define PCI_EPC_GEN_HDR_DID			0x34
#define PCI_EPC_GEN_HDR_SVID			0x38
#define PCI_EPC_GEN_HDR_SSID			0x3c
#define PCI_EPC_GEN_HDR_CLASS			0x40
#define PCI_EPC_GEN_HDR_REV			0x44
#define PCI_EPC_GEN_HDR_INTPIN			0x48

/* BAR configuration */
#define PCI_EPC_GEN_BAR_PHYS_LO			0x50
#define PCI_EPC_GEN_BAR_PHYS_HI			0x54
#define PCI_EPC_GEN_BAR_SIZE_LO			0x58
#define PCI_EPC_GEN_BAR_SIZE_HI			0x5c
#define PCI_EPC_GEN_BAR_FLAGS			0x60

/* Outbound address mapping */
#define PCI_EPC_GEN_MAP_PHYS_LO			0x70
#define PCI_EPC_GEN_MAP_PHYS_HI			0x74
#define PCI_EPC_GEN_MAP_PCI_LO			0x78
#define PCI_EPC_GEN_MAP_PCI_HI			0x7c
#define PCI_EPC_GEN_MAP_SIZE_LO			0x80
#define PCI_EPC_GEN_MAP_SIZE_HI			0x84

/* MSI/MSI-X configuration */
#define PCI_EPC_GEN_MSI_COUNT			0x90
#define PCI_EPC_GEN_MSIX_BAR			0x94
#define PCI_EPC_GEN_MSIX_OFFSET			0x98

/* RC-programmed MSI/MSI-X readback registers — read-only from driver */
#define PCI_EPC_GEN_MSI_READBACK		0xa0
#define PCI_EPC_GEN_MSIX_READBACK		0xa4
#define PCI_EPC_GEN_MSI_ADDR_LO			0xb0
#define PCI_EPC_GEN_MSI_ADDR_HI			0xb4
#define PCI_EPC_GEN_MSI_DATA			0xb8

#define PCI_EPC_GEN_CMD				0xf0
#define   PCI_EPC_GEN_CMD_WRITE_HEADER	0x01
#define   PCI_EPC_GEN_CMD_SET_BAR		0x02
#define   PCI_EPC_GEN_CMD_CLEAR_BAR		0x03
#define   PCI_EPC_GEN_CMD_MAP			0x04
#define   PCI_EPC_GEN_CMD_UNMAP		0x05
#define   PCI_EPC_GEN_CMD_SET_MSI		0x06
#define   PCI_EPC_GEN_CMD_SET_MSIX		0x07
#define   PCI_EPC_GEN_CMD_RAISE_IRQ		0x08
#define   PCI_EPC_GEN_CMD_START		0x09
#define   PCI_EPC_GEN_CMD_STOP		0x0a

struct pci_ep_generic {
	void __iomem *base;
	struct pci_epc_features features;
};

static int pci_ep_generic_write_header(struct pci_epc *epc, u8 fn, u8 vfn,
				 struct pci_epf_header *hdr)
{
	struct pci_ep_generic *epc_priv = epc_get_drvdata(epc);

	writel_relaxed(fn, epc_priv->base + PCI_EPC_GEN_FUNC_NO);
	writel_relaxed(hdr->vendorid, epc_priv->base + PCI_EPC_GEN_HDR_VID);
	writel_relaxed(hdr->deviceid, epc_priv->base + PCI_EPC_GEN_HDR_DID);
	writel_relaxed(hdr->subsys_vendor_id, epc_priv->base + PCI_EPC_GEN_HDR_SVID);
	writel_relaxed(hdr->subsys_id, epc_priv->base + PCI_EPC_GEN_HDR_SSID);
	writel_relaxed((hdr->baseclass_code << 16) |
	       (hdr->subclass_code << 8) |
	       hdr->progif_code, epc_priv->base + PCI_EPC_GEN_HDR_CLASS);
	writel_relaxed(hdr->revid, epc_priv->base + PCI_EPC_GEN_HDR_REV);
	writel_relaxed(hdr->interrupt_pin, epc_priv->base + PCI_EPC_GEN_HDR_INTPIN);
	writel_relaxed(PCI_EPC_GEN_CMD_WRITE_HEADER, epc_priv->base + PCI_EPC_GEN_CMD);

	return 0;
}

static int pci_ep_generic_set_bar(struct pci_epc *epc, u8 fn, u8 vfn,
			    struct pci_epf_bar *epf_bar)
{
	struct pci_ep_generic *epc_priv = epc_get_drvdata(epc);

	writel_relaxed(fn, epc_priv->base + PCI_EPC_GEN_FUNC_NO);
	writel_relaxed(epf_bar->barno, epc_priv->base + PCI_EPC_GEN_BAR_NO);
	writel_relaxed(lower_32_bits(epf_bar->phys_addr), epc_priv->base + PCI_EPC_GEN_BAR_PHYS_LO);
	writel_relaxed(upper_32_bits(epf_bar->phys_addr), epc_priv->base + PCI_EPC_GEN_BAR_PHYS_HI);
	writel_relaxed(lower_32_bits(epf_bar->size), epc_priv->base + PCI_EPC_GEN_BAR_SIZE_LO);
	writel_relaxed(upper_32_bits(epf_bar->size), epc_priv->base + PCI_EPC_GEN_BAR_SIZE_HI);
	writel_relaxed(epf_bar->flags, epc_priv->base + PCI_EPC_GEN_BAR_FLAGS);
	writel_relaxed(PCI_EPC_GEN_CMD_SET_BAR, epc_priv->base + PCI_EPC_GEN_CMD);

	return 0;
}

static void pci_ep_generic_clear_bar(struct pci_epc *epc, u8 fn, u8 vfn,
			       struct pci_epf_bar *epf_bar)
{
	struct pci_ep_generic *epc_priv = epc_get_drvdata(epc);

	writel_relaxed(fn, epc_priv->base + PCI_EPC_GEN_FUNC_NO);
	writel_relaxed(epf_bar->barno, epc_priv->base + PCI_EPC_GEN_BAR_NO);
	writel_relaxed(PCI_EPC_GEN_CMD_CLEAR_BAR, epc_priv->base + PCI_EPC_GEN_CMD);
}

static int pci_ep_generic_map_addr(struct pci_epc *epc, u8 fn, u8 vfn,
			     phys_addr_t phys_addr, u64 pci_addr, size_t size)
{
	struct pci_ep_generic *epc_priv = epc_get_drvdata(epc);

	writel_relaxed(fn, epc_priv->base + PCI_EPC_GEN_FUNC_NO);
	writel_relaxed(lower_32_bits(phys_addr), epc_priv->base + PCI_EPC_GEN_MAP_PHYS_LO);
	writel_relaxed(upper_32_bits(phys_addr), epc_priv->base + PCI_EPC_GEN_MAP_PHYS_HI);
	writel_relaxed(lower_32_bits(pci_addr), epc_priv->base + PCI_EPC_GEN_MAP_PCI_LO);
	writel_relaxed(upper_32_bits(pci_addr), epc_priv->base + PCI_EPC_GEN_MAP_PCI_HI);
	writel_relaxed(lower_32_bits((u64)size), epc_priv->base + PCI_EPC_GEN_MAP_SIZE_LO);
	writel_relaxed(upper_32_bits((u64)size), epc_priv->base + PCI_EPC_GEN_MAP_SIZE_HI);
	writel_relaxed(PCI_EPC_GEN_CMD_MAP, epc_priv->base + PCI_EPC_GEN_CMD);

	return 0;
}

static void pci_ep_generic_unmap_addr(struct pci_epc *epc, u8 fn, u8 vfn,
				phys_addr_t phys_addr)
{
	struct pci_ep_generic *epc_priv = epc_get_drvdata(epc);

	writel_relaxed(fn, epc_priv->base + PCI_EPC_GEN_FUNC_NO);
	writel_relaxed(lower_32_bits(phys_addr), epc_priv->base + PCI_EPC_GEN_MAP_PHYS_LO);
	writel_relaxed(upper_32_bits(phys_addr), epc_priv->base + PCI_EPC_GEN_MAP_PHYS_HI);
	writel_relaxed(PCI_EPC_GEN_CMD_UNMAP, epc_priv->base + PCI_EPC_GEN_CMD);
}

static int pci_ep_generic_set_msi(struct pci_epc *epc, u8 fn, u8 vfn, u8 nr_irqs)
{
	struct pci_ep_generic *epc_priv = epc_get_drvdata(epc);

	writel_relaxed(fn, epc_priv->base + PCI_EPC_GEN_FUNC_NO);
	writel_relaxed(nr_irqs, epc_priv->base + PCI_EPC_GEN_MSI_COUNT);
	writel_relaxed(PCI_EPC_GEN_CMD_SET_MSI, epc_priv->base + PCI_EPC_GEN_CMD);

	return 0;
}

static int pci_ep_generic_get_msi(struct pci_epc *epc, u8 fn, u8 vfn)
{
	struct pci_ep_generic *epc_priv = epc_get_drvdata(epc);
	u32 count;

	count = readl_relaxed(epc_priv->base + PCI_EPC_GEN_MSI_READBACK);
	return count ? (int)count : -EINVAL;
}

static int pci_ep_generic_set_msix(struct pci_epc *epc, u8 fn, u8 vfn,
			     u16 nr_irqs, enum pci_barno bar, u32 offset)
{
	struct pci_ep_generic *epc_priv = epc_get_drvdata(epc);

	writel_relaxed(fn, epc_priv->base + PCI_EPC_GEN_FUNC_NO);
	writel_relaxed(nr_irqs, epc_priv->base + PCI_EPC_GEN_MSI_COUNT);
	writel_relaxed(bar, epc_priv->base + PCI_EPC_GEN_MSIX_BAR);
	writel_relaxed(offset, epc_priv->base + PCI_EPC_GEN_MSIX_OFFSET);
	writel_relaxed(PCI_EPC_GEN_CMD_SET_MSIX, epc_priv->base + PCI_EPC_GEN_CMD);

	return 0;
}

static int pci_ep_generic_get_msix(struct pci_epc *epc, u8 fn, u8 vfn)
{
	struct pci_ep_generic *epc_priv = epc_get_drvdata(epc);
	u32 count;

	count = readl_relaxed(epc_priv->base + PCI_EPC_GEN_MSIX_READBACK);
	return count ? (int)count : -EINVAL;
}

static int pci_ep_generic_raise_irq(struct pci_epc *epc, u8 fn, u8 vfn,
			      unsigned int type, u16 irq_num)
{
	struct pci_ep_generic *epc_priv = epc_get_drvdata(epc);

	writel_relaxed(fn, epc_priv->base + PCI_EPC_GEN_FUNC_NO);
	writel_relaxed(type, epc_priv->base + PCI_EPC_GEN_IRQ_TYPE);
	writel_relaxed(irq_num, epc_priv->base + PCI_EPC_GEN_IRQ_NUM);
	writel_relaxed(PCI_EPC_GEN_CMD_RAISE_IRQ, epc_priv->base + PCI_EPC_GEN_CMD);

	return 0;
}

static int pci_ep_generic_map_msi_irq(struct pci_epc *epc, u8 fn, u8 vfn,
				phys_addr_t phys_addr, u8 irq_num,
				u32 entry_size, u32 *msi_data,
				u32 *msi_addr_offset)
{
	struct pci_ep_generic *epc_priv = epc_get_drvdata(epc);
	u64 msi_addr, aligned;

	msi_addr = ((u64)readl_relaxed(epc_priv->base + PCI_EPC_GEN_MSI_ADDR_HI) << 32) |
		   readl_relaxed(epc_priv->base + PCI_EPC_GEN_MSI_ADDR_LO);

	aligned = ALIGN_DOWN(msi_addr, entry_size);
	*msi_addr_offset = (msi_addr - aligned) + (u32)irq_num * entry_size;
	*msi_data = readl_relaxed(epc_priv->base + PCI_EPC_GEN_MSI_DATA) + irq_num;

	return pci_ep_generic_map_addr(epc, fn, vfn, phys_addr, aligned, entry_size);
}

static int pci_ep_generic_start(struct pci_epc *epc)
{
	struct pci_ep_generic *epc_priv = epc_get_drvdata(epc);
	u32 linkup;
	int ret;

	writel_relaxed(PCI_EPC_GEN_CMD_START, epc_priv->base + PCI_EPC_GEN_CMD);

	ret = readl_relaxed_poll_timeout(epc_priv->base + PCI_EPC_GEN_LINKUP,
					 linkup, linkup, 1000, 1000000);
	if (ret)
		return ret;

	pci_epc_linkup(epc);

	return 0;
}

static void pci_ep_generic_stop(struct pci_epc *epc)
{
	struct pci_ep_generic *epc_priv = epc_get_drvdata(epc);

	writel_relaxed(PCI_EPC_GEN_CMD_STOP, epc_priv->base + PCI_EPC_GEN_CMD);
}

static const struct pci_epc_features *pci_ep_generic_get_features(struct pci_epc *epc,
							    u8 fn, u8 vfn)
{
	return &((struct pci_ep_generic *)epc_get_drvdata(epc))->features;
}

static const struct pci_epc_ops pci_ep_genericc_ops = {
	.write_header = pci_ep_generic_write_header,
	.set_bar = pci_ep_generic_set_bar,
	.clear_bar = pci_ep_generic_clear_bar,
	.map_addr = pci_ep_generic_map_addr,
	.unmap_addr = pci_ep_generic_unmap_addr,
	.set_msi = pci_ep_generic_set_msi,
	.get_msi = pci_ep_generic_get_msi,
	.set_msix = pci_ep_generic_set_msix,
	.get_msix = pci_ep_generic_get_msix,
	.raise_irq = pci_ep_generic_raise_irq,
	.map_msi_irq = pci_ep_generic_map_msi_irq,
	.start = pci_ep_generic_start,
	.stop = pci_ep_generic_stop,
	.get_features = pci_ep_generic_get_features,
};

static void pci_ep_generic_init_features(struct pci_ep_generic *epc_priv)
{
	u32 caps = readl_relaxed(epc_priv->base + PCI_EPC_GEN_CAPS);
	struct pci_epc_features *f = &epc_priv->features;

	f->intx_capable = !!(caps & PCI_EPC_GEN_CAP_INTX);
	f->msi_capable = !!(caps & PCI_EPC_GEN_CAP_MSI);
	f->msix_capable = !!(caps & PCI_EPC_GEN_CAP_MSIX);
	f->linkup_notifier = !!(caps & PCI_EPC_GEN_CAP_LINKUP_NOTIFY);
	f->dynamic_inbound_mapping = !!(caps & PCI_EPC_GEN_CAP_DYN_INBOUND_MAP);
	f->subrange_mapping = !!(caps & PCI_EPC_GEN_CAP_SUBRANGE);

	/* All BARs are fully programmable */
	for (int i = 0; i < PCI_STD_NUM_BARS; i++)
		f->bar[i].type = BAR_PROGRAMMABLE;
}

static int pci_ep_generic_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct pci_ep_generic *epc_priv;
	struct pci_epc *epc;
	struct resource *res;
	int err;

	epc_priv = devm_kzalloc(dev, sizeof(*epc_priv), GFP_KERNEL);
	if (!epc_priv)
		return -ENOMEM;

	epc_priv->base = devm_platform_ioremap_resource_byname(pdev, "ctrl");
	if (IS_ERR(epc_priv->base))
		return PTR_ERR(epc_priv->base);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "outbound");
	if (!res) {
		dev_err(dev, "missing \"outbound\" memory resource\n");
		return -ENODEV;
	}

	if (!devm_request_mem_region(dev, res->start, resource_size(res),
				     dev_name(dev))) {
		dev_err(dev, "outbound window already in use\n");
		return -EBUSY;
	}

	/*
	 * Endpoint BAR backing memory and outbound-mapped RC memory both live
	 * in system RAM below 4G, so restrict coherent DMA to 32 bits.  This
	 * lets pci_epf_alloc_space()/dma_alloc_coherent() allocate from system
	 * RAM, which the endpoint function then exposes through its BARs.
	 */
	err = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (err) {
		dev_err(dev, "failed to set DMA mask: %d\n", err);
		return err;
	}

	pci_ep_generic_init_features(epc_priv);

	epc = devm_pci_epc_create(dev, &pci_ep_genericc_ops);
	if (IS_ERR(epc))
		return PTR_ERR(epc);

	epc->max_functions = readl_relaxed(epc_priv->base + PCI_EPC_GEN_MAX_FUNC);
	epc_set_drvdata(epc, epc_priv);
	platform_set_drvdata(pdev, epc);

	err = pci_epc_mem_init(epc, res->start, resource_size(res), PAGE_SIZE);
	if (err) {
		dev_err(dev, "failed to initialise outbound window: %d\n", err);
		return err;
	}

	pci_epc_init_notify(epc);

	return 0;
}

static void pci_ep_generic_remove(struct platform_device *pdev)
{
	pci_epc_mem_exit(platform_get_drvdata(pdev));
}

static const struct of_device_id pci_ep_generic_of_match[] = {
	{ .compatible = "pci-ep-generic" },
	{ }
};
MODULE_DEVICE_TABLE(of, pci_ep_generic_of_match);

static struct platform_driver pci_ep_generic_driver = {
	.driver = {
		.name = "pci-ep-generic",
		.of_match_table = pci_ep_generic_of_match,
	},
	.probe = pci_ep_generic_probe,
	.remove = pci_ep_generic_remove,
};
module_platform_driver(pci_ep_generic_driver);

MODULE_DESCRIPTION("Generic PCI Endpoint Controller driver");
MODULE_LICENSE("GPL");
