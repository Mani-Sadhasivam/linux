// SPDX-License-Identifier: GPL-2.0+
/*
 * Virtio message transport over PCI.
 *
 * Copyright (C) 2024 Linaro.
 * Author: Manivannan Sadhasivam <manivannan.sadhasivam@linaro.org>
 */

#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/slab.h>
#include <linux/virtio.h>
#include <linux/virtio_config.h>
#include <linux/virtio_ring.h>

#include "virtio_msg.h"

#define VIRTIO_MSG_PCI_MSGS 64

struct virtio_msg_pci_device {
	struct virtio_msg_device vmdev;
	struct pci_dev *pci_dev; /* Really needed? */

	void __iomem *notification;
	struct virtio_msg_pci_regs *msg;

	u64 driver_bitmap ____cacheline_aligned;
	u64 device_bitmap ____cacheline_aligned;
};

struct virtio_msg_pci_regs {
	__le64 num_msgs;
	__le64 driver_bitmap;
	__le64 device_bitmap;
	
	struct virtio_msg msgs[VIRTIO_MSG_PCI_MSGS];
} ____cacheline_aligned;

#define to_virtio_msg_pci_device(_vmdev) \
	container_of(_vmdev, struct virtio_msg_pci_device, vmdev)

static irqreturn_t config_interrupt(int irq, void *opaque)
{
	struct virtio_msg_pci_device *vmp_dev = opaque;
	struct virtio_msg msg;

	/* We don't have any msg here, lets create one to make it work */
	memset(&msg, 0, sizeof(msg));
	msg.id = VIRTIO_MSG_EVENT_CONFIG;

	virtio_msg_receive(&vmp_dev->vmdev, &msg);

	return IRQ_HANDLED;
}

static int virtio_msg_pci_send(struct virtio_msg_device *vmdev,
				struct virtio_msg *request,
				struct virtio_msg *response)
{
	struct virtio_msg_pci_device *vmp_dev = to_virtio_msg_pci_device(vmdev);
	u64 msg_bitmap;
	u32 entry, val;

	/* Update the device bitmap */
	vmp_dev->device_bitmap = readq(&vmp_dev->msg->device_bitmap);

	/* Find a free entry */
	msg_bitmap = vmp_dev->driver_bitmap ^ vmp_dev->device_bitmap;
	entry = ffs(~msg_bitmap);
	if (!entry) {
		pr_err("%s: No entry available\n", __func__);
		return -ENOENT; /* TODO */
	}
	entry--;

	/* Fill the message */
	memcpy_toio(&vmp_dev->msg->msgs[entry],
			request, sizeof(struct virtio_msg));

	/* Update the driver bitmap */
	vmp_dev->driver_bitmap = vmp_dev->driver_bitmap ^ BIT(entry);
	writeq(vmp_dev->driver_bitmap, &vmp_dev->msg->driver_bitmap);
	
	/* Send notification */
	writel(1, vmp_dev->notification);

	/* Poll for the response */
	readl_poll_timeout(vmp_dev->notification, val, !val, 0, 0);

	/* Update the device bitmap */
	vmp_dev->device_bitmap = readq(&vmp_dev->msg->device_bitmap);

	/* FIXME */
	if (!response)
		return 0;

	msg_bitmap = vmp_dev->driver_bitmap ^ vmp_dev->device_bitmap;
	entry = ffs(~msg_bitmap);
	if (!entry) {
		pr_err("%s: No entry available\n", __func__);
		return -ENOENT; /* TODO */
	}
	entry--;

	/* Get the response */
	memcpy_fromio(response, &vmp_dev->msg->msgs[entry],
			sizeof(struct virtio_msg));

	return 0;
}

static const char *virtio_msg_pci_bus_name(struct virtio_msg_device *vmdev)
{
	struct virtio_msg_pci_device *vmp_dev = to_virtio_msg_pci_device(vmdev);

	return pci_name(vmp_dev->pci_dev);
}

static void virtio_msg_pci_synchronize_cbs(struct virtio_msg_device *vmdev)
{
	struct virtio_msg_pci_device *vmp_dev = to_virtio_msg_pci_device(vmdev);

	synchronize_irq(pci_irq_vector(vmp_dev->pci_dev, 0));
}

static void virtio_msg_pci_release(struct virtio_msg_device *vmdev)
{
	struct virtio_msg_pci_device *vmp_dev = to_virtio_msg_pci_device(vmdev);

	kfree(vmp_dev);
}

static int virtio_msg_pci_vqs_prepare(struct virtio_msg_device *vmdev, u32 nvqs)
{
	struct virtio_msg_pci_device *vmp_dev = to_virtio_msg_pci_device(vmdev);
	int ret;

	/* TODO: Fallback to shared vector */
	ret = pci_alloc_irq_vectors(vmp_dev->pci_dev, 1, nvqs, PCI_IRQ_MSI);
	if (ret < 0) {
		dev_err(&vmp_dev->pci_dev->dev, "Error allocating MSI vectors %d\n", ret);
		return ret;
	}

	return 0;
}

#define VIRTIO_MSG_NVQ 3 /* FIXME */

static void virtio_msg_pci_vqs_release(struct virtio_msg_device *vmdev)
{
	struct virtio_msg_pci_device *vmp_dev = to_virtio_msg_pci_device(vmdev);

	pci_free_irq(vmp_dev->pci_dev, 0, vmp_dev);
}

static int virtio_msg_alloc_vq_vector(struct virtio_msg_device *vmdev, struct virtqueue *vq,
			       const char *name, u32 queue_idx)
{
	struct virtio_msg_pci_device *vmp_dev = to_virtio_msg_pci_device(vmdev);
	char *vec_name;

	vec_name = devm_kasprintf(&vmp_dev->pci_dev->dev, GFP_KERNEL, "%s: %d",
			      name, queue_idx);
	if (!vec_name)
		return -ENOMEM;

	if (queue_idx == VIRTIO_MSG_NVQ - 1)
		return request_irq(pci_irq_vector(vmp_dev->pci_dev, queue_idx), config_interrupt, 0,
					vec_name, vmp_dev);
	else
		return request_irq(pci_irq_vector(vmp_dev->pci_dev, queue_idx), vring_interrupt, 0,
					vec_name, vq);
}

static void virtio_msg_free_vq_vector(struct virtio_msg_device *vmdev,
				      struct virtqueue *vq, u32 queue_idx)
{
	struct virtio_msg_pci_device *vmp_dev = to_virtio_msg_pci_device(vmdev);

	free_irq(pci_irq_vector(vmp_dev->pci_dev, queue_idx), vq);
}

static struct virtio_msg_ops vmp_ops = {
	.send = virtio_msg_pci_send,
	.bus_name = virtio_msg_pci_bus_name,
	.synchronize_cbs = virtio_msg_pci_synchronize_cbs,
	.release = virtio_msg_pci_release,
	.prepare_vqs = virtio_msg_pci_vqs_prepare,
	.release_vqs = virtio_msg_pci_vqs_release,
	.alloc_vq_vector = virtio_msg_alloc_vq_vector,
	.free_vq_vector = virtio_msg_free_vq_vector,
};

/* FIXME: Using the Qcom's modem device id */
static const struct pci_device_id virtio_msg_pci_id_table[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_QCOM, 0x0306) },
	{ 0 }
};

MODULE_DEVICE_TABLE(pci, virtio_msg_pci_id_table);

static int virtio_msg_pci_probe(struct pci_dev *pci_dev,
			    const struct pci_device_id *id)
{
	struct virtio_msg_pci_device *vmp_dev;
	struct device *dev = &pci_dev->dev;
	int ret;

	/* devm? */
	vmp_dev = kzalloc(sizeof(struct virtio_msg_pci_device), GFP_KERNEL);
	if (!vmp_dev)
		return -ENOMEM;

	pci_set_drvdata(pci_dev, vmp_dev);
	vmp_dev->vmdev.vdev.dev.parent = dev;
	vmp_dev->vmdev.ops = &vmp_ops;
	vmp_dev->pci_dev = pci_dev;

	ret = pcim_enable_device(pci_dev);
	if (ret)
		goto err_free_vmp_dev;

	ret = pcim_request_region(pci_dev, 0, "virtio-msg-pci");
	if (ret)
		goto err_free_vmp_dev;

	/* 
	 * All regions are in BAR0 with below offsets:
	 *
	 * 0x0 - Doorbell
	 * 0x1000 - Msg
	 */
	vmp_dev->notification = pcim_iomap_range(pci_dev, 0, 0x0, SZ_4K);
	if (IS_ERR(vmp_dev->notification)) {
		ret = PTR_ERR(vmp_dev->notification);
		goto err_free_vmp_dev;
	}

	vmp_dev->msg = pcim_iomap_range(pci_dev, 0, 0x1000, SZ_4K);
	if (IS_ERR(vmp_dev->msg)) {
		ret = PTR_ERR(vmp_dev->msg);
		goto err_free_vmp_dev;
	}

	dma_set_mask_and_coherent(dev, DMA_BIT_MASK(64));

	return virtio_msg_register(&vmp_dev->vmdev);

err_free_vmp_dev:
	kfree(vmp_dev);

	return ret;
}

static void virtio_msg_pci_remove(struct pci_dev *pci_dev)
{
	struct virtio_msg_pci_device *vmp_dev = pci_get_drvdata(pci_dev);

	virtio_msg_unregister(&vmp_dev->vmdev);
}

static struct pci_driver virtio_msg_pci_driver = {
	.name		= "virtio-msg-pci",
	.id_table	= virtio_msg_pci_id_table,
	.probe		= virtio_msg_pci_probe,
	.remove		= virtio_msg_pci_remove,
};

module_pci_driver(virtio_msg_pci_driver);

MODULE_AUTHOR("Manivannan Sadhasivam <manivannan.sadhasivam@linaro.org>");
MODULE_DESCRIPTION("Virtio message transport over PCI");
MODULE_LICENSE("GPL");
