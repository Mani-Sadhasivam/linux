/* SPDX-License-Identifier: ((GPL-2.0 WITH Linux-syscall-note) OR BSD-3-Clause) */
/*
 * Virtio message transport header.
 *
 * Copyright (c) 2024 Advanced Micro Devices, Inc.
 * Written by Edgar E. Iglesias <edgar.iglesias@amd.com>
 *
 * Copyright (C) 2024 Google LLC and Linaro.
 * Viresh Kumar <viresh.kumar@linaro.org>
 *
 * The Virtio message transport allows virtio devices to be used over a virtual
 * virtio-msg channel. The channel interface is meant to be implemented using
 * the architecture specific hardware-assisted fast path, like ARM Firmware
 * Framework (FFA).
 */

#ifndef _DRIVERS_VIRTIO_VIRTIO_MSG_H
#define _DRIVERS_VIRTIO_VIRTIO_MSG_H

#include <linux/list.h>
#include <linux/pm.h>
#include <linux/spinlock.h>
#include <linux/virtio.h>
#include <uapi/linux/virtio_msg.h>

struct virtio_msg_device;

/**
 * struct virtio_msg_ops - operations for configuring a virtio message device
 * @send: Transfer a message.
 * @remove: Unregister the device.
 */
struct virtio_msg_ops {
	int (*send)(struct virtio_msg_device *vmdev, struct virtio_msg *request,
		    struct virtio_msg *response);
	const char *(*bus_name)(struct virtio_msg_device *vmdev);
	void (*synchronize_cbs)(struct virtio_msg_device *vmdev);
	void (*release)(struct virtio_msg_device *vmdev);
	int (*prepare_vqs)(struct virtio_msg_device *vmdev);
	void (*release_vqs)(struct virtio_msg_device *vmdev);
};

/**
 * struct virtio_msg_device - representation of a device using virtio message
 */
struct virtio_msg_device {
	struct virtio_device vdev;
	struct virtio_msg_ops *ops;
	const void *data;

	/* a list of queues so we can dispatch IRQs */
	spinlock_t lock;
	struct list_head virtqueues;
};

struct virtio_msg_vq {
	struct virtqueue *vq;
	struct list_head node;
};

#ifdef CONFIG_PM_SLEEP
static inline int virtio_msg_freeze(struct virtio_msg_device *vmdev)
{
	return virtio_device_freeze(&vmdev->vdev);
}

static inline int virtio_msg_restore(struct virtio_msg_device *vmdev)
{
	return virtio_device_restore(&vmdev->vdev);
}
#endif

int virtio_msg_receive(struct virtio_msg_device *vmdev, struct virtio_msg *msg);
int virtio_msg_register(struct virtio_msg_device *vmdev);
void virtio_msg_unregister(struct virtio_msg_device *vmdev);

#endif /* _DRIVERS_VIRTIO_VIRTIO_MSG_H */
