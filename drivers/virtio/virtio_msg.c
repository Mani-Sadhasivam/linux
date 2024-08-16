// SPDX-License-Identifier: GPL-2.0+
/*
 * Virtio message transport.
 *
 * Copyright (C) 2024 Google LLC and Linaro.
 * Viresh Kumar <viresh.kumar@linaro.org>
 *
 * The Virtio message transport allows virtio devices to be used over a virtual
 * virtio-msg channel. The channel interface is meant to be implemented using
 * the architecture specific hardware-assisted fast path, like ARM Firmware
 * Framework (FFA).
 */

#define pr_fmt(fmt) "virtio-msg: " fmt

#include <linux/limits.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/virtio.h>
#include <linux/virtio_config.h>
#include <linux/virtio_ring.h>
#include <uapi/linux/virtio_msg.h>

#include "virtio_msg.h"

#define U24_MAX				((1 << 24) - 1)
#define to_virtio_msg_device(_dev)	container_of(_dev, struct virtio_msg_device, vdev)

static void virtio_msg_prepare(struct virtio_msg *msg, bool request, u8 msg_id,
			       u16 dev_id)
{
	/* Initialize all fields (including padding) to 0 */
	memset(msg, 0, sizeof(*msg));

	if (!request)
		msg->type = VIRTIO_MSG_TYPE_RESPONSE;

	msg->id = msg_id;
	msg->dev_id = cpu_to_le16(dev_id);
}

static void virtio_request_prepare(struct virtio_msg *msg, u8 msg_id, u16 dev_id)
{
	virtio_msg_prepare(msg, true, msg_id, dev_id);
}

static int vmsg_get_device_info(struct virtio_msg_device *vmdev)
{
	struct virtio_msg request, response;
	int ret;

	virtio_request_prepare(&request, VIRTIO_MSG_DEVICE_INFO, 0);

	ret = vmdev->ops->send(vmdev, &request, &response);
	if (ret < 0)
		return ret;

	vmdev->vdev.id.device = le32_to_cpu(response.get_device_info_resp.device_id);
	if (vmdev->vdev.id.device == 0) {
		/*
		 * virtio-msg device with an ID 0 is a (dummy) placeholder
		 * with no function.
		 */
		return -ENODEV;
	}

	vmdev->vdev.id.vendor = le32_to_cpu(response.get_device_info_resp.vendor_id);

	/* TODO: Device Version ? */
	return 0;
}

static u64 vmsg_get_features(struct virtio_device *vdev)
{
	struct virtio_msg_device *vmdev = to_virtio_msg_device(vdev);
	struct virtio_msg request, response;
	int ret;

	virtio_request_prepare(&request, VIRTIO_MSG_GET_FEATURES, vmdev->vdev.id.device);
	request.get_features.index = 0;

	ret = vmdev->ops->send(vmdev, &request, &response);
	if (ret < 0)
		return ret;

	return le64_to_cpu(response.get_features_resp.features[0]);
}

static int vmsg_finalize_features(struct virtio_device *vdev)
{
	struct virtio_msg_device *vmdev = to_virtio_msg_device(vdev);
	struct virtio_msg request;
	int ret;

	/* Give virtio_ring a chance to accept features. */
	vring_transport_features(vdev);

	virtio_request_prepare(&request, VIRTIO_MSG_SET_FEATURES, vmdev->vdev.id.device);
	request.set_features.index = 0;
	request.set_features.features[0]= cpu_to_le64(vmdev->vdev.features);

	ret = vmdev->ops->send(vmdev, &request, NULL);
	if (ret < 0)
		return ret;

	return 0;
}

static void vmsg_get(struct virtio_device *vdev, unsigned int offset,
		   void *buf, unsigned int len)
{
	struct virtio_msg_device *vmdev = to_virtio_msg_device(vdev);
	struct virtio_msg request, response;
	u64 data;
	int ret;

	/* The protocol has reserved only 3 bytes for the offset */
	BUG_ON(offset > U24_MAX);
	BUG_ON(len > 8);

	virtio_request_prepare(&request, VIRTIO_MSG_GET_CONFIG, vmdev->vdev.id.device);

	offset = cpu_to_le32(offset);
	request.get_config.offset[0] = (u8) offset;
	request.get_config.offset[1] = (u8) (offset >> 8);
	request.get_config.offset[2] = (u8) (offset >> 16);
	request.get_config.size = (u8) len;

	ret = vmdev->ops->send(vmdev, &request, &response);
	if (ret < 0) {
		pr_err("%s: Failed to send request (%d)\n", __func__, ret);
		return;
	}

	data = le64_to_cpu(response.get_config_resp.data[0]);
	memcpy(buf, &data, len);
}

static void vmsg_set(struct virtio_device *vdev, unsigned int offset,
		   const void *buf, unsigned int len)
{
	struct virtio_msg_device *vmdev = to_virtio_msg_device(vdev);
	struct virtio_msg request;
	u64 data;
	int ret;

	/* The protocol has reserved only 3 bytes for the offset */
	BUG_ON(offset > U24_MAX);
	BUG_ON(len > 8);

	virtio_request_prepare(&request, VIRTIO_MSG_SET_CONFIG, vmdev->vdev.id.device);

	offset = cpu_to_le32(offset);
	request.set_config.offset[0] = (u8) offset;
	request.set_config.offset[1] = (u8) (offset >> 8);
	request.set_config.offset[2] = (u8) (offset >> 16);
	request.set_config.size = (u8) len;

	memcpy(&data, buf, len);
	request.set_config.data[0] = le64_to_cpu(data);

	ret = vmdev->ops->send(vmdev, &request, NULL);
	if (ret < 0) {
		pr_err("%s: Failed to send request (%d)\n", __func__, ret);
		return;
	}
}

static u32 vmsg_generation(struct virtio_device *vdev)
{
	struct virtio_msg_device *vmdev = to_virtio_msg_device(vdev);
	struct virtio_msg request, response;
	int ret;

	virtio_request_prepare(&request, VIRTIO_MSG_GET_CONFIG_GEN, vmdev->vdev.id.device);

	ret = vmdev->ops->send(vmdev, &request, &response);
	if (ret < 0) {
		pr_err("%s: Failed to send request (%d)\n", __func__, ret);
		return 0;
	}

	return le32_to_cpu(response.get_config_gen_resp.generation);
}

static u8 vmsg_get_status(struct virtio_device *vdev)
{
	struct virtio_msg_device *vmdev = to_virtio_msg_device(vdev);
	struct virtio_msg request, response;
	int ret;

	virtio_request_prepare(&request, VIRTIO_MSG_GET_DEVICE_STATUS, vmdev->vdev.id.device);

	ret = vmdev->ops->send(vmdev, &request, &response);
	if (ret < 0) {
		pr_err("%s: Failed to send request (%d)\n", __func__, ret);
		return 0;
	}

	return (u8) le32_to_cpu(response.get_device_status_resp.status);
}

static void vmsg_set_status(struct virtio_device *vdev, u8 status)
{
	struct virtio_msg_device *vmdev = to_virtio_msg_device(vdev);
	struct virtio_msg request;
	int ret;

	virtio_request_prepare(&request, VIRTIO_MSG_SET_DEVICE_STATUS, vmdev->vdev.id.device);
	request.set_device_status.status = cpu_to_le32(status);

	ret = vmdev->ops->send(vmdev, &request, NULL);
	if (ret < 0)
		pr_err("%s: Failed to send request (%d)\n", __func__, ret);
}

static void vmsg_reset(struct virtio_device *vdev)
{
	/* 0 status means a reset. */
	vmsg_set_status(vdev, 0);
}

static bool vmsg_notify(struct virtqueue *vq)
{
	struct virtio_msg_device *vmdev = to_virtio_msg_device(vq->vdev);
	struct virtio_msg request;
	int ret;

	virtio_request_prepare(&request, VIRTIO_MSG_EVENT_AVAIL, vmdev->vdev.id.device);
	request.event_avail.index = cpu_to_le32(vq->index);

	ret = vmdev->ops->send(vmdev, &request, NULL);
	if (ret < 0) {
		pr_err("%s: Failed to send request (%d)\n", __func__, ret);
		return false;
	}

	return true;
}

static bool vmsg_notify_with_data(struct virtqueue *vq)
{
	struct virtio_msg_device *vmdev = to_virtio_msg_device(vq->vdev);
	u32 data = vring_notification_data(vq);
	struct virtio_msg request;
	int ret;

	virtio_request_prepare(&request, VIRTIO_MSG_EVENT_AVAIL, vmdev->vdev.id.device);
	request.event_avail.index = cpu_to_le32(data | 0xFFFF);
	data >>= 16;

	request.event_avail.next_offset = cpu_to_le64(data | 0x7FFF);
	request.event_avail.next_wrap = cpu_to_le64(data >> 15);

	ret = vmdev->ops->send(vmdev, &request, NULL);
	if (ret < 0) {
		pr_err("%s: Failed to send request (%d)\n", __func__, ret);
		return false;
	}

	return true;
}

int virtio_msg_receive(struct virtio_msg_device *vmdev, struct virtio_msg *msg)
{
	struct virtio_msg_vq *info;
	bool handled = false;
	unsigned long flags;
	unsigned int index;
	int ret = 0;

	if (msg->id == VIRTIO_MSG_EVENT_CONFIG) {
		/*
		 * Though we have the config changes available here, the kernel
		 * implementation lets the driver fetch them again.
		 */
		virtio_config_changed(&vmdev->vdev);
	} else if (msg->id == VIRTIO_MSG_EVENT_USED) {
		index = le32_to_cpu(msg->event_used.index);

		spin_lock_irqsave(&vmdev->lock, flags);
		list_for_each_entry(info, &vmdev->virtqueues, node) {
			if (index == info->vq->index) {
				if (vring_interrupt(0, info->vq) != IRQ_HANDLED)
					ret = -EIO;

				handled = true;
				break;
			}
		}
		spin_unlock_irqrestore(&vmdev->lock, flags);

		if (!handled) {
			ret = -EINVAL;
			pr_err("%s: Failed to find virtqueue for message (%u)", __func__, index);
		}
	} else {
		pr_err("%s: Unexpected message id: (%u)\n", __func__, msg->id);
		ret = -EINVAL;
	}

	return ret;
}
EXPORT_SYMBOL_GPL(virtio_msg_receive);

static void vmsg_del_vq(struct virtqueue *vq)
{
	struct virtio_msg_device *vmdev = to_virtio_msg_device(vq->vdev);
	struct virtio_msg_vq *info = vq->priv;
	struct virtio_msg request;
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&vmdev->lock, flags);
	list_del(&info->node);
	spin_unlock_irqrestore(&vmdev->lock, flags);

	/* Reset the virtqueue */
	virtio_request_prepare(&request, VIRTIO_MSG_RESET_VQUEUE, vmdev->vdev.id.device);
	request.reset_vqueue.index = cpu_to_le32(vq->index);

	ret = vmdev->ops->send(vmdev, &request, NULL);
	if (ret < 0)
		pr_err("%s: Failed to send request (%d)\n", __func__, ret);

	vring_del_virtqueue(vq);

	kfree(info);
}

static void vmsg_del_vqs(struct virtio_device *vdev)
{
	struct virtio_msg_device *vmdev = to_virtio_msg_device(vdev);
	struct virtqueue *vq, *n;

	list_for_each_entry_safe(vq, n, &vmdev->vdev.vqs, list)
		vmsg_del_vq(vq);

	vmdev->ops->release_vqs(vmdev);
}

static struct virtqueue *vmsg_setup_vq(struct virtio_msg_device *vmdev,
				       unsigned int index,
				       void (*callback)(struct virtqueue *vq),
				       const char *name, bool ctx)
{
	bool (*notify)(struct virtqueue *vq);
	struct virtio_msg_vq *info;
	struct virtio_msg request, response;
	struct virtqueue *vq;
	unsigned long flags;
	unsigned int num;
	int ret;

	if (__virtio_test_bit(&vmdev->vdev, VIRTIO_F_NOTIFICATION_DATA))
		notify = vmsg_notify_with_data;
	else
		notify = vmsg_notify;

	/* Get virtqueue max size from device */
	virtio_request_prepare(&request, VIRTIO_MSG_GET_VQUEUE, vmdev->vdev.id.device);
	request.get_vqueue.index = cpu_to_le32(index);

	ret = vmdev->ops->send(vmdev, &request, &response);
	if (ret < 0) {
		pr_err("%s: Failed to send request (%d)\n", __func__, ret);
		return ERR_PTR(ret);
	}

	num = le64_to_cpu(response.get_vqueue_resp.max_size);
	if (!num)
		return ERR_PTR(-ENOENT);

	info = kmalloc(sizeof(*info), GFP_KERNEL);
	if (!info)
		return ERR_PTR(-ENOMEM);

	/* Create the vring */
	vq = vring_create_virtqueue(index, num, PAGE_SIZE, &vmdev->vdev, true,
				    true, ctx, notify, callback, name);
	if (!vq) {
		ret = -ENOMEM;
		goto free_info;
	}

	vq->num_max = num;

	/* Send virtqueue configuration to the device */
	virtio_request_prepare(&request, VIRTIO_MSG_SET_VQUEUE, vmdev->vdev.id.device);
	request.set_vqueue.index = cpu_to_le32(index);
	request.set_vqueue.size = cpu_to_le64(virtqueue_get_vring_size(vq));
	request.set_vqueue.descriptor_addr = cpu_to_le64(virtqueue_get_desc_addr(vq));
	request.set_vqueue.driver_addr = cpu_to_le64(virtqueue_get_avail_addr(vq));
	request.set_vqueue.device_addr = cpu_to_le64(virtqueue_get_used_addr(vq));

	ret = vmdev->ops->send(vmdev, &request, NULL);
	if (ret < 0) {
		pr_err("%s: Failed to send request (%d)\n", __func__, ret);
		goto del_vq;
	}

	vq->priv = info;
	info->vq = vq;

	spin_lock_irqsave(&vmdev->lock, flags);
	list_add(&info->node, &vmdev->virtqueues);
	spin_unlock_irqrestore(&vmdev->lock, flags);

	return vq;

del_vq:
	vring_del_virtqueue(vq);
free_info:
	kfree(info);
	return ERR_PTR(ret);
}

static int vmsg_find_vqs(struct virtio_device *vdev, unsigned int nvqs,
		       struct virtqueue *vqs[],
		       struct virtqueue_info vqs_info[],
		       struct irq_affinity *desc)
{
	struct virtio_msg_device *vmdev = to_virtio_msg_device(vdev);
	int i, ret, queue_idx = 0;

	ret = vmdev->ops->prepare_vqs(vmdev);
	if (ret)
		return ret;

	for (i = 0; i < nvqs; ++i) {
		struct virtqueue_info *vqi = &vqs_info[i];

		if (!vqi->name) {
			vqs[i] = NULL;
			continue;
		}

		vqs[i] = vmsg_setup_vq(vmdev, queue_idx++, vqi->callback,
				     vqi->name, vqi->ctx);
		if (IS_ERR(vqs[i])) {
			vmsg_del_vqs(vdev);
			return PTR_ERR(vqs[i]);
		}
	}

	return 0;
}

static const char *vmsg_bus_name(struct virtio_device *vdev)
{
	struct virtio_msg_device *vmdev = to_virtio_msg_device(vdev);

	return vmdev->ops->bus_name(vmdev);
}

static void vmsg_synchronize_cbs(struct virtio_device *vdev)
{
	struct virtio_msg_device *vmdev = to_virtio_msg_device(vdev);

	vmdev->ops->synchronize_cbs(vmdev);
}

static void virtio_msg_release_dev(struct device *_d)
{
	struct virtio_device *vdev =
			container_of(_d, struct virtio_device, dev);
	struct virtio_msg_device *vmdev = to_virtio_msg_device(vdev);

	vmdev->ops->release(vmdev);
}

static struct virtio_config_ops virtio_msg_config_ops = {
	.get		= vmsg_get,
	.set		= vmsg_set,
	.generation	= vmsg_generation,
	.get_status	= vmsg_get_status,
	.set_status	= vmsg_set_status,
	.reset		= vmsg_reset,
	.find_vqs	= vmsg_find_vqs,
	.del_vqs	= vmsg_del_vqs,
	.get_features	= vmsg_get_features,
	.finalize_features = vmsg_finalize_features,
	.bus_name	= vmsg_bus_name,
};

int virtio_msg_register(struct virtio_msg_device *vmdev)
{
	int ret;

	/*
	 * Field expected to be filled by underlying architecture specific
	 * transport layer are vmdev->data (optional), vmdev->ops, and
	 * vmdev->vdev.dev.parent.
	 */
	if (!vmdev || !vmdev->ops) {
		ret = -EINVAL;
		goto out;
	}

	vmdev->vdev.config = &virtio_msg_config_ops;
	vmdev->vdev.dev.release = virtio_msg_release_dev;
	INIT_LIST_HEAD(&vmdev->virtqueues);
	spin_lock_init(&vmdev->lock);

	if (vmdev->ops->synchronize_cbs)
		virtio_msg_config_ops.synchronize_cbs = vmsg_synchronize_cbs;

	ret = vmsg_get_device_info(vmdev);
	if (ret)
		goto out;

	ret = register_virtio_device(&vmdev->vdev);
	if (ret)
		put_device(&vmdev->vdev.dev);

	return ret;

out:
	vmdev->ops->release(vmdev);
	return ret;
}
EXPORT_SYMBOL_GPL(virtio_msg_register);

void virtio_msg_unregister(struct virtio_msg_device *vmdev)
{
	unregister_virtio_device(&vmdev->vdev);
}
EXPORT_SYMBOL_GPL(virtio_msg_unregister);

MODULE_AUTHOR("Viresh Kumar <viresh.kumar@linaro.org>");
MODULE_DESCRIPTION("Virtio message transport");
MODULE_LICENSE("GPL");
