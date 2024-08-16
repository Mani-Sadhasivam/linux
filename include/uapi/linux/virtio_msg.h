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

#ifndef _LINUX_VIRTIO_MSG_H
#define _LINUX_VIRTIO_MSG_H

#include <linux/types.h>

/* Message types */
#define VIRTIO_MSG_CONNECT		0x01
#define VIRTIO_MSG_DISCONNECT		0x02
#define VIRTIO_MSG_DEVICE_INFO		0x03
#define VIRTIO_MSG_GET_FEATURES		0x04
#define VIRTIO_MSG_SET_FEATURES		0x05
#define VIRTIO_MSG_GET_CONFIG		0x06
#define VIRTIO_MSG_SET_CONFIG		0x07
#define VIRTIO_MSG_GET_CONFIG_GEN	0x08
#define VIRTIO_MSG_GET_DEVICE_STATUS	0x09
#define VIRTIO_MSG_SET_DEVICE_STATUS	0x0a
#define VIRTIO_MSG_GET_VQUEUE		0x0b
#define VIRTIO_MSG_SET_VQUEUE		0x0c
#define VIRTIO_MSG_RESET_VQUEUE		0x0d
#define VIRTIO_MSG_EVENT_CONFIG		0x10
#define VIRTIO_MSG_EVENT_AVAIL		0x11
#define VIRTIO_MSG_EVENT_USED		0x12
#define VIRTIO_MSG_MAX			VIRTIO_MSG_EVENT_USED

#define VIRTIO_MSG_MAX_SIZE		40

#define VIRTIO_MSG_TYPE_RESPONSE	(1 << 0)
#define VIRTIO_MSG_TYPE_BUS		(1 << 1)

/* Message payload format */

struct get_device_info_resp {
	__le32 device_version;
	__le32 device_id;
	__le32 vendor_id;
};

struct get_features {
	__le32 index;
};

struct get_features_resp {
	__le32 index;
	__le64 features[4];
} __attribute__((packed));

struct set_features {
	__le32 index;
	__le64 features[4];
} __attribute__((packed));

struct set_features_resp {
	__le32 index;
	__le64 features[4];
} __attribute__((packed));

struct get_config {
	__u8 offset[3];
	__u8 size;
};

struct get_config_resp {
	__u8 offset[3];
	__u8 size;
	__le64 data[4];
} __attribute__((packed));

struct set_config {
	__u8 offset[3];
	__u8 size;
	__le64 data[4];
} __attribute__((packed));

struct set_config_resp {
	__u8 offset[3];
	__u8 size;
	__le64 data[4];
} __attribute__((packed));

struct get_config_gen_resp {
	__le32 generation;
};

struct get_device_status_resp {
	__le32 status;
};

struct set_device_status {
	__le32 status;
};

struct get_vqueue {
	__le32 index;
};

struct get_vqueue_resp {
	__le32 index;
	__le64 max_size;
} __attribute__((packed));

struct set_vqueue {
	__le32 index;
	__le64 size;
	__le64 descriptor_addr;
	__le64 driver_addr;
	__le64 device_addr;
} __attribute__((packed));

struct reset_vqueue {
	__le32 index;
};

struct event_config {
	__le32 status;
	__u8 offset[3];
	__u8 size;
	__le32 value[4];
};

struct event_avail {
	__le32 index;
	__le64 next_offset;
	__le64 next_wrap;
} __attribute__((packed));

struct event_used {
	__le32 index;
};

struct virtio_msg {
	__u8 type;
	__u8 id;
	__le16 dev_id;

	union {
		__u8 payload_u8[36];

		struct get_device_info_resp get_device_info_resp;
		struct get_features get_features;
		struct get_features_resp get_features_resp;
		struct set_features set_features;
		struct set_features_resp set_features_resp;
		struct get_config get_config;
		struct get_config_resp get_config_resp;
		struct set_config set_config;
		struct set_config_resp set_config_resp;
		struct get_config_gen_resp get_config_gen_resp;
		struct get_device_status_resp get_device_status_resp;
		struct set_device_status set_device_status;
		struct get_vqueue get_vqueue;
		struct get_vqueue_resp get_vqueue_resp;
		struct set_vqueue set_vqueue;
		struct reset_vqueue reset_vqueue;
		struct event_config event_config;
		struct event_avail event_avail;
		struct event_used event_used;
	};
} __attribute__((packed));

#endif /* _LINUX_VIRTIO_MSG_H */
