/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __PCI_EPF_VIRTIO_H__
#define __PCI_EPF_VIRTIO_H__

#include <linux/dmaengine.h>
#include <linux/pci-epc.h>
#include <linux/pci-epf.h>
#include <linux/vringh.h>
#include <uapi/linux/virtio_msg.h>

struct epf_vringh {
	struct vringh vrh;
	void __iomem *desc_virt;
	phys_addr_t desc_phys;
	unsigned int num;
	u16 msi_vec;
	u16 queue_sel;
};

struct epf_vq {
	struct kthread_worker *worker;
	struct kthread_work work;
	struct epf_virtio *evio;
};

#define VIRTIO_MSG_PCI_MSGS 64
struct virtio_msg_pci_regs {
	__le64 num_msgs;
	__le64 head;
	__le64 tail;
	
	struct virtio_msg msgs[VIRTIO_MSG_PCI_MSGS];
} ____cacheline_aligned;

struct epf_virtio {
	/* Base PCI Endpoint function */
	struct pci_epf *epf;

	/* Virtio parameters */
	u64 features;
	size_t bar_size;
	size_t nvq;
	size_t vqlen;

	/* struct to access virtqueue on remote host */
	struct epf_vringh **vrhs;

	struct epf_vq **vqs;

	/* struct for thread to emulate virtio device */
	struct task_struct *bgtask;

	/* Virtual address of PCI configuration space */
	struct virtio_msg_pci_regs *msg;

	void (*qn_callback)(void *param, u32 index);
	void *qn_param;

	/* Callback function and parameter for initialize complete */
	void (*ic_callback)(void *param);
	void *ic_param;

	u64 (*get_device_config)(struct epf_virtio *evio, u32 offset, u8 size);
	void (*set_device_config)(struct epf_virtio *evio, u32 offset, u64 val, u8 size);

	struct workqueue_struct *dma_wq;
	struct dma_chan *tx_dma_chan, *rx_dma_chan;
	struct work_struct completion_work;
	spinlock_t list_lock;
	struct list_head completion_list;

	u64 cached_head ____cacheline_aligned;
	u64 cached_tail ____cacheline_aligned;
	u32 device_id;
	u32 vendor_id;
	u8 status;

	bool running;
};

struct epf_virtio_dma_transfer
{
	struct epf_virtio *evio;
	struct list_head node;
	dma_addr_t dma_addr;
	enum dma_transfer_direction dir;
	size_t len;
	void (*cb)(struct epf_virtio *evio, void *priv);
	void *priv;
};

#define VIRTIO_MSG_PCI_MSGS 64

#if 0
#define DEFINE_EPF_VIRTIO_CFG_READ(size)                 \
	static inline u##size epf_virtio_cfg_read##size( \
		struct epf_virtio *evio, size_t offset)  \
	{                                                \
		void __iomem *base = evio->bar + offset; \
		return ioread##size(base);               \
	}

DEFINE_EPF_VIRTIO_CFG_READ(8)
DEFINE_EPF_VIRTIO_CFG_READ(16)
DEFINE_EPF_VIRTIO_CFG_READ(32)

#define DEFINE_EPF_VIRTIO_CFG_WRITE(size)                              \
	static inline void epf_virtio_cfg_write##size(                 \
		struct epf_virtio *evio, size_t offset, u##size value) \
	{                                                              \
		void __iomem *base = evio->bar + offset;               \
		iowrite##size(value, base);                            \
	}

DEFINE_EPF_VIRTIO_CFG_WRITE(8);
DEFINE_EPF_VIRTIO_CFG_WRITE(16);
DEFINE_EPF_VIRTIO_CFG_WRITE(32);

#define DEFINE_EPF_VIRTIO_CFG_SET(size)                                \
	static inline void epf_virtio_cfg_set##size(                   \
		struct epf_virtio *evio, size_t offset, u##size value) \
	{                                                              \
		void __iomem *base = evio->bar + offset;               \
		iowrite##size(ioread##size(base) | value, base);       \
	}

DEFINE_EPF_VIRTIO_CFG_SET(8)
DEFINE_EPF_VIRTIO_CFG_SET(16)
DEFINE_EPF_VIRTIO_CFG_SET(32)

#define DEFINE_EPF_VIRTIO_CFG_CLEAR(size)                              \
	static inline void epf_virtio_cfg_clear##size(                 \
		struct epf_virtio *evio, size_t offset, u##size value) \
	{                                                              \
		void __iomem *base = evio->bar + offset;               \
		iowrite##size(ioread##size(base) & ~value, base);      \
	}

DEFINE_EPF_VIRTIO_CFG_CLEAR(8)
DEFINE_EPF_VIRTIO_CFG_CLEAR(16)
DEFINE_EPF_VIRTIO_CFG_CLEAR(32)

static inline void epf_virtio_cfg_memcpy_toio(struct epf_virtio *evio,
					      size_t offset, void *buf,
					      size_t len)
{
	void __iomem *base = evio->bar + offset;

	memcpy_toio(base, buf, len);
}
#endif

int epf_virtio_init(struct epf_virtio *evio, struct pci_epf_header *hdr,
		    size_t bar_size);
void epf_virtio_final(struct epf_virtio *evio);
int epf_virtio_launch_bgtask(struct epf_virtio *evio);
void epf_virtio_terminate_bgtask(struct epf_virtio *evio);
int epf_virtio_reset(struct epf_virtio *evio);

int epf_virtio_getdesc(struct epf_virtio *evio, int index,
		       struct vringh_kiov *riov, struct vringh_kiov *wiov,
		       u16 *head);
void epf_virtio_abandon(struct epf_virtio *evio, int index, int num);
void epf_virtio_iov_complete(struct epf_virtio *evio, int index, u16 head,
			     size_t total_len);

struct dma_chan *epf_request_dma_chan(struct device *dma_dev,
				      enum dma_transfer_direction dir);
void epf_release_dma_chan(struct dma_chan* chan);

int epf_virtio_vq2vq_dma(struct epf_virtio *evio, struct dma_chan *chan, struct vringh_kiov *siov,
			 struct vringh_kiov *diov,
			 enum dma_transfer_direction dir, void (*cb)(struct epf_virtio *evio, void *priv),
			 void *param);

int epf_virtio_vq2vq_memcpy(struct epf_virtio *evio, struct vringh_kiov *siov,
			    struct vringh_kiov *diov,
			    enum dma_transfer_direction dir);
int epf_virtio_setup_edma(struct epf_virtio *evio, struct device *dma_dev);
void epf_virtio_cleanup_edma(struct epf_virtio *evio);

#endif /* __PCI_EPF_VIRTIO_H__ */
