// SPDX-License-Identifier: GPL-2.0
/*
 * Helpers to implement PCIe virtio EP function.
 */
#include <linux/iommu.h>
#include <linux/iopoll.h>
#include <linux/kthread.h>
#include <linux/virtio_config.h>
#include <linux/virtio_pci.h>
#include <linux/vringh.h>
#include <uapi/linux/virtio_msg.h>

#include "pci-epf-virtio.h"

int epf_virtio_vq2vq_memcpy(struct epf_virtio *evio, struct vringh_kiov *siov,
			    struct vringh_kiov *diov,
			    enum dma_transfer_direction dir)
{
	struct iommu_domain *domain = iommu_get_domain_for_dev(evio->epf->epc->dev.parent);
	size_t slen, dlen, len = 0, rem = 0, offset = 0;
	struct pci_epf *epf = evio->epf;
	struct device *dev = &epf->dev;
	u64 sbase, dbase;
	phys_addr_t phys;
	void *dst, *src;

	while (siov->i < siov->used) {
		void *mapped;

		slen = siov->iov[siov->i].iov_len;
		sbase = (u64)siov->iov[siov->i].iov_base;
		dlen = diov->iov[diov->i].iov_len;
		dbase = (u64)diov->iov[diov->i].iov_base;

		if (rem) {
			slen = rem;
			sbase += offset;
		} else
			rem = slen;

		len = min(slen, dlen);
		rem -= len;

		if (dir == DMA_MEM_TO_DEV) {
			src = phys_to_virt(iommu_iova_to_phys(domain, sbase));

			mapped = dst = pci_epc_map_aligned(epf->epc,
							   epf->func_no,
							   epf->vfunc_no, dbase,
							   &phys, len);
		} else {
			mapped = src = pci_epc_map_aligned(epf->epc,
							   epf->func_no,
							   epf->vfunc_no, sbase,
							   &phys, len);
			dst = phys_to_virt(iommu_iova_to_phys(domain, dbase));
		}

		if (IS_ERR(mapped)) {
			dev_err(dev,
				"Failed to map pci memory space to local\n");
			return PTR_ERR(mapped);
		}

		if (dir == DMA_MEM_TO_DEV) {
			memcpy_toio(dst, src, len);
		} else {
			memcpy_fromio(dst, src, len);
		}

		pci_epc_unmap_aligned(epf->epc, epf->func_no, epf->vfunc_no,
				      phys, mapped, len);

		if (rem) {
			diov->i++;
			offset += len;
		} else {
			siov->i++;
			diov->i++;
			offset = 0;
		}
	}

	return 0;
}

struct epf_dma_filter_param {
	struct device *dev;
	u32 dma_mask;
};

static bool epf_dma_filter(struct dma_chan *chan, void *param)
{
	struct epf_dma_filter_param *fparam = param;
	struct dma_slave_caps caps;

	memset(&caps, 0, sizeof(caps));
	dma_get_slave_caps(chan, &caps);

	return chan->device->dev == fparam->dev &&
	       fparam->dma_mask & caps.directions;
}

struct dma_chan *epf_request_dma_chan(struct device *dma_dev,
				      enum dma_transfer_direction dir)
{
	struct epf_dma_filter_param param;
	dma_cap_mask_t mask;

	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE, mask);

	param.dev = dma_dev;
	param.dma_mask = BIT(dir);

	return dma_request_channel(mask, epf_dma_filter, &param);
}

void epf_release_dma_chan(struct dma_chan *chan)
{
	dma_release_channel(chan);
}

int epf_virtio_setup_edma(struct epf_virtio *evio, struct device *dma_dev)
{
	int err;

	evio->tx_dma_chan = epf_request_dma_chan(dma_dev, DMA_MEM_TO_DEV);
	if (!evio->tx_dma_chan)
		return -ENODEV;

	evio->rx_dma_chan = epf_request_dma_chan(dma_dev, DMA_DEV_TO_MEM);
	if (!evio->rx_dma_chan) {
		err = -ENODEV;
		goto err_release_tx_chan;
	}

	return 0;

err_release_tx_chan:
	epf_release_dma_chan(evio->tx_dma_chan);

	return err;
}

void epf_virtio_cleanup_edma(struct epf_virtio *evio)
{
	epf_release_dma_chan(evio->tx_dma_chan);
	epf_release_dma_chan(evio->rx_dma_chan);
}

static void epf_virtio_completion_handler(struct work_struct *work)
{
	struct epf_virtio *evio =
		container_of(work, struct epf_virtio, completion_work);
	struct epf_virtio_dma_transfer *itr, *tmp;
	struct device *dma_dev = evio->epf->epc->dev.parent;
	unsigned long flags;
	LIST_HEAD(head);

	spin_lock_irqsave(&evio->list_lock, flags);
	list_splice_tail_init(&evio->completion_list, &head);
	spin_unlock_irqrestore(&evio->list_lock, flags);

	list_for_each_entry_safe(itr, tmp, &head, node) {
		list_del(&itr->node);

		dma_unmap_single(dma_dev, itr->dma_addr, itr->len, itr->dir);

		if (itr->cb)
			itr->cb(evio, itr->priv);

		kfree(itr);
	}
}

static void epf_virtio_dma_done(void *param)
{
	struct epf_virtio_dma_transfer *transfer = param;
	struct epf_virtio *evio = transfer->evio;

	spin_lock(&evio->list_lock);
	list_add_tail(&transfer->node, &evio->completion_list);
	spin_unlock(&evio->list_lock);

	queue_work(evio->dma_wq, &evio->completion_work);
}

int epf_virtio_vq2vq_dma(struct epf_virtio *evio, struct dma_chan *chan, struct vringh_kiov *siov,
			 struct vringh_kiov *diov,
			 enum dma_transfer_direction dir, void (*cb)(struct epf_virtio *evio, void *priv),
			 void *param)
{
	struct iommu_domain *domain = iommu_get_domain_for_dev(evio->epf->epc->dev.parent);
	struct device *dma_dev = evio->epf->epc->dev.parent;
	size_t slen, dlen, len = 0, rem = 0, offset = 0;
	struct dma_async_tx_descriptor *desc = NULL;
	struct epf_virtio_dma_transfer *transfer;
	DECLARE_COMPLETION_ONSTACK(complete);
	unsigned long flags = 0;
	dma_cookie_t cookie;
	u64 sbase, dbase;
	dma_addr_t dma;
	void *virt;
	int ret;

	while (siov->i < siov->used) {
		slen = siov->iov[siov->i].iov_len;
		sbase = (u64)siov->iov[siov->i].iov_base;
		dlen = diov->iov[diov->i].iov_len;
		dbase = (u64)diov->iov[diov->i].iov_base;

		if (rem) {
			slen = rem;
			sbase += offset;
		} else
			rem = slen;

		len = min(slen, dlen);
		rem -= len;

		flags = DMA_CTRL_ACK | DMA_PREP_INTERRUPT;

		struct dma_slave_config config = {};

		if (dir == DMA_MEM_TO_DEV) {
			config.dst_addr = dbase;
			virt = phys_to_virt(iommu_iova_to_phys(domain, sbase));
		} else {
			config.src_addr = sbase;
			virt = phys_to_virt(iommu_iova_to_phys(domain, dbase));
		}

		if (unlikely(dmaengine_slave_config(chan, &config)))
			return -EINVAL;

		dma = dma_map_single(dma_dev, virt, len, dir);
		ret = dma_mapping_error(dma_dev, dma);
		if (ret) {
			dev_err(chan->slave, "Failed to map remote memory\n");
			return ret;
		}

		desc = dmaengine_prep_slave_single(chan, dma, len, dir, flags);
		if (!desc) {
			dev_err(chan->slave, "failed the preparation of DMA");
			return -EIO;
		}

		transfer = kzalloc(sizeof(*transfer), GFP_KERNEL);
		if (!transfer)
			return -ENOMEM;

		transfer->dma_addr = dma;
		transfer->len = len;
		transfer->dir = dir;
		transfer->evio = evio;
		if (siov->i == siov->used - 1 && !rem) {
			transfer->cb = cb;
			transfer->priv = param;
		}

		desc->callback = epf_virtio_dma_done;
		desc->callback_param = transfer;

		cookie = dmaengine_submit(desc);
		if (unlikely(dma_submit_error(cookie))) {
			kfree(transfer);
			dev_err(chan->slave, "failed the dma submission");
			return cookie;
		}

		if (rem) {
			diov->i++;
			offset += len;
		} else {
			siov->i++;
			diov->i++;
			offset = 0;
		}
	}

	dma_async_issue_pending(chan);

	return 0;
}

static void epf_virtio_unmap_vq(struct pci_epf *epf, void __iomem *vq_virt,
				phys_addr_t vq_phys, unsigned int num)
{
	size_t vq_size = vring_size(num, VIRTIO_PCI_VRING_ALIGN);

	pci_epc_unmap_aligned(epf->epc, epf->func_no, epf->vfunc_no, vq_phys,
			      vq_virt, vq_size);
}

static void __iomem *epf_virtio_map_vq(struct pci_epf *epf,
				       phys_addr_t vq_pci_addr,
				       unsigned int num, phys_addr_t *vq_phys)
{
	size_t vq_size = vring_size(num, VIRTIO_PCI_VRING_ALIGN);

	return pci_epc_map_aligned(epf->epc, epf->func_no, epf->vfunc_no,
				   vq_pci_addr, vq_phys, vq_size);
}

static void epf_virtio_free_vringh(struct pci_epf *epf, struct epf_vringh *evrh)
{
	epf_virtio_unmap_vq(epf, evrh->desc_virt, evrh->desc_phys, evrh->num);
	kfree(evrh);
}

static struct epf_vringh *epf_virtio_alloc_vringh(struct pci_epf *epf,
						  u64 features,
						  phys_addr_t desc,
						  phys_addr_t avail,
						  phys_addr_t used,
						  unsigned int num)
{
	struct epf_vringh *evrh;
	void __iomem *avail_virt, *used_virt;
	int err;

	evrh = kmalloc(sizeof(*evrh), GFP_KERNEL);
	if (!evrh)
		return ERR_PTR(-ENOMEM);

	evrh->num = num;

	evrh->desc_virt = epf_virtio_map_vq(epf, desc, num, &evrh->desc_phys);
	if (IS_ERR(evrh->desc_virt)) {
		err = PTR_ERR(evrh->desc_virt);
		dev_err(&epf->dev, "Failed to map virtqueue descriptor\n");
		goto err_free_evrh;
	}

	/*
	 * epf_virtio_map_vq() has already mapped the avail and used descriptor
	 * regions (thanks to vring_size()!). So we can just manually derive the
	 * virtual address instead of mapping them separately.
	 */
	avail_virt = evrh->desc_virt + (avail - desc);
	used_virt = evrh->desc_virt + (used - desc);

	err = vringh_init_iomem(&evrh->vrh, features, num, false, evrh->desc_virt,
				avail_virt, used_virt);
	if (err)
		goto err_unmap_desc;

	return evrh;

err_unmap_desc:
	epf_virtio_unmap_vq(epf, evrh->desc_virt, evrh->desc_phys, evrh->num);

err_free_evrh:
	kfree(evrh);

	return ERR_PTR(err);
}

#define VIRTIO_PCI_LEGACY_CFG_BAR 0

static void __iomem *epf_virtio_alloc_bar(struct pci_epf *epf, size_t size)
{
	struct pci_epf_bar *config_bar = &epf->bar[VIRTIO_PCI_LEGACY_CFG_BAR];
	const struct pci_epc_features *features;
	void __iomem *bar;
	struct device *dev = &epf->dev;
	int err;

	features = pci_epc_get_features(epf->epc, epf->func_no, epf->vfunc_no);
	if (!features) {
		dev_dbg(dev, "Failed to get PCI EPC features\n");
		return ERR_PTR(-EOPNOTSUPP);
	}

	if (features->bar[VIRTIO_PCI_LEGACY_CFG_BAR].type == BAR_RESERVED) {
		dev_dbg(dev, "Cannot use the PCI BAR for legacy virtio pci\n");
		return ERR_PTR(-EOPNOTSUPP);
	}

	if (features->bar[VIRTIO_PCI_LEGACY_CFG_BAR].type == BAR_FIXED &&
	    size > features->bar[VIRTIO_PCI_LEGACY_CFG_BAR].fixed_size) {
		dev_dbg(dev, "PCI BAR size is not enough\n");
		return ERR_PTR(-ENOMEM);
	}

	bar = pci_epf_alloc_space(epf, size, VIRTIO_PCI_LEGACY_CFG_BAR,
				  features, PRIMARY_INTERFACE);
	if (!bar) {
		dev_dbg(dev, "Failed to allocate virtio-net config memory\n");
		return ERR_PTR(-ENOMEM);
	}

	config_bar->flags |= PCI_BASE_ADDRESS_MEM_TYPE_64;
	err = pci_epc_set_bar(epf->epc, epf->func_no, epf->vfunc_no,
			      config_bar);
	if (err) {
		dev_dbg(dev, "Failed to set PCI BAR");
		goto err_free_space;
	}

	return bar;

err_free_space:

	pci_epf_free_space(epf, bar, VIRTIO_PCI_LEGACY_CFG_BAR,
			   PRIMARY_INTERFACE);

	return ERR_PTR(err);
}

static void epf_virtio_free_bar(struct pci_epf *epf, void __iomem *bar)
{
	struct pci_epf_bar *config_bar = &epf->bar[VIRTIO_PCI_LEGACY_CFG_BAR];

	pci_epc_clear_bar(epf->epc, epf->func_no, epf->vfunc_no, config_bar);
	pci_epf_free_space(epf, bar, VIRTIO_PCI_LEGACY_CFG_BAR,
			   PRIMARY_INTERFACE);
}

#if 0
static void epf_virtio_init_bar(struct epf_virtio *evio, void __iomem *bar)
{
	/* Virtio common cfg lives at offset 0 */
	epf_virtio_cfg_write32(evio, offsetof(struct virtio_pci_common_cfg,
				device_feature), evio->features & 0xffffffff);
	epf_virtio_cfg_write16(evio, offsetof(struct virtio_pci_common_cfg,
				num_queues), 4);
	epf_virtio_cfg_write16(evio, offsetof(struct virtio_pci_common_cfg,
				queue_size), evio->vqlen);
}
#endif

/**
 * epf_virtio_init - initialize struct epf_virtio and setup BAR for virtio
 * @evio: struct epf_virtio to initialize.
 * @hdr: PCI configuration space to show remote host.
 * @bar_size: PCI BAR size it depends on the virtio device type.
 *
 * Returns zero or a negative error.
 */
int epf_virtio_init(struct epf_virtio *evio, struct pci_epf_header *hdr,
		    size_t bar_size)
{
	struct pci_epf *epf = evio->epf;
	void __iomem *bar;
	int err;

	err = pci_epc_set_msi(epf->epc, epf->func_no, epf->vfunc_no,
			      32);
	if (err) {
		pr_err("Failed to set MSI configuration: %d\n", err);
		return err;
	}

	err = pci_epc_write_header(epf->epc, epf->func_no, epf->vfunc_no, hdr);
	if (err)
		return err;

	bar = epf_virtio_alloc_bar(epf, bar_size);
	if (IS_ERR(bar)) /* check err code */
		return PTR_ERR(bar);

	//epf_virtio_init_bar(evio, bar);
	evio->notification = bar;
	evio->msg = bar + 0x1000;
	evio->device_id = hdr->subsys_id;
	evio->vendor_id = hdr->subsys_vendor_id;

	evio->vrhs = kmalloc_array(evio->nvq, sizeof(evio->vrhs[0]), GFP_KERNEL);
	if (!evio->vrhs) {
		err = -ENOMEM;
		goto err_free_bar;
	}

	INIT_LIST_HEAD(&evio->completion_list);

	evio->dma_wq =
		alloc_workqueue("pci-epf-virtio/dma-wq",
				WQ_MEM_RECLAIM | WQ_HIGHPRI | WQ_UNBOUND, 0);
	if (!evio->dma_wq) {
		err = -ENOMEM;
		goto err_free_vrhs;
	}

	INIT_WORK(&evio->completion_work, epf_virtio_completion_handler);
	spin_lock_init(&evio->list_lock);

	return 0;

err_free_vrhs:
	kfree(evio->vrhs);
err_free_bar:
	epf_virtio_free_bar(evio->epf, evio->notification);

	return err;
}
EXPORT_SYMBOL_GPL(epf_virtio_init);

/**
 * epf_virtio_final - finalize struct epf_virtio. it frees BAR and memories
 * @evio: struct epf_virtio to finalize.
 */
void epf_virtio_final(struct epf_virtio *evio)
{
	flush_work(&evio->completion_work);
	destroy_workqueue(evio->dma_wq);

	epf_virtio_free_bar(evio->epf, evio->notification);

	for (int i = 0; i < evio->nvq; i++)
		epf_virtio_free_vringh(evio->epf, evio->vrhs[i]);

	kfree(evio->vrhs);
}
EXPORT_SYMBOL_GPL(epf_virtio_final);

#if 0
static int epf_virtio_negotiate_vq(struct epf_virtio *evio)
{
	int i = 0;
	struct _pair {
		u32 desc_low;
		u32 desc_high;
		u32 avail_low;
		u32 avail_high;
		u32 used_low;
		u32 used_high;
		u16 sel;
		u16 msi_vec;
	} *tmp;
	int err = 0;
	size_t nvq = evio->nvq;

	tmp = kmalloc_array(nvq, sizeof(tmp[0]), GFP_KERNEL);
	if (!tmp)
		return -ENOMEM;

	for (i = 0; i < nvq; i++) {
		/*
		 * TODO: config_generation field is used as a sync point. Fix
		 * it properly.
		 */
		while (!(epf_virtio_cfg_read8(evio,
			offsetof(struct virtio_pci_common_cfg, config_generation))) && evio->running)
			;
		tmp[i].desc_low = epf_virtio_cfg_read32(evio, offsetof(struct virtio_pci_common_cfg, queue_desc_lo));
		tmp[i].desc_high = epf_virtio_cfg_read32(evio, offsetof(struct virtio_pci_common_cfg, queue_desc_hi));
		tmp[i].avail_low = epf_virtio_cfg_read32(evio, offsetof(struct virtio_pci_common_cfg, queue_avail_lo));
		tmp[i].avail_high = epf_virtio_cfg_read32(evio, offsetof(struct virtio_pci_common_cfg, queue_avail_hi));
		tmp[i].used_low = epf_virtio_cfg_read32(evio, offsetof(struct virtio_pci_common_cfg, queue_used_lo));
		tmp[i].used_high = epf_virtio_cfg_read32(evio, offsetof(struct virtio_pci_common_cfg, queue_used_hi));
		tmp[i].msi_vec = epf_virtio_cfg_read16(evio, offsetof(struct virtio_pci_common_cfg, queue_msix_vector));
		epf_virtio_cfg_write8(evio, offsetof(struct virtio_pci_common_cfg, config_generation), 0);
	}

	if (!evio->running)
		goto err_out;

	evio->vrhs = kmalloc_array(nvq, sizeof(evio->vrhs[0]), GFP_KERNEL);
	if (!evio->vrhs) {
		err = -ENOMEM;
		goto err_out;
	}

	for (i = 0; i < nvq; i++) {
		phys_addr_t desc = tmp[i].desc_low | tmp[i].desc_high;
		phys_addr_t avail = tmp[i].avail_low | tmp[i].avail_high;
		phys_addr_t used = tmp[i].used_low | tmp[i].used_high;

		evio->vrhs[i] = epf_virtio_alloc_vringh(
			evio->epf, evio->features, desc, avail, used, evio->vqlen);
		if (IS_ERR(evio->vrhs[i])) {
			err = PTR_ERR(evio->vrhs[i]);
			goto err_free_evrhs;
		}

		evio->vrhs[i]->msi_vec = tmp[i].msi_vec;
		evio->vrhs[i]->queue_sel = tmp[i].sel;
	}

	kfree(tmp);

	return 0;

err_free_evrhs:
	for (i -= 1; i > 0; i--)
		epf_virtio_free_vringh(evio->epf, evio->vrhs[i]);

	kfree(evio->vrhs);

err_out:
	kfree(tmp);

	return err;
}

#define EPF_VIRTIO_VQ_CALLBACK(n)				\
static void epf_virtio_vq_callback##n(struct kthread_work *work)	\
{								\
	struct epf_vq *vq =					\
		container_of(work, struct epf_vq, work);	\
	struct epf_virtio *evio = vq->evio;			\
	const u16 qn_default = evio->nvq;			\
	u16 tmp;						\
								\
	while (evio->running) {					\
		tmp = epf_virtio_cfg_read16(evio, 0x3000 + (n * 4));	\
		if (tmp == qn_default)					\
			continue;					\
									\
		epf_virtio_cfg_write16(evio, 0x3000 + (n * 4), qn_default); \
									\
		evio->qn_callback(evio->qn_param, tmp);			\
	}								\
}

EPF_VIRTIO_VQ_CALLBACK(0)
EPF_VIRTIO_VQ_CALLBACK(1)
EPF_VIRTIO_VQ_CALLBACK(2)

static void epf_virtio_negotiate_features(struct epf_virtio *evio)
{
	u32 feature_offset = offsetof(struct virtio_pci_common_cfg, device_feature);

	/*
	 * Host will first read the feature bits 0-31 and then it will write 1
	 * to 'device_feature_select' to read the feature bits 32-63.
	 *
	 * TODO: config_generation field is used as a sync point. Fix
	 * it properly.
	 */
	while (!(epf_virtio_cfg_read8(evio,
		offsetof(struct virtio_pci_common_cfg, config_generation))) && evio->running)
		;
	epf_virtio_cfg_write32(evio, feature_offset, evio->features >> 32);
	epf_virtio_cfg_write8(evio, offsetof(struct virtio_pci_common_cfg, config_generation), 0);
}

static struct epf_vq *epf_virtio_alloc_vqs(struct epf_virtio *evio, int idx, void (*fn)(struct kthread_work *))
{
	struct epf_vq *vq;
	int err;

	vq = kmalloc(sizeof(*vq), GFP_KERNEL);
	if (!vq)
		return ERR_PTR(-ENOMEM);

	vq->evio = evio;

	vq->worker = kthread_create_worker(0, "pci-epf-virtio/queue%d", idx);
	if (IS_ERR(vq->worker)) {
		err = PTR_ERR(vq->worker);
		goto err_free_vqs;
	}

	kthread_init_work(&vq->work, fn);
	kthread_queue_work(vq->worker, &vq->work);

	return vq;

err_free_vqs:
	kfree(vq);

	return ERR_PTR(err);
}

static void epf_virtio_free_vqs(struct epf_virtio *evio, int idx)
{
	struct epf_vq *vq = evio->vqs[idx];

	kthread_destroy_worker(vq->worker);
	kfree(vq);
}

static int epf_virtio_bgtask(void *param)
{
	static void (*func[])(struct kthread_work *) = {
		epf_virtio_vq_callback0,
		epf_virtio_vq_callback1,
		epf_virtio_vq_callback2,
	};
	struct epf_virtio *evio = param;
	struct device *dev = &evio->epf->dev;
	int err, i;

	epf_virtio_negotiate_features(evio);

	err = epf_virtio_negotiate_vq(evio);
	if (err < 0) {
		dev_err(dev, "Failed to negotiate configs with driver\n");
		return err;
	}

	while (!(epf_virtio_cfg_read8(evio, offsetof(struct virtio_pci_common_cfg, device_status)) &
		 VIRTIO_CONFIG_S_DRIVER_OK) &&
	       evio->running)
		;

	if (evio->ic_callback && evio->running)
		evio->ic_callback(evio->ic_param);

	evio->vqs = kmalloc_array(evio->nvq, sizeof(evio->vqs[0]), GFP_KERNEL);
	if (!evio->vqs)
		return -ENOMEM;

	for (i = 0; i < evio->nvq; i++)
	{
		evio->vqs[i] = epf_virtio_alloc_vqs(evio, i, func[i]);
		if (!evio->vqs[i])
			goto err_free_vqs;
	}

	return 0;

err_free_vqs:
	for (--i; i >= 0; i--)
		epf_virtio_free_vqs(evio, i);
	kfree(evio->vqs);

	return err;
}
#endif

static int epf_virtio_handle_set_vqueue(struct epf_virtio *evio, struct virtio_msg *request)
{
	u64 size, desc, avail, used;
	u32 index;

	index = le32_to_cpu(request->set_vqueue.index);
	size = le32_to_cpu(request->set_vqueue.size);
	desc = le32_to_cpu(request->set_vqueue.descriptor_addr);
	avail = le32_to_cpu(request->set_vqueue.driver_addr);
	used = le32_to_cpu(request->set_vqueue.device_addr);

	evio->vrhs[index] = epf_virtio_alloc_vringh(evio->epf, evio->features,
						    desc, avail, used, size);
	if (IS_ERR(evio->vrhs[index]))
		return PTR_ERR(evio->vrhs[index]);

	evio->vrhs[index]->msi_vec = index;

	/* Signal init complete if last vq is configured */
	if (index == evio->nvq - 1) {
		if (evio->ic_callback && evio->running)
			evio->ic_callback(evio->ic_param);
	}

	return 0;
}

static int epf_virtio_handle_notification(struct epf_virtio *evio, struct virtio_msg *request)
{
	u32 index;

	index = le32_to_cpu(request->event_avail.index);

	evio->qn_callback(evio->qn_param, index);

	return 0;	
}

static void epf_virtio_handle_get_config(struct epf_virtio *evio, struct virtio_msg *request)
{
	u32 offset;

	offset = request->get_config.offset[0] | request->get_config.offset[1] << 8 |
			request->get_config.offset[2] << 16;

	if (evio->get_device_config)
		request->get_config_resp.data[0] = cpu_to_le64(evio->get_device_config(evio,
							offset, request->get_config.size));
}

static void epf_virtio_handle_set_config(struct epf_virtio *evio, struct virtio_msg *request)
{
	u32 offset;

	offset = request->get_config.offset[0] | request->get_config.offset[1] << 8 |
			request->get_config.offset[2] << 16;

	if (evio->set_device_config)
		evio->set_device_config(evio, offset, le64_to_cpu(request->set_config.data[0]),
				request->set_config.size);
}

static int epf_virtio_process_message(struct epf_virtio *evio, struct virtio_msg *request)
{
	int ret;

	switch (request->id) {
	case VIRTIO_MSG_DEVICE_INFO:
		request->get_device_info_resp.device_id = cpu_to_le32(evio->device_id);
		request->get_device_info_resp.vendor_id = cpu_to_le32(evio->vendor_id);
		break;
	case VIRTIO_MSG_GET_FEATURES:
		request->get_features_resp.features[0] = cpu_to_le64(evio->features);
		break;
	case VIRTIO_MSG_GET_VQUEUE:
		/* Same queue size for all queues */
		request->get_vqueue_resp.max_size = cpu_to_le32(evio->vqlen);	
		break;
	case VIRTIO_MSG_SET_VQUEUE:
		ret = epf_virtio_handle_set_vqueue(evio, request);
		if (ret)
			return ret;
		break;
	case VIRTIO_MSG_GET_CONFIG:
		/* TODO: return err if callback not present? */
		epf_virtio_handle_get_config(evio, request);
		break;
	case VIRTIO_MSG_SET_CONFIG:
		epf_virtio_handle_set_config(evio, request);
		break;
	case VIRTIO_MSG_EVENT_AVAIL:
		ret = epf_virtio_handle_notification(evio, request);
		if (ret)
			return ret;
		break;
	case VIRTIO_MSG_SET_DEVICE_STATUS:
		evio->status = le32_to_cpu(request->set_device_status.status);
		break;
	case VIRTIO_MSG_GET_DEVICE_STATUS:
		request->get_device_status_resp.status = cpu_to_le32(evio->status);
		break;
	default:
		/* TODO */
		break;
	}

	return 0;
}

static int epf_virtio_bgtask(void *param)
{
	struct epf_virtio *evio = param;
	u32 entry, val;
	u64 msg_bitmap;
	int ret;

	while (1) {
		readl_poll_timeout(evio->notification, val, val == 1, 0, 0);

		do {
			struct virtio_msg request;

			/* Update the driver bitmap */
			evio->driver_bitmap = readq(&evio->msg->driver_bitmap);
			msg_bitmap = evio->driver_bitmap ^ evio->device_bitmap;
			entry = ffs(msg_bitmap);
			if (!entry) {
				pr_err("%s: No entry available\n", __func__);
				continue; /* TODO */
			}
			entry--;

			/* Fetch the message */
			memcpy_fromio(&request, &evio->msg->msgs[entry], sizeof(struct virtio_msg));

			ret = epf_virtio_process_message(evio, &request);
			if (ret)
				return ret;

			if (request.type != VIRTIO_MSG_TYPE_RESPONSE) {
				/* Send response in the same message */
				memcpy_toio(&evio->msg->msgs[entry], &request, sizeof(struct virtio_msg));
			}

			/*
			 * Ensure that the messages are read before updating the
			 * entry.
			 */
			dma_rmb();

			/* Update the device bitmap */
			evio->device_bitmap = evio->device_bitmap ^ BIT(entry);
			writeq(evio->device_bitmap, &evio->msg->device_bitmap);
	
		} while(entry);

		/* Clear notification */
		writel(0, evio->notification);
	}

	return 0;
}

/**
 * epf_virtio_launch_bgtask - spawn a kthread that emulates virtio device
 * operations.
 * @evio: It should be initialized prior with epf_virtio_init().
 *
 * Returns zero or a negative error.
 */
int epf_virtio_launch_bgtask(struct epf_virtio *evio)
{
	evio->bgtask = kthread_create(epf_virtio_bgtask, evio,
				      "pci-epf-virtio/bgtask");
	if (IS_ERR(evio->bgtask))
		return PTR_ERR(evio->bgtask);

	evio->running = true;

	sched_set_fifo(evio->bgtask);
	wake_up_process(evio->bgtask);

	return 0;
}
EXPORT_SYMBOL_GPL(epf_virtio_launch_bgtask);

/**
 * epf_virtio_terminate_bgtask - shutdown a device emulation kthread.
 * @evio: struct epf_virtio it already launched bgtask.
 */
void epf_virtio_terminate_bgtask(struct epf_virtio *evio)
{
	evio->running = false;

	kthread_stop(evio->bgtask);
}
EXPORT_SYMBOL_GPL(epf_virtio_terminate_bgtask);

/**
 * epf_virtio_reset - reset virtio status
 * @evio: struct epf_virtio to reset
 *
 * Returns zero or a negative error.
 */
int epf_virtio_reset(struct epf_virtio *evio)
{
	epf_virtio_terminate_bgtask(evio);
//	epf_virtio_init_bar(evio, evio->bar);

	return epf_virtio_launch_bgtask(evio);
}
EXPORT_SYMBOL_GPL(epf_virtio_reset);

int epf_virtio_getdesc(struct epf_virtio *evio, int index,
		       struct vringh_kiov *riov, struct vringh_kiov *wiov,
		       u16 *head)
{
	struct vringh *vrh = &evio->vrhs[index]->vrh;

	return vringh_getdesc_iomem(vrh, riov, wiov, head, GFP_KERNEL);
}

void epf_virtio_abandon(struct epf_virtio *evio, int index, int num)
{
	struct vringh *vrh = &evio->vrhs[index]->vrh;

	vringh_abandon_iomem(vrh, num);
}

void epf_virtio_iov_complete(struct epf_virtio *evio, int index, u16 head,
			     size_t total_len)
{
	struct vringh *vrh = &evio->vrhs[index]->vrh;

	vringh_complete_iomem(vrh, head, total_len);
}

MODULE_LICENSE("GPL");
