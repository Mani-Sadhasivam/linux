// SPDX-License-Identifier: GPL-2.0
/*
 * PCI Endpoint function driver to impliment virtio-net device.
 */

#include <linux/iommu.h>
#include <linux/module.h>
#include <linux/pci-epf.h>
#include <linux/virtio_config.h>
#include <linux/virtio_net.h>
#include <linux/virtio_pci.h>
#include <linux/virtio_ring.h>
#include <uapi/linux/virtio_msg.h>

#include "pci-epf-virtio.h"

static int virtio_queue_size = 0x1000;
module_param(virtio_queue_size, int, 0444);
MODULE_PARM_DESC(virtio_queue_size, "A length of virtqueue");

struct epf_vnet {
	/* virtio feature and configurations for virtio-net. It is commonly used
	 * local and remote. */
	struct virtio_net_config vnet_cfg;
	struct virtio_net_config vdev_vnet_cfg;
	u64 features;

	/* To access virtqueues on remote host */
	struct epf_virtio evio;
	struct vringh_kiov *rdev_iovs;

	/* To register a local virtio bus */
	struct virtio_device vdev;

	/* To access virtqueus of local host driver */
	struct vringh *vdev_vrhs;
	struct vringh_kiov *vdev_iovs;
	struct virtqueue **vdev_vqs;

	const struct pci_epc_features *epc_features;
	struct workqueue_struct *task_tx_wq, *task_rx_wq;
	struct work_struct tx_work, rx_work;
	struct work_struct vdev_ctrl_work, ep_ctrl_work;

	bool enable_edma;

#define EPF_VNET_INIT_COMPLETE_VDEV BIT(0)
#define EPF_VNET_INIT_COMPLETE_EP_FUNC BIT(1)
	u8 init_state;
	u8 vdev_status;
};

struct epf_vnet_done_cb_param {
	size_t total_len;
	unsigned int local_vq_index, remote_vq_index;
	u16 local_head, remote_head;
	enum dma_transfer_direction dir;
	struct epf_vnet *vnet;
};

static inline struct epf_vnet *vdev_to_vnet(struct virtio_device *vdev)
{
	return container_of(vdev, struct epf_vnet, vdev);
}

static u16 epf_vnet_get_nvq(struct epf_vnet *vnet)
{
	/* tx and rx queue pair and control queue. */
	return vnet->vnet_cfg.max_virtqueue_pairs * 2 +
	       !!(vnet->features & BIT(VIRTIO_NET_F_CTRL_VQ));
}

static void epf_vnet_vdev_announce_linkup(struct epf_vnet *vnet);
static void epf_vnet_qnotify_callback(void *param, u32 index);

static void epf_vnet_ep_announce_linkup(struct epf_vnet *vnet)
{
	struct epf_virtio *evio = &vnet->evio;
	struct pci_epf *epf = evio->epf;

#if 0
	epf_virtio_cfg_set16(evio,
			     0x2000 +
				     offsetof(struct virtio_net_config, status),
			     VIRTIO_NET_S_LINK_UP | VIRTIO_NET_S_ANNOUNCE);
	epf_virtio_cfg_set16(evio, 0x1000 + VIRTIO_PCI_ISR, VIRTIO_PCI_ISR_CONFIG);
#endif

//	evio->status = VIRTIO_NET_S_LINK_UP | VIRTIO_NET_S_ANNOUNCE;
	evio->status = VIRTIO_NET_S_LINK_UP;

	/* MSI vector for config vq is fixed to 0 */
	pci_epc_raise_irq(epf->epc, epf->func_no, epf->vfunc_no, PCI_IRQ_MSI,
			  1);
}

static bool __epf_vnet_init_complete(struct epf_vnet *vnet)
{
	if ((vnet->init_state & EPF_VNET_INIT_COMPLETE_VDEV) &&
	    (vnet->init_state & EPF_VNET_INIT_COMPLETE_EP_FUNC))
		return true;

	return false;
}

static void epf_vnet_init_complete(struct epf_vnet *vnet, u8 from)
{
	vnet->init_state |= from;

	if (!__epf_vnet_init_complete(vnet))
		return;

	epf_vnet_vdev_announce_linkup(vnet);
	epf_vnet_ep_announce_linkup(vnet);
}

static void epf_vnet_ep_init_complete(void *param)
{
	struct epf_vnet *vnet = param;

	epf_vnet_init_complete(vnet, EPF_VNET_INIT_COMPLETE_EP_FUNC);
}

static u64 epf_vnet_get_device_config(struct epf_virtio *evio, u32 offset, u8 size)
{
	struct epf_vnet *vnet = container_of(evio, struct epf_vnet, evio);

	if (offset == offsetof(struct virtio_net_config, status)) {
		if (__epf_vnet_init_complete(vnet))
			return VIRTIO_NET_S_LINK_UP;
	}

	return 0;
}

static void epf_vnet_complete_xfer(struct epf_virtio *evio, void *priv)
{
	struct epf_vnet *vnet = container_of(evio, struct epf_vnet, evio);
	struct epf_vnet_done_cb_param *cb_param = priv;
	struct vringh *vrh = &vnet->vdev_vrhs[cb_param->local_vq_index];
	struct virtqueue *vq = vnet->vdev_vqs[cb_param->local_vq_index];
//	u16 msi_vec = evio->vrhs[cb_param->remote_vq_index]->msi_vec;
	struct pci_epf *epf = evio->epf;


	epf_virtio_iov_complete(evio, cb_param->remote_vq_index, cb_param->remote_head,
	 	 cb_param->total_len);
	vringh_complete_kern(vrh, cb_param->local_head, cb_param->total_len);

	/*
	 * Since the virtqueue callbacks of virtio-net schedules NAPI
	 * processing in SoftIRQ context, we need to explicitly disable and
	 * enable BH to trigger any pending SoftIRQs.
	 */
	local_bh_disable();
	vring_interrupt(0, vq);
	local_bh_enable();

	pci_epc_raise_irq(epf->epc, epf->func_no, epf->vfunc_no, PCI_IRQ_MSI,
			  2); /* FIXME: MSI */

	kfree(cb_param);
}

static struct pci_epf_header epf_vnet_pci_header = {
	.vendorid = PCI_VENDOR_ID_QCOM,
	.deviceid = 0x0306, /* FIXME */
	.subsys_vendor_id = PCI_VENDOR_ID_QCOM,
	.subsys_id = VIRTIO_ID_NET,
	.revid = 0,
	.baseclass_code = PCI_BASE_CLASS_NETWORK,
	.interrupt_pin = PCI_INTERRUPT_PIN,
};

#if 0
static void epf_vnet_setup_pci_cfgs(struct epf_vnet *vnet,
				    struct epf_virtio *evio)
{
	epf_virtio_cfg_memcpy_toio(evio, 0x2000,
				   &vnet->vnet_cfg, sizeof(vnet->vnet_cfg));
}
#endif

static int epf_vnet_setup_ep_func(struct epf_vnet *vnet, struct pci_epf *epf)
{
	struct epf_virtio *evio = &vnet->evio;
	u16 nvq = epf_vnet_get_nvq(vnet);
	int err;

	vnet->rdev_iovs =
		kmalloc_array(sizeof(vnet->rdev_iovs[0]), nvq, GFP_KERNEL);
	if (!vnet->rdev_iovs)
		return -ENOMEM;

	for (int i = 0; i < nvq; i++)
		vringh_kiov_init(&vnet->rdev_iovs[i], NULL, 0);

	evio->epf = epf;
	evio->features = vnet->features;
	evio->nvq = nvq;
	evio->vqlen = virtio_queue_size;

	evio->qn_callback = epf_vnet_qnotify_callback;
	evio->qn_param = vnet;

	evio->ic_callback = epf_vnet_ep_init_complete;
	evio->ic_param = vnet;

	evio->get_device_config = epf_vnet_get_device_config;

	err = epf_virtio_init(evio, &epf_vnet_pci_header, SZ_16K);
	if (err)
		goto err_cleanup_kiov;

	//epf_vnet_setup_pci_cfgs(vnet, evio);

	return 0;

err_cleanup_kiov:
	kfree(vnet->rdev_iovs);

	return err;
}

static void epf_vnet_cleanup_ep_func(struct epf_vnet *vnet)
{
	epf_virtio_terminate_bgtask(&vnet->evio);

	epf_virtio_final(&vnet->evio);

	kfree(vnet->rdev_iovs);
}

enum {
	VNET_VIRTQUEUE_RX,
	VNET_VIRTQUEUE_TX,
	VNET_VIRTQUEUE_CTRL,
};

static struct epf_vnet_done_cb_param *
epf_virtio_create_cb_param(struct epf_vnet *vnet, enum dma_transfer_direction dir, size_t total_len,
			     struct vringh *vrh, struct epf_virtio *evio,
			     u16 local_head, u16 remote_head)
{
	struct epf_vnet_done_cb_param *cb_param;
	unsigned int local_vq_index, remote_vq_index;

	if (dir == DMA_MEM_TO_DEV) {
		local_vq_index = VNET_VIRTQUEUE_TX;
		remote_vq_index = VNET_VIRTQUEUE_RX;
	} else {
		local_vq_index = VNET_VIRTQUEUE_RX;
		remote_vq_index = VNET_VIRTQUEUE_TX;
	}

	cb_param = kmalloc(sizeof(*cb_param), GFP_KERNEL);
	if (!cb_param)
		return ERR_PTR(-ENOMEM);

	*cb_param = (struct epf_vnet_done_cb_param){
		.vnet = vnet,
		.total_len = total_len,
		.local_vq_index = local_vq_index,
		.remote_vq_index = remote_vq_index,
		.local_head = local_head,
		.remote_head = remote_head,
		.dir = dir,
	};

	return cb_param;
}

static void epf_vnet_rx_handler(struct work_struct *work)
{
	struct epf_vnet *vnet =
		container_of(work, struct epf_vnet, rx_work);
	struct epf_virtio *evio = &vnet->evio;
	struct vringh *dvrh;
	struct vringh_kiov *siov, *diov;
	int ret;

	dvrh = &vnet->vdev_vrhs[VNET_VIRTQUEUE_RX];
	siov = &vnet->rdev_iovs[VNET_VIRTQUEUE_TX];
	diov = &vnet->vdev_iovs[VNET_VIRTQUEUE_RX];

	do {
		u16 shead, dhead;
		size_t total_len;
		struct epf_vnet_done_cb_param *cb_param;

		ret = epf_virtio_getdesc(evio, VNET_VIRTQUEUE_TX, siov, NULL,
					 &shead);
		if (ret <= 0) {
			break;
		}

		ret = vringh_getdesc_kern(dvrh, NULL, diov, &dhead, GFP_KERNEL);
		if (ret <= 0) {
			epf_virtio_abandon(evio, VNET_VIRTQUEUE_TX, 1);
			break;
		}

		total_len = vringh_kiov_length(siov);

		cb_param = epf_virtio_create_cb_param(vnet, DMA_DEV_TO_MEM, total_len, dvrh, evio,
						dhead, shead);
		if (IS_ERR(cb_param)) {
			dev_err(&evio->epf->dev, "failed to setup dma cb param: %ld",
				PTR_ERR(cb_param));
		}

		if (vnet->enable_edma) {
			ret = epf_virtio_vq2vq_dma(evio, evio->rx_dma_chan, siov,
						   diov, DMA_DEV_TO_MEM, epf_vnet_complete_xfer,
						   cb_param);
			if (ret)
				dev_err(&evio->epf->dev, "failed the DMA: %d", ret);

		} else {
			ret = epf_virtio_vq2vq_memcpy(evio, siov, diov, DMA_DEV_TO_MEM);
			if (ret) {
				dev_err(&evio->epf->dev, "failed the cpu transfer: %d", ret);
				epf_virtio_abandon(evio, VNET_VIRTQUEUE_TX, 1);
				break;
			}

			epf_vnet_complete_xfer(evio, cb_param);
		}
	} while (ret >= 0);
}

static void epf_vnet_tx_handler(struct work_struct *work)
{
	struct epf_vnet *vnet = container_of(work, struct epf_vnet, tx_work);
	struct epf_virtio *evio = &vnet->evio;
	struct vringh *svrh;
	struct vringh_kiov *siov, *diov;
	int ret;

	svrh = &vnet->vdev_vrhs[VNET_VIRTQUEUE_TX];
	siov = &vnet->vdev_iovs[VNET_VIRTQUEUE_TX];
	diov = &vnet->rdev_iovs[VNET_VIRTQUEUE_RX];

	do {
		u16 shead, dhead;
		size_t total_len;
		struct epf_vnet_done_cb_param *cb_param;

		ret = vringh_getdesc_kern(svrh, siov, NULL, &shead, GFP_KERNEL);
		if (ret <= 0)
			break;

		ret = epf_virtio_getdesc(evio, VNET_VIRTQUEUE_RX, NULL, diov,
					 &dhead);
		if (ret <= 0) {
			vringh_abandon_kern(svrh, 1);
			break;
		}

		total_len = vringh_kiov_length(siov);

		cb_param = epf_virtio_create_cb_param(vnet, DMA_MEM_TO_DEV, total_len, svrh, evio,
							shead, dhead);
		if (IS_ERR(cb_param)) {
			dev_err(&evio->epf->dev, "failed to setup dma cb param: %ld",
				PTR_ERR(cb_param));
		}

		if (vnet->enable_edma) {
			ret = epf_virtio_vq2vq_dma(evio, evio->tx_dma_chan, siov,
						   diov, DMA_MEM_TO_DEV, epf_vnet_complete_xfer,
						   cb_param);
			if (ret)
				dev_err(&evio->epf->dev, "failed the DMA: %d", ret);

		} else {
			ret = epf_virtio_vq2vq_memcpy(evio, siov, diov,
						DMA_MEM_TO_DEV);
			if (ret) {
				dev_err(&evio->epf->dev, "failed the cpu transfer: %d", ret);
				vringh_abandon_kern(svrh, 1);
				break;
			}

			epf_vnet_complete_xfer(evio, cb_param);
		}
	} while (ret >= 0);
}

static void epf_vnet_ep_ctrl_handler(struct work_struct *work)
{
	struct epf_vnet *vnet =
		container_of(work, struct epf_vnet, ep_ctrl_work);
	struct epf_virtio *evio = &vnet->evio;
	struct vringh_kiov riov, wiov;
	struct vringh *vrh = &evio->vrhs[VNET_VIRTQUEUE_CTRL]->vrh;
	struct pci_epf *epf = evio->epf;
	struct virtio_net_ctrl_hdr *hdr;
	int err;
	u16 head;
	size_t total_len, rlen, wlen;
	u8 class, cmd;
	void __iomem *rvirt, *wvirt;
	phys_addr_t rphys, wphys;

	vringh_kiov_init(&riov, NULL, 0);
	vringh_kiov_init(&wiov, NULL, 0);

	err = vringh_getdesc_iomem(vrh, &riov, &wiov, &head, GFP_KERNEL);
	if (err <= 0)
		return;

	total_len = vringh_kiov_length(&riov);

	rlen = riov.iov[riov.i].iov_len;
	rvirt = pci_epc_map_aligned(epf->epc, epf->func_no, epf->vfunc_no,
				    (u64)riov.iov[riov.i].iov_base, &rphys,
				    rlen);
	if (IS_ERR(rvirt)) {
		err = PTR_ERR(rvirt);
		goto err_out;
	}

	wlen = wiov.iov[wiov.i].iov_len;
	wvirt = pci_epc_map_aligned(epf->epc, epf->func_no, epf->vfunc_no,
				    (u64)wiov.iov[wiov.i].iov_base, &wphys,
				    wlen);
	if (IS_ERR(wvirt)) {
		err = PTR_ERR(wvirt);
		goto err_unmap_command;
	}

	hdr = rvirt;
	class = ioread8(&hdr->class);
	cmd = ioread8(&hdr->cmd);
	switch (class) {
	case VIRTIO_NET_CTRL_ANNOUNCE:
		if (cmd != VIRTIO_NET_CTRL_ANNOUNCE_ACK) {
			pr_err("Found invalid command: announce: %d\n", cmd);
			break;
		}
#if 0
		epf_virtio_cfg_clear16(
			evio,
			0x2000 +
				offsetof(struct virtio_net_config, status),
			VIRTIO_NET_S_ANNOUNCE);
		epf_virtio_cfg_clear16(evio, 0x1000 + VIRTIO_PCI_ISR,
				       VIRTIO_PCI_ISR_CONFIG); /* TODO: Not needed for MSI */

#endif
		iowrite8(VIRTIO_NET_OK, wvirt);
//		evio->status = VIRTIO_NET_OK;
		/* MSI vector for config vq is fixed to 0 */
//		pci_epc_raise_irq(epf->epc, epf->func_no, epf->vfunc_no, PCI_IRQ_MSI,
//				  1);
		break;
	default:
		pr_err("Found unsupported class in control queue: %d\n", class);
		break;
	}

	vringh_complete_iomem(vrh, head, total_len);
	pci_epc_unmap_aligned(epf->epc, epf->func_no, epf->vfunc_no, rphys,
			      rvirt, rlen);
	pci_epc_unmap_aligned(epf->epc, epf->func_no, epf->vfunc_no, wphys,
			      wvirt, wlen);

	vringh_kiov_cleanup(&riov);
	vringh_kiov_cleanup(&wiov);

	return;

err_unmap_command:
	pci_epc_unmap_aligned(epf->epc, epf->func_no, epf->vfunc_no, rphys,
			      rvirt, rlen);
err_out:
	return;
}

static void epf_vnet_qnotify_callback(void *param, u32 index)
{
	struct epf_vnet *vnet = param;

	switch (index) {
	case VNET_VIRTQUEUE_RX:
		queue_work(vnet->task_tx_wq, &vnet->tx_work);
		break;
	case VNET_VIRTQUEUE_TX:
		queue_work(vnet->task_rx_wq, &vnet->rx_work);
		break;
	case VNET_VIRTQUEUE_CTRL:
		queue_work(vnet->task_tx_wq, &vnet->ep_ctrl_work);
		break;
	default:
		break;
	}
}

static void epf_vnet_vdev_cfg_set_status(struct epf_vnet *vnet, u16 status)
{
	vnet->vdev_vnet_cfg.status |= status;
}

static void epf_vnet_vdev_cfg_clear_status(struct epf_vnet *vnet, u16 status)
{
	vnet->vdev_vnet_cfg.status &= ~status;
}

static void epf_vnet_vdev_announce_linkup(struct epf_vnet *vnet)
{
	epf_vnet_vdev_cfg_set_status(vnet, VIRTIO_NET_S_LINK_UP |
						   VIRTIO_NET_S_ANNOUNCE);
	virtio_config_changed(&vnet->vdev);
}

static void epf_vnet_vdev_ctrl_handler(struct work_struct *work)
{
	struct epf_vnet *vnet =
		container_of(work, struct epf_vnet, vdev_ctrl_work);
	struct vringh *vrh = &vnet->vdev_vrhs[VNET_VIRTQUEUE_CTRL];
	struct vringh_kiov riov, wiov;
	struct virtio_net_ctrl_hdr *hdr;
	struct iommu_domain *domain = iommu_get_domain_for_dev(vnet->vdev.dev.parent);
	virtio_net_ctrl_ack *ack;
	int err;
	u16 head;
	size_t len;

	vringh_kiov_init(&riov, NULL, 0);
	vringh_kiov_init(&wiov, NULL, 0);

	err = vringh_getdesc_kern(vrh, &riov, &wiov, &head, GFP_KERNEL);
	if (err <= 0)
		return;

	len = vringh_kiov_length(&riov);
	if (len < sizeof(*hdr)) {
		pr_debug("Command is too short: %ld\n", len);
		err = -EIO;
		goto done;
	}

	if (vringh_kiov_length(&wiov) < sizeof(*ack)) {
		pr_debug("Space for ack is not enough\n");
		err = -EIO;
		goto done;
	}

	hdr = phys_to_virt(iommu_iova_to_phys(domain, (unsigned long)riov.iov[riov.i].iov_base));
	ack = phys_to_virt(iommu_iova_to_phys(domain, (unsigned long)wiov.iov[wiov.i].iov_base));

	switch (hdr->class) {
	case VIRTIO_NET_CTRL_ANNOUNCE:
		if (hdr->cmd != VIRTIO_NET_CTRL_ANNOUNCE_ACK) {
			pr_debug("Invalid command: announce: %d\n", hdr->cmd);
			goto done;
		}

		epf_vnet_vdev_cfg_clear_status(vnet, VIRTIO_NET_S_ANNOUNCE);
		*ack = VIRTIO_NET_OK;
		break;
	default:
		pr_info("Class (%d) not supported\n", hdr->class);
		err = -EIO;
	}

done:
	vringh_complete_kern(vrh, head, len);

	vringh_kiov_cleanup(&riov);
	vringh_kiov_cleanup(&wiov);
	return;
}

static int epf_vnet_setup_common(struct epf_vnet *vnet)
{
	vnet->features =
		BIT(VIRTIO_F_ACCESS_PLATFORM) | BIT(VIRTIO_NET_F_STATUS) |
		BIT(VIRTIO_F_VERSION_1) |
		/* Following features are to skip any of checking and offloading, Like a
		 * transmission between virtual machines on same system. Details are on
		 * section 5.1.5 in virtio specification.
		 */
		BIT(VIRTIO_NET_F_GUEST_CSUM) | BIT(VIRTIO_NET_F_GUEST_TSO4) |
		BIT(VIRTIO_NET_F_GUEST_TSO6) | BIT(VIRTIO_NET_F_GUEST_ECN) |
		BIT(VIRTIO_NET_F_GUEST_UFO) |
		/* The control queue is just used for linkup announcement. */
		BIT(VIRTIO_NET_F_CTRL_VQ);

	// Currently support only one vq pair(tx/rx).
	vnet->vnet_cfg.max_virtqueue_pairs = 1;
	vnet->vnet_cfg.status = 0;

	memcpy(&vnet->vdev_vnet_cfg, &vnet->vnet_cfg, sizeof(vnet->vnet_cfg));

	vnet->task_tx_wq =
		alloc_workqueue("pci-epf-vnet/task-wq-tx",
				WQ_MEM_RECLAIM | WQ_HIGHPRI | WQ_UNBOUND, 0);
	if (!vnet->task_tx_wq)
		return -ENOMEM;

	vnet->task_rx_wq =
		alloc_workqueue("pci-epf-vnet/task-wq-rx",
				WQ_MEM_RECLAIM | WQ_HIGHPRI | WQ_UNBOUND, 0);
	if (!vnet->task_rx_wq)
		return -ENOMEM;

	INIT_WORK(&vnet->rx_work, epf_vnet_rx_handler);
	INIT_WORK(&vnet->tx_work, epf_vnet_tx_handler);
	INIT_WORK(&vnet->ep_ctrl_work, epf_vnet_ep_ctrl_handler);
	INIT_WORK(&vnet->vdev_ctrl_work, epf_vnet_vdev_ctrl_handler);

	return 0;
}

static void epf_vnet_cleanup_common(struct epf_vnet *vnet)
{
	flush_work(&vnet->tx_work);
	flush_work(&vnet->rx_work);

	destroy_workqueue(vnet->task_tx_wq);
	destroy_workqueue(vnet->task_rx_wq);
}

/*
 * Functions for local virtio device operation
 */
static u64 epf_vnet_vdev_get_features(struct virtio_device *vdev)
{
	struct epf_vnet *vnet = vdev_to_vnet(vdev);

	return vnet->features;
}

static int epf_vnet_vdev_finalize_features(struct virtio_device *vdev)
{
	struct epf_vnet *vnet = vdev_to_vnet(vdev);

	return vdev->features != vnet->features;
}

static void epf_vnet_vdev_get_config(struct virtio_device *vdev,
				     unsigned int offset, void *buf,
				     unsigned int len)
{
	struct epf_vnet *vnet = vdev_to_vnet(vdev);
	const unsigned int mac_len = sizeof(vnet->vdev_vnet_cfg.mac);
	const unsigned int status_len = sizeof(vnet->vdev_vnet_cfg.status);
	unsigned int copy_len;

	switch (offset) {
	case offsetof(struct virtio_net_config, mac):
		/* This PCIe EP function doesn't provide a VIRTIO_NET_F_MAC feature, so just
		 * clear the buffer.
		 */
		copy_len = len >= mac_len ? mac_len : len;
		memset(buf, 0x00, copy_len);
		len -= copy_len;
		buf += copy_len;
		fallthrough;
	case offsetof(struct virtio_net_config, status):
		copy_len = len >= status_len ? status_len : len;
		memcpy(buf, &vnet->vdev_vnet_cfg.status, copy_len);
		len -= copy_len;
		buf += copy_len;
		fallthrough;
	default:
		if (offset > sizeof(vnet->vdev_vnet_cfg)) {
			memset(buf, 0x00, len);
			break;
		}
		memcpy(buf, (void *)&vnet->vdev_vnet_cfg + offset, len);
	}
}

static void epf_vnet_vdev_set_config(struct virtio_device *vdev,
				     unsigned int offset, const void *buf,
				     unsigned int len)
{
	/* Do nothing because this console device doesn't any support features */
}

static u8 epf_vnet_vdev_get_status(struct virtio_device *vdev)
{
	struct epf_vnet *vnet = vdev_to_vnet(vdev);

	return vnet->vdev_status;
}

static void epf_vnet_vdev_set_status(struct virtio_device *vdev, u8 status)
{
	struct epf_vnet *vnet = vdev_to_vnet(vdev);

	vnet->vdev_status = status;

	if (status & VIRTIO_CONFIG_S_DRIVER_OK)
		epf_vnet_init_complete(vnet, EPF_VNET_INIT_COMPLETE_VDEV);
}

static void epf_vnet_vdev_reset(struct virtio_device *vdev)
{
	pr_debug("doesn't support yet");
}

static bool epf_vnet_vdev_vq_notify(struct virtqueue *vq)
{
	struct epf_vnet *vnet = vdev_to_vnet(vq->vdev);

	/* Support only one queue pair */
	switch (vq->index) {
	case VNET_VIRTQUEUE_RX:
		if (unlikely(vnet->init_state !=
			     (EPF_VNET_INIT_COMPLETE_VDEV |
			      EPF_VNET_INIT_COMPLETE_EP_FUNC)))
			break;
		queue_work(vnet->task_rx_wq, &vnet->rx_work);
		break;
	case VNET_VIRTQUEUE_TX:
		queue_work(vnet->task_tx_wq, &vnet->tx_work);
		break;
	case VNET_VIRTQUEUE_CTRL:
		queue_work(vnet->task_tx_wq, &vnet->vdev_ctrl_work);
		break;
	default:
		return false;
	}

	return true;
}

static int epf_vnet_vdev_find_vqs(struct virtio_device *vdev, unsigned int nvqs,
				  struct virtqueue *vqs[],
				  struct virtqueue_info vqs_info[],
				  struct irq_affinity *desc)
{
	struct epf_vnet *vnet = vdev_to_vnet(vdev);
	struct virtqueue_info *vqi;
	int i;
	int err;
	int qidx;

	if (nvqs > epf_vnet_get_nvq(vnet))
		return -EINVAL;

	for (qidx = 0, i = 0; i < nvqs; i++) {
		struct virtqueue *vq;
		const struct vring *vring;
		vqi = &vqs_info[i];

		if (!vqi->name) {
			vqs[i] = NULL;
			continue;
		}

		vq = vring_create_virtqueue(qidx++, virtio_queue_size,
					    VIRTIO_PCI_VRING_ALIGN, vdev, true,
					    false, vqi->ctx,
					    epf_vnet_vdev_vq_notify,
					    vqi->callback, vqi->name);
		if (!vq) {
			err = -ENOMEM;
			goto err_del_vqs;
		}

		vqs[i] = vq;
		vnet->vdev_vqs[i] = vq;
		vring = virtqueue_get_vring(vq);

		err = vringh_init_kern(&vnet->vdev_vrhs[i], vnet->features,
				       virtio_queue_size, false, vring->desc,
				       vring->avail, vring->used);
		if (err) {
			pr_err("failed to init vringh for vring %d\n", i);
			goto err_del_vqs;
		}
	}

	return 0;

err_del_vqs:
	for (; i >= 0; i--) {
		if (!vqi->name)
			continue;

		if (!vqs[i])
			continue;

		vring_del_virtqueue(vqs[i]);
	}
	return err;
}

static void epf_vnet_vdev_del_vqs(struct virtio_device *vdev)
{
	struct epf_vnet *vnet = vdev_to_vnet(vdev);

	for (int i = 0; i < epf_vnet_get_nvq(vnet); i++) {
		if (!vnet->vdev_vqs[i])
			continue;

		vring_del_virtqueue(vnet->vdev_vqs[i]);
	}
}

static void epf_vnet_vdev_release(struct device *dev)
{
	/* Do nothing, because the struct virtio_device will be reused. */
}

static const struct virtio_config_ops epf_vnet_vdev_config_ops = {
	.get_features = epf_vnet_vdev_get_features,
	.finalize_features = epf_vnet_vdev_finalize_features,
	.get = epf_vnet_vdev_get_config,
	.set = epf_vnet_vdev_set_config,
	.get_status = epf_vnet_vdev_get_status,
	.set_status = epf_vnet_vdev_set_status,
	.reset = epf_vnet_vdev_reset,
	.find_vqs = epf_vnet_vdev_find_vqs,
	.del_vqs = epf_vnet_vdev_del_vqs,
};

static int epf_vnet_setup_vdev(struct epf_vnet *vnet, struct device *parent)
{
	u16 nvq = epf_vnet_get_nvq(vnet);
	struct virtio_device *vdev = &vnet->vdev;
	int err;

	vnet->vdev_vrhs =
		kmalloc_array(nvq, sizeof(vnet->vdev_vrhs[0]), GFP_KERNEL);
	if (!vnet->vdev_vrhs)
		return -ENOMEM;

	vnet->vdev_iovs =
		kmalloc_array(nvq, sizeof(vnet->vdev_iovs[0]), GFP_KERNEL);
	if (!vnet->vdev_iovs) {
		err = -ENOMEM;
		goto err_free_vrhs;
	}

	for (int i = 0; i < nvq; i++)
		vringh_kiov_init(&vnet->vdev_iovs[i], NULL, 0);

	vnet->vdev_vqs =
		kmalloc_array(nvq, sizeof(vnet->vdev_vrhs[0]), GFP_KERNEL);
	if (!vnet->vdev_vqs) {
		err = -ENOMEM;
		goto err_cleanup_kiov;
	}

	vdev->dev.parent = parent;
	vdev->dev.release = epf_vnet_vdev_release;
	vdev->config = &epf_vnet_vdev_config_ops;
	vdev->id.vendor = PCI_VENDOR_ID_REDHAT_QUMRANET;
	vdev->id.device = VIRTIO_ID_NET;

	err = register_virtio_device(vdev);
	if (err)
		goto err_free_vdev_vqs;

	return 0;

err_free_vdev_vqs:
	kfree(vnet->vdev_vqs);

err_cleanup_kiov:
	for (int i = 0; i < nvq; i++)
		vringh_kiov_cleanup(&vnet->vdev_iovs[i]);

	kfree(vnet->vdev_iovs);

err_free_vrhs:
	kfree(vnet->vdev_vrhs);

	return err;
}

static void epf_vnet_cleanup_vdev(struct epf_vnet *vnet)
{
	unregister_virtio_device(&vnet->vdev);
	/* Cleanup struct virtio_device that has kobject, otherwise error occures when
	 * reregister the virtio device. */
	memset(&vnet->vdev, 0x00, sizeof(vnet->vdev));

	kfree(vnet->vdev_vqs);

	for (int i = 0; i < epf_vnet_get_nvq(vnet); i++)
		vringh_kiov_cleanup(&vnet->vdev_iovs[i]);

	kfree(vnet->vdev_iovs);
	kfree(vnet->vdev_vrhs);
}

static int epf_vnet_bind(struct pci_epf *epf)
{
	struct epf_vnet *vnet = epf_get_drvdata(epf);
	const struct pci_epc_features *epc_features;
	int err;

	epc_features = pci_epc_get_features(epf->epc, epf->func_no, epf->vfunc_no);
	if (!epc_features) {
		dev_err(&epf->dev, "epc_features not implemented\n");
		return -EOPNOTSUPP;
	}

	vnet->epc_features = epc_features;

	err = epf_vnet_setup_common(vnet);
	if (err)
		return err;

	return 0;
}

static void epf_vnet_unbind(struct pci_epf *epf)
{
	struct epf_vnet *vnet = epf_get_drvdata(epf);

	if (vnet->enable_edma)
		epf_virtio_cleanup_edma(&vnet->evio);
	epf_vnet_cleanup_common(vnet);
	epf_vnet_cleanup_ep_func(vnet);
	epf_vnet_cleanup_vdev(vnet);
}

static struct pci_epf_ops epf_vnet_ops = {
	.bind = epf_vnet_bind,
	.unbind = epf_vnet_unbind,
};

static int epf_vnet_epc_init(struct pci_epf *epf)
{
	struct epf_vnet *vnet = epf_get_drvdata(epf);
	bool linkup_notifier = false;
	int err;

	err = epf_vnet_setup_ep_func(vnet, epf);
	if (err) {
		dev_err(&epf->dev,
			"Failed to setup PCIe EP virtio-net function");
		return err;
	}

	err = epf_virtio_setup_edma(&vnet->evio, epf->epc->dev.parent);
	if (err) {
		dev_info(
			&epf->dev,
			"PCIe embedded DMAC wasn't found. Fallback to CPU transfer\n");
	}

	vnet->enable_edma = !err;

	err = epf_vnet_setup_vdev(vnet, epf->epc->dev.parent);
	if (err) /* FIXME: err path */
		goto err_cleanup_edma;

	linkup_notifier = vnet->epc_features->linkup_notifier;
	if (!linkup_notifier) {
		err = epf_virtio_launch_bgtask(&vnet->evio);
		if (unlikely(err))
			goto err_cleanup_vdev;
	}

	return 0;

err_cleanup_vdev:
	epf_vnet_cleanup_vdev(vnet);
err_cleanup_edma:
	if (vnet->enable_edma)
		epf_virtio_cleanup_edma(&vnet->evio);
	epf_vnet_cleanup_ep_func(vnet);
	epf_vnet_cleanup_common(vnet);

	return err;
}

static int epf_vnet_link_up(struct pci_epf *epf)
{
	struct epf_vnet *vnet = epf_get_drvdata(epf);
	int err;

	err = epf_virtio_launch_bgtask(&vnet->evio);
	if (unlikely(err))
		goto err_cleanup;

	return 0;

err_cleanup:
	epf_vnet_cleanup_vdev(vnet);
	if (vnet->enable_edma)
		epf_virtio_cleanup_edma(&vnet->evio);
	epf_vnet_cleanup_ep_func(vnet);
	epf_vnet_cleanup_common(vnet);

	return err;
}

static int epf_vnet_link_down(struct pci_epf *epf)
{
	struct epf_vnet *vnet = epf_get_drvdata(epf);

	epf_virtio_terminate_bgtask(&vnet->evio);

	return 0;
}

static const struct pci_epc_event_ops epf_vnet_event_ops = {
	.epc_init = epf_vnet_epc_init,
	.link_up = epf_vnet_link_up,
	.link_down = epf_vnet_link_down,
};

static const struct pci_epf_device_id epf_vnet_ids[] = {
	{ .name = "pci_epf_vnet" },
	{}
};

static int epf_vnet_probe(struct pci_epf *epf,
			  const struct pci_epf_device_id *id)
{
	struct epf_vnet *vnet;

	vnet = devm_kzalloc(&epf->dev, sizeof(*vnet), GFP_KERNEL);
	if (!vnet)
		return -ENOMEM;

	epf->event_ops = &epf_vnet_event_ops;
	epf_set_drvdata(epf, vnet);

	return 0;
}

static struct pci_epf_driver epf_vnet_drv = {
	.driver.name = "pci_epf_vnet",
	.ops = &epf_vnet_ops,
	.id_table = epf_vnet_ids,
	.probe = epf_vnet_probe,
	.owner = THIS_MODULE,
};

static int __init epf_vnet_init(void)
{
	int err;

	err = pci_epf_register_driver(&epf_vnet_drv);
	if (err) {
		pr_err("Failed to register epf vnet driver\n");
		return err;
	}

	return 0;
}
module_init(epf_vnet_init);

static void epf_vnet_exit(void)
{
	pci_epf_unregister_driver(&epf_vnet_drv);
}
module_exit(epf_vnet_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Shunsuke Mie <mie@igel.co.jp>");
MODULE_DESCRIPTION("PCI endpoint function acts as virtio net device");
