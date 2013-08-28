/*
 * Copyright 2013 Tilera Corporation. All Rights Reserved.
 *
 *   This program is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU General Public License
 *   as published by the Free Software Foundation, version 2.
 *
 *   This program is distributed in the hope that it will be useful, but
 *   WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE, GOOD TITLE or
 *   NON INFRINGEMENT.  See the GNU General Public License for
 *   more details.
 */

/* Referred lguest & s390 implemenation */
/*
 * kvm_virtio.c - virtio for kvm on s390
 *
 * Copyright IBM Corp. 2008
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License (version 2 only)
 * as published by the Free Software Foundation.
 *
 *    Author(s): Christian Borntraeger <borntraeger@de.ibm.com>
 */

#include <linux/bootmem.h>
#include <linux/io.h>
#include <linux/vmalloc.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/export.h>
#include <linux/virtio.h>
#include <linux/virtio_config.h>
#include <linux/virtio_console.h>
#include <linux/virtio_ring.h>
#include <linux/virtio_pci.h>

#include <linux/kvm_para.h>
#include <asm/kvm_virtio.h>

static void *kvm_devices;

/*
 * TODO: We actually does not use PCI virtio here. We use this
 * because qemu: virtqueue_init() uses VIRTIO_PCI_VRING_ALIGN.
 * Maybe we should change them to generic definitions in both qemu & Linux.
 * Besides, Let's check whether the alignment value (4096, i.e. default
 * x86 page size) affects performance later.
 */
#define KVM_TILE_VIRTIO_RING_ALIGN	VIRTIO_PCI_VRING_ALIGN
#define to_kvmdev(vd)	container_of(vd, struct kvm_device, vdev)

/*
 * memory layout: (Total: PAGE_SIZE)
 * <device 0>
 * - kvm device descriptor
 *        struct kvm_device_desc
 * - vqueue configuration (totally desc->num_vq)
 *        struct kvm_vqconfig
 *        ......
 *        struct kvm_vqconfig
 * - feature bits (size: desc->feature_len * 2)
 * - config space (size: desc->config_len)
 * <device 1>
 * ......
 */
static struct kvm_vqconfig *kvm_vq_config(const struct kvm_device_desc *desc)
{
	return (struct kvm_vqconfig *)(desc + 1);
}

static u8 *kvm_vq_features(const struct kvm_device_desc *desc)
{
	return (u8 *)(kvm_vq_config(desc) + desc->num_vq);
}

static u8 *kvm_vq_configspace(const struct kvm_device_desc *desc)
{
	return kvm_vq_features(desc) + desc->feature_len * 2;
}

/*
 * The total size of the config page used by this device (incl. desc)
 */
static unsigned desc_size(const struct kvm_device_desc *desc)
{
	return sizeof(*desc)
		+ desc->num_vq * sizeof(struct kvm_vqconfig)
		+ desc->feature_len * 2
		+ desc->config_len;
}

/* This gets the device's feature bits. */
static u32 kvm_get_features(struct virtio_device *vdev)
{
	unsigned int i;
	u32 features = 0;
	struct kvm_device_desc *desc = to_kvmdev(vdev)->desc;
	u8 *in_features = kvm_vq_features(desc);

	for (i = 0; i < min(desc->feature_len * 8, 32); i++)
		if (in_features[i / 8] & (1 << (i % 8)))
			features |= (1 << i);
	return features;
}

static void kvm_finalize_features(struct virtio_device *vdev)
{
	unsigned int i, bits;
	struct kvm_device_desc *desc = to_kvmdev(vdev)->desc;
	/* Second half of bitmap is features we accept. */
	u8 *out_features = kvm_vq_features(desc) + desc->feature_len;

	/* Give virtio_ring a chance to accept features. */
	vring_transport_features(vdev);

	memset(out_features, 0, desc->feature_len);
	bits = min_t(unsigned, desc->feature_len, sizeof(vdev->features)) * 8;
	for (i = 0; i < bits; i++) {
		if (test_bit(i, vdev->features))
			out_features[i / 8] |= (1 << (i % 8));
	}
}

/*
 * Reading and writing elements in config space
 */
static void kvm_get(struct virtio_device *vdev, unsigned int offset,
		   void *buf, unsigned len)
{
	struct kvm_device_desc *desc = to_kvmdev(vdev)->desc;

	BUG_ON(offset + len > desc->config_len);
	memcpy(buf, kvm_vq_configspace(desc) + offset, len);
}

static void kvm_set(struct virtio_device *vdev, unsigned int offset,
		   const void *buf, unsigned len)
{
	struct kvm_device_desc *desc = to_kvmdev(vdev)->desc;

	BUG_ON(offset + len > desc->config_len);
	memcpy(kvm_vq_configspace(desc) + offset, buf, len);
}

/*
 * The operations to get and set the status word just access
 * the status field of the device descriptor. set_status will also
 * make a hypercall to the host, to tell about status changes
 */
static u8 kvm_get_status(struct virtio_device *vdev)
{
	return to_kvmdev(vdev)->desc->status;
}

static void kvm_set_status(struct virtio_device *vdev, u8 status)
{
	BUG_ON(!status);
	to_kvmdev(vdev)->desc->status = status;
	hcall_virtio(KVM_VIRTIO_SET_STATUS, to_kvmdev(vdev)->desc_pa);
}

/*
 * To reset the device, we use the KVM_VIRTIO_RESET hypercall, using the
 * descriptor address. The Host will zero the status and all the
 * features.
 */
static void kvm_reset(struct virtio_device *vdev)
{
	hcall_virtio(KVM_VIRTIO_RESET, to_kvmdev(vdev)->desc_pa);
}

/*
 * When the virtio_ring code wants to notify the Host, it calls us here and we
 * make a hypercall.  We hand the address  of the virtqueue so the Host
 * knows which virtqueue we're talking about.
 */
static void kvm_notify(struct virtqueue *vq)
{
	struct kvm_vqinfo *vqi = vq->priv;

	hcall_virtio(KVM_VIRTIO_NOTIFY, vqi->config->pa);
}

/*
 * Must set some caching mode to keep set_pte() happy.
 * It doesn't matter what we choose, because the PFN
 * is illegal, so we're going to take a page fault anyway.
 */
static inline pgprot_t io_prot(void)
{
	return hv_pte_set_mode(PAGE_KERNEL, HV_PTE_MODE_UNCACHED);
}

/*
 * This routine finds the first virtqueue described in the configuration of
 * this device and sets it up.
 */
static struct virtqueue *kvm_find_vq(struct virtio_device *vdev,
				     unsigned index,
				     void (*callback)(struct virtqueue *vq),
				     const char *name)
{
	struct kvm_device *kdev = to_kvmdev(vdev);
	struct kvm_vqinfo *vqi;
	struct kvm_vqconfig *config;
	struct virtqueue *vq;
	long irq;
	int err = -EINVAL;

	if (index >= kdev->desc->num_vq)
		return ERR_PTR(-ENOENT);

	vqi = kzalloc(sizeof(*vqi), GFP_KERNEL);
	if (!vqi)
		return ERR_PTR(-ENOMEM);

	config = kvm_vq_config(kdev->desc)+index;

	vqi->config = config;
	vqi->pages = generic_remap_prot(config->pa,
				vring_size(config->num,
					KVM_TILE_VIRTIO_RING_ALIGN),
					0, io_prot());
	if (!vqi->pages) {
		err = -ENOMEM;
		goto out;
	}

	vq = vring_new_virtqueue(index, config->num, KVM_TILE_VIRTIO_RING_ALIGN,
				 vdev, 0, vqi->pages,
				 kvm_notify, callback, name);
	if (!vq) {
		err = -ENOMEM;
		goto unmap;
	}

	/*
	 * Trigger the IPI interrupt in SW way.
	 * TODO: We do not need to create one irq for each vq. A bit wasteful.
	 */
	irq = create_irq();
	if (irq < 0) {
		err = -ENXIO;
		goto del_virtqueue;
	}

	tile_irq_activate(irq, TILE_IRQ_SW_CLEAR);

	if (request_irq(irq, vring_interrupt, 0, dev_name(&vdev->dev), vq)) {
		err = -ENXIO;
		destroy_irq(irq);
		goto del_virtqueue;
	}

	config->irq = irq;

	vq->priv = vqi;
	return vq;

del_virtqueue:
	vring_del_virtqueue(vq);
unmap:
	vunmap(vqi->pages);
out:
	return ERR_PTR(err);
}

static void kvm_del_vq(struct virtqueue *vq)
{
	struct kvm_vqinfo *vqi = vq->priv;

	vring_del_virtqueue(vq);
	vunmap(vqi->pages);
	kfree(vqi);
}

static void kvm_del_vqs(struct virtio_device *vdev)
{
	struct virtqueue *vq, *n;

	list_for_each_entry_safe(vq, n, &vdev->vqs, list)
		kvm_del_vq(vq);
}

static int kvm_find_vqs(struct virtio_device *vdev, unsigned nvqs,
			struct virtqueue *vqs[],
			vq_callback_t *callbacks[],
			const char *names[])
{
	struct kvm_device *kdev = to_kvmdev(vdev);
	int i;

	/* We must have this many virtqueues. */
	if (nvqs > kdev->desc->num_vq)
		return -ENOENT;

	for (i = 0; i < nvqs; ++i) {
		vqs[i] = kvm_find_vq(vdev, i, callbacks[i], names[i]);
		if (IS_ERR(vqs[i]))
			goto error;
	}
	return 0;

error:
	kvm_del_vqs(vdev);
	return PTR_ERR(vqs[i]);
}

/*
 * The config ops structure as defined by virtio config
 */
static struct virtio_config_ops kvm_vq_config_ops = {
	.get_features = kvm_get_features,
	.finalize_features = kvm_finalize_features,
	.get = kvm_get,
	.set = kvm_set,
	.get_status = kvm_get_status,
	.set_status = kvm_set_status,
	.reset = kvm_reset,
	.find_vqs = kvm_find_vqs,
	.del_vqs = kvm_del_vqs,
};

/*
 * The root device for the kvm virtio devices.
 * This makes them appear as /sys/devices/kvm_tile/0,1,2 not /sys/devices/0,1,2.
 */
static struct device *kvm_root;

/*
 * adds a new device and register it with virtio
 * appropriate drivers are loaded by the device model
 */
static void add_kvm_device(struct kvm_device_desc *d, unsigned int offset)
{
	struct kvm_device *kdev;

	kdev = kzalloc(sizeof(*kdev), GFP_KERNEL);
	if (!kdev) {
		pr_emerg("Cannot allocate kvm dev %u type %u\n",
			 offset, d->type);
		return;
	}

	kdev->vdev.dev.parent = kvm_root;
	kdev->vdev.id.device = d->type;
	kdev->vdev.config = &kvm_vq_config_ops;
	kdev->desc = d;
	kdev->desc_pa = PFN_PHYS(max_pfn) + offset;

	if (register_virtio_device(&kdev->vdev) != 0) {
		pr_err("Failed to register kvm device %u type %u\n",
		       offset, d->type);
		kfree(kdev);
	}
}

/*
 * scan_devices() simply iterates through the device page.
 * The type 0 is reserved to mean "end of devices".
 */
static void scan_devices(void)
{
	unsigned int i;
	struct kvm_device_desc *d;

	for (i = 0; i < PAGE_SIZE; i += desc_size(d)) {
		d = kvm_devices + i;

		if (d->type == 0)
			break;

		add_kvm_device(d, i);
	}
}

/*
 * Init function for virtio.
 * devices are in a single page above the top of "normal" mem.
 */
static int __init kvm_devices_init(void)
{
	int rc = -ENOMEM;

	kvm_root = root_device_register("kvm_tile");
	if (IS_ERR(kvm_root)) {
		rc = PTR_ERR(kvm_root);
		pr_err("Could not register kvm_tile root device");
		return rc;
	}

	kvm_devices = generic_remap_prot(PFN_PHYS(max_pfn), PAGE_SIZE,
					 0, io_prot());
	if (!kvm_devices) {
		kvm_devices = NULL;
		root_device_unregister(kvm_root);
		return rc;
	}

	scan_devices();
	return 0;
}

/* code for early console output with virtio_console */
static __init int early_put_chars(u32 vtermno, const char *buf, int len)
{
	char scratch[512];

	if (len > sizeof(scratch) - 1)
		len = sizeof(scratch) - 1;
	scratch[len] = '\0';
	memcpy(scratch, buf, len);
	hcall_virtio(KVM_VIRTIO_NOTIFY, __pa(scratch));

	return len;
}

static int __init tile_virtio_console_init(void)
{
	return virtio_cons_early_init(early_put_chars);
}
console_initcall(tile_virtio_console_init);

/*
 * We do this after core stuff, but before the drivers.
 */
postcore_initcall(kvm_devices_init);
