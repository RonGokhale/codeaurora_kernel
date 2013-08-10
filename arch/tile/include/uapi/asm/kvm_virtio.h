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

#ifndef _UAPI_ASM_TILE_KVM_VIRTIO_H
#define _UAPI_ASM_TILE_KVM_VIRTIO_H

#include <linux/types.h>

#define KVM_VIRTIO_UNKNOWN	0
#define KVM_VIRTIO_NOTIFY	1
#define KVM_VIRTIO_RESET	2
#define KVM_VIRTIO_SET_STATUS	3

struct kvm_device_desc {
	/* The device type: console, network, disk etc.  Type 0 terminates. */
	__u8 type;
	/* The number of virtqueues (first in config array) */
	__u8 num_vq;
	/*
	 * The number of bytes of feature bits.  Multiply by 2: one for host
	 * features and one for Guest acknowledgements.
	 */
	__u8 feature_len;
	/* The number of bytes of the config array after virtqueues. */
	__u8 config_len;
	/* A status byte, written by the Guest. */
	__u8 status;
	__u64 config[0];
};

struct kvm_vqinfo {
	/* Pointer to the information contained in the device config. */
	struct kvm_vqconfig *config;
	/* The address where we mapped the virtio ring, so we can unmap it. */
	void *pages;
};

struct kvm_vqconfig {
	/* The physical address of the virtio ring */
	__u64 pa;
	/* The number of entries in the virtio_ring */
	__u64 num;
	/* The interrupt we get when something happens. Set by the guest. */
	__u32 irq;

};


#endif /* _UAPI_ASM_TILE_KVM_VIRTIO_H */
