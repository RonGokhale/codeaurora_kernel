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
#ifndef _ASM_TILE_KVM_VIRTIO_H
#define _ASM_TILE_KVM_VIRTIO_H

#include <uapi/asm/kvm_virtio.h>


struct kvm_device {
	struct virtio_device vdev;
	struct kvm_device_desc *desc;
	unsigned long desc_pa;
};

#endif /* _ASM_TILE_KVM_VIRTIO_H */
