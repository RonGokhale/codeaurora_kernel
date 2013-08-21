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
#ifndef _ASM_TILE_KVM_H
#define _ASM_TILE_KVM_H

#include <hv/hypervisor.h>
#include <uapi/asm/kvm.h>

#ifndef __ASSEMBLER__
/* For hv_*() */
#define KVM_EMULATE(name) [HV_SYS_##name] = kvm_emulate_hv_##name,
#define USER_EMULATE(name) [HV_SYS_##name] = kvm_deliver_to_user,
#define NO_EMULATE(name) [HV_SYS_##name] = kvm_emulate_illegal,
#define BOTH_EMULATE(name) [HV_SYS_##name] = kvm_emulate_hv_##name,
/* For others */
#define USER_HCALL(name) [KVM_HCALL_##name] = kvm_deliver_to_user,
#endif
#endif /* _ASM_TILE_KVM_H */
