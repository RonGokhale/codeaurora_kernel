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

#ifndef _UAPI_ASM_TILE_KVM_H
#define _UAPI_ASM_TILE_KVM_H

#ifndef __ASSEMBLER__
#include <linux/ptrace.h>
#endif

#include <arch/abi.h>

/*
 * For Hypervisor syscalls. Note this comes from the hv: syscall.h,
 * with small modifications: Remove HV_SYS_fence_incoherent.
 */
/* Syscall allowed from guest PL bit mask. */
#define HV_SYS_GUEST_SHIFT                12
#define HV_SYS_GUEST_MASK                 (1 << HV_SYS_GUEST_SHIFT)
/* downcall_dispatch; this syscall number must be zero */
#define HV_SYS_downcall_dispatch          0
/* install_context */
#define HV_SYS_install_context            1
/* sysconf */
#define HV_SYS_sysconf                    2
/* get_rtc */
#define HV_SYS_get_rtc                    3
/* set_rtc */
#define HV_SYS_set_rtc                    4
/* flush_asid */
#define HV_SYS_flush_asid                 5
/* flush_page */
#define HV_SYS_flush_page                 6
/* flush_pages */
#define HV_SYS_flush_pages                7
/* restart */
#define HV_SYS_restart                    8
/* halt */
#define HV_SYS_halt                       9
/* power_off */
#define HV_SYS_power_off                 10
/* inquire_physical */
#define HV_SYS_inquire_physical          11
/* inquire_memory_controller */
#define HV_SYS_inquire_memory_controller 12
/* inquire_virtual */
#define HV_SYS_inquire_virtual           13
/* inquire_asid */
#define HV_SYS_inquire_asid              14
/* console_read_if_ready */
#define HV_SYS_console_read_if_ready     15
/* console_write */
#define HV_SYS_console_write             16
/* init */
#define HV_SYS_init                      17
/* inquire_topology */
#define HV_SYS_inquire_topology          18
/* fs_findfile */
#define HV_SYS_fs_findfile               19
/* fs_fstat */
#define HV_SYS_fs_fstat                  20
/* fs_pread */
#define HV_SYS_fs_pread                  21
/* physaddr_read64 */
#define HV_SYS_physaddr_read64           22
/* physaddr_write64 */
#define HV_SYS_physaddr_write64          23
/* get_command_line */
#define HV_SYS_get_command_line          24
/* set_caching */
#define HV_SYS_set_caching               25
/* bzero_page */
#define HV_SYS_bzero_page                26
/* register_message_state */
#define HV_SYS_register_message_state    27
/* send_message */
#define HV_SYS_send_message              28
/* receive_message */
#define HV_SYS_receive_message           29
/* inquire_context */
#define HV_SYS_inquire_context           30
/* start_all_tiles */
#define HV_SYS_start_all_tiles           31
/* dev_open */
#define HV_SYS_dev_open                  32
/* dev_close */
#define HV_SYS_dev_close                 33
/* dev_pread */
#define HV_SYS_dev_pread                 34
/* dev_pwrite */
#define HV_SYS_dev_pwrite                35
/* dev_poll */
#define HV_SYS_dev_poll                  36
/* dev_poll_cancel */
#define HV_SYS_dev_poll_cancel           37
/* dev_preada */
#define HV_SYS_dev_preada                38
/* dev_pwritea */
#define HV_SYS_dev_pwritea               39
/* flush_remote */
#define HV_SYS_flush_remote              40
/* console_putc */
#define HV_SYS_console_putc              41
/* inquire_tiles */
#define HV_SYS_inquire_tiles             42
/* confstr */
#define HV_SYS_confstr                   43
/* reexec */
#define HV_SYS_reexec                    44
/* set_command_line */
#define HV_SYS_set_command_line          45

/* store_mapping */
#define HV_SYS_store_mapping             52
/* inquire_realpa */
#define HV_SYS_inquire_realpa            53
/* flush_all */
#define HV_SYS_flush_all                 54
/* get_ipi_pte */
#define HV_SYS_get_ipi_pte               55
/* set_pte_super_shift */
#define HV_SYS_set_pte_super_shift       56
/* set_speed */
#define HV_SYS_set_speed                 57
/* install_virt_context */
#define HV_SYS_install_virt_context      58
/* inquire_virt_context */
#define HV_SYS_inquire_virt_context      59
/* inquire_guest_context */
#define HV_SYS_install_guest_context     60
/* inquire_guest_context */
#define HV_SYS_inquire_guest_context     61

/*
 * Number of hypercall (from guest os to host os) other than hv_*().
 * We leave the previous 128 entries to the usual hv_*() calls
 * as defined in hypervisor.h.
 */
#define KVM_OTHER_HCALL                  128

/* Hypercall index for virtio. */
#define KVM_HCALL_virtio                 128

/* One greater than the maximum hypercall number. */
#define KVM_NUM_HCALLS                   256

#ifndef __ASSEMBLER__

struct kvm_regs {
	struct pt_regs regs;
};

#define FOR_EACH_GUEST_SPR(f)			\
	f(INTERRUPT_MASK_1);			\
	f(INTERRUPT_VECTOR_BASE_1);		\
	f(EX_CONTEXT_1_0);			\
	f(EX_CONTEXT_1_1);			\
	f(SYSTEM_SAVE_1_0);			\
	f(SYSTEM_SAVE_1_1);			\
	f(SYSTEM_SAVE_1_2);			\
	f(SYSTEM_SAVE_1_3);			\
	f(INTCTRL_1_STATUS);			\
	f(IPI_MASK_1);				\
	f(IPI_EVENT_1);				\
	f(SINGLE_STEP_CONTROL_1);		\
	f(SINGLE_STEP_EN_1_1);			\

struct kvm_sregs {
#define DECLARE_SPR(f) unsigned long f
	FOR_EACH_GUEST_SPR(DECLARE_SPR)
#undef DECLARE_SPR
};

struct kvm_fpu {
};

struct kvm_debug_exit_arch {
};

struct kvm_guest_debug_arch {
};

/* definition of registers in kvm_run */
struct kvm_sync_regs {
};

#ifndef __KERNEL__
/* For hv_*() */
#define KVM_EMULATE(name) [HV_SYS_##name] = qemu_emulate_illegal,
#define USER_EMULATE(name) [HV_SYS_##name] = qemu_emulate_hv_##name,
#define NO_EMULATE(name) [HV_SYS_##name] = qemu_emulate_illegal,
#define BOTH_EMULATE(name) [HV_SYS_##name] = qemu_emulate_hv_##name,
/* For others */
#define USER_HCALL(name) [KVM_HCALL_##name] = qemu_handle_##name,
#endif

#define HCALL_DEFS \
	/* For hv_*() */ \
	KVM_EMULATE(init) \
	NO_EMULATE(install_context) \
	KVM_EMULATE(sysconf) \
	KVM_EMULATE(get_rtc) \
	KVM_EMULATE(set_rtc) \
	NO_EMULATE(flush_asid) \
	NO_EMULATE(flush_page) \
	NO_EMULATE(flush_pages) \
	USER_EMULATE(restart) \
	USER_EMULATE(halt) \
	USER_EMULATE(power_off) \
	USER_EMULATE(inquire_physical) \
	USER_EMULATE(inquire_memory_controller) \
	KVM_EMULATE(inquire_virtual) \
	KVM_EMULATE(inquire_asid) \
	NO_EMULATE(console_read_if_ready) \
	NO_EMULATE(console_write) \
	NO_EMULATE(downcall_dispatch) \
	KVM_EMULATE(inquire_topology) \
	USER_EMULATE(fs_findfile) \
	USER_EMULATE(fs_fstat) \
	USER_EMULATE(fs_pread) \
	KVM_EMULATE(physaddr_read64) \
	KVM_EMULATE(physaddr_write64) \
	USER_EMULATE(get_command_line) \
	USER_EMULATE(set_caching) \
	NO_EMULATE(bzero_page) \
	KVM_EMULATE(register_message_state) \
	KVM_EMULATE(send_message) \
	KVM_EMULATE(receive_message) \
	KVM_EMULATE(inquire_context) \
	KVM_EMULATE(start_all_tiles) \
	USER_EMULATE(dev_open) \
	USER_EMULATE(dev_close) \
	USER_EMULATE(dev_pread) \
	USER_EMULATE(dev_pwrite) \
	USER_EMULATE(dev_poll) \
	USER_EMULATE(dev_poll_cancel) \
	USER_EMULATE(dev_preada) \
	USER_EMULATE(dev_pwritea) \
	USER_EMULATE(flush_remote) \
	NO_EMULATE(console_putc) \
	KVM_EMULATE(inquire_tiles) \
	KVM_EMULATE(confstr) \
	USER_EMULATE(reexec) \
	USER_EMULATE(set_command_line) \
	USER_EMULATE(store_mapping) \
	NO_EMULATE(inquire_realpa) \
	NO_EMULATE(flush_all) \
	KVM_EMULATE(get_ipi_pte) \
	KVM_EMULATE(set_pte_super_shift) \
	KVM_EMULATE(set_speed) \
	/* For others */ \
	USER_HCALL(virtio)

#endif

#endif /* _UAPI_ASM_TILE_KVM_H */
