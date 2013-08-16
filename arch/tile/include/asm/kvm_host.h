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

#ifndef _ASM_TILE_KVM_HOST_H
#define _ASM_TILE_KVM_HOST_H

#define KVM_MAX_VCPUS 64
#define KVM_USER_MEM_SLOTS 32
#define KVM_PRIVATE_MEM_SLOTS 4

/* For now, claim we have no huge pages. */
#define KVM_HPAGE_GFN_SHIFT(x)  0
#define KVM_NR_PAGE_SIZES       1
#define KVM_PAGES_PER_HPAGE(x)  1

/* Max number of message tags for hv_send/receive_message() */
#define MAX_MSG_TAG	(sizeof(unsigned long) * 8)

/* Bits in pending_downcalls */
#define DOWNCALL_MESSAGE_RCV     0x01  /**< Message receive */

#ifndef __ASSEMBLY__

#include <linux/types.h>
#include <linux/ptrace.h>

struct kvm_vcpu_stat {
	/* None yet. */
};

struct kvm_vcpu_arch {
	struct pt_regs regs;
	struct kvm_sregs sregs;
	unsigned long host_sp; /* Host "real" sp during vmresume. */
	HV_Context guest_context;
	unsigned long pending_msgs; /* Pending guest messages */
	unsigned long ipi_events; /* Pending guest ipi events. */
	unsigned long ipi_gpa; /* pa for hv_get_ipi_pte() */
	pte_t ipi_gpte; /* pte for hv_get_ipi_pte() */
	unsigned long fault_addr;  /* addr for VPGTABLE_MISS faults */
	int suspended;  /* true for cores not yet started by host */
	unsigned long timer_control;  /* AUX_TILE_TIMER_CONTROL value */
	unsigned long vmexit_cycles;  /* cycle count of last vmexit */
};

struct kvm_vm_stat {
	/*
	 * FIXME - does this make sense for us?  It's used in common KVM
	 * code.
	 */
	u32 remote_tlb_flush;
};

struct kvm_arch_memory_slot {
};

struct kvm_arch {
	pgd_t *vpgd;
	unsigned long resv_gpa_start; /* For special purpose. */
	struct completion smp_start;
};

struct kvm_vcpu;

extern void kvm_vmresume(struct pt_regs *guest,
			 unsigned long *host_sp_ptr);
extern void kvm_vmexit(unsigned long host_sp);
extern void kvm_trigger_vmexit(struct pt_regs *regs, int exit_reason);
extern void kvm_do_hypervisor_call(struct pt_regs *regs, int fault_num);
extern void kvm_do_vpgtable_miss(struct pt_regs *regs, int fault_num,
				 unsigned long, unsigned long);
extern void kvm_do_vguest_fatal(struct pt_regs *regs, int fault_num);

extern void kvm_vcpu_kick(struct kvm_vcpu *vcpu);

#define gpud_offset(kvm, pgd, address) pud_offset(pgd, address)

#define gpud_page_vaddr(kvm, pud) gfn_to_hva(kvm, pud_pfn(pud))

#define gpmd_offset(kvm, pud, address) \
	((pmd_t *)gpud_page_vaddr(kvm, *(pud)) + pmd_index(address))

#define gpmd_page_vaddr(kvm, pmd) gfn_to_hva(kvm, pmd_pfn(pmd))

#define gpte_offset_kernel(kvm, pmd, address) \
	((pte_t *) gpmd_page_vaddr(kvm, *(pmd)) + pte_index(address))

#endif /* __ASSEMBLY__*/

#endif /* _ASM_TILE_KVM_HOST_H */
