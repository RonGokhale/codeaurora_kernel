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

#include <linux/err.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/kvm.h>
#include <linux/kvm_host.h>
#include <linux/kvm_types.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/ptrace.h>
#include <asm/traps.h>
#include <asm/pgalloc.h>
#include <hv/hypervisor.h>
#include <linux/rtc.h>
#include <asm/atomic.h>
#include <asm/tlbflush.h>
#include <arch/spr_def.h>
#include <arch/sim.h>
#include <generated/utsrelease.h>


struct kvm_stats_debugfs_item debugfs_entries[] = {
	{ NULL }
};

static pte_t *get_vpgd_pte(struct kvm *kvm, unsigned long address)
{
	struct mm_struct *mm = kvm->mm;
	pgd_t *pgd;
	pud_t *pud;
	pmd_t *pmd;

	if (kvm->arch.vpgd == NULL)
		kvm->arch.vpgd = pgd_alloc(kvm->mm);
	pgd = kvm->arch.vpgd + pgd_index(address);
	pud = pud_alloc(mm, pgd, address);
	if (!pud)
		return NULL;
	pmd = pmd_alloc(mm, pud, address);
	if (!pmd)
		return NULL;
	return pte_alloc_kernel(pmd, address);
}

int kvm_arch_vcpu_fault(struct kvm_vcpu *vcpu, struct vm_fault *vmf)
{
	return VM_FAULT_SIGBUS;
}

void kvm_arch_free_memslot(struct kvm_memory_slot *free,
			   struct kvm_memory_slot *dont)
{
}

int kvm_arch_create_memslot(struct kvm_memory_slot *slot, unsigned long npages)
{
	return 0;
}

/* FIXME: support huge pages. */
int kvm_arch_prepare_memory_region(struct kvm *kvm,
				   struct kvm_memory_slot *memslot,
				   struct kvm_userspace_memory_region *mem,
				   enum kvm_mr_change change)
{
	unsigned long gpa, i;

	gpa = mem->guest_phys_addr;
	for (i = 0; i < mem->memory_size; i += PAGE_SIZE, gpa += PAGE_SIZE)
		if (get_vpgd_pte(kvm, gpa) == NULL)
			return -ENOMEM;

	return 0;
}

void kvm_arch_commit_memory_region(struct kvm *kvm,
				   struct kvm_userspace_memory_region *mem,
				   const struct kvm_memory_slot *old,
				   enum kvm_mr_change change)
{
	unsigned long gpa, address, pfn, i;
	struct page *page[1];
	pte_t *ptep, *vptep;

	gpa = mem->guest_phys_addr;
	address = mem->userspace_addr;
	for (i = 0; i < mem->memory_size;
	     i += PAGE_SIZE, gpa += PAGE_SIZE, address += PAGE_SIZE) {
		vptep = get_vpgd_pte(kvm, gpa);
		BUG_ON(vptep == NULL);
		get_user_pages_fast(address, 1, 1, page);
		pfn = page_to_pfn(page[0]);
		ptep = virt_to_pte(NULL, (unsigned long)__va(PFN_PHYS(pfn)));
		*vptep = *ptep;
	}
}

void kvm_arch_flush_shadow_all(struct kvm *kvm)
{
}

void kvm_arch_flush_shadow_memslot(struct kvm *kvm,
				   struct kvm_memory_slot *slot)
{
	kvm_arch_flush_shadow_all(kvm);
}

gfn_t unalias_gfn(struct kvm *kvm, gfn_t gfn)
{
	return 0;
}

long kvm_arch_dev_ioctl(struct file *filp,
			unsigned int ioctl, unsigned long arg)
{
	return 0;
}

static int kvm_vcpu_ioctl_interrupt(struct kvm_vcpu *vcpu, unsigned long irq)
{
	if (irq < 0)
		return -EINVAL;

	set_bit(irq, &vcpu->arch.ipi_events);
	kvm_vcpu_kick(vcpu);

	return 0;
}

long kvm_arch_vcpu_ioctl(struct file *filp,
			 unsigned int ioctl, unsigned long arg)
{
	struct kvm_vcpu *vcpu = filp->private_data;
	void __user *argp = (void __user *)arg;
	int r = 0;

	switch (ioctl) {
	case KVM_INTERRUPT: {
		struct kvm_interrupt irq;

		r = -EFAULT;
		if (copy_from_user(&irq, argp, sizeof(irq)))
			goto out;
		r = kvm_vcpu_ioctl_interrupt(vcpu, irq.irq);
		if (r)
			goto out;
		r = 0;
		break;
	}
	default:
		r = -EINVAL;
	}

out:
	return r;
}

int kvm_dev_ioctl_check_extension(long ext)
{
	return 0;
}

int kvm_vm_ioctl_get_dirty_log(struct kvm *kvm,
			       struct kvm_dirty_log *log)
{
	return 0;
}

long kvm_arch_vm_ioctl(struct file *filp,
		       unsigned int ioctl, unsigned long arg)
{
	long r = -EINVAL;

	return r;
}

int kvm_arch_vcpu_ioctl_get_fpu(struct kvm_vcpu *vcpu, struct kvm_fpu *fpu)
{
	return 0;
}

int kvm_arch_vcpu_ioctl_set_fpu(struct kvm_vcpu *vcpu, struct kvm_fpu *fpu)
{
	return 0;
}

int kvm_arch_vcpu_ioctl_translate(struct kvm_vcpu *vcpu,
				  struct kvm_translation *tr)
{
	struct kvm *kvm = vcpu->kvm;
	unsigned long page_size;
	unsigned long gva = tr->linear_address;
	unsigned long gpgd_gpa, gpmd_gpa, gpte_gpa;
	pud_t gpud;
	pmd_t gpmd;
	pte_t gpte;

	/* Get guest pgd (aka pud for three-level tables). */
	gpgd_gpa = vcpu->arch.guest_context.page_table +
		(sizeof(pgd_t) * pgd_index(gva));
	if (kvm_read_guest(kvm, gpgd_gpa, &gpud, sizeof(pgd_t)) < 0)
		goto fail;
	if (!pud_present(gpud))
		goto fail;

	/* Get guest pmd. */
	if (pud_huge_page(gpud)) {
		/* FIXME: no super huge page support yet. */
		if (pte_super(*(pte_t *)&gpud))
			goto fail;
		gpte = *(pte_t *)&gpud;
		page_size = PGDIR_SIZE;
		goto ok;
	}
	gpmd_gpa = (pud_ptfn(gpud) << HV_LOG2_PAGE_TABLE_ALIGN) +
		(sizeof(pmd_t) * pmd_index(gva));
	if (kvm_read_guest(kvm, gpmd_gpa, &gpmd, sizeof(pmd_t)) < 0)
		goto fail;
	if (!pmd_present(gpmd))
		goto fail;

	/* Get guest pte. */
	if (pmd_huge_page(gpmd)) {
		/* FIXME: no super huge page support yet. */
		if (pte_super(*(pte_t *)&gpmd))
			goto fail;
		gpte = *(pte_t *)&gpmd;
		page_size = PMD_SIZE;
		goto ok;
	}
	gpte_gpa = (pmd_ptfn(gpmd) << HV_LOG2_PAGE_TABLE_ALIGN) +
		(sizeof(pte_t) * pte_index(gva));
	if (kvm_read_guest(kvm, gpte_gpa, &gpte, sizeof(pte_t)) < 0)
		goto fail;
	if (!pte_present(gpte))
		goto fail;

	page_size = PAGE_SIZE;

ok:
	tr->physical_address =
		PFN_PHYS(pte_pfn(gpte)) + (gva & (page_size - 1));
	tr->valid = 1;
	tr->writeable = pte_write(gpte);
	tr->usermode = pte_user(gpte);

	return 0;

fail:
	tr->valid = 0;
	return 0;
}

int kvm_arch_vcpu_ioctl_get_regs(struct kvm_vcpu *vcpu, struct kvm_regs *regs)
{
	regs->regs = vcpu->arch.regs;
	return 0;
}

int kvm_arch_vcpu_ioctl_set_regs(struct kvm_vcpu *vcpu, struct kvm_regs *regs)
{
	vcpu->arch.regs = regs->regs;
	vcpu->arch.regs.flags = PT_FLAGS_CALLER_SAVES | PT_FLAGS_RESTORE_REGS;
	return 0;
}

int kvm_arch_vcpu_ioctl_get_sregs(struct kvm_vcpu *vcpu,
				  struct kvm_sregs *sregs)
{
	*sregs = vcpu->arch.sregs;
	return 0;
}

int kvm_arch_vcpu_ioctl_set_sregs(struct kvm_vcpu *vcpu,
				  struct kvm_sregs *sregs)
{
	vcpu->arch.sregs = *sregs;
	return 0;
}

int kvm_arch_vcpu_ioctl_get_mpstate(struct kvm_vcpu *vcpu,
				    struct kvm_mp_state *mp_state)
{
	return 0;
}

int kvm_arch_vcpu_ioctl_set_mpstate(struct kvm_vcpu *vcpu,
				    struct kvm_mp_state *mp_state)
{
	return 0;
}

int kvm_arch_vcpu_ioctl_set_guest_debug(struct kvm_vcpu *vcpu,
					struct kvm_guest_debug *dbg)
{
	return 0;
}

/*
 * panic_hv() will dump stack info of both guest os and host os, and set
 * proper exit reason so that qemu can terminate the guest process.
 *
 * FIXME: Probably KVM_EXIT_EXCEPTION?  If using KVM_EXIT_EXCEPTION,
 * current qemu process will "hang" (killable but Ctrl+C not working),
 * so use KVM_EXIT_SHUTDOWN here temporarily.
 */
static int panic_hv(struct kvm_vcpu *vcpu, const char *fmt, ...)
{
	char panic_buf[256];
	struct pt_regs *regs;
	va_list ap;
	int i;

	va_start(ap, fmt);
	vsnprintf(panic_buf, sizeof(panic_buf), fmt, ap);
	va_end(ap);
	pr_err("KVM guest panic (vcpu %d) - %s\n", vcpu->vcpu_id, panic_buf);

	/* Show guest os info */
	regs = &vcpu->arch.regs;
	for (i = 0; i < 17; i++)
		pr_err(" r%-2d: "REGFMT" r%-2d: "REGFMT" r%-2d: "REGFMT"\n",
		       i, regs->regs[i], i+18, regs->regs[i+18],
		       i+36, regs->regs[i+36]);
	pr_err(" r18: "REGFMT" r35: "REGFMT" tp : "REGFMT"\n",
	       regs->regs[18], regs->regs[35], regs->tp);
	pr_err(" sp : "REGFMT" lr : "REGFMT"\n", regs->sp, regs->lr);
	pr_err(" pc : "REGFMT" ex1: %ld     faultnum: %ld\n",
	       regs->pc, regs->ex1, regs->faultnum);

	/* Show host os info */
	pr_err("\nKVM stack in the host:\n");
	dump_stack();

	/* Shut down the guest os */
	pr_err("Shutting down guest.\n");
	vcpu->run->exit_reason = KVM_EXIT_SHUTDOWN;
	return 0;
}

/* Copied from virt/kvm/kvm_main.c */
static int next_segment(unsigned long len, int offset)
{
	if (len > PAGE_SIZE - offset)
		return PAGE_SIZE - offset;
	else
		return len;
}

static int kvm_read_guest_va(struct kvm_vcpu *vcpu, unsigned long gva,
			     void *data, unsigned long len)
{
	struct kvm *kvm = vcpu->kvm;
	int seg;
	int offset = offset_in_page(gva);
	int ret;

	while ((seg = next_segment(len, offset)) != 0) {
		struct kvm_translation tr;
		tr.linear_address = gva;
		kvm_arch_vcpu_ioctl_translate(vcpu, &tr);
		if (!tr.valid)
			return -EFAULT;
		ret = kvm_read_guest_page(kvm, PFN_DOWN(tr.physical_address),
					  data, offset, seg);
		if (ret < 0)
			return ret;
		offset = 0;
		len -= seg;
		data += seg;
		gva += seg;
	}
	return 0;
}

static int kvm_write_guest_va(struct kvm_vcpu *vcpu, unsigned long gva,
			      const void *data, unsigned long len)
{
	struct kvm *kvm = vcpu->kvm;
	int seg;
	int offset = offset_in_page(gva);
	int ret;

	while ((seg = next_segment(len, offset)) != 0) {
		struct kvm_translation tr;
		tr.linear_address = gva;
		kvm_arch_vcpu_ioctl_translate(vcpu, &tr);
		if (!tr.valid)
			return -EFAULT;
		ret = kvm_write_guest_page(kvm, PFN_DOWN(tr.physical_address),
					   data, offset, seg);
		if (ret < 0)
			return ret;
		offset = 0;
		len -= seg;
		data += seg;
		gva += seg;
	}
	return 0;
}

static int kvm_clear_guest_va(struct kvm_vcpu *vcpu, unsigned long gva,
			      unsigned long len)
{
	struct kvm *kvm = vcpu->kvm;
	int seg;
	int offset = offset_in_page(gva);
	int ret;

	while ((seg = next_segment(len, offset)) != 0) {
		struct kvm_translation tr;
		tr.linear_address = gva;
		kvm_arch_vcpu_ioctl_translate(vcpu, &tr);
		if (!tr.valid)
			return -EFAULT;
		ret = kvm_clear_guest_page(kvm, PFN_DOWN(tr.physical_address),
					   offset, seg);
		if (ret < 0)
			return ret;
		offset = 0;
		len -= seg;
		gva += seg;
	}
	return 0;
}

/*
 * The following functions are emulation functions for various
 * hypervisor system calls (i.e. hv_*()). Return value:
 *   1 if the host os can emulate it completely.
 *   < 0 if errors occur and then qemu will handle them.
 *   0 if qemu emulation is needed.
 * In both the < 0 and the == 0 cases, exit reason should
 * be set for qemu handling.
 */

/* generic handler for hypercall which needs user (QEMU) to handle. */
static int kvm_deliver_to_user(struct kvm_vcpu *vcpu)
{
	vcpu->run->exit_reason = KVM_EXIT_HYPERCALL;
	return 0;
}

/* handler for illegal hypercall */
static int kvm_emulate_illegal(struct kvm_vcpu *vcpu)
{
	return panic_hv(vcpu, "Illegal kvm hypercall: %ld",
			(unsigned long)vcpu->arch.regs.regs[10]);
}

static int kvm_emulate_hv_init(struct kvm_vcpu *vcpu)
{
	int version = vcpu->arch.regs.regs[0];
	int chip_num = vcpu->arch.regs.regs[1];
	int chip_rev_num = vcpu->arch.regs.regs[2];
	int client_pl = vcpu->arch.regs.regs[3];

	if (client_pl != 1)
		return panic_hv(vcpu, "Guest is requesting PL %d, but KVM"
				" guests must request PL 1.\n"
				"Reconfigure your guest with KVM_GUEST set.\n",
				client_pl);

	if (version != HV_VERSION)
		return panic_hv(vcpu, "Client built for hv version %d, but"
				" this hv is version %d\n",
				version, HV_VERSION);

	if (chip_num != TILE_CHIP)
		return panic_hv(vcpu, "Client built for chip %d, but this"
				" hardware is chip %d\n",
				chip_num, TILE_CHIP);

	if (chip_rev_num != TILE_CHIP_REV)
		return panic_hv(vcpu, "Client built for chip rev %d, but this"
				" hardware is chip rev %d\n",
				chip_rev_num, TILE_CHIP_REV);

	return 1;
}

static int kvm_emulate_hv_sysconf(struct kvm_vcpu *vcpu)
{
	HV_SysconfQuery query = (HV_SysconfQuery)vcpu->arch.regs.regs[0];
	long rc;

	switch (query) {
	case HV_SYSCONF_PAGE_SIZE_SMALL:
		rc = PAGE_SIZE;
		break;

	case HV_SYSCONF_PAGE_SIZE_LARGE:
		rc = HPAGE_SIZE;
		break;

	case HV_SYSCONF_VALID_PAGE_SIZES:
#if PAGE_SHIFT == 16
		rc = HV_CTX_PG_SM_64K;
#elif PAGE_SHIFT == 14
		rc = HV_CTX_PG_SM_16K;
#else
# error Fix hv_sysconf emulation for new page size
#endif
		break;

	case HV_SYSCONF_PAGE_SIZE_JUMBO:
		rc = 0;  /* FIXME add super page support */
		break;

	case HV_SYSCONF_CPU_SPEED:
	case HV_SYSCONF_CPU_TEMP:
	case HV_SYSCONF_BOARD_TEMP:
		rc = hv_sysconf(query);
		break;

	default:
		rc = -EINVAL;
		break;
	}

	vcpu->arch.regs.regs[0] = rc;
	return 1;
}

static int kvm_emulate_hv_confstr(struct kvm_vcpu *vcpu)
{
	HV_SysconfQuery query = (HV_SysconfQuery)vcpu->arch.regs.regs[0];
	long buflen = vcpu->arch.regs.regs[2];
	char hvbuf[256];
	const char *p;
	long rc;

	switch (query) {

	/* For hardware attributes, just pass to the hypervisor. */
	case HV_CONFSTR_BOARD_PART_NUM:
	case HV_CONFSTR_BOARD_SERIAL_NUM:
	case HV_CONFSTR_CHIP_SERIAL_NUM:
	case HV_CONFSTR_BOARD_REV:
	case HV_CONFSTR_CHIP_MODEL:
	case HV_CONFSTR_BOARD_DESC:
	case HV_CONFSTR_MEZZ_PART_NUM:
	case HV_CONFSTR_MEZZ_SERIAL_NUM:
	case HV_CONFSTR_MEZZ_REV:
	case HV_CONFSTR_MEZZ_DESC:
	case HV_CONFSTR_SWITCH_CONTROL:
	case HV_CONFSTR_CHIP_REV:
	case HV_CONFSTR_CPUMOD_PART_NUM:
	case HV_CONFSTR_CPUMOD_SERIAL_NUM:
	case HV_CONFSTR_CPUMOD_REV:
	case HV_CONFSTR_CPUMOD_DESC:
		rc = hv_confstr(query, (HV_VirtAddr)hvbuf, sizeof(hvbuf));
		if (rc > sizeof(hvbuf)) {
			/* Not the best answer, but very unlikely anyway. */
			rc = sizeof(hvbuf);
			hvbuf[sizeof(hvbuf)-1] = '\0';
		}
		p = hvbuf;
		break;

	/* For hypervisor version info, just report the kernel version. */
	case HV_CONFSTR_HV_SW_VER:
		p = UTS_RELEASE;
		break;
	case HV_CONFSTR_HV_CONFIG:
	case HV_CONFSTR_HV_CONFIG_VER:
		p = "";
		break;

	default:
		rc = HV_EINVAL;
		goto done;
	}

	rc = strlen(p) + 1;  /* include NUL */
	if (kvm_write_guest_va(vcpu, vcpu->arch.regs.regs[1],
			       p, min(rc, buflen)))
		rc = HV_EFAULT;

done:
	vcpu->arch.regs.regs[0] = rc;
	return 1;
}

static int kvm_emulate_hv_get_rtc(struct kvm_vcpu *vcpu)
{
	HV_RTCTime *hvtm = (HV_RTCTime *) &vcpu->arch.regs.regs[0];
	struct rtc_time tm;
	struct timeval tv;

	do_gettimeofday(&tv);
	rtc_time_to_tm(tv.tv_sec, &tm);
	hvtm->tm_sec = tm.tm_sec;
	hvtm->tm_min = tm.tm_min;
	hvtm->tm_hour = tm.tm_hour;
	hvtm->tm_mday = tm.tm_mday;
	hvtm->tm_mon = tm.tm_mon;
	hvtm->tm_year = tm.tm_year;
	hvtm->flags = 0;

	return 1;
}

static int kvm_emulate_hv_set_rtc(struct kvm_vcpu *vcpu)
{
	/* Do nothing here. */
	pr_warn("hv_set_rtc() will not work in kvm guest\n");
	return 1;
}

static int kvm_emulate_hv_inquire_virtual(struct kvm_vcpu *vcpu)
{
	int idx = vcpu->arch.regs.regs[0];
	HV_VirtAddrRange *var = (HV_VirtAddrRange *)&vcpu->arch.regs.regs[0];

	switch (idx) {
	case 0:
		var->start =                  0UL;
		var->size  =       0x20000000000UL;
		break;
	case 1:
		var->start = 0xFFFFFFFF80000000UL;
		var->size  =         0x80000000UL;
		break;
	default:
		var->start =                  0UL;
		var->size  =                  0UL;
		break;
	}

	return 1;
}

/* Give all the ASIDs to the guest; we flush the whole TLB anyway. */
static int kvm_emulate_hv_inquire_asid(struct kvm_vcpu *vcpu)
{
	int idx = vcpu->arch.regs.regs[0];
	HV_ASIDRange *var = (HV_ASIDRange *)&vcpu->arch.regs.regs[0];

	if (idx == 0) {
		var->start = min_asid;
		var->size = max_asid - min_asid + 1;
	} else {
		var->start = 0;
		var->size = 0;
	}

	return 1;
}

static int kvm_emulate_hv_inquire_topology(struct kvm_vcpu *vcpu)
{
	HV_Topology *tp;
	int cpus;

	/* Depends on the definition of struct HV_Topology */
	tp = (HV_Topology *)&vcpu->arch.regs.regs[0];

	cpus = atomic_read(&vcpu->kvm->online_vcpus);
	tp->coord.x = vcpu->vcpu_id;
	tp->coord.y = 0;
	tp->width = cpus;
	tp->height = 1;

	return 1;
}

static int xy_to_vcpu(struct kvm *kvm, int x, int y)
{
	if (y != 0 || x < 0 || x >= atomic_read(&kvm->online_vcpus))
		return -1;
	return x;
}

/*
 * The primary vcpu is the one that initially runs while the others
 * all block.  It is the only that is allowed to call hv_start_all_tiles().
 * The other cpus are secondary.
 */
static bool is_secondary_vcpu(struct kvm_vcpu *vcpu)
{
	return vcpu->vcpu_id != 0;
}

static int kvm_emulate_hv_start_all_tiles(struct kvm_vcpu *vcpu)
{
	struct completion *c = &vcpu->kvm->arch.smp_start;
	if (is_secondary_vcpu(vcpu) || completion_done(c))
		return panic_hv(vcpu, "start_all_tiles() called again");
	complete_all(c);
	return 1;
}

static int kvm_emulate_hv_physaddr_read64(struct kvm_vcpu *vcpu)
{
	gpa_t gpa = vcpu->arch.regs.regs[0];
	HV_PTE *access = (HV_PTE *) &vcpu->arch.regs.regs[1];
	gfn_t gfn;
	pfn_t pfn;
	hpa_t hpa;

	gfn = gpa_to_gfn(gpa);
	pfn = gfn_to_pfn(vcpu->kvm, gfn);
	if (is_error_pfn(pfn))
		return panic_hv(vcpu, "bogus PA %llx in physaddr_write64()",
			 gpa);
	hpa = pfn_to_hpa(pfn) | (gpa & ~PAGE_MASK);

	vcpu->arch.regs.regs[0] = hv_physaddr_read64(hpa, *access);

	return 1;
}

static int kvm_emulate_hv_physaddr_write64(struct kvm_vcpu *vcpu)
{
	gpa_t gpa = vcpu->arch.regs.regs[0];
	HV_PTE *access = (HV_PTE *)vcpu->arch.regs.regs[1];
	uint64_t val = vcpu->arch.regs.regs[2];
	gfn_t gfn;
	pfn_t pfn;
	hpa_t hpa;

	gfn = gpa_to_gfn(gpa);
	pfn = gfn_to_pfn(vcpu->kvm, gfn);
	if (is_error_pfn(pfn))
		return panic_hv(vcpu, "bogus PA %llx in physaddr_write64()",
			 gpa);
	hpa = pfn_to_hpa(pfn) | (gpa & ~PAGE_MASK);

	hv_physaddr_write64(hpa, *access, val);

	return 1;
}

static int kvm_emulate_hv_register_message_state(struct kvm_vcpu *vcpu)
{
	/* Do we care about the argument msgstate? */
	vcpu->arch.regs.regs[0] = HV_OK;

	return 1;
}

/*
 * NOTE: we may coalesce multiple messages with the same tag to the
 * same recepient.  Currently the only messages used by Linux are
 * start/stop cpu (where coalescing is OK), and the smp_call_function()
 * IPI message tag.  In the latter case we rely on the generic
 * smp_call_function code to properly handle this, and since it only
 * uses the IPI as a way to wake up the generic list-walking code,
 * it's OK if we coalesce several IPI deliveries before the recipient
 * core takes action.
 */
static int kvm_emulate_hv_send_message(struct kvm_vcpu *vcpu)
{
	struct kvm *kvm = vcpu->kvm;
	struct kvm_vcpu *vcpui;
	HV_Recipient recip[NR_CPUS];
	HV_Recipient *recips = (HV_Recipient *)vcpu->arch.regs.regs[0];
	int nrecip = vcpu->arch.regs.regs[1];
	int buflen = vcpu->arch.regs.regs[3];
	int sent, vcpu_id, tag;

	/* NOTE: we only support the Linux usage of buflen == sizeof(int). */
	if (unlikely(buflen != sizeof(int) ||
		     nrecip >= atomic_read(&kvm->online_vcpus))) {
		vcpu->arch.regs.regs[0] = HV_EINVAL;
		return 1;
	}

	/* Get the buf info */
	if (kvm_read_guest_va(vcpu, vcpu->arch.regs.regs[2],
			      &tag, sizeof(tag))) {
		vcpu->arch.regs.regs[0] = HV_EFAULT;
		return 1;
	}

	/* Range-check the tag value. */
	if (tag < 0 || tag >= MAX_MSG_TAG) {
		vcpu->arch.regs.regs[0] = HV_EFAULT;
		return 1;
	}

	/* Get all the recipients */
	if (kvm_read_guest_va(vcpu, (unsigned long)recips, &recip,
			      nrecip * sizeof(HV_Recipient))) {
		vcpu->arch.regs.regs[0] = HV_EFAULT;
		return 1;
	}

	for (sent = 0; sent < nrecip; sent++) {
		if (recip[sent].state != HV_TO_BE_SENT)
			continue;
		vcpu_id = xy_to_vcpu(kvm, recip[sent].x, recip[sent].y);
		if (unlikely(vcpu_id < 0 || vcpu_id == vcpu->vcpu_id)) {
			recip[sent].state = HV_BAD_RECIP;
			continue;
		}
		vcpui = kvm_get_vcpu(kvm, vcpu_id);
		set_bit(tag, &vcpui->arch.pending_msgs);
		kvm_vcpu_kick(vcpui);
		recip[sent].state = HV_SENT;
	}

	if (kvm_write_guest_va(vcpu, (unsigned long)recips, &recip,
			       nrecip * sizeof(HV_Recipient))) {
		vcpu->arch.regs.regs[0] = HV_EFAULT;
		return 1;
	}

	vcpu->arch.regs.regs[0] = sent;

	return 1;
}

static int kvm_emulate_hv_receive_message(struct kvm_vcpu *vcpu)
{
	HV_RcvMsgInfo *rmi = (HV_RcvMsgInfo *)&vcpu->arch.regs.regs[0];
	int buflen = vcpu->arch.regs.regs[3];
	int tag;

	/* Currently we only support messages from other tiles. */
	rmi->source = HV_MSG_TILE;

	if (buflen <= sizeof(int)) {
		rmi->msglen = HV_E2BIG;
		return 1;
	}

	tag = find_first_bit(&vcpu->arch.pending_msgs, MAX_MSG_TAG);
	if (tag >= MAX_MSG_TAG) {
		/* No more messages */
		rmi->msglen = 0;
		return 1;
	}

	if (kvm_write_guest_va(vcpu, vcpu->arch.regs.regs[2],
			       &tag, sizeof(int))) {
		rmi->msglen = HV_EFAULT;
		return 1;
	}

	/*
	 * This clear_bit could race with a set_bit as another core
	 * delivers a new smp_function_call to this core.  However,
	 * the smp_function_call code will have set up the additional
	 * smp_function_call data on the kernel's list prior to
	 * raising the interrupt, so even if we lose the new
	 * interrupt due to the race, we still haven't dispatched
	 * to the original interrupt handler, and when we do, it
	 * will find both smp_function_calls waiting for it, so the
	 * race is harmless.  This is consistent with the fact that
	 * the generic code is trying to support pretty much
	 * arbitrary architecture-dependent IPI semantics, so it
	 * is very conservative about what it assumes.
	 *
	 * Also note that we only clear_bit on the core that owns
	 * the mask, so there's no race condition caused by the
	 * find_first_bit above and the clear_bit here, since once
	 * a bit is found it will stay set until this point.
	 */
	clear_bit(tag, &vcpu->arch.pending_msgs);
	rmi->msglen = sizeof(int);
	return 1;
}

static int kvm_emulate_hv_inquire_context(struct kvm_vcpu *vcpu)
{
	HV_Context *ctx = (HV_Context *) &vcpu->arch.regs.regs[0];

	*ctx = hv_inquire_guest_context();

	return 1;
}

static int kvm_emulate_hv_inquire_tiles(struct kvm_vcpu *vcpu)
{
	struct kvm *kvm = vcpu->kvm;
	HV_InqTileSet set = vcpu->arch.regs.regs[0];
	unsigned long gva = vcpu->arch.regs.regs[1];
	int length = vcpu->arch.regs.regs[2];
	struct cpumask mask = CPU_MASK_NONE;
	int cpus, i, retval, bytes2copy, bytes2zero;

	switch (set) {
	case HV_INQ_TILES_AVAIL:
	case HV_INQ_TILES_HFH_CACHE:
	case HV_INQ_TILES_LOTAR:
		cpus = atomic_read(&kvm->online_vcpus);
		for (i = 0; i < cpus; ++i)
			cpumask_set_cpu(i, &mask);
		break;
	case HV_INQ_TILES_SHARED:
		break;
	default:
		retval = HV_EINVAL;
		goto done;
	}

	bytes2copy = (length > sizeof(mask)) ? sizeof(mask) : length;
	bytes2zero = length - bytes2copy;

	if (kvm_write_guest_va(vcpu, gva, &mask, bytes2copy)) {
		retval = HV_EFAULT;
		goto done;
	}

	if (kvm_clear_guest_va(vcpu, gva + bytes2copy, bytes2zero)) {
		retval = HV_EFAULT;
		goto done;
	}

	retval = HV_OK;
done:
	vcpu->arch.regs.regs[0] = retval;
	return 1;
}

static int kvm_emulate_hv_get_ipi_pte(struct kvm_vcpu *vcpu)
{
	HV_Coord vtarget = *(HV_Coord *)&vcpu->arch.regs.regs[0];
	int pl = (int) vcpu->arch.regs.regs[1];
	struct kvm_vcpu *target_vcpu;
	int vcpu_id;

	vcpu_id = vtarget.x;
	if (pl != GUEST_PL || vtarget.y != 0 || vcpu_id < 0 ||
	    vcpu_id >= atomic_read(&vcpu->kvm->online_vcpus)) {
		vcpu->arch.regs.regs[0] = HV_EINVAL;
		return 1;
	}

	target_vcpu = kvm_get_vcpu(vcpu->kvm, vcpu_id);
	if (kvm_write_guest_va(vcpu, vcpu->arch.regs.regs[2],
			    &target_vcpu->arch.ipi_gpte, sizeof(pte_t))) {
		vcpu->arch.regs.regs[0] = HV_EFAULT;
		return 1;
	}

	vcpu->arch.regs.regs[0] = HV_OK;

	return 1;
}

struct kvm_vcpu *ipi_vcpu_lookup(struct kvm *kvm, unsigned long gpa)
{
	struct kvm_vcpu *vcpui;
	unsigned long idx;

	kvm_for_each_vcpu(idx, vcpui, kvm)
		if (vcpui->arch.ipi_gpa == gpa)
			return vcpui;

	return NULL;
}

/*
 * Most page faults will be downcall-ed from hv to and be handled directly
 * by either guest os or host os. This function is used to handle the
 * rest cases.
 */
static int handle_mmio(struct kvm_vcpu *vcpu)
{
	struct kvm *kvm = vcpu->kvm;
	struct kvm_translation tr;
	struct kvm_vcpu *ipi_vcpu;

	tr.linear_address = (__u64) vcpu->arch.fault_addr;
	kvm_arch_vcpu_ioctl_translate(vcpu, &tr);
	if (!tr.valid)
		return 0;

	/* ipi PTE for rescheduling interrupt? */
	ipi_vcpu = ipi_vcpu_lookup(kvm, tr.physical_address);
	if (!ipi_vcpu)
		return 0;

	set_bit(IRQ_RESCHEDULE, &ipi_vcpu->arch.ipi_events);
	kvm_vcpu_kick(ipi_vcpu);

	/* Juke the PC past the store instruction. */
	vcpu->arch.regs.pc += 8;
	return 1;
}

static int kvm_emulate_hv_set_pte_super_shift(struct kvm_vcpu *vcpu)
{
	/*
	 * We do not expect this call in guest so far. At least guest os
	 * should just follow host os instead of *set*. Besides,
	 * hv_set_pte_super_shift() will not be called in guest os with
	 * current guest os setting.
	 */
	vcpu->arch.regs.regs[0] = HV_EINVAL;

	return 1;
}

static int kvm_emulate_hv_set_speed(struct kvm_vcpu *vcpu)
{
	HV_SetSpeed *hvss = (HV_SetSpeed *) &vcpu->arch.regs.regs[0];

	hvss->new_speed = HV_EPERM;
	hvss->end_cycle = 0;
	hvss->delta_ns = 0;

	return 1;
}

static int (*hcall_handlers[KVM_NUM_HCALLS])(struct kvm_vcpu *vcpu) = {
	HCALL_DEFS
};

static int kvm_handle_exit(struct kvm_vcpu *vcpu)
{
	unsigned long hcall_idx;

	switch (vcpu->run->exit_reason) {
	case KVM_EXIT_HYPERCALL:
		hcall_idx = vcpu->arch.regs.regs[10];
		if (unlikely(hcall_idx >= KVM_NUM_HCALLS ||
			     hcall_handlers[hcall_idx] == NULL))
			return kvm_emulate_illegal(vcpu);

		/* Juke us past the swint0 when we return. */
		vcpu->arch.regs.pc += 8;

		return hcall_handlers[hcall_idx](vcpu);

	case KVM_EXIT_MMIO:
		if (handle_mmio(vcpu))
			return 1;
		return panic_hv(vcpu, "Out-of-bounds client memory access");

	case KVM_EXIT_AGAIN:
		return 1;

	default:
		return 0;
	}
}

static void kvm_kick_func(void *info)
{
	struct kvm_vcpu *vcpu = info;

	/* If this is not the thread that we expect, just return. */
	if (unlikely(vcpu->pid != get_task_pid(current, PIDTYPE_PID)))
		return;

	/* Setting this flag will cause a vmexit instead of a vmresume. */
	set_thread_flag(TIF_VIRT_EXIT);
}

/* Note this function has been a standard kvm interface in latest Linux. */
void kvm_vcpu_kick(struct kvm_vcpu *vcpu)
{
	int me, cpu;

	/* If it is waiting in kvm_vcpu_block(), wake it up. */
	if (waitqueue_active(&vcpu->wq))
		wake_up_interruptible(&vcpu->wq);

	/* If we are kicking our own vcpu, make sure we vmexit. */
	if (vcpu == current_thread_info()->vcpu) {
		set_thread_flag(TIF_VIRT_EXIT);
		return;
	}

	/*
	 * If the vcpu is running the guest, interrupt its cpu,
	 * causing it to vmexit by setting TIF_VIRT_EXIT.  Note we can
	 * race with a guest already doing a vmexit, but that is benign.
	 */
	cpu = vcpu->cpu;
	me = get_cpu();
	if (cpu != me && (unsigned) cpu < nr_cpu_ids && cpu_online(cpu))
		if (!test_and_set_bit(KVM_REQ_KICK, &vcpu->requests))
			smp_call_function_single(cpu, kvm_kick_func, vcpu, 0);
	put_cpu();
}
EXPORT_SYMBOL_GPL(kvm_vcpu_kick);

/*
 * Any interrupt that would normally be handled by the host at PL2
 * needs to be reassigned to the guest at PL1 as we enter.
 *
 * The TLB interrupts remain handled by the hypervisor and are downcalled
 * to the appropriate host or guest as necessary.
 *
 * FIXME: We don't give the UDN interrupts for now; at some point we
 * plan to allow an option to pin the vcpus and report the true
 * geometry to the guest, at which point passing the UDN access would
 * make sense.
 *
 * FIXME: For now we don't pass the profiling interrupts to the guest,
 * and instead require profiling be run in the host; we should be able
 * to support guest-level profiling pretty easily, but we need to
 * think about whether there are vcpu migration issues there.
 */
static void kvm_grant_mpls(void)
{
	__insn_mtspr(SPR_MPL_SWINT_1_SET_1, 1);
	__insn_mtspr(SPR_MPL_ILL_SET_1, 1);
	__insn_mtspr(SPR_MPL_GPV_SET_1, 1);
	__insn_mtspr(SPR_MPL_ILL_TRANS_SET_1, 1);
	__insn_mtspr(SPR_MPL_UNALIGN_DATA_SET_1, 1);
}

static void kvm_ungrant_mpls(void)
{
	__insn_mtspr(SPR_MPL_SWINT_1_SET_2, 1);
	__insn_mtspr(SPR_MPL_ILL_SET_2, 1);
	__insn_mtspr(SPR_MPL_GPV_SET_2, 1);
	__insn_mtspr(SPR_MPL_ILL_TRANS_SET_2, 1);
	__insn_mtspr(SPR_MPL_UNALIGN_DATA_SET_2, 1);
}

/*
 * There is lots of state that is (for the non-virtualized case) held
 * permanently in SPRs, or that is in any case not context-switched.
 * The next two routines switch in and out all the SPR state.
 *
 * We try to fix the timer so that when we restart, we fix up the
 * timer value so that will fire at the correct wall-clock time even
 * if we have been scheduled out for a little bit.  This may also
 * mean we end up firing it immediately on return, and suffer a
 * timer delay in the guest.
 */
static void kvm_save_sprs(struct kvm_vcpu *vcpu)
{
	vcpu->arch.timer_control = __insn_mfspr(SPR_AUX_TILE_TIMER_CONTROL);
	vcpu->arch.vmexit_cycles = get_cycles();

#define SAVE_SPR(x) vcpu->arch.sregs.x = __insn_mfspr(SPR_ ## x)
	FOR_EACH_GUEST_SPR(SAVE_SPR);
#undef SAVE_SPR
}

static void kvm_restore_sprs(struct kvm_vcpu *vcpu)
{
	unsigned long count = vcpu->arch.timer_control;
	unsigned long underflow =
		(count >> SPR_AUX_TILE_TIMER_CONTROL__UNDERFLOW_SHIFT) & 1;
	unsigned long disabled =
		(count >> SPR_AUX_TILE_TIMER_CONTROL__DISABLE_SHIFT) & 1;

	if (!disabled) {
		unsigned long delta = get_cycles() - vcpu->arch.vmexit_cycles;
		count &= SPR_AUX_TILE_TIMER_CONTROL__COUNT_MASK;
		underflow |= delta > count;
		count -= delta;
		count &= SPR_AUX_TILE_TIMER_CONTROL__COUNT_MASK;
		count |= (underflow << SPR_AUX_TILE_TIMER_CONTROL__UNDERFLOW_SHIFT);
	}
	__insn_mtspr(SPR_AUX_TILE_TIMER_CONTROL, count);

#define RESTORE_SPR(x) __insn_mtspr(SPR_ ## x, vcpu->arch.sregs.x)
	FOR_EACH_GUEST_SPR(RESTORE_SPR);
#undef RESTORE_SPR
}

/*
 * When entering the guest, we need to eliminate any PL0 translations
 * that were in use by qemu, since the guest's PL0 translations will
 * be different.  We also flush PL1 translations in case there have
 * been changes to the virtualization page table, etc.
 *
 * FIXME: Add a way to just flush PL0/PL1, or just flush below
 * the host PAGE_OFFSET, or add vpid support, etc.
 */
static void kvm_guest_context_enter(struct kvm_vcpu *vcpu)
{
	HV_Context *ctx;
	pgd_t *vpgdir;
	pte_t *ptep;
	int rc;

	/* Install virtualization context */
	vpgdir = vcpu->kvm->arch.vpgd;
	BUG_ON(vpgdir == NULL);
	ptep = virt_to_pte(NULL, (unsigned long)vpgdir);
	rc = hv_install_virt_context(__pa(vpgdir), *ptep, 0, 0);
	WARN_ON_ONCE(rc < 0);

	/* Install guest context */
	ctx = &vcpu->arch.guest_context;
	rc = hv_install_guest_context(ctx->page_table, ctx->access,
				      ctx->asid, ctx->flags);
	WARN_ONCE(rc < 0, "install_guest_context(%#llx,%#llx,%#x,%#x): %d\n",
		  ctx->page_table, ctx->access.val,
		  ctx->asid, ctx->flags, rc);

	hv_flush_all(0);
}

/*
 * De-install the virtualization context so we take faults below the
 * host Linux PL in the normal manner going forward.
 *
 * We flush all the TLB mappings as we exit the guest, since the
 * guest has been using the ASIDs as it pleases, and may have installed
 * incompatible mappings for qemu's process as well.  Note that we don't
 * worry about host-PL interrupts that occur while the guest is running,
 * on the assumption that such interrupts can't touch userspace
 * addresses legally anyway.
 *
 * NOTE: we may want to add a hypervisor call to just flush mappings
 * below PL2 and use that here instead.
 */
static void kvm_guest_context_exit(struct kvm_vcpu *vcpu)
{
	int rc;

	/* Remember guest context */
	vcpu->arch.guest_context = hv_inquire_guest_context();

	/* Disable virtualization context */
	rc = hv_install_virt_context(HV_CTX_NONE, hv_pte(0), 0, 0);
	WARN_ON_ONCE(rc < 0);

	/* Flush everything in the TLB. */
	hv_flush_all(0);
}

static void kvm_inject_interrupts(struct kvm_vcpu *vcpu)
{
	/*
	 * Capture current set of ipi_events.  We might race with
	 * another thread adding an event, but if so we'll just miss
	 * it on this go-around and see it next time.
	 */
	vcpu->arch.sregs.IPI_EVENT_1 |= __insn_exch(&vcpu->arch.ipi_events, 0);

	/*
	 * Note: We could set PC and EX1 for the guest os to jump
	 * directly to the INT_MESSAGE_RCV_DWNCL handler if the interrupt
	 * is unmasked and the guest is not at PL1 with ICS set.
	 * But in fact it's about as fast to just set INTCTRL_1_STATUS
	 * here and then run the short INTCTRL_1 handler in the guest.
	 */
	vcpu->arch.sregs.INTCTRL_1_STATUS = (vcpu->arch.pending_msgs != 0);
}

static void kvm_tile_run(struct kvm_vcpu *vcpu)
{
	struct thread_info *ti = current_thread_info();
	unsigned long prev_k_0 = __insn_mfspr(SPR_SYSTEM_SAVE_K_0);

	/*
	 * Disable interrupts while we set up the guest state.
	 * This way, if we race with another core trying to tell us
	 * to fix up our guest state, we will take the kick only as
	 * we actually try to enter the guest, and instead we will
	 * vmexit and end up retrying.
	 */
	local_irq_disable();
	kvm_guest_context_enter(vcpu);
	clear_bit(KVM_REQ_KICK, &vcpu->requests);
	ti->vcpu = vcpu;
	vcpu->cpu = get_cpu();
	kvm_inject_interrupts(vcpu);
	kvm_grant_mpls();
	kvm_restore_sprs(vcpu);

	/* Calling this function irets into the guest. */
	kvm_vmresume(&vcpu->arch.regs, &vcpu->arch.host_sp);

	/* We resume here due to a call to kvm_vmexit. */
	__insn_mtspr(SPR_SYSTEM_SAVE_K_0, prev_k_0);

	vcpu->cpu = -1;
	put_cpu();
	ti->vcpu = NULL;
	set_bit(KVM_REQ_KICK, &vcpu->requests);
	vcpu->run->ready_for_interrupt_injection = 1;
	kvm_ungrant_mpls();
	kvm_save_sprs(vcpu);
	__insn_mtspr(SPR_INTERRUPT_MASK_1, -1UL);
	kvm_guest_context_exit(vcpu);
	local_irq_enable();
}

static int __vcpu_run(struct kvm_vcpu *vcpu, struct kvm_run *kvm_run)
{
	int r = 1;

	while (r > 0) {
		kvm_guest_enter();
		kvm_tile_run(vcpu);
		kvm_guest_exit();

		r = kvm_handle_exit(vcpu);
		/*
		 * <0: error for userspace.
		 * =0: QEMU to handle.
		 * >0: host os can handle it fully.
		 */
		if (r <= 0)
			break;

		if (signal_pending(current)) {
			vcpu->run->exit_reason = KVM_EXIT_INTR;
			r = -EINTR;
			break;
		}

#ifdef CONFIG_HOMECACHE
		if (current_thread_info()->homecache_cpu !=
		    smp_processor_id()) {
			/* Do homecache migration when returning to qemu. */
			vcpu->run->exit_reason = KVM_EXIT_INTR;
			r = -EINTR;
			break;
		}
#endif

		kvm_resched(vcpu);
	}

	return r;
}

int kvm_arch_vcpu_ioctl_run(struct kvm_vcpu *vcpu, struct kvm_run *kvm_run)
{
	int r;
	sigset_t sigsaved;

	/* Secondary cpus must wait until they are told they can start. */
	if (vcpu->arch.suspended) {
		struct completion *c = &vcpu->kvm->arch.smp_start;
		if (wait_for_completion_interruptible(c))
			return -EINTR;
		vcpu->arch.suspended = 0;
	}

	if (vcpu->sigset_active)
		sigprocmask(SIG_SETMASK, &vcpu->sigset, &sigsaved);

	r = __vcpu_run(vcpu, kvm_run);

	if (vcpu->sigset_active)
		sigprocmask(SIG_SETMASK, &sigsaved, NULL);

	return r;
}

int kvm_arch_init(void *opaque)
{
	return 0;
}

void kvm_arch_exit(void)
{
}

int kvm_arch_vcpu_init(struct kvm_vcpu *vcpu)
{
	int i;
	unsigned long resv_gfn_start;
	struct kvm_memory_slot *s;
	struct kvm *kvm = vcpu->kvm;

	if (!kvm->arch.resv_gpa_start) {
		resv_gfn_start = 0;

		for (i = 0; i < KVM_USER_MEM_SLOTS; i++) {
			s = &kvm->memslots->memslots[i];

			if (!s->npages)
				continue;

			if ((s->base_gfn + s->npages) > resv_gfn_start)
				resv_gfn_start = s->base_gfn + s->npages;
		}

		kvm->arch.resv_gpa_start = PFN_PHYS(resv_gfn_start);
	}

	/* Initialize to enter fake PA=VA mode in hypervisor. */
	vcpu->arch.guest_context.page_table = HV_CTX_NONE;

	vcpu->arch.ipi_gpa =
		kvm->arch.resv_gpa_start + (vcpu->vcpu_id * PAGE_SIZE);
	vcpu->arch.ipi_gpte =
		pfn_pte(PFN_DOWN(vcpu->arch.ipi_gpa), PAGE_KERNEL);

	/* Mark the core suspended if it is not the boot cpu. */
	vcpu->arch.suspended = is_secondary_vcpu(vcpu);

	return 0;
}

void kvm_arch_vcpu_uninit(struct kvm_vcpu *vcpu)
{
}

void kvm_arch_vcpu_load(struct kvm_vcpu *vcpu, int cpu)
{
	/* Notify simulator that this task handles this vcpu. */
	sim_set_vcpu(vcpu->vcpu_id);
}

void kvm_arch_vcpu_put(struct kvm_vcpu *vcpu)
{
	sim_clear_vcpu();
}

struct kvm_vcpu *kvm_arch_vcpu_create(struct kvm *kvm, unsigned int id)
{
	/* FIXME: some archs set up a cache for these structs? */
	struct kvm_vcpu *vcpu = kzalloc(sizeof(struct kvm_vcpu), GFP_KERNEL);
	int rc;

	if (!vcpu)
		return ERR_PTR(-ENOMEM);

	rc = kvm_vcpu_init(vcpu, kvm, id);
	if (rc) {
		kfree(vcpu);
		return ERR_PTR(rc);
	}

	return vcpu;
}

int kvm_arch_vcpu_setup(struct kvm_vcpu *vcpu)
{
	memset(&vcpu->arch.regs, 0, sizeof(struct pt_regs));
	memset(&vcpu->arch.sregs, 0, sizeof(struct pt_regs));
	vcpu->arch.sregs.IPI_MASK_1 = -1UL;
	vcpu->arch.sregs.INTERRUPT_MASK_1 = -1UL;
	vcpu->arch.sregs.INTERRUPT_VECTOR_BASE_1 = 0xfd000000;
	return 0;
}

int kvm_arch_vcpu_postcreate(struct kvm_vcpu *vcpu)
{
	return 0;
}

void kvm_arch_vcpu_destroy(struct kvm_vcpu *vcpu)
{
	kvm_vcpu_uninit(vcpu);
	kfree(vcpu);
}

void kvm_arch_vcpu_free(struct kvm_vcpu *vcpu)
{
	return kvm_arch_vcpu_destroy(vcpu);
}

int kvm_arch_hardware_enable(void *garbage)
{
	return 0;
}

void kvm_arch_hardware_disable(void *garbage)
{
}

int kvm_arch_hardware_setup(void)
{
	return 0;
}

void kvm_arch_hardware_unsetup(void)
{
}

void kvm_arch_check_processor_compat(void *rtn)
{
}

int kvm_arch_vcpu_runnable(struct kvm_vcpu *vcpu)
{
	return 0;
}

int kvm_arch_init_vm(struct kvm *kvm, unsigned long type)
{
	if (type)
		return -EINVAL;

	init_completion(&kvm->arch.smp_start);
	return 0;
}

void kvm_arch_destroy_vm(struct kvm *kvm)
{
	struct kvm_vcpu *vcpu;
	int i;

	kvm_for_each_vcpu(i, vcpu, kvm)
		kvm_arch_vcpu_free(vcpu);

	/* Seems to be unnecessary? */
	mutex_lock(&kvm->lock);
	for (i = 0; i < atomic_read(&kvm->online_vcpus); i++)
		kvm->vcpus[i] = NULL;

	atomic_set(&kvm->online_vcpus, 0);
	mutex_unlock(&kvm->lock);

	/* FIXME: release all the pmds and ptes as well! */
	if (kvm->arch.vpgd)
		pgd_free(kvm->mm, kvm->arch.vpgd);
}

void kvm_arch_sync_events(struct kvm *kvm)
{
}

int kvm_cpu_has_pending_timer(struct kvm_vcpu *vcpu)
{
	return 0;
}

/* Called from guest hv glue via swint0 traps. */
void kvm_do_hypervisor_call(struct pt_regs *regs, int fault_num)
{
	/* Hypercalls are only valid from PL1. */
	if (EX1_PL(regs->ex1) != 0) {
		kvm_trigger_vmexit(regs, KVM_EXIT_HYPERCALL);
		/*NORETURN*/
	}
	do_trap(regs, fault_num, 0);
}

void kvm_do_vpgtable_miss(struct pt_regs *regs, int fault_num,
			  unsigned long fault_addr, unsigned long write)
{
	struct kvm_vcpu *vcpu = current_thread_info()->vcpu;
	BUG_ON(vcpu == NULL);
	vcpu->arch.fault_addr = fault_addr;
	kvm_trigger_vmexit(regs, KVM_EXIT_MMIO);
	/*NORETURN*/
}

void kvm_do_vguest_fatal(struct pt_regs *regs, int fault_num)
{
	kvm_trigger_vmexit(regs, KVM_EXIT_SHUTDOWN);
	/*NORETURN*/
}

void kvm_trigger_vmexit(struct pt_regs *regs, int exit_reason)
{
	struct kvm_vcpu *vcpu = current_thread_info()->vcpu;
	vcpu->run->exit_reason = exit_reason;
	vcpu->arch.regs = *regs;
	vcpu->arch.regs.flags = PT_FLAGS_CALLER_SAVES | PT_FLAGS_RESTORE_REGS;
	kvm_vmexit(vcpu->arch.host_sp);
	/*NORETURN*/
}

static int __init kvm_tile_init(void)
{
	return kvm_init(NULL, sizeof(struct kvm_vcpu),
			__alignof__(struct kvm_vcpu), THIS_MODULE);
}

static void __exit kvm_tile_exit(void)
{
	kvm_exit();
}

module_init(kvm_tile_init);
module_exit(kvm_tile_exit);
