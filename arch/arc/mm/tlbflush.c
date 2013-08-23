/*
 * Copyright (C) 2013-2014 Synopsys, Inc. (www.synopsys.com)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/sched.h>
#include <asm/tlbflush.h>

struct tlb_args {
	struct vm_area_struct *ta_vma;
	unsigned long ta_start;
	unsigned long ta_end;
};

static inline void ipi_flush_tlb_all(void *ignored)
{
	local_flush_tlb_all();
}

static inline void ipi_flush_tlb_mm(void *mm)
{
	local_flush_tlb_mm((struct mm_struct *)mm);
}

static inline void ipi_flush_tlb_page(void *arg)
{
	struct tlb_args *ta = (struct tlb_args *)arg;

	local_flush_tlb_page(ta->ta_vma, ta->ta_start);
}

static inline void ipi_flush_tlb_range(void *arg)
{
	struct tlb_args *ta = (struct tlb_args *)arg;

	local_flush_tlb_range(ta->ta_vma, ta->ta_start, ta->ta_end);
}

static inline void ipi_flush_tlb_kernel_range(void *arg)
{
	struct tlb_args *ta = (struct tlb_args *)arg;

	local_flush_tlb_kernel_range(ta->ta_start, ta->ta_end);
}

void flush_tlb_all(void)
{
	on_each_cpu(ipi_flush_tlb_all, NULL, 1);
}

void flush_tlb_mm(struct mm_struct *mm)
{
	on_each_cpu_mask(mm_cpumask(mm), ipi_flush_tlb_mm, mm, 1);
}

void flush_tlb_page(struct vm_area_struct *vma, unsigned long uaddr)
{
	struct tlb_args ta = {
		.ta_vma = vma,
		.ta_start = uaddr
	};

	on_each_cpu_mask(mm_cpumask(vma->vm_mm), ipi_flush_tlb_page, &ta, 1);
}

void flush_tlb_range(struct vm_area_struct *vma, unsigned long start,
		     unsigned long end)
{
	struct tlb_args ta = {
		.ta_vma = vma,
		.ta_start = start,
		.ta_end = end
	};

	on_each_cpu_mask(mm_cpumask(vma->vm_mm), ipi_flush_tlb_range, &ta, 1);
}

void flush_tlb_kernel_range(unsigned long start, unsigned long end)
{
	struct tlb_args ta = {
		.ta_start = start,
		.ta_end = end
	};

	on_each_cpu(ipi_flush_tlb_kernel_range, &ta, 1);
}
