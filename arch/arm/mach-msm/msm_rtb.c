/*
 * Copyright (c) 2012, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <mach/memory.h>
#include <asm-generic/sizes.h>
#include <stdarg.h>
#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <mach/msm_rtb.h>
#include <linux/export.h>
#include <mach/system.h>
#include <linux/atomic.h>
#include <asm/io.h>
#include <linux/memory_alloc.h>
#include <linux/slab.h>

#define SENTINEL_BYTE_1	0xFF
#define SENTINEL_BYTE_2 0xAA
#define SENTINEL_BYTE_3	0xFF

/* Write
 * 1) 3 bytes sentinel
 * 2) 1 bytes of log type
 * 3) 4 bytes of where the caller came from
 * 4) 4 bytes index
 * 4) 4 bytes extra data from the caller
 *
 * Total = 16 bytes.
 */
struct msm_rtb_layout {
	char sentinel[3];
	char log_type;
	void *caller;
	unsigned long idx;
	void *data;
} __attribute__ ((__packed__));


struct msm_rtb_state {
	struct msm_rtb_layout *rtb;
	unsigned long phys;
	int nentries; /* Default 4k / msm_rtb_layout */
	int size;
	int init;
};


#if defined(CONFIG_MSM_RTB_SEPARATE_CPUS)
DEFINE_PER_CPU(atomic_t, cpu_cnt);
#else
static atomic_t msm_rtb_idx;
#endif

struct msm_rtb_state msm_rtb = {
	.size = SZ_4K,
};

static void emit_sentinel(struct msm_rtb_layout *start)
{
	start->sentinel[0] = SENTINEL_BYTE_1;
	start->sentinel[1] = SENTINEL_BYTE_2;
	start->sentinel[2] = SENTINEL_BYTE_3;
}

static void write_type(enum uncached_log_type log_type,
			struct msm_rtb_layout *start)
{
	start->log_type = (char)log_type;
}

static void write_caller(void *caller, struct msm_rtb_layout *start)
{
	start->caller = caller;
}

static void write_idx(unsigned long idx,
				struct msm_rtb_layout *start)
{
	start->idx = idx;
}

static void write_data(void *data, struct msm_rtb_layout *start)
{
	start->data = data;
}

static int __init set_buffer_size(char *p)
{
	int s;

	s = memparse(p, NULL);
	msm_rtb.size = ALIGN(s, SZ_4K);
	return 0;
}
early_param("msm_rtb_size", set_buffer_size);


void __log_uncached_data(enum uncached_log_type log_type, void *caller,
				void *data)
{
	int i;
	struct msm_rtb_layout *start;
#if defined(CONFIG_MSM_RTB_SEPARATE_CPUS)
	atomic_t *index;
	int cpu;
#endif

	if (!msm_rtb.init)
		return;

#if defined(CONFIG_MSM_RTB_SEPARATE_CPUS)
	/*
	 * ideally we would use get_cpu but this is a close enough
	 * approximation for our purposes.
	 */

	cpu = raw_smp_processor_id();

	index = &per_cpu(cpu_cnt, cpu);

	i = atomic_add_return(num_possible_cpus(), index);
	i -= num_possible_cpus();
#else
	i = atomic_inc_return(&msm_rtb_idx);
	i--;
#endif

	start = &msm_rtb.rtb[i & (msm_rtb.nentries - 1)];

	emit_sentinel(start);
	write_type(log_type, start);
	write_caller(caller, start);
	write_idx(i, start);
	write_data(data, start);
	mb();
}
EXPORT_SYMBOL(__log_uncached_data);

noinline void log_uncached_data(enum uncached_log_type log_type, void *data)
{
	return __log_uncached_data(log_type, __builtin_return_address(0), data);
}
EXPORT_SYMBOL(log_uncached_data);

int msm_rtb_init(void)
{
#if defined(CONFIG_MSM_RTB_SEPARATE_CPUS)
	unsigned int cpu;
#endif

	if (msm_rtb.size <= 0 || msm_rtb.size > SZ_1M)
		return -EINVAL;

	/*
	 * The ioremap call is made separately to store the physical
	 * address of the buffer. This is necessary for cases where
	 * the only way to access the buffer is a physical address.
	 */
	msm_rtb.phys = allocate_contiguous_ebi_nomap(msm_rtb.size, SZ_4K);

	if (!msm_rtb.phys)
		return -ENOMEM;

	msm_rtb.rtb = ioremap(msm_rtb.phys, msm_rtb.size);

	if (!msm_rtb.rtb) {
		free_contiguous_memory_by_paddr(msm_rtb.phys);
		return -ENOMEM;
	}

	msm_rtb.nentries = msm_rtb.size / sizeof(struct msm_rtb_layout);

	/* Round this down to a power of 2 */
	msm_rtb.nentries = __rounddown_pow_of_two(msm_rtb.nentries);

	memset(msm_rtb.rtb, 0, msm_rtb.size);


#if defined(CONFIG_MSM_RTB_SEPARATE_CPUS)
	for_each_possible_cpu(cpu) {
		atomic_t *a = &per_cpu(cpu_cnt, cpu);
		atomic_set(a, cpu);
	}
#else
	atomic_set(&msm_rtb_idx, 0);
#endif


	msm_rtb.init = 1;
	return 0;
}
module_init(msm_rtb_init)
