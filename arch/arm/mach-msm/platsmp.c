/*
 *  Copyright (C) 2002 ARM Ltd.
 *  All Rights Reserved
 *  Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/cpumask.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>

#include <asm/hardware/gic.h>
#include <asm/cacheflush.h>
#include <asm/cputype.h>
#include <asm/mach-types.h>

#include <mach/smp.h>
#include <mach/hardware.h>
#include <mach/msm_iomap.h>

#include "pm.h"
#include "scm-boot.h"
#include "acpuclock.h"

#define SECONDARY_CPU_WAIT_MS 10

int pen_release = -1;

static inline int get_core_count(void)
{
	/* 1 + the PART[1:0] field of MIDR */
	return ((read_cpuid_id() >> 4) & 3) + 1;
}

/* Initialize the present map (cpu_set(i, cpu_present_map)). */
void __init platform_smp_prepare_cpus(unsigned int max_cpus)
{
	int i;

	for (i = 0; i < max_cpus; i++)
		cpu_set(i, cpu_present_map);
}

void smp_init_cpus(void)
{
	unsigned int i, ncores = get_core_count();

	for (i = 0; i < ncores; i++)
		cpu_set(i, cpu_possible_map);
}

static void release_secondary(void)
{
	void *base_ptr;

	/* KraitMP or ScorpionMP ? */
	if ((read_cpuid_id() & 0xFF0) >> 4 != 0x2D) {
		base_ptr = ioremap_nocache(0x02098000, SZ_4K);
		if (base_ptr) {
			writel_relaxed(0x10, base_ptr+0x04);
			writel_relaxed(0x80, base_ptr+0x04);
			dsb();
			iounmap(base_ptr);
		}
	} else {
		base_ptr = ioremap_nocache(0x00902000, SZ_4K*2);
		if (base_ptr) {
			writel_relaxed(0x0, base_ptr+0x15A0);
			dmb();
			writel_relaxed(0x0, base_ptr+0xD80);
			writel_relaxed(0x3, base_ptr+0xE64);
			dsb();
			iounmap(base_ptr);
		}
	}
}

/* Executed by primary CPU, brings other CPUs out of reset. Called at boot
   as well as when a CPU is coming out of shutdown induced by echo 0 >
   /sys/devices/.../cpuX.
*/
int boot_secondary(unsigned int cpu, struct task_struct *idle)
{
	static int cold_boot_done;
	int cnt = 0;
	int ret;

	pr_debug("Starting secondary CPU %d\n", cpu);

	/* Set preset_lpj to avoid subsequent lpj recalculations */
	preset_lpj = loops_per_jiffy;

	if (cold_boot_done == false) {
		ret = scm_set_boot_addr((void *)
					virt_to_phys(msm_secondary_startup),
					SCM_FLAG_COLDBOOT_CPU1);
		if (ret == 0)
			release_secondary();
		else
			printk(KERN_DEBUG "Failed to set secondary core boot "
					  "address\n");
		cold_boot_done = true;
	}

	pen_release = cpu;
	dmac_flush_range((void *)&pen_release,
			 (void *)(&pen_release + sizeof(pen_release)));
	__asm__("sev");
	dsb();

	/* Use smp_cross_call() to send a soft interrupt to wake up
	 * the other core.
	 */
	smp_cross_call(cpumask_of(cpu), 1);

	while (pen_release != 0xFFFFFFFF) {
		dmac_inv_range((void *)&pen_release,
			       (void *)(&pen_release+sizeof(pen_release)));
		msleep_interruptible(1);
		if (cnt++ >= SECONDARY_CPU_WAIT_MS)
			break;
	}

	return 0;
}

/* Initialization routine for secondary CPUs after they are brought out of
 * reset.
*/
void platform_secondary_init(unsigned int cpu)
{
	pr_debug("CPU%u: Booted secondary processor\n", cpu);

#ifdef CONFIG_HOTPLUG_CPU
	WARN_ON(msm_pm_platform_secondary_init(cpu));
#endif

	trace_hardirqs_off();

	/* Edge trigger PPIs except AVS_SVICINT and AVS_SVICINTSWDONE */
	writel(0xFFFFD7FF, MSM_QGIC_DIST_BASE + GIC_DIST_CONFIG + 4);

	/* RUMI does not adhere to GIC spec by enabling STIs by default.
	 * Enable/clear is supposed to be RO for STIs, but is RW on RUMI.
	 */
	if (!machine_is_msm8x60_sim())
		writel(0x0000FFFF, MSM_QGIC_DIST_BASE + GIC_DIST_ENABLE_SET);

	gic_secondary_init(0);

	/* Setup acpuclock for non-primary CPU. */
	acpuclock_secondary_init();
}
