/*
 * Versatile Express Serial Power Controller (SPC) support
 *
 * Copyright (C) 2013 ARM Ltd.
 *
 * Authors: Sudeep KarkadaNagesha <sudeep.karkadanagesha@arm.com>
 *          Achin Gupta           <achin.gupta@arm.com>
 *          Lorenzo Pieralisi     <lorenzo.pieralisi@arm.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/device.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/of_address.h>
#include <linux/slab.h>
#include <linux/vexpress.h>

#include <asm/cacheflush.h>

#define SPCLOG "vexpress-spc: "

/* SCC conf registers */
#define A15_CONF		0x400
#define SYS_INFO		0x700

/* SPC registers base */
#define SPC_BASE		0xB00

/* SPC wake-up IRQs status and mask */
#define WAKE_INT_MASK		(SPC_BASE + 0x24)
#define WAKE_INT_RAW		(SPC_BASE + 0x28)
#define WAKE_INT_STAT		(SPC_BASE + 0x2c)
/* SPC power down registers */
#define A15_PWRDN_EN		(SPC_BASE + 0x30)
#define A7_PWRDN_EN		(SPC_BASE + 0x34)
/* SPC per-CPU mailboxes */
#define A15_BX_ADDR0		(SPC_BASE + 0x68)
#define A7_BX_ADDR0		(SPC_BASE + 0x78)

/* wake-up interrupt masks */
#define GBL_WAKEUP_INT_MSK	(0x3 << 10)

/* TC2 static dual-cluster configuration */
#define MAX_CLUSTERS		2

struct ve_spc_drvdata {
	void __iomem *baseaddr;
	/*
	 * A15s cluster identifier
	 * It corresponds to A15 processors MPIDR[15:8] bitfield
	 */
	u32 a15_clusid;
};

static struct ve_spc_drvdata *info;

static inline bool cluster_is_a15(u32 cluster)
{
	return cluster == info->a15_clusid;
}

/**
 * ve_spc_global_wakeup_irq()
 *
 * Function to set/clear global wakeup IRQs. Not protected by locking since
 * it might be used in code paths where normal cacheable locks are not
 * working. Locking must be provided by the caller to ensure atomicity.
 *
 * @set: if true, global wake-up IRQs are set, if false they are cleared
 */
void ve_spc_global_wakeup_irq(bool set)
{
	u32 reg;

	reg = readl_relaxed(info->baseaddr + WAKE_INT_MASK);

	if (set)
		reg |= GBL_WAKEUP_INT_MSK;
	else
		reg &= ~GBL_WAKEUP_INT_MSK;

	writel_relaxed(reg, info->baseaddr + WAKE_INT_MASK);
}

/**
 * ve_spc_cpu_wakeup_irq()
 *
 * Function to set/clear per-CPU wake-up IRQs. Not protected by locking since
 * it might be used in code paths where normal cacheable locks are not
 * working. Locking must be provided by the caller to ensure atomicity.
 *
 * @cluster: mpidr[15:8] bitfield describing cluster affinity level
 * @cpu: mpidr[7:0] bitfield describing cpu affinity level
 * @set: if true, wake-up IRQs are set, if false they are cleared
 */
void ve_spc_cpu_wakeup_irq(u32 cluster, u32 cpu, bool set)
{
	u32 mask, reg;

	if (cluster >= MAX_CLUSTERS)
		return;

	mask = 1 << cpu;

	if (!cluster_is_a15(cluster))
		mask <<= 4;

	reg = readl_relaxed(info->baseaddr + WAKE_INT_MASK);

	if (set)
		reg |= mask;
	else
		reg &= ~mask;

	writel_relaxed(reg, info->baseaddr + WAKE_INT_MASK);
}

/**
 * ve_spc_set_resume_addr() - set the jump address used for warm boot
 *
 * @cluster: mpidr[15:8] bitfield describing cluster affinity level
 * @cpu: mpidr[7:0] bitfield describing cpu affinity level
 * @addr: physical resume address
 */
void ve_spc_set_resume_addr(u32 cluster, u32 cpu, u32 addr)
{
	void __iomem *baseaddr;

	if (cluster >= MAX_CLUSTERS)
		return;

	if (cluster_is_a15(cluster))
		baseaddr = info->baseaddr + A15_BX_ADDR0 + (cpu << 2);
	else
		baseaddr = info->baseaddr + A7_BX_ADDR0 + (cpu << 2);

	writel_relaxed(addr, baseaddr);
}

/**
 * ve_spc_get_nr_cpus() - get number of cpus in a cluster
 *
 * @cluster: mpidr[15:8] bitfield describing cluster affinity level
 *
 * Return: > 0 number of cpus in the cluster
 *         or 0 if cluster number invalid
 */
u32 ve_spc_get_nr_cpus(u32 cluster)
{
	u32 val;

	if (cluster >= MAX_CLUSTERS)
		return 0;

	val = readl_relaxed(info->baseaddr + SYS_INFO);
	val = cluster_is_a15(cluster) ? (val >> 16) : (val >> 20);
	return val & 0xf;
}

/**
 * ve_spc_powerdown()
 *
 * Function to enable/disable cluster powerdown. Not protected by locking
 * since it might be used in code paths where normal cacheable locks are not
 * working. Locking must be provided by the caller to ensure atomicity.
 *
 * @cluster: mpidr[15:8] bitfield describing cluster affinity level
 * @enable: if true enables powerdown, if false disables it
 */
void ve_spc_powerdown(u32 cluster, bool enable)
{
	u32 pwdrn_reg;

	if (cluster >= MAX_CLUSTERS)
		return;

	pwdrn_reg = cluster_is_a15(cluster) ? A15_PWRDN_EN : A7_PWRDN_EN;
	writel_relaxed(enable, info->baseaddr + pwdrn_reg);
}

static const struct of_device_id ve_spc_ids[] __initconst = {
	{ .compatible = "arm,vexpress-spc,v2p-ca15_a7" },
	{ .compatible = "arm,vexpress-spc" },
	{},
};

static int __init ve_spc_probe(void)
{
	int ret;
	struct device_node *node = of_find_matching_node(NULL, ve_spc_ids);

	if (!node)
		return -ENODEV;

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info) {
		pr_err(SPCLOG "unable to allocate mem\n");
		return -ENOMEM;
	}

	info->baseaddr = of_iomap(node, 0);
	if (!info->baseaddr) {
		pr_err(SPCLOG "unable to ioremap memory\n");
		ret = -ENXIO;
		goto mem_free;
	}

	info->a15_clusid = readl_relaxed(info->baseaddr + A15_CONF) & 0xf;

	/*
	 * Multi-cluster systems may need this data when non-coherent, during
	 * cluster power-up/power-down. Make sure driver info reaches main
	 * memory.
	 */
	sync_cache_w(info);
	sync_cache_w(&info);
	pr_info("vexpress-spc loaded at %p\n", info->baseaddr);
	return 0;

mem_free:
	kfree(info);
	return ret;
}

/**
 * ve_spc_init()
 *
 * Function exported to manage pre early_initcall initialization.
 * SPC code is needed very early in the boot process to bring CPUs out of
 * reset and initialize power management back-end so an init interface is
 * provided to platform code to allow early initialization. The init
 * interface can be removed as soon as the DT layer and platform bus allow
 * platform device creation and probing before SMP boot.
 */
int __init ve_spc_init(void)
{
	static int ve_spc_init_status __initdata = -EAGAIN;

	if (ve_spc_init_status == -EAGAIN)
		ve_spc_init_status = ve_spc_probe();

	return ve_spc_init_status;
}
early_initcall(ve_spc_init);
