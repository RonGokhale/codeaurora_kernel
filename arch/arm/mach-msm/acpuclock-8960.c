/* Copyright (c) 2011, Code Aurora Forum. All rights reserved.
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/cpufreq.h>
#include <linux/cpu.h>
#include <asm/mach-types.h>

#include <asm/cpu.h>

#include <mach/board.h>
#include <mach/msm_iomap.h>

#include "acpuclock.h"

#define dprintk(msg...) \
	cpufreq_debug_printk(CPUFREQ_DEBUG_DRIVER, "cpufreq-msm", msg)

/*
 * Source IDs.
 * These must be negative to not overlap with the source IDs
 * used by the 8x60 local clock driver.
 */
#define PLL_8			 0
#define HFPLL			-1
#define QSB			-2

/* Mux source selects. */
#define PRI_SRC_SEL_SEC_SRC	0
#define PRI_SRC_SEL_HFPLL	1
#define PRI_SRC_SEL_HFPLL_DIV2	2
#define SEC_SRC_SEL_QSB		0

/* HFPLL registers offsets. */
#define HFPLL_MODE		0x00
#define HFPLL_CONFIG_CTL	0x04
#define HFPLL_L_VAL		0x08
#define HFPLL_M_VAL		0x0C
#define HFPLL_N_VAL		0x10
#define HFPLL_DROOP_CTL		0x14
#define HFPLL_STATUS		0x1C

/* CP15 L2 indirect addresses. */
#define L2CPMR_IADDR		0x500
#define L2CPUCPMR_IADDR		0x501

#define STBY_KHZ		1

enum scalables {
	CPU0 = 0,
	CPU1,
	L2,
	NUM_SCALABLES
};

static const void * __iomem const hf_pll_base[] = {
	[CPU0]	= MSM_HFPLL_BASE + 0x200,
	[CPU1]	= MSM_HFPLL_BASE + 0x300,
	[L2]	= MSM_HFPLL_BASE + 0x400,
};

static const uint32_t l2cpmr_iaddr[NUM_SCALABLES] = {
	[CPU0] = L2CPUCPMR_IADDR,
	[CPU1] = L2CPUCPMR_IADDR,
	[L2]   = L2CPMR_IADDR,
};

static const void * __iomem const aux_clk_sel[NUM_SCALABLES] = {
	[CPU0] = MSM_ACC0_BASE + 0x014,
	[CPU1] = MSM_ACC1_BASE + 0x014,
	[L2]   = MSM_APCS_GCC_BASE  + 0x028,
};

struct core_speed {
	unsigned int		khz;
	int			src;
	unsigned int		pri_src_sel;
	unsigned int		sec_src_sel;
	unsigned int		pll_l_val;
};

struct l2_level {
	struct core_speed	speed;
};

struct acpu_level {
	unsigned int		use_for_scaling;
	struct core_speed	speed;
	struct l2_level		*l2_level;
};

static struct clock_state {
	struct core_speed	*current_speed[NUM_SCALABLES];
	spinlock_t		l2_lock;
	struct mutex		lock;
	uint32_t		acpu_switch_time_us;
	uint32_t		vdd_switch_time_us;
} drv_state;

#define L2(x) (&l2_freq_tbl[(x)])
static struct l2_level l2_freq_tbl[] = {
	[0]  = { {STBY_KHZ, QSB,   0, 0, 0x00 } },
	[1]  = { {  384000, PLL_8, 0, 2, 0x00 } },
	[2]  = { {  432000, HFPLL, 2, 0, 0x20 } },
	[3]  = { {  486000, HFPLL, 2, 0, 0x24 } },
	[4]  = { {  594000, HFPLL, 1, 0, 0x16 } },
	[5]  = { {  648000, HFPLL, 1, 0, 0x18 } },
	[6]  = { {  702000, HFPLL, 1, 0, 0x1A } },
	[7]  = { {  756000, HFPLL, 1, 0, 0x1C } },
	[8]  = { {  810000, HFPLL, 1, 0, 0x1E } },
	[9]  = { {  864000, HFPLL, 1, 0, 0x20 } },
	[10] = { {  918000, HFPLL, 1, 0, 0x22 } },
	[11] = { {  972000, HFPLL, 1, 0, 0x24 } },
	[12] = { { 1026000, HFPLL, 1, 0, 0x26 } },
	[13] = { { 1080000, HFPLL, 1, 0, 0x28 } },
	[14] = { { 1134000, HFPLL, 1, 0, 0x2A } },
	[15] = { { 1188000, HFPLL, 1, 0, 0x2C } },
	[16] = { { 1242000, HFPLL, 1, 0, 0x2E } },
	[17] = { { 1296000, HFPLL, 1, 0, 0x30 } },
	[18] = { { 1350000, HFPLL, 1, 0, 0x32 } },
	[19] = { { 1404000, HFPLL, 1, 0, 0x34 } },
	[20] = { { 1458000, HFPLL, 1, 0, 0x36 } },
	[21] = { { 1512000, HFPLL, 1, 0, 0x38 } },
	[22] = { { 1566000, HFPLL, 1, 0, 0x3A } },
	[23] = { { 1620000, HFPLL, 1, 0, 0x3C } },
	[24] = { { 1674000, HFPLL, 1, 0, 0x3E } },
};

static struct acpu_level acpu_freq_tbl[] = {
	{ 0, {STBY_KHZ, QSB,   0, 0, 0x00 }, L2(1)  },
	{ 1, {  384000, PLL_8, 0, 2, 0x00 }, L2(1)  },
	{ 1, {  432000, HFPLL, 2, 0, 0x20 }, L2(2)  },
	{ 1, {  486000, HFPLL, 2, 0, 0x24 }, L2(3)  },
	{ 1, {  594000, HFPLL, 1, 0, 0x16 }, L2(4)  },
	{ 1, {  648000, HFPLL, 1, 0, 0x18 }, L2(5)  },
	{ 1, {  702000, HFPLL, 1, 0, 0x1A }, L2(6)  },
	{ 0, {  756000, HFPLL, 1, 0, 0x1C }, L2(7)  },
	{ 0, {  810000, HFPLL, 1, 0, 0x1E }, L2(8)  },
	{ 0, {  864000, HFPLL, 1, 0, 0x20 }, L2(9)  },
	{ 0, {  918000, HFPLL, 1, 0, 0x22 }, L2(10) },
	{ 0, { 0 } }
};

unsigned long acpuclk_get_rate(int cpu)
{
	return drv_state.current_speed[cpu]->khz;
}

uint32_t acpuclk_get_switch_time(void)
{
	return drv_state.acpu_switch_time_us;
}

unsigned long acpuclk_power_collapse(void)
{
	int ret = acpuclk_get_rate(smp_processor_id());
	acpuclk_set_rate(smp_processor_id(), STBY_KHZ, SETRATE_PC);
	return ret;
}

unsigned long acpuclk_wait_for_irq(void)
{
	int ret = acpuclk_get_rate(smp_processor_id());
	acpuclk_set_rate(smp_processor_id(), STBY_KHZ, SETRATE_SWFI);
	return ret;
}

/* Read an 'indirectly' addressed L2 CP15 register. */
static uint32_t readl_cp15_l2ind(uint32_t addr)
{
	uint32_t regval;

	/*
	 * TODO: CP15 registers are not emulated on RUMI3.
	 * Remove this check if/when they are.
	 */
	if (machine_is_msm8960_rumi3())
		return 0;

	asm volatile ("mcr     p15, 3, %[l2cpsler], c15, c0, 6\n\t"
		      "mrc     p15, 3, %[l2cpdr],   c15, c0, 7\n\t"
			: [l2cpdr]"=r" (regval)
			: [l2cpsler]"r" (addr)
			: "cc"
	);
	return regval;
}

/* Write an 'indirectly' addressed L2 CP15 register. */
static void writel_cp15_l2ind(uint32_t regval, uint32_t addr)
{
	/*
	 * TODO: CP15 registers are not emulated on RUMI3.
	 * Remove this check if/when they are.
	 */
	if (machine_is_msm8960_rumi3())
		return;

	dsb();
	asm volatile ("mcr     p15, 3, %[l2cpsler], c15, c0, 6\n\t"
		      "mcr     p15, 3, %[l2cpdr],   c15, c0, 7\n\t"
			:
			: [l2cpsler]"r" (addr), [l2cpdr]"r" (regval)
			: "cc"
	);
	isb();
	BUG_ON(readl_cp15_l2ind(addr) != regval);
}

/* Get the selected source on primary MUX. */
static int get_pri_clk_src(enum scalables id)
{
	uint32_t regval = 0;

	BUG_ON(id >= NUM_SCALABLES);
	BUG_ON(id != smp_processor_id() && id != L2);

	regval = readl_cp15_l2ind(l2cpmr_iaddr[id]);
	return regval & 0x3;
}

/* Set the selected source on primary MUX. */
static void set_pri_clk_src(enum scalables id, uint32_t pri_src_sel)
{
	uint32_t regval;

	BUG_ON(id >= NUM_SCALABLES);
	BUG_ON(id != smp_processor_id() && id != L2);
	BUG_ON(pri_src_sel > 3);

	regval = readl_cp15_l2ind(l2cpmr_iaddr[id]);
	regval &= ~0x3;
	regval |= (pri_src_sel & 0x3);
	writel_cp15_l2ind(regval, l2cpmr_iaddr[id]);
}

/* Set the selected source on secondary MUX. */
static void set_sec_clk_src(enum scalables id, uint32_t sec_src_sel)
{
	uint32_t regval;

	BUG_ON(id >= NUM_SCALABLES);
	BUG_ON(id != smp_processor_id() && id != L2);
	BUG_ON(sec_src_sel > 3);

	regval = readl_cp15_l2ind(l2cpmr_iaddr[id]);
	regval &= ~(0x3 << 2);
	regval |= ((sec_src_sel & 0x3) << 2);
	writel_cp15_l2ind(regval, l2cpmr_iaddr[id]);
}

/* Enable an already-configured HFPLL. */
static void hfpll_enable(enum scalables id)
{
	BUG_ON(id >= NUM_SCALABLES);

	/* Disable PLL bypass mode. */
	writel_relaxed(0x2, hf_pll_base[id] + HFPLL_MODE);

	/*
	 * H/W requires a 5us delay between disabling the bypass and
	 * de-asserting the reset. Delay 10us just to be safe.
	 */
	dsb();
	udelay(10);

	/* De-assert active-low PLL reset. */
	writel_relaxed(0x6, hf_pll_base[id] + HFPLL_MODE);

	/* Enable PLL output. */
	writel_relaxed(0x7, hf_pll_base[id] + HFPLL_MODE);

	/*
	 * TODO: HFPLL status bits are not emulated on RUMI3 or SIM.
	 * Remove this check if/when they are.
	 */
	if (machine_is_msm8960_sim() || machine_is_msm8960_rumi3())
		return;

	/* Wait until PLL is enabled. */
	while (!readl_relaxed(hf_pll_base[id] + HFPLL_STATUS))
		cpu_relax();
}

/* Disable a HFPLL for power-savings or while its being reprogrammed. */
static void hfpll_disable(enum scalables id)
{
	BUG_ON(id >= NUM_SCALABLES);

	/*
	 * Disable the PLL output, disable test mode, enable
	 * the bypass mode, and assert the reset.
	 */
	writel_relaxed(0, hf_pll_base[id] + HFPLL_MODE);
}

/* Program the HFPLL rate. Assumes HFPLL is already disabled. */
static void hfpll_set_rate(enum scalables id, struct core_speed *tgt_s)
{
	BUG_ON(id >= NUM_SCALABLES);
	writel_relaxed(tgt_s->pll_l_val, hf_pll_base[id] + HFPLL_L_VAL);
}

/* Return the L2 speed that should be applied. */
static struct core_speed *compute_l2_speed(unsigned int voting_cpu,
					   struct l2_level *l2_level)
{
	static struct core_speed *l2_vote[NR_CPUS];
	struct core_speed *new_s, *vote_s = &l2_level->speed;
	int cpu;

	/* Bounds check. */
	BUG_ON(l2_level >= (l2_freq_tbl + ARRAY_SIZE(l2_freq_tbl)));

	/* Find max L2 speed vote. */
	l2_vote[voting_cpu] = vote_s;
	new_s = &l2_freq_tbl->speed;
	for_each_online_cpu(cpu)
		new_s = max(new_s, l2_vote[cpu]);

	return new_s;
}

/* Set the CPU or L2 clock speed. */
static void set_speed(enum scalables id, struct core_speed *tgt_s,
		      enum setrate_reason reason)
{
	struct core_speed *strt_s = drv_state.current_speed[id];

	BUG_ON(id >= NUM_SCALABLES);

	if (tgt_s == strt_s)
		return;

	if (strt_s->src == HFPLL && tgt_s->src == HFPLL) {
		/* Move CPU to QSB source. */
		set_sec_clk_src(id, SEC_SRC_SEL_QSB);
		set_pri_clk_src(id, PRI_SRC_SEL_SEC_SRC);

		/* Program CPU HFPLL. */
		hfpll_disable(id);
		hfpll_set_rate(id, tgt_s);
		hfpll_enable(id);

		/* Move CPU to HFPLL source. */
		set_pri_clk_src(id, tgt_s->pri_src_sel);
	} else if (strt_s->src == HFPLL && tgt_s->src != HFPLL) {
		/* TODO: Enable source. */
		/*
		 * If responding to CPU_DEAD we must be running on another
		 * CPU.  Therefore, we can't access the downed CPU's CP15
		 * clock MUX registers from here and can't change clock sources.
		 * Just turn off the PLL- since the CPU is down already, halting
		 * its clock should be safe.
		 */
		if (reason != SETRATE_HOTPLUG || id == L2) {
			set_sec_clk_src(id, tgt_s->sec_src_sel);
			set_pri_clk_src(id, tgt_s->pri_src_sel);
		}
		hfpll_disable(id);
	} else if (strt_s->src != HFPLL && tgt_s->src == HFPLL) {
		hfpll_set_rate(id, tgt_s);
		hfpll_enable(id);
		/*
		 * If responding to CPU_UP_PREPARE, we can't change CP15
		 * registers for the CPU that's coming up since we're not
		 * running on that CPU.  That's okay though, since the MUX
		 * source was not changed on the way down, either.
		 */
		if (reason != SETRATE_HOTPLUG || id == L2)
			set_pri_clk_src(id, tgt_s->pri_src_sel);
		/* TODO: Disable source. */
	} else if (strt_s->src != HFPLL && tgt_s->src != HFPLL) {
		/* TODO: Enable source. */
		if (reason != SETRATE_HOTPLUG || id == L2)
			set_sec_clk_src(id, tgt_s->sec_src_sel);
		/* TODO: Disable source. */
	} else {
		BUG();
	}

	drv_state.current_speed[id] = tgt_s;
}

/* Set the CPU's clock rate and adjust the L2 rate, if appropriate. */
int acpuclk_set_rate(int cpu, unsigned long rate, enum setrate_reason reason)
{
	struct core_speed *strt_acpu_s, *tgt_acpu_s, *tgt_l2_s;
	struct acpu_level *tgt;
	unsigned long flags;
	int rc = 0;

	BUG_ON(cpu >= 2);

	if (cpu > num_possible_cpus()) {
		rc = -EINVAL;
		goto out;
	}

	if (reason == SETRATE_CPUFREQ || reason == SETRATE_HOTPLUG)
		mutex_lock(&drv_state.lock);

	strt_acpu_s = drv_state.current_speed[cpu];

	/* Return early if rate didn't change. */
	if (rate == strt_acpu_s->khz)
		goto out;

	/* Find target frequency. */
	for (tgt = acpu_freq_tbl; tgt->speed.khz != 0; tgt++) {
		if (tgt->speed.khz == rate) {
			tgt_acpu_s = &tgt->speed;
			break;
		}
	}
	if (tgt->speed.khz == 0) {
		rc = -EINVAL;
		goto out;
	}

	dprintk("Switching from ACPU%d rate %u KHz -> %u KHz\n",
		cpu, strt_acpu_s->khz, tgt_acpu_s->khz);

	/* Set the CPU speed. */
	set_speed(cpu, tgt_acpu_s, reason);

	/* Update the L2 vote and apply the rate change. */
	spin_lock_irqsave(&drv_state.l2_lock, flags);
	tgt_l2_s = compute_l2_speed(cpu, tgt->l2_level);
	set_speed(L2, tgt_l2_s, reason);
	spin_unlock_irqrestore(&drv_state.l2_lock, flags);

	/* Nothing else to do for power collapse or SWFI. */
	if (reason == SETRATE_PC || reason == SETRATE_SWFI)
		goto out;

	dprintk("ACPU%d speed change complete\n", cpu);

out:
	if (reason == SETRATE_CPUFREQ || reason == SETRATE_HOTPLUG)
		mutex_unlock(&drv_state.lock);
	return rc;
}

/* Initialize a HFPLL at a given rate and enable it. */
static void __init hfpll_init(enum scalables id, struct core_speed *tgt_s)
{
	BUG_ON(id >= NUM_SCALABLES);

	dprintk("Initializing HFPLL%d\n", id);

	/* Disable the PLL for re-programming. */
	hfpll_disable(id);

	/* Configure PLL parameters for integer mode. */
	writel_relaxed(0x7845C665, hf_pll_base[id] + HFPLL_CONFIG_CTL);
	writel_relaxed(0, hf_pll_base[id] + HFPLL_M_VAL);
	writel_relaxed(1, hf_pll_base[id] + HFPLL_N_VAL);

	/* Set up droop controller. TODO: Enable droop controller. */
	writel_relaxed(0x0100C000, hf_pll_base[id] + HFPLL_DROOP_CTL);

	/* Set an initial rate and enable the PLL. */
	hfpll_set_rate(id, tgt_s);
	hfpll_enable(id);
}

#define INIT_QSB_ID	0
#define INIT_HFPLL_ID	1
/* Set initial rate for a given core. */
static void __init init_clock_sources(enum scalables id)
{
	struct core_speed *temp_s, *tgt_s;
	uint32_t pri_src, regval;
	static struct core_speed speed[] = {
		[INIT_QSB_ID] =   { STBY_KHZ, QSB,   0, 0, 0x00 },
		[INIT_HFPLL_ID] = { 432000,   HFPLL, 2, 0, 0x20 },
	};

	/*
	 * If the HFPLL is in use, program AUX source for QSB, switch to it,
	 * re-initialize the HFPLL, and switch back to the HFPLL. Otherwise,
	 * the HFPLL is not in use, so we can switch directly to it.
	 */
	tgt_s = &speed[INIT_HFPLL_ID];
	pri_src = get_pri_clk_src(id);
	if (pri_src == PRI_SRC_SEL_HFPLL || pri_src == PRI_SRC_SEL_HFPLL_DIV2) {
		temp_s = &speed[INIT_QSB_ID];
		set_sec_clk_src(id, temp_s->sec_src_sel);
		set_pri_clk_src(id, temp_s->pri_src_sel);
	}
	hfpll_init(id, tgt_s);

	/* Set PRI_SRC_SEL_HFPLL_DIV2 divider to div-2. */
	regval = readl_cp15_l2ind(l2cpmr_iaddr[id]);
	regval &= ~(0x3 << 6);
	writel_cp15_l2ind(regval, l2cpmr_iaddr[id]);

	/* Select PLL8 as AUX source input to the secondary MUX. */
	writel_relaxed(0x3, aux_clk_sel[id]);

	set_pri_clk_src(id, tgt_s->pri_src_sel);
	drv_state.current_speed[id] = tgt_s;
}

/* Perform CPU0-specific setup. */
int __init msm_acpu_clock_early_init(void)
{
	BUG_ON(smp_processor_id() != 0);
	init_clock_sources(L2);
	init_clock_sources(CPU0);

	return 0;
}
early_initcall(msm_acpu_clock_early_init);

/* Perform CPU1-specific setup. */
void __cpuinit acpuclock_secondary_init(void)
{
	static bool warm_boot;

	BUG_ON(smp_processor_id() == 0);
	if (warm_boot)
		return;

	init_clock_sources(CPU1);

	/* Secondary CPU has booted, don't repeat for subsequent warm boots. */
	warm_boot = true;
}

#ifdef CONFIG_CPU_FREQ_MSM
static struct cpufreq_frequency_table freq_table[NR_CPUS][30];

static void __init cpufreq_table_init(void)
{
	int cpu;

	for_each_possible_cpu(cpu) {
		int i, freq_cnt = 0;
		/* Construct the freq_table tables from acpu_freq_tbl. */
		for (i = 0; acpu_freq_tbl[i].speed.khz != 0
				&& freq_cnt < ARRAY_SIZE(*freq_table); i++) {
			if (acpu_freq_tbl[i].use_for_scaling) {
				freq_table[cpu][freq_cnt].index = freq_cnt;
				freq_table[cpu][freq_cnt].frequency
					= acpu_freq_tbl[i].speed.khz;
				freq_cnt++;
			}
		}
		/* freq_table not big enough to store all usable freqs. */
		BUG_ON(acpu_freq_tbl[i].speed.khz != 0);

		freq_table[cpu][freq_cnt].index = freq_cnt;
		freq_table[cpu][freq_cnt].frequency = CPUFREQ_TABLE_END;

		pr_info("CPU%d: %d scaling frequencies supported.\n",
			cpu, freq_cnt);

		/* Register table with CPUFreq. */
		cpufreq_frequency_table_get_attr(freq_table[cpu], cpu);
	}
}
#else
static void __init cpufreq_table_init(void) {}
#endif


#define HOT_UNPLUG_KHZ STBY_KHZ
static int __cpuinit acpuclock_cpu_callback(struct notifier_block *nfb,
					    unsigned long action, void *hcpu)
{
	static int prev_khz[NR_CPUS];
	int cpu = (int)hcpu;

	switch (action) {
	case CPU_DEAD:
		prev_khz[cpu] = acpuclk_get_rate(cpu);
		/* Fall through. */
	case CPU_UP_CANCELED:
		acpuclk_set_rate(cpu, HOT_UNPLUG_KHZ, SETRATE_HOTPLUG);
		break;
	case CPU_UP_PREPARE:
		if (WARN_ON(!prev_khz[cpu]))
			prev_khz[cpu] = acpu_freq_tbl->speed.khz;
		acpuclk_set_rate(cpu, prev_khz[cpu], SETRATE_HOTPLUG);
		break;
	default:
		break;
	}

	return NOTIFY_OK;
}

static struct notifier_block __cpuinitdata acpuclock_cpu_notifier = {
	.notifier_call = acpuclock_cpu_callback,
};

void __init msm_acpu_clock_init(struct msm_acpu_clock_platform_data *clkdata)
{
	mutex_init(&drv_state.lock);
	drv_state.acpu_switch_time_us = clkdata->acpu_switch_time_us;
	drv_state.vdd_switch_time_us = clkdata->vdd_switch_time_us;
	cpufreq_table_init();
	register_hotcpu_notifier(&acpuclock_cpu_notifier);
}
