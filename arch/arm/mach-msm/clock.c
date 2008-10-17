/* arch/arm/mach-msm/clock.c
 *
 * MSM architecture clock driver
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2007 QUALCOMM Incorporated
 * Author: San Mehat <san@android.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 */

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/cpufreq.h>
#include <linux/mutex.h>
#include <linux/debugfs.h>
#include <linux/spinlock.h>
#include <asm/io.h>
#include <asm/arch/msm_iomap.h>
#include "clock.h"
#include <linux/cpufreq.h>

#ifdef CONFIG_ANDROID_POWER
#include <linux/android_power.h>
#endif

#include "proc_comm.h"

#define PERF_SWITCH_DEBUG 0
#define PERF_SWITCH_STEP_DEBUG 0

struct clock_state
{
	struct clkctl_acpu_speed	*current_speed;
	struct mutex			lock;
	uint32_t			acpu_switch_time_us;
	uint32_t			max_speed_delta_khz;
	uint32_t			vdd_switch_time_us;
	unsigned long			power_collapse_khz;
	unsigned long			wait_for_irq_khz;
};

static struct clock_state drv_state = { 0 };
static DEFINE_MUTEX(clocks_mutex);
static LIST_HEAD(clocks);
static spinlock_t clk_set_lock;

static void acpuclk_init(struct clk *clk);
static int acpuclk_enable(struct clk *clk);
static void acpuclk_disable(struct clk *clk);
static unsigned long acpuclk_get_rate(struct clk *clk);
int acpuclk_set_rate(struct clk *clk, unsigned long rate, int for_power_collapse);

extern struct clkctl_acpu_speed acpu_freq_tbl[];

/* Initalize the lpj field in the acpu_freq_tbl. Caller _must_ hold clk_set_lock. */
static void init_lpj(const struct clkctl_acpu_speed *base_clk) {
	int i;
	for (i = 0; acpu_freq_tbl[i].a11clk_khz; i++) {
		acpu_freq_tbl[i].lpj = cpufreq_scale(loops_per_jiffy,
						base_clk->a11clk_khz,
						acpu_freq_tbl[i].a11clk_khz);
	}
}

static int pc_clk_enable(unsigned id)
{
	return msm_proc_comm(PCOM_CLKCTL_RPC_ENABLE, &id, 0);
}

static void pc_clk_disable(unsigned id)
{
	msm_proc_comm(PCOM_CLKCTL_RPC_DISABLE, &id, 0);
}

static int pc_clk_set_rate(unsigned id, unsigned rate)
{
	return msm_proc_comm(PCOM_CLKCTL_RPC_SET_RATE, &id, &rate);
}

static int pc_clk_set_min_rate(unsigned id, unsigned rate)
{
	return msm_proc_comm(PCOM_CLKCTL_RPC_MIN_RATE, &id, &rate);
}

static int pc_clk_set_max_rate(unsigned id, unsigned rate)
{
	return msm_proc_comm(PCOM_CLKCTL_RPC_MAX_RATE, &id, &rate);
}

static int pc_clk_set_flags(unsigned id, unsigned flags)
{
	return msm_proc_comm(PCOM_CLKCTL_RPC_SET_FLAGS, &id, &flags);
}

static unsigned pc_clk_get_rate(unsigned id)
{
	if (msm_proc_comm(PCOM_CLKCTL_RPC_RATE, &id, 0))
		return 0;
	else
		return id;
}

static unsigned pc_clk_is_enabled(unsigned id)
{
	if (msm_proc_comm(PCOM_CLKCTL_RPC_ENABLED, &id, 0))
		return 0;
	else
		return id;
}

static int pc_pll_request(unsigned id, unsigned on)
{
	int res;
	on = !!on;

#if PERF_SWITCH_DEBUG
	if (on)
		printk(KERN_DEBUG "Enabling PLL %d\n", id);
	else
		printk(KERN_DEBUG "Disabling PLL %d\n", id);
#endif

	res = msm_proc_comm(PCOM_CLKCTL_RPC_PLL_REQUEST, &id, &on);
	if (res < 0)
		return res;

#if PERF_SWITCH_DEBUG
	if (on)
		printk(KERN_DEBUG "PLL %d enabled\n", id);
	else
		printk(KERN_DEBUG "PLL %d disabled\n", id);
#endif
	return res;
}

/*----------------------------------------------------------------------------
 * Architecture level clock registration
 *---------------------------------------------------------------------------*/

int clk_register(struct clk *clk)
{
	if (!clk || IS_ERR(clk))
		return -EINVAL;
	mutex_lock(&clocks_mutex);
	spin_lock_init(&clk->lock);
	list_add_tail(&clk->list, &clocks);
	if (clk->id == ACPU_CLK)
		acpuclk_init(clk);
	mutex_unlock(&clocks_mutex);
	return 0;
}

/*----------------------------------------------------------------------------
 * Standard clock functions defined in include/linux/clk.h
 *---------------------------------------------------------------------------*/
struct clk *clk_get(struct device *dev, const char *id)
{
	struct clk *c_clk;

	mutex_lock(&clocks_mutex);
	list_for_each_entry(c_clk, &clocks, list) {
		if (!strcmp(id, c_clk->name) && try_module_get(c_clk->owner)) {
			mutex_unlock(&clocks_mutex);
			return c_clk;
		}
	}
	mutex_unlock(&clocks_mutex);
	return ERR_PTR(-ENOENT);
}
EXPORT_SYMBOL(clk_get);

int clk_enable(struct clk *clk)
{
	unsigned long flags;
	if (clk->id == ACPU_CLK)
		return acpuclk_enable(clk);
	spin_lock_irqsave(&clk->lock, flags);
	clk->count++;
	if (clk->count == 1)
		pc_clk_enable(clk->id);
	spin_unlock_irqrestore(&clk->lock, flags);
	return 0;
}
EXPORT_SYMBOL(clk_enable);

void clk_disable(struct clk *clk)
{
	unsigned long flags;
	if (clk->id == ACPU_CLK)
		return acpuclk_disable(clk);
	spin_lock_irqsave(&clk->lock, flags);
	BUG_ON(clk->count == 0);
	clk->count--;
	if (clk->count == 0)
		pc_clk_disable(clk->id);
	spin_unlock_irqrestore(&clk->lock, flags);
}
EXPORT_SYMBOL(clk_isable);

unsigned long clk_get_rate(struct clk *clk)
{
	if (clk->id == ACPU_CLK)
		return acpuclk_get_rate(clk);
	return pc_clk_get_rate(clk->id);
}
EXPORT_SYMBOL(clk_get_rate);

void clk_put(struct clk *clk)
{
	module_put(clk->owner);
}
EXPORT_SYMBOL(clk_put);

int clk_set_rate(struct clk *clk, unsigned long rate)
{
	int ret;
	if (clk->id == ACPU_CLK)
		return acpuclk_set_rate(clk, rate, 0);
	if (clk->flags & CLKFLAG_USE_MIN_MAX_TO_SET) {
		if ((ret = pc_clk_set_max_rate(clk->id, rate)))
			return ret;
		return pc_clk_set_min_rate(clk->id, rate);
	}
	return pc_clk_set_rate(clk->id, rate);
}
EXPORT_SYMBOL(clk_set_rate);

int clk_set_parent(struct clk *clk, struct clk *parent)
{
	/*
	 * XXX: Waiting for proper clock tree info from QC
	 */
	return -ENOSYS;
}
EXPORT_SYMBOL(clk_set_parent);

struct clk *clk_get_parent(struct clk *clk)
{
	/*
	 * XXX: Waiting for peroper clock tree info from QC
	 */
	return ERR_PTR(-ENOSYS);
}
EXPORT_SYMBOL(clk_get_parent);

int clk_set_flags(struct clk *clk, unsigned long flags)
{
	if (clk == NULL || IS_ERR(clk))
		return -EINVAL;
	if (clk->id == ACPU_CLK)
		return -EINVAL;
	return pc_clk_set_flags(clk->id, flags);
}
EXPORT_SYMBOL(clk_set_flags);

/*----------------------------------------------------------------------------
 * ARM11 'owned' clock control
 *---------------------------------------------------------------------------*/

int acpuclk_power_collapse(void) {
	return acpuclk_set_rate(NULL, drv_state.power_collapse_khz, 1);
}

int acpuclk_wait_for_irq(void) {
	return acpuclk_set_rate(NULL, drv_state.wait_for_irq_khz, 1);
}

static int acpuclk_enable(struct clk *clk)
{
	return 0; /* acpuclk is always enabled */
}

static void acpuclk_disable(struct clk *clk)
{
	/* acpuclk is always enabled */
}

static int acpuclk_set_vdd_level(int vdd)
{
	uint32_t current_vdd;

	current_vdd = readl(A11S_VDD_SVS_PLEVEL_ADDR) & 0x07;

#if PERF_SWITCH_DEBUG
	printk(KERN_DEBUG "clock: Switching VDD from %u -> %d\n",
	       current_vdd, vdd);
#endif
	writel((1 << 7) | (vdd << 3), A11S_VDD_SVS_PLEVEL_ADDR);
	udelay(drv_state.vdd_switch_time_us);
	if ((readl(A11S_VDD_SVS_PLEVEL_ADDR) & 0x7) != vdd) {
#if PERF_SWITCH_DEBUG
		printk(KERN_ERR "clock: VDD set failed\n");
#endif
		return -EIO;
	}

#if PERF_SWITCH_DEBUG
	printk(KERN_DEBUG "clock: VDD switched\n");
#endif
	return 0;
}

/* Set proper dividers for the given clock speed. */
static void acpuclk_set_div(const struct clkctl_acpu_speed *hunt_s) {
	uint32_t reg_clkctl, reg_clksel, clk_div;

	/* AHB_CLK_DIV */
	clk_div = (readl(A11S_CLK_SEL_ADDR) >> 1) & 0x03;
	/*
	 * If the new clock divider is higher than the previous, then
	 * program the divider before switching the clock
	 */
	if (hunt_s->ahbclk_div > clk_div) {
		reg_clksel = readl(A11S_CLK_SEL_ADDR);
		reg_clksel &= ~(0x3 << 1);
		reg_clksel |= (hunt_s->ahbclk_div << 1);
		writel(reg_clksel, A11S_CLK_SEL_ADDR);
	}
	if ((readl(A11S_CLK_SEL_ADDR) & 0x01) == 0) {
		/* SRC0 */

		/* Program clock source */
		reg_clkctl = readl(A11S_CLK_CNTL_ADDR);
		reg_clkctl &= ~(0x07 << 4);
		reg_clkctl |= (hunt_s->a11clk_src_sel << 4);
		writel(reg_clkctl, A11S_CLK_CNTL_ADDR);

		/* Program clock divider */
		reg_clkctl = readl(A11S_CLK_CNTL_ADDR);
		reg_clkctl &= ~0xf;
		reg_clkctl |= hunt_s->a11clk_src_div;
		writel(reg_clkctl, A11S_CLK_CNTL_ADDR);

		/* Program clock source selection */
		reg_clksel = readl(A11S_CLK_SEL_ADDR);
		reg_clksel |= 1; /* CLK_SEL_SRC1NO  == SRC1 */
		writel(reg_clksel, A11S_CLK_SEL_ADDR);
	} else {
		/* SRC1 */

		/* Program clock source */
		reg_clkctl = readl(A11S_CLK_CNTL_ADDR);
		reg_clkctl &= ~(0x07 << 12);
		reg_clkctl |= (hunt_s->a11clk_src_sel << 12);
		writel(reg_clkctl, A11S_CLK_CNTL_ADDR);

		/* Program clock divider */
		reg_clkctl = readl(A11S_CLK_CNTL_ADDR);
		reg_clkctl &= ~(0xf << 8);
		reg_clkctl |= (hunt_s->a11clk_src_div << 8);
		writel(reg_clkctl, A11S_CLK_CNTL_ADDR);

		/* Program clock source selection */
		reg_clksel = readl(A11S_CLK_SEL_ADDR);
		reg_clksel &= ~1; /* CLK_SEL_SRC1NO  == SRC0 */
		writel(reg_clksel, A11S_CLK_SEL_ADDR);
	}

	/*
	 * If the new clock divider is lower than the previous, then
	 * program the divider after switching the clock
	 */
	if (hunt_s->ahbclk_div < clk_div) {
		reg_clksel = readl(A11S_CLK_SEL_ADDR);
		reg_clksel &= ~(0x3 << 1);
		reg_clksel |= (hunt_s->ahbclk_div << 1);
		writel(reg_clksel, A11S_CLK_SEL_ADDR);
	}
}

int acpuclk_set_rate(struct clk *clk, unsigned long rate, int for_power_collapse)
{
	uint32_t reg_clkctl;
	struct clkctl_acpu_speed *cur_s, *tgt_s, *strt_s;
	int rc = 0;
	unsigned long flags;
#if defined(CONFIG_CPU_FREQ)
	struct cpufreq_freqs freqs;
#endif

	strt_s = cur_s = drv_state.current_speed;

	if (rate == (cur_s->a11clk_khz * 1000))
		return 0;

	for (tgt_s = acpu_freq_tbl; tgt_s->a11clk_khz != 0; tgt_s++) {
		if (tgt_s->a11clk_khz == (rate / 1000))
			break;
	}

	if (tgt_s->a11clk_khz == 0)
		return -EINVAL;

	/* Choose the highest speed speed at or below 'rate' with same PLL. */
	if (for_power_collapse && tgt_s->a11clk_khz < cur_s->a11clk_khz) {
		while (tgt_s->pll != ACPU_PLL_TCXO && tgt_s->pll != cur_s->pll)
			tgt_s--;
	}

	/* Save baseline lpj on the first clock change. */
	spin_lock_irqsave(&clk_set_lock, flags);
	if (acpu_freq_tbl[0].lpj == 0)
		init_lpj(cur_s);
	spin_unlock_irqrestore(&clk_set_lock, flags);

	if (!for_power_collapse) {
		mutex_lock(&drv_state.lock);
#if defined(CONFIG_CPU_FREQ)
		freqs.old = cur_s->a11clk_khz;
		freqs.new = tgt_s->a11clk_khz;
		freqs.cpu = smp_processor_id();
		cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);
#endif
		if (strt_s->pll != tgt_s->pll && tgt_s->pll != ACPU_PLL_TCXO) {
			if ((rc = pc_pll_request(tgt_s->pll, 1)) < 0) {
				printk(KERN_ERR "PLL enable failed (%d)\n", rc);
				goto out;
			}
		}
		/* Increase VDD if needed. */
		if (tgt_s->vdd > cur_s->vdd) {
			if ((rc = acpuclk_set_vdd_level(tgt_s->vdd)) < 0) {
				printk(KERN_ERR "Unable to switch ACPU vdd\n");
				goto out;
			}
		}
	}

	/* Set wait states for CPU inbetween frequency changes */
	reg_clkctl = readl(A11S_CLK_CNTL_ADDR);
	reg_clkctl |= (100 << 16); /* set WT_ST_CNT */
	writel(reg_clkctl, A11S_CLK_CNTL_ADDR);

#if PERF_SWITCH_DEBUG
	printk(KERN_INFO "clock: Switching from ACPU rate %u -> %u\n",
	       strt_s->a11clk_khz * 1000, tgt_s->a11clk_khz * 1000);
#endif

	while (cur_s != tgt_s) {
		/*
		 * Always jump to target freq if within 256mhz, regulardless of
		 * PLL. If differnece is greater, use the predefinied
		 * steppings in the table.
		 */
		int d = abs((int)(cur_s->a11clk_khz - tgt_s->a11clk_khz));
		if (d > drv_state.max_speed_delta_khz) {
			/* Step up or down depending on target vs current. */
			int clk_index = tgt_s->a11clk_khz > cur_s->a11clk_khz ?
				cur_s->up : cur_s->down;
			if (clk_index < 0) { /* This should not happen. */
				printk(KERN_ERR "cur:%u target: %u\n",
					cur_s->a11clk_khz, tgt_s->a11clk_khz);
				rc = -EINVAL;
				goto out;
			}
			cur_s = &acpu_freq_tbl[clk_index];
		} else {
			cur_s = tgt_s;
		}
#if PERF_SWITCH_STEP_DEBUG
		printk(KERN_DEBUG "%s: STEP khz = %u, pll = %d\n",
			__FUNCTION__, cur_s->a11clk_khz, cur_s->pll);
#endif
		acpuclk_set_div(cur_s);
		drv_state.current_speed = cur_s;
		/* Re-adjust lpj for the new clock speed. */
		loops_per_jiffy = cur_s->lpj;
		udelay(drv_state.acpu_switch_time_us);
	}


	/* Nothing else to do for power collapse. */
	if (for_power_collapse)
		return 0;

	/* Disable PLL we are not using anymore. */
	if (strt_s->pll != tgt_s->pll && tgt_s->pll != ACPU_PLL_TCXO) {
		if ((rc = pc_pll_request(strt_s->pll, 0)) < 0) {
			printk(KERN_ERR "PLL disable failed (%d)\n", rc);
			goto out;
		}
	}

	/* Drop VDD level if we can. */
	if (tgt_s->vdd < strt_s->vdd) {
		if (acpuclk_set_vdd_level(tgt_s->vdd) < 0)
			printk(KERN_ERR "clock: Unable to drop ACPU vdd\n");
	}

#if PERF_SWITCH_DEBUG
	printk(KERN_DEBUG "%s: ACPU speed change complete\n", __FUNCTION__);
#endif
#if defined(CONFIG_CPU_FREQ)
	cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);
#endif
out:
	if (!for_power_collapse)
		mutex_unlock(&drv_state.lock);
	return rc;
}

static void acpuclk_init(struct clk *clk)
{
	struct clkctl_acpu_speed *speed;
	uint32_t div, sel, current_vdd;

	/*
	 * Determine the rate of ACPU clock
	 */

	if (!(readl(A11S_CLK_SEL_ADDR) & 0x01)) { /* CLK_SEL_SRC1N0 */
		/* CLK_SRC0_SEL */
		sel = (readl(A11S_CLK_CNTL_ADDR) >> 12) & 0x7;
		/* CLK_SRC0_DIV */
		div = (readl(A11S_CLK_CNTL_ADDR) >> 8) & 0x0f;
	} else {
		/* CLK_SRC1_SEL */
		sel = (readl(A11S_CLK_CNTL_ADDR) >> 4) & 0x07;
		/* CLK_SRC1_DIV */
		div = readl(A11S_CLK_CNTL_ADDR) & 0x0f;
	}

	for (speed = acpu_freq_tbl; speed->a11clk_khz != 0; speed++) {
		if (speed->a11clk_src_sel == sel
		 && (speed->a11clk_src_div == div))
			break;
	}
	if (speed->a11clk_khz == 0) {
		printk(KERN_WARNING "Warning - ACPU clock reports invalid speed\n");
		return;
	}

	drv_state.current_speed = speed;

	printk(KERN_INFO "ACPU running at %d KHz\n", speed->a11clk_khz);
	/*
	 * Ensure that the current freq is okay at this VDD. Earlier
	 * versions of the bootloader would not update VDD properly.
	 */
	current_vdd = readl(A11S_VDD_SVS_PLEVEL_ADDR) & 0x07;
	if (speed->vdd != current_vdd)
		printk(KERN_WARNING "WARNING - Bad VDD (%d != %d) for this freq\n",
		       current_vdd, speed->vdd);
}

static unsigned long acpuclk_get_rate(struct clk *clk)
{
	return drv_state.current_speed->a11clk_khz * 1000;
}

#if defined(CONFIG_DEBUG_FS)
struct clk *msm_clock_get_nth(unsigned index);

static int clock_debug_set(void *data, u64 val)
{
	struct clk *clock = data;
	int ret;

	ret = clk_set_rate(clock, val);
	if (ret != 0)
		printk(KERN_ERR "clk_set_rate failed (%d)\n", ret);
	return ret;
}

static int clock_debug_get(void *data, u64 *val)
{
	struct clk *clock = data;
	*val = clk_get_rate(clock);
	return 0;
}

static int clock_enable_set(void *data, u64 val)
{
	struct clk *clock = data;
	int rc = 0;

	if (val) {
		rc = pc_clk_enable(clock->id);
	} else {
		pc_clk_disable(clock->id);
	}
	return rc;
}

static int clock_enable_get(void *data, u64 *val)
{
	struct clk *clock = data;
	*val = pc_clk_is_enabled(clock->id);
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(clock_rate_fops, clock_debug_get, clock_debug_set, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(clock_enable_fops, clock_enable_get, clock_enable_set, "%llu\n");

static int __init clock_debug_init(void)
{
	struct dentry *dent_rate;
	struct dentry *dent_enable;
	struct clk *clock;
	unsigned n = 0;

	dent_rate = debugfs_create_dir("clk_rate", 0);
	if (IS_ERR(dent_rate))
		return PTR_ERR(dent_rate);

	dent_enable = debugfs_create_dir("clk_enable", 0);
	if (IS_ERR(dent_enable))
		return PTR_ERR(dent_enable);

	while ((clock = msm_clock_get_nth(n++)) != 0) {
		debugfs_create_file(clock->name, 0644, dent_rate,
				    clock, &clock_rate_fops);
		debugfs_create_file(clock->name, 0644, dent_enable,
				    clock, &clock_enable_fops);
	}
	return 0;
}

device_initcall(clock_debug_init);
#endif

#if defined(CONFIG_CPU_FREQ)

/*
 * Clock stable speeds for cpufreq driver. The min/max values of the policy are
 * controlled by the android power driver.
 */
#define CPUFREQ_TABLE_MAX 4
static struct cpufreq_frequency_table cpufreq_table[] =  {
	{ 0, 81920 },
	{ 1, 122880 },
	{ 2, 245760 },
	{ 3, 384000 },
	{ CPUFREQ_TABLE_MAX, CPUFREQ_TABLE_END },
};

static int msm_cpufreq_verify(struct cpufreq_policy *policy)
{
	struct clkctl_acpu_speed *tgt_s;

	cpufreq_verify_within_limits(policy, policy->cpuinfo.min_freq,
				     policy->cpuinfo.max_freq);

	for (tgt_s = acpu_freq_tbl; tgt_s->a11clk_khz != 0; tgt_s++) {
		if (tgt_s->a11clk_khz >= policy->min)
			break;
		if (tgt_s->a11clk_khz > policy->max)
			break;
		if (tgt_s[1].a11clk_khz == 0)
			break;
	}
	if (tgt_s->a11clk_khz < policy->min)
		policy->min = tgt_s->a11clk_khz;
	if (tgt_s->a11clk_khz > policy->max)
		policy->max = tgt_s->a11clk_khz;

	cpufreq_verify_within_limits(policy, policy->cpuinfo.min_freq,
				     policy->cpuinfo.max_freq);
	return 0;
}

static int msm_cpufreq_target(struct cpufreq_policy *policy,
				 unsigned int target_freq,
				 unsigned int relation)
{
	int i;
	for (i = 0; cpufreq_table[i].frequency != CPUFREQ_TABLE_END; i++) {
		if (target_freq <= cpufreq_table[i].frequency)
			break;
	}

	switch(relation) {
		/* Lowest value at or above target frequency. */
		case CPUFREQ_RELATION_L:
			break;
		/* Highest value at or below target frequency. */
		case CPUFREQ_RELATION_H:
			if (i > 0 && cpufreq_table[i].frequency > target_freq)
				i--;
			break;
		default:
			return -EINVAL;
	}
#if defined(CONFIG_CPU_FREQ_DEBUG)
	printk("msm_cpufreq_target %d r %d (%d-%d) selected %d\n", target_freq,
		relation, policy->min, policy->max, cpufreq_table[i].frequency);
#endif
	acpuclk_set_rate(NULL, cpufreq_table[i].frequency * 1000, 0);
	return 0;
}

static unsigned int msm_cpufreq_get(unsigned int cpu)
{
	return drv_state.current_speed->a11clk_khz;
}


#ifdef CONFIG_ANDROID_POWER
/*
 * Early suspend and late resume hooks for android power driver.
 * We use early_suspend late_resume instead of the default
 * cpufreq_driver.suspend/resume to change the max/min policy when
 * screen is on/off while if userspace has a wakelock.
 */
static int update_cpufreq_policy(int screen_on) {
	struct cpufreq_policy *policy;
	if (lock_policy_rwsem_write(smp_processor_id()))
		return -EINVAL;

	if ((policy = cpufreq_cpu_get(smp_processor_id())) == NULL)
		goto out;

	if (screen_on) {
		policy->user_policy.min = cpufreq_table[2].frequency; // 245mhz
		policy->user_policy.max = cpufreq_table[3].frequency; // 384mhz
	} else {
		policy->user_policy.min = cpufreq_table[0].frequency; // 82mhz
		policy->user_policy.max = cpufreq_table[2].frequency; // 245mhz
	}
out:
	unlock_policy_rwsem_write(smp_processor_id());
	if (unlikely(policy == NULL))
		return 0;

	cpufreq_cpu_put(policy);
	/* Give up policy lock before calling update or else deadlock. */
	if (cpufreq_update_policy(policy->cpu))
		printk(KERN_ERR "cpufreq_update_policy(): FAILED\n");
	return 0;
}
static void msm_early_suspend(android_early_suspend_t *handler) {
	update_cpufreq_policy(0);
}

static void msm_late_resume(android_early_suspend_t *handler) {
	update_cpufreq_policy(1);
}

static struct android_early_suspend msm_power_suspend = {
	.suspend = msm_early_suspend,
	.resume = msm_late_resume,
};
#endif

static int msm_cpufreq_init(struct cpufreq_policy *policy)
{
	struct clkctl_acpu_speed *tgt_s, *max_s = NULL;
	if (policy->cpu != 0)
		return -EINVAL;
	for (tgt_s = acpu_freq_tbl; tgt_s->a11clk_khz != 0; tgt_s++)
		max_s = tgt_s;
	if (max_s == NULL)
		return -EINVAL;
	policy->cur = msm_cpufreq_get(smp_processor_id());

	/* Set default policy min to 245mhz and max to 384mhz */
	policy->min = cpufreq_table[2].frequency;
	policy->max = cpufreq_table[CPUFREQ_TABLE_MAX - 1].frequency;

	/* Full speed ranges available are 81mhz - 384mhz. */
	policy->cpuinfo.min_freq = cpufreq_table[0].frequency;
	policy->cpuinfo.max_freq = cpufreq_table[CPUFREQ_TABLE_MAX - 1].frequency;
	policy->cpuinfo.transition_latency =
		drv_state.acpu_switch_time_us * NSEC_PER_USEC;

	/* Register the cpufreq table with the policy's cpu. */
	cpufreq_frequency_table_get_attr(cpufreq_table, policy->cpu);

#ifdef CONFIG_ANDROID_POWER
	android_register_early_suspend(&msm_power_suspend);
#endif
	return 0;
}

static struct cpufreq_driver msm_cpufreq_driver = {
	/* lps calculstaions are handled here. */
	.flags		= CPUFREQ_STICKY | CPUFREQ_CONST_LOOPS,
	.init		= msm_cpufreq_init,
	.verify		= msm_cpufreq_verify,
	.target		= msm_cpufreq_target,
	.name		= "msm",
};

static int __init msm_cpufreq_register(void)
{
	return cpufreq_register_driver(&msm_cpufreq_driver);
}

device_initcall(msm_cpufreq_register);

#endif

/*----------------------------------------------------------------------------
 * Clock driver initialization
 *---------------------------------------------------------------------------*/
void __init clock_init(uint32_t acpu_switch_time_us,
			uint32_t max_speed_delta_khz,
			uint32_t vdd_switch_time_us,
			unsigned long power_collapse_khz,
			unsigned long wait_for_irq_khz)
{
	pr_info("clock_init()\n");

	spin_lock_init(&clk_set_lock);
	mutex_init(&drv_state.lock);
	drv_state.acpu_switch_time_us = acpu_switch_time_us;
	drv_state.max_speed_delta_khz = max_speed_delta_khz;
	drv_state.vdd_switch_time_us = vdd_switch_time_us;
	drv_state.power_collapse_khz = power_collapse_khz;
	drv_state.wait_for_irq_khz = wait_for_irq_khz;
}

/* The bootloader and/or AMSS may have left various clocks enabled.
 * Disable any clocks that belong to us (CLKFLAG_AUTO_OFF) but have
 * not been explicitly enabled by a clk_enable() call.
 */
static int __init clock_late_init(void)
{
	unsigned long flags;
	struct clk *clk;
	unsigned count = 0;

	mutex_lock(&clocks_mutex);
	list_for_each_entry(clk, &clocks, list) {
		if (clk->flags & CLKFLAG_AUTO_OFF) {
			spin_lock_irqsave(&clk->lock, flags);
			if (!clk->count) {
				count++;
				pc_clk_disable(clk->id);
			}
			spin_unlock_irqrestore(&clk->lock, flags);
		}
	}
	mutex_unlock(&clocks_mutex);
	pr_info("clock_late_init() disabled %d unused clocks\n", count);
	return 0;
}

late_initcall(clock_late_init);
