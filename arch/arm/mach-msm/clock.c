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
#include <linux/mutex.h>
#include <asm/io.h>
#include <asm/arch/msm_iomap.h>
#include <asm/arch/rpc_clkctl.h>
#include "clock.h"

#define PERF_SWITCH_DEBUG 1
#define USE_RPC_FOR_CLOCKCTL 1
#define ACPU_BADVDD_FIX 0

struct clock_state
{
	clkctl_acpu_speed_t		acpu_speed;
	clkctl_acpu_perf_level_t	acpu_perf_level;
#ifdef CONFIG_SMP
	unsigned long			loops_per_jiffy[CLKCTL_NUM_ACPU_SPEEDS];
#endif
	int				sleeping;
	struct clkctl_rpc_ops		*rpc;
	struct mutex			lock;
	uint32_t			acpu_switch_time_us;
	uint32_t			max_speed_delta_khz;
	uint32_t			vdd_switch_time_us;
};

static struct clock_state drv_state = { 0 };
static DEFINE_MUTEX(clocks_mutex);
static LIST_HEAD(clocks);

static void acpuclk_init(struct clk *clk);
static void arm9clk_init(struct clk *clk);
static int acpuclk_enable(struct clk *clk);
static void acpuclk_disable(struct clk *clk);
static unsigned long acpuclk_get_rate(struct clk *clk);
static int acpuclk_set_rate(struct clk *clk, unsigned long rate);

/* XXX: refactor extern reference */
extern clkctl_acpu_perf_cfg_t acpu_perf_cfg[];
extern clkctl_acpu_clk_cfg_t  acpu_clk_cfg[];

/*----------------------------------------------------------------------------
 * Architecture level clock registration
 *---------------------------------------------------------------------------*/

int clk_register(struct clk *clk)
{
	if (!clk || IS_ERR(clk))
		return -EINVAL;
	mutex_lock(&clocks_mutex);
	list_add_tail(&clk->list, &clocks);
	if (clk->id == ACPU_CLK)
		acpuclk_init(clk);
	else if (clk->a9_controlled)
		arm9clk_init(clk);
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

int clk_enable(struct clk *clk)
{
	if (clk == NULL || IS_ERR(clk))
		return -EINVAL;

	if (clk->id == ACPU_CLK)
		return acpuclk_enable(clk);
	else if (clk->a9_controlled) {
		if (!drv_state.rpc)
			return -EAGAIN;
		return drv_state.rpc->enable(clk->id);
	} else {
		printk(KERN_ERR "clock: Can't enable clock '%s' (not a9 controlled)\n",
		       clk->name);
		return -ENOSYS;
	}
}

void clk_disable(struct clk *clk)
{
	if (clk == NULL || IS_ERR(clk))
		return;

	if (clk->id == ACPU_CLK)
		return acpuclk_disable(clk);
	else if (clk->a9_controlled) {
		if (!drv_state.rpc)
			return;
		drv_state.rpc->disable(clk->id);
	} else {
		printk(KERN_ERR "clock: Can't disable clock '%s' (not a9 controlled)\n",
		       clk->name);
		return;
	}
}

unsigned long clk_get_rate(struct clk *clk)
{
	if (clk == NULL || IS_ERR(clk))
		return 0;

	if (clk->id == ACPU_CLK)
		return acpuclk_get_rate(clk);
	else if (clk->a9_controlled) {
		if (!drv_state.rpc)
			return 0;
		return drv_state.rpc->get_rate(clk->id);
	} else {
		printk(KERN_ERR "clock: Can't getrate clock '%s' (not a9 controlled)\n",
		       clk->name);
		return 0;
	}
}

void clk_put(struct clk *clk)
{
	if (clk && !IS_ERR(clk))
		module_put(clk->owner);
}

int clk_set_rate(struct clk *clk, unsigned long rate)
{
	if (clk == NULL || IS_ERR(clk))
		return -EINVAL;

	if (clk->id == ACPU_CLK)
		return acpuclk_set_rate(clk, rate);
	else if (clk->a9_controlled) {
		if (!drv_state.rpc)
			return -EAGAIN;
		return drv_state.rpc->set_rate(clk->id, rate);
	} else {
		printk(KERN_ERR "clock: Can't setrate clock '%s' (not a9 controlled)\n",
		       clk->name);
		return -ENOSYS;
	}
}

int clk_set_parent(struct clk *clk, struct clk *parent)
{
	/*
	 * XXX: Waiting for proper clock tree info from QC
	 */
	printk(KERN_ERR "clock: clk_set_parent not yet implemented\n");
	return -ENOSYS;
}

struct clk *clk_get_parent(struct clk *clk)
{
	/*
	 * XXX: Waiting for peroper clock tree info from QC
	 */
	printk(KERN_ERR "clock: clk_get_parent not yet implemented\n");
	return ERR_PTR(-ENOSYS);
}


/*----------------------------------------------------------------------------
 * ARM9 'owned' clock control
 *---------------------------------------------------------------------------*/

static void arm9clk_init(struct clk *clk)
{

	/*
	 * At the moment, ARM9 controlled clocks are manipulated via
	 * ONCRPC (ugh), so its too early at this point to muck with the
	 * clocks (we need to wait for the rpc_clkctl driver to register
	 * its RPC operations with us), so don't do anything here.
	 *
	 * Once QC has updated the clock control interface to run via
	 * a lower level interface (at our request), we can do peripheral
	 * clock stuff here.
	 */
}

/*----------------------------------------------------------------------------
 * ARM11 'owned' clock control
 *---------------------------------------------------------------------------*/

static int acpuclk_enable(struct clk *clk)
{
	return 0; /* acpuclk is always enabled */
}

static void acpuclk_disable(struct clk *clk)
{
	/* acpuclk is always enabled */
}

static int acpuclk_set_vdd_level(clkctl_acpu_vdd_t vdd)
{
	uint32_t current_vdd;

	if (vdd >= CLKCTL_NUM_ACPU_VDD_LEVELS)
		return -EINVAL;

	current_vdd = readl(A11S_VDD_SVS_PLEVEL_ADDR) & 0x07;

#if PERF_SWITCH_DEBUG
	printk(KERN_INFO "clock: Switching VDD from %u -> %d\n",
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
	printk(KERN_INFO "clock: VDD switched\n");
#endif
	return 0;
}

static int acpuclk_switch_clk(clkctl_acpu_speed_t new_clk)
{
	uint32_t reg_clkctl, reg_clksel;
	uint32_t clk_div;
	int ramp_up_or_down;
	int start_clk, end_clk, next_clk;
	int rc, i, delta;
	clkctl_source_t target_pll;
	clkctl_source_t current_pll;
	clkctl_source_t next_pll;
	uint32_t cur_speed = drv_state.acpu_speed;

	if (new_clk == cur_speed)
		return 0;

	mutex_lock(&drv_state.lock);

	/* Set wait states for CPU inbetween frequency changes */
	reg_clkctl = readl(A11S_CLK_CNTL_ADDR);
	reg_clkctl |= (100 << 16); /* set WT_ST_CNT */
	writel(reg_clkctl, A11S_CLK_CNTL_ADDR);

	if (new_clk > cur_speed)
		ramp_up_or_down = 1; /* up */
	else
		ramp_up_or_down = -1; /* down */

	start_clk = cur_speed;
	current_pll = acpu_clk_cfg[start_clk].source;
	end_clk = new_clk;
	target_pll = acpu_clk_cfg[end_clk].source;

#if PERF_SWITCH_DEBUG
	printk(KERN_INFO "clock: Switching from clock %d -> %d\n", start_clk, end_clk);
	printk(KERN_INFO "clock: Current PLL %d, Target PLL %d\n", current_pll, target_pll);
#endif
	if ((current_pll != target_pll) && !drv_state.rpc) {
		printk(KERN_ERR "No RPC for PLL request, acpu switch failed");
		mutex_unlock(&drv_state.lock);
		return -EIO;
	}

	if ((current_pll != target_pll) && (target_pll != CLKCTL_TCXO)
	 && !drv_state.sleeping) {
		rc = drv_state.rpc->pll_request(target_pll - CLKCTL_PLL0, 1);
		if (rc < 0)
			printk(KERN_ERR "PLL enable failed (%d)\n", rc);
	}

	next_clk = start_clk;
	while (next_clk != end_clk) {
		i = next_clk;
		do {
			i += ramp_up_or_down;

			next_pll = acpu_clk_cfg[i].source;
			if (next_pll != current_pll && next_pll != target_pll)
				continue;

			delta = abs((int) (acpu_clk_cfg[cur_speed].a11s_clk_khz
				- acpu_clk_cfg[i].a11s_clk_khz));
			if (delta > drv_state.max_speed_delta_khz)
				break;
			next_clk = i;
		} while (i != end_clk);

		/* Validate step */
		if (next_clk == cur_speed) {
			printk(KERN_ERR "clock: Error - Valid step not found\n");
			mutex_unlock(&drv_state.lock);
			return -EINVAL;
		}

		/* AHB_CLK_DIV */
		clk_div = (readl(A11S_CLK_SEL_ADDR) >> 1) & 0x03;

		/*
		 * If the new clock divider is higher than the previous, then
		 * program the divider before switching the clock
		 */
		if (acpu_clk_cfg[next_clk].ahb_clk_div > clk_div) {
			reg_clksel = readl(A11S_CLK_SEL_ADDR);
			reg_clksel |= (acpu_clk_cfg[next_clk].ahb_clk_div << 1);
			writel(reg_clksel, A11S_CLK_SEL_ADDR);
		}

		if ((readl(A11S_CLK_SEL_ADDR) & 0x01) == 0) {
			/* SRC0 */

			/* Program clock source */
			reg_clkctl = readl(A11S_CLK_CNTL_ADDR);
			reg_clkctl &= ~(0x07 << 4);
			reg_clkctl |= (acpu_clk_cfg[next_clk].a11s_clk_src_sel << 4);
			writel(reg_clkctl, A11S_CLK_CNTL_ADDR);

			/* Program clock divider */
			reg_clkctl = readl(A11S_CLK_CNTL_ADDR);
			reg_clkctl &= ~0xf;
			reg_clkctl |= acpu_clk_cfg[next_clk].a11s_clk_src_div;
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
			reg_clkctl |= (acpu_clk_cfg[next_clk].a11s_clk_src_sel << 12);
			writel(reg_clkctl, A11S_CLK_CNTL_ADDR);

			/* Program clock divider */
			reg_clkctl = readl(A11S_CLK_CNTL_ADDR);
			reg_clkctl &= ~(0xf << 8);
			reg_clkctl |= (acpu_clk_cfg[next_clk].a11s_clk_src_div << 8);
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
		if (acpu_clk_cfg[next_clk].ahb_clk_div < clk_div) {
			reg_clksel = readl(A11S_CLK_SEL_ADDR);
			reg_clksel |= (acpu_clk_cfg[next_clk].ahb_clk_div << 1);
			writel(reg_clksel, A11S_CLK_SEL_ADDR);
		}

		drv_state.acpu_speed = (clkctl_acpu_speed_t) next_clk;

#ifdef CONFIG_SMP
		loops_per_jiffy = drv_state.loops_per_jiffy[drv_state.acpu_speed];
#endif

		udelay(drv_state.acpu_switch_time_us);
	}

	if ((current_pll != target_pll) && (current_pll != CLKCTL_TCXO)
	 && !drv_state.sleeping) {
		rc = drv_state.rpc->pll_request(current_pll - CLKCTL_PLL0, 0);
		if (rc < 0)
			printk(KERN_ERR "PLL disable failed (%d)\n", rc);
	}

	mutex_unlock(&drv_state.lock);
	return 0;
}

static int acpuclk_set_performance_level(int perf)
{
	int rc;

	if (perf >= CLKCTL_NUM_ACPU_PERF_LEVELS)
		return -EINVAL;

#if PERF_SWITCH_DEBUG
	printk(KERN_INFO "clock: Switching to performance level %d\n", perf);
#endif

	if (perf == drv_state.acpu_perf_level)
		return 0;
	else if (perf >= drv_state.acpu_perf_level) {
		rc = acpuclk_set_vdd_level(acpu_perf_cfg[perf].vdd);
		if (rc < 0)
			return rc;
		rc = acpuclk_switch_clk(acpu_perf_cfg[perf].speed);
		if (rc < 0) {
			acpuclk_set_vdd_level(
			       acpu_perf_cfg[drv_state.acpu_perf_level].vdd);
			return rc;
		}
	} else {
		rc = acpuclk_switch_clk(acpu_perf_cfg[perf].speed);
		if (rc < 0)
			return rc;
		rc = acpuclk_set_vdd_level(acpu_perf_cfg[perf].vdd);
		if (rc < 0) {
			acpuclk_switch_clk(
			       acpu_perf_cfg[drv_state.acpu_perf_level].speed);
			return rc;
		}
	}
	return 0;
}

static void acpuclk_init(struct clk *clk)
{
	uint32_t acpu_speed, div, sel;
	uint32_t perf, current_vdd;

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

	for (acpu_speed = 0; acpu_speed < CLKCTL_NUM_ACPU_SPEEDS; acpu_speed++) {
		if (acpu_clk_cfg[acpu_speed].a11s_clk_src_sel == sel
		 && acpu_clk_cfg[acpu_speed].a11s_clk_src_div == div)
			break;
	}

	if (acpu_speed == CLKCTL_NUM_ACPU_SPEEDS) {
		printk(KERN_WARNING "Warning - ACPU clock reports invalid speed\n");
		return;
	}

	drv_state.acpu_speed = acpu_speed;

	for (perf = 0; perf < CLKCTL_NUM_ACPU_PERF_LEVELS; perf ++) {
		if (drv_state.acpu_speed == acpu_perf_cfg[perf].speed)
			break;
	}
	if (perf == CLKCTL_NUM_ACPU_PERF_LEVELS) {
		printk(KERN_WARNING "Warning - ACPU clock reports invalid perf level\n");
		return;
	}
	drv_state.acpu_perf_level = perf;


	printk(KERN_INFO "ACPU running at %d KHz (performance level %d)\n",
	       acpu_clk_cfg[acpu_speed].a11s_clk_khz, perf);
	/*
	 * Ensure that the current freq is okay at this VDD. Earlier
	 * versions of the bootloader would not update VDD properly.
	 */
	current_vdd = readl(A11S_VDD_SVS_PLEVEL_ADDR) & 0x07;
	if (acpu_perf_cfg[perf].vdd != current_vdd) {
		printk(KERN_WARNING "WARNING - Bad VDD (%d != %d) for this freq.. fixing\n",
		       current_vdd, acpu_perf_cfg[perf].vdd);
#if ACPU_BADVDD_FIX
		if (acpuclk_set_vdd_level(acpu_perf_cfg[perf].vdd) < 0)
			printk(KERN_ERR "Unable to fix VDD, system may be unstable!\n");
#endif
	}

#ifdef CONFIG_SMP
	for (i = 0; i < CLKCTL_NUM_ACPU_SPEEDS; i++)
		drv_state.loops_per_jiffy[i] = (loops_per_jiffy *
			 (clkctl_acpu_clk_cfg[i].a11s_clk_khz / 1024)) /
			 (clkctl_acpu_clk_cfg[acpu_speed].a11s_clk_khz / 1024);
#endif
}

static int acpuclk_set_rate(struct clk *clk, unsigned long rate)
{
	int perf;

	for (perf = 0; perf < CLKCTL_NUM_ACPU_PERF_LEVELS; perf++) {
		if (acpu_perf_cfg[perf].speed == rate)
			break;
	}

	return acpuclk_set_performance_level(perf);
}

static unsigned long acpuclk_get_rate(struct clk *clk)
{
	return acpu_clk_cfg[drv_state.acpu_speed].a11s_clk_khz * 1000;
}


/*----------------------------------------------------------------------------
 * Clock driver initialization
 *---------------------------------------------------------------------------*/
void __init clock_init(uint32_t acpu_switch_time_us,
		       uint32_t max_speed_delta_khz,
		       uint32_t vdd_switch_time_us)
{
	printk(KERN_INFO "clock_init():\n");

	mutex_init(&drv_state.lock);
	drv_state.acpu_switch_time_us = acpu_switch_time_us;
	drv_state.max_speed_delta_khz = max_speed_delta_khz;
	drv_state.vdd_switch_time_us = vdd_switch_time_us;
}

#if USE_RPC_FOR_CLOCKCTL
/*
 * This should go away when we can do remote clock control via some other
 * method than ONCRPC (yuck)
 */
void clock_register_rpc(struct clkctl_rpc_ops *rpc_ops)
{
	drv_state.rpc = rpc_ops;
}
#endif

#if USE_RPC_FOR_CLOCKCTL
EXPORT_SYMBOL(clock_register_rpc);
#endif
