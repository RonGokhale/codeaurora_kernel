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
#include <linux/debugfs.h>
#include <asm/io.h>
#include <asm/arch/msm_iomap.h>
#include <asm/arch/rpc_clkctl.h>
#include "clock.h"

#define PERF_SWITCH_DEBUG 1
#define ENABLE_ALL_PLLS 0
#define PERF_SWITCH_STEP_DEBUG 0

struct clock_state
{
	struct clkctl_acpu_speed	*current_speed;
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
static int acpuclk_enable(struct clk *clk);
static void acpuclk_disable(struct clk *clk);
static unsigned long acpuclk_get_rate(struct clk *clk);
int acpuclk_set_rate(struct clk *clk, unsigned long rate, int for_power_collapse);

extern struct clkctl_acpu_speed acpu_freq_tbl[];

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
	if (!drv_state.rpc)
		return -EAGAIN;
	return drv_state.rpc->enable(clk->id);
}

void clk_disable(struct clk *clk)
{
	if (clk == NULL || IS_ERR(clk))
		return;

	if (clk->id == ACPU_CLK)
		return acpuclk_disable(clk);
	if (!drv_state.rpc)
		return;
	drv_state.rpc->disable(clk->id);
}

unsigned long clk_get_rate(struct clk *clk)
{
	if (clk == NULL || IS_ERR(clk))
		return 0;

	if (clk->id == ACPU_CLK)
		return acpuclk_get_rate(clk);
	if (!drv_state.rpc)
		return 0;
	return drv_state.rpc->get_rate(clk->id);
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
		return acpuclk_set_rate(clk, rate, 0);
	if (!drv_state.rpc)
		return -EAGAIN;
	return drv_state.rpc->set_rate(clk->id, rate);
}

int clk_set_parent(struct clk *clk, struct clk *parent)
{
	/*
	 * XXX: Waiting for proper clock tree info from QC
	 */
	return -ENOSYS;
}

struct clk *clk_get_parent(struct clk *clk)
{
	/*
	 * XXX: Waiting for peroper clock tree info from QC
	 */
	return ERR_PTR(-ENOSYS);
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

static int acpuclk_set_vdd_level(int vdd)
{
	uint32_t current_vdd;

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

int acpuclk_set_rate(struct clk *clk, unsigned long rate, int for_power_collapse)
{
	uint32_t reg_clkctl, reg_clksel, clk_div;
	struct clkctl_acpu_speed *cur_s, *tgt_s, *strt_s;
	int ramp_up = 0; /* 1 == up, 0 == down */
	int rc;

	strt_s = cur_s = drv_state.current_speed;

	if (rate == (cur_s->a11clk_khz * 1000))
		return 0;

	for (tgt_s = acpu_freq_tbl; tgt_s->a11clk_khz != 0; tgt_s++) {
		if (tgt_s->a11clk_khz == (rate / 1000))
			break;
	}

	if (tgt_s->a11clk_khz == 0)
		return -EINVAL;

	if (tgt_s->a11clk_khz > strt_s->a11clk_khz)
		ramp_up = 1;
	else
		ramp_up = 0;

	if (!for_power_collapse)
		mutex_lock(&drv_state.lock);

	if (tgt_s->vdd > strt_s->vdd) {
		rc = acpuclk_set_vdd_level(tgt_s->vdd);
		if (rc < 0) {
			printk(KERN_ERR "clock: Unable to switch ACPU vdd\n");
			if (!for_power_collapse)
				mutex_unlock(&drv_state.lock);
			return rc;
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
	if (strt_s->pll != tgt_s->pll && !drv_state.rpc) {
		printk(KERN_ERR "No RPC for PLL request, acpu switch failed");
		if (!for_power_collapse)
			mutex_unlock(&drv_state.lock);
		return -EAGAIN;
	}

	if (!for_power_collapse &&
	    (strt_s->pll != tgt_s->pll) && (tgt_s->pll != ACPU_PLL_TCXO)) {
#if PERF_SWITCH_DEBUG
		printk("%s: Enabling PLL %d\n", __FUNCTION__, tgt_s->pll);
#endif
		rc = drv_state.rpc->pll_request(tgt_s->pll, 1);
		if (rc < 0) {
			printk(KERN_ERR "PLL enable failed (%d)\n", rc);
			mutex_unlock(&drv_state.lock);
			return rc;
		}
#if PERF_SWITCH_DEBUG
		printk(KERN_DEBUG "%s: PLL %d enabled\n", __FUNCTION__, tgt_s->pll);
#endif
	}

	while (cur_s != tgt_s) {
		struct clkctl_acpu_speed *hunt_s = cur_s;
		while (1) {
			int d;

			if (!ramp_up && hunt_s->a11clk_khz == 19200) {
				hunt_s = NULL;
				break;
			}
			if (ramp_up && hunt_s->a11clk_khz == 528000) {
				hunt_s = NULL;
				break;
			}

			if (ramp_up)
				hunt_s++; /* next clk in table */
			else
				hunt_s--; /* prev clk in table */

			/* Only allow clocks which match the target PLL */
			if ((hunt_s->pll != cur_s->pll)
			 && (hunt_s->pll != tgt_s->pll)) {
				continue; /* Skip speed; not at tgt PLL */
			}

			d = abs((int)(cur_s->a11clk_khz - hunt_s->a11clk_khz));
			if (d > drv_state.max_speed_delta_khz) {
				hunt_s = NULL;
				break; /* Unable to find a step */
			}

			/* Valid speed found */
			break;
		}

		if (!hunt_s) {
			printk(KERN_ERR "clock: Invalid ACPU step\n");
			if(!for_power_collapse)
				mutex_unlock(&drv_state.lock);
			return -EINVAL;
		}
#if PERF_SWITCH_STEP_DEBUG
		printk(KERN_DEBUG "%s: STEP khz = %u, pll = %d\n", __FUNCTION__, hunt_s->a11clk_khz, hunt_s->pll);
#endif

		/* AHB_CLK_DIV */
		clk_div = (readl(A11S_CLK_SEL_ADDR) >> 1) & 0x03;

		/*
		 * If the new clock divider is higher than the previous, then
		 * program the divider before switching the clock
		 */
		if (hunt_s->ahbclk_div > clk_div) {
			reg_clksel = readl(A11S_CLK_SEL_ADDR);
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
			reg_clksel |= (hunt_s->ahbclk_div << 1);
			writel(reg_clksel, A11S_CLK_SEL_ADDR);
		}
		cur_s = hunt_s;
		drv_state.current_speed = cur_s;
		udelay(drv_state.acpu_switch_time_us);
	}

	if (!for_power_collapse &&
	    (strt_s->pll != tgt_s->pll) && (strt_s->pll != ACPU_PLL_TCXO)) {
#if PERF_SWITCH_DEBUG
		printk(KERN_DEBUG "%s: Disabling PLL %d\n", __FUNCTION__, strt_s->pll);
#endif
		rc = drv_state.rpc->pll_request(strt_s->pll, 0);
		if (rc < 0) {
			printk(KERN_ERR "PLL disable failed (%d)\n", rc);
			mutex_unlock(&drv_state.lock);
			return rc;
		}
#if PERF_SWITCH_DEBUG
		printk(KERN_DEBUG "%s: PLL %d disabled\n", __FUNCTION__, strt_s->pll);
#endif
	}

#if PERF_SWITCH_DEBUG
	printk(KERN_DEBUG "%s: ACPU speed change complete\n", __FUNCTION__);
#endif
	if (!for_power_collapse)
		mutex_unlock(&drv_state.lock);
	return 0;
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

static void clock_debug_set(void *data, u64 val)
{
	struct clk *clock = data;
	int ret;
	
	ret = clk_set_rate(clock, val);
	if (ret != 0)
		printk(KERN_ERR "clk_set_rate failed (%d)\n", ret);
		
}

static u64 clock_debug_get(void *data)
{
	struct clk *clock = data;
	return clk_get_rate(clock);
}

DEFINE_SIMPLE_ATTRIBUTE(clock_debug_fops, clock_debug_get, clock_debug_set, "%llu\n");

static void __init clock_debug_init(void)
{
	struct dentry *dent;
	struct clk *clock;
	unsigned n = 0;

	dent = debugfs_create_dir("clk", 0);
	if (IS_ERR(dent))
		return;

	while ((clock = msm_clock_get_nth(n++)) != 0) {
		debugfs_create_file(clock->name, 0644, dent, clock, &clock_debug_fops);
	}
}
#else
static void clock_debug_init(void) {}
#endif

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

/*
 * This should go away when we can do remote clock control via some other
 * method than ONCRPC (yuck)
 */
void clock_register_rpc(struct clkctl_rpc_ops *rpc_ops)
{
	drv_state.rpc = rpc_ops;
#if ENABLE_ALL_PLLS
	{
	int i;
	int rc;
	for (i = 0; i <= 3; i++) {
		rc = drv_state.rpc->pll_request(i, 1);
		if (rc < 0)
			printk(KERN_INFO "clock: Unable to enable PLL index %d (%d)\n", i, rc);
		else
			printk(KERN_INFO "clock: PLL index %d enabled OK\n", i);
	}
	}
#endif

	clock_debug_init();
}
EXPORT_SYMBOL(clock_register_rpc);
