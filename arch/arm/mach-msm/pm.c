/* arch/arm/mach-msm/pm.c
 *
 * MSM Power Management Routines
 *
 * Copyright (C) 2007 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/init.h>
#include <linux/pm.h>
#include <asm/arch/msm_iomap.h>
#include <asm/arch/system.h>
#include <asm/io.h>

#include "smd_private.h"
#include "clock.h"

enum {
	MSM_PM_SLEEP_MODE_POWER_COLLAPSE_SUSPEND,
	MSM_PM_SLEEP_MODE_POWER_COLLAPSE,
	MSM_PM_SLEEP_MODE_APPS_SLEEP,
	MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT,
	MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT,
};
static int msm_pm_sleep_mode = CONFIG_MSM7X00A_SLEEP_MODE;
module_param_named(sleep_mode, msm_pm_sleep_mode, int, S_IRUGO | S_IWUSR | S_IWGRP);

#define A11S_CLK_SLEEP_EN (MSM_CSR_BASE + 0x11c)
#define A11S_PWRDOWN (MSM_CSR_BASE + 0x440)
#define A11S_STANDBY_CTL (MSM_CSR_BASE + 0x108)
#define A11RAMBACKBIAS (MSM_CSR_BASE + 0x508)

static struct clk *acpu_clk;
unsigned long pm_saved_acpu_clk_rate;

int acpuclk_set_rate(struct clk *clk, unsigned long rate, int for_power_collapse);

int msm_pm_collapse(void);
void msm_pm_collapse_exit(void);

static uint32_t *msm_pm_reset_vector;

#define TARGET_CLOCK_RATE 19200000

static int
msm_pm_wait_state(uint32_t wait_state_all_set, uint32_t wait_state_all_clear,
                  uint32_t wait_state_any_set, uint32_t wait_state_any_clear)
{
	int i;
	uint32_t state;

	for (i = 0; i < 100000; i++) {
		state = smsm_get_state();
		if (((state & wait_state_all_set) == wait_state_all_set) &&
		    ((~state & wait_state_all_clear) == wait_state_all_clear) &&
		    (wait_state_any_set == 0 || (state & wait_state_any_set) ||
		     wait_state_any_clear == 0 || (state & wait_state_any_clear)))
			return 0;
	}
	printk(KERN_ERR "msm_pm_wait_state(%x, %x, %x, %x) failed %x\n",
	       wait_state_all_set, wait_state_all_clear,
	       wait_state_any_set, wait_state_any_clear, state);
	return -ETIMEDOUT;
}

static int msm_pm_enter(suspend_state_t state)
{
	uint32_t saved_vector[2];
	int collapsed;
	struct smsm_interrupt_info int_info;
	void msm_irq_enter_sleep(bool arm9_wake);
	void msm_irq_exit_sleep(void);
	void msm_gpio_enter_sleep(void);
	void msm_gpio_exit_sleep(void);
	int sleep_mode = msm_pm_sleep_mode;
	uint32_t enter_state;
	uint32_t exit_state;
	uint32_t exit_wait_clear = 0;
	uint32_t exit_wait_set = 0;
	int ret;

	printk(KERN_INFO "msm_pm_enter(): mode %d\n", sleep_mode);

	switch (sleep_mode) {
	case MSM_PM_SLEEP_MODE_POWER_COLLAPSE:
		enter_state = SMSM_PWRC;
		exit_state = SMSM_WFPI;
		exit_wait_clear = SMSM_RSA;
		break;
	case MSM_PM_SLEEP_MODE_POWER_COLLAPSE_SUSPEND:
		enter_state = SMSM_PWRC_SUSPEND;
		exit_state = SMSM_WFPI;
		exit_wait_clear = SMSM_RSA;
		break;
	case MSM_PM_SLEEP_MODE_APPS_SLEEP:
		enter_state = SMSM_SLEEP;
		exit_state = SMSM_SLEEPEXIT;
		exit_wait_set = SMSM_SLEEPEXIT;
		break;
	default:
		enter_state = 0;
		exit_state = 0;
	}

	if (enter_state) {
		int_info.aArm_en_mask = 0;
		int_info.aArm_interrupts_pending = 0;
		smsm_set_interrupt_info(&int_info);
	}

	msm_gpio_enter_sleep();

	if (enter_state) {
		if (sleep_mode < MSM_PM_SLEEP_MODE_APPS_SLEEP)
			smsm_set_sleep_duration(0);
		else
			smsm_set_sleep_duration(192000*5); /* APPS_SLEEP does not allow infinite timeout */
		ret = smsm_change_state(SMSM_RUN, enter_state);
		if (ret) {
			printk(KERN_INFO "msm_pm_enter(): smsm_change_state %x failed\n", enter_state);
			enter_state = 0;
			exit_state = 0;
		}
	}
	msm_irq_enter_sleep(!!enter_state);

	if (enter_state) {
		writel(0x1f, A11S_CLK_SLEEP_EN);
		writel(1, A11S_PWRDOWN);

		writel(0, A11S_STANDBY_CTL);
		writel(0, A11RAMBACKBIAS);

		printk(KERN_INFO "msm_pm_enter(): enter A11S_CLK_SLEEP_EN %x, A11S_PWRDOWN %x, smsm_get_state %x\n",
		       readl(A11S_CLK_SLEEP_EN), readl(A11S_PWRDOWN), smsm_get_state());
	}

	if (sleep_mode <= MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT) {
		pm_saved_acpu_clk_rate = clk_get_rate(acpu_clk);
		printk(KERN_INFO "msm_pm_enter(): change clk %ld -> %d\n", pm_saved_acpu_clk_rate, TARGET_CLOCK_RATE);
		acpuclk_set_rate(acpu_clk, TARGET_CLOCK_RATE, 1);
	}
	if (sleep_mode < MSM_PM_SLEEP_MODE_APPS_SLEEP) {
		saved_vector[0] = msm_pm_reset_vector[0];
		saved_vector[1] = msm_pm_reset_vector[1];
		msm_pm_reset_vector[0] = 0xE51FF004; /* ldr pc, 4 */
		msm_pm_reset_vector[1] = virt_to_phys(msm_pm_collapse_exit);
		printk(KERN_INFO "msm_pm_enter: vector %x %x -> %x %x\n",
		       saved_vector[0], saved_vector[1], msm_pm_reset_vector[0], msm_pm_reset_vector[1]);
		collapsed = msm_pm_collapse();
		msm_pm_reset_vector[0] = saved_vector[0];
		msm_pm_reset_vector[1] = saved_vector[1];
		if (collapsed)
			cpu_init();
		printk(KERN_INFO "msm_pm_collapse(): returned %d\n", collapsed);
	} else
		arch_idle();

	if (sleep_mode <= MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT) {
		printk(KERN_INFO "msm_pm_enter(): change clk %d -> %ld\n", TARGET_CLOCK_RATE, pm_saved_acpu_clk_rate);
		if (acpuclk_set_rate(acpu_clk, pm_saved_acpu_clk_rate, 1) < 0)
			printk(KERN_INFO "msm_pm_enter(): clk_set_rate %ld failed\n", pm_saved_acpu_clk_rate);
	}
	printk(KERN_INFO "msm_pm_enter(): exit A11S_CLK_SLEEP_EN %x, A11S_PWRDOWN %x, smsm_get_state %x\n",
	       readl(A11S_CLK_SLEEP_EN), readl(A11S_PWRDOWN), smsm_get_state());
	if (enter_state) {
		writel(0, A11S_PWRDOWN);
		smsm_change_state(enter_state, exit_state);
		msm_pm_wait_state(exit_wait_set, exit_wait_clear, 0, 0);
		printk(KERN_INFO "msm_pm_enter(): sleep exit A11S_CLK_SLEEP_EN %x, A11S_PWRDOWN %x, smsm_get_state %x\n",
		       readl(A11S_CLK_SLEEP_EN), readl(A11S_PWRDOWN), smsm_get_state());
		smsm_change_state(exit_state, SMSM_RUN);
		msm_pm_wait_state(SMSM_RUN, 0, 0, 0);
		printk(KERN_INFO "msm_pm_enter(): sleep exit A11S_CLK_SLEEP_EN %x, A11S_PWRDOWN %x, smsm_get_state %x\n",
		       readl(A11S_CLK_SLEEP_EN), readl(A11S_PWRDOWN), smsm_get_state());
	}
	msm_irq_exit_sleep();
	msm_gpio_exit_sleep();
	return 0;
}

static struct pm_ops msm_pm_ops = {
	.enter		= msm_pm_enter,
	.valid		= pm_valid_only_mem,
};

static int __init msm_pm_init(void)
{
	acpu_clk = clk_get(NULL, "acpu_clk");
	if (acpu_clk == NULL) {
		printk(KERN_ERR "msm_pm_init: failed get acpu_clk\n");
		return -ENODEV;
	}

	msm_pm_reset_vector = ioremap(0, PAGE_SIZE);
	if (msm_pm_reset_vector == NULL) {
		printk(KERN_ERR "msm_pm_init: failed to map reset vector\n");
		return -ENODEV;
	}

	pm_set_ops(&msm_pm_ops);
	return 0;
}

__initcall(msm_pm_init);
