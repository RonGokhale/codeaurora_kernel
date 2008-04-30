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
#include <linux/suspend.h>
#include <linux/reboot.h>
#include <asm/arch/msm_iomap.h>
#include <asm/arch/system.h>
#include <asm/io.h>

#include "smd_private.h"
#include "clock.h"
#include "proc_comm.h"

enum {
	MSM_PM_DEBUG_SUSPEND = 1U << 0,
	MSM_PM_DEBUG_POWER_COLLAPSE = 1U << 1,
	MSM_PM_DEBUG_STATE = 1U << 2,
	MSM_PM_DEBUG_CLOCK = 1U << 3,
	MSM_PM_DEBUG_RESET_VECTOR = 1U << 4,
	MSM_PM_DEBUG_SMSM_STATE = 1U << 5,
};
static int msm_pm_debug_mask = MSM_PM_DEBUG_SUSPEND | MSM_PM_DEBUG_POWER_COLLAPSE;
module_param_named(debug_mask, msm_pm_debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP);

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

static int msm_sleep(int sleep_mode, uint32_t sleep_delay, int from_idle)
{
	uint32_t saved_vector[2];
	int collapsed;
	void msm_irq_enter_sleep1(bool arm9_wake, int from_idle);
	void msm_irq_enter_sleep2(bool arm9_wake, int from_idle);
	void msm_irq_exit_sleep1(void);
	void msm_irq_exit_sleep2(void);
	void msm_irq_exit_sleep3(void);
	void msm_gpio_enter_sleep(void);
	void msm_gpio_exit_sleep(void);
	uint32_t enter_state;
	uint32_t exit_state;
	uint32_t exit_wait_clear = 0;
	uint32_t exit_wait_set = 0;
	int ret;

	if (msm_pm_debug_mask & MSM_PM_DEBUG_SUSPEND)
		printk(KERN_INFO "msm_pm_enter(): mode %d delay %u idle %d\n",
		       sleep_mode, sleep_delay, from_idle);

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

	msm_irq_enter_sleep1(!!enter_state, from_idle);
	msm_gpio_enter_sleep();

	if (enter_state) {
		if (sleep_delay == 0 && sleep_mode >= MSM_PM_SLEEP_MODE_APPS_SLEEP)
			sleep_delay = 192000*5; /* APPS_SLEEP does not allow infinite timeout */
		smsm_set_sleep_duration(sleep_delay);
		ret = smsm_change_state(SMSM_RUN, enter_state);
		if (ret) {
			printk(KERN_ERR "msm_pm_enter(): smsm_change_state %x failed\n", enter_state);
			enter_state = 0;
			exit_state = 0;
		}
	}
	msm_irq_enter_sleep2(!!enter_state, from_idle);

	if (enter_state) {
		writel(0x1f, A11S_CLK_SLEEP_EN);
		writel(1, A11S_PWRDOWN);

		writel(0, A11S_STANDBY_CTL);
		writel(0, A11RAMBACKBIAS);

		if (msm_pm_debug_mask & MSM_PM_DEBUG_STATE)
			printk(KERN_INFO "msm_pm_enter(): enter "
			       "A11S_CLK_SLEEP_EN %x, A11S_PWRDOWN %x, "
			       "smsm_get_state %x\n", readl(A11S_CLK_SLEEP_EN),
			       readl(A11S_PWRDOWN), smsm_get_state());
	}

	if (sleep_mode <= MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT) {
		pm_saved_acpu_clk_rate = clk_get_rate(acpu_clk);
		if (msm_pm_debug_mask & MSM_PM_DEBUG_CLOCK)
			printk(KERN_INFO "msm_pm_enter(): change clk %ld -> %d"
			       "\n", pm_saved_acpu_clk_rate, TARGET_CLOCK_RATE);
		acpuclk_set_rate(acpu_clk, TARGET_CLOCK_RATE, 1);
	}
	if (sleep_mode < MSM_PM_SLEEP_MODE_APPS_SLEEP) {
		if (msm_pm_debug_mask & MSM_PM_DEBUG_SMSM_STATE)
			smsm_print_sleep_info();
		saved_vector[0] = msm_pm_reset_vector[0];
		saved_vector[1] = msm_pm_reset_vector[1];
		msm_pm_reset_vector[0] = 0xE51FF004; /* ldr pc, 4 */
		msm_pm_reset_vector[1] = virt_to_phys(msm_pm_collapse_exit);
		if (msm_pm_debug_mask & MSM_PM_DEBUG_RESET_VECTOR)
			printk(KERN_INFO "msm_pm_enter: vector %x %x -> "
			       "%x %x\n", saved_vector[0], saved_vector[1],
			       msm_pm_reset_vector[0], msm_pm_reset_vector[1]);
		collapsed = msm_pm_collapse();
		msm_pm_reset_vector[0] = saved_vector[0];
		msm_pm_reset_vector[1] = saved_vector[1];
		if (collapsed) {
			cpu_init();
			local_fiq_enable();
		}
		if (msm_pm_debug_mask & MSM_PM_DEBUG_POWER_COLLAPSE)
			printk(KERN_INFO "msm_pm_collapse(): returned %d\n",
			       collapsed);
		if (msm_pm_debug_mask & MSM_PM_DEBUG_SMSM_STATE)
			smsm_print_sleep_info();
	} else
		arch_idle();

	if (sleep_mode <= MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT) {
		if (msm_pm_debug_mask & MSM_PM_DEBUG_CLOCK)
			printk(KERN_INFO "msm_pm_enter(): change clk %d -> %ld"
			       "\n", TARGET_CLOCK_RATE, pm_saved_acpu_clk_rate);
		if (acpuclk_set_rate(acpu_clk, pm_saved_acpu_clk_rate, 1) < 0)
			printk(KERN_ERR "msm_pm_enter(): clk_set_rate %ld "
			       "failed\n", pm_saved_acpu_clk_rate);
	}
	if (msm_pm_debug_mask & MSM_PM_DEBUG_STATE)
		printk(KERN_INFO "msm_pm_enter(): exit A11S_CLK_SLEEP_EN %x, "
		       "A11S_PWRDOWN %x, smsm_get_state %x\n",
		       readl(A11S_CLK_SLEEP_EN), readl(A11S_PWRDOWN),
		       smsm_get_state());
	msm_irq_exit_sleep1();
	if (enter_state) {
		writel(0x00, A11S_CLK_SLEEP_EN);
		writel(0, A11S_PWRDOWN);
		smsm_change_state(enter_state, exit_state);
		msm_pm_wait_state(exit_wait_set, exit_wait_clear, 0, 0);
		if (msm_pm_debug_mask & MSM_PM_DEBUG_STATE)
			printk(KERN_INFO "msm_pm_enter(): sleep exit "
			       "A11S_CLK_SLEEP_EN %x, A11S_PWRDOWN %x, "
			       "smsm_get_state %x\n", readl(A11S_CLK_SLEEP_EN),
			       readl(A11S_PWRDOWN), smsm_get_state());
		if (msm_pm_debug_mask & MSM_PM_DEBUG_SMSM_STATE)
			smsm_print_sleep_info();
	}
	msm_irq_exit_sleep2();
	if (enter_state) {
		smsm_change_state(exit_state, SMSM_RUN);
		msm_pm_wait_state(SMSM_RUN, 0, 0, 0);
		if (msm_pm_debug_mask & MSM_PM_DEBUG_STATE)
			printk(KERN_INFO "msm_pm_enter(): sleep exit "
			       "A11S_CLK_SLEEP_EN %x, A11S_PWRDOWN %x, "
			       "smsm_get_state %x\n", readl(A11S_CLK_SLEEP_EN),
			       readl(A11S_PWRDOWN), smsm_get_state());
	}
	msm_irq_exit_sleep3();
	msm_gpio_exit_sleep();
	return 0;
}

static int msm_pm_enter(suspend_state_t state)
{
	msm_sleep(msm_pm_sleep_mode, 0, 0);
	return 0;
}

static struct platform_suspend_ops msm_pm_ops = {
	.enter		= msm_pm_enter,
	.valid		= suspend_valid_only_mem,
};

static uint32_t restart_reason = 0x776655AA;

static void msm_pm_power_off(void)
{
	msm_proc_comm(PCOM_POWER_DOWN, 0, 0);
	for (;;) ;
}

static void msm_pm_restart(char str)
{
	/* If there's a hard reset hook and the restart_reason
	 * is the default, prefer that to the (slower) proc_comm
	 * reset command.
	 */
	if ((restart_reason == 0x776655AA) && msm_reset_hook) {
		msm_reset_hook(str);
	} else {
		msm_proc_comm(PCOM_RESET_CHIP, &restart_reason, 0);
	}
	for (;;) ;
}

static int msm_reboot_call(struct notifier_block *this, unsigned long code, void *_cmd)
{
	if((code == SYS_RESTART) && _cmd) {
		char *cmd = _cmd;
		if (!strcmp(cmd, "bootloader")) {
			restart_reason = 0x77665500;
		} else if (!strcmp(cmd, "recovery")) {
			restart_reason = 0x77665502;
		} else if (!strcmp(cmd, "eraseflash")) {
			restart_reason = 0x776655EF;
		} else if (!strncmp(cmd, "oem-", 4)) {
			unsigned code = simple_strtoul(cmd + 4, 0, 16) & 0xff;
			restart_reason = 0x6f656d00 | code;
		} else {
			restart_reason = 0x77665501;
		}
	}
	return NOTIFY_DONE;
}

static struct notifier_block msm_reboot_notifier =
{
	.notifier_call = msm_reboot_call,
};

static int __init msm_pm_init(void)
{
	pm_power_off = msm_pm_power_off;
	arm_pm_restart = msm_pm_restart;

	register_reboot_notifier(&msm_reboot_notifier);

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

	suspend_set_ops(&msm_pm_ops);
	return 0;
}

__initcall(msm_pm_init);
