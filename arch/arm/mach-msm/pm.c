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
#include <linux/init.h>
#include <linux/pm.h>
#include <asm/arch/system.h>

static int msm_pm_enter(suspend_state_t state)
{
	void msm_irq_enter_sleep(void);
	void msm_irq_exit_sleep(void);
	void msm_gpio_enter_sleep(void);
	void msm_gpio_exit_sleep(void);

	printk(KERN_INFO "msm_pm_enter():\n");
	msm_gpio_enter_sleep();
	msm_irq_enter_sleep();
	arch_idle();
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
	pm_set_ops(&msm_pm_ops);
	return 0;
}

__initcall(msm_pm_init);
