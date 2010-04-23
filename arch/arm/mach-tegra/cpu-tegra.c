/*
 * arch/arm/mach-tegra/cpu-tegra.c
 *
 * Copyright (C) 2010 Google, Inc.
 *
 * Author:
 *	Colin Cross <ccross@google.com>
 *	Based on arch/arm/plat-omap/cpu-omap.c, (C) 2005 Nokia Corporation
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/cpufreq.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>

#include <mach/hardware.h>
#include <asm/system.h>

static struct cpufreq_frequency_table freq_table[] = {
	{ 1, 750000 },
	{ 2, 1000000 },
	{ 0, CPUFREQ_TABLE_END },
};

#define CPU_CLK		"cpu"
#define SWITCH_CLK	"clk_m"
#define MAIN_CLK	"pll_x"

#define NUM_CPUS	2

static struct clk *cpu_clk;
static struct clk *switch_clk;
static struct clk *main_clk;

static unsigned long target_cpu_speed[NUM_CPUS];

int tegra_verify_speed(struct cpufreq_policy *policy)
{
	return cpufreq_frequency_table_verify(policy, freq_table);
}

unsigned int tegra_getspeed(unsigned int cpu)
{
	unsigned long rate;

	if (cpu >= NUM_CPUS)
		return 0;

	rate = clk_get_rate(cpu_clk) / 1000;
	return rate;
}

static int tegra_update_cpu_speed(void)
{
	int i;
	unsigned long rate = 0;
	int ret = 0;
	struct cpufreq_freqs freqs;

	for_each_online_cpu(i)
		rate = max(rate, target_cpu_speed[i]);

	freqs.old = tegra_getspeed(0);
	freqs.new = rate;

	if (freqs.old == freqs.new)
		return ret;

	for_each_online_cpu(freqs.cpu)
		cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);

#ifdef CONFIG_CPU_FREQ_DEBUG
	printk(KERN_DEBUG "cpufreq-tegra: transition: %u --> %u\n",
	       freqs.old, freqs.new);
#endif

	ret = clk_set_parent(cpu_clk, switch_clk);
	if (ret) {
		pr_err("Failed to switch cpu to clock %s\n", SWITCH_CLK);
		return ret;
	}

	ret = clk_set_rate(main_clk, freqs.new * 1000);
	if (ret) {
		pr_err("Failed to change cpu pll to %d\n", freqs.new * 1000);
		return ret;
	}

	ret = clk_set_parent(cpu_clk, main_clk);
	if (ret) {
		pr_err("Failed to switch cpu to clock %s\n", MAIN_CLK);
		return ret;
	}

	for_each_online_cpu(freqs.cpu)
		cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);

	return 0;
}

static int tegra_target(struct cpufreq_policy *policy,
		       unsigned int target_freq,
		       unsigned int relation)
{
	/* Ensure desired rate is within allowed range.  Some govenors
	 * (ondemand) will just pass target_freq=0 to get the minimum. */
	clamp(target_freq, policy->min, policy->max);

	target_cpu_speed[policy->cpu] = target_freq;

	return tegra_update_cpu_speed();
}

static int __init tegra_cpu_init(struct cpufreq_policy *policy)
{
	if (policy->cpu >= NUM_CPUS)
		return -EINVAL;

	cpu_clk = clk_get_sys(NULL, CPU_CLK);
	if (IS_ERR(cpu_clk))
		return PTR_ERR(cpu_clk);

	main_clk = clk_get_sys(NULL, MAIN_CLK);
	if (IS_ERR(main_clk))
		return PTR_ERR(main_clk);

	switch_clk = clk_get_sys(NULL, SWITCH_CLK);
	if (IS_ERR(switch_clk))
		return PTR_ERR(switch_clk);

	cpufreq_frequency_table_cpuinfo(policy, freq_table);
	policy->cur = policy->min = policy->max = tegra_getspeed(policy->cpu);

	/* FIXME: what's the actual transition time? */
	policy->cpuinfo.transition_latency = 300 * 1000;

	return 0;
}

static int tegra_cpu_exit(struct cpufreq_policy *policy)
{
	clk_put(cpu_clk);
	clk_put(main_clk);
	clk_put(switch_clk);
	return 0;
}

static struct freq_attr *tegra_cpufreq_attr[] = {
	&cpufreq_freq_attr_scaling_available_freqs,
	NULL,
};

static struct cpufreq_driver tegra_cpufreq_driver = {
	.verify		= tegra_verify_speed,
	.target		= tegra_target,
	.get		= tegra_getspeed,
	.init		= tegra_cpu_init,
	.exit		= tegra_cpu_exit,
	.name		= "tegra",
	.attr		= tegra_cpufreq_attr,
};

static int __init tegra_cpufreq_init(void)
{
	return cpufreq_register_driver(&tegra_cpufreq_driver);
}

static void __exit tegra_cpufreq_exit(void)
{
        cpufreq_unregister_driver(&tegra_cpufreq_driver);
}


MODULE_AUTHOR("Colin Cross <ccross@android.com>");
MODULE_DESCRIPTION("cpufreq driver for Nvidia Tegra2");
MODULE_LICENSE("GPL");
module_init(tegra_cpufreq_init);
module_exit(tegra_cpufreq_exit);
