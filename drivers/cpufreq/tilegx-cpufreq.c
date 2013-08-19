/*
 * Copyright 2012 Tilera Corporation. All Rights Reserved.
 *
 *   This program is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU General Public License
 *   as published by the Free Software Foundation, version 2.
 *
 *   This program is distributed in the hope that it will be useful, but
 *   WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE, GOOD TITLE or
 *   NON INFRINGEMENT.  See the GNU General Public License for
 *   more details.
 *
 * Support dynamic frequency changes for TILE-Gx.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/cpufreq.h>
#include <linux/timex.h>

static unsigned int tilegx_cpufreq_get(unsigned int cpu)
{
	return get_clock_rate() / 1000;
}

/**
 * tilegx_cpufreq_target() - set a new CPUFreq policy
 * @policy:		New policy.
 * @target_freq:	The target frequency.
 * @relation:		How that frequency relates to achieved frequency
 *			(CPUFREQ_RELATION_L or CPUFREQ_RELATION_H).
 */
static int tilegx_cpufreq_target(struct cpufreq_policy *policy,
				 unsigned int target_freq,
				 unsigned int relation)
{
	struct cpufreq_freqs freqs;
	long new_freq;
	HV_SetSpeed hvss = hv_set_speed(target_freq * 1000, 0,
					HV_SET_SPEED_DRYRUN |
					(relation == CPUFREQ_RELATION_L ?
					 HV_SET_SPEED_ROUNDUP : 0));
	new_freq = hvss.new_speed;

	/* If hv_set_speed failed, give up now. */
	if (new_freq < 0)
		return -ENOSYS;

	freqs.old = tilegx_cpufreq_get(0);
	freqs.new = new_freq / 1000;

	/* If they aren't changing the speed, nothing to do. */
	if (freqs.old == freqs.new)
		return 0;

	pr_debug("Changing Gx core frequency from %u to %u kHz\n",
		 freqs.old, freqs.new);

	cpufreq_notify_transition(policy, &freqs, CPUFREQ_PRECHANGE);

	freqs.new = set_clock_rate(new_freq);

	cpufreq_notify_transition(policy, &freqs, CPUFREQ_POSTCHANGE);

	return 0;
}

/**
 * tilegx_cpufreq_verify() - verify a new CPUFreq policy
 * @policy:		New policy.
 */
static int tilegx_cpufreq_verify(struct cpufreq_policy *policy)
{
	cpufreq_verify_within_limits(policy, policy->cpuinfo.min_freq,
				     policy->cpuinfo.max_freq);
	return 0;
}

/**
 * tilegx_cpufreq_cpu_init() - configure the TILE-Gx CPUFreq driver
 * @policy:		Policy.
 */
static int tilegx_cpufreq_cpu_init(struct cpufreq_policy *policy)
{
	unsigned int speed = tilegx_cpufreq_get(policy->cpu);
	HV_SetSpeed hvss;

	if (!speed)
		return -EIO;

	/* All of our CPUs share the same speed. */
	cpumask_copy(policy->cpus, cpu_online_mask);

	/* Set cpuinfo and default policy values. */
	policy->cur = speed;

	hvss = hv_set_speed(0, 0, HV_SET_SPEED_DRYRUN | HV_SET_SPEED_ROUNDUP);
	if (hvss.new_speed < 0)
		return -EPERM;
	policy->cpuinfo.min_freq = hvss.new_speed / 1000;

	hvss = hv_set_speed(LONG_MAX, 0, HV_SET_SPEED_DRYRUN);
	if (hvss.new_speed < 0)
		return -EPERM;
	policy->cpuinfo.max_freq = hvss.new_speed / 1000;

	/*
	 * This is worst-case, going from 40 MHz to 1200 MHz with voltage
	 * scaling enabled.  If you aren't doing anything that extreme,
	 * it'll be a lot lower, and you could reasonably tweak the
	 * governor sampling rate down via sysfs.
	 */
	policy->cpuinfo.transition_latency = 4200000;

	policy->cur = speed;
	policy->min = policy->cpuinfo.min_freq;
	policy->max = policy->cpuinfo.max_freq;

	policy->shared_type = CPUFREQ_SHARED_TYPE_ANY;

	return 0;
}

/**
 * tilegx_cpufreq_cpu_exit() - deconfigure the TILE-Gx CPUFreq driver
 * @policy:		Policy.
 */
static int tilegx_cpufreq_cpu_exit(struct cpufreq_policy *policy)
{
	return 0;
}

/* TILE-Gx CPUFreq attributes. */
static struct freq_attr *tilegx_cpufreq_attr[] = {
	NULL,
};

/* TILE-Gx CPUFreq operations vector. */
static struct cpufreq_driver tilegx_cpufreq_driver = {
	.name	= "tilegx_cpufreq",
	.verify	= tilegx_cpufreq_verify,
	.target	= tilegx_cpufreq_target,
	.init	= tilegx_cpufreq_cpu_init,
	.exit	= tilegx_cpufreq_cpu_exit,
	.get	= tilegx_cpufreq_get,
	.owner	= THIS_MODULE,
	.attr	= tilegx_cpufreq_attr,
};

/* Initialize the TILE-Gx CPUFreq driver. */
static int __init tilegx_cpufreq_init(void)
{
	return cpufreq_register_driver(&tilegx_cpufreq_driver);
}

/* Remove the TILE-Gx CPUFreq driver. */
static void __exit tilegx_cpufreq_exit(void)
{
	cpufreq_unregister_driver(&tilegx_cpufreq_driver);
}

MODULE_AUTHOR("Tilera Corporation");
MODULE_DESCRIPTION("CPU Frequency driver for TILE-Gx");
MODULE_LICENSE("GPL");

module_init(tilegx_cpufreq_init);
module_exit(tilegx_cpufreq_exit);
