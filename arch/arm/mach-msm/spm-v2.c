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
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <mach/msm_iomap.h>

#include "spm.h"


enum {
	MSM_SPM_DEBUG_SHADOW = 1U << 0,
	MSM_SPM_DEBUG_VCTL = 1U << 1,
};

static int msm_spm_debug_mask;
module_param_named(
	debug_mask, msm_spm_debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP
);

#define MSM_SPM_PMIC_STATE_IDLE  0


static uint32_t msm_spm_reg_offsets[MSM_SPM_REG_NR] = {
	[MSM_SPM_REG_SAW2_SECURE] = 0x00,

	[MSM_SPM_REG_SAW2_ID] = 0x04,
	[MSM_SPM_REG_SAW2_CFG] = 0x08,
	[MSM_SPM_REG_SAW2_STS0] = 0x0C,
	[MSM_SPM_REG_SAW2_STS1] = 0x10,

	[MSM_SPM_REG_SAW2_VCTL] = 0x14,

	[MSM_SPM_REG_SAW2_AVS_CTL] = 0x18,
	[MSM_SPM_REG_SAW2_AVS_HYSTERESIS] = 0x1C,

	[MSM_SPM_REG_SAW2_SPM_CTL] = 0x20,
	[MSM_SPM_REG_SAW2_PMIC_DLY] = 0x24,
	[MSM_SPM_REG_SAW2_PMIC_DATA_0] = 0x28,
	[MSM_SPM_REG_SAW2_PMIC_DATA_1] = 0x2C,
	[MSM_SPM_REG_SAW2_RST] = 0x30,

	[MSM_SPM_REG_SAW2_SEQ_ENTRY] = 0x80,
};

struct msm_spm_power_modes {
	uint32_t mode;
	bool notify_rpm;
	uint32_t start_addr;
};

struct msm_spm_device {
	void __iomem *reg_base_addr;
	uint32_t reg_shadow[MSM_SPM_REG_NR_INITIALIZE];
	uint32_t *reg_seq_entry_shadow;

	uint8_t awake_vlevel;
	uint32_t vctl_timeout_us;

	uint32_t low_power_mode;
	bool notify_rpm;
	bool dirty;

	uint32_t num_modes;
	struct msm_spm_power_modes *modes;
};

static DEFINE_PER_CPU_SHARED_ALIGNED(struct msm_spm_device, msm_spm_devices);
static atomic_t msm_spm_set_vdd_x_cpu_allowed = ATOMIC_INIT(1);
/******************************************************************************
 * Internal helper functions
 *****************************************************************************/

static inline void msm_spm_set_vctl(
	struct msm_spm_device *dev, uint32_t vlevel)
{
	dev->reg_shadow[MSM_SPM_REG_SAW2_VCTL] &= ~0xFF;
	dev->reg_shadow[MSM_SPM_REG_SAW2_VCTL] |= vlevel;

	dev->reg_shadow[MSM_SPM_REG_SAW2_PMIC_DATA_0] &= ~0xFF;
	dev->reg_shadow[MSM_SPM_REG_SAW2_PMIC_DATA_0] |= vlevel;
}

static inline void msm_spm_set_pmic_ctl(struct msm_spm_device *dev,
	uint32_t awake_vlevel, uint32_t mid_vlevel, uint32_t sleep_vlevel)
{
	/* Currently the data is set through MSM_SPM_SET_PMIC_DATA0
	 * and MSM_SPM_SET_PMIC_DATA1 at init time.
	 * Since the sequencer controls the process we can set it at init
	 * time and would not have to worry about having to set it every time
	 * we do a power collapse.
	 */
}

static void msm_spm_flush_shadow(
	struct msm_spm_device *dev, unsigned int reg_index)
{
	if (dev->reg_shadow[reg_index])
		__raw_writel(dev->reg_shadow[reg_index],
			dev->reg_base_addr + msm_spm_reg_offsets[reg_index]);
}

static void msm_spm_load_shadow(
	struct msm_spm_device *dev, unsigned int reg_index)
{
	dev->reg_shadow[reg_index] =
		__raw_readl(dev->reg_base_addr +
				msm_spm_reg_offsets[reg_index]);
}

static inline uint32_t msm_spm_get_awake_vlevel(struct msm_spm_device *dev)
{
	return dev->reg_shadow[MSM_SPM_REG_SAW2_PMIC_DATA_0] & 0xFF;
}

static inline uint32_t msm_spm_get_sts_pmic_state(struct msm_spm_device *dev)
{
	return (dev->reg_shadow[MSM_SPM_REG_SAW2_STS0] >> 10) & 0x03;
}

static inline uint32_t msm_spm_get_sts_curr_pmic_data(
	struct msm_spm_device *dev)
{
	return dev->reg_shadow[MSM_SPM_REG_SAW2_STS1] & 0xFF;
}

static inline uint32_t msm_spm_get_num_spm_entry(
		struct msm_spm_device *dev)
{
	return 32;
}

static inline void msm_spm_set_start_addr(
		struct msm_spm_device *dev, uint32_t addr)
{
	addr &= 0x7F;
	addr <<= 4;
	dev->reg_shadow[MSM_SPM_REG_SAW2_SPM_CTL] &= 0xFFFFF80F;
	dev->reg_shadow[MSM_SPM_REG_SAW2_SPM_CTL] |= addr;
}

static inline void msm_spm_set_spm_enable(
		struct msm_spm_device *dev, int enable)
{
	enable &= 0x01;
	dev->reg_shadow[MSM_SPM_REG_SAW2_SPM_CTL] |= enable;
}

static inline uint32_t msm_spm_write_seq_data(
		struct msm_spm_device *dev, uint8_t *cmd, uint32_t offset)
{
	uint32_t cmd_w;
	uint32_t ret = 0;
	uint32_t offset_w = offset / 4;
	uint8_t last_cmd = 0;
	int num_spm_entry = msm_spm_get_num_spm_entry(dev);

	while (1) {
		int i;
		cmd_w = 0;
		for (i = 0; i < 4; i++) {
			last_cmd = (last_cmd == 0x0f) ? 0x0f : *(cmd + i);
			cmd_w |= last_cmd << (i * 8);
			ret++;
		}

		WARN_ON(offset_w > num_spm_entry);
		cmd += i;
		dev->reg_seq_entry_shadow[offset_w++] = cmd_w;
		if (last_cmd == 0x0f)
			break;
	}
	return ret;
}

static inline void msm_spm_flush_seq_entry(
		struct msm_spm_device *dev)
{
	int i;
	int num_spm_entry = msm_spm_get_num_spm_entry(dev);

	for (i = 0; i < num_spm_entry; i++) {
		__raw_writel(dev->reg_seq_entry_shadow[i],
			dev->reg_base_addr
			+ msm_spm_reg_offsets[MSM_SPM_REG_SAW2_SEQ_ENTRY]
			+ 4 * i);
	}
	dsb();
}

/******************************************************************************
 * Public functions
 *****************************************************************************/
int msm_spm_set_low_power_mode(unsigned int mode, bool notify_rpm)
{

	struct msm_spm_device *dev = &__get_cpu_var(msm_spm_devices);
	uint32_t i;
	uint32_t start_addr = 0;

	for (i = 0; i < dev->num_modes; i++) {
		if ((dev->modes[i].mode == mode) &&
			(dev->modes[i].notify_rpm == notify_rpm)) {
			start_addr = dev->modes[i].start_addr;
			break;
		}
	}
	/* SPM is configured to reset start address to zero after end of Program
	 */
	msm_spm_set_start_addr(dev, start_addr);

	if (start_addr) {
		msm_spm_flush_shadow(dev, MSM_SPM_REG_SAW2_SPM_CTL);
		wmb();
	}

	dev->low_power_mode = mode;
	dev->notify_rpm = notify_rpm;
	dev->dirty = false;

	if (msm_spm_debug_mask & MSM_SPM_DEBUG_SHADOW) {
		int i;
		for (i = 0; i < MSM_SPM_REG_NR; i++)
			pr_info("%s: reg %02x = 0x%08x\n", __func__,
				msm_spm_reg_offsets[i], dev->reg_shadow[i]);
	}

	return 0;
}

int msm_spm_set_vdd(unsigned int cpu, unsigned int vlevel)
{
	unsigned long flags;
	struct msm_spm_device *dev;
	uint32_t timeout_us;

	local_irq_save(flags);

	if (!atomic_read(&msm_spm_set_vdd_x_cpu_allowed) &&
				unlikely(smp_processor_id() != cpu)) {
		if (msm_spm_debug_mask & MSM_SPM_DEBUG_VCTL)
			pr_info("%s: attempting to set vdd of cpu %u from "
				"cpu %u\n", __func__, cpu, smp_processor_id());
		goto set_vdd_x_cpu_bail;
	}

	dev = &per_cpu(msm_spm_devices, cpu);

	if (msm_spm_debug_mask & MSM_SPM_DEBUG_VCTL)
		pr_info("%s: requesting cpu %u vlevel 0x%x\n",
			__func__, cpu, vlevel);

	msm_spm_set_vctl(dev, vlevel);
	msm_spm_flush_shadow(dev, MSM_SPM_REG_SAW2_VCTL);
	msm_spm_flush_shadow(dev, MSM_SPM_REG_SAW2_PMIC_DATA_0);
	dsb();

	/* Wait for PMIC state to return to idle or until timeout */
	timeout_us = dev->vctl_timeout_us;
	msm_spm_load_shadow(dev, MSM_SPM_REG_SAW2_STS0);
	while (msm_spm_get_sts_pmic_state(dev) != MSM_SPM_PMIC_STATE_IDLE) {
		if (!timeout_us)
			goto set_vdd_bail;

		if (timeout_us > 10) {
			udelay(10);
			timeout_us -= 10;
		} else {
			udelay(timeout_us);
			timeout_us = 0;
		}
		msm_spm_load_shadow(dev, MSM_SPM_REG_SAW2_STS0);
	}

	msm_spm_load_shadow(dev, MSM_SPM_REG_SAW2_STS1);

	if (msm_spm_get_sts_curr_pmic_data(dev) != vlevel)
		goto set_vdd_bail;

	dev->awake_vlevel = vlevel;
	dev->dirty = true;

	if (msm_spm_debug_mask & MSM_SPM_DEBUG_VCTL)
		pr_info("%s: cpu %u done, remaining timeout %uus\n",
			__func__, cpu, timeout_us);

	local_irq_restore(flags);
	return 0;

set_vdd_bail:
	pr_err("%s: cpu %u failed, remaining timeout %uus, vlevel 0x%x\n",
	       __func__, cpu, timeout_us, msm_spm_get_sts_curr_pmic_data(dev));

set_vdd_x_cpu_bail:
	local_irq_restore(flags);
	return -EIO;
}

void msm_spm_reinit(void)
{
	struct msm_spm_device *dev = &__get_cpu_var(msm_spm_devices);
	int i;

	for (i = 0; i < MSM_SPM_REG_NR_INITIALIZE; i++)
		msm_spm_flush_shadow(dev, i);

	msm_spm_flush_seq_entry(dev);
	dsb();
}

void msm_spm_allow_x_cpu_set_vdd(bool allowed)
{
	atomic_set(&msm_spm_set_vdd_x_cpu_allowed, allowed ? 1 : 0);
}

int __init msm_spm_init(struct msm_spm_platform_data *data, int nr_devs)
{
	unsigned int cpu;

	BUG_ON(nr_devs < num_possible_cpus());

	for_each_possible_cpu(cpu) {
		struct msm_spm_device *dev;
		int num_spm_entry;

		int i, offset = 0;
		dev = &per_cpu(msm_spm_devices, cpu);

		dev->reg_base_addr = data[cpu].reg_base_addr;
		memcpy(dev->reg_shadow, data[cpu].reg_init_values,
			sizeof(data[cpu].reg_init_values));

		dev->awake_vlevel = data[cpu].awake_vlevel;
		dev->vctl_timeout_us = data[cpu].vctl_timeout_us;

		for (i = 0; i < MSM_SPM_REG_NR_INITIALIZE; i++)
			msm_spm_flush_shadow(dev, i);
		/* barrier to ensure write completes before we update shadow
		 * registers
		 */
		dsb();

		for (i = 0; i < MSM_SPM_REG_NR_INITIALIZE; i++)
			msm_spm_load_shadow(dev, i);

		/* barrier to ensure read completes before we proceed further*/
		dsb();

		num_spm_entry = msm_spm_get_num_spm_entry(dev);
		dev->low_power_mode = MSM_SPM_MODE_CLOCK_GATING;
		dev->awake_vlevel = msm_spm_get_awake_vlevel(dev);
		dev->notify_rpm = false;
		dev->dirty = true;

		BUG_ON(!data[cpu].modes);

		dev->reg_seq_entry_shadow =
			kmalloc(sizeof(*dev->reg_seq_entry_shadow)
				* num_spm_entry, GFP_KERNEL);

		memset(dev->reg_seq_entry_shadow, 0x0f,
			num_spm_entry * sizeof(*dev->reg_seq_entry_shadow));

		dev->num_modes = data[cpu].num_modes;

		dev->modes = kmalloc(sizeof(struct msm_spm_power_modes)
				* dev->num_modes, GFP_KERNEL);

		for (i = 0; i < dev->num_modes; i++) {
			dev->modes[i].mode = data[cpu].modes[i].mode;
			dev->modes[i].notify_rpm =
				data[cpu].modes[i].notify_rpm;
			dev->modes[i].start_addr = offset;
			offset += msm_spm_write_seq_data(dev,
					data[cpu].modes[i].cmd, offset);
		}
		msm_spm_flush_seq_entry(dev);

	}

	return 0;
}
