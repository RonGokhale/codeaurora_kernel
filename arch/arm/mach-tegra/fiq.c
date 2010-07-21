/*
 * Copyright (C) 2010 Google, Inc.
 *
 * Author:
 *	Brian Swetland <swetland@google.com>
 *	Iliyan Malchev <malchev@google.com>
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
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/slab.h>

#include <asm/hardware/gic.h>

#include <mach/iomap.h>
#include <mach/fiq.h>
#include <mach/legacy_irq.h>

#include "board.h"

void tegra_fiq_enable(int irq)
{
	void __iomem *base = IO_ADDRESS(TEGRA_ARM_PERIF_BASE + 0x100);
	/* enable FIQ */
	u32 val = readl(base + GIC_CPU_CTRL);
	val &= ~8; /* pass FIQs through */
	val |= 2; /* enableNS */
	writel(val, base + GIC_CPU_CTRL);
	tegra_legacy_unmask_irq(irq);
}

void tegra_fiq_disable(int irq)
{
	tegra_legacy_mask_irq(irq);
}

void tegra_fiq_select(int irq, int on)
{
	tegra_legacy_select_fiq(irq, !!on);
}

extern unsigned char fiq_glue, fiq_glue_end;

static void (*fiq_func)(void *data, void *regs, void *svc_sp);
static void *fiq_data;
static void *fiq_stack;

void fiq_glue_setup(void *func, void *data, void *sp);
void set_fiq_handler(void *start, unsigned int length);

int tegra_fiq_set_handler(void (*func)(void *data, void *regs, void *svc_sp),
		void *data)
{
	unsigned long flags;
	int rc = -ENOMEM;

	if (!fiq_stack)
		fiq_stack = kmalloc(THREAD_SIZE, GFP_KERNEL);
	if (!fiq_stack)
		return rc;

	local_irq_save(flags);
	if (fiq_func == 0) {
		fiq_func = func;
		fiq_data = data;
		fiq_glue_setup(func, data, fiq_stack + THREAD_START_SP);
		set_fiq_handler(&fiq_glue, (&fiq_glue_end - &fiq_glue));
		rc = 0;
	}
	local_irq_restore(flags);

	return rc;
}
