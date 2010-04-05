/*
 * arch/arm/mach-tegra/legacy_irq.c
 *
 * Copyright (C) 2010 Google, Inc.
 * Author: Colin Cross <ccross@android.com>
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

#include <linux/io.h>
#include <mach/iomap.h>
#include "legacy_irq.h"

#define ICTLR_CPU_IER		0x20
#define ICTLR_CPU_IER_SET	0x24
#define ICTLR_CPU_IER_CLR	0x28
#define ICTLR_CPU_IEP_CLASS	0x2C

static void __iomem *ictlr_reg_base[] = {
	IO_ADDRESS(TEGRA_PRIMARY_ICTLR_BASE),
	IO_ADDRESS(TEGRA_SECONDARY_ICTLR_BASE),
	IO_ADDRESS(TEGRA_TERTIARY_ICTLR_BASE),
	IO_ADDRESS(TEGRA_QUATERNARY_ICTLR_BASE),
};

/* When going into deep sleep, the CPU is powered down, taking the GIC with it
   In order to wake, the wake interrupts need to be enabled in the legacy
   interrupt controller. */
void tegra_legacy_unmask_irq(unsigned int irq) {
	void __iomem *base;
	pr_debug("%s: %d\n", __FUNCTION__, irq);
	gic_unmask_irq(irq);

	irq -= 32;
	base = ictlr_reg_base[irq>>5];
	writel(1 << (irq & 31), base + ICTLR_CPU_IER_SET);
}

void tegra_legacy_mask_irq(unsigned int irq) {
	void __iomem *base;
	pr_debug("%s: %d\n", __FUNCTION__, irq);

	irq -= 32;
	base = ictlr_reg_base[irq>>5];
	writel(1 << (irq & 31), base + ICTLR_CPU_IER_CLR);
}
