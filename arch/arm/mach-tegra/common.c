/*
 * arch/arm/mach-tegra/common.c
 *
 * Copyright (C) 2010 Google, Inc.
 *
 * Author:
 *	Colin Cross <ccross@android.com>
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

#include <linux/console.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/delay.h>

#include <asm/hardware/cache-l2x0.h>
#include <asm/system.h>

#include <mach/iomap.h>
#include <mach/pinmux.h>
#include <mach/powergate.h>
#include <mach/system.h>

#include "apbio.h"
#include "board.h"
#include "clock.h"
#include "fuse.h"

void (*arch_reset)(char mode, const char *cmd) = tegra_assert_system_reset;

void tegra_assert_system_reset(char mode, const char *cmd)
{
	void __iomem *reset = IO_ADDRESS(TEGRA_CLK_RESET_BASE + 0x04);
	u32 reg;

	/* use *_related to avoid spinlock since caches are off */
	reg = readl_relaxed(reset);
	reg |= 0x04;
	writel_relaxed(reg, reset);
}

static __initdata struct tegra_clk_init_table common_clk_init_table[] = {
	/* name		parent		rate		enabled */
	{ "clk_m",	NULL,		0,		true },
	{ "pll_p",	"clk_m",	216000000,	true },
	{ "pll_m_out1",	"pll_m",	120000000,	true },
	{ "pll_c",	"clk_m",	600000000,	true },
	{ "pll_c_out1",	"pll_c",	120000000,	true },
	{ "pll_p_out1",	"pll_p",	28800000,	true },
	{ "pll_p_out2",	"pll_p",	48000000,	true },
	{ "pll_p_out3",	"pll_p",	72000000,	true },
	{ "pll_p_out4",	"pll_p",	108000000,	true },
	{ "sclk",	"pll_c_out1",	120000000,	true },
	{ "hclk",	"sclk",		120000000,	true },
	{ "pclk",	"hclk",		60000000,	true },
	{ "csite",	NULL,		0,		true },
	{ "emc",	NULL,		0,		true },
	{ "cpu",	NULL,		0,		true },

	/* reparent some clocks originally on pll_m */
	{ "3d",		"pll_c",	0,		false},
	{ "2d",		"pll_c",	0,		false},
	{ "vi",		"pll_c",	0,		false},
	{ "vi_sensor",	"pll_c",	0,		false},
	{ "epp",	"pll_c",	0,		false},
	{ "mpe",	"pll_c",	0,		false},
	{ "vde",	"pll_c",	0,		false},

	{ "kfuse",	NULL,		0,		true },
	{ "pll_u",	"clk_m",	480000000,	false },
	{ "sdmmc1",	"pll_p",	48000000,	false},
	{ "sdmmc2",	"pll_p",	48000000,	false},
	{ "sdmmc3",	"pll_p",	48000000,	false},
	{ "sdmmc4",	"pll_p",	48000000,	false},
	{ "csite",	NULL,		0,		true },
	{ "emc",	NULL,		0,		true },
	{ "cpu",	NULL,		0,		true },
	{ NULL,		NULL,		0,		0},
};

static void __init tegra_init_cache(void)
{
#ifdef CONFIG_CACHE_L2X0
	void __iomem *p = IO_ADDRESS(TEGRA_ARM_PERIF_BASE) + 0x3000;

	writel_relaxed(0x331, p + L2X0_TAG_LATENCY_CTRL);
	writel_relaxed(0x441, p + L2X0_DATA_LATENCY_CTRL);

	l2x0_init(p, 0x6C480001, 0x8200c3fe);
#endif

}

static void __init tegra_init_power(void)
{
	tegra_powergate_power_off(TEGRA_POWERGATE_MPE);
	tegra_powergate_power_off(TEGRA_POWERGATE_3D);
}

static bool console_flushed;

void tegra_pm_flush_console(void)
{
	if (console_flushed)
		return;
	console_flushed = true;

	printk("\n");
	pr_emerg("Restarting %s\n", linux_banner);
	if (console_trylock()) {
		console_unlock();
		return;
	}

	mdelay(50);

	local_irq_disable();
	if (!console_trylock())
		pr_emerg("tegra_restart: Console was locked! Busting\n");
	else
		pr_emerg("tegra_restart: Console was locked!\n");
	console_unlock();
}

static void tegra_pm_restart(char mode, const char *cmd)
{
	tegra_pm_flush_console();
	arm_machine_restart(mode, cmd);
}

void __init tegra_init_early(void)
{
	arm_pm_restart = tegra_pm_restart;
	tegra_init_fuse();
	tegra_init_clock();
	tegra_clk_init_from_table(common_clk_init_table);
	tegra_init_power();
	tegra_init_cache();
}
