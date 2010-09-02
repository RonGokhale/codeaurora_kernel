/*
 * arch/arm/mach-tegra/board-harmony-panel.c
 *
 * Copyright (C) 2010 Google, Inc.
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

#include <linux/resource.h>
#include <linux/platform_device.h>
#include <linux/nvhost.h>
#include <asm/mach-types.h>
#include <mach/irqs.h>
#include <mach/iomap.h>
#include <mach/dc.h>
#include <mach/fb.h>


/* Display Controller */
static struct resource harmony_panel_resources[] = {
	{
		.name   = "irq",
		.start  = INT_DISPLAY_GENERAL,
		.end    = INT_DISPLAY_GENERAL,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.name   = "regs",
		.start	= TEGRA_DISPLAY_BASE,
		.end	= TEGRA_DISPLAY_BASE + TEGRA_DISPLAY_SIZE-1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name   = "fbmem",
		.start	= 0x1c012000,
		.end	= 0x1c012000 + 0x500000 - 1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct tegra_dc_mode harmony_panel_modes[] = {
	{
		.pclk = 79500000,
		.h_ref_to_sync = 4,
		.v_ref_to_sync = 2,
		.h_sync_width = 136,
		.v_sync_width = 4,
		.h_back_porch = 138,
		.v_back_porch = 21,
		.h_active = 1024,
		.v_active = 600,
		.h_front_porch = 34,
		.v_front_porch = 4,
	},
};

static struct tegra_fb_data harmony_fb_data = {
	.win            = 0,
	.xres           = 1024,
	.yres           = 600,
	.bits_per_pixel = 24,
};

static struct tegra_dc_out harmony_panel_out = {
	.type = TEGRA_DC_OUT_RGB,

	.align = TEGRA_DC_ALIGN_MSB,
	.order = TEGRA_DC_ORDER_RED_BLUE,

	.modes = harmony_panel_modes,
	.n_modes = ARRAY_SIZE(harmony_panel_modes),
};

static struct tegra_dc_platform_data harmony_panel_pdata = {
	.flags       = TEGRA_DC_FLAG_ENABLED,
	.default_out = &harmony_panel_out,
	.fb          = &harmony_fb_data,
};

static struct nvhost_device harmony_panel_device = {
	.name          = "tegradc",
	.id            = 0,
	.resource      = harmony_panel_resources,
	.num_resources = ARRAY_SIZE(harmony_panel_resources),
	.dev = {
		.platform_data = &harmony_panel_pdata,
	},
};

int __init harmony_panel_init(void) {
	return nvhost_device_register(&harmony_panel_device);
}

