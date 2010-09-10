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
#include <mach/pwm-bl.h>
#include <mach/gpio.h>

#include "devices.h"
#include "gpio-names.h"
#include "board-harmony.h"


static struct tegra_pwm_bl_platform_data harmony_bl = {
	.pwr_gpio	= TEGRA_GPIO_BACKLIGHT,
	.init_intensity	= 1,
};

static struct resource harmony_pwm_resources[] = {
	[0] = {
		.start	= TEGRA_PWFM2_BASE,
		.end	= TEGRA_PWFM2_BASE + TEGRA_PWFM2_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct platform_device harmony_pwm_bl_device = {
	.name		= "tegra-pwm-bl",
	.id		= 0,
	.resource	= harmony_pwm_resources,
	.num_resources	= ARRAY_SIZE(harmony_pwm_resources),
	.dev	= {
		.platform_data	= &harmony_bl,
	},
};

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

int __init harmony_panel_init(void) 
{
	int err;

	tegra_gpio_enable(TEGRA_GPIO_BACKLIGHT);
	tegra_gpio_enable(TEGRA_GPIO_BACKLIGHT_PWM);
	tegra_gpio_enable(TEGRA_GPIO_LVDS_SHUTDOWN);
	tegra_gpio_enable(TEGRA_GPIO_BACKLIGHT_VDD);
	tegra_gpio_enable(TEGRA_GPIO_EN_VDD_PNL);

	err = gpio_request(TEGRA_GPIO_LVDS_SHUTDOWN, "lvds shutdown");
	if (err < 0) {
		pr_err("could not acquire LVDS shutdown GPIO\n");
	} else {
		gpio_direction_output(TEGRA_GPIO_LVDS_SHUTDOWN, 1);
		gpio_free(TEGRA_GPIO_LVDS_SHUTDOWN);
	}

	err = gpio_request(TEGRA_GPIO_BACKLIGHT_VDD, "backlight vdd");
	if (err < 0) {
		pr_err("could not acquire backlight VDD GPIO\n");
	} else {
		gpio_direction_output(TEGRA_GPIO_BACKLIGHT_VDD, 1);
		gpio_free(TEGRA_GPIO_BACKLIGHT_VDD);
	}

	err = gpio_request(TEGRA_GPIO_BACKLIGHT_PWM, "backlight pwm");
	if (err < 0) {
		pr_err("could not acquire backlight PWM GPIP\n");
	} else {
		gpio_direction_output(TEGRA_GPIO_BACKLIGHT_PWM, 1);
		gpio_free(TEGRA_GPIO_BACKLIGHT_PWM);
	}

	err = gpio_request(TEGRA_GPIO_EN_VDD_PNL, "enable VDD to panel");
	if (err < 0) {
		pr_err("could not acquire panel VDD enable GPIO\n");
	} else {
		gpio_direction_output(TEGRA_GPIO_EN_VDD_PNL, 1);
		gpio_free(TEGRA_GPIO_EN_VDD_PNL);
	}

	platform_device_register(&harmony_pwm_bl_device);
	printk("panel init done\n");

	return nvhost_device_register(&harmony_panel_device);
}

