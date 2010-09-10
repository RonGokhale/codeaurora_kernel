/*
 * arch/arm/mach-tegra/board-seaboard-panel.c
 *
 * Copyright (c) 2010, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/resource.h>
#include <asm/mach-types.h>
#include <linux/nvhost.h>
#include <linux/nvmap.h>
#include <linux/platform_device.h>
#include <mach/irqs.h>
#include <mach/iomap.h>
#include <mach/dc.h>
#include <mach/fb.h>
#include <mach/pwm-bl.h>
#include <mach/gpio.h>

#include "devices.h"
#include "gpio-names.h"
#include "board-seaboard.h"

static struct tegra_pwm_bl_platform_data seaboard_bl = {
	.pwr_gpio	= TEGRA_GPIO_PD4,
	.init_intensity	= 1,
};

static struct resource seaboard_pwm_resources[] = {
	[0] = {
		.start	= TEGRA_PWFM2_BASE,
		.end	= TEGRA_PWFM2_BASE + TEGRA_PWFM2_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct platform_device seaboard_pwm_bl_device = {
	.name		= "tegra-pwm-bl",
	.id		= 0,
	.resource	= seaboard_pwm_resources,
	.num_resources	= ARRAY_SIZE(seaboard_pwm_resources),
	.dev	= {
		.platform_data	= &seaboard_bl,
	},
};

static struct resource seaboard_disp1_resources[] = {
	{
		.name	= "irq",
		.start	= INT_DISPLAY_GENERAL,
		.end	= INT_DISPLAY_GENERAL,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name	= "regs",
		.start	= TEGRA_DISPLAY_BASE,
		.end	= TEGRA_DISPLAY_BASE + TEGRA_DISPLAY_SIZE-1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "fbmem",
		.start	= 0x18012000,
		.end	= 0x18414000 - 1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct tegra_dc_mode seaboard_panel_modes[] = {
	{
		.pclk = 62200000,
		.h_ref_to_sync = 11,
		.v_ref_to_sync = 1,
		.h_sync_width = 58,
		.v_sync_width = 4,
		.h_back_porch = 58,
		.v_back_porch = 4,
		.h_active = 1366,
		.v_active = 768,
		.h_front_porch = 58,
		.v_front_porch = 4,
	},
};

static struct tegra_fb_data seaboard_fb_data = {
	.win		= 0,
	.xres		= 1366,
	.yres		= 768,
	.pitch		= 2736,
	.bits_per_pixel	= 16,
};

static struct tegra_dc_out seaboard_disp1_out = {
	.type = TEGRA_DC_OUT_RGB,

	.align = TEGRA_DC_ALIGN_MSB,
	.order = TEGRA_DC_ORDER_RED_BLUE,

	.modes = seaboard_panel_modes,
	.n_modes = ARRAY_SIZE(seaboard_panel_modes),
};

static struct tegra_dc_platform_data seaboard_disp1_pdata = {
	.flags		= TEGRA_DC_FLAG_ENABLED,
	.default_out	= &seaboard_disp1_out,
	.fb		= &seaboard_fb_data,
};

static struct nvhost_device seaboard_disp1_device = {
	.name		= "tegradc",
	.id		= 0,
	.resource	= seaboard_disp1_resources,
	.num_resources	= ARRAY_SIZE(seaboard_disp1_resources),
	.dev = {
		.platform_data = &seaboard_disp1_pdata,
	},
};

int __init seaboard_panel_init(void)
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

	platform_device_register(&seaboard_pwm_bl_device);

	return nvhost_device_register(&seaboard_disp1_device);
}

