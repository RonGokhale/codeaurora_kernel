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
#include <linux/pwm_backlight.h>
#include <mach/irqs.h>
#include <mach/iomap.h>
#include <mach/dc.h>
#include <mach/fb.h>
#include <mach/gpio.h>

#include "devices.h"
#include "gpio-names.h"
#include "board-seaboard.h"

static int seaboard_backlight_init(struct device *dev)
{
	int ret;

	ret = gpio_request(TEGRA_GPIO_BACKLIGHT_VDD, "backlight vdd");
	if (ret < 0)
		return ret;

	ret = gpio_direction_output(TEGRA_GPIO_BACKLIGHT_VDD, 1);
	if (ret < 0)
		goto free_bl_vdd;
	else
		tegra_gpio_enable(TEGRA_GPIO_BACKLIGHT_VDD);

	ret = gpio_request(TEGRA_GPIO_EN_VDD_PNL, "enable VDD to panel");
	if (ret < 0)
		goto free_bl_vdd;

	ret = gpio_direction_output(TEGRA_GPIO_EN_VDD_PNL, 1);
	if (ret < 0)
		goto free_en_vdd_pnl;
	else
		tegra_gpio_enable(TEGRA_GPIO_EN_VDD_PNL);

	ret = gpio_request(TEGRA_GPIO_BACKLIGHT, "backlight_enb");
	if (ret < 0)
		goto free_en_vdd_pnl;

	ret = gpio_direction_output(TEGRA_GPIO_BACKLIGHT, 1);
	if (ret < 0)
		goto free_bl_enb;
	else
		tegra_gpio_enable(TEGRA_GPIO_BACKLIGHT);

	return ret;

free_bl_enb:
	gpio_free(TEGRA_GPIO_BACKLIGHT);
free_en_vdd_pnl:
	gpio_free(TEGRA_GPIO_EN_VDD_PNL);
free_bl_vdd:
	gpio_free(TEGRA_GPIO_BACKLIGHT_VDD);

	return ret;
};

static void seaboard_backlight_exit(struct device *dev)
{
	gpio_set_value(TEGRA_GPIO_BACKLIGHT, 0);
	gpio_free(TEGRA_GPIO_BACKLIGHT);
	tegra_gpio_disable(TEGRA_GPIO_BACKLIGHT);

	gpio_set_value(TEGRA_GPIO_BACKLIGHT_VDD, 0);
	gpio_free(TEGRA_GPIO_BACKLIGHT_VDD);
	tegra_gpio_disable(TEGRA_GPIO_BACKLIGHT_VDD);

	gpio_set_value(TEGRA_GPIO_EN_VDD_PNL, 0);
	gpio_free(TEGRA_GPIO_EN_VDD_PNL);
	tegra_gpio_disable(TEGRA_GPIO_EN_VDD_PNL);
}

static int seaboard_backlight_notify(struct device *unused, int brightness)
{
	gpio_set_value(TEGRA_GPIO_BACKLIGHT_VDD, !!brightness);
	gpio_set_value(TEGRA_GPIO_EN_VDD_PNL, !!brightness);
	gpio_set_value(TEGRA_GPIO_BACKLIGHT, !!brightness);
	return brightness;
}

static struct platform_pwm_backlight_data seaboard_backlight_data = {
	.pwm_id		= 2,
	.max_brightness	= 255,
	.dft_brightness	= 224,
	.pwm_period_ns	= 5000000,
	.init		= seaboard_backlight_init,
	.exit		= seaboard_backlight_exit,
	.notify		= seaboard_backlight_notify,
};

static struct platform_device seaboard_backlight_device = {
	.name	= "pwm-backlight",
	.id	= -1,
	.dev	= {
		.platform_data = &seaboard_backlight_data,
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

static int seaboard_panel_enable(void)
{
	gpio_set_value(TEGRA_GPIO_EN_VDD_PNL, 1);
	return 0;
}

static int seaboard_panel_disable(void)
{
	gpio_set_value(TEGRA_GPIO_EN_VDD_PNL, 0);
	return 0;
}

static struct tegra_dc_out seaboard_disp1_out = {
	.type = TEGRA_DC_OUT_RGB,

	.align = TEGRA_DC_ALIGN_MSB,
	.order = TEGRA_DC_ORDER_RED_BLUE,

	.modes = seaboard_panel_modes,
	.n_modes = ARRAY_SIZE(seaboard_panel_modes),
	.enable = seaboard_panel_enable,
	.disable = seaboard_panel_disable,
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

static struct platform_device *seaboard_panel_devices[] __initdata = {
	&tegra_pwfm2_device,
	&seaboard_backlight_device,
};

int __init seaboard_panel_init(void)
{
	int err;

	tegra_gpio_enable(TEGRA_GPIO_LVDS_SHUTDOWN);

	err = gpio_request(TEGRA_GPIO_LVDS_SHUTDOWN, "lvds shutdown");
	if (err < 0) {
			pr_err("could not acquire LVDS shutdown GPIO\n");
	} else {
			gpio_direction_output(TEGRA_GPIO_LVDS_SHUTDOWN, 1);
			gpio_free(TEGRA_GPIO_LVDS_SHUTDOWN);
	}

	err = platform_add_devices(seaboard_panel_devices,
				ARRAY_SIZE(seaboard_panel_devices));

	if (!err)
		err = nvhost_device_register(&seaboard_disp1_device);

	return err;
}

