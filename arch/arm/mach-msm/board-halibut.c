/* linux/arch/arm/mach-msm/board-halibut.c
 *
 * Copyright (C) 2007 Google, Inc.
 * Author: Brian Swetland <swetland@google.com>
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
#include <linux/platform_device.h>
#include <linux/input.h>

#include <asm/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/flash.h>

#include <asm/arch/board.h>
#include <asm/arch/clock.h>
#include <asm/arch/msm_iomap.h>
#include <asm/arch/msm_fb.h>

#include <asm/io.h>
#include <asm/delay.h>
#include <linux/delay.h>

#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>

static int halibut_ffa;
module_param_named(ffa, halibut_ffa, int, S_IRUGO | S_IWUSR | S_IWGRP);

void halibut_init_keypad(int ffa);

static struct resource smc91x_resources[] = {
	[0] = {
		.start	= 0x9C004300,
		.end	= 0x9C004400,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= MSM_GPIO_TO_INT(49),
		.end	= MSM_GPIO_TO_INT(49),
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device smc91x_device = {
	.name		= "smc91x",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(smc91x_resources),
	.resource	= smc91x_resources,
};

#define MDDI_CLIENT_CORE_BASE  0x108000
#define LCD_CONTROL_BLOCK_BASE 0x110000
#define PWM_BLOCK_BASE         0x140000
#define DPSUS       (MDDI_CLIENT_CORE_BASE|0x24)
#define SYSCLKENA   (MDDI_CLIENT_CORE_BASE|0x2C)
#define START       (LCD_CONTROL_BLOCK_BASE|0x08)
#define PWM0OFF     (PWM_BLOCK_BASE|0x1C)

static void mddi0_panel_power(struct mddi_panel_info *panel, int on)
{
	if(on) {
		mddi_remote_write(panel->mddi, 0, DPSUS);
		udelay(122);
		mddi_remote_write(panel->mddi, 1, SYSCLKENA);
		mddi_remote_write(panel->mddi, halibut_ffa ? 0 : 0x00001387, PWM0OFF);
	}
	else {
		mddi_remote_write(panel->mddi, halibut_ffa ? 0x00001387 : 0, PWM0OFF);
		udelay(122);
		mddi_remote_write(panel->mddi, 0, SYSCLKENA);
		mddi_remote_write(panel->mddi, 1, DPSUS);
	}
}

static struct msm_mddi_platform_data msm_mddi0_pdata = {
	.panel_power	= mddi0_panel_power,
	.has_vsync_irq	= 0,
};

static struct platform_device msm_mddi0_device = {
	.name	= "msm_mddi",
	.id	= 0,
	.dev	= {
		.platform_data = &msm_mddi0_pdata
	},
};

static struct resource msm_serial0_resources[] = {
	{
		.start	= INT_UART1,
		.end	= INT_UART1,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= MSM_UART1_PHYS,
		.end	= MSM_UART1_PHYS + MSM_UART1_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct platform_device msm_serial0_device = {
	.name	= "msm_serial",
	.id	= 0,
	.num_resources	= ARRAY_SIZE(msm_serial0_resources),
	.resource	= msm_serial0_resources,
};

static struct resource usb_resources[] = {
	{
		.start	= MSM_HSUSB_PHYS,
		.end	= MSM_HSUSB_PHYS + MSM_HSUSB_SIZE,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_USB_HS,
		.end	= INT_USB_HS,
		.flags	= IORESOURCE_IRQ,
	},
};

/* The HSUSB PHY on Halibut has a hardware bug where VBUS
 * interrupts can lock up the ULPI bus, causing USB to fail.
 * Disable these interrupts to avoid this issue.
 */
static int halibut_phy_init_seq[] = { 0x1D, 0x0D, 0x1D, 0x10, -1 };

static struct msm_hsusb_platform_data msm_hsusb_pdata = {
	.phy_init_seq	= halibut_phy_init_seq,
	.vendor_id	= 0x18d1,
	.product_id	= 0xd00d,
	.version	= 0x0100,
	.product_name	= "Halibut",
};

static struct platform_device msm_hsusb_device = {
	.name		= "msm_hsusb",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(usb_resources),
	.resource	= usb_resources,
	.dev		= {
		.coherent_dma_mask	= 0xffffffff,
		.platform_data		= &msm_hsusb_pdata,
	},
};

static struct platform_device *devices[] __initdata = {
	&msm_serial0_device,
	&msm_mddi0_device,
	&msm_hsusb_device,
	&smc91x_device,
};

extern struct sys_timer msm_timer;

static void __init halibut_init_irq(void)
{
	msm_init_irq();
}

static struct msm_clock_platform_data halibut_clock_data = {
	.acpu_switch_time_us = 50,
	.max_speed_delta_khz = 256000,
	.vdd_switch_time_us = 62,
};

static void __init halibut_init(void)
{
	msm_init_gpio();
	platform_add_devices(devices, ARRAY_SIZE(devices));
	halibut_init_keypad(halibut_ffa);
	msm_add_devices();
	msm_clock_init(&halibut_clock_data);
}

static void __init halibut_map_io(void)
{
	msm_map_common_io();
}

MACHINE_START(HALIBUT, "Halibut Board (QCT SURF7200A)")

/* UART for LL DEBUG */
	.phys_io	= MSM_UART1_PHYS,
	.io_pg_offst	= ((MSM_UART1_BASE) >> 18) & 0xfffc,

	.boot_params	= 0x10000100,
	.map_io		= halibut_map_io,
	.init_irq	= halibut_init_irq,
	.init_machine	= halibut_init,
	.timer		= &msm_timer,
MACHINE_END
