/* linux/arch/arm/mach-msm/common.c
 *
 * Common setup code for MSM7K Boards
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

#include <asm/mach/flash.h>
#include <asm/io.h>

#include <asm/setup.h>

#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>

#include <asm/arch/msm_iomap.h>

#include <asm/arch/board.h>
#include <asm/arch/system.h>

void (*msm_reset_hook)(char mode);

struct flash_platform_data msm_nand_data = {
	.parts		= 0,
	.nr_parts	= 0,
};

static struct resource msm_nand_resources[] = {
	[0] = {
		.start	= 7,
		.end	= 7,
		.flags	= IORESOURCE_DMA,
	},
};

static struct platform_device msm_nand_device = {
	.name		= "msm_nand",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(msm_nand_resources),
	.resource	= msm_nand_resources,
	.dev		= {
		.platform_data	= &msm_nand_data,
	},
};

static struct platform_device msm_smd_device = {
	.name	= "msm_smd",
	.id	= -1,
};

static struct resource msm_i2c_resources[] = {
	{
		.start	= MSM_I2C_BASE,
		.end	= MSM_I2C_BASE + MSM_I2C_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_PWB_I2C,
		.end	= INT_PWB_I2C,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device msm_i2c_device = {
	.name		= "msm_i2c",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(msm_i2c_resources),
	.resource	= msm_i2c_resources,
};

static struct platform_device *devices[] __initdata = {
	&msm_nand_device,
	&msm_smd_device,
	&msm_i2c_device,
};

void __init msm_add_devices(void)
{
	platform_add_devices(devices, ARRAY_SIZE(devices));
}

static struct resource msm_sdc1_resources[] = {
	{
		.start	= MSM_SDC1_BASE,
		.end	= MSM_SDC1_BASE + MSM_SDC1_SIZE -1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_SDC1_0,
		.end	= INT_SDC1_1,
		.flags	= IORESOURCE_IRQ,
	},
#if 0
	{
		.start	= 8,
		.end	= 8,
		.flags	= IORESOURCE_DMA,
	},
#endif
};

static struct platform_device msm_sdc1_device = {
	.name		= "msm_sdcc",
	.id		= 1,
	.num_resources	= ARRAY_SIZE(msm_sdc1_resources),
	.resource	= msm_sdc1_resources,
	.dev		= {
		.coherent_dma_mask	= 0xffffffff,
	},
};

static struct resource msm_sdc2_resources[] = {
	{
		.start	= MSM_SDC2_BASE,
		.end	= MSM_SDC2_BASE + MSM_SDC2_SIZE -1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_SDC2_0,
		.end	= INT_SDC2_1,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= 8,
		.end	= 8,
		.flags	= IORESOURCE_DMA,
	},
};

static struct platform_device msm_sdc2_device = {
	.name		= "msm_sdcc",
	.id		= 2,
	.num_resources	= ARRAY_SIZE(msm_sdc2_resources),
	.resource	= msm_sdc2_resources,
	.dev		= {
		.coherent_dma_mask	= 0xffffffff,
	},
};

static struct resource msm_sdc3_resources[] = {
	{
		.start	= MSM_SDC3_BASE,
		.end	= MSM_SDC3_BASE + MSM_SDC3_SIZE -1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_SDC3_0,
		.end	= INT_SDC3_1,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= 8,
		.end	= 8,
		.flags	= IORESOURCE_DMA,
	},
};

static struct platform_device msm_sdc3_device = {
	.name		= "msm_sdcc",
	.id		= 3,
	.num_resources	= ARRAY_SIZE(msm_sdc3_resources),
	.resource	= msm_sdc3_resources,
	.dev		= {
		.coherent_dma_mask	= 0xffffffff,
	},
};

static struct resource msm_sdc4_resources[] = {
	{
		.start	= MSM_SDC4_BASE,
		.end	= MSM_SDC4_BASE + MSM_SDC4_SIZE -1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_SDC4_0,
		.end	= INT_SDC4_1,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= 8,
		.end	= 8,
		.flags	= IORESOURCE_DMA,
	},
};

static struct platform_device msm_sdc4_device = {
	.name		= "msm_sdcc",
	.id		= 4,
	.num_resources	= ARRAY_SIZE(msm_sdc4_resources),
	.resource	= msm_sdc4_resources,
	.dev		= {
		.coherent_dma_mask	= 0xffffffff,
	},
};

static struct platform_device *msm_sdcc_devices[] __initdata = {
	&msm_sdc1_device,
	&msm_sdc2_device,
	&msm_sdc3_device,
	&msm_sdc4_device,
};

int __init msm_add_sdcc(unsigned int controller, struct mmc_platform_data *plat)
{
	struct platform_device	*pdev;

	if (controller < 1 || controller > 4)
		return -EINVAL;

	pdev = msm_sdcc_devices[controller-1];
	pdev->dev.platform_data = plat;
	return platform_device_register(pdev);
}

