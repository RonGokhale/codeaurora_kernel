/*
	All files except if stated otherwise in the begining of the file are under the ISC license:
	-----------------------------------------------------------------------------------

	Copyright (c) 2010-2012 Design Art Networks Ltd.

	Permission to use, copy, modify, and/or distribute this software for any
	purpose with or without fee is hereby granted, provided that the above
	copyright notice and this permission notice appear in all copies.

	THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
	WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
	MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
	ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
	WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
	ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
	OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
*/


/*
 *  linux/arch/arm/mach-dan/devices.c
 */
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/mtd/physmap.h>
#include <linux/spi/flash.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/scatterlist.h>
#include <linux/spi/spi.h>
#include <linux/spi/dw_spi.h>
#include <linux/stmmac.h>
#include <linux/phy.h>
#include <linux/mii.h>
#include <linux/netdevice.h>
#include <linux/serial_8250.h>


#include <mach/hardware.h>
#include <mach/irqs.h>
#include <asm/irq.h>
#include <asm/setup.h>
#include <asm/param.h>		/* HZ */
#include <asm/mach-types.h>

#include <asm/mach/arch.h>
#include <asm/mach/irq.h>
#include <asm/mach/map.h>

#include "board.h"

static struct plat_serial8250_port serial_platform_data[] = {
	{
		.membase	= (void *)IO_ADDRESS(DAN_UART_BASE),
		.mapbase	= DAN_UART_BASE,
		.irq		= IRQ_UART,
		.irqflags	= IRQF_TRIGGER_HIGH,
		.uartclk	= BUS_CLOCK,
		.regshift	= 2,
		.iotype		= UPIO_DWAPB,
		.private_data	= (void *)IO_ADDRESS(DAN_UART_BASE + 0x7C),
		.type		= PORT_16550A,
		.flags		= UPF_BOOT_AUTOCONF | UPF_SKIP_TEST | UPF_FIXED_TYPE | UPF_SPD_VHI,
	},
	{},
};

static struct platform_device serial_device = {
	.name	= "serial8250",
	.id	= PLAT8250_DEV_PLATFORM,
	.dev	= {
		.platform_data = serial_platform_data,
	},
};


static struct resource stmmac_resource[] = {
        {
               .start = DAN_MMAC_BASE,
               .end = DAN_MMAC_BASE + SZ_16K,
               .flags = IORESOURCE_MEM
        },
        {
                .name   = "macirq",
                .start = IRQ_MMAC,
                .end = IRQ_MMAC,
                .flags = IORESOURCE_IRQ
        }
};

static struct plat_stmmacenet_data stmmac_platdata = {
        .bus_id 	= 0,
        .has_gmac	= 1,
	.enh_desc	= MMAC_ENHANCED_DESC,
	.pbl		= 4,
	.tx_coe		= MMAC_TX_COE,
	.bus_setup	= dan_bus_setup
};


static u64 eth_dmamask = DMA_BIT_MASK(32);
static struct platform_device stmmac_device = {
        .name           = "stmmaceth",
        .id             = 0,
        .num_resources  = ARRAY_SIZE(stmmac_resource),
        .resource       = stmmac_resource,
        .dev            = {
                .platform_data  	= &stmmac_platdata,
		.dma_mask		= &eth_dmamask,
		.coherent_dma_mask	= DMA_BIT_MASK(32),
        }
};

static struct plat_stmmacphy_data phy_private_data = {
        .bus_id = 0,
        .phy_addr = PHY_ADDR,
        .phy_mask = 0,
        .interface = PHY_INTERFACE_MODE,
        .phy_reset = NULL,
};

static struct platform_device stmphy_device = {
        .name           = "stmmacphy",
        .id             = 0,
        .num_resources  = 1,
        .resource       = (struct resource[]) {
                {
                        .name   = "phyirq",
                        .start  = -1 /*IRQ_MMAC*/,
                        .end    = -1 /*IRQ_MMAC*/,
                        .flags  = IORESOURCE_IRQ,
                },
        },
        .dev = {
                .platform_data = &phy_private_data,
         }
};

static struct resource dan_spi_resources[] = {
	{
		.start	= DAN_SPI_BASE,
		.end	= DAN_SPI_BASE + SZ_1K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= IRQ_SPI,
		.end	= IRQ_SPI,
		.flags	= IORESOURCE_IRQ,
	}
};

static struct platform_device dan_spi_device =
{
	.name           = "dw_spi_mmio",
	.resource	= dan_spi_resources,
	.num_resources	= ARRAY_SIZE(dan_spi_resources)
};

struct dw_spi_chip dan_spi_chip = 
{
	.poll_mode	= 1,
	.type           = SSI_MOTO_SPI,
	.enable_dma	= 0,
	.cs_control	= NULL,
};

static struct platform_device dan_flash_device =
{
	.name           = "m25p80",
	.resource	= NULL,
	.num_resources	= 0
};

static struct mtd_partition dan_flash0_parts[] = {
	{
		.name = "flash0 (apps.jffs2)",
		.size = SZ_1M * 7,
		.offset = SZ_2M,
		.mask_flags = 0
	},
};

static struct mtd_partition dan_flash1_parts[] = {
	{
		.name = "flash1 (rootfs.squashfs)",
		.size = MTDPART_SIZ_FULL,
		.offset = 0,
		.mask_flags = 0
	},
};

static struct mtd_partition dan_flash2_parts[] = {
	{
		.name = "flash2 (danapps.squashfs)",
		.size = MTDPART_SIZ_FULL,
		.offset = 0,
		.mask_flags = 0
	},
};

static struct mtd_partition dan_flash3_parts[] = {
	{
		.name = "flash3 (userdata.jffs2)",
		.size = 15 * SZ_1M,
		.offset = 0,
		.mask_flags = 0
	},
        {
                .name = "flash3 (local.jffs2)",
                .size = SZ_1M,
                .offset = MTDPART_OFS_APPEND,
                .mask_flags = 0
        },
};

static const struct flash_platform_data dan_flash0 = {
  	.type		= FLASH_TYPE,
	.name		= "spi_flash",
	.parts		= dan_flash0_parts,
	.nr_parts	= ARRAY_SIZE(dan_flash0_parts)
};

static const struct flash_platform_data dan_flash1 = {
  	.type		= FLASH_TYPE,
	.name		= "spi_flash",
	.parts		= dan_flash1_parts,
	.nr_parts	= ARRAY_SIZE(dan_flash1_parts)
};

static const struct flash_platform_data dan_flash2 = {
  	.type		= FLASH_TYPE,
	.name		= "spi_flash",
	.parts		= dan_flash2_parts,
	.nr_parts	= ARRAY_SIZE(dan_flash2_parts)
};

static const struct flash_platform_data dan_flash3 = {
  	.type		= FLASH_TYPE,
	.name		= "spi_flash",
	.parts		= dan_flash3_parts,
	.nr_parts	= ARRAY_SIZE(dan_flash3_parts)
};

struct spi_board_info __initdata dan_spi_board_info[] =
{
	{
		.modalias       = "m25p80",
		.max_speed_hz   = SPI_SPEED,
		.controller_data= &dan_spi_chip,
		.mode           = SPI_MODE_3,
		.bus_num        = 0,
		.chip_select    = 1,
		.platform_data	= &dan_flash1,
		.irq		= -1
	},
	{
		.modalias       = "m25p80",
		.max_speed_hz   = SPI_SPEED,
		.controller_data= &dan_spi_chip,
		.mode           = SPI_MODE_3,
		.bus_num        = 0,
		.chip_select    = 2,
		.platform_data	= &dan_flash2,
		.irq		= -1
	},
	{
		.modalias       = "m25p80",
		.max_speed_hz   = SPI_SPEED,
		.controller_data= &dan_spi_chip,
		.mode           = SPI_MODE_3,
		.bus_num        = 0,
		.chip_select    = 3,
		.platform_data	= &dan_flash3,
		.irq		= -1
	},
	{
		.modalias       = "m25p80",
		.max_speed_hz   = SPI_SPEED,
		.controller_data= &dan_spi_chip,
		.mode           = SPI_MODE_3,
		.bus_num        = 0,
		.chip_select    = 0,
		.platform_data	= &dan_flash0,
		.irq		= -1
	}
};

const int dan_spi_board_info_arrsize = ARRAY_SIZE(dan_spi_board_info);

struct platform_device *dan_devs[] __initdata = {
	&serial_device,
	&dan_spi_device,
	&dan_flash_device,
	&stmmac_device,
	&stmphy_device
};

const int dan_devs_arrsize = ARRAY_SIZE(dan_devs);
