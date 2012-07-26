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
 *  linux/arch/arm/mach-dan/map_io.c
 */
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <asm/page.h>

#include <asm/mach/map.h>
#include <mach/hardware.h>

#include "board.h"


static struct map_desc dan_io_desc[] __initdata = {
	{
		.virtual	= IO_ADDRESS(DAN_ICTL_BASE_ARM),
		.pfn		= __phys_to_pfn(DAN_ICTL_BASE_ARM),
		.length		= DAN_ICTL_MAP_SIZE,
		.type		= MT_DEVICE
	},
	{
		.virtual	= IO_ADDRESS(DAN_UART_BASE),
		.pfn		= __phys_to_pfn(DAN_UART_BASE),
		.length		= DAN_UART_MAP_SIZE,
		.type		= MT_DEVICE
	},
#if defined(CONFIG_ARCH_DAN3x00)
	{
		.virtual	= IO_ADDRESS(DAN_CLOCK_SOURCE_BASE),
		.pfn		= __phys_to_pfn(DAN_CLOCK_SOURCE_BASE),
		.length		= SZ_8K,
		.type		= MT_DEVICE
	},
	{
		.virtual	= IO_ADDRESS(DAN_TIMER_BASE),
		.pfn		= __phys_to_pfn(DAN_TIMER_BASE),
		.length		= SZ_8K,
		.type		= MT_DEVICE
	},
	{
		.virtual	= IO_ADDRESS(DAN_SPI_BASE),
		.pfn		= __phys_to_pfn(DAN_SPI_BASE),
		.length		= SZ_8K,
		.type		= MT_DEVICE
	},
	{
		.virtual	= IO_ADDRESS(DAN_MMAC_BASE),
		.pfn		= __phys_to_pfn(DAN_MMAC_BASE),
		.length		= SZ_8K,
		.type		= MT_DEVICE
	},
	{
		.virtual	= IO_ADDRESS(DAN_PM_BASE),
		.pfn		= __phys_to_pfn(DAN_PM_BASE),
		.length		= DAN_PM_MAP_SIZE,
		.type		= MT_DEVICE
	}
#endif
};

void __init dan_map_io(void)
{
	iotable_init(dan_io_desc, ARRAY_SIZE(dan_io_desc));
}
