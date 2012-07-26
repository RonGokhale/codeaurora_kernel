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
 *  linux/arch/arm/mach-dan/init.c
 */
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/phy.h>

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

extern int dan_phy_fixup(struct phy_device *phydev);  // FIXME

static void __init dan_resources_init(void)
{
	platform_add_devices(dan_devs, dan_devs_arrsize);
	spi_register_board_info(dan_spi_board_info, dan_spi_board_info_arrsize);
}

void __init dan_init(void)
{
	dan_resources_init();
	phy_register_fixup_for_id(PHY_FIXUP_BUS_ID, dan_phy_fixup);
}

MACHINE_START(DAN, MACHINE_NAME)
	/* Maintainer: ... */
	.boot_params	= DAN_BOOT_PARAMS,
	.map_io		= dan_map_io,
	.init_irq	= dan_init_irq,
	.timer		= &dan_timer,
	.init_machine	= dan_init,
MACHINE_END
