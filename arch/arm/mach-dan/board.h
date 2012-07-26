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
 *  linux/arch/arm/mach-dan/board.h
 */

#ifndef __MACH_DAN_BOARD_H
#define __MACH_DAN_BOARD_H

#include <linux/types.h>
#include <linux/spi/spi.h>

#include <mach/hardware.h>

extern void __init dan_init_irq(void);
extern void __init dan_map_io(void);
extern void __init dan_init(void);

extern void dan_bus_setup(void __iomem *ioaddr);

extern struct sys_timer __initdata dan_timer;

extern struct platform_device __initdata *dan_devs[];
extern struct spi_board_info __initdata dan_spi_board_info[];

extern const int dan_spi_board_info_arrsize;
extern const int dan_devs_arrsize;

extern void dan_unmask_irq(unsigned int irq);

#endif
