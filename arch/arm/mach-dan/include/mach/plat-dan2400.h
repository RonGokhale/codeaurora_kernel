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
 */
/* ************************************************************************
 *
 *   DAN2400 platform
 *
 * ***********************************************************************/

#ifndef __PLAT_DAN2400_H__
#define __PLAT_DAN2400_H__


#define BUS_CLOCK			90909091

#define SPI_SPEED			( BUS_CLOCK / 6 )

#define GMAC_MII_FIXUP			0x2700
#define PHY_FIXUP_BUS_ID		"0:00"
#define PHY_ADDR			0x0

#define DAN_RST_OFFSET			0x30
#define DAN_RST_VALUE			0x00800000

#define MMAC_ENHANCED_DESC		0
#define MMAC_TX_COE			0

#define PHY_INTERFACE_MODE		PHY_INTERFACE_MODE_GMII

/**
 ** Memory map
 **/
/* DAN3x00 has address bit 31 always cleared, i.e. 0x9000_0000 = 0x1000_0000
 * 0x9xxx_xxxx is cached access, 0x1xxx_xxxx is uncached access.
 * I use this convention for both DAN3x00 and DAN2400.
 *
 * NOTE: set VMSPLIT_2G to to cause PAGE_OFFSET be defined as 0x80000000.
 */

/*
 * DAN2400 memory mapping:
 * 
 * Physical	Logical		Description	Size	Note
 * ----------------------------------------------------------------------------
 * 10000000	90000000	Main RAM	64MB	Passed via ATAG_MEM
 * 20000000	a0000a00	Ictl		1kB     For ARM2
 * 20000c00	a0000c00	Timers		1kB	No separate mapping,
 * 20001900	a0001900	Timer mask	1kB	mapped with ictl
 * 20001c00	a0001c00	SPI		1kB	mapped with ictl
 * 20002000	a0002000	UART		1kB
 * 20006000	a0006000	MMAC		16kB
 */

#define IO_SIZE				SZ_16K
#define IO_SPACE_LIMIT			0xffff

/* 8MB till beginning of I/O virtual memory */
#define DAN_VMALLOC_END			0x9F800000

#define DAN_IO_BASE			0x20000000
#define DAN_CGEN_BASE			( DAN_IO_BASE + 0x1d000 )
#define DAN_ICTL_BASE			( DAN_IO_BASE + 0x0000 )
#define DAN_ICTL_BASE_ARM		( DAN_ICTL_BASE + 0x0800 )
#define DAN_TIMER_BASE			( DAN_IO_BASE + 0x0c00 )
#define DAN_TIMER_MASK_BASE		( DAN_IO_BASE + 0x1900 )
#define DAN_SPI_BASE			( DAN_IO_BASE + 0x1c00 )
#define DAN_UART_BASE			( DAN_IO_BASE + 0x2000 )
#define DAN_MMAC_BASE			( DAN_IO_BASE + 0x6000 )
#define DAN_CLOCK_SOURCE_BASE		( DAN_TIMER_BASE )

#define DAN_ICTL_MAP_SIZE		( SZ_8K - SZ_1K )
#define DAN_UART_MAP_SIZE		SZ_4K

#define DAN_TIMER_IRQ_SET_ADDR		( DAN_TIMER_MASK_BASE + 2 * sizeof(dan_timer_mask_t) )
#define DAN_TIMER_IRQ_SET_VALUE		( 1 << 7 )
#define DAN_SPI_IRQ_SET_ADDR		( 0 )
#define DAN_SPI_IRQ_SET_VALUE		( 0 )
#define DAN_UART_IRQ_SET_ADDR		( 0 )
#define DAN_UART_IRQ_SET_VALUE		( 0 )
#define DAN_MMAC_IRQ_SET_ADDR		( 0 )
#define DAN_MMAC_IRQ_SET_VALUE		( 0 )

#endif /* __PLAT_DAN2400_H__ */
