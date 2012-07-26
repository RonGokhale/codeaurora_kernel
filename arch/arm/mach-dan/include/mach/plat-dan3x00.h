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
 *   DAN3x00 platform
 *
 * ***********************************************************************/

#ifndef __PLAT_DAN3x00_H__
#define __PLAT_DAN3x00_H__


#define BUS_CLOCK			300000000

#define SPI_SPEED			( BUS_CLOCK / 21 )

#define GMAC_MII_FIXUP			0x00000000
#define PHY_FIXUP_BUS_ID		"0:15"
#define PHY_ADDR			0x15

#define DAN_RST_OFFSET			0x0
#define DAN_RST_VALUE			0x00010000

#define MMAC_ENHANCED_DESC		1
#define MMAC_TX_COE			1

#define PHY_INTERFACE_MODE		PHY_INTERFACE_MODE_RGMII

/**
 ** Memory map
 **/
/* DAN3x00 has address bit 31 always cleared, i.e. 0xa000_0000 = 0x2000_0000
 * 0xaxxx_xxxx is cached access, 0x2xxx_xxxx is uncached access.
 * I use this convention for both DAN3x00 and DAN2400.
 *
 * NOTE: set VMSPLIT_2G to to cause PAGE_OFFSET be defined as 0x80000000.
 */

/*
 * DAN3400 memory mapping:
 * 
 * Physical	Logical		Description	Size	Note
 * ----------------------------------------------------------------------------
 * 10000000	90000000	Main RAM	64MB	Passed via ATAG_MEM
 * 65764000	e5764000	Ictl		8kB     For ARM3
 * 65794000	e5794000	Timer		8kB	
 * 65770000	e5770000	SPI		8kB	
 * 6572e000	e572e000	UART		8kB
 * 65738000	e5738000	MMAC		8kB
 */

#define IO_SIZE				SZ_1024K
#define IO_SPACE_LIMIT			0xfffff

/* 8MB till beginning of I/O virtual memory */
#define DAN_VMALLOC_END			0xe4e00000

#define DAN_IO_BASE			0x65700000
#define DAN_CGEN_BASE			( DAN_IO_BASE + 0x3e000 )
#define DAN_ICTL_BASE			( DAN_IO_BASE + 0x64000 )
#define DAN_ICTL_BASE_ARM		( DAN_ICTL_BASE + 0x000 )
#define DAN_PM_BASE			( DAN_IO_BASE + 0x66000 )
#define DAN_TIMER_BASE			( DAN_IO_BASE + 0x94000 )
#define DAN_SPI_BASE			( DAN_IO_BASE + 0x70000 )
#define DAN_UART_BASE			( DAN_IO_BASE + 0x2e000 )
#define DAN_MMAC_BASE			( DAN_IO_BASE + 0x38000 )
#define DAN_CLOCK_SOURCE_BASE		( DAN_IO_BASE + 0x30000 )

#define DAN_ICTL_MAP_SIZE		SZ_8K
#define DAN_PM_MAP_SIZE			SZ_8K
#define DAN_UART_MAP_SIZE		SZ_8K

#define DAN_TIMER_IRQ_SET_ADDR		( DAN_IO_BASE + 0x6601c )
#define DAN_TIMER_IRQ_SET_VALUE		( 40 )
#define DAN_SPI_IRQ_SET_ADDR		( DAN_IO_BASE + 0x66014 )
#define DAN_SPI_IRQ_SET_VALUE		( 98 << 16 )
#define DAN_UART_IRQ_SET_ADDR		( DAN_IO_BASE + 0x66018 )
#define DAN_UART_IRQ_SET_VALUE		( 65 )
#define DAN_MMAC_IRQ_SET_ADDR		( DAN_IO_BASE + 0x66028 )
#define DAN_MMAC_IRQ_SET_VALUE		( 150 )
#define DAN_IPC_IRQ_SET_ADDR		( DAN_IO_BASE + 0x66000 + (30*2) )
#define DAN_IPC_IRQ_SET_VALUE		( 155 )

#define DAN3x00_CHIP

#endif /* __PLAT_DAN3x00_H__ */
