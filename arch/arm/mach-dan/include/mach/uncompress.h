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
 * Based on arch/arm/mach-dan/include/mach/uncompress.h
 */

#ifndef __DAN_UNCOMPRESS_H
#define __DAN_UNCOMPRESS_H

#include <linux/serial_reg.h>
#include <mach/hardware.h>

static volatile uint32_t *UART = (volatile uint32_t *)DAN_UART_BASE;

/*
 * The following code assumes the serial port has already been
 * initialized by the bootloader.  If you didn't setup a port in
 * your bootloader then nothing will appear (which might be desired).
 */
static inline void putc(char c)
{
	while (!(UART[UART_LSR] & UART_LSR_THRE))
		barrier();
	UART[UART_TX] = c;
}

static inline void flush(void)
{
}

/*
 * nothing to do
 */
#define arch_decomp_setup()

#define arch_decomp_wdog()

#endif /* __DAN_UNCOMPRESS_H */
