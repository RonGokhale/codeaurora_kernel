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
 *  arch/arm/mach-dan/include/mach/io.h
 */
#ifndef __ASM_ARM_ARCH_IO_H
#define __ASM_ARM_ARCH_IO_H

#include <asm/sizes.h>

#define __io(a)				__typesafe_io(a)

#define IO_VIRT_BASE			0x80000000

#define IO_ADDRESS(x)			( (x) | IO_VIRT_BASE )

#define readb(c)                ({ u8  __v = (u8)(*(volatile u32 *)(c)); __v; })
#define readw(c)                ({ u16 __v = (u16)(*(volatile u32 *)(c)); __v; })
#define readl(c)                ({ u32 __v = (*(volatile u32 *)(c)); __v; })

#define writeb(v,c)             ({ (*(volatile u32 *)(c)) = (u32)v; })
#define writew(v,c)             ({ (*(volatile u32 *)(c)) = (u32)v; })
#define writel(v,c)             ({ (*(volatile u32 *)(c)) = (u32)v; })

#define memcpy_fromio(a,c,l)    _memcpy_fromio((a),(c),(l))
#define memcpy_toio(c,a,l)      _memcpy_toio((c),(a),(l))

#endif /* __ASM_ARM_ARCH_IO_H */
