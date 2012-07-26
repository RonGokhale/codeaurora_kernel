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
 *  arch/arm/mach-dan/include/mach/irqs.h
 */

#ifndef __DAN_IRQS_H__
#define __DAN_IRQS_H__

#include <mach/hardware.h>
#include <mach/io.h>

#define IRQ_INTEN_L			( IO_ADDRESS(DAN_ICTL_BASE_ARM) + 0x00 )
#define IRQ_INTEN_H			( IO_ADDRESS(DAN_ICTL_BASE_ARM) + 0x04 )
#define IRQ_INTMASK_L			( IO_ADDRESS(DAN_ICTL_BASE_ARM) + 0x08 )
#define IRQ_INTMASK_H			( IO_ADDRESS(DAN_ICTL_BASE_ARM) + 0x0c )
#define IRQ_FINALSTATUS_L		( IO_ADDRESS(DAN_ICTL_BASE_ARM) + 0x30 )

#define IRQ_SET_PRESENT(irq)		( DAN_##irq##_IRQ_SET_ADDR )
#define IRQ_SET_REG(irq)		( IO_ADDRESS(DAN_##irq##_IRQ_SET_ADDR) )
#define IRQ_SET_VAL(irq)		( DAN_##irq##_IRQ_SET_VALUE )

#define ICTL_REG(reg)			( *((volatile uint32_t *)(reg)) )

#define IRQ_SET_DO(irq)			( ICTL_REG(IRQ_SET_REG(irq)) = IRQ_SET_VAL(irq) )
#define IRQ_SET(irq)			{ if (IRQ_SET_PRESENT(irq)) IRQ_SET_DO(irq); }

#define IRQ_SPI				11
#define IRQ_UART			12
#define IRQ_TIMER			14
#define IRQ_MMAC			20
#define IRQ_IPC				30

#define NR_IRQS                         32

#endif /* __DAN_IRQS_H__ */
