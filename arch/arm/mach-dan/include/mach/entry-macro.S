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
 * arch/arm/mach-dan/include/mach/entry-macro.S
 */

#include <mach/hardware.h>
#include <mach/irqs.h>


 		.macro	disable_fiq
		.endm

		.macro  get_irqnr_preamble, base, tmp
		.endm

		.macro  arch_ret_to_user, tmp1, tmp2
		.endm

	        .macro  get_irqnr_and_base, irqnr, irqstat, base, tmp
	        ldr     \irqstat, =IRQ_FINALSTATUS_L
	        ldr     \irqnr, [\irqstat]
	        cmp     \irqnr, #0
	        beq     2313f
	        mov     \tmp, \irqnr
	        mov     \irqnr, #0
2312:
	        tst     \tmp, #1
	        bne     2313f
	        add     \irqnr, \irqnr, #1
	        mov     \tmp, \tmp, lsr #1
		cmp     \irqnr, #31
		bcc     2312b
2313:
	        .endm
