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
 *  linux/arch/arm/mach-dan/timer.c
 */
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/sysdev.h>
#include <linux/clocksource.h>
#include <linux/clockchips.h>


#include <mach/timex.h>
#include <mach/irqs.h>
#include <asm/irq.h>
#include <asm/mach/time.h>

#include "board.h"

static volatile dan_timer_t *dan_timer_p = (volatile dan_timer_t *)(IO_ADDRESS(DAN_TIMER_BASE));
static volatile dan_timer_t *cs_timer_p = (volatile dan_timer_t *)(IO_ADDRESS(DAN_CLOCK_SOURCE_BASE));

#define TIMER		dan_timer_p[7]
#define CS_TIMER	cs_timer_p[0]

static int		one_shot_mode;

/*
 * IRQ handler for the timer
 */
static irqreturn_t dan_timer_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *evt = dev_id;
	volatile uint32_t	dummy;

	if (one_shot_mode) {
		TIMER.control_reg = 0;	/* Disable timer */
		TIMER.load_count = 0xffffffff;
	}

	dummy = TIMER.eoi;		/* Clear ... */
	dan_unmask_irq(irq);		/* ... and unmask timer interrupt */

	evt->event_handler(evt);

	return IRQ_HANDLED;
}

static cycle_t dan_clocksource_read(struct clocksource *cs)
{
	return (0xffffffff - CS_TIMER.current_value);
}



static struct clocksource dan_clocksource = {
	.name		= "wall_clock",
	.rating		= 300,
	.read		= dan_clocksource_read,
	.mask		= CLOCKSOURCE_MASK(32),
	.flags		= CLOCK_SOURCE_IS_CONTINUOUS,
};

static int clkevt_set_next_event(unsigned long delta, struct clock_event_device *evt)
{
	TIMER.control_reg	= 0;      /* Disable timer */
	TIMER.control_reg	= (1<<1); /* User-defined mode, unmasked. */
	TIMER.load_count	= delta;
	TIMER.control_reg	= 3;      /* Enable timer in reload mode */

	return 0;
}

static void clkevt_set_mode(enum clock_event_mode mode,
                struct clock_event_device *evt)
{

	switch(mode)
	{
	case CLOCK_EVT_MODE_SHUTDOWN:
		break;

	case CLOCK_EVT_MODE_PERIODIC:
		clkevt_set_next_event(CLOCK_TICK_RATE, NULL);
		one_shot_mode = 0;
		break;

	case CLOCK_EVT_MODE_ONESHOT:
		one_shot_mode = 1;
		break;

	case CLOCK_EVT_MODE_UNUSED:
	case CLOCK_EVT_MODE_RESUME:
	default:
		BUG();
		break;
	}
}


static struct clock_event_device dan_clockevent = {
	.name		= "dan_clockevent",
	.features	= CLOCK_EVT_FEAT_PERIODIC | CLOCK_EVT_FEAT_ONESHOT,
	.set_mode	= clkevt_set_mode,
	.set_next_event	= clkevt_set_next_event,
	.rating		= 300,
	.cpumask	= cpu_all_mask,
};


static struct irqaction dan_timer_irq = {
	.name		= "DAN_timer",
	.flags		= IRQF_TRIGGER_HIGH | IRQF_DISABLED | IRQF_TIMER | IRQF_IRQPOLL,
	.handler	= dan_timer_interrupt,
	.dev_id		= &dan_clockevent
};

static void __init dan_init_timer(void)
{
	/* Free running timer (for clocksource): */
	if ( !CS_TIMER.control_reg ) {
		CS_TIMER.load_count  =	0xffffffff;
		CS_TIMER.control_reg =	1;	/* Enable cs_timer in free running mode */
	}

	TIMER.control_reg =		0;	/* Disable timer */
	TIMER.control_reg =		(1<<1);	/* User-defined mode, unmasked. */
	TIMER.load_count  =		CLOCK_TICK_RATE;

	setup_irq(IRQ_TIMER, &dan_timer_irq);

	TIMER.control_reg =		3;	/* Enable timer in reload mode */

	clocksource_register_khz(&dan_clocksource, BUS_CLOCK / 1000 );

	clockevents_calc_mult_shift(&dan_clockevent, BUS_CLOCK, 5);

	dan_clockevent.max_delta_ns =
		clockevent_delta2ns(0x1fffffff, &dan_clockevent);
	dan_clockevent.min_delta_ns =
		clockevent_delta2ns(BUS_CLOCK / USEC_PER_SEC, &dan_clockevent);

	clockevents_register_device(&dan_clockevent);
}

struct sys_timer dan_timer = {
	.init		= dan_init_timer,
};

