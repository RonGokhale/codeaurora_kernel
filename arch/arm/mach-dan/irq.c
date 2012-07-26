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
 *  linux/arch/arm/mach-dan/irq.c
 */
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/sysdev.h>

#include <mach/irqs.h>
#include <mach/timex.h>

#include "board.h"


#define MASK_IRQ(irq)		ICTL_REG(IRQ_INTMASK_L) |= (1<<irq)
#define UNMASK_IRQ(irq)		ICTL_REG(IRQ_INTMASK_L) &= ~(1<<irq)
#define ENABLE_IRQ(irq)		ICTL_REG(IRQ_INTEN_L) |= (1<<irq)
#define DISABLE_IRQ(irq)	ICTL_REG(IRQ_INTEN_L) &= ~(1<<irq)


static void dan_mask_irq(unsigned int irq)
{
	MASK_IRQ(irq);
}

void dan_unmask_irq(unsigned int irq)
{
	UNMASK_IRQ(irq);
}

static void dan_en_irq(unsigned int irq)
{
	ENABLE_IRQ(irq);
	UNMASK_IRQ(irq);
}

static void dan_dis_irq(unsigned int irq)
{
	DISABLE_IRQ(irq);
}

static struct irq_chip dan_chip = {
	.name		= "DAN_irq",
	.enable		= dan_en_irq,
	.disable	= dan_dis_irq,
	.ack		= dan_mask_irq,
	.mask		= dan_mask_irq,
	.mask_ack	= dan_mask_irq,
	.unmask 	= dan_unmask_irq,
};

static void __init dan_interrupts_init(void)
{
	IRQ_SET(TIMER);
	IRQ_SET(UART);
	IRQ_SET(SPI);
	IRQ_SET(MMAC);
	IRQ_SET(IPC);
}

void __init dan_init_irq(void)
{
	unsigned int i;

	/* Disable all interrupts sources. */
	ICTL_REG(IRQ_INTEN_L) = 0;
	ICTL_REG(IRQ_INTEN_H) = 0;

	for (i = 0; i < NR_IRQS; i++)
	{
		set_irq_chip_and_handler(i, &dan_chip, handle_level_irq);
		set_irq_flags(i, IRQF_VALID);
	}
	dan_interrupts_init();
}

#ifdef CONFIG_PM
static unsigned long ic_irq_enable;

static int irq_suspend(struct sys_device *dev, pm_message_t state)
{
	return 0;
}

static int irq_resume(struct sys_device *dev)
{
	return 0;
}
#else
#define irq_suspend NULL
#define irq_resume NULL
#endif

static struct sysdev_class irq_class = {
	.name		= "irq",
	.suspend	= irq_suspend,
	.resume		= irq_resume,
};

static struct sys_device irq_device = {
	.id	= 0,
	.cls	= &irq_class,
};

static int __init irq_init_sysfs(void)
{
	int ret = sysdev_class_register(&irq_class);
	if (ret == 0)
		ret = sysdev_register(&irq_device);
	return ret;
}

device_initcall(irq_init_sysfs);
