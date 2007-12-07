/* linux/arch/arm/mach-msm/irq.c
 *
 * Copyright (C) 2007 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/ptrace.h>
#include <linux/timer.h>

#include <linux/irq.h>
#include <asm/hardware.h>
#include <asm/irq.h>

#include <asm/io.h>

#include <asm/arch/msm_iomap.h>

#define VIC_REG(off) (MSM_VIC_BASE + (off))

#define VIC_INT_SELECT0     VIC_REG(0x0000)  /* 1: FIQ, 0: IRQ */
#define VIC_INT_SELECT1     VIC_REG(0x0004)  /* 1: FIQ, 0: IRQ */
#define VIC_INT_EN0         VIC_REG(0x0010)
#define VIC_INT_EN1         VIC_REG(0x0014)
#define VIC_INT_ENCLEAR0    VIC_REG(0x0020)
#define VIC_INT_ENCLEAR1    VIC_REG(0x0024)
#define VIC_INT_ENSET0      VIC_REG(0x0030)
#define VIC_INT_ENSET1      VIC_REG(0x0034)
#define VIC_INT_TYPE0       VIC_REG(0x0040)  /* 1: EDGE, 0: LEVEL  */
#define VIC_INT_TYPE1       VIC_REG(0x0044)  /* 1: EDGE, 0: LEVEL  */
#define VIC_INT_POLARITY0   VIC_REG(0x0050)  /* 1: NEG, 0: POS */
#define VIC_INT_POLARITY1   VIC_REG(0x0054)  /* 1: NEG, 0: POS */
#define VIC_NO_PEND_VAL     VIC_REG(0x0060)
#define VIC_INT_MASTEREN    VIC_REG(0x0064)  /* 1: IRQ, 2: FIQ     */
#define VIC_PROTECTION      VIC_REG(0x006C)  /* 1: ENABLE          */
#define VIC_CONFIG          VIC_REG(0x0068)  /* 1: USE ARM1136 VIC */
#define VIC_IRQ_STATUS0     VIC_REG(0x0080)
#define VIC_IRQ_STATUS1     VIC_REG(0x0084)
#define VIC_FIQ_STATUS0     VIC_REG(0x0090)
#define VIC_FIQ_STATUS1     VIC_REG(0x0094)
#define VIC_RAW_STATUS0     VIC_REG(0x00A0)
#define VIC_RAW_STATUS1     VIC_REG(0x00A4)
#define VIC_INT_CLEAR0      VIC_REG(0x00B0)
#define VIC_INT_CLEAR1      VIC_REG(0x00B4)
#define VIC_SOFTINT0        VIC_REG(0x00C0)
#define VIC_SOFTINT1        VIC_REG(0x00C4)
#define VIC_IRQ_VEC_RD      VIC_REG(0x00D0)  /* pending int # */
#define VIC_IRQ_VEC_PEND_RD VIC_REG(0x00D4)  /* pending vector addr */
#define VIC_IRQ_VEC_WR      VIC_REG(0x00D8)
#define VIC_IRQ_IN_SERVICE  VIC_REG(0x00E0)
#define VIC_IRQ_IN_STACK    VIC_REG(0x00E4)
#define VIC_TEST_BUS_SEL    VIC_REG(0x00E8)

#define VIC_VECTPRIORITY(n) VIC_REG(0x0200+((n) * 4))
#define VIC_VECTADDR(n)     VIC_REG(0x0400+((n) * 4))

static uint32_t msm_irq_wake_enable[2];
static struct {
	uint32_t int_en;
	uint32_t int_type;
	uint32_t int_polarity;
} msm_irq_shadow_reg[2];

static void msm_irq_ack(unsigned int irq)
{
	unsigned reg = VIC_INT_CLEAR0 + ((irq & 32) ? 4 : 0);
	irq = 1 << (irq & 31);
	writel(irq, reg);
}

static void msm_irq_mask(unsigned int irq)
{
	unsigned reg = VIC_INT_ENCLEAR0 + ((irq & 32) ? 4 : 0);
	unsigned index = (irq >> 5) & 1;
	uint32_t mask = 1UL << (irq & 31);
	msm_irq_shadow_reg[index].int_en &= ~mask;
	writel(mask, reg);
}

static void msm_irq_unmask(unsigned int irq)
{
	unsigned reg = VIC_INT_ENSET0 + ((irq & 32) ? 4 : 0);
	unsigned index = (irq >> 5) & 1;
	uint32_t mask = 1UL << (irq & 31);
	msm_irq_shadow_reg[index].int_en |= mask;
	writel(1 << (irq & 31), reg);
}

static int msm_irq_set_wake(unsigned int irq, unsigned int on)
{
	unsigned index = (irq >> 5) & 1;
	uint32_t mask = 1UL << (irq & 31);
	if (on)
		msm_irq_wake_enable[index] |= mask;
	else
		msm_irq_wake_enable[index] &= ~mask;
	return 0;
}

static int msm_irq_set_type(unsigned int irq, unsigned int flow_type)
{
	unsigned treg = VIC_INT_TYPE0 + ((irq & 32) ? 4 : 0);
	unsigned preg = VIC_INT_POLARITY0 + ((irq & 32) ? 4 : 0);
	unsigned index = (irq >> 5) & 1;
	int b = 1 << (irq & 31);
	uint32_t polarity;
	uint32_t type;

	polarity = msm_irq_shadow_reg[index].int_polarity;
	if (flow_type & (IRQF_TRIGGER_FALLING | IRQF_TRIGGER_LOW))
		polarity |= b;
	if (flow_type & (IRQF_TRIGGER_RISING | IRQF_TRIGGER_HIGH))
		polarity &= ~b;
	writel(polarity, preg);
	msm_irq_shadow_reg[index].int_polarity = polarity;

	type = msm_irq_shadow_reg[index].int_type;
	if (flow_type & (IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING)) {
		type |= b;
		irq_desc[irq].handle_irq = handle_edge_irq;
	}
	if (flow_type & (IRQF_TRIGGER_HIGH | IRQF_TRIGGER_LOW)) {
		type &= ~b;
		irq_desc[irq].handle_irq = handle_level_irq;
	}
	writel(type, treg);
	msm_irq_shadow_reg[index].int_type = type;
	return 0;
}

void msm_irq_enter_sleep(void)
{
	int limit = 10;
	msm_irq_wake_enable[0] = 1U << INT_A9_M2A_6;
	msm_irq_wake_enable[1] = 0;
	msm_irq_set_type(INT_A9_M2A_6, IRQF_TRIGGER_RISING);
	writel(~msm_irq_wake_enable[0], VIC_INT_ENCLEAR0);
	writel(~msm_irq_wake_enable[1], VIC_INT_ENCLEAR1);

	while(limit-- > 0) {
		int irq = readl(VIC_IRQ_VEC_RD);
		if(irq == -1)
			break;
		printk("msm_irq_enter_sleep cleared int %d\n", irq);
	}

	writel(msm_irq_wake_enable[0], VIC_INT_ENSET0);
	writel(msm_irq_wake_enable[1], VIC_INT_ENSET1);
}

void msm_irq_exit_sleep(void)
{
	int i;
	msm_irq_ack(INT_A9_M2A_6);
	for(i = 0; i < 2; i++) {
		writel(msm_irq_shadow_reg[i].int_type, VIC_INT_TYPE0 + i * 4);
		writel(msm_irq_shadow_reg[i].int_polarity, VIC_INT_POLARITY0 + i * 4);
		writel(msm_irq_shadow_reg[i].int_en, VIC_INT_EN0 + i * 4);
	}
	writel(1, VIC_INT_MASTEREN);
}

static struct irq_chip msm_irq_chip = {
	.name      = "msm",
	.ack       = msm_irq_ack,
	.mask      = msm_irq_mask,
	.unmask    = msm_irq_unmask,
	.set_wake  = msm_irq_set_wake,
	.set_type  = msm_irq_set_type,
};

void __init msm_init_irq(void)
{
	unsigned n;

	/* select level interrupts */
	writel(0, VIC_INT_TYPE0);
	writel(0, VIC_INT_TYPE1);

	/* select highlevel interrupts */
	writel(0, VIC_INT_POLARITY0);
	writel(0, VIC_INT_POLARITY1);

	/* select IRQ for all INTs */
	writel(0, VIC_INT_SELECT0);
	writel(0, VIC_INT_SELECT1);

	/* disable all INTs */
	writel(0, VIC_INT_EN0);
	writel(0, VIC_INT_EN1);

	/* don't use 1136 vic */
	writel(0, VIC_CONFIG);

	/* enable interrupt controller */
	writel(1, VIC_INT_MASTEREN);

	for (n = 0; n < NR_MSM_IRQS; n++) {
		set_irq_chip(n, &msm_irq_chip);
		set_irq_handler(n, handle_level_irq);
		set_irq_flags(n, IRQF_VALID);
	}
}
