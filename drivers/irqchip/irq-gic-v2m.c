/*
 * ARM GIC v2m MSI(-X) support
 * Support for Message Signalelled Interrupts for systems that
 * implement ARM Generic Interrupt Controller: GICv2m.
 *
 * Copyright (C) 2014 Advanced Micro Devices, Inc.
 * Authors: Suravee Suthikulpanit <suravee.suthikulpanit@amd.com>
 *          Harish Kasiviswanathan <harish.kasiviswanathan@amd.com>
 *          Brandon Anderson <brandon.anderson@amd.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pci.h>
#include <linux/irq.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <linux/of_address.h>
#include <linux/of_pci.h>
#include <linux/bitmap.h>

#include "irqchip.h"
#include "irq-gic.h"

/*
* MSI_TYPER:
*     [31:26] Reserved
*     [25:16] lowest SPI assigned to MSI
*     [15:10] Reserved
*     [9:0]   Numer of SPIs assigned to MSI
*/
#define V2M_MSI_TYPER			0x008
#define V2M_MSI_TYPER_BASE_SHIFT	(16)
#define V2M_MSI_TYPER_BASE_MASK		(0x3FF)
#define V2M_MSI_TYPER_NUM_MASK		(0x3FF)
#define V2M_MSI_SETSPI_NS		0x040
#define V2M_MIN_SPI			32
#define V2M_MAX_SPI			1019

#define GIC_OF_MSIV2M_RANGE_INDEX	4

/*
 * alloc_msi_irq - Allocate MSIs from avaialbe MSI bitmap.
 * @data: Pointer to v2m_data
 * @nvec: Number of interrupts to allocate
 * @irq: Pointer to the allocated irq
 *
 * Allocates interrupts only if the contiguous range of MSIs
 * with specified nvec are available. Otherwise return the number
 * of available interrupts. If none are available, then returns -ENOENT.
 */
static int alloc_msi_irq(struct v2m_data *data, int nvec, int *irq)
{
	int size = data->nr_spis;
	int next = size, i = nvec, ret;

	/* We should never allocate more than available nr_spis */
	if (i >= size)
		i = size;

	spin_lock(&data->msi_cnt_lock);

	for (; i > 0; i--) {
		next = bitmap_find_next_zero_area(data->bm,
					size, 0, i, 0);
		if (next < size)
			break;
	}

	if (i != nvec) {
		ret = i ? : -ENOENT;
	} else {
		bitmap_set(data->bm, next, nvec);
		*irq = data->spi_start + next;
		ret = 0;
	}

	spin_unlock(&data->msi_cnt_lock);

	return ret;
}

static struct v2m_data *to_v2m_data(struct msi_chip *chip)
{
	struct gic_chip_data *gic = container_of(chip, struct gic_chip_data,
						 msi_chip);
	return &gic->v2m_data;
}

static void gicv2m_teardown_msi_irq(struct msi_chip *chip, unsigned int irq)
{
	int pos;
	struct v2m_data *data = to_v2m_data(chip);

	spin_lock(&data->msi_cnt_lock);

	pos = irq - data->spi_start;
	if (pos >= 0 && pos < data->nr_spis)
		bitmap_clear(data->bm, pos, 1);

	spin_unlock(&data->msi_cnt_lock);
}

static int gicv2m_setup_msi_irq(struct msi_chip *chip, struct pci_dev *pdev,
		      struct msi_desc *desc)
{
	int avail, irq = 0;
	struct msi_msg msg;
	phys_addr_t addr;
	struct v2m_data *data = to_v2m_data(chip);

	if (!desc) {
		dev_err(&pdev->dev,
			"GICv2m: MSI setup failed. Invalid msi descriptor\n");
		return -EINVAL;
	}

	avail = alloc_msi_irq(data, 1, &irq);
	if (avail != 0) {
		dev_err(&pdev->dev,
			"GICv2m: MSI setup failed. Cannnot allocate IRQ\n");
		return -ENOSPC;
	}

	irq_set_chip_data(irq, chip);
	irq_set_msi_desc(irq, desc);
	irq_set_irq_type(irq, IRQ_TYPE_EDGE_RISING);

	addr = data->res.start + V2M_MSI_SETSPI_NS;

	msg.address_hi = (u32)(addr >> 32);
	msg.address_lo = (u32)(addr);
	msg.data = irq;
	write_msi_msg(irq, &msg);

	return 0;
}

static int __init
gicv2m_msi_init(struct device_node *node, struct v2m_data *v2m)
{
	unsigned int val;

	if (of_address_to_resource(node, GIC_OF_MSIV2M_RANGE_INDEX,
				   &v2m->res)) {
		pr_err("GICv2m: Failed locate GICv2m MSI register frame\n");
		return -EINVAL;
	}

	v2m->base = of_iomap(node, GIC_OF_MSIV2M_RANGE_INDEX);
	if (!v2m->base) {
		pr_err("GICv2m: Failed to map GIC MSI registers\n");
		return -EINVAL;
	}

	val = readl_relaxed(v2m->base + V2M_MSI_TYPER);
	if (!val) {
		pr_warn("GICv2m: Failed to read V2M_MSI_TYPER register\n");
		return -EINVAL;
	}

	v2m->spi_start = (val >> V2M_MSI_TYPER_BASE_SHIFT) &
				V2M_MSI_TYPER_BASE_MASK;
	v2m->nr_spis = val & V2M_MSI_TYPER_NUM_MASK;
	if ((v2m->spi_start < V2M_MIN_SPI) || (v2m->nr_spis >= V2M_MAX_SPI)) {
			pr_err("GICv2m: Invalid MSI_TYPER (%#x)\n", val);
			return -EINVAL;
	}

	v2m->bm = kzalloc(sizeof(long) * BITS_TO_LONGS(v2m->nr_spis),
			  GFP_KERNEL);
	if (!v2m->bm) {
		pr_err("GICv2m: Failed to allocate MSI bitmap\n");
		return -ENOMEM;
	}

	spin_lock_init(&v2m->msi_cnt_lock);

	pr_info("GICv2m: SPI range [%d:%d]\n",
		v2m->spi_start, (v2m->spi_start + v2m->nr_spis));

	return 0;
}

static void gicv2m_mask_irq(struct irq_data *d)
{
	gic_mask_irq(d);
	if (d->msi_desc)
		mask_msi_irq(d);
}

static void gicv2m_unmask_irq(struct irq_data *d)
{
	gic_unmask_irq(d);
	if (d->msi_desc)
		unmask_msi_irq(d);
}

static struct irq_chip gicv2m_chip = {
	.name			= "GICv2m",
	.irq_mask		= gicv2m_mask_irq,
	.irq_unmask		= gicv2m_unmask_irq,
	.irq_eoi		= gic_eoi_irq,
	.irq_set_type		= gic_set_type,
	.irq_retrigger		= gic_retrigger,
#ifdef CONFIG_SMP
	.irq_set_affinity	= gic_set_affinity,
#endif
#ifdef CONFIG_PM
	.irq_set_wake		= gic_set_wake,
#endif
};

#ifdef CONFIG_OF
static int __init
gicv2m_of_init(struct device_node *node, struct device_node *parent)
{
	struct gic_chip_data *gic;
	int ret;

	ret = _gic_of_init(node, parent, &gicv2m_chip, &gic);
	if (ret) {
		pr_err("GICv2m: Failed to initialize GIC\n");
		return ret;
	}

	gic->msi_chip.owner = THIS_MODULE;
	gic->msi_chip.of_node = node;
	gic->msi_chip.setup_irq = gicv2m_setup_msi_irq;
	gic->msi_chip.teardown_irq = gicv2m_teardown_msi_irq;
	ret = of_pci_msi_chip_add(&gic->msi_chip);
	if (ret) {
		/*
		* Note: msi-controller is checked in of_pci_msi_chip_add().
		* MSI support is optional, and enabled only if msi-controller
		* is specified. Hence, return 0.
		*/
		return 0;
	}

	return gicv2m_msi_init(node, &gic->v2m_data);
}

IRQCHIP_DECLARE(arm_gic_400_v2m, "arm,gic-400-v2m", gicv2m_of_init);

#endif /* CONFIG_OF */
