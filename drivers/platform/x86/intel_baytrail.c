/*
 * Baytrail IOSF-SB MailBox Interface Driver
 * Copyright (c) 2013, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 *
 * The IOSF-SB is a fabric bus available on Atom based SOC's that uses a
 * mailbox interface (MBI) to communicate with mutiple devices. This
 * driver implements BayTrail-specific access to this interface.
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/pnp.h>

#include "intel_baytrail.h"

static DEFINE_SPINLOCK(iosf_mbi_lock);

static inline u32 iosf_mbi_form_mcr(u8 op, u8 port, u8 offset)
{
	return (op << 24) | (port << 16) | (offset << 8) | BT_MBI_ENABLE;
}

static struct {
	void __iomem *addr;
	bool probed;
} iosf_mbi_data;

/* Hold lock before calling */
static u32 iosf_mbi_read_mdr(u32 mcrx, u32 mcr, void __iomem *addr)
{
	if (mcrx)
		iowrite32(mcrx, addr + BT_MBI_MCRX_OFFSET);
	iowrite32(mcr, addr + BT_MBI_MCR_OFFSET);
	return ioread32(addr + BT_MBI_MDR_OFFSET);
}

/* Hold lock before calling */
static void iosf_mbi_write_mdr(u32 mcrx, u32 mcr, u32 mdr, void __iomem *addr)
{

	iowrite32(mdr, addr + BT_MBI_MDR_OFFSET);
	if (mcrx)
		iowrite32(mcrx, addr + BT_MBI_MCRX_OFFSET);
	iowrite32(mcr, addr + BT_MBI_MCR_OFFSET);
}

u32 bt_mbi_read(u8 port, u8 opcode, u32 offset)
{
	u32 mcr, mcrx;
	u32 ret;
	unsigned long flags;

	/*Access to the GFX unit is handled by GPU code */
	BUG_ON(port == BT_MBI_UNIT_GFX);

	mcr = iosf_mbi_form_mcr(opcode, port, offset & BT_MBI_MASK_LO);
	mcrx = offset & BT_MBI_MASK_HI;

	spin_lock_irqsave(&iosf_mbi_lock, flags);
	ret = iosf_mbi_read_mdr(mcrx, mcr, iosf_mbi_data.addr);
	spin_unlock_irqrestore(&iosf_mbi_lock, flags);

	return ret;
}
EXPORT_SYMBOL(bt_mbi_read);

void bt_mbi_write(u8 port, u8 opcode, u32 offset, u32 mdr)
{
	u32 mcr, mcrx;
	unsigned long flags;

	/*Access to the GFX unit is handled by GPU code */
	BUG_ON(port == BT_MBI_UNIT_GFX);

	mcr = iosf_mbi_form_mcr(opcode, port, offset & BT_MBI_MASK_LO);
	mcrx = offset & BT_MBI_MASK_HI;

	spin_lock_irqsave(&iosf_mbi_lock, flags);
	iosf_mbi_write_mdr(mcrx, mcr, mdr, iosf_mbi_data.addr);
	spin_unlock_irqrestore(&iosf_mbi_lock, flags);
}
EXPORT_SYMBOL(bt_mbi_write);

void bt_mbi_modify(u8 port, u8 opcode, u32 offset, u32 mdr, u32 mask)
{
	u32 mcr, mcrx;
	u32 value;
	unsigned long flags;

	/*Access to the GFX unit is handled by GPU code */
	BUG_ON(port == BT_MBI_UNIT_GFX);

	mcr = iosf_mbi_form_mcr(opcode, port, offset & BT_MBI_MASK_LO);
	mcrx = offset & BT_MBI_MASK_HI;

	spin_lock_irqsave(&iosf_mbi_lock, flags);

	/* Read current mdr value */
	value = iosf_mbi_read_mdr(mcrx, mcr & BT_MBI_RD_MASK,
				  iosf_mbi_data.addr);

	/* Apply mask */
	value &= ~mask;
	mdr &= mask;
	value |= mdr;

	/* Write back */
	iosf_mbi_write_mdr(mcrx, mcr | BT_MBI_WR_MASK, value,
			   iosf_mbi_data.addr);

	spin_unlock_irqrestore(&iosf_mbi_lock, flags);
}
EXPORT_SYMBOL(bt_mbi_modify);

static int iosf_mbi_pnp_probe(struct pnp_dev *pnp,
			      const struct pnp_device_id *dev_id)
{
	struct resource *mem;

	/* Get and map MBI address space */
	mem = pnp_get_resource(pnp, IORESOURCE_MEM, 0);
	if (!mem)
		return -ENOMEM;

	iosf_mbi_data.addr = devm_ioremap_resource(&pnp->dev, mem);
	if (IS_ERR(iosf_mbi_data.addr))
		return PTR_ERR(iosf_mbi_data.addr);

	iosf_mbi_data.probed = true;
	return 0;
}

static void iosf_mbi_pnp_remove(struct pnp_dev *pdev)
{
	return;
}

static const struct pnp_device_id iosf_mbi_dev_table[] = {
	{ "INT33BD", 0},
	{ "", 0},
};
MODULE_DEVICE_TABLE(pnp, iosf_mbi_dev_table);

static struct pnp_driver iosf_mbi_pnp_driver = {
	.name		= "bt_iosf_mbi",
	.probe		= iosf_mbi_pnp_probe,
	.remove		= iosf_mbi_pnp_remove,
	.id_table	= iosf_mbi_dev_table,
};

static int __init iosf_mbi_init(void)
{
	int ret;

	iosf_mbi_data.probed = false;

	ret = pnp_register_driver(&iosf_mbi_pnp_driver);
	if (!ret && !iosf_mbi_data.probed) {
		pnp_unregister_driver(&iosf_mbi_pnp_driver);
		return -ENODEV;
	}

	return ret;
}

static void __exit iosf_mbi_exit(void)
{
	pnp_unregister_driver(&iosf_mbi_pnp_driver);
}

module_init(iosf_mbi_init);
module_exit(iosf_mbi_exit);

MODULE_AUTHOR("David E. Box <david.e.box@linux.intel.com>");
MODULE_DESCRIPTION("BayTrail Mailbox Interface accessor");
MODULE_LICENSE("GPL v2");
