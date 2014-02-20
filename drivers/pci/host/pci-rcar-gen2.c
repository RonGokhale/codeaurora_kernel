/*
 *  pci-rcar-gen2: internal PCI bus support
 *
 * Copyright (C) 2013 Renesas Solutions Corp.
 * Copyright (C) 2013 Cogent Embedded, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/delay.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/genalloc.h>

/* AHB-PCI Bridge PCI communication registers */
#define RCAR_AHBPCI_PCICOM_OFFSET	0x800

#define RCAR_PCIAHB_WIN1_CTR_REG	(RCAR_AHBPCI_PCICOM_OFFSET + 0x00)
#define RCAR_PCIAHB_WIN2_CTR_REG	(RCAR_AHBPCI_PCICOM_OFFSET + 0x04)
#define RCAR_PCIAHB_PREFETCH0		0x0
#define RCAR_PCIAHB_PREFETCH4		0x1
#define RCAR_PCIAHB_PREFETCH8		0x2
#define RCAR_PCIAHB_PREFETCH16		0x3

#define RCAR_AHBPCI_WIN1_CTR_REG	(RCAR_AHBPCI_PCICOM_OFFSET + 0x10)
#define RCAR_AHBPCI_WIN2_CTR_REG	(RCAR_AHBPCI_PCICOM_OFFSET + 0x14)
#define RCAR_AHBPCI_WIN_CTR_MEM		(3 << 1)
#define RCAR_AHBPCI_WIN_CTR_CFG		(5 << 1)
#define RCAR_AHBPCI_WIN1_HOST		(1 << 30)
#define RCAR_AHBPCI_WIN1_DEVICE		(1 << 31)

#define RCAR_PCI_INT_ENABLE_REG		(RCAR_AHBPCI_PCICOM_OFFSET + 0x20)
#define RCAR_PCI_INT_STATUS_REG		(RCAR_AHBPCI_PCICOM_OFFSET + 0x24)
#define RCAR_PCI_INT_SIGTABORT		(1 << 0)
#define RCAR_PCI_INT_SIGRETABORT	(1 << 1)
#define RCAR_PCI_INT_REMABORT		(1 << 2)
#define RCAR_PCI_INT_PERR		(1 << 3)
#define RCAR_PCI_INT_SIGSERR		(1 << 4)
#define RCAR_PCI_INT_RESERR		(1 << 5)
#define RCAR_PCI_INT_WIN1ERR		(1 << 12)
#define RCAR_PCI_INT_WIN2ERR		(1 << 13)
#define RCAR_PCI_INT_A			(1 << 16)
#define RCAR_PCI_INT_B			(1 << 17)
#define RCAR_PCI_INT_PME		(1 << 19)
#define RCAR_PCI_INT_ALLERRORS (RCAR_PCI_INT_SIGTABORT		| \
				RCAR_PCI_INT_SIGRETABORT	| \
				RCAR_PCI_INT_SIGRETABORT	| \
				RCAR_PCI_INT_REMABORT		| \
				RCAR_PCI_INT_PERR		| \
				RCAR_PCI_INT_SIGSERR		| \
				RCAR_PCI_INT_RESERR		| \
				RCAR_PCI_INT_WIN1ERR		| \
				RCAR_PCI_INT_WIN2ERR)

#define RCAR_AHB_BUS_CTR_REG		(RCAR_AHBPCI_PCICOM_OFFSET + 0x30)
#define RCAR_AHB_BUS_MMODE_HTRANS	(1 << 0)
#define RCAR_AHB_BUS_MMODE_BYTE_BURST	(1 << 1)
#define RCAR_AHB_BUS_MMODE_WR_INCR	(1 << 2)
#define RCAR_AHB_BUS_MMODE_HBUS_REQ	(1 << 7)
#define RCAR_AHB_BUS_SMODE_READYCTR	(1 << 17)
#define RCAR_AHB_BUS_MODE		(RCAR_AHB_BUS_MMODE_HTRANS |	\
					RCAR_AHB_BUS_MMODE_BYTE_BURST |	\
					RCAR_AHB_BUS_MMODE_WR_INCR |	\
					RCAR_AHB_BUS_MMODE_HBUS_REQ |	\
					RCAR_AHB_BUS_SMODE_READYCTR)

#define RCAR_USBCTR_REG			(RCAR_AHBPCI_PCICOM_OFFSET + 0x34)
#define RCAR_USBCTR_USBH_RST		(1 << 0)
#define RCAR_USBCTR_PCICLK_MASK		(1 << 1)
#define RCAR_USBCTR_PLL_RST		(1 << 2)
#define RCAR_USBCTR_DIRPD		(1 << 8)
#define RCAR_USBCTR_PCIAHB_WIN2_EN	(1 << 9)
#define RCAR_USBCTR_PCIAHB_WIN1_256M	(0 << 10)
#define RCAR_USBCTR_PCIAHB_WIN1_512M	(1 << 10)
#define RCAR_USBCTR_PCIAHB_WIN1_1G	(2 << 10)
#define RCAR_USBCTR_PCIAHB_WIN1_2G	(3 << 10)
#define RCAR_USBCTR_PCIAHB_WIN1_MASK	(3 << 10)

#define RCAR_PCI_ARBITER_CTR_REG	(RCAR_AHBPCI_PCICOM_OFFSET + 0x40)
#define RCAR_PCI_ARBITER_PCIREQ0	(1 << 0)
#define RCAR_PCI_ARBITER_PCIREQ1	(1 << 1)
#define RCAR_PCI_ARBITER_PCIBP_MODE	(1 << 12)

#define RCAR_PCI_UNIT_REV_REG		(RCAR_AHBPCI_PCICOM_OFFSET + 0x48)

struct rcar_pci_priv {
	struct device *dev;
	void __iomem *reg;
	struct resource io_res;
	struct resource mem_res;
	struct resource *cfg_res;
	int irq;
	unsigned long window_size;
	u32 window_base;
	struct notifier_block bus_notifier;
	struct gen_pool *dma_pool;
	unsigned long dma_size;
	dma_addr_t dma_phys;
};

/* PCI configuration space operations */
static void __iomem *rcar_pci_cfg_base(struct pci_bus *bus, unsigned int devfn,
				       int where)
{
	struct pci_sys_data *sys = bus->sysdata;
	struct rcar_pci_priv *priv = sys->private_data;
	int slot, val;

	if (sys->busnr != bus->number || PCI_FUNC(devfn))
		return NULL;

	/* Only one EHCI/OHCI device built-in */
	slot = PCI_SLOT(devfn);
	if (slot > 2)
		return NULL;

	/* bridge logic only has registers to 0x40 */
	if (slot == 0x0 && where >= 0x40)
		return NULL;

	val = slot ? RCAR_AHBPCI_WIN1_DEVICE | RCAR_AHBPCI_WIN_CTR_CFG :
		     RCAR_AHBPCI_WIN1_HOST | RCAR_AHBPCI_WIN_CTR_CFG;

	iowrite32(val, priv->reg + RCAR_AHBPCI_WIN1_CTR_REG);
	return priv->reg + (slot >> 1) * 0x100 + where;
}

static int rcar_pci_read_config(struct pci_bus *bus, unsigned int devfn,
				int where, int size, u32 *val)
{
	void __iomem *reg = rcar_pci_cfg_base(bus, devfn, where);

	if (!reg)
		return PCIBIOS_DEVICE_NOT_FOUND;

	switch (size) {
	case 1:
		*val = ioread8(reg);
		break;
	case 2:
		*val = ioread16(reg);
		break;
	default:
		*val = ioread32(reg);
		break;
	}

	return PCIBIOS_SUCCESSFUL;
}

static int rcar_pci_write_config(struct pci_bus *bus, unsigned int devfn,
				 int where, int size, u32 val)
{
	void __iomem *reg = rcar_pci_cfg_base(bus, devfn, where);

	if (!reg)
		return PCIBIOS_DEVICE_NOT_FOUND;

	switch (size) {
	case 1:
		iowrite8(val, reg);
		break;
	case 2:
		iowrite16(val, reg);
		break;
	default:
		iowrite32(val, reg);
		break;
	}

	return PCIBIOS_SUCCESSFUL;
}

/* PCI interrupt mapping */
static int rcar_pci_map_irq(const struct pci_dev *dev, u8 slot, u8 pin)
{
	struct pci_sys_data *sys = dev->bus->sysdata;
	struct rcar_pci_priv *priv = sys->private_data;

	return priv->irq;
}

#ifdef CONFIG_PCI_DEBUG
/* if debug enabled, then attach an error handler irq to the bridge */

static irqreturn_t rcar_pci_err_irq(int irq, void *pw)
{
	struct rcar_pci_priv *priv = pw;
	u32 status = ioread32(priv->reg + RCAR_PCI_INT_STATUS_REG);

	if (status & RCAR_PCI_INT_ALLERRORS) {
		dev_err(priv->dev, "error irq: status %08x\n", status);

		/* clear the error(s) */
		iowrite32(status & RCAR_PCI_INT_ALLERRORS,
			  priv->reg + RCAR_PCI_INT_STATUS_REG);
		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}

static void rcar_pci_setup_errirq(struct rcar_pci_priv *priv)
{
	int ret;
	u32 val;

	ret = devm_request_irq(priv->dev, priv->irq, rcar_pci_err_irq,
			       IRQF_SHARED, "error irq", priv);
	if (ret) {
		dev_err(priv->dev, "cannot claim IRQ for error handling\n");
		return;
	}

	val = ioread32(priv->reg + RCAR_PCI_INT_ENABLE_REG);
	val |= RCAR_PCI_INT_ALLERRORS;
	iowrite32(val, priv->reg + RCAR_PCI_INT_ENABLE_REG);
}
#else
static inline void rcar_pci_setup_errirq(struct rcar_pci_priv *priv) { }
#endif

/* PCI host controller setup */
static int rcar_pci_setup(int nr, struct pci_sys_data *sys)
{
	struct rcar_pci_priv *priv = sys->private_data;
	void __iomem *reg = priv->reg;
	u32 val;

	pm_runtime_enable(priv->dev);
	pm_runtime_get_sync(priv->dev);

	val = ioread32(reg + RCAR_PCI_UNIT_REV_REG);
	dev_info(priv->dev, "PCI: bus%u revision %x\n", sys->busnr, val);

	/* Disable Direct Power Down State and assert reset */
	val = ioread32(reg + RCAR_USBCTR_REG) & ~RCAR_USBCTR_DIRPD;
	val |= RCAR_USBCTR_USBH_RST | RCAR_USBCTR_PLL_RST;
	iowrite32(val, reg + RCAR_USBCTR_REG);
	udelay(4);

	/* De-assert reset and reset PCIAHB window1 size */
	val &= ~(RCAR_USBCTR_PCIAHB_WIN1_MASK | RCAR_USBCTR_PCICLK_MASK |
		 RCAR_USBCTR_USBH_RST | RCAR_USBCTR_PLL_RST);

	/* Setup PCIAHB window1 size */
	switch (priv->window_size) {
	case SZ_2G:
		val |= RCAR_USBCTR_PCIAHB_WIN1_2G;
		break;
	case SZ_1G:
		val |= RCAR_USBCTR_PCIAHB_WIN1_1G;
		break;
	case SZ_512M:
		val |= RCAR_USBCTR_PCIAHB_WIN1_512M;
		break;
	default:
		pr_warn("unknown window size %ld - defaulting to 256M\n",
			priv->window_size);
		priv->window_size = SZ_256M;
		/* fall-through */
	case SZ_256M:
		val |= RCAR_USBCTR_PCIAHB_WIN1_256M;
		break;
	}
	iowrite32(val, reg + RCAR_USBCTR_REG);

	/* Configure AHB master and slave modes */
	iowrite32(RCAR_AHB_BUS_MODE, reg + RCAR_AHB_BUS_CTR_REG);

	/* Configure PCI arbiter */
	val = ioread32(reg + RCAR_PCI_ARBITER_CTR_REG);
	val |= RCAR_PCI_ARBITER_PCIREQ0 | RCAR_PCI_ARBITER_PCIREQ1 |
	       RCAR_PCI_ARBITER_PCIBP_MODE;
	iowrite32(val, reg + RCAR_PCI_ARBITER_CTR_REG);

	/* PCI-AHB mapping: dynamic base */
	iowrite32(priv->window_base | RCAR_PCIAHB_PREFETCH16,
		  reg + RCAR_PCIAHB_WIN1_CTR_REG);

	/* AHB-PCI mapping: OHCI/EHCI registers */
	val = priv->mem_res.start | RCAR_AHBPCI_WIN_CTR_MEM;
	iowrite32(val, reg + RCAR_AHBPCI_WIN2_CTR_REG);

	/* Enable AHB-PCI bridge PCI configuration access */
	iowrite32(RCAR_AHBPCI_WIN1_HOST | RCAR_AHBPCI_WIN_CTR_CFG,
		  reg + RCAR_AHBPCI_WIN1_CTR_REG);
	/* Set PCI-AHB Window1 address */
	iowrite32(priv->window_base | PCI_BASE_ADDRESS_MEM_PREFETCH,
		  reg + PCI_BASE_ADDRESS_1);
	/* Set AHB-PCI bridge PCI communication area address */
	val = priv->cfg_res->start + RCAR_AHBPCI_PCICOM_OFFSET;
	iowrite32(val, reg + PCI_BASE_ADDRESS_0);

	val = ioread32(reg + PCI_COMMAND);
	val |= PCI_COMMAND_SERR | PCI_COMMAND_PARITY |
	       PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER;
	iowrite32(val, reg + PCI_COMMAND);

	/* Enable PCI interrupts */
	iowrite32(RCAR_PCI_INT_A | RCAR_PCI_INT_B | RCAR_PCI_INT_PME,
		  reg + RCAR_PCI_INT_ENABLE_REG);

	if (priv->irq > 0)
		rcar_pci_setup_errirq(priv);

	/* Add PCI resources */
	pci_add_resource(&sys->resources, &priv->io_res);
	pci_add_resource(&sys->resources, &priv->mem_res);

	/* Setup bus number based on platform device id */
	sys->busnr = to_platform_device(priv->dev)->id;
	return 1;
}

static struct pci_ops rcar_pci_ops = {
	.read	= rcar_pci_read_config,
	.write	= rcar_pci_write_config,
};

static struct rcar_pci_priv *rcar_pci_dev_to_priv(struct device *dev)
{
	struct pci_sys_data *sys = to_pci_dev(dev)->sysdata;

	return sys->private_data;
}

static void *rcar_pci_dma_alloc(struct device *dev, size_t size,
				dma_addr_t *handle, gfp_t gfp,
				struct dma_attrs *attrs)
{
	struct rcar_pci_priv *p = rcar_pci_dev_to_priv(dev);

	return (void *)gen_pool_dma_alloc(p->dma_pool, size, handle);
}

static void rcar_pci_dma_free(struct device *dev, size_t size, void *cpu_addr,
			      dma_addr_t handle, struct dma_attrs *attrs)
{
	struct rcar_pci_priv *p = rcar_pci_dev_to_priv(dev);

	gen_pool_free(p->dma_pool, (unsigned long)cpu_addr, size);
}

static struct dma_map_ops rcar_pci_dma_ops_init = {
	.alloc = rcar_pci_dma_alloc,
	.free = rcar_pci_dma_free,
};

static int rcar_pci_dma_mmap(struct device *dev, struct vm_area_struct *vma,
			     void *cpu_addr, dma_addr_t dma_addr, size_t size,
			     struct dma_attrs *attrs)
{
	struct rcar_pci_priv *p = rcar_pci_dev_to_priv(dev);

	return get_dma_ops(p->dev)->mmap(p->dev, vma, cpu_addr,
					 dma_addr, size, attrs);
}

static int rcar_pci_dma_get_sgtable(struct device *dev, struct sg_table *sgt,
				    void *cpu_addr, dma_addr_t handle,
				    size_t size, struct dma_attrs *attrs)
{
	struct rcar_pci_priv *p = rcar_pci_dev_to_priv(dev);

	return get_dma_ops(p->dev)->get_sgtable(p->dev, sgt, cpu_addr,
						handle, size, attrs);
}

static dma_addr_t rcar_pci_dma_map_page(struct device *dev, struct page *page,
					unsigned long offset, size_t size,
					enum dma_data_direction dir,
					struct dma_attrs *attrs)
{
	struct rcar_pci_priv *p = rcar_pci_dev_to_priv(dev);

	return get_dma_ops(p->dev)->map_page(p->dev, page, offset,
					     size, dir, attrs);
}

static void rcar_pci_dma_unmap_page(struct device *dev, dma_addr_t dma_handle,
				    size_t size, enum dma_data_direction dir,
				    struct dma_attrs *attrs)
{
	struct rcar_pci_priv *p = rcar_pci_dev_to_priv(dev);

	get_dma_ops(p->dev)->unmap_page(p->dev, dma_handle, size, dir, attrs);
}

static int rcar_pci_dma_map_sg(struct device *dev, struct scatterlist *sg,
			       int nents, enum dma_data_direction dir,
			       struct dma_attrs *attrs)
{
	struct rcar_pci_priv *p = rcar_pci_dev_to_priv(dev);

	return get_dma_ops(p->dev)->map_sg(p->dev, sg, nents, dir, attrs);
}

static void rcar_pci_dma_unmap_sg(struct device *dev, struct scatterlist *sg,
				  int nents, enum dma_data_direction dir,
				  struct dma_attrs *attrs)
{
	struct rcar_pci_priv *p = rcar_pci_dev_to_priv(dev);

	get_dma_ops(p->dev)->unmap_sg(p->dev, sg, nents, dir, attrs);
}

static void rcar_pci_dma_sync_single_for_cpu(struct device *dev,
					     dma_addr_t dma_handle,
					     size_t size,
					     enum dma_data_direction dir)
{
	struct rcar_pci_priv *p = rcar_pci_dev_to_priv(dev);

	get_dma_ops(p->dev)->sync_single_for_cpu(p->dev, dma_handle,
						 size, dir);
}

static void rcar_pci_dma_sync_single_for_device(struct device *dev,
						dma_addr_t dma_handle,
						size_t size,
						enum dma_data_direction dir)
{
	struct rcar_pci_priv *p = rcar_pci_dev_to_priv(dev);

	get_dma_ops(p->dev)->sync_single_for_device(p->dev, dma_handle,
						    size, dir);
}

static void rcar_pci_dma_sync_sg_for_cpu(struct device *dev,
					 struct scatterlist *sg,
					 int nents,
					 enum dma_data_direction dir)
{
	struct rcar_pci_priv *p = rcar_pci_dev_to_priv(dev);

	get_dma_ops(p->dev)->sync_sg_for_cpu(p->dev, sg, nents, dir);
}

static void rcar_pci_dma_sync_sg_for_device(struct device *dev,
					    struct scatterlist *sg,
					    int nents,
					    enum dma_data_direction dir)
{
	struct rcar_pci_priv *p = rcar_pci_dev_to_priv(dev);

	get_dma_ops(p->dev)->sync_sg_for_device(p->dev, sg, nents, dir);
}

static int rcar_pci_dma_mapping_error(struct device *dev, dma_addr_t dma_addr)
{
	struct rcar_pci_priv *p = rcar_pci_dev_to_priv(dev);

	return get_dma_ops(p->dev)->mapping_error(p->dev, dma_addr);
}

static int rcar_pci_dma_supported(struct device *dev, u64 mask)
{
	struct rcar_pci_priv *p = rcar_pci_dev_to_priv(dev);

	return get_dma_ops(p->dev)->dma_supported(p->dev, mask);
}

static int rcar_pci_dma_set_dma_mask(struct device *dev, u64 mask)
{
	struct rcar_pci_priv *p = rcar_pci_dev_to_priv(dev);

	return get_dma_ops(p->dev)->set_dma_mask(p->dev, mask);
}

static struct dma_map_ops rcar_pci_dma_ops = {
	.alloc = rcar_pci_dma_alloc,
	.free = rcar_pci_dma_free,
	.mmap = rcar_pci_dma_mmap,
	.get_sgtable = rcar_pci_dma_get_sgtable,
	.map_page = rcar_pci_dma_map_page,
	.unmap_page = rcar_pci_dma_unmap_page,
	.map_sg = rcar_pci_dma_map_sg,
	.unmap_sg = rcar_pci_dma_unmap_sg,
	.sync_single_for_cpu = rcar_pci_dma_sync_single_for_cpu,
	.sync_single_for_device = rcar_pci_dma_sync_single_for_device,
	.sync_sg_for_cpu = rcar_pci_dma_sync_sg_for_cpu,
	.sync_sg_for_device = rcar_pci_dma_sync_sg_for_device,
	.mapping_error = rcar_pci_dma_mapping_error,
	.dma_supported = rcar_pci_dma_supported,
	.set_dma_mask = rcar_pci_dma_set_dma_mask,
};

static int rcar_pci_bus_notify(struct notifier_block *nb,
			       unsigned long action, void *data)
{
	struct rcar_pci_priv *priv;
	struct device *dev = data;
	struct pci_dev *pci_dev = to_pci_dev(dev);
	struct pci_sys_data *sys = pci_dev->sysdata;

	priv = container_of(nb, struct rcar_pci_priv, bus_notifier);
	if (priv != sys->private_data)
		return 0;

	if (action == BUS_NOTIFY_BIND_DRIVER)
		set_dma_ops(dev, &rcar_pci_dma_ops);

	return 0;
}

static void rcar_pci_add_bus(struct pci_bus *bus)
{
	struct pci_sys_data *sys = bus->sysdata;
	struct rcar_pci_priv *priv = sys->private_data;

	bus_register_notifier(&pci_bus_type, &priv->bus_notifier);
}

static void rcar_pci_remove_bus(struct pci_bus *bus)
{
	struct pci_sys_data *sys = bus->sysdata;
	struct rcar_pci_priv *priv = sys->private_data;

	bus_unregister_notifier(&pci_bus_type, &priv->bus_notifier);
}

static int rcar_pci_needs_bounce(struct device *dev,
				 dma_addr_t dma_addr, size_t size)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rcar_pci_priv *priv = platform_get_drvdata(pdev);

	if (dma_addr < priv->window_base)
		return 1;

	if (dma_addr >= (priv->window_base + priv->window_size))
		return 1;

	return 0;
}

static void rcar_pci_init_bounce(struct rcar_pci_priv *priv)
{
	/* use notifier to hook pool allocator */
	priv->bus_notifier.notifier_call = rcar_pci_bus_notify;

	/* setup alloc()/free() to let dmabounce allocate from our pool */
	set_dma_ops(priv->dev, &rcar_pci_dma_ops_init);

	/* shared dmabounce for the PCI bridge */
	dmabounce_register_dev(priv->dev, 2048, 4096, rcar_pci_needs_bounce);
}

static int rcar_pci_probe(struct platform_device *pdev)
{
	struct resource *cfg_res, *mem_res;
	struct rcar_pci_priv *priv;
	void __iomem *reg;
	struct hw_pci hw;
	void *hw_private[1];
	void *dma_virt;
	int ret;

	cfg_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	reg = devm_ioremap_resource(&pdev->dev, cfg_res);
	if (IS_ERR(reg))
		return PTR_ERR(reg);

	mem_res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!mem_res || !mem_res->start)
		return -ENODEV;

	priv = devm_kzalloc(&pdev->dev,
			    sizeof(struct rcar_pci_priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->mem_res = *mem_res;
	/*
	 * The controller does not support/use port I/O,
	 * so setup a dummy port I/O region here.
	 */
	priv->io_res.start = priv->mem_res.start;
	priv->io_res.end = priv->mem_res.end;
	priv->io_res.flags = IORESOURCE_IO;

	priv->cfg_res = cfg_res;

	priv->irq = platform_get_irq(pdev, 0);
	priv->reg = reg;
	priv->dev = &pdev->dev;
	platform_set_drvdata(pdev, priv);

	if (priv->irq < 0) {
		dev_err(&pdev->dev, "no valid irq found\n");
		return priv->irq;
	}

	priv->window_size = SZ_1G;
	priv->dma_size = SZ_4M;

	/* allocate pool of memory guaranteed to be inside window */
	priv->dma_pool = devm_gen_pool_create(&pdev->dev, 7, -1);
	if (!priv->dma_pool)
		return -ENOMEM;

	dma_virt = dmam_alloc_coherent(&pdev->dev, priv->dma_size,
				       &priv->dma_phys, GFP_KERNEL);
	if (!dma_virt)
		return -ENOMEM;

	ret = gen_pool_add_virt(priv->dma_pool,	(unsigned long)dma_virt,
				priv->dma_phys, priv->dma_size, -1);
	if (ret)
		return ret;

	/* select window base address based on physical address for memory */
	priv->window_base = priv->dma_phys & ~(priv->window_size - 1);

	rcar_pci_init_bounce(priv);

	hw_private[0] = priv;
	memset(&hw, 0, sizeof(hw));
	hw.nr_controllers = ARRAY_SIZE(hw_private);
	hw.private_data = hw_private;
	hw.map_irq = rcar_pci_map_irq;
	hw.ops = &rcar_pci_ops;
	hw.setup = rcar_pci_setup;
	hw.add_bus = rcar_pci_add_bus;
	hw.remove_bus = rcar_pci_remove_bus;
	pci_common_init_dev(&pdev->dev, &hw);
	return 0;
}

static struct platform_driver rcar_pci_driver = {
	.driver = {
		.name = "pci-rcar-gen2",
		.owner = THIS_MODULE,
		.suppress_bind_attrs = true,
	},
	.probe = rcar_pci_probe,
};

module_platform_driver(rcar_pci_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Renesas R-Car Gen2 internal PCI");
MODULE_AUTHOR("Valentine Barshak <valentine.barshak@cogentembedded.com>");
