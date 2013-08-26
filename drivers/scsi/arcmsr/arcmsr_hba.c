/*
*******************************************************************************
**        O.S   : Linux
**   FILE NAME  : arcmsr_hba.c
**        BY    : Nick Cheng, C.L. Huang
**   Description: SCSI RAID Device Driver for
**                ARECA RAID Host adapter
*******************************************************************************
** Copyright (C) 2002 - 2005, Areca Technology Corporation All rights reserved
**
**     Web site: www.areca.com.tw
**       E-mail: support@areca.com.tw
**
** This program is free software; you can redistribute it and/or modify
** it under the terms of the GNU General Public License version 2 as
** published by the Free Software Foundation.
** This program is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU General Public License for more details.
*******************************************************************************
** Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions
** are met:
** 1. Redistributions of source code must retain the above copyright
**    notice, this list of conditions and the following disclaimer.
** 2. Redistributions in binary form must reproduce the above copyright
**    notice, this list of conditions and the following disclaimer in the
**    documentation and/or other materials provided with the distribution.
** 3. The name of the author may not be used to endorse or promote products
**    derived from this software without specific prior written permission.
**
** THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
** IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
** OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
** IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
** INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES(INCLUDING,BUT
** NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
** DATA, OR PROFITS; OR BUSINESS INTERRUPTION)HOWEVER CAUSED AND ON ANY
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE)ARISING IN ANY WAY OUT OF THE USE OF
** THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************
** For history of changes, see Documentation/scsi/ChangeLog.arcmsr
**     Firmware Specification, see Documentation/scsi/arcmsr_spec.txt
*******************************************************************************
*/
#include <linux/module.h>
#include <linux/reboot.h>
#include <linux/spinlock.h>
#include <linux/pci_ids.h>
#include <linux/interrupt.h>
#include <linux/moduleparam.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/timer.h>
#include <linux/slab.h>
#include <linux/pci.h>
#include <linux/aer.h>
#include <asm/dma.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <scsi/scsi_host.h>
#include <scsi/scsi.h>
#include <scsi/scsi_cmnd.h>
#include <scsi/scsi_tcq.h>
#include <scsi/scsi_device.h>
#include <scsi/scsi_transport.h>
#include <scsi/scsicam.h>
#include "arcmsr.h"
MODULE_AUTHOR("Nick Cheng <support@areca.com.tw>");
MODULE_DESCRIPTION("Areca SAS,SATA RAID Controller Driver");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_VERSION(ARCMSR_DRIVER_VERSION);

#define ARCMSR_SLEEPTIME	10
#define ARCMSR_RETRYCOUNT	12

wait_queue_head_t wait_q;
static int arcmsr_iop_message_xfer(struct AdapterControlBlock *acb,
					struct scsi_cmnd *cmd);
static int arcmsr_iop_confirm(struct AdapterControlBlock *acb);
static int arcmsr_abort(struct scsi_cmnd *);
static int arcmsr_bus_reset(struct scsi_cmnd *);
static int arcmsr_bios_param(struct scsi_device *sdev,
		struct block_device *bdev, sector_t capacity, int *info);
static int arcmsr_queue_command(struct Scsi_Host *h, struct scsi_cmnd *cmd);
static int arcmsr_probe(struct pci_dev *pdev,
				const struct pci_device_id *id);
static int arcmsr_suspend(struct pci_dev *pdev, pm_message_t state);
static int arcmsr_resume(struct pci_dev *pdev);
static void arcmsr_remove(struct pci_dev *pdev);
static void arcmsr_shutdown(struct pci_dev *pdev);
static void arcmsr_iop_init(struct AdapterControlBlock *acb);
static void arcmsr_free_ccb_pool(struct AdapterControlBlock *acb);
static u32 arcmsr_disable_outbound_ints(struct AdapterControlBlock *acb);
static void arcmsr_enable_outbound_ints(struct AdapterControlBlock *acb,
	u32 orig_mask);
static void arcmsr_stop_adapter_bgrb(struct AdapterControlBlock *acb);
static void arcmsr_hbaA_flush_cache(struct AdapterControlBlock *acb);
static void arcmsr_hbaB_flush_cache(struct AdapterControlBlock *acb);
static void arcmsr_request_device_map(unsigned long pacb);
static void arcmsr_hbaA_request_device_map(struct AdapterControlBlock *acb);
static void arcmsr_hbaB_request_device_map(struct AdapterControlBlock *acb);
static void arcmsr_hbaC_request_device_map(struct AdapterControlBlock *acb);
static void arcmsr_message_isr_bh_fn(struct work_struct *work);
static bool arcmsr_get_firmware_spec(struct AdapterControlBlock *acb);
static void arcmsr_start_adapter_bgrb(struct AdapterControlBlock *acb);
static void arcmsr_hbaC_message_isr(struct AdapterControlBlock *pACB);
static void arcmsr_hbaD_message_isr(struct AdapterControlBlock *acb);
static void arcmsr_hardware_reset(struct AdapterControlBlock *acb);
static const char *arcmsr_info(struct Scsi_Host *);
static irqreturn_t arcmsr_interrupt(struct AdapterControlBlock *acb);
static int arcmsr_adjust_disk_queue_depth(struct scsi_device *sdev,
					  int queue_depth, int reason)
{
	if (reason != SCSI_QDEPTH_DEFAULT)
		return -EOPNOTSUPP;

	if (queue_depth > ARCMSR_MAX_CMD_PERLUN)
		queue_depth = ARCMSR_MAX_CMD_PERLUN;
	scsi_adjust_queue_depth(sdev, MSG_ORDERED_TAG, queue_depth);
	return queue_depth;
}

static struct scsi_host_template arcmsr_scsi_host_template = {
	.module			= THIS_MODULE,
	.name			= "ARCMSR ARECA SATA/SAS RAID Controller"
				ARCMSR_DRIVER_VERSION,
	.info			= arcmsr_info,
	.queuecommand		= arcmsr_queue_command,
	.eh_abort_handler		= arcmsr_abort,
	.eh_bus_reset_handler	= arcmsr_bus_reset,
	.bios_param		= arcmsr_bios_param,
	.change_queue_depth	= arcmsr_adjust_disk_queue_depth,
	.can_queue		= ARCMSR_MAX_FREECCB_NUM,
	.this_id			= ARCMSR_SCSI_INITIATOR_ID,
	.cmd_per_lun		= ARCMSR_MAX_CMD_PERLUN,
	.use_clustering		= ENABLE_CLUSTERING,
	.shost_attrs		= arcmsr_host_attrs,
};
static struct pci_device_id arcmsr_device_id_table[] = {
	{PCI_DEVICE(PCI_VENDOR_ID_ARECA, PCI_DEVICE_ID_ARECA_1110)},
	{PCI_DEVICE(PCI_VENDOR_ID_ARECA, PCI_DEVICE_ID_ARECA_1120)},
	{PCI_DEVICE(PCI_VENDOR_ID_ARECA, PCI_DEVICE_ID_ARECA_1130)},
	{PCI_DEVICE(PCI_VENDOR_ID_ARECA, PCI_DEVICE_ID_ARECA_1160)},
	{PCI_DEVICE(PCI_VENDOR_ID_ARECA, PCI_DEVICE_ID_ARECA_1170)},
	{PCI_DEVICE(PCI_VENDOR_ID_ARECA, PCI_DEVICE_ID_ARECA_1200)},
	{PCI_DEVICE(PCI_VENDOR_ID_ARECA, PCI_DEVICE_ID_ARECA_1201)},
	{PCI_DEVICE(PCI_VENDOR_ID_ARECA, PCI_DEVICE_ID_ARECA_1202)},
	{PCI_DEVICE(PCI_VENDOR_ID_ARECA, PCI_DEVICE_ID_ARECA_1210)},
	{PCI_DEVICE(PCI_VENDOR_ID_ARECA, PCI_DEVICE_ID_ARECA_1214)},
	{PCI_DEVICE(PCI_VENDOR_ID_ARECA, PCI_DEVICE_ID_ARECA_1220)},
	{PCI_DEVICE(PCI_VENDOR_ID_ARECA, PCI_DEVICE_ID_ARECA_1230)},
	{PCI_DEVICE(PCI_VENDOR_ID_ARECA, PCI_DEVICE_ID_ARECA_1260)},
	{PCI_DEVICE(PCI_VENDOR_ID_ARECA, PCI_DEVICE_ID_ARECA_1270)},
	{PCI_DEVICE(PCI_VENDOR_ID_ARECA, PCI_DEVICE_ID_ARECA_1280)},
	{PCI_DEVICE(PCI_VENDOR_ID_ARECA, PCI_DEVICE_ID_ARECA_1680)},
	{PCI_DEVICE(PCI_VENDOR_ID_ARECA, PCI_DEVICE_ID_ARECA_1681)},
	{PCI_DEVICE(PCI_VENDOR_ID_ARECA, PCI_DEVICE_ID_ARECA_1880)},
	{0, 0}, /* Terminating entry */
};
MODULE_DEVICE_TABLE(pci, arcmsr_device_id_table);
static struct pci_driver arcmsr_pci_driver = {
	.name			= "arcmsr",
	.id_table			= arcmsr_device_id_table,
	.probe			= arcmsr_probe,
	.remove			= arcmsr_remove,
		.suspend		= arcmsr_suspend,
		.resume		= arcmsr_resume,
	.shutdown		= arcmsr_shutdown,
};
/*
****************************************************************************
****************************************************************************
*/
static void arcmsr_free_mu(struct AdapterControlBlock *acb)
{
	switch (acb->adapter_type) {
	case ACB_ADAPTER_TYPE_A:
	case ACB_ADAPTER_TYPE_C:
		break;
	case ACB_ADAPTER_TYPE_B: {
		struct MessageUnit_B *reg = acb->pmuB;
		dma_free_coherent(&acb->pdev->dev,
			sizeof(struct MessageUnit_B),
			reg, acb->dma_coherent_handle2);
		break;
	}
	case ACB_ADAPTER_TYPE_D: {
		dma_free_coherent(&acb->pdev->dev,
			sizeof(struct MessageUnit_D),
			acb->dma_coherent,
			acb->dma_coherent_handle);
		break;
	}
	}
}

static bool arcmsr_remap_pciregion(struct AdapterControlBlock *acb)
{
	struct pci_dev *pdev = acb->pdev;
	switch (acb->adapter_type) {
	case ACB_ADAPTER_TYPE_A: {
		acb->pmuA = ioremap(pci_resource_start(pdev, 0),
			pci_resource_len(pdev, 0));
		if (!acb->pmuA) {
			pr_notice("arcmsr%d: memory mapping "
				"region fail\n", acb->host->host_no);
			return false;
		}
		break;
	}
	case ACB_ADAPTER_TYPE_B: {
		void __iomem *mem_base0, *mem_base1;
		mem_base0 = ioremap(pci_resource_start(pdev, 0),
			pci_resource_len(pdev, 0));
		if (!mem_base0) {
			pr_notice("arcmsr%d: memory mapping "
				"region fail\n", acb->host->host_no);
			return false;
		}
		mem_base1 = ioremap(pci_resource_start(pdev, 2),
			pci_resource_len(pdev, 2));
		if (!mem_base1) {
			iounmap(mem_base0);
			pr_notice("arcmsr%d: memory mapping "
				"region fail\n", acb->host->host_no);
			return false;
		}
		acb->mem_base0 = mem_base0;
		acb->mem_base1 = mem_base1;
		break;
	}
	case ACB_ADAPTER_TYPE_C: {
		acb->pmuC = ioremap_nocache(pci_resource_start(pdev, 1),
			pci_resource_len(pdev, 1));
		if (!acb->pmuC) {
			pr_notice("arcmsr%d: memory mapping "
				"region fail\n", acb->host->host_no);
			return false;
		}
		if (readl(&acb->pmuC->outbound_doorbell) &
			ARCMSR_HBCMU_IOP2DRV_MESSAGE_CMD_DONE) {
			writel(ARCMSR_HBCMU_IOP2DRV_MESSAGE_CMD_DONE_DOORBELL_CLEAR,
				&acb->pmuC->outbound_doorbell_clear);
			return true;
		}
		break;
	}
	case ACB_ADAPTER_TYPE_D: {
		void __iomem *mem_base0;
		unsigned long addr, range, flags;

		addr = (unsigned long)pci_resource_start(pdev, 0);
		range = pci_resource_len(pdev, 0);
		flags = pci_resource_flags(pdev, 0);
		if (flags & IORESOURCE_CACHEABLE)
			mem_base0 = ioremap(addr, range);
		else
			mem_base0 = ioremap_nocache(addr, range);
		if (!mem_base0) {
			pr_notice("arcmsr%d: memory mapping region fail\n",
			acb->host->host_no);
			return false;
		}
		acb->mem_base0 = mem_base0;
		break;
	}
	}
	return true;
}

static void arcmsr_unmap_pciregion(struct AdapterControlBlock *acb)
{
	switch (acb->adapter_type) {
	case ACB_ADAPTER_TYPE_A: {
		iounmap(acb->pmuA);
		break;
	}
	case ACB_ADAPTER_TYPE_B: {
		iounmap(acb->mem_base0);
		iounmap(acb->mem_base1);
		break;
	}
	case ACB_ADAPTER_TYPE_C: {
		iounmap(acb->pmuC);
		break;
	}
	case ACB_ADAPTER_TYPE_D: {
		iounmap(acb->mem_base0);
	}
	}
}

static irqreturn_t arcmsr_do_interrupt(int irq, void *dev_id)
{
	irqreturn_t handle_state;
	struct AdapterControlBlock *acb = dev_id;

	handle_state = arcmsr_interrupt(acb);
	return handle_state;
}

static int arcmsr_bios_param(struct scsi_device *sdev,
		struct block_device *bdev, sector_t capacity, int *geom)
{
	int ret, heads, sectors, cylinders, total_capacity;
	unsigned char *buffer;

	buffer = scsi_bios_ptable(bdev);
	if (buffer) {
		ret = scsi_partsize(buffer, capacity, &geom[2], &geom[0],
			&geom[1]);
		kfree(buffer);
		if (ret != -1)
			return ret;
	}
	total_capacity = capacity;
	heads = 64;
	sectors = 32;
	cylinders = total_capacity / (heads * sectors);
	if (cylinders > 1024) {
		heads = 255;
		sectors = 63;
		cylinders = total_capacity / (heads * sectors);
	}
	geom[0] = heads;
	geom[1] = sectors;
	geom[2] = cylinders;
	return 0;
}

static bool
arcmsr_define_adapter_type(struct AdapterControlBlock *acb)
{
	u16 dev_id;
	struct pci_dev *pdev = acb->pdev;

	pci_read_config_word(pdev, PCI_DEVICE_ID, &dev_id);
	acb->dev_id = dev_id;
	switch (dev_id) {
	case 0x1880: {
		acb->adapter_type = ACB_ADAPTER_TYPE_C;
		break;
	}
	case 0x1200:
	case 0x1201:
	case 0x1202: {
		acb->adapter_type = ACB_ADAPTER_TYPE_B;
		break;
	}
	case 0x1110:
	case 0x1120:
	case 0x1130:
	case 0x1160:
	case 0x1170:
	case 0x1210:
	case 0x1220:
	case 0x1230:
	case 0x1260:
	case 0x1280:
	case 0x1680: {
		acb->adapter_type = ACB_ADAPTER_TYPE_A;
		break;
	}
	case 0x1214: {
		acb->adapter_type = ACB_ADAPTER_TYPE_D;
		break;
	}
	default: {
		pr_notice("Unknown device ID = 0x%x\n", dev_id);
		return false;
	}
	}
	return true;
}

static bool
arcmsr_hbaA_wait_msgint_ready(struct AdapterControlBlock *acb)
{
	struct MessageUnit_A __iomem *reg = acb->pmuA;
	int i;

	for (i = 0; i < 2000; i++) {
		if (readl(&reg->outbound_intstatus) &
				ARCMSR_MU_OUTBOUND_MESSAGE0_INT) {
			writel(ARCMSR_MU_OUTBOUND_MESSAGE0_INT,
				&reg->outbound_intstatus);
			return true;
		}
		msleep(10);
	} /* max 20 seconds */

	return false;
}

static bool
arcmsr_hbaB_wait_msgint_ready(struct AdapterControlBlock *acb)
{
	struct MessageUnit_B *reg = acb->pmuB;
	int i;

	for (i = 0; i < 2000; i++) {
		if (readl(reg->iop2drv_doorbell)
			& ARCMSR_IOP2DRV_MESSAGE_CMD_DONE) {
			writel(ARCMSR_MESSAGE_INT_CLEAR_PATTERN,
					reg->iop2drv_doorbell);
			writel(ARCMSR_DRV2IOP_END_OF_INTERRUPT,
					reg->drv2iop_doorbell);
			return true;
		}
		msleep(10);
	} /* max 20 seconds */

	return false;
}

static bool
arcmsr_hbaC_wait_msgint_ready(struct AdapterControlBlock *pACB)
{
	struct MessageUnit_C *phbcmu = (struct MessageUnit_C *)pACB->pmuC;
	int i;

	for (i = 0; i < 2000; i++) {
		if (readl(&phbcmu->outbound_doorbell)
			& ARCMSR_HBCMU_IOP2DRV_MESSAGE_CMD_DONE) {
			writel(ARCMSR_HBCMU_IOP2DRV_MESSAGE_CMD_DONE_DOORBELL_CLEAR,
			&phbcmu->outbound_doorbell_clear);
			return true;
		}
		msleep(10);
	}
	return false;
}

static bool
arcmsr_hbaD_wait_msgint_ready(struct AdapterControlBlock *pACB)
{
	int i;
	struct MessageUnit_D __iomem *reg = (struct MessageUnit_D *)pACB->pmuD;
	for (i = 0; i < 2000; i++) {
		if (readl(reg->outbound_doorbell)
			& ARCMSR_ARC1214_IOP2DRV_MESSAGE_CMD_DONE) {
			writel(ARCMSR_ARC1214_IOP2DRV_MESSAGE_CMD_DONE,
				reg->outbound_doorbell);
			return true;
		}
		msleep(10);
	} /* max 20 seconds */
	return false;
}

static void
arcmsr_hbaA_flush_cache(struct AdapterControlBlock *acb)
{
	struct MessageUnit_A __iomem *reg = acb->pmuA;
	int retry_count = 30;
	writel(ARCMSR_INBOUND_MESG0_FLUSH_CACHE, &reg->inbound_msgaddr0);
	do {
		if (arcmsr_hbaA_wait_msgint_ready(acb))
			break;
		else {
			retry_count--;
			pr_notice("arcmsr%d: wait 'flush adapter "
			"cache' timeout, retry count down = %d\n",
			acb->host->host_no, retry_count);
		}
	} while (retry_count != 0);
}

static void
arcmsr_hbaB_flush_cache(struct AdapterControlBlock *acb)
{
	struct MessageUnit_B *reg = acb->pmuB;
	int retry_count = 30;
	writel(ARCMSR_MESSAGE_FLUSH_CACHE, reg->drv2iop_doorbell);
	do {
		if (arcmsr_hbaB_wait_msgint_ready(acb))
			break;
		else {
			retry_count--;
			pr_notice("arcmsr%d: wait 'flush adapter "
			"cache' timeout, retry count down = %d\n",
			acb->host->host_no, retry_count);
		}
	} while (retry_count != 0);
}

static void
arcmsr_hbaC_flush_cache(struct AdapterControlBlock *pACB)
{
	struct MessageUnit_C *reg = (struct MessageUnit_C *)pACB->pmuC;
	int retry_count = 6;/* enlarge wait flush adapter cache time: 10 minute */
	writel(ARCMSR_INBOUND_MESG0_FLUSH_CACHE,
		&reg->inbound_msgaddr0);
	writel(ARCMSR_HBCMU_DRV2IOP_MESSAGE_CMD_DONE,
		&reg->inbound_doorbell);
	readl(&reg->inbound_doorbell);
	readl(&reg->inbound_msgaddr0);
	do {
		if (arcmsr_hbaC_wait_msgint_ready(pACB)) {
			break;
		} else {
			retry_count--;
			pr_notice("arcmsr%d: wait 'flush adapter "
			"cache' timeout, retry count down = %d\n",
			pACB->host->host_no, retry_count);
		}
	} while (retry_count != 0);
	return;
}

static void
arcmsr_hbaD_flush_cache(struct AdapterControlBlock *pACB)
{
	int retry_count = 6;
	struct MessageUnit_D __iomem *reg =
		(struct MessageUnit_D *)pACB->pmuD;

	writel(ARCMSR_INBOUND_MESG0_FLUSH_CACHE,
		reg->inbound_msgaddr0);
	do {
		if (arcmsr_hbaD_wait_msgint_ready(pACB)) {
			break;
		} else {
			retry_count--;
			pr_notice("arcmsr%d: wait 'flush adapter "
			"cache' timeout, retry count down = %d\n",
			pACB->host->host_no,
			retry_count);
		}
	} while (retry_count != 0);
	return;
}

static void
arcmsr_flush_adapter_cache(struct AdapterControlBlock *acb)
{
	switch (acb->adapter_type) {

	case ACB_ADAPTER_TYPE_A: {
		arcmsr_hbaA_flush_cache(acb);
		}
		break;

	case ACB_ADAPTER_TYPE_B: {
		arcmsr_hbaB_flush_cache(acb);
		}
		break;
	case ACB_ADAPTER_TYPE_C: {
		arcmsr_hbaC_flush_cache(acb);
		}
		break;
	case ACB_ADAPTER_TYPE_D: {
		arcmsr_hbaD_flush_cache(acb);
	}
	}
}

static int
arcmsr_alloc_ccb_pool(struct AdapterControlBlock *acb)
{
	struct pci_dev *pdev = acb->pdev;
	void *dma_coherent;
	dma_addr_t dma_coherent_handle;
	struct CommandControlBlock *ccb_tmp = NULL;
	int i = 0, j = 0;
	dma_addr_t cdb_phyaddr;
	unsigned long roundup_ccbsize = 0;
	unsigned long max_xfer_len;
	unsigned long max_sg_entrys;
	uint32_t  firm_config_version;
	max_xfer_len = ARCMSR_MAX_XFER_LEN;
	max_sg_entrys = ARCMSR_DEFAULT_SG_ENTRIES;
	firm_config_version = acb->firm_cfg_version;
	if ((firm_config_version & 0xFF) >= 3) {
		max_xfer_len = (ARCMSR_CDB_SG_PAGE_LENGTH <<
			((firm_config_version >> 8) & 0xFF)) * 1024;
		max_sg_entrys = (max_xfer_len / 4096);
	}
	acb->host->max_sectors = max_xfer_len / 512;
	acb->host->sg_tablesize = max_sg_entrys;
	switch (acb->adapter_type) {
	case ACB_ADAPTER_TYPE_A: {
		roundup_ccbsize =
		roundup(sizeof(struct CommandControlBlock) +
		max_sg_entrys * sizeof(struct SG64ENTRY), 32);
		acb->uncache_size = roundup_ccbsize *
			ARCMSR_MAX_FREECCB_NUM;
		dma_coherent = dma_alloc_coherent(&pdev->dev,
			acb->uncache_size, &dma_coherent_handle,
			GFP_KERNEL);
		if (!dma_coherent) {
			pr_notice("arcmsr%d: dma_alloc_coherent "
			"got error\n", acb->host->host_no);
			return -ENOMEM;
		}
		memset(dma_coherent, 0, acb->uncache_size);
		acb->dma_coherent = dma_coherent;
		acb->dma_coherent_handle = dma_coherent_handle;
		ccb_tmp = (struct CommandControlBlock *)dma_coherent;
		acb->vir2phy_offset = (unsigned long)dma_coherent -
			(unsigned long)dma_coherent_handle;
		for (i = 0; i < ARCMSR_MAX_FREECCB_NUM; i++) {
			cdb_phyaddr = dma_coherent_handle +
				offsetof(struct CommandControlBlock,
				arcmsr_cdb);
			ccb_tmp->cdb_phyaddr = cdb_phyaddr >> 5;
			acb->pccb_pool[i] = ccb_tmp;
			ccb_tmp->acb = acb;
			INIT_LIST_HEAD(&ccb_tmp->list);
			list_add_tail(&ccb_tmp->list,
			&acb->ccb_free_list);
			ccb_tmp = (struct CommandControlBlock *)
			((unsigned long)ccb_tmp + roundup_ccbsize);
			dma_coherent_handle = dma_coherent_handle +
				roundup_ccbsize;
		}
		break;
	}
	case ACB_ADAPTER_TYPE_B: {
		roundup_ccbsize = roundup(sizeof(struct CommandControlBlock) +
			(max_sg_entrys - 1) * sizeof(struct SG64ENTRY), 32);
		acb->uncache_size = roundup_ccbsize *
			ARCMSR_MAX_FREECCB_NUM;
		dma_coherent = dma_alloc_coherent(&pdev->dev,
			acb->uncache_size, &dma_coherent_handle,
			GFP_KERNEL);
		if (!dma_coherent) {
			pr_notice("DMA allocation failed....\n");
			return -ENOMEM;
		}
		memset(dma_coherent, 0, acb->uncache_size);
		acb->dma_coherent = dma_coherent;
		acb->dma_coherent_handle = dma_coherent_handle;
		ccb_tmp = (struct CommandControlBlock *)dma_coherent;
		acb->vir2phy_offset = (unsigned long)dma_coherent -
			(unsigned long)dma_coherent_handle;
		for (i = 0; i < ARCMSR_MAX_FREECCB_NUM; i++) {
			cdb_phyaddr = dma_coherent_handle +
			offsetof(struct CommandControlBlock,
			arcmsr_cdb);
			ccb_tmp->cdb_phyaddr = cdb_phyaddr >> 5;
			acb->pccb_pool[i] = ccb_tmp;
			ccb_tmp->acb = acb;
			INIT_LIST_HEAD(&ccb_tmp->list);
			list_add_tail(&ccb_tmp->list,
			&acb->ccb_free_list);
			ccb_tmp = (struct CommandControlBlock *)
			((unsigned long)ccb_tmp + roundup_ccbsize);
			dma_coherent_handle = dma_coherent_handle +
			roundup_ccbsize;
		}
		break;
	}
	case ACB_ADAPTER_TYPE_C: {
		roundup_ccbsize =
		roundup(sizeof(struct CommandControlBlock) +
		(max_sg_entrys - 1) * sizeof(struct SG64ENTRY), 32);
		acb->uncache_size = roundup_ccbsize *
		ARCMSR_MAX_FREECCB_NUM;
		dma_coherent = dma_alloc_coherent(&pdev->dev,
			acb->uncache_size,
			&dma_coherent_handle,
			GFP_KERNEL);
		if (!dma_coherent) {
			pr_notice("arcmsr%d: dma_alloc_coherent "
			"got error\n", acb->host->host_no);
			return -ENOMEM;
		}
		memset(dma_coherent, 0, acb->uncache_size);
		acb->dma_coherent = dma_coherent;
		acb->dma_coherent_handle = dma_coherent_handle;
		acb->vir2phy_offset = (unsigned long)dma_coherent -
		(unsigned long)dma_coherent_handle;
		ccb_tmp = dma_coherent;
		for (i = 0; i < ARCMSR_MAX_FREECCB_NUM; i++) {
			cdb_phyaddr = dma_coherent_handle +
			offsetof(struct CommandControlBlock,
			arcmsr_cdb);
			ccb_tmp->cdb_phyaddr = cdb_phyaddr;
			acb->pccb_pool[i] = ccb_tmp;
			ccb_tmp->acb = acb;
			INIT_LIST_HEAD(&ccb_tmp->list);
			list_add_tail(&ccb_tmp->list,
				&acb->ccb_free_list);
			ccb_tmp = (struct CommandControlBlock *)
				((unsigned long)ccb_tmp +
				roundup_ccbsize);
			dma_coherent_handle = dma_coherent_handle +
				roundup_ccbsize;
		}
		break;
	}
	case ACB_ADAPTER_TYPE_D: {
		void *dma_coherent;
		dma_addr_t dma_coherent_handle;

		roundup_ccbsize = roundup(sizeof(struct CommandControlBlock)
		+ (max_sg_entrys - 1) * sizeof(struct SG64ENTRY), 32);
		dma_coherent = dma_alloc_coherent(&pdev->dev,
			roundup_ccbsize * ARCMSR_MAX_FREECCB_NUM,
			&dma_coherent_handle, GFP_KERNEL);
		if (!dma_coherent) {
			pr_notice("DMA allocation failed...\n");
			return -ENOMEM;
		}
		acb->roundup_ccbsize = roundup_ccbsize;
		acb->dma_coherent2 = dma_coherent;
		acb->dma_coherent_handle2 = dma_coherent_handle;
		ccb_tmp = dma_coherent;
		acb->vir2phy_offset = (unsigned long)dma_coherent -
			(unsigned long)dma_coherent_handle;
		for (i = 0; i < ARCMSR_MAX_FREECCB_NUM; i++) {
			cdb_phyaddr = dma_coherent_handle +
			offsetof(struct CommandControlBlock, arcmsr_cdb);
			ccb_tmp->cdb_phyaddr =
			cdb_phyaddr;
			acb->pccb_pool[i] = ccb_tmp;
			ccb_tmp->acb = acb;
			INIT_LIST_HEAD(&ccb_tmp->list);
			list_add_tail(&ccb_tmp->list, &acb->ccb_free_list);
			ccb_tmp = (struct CommandControlBlock *)
				((unsigned long)ccb_tmp + roundup_ccbsize);
			dma_coherent_handle = dma_coherent_handle
				+ roundup_ccbsize;
		}
	}
	}
	for (i = 0; i < ARCMSR_MAX_TARGETID; i++)
		for (j = 0; j < ARCMSR_MAX_TARGETLUN; j++)
			acb->devstate[i][j] = ARECA_RAID_GONE;
	return 0;
}

static void
arcmsr_message_isr_bh_fn(struct work_struct *work)
{
	struct AdapterControlBlock *acb = container_of(work,
		struct AdapterControlBlock, arcmsr_do_message_isr_bh);
	switch (acb->adapter_type) {
	case ACB_ADAPTER_TYPE_A: {
		struct MessageUnit_A __iomem *reg  = acb->pmuA;
		char *acb_dev_map = (char *)acb->device_map;
		uint32_t __iomem *signature = (uint32_t __iomem *)
			(&reg->message_rwbuffer[0]);
		char __iomem *devicemap = (char __iomem *)
			(&reg->message_rwbuffer[21]);
		int target, lun;
		struct scsi_device *psdev;
		char diff;

		atomic_inc(&acb->rq_map_token);
		if (readl(signature) == ARCMSR_SIGNATURE_GET_CONFIG) {
			for (target = 0; target < ARCMSR_MAX_TARGETID - 1;
			target++) {
				diff = (*acb_dev_map) ^ readb(devicemap);
				if (diff != 0) {
					char temp;
					*acb_dev_map = readb(devicemap);
					temp = *acb_dev_map;
					for (lun = 0; lun <
					ARCMSR_MAX_TARGETLUN; lun++) {
						if ((temp & 0x01) == 1 &&
						(diff & 0x01) == 1) {
							scsi_add_device(acb->host,
							0, target, lun);
						} else if ((temp & 0x01) == 0
						&& (diff & 0x01) == 1) {
							psdev =
							scsi_device_lookup(acb->host,
							0, target, lun);
							if (psdev != NULL) {
								scsi_remove_device(psdev);
								scsi_device_put(psdev);
							}
						}
						temp >>= 1;
						diff >>= 1;
					}
				}
				devicemap++;
				acb_dev_map++;
			}
		}
		break;
	}

	case ACB_ADAPTER_TYPE_B: {
		struct MessageUnit_B *reg  = acb->pmuB;
		char *acb_dev_map = (char *)acb->device_map;
		uint32_t __iomem *signature =
			(uint32_t __iomem *)(&reg->message_rwbuffer[0]);
		char __iomem *devicemap =
			(char __iomem *)(&reg->message_rwbuffer[21]);
		int target, lun;
		struct scsi_device *psdev;
		char diff;

		atomic_inc(&acb->rq_map_token);
		if (readl(signature) == ARCMSR_SIGNATURE_GET_CONFIG) {
			for (target = 0; target <
			ARCMSR_MAX_TARGETID - 1; target++) {
				diff = (*acb_dev_map) ^ readb(devicemap);
				if (diff != 0) {
					char temp;
					*acb_dev_map = readb(devicemap);
					temp = *acb_dev_map;
					for (lun = 0;
					lun < ARCMSR_MAX_TARGETLUN;
					lun++) {
						if ((temp & 0x01) == 1 &&
						(diff & 0x01) == 1) {
							scsi_add_device(acb->host,
							0, target, lun);
						} else if ((temp & 0x01) == 0
						&& (diff & 0x01) == 1) {
							psdev = scsi_device_lookup(acb->host,
							0, target, lun);
							if (psdev != NULL) {
								scsi_remove_device(psdev);
								scsi_device_put(psdev);
							}
						}
						temp >>= 1;
						diff >>= 1;
					}
				}
				devicemap++;
				acb_dev_map++;
			}
		}
	}
	break;
	case ACB_ADAPTER_TYPE_C: {
		struct MessageUnit_C *reg  = acb->pmuC;
		char *acb_dev_map = (char *)acb->device_map;
		uint32_t __iomem *signature =
			(uint32_t __iomem *)(&reg->msgcode_rwbuffer[0]);
		char __iomem *devicemap =
			(char __iomem *)(&reg->msgcode_rwbuffer[21]);
		int target, lun;
		struct scsi_device *psdev;
		char diff;

		atomic_inc(&acb->rq_map_token);
		if (readl(signature) == ARCMSR_SIGNATURE_GET_CONFIG) {
			for (target = 0; target <
				ARCMSR_MAX_TARGETID - 1; target++) {
				diff = (*acb_dev_map) ^ readb(devicemap);
				if (diff != 0) {
					char temp;
					*acb_dev_map =
						readb(devicemap);
					temp = *acb_dev_map;
					for (lun = 0; lun <
					ARCMSR_MAX_TARGETLUN; lun++) {
						if ((temp & 0x01) == 1 &&
						(diff & 0x01) == 1) {
							scsi_add_device(acb->host,
							0, target, lun);
						} else if ((temp & 0x01) == 0
						&& (diff & 0x01) == 1) {
							psdev = scsi_device_lookup(acb->host,
							0, target, lun);
							if (psdev != NULL) {
								scsi_remove_device(psdev);
								scsi_device_put(psdev);
							}
						}
						temp >>= 1;
						diff >>= 1;
					}
				}
				devicemap++;
				acb_dev_map++;
			}
		}
	}
	break;
	case ACB_ADAPTER_TYPE_D: {
		struct MessageUnit_D __iomem *reg  = acb->pmuD;
		char *acb_dev_map = (char *)acb->device_map;
		uint32_t __iomem *signature =
			(uint32_t __iomem *)(&reg->msgcode_rwbuffer[0]);
		char __iomem *devicemap =
			(char __iomem *)(&reg->msgcode_rwbuffer[21]);
		int target, lun;
		struct scsi_device *psdev;
		char diff;

		atomic_inc(&acb->rq_map_token);
		if (readl(signature) == ARCMSR_SIGNATURE_GET_CONFIG) {
			for (target = 0; target <
				ARCMSR_MAX_TARGETID - 1; target++) {
				diff = (*acb_dev_map) ^ readb(devicemap);
				if (diff != 0) {
					char temp;
					*acb_dev_map =
						readb(devicemap);
					temp = *acb_dev_map;
					for (lun = 0; lun <
					ARCMSR_MAX_TARGETLUN; lun++) {
						if ((temp & 0x01) == 1 &&
						(diff & 0x01) == 1) {
							scsi_add_device(acb->host,
							0, target, lun);
						} else if ((temp & 0x01) == 0
						&& (diff & 0x01) == 1) {
							psdev = scsi_device_lookup(acb->host,
							0, target, lun);
							if (psdev != NULL) {
								scsi_remove_device(psdev);
								scsi_device_put(psdev);
							}
						}
						temp >>= 1;
						diff >>= 1;
					}
				}
				devicemap++;
				acb_dev_map++;
			}
		}
		break;
	}
	}
}

static int
arcmsr_suspend(struct pci_dev *pdev, pm_message_t state)
{
	int i;
	uint32_t intmask_org;
	struct Scsi_Host *host = pci_get_drvdata(pdev);
	struct AdapterControlBlock *acb =
		(struct AdapterControlBlock *)host->hostdata;

	intmask_org = arcmsr_disable_outbound_ints(acb);
	if (acb->acb_flags & ACB_F_MSI_ENABLED) {
		free_irq(pdev->irq, acb);
		pci_disable_msi(pdev);
	} else if (acb->acb_flags & ACB_F_MSIX_ENABLED) {
		for (i = 0; i < ARCMST_NUM_MSIX_VECTORS; i++)
			free_irq(acb->entries[i].vector, acb);
		pci_disable_msix(pdev);
	} else
		free_irq(pdev->irq, acb);
	del_timer_sync(&acb->eternal_timer);
	flush_scheduled_work();
	arcmsr_stop_adapter_bgrb(acb);
	arcmsr_flush_adapter_cache(acb);
	arcmsr_enable_outbound_ints(acb, intmask_org);
	pci_set_drvdata(pdev, host);
	pci_save_state(pdev);
	pci_disable_device(pdev);
	pci_set_power_state(pdev, pci_choose_state(pdev, state));
	return 0;
}

static int
arcmsr_resume(struct pci_dev *pdev)
{
	int error, i, j;
	struct Scsi_Host *host = pci_get_drvdata(pdev);
	struct AdapterControlBlock *acb =
		(struct AdapterControlBlock *)host->hostdata;
	struct msix_entry entries[ARCMST_NUM_MSIX_VECTORS];
	pci_set_power_state(pdev, PCI_D0);
	pci_enable_wake(pdev, PCI_D0, 0);
	pci_restore_state(pdev);
	if (pci_enable_device(pdev)) {
		pr_warn("%s: pci_enable_device error\n", __func__);
		return -ENODEV;
	}
	error = pci_set_dma_mask(pdev, DMA_BIT_MASK(64));
	if (error) {
		error = pci_set_dma_mask(pdev, DMA_BIT_MASK(32));
		if (error) {
				pr_warn("scsi%d: No suitable DMA mask available\n",
			       host->host_no);
			goto controller_unregister;
		}
	}
	pci_set_master(pdev);
	if (pci_find_capability(pdev, PCI_CAP_ID_MSIX)) {
		if (!pci_enable_msix(pdev, entries,
			ARCMST_NUM_MSIX_VECTORS)) {
			for (i = 0; i < ARCMST_NUM_MSIX_VECTORS;
			i++) {
				entries[i].entry = i;
				if (request_irq(entries[i].vector,
				arcmsr_do_interrupt, 0,
				"arcmsr", acb)) {
					for (j = 0 ; j < i ; j++)
						free_irq(entries[i].vector,
						acb);
					goto controller_stop;
				}
				acb->entries[i] = entries[i];
			}
			acb->acb_flags |= ACB_F_MSIX_ENABLED;
		} else {
			pr_warn("arcmsr%d: MSI-X "
			"failed to enable\n", acb->host->host_no);
			if (request_irq(pdev->irq,
			arcmsr_do_interrupt, IRQF_SHARED,
			"arcmsr", acb)) {
				goto controller_stop;
			}
		}
	} else if (pci_find_capability(pdev, PCI_CAP_ID_MSI)) {
		if (!pci_enable_msi(pdev))
			acb->acb_flags |= ACB_F_MSI_ENABLED;
		if (request_irq(pdev->irq, arcmsr_do_interrupt,
			IRQF_SHARED, "arcmsr", acb)) {
			goto controller_stop;
		}
	} else {
		if (request_irq(pdev->irq, arcmsr_do_interrupt,
			IRQF_SHARED, "arcmsr", acb)) {
			goto controller_stop;
		}
	}
	arcmsr_iop_init(acb);
	INIT_WORK(&acb->arcmsr_do_message_isr_bh,
	arcmsr_message_isr_bh_fn);
	atomic_set(&acb->rq_map_token, 16);
	atomic_set(&acb->ante_token_value, 16);
	acb->fw_flag = FW_NORMAL;
	init_timer(&acb->eternal_timer);
	acb->eternal_timer.expires = jiffies +
		msecs_to_jiffies(6 * HZ);
	acb->eternal_timer.data = (unsigned long) acb;
	acb->eternal_timer.function =
		&arcmsr_request_device_map;
	add_timer(&acb->eternal_timer);
	return 0;
controller_stop:
		arcmsr_stop_adapter_bgrb(acb);
		arcmsr_flush_adapter_cache(acb);
controller_unregister:
		scsi_remove_host(host);
		arcmsr_free_ccb_pool(acb);
		arcmsr_unmap_pciregion(acb);
		pci_release_regions(pdev);
		scsi_host_put(host);
		pci_disable_device(pdev);
	return -ENODEV;
}

static int arcmsr_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	struct Scsi_Host *host;
	struct AdapterControlBlock *acb;
	uint8_t bus, dev_fun;
	struct msix_entry entries[ARCMST_NUM_MSIX_VECTORS];
	int error, i, j;
	error = pci_enable_device(pdev);
	if (error)
		return -ENODEV;
	host = scsi_host_alloc(&arcmsr_scsi_host_template,
		sizeof(struct AdapterControlBlock));
	if (!host)
    		goto pci_disable_dev;
	error = pci_set_dma_mask(pdev, DMA_BIT_MASK(64));
	if (error) {
		error = pci_set_dma_mask(pdev, DMA_BIT_MASK(32));
		if (error) {
			pr_warn("scsi%d: No suitable DMA mask available\n",
			       host->host_no);
			goto scsi_host_release;
		}
	}
	init_waitqueue_head(&wait_q);
	bus = pdev->bus->number;
	dev_fun = pdev->devfn;
	acb = (struct AdapterControlBlock *) host->hostdata;
	memset(acb,0,sizeof(struct AdapterControlBlock));
	acb->pdev = pdev;
	acb->host = host;
	host->max_lun = ARCMSR_MAX_TARGETLUN;
	host->max_id = ARCMSR_MAX_TARGETID;
	host->max_cmd_len = 16;
	host->can_queue = ARCMSR_MAX_FREECCB_NUM;
	host->cmd_per_lun = ARCMSR_MAX_CMD_PERLUN;
	host->this_id = ARCMSR_SCSI_INITIATOR_ID;
	host->unique_id = (bus << 8) | dev_fun;
	pci_set_drvdata(pdev, host);
	pci_set_master(pdev);
	error = pci_request_regions(pdev, "arcmsr");
	if (error)
		goto scsi_host_release;
	spin_lock_init(&acb->eh_lock);
	spin_lock_init(&acb->ccblist_lock);
	spin_lock_init(&acb->postq_lock);
	spin_lock_init(&acb->doneq_lock);
	spin_lock_init(&acb->rqbuffer_lock);
	spin_lock_init(&acb->wqbuffer_lock);
	acb->acb_flags |= (ACB_F_MESSAGE_WQBUFFER_CLEARED |
			ACB_F_MESSAGE_RQBUFFER_CLEARED |
			ACB_F_MESSAGE_WQBUFFER_READED);
	acb->acb_flags &= ~ACB_F_SCSISTOPADAPTER;
	INIT_LIST_HEAD(&acb->ccb_free_list);
	error = arcmsr_define_adapter_type(acb);
	if (!error)
		goto pci_release_regs;
	error = arcmsr_remap_pciregion(acb);
	if (!error)
		goto pci_release_regs;
	error = arcmsr_get_firmware_spec(acb);
	if (!error)
		goto unmap_pci_region;
	error = arcmsr_alloc_ccb_pool(acb);
	if (error)
		goto free_hbb_mu;
	error = scsi_add_host(host, &pdev->dev);
	if (error)
		goto RAID_controller_stop;
	if (pci_find_capability(pdev, PCI_CAP_ID_MSIX)) {
		if (!pci_enable_msix(pdev, entries, ARCMST_NUM_MSIX_VECTORS)) {
			for (i = 0; i < ARCMST_NUM_MSIX_VECTORS; i++) {
				entries[i].entry = i;
				if (request_irq(entries[i].vector,
					arcmsr_do_interrupt, 0, "arcmsr",
					acb)) {
					for (j = 0 ; j < i ; j++)
						free_irq(entries[i].vector,
						acb);
					goto scsi_host_remove;
				}
				acb->entries[i] = entries[i];
			}
			acb->acb_flags |= ACB_F_MSIX_ENABLED;
		} else {
			if (request_irq(pdev->irq, arcmsr_do_interrupt,
				IRQF_SHARED, "arcmsr", acb)) {
				printk("arcmsr%d: request_irq = %d failed!\n",
					acb->host->host_no, pdev->irq);
				goto scsi_host_remove;
			}
		}
	} else if (pci_find_capability(pdev, PCI_CAP_ID_MSI)) {
		if (!pci_enable_msi(pdev))
			acb->acb_flags |= ACB_F_MSI_ENABLED;
		if (request_irq(pdev->irq, arcmsr_do_interrupt,
			IRQF_SHARED, "arcmsr", acb)) {
			pr_warn("arcmsr%d: request_irq =%d failed!\n",
				acb->host->host_no, pdev->irq);
			goto scsi_host_remove;
		}
	} else {
		if (request_irq(pdev->irq, arcmsr_do_interrupt,
			IRQF_SHARED, "arcmsr", acb)) {
			pr_warn("arcmsr%d: request_irq = %d failed!\n",
				acb->host->host_no, pdev->irq);
			goto scsi_host_remove;
		}
	}
	host->irq = pdev->irq;
	arcmsr_iop_init(acb);
	scsi_scan_host(host);
	INIT_WORK(&acb->arcmsr_do_message_isr_bh,
		arcmsr_message_isr_bh_fn);
	atomic_set(&acb->rq_map_token, 16);
	atomic_set(&acb->ante_token_value, 16);
	acb->fw_flag = FW_NORMAL;
	init_timer(&acb->eternal_timer);
	acb->eternal_timer.expires = jiffies +
		msecs_to_jiffies(6 * HZ);
	acb->eternal_timer.data = (unsigned long) acb;
	acb->eternal_timer.function =
		&arcmsr_request_device_map;
	add_timer(&acb->eternal_timer);
	if (arcmsr_alloc_sysfs_attr(acb))
		goto out_free_sysfs;
	return 0;
out_free_sysfs:
scsi_host_remove:
	if (acb->acb_flags & ACB_F_MSI_ENABLED) {
		pci_disable_msi(pdev);
	} else if (acb->acb_flags & ACB_F_MSIX_ENABLED) {
		pci_disable_msix(pdev);
	}
	scsi_remove_host(host);
RAID_controller_stop:
	arcmsr_stop_adapter_bgrb(acb);
	arcmsr_flush_adapter_cache(acb);
	arcmsr_free_ccb_pool(acb);
free_hbb_mu:
	arcmsr_free_mu(acb);
unmap_pci_region:
	arcmsr_unmap_pciregion(acb);
pci_release_regs:
	pci_release_regions(pdev);
scsi_host_release:
	scsi_host_put(host);
pci_disable_dev:
	pci_disable_device(pdev);
	return -ENODEV;
}

static uint8_t
arcmsr_hbaA_abort_allcmd(struct AdapterControlBlock *acb)
{
	struct MessageUnit_A __iomem *reg = acb->pmuA;
	writel(ARCMSR_INBOUND_MESG0_ABORT_CMD,
		&reg->inbound_msgaddr0);
	if (!arcmsr_hbaA_wait_msgint_ready(acb)) {
		pr_notice("arcmsr%d: wait 'abort all outstanding "
		"command' timeout\n"
		, acb->host->host_no);
		return false;
	}
	return true;
}

static uint8_t
arcmsr_hbaB_abort_allcmd(struct AdapterControlBlock *acb)
{
	struct MessageUnit_B *reg = acb->pmuB;

	writel(ARCMSR_MESSAGE_ABORT_CMD,
		reg->drv2iop_doorbell);
	if (!arcmsr_hbaB_wait_msgint_ready(acb)) {
		pr_notice("arcmsr%d: wait 'abort all outstanding "
		"command' timeout\n"
		, acb->host->host_no);
		return false;
	}
	return true;
}
static uint8_t
arcmsr_hbaC_abort_allcmd(struct AdapterControlBlock *pACB)
{
	struct MessageUnit_C *reg = (struct MessageUnit_C *)pACB->pmuC;
	writel(ARCMSR_INBOUND_MESG0_ABORT_CMD, &reg->inbound_msgaddr0);
	writel(ARCMSR_HBCMU_DRV2IOP_MESSAGE_CMD_DONE, &reg->inbound_doorbell);
	if (!arcmsr_hbaC_wait_msgint_ready(pACB)) {
		pr_notice("arcmsr%d: wait 'abort all outstanding "
		"command' timeout\n"
		, pACB->host->host_no);
		return false;
	}
	return true;
}
static uint8_t
arcmsr_hbaD_abort_allcmd(struct AdapterControlBlock *pACB)
{
	struct MessageUnit_D __iomem *reg = (struct MessageUnit_D *)pACB->pmuD;
	writel(ARCMSR_INBOUND_MESG0_ABORT_CMD, reg->inbound_msgaddr0);
	if (!arcmsr_hbaD_wait_msgint_ready(pACB)) {
		pr_notice("arcmsr%d: wait 'abort all outstanding "
		"command' timeout\n"
		, pACB->host->host_no);
		return false;
	}
	return true;
}
static uint8_t
arcmsr_abort_allcmd(struct AdapterControlBlock *acb)
{
	uint8_t rtnval = 0;
	switch (acb->adapter_type) {
	case ACB_ADAPTER_TYPE_A: {
		rtnval = arcmsr_hbaA_abort_allcmd(acb);
		}
		break;

	case ACB_ADAPTER_TYPE_B: {
		rtnval = arcmsr_hbaB_abort_allcmd(acb);
		}
		break;

	case ACB_ADAPTER_TYPE_C: {
		rtnval = arcmsr_hbaC_abort_allcmd(acb);
		}
		break;
	case ACB_ADAPTER_TYPE_D: {
		rtnval = arcmsr_hbaD_abort_allcmd(acb);
	}
	}
	return rtnval;
}

static bool
arcmsr_hbb_enable_driver_mode(struct AdapterControlBlock *pacb)
{
	struct MessageUnit_B *reg = pacb->pmuB;
	writel(ARCMSR_MESSAGE_START_DRIVER_MODE,
		reg->drv2iop_doorbell);
	if (!arcmsr_hbaB_wait_msgint_ready(pacb)) {
		pr_err("arcmsr%d: can't set driver mode.\n",
			pacb->host->host_no);
		return false;
	}
    	return true;
}

static void
arcmsr_pci_unmap_dma(struct CommandControlBlock *ccb)
{
	struct scsi_cmnd *pcmd = ccb->pcmd;
	scsi_dma_unmap(pcmd);
}

static void
arcmsr_ccb_complete(struct CommandControlBlock *ccb)
{
	struct AdapterControlBlock *acb = ccb->acb;
	struct scsi_cmnd *pcmd = ccb->pcmd;
	unsigned long flags;
	atomic_dec(&acb->ccboutstandingcount);
	arcmsr_pci_unmap_dma(ccb);
	ccb->startdone = ARCMSR_CCB_DONE;
	spin_lock_irqsave(&acb->ccblist_lock, flags);
	list_add_tail(&ccb->list, &acb->ccb_free_list);
	spin_unlock_irqrestore(&acb->ccblist_lock, flags);
	pcmd->scsi_done(pcmd);
}

static void
arcmsr_report_sense_info(struct CommandControlBlock *ccb)
{

	struct scsi_cmnd *pcmd = ccb->pcmd;
	struct SENSE_DATA *sensebuffer =
		(struct SENSE_DATA *)pcmd->sense_buffer;
	pcmd->result = (DID_OK << 16) | (CHECK_CONDITION << 1)
		| (DRIVER_SENSE << 24);
	if (sensebuffer) {
		int sense_data_length =
			sizeof(struct SENSE_DATA) < SCSI_SENSE_BUFFERSIZE
			? sizeof(struct SENSE_DATA) : SCSI_SENSE_BUFFERSIZE;
		memset(sensebuffer, 0, SCSI_SENSE_BUFFERSIZE);
		memcpy(sensebuffer, ccb->arcmsr_cdb.SenseData,
			sense_data_length);
		sensebuffer->ErrorCode = SCSI_SENSE_CURRENT_ERRORS;
		sensebuffer->Valid = 1;
	}
}

static u32
arcmsr_disable_outbound_ints(struct AdapterControlBlock *acb)
{
	u32 orig_mask = 0;
	switch (acb->adapter_type) {	
	case ACB_ADAPTER_TYPE_A : {
		struct MessageUnit_A __iomem *reg = acb->pmuA;
		orig_mask = readl(&reg->outbound_intmask);
		writel(orig_mask |
			ARCMSR_MU_OUTBOUND_ALL_INTMASKENABLE,
			&reg->outbound_intmask);
		}
		break;
	case ACB_ADAPTER_TYPE_B : {
		struct MessageUnit_B *reg = acb->pmuB;
		orig_mask = readl(reg->iop2drv_doorbell_mask);
		writel(0, reg->iop2drv_doorbell_mask);
		}
		break;
	case ACB_ADAPTER_TYPE_C: {
		struct MessageUnit_C __iomem *reg =
			(struct MessageUnit_C *)acb->pmuC;
		/* disable all outbound interrupt */
		orig_mask = readl(&reg->host_int_mask);
		writel(orig_mask | ARCMSR_HBCMU_ALL_INTMASKENABLE,
			&reg->host_int_mask);
		readl(&reg->host_int_mask);
		}
		break;
	case ACB_ADAPTER_TYPE_D: {
		struct MessageUnit_D __iomem *reg =
			(struct MessageUnit_D *)acb->pmuD;
		/* disable all outbound interrupt */
		writel(ARCMSR_ARC1214_ALL_INT_DISABLE,
		reg->pcief0_int_enable);
		break;
	}
	}
	return orig_mask;
}

static void
arcmsr_report_ccb_state(struct AdapterControlBlock *acb,
			struct CommandControlBlock *ccb, bool error)
{
	uint8_t id, lun;
	id = ccb->pcmd->device->id;
	lun = ccb->pcmd->device->lun;
	if (!error) {
		if (acb->devstate[id][lun] == ARECA_RAID_GONE)
			acb->devstate[id][lun] = ARECA_RAID_GOOD;
		ccb->pcmd->result = DID_OK << 16;
		arcmsr_ccb_complete(ccb);
	} else {
		switch (ccb->arcmsr_cdb.DeviceStatus) {
		case ARCMSR_DEV_SELECT_TIMEOUT: {
			acb->devstate[id][lun] = ARECA_RAID_GONE;
			ccb->pcmd->result = DID_NO_CONNECT << 16;
			arcmsr_ccb_complete(ccb);
			}
			break;

		case ARCMSR_DEV_ABORTED:

		case ARCMSR_DEV_INIT_FAIL: {
			acb->devstate[id][lun] = ARECA_RAID_GONE;
			ccb->pcmd->result = DID_BAD_TARGET << 16;
			arcmsr_ccb_complete(ccb);
			}
			break;

		case ARCMSR_DEV_CHECK_CONDITION: {
			acb->devstate[id][lun] = ARECA_RAID_GOOD;
			arcmsr_report_sense_info(ccb);
			arcmsr_ccb_complete(ccb);
			}
			break;

		default:
			pr_notice("arcmsr%d: scsi id = %d lun = %d "
			"isr get command error done, but got unknown "
			"DeviceStatus = 0x%x\n"
			, acb->host->host_no
			, id
			, lun
			, ccb->arcmsr_cdb.DeviceStatus);
			acb->devstate[id][lun] = ARECA_RAID_GONE;
			ccb->pcmd->result = DID_NO_CONNECT << 16;
			arcmsr_ccb_complete(ccb);
			break;
		}
	}
}

static void
arcmsr_drain_donequeue(struct AdapterControlBlock *acb,
	struct CommandControlBlock *pCCB, bool error)
{
	if ((pCCB->acb != acb) || (pCCB->startdone != ARCMSR_CCB_START)) {
		pr_notice("arcmsr%d: isr get an illegal ccb "
		"command done acb = 0x%p, "
		"ccb = 0x%p, "
		"ccbacb = 0x%p, "
		"startdone = 0x%x, "
		"pscsi_cmd = 0x%p, "
		"ccboutstandingcount = %d\n"
		, acb->host->host_no
		, acb
		, pCCB
		, pCCB->acb
		, pCCB->startdone
		, pCCB->pcmd
		, atomic_read(&acb->ccboutstandingcount));
		return;
	}
	arcmsr_report_ccb_state(acb, pCCB, error);
}

static void
arcmsr_done4abort_postqueue(struct AdapterControlBlock *acb)
{
	int i = 0;
	uint32_t flag_ccb;
	struct ARCMSR_CDB *pARCMSR_CDB;
	bool error;
	struct CommandControlBlock *pCCB;
	switch (acb->adapter_type) {

	case ACB_ADAPTER_TYPE_A: {
		struct MessageUnit_A __iomem *reg = acb->pmuA;
		uint32_t outbound_intstatus;
		outbound_intstatus = readl(&reg->outbound_intstatus) &
					acb->outbound_int_enable;
		/*clear and abort all outbound posted Q*/
		writel(outbound_intstatus, &reg->outbound_intstatus);
		while (((flag_ccb = readl(&reg->outbound_queueport))
			!= 0xFFFFFFFF)
			&& (i++ < ARCMSR_MAX_OUTSTANDING_CMD)) {
			pARCMSR_CDB = (struct ARCMSR_CDB *)
				(acb->vir2phy_offset + (flag_ccb << 5));
			pCCB = container_of(pARCMSR_CDB,
			struct CommandControlBlock, arcmsr_cdb);
			error = (flag_ccb & ARCMSR_CCBREPLY_FLAG_ERROR_MODE0) ?
				true : false;
			arcmsr_drain_donequeue(acb, pCCB, error);
		}
		}
		break;

	case ACB_ADAPTER_TYPE_B: {
		struct MessageUnit_B *reg = acb->pmuB;
		writel(ARCMSR_DOORBELL_INT_CLEAR_PATTERN,
			reg->iop2drv_doorbell);
		for (i = 0; i < ARCMSR_MAX_HBB_POSTQUEUE; i++) {
			flag_ccb = readl(&reg->done_qbuffer[i]);
			if (flag_ccb != 0) {
				writel(0, &reg->done_qbuffer[i]);
				pARCMSR_CDB = (struct ARCMSR_CDB *)
				(acb->vir2phy_offset + (flag_ccb << 5));
				pCCB = container_of(pARCMSR_CDB,
				struct CommandControlBlock, arcmsr_cdb);
				error = (flag_ccb &
				ARCMSR_CCBREPLY_FLAG_ERROR_MODE0) ?
				true : false;
				arcmsr_drain_donequeue(acb, pCCB, error);
			}
			reg->post_qbuffer[i] = 0;
		}
		reg->doneq_index = 0;
		reg->postq_index = 0;
		}
		break;
	case ACB_ADAPTER_TYPE_C: {
		struct MessageUnit_C *reg = acb->pmuC;
		struct  ARCMSR_CDB *pARCMSR_CDB;
		uint32_t flag_ccb, ccb_cdb_phy;
		bool error;
		struct CommandControlBlock *pCCB;
		while ((readl(&reg->host_int_status) &
			ARCMSR_HBCMU_OUTBOUND_POSTQUEUE_ISR)
			&& (i++ < ARCMSR_MAX_OUTSTANDING_CMD)) {
			/*need to do*/
			flag_ccb = readl(&reg->outbound_queueport_low);
			ccb_cdb_phy = (flag_ccb & 0xFFFFFFF0);
			pARCMSR_CDB = (struct  ARCMSR_CDB *)
				(acb->vir2phy_offset+ccb_cdb_phy);
			pCCB = container_of(pARCMSR_CDB,
				struct CommandControlBlock, arcmsr_cdb);
			error = (flag_ccb & ARCMSR_CCBREPLY_FLAG_ERROR_MODE1)
				? true : false;
			arcmsr_drain_donequeue(acb, pCCB, error);
		}
		}
		break;
	case ACB_ADAPTER_TYPE_D: {
		struct MessageUnit_D __iomem *pmu = acb->pmuD;
		uint32_t ccb_cdb_phy, outbound_write_pointer;
		uint32_t doneq_index, index_stripped, addressLow, residual;
		bool error;
		struct CommandControlBlock *pCCB;
		outbound_write_pointer = pmu->done_qbuffer[0].addressLow + 1;
		doneq_index = pmu->doneq_index;
		residual = atomic_read(&acb->ccboutstandingcount);
		for (i = 0; i < residual; i++) {
			while ((doneq_index & 0xFFF) != (outbound_write_pointer & 0xFFF)) {
				if (doneq_index & 0x4000) {
					index_stripped = doneq_index & 0xFFF;
					index_stripped += 1;
					index_stripped %=
					ARCMSR_MAX_ARC1214_DONEQUEUE;
					pmu->doneq_index = index_stripped ?
					(index_stripped | 0x4000) : (index_stripped + 1);
				} else {
					index_stripped = doneq_index;
					index_stripped += 1;
					index_stripped %=
					ARCMSR_MAX_ARC1214_DONEQUEUE;
					pmu->doneq_index =
					index_stripped ? index_stripped : ((index_stripped | 0x4000) + 1);
				}
				doneq_index = pmu->doneq_index;
				addressLow =
				pmu->done_qbuffer[doneq_index & 0xFFF].addressLow;
				ccb_cdb_phy = (addressLow & 0xFFFFFFF0);
				pARCMSR_CDB = (struct  ARCMSR_CDB *)
				(acb->vir2phy_offset + ccb_cdb_phy);
				pCCB = container_of(pARCMSR_CDB,
				struct CommandControlBlock, arcmsr_cdb);
				error =
				(addressLow & ARCMSR_CCBREPLY_FLAG_ERROR_MODE1)
				? true : false;
				arcmsr_drain_donequeue(acb, pCCB, error);
				writel(doneq_index,
					pmu->outboundlist_read_pointer);
			}
			mdelay(10);
			outbound_write_pointer =
				pmu->done_qbuffer[0].addressLow + 1;
			doneq_index = pmu->doneq_index;
		}
		pmu->postq_index = 0;
		pmu->doneq_index = 0x40FF;
		break;
	}
	}
}

static void
arcmsr_remove(struct pci_dev *pdev)
{
	struct Scsi_Host *host = pci_get_drvdata(pdev);
	struct AdapterControlBlock *acb =
		(struct AdapterControlBlock *)host->hostdata;
	int poll_count = 0, i;
	arcmsr_free_sysfs_attr(acb);
	scsi_remove_host(host);
	flush_work_sync(&acb->arcmsr_do_message_isr_bh);
	del_timer_sync(&acb->eternal_timer);
	arcmsr_disable_outbound_ints(acb);
	arcmsr_stop_adapter_bgrb(acb);
	arcmsr_flush_adapter_cache(acb);	
	acb->acb_flags |= ACB_F_SCSISTOPADAPTER;
	acb->acb_flags &= ~ACB_F_IOP_INITED;

	for (poll_count = 0; poll_count < ARCMSR_MAX_OUTSTANDING_CMD;
	poll_count++) {
		if (!atomic_read(&acb->ccboutstandingcount))
			break;
		arcmsr_interrupt(acb);
		msleep(25);
	}

	if (atomic_read(&acb->ccboutstandingcount)) {
		arcmsr_abort_allcmd(acb);
		arcmsr_done4abort_postqueue(acb);
		for (i = 0; i < ARCMSR_MAX_FREECCB_NUM; i++) {
			struct CommandControlBlock *ccb =
				acb->pccb_pool[i];
			if (ccb->startdone == ARCMSR_CCB_START) {
				ccb->startdone = ARCMSR_CCB_ABORTED;
				ccb->pcmd->result = DID_ABORT << 16;
				arcmsr_ccb_complete(ccb);
			}
		}
	}
	arcmsr_free_ccb_pool(acb);
	arcmsr_free_mu(acb);
	if (acb->acb_flags & ACB_F_MSI_ENABLED) {
		free_irq(pdev->irq, acb);
		pci_disable_msi(pdev);
	} else if (acb->acb_flags & ACB_F_MSIX_ENABLED) {
		for (i = 0; i < ARCMST_NUM_MSIX_VECTORS; i++)
			free_irq(acb->entries[i].vector, acb);
		pci_disable_msix(pdev);
	} else
		free_irq(pdev->irq, acb);
	arcmsr_unmap_pciregion(acb);
	pci_release_regions(pdev);
	scsi_host_put(host);
	pci_disable_device(pdev);
	pci_set_drvdata(pdev, NULL);
}

static void
arcmsr_shutdown(struct pci_dev *pdev)
{
	int i;
	struct Scsi_Host *host = pci_get_drvdata(pdev);
	struct AdapterControlBlock *acb =
		(struct AdapterControlBlock *)host->hostdata;
	del_timer_sync(&acb->eternal_timer);
	arcmsr_disable_outbound_ints(acb);
	if (acb->acb_flags & ACB_F_MSIX_ENABLED) {
		for (i = 0; i < ARCMST_NUM_MSIX_VECTORS; i++)
			free_irq(acb->entries[i].vector, acb);
		pci_disable_msix(pdev);
	} else
		free_irq(pdev->irq, acb);
	flush_work_sync(&acb->arcmsr_do_message_isr_bh);
	arcmsr_stop_adapter_bgrb(acb);
	arcmsr_flush_adapter_cache(acb);
}

static int arcmsr_module_init(void)
{
	int error = 0;
	error = pci_register_driver(&arcmsr_pci_driver);
	return error;
}

static void arcmsr_module_exit(void)
{
	pci_unregister_driver(&arcmsr_pci_driver);
}
module_init(arcmsr_module_init);
module_exit(arcmsr_module_exit);

static void
arcmsr_enable_outbound_ints(struct AdapterControlBlock *acb,
						u32 intmask_org)
{
	u32 mask;
	switch (acb->adapter_type) {

	case ACB_ADAPTER_TYPE_A: {
		struct MessageUnit_A __iomem *reg = acb->pmuA;
		mask = intmask_org &
		~(ARCMSR_MU_OUTBOUND_POSTQUEUE_INTMASKENABLE |
		     ARCMSR_MU_OUTBOUND_DOORBELL_INTMASKENABLE |
		     ARCMSR_MU_OUTBOUND_MESSAGE0_INTMASKENABLE);
		writel(mask, &reg->outbound_intmask);
		acb->outbound_int_enable = ~(intmask_org & mask) &
			0x000000ff;
		}
		break;

	case ACB_ADAPTER_TYPE_B: {
		struct MessageUnit_B *reg = acb->pmuB;
		mask = intmask_org | (ARCMSR_IOP2DRV_DATA_WRITE_OK |
			ARCMSR_IOP2DRV_DATA_READ_OK |
			ARCMSR_IOP2DRV_CDB_DONE |
			ARCMSR_IOP2DRV_MESSAGE_CMD_DONE);
		writel(mask, reg->iop2drv_doorbell_mask);
		acb->outbound_int_enable = (intmask_org | mask) &
			0x0000000f;
		}
		break;
	case ACB_ADAPTER_TYPE_C: {
		struct MessageUnit_C __iomem *reg = acb->pmuC;
		mask = ~(ARCMSR_HBCMU_UTILITY_A_ISR_MASK |
			ARCMSR_HBCMU_OUTBOUND_DOORBELL_ISR_MASK |
			ARCMSR_HBCMU_OUTBOUND_POSTQUEUE_ISR_MASK);
		writel(intmask_org & mask, &reg->host_int_mask);
		acb->outbound_int_enable = ~(intmask_org & mask) &
			0x0000000f;
		}
		break;
	case ACB_ADAPTER_TYPE_D: {
		struct MessageUnit_D __iomem *reg = acb->pmuD;
		mask = ARCMSR_ARC1214_ALL_INT_ENABLE;
		writel(intmask_org | mask, reg->pcief0_int_enable);
	}
	}
}

static int
arcmsr_build_ccb(struct AdapterControlBlock *acb,
	struct CommandControlBlock *ccb, struct scsi_cmnd *pcmd)
{
	struct ARCMSR_CDB *arcmsr_cdb =
		(struct ARCMSR_CDB *)&ccb->arcmsr_cdb;
	int8_t *psge = (int8_t *)&arcmsr_cdb->u;
	__le32 address_lo, address_hi;
	int arccdbsize = 0x30;
	__le32 length = 0;
	int i;
	struct scatterlist *sg;
	int nseg;
	ccb->pcmd = pcmd;
	memset(arcmsr_cdb, 0, sizeof(struct ARCMSR_CDB));
	arcmsr_cdb->TargetID = pcmd->device->id;
	arcmsr_cdb->LUN = pcmd->device->lun;
	arcmsr_cdb->Function = 1;
	memcpy(arcmsr_cdb->Cdb, pcmd->cmnd, pcmd->cmd_len);

	nseg = scsi_dma_map(pcmd);
	if (unlikely(nseg > acb->host->sg_tablesize || nseg < 0))
		return FAILED;
	scsi_for_each_sg(pcmd, sg, nseg, i) {
		/* Get the physical address of the current data pointer */
		length = cpu_to_le32(sg_dma_len(sg));
		address_lo = cpu_to_le32(dma_addr_lo32(sg_dma_address(sg)));
		address_hi = cpu_to_le32(dma_addr_hi32(sg_dma_address(sg)));
		if (address_hi == 0) {
			struct SG32ENTRY *pdma_sg =
				(struct SG32ENTRY *)psge;
			pdma_sg->address = address_lo;
			pdma_sg->length = length;
			psge += sizeof (struct SG32ENTRY);
			arccdbsize += sizeof (struct SG32ENTRY);
		} else {
			struct SG64ENTRY *pdma_sg =
			(struct SG64ENTRY *)psge;
			pdma_sg->addresshigh = address_hi;
			pdma_sg->address = address_lo;
			pdma_sg->length = length |
				cpu_to_le32(IS_SG64_ADDR);
			psge += sizeof (struct SG64ENTRY);
			arccdbsize += sizeof (struct SG64ENTRY);
		}
	}
	arcmsr_cdb->sgcount = (uint8_t)nseg;
	arcmsr_cdb->DataLength = scsi_bufflen(pcmd);
	arcmsr_cdb->msgPages = arccdbsize / 0x100 +
		(arccdbsize % 0x100 ? 1 : 0);
	if ( arccdbsize > 256)
		arcmsr_cdb->Flags |= ARCMSR_CDB_FLAG_SGL_BSIZE;
	if (pcmd->sc_data_direction == DMA_TO_DEVICE)
		arcmsr_cdb->Flags |= ARCMSR_CDB_FLAG_WRITE;
	ccb->arc_cdb_size = arccdbsize;
	return SUCCESS;
}

static void
arcmsr_post_ccb(struct AdapterControlBlock *acb,
	struct CommandControlBlock *ccb)
{
	uint32_t cdb_phyaddr = ccb->cdb_phyaddr;
	struct ARCMSR_CDB *arcmsr_cdb =
		(struct ARCMSR_CDB *)&ccb->arcmsr_cdb;
	u32 arccdbsize = ccb->arc_cdb_size;
	atomic_inc(&acb->ccboutstandingcount);
	ccb->startdone = ARCMSR_CCB_START;
	switch (acb->adapter_type) {
	case ACB_ADAPTER_TYPE_A: {
		struct MessageUnit_A __iomem *reg = acb->pmuA;

		if (arcmsr_cdb->Flags & ARCMSR_CDB_FLAG_SGL_BSIZE)
			writel(cdb_phyaddr |
			ARCMSR_CCBPOST_FLAG_SGL_BSIZE,
			&reg->inbound_queueport);
		else {
			writel(cdb_phyaddr, &reg->inbound_queueport);
		}
		}
		break;

	case ACB_ADAPTER_TYPE_B: {
		struct MessageUnit_B *reg = acb->pmuB;
		uint32_t ending_index, index = reg->postq_index;

		ending_index = ((index + 1) %
			ARCMSR_MAX_HBB_POSTQUEUE);
		writel(0, &reg->post_qbuffer[ending_index]);
		if (arcmsr_cdb->Flags & ARCMSR_CDB_FLAG_SGL_BSIZE) {
			writel(cdb_phyaddr |
			ARCMSR_CCBPOST_FLAG_SGL_BSIZE,
			 &reg->post_qbuffer[index]);
		} else {
			writel(cdb_phyaddr,
				&reg->post_qbuffer[index]);
		}
		index++;
		index %= ARCMSR_MAX_HBB_POSTQUEUE;
		reg->postq_index = index;
		writel(ARCMSR_DRV2IOP_CDB_POSTED,
			reg->drv2iop_doorbell);
		}
		break;
	case ACB_ADAPTER_TYPE_C: {
		struct MessageUnit_C *phbcmu =
			(struct MessageUnit_C *)acb->pmuC;
		uint32_t ccb_post_stamp, arc_cdb_size;

		arc_cdb_size = (ccb->arc_cdb_size > 0x300)
			? 0x300 : ccb->arc_cdb_size;
		ccb_post_stamp = (cdb_phyaddr |
			((arc_cdb_size - 1) >> 6) | 1);
		if (acb->cdb_phyaddr_hi32) {
			writel(acb->cdb_phyaddr_hi32,
			&phbcmu->inbound_queueport_high);
			writel(ccb_post_stamp,
			&phbcmu->inbound_queueport_low);
		} else {
			writel(ccb_post_stamp,
			&phbcmu->inbound_queueport_low);
		}
		}
		break;
	case ACB_ADAPTER_TYPE_D: {
		struct MessageUnit_D *pmu = acb->pmuD;
		u16 index_stripped;
		u16 postq_index;
		unsigned long flags;
		struct InBound_SRB *pinbound_srb;
		spin_lock_irqsave(&acb->postq_lock, flags);
		postq_index = pmu->postq_index;
		pinbound_srb = (struct InBound_SRB *)&pmu->post_qbuffer[postq_index & 0xFF];
		pinbound_srb->addressHigh = dma_addr_hi32(cdb_phyaddr);
		pinbound_srb->addressLow = dma_addr_lo32(cdb_phyaddr);
		pinbound_srb->length = arccdbsize / 4;
		arcmsr_cdb->msgContext = dma_addr_lo32(cdb_phyaddr);
		if (postq_index & 0x4000) {
			index_stripped = postq_index & 0xFF;
			index_stripped += 1;
			index_stripped %= ARCMSR_MAX_ARC1214_POSTQUEUE;
			pmu->postq_index = index_stripped ? (index_stripped | 0x4000) : index_stripped;
		} else {
			index_stripped = postq_index;
			index_stripped += 1;
			index_stripped %= ARCMSR_MAX_ARC1214_POSTQUEUE;
			pmu->postq_index = index_stripped ? index_stripped : (index_stripped | 0x4000);
		}
		writel(postq_index, pmu->inboundlist_write_pointer);
		spin_unlock_irqrestore(&acb->postq_lock, flags);
	}
	}
}

static void arcmsr_hbaA_stop_bgrb(struct AdapterControlBlock *acb)
{
	struct MessageUnit_A __iomem *reg = acb->pmuA;
	acb->acb_flags &= ~ACB_F_MSG_START_BGRB;
	writel(ARCMSR_INBOUND_MESG0_STOP_BGRB, &reg->inbound_msgaddr0);
	if (!arcmsr_hbaA_wait_msgint_ready(acb)) {
		pr_notice("arcmsr%d: wait 'stop adapter "
		"background rebulid' timeout\n"
		, acb->host->host_no);
	}
}

static void
arcmsr_hbaB_stop_bgrb(struct AdapterControlBlock *acb)
{
	struct MessageUnit_B *reg = acb->pmuB;
	acb->acb_flags &= ~ACB_F_MSG_START_BGRB;
	writel(ARCMSR_MESSAGE_STOP_BGRB, reg->drv2iop_doorbell);

	if (!arcmsr_hbaB_wait_msgint_ready(acb)) {
		pr_notice("arcmsr%d: wait 'stop adapter "
		"background rebulid' timeout\n"
		, acb->host->host_no);
	}
}

static void
arcmsr_hbaC_stop_bgrb(struct AdapterControlBlock *pACB)
{
	struct MessageUnit_C *reg = (struct MessageUnit_C *)pACB->pmuC;
	pACB->acb_flags &= ~ACB_F_MSG_START_BGRB;
	writel(ARCMSR_INBOUND_MESG0_STOP_BGRB, &reg->inbound_msgaddr0);
	writel(ARCMSR_HBCMU_DRV2IOP_MESSAGE_CMD_DONE,
		&reg->inbound_doorbell);
	if (!arcmsr_hbaC_wait_msgint_ready(pACB)) {
		pr_notice("arcmsr%d: wait 'stop adapter "
		"background rebulid' timeout\n"
		, pACB->host->host_no);
	}
	return;
}

static void
arcmsr_hbaD_stop_bgrb(struct AdapterControlBlock *pACB)
{
	struct MessageUnit_D __iomem *reg =
		(struct MessageUnit_D *)pACB->pmuD;

	pACB->acb_flags &= ~ACB_F_MSG_START_BGRB;
	writel(ARCMSR_INBOUND_MESG0_STOP_BGRB,
		reg->inbound_msgaddr0);
	if (!arcmsr_hbaD_wait_msgint_ready(pACB)) {
		pr_notice("arcmsr%d: wait 'stop adapter background "
		"rebulid' timeout\n"
		, pACB->host->host_no);
	}
	return;
}

static void
arcmsr_stop_adapter_bgrb(struct AdapterControlBlock *acb)
{
	switch (acb->adapter_type) {
	case ACB_ADAPTER_TYPE_A: {
		arcmsr_hbaA_stop_bgrb(acb);
		break;
	}
	case ACB_ADAPTER_TYPE_B: {
		arcmsr_hbaB_stop_bgrb(acb);
		break;
	}
	case ACB_ADAPTER_TYPE_C: {
		arcmsr_hbaC_stop_bgrb(acb);
		break;
	}
	case ACB_ADAPTER_TYPE_D: {
		arcmsr_hbaD_stop_bgrb(acb);
		break;
	}
	}
}

static void
arcmsr_free_ccb_pool(struct AdapterControlBlock *acb)
{
	switch (acb->adapter_type) {
	case ACB_ADAPTER_TYPE_A:
	case ACB_ADAPTER_TYPE_C:
	case ACB_ADAPTER_TYPE_B:
		dma_free_coherent(&acb->pdev->dev, acb->uncache_size,
			acb->dma_coherent, acb->dma_coherent_handle);
		break;
	case ACB_ADAPTER_TYPE_D: {
		dma_free_coherent(&acb->pdev->dev, acb->roundup_ccbsize,
			acb->dma_coherent2, acb->dma_coherent_handle2);
		break;
	}
	}
}

void
arcmsr_iop_message_read(struct AdapterControlBlock *acb)
{
	switch (acb->adapter_type) {
	case ACB_ADAPTER_TYPE_A: {
		struct MessageUnit_A __iomem *reg = acb->pmuA;
		writel(ARCMSR_INBOUND_DRIVER_DATA_READ_OK,
		&reg->inbound_doorbell);
		}
		break;

	case ACB_ADAPTER_TYPE_B: {
		struct MessageUnit_B *reg = acb->pmuB;
		writel(ARCMSR_DRV2IOP_DATA_READ_OK,
		reg->drv2iop_doorbell);
		}
		break;
	case ACB_ADAPTER_TYPE_C: {
		struct MessageUnit_C __iomem *reg = acb->pmuC;
		writel(ARCMSR_HBCMU_DRV2IOP_DATA_READ_OK, &reg->inbound_doorbell);
		/*readl(&reg->inbound_doorbell);*/
		}
		break;
	case ACB_ADAPTER_TYPE_D: {
		struct MessageUnit_D __iomem *reg = acb->pmuD;
		writel(ARCMSR_ARC1214_DRV2IOP_DATA_OUT_READ,
		reg->inbound_doorbell);
		/*readl(reg->inbound_doorbell);*/
		break;
	}
	}
}

static void
arcmsr_iop_message_wrote(struct AdapterControlBlock *acb)
{
	switch (acb->adapter_type) {
	case ACB_ADAPTER_TYPE_A: {
		struct MessageUnit_A __iomem *reg = acb->pmuA;
		/*
		** push inbound doorbell tell iop, driver data write ok
		** and wait reply on next hwinterrupt for next Qbuffer post
		*/
		writel(ARCMSR_INBOUND_DRIVER_DATA_WRITE_OK,
		&reg->inbound_doorbell);
		}
		break;

	case ACB_ADAPTER_TYPE_B: {
		struct MessageUnit_B *reg = acb->pmuB;
		/*
		** push inbound doorbell tell iop, driver data write ok
		** and wait reply on next hwinterrupt for next Qbuffer post
		*/
		writel(ARCMSR_DRV2IOP_DATA_WRITE_OK,
		reg->drv2iop_doorbell);
		}
		break;
	case ACB_ADAPTER_TYPE_C: {
		struct MessageUnit_C __iomem *reg = acb->pmuC;
		/*
		** push inbound doorbell tell iop, driver data write ok
		** and wait reply on next hwinterrupt for next Qbuffer post
		*/
		writel(ARCMSR_HBCMU_DRV2IOP_DATA_WRITE_OK,
		&reg->inbound_doorbell);
		}
		break;
	case ACB_ADAPTER_TYPE_D: {
		struct MessageUnit_D __iomem *reg = acb->pmuD;
		writel(ARCMSR_ARC1214_DRV2IOP_DATA_IN_READY,
		reg->inbound_doorbell);
		break;
	}
	}
}

struct QBUFFER __iomem
*arcmsr_get_iop_rqbuffer(struct AdapterControlBlock *acb)
{
	struct QBUFFER __iomem *qbuffer = NULL;
	switch (acb->adapter_type) {

	case ACB_ADAPTER_TYPE_A: {
		struct MessageUnit_A __iomem *reg = acb->pmuA;
		qbuffer = (struct QBUFFER __iomem *)&reg->message_rbuffer;
		}
		break;

	case ACB_ADAPTER_TYPE_B: {
		struct MessageUnit_B *reg = acb->pmuB;
		qbuffer = (struct QBUFFER __iomem *)reg->message_rbuffer;
		}
		break;
	case ACB_ADAPTER_TYPE_C: {
		struct MessageUnit_C *phbcmu =
			(struct MessageUnit_C *)acb->pmuC;
		qbuffer = (struct QBUFFER __iomem *)&phbcmu->message_rbuffer;
		}
		break;
	case ACB_ADAPTER_TYPE_D: {
		struct MessageUnit_D __iomem *reg =
			(struct MessageUnit_D *)acb->pmuD;
		qbuffer = (struct QBUFFER __iomem *)reg->message_rbuffer;
		break;
	}
	}
	return qbuffer;
}

struct QBUFFER __iomem
*arcmsr_get_iop_wqbuffer(struct AdapterControlBlock *acb)
{
	struct QBUFFER __iomem *pqbuffer = NULL;
	switch (acb->adapter_type) {

	case ACB_ADAPTER_TYPE_A: {
		struct MessageUnit_A __iomem *reg = acb->pmuA;
		pqbuffer = (struct QBUFFER __iomem *)&reg->message_wbuffer;
		}
		break;

	case ACB_ADAPTER_TYPE_B: {
		struct MessageUnit_B  *reg = acb->pmuB;
		pqbuffer = (struct QBUFFER __iomem *)reg->message_wbuffer;
		}
		break;
	case ACB_ADAPTER_TYPE_C: {
		struct MessageUnit_C *reg = (struct MessageUnit_C *)acb->pmuC;
		pqbuffer = (struct QBUFFER __iomem *)&reg->message_wbuffer;
		}
		break;
	case ACB_ADAPTER_TYPE_D: {
		struct MessageUnit_D __iomem *pmu =
		(struct MessageUnit_D *)acb->pmuD;
		pqbuffer = (struct QBUFFER __iomem *)pmu->message_wbuffer;
		break;
	}
	}
	return pqbuffer;
}

void
arcmsr_iop2drv_data_wrote_handle(struct AdapterControlBlock *acb)
{
	uint8_t __iomem *iop_data, *pQbuffer, *vaddr, *temp;
	int32_t buf_empty_len, data_len, data_len_residual;
	uint32_t rqbuf_firstindex, rqbuf_lastindex;
	unsigned long flags;
	struct QBUFFER __iomem  *prbuffer;
	spin_lock_irqsave(&acb->rqbuffer_lock, flags);
	rqbuf_lastindex = acb->rqbuf_lastindex;
	rqbuf_firstindex = acb->rqbuf_firstindex;
	prbuffer = arcmsr_get_iop_rqbuffer(acb);
	iop_data = (uint8_t __iomem *)prbuffer->data;
	data_len_residual = data_len = readl(&prbuffer->data_len);
	buf_empty_len = (rqbuf_firstindex - rqbuf_lastindex - 1) &
		(ARCMSR_MAX_QBUFFER - 1);
	if (buf_empty_len >= data_len) {
		if (data_len > 0) {
			temp = vaddr = kmalloc(data_len, GFP_ATOMIC);
			if (!vaddr) {
				goto leave;
			}
			acb->acb_flags &= ~ACB_F_IOPDATA_OVERFLOW;
			while (data_len_residual >= 4) {
				memcpy(temp, iop_data, 4);
				temp += 4;
				iop_data += 4;
				data_len_residual -= 4;
			}
			if ((data_len_residual > 0) &&
			(data_len_residual < 4)) {
				memcpy(temp, iop_data, data_len_residual);
			}
			pQbuffer = &acb->rqbuffer[rqbuf_lastindex];
			temp = vaddr;
			if ((rqbuf_lastindex + data_len) >
			ARCMSR_MAX_QBUFFER) {
				memcpy(pQbuffer, temp,
				ARCMSR_MAX_QBUFFER - rqbuf_lastindex);
				temp += (ARCMSR_MAX_QBUFFER -
				rqbuf_lastindex);
				rqbuf_lastindex = (rqbuf_lastindex + data_len)
				% ARCMSR_MAX_QBUFFER;
				memcpy(&acb->rqbuffer[0], temp,
				rqbuf_lastindex);
			} else {
				memcpy(pQbuffer, temp, data_len);
				rqbuf_lastindex = (rqbuf_lastindex + data_len)
				% ARCMSR_MAX_QBUFFER;
			}
			kfree(vaddr);
			acb->rqbuf_lastindex = rqbuf_lastindex;
			arcmsr_iop_message_read(acb);
		}
	} else {
		acb->acb_flags |= ACB_F_IOPDATA_OVERFLOW;
	}
	leave:
	spin_unlock_irqrestore(&acb->rqbuffer_lock, flags);
}

void
arcmsr_iop2drv_data_read_handle(struct AdapterControlBlock *acb)
{
	unsigned long flags;
	spin_lock_irqsave(&acb->wqbuffer_lock, flags);
	acb->acb_flags |= ACB_F_MESSAGE_WQBUFFER_READED;
	if (acb->wqbuf_firstindex != acb->wqbuf_lastindex) {
		uint8_t *pQbuffer;
		struct QBUFFER __iomem *pwbuffer;
		uint8_t __iomem *iop_data;
		int32_t allxfer_len = 0;

		acb->acb_flags &= (~ACB_F_MESSAGE_WQBUFFER_READED);
		pwbuffer = arcmsr_get_iop_wqbuffer(acb);
		iop_data = (uint8_t __iomem *)pwbuffer->data;
		if (acb->wqbuf_firstindex > acb->wqbuf_lastindex) {
			if ((ARCMSR_MAX_QBUFFER - acb->wqbuf_firstindex) >= 4) {
				do {
					pQbuffer =
					&acb->wqbuffer[acb->wqbuf_firstindex];
					if (acb->wqbuf_firstindex + 4
					> ARCMSR_MAX_QBUFFER) {
						memcpy(iop_data, pQbuffer,
						ARCMSR_MAX_QBUFFER
						- acb->wqbuf_firstindex);
						iop_data += ARCMSR_MAX_QBUFFER
						- acb->wqbuf_firstindex;
						acb->wqbuf_firstindex += 4;
						acb->wqbuf_firstindex %=
						ARCMSR_MAX_QBUFFER;
						memcpy(iop_data,
						&acb->wqbuffer[0],
						acb->wqbuf_firstindex);
						iop_data +=
						acb->wqbuf_firstindex;
					} else {
						if ((acb->wqbuf_lastindex
						- acb->wqbuf_firstindex) > 4) {
							memcpy(iop_data,
							pQbuffer, 4);
							acb->wqbuf_firstindex
							+= 4;
							acb->wqbuf_firstindex
							%= ARCMSR_MAX_QBUFFER;
							iop_data += 4;
						} else {
							memcpy(iop_data, pQbuffer,
							acb->wqbuf_lastindex
							- acb->wqbuf_firstindex);
							allxfer_len +=
							acb->wqbuf_lastindex -
							acb->wqbuf_firstindex;
							acb->wqbuf_firstindex =
							acb->wqbuf_lastindex;
							break;
						}
					}
					allxfer_len += 4;
				} while ((acb->wqbuf_firstindex !=
				acb->wqbuf_lastindex) && (allxfer_len < 124));
			} else {
				pQbuffer =
				&acb->wqbuffer[acb->wqbuf_firstindex];
				memcpy(iop_data, pQbuffer, ARCMSR_MAX_QBUFFER
				- acb->wqbuf_firstindex);
				iop_data += ARCMSR_MAX_QBUFFER
				- acb->wqbuf_firstindex;
				allxfer_len = ARCMSR_MAX_QBUFFER
				- acb->wqbuf_firstindex;
				acb->wqbuf_firstindex = 0;
				do {
					pQbuffer =
					&acb->wqbuffer[acb->wqbuf_firstindex];
					if ((acb->wqbuf_lastindex -
					acb->wqbuf_firstindex) > 4) {
						memcpy(iop_data, pQbuffer, 4);
						acb->wqbuf_firstindex += 4;
						acb->wqbuf_firstindex %=
						ARCMSR_MAX_QBUFFER;
						iop_data += 4;
					} else {
						memcpy(iop_data, pQbuffer,
						acb->wqbuf_lastindex -
						acb->wqbuf_firstindex);
						allxfer_len +=
						acb->wqbuf_lastindex
						- acb->wqbuf_firstindex;
						acb->wqbuf_firstindex =
						acb->wqbuf_lastindex;
						break;
					}
					allxfer_len += 4;
				} while ((acb->wqbuf_firstindex !=
				acb->wqbuf_lastindex) && (allxfer_len < 124));
			}
		} else {
			do {
				pQbuffer =
				&acb->wqbuffer[acb->wqbuf_firstindex];
				if ((acb->wqbuf_lastindex -
				acb->wqbuf_firstindex) > 4) {
					memcpy(iop_data, pQbuffer, 4);
					acb->wqbuf_firstindex += 4;
					acb->wqbuf_firstindex %=
					ARCMSR_MAX_QBUFFER;
					iop_data += 4;
				} else {
					memcpy(iop_data, pQbuffer,
					acb->wqbuf_lastindex -
					acb->wqbuf_firstindex);
					allxfer_len += acb->wqbuf_lastindex
					- acb->wqbuf_firstindex;
					acb->wqbuf_firstindex =
					acb->wqbuf_lastindex;
					break;
				}
				allxfer_len += 4;
			} while ((acb->wqbuf_firstindex !=
			acb->wqbuf_lastindex) && (allxfer_len < 124));
		}
		writel(allxfer_len, &pwbuffer->data_len);
		arcmsr_iop_message_wrote(acb);/*notice IOP the message has been written*/
	}
	if (acb->wqbuf_firstindex == acb->wqbuf_lastindex) {
		acb->acb_flags |= ACB_F_MESSAGE_WQBUFFER_CLEARED;
	}
	spin_unlock_irqrestore(&acb->wqbuffer_lock, flags);
}

static void
arcmsr_hbaA_doorbell_isr(struct AdapterControlBlock *acb)
{
	uint32_t outbound_doorbell;
	struct MessageUnit_A __iomem *reg  = acb->pmuA;
	outbound_doorbell = readl(&reg->outbound_doorbell);
	do {
		writel(outbound_doorbell, &reg->outbound_doorbell);
		if (outbound_doorbell &
			ARCMSR_OUTBOUND_IOP331_DATA_WRITE_OK) {
			arcmsr_iop2drv_data_wrote_handle(acb);
		}
		if (outbound_doorbell &
			ARCMSR_OUTBOUND_IOP331_DATA_READ_OK) {
			arcmsr_iop2drv_data_read_handle(acb);
		}
		outbound_doorbell = readl(&reg->outbound_doorbell);
	} while (outbound_doorbell &
	(ARCMSR_OUTBOUND_IOP331_DATA_WRITE_OK
	| ARCMSR_OUTBOUND_IOP331_DATA_READ_OK));
}
static void
arcmsr_hbaC_doorbell_isr(struct AdapterControlBlock *pACB)
{
	uint32_t outbound_doorbell;
	struct MessageUnit_C __iomem *reg =
		(struct MessageUnit_C *)pACB->pmuC;
	outbound_doorbell = readl(&reg->outbound_doorbell);
	do {
		if (outbound_doorbell &
			ARCMSR_HBCMU_IOP2DRV_MESSAGE_CMD_DONE) {
			arcmsr_hbaC_message_isr(pACB);
		}
		writel(outbound_doorbell, &reg->outbound_doorbell_clear);
		readl(&reg->outbound_doorbell_clear);
		if (outbound_doorbell &
			ARCMSR_HBCMU_IOP2DRV_DATA_WRITE_OK) {
			arcmsr_iop2drv_data_wrote_handle(pACB);
		}
		if (outbound_doorbell &
			ARCMSR_HBCMU_IOP2DRV_DATA_READ_OK) {
			arcmsr_iop2drv_data_read_handle(pACB);
		}
		outbound_doorbell = readl(&reg->outbound_doorbell);
	} while (outbound_doorbell & (ARCMSR_HBCMU_IOP2DRV_DATA_WRITE_OK
	| ARCMSR_HBCMU_IOP2DRV_DATA_READ_OK
	| ARCMSR_HBCMU_IOP2DRV_MESSAGE_CMD_DONE));
	return;
}

static void
arcmsr_hbaD_doorbell_isr(struct AdapterControlBlock *pACB)
{
	uint32_t outbound_doorbell;
	struct MessageUnit_D __iomem *pmu =
		(struct MessageUnit_D *)pACB->pmuD;

	outbound_doorbell = readl(pmu->outbound_doorbell);
	do {
		writel(outbound_doorbell, pmu->outbound_doorbell);
		if (outbound_doorbell &
			ARCMSR_ARC1214_IOP2DRV_MESSAGE_CMD_DONE) {
			arcmsr_hbaD_message_isr(pACB);
		}
		if (outbound_doorbell &
			ARCMSR_ARC1214_IOP2DRV_DATA_WRITE_OK) {
			arcmsr_iop2drv_data_wrote_handle(pACB);
		}
		if (outbound_doorbell &
			ARCMSR_ARC1214_IOP2DRV_DATA_READ_OK) {
			arcmsr_iop2drv_data_read_handle(pACB);
		}
		outbound_doorbell = readl(pmu->outbound_doorbell);
	} while (outbound_doorbell & (ARCMSR_ARC1214_IOP2DRV_DATA_WRITE_OK
	| ARCMSR_ARC1214_IOP2DRV_DATA_READ_OK
	| ARCMSR_ARC1214_IOP2DRV_MESSAGE_CMD_DONE));
	return;
}

static void
arcmsr_hbaA_postqueue_isr(struct AdapterControlBlock *acb)
{
	uint32_t flag_ccb;
	struct MessageUnit_A __iomem *reg = acb->pmuA;
	struct ARCMSR_CDB *pARCMSR_CDB;
	struct CommandControlBlock *pCCB;
	bool error;
	while ((flag_ccb = readl(&reg->outbound_queueport)) != 0xFFFFFFFF) {
		pARCMSR_CDB = (struct ARCMSR_CDB *)
			(acb->vir2phy_offset + (flag_ccb << 5));
		pCCB = container_of(pARCMSR_CDB,
			struct CommandControlBlock, arcmsr_cdb);
		error = (flag_ccb & ARCMSR_CCBREPLY_FLAG_ERROR_MODE0)
			? true : false;
		arcmsr_drain_donequeue(acb, pCCB, error);
	}
}
static void
arcmsr_hbaB_postqueue_isr(struct AdapterControlBlock *acb)
{
	uint32_t index;
	uint32_t flag_ccb;
	struct MessageUnit_B *reg = acb->pmuB;
	struct ARCMSR_CDB *pARCMSR_CDB;
	struct CommandControlBlock *pCCB;
	bool error;
	index = reg->doneq_index;
	while ((flag_ccb = readl(&reg->done_qbuffer[index])) != 0) {
		writel(0, &reg->done_qbuffer[index]);
		pARCMSR_CDB = (struct ARCMSR_CDB *)
		(acb->vir2phy_offset + (flag_ccb << 5));
		pCCB = container_of(pARCMSR_CDB,
		struct CommandControlBlock, arcmsr_cdb);
		error = (flag_ccb &
		ARCMSR_CCBREPLY_FLAG_ERROR_MODE0)
		? true : false;
		arcmsr_drain_donequeue(acb, pCCB, error);
		index++;
		index %= ARCMSR_MAX_HBB_POSTQUEUE;
		reg->doneq_index = index;
	}
}

static void
arcmsr_hbaC_postqueue_isr(struct AdapterControlBlock *acb)
{
	uint32_t flag_ccb, ccb_cdb_phy, throttling = 0;
	int error;
	struct MessageUnit_C __iomem *phbcmu;
	struct ARCMSR_CDB *arcmsr_cdb;
	struct CommandControlBlock *ccb;

	phbcmu = (struct MessageUnit_C *)acb->pmuC;
	/* areca cdb command done */
	/* Use correct offset and size for syncing */
	do {
		/* check if command done with no error*/
		flag_ccb = readl(&phbcmu->outbound_queueport_low);
		ccb_cdb_phy = (flag_ccb & 0xFFFFFFF0);
		arcmsr_cdb = (struct ARCMSR_CDB *)(acb->vir2phy_offset
			+ ccb_cdb_phy);
		ccb = container_of(arcmsr_cdb, struct CommandControlBlock,
			arcmsr_cdb);
		error = (flag_ccb & ARCMSR_CCBREPLY_FLAG_ERROR_MODE1)
			? true : false;
		/* check if command done with no error */
		arcmsr_drain_donequeue(acb, ccb, error);
		if (throttling == ARCMSR_HBC_ISR_THROTTLING_LEVEL) {
			writel(ARCMSR_HBCMU_DRV2IOP_POSTQUEUE_THROTTLING,
				&phbcmu->inbound_doorbell);
			continue;
		}
		throttling++;
	} while (readl(&phbcmu->host_int_status) &
	ARCMSR_HBCMU_OUTBOUND_POSTQUEUE_ISR);
}

static void
arcmsr_hbaD_postqueue_isr(struct AdapterControlBlock *acb)
{
	u32 outbound_write_pointer, doneq_index, index_stripped;
	uint32_t addressLow, ccb_cdb_phy;
	int error;
	struct MessageUnit_D __iomem *pmu;
	struct ARCMSR_CDB *arcmsr_cdb;
	struct CommandControlBlock *ccb;
	unsigned long flags;

	spin_lock_irqsave(&acb->doneq_lock, flags);
	pmu = (struct MessageUnit_D *)acb->pmuD;
	outbound_write_pointer = pmu->done_qbuffer[0].addressLow + 1;
	doneq_index = pmu->doneq_index;
	if ((doneq_index & 0xFFF) != (outbound_write_pointer & 0xFFF)) {
		do {
			if (doneq_index & 0x4000) {
				index_stripped = doneq_index & 0xFFF;
				index_stripped += 1;
				index_stripped %= ARCMSR_MAX_ARC1214_DONEQUEUE;
				pmu->doneq_index = index_stripped
				? (index_stripped | 0x4000) : (index_stripped + 1);
			} else {
				index_stripped = doneq_index;
				index_stripped += 1;
				index_stripped %= ARCMSR_MAX_ARC1214_DONEQUEUE;
				pmu->doneq_index = index_stripped
				? index_stripped : ((index_stripped | 0x4000) + 1);
			}
			doneq_index = pmu->doneq_index;
			addressLow =
			pmu->done_qbuffer[doneq_index & 0xFFF].addressLow;
			ccb_cdb_phy = (addressLow & 0xFFFFFFF0);
			arcmsr_cdb = (struct ARCMSR_CDB *)(acb->vir2phy_offset
				+ ccb_cdb_phy);
			ccb = container_of(arcmsr_cdb,
				struct CommandControlBlock, arcmsr_cdb);
			error = (addressLow & ARCMSR_CCBREPLY_FLAG_ERROR_MODE1)
				? true : false;
			arcmsr_drain_donequeue(acb, ccb, error);
			writel(doneq_index,
				pmu->outboundlist_read_pointer);
		} while ((doneq_index & 0xFFF) !=
		(outbound_write_pointer & 0xFFF));
	}
	writel(ARCMSR_ARC1214_OUTBOUND_LIST_INTERRUPT_CLEAR,
		pmu->outboundlist_interrupt_cause);
	readl(pmu->outboundlist_interrupt_cause);
	spin_unlock_irqrestore(&acb->doneq_lock, flags);
}

static void
arcmsr_hbaA_message_isr(struct AdapterControlBlock *acb)
{
	struct MessageUnit_A *reg  = acb->pmuA;
	/*clear interrupt and message state*/
	writel(ARCMSR_MU_OUTBOUND_MESSAGE0_INT, &reg->outbound_intstatus);
	schedule_work(&acb->arcmsr_do_message_isr_bh);
}
static void
arcmsr_hbaB_message_isr(struct AdapterControlBlock *acb)
{
	struct MessageUnit_B *reg  = acb->pmuB;

	/*clear interrupt and message state*/
	writel(ARCMSR_MESSAGE_INT_CLEAR_PATTERN, reg->iop2drv_doorbell);
	schedule_work(&acb->arcmsr_do_message_isr_bh);
}

static void
arcmsr_hbaC_message_isr(struct AdapterControlBlock *acb)
{
	struct MessageUnit_C *reg  = acb->pmuC;
	/*clear interrupt and message state*/
	writel(ARCMSR_HBCMU_IOP2DRV_MESSAGE_CMD_DONE_DOORBELL_CLEAR,
	&reg->outbound_doorbell_clear);
	schedule_work(&acb->arcmsr_do_message_isr_bh);
}

static void
arcmsr_hbaD_message_isr(struct AdapterControlBlock *acb)
{
	struct MessageUnit_D __iomem *reg  = acb->pmuD;
	writel(ARCMSR_ARC1214_IOP2DRV_MESSAGE_CMD_DONE,
		reg->outbound_doorbell);
	readl(reg->outbound_doorbell);
	schedule_work(&acb->arcmsr_do_message_isr_bh);
}

static irqreturn_t
arcmsr_hbaA_handle_isr(struct AdapterControlBlock *acb)
{
	uint32_t outbound_intstatus;
	struct MessageUnit_A __iomem *reg = acb->pmuA;
	outbound_intstatus =
	readl(&reg->outbound_intstatus) & acb->outbound_int_enable;
	if (!(outbound_intstatus & ARCMSR_MU_OUTBOUND_HANDLE_INT)) {
		return IRQ_NONE;
	}
	do {
		writel(outbound_intstatus, &reg->outbound_intstatus);
		if (outbound_intstatus & ARCMSR_MU_OUTBOUND_DOORBELL_INT)
			arcmsr_hbaA_doorbell_isr(acb);
		if (outbound_intstatus & ARCMSR_MU_OUTBOUND_POSTQUEUE_INT)
			arcmsr_hbaA_postqueue_isr(acb);
		if (outbound_intstatus & ARCMSR_MU_OUTBOUND_MESSAGE0_INT)
			arcmsr_hbaA_message_isr(acb);
		outbound_intstatus = readl(&reg->outbound_intstatus) &
			acb->outbound_int_enable;
	} while (outbound_intstatus & (ARCMSR_MU_OUTBOUND_DOORBELL_INT
	| ARCMSR_MU_OUTBOUND_POSTQUEUE_INT
	| ARCMSR_MU_OUTBOUND_MESSAGE0_INT));
	return IRQ_HANDLED;
}

static irqreturn_t
arcmsr_hbaB_handle_isr(struct AdapterControlBlock *acb)
{
	uint32_t outbound_doorbell;
	struct MessageUnit_B *reg = acb->pmuB;
	outbound_doorbell = readl(reg->iop2drv_doorbell)
		& acb->outbound_int_enable;
	if (!outbound_doorbell)
		return IRQ_NONE;
	do {
		writel(~outbound_doorbell, reg->iop2drv_doorbell);
		readl(reg->iop2drv_doorbell);
		writel(ARCMSR_DRV2IOP_END_OF_INTERRUPT,
			reg->drv2iop_doorbell);
		readl(reg->drv2iop_doorbell);
		if (outbound_doorbell & ARCMSR_IOP2DRV_DATA_WRITE_OK)
			arcmsr_iop2drv_data_wrote_handle(acb);
		if (outbound_doorbell & ARCMSR_IOP2DRV_DATA_READ_OK)
			arcmsr_iop2drv_data_read_handle(acb);
		if (outbound_doorbell & ARCMSR_IOP2DRV_CDB_DONE)
			arcmsr_hbaB_postqueue_isr(acb);
		if (outbound_doorbell & ARCMSR_IOP2DRV_MESSAGE_CMD_DONE)
			arcmsr_hbaB_message_isr(acb);
		outbound_doorbell = readl(reg->iop2drv_doorbell) &
			acb->outbound_int_enable;
	} while (outbound_doorbell & (ARCMSR_IOP2DRV_DATA_WRITE_OK
	| ARCMSR_IOP2DRV_DATA_READ_OK
	| ARCMSR_IOP2DRV_CDB_DONE
	| ARCMSR_IOP2DRV_MESSAGE_CMD_DONE));
	return IRQ_HANDLED;
}

static irqreturn_t
arcmsr_hbaC_handle_isr(struct AdapterControlBlock *pACB)
{
	uint32_t host_interrupt_status;
	struct MessageUnit_C __iomem *phbcmu =
		(struct MessageUnit_C *)pACB->pmuC;
	host_interrupt_status =
		readl(&phbcmu->host_int_status);
	do {
		if (host_interrupt_status &
			ARCMSR_HBCMU_OUTBOUND_DOORBELL_ISR) {
			arcmsr_hbaC_doorbell_isr(pACB);
		}
		/* MU post queue interrupts*/
		if (host_interrupt_status &
			ARCMSR_HBCMU_OUTBOUND_POSTQUEUE_ISR) {
			arcmsr_hbaC_postqueue_isr(pACB);
		}
		host_interrupt_status = readl(&phbcmu->host_int_status);
	} while (host_interrupt_status &
	(ARCMSR_HBCMU_OUTBOUND_POSTQUEUE_ISR |
	ARCMSR_HBCMU_OUTBOUND_DOORBELL_ISR));
	return IRQ_HANDLED;
}

static irqreturn_t
arcmsr_hbaD_handle_isr(struct AdapterControlBlock *pACB)
{
	u32 host_interrupt_status;
	struct MessageUnit_D __iomem *pmu =
		(struct MessageUnit_D *)pACB->pmuD;
	host_interrupt_status = readl(pmu->host_int_status);
	do {
		/* MU post queue interrupts*/
		if (host_interrupt_status &
			ARCMSR_ARC1214_OUTBOUND_POSTQUEUE_ISR) {
			arcmsr_hbaD_postqueue_isr(pACB);
		}
		if (host_interrupt_status &
			ARCMSR_ARC1214_OUTBOUND_DOORBELL_ISR) {
			arcmsr_hbaD_doorbell_isr(pACB);
		}
		host_interrupt_status = readl(pmu->host_int_status);
	} while (host_interrupt_status &
	(ARCMSR_ARC1214_OUTBOUND_POSTQUEUE_ISR |
	ARCMSR_ARC1214_OUTBOUND_DOORBELL_ISR));
	return IRQ_HANDLED;
}

static irqreturn_t
arcmsr_interrupt(struct AdapterControlBlock *acb)
{
	switch (acb->adapter_type) {
	case ACB_ADAPTER_TYPE_A: {
		return arcmsr_hbaA_handle_isr(acb);
		break;
	}
	case ACB_ADAPTER_TYPE_B: {
		return arcmsr_hbaB_handle_isr(acb);
		break;
	}
	case ACB_ADAPTER_TYPE_C: {
		return arcmsr_hbaC_handle_isr(acb);
		break;
	}
	case ACB_ADAPTER_TYPE_D: {
		return arcmsr_hbaD_handle_isr(acb);
		break;
	}
	default:
		return IRQ_NONE;
	}
}

static void
arcmsr_iop_parking(struct AdapterControlBlock *acb)
{
	if (acb) {
		/* stop adapter background rebuild */
		if (acb->acb_flags & ACB_F_MSG_START_BGRB) {
			uint32_t intmask_org;
			acb->acb_flags &= ~ACB_F_MSG_START_BGRB;
			intmask_org = arcmsr_disable_outbound_ints(acb);
			arcmsr_stop_adapter_bgrb(acb);
			arcmsr_flush_adapter_cache(acb);
			arcmsr_enable_outbound_ints(acb, intmask_org);
		}
	}
}

void
arcmsr_post_ioctldata2iop(struct AdapterControlBlock *acb)
{
	if (acb->acb_flags & ACB_F_MESSAGE_WQBUFFER_READED) {
		if (acb->wqbuf_firstindex != acb->wqbuf_lastindex) {
			uint8_t *pQbuffer;
			uint8_t __iomem *iop_data;
			int32_t allxfer_len = 0;
			struct QBUFFER __iomem *pwbuffer;
			acb->acb_flags &= (~ACB_F_MESSAGE_WQBUFFER_READED);
			pwbuffer = arcmsr_get_iop_wqbuffer(acb);
			iop_data = (uint8_t __iomem *)pwbuffer->data;
			if (acb->wqbuf_firstindex > acb->wqbuf_lastindex) {
				if ((ARCMSR_MAX_QBUFFER -
					acb->wqbuf_firstindex) >= 4) {
					do {
						pQbuffer =
						&acb->wqbuffer[acb->wqbuf_firstindex];
						if (acb->wqbuf_firstindex + 4 >
						ARCMSR_MAX_QBUFFER) {
							memcpy(iop_data, pQbuffer,
							ARCMSR_MAX_QBUFFER - acb->wqbuf_firstindex);
							iop_data += ARCMSR_MAX_QBUFFER
							- acb->wqbuf_firstindex;
							acb->wqbuf_firstindex += 4;
							acb->wqbuf_firstindex %=
							ARCMSR_MAX_QBUFFER;
							memcpy(iop_data,
							&acb->wqbuffer[0],
							acb->wqbuf_firstindex);
							iop_data +=
							acb->wqbuf_firstindex;
						} else {
							if ((acb->wqbuf_lastindex -
							acb->wqbuf_firstindex) > 4) {
								memcpy(iop_data,
								pQbuffer, 4);
								acb->wqbuf_firstindex
								+= 4;
								acb->wqbuf_firstindex
								%= ARCMSR_MAX_QBUFFER;
								iop_data += 4;
							} else {
								memcpy(iop_data,
								pQbuffer,
								acb->wqbuf_lastindex
								- acb->wqbuf_firstindex);
								allxfer_len +=
								acb->wqbuf_lastindex
								- acb->wqbuf_firstindex;
								acb->wqbuf_firstindex
								= acb->wqbuf_lastindex;
								break;
							}
						}
						allxfer_len += 4;
					} while ((acb->wqbuf_firstindex !=
					acb->wqbuf_lastindex) &&
					(allxfer_len < 124));
				} else {
					pQbuffer =
					&acb->wqbuffer[acb->wqbuf_firstindex];
					memcpy(iop_data, pQbuffer,
					ARCMSR_MAX_QBUFFER -
					acb->wqbuf_firstindex);
					iop_data += ARCMSR_MAX_QBUFFER
					- acb->wqbuf_firstindex;
					allxfer_len = ARCMSR_MAX_QBUFFER
					- acb->wqbuf_firstindex;
					acb->wqbuf_firstindex = 0;
					do {
						pQbuffer =
						&acb->wqbuffer[acb->wqbuf_firstindex];
						if ((acb->wqbuf_lastindex -
						acb->wqbuf_firstindex) > 4) {
							memcpy(iop_data,
							pQbuffer, 4);
							acb->wqbuf_firstindex
							+= 4;
							acb->wqbuf_firstindex
							%= ARCMSR_MAX_QBUFFER;
							iop_data += 4;
						} else {
							memcpy(iop_data, pQbuffer,
							acb->wqbuf_lastindex
							- acb->wqbuf_firstindex);
							allxfer_len +=
							acb->wqbuf_lastindex
							- acb->wqbuf_firstindex;
							acb->wqbuf_firstindex =
							acb->wqbuf_lastindex;
							break;
						}
						allxfer_len += 4;
					} while ((acb->wqbuf_firstindex
					!= acb->wqbuf_lastindex) &&
					(allxfer_len < 124));
				}
			} else {
				do {
					pQbuffer =
					&acb->wqbuffer[acb->wqbuf_firstindex];
					if ((acb->wqbuf_lastindex -
					acb->wqbuf_firstindex) > 4) {
						memcpy(iop_data,
						pQbuffer, 4);
						acb->wqbuf_firstindex
						+= 4;
						acb->wqbuf_firstindex
						%= ARCMSR_MAX_QBUFFER;
						iop_data += 4;
					} else {
						memcpy(iop_data, pQbuffer,
						acb->wqbuf_lastindex -
						acb->wqbuf_firstindex);
						allxfer_len += acb->wqbuf_lastindex
						- acb->wqbuf_firstindex;
						acb->wqbuf_firstindex =
						acb->wqbuf_lastindex;
						break;
					}
					allxfer_len += 4;
				} while ((acb->wqbuf_firstindex !=
				acb->wqbuf_lastindex) && (allxfer_len < 124));
			}
			writel(allxfer_len, &pwbuffer->data_len);
			arcmsr_iop_message_wrote(acb);
		}
	}
}

static int
arcmsr_iop_message_xfer(struct AdapterControlBlock *acb,
					struct scsi_cmnd *cmd)
{
	char *buffer;
	unsigned short use_sg;
	int retvalue = 0, transfer_len = 0;
	unsigned long flags;
	struct CMD_MESSAGE_FIELD *pcmdmessagefld;
	uint32_t	controlcode = (uint32_t)cmd->cmnd[5] << 24 |
		(uint32_t)cmd->cmnd[6] << 16 |
		(uint32_t)cmd->cmnd[7] << 8 |
		(uint32_t)cmd->cmnd[8];
	struct scatterlist *sg;

	use_sg = scsi_sg_count(cmd);
	sg = scsi_sglist(cmd);
	buffer = kmap_atomic(sg_page(sg)) + sg->offset;
	if (use_sg > 1) {
		retvalue = ARCMSR_MESSAGE_FAIL;
		goto message_out;
	}
	transfer_len += sg->length;
	if (transfer_len > sizeof(struct CMD_MESSAGE_FIELD)) {
		retvalue = ARCMSR_MESSAGE_FAIL;
		printk("%s: ARCMSR_MESSAGE_FAIL!\n", __func__);
		goto message_out;
	}
	pcmdmessagefld = (struct CMD_MESSAGE_FIELD *)buffer;
	switch (controlcode) {
	case ARCMSR_MESSAGE_READ_RQBUFFER: {
		unsigned char *ver_addr;
		uint8_t *pQbuffer, *ptmpQbuffer;
		uint32_t allxfer_len = 0;
		ver_addr = kmalloc(1032, GFP_ATOMIC);
		if (!ver_addr) {
			retvalue = ARCMSR_MESSAGE_FAIL;
			printk("%s: memory not enough!\n", __func__);
			goto message_out;
		}
		ptmpQbuffer = ver_addr;
		spin_lock_irqsave(&acb->rqbuffer_lock, flags);
		if (acb->rqbuf_firstindex != acb->rqbuf_lastindex) {
			pQbuffer = &acb->rqbuffer[acb->rqbuf_firstindex];
			if (acb->rqbuf_firstindex > acb->rqbuf_lastindex) {
				if ((ARCMSR_MAX_QBUFFER -
					acb->rqbuf_firstindex) >= 1032) {
					memcpy(ptmpQbuffer, pQbuffer, 1032);
					acb->rqbuf_firstindex += 1032;
					allxfer_len = 1032;
				} else {
					if (((ARCMSR_MAX_QBUFFER -
						acb->rqbuf_firstindex) +
						acb->rqbuf_lastindex) > 1032) {
						memcpy(ptmpQbuffer,
						pQbuffer, ARCMSR_MAX_QBUFFER
						- acb->rqbuf_firstindex);
						ptmpQbuffer +=
						ARCMSR_MAX_QBUFFER -
						acb->rqbuf_firstindex;
						memcpy(ptmpQbuffer,
						acb->rqbuffer, 1032 -
						(ARCMSR_MAX_QBUFFER
						- acb->rqbuf_firstindex));
						acb->rqbuf_firstindex =
						1032 - (ARCMSR_MAX_QBUFFER
						- acb->rqbuf_firstindex);
						allxfer_len = 1032;
					} else {
						memcpy(ptmpQbuffer,
						pQbuffer, ARCMSR_MAX_QBUFFER
						- acb->rqbuf_firstindex);
						ptmpQbuffer +=
						ARCMSR_MAX_QBUFFER -
						acb->rqbuf_firstindex;
						memcpy(ptmpQbuffer,
						acb->rqbuffer,
						acb->rqbuf_lastindex);
						allxfer_len = ARCMSR_MAX_QBUFFER
						- acb->rqbuf_firstindex +
						acb->rqbuf_lastindex;
						acb->rqbuf_firstindex =
						acb->rqbuf_lastindex;
					}
				}
			} else {
				if ((acb->rqbuf_lastindex -
				acb->rqbuf_firstindex) > 1032) {
					memcpy(ptmpQbuffer, pQbuffer, 1032);
					acb->rqbuf_firstindex += 1032;
					allxfer_len = 1032;
				} else {
					memcpy(ptmpQbuffer, pQbuffer,
					acb->rqbuf_lastindex - acb->rqbuf_firstindex);
					allxfer_len = acb->rqbuf_lastindex
					- acb->rqbuf_firstindex;
					acb->rqbuf_firstindex =
					acb->rqbuf_lastindex;
				}
			}
		}
		if (acb->acb_flags & ACB_F_IOPDATA_OVERFLOW) {
			struct QBUFFER __iomem *prbuffer;
			uint8_t __iomem *iop_data, *vaddr, *temp;
			uint32_t data_len_residual, data_len, rqbuf_lastindex;
			rqbuf_lastindex = acb->rqbuf_lastindex;
			prbuffer = arcmsr_get_iop_rqbuffer(acb);
			iop_data = (uint8_t __iomem *)prbuffer->data;
			data_len_residual = data_len = readl(&prbuffer->data_len);
			if (data_len > 0) {
				temp = vaddr = kmalloc(data_len, GFP_ATOMIC);
				if (!vaddr) {
					goto leave;
				}
				acb->acb_flags &= ~ACB_F_IOPDATA_OVERFLOW;
				while (data_len_residual >= 4) {
					memcpy(temp, iop_data, 4);
					temp += 4;
					iop_data += 4;
					data_len_residual -= 4;
				}
				if ((data_len_residual > 0) &&
				(data_len_residual < 4)) {
					memcpy(temp, iop_data,
					data_len_residual);
				}
				pQbuffer =
				&acb->rqbuffer[acb->rqbuf_lastindex];
				temp = vaddr;
				if ((rqbuf_lastindex + data_len) >
				ARCMSR_MAX_QBUFFER) {
					memcpy(pQbuffer, temp,
					ARCMSR_MAX_QBUFFER - rqbuf_lastindex);
					temp += (ARCMSR_MAX_QBUFFER -
					rqbuf_lastindex);
					rqbuf_lastindex = (rqbuf_lastindex +
					data_len) % ARCMSR_MAX_QBUFFER;
					memcpy(&acb->rqbuffer[0],
					temp, rqbuf_lastindex);
				} else {
					memcpy(pQbuffer, temp, data_len);
					rqbuf_lastindex =
					(rqbuf_lastindex + data_len) %
					ARCMSR_MAX_QBUFFER;
				}
				kfree(vaddr);
				acb->rqbuf_lastindex = rqbuf_lastindex;
				arcmsr_iop_message_read(acb);
			}
		}
		leave:
		spin_unlock_irqrestore(&acb->rqbuffer_lock, flags);
		memcpy(pcmdmessagefld->messagedatabuffer, ver_addr, allxfer_len);
		pcmdmessagefld->cmdmessage.Length = allxfer_len;
		if (acb->fw_flag == FW_DEADLOCK) {
			pcmdmessagefld->cmdmessage.ReturnCode =
			ARCMSR_MESSAGE_RETURNCODE_BUS_HANG_ON;
		} else {
			pcmdmessagefld->cmdmessage.ReturnCode =
			ARCMSR_MESSAGE_RETURNCODE_OK;
		}
		kfree(ver_addr);
		break;
	}
	case ARCMSR_MESSAGE_WRITE_WQBUFFER: {
		unsigned char *ver_addr;
		int32_t my_empty_len, user_len, wqbuf_firstindex,
		wqbuf_lastindex;
		uint8_t *pQbuffer, *ptmpuserbuffer;
		ver_addr = kmalloc(1032, GFP_ATOMIC);
		if (!ver_addr) {
			retvalue = ARCMSR_MESSAGE_FAIL;
			goto message_out;
		}
		if (acb->fw_flag == FW_DEADLOCK) {
			pcmdmessagefld->cmdmessage.ReturnCode =
			ARCMSR_MESSAGE_RETURNCODE_BUS_HANG_ON;
		} else {
			pcmdmessagefld->cmdmessage.ReturnCode =
			ARCMSR_MESSAGE_RETURNCODE_OK;
		}
		ptmpuserbuffer = ver_addr;
		user_len = pcmdmessagefld->cmdmessage.Length;
		memcpy(ptmpuserbuffer, pcmdmessagefld->messagedatabuffer,
		user_len);
		spin_lock_irqsave(&acb->wqbuffer_lock, flags);
		wqbuf_lastindex = acb->wqbuf_lastindex;
		wqbuf_firstindex = acb->wqbuf_firstindex;
		if (wqbuf_lastindex != wqbuf_firstindex) {
			struct SENSE_DATA *sensebuffer =
			(struct SENSE_DATA *)cmd->sense_buffer;
			arcmsr_post_ioctldata2iop(acb);
			/* has error report sensedata */
			sensebuffer->ErrorCode = SCSI_SENSE_CURRENT_ERRORS;
			sensebuffer->SenseKey = ILLEGAL_REQUEST;
			sensebuffer->AdditionalSenseLength = 0x0A;
			sensebuffer->AdditionalSenseCode = 0x20;
			sensebuffer->Valid = 1;
			retvalue = ARCMSR_MESSAGE_FAIL;
		} else {
			my_empty_len = (wqbuf_firstindex - wqbuf_lastindex - 1)
			& (ARCMSR_MAX_QBUFFER - 1);
			if (my_empty_len >= user_len) {
				while (user_len > 0) {
					pQbuffer = &acb->wqbuffer[acb->wqbuf_lastindex];
					if ((acb->wqbuf_lastindex + user_len)
						> ARCMSR_MAX_QBUFFER) {
						memcpy(pQbuffer, ptmpuserbuffer,
						ARCMSR_MAX_QBUFFER -
						acb->wqbuf_lastindex);
						ptmpuserbuffer += (ARCMSR_MAX_QBUFFER
						- acb->wqbuf_lastindex);
						user_len -= (ARCMSR_MAX_QBUFFER
						- acb->wqbuf_lastindex);
						acb->wqbuf_lastindex = 0;
					} else {
						memcpy(pQbuffer, ptmpuserbuffer,
						user_len);
						acb->wqbuf_lastindex += user_len;
						acb->wqbuf_lastindex %=
						ARCMSR_MAX_QBUFFER;
						user_len = 0;
					}
				}
				if (acb->acb_flags &
				ACB_F_MESSAGE_WQBUFFER_CLEARED) {
					acb->acb_flags &=
					~ACB_F_MESSAGE_WQBUFFER_CLEARED;
					arcmsr_post_ioctldata2iop(acb);
				}
			} else {
				struct SENSE_DATA *sensebuffer =
				(struct SENSE_DATA *)cmd->sense_buffer;
				/* has error report sensedata */
				sensebuffer->ErrorCode =
				SCSI_SENSE_CURRENT_ERRORS;
				sensebuffer->SenseKey = ILLEGAL_REQUEST;
				sensebuffer->AdditionalSenseLength = 0x0A;
				sensebuffer->AdditionalSenseCode = 0x20;
				sensebuffer->Valid = 1;
				retvalue = ARCMSR_MESSAGE_FAIL;
			}
		}
		spin_unlock_irqrestore(&acb->wqbuffer_lock, flags);
		kfree(ver_addr);
		break;
	}
	case ARCMSR_MESSAGE_CLEAR_RQBUFFER: {
		uint8_t *pQbuffer = acb->rqbuffer;

		spin_lock_irqsave(&acb->rqbuffer_lock, flags);
		if (acb->acb_flags & ACB_F_IOPDATA_OVERFLOW) {
			acb->acb_flags &= ~ACB_F_IOPDATA_OVERFLOW;
			arcmsr_iop_message_read(acb);
		}
		acb->acb_flags |= ACB_F_MESSAGE_RQBUFFER_CLEARED;
		acb->rqbuf_firstindex = 0;
		acb->rqbuf_lastindex = 0;
		memset(pQbuffer, 0, ARCMSR_MAX_QBUFFER);
		spin_unlock_irqrestore(&acb->rqbuffer_lock, flags);
		if (acb->fw_flag == FW_DEADLOCK) {
			pcmdmessagefld->cmdmessage.ReturnCode =
			ARCMSR_MESSAGE_RETURNCODE_BUS_HANG_ON;
		} else {
			pcmdmessagefld->cmdmessage.ReturnCode =
			ARCMSR_MESSAGE_RETURNCODE_OK;
		}
		break;
	}
	case ARCMSR_MESSAGE_CLEAR_WQBUFFER: {
		uint8_t *pQbuffer = acb->wqbuffer;
		if (acb->fw_flag == FW_DEADLOCK) {
			pcmdmessagefld->cmdmessage.ReturnCode =
			ARCMSR_MESSAGE_RETURNCODE_BUS_HANG_ON;
		} else {
			pcmdmessagefld->cmdmessage.ReturnCode =
			ARCMSR_MESSAGE_RETURNCODE_OK;
		}
		spin_lock_irqsave(&acb->wqbuffer_lock, flags);
		if (acb->acb_flags & ACB_F_IOPDATA_OVERFLOW) {
			acb->acb_flags &= ~ACB_F_IOPDATA_OVERFLOW;
			arcmsr_iop_message_read(acb);
		}
		acb->acb_flags |= (ACB_F_MESSAGE_WQBUFFER_CLEARED |
		ACB_F_MESSAGE_WQBUFFER_READED);
		acb->wqbuf_firstindex = 0;
		acb->wqbuf_lastindex = 0;
		memset(pQbuffer, 0, ARCMSR_MAX_QBUFFER);
		spin_unlock_irqrestore(&acb->wqbuffer_lock, flags);
		break;
	}
	case ARCMSR_MESSAGE_CLEAR_ALLQBUFFER: {
		uint8_t *pQbuffer;
		if (acb->acb_flags & ACB_F_IOPDATA_OVERFLOW) {
			acb->acb_flags &= ~ACB_F_IOPDATA_OVERFLOW;
			arcmsr_iop_message_read(acb);
		}
		spin_lock_irqsave(&acb->rqbuffer_lock, flags);
		acb->acb_flags |= ACB_F_MESSAGE_RQBUFFER_CLEARED;
		acb->rqbuf_firstindex = 0;
		acb->rqbuf_lastindex = 0;
		pQbuffer = acb->rqbuffer;
		memset(pQbuffer, 0, sizeof(struct QBUFFER));
		spin_unlock_irqrestore(&acb->rqbuffer_lock, flags);
		spin_lock_irqsave(&acb->wqbuffer_lock, flags);
		acb->acb_flags |= (ACB_F_MESSAGE_WQBUFFER_CLEARED |
		ACB_F_MESSAGE_WQBUFFER_READED);
		acb->wqbuf_firstindex = 0;
		acb->wqbuf_lastindex = 0;
		pQbuffer = acb->wqbuffer;
		memset(pQbuffer, 0, sizeof(struct QBUFFER));
		spin_unlock_irqrestore(&acb->wqbuffer_lock, flags);
		if (acb->fw_flag == FW_DEADLOCK) {
			pcmdmessagefld->cmdmessage.ReturnCode =
			ARCMSR_MESSAGE_RETURNCODE_BUS_HANG_ON;
		} else {
			pcmdmessagefld->cmdmessage.ReturnCode =
			ARCMSR_MESSAGE_RETURNCODE_OK;
		}
		break;
	}
	case ARCMSR_MESSAGE_RETURN_CODE_3F: {
		if (acb->fw_flag == FW_DEADLOCK) {
			pcmdmessagefld->cmdmessage.ReturnCode =
			ARCMSR_MESSAGE_RETURNCODE_BUS_HANG_ON;
		} else {
			pcmdmessagefld->cmdmessage.ReturnCode =
			ARCMSR_MESSAGE_RETURNCODE_3F;
		}
		break;
	}
	case ARCMSR_MESSAGE_SAY_HELLO: {
		int8_t *hello_string = "Hello! I am ARCMSR";
		if (acb->fw_flag == FW_DEADLOCK) {
			pcmdmessagefld->cmdmessage.ReturnCode =
			ARCMSR_MESSAGE_RETURNCODE_BUS_HANG_ON;
		} else {
			pcmdmessagefld->cmdmessage.ReturnCode =
			ARCMSR_MESSAGE_RETURNCODE_OK;
		}
		memcpy(pcmdmessagefld->messagedatabuffer,
		hello_string, (int16_t)strlen(hello_string));
		break;
	}
	case ARCMSR_MESSAGE_SAY_GOODBYE: {
		if (acb->fw_flag == FW_DEADLOCK) {
			pcmdmessagefld->cmdmessage.ReturnCode =
			ARCMSR_MESSAGE_RETURNCODE_BUS_HANG_ON;
		} else {
			pcmdmessagefld->cmdmessage.ReturnCode =
			ARCMSR_MESSAGE_RETURNCODE_OK;
		}
		arcmsr_iop_parking(acb);
		break;
	}
	case ARCMSR_MESSAGE_FLUSH_ADAPTER_CACHE: {
		if (acb->fw_flag == FW_DEADLOCK) {
			pcmdmessagefld->cmdmessage.ReturnCode =
			ARCMSR_MESSAGE_RETURNCODE_BUS_HANG_ON;
		} else {
			pcmdmessagefld->cmdmessage.ReturnCode =
			ARCMSR_MESSAGE_RETURNCODE_OK;
		}
		arcmsr_flush_adapter_cache(acb);
		break;
	}
	default:
		retvalue = ARCMSR_MESSAGE_FAIL;
		printk("unknown controlcode(%d)\n", __LINE__);
	}
	message_out:
	if (use_sg) {
		struct scatterlist *sg;
		sg = scsi_sglist(cmd);
		kunmap_atomic(buffer - sg->offset);
	}
	return retvalue;
}

struct CommandControlBlock
*arcmsr_get_freeccb(struct AdapterControlBlock *acb)
{
	struct list_head *head = &acb->ccb_free_list;
	struct CommandControlBlock *ccb = NULL;
	unsigned long flags;
	spin_lock_irqsave(&acb->ccblist_lock, flags);
	if (!list_empty(head)) {
		ccb = list_entry(head->next,
			struct CommandControlBlock, list);
		list_del_init(&ccb->list);
	} else {
		spin_unlock_irqrestore(&acb->ccblist_lock, flags);
		return 0;
	}
	spin_unlock_irqrestore(&acb->ccblist_lock, flags);
	return ccb;
}

void
arcmsr_handle_virtual_command(struct AdapterControlBlock *acb,
		struct scsi_cmnd *cmd)
{
	switch (cmd->cmnd[0]) {
	case INQUIRY: {
		unsigned char inqdata[36];
		char *buffer;
		struct scatterlist *sg;
		if (cmd->device->lun) {
			cmd->result = (DID_TIME_OUT << 16);
			cmd->scsi_done(cmd);
			return;
		}
		inqdata[0] = TYPE_PROCESSOR;
		/* Periph Qualifier & Periph Dev Type */
		inqdata[1] = 0;
		/* rem media bit & Dev Type Modifier */
		inqdata[2] = 0;
		/* ISO, ECMA, & ANSI versions */
		inqdata[4] = 31;
		/* length of additional data */
		strncpy(&inqdata[8], "Areca   ", 8);
		/* Vendor Identification */
		strncpy(&inqdata[16], "RAID controller ", 16);
		/* Product Identification */
		strncpy(&inqdata[32], "R001", 4); /* Product Revision */
		sg = scsi_sglist(cmd);
		buffer = kmap_atomic(sg_page(sg)) + sg->offset;
		memcpy(buffer, inqdata, sizeof(inqdata));
		sg = scsi_sglist(cmd);
		kunmap_atomic(buffer - sg->offset);
		cmd->scsi_done(cmd);
	}
	break;
	case WRITE_BUFFER:
	case READ_BUFFER: {
		if (arcmsr_iop_message_xfer(acb, cmd))
			cmd->result = (DID_ERROR << 16);
		cmd->scsi_done(cmd);
	}
	break;
	default:
		cmd->scsi_done(cmd);
	}
}

static int
arcmsr_queue_command_lck(struct scsi_cmnd *cmd,
	void (* done)(struct scsi_cmnd *))
{
	struct Scsi_Host *host = cmd->device->host;
	struct AdapterControlBlock *acb =
	(struct AdapterControlBlock *)host->hostdata;
	struct CommandControlBlock *ccb;
	int target = cmd->device->id;
	int lun = cmd->device->lun;
	uint8_t scsicmd = cmd->cmnd[0];
	cmd->scsi_done = done;
	cmd->host_scribble = NULL;
	cmd->result = 0;
	if ((scsicmd == SYNCHRONIZE_CACHE) ||
		(scsicmd == SEND_DIAGNOSTIC)) {
		if (acb->devstate[target][lun] ==
			ARECA_RAID_GONE) {
    			cmd->result = (DID_NO_CONNECT << 16);
		}
		cmd->scsi_done(cmd);
		return 0;
	}
	if (target == 16) {
		/* virtual device for iop message transfer */
		arcmsr_handle_virtual_command(acb, cmd);
		return 0;
	}
	if (atomic_read(&acb->ccboutstandingcount) >=
	acb->maxOutstanding)
		return SCSI_MLQUEUE_HOST_BUSY;
	ccb = arcmsr_get_freeccb(acb);
	if (!ccb)
		return SCSI_MLQUEUE_HOST_BUSY;
	if (arcmsr_build_ccb( acb, ccb, cmd ) == FAILED) {
		cmd->result = (DID_ERROR << 16) |
			(RESERVATION_CONFLICT << 1);
		cmd->scsi_done(cmd);
		return 0;
	}
	arcmsr_post_ccb(acb, ccb);
	return 0;
}

static DEF_SCSI_QCMD(arcmsr_queue_command)

static bool
arcmsr_hbaA_get_config(struct AdapterControlBlock *acb)
{
	struct MessageUnit_A __iomem *reg = acb->pmuA;
	char *acb_firm_model = acb->firm_model;
	char *acb_firm_version = acb->firm_version;
	char *acb_device_map = acb->device_map;
	char __iomem *iop_firm_model =
		(char __iomem *)(&reg->message_rwbuffer[15]);
	char __iomem *iop_firm_version =
		(char __iomem *)(&reg->message_rwbuffer[17]);
	char __iomem *iop_device_map =
		(char __iomem *)(&reg->message_rwbuffer[21]);
	int count;
	writel(ARCMSR_INBOUND_MESG0_GET_CONFIG,
		&reg->inbound_msgaddr0);
	if (!arcmsr_hbaA_wait_msgint_ready(acb)) {
		pr_notice("arcmsr%d: wait 'get adapter firmware "
		"miscellaneous data' timeout\n",
		acb->host->host_no);
		return false;
	}
	count = 8;
	while (count) {
		*acb_firm_model = readb(iop_firm_model);
		acb_firm_model++;
		iop_firm_model++;
		count--;
	}

	count = 16;
	while (count) {
		*acb_firm_version = readb(iop_firm_version);
		acb_firm_version++;
		iop_firm_version++;
		count--;
	}

	count=16;
	while (count) {
		*acb_device_map = readb(iop_device_map);
		acb_device_map++;
		iop_device_map++;
		count--;
	}
	pr_notice("Areca RAID Controller%d: F/W %s "
		"& Model %s\n",
		acb->host->host_no,
		acb->firm_version,
		acb->firm_model);
	acb->signature = readl(&reg->message_rwbuffer[0]);
	acb->firm_request_len = readl(&reg->message_rwbuffer[1]);
	acb->firm_numbers_queue = readl(&reg->message_rwbuffer[2]);
	acb->firm_sdram_size = readl(&reg->message_rwbuffer[3]);
	acb->firm_hd_channels = readl(&reg->message_rwbuffer[4]);
	acb->firm_cfg_version = readl(&reg->message_rwbuffer[25]);
	return true;
}

static bool
arcmsr_hbaB_get_config(struct AdapterControlBlock *acb)
{
	struct MessageUnit_B *reg = acb->pmuB;
	struct pci_dev *pdev = acb->pdev;
	void *dma_coherent;
	dma_addr_t dma_coherent_handle;
	char *acb_firm_model = acb->firm_model;
	char *acb_firm_version = acb->firm_version;
	char *acb_device_map = acb->device_map;
	char __iomem *iop_firm_model;
	/*firm_model,15,60-67*/
	char __iomem *iop_firm_version;
	/*firm_version,17,68-83*/
	char __iomem *iop_device_map;
	/*firm_version,21,84-99*/
	int count;
	dma_coherent = dma_alloc_coherent(&pdev->dev,
		sizeof(struct MessageUnit_B), &dma_coherent_handle,
		GFP_KERNEL);
	if (!dma_coherent) {
		pr_notice("arcmsr%d: dma_alloc_coherent "
		"got error for hbb mu\n", acb->host->host_no);
		return false;
	}
	acb->dma_coherent_handle2 = dma_coherent_handle;
	reg = (struct MessageUnit_B *)dma_coherent;
	acb->pmuB = reg;
	reg->drv2iop_doorbell = (uint32_t __iomem *)
		((unsigned long)acb->mem_base0 +
		ARCMSR_DRV2IOP_DOORBELL);
	reg->drv2iop_doorbell_mask = (uint32_t __iomem *)
		((unsigned long)acb->mem_base0 +
		ARCMSR_DRV2IOP_DOORBELL_MASK);
	reg->iop2drv_doorbell = (uint32_t __iomem *)
		((unsigned long)acb->mem_base0 +
		ARCMSR_IOP2DRV_DOORBELL);
	reg->iop2drv_doorbell_mask = (uint32_t __iomem *)
		((unsigned long)acb->mem_base0 +
		ARCMSR_IOP2DRV_DOORBELL_MASK);
	reg->message_wbuffer = (uint32_t __iomem *)
		((unsigned long)acb->mem_base1 +
		ARCMSR_MESSAGE_WBUFFER);
	reg->message_rbuffer = (uint32_t __iomem *)
		((unsigned long)acb->mem_base1 +
		ARCMSR_MESSAGE_RBUFFER);
	reg->message_rwbuffer = (uint32_t __iomem *)
		((unsigned long)acb->mem_base1 +
		ARCMSR_MESSAGE_RWBUFFER);
	iop_firm_model = (char __iomem *)(&reg->message_rwbuffer[15]);
	iop_firm_version = (char __iomem *)(&reg->message_rwbuffer[17]);
	iop_device_map = (char __iomem *)(&reg->message_rwbuffer[21]);

	writel(ARCMSR_MESSAGE_GET_CONFIG, reg->drv2iop_doorbell);
	if (!arcmsr_hbaB_wait_msgint_ready(acb)) {
		pr_notice("arcmsr%d: wait 'get adapter firmware "
		"miscellaneous data' timeout\n", acb->host->host_no);
		return false;
	}
	count = 8;
	while (count) {
		*acb_firm_model = readb(iop_firm_model);
		acb_firm_model++;
		iop_firm_model++;
		count--;
	}
	count = 16;
	while (count) {
		*acb_firm_version = readb(iop_firm_version);
		acb_firm_version++;
		iop_firm_version++;
		count--;
	}

	count = 16;
	while (count) {
		*acb_device_map = readb(iop_device_map);
		acb_device_map++;
		iop_device_map++;
		count--;
	}

	pr_notice("Areca RAID Controller%d: "
		"F/W %s & Model %s\n",
		acb->host->host_no,
		acb->firm_version,
		acb->firm_model);

	acb->signature = readl(&reg->message_rwbuffer[1]);
	/*firm_signature,1,00-03*/
	acb->firm_request_len = readl(&reg->message_rwbuffer[2]);
	/*firm_request_len,1,04-07*/
	acb->firm_numbers_queue = readl(&reg->message_rwbuffer[3]);
	/*firm_numbers_queue,2,08-11*/
	acb->firm_sdram_size = readl(&reg->message_rwbuffer[4]);
	/*firm_sdram_size,3,12-15*/
	acb->firm_hd_channels = readl(&reg->message_rwbuffer[5]);
	/*firm_ide_channels,4,16-19*/
	acb->firm_cfg_version = readl(&reg->message_rwbuffer[25]);
	/*firm_ide_channels,4,16-19*/
	return true;
}

static bool
arcmsr_hbaC_get_config(struct AdapterControlBlock *pACB)
{
	uint32_t intmask_org, Index, firmware_state = 0;
	struct MessageUnit_C *reg = pACB->pmuC;
	char *acb_firm_model = pACB->firm_model;
	char *acb_firm_version = pACB->firm_version;
	char *iop_firm_model = (char *)(&reg->msgcode_rwbuffer[15]);
	char *iop_firm_version = (char *)(&reg->msgcode_rwbuffer[17]);
	int count;
	/* disable all outbound interrupt */
	intmask_org = readl(&reg->host_int_mask);
	writel(intmask_org | ARCMSR_HBCMU_ALL_INTMASKENABLE,
		&reg->host_int_mask);
	do {
		firmware_state = readl(&reg->outbound_msgaddr1);
	} while ((firmware_state & ARCMSR_HBCMU_MESSAGE_FIRMWARE_OK) == 0);
	/* post "get config" instruction */
	writel(ARCMSR_INBOUND_MESG0_GET_CONFIG, &reg->inbound_msgaddr0);
	writel(ARCMSR_HBCMU_DRV2IOP_MESSAGE_CMD_DONE,
		&reg->inbound_doorbell);
	/* wait message ready */
	for (Index = 0; Index < 2000; Index++) {
		if (readl(&reg->outbound_doorbell) &
		ARCMSR_HBCMU_IOP2DRV_MESSAGE_CMD_DONE) {
			writel(ARCMSR_HBCMU_IOP2DRV_MESSAGE_CMD_DONE_DOORBELL_CLEAR,
				&reg->outbound_doorbell_clear);
			break;
		}
		udelay(10);
	} /*max 1 seconds*/
	if (Index >= 2000) {
		pr_notice("arcmsr%d: wait 'get adapter firmware "
		"miscellaneous data' timeout\n", pACB->host->host_no);
		return false;
	}
	count = 8;
	while (count) {
		*acb_firm_model = readb(iop_firm_model);
		acb_firm_model++;
		iop_firm_model++;
		count--;
	}
	count = 16;
	while (count) {
		*acb_firm_version = readb(iop_firm_version);
		acb_firm_version++;
		iop_firm_version++;
		count--;
	}
	pr_notice("Areca RAID Controller%d: F/W %s & "
	"Model %s\n",
	pACB->host->host_no,
	pACB->firm_version,
	pACB->firm_model);
	pACB->firm_request_len = readl(&reg->msgcode_rwbuffer[1]);
	pACB->firm_numbers_queue = readl(&reg->msgcode_rwbuffer[2]);
	pACB->firm_sdram_size = readl(&reg->msgcode_rwbuffer[3]);
	pACB->firm_hd_channels = readl(&reg->msgcode_rwbuffer[4]);
	pACB->firm_cfg_version = readl(&reg->msgcode_rwbuffer[25]);
	/*all interrupt service will be enable at arcmsr_iop_init*/
	return true;
}

static bool
arcmsr_hbaD_get_config(struct AdapterControlBlock *acb)
{
	char *acb_firm_model = acb->firm_model;
	char *acb_firm_version = acb->firm_version;
	char *acb_device_map = acb->device_map;
	char __iomem *iop_firm_model;
	char __iomem *iop_firm_version;
	char __iomem *iop_device_map;
	u32 count;
	struct MessageUnit_D *reg ;
	void *dma_coherent;
	dma_addr_t dma_coherent_handle;
	struct pci_dev *pdev = acb->pdev;

	acb->uncache_size = roundup(sizeof(struct MessageUnit_D), 32);
	dma_coherent = dma_alloc_coherent(&pdev->dev, acb->uncache_size,
	&dma_coherent_handle, GFP_KERNEL);
	if (!dma_coherent) {
		pr_notice("DMA allocation failed...\n");
		return -ENOMEM;
	}
	memset(dma_coherent, 0, acb->uncache_size);
	acb->dma_coherent = dma_coherent;
	acb->dma_coherent_handle = dma_coherent_handle;
	reg = (struct MessageUnit_D *)dma_coherent;
	acb->pmuD = reg;
	reg->chip_id = (u32 __iomem *)((unsigned long)
	acb->mem_base0 + ARCMSR_ARC1214_CHIP_ID);
	reg->cpu_mem_config = (u32 __iomem *)((unsigned long)
	acb->mem_base0 + ARCMSR_ARC1214_CPU_MEMORY_CONFIGURATION);
	reg->i2o_host_interrupt_mask = (u32 __iomem *)((unsigned long)
	acb->mem_base0 + ARCMSR_ARC1214_I2_HOST_INTERRUPT_MASK);
	reg->sample_at_reset = (u32 __iomem *)((unsigned long)
	acb->mem_base0 + ARCMSR_ARC1214_SAMPLE_RESET);
	reg->reset_request = (u32 __iomem *)((unsigned long)
	acb->mem_base0 + ARCMSR_ARC1214_RESET_REQUEST);
	reg->host_int_status = (u32 __iomem *)((unsigned long)
	acb->mem_base0 + ARCMSR_ARC1214_MAIN_INTERRUPT_STATUS);
	reg->pcief0_int_enable = (u32 __iomem *)((unsigned long)
	acb->mem_base0 + ARCMSR_ARC1214_PCIE_F0_INTERRUPT_ENABLE);
	reg->inbound_msgaddr0 = (u32 __iomem *)((unsigned long)
	acb->mem_base0 + ARCMSR_ARC1214_INBOUND_MESSAGE0);
	reg->inbound_msgaddr1 = (u32 __iomem *)((unsigned long)
	acb->mem_base0 + ARCMSR_ARC1214_INBOUND_MESSAGE1);
	reg->outbound_msgaddr0 = (u32 __iomem *)((unsigned long)
	acb->mem_base0 + ARCMSR_ARC1214_OUTBOUND_MESSAGE0);
	reg->outbound_msgaddr1 = (u32 __iomem *)((unsigned long)
	acb->mem_base0 + ARCMSR_ARC1214_OUTBOUND_MESSAGE1);
	reg->inbound_doorbell = (u32 __iomem *)((unsigned long)
	acb->mem_base0 + ARCMSR_ARC1214_INBOUND_DOORBELL);
	reg->outbound_doorbell = (u32 __iomem *)((unsigned long)
	acb->mem_base0 + ARCMSR_ARC1214_OUTBOUND_DOORBELL);
	reg->outbound_doorbell_enable = (u32 __iomem *)((unsigned long)
	acb->mem_base0 + ARCMSR_ARC1214_OUTBOUND_DOORBELL_ENABLE);
	reg->inboundlist_base_low = (u32 __iomem *)((unsigned long)
	acb->mem_base0 + ARCMSR_ARC1214_INBOUND_LIST_BASE_LOW);
	reg->inboundlist_base_high = (u32 __iomem *)((unsigned long)
	acb->mem_base0 + ARCMSR_ARC1214_INBOUND_LIST_BASE_HIGH);
	reg->inboundlist_write_pointer = (u32 __iomem *)((unsigned long)
	acb->mem_base0 + ARCMSR_ARC1214_INBOUND_LIST_WRITE_POINTER);
	reg->outboundlist_base_low = (u32 __iomem *)((unsigned long)
	acb->mem_base0 + ARCMSR_ARC1214_OUTBOUND_LIST_BASE_LOW);
	reg->outboundlist_base_high = (u32 __iomem *)((unsigned long)
	acb->mem_base0 + ARCMSR_ARC1214_OUTBOUND_LIST_BASE_HIGH);
	reg->outboundlist_copy_pointer = (u32 __iomem *)((unsigned long)
	acb->mem_base0 + ARCMSR_ARC1214_OUTBOUND_LIST_COPY_POINTER);
	reg->outboundlist_read_pointer = (u32 __iomem *)((unsigned long)
	acb->mem_base0 + ARCMSR_ARC1214_OUTBOUND_LIST_READ_POINTER);
	reg->outboundlist_interrupt_cause = (u32 __iomem *)((unsigned long)
	acb->mem_base0 + ARCMSR_ARC1214_OUTBOUND_INTERRUPT_CAUSE);
	reg->outboundlist_interrupt_enable = (u32 __iomem *)((unsigned long)
	acb->mem_base0 + ARCMSR_ARC1214_OUTBOUND_INTERRUPT_ENABLE);
	reg->message_wbuffer = (u32 __iomem *)((unsigned long)
	acb->mem_base0 + ARCMSR_ARC1214_MESSAGE_WBUFFER);
	reg->message_rbuffer = (u32 __iomem *)((unsigned long)
	acb->mem_base0 + ARCMSR_ARC1214_MESSAGE_RBUFFER);
	reg->msgcode_rwbuffer = (u32 __iomem *)((unsigned long)
	acb->mem_base0 + ARCMSR_ARC1214_MESSAGE_RWBUFFER);
	iop_firm_model = (char __iomem *)(&reg->msgcode_rwbuffer[15]);
	iop_firm_version = (char __iomem *)(&reg->msgcode_rwbuffer[17]);
	iop_device_map = (char __iomem *)(&reg->msgcode_rwbuffer[21]);
	if (readl(acb->pmuD->outbound_doorbell) &
	ARCMSR_ARC1214_IOP2DRV_MESSAGE_CMD_DONE) {
		writel(ARCMSR_ARC1214_IOP2DRV_MESSAGE_CMD_DONE,
			acb->pmuD->outbound_doorbell);/*clear interrupt*/
	}
	/* post "get config" instruction */
	writel(ARCMSR_INBOUND_MESG0_GET_CONFIG, reg->inbound_msgaddr0);
	/* wait message ready */
	if (!arcmsr_hbaD_wait_msgint_ready(acb)) {
		pr_notice("arcmsr%d: wait get adapter firmware "
		"miscellaneous data timeout\n", acb->host->host_no);
		dma_free_coherent(&acb->pdev->dev, acb->uncache_size,
			acb->dma_coherent, acb->dma_coherent_handle);
		return false;
	}
	count = 8;
	while (count) {
		*acb_firm_model = readb(iop_firm_model);
		acb_firm_model++;
		iop_firm_model++;
		count--;
	}
	count = 16;
	while (count) {
		*acb_firm_version = readb(iop_firm_version);
		acb_firm_version++;
		iop_firm_version++;
		count--;
	}
	count = 16;
	while (count) {
		*acb_device_map = readb(iop_device_map);
		acb_device_map++;
		iop_device_map++;
		count--;
	}
	acb->signature = readl(&reg->msgcode_rwbuffer[1]);
	/*firm_signature,1,00-03*/
	acb->firm_request_len = readl(&reg->msgcode_rwbuffer[2]);
	/*firm_request_len,1,04-07*/
	acb->firm_numbers_queue = readl(&reg->msgcode_rwbuffer[3]);
	/*firm_numbers_queue,2,08-11*/
	acb->firm_sdram_size = readl(&reg->msgcode_rwbuffer[4]);
	/*firm_sdram_size,3,12-15*/
	acb->firm_hd_channels = readl(&reg->msgcode_rwbuffer[5]);
	/*firm_hd_channels,4,16-19*/
	acb->firm_cfg_version = readl(&reg->msgcode_rwbuffer[25]);
	pr_notice("Areca RAID Controller%d: F/W %s & Model %s\n",
	acb->host->host_no, acb->firm_version, acb->firm_model);
	return true;
}

static bool
arcmsr_get_firmware_spec(struct AdapterControlBlock *acb)
{
	bool rtn = false;
	switch (acb->adapter_type) {
	case ACB_ADAPTER_TYPE_A:
		rtn = arcmsr_hbaA_get_config(acb);
		break;
	case ACB_ADAPTER_TYPE_B:
		rtn = arcmsr_hbaB_get_config(acb);
		break;
	case ACB_ADAPTER_TYPE_C:
		rtn = arcmsr_hbaC_get_config(acb);
		break;
	case ACB_ADAPTER_TYPE_D:
		rtn = arcmsr_hbaD_get_config(acb);
		break;
	default:
		break;
	}
	if(acb->firm_numbers_queue > ARCMSR_MAX_FREECCB_NUM)
		acb->maxOutstanding = ARCMSR_MAX_FREECCB_NUM-1;
	else
		acb->maxOutstanding = acb->firm_numbers_queue - 1;
	return rtn;
}

static int
arcmsr_hbaA_polling_ccbdone(struct AdapterControlBlock *acb,
	struct CommandControlBlock *poll_ccb)
{
	struct MessageUnit_A __iomem *reg = acb->pmuA;
	struct CommandControlBlock *ccb;
	struct ARCMSR_CDB *arcmsr_cdb;
	uint32_t flag_ccb, outbound_intstatus, poll_ccb_done = 0;
	uint32_t poll_count = 0;
	int rtn;
	bool error;
	polling_hba_ccb_retry:
	poll_count++;
	outbound_intstatus = readl(&reg->outbound_intstatus) &
		acb->outbound_int_enable;
	writel(outbound_intstatus, &reg->outbound_intstatus);
	while (1) {
		flag_ccb = readl(&reg->outbound_queueport);
		if (flag_ccb == 0xFFFFFFFF) {
			if (poll_ccb_done) {
				rtn = SUCCESS;
				break;
			} else {
				msleep(25);
				if (poll_count > 100) {
					rtn = FAILED;
					break;
				}
				goto polling_hba_ccb_retry;
			}
		}
		arcmsr_cdb = (struct ARCMSR_CDB *)(acb->vir2phy_offset +
			(flag_ccb << 5));
		ccb = container_of(arcmsr_cdb, struct CommandControlBlock,
			arcmsr_cdb);
		poll_ccb_done = (ccb == poll_ccb) ? 1:0;
		if ((ccb->acb != acb) || (ccb->startdone != ARCMSR_CCB_START)) {
			if ((ccb->startdone == ARCMSR_CCB_ABORTED) ||
			(ccb == poll_ccb)) {
				pr_notice("arcmsr%d: scsi id = %d "
				"lun = %d ccb = '0x%p' poll command "
				"abort successfully\n"
				, acb->host->host_no
				, ccb->pcmd->device->id
				, ccb->pcmd->device->lun
				, ccb);
				ccb->pcmd->result = DID_ABORT << 16;
				arcmsr_ccb_complete(ccb);
				continue;
			}
			pr_notice("arcmsr%d: polling get an illegal "
			"ccb command done ccb = '0x%p' "
			"ccboutstandingcount = %d\n"
			, acb->host->host_no
			, ccb
			, atomic_read(&acb->ccboutstandingcount));
			continue;
		}
		error = (flag_ccb & ARCMSR_CCBREPLY_FLAG_ERROR_MODE0) ?
			true : false;
		arcmsr_report_ccb_state(acb, ccb, error);
	}
	return rtn;
}

int
arcmsr_hbaB_polling_ccbdone(struct AdapterControlBlock *acb,
				struct CommandControlBlock *poll_ccb)
{
	struct MessageUnit_B *reg = acb->pmuB;
	struct ARCMSR_CDB *arcmsr_cdb;
	struct CommandControlBlock *ccb;
	uint32_t flag_ccb, poll_ccb_done = 0, poll_count = 0;
	int index, rtn;
	bool error;
	polling_hbb_ccb_retry:

	poll_count++;
	/* clear doorbell interrupt */
	writel(ARCMSR_DOORBELL_INT_CLEAR_PATTERN,
	reg->iop2drv_doorbell);
	while (1) {
		index = reg->doneq_index;
		flag_ccb = readl(&reg->done_qbuffer[index]);
		if (flag_ccb == 0) {
			if (poll_ccb_done) {
				rtn = SUCCESS;
				break;
			} else {
				msleep(25);
				if (poll_count > 100) {
					rtn = FAILED;
					break;
				}
				goto polling_hbb_ccb_retry;
			}
		}
		writel(0, &reg->done_qbuffer[index]);
		index++;
		/*if last index number set it to 0 */
		index %= ARCMSR_MAX_HBB_POSTQUEUE;
		reg->doneq_index = index;
		/* check if command done with no error*/
		arcmsr_cdb = (struct ARCMSR_CDB *)
		(acb->vir2phy_offset + (flag_ccb << 5));
		ccb = container_of(arcmsr_cdb,
			struct CommandControlBlock,
			arcmsr_cdb);
		poll_ccb_done = (ccb == poll_ccb) ? 1 : 0;
		if ((ccb->acb != acb) ||
		(ccb->startdone != ARCMSR_CCB_START)) {
			if ((ccb->startdone == ARCMSR_CCB_ABORTED) ||
				(ccb == poll_ccb)) {
				pr_notice("arcmsr%d: "
				"scsi id = %d lun = %d ccb = '0x%p' poll "
				"command abort successfully\n"
				, acb->host->host_no
				, ccb->pcmd->device->id
				, ccb->pcmd->device->lun
				, ccb);
				ccb->pcmd->result = DID_ABORT << 16;
				arcmsr_ccb_complete(ccb);
				continue;
			}
			pr_notice("arcmsr%d: polling get an "
				"illegal ccb command done ccb = '0x%p' "
				"ccboutstandingcount = %d\n"
				, acb->host->host_no
				, ccb
				, atomic_read(&acb->ccboutstandingcount));
			continue;
		} 
		error = (flag_ccb & ARCMSR_CCBREPLY_FLAG_ERROR_MODE0)
		? true : false;
		arcmsr_report_ccb_state(acb, ccb, error);
	}
	return rtn;
}

static int
arcmsr_hbaC_polling_ccbdone(struct AdapterControlBlock *acb,
	struct CommandControlBlock *poll_ccb)
{
	struct MessageUnit_C *reg = (struct MessageUnit_C *)acb->pmuC;
	uint32_t flag_ccb, ccb_cdb_phy;
	struct ARCMSR_CDB *arcmsr_cdb;
	bool error;
	struct CommandControlBlock *pCCB;
	uint32_t poll_ccb_done = 0, poll_count = 0;
	int rtn;
polling_hbc_ccb_retry:
	poll_count++;
	while (1) {
		if ((readl(&reg->host_int_status) &
			ARCMSR_HBCMU_OUTBOUND_POSTQUEUE_ISR) == 0) {
			if (poll_ccb_done) {
				rtn = SUCCESS;
				break;
			} else {
				msleep(25);
				if (poll_count > 100) {
					rtn = FAILED;
					break;
				}
				goto polling_hbc_ccb_retry;
			}
		}
		flag_ccb = readl(&reg->outbound_queueport_low);
		ccb_cdb_phy = (flag_ccb & 0xFFFFFFF0);
		arcmsr_cdb = (struct ARCMSR_CDB *)(acb->vir2phy_offset
			+ ccb_cdb_phy);
		pCCB = container_of(arcmsr_cdb, struct CommandControlBlock,
			arcmsr_cdb);
		poll_ccb_done = (pCCB == poll_ccb) ? 1 : 0;
		/* check ifcommand done with no error*/
		if ((pCCB->acb != acb) ||
			(pCCB->startdone != ARCMSR_CCB_START)) {
			if (pCCB->startdone == ARCMSR_CCB_ABORTED) {
				pr_notice("arcmsr%d: "
				"scsi id = %d lun = %d ccb = '0x%p' poll "
				"command abort successfully\n"
				, acb->host->host_no
				, pCCB->pcmd->device->id
				, pCCB->pcmd->device->lun
				, pCCB);
				pCCB->pcmd->result = DID_ABORT << 16;
				arcmsr_ccb_complete(pCCB);
				continue;
			}
			pr_notice("arcmsr%d: polling get an illegal "
			"ccb command done ccb = '0x%p' "
			"ccboutstandingcount = %d\n"
			, acb->host->host_no
			, pCCB
			, atomic_read(&acb->ccboutstandingcount));
			continue;
		}
		error = (flag_ccb & ARCMSR_CCBREPLY_FLAG_ERROR_MODE1)
			? true : false;
		arcmsr_report_ccb_state(acb, pCCB, error);
	}
	return rtn;
}

static int
arcmsr_hbaD_polling_ccbdone(struct AdapterControlBlock *acb,
				struct CommandControlBlock *poll_ccb)
{
	bool error;
	uint32_t poll_ccb_done = 0, poll_count = 0, flag_ccb, ccb_cdb_phy;
	int rtn, doneq_index, index_stripped, outbound_write_pointer;
	unsigned long flags;
	struct ARCMSR_CDB *arcmsr_cdb;
	struct CommandControlBlock *pCCB;
	struct MessageUnit_D __iomem *pmu =
		(struct MessageUnit_D *)acb->pmuD;

	spin_lock_irqsave(&acb->doneq_lock, flags);
	polling_hbaD_ccb_retry:
	poll_count++;
	while (1) {
		outbound_write_pointer =
		pmu->done_qbuffer[0].addressLow + 1;
		doneq_index = pmu->doneq_index;
		if ((outbound_write_pointer & 0xFFF) == (doneq_index & 0xFFF)) {
			if (poll_ccb_done) {
				rtn = SUCCESS;
				break;
			} else {
				msleep(25);
				if (poll_count > 100) {
					rtn = FAILED;
					break;
				}
				goto polling_hbaD_ccb_retry;
			}
		}
		if (doneq_index & 0x4000) {
			index_stripped = doneq_index & 0xFFF;
			index_stripped += 1;
			index_stripped %= ARCMSR_MAX_ARC1214_DONEQUEUE;
			pmu->doneq_index = index_stripped ? (index_stripped | 0x4000)
				: (index_stripped + 1);
		} else {
			index_stripped = doneq_index;
			index_stripped += 1;
			index_stripped %= ARCMSR_MAX_ARC1214_DONEQUEUE;
			pmu->doneq_index = index_stripped ? index_stripped :
				((index_stripped | 0x4000) + 1);
		}
		doneq_index = pmu->doneq_index;
		flag_ccb = pmu->done_qbuffer[doneq_index & 0xFFF].addressLow;
		ccb_cdb_phy = (flag_ccb & 0xFFFFFFF0);
		arcmsr_cdb = (struct ARCMSR_CDB *)(acb->vir2phy_offset +
			ccb_cdb_phy);
		pCCB = container_of(arcmsr_cdb,
			struct CommandControlBlock, arcmsr_cdb);
		poll_ccb_done = (pCCB == poll_ccb) ? 1 : 0;
		if ((pCCB->acb != acb) ||
		(pCCB->startdone != ARCMSR_CCB_START)) {
			if (pCCB->startdone == ARCMSR_CCB_ABORTED) {
				pr_notice("arcmsr%d: scsi id = %d "
				"lun = %d ccb = '0x%p' poll command "
				"abort successfully\n"
				, acb->host->host_no
				, pCCB->pcmd->device->id
				, pCCB->pcmd->device->lun
				, pCCB);
				pCCB->pcmd->result = DID_ABORT << 16;
				arcmsr_ccb_complete(pCCB);
				continue;
			}
			pr_notice("arcmsr%d: polling an illegal "
			"ccb command done ccb = '0x%p' "
			"ccboutstandingcount = %d\n"
			, acb->host->host_no
			, pCCB
			, atomic_read(&acb->ccboutstandingcount));
			continue;
		}
		error = (flag_ccb & ARCMSR_CCBREPLY_FLAG_ERROR_MODE1)
		? true : false;
		arcmsr_report_ccb_state(acb, pCCB, error);
	}
	spin_unlock_irqrestore(&acb->doneq_lock, flags);
	return rtn;
}

static int
arcmsr_polling_ccbdone(struct AdapterControlBlock *acb,
				struct CommandControlBlock *poll_ccb)
{
	int rtn = 0;
	switch (acb->adapter_type) {
	case ACB_ADAPTER_TYPE_A:{
		rtn = arcmsr_hbaA_polling_ccbdone(acb, poll_ccb);
		break;
	}
	case ACB_ADAPTER_TYPE_B:{
		rtn = arcmsr_hbaB_polling_ccbdone(acb, poll_ccb);
		break;
	}
	case ACB_ADAPTER_TYPE_C: {
		rtn = arcmsr_hbaC_polling_ccbdone(acb, poll_ccb);
		break;
	}
	case ACB_ADAPTER_TYPE_D: {
		rtn = arcmsr_hbaD_polling_ccbdone(acb, poll_ccb);
	}
	}
	return rtn;
}

static int
arcmsr_iop_confirm(struct AdapterControlBlock *acb)
{
	uint32_t cdb_phyaddr, cdb_phyaddr_hi32, cdb_phyaddr_lo32;
	dma_addr_t dma_coherent_handle;
	/*
	********************************************************************
	** here we need to tell iop 331 our freeccb.HighPart
	** if freeccb.HighPart is not zero
	********************************************************************
	*/
	dma_coherent_handle = acb->dma_coherent_handle;
	cdb_phyaddr_lo32 = (uint32_t)(dma_coherent_handle & 0xffffffff);
	cdb_phyaddr_hi32 = (uint32_t)((cdb_phyaddr >> 16) >> 16);
	acb->cdb_phyaddr_hi32 = cdb_phyaddr_hi32;
	/*
	***********************************************************************
	**    if adapter type B, set window of "post command Q"
	***********************************************************************
	*/
	switch (acb->adapter_type) {

	case ACB_ADAPTER_TYPE_A: {
		if (cdb_phyaddr_hi32 != 0) {
			struct MessageUnit_A __iomem *reg = acb->pmuA;
			writel(ARCMSR_SIGNATURE_SET_CONFIG,
			&reg->message_rwbuffer[0]);
			writel(cdb_phyaddr_hi32, &reg->message_rwbuffer[1]);
			writel(ARCMSR_INBOUND_MESG0_SET_CONFIG,
			&reg->inbound_msgaddr0);
			if (!arcmsr_hbaA_wait_msgint_ready(acb)) {
				pr_notice("arcmsr%d: set ccb "
				"high part physical address timeout\n",
				acb->host->host_no);
				return 1;
			}
		}
		}
		break;

	case ACB_ADAPTER_TYPE_B: {
		unsigned long post_queue_phyaddr;
		uint32_t __iomem *rwbuffer;

		struct MessageUnit_B *reg = acb->pmuB;
		reg->postq_index = 0;
		reg->doneq_index = 0;
		writel(ARCMSR_MESSAGE_SET_POST_WINDOW,
		reg->drv2iop_doorbell);
		if (!arcmsr_hbaB_wait_msgint_ready(acb)) {
			pr_notice("arcmsr%d:can not set diver mode\n",
			acb->host->host_no);
			return 1;
		}
		post_queue_phyaddr = acb->dma_coherent_handle2;
		rwbuffer = reg->message_rwbuffer;
		/* driver "set config" signature */
		writel(ARCMSR_SIGNATURE_SET_CONFIG, rwbuffer++);
		/* normal should be zero */
		writel(cdb_phyaddr_hi32, rwbuffer++);
		/* postQ size (256 + 8)*4	 */
		writel(post_queue_phyaddr, rwbuffer++);
		/* doneQ size (256 + 8)*4	 */
		writel(post_queue_phyaddr + 1056, rwbuffer++);
		/* ccb maxQ size must be --> [(256 + 8)*4]*/
		writel(1056, rwbuffer);
		writel(ARCMSR_MESSAGE_SET_CONFIG, reg->drv2iop_doorbell);
		if (!arcmsr_hbaB_wait_msgint_ready(acb)) {
			pr_notice("arcmsr%d: 'set command Q window' "
			"timeout\n", acb->host->host_no);
			return 1;
		}
		arcmsr_hbb_enable_driver_mode(acb);
		}
		break;
	case ACB_ADAPTER_TYPE_C: {
		if (cdb_phyaddr_hi32 != 0) {
			struct MessageUnit_C *reg =
			(struct MessageUnit_C *)acb->pmuC;
			pr_notice("arcmsr%d: cdb_phyaddr_hi32 = 0x%x\n",
			acb->adapter_index, cdb_phyaddr_hi32);
			writel(ARCMSR_SIGNATURE_SET_CONFIG,
				&reg->msgcode_rwbuffer[0]);
			writel(cdb_phyaddr_hi32,
				&reg->msgcode_rwbuffer[1]);
			writel(ARCMSR_INBOUND_MESG0_SET_CONFIG,
				&reg->inbound_msgaddr0);
			writel(ARCMSR_HBCMU_DRV2IOP_MESSAGE_CMD_DONE,
				&reg->inbound_doorbell);
			if (!arcmsr_hbaC_wait_msgint_ready(acb)) {
				pr_notice("arcmsr%d: 'set "
				"command Q window' timeout\n",
				acb->host->host_no);
				return 1;
			}
		}
		}
		break;
	case ACB_ADAPTER_TYPE_D: {
		uint32_t __iomem *rwbuffer;

		struct MessageUnit_D *reg =
			(struct MessageUnit_D *)acb->pmuD;
		reg->postq_index = 0;
		reg->doneq_index = 0;
		rwbuffer = reg->msgcode_rwbuffer;
		writel(ARCMSR_SIGNATURE_SET_CONFIG, rwbuffer++);
		writel(cdb_phyaddr_hi32, rwbuffer++);
		writel(cdb_phyaddr_lo32, rwbuffer++);
		writel(cdb_phyaddr_lo32 +
		(ARCMSR_MAX_ARC1214_POSTQUEUE * sizeof(struct InBound_SRB)),
		rwbuffer++);
		writel(0x100, rwbuffer);
		writel(ARCMSR_INBOUND_MESG0_SET_CONFIG,
			reg->inbound_msgaddr0);
		if (!arcmsr_hbaD_wait_msgint_ready(acb))
			pr_notice("arcmsr%d: 'set command Q "
			"window' timeout\n", acb->host->host_no);
		break;
	}
	}
	return 0;
}

static void
arcmsr_wait_firmware_ready(struct AdapterControlBlock *acb)
{
	uint32_t firmware_state = 0;
	switch (acb->adapter_type) {

	case ACB_ADAPTER_TYPE_A: {
		struct MessageUnit_A __iomem *reg = acb->pmuA;
		do {
			firmware_state = readl(&reg->outbound_msgaddr1);
		} while ((firmware_state &
		ARCMSR_OUTBOUND_MESG1_FIRMWARE_OK) == 0);
		}
		break;
	case ACB_ADAPTER_TYPE_B: {
		struct MessageUnit_B *reg = acb->pmuB;
		do {
			firmware_state = readl(reg->iop2drv_doorbell);
		} while ((firmware_state & ARCMSR_MESSAGE_FIRMWARE_OK) == 0);
		writel(ARCMSR_DRV2IOP_END_OF_INTERRUPT,
		reg->drv2iop_doorbell);
		}
		break;
	case ACB_ADAPTER_TYPE_C: {
		struct MessageUnit_C *reg =
			(struct MessageUnit_C *)acb->pmuC;
		do {
			firmware_state = readl(&reg->outbound_msgaddr1);
		} while ((firmware_state &
		ARCMSR_HBCMU_MESSAGE_FIRMWARE_OK) == 0);
		}
		break;
	case ACB_ADAPTER_TYPE_D: {
		struct MessageUnit_D __iomem *reg
			= (struct MessageUnit_D *)acb->pmuD;
		do {
			firmware_state = readl(reg->outbound_msgaddr1);
		} while ((firmware_state &
		ARCMSR_ARC1214_MESSAGE_FIRMWARE_OK) == 0);
		break;
	}
	}
}

static void
arcmsr_hbaA_request_device_map(struct AdapterControlBlock *acb)
{
	struct MessageUnit_A __iomem *reg = acb->pmuA;
	if (unlikely(atomic_read(&acb->rq_map_token) == 0) ||
		((acb->acb_flags & ACB_F_BUS_RESET) != 0)
		|| ((acb->acb_flags & ACB_F_ABORT) != 0)) {
		mod_timer(&acb->eternal_timer,
			jiffies + msecs_to_jiffies(6 * HZ));
		return;
	} else {
		acb->fw_flag = FW_NORMAL;
		if (atomic_read(&acb->ante_token_value) ==
			atomic_read(&acb->rq_map_token)) {
			atomic_set(&acb->rq_map_token, 16);
		}
		atomic_set(&acb->ante_token_value,
			atomic_read(&acb->rq_map_token));
		if (atomic_dec_and_test(&acb->rq_map_token)) {
			mod_timer(&acb->eternal_timer,
				jiffies + msecs_to_jiffies(6 * HZ));
			return;
		}
		writel(ARCMSR_INBOUND_MESG0_GET_CONFIG,
			&reg->inbound_msgaddr0);
		mod_timer(&acb->eternal_timer, jiffies +
			msecs_to_jiffies(6 * HZ));
	}
	return;
}

static void
arcmsr_hbaB_request_device_map(struct AdapterControlBlock *acb)
{
	struct MessageUnit_B __iomem *reg = acb->pmuB;
	if (unlikely(atomic_read(&acb->rq_map_token) == 0) ||
		((acb->acb_flags & ACB_F_BUS_RESET) != 0) ||
		((acb->acb_flags & ACB_F_ABORT) != 0)) {
		mod_timer(&acb->eternal_timer, jiffies +
			msecs_to_jiffies(6 * HZ));
		return;
	} else {
		acb->fw_flag = FW_NORMAL;
		if (atomic_read(&acb->ante_token_value) ==
			atomic_read(&acb->rq_map_token)) {
			atomic_set(&acb->rq_map_token, 16);
		}
		atomic_set(&acb->ante_token_value,
			atomic_read(&acb->rq_map_token));
		if (atomic_dec_and_test(&acb->rq_map_token)) {
			mod_timer(&acb->eternal_timer,
				jiffies + msecs_to_jiffies(6 * HZ));
			return;
		}
		writel(ARCMSR_MESSAGE_GET_CONFIG,
			reg->drv2iop_doorbell);
		mod_timer(&acb->eternal_timer, jiffies +
			msecs_to_jiffies(6 * HZ));
	}
	return;
}

static void
arcmsr_hbaC_request_device_map(struct AdapterControlBlock *acb)
{
	struct MessageUnit_C __iomem *reg = acb->pmuC;
	if (unlikely(atomic_read(&acb->rq_map_token) == 0) ||
		((acb->acb_flags & ACB_F_BUS_RESET) != 0) ||
		((acb->acb_flags & ACB_F_ABORT) != 0)) {
		mod_timer(&acb->eternal_timer, jiffies +
			msecs_to_jiffies(6 * HZ));
		return;
	} else {
		acb->fw_flag = FW_NORMAL;
		if (atomic_read(&acb->ante_token_value) ==
			atomic_read(&acb->rq_map_token)) {
			atomic_set(&acb->rq_map_token, 16);
		}
		atomic_set(&acb->ante_token_value,
			atomic_read(&acb->rq_map_token));
		if (atomic_dec_and_test(&acb->rq_map_token)) {
			mod_timer(&acb->eternal_timer,
				jiffies + msecs_to_jiffies(6 * HZ));
			return;
		}
		writel(ARCMSR_INBOUND_MESG0_GET_CONFIG,
			&reg->inbound_msgaddr0);
		writel(ARCMSR_HBCMU_DRV2IOP_MESSAGE_CMD_DONE,
			&reg->inbound_doorbell);
		mod_timer(&acb->eternal_timer, jiffies +
			msecs_to_jiffies(6 * HZ));
	}
	return;
}

static void
arcmsr_hbaD_request_device_map(struct AdapterControlBlock *acb)
{
	struct MessageUnit_D __iomem *reg = acb->pmuD;
	if (unlikely(atomic_read(&acb->rq_map_token) == 0) ||
		((acb->acb_flags & ACB_F_BUS_RESET) != 0) ||
		((acb->acb_flags & ACB_F_ABORT) != 0)) {
		mod_timer(&acb->eternal_timer,
			jiffies + msecs_to_jiffies(6 * HZ));
	} else {
		acb->fw_flag = FW_NORMAL;
		if (atomic_read(&acb->ante_token_value) ==
			atomic_read(&acb->rq_map_token)) {
			atomic_set(&acb->rq_map_token, 16);
		}
		atomic_set(&acb->ante_token_value,
			atomic_read(&acb->rq_map_token));
		if (atomic_dec_and_test(&acb->rq_map_token)) {
			mod_timer(&acb->eternal_timer, jiffies +
				msecs_to_jiffies(6 * HZ));
			return;
		}
		writel(ARCMSR_INBOUND_MESG0_GET_CONFIG,
			reg->inbound_msgaddr0);
		mod_timer(&acb->eternal_timer, jiffies +
			msecs_to_jiffies(6 * HZ));
	}
	return;
}

static void
arcmsr_request_device_map(unsigned long pacb)
{
	struct AdapterControlBlock *acb =
		(struct AdapterControlBlock *)pacb;
	switch (acb->adapter_type) {
		case ACB_ADAPTER_TYPE_A: {
			arcmsr_hbaA_request_device_map(acb);
			break;
		}
		case ACB_ADAPTER_TYPE_B: {
			arcmsr_hbaB_request_device_map(acb);
			break;
		}
		case ACB_ADAPTER_TYPE_C: {
			arcmsr_hbaC_request_device_map(acb);
			break;
		}
		case ACB_ADAPTER_TYPE_D: {
			arcmsr_hbaD_request_device_map(acb);
		}
	}
}

static void
arcmsr_hbaA_start_bgrb(struct AdapterControlBlock *acb)
{
	struct MessageUnit_A __iomem *reg = acb->pmuA;
	acb->acb_flags |= ACB_F_MSG_START_BGRB;
	writel(ARCMSR_INBOUND_MESG0_START_BGRB,
		&reg->inbound_msgaddr0);
	if (!arcmsr_hbaA_wait_msgint_ready(acb)) {
		pr_notice("arcmsr%d: wait 'start adapter "
		"background rebulid' timeout\n", acb->host->host_no);
	}
}

static void
arcmsr_hbaB_start_bgrb(struct AdapterControlBlock *acb)
{
	struct MessageUnit_B *reg = acb->pmuB;
	acb->acb_flags |= ACB_F_MSG_START_BGRB;
	writel(ARCMSR_MESSAGE_START_BGRB, reg->drv2iop_doorbell);
	if (!arcmsr_hbaB_wait_msgint_ready(acb)) {
		pr_notice("arcmsr%d: wait 'start adapter "
		"backgroundrebulid' timeout\n", acb->host->host_no);
	}
}

static void
arcmsr_hbaC_start_bgrb(struct AdapterControlBlock *pACB)
{
	struct MessageUnit_C *phbcmu =
		(struct MessageUnit_C *)pACB->pmuC;
	pACB->acb_flags |= ACB_F_MSG_START_BGRB;
	writel(ARCMSR_INBOUND_MESG0_START_BGRB,
		&phbcmu->inbound_msgaddr0);
	writel(ARCMSR_HBCMU_DRV2IOP_MESSAGE_CMD_DONE,
		&phbcmu->inbound_doorbell);
	if (!arcmsr_hbaC_wait_msgint_ready(pACB)) {
		pr_notice("arcmsr%d: wait 'start adapter "
		"background rebulid' timeout\n", pACB->host->host_no);
	}
	return;
}

static void
arcmsr_hbaD_start_bgrb(struct AdapterControlBlock *pACB)
{
	struct MessageUnit_D __iomem *pmu = (struct MessageUnit_D *)pACB->pmuD;
	pACB->acb_flags |= ACB_F_MSG_START_BGRB;
	writel(ARCMSR_INBOUND_MESG0_START_BGRB, pmu->inbound_msgaddr0);
	if (!arcmsr_hbaD_wait_msgint_ready(pACB)) {
		pr_notice("arcmsr%d: wait 'start adapter "
		"background rebulid' timeout\n", pACB->host->host_no);
	}
	return;
}

static void
arcmsr_start_adapter_bgrb(struct AdapterControlBlock *acb)
{
	switch (acb->adapter_type) {
	case ACB_ADAPTER_TYPE_A:
		arcmsr_hbaA_start_bgrb(acb);
		break;
	case ACB_ADAPTER_TYPE_B:
		arcmsr_hbaB_start_bgrb(acb);
		break;
	case ACB_ADAPTER_TYPE_C:
		arcmsr_hbaC_start_bgrb(acb);
		break;
	case ACB_ADAPTER_TYPE_D:
		arcmsr_hbaD_start_bgrb(acb);
		break;
	}
}

static void
arcmsr_clear_doorbell_queue_buffer(struct AdapterControlBlock *acb)
{
	switch (acb->adapter_type) {
	case ACB_ADAPTER_TYPE_A: {
		struct MessageUnit_A __iomem *reg = acb->pmuA;
		uint32_t outbound_doorbell;
		/* empty doorbell Qbuffer if door bell ringed */
		outbound_doorbell = readl(&reg->outbound_doorbell);
		/*clear doorbell interrupt */
		writel(outbound_doorbell, &reg->outbound_doorbell);
		writel(ARCMSR_INBOUND_DRIVER_DATA_READ_OK,
			&reg->inbound_doorbell);
		}
		break;

	case ACB_ADAPTER_TYPE_B: {
		struct MessageUnit_B *reg = acb->pmuB;
		/*clear interrupt and message state*/
		writel(ARCMSR_MESSAGE_INT_CLEAR_PATTERN,
		reg->iop2drv_doorbell);
		writel(ARCMSR_DRV2IOP_DATA_READ_OK,
			reg->drv2iop_doorbell);
		/* let IOP know data has been read */
		}
		break;
	case ACB_ADAPTER_TYPE_C: {
		struct MessageUnit_C *reg =
			(struct MessageUnit_C *)acb->pmuC;
		uint32_t outbound_doorbell;
		/* empty doorbell Qbuffer if door bell ringed */
		outbound_doorbell = readl(&reg->outbound_doorbell);
		writel(outbound_doorbell, &reg->outbound_doorbell_clear);
		writel(ARCMSR_HBCMU_DRV2IOP_DATA_READ_OK,
			&reg->inbound_doorbell);
		}
		break;
	case ACB_ADAPTER_TYPE_D: {
		struct MessageUnit_D __iomem *reg =
			(struct MessageUnit_D *)acb->pmuD;
		uint32_t outbound_doorbell;
		/* empty doorbell Qbuffer if door bell ringed */
		outbound_doorbell = readl(reg->outbound_doorbell);
		writel(outbound_doorbell, reg->outbound_doorbell);
		writel(ARCMSR_ARC1214_DRV2IOP_DATA_OUT_READ,
			reg->inbound_doorbell);
		break;
	}
	}
}

static void
arcmsr_enable_eoi_mode(struct AdapterControlBlock *acb)
{
	switch (acb->adapter_type) {
	case ACB_ADAPTER_TYPE_A:
	case ACB_ADAPTER_TYPE_C:
	case ACB_ADAPTER_TYPE_D:
		return;
	case ACB_ADAPTER_TYPE_B: {
		struct MessageUnit_B *reg = acb->pmuB;
			writel(ARCMSR_MESSAGE_ACTIVE_EOI_MODE,
				reg->drv2iop_doorbell);
			if (!arcmsr_hbaB_wait_msgint_ready(acb)) {
				pr_notice("ARCMSR IOP "
				"enables EOI_MODE TIMEOUT");
				return;
			}
	}
	break;
	}
	return;
}

static void
arcmsr_hardware_reset(struct AdapterControlBlock *acb)
{
	uint8_t value[64];
	int i, count = 0;
	struct MessageUnit_A __iomem *pmuA = acb->pmuA;
	struct MessageUnit_C __iomem *pmuC = acb->pmuC;
	u32 temp = 0;
	/* backup pci config data */
	pr_notice("arcmsr%d: executing hw bus reset .....\n",
	acb->host->host_no);
	for (i = 0; i < 64; i++) {
		pci_read_config_byte(acb->pdev, i, &value[i]);
	}
	/* hardware reset signal */
	if ((acb->dev_id == 0x1680)) {
		writel(ARCMSR_ARC1680_BUS_RESET,
			&pmuA->reserved1[0]);
	} else if ((acb->dev_id == 0x1880)) {
		do {
			count++;
			writel(0xF, &pmuC->write_sequence);
			writel(0x4, &pmuC->write_sequence);
			writel(0xB, &pmuC->write_sequence);
			writel(0x2, &pmuC->write_sequence);
			writel(0x7, &pmuC->write_sequence);
			writel(0xD, &pmuC->write_sequence);
		} while ((((temp = readl(&pmuC->host_diagnostic)) |
		ARCMSR_ARC1880_DiagWrite_ENABLE) == 0) &&
		(count < 5));
		writel(ARCMSR_ARC1880_RESET_ADAPTER,
			&pmuC->host_diagnostic);
	} else {
		pci_write_config_byte(acb->pdev, 0x84, 0x20);
	}
	msleep(2000);
	/* write back pci config data */
	for (i = 0; i < 64; i++) {
		pci_write_config_byte(acb->pdev, i, value[i]);
	}
	msleep(1000);
	return;
}

static void
arcmsr_iop_init(struct AdapterControlBlock *acb)
{
	uint32_t intmask_org;
	/* disable all outbound interrupt */
	intmask_org = arcmsr_disable_outbound_ints(acb);
	arcmsr_wait_firmware_ready(acb);
	arcmsr_iop_confirm(acb);
	/*start background rebuild*/
	arcmsr_start_adapter_bgrb(acb);
	/* empty doorbell Qbuffer if door bell ringed */
	arcmsr_clear_doorbell_queue_buffer(acb);
	arcmsr_enable_eoi_mode(acb);
	/* enable outbound Post Queue,outbound doorbell Interrupt */
	arcmsr_enable_outbound_ints(acb, intmask_org);
	acb->acb_flags |= ACB_F_IOP_INITED;
}

static uint8_t
arcmsr_iop_reset(struct AdapterControlBlock *acb)
{
	struct CommandControlBlock *ccb;
	uint32_t intmask_org;
	uint8_t rtnval = 0x00;
	int i = 0;
	unsigned long flags;

	if (atomic_read(&acb->ccboutstandingcount) != 0) {
		/* disable all outbound interrupt */
		intmask_org = arcmsr_disable_outbound_ints(acb);
		/* talk to iop 331 outstanding command aborted */
		rtnval = arcmsr_abort_allcmd(acb);
		/* clear all outbound posted Q */
		arcmsr_done4abort_postqueue(acb);
		for (i = 0; i < ARCMSR_MAX_FREECCB_NUM; i++) {
			ccb = acb->pccb_pool[i];
			if (ccb->startdone == ARCMSR_CCB_START) {
				scsi_dma_unmap(ccb->pcmd);
				ccb->startdone = ARCMSR_CCB_DONE;
				ccb->ccb_flags = 0;
				spin_lock_irqsave(&acb->ccblist_lock, flags);
				list_add_tail(&ccb->list, &acb->ccb_free_list);
				spin_unlock_irqrestore(&acb->ccblist_lock,
					flags);
			}
		}
		atomic_set(&acb->ccboutstandingcount, 0);
		/* enable all outbound interrupt */
		arcmsr_enable_outbound_ints(acb, intmask_org);
		return rtnval;
	}
	return rtnval;
}

static int
arcmsr_bus_reset(struct scsi_cmnd *cmd)
{
	struct AdapterControlBlock *acb;
	uint32_t intmask_org, outbound_doorbell;
	int retry_count = 0;
	int rtn = FAILED;
	acb = (struct AdapterControlBlock *) cmd->device->host->hostdata;
	pr_err("arcmsr: executing bus reset eh.....num_resets = %d, "
	"num_aborts = %d\n", acb->num_resets, acb->num_aborts);
	acb->num_resets++;

	switch (acb->adapter_type) {
	case ACB_ADAPTER_TYPE_A: {
		if (acb->acb_flags & ACB_F_BUS_RESET) {
			long timeout;
			pr_err("arcmsr: there is an bus "
			"reset eh proceeding.......\n");
			timeout = wait_event_timeout(wait_q,
			(acb->acb_flags & ACB_F_BUS_RESET)
			== 0, 220 * HZ);
			if (timeout) {
				return SUCCESS;
			}
		}
		acb->acb_flags |= ACB_F_BUS_RESET;
		if (!arcmsr_iop_reset(acb)) {
			struct MessageUnit_A __iomem *reg;
			reg = acb->pmuA;
			arcmsr_hardware_reset(acb);
			acb->acb_flags &= ~ACB_F_IOP_INITED;
sleep_again:
			ssleep(ARCMSR_SLEEPTIME);
			if ((readl(&reg->outbound_msgaddr1) &
			ARCMSR_OUTBOUND_MESG1_FIRMWARE_OK) == 0) {
				pr_err("arcmsr%d: waiting for "
				"hw bus reset return, retry = %d\n",
				acb->host->host_no, retry_count);
				if (retry_count > ARCMSR_RETRYCOUNT) {
					acb->fw_flag = FW_DEADLOCK;
					pr_err("arcmsr%d: waiting "
					"for hw bus reset return, "
					"RETRY TERMINATED!!\n",
					acb->host->host_no);
					return FAILED;
				}
				retry_count++;
				goto sleep_again;
			}
			acb->acb_flags |= ACB_F_IOP_INITED;
			/* disable all outbound interrupt */
			intmask_org = arcmsr_disable_outbound_ints(acb);
			arcmsr_get_firmware_spec(acb);
			arcmsr_start_adapter_bgrb(acb);
			/* clear Qbuffer if door bell ringed */
			outbound_doorbell = readl(&reg->outbound_doorbell);
			writel(outbound_doorbell, &reg->outbound_doorbell);
			writel(ARCMSR_INBOUND_DRIVER_DATA_READ_OK,
				&reg->inbound_doorbell);
			arcmsr_enable_outbound_ints(acb, intmask_org);
			atomic_set(&acb->rq_map_token, 16);
			atomic_set(&acb->ante_token_value, 16);
			acb->fw_flag = FW_NORMAL;
			mod_timer(&acb->eternal_timer, jiffies +
				msecs_to_jiffies(6 * HZ));
			acb->acb_flags &= ~ACB_F_BUS_RESET;
			rtn = SUCCESS;
			pr_err("arcmsr: scsi bus reset eh "
			"returns with success\n");
		} else {
			acb->acb_flags &= ~ACB_F_BUS_RESET;
			atomic_set(&acb->rq_map_token, 16);
			atomic_set(&acb->ante_token_value, 16);
			acb->fw_flag = FW_NORMAL;
			mod_timer(&acb->eternal_timer,
			jiffies + msecs_to_jiffies(6 * HZ));
			rtn = SUCCESS;
		}
		break;
	}
	case ACB_ADAPTER_TYPE_B: {
		acb->acb_flags |= ACB_F_BUS_RESET;
		if (!arcmsr_iop_reset(acb)) {
			acb->acb_flags &= ~ACB_F_BUS_RESET;
			rtn = FAILED;
		} else {
			acb->acb_flags &= ~ACB_F_BUS_RESET;
			atomic_set(&acb->rq_map_token, 16);
			atomic_set(&acb->ante_token_value, 16);
			acb->fw_flag = FW_NORMAL;
			mod_timer(&acb->eternal_timer,
				jiffies + msecs_to_jiffies(6 * HZ));
			rtn = SUCCESS;
		}
		break;
	}
	case ACB_ADAPTER_TYPE_C: {
		if (acb->acb_flags & ACB_F_BUS_RESET) {
			long timeout;
			pr_err("arcmsr: there is an bus "
			"reset eh proceeding.......\n");
			timeout = wait_event_timeout(wait_q,
			(acb->acb_flags & ACB_F_BUS_RESET) == 0,
			220 * HZ);
			if (timeout) {
				return SUCCESS;
			}
		}
		acb->acb_flags |= ACB_F_BUS_RESET;
		if (!arcmsr_iop_reset(acb)) {
			struct MessageUnit_C __iomem *reg;
			reg = acb->pmuC;
			arcmsr_hardware_reset(acb);
			acb->acb_flags &= ~ACB_F_IOP_INITED;
sleep:
			ssleep(ARCMSR_SLEEPTIME);
			if ((readl(&reg->host_diagnostic) & 0x04) != 0) {
				pr_err("arcmsr%d: waiting "
				"for hw bus reset return, retry = %d\n",
				acb->host->host_no, retry_count);
				if (retry_count > ARCMSR_RETRYCOUNT) {
					acb->fw_flag = FW_DEADLOCK;
					pr_err("arcmsr%d: "
					"waiting for hw bus reset return, "
					"RETRY TERMINATED!!\n",
					acb->host->host_no);
					return FAILED;
				}
				retry_count++;
				goto sleep;
			}
			acb->acb_flags |= ACB_F_IOP_INITED;
			/* disable all outbound interrupt */
			intmask_org =
			arcmsr_disable_outbound_ints(acb);
			arcmsr_get_firmware_spec(acb);
			arcmsr_start_adapter_bgrb(acb);
			/* clear Qbuffer if door bell ringed */
			outbound_doorbell =
			readl(&reg->outbound_doorbell);
			writel(outbound_doorbell,
				&reg->outbound_doorbell_clear);
			writel(ARCMSR_HBCMU_DRV2IOP_DATA_READ_OK,
				&reg->inbound_doorbell);
			arcmsr_enable_outbound_ints(acb,
				intmask_org);
			atomic_set(&acb->rq_map_token, 16);
			atomic_set(&acb->ante_token_value, 16);
			acb->fw_flag = FW_NORMAL;
			mod_timer(&acb->eternal_timer, jiffies +
				msecs_to_jiffies(6 * HZ));
			acb->acb_flags &= ~ACB_F_BUS_RESET;
			rtn = SUCCESS;
			pr_err("arcmsr: scsi bus reset "
			"eh returns with success\n");
		} else {
			acb->acb_flags &= ~ACB_F_BUS_RESET;
			atomic_set(&acb->rq_map_token, 16);
			atomic_set(&acb->ante_token_value, 16);
			acb->fw_flag = FW_NORMAL;
			mod_timer(&acb->eternal_timer, jiffies +
			msecs_to_jiffies(6 * HZ));
			rtn = SUCCESS;
		}
		break;
	}
	case ACB_ADAPTER_TYPE_D: {
		if (acb->acb_flags & ACB_F_BUS_RESET) {
			long timeout;
			pr_notice("arcmsr: there is an bus reset eh proceeding.......\n");
			timeout = wait_event_timeout(wait_q, (acb->acb_flags
				& ACB_F_BUS_RESET) == 0, 220 * HZ);
			if (timeout)
				return SUCCESS;
		}
		acb->acb_flags |= ACB_F_BUS_RESET;
		if (!arcmsr_iop_reset(acb)) {
			struct MessageUnit_D __iomem *reg;
			reg = acb->pmuD;
			arcmsr_hardware_reset(acb);
			acb->acb_flags &= ~ACB_F_IOP_INITED;
			nap:
			ssleep(ARCMSR_SLEEPTIME);
			if ((readl(reg->sample_at_reset) & 0x80) != 0) {
				pr_err("arcmsr%d: waiting for "
				"hw bus reset return, retry=%d\n",
				acb->host->host_no, retry_count);
				if (retry_count > ARCMSR_RETRYCOUNT) {
					acb->fw_flag = FW_DEADLOCK;
					pr_err("arcmsr%d: "
					"waiting for hw bus reset return, "
					"RETRY TERMINATED!!\n",
					acb->host->host_no);
					return FAILED;
				}
				retry_count++;
				goto nap;
			}
			acb->acb_flags |= ACB_F_IOP_INITED;
			/* disable all outbound interrupt */
			intmask_org = arcmsr_disable_outbound_ints(acb);
			arcmsr_get_firmware_spec(acb);
			arcmsr_start_adapter_bgrb(acb);
			arcmsr_clear_doorbell_queue_buffer(acb);
			arcmsr_enable_outbound_ints(acb, intmask_org);
			atomic_set(&acb->rq_map_token, 16);
			atomic_set(&acb->ante_token_value, 16);
			acb->fw_flag = FW_NORMAL;
			mod_timer(&acb->eternal_timer,
				jiffies + msecs_to_jiffies(6 * HZ));
			acb->acb_flags &= ~ACB_F_BUS_RESET;
			rtn = SUCCESS;
			pr_err("arcmsr: scsi bus reset "
			"eh returns with success\n");
		} else {
			acb->acb_flags &= ~ACB_F_BUS_RESET;
			atomic_set(&acb->rq_map_token, 16);
			atomic_set(&acb->ante_token_value, 16);
			acb->fw_flag = FW_NORMAL;
			mod_timer(&acb->eternal_timer,
				jiffies + msecs_to_jiffies(6 * HZ));
			rtn = SUCCESS;
		}
		break;
	}
	}
	return rtn;
}

static int
arcmsr_abort_one_cmd(struct AdapterControlBlock *acb,
		struct CommandControlBlock *ccb)
{
	int rtn;
	rtn = arcmsr_polling_ccbdone(acb, ccb);
	return rtn;
}

static int
arcmsr_abort(struct scsi_cmnd *cmd)
{
	struct AdapterControlBlock *acb = (struct AdapterControlBlock *)
		cmd->device->host->hostdata;
	int i = 0;
	int rtn = FAILED;
	pr_notice("arcmsr%d: abort device command of "
		"scsi id = %d lun = %d\n",
		acb->host->host_no,
		cmd->device->id, cmd->device->lun);
	acb->acb_flags |= ACB_F_ABORT;
	acb->num_aborts++;
	/*
	************************************************
	** the all interrupt service routine is locked
	** we need to handle it as soon as possible and exit
	************************************************
	*/
	if (!atomic_read(&acb->ccboutstandingcount))
		return rtn;

	for (i = 0; i < ARCMSR_MAX_FREECCB_NUM; i++) {
		struct CommandControlBlock *ccb = acb->pccb_pool[i];
		if (ccb->startdone == ARCMSR_CCB_START &&
			ccb->pcmd == cmd) {
			ccb->startdone = ARCMSR_CCB_ABORTED;
			rtn = arcmsr_abort_one_cmd(acb, ccb);
			break;
		}
	}
	acb->acb_flags &= ~ACB_F_ABORT;
	return rtn;
}

static const char
*arcmsr_info(struct Scsi_Host *host)
{
	struct AdapterControlBlock *acb =
	(struct AdapterControlBlock *)host->hostdata;
	static char buf[256];
	char *type;
	int raid6 = 1;
	switch (acb->pdev->device) {
	case PCI_DEVICE_ID_ARECA_1110:
	case PCI_DEVICE_ID_ARECA_1200:
	case PCI_DEVICE_ID_ARECA_1202:
	case PCI_DEVICE_ID_ARECA_1210:
		raid6 = 0;
		/*FALLTHRU*/
	case PCI_DEVICE_ID_ARECA_1120:
	case PCI_DEVICE_ID_ARECA_1130:
	case PCI_DEVICE_ID_ARECA_1160:
	case PCI_DEVICE_ID_ARECA_1170:
	case PCI_DEVICE_ID_ARECA_1201:
	case PCI_DEVICE_ID_ARECA_1214:
	case PCI_DEVICE_ID_ARECA_1220:
	case PCI_DEVICE_ID_ARECA_1230:
	case PCI_DEVICE_ID_ARECA_1260:
	case PCI_DEVICE_ID_ARECA_1270:
	case PCI_DEVICE_ID_ARECA_1280:
		type = "SATA";
		break;
	case PCI_DEVICE_ID_ARECA_1680:
	case PCI_DEVICE_ID_ARECA_1681:
	case PCI_DEVICE_ID_ARECA_1880:
		type = "SAS";
		break;
	default:
		type = "X-TYPE";
		break;
	}
	sprintf(buf, "Areca %s Host Adapter RAID Controller%s\n %s",
			type, raid6 ? "( RAID6 capable)" : "",
			ARCMSR_DRIVER_VERSION);
	return buf;
}

