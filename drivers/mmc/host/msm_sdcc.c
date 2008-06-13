/*
 *  linux/drivers/mmc/host/msm_sdcc.c - Qualcomm MSM 7X00A SDCC Driver
 *
 *  Copyright (C) 2007 Google Inc,
 *  Copyright (C) 2003 Deep Blue Solutions, Ltd, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Based on mmci.c
 *
 * Author: San Mehat (san@android.com)
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/highmem.h>
#include <linux/log2.h>
#include <linux/mmc/host.h>
#include <linux/mmc/card.h>
#include <linux/clk.h>
#include <linux/scatterlist.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/debugfs.h>

#include <asm/cacheflush.h>
#include <asm/div64.h>
#include <asm/io.h>
#include <asm/sizes.h>
#include <asm/memory.h>
#include <asm/dma-mapping.h>

#include <asm/mach/mmc.h>
#include <asm/arch/msm_iomap.h>
#include <asm/arch/dma.h>


#include "msm_sdcc.h"

#define DRIVER_NAME "msm-sdcc"

#define DBG(host, fmt, args...)	\
	pr_debug("%s: %s: " fmt, mmc_hostname(host->mmc), __func__ , args)

#if defined(CONFIG_DEBUG_FS)
static void msmsdcc_dbg_createhost(struct msmsdcc_host *);

static struct dentry *debugfs_dir;
#endif

static unsigned int msmsdcc_fmin = 144000;
static unsigned int msmsdcc_fmax = 20000000;
static unsigned int msmsdcc_4bit = 0;

static char *msmsdcc_clks[] = { NULL, "sdc1_clk", "sdc2_clk", "sdc3_clk",
				"sdc4_clk" };
static char *msmsdcc_pclks[] = { NULL, "sdc1_pclk", "sdc2_pclk", "sdc3_pclk",
				 "sdc4_pclk" };


#define VERBOSE_COMMAND_TIMEOUTS	0

static irqreturn_t msmsdcc_pio_irq(int irq, void *dev_id);

static void msmsdcc_start_command(struct msmsdcc_host *host,
				  struct mmc_command *cmd, u32 c);

static void
msmsdcc_request_end(struct msmsdcc_host *host, struct mmc_request *mrq)
{

	writel(0, host->base + MMCICOMMAND);

	BUG_ON(host->data);

	host->mrq = NULL;
	host->cmd = NULL;

	if (mrq->data && mrq->data->error) {
		/*
		 * The higher level MMC stack only handles
		 * cmd->error.
		 */
		mrq->cmd->error = mrq->data->error;
	}

	if (mrq->data)
		mrq->data->bytes_xfered = host->data_xfered;

	if (mrq->cmd->error == -ETIMEDOUT)
		mdelay(5);

	if (mrq->cmd->error == -ETIME) {
		host->num_fail++;
		if (host->num_fail > 5) {
			printk(KERN_ERR
			       "%s: MMC bus dead - Simulating eject\n",
			       mmc_hostname(host->mmc));
			host->eject = 1;
		} else 
			mdelay(250);
	} else
		host->num_fail = 0;

	/*
	 * Need to drop the host lock here; mmc_request_done may call
	 * back into the driver...
	 */
	spin_unlock(&host->lock);
	mmc_request_done(host->mmc, mrq);
	spin_lock(&host->lock);
}

static void
msmsdcc_stop_data(struct msmsdcc_host *host)
{
	writel(0, host->base + MMCIDATACTRL);
	writel(0, host->base + MMCIMASK1);
	host->data = NULL;
}

uint32_t msmsdcc_fifo_addr(struct msmsdcc_host *host)
{
	if (host->pdev_id == 1)
		return MSM_SDC1_PHYS + MMCIFIFO;
	else if (host->pdev_id == 2)
		return MSM_SDC2_PHYS + MMCIFIFO;
	else if (host->pdev_id == 3)
		return MSM_SDC3_PHYS + MMCIFIFO;
	else if (host->pdev_id == 4)
		return MSM_SDC4_PHYS + MMCIFIFO;
	else
		BUG();
}

static void
msmsdcc_dma_complete_func(struct msm_dmov_cmd *cmd,
			  unsigned int result,
			  struct msm_dmov_errdata *err)
{
	struct msmsdcc_dma_data	*dma_data =
		container_of(cmd, struct msmsdcc_dma_data, hdr);
	struct msmsdcc_host	*host = dma_data->host;
	unsigned long		flags;

	spin_lock_irqsave(&host->lock, flags);

	dma_unmap_sg(mmc_dev(host->mmc), host->dma.sg, host->dma.num_ents,
		     host->dma.dir);

	if (result != 0x80000002) {
		struct mmc_request *mrq = host->mrq;

		WARN_ON(!mrq);

		printk(KERN_ERR "%s: DMA failure (Result 0x%x)\n",
		       mmc_hostname(host->mmc), result);
		
		if (err)
			printk(KERN_ERR
			       "DMA Flush: %.8x %.8x %.8x %.8x %.8x %.8x\n",
			       err->flush[0], err->flush[1], err->flush[2],
			       err->flush[3], err->flush[4], err->flush[5]);

		if (!mrq->cmd->error && !mrq->data->error)
			mrq->data->error = -EIO;

		msmsdcc_stop_data(host);
		/*
		 * In the case where we get a command timeout
		 * on a request which has already configured DMA
		 * (ie: a read), we won't want to send the
		 * STOP command since transmission hasn't
		 * started yet.
		 */
		if (!mrq->data->stop || mrq->cmd->error)
			msmsdcc_request_end(host, mrq);
		else
			msmsdcc_start_command(host, mrq->data->stop, 0);
	}

	if (host->dma.user_pages) {
		struct scatterlist *sg = host->dma.sg;
		int i;

		for (i = 0; i < host->dma.num_ents; i++, sg++)
			flush_dcache_page(sg_page(sg));
	}

	host->dma.sg = NULL;

	spin_unlock_irqrestore(&host->lock, flags);
	return;
}

static int msmsdcc_config_dma(struct msmsdcc_host *host, struct mmc_data *data)
{
	struct msmsdcc_nc_dmadata *nc;
	dmov_box *box;
	uint32_t rows;
	uint32_t crci;
	unsigned int n;
	int i;
	struct scatterlist *sg = data->sg;

	if (data->blksz < 32)
		return -EINVAL;
	if (host->dma.channel == -1)
		return -ENOENT;

	host->dma.sg = data->sg;
	host->dma.num_ents = data->sg_len;

	nc = host->dma.nc;

	if (host->pdev_id == 1)
		crci = MSMSDCC_CRCI_SDC1;
	else if (host->pdev_id == 2)
		crci = MSMSDCC_CRCI_SDC2;
	else if (host->pdev_id == 3)
		crci = MSMSDCC_CRCI_SDC3;
	else if (host->pdev_id == 4)
		crci = MSMSDCC_CRCI_SDC4;
	else {
		host->dma.sg = NULL;
		host->dma.num_ents = 0;
		return -ENOENT;
	}

	if (data->flags & MMC_DATA_READ)
		host->dma.dir = DMA_FROM_DEVICE;
	else
		host->dma.dir = DMA_TO_DEVICE;

	host->dma.user_pages = (data->flags & MMC_DATA_USERPAGE);

	n = dma_map_sg(mmc_dev(host->mmc), host->dma.sg,
			host->dma.num_ents, host->dma.dir);

	if (n != host->dma.num_ents) {
		printk(KERN_ERR "%s: Unable to map in all sg elements\n",
		       mmc_hostname(host->mmc));
		host->dma.sg = NULL;
		host->dma.num_ents = 0;
		return -ENOMEM;
	}

	box = &nc->cmd[0];
	for (i = 0; i < host->dma.num_ents; i++) {
		box->cmd = CMD_MODE_BOX;

		if (i == (host->dma.num_ents - 1))
			box->cmd |= CMD_LC;
		rows = (sg_dma_len(sg) % MCI_FIFOSIZE) ?
			(sg_dma_len(sg) / MCI_FIFOSIZE) + 1:
			(sg_dma_len(sg) / MCI_FIFOSIZE) ;

		if (data->flags & MMC_DATA_READ) {
			box->src_row_addr = msmsdcc_fifo_addr(host);
			box->dst_row_addr = sg_dma_address(sg);

			box->src_dst_len = (MCI_FIFOSIZE << 16) | (MCI_FIFOSIZE);
			box->row_offset = MCI_FIFOSIZE;

			box->num_rows = rows * ((1 << 16) + 1);
			box->cmd |= CMD_SRC_CRCI(crci);
		} else {
			box->src_row_addr = sg_dma_address(sg);
			box->dst_row_addr = msmsdcc_fifo_addr(host);

			box->src_dst_len = (MCI_FIFOSIZE << 16) | (MCI_FIFOSIZE);
			box->row_offset = (MCI_FIFOSIZE << 16);

			box->num_rows = rows * ((1 << 16) + 1);
			box->cmd |= CMD_DST_CRCI(crci);
		}
		box++;
		sg++;
	}

	/* location of command block must be 64 bit aligned */
	BUG_ON(host->dma.cmd_busaddr & 0x07);

	nc->cmdptr = (host->dma.cmd_busaddr >> 3) | CMD_PTR_LP;
	host->dma.hdr.cmdptr = DMOV_CMD_PTR_LIST |
			       DMOV_CMD_ADDR(host->dma.cmdptr_busaddr);
	host->dma.hdr.complete_func = msmsdcc_dma_complete_func;

	return 0;
}

static void
msmsdcc_start_data(struct msmsdcc_host *host, struct mmc_data *data)
{
	unsigned int datactrl, timeout, irqmask;
	unsigned long long clks;
	void __iomem *base = host->base;
	int rc;

	host->data = data;
	host->xfer_size = data->blksz * data->blocks;
	host->xfer_remain = host->xfer_size;
	host->data_xfered = 0;

	msmsdcc_init_sg(host, data);

	clks = (unsigned long long)data->timeout_ns * host->clk_rate;
	do_div(clks, 1000000000UL);
	timeout = data->timeout_clks + (unsigned int)clks;
	writel(timeout, base + MMCIDATATIMER);

	writel(host->xfer_size, base + MMCIDATALENGTH);

	datactrl = MCI_DPSM_ENABLE | (data->blksz << 4);

	rc = msmsdcc_config_dma(host, data);
	if (!rc)
		datactrl |= MCI_DPSM_DMAENABLE;

	if (data->flags & MMC_DATA_READ) {
		datactrl |= MCI_DPSM_DIRECTION;
		irqmask = MCI_RXFIFOHALFFULLMASK;

		/*
		 * If we have less than a FIFOSIZE of bytes to transfer,
		 * trigger a PIO interrupt as soon as any data is available.
		 */
		if (host->xfer_size < MCI_FIFOSIZE)
			irqmask |= MCI_RXDATAAVLBLMASK;
	} else {
		/*
		 * We don't actually need to include "FIFO empty" here
		 * since its implicit in "FIFO half empty".
		 */
		irqmask = MCI_TXFIFOHALFEMPTYMASK;
	}

	if (datactrl & MCI_DPSM_DMAENABLE)
		irqmask = 0;
	writel(irqmask, base + MMCIMASK1);
	writel(datactrl, base + MMCIDATACTRL);

	if (datactrl & MCI_DPSM_DMAENABLE)
		msm_dmov_enqueue_cmd(host->dma.channel, &host->dma.hdr);
}

static void
msmsdcc_start_command(struct msmsdcc_host *host, struct mmc_command *cmd, u32 c)
{
	void __iomem *base = host->base;

	DBG(host, "op %02x arg %08x flags %08x\n",
	    cmd->opcode, cmd->arg, cmd->flags);

	if (readl(base + MMCICOMMAND) & MCI_CPSM_ENABLE) {
		writel(0, base + MMCICOMMAND);
		udelay(2 + ((5 * 1000000) / host->clk_rate));
	}

	c |= cmd->opcode | MCI_CPSM_ENABLE;
	if (cmd->flags & MMC_RSP_PRESENT) {
		if (cmd->flags & MMC_RSP_136)
			c |= MCI_CPSM_LONGRSP;
		c |= MCI_CPSM_RESPONSE;
	}
	if (/*interrupt*/0)
		c |= MCI_CPSM_INTERRUPT;

	if ((((cmd->opcode == 17) || (cmd->opcode == 18))  ||
	     ((cmd->opcode == 24) || (cmd->opcode == 25))) ||
		(cmd->opcode == 53))
		c |= MCI_CSPM_DATCMD;

	if (cmd == cmd->mrq->stop)
		c |= MCI_CSPM_MCIABORT;

	host->cmd = cmd;

	writel(cmd->arg, base + MMCIARGUMENT);
	writel(c, base + MMCICOMMAND);

	mod_timer(&host->transaction_timer, jiffies + (HZ / 2));
}

static void
msmsdcc_data_irq(struct msmsdcc_host *host, struct mmc_data *data,
	      unsigned int status)
{
	if (status & MCI_DATABLOCKEND) {
		/*
		 * Don't use DATABLOCKEND as an indicator
		 * for progress during DMA since blocks
		 * come faster than we can respond to them.
		 */
		if (!host->dma.sg)
			host->data_xfered += data->blksz;
	}

	if (status & (MCI_DATACRCFAIL|MCI_DATATIMEOUT|MCI_TXUNDERRUN|
		      MCI_RXOVERRUN)) {
		if (status & MCI_DATACRCFAIL) {
			printk(KERN_ERR "%s: Data CRC error\n",
			       mmc_hostname(host->mmc));
			data->error = -EILSEQ;
		} else if (status & MCI_DATATIMEOUT) {
			printk(KERN_ERR "%s: Data timeout\n",
			       mmc_hostname(host->mmc));
			data->error = -ETIMEDOUT;
		} else if (status & (MCI_TXUNDERRUN|MCI_RXOVERRUN))
			data->error = -EIO;
		status |= MCI_DATAEND;

		/*
		 * We hit an error condition.  Ensure that any data
		 * partially written to a page is properly coherent.
		 */
		if (host->sg_len && (data->flags & MMC_DATA_READ)
				 && (data->flags & MMC_DATA_USERPAGE))
			flush_dcache_page(sg_page(host->sg_ptr));

		/*
		 * If a DMA was scheduled, then abort the transfer
		 * Once aborted, we return immediately; cleanup and
		 * stop command dispatch will happen from the
		 * completion handler
		 */
		if (host->dma.sg) {
			printk(KERN_ERR "%s: Aborting DMA operation for "
			       "MMC cmd 0x%p (data error)\n",
			       mmc_hostname(host->mmc), data->mrq->cmd);
			msm_dmov_stop_cmd(host->dma.channel, &host->dma.hdr);
			return;
		}
	}
	if (status & MCI_DATAEND) {

		if (!host->dma.sg) {
			/*
			 * There appears to be an issue in the SDCC where
			 * if you request a short block transfer, you may
			 * get your DATAEND/DATABLKEND irq before the PIO
			 * data. Check to see if there is still data to be
			 * read, and then simulate a PIO IRQ.
			 */
			uint32_t status2 = readl(host->base + MMCISTATUS);

			if (status2 & MCI_RXDATAAVLBL)
				msmsdcc_pio_irq(1, host);
		} else
			host->data_xfered = data->blksz * data->blocks;

		msmsdcc_stop_data(host);
		if (!data->stop) {
			msmsdcc_request_end(host, data->mrq);
		} else 
			msmsdcc_start_command(host, data->stop, 0);
	}
}

static void
msmsdcc_cmd_irq(struct msmsdcc_host *host, struct mmc_command *cmd,
	     unsigned int status)
{
	void __iomem *base = host->base;

	host->cmd = NULL;
	del_timer(&host->transaction_timer);

	cmd->resp[0] = readl(base + MMCIRESPONSE0);
	cmd->resp[1] = readl(base + MMCIRESPONSE1);
	cmd->resp[2] = readl(base + MMCIRESPONSE2);
	cmd->resp[3] = readl(base + MMCIRESPONSE3);

	if (status & MCI_CMDTIMEOUT) {
#if VERBOSE_COMMAND_TIMEOUTS
		printk(KERN_ERR "%s: Command timeout\n",
		       mmc_hostname(host->mmc));
#endif
		cmd->error = -ETIMEDOUT;
	} else if (status & MCI_CMDCRCFAIL && cmd->flags & MMC_RSP_CRC) {
		printk(KERN_ERR "%s: Command CRC error\n",
		       mmc_hostname(host->mmc));
		cmd->error = -EILSEQ;
	}

	if (!cmd->data || cmd->error) {
		if (host->data) {
			/*
			 * If the host has enqueued a DMA request
			 * then cancel it and return. request cleanup
			 * is handled in the dmov command completion
			 * handler.
			 */
			if (host->dma.sg) {
				printk(KERN_ERR
				       "%s: Aborting DMA operation for "
				       "MMC cmd 0x%p (command err)\n",
				       mmc_hostname(host->mmc), cmd);
				msm_dmov_stop_cmd(host->dma.channel,
						  &host->dma.hdr);
				return;
			} else
				msmsdcc_stop_data(host);
		}
		msmsdcc_request_end(host, cmd->mrq);
	} else if (!(cmd->data->flags & MMC_DATA_READ))
		msmsdcc_start_data(host, cmd->data);
}

static int
msmsdcc_pio_read(struct msmsdcc_host *host, char *buffer, unsigned int remain)
{
	void __iomem	*base = host->base;
	uint32_t	*ptr = (uint32_t *) buffer;
	int		count = 0;

	while (readl(base + MMCISTATUS) & MCI_RXDATAAVLBL) {

		*ptr = readl(base + MMCIFIFO + (count % MCI_FIFOSIZE));
		ptr++;
		count += sizeof(uint32_t);

		writel(0x018007ff, base + MMCISTATUS);

		remain -=  sizeof(uint32_t);
		if (remain == 0)
			break;
	}

	return count;
}

static int
msmsdcc_pio_write(struct msmsdcc_host *host, char *buffer,
		  unsigned int remain, u32 status)
{
	void __iomem *base = host->base;
	char *ptr = buffer;

	do {
		unsigned int count, maxcnt;

		maxcnt = status & MCI_TXFIFOEMPTY ? MCI_FIFOSIZE : MCI_FIFOHALFSIZE;
		count = min(remain, maxcnt);

		writesl(base + MMCIFIFO, ptr, count >> 2);
		ptr += count;
		remain -= count;

		if (remain == 0)
			break;

		status = readl(base + MMCISTATUS);
	} while (status & MCI_TXFIFOHALFEMPTY);

	return ptr - buffer;
}

/*
 * PIO data transfer IRQ handler.
 */
static irqreturn_t
msmsdcc_pio_irq(int irq, void *dev_id)
{
	struct msmsdcc_host *host = dev_id;
	void __iomem *base = host->base;
	struct mmc_data *data = host->data;
	u32 status;

	status = readl(base + MMCISTATUS);

	DBG(host, "irq1 %08x\n", status);

	WARN_ON(!data);

	do {
		unsigned long flags;
		unsigned int remain, len;
		char *buffer;

		/*
		 * For write, we only need to test the half-empty flag
		 * here - if the FIFO is completely empty, then by
		 * definition it is more than half empty.
		 *
		 * For read, check for data available.
		 */

		if (!(status & (MCI_TXFIFOHALFEMPTY|MCI_RXDATAAVLBL)))
			break;

		/*
		 * Map the current scatter buffer.
		 */
		buffer = msmsdcc_kmap_atomic(host, &flags) + host->sg_off;
		remain = host->sg_ptr->length - host->sg_off;

		len = 0;
		if (status & MCI_RXDATAAVLBL)
			len = msmsdcc_pio_read(host, buffer, remain);
		if (status & MCI_TXACTIVE)
			len = msmsdcc_pio_write(host, buffer, remain, status);

		/*
		 * Unmap the buffer.
		 */
		msmsdcc_kunmap_atomic(host, buffer, &flags);

		host->sg_off += len;
		host->xfer_remain -= len;
		remain -= len;

		if (remain)
			break;

		/*
		 * If we were reading, and we have completed this
		 * page, ensure that the data cache is coherent.
		 */
		if (status & MCI_RXACTIVE && data->flags & MMC_DATA_USERPAGE)
			flush_dcache_page(sg_page(host->sg_ptr));

		if (!msmsdcc_next_sg(host))
			break;

		status = readl(base + MMCISTATUS);
	} while (1);

	/*
	 * If we're nearing the end of the read, switch to
	 * "any data available" mode.
	 */
	if (status & MCI_RXACTIVE && host->xfer_remain < MCI_FIFOSIZE)
		writel(MCI_RXDATAAVLBLMASK, base + MMCIMASK1);

	/*
	 * If we run out of data, disable the data IRQs; this
	 * prevents a race where the FIFO becomes empty before
	 * the chip itself has disabled the data path, and
	 * stops us racing with our data end IRQ.
	 */
	if (host->xfer_remain == 0) {
		writel(0, base + MMCIMASK1);
		writel(readl(base + MMCIMASK0) | MCI_DATAENDMASK,
		       base + MMCIMASK0);
	}

	return IRQ_HANDLED;
}

/*
 * Handle completion of command and data transfers.
 */
static irqreturn_t
msmsdcc_irq(int irq, void *dev_id)
{
	struct msmsdcc_host *host = dev_id;
	u32 status;
	int ret = 0;

	spin_lock(&host->lock);

	WARN_ON(host->mrq == NULL);

	do {
		struct mmc_command *cmd;
		struct mmc_data *data;
		status = readl(host->base + MMCISTATUS);
		DBG(host, "irq0 %08x\n", status);

		status &= readl(host->base + MMCIMASK0);
		writel(status, host->base + MMCICLEAR);

		data = host->data;
		if (status & (MCI_DATACRCFAIL|MCI_DATATIMEOUT|MCI_TXUNDERRUN|
			      MCI_RXOVERRUN|MCI_DATAEND|MCI_DATABLOCKEND) &&
			      data) {
			msmsdcc_data_irq(host, data, status);
		}

		cmd = host->cmd;
		if (status & (MCI_CMDCRCFAIL|MCI_CMDTIMEOUT|MCI_CMDSENT|MCI_CMDRESPEND) && cmd)
			msmsdcc_cmd_irq(host, cmd, status);

		ret = 1;
	} while (status);

	spin_unlock(&host->lock);

	return IRQ_RETVAL(ret);
}

static void
msmsdcc_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct msmsdcc_host *host = mmc_priv(mmc);

	WARN_ON(host->mrq != NULL);

	spin_lock_irq(&host->lock);

	if (host->eject) {
		if (mrq->data && !(mrq->data->flags & MMC_DATA_READ)) {
			mrq->cmd->error = 0;
			mrq->data->bytes_xfered = mrq->data->blksz * mrq->data->blocks;
		} else
			mrq->cmd->error = -ENOMEDIUM;

		spin_unlock_irq(&host->lock);
		mmc_request_done(mmc, mrq);
		return;
	}

	host->mrq = mrq;

	if (mrq->data && mrq->data->flags & MMC_DATA_READ)
		msmsdcc_start_data(host, mrq->data);

	msmsdcc_start_command(host, mrq->cmd, 0);

	spin_unlock_irq(&host->lock);
}

static void
msmsdcc_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct msmsdcc_host *host = mmc_priv(mmc);
	u32 clk = 0, pwr = 0;
	int rc;

	if (ios->clock) {

		if (!host->clks_on) {
			clk_enable(host->pclk);
			clk_enable(host->clk);
			host->clks_on = 1;
		}
		if (ios->clock != host->clk_rate) {
			rc = clk_set_rate(host->clk, ios->clock);
			if (rc < 0)
				printk(KERN_ERR
				       "Error setting clock rate (%d)\n", rc);
			else
				host->clk_rate = ios->clock;
		}
		clk |= MCI_CLK_ENABLE;
	} 

	if (ios->bus_width == MMC_BUS_WIDTH_4)
		clk |= (2 << 10); /* Set WIDEBUS */

	clk |= (1 << 12); /* FLOW_ENA */
	clk |= (1 << 9); /* PWRSAVE */
	clk |= (1 << 15); /* feedback clock */

	if (host->plat->translate_vdd)
		pwr |= host->plat->translate_vdd(mmc_dev(mmc), ios->vdd);

	switch (ios->power_mode) {
	case MMC_POWER_OFF:
		break;
	case MMC_POWER_UP:
		pwr |= MCI_PWR_UP;
		break;
	case MMC_POWER_ON:
		pwr |= MCI_PWR_ON;
		break;
	}

	if (ios->bus_mode == MMC_BUSMODE_OPENDRAIN)
		pwr |= MCI_OD;

	writel(clk, host->base + MMCICLOCK);

	if (host->pwr != pwr) {
		host->pwr = pwr;
		writel(pwr, host->base + MMCIPOWER);
	}

	if (!(clk & MCI_CLK_ENABLE) && host->clks_on) {
		clk_disable(host->clk);
		clk_disable(host->pclk);
		host->clks_on = 0;
	}
}

static const struct mmc_host_ops msmsdcc_ops = {
	.request	= msmsdcc_request,
	.set_ios	= msmsdcc_set_ios,
};

static void
msmsdcc_check_status(unsigned long data)
{
	struct msmsdcc_host *host = (struct msmsdcc_host *)data;
	unsigned int status;

	if (!host->plat->status) {
		mmc_detect_change(host->mmc, 0);
		goto out;
	}

	status = host->plat->status(mmc_dev(host->mmc));
	host->eject = !status;
	if (status ^ host->oldstat) {
		printk(KERN_INFO
		       "%s: Slot status change detected (%d -> %d)\n",
		       mmc_hostname(host->mmc), host->oldstat, status);
		mmc_detect_change(host->mmc, 0);
	}

	host->oldstat = status;

out:
	if (host->timer.function)
		mod_timer(&host->timer, jiffies + HZ);
}

static irqreturn_t
msmsdcc_platform_status_irq(int irq, void *dev_id)
{
	struct msmsdcc_host *host = dev_id;
	msmsdcc_check_status((unsigned long) host);
	return IRQ_HANDLED;
}

static void
msmsdcc_status_notify_cb(int card_present, void *dev_id)
{
	struct msmsdcc_host *host = dev_id;

	printk("%s:\n", __func__);
	msmsdcc_check_status((unsigned long) host);
}

/*
 * called when a transaction expires.
 * Dump some debugging, and then error
 * out the transaction.
 */
static void
msmsdcc_transaction_expired(unsigned long _data)
{
	struct msmsdcc_host *host = (struct msmsdcc_host *) _data;
	struct mmc_request *mrq = NULL;
	struct mmc_command *cmd = NULL;
	struct mmc_data *data = NULL;
	unsigned long flags;

	spin_lock_irqsave(&host->lock, flags);

	if (!host->mrq) {
		printk(KERN_INFO "%s: Transaction expiry misfire\n",
		       mmc_hostname(host->mmc));
		spin_unlock_irqrestore(&host->lock, flags);
		return;
	}

	mrq = host->mrq;
	cmd = mrq->cmd;
	data = mrq->data;

	printk(KERN_ERR "%s: Transaction timeout (%p %p %p %p)\n",
	       mmc_hostname(host->mmc), mrq, cmd, data, host->dma.sg);

	if (host->dma.sg) {
		if (data)
			data->error = -ETIME;
		msm_dmov_stop_cmd(host->dma.channel, &host->dma.hdr);
		spin_unlock_irqrestore(&host->lock, flags);
		return;
	}

	if (cmd)
		cmd->error = -ETIME;
	msmsdcc_stop_data(host);
	msmsdcc_request_end(host, host->mrq);

	spin_unlock_irqrestore(&host->lock, flags);
}

static int
msmsdcc_init_dma(struct msmsdcc_host *host)
{
	memset(&host->dma, 0, sizeof(struct msmsdcc_dma_data));
	host->dma.host = host;
	host->dma.channel = -1;

	if (!host->dmares)
		return -ENODEV;

	host->dma.nc = dma_alloc_coherent(NULL,
					  sizeof(struct msmsdcc_nc_dmadata),
					  &host->dma.nc_busaddr,
					  GFP_KERNEL);
	if (host->dma.nc == NULL) {
		printk(KERN_ERR "Unable to allocate DMA buffer\n");
		return -ENOMEM;
	}
	printk(KERN_INFO
	       "%s: DM non-cached buffer at %p, dma_addr 0x%.8x\n",
	       __func__, host->dma.nc, host->dma.nc_busaddr);
	memset(host->dma.nc, 0x00, sizeof(struct msmsdcc_nc_dmadata));
	host->dma.cmd_busaddr = host->dma.nc_busaddr;
	host->dma.cmdptr_busaddr = host->dma.nc_busaddr +
				offsetof(struct msmsdcc_nc_dmadata, cmdptr);
	printk(KERN_INFO
	       "%s: DM cmd busaddr %u, cmdptr busaddr %u\n",
	       __func__, host->dma.cmd_busaddr,
	       host->dma.cmdptr_busaddr);
	host->dma.channel = host->dmares->start;

	return 0;
}

static int
msmsdcc_probe(struct platform_device *pdev)
{
	struct mmc_platform_data *plat = pdev->dev.platform_data;
	struct msmsdcc_host *host;
	struct mmc_host *mmc;
	struct resource *irqres = NULL;
	struct resource *memres = NULL;
	struct resource *dmares = NULL;
	int ret;
	int i;

	/* must have platform data */
	if (!plat) {
		printk(KERN_ERR "%s: Platform data not available\n", __func__);
		ret = -EINVAL;
		goto out;
	}

	if (pdev->id < 1 || pdev->id > 4)
		return -EINVAL;

	if (pdev->resource == NULL || pdev->num_resources < 2) {
		printk(KERN_ERR "%s: Invalid resource\n", __func__);
		return -ENXIO;
	}

	for (i = 0; i < pdev->num_resources; i++) {
		if (pdev->resource[i].flags & IORESOURCE_MEM)
			memres = &pdev->resource[i];
		if (pdev->resource[i].flags & IORESOURCE_IRQ)
			irqres = &pdev->resource[i];
		if (pdev->resource[i].flags & IORESOURCE_DMA)
			dmares = &pdev->resource[i];
	}
	if (!irqres || !memres) {
		printk(KERN_ERR "%s: Invalid resource\n", __func__);
		return -ENXIO;
	}

	/*
	 * Setup our host structure
	 */

	mmc = mmc_alloc_host(sizeof(struct msmsdcc_host), &pdev->dev);
	if (!mmc) {
		ret = -ENOMEM;
		goto out;
	}

	host = mmc_priv(mmc);
	host->pdev_id = pdev->id;
	host->plat = plat;
	host->mmc = mmc;
	host->base = memres->start;
	host->irqres = irqres;
	host->memres = memres;
	host->dmares = dmares;
	spin_lock_init(&host->lock);

#ifdef CONFIG_MMC_EMBEDDED_SDIO
	if (plat->embedded_sdio)
		mmc_set_embedded_sdio_data(mmc,
					   &plat->embedded_sdio->cis,
					   &plat->embedded_sdio->cccr,
					   plat->embedded_sdio->funcs,
					   plat->embedded_sdio->num_funcs);
#endif

	/*
	 * Setup DMA
	 */
	ret = msmsdcc_init_dma(host);
	if (ret)
		printk(KERN_ERR "%s: DMA setup failed (%d)\n",
		       __func__, ret);

	/*
	 * Setup main peripheral bus clock
	 */
	host->pclk = clk_get(&pdev->dev, msmsdcc_pclks[pdev->id]);
	if (IS_ERR(host->pclk)) {
		ret = PTR_ERR(host->pclk);
		goto host_free;
	}

	ret = clk_enable(host->pclk);
	if (ret)
		goto pclk_put;

	host->pclk_rate = clk_get_rate(host->pclk);

	/*
	 * Setup SDC MMC clock
	 */
	host->clk = clk_get(&pdev->dev, msmsdcc_clks[pdev->id]);
	if (IS_ERR(host->clk)) {
		ret = PTR_ERR(host->clk);
		goto pclk_disable;
	}

	ret = clk_enable(host->clk);
	if (ret)
		goto clk_put;

	ret = clk_set_rate(host->clk, msmsdcc_fmin);
	if (ret) {
		printk(KERN_ERR "%s: Clock rate set failed (%d)\n",
		       __func__, ret);
		goto clk_disable;
	}

	host->clk_rate = clk_get_rate(host->clk);

	host->clks_on = 1;

	/*
	 * Setup MMC host structure
	 */
	mmc->ops = &msmsdcc_ops;
	mmc->f_min = msmsdcc_fmin;
	mmc->f_max = msmsdcc_fmax;
	mmc->ocr_avail = plat->ocr_mask;
	mmc->caps = MMC_CAP_MULTIWRITE;

	if (msmsdcc_4bit)
		mmc->caps |= MMC_CAP_4_BIT_DATA;

	mmc->max_phys_segs = NR_SG;
	mmc->max_hw_segs = NR_SG;
	mmc->max_blk_size = 4096;	/* MCI_DATA_CTL BLOCKSIZE up to 4096 */
	mmc->max_blk_count = 65536;

	mmc->max_req_size = 33554432;	/* MCI_DATA_LENGTH is 25 bits */
	mmc->max_seg_size = mmc->max_req_size;
		
	writel(0, host->base + MMCIMASK0);
	writel(0, host->base + MMCIMASK1);
	writel(0x5c007ff, host->base + MMCICLEAR);

	writel(MCI_IRQENABLE, host->base + MMCIMASK0);

	/*
	 * Setup card detect change
	 */

	memset(&host->timer, 0, sizeof(host->timer));

	if (plat->status_irq) {
		ret = request_irq(plat->status_irq,
				  msmsdcc_platform_status_irq,
				  IRQF_SHARED,
				  DRIVER_NAME " (slot)",
				  host);
		if (ret) {
			printk(KERN_ERR "Unable to get slot IRQ %d (%d)\n",
			       plat->status_irq, ret);
			goto clk_disable;
		}
	} else if (plat->register_status_notify) {
		plat->register_status_notify(msmsdcc_status_notify_cb, host);
	} else if (!plat->status)
		printk(KERN_ERR "%s: No card detect facilities available\n",
		       mmc_hostname(mmc));
	else {
		init_timer(&host->timer);
		host->timer.data = (unsigned long)host;
		host->timer.function = msmsdcc_check_status;
		host->timer.expires = jiffies + HZ;
		add_timer(&host->timer);
	}

	if (plat->status) {
		host->oldstat = host->plat->status(mmc_dev(host->mmc));
		host->eject = !host->oldstat;
	}

	/*
	 * Setup a transaction timer. We currently need this due to
	 * some 'strange' timeout / error handling situations.
	 */
	init_timer(&host->transaction_timer);
	host->transaction_timer.data = (unsigned long) host;
	host->transaction_timer.function = msmsdcc_transaction_expired;

	ret = request_irq(irqres->start, msmsdcc_irq, IRQF_SHARED,
			  DRIVER_NAME " (cmd)", host);
	if (ret)
		goto platform_irq_free;

	ret = request_irq(irqres->end, msmsdcc_pio_irq, IRQF_SHARED,
			  DRIVER_NAME " (pio)", host);
	if (ret)
		goto irq0_free;

	mmc_set_drvdata(pdev, mmc);
	mmc_add_host(mmc);

	printk(KERN_INFO
	       "%s: Qualcomm MSM SDCC at 0x%016llx irq %d,%d,%d dma %d\n",
	       mmc_hostname(mmc), (unsigned long long)memres->start,
	       (unsigned int) irqres->start, (unsigned int)irqres->end,
	       (unsigned int) plat->status_irq, host->dma.channel);
	printk(KERN_INFO "%s: 4 bit data mode %s\n", mmc_hostname(mmc),
	       (mmc->caps & MMC_CAP_4_BIT_DATA ? "enabled" : "disabled"));
	printk(KERN_INFO "%s: MMC clock %u -> %u Hz, PCLK %u Hz\n",
	       mmc_hostname(mmc), msmsdcc_fmin, msmsdcc_fmax, host->pclk_rate);
	if (host->timer.function)
		printk(KERN_INFO "%s: Polling status mode enabled\n",
		       mmc_hostname(mmc));

#if defined(CONFIG_DEBUG_FS)
	msmsdcc_dbg_createhost(host);
#endif
	return 0;

 irq0_free:
	free_irq(irqres->start, host);
 platform_irq_free:
	if (plat->status_irq)
		free_irq(plat->status_irq, host);
 clk_disable:
	clk_disable(host->clk);
 clk_put:
	clk_put(host->clk);
 pclk_disable:
	clk_disable(host->pclk);
 pclk_put:
	clk_put(host->pclk);
 host_free:
	mmc_free_host(mmc);
 out:
	return ret;
}

static int
msmsdcc_suspend(struct platform_device *dev, pm_message_t state)
{
	struct mmc_host *mmc = mmc_get_drvdata(dev);
	int rc = 0;

	if (mmc) {
		struct msmsdcc_host *host = mmc_priv(mmc);

		if (mmc->card && mmc->card->type != MMC_TYPE_SDIO)
			rc = mmc_suspend_host(mmc, state);
		if (!rc) {
			writel(0, host->base + MMCIMASK0);

			if (host->clks_on) {
				clk_disable(host->clk);
				clk_disable(host->pclk);
				host->clks_on = 0;
			}
		}
	}
	return rc;
}

static int
msmsdcc_resume(struct platform_device *dev)
{
	struct mmc_host *mmc = mmc_get_drvdata(dev);
	int rc = 0;

	if (mmc) {
		struct msmsdcc_host *host = mmc_priv(mmc);

		if (!host->clks_on) {
			clk_enable(host->pclk);
			clk_enable(host->clk);
			host->clks_on = 1;
		}

		writel(MCI_IRQENABLE, host->base + MMCIMASK0);
		if (mmc->card && mmc->card->type != MMC_TYPE_SDIO)
			rc = mmc_resume_host(mmc);
	}
	return rc;
}

static struct platform_driver msmsdcc_driver = {
	.probe		= msmsdcc_probe,
	.suspend	= msmsdcc_suspend,
	.resume		= msmsdcc_resume,
	.driver		= {
		.name	= "msm_sdcc",
	},
};

static int __init msmsdcc_init(void)
{
	return platform_driver_register(&msmsdcc_driver);
}

static void __exit msmsdcc_exit(void)
{
	platform_driver_unregister(&msmsdcc_driver);
}

static int __init msmsdcc_4bit_setup(char *__unused)
{
	msmsdcc_4bit = 1;
	return 1;
}

static int __init msmsdcc_1bit_setup(char *__unused)
{
	msmsdcc_4bit = 0;
	return 1;
}

static int __init msmsdcc_fmin_setup(char *str)
{
	unsigned int n;

	if (!get_option(&str, &n))
		return 0;
	msmsdcc_fmin = n;
	return 1;
}

static int __init msmsdcc_fmax_setup(char *str)
{
	unsigned int n;

	if (!get_option(&str, &n))
		return 0;
	msmsdcc_fmax = n;
	return 1;
}

__setup("msmsdcc_4bit", msmsdcc_4bit_setup);
__setup("msmsdcc_1bit", msmsdcc_1bit_setup);
__setup("msmsdcc_fmin=", msmsdcc_fmin_setup);
__setup("msmsdcc_fmax=", msmsdcc_fmax_setup);

module_init(msmsdcc_init);
module_exit(msmsdcc_exit);
module_param(msmsdcc_fmin, uint, 0444);
module_param(msmsdcc_fmax, uint, 0444);
module_param(msmsdcc_4bit, uint, 0444);

MODULE_DESCRIPTION("Qualcomm MSM 7X00A Multimedia Card Interface driver");
MODULE_LICENSE("GPL");

#if defined(CONFIG_DEBUG_FS)

static int 
msmsdcc_dbg_state_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t
msmsdcc_dbg_state_read(struct file *file, char __user *ubuf,
		       size_t count, loff_t *ppos)
{
	struct msmsdcc_host *host = (struct msmsdcc_host *) file->private_data;
	char buf[1024];
	int max, i;

	i = 0;
	max = sizeof(buf) -1;

	i += scnprintf(buf + i, max - i, "STAT: %p %p %p\n", host->mrq, host->cmd,
		       host->data);
	if (host->cmd) {
		struct mmc_command *cmd = host->cmd;

		i+= scnprintf(buf + i, max - i, "CMD : %.8x %.8x %.8x\n",
			      cmd->opcode, cmd->arg, cmd->flags);
	}
	if (host->data) {
		struct mmc_data *data = host->data;
		i+= scnprintf(buf + i, max - i, "DAT0: %.8x %.8x %.8x %.8x %.8x %.8x\n",
			      data->timeout_ns, data->timeout_clks,
			      data->blksz, data->blocks, data->error,
			      data->flags);
		i+= scnprintf(buf + i, max - i, "DAT1: %.8x %.8x %.8x %p\n",
			      host->xfer_size, host->xfer_remain,
			      host->data_xfered, host->dma.sg);
	}

	return simple_read_from_buffer(ubuf, count, ppos, buf, i);
}

static const struct file_operations msmsdcc_dbg_state_ops = {
	.read	= msmsdcc_dbg_state_read,
	.open	= msmsdcc_dbg_state_open,
};

static void msmsdcc_dbg_createhost(struct msmsdcc_host *host)
{
	if (debugfs_dir) {
		debugfs_create_file(mmc_hostname(host->mmc), 0644, debugfs_dir,
				    host, &msmsdcc_dbg_state_ops);
	}
}

static int __init msmsdcc_dbg_init(void)
{
	int err;

	debugfs_dir = debugfs_create_dir("msmsdcc", 0);
	if (IS_ERR(debugfs_dir)) {
		err = PTR_ERR(debugfs_dir);
		debugfs_dir = NULL;
		return err;
	}

	return 0;
}

device_initcall(msmsdcc_dbg_init);
#endif
