/* drivers/mtd/devices/msm_nand.c
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <asm/dma-mapping.h>

#include <asm/io.h>
#include <asm/mach/flash.h>
#include <asm/arch/dma.h>

#define MSM_NAND_BASE 0xA0A00000
#include "msm_nand.h"

#define MSM_NAND_DMA_BUFFER_SIZE SZ_4K
#define MSM_NAND_DMA_BUFFER_SLOTS \
	(MSM_NAND_DMA_BUFFER_SIZE / (sizeof(((atomic_t *)0)->counter) * 8))

#define VERBOSE 0

struct msm_nand_chip {
	struct device *dev;
	wait_queue_head_t wait_queue;
	atomic_t dma_buffer_busy;
	unsigned dma_channel;
	uint8_t *dma_buffer;
	dma_addr_t dma_addr;
	unsigned CFG0, CFG1;
};

#define CFG1_WIDE_FLASH (1U << 1)

/* TODO: move datamover code out */

#define SRC_CRCI_NAND_CMD  CMD_SRC_CRCI(DMOV_NAND_CRCI_CMD)
#define DST_CRCI_NAND_CMD  CMD_DST_CRCI(DMOV_NAND_CRCI_CMD)
#define SRC_CRCI_NAND_DATA CMD_SRC_CRCI(DMOV_NAND_CRCI_DATA)
#define DST_CRCI_NAND_DATA CMD_DST_CRCI(DMOV_NAND_CRCI_DATA)

#define msm_virt_to_dma(chip, vaddr) ((void)(*(vaddr)), (chip)->dma_addr + ((uint8_t *)(vaddr) - (chip)->dma_buffer))

/**
 * msm_nand_oob_64 - oob info for large (2KB) page
 */
static struct nand_ecclayout msm_nand_oob_64 = {
	.eccbytes	= 40,
	.eccpos		= {
		0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09,
		0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19,
		0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29,
		0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39,
		},
	.oobfree	= {
		{0xa, 5}, {0x1a, 5}, {0x2a, 5}, {0x3a, 5},
	}
};

static void *msm_nand_get_dma_buffer(struct msm_nand_chip *chip, size_t size)
{
	unsigned int bitmask, free_bitmask, old_bitmask;
	unsigned int need_mask, current_need_mask;
	int free_index;

	need_mask = (1UL << DIV_ROUND_UP(size, MSM_NAND_DMA_BUFFER_SLOTS)) - 1;
	bitmask = atomic_read(&chip->dma_buffer_busy);
	free_bitmask = ~bitmask;
	do {
		free_index = __ffs(free_bitmask);
		current_need_mask = need_mask << free_index;
		if ((bitmask & current_need_mask) == 0) {
			old_bitmask = atomic_cmpxchg(&chip->dma_buffer_busy, bitmask,
			                             bitmask | current_need_mask);
			if (old_bitmask == bitmask)
				return chip->dma_buffer + free_index * MSM_NAND_DMA_BUFFER_SLOTS;
			free_bitmask = 0; /* force return */
		}
		/* current free range was too small, clear all free bits */
		/* below the top busy bit within current_need_mask */
		free_bitmask &= ~(~0U >> (32 - fls(bitmask & current_need_mask)));
	} while(free_bitmask);

	return NULL;
}

static void msm_nand_release_dma_buffer(struct msm_nand_chip *chip, void *buffer, size_t size)
{
	int index;
	unsigned int used_mask;

	used_mask = (1UL << DIV_ROUND_UP(size, MSM_NAND_DMA_BUFFER_SLOTS)) - 1;
	index = ((uint8_t *)buffer - chip->dma_buffer) / MSM_NAND_DMA_BUFFER_SLOTS;
	atomic_sub(used_mask << index, &chip->dma_buffer_busy);

	wake_up(&chip->wait_queue);
}

uint32_t flash_read_id(struct msm_nand_chip *chip)
{
	struct {
		dmov_s cmd[5];
		unsigned cmdptr;
		unsigned data[5];
	} *dma_buffer;
	uint32_t rv;

	wait_event(chip->wait_queue, (dma_buffer = msm_nand_get_dma_buffer(chip, sizeof(*dma_buffer))));

	dma_buffer->data[0] = 0 | 4;
	dma_buffer->data[1] = NAND_CMD_FETCH_ID;
	dma_buffer->data[2] = 1;
	dma_buffer->data[3] = 0xeeeeeeee;
	dma_buffer->data[4] = 0xeeeeeeee;
	BUILD_BUG_ON(4 != ARRAY_SIZE(dma_buffer->data) - 1);

	dma_buffer->cmd[0].cmd = 0 | CMD_OCB;
	dma_buffer->cmd[0].src = msm_virt_to_dma(chip, &dma_buffer->data[0]);
	dma_buffer->cmd[0].dst = NAND_FLASH_CHIP_SELECT;
	dma_buffer->cmd[0].len = 4;

	dma_buffer->cmd[1].cmd = DST_CRCI_NAND_CMD;
	dma_buffer->cmd[1].src = msm_virt_to_dma(chip, &dma_buffer->data[1]);
	dma_buffer->cmd[1].dst = NAND_FLASH_CMD;
	dma_buffer->cmd[1].len = 4;

	dma_buffer->cmd[2].cmd = 0;
	dma_buffer->cmd[2].src = msm_virt_to_dma(chip, &dma_buffer->data[2]);
	dma_buffer->cmd[2].dst = NAND_EXEC_CMD;
	dma_buffer->cmd[2].len = 4;

	dma_buffer->cmd[3].cmd = SRC_CRCI_NAND_DATA;
	dma_buffer->cmd[3].src = NAND_FLASH_STATUS;
	dma_buffer->cmd[3].dst = msm_virt_to_dma(chip, &dma_buffer->data[3]);
	dma_buffer->cmd[3].len = 4;

	dma_buffer->cmd[4].cmd = CMD_OCU | CMD_LC;
	dma_buffer->cmd[4].src = NAND_READ_ID;
	dma_buffer->cmd[4].dst = msm_virt_to_dma(chip, &dma_buffer->data[4]);
	dma_buffer->cmd[4].len = 4;
	BUILD_BUG_ON(4 != ARRAY_SIZE(dma_buffer->cmd) - 1);

	dma_buffer->cmdptr = (msm_virt_to_dma(chip, dma_buffer->cmd) >> 3) | CMD_PTR_LP;

	msm_dmov_exec_cmd(chip->dma_channel, DMOV_CMD_PTR_LIST | DMOV_CMD_ADDR(msm_virt_to_dma(chip, &dma_buffer->cmdptr)));

	printk("status: %x\n", dma_buffer->data[3]);
	printk("nandid: %x maker %02x device %02x\n",
	       dma_buffer->data[4], dma_buffer->data[4] & 0xff, (dma_buffer->data[4] >> 8) & 0xff);
	rv = dma_buffer->data[4];
	msm_nand_release_dma_buffer(chip, dma_buffer, sizeof(*dma_buffer));
	return rv;
}

int flash_read_config(struct msm_nand_chip *chip)
{
	struct {
		dmov_s cmd[2];
		unsigned cmdptr;
		unsigned cfg0;
		unsigned cfg1;
	} *dma_buffer;

	wait_event(chip->wait_queue, (dma_buffer = msm_nand_get_dma_buffer(chip, sizeof(*dma_buffer))));
	dma_buffer->cfg0 = 0;
	dma_buffer->cfg1 = 0;

	dma_buffer->cmd[0].cmd = CMD_OCB;
	dma_buffer->cmd[0].src = NAND_DEV0_CFG0;
	dma_buffer->cmd[0].dst = msm_virt_to_dma(chip, &dma_buffer->cfg0);
	dma_buffer->cmd[0].len = 4;

	dma_buffer->cmd[1].cmd = CMD_OCU | CMD_LC;
	dma_buffer->cmd[1].src = NAND_DEV0_CFG1;
	dma_buffer->cmd[1].dst = msm_virt_to_dma(chip, &dma_buffer->cfg1);
	dma_buffer->cmd[1].len = 4;
	BUILD_BUG_ON(1 != ARRAY_SIZE(dma_buffer->cmd) - 1);

	dma_buffer->cmdptr = (msm_virt_to_dma(chip, dma_buffer->cmd) >> 3) | CMD_PTR_LP;

	msm_dmov_exec_cmd(chip->dma_channel, DMOV_CMD_PTR_LIST | DMOV_CMD_ADDR(msm_virt_to_dma(chip, &dma_buffer->cmdptr)));

	chip->CFG0 = dma_buffer->cfg0;
	chip->CFG1 = dma_buffer->cfg1;
	printk("read CFG0 = %x, CFG1 = %x\n", chip->CFG0, chip->CFG1);
	/* set 4 codeword per page for 2k nand */
	chip->CFG0 = (chip->CFG0 & ~0x1c0) | (3 << 6);
	/* 5 spare bytes */
	chip->CFG0 = (chip->CFG0 & ~(0xf << 23)) | (5 << 23);
	/* set bad block marker location and enable ECC */
	chip->CFG1 = (chip->CFG1 & ~0x1ffc1) | (465 << 6);

	msm_nand_release_dma_buffer(chip, dma_buffer, sizeof(*dma_buffer));

	if ((chip->CFG0 == 0) || (chip->CFG1 == 0)) {
		return -1;
	}

	return 0;
}

unsigned flash_rd_reg(struct msm_nand_chip *chip, unsigned addr)
{
	struct {
		dmov_s cmd;
		unsigned cmdptr;
		unsigned data;
	} *dma_buffer;
	unsigned rv;

	wait_event(chip->wait_queue, (dma_buffer = msm_nand_get_dma_buffer(chip, sizeof(*dma_buffer))));

	dma_buffer->cmd.cmd = CMD_LC;
	dma_buffer->cmd.src = addr;
	dma_buffer->cmd.dst = msm_virt_to_dma(chip, &dma_buffer->data);
	dma_buffer->cmd.len = 4;

	dma_buffer->cmdptr = (msm_virt_to_dma(chip, &dma_buffer->cmd) >> 3) | CMD_PTR_LP;
	dma_buffer->data = 0xeeeeeeee;

	msm_dmov_exec_cmd(chip->dma_channel, DMOV_CMD_PTR_LIST | DMOV_CMD_ADDR(msm_virt_to_dma(chip, &dma_buffer->cmdptr)));
	rv = dma_buffer->data;

	msm_nand_release_dma_buffer(chip, dma_buffer, sizeof(*dma_buffer));

	return rv;
}

void flash_wr_reg(struct msm_nand_chip *chip, unsigned addr, unsigned val)
{
	struct {
		dmov_s cmd;
		unsigned cmdptr;
		unsigned data;
	} *dma_buffer;

	wait_event(chip->wait_queue, (dma_buffer = msm_nand_get_dma_buffer(chip, sizeof(*dma_buffer))));

	dma_buffer->cmd.cmd = CMD_LC;
	dma_buffer->cmd.src = msm_virt_to_dma(chip, &dma_buffer->data);
	dma_buffer->cmd.dst = addr;
	dma_buffer->cmd.len = 4;

	dma_buffer->cmdptr = (msm_virt_to_dma(chip, &dma_buffer->cmd) >> 3) | CMD_PTR_LP;
	dma_buffer->data = val;

	msm_dmov_exec_cmd(chip->dma_channel, DMOV_CMD_PTR_LIST | DMOV_CMD_ADDR(msm_virt_to_dma(chip, &dma_buffer->cmdptr)));

	msm_nand_release_dma_buffer(chip, dma_buffer, sizeof(*dma_buffer));
}

static int
msm_nand_read_oob_only(struct msm_nand_chip *chip, loff_t from, struct mtd_oob_ops *ops)
{
	struct {
		dmov_s cmd[4 * 4 + 4];
		unsigned cmdptr;
		struct {
			struct {
				uint32_t cmd;
				uint32_t addr0;
				uint32_t addr1;
				uint32_t chipsel;
			} cmd;
			struct {
				uint32_t cmd;
				uint32_t addr0;
			} cmdn[3];
			struct {
				uint32_t cfg0;
				uint32_t cfg1;
			} cfg;
			uint32_t cfg0_1;
			uint32_t exec;
			uint32_t devcmd1_tmp;
			uint32_t devcmd1_restore;
			struct {
				uint32_t flash_status;
				uint32_t buffer_status;
			} result[4];
		} data;
	} *dma_buffer;
	dmov_s *cmd;
	unsigned n;
	unsigned page = from / 2048;
	uint32_t oob_len = ops->ooblen;
	uint32_t sectoroobsize = 5;
	int err, rawerr;
	dma_addr_t oob_dma_addr = 0;
	dma_addr_t oob_dma_addr_curr = 0;
	uint32_t oob_col = 0;
	unsigned page_count;
	unsigned pages_read = 0;

	if (ops->mode != MTD_OOB_AUTO) {
		sectoroobsize = 15;
		page_count = ops->ooblen / 64;
	} else {
		page_count = 1;
	}

	/* printk("msm_nand_read_oob %x %x %x\n", from, ops->len, ops->ooblen); */
#if VERBOSE
	printk("msm_nand_read_oob_only %llx %p %x %p %x\n", from, ops->datbuf, ops->len, ops->oobbuf, ops->ooblen);
#endif

	memset(ops->oobbuf, 0xff, ops->ooblen);
	oob_dma_addr_curr = oob_dma_addr = dma_map_single(chip->dev, ops->oobbuf, ops->ooblen, DMA_BIDIRECTIONAL);
	/* oob_dma_addr = dma_map_single(chip->dev, ops->oobbuf, ops->ooblen, DMA_FROM_DEVICE); */
	if (dma_mapping_error(oob_dma_addr)) {
		printk("msm_nand_read_oob: failed to get dma addr for %p\n", ops->oobbuf);
		err = -EIO;
		goto err_dma_map_oobbuf_failed;
	}

	wait_event(chip->wait_queue, (dma_buffer = msm_nand_get_dma_buffer(chip, sizeof(*dma_buffer))));

	while (page_count-- > 0) {
		cmd = dma_buffer->cmd;

		/* GO bit for the EXEC register */
		dma_buffer->data.exec = 1;

		dma_buffer->data.devcmd1_tmp = 0xF00FE005;
		dma_buffer->data.devcmd1_restore = 0xF00F3000;

		oob_col = 0x200;
		if (ops->mode == MTD_OOB_AUTO)
			oob_col += 10;

		if (chip->CFG1 & CFG1_WIDE_FLASH) /* (assuming 2k nand) */
			oob_col += 2; /* two bad block markers */
		else
			oob_col += 1; /* one bad block marker */

		/* CMD / ADDR0 / ADDR1 / CHIPSEL program values */
		dma_buffer->data.cmd.cmd = NAND_CMD_PAGE_READ;
		dma_buffer->data.cmd.addr0 = (page << 16) | oob_col;
		/* qc example is (page >> 16) && 0xff !? */
		dma_buffer->data.cmd.addr1 = (page >> 16) & 0xff;
		/* flash0 + datamover enable */
		dma_buffer->data.cmd.chipsel = 0 | 4;

		/* SPARE_SIZE_BYTES, ECC_PARITY_SIZE_BYTES, 
		 * UD_SIZE_BYTES, CW_PER_PAGE
		 */
		dma_buffer->data.cfg.cfg0 = 
			(chip->CFG0 & ~((0xfU << 23) | (0xfU << 19) | 
					(0x3ffU << 9) | (7U << 6))) |
			(sectoroobsize << 9);
		/* bad block byte = 0, disable ECC, bad block in spare */
		dma_buffer->data.cfg.cfg1 =
			(chip->CFG1 & ~(0x3fU << 6)) | 1 | (1U << 16);
		/* NUM_ADDR_CYCLES 2 col addr cycles for 2k nand; */
		dma_buffer->data.cfg0_1 =
			(dma_buffer->data.cfg.cfg0 & ~(7U << 27)) |
			(2 << 27);

		BUILD_BUG_ON(4 != ARRAY_SIZE(dma_buffer->data.result));
		for(n = 0; n < 4; n++) {

			/* flash + buffer status return words */
			dma_buffer->data.result[n].flash_status = 0xeeeeeeee;
			dma_buffer->data.result[n].buffer_status = 0xeeeeeeee;

				/* block on cmd ready, then
				** write CMD / ADDR0 / ADDR1 / CHIPSEL regs in a burst
				*/
			if (n == 0) {
				cmd->cmd = DST_CRCI_NAND_CMD;
				cmd->src = msm_virt_to_dma(chip, &dma_buffer->data.cmd);
				cmd->dst = NAND_FLASH_CMD;
				cmd->len = 16;
				cmd++;

				cmd->cmd = 0;
				cmd->src = msm_virt_to_dma(chip, &dma_buffer->data.cfg);
				cmd->dst = NAND_DEV0_CFG0;
				cmd->len = 8;
				cmd++;
			} else {
				dma_buffer->data.cmdn[n - 1].cmd = NAND_CMD_PAGE_READ_ECC;
				dma_buffer->data.cmdn[n - 1].addr0 = (page << 16) | oob_col;

				cmd->cmd = DST_CRCI_NAND_CMD;
				cmd->src = msm_virt_to_dma(chip, &dma_buffer->data.cmdn[n - 1]);
				cmd->dst = NAND_FLASH_CMD;
				cmd->len = 8;
				cmd++;

				if (n == 1) {
					cmd->cmd = 0;
					cmd->src = msm_virt_to_dma(chip, &dma_buffer->data.cfg0_1);
					cmd->dst = NAND_DEV0_CFG0;
					cmd->len = 4;
					cmd++;

					cmd->cmd = 0;
					cmd->src = msm_virt_to_dma(chip, &dma_buffer->data.devcmd1_tmp);
					cmd->dst = NAND_DEV_CMD1;
					cmd->len = 4;
					cmd++;
				}
			}


				/* kick the execute register */
			cmd->cmd = 0;
			cmd->src = msm_virt_to_dma(chip, &dma_buffer->data.exec);
			cmd->dst = NAND_EXEC_CMD;
			cmd->len = 4;
			cmd++;

				/* block on data ready, then
				** read the status register
				*/
			cmd->cmd = SRC_CRCI_NAND_DATA;
			cmd->src = NAND_FLASH_STATUS;
			cmd->dst = msm_virt_to_dma(chip, &dma_buffer->data.result[n]);
			/* NAND_FLASH_STATUS + NAND_BUFFER_STATUS */
			cmd->len = 8;
			cmd++;

			cmd->cmd = 0;
			cmd->src = NAND_FLASH_BUFFER;

			cmd->dst = oob_dma_addr_curr;
			if (sectoroobsize < oob_len) {
				cmd->len = sectoroobsize + (ops->mode != MTD_OOB_AUTO /*add fake bad block marker*/);
			} else
				cmd->len = oob_len;
			oob_dma_addr_curr += cmd->len;
			oob_len -= cmd->len;
			if (cmd->len > 0)
				cmd++;
			oob_col += 528;
		}
		cmd->cmd = 0;
		cmd->src = msm_virt_to_dma(chip, &dma_buffer->data.devcmd1_restore);
		cmd->dst = NAND_DEV_CMD1;
		cmd->len = 4;
		cmd++;

		BUILD_BUG_ON(4 * 4 + 4 != ARRAY_SIZE(dma_buffer->cmd));
		BUG_ON(cmd - dma_buffer->cmd > ARRAY_SIZE(dma_buffer->cmd));
		dma_buffer->cmd[0].cmd |= CMD_OCB;
		cmd[-1].cmd |= CMD_OCU | CMD_LC;

		dma_buffer->cmdptr = (msm_virt_to_dma(chip, dma_buffer->cmd) >> 3) | CMD_PTR_LP;

		msm_dmov_exec_cmd(chip->dma_channel, DMOV_CMD_PTR_LIST | DMOV_CMD_ADDR(msm_virt_to_dma(chip, &dma_buffer->cmdptr)));

			/* if any of the writes failed (0x10), or there was a
			** protection violation (0x100), we lose
			*/
		err = rawerr = 0;
		for(n = 0; n < 4; n++) {
			if (dma_buffer->data.result[n].flash_status & 0x110) {
				rawerr = -EIO;
				break;
			}
		}
		if (rawerr) {
			for(n = 0; n < ops->ooblen; n++) {
				if (ops->oobbuf[n] != 0xff) {
					err = rawerr;
					break;
				}
			}
		}

#if VERBOSE
		if (rawerr && !err) {
			printk("msm_nand_read_oob %llx %x %x empty page\n", (loff_t)page * 2048, ops->len, ops->ooblen);
		} else {
			printk("status: %x %x %x %x %x %x %x %x\n",
			       dma_buffer->data.result[0].flash_status,
			       dma_buffer->data.result[0].buffer_status,
			       dma_buffer->data.result[1].flash_status,
			       dma_buffer->data.result[1].buffer_status,
			       dma_buffer->data.result[2].flash_status,
			       dma_buffer->data.result[2].buffer_status,
			       dma_buffer->data.result[3].flash_status,
			       dma_buffer->data.result[3].buffer_status);
		}
#endif
		if (err)
			break;
		pages_read++;
		page++;
	}
	msm_nand_release_dma_buffer(chip, dma_buffer, sizeof(*dma_buffer));

	dma_unmap_single(chip->dev, oob_dma_addr, ops->ooblen, DMA_FROM_DEVICE);
err_dma_map_oobbuf_failed:

#if VERBOSE
	if (!rawerr || err) { /* not empty */
		unsigned *ptr;
		ptr = (unsigned*) ops->oobbuf;
		printk("oob data:	%x %x %x %x\n",
		       ptr[0], ptr[1], ptr[2], ptr[3]);
	}
#endif

	if (err) {
		printk("msm_nand_read_oob %llx %x %x failed\n", from, ops->len, ops->ooblen);
	} else {
		ops->retlen = 0;
		ops->oobretlen = ops->ooblen - oob_len;
	}

	return err;
}

static int
msm_nand_read_oob(struct mtd_info *mtd, loff_t from, struct mtd_oob_ops *ops)
{
	struct msm_nand_chip *chip = mtd->priv;

	struct {
		dmov_s cmd[4 * 5 + 1];
		unsigned cmdptr;
		struct {
			uint32_t cmd;
			uint32_t addr0;
			uint32_t addr1;
			uint32_t chipsel;
			uint32_t cfg0;
			uint32_t cfg1;
			uint32_t exec;
			struct {
				uint32_t flash_status;
				uint32_t buffer_status;
			} result[4];
		} data;
	} *dma_buffer;
	dmov_s *cmd;
	unsigned n;
	unsigned page = from / 2048;
	uint32_t oob_len = ops->ooblen;
	uint32_t sectoroobsize = 5;
	int err, rawerr;
	dma_addr_t data_dma_addr = 0;
	dma_addr_t oob_dma_addr = 0;
	dma_addr_t data_dma_addr_curr = 0;
	dma_addr_t oob_dma_addr_curr = 0;
	unsigned page_count;
	unsigned pages_read = 0;

	if (from & (mtd->writesize - 1)) {
		printk("%s: unsupported from, 0x%llx\n",
		       __FUNCTION__, from);
		return -EINVAL;
	}
	if (ops->datbuf != NULL && (ops->len % mtd->writesize) != 0) {
		/* when ops->datbuf is NULL, ops->len may refer to ooblen */
		printk("%s: unsupported ops->len, %d\n",
		       __FUNCTION__, ops->len);
		return -EINVAL;
	}
	if (ops->ooblen != 0 && ops->ooboffs != 0) {
		printk("%s: unsupported ops->ooboffs, %d\n",
		       __FUNCTION__, ops->ooboffs);
		return -EINVAL;
	}

	if (ops->oobbuf && !ops->datbuf)
		return msm_nand_read_oob_only(chip, from, ops);

	if (ops->mode != MTD_OOB_AUTO)
		sectoroobsize = 16;
	page_count = ops->len / mtd->writesize;

#if 0 /* yaffs reads more oob data than it needs */
	if (ops->ooblen >= sectoroobsize * 4) {
		printk("%s: unsupported ops->ooblen, %d\n",
		       __FUNCTION__, ops->ooblen);
		return -EINVAL;
	}
#endif

	/* printk("msm_nand_read_oob %x %x %x\n", from, ops->len, ops->ooblen); */
#if VERBOSE
	printk("msm_nand_read_oob %llx %p %x %p %x\n", from, ops->datbuf, ops->len, ops->oobbuf, ops->ooblen);
#endif
	if (ops->datbuf) {
		/* memset(ops->datbuf, 0x55, ops->len); */
		data_dma_addr_curr = data_dma_addr = dma_map_single(chip->dev, ops->datbuf, ops->len, DMA_FROM_DEVICE);
		if (dma_mapping_error(data_dma_addr)) {
			printk("msm_nand_read_oob: failed to get dma addr for %p\n", ops->datbuf);
			return -EIO;
		}
	}
	if (ops->oobbuf) {
		memset(ops->oobbuf, 0xff, ops->ooblen);
		oob_dma_addr_curr = oob_dma_addr = dma_map_single(chip->dev, ops->oobbuf, ops->ooblen, DMA_BIDIRECTIONAL);
		/* oob_dma_addr = dma_map_single(chip->dev, ops->oobbuf, ops->ooblen, DMA_FROM_DEVICE); */
		if (dma_mapping_error(oob_dma_addr)) {
			printk("msm_nand_read_oob: failed to get dma addr for %p\n", ops->oobbuf);
			err = -EIO;
			goto err_dma_map_oobbuf_failed;
		}
	}

	wait_event(chip->wait_queue, (dma_buffer = msm_nand_get_dma_buffer(chip, sizeof(*dma_buffer))));

	while (page_count-- > 0) {
		cmd = dma_buffer->cmd;

		/* CMD / ADDR0 / ADDR1 / CHIPSEL program values */
		if (ops->oobbuf)
			dma_buffer->data.cmd = NAND_CMD_PAGE_READ_ALL;
		else
			dma_buffer->data.cmd = NAND_CMD_PAGE_READ; /* ecc? */
		dma_buffer->data.addr0 = (page << 16);
		/* qc example is (page >> 16) && 0xff !? */
		dma_buffer->data.addr1 = (page >> 16) & 0xff;
		/* flash0 + undoc bit */
		dma_buffer->data.chipsel = 0 | 4;


		dma_buffer->data.cfg0 = chip->CFG0;
		dma_buffer->data.cfg1 = chip->CFG1;

		/* GO bit for the EXEC register */
		dma_buffer->data.exec = 1;


		BUILD_BUG_ON(4 != ARRAY_SIZE(dma_buffer->data.result));

		for(n = 0; n < 4; n++) {
			/* flash + buffer status return words */
			dma_buffer->data.result[n].flash_status = 0xeeeeeeee;
			dma_buffer->data.result[n].buffer_status = 0xeeeeeeee;

				/* block on cmd ready, then
				 * write CMD / ADDR0 / ADDR1 / CHIPSEL
				 * regs in a burst
				 */
			cmd->cmd = DST_CRCI_NAND_CMD;
			cmd->src = msm_virt_to_dma(chip, &dma_buffer->data.cmd);
			cmd->dst = NAND_FLASH_CMD;
			if (n == 0)
				cmd->len = 16;
			else
				cmd->len = 4;
			cmd++;

			if (n == 0) {
				cmd->cmd = 0;
				cmd->src = msm_virt_to_dma(chip, &dma_buffer->data.cfg0);
				cmd->dst = NAND_DEV0_CFG0;
				cmd->len = 8;
				cmd++;
			}

				/* kick the execute register */
			cmd->cmd = 0;
			cmd->src = msm_virt_to_dma(chip, &dma_buffer->data.exec);
			cmd->dst = NAND_EXEC_CMD;
			cmd->len = 4;
			cmd++;

				/* block on data ready, then
				** read the status register
				*/
			cmd->cmd = SRC_CRCI_NAND_DATA;
			cmd->src = NAND_FLASH_STATUS;
			cmd->dst = msm_virt_to_dma(chip, &dma_buffer->data.result[n]);
			/* NAND_FLASH_STATUS + NAND_BUFFER_STATUS */
			cmd->len = 8;
			cmd++;

				/* read data block
				 * (only valid if status says success)
				 */
			if (ops->datbuf) {
				cmd->cmd = 0;
				cmd->src = NAND_FLASH_BUFFER;
				/* cmd->dst = virt_to_phys(data_addr + n * 512); */
				cmd->dst = data_dma_addr_curr;
				data_dma_addr_curr += 512;
				cmd->len = 512;
				cmd++;
			}

			if (ops->oobbuf) {
				cmd->cmd = 0;
				if (ops->mode == MTD_OOB_AUTO)
					cmd->src = NAND_FLASH_BUFFER + 512 + 10;
				else
					cmd->src = NAND_FLASH_BUFFER + 512;

				/* cmd->dst = virt_to_phys(oob_addr + n * oobsize); */
				cmd->dst = oob_dma_addr_curr;
				if (sectoroobsize < oob_len)
					cmd->len = sectoroobsize;
				else
					cmd->len = oob_len;
				oob_dma_addr_curr += cmd->len;
				oob_len -= cmd->len;
				if (cmd->len > 0)
					cmd++;
			}
		}

		BUILD_BUG_ON(4 * 5 + 1 != ARRAY_SIZE(dma_buffer->cmd));
		BUG_ON(cmd - dma_buffer->cmd > ARRAY_SIZE(dma_buffer->cmd));
		dma_buffer->cmd[0].cmd |= CMD_OCB;
		cmd[-1].cmd |= CMD_OCU | CMD_LC;

		dma_buffer->cmdptr = (msm_virt_to_dma(chip, dma_buffer->cmd) >> 3) | CMD_PTR_LP;

		msm_dmov_exec_cmd(chip->dma_channel, DMOV_CMD_PTR_LIST | DMOV_CMD_ADDR(msm_virt_to_dma(chip, &dma_buffer->cmdptr)));

			/* if any of the writes failed (0x10), or there was a
			** protection violation (0x100), we lose
			*/
		err = rawerr = 0;
		for(n = 0; n < 4; n++) {
			if (dma_buffer->data.result[n].flash_status & 0x110) {
				rawerr = -EIO;
				break;
			}
		}
		if (rawerr) {
			if (ops->datbuf) {
				for(n = 0; n < 2048; n++) {
					/* empty blocks read 0x76 at 
					 * these offsets
					 */
					if (n % 512 == 243)
						ops->datbuf[n] = 0xff;
					if (ops->datbuf[n] != 0xff) {
						/* printk("msm_nand_read_oob %llx %x %x byte at %d not 0xff, 0x%x\n", (loff_t)page * mtd->writesize, ops->len, ops->ooblen, n, ops->datbuf[n]); */
						err = rawerr;
						break;
					}
				}
			}
			if (ops->oobbuf) {
				for(n = 0; n < ops->ooblen; n++) {
					if (ops->oobbuf[n] != 0xff) {
						err = rawerr;
						break;
					}
				}
#if 0
				if (n == ops->ooblen && err) {
					printk("msm_nand_read_oob %llx %x %x: memset data to 0xff\n", (loff_t)page * mtd->writesize, ops->len, ops->ooblen);
					memset(ops->datbuf, 0xff, 2048);
					err = 0;
				}
#endif
			}
		}

#if VERBOSE
		if (rawerr && !err) {
			printk("msm_nand_read_oob %llx %x %x empty page\n", (loff_t)page * mtd->writesize, ops->len, ops->ooblen);
		} else {
			printk("status: %x %x %x %x %x %x %x %x\n",
			       dma_buffer->data.result[0].flash_status,
			       dma_buffer->data.result[0].buffer_status,
			       dma_buffer->data.result[1].flash_status,
			       dma_buffer->data.result[1].buffer_status,
			       dma_buffer->data.result[2].flash_status,
			       dma_buffer->data.result[2].buffer_status,
			       dma_buffer->data.result[3].flash_status,
			       dma_buffer->data.result[3].buffer_status);
		}
#endif
		if (err)
			break;
		pages_read++;
		page++;
	}
	msm_nand_release_dma_buffer(chip, dma_buffer, sizeof(*dma_buffer));

	if (ops->oobbuf) {
		dma_unmap_single(chip->dev, oob_dma_addr, ops->ooblen, DMA_FROM_DEVICE);
	}
err_dma_map_oobbuf_failed:
	if (ops->datbuf) {
		dma_unmap_single(chip->dev, data_dma_addr, ops->len, DMA_FROM_DEVICE);
	}

#if VERBOSE
	if (!rawerr || err) { /* not empty */
		unsigned *ptr;
		if (ops->datbuf) {
			ptr = (unsigned*) ops->datbuf;
			printk("data:	%x %x %x %x\n", ptr[0], ptr[1], ptr[2], ptr[3]);
		}
		if (ops->oobbuf) {
			ptr = (unsigned*) ops->oobbuf;
			printk("oob data:	%x %x %x %x\n", ptr[0], ptr[1], ptr[2], ptr[3]);
		}
	}
#endif

	if (err) {
		printk("msm_nand_read_oob %llx %x %x failed\n", from, ops->len, ops->ooblen);
	} else {
		ops->retlen = mtd->writesize * pages_read;
		ops->oobretlen = ops->ooblen - oob_len;
	}

	return err;
}

static int
msm_nand_read(struct mtd_info *mtd, loff_t from, size_t len,
              size_t *retlen, u_char *buf)
{
	int ret;
	struct mtd_oob_ops ops;

	/* printk("msm_nand_read %llx %x\n", from, len); */

	ops.mode = MTD_OOB_PLACE;
	ops.len = len;
	ops.retlen = 0;
	ops.ooblen = 0;
	ops.datbuf = buf;
	ops.oobbuf = NULL;
	ret =  msm_nand_read_oob(mtd, from, &ops);
	*retlen = ops.retlen;
	/* printk("msm_nand_read %llx %x, returned %d, retlen %d\n", from, len, ret, ops.retlen); */
	return ret;
}

static int
msm_nand_write_oob(struct mtd_info *mtd, loff_t to, struct mtd_oob_ops *ops)
{
	struct msm_nand_chip *chip = mtd->priv;
	struct {
		dmov_s cmd[4 * 5 + 1];
		unsigned cmdptr;
		struct {
			uint32_t cmd;
			uint32_t addr0;
			uint32_t addr1;
			uint32_t chipsel;
			uint32_t cfg0;
			uint32_t cfg1;
			uint32_t exec;
			uint32_t flash_status[4];
		} data;
	} *dma_buffer;
	dmov_s *cmd;
	unsigned n;
	unsigned page = to / 2048;
	uint32_t oob_len = ops->ooblen;
	uint32_t sectoroobsize = 5;
	uint32_t sectoroobwritesize = 5;
	int err;
	dma_addr_t data_dma_addr = 0;
	dma_addr_t oob_dma_addr = 0;
	dma_addr_t data_dma_addr_curr = 0;
	dma_addr_t oob_dma_addr_curr = 0;
	unsigned page_count;
	unsigned pages_written = 0;

	if (ops->mode != MTD_OOB_AUTO)
		sectoroobsize = 16;

	if (to & (mtd->writesize - 1)) {
		printk("%s: unsupported to, 0x%llx\n", __FUNCTION__, to);
		return -EINVAL;
	}
	if (ops->ooblen != 0 && ops->mode != MTD_OOB_AUTO) {
		printk("%s: unsupported ops->mode, %d\n", __FUNCTION__, ops->mode);
		return -EINVAL;
	}

	if (ops->datbuf == NULL) {
		printk("%s: unsupported ops->datbuf == NULL\n", __FUNCTION__);
		return -EINVAL;
	}
	if ((ops->len % mtd->writesize) != 0) {
		printk("%s: unsupported ops->len, %d\n", __FUNCTION__, ops->len);
		return -EINVAL;
	}
#if 0 /* yaffs writes more oob data than it needs */
	if (ops->ooblen >= sectoroobsize * 4) {
		printk("%s: unsupported ops->ooblen, %d\n", __FUNCTION__, ops->ooblen);
		return -EINVAL;
	}
#endif
	if (ops->ooblen != 0 && ops->ooboffs != 0) {
		printk("%s: unsupported ops->ooboffs, %d\n", __FUNCTION__, ops->ooboffs);
		return -EINVAL;
	}

	if (ops->datbuf) {
		data_dma_addr_curr = data_dma_addr = dma_map_single(chip->dev, ops->datbuf, ops->len, DMA_TO_DEVICE);
		if (dma_mapping_error(data_dma_addr)) {
			printk("msm_nand_write_oob: failed to get dma addr for %p\n", ops->datbuf);
			return -EIO;
		}
	}
	if (ops->oobbuf) {
		oob_dma_addr_curr = oob_dma_addr = dma_map_single(chip->dev, ops->oobbuf, ops->ooblen, DMA_TO_DEVICE);
		if (dma_mapping_error(oob_dma_addr)) {
			printk("msm_nand_write_oob: failed to get dma addr for %p\n", ops->oobbuf);
			err = -EIO;
			goto err_dma_map_oobbuf_failed;
		}
	}

	page_count = ops->len / mtd->writesize;

	wait_event(chip->wait_queue, (dma_buffer = msm_nand_get_dma_buffer(chip, sizeof(*dma_buffer))));

	while (page_count-- > 0) {
		cmd = dma_buffer->cmd;

			/* CMD / ADDR0 / ADDR1 / CHIPSEL program values */
		if (ops->oobbuf) {
			dma_buffer->data.cmd = NAND_CMD_PRG_PAGE_ALL;
		} else {
			dma_buffer->data.cmd = NAND_CMD_PRG_PAGE;
		}
		dma_buffer->data.addr0 = page << 16;
		dma_buffer->data.addr1 = (page >> 16) & 0xff;	 /* qc example is (page >> 16) && 0xff !? */
		dma_buffer->data.chipsel= 0 | 4; /* flash0 + undoc bit */

		dma_buffer->data.cfg0= chip->CFG0;
		dma_buffer->data.cfg1= chip->CFG1;

			/* GO bit for the EXEC register */
		dma_buffer->data.exec = 1;

		BUILD_BUG_ON(4 != ARRAY_SIZE(dma_buffer->data.flash_status));

		for(n = 0; n < 4; n++) {
			/* status return words */
			dma_buffer->data.flash_status[n] = 0xeeeeeeee;
				/* block on cmd ready, then
				** write CMD / ADDR0 / ADDR1 / CHIPSEL regs in a burst
				*/
			cmd->cmd = DST_CRCI_NAND_CMD;
			cmd->src = msm_virt_to_dma(chip, &dma_buffer->data.cmd);
			cmd->dst = NAND_FLASH_CMD;
			if (n == 0)
				cmd->len = 16;
			else
				cmd->len = 4;
			cmd++;

			if (n == 0) {
				cmd->cmd = 0;
				cmd->src = msm_virt_to_dma(chip, &dma_buffer->data.cfg0);
				cmd->dst = NAND_DEV0_CFG0;
				cmd->len = 8;
				cmd++;
			}

				/* write data block */
			cmd->cmd = 0;
			cmd->src = data_dma_addr_curr;
			data_dma_addr_curr += 512;
			cmd->dst = NAND_FLASH_BUFFER;
			cmd->len = 512;
			cmd++;

			if (ops->oobbuf) {
				cmd->cmd = 0;
				cmd->src = oob_dma_addr_curr;
				cmd->dst = NAND_FLASH_BUFFER + 512;
				if (sectoroobwritesize < oob_len)
					cmd->len = sectoroobwritesize;
				else
					cmd->len = oob_len;
				oob_dma_addr_curr += cmd->len;
				oob_len -= cmd->len;
				if (cmd->len > 0)
					cmd++;
			}

				/* kick the execute register */
			cmd->cmd = 0;
			cmd->src = msm_virt_to_dma(chip, &dma_buffer->data.exec);
			cmd->dst = NAND_EXEC_CMD;
			cmd->len = 4;
			cmd++;

				/* block on data ready, then
				** read the status register
				*/
			cmd->cmd = SRC_CRCI_NAND_DATA;
			cmd->src = NAND_FLASH_STATUS;
			cmd->dst = msm_virt_to_dma(chip, &dma_buffer->data.flash_status[n]);
			cmd->len = 4;
			cmd++;
		}
		dma_buffer->cmd[0].cmd |= CMD_OCB;
		cmd[-1].cmd |= CMD_OCU | CMD_LC;
		BUILD_BUG_ON(4 * 5 + 1 != ARRAY_SIZE(dma_buffer->cmd));
		BUG_ON(cmd - dma_buffer->cmd > ARRAY_SIZE(dma_buffer->cmd));
		dma_buffer->cmdptr = (msm_virt_to_dma(chip, dma_buffer->cmd) >> 3) | CMD_PTR_LP;

		msm_dmov_exec_cmd(chip->dma_channel, DMOV_CMD_PTR_LIST | DMOV_CMD_ADDR(msm_virt_to_dma(chip, &dma_buffer->cmdptr)));

			/* if any of the writes failed (0x10), or there was a
			** protection violation (0x100), or the program success
			** bit (0x80) is unset, we lose
			*/
		err = 0;
		for(n = 0; n < 4; n++) {
			if (dma_buffer->data.flash_status[n] & 0x110) {
				err = -EIO;
				break;
			}
			if (!(dma_buffer->data.flash_status[n] & 0x80)) {
				err = -EIO;
				break;
			}
		}

#if VERBOSE
		printk("write page %d: status: %x %x %x %x\n", page,
		       dma_buffer->data.flash_status[0], dma_buffer->data.flash_status[1],
		       dma_buffer->data.flash_status[2], dma_buffer->data.flash_status[3]);
#endif
		if (err)
			break;
		pages_written++;
		page++;
	}
	ops->retlen = mtd->writesize * pages_written;
	ops->oobretlen = ops->ooblen - oob_len;

	msm_nand_release_dma_buffer(chip, dma_buffer, sizeof(*dma_buffer));

	if (ops->oobbuf) {
		dma_unmap_single(chip->dev, oob_dma_addr, ops->ooblen, DMA_TO_DEVICE);
	}
err_dma_map_oobbuf_failed:
	if (ops->datbuf) {
		dma_unmap_single(chip->dev, data_dma_addr, 2048, DMA_TO_DEVICE);
	}

	return err;
}

static int
msm_nand_write(struct mtd_info *mtd, loff_t to, size_t len,
               size_t *retlen, const u_char *buf)
{
	int ret;
	struct mtd_oob_ops ops;

	ops.mode = MTD_OOB_PLACE;
	ops.len = len;
	ops.retlen = 0;
	ops.ooblen = 0;
	ops.datbuf = (uint8_t *)buf;
	ops.oobbuf = NULL;
	ret =  msm_nand_write_oob(mtd, to, &ops);
	*retlen = ops.retlen;
	return ret;
}

static int
msm_nand_erase(struct mtd_info *mtd, struct erase_info *instr)
{
	int err;
	struct msm_nand_chip *chip = mtd->priv;
	struct {
		dmov_s cmd[4];
		unsigned cmdptr;
		unsigned data[8];
	} *dma_buffer;
	unsigned page = instr->addr / 2048;

	if (instr->addr & (mtd->erasesize - 1)) {
		printk("%s: unsupported erase address, 0x%x\n", __FUNCTION__, instr->addr);
		return -EINVAL;
	}
	if (instr->len != mtd->erasesize) {
		printk("%s: unsupported erase len, %d\n", __FUNCTION__, instr->len);
		return -EINVAL;
	}

	wait_event(chip->wait_queue, (dma_buffer = msm_nand_get_dma_buffer(chip, sizeof(*dma_buffer))));

	dma_buffer->data[0] = NAND_CMD_BLOCK_ERASE;
	dma_buffer->data[1] = page;
	dma_buffer->data[2] = 0;
	dma_buffer->data[3] = 0 | 4;
	dma_buffer->data[4] = 1;
	dma_buffer->data[5] = 0xeeeeeeee;
	dma_buffer->data[6] = chip->CFG0 & (~(7 << 6));  /* CW_PER_PAGE = 0 */
	dma_buffer->data[7] = chip->CFG1;
	BUILD_BUG_ON(7 != ARRAY_SIZE(dma_buffer->data) - 1);

	dma_buffer->cmd[0].cmd = DST_CRCI_NAND_CMD | CMD_OCB;
	dma_buffer->cmd[0].src = msm_virt_to_dma(chip, &dma_buffer->data[0]);
	dma_buffer->cmd[0].dst = NAND_FLASH_CMD;
	dma_buffer->cmd[0].len = 16;

	dma_buffer->cmd[1].cmd = 0;
	dma_buffer->cmd[1].src = msm_virt_to_dma(chip, &dma_buffer->data[6]);
	dma_buffer->cmd[1].dst = NAND_DEV0_CFG0;
	dma_buffer->cmd[1].len = 8;

	dma_buffer->cmd[2].cmd = 0;
	dma_buffer->cmd[2].src = msm_virt_to_dma(chip, &dma_buffer->data[4]);
	dma_buffer->cmd[2].dst = NAND_EXEC_CMD;
	dma_buffer->cmd[2].len = 4;

	dma_buffer->cmd[3].cmd = SRC_CRCI_NAND_DATA | CMD_OCU | CMD_LC;;
	dma_buffer->cmd[3].src = NAND_FLASH_STATUS;
	dma_buffer->cmd[3].dst = msm_virt_to_dma(chip, &dma_buffer->data[5]);
	dma_buffer->cmd[3].len = 4;

	BUILD_BUG_ON(3 != ARRAY_SIZE(dma_buffer->cmd) - 1);

	dma_buffer->cmdptr = (msm_virt_to_dma(chip, dma_buffer->cmd) >> 3) | CMD_PTR_LP;

	msm_dmov_exec_cmd(chip->dma_channel, DMOV_CMD_PTR_LIST | DMOV_CMD_ADDR(msm_virt_to_dma(chip, &dma_buffer->cmdptr)));

#if VERBOSE
	printk("status: %x\n", dma_buffer->data[5]);
#endif

		/* we fail if there was an operation error, a mpu error, or the
		** erase success bit was not set.
		*/

	if (dma_buffer->data[5] & 0x110 || !(dma_buffer->data[5] & 0x80))
		err = -EIO;
	else
		err = 0;

	msm_nand_release_dma_buffer(chip, dma_buffer, sizeof(*dma_buffer));
	if (err) {
		printk("%s: erase failed, 0x%x\n", __FUNCTION__, instr->addr);
		instr->fail_addr = instr->addr;
		instr->state = MTD_ERASE_FAILED;
	} else {
		instr->state = MTD_ERASE_DONE;
		instr->fail_addr = 0xffffffff;
		mtd_erase_callback(instr);
	}
	return err;
}

/* static void msm_nand_sync(struct mtd_info *mtd) */
/* { */
/* } */


static int
msm_nand_block_isbad(struct mtd_info *mtd, loff_t ofs)
{
	/* Check for invalid offset */
	if (ofs > mtd->size)
		return -EINVAL;

	return 0;
}


static int
msm_nand_block_markbad(struct mtd_info *mtd, loff_t ofs)
{
	/* struct msm_nand_chip *this = mtd->priv; */
	int ret;

	ret = msm_nand_block_isbad(mtd, ofs);
	if (ret) {
		/* If it was bad already, return success and do nothing */
		if (ret > 0)
			return 0;
		return ret;
	}

	return -EIO;
}

/* static int msm_nand_unlock(struct mtd_info *mtd, loff_t ofs, size_t len) */
/* { */
/* } */


/* static int msm_nand_unlock_all(struct mtd_info *mtd) */
/* { */
/* } */



/**
 * msm_nand_suspend - [MTD Interface] Suspend the msm_nand flash
 * @param mtd		MTD device structure
 */
static int msm_nand_suspend(struct mtd_info *mtd)
{
	return 0;
}

/**
 * msm_nand_resume - [MTD Interface] Resume the msm_nand flash
 * @param mtd		MTD device structure
 */
static void msm_nand_resume(struct mtd_info *mtd)
{
}

/**
 * msm_nand_scan - [msm_nand Interface] Scan for the msm_nand device
 * @param mtd		MTD device structure
 * @param maxchips	Number of chips to scan for
 *
 * This fills out all the not initialized function pointers
 * with the defaults.
 * The flash ID is read and the mtd/chip structures are
 * filled with the appropriate values.
 */
int msm_nand_scan(struct mtd_info *mtd, int maxchips)
{
	unsigned n;
	struct msm_nand_chip *chip = mtd->priv;
	uint32_t flash_id;


	if (flash_read_config(chip /*flash_cmdlist, flash_ptrlist*/)) {
		printk("ERRROR: could not save CFG0 & CFG1 state\n");
		return -ENODEV;
	}
	printk("CFG0 = %x, CFG1 = %x\n", chip->CFG0, chip->CFG1);
	printk("CFG0: cw/page=%d ud_sz=%d ecc_sz=%d spare_sz=%d\n",
	       (chip->CFG0 >> 6) & 7, (chip->CFG0 >> 9) & 0x3ff, (chip->CFG0 >> 19) & 15, (chip->CFG0 >> 23) & 15);

	printk("NAND_READ_ID = %x\n", flash_rd_reg(chip, NAND_READ_ID));
	flash_wr_reg(chip, NAND_READ_ID, 0x12345678);

	flash_id = flash_read_id(chip /*flash_cmdlist, flash_ptrlist*/);

	n = flash_rd_reg(chip, NAND_DEV0_CFG0);
	printk("CFG0: cw/page=%d ud_sz=%d ecc_sz=%d spare_sz=%d\n",
	       (n >> 6) & 7, (n >> 9) & 0x3ff, (n >> 19) & 15, (n >> 23) & 15);

	n = flash_rd_reg(chip, NAND_DEV_CMD1);
	printk("DEV_CMD1: %x\n", n);

	if ((flash_id & 0xffff) == 0xaaec) /* 2Gbit Samsung chip */
		mtd->size = 256 << 20; /* * num_chips */

	mtd->writesize = 2048;
	mtd->oobsize = mtd->writesize >> 5; /* TODO: check */
	mtd->erasesize = mtd->writesize << 6; /* TODO: check */
	mtd->ecclayout = &msm_nand_oob_64;

	/* Fill in remaining MTD driver data */
	mtd->type = MTD_NANDFLASH;
	mtd->flags = MTD_CAP_NANDFLASH;
	/* mtd->ecctype = MTD_ECC_SW; */
	mtd->erase = msm_nand_erase;
	mtd->point = NULL;
	mtd->unpoint = NULL;
	mtd->read = msm_nand_read;
	mtd->write = msm_nand_write;
	mtd->read_oob = msm_nand_read_oob;
	mtd->write_oob = msm_nand_write_oob;
	/* mtd->sync = msm_nand_sync; */
	mtd->lock = NULL;
	/* mtd->unlock = msm_nand_unlock; */
	mtd->suspend = msm_nand_suspend;
	mtd->resume = msm_nand_resume;
	mtd->block_isbad = msm_nand_block_isbad;
	mtd->block_markbad = msm_nand_block_markbad;
	mtd->owner = THIS_MODULE;

	/* Unlock whole block */
	/* msm_nand_unlock_all(mtd); */

	/* return this->scan_bbt(mtd); */
	return 0;
}

/**
 * msm_nand_release - [msm_nand Interface] Free resources held by the msm_nand device
 * @param mtd		MTD device structure
 */
void msm_nand_release(struct mtd_info *mtd)
{
	/* struct msm_nand_chip *this = mtd->priv; */

#ifdef CONFIG_MTD_PARTITIONS
	/* Deregister partitions */
	del_mtd_partitions (mtd);
#endif
	/* Deregister the device */
	del_mtd_device (mtd);
}

EXPORT_SYMBOL_GPL(msm_nand_scan);
EXPORT_SYMBOL_GPL(msm_nand_release);

#ifdef CONFIG_MTD_PARTITIONS
static const char *part_probes[] = { "cmdlinepart", NULL,  };
#endif

struct msm_nand_info {
	struct mtd_info		mtd;
	struct mtd_partition	*parts;
	struct msm_nand_chip	msm_nand;
};

static int __devinit msm_nand_probe(struct platform_device *pdev)
{
	struct msm_nand_info *info;
	struct flash_platform_data *pdata = pdev->dev.platform_data;
	int err;

	if (pdev->num_resources != 1) {
		printk("invalid num_resources");
		return -ENODEV;
	}
	if (pdev->resource[0].flags != IORESOURCE_DMA) {
		printk("invalid resource type");
		return -ENODEV;
	}

	info = kzalloc(sizeof(struct msm_nand_info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->msm_nand.dev = &pdev->dev;

	init_waitqueue_head(&info->msm_nand.wait_queue);

	info->msm_nand.dma_channel = pdev->resource[0].start;
	/* this currently fails if dev is passed in */
	info->msm_nand.dma_buffer = dma_alloc_coherent(/*dev*/ NULL, MSM_NAND_DMA_BUFFER_SIZE, &info->msm_nand.dma_addr, GFP_KERNEL);
	if (info->msm_nand.dma_buffer == NULL) {
		err = -ENOMEM;
		goto out_free_info;
	}

	printk("allocated dma buffer at %p, dma_addr %x\n", info->msm_nand.dma_buffer, info->msm_nand.dma_addr);

	info->mtd.name = pdev->dev.bus_id;
	info->mtd.priv = &info->msm_nand;
	info->mtd.owner = THIS_MODULE;

	if (msm_nand_scan(&info->mtd, 1)) {
		err = -ENXIO;
		goto out_free_dma_buffer;
	}

#ifdef CONFIG_MTD_PARTITIONS
	err = parse_mtd_partitions(&info->mtd, part_probes, &info->parts, 0);
	if (err > 0)
		add_mtd_partitions(&info->mtd, info->parts, err);
	else if (err < 0 && pdata && pdata->parts)
		add_mtd_partitions(&info->mtd, pdata->parts, pdata->nr_parts);
	else
#endif
		err = add_mtd_device(&info->mtd);

	dev_set_drvdata(&pdev->dev, info);

	return 0;

out_free_dma_buffer:
	dma_free_coherent(/*dev*/ NULL, SZ_4K, info->msm_nand.dma_buffer, info->msm_nand.dma_addr);
out_free_info:
	kfree(info);

	return err;
}

static int __devexit msm_nand_remove(struct platform_device *pdev)
{
	struct msm_nand_info *info = dev_get_drvdata(&pdev->dev);

	dev_set_drvdata(&pdev->dev, NULL);

	if (info) {
#ifdef CONFIG_MTD_PARTITIONS
		if (info->parts)
			del_mtd_partitions(&info->mtd);
		else
#endif
			del_mtd_device(&info->mtd);

		msm_nand_release(&info->mtd);
		dma_free_coherent(/*dev*/ NULL, SZ_4K, info->msm_nand.dma_buffer, info->msm_nand.dma_addr);
		kfree(info);
	}

	return 0;
}

#define DRIVER_NAME "msm_nand"

static struct platform_driver msm_nand_driver = {
	.probe		= msm_nand_probe,
	.remove		= __devexit_p(msm_nand_remove),
	.driver = {
		.name		= DRIVER_NAME,
	}
};

MODULE_ALIAS(DRIVER_NAME);

static int __init msm_nand_init(void)
{
	return platform_driver_register(&msm_nand_driver);
}

static void __exit msm_nand_exit(void)
{
	platform_driver_unregister(&msm_nand_driver);
}

module_init(msm_nand_init);
module_exit(msm_nand_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("msm_nand flash driver code");
