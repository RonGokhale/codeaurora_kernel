/* drivers/video/msm_fb/mdp.c
 *
 * MSM MDP Interface (used by framebuffer core)
 *
 * Copyright (C) 2007 QUALCOMM Incorporated
 * Copyright (C) 2007 Google Incorporated
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/wait.h>

#include <asm/io.h>
#include <asm/arch/msm_iomap.h>

#include "mdp_hw.h"

#define MDP_CMD_DEBUG_ACCESS_BASE (MSM_MDP_BASE + 0x10000)

static uint16_t mdp_default_ccs[] = {
	0x254, 0x000, 0x331, 0x254, 0xF38, 0xE61, 0x254, 0x409, 0x000,
	0x010, 0x080, 0x080
};

/* setup color conversion coefficients */
void mdp_set_ccs(uint16_t *ccs)
{
	int n;
	for (n = 0; n < 9; n++)
		writel(ccs[n], MSM_MDP_BASE + 0x40440 + 4 * n);
	writel(ccs[9], MSM_MDP_BASE + 0x40500 + 4 * 0);
	writel(ccs[10], MSM_MDP_BASE + 0x40500 + 4 * 0);
	writel(ccs[11], MSM_MDP_BASE + 0x40500 + 4 * 0);
}

static volatile int mdp_dma2_busy;

static DECLARE_WAIT_QUEUE_HEAD(mdp_dma2_waitqueue);

static irqreturn_t mdp_isr(int irq, void *data)
{
	uint32_t status;

	status = readl(MDP_INTR_STATUS);
	writel(status, MDP_INTR_CLEAR);

	if (status & DL0_DMA2_TERM_DONE) {
		mdp_dma2_busy = 0;
		wake_up(&mdp_dma2_waitqueue);
	}
	return IRQ_HANDLED;
}

void mdp_dma_wait(void)
{
	int r = wait_event_timeout(mdp_dma2_waitqueue, !mdp_dma2_busy, HZ);
	if (r <= 0)
		printk(KERN_ERR "mdp_dma_wait: timeout waiting for dma to complete\n");
}

void mdp_dma_to_mddi(uint32_t addr, uint32_t stride, uint32_t width, uint32_t height, uint32_t x, uint32_t y)
{

	uint32_t dma2_cfg;
	uint16_t ld_param = 0; /* 0=PRIM, 1=SECD, 2=EXT */

	if (mdp_dma2_busy) return;

	mdp_dma2_busy = 1;

	dma2_cfg = DMA_PACK_TIGHT |
		DMA_PACK_ALIGN_LSB |
		DMA_PACK_PATTERN_RGB |
		DMA_OUT_SEL_AHB |
		DMA_IBUF_NONCONTIGUOUS;

	dma2_cfg |= DMA_IBUF_FORMAT_RGB565;

	dma2_cfg |= DMA_OUT_SEL_MDDI;

	dma2_cfg |= DMA_MDDI_DMAOUT_LCD_SEL_PRIMARY;

	dma2_cfg |= DMA_DITHER_EN;

	/* setup size, address, and stride */
	writel((height << 16) | (width), MDP_CMD_DEBUG_ACCESS_BASE + 0x0184);
	writel(addr, MDP_CMD_DEBUG_ACCESS_BASE + 0x0188);
	writel(stride, MDP_CMD_DEBUG_ACCESS_BASE + 0x018C);

	/* 666 18BPP */
	dma2_cfg |= DMA_DSTC0G_6BITS | DMA_DSTC1B_6BITS | DMA_DSTC2R_6BITS;

	/* set y & x offset and MDDI transaction parameters */
	writel((y << 16) | (x), MDP_CMD_DEBUG_ACCESS_BASE + 0x0194);
	writel(ld_param, MDP_CMD_DEBUG_ACCESS_BASE + 0x01a0);
	writel((MDDI_VDO_PACKET_DESC << 16) | MDDI_VDO_PACKET_PRIM,
		   MDP_CMD_DEBUG_ACCESS_BASE + 0x01a4);

	writel(dma2_cfg, MDP_CMD_DEBUG_ACCESS_BASE + 0x0180);

	/* start DMA2 */
	writel(0, MDP_CMD_DEBUG_ACCESS_BASE + 0x0044);
}

void mdp_set_grp_disp(unsigned disp_id)
{
	disp_id &= 0xf;
	writel(disp_id, MDP_FULL_BYPASS_WORD43);
}

#include "mdp_csc_table.h"
#include "mdp_scale_tables.h"

int mdp_init(void)
{
	int ret;
	int n;

	ret = request_irq(INT_MDP, mdp_isr, IRQF_DISABLED, "msm_mdp", 0);

	/* debug interface write access */
	writel(1, MSM_MDP_BASE + 0x60);

	writel(MDP_ANY_INTR_MASK, MDP_INTR_ENABLE);
	writel(1, MDP_EBI2_PORTMAP_MODE);

	writel(0, MDP_CMD_DEBUG_ACCESS_BASE + 0x01f8);
	writel(0, MDP_CMD_DEBUG_ACCESS_BASE + 0x01fc);

	for (n = 0; n < ARRAY_SIZE(csc_table); n++)
		writel(csc_table[n].val, csc_table[n].reg);

	/* clear up unused fg/main registers */
	/* comp.plane 2&3 ystride */
	writel(0, MDP_CMD_DEBUG_ACCESS_BASE + 0x0120);

	/* unpacked pattern */
	writel(0, MDP_CMD_DEBUG_ACCESS_BASE + 0x012c);
	writel(0, MDP_CMD_DEBUG_ACCESS_BASE + 0x0130);
	writel(0, MDP_CMD_DEBUG_ACCESS_BASE + 0x0134);
	writel(0, MDP_CMD_DEBUG_ACCESS_BASE + 0x0158);
	writel(0, MDP_CMD_DEBUG_ACCESS_BASE + 0x015c);
	writel(0, MDP_CMD_DEBUG_ACCESS_BASE + 0x0160);
	writel(0, MDP_CMD_DEBUG_ACCESS_BASE + 0x0170);
	writel(0, MDP_CMD_DEBUG_ACCESS_BASE + 0x0174);
	writel(0, MDP_CMD_DEBUG_ACCESS_BASE + 0x017c);

	/* comp.plane 2 & 3 */
	writel(0, MDP_CMD_DEBUG_ACCESS_BASE + 0x0114);
	writel(0, MDP_CMD_DEBUG_ACCESS_BASE + 0x0118);

	/* clear unused bg registers */
	writel(0, MDP_CMD_DEBUG_ACCESS_BASE + 0x01c8);
	writel(0, MDP_CMD_DEBUG_ACCESS_BASE + 0x01d0);
	writel(0, MDP_CMD_DEBUG_ACCESS_BASE + 0x01dc);
	writel(0, MDP_CMD_DEBUG_ACCESS_BASE + 0x01e0);
	writel(0, MDP_CMD_DEBUG_ACCESS_BASE + 0x01e4);

	for (n = 0; n < ARRAY_SIZE(mdp_upscale_table); n++)
		writel(mdp_upscale_table[n].val, mdp_upscale_table[n].reg);

	mdp_set_ccs(mdp_default_ccs);

	return 0;
}


