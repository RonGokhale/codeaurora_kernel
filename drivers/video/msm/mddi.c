/* drivers/video/msm_fb/mddi.c
 *
 * MSM MDDI Transport
 *
 * Copyright (C) 2007 Google Incorporated
 * Copyright (C) 2007 QUALCOMM Incorporated
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/spinlock.h>

#include <asm/io.h>
#include <asm/arch/msm_iomap.h>
#include <asm/arch/irqs.h>
#include <asm/arch/board.h>
#include <asm/delay.h>

#ifdef CONFIG_ANDROID_POWER
#include <linux/android_power.h>
#endif

#include "msm_fb.h"
#include "mddi_hw.h"

#define FLAG_DISABLE_HIBERNATION 0x0001
#define FLAG_HAVE_CAPS		 0x0002
#define FLAG_HAS_VSYNC_IRQ	 0x0004

#define CMD_GET_CLIENT_CAP     0x0601
#define CMD_GET_CLIENT_STATUS  0x0602

static uint32_t mddi_debug_flags;

union mddi_rev
{
	unsigned char raw[MDDI_REV_BUFFER_SIZE];
	struct mddi_rev_packet hdr;
	struct mddi_client_status status;
	struct mddi_client_caps caps;
	struct mddi_register_access reg;
};

struct reg_read_info
{
	struct completion done;
	uint32_t reg;
	uint32_t status;
	uint32_t result;
};

struct mddi_info
{
	const char *name;
	uint16_t flags;
	uint16_t version;
	uint32_t base;
	int irq;

	/* buffer for rev encap packets */
	void *rev_data;
	dma_addr_t rev_addr;
	struct mddi_llentry *reg_write_data;
	dma_addr_t reg_write_addr;
	struct mddi_llentry *reg_read_data;
	dma_addr_t reg_read_addr;
	size_t rev_data_curr;

	spinlock_t int_lock;
	uint32_t int_enable;
	uint32_t got_int;
	wait_queue_head_t int_wait;

	struct mutex reg_write_lock;
	struct mutex reg_read_lock;
	struct reg_read_info *reg_read;

	struct mddi_client_caps caps;

#ifdef CONFIG_ANDROID_POWER
	android_early_suspend_t early_suspend;
#endif

	void (*panel_power)(int on);

	/* client device published to bind us to the
	** appropriate mddi_client driver
	*/
	char client_name[20];
	struct platform_device client_pdev;

	/* panel device we will publish when a mddi_client
	** driver registers a panel with us
	*/
	struct platform_device panel_pdev;
	struct mddi_panel_info panel_info;
};


static void mddi_init_rev_encap(struct mddi_info *mddi);

#define mddi_readl(r) readl(mddi->base + (MDDI_##r))
#define mddi_writel(v, r) writel((v), mddi->base + (MDDI_##r))

void mddi_activate_link(struct mddi_info *mddi)
{
	mddi_writel(MDDI_CMD_LINK_ACTIVE, CMD);
}

static void mddi_handle_link_list_done(struct mddi_info *mddi)
{
}

static void mddi_handle_rev_data(struct mddi_info *mddi, union mddi_rev *rev)
{
	int i;
	struct reg_read_info *ri;

	if ((rev->hdr.length < MDDI_REV_BUFFER_SIZE) &&
	   (rev->hdr.length >= sizeof(struct mddi_rev_packet))) {
		/* printk(KERN_INFO "rev: len=%04x type=%04x\n", rev->hdr.length, rev->hdr.type); */

		switch (rev->hdr.type) {
		case 66: /* client caps */
			memcpy(&mddi->caps, &rev->caps,
			       sizeof(struct mddi_client_caps));
			mddi->flags |= FLAG_HAVE_CAPS;
			wake_up(&mddi->int_wait);
			break;
		case TYPE_REGISTER_ACCESS:
			/* printk(KERN_INFO "rev: reg %x = %x\n", rev->reg.register_address, rev->reg.register_data_list); */
			ri = mddi->reg_read;
			if (ri == 0) {
				printk(KERN_INFO "rev: got reg %x = %x without pending read\n", rev->reg.register_address, rev->reg.register_data_list);
				break;
			}
			if (ri->reg != rev->reg.register_address) {
				printk(KERN_INFO "rev: got reg %x = %x for wrong register, expected %x\n", rev->reg.register_address, rev->reg.register_data_list, ri->reg);
				break;
			}
			mddi->reg_read = NULL;
			ri->status = 0;
			ri->result = rev->reg.register_data_list;
			complete(&ri->done);
			break;
		default:
			printk(KERN_INFO "rev: unknown reverse packet: len=%04x type=%04x CURR_REV_PTR=%x\n", rev->hdr.length, rev->hdr.type, mddi_readl(CURR_REV_PTR));
			for (i = 0; i < rev->hdr.length + 2; i++) {
				if ((i % 16) == 0)
					printk(KERN_INFO "\n");
				printk(KERN_INFO " %02x", rev->raw[i]);
			}
			printk(KERN_INFO "\n");
		}
	} else {
		printk(KERN_INFO "bad rev length, %d, CURR_REV_PTR %x\n", rev->hdr.length, mddi_readl(CURR_REV_PTR));
	}
}

static void mddi_handle_rev_data_avail(struct mddi_info *mddi)
{
	union mddi_rev *rev = mddi->rev_data;
	uint32_t rev_data_count;
	uint32_t rev_crc_err_count;
	int i;
	struct reg_read_info *ri;
	size_t prev_offset;
	uint16_t length;

	union mddi_rev *crev = mddi->rev_data + mddi->rev_data_curr;

	/* clear the interrupt */
	mddi_writel(MDDI_INT_REV_DATA_AVAIL, INT);
	rev_data_count = mddi_readl(REV_PKT_CNT);
	rev_crc_err_count = mddi_readl(REV_CRC_ERR);
	if (rev_data_count > 1)
		printk(KERN_INFO "rev_data_count %d\n", rev_data_count);
	/* printk(KERN_INFO "rev_data_count %d, INT %x\n", rev_data_count,  mddi_readl(INT)); */

	if (rev_crc_err_count) {
		printk(KERN_INFO "rev_crc_err_count %d, INT %x\n", rev_crc_err_count,  mddi_readl(INT));
		ri = mddi->reg_read;
		if (ri == 0) {
			printk(KERN_INFO "rev: got crc error without pending read\n");
		} else {
			mddi->reg_read = NULL;
			ri->status = -EIO;
			ri->result = -1;
			complete(&ri->done);
		}
	}

	if (rev_data_count == 0)
		return;

	if (mddi_debug_flags & 1) {
		printk(KERN_INFO "INT %x, STAT %x, CURR_REV_PTR %x\n", mddi_readl(INT), mddi_readl(STAT), mddi_readl(CURR_REV_PTR));
		for (i = 0; i < MDDI_REV_BUFFER_SIZE; i++) {
			if ((i % 16) == 0)
				printk(KERN_INFO "\n");
			printk(KERN_INFO " %02x", rev->raw[i]);
		}
		printk(KERN_INFO "\n");
	}

	/* printk(KERN_INFO "rev_data_curr %d + %d\n", mddi->rev_data_curr, crev->hdr.length); */
	prev_offset = mddi->rev_data_curr;

	length = *((uint8_t *)mddi->rev_data + mddi->rev_data_curr);
	mddi->rev_data_curr++;
	if (mddi->rev_data_curr == MDDI_REV_BUFFER_SIZE)
		mddi->rev_data_curr = 0;
	length += *((uint8_t *)mddi->rev_data + mddi->rev_data_curr) << 8;
	mddi->rev_data_curr += 1 + length;
	if (mddi->rev_data_curr >= MDDI_REV_BUFFER_SIZE)
		mddi->rev_data_curr = mddi->rev_data_curr % MDDI_REV_BUFFER_SIZE;

	/* printk(KERN_INFO "rev_data_curr %d + 2 + %d = %d\n", prev_offset, length, mddi->rev_data_curr); */
	if (prev_offset + 2 + length >= MDDI_REV_BUFFER_SIZE) {
		union mddi_rev tmprev;
		size_t rem = MDDI_REV_BUFFER_SIZE - prev_offset;
		memcpy(&tmprev.raw[0], mddi->rev_data + prev_offset, rem);
		memcpy(&tmprev.raw[rem], mddi->rev_data, 2 + length - rem);
		mddi_handle_rev_data(mddi, &tmprev);
		if (mddi_debug_flags & 2) {
			memset(mddi->rev_data + prev_offset, 0xee, rem);
			memset(mddi->rev_data, 0xee, mddi->rev_data_curr);
		}
	} else {
		mddi_handle_rev_data(mddi, crev);
		if (mddi_debug_flags & 2)
			memset(mddi->rev_data + prev_offset, 0xee, mddi->rev_data_curr - prev_offset);
	}

	/* if(mddi->rev_data_curr + MDDI_MAX_REV_PKT_SIZE >= MDDI_REV_BUFFER_SIZE) { */
	if (prev_offset < MDDI_REV_BUFFER_SIZE / 2 && mddi->rev_data_curr >= MDDI_REV_BUFFER_SIZE / 2) {
		/* printk(KERN_INFO "passed buffer half full: rev_data_curr %d\n", mddi->rev_data_curr); */
		mddi_writel(mddi->rev_addr, REV_PTR);
	}
}

static irqreturn_t mddi_isr(int irq, void *data)
{
	struct mddi_info *mddi = data;
	uint32_t active, status;

	spin_lock(&mddi->int_lock);

	active = mddi_readl(INT);
	status = mddi_readl(STAT);

	mddi_writel(active, INT);

/*	printk(KERN_INFO "%s: isr a=%08x e=%08x s=%08x\n", */
/*	       mddi->name, active, mddi->int_enable, status); */

	/* ignore any interrupts we have disabled */
	active &= mddi->int_enable;

	mddi->got_int |= active;
	wake_up(&mddi->int_wait);

	if (active & MDDI_INT_LINK_ACTIVE) {
		mddi->int_enable &= (~MDDI_INT_LINK_ACTIVE);
		mddi->int_enable |= MDDI_INT_IN_HIBERNATION;
/*		printk(KERN_INFO "%s: active\n", mddi->name); */
	}
	if (active & MDDI_INT_IN_HIBERNATION) {
		mddi->int_enable &= (~MDDI_INT_IN_HIBERNATION);
		mddi->int_enable |= MDDI_INT_LINK_ACTIVE;
/*		printk(KERN_INFO "%s: hibernate\n", mddi->name); */
	}
	if (active & MDDI_INT_PRI_LINK_LIST_DONE) {
		mddi->int_enable &= (~MDDI_INT_PRI_LINK_LIST_DONE);
		mddi_handle_link_list_done(mddi);
	}
	if (active & MDDI_INT_REV_DATA_AVAIL)
		mddi_handle_rev_data_avail(mddi);
	if (active & ~MDDI_INT_NEED_CLEAR)
		mddi->int_enable &= ~(active & ~MDDI_INT_NEED_CLEAR);

	mddi_writel(mddi->int_enable, INTEN);
	spin_unlock(&mddi->int_lock);

	return IRQ_HANDLED;
}

static long mddi_wait_interrupt_timeout(struct mddi_info *mddi, uint32_t intmask, int timeout)
{
	unsigned long irq_flags;

	spin_lock_irqsave(&mddi->int_lock, irq_flags);
	mddi->got_int &= ~intmask;
	mddi->int_enable |= intmask;
	mddi_writel(mddi->int_enable, INTEN);
	spin_unlock_irqrestore(&mddi->int_lock, irq_flags);
	return wait_event_timeout(mddi->int_wait, mddi->got_int & intmask, timeout);
}

static void mddi_wait_interrupt(struct mddi_info *mddi, uint32_t intmask)
{
	if (mddi_wait_interrupt_timeout(mddi, intmask, HZ/10) == 0)
		printk(KERN_INFO KERN_ERR "mddi_wait_interrupt, timeout waiting for %x, "
		       "INT = %x, STAT = %x\n",
		       intmask, mddi_readl(INT), mddi_readl(STAT));
}

static void mddi_init_rev_encap(struct mddi_info *mddi)
{
	memset(mddi->rev_data, 0xee, MDDI_REV_BUFFER_SIZE);
	mddi_writel(mddi->rev_addr, REV_PTR);
	mddi_writel(MDDI_CMD_FORCE_NEW_REV_PTR, CMD);
	mddi_wait_interrupt(mddi, MDDI_INT_NO_CMD_PKTS_PEND);
}

static void mddi_init_registers(struct mddi_info *mddi)
{
	/* XXX enable clock */

	mddi_writel(MDDI_CMD_RESET, CMD);
	mddi_wait_interrupt(mddi, MDDI_INT_NO_CMD_PKTS_PEND);

	mddi_writel(0x0001, VERSION);
	mddi_writel(MDDI_HOST_BYTES_PER_SUBFRAME, BPS);
	mddi_writel(0x0003, SPM); /* subframes per media */
	mddi_writel(0x0005, TA1_LEN);
	mddi_writel(MDDI_HOST_TA2_LEN, TA2_LEN);
	mddi_writel(0x0096, DRIVE_HI);
	/* 0x32 normal, 0x50 for Toshiba display */
	mddi_writel(0x0050, DRIVE_LO);
	mddi_writel(0x003C, DISP_WAKE); /* wakeup counter */
	mddi_writel(MDDI_HOST_REV_RATE_DIV, REV_RATE_DIV);

	mddi_writel(MDDI_REV_BUFFER_SIZE, REV_SIZE);
	mddi_writel(MDDI_MAX_REV_PKT_SIZE, REV_ENCAP_SZ);

	/* disable periodic rev encap */
	mddi_writel(MDDI_CMD_PERIODIC_REV_ENCAP, CMD);
	mddi_wait_interrupt(mddi, MDDI_INT_NO_CMD_PKTS_PEND);

	if (mddi_readl(PAD_CTL) == 0) {
		/* If we are turning on band gap, need to wait 5us before turning
		 * on the rest of the PAD */
		mddi_writel(0x08000, PAD_CTL);
		udelay(5);
	}

	/* Recommendation from PAD hw team */
	mddi_writel(0xa850f, PAD_CTL);

	mddi->version = mddi_readl(CORE_VER) & 0xffff;

	/* Need an even number for counts */
	mddi_writel(0x60006, DRIVER_START_CNT);

	if (mddi->flags & FLAG_DISABLE_HIBERNATION) {
		mddi_writel(MDDI_CMD_HIBERNATE | 0, CMD);
	} else {
		/* hibernate after 1 empty subframe */
		mddi_writel(MDDI_CMD_HIBERNATE | 1, CMD);
	}
	mddi_wait_interrupt(mddi, MDDI_INT_NO_CMD_PKTS_PEND);

#if 1 /* ignore listen */
	mddi_writel(MDDI_CMD_DISP_IGNORE, CMD);
#else
	mddi_writel(MDDI_CMD_DISP_LISTEN, CMD);
#endif
	mddi_wait_interrupt(mddi, MDDI_INT_NO_CMD_PKTS_PEND);

	mddi_init_rev_encap(mddi);
}

#ifdef CONFIG_ANDROID_POWER
static void mddi_early_suspend(android_early_suspend_t *h)
{
	struct mddi_info *mddi = container_of(h, struct mddi_info, early_suspend);
	if(mddi->panel_power)
		mddi->panel_power(0);
}

static void mddi_early_resume(android_early_suspend_t *h)
{
	struct mddi_info *mddi = container_of(h, struct mddi_info, early_suspend);
	if(mddi->panel_power)
		mddi->panel_power(1);
}
#endif

static int __init mddi_init(struct mddi_info *mddi, const char *name,
			    struct msm_mddi_platform_data *pd,
			    uint32_t base, int irq)
{
	int ret;
	int i, j;
	void *dma;
	dma_addr_t dma_addr;

	printk(KERN_INFO "%s: init() base=0x%08x irq=%d\n", name, base, irq);

	mddi->base = base;
	mddi->name = name;
	mddi->irq = irq;

	mutex_init(&mddi->reg_write_lock);
	mutex_init(&mddi->reg_read_lock);
	spin_lock_init(&mddi->int_lock);
	init_waitqueue_head(&mddi->int_wait);

	mddi->flags = FLAG_DISABLE_HIBERNATION;

	if (pd) {
		mddi->panel_power = pd->panel_power;
		if (pd->has_vsync_irq)
			mddi->flags |= FLAG_HAS_VSYNC_IRQ;
	}

	dma = dma_alloc_coherent(NULL, 0x1000, &dma_addr, GFP_KERNEL);
	if (dma == 0) {
		ret = -ENOMEM;
		goto fail0;
	}

	mddi->int_enable = 0;
	mddi_writel(mddi->int_enable, INTEN);

	ret = request_irq(irq, mddi_isr, IRQF_DISABLED, name, mddi);
	if (ret)
		goto fail1;

#ifdef CONFIG_ANDROID_POWER
	mddi->early_suspend.suspend = mddi_early_suspend;
	mddi->early_suspend.resume = mddi_early_resume;
	android_register_early_suspend(&mddi->early_suspend);
#endif

	if (mddi->panel_power)
		mddi->panel_power(1);

	mddi->rev_data = dma;
	mddi->rev_data_curr = 0;
	mddi->rev_addr = dma_addr;
	mddi->reg_write_data = dma + MDDI_REV_BUFFER_SIZE;
	mddi->reg_write_addr = dma_addr + MDDI_REV_BUFFER_SIZE;
	mddi->reg_read_data = mddi->reg_write_data + 1;
	mddi->reg_read_addr = mddi->reg_write_addr + sizeof(*mddi->reg_write_data);

	mddi_init_registers(mddi);

	if (mddi->version < 0x20) {
		printk(KERN_INFO "%s: unsupported version 0x%x\n",
		       mddi->name, mddi->version);
		ret = -ENODEV;
		goto fail2;
	}

	/* clear any stale interrupts */
	mddi_writel(0xffffffff, INT);

	mddi->int_enable = MDDI_INT_LINK_ACTIVE |
		       MDDI_INT_IN_HIBERNATION |
		       MDDI_INT_PRI_LINK_LIST_DONE |
		       MDDI_INT_REV_DATA_AVAIL |
		       MDDI_INT_REV_OVERFLOW |
		       MDDI_INT_REV_OVERWRITE |
		       MDDI_INT_RTD_FAILURE;
	mddi_writel(mddi->int_enable, INTEN);

	mddi_writel(MDDI_CMD_LINK_ACTIVE, CMD);
	mddi_wait_interrupt(mddi, MDDI_INT_NO_CMD_PKTS_PEND);

	for (j = 0; j < 3; j++) {
		/* the toshiba vga panel does not respond to get
		 * caps unless you SEND_RTD, but the first SEND_RTD
		 * will fail...
		 */
		for (i = 0; i < 4; i++) {
			uint32_t stat;

			/* printk(KERN_INFO "MDDI_CMD_SEND_RTD: int %x, stat %x, rtd val %x\n", mddi_readl(INT), stat, mddi_readl(RTD_VAL)); */
			mddi_writel(MDDI_CMD_SEND_RTD, CMD);
			/* printk(KERN_INFO "MDDI_CMD_SEND_RTD: int %x, stat %x, rtd val %x\n", mddi_readl(INT), stat, mddi_readl(RTD_VAL)); */
			mddi_wait_interrupt(mddi, MDDI_INT_NO_CMD_PKTS_PEND);
			/* mddi_writel(mddi_readl(INTEN) | MDDI_INT_NO_CMD_PKTS_PEND, INTEN); */
			/* printk(KERN_INFO "MDDI_CMD_SEND_RTD: stat %x, rtd val %x\n", mddi_readl(STAT), mddi_readl(RTD_VAL)); */
			stat = mddi_readl(STAT);
			printk(KERN_INFO "MDDI_CMD_SEND_RTD: int %x, stat %x, rtd val %x\n", mddi_readl(INT), stat, mddi_readl(RTD_VAL));
			/* msleep(10); */
			/* printk(KERN_INFO "MDDI_CMD_SEND_RTD: stat %x, rtd val %x\n", mddi_readl(STAT), mddi_readl(RTD_VAL)); */
			/* if((stat & MDDI_STAT_RTD_MEAS_FAIL) == 0) */
			/*	break; */
			if ((stat & MDDI_STAT_RTD_MEAS_FAIL) == 0)
				break;
			msleep(1);
		}

		mddi_writel(CMD_GET_CLIENT_CAP, CMD);
		mddi_wait_interrupt(mddi, MDDI_INT_NO_CMD_PKTS_PEND);
		wait_event_timeout(mddi->int_wait, mddi->flags & FLAG_HAVE_CAPS, HZ / 100);
		if (mddi->flags & FLAG_HAVE_CAPS)
			break;
		printk(KERN_INFO KERN_ERR "mddi_init, timeout waiting for caps\n");
	}

	if (mddi->flags & FLAG_HAVE_CAPS) {
		mddi_writel(MDDI_CMD_HIBERNATE | 1, CMD); /* hibernate after 1 empty subframe */
		mddi_wait_interrupt(mddi, MDDI_INT_NO_CMD_PKTS_PEND);

		/* setup panel_info which will be used by the fb core */
		mddi->panel_info.mddi = mddi;
		mddi->panel_info.width = mddi->caps.Bitmap_Width;
		mddi->panel_info.height = mddi->caps.Bitmap_Height;

		/* setup panel_pdev which will be used by the fb core */
		mddi->panel_pdev.id = 0;
		mddi->panel_pdev.name = "mddi_panel";
		mddi->panel_pdev.dev.platform_data = &mddi->panel_info;

		/* setup a client device for publishing */
		sprintf(mddi->client_name, "mddi_c_%04x_%04x",
			mddi->caps.Mfr_Name, mddi->caps.Product_Code);
		mddi->client_pdev.id = 0;
		mddi->client_pdev.name = mddi->client_name;
		mddi->client_pdev.dev.platform_data = mddi;

		printk(KERN_INFO "%s: publish: %s\n", mddi->name, mddi->client_name);
		platform_device_register(&mddi->client_pdev);
	} else {
		printk(KERN_INFO "%s: no client found\n", mddi->name);
		/* XXX power down */
		mddi_writel(MDDI_CMD_POWERDOWN, CMD);
		printk(KERN_INFO "mddi powerdown: stat %x\n", mddi_readl(STAT));
		msleep(100);
		printk(KERN_INFO "mddi powerdown: stat %x\n", mddi_readl(STAT));
	}
	return 0;

fail2:
	free_irq(irq, 0);
fail1:
	dma_free_coherent(NULL, 0x1000, dma, dma_addr);
fail0:
	printk(KERN_INFO "%s: mddi_init() failed (%d)\n", name, ret);
	return ret;
}

void mddi_remote_write(struct mddi_info *mddi, unsigned val, unsigned reg)
{
	struct mddi_llentry *ll;
	struct mddi_register_access *ra;
	/* unsigned s; */

	mutex_lock(&mddi->reg_write_lock);

	ll = mddi->reg_write_data;

	ra = &(ll->u.r);
	ra->length = 14 + 4;
	ra->type = TYPE_REGISTER_ACCESS;
	ra->client_id = 0;
	ra->read_write_info = MDDI_WRITE | 1;
	ra->crc16 = 0;

	ra->register_address = reg;
	ra->register_data_list = val;

	ll->flags = 1;
	ll->header_count = 14;
	ll->data_count = 4;
	ll->data = mddi->reg_write_addr + offsetof(struct mddi_llentry, u.r.register_data_list);
	ll->next = 0;
	ll->reserved = 0;

	/* s = mddi_readl(STAT); */
	/* printk(KERN_INFO "mddi_remote_write(%x, %x), stat = %x\n", val, reg, s); */

	mddi_writel(mddi->reg_write_addr, PRI_PTR);

	/* s = mddi_readl(STAT); */
	/* printk(KERN_INFO "mddi_remote_write(%x, %x) sent, stat = %x\n", val, reg, s); */

	mddi_wait_interrupt(mddi, MDDI_INT_PRI_LINK_LIST_DONE);
	/* printk(KERN_INFO "mddi_remote_write(%x, %x) done, stat = %x\n", val, reg, s); */
	mutex_unlock(&mddi->reg_write_lock);
}

unsigned mddi_remote_read(struct mddi_info *mddi, unsigned reg)
{
	struct mddi_llentry *ll;
	struct mddi_register_access *ra;
	struct reg_read_info ri;
	unsigned s;
	int retry_count = 2;
	unsigned long irq_flags;

	mutex_lock(&mddi->reg_read_lock);

	ll = mddi->reg_read_data;

	ra = &(ll->u.r);
	ra->length = 14;
	ra->type = TYPE_REGISTER_ACCESS;
	ra->client_id = 0;
	ra->read_write_info = MDDI_READ | 1;
	ra->crc16 = 0;

	ra->register_address = reg;

	ll->flags = 0x11;
	ll->header_count = 14;
	ll->data_count = 0;
	ll->data = 0;
	ll->next = 0;
	ll->reserved = 0;

	s = mddi_readl(STAT);
	/* printk(KERN_INFO "mddi_remote_read(%x), stat = %x\n", reg, s); */

	ri.reg = reg;
	ri.status = -1;

	do {
		init_completion(&ri.done);
		mddi->reg_read = &ri;
		mddi_writel(mddi->reg_read_addr, PRI_PTR);

		mddi_wait_interrupt(mddi, MDDI_INT_PRI_LINK_LIST_DONE);
		/* s = mddi_readl(STAT); */
		/* printk(KERN_INFO "mddi_remote_read(%x) sent, stat = %x\n", reg, s); */

		/* s = mddi_readl(STAT); */
		/* while((s & MDDI_STAT_PRI_LINK_LIST_DONE) == 0){ */
		/*	s = mddi_readl(STAT); */
		/* } */

		/* Enable Periodic Reverse Encapsulation. */
		mddi_writel(MDDI_CMD_PERIODIC_REV_ENCAP | 1, CMD);
		mddi_wait_interrupt(mddi, MDDI_INT_NO_CMD_PKTS_PEND);
		if (wait_for_completion_timeout(&ri.done, HZ/60) == 0 && !ri.done.done) {
			printk(KERN_INFO "mddi_remote_read(%x) timeout (%d %d %d)\n", reg, ri.status, ri.result, ri.done.done);
			spin_lock_irqsave(&mddi->int_lock, irq_flags);
			mddi->reg_read = NULL;
			spin_unlock_irqrestore(&mddi->int_lock, irq_flags);
			ri.status = -1;
			ri.result = -1;
		}
		if (ri.status == 0)
			break;

		/* printk(KERN_INFO "mddi_remote_read: failed, sent MDDI_CMD_SEND_RTD: int %x, stat %x, rtd val %x\n", mddi_readl(INT), mddi_readl(STAT), mddi_readl(RTD_VAL)); */
		mddi_writel(MDDI_CMD_SEND_RTD, CMD);
		mddi_writel(MDDI_CMD_LINK_ACTIVE, CMD);
		/* printk(KERN_INFO "mddi_remote_read: failed, sent MDDI_CMD_SEND_RTD: int %x, stat %x, rtd val %x\n", mddi_readl(INT), mddi_readl(STAT), mddi_readl(RTD_VAL)); */
		mddi_wait_interrupt(mddi, MDDI_INT_NO_CMD_PKTS_PEND);
		printk(KERN_INFO "mddi_remote_read: failed, sent MDDI_CMD_SEND_RTD: int %x, stat %x, rtd val %x\n", mddi_readl(INT), mddi_readl(STAT), mddi_readl(RTD_VAL));
	} while (retry_count-- > 0);
	/* Disable Periodic Reverse Encapsulation. */
	mddi_writel(MDDI_CMD_PERIODIC_REV_ENCAP | 0, CMD);
	mddi_wait_interrupt(mddi, MDDI_INT_NO_CMD_PKTS_PEND);
	/* printk(KERN_INFO "mddi_remote_read(%x) done, stat = %x, return %x\n", reg, s, ri.result); */
	mddi->reg_read = NULL;
	mutex_unlock(&mddi->reg_read_lock);
	return ri.result;
}

int mddi_add_panel(struct mddi_info *mddi, struct mddi_panel_ops *ops)
{
	printk(KERN_INFO "%s: mddi_add_panel(%p, %p)\n", mddi->name, mddi, ops);

	if (mddi->panel_info.panel_ops == 0) {
		mddi->panel_info.panel_ops = ops;

		/* TODO: this should actually be decided by the panel
		** driver, as it may have a non-irq fallback handler
		*/
		if (!(mddi->flags & FLAG_HAS_VSYNC_IRQ))
			ops->wait_vsync = 0;

		printk(KERN_INFO "%s: publish: %s\n", mddi->name, mddi->panel_pdev.name);
		platform_device_register(&mddi->panel_pdev);
		return 0;
	}
	return -1;
}

static struct mddi_info mddi_pmdh;
static struct mddi_info mddi_emdh;

static int __init mddi_probe(struct platform_device *pdev)
{
	struct msm_mddi_platform_data *pd = pdev->dev.platform_data;

	switch (pdev->id) {
	case 0:
		return mddi_init(&mddi_pmdh, "mddi_pmdh", pd,
				 MSM_PMDH_BASE, INT_MDDI_PRI);
	case 1:
		return mddi_init(&mddi_emdh, "mddi_emdh", pd,
				 MSM_EMDH_BASE, INT_MDDI_EXT);
	default:
		return -ENODEV;
	}
}

#if 1 /* read/write mddi registers from userspace */
module_param_named(debug, mddi_debug_flags, uint, 0644);

static uint32_t selected_register;
module_param_named(reg, selected_register, uint, 0644);

static int set_reg(const char *val, struct kernel_param *kp)
{
	char *endp;
	uint32_t l;

	if (!val) return -EINVAL;
	l = simple_strtoul(val, &endp, 0);
	if (endp == val || ((uint32_t)l != l))
		return -EINVAL;
	mddi_remote_write(kp->arg, l, selected_register);
	return 0;
}

static int get_reg(char *buffer, struct kernel_param *kp)
{
	int val;
	val = mddi_remote_read(kp->arg, selected_register);
	return sprintf(buffer, "%x", val);
}

module_param_call(pmdh_val, set_reg, get_reg, &mddi_pmdh, 0644);
module_param_call(emdh_val, set_reg, get_reg, &mddi_emdh, 0644);
#endif

static struct platform_driver mddi_driver = {
	.probe = mddi_probe,
	.driver = { .name = "msm_mddi" },
};

static int __init _mddi_init(void)
{
	return platform_driver_register(&mddi_driver);
}

module_init(_mddi_init);
