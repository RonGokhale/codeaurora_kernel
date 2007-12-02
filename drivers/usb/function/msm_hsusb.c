/* drivers/usb/function/msm_hsusb.c
 *
 * Driver for HighSpeed USB Client Controller in MSM7K
 *
 * Copyright (C) 2007 Google, Inc.
 * Author: Brian Swetland <swetland@google.com>
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
#include <linux/kernel.h>
#include <linux/list.h>

#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>

#include <linux/usb/ch9.h>

#include <asm/io.h>

#define MSM_USB_BASE ((unsigned) ui->addr)

#include "msm_hsusb_hw.h"

#include "usb_function.h"

#define EPT_FLAG_IN        0x0001

struct usb_fi_ept
{
	struct usb_endpoint *ept;
	struct usb_endpoint_descriptor desc;
};

struct usb_function_info
{
	struct list_head list;
	unsigned endpoints; /* nonzero if bound */

	struct usb_function *func;
	struct usb_interface_descriptor ifc;
	struct usb_fi_ept ept[0];
};

struct msm_request
{
	struct usb_request req;

	struct usb_info *ui;
	struct msm_request *next;

	unsigned busy:1;

	dma_addr_t dma;
	unsigned dma_size;

	struct ept_queue_item *item;
	dma_addr_t item_dma;
};

#define to_msm_request(r) container_of(r, struct msm_request, req)

struct usb_endpoint
{
	struct usb_info *ui;
	struct msm_request *req; /* head of pending requests */
	struct msm_request *last;
	unsigned flags;

	/* bit number (0-31) in various status registers
	** as well as the index into the usb_info's array
	** of all endpoints
	*/
	unsigned char bit;
	unsigned char num;

	unsigned short max_pkt;

	/* pointers to DMA transfer list area */
	/* these are allocated from the usb_info dma space */
	struct ept_queue_head *head;

	struct usb_function_info *owner;
};

struct usb_info
{
	spinlock_t lock;

	/* single request used for handling setup transactions */
	struct usb_request *setup_req;

	struct platform_device *pdev;
	int irq;
	void *addr;
	unsigned online;

	/* dma page to back the queue heads and items */
	unsigned char *buf;
	dma_addr_t dma;

	struct ept_queue_head *head;
	struct ept_queue_item *item;

	struct list_head *flist;


	/* used for allocation */
	unsigned next_item;
	unsigned next_ifc_num;

	/* endpoints are ordered based on their status bits,
	** so they are OUT0, OUT1, ... OUT15, IN0, IN1, ... IN15
	*/
	struct usb_endpoint ept[32];

#define ep0out ept[0]
#define ep0in  ept[16]
};

static void init_endpoints(struct usb_info *ui)
{
	unsigned n;

	for (n = 0; n < 31; n++) {
		struct usb_endpoint *ept = ui->ept + n;

		ept->ui = ui;
		ept->bit = n;
		ept->num = n & 15;

		if (ept->bit > 15) {
			/* IN endpoint */
			ept->head = ui->head + (ept->num << 1) + 1;
			ept->flags = EPT_FLAG_IN;
		} else {
			/* OUT endpoint */
			ept->head = ui->head + (ept->num << 1);
			ept->flags = 0;
		}

	}
}

static void configure_endpoints(struct usb_info *ui)
{
	unsigned n;
	unsigned cfg;

	for (n = 0; n < 31; n++) {
		struct usb_endpoint *ept = ui->ept + n;

		cfg = CONFIG_MAX_PKT(ept->max_pkt) | CONFIG_ZLT;

		if (ept->bit == 0)
			/* ep0 out needs interrupt-on-setup */
			cfg |= CONFIG_IOS;

		ept->head->config = cfg;
		ept->head->next = TERMINATE;

		if (ept->max_pkt)
			printk(KERN_INFO "ept #%d %s max:%d head:%p bit:%d\n",
			       ept->num,
			       (ept->flags & EPT_FLAG_IN) ? "in" : "out",
			       ept->max_pkt, ept->head, ept->bit);
	}
}

static struct usb_endpoint *alloc_endpoint(struct usb_info *ui,
					   struct usb_function_info *owner,
					   unsigned kind)
{
	unsigned n;
	for (n = 0; n < 31; n++) {
		struct usb_endpoint *ept = ui->ept + n;
		if (ept->num == 0)
			continue;
		if (ept->owner)
			continue;

		if (ept->flags & EPT_FLAG_IN) {
			if (kind == EPT_BULK_IN) {
				ept->max_pkt = 512;
				ept->owner = owner;
				return ept;
			}
		} else {
			if (kind == EPT_BULK_OUT) {
				ept->max_pkt = 512;
				ept->owner = owner;
				return ept;
			}
		}
	}

	return 0;
}

static void free_endpoints(struct usb_info *ui, struct usb_function_info *owner)
{
	unsigned n;
	for (n = 0; n < 31; n++) {
		struct usb_endpoint *ept = ui->ept + n;
		if (ept->owner == owner) {
			ept->owner = 0;
			ept->max_pkt = 0;
		}
	}
}

static struct ept_queue_item *alloc_queue_item(struct usb_info *ui, dma_addr_t *dma)
{
	unsigned long flags;
	struct ept_queue_item *item;

	/* XXX implement better */

	spin_lock_irqsave(&ui->lock, flags);

	if (ui->next_item < 64) {
		*dma = ui->dma + 2048 + (ui->next_item * 32);
		item = ui->item + ui->next_item;
		ui->next_item++;
	} else {
		item = 0;
	}

	spin_unlock_irqrestore(&ui->lock, flags);

	return item;
}

static void free_queue_item(struct usb_info *ui, struct ept_queue_item *item)
{
	/* XXX implement */
}

struct usb_request *usb_ept_alloc_req(struct usb_endpoint *ept, unsigned bufsize)
{
	struct usb_info *ui = ept->ui;
	struct msm_request *req;

	req = kzalloc(sizeof(*req), GFP_KERNEL);
	if (!req)
		goto fail1;

	req->item = alloc_queue_item(ui, &req->item_dma);
	if (!req->item)
		goto fail2;

	req->req.buf = dma_alloc_coherent(&ui->pdev->dev, bufsize, &req->dma, GFP_KERNEL);
	if (!req->req.buf)
		goto fail3;

	return &req->req;

fail3:
	free_queue_item(ui, req->item);
fail2:
	kfree(req);
fail1:
	return 0;
}

void usb_ept_free_req(struct usb_endpoint *ept, struct usb_request *_req)
{
	struct msm_request *req = to_msm_request(_req);
	struct usb_info *ui = ept->ui;

	if (req->busy) {
		/* XXX must dequeue */
	}

	free_queue_item(ui, req->item);
	dma_free_coherent(&ui->pdev->dev, req->dma_size, req->req.buf, req->dma);
	kfree(req);
}

static void usb_ept_enable(struct usb_endpoint *ept, int yes)
{
	struct usb_info *ui = ept->ui;
	int in = ept->flags & EPT_FLAG_IN;
	unsigned n;

	n = readl(USB_ENDPTCTRL(ept->num));

	if (in) {
		n = (n & (~CTRL_TXT_MASK)) | CTRL_TXT_BULK;
		if (yes) {
			n |= CTRL_TXE | CTRL_TXR;
		} else {
			n &= (~CTRL_TXE);
		}
	} else {
		n = (n & (~CTRL_RXT_MASK)) | CTRL_RXT_BULK;
		if (yes) {
			n |= CTRL_RXE | CTRL_RXR;
		} else {
			n &= ~(CTRL_RXE);
		}
	}
	writel(n, USB_ENDPTCTRL(ept->num));

#if 0
	printk(KERN_INFO "ept %d %s %s\n",
	       ept->num, in ? "in" : "out", yes ? "enabled" : "disabled");
#endif
}

int usb_ept_queue_xfer(struct usb_endpoint *ept, struct usb_request *_req)
{
	unsigned long flags;
	struct msm_request *req = to_msm_request(_req);
	struct msm_request *last;
	struct usb_info *ui = ept->ui;
	struct ept_queue_item *item = req->item;
	unsigned bit = 1 << ept->bit;
	unsigned tmp;

	spin_lock_irqsave(&ui->lock, flags);

	if (req->busy) {
		req->req.status = -EBUSY;
		spin_unlock_irqrestore(&ui->lock, flags);
		printk(KERN_INFO
		       "usb_ept_queue_xfer() tried to queue busy request\n");
		return -EBUSY;
	}

	if (!ui->online && (ept->num != 0)) {
		req->req.status = -ENODEV;
		spin_unlock_irqrestore(&ui->lock, flags);
		printk(KERN_INFO "usb_ept_queue_xfer() tried to queue request while offline\n");
		return -ENODEV;
	}

	req->busy = 1;
	req->next = 0;
	req->req.status = -EBUSY;

	/* add us to the end of the list and make note of the 'old'
	** last item on the list if there was one
	*/
	last = ept->last;
	if (last) {
		last->next = req;
	} else {
		ept->req = req;
	}
	ept->last = req;

	item->next = TERMINATE;
	item->info = INFO_BYTES(req->req.length) | INFO_IOC | INFO_ACTIVE;
	item->page0 = req->dma;

	if (last) {
		/* we are appending to the end -- link us in */
		last->item->next = req->item_dma;

		/* if we're still primed at this point, we're good to go */
		if (readl(USB_ENDPTPRIME) & bit) goto done;

		/* Otherwise we need to see if a transaction is in-flight.
		** In that case we'll reprime at the end and everything
		** is still fine.
		**
		** The status bit can momentarily clear.  The tripwire bit
		** in the command register is used to detect this condition
		** and work around it by re-sampling the status bit:
		*/
		do {
			writel(readl(USB_USBCMD) | USBCMD_ATDTW, USB_USBCMD);
			tmp = readl(USB_ENDPTSTAT);
		} while ((readl(USB_USBCMD) & USBCMD_ATDTW) == 0);
		writel(readl(USB_USBCMD) & (~USBCMD_ATDTW), USB_USBCMD);

		/* if the status bit is set (transfer was active) we're good */
		if (tmp & bit) goto done;

		/* otherwise we missed the window and now queue as if we were
		** a new request... fall through
		*/
	}

	/* queue was empty -- start us up */
	ept->head->next = req->item_dma;
	ept->head->info = 0;
	writel(bit, USB_ENDPTPRIME);

done:
	spin_unlock_irqrestore(&ui->lock, flags);
	return 0;
}


/* --- endpoint 0 handling --- */

static void set_configuration(struct usb_info *ui, int yes)
{
	struct list_head *entry;
	unsigned n;

	ui->online = !!yes;

	list_for_each(entry, ui->flist) {
		struct usb_function_info *fi =
			list_entry(entry, struct usb_function_info, list);

		if (fi->endpoints == 0)
			continue;

		for (n = 0; n < fi->endpoints; n++)
			usb_ept_enable(fi->ept[n].ept, yes);

		fi->func->configure(yes, fi->func->context);
	}
}

static void ep0out_complete(struct usb_endpoint *ept, struct usb_request *req)
{
	req->complete = 0;
}

static void ep0in_complete(struct usb_endpoint *ept, struct usb_request *req)
{
	struct usb_info *ui = ept->ui;

	/* recv ack */
	req->length = 0;
	req->complete = ep0out_complete;
	usb_ept_queue_xfer(&ui->ep0out, req);
}

static void ep0_setup_ack(struct usb_info *ui)
{
	struct usb_request *req = ui->setup_req;
	req->length = 0;
	req->complete = 0;
	usb_ept_queue_xfer(&ui->ep0in, req);
}

static void ep0_setup_stall(struct usb_info *ui)
{
	writel((1<<16) | (1<<0), USB_ENDPTCTRL(0));
}

static void ep0_setup_send(struct usb_info *ui)
{
	struct usb_request *req = ui->setup_req;
	struct usb_endpoint *ept = &ui->ep0in;
	req->complete = ep0in_complete;
	usb_ept_queue_xfer(ept, req);
}


int usb_find_descriptor(struct usb_info *ui, unsigned id, struct usb_request *req);

static void handle_setup(struct usb_info *ui)
{
	struct usb_ctrlrequest ctl;

	memcpy(&ctl, ui->ep0out.head->setup_data, sizeof(ctl));
	writel(EPT_RX(0), USB_ENDPTSETUPSTAT);

	/* TODO? cancel any pending setup in/out txns */

#if 0
	printk(KERN_INFO
	       "setup: type=%02x req=%02x val=%04x idx=%04x len=%04x\n",
	       ctl.bRequestType, ctl.bRequest, ctl.wValue,
	       ctl.wIndex, ctl.wLength);
#endif

	if (ctl.bRequestType == (USB_DIR_IN | USB_TYPE_STANDARD)) {
		if (ctl.bRequest == USB_REQ_GET_STATUS) {
			if (ctl.wLength == 2) {
				struct usb_request *req = ui->setup_req;
				req->length = 2;
				memset(req->buf, 0, 2);
				ep0_setup_send(ui);
				return;
			}
		}
		if (ctl.bRequest == USB_REQ_GET_DESCRIPTOR) {
			struct usb_request *req = ui->setup_req;

			if (!usb_find_descriptor(ui, ctl.wValue, req)) {
				if (req->length > ctl.wLength)
					req->length = ctl.wLength;
				ep0_setup_send(ui);
				return;
			}
		}
	}
	if (ctl.bRequestType == (USB_DIR_OUT | USB_TYPE_STANDARD | USB_RECIP_ENDPOINT)) {
		if (ctl.bRequest == USB_REQ_CLEAR_FEATURE) {
			if ((ctl.wValue == 0) && (ctl.wLength == 0)) {
				unsigned num = ctl.wIndex & 0x0f;

				if (num != 0) {
					printk(KERN_INFO
					       "clear endpoint %d %s\n", num,
					       ctl.wIndex & 0x80 ? "in" : "out");

					if (ctl.wIndex & 0x80)
						num += 16;

					usb_ept_enable(ui->ept + num, 1);
					ep0_setup_ack(ui);
					return;
				}
			}
		}
	}
	if (ctl.bRequestType == (USB_DIR_OUT | USB_TYPE_STANDARD | USB_RECIP_INTERFACE)) {
		if (ctl.bRequest == USB_REQ_SET_INTERFACE) {
			if ((ctl.wValue == 0) && (ctl.wIndex == 0) && (ctl.wLength == 0)) {
				/* XXX accept for non-0 interfaces */
				ep0_setup_ack(ui);
				return;
			}
		}
	}
	if (ctl.bRequestType == (USB_DIR_OUT | USB_TYPE_STANDARD)) {
		if (ctl.bRequest == USB_REQ_SET_CONFIGURATION) {
			set_configuration(ui, ctl.wValue);
			goto ack;
		}
		if (ctl.bRequest == USB_REQ_SET_ADDRESS) {
			/* write address delayed (will take effect
			** after the next IN txn)
			*/
			writel((ctl.wValue << 25) | (1 << 24), USB_DEVICEADDR);
			goto ack;
		}
	}

	ep0_setup_stall(ui);
	return;

ack:
	ep0_setup_ack(ui);
}

static void handle_endpoint(struct usb_info *ui, unsigned bit)
{
	struct usb_endpoint *ept = ui->ept + bit;
	struct msm_request *req;
	unsigned info;

#if 0
	printk(KERN_INFO "handle_endpoint() %d %s req=%p(%08x)\n",
	       ept->num, (ept->flags & EPT_FLAG_IN) ? "in" : "out",
	       ept->req, ept->req ? ept->req->item_dma : 0);
#endif

	/* expire all requests that are no longer active */
	while ((req = ept->req)) {
		info = req->item->info;

		if (info & INFO_ACTIVE)
			break;

		/* advance ept queue to the next request */
		ept->req = req->next;
		if (ept->req == 0)
			ept->last = 0;

		if (info & (INFO_HALTED | INFO_BUFFER_ERROR | INFO_TXN_ERROR)) {
			/* XXX pass on more specific error code */
			req->req.status = -EIO;
			req->req.actual = 0;
			printk(KERN_INFO "hsusb: ept %d %s error. info=%08x\n",
			       ept->num,
			       (ept->flags & EPT_FLAG_IN) ? "in" : "out",
			       info);
		} else {
			req->req.status = 0;
			req->req.actual = req->req.length - ((info >> 16) & 0x7FFF);
		}
		req->busy = 0;

		if (req->req.complete)
			req->req.complete(ept, &req->req);
	}
}

static void flush_endpoint(struct usb_endpoint *ept)
{
	struct msm_request *req, *next;

	/* cancel any pending requests */
	for (req = ept->req; req; req = next) {
		next = req->next;
#if 0
		printk(KERN_INFO "ept %p canceling req %p\n", ept, req);
#endif
		req->busy = 0;
		req->req.status = -ENODEV;
		req->req.actual = 0;
		if (req->req.complete)
			req->req.complete(ept, &req->req);
	}

	/* zap the queue */
	ept->req = 0;
	ept->last = 0;

	/* put the queue head in a sane state */
	ept->head->info = 0;
	ept->head->next = TERMINATE;
}


static irqreturn_t usb_interrupt(int irq, void *data)
{
	struct usb_info *ui = data;
	unsigned n, i;

	n = readl(USB_USBSTS);
	writel(n, USB_USBSTS);

	if (n & STS_UEI)
		printk(KERN_INFO "usb: error\n");

	if (n & STS_PCI)
		printk(KERN_INFO "usb: portchange\n");

	if (n & STS_URI) {
		ui->online = 0;

		writel(readl(USB_ENDPTCOMPLETE), USB_ENDPTCOMPLETE);
		writel(readl(USB_ENDPTSETUPSTAT), USB_ENDPTSETUPSTAT);
		writel(0xffffffff, USB_ENDPTFLUSH);
		writel(0, USB_ENDPTCTRL(1));

		for (i = 0; i < 32; i++) {
			if (ui->ept[i].req)
				flush_endpoint(ui->ept + i);
		}

		/* XXX: we can't seem to detect going offline, so deconfigure
		 * XXX: on reset for the time being
		 */
		printk(KERN_INFO "usb: reset\n");
		set_configuration(ui, 0);
	}

	if (n & STS_SLI)
		printk(KERN_INFO "usb: suspend\n");

	if (n & STS_UI) {
		n = readl(USB_ENDPTCOMPLETE);
		writel(n, USB_ENDPTCOMPLETE);
		while (n) {
			unsigned bit = __ffs(n);
			handle_endpoint(ui, bit);
			n = n & (~(1 << bit));
		}

		n = readl(USB_ENDPTSETUPSTAT);
		if (n & EPT_RX(0))
			handle_setup(ui);
	}
	return IRQ_HANDLED;
}

static int usb_prepare(struct usb_info *ui)
{
	spin_lock_init(&ui->lock);

	memset(ui->buf, 0, 4096);
	ui->head = (void *) (ui->buf + 0);
	ui->item = (void *) (ui->buf + 2048);

	init_endpoints(ui);

	ui->ep0in.max_pkt = 64;
	ui->ep0out.max_pkt = 64;

	ui->setup_req = usb_ept_alloc_req(&ui->ep0in, 4096);

	/* select ULPI phy */
	writel(0x81000000, USB_PORTSC);

	/* RESET */
	writel(0x00080002, USB_USBCMD);
	mdelay(10);

	writel(ui->dma, USB_ENDPOINTLISTADDR);

	/* select DEVICE mode */
	writel(0x02, USB_USBMODE);

	writel(0xffffffff, USB_ENDPTFLUSH);

	return 0;
}

static void usb_bind_driver(struct usb_info *ui, struct usb_function_info *fi)
{
	struct usb_endpoint *ept;
	struct usb_endpoint_descriptor *ed;
	struct usb_endpoint *elist[8];
	struct usb_function *func = fi->func;
	unsigned n, count;

	printk(KERN_INFO "usb_bind_func() '%s'\n", func->name);

	count = func->ifc_ept_count;

	if (count > 8)
		return;

	fi->ifc.bLength = USB_DT_INTERFACE_SIZE;
	fi->ifc.bDescriptorType = USB_DT_INTERFACE;
	fi->ifc.bAlternateSetting = 0;
	fi->ifc.bNumEndpoints = count;
	fi->ifc.bInterfaceClass = func->ifc_class;
	fi->ifc.bInterfaceSubClass = func->ifc_subclass;
	fi->ifc.bInterfaceProtocol = func->ifc_protocol;
	fi->ifc.iInterface = 0;

	for (n = 0; n < count; n++) {
		ept = alloc_endpoint(ui, fi, func->ifc_ept_type[n]);
		if (!ept) {
			printk(KERN_INFO
			       "failed to allocated endpoint %d\n", n);
			free_endpoints(ui, fi);
			return;
		}
		ed = &(fi->ept[n].desc);

		ed->bLength = USB_DT_ENDPOINT_SIZE;
		ed->bDescriptorType = USB_DT_ENDPOINT;
		ed->bEndpointAddress = ept->num | ((ept->flags & EPT_FLAG_IN) ? 0x80 : 0);
		ed->bmAttributes = 0x02; /* XXX hardcoded bulk */
		ed->wMaxPacketSize = ept->max_pkt;
		ed->bInterval = 0; /* XXX hardcoded bulk */

		elist[n] = ept;
		fi->ept[n].ept = ept;
	}

	fi->ifc.bInterfaceNumber = ui->next_ifc_num++;

	fi->endpoints = count;

	func->bind(elist, func->context);
}

static void usb_start(struct usb_info *ui, struct list_head *flist)
{
	unsigned count = 0;

	struct list_head *entry;
	list_for_each(entry, flist) {
		struct usb_function_info *fi =
			list_entry(entry, struct usb_function_info, list);
		usb_bind_driver(ui, fi);
		if (fi->endpoints)
			count++;
	}

	configure_endpoints(ui);

	ui->flist = flist;

	if (count == 0) {
		printk(KERN_INFO
		       "usb_start: no functions bound. not starting\n");
		return;
	}

	/* enable interrupts */
	writel(STS_URI | STS_SLI | STS_UI | STS_PCI | STS_UEI, USB_USBINTR);

	/* go to RUN mode (D+ pullup enable) */
	writel(0x00080001, USB_USBCMD);
}

static int usb_free(struct usb_info *ui, int ret)
{
	if (ui->irq)
		free_irq(ui->irq, 0);
	if (ui->dma)
		dma_free_coherent(&ui->pdev->dev, 4096, ui->buf, ui->dma);
	if (ui->addr)
		iounmap(ui->addr);
	kfree(ui);
	return ret;
}


static struct usb_info *the_usb_info;

static LIST_HEAD(func_list);


ssize_t usb_show_status(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct usb_info *ui = the_usb_info;
	unsigned long flags;
	struct usb_endpoint *ept;
	struct msm_request *req;
	int n;
	int i = 0;

	spin_lock_irqsave(&ui->lock, flags);

	i += scnprintf(buf + i, PAGE_SIZE - i,
		       "regs: setup=%08x prime=%08x stat=%08x done=%08x\n\n",
		       readl(USB_ENDPTSETUPSTAT),
		       readl(USB_ENDPTPRIME),
		       readl(USB_ENDPTSTAT),
		       readl(USB_ENDPTCOMPLETE));


	for (n = 0; n < 32; n++) {
		ept = ui->ept + n;
		if (ept->max_pkt == 0)
			continue;

		i += scnprintf(buf + i, PAGE_SIZE - i,
			       "ept%d %s cfg=%08x active=%08x next=%08x info=%08x\n",
			       ept->num, (ept->flags & EPT_FLAG_IN) ? "in " : "out",
			       ept->head->config, ept->head->active,
			       ept->head->next, ept->head->info);

		for (req = ept->req; req; req = req->next)
			i += scnprintf(buf + i, PAGE_SIZE - i,
				       "  req @%08x next=%08x info=%08x page0=%08x\n",
				       req->item_dma, req->item->next,
				       req->item->info, req->item->page0);
	}
	spin_unlock_irqrestore(&ui->lock, flags);

	return i;
}

static DEVICE_ATTR(status, 0600, usb_show_status, 0);

static int usb_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct usb_info *ui;
	int irq;
	int ret;

	ui = kzalloc(sizeof(struct usb_info), GFP_KERNEL);
	if (!ui)
		return -ENOMEM;

	ui->pdev = pdev;

	irq = platform_get_irq(pdev, 0);
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res || (irq < 0))
		return usb_free(ui, -ENODEV);

	ui->addr = ioremap(res->start, 4096);
	if (!ui->addr)
		return usb_free(ui, -ENOMEM);

	ui->buf = dma_alloc_coherent(&pdev->dev, 4096, &ui->dma, GFP_KERNEL);
	if (!ui->buf)
		return usb_free(ui, -ENOMEM);

	printk(KERN_INFO "usb_probe() io=%p, irq=%d, dma=%p(%x)\n",
	       ui->addr, irq, ui->buf, ui->dma);

	ret = request_irq(irq, usb_interrupt, 0, pdev->name, ui);
	if (ret)
		return usb_free(ui, ret);
	ui->irq = irq;

	ret = usb_prepare(ui);
	if (ret)
		return usb_free(ui, ret);

	the_usb_info = ui;

	if (device_create_file(&pdev->dev, &dev_attr_status))
		printk(KERN_ERR "usb_probe: failed to publish status file\n");

	return 0;
}

int usb_function_register(struct usb_function *driver)
{
	struct usb_function_info *fi;
	unsigned n;

	n = driver->ifc_ept_count;

	printk(KERN_INFO "usb_function_register() '%s'\n", driver->name);

	fi = kzalloc(sizeof(*fi) + n * sizeof(struct usb_fi_ept), GFP_KERNEL);
	if (!fi)
		return -ENOMEM;

	INIT_LIST_HEAD(&fi->list);
	fi->func = driver;

	list_add(&fi->list, &func_list);

	return 0;
}

static struct platform_driver usb_driver = {
	.probe = usb_probe,
	.driver = { .name = "msm_hsusb", },
};

static int __init usb_init(void)
{
	return platform_driver_register(&usb_driver);
}

static int __init usb_late_init(void)
{
	printk(KERN_INFO "usb_late_init()\n");

	if (the_usb_info)
		usb_start(the_usb_info, &func_list);

	return 0;
}

module_init(usb_init);

late_initcall(usb_late_init);


/* -- -- -- -- -- -- -- -- android driver -- -- -- -- -- -- -- */

struct usb_device_descriptor desc_device = {
	.bLength = USB_DT_DEVICE_SIZE,
	.bDescriptorType = USB_DT_DEVICE,

	.bcdUSB = 0x0102,
	.bDeviceClass = 0,
	.bDeviceSubClass = 0,
	.bDeviceProtocol = 0,
	.bMaxPacketSize0 = 64,
	.idVendor = 0x18d1,
	.idProduct = 0xd00d,
	.bcdDevice = 0x0100,
	.iManufacturer = 0,
	.iProduct = 0,
	.iSerialNumber = 0,
	.bNumConfigurations = 1,
};

int usb_find_descriptor(struct usb_info *ui, unsigned id, struct usb_request *req)
{
	unsigned type = id >> 8;
	id &= 0xff;

	if ((type == USB_DT_DEVICE) && (id == 0)) {
		req->length = sizeof(desc_device);
		memcpy(req->buf, &desc_device, req->length);
		return 0;
	}

	if ((type == USB_DT_CONFIG) && (id == 0)) {
		struct list_head *entry;
		struct usb_config_descriptor cfg;
		unsigned ifc_count = 0;
		unsigned n;
		char *ptr, *start;

		start = req->buf;
		ptr = start + USB_DT_CONFIG_SIZE;

		list_for_each(entry, ui->flist) {
			struct usb_function_info *fi =
				list_entry(entry, struct usb_function_info, list);
			if (fi->endpoints == 0)
				continue; /* 0 endpoints -> unbound, ignore */

			ifc_count++;

			memcpy(ptr, &fi->ifc, fi->ifc.bLength);
			ptr += fi->ifc.bLength;

			for (n = 0; n < fi->endpoints; n++) {
				memcpy(ptr, &(fi->ept[n].desc), fi->ept[n].desc.bLength);
				ptr += fi->ept[n].desc.bLength;
			}
		}

		cfg.bLength = USB_DT_CONFIG_SIZE;
		cfg.bDescriptorType = USB_DT_CONFIG;
		cfg.wTotalLength = ptr - start;
		cfg.bNumInterfaces = ifc_count;
		cfg.bConfigurationValue = 1;
		cfg.iConfiguration = 0;
		cfg.bmAttributes = 0x80;
		cfg.bMaxPower = 0x80;

		memcpy(start, &cfg, USB_DT_CONFIG_SIZE);

		req->length = ptr - start;
		return 0;
	}

	return -1;
}

