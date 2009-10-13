/* drivers/usb/function/adb.c
 *
 * Function Device for the Android ADB Protocol
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
#include <linux/miscdevice.h>
#include <linux/fs.h>

#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/list.h>

#include <asm/atomic.h>
#include <asm/uaccess.h>

#include "usb_function.h"

#if 1
#define DBG(x...) do {} while (0)
#else
#define DBG(x...) printk(x)
#endif

#define TXN_MAX 4096

/* number of tx requests to allocate */
#define TX_REQ_MAX 4

#define ADB_FUNCTION_NAME "adb"

struct adb_context
{
	int online;
	int error;

	atomic_t read_excl;
	atomic_t write_excl;
	atomic_t open_excl;
	atomic_t enable_excl;
	spinlock_t lock;

	struct usb_endpoint *out;
	struct usb_endpoint *in;

	struct list_head tx_idle;

	wait_queue_head_t read_wq;
	wait_queue_head_t write_wq;
	struct usb_request *rx_req;
	int rx_done;
};

static struct adb_context _context;

static inline int _lock(atomic_t *excl)
{
	if (atomic_inc_return(excl) == 1) {
		return 0;
	} else {
		atomic_dec(excl);
		return -1;
	}
}

static inline void _unlock(atomic_t *excl)
{
	atomic_dec(excl);
}

/* add a request to the tail of a list */
void req_put(struct adb_context *ctxt, struct list_head *head, struct usb_request *req)
{
	unsigned long flags;

	spin_lock_irqsave(&ctxt->lock, flags);
	list_add_tail(&req->list, head);
	spin_unlock_irqrestore(&ctxt->lock, flags);
}

/* remove a request from the head of a list */
struct usb_request *req_get(struct adb_context *ctxt, struct list_head *head)
{
	unsigned long flags;
	struct usb_request *req;

	spin_lock_irqsave(&ctxt->lock, flags);
	if (list_empty(head)) {
		req = 0;
	} else {
		req = list_first_entry(head, struct usb_request, list);
		list_del(&req->list);
	}
	spin_unlock_irqrestore(&ctxt->lock, flags);
	return req;
}

static void adb_complete_in(struct usb_endpoint *ept, struct usb_request *req)
{
	struct adb_context *ctxt = req->context;

	if (req->status != 0)
		ctxt->error = 1;

	req_put(ctxt, &ctxt->tx_idle, req);

	wake_up(&ctxt->write_wq);
}

static void adb_complete_out(struct usb_endpoint *ept, struct usb_request *req)
{
	struct adb_context *ctxt = req->context;

	DBG("out: status: %d actual: %d\n", req->status, req->actual);
	ctxt->rx_done = 1;
	wake_up(&ctxt->read_wq);
}

static ssize_t adb_read(struct file *fp, char __user *buf,
			size_t count, loff_t *pos)
{
	struct adb_context *ctxt = &_context;
	struct usb_request *req;
	int r = count, xfer;
	int ret;

	DBG("adb_read(%d)\n", count);

	if (count > TXN_MAX)
		return -EINVAL;

	if (_lock(&ctxt->read_excl))
		return -EBUSY;

	/* we will block until we're online */
	while (!(ctxt->online || ctxt->error)) {
		DBG("adb_read: waiting for online state\n");
		ret = wait_event_interruptible(ctxt->read_wq, (ctxt->online || ctxt->error));
		if (ret < 0) {
			r = ret;
			goto done;
		}
	}
	if (ctxt->error) {
		r = -EIO;
		goto done;
	}

requeue_req:
	/* queue a request */
	req = ctxt->rx_req;
	req->length = count;
	ctxt->rx_done = 0;
	ret = usb_ept_queue_xfer(ctxt->out, req);
	if (ret < 0) {
		DBG("adb_read: failed to queue req %p (%d)\n", req, ret);
		r = -EIO;
		ctxt->error = 1;
		goto done;
	} else {
		DBG("rx %p queue\n", req);
	}

	/* wait for a request to complete */
	wait_event(ctxt->read_wq, ctxt->rx_done);

	if (!ctxt->error) {
		/* If we got a 0-len packet, throw it back and try again. */
		if (req->actual == 0)
			goto requeue_req;

		DBG("rx %p %d\n", req, req->actual);
		xfer = (req->actual < count) ? req->actual : count;
		if (copy_to_user(buf, req->buf, xfer))
			r = -EFAULT;
	} else
		r = -EIO;

done:
	_unlock(&ctxt->read_excl);
	return r;
}

static ssize_t adb_write(struct file *fp, const char __user *buf,
			 size_t count, loff_t *pos)
{
	struct adb_context *ctxt = &_context;
	struct usb_request *req = 0;
	int r = count, xfer;
	int ret;

	DBG("adb_write(%d)\n", count);

	if (_lock(&ctxt->write_excl))
		return -EBUSY;

	while (count > 0) {
		if (ctxt->error) {
			r = -EIO;
			break;
		}

		/* get an idle tx request to use */
		req = 0;
		ret = wait_event_interruptible(ctxt->write_wq,
					       ((req = req_get(ctxt, &ctxt->tx_idle)) || ctxt->error));

		if (ret < 0) {
			r = ret;
			break;
		}

		if (req != 0) {
			xfer = count > TXN_MAX ? TXN_MAX : count;
			if (copy_from_user(req->buf, buf, xfer)) {
				r = -EFAULT;
				break;
			}

			req->length = xfer;
			ret = usb_ept_queue_xfer(ctxt->in, req);
			if (ret < 0) {
				DBG("adb_write: xfer error %d\n", ret);
				ctxt->error = 1;
				r = -EIO;
				break;
			}

			buf += xfer;
			count -= xfer;

			/* zero this so we don't try to free it on error exit */
			req = 0;
		}
	}


	if (req)
		req_put(ctxt, &ctxt->tx_idle, req);

	_unlock(&ctxt->write_excl);
	return r;
}

static int adb_open(struct inode *ip, struct file *fp)
{
	struct adb_context *ctxt = &_context;

	if (_lock(&ctxt->open_excl))
		return -EBUSY;

	/* clear the error latch */
	ctxt->error = 0;

	return 0;
}

static int adb_release(struct inode *ip, struct file *fp)
{
	struct adb_context *ctxt = &_context;

	_unlock(&ctxt->open_excl);
	return 0;
}

static struct file_operations adb_fops = {
	.owner =   THIS_MODULE,
	.read =    adb_read,
	.write =   adb_write,
	.open =    adb_open,
	.release = adb_release,
};

static struct miscdevice adb_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "android_adb",
	.fops = &adb_fops,
};

static int adb_enable_open(struct inode *ip, struct file *fp)
{
	struct adb_context *ctxt = &_context;

	if (_lock(&ctxt->enable_excl))
		return -EBUSY;

	printk(KERN_INFO "enabling adb function\n");
	usb_function_enable(ADB_FUNCTION_NAME, 1);
	/* clear the error latch */
	ctxt->error = 0;

	return 0;
}

static int adb_enable_release(struct inode *ip, struct file *fp)
{
	struct adb_context *ctxt = &_context;

	printk(KERN_INFO "disabling adb function\n");
	usb_function_enable(ADB_FUNCTION_NAME, 0);
	_unlock(&ctxt->enable_excl);
	return 0;
}

static struct file_operations adb_enable_fops = {
	.owner =   THIS_MODULE,
	.open =    adb_enable_open,
	.release = adb_enable_release,
};

static struct miscdevice adb_enable_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "android_adb_enable",
	.fops = &adb_enable_fops,
};

static void adb_unbind(void *_ctxt)
{
	struct adb_context *ctxt = _ctxt;
	struct usb_request *req;

	printk(KERN_INFO "abd_unbind()\n");

	usb_ept_free_req(ctxt->out, ctxt->rx_req);
	while ((req = req_get(ctxt, &ctxt->tx_idle))) {
		usb_ept_free_req(ctxt->in, req);
	}

	ctxt->online = 0;
	ctxt->error = 1;

	/* readers may be blocked waiting for us to go online */
	wake_up(&ctxt->read_wq);
}

static void adb_bind(struct usb_endpoint **ept, void *_ctxt)
{
	struct adb_context *ctxt = _ctxt;
	struct usb_request *req;
	int n;

	ctxt->out = ept[0];
	ctxt->in = ept[1];

	printk(KERN_INFO "adb_bind() %p, %p\n", ctxt->out, ctxt->in);

	req = usb_ept_alloc_req(ctxt->out, 4096);
	if (req == 0) goto fail;
	req->context = ctxt;
	req->complete = adb_complete_out;
	ctxt->rx_req = req;

	for (n = 0; n < TX_REQ_MAX; n++) {
		req = usb_ept_alloc_req(ctxt->in, 4096);
		if (req == 0) goto fail;
		req->context = ctxt;
		req->complete = adb_complete_in;
		req_put(ctxt, &ctxt->tx_idle, req);
	}

	printk(KERN_INFO
	       "adb_bind() allocated %d tx requests\n", TX_REQ_MAX);

	misc_register(&adb_device);
	misc_register(&adb_enable_device);
	return;

fail:
	printk(KERN_ERR "adb_bind() could not allocate requests\n");
	adb_unbind(ctxt);
}

static void adb_configure(int configured, void *_ctxt)
{
	struct adb_context *ctxt = _ctxt;

	DBG("adb_configure() %d\n", configured);

	if (configured) {
		ctxt->online = 1;
	} else {
		ctxt->online = 0;
		ctxt->error = 1;
	}

	/* readers may be blocked waiting for us to go online */
	wake_up(&ctxt->read_wq);
}

static struct usb_function usb_func_adb = {
	.bind = adb_bind,
	.unbind = adb_unbind,
	.configure = adb_configure,

	.name = ADB_FUNCTION_NAME,
	.context = &_context,

	.ifc_class = 0xff,
	.ifc_subclass = 0x42,
	.ifc_protocol = 0x01,

	.ifc_name = "adb",

	.ifc_ept_count = 2,
	.ifc_ept_type = { EPT_BULK_OUT, EPT_BULK_IN },

	/* the adb function is only enabled when its driver file is open */
	.disabled = 1,
};

static int __init adb_init(void)
{
	struct adb_context *ctxt = &_context;
	DBG("adb_init()\n");

	init_waitqueue_head(&ctxt->read_wq);
	init_waitqueue_head(&ctxt->write_wq);

	atomic_set(&ctxt->open_excl, 0);
	atomic_set(&ctxt->read_excl, 0);
	atomic_set(&ctxt->write_excl, 0);
	atomic_set(&ctxt->enable_excl, 0);

	spin_lock_init(&ctxt->lock);

	INIT_LIST_HEAD(&ctxt->tx_idle);

	return usb_function_register(&usb_func_adb);
}

module_init(adb_init);
