/* arch/arm/mach-msm/adsp_driver.c
 *
 * Copyright (C) 2008 Google, Inc.
 * Author: Iliyan Malchev <ibm@android.com>
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

#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/list.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>

#include "adsp.h"

#include <linux/msm_adsp.h>
#include <linux/android_pmem.h>

#define MAX_DATA_SIZE 496

/* This gets mapped to an adsp_event_t which is sent to the user. */
struct adsp_event {
	struct list_head list;
	uint32_t size; /* always in bytes */
	uint16_t msg_id;
	uint16_t type; /* 0 for msgs (from aDSP), 1 for events (from ARM9) */
	int is16; /* always 0 (msg is 32-bit) when the event type is 1(ARM9) */
	union {
		uint16_t msg16[MAX_DATA_SIZE / 2];
		uint32_t msg32[MAX_DATA_SIZE / 4];
	} data;
};

struct adsp_pmem_info {
	int fd;
	void *vaddr;
};

struct adsp_pmem_region {
	struct hlist_node list;
	void *vaddr;
	unsigned long paddr;
	unsigned long len;
	struct file *file;
};

struct adsp_device {
	struct msm_adsp_module *module;

	spinlock_t event_queue_lock;
	wait_queue_head_t event_wait;
	struct list_head event_queue;
	int abort;

	struct hlist_head regions;

	const char *name;
	struct device *device;
	struct cdev cdev;
};

static struct adsp_device *inode_to_device(struct inode *inode);

static int adsp_pmem_table_add(struct adsp_device *adev, struct file *file,
			       void *vaddr, unsigned long paddr,
			       unsigned long len)
{
	struct adsp_pmem_region *region =
		kmalloc(sizeof(*region), GFP_KERNEL);
	if (!region)
		return -ENOMEM;
	INIT_HLIST_NODE(&region->list);
	region->vaddr = vaddr;
	region->paddr = paddr;
	region->len = len;
	region->file = file;
	hlist_add_head(&region->list, &adev->regions);
	return 0;
}

unsigned long adsp_pmem_table_lookup(struct adsp_device *adev,
				     void *vaddr, unsigned long len)
{
	struct hlist_node *node;
	struct adsp_pmem_region *region;

	/* returns physical address or zero */
	hlist_for_each_entry(region, node, &adev->regions, list) {
		if (vaddr >= region->vaddr && vaddr < region->vaddr + len &&
		    vaddr + len <= region->vaddr + region->len)
			/* offset since we could pass vaddr inside a registerd
			 *  pmem buffer */
			return region->paddr + (vaddr - region->vaddr);
	}
	return 0;
}

#if 0
int adsp_pmem_table_remove(struct adsp_device *adev, void *vaddr)
{
	struct hlist_node *entry;
	struct adsp_pmem_region *region;
	/* returns physical address or zero */
	hlist_for_each(entry, &adev->regions) {
		region = hlist_entry(entry, struct adsp_pmem_region, list);
		if (region->vaddr == vaddr) {
			hlist_del(entry);
			kfree(region);
			return 0;
		}
	}
	return -1;
}
#endif

#define LOAD16(bottom, top) ((void *)((unsigned long)bottom | \
	 (((unsigned long)top) << 16)))
#define STORE16(val, bottom, top) do { bottom = val & 0xffff;\
				       top = (val & 0xffff0000) >> 16;\
				     } while (0)

static int adsp_verify_cmd(struct adsp_device *adsp, uint32_t dsp_queue_addr,
			   void *cmd_buf, uint32_t cmd_size)
{
	uint16_t *cmd16 = (uint16_t *)cmd_buf;
	unsigned long paddr;
	void *vaddr;
	if (dsp_queue_addr == QDSP_uPAudPPCmd2Queue &&
	    cmd16[2] == 0 && /* AUDPP_CMD_HOST_PCM_CONFIG_CMD_V */
 	    cmd16[0] == 1) { /* QDSP_AUDPPTASK_AUDPP_HOSTPCM_CMD_HNDLR */
		vaddr = LOAD16(cmd16[12], cmd16[13]);
		paddr = adsp_pmem_table_lookup(adsp, vaddr, cmd16[14]);
		if (!paddr)
			return -1;
		STORE16(paddr, cmd16[12], cmd16[13]);
		vaddr = LOAD16(cmd16[15], cmd16[16]);
		paddr = adsp_pmem_table_lookup(adsp, vaddr, cmd16[17]);
		if (!paddr)
			return -1;
		STORE16(paddr, cmd16[15], cmd16[16]);
	} else if (dsp_queue_addr == QDSP_uPAudRecCmdQueue &&
		   cmd16[0] == 1) {
			/*QDSP_AUDRECTASK_AUDREC_CMD_AREC0PARAM_CFG*/
		vaddr = LOAD16(cmd16[2], cmd16[1]);
		paddr = adsp_pmem_table_lookup(adsp, vaddr, cmd16[3]);
		if (!paddr)
			return -1;
		STORE16(paddr, cmd16[2], cmd16[1]);
	}
	return 0;
}

static long adsp_write_cmd(struct adsp_device *adev, void __user *arg)
{
	struct adsp_command_t cmd;
	unsigned char buf[256];
	void *cmd_data;
	long rc;

	if (copy_from_user(&cmd, (void __user *)arg, sizeof(cmd)))
		return -EFAULT;

	if (cmd.len > 256) {
		cmd_data = kmalloc(cmd.len, GFP_USER);
		if (!cmd_data)
			return -ENOMEM;
	} else {
		cmd_data = buf;
	}

	if (copy_from_user(cmd_data, (void __user *)(cmd.data), cmd.len)) {
		rc = -EFAULT;
		goto end;
	}
	if (adsp_verify_cmd(adev, cmd.queue, cmd_data, cmd.len)) {
		rc = -EINVAL;
		goto end;
	}
	rc = msm_adsp_write(adev->module, cmd.queue, cmd_data, cmd.len);

end:
	if (cmd.len > 256)
		kfree(cmd_data);

	return rc;
}

static int adsp_events_pending(struct adsp_device *adev)
{
	unsigned long flags;
	int yes;
	spin_lock_irqsave(&adev->event_queue_lock, flags);
	yes = !list_empty(&adev->event_queue);
	spin_unlock_irqrestore(&adev->event_queue_lock, flags);
	return yes || adev->abort;
}

static long adsp_get_event(struct adsp_device *adev, void __user *arg)
{
	unsigned long flags;
	struct adsp_event *data = NULL;
	struct adsp_event_t evt;
	int timeout;
	long rc = 0;

	if (copy_from_user(&evt, arg, sizeof(struct adsp_event_t)))
		return -EFAULT;

	timeout = (int)evt.timeout_ms;

	if (timeout > 0) {
		rc = wait_event_interruptible_timeout(
			adev->event_wait, adsp_events_pending(adev),
			msecs_to_jiffies(timeout));
		if (rc == 0)
			return -ETIMEDOUT;
	} else {
		rc = wait_event_interruptible(
			adev->event_wait, adsp_events_pending(adev));
	}
	if (rc < 0)
		return rc;

	if (adev->abort)
		return -ENODEV;

	spin_lock_irqsave(&adev->event_queue_lock, flags);
	if (!list_empty(&adev->event_queue)) {
		data = list_first_entry(&adev->event_queue,
					struct adsp_event, list);
		list_del(&data->list);
	}
	spin_unlock_irqrestore(&adev->event_queue_lock, flags);

	if (!data)
		return -EAGAIN;

	/* map adsp_event --> adsp_event_t */
	if (evt.len < data->size) {
		rc = -ETOOSMALL;
		goto end;
	}
	if (copy_to_user((void *)(evt.data), data->data.msg16, data->size)) {
		rc = -EFAULT;
		goto end;
	}

	evt.type = data->type; /* 0 --> from aDSP, 1 --> from ARM9 */
	evt.msg_id = data->msg_id;
	evt.flags = data->is16;
	evt.len = data->size;
	if (copy_to_user(arg, &evt, sizeof(evt)))
		rc = -EFAULT;
end:
	kfree(data);
	return rc;
}


static long adsp_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct adsp_device *adev = filp->private_data;

	switch (cmd) {
	case ADSP_IOCTL_ENABLE:
		return msm_adsp_enable(adev->module);

	case ADSP_IOCTL_DISABLE:
		return msm_adsp_disable(adev->module);

	case ADSP_IOCTL_DISABLE_EVENT_RSP:
#if 0
		pr_info("adsp: disabling event response for module %s\n",
			adsp_module->name);
		mutex_lock(&adsp_module->lock);
		rc = rpc_adsp_rtos_app_to_modem(
			RPC_ADSP_RTOS_CMD_DISABLE_EVENT_RSP,
			adsp_module->id, adsp_module);
		mutex_unlock(&adsp_module->lock);
#endif
		return 0;

	case ADSP_IOCTL_DISABLE_ACK:
		pr_err("adsp: ADSP_IOCTL_DISABLE_ACK is not implemented.\n");
		break;

	case ADSP_IOCTL_WRITE_COMMAND:
		return adsp_write_cmd(adev, (void __user *) arg);

	case ADSP_IOCTL_GET_EVENT:
		return adsp_get_event(adev, (void __user *) arg);

	case ADSP_IOCTL_REGISTER_PMEM: {
		struct adsp_pmem_info info;
		unsigned long paddr, len;
		struct file *file;
		if (copy_from_user(&info, (void *) arg, sizeof(info)))
			return -EFAULT;
		get_pmem_file(info.fd, &paddr, &len, &file);
		return adsp_pmem_table_add(adev, file, info.vaddr, paddr, len);
	}
#if 0 /* XXX not valid while module enabled */
	case ADSP_IOCTL_UNREGISTER_PMEM: {
		struct adsp_pmem_info info;
		if (copy_from_user(&info, (void *)arg, sizeof(info)))
			return -EFAULT;
		put_pmem_fd(info.fd);
		rc = adsp_pmem_table_remove(adsp_module, info.vaddr);
		break;
	}
#endif
	case ADSP_IOCTL_ABORT_EVENT_READ:
		adev->abort = 1;
		wake_up(&adev->event_wait);
		break;
	default:
		break;
	}
	return -EINVAL;
}

static int adsp_release(struct inode *inode, struct file *filp)
{
	struct adsp_device *adev = filp->private_data;
	struct msm_adsp_module *module = adev->module;
	struct hlist_node *node, *tmp;
	struct adsp_pmem_region *region;

	pr_info("adsp_release() '%s'\n", adev->name);

	/* clear module before putting it to avoid race with open() */
	adev->module = NULL;

	hlist_for_each_safe(node, tmp, &adev->regions) {
		region = hlist_entry(node, struct adsp_pmem_region, list);
		hlist_del(node);
		put_pmem_file(region->file);
		kfree(region);
	}

	msm_adsp_put(module);
	return 0;
}

static void adsp_event(void *driver_data, unsigned id, size_t len,
		       void (*getevent)(void *ptr, size_t len))
{
	struct adsp_device *adev = driver_data;
	struct adsp_event *event;
	unsigned long flags;

	if (len > MAX_DATA_SIZE) {
		pr_err("adsp_event: event too large (%d bytes)\n", len);
		return;
	}

	event = kmalloc(sizeof(*event), GFP_ATOMIC);
	if (!event) {
		pr_err("adsp_event: cannot allocate buffer\n");
		return;
	}

	event->type = 0;
	event->is16 = 0;
	event->msg_id = id;
	event->size = len;

	getevent(event->data.msg16, len);

	spin_lock_irqsave(&adev->event_queue_lock, flags);
	list_add_tail(&event->list, &adev->event_queue);
	spin_unlock_irqrestore(&adev->event_queue_lock, flags);
	wake_up(&adev->event_wait);
}

static struct msm_adsp_ops adsp_ops = {
	.event = adsp_event,
};

static int adsp_open(struct inode *inode, struct file *filp)
{
	struct adsp_device *adev;
	int rc;

	rc = nonseekable_open(inode, filp);
	if (rc < 0)
		return rc;

	adev = inode_to_device(inode);
	if (!adev)
		return -ENODEV;

	pr_info("adsp_open() name = '%s'\n", adev->name);

	rc = msm_adsp_get(adev->name, &adev->module, &adsp_ops, adev);
	if (rc)
		return rc;

	pr_info("adsp_open() module '%s' adev %p\n", adev->name, adev);
	filp->private_data = adev;
	adev->abort = 0;

	return 0;
}

static unsigned adsp_device_count;
static struct adsp_device *adsp_devices;

static struct adsp_device *inode_to_device(struct inode *inode)
{
	unsigned n = MINOR(inode->i_rdev);
	if (n < adsp_device_count) {
		if (adsp_devices[n].device)
			return adsp_devices + n;
	}
	return NULL;
}

static dev_t adsp_devno;
static struct class *adsp_class;

static struct file_operations adsp_fops = {
	.owner = THIS_MODULE,
	.open = adsp_open,
	.unlocked_ioctl = adsp_ioctl,
	.release = adsp_release,
};

static void adsp_create(struct adsp_device *adev, const char *name,
			struct device *parent, dev_t devt)
{
	struct device *dev;
	int rc;

	dev = device_create(adsp_class, parent, devt, "%s", name);
	if (IS_ERR(dev))
		return;

	init_waitqueue_head(&adev->event_wait);
	INIT_LIST_HEAD(&adev->event_queue);
	INIT_HLIST_HEAD(&adev->regions);
	spin_lock_init(&adev->event_queue_lock);

	cdev_init(&adev->cdev, &adsp_fops);
	adev->cdev.owner = THIS_MODULE;

	rc = cdev_add(&adev->cdev, devt, 1);
	if (rc < 0) {
		device_destroy(adsp_class, devt);
	} else {
		adev->device = dev;
		adev->name = name;
	}
}

void msm_adsp_publish_cdevs(struct msm_adsp_module *modules, unsigned n)
{
	int rc;

	adsp_devices = kzalloc(sizeof(struct adsp_device) * n, GFP_KERNEL);
	if (!adsp_devices)
		return;

	adsp_class = class_create(THIS_MODULE, "adsp");
	if (IS_ERR(adsp_class))
		goto fail_create_class;

	rc = alloc_chrdev_region(&adsp_devno, 0, n, "adsp");
	if (rc < 0)
		goto fail_alloc_region;

	adsp_device_count = n;
	for (n = 0; n < adsp_device_count; n++) {
		adsp_create(adsp_devices + n,
			    modules[n].name, &modules[n].pdev.dev,
			    MKDEV(MAJOR(adsp_devno), n));
	}

	return;

fail_alloc_region:
	class_unregister(adsp_class);
fail_create_class:
	kfree(adsp_devices);
}

