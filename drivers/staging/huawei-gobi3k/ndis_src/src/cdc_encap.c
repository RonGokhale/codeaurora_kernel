/* Copyright (C) 2011 by Franko Fang (Huawei Technologies Co., Ltd.)*/

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#include "cdc_encap.h"
#include <asm/uaccess.h>

static struct class *hw_class;
static struct device *hw_device;
static struct cdc_encap *encap;
static const char dev_name_prefix[] = CDC_ENCAP_NAME_PREFIX;
#define MAJOR_NUM       251
static ssize_t classfile_sysfs_show(struct class *class, char *buf)
{
         printk(KERN_INFO "class file show \n");
        return 0;
}
 
static ssize_t devfile_sysfs_show(struct device *dev, struct device_attribute *attr,
                        char *buf)
{
         printk(KERN_INFO "dev file show \n");
        return 0;
}
 
static CLASS_ATTR(classfile, S_IRUGO, classfile_sysfs_show, NULL);
static DEVICE_ATTR(devfile, S_IRUGO, devfile_sysfs_show, NULL);

static ssize_t cdc_encap_read(struct file *filp, char __user *buf, size_t size,
	loff_t *z)
{
	struct cdc_encap *encap = filp->private_data;
	void *data = NULL;
	int status;
	struct usb_interface *intf = NULL;
	printk(KERN_ERR"fxz-%s: called\n", __func__);
	
	if (NULL == encap || NULL == encap->udev || NULL == encap->udev->actconfig) {
		return -ENODEV;
	}
	
	
	if (size) {
		data = kmalloc(size, GFP_KERNEL);
		if (data == NULL)
			return -ENOMEM;
	}
	if (encap->iface_num < encap->udev->actconfig->desc.bNumInterfaces)
		intf = encap->udev->actconfig->interface[encap->iface_num];
	if (NULL != intf)
		usb_autopm_get_interface(intf);
	encap->rsp_ready = 0;

	status = usb_control_msg(encap->udev, usb_rcvctrlpipe(encap->udev, 0),
		CDC_ENCAP_REQ_GET_RESPONSE, CDC_ENCAP_REQT_IN, 0,
		cpu_to_le16(encap->iface_num), data, size,
		msecs_to_jiffies(CDC_ENCAP_TIMEOUT_MS));
	if (status <= 0 || size == 0)
		goto out;

	if (copy_to_user(buf, data, status))
		status = -EACCES;
out:
	if (data)
		kfree(data);
	if (NULL != intf)
		usb_autopm_put_interface(intf);
	return status;
}

static ssize_t cdc_encap_write(struct file *filp, const char __user *buf,
	size_t size, loff_t *z)
{
	struct cdc_encap *encap = filp->private_data;
	void *data = NULL;
	int status;
	struct usb_interface *intf = NULL;
	if (NULL == encap || NULL == encap->udev || NULL == encap->udev->actconfig) {
		return -ENODEV;
	}
	printk(KERN_ERR"fxz-%s: called\n", __func__);
	if (size) {
		data = kmalloc(size, GFP_KERNEL);
		if (data == NULL)
			return -ENOMEM;

		if (copy_from_user(data, buf, size)) {
			status = -EACCES;
			goto out;
		}
	}
	if (encap->iface_num < encap->udev->actconfig->desc.bNumInterfaces)
		intf = encap->udev->actconfig->interface[encap->iface_num];
	if (NULL != intf)
		usb_autopm_get_interface(intf);
	printk(KERN_ERR"fxz-%s: wriet[%s]\n", __func__, (char *)data);
	status = usb_control_msg(encap->udev, usb_sndctrlpipe(encap->udev, 0),
		CDC_ENCAP_REQ_SEND_COMMAND, CDC_ENCAP_REQT_OUT, 0,
		cpu_to_le16(encap->iface_num), data, size,
		msecs_to_jiffies(CDC_ENCAP_TIMEOUT_MS));
	
out:
	if (data)
		kfree(data);
	if (NULL != intf)
		usb_autopm_put_interface(intf);
	return status;
}

static unsigned int cdc_encap_poll(struct file *filp, poll_table *wait)
{
	struct cdc_encap *encap = filp->private_data;
	unsigned int mask = POLLOUT | POLLWRNORM;

	poll_wait(filp, &encap->readq, wait);

	if (encap->rsp_ready)
		mask |= POLLIN | POLLRDNORM;

	return mask;
}

static int cdc_encap_open(struct inode *i, struct file *filp)
{
	struct cdc_encap *encap;
	printk(KERN_ERR"fxz-%s: called\n", __func__);
	encap = container_of(i->i_cdev, struct cdc_encap, cdev);
	struct usb_interface *intf = NULL;

	if (NULL == encap || NULL == encap->udev || NULL == encap->udev->actconfig) {
		return -ENODEV;
	}
	if (encap->iface_num < encap->udev->actconfig->desc.bNumInterfaces)
		intf = encap->udev->actconfig->interface[encap->iface_num];
	if (NULL != intf)
		usb_autopm_get_interface(intf);
	
	if (down_trylock(&encap->sem) != 0) {
		dev_dbg(&encap->udev->dev, "encap is already open\n");
		return -EBUSY;
	}

	filp->private_data = encap;

	dev_dbg(&encap->udev->dev, "cdc_encap_open\n");
	if (NULL != intf)
		usb_autopm_put_interface(intf);
	return 0;
}

static int cdc_encap_release(struct inode *i, struct file *filp)
{
	struct cdc_encap *encap;

	encap = container_of(i->i_cdev, struct cdc_encap, cdev);

	filp->private_data = NULL;
	up(&encap->sem);

	dev_dbg(&encap->udev->dev, "cdc_encap_release\n");
	
	return 0;
}

static struct file_operations cdc_encap_fops = {
	.owner =	THIS_MODULE,
	.read =		cdc_encap_read,
	.write =	cdc_encap_write,
	.poll =		cdc_encap_poll,
	.open =		cdc_encap_open,
	.release =	cdc_encap_release
};

int cdc_encap_init(char *dev_id, struct usb_interface *intf,
	struct cdc_encap **encap_p)
{
	if (NULL == encap)
		return -1;
	encap->udev = interface_to_usbdev(intf);
	encap->iface_num = intf->cur_altsetting->desc.bInterfaceNumber;
	*encap_p = encap;
	return 0;
}
EXPORT_SYMBOL_GPL(cdc_encap_init);

void cdc_encap_uninit(struct cdc_encap *encap)
{
}
EXPORT_SYMBOL_GPL(cdc_encap_uninit);

void cdc_encap_response_avail(struct cdc_encap *encap)
{
	dev_dbg(&encap->udev->dev, "cdc_encap_response_avail\n");

	encap->rsp_ready = 1;
	wake_up_interruptible(&encap->readq);
}
EXPORT_SYMBOL_GPL(cdc_encap_response_avail);

static int __init cdc_encap_module_init(void)
{
	int error;
	int name_len = strlen("rmnet") + sizeof(dev_name_prefix);

	encap = kmalloc(sizeof(*encap) + 1 + name_len, GFP_KERNEL);
	if (!encap) {
		printk(KERN_ERR"fxz-%s: !encap\n", __func__);
		return -ENOMEM;
	}

	memset(encap, 0, sizeof(*encap));
	sprintf(encap->name, "%s%s", dev_name_prefix, "rmnet");


	error = alloc_chrdev_region(&encap->devno, 0, 1, encap->name);
	if (error) {
		printk(KERN_ERR"fxz-%s: alloc error[%d]\n", __func__, error);
		kfree(encap);
		return error;
	}

	cdev_init(&encap->cdev, &cdc_encap_fops);
	encap->cdev.owner = THIS_MODULE;
	encap->cdev.ops = &cdc_encap_fops;

	error = cdev_add(&encap->cdev, encap->devno, 1);
	if (error) {
		printk(KERN_ERR"fxz-%s: add error[%d]\n", __func__, error);
		unregister_chrdev_region(encap->devno, 1);
		kfree(encap);
		return error;
	}

	init_MUTEX(&encap->sem);
	init_waitqueue_head(&encap->readq);

	/**/
	hw_class = class_create(THIS_MODULE, "huawei_ecm");
	hw_device = device_create(hw_class, NULL ,encap->devno, NULL, encap->name);
	(void)class_create_file(hw_class , &class_attr_classfile);
	(void)device_create_file(hw_device ,&dev_attr_devfile);
	/**/
 	return 0;
}
module_init(cdc_encap_module_init);

static void __exit cdc_encap_exit(void)
{
	if (NULL == encap)
		return;
	cdev_del(&encap->cdev);
	unregister_chrdev_region(encap->devno, 1);

	class_remove_file(hw_class, &class_attr_classfile);
	device_remove_file(hw_device,&dev_attr_devfile);
	device_destroy(hw_class,MKDEV(encap->devno ,0));
	class_destroy(hw_class);

	kfree(encap);
}
module_exit(cdc_encap_exit);

MODULE_AUTHOR("Franko Fang <huananhu@huawei.com>");
MODULE_DESCRIPTION("USB CDC encapsulated command");
MODULE_LICENSE("GPL");
