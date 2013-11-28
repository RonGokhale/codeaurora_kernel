/* Copyright (c) 2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/wakelock.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/pm_wakeup.h>
#include <linux/clk.h>
#include <linux/pm_qos.h>

#include <linux/mpq_mcu_gpio_comm.h>
#include <mach/mpq_mcu_comm_pdata.h>

#define PIN_HIGH 1
#define PIN_LOW  0

static int mcu_timeout = 200;

struct mcu_dev_data {
	struct device *dev;
	struct device *dev_node;
	struct class  *dev_class;
	dev_t         base_dev;
	struct cdev   mcu_cdev;
	struct mpq_mcu_comm_platform_data *pdata;
	struct wake_lock mcu_wakelock;
	int    usage_count;
	unsigned int wakeup_irq;
	struct pm_qos_request pm_qos_req;
	struct timer_list mcu_timer;
	bool pm_qos_vote;
	int irq_latency;
};

struct mcu_dev_data *mcu_data;

static void mcu_comm_timer(unsigned long data)
{
	struct mcu_dev_data *dev_data = (struct mcu_dev_data *)data;

	pm_qos_update_request(&dev_data->pm_qos_req, PM_QOS_DEFAULT_VALUE);
	pm_qos_request_active(&dev_data->pm_qos_req);
	dev_data->pm_qos_vote = 0;
}


static irqreturn_t mpq_wakeup_irq(int irq, void *dev_id)
{
	struct mcu_dev_data *dev_data = (struct mcu_dev_data *)dev_id;

	dev_data->pm_qos_vote = 1;
	pm_qos_update_request(&dev_data->pm_qos_req, dev_data->irq_latency);

	mod_timer(&dev_data->mcu_timer,
			jiffies + msecs_to_jiffies(mcu_timeout));

	return IRQ_HANDLED;
}

static int mpq_mcu_dev_suspend(struct device *dev)
{

	struct platform_device *pdev = to_platform_device(dev);
	struct mcu_dev_data *dev_data = platform_get_drvdata(pdev);

	gpio_direction_output(dev_data->pdata->wakeup_gpio, PIN_LOW);
	udelay(10);
	gpio_direction_output(dev_data->pdata->wakeup_gpio, PIN_HIGH);
	udelay(20);
	gpio_direction_output(dev_data->pdata->status_gpio, PIN_HIGH);
	udelay(10);
	gpio_direction_output(dev_data->pdata->status_gpio, PIN_LOW);

	gpio_direction_input(dev_data->pdata->wakeup_gpio);

	if (device_may_wakeup(dev))
		enable_irq_wake(dev_data->wakeup_irq);

	enable_irq(dev_data->wakeup_irq);

	return 0;
}

static int mpq_mcu_dev_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct mcu_dev_data *dev_data = platform_get_drvdata(pdev);

	disable_irq(dev_data->wakeup_irq);

	if (device_may_wakeup(dev))
		disable_irq_wake(dev_data->wakeup_irq);

	gpio_direction_output(dev_data->pdata->wakeup_gpio, PIN_LOW);
	udelay(10);
	gpio_direction_output(dev_data->pdata->wakeup_gpio, PIN_HIGH);
	udelay(20);
	gpio_direction_output(dev_data->pdata->status_gpio, PIN_HIGH);
	udelay(10);
	gpio_direction_output(dev_data->pdata->status_gpio, PIN_LOW);

	return 0;
}

void mpq_mcu_dev_power_off(void)
{
	if (mcu_data == NULL)
		return;

	gpio_direction_output(mcu_data->pdata->wakeup_gpio, PIN_LOW);
	udelay(10);
	gpio_direction_output(mcu_data->pdata->wakeup_gpio, PIN_HIGH);
	udelay(10);
	gpio_direction_output(mcu_data->pdata->wakeup_gpio, PIN_LOW);
	udelay(10);
	gpio_direction_output(mcu_data->pdata->wakeup_gpio, PIN_HIGH);
	udelay(20);
	gpio_direction_output(mcu_data->pdata->status_gpio, PIN_HIGH);
	udelay(10);
	gpio_direction_output(mcu_data->pdata->status_gpio, PIN_LOW);
}
EXPORT_SYMBOL(mpq_mcu_dev_power_off);

static int mpq_mcu_dev_open(struct inode *inode, struct file *file)
{
	struct cdev *input_cdev = inode->i_cdev;
	struct mcu_dev_data *dev_data =
		container_of(input_cdev, struct mcu_dev_data, mcu_cdev);

	if (dev_data->usage_count) {
		dev_err(dev_data->dev,
		"Device is already open..only one instance is allowed\n");
		return -EBUSY;
	}
	dev_data->usage_count++;
	file->private_data = dev_data;

	return 0;
}

long mpq_mcu_dev_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct mcu_dev_data *dev_data = file->private_data;
	int rc = 0;

	dev_info(dev_data->dev, "mpq_mcu_dev_ioctl\n");

	switch (cmd) {
	case MPQ_SET_MCU_BOOT_DLOAD:
		rc = gpio_direction_output(dev_data->pdata->wakeup_gpio,
					PIN_LOW);
		if (rc)
			dev_err(dev_data->dev,
				"SET_MCU_BOOT_DLOAD failed with %d\n", rc);
		break;
	case MPQ_SET_MCU_BOOT_NORMAL:
		rc = gpio_direction_output(dev_data->pdata->wakeup_gpio,
					PIN_HIGH);
		if (rc)
			dev_err(dev_data->dev,
				"SET_MCU_BOOT_NORMAL failed with %d\n", rc);
		break;
	case MPQ_SET_MCU_WAKEUP_SRC:
		rc = gpio_direction_input(dev_data->pdata->wakeup_gpio);
		if (rc)
			dev_err(dev_data->dev,
				"SET_MCU_WAKEUP_SRC failed with %d\n", rc);
		break;
	default:
		rc = -EINVAL;
		dev_err(dev_data->dev, "Invalid IOCTL :%d\n", cmd);
	}

	return rc;
}

static int mpq_mcu_dev_release(struct inode *inode, struct file *file)
{
	struct mcu_dev_data *dev_data = file->private_data;
	dev_data->usage_count--;
	return 0;
}

const struct file_operations mpq_mcu_comm_fops = {
	.owner   = THIS_MODULE,
	.open    = mpq_mcu_dev_open,
	.unlocked_ioctl = mpq_mcu_dev_ioctl,
	.release = mpq_mcu_dev_release,
};

static int __devinit mpq_mcu_dev_probe(struct platform_device *pdev)
{
	struct mpq_mcu_comm_platform_data *pdata = pdev->dev.platform_data;
	int rc = 0;

	if (!pdata) {
		dev_err(&pdev->dev, "No platform data\n");
		return -EINVAL;
	}

	mcu_data = devm_kzalloc(&pdev->dev, sizeof(*mcu_data), GFP_KERNEL);
	if (!mcu_data) {
		dev_err(&pdev->dev, "Failed to allocate device data\n");
		return -ENOMEM;
	}

	mcu_data->dev_class = class_create(THIS_MODULE, "mpq_mcu_comm");
	if (IS_ERR(mcu_data->dev_class)) {
		rc = PTR_ERR(mcu_data->dev_class);
		goto err;
	}

	rc = alloc_chrdev_region(&mcu_data->base_dev, 0, 1, "mcu_dev");
	if (rc) {
		dev_err(&pdev->dev, "Failed in alloc_chrdev_region %d\n", rc);
		goto alloc_chrdev_err;
	}

	cdev_init(&mcu_data->mcu_cdev, &mpq_mcu_comm_fops);
	rc = cdev_add(&mcu_data->mcu_cdev, mcu_data->base_dev, 1);
	if (rc) {
		dev_err(&pdev->dev, "cdev_add failed with %d\n", rc);
		goto cdev_add_err;
	}

	mcu_data->dev_node = device_create(mcu_data->dev_class, NULL,
				MKDEV(MAJOR(mcu_data->base_dev), 0), NULL,
				"mpq_mcu_gpio_dev%d", 0);
	if (IS_ERR(mcu_data->dev_node)) {
		rc = PTR_ERR(mcu_data->dev_node);
		dev_err(&pdev->dev, "device_create failed with %d\n", rc);
		goto device_create_err;
	}

	/* Request Wake-up and Status GPIO */
	rc = gpio_request(pdata->wakeup_gpio, "mpq_mcu_wakeup");
	if (rc) {
		dev_err(&pdev->dev, "gpio_request for %d failed with %d\n",
			pdata->wakeup_gpio, rc);
		goto err_allocate_device;
	}

	gpio_direction_output(pdata->wakeup_gpio, PIN_HIGH);

	rc = gpio_request(pdata->status_gpio, "mpq_state");
	if (rc) {
		dev_err(&pdev->dev, "gpio_request for %d failed with %d\n",
			pdata->status_gpio, rc);
		goto gpio_cleanup_1;
	}

	gpio_direction_output(pdata->status_gpio, PIN_LOW);

	mcu_data->irq_latency = pdata->swfi_latency + 1;
	mcu_data->pm_qos_vote = 0;

	mcu_data->wakeup_irq = platform_get_irq(pdev, 0);

	rc = request_threaded_irq(mcu_data->wakeup_irq, NULL, mpq_wakeup_irq,
			IRQF_TRIGGER_FALLING, "mcu wakeup intr", mcu_data);
	if (rc) {
		dev_err(&pdev->dev, "request_threaded_irq failed with %d\n",
			rc);
		goto gpio_cleanup_2;
	}

	disable_irq(mcu_data->wakeup_irq);

	pm_qos_add_request(&mcu_data->pm_qos_req, PM_QOS_CPU_DMA_LATENCY,
			PM_QOS_DEFAULT_VALUE);
	setup_timer(&mcu_data->mcu_timer, mcu_comm_timer,
			(unsigned long)mcu_data);

	mcu_data->usage_count = 0;
	mcu_data->dev   = &pdev->dev;
	mcu_data->pdata = pdata;
	platform_set_drvdata(pdev, mcu_data);
	device_init_wakeup(&pdev->dev, 1);

	dev_info(&pdev->dev, "MPQ-MCU : Device Initialization Complete.\n");
	return 0;

gpio_cleanup_2:
	gpio_free(pdata->status_gpio);
gpio_cleanup_1:
	gpio_free(pdata->wakeup_gpio);
err_allocate_device:
	device_destroy(mcu_data->dev_class,
			MKDEV(MAJOR(mcu_data->base_dev), 0));
cdev_add_err:
	unregister_chrdev_region(mcu_data->base_dev, 1);
device_create_err:
	cdev_del(&mcu_data->mcu_cdev);
alloc_chrdev_err:
	class_destroy(mcu_data->dev_class);
err:
	kfree(mcu_data);
	return rc;

}

static int __devexit mpq_mcu_dev_remove(struct platform_device *pdev)
{
	struct mcu_dev_data *dev_data = platform_get_drvdata(pdev);

	platform_set_drvdata(pdev, NULL);
	del_timer_sync(&dev_data->mcu_timer);
	pm_qos_remove_request(&dev_data->pm_qos_req);
	free_irq(dev_data->wakeup_irq, dev_data);
	gpio_free(dev_data->pdata->wakeup_gpio);
	gpio_free(dev_data->pdata->status_gpio);
	device_destroy(dev_data->dev_class,
			MKDEV(MAJOR(dev_data->base_dev), 0));
	unregister_chrdev_region(dev_data->base_dev, 1);
	cdev_del(&dev_data->mcu_cdev);
	class_destroy(dev_data->dev_class);
	kfree(dev_data);

	return 0;
}

static const struct dev_pm_ops mpq_mcu_dev_pm_ops = {
	.suspend = mpq_mcu_dev_suspend,
	.resume  = mpq_mcu_dev_resume,
};

static struct platform_driver mpq_mcu_dev_driver = {
	.probe = mpq_mcu_dev_probe,
	.remove = __devexit_p(mpq_mcu_dev_remove),
	.driver = {
		.name = "mcu_gpio_comm",
		.owner = THIS_MODULE,
		.pm = &mpq_mcu_dev_pm_ops,
	},
};

static int __init mpq_mcu_dev_init(void)
{
	return platform_driver_register(&mpq_mcu_dev_driver);
}
module_init(mpq_mcu_dev_init);

static void __exit mpq_mcu_dev_exit(void)
{
	platform_driver_unregister(&mpq_mcu_dev_driver);
}
module_exit(mpq_mcu_dev_exit);

MODULE_DESCRIPTION("MPQ MCU GPIO Communication driver");
MODULE_LICENSE("GPL v2");
