/* Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 */

/* Uncomment this line to enable debug log messages
 * #define DEBUG
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/mfd/pmic8058.h>
#include <linux/regulator/consumer.h>
#include <mach/gpio.h>

#include "pmic8058-femto.h"
#include "tlmm-fsm9xxx.h"

struct private {
	struct mutex lock;
	u8 state;
};

static int pm8058_femto_ioctl(struct inode *, struct file *,
	unsigned int cmd, unsigned long arg);
static int pm8058_femto_open(struct inode *, struct file *);
static int pm8058_femto_release(struct inode *, struct file *);

static struct pm8058_chip *pm_chip;

static int pm8058_femto_open(struct inode *inode, struct file *filp)
{
	struct private *priv;
	priv = filp->private_data;

	/* Check if already initialized */
	if ((priv != NULL) && (priv->state == PMIC_DEVICE_READY))
		return 0;

	/* Initialize */
	priv = kmalloc(sizeof(struct private), GFP_KERNEL);
	if (priv == NULL) {
		pr_alert("[%s] Not enough memory to initialize device\n",
			 __func__);
		return -ENOMEM;
	}

	mutex_init(&priv->lock);
	priv->state = PMIC_DEVICE_READY;
	filp->private_data = priv;

	return 0;
}

static int pm8058_femto_release(struct inode *inode, struct file *filp)
{
	struct private *priv = filp->private_data;

	pr_info("[%s]\n", __func__);
	if (priv != NULL)
		kfree(priv);
	filp->private_data = NULL;

	return 0;
}

static inline int pm8058_femto_write(u16 addr, u8 value)
{
	u8 data = value;
	int err = 0;
	if (pm8058_write(pm_chip, addr, &data, 1)) {
		pr_err("[%s] Error writing to PMIC\n", __func__);
		err = -EFAULT;
	}
	return err;
}

static int pm8058_femto_read_adc(u16 *adc_value)
{
	int err = 0;
	u8 adc_raw[2];

	/* Allow mux selection to settle */
	msleep(16);

	/* Initiate conversion */
	err = pm8058_femto_write(0x192, 0xff);
	if (err)
		return err;

	err = pm8058_femto_write(0x190, 0x83);
	if (err)
		return err;

	msleep(16);

	if (pm8058_read(pm_chip, 0x195, &adc_raw[0], 1)) {
			pr_err("[%s] Error reading from PMIC\n", __func__);
			err = -EFAULT;
			return err;
	}
	if (pm8058_read(pm_chip, 0x196, &adc_raw[1], 1)) {
			pr_err("[%s] Error reading from PMIC\n", __func__);
			err = -EFAULT;
			return err;
	}

	/* MSB is the one read from address 0x195 */
	*adc_value = adc_raw[0] << 8 | adc_raw[1];

	/* Disable arbiter, and ADC modulator */
	err = pm8058_femto_write(0x190, 0x00);
	if (err)
		return err;

	err = pm8058_femto_write(0x190, 0x00);
	if (err)
		return err;

	err = pm8058_femto_write(0x191, 0xff);
	if (err)
		return err;

	err = pm8058_femto_write(0x192, 0xfe);
	if (err)
		return err;

	return err;
}

static int
pm8058_femto_enable_arbiter(void)
{
	int err = 0;

	/* Set up the secure processor arbiter for control of adc/mux */
	err = pm8058_femto_write(0x190, 0x42);
	if (err)
		return err;

	msleep(100);

	/* Enable arbiter, and set conversion time to 3ms default */
	err = pm8058_femto_write(0x190, 0x73);
	if (err)
		return err;

	err = pm8058_femto_write(0x190, 0x73);
	if (err)
		return err;

	err = pm8058_femto_write(0x18f, 0x00);
	if (err)
		return err;

	msleep(16);

	return err;
}

static int
pm8058_femto_ioctl(struct inode *inode,
	struct file *filp, unsigned int cmd, unsigned long arg)
{
	int err = 0;
	u16 adc_user = 0;
	struct pm8058_femto_req req;
	struct private *priv = filp->private_data;

	/* Verify user arguments. */
	if (_IOC_TYPE(cmd) != PM8058_FEMTO_IOC_MAGIC)
		return -ENOTTY;

	/* Lock for access */
	if (mutex_lock_interruptible(&priv->lock)) {
		return -ERESTARTSYS;
	}

	switch (cmd) {
	case PM8058_FEMTO_IOC_1V25:
		pr_debug("[%s] Read 1.25V ADC command issued.\n",
			 __func__);

		if (copy_from_user(&req, (void __user *)arg,
			sizeof(req))) {
			pr_err("[%s] Error copying from user space\n",
				__func__);
			err = -EFAULT;
			break;
		}

		if (pm8058_femto_enable_arbiter()) {
			pr_err("[%s] Error enabling PMIC arbiter\n",
				__func__);
			err = -EFAULT;
			break;
		}

		/* Select 1.25V input and reset filters */
		err = pm8058_femto_write(0x191, 0xd0);
		if (err)
			break;

		err = pm8058_femto_write(0x194, 0x20);
		if (err)
			break;

		err = pm8058_femto_write(0x194, 0x20);
		if (err)
			break;

		err = pm8058_femto_write(0x194, 0x21);
		if (err)
			break;

		err = pm8058_femto_write(0x194, 0x20);
		if (err)
			break;

		/* Read data */
		if (pm8058_femto_read_adc(&adc_user)) {
			pr_err("[%s] Error reading PMIC ADC\n",
				__func__);
			err = -EFAULT;
			break;
		}

		/* Print user data for debugging */
		pr_debug("[%s] Read data = %x\n", __func__, adc_user);

		if (copy_to_user((void __user *)req.data,
				&adc_user, sizeof(adc_user))) {
			pr_err("[%s] Error copying to user space\n",
				__func__);
			err = -EFAULT;
		}

		break;
	case PM8058_FEMTO_IOC_DIE_TEMP:
		pr_debug("[%s] Read Die Temp ADC command issued.\n",
			 __func__);

		if (copy_from_user(&req, (void __user *)arg,
			sizeof(req))) {
			pr_err("[%s] Error copying from user space\n",
				__func__);
			err = -EFAULT;
			break;
		}

		if (pm8058_femto_enable_arbiter()) {
			pr_err("[%s] Error enabling PMIC arbiter\n",
				__func__);
			err = -EFAULT;
			break;
		}

		/* Select Die Temp input and reset filters */
		err = pm8058_femto_write(0x191, 0xb0);
		if (err)
			break;

		err = pm8058_femto_write(0x194, 0x20);
		if (err)
			break;

		err = pm8058_femto_write(0x194, 0x20);
		if (err)
			break;

		err = pm8058_femto_write(0x194, 0x21);
		if (err)
			break;

		err = pm8058_femto_write(0x194, 0x20);
		if (err)
			break;

		/* Read data */
		if (pm8058_femto_read_adc(&adc_user)) {
			pr_err("[%s] Error reading PMIC ADC\n",
				__func__);
			err = -EFAULT;
			break;
		}

		/* Print user data for debugging */
		pr_debug("[%s] Read data = %x\n", __func__, adc_user);

		if (copy_to_user((void __user *)req.data,
				&adc_user, sizeof(adc_user))) {
			pr_err("[%s] Error copying to user space\n",
				__func__);
			err = -EFAULT;
		}

		break;
	case PM8058_FEMTO_IOC_THERM_PA:
		pr_debug("[%s] Read Therm PA ADC command issued.\n",
			 __func__);

		if (copy_from_user(&req, (void __user *)arg,
			sizeof(req))) {
			pr_err("[%s] Error copying from user space\n",
				__func__);
			err = -EFAULT;
			break;
		}

		if (pm8058_femto_enable_arbiter()) {
			pr_err("[%s] Error enabling PMIC arbiter\n",
				__func__);
			err = -EFAULT;
			break;
		}

		/* Select Therm PA input and reset filters */
		err = pm8058_femto_write(0x191, 0x60);
		if (err)
			break;

		err = pm8058_femto_write(0x194, 0x20);
		if (err)
			break;

		err = pm8058_femto_write(0x194, 0x20);
		if (err)
			break;

		err = pm8058_femto_write(0x194, 0x21);
		if (err)
			break;

		err = pm8058_femto_write(0x194, 0x20);
		if (err)
			break;

		/* Read data */
		if (pm8058_femto_read_adc(&adc_user)) {
			pr_err("[%s] Error reading PMIC ADC\n",
				__func__);
			err = -EFAULT;
			break;
		}

		/* Print user data for debugging */
		pr_debug("[%s] Read data = %x\n", __func__, adc_user);

		if (copy_to_user((void __user *)req.data,
				&adc_user, sizeof(adc_user))) {
			pr_err("[%s] Error copying to user space\n",
				__func__);
			err = -EFAULT;
		}

		break;
	case PM8058_FEMTO_IOC_THERM_FSM:
		pr_debug("[%s] Read Therm FSM ADC command issued.\n",
			 __func__);

		if (copy_from_user(&req, (void __user *)arg,
			sizeof(req))) {
			pr_err("[%s] Error copying from user space\n",
				__func__);
			err = -EFAULT;
			break;
		}

		if (pm8058_femto_enable_arbiter()) {
			pr_err("[%s] Error enabling PMIC arbiter\n",
				__func__);
			err = -EFAULT;
			break;
		}

		/* Select Therm FSM input and reset filters */
		err = pm8058_femto_write(0x191, 0x50);
		if (err)
			break;

		err = pm8058_femto_write(0x194, 0x20);
		if (err)
			break;

		err = pm8058_femto_write(0x194, 0x20);
		if (err)
			break;

		err = pm8058_femto_write(0x194, 0x21);
		if (err)
			break;

		err = pm8058_femto_write(0x194, 0x20);
		if (err)
			break;

		/* Read data */
		if (pm8058_femto_read_adc(&adc_user)) {
			pr_err("[%s] Error reading PMIC ADC\n",
				__func__);
			err = -EFAULT;
			break;
		}

		/* Print user data for debugging */
		pr_debug("[%s] Read data = %x\n", __func__, adc_user);

		if (copy_to_user((void __user *)req.data,
				&adc_user, sizeof(adc_user))) {
			pr_err("[%s] Error copying to user space\n",
				__func__);
			err = -EFAULT;
		}

		break;
	case PM8058_FEMTO_IOC_THERM_VCTCXO:
		pr_debug("[%s] Read Therm VCTCXO ADC command issued.\n",
			 __func__);

		if (copy_from_user(&req, (void __user *)arg,
			sizeof(req))) {
			pr_err("[%s] Error copying from user space\n",
				__func__);
			err = -EFAULT;
			break;
		}

		if (pm8058_femto_enable_arbiter()) {
			pr_err("[%s] Error enabling PMIC arbiter\n",
				__func__);
			err = -EFAULT;
			break;
		}

		/* Select Therm VCTCXO input and reset filters */
		err = pm8058_femto_write(0x191, 0xff);
		if (err)
			break;

		err = pm8058_femto_write(0x194, 0x10);
		if (err)
			break;

		err = pm8058_femto_write(0x193, 0x13);
		if (err)
			break;

		err = pm8058_femto_write(0x194, 0x10);
		if (err)
			break;

		err = pm8058_femto_write(0x194, 0x11);
		if (err)
			break;

		err = pm8058_femto_write(0x194, 0x10);
		if (err)
			break;

		/* Read data */
		if (pm8058_femto_read_adc(&adc_user)) {
			pr_err("[%s] Error reading PMIC ADC\n",
				__func__);
			err = -EFAULT;
			break;
		}

		/* Print user data for debugging */
		pr_debug("[%s] Read data = %x\n", __func__, adc_user);

		if (copy_to_user((void __user *)req.data,
				&adc_user, sizeof(adc_user))) {
			pr_err("[%s] Error copying to user space\n",
				__func__);
			err = -EFAULT;
		}

		break;
	case PM8058_FEMTO_IOC_CLKBUF:
		pr_debug("[%s] CLKBUF control command issued.\n",
			 __func__);

		if (copy_from_user(&req, (void __user *)arg,
			sizeof(req))) {
			pr_err("[%s] Error copying from user space\n",
				__func__);
			err = -EFAULT;
			break;
		}

		if (req.clkBuffer == XO_BUFFER_A0) {
			if (req.clkBufEnable)
				err = pm8058_femto_write(0x185, 0x04);
			else
				err = pm8058_femto_write(0x185, 0x08);
		} else if (req.clkBuffer == XO_BUFFER_A1) {
			if (req.clkBufEnable)
				err = pm8058_femto_write(0x186, 0x04);
			else
				err = pm8058_femto_write(0x186, 0x08);
		} else {
			pr_err("[%s] Invalid ioctl argument.\n", __func__);
			err = -ENOTTY;
		}
		break;
	case PM8058_FEMTO_IOC_BOARD_RESET:
		if (gpio_tlmm_config(GPIO_CFG(FEMTO_GPIO_PS_HOLD, 0,
				GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
				GPIO_CFG_2MA), GPIO_CFG_ENABLE))
			pr_err("%s: gpio_tlmm_config (gpio=%d) failed\n",
				__func__, FEMTO_GPIO_PS_HOLD);
		gpio_set_value(FEMTO_GPIO_PS_HOLD, 0);
		break;
	default:
		pr_err("[%s] Invalid ioctl command.\n", __func__);
		err = -ENOTTY;
		break;
	}

	mutex_unlock(&priv->lock);
	return err;
}

static const struct file_operations pm8058_femto_fops = {
	.owner = THIS_MODULE,
	.ioctl = pm8058_femto_ioctl,
	.open = pm8058_femto_open,
	.release = pm8058_femto_release
};

static struct miscdevice pm8058_femto_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "pmic",
	.fops = &pm8058_femto_fops
};

static int __devinit pm8058_femto_probe(struct platform_device *pdev)
{
	pm_chip = dev_get_drvdata(&pdev->dev);
	misc_register(&pm8058_femto_dev);
	return 0;
}

static int __devexit pm8058_femto_remove(struct platform_device *pdev)
{
	misc_deregister(&pm8058_femto_dev);
	return 0;
}

static struct platform_driver pm8058_femto_driver = {
	.probe          = pm8058_femto_probe,
	.remove         = __devexit_p(pm8058_femto_remove),
	.driver         = {
		.name = "pm8058-femto",
	}
};

static int __init pm8058_femto_init(void)
{
       return platform_driver_register(&pm8058_femto_driver);
}

static void __exit pm8058_femto_exit(void)
{
	platform_driver_unregister(&pm8058_femto_driver);
}

module_init(pm8058_femto_init);
module_exit(pm8058_femto_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Rohit Vaswani <rvaswani@codeaurora.org>");
MODULE_DESCRIPTION("Driver to initialize PM8058 Device and read ADC inputs.");
MODULE_VERSION("1.00");
