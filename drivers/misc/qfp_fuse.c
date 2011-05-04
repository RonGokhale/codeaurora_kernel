/* Copyright (c) 2011, Code Aurora Forum. All rights reserved.
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>
#include <linux/qfp_fuse.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/regulator/consumer.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>

struct private {
	struct semaphore lock;
	struct regulator *lvs0;
	u8 state;
};

static uint32_t qfp_qfprom_base;
static uint32_t qfp_qfprom_end;

static int qfp_fuse_open(struct inode *inode, struct file *filp)
{
	struct private *priv;
	priv = filp->private_data;

	/* Check if already initialized */
	if ((priv != NULL) && (priv->state == QFP_FUSE_READY))
		return 0;

	/* Initialize */
	priv = kmalloc(sizeof(struct private), GFP_KERNEL);
	if (priv == NULL) {
		pr_alert("[%s] Not enough memory to initialize device\n",
			 __func__);
		return -ENOMEM;
	}

	/* Get LVS0 regulator for QFPROM writes */
	priv->lvs0 = regulator_get(NULL, PM8058_LVS0_REGULATOR);
	if (IS_ERR(priv->lvs0)) {
		pr_err("[%s] Error getting %s regulator\n",
		       __func__, PM8058_LVS0_REGULATOR);
		return PTR_ERR(priv->lvs0);
	}

	sema_init(&priv->lock, 1);
	priv->state = QFP_FUSE_READY;
	filp->private_data = priv;

	return 0;
}

static int qfp_fuse_release(struct inode *inode, struct file *filp)
{
	struct private *priv = filp->private_data;

	if (priv != NULL) {
		regulator_put(priv->lvs0);
		priv->lvs0 = NULL;
		kfree(priv);
	}
	filp->private_data = NULL;

	return 0;
}

static inline int qfp_fuse_wait_for_fuse_blow(u32 *status)
{
	u32 timeout = QFPROM_BLOW_TIMEOUT_US;
	do {
		*status = readl(QFPROM_BLOW_STATUS_OFFSET);
		if (!(*status & QFPROM_BLOW_STATUS_BUSY))
			return 0;

		timeout--;
		udelay(1);
	} while (timeout);
	pr_err("[%s] Timeout waiting for FUSE blow, status = %x\n",
	       __func__, *status);
	return -ETIMEDOUT;
}

static int qfp_fuse_write_word(u32 *addr, u32 data, struct regulator *lvs0)
{
	u32 blow_status = 0;
	u32 read_data;
	int err;

	/* Set QFPROM  blow timer register */
	writel(QFPROM_BLOW_TIMER_VALUE, QFPROM_BLOW_TIMER_OFFSET);

	/* Enable LVS0 regulator */
	err = regulator_enable(lvs0);
	if (err != 0) {
		pr_err(
		    "[%s] Error enabling PM8058 LVS0 regulator.\n", __func__);
		return -EFAULT;
	}
	/*
	 * As per documentation of PM8058, each 25mV change requires a delay
	 * of 10uSec. As such 0->1.8 and 1.8->0 would require delay of .72 mSec
	 * or ~ 1mSec.
	 */
	msleep(1);

	/* Write data */
	writel(data, addr);

	/* Disable regulator */
	err = regulator_disable(lvs0);
	if (err != 0) {
		pr_err(
		    "[%s] Error disabling PM8058 LVS0 regulator\n", __func__);
		return -EFAULT;
	}
	msleep(1);

	blow_status = QFPROM_BLOW_STATUS_BUSY;
	err = qfp_fuse_wait_for_fuse_blow(&blow_status);
	if (err)
		return err;

	/* Check error status */
	if (blow_status & QFPROM_BLOW_STATUS_ERROR) {
		pr_err("[%s] Error: fuse blow status indicates error\n",
		       __func__);
		return -EFAULT;
	}

	/* Verify written data */
	read_data = readl(addr);
	if (read_data != data) {
		pr_err("[%s] Error: read/write data mismatch\n", __func__);
		pr_err("\t address = %p written data = %x read data = %x\n",
		       addr, data, read_data);
		return -EFAULT;
	}

	return 0;
}

static int
qfp_fuse_ioctl(struct inode *inode,
	       struct file *filp, unsigned int cmd, unsigned long arg)
{
	int err = 0;
	struct qfp_fuse_req req;
	u32 *buf = NULL;
	int i;
	struct private *priv = filp->private_data;

	/* Verify user arguments. */
	if (_IOC_TYPE(cmd) != QFP_FUSE_IOC_MAGIC)
		return -ENOTTY;

	switch (cmd) {
	case QFP_FUSE_IOC_READ:
		if (arg == 0) {
			pr_err("[%s] user space arg not supplied\n",
			       __func__);
			err = -EFAULT;
			break;
		}

		if (copy_from_user(&req, (void __user *)arg, sizeof(req))) {
			pr_err("[%s] Error copying req from user space\n",
			       __func__);
			err = -EFAULT;
			break;
		}

		/* Check for limits */
		if (!req.size) {
			pr_err("[%s] Request size zero.\n", __func__);
			err = -EFAULT;
			break;
		}
		if (qfp_qfprom_base + req.offset + (req.size) * 4 >
			    qfp_qfprom_end) {
			pr_err(
			"[%s] Requested size exceeds QFPROM addresss space.\n"
			, __func__);
			err = -EFAULT;
			break;
		}

		/* Allocate memory for buffer */
		buf = kzalloc(req.size * 4, GFP_KERNEL);
		if (buf == NULL) {
			pr_alert("[%s] No memory for data\n", __func__);
			err = -ENOMEM;
			break;
		}

		/* Lock for writing */
		if (down_interruptible(&priv->lock)) {
			err = -ERESTARTSYS;
			break;
		}

		/* Read data */
		for (i = 0; i < req.size; i++)
			buf[i] =
			    readl(((u32 *) (qfp_qfprom_base + req.offset)) + i);

		/* Print user data for debugging */
		pr_debug("[%s] Read data\n", __func__);
		for (i = 0; i < req.size; i++)
			pr_debug("%x", buf[i]);

		if (copy_to_user((void __user *)req.data, buf, 4*(req.size))) {
			pr_err("[%s] Error copying to user space\n", __func__);
			err = -EFAULT;
		}
		up(&priv->lock);
		break;

	case QFP_FUSE_IOC_WRITE:
		if (arg == 0) {
			pr_err("[%s] user space arg not supplied\n",
			       __func__);
			err = -EFAULT;
			break;
		}

		if (copy_from_user(&req, (void __user *)arg, sizeof(req))) {
			pr_err("[%s] Error copying req from user space\n",
			       __func__);
			err = -EFAULT;
			break;
		}
		/* Check for limits */
		if (!req.size) {
			pr_err("[%s] Request size zero.\n", __func__);
			err = -EFAULT;
			break;
		}
		if (qfp_qfprom_base + req.offset + (req.size) * 4 >
			qfp_qfprom_end) {
			pr_err(
			"[%s] Requested size exceeds QFPROM addresss space.\n",
			    __func__);
			err = -EFAULT;
			break;
		}

		/* Allocate memory for buffer */
		buf = kzalloc(4 * (req.size), GFP_KERNEL);
		if (buf == NULL) {
			pr_alert("[%s] No memory for data\n", __func__);
			err = -ENOMEM;
			break;
		}

		/* Copy user data to local buffer */
		if (copy_from_user(buf, (void __user *)req.data,
				   4 * (req.size))) {
			pr_err("[%s] Error copying data from user space\n",
			       __func__);
			err = -EFAULT;
			break;
		}

		/* Print user data for debugging */
		pr_debug("[%s] Data to be written\n", __func__);
		for (i = 0; i < req.size; i++)
			pr_debug("%x", buf[i]);

		/* Lock for writing */
		if (down_interruptible(&priv->lock)) {
			err = -ERESTARTSYS;
			break;
		}

		/* Write data word at a time */
		for (i = 0; i < req.size && !err; i++) {
			err = qfp_fuse_write_word(((u32 *) (qfp_qfprom_base +
							    req.offset) + i),
						  buf[i], priv->lvs0);
		}

		if (!err) {
			pr_debug(
			    "[%s] Successfully written %d words to QFPROM.\n",
			    __func__, req.size);
		}
		up(&priv->lock);
		break;
	default:
		pr_err("[%s] Invalid ioctl command.\n", __func__);
		return -ENOTTY;
	}
	kfree(buf);
	return err;
}

static const struct file_operations qfp_fuse_fops = {
	.owner = THIS_MODULE,
	.ioctl = qfp_fuse_ioctl,
	.open = qfp_fuse_open,
	.release = qfp_fuse_release
};

static struct miscdevice qfp_fuse_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "qfpfuse",
	.fops = &qfp_fuse_fops
};


static int qfp_fuse_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct resource *res;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	qfp_qfprom_base = res->start;
	qfp_qfprom_end = res->end;

	ret = misc_register(&qfp_fuse_dev);
	if (ret < 0)
		return ret;

	pr_info("[%s] base %x end %x\n", __func__, qfp_qfprom_base,
		qfp_qfprom_end);
	return 0;
}

static int __devexit qfp_fuse_remove(struct platform_device *plat)
{
	misc_deregister(&qfp_fuse_dev);
	pr_info("[%s]\n", __func__);
	return 0;
}

static struct platform_driver qfp_fuse_driver = {
    .probe = qfp_fuse_probe,
    .remove = qfp_fuse_remove,
    .driver = {
	.name = "qfp_fuse_driver",
	.owner = THIS_MODULE,
    },
};

static int __init qfp_fuse_init(void)
{
	return platform_driver_register(&qfp_fuse_driver);
}


static void __exit qfp_fuse_exit(void)
{
	platform_driver_unregister(&qfp_fuse_driver);
}

module_init(qfp_fuse_init);
module_exit(qfp_fuse_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Rohit Vaswani <rvaswani@codeaurora.org>");
MODULE_DESCRIPTION("Driver to read/write to QFPROM fuses.");
MODULE_VERSION("1.01");
