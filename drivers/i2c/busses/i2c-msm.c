/* drivers/i2c/busses/i2c-msm.c
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

#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/delay.h>

#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/hardware/ioc.h>
#include <asm/system.h>

#include <asm/arch/msm_iomap.h>

enum {
	I2C_WRITE_DATA          = 0x00,
	I2C_CLK_CTL             = 0x04,
	I2C_STATUS              = 0x08,
	I2C_READ_DATA           = 0x0c,
	I2C_INTERFACE_SELECT    = 0x10,

	I2C_WRITE_DATA_DATA_BYTE            = 0xff,
	I2C_WRITE_DATA_ADDR_BYTE            = 1U << 8,
	I2C_WRITE_DATA_LAST_BYTE            = 1U << 9,

	I2C_CLK_CTL_FS_DIVIDER_VALUE        = 0xff,
	I2C_CLK_CTL_HS_DIVIDER_VALUE        = 7U << 8,

	I2C_STATUS_WR_BUFFER_FULL           = 1U << 0,
	I2C_STATUS_RD_BUFFER_FULL           = 1U << 1,
	I2C_STATUS_BUS_ERROR                = 1U << 2,
	I2C_STATUS_PACKET_NACKED            = 1U << 3,
	I2C_STATUS_ARB_LOST                 = 1U << 4,
	I2C_STATUS_INVALID_WRITE            = 1U << 5,
	I2C_STATUS_FAILED                   = 3U << 6,
	I2C_STATUS_BUS_ACTIVE               = 1U << 8,
	I2C_STATUS_BUS_MASTER               = 1U << 9,
	I2C_STATUS_ERROR_MASK               = 0xfc,

	I2C_INTERFACE_SELECT_INTF_SELECT    = 1U << 0,
	I2C_INTERFACE_SELECT_SCL            = 1U << 8,
	I2C_INTERFACE_SELECT_SDA            = 1U << 9,
};

struct msm_i2c_dev {
	struct device      *dev;
	void __iomem       *base;		/* virtual */
	int                 irq;
	uint32_t            last_status;
	uint32_t            error;
	struct i2c_adapter  adapter;
	wait_queue_head_t   wait;
	spinlock_t          lock;
};

static void msm_i2c_print_error(const char *prefix, uint32_t error, uint32_t status)
{
	printk(KERN_ERR "%s %x%s%s%s%s, status %x\n", prefix, error,
		(error & I2C_STATUS_BUS_ERROR) ? ",bus error" : "",
		(error & I2C_STATUS_PACKET_NACKED) ? ",nak" : "",
		(error & I2C_STATUS_ARB_LOST) ? ",arb lost" : "",
		(error & I2C_STATUS_INVALID_WRITE) ? ",invalid write" : "",
		status);
}

static uint32_t msm_i2c_read_status(struct msm_i2c_dev *dev)
{
	unsigned long irq_flags;
	uint32_t status;

	spin_lock_irqsave(&dev->lock, irq_flags);
	status = readl(dev->base + I2C_STATUS);
	if (status != dev->last_status) {
		uint32_t new_error = status & I2C_STATUS_ERROR_MASK;
		if (new_error) {
			msm_i2c_print_error("got error", new_error, status);
			dev->error |= new_error;
		}
		/* printk("status changed %x -> %x\n", dev->last_status, status); */
		dev->last_status = status;
	}
	spin_unlock_irqrestore(&dev->lock, irq_flags);
	return status | dev->error;
}

static uint32_t msm_i2c_get_error(struct msm_i2c_dev *dev)
{
	unsigned long irq_flags;
	uint32_t error;
	uint32_t status;

	status = msm_i2c_read_status(dev);

	spin_lock_irqsave(&dev->lock, irq_flags);
	error = dev->error;
	dev->error = 0;
	if (error)
		msm_i2c_print_error("report error", error, status);
	spin_unlock_irqrestore(&dev->lock, irq_flags);
	return error;
}

static irqreturn_t
msm_i2c_interrupt(int irq, void *devid)
{
	struct msm_i2c_dev *dev = devid;
	uint32_t status;
	status = msm_i2c_read_status(dev);
	/* printk("msm_i2c_interrupt status %x\n", status); */
	wake_up(&dev->wait);
	return IRQ_HANDLED;
}

static int
msm_i2c_poll_status(struct msm_i2c_dev *dev, uint32_t mask, uint32_t value)
{
	uint32_t status;
	int timeout = jiffies + HZ;
	do {
		status = msm_i2c_read_status(dev);
		if ((status & mask) == value)
			return 0;
	} while ((int)(timeout - jiffies) >= 0);
	dev_err(dev->dev, "poll status %x = %x failed, status - %x\n", mask, value, status);
	return -ETIMEDOUT;
}

static int
msm_i2c_wait_status(struct msm_i2c_dev *dev, uint32_t mask, uint32_t invert)
{
	uint32_t status;
	uint32_t status2;
	status = msm_i2c_read_status(dev);
	if (wait_event_timeout(dev->wait, (status ^ invert) & mask, HZ))
		return 0;
	status2 = msm_i2c_read_status(dev);
	dev_err(dev->dev, "wait status %x ^%x failed, status - %x %x\n", mask, invert, status, status2);
	return -ETIMEDOUT;
}

static int
msm_i2c_read(struct msm_i2c_dev *dev, uint8_t *buf, int count)
{
	int ret = -EINVAL;
	uint32_t status;
	int write_last_done = 0;
	while (count--) {
		ret = msm_i2c_wait_status(dev, I2C_STATUS_RD_BUFFER_FULL | I2C_STATUS_ERROR_MASK, 0);
		if (ret) {
			dev_err(dev->dev, "read timeout, rem %d\n", count);
			break;
		}
		if (write_last_done)
			write_last_done = 2;
		status = msm_i2c_get_error(dev);
		if (status) {
			write_last_done = 1;
			dev_err(dev->dev, "read failed, status - %x, rem %d\n", status, count);
			ret = -EIO;
			break;
		}
		if (count <= 1 && !write_last_done) {
			/* printk("msm_i2c_read write data %x\n", I2C_WRITE_DATA_LAST_BYTE); */
			writel(I2C_WRITE_DATA_LAST_BYTE, dev->base + I2C_WRITE_DATA);
			write_last_done = 1;
		}
		*buf++ = readl(dev->base + I2C_READ_DATA);
		/* printk("msm_i2c_read data %x\n", buf[-1]); */
	}
	if (write_last_done == 1) {
		ret = msm_i2c_wait_status(dev, I2C_STATUS_RD_BUFFER_FULL | I2C_STATUS_ERROR_MASK, 0);
		if (ret)
			dev_err(dev->dev, "read timeout on dummy byte, rem %d\n", count);
		readl(dev->base + I2C_READ_DATA); /* clear buffer */
	}
	count = 3;
	while (count--) {
		status = msm_i2c_read_status(dev);
		if (!(status & I2C_STATUS_RD_BUFFER_FULL))
			break;

		dev_err(dev->dev, "read did not stop, status - %x\n", status);
		writel(I2C_WRITE_DATA_LAST_BYTE, dev->base + I2C_WRITE_DATA);
		readl(dev->base + I2C_READ_DATA); /* clear buffer */
	}
	readl(dev->base + I2C_READ_DATA); /* clear buffer */
	return ret;
}

static int
msm_i2c_write(struct msm_i2c_dev *dev, const uint8_t *buf, int count, int last)
{
	int ret = 0;
	uint32_t status;
	while (count--) {
		uint16_t data = *buf++;
		if (count == 0 && last)
			data |= I2C_WRITE_DATA_LAST_BYTE;
		ret = msm_i2c_wait_status(dev, I2C_STATUS_WR_BUFFER_FULL, I2C_STATUS_WR_BUFFER_FULL);
		if (ret)
			return ret;
		status = msm_i2c_get_error(dev);
		if (status) {
			dev_err(dev->dev, "write failed, status - %x\n", status);
			ret = -EIO;
			break;
		}
		/* printk("msm_i2c_write data %x\n", data); */
		writel(data, dev->base + I2C_WRITE_DATA);
		ret = msm_i2c_wait_status(dev, I2C_STATUS_WR_BUFFER_FULL, I2C_STATUS_WR_BUFFER_FULL);
		if (ret)
			return ret;
	}
	return ret;
}

static int
msm_i2c_get_bus(struct msm_i2c_dev *dev)
{
	int ret;
	int count;
	uint32_t status;

	ret = msm_i2c_poll_status(dev, I2C_STATUS_BUS_ACTIVE, 0);
	if (ret == 0)
		return 0;
	status = msm_i2c_read_status(dev);
	dev_err(dev->dev, "wait for bus not active failed, status=%x, try to clear\n", status);
	count = 3;
	while (count--) {
		status = msm_i2c_read_status(dev);
		if (!(status & I2C_STATUS_RD_BUFFER_FULL))
			break;

		dev_err(dev->dev, "read did not stop, status - %x\n", status);
		writel(I2C_WRITE_DATA_LAST_BYTE, dev->base + I2C_WRITE_DATA);
		readl(dev->base + I2C_READ_DATA); /* clear buffer */
	}
	ret = msm_i2c_poll_status(dev, I2C_STATUS_BUS_ACTIVE, 0);
	if (ret == 0)
		return 0;
	dev_err(dev->dev, "wait for bus not active failed\n");
	return ret;
}

static int
msm_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg msgs[], int num)
{
	struct msm_i2c_dev *dev = i2c_get_adapdata(adap);
	int ret;
	int rem = num;
	uint16_t addr;
	uint32_t status;
	/* status = msm_i2c_read_status(dev); */
	/* printk("msm_i2c_xfer %p %p %d\n", adap, msgs, num); */
	/* printk("msm_i2c_xfer status %x\n", status); */

	while (rem) {
		addr = msgs->addr << 1;
		if (msgs->flags & I2C_M_RD)
			addr |= 1;
		if (rem == num) {
			ret = msm_i2c_get_bus(dev);
			if (ret) {
				dev_err(dev->dev, "msm_i2c_get_bus failed\n");
				return ret;
			}
			ret = msm_i2c_get_error(dev); /* clear old error */
			if (ret)
				dev_err(dev->dev, "cleared old error\n");
		}
		if (rem == 1 && msgs->len == 0)
			addr |= I2C_WRITE_DATA_LAST_BYTE;
		ret = msm_i2c_wait_status(dev, I2C_STATUS_WR_BUFFER_FULL, I2C_STATUS_WR_BUFFER_FULL);
		if (ret)
			return ret;
		/* printk("msm_i2c_xfer addr %x\n", I2C_WRITE_DATA_ADDR_BYTE | addr); */
		writel(I2C_WRITE_DATA_ADDR_BYTE | addr, dev->base + I2C_WRITE_DATA);

		ret = msm_i2c_wait_status(dev, I2C_STATUS_WR_BUFFER_FULL, I2C_STATUS_WR_BUFFER_FULL);
		if (ret) {
			dev_err(dev->dev, "wait for writebuffer not full failed\n");
			return ret;
		}
		status = msm_i2c_get_error(dev);
		/* printk("msm_i2c_xfer addr %x sent, error %x\n", addr, status); */
		if (status) {
			dev_err(dev->dev, "addr failed, status - %x\n", status);
			return -EIO;
		}

		if (msgs->flags & I2C_M_RD)
			ret = msm_i2c_read(dev, msgs->buf, msgs->len);
		else
			ret = msm_i2c_write(dev, msgs->buf, msgs->len, rem == 1);
		if (ret)
			return ret;
		if (rem == 1) {
			ret = msm_i2c_get_bus(dev);
			if (ret) {
				dev_err(dev->dev, "wait for bus not active failed after cmd\n");
				return ret;
			}
		}
		status = msm_i2c_get_error(dev);
		if (status) {
			dev_err(dev->dev, "cmd failed, status - %x\n", status);
			return -EIO;
		}

		msgs++;
		rem--;
	}
#if 0
	status = msm_i2c_read_status(dev);
	if (status)
		printk(KERN_INFO "msm_i2c_xfer done status1 %x\n", status);
#endif

	return num;
}

static u32
msm_i2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | (I2C_FUNC_SMBUS_EMUL & ~I2C_FUNC_SMBUS_QUICK);
}


static const struct i2c_algorithm msm_i2c_algo = {
	.master_xfer	= msm_i2c_xfer,
	.functionality	= msm_i2c_func,
};

static int
msm_i2c_probe(struct platform_device *pdev)
{
	struct msm_i2c_dev	*dev;
	struct resource		*mem, *irq, *ioarea;
	int ret;
	int fs_div;
	int hs_div;
	int i2c_clk;
	int clk_ctl;
	int target_clk;

	printk(KERN_INFO "msm_i2c_probe\n");

	/* FIXME: this needs to use the clock api once it is supported */
	writel((1U << 11) | (1U << 9), MSM_CLK_CTL_BASE + 0x68);
	mdelay(10);

	/* NOTE: driver uses the static register mapping */
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem) {
		dev_err(&pdev->dev, "no mem resource?\n");
		return -ENODEV;
	}
	irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!irq) {
		dev_err(&pdev->dev, "no irq resource?\n");
		return -ENODEV;
	}

	ioarea = request_mem_region(mem->start, (mem->end - mem->start) + 1,
			pdev->name);
	if (!ioarea) {
		dev_err(&pdev->dev, "I2C region already claimed\n");
		return -EBUSY;
	}

	dev = kzalloc(sizeof(struct msm_i2c_dev), GFP_KERNEL);
	if (!dev) {
		ret = -ENOMEM;
		goto err_alloc_dev_failed;
	}

	dev->dev = &pdev->dev;
	dev->irq = irq->start;
	dev->base = (void __iomem *)(size_t)mem->start;
	init_waitqueue_head(&dev->wait);
	spin_lock_init(&dev->lock);
	platform_set_drvdata(pdev, dev);

	/* I2C_HS_CLK = I2C_CLK/(3*(HS_DIVIDER_VALUE+1) */
	/* I2C_FS_CLK = I2C_CLK/(2*(FS_DIVIDER_VALUE+3) */
	/* FS_DIVIDER_VALUE = ((I2C_CLK / I2C_FS_CLK) / 2) - 3 */
	i2c_clk = 19200000; /* input clock */
	target_clk = 100000;
	/* target_clk = 200000; */
	fs_div = ((i2c_clk / target_clk) / 2) - 3;
	hs_div = 3;
	clk_ctl = ((hs_div & 0x7) << 8) | (fs_div & 0xff);
	writel(clk_ctl, dev->base + I2C_CLK_CTL);
	printk(KERN_INFO "msm_i2c_probe: clk_ctl %x, %d Hz\n",
	       clk_ctl, i2c_clk / (2 * ((clk_ctl & 0xff) + 3)));

	i2c_set_adapdata(&dev->adapter, dev);
	dev->adapter.algo = &msm_i2c_algo;
	strncpy(dev->adapter.name, "MSM I2C adapter", sizeof(dev->adapter.name));

	dev->adapter.nr = pdev->id;
	ret = i2c_add_numbered_adapter(&dev->adapter);
	if (ret) {
		dev_err(&pdev->dev, "i2c_add_adapter failed\n");
		goto err_i2c_add_adapter_failed;
	}

	ret = request_irq(dev->irq, msm_i2c_interrupt,
			IRQF_TRIGGER_RISING, pdev->name, dev);
	if (ret) {
		dev_err(&pdev->dev, "request_irq failed\n");
		goto err_request_irq_failed;
	}
	return 0;

/*	free_irq(dev->irq, dev); */
err_request_irq_failed:
	i2c_del_adapter(&dev->adapter);
err_i2c_add_adapter_failed:
	kfree(dev);
err_alloc_dev_failed:
	release_mem_region(mem->start, (mem->end - mem->start) + 1);
	return ret;
}

static int
msm_i2c_remove(struct platform_device *pdev)
{
	struct msm_i2c_dev	*dev = platform_get_drvdata(pdev);
	struct resource		*mem;

	platform_set_drvdata(pdev, NULL);
	free_irq(dev->irq, dev);
	i2c_del_adapter(&dev->adapter);
	kfree(dev);
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(mem->start, (mem->end - mem->start) + 1);
	return 0;
}

static struct platform_driver msm_i2c_driver = {
	.probe		= msm_i2c_probe,
	.remove		= msm_i2c_remove,
	.driver		= {
		.name	= "msm_i2c",
		.owner	= THIS_MODULE,
	},
};

/* I2C may be needed to bring up other drivers */
static int __init
msm_i2c_init_driver(void)
{
	return platform_driver_register(&msm_i2c_driver);
}
subsys_initcall(msm_i2c_init_driver);

static void __exit msm_i2c_exit_driver(void)
{
	platform_driver_unregister(&msm_i2c_driver);
}
module_exit(msm_i2c_exit_driver);

