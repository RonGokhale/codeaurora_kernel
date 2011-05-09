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
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/mfd/core.h>
#include <linux/mfd/wcd9310/core.h>
#include <linux/mfd/wcd9310/pdata.h>
#include <linux/delay.h>
#include <sound/soc.h>

#define TABLA_REGISTER_START_OFFSET 0x800
static int tabla_read(struct tabla *tabla, unsigned short reg,
		       int bytes, void *dest, bool interface_reg)
{
	int ret;
	u8 *buf = dest;

	if (bytes <= 0) {
		dev_err(tabla->dev, "Invalid byte read length %d\n", bytes);
		return -EINVAL;
	}

	ret = tabla->read_dev(tabla, reg, bytes, dest, interface_reg);
	if (ret < 0) {
		dev_err(tabla->dev, "Tabla read failed\n");
		return ret;
	} else
		dev_info(tabla->dev, "Read 0x%02x from R%d(0x%x)\n",
			 *buf, reg, reg);

	return 0;
}
int tabla_reg_read(struct tabla *tabla, unsigned short reg)
{
	u8 val;
	int ret;

	mutex_lock(&tabla->io_lock);
	ret = tabla_read(tabla, reg, 1, &val, false);
	mutex_unlock(&tabla->io_lock);

	if (ret < 0)
		return ret;
	else
		return val;
}
EXPORT_SYMBOL_GPL(tabla_reg_read);

static int tabla_write(struct tabla *tabla, unsigned short reg,
			int bytes, void *src, bool interface_reg)
{
	u8 *buf = src;

	if (bytes <= 0) {
		pr_err("%s: Error, invalid write length\n", __func__);
		return -EINVAL;
	}

	dev_info(tabla->dev, "Write %02x to R%d(0x%x)\n",
		 *buf, reg, reg);

	return tabla->write_dev(tabla, reg, bytes, src, interface_reg);
}

int tabla_reg_write(struct tabla *tabla, unsigned short reg,
		     u8 val)
{
	int ret;

	mutex_lock(&tabla->io_lock);
	ret = tabla_write(tabla, reg, 1, &val, false);
	mutex_unlock(&tabla->io_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(tabla_reg_write);

int tabla_interface_reg_read(struct tabla *tabla, unsigned short reg)
{
	u8 val;
	int ret;

	mutex_lock(&tabla->io_lock);
	ret = tabla_read(tabla, reg, 1, &val, true);
	mutex_unlock(&tabla->io_lock);

	if (ret < 0)
		return ret;
	else
		return val;
}
EXPORT_SYMBOL_GPL(tabla_interface_reg_read);

int tabla_interface_reg_write(struct tabla *tabla, unsigned short reg,
		     u8 val)
{
	int ret;

	mutex_lock(&tabla->io_lock);
	ret = tabla_write(tabla, reg, 1, &val, true);
	mutex_unlock(&tabla->io_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(tabla_interface_reg_write);

int tabla_bulk_read(struct tabla *tabla, unsigned short reg,
		     int count, u8 *buf)
{
	int ret;

	mutex_lock(&tabla->io_lock);

	ret = tabla_read(tabla, reg, count, buf, false);

	mutex_unlock(&tabla->io_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(tabla_bulk_read);

int tabla_bulk_write(struct tabla *tabla, unsigned short reg,
		     int count, u8 *buf)
{
	int ret;

	mutex_lock(&tabla->io_lock);

	ret = tabla_write(tabla, reg, count, buf, false);

	mutex_unlock(&tabla->io_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(tabla_bulk_write);

static int tabla_slim_read_device(struct tabla *tabla, unsigned short reg,
				int bytes, void *dest, bool interface)
{
	int ret;
	struct slim_ele_access msg;
	msg.start_offset = TABLA_REGISTER_START_OFFSET + reg;
	msg.num_bytes = bytes;
	msg.comp = NULL;

	mutex_lock(&tabla->xfer_lock);
	if (interface)
		ret = slim_request_val_element(tabla->slim_slave, &msg, dest,
			bytes);
	else
		ret = slim_request_val_element(tabla->slim, &msg, dest, bytes);

	if (ret)
		pr_err("%s: Error, Tabla read failed\n", __func__);

	mutex_unlock(&tabla->xfer_lock);
	return ret;
}
/* Interface specifies whether the write is to the interface or general
 * registers.
 */
static int tabla_slim_write_device(struct tabla *tabla, unsigned short reg,
				   int bytes, void *src, bool interface)
{
	int ret;
	struct slim_ele_access msg;
	msg.start_offset = TABLA_REGISTER_START_OFFSET + reg;
	msg.num_bytes = bytes;
	msg.comp = NULL;

	mutex_lock(&tabla->xfer_lock);
	if (interface)
		ret = slim_change_val_element(tabla->slim_slave, &msg, src,
			bytes);
	else
		ret = slim_change_val_element(tabla->slim, &msg, src, bytes);
	if (ret)
		pr_err("%s: Error, Tabla write failed\n", __func__);

	mutex_unlock(&tabla->xfer_lock);
	return ret;
}

static struct mfd_cell tabla_devs[] = {
	{
		.name = "tabla_codec",
	},
};
static int tabla_device_init(struct tabla *tabla, int irq)
{
	int ret;

	mutex_init(&tabla->io_lock);
	mutex_init(&tabla->xfer_lock);
	dev_set_drvdata(tabla->dev, tabla);

	ret = tabla_irq_init(tabla);
	if (ret) {
		pr_err("IRQ initialization failed\n");
		goto err_irq;
	}

	ret = mfd_add_devices(tabla->dev, -1,
			      tabla_devs, ARRAY_SIZE(tabla_devs),
			      NULL, 0);
	if (ret != 0) {
		dev_err(tabla->dev, "Failed to add children: %d\n", ret);
		goto err;
	}

	return ret;
err:
	tabla_irq_exit(tabla);
err_irq:
	kfree(tabla);
	return ret;
}
static void tabla_device_exit(struct tabla *tabla)
{
	mutex_destroy(&tabla->io_lock);
	mutex_destroy(&tabla->xfer_lock);
	tabla_irq_exit(tabla);
	kfree(tabla);
}

static int tabla_slim_probe(struct slim_device *slim)
{
	struct tabla *tabla;
	struct tabla_pdata *pdata;
	int ret = 0;

	pdata = slim->dev.platform_data;

	if (!pdata) {
		dev_err(&slim->dev, "Error, no platform data\n");
		ret = -EINVAL;
		goto err_initialization;
	}

	tabla = kzalloc(sizeof(struct tabla), GFP_KERNEL);
	if (tabla == NULL) {
		pr_err("%s: error, allocation failed\n", __func__);
		ret = -ENOMEM;
		goto err_initialization;
	}
	if (!slim->ctrl) {
		pr_err("Error, no SLIMBUS control data\n");
		goto err_slim;
		ret = -EINVAL;
	}
	tabla->slim = slim;
	slim_set_clientdata(slim, tabla);

	/* In our simulator environment with the kernel subsystem, two SLIMBUS
	 * interrupts must be manually triggered before a SLIMBUS device
	 * calls slim_get_logical_addr, or the function call fails.  This delay
	 * gives the tester 10 seconds to trigger those interrupts so that the
	 * function call succeeds. During codec bringup, it will be determined
	 * if this delay is necessary on the actual hardware. If the delay is
	 * necessary then it will be determined what the minimum delay is for
	 * the interrupts to occur.
	 */

	msleep(10000);

	ret = slim_get_logical_addr(tabla->slim, tabla->slim->e_addr,
		ARRAY_SIZE(tabla->slim->e_addr), &tabla->slim->laddr);
	if (ret) {
		pr_err("fail to get slimbus logical address %d\n", ret);
		goto err_slim;
	}
	tabla->read_dev = tabla_slim_read_device;
	tabla->write_dev = tabla_slim_write_device;
	tabla->irq = pdata->irq;
	tabla->irq_base = pdata->irq_base;

	if (pdata->num_irqs < TABLA_NUM_IRQS) {
		pr_err("%s: Error, not enough interrupt lines allocated\n",
			__func__);
		goto err_slim;
	}

	tabla->dev = &slim->dev;

	tabla->slim_slave = &pdata->slimbus_slave_device;

	ret = slim_add_device(slim->ctrl, tabla->slim_slave);
	if (ret) {
		pr_err("%s: error, adding SLIMBUS device failed\n", __func__);
		goto err;
	}

	ret = slim_get_logical_addr(tabla->slim_slave,
		tabla->slim_slave->e_addr,
		ARRAY_SIZE(tabla->slim_slave->e_addr),
			&tabla->slim_slave->laddr);
	if (ret) {
		pr_err("fail to get slimbus slave logical address %d\n", ret);
		goto err;
	}

	ret = tabla_device_init(tabla, tabla->irq);
	if (ret) {
		pr_err("%s: error, initializing device failed\n", __func__);
		goto err;
	}
	return ret;

err:
	kfree(tabla->slim_slave);
err_slim:
	kfree(tabla);
err_initialization:
	return ret;
}
static int tabla_slim_remove(struct slim_device *pdev)
{
	struct tabla *tabla;
	tabla = slim_get_devicedata(pdev);
	tabla_device_exit(tabla);

	return 0;
}
static const struct slim_device_id slimtest_id[] = {
	{"tabla-slim", 0},
};
static struct slim_driver tabla_slim_driver = {
	.driver = {
		.name = "tabla-slim",
		.owner = THIS_MODULE,
	},
	.probe = tabla_slim_probe,
	.remove = tabla_slim_remove,
	.id_table = slimtest_id,
};
static int __init tabla_init(void)
{
	int ret;

	ret = slim_driver_register(&tabla_slim_driver);
	if (ret != 0) {
		pr_err("Failed to register tabla SB driver: %d\n", ret);
		goto err;
	}
err:
	return ret;
}
module_init(tabla_init);

static void __exit tabla_exit(void)
{
}
module_exit(tabla_exit);

MODULE_DESCRIPTION("Tabla core driver");
MODULE_VERSION("1.0");
MODULE_LICENSE("GPL v2");
