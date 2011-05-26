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
#include <linux/mfd/pm8xxx/pm8921.h>
#include <linux/mfd/wcd9310/core.h>
#include <linux/mfd/wcd9310/pdata.h>
#include <linux/mfd/wcd9310/registers.h>
#include <linux/delay.h>
#include <linux/gpio.h>
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
		dev_dbg(tabla->dev, "Read 0x%02x from R%d(0x%x)\n",
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

	dev_dbg(tabla->dev, "Write %02x to R%d(0x%x)\n",
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

static u8 tabla_pgd_la;
static u8 tabla_inf_la;

int tabla_get_logical_addresses(u8 *pgd_la, u8 *inf_la)
{
	*pgd_la = tabla_pgd_la;
	*inf_la = tabla_inf_la;
	return 0;

}
EXPORT_SYMBOL_GPL(tabla_get_logical_addresses);

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

static void tabla_bring_up(struct tabla *tabla)
{
	tabla_reg_write(tabla, TABLA_A_LEAKAGE_CTL, 0x4);
	tabla_reg_write(tabla, TABLA_A_CDC_CTL, 0);
	usleep_range(5000, 5000);
	tabla_reg_write(tabla, TABLA_A_CDC_CTL, 3);
	tabla_reg_write(tabla, TABLA_A_LEAKAGE_CTL, 3);
}

static void tabla_bring_down(struct tabla *tabla)
{
	tabla_reg_write(tabla, TABLA_A_LEAKAGE_CTL, 0x7);
	tabla_reg_write(tabla, TABLA_A_LEAKAGE_CTL, 0x6);
	tabla_reg_write(tabla, TABLA_A_LEAKAGE_CTL, 0xe);
	tabla_reg_write(tabla, TABLA_A_LEAKAGE_CTL, 0x8);
}

static int tabla_reset(struct tabla *tabla)
{
	int ret;
	struct pm_gpio param = {
		.direction      = PM_GPIO_DIR_OUT,
		.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
		.output_value   = 1,
		.pull	   = PM_GPIO_PULL_NO,
		.vin_sel	= PM_GPIO_VIN_S4,
		.out_strength   = PM_GPIO_STRENGTH_MED,
		.function       = PM_GPIO_FUNC_NORMAL,
	};

	if (tabla->reset_gpio) {
		ret = gpio_request(tabla->reset_gpio, "CDC_RESET");
		if (ret) {
			pr_err("%s: Failed to request gpio %d\n", __func__,
				tabla->reset_gpio);
			tabla->reset_gpio = 0;
			return ret;
		}

		ret = pm8xxx_gpio_config(tabla->reset_gpio, &param);
		if (ret)
			pr_err("%s: Failed to configure gpio\n", __func__);

		gpio_direction_output(tabla->reset_gpio, 1);
		msleep(20);
		gpio_direction_output(tabla->reset_gpio, 0);
		msleep(20);
		gpio_direction_output(tabla->reset_gpio, 1);
		msleep(20);
	}
	return 0;
}

static void tabla_free_reset(struct tabla *tabla)
{
	if (tabla->reset_gpio) {
		gpio_free(tabla->reset_gpio);
		tabla->reset_gpio = 0;
	}
}

static int tabla_device_init(struct tabla *tabla, int irq)
{
	int ret;

	mutex_init(&tabla->io_lock);
	mutex_init(&tabla->xfer_lock);
	dev_set_drvdata(tabla->dev, tabla);

	tabla_bring_up(tabla);

	ret = tabla_irq_init(tabla);
	if (ret) {
		pr_err("IRQ initialization failed\n");
		goto err;
	}

	ret = mfd_add_devices(tabla->dev, -1,
			      tabla_devs, ARRAY_SIZE(tabla_devs),
			      NULL, 0);
	if (ret != 0) {
		dev_err(tabla->dev, "Failed to add children: %d\n", ret);
		goto err_irq;
	}

	return ret;
err_irq:
	tabla_irq_exit(tabla);
err:
	tabla_bring_down(tabla);
	mutex_destroy(&tabla->io_lock);
	mutex_destroy(&tabla->xfer_lock);
	return ret;
}
static void tabla_device_exit(struct tabla *tabla)
{
	tabla_irq_exit(tabla);
	tabla_bring_down(tabla);
	tabla_free_reset(tabla);
	mutex_destroy(&tabla->io_lock);
	mutex_destroy(&tabla->xfer_lock);
	slim_remove_device(tabla->slim_slave);
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
		goto err;
	}

	tabla = kzalloc(sizeof(struct tabla), GFP_KERNEL);
	if (tabla == NULL) {
		pr_err("%s: error, allocation failed\n", __func__);
		ret = -ENOMEM;
		goto err;
	}
	if (!slim->ctrl) {
		pr_err("Error, no SLIMBUS control data\n");
		ret = -EINVAL;
		goto err_tabla;
	}
	tabla->slim = slim;
	slim_set_clientdata(slim, tabla);
	tabla->reset_gpio = pdata->reset_gpio;

	ret = tabla_reset(tabla);
	if (ret) {
		pr_err("%s: Resetting Tabla failed\n", __func__);
		goto err_tabla;
	}

	ret = slim_get_logical_addr(tabla->slim, tabla->slim->e_addr,
		ARRAY_SIZE(tabla->slim->e_addr), &tabla->slim->laddr);
	if (ret) {
		pr_err("fail to get slimbus logical address %d\n", ret);
		goto err_reset;
	}
	tabla->read_dev = tabla_slim_read_device;
	tabla->write_dev = tabla_slim_write_device;
	tabla->irq = pdata->irq;
	tabla->irq_base = pdata->irq_base;
	tabla_pgd_la = tabla->slim->laddr;

	if (pdata->num_irqs < TABLA_NUM_IRQS) {
		pr_err("%s: Error, not enough interrupt lines allocated\n",
			__func__);
		goto err_reset;
	}

	tabla->dev = &slim->dev;

	tabla->slim_slave = &pdata->slimbus_slave_device;

	ret = slim_add_device(slim->ctrl, tabla->slim_slave);
	if (ret) {
		pr_err("%s: error, adding SLIMBUS device failed\n", __func__);
		goto err_reset;
	}

	ret = slim_get_logical_addr(tabla->slim_slave,
		tabla->slim_slave->e_addr,
		ARRAY_SIZE(tabla->slim_slave->e_addr),
			&tabla->slim_slave->laddr);
	if (ret) {
		pr_err("fail to get slimbus slave logical address %d\n", ret);
		goto err_slim_add;
	}
	tabla_inf_la = tabla->slim_slave->laddr;

	ret = tabla_device_init(tabla, tabla->irq);
	if (ret) {
		pr_err("%s: error, initializing device failed\n", __func__);
		goto err_slim_add;
	}
	return ret;

err_slim_add:
	slim_remove_device(tabla->slim_slave);
err_reset:
	tabla_free_reset(tabla);
err_tabla:
	kfree(tabla);
err:
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
	{}
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
