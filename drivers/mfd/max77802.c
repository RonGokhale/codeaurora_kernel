/*
 * max77802.c - mfd core driver for the Maxim 77802
 *
 * Copyright (C) 2013-2014 Google, Inc
 * Simon Glass <sjg@chromium.org>
 *
 * Copyright (C) 2012 Samsung Electronics
 * Chiwoong Byun <woong.byun@smasung.com>
 * Jonghwa Lee <jonghwa3.lee@samsung.com>
 *
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
 * This driver is based on max8997.c
 */

#include <linux/export.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/pm_runtime.h>
#include <linux/module.h>
#include <linux/mfd/core.h>
#include <linux/gpio/consumer.h>
#include <linux/mfd/max77802.h>
#include <linux/mfd/max77802-private.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/err.h>

static const struct mfd_cell max77802_devs[] = {
};

static bool max77802_pmic_is_accessible_reg(struct device *dev,
					    unsigned int reg)
{
	return (reg >= MAX77802_REG_DEVICE_ID && reg < MAX77802_REG_PMIC_END);
}

static bool max77802_rtc_is_accessible_reg(struct device *dev,
					   unsigned int reg)
{
	return (reg >= MAX77802_RTC_INT && reg < MAX77802_RTC_END);
}

static bool max77802_is_accessible_reg(struct device *dev, unsigned int reg)
{
	return (max77802_pmic_is_accessible_reg(dev, reg) ||
		max77802_rtc_is_accessible_reg(dev, reg));
}

static bool max77802_pmic_is_precious_reg(struct device *dev, unsigned int reg)
{
	return (reg == MAX77802_REG_INTSRC || reg == MAX77802_REG_INT1 ||
		reg == MAX77802_REG_INT2);
}

static bool max77802_rtc_is_precious_reg(struct device *dev, unsigned int reg)
{
	return (reg == MAX77802_RTC_INT ||
		reg == MAX77802_RTC_UPDATE0 ||
		reg == MAX77802_RTC_UPDATE1);
}

static bool max77802_is_precious_reg(struct device *dev, unsigned int reg)
{
	return (max77802_pmic_is_precious_reg(dev, reg) ||
		max77802_rtc_is_precious_reg(dev, reg));
}

static bool max77802_pmic_is_volatile_reg(struct device *dev, unsigned int reg)
{
	return (max77802_is_precious_reg(dev, reg) ||
		reg == MAX77802_REG_STATUS1 || reg == MAX77802_REG_STATUS2 ||
		reg == MAX77802_REG_PWRON);
}

static bool max77802_rtc_is_volatile_reg(struct device *dev, unsigned int reg)
{
	return (max77802_rtc_is_precious_reg(dev, reg) ||
		reg == MAX77802_RTC_SEC ||
		reg == MAX77802_RTC_MIN ||
		reg == MAX77802_RTC_HOUR ||
		reg == MAX77802_RTC_WEEKDAY ||
		reg == MAX77802_RTC_MONTH ||
		reg == MAX77802_RTC_YEAR ||
		reg == MAX77802_RTC_DATE);
}

static bool max77802_is_volatile_reg(struct device *dev, unsigned int reg)
{
	return (max77802_pmic_is_volatile_reg(dev, reg) ||
		max77802_rtc_is_volatile_reg(dev, reg));
}

static struct regmap_config max77802_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.writeable_reg = max77802_is_accessible_reg,
	.readable_reg = max77802_is_accessible_reg,
	.precious_reg = max77802_is_precious_reg,
	.volatile_reg = max77802_is_volatile_reg,
	.name = "max77802-pmic",
	.cache_type = REGCACHE_RBTREE,
};

static const struct regmap_irq max77802_irqs[] = {
	/* INT1 interrupts */
	{ .reg_offset = 0, .mask = MAX77802_INT1_PWRONF_MSK, },
	{ .reg_offset = 0, .mask = MAX77802_INT1_PWRONR_MSK, },
	{ .reg_offset = 0, .mask = MAX77802_INT1_JIGONBF_MSK, },
	{ .reg_offset = 0, .mask = MAX77802_INT1_JIGONBR_MSK, },
	{ .reg_offset = 0, .mask = MAX77802_INT1_ACOKBF_MSK, },
	{ .reg_offset = 0, .mask = MAX77802_INT1_ACOKBR_MSK, },
	{ .reg_offset = 0, .mask = MAX77802_INT1_ONKEY1S_MSK, },
	{ .reg_offset = 0, .mask = MAX77802_INT1_MRSTB_MSK, },
	/* INT2 interrupts */
	{ .reg_offset = 1, .mask = MAX77802_INT2_140C_MSK, },
	{ .reg_offset = 1, .mask = MAX77802_INT2_120C_MSK, },
};

static const struct regmap_irq_chip max77802_irq_chip = {
	.name			= "max77802-pmic",
	.status_base		= MAX77802_REG_INT1,
	.mask_base		= MAX77802_REG_INT1MSK,
	.num_regs		= 2,
	.irqs			= max77802_irqs,
	.num_irqs		= ARRAY_SIZE(max77802_irqs),
};

static const struct regmap_irq max77802_rtc_irqs[] = {
	/* RTC interrupts */
	{ .reg_offset = 0, .mask = MAX77802_RTCINT_RTC60S_MSK, },
	{ .reg_offset = 0, .mask = MAX77802_RTCINT_RTCA1_MSK, },
	{ .reg_offset = 0, .mask = MAX77802_RTCINT_RTCA2_MSK, },
	{ .reg_offset = 0, .mask = MAX77802_RTCINT_SMPL_MSK, },
	{ .reg_offset = 0, .mask = MAX77802_RTCINT_RTC1S_MSK, },
	{ .reg_offset = 0, .mask = MAX77802_RTCINT_WTSR_MSK, },
};

static const struct regmap_irq_chip max77802_rtc_irq_chip = {
	.name			= "max77802-rtc",
	.status_base		= MAX77802_RTC_INT,
	.mask_base		= MAX77802_RTC_INTM,
	.num_regs		= 1,
	.irqs			= max77802_rtc_irqs,
	.num_irqs		= ARRAY_SIZE(max77802_rtc_irqs),
};

#ifdef CONFIG_OF
static struct of_device_id max77802_pmic_dt_match[] = {
	{.compatible = "maxim,max77802", .data = NULL},
	{},
};

static void max77802_dt_parse_dvs_gpio(struct device *dev,
				       struct max77802_platform_data *pd,
				       struct device_node *np)
{
	int i;

	/*
	 * NOTE: we don't consider GPIO errors fatal; board may have some lines
	 * directly pulled high or low and thus doesn't specify them.
	 */
	for (i = 0; i < ARRAY_SIZE(pd->buck_gpio_dvs); i++)
		pd->buck_gpio_dvs[i] =
			devm_gpiod_get_index(dev, "max77802,pmic-buck-dvs", i);

	for (i = 0; i < ARRAY_SIZE(pd->buck_gpio_selb); i++)
		pd->buck_gpio_selb[i] =
			devm_gpiod_get_index(dev, "max77802,pmic-buck-selb", i);
}

static struct max77802_platform_data *max77802_i2c_parse_dt_pdata(struct device
								  *dev)
{
	struct device_node *np = dev->of_node;
	struct max77802_platform_data *pd;

	pd = devm_kzalloc(dev, sizeof(*pd), GFP_KERNEL);
	if (!pd) {
		dev_err(dev, "could not allocate memory for pdata\n");
		return NULL;
	}

	/* Read default index and ignore errors, since default is 0 */
	of_property_read_u32(np, "max77802,pmic-buck-default-dvs-idx",
			     &pd->buck_default_idx);

	max77802_dt_parse_dvs_gpio(dev, pd, np);

	dev->platform_data = pd;
	return pd;
}
#else
static struct max77802_platform_data *max77802_i2c_parse_dt_pdata(struct device
								  *dev)
{
	return 0;
}
#endif

static int max77802_i2c_probe(struct i2c_client *i2c,
			      const struct i2c_device_id *id)
{
	struct max77802_dev *max77802 = NULL;
	struct max77802_platform_data *pdata = dev_get_platdata(&i2c->dev);
	unsigned int data;
	int ret = 0;

	if (i2c->dev.of_node)
		pdata = max77802_i2c_parse_dt_pdata(&i2c->dev);

	if (!pdata) {
		dev_err(&i2c->dev, "No platform data found.\n");
		return -EIO;
	}

	max77802 = devm_kzalloc(&i2c->dev, sizeof(struct max77802_dev),
				GFP_KERNEL);
	if (max77802 == NULL)
		return -ENOMEM;

	i2c_set_clientdata(i2c, max77802);

	max77802->dev = &i2c->dev;
	max77802->i2c = i2c;
	max77802->type = id->driver_data;

	max77802->wakeup = pdata->wakeup;
	max77802->irq = i2c->irq;

	max77802->regmap = devm_regmap_init_i2c(i2c, &max77802_regmap_config);
	if (IS_ERR(max77802->regmap)) {
		ret = PTR_ERR(max77802->regmap);
		dev_err(max77802->dev, "Failed to allocate register map: %d\n",
			ret);
		return ret;
	}

	if (regmap_read(max77802->regmap,
			 MAX77802_REG_DEVICE_ID, &data) < 0) {
		dev_err(max77802->dev,
			"device not found on this channel\n");
		return -ENODEV;
	}

	ret = regmap_add_irq_chip(max77802->regmap, max77802->irq,
				  IRQF_TRIGGER_FALLING | IRQF_ONESHOT |
				  IRQF_SHARED, 0, &max77802_irq_chip,
				  &max77802->irq_data);
	if (ret != 0) {
		dev_err(&i2c->dev, "failed to add PMIC irq chip: %d\n", ret);
		return ret;
	}
	ret = regmap_add_irq_chip(max77802->regmap, max77802->irq,
				  IRQF_TRIGGER_FALLING | IRQF_ONESHOT |
				  IRQF_SHARED, 0, &max77802_rtc_irq_chip,
				  &max77802->rtc_irq_data);
	if (ret != 0) {
		dev_err(&i2c->dev, "failed to add RTC irq chip: %d\n", ret);
		goto err_del_irqc;
	}

	ret = mfd_add_devices(max77802->dev, -1, max77802_devs,
			      ARRAY_SIZE(max77802_devs), NULL, 0, NULL);
	if (ret < 0) {
		dev_err(&i2c->dev, "failed to add MFD devices: %d\n", ret);
		goto err_del_rtc_irqc;
	}

	return 0;

err_del_rtc_irqc:
	regmap_del_irq_chip(max77802->irq, max77802->rtc_irq_data);
err_del_irqc:
	regmap_del_irq_chip(max77802->irq, max77802->irq_data);
	return ret;
}

static int max77802_i2c_remove(struct i2c_client *i2c)
{
	struct max77802_dev *max77802 = i2c_get_clientdata(i2c);

	mfd_remove_devices(max77802->dev);

	return 0;
}

static const struct i2c_device_id max77802_i2c_id[] = {
	{ "max77802", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max77802_i2c_id);

#ifdef CONFIG_PM_SLEEP
static int max77802_suspend(struct device *dev)
{
	struct i2c_client *i2c = container_of(dev, struct i2c_client, dev);
	struct max77802_dev *max77802 = i2c_get_clientdata(i2c);

	if (device_may_wakeup(dev))
		enable_irq_wake(max77802->irq);

	disable_irq(max77802->irq);

	return 0;
}

static int max77802_resume(struct device *dev)
{
	struct i2c_client *i2c = container_of(dev, struct i2c_client, dev);
	struct max77802_dev *max77802 = i2c_get_clientdata(i2c);

	if (device_may_wakeup(dev))
		disable_irq_wake(max77802->irq);

	enable_irq(max77802->irq);

	return 0;
}
#endif /* CONFIG_PM_SLEEP */

static SIMPLE_DEV_PM_OPS(max77802_pm, max77802_suspend, max77802_resume);

static struct i2c_driver max77802_i2c_driver = {
	.driver = {
		   .name = "max77802",
		   .owner = THIS_MODULE,
		   .pm = &max77802_pm,
		   .of_match_table = of_match_ptr(max77802_pmic_dt_match),
	},
	.probe = max77802_i2c_probe,
	.remove = max77802_i2c_remove,
	.id_table = max77802_i2c_id,
};

static int __init max77802_i2c_init(void)
{
	return i2c_add_driver(&max77802_i2c_driver);
}
/* init early so consumer devices can complete system boot */
subsys_initcall(max77802_i2c_init);

static void __exit max77802_i2c_exit(void)
{
	i2c_del_driver(&max77802_i2c_driver);
}
module_exit(max77802_i2c_exit);

MODULE_DESCRIPTION("MAXIM 77802 multi-function core driver");
MODULE_AUTHOR("Simon Glass <sjg@chromium.org>");
MODULE_LICENSE("GPL");
