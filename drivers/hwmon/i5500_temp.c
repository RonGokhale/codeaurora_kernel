/*
 * i5500_temp - Driver for Intel 5500/5520/X58 chipset thermal sensor
 *
 * Copyright (C) 2012, 2014 Jean Delvare <jdelvare@suse.de>
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
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/device.h>
#include <linux/pci.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>
#include <linux/mutex.h>

/* Register definitions from datasheet */
#define REG_TSTHRCATA	0xE2
#define REG_TSCTRL	0xE8
#define REG_TSTHRRPEX	0xEB
#define REG_TSTHRLO	0xEC
#define REG_TSTHRHI	0xEE
#define REG_CTHINT	0xF0
#define REG_TSFSC	0xF3
#define REG_CTSTS	0xF4
#define REG_TSTHRRQPI	0xF5
#define REG_CTCTRL	0xF7
#define REG_TSTIMER	0xF8

struct i5500_temp_data {
	struct pci_dev *pdev;
	struct mutex update_lock;
	char valid;		/* zero until following fields are valid */
	unsigned long last_updated;	/* in jiffies */

	/* registers values */
	u16 tsthr[3];	/* 0: tsthrcata
			 * 1: tsthrlo
			 * 2: tsthrhi */
	s8 tsfsc;
	u8 ctsts;
};

static struct i5500_temp_data *
i5500_temp_update_device(struct device *dev)
{
	struct i5500_temp_data *data = dev_get_drvdata(dev);
	struct pci_dev *pdev = data->pdev;

	mutex_lock(&data->update_lock);

	if (!data->valid
	    || time_after(jiffies, data->last_updated + HZ)) {
		pci_read_config_word(pdev, REG_TSTHRCATA, &data->tsthr[0]);
		pci_read_config_word(pdev, REG_TSTHRLO, &data->tsthr[1]);
		pci_read_config_word(pdev, REG_TSTHRHI, &data->tsthr[2]);
		pci_read_config_byte(pdev, REG_TSFSC, &data->tsfsc);
		pci_read_config_byte(pdev, REG_CTSTS, &data->ctsts);

		data->last_updated = jiffies;
		data->valid = 1;
	}

	mutex_unlock(&data->update_lock);
	return data;
}

/*
 * Sysfs stuff
 */

/* Sensor resolution : 0.5 degree C */
static ssize_t show_temp(struct device *dev,
			 struct device_attribute *devattr, char *buf)
{
	struct i5500_temp_data *data = i5500_temp_update_device(dev);
	long temp;

	temp = ((long)data->tsthr[2] - data->tsfsc) * 500;

	return sprintf(buf, "%ld\n", temp);
}

static ssize_t show_thresh(struct device *dev,
			   struct device_attribute *devattr, char *buf)
{
	struct i5500_temp_data *data = i5500_temp_update_device(dev);
	int nr = to_sensor_dev_attr(devattr)->index;
	long temp;

	temp = data->tsthr[nr] * 500;

	return sprintf(buf, "%ld\n", temp);
}

static ssize_t show_alarm(struct device *dev,
			  struct device_attribute *devattr, char *buf)
{
	struct i5500_temp_data *data = i5500_temp_update_device(dev);
	int nr = to_sensor_dev_attr(devattr)->index;

	return sprintf(buf, "%u\n", (unsigned int)data->ctsts & (1 << nr));
}

static DEVICE_ATTR(temp1_input, S_IRUGO, show_temp, NULL);
static SENSOR_DEVICE_ATTR(temp1_crit, S_IRUGO, show_thresh, NULL, 0);
static SENSOR_DEVICE_ATTR(temp1_max_hyst, S_IRUGO, show_thresh, NULL, 1);
static SENSOR_DEVICE_ATTR(temp1_max, S_IRUGO, show_thresh, NULL, 2);
static SENSOR_DEVICE_ATTR(temp1_crit_alarm, S_IRUGO, show_alarm, NULL, 0);
static SENSOR_DEVICE_ATTR(temp1_max_alarm, S_IRUGO, show_alarm, NULL, 1);

static struct attribute *i5500_temp_attributes[] = {
	&dev_attr_temp1_input.attr,
	&sensor_dev_attr_temp1_crit.dev_attr.attr,
	&sensor_dev_attr_temp1_max_hyst.dev_attr.attr,
	&sensor_dev_attr_temp1_max.dev_attr.attr,
	&sensor_dev_attr_temp1_crit_alarm.dev_attr.attr,
	&sensor_dev_attr_temp1_max_alarm.dev_attr.attr,
	NULL
};

static const struct attribute_group i5500_temp_group = {
	.attrs = i5500_temp_attributes,
};

static const struct attribute_group *i5500_temp_groups[] = {
	&i5500_temp_group,
	NULL
};

static const struct pci_device_id i5500_temp_ids[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_INTEL, 0x3438) },
	{ 0 },
};

MODULE_DEVICE_TABLE(pci, i5500_temp_ids);

static int i5500_temp_probe(struct pci_dev *pdev,
			    const struct pci_device_id *id)
{
	int err;
	struct i5500_temp_data *data;
	struct device *hwmon_dev;

	data = devm_kzalloc(&pdev->dev, sizeof(struct i5500_temp_data),
			    GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	mutex_init(&data->update_lock);
	data->pdev = pdev;

	err = pci_enable_device(pdev);
	if (err) {
		dev_err(&pdev->dev, "Failed to enable device\n");
		return err;
	}

	hwmon_dev = devm_hwmon_device_register_with_groups(&pdev->dev,
							   "intel5500",
							   data,
							   i5500_temp_groups);

	return PTR_ERR_OR_ZERO(hwmon_dev);
}

static struct pci_driver i5500_temp_driver = {
	.name = "i5500_temp",
	.id_table = i5500_temp_ids,
	.probe = i5500_temp_probe,
};

static int __init i5500_temp_init(void)
{
	return pci_register_driver(&i5500_temp_driver);
}

static void __exit i5500_temp_exit(void)
{
	pci_unregister_driver(&i5500_temp_driver);
}

MODULE_AUTHOR("Jean Delvare <jdelvare@suse.de>");
MODULE_DESCRIPTION("Intel 5500/5200/X58 chipset thermal sensor driver");
MODULE_LICENSE("GPL");

module_init(i5500_temp_init)
module_exit(i5500_temp_exit)
