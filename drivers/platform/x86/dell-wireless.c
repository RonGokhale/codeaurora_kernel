/*
 *  dell-wireless button for Windows 8
 *
 *  Copyright (C) 2014 Alex Hung
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/acpi.h>
#include <acpi/acpi_bus.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Alex Hung");
MODULE_ALIAS("acpi*:DELLABCE:*");

#define DELL_TOGGLE_SWITCH	0
#define DELL_SLIDER_SWITCH	1

static int switch_type;

static struct input_dev *dellwl_input_dev;

static const struct acpi_device_id dellwl_ids[] = {
	{"DELLABCE", 0},
	{"", 0},
};

static int dell_wireless_input_setup(void)
{
	int err;

	dellwl_input_dev = input_allocate_device();
	if (!dellwl_input_dev)
		return -ENOMEM;

	dellwl_input_dev->name = "DELL Wireless hotkeys";
	dellwl_input_dev->phys = "dellabce/input0";
	dellwl_input_dev->id.bustype = BUS_HOST;
	dellwl_input_dev->evbit[0] = BIT(EV_KEY);
	set_bit(KEY_RFKILL, dellwl_input_dev->keybit);

	err = input_register_device(dellwl_input_dev);
	if (err)
		goto err_free_dev;

	return 0;

err_free_dev:
	input_free_device(dellwl_input_dev);
	return err;
}

static void dell_wireless_input_destroy(void)
{
	input_unregister_device(dellwl_input_dev);
}

static void dellwl_notify(struct acpi_device *acpi_dev, u32 event)
{
	if (event != 0x80) {
		pr_info("Received unknown event (0x%x)\n", event);
		return;
	}

	if (switch_type != DELL_TOGGLE_SWITCH)
		return;

	input_report_key(dellwl_input_dev, KEY_RFKILL, 1);
	input_sync(dellwl_input_dev);
	input_report_key(dellwl_input_dev, KEY_RFKILL, 0);
	input_sync(dellwl_input_dev);
}

static int dellwl_add(struct acpi_device *device)
{
	acpi_status status;
	unsigned long long output;
	int err = 0;

	status = acpi_evaluate_integer(device->handle, "CRBT", NULL, &output);
	if (!ACPI_SUCCESS(status))
		return -EINVAL;

	switch (output) {
	case 0:
	case 1:
		switch_type = DELL_TOGGLE_SWITCH;
		err = dell_wireless_input_setup();
		break;
	case 2:
	case 3:
		/* hard block is handled by module drivers */
		switch_type = DELL_SLIDER_SWITCH;
		break;
	default:
		err = -EINVAL;
	}

	return err;
}

static int dellwl_remove(struct acpi_device *device)
{
	if (switch_type == DELL_TOGGLE_SWITCH)
		dell_wireless_input_destroy();

	return 0;
}

static struct acpi_driver dellwl_driver = {
	.name	= "dell-wireless",
	.owner	= THIS_MODULE,
	.ids	= dellwl_ids,
	.ops	= {
		.add	= dellwl_add,
		.remove	= dellwl_remove,
		.notify	= dellwl_notify,
	},
};

static int __init dellwl_init(void)
{
	int err;

	pr_info("Initializing DELLABCE module\n");
	err = acpi_bus_register_driver(&dellwl_driver);
	if (err) {
		pr_err("Unable to register DELL wireless control driver.\n");
		goto error_acpi_register;
	}

	return 0;

error_acpi_register:
	return err;
}

static void __exit dellwl_exit(void)
{
	pr_info("Exiting DELLABCE module\n");
	acpi_bus_unregister_driver(&dellwl_driver);
}

module_init(dellwl_init);
module_exit(dellwl_exit);
