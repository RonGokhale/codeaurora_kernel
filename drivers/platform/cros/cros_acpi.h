/*
 *  ChromeOS platform support code. Glue layer between higher level functions
 *  and per-platform firmware interfaces.
 *
 *  Copyright (C) 2011 The Chromium OS Authors
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
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef _DRIVERS_PLATFORM_CROS_CROS_ACPI_H
#define _DRIVERS_PLATFORM_CROS_CROS_ACPI_H

#ifdef CONFIG_ACPI

#include <linux/acpi.h>

#else

/* provide minimum acpi functionalities when there's no
 * acpi capable bios */
#include <linux/types.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/err.h>

extern int acpi_disabled;

#define ACPI_NAME_SIZE		4
typedef u32 acpi_status;

#define ACPI_SUCCESS(a)		(!(a))

typedef void *acpi_handle;
typedef u64 acpi_size;
typedef u32 acpi_object_type;

#define ACPI_TYPE_INTEGER	0x01
#define ACPI_TYPE_STRING	0x02
#define ACPI_TYPE_BUFFER	0x03
#define ACPI_TYPE_PACKAGE	0x04

union acpi_object {
	acpi_object_type type;
	struct {
		acpi_object_type type;	/* ACPI_TYPE_INTEGER */
		u64 value;
	} integer;

	struct {
		acpi_object_type type;	/* ACPI_TYPE_STRING */
		u32 length;
		char *pointer;
	} string;

	struct {
		acpi_object_type type;	/* ACPI_TYPE_BUFFER */
		u32 length;
		char *pointer;
	} buffer;

	struct {
		acpi_object_type type;	/* ACPI_TYPE_PACKAGE */
		u32 count;
		union acpi_object *elements;
	} package;
};

struct acpi_object_list {
	u32 count;
	union acpi_object *pointer;
};

#define ACPI_ALLOCATE_BUFFER	(acpi_size)(-1)

struct acpi_buffer {
	acpi_size length;
	void *pointer;
};

struct acpi_device {
	acpi_handle handle;
};

typedef int (*acpi_ops_add)(struct acpi_device *device);
typedef int (*acpi_ops_remove)(struct acpi_device *device, int type);

struct acpi_device_ops {
	acpi_ops_add add;
	acpi_ops_remove remove;
};

struct acpi_driver {
	char name[80];
	char class[80];
	const struct acpi_device_id *ids;
	struct acpi_device_ops ops;
	struct device_driver drv;
	struct module *owner;
};

acpi_status acpi_evaluate_object(acpi_handle object, char *pathname,
	struct acpi_object_list *parameter_objects,
	struct acpi_buffer *return_object_output);
int acpi_bus_register_driver(struct acpi_driver *driver);

#endif /* CONFIG_ACPI */
#endif /* _DRIVERS_PLATFORM_CROS_CROS_SYS_H */

