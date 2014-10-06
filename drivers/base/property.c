/*
 * property.c - Unified device property interface.
 *
 * Copyright (C) 2014, Intel Corporation
 * Authors: Rafael J. Wysocki <rafael.j.wysocki@intel.com>
 *          Mika Westerberg <mika.westerberg@linux.intel.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/property.h>
#include <linux/export.h>
#include <linux/acpi.h>
#include <linux/of.h>

/**
 * device_get_property - return a raw property of a device
 * @dev: Device get the property of
 * @propname: Name of the property
 * @valptr: The raw property value is stored here
 *
 * Function reads property @propname from the device firmware description and
 * stores the raw value into @valptr if found.  Otherwise returns a negative
 * errno as specified below.
 *
 * Return: %0 if the property was found (success),
 *	   %-EINVAL if given arguments are not valid,
 *	   %-ENODATA if the property does not exist.
 */
int device_get_property(struct device *dev, const char *propname, void **valptr)
{
	if (IS_ENABLED(CONFIG_OF) && dev->of_node)
		return of_dev_prop_get(dev->of_node, propname, valptr);

	return acpi_dev_prop_get(ACPI_COMPANION(dev), propname, valptr);
}
EXPORT_SYMBOL_GPL(device_get_property);

/**
 * device_read_property - read a typed property of a device
 * @dev: Device to get the property of
 * @propname: Name of the property
 * @proptype: Type of the property
 * @val: The value is stored here
 *
 * Function reads property @propname from the device firmware description and
 * stores the value into @val if found. The value is checked to be of type
 * @proptype.
 *
 * Return: %0 if the property was found (success),
 *	   %-EINVAL if given arguments are not valid,
 *	   %-ENODATA if the property does not exist,
 *	   %-EPROTO if the property type does not match @proptype,
 *	   %-EOVERFLOW if the property value is out of bounds of @proptype.
 */
int device_read_property(struct device *dev, const char *propname,
			 enum dev_prop_type proptype, void *val)
{
	if (IS_ENABLED(CONFIG_OF) && dev->of_node)
		return of_dev_prop_read(dev->of_node, propname, proptype, val);

	return acpi_dev_prop_read(ACPI_COMPANION(dev), propname, proptype, val);
}
EXPORT_SYMBOL_GPL(device_read_property);

/**
 * device_read_property_array - read an array property of a device
 * @dev: Device to get the property of
 * @propname: Name of the property
 * @proptype: Type of the property
 * @val: The values are stored here
 * @nval: Size of the @val array
 *
 * Function reads an array of properties with @propname from the device
 * firmware description and stores them to @val if found. All the values
 * in the array must be of type @proptype.
 *
 * Return: %0 if the property was found (success),
 *	   %-EINVAL if given arguments are not valid,
 *	   %-ENODATA if the property does not exist,
 *	   %-EPROTO if the property type does not match @proptype,
 *	   %-EOVERFLOW if the property value is out of bounds of @proptype.
 */
int device_read_property_array(struct device *dev, const char *propname,
			       enum dev_prop_type proptype, void *val,
			       size_t nval)
{
	if (IS_ENABLED(CONFIG_OF) && dev->of_node)
		return of_dev_prop_read_array(dev->of_node, propname, proptype,
					      val, nval);

	return acpi_dev_prop_read_array(ACPI_COMPANION(dev), propname, proptype,
					val, nval);
}
EXPORT_SYMBOL_GPL(device_read_property_array);
