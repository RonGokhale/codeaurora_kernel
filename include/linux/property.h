/*
 * property.h - Unified device property interface.
 *
 * Copyright (C) 2014, Intel Corporation
 * Authors: Rafael J. Wysocki <rafael.j.wysocki@intel.com>
 *          Mika Westerberg <mika.westerberg@linux.intel.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _LINUX_PROPERTY_H_
#define _LINUX_PROPERTY_H_

#include <linux/types.h>

struct device;

enum dev_prop_type {
	DEV_PROP_U8,
	DEV_PROP_U16,
	DEV_PROP_U32,
	DEV_PROP_U64,
	DEV_PROP_STRING,
	DEV_PROP_MAX,
};

int device_get_property(struct device *dev, const char *propname,
			void **valptr);
int device_read_property(struct device *dev, const char *propname,
			 enum dev_prop_type proptype, void *val);
int device_read_property_array(struct device *dev, const char *propname,
			       enum dev_prop_type proptype, void *val,
			       size_t nval);

int device_get_child_property(struct device *dev, void *child,
			      const char *propname, void **valptr);
int device_read_child_property(struct device *dev, void *child,
			       const char *propname,
			       enum dev_prop_type proptype, void *val);
int device_read_child_property_array(struct device *dev, void *child,
				     const char *propname,
				     enum dev_prop_type proptype, void *val,
				     size_t nval);

void *device_get_next_child_node(struct device *dev, void *child);
void device_put_child_node(struct device *dev, void *child);

unsigned int device_get_child_node_count(struct device *dev);

static inline int device_property_read_u8(struct device *dev,
					  const char *propname, u8 *out_value)
{
	return device_read_property(dev, propname, DEV_PROP_U8, out_value);
}

static inline int device_property_read_u16(struct device *dev,
					  const char *propname, u16 *out_value)
{
	return device_read_property(dev, propname, DEV_PROP_U16, out_value);
}

static inline int device_property_read_u32(struct device *dev,
					  const char *propname, u32 *out_value)
{
	return device_read_property(dev, propname, DEV_PROP_U32, out_value);
}

static inline int device_property_read_u64(struct device *dev,
					  const char *propname, u64 *out_value)
{
	return device_read_property(dev, propname, DEV_PROP_U64, out_value);
}

static inline int device_property_read_u8_array(struct device *dev,
						const char *propname,
						u8 *val, size_t nval)
{
	return device_read_property_array(dev, propname, DEV_PROP_U8, val,
					  nval);
}

static inline int device_property_read_u16_array(struct device *dev,
						 const char *propname,
						 u16 *val, size_t nval)
{
	return device_read_property_array(dev, propname, DEV_PROP_U16, val,
					  nval);
}

static inline int device_property_read_u32_array(struct device *dev,
						 const char *propname,
						 u32 *val, size_t nval)
{
	return device_read_property_array(dev, propname, DEV_PROP_U32, val,
					  nval);
}

static inline int device_property_read_u64_array(struct device *dev,
						 const char *propname,
						 u64 *val, size_t nval)
{
	return device_read_property_array(dev, propname, DEV_PROP_U64, val,
					  nval);
}

static inline int device_property_read_string(struct device *dev,
					      const char *propname,
					      const char **out_string)
{
	return device_read_property(dev, propname, DEV_PROP_STRING, out_string);
}

static inline int device_property_read_string_array(struct device *dev,
						    const char *propname,
						    const char **out_strings,
						    size_t nstrings)
{
	return device_read_property_array(dev, propname, DEV_PROP_STRING,
					  out_strings, nstrings);
}

static inline int device_child_property_read_u8(struct device *dev, void *child,
						const char *propname,
						u8 *out_value)
{
	return device_read_child_property(dev, child, propname, DEV_PROP_U8,
					  out_value);
}

static inline int device_child_property_read_u16(struct device *dev, void *child,
						 const char *propname,
						 u16 *out_value)
{
	return device_read_child_property(dev, child, propname, DEV_PROP_U16,
					  out_value);
}

static inline int device_child_property_read_u32(struct device *dev, void *child,
						 const char *propname,
						 u32 *out_value)
{
	return device_read_child_property(dev, child, propname, DEV_PROP_U32,
					  out_value);
}

static inline int device_child_property_read_u64(struct device *dev, void *child,
						 const char *propname,
						 u64 *out_value)
{
	return device_read_child_property(dev, child, propname, DEV_PROP_U64,
					  out_value);
}

static inline int device_child_property_read_u8_array(struct device *dev,
						      void *child,
						      const char *propname,
						      u8 *val, size_t nval)
{
	return device_read_child_property_array(dev, child, propname,
						DEV_PROP_U8, val, nval);
}

static inline int device_child_property_read_u16_array(struct device *dev,
						       void *child,
						       const char *propname,
						       u16 *val, size_t nval)
{
	return device_read_child_property_array(dev, child, propname,
						DEV_PROP_U16, val, nval);
}

static inline int device_child_property_read_u32_array(struct device *dev,
						       void *child,
						       const char *propname,
						       u32 *val, size_t nval)
{
	return device_read_child_property_array(dev, child, propname,
						DEV_PROP_U32, val, nval);
}

static inline int device_child_property_read_u64_array(struct device *dev,
						       void *child,
						       const char *propname,
						       u64 *val, size_t nval)
{
	return device_read_child_property_array(dev, child, propname,
						DEV_PROP_U64, val, nval);
}

static inline int device_child_property_read_string(struct device *dev,
						    void *child,
						    const char *propname,
						    const char **out_string)
{
	return device_read_child_property(dev, child, propname, DEV_PROP_STRING,
					  out_string);
}

static inline int device_child_property_read_string_array(struct device *dev,
						void *child,
						const char *propname,
						const char **out_strings,
						size_t nstrings)
{
	return device_read_child_property_array(dev, child, propname,
						DEV_PROP_STRING,
						out_strings, nstrings);
}

#define device_for_each_child_node(dev, child) \
	for (child = device_get_next_child_node(dev, NULL); child; \
	     child = device_get_next_child_node(dev, child))

#endif /* _LINUX_PROPERTY_H_ */
