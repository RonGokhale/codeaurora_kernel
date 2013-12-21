/*
 * Atmel maXTouch Touchscreen driver
 *
 * Copyright (C) 2010 Samsung Electronics Co.Ltd
 * Author: Joonyoung Shim <jy0922.shim@samsung.com>
 * Copyright (c) 2011-2012, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef __LINUX_ATMEL_TOUCH_H
#define __LINUX_ATMEL_TOUCH_H

#include <linux/types.h>

/* Orient */
#define MXT_NORMAL		0x0
#define MXT_DIAGONAL		0x1
#define MXT_HORIZONTAL_FLIP	0x2
#define MXT_ROTATED_90_COUNTER	0x3
#define MXT_VERTICAL_FLIP	0x4
#define MXT_ROTATED_90		0x5
#define MXT_ROTATED_180		0x6
#define MXT_DIAGONAL_COUNTER	0x7
#define MXT_BOOTLOADER_ID_1386E		0x10


/* Config data for a given maXTouch controller with a specific firmware */
struct mxt_config_info {
	const u8 *config;
	size_t config_length;
	u8 family_id;
	u8 variant_id;
	u8 version;
	u8 build;
	u8 bootldr_id;
	/* Points to the firmware name to be upgraded to */
	const char *fw_name;
};


/* The platform data for the Atmel maXTouch touchscreen driver */

struct autoplat001_mxt_platform_data {
	int (*init_platform_hw)(struct i2c_client *client);
	unsigned long irqflags;
	u8 (*read_chg) (void);
	char	*config_file;
};

#endif /* __LINUX_ATMEL_TOUCH_H */

