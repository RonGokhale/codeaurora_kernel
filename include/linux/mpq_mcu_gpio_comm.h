/* Copyright (c) 2013, The Linux Foundation. All rights reserved.
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

#ifndef _MPQ_MCU_GPIO_COMM_H
#define _MPQ_MCU_GPIO_COMM_H

#include <linux/types.h>
#include <linux/ioctl.h>

#define MPQ_MCU_MAGIC 'M'

#define MPQ_SET_MCU_BOOT_DLOAD \
	_IO(MPQ_MCU_MAGIC, 0x1)
#define MPQ_SET_MCU_BOOT_NORMAL \
	_IO(MPQ_MCU_MAGIC, 0x2)
#define MPQ_SET_MCU_WAKEUP_SRC \
	_IO(MPQ_MCU_MAGIC, 0x3)

#ifdef __KERNEL__
void mpq_mcu_dev_power_off(void);
#endif
#endif
