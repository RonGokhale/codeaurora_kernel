/* linux/include/asm-arm/arch-msm/board.h
 *
 * Copyright (C) 2007 Google, Inc.
 * Author: Brian Swetland <swetland@google.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __ASM_ARCH_MSM_BOARD_H
#define __ASM_ARCH_MSM_BOARD_H

#include <linux/types.h>

/* platform device data structures */

struct mddi_panel_info;
struct msm_mddi_platform_data
{
	void (*mddi_client_power)(int on);
	void (*mddi_enable)(struct mddi_panel_info *panel, int on);
	void (*panel_power)(struct mddi_panel_info *panel, int on);
	unsigned has_vsync_irq:1;
	unsigned long fb_base;
	unsigned long fb_size;
};

struct android_pmem_platform_data
{
	const char* name;
	/* starting physical address of memory region */
	unsigned long start;
	/* size of memory region */
	unsigned long size;
	/* set to indicate the region should not be managed with an allocator */
	unsigned no_allocator;
	/* set to indicate maps of this region should be cached, if a mix of
	 * cached and uncached is desired, set this and open the device with
	 * O_SYNC to get an uncached region */
	unsigned cached;
	/* The MSM7k has bits to enable a write buffer in the bus controller*/
	unsigned buffered;
};

struct msm_camera_device_platform_data{
	int sensor_reset;
	int sensor_pwd;
	int vcm_pwd;
};

struct snd_endpoint
{
	int id;
	const char *name;
};

struct msm_snd_endpoints
{
        struct snd_endpoint *endpoints;
        unsigned num;
};

/* common init routines for use by arch/arm/mach-msm/board-*.c */

void __init msm_add_devices(void);
void __init msm_map_common_io(void);
void __init msm_init_irq(void);
void __init msm_init_gpio(void);

struct mmc_platform_data;
int __init msm_add_sdcc(unsigned int controller, struct mmc_platform_data *plat);

#if defined(CONFIG_USB_FUNCTION_MSM_HSUSB)
void msm_hsusb_set_vbus_state(int online);
#else
static inline void msm_hsusb_set_vbus_state(int online) {}
#endif

#endif
