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
 *
 */

#ifndef __H_VPU_DEBUG_H__
#define __H_VPU_DEBUG_H__

#include <linux/debugfs.h>

#include "vpu_v4l2.h"

#define VPU_DBG_TAG "VPU, %d: "

/* To enable messages OR these values and
 * echo the result to debugfs file.
 *
 * To enable all messages set debug_level = 0x7F
 */

enum vpu_msg_level {
	VPU_ERR  = 0x01,
	VPU_WARN = 0x02,
	VPU_INFO = 0x04,
	VPU_DBG  = 0x08,
	VPU_PKT  = 0x10,
	VPU_HW   = 0x20,

	VPU_ISR  = 0x40,

	VPU_ALL  = 0x7F
};

enum vpu_msg_out {
	VPU_OUT_PRINTK = 0,
	VPU_OUT_FTRACE
};

extern u8 vpu_debug_level;
extern int vpu_debug_out_type;

extern wait_queue_head_t maple_log_wq;
extern int maple_log_being_read;

#define dprintk(__level, __fmt, arg...)	\
	do { \
		if (vpu_debug_level & __level) { \
			if (vpu_debug_out_type == VPU_OUT_PRINTK) { \
				printk(/*KERN_DEBUG*/KERN_INFO VPU_DBG_TAG \
						__fmt, __level, ## arg); \
			} else if (vpu_debug_out_type == VPU_OUT_FTRACE) { \
				trace_printk(KERN_DEBUG VPU_DBG_TAG \
						__fmt, __level, ## arg); \
			} \
		} \
	} while (0)

/* enter & exit function macros */
#define VPU_ENTER_FUNC(fmt, arg...) \
		dprintk(VPU_DBG, "Enter %s " fmt "\n", __func__, ## arg)
#define VPU_EXIT_FUNC(fmt, arg...) \
		dprintk(VPU_DBG, "Exit %s " fmt "\n", __func__, ## arg)

struct dentry *init_vpu_debugfs(struct vpu_dev_core *core);

void cleanup_vpu_debugfs(struct dentry *dir);

static inline int vpu_get_debug_level(void)
{
	return (int)vpu_debug_level;
}

static inline bool vpu_logging_enabled(void)
{
	return vpu_get_debug_level() & VPU_HW;
}

/* hfi layer uses this to inform debug layer that maple sent a log msg */
void vpu_wakeup_maple_logging_wq(void);

#endif /* __H_VPU_DEBUG_H__ */

