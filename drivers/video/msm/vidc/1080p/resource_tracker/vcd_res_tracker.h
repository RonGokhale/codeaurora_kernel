/* Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of Code Aurora Forum, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#ifndef _VIDEO_720P_RESOURCE_TRACKER_H_
#define _VIDEO_720P_RESOURCE_TRACKER_H_

#include <linux/regulator/consumer.h>
#include "vcd_res_tracker_api.h"
#ifdef CONFIG_MSM_BUS_SCALING
#include <mach/msm_bus.h>
#include <mach/msm_bus_board.h>
#endif
#include <mach/board.h>

#define RESTRK_1080P_VGA_PERF_LEVEL    VCD_MIN_PERF_LEVEL
#define RESTRK_1080P_720P_PERF_LEVEL   108000
#define RESTRK_1080P_1080P_PERF_LEVEL  244800

#define RESTRK_1080P_MIN_PERF_LEVEL RESTRK_1080P_VGA_PERF_LEVEL
#define RESTRK_1080P_MAX_PERF_LEVEL RESTRK_1080P_1080P_PERF_LEVEL
struct res_trk_context {
	struct device *device;
	u32 irq_num;
	struct mutex lock;
	struct clk *vcodec_clk;
	struct clk *vcodec_pclk;
	unsigned long vcodec_clk_rate;
	unsigned int clock_enabled;
	unsigned int perf_level;
	struct regulator *footswitch;
	struct msm_vidc_platform_data *vidc_platform_data;
	int memtype;
#ifdef CONFIG_MSM_BUS_SCALING
	struct msm_bus_scale_pdata *vidc_bus_client_pdata;
	uint32_t     pcl;
#endif
	u32 core_type;
	struct ddl_buf_addr firmware_addr;
};

#if DEBUG

#define VCDRES_MSG_LOW(xx_fmt...)	printk(KERN_INFO "\n\t* " xx_fmt)
#define VCDRES_MSG_MED(xx_fmt...)	printk(KERN_INFO "\n  * " xx_fmt)

#else

#define VCDRES_MSG_LOW(xx_fmt...)
#define VCDRES_MSG_MED(xx_fmt...)

#endif

#define VCDRES_MSG_HIGH(xx_fmt...)	printk(KERN_WARNING "\n" xx_fmt)
#define VCDRES_MSG_ERROR(xx_fmt...)	printk(KERN_ERR "\n err: " xx_fmt)
#define VCDRES_MSG_FATAL(xx_fmt...)	printk(KERN_ERR "\n<FATAL> " xx_fmt)

#endif
