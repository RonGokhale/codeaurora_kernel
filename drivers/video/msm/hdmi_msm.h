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
 */

#ifndef __HDMI_MSM_H__
#define __HDMI_MSM_H__

#include <mach/msm_iomap.h>
#include "external_common.h"
/* #define PORT_DEBUG */

#ifdef PORT_DEBUG
const char *hdmi_msm_name(uint32 offset);
void hdmi_outp(uint32 offset, uint32 value);
uint32 hdmi_inp(uint32 offset);

#define HDMI_OUTP_ND(offset, value)	outpdw(MSM_HDMI_BASE+(offset), (value))
#define HDMI_OUTP(offset, value)	hdmi_outp((offset), (value))
#define HDMI_INP_ND(offset)		inpdw(MSM_HDMI_BASE+(offset))
#define HDMI_INP(offset)		hdmi_inp((offset))
#else
#define HDMI_OUTP_ND(offset, value)	outpdw(MSM_HDMI_BASE+(offset), (value))
#define HDMI_OUTP(offset, value)	outpdw(MSM_HDMI_BASE+(offset), (value))
#define HDMI_INP_ND(offset)		inpdw(MSM_HDMI_BASE+(offset))
#define HDMI_INP(offset)		inpdw(MSM_HDMI_BASE+(offset))
#endif

#define QFPROM_BASE		((uint32)hdmi_msm_state->qfprom_io)

struct hdmi_msm_state_type {
	boolean panel_power_on;
	boolean hpd_initialized;
#ifdef CONFIG_SUSPEND
	boolean pm_suspended;
#endif
	int hpd_stable;
	boolean hpd_prev_state;
	boolean hpd_cable_chg_detected;
	boolean full_auth_done;
	boolean hpd_during_auth;
	struct work_struct hpd_state_work, hpd_read_work;
	struct timer_list hpd_state_timer;
	struct completion ddc_sw_done;

#ifdef CONFIG_FB_MSM_HDMI_MSM_PANEL_HDCP_SUPPORT
	boolean hdcp_activating;
	boolean reauth ;
	struct work_struct hdcp_reauth_work, hdcp_work;
	struct completion hdcp_success_done;
	struct timer_list hdcp_timer;
#endif /* CONFIG_FB_MSM_HDMI_MSM_PANEL_HDCP_SUPPORT */

	int irq;
	struct msm_hdmi_platform_data *pd;
	struct clk *hdmi_app_clk;
	struct clk *hdmi_m_pclk;
	struct clk *hdmi_s_pclk;
	void __iomem *qfprom_io;
	void __iomem *hdmi_io;

	struct external_common_state_type common;
};

extern struct hdmi_msm_state_type *hdmi_msm_state;

uint32 hdmi_msm_get_io_base(void);

#ifdef CONFIG_FB_MSM_HDMI_COMMON
void hdmi_msm_set_mode(boolean power_on);
int hdmi_msm_clk(int on);
void hdmi_phy_reset(void);
void hdmi_msm_reset_core(void);
void hdmi_msm_init_phy(int video_format);
void hdmi_msm_powerdown_phy(void);
void hdmi_frame_ctrl_cfg(const struct hdmi_disp_mode_timing_type *timing);
void hdmi_msm_phy_status_poll(void);
#endif

#endif /* __HDMI_MSM_H__ */
