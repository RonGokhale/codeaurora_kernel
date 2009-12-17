/* Copyright (c) 2008-2009, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NON-INFRINGEMENT ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef MDP_PPP_DQ_H
#define MDP_PPP_DQ_H

#include "msm_fb_def.h"

#define MDP_PPP_DEBUG_MSG MSM_FB_DEBUG

/* Max number of <Reg,Val> pairs in a ROI */
#define MDP_PPP_ROI_MAX_SIZE 256

/* ROI config command (<Reg,Val> pair) for MDP PPP block */
struct mdp_ppp_roi_cmd {
	uint32_t reg;
	uint32_t val;
};

/* This defines a ROI for MDP PPP block */
struct mdp_ppp_roi {
	uint32_t ncmds;
	struct mdp_ppp_roi_cmd cmd[MDP_PPP_ROI_MAX_SIZE];
};

/* MDP PPP Display Job (DJob) */
struct mdp_ppp_djob {
	struct list_head entry;
	/* One ROI per MDP PPP DJob */
	struct mdp_ppp_roi roi;
	struct mdp_blit_req req;
	struct fb_info *info;
	struct delayed_work cleaner;
	struct file *p_src_file, *p_dst_file;
};

extern struct completion mdp_ppp_comp;
extern boolean mdp_ppp_waiting;
extern unsigned long mdp_timer_duration;

unsigned int mdp_ppp_async_op_get(void);
void mdp_ppp_async_op_set(unsigned int flag);
void msm_fb_ensure_mem_coherency_after_dma(struct fb_info *info,
	struct mdp_blit_req *req_list, int req_list_count);
void mdp_ppp_put_img(struct file *p_src_file, struct file *p_dst_file);
void mdp_ppp_dq_init(void);
void mdp_ppp_outdw(uint32_t addr, uint32_t data);
struct mdp_ppp_djob *mdp_ppp_new_djob(void);
void mdp_ppp_process_curr_djob(void);
int mdp_ppp_get_ret_code(void);
void mdp_ppp_djob_done(void);
void mdp_ppp_wait(void);

#endif /* MDP_PPP_DQ_H */
