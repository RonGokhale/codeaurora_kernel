/* Copyright (c) 2008-2009, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora Forum nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * Alternatively, provided that this notice is retained in full, this software
 * may be relicensed by the recipient under the terms of the GNU General Public
 * License version 2 ("GPL") and only version 2, in which case the provisions of
 * the GPL apply INSTEAD OF those given above.  If the recipient relicenses the
 * software under the GPL, then the identification text in the MODULE_LICENSE
 * macro must be changed to reflect "GPLv2" instead of "Dual BSD/GPL".  Once a
 * recipient changes the license terms to the GPL, subsequent recipients shall
 * not relicense under alternate licensing terms, including the BSD or dual
 * BSD/GPL terms.  In addition, the following license statement immediately
 * below and between the words START and END shall also then apply when this
 * software is relicensed under the GPL:
 *
 * START
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 and only version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * END
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "mdp.h"

static boolean mdp_ppp_intr_flag = FALSE;
static boolean mdp_ppp_busy_flag = FALSE;

/* Worker to cleanup Display Jobs */
static struct workqueue_struct *mdp_ppp_djob_clnr;

/* Display Queue (DQ) for MDP PPP Block */
static LIST_HEAD(mdp_ppp_dq);
static DEFINE_SPINLOCK(mdp_ppp_dq_lock);

/* Current Display Job for MDP PPP */
static struct mdp_ppp_djob *curr_djob;

/* Track ret code for the last opeartion */
static int mdp_ppp_ret_code;

/* Flag to enable/disable MDP PPP Async ops */
static unsigned int mdp_ppp_async_op;

inline int mdp_ppp_get_ret_code(void)
{
	return mdp_ppp_ret_code;
}

inline unsigned int mdp_ppp_async_op_get(void)
{
	return mdp_ppp_async_op;
}

inline void mdp_ppp_async_op_set(unsigned int flag)
{

	/* if enabled, make sure MDP is free before disabling */
	if (mdp_ppp_async_op && !flag)
		mdp_ppp_wait();

	mdp_ppp_async_op = flag;
}

/* Push <Reg, Val> pair into DQ (if available) to later
 * program the MDP PPP Block */
inline void mdp_ppp_outdw(uint32_t addr, uint32_t data)
{
	if (curr_djob) {
		struct mdp_ppp_roi *roi = &curr_djob->roi;

		if (roi->ncmds >= MDP_PPP_ROI_MAX_SIZE) {
			printk(KERN_ERR "MDP_PPP: num of ROI cmds"
				" = %d not supported, max is %d \n",
				roi->ncmds, MDP_PPP_ROI_MAX_SIZE);
			mdp_ppp_ret_code = -EINVAL;
			return;
		}

		/* register ROI commands */
		roi->cmd[roi->ncmds].reg = addr;
		roi->cmd[roi->ncmds].val = data;
		roi->ncmds++;
	} else
		/* Program MDP PPP block now */
		outpdw((addr), (data));
}

/* Initialize DQ */
inline void mdp_ppp_dq_init(void)
{
	mdp_ppp_djob_clnr = create_singlethread_workqueue("MDPDJobClnrThrd");
}

/* Worker thread to reclaim resources once a display job is done */
static void mdp_ppp_djob_cleaner(struct work_struct *work)
{
	struct mdp_ppp_djob *job;
	unsigned long flags;

	MDP_PPP_DEBUG_MSG("mdp ppp display job cleaner started \n");

	/* if MDP PPP engine is free, disable INT_MDP if enabled */
	spin_lock_irqsave(&mdp_ppp_dq_lock, flags);
	if (!test_bit(0, (unsigned long *)&mdp_ppp_busy_flag) &&
		test_and_clear_bit(0, (unsigned long *)&mdp_ppp_intr_flag))
			mdp_disable_irq(MDP_PPP_TERM);
	spin_unlock_irqrestore(&mdp_ppp_dq_lock, flags);

	/* cleanup display job */
	job = container_of(work, struct mdp_ppp_djob, cleaner.work);
	if (likely(work && job)) {
		/* keep mem state coherent */
		msm_fb_ensure_mem_coherency_after_dma(job->info, &job->req, 1);

		/* release mem */
		mdp_ppp_put_img(job->p_src_file, job->p_dst_file);
		kfree(job);
	}
}

/* Create a new Display Job (DJob) */
inline struct mdp_ppp_djob *mdp_ppp_new_djob(void)
{
	struct mdp_ppp_djob *job;

	/* create a new djob */
	job = kmalloc(sizeof(struct mdp_ppp_djob), GFP_KERNEL);
	if (!job)
		return NULL;

	/* make this current djob */
	curr_djob = job;

	/* no ROI commands initially */
	curr_djob->roi.ncmds = 0;

	/* register this djob with the djob cleaner */
	INIT_DELAYED_WORK(&curr_djob->cleaner, mdp_ppp_djob_cleaner);
	INIT_LIST_HEAD(&curr_djob->entry);
	return job;
}

/* If MDP PPP engine is busy, wait until it is available again */
void mdp_ppp_wait(void)
{
	unsigned long flags;

	/* if MDP PPP Async Ops not enabled, return immediately */
	if (!mdp_ppp_async_op)
		return;

	spin_lock_irqsave(&mdp_ppp_dq_lock, flags);
	if (test_bit(0, (unsigned long *)&mdp_ppp_busy_flag)) {

		/* prepare for the wakeup event */
		test_and_set_bit(0, (unsigned long *)&mdp_ppp_waiting);
		INIT_COMPLETION(mdp_ppp_comp);
		spin_unlock_irqrestore(&mdp_ppp_dq_lock, flags);

		/* block uninterruptibly until available */
		MDP_PPP_DEBUG_MSG("waiting for mdp... \n");
		wait_for_completion_killable(&mdp_ppp_comp);
	} else
		spin_unlock_irqrestore(&mdp_ppp_dq_lock, flags);

	/* force cleanup for mem coherency */
	flush_workqueue(mdp_ppp_djob_clnr);
}

/* Program MDP PPP block to process this ROI */
static void mdp_ppp_process_roi(struct mdp_ppp_roi *roi)
{
	int i = 0, count = roi->ncmds;

	BUG_ON(!count || count >= MDP_PPP_ROI_MAX_SIZE);

	/* program PPP engine with registered ROI commands */
	for (; i < count; i++) {
		MDP_PPP_DEBUG_MSG("%d: reg: 0x%x val: 0x%x \n",
			i, roi->cmd[i].reg, roi->cmd[i].val);
		outpdw(roi->cmd[i].reg, roi->cmd[i].val);
	}

	MDP_PPP_DEBUG_MSG("reg count = %d \n", count);

	/* kickoff MDP PPP engine */
	MDP_PPP_DEBUG_MSG("kicking off mdp \n");
	outpdw(MDP_BASE + 0x30, 0x1000);
}

/* Submit this display job to MDP PPP engine */
static void mdp_ppp_dispatch_djob(struct mdp_ppp_djob *job)
{
	/* enable INT_MDP if disabled */
	if (!test_and_set_bit(0, (unsigned long *)&mdp_ppp_intr_flag))
		mdp_enable_irq(MDP_PPP_TERM);

	/* turn on PPP and CMD blocks */
	mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_ON, FALSE);
	mdp_pipe_ctrl(MDP_PPP_BLOCK, MDP_BLOCK_POWER_ON, FALSE);

	/* process this ROI */
	mdp_ppp_process_roi(&job->roi);
}

/* Enqueue this display job to be cleaned up later in "mdp_ppp_djob_done" */
static inline void mdp_ppp_enqueue_djob(struct mdp_ppp_djob *job)
{
	unsigned long flags;

	spin_lock_irqsave(&mdp_ppp_dq_lock, flags);
	list_add_tail(&job->entry, &mdp_ppp_dq);
	spin_unlock_irqrestore(&mdp_ppp_dq_lock, flags);
}

/* First enqueue display job for cleanup and dispatch immediately
 * if MDP PPP engine is free */
void mdp_ppp_process_curr_djob(void)
{
	/* enqueue djob */
	mdp_ppp_enqueue_djob(curr_djob);

	/* dispatch now if MDP PPP engine is free */
	if (!test_and_set_bit(0, (unsigned long *)&mdp_ppp_busy_flag))
		mdp_ppp_dispatch_djob(curr_djob);

	/* done with the current djob */
	curr_djob = NULL;
}

/* Called from mdp_isr - cleanup finished job and start with next
 * if available else set MDP PPP engine free */
void mdp_ppp_djob_done(void)
{
	struct mdp_ppp_djob *curr, *next;
	unsigned long flags;

	/* dequeue current */
	spin_lock_irqsave(&mdp_ppp_dq_lock, flags);
	curr = list_entry(mdp_ppp_dq.next, struct mdp_ppp_djob, entry);
	list_del_init(&curr->entry);
	spin_unlock_irqrestore(&mdp_ppp_dq_lock, flags);

	/* cleanup current */
	queue_delayed_work(mdp_ppp_djob_clnr, &curr->cleaner,
		mdp_timer_duration);

	/* grab pending */
	spin_lock_irqsave(&mdp_ppp_dq_lock, flags);
	if (!list_empty(&mdp_ppp_dq)) {
		next = list_entry(mdp_ppp_dq.next, struct mdp_ppp_djob,
			entry);
		spin_unlock_irqrestore(&mdp_ppp_dq_lock, flags);

		/* process next in the queue */
		mdp_ppp_process_roi(&next->roi);
	} else {
		/* no pending display job */
		spin_unlock_irqrestore(&mdp_ppp_dq_lock, flags);

		/* turn off PPP and CMD blocks - "in_isr" is TRUE */
		mdp_pipe_ctrl(MDP_PPP_BLOCK, MDP_BLOCK_POWER_OFF, TRUE);
		mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_OFF, TRUE);

		/* notify if waiting */
		if (test_and_clear_bit(0, (unsigned long *)&mdp_ppp_waiting))
			complete(&mdp_ppp_comp);

		/* set free */
		test_and_clear_bit(0, (unsigned long *)&mdp_ppp_busy_flag);
	}
}

