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

#ifndef __ARCH_ARM_MACH_MSM_CLOCK_RPM_H
#define __ARCH_ARM_MACH_MSM_CLOCK_RPM_H

#include <mach/rpm.h>

struct clk_ops;
extern struct clk_ops clk_ops_rpm;

struct rpm_clk {
	const int rpm_clk_id;
	const int rpm_status_id;
	const bool active_only;
	unsigned last_set_khz;
	/* 0 if active_only. Otherwise, same as last_set_khz. */
	unsigned last_set_sleep_khz;
	bool enabled;

	struct rpm_clk *peer;
	struct clk c;
};

static inline struct rpm_clk *to_rpm_clk(struct clk *clk)
{
	return container_of(clk, struct rpm_clk, c);
}

#define DEFINE_CLK_RPM(name, active, r_id) \
	static struct rpm_clk active; \
	static struct rpm_clk name = { \
		.rpm_clk_id = MSM_RPM_ID_##r_id##_CLK, \
		.rpm_status_id = MSM_RPM_STATUS_ID_##r_id##_CLK, \
		.peer = &active, \
		.c = { \
			.ops = &clk_ops_rpm, \
			.flags = CLKFLAG_SKIP_AUTO_OFF | CLKFLAG_MIN, \
			.dbg_name = #name, \
			CLK_INIT(name.c), \
		}, \
	}; \
	static struct rpm_clk active = { \
		.rpm_clk_id = MSM_RPM_ID_##r_id##_CLK, \
		.rpm_status_id = MSM_RPM_STATUS_ID_##r_id##_CLK, \
		.peer = &name, \
		.active_only = true, \
		.c = { \
			.ops = &clk_ops_rpm, \
			.flags = CLKFLAG_SKIP_AUTO_OFF | CLKFLAG_MIN, \
			.dbg_name = #active, \
			CLK_INIT(active.c), \
		}, \
	};

#endif
