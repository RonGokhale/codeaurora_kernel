/* Copyright (c) 2009-2011, Code Aurora Forum. All rights reserved.
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

#ifndef __ARCH_ARM_MACH_MSM_CLOCK_LOCAL_H
#define __ARCH_ARM_MACH_MSM_CLOCK_LOCAL_H

#include <linux/spinlock.h>
#include "clock.h"

/*
 * Bit manipulation macros
 */
#define BM(msb, lsb)	(((((uint32_t)-1) << (31-msb)) >> (31-msb+lsb)) << lsb)
#define BVAL(msb, lsb, val)	(((val) << lsb) & BM(msb, lsb))

/*
 * Halt/Status Checking Mode Macros
 */
#define HALT		0	/* Bit pol: 1 = halted */
#define NOCHECK		1	/* No bit to check, do nothing */
#define HALT_VOTED	2	/* Bit pol: 1 = halted; delay on disable */
#define ENABLE		3	/* Bit pol: 1 = running */
#define ENABLE_VOTED	4	/* Bit pol: 1 = running; delay on disable */
#define DELAY		5	/* No bit to check, just delay */

/*
 * Clock Definition Macros
 */
#define DEFINE_CLK_MEASURE(name) \
	struct clk name = { \
		.ops = &clk_ops_measure, \
		.dbg_name = #name, \
		CLK_INIT(name), \
	}; \

/*
 * Generic frequency-definition structs and macros
 */
struct clk_freq_tbl {
	const uint32_t	freq_hz;
	struct clk	*src_clk;
	const uint32_t	md_val;
	const uint32_t	ns_val;
	const uint32_t	ctl_val;
	uint32_t	mnd_en_mask;
	const unsigned	sys_vdd;
	void		*const extra_freq_data;
};

/* Some clocks have two banks to avoid glitches when switching frequencies.
 * The unused bank is programmed while running on the other bank, and
 * switched to afterwards. The following two structs describe the banks. */
struct bank_mask_info {
	void *const md_reg;
	const uint32_t	ns_mask;
	const uint32_t	rst_mask;
	const uint32_t	mnd_en_mask;
	const uint32_t	mode_mask;
};

struct bank_masks {
	const uint32_t			bank_sel_mask;
	const struct bank_mask_info	bank0_mask;
	const struct bank_mask_info	bank1_mask;
};

#define F_RAW(f, sc, m_v, n_v, c_v, m_m, v, e) { \
	.freq_hz = f, \
	.src_clk = sc, \
	.md_val = m_v, \
	.ns_val = n_v, \
	.ctl_val = c_v, \
	.mnd_en_mask = m_m, \
	.sys_vdd = v, \
	.extra_freq_data = e, \
	}
#define FREQ_END	(UINT_MAX-1)
#define F_END \
	{ \
		.freq_hz = FREQ_END, \
		.sys_vdd = LOW, \
	}

/**
 * struct branch - branch on/off
 * @ctl_reg: clock control register
 * @en_mask: ORed with @ctl_reg to enable the clock
 * @halt_reg: halt register
 * @halt_check: type of halt check to perform
 * @halt_bit: ANDed with @halt_reg to test for clock halted
 * @reset_reg: reset register
 * @reset_mask: ORed with @reset_reg to reset the clock domain
 */
struct branch {
	void __iomem *const ctl_reg;
	const u32 en_mask;

	void __iomem *const halt_reg;
	const u16 halt_check;
	const u16 halt_bit;

	void __iomem *const reset_reg;
	const u32 reset_mask;
};

int branch_reset(struct branch *clk, enum clk_reset_action action);

/*
 * Generic clock-definition struct and macros
 */
struct rcg_clk {
	bool		enabled;
	void		*const ns_reg;
	void		*const md_reg;

	const uint32_t	root_en_mask;
	uint32_t	ns_mask;
	const uint32_t	ctl_mask;
	struct bank_masks *const bank_masks;

	void   (*set_rate)(struct rcg_clk *, struct clk_freq_tbl *);
	struct clk_freq_tbl *const freq_tbl;
	struct clk_freq_tbl *current_freq;

	struct clk *depends;
	struct branch	b;
	struct clk	c;
};

static inline struct rcg_clk *to_rcg_clk(struct clk *clk)
{
	return container_of(clk, struct rcg_clk, c);
}

int rcg_clk_enable(struct clk *clk);
void rcg_clk_disable(struct clk *clk);
void rcg_clk_auto_off(struct clk *clk);
int rcg_clk_set_rate(struct clk *clk, unsigned rate);
int rcg_clk_set_min_rate(struct clk *clk, unsigned rate);
unsigned rcg_clk_get_rate(struct clk *clk);
int rcg_clk_list_rate(struct clk *clk, unsigned n);
int rcg_clk_is_enabled(struct clk *clk);
long rcg_clk_round_rate(struct clk *clk, unsigned rate);
struct clk *rcg_clk_get_parent(struct clk *c);

/*
 * SYS_VDD voltage levels
 */
enum sys_vdd_level {
	NONE,
	LOW,
	NOMINAL,
	HIGH,
	NUM_SYS_VDD_LEVELS
};

/**
 * struct fixed_clk - fixed rate clock (used for crystal oscillators)
 * @rate: output rate
 * @c: clk
 */
struct fixed_clk {
	unsigned long rate;
	struct clk c;
};

static inline struct fixed_clk *to_fixed_clk(struct clk *clk)
{
	return container_of(clk, struct fixed_clk, c);
}

static inline unsigned fixed_clk_get_rate(struct clk *clk)
{
	struct fixed_clk *f = to_fixed_clk(clk);
	return f->rate;
}


/**
 * struct pll_vote_clk - phase locked loop (HW voteable)
 * @rate: output rate
 * @en_reg: enable register
 * @en_mask: ORed with @en_reg to enable the clock
 * @status_reg: status register
 * @parent: clock source
 * @c: clk
 */
struct pll_vote_clk {
	unsigned long rate;

	void __iomem *const en_reg;
	const u32 en_mask;

	void __iomem *const status_reg;

	struct clk *parent;
	struct clk c;
};

extern struct clk_ops clk_ops_pll_vote;

static inline struct pll_vote_clk *to_pll_vote_clk(struct clk *clk)
{
	return container_of(clk, struct pll_vote_clk, c);
}

/**
 * struct pll_clk - phase locked loop
 * @rate: output rate
 * @mode_reg: enable register
 * @parent: clock source
 * @c: clk
 */
struct pll_clk {
	unsigned long rate;

	void __iomem *const mode_reg;

	struct clk *parent;
	struct clk c;
};

extern struct clk_ops clk_ops_pll;

static inline struct pll_clk *to_pll_clk(struct clk *clk)
{
	return container_of(clk, struct pll_clk, c);
}

/**
 * struct branch_clk - branch
 * @enabled: true if clock is on, false otherwise
 * @b: branch
 * @parent: clock source
 * @c: clk
 *
 * An on/off switch with a rate derived from the parent.
 */
struct branch_clk {
	bool enabled;
	struct branch b;
	struct clk *parent;
	struct clk *depends;
	struct clk c;
};

static inline struct branch_clk *to_branch_clk(struct clk *clk)
{
	return container_of(clk, struct branch_clk, c);
}

int branch_clk_enable(struct clk *clk);
void branch_clk_disable(struct clk *clk);
struct clk *branch_clk_get_parent(struct clk *clk);
int branch_clk_set_parent(struct clk *clk, struct clk *parent);
int branch_clk_is_enabled(struct clk *clk);
void branch_clk_auto_off(struct clk *clk);
int branch_clk_reset(struct clk *c, enum clk_reset_action action);

/**
 * struct measure_clk - for rate measurement debug use
 * @sample_ticks: sample period in reference clock ticks
 * @multiplier: measurement scale-up factor
 * @divider: measurement scale-down factor
 * @c: clk
*/
struct measure_clk {
	u64 sample_ticks;
	u32 multiplier;
	u32 divider;
	struct clk c;
};

extern struct clk_ops clk_ops_measure;

static inline struct measure_clk *to_measure_clk(struct clk *clk)
{
	return container_of(clk, struct measure_clk, c);
}

/*
 * Variables from clock-local driver
 */
extern spinlock_t		local_clock_reg_lock;
extern struct clk_freq_tbl	local_dummy_freq;
extern struct fixed_clk		gnd_clk;

/*
 * Local-clock APIs
 */
int local_vote_sys_vdd(enum sys_vdd_level level);
int local_unvote_sys_vdd(enum sys_vdd_level level);
bool local_clk_is_local(struct clk *clk);

/*
 * Required SoC-specific functions, implemented for every supported SoC
 */
extern int (*soc_update_sys_vdd)(enum sys_vdd_level level);

/*
 * Generic set-rate implementations
 */
void set_rate_mnd(struct rcg_clk *clk, struct clk_freq_tbl *nf);
void set_rate_nop(struct rcg_clk *clk, struct clk_freq_tbl *nf);
void set_rate_mnd_8(struct rcg_clk *clk, struct clk_freq_tbl *nf);
void set_rate_mnd_banked(struct rcg_clk *clk, struct clk_freq_tbl *nf);
void set_rate_div_banked(struct rcg_clk *clk, struct clk_freq_tbl *nf);

#endif /* __ARCH_ARM_MACH_MSM_CLOCK_LOCAL_H */

