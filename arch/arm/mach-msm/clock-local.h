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
#define NOCHECK		0	/* No bit to check, do nothing */
#define HALT		1	/* Bit pol: 1 = halted */
#define HALT_VOTED	2	/* Bit pol: 1 = halted; delay on disable */
#define ENABLE		3	/* Bit pol: 1 = running */
#define ENABLE_VOTED	4	/* Bit pol: 1 = running; delay on disable */
#define DELAY		5	/* No bit to check, just delay */

/*
 * Generic frequency-definition structs and macros
 */
struct clk_freq_tbl {
	const uint32_t	freq_hz;
	struct clk	*src_clk;
	const uint32_t	md_val;
	const uint32_t	ns_val;
	const uint32_t	cc_val;
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
	.cc_val = c_v, \
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
 * @en_reg: enable register
 * @en_mask: ORed with @en_reg to enable the clock
 * @halt_reg: halt register
 * @halt_check: type of halt check to perform
 * @halt_bit: ANDed with @halt_reg to test for clock halted
 * @reset_reg: reset register
 * @reset_mask: ORed with @reset_reg to reset the clock domain
 * @test_vector: bits to program to measure the clock
 */
struct branch {
	void __iomem *const en_reg;
	const u32 en_mask;

	void __iomem *const halt_reg;
	const u16 halt_check;
	const u16 halt_bit;

	void __iomem *const reset_reg;
	const u32 reset_mask;

	const u32 test_vector;
};

int branch_reset(struct branch *clk, enum clk_reset_action action);

/*
 * Generic clock-definition struct and macros
 */
struct clk_local {
	bool		enabled;
	void		*const ns_reg;
	void		*const md_reg;

	const uint32_t	root_en_mask;
	uint32_t	ns_mask;
	const uint32_t	cc_mask;
	struct bank_masks *const bank_masks;

	void   (*set_rate)(struct clk_local *, struct clk_freq_tbl *);
	struct clk_freq_tbl *const freq_tbl;
	struct clk_freq_tbl *current_freq;

	struct clk *depends;
	struct branch	b;
	struct clk	c;
};

static inline struct clk_local *to_local(struct clk *clk)
{
	return container_of(clk, struct clk_local, c);
}

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
 * @status_mask: ANDed with @status_reg to test if the PLL is enabled
 * @parent: clock source
 * @c: clk
 */
struct pll_vote_clk {
	unsigned long rate;

	void __iomem *const en_reg;
	const u32 en_mask;

	void __iomem *const status_reg;
	const u32 status_mask;

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
 * @status_reg: status register
 * @parent: clock source
 * @c: clk
 */
struct pll_clk {
	unsigned long rate;

	void __iomem *const mode_reg;
	void __iomem *const status_reg;

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

/*
 * clk_ops APIs
 */
int local_clk_enable(struct clk *clk);
void local_clk_disable(struct clk *clk);
void local_clk_auto_off(struct clk *clk);
int local_clk_set_rate(struct clk *clk, unsigned rate);
int local_clk_set_min_rate(struct clk *clk, unsigned rate);
int local_clk_set_max_rate(struct clk *clk, unsigned rate);
unsigned local_clk_get_rate(struct clk *clk);
int local_clk_list_rate(struct clk *clk, unsigned n);
int local_clk_is_enabled(struct clk *clk);
long local_clk_round_rate(struct clk *clk, unsigned rate);
bool local_clk_is_local(struct clk *clk);
struct clk *local_clk_get_parent(struct clk *c);

/*
 * Required SoC-specific functions, implemented for every supported SoC
 */
int soc_update_sys_vdd(enum sys_vdd_level level);
int soc_set_pwr_rail(struct clk *clk, int enable);
int soc_clk_set_flags(struct clk *clk, unsigned flags);

/*
 * Generic set-rate implementations
 */
void set_rate_mnd(struct clk_local *clk, struct clk_freq_tbl *nf);
void set_rate_nop(struct clk_local *clk, struct clk_freq_tbl *nf);
void set_rate_mnd_8(struct clk_local *clk, struct clk_freq_tbl *nf);
void set_rate_mnd_banked(struct clk_local *clk, struct clk_freq_tbl *nf);
void set_rate_div_banked(struct clk_local *clk, struct clk_freq_tbl *nf);

#endif /* __ARCH_ARM_MACH_MSM_CLOCK_LOCAL_H */

