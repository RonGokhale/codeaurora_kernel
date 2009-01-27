/*
 * Copyright (c) 2008 QUALCOMM Incorporated.
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/errno.h>

#include <mach/board.h>
#include <mach/msm_iomap.h>

#include "acpuclock.h"

#define SHOT_SWITCH 4
#define HOP_SWITCH 5
#define SIMPLE_SLEW 6
#define COMPLEX_SLEW 7

#define SPSS_CLK_CNTL_ADDR (MSM_CSR_BASE + 0x100)
#define SPSS_CLK_SEL_ADDR (MSM_CSR_BASE + 0x104)

/* Scorpion PLL registers */
#define SCPLL_CTL_ADDR         (MSM_SCPLL_BASE + 0x4)
#define SCPLL_STATUS_ADDR      (MSM_SCPLL_BASE + 0x18)
#define SCPLL_FSM_CTL_EXT_ADDR (MSM_SCPLL_BASE + 0x10)

struct clkctl_acpu_speed {
	unsigned int     use_for_scaling;
	unsigned int     a11clk_khz;
	int              pll;
	unsigned int     a11clk_src_sel;
	unsigned int     a11clk_src_div;
	unsigned int     ahbclk_khz;
	unsigned int     ahbclk_div;
	unsigned int     sc_core_src_sel_mask;
	unsigned int     sc_l_value;
	int              vdd;
};

#define ACPU_PLL_TCXO   -1
#define ACPU_PLL_0      0
#define ACPU_PLL_1      1
#define ACPU_PLL_2      2
#define ACPU_PLL_3      3

struct clkctl_acpu_speed acpu_freq_tbl[] = {
	{ 0, 19200, ACPU_PLL_TCXO, 0, 0, 0, 0, 0, 0, 0},
	{ 0, 48000, ACPU_PLL_1, 1, 0xF, 0, 0, 0, 0, 0},
	{ 0, 64000, ACPU_PLL_1, 1, 0xB, 0, 0, 0, 0, 0},
	{ 0, 96000, ACPU_PLL_1, 1, 7, 0, 0, 0, 0, 0},
	{ 0, 128000, ACPU_PLL_1, 1, 5, 0, 0, 2, 0, 0},
	{ 0, 192000, ACPU_PLL_1, 1, 3, 0, 0, 0, 0, 0},
	{ 1, 245000, ACPU_PLL_0, 4, 0, 0, 0, 0, 0, 0}, /* 235.93 on CDMA */
	{ 0, 256000, ACPU_PLL_1, 1, 2, 0, 0, 0, 0, 0},
	{ 1, 384000, ACPU_PLL_3, 0, 0, 0, 0, 1, 0xA, 0},
	{ 0, 422400, ACPU_PLL_3, 0, 0, 0, 0, 1, 0xB, 0},
	{ 0, 460800, ACPU_PLL_3, 0, 0, 0, 0, 1, 0xC, 0},
	{ 0, 499200, ACPU_PLL_3, 0, 0, 0, 0, 1, 0xD, 0},
	{ 0, 537600, ACPU_PLL_3, 0, 0, 0, 0, 1, 0xE, 0},
	{ 1, 576000, ACPU_PLL_3, 0, 0, 0, 0, 1, 0xF, 0},
	{ 0, 614400, ACPU_PLL_3, 0, 0, 0, 0, 1, 0x10, 0},
	{ 0, 652800, ACPU_PLL_3, 0, 0, 0, 0, 1, 0x11, 0},
	{ 0, 691200, ACPU_PLL_3, 0, 0, 0, 0, 1, 0x12, 0},
	{ 0, 729600, ACPU_PLL_3, 0, 0, 0, 0, 1, 0x13, 0},
	{ 1, 768000, ACPU_PLL_3, 0, 0, 0, 0, 1, 0x14, 0},
	{ 0, 806400, ACPU_PLL_3, 0, 0, 0, 0, 1, 0x15, 0},
	{ 0, 844800, ACPU_PLL_3, 0, 0, 0, 0, 1, 0x16, 0},
	{ 0, 883200, ACPU_PLL_3, 0, 0, 0, 0, 1, 0x17, 0},
	{ 0, 921600, ACPU_PLL_3, 0, 0, 0, 0, 1, 0x18, 0},
	{ 0, 960000, ACPU_PLL_3, 0, 0, 0, 0, 1, 0x19, 0},
	{ 1, 998400, ACPU_PLL_3, 0, 0, 0, 0, 1, 0x1A, 0},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
};
struct clkctl_acpu_speed *switching_pll_speed = &acpu_freq_tbl[4];

struct clock_state {
	struct clkctl_acpu_speed	*current_speed;
	struct mutex			lock;
	uint32_t			acpu_switch_time_us;
	uint32_t			max_speed_delta_khz;
	uint32_t			vdd_switch_time_us;
	unsigned long			power_collapse_khz;
	unsigned long			wait_for_irq_khz;
};

static struct clock_state drv_state = { 0 };

static void scpll_set_freq(uint32_t lval, unsigned freq_switch)
{
	uint32_t regval;

	if (lval > 33)
		lval = 33;
	if (lval < 10)
		lval = 10;

	/* wait for any calibrations or frequency switches to finish */
	while (readl(SCPLL_STATUS_ADDR) & 0x3)
		;

	/* write the new L val and switch mode */
	regval = readl(SCPLL_FSM_CTL_EXT_ADDR);
	regval &= ~(0x3f << 3);
	regval |= (lval << 3);
	if (freq_switch == SIMPLE_SLEW)
		regval |= (0x1 << 9);

	regval &= ~(0x3 << 0);
	regval |= (freq_switch << 0);
	writel(regval, SCPLL_FSM_CTL_EXT_ADDR);

	dmb();

	/* put in normal mode */
	regval = readl(SCPLL_CTL_ADDR);
	regval |= 0x7;
	writel(regval, SCPLL_CTL_ADDR);

	dmb();

	/* wait for frequency switch to finish */
	while (readl(SCPLL_STATUS_ADDR) & 0x1)
		;

	/* status bit seems to clear early, requires at least
	 * ~8 microseconds to settle, using 20 to be safe  */
	udelay(20);
}

static void scpll_apps_enable(bool state)
{
	uint32_t regval;

	/* Wait for any frequency switches to finish. */
	while (readl(SCPLL_STATUS_ADDR) & 0x1)
		;

	/* put the pll in standby mode */
	regval = readl(SCPLL_CTL_ADDR);
	regval &= ~(0x7);
	regval |= (0x2);
	writel(regval, SCPLL_CTL_ADDR);

	dmb();

	if (state) {
		/* put the pll in normal mode */
		regval = readl(SCPLL_CTL_ADDR);
		regval |= (0x7);
		writel(regval, SCPLL_CTL_ADDR);
	} else {
		/* put the pll in power down mode */
		regval = readl(SCPLL_CTL_ADDR);
		regval &= ~(0x7);
		writel(regval, SCPLL_CTL_ADDR);
	}
	udelay(drv_state.vdd_switch_time_us);
}

static void scpll_init(uint32_t lval)
{
	/* power down scpll */
	writel(0x0, SCPLL_CTL_ADDR);

	dmb();

	/* set bypassnl, put into standby */
	writel(0x00400002, SCPLL_CTL_ADDR);

	/* set bypassnl, reset_n, full calibration */
	writel(0x00600004, SCPLL_CTL_ADDR);

	/* Ensure register write to initiate calibration has taken
	effect before reading status flag */
	dmb();

	/* wait for cal_all_done */
	while (readl(SCPLL_STATUS_ADDR) & 0x2)
		;

	/* power down scpll */
	writel(0x0, SCPLL_CTL_ADDR);

	/* switch scpll to desired freq */
	scpll_set_freq(lval, SHOT_SWITCH);
}

static void config_pll(struct clkctl_acpu_speed *s)
{
	uint32_t regval;

	if (s->pll == ACPU_PLL_3)
		scpll_set_freq(s->sc_l_value, SHOT_SWITCH);
	else {
		/* get the current clock source selection */
		regval = readl(SPSS_CLK_SEL_ADDR) & 0x1;

		/* configure the other clock source, then switch to it,
		 * using the glitch free mux */
		switch (regval) {
		case 0x0:
			regval = readl(SPSS_CLK_CNTL_ADDR);
			regval &= ~(0x7 << 4 | 0xf);
			regval |= (s->a11clk_src_sel << 4);
			regval |= (s->a11clk_src_div << 0);
			writel(regval, SPSS_CLK_CNTL_ADDR);

			regval = readl(SPSS_CLK_SEL_ADDR);
			regval |= 0x1;
			writel(regval, SPSS_CLK_SEL_ADDR);
			break;

		case 0x1:
			regval = readl(SPSS_CLK_CNTL_ADDR);
			regval &= ~(0x7 << 12 | 0xf << 8);
			regval |= (s->a11clk_src_sel << 12);
			regval |= (s->a11clk_src_div << 8);
			writel(regval, SPSS_CLK_CNTL_ADDR);

			regval = readl(SPSS_CLK_SEL_ADDR);
			regval &= ~0x1;
			writel(regval, SPSS_CLK_SEL_ADDR);
			break;
		}
		dmb();
	}

	regval = readl(SPSS_CLK_SEL_ADDR);
	regval &= ~(0x3 << 1);
	regval |= (s->sc_core_src_sel_mask << 1);
	writel(regval, SPSS_CLK_SEL_ADDR);
}

int acpuclk_set_rate(unsigned long rate, int for_power_collapse)
{
	struct clkctl_acpu_speed *tgt_s, *strt_s;

	strt_s = drv_state.current_speed;

	if (rate == (strt_s->a11clk_khz * 1000))
		return 0;

	for (tgt_s = acpu_freq_tbl; tgt_s->a11clk_khz != 0; tgt_s++) {
		if (tgt_s->a11clk_khz == (rate / 1000))
			break;
	}

	if (tgt_s->a11clk_khz == 0)
		return -EINVAL;

	if (!for_power_collapse)
		mutex_lock(&drv_state.lock);

	if (strt_s->pll != ACPU_PLL_3 && tgt_s->pll != ACPU_PLL_3) {
		config_pll(tgt_s);
	} else if (strt_s->pll != ACPU_PLL_3 && tgt_s->pll == ACPU_PLL_3) {
		scpll_apps_enable(1);
		if (strt_s != switching_pll_speed)
			config_pll(switching_pll_speed);
		config_pll(tgt_s);
	} else if (strt_s->pll == ACPU_PLL_3 && tgt_s->pll != ACPU_PLL_3) {
		config_pll(tgt_s);
		scpll_apps_enable(0);
	} else {
		config_pll(switching_pll_speed);
		config_pll(tgt_s);
	}

	/* Update the driver state with the new clock freq */
	drv_state.current_speed = tgt_s;

	if (!for_power_collapse)
		mutex_unlock(&drv_state.lock);

	return 0;
}

static void __init acpuclk_init(void)
{
	struct clkctl_acpu_speed *speed;
	uint32_t div, sel, regval;

	/* initialize scpll at 998Mhz */
	scpll_init(0x1A);

	/* Determine the source of the Scorpion clock. */
	regval = readl(SPSS_CLK_SEL_ADDR);
	switch ((regval & 0x6) >> 1) {
	case 0: /* raw source clock */
	case 3: /* low jitter PLL1 (768Mhz) */
		if (regval & 0x1) {
			sel = ((readl(SPSS_CLK_CNTL_ADDR) >> 4) & 0x7);
			div = ((readl(SPSS_CLK_CNTL_ADDR) >> 0) & 0xf);
		} else {
			sel = ((readl(SPSS_CLK_CNTL_ADDR) >> 12) & 0x7);
			div = ((readl(SPSS_CLK_CNTL_ADDR) >> 8) & 0xf);
		}

		/* Find the matching clock rate. */
		for (speed = acpu_freq_tbl; speed->a11clk_khz != 0; speed++) {
			if (speed->a11clk_src_sel == sel &&
			    speed->a11clk_src_div == div)
				break;
		}
		break;

	case 1: /* unbuffered scorpion pll (384Mhz to 998.4Mhz) */
		sel = ((readl(SCPLL_FSM_CTL_EXT_ADDR) >> 3) & 0x3f);

		/* Find the matching clock rate. */
		for (speed = acpu_freq_tbl; speed->a11clk_khz != 0; speed++) {
			if (speed->sc_l_value == sel &&
			    speed->sc_core_src_sel_mask == 1)
				break;
		}
		break;

	case 2: /* AXI bus clock (128Mhz) */
	default:
		speed = &acpu_freq_tbl[4];
	}

	if (speed->a11clk_khz == 0) {
		printk(KERN_WARNING "Warning - ACPU clock reports invalid "
			"speed\n");
		return;
	}

	drv_state.current_speed = speed;

	printk(KERN_INFO "ACPU running at %d KHz\n", speed->a11clk_khz);

	acpuclk_set_rate(998400000, 0);
}

unsigned long acpuclk_get_rate(void)
{
	return drv_state.current_speed->a11clk_khz;
}

unsigned long acpuclk_power_collapse(void)
{
	int ret = acpuclk_get_rate();
	acpuclk_set_rate(drv_state.power_collapse_khz, 1);
	return ret * 1000;
}

unsigned long acpuclk_wait_for_irq(void)
{
	int ret = acpuclk_get_rate();
	acpuclk_set_rate(drv_state.wait_for_irq_khz, 1);
	return ret * 1000;
}

void __init msm_acpu_clock_init(struct msm_acpu_clock_platform_data *clkdata)
{
	mutex_init(&drv_state.lock);

	drv_state.acpu_switch_time_us = clkdata->acpu_switch_time_us;
	drv_state.max_speed_delta_khz = clkdata->max_speed_delta_khz;
	drv_state.vdd_switch_time_us = clkdata->vdd_switch_time_us;
	drv_state.power_collapse_khz = clkdata->power_collapse_khz;
	drv_state.wait_for_irq_khz = clkdata->wait_for_irq_khz;

	acpuclk_init();
}
