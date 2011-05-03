/* Copyright (c) 2008-2011, Code Aurora Forum. All rights reserved.
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/time.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/semaphore.h>
#include <linux/uaccess.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <asm/system.h>
#include <asm/mach-types.h>
#include <mach/hardware.h>
#include <mach/gpio.h>
#include <mach/clk.h>
#include <mach/msm_iomap.h>

#include "msm_fb.h"
#include "mipi_dsi.h"
#include "mdp.h"
#include "mdp4.h"

u32 dsi_irq;
int mipi_dsi_clk_on;
static struct dsi_clk_desc dsicore_clk;
static struct dsi_clk_desc dsi_pclk;

static int mipi_dsi_probe(struct platform_device *pdev);
static int mipi_dsi_remove(struct platform_device *pdev);

static int mipi_dsi_off(struct platform_device *pdev);
static int mipi_dsi_on(struct platform_device *pdev);

static struct clk *dsi_byte_div_clk;
static struct clk *dsi_esc_clk;
#ifndef CONFIG_FB_MSM_MDP303
static struct clk *dsi_m_pclk;
static struct clk *dsi_s_pclk;

static struct clk *amp_pclk;

static char *mmss_cc_base;	/* mutimedia sub system clock control */
static char *mmss_sfpb_base;	/* mutimedia sub system sfpb */

static void  __iomem *periph_base;
#else
static struct clk *dsi_pixel_clk;
static struct clk *dsi_clk;
static struct clk *dsi_ref_clk;
static struct clk *mdp_dsi_pclk;
static struct clk *ahb_m_clk;
static struct clk *ahb_s_clk;
#endif

static struct platform_device *pdev_list[MSM_FB_MAX_DEV_LIST];
static int pdev_list_cnt;
static struct mipi_dsi_platform_data *mipi_dsi_pdata;

static int vsync_gpio = -1;

static struct platform_driver mipi_dsi_driver = {
	.probe = mipi_dsi_probe,
	.remove = mipi_dsi_remove,
	.shutdown = NULL,
	.driver = {
		   .name = "mipi_dsi",
		   },
};

struct device dsi_dev;

static void mipi_dsi_configure_serdes(void)
{
	void __iomem *cc;

	/* PHY registers programemd thru S2P interface */
	if (periph_base) {
		MIPI_OUTP(periph_base + 0x2c, 0x000000b6);
		MIPI_OUTP(periph_base + 0x2c, 0x000001b5);
		MIPI_OUTP(periph_base + 0x2c, 0x000001b4);
		MIPI_OUTP(periph_base + 0x2c, 0x000003b3);
		MIPI_OUTP(periph_base + 0x2c, 0x000003a2);
		MIPI_OUTP(periph_base + 0x2c, 0x000002a1);
		MIPI_OUTP(periph_base + 0x2c, 0x000008a0);
		MIPI_OUTP(periph_base + 0x2c, 0x00000d9f);
		MIPI_OUTP(periph_base + 0x2c, 0x0000109e);
		MIPI_OUTP(periph_base + 0x2c, 0x0000209d);
		MIPI_OUTP(periph_base + 0x2c, 0x0000109c);
		MIPI_OUTP(periph_base + 0x2c, 0x0000079a);
		MIPI_OUTP(periph_base + 0x2c, 0x00000c99);
		MIPI_OUTP(periph_base + 0x2c, 0x00002298);
		MIPI_OUTP(periph_base + 0x2c, 0x000000a7);
		MIPI_OUTP(periph_base + 0x2c, 0x000000a6);
		MIPI_OUTP(periph_base + 0x2c, 0x000000a5);
		MIPI_OUTP(periph_base + 0x2c, 0x00007fa4);
		MIPI_OUTP(periph_base + 0x2c, 0x0000eea8);
		MIPI_OUTP(periph_base + 0x2c, 0x000006aa);
		MIPI_OUTP(periph_base + 0x2c, 0x00002095);
		MIPI_OUTP(periph_base + 0x2c, 0x00000493);
		MIPI_OUTP(periph_base + 0x2c, 0x00001092);
		MIPI_OUTP(periph_base + 0x2c, 0x00000691);
		MIPI_OUTP(periph_base + 0x2c, 0x00005490);
		MIPI_OUTP(periph_base + 0x2c, 0x0000038d);
		MIPI_OUTP(periph_base + 0x2c, 0x0000148c);
		MIPI_OUTP(periph_base + 0x2c, 0x0000058b);
		MIPI_OUTP(periph_base + 0x2c, 0x0000078a);
		MIPI_OUTP(periph_base + 0x2c, 0x00001f89);
		MIPI_OUTP(periph_base + 0x2c, 0x00003388);
		MIPI_OUTP(periph_base + 0x2c, 0x00006387);
		MIPI_OUTP(periph_base + 0x2c, 0x00004886);
		MIPI_OUTP(periph_base + 0x2c, 0x00005085);
		MIPI_OUTP(periph_base + 0x2c, 0x00000084);
		MIPI_OUTP(periph_base + 0x2c, 0x0000da83);
		MIPI_OUTP(periph_base + 0x2c, 0x0000b182);
		MIPI_OUTP(periph_base + 0x2c, 0x00002f81);
		MIPI_OUTP(periph_base + 0x2c, 0x00004080);
		MIPI_OUTP(periph_base + 0x2c, 0x00004180);
		MIPI_OUTP(periph_base + 0x2c, 0x000006aa);
	}

	cc = MIPI_DSI_BASE + 0x0130;
	MIPI_OUTP(cc, 0x806c11c8);
	MIPI_OUTP(cc, 0x804c11c8);
	MIPI_OUTP(cc, 0x806d0080);
	MIPI_OUTP(cc, 0x804d0080);
	MIPI_OUTP(cc, 0x00000000);
	MIPI_OUTP(cc, 0x807b1597);
	MIPI_OUTP(cc, 0x805b1597);
	MIPI_OUTP(cc, 0x807c0080);
	MIPI_OUTP(cc, 0x805c0080);
	MIPI_OUTP(cc, 0x00000000);
	MIPI_OUTP(cc, 0x807911c8);
	MIPI_OUTP(cc, 0x805911c8);
	MIPI_OUTP(cc, 0x807a0080);
	MIPI_OUTP(cc, 0x805a0080);
	MIPI_OUTP(cc, 0x00000000);
	MIPI_OUTP(cc, 0x80721555);
	MIPI_OUTP(cc, 0x80521555);
	MIPI_OUTP(cc, 0x80730000);
	MIPI_OUTP(cc, 0x80530000);
	MIPI_OUTP(cc, 0x00000000);
}

static void mipi_dsi_clk(struct dsi_clk_desc *clk, int clk_en)
{
#ifndef CONFIG_FB_MSM_MDP303
	char	*cc, *ns, *md;
	int	pmxo_sel = 0;
	char	mnd_en = 1, root_en = 1;
	uint32	data, val;

	cc = mmss_cc_base + 0x004c;
	md = mmss_cc_base + 0x0050;
	ns = mmss_cc_base + 0x0054;

	if (clk_en) {
		if (clk->mnd_mode == 0) {
			data  = clk->pre_div_func << 14;
			data |= clk->src;
			MIPI_OUTP_SECURE(ns, data);
			MIPI_OUTP_SECURE(cc, ((pmxo_sel << 8)
						| (clk->mnd_mode << 6)
						| (root_en << 2) | clk_en));
		} else {
			val = clk->d * 2;
			data = (~val) & 0x0ff;
			data |= clk->m << 8;
			MIPI_OUTP_SECURE(md, data);

			val = clk->n - clk->m;
			data = (~val) & 0x0ff;
			data <<= 24;
			data |= clk->src;
			MIPI_OUTP_SECURE(ns, data);

			MIPI_OUTP_SECURE(cc, ((pmxo_sel << 8)
					      | (clk->mnd_mode << 6)
					      | (mnd_en << 5)
					      | (root_en << 2) | clk_en));
		}

	} else
		MIPI_OUTP_SECURE(cc, 0);

	wmb();
#else
	uint32 data;
	if (clk_en) {
		data = (clk->pre_div_func) << 24 |
			(clk->m) << 16 | (clk->n) << 8 |
			((clk->d) * 2);
		clk_set_rate(dsi_clk, data);
		clk_enable(dsi_clk);
	} else
		clk_disable(dsi_clk);
#endif
}

#ifndef CONFIG_FB_MSM_MDP303
static void mipi_dsi_sfpb_cfg(void)
{
	char *sfpb;
	int data;

	sfpb = mmss_sfpb_base + 0x058;

	data = MIPI_INP(sfpb);
	data |= 0x01800;
	MIPI_OUTP(sfpb, data);
	wmb();
}
#endif

static void mipi_dsi_pclk(struct dsi_clk_desc *clk, int clk_en)
{
#ifndef CONFIG_FB_MSM_MDP303
	char	*cc, *ns, *md;
	char	mnd_en = 1, root_en = 1;
	uint32	data, val;

	cc = mmss_cc_base + 0x0130;
	md = mmss_cc_base + 0x0134;
	ns = mmss_cc_base + 0x0138;

	if (clk_en) {
		if (clk->mnd_mode == 0) {
			data  = clk->pre_div_func << 12;
			data |= clk->src;
			MIPI_OUTP_SECURE(ns, data);
			MIPI_OUTP_SECURE(cc, ((clk->mnd_mode << 6)
					      | (root_en << 2) | clk_en));
		} else {
			val = clk->d * 2;
			data = (~val) & 0x0ff;
			data |= clk->m << 8;
			MIPI_OUTP_SECURE(md, data);

			val = clk->n - clk->m;
			data = (~val) & 0x0ff;
			data <<= 24;
			data |= clk->src;
			MIPI_OUTP_SECURE(ns, data);

			MIPI_OUTP_SECURE(cc, ((clk->mnd_mode << 6)
					      | (mnd_en << 5)
					      | (root_en << 2) | clk_en));
		}

	} else
		MIPI_OUTP_SECURE(cc, 0);

	wmb();
#else
	uint32 data;

	if (clk_en) {
		data = (clk->pre_div_func) << 24 | (clk->m) << 16
			| (clk->n) << 8 | ((clk->d) * 2);
		if ((clk_set_rate(dsi_pixel_clk, data)) < 0)
			pr_err("%s: pixel clk set rate failed\n", __func__);
		if (clk_enable(dsi_pixel_clk))
			pr_err("%s clk enable failed\n", __func__);
	} else {
		clk_disable(dsi_pixel_clk);
	}
#endif
}

#ifndef CONFIG_FB_MSM_MDP303
static void mipi_dsi_ahb_en(void)
{
	char	*ahb;

	ahb = mmss_cc_base + 0x08;

	pr_debug("%s: ahb=%x %x\n",
		__func__, (int) ahb, MIPI_INP_SECURE(ahb));
}
#endif

static void mipi_dsi_calibration(void)
{
#ifndef CONFIG_FB_MSM_MDP303
	uint32 data;

	MIPI_OUTP(MIPI_DSI_BASE + 0xf4, 0x0000ff11); /* cal_ctrl */
	MIPI_OUTP(MIPI_DSI_BASE + 0xf0, 0x01); /* cal_hw_trigger */

	while (1) {
		data = MIPI_INP(MIPI_DSI_BASE + 0xfc); /* cal_status */
		if ((data & 0x10000000) == 0)
			break;

		udelay(10);
	}
#endif
	MIPI_OUTP(MIPI_DSI_BASE + 0xf8, 0x00a105a1); /* cal_hw_ctrl */
}

struct dsiphy_pll_divider_config {
	u32 clk_rate;
	u32 fb_divider;
	u32 ref_divider_ratio;
	u32 bit_clk_divider;	/* oCLK1 */
	u32 byte_clk_divider;	/* oCLK2 */
	u32 dsi_clk_divider;	/* oCLK3 */
};

struct dsi_clk_mnd_table {
	uint8 lanes;
	uint8 bpp;
	uint8 dsiclk_div;
	uint8 dsiclk_m;
	uint8 dsiclk_n;
	uint8 dsiclk_d;
	uint8 pclk_m;
	uint8 pclk_n;
	uint8 pclk_d;
};

#ifndef CONFIG_FB_MSM_MDP303
#define PREF_DIV_RATIO 27
#else
#define PREF_DIV_RATIO 19
#endif
static struct dsiphy_pll_divider_config pll_divider_config;

static const struct dsi_clk_mnd_table mnd_table[] = {
	{ 1, 2, 8, 1, 1, 0, 1,  2, 0},
	{ 1, 3, 8, 1, 1, 0, 1,  3, 2},
	{ 2, 2, 4, 1, 1, 0, 1,  2, 0},
	{ 2, 3, 4, 1, 1, 0, 1,  3, 2},
	{ 3, 2, 1, 3, 8, 4, 3, 16, 7},
	{ 3, 3, 1, 3, 8, 4, 1,  8, 4},
	{ 4, 2, 2, 1, 1, 0, 1,  2, 0},
	{ 4, 3, 2, 1, 1, 0, 1,  3, 2},
};

int mipi_dsi_phy_pll_config(u32 clk_rate)
{
	struct dsiphy_pll_divider_config *dividers;
	u32 fb_divider, tmp;
	dividers = &pll_divider_config;

	/* DSIPHY_PLL_CTRL_x:    1     2     3     8     9     10 */
	/* masks               0xff  0x07  0x3f  0x0f  0xff  0xff */

	/* DSIPHY_PLL_CTRL_1 */
	fb_divider = ((dividers->fb_divider) / 2) - 1;
	MIPI_OUTP(MIPI_DSI_BASE + 0x204, fb_divider & 0xff);

	/* DSIPHY_PLL_CTRL_2 */
	tmp = MIPI_INP(MIPI_DSI_BASE + 0x208);
	tmp &= ~0x07;
	tmp |= (fb_divider >> 8) & 0x07;
	MIPI_OUTP(MIPI_DSI_BASE + 0x208, tmp);

	/* DSIPHY_PLL_CTRL_3 */
	tmp = MIPI_INP(MIPI_DSI_BASE + 0x20c);
	tmp &= ~0x3f;
	tmp |= (dividers->ref_divider_ratio - 1) & 0x3f;
	MIPI_OUTP(MIPI_DSI_BASE + 0x20c, tmp);

	/* DSIPHY_PLL_CTRL_8 */
	tmp = MIPI_INP(MIPI_DSI_BASE + 0x220);
	tmp &= ~0x0f;
	tmp |= (dividers->bit_clk_divider - 1) & 0x0f;
	MIPI_OUTP(MIPI_DSI_BASE + 0x220, tmp);

	/* DSIPHY_PLL_CTRL_9 */
	MIPI_OUTP(MIPI_DSI_BASE + 0x224, (dividers->byte_clk_divider - 1));

	/* DSIPHY_PLL_CTRL_10 */
	MIPI_OUTP(MIPI_DSI_BASE + 0x228, (dividers->dsi_clk_divider - 1));

	return 0;
}

int mipi_dsi_clk_div_config(uint8 bpp, uint8 lanes,
			    uint32 *expected_dsi_pclk)
{
	u32 fb_divider, rate, vco;
	u32 div_ratio = 0;
	struct dsi_clk_mnd_table const *mnd_entry = mnd_table;
	if (pll_divider_config.clk_rate == 0)
		pll_divider_config.clk_rate = 454000000;

	rate = pll_divider_config.clk_rate / 1000000; /* In Mhz */

	if (rate < 125) {
		vco = rate * 8;
		div_ratio = 8;
	} else if (rate < 250) {
		vco = rate * 4;
		div_ratio = 4;
	} else if (rate < 500) {
		vco = rate * 2;
		div_ratio = 2;
	} else {
		vco = rate * 1;
		div_ratio = 1;
	}

	/* find the mnd settings from mnd_table entry */
	for (; mnd_entry != mnd_table + ARRAY_SIZE(mnd_table); ++mnd_entry) {
		if (((mnd_entry->lanes) == lanes) &&
			((mnd_entry->bpp) == bpp))
			break;
	}

	if (mnd_entry == mnd_table + ARRAY_SIZE(mnd_table)) {
		pr_err("%s: requested Lanes, %u & BPP, %u, not supported\n",
			__func__, lanes, bpp);
		return -EINVAL;
	}
	fb_divider = ((vco * PREF_DIV_RATIO) / 27);
	pll_divider_config.fb_divider = fb_divider;
	pll_divider_config.ref_divider_ratio = PREF_DIV_RATIO;
	pll_divider_config.bit_clk_divider = div_ratio;
	pll_divider_config.byte_clk_divider =
			pll_divider_config.bit_clk_divider * 8;
	pll_divider_config.dsi_clk_divider =
			(mnd_entry->dsiclk_div) * div_ratio;

	if ((mnd_entry->dsiclk_d == 0)
		|| (mnd_entry->dsiclk_m == 1)) {
		dsicore_clk.mnd_mode = 0;
		dsicore_clk.src = 0x3;
		dsicore_clk.pre_div_func = (mnd_entry->dsiclk_n - 1);
	} else {
		dsicore_clk.mnd_mode = 2;
		dsicore_clk.src = 0x3;
		dsicore_clk.m = mnd_entry->dsiclk_m;
		dsicore_clk.n = mnd_entry->dsiclk_n;
		dsicore_clk.d = mnd_entry->dsiclk_d;
	}

	if ((mnd_entry->pclk_d == 0)
		|| (mnd_entry->pclk_m == 1)) {
		dsi_pclk.mnd_mode = 0;
		dsi_pclk.src = 0x3;
		dsi_pclk.pre_div_func = (mnd_entry->pclk_n - 1);
		*expected_dsi_pclk = ((vco * 1000000) /
					((pll_divider_config.dsi_clk_divider)
					* (mnd_entry->pclk_n)));
	} else {
		dsi_pclk.mnd_mode = 2;
		dsi_pclk.src = 0x3;
		dsi_pclk.m = mnd_entry->pclk_m;
		dsi_pclk.n = mnd_entry->pclk_n;
		dsi_pclk.d = mnd_entry->pclk_d;
		*expected_dsi_pclk = ((vco * 1000000 * dsi_pclk.m) /
					((pll_divider_config.dsi_clk_divider)
					* (mnd_entry->pclk_n)));
	}
#ifdef CONFIG_FB_MSM_MDP303
	dsicore_clk.m = 1;
	dsicore_clk.n = 1;
	dsicore_clk.d = 2;
	dsicore_clk.pre_div_func = 0;

	dsi_pclk.m = 1;
	dsi_pclk.n = 3;
	dsi_pclk.d = 2;
	dsi_pclk.pre_div_func = 0;
#endif
	return 0;
}

void mipi_dsi_phy_init(int panel_ndx, struct msm_panel_info const *panel_info)
{
	struct mipi_dsi_phy_ctrl *pd;
	int i, off;

	MIPI_OUTP(MIPI_DSI_BASE + 0x128, 0x0001);/* start phy sw reset */
	msleep(100);
	MIPI_OUTP(MIPI_DSI_BASE + 0x128, 0x0000);/* end phy w reset */
	MIPI_OUTP(MIPI_DSI_BASE + 0x2cc, 0x0003);/* regulator_ctrl_0 */
	MIPI_OUTP(MIPI_DSI_BASE + 0x2d0, 0x0001);/* regulator_ctrl_1 */
	MIPI_OUTP(MIPI_DSI_BASE + 0x2d4, 0x0001);/* regulator_ctrl_2 */
	MIPI_OUTP(MIPI_DSI_BASE + 0x2d8, 0x0000);/* regulator_ctrl_3 */
#ifdef DSI_POWER
	MIPI_OUTP(MIPI_DSI_BASE + 0x2dc, 0x0100);/* regulator_ctrl_4 */
#endif

	pd = (panel_info->mipi).dsi_phy_db;

	off = 0x02cc;	/* regulator ctrl 0 */
	for (i = 0; i < 4; i++) {
		MIPI_OUTP(MIPI_DSI_BASE + off, pd->regulator[i]);
		wmb();
		off += 4;
	}

	off = 0x0260;	/* phy timig ctrl 0 */
	for (i = 0; i < 11; i++) {
		MIPI_OUTP(MIPI_DSI_BASE + off, pd->timing[i]);
		wmb();
		off += 4;
	}

	off = 0x0290;	/* ctrl 0 */
	for (i = 0; i < 4; i++) {
		MIPI_OUTP(MIPI_DSI_BASE + off, pd->ctrl[i]);
		wmb();
		off += 4;
	}

	off = 0x02a0;	/* strength 0 */
	for (i = 0; i < 4; i++) {
		MIPI_OUTP(MIPI_DSI_BASE + off, pd->strength[i]);
		wmb();
		off += 4;
	}

	off = 0x0204;	/* pll ctrl 1, skip 0 */
	for (i = 1; i < 21; i++) {
		MIPI_OUTP(MIPI_DSI_BASE + off, pd->pll[i]);
		wmb();
		off += 4;
	}

#ifndef CONFIG_FB_MSM_MDP303
	if (panel_info)
		mipi_dsi_phy_pll_config(panel_info->clk_rate);
#else
	MIPI_OUTP(MIPI_DSI_BASE + 0x100, 0x67);
#endif

	/* pll ctrl 0 */
	MIPI_OUTP(MIPI_DSI_BASE + 0x200, pd->pll[0]);
	wmb();
	MIPI_OUTP(MIPI_DSI_BASE + 0x200, (pd->pll[0] | 0x01));
}

void mipi_dsi_clk_enable(void)
{
#ifdef CONFIG_FB_MSM_MDP303
	unsigned data = 0;
#endif

	if (mipi_dsi_clk_on) {
		pr_err("%s: mipi_dsi_clk already ON\n", __func__);
		return;
	}

	mipi_dsi_clk_on = 1;

#ifndef CONFIG_FB_MSM_MDP303
	clk_enable(amp_pclk); /* clock for AHB-master to AXI */
	clk_enable(dsi_m_pclk);
	clk_enable(dsi_s_pclk);
#else
	clk_set_rate(dsi_byte_div_clk, data);
	clk_set_rate(dsi_esc_clk, data);
	clk_enable(mdp_dsi_pclk);
	clk_enable(ahb_m_clk);
	clk_enable(ahb_s_clk);
#endif
	clk_enable(dsi_byte_div_clk);
	clk_enable(dsi_esc_clk);
	mipi_dsi_pclk(&dsi_pclk, 1);
	mipi_dsi_clk(&dsicore_clk, 1);
#ifndef CONFIG_FB_MSM_MDP303
	mipi_dsi_ahb_en();
	mipi_dsi_sfpb_cfg();
#endif
}

void mipi_dsi_clk_disable(void)
{
	if (mipi_dsi_clk_on == 0) {
		pr_err("%s: mipi_dsi_clk already OFF\n", __func__);
		return;
	}

	mipi_dsi_clk_on = 0;

	MIPI_OUTP(MIPI_DSI_BASE + 0x0118, 0);

	mipi_dsi_pclk(&dsi_pclk, 0);
	mipi_dsi_clk(&dsicore_clk, 0);
	clk_disable(dsi_esc_clk);
	clk_disable(dsi_byte_div_clk);
#ifndef CONFIG_FB_MSM_MDP303
	clk_disable(dsi_m_pclk);
	clk_disable(dsi_s_pclk);
	clk_disable(amp_pclk); /* clock for AHB-master to AXI */
#else
	clk_disable(mdp_dsi_pclk);
	clk_disable(ahb_m_clk);
	clk_disable(ahb_s_clk);
#endif
}

static int mipi_dsi_off(struct platform_device *pdev)
{
	int ret = 0;
	struct msm_fb_data_type *mfd;
	struct msm_panel_info *pinfo;

	mfd = platform_get_drvdata(pdev);
	pinfo = &mfd->panel_info;

#ifndef CONFIG_FB_MSM_MDP303
	mdp4_overlay_dsi_state_set(ST_DSI_SUSPEND);
#endif

	/* change to DSI_CMD_MODE since it needed to
	 * tx DCS dsiplay off comamnd to panel
	 */
	mipi_dsi_op_mode_config(DSI_CMD_MODE);

	if (mfd->panel_info.type == MIPI_CMD_PANEL) {
		if (pinfo->lcd.vsync_enable) {
			if (pinfo->lcd.hw_vsync_mode && vsync_gpio > 0)
				gpio_free(vsync_gpio);

			mipi_dsi_set_tear_off(mfd);
		}
	}

	ret = panel_next_off(pdev);

#ifndef CONFIG_FB_MSM_MDP303
	mutex_lock(&mfd->dma->ov_mutex);

	/* make sure mdp dma is not running */
	if (mfd->panel_info.type == MIPI_CMD_PANEL)
		mdp4_dsi_cmd_dma_busy_wait(mfd);
#else
	down(&mfd->dma->mutex);
	/* make sure mdp dma is not running */
	mdp3_dsi_cmd_dma_busy_wait(mfd);
#endif


#ifdef CONFIG_MSM_BUS_SCALING
	mdp_bus_scale_update_request(0);
#endif

	/* DSIPHY_PLL_CTRL_5 */
	MIPI_OUTP(MIPI_DSI_BASE + 0x0214, 0x05f);

	/* DSIPHY_TPA_CTRL_1 */
	MIPI_OUTP(MIPI_DSI_BASE + 0x0258, 0x08f);

	/* DSIPHY_TPA_CTRL_2 */
	MIPI_OUTP(MIPI_DSI_BASE + 0x025c, 0x001);

	/* DSIPHY_REGULATOR_CTRL_0 */
	MIPI_OUTP(MIPI_DSI_BASE + 0x02cc, 0x02);

	/* DSIPHY_CTRL_0 */
	MIPI_OUTP(MIPI_DSI_BASE + 0x0290, 0x00);

	/* DSIPHY_CTRL_1 */
	MIPI_OUTP(MIPI_DSI_BASE + 0x0294, 0x7f);

	/* DSIPHY_PLL_CTRL_0, disbale dsi pll */
	MIPI_OUTP(MIPI_DSI_BASE + 0x0200, 0x40);

	/* disbale dsi clk */
	MIPI_OUTP(MIPI_DSI_BASE + 0x0118, 0);

	local_bh_disable();
	mipi_dsi_clk_disable();
	local_bh_enable();

	/* disbale dsi engine */
	MIPI_OUTP(MIPI_DSI_BASE + 0x0000, 0);

	if (mipi_dsi_pdata && mipi_dsi_pdata->dsi_power_save)
		mipi_dsi_pdata->dsi_power_save(0);

#ifndef CONFIG_FB_MSM_MDP303
	mutex_unlock(&mfd->dma->ov_mutex);
#else
	up(&mfd->dma->mutex);
#endif

	pr_debug("%s:\n", __func__);

	return ret;
}

static int mipi_dsi_on(struct platform_device *pdev)
{
	int ret = 0;
	u32 clk_rate;
	struct msm_fb_data_type *mfd;
	struct fb_info *fbi;
	struct fb_var_screeninfo *var;
	struct msm_panel_info *pinfo;
	struct mipi_panel_info *mipi;
	u32 hbp, hfp, vbp, vfp, hspw, vspw, width, height;
	u32 ystride, bpp, data;

	mfd = platform_get_drvdata(pdev);
	fbi = mfd->fbi;
	var = &fbi->var;
	pinfo = &mfd->panel_info;

#ifndef CONFIG_FB_MSM_MDP303
	mdp4_overlay_dsi_state_set(ST_DSI_RESUME);
#endif

	if (mipi_dsi_pdata && mipi_dsi_pdata->dsi_power_save)
		mipi_dsi_pdata->dsi_power_save(1);

	clk_rate = mfd->fbi->var.pixclock;
	clk_rate = min(clk_rate, mfd->panel_info.clk_max);

#ifndef CONFIG_FB_MSM_MDP303
	if (clk_set_rate(dsi_byte_div_clk, 1) < 0)	/* divided by 1 */
		pr_err("%s: clk_set_rate failed\n",	__func__);
#endif

	local_bh_disable();
	mipi_dsi_clk_enable();
	local_bh_enable();

	hbp = var->left_margin;
	hfp = var->right_margin;
	vbp = var->upper_margin;
	vfp = var->lower_margin;
	hspw = var->hsync_len;
	vspw = var->vsync_len;
	width = mfd->panel_info.xres;
	height = mfd->panel_info.yres;

	/* DSIPHY_PLL_CTRL_5 */
	MIPI_OUTP(MIPI_DSI_BASE + 0x0214, 0x050);

	/* DSIPHY_TPA_CTRL_1 */
	MIPI_OUTP(MIPI_DSI_BASE + 0x0258, 0x00f);

	/* DSIPHY_TPA_CTRL_2 */
	MIPI_OUTP(MIPI_DSI_BASE + 0x025c, 0x000);

	mipi_dsi_phy_init(0, &(mfd->panel_info));

	mipi  = &mfd->panel_info.mipi;
	if (mfd->panel_info.type == MIPI_VIDEO_PANEL) {
#ifndef CONFIG_FB_MSM_MDP303
		MIPI_OUTP(MIPI_DSI_BASE + 0x20,
			((hspw + hbp + width) << 16 | (hspw + hbp)));
		MIPI_OUTP(MIPI_DSI_BASE + 0x24,
			((vspw + vbp + height) << 16 | (vspw + vbp)));
		MIPI_OUTP(MIPI_DSI_BASE + 0x28,
			(vspw + vbp + height + vfp - 1) << 16 |
				(hspw + hbp + width + hfp - 1));
#else
	/* DSI_LAN_SWAP_CTRL */
		MIPI_OUTP(MIPI_DSI_BASE + 0x00ac, mipi->dlane_swap);

		MIPI_OUTP(MIPI_DSI_BASE + 0x20,
			((hbp + width) << 16 | (hbp)));
		MIPI_OUTP(MIPI_DSI_BASE + 0x24,
			((vbp + height) << 16 | (vbp)));
		MIPI_OUTP(MIPI_DSI_BASE + 0x28,
			(vbp + height + vfp) << 16 |
				(hbp + width + hfp));
#endif
		MIPI_OUTP(MIPI_DSI_BASE + 0x2c, (hspw << 16));
		MIPI_OUTP(MIPI_DSI_BASE + 0x30, 0);
		MIPI_OUTP(MIPI_DSI_BASE + 0x34, (vspw << 16));

	} else {		/* command mode */
		if (mipi->dst_format == DSI_CMD_DST_FORMAT_RGB888)
			bpp = 3;
		else if (mipi->dst_format == DSI_CMD_DST_FORMAT_RGB666)
			bpp = 3;
		else if (mipi->dst_format == DSI_CMD_DST_FORMAT_RGB565)
			bpp = 2;
		else
			bpp = 3;	/* Default format set to RGB888 */

		ystride = width * bpp + 1;

		/* DSI_COMMAND_MODE_MDP_STREAM_CTRL */
		data = (ystride << 16) | (mipi->vc << 8) | DTYPE_DCS_LWRITE;
		MIPI_OUTP(MIPI_DSI_BASE + 0x5c, data);
		MIPI_OUTP(MIPI_DSI_BASE + 0x54, data);

		/* DSI_COMMAND_MODE_MDP_STREAM_TOTAL */
		data = height << 16 | width;
		MIPI_OUTP(MIPI_DSI_BASE + 0x60, data);
		MIPI_OUTP(MIPI_DSI_BASE + 0x58, data);
	}

	mipi_dsi_host_init(mipi);

	if (mipi_dsi_pdata && mipi_dsi_pdata->target_type == 1)
		mipi_dsi_configure_serdes();

	mipi_dsi_cmd_bta_sw_trigger(); /* clean up ack_err_status */

	ret = panel_next_on(pdev);

	mipi_dsi_op_mode_config(mipi->mode);

	if (mfd->panel_info.type == MIPI_CMD_PANEL) {
		if (pinfo->lcd.vsync_enable) {
			if (pinfo->lcd.hw_vsync_mode && vsync_gpio > 0) {
				if (gpio_request(vsync_gpio, "MDP_VSYNC") == 0)
					gpio_direction_input(vsync_gpio);
				else
					pr_err("%s: unable to request gpio=%d\n",
						__func__, vsync_gpio);
			}
			mipi_dsi_set_tear_on(mfd);
		}
	}

#ifdef CONFIG_MSM_BUS_SCALING
	mdp_bus_scale_update_request(2);
#endif
	return ret;
}


static int mipi_dsi_resource_initialized;

static int mipi_dsi_probe(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;
	struct fb_info *fbi;
	struct msm_panel_info *pinfo;
	struct mipi_panel_info *mipi;
	struct platform_device *mdp_dev = NULL;
	struct msm_fb_panel_data *pdata = NULL;
	int rc;
	uint8 lanes = 0, bpp;
	uint32 h_period, v_period, dsi_pclk_rate;

	resource_size_t size ;

	if ((pdev->id == 0) && (pdev->num_resources >= 0)) {
		mipi_dsi_pdata = pdev->dev.platform_data;

		size =  resource_size(&pdev->resource[0]);
		mipi_dsi_base =  ioremap(pdev->resource[0].start, size);

		MSM_FB_INFO("mipi_dsi base phy_addr = 0x%x virt = 0x%x\n",
				pdev->resource[0].start, (int) mipi_dsi_base);

		if (!mipi_dsi_base)
			return -ENOMEM;

#ifndef CONFIG_FB_MSM_MDP303
		mmss_cc_base = MSM_MMSS_CLK_CTL_BASE;
		MSM_FB_INFO("msm_mmss_cc base = 0x%x\n",
				(int) mmss_cc_base);

		if (!mmss_cc_base)
			return -ENOMEM;

		mmss_sfpb_base =  ioremap(MMSS_SFPB_BASE_PHY, 0x100);
		MSM_FB_INFO("mmss_sfpb  base phy_addr = 0x%x virt = 0x%x\n",
				MMSS_SFPB_BASE_PHY, (int) mmss_sfpb_base);

		if (!mmss_sfpb_base)
			return -ENOMEM;
#endif

		dsi_irq = platform_get_irq(pdev, 0);
		if (dsi_irq < 0) {
			pr_err("mipi_dsi: can not get mdp irq\n");
			return -ENOMEM;
		}

		rc = request_irq(dsi_irq, mipi_dsi_isr, IRQF_DISABLED,
						"MIPI_DSI", 0);
		if (rc) {
			pr_err("mipi_dsi_host request_irq() failed!\n");
			return rc;
		}

		disable_irq(dsi_irq);
#ifndef CONFIG_FB_MSM_MDP303

		if (mipi_dsi_pdata && mipi_dsi_pdata->target_type == 1) {
			/* Target type is 1 for device with (De)serializer
			 * 0x4f00000 is the base for TV Encoder.
			 * Unused Offset 0x1000 is used for
			 * (de)serializer on emulation platform
			 */
			periph_base = ioremap(MMSS_SERDES_BASE_PHY, 0x100);

			if (periph_base) {
				pr_debug("periph_base %p\n", periph_base);
				writel(0x4, periph_base + 0x28);
				writel(0xc, periph_base + 0x28);
			} else {
				pr_err("periph_base is NULL\n");
				free_irq(dsi_irq, 0);
				return -ENOMEM;
			}
		}
#endif

		mipi_dsi_calibration();

		if (mipi_dsi_pdata) {
			vsync_gpio = mipi_dsi_pdata->vsync_gpio;
			pr_debug("%s: vsync_gpio=%d\n", __func__, vsync_gpio);
#ifdef CONFIG_FB_MSM_MDP303
			if (mipi_dsi_pdata->dsi_client_reset) {
				if (mipi_dsi_pdata->dsi_client_reset())
					pr_err("%s: DSI Client Reset failed!\n",
						__func__);
				else
					pr_debug("%s: DSI Client Reset success\n",
						__func__);
			}
#endif
		}

		mipi_dsi_resource_initialized = 1;

		return 0;
	}

	if (!mipi_dsi_resource_initialized)
		return -EPERM;

	mfd = platform_get_drvdata(pdev);

	if (!mfd)
		return -ENODEV;

	if (mfd->key != MFD_KEY)
		return -EINVAL;

	if (pdev_list_cnt >= MSM_FB_MAX_DEV_LIST)
		return -ENOMEM;


	mdp_dev = platform_device_alloc("mdp", pdev->id);
	if (!mdp_dev)
		return -ENOMEM;

	/*
	 * link to the latest pdev
	 */
	mfd->pdev = mdp_dev;
	mfd->dest = DISPLAY_LCD;

	/*
	 * alloc panel device data
	 */
	if (platform_device_add_data
	    (mdp_dev, pdev->dev.platform_data,
	     sizeof(struct msm_fb_panel_data))) {
		pr_err("mipi_dsi_probe: platform_device_add_data failed!\n");
		platform_device_put(mdp_dev);
		return -ENOMEM;
	}
	/*
	 * data chain
	 */
	pdata = mdp_dev->dev.platform_data;
	pdata->on = mipi_dsi_on;
	pdata->off = mipi_dsi_off;
	pdata->next = pdev;

	/*
	 * get/set panel specific fb info
	 */
	mfd->panel_info = pdata->panel_info;
	pinfo = &mfd->panel_info;

	if (mfd->index == 0)
		mfd->fb_imgType = MSMFB_DEFAULT_TYPE;
	else
		mfd->fb_imgType = MDP_RGB_565;

	fbi = mfd->fbi;
	fbi->var.pixclock = mfd->panel_info.clk_rate;
	fbi->var.left_margin = mfd->panel_info.lcdc.h_back_porch;
	fbi->var.right_margin = mfd->panel_info.lcdc.h_front_porch;
	fbi->var.upper_margin = mfd->panel_info.lcdc.v_back_porch;
	fbi->var.lower_margin = mfd->panel_info.lcdc.v_front_porch;
	fbi->var.hsync_len = mfd->panel_info.lcdc.h_pulse_width;
	fbi->var.vsync_len = mfd->panel_info.lcdc.v_pulse_width;

	h_period = ((mfd->panel_info.lcdc.h_pulse_width)
			+ (mfd->panel_info.lcdc.h_back_porch)
			+ (mfd->panel_info.xres)
			+ (mfd->panel_info.lcdc.h_front_porch));

	v_period = ((mfd->panel_info.lcdc.v_pulse_width)
			+ (mfd->panel_info.lcdc.v_back_porch)
			+ (mfd->panel_info.yres)
			+ (mfd->panel_info.lcdc.v_front_porch));

	mipi  = &mfd->panel_info.mipi;

	if (mipi->data_lane3)
		lanes += 1;
	if (mipi->data_lane2)
		lanes += 1;
	if (mipi->data_lane1)
		lanes += 1;
	if (mipi->data_lane0)
		lanes += 1;

	if ((mipi->dst_format == DSI_CMD_DST_FORMAT_RGB888)
	    || (mipi->dst_format == DSI_VIDEO_DST_FORMAT_RGB888)
	    || (mipi->dst_format == DSI_VIDEO_DST_FORMAT_RGB666_LOOSE))
		bpp = 3;
	else if ((mipi->dst_format == DSI_CMD_DST_FORMAT_RGB565)
		 || (mipi->dst_format == DSI_VIDEO_DST_FORMAT_RGB565))
		bpp = 2;
	else
		bpp = 3;		/* Default format set to RGB888 */

#ifndef CONFIG_FB_MSM_MDP303
	if (mfd->panel_info.type == MIPI_VIDEO_PANEL) {
		if (lanes > 0) {
			pll_divider_config.clk_rate =
			((h_period * v_period * (mipi->frame_rate) * bpp * 8)
			   / lanes);
		} else {
			pr_err("%s: forcing mipi_dsi lanes to 1\n", __func__);
			pll_divider_config.clk_rate =
				(h_period * v_period
					 * (mipi->frame_rate) * bpp * 8);
		}
	} else
#endif
		pll_divider_config.clk_rate = mfd->panel_info.clk_rate;

	rc = mipi_dsi_clk_div_config(bpp, lanes, &dsi_pclk_rate);
	if (rc)
		goto mipi_dsi_probe_err;

	if ((dsi_pclk_rate < 3300000) || (dsi_pclk_rate > 103300000))
		dsi_pclk_rate = 35000000;
	mipi->dsi_pclk_rate = dsi_pclk_rate;

#ifdef DSI_CLK
	clk_rate = mfd->panel_info.clk_max;
	if (clk_set_max_rate(mipi_dsi_clk, clk_rate) < 0)
		pr_err("%s: clk_set_max_rate failed\n", __func__);
	mfd->panel_info.clk_rate = mfd->panel_info.clk_min;
#endif

	/*
	 * set driver data
	 */
	platform_set_drvdata(mdp_dev, mfd);

	/*
	 * register in mdp driver
	 */
	rc = platform_device_add(mdp_dev);
	if (rc)
		goto mipi_dsi_probe_err;

	pdev_list[pdev_list_cnt++] = pdev;

#ifdef CONFIG_FB_MSM_MDP303
	if (clk_enable(dsi_ref_clk))
		pr_err("%s: dsi_ref_clk clk enable fail\n", __func__);
#endif
	return 0;

mipi_dsi_probe_err:
	platform_device_put(mdp_dev);
	return rc;
}

static int mipi_dsi_remove(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;

	mfd = platform_get_drvdata(pdev);
	iounmap(mipi_dsi_base);
	return 0;
}

static int mipi_dsi_register_driver(void)
{
	return platform_driver_register(&mipi_dsi_driver);
}

static int __init mipi_dsi_driver_init(void)
{
	int ret;

#ifndef CONFIG_FB_MSM_MDP303
	amp_pclk = clk_get(NULL, "amp_pclk");
	if (IS_ERR(amp_pclk)) {
		pr_err("can't find amp_pclk\n");
		return PTR_ERR(amp_pclk);
	}

	dsi_m_pclk = clk_get(NULL, "dsi_m_pclk");
	if (IS_ERR(dsi_m_pclk)) {
		pr_err("can't find dsi_m_pclk\n");
		return PTR_ERR(dsi_m_pclk);
	}

	dsi_s_pclk = clk_get(NULL, "dsi_s_pclk");
	if (IS_ERR(dsi_s_pclk)) {
		pr_err("can't find dsi_s_pclk\n");
		return PTR_ERR(dsi_s_pclk);
	}

	dsi_byte_div_clk = clk_get(NULL, "dsi_byte_div_clk");
	if (IS_ERR(dsi_byte_div_clk)) {
		pr_err("can't find dsi_byte_div_clk\n");
		return PTR_ERR(dsi_byte_div_clk);
	}
#endif

	dsi_esc_clk = clk_get(NULL, "dsi_esc_clk");
	if (IS_ERR(dsi_esc_clk)) {
		printk(KERN_ERR "can't find dsi_esc_clk\n");
		return PTR_ERR(dsi_esc_clk);
	}

#ifdef CONFIG_FB_MSM_MDP303
	dsi_byte_div_clk = clk_get(NULL, "dsi_byte_clk");
	if (IS_ERR(dsi_byte_div_clk)) {
		pr_err("can't find dsi_byte_div_clk\n");
		return PTR_ERR(dsi_byte_div_clk);
	}

	dsi_pixel_clk = clk_get(NULL, "dsi_pixel_clk");
	if (IS_ERR(dsi_pixel_clk)) {
		pr_err("can't find dsi_pixel_clk\n");
		return PTR_ERR(dsi_pixel_clk);
	}

	dsi_clk = clk_get(NULL, "dsi_clk");
	if (IS_ERR(dsi_clk)) {
		pr_err("can't find dsi_clk\n");
		return PTR_ERR(dsi_clk);
	}

	dsi_ref_clk = clk_get(NULL, "dsi_ref_clk");
	if (IS_ERR(dsi_ref_clk)) {
		pr_err("can't find dsi_ref_clk\n");
		return PTR_ERR(dsi_ref_clk);
	}

	mdp_dsi_pclk = clk_get(NULL, "mdp_dsi_pclk");
	if (IS_ERR(mdp_dsi_pclk)) {
		pr_err("can't find mdp_dsi_pclk\n");
		return PTR_ERR(mdp_dsi_pclk);
	}

	ahb_m_clk = clk_get(NULL, "ahb_m_clk");
	if (IS_ERR(ahb_m_clk)) {
		pr_err("can't find ahb_m_clk\n");
		return PTR_ERR(ahb_m_clk);
	}

	ahb_s_clk = clk_get(NULL, "ahb_s_clk");
	if (IS_ERR(ahb_s_clk)) {
		pr_err("can't find ahb_s_clk\n");
		return PTR_ERR(ahb_s_clk);
	}
#endif
	mipi_dsi_init();

	ret = mipi_dsi_register_driver();

	device_initialize(&dsi_dev);

	if (ret) {
#ifndef CONFIG_FB_MSM_MDP303
		clk_disable(amp_pclk);
		clk_put(amp_pclk);
		clk_disable(dsi_m_pclk);
		clk_put(dsi_m_pclk);
		clk_disable(dsi_s_pclk);
		clk_put(dsi_s_pclk);
#else
		clk_disable(mdp_dsi_pclk);
		clk_put(mdp_dsi_pclk);
		clk_disable(ahb_m_clk);
		clk_put(ahb_m_clk);
		clk_disable(ahb_s_clk);
		clk_put(ahb_s_clk);
		clk_disable(dsi_ref_clk);
		clk_put(dsi_ref_clk);
#endif
		clk_disable(dsi_byte_div_clk);
		clk_put(dsi_byte_div_clk);
		clk_disable(dsi_esc_clk);
		clk_put(dsi_esc_clk);
		pr_err("mipi_dsi_register_driver() failed!\n");
		return ret;
	}

	return ret;
}

module_init(mipi_dsi_driver_init);
