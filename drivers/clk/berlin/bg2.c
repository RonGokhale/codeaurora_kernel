/*
 * Copyright (c) 2014 Marvell Technology Group Ltd.
 *
 * Sebastian Hesselbarth <sebastian.hesselbarth@gmail.com>
 * Alexandre Belloni <alexandre.belloni@free-electrons.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/slab.h>

#include "berlin2-div.h"

/*
 * BG2/BG2CD SoCs have the following audio/video I/O units:
 *
 * audiohd: HDMI TX audio
 * audio0:  7.1ch TX
 * audio1:  2ch TX
 * audio2:  2ch RX
 * audio3:  SPDIF TX
 * video0:  HDMI video
 * video1:  Secondary video
 * video2:  SD auxiliary video
 *
 * There are no external audio clocks (ACLKI0, ACLKI1) and
 * only one external video clock (VCLKI0).
 *
 * Currently missing bits and pieces:
 * - audio_fast_pll is unknown
 * - audiohd_pll is unknown
 * - video0_pll is unknown
 * - audio[023], audiohd parent pll is assumed to be audio_fast_pll
 *
 */

struct bg2_gate_data {
	const char *name;
	const char *parent_name;
	u8 bit_idx;
	unsigned long flags;
};

#define REG_CLKENABLE		0x00
#define REG_CLKSELECT0		0x04
#define REG_CLKSELECT1		0x08
#define REG_CLKSELECT2		0x0c
#define REG_CLKSELECT3		0x10
#define REG_CLKSWITCH0		0x14
#define REG_CLKSWITCH1		0x18

enum {
	/* clock divider cells */
	SYS, CPU, DRMFIGO, CFG, GFX, ZSP, PERIF, PCUBE, VSCOPE, NFC_ECC,
	VPP, APP, AUDIO0, AUDIO2, AUDIO3, AUDIO1,
	/* clock gates */
	GETH0, GETH1, SATA, AHBAPB, USB0, USB1, PBRIDGE, SDIO0, SDIO1,
	NFC, SMEMC, AUDIOHD, VIDEO0, VIDEO1, VIDEO2,
	MAX_CLKS
};

enum { REFCLK, SYSPLL, MEMPLL, CPUPLL };
static const char *refclk_names[] = { "refclk", "syspll", "mempll", "cpupll" };

enum { CH1, CH2, CH3, CH4, CH5, CH6, CH7, CH8 };
static const char *avplla_names[] = {
	"avpll_a1", "avpll_a2", "avpll_a3", "avpll_a4",
	"avpll_a5", "avpll_a6", "avpll_a7", "avpll_a8"
};
static const char *avpllb_names[] = {
	"avpll_b1", "avpll_b2", "avpll_b3", "avpll_b4",
	"avpll_b5", "avpll_b6", "avpll_b7", "avpll_a8"
};
static const char *video_ext0_name = "video_ext0";

static DEFINE_SPINLOCK(lock);
static struct clk *clks[MAX_CLKS];
static struct clk_onecell_data clk_data;

static const struct berlin2_div_data bg2_divs[] __initconst = {
	{
		.name = "sys",
		.flags = CLK_IGNORE_UNUSED,
		.map = {
			BERLIN2_DIV_GATE(REG_CLKENABLE, 0),
			BERLIN2_PLL_SELECT(REG_CLKSELECT0, 0),
			BERLIN2_DIV_SELECT(REG_CLKSELECT0, 3),
			BERLIN2_PLL_SWITCH(REG_CLKSWITCH0, 3),
			BERLIN2_DIV_SWITCH(REG_CLKSWITCH0, 4),
			BERLIN2_DIV_D3SWITCH(REG_CLKSWITCH0, 5),
		},
		.div_flags = BERLIN2_DIV_HAS_GATE | BERLIN2_DIV_HAS_MUX,
	},
	{
		.name = "cpu",
		.flags = 0,
		.map = {
			BERLIN2_PLL_SELECT(REG_CLKSELECT0, 6),
			BERLIN2_DIV_SELECT(REG_CLKSELECT0, 9),
			BERLIN2_PLL_SWITCH(REG_CLKSWITCH0, 6),
			BERLIN2_DIV_SWITCH(REG_CLKSWITCH0, 7),
			BERLIN2_DIV_D3SWITCH(REG_CLKSWITCH0, 8),
		},
		.div_flags = BERLIN2_DIV_HAS_MUX,
	},
	{
		.name = "drmfigo",
		.flags = 0,
		.map = {
			BERLIN2_DIV_GATE(REG_CLKENABLE, 16),
			BERLIN2_PLL_SELECT(REG_CLKSELECT0, 17),
			BERLIN2_DIV_SELECT(REG_CLKSELECT0, 20),
			BERLIN2_PLL_SWITCH(REG_CLKSWITCH0, 12),
			BERLIN2_DIV_SWITCH(REG_CLKSWITCH0, 13),
			BERLIN2_DIV_D3SWITCH(REG_CLKSWITCH0, 14),
		},
		.div_flags = BERLIN2_DIV_HAS_GATE | BERLIN2_DIV_HAS_MUX,
	},
	{
		.name = "cfg",
		.flags = 0,
		.map = {
			BERLIN2_DIV_GATE(REG_CLKENABLE, 1),
			BERLIN2_PLL_SELECT(REG_CLKSELECT0, 23),
			BERLIN2_DIV_SELECT(REG_CLKSELECT0, 26),
			BERLIN2_PLL_SWITCH(REG_CLKSWITCH0, 15),
			BERLIN2_DIV_SWITCH(REG_CLKSWITCH0, 16),
			BERLIN2_DIV_D3SWITCH(REG_CLKSWITCH0, 17),
		},
		.div_flags = BERLIN2_DIV_HAS_GATE | BERLIN2_DIV_HAS_MUX,
	},
	{
		.name = "gfx",
		.flags = 0,
		.map = {
			BERLIN2_DIV_GATE(REG_CLKENABLE, 4),
			BERLIN2_PLL_SELECT(REG_CLKSELECT0, 29),
			BERLIN2_DIV_SELECT(REG_CLKSELECT1, 0),
			BERLIN2_PLL_SWITCH(REG_CLKSWITCH0, 18),
			BERLIN2_DIV_SWITCH(REG_CLKSWITCH0, 19),
			BERLIN2_DIV_D3SWITCH(REG_CLKSWITCH0, 20),
		},
		.div_flags = BERLIN2_DIV_HAS_GATE | BERLIN2_DIV_HAS_MUX,
	},
	{
		.name = "zsp",
		.flags = 0,
		.map = {
			BERLIN2_DIV_GATE(REG_CLKENABLE, 5),
			BERLIN2_PLL_SELECT(REG_CLKSELECT1, 3),
			BERLIN2_DIV_SELECT(REG_CLKSELECT1, 6),
			BERLIN2_PLL_SWITCH(REG_CLKSWITCH0, 21),
			BERLIN2_DIV_SWITCH(REG_CLKSWITCH0, 22),
			BERLIN2_DIV_D3SWITCH(REG_CLKSWITCH0, 23),
		},
		.div_flags = BERLIN2_DIV_HAS_GATE | BERLIN2_DIV_HAS_MUX,
	},
	{
		.name = "perif",
		.flags = CLK_IGNORE_UNUSED,
		.map = {
			BERLIN2_DIV_GATE(REG_CLKENABLE, 6),
			BERLIN2_PLL_SELECT(REG_CLKSELECT1, 9),
			BERLIN2_DIV_SELECT(REG_CLKSELECT1, 12),
			BERLIN2_PLL_SWITCH(REG_CLKSWITCH0, 24),
			BERLIN2_DIV_SWITCH(REG_CLKSWITCH0, 25),
			BERLIN2_DIV_D3SWITCH(REG_CLKSWITCH0, 26),
		},
		.div_flags = BERLIN2_DIV_HAS_GATE | BERLIN2_DIV_HAS_MUX,
	},
	{
		.name = "pcube",
		.flags = 0,
		.map = {
			BERLIN2_DIV_GATE(REG_CLKENABLE, 2),
			BERLIN2_PLL_SELECT(REG_CLKSELECT1, 15),
			BERLIN2_DIV_SELECT(REG_CLKSELECT1, 18),
			BERLIN2_PLL_SWITCH(REG_CLKSWITCH0, 27),
			BERLIN2_DIV_SWITCH(REG_CLKSWITCH0, 28),
			BERLIN2_DIV_D3SWITCH(REG_CLKSWITCH0, 29),
		},
		.div_flags = BERLIN2_DIV_HAS_GATE | BERLIN2_DIV_HAS_MUX,
	},
	{
		.name = "vscope",
		.flags = 0,
		.map = {
			BERLIN2_DIV_GATE(REG_CLKENABLE, 3),
			BERLIN2_PLL_SELECT(REG_CLKSELECT1, 21),
			BERLIN2_DIV_SELECT(REG_CLKSELECT1, 24),
			BERLIN2_PLL_SWITCH(REG_CLKSWITCH0, 30),
			BERLIN2_DIV_SWITCH(REG_CLKSWITCH0, 31),
			BERLIN2_DIV_D3SWITCH(REG_CLKSWITCH1, 0),
		},
		.div_flags = BERLIN2_DIV_HAS_GATE | BERLIN2_DIV_HAS_MUX,
	},
	{
		.name = "nfc_ecc",
		.flags = 0,
		.map = {
			BERLIN2_DIV_GATE(REG_CLKENABLE, 18),
			BERLIN2_PLL_SELECT(REG_CLKSELECT1, 27),
			BERLIN2_DIV_SELECT(REG_CLKSELECT2, 0),
			BERLIN2_PLL_SWITCH(REG_CLKSWITCH1, 1),
			BERLIN2_DIV_SWITCH(REG_CLKSWITCH1, 2),
			BERLIN2_DIV_D3SWITCH(REG_CLKSWITCH1, 3),
		},
		.div_flags = BERLIN2_DIV_HAS_GATE | BERLIN2_DIV_HAS_MUX,
	},
	{
		.name = "vpp",
		.flags = 0,
		.map = {
			BERLIN2_DIV_GATE(REG_CLKENABLE, 21),
			BERLIN2_PLL_SELECT(REG_CLKSELECT2, 3),
			BERLIN2_DIV_SELECT(REG_CLKSELECT2, 6),
			BERLIN2_PLL_SWITCH(REG_CLKSWITCH1, 4),
			BERLIN2_DIV_SWITCH(REG_CLKSWITCH1, 5),
			BERLIN2_DIV_D3SWITCH(REG_CLKSWITCH1, 6),
		},
		.div_flags = BERLIN2_DIV_HAS_GATE | BERLIN2_DIV_HAS_MUX,
	},
	{
		.name = "app",
		.flags = 0,
		.map = {
			BERLIN2_DIV_GATE(REG_CLKENABLE, 20),
			BERLIN2_PLL_SELECT(REG_CLKSELECT2, 9),
			BERLIN2_DIV_SELECT(REG_CLKSELECT2, 12),
			BERLIN2_PLL_SWITCH(REG_CLKSWITCH1, 7),
			BERLIN2_DIV_SWITCH(REG_CLKSWITCH1, 8),
			BERLIN2_DIV_D3SWITCH(REG_CLKSWITCH1, 9),
		},
		.div_flags = BERLIN2_DIV_HAS_GATE | BERLIN2_DIV_HAS_MUX,
	},
	{
		.name = "audio0",
		.flags = 0,
		.map = {
			BERLIN2_DIV_GATE(REG_CLKENABLE, 22),
			BERLIN2_DIV_SELECT(REG_CLKSELECT2, 17),
			BERLIN2_DIV_SWITCH(REG_CLKSWITCH1, 10),
			BERLIN2_DIV_D3SWITCH(REG_CLKSWITCH1, 11),
		},
		.div_flags = BERLIN2_DIV_HAS_GATE,
	},
	{
		.name = "audio2",
		.flags = 0,
		.map = {
			BERLIN2_DIV_GATE(REG_CLKENABLE, 24),
			BERLIN2_DIV_SELECT(REG_CLKSELECT2, 20),
			BERLIN2_DIV_SWITCH(REG_CLKSWITCH1, 14),
			BERLIN2_DIV_D3SWITCH(REG_CLKSWITCH1, 15),
		},
		.div_flags = BERLIN2_DIV_HAS_GATE,
	},
	{
		.name = "audio3",
		.flags = 0,
		.map = {
			BERLIN2_DIV_GATE(REG_CLKENABLE, 25),
			BERLIN2_DIV_SELECT(REG_CLKSELECT2, 23),
			BERLIN2_DIV_SWITCH(REG_CLKSWITCH1, 16),
			BERLIN2_DIV_D3SWITCH(REG_CLKSWITCH1, 17),
		},
		.div_flags = BERLIN2_DIV_HAS_GATE,
	},
	{
		.name = "audio1",
		.flags = 0,
		.map = {
			BERLIN2_DIV_GATE(REG_CLKENABLE, 23),
			BERLIN2_DIV_SELECT(REG_CLKSELECT3, 0),
			BERLIN2_DIV_SWITCH(REG_CLKSWITCH1, 12),
			BERLIN2_DIV_D3SWITCH(REG_CLKSWITCH1, 13),
		},
		.div_flags = BERLIN2_DIV_HAS_GATE,
	},
};

static const struct bg2_gate_data bg2_gates[] __initconst = {
	{ "geth0",	"perif",	7 },
	{ "geth1",	"perif",	8 },
	{ "sata",	"perif",	9 },
	{ "ahbapb",	"perif",	10, CLK_IGNORE_UNUSED },
	{ "usb0",	"perif",	11 },
	{ "usb1",	"perif",	12 },
	{ "pbridge",	"perif",	13, CLK_IGNORE_UNUSED },
	{ "sdio0",	"perif",	14 },
	{ "sdio1",	"perif",	15 },
	{ "nfc",	"perif",	17 },
	{ "smemc",	"perif",	19 },
	{ "audiohd",	"audiohd_pll",	26 },
	{ "video0",	"video0_in",	27 },
	{ "video1",	"video1_in",	28 },
	{ "video2",	"video2_in",	29 },
};

static int __init collect_refclks(struct device_node *np)
{
	struct clk *iclk;
	int n;

	/* overwrite default clock names with DT provided ones */
	/* reference clocks */
	for (n = 0; n < ARRAY_SIZE(refclk_names); n++) {
		iclk = of_clk_get_by_name(np, refclk_names[n]);
		if (!IS_ERR(iclk)) {
			refclk_names[n] = __clk_get_name(iclk);
			clk_put(iclk);
		}
	}

	/* AVPLL_A inputs */
	for (n = 0; n < ARRAY_SIZE(avplla_names); n++) {
		iclk = of_clk_get_by_name(np, avplla_names[n]);
		if (!IS_ERR(iclk)) {
			avplla_names[n] = __clk_get_name(iclk);
			clk_put(iclk);
		}
	}

	/* AVPLL_B inputs */
	for (n = 0; n < ARRAY_SIZE(avpllb_names); n++) {
		iclk = of_clk_get_by_name(np, avpllb_names[n]);
		if (!IS_ERR(iclk)) {
			avpllb_names[n] = __clk_get_name(iclk);
			clk_put(iclk);
		}
	}

	/* video_ext0 input */
	iclk = of_clk_get_by_name(np, video_ext0_name);
	if (!IS_ERR(iclk)) {
		video_ext0_name = __clk_get_name(iclk);
		clk_put(iclk);
	}

	return 0;
}

static void __init berlin2_core_clock_of_setup(struct device_node *np)
{
	const struct berlin2_div_data *data;
	const char *parent_names[9];
	void __iomem *base;
	struct clk *clk;
	int n;

	if (collect_refclks(np))
		return;

	base = of_iomap(np, 0);
	if (!base) {
		pr_err("%s: Unable to map register base\n", np->full_name);
		return;
	}

	/* reference clock bypass switches */
	parent_names[0] = refclk_names[SYSPLL];
	parent_names[1] = refclk_names[REFCLK];
	clk = clk_register_mux(NULL, "syspll_in", parent_names, 2, 0,
			       base + REG_CLKSWITCH0, 0, 1, 0, &lock);
	if (IS_ERR(clk))
		goto core_clock_fail;
	refclk_names[SYSPLL] = __clk_get_name(clk);

	parent_names[0] = refclk_names[MEMPLL];
	parent_names[1] = refclk_names[REFCLK];
	clk = clk_register_mux(NULL, "mempll_in", parent_names, 2, 0,
			       base + REG_CLKSWITCH0, 1, 1, 0, &lock);
	if (IS_ERR(clk))
		goto core_clock_fail;
	refclk_names[MEMPLL] = __clk_get_name(clk);

	parent_names[0] = refclk_names[CPUPLL];
	parent_names[1] = refclk_names[REFCLK];
	clk = clk_register_mux(NULL, "cpupll_in", parent_names, 2, 0,
			       base + REG_CLKSWITCH0, 2, 1, 0, &lock);
	if (IS_ERR(clk))
		goto core_clock_fail;
	refclk_names[CPUPLL] = __clk_get_name(clk);

	/* clock muxes */
	parent_names[0] = avpllb_names[CH3];
	parent_names[1] = avplla_names[CH3];
	clk = clk_register_mux(NULL, "audio1_pll", parent_names, 2, 0,
			       base + REG_CLKSELECT2, 29, 1, 0, &lock);
	if (IS_ERR(clk))
		goto core_clock_fail;

	parent_names[0] = "video0_pll";
	parent_names[1] = video_ext0_name;
	clk = clk_register_mux(NULL, "video0_in", parent_names, 2, 0,
			       base + REG_CLKSELECT3, 4, 1, 0, &lock);
	if (IS_ERR(clk))
		goto core_clock_fail;

	parent_names[0] = "video1_pll";
	parent_names[1] = video_ext0_name;
	clk = clk_register_mux(NULL, "video1_in", parent_names, 2, 0,
			       base + REG_CLKSELECT3, 6, 1, 0, &lock);
	if (IS_ERR(clk))
		goto core_clock_fail;

	parent_names[0] = avplla_names[CH2];
	parent_names[1] = avpllb_names[CH2];
	clk = clk_register_mux(NULL, "video1_pll", parent_names, 2, 0,
			       base + REG_CLKSELECT3, 7, 1, 0, &lock);
	if (IS_ERR(clk))
		goto core_clock_fail;

	parent_names[0] = "video2_pll";
	parent_names[1] = video_ext0_name;
	clk = clk_register_mux(NULL, "video2_in", parent_names, 2, 0,
			       base + REG_CLKSELECT3, 9, 1, 0, &lock);
	if (IS_ERR(clk))
		goto core_clock_fail;

	parent_names[0] = avpllb_names[CH1];
	parent_names[1] = avplla_names[CH5];
	clk = clk_register_mux(NULL, "video2_pll", parent_names, 2, 0,
			       base + REG_CLKSELECT3, 10, 1, 0, &lock);
	if (IS_ERR(clk))
		goto core_clock_fail;

	/* clock divider cells */
	parent_names[1] = avpllb_names[CH4];
	parent_names[2] = avpllb_names[CH5];
	parent_names[3] = avpllb_names[CH6];
	parent_names[4] = avpllb_names[CH7];

	parent_names[0] = refclk_names[SYSPLL];
	data = &bg2_divs[SYS];
	clks[SYS] = berlin2_div_register(&data->map, base, data->name,
			 data->div_flags, parent_names, 5, data->flags, &lock);

	parent_names[0] = refclk_names[CPUPLL];
	parent_names[5] = refclk_names[MEMPLL];
	data = &bg2_divs[CPU];
	clks[CPU] = berlin2_div_register(&data->map, base, data->name,
			 data->div_flags, parent_names, 6, data->flags, &lock);

	parent_names[0] = refclk_names[SYSPLL];
	for (n = DRMFIGO; n <= APP; n++) {
		data = &bg2_divs[n];
		clks[n] = berlin2_div_register(&data->map, base, data->name,
			 data->div_flags, parent_names, 5, data->flags, &lock);
	}

	parent_names[0] = "audio_fast_pll";
	for (n = AUDIO0; n <= AUDIO3; n++) {
		data = &bg2_divs[n];
		clks[n] = berlin2_div_register(&data->map, base, data->name,
			 data->div_flags, parent_names, 1, data->flags, &lock);
	}

	parent_names[0] = "audio1_pll";
	data = &bg2_divs[AUDIO1];
	clks[AUDIO1] = berlin2_div_register(&data->map, base, data->name,
			 data->div_flags, parent_names, 1, data->flags, &lock);

	/* clock gate cells */
	for (n = 0; n < ARRAY_SIZE(bg2_gates); n++) {
		const struct bg2_gate_data *gd = &bg2_gates[n];

		clks[GETH0 + n] = clk_register_gate(NULL, gd->name,
			    gd->parent_name, gd->flags, base + REG_CLKENABLE,
			    gd->bit_idx, 0, &lock);
	}

	/* check for errors on leaf clocks */
	for (n = 0; n < MAX_CLKS; n++) {
		if (!IS_ERR(clks[n]))
			continue;

		pr_err("%s: Unable to register leaf clock %d\n",
		       np->full_name, n);
		goto core_clock_fail;
	}

	/* register clk-provider */
	clk_data.clks = clks;
	clk_data.clk_num = MAX_CLKS;
	of_clk_add_provider(np, of_clk_src_onecell_get, &clk_data);
	return;

core_clock_fail:
	iounmap(base);
}
CLK_OF_DECLARE(berlin2_coreclk, "marvell,berlin2-core-clocks",
	       berlin2_core_clock_of_setup);
