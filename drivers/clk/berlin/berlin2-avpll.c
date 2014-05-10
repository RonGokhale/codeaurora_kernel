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
#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/slab.h>

/*
 * Berlin2 AVPLL comprises two PLLs (VCOs) with 8 channels each,
 * channel 8 is the odd-one-out and does not provide mul/div.
 *
 * Unfortunately, its registers are just numbered from 0-63. To
 * get in at least some kind of structure, we split both VCOs and
 * each of the channels into separate clock drivers.
 *
 * Also, here and there the VCO registers are a bit different with
 * respect to bit shifts. Make sure to add a comment for those.
 */
#define NUM_VCOS	2
#define NUM_CHANNELS	8

#define AVPLL_CTRL(x)		((x) * 0x4)
/* Second VCO starts at AVPLL_CTRL31 */
#define VCO_OFFSET		AVPLL_CTRL(31)

#define VCO_CTRL0		AVPLL_CTRL(0)
/* VCO_B has an additional shift of 4 for its VCO_CTRL0 reg */
#define  VCO_RESET		BIT(0)
#define  VCO_POWERUP		BIT(1)
#define  VCO_INTERPOL_SHIFT	2
#define  VCO_INTERPOL_MASK	(0xf << VCO_INTERPOL_SHIFT)
#define  VCO_REG1V45_SEL_SHIFT	6
#define  VCO_REG1V45_SEL(x)	((x) << VCO_REG1V45_SEL_SHIFT)
#define  VCO_REG1V45_SEL_1V40	VCO_REG1V45_SEL(0)
#define  VCO_REG1V45_SEL_1V45	VCO_REG1V45_SEL(1)
#define  VCO_REG1V45_SEL_1V50	VCO_REG1V45_SEL(2)
#define  VCO_REG1V45_SEL_1V55	VCO_REG1V45_SEL(3)
#define  VCO_REG1V45_SEL_MASK	VCO_REG1V45_SEL(3)
#define  VCO_REG0V9_SEL_SHIFT	8
#define  VCO_REG0V9_SEL_MASK	(0xf << VCO_REG0V9_SEL_SHIFT)
#define  VCO_VTHCAL_SHIFT	12
#define  VCO_VTHCAL(x)		((x) << VCO_VTHCAL_SHIFT)
#define  VCO_VTHCAL_0V90	VCO_VTHCAL(0)
#define  VCO_VTHCAL_0V95	VCO_VTHCAL(1)
#define  VCO_VTHCAL_1V00	VCO_VTHCAL(2)
#define  VCO_VTHCAL_1V05	VCO_VTHCAL(3)
#define  VCO_VTHCAL_MASK	VCO_VTHCAL(3)
#define  VCO_KVCOEXT_SHIFT	14
#define  VCO_KVCOEXT_MASK	(0x3 << VCO_KVCOEXT_SHIFT)
#define  VCO_KVCOEXT_ENABLE	BIT(17)
#define  VCO_V2IEXT_SHIFT	18
#define  VCO_V2IEXT_MASK	(0xf << VCO_V2IEXT_SHIFT)
#define  VCO_V2IEXT_ENABLE	BIT(22)
#define  VCO_SPEED_SHIFT	23
#define  VCO_SPEED(x)		((x) << VCO_SPEED_SHIFT)
#define  VCO_SPEED_1G08_1G21	VCO_SPEED(0)
#define  VCO_SPEED_1G21_1G40	VCO_SPEED(1)
#define  VCO_SPEED_1G40_1G61	VCO_SPEED(2)
#define  VCO_SPEED_1G61_1G86	VCO_SPEED(3)
#define  VCO_SPEED_1G86_2G00	VCO_SPEED(4)
#define  VCO_SPEED_2G00_2G22	VCO_SPEED(5)
#define  VCO_SPEED_2G22		VCO_SPEED(6)
#define  VCO_SPEED_MASK		VCO_SPEED(0x7)
#define  VCO_CLKDET_ENABLE	BIT(26)
#define VCO_CTRL1		AVPLL_CTRL(1)
#define  VCO_REFDIV_SHIFT	0
#define  VCO_REFDIV(x)		((x) << VCO_REFDIV_SHIFT)
#define  VCO_REFDIV_1		VCO_REFDIV(0)
#define  VCO_REFDIV_2		VCO_REFDIV(1)
#define  VCO_REFDIV_4		VCO_REFDIV(2)
#define  VCO_REFDIV_3		VCO_REFDIV(3)
#define  VCO_REFDIV_MASK	VCO_REFDIV(0x3f)
#define  VCO_FBDIV_SHIFT	6
#define  VCO_FBDIV(x)		((x) << VCO_FBDIV_SHIFT)
#define  VCO_FBDIV_MASK		VCO_FBDIV(0xff)
#define  VCO_ICP_SHIFT		14
/* PLL Charge Pump Current = 10uA * (x + 1) */
#define  VCO_ICP(x)		((x) << VCO_ICP_SHIFT)
#define  VCO_ICP_MASK		VCO_ICP(0xf)
#define  VCO_LOAD_CAP		BIT(18)
#define  VCO_CALIBRATION_START	BIT(19)
#define VCO_FREQOFFSETn(x)	AVPLL_CTRL(3 + (x))
#define  VCO_FREQOFFSET_MASK	0x7ffff
#define VCO_CTRL11		AVPLL_CTRL(11)
#define VCO_CTRL12		AVPLL_CTRL(12)
#define VCO_CTRL13		AVPLL_CTRL(13)
#define VCO_CTRL14		AVPLL_CTRL(14)
#define VCO_CTRL15		AVPLL_CTRL(15)
#define VCO_SYNC1n(x)		AVPLL_CTRL(15 + (x))
#define  VCO_SYNC1_MASK		0x1ffff
#define VCO_SYNC2n(x)		AVPLL_CTRL(23 + (x))
#define  VCO_SYNC2_MASK		0x1ffff
#define VCO_CTRL30		AVPLL_CTRL(30)
#define  VCO_DPLL_CH1_ENABLE	BIT(17)

struct avpll_vco;
struct avpll;

struct avpll_channel {
	struct clk_hw hw;
	struct avpll_vco *vco;
	u8 index;
};

struct avpll_vco {
	struct clk_hw hw;
	struct avpll_channel channel[NUM_CHANNELS];
	struct clk *clks[NUM_CHANNELS];
	void __iomem *base;
	struct avpll *pll;
	u8 index;
};

struct avpll {
	struct avpll_vco vco[NUM_VCOS];
	void __iomem *base;
};

#define to_avpll_vco(hw) container_of(hw, struct avpll_vco, hw)
#define to_avpll_channel(hw) container_of(hw, struct avpll_channel, hw)

static u8 vco_refdiv[] = { 1, 2, 4, 3 };

static unsigned long
avpll_vco_recalc_rate(struct clk_hw *hw, unsigned long parent_rate)
{
	struct avpll_vco *vco = to_avpll_vco(hw);
	u32 reg, refdiv;
	u64 freq;

	/* AVPLL VCO frequency: Fvco = (Fref / refdiv) * FBdiv */
	reg = readl_relaxed(vco->base + VCO_CTRL1);
	refdiv = (reg & VCO_REFDIV_MASK) >> VCO_REFDIV_SHIFT;
	refdiv = vco_refdiv[refdiv];
	freq = (reg & VCO_FBDIV_MASK) >> VCO_FBDIV_SHIFT;
	freq *= parent_rate;
	do_div(freq, refdiv);

	return (unsigned long)freq;
}

static const struct clk_ops avpll_vco_ops = {
	.recalc_rate	= avpll_vco_recalc_rate,
};

static const u8 div_hdmi[] = { 1, 2, 4, 6 };
static const u8 div_av1[] = { 1, 2, 5, 5 };

static unsigned long
avpll_channel_recalc_rate(struct clk_hw *hw, unsigned long parent_rate)
{
	struct avpll_channel *ch = to_avpll_channel(hw);
	struct avpll_vco *vco = ch->vco;
	u32 reg, divider = 1;
	u64 freq = parent_rate;

	reg = readl_relaxed(vco->base + VCO_CTRL30);
	if ((reg & (VCO_DPLL_CH1_ENABLE << ch->index)) == 0)
		goto skip_div;

	/*
	 * Fch = (Fref * sync2) /
	 *    (sync1 * div_hdmi * div_av1 * div_av2 * div_av3)
	 */

	reg = readl_relaxed(vco->base + VCO_SYNC1n(ch->index));
	/* SYNC1 for Channel 1 is shifted by 4 bits */
	if (ch->index == 0)
		reg >>= 4;
	divider = reg & VCO_SYNC1_MASK;

	reg = readl_relaxed(vco->base + VCO_SYNC2n(ch->index));
	freq *= reg & VCO_SYNC2_MASK;

	/* Channel 8 has no dividers */
	if (ch->index == 7)
		goto skip_div;

	/*
	 * HDMI divider start at VCO_CTRL11, bit 7; each 3 bits wide but
	 * only 0-3 are valid values.
	 */
	reg = readl_relaxed(vco->base + VCO_CTRL11) >> 7;
	divider *= div_hdmi[(reg >> (ch->index * 3)) & 0x3];

	/*
	 * AV1 divider start at VCO_CTRL11, bit 28; each 3 bits wide but
	 * only 0-3 are valid values.
	 */
	if (ch->index == 0) {
		reg = readl_relaxed(vco->base + VCO_CTRL11);
		reg >>= 28;
	} else {
		reg = readl_relaxed(vco->base + VCO_CTRL12);
		reg >>= (ch->index-1) * 3;
	}
	divider *= div_av1[reg & 0x3];

	/*
	 * AV2 divider start at VCO_CTRL12, bit 18; each 7 bits wide,
	 * zero is not a valid value.
	 */
	if (ch->index < 2) {
		reg = readl_relaxed(vco->base + VCO_CTRL12);
		reg >>= 18 + (ch->index * 7);
	} else if (ch->index < 7) {
		reg = readl_relaxed(vco->base + VCO_CTRL13);
		reg >>= (ch->index - 2) * 7;
	} else {
		reg = readl_relaxed(vco->base + VCO_CTRL14);
	}

	reg &= 0x7f;
	if (reg)
		divider *= reg;

	/*
	 * AV3 divider start at VCO_CTRL14, bit 7; each 4 bits wide,
	 * only 0, 1 are valid values. AV3=1 divides by 1/2, AV3=0 is bypass.
	 */
	if (ch->index < 6) {
		reg = readl_relaxed(vco->base + VCO_CTRL14);
		reg >>= 7 + (ch->index * 4);
	} else {
		reg = readl_relaxed(vco->base + VCO_CTRL15);
	}

	if (reg & 0x1)
		freq *= 2;

skip_div:
	do_div(freq, divider);
	return (unsigned long)freq;
}

static const struct clk_ops avpll_channel_ops = {
	.recalc_rate	= avpll_channel_recalc_rate,
};

/*
 * Another nice quirk: on production SoC revisions, AVPLL_B outputs
 * are scrambled. Use a translation table to deal with it.
 */
static const u8 avpllb_channel_map[] = { 0, 6, 5, 4, 3, 2, 1, 7 };

struct clk *avpll_src_get(struct of_phandle_args *clkspec, void *data)
{
	struct avpll *pll = data;
	unsigned int vco = clkspec->args[0];
	unsigned int ch = clkspec->args[1] - 1;

	if (vco > NUM_VCOS || ch > NUM_CHANNELS) {
		pr_err("%s: invalid clock index %d.%d\n",
		       __func__, vco, ch);
		return ERR_PTR(-EINVAL);
	}

	if (vco == 1)
		ch = avpllb_channel_map[ch];

	return pll->vco[vco].clks[ch];
}

static void __init avpll_of_setup(struct device_node *np)
{
	struct avpll *pll;
	struct clk_init_data init;
	struct clk *refclk;
	const char *refclk_name;
	char *pll_name = NULL;
	int nvco, nch;

	pll = kzalloc(sizeof(*pll), GFP_KERNEL);
	if (!pll)
		return;

	pll->base = of_iomap(np, 0);
	if (!pll->base) {
		pr_err("%s: Unable to map pll register\n", np->full_name);
		kfree(pll);
		return;
	}

	refclk = of_clk_get(np, 0);
	if (IS_ERR(refclk)) {
		pr_err("%s: Missing reference clock\n", np->full_name);
		goto avpll_fail;
	}
	refclk_name = __clk_get_name(refclk);
	clk_put(refclk);

	pll_name = of_clk_create_name(np);

	for (nvco = 0; nvco < NUM_VCOS; nvco++) {
		struct avpll_vco *vco = &pll->vco[nvco];
		const char *vco_name =
			kasprintf(GFP_KERNEL, "%s.vco%d", pll_name, nvco);
		struct clk *clk;

		init.name = vco_name;
		init.ops = &avpll_vco_ops;
		init.parent_names = &refclk_name;
		init.num_parents = 1;
		init.flags = 0;

		vco->hw.init = &init;
		vco->base = pll->base + (nvco * VCO_OFFSET);
		vco->pll = pll;
		vco->index = nvco;

		clk = clk_register(NULL, &vco->hw);
		kfree(vco_name);
		if (IS_ERR(clk)) {
			pr_err("%s: Unable to register vco %d\n",
			       np->full_name, nvco);
			goto avpll_fail;
		}
		vco_name = __clk_get_name(clk);

		for (nch = 0; nch < NUM_CHANNELS; nch++) {
			struct avpll_channel *ch = &vco->channel[nch];
			char *ch_name = kasprintf(GFP_KERNEL,
						  "%s.%d", vco_name, nch);

			init.name = ch_name;
			init.ops = &avpll_channel_ops;
			init.parent_names = &vco_name;
			init.num_parents = 1;
			init.flags = CLK_SET_RATE_PARENT;

			ch->hw.init = &init;
			ch->vco = vco;
			ch->index = nch;

			clk = clk_register(NULL, &ch->hw);
			kfree(ch_name);
			if (IS_ERR(clk)) {
				pr_err("%s: Unable to register channel %d.%d\n",
				       np->full_name, nvco, nch);
				goto avpll_fail;
			}
			vco->clks[nch] = clk;
		}
	}

	of_clk_add_provider(np, avpll_src_get, pll);
	kfree(pll_name);
	return;

avpll_fail:
	kfree(pll_name);
	iounmap(pll->base);
	kfree(pll);
}
CLK_OF_DECLARE(berlin2_avpll, "marvell,berlin2-avpll", avpll_of_setup);
