/*
 * Copyright (c) 2014 Marvell Technology Group Ltd.
 *
 * Alexandre Belloni <alexandre.belloni@free-electrons.com>
 * Sebastian Hesselbarth <sebastian.hesselbarth@gmail.com>
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
#include <asm/div64.h>

struct berlin2_pll_map {
	const u8 vcodiv[16];
	u8 mult;
	u8 fbdiv_shift;
	u8 rfdiv_shift;
	u8 divsel_shift;
};

struct berlin2_pll {
	struct clk_hw hw;
	void __iomem *base;
	struct berlin2_pll_map map;
};

#define to_berlin2_pll(hw) container_of(hw, struct berlin2_pll, hw)

#define SPLL_CTRL0	0x00
#define SPLL_CTRL1	0x04
#define SPLL_CTRL2	0x08
#define SPLL_CTRL3	0x0c
#define SPLL_CTRL4	0x10

#define FBDIV_MASK	0x1ff
#define RFDIV_MASK	0x1f
#define DIVSEL_MASK	0xf

/*
 * The output frequency formula for the pll is:
 * clkout = fbdiv / refdiv * parent / vcodiv
 */
static unsigned long
berlin2_pll_recalc_rate(struct clk_hw *hw, unsigned long parent_rate)
{
	struct berlin2_pll *pll = to_berlin2_pll(hw);
	struct berlin2_pll_map *map = &pll->map;
	u32 val, fbdiv, rfdiv, vcodivsel, vcodiv;
	u64 rate = parent_rate;

	val = readl_relaxed(pll->base + SPLL_CTRL0);
	fbdiv = (val >> map->fbdiv_shift) & FBDIV_MASK;
	rfdiv = (val >> map->rfdiv_shift) & RFDIV_MASK;
	if (rfdiv == 0) {
		pr_warn("%s has zero rfdiv\n", __clk_get_name(hw->clk));
		rfdiv = 1;
	}

	val = readl_relaxed(pll->base + SPLL_CTRL1);
	vcodivsel = (val >> map->divsel_shift) & DIVSEL_MASK;
	vcodiv = map->vcodiv[vcodivsel];
	if (vcodiv == 0) {
		pr_warn("%s has zero vcodiv (index %d)\n",
			__clk_get_name(hw->clk), vcodivsel);
		vcodiv = 1;
	}

	rate *= fbdiv * map->mult;
	do_div(rate, rfdiv * vcodiv);

	return (unsigned long)rate;
}

static const struct clk_ops berlin2_pll_ops = {
	.recalc_rate	= berlin2_pll_recalc_rate,
};

static struct clk * __init
berlin2_pll_register(const struct berlin2_pll_map *map,
		     void __iomem *base, const char *name,
		     const char *parent_name, unsigned long flags)
{
	struct clk_init_data init;
	struct berlin2_pll *pll;

	pll = kzalloc(sizeof(*pll), GFP_KERNEL);
	if (!pll)
		return ERR_PTR(-ENOMEM);

	/* copy pll_map to allow __initconst */
	memcpy(&pll->map, map, sizeof(*map));
	pll->base = base;
	pll->hw.init = &init;
	init.name = name;
	init.ops = &berlin2_pll_ops;
	init.parent_names = &parent_name;
	init.num_parents = 1;
	init.flags = flags;

	return clk_register(NULL, &pll->hw);
}

static const struct berlin2_pll_map bg2_pll_map __initconst = {
	.vcodiv		= {10, 15, 20, 25, 30, 40, 50, 60, 80},
	.mult		= 10,
	.fbdiv_shift	= 6,
	.rfdiv_shift	= 1,
	.divsel_shift	= 7,
};

static const struct berlin2_pll_map bg2q_pll_map __initconst = {
	.vcodiv		= {1, 0, 2, 0, 3, 4, 0, 6, 8},
	.mult		= 1,
	.fbdiv_shift	= 7,
	.rfdiv_shift	= 2,
	.divsel_shift	= 9,
};

static const struct of_device_id pll_matches[] __initconst = {
	{ .compatible = "marvell,berlin2-pll", .data = &bg2_pll_map },
	{ .compatible = "marvell,berlin2q-pll", .data = &bg2q_pll_map },
	{ }
};

static void __init berlin2_pll_of_setup(struct device_node *np)
{
	const struct of_device_id *match = of_match_node(pll_matches, np);
	const struct berlin2_pll_map *map = match->data;
	void __iomem *base;
	struct clk *refclk;
	struct clk *pll;
	char *pll_name;

	base = of_iomap(np, 0);
	if (!base) {
		pr_err("%s: Unable to map pll register\n", np->full_name);
		return;
	}

	refclk = of_clk_get(np, 0);
	if (IS_ERR(refclk)) {
		pr_err("%s: Missing reference clock\n", np->full_name);
		iounmap(base);
		return;
	}

	pll_name = of_clk_create_name(np);
	pll = berlin2_pll_register(map, base, pll_name,
				   __clk_get_name(refclk), 0);
	if (!IS_ERR(pll))
		of_clk_add_provider(np, of_clk_src_simple_get, pll);

	clk_put(refclk);
	kfree(pll_name);
}
CLK_OF_DECLARE(berlin2_pll, "marvell,berlin2-pll", berlin2_pll_of_setup);
CLK_OF_DECLARE(berlin2q_pll, "marvell,berlin2q-pll", berlin2_pll_of_setup);
