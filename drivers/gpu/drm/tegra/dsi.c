/*
 * Copyright (C) 2013 NVIDIA Corporation
 *
 * Permission to use, copy, modify, distribute, and sell this software and its
 * documentation for any purpose is hereby granted without fee, provided that
 * the above copyright notice appear in all copies and that both that copyright
 * notice and this permission notice appear in supporting documentation, and
 * that the name of the copyright holders not be used in advertising or
 * publicity pertaining to distribution of the software without specific,
 * written prior permission.  The copyright holders make no representations
 * about the suitability of this software for any purpose.  It is provided "as
 * is" without express or implied warranty.
 *
 * THE COPYRIGHT HOLDERS DISCLAIM ALL WARRANTIES WITH REGARD TO THIS SOFTWARE,
 * INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS, IN NO
 * EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE FOR ANY SPECIAL, INDIRECT OR
 * CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE,
 * DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER
 * TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THIS SOFTWARE.
 */

#include <linux/clk.h>
#include <linux/clk/tegra.h>
#include <linux/debugfs.h>
#include <linux/host1x.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include "drm.h"
#include "dc.h"

#define DSI_INCR_SYNCPT			0x00
#define DSI_INCR_SYNCPT_CONTROL		0x01
#define DSI_INCR_SYNCPT_ERROR		0x02
#define DSI_CTXSW			0x08
#define DSI_RD_DATA			0x09
#define DSI_WR_DATA			0x0a
#define DSI_POWER_CONTROL		0x0b
#define DSI_POWER_CONTROL_ENABLE	(1 << 0)
#define DSI_INT_ENABLE			0x0c
#define DSI_INT_STATUS			0x0d
#define DSI_INT_MASK			0x0e
#define DSI_HOST_CONTROL		0x0f
#define DSI_HOST_CONTROL_RAW		(1 << 6)
#define DSI_HOST_CONTROL_HS		(1 << 5)
#define DSI_HOST_CONTROL_CS		(1 << 1)
#define DSI_HOST_CONTROL_ECC		(1 << 0)
#define DSI_CONTROL			0x10
#define DSI_CONTROL_HS_CLK_CTRL		(1 << 20)
#define DSI_CONTROL_CHANNEL(c)		(((c) & 0x3) << 16)
#define DSI_CONTROL_FORMAT(f)		(((f) & 0x3) << 12)
#define DSI_CONTROL_TX_TRIG(x)		(((x) & 0x3) <<  8)
#define DSI_CONTROL_LANES(n)		(((n) & 0x3) <<  4)
#define DSI_CONTROL_DCS_ENABLE		(1 << 3)
#define DSI_CONTROL_SOURCE(s)		(((s) & 0x1) <<  2)
#define DSI_CONTROL_VIDEO_ENABLE	(1 << 1)
#define DSI_CONTROL_HOST_ENABLE		(1 << 0)
#define DSI_SOL_DELAY			0x11
#define DSI_MAX_THRESHOLD		0x12
#define DSI_TRIGGER			0x13
#define DSI_TX_CRC			0x14
#define DSI_STATUS			0x15
#define DSI_INIT_SEQ_CONTROL		0x1a
#define DSI_INIT_SEQ_DATA_0		0x1b
#define DSI_INIT_SEQ_DATA_1		0x1c
#define DSI_INIT_SEQ_DATA_2		0x1d
#define DSI_INIT_SEQ_DATA_3		0x1e
#define DSI_INIT_SEQ_DATA_4		0x1f
#define DSI_INIT_SEQ_DATA_5		0x20
#define DSI_INIT_SEQ_DATA_6		0x21
#define DSI_INIT_SEQ_DATA_7		0x22
#define DSI_PKT_SEQ_0_LO		0x23
#define DSI_PKT_SEQ_0_HI		0x24
#define DSI_PKT_SEQ_1_LO		0x25
#define DSI_PKT_SEQ_1_HI		0x26
#define DSI_PKT_SEQ_2_LO		0x27
#define DSI_PKT_SEQ_2_HI		0x28
#define DSI_PKT_SEQ_3_LO		0x29
#define DSI_PKT_SEQ_3_HI		0x2a
#define DSI_PKT_SEQ_4_LO		0x2b
#define DSI_PKT_SEQ_4_HI		0x2c
#define DSI_PKT_SEQ_5_LO		0x2d
#define DSI_PKT_SEQ_5_HI		0x2e
#define DSI_DCS_CMDS			0x33
#define DSI_PKT_LEN_0_1			0x34
#define DSI_PKT_LEN_2_3			0x35
#define DSI_PKT_LEN_4_5			0x36
#define DSI_PKT_LEN_6_7			0x37
#define DSI_PHY_TIMING_0		0x3c
#define DSI_PHY_TIMING_1		0x3d
#define DSI_PHY_TIMING_2		0x3e
#define DSI_BTA_TIMING			0x3f

#define DSI_TIMING_FIELD(value, period, hwinc) \
	((DIV_ROUND_CLOSEST(value, period) - (hwinc)) & 0xff)

#define DSI_TIMEOUT_0			0x44
#define DSI_TIMEOUT_LRX(x)		(((x) & 0xffff) << 16)
#define DSI_TIMEOUT_HTX(x)		(((x) & 0xffff) <<  0)
#define DSI_TIMEOUT_1			0x45
#define DSI_TIMEOUT_PR(x)		(((x) & 0xffff) << 16)
#define DSI_TIMEOUT_TA(x)		(((x) & 0xffff) <<  0)
#define DSI_TO_TALLY			0x46
#define DSI_TALLY_TA(x)			(((x) & 0xff) << 16)
#define DSI_TALLY_LRX(x)		(((x) & 0xff) <<  8)
#define DSI_TALLY_HTX(x)		(((x) & 0xff) <<  0)
#define DSI_PAD_CONTROL_0		0x4b
#define DSI_PAD_CONTROL_VS1_PDIO(x)	(((x) & 0xf) <<  0)
#define DSI_PAD_CONTROL_VS1_PDIO_CLK	(1 <<  8)
#define DSI_PAD_CONTROL_VS1_PULLDN(x)	(((x) & 0xf) << 16)
#define DSI_PAD_CONTROL_VS1_PULLDN_CLK	(1 << 24)
#define DSI_PAD_CONTROL_CD		0x4c
#define DSI_PAD_CD_STATUS		0x4d
#define DSI_VIDEO_MODE_CONTROL		0x4e
#define DSI_PAD_CONTROL_1		0x4f
#define DSI_PAD_CONTROL_2		0x50
#define DSI_PAD_OUT_CLK(x)		(((x) & 0x7) <<  0)
#define DSI_PAD_LP_DN(x)		(((x) & 0x7) <<  4)
#define DSI_PAD_LP_UP(x)		(((x) & 0x7) <<  8)
#define DSI_PAD_SLEW_DN(x)		(((x) & 0x7) << 12)
#define DSI_PAD_SLEW_UP(x)		(((x) & 0x7) << 16)
#define DSI_PAD_CONTROL_3		0x51
#define DSI_PAD_CONTROL_4		0x52
#define DSI_GANGED_MODE_CONTROL		0x53
#define DSI_GANGED_MODE_START		0x54
#define DSI_GANGED_MODE_SIZE		0x55
#define DSI_RAW_DATA_BYTE_COUNT		0x56
#define DSI_ULTRA_LOW_POWER_CONTROL	0x57
#define DSI_INIT_SEQ_DATA_8		0x58
#define DSI_INIT_SEQ_DATA_9		0x59
#define DSI_INIT_SEQ_DATA_10		0x5a
#define DSI_INIT_SEQ_DATA_11		0x5b
#define DSI_INIT_SEQ_DATA_12		0x5c
#define DSI_INIT_SEQ_DATA_13		0x5d
#define DSI_INIT_SEQ_DATA_14		0x5e
#define DSI_INIT_SEQ_DATA_15		0x5f

enum dsi_format {
	DSI_FORMAT_16P,
	DSI_FORMAT_18NP,
	DSI_FORMAT_18P,
	DSI_FORMAT_24P,
};

/*
 * D-PHY timing parameters
 *
 * A detailed description of these parameters can be found in the  MIPI
 * Alliance Specification for D-PHY, Section 5.9 "Global Operation Timing
 * Parameters".
 *
 * All parameters are specified in nanoseconds.
 */
struct dsi_phy_timing {
	unsigned int clkmiss;
	unsigned int clkpost;
	unsigned int clkpre;
	unsigned int clkprepare;
	unsigned int clksettle;
	unsigned int clktermen;
	unsigned int clktrail;
	unsigned int clkzero;
	unsigned int dtermen;
	unsigned int eot;
	unsigned int hsexit;
	unsigned int hsprepare;
	unsigned int hszero;
	unsigned int hssettle;
	unsigned int hsskip;
	unsigned int hstrail;
	unsigned int init;
	unsigned int lpx;
	unsigned int taget;
	unsigned int tago;
	unsigned int tasure;
	unsigned int wakeup;
};

/*
 * Default D-PHY timings based on MIPI D-PHY specification. Derived from
 * the valid ranges specified in Section 5.9 of the D-PHY specification
 * with minor adjustments.
 */
static int dsi_phy_timing_get_default(struct dsi_phy_timing *timing,
				      unsigned long period)
{
	timing->clkmiss = 0;
	timing->clkpost = 70 + 52 * period;
	timing->clkpre = 8;
	timing->clkprepare = 65;
	timing->clksettle = 95;
	timing->clktermen = 0;
	timing->clktrail = 80;
	timing->clkzero = 260;
	timing->dtermen = 0;
	timing->eot = 0;
	timing->hsexit = 120;
	timing->hsprepare = 65 + 5 * period;
	timing->hszero = 145 + 5 * period;
	timing->hssettle = 85 + 6 * period;
	timing->hsskip = 40;
	timing->hstrail = max(8 * period, 60 + 4 * period);
	timing->init = 100000;
	timing->lpx = 60;
	timing->taget = 5 * timing->lpx;
	timing->tago = 4 * timing->lpx;
	timing->tasure = 2 * timing->lpx;
	timing->wakeup = 1000000;

	return 0;
}

/*
 * Validate D-PHY timing according to MIPI Alliance Specification for D-PHY,
 * Section 5.9 "Global Operation Timing Parameters".
 */
static int dsi_phy_timing_validate(struct dsi_phy_timing *timing,
				   unsigned long period)
{
	if (timing->clkmiss > 60)
		return -EINVAL;

	if (timing->clkpost < (60 + 52 * period))
		return -EINVAL;

	if (timing->clkpre < 8)
		return -EINVAL;

	if (timing->clkprepare < 38 || timing->clkprepare > 95)
		return -EINVAL;

	if (timing->clksettle < 95 || timing->clksettle > 300)
		return -EINVAL;

	if (timing->clktermen > 38)
		return -EINVAL;

	if (timing->clktrail < 60)
		return -EINVAL;

	if (timing->clkprepare + timing->clkzero < 300)
		return -EINVAL;

	if (timing->dtermen > 35 + 4 * period)
		return -EINVAL;

	if (timing->eot > 105 + 12 * period)
		return -EINVAL;

	if (timing->hsexit < 100)
		return -EINVAL;

	if (timing->hsprepare < 40 + 4 * period ||
	    timing->hsprepare > 85 + 6 * period)
		return -EINVAL;

	if (timing->hsprepare + timing->hszero < 145 + 10 * period)
		return -EINVAL;

	if ((timing->hssettle < 85 + 6 * period) ||
	    (timing->hssettle > 145 + 10 * period))
		return -EINVAL;

	if (timing->hsskip < 40 || timing->hsskip > 55 + 4 * period)
		return -EINVAL;

	if (timing->hstrail < max(8 * period, 60 + 4 * period))
		return -EINVAL;

	if (timing->init < 100000)
		return -EINVAL;

	if (timing->lpx < 50)
		return -EINVAL;

	if (timing->taget != 5 * timing->lpx)
		return -EINVAL;

	if (timing->tago != 4 * timing->lpx)
		return -EINVAL;

	if (timing->tasure < timing->lpx || timing->tasure > 2 * timing->lpx)
		return -EINVAL;

	if (timing->wakeup < 1000000)
		return -EINVAL;

	return 0;
}

struct tegra_dsi {
	struct host1x_client client;
	struct tegra_output output;
	struct device *dev;

	void __iomem *regs;

	struct clk *clk_parent;
	struct clk *clk;

	struct drm_info_list *debugfs_files;
	struct drm_minor *minor;
	struct dentry *debugfs;

	enum dsi_format format;
	unsigned int lanes;
};

static inline struct tegra_dsi *
host1x_client_to_dsi(struct host1x_client *client)
{
	return container_of(client, struct tegra_dsi, client);
}

static inline struct tegra_dsi *to_dsi(struct tegra_output *output)
{
	return container_of(output, struct tegra_dsi, output);
}

static inline unsigned long tegra_dsi_readl(struct tegra_dsi *dsi,
					    unsigned long reg)
{
	return readl(dsi->regs + (reg << 2));
}

static inline void tegra_dsi_writel(struct tegra_dsi *dsi, unsigned long value,
				    unsigned long reg)
{
	writel(value, dsi->regs + (reg << 2));
}

static int tegra_dsi_show_regs(struct seq_file *s, void *data)
{
	struct drm_info_node *node = s->private;
	struct tegra_dsi *dsi = node->info_ent->data;

#define DUMP_REG(name)						\
	seq_printf(s, "%-32s %#05x %08lx\n", #name, name,	\
		   tegra_dsi_readl(dsi, name))

	DUMP_REG(DSI_INCR_SYNCPT);
	DUMP_REG(DSI_INCR_SYNCPT_CONTROL);
	DUMP_REG(DSI_INCR_SYNCPT_ERROR);
	DUMP_REG(DSI_CTXSW);
	DUMP_REG(DSI_RD_DATA);
	DUMP_REG(DSI_WR_DATA);
	DUMP_REG(DSI_POWER_CONTROL);
	DUMP_REG(DSI_INT_ENABLE);
	DUMP_REG(DSI_INT_STATUS);
	DUMP_REG(DSI_INT_MASK);
	DUMP_REG(DSI_HOST_CONTROL);
	DUMP_REG(DSI_CONTROL);
	DUMP_REG(DSI_SOL_DELAY);
	DUMP_REG(DSI_MAX_THRESHOLD);
	DUMP_REG(DSI_TRIGGER);
	DUMP_REG(DSI_TX_CRC);
	DUMP_REG(DSI_STATUS);

	DUMP_REG(DSI_INIT_SEQ_CONTROL);
	DUMP_REG(DSI_INIT_SEQ_DATA_0);
	DUMP_REG(DSI_INIT_SEQ_DATA_1);
	DUMP_REG(DSI_INIT_SEQ_DATA_2);
	DUMP_REG(DSI_INIT_SEQ_DATA_3);
	DUMP_REG(DSI_INIT_SEQ_DATA_4);
	DUMP_REG(DSI_INIT_SEQ_DATA_5);
	DUMP_REG(DSI_INIT_SEQ_DATA_6);
	DUMP_REG(DSI_INIT_SEQ_DATA_7);

	DUMP_REG(DSI_PKT_SEQ_0_LO);
	DUMP_REG(DSI_PKT_SEQ_0_HI);
	DUMP_REG(DSI_PKT_SEQ_1_LO);
	DUMP_REG(DSI_PKT_SEQ_1_HI);
	DUMP_REG(DSI_PKT_SEQ_2_LO);
	DUMP_REG(DSI_PKT_SEQ_2_HI);
	DUMP_REG(DSI_PKT_SEQ_3_LO);
	DUMP_REG(DSI_PKT_SEQ_3_HI);
	DUMP_REG(DSI_PKT_SEQ_4_LO);
	DUMP_REG(DSI_PKT_SEQ_4_HI);
	DUMP_REG(DSI_PKT_SEQ_5_LO);
	DUMP_REG(DSI_PKT_SEQ_5_HI);

	DUMP_REG(DSI_DCS_CMDS);

	DUMP_REG(DSI_PKT_LEN_0_1);
	DUMP_REG(DSI_PKT_LEN_2_3);
	DUMP_REG(DSI_PKT_LEN_4_5);
	DUMP_REG(DSI_PKT_LEN_6_7);

	DUMP_REG(DSI_PHY_TIMING_0);
	DUMP_REG(DSI_PHY_TIMING_1);
	DUMP_REG(DSI_PHY_TIMING_2);
	DUMP_REG(DSI_BTA_TIMING);

	DUMP_REG(DSI_TIMEOUT_0);
	DUMP_REG(DSI_TIMEOUT_1);
	DUMP_REG(DSI_TO_TALLY);

	DUMP_REG(DSI_PAD_CONTROL_0);
	DUMP_REG(DSI_PAD_CONTROL_CD);
	DUMP_REG(DSI_PAD_CD_STATUS);
	DUMP_REG(DSI_VIDEO_MODE_CONTROL);
	DUMP_REG(DSI_PAD_CONTROL_1);
	DUMP_REG(DSI_PAD_CONTROL_2);
	DUMP_REG(DSI_PAD_CONTROL_3);
	DUMP_REG(DSI_PAD_CONTROL_4);

	DUMP_REG(DSI_GANGED_MODE_CONTROL);
	DUMP_REG(DSI_GANGED_MODE_START);
	DUMP_REG(DSI_GANGED_MODE_SIZE);

	DUMP_REG(DSI_RAW_DATA_BYTE_COUNT);
	DUMP_REG(DSI_ULTRA_LOW_POWER_CONTROL);

	DUMP_REG(DSI_INIT_SEQ_DATA_8);
	DUMP_REG(DSI_INIT_SEQ_DATA_9);
	DUMP_REG(DSI_INIT_SEQ_DATA_10);
	DUMP_REG(DSI_INIT_SEQ_DATA_11);
	DUMP_REG(DSI_INIT_SEQ_DATA_12);
	DUMP_REG(DSI_INIT_SEQ_DATA_13);
	DUMP_REG(DSI_INIT_SEQ_DATA_14);
	DUMP_REG(DSI_INIT_SEQ_DATA_15);

#undef DUMP_REG

	return 0;
}

static struct drm_info_list debugfs_files[] = {
	{ "regs", tegra_dsi_show_regs, 0, NULL },
};

static int tegra_dsi_debugfs_init(struct tegra_dsi *dsi,
				  struct drm_minor *minor)
{
	const char *name = dev_name(dsi->dev);
	unsigned int i;
	int err;

	dsi->debugfs = debugfs_create_dir(name, minor->debugfs_root);
	if (!dsi->debugfs)
		return -ENOMEM;

	dsi->debugfs_files = kmemdup(debugfs_files, sizeof(debugfs_files),
				     GFP_KERNEL);
	if (!dsi->debugfs_files) {
		err = -ENOMEM;
		goto remove;
	}

	for (i = 0; i < ARRAY_SIZE(debugfs_files); i++)
		dsi->debugfs_files[i].data = dsi;

	err = drm_debugfs_create_files(dsi->debugfs_files,
				       ARRAY_SIZE(debugfs_files),
				       dsi->debugfs, minor);
	if (err < 0)
		goto free;

	dsi->minor = minor;

	return 0;

free:
	kfree(dsi->debugfs_files);
	dsi->debugfs_files = NULL;
remove:
	debugfs_remove(dsi->debugfs);
	dsi->debugfs = NULL;

	return err;
}

static int tegra_dsi_debugfs_exit(struct tegra_dsi *dsi)
{
	drm_debugfs_remove_files(dsi->debugfs_files, ARRAY_SIZE(debugfs_files),
				 dsi->minor);
	dsi->minor = NULL;

	kfree(dsi->debugfs_files);
	dsi->debugfs_files = NULL;

	debugfs_remove(dsi->debugfs);
	dsi->debugfs = NULL;

	return 0;
}

enum {
	CMD_VS = 0x01,
	CMD_VE = 0x11,

	CMD_HS = 0x21,
	CMD_HE = 0x31,

	CMD_EOT = 0x08,
	CMD_NULL = 0x09,
	CMD_SHORTW = 0x15,
	CMD_BLNK = 0x19,
	CMD_LONGW = 0x39,

	CMD_RGB = 0x00,
	CMD_RGB_16BPP = 0x0E,
	CMD_RGB_18BPP = 0x1E,
	CMD_RGB_18BPPNP = 0x2E,
	CMD_RGB_24BPP = 0x3E,
};

#define PKT_ID0(id)	((((id) & 0x3f) << 3) | (1 << 9))
#define PKT_LEN0(len)	(((len) & 0x7) << 0)
#define PKT_ID1(id)	((((id) & 0x3f) << 13) | (1 << 19))
#define PKT_LEN1(len)	(((len) & 0x7) << 10)
#define PKT_ID2(id)	((((id) & 0x3f) << 23) | (1 << 29))
#define PKT_LEN2(len)	(((len) & 0x7) << 20)
#define PKT_ID3(id)	((((id) & 0x3f) << 3) | (1 << 9))
#define PKT_LEN3(len)	(((len) & 0x7) << 0)
#define PKT_ID4(id)	((((id) & 0x3f) << 13) | (1 << 19))
#define PKT_LEN4(len)	(((len) & 0x7) << 10)
#define PKT_ID5(id)	((((id) & 0x3f) << 23) | (1 << 29))
#define PKT_LEN5(len)	(((len) & 0x7) << 20)
#define PKT_LP		(1 << 30)
#define NUM_PKT_SEQ	12

/* non-burst mode with sync-end */
static const u32 pkt_seq_vnb_syne[NUM_PKT_SEQ] = {
	PKT_ID0(CMD_VS) | PKT_LEN0(0) | PKT_ID1(CMD_BLNK) | PKT_LEN1(1) |
	PKT_ID2(CMD_HE) | PKT_LEN2(0) | PKT_LP,
	0,
	PKT_ID0(CMD_VE) | PKT_LEN0(0) | PKT_ID1(CMD_BLNK) | PKT_LEN1(1) |
	PKT_ID2(CMD_HE) | PKT_LEN2(0) | PKT_LP,
	0,
	PKT_ID0(CMD_HS) | PKT_LEN0(0) | PKT_ID1(CMD_BLNK) | PKT_LEN1(1) |
	PKT_ID2(CMD_HE) | PKT_LEN2(0) | PKT_LP,
	0,
	PKT_ID0(CMD_HS) | PKT_LEN0(0) | PKT_ID1(CMD_BLNK) | PKT_LEN1(1) |
	PKT_ID2(CMD_HE) | PKT_LEN2(0),
	PKT_ID3(CMD_BLNK) | PKT_LEN3(2) | PKT_ID4(CMD_RGB_24BPP) | PKT_LEN4(3) |
	PKT_ID5(CMD_BLNK) | PKT_LEN5(4),
	PKT_ID0(CMD_HS) | PKT_LEN0(0) | PKT_ID1(CMD_BLNK) | PKT_LEN1(1) |
	PKT_ID2(CMD_HE) | PKT_LEN2(0) | PKT_LP,
	0,
	PKT_ID0(CMD_HS) | PKT_LEN0(0) | PKT_ID1(CMD_BLNK) | PKT_LEN1(1) |
	PKT_ID2(CMD_HE) | PKT_LEN2(0),
	PKT_ID3(CMD_BLNK) | PKT_LEN3(2) | PKT_ID4(CMD_RGB_24BPP) | PKT_LEN4(3) |
	PKT_ID5(CMD_BLNK) | PKT_LEN5(4),
};

static int tegra_dsi_set_phy_timing(struct tegra_dsi *dsi)
{
	struct dsi_phy_timing timing;
	unsigned long value, period;
	long rate;
	int err;

	rate = clk_get_rate(dsi->clk);
	if (rate < 0)
		return rate;

	period = DIV_ROUND_CLOSEST(1000000000UL, rate * 2);

	err = dsi_phy_timing_get_default(&timing, period);
	if (err < 0)
		return err;

	err = dsi_phy_timing_validate(&timing, period);
	if (err < 0) {
		dev_err(dsi->dev, "failed to validate D-PHY timing: %d\n", err);
		return err;
	}

	/*
	 * The D-PHY timing fields below are expressed in byte-clock cycles,
	 * so multiply the period by 8.
	 */
	period *= 8;

	value = DSI_TIMING_FIELD(timing.hsexit, period, 1) << 24 |
		DSI_TIMING_FIELD(timing.hstrail, period, 0) << 16 |
		DSI_TIMING_FIELD(timing.hszero, period, 3) << 8 |
		DSI_TIMING_FIELD(timing.hsprepare, period, 1);
	tegra_dsi_writel(dsi, value, DSI_PHY_TIMING_0);

	value = DSI_TIMING_FIELD(timing.clktrail, period, 1) << 24 |
		DSI_TIMING_FIELD(timing.clkpost, period, 1) << 16 |
		DSI_TIMING_FIELD(timing.clkzero, period, 1) << 8 |
		DSI_TIMING_FIELD(timing.lpx, period, 1);
	tegra_dsi_writel(dsi, value, DSI_PHY_TIMING_1);

	value = DSI_TIMING_FIELD(timing.clkprepare, period, 1) << 16 |
		DSI_TIMING_FIELD(timing.clkpre, period, 1) << 8 |
		DSI_TIMING_FIELD(0xff * period, period, 0) << 0;
	tegra_dsi_writel(dsi, value, DSI_PHY_TIMING_2);

	value = DSI_TIMING_FIELD(timing.taget, period, 1) << 16 |
		DSI_TIMING_FIELD(timing.tasure, period, 1) << 8 |
		DSI_TIMING_FIELD(timing.tago, period, 1);
	tegra_dsi_writel(dsi, value, DSI_BTA_TIMING);

	return 0;
}

static int tegra_dsi_get_muldiv(enum dsi_format format, unsigned int *mulp,
				unsigned int *divp)
{
	switch (format) {
	case DSI_FORMAT_16P:
		*mulp = 2;
		*divp = 1;
		break;

	case DSI_FORMAT_18NP:
		*mulp = 9;
		*divp = 4;
		break;

	case DSI_FORMAT_18P:
	case DSI_FORMAT_24P:
		*mulp = 3;
		*divp = 1;
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static int tegra_output_dsi_enable(struct tegra_output *output)
{
	struct tegra_dc *dc = to_tegra_dc(output->encoder.crtc);
	struct drm_display_mode *mode = &dc->base.mode;
	unsigned int hact, hsw, hbp, hfp, i, mul, div;
	struct tegra_dsi *dsi = to_dsi(output);
	/* FIXME: don't hardcode this */
	const u32 *pkt_seq = pkt_seq_vnb_syne;
	unsigned long value;
	int err;

	err = tegra_dsi_get_muldiv(dsi->format, &mul, &div);
	if (err < 0)
		return err;

	err = clk_enable(dsi->clk);
	if (err < 0)
		return err;

	tegra_periph_reset_deassert(dsi->clk);

	value = DSI_CONTROL_CHANNEL(0) | DSI_CONTROL_FORMAT(dsi->format) |
		DSI_CONTROL_LANES(dsi->lanes - 1) | DSI_CONTROL_SOURCE(0);
	tegra_dsi_writel(dsi, value, DSI_CONTROL);

	tegra_dc_writel(dc, DSI_ENABLE, DC_DISP_DISP_WIN_OPTIONS);

	value = PW0_ENABLE | PW1_ENABLE | PW2_ENABLE | PW3_ENABLE |
		PW4_ENABLE | PM0_ENABLE | PM1_ENABLE;
	tegra_dc_writel(dc, value, DC_CMD_DISPLAY_POWER_CONTROL);

	tegra_dc_writel(dc, DISP_CTRL_MODE_C_DISPLAY, DC_CMD_DISPLAY_COMMAND);
	tegra_dc_writel(dc, GENERAL_ACT_REQ << 8, DC_CMD_STATE_CONTROL);
	tegra_dc_writel(dc, GENERAL_ACT_REQ, DC_CMD_STATE_CONTROL);

	tegra_dsi_writel(dsi, 0x000001e0, DSI_MAX_THRESHOLD);

	value = DSI_HOST_CONTROL_HS | DSI_HOST_CONTROL_CS |
		DSI_HOST_CONTROL_ECC;
	tegra_dsi_writel(dsi, value, DSI_HOST_CONTROL);

	value = tegra_dsi_readl(dsi, DSI_CONTROL);
	value |= DSI_CONTROL_HS_CLK_CTRL;
	value &= ~DSI_CONTROL_TX_TRIG(3);
	value &= ~DSI_CONTROL_DCS_ENABLE;
	value |= DSI_CONTROL_VIDEO_ENABLE;
	value &= ~DSI_CONTROL_HOST_ENABLE;
	tegra_dsi_writel(dsi, value, DSI_CONTROL);

	err = tegra_dsi_set_phy_timing(dsi);
	if (err < 0)
		return err;

	for (i = 0; i < NUM_PKT_SEQ; i++)
		tegra_dsi_writel(dsi, pkt_seq[i], DSI_PKT_SEQ_0_LO + i);

	/* horizontal active pixels */
	hact = mode->hdisplay * mul / div;

	/* horizontal sync width */
	hsw = (mode->hsync_end - mode->hsync_start) * mul / div;
	hsw -= 10;

	/* horizontal back porch */
	hbp = (mode->htotal - mode->hsync_end) * mul / div;
	hbp -= 14;

	/* horizontal front porch */
	hfp = (mode->hsync_start  - mode->hdisplay) * mul / div;
	hfp -= 8;

	tegra_dsi_writel(dsi, hsw << 16 | 0, DSI_PKT_LEN_0_1);
	tegra_dsi_writel(dsi, hact << 16 | hbp, DSI_PKT_LEN_2_3);
	tegra_dsi_writel(dsi, hfp, DSI_PKT_LEN_4_5);
	tegra_dsi_writel(dsi, 0x0f0f << 16, DSI_PKT_LEN_6_7);

	/* set SOL delay */
	tegra_dsi_writel(dsi, 8 * mul / div, DSI_SOL_DELAY);

	return 0;
}

static int tegra_output_dsi_disable(struct tegra_output *output)
{
	struct tegra_dsi *dsi = to_dsi(output);

	clk_disable(dsi->clk);

	return 0;
}

static int tegra_output_dsi_setup_clock(struct tegra_output *output,
					struct clk *clk, unsigned long pclk)
{
	struct tegra_dc *dc = to_tegra_dc(output->encoder.crtc);
	struct drm_display_mode *mode = &dc->base.mode;
	struct tegra_dsi *dsi = to_dsi(output);
	unsigned long bclk, plld, value;
	unsigned int timeout, mul, div;
	struct clk *base;
	int err;

	err = tegra_dsi_get_muldiv(dsi->format, &mul, &div);
	if (err < 0)
		return err;

	pclk = mode->htotal * mode->vtotal * mode->vrefresh;
	bclk = (pclk * mul) / (div * dsi->lanes);
	plld = DIV_ROUND_UP(bclk * 8, 1000000);
	pclk = (plld * 1000000) / 2;

	err = clk_set_parent(clk, dsi->clk_parent);
	if (err < 0) {
		dev_err(dsi->dev, "failed to set parent clock: %d\n", err);
		return err;
	}

	base = clk_get_parent(dsi->clk_parent);

	/*
	 * This assumes that the parent clock is pll_d_out0 or pll_d2_out
	 * respectively, each of which divides the base pll_d by 2.
	 */
	err = clk_set_rate(base, pclk * 2);
	if (err < 0) {
		dev_err(dsi->dev, "failed to set base clock rate to %lu Hz\n",
			pclk * 2);
		return err;
	}

	/* one frame high-speed transmission timeout */
	timeout = (bclk / mode->vrefresh) / 512;
	value = DSI_TIMEOUT_LRX(0x2000) | DSI_TIMEOUT_HTX(timeout);
	tegra_dsi_writel(dsi, value, DSI_TIMEOUT_0);

	/* 2 ms peripheral timeout for panel */
	timeout = 2 * bclk / 512 * 1000;
	value = DSI_TIMEOUT_PR(timeout) | DSI_TIMEOUT_TA(0x2000);
	tegra_dsi_writel(dsi, value, DSI_TIMEOUT_1);

	value = DSI_TALLY_TA(0) | DSI_TALLY_LRX(0) | DSI_TALLY_HTX(0);
	tegra_dsi_writel(dsi, value, DSI_TO_TALLY);

	return 0;
}

static int tegra_output_dsi_check_mode(struct tegra_output *output,
				       struct drm_display_mode *mode,
				       enum drm_mode_status *status)
{
	/*
	 * FIXME: For now, always assume that the mode is okay.
	 */

	*status = MODE_OK;

	return 0;
}

static const struct tegra_output_ops dsi_ops = {
	.enable = tegra_output_dsi_enable,
	.disable = tegra_output_dsi_disable,
	.setup_clock = tegra_output_dsi_setup_clock,
	.check_mode = tegra_output_dsi_check_mode,
};

static int tegra_dsi_pad_enable(struct tegra_dsi *dsi)
{
	unsigned long value;

	value = DSI_PAD_CONTROL_VS1_PULLDN(0) | DSI_PAD_CONTROL_VS1_PDIO(0);
	tegra_dsi_writel(dsi, value, DSI_PAD_CONTROL_0);

	return 0;
}

static int tegra_dsi_init(struct host1x_client *client)
{
	struct tegra_drm *tegra = dev_get_drvdata(client->parent);
	struct tegra_dsi *dsi = host1x_client_to_dsi(client);
	unsigned long value, i;
	int err;

	dsi->output.type = TEGRA_OUTPUT_DSI;
	dsi->output.dev = client->dev;
	dsi->output.ops = &dsi_ops;

	err = tegra_output_init(tegra->drm, &dsi->output);
	if (err < 0) {
		dev_err(client->dev, "output setup failed: %d\n", err);
		return err;
	}

	if (IS_ENABLED(CONFIG_DEBUG_FS)) {
		err = tegra_dsi_debugfs_init(dsi, tegra->drm->primary);
		if (err < 0)
			dev_err(dsi->dev, "debugfs setup failed: %d\n", err);
	}

	/*
	 * enable high-speed mode, checksum generation, ECC generation and
	 * disable raw mode
	 */
	value = tegra_dsi_readl(dsi, DSI_HOST_CONTROL);
	value |= DSI_HOST_CONTROL_ECC | DSI_HOST_CONTROL_CS |
		 DSI_HOST_CONTROL_HS;
	value &= ~DSI_HOST_CONTROL_RAW;
	tegra_dsi_writel(dsi, value, DSI_HOST_CONTROL);

	tegra_dsi_writel(dsi, 0, DSI_SOL_DELAY);
	tegra_dsi_writel(dsi, 0, DSI_MAX_THRESHOLD);

	tegra_dsi_writel(dsi, 0, DSI_INIT_SEQ_CONTROL);

	for (i = 0; i < 8; i++) {
		tegra_dsi_writel(dsi, 0, DSI_INIT_SEQ_DATA_0 + i);
		tegra_dsi_writel(dsi, 0, DSI_INIT_SEQ_DATA_8 + i);
	}

	for (i = 0; i < 12; i++)
		tegra_dsi_writel(dsi, 0, DSI_PKT_SEQ_0_LO + i);

	tegra_dsi_writel(dsi, 0, DSI_DCS_CMDS);

	tegra_dsi_writel(dsi, 0, DSI_PAD_CONTROL_0);
	tegra_dsi_writel(dsi, 0, DSI_PAD_CONTROL_1);
	tegra_dsi_writel(dsi, 0, DSI_PAD_CONTROL_2);
	tegra_dsi_writel(dsi, 0, DSI_PAD_CONTROL_3);
	tegra_dsi_writel(dsi, 0, DSI_PAD_CONTROL_4);

	/* start calibration */
	tegra_dsi_pad_enable(dsi);

	value = DSI_PAD_SLEW_UP(0x7) | DSI_PAD_SLEW_DN(0x7) |
		DSI_PAD_LP_UP(0x1) | DSI_PAD_LP_DN(0x1) |
		DSI_PAD_OUT_CLK(0x0);
	tegra_dsi_writel(dsi, value, DSI_PAD_CONTROL_2);

	err = tegra_mipi_calibrate(dsi->dev);
	if (err < 0) {
		dev_err(dsi->dev, "MIPI calibration failed: %d\n", err);
		return err;
	}

	tegra_dsi_writel(dsi, DSI_POWER_CONTROL_ENABLE, DSI_POWER_CONTROL);
	usleep_range(300, 1000);

	return 0;
}

static int tegra_dsi_exit(struct host1x_client *client)
{
	struct tegra_dsi *dsi = host1x_client_to_dsi(client);
	int err;

	if (IS_ENABLED(CONFIG_DEBUG_FS)) {
		err = tegra_dsi_debugfs_exit(dsi);
		if (err < 0)
			dev_err(dsi->dev, "debugfs cleanup failed: %d\n", err);
	}

	err = tegra_output_disable(&dsi->output);
	if (err < 0) {
		dev_err(client->dev, "output failed to disable: %d\n", err);
		return err;
	}

	err = tegra_output_exit(&dsi->output);
	if (err < 0) {
		dev_err(client->dev, "output cleanup failed: %d\n", err);
		return err;
	}

	return 0;
}

static const struct host1x_client_ops dsi_client_ops = {
	.init = tegra_dsi_init,
	.exit = tegra_dsi_exit,
};

static int tegra_dsi_setup_clocks(struct tegra_dsi *dsi)
{
	struct clk *parent;
	int err;

	parent = clk_get_parent(dsi->clk);
	if (!parent)
		return -EINVAL;

	err = clk_set_parent(parent, dsi->clk_parent);
	if (err < 0)
		return err;

	return 0;
}

static int tegra_dsi_probe(struct platform_device *pdev)
{
	struct tegra_dsi *dsi;
	struct resource *regs;
	int err;

	dsi = devm_kzalloc(&pdev->dev, sizeof(*dsi), GFP_KERNEL);
	if (!dsi)
		return -ENOMEM;

	dsi->output.dev = dsi->dev = &pdev->dev;

	err = tegra_output_probe(&dsi->output);
	if (err < 0)
		return err;

	/*
	 * FIXME: Don't hardcode these. Perhaps they should be queried from
	 *        the panel or from the DSI interface's DT node.
	 */
	dsi->format = DSI_FORMAT_24P;
	dsi->lanes = 4;

	dsi->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(dsi->clk))
		return PTR_ERR(dsi->clk);;

	err = clk_prepare_enable(dsi->clk);
	if (err < 0)
		return err;

	dsi->clk_parent = devm_clk_get(&pdev->dev, "parent");
	if (IS_ERR(dsi->clk_parent))
		return PTR_ERR(dsi->clk_parent);

	err = clk_prepare_enable(dsi->clk_parent);
	if (err < 0)
		return err;

	err = tegra_dsi_setup_clocks(dsi);
	if (err < 0)
		return err;

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	dsi->regs = devm_ioremap_resource(&pdev->dev, regs);
	if (!dsi->regs)
		return -EADDRNOTAVAIL;

	INIT_LIST_HEAD(&dsi->client.list);
	dsi->client.ops = &dsi_client_ops;
	dsi->client.dev = &pdev->dev;

	err = host1x_client_register(&dsi->client);
	if (err < 0) {
		dev_err(&pdev->dev, "failed to register host1x client: %d\n",
			err);
		return err;
	}

	platform_set_drvdata(pdev, dsi);

	return 0;
}

static int tegra_dsi_remove(struct platform_device *pdev)
{
	struct tegra_dsi *dsi = platform_get_drvdata(pdev);
	int err;

	err = host1x_client_unregister(&dsi->client);
	if (err < 0) {
		dev_err(&pdev->dev, "failed to unregister host1x client: %d\n",
			err);
		return err;
	}

	clk_disable_unprepare(dsi->clk_parent);
	clk_disable_unprepare(dsi->clk);

	err = tegra_output_remove(&dsi->output);
	if (err < 0) {
		dev_err(&pdev->dev, "failed to remove output: %d\n", err);
		return err;
	}

	return 0;
}

static const struct of_device_id tegra_dsi_of_match[] = {
	{ .compatible = "nvidia,tegra114-dsi", },
	{ },
};

struct platform_driver tegra_dsi_driver = {
	.driver = {
		.name = "tegra-dsi",
		.of_match_table = tegra_dsi_of_match,
	},
	.probe = tegra_dsi_probe,
	.remove = tegra_dsi_remove,
};
