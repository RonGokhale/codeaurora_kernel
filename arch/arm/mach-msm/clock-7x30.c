/* Copyright (c) 2009-2011, Code Aurora Forum. All rights reserved.
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/ctype.h>
#include <linux/bitops.h>
#include <linux/io.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/string.h>

#include <mach/msm_iomap.h>
#include <mach/clk.h>
#include <mach/internal_power_rail.h>

#include "clock.h"
#include "clock-local.h"
#include "clock-pcom.h"
#include "clock-voter.h"
#include "proc_comm.h"

#define REG_BASE(off) (MSM_CLK_CTL_BASE + (off))
#define REG(off) (MSM_CLK_CTL_SH2_BASE + (off))

/* Shadow-region 2 (SH2) registers. */
#define	QUP_I2C_NS_REG		REG(0x04F0)
#define CAM_NS_REG		REG(0x0374)
#define CAM_VFE_NS_REG		REG(0x0044)
#define CLK_HALT_STATEA_REG	REG(0x0108)
#define CLK_HALT_STATEB_REG	REG(0x010C)
#define CLK_HALT_STATEC_REG	REG(0x02D4)
#define CSI_NS_REG		REG(0x0174)
#define EMDH_NS_REG		REG(0x0050)
#define GLBL_CLK_ENA_2_SC_REG	REG(0x03C0)
#define GLBL_CLK_ENA_SC_REG	REG(0x03BC)
#define GLBL_CLK_STATE_2_REG	REG(0x037C)
#define GLBL_CLK_STATE_REG	REG(0x0004)
#define GRP_2D_NS_REG		REG(0x0034)
#define GRP_NS_REG		REG(0x0084)
#define HDMI_NS_REG		REG(0x0484)
#define I2C_2_NS_REG		REG(0x02D8)
#define I2C_NS_REG		REG(0x0068)
#define JPEG_NS_REG		REG(0x0164)
#define LPA_CORE_CLK_MA0_REG	REG(0x04F4)
#define LPA_CORE_CLK_MA2_REG	REG(0x04FC)
#define LPA_NS_REG		REG(0x02E8)
#define MDC_NS_REG		REG(0x007C)
#define MDP_LCDC_NS_REG		REG(0x0390)
#define MDP_NS_REG		REG(0x014C)
#define MDP_VSYNC_REG		REG(0x0460)
#define MFC_NS_REG		REG(0x0154)
#define MI2S_CODEC_RX_DIV_REG	REG(0x02EC)
#define MI2S_CODEC_TX_DIV_REG	REG(0x02F0)
#define MI2S_DIV_REG		REG(0x02E4)
#define MI2S_NS_REG		REG(0x02E0)
#define MI2S_RX_NS_REG		REG(0x0070)
#define MI2S_TX_NS_REG		REG(0x0078)
#define MIDI_NS_REG		REG(0x02D0)
#define PLL_ENA_REG		REG(0x0264)
#define PMDH_NS_REG		REG(0x008C)
#define SDAC_NS_REG		REG(0x009C)
#define SDCn_NS_REG(n)		REG(0x00A4+(0x8*((n)-1)))
#define SPI_NS_REG		REG(0x02C8)
#define TSIF_NS_REG		REG(0x00C4)
#define TV_NS_REG		REG(0x00CC)
#define UART1DM_NS_REG		REG(0x00D4)
#define UART2DM_NS_REG		REG(0x00DC)
#define UART2_NS_REG		REG(0x0464)
#define UART_NS_REG		REG(0x00E0)
#define USBH2_NS_REG		REG(0x046C)
#define USBH3_NS_REG		REG(0x0470)
#define USBH_MD_REG		REG(0x02BC)
#define USBH_NS_REG		REG(0x02C0)
#define VPE_NS_REG		REG(0x015C)

/* Registers in the base (non-shadow) region. */
#define CLK_TEST_BASE_REG	REG_BASE(0x011C)
#define CLK_TEST_2_BASE_REG	REG_BASE(0x0384)
#define MISC_CLK_CTL_BASE_REG	REG_BASE(0x0110)
#define PRPH_WEB_NS_BASE_REG	REG_BASE(0x0080)
#define PLL0_STATUS_BASE_REG	REG_BASE(0x0318)
#define PLL1_STATUS_BASE_REG	REG_BASE(0x0334)
#define PLL2_STATUS_BASE_REG	REG_BASE(0x0350)
#define PLL3_STATUS_BASE_REG	REG_BASE(0x036C)
#define PLL4_STATUS_BASE_REG	REG_BASE(0x0254)
#define PLL5_STATUS_BASE_REG	REG_BASE(0x0258)
#define PLL6_STATUS_BASE_REG	REG_BASE(0x04EC)
#define RINGOSC_CNT_BASE_REG	REG_BASE(0x00FC)
#define SH2_OWN_APPS1_BASE_REG	REG_BASE(0x040C)
#define SH2_OWN_APPS2_BASE_REG	REG_BASE(0x0414)
#define SH2_OWN_APPS3_BASE_REG	REG_BASE(0x0444)
#define SH2_OWN_GLBL_BASE_REG	REG_BASE(0x0404)
#define SH2_OWN_ROW1_BASE_REG	REG_BASE(0x041C)
#define SH2_OWN_ROW2_BASE_REG	REG_BASE(0x0424)
#define TCXO_CNT_BASE_REG	REG_BASE(0x00F8)
#define TCXO_CNT_DONE_BASE_REG	REG_BASE(0x00F8)


/* MUX source input identifiers. */
#define SRC_SEL_pll0		4 /* Modem PLL */
#define SRC_SEL_pll1		1 /* Global PLL */
#define SRC_SEL_pll3		3 /* Multimedia/Peripheral PLL or Backup PLL1 */
#define SRC_SEL_pll4		2 /* Display PLL */
#define SRC_SEL_SDAC_lpxo	5 /* Low-power XO for SDAC */
#define SRC_SEL_lpxo		6 /* Low-power XO */
#define SRC_SEL_tcxo		0 /* Used for rates from TCXO */
#define SRC_SEL_axi		0 /* Used for rates that sync to AXI */
#define SRC_SEL_gnd		7 /* No clock */

/* Clock declaration macros. */
#define MN_MODE_DUAL_EDGE	0x2
#define MD8(m, n)		(BVAL(15, 8, m) | BVAL(7, 0, ~(n)))
#define N8(msb, lsb, m, n)	(BVAL(msb, lsb, ~(n-m)) | BVAL(6, 5, \
					(MN_MODE_DUAL_EDGE * !!(n))))
#define MD16(m, n)		(BVAL(31, 16, m) | BVAL(15, 0, ~(n)))
#define N16(m, n)		(BVAL(31, 16, ~(n-m)) | BVAL(6, 5, \
					(MN_MODE_DUAL_EDGE * !!(n))))
#define SPDIV(s, d)		(BVAL(4, 3, d-1) | BVAL(2, 0, s))
#define SDIV(s, d)		(BVAL(6, 3, d-1) | BVAL(2, 0, s))
#define F_MASK_BASIC		(BM(6, 3)|BM(2, 0))
#define F_MASK_MND16		(BM(31, 16)|BM(6, 5)|BM(4, 3)|BM(2, 0))
#define F_MASK_MND8(m, l)	(BM(m, l)|BM(6, 5)|BM(4, 3)|BM(2, 0))

/*
 * Clock frequency definitions and macros
 */
#define F_BASIC(f, s, div, v) \
	{ \
		.freq_hz = f, \
		.src_clk = &s##_clk.c, \
		.ns_val = SDIV(SRC_SEL_##s, div), \
		.sys_vdd = v, \
	}

#define F_MND16(f, s, div, m, n, v) \
	{ \
		.freq_hz = f, \
		.src_clk = &s##_clk.c, \
		.md_val = MD16(m, n), \
		.ns_val = N16(m, n) | SPDIV(SRC_SEL_##s, div), \
		.mnd_en_mask = BIT(8) * !!(n), \
		.sys_vdd = v, \
	}

#define F_MND8(f, nmsb, nlsb, s, div, m, n, v) \
	{ \
		.freq_hz = f, \
		.src_clk = &s##_clk.c, \
		.md_val = MD8(m, n), \
		.ns_val = N8(nmsb, nlsb, m, n) | SPDIV(SRC_SEL_##s, div), \
		.mnd_en_mask = BIT(8) * !!(n), \
		.sys_vdd = v, \
	}

static struct clk_ops soc_clk_ops_7x30;

#define PCOM_XO_DISABLE	0
#define PCOM_XO_ENABLE	1
#define PCOM_XO_TCXO	0
#define PCOM_XO_LPXO	1

static bool pcom_is_local(struct clk *clk)
{
	return false;
}

static int pcom_xo_enable(unsigned pcom_id, unsigned enable)
{
	/* TODO: Check return code in pcom_id */
	return msm_proc_comm(PCOM_CLKCTL_RPC_SRC_REQUEST, &pcom_id, &enable);
}

static int tcxo_clk_enable(struct clk *clk)
{
	return pcom_xo_enable(PCOM_XO_TCXO, PCOM_XO_ENABLE);
}

static void tcxo_clk_disable(struct clk *clk)
{
	pcom_xo_enable(PCOM_XO_TCXO, PCOM_XO_DISABLE);
}

static struct clk_ops clk_ops_tcxo = {
	.enable = tcxo_clk_enable,
	.disable = tcxo_clk_disable,
	.get_rate = fixed_clk_get_rate,
	.is_local = pcom_is_local,
};

static struct fixed_clk tcxo_clk = {
	.rate = 19200000,
	.c = {
		.dbg_name = "tcxo_clk",
		.ops = &clk_ops_tcxo,
		CLK_INIT(tcxo_clk.c),
	},
};

static int lpxo_clk_enable(struct clk *clk)
{
	return pcom_xo_enable(PCOM_XO_LPXO, PCOM_XO_ENABLE);
}

static void lpxo_clk_disable(struct clk *clk)
{
	pcom_xo_enable(PCOM_XO_LPXO, PCOM_XO_DISABLE);
}

static struct clk_ops clk_ops_lpxo = {
	.enable = lpxo_clk_enable,
	.disable = lpxo_clk_disable,
	.get_rate = fixed_clk_get_rate,
	.is_local = pcom_is_local,
};

static struct fixed_clk lpxo_clk = {
	.rate = 24576000,
	.c = {
		.dbg_name = "lpxo_clk",
		.ops = &clk_ops_lpxo,
		CLK_INIT(lpxo_clk.c),
	},
};

static struct pll_vote_clk pll1_clk = {
	.rate = 768000000,
	.en_reg = PLL_ENA_REG,
	.en_mask = BIT(1),
	.status_reg = PLL1_STATUS_BASE_REG,
	.status_mask = BIT(16),
	.parent = &tcxo_clk.c,
	.c = {
		.dbg_name = "pll1_clk",
		.ops = &clk_ops_pll_vote,
		CLK_INIT(pll1_clk.c),
	},
};

static struct pll_vote_clk pll2_clk = {
	.rate = 806400000, /* TODO: Support scaling */
	.en_reg = PLL_ENA_REG,
	.en_mask = BIT(2),
	.status_reg = PLL2_STATUS_BASE_REG,
	.status_mask = BIT(16),
	.parent = &tcxo_clk.c,
	.c = {
		.dbg_name = "pll2_clk",
		.ops = &clk_ops_pll_vote,
		CLK_INIT(pll2_clk.c),
	},
};

static struct pll_vote_clk pll3_clk = {
	.rate = 737280000,
	.en_reg = PLL_ENA_REG,
	.en_mask = BIT(3),
	.status_reg = PLL3_STATUS_BASE_REG,
	.status_mask = BIT(16),
	.parent = &lpxo_clk.c,
	.c = {
		.dbg_name = "pll3_clk",
		.ops = &clk_ops_pll_vote,
		CLK_INIT(pll3_clk.c),
	},
};

static struct pll_vote_clk pll4_clk = {
	.rate = 891000000,
	.en_reg = PLL_ENA_REG,
	.en_mask = BIT(4),
	.status_reg = PLL4_STATUS_BASE_REG,
	.status_mask = BIT(16),
	.parent = &lpxo_clk.c,
	.c = {
		.dbg_name = "pll4_clk",
		.ops = &clk_ops_pll_vote,
		CLK_INIT(pll4_clk.c),
	},
};

static struct clk_ops clk_ops_branch;

static struct clk_freq_tbl clk_tbl_axi[] = {
	F_RAW(1, &lpxo_clk.c, 0, 0, 0, 0, NOMINAL, NULL),
	F_END,
};

/* For global clocks to be on we must have GLBL_ROOT_ENA set */
static struct clk_local glbl_root_clk = {
	.b = {
		.en_reg = GLBL_CLK_ENA_SC_REG,
		.en_mask = BIT(29),
		.halt_check = NOCHECK,
	},
	.freq_tbl = clk_tbl_axi,
	.set_rate = set_rate_nop,
	.current_freq = &local_dummy_freq,
	.set_rate = set_rate_nop,
	.c = {
		.dbg_name = "glbl_root_clk",
		.ops = &soc_clk_ops_7x30,
		CLK_INIT(glbl_root_clk.c),
	},
};

/* AXI bridge clocks. */
static struct branch_clk axi_li_apps_clk = {
	.b = {
		.en_reg = GLBL_CLK_ENA_SC_REG,
		.en_mask = BIT(2),
		.halt_reg = GLBL_CLK_STATE_REG,
		.halt_check = HALT_VOTED,
		.halt_bit = 2,
		.test_vector = 0x4900,
	},
	.parent = &glbl_root_clk.c,
	.c = {
		.dbg_name = "axi_li_apps_clk",
		.ops = &clk_ops_branch,
		CLK_INIT(axi_li_apps_clk.c),
	},
};

static struct branch_clk axi_li_adsp_a_clk = {
	.b = {
		.en_reg = GLBL_CLK_ENA_2_SC_REG,
		.en_mask = BIT(14),
		.halt_reg = GLBL_CLK_STATE_2_REG,
		.halt_check = HALT_VOTED,
		.halt_bit = 14,
		.test_vector = 0x6400,
	},
	.parent = &axi_li_apps_clk.c,
	.c = {
		.dbg_name = "axi_li_adsp_a_clk",
		.ops = &clk_ops_branch,
		CLK_INIT(axi_li_adsp_a_clk.c),
	},
};

static struct branch_clk axi_li_jpeg_clk = {
	.b = {
		.en_reg = GLBL_CLK_ENA_2_SC_REG,
		.en_mask = BIT(19),
		.halt_reg = GLBL_CLK_STATE_2_REG,
		.halt_check = HALT_VOTED,
		.halt_bit = 19,
		.test_vector = 0x4E00,
	},
	.parent = &axi_li_apps_clk.c,
	.c = {
		.dbg_name = "axi_li_jpeg_clk",
		.ops = &clk_ops_branch,
		CLK_INIT(axi_li_jpeg_clk.c),
	},
};

static struct branch_clk axi_li_vfe_clk = {
	.b = {
		.en_reg = GLBL_CLK_ENA_SC_REG,
		.en_mask = BIT(23),
		.halt_reg = GLBL_CLK_STATE_REG,
		.halt_check = HALT_VOTED,
		.halt_bit = 23,
		.test_vector = 0x5B00,
	},
	.parent = &axi_li_apps_clk.c,
	.c = {
		.dbg_name = "axi_li_vfe_clk",
		.ops = &clk_ops_branch,
		CLK_INIT(axi_li_vfe_clk.c),
	},
};

static struct branch_clk axi_mdp_clk = {
	.b = {
		.en_reg = GLBL_CLK_ENA_2_SC_REG,
		.en_mask = BIT(29),
		.halt_reg = GLBL_CLK_STATE_2_REG,
		.halt_check = HALT_VOTED,
		.halt_bit = 29,
		.test_vector = 0x6B00,
	},
	.parent = &axi_li_apps_clk.c,
	.c = {
		.dbg_name = "axi_mdp_clk",
		.ops = &clk_ops_branch,
		CLK_INIT(axi_mdp_clk.c),
	},
};

static struct branch_clk axi_li_vg_clk = {
	.b = {
		.en_reg = GLBL_CLK_ENA_SC_REG,
		.en_mask = BIT(3),
		.halt_reg = GLBL_CLK_STATE_REG,
		.halt_check = HALT_VOTED,
		.halt_bit = 3,
		.test_vector = 0x4700,
	},
	.parent = &glbl_root_clk.c,
	.c = {
		.dbg_name = "axi_li_vg_clk",
		.ops = &clk_ops_branch,
		CLK_INIT(axi_li_vg_clk.c),
	},
};

static struct branch_clk axi_grp_2d_clk = {
	.b = {
		.en_reg = GLBL_CLK_ENA_SC_REG,
		.en_mask = BIT(21),
		.halt_reg = GLBL_CLK_STATE_REG,
		.halt_check = HALT_VOTED,
		.halt_bit = 21,
		.test_vector = 0x5900,
	},
	.parent = &axi_li_vg_clk.c,
	.c = {
		.dbg_name = "axi_grp_2d_clk",
		.ops = &clk_ops_branch,
		CLK_INIT(axi_grp_2d_clk.c),
	},
};

static struct branch_clk axi_li_grp_clk = {
	.b = {
		.en_reg = GLBL_CLK_ENA_SC_REG,
		.en_mask = BIT(22),
		.halt_reg = GLBL_CLK_STATE_REG,
		.halt_check = HALT_VOTED,
		.halt_bit = 22,
		.test_vector = 0x5A00,
	},
	.parent = &axi_li_vg_clk.c,
	.c = {
		.dbg_name = "axi_li_grp_clk",
		.ops = &clk_ops_branch,
		CLK_INIT(axi_li_grp_clk.c),
	},
};

static struct branch_clk axi_mfc_clk = {
	.b = {
		.en_reg = GLBL_CLK_ENA_2_SC_REG,
		.en_mask = BIT(20),
		.halt_reg = GLBL_CLK_STATE_2_REG,
		.halt_check = HALT_VOTED,
		.halt_bit = 20,
		.test_vector = 0x6A00,
	},
	.parent = &axi_li_vg_clk.c,
	.c = {
		.dbg_name = "axi_mfc_clk",
		.ops = &clk_ops_branch,
		CLK_INIT(axi_mfc_clk.c),
	},
};

static struct branch_clk axi_rotator_clk = {
	.b = {
		.en_reg = GLBL_CLK_ENA_2_SC_REG,
		.en_mask = BIT(22),
		.halt_reg = GLBL_CLK_STATE_2_REG,
		.halt_check = HALT_VOTED,
		.halt_bit = 22,
		.test_vector = 0x4300,
		.reset_mask = P_AXI_ROTATOR_CLK,
	},
	.parent = &axi_li_vg_clk.c,
	.c = {
		.dbg_name = "axi_rotator_clk",
		.ops = &clk_ops_branch,
		CLK_INIT(axi_rotator_clk.c),
	},
};

static struct branch_clk axi_vpe_clk = {
	.b = {
		.en_reg = GLBL_CLK_ENA_2_SC_REG,
		.en_mask = BIT(21),
		.halt_reg = GLBL_CLK_STATE_2_REG,
		.halt_check = HALT_VOTED,
		.halt_bit = 21,
		.test_vector = 0x6700,
	},
	.parent = &axi_li_vg_clk.c,
	.c = {
		.dbg_name = "axi_vpe_clk",
		.ops = &clk_ops_branch,
		CLK_INIT(axi_vpe_clk.c),
	},
};

/* Peripheral bus clocks. */
static struct branch_clk adm_clk = {
	.b = {
		.en_reg = GLBL_CLK_ENA_SC_REG,
		.en_mask = BIT(5),
		.halt_reg = GLBL_CLK_STATE_REG,
		.halt_check = HALT_VOTED,
		.halt_bit = 5,
		.test_vector = 0x4000,
		.reset_mask = P_ADM_CLK,
	},
	.parent = &axi_li_apps_clk.c,
	.c = {
		.dbg_name = "adm_clk",
		.ops = &clk_ops_branch,
		CLK_INIT(adm_clk.c),
	},
};

static struct branch_clk adm_p_clk = {
	.b = {
		.en_reg = GLBL_CLK_ENA_2_SC_REG,
		.en_mask = BIT(15),
		.halt_reg = GLBL_CLK_STATE_2_REG,
		.halt_check = HALT_VOTED,
		.halt_bit = 15,
		.test_vector = 0x11,
	},
	.parent = &glbl_root_clk.c,
	.c = {
		.dbg_name = "adm_p_clk",
		.ops = &clk_ops_branch,
		CLK_INIT(adm_p_clk.c),
	},
};

static struct branch_clk ce_clk = {
	.b = {
		.en_reg = GLBL_CLK_ENA_SC_REG,
		.en_mask = BIT(6),
		.halt_reg = GLBL_CLK_STATE_REG,
		.halt_check = HALT_VOTED,
		.halt_bit = 6,
		.test_vector = 0x4D43,
		.reset_mask = P_CE_CLK,
	},
	.parent = &glbl_root_clk.c,
	.c = {
		.dbg_name = "ce_clk",
		.ops = &clk_ops_branch,
		CLK_INIT(ce_clk.c),
	},
};

static struct branch_clk camif_pad_p_clk = {
	.b = {
		.en_reg = GLBL_CLK_ENA_SC_REG,
		.en_mask = BIT(9),
		.halt_reg = GLBL_CLK_STATE_REG,
		.halt_check = HALT_VOTED,
		.halt_bit = 9,
		.test_vector = 0x1A,
		.reset_mask = P_CAMIF_PAD_P_CLK,
	},
	.parent = &glbl_root_clk.c,
	.c = {
		.dbg_name = "camif_pad_p_clk",
		.flags = CLKFLAG_AUTO_OFF,
		.ops = &clk_ops_branch,
		CLK_INIT(camif_pad_p_clk.c),
	},
};

static struct branch_clk csi0_p_clk = {
	.b = {
		.en_reg = GLBL_CLK_ENA_SC_REG,
		.en_mask = BIT(30),
		.halt_reg = GLBL_CLK_STATE_REG,
		.halt_check = HALT_VOTED,
		.halt_bit = 30,
		.test_vector = 0x7300,
		.reset_mask = P_CSI0_P_CLK,
	},
	.parent = &glbl_root_clk.c,
	.c = {
		.dbg_name = "csi0_p_clk",
		.ops = &clk_ops_branch,
		CLK_INIT(csi0_p_clk.c),
	},
};

static struct branch_clk emdh_p_clk = {
	.b = {
		.en_reg = GLBL_CLK_ENA_2_SC_REG,
		.en_mask = BIT(3),
		.halt_reg = GLBL_CLK_STATE_2_REG,
		.halt_check = HALT_VOTED,
		.halt_bit = 3,
		.test_vector = 0x03,
		.reset_mask = P_EMDH_P_CLK,
	},
	.parent = &glbl_root_clk.c,
	.c = {
		.dbg_name = "emdh_p_clk",
		.flags = CLKFLAG_AUTO_OFF,
		.ops = &clk_ops_branch,
		CLK_INIT(emdh_p_clk.c),
	},
};

static struct branch_clk grp_2d_p_clk = {
	.b = {
		.en_reg = GLBL_CLK_ENA_SC_REG,
		.en_mask = BIT(24),
		.halt_reg = GLBL_CLK_STATE_REG,
		.halt_check = HALT_VOTED,
		.halt_bit = 24,
		.test_vector = 0x4D4C,
		.reset_mask = P_GRP_2D_P_CLK,
	},
	.parent = &glbl_root_clk.c,
	.c = {
		.dbg_name = "grp_2d_p_clk",
		.ops = &clk_ops_branch,
		CLK_INIT(grp_2d_p_clk.c),
	},
};

static struct branch_clk grp_3d_p_clk = {
	.b = {
		.en_reg = GLBL_CLK_ENA_2_SC_REG,
		.en_mask = BIT(17),
		.halt_reg = GLBL_CLK_STATE_2_REG,
		.halt_check = HALT_VOTED,
		.halt_bit = 17,
		.test_vector = 0x4D67,
		.reset_mask = P_GRP_3D_P_CLK,
	},
	.parent = &glbl_root_clk.c,
	.c = {
		.dbg_name = "grp_3d_p_clk",
		.ops = &clk_ops_branch,
		CLK_INIT(grp_3d_p_clk.c),
	},
};

static struct branch_clk jpeg_p_clk = {
	.b = {
		.en_reg = GLBL_CLK_ENA_2_SC_REG,
		.en_mask = BIT(24),
		.halt_reg = GLBL_CLK_STATE_2_REG,
		.halt_check = HALT_VOTED,
		.halt_bit = 24,
		.test_vector = 0x4D5E,
		.reset_mask = P_JPEG_P_CLK,
	},
	.parent = &glbl_root_clk.c,
	.c = {
		.dbg_name = "jpeg_p_clk",
		.flags = CLKFLAG_AUTO_OFF,
		.ops = &clk_ops_branch,
		CLK_INIT(jpeg_p_clk.c),
	},
};

static struct branch_clk lpa_p_clk = {
	.b = {
		.en_reg = GLBL_CLK_ENA_2_SC_REG,
		.en_mask = BIT(7),
		.halt_reg = GLBL_CLK_STATE_2_REG,
		.halt_check = HALT_VOTED,
		.halt_bit = 7,
		.test_vector = 0x07,
		.reset_mask = P_LPA_P_CLK,
	},
	.parent = &glbl_root_clk.c,
	.c = {
		.dbg_name = "lpa_p_clk",
		.ops = &clk_ops_branch,
		CLK_INIT(lpa_p_clk.c),
	},
};

static struct branch_clk mdp_p_clk = {
	.b = {
		.en_reg = GLBL_CLK_ENA_2_SC_REG,
		.en_mask = BIT(6),
		.halt_reg = GLBL_CLK_STATE_2_REG,
		.halt_check = HALT_VOTED,
		.halt_bit = 6,
		.test_vector = 0x06,
		.reset_mask = P_MDP_P_CLK,
	},
	.parent = &glbl_root_clk.c,
	.c = {
		.dbg_name = "mdp_p_clk",
		.ops = &clk_ops_branch,
		CLK_INIT(mdp_p_clk.c),
	},
};

static struct branch_clk mfc_p_clk = {
	.b = {
		.en_reg = GLBL_CLK_ENA_2_SC_REG,
		.en_mask = BIT(26),
		.halt_reg = GLBL_CLK_STATE_2_REG,
		.halt_check = HALT_VOTED,
		.halt_bit = 26,
		.test_vector = 0x4D75,
		.reset_mask = P_MFC_P_CLK,
	},
	.parent = &glbl_root_clk.c,
	.c = {
		.dbg_name = "mfc_p_clk",
		.ops = &clk_ops_branch,
		CLK_INIT(mfc_p_clk.c),
	},
};

static struct branch_clk pmdh_p_clk = {
	.b = {
		.en_reg = GLBL_CLK_ENA_2_SC_REG,
		.en_mask = BIT(4),
		.halt_reg = GLBL_CLK_STATE_2_REG,
		.halt_check = HALT_VOTED,
		.halt_bit = 4,
		.test_vector = 0x04,
		.reset_mask = P_PMDH_P_CLK,
	},
	.parent = &glbl_root_clk.c,
	.c = {
		.dbg_name = "pmdh_p_clk",
		.ops = &clk_ops_branch,
		CLK_INIT(pmdh_p_clk.c),
	},
};

static struct branch_clk rotator_imem_clk = {
	.b = {
		.en_reg = GLBL_CLK_ENA_2_SC_REG,
		.en_mask = BIT(23),
		.halt_reg = GLBL_CLK_STATE_2_REG,
		.halt_check = HALT_VOTED,
		.halt_bit = 23,
		.test_vector = 0x6600,
		.reset_mask = P_ROTATOR_IMEM_CLK,
	},
	.parent = &glbl_root_clk.c,
	.c = {
		.dbg_name = "rotator_imem_clk",
		.flags = CLKFLAG_AUTO_OFF,
		.ops = &clk_ops_branch,
		CLK_INIT(rotator_imem_clk.c),
	},
};

static struct branch_clk rotator_p_clk = {
	.b = {
		.en_reg = GLBL_CLK_ENA_2_SC_REG,
		.en_mask = BIT(25),
		.halt_reg = GLBL_CLK_STATE_2_REG,
		.halt_check = HALT_VOTED,
		.halt_bit = 25,
		.test_vector = 0x4D6D,
		.reset_mask = P_ROTATOR_P_CLK,
	},
	.parent = &glbl_root_clk.c,
	.c = {
		.dbg_name = "rotator_p_clk",
		.flags = CLKFLAG_AUTO_OFF,
		.ops = &clk_ops_branch,
		CLK_INIT(rotator_p_clk.c),
	},
};

static struct branch_clk sdc1_p_clk = {
	.b = {
		.en_reg = GLBL_CLK_ENA_SC_REG,
		.en_mask = BIT(7),
		.halt_reg = GLBL_CLK_STATE_REG,
		.halt_check = HALT_VOTED,
		.halt_bit = 7,
		.test_vector = 0x4D61,
		.reset_mask = P_SDC1_P_CLK,
	},
	.parent = &glbl_root_clk.c,
	.c = {
		.dbg_name = "sdc1_p_clk",
		.flags = CLKFLAG_AUTO_OFF,
		.ops = &clk_ops_branch,
		CLK_INIT(sdc1_p_clk.c),
	},
};

static struct branch_clk sdc2_p_clk = {
	.b = {
		.en_reg = GLBL_CLK_ENA_SC_REG,
		.en_mask = BIT(8),
		.halt_reg = GLBL_CLK_STATE_REG,
		.halt_check = HALT_VOTED,
		.halt_bit = 8,
		.test_vector = 0x4F63,
		.reset_mask = P_SDC2_P_CLK,
	},
	.parent = &glbl_root_clk.c,
	.c = {
		.dbg_name = "sdc2_p_clk",
		.flags = CLKFLAG_AUTO_OFF,
		.ops = &clk_ops_branch,
		CLK_INIT(sdc2_p_clk.c),
	},
};

static struct branch_clk sdc3_p_clk = {
	.b = {
		.en_reg = GLBL_CLK_ENA_SC_REG,
		.en_mask = BIT(27),
		.halt_reg = GLBL_CLK_STATE_REG,
		.halt_check = HALT_VOTED,
		.halt_bit = 27,
		.test_vector = 0x4D79,
		.reset_mask = P_SDC3_P_CLK,
	},
	.parent = &glbl_root_clk.c,
	.c = {
		.dbg_name = "sdc3_p_clk",
		.flags = CLKFLAG_AUTO_OFF,
		.ops = &clk_ops_branch,
		CLK_INIT(sdc3_p_clk.c),
	},
};

static struct branch_clk sdc4_p_clk = {
	.b = {
		.en_reg = GLBL_CLK_ENA_SC_REG,
		.en_mask = BIT(28),
		.halt_reg = GLBL_CLK_STATE_REG,
		.halt_check = HALT_VOTED,
		.halt_bit = 28,
		.test_vector = 0x4D7B,
		.reset_mask = P_SDC4_P_CLK,
	},
	.parent = &glbl_root_clk.c,
	.c = {
		.dbg_name = "sdc4_p_clk",
		.flags = CLKFLAG_AUTO_OFF,
		.ops = &clk_ops_branch,
		CLK_INIT(sdc4_p_clk.c),
	},
};

static struct branch_clk spi_p_clk = {
	.b = {
		.en_reg = GLBL_CLK_ENA_2_SC_REG,
		.en_mask = BIT(10),
		.halt_reg = GLBL_CLK_STATE_2_REG,
		.halt_check = HALT_VOTED,
		.halt_bit = 10,
		.test_vector = 0x18,
		.reset_mask = P_SPI_P_CLK,
	},
	.parent = &glbl_root_clk.c,
	.c = {
		.dbg_name = "spi_p_clk",
		.ops = &clk_ops_branch,
		CLK_INIT(spi_p_clk.c),
	},
};

static struct branch_clk tsif_p_clk = {
	.b = {
		.en_reg = GLBL_CLK_ENA_SC_REG,
		.en_mask = BIT(18),
		.halt_reg = GLBL_CLK_STATE_REG,
		.halt_check = HALT_VOTED,
		.halt_bit = 18,
		.test_vector = 0x4D65,
		.reset_mask = P_TSIF_P_CLK,
	},
	.parent = &glbl_root_clk.c,
	.c = {
		.dbg_name = "tsif_p_clk",
		.ops = &clk_ops_branch,
		CLK_INIT(tsif_p_clk.c),
	},
};

static struct branch_clk uart1dm_p_clk = {
	.b = {
		.en_reg = GLBL_CLK_ENA_SC_REG,
		.en_mask = BIT(17),
		.halt_reg = GLBL_CLK_STATE_REG,
		.halt_check = HALT_VOTED,
		.halt_bit = 17,
		.test_vector = 0x4D5C,
	},
	.parent = &glbl_root_clk.c,
	.c = {
		.dbg_name = "uart1dm_p_clk",
		.ops = &clk_ops_branch,
		CLK_INIT(uart1dm_p_clk.c),
	},
};

static struct branch_clk uart2dm_p_clk = {
	.b = {
		.en_reg = GLBL_CLK_ENA_SC_REG,
		.en_mask = BIT(26),
		.halt_reg = GLBL_CLK_STATE_REG,
		.halt_check = HALT_VOTED,
		.halt_bit = 26,
		.test_vector = 0x4D7E,
	},
	.parent = &glbl_root_clk.c,
	.c = {
		.dbg_name = "uart2dm_p_clk",
		.ops = &clk_ops_branch,
		CLK_INIT(uart2dm_p_clk.c),
	},
};

static struct branch_clk usb_hs2_p_clk = {
	.b = {
		.en_reg = GLBL_CLK_ENA_2_SC_REG,
		.en_mask = BIT(8),
		.halt_reg = GLBL_CLK_STATE_2_REG,
		.halt_check = HALT_VOTED,
		.halt_bit = 8,
		.test_vector = 0x08,
		.reset_mask = P_USB_HS2_P_CLK,
	},
	.parent = &glbl_root_clk.c,
	.c = {
		.dbg_name = "usb_hs2_p_clk",
		.flags = CLKFLAG_AUTO_OFF,
		.ops = &clk_ops_branch,
		CLK_INIT(usb_hs2_p_clk.c),
	},
};

static struct branch_clk usb_hs3_p_clk = {
	.b = {
		.en_reg = GLBL_CLK_ENA_2_SC_REG,
		.en_mask = BIT(9),
		.halt_reg = GLBL_CLK_STATE_2_REG,
		.halt_check = HALT_VOTED,
		.halt_bit = 9,
		.test_vector = 0x10,
		.reset_mask = P_USB_HS3_P_CLK,
	},
	.parent = &glbl_root_clk.c,
	.c = {
		.dbg_name = "usb_hs3_p_clk",
		.flags = CLKFLAG_AUTO_OFF,
		.ops = &clk_ops_branch,
		CLK_INIT(usb_hs3_p_clk.c),
	},
};

static struct branch_clk usb_hs_p_clk = {
	.b = {
		.en_reg = GLBL_CLK_ENA_SC_REG,
		.en_mask = BIT(25),
		.halt_reg = GLBL_CLK_STATE_REG,
		.halt_check = HALT_VOTED,
		.halt_bit = 25,
		.test_vector = 0x4D58,
		.reset_mask = P_USB_HS_P_CLK,
	},
	.parent = &glbl_root_clk.c,
	.c = {
		.dbg_name = "usb_hs_p_clk",
		.flags = CLKFLAG_AUTO_OFF,
		.ops = &clk_ops_branch,
		CLK_INIT(usb_hs_p_clk.c),
	},
};

static struct branch_clk vfe_p_clk = {
	.b = {
		.en_reg = GLBL_CLK_ENA_2_SC_REG,
		.en_mask = BIT(27),
		.halt_reg = GLBL_CLK_STATE_2_REG,
		.halt_check = HALT_VOTED,
		.halt_bit = 27,
		.test_vector = 0x4D55,
		.reset_mask = P_VFE_P_CLK,
	},
	.parent = &glbl_root_clk.c,
	.c = {
		.dbg_name = "vfe_p_clk",
		.flags = CLKFLAG_AUTO_OFF,
		.ops = &clk_ops_branch,
		CLK_INIT(vfe_p_clk.c),
	},
};

static struct clk_freq_tbl clk_tbl_csi[] = {
	F_MND8(        0,  0,  0, gnd,  1, 0, 0, NONE),
	F_MND8(153600000, 24, 17, pll1, 2, 2, 5, NOMINAL),
	F_MND8(192000000, 24, 17, pll1, 4, 0, 0, NOMINAL),
	F_MND8(384000000, 24, 17, pll1, 2, 0, 0, NOMINAL),
	F_END,
};

static struct clk_local csi0_clk = {
	.b = {
		.en_reg = CSI_NS_REG,
		.en_mask = BIT(9),
		.halt_reg = CLK_HALT_STATEC_REG,
		.halt_check = HALT,
		.halt_bit = 17,
		.test_vector = 0x7100,
		.reset_mask = P_CSI0_CLK,
	},
	.ns_reg = CSI_NS_REG,
	.md_reg = CSI_NS_REG - 4,
	.ns_mask = F_MASK_MND8(24, 17),
	.root_en_mask = BIT(11),
	.freq_tbl = clk_tbl_csi,
	.current_freq = &local_dummy_freq,
	.set_rate = set_rate_mnd,
	.c = {
		.dbg_name = "csi0_clk",
		.ops = &soc_clk_ops_7x30,
		CLK_INIT(csi0_clk.c),
	},
};

static struct clk_freq_tbl clk_tbl_tcxo[] = {
	F_RAW(19200000, &tcxo_clk.c, 0, 0, 0, 0, NOMINAL, NULL),
	F_END,
};

static struct clk_local i2c_clk = {
	.b = {
		.en_reg = I2C_NS_REG,
		.en_mask = BIT(9),
		.halt_reg = CLK_HALT_STATEA_REG,
		.halt_check = HALT,
		.halt_bit = 15,
		.test_vector = 0x4D4D,
		.reset_mask = P_I2C_CLK,
	},
	.set_rate = set_rate_nop,
	.freq_tbl = clk_tbl_tcxo,
	.root_en_mask = BIT(11),
	.current_freq = &local_dummy_freq,
	.c = {
		.dbg_name = "i2c_clk",
		.ops = &soc_clk_ops_7x30,
		CLK_INIT(i2c_clk.c),
	},
};

static struct clk_local i2c_2_clk = {
	.b = {
		.en_reg = I2C_2_NS_REG,
		.en_mask = BIT(0),
		.halt_reg = CLK_HALT_STATEC_REG,
		.halt_check = HALT,
		.halt_bit = 2,
		.test_vector = 0x0B,
		.reset_mask = P_I2C_2_CLK,
	},
	.root_en_mask = BIT(2),
	.freq_tbl = clk_tbl_tcxo,
	.set_rate = set_rate_nop,
	.current_freq = &local_dummy_freq,
	.c = {
		.dbg_name = "i2c_2_clk",
		.ops = &soc_clk_ops_7x30,
		CLK_INIT(i2c_2_clk.c),
	},
};

static struct clk_local qup_i2c_clk = {
	.b = {
		.en_reg = QUP_I2C_NS_REG,
		.en_mask = BIT(0),
		.halt_reg = CLK_HALT_STATEB_REG,
		.halt_check = HALT,
		.halt_bit = 31,
		.test_vector = 0x1C,
		.reset_mask = P_QUP_I2C_CLK,
	},
	.root_en_mask = BIT(2),
	.freq_tbl = clk_tbl_tcxo,
	.set_rate = set_rate_nop,
	.current_freq = &local_dummy_freq,
	.c = {
		.dbg_name = "qup_i2c_clk",
		.ops = &soc_clk_ops_7x30,
		CLK_INIT(qup_i2c_clk.c),
	},
};

static struct clk_local uart1_clk = {
	.b = {
		.en_reg = UART_NS_REG,
		.en_mask = BIT(5),
		.halt_reg = CLK_HALT_STATEB_REG,
		.halt_check = HALT,
		.halt_bit = 7,
		.test_vector = 0x4D6F,
		.reset_mask = P_UART1_CLK,
	},
	.root_en_mask = BIT(4),
	.freq_tbl = clk_tbl_tcxo,
	.set_rate = set_rate_nop,
	.current_freq = &local_dummy_freq,
	.c = {
		.dbg_name = "uart1_clk",
		.flags = CLKFLAG_AUTO_OFF,
		.ops = &soc_clk_ops_7x30,
		CLK_INIT(uart1_clk.c),
	},
};

static struct clk_local uart2_clk = {
	.b = {
		.en_reg = UART2_NS_REG,
		.en_mask = BIT(5),
		.halt_reg = CLK_HALT_STATEB_REG,
		.halt_check = HALT,
		.halt_bit = 5,
		.test_vector = 0x4D71,
		.reset_mask = P_UART2_CLK,
	},
	.root_en_mask = BIT(4),
	.freq_tbl = clk_tbl_tcxo,
	.set_rate = set_rate_nop,
	.current_freq = &local_dummy_freq,
	.c = {
		.dbg_name = "uart2_clk",
		.ops = &soc_clk_ops_7x30,
		CLK_INIT(uart2_clk.c),
	},
};

static struct clk_freq_tbl clk_tbl_uartdm[] = {
	F_MND16(       0, gnd,  1,   0,   0, NONE),
	F_MND16( 3686400, pll3, 3,   3, 200, NOMINAL),
	F_MND16( 7372800, pll3, 3,   3, 100, NOMINAL),
	F_MND16(14745600, pll3, 3,   3,  50, NOMINAL),
	F_MND16(32000000, pll3, 3,  25, 192, NOMINAL),
	F_MND16(40000000, pll3, 3, 125, 768, NOMINAL),
	F_MND16(46400000, pll3, 3, 145, 768, NOMINAL),
	F_MND16(48000000, pll3, 3,  25, 128, NOMINAL),
	F_MND16(51200000, pll3, 3,   5,  24, NOMINAL),
	F_MND16(56000000, pll3, 3, 175, 768, NOMINAL),
	F_MND16(58982400, pll3, 3,   6,  25, NOMINAL),
	F_MND16(64000000, pll1, 4,   1,   3, NOMINAL),
	F_END,
};

static struct clk_local uart1dm_clk = {
	.b = {
		.en_reg = UART1DM_NS_REG,
		.en_mask = BIT(9),
		.halt_reg = CLK_HALT_STATEB_REG,
		.halt_check = HALT,
		.halt_bit = 6,
		.test_vector = 0x4D70,
		.reset_mask = P_UART1DM_CLK,
	},
	.ns_reg = UART1DM_NS_REG,
	.md_reg = UART1DM_NS_REG - 4,
	.root_en_mask = BIT(11),
	.freq_tbl = clk_tbl_uartdm,
	.ns_mask = F_MASK_MND16,
	.current_freq = &local_dummy_freq,
	.set_rate = set_rate_mnd,
	.c = {
		.dbg_name = "uart1dm_clk",
		.flags = CLKFLAG_AUTO_OFF,
		.ops = &soc_clk_ops_7x30,
		CLK_INIT(uart1dm_clk.c),
	},
};

static struct clk_local uart2dm_clk = {
	.b = {
		.en_reg = UART2DM_NS_REG,
		.en_mask = BIT(9),
		.halt_reg = CLK_HALT_STATEB_REG,
		.halt_check = HALT,
		.halt_bit = 23,
		.test_vector = 0x4D7D,
		.reset_mask = P_UART2DM_CLK,
	},
	.ns_reg = UART2DM_NS_REG,
	.md_reg = UART2DM_NS_REG - 4,
	.root_en_mask = BIT(11),
	.freq_tbl = clk_tbl_uartdm,
	.ns_mask = F_MASK_MND16,
	.set_rate = set_rate_mnd,
	.current_freq = &local_dummy_freq,
	.c = {
		.dbg_name = "uart2dm_clk",
		.ops = &soc_clk_ops_7x30,
		CLK_INIT(uart2dm_clk.c),
	},
};

static struct clk_freq_tbl clk_tbl_mdh[] = {
	F_BASIC(        0, gnd,   1, NONE),
	F_BASIC( 49150000, pll3, 15, NOMINAL),
	F_BASIC( 92160000, pll3,  8, NOMINAL),
	F_BASIC(122880000, pll3,  6, NOMINAL),
	F_BASIC(184320000, pll3,  4, NOMINAL),
	F_BASIC(245760000, pll3,  3, NOMINAL),
	F_BASIC(368640000, pll3,  2, NOMINAL),
	F_BASIC(384000000, pll1,  2, NOMINAL),
	F_BASIC(445500000, pll4,  2, NOMINAL),
	F_END,
};

static struct clk_local emdh_clk = {
	.b = {
		.en_reg = EMDH_NS_REG,
		.halt_check = DELAY,
		.test_vector = 0x4F00,
		.reset_mask = P_EMDH_CLK,
	},
	.root_en_mask = BIT(11),
	.ns_reg = EMDH_NS_REG,
	.ns_mask = F_MASK_BASIC,
	.depends = &axi_li_adsp_a_clk.c,
	.set_rate = set_rate_nop,
	.freq_tbl = clk_tbl_mdh,
	.current_freq = &local_dummy_freq,
	.c = {
		.dbg_name = "emdh_clk",
		.flags = CLKFLAG_AUTO_OFF | CLKFLAG_MIN | CLKFLAG_MAX,
		.ops = &soc_clk_ops_7x30,
		CLK_INIT(emdh_clk.c),
	},
};

static struct clk_local pmdh_clk = {
	.b = {
		.en_reg = PMDH_NS_REG,
		.halt_check = DELAY,
		.test_vector = 0x5500,
		.reset_mask = P_PMDH_CLK,
	},
	.root_en_mask = BIT(11),
	.ns_reg = PMDH_NS_REG,
	.ns_mask = F_MASK_BASIC,
	.depends = &axi_li_adsp_a_clk.c,
	.set_rate = set_rate_nop,
	.freq_tbl = clk_tbl_mdh,
	.current_freq = &local_dummy_freq,
	.c = {
		.dbg_name = "pmdh_clk",
		.flags = CLKFLAG_AUTO_OFF | CLKFLAG_MIN | CLKFLAG_MAX,
		.ops = &soc_clk_ops_7x30,
		CLK_INIT(pmdh_clk.c),
	},
};

static struct clk_freq_tbl clk_tbl_grp[] = {
	F_BASIC( 24576000, lpxo,  1, NOMINAL),
	F_BASIC( 46080000, pll3, 16, NOMINAL),
	F_BASIC( 49152000, pll3, 15, NOMINAL),
	F_BASIC( 52662875, pll3, 14, NOMINAL),
	F_BASIC( 56713846, pll3, 13, NOMINAL),
	F_BASIC( 61440000, pll3, 12, NOMINAL),
	F_BASIC( 67025454, pll3, 11, NOMINAL),
	F_BASIC( 73728000, pll3, 10, NOMINAL),
	F_BASIC( 81920000, pll3,  9, NOMINAL),
	F_BASIC( 92160000, pll3,  8, NOMINAL),
	F_BASIC(105325714, pll3,  7, NOMINAL),
	F_BASIC(122880000, pll3,  6, NOMINAL),
	F_BASIC(147456000, pll3,  5, NOMINAL),
	F_BASIC(184320000, pll3,  4, NOMINAL),
	F_BASIC(192000000, pll1,  4, NOMINAL),
	F_BASIC(245760000, pll3,  3, HIGH),
	/* Sync to AXI. Hence this "rate" is not fixed. */
	F_RAW(1, &lpxo_clk.c, 0, BIT(14), 0, 0, NOMINAL, NULL),
	F_END,
};

static struct clk_local grp_2d_clk = {
	.b = {
		.en_reg = GRP_2D_NS_REG,
		.en_mask = BIT(7),
		.halt_reg = CLK_HALT_STATEA_REG,
		.halt_check = HALT,
		.halt_bit = 31,
		.test_vector = 0x5C00,
		.reset_mask = P_GRP_2D_CLK,
	},
	.ns_reg = GRP_2D_NS_REG,
	.root_en_mask = BIT(11),
	.ns_mask = F_MASK_BASIC | (7 << 12),
	.set_rate = set_rate_nop,
	.freq_tbl = clk_tbl_grp,
	.depends = &axi_grp_2d_clk.c,
	.current_freq = &local_dummy_freq,
	.c = {
		.dbg_name = "grp_2d_clk",
		.ops = &soc_clk_ops_7x30,
		CLK_INIT(grp_2d_clk.c),
	},
};

static struct clk_local grp_3d_src_clk = {
	.ns_reg = GRP_NS_REG,
	.b = {
		.en_reg = GRP_NS_REG,
	},
	.root_en_mask = BIT(11),
	.ns_mask = F_MASK_BASIC | (7 << 12),
	.set_rate = set_rate_nop,
	.freq_tbl = clk_tbl_grp,
	.depends = &axi_li_grp_clk.c,
	.current_freq = &local_dummy_freq,
	.c = {
		.dbg_name = "grp_3d_src_clk",
		.ops = &soc_clk_ops_7x30,
		CLK_INIT(grp_3d_src_clk.c),
	},
};

static struct branch_clk grp_3d_clk = {
	.b = {
		.en_reg = GRP_NS_REG,
		.en_mask = BIT(7),
		.halt_reg = CLK_HALT_STATEB_REG,
		.halt_check = HALT,
		.halt_bit = 18,
		.test_vector = 0x5E00,
		.reset_mask = P_GRP_3D_CLK,
	},
	.parent = &grp_3d_src_clk.c,
	.c = {
		.dbg_name = "grp_3d_clk",
		.ops = &clk_ops_branch,
		CLK_INIT(grp_3d_clk.c),
	},
};

static struct branch_clk imem_clk = {
	.b = {
		.en_reg = GRP_NS_REG,
		.en_mask = BIT(9),
		.halt_reg = CLK_HALT_STATEB_REG,
		.halt_check = HALT,
		.halt_bit = 19,
		.test_vector = 0x5F00,
		.reset_mask = P_IMEM_CLK,
	},
	.parent = &grp_3d_src_clk.c,
	.c = {
		.dbg_name = "imem_clk",
		.flags = CLKFLAG_AUTO_OFF,
		.ops = &clk_ops_branch,
		CLK_INIT(imem_clk.c),
	},
};

static struct clk_freq_tbl clk_tbl_sdc1_3[] = {
	F_MND8(       0,  0,  0, gnd,  1,   0,  0,   NONE),
	F_MND8(  144000, 19, 12, lpxo, 1,   1,  171, NOMINAL),
	F_MND8(  400000, 19, 12, lpxo, 1,   2,  123, NOMINAL),
	F_MND8(16027000, 19, 12, pll3, 3,  14,  215, NOMINAL),
	F_MND8(17000000, 19, 12, pll3, 4,  19,  206, NOMINAL),
	F_MND8(20480000, 19, 12, pll3, 4,  23,  212, NOMINAL),
	F_MND8(24576000, 19, 12, lpxo, 1,   0,    0, NOMINAL),
	F_MND8(49152000, 19, 12, pll3, 3,   1,    5, NOMINAL),
	F_END,
};

static struct clk_local sdc1_clk = {
	.b = {
		.en_reg = SDCn_NS_REG(1),
		.en_mask = BIT(9),
		.halt_reg = CLK_HALT_STATEA_REG,
		.halt_check = HALT,
		.halt_bit = 1,
		.test_vector = 0x4D62,
		.reset_mask = P_SDC1_CLK,
	},
	.ns_reg = SDCn_NS_REG(1),
	.md_reg = SDCn_NS_REG(1) - 4,
	.ns_mask = F_MASK_MND8(19, 12),
	.root_en_mask = BIT(11),
	.freq_tbl = clk_tbl_sdc1_3,
	.current_freq = &local_dummy_freq,
	.set_rate = set_rate_mnd,
	.c = {
		.dbg_name = "sdc1_clk",
		.flags = CLKFLAG_AUTO_OFF,
		.ops = &soc_clk_ops_7x30,
		CLK_INIT(sdc1_clk.c),
	},
};

static struct clk_local sdc3_clk = {
	.b = {
		.en_reg = SDCn_NS_REG(3),
		.en_mask = BIT(9),
		.halt_reg = CLK_HALT_STATEB_REG,
		.halt_check = HALT,
		.halt_bit = 24,
		.test_vector = 0x4D7A,
		.reset_mask = P_SDC3_CLK,
	},
	.ns_reg = SDCn_NS_REG(3),
	.md_reg = SDCn_NS_REG(3) - 4,
	.ns_mask = F_MASK_MND8(19, 12),
	.root_en_mask = BIT(11),
	.freq_tbl = clk_tbl_sdc1_3,
	.current_freq = &local_dummy_freq,
	.set_rate = set_rate_mnd,
	.c = {
		.dbg_name = "sdc3_clk",
		.flags = CLKFLAG_AUTO_OFF,
		.ops = &soc_clk_ops_7x30,
		CLK_INIT(sdc3_clk.c),
	},
};

static struct clk_freq_tbl clk_tbl_sdc2_4[] = {
	F_MND8(       0,  0,  0, gnd,  1,   0,  0,   NONE),
	F_MND8(  144000, 20, 13, lpxo, 1,   1,  171, NOMINAL),
	F_MND8(  400000, 20, 13, lpxo, 1,   2,  123, NOMINAL),
	F_MND8(16027000, 20, 13, pll3, 3,  14,  215, NOMINAL),
	F_MND8(17000000, 20, 13, pll3, 4,  19,  206, NOMINAL),
	F_MND8(20480000, 20, 13, pll3, 4,  23,  212, NOMINAL),
	F_MND8(24576000, 20, 13, lpxo, 1,   0,    0, NOMINAL),
	F_MND8(49152000, 20, 13, pll3, 3,   1,    5, NOMINAL),
	F_END,
};

static struct clk_local sdc2_clk = {
	.b = {
		.en_reg = SDCn_NS_REG(2),
		.en_mask = BIT(9),
		.halt_reg = CLK_HALT_STATEA_REG,
		.halt_check = HALT,
		.halt_bit = 0,
		.test_vector = 0x4D64,
		.reset_mask = P_SDC2_CLK,
	},
	.ns_reg = SDCn_NS_REG(2),
	.md_reg = SDCn_NS_REG(2) - 4,
	.ns_mask = F_MASK_MND8(20, 13),
	.root_en_mask = BIT(11),
	.freq_tbl = clk_tbl_sdc2_4,
	.current_freq = &local_dummy_freq,
	.set_rate = set_rate_mnd,
	.c = {
		.dbg_name = "sdc2_clk",
		.flags = CLKFLAG_AUTO_OFF,
		.ops = &soc_clk_ops_7x30,
		CLK_INIT(sdc2_clk.c),
	},
};

static struct clk_local sdc4_clk = {
	.b = {
		.en_reg = SDCn_NS_REG(4),
		.en_mask = BIT(9),
		.halt_reg = CLK_HALT_STATEB_REG,
		.halt_check = HALT,
		.halt_bit = 25,
		.test_vector = 0x4D7C,
		.reset_mask = P_SDC4_CLK,
	},
	.ns_reg = SDCn_NS_REG(4),
	.md_reg = SDCn_NS_REG(4) - 4,
	.ns_mask = F_MASK_MND8(20, 13),
	.root_en_mask = BIT(11),
	.freq_tbl = clk_tbl_sdc2_4,
	.current_freq = &local_dummy_freq,
	.set_rate = set_rate_mnd,
	.c = {
		.dbg_name = "sdc4_clk",
		.flags = CLKFLAG_AUTO_OFF,
		.ops = &soc_clk_ops_7x30,
		CLK_INIT(sdc4_clk.c),
	},
};

static struct clk_freq_tbl clk_tbl_mdp_core[] = {
	F_BASIC( 24576000, lpxo,  1, NOMINAL),
	F_BASIC( 46080000, pll3, 16, NOMINAL),
	F_BASIC( 49152000, pll3, 15, NOMINAL),
	F_BASIC( 52663000, pll3, 14, NOMINAL),
	F_BASIC( 92160000, pll3,  8, NOMINAL),
	F_BASIC(122880000, pll3,  6, NOMINAL),
	F_BASIC(147456000, pll3,  5, NOMINAL),
	F_BASIC(153600000, pll1,  5, NOMINAL),
	F_BASIC(192000000, pll1,  4, HIGH),
	F_END,
};

static struct clk_local mdp_clk = {
	.b = {
		.en_reg = MDP_NS_REG,
		.en_mask = BIT(9),
		.halt_reg = CLK_HALT_STATEB_REG,
		.halt_check = HALT,
		.halt_bit = 16,
		.test_vector = 0x5400,
		.reset_mask = P_MDP_CLK,
	},
	.ns_reg = MDP_NS_REG,
	.root_en_mask = BIT(11),
	.ns_mask = F_MASK_BASIC,
	.depends = &axi_mdp_clk.c,
	.set_rate = set_rate_nop,
	.freq_tbl = clk_tbl_mdp_core,
	.current_freq = &local_dummy_freq,
	.c = {
		.dbg_name = "mdp_clk",
		.flags = CLKFLAG_AUTO_OFF,
		.ops = &soc_clk_ops_7x30,
		CLK_INIT(mdp_clk.c),
	},
};

static struct clk_freq_tbl clk_tbl_mdp_lcdc[] = {
	F_MND16(       0, gnd,  1,   0,   0, NONE),
	F_MND16(24576000, lpxo, 1,   0,   0, NOMINAL),
	F_MND16(30720000, pll3, 4,   1,   6, NOMINAL),
	F_MND16(32768000, pll3, 3,   2,  15, NOMINAL),
	F_MND16(40960000, pll3, 2,   1,   9, NOMINAL),
	F_MND16(73728000, pll3, 2,   1,   5, NOMINAL),
	F_END,
};

static struct clk_local mdp_lcdc_pclk_clk = {
	.b = {
		.en_reg = MDP_LCDC_NS_REG,
		.en_mask = BIT(9),
		.halt_reg = CLK_HALT_STATEB_REG,
		.halt_check = HALT,
		.halt_bit = 28,
		.test_vector = 0x4200,
		.reset_mask = P_MDP_LCDC_PCLK_CLK,
	},
	.ns_reg = MDP_LCDC_NS_REG,
	.md_reg = MDP_LCDC_NS_REG - 4,
	.root_en_mask = BIT(11),
	.ns_mask = F_MASK_MND16,
	.set_rate = set_rate_mnd,
	.freq_tbl = clk_tbl_mdp_lcdc,
	.current_freq = &local_dummy_freq,
	.c = {
		.dbg_name = "mdp_lcdc_pclk_clk",
		.ops = &soc_clk_ops_7x30,
		CLK_INIT(mdp_lcdc_pclk_clk.c),
	},
};

static struct branch_clk mdp_lcdc_pad_pclk_clk = {
	.b = {
		.en_reg = MDP_LCDC_NS_REG,
		.en_mask = BIT(12),
		.halt_reg = CLK_HALT_STATEB_REG,
		.halt_check = HALT,
		.halt_bit = 29,
		.test_vector = 0x4100,
		.reset_mask = P_MDP_LCDC_PAD_PCLK_CLK,
	},
	.parent = &mdp_lcdc_pclk_clk.c,
	.c = {
		.dbg_name = "mdp_lcdc_pad_pclk_clk",
		.ops = &clk_ops_branch,
		CLK_INIT(mdp_lcdc_pad_pclk_clk.c),
	},
};

static struct clk_freq_tbl clk_tbl_mdp_vsync[] = {
	F_RAW(       0, &gnd_clk.c,  0, (0x3<<2), 0, 0, NONE,    NULL),
	F_RAW(24576000, &lpxo_clk.c, 0, (0x1<<2), 0, 0, NOMINAL, NULL),
	F_END,
};

static struct clk_local mdp_vsync_clk = {
	.b = {
		.en_reg = MDP_VSYNC_REG,
		.en_mask = BIT(0),
		.halt_reg = CLK_HALT_STATEB_REG,
		.halt_check = HALT,
		.halt_bit = 30,
		.test_vector = 0x4D53,
		.reset_mask = P_MDP_VSYNC_CLK,
	},
	.ns_reg = MDP_VSYNC_REG,
	.ns_mask = BM(3, 2),
	.freq_tbl = clk_tbl_mdp_vsync,
	.set_rate = set_rate_nop,
	.current_freq = &local_dummy_freq,
	.c = {
		.dbg_name = "mdp_vsync_clk",
		.flags = CLKFLAG_AUTO_OFF,
		.ops = &soc_clk_ops_7x30,
		CLK_INIT(mdp_vsync_clk.c),
	},
};

static struct clk_freq_tbl clk_tbl_mi2s_codec[] = {
	F_MND16(       0, gnd,  1,   0,   0, NONE),
	F_MND16( 2048000, lpxo, 4,   1,   3, NOMINAL),
	F_MND16(12288000, lpxo, 2,   0,   0, NOMINAL),
	F_END,
};

static struct clk_local mi2s_codec_rx_m_clk = {
	.b = {
		.en_reg = MI2S_RX_NS_REG,
		.en_mask = BIT(12),
		.halt_reg = CLK_HALT_STATEA_REG,
		.halt_check = HALT,
		.halt_bit = 12,
		.test_vector = 0x4D4E,
		.reset_mask = P_MI2S_CODEC_RX_M_CLK,
	},
	.ns_reg = MI2S_RX_NS_REG,
	.md_reg = MI2S_RX_NS_REG - 4,
	.root_en_mask = BIT(11),
	.ns_mask = F_MASK_MND16,
	.set_rate = set_rate_mnd,
	.freq_tbl = clk_tbl_mi2s_codec,
	.current_freq = &local_dummy_freq,
	.c = {
		.dbg_name = "mi2s_codec_rx_m_clk",
		.ops = &soc_clk_ops_7x30,
		CLK_INIT(mi2s_codec_rx_m_clk.c),
	},
};

static struct branch_clk mi2s_codec_rx_s_clk = {
	.b = {
		.en_reg = MI2S_RX_NS_REG,
		.en_mask = BIT(9),
		.halt_reg = CLK_HALT_STATEA_REG,
		.halt_check = HALT,
		.halt_bit = 13,
		.test_vector = 0x4D4F,
		.reset_mask = P_MI2S_CODEC_RX_S_CLK,
	},
	.parent = &mi2s_codec_rx_m_clk.c,
	.c = {
		.dbg_name = "mi2s_codec_rx_s_clk",
		.ops = &clk_ops_branch,
		CLK_INIT(mi2s_codec_rx_s_clk.c),
	},
};

static struct clk_local mi2s_codec_tx_m_clk = {
	.b = {
		.en_reg = MI2S_TX_NS_REG,
		.en_mask = BIT(12),
		.halt_reg = CLK_HALT_STATEC_REG,
		.halt_check = HALT,
		.halt_bit = 8,
		.test_vector = 0x4D50,
		.reset_mask = P_MI2S_CODEC_TX_M_CLK,
	},
	.ns_reg = MI2S_TX_NS_REG,
	.md_reg = MI2S_TX_NS_REG - 4,
	.root_en_mask = BIT(11),
	.ns_mask = F_MASK_MND16,
	.set_rate = set_rate_mnd,
	.freq_tbl = clk_tbl_mi2s_codec,
	.current_freq = &local_dummy_freq,
	.c = {
		.dbg_name = "mi2s_codec_tx_m_clk",
		.ops = &soc_clk_ops_7x30,
		CLK_INIT(mi2s_codec_tx_m_clk.c),
	},
};

static struct branch_clk mi2s_codec_tx_s_clk = {
	.b = {
		.en_reg = MI2S_TX_NS_REG,
		.en_mask = BIT(9),
		.halt_reg = CLK_HALT_STATEA_REG,
		.halt_check = HALT,
		.halt_bit = 11,
		.test_vector = 0x17,
		.reset_mask = P_MI2S_CODEC_TX_S_CLK,
	},
	.parent = &mi2s_codec_tx_m_clk.c,
	.c = {
		.dbg_name = "mi2s_codec_tx_s_clk",
		.ops = &clk_ops_branch,
		CLK_INIT(mi2s_codec_tx_s_clk.c),
	},
};

static struct clk_freq_tbl clk_tbl_mi2s[] = {
	F_MND16(       0, gnd,  1,   0,   0, NONE),
	F_MND16(12288000, lpxo, 2,   0,   0, NOMINAL),
	F_END,
};

static struct clk_local mi2s_m_clk = {
	.b = {
		.en_reg = MI2S_NS_REG,
		.en_mask = BIT(12),
		.halt_reg = CLK_HALT_STATEC_REG,
		.halt_check = HALT,
		.halt_bit = 4,
		.test_vector = 0x0D,
		.reset_mask = P_MI2S_M_CLK,
	},
	.ns_reg = MI2S_NS_REG,
	.md_reg = MI2S_NS_REG - 4,
	.root_en_mask = BIT(11),
	.ns_mask = F_MASK_MND16,
	.set_rate = set_rate_mnd,
	.freq_tbl = clk_tbl_mi2s,
	.current_freq = &local_dummy_freq,
	.c = {
		.dbg_name = "mi2s_m_clk",
		.ops = &soc_clk_ops_7x30,
		CLK_INIT(mi2s_m_clk.c),
	},
};

static struct branch_clk mi2s_s_clk = {
	.b = {
		.en_reg = MI2S_NS_REG,
		.en_mask = BIT(9),
		.halt_reg = CLK_HALT_STATEC_REG,
		.halt_check = HALT,
		.halt_bit = 3,
		.test_vector = 0,
		.reset_mask = P_MI2S_S_CLK,
	},
	.parent = &mi2s_m_clk.c,
	.c = {
		.dbg_name = "mi2s_s_clk",
		.ops = &clk_ops_branch,
		CLK_INIT(mi2s_s_clk.c),
	},
};

static struct clk_freq_tbl clk_tbl_midi[] = {
	F_MND8(       0,  0,  0, gnd,  1,  0,  0, NONE),
	F_MND8(98304000, 19, 12, pll3, 3,  2,  5, NOMINAL),
	F_END,
};

static struct clk_local midi_clk = {
	.b = {
		.en_reg = MIDI_NS_REG,
		.en_mask = BIT(9),
		.halt_reg = CLK_HALT_STATEC_REG,
		.halt_check = HALT,
		.halt_bit = 1,
		.test_vector = 0x0A,
	},
	.ns_reg = MIDI_NS_REG,
	.md_reg = MIDI_NS_REG - 4,
	.ns_mask = F_MASK_MND8(19, 12),
	.root_en_mask = BIT(11),
	.freq_tbl = clk_tbl_midi,
	.current_freq = &local_dummy_freq,
	.set_rate = set_rate_mnd,
	.c = {
		.dbg_name = "midi_clk",
		.ops = &soc_clk_ops_7x30,
		CLK_INIT(midi_clk.c),
	},
};

#define F_SDAC(f, s, div, m, n, v) \
	{ \
		.freq_hz = f, \
		.md_val = MD16(m, n), \
		.ns_val = N16(m, n) | SPDIV(SRC_SEL_SDAC_##s, div), \
		.mnd_en_mask = BIT(8) * !!(n), \
		.sys_vdd = v, \
		.src_clk = &s##_clk.c, \
	}

static struct clk_freq_tbl clk_tbl_sdac[] = {
	F_SDAC( 256000, lpxo, 4,   1,    24, NOMINAL),
	F_SDAC( 352800, lpxo, 1, 147, 10240, NOMINAL),
	F_SDAC( 384000, lpxo, 4,   1,    16, NOMINAL),
	F_SDAC( 512000, lpxo, 4,   1,    12, NOMINAL),
	F_SDAC( 705600, lpxo, 1, 147,  5120, NOMINAL),
	F_SDAC( 768000, lpxo, 4,   1,     8, NOMINAL),
	F_SDAC(1024000, lpxo, 4,   1,     6, NOMINAL),
	F_SDAC(1411200, lpxo, 1, 147,  2560, NOMINAL),
	F_SDAC(1536000, lpxo, 4,   1,     4, NOMINAL),
	F_END,
};

static struct clk_local sdac_clk = {
	.b = {
		.en_reg = SDAC_NS_REG,
		.en_mask = BIT(9),
		.halt_reg = CLK_HALT_STATEA_REG,
		.halt_check = HALT,
		.halt_bit = 2,
		.test_vector = 0x4D60,
		.reset_mask = P_SDAC_CLK,
	},
	.ns_reg = SDAC_NS_REG,
	.md_reg = SDAC_NS_REG - 4,
	.root_en_mask = BIT(11),
	.freq_tbl = clk_tbl_sdac,
	.ns_mask = F_MASK_MND16,
	.set_rate = set_rate_mnd,
	.current_freq = &local_dummy_freq,
	.c = {
		.dbg_name = "sdac_clk",
		.flags = CLKFLAG_AUTO_OFF,
		.ops = &soc_clk_ops_7x30,
		CLK_INIT(sdac_clk.c),
	},
};

static struct branch_clk sdac_m_clk = {
	.b = {
		.en_reg = SDAC_NS_REG,
		.en_mask = BIT(12),
		.halt_reg = CLK_HALT_STATEB_REG,
		.halt_check = HALT,
		.halt_bit = 17,
		.test_vector = 0x4D66,
		.reset_mask = P_SDAC_M_CLK,
	},
	.parent = &sdac_clk.c,
	.c = {
		.dbg_name = "sdac_m_clk",
		.flags = CLKFLAG_AUTO_OFF,
		.ops = &clk_ops_branch,
		CLK_INIT(sdac_m_clk.c),
	},
};

static struct clk_freq_tbl clk_tbl_tv[] = {
	F_MND8(       0,  0,  0, gnd,  1,  0,   0, NONE),
	F_MND8(27000000, 23, 16, pll4, 2,  2,  33, NOMINAL),
	F_MND8(74250000, 23, 16, pll4, 2,  1,   6, NOMINAL),
	F_END,
};

static struct clk_local tv_clk = {
	.ns_reg = TV_NS_REG,
	.b = {
		.en_reg = TV_NS_REG,
	},
	.md_reg = TV_NS_REG - 4,
	.ns_mask = F_MASK_MND8(23, 16),
	.root_en_mask = BIT(11),
	.freq_tbl = clk_tbl_tv,
	.current_freq = &local_dummy_freq,
	.set_rate = set_rate_mnd,
	.c = {
		.dbg_name = "tv_clk",
		.ops = &soc_clk_ops_7x30,
		CLK_INIT(tv_clk.c),
	},
};

static struct branch_clk hdmi_clk = {
	.b = {
		.en_reg = HDMI_NS_REG,
		.en_mask = BIT(9),
		.halt_reg = CLK_HALT_STATEC_REG,
		.halt_check = HALT,
		.halt_bit = 7,
		.test_vector = 0x13,
		.reset_mask = P_HDMI_CLK,
	},
	.parent = &tv_clk.c,
	.c = {
		.dbg_name = "hdmi_clk",
		.ops = &clk_ops_branch,
		CLK_INIT(hdmi_clk.c),
	},
};

static struct branch_clk tv_dac_clk = {
	.b = {
		.en_reg = TV_NS_REG,
		.en_mask = BIT(12),
		.halt_reg = CLK_HALT_STATEB_REG,
		.halt_check = HALT,
		.halt_bit = 27,
		.test_vector = 0x4D6C,
		.reset_mask = P_TV_DAC_CLK,
	},
	.parent = &tv_clk.c,
	.c = {
		.dbg_name = "tv_dac_clk",
		.ops = &clk_ops_branch,
		CLK_INIT(tv_dac_clk.c),
	},
};

static struct branch_clk tv_enc_clk = {
	.b = {
		.en_reg = TV_NS_REG,
		.en_mask = BIT(9),
		.halt_reg = CLK_HALT_STATEB_REG,
		.halt_check = HALT,
		.halt_bit = 10,
		.test_vector = 0x4D6B,
		.reset_mask = P_TV_ENC_CLK,
	},
	.parent = &tv_clk.c,
	.c = {
		.dbg_name = "tv_enc_clk",
		.ops = &clk_ops_branch,
		CLK_INIT(tv_enc_clk.c),
	},
};

/* Hacking root & branch into one param. */
static struct branch_clk tsif_ref_clk = {
	.b = {
		.en_reg = TSIF_NS_REG,
		.en_mask = BIT(9)|BIT(11),
		.halt_reg = CLK_HALT_STATEB_REG,
		.halt_check = HALT,
		.halt_bit = 11,
		.test_vector = 0x4D6A,
		.reset_mask = P_TSIF_REF_CLK,
	},
	.parent = &tv_clk.c,
	.c = {
		.dbg_name = "tsif_ref_clk",
		.ops = &clk_ops_branch,
		CLK_INIT(tsif_ref_clk.c),
	},
};

static struct clk_freq_tbl clk_tbl_usb[] = {
	F_MND8(       0,  0,  0, gnd,  1,  0,  0,  NONE),
	F_MND8(60000000, 23, 16, pll1, 2,  5,  32, NOMINAL),
	F_END,
};

static struct clk_local usb_hs_src_clk = {
	.ns_reg = USBH_NS_REG,
	.b = {
		.en_reg = USBH_NS_REG,
	},
	.md_reg = USBH_NS_REG - 4,
	.ns_mask = F_MASK_MND8(23, 16),
	.root_en_mask = BIT(11),
	.freq_tbl = clk_tbl_usb,
	.current_freq = &local_dummy_freq,
	.depends = &axi_li_adsp_a_clk.c,
	.set_rate = set_rate_mnd,
	.c = {
		.dbg_name = "usb_hs_src_clk",
		.ops = &soc_clk_ops_7x30,
		CLK_INIT(usb_hs_src_clk.c),
	},
};

static struct branch_clk usb_hs_clk = {
	.b = {
		.en_reg = USBH_NS_REG,
		.en_mask = BIT(9),
		.halt_reg = CLK_HALT_STATEB_REG,
		.halt_check = HALT,
		.halt_bit = 26,
		.test_vector = 0x4D7F,
		.reset_mask = P_USB_HS_CLK,
	},
	.c = {
		.dbg_name = "usb_hs_clk",
		.flags = CLKFLAG_AUTO_OFF,
		.ops = &clk_ops_branch,
		CLK_INIT(usb_hs_clk.c),
	},
};

static struct branch_clk usb_hs_core_clk = {
	.b = {
		.en_reg = USBH_NS_REG,
		.en_mask = BIT(13),
		.halt_reg = CLK_HALT_STATEA_REG,
		.halt_check = HALT,
		.halt_bit = 27,
		.test_vector = 0x14,
		.reset_mask = P_USB_HS_CORE_CLK,
	},
	.parent = &usb_hs_src_clk.c,
	.c = {
		.dbg_name = "usb_hs_core_clk",
		.flags = CLKFLAG_AUTO_OFF,
		.ops = &clk_ops_branch,
		CLK_INIT(usb_hs_core_clk.c),
	},
};

static struct branch_clk usb_hs2_clk = {
	.b = {
		.en_reg = USBH2_NS_REG,
		.en_mask = BIT(9),
		.halt_reg = CLK_HALT_STATEB_REG,
		.halt_check = HALT,
		.halt_bit = 3,
		.test_vector = 0x4D73,
		.reset_mask = P_USB_HS2_CLK,
	},
	.parent = &usb_hs_src_clk.c,
	.c = {
		.dbg_name = "usb_hs2_clk",
		.flags = CLKFLAG_AUTO_OFF,
		.ops = &clk_ops_branch,
		CLK_INIT(usb_hs2_clk.c),
	},
};

static struct branch_clk usb_hs2_core_clk = {
	.b = {
		.en_reg = USBH2_NS_REG,
		.en_mask = BIT(4),
		.halt_reg = CLK_HALT_STATEA_REG,
		.halt_check = HALT,
		.halt_bit = 28,
		.test_vector = 0x15,
		.reset_mask = P_USB_HS2_CORE_CLK,
	},
	.parent = &usb_hs_src_clk.c,
	.c = {
		.dbg_name = "usb_hs2_core_clk",
		.flags = CLKFLAG_AUTO_OFF,
		.ops = &clk_ops_branch,
		CLK_INIT(usb_hs2_core_clk.c),
	},
};

static struct branch_clk usb_hs3_clk = {
	.b = {
		.en_reg = USBH3_NS_REG,
		.en_mask = BIT(9),
		.halt_reg = CLK_HALT_STATEB_REG,
		.halt_check = HALT,
		.halt_bit = 2,
		.test_vector = 0x4D74,
		.reset_mask = P_USB_HS3_CLK,
	},
	.parent = &usb_hs_src_clk.c,
	.c = {
		.dbg_name = "usb_hs3_clk",
		.flags = CLKFLAG_AUTO_OFF,
		.ops = &clk_ops_branch,
		CLK_INIT(usb_hs3_clk.c),
	},
};

static struct branch_clk usb_hs3_core_clk = {
	.b = {
		.en_reg = USBH3_NS_REG,
		.en_mask = BIT(4),
		.halt_reg = CLK_HALT_STATEA_REG,
		.halt_check = HALT,
		.halt_bit = 29,
		.test_vector = 0x16,
		.reset_mask = P_USB_HS3_CORE_CLK,
	},
	.parent = &usb_hs_src_clk.c,
	.c = {
		.dbg_name = "usb_hs3_core_clk",
		.flags = CLKFLAG_AUTO_OFF,
		.ops = &clk_ops_branch,
		CLK_INIT(usb_hs3_core_clk.c),
	},
};

static struct clk_freq_tbl clk_tbl_vfe_jpeg[] = {
	F_MND16( 24576000, lpxo, 1,   0,   0, NOMINAL),
	F_MND16( 36864000, pll3, 4,   1,   5, NOMINAL),
	F_MND16( 46080000, pll3, 4,   1,   4, NOMINAL),
	F_MND16( 61440000, pll3, 4,   1,   3, NOMINAL),
	F_MND16( 73728000, pll3, 2,   1,   5, NOMINAL),
	F_MND16( 81920000, pll3, 3,   1,   3, NOMINAL),
	F_MND16( 92160000, pll3, 4,   1,   2, NOMINAL),
	F_MND16( 98304000, pll3, 3,   2,   5, NOMINAL),
	F_MND16(105326000, pll3, 2,   2,   7, NOMINAL),
	F_MND16(122880000, pll3, 2,   1,   3, NOMINAL),
	F_MND16(147456000, pll3, 2,   2,   5, NOMINAL),
	F_MND16(153600000, pll1, 2,   2,   5, NOMINAL),
	F_MND16(192000000, pll1, 4,   0,   0, HIGH),
	F_END,
};

static struct clk_local jpeg_clk = {
	.b = {
		.en_reg = JPEG_NS_REG,
		.en_mask = BIT(9),
		.halt_reg = CLK_HALT_STATEB_REG,
		.halt_check = HALT,
		.halt_bit = 1,
		.test_vector = 0x6000,
		.reset_mask = P_JPEG_CLK,
	},
	.ns_reg = JPEG_NS_REG,
	.md_reg = JPEG_NS_REG - 4,
	.root_en_mask = BIT(11),
	.freq_tbl = clk_tbl_vfe_jpeg,
	.ns_mask = F_MASK_MND16,
	.set_rate = set_rate_mnd,
	.depends = &axi_li_jpeg_clk.c,
	.current_freq = &local_dummy_freq,
	.c = {
		.dbg_name = "jpeg_clk",
		.flags = CLKFLAG_AUTO_OFF,
		.ops = &soc_clk_ops_7x30,
		CLK_INIT(jpeg_clk.c),
	},
};

static struct clk_local vfe_clk = {
	.b = {
		.en_reg = CAM_VFE_NS_REG,
		.en_mask = BIT(9),
		.halt_reg = CLK_HALT_STATEB_REG,
		.halt_check = HALT,
		.halt_bit = 0,
		.test_vector = 0x4D76,
		.reset_mask = P_VFE_CLK,
	},
	.ns_reg = CAM_VFE_NS_REG,
	.md_reg = CAM_VFE_NS_REG - 4,
	.root_en_mask = BIT(13),
	.freq_tbl = clk_tbl_vfe_jpeg,
	.ns_mask = F_MASK_MND16,
	.set_rate = set_rate_mnd,
	.depends = &axi_li_vfe_clk.c,
	.current_freq = &local_dummy_freq,
	.c = {
		.dbg_name = "vfe_clk",
		.ops = &soc_clk_ops_7x30,
		CLK_INIT(vfe_clk.c),
	},
};

static struct branch_clk vfe_mdc_clk = {
	.b = {
		.en_reg = CAM_VFE_NS_REG,
		.en_mask = BIT(11),
		.halt_reg = CLK_HALT_STATEA_REG,
		.halt_check = HALT,
		.halt_bit = 9,
		.test_vector = 0x4D57,
		.reset_mask = P_VFE_MDC_CLK,
	},
	.parent = &vfe_clk.c,
	.c = {
		.dbg_name = "vfe_mdc_clk",
		.ops = &clk_ops_branch,
		CLK_INIT(vfe_mdc_clk.c),
	},
};

static struct branch_clk vfe_camif_clk = {
	.b = {
		.en_reg = CAM_VFE_NS_REG,
		.en_mask = BIT(15),
		.halt_reg = CLK_HALT_STATEC_REG,
		.halt_check = HALT,
		.halt_bit = 13,
		.test_vector = 0x7000,
		.reset_mask = P_VFE_CAMIF_CLK,
	},
	.parent = &vfe_clk.c,
	.c = {
		.dbg_name = "vfe_camif_clk",
		.ops = &clk_ops_branch,
		CLK_INIT(vfe_camif_clk.c),
	},
};

static struct branch_clk csi0_vfe_clk = {
	.b = {
		.en_reg = CSI_NS_REG,
		.en_mask = BIT(15),
		.halt_reg = CLK_HALT_STATEC_REG,
		.halt_check = HALT,
		.halt_bit = 16,
		.test_vector = 0x7200,
		.reset_mask = P_CSI0_VFE_CLK,
	},
	.parent = &vfe_clk.c,
	.c = {
		.dbg_name = "csi0_vfe_clk",
		.ops = &clk_ops_branch,
		CLK_INIT(csi0_vfe_clk.c),
	},
};

static struct clk_freq_tbl clk_tbl_cam[] = {
	F_MND16(       0, gnd,  1,   0,   0, NONE),
	F_MND16( 6000000, pll1, 4,   1,  32, NOMINAL),
	F_MND16( 8000000, pll1, 4,   1,  24, NOMINAL),
	F_MND16(12000000, pll1, 4,   1,  16, NOMINAL),
	F_MND16(16000000, pll1, 4,   1,  12, NOMINAL),
	F_MND16(19200000, pll1, 4,   1,  10, NOMINAL),
	F_MND16(24000000, pll1, 4,   1,   8, NOMINAL),
	F_MND16(32000000, pll1, 4,   1,   6, NOMINAL),
	F_MND16(48000000, pll1, 4,   1,   4, NOMINAL),
	F_MND16(64000000, pll1, 4,   1,   3, NOMINAL),
	F_END,
};

static struct clk_local cam_m_clk = {
	.b = {
		.en_reg = CAM_NS_REG,
		.halt_check = DELAY,
		.test_vector = 0x4D44,
		.reset_mask = P_CAM_M_CLK,
	},
	.ns_reg = CAM_NS_REG,
	.md_reg = CAM_NS_REG - 4,
	.root_en_mask = BIT(9),
	.freq_tbl = clk_tbl_cam,
	.ns_mask = F_MASK_MND16,
	.set_rate = set_rate_mnd,
	.current_freq = &local_dummy_freq,
	.c = {
		.dbg_name = "cam_m_clk",
		.ops = &soc_clk_ops_7x30,
		CLK_INIT(cam_m_clk.c),
	},
};

static struct clk_freq_tbl clk_tbl_vpe[] = {
	F_MND8( 24576000, 22, 15, lpxo, 1,   0,   0, NOMINAL),
	F_MND8( 30720000, 22, 15, pll3, 4,   1,   6, NOMINAL),
	F_MND8( 61440000, 22, 15, pll3, 4,   1,   3, NOMINAL),
	F_MND8( 81920000, 22, 15, pll3, 3,   1,   3, NOMINAL),
	F_MND8(122880000, 22, 15, pll3, 3,   1,   2, NOMINAL),
	F_MND8(147456000, 22, 15, pll3, 1,   1,   5, NOMINAL),
	F_MND8(153600000, 22, 15, pll1, 1,   1,   5, NOMINAL),
	F_END,
};

static struct clk_local vpe_clk = {
	.b = {
		.en_reg = VPE_NS_REG,
		.en_mask = BIT(9),
		.halt_reg = CLK_HALT_STATEC_REG,
		.halt_check = HALT,
		.halt_bit = 10,
		.test_vector = 0x6C00,
		.reset_mask = P_VPE_CLK,
	},
	.ns_reg = VPE_NS_REG,
	.md_reg = VPE_NS_REG - 4,
	.ns_mask = F_MASK_MND8(22, 15),
	.root_en_mask = BIT(11),
	.freq_tbl = clk_tbl_vpe,
	.current_freq = &local_dummy_freq,
	.depends = &axi_vpe_clk.c,
	.set_rate = set_rate_mnd,
	.c = {
		.dbg_name = "vpe_clk",
		.ops = &soc_clk_ops_7x30,
		CLK_INIT(vpe_clk.c),
	},
};


static struct clk_freq_tbl clk_tbl_mfc[] = {
	F_MND8( 24576000, 24, 17, lpxo, 1,   0,   0, NOMINAL),
	F_MND8( 30720000, 24, 17, pll3, 4,   1,   6, NOMINAL),
	F_MND8( 61440000, 24, 17, pll3, 4,   1,   3, NOMINAL),
	F_MND8( 81920000, 24, 17, pll3, 3,   1,   3, NOMINAL),
	F_MND8(122880000, 24, 17, pll3, 3,   1,   2, NOMINAL),
	F_MND8(147456000, 24, 17, pll3, 1,   1,   5, NOMINAL),
	F_MND8(153600000, 24, 17, pll1, 1,   1,   5, NOMINAL),
	F_MND8(170667000, 24, 17, pll1, 1,   2,   9, NOMINAL),
	F_END,
};

static struct clk_local mfc_clk = {
	.b = {
		.en_reg = MFC_NS_REG,
		.en_mask = BIT(9),
		.halt_reg = CLK_HALT_STATEC_REG,
		.halt_check = HALT,
		.halt_bit = 12,
		.test_vector = 0x38,
		.reset_mask = P_MFC_CLK,
	},
	.ns_reg = MFC_NS_REG,
	.md_reg = MFC_NS_REG - 4,
	.ns_mask = F_MASK_MND8(24, 17),
	.root_en_mask = BIT(11),
	.freq_tbl = clk_tbl_mfc,
	.current_freq = &local_dummy_freq,
	.depends = &axi_mfc_clk.c,
	.set_rate = set_rate_mnd,
	.c = {
		.dbg_name = "mfc_clk",
		.ops = &soc_clk_ops_7x30,
		CLK_INIT(mfc_clk.c),
	},
};

static struct branch_clk mfc_div2_clk = {
	.b = {
		.en_reg = MFC_NS_REG,
		.en_mask = BIT(15),
		.halt_reg = CLK_HALT_STATEC_REG,
		.halt_check = HALT,
		.halt_bit = 11,
		.test_vector = 0x1F,
		.reset_mask = P_MFC_DIV2_CLK,
	},
	.parent = &mfc_clk.c,
	.c = {
		.dbg_name = "mfc_div2_clk",
		.ops = &clk_ops_branch,
		CLK_INIT(mfc_div2_clk.c),
	},
};

static struct clk_freq_tbl clk_tbl_spi[] = {
	F_MND8(       0,  0,  0, gnd,  1,   0,     0, NONE),
	F_MND8( 9963243, 19, 12, pll3, 4,   2,    37, NOMINAL),
	F_MND8(26331429, 19, 12, pll3, 4,   1,     7, NOMINAL),
	F_END,
};

static struct clk_local spi_clk = {
	.b = {
		.en_reg = SPI_NS_REG,
		.en_mask = BIT(9),
		.halt_reg = CLK_HALT_STATEC_REG,
		.halt_check = HALT,
		.halt_bit = 0,
		.test_vector = 0x09,
		.reset_mask = P_SPI_CLK,
	},
	.ns_reg = SPI_NS_REG,
	.md_reg = SPI_NS_REG - 4,
	.ns_mask = F_MASK_MND8(19, 12),
	.root_en_mask = BIT(11),
	.freq_tbl = clk_tbl_spi,
	.current_freq = &local_dummy_freq,
	.set_rate = set_rate_mnd,
	.c = {
		.dbg_name = "spi_clk",
		.ops = &soc_clk_ops_7x30,
		CLK_INIT(spi_clk.c),
	},
};

static struct clk_freq_tbl clk_tbl_lpa_codec[] = {
	F_RAW(1, NULL, 0, 0, 0, 0, LOW, NULL), /* src MI2S_CODEC_RX */
	F_RAW(2, NULL, 0, 1, 0, 0, LOW, NULL), /* src ECODEC_CIF */
	F_RAW(3, NULL, 0, 2, 0, 0, LOW, NULL), /* src MI2S */
	F_RAW(4, NULL, 0, 3, 0, 0, LOW, NULL), /* src SDAC */
	F_END,
};

static struct clk_local lpa_codec_clk = {
	.b = {
		.en_reg = LPA_NS_REG,
		.en_mask = BIT(9),
		.halt_reg = CLK_HALT_STATEC_REG,
		.halt_check = HALT,
		.halt_bit = 6,
		.test_vector = 0x0F,
		.reset_mask = P_LPA_CODEC_CLK,
	},
	.ns_reg = LPA_NS_REG,
	.ns_mask = BM(1, 0),
	.set_rate = set_rate_nop,
	.freq_tbl = clk_tbl_lpa_codec,
	.current_freq = &local_dummy_freq,
	.c = {
		.dbg_name = "lpa_codec_clk",
		.ops = &soc_clk_ops_7x30,
		CLK_INIT(lpa_codec_clk.c),
	},
};

static struct clk_freq_tbl clk_tbl_mdc[] = {
	F_RAW(1, NULL, 0, 0, 0, 0, LOW, NULL),
	F_END
};

static struct clk_local mdc_clk = {
	.b = {
		.en_reg = MDC_NS_REG,
		.en_mask = BIT(9),
		.halt_reg = CLK_HALT_STATEA_REG,
		.halt_check = HALT,
		.halt_bit = 10,
		.test_vector = 0x4D56,
		.reset_mask = P_MDC_CLK,
	},
	.ns_reg = MDC_NS_REG,
	.root_en_mask = BIT(11),
	.freq_tbl = clk_tbl_mdc,
	.current_freq = &local_dummy_freq,
	.set_rate = set_rate_nop,
	.c = {
		.dbg_name = "mdc_clk",
		.ops = &soc_clk_ops_7x30,
		CLK_INIT(mdc_clk.c),
	},
};

static struct branch_clk lpa_core_clk = {
	.b = {
		.en_reg = LPA_NS_REG,
		.en_mask = BIT(5),
		.halt_reg = CLK_HALT_STATEC_REG,
		.halt_check = HALT,
		.halt_bit = 5,
		.test_vector = 0x0E,
		.reset_mask = P_LPA_CORE_CLK,
	},
	.c = {
		.dbg_name = "lpa_core_clk",
		.ops = &clk_ops_branch,
		CLK_INIT(lpa_core_clk.c),
	},
};

static DEFINE_CLK_PCOM(adsp_clk, ADSP_CLK, 0);
static DEFINE_CLK_PCOM(codec_ssbi_clk,	CODEC_SSBI_CLK, 0);
static DEFINE_CLK_PCOM(ebi1_clk, EBI1_CLK, CLK_MIN);
static DEFINE_CLK_PCOM(ecodec_clk, ECODEC_CLK, 0);
static DEFINE_CLK_PCOM(gp_clk, GP_CLK, 0);
static DEFINE_CLK_PCOM(uart3_clk, UART3_CLK, CLKFLAG_AUTO_OFF);
static DEFINE_CLK_PCOM(usb_phy_clk, USB_PHY_CLK, CLKFLAG_AUTO_OFF | CLK_MIN);
static DEFINE_CLK_PCOM(vdc_clk, VDC_CLK, 0);

static DEFINE_CLK_PCOM(p_grp_2d_clk, GRP_2D_CLK, 0);
static DEFINE_CLK_PCOM(p_grp_2d_p_clk, GRP_2D_P_CLK, 0);
static DEFINE_CLK_PCOM(p_hdmi_clk, HDMI_CLK, 0);
static DEFINE_CLK_PCOM(p_jpeg_clk, JPEG_CLK, CLKFLAG_AUTO_OFF);
static DEFINE_CLK_PCOM(p_jpeg_p_clk, JPEG_P_CLK, CLKFLAG_AUTO_OFF);
static DEFINE_CLK_PCOM(p_lpa_codec_clk, LPA_CODEC_CLK, 0);
static DEFINE_CLK_PCOM(p_lpa_core_clk, LPA_CORE_CLK, 0);
static DEFINE_CLK_PCOM(p_lpa_p_clk, LPA_P_CLK, 0);
static DEFINE_CLK_PCOM(p_mi2s_m_clk, MI2S_M_CLK, 0);
static DEFINE_CLK_PCOM(p_mi2s_s_clk, MI2S_S_CLK, 0);
static DEFINE_CLK_PCOM(p_mi2s_codec_rx_m_clk, MI2S_CODEC_RX_M_CLK, 0);
static DEFINE_CLK_PCOM(p_mi2s_codec_rx_s_clk, MI2S_CODEC_RX_S_CLK, 0);
static DEFINE_CLK_PCOM(p_mi2s_codec_tx_m_clk, MI2S_CODEC_TX_M_CLK, 0);
static DEFINE_CLK_PCOM(p_mi2s_codec_tx_s_clk, MI2S_CODEC_TX_S_CLK, 0);
static DEFINE_CLK_PCOM(p_sdac_clk, SDAC_CLK, CLKFLAG_AUTO_OFF);
static DEFINE_CLK_PCOM(p_sdac_m_clk, SDAC_M_CLK, CLKFLAG_AUTO_OFF);
static DEFINE_CLK_PCOM(p_vfe_clk, VFE_CLK, 0);
static DEFINE_CLK_PCOM(p_vfe_camif_clk, VFE_CAMIF_CLK, 0);
static DEFINE_CLK_PCOM(p_vfe_mdc_clk, VFE_MDC_CLK, 0);
static DEFINE_CLK_PCOM(p_vfe_p_clk, VFE_P_CLK, CLKFLAG_AUTO_OFF);
static DEFINE_CLK_PCOM(p_grp_3d_clk, GRP_3D_CLK, 0);
static DEFINE_CLK_PCOM(p_grp_3d_p_clk, GRP_3D_P_CLK, 0);
static DEFINE_CLK_PCOM(p_imem_clk, IMEM_CLK, CLKFLAG_AUTO_OFF);
static DEFINE_CLK_PCOM(p_mdp_lcdc_pad_pclk_clk, MDP_LCDC_PAD_PCLK_CLK, 0);
static DEFINE_CLK_PCOM(p_mdp_lcdc_pclk_clk, MDP_LCDC_PCLK_CLK, 0);
static DEFINE_CLK_PCOM(p_mdp_p_clk, MDP_P_CLK, 0);
static DEFINE_CLK_PCOM(p_mdp_vsync_clk, MDP_VSYNC_CLK, CLKFLAG_AUTO_OFF);
static DEFINE_CLK_PCOM(p_tsif_ref_clk, TSIF_REF_CLK, 0);
static DEFINE_CLK_PCOM(p_tsif_p_clk, TSIF_P_CLK, 0);
static DEFINE_CLK_PCOM(p_tv_dac_clk, TV_DAC_CLK, 0);
static DEFINE_CLK_PCOM(p_tv_enc_clk, TV_ENC_CLK, 0);
static DEFINE_CLK_PCOM(p_emdh_clk, EMDH_CLK,
		CLKFLAG_AUTO_OFF | CLKFLAG_MIN | CLKFLAG_MAX);
static DEFINE_CLK_PCOM(p_emdh_p_clk, EMDH_P_CLK, CLKFLAG_AUTO_OFF);
static DEFINE_CLK_PCOM(p_i2c_clk, I2C_CLK, 0);
static DEFINE_CLK_PCOM(p_i2c_2_clk, I2C_2_CLK, 0);
static DEFINE_CLK_PCOM(p_mdc_clk, MDC_CLK, 0);
static DEFINE_CLK_PCOM(p_pmdh_clk, PMDH_CLK,
		CLKFLAG_AUTO_OFF | CLKFLAG_MIN | CLKFLAG_MAX);
static DEFINE_CLK_PCOM(p_pmdh_p_clk, PMDH_P_CLK, 0);
static DEFINE_CLK_PCOM(p_sdc1_clk, SDC1_CLK, CLKFLAG_AUTO_OFF);
static DEFINE_CLK_PCOM(p_sdc1_p_clk, SDC1_P_CLK, CLKFLAG_AUTO_OFF);
static DEFINE_CLK_PCOM(p_sdc2_clk, SDC2_CLK, CLKFLAG_AUTO_OFF);
static DEFINE_CLK_PCOM(p_sdc2_p_clk, SDC2_P_CLK, CLKFLAG_AUTO_OFF);
static DEFINE_CLK_PCOM(p_sdc3_clk, SDC3_CLK, CLKFLAG_AUTO_OFF);
static DEFINE_CLK_PCOM(p_sdc3_p_clk, SDC3_P_CLK, CLKFLAG_AUTO_OFF);
static DEFINE_CLK_PCOM(p_sdc4_clk, SDC4_CLK, CLKFLAG_AUTO_OFF);
static DEFINE_CLK_PCOM(p_sdc4_p_clk, SDC4_P_CLK, CLKFLAG_AUTO_OFF);
static DEFINE_CLK_PCOM(p_uart2_clk, UART2_CLK, 0);
static DEFINE_CLK_PCOM(p_usb_hs2_clk, USB_HS2_CLK, CLKFLAG_AUTO_OFF);
static DEFINE_CLK_PCOM(p_usb_hs2_core_clk, USB_HS2_CORE_CLK, CLKFLAG_AUTO_OFF);
static DEFINE_CLK_PCOM(p_usb_hs2_p_clk, USB_HS2_P_CLK, CLKFLAG_AUTO_OFF);
static DEFINE_CLK_PCOM(p_usb_hs3_clk, USB_HS3_CLK, CLKFLAG_AUTO_OFF);
static DEFINE_CLK_PCOM(p_usb_hs3_core_clk, USB_HS3_CORE_CLK, CLKFLAG_AUTO_OFF);
static DEFINE_CLK_PCOM(p_usb_hs3_p_clk, USB_HS3_P_CLK, CLKFLAG_AUTO_OFF);
static DEFINE_CLK_PCOM(p_qup_i2c_clk, QUP_I2C_CLK, 0);
static DEFINE_CLK_PCOM(p_spi_clk, SPI_CLK, 0);
static DEFINE_CLK_PCOM(p_spi_p_clk, SPI_P_CLK, 0);
static DEFINE_CLK_PCOM(p_uart1_clk, UART1_CLK, CLKFLAG_AUTO_OFF);
static DEFINE_CLK_PCOM(p_uart1dm_clk, UART1DM_CLK, CLKFLAG_AUTO_OFF);
static DEFINE_CLK_PCOM(p_uart2dm_clk, UART2DM_CLK, 0);
static DEFINE_CLK_PCOM(p_usb_hs_clk, USB_HS_CLK, CLKFLAG_AUTO_OFF);
static DEFINE_CLK_PCOM(p_usb_hs_core_clk, USB_HS_CORE_CLK, CLKFLAG_AUTO_OFF);
static DEFINE_CLK_PCOM(p_usb_hs_p_clk, USB_HS_P_CLK, CLKFLAG_AUTO_OFF);
static DEFINE_CLK_PCOM(p_cam_m_clk, CAM_M_CLK, 0);
static DEFINE_CLK_PCOM(p_camif_pad_p_clk, CAMIF_PAD_P_CLK, CLKFLAG_AUTO_OFF);
static DEFINE_CLK_PCOM(p_csi0_clk, CSI0_CLK, 0);
static DEFINE_CLK_PCOM(p_csi0_vfe_clk, CSI0_VFE_CLK, 0);
static DEFINE_CLK_PCOM(p_csi0_p_clk, CSI0_P_CLK, 0);
static DEFINE_CLK_PCOM(p_mdp_clk, MDP_CLK, CLKFLAG_AUTO_OFF);
static DEFINE_CLK_PCOM(p_mfc_clk, MFC_CLK, 0);
static DEFINE_CLK_PCOM(p_mfc_div2_clk, MFC_DIV2_CLK, 0);
static DEFINE_CLK_PCOM(p_mfc_p_clk, MFC_P_CLK, 0);
static DEFINE_CLK_PCOM(p_vpe_clk, VPE_CLK, 0);
static DEFINE_CLK_PCOM(p_adm_clk, ADM_CLK, 0);
static DEFINE_CLK_PCOM(p_ce_clk, CE_CLK, 0);
static DEFINE_CLK_PCOM(p_axi_rotator_clk, AXI_ROTATOR_CLK, 0);
static DEFINE_CLK_PCOM(p_rotator_imem_clk, ROTATOR_IMEM_CLK, CLKFLAG_AUTO_OFF);
static DEFINE_CLK_PCOM(p_rotator_p_clk, ROTATOR_P_CLK, CLKFLAG_AUTO_OFF);

static struct pcom_clk pbus_clk = {
	.id = P_PBUS_CLK,
	.c = {
		.ops = &clk_ops_pcom_div2,
		.flags = CLKFLAG_MIN,
		.dbg_name = "pbus_clk",
		CLK_INIT(pbus_clk.c),
	},
};

static DEFINE_CLK_VOTER(ebi_dtv_clk, &pbus_clk.c);
static DEFINE_CLK_VOTER(ebi_kgsl_clk, &pbus_clk.c);
static DEFINE_CLK_VOTER(ebi_lcdc_clk, &pbus_clk.c);
static DEFINE_CLK_VOTER(ebi_mddi_clk, &pbus_clk.c);
static DEFINE_CLK_VOTER(ebi_tv_clk, &pbus_clk.c);
static DEFINE_CLK_VOTER(ebi_vcd_clk, &pbus_clk.c);
static DEFINE_CLK_VOTER(ebi_vfe_clk, &pbus_clk.c);

/*
 * SoC-specific functions required by clock-local driver
 */

/* Update the sys_vdd voltage given a level. */
int soc_update_sys_vdd(enum sys_vdd_level level)
{
	int rc, target_mv;
	static const int mv[NUM_SYS_VDD_LEVELS] = {
		[NONE...LOW] = 1000,
		[NOMINAL] = 1100,
		[HIGH]    = 1200,
	};

	target_mv = mv[level];
	rc = msm_proc_comm(PCOM_CLKCTL_RPC_MIN_MSMC1, &target_mv, NULL);
	if (rc)
		goto out;
	if (target_mv) {
		rc = -EINVAL;
		goto out;
	}
out:
	return rc;
}

/* Enable/disable a power rail associated with a clock. */
int soc_set_pwr_rail(struct clk *clk, int enable)
{
	int pwr_id = 0;

	if (clk == &axi_rotator_clk.c)
		pwr_id = PWR_RAIL_ROTATOR_CLK;
	else if (clk == &grp_2d_clk.c)
		pwr_id = PWR_RAIL_GRP_2D_CLK;
	else if (clk == &grp_3d_clk.c)
		pwr_id = PWR_RAIL_GRP_CLK;
	else if (clk == &mdp_clk.c)
		pwr_id = PWR_RAIL_MDP_CLK;
	else if (clk == &mfc_clk.c)
		pwr_id = PWR_RAIL_MFC_CLK;
	else if (clk == &vfe_clk.c)
		pwr_id = PWR_RAIL_VFE_CLK;
	else if (clk == &vpe_clk.c)
		pwr_id = PWR_RAIL_VPE_CLK;
	else
		return 0;

	return internal_pwr_rail_ctl_auto(pwr_id, enable);
}

/* Sample clock for 'tcxo4_ticks' reference clock ticks. */
static uint32_t run_measurement(unsigned tcxo4_ticks)
{
	/* TCXO4_CNT_EN and RINGOSC_CNT_EN register values. */
	uint32_t reg_val_enable = readl(MISC_CLK_CTL_BASE_REG) | 0x3;
	uint32_t reg_val_disable = reg_val_enable & ~0x3;

	/* Stop counters and set the TCXO4 counter start value. */
	writel(reg_val_disable, MISC_CLK_CTL_BASE_REG);
	writel(tcxo4_ticks, TCXO_CNT_BASE_REG);

	/* Run measurement and wait for completion. */
	writel(reg_val_enable, MISC_CLK_CTL_BASE_REG);
	while (readl(TCXO_CNT_DONE_BASE_REG) == 0)
		cpu_relax();

	/* Stop counters. */
	writel(reg_val_disable, MISC_CLK_CTL_BASE_REG);

	return readl(RINGOSC_CNT_BASE_REG);
}

/* Perform a hardware rate measurement for a given clock.
   FOR DEBUG USE ONLY: Measurements take ~15 ms! */
static signed __soc_clk_measure_rate(u32 test_vector)
{
	unsigned long flags;
	uint32_t regval, prph_web_reg_old;
	uint64_t raw_count_short, raw_count_full;
	signed ret;

	if (test_vector == 0)
		return -EPERM;

	spin_lock_irqsave(&local_clock_reg_lock, flags);

	/* Program test vector. */
	if (test_vector <= 0xFF) {
		/* Select CLK_TEST_2 */
		writel(0x4D40, CLK_TEST_BASE_REG);
		writel(test_vector, CLK_TEST_2_BASE_REG);
	} else
		writel(test_vector, CLK_TEST_BASE_REG);

	/* Enable TCXO4 clock branch and root. */
	prph_web_reg_old = readl(PRPH_WEB_NS_BASE_REG);
	regval = prph_web_reg_old | BIT(9) | BIT(11);
	clk_enable(&tcxo_clk.c);
	writel(regval, PRPH_WEB_NS_BASE_REG);

	/*
	 * The ring oscillator counter will not reset if the measured clock
	 * is not running.  To detect this, run a short measurement before
	 * the full measurement.  If the raw results of the two are the same
	 * then the clock must be off.
	 */

	/* Run a short measurement. (~1 ms) */
	raw_count_short = run_measurement(0x1000);
	/* Run a full measurement. (~14 ms) */
	raw_count_full = run_measurement(0x10000);

	/* Disable TCXO4 clock branch and root. */
	writel(prph_web_reg_old, PRPH_WEB_NS_BASE_REG);
	clk_disable(&tcxo_clk.c);

	/* Return 0 if the clock is off. */
	if (raw_count_full == raw_count_short)
		ret = 0;
	else {
		/* Compute rate in Hz. */
		raw_count_full = ((raw_count_full * 10) + 15) * 4800000;
		do_div(raw_count_full, ((0x10000 * 10) + 35));
		ret = (signed)raw_count_full;
	}

	spin_unlock_irqrestore(&local_clock_reg_lock, flags);

	return ret;
}

static signed branch_clk_measure_rate(struct clk *clk)
{
	return __soc_clk_measure_rate(to_branch_clk(clk)->b.test_vector);
}

static signed soc_clk_measure_rate(struct clk *clk)
{
	return __soc_clk_measure_rate(to_local(clk)->b.test_vector);
}

/* Implementation for clk_set_flags(). */
int soc_clk_set_flags(struct clk *clk, unsigned clk_flags)
{
	uint32_t regval, ret = 0;
	unsigned long flags;

	spin_lock_irqsave(&local_clock_reg_lock, flags);

	if (clk == &vfe_clk.c) {
		regval = readl(CAM_VFE_NS_REG);
		/* Flag values chosen for backward compatibility
		 * with proc_comm remote clock control. */
		if (clk_flags == 0x00000100) {
			/* Select external source. */
			regval |= BIT(14);
		} else if (clk_flags == 0x00000200) {
			/* Select internal source. */
			regval &= ~BIT(14);
		} else
			ret = -EINVAL;

		writel(regval, CAM_VFE_NS_REG);
	} else
		ret = -EPERM;

	spin_unlock_irqrestore(&local_clock_reg_lock, flags);

	return ret;
}

static int msm7x30_clk_reset(struct clk *clk, enum clk_reset_action action)
{
	/* reset_mask is actually a proc_comm id */
	unsigned id = to_local(clk)->b.reset_mask;
	return pc_clk_reset(id, action);
}

static int soc_branch_clk_reset(struct clk *clk, enum clk_reset_action action)
{
	unsigned id = to_branch_clk(clk)->b.reset_mask;
	return pc_clk_reset(id, action);
}

/*
 * Clock ownership detection code
 */

enum {
	SH2_OWN_GLBL,
	SH2_OWN_APPS1,
	SH2_OWN_APPS2,
	SH2_OWN_ROW1,
	SH2_OWN_ROW2,
	SH2_OWN_APPS3,
	NUM_OWNERSHIP
};
static __initdata uint32_t ownership_regs[NUM_OWNERSHIP];

static void __init cache_ownership(void)
{
	ownership_regs[SH2_OWN_GLBL]  = readl(SH2_OWN_GLBL_BASE_REG);
	ownership_regs[SH2_OWN_APPS1] = readl(SH2_OWN_APPS1_BASE_REG);
	ownership_regs[SH2_OWN_APPS2] = readl(SH2_OWN_APPS2_BASE_REG);
	ownership_regs[SH2_OWN_ROW1]  = readl(SH2_OWN_ROW1_BASE_REG);
	ownership_regs[SH2_OWN_ROW2]  = readl(SH2_OWN_ROW2_BASE_REG);
	ownership_regs[SH2_OWN_APPS3] = readl(SH2_OWN_APPS3_BASE_REG);
}

static void __init print_ownership(void)
{
	pr_info("Clock ownership\n");
	pr_info("  GLBL  : %08x\n", ownership_regs[SH2_OWN_GLBL]);
	pr_info("  APPS  : %08x %08x %08x\n", ownership_regs[SH2_OWN_APPS1],
		ownership_regs[SH2_OWN_APPS2], ownership_regs[SH2_OWN_APPS3]);
	pr_info("  ROW   : %08x %08x\n", ownership_regs[SH2_OWN_ROW1],
		ownership_regs[SH2_OWN_ROW2]);
}

#define O(x) (&ownership_regs[(SH2_OWN_##x)])
#define OWN(r, b, name, clk, dev) \
	{ \
		.lk = CLK_LOOKUP(name, clk.c, dev), \
		.remote = &p_##clk.c, \
		.reg = O(r), \
		.bit = BIT(b), \
	}

static struct clk_local_ownership {
	struct clk_lookup lk;
	const u32 *reg;
	const u32 bit;
	struct clk *remote;
} ownership_map[] __initdata = {
	/* Sources */
	{ CLK_LOOKUP("pll1_clk",	pll1_clk.c,	"acpu") },
	{ CLK_LOOKUP("pll2_clk",	pll2_clk.c,	"acpu") },
	{ CLK_LOOKUP("pll3_clk",	pll3_clk.c,	"acpu") },

	/* PCOM */
	{ CLK_LOOKUP("adsp_clk",	adsp_clk.c,	NULL) },
	{ CLK_LOOKUP("codec_ssbi_clk",	codec_ssbi_clk.c,	NULL) },
	{ CLK_LOOKUP("ebi1_clk",	ebi1_clk.c,	NULL) },
	{ CLK_LOOKUP("ecodec_clk",	ecodec_clk.c,	NULL) },
	{ CLK_LOOKUP("gp_clk",		gp_clk.c,	NULL) },
	{ CLK_LOOKUP("uart_clk",	uart3_clk.c,	"msm_serial.2") },
	{ CLK_LOOKUP("usb_phy_clk",	usb_phy_clk.c,	NULL) },
	{ CLK_LOOKUP("vdc_clk",		vdc_clk.c,	NULL) },
	{ CLK_LOOKUP("pbus_clk",	pbus_clk.c,	NULL) },

	/* Voters */
	{ CLK_LOOKUP("ebi1_dtv_clk",	ebi_dtv_clk.c,	NULL) },
	{ CLK_LOOKUP("ebi1_kgsl_clk",	ebi_kgsl_clk.c,	NULL) },
	{ CLK_LOOKUP("ebi1_lcdc_clk",	ebi_lcdc_clk.c,	NULL) },
	{ CLK_LOOKUP("ebi1_mddi_clk",	ebi_mddi_clk.c,	NULL) },
	{ CLK_LOOKUP("ebi1_tv_clk",	ebi_tv_clk.c,	NULL) },
	{ CLK_LOOKUP("ebi1_vcd_clk",	ebi_vcd_clk.c,	NULL) },
	{ CLK_LOOKUP("ebi1_vfe_clk",	ebi_vfe_clk.c,	NULL) },

	/*
	 * This is a many-to-one mapping because we don't know how the remote
	 * clock code has decided to handle the dependencies between clocks for
	 * a particular hardware block. We determine the ownership for all the
	 * clocks going into a block by checking the ownership bit of one
	 * register (usually the ns register).
	 */
	OWN(APPS1,  6, "grp_2d_clk",	grp_2d_clk,	NULL),
	OWN(APPS1,  6, "grp_2d_pclk",	grp_2d_p_clk,	NULL),
	OWN(APPS1, 31, "hdmi_clk",	hdmi_clk,	NULL),
	OWN(APPS1,  0, "jpeg_clk",	jpeg_clk,	NULL),
	OWN(APPS1,  0, "jpeg_pclk",	jpeg_p_clk,	NULL),
	OWN(APPS1, 23, "lpa_codec_clk", lpa_codec_clk,	NULL),
	OWN(APPS1, 23, "lpa_core_clk",	lpa_core_clk,	NULL),
	OWN(APPS1, 23, "lpa_pclk",	lpa_p_clk,	NULL),
	OWN(APPS1, 28, "mi2s_m_clk",	mi2s_m_clk,	NULL),
	OWN(APPS1, 28, "mi2s_s_clk",	mi2s_s_clk,	NULL),
	OWN(APPS1, 12, "mi2s_codec_rx_m_clk", mi2s_codec_rx_m_clk, NULL),
	OWN(APPS1, 12, "mi2s_codec_rx_s_clk", mi2s_codec_rx_s_clk, NULL),
	OWN(APPS1, 14, "mi2s_codec_tx_m_clk", mi2s_codec_tx_m_clk, NULL),
	OWN(APPS1, 14, "mi2s_codec_tx_s_clk", mi2s_codec_tx_s_clk, NULL),
	{ CLK_LOOKUP("midi_clk",        midi_clk.c,     NULL),
		O(APPS1), BIT(22) },
	OWN(APPS1, 26, "sdac_clk",	sdac_clk,	NULL),
	OWN(APPS1, 26, "sdac_m_clk",	sdac_m_clk,	NULL),
	OWN(APPS1,  8, "vfe_clk",	vfe_clk,	NULL),
	OWN(APPS1,  8, "vfe_camif_clk", vfe_camif_clk,	NULL),
	OWN(APPS1,  8, "vfe_mdc_clk",	vfe_mdc_clk,	NULL),
	OWN(APPS1,  8, "vfe_pclk",	vfe_p_clk,	NULL),

	OWN(APPS2,  0, "grp_clk",	grp_3d_clk,	NULL),
	OWN(APPS2,  0, "grp_pclk",	grp_3d_p_clk,	NULL),
	{ CLK_LOOKUP("grp_src_clk",     grp_3d_src_clk.c, NULL),
		O(APPS2), BIT(0), &p_grp_3d_clk.c },
	OWN(APPS2,  0, "imem_clk",	imem_clk,	NULL),
	OWN(APPS2,  4, "mdp_lcdc_pad_pclk_clk", mdp_lcdc_pad_pclk_clk, NULL),
	OWN(APPS2,  4, "mdp_lcdc_pclk_clk", mdp_lcdc_pclk_clk, NULL),
	OWN(APPS2,  4, "mdp_pclk",	mdp_p_clk,	NULL),
	OWN(APPS2, 28, "mdp_vsync_clk", mdp_vsync_clk,	NULL),
	OWN(APPS2,  5, "tsif_ref_clk",	tsif_ref_clk,	NULL),
	OWN(APPS2,  5, "tsif_pclk",	tsif_p_clk,	NULL),
	{ CLK_LOOKUP("tv_src_clk",      tv_clk.c,       NULL),
		O(APPS2), BIT(2), &p_tv_enc_clk.c },
	OWN(APPS2,  2, "tv_dac_clk",	tv_dac_clk,	NULL),
	OWN(APPS2,  2, "tv_enc_clk",	tv_enc_clk,	NULL),

	OWN(ROW1,  7, "emdh_clk",	emdh_clk,	"msm_mddi.1"),
	OWN(ROW1,  7, "emdh_pclk",	emdh_p_clk,	"msm_mddi.1"),
	OWN(ROW1, 11, "i2c_clk",	i2c_clk,	"msm_i2c.0"),
	OWN(ROW1, 12, "i2c_clk",	i2c_2_clk,	"msm_i2c.2"),
	OWN(ROW1, 17, "mdc_clk",	mdc_clk,	NULL),
	OWN(ROW1, 19, "mddi_clk",	pmdh_clk,	NULL),
	OWN(ROW1, 19, "mddi_pclk",	pmdh_p_clk,	NULL),
	OWN(ROW1, 23, "sdc_clk",	sdc1_clk,	"msm_sdcc.1"),
	OWN(ROW1, 23, "sdc_pclk",	sdc1_p_clk,	"msm_sdcc.1"),
	OWN(ROW1, 25, "sdc_clk",	sdc2_clk,	"msm_sdcc.2"),
	OWN(ROW1, 25, "sdc_pclk",	sdc2_p_clk,	"msm_sdcc.2"),
	OWN(ROW1, 27, "sdc_clk",	sdc3_clk,	"msm_sdcc.3"),
	OWN(ROW1, 27, "sdc_pclk",	sdc3_p_clk,	"msm_sdcc.3"),
	OWN(ROW1, 29, "sdc_clk",	sdc4_clk,	"msm_sdcc.4"),
	OWN(ROW1, 29, "sdc_pclk",	sdc4_p_clk,	"msm_sdcc.4"),
	OWN(ROW1,  0, "uart_clk",	uart2_clk,	"msm_serial.1"),
	OWN(ROW1,  2, "usb_hs2_clk",	usb_hs2_clk,	NULL),
	OWN(ROW1,  2, "usb_hs2_core_clk", usb_hs2_core_clk, NULL),
	OWN(ROW1,  2, "usb_hs2_pclk",	usb_hs2_p_clk,	NULL),
	OWN(ROW1,  4, "usb_hs3_clk",	usb_hs3_clk,	NULL),
	OWN(ROW1,  4, "usb_hs3_core_clk", usb_hs3_core_clk, NULL),
	OWN(ROW1,  4, "usb_hs3_pclk",	usb_hs3_p_clk,	NULL),

	OWN(ROW2,  3, "qup_clk",	qup_i2c_clk,	"qup_i2c.4"),
	OWN(ROW2,  1, "spi_clk",	spi_clk,	NULL),
	OWN(ROW2,  1, "spi_pclk",	spi_p_clk,	NULL),
	OWN(ROW2,  9, "uart_clk",	uart1_clk,	"msm_serial.0"),
	OWN(ROW2,  6, "uartdm_clk",	uart1dm_clk,	"msm_serial_hs.0"),
	OWN(ROW2,  8, "uartdm_clk",	uart2dm_clk,	"msm_serial_hs.1"),
	OWN(ROW2, 11, "usb_hs_clk",	usb_hs_clk,	NULL),
	OWN(ROW2, 11, "usb_hs_core_clk", usb_hs_core_clk, NULL),
	OWN(ROW2, 11, "usb_hs_pclk",	usb_hs_p_clk,	NULL),

	OWN(APPS3,  6, "cam_m_clk",	cam_m_clk,	NULL),
	OWN(APPS3,  6, "camif_pad_pclk", camif_pad_p_clk, NULL),
	OWN(APPS3, 11, "csi_clk",	csi0_clk,	NULL),
	OWN(APPS3, 11, "csi_vfe_clk",	csi0_vfe_clk,	NULL),
	OWN(APPS3, 11, "csi_pclk",	csi0_p_clk,	NULL),
	OWN(APPS3,  0, "mdp_clk",	mdp_clk,	NULL),
	OWN(APPS3,  2, "mfc_clk",	mfc_clk,	NULL),
	OWN(APPS3,  2, "mfc_div2_clk",	mfc_div2_clk,	NULL),
	OWN(APPS3,  2, "mfc_pclk",	mfc_p_clk,	NULL),
	OWN(APPS3,  4, "vpe_clk",	vpe_clk,	NULL),

	OWN(GLBL,  8, "adm_clk",	adm_clk,	NULL),
	{ CLK_LOOKUP("adm_pclk", adm_p_clk.c,	NULL),
		O(GLBL), BIT(13), &dummy_clk },
	OWN(GLBL,  8, "ce_clk",	ce_clk,		NULL),
	OWN(GLBL, 13, "rotator_clk",	axi_rotator_clk, NULL),
	OWN(GLBL, 13, "rotator_imem_clk", rotator_imem_clk, NULL),
	OWN(GLBL, 13, "rotator_pclk",	rotator_p_clk,	NULL),
	{ CLK_LOOKUP("uartdm_pclk",     uart1dm_p_clk.c, "msm_serial_hs.0"),
		O(GLBL), BIT(8), &dummy_clk },
	{ CLK_LOOKUP("uartdm_pclk",     uart2dm_p_clk.c, "msm_serial_hs.1"),
		O(GLBL), BIT(8), &dummy_clk },
};

struct clk_lookup msm_clocks_7x30[ARRAY_SIZE(ownership_map)];
unsigned msm_num_clocks_7x30 = ARRAY_SIZE(msm_clocks_7x30);

static void set_clock_ownership(void)
{
	unsigned i;
	struct clk_lookup *lk;

	for (i = 0; i < ARRAY_SIZE(ownership_map); i++) {
		const u32 *reg = ownership_map[i].reg;
		u32 bit = ownership_map[i].bit;
		struct clk *remote = ownership_map[i].remote;

		lk = &ownership_map[i].lk;
		memcpy(&msm_clocks_7x30[i], lk, sizeof(*lk));

		if (reg && !(*reg & bit))
			msm_clocks_7x30[i].clk = remote;
	}
}

/*
 * Miscellaneous clock register initializations
 */
static const struct reg_init {
	const void __iomem *reg;
	uint32_t mask;
	uint32_t val;
} ri_list[] __initconst = {
	/* Enable UMDX_P clock. Known to causes issues, so never turn off. */
	{GLBL_CLK_ENA_2_SC_REG, BIT(2), BIT(2)},

	/* Disable all the child clocks of USB_HS_SRC. */
	{ USBH_NS_REG, BIT(13) | BIT(9), 0 },
	{ USBH2_NS_REG, BIT(9) | BIT(4), 0 },
	{ USBH3_NS_REG, BIT(9) | BIT(4), 0 },

	{EMDH_NS_REG, BM(18, 17) , BVAL(18, 17, 0x3)}, /* RX div = div-4. */
	{PMDH_NS_REG, BM(18, 17), BVAL(18, 17, 0x3)}, /* RX div = div-4. */
	/* MI2S_CODEC_RX_S src = MI2S_CODEC_RX_M. */
	{MI2S_RX_NS_REG, BIT(14), 0x0},
	/* MI2S_CODEC_TX_S src = MI2S_CODEC_TX_M. */
	{MI2S_TX_NS_REG, BIT(14), 0x0},
	{MI2S_NS_REG, BIT(14), 0x0}, /* MI2S_S src = MI2S_M. */
	/* Allow DSP to decide the LPA CORE src. */
	{LPA_CORE_CLK_MA0_REG, BIT(0), BIT(0)},
	{LPA_CORE_CLK_MA2_REG, BIT(0), BIT(0)},
	{MI2S_CODEC_RX_DIV_REG, 0xF, 0xD}, /* MI2S_CODEC_RX_S div = div-8. */
	{MI2S_CODEC_TX_DIV_REG, 0xF, 0xD}, /* MI2S_CODEC_TX_S div = div-8. */
	{MI2S_DIV_REG, 0xF, 0x7}, /* MI2S_S div = div-8. */
	{MDC_NS_REG, 0x3, 0x3}, /* MDC src = external MDH src. */
	{SDAC_NS_REG, BM(15, 14), 0x0}, /* SDAC div = div-1. */
	/* Disable sources TCXO/5 & TCXO/6. UART1 src = TCXO*/
	{UART_NS_REG, BM(26, 25) | BM(2, 0), 0x0},
	/* HDMI div = div-1, non-inverted. tv_enc_src = tv_clk_src */
	{HDMI_NS_REG, 0x7, 0x0},
	{TV_NS_REG, BM(15, 14), 0x0}, /* tv_clk_src_div2 = div-1 */

	/* USBH core clocks src = USB_HS_SRC. */
	{USBH_NS_REG, BIT(15), BIT(15)},
	{USBH2_NS_REG, BIT(6), BIT(6)},
	{USBH3_NS_REG, BIT(6), BIT(6)},
};

/* Local clock driver initialization. */
void __init msm_clk_soc_init(void)
{
	int i;
	uint32_t val;

	cache_ownership();
	print_ownership();
	set_clock_ownership();

	/* When we have no local clock control, the rest of the code in this
	 * function is a NOP since writes to shadow regions that we don't own
	 * are ignored. */

	clk_set_rate(&usb_hs_src_clk.c, clk_tbl_usb[1].freq_hz);

	for (i = 0; i < ARRAY_SIZE(ri_list); i++) {
		val = readl(ri_list[i].reg);
		val &= ~ri_list[i].mask;
		val |= ri_list[i].val;
		writel(val, ri_list[i].reg);
	}

	clk_set_rate(&i2c_clk.c, 19200000);
	clk_set_rate(&i2c_2_clk.c, 19200000);
	clk_set_rate(&qup_i2c_clk.c, 19200000);
	clk_set_rate(&uart1_clk.c, 19200000);
	clk_set_rate(&uart2_clk.c, 19200000);
	clk_set_rate(&mi2s_m_clk.c, 12288000);
	clk_set_rate(&midi_clk.c, 98304000);
	clk_set_rate(&mdp_vsync_clk.c, 24576000);
	clk_set_rate(&glbl_root_clk.c, 1);
	clk_set_rate(&mdc_clk.c, 1);
	/* Sync the LPA_CODEC clock to MI2S_CODEC_RX */
	clk_set_rate(&lpa_codec_clk.c, 1);
	/* Sync the GRP2D clock to AXI */
	clk_set_rate(&grp_2d_clk.c, 1);
}

/*
 * Clock operation handler registration
 */
static struct clk_ops soc_clk_ops_7x30 = {
	.enable = local_clk_enable,
	.disable = local_clk_disable,
	.auto_off = local_clk_auto_off,
	.set_rate = local_clk_set_rate,
	.set_min_rate = local_clk_set_min_rate,
	.set_max_rate = local_clk_set_max_rate,
	.get_rate = local_clk_get_rate,
	.list_rate = local_clk_list_rate,
	.is_enabled = local_clk_is_enabled,
	.round_rate = local_clk_round_rate,
	.reset = msm7x30_clk_reset,
	.set_flags = soc_clk_set_flags,
	.measure_rate = soc_clk_measure_rate,
	.is_local = local_clk_is_local,
	.get_parent = local_clk_get_parent,
};

static struct clk_ops clk_ops_branch = {
	.enable = branch_clk_enable,
	.disable = branch_clk_disable,
	.auto_off = branch_clk_auto_off,
	.is_enabled = branch_clk_is_enabled,
	.reset = soc_branch_clk_reset,
	.set_flags = soc_clk_set_flags,
	.is_local = local_clk_is_local,
	.measure_rate = branch_clk_measure_rate,
	.get_parent = branch_clk_get_parent,
	.set_parent = branch_clk_set_parent,
};
