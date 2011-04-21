/* Copyright (c) 2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/elf.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <mach/msm_iomap.h>

#include "peripheral-loader.h"

#define MSM_FW_QDSP6SS_PHYS	0x08800000
#define MSM_SW_QDSP6SS_PHYS	0x08900000
#define MSM_LPASS_QDSP6SS_PHYS	0x28800000
#define MSM_MSS_ENABLE_PHYS	0x08B00000

#define QDSP6SS_RST_EVB		0x0
#define QDSP6SS_RESET		0x04
#define QDSP6SS_STRAP_TCM	0x1C
#define QDSP6SS_STRAP_AHB	0x20
#define QDSP6SS_GFMUX_CTL	0x30
#define QDSP6SS_PWR_CTL		0x38

#define MMS_MODEM_RESET		0x2C48
#define MMS_RESET		0x2C64

#define MODEM_Q6_STRAP_TCM_BASE		(0x40 << 16)
#define MODEM_Q6_STRAP_AHB_UPPER	(0x9 << 16)
#define MODEM_Q6_STRAP_AHB_LOWER	(0x8 << 4)

#define LPASS_Q6_STRAP_TCM_BASE		(0x146 << 4)
#define LPASS_Q6_STRAP_AHB_UPPER	(0x29 << 16)
#define LPASS_Q6_STRAP_AHB_LOWER	(0x28 << 4)

#define Q6SS_SS_ARES		BIT(0)
#define Q6SS_CORE_ARES		BIT(1)
#define Q6SS_ISDB_ARES		BIT(2)
#define Q6SS_ETM_ARES		BIT(3)
#define Q6SS_STOP_CORE_ARES	BIT(4)
#define Q6SS_PRIV_ARES		BIT(5)

#define Q6SS_L2DATA_SLP_NRET_N	BIT(0)
#define Q6SS_SLP_RET_N		BIT(1)
#define Q6SS_L1TCM_SLP_NRET_N	BIT(2)
#define Q6SS_L2TAG_SLP_NRET_N	BIT(3)
#define Q6SS_ETB_SLEEP_NRET_N	BIT(4)
#define Q6SS_ARR_STBY_N		BIT(5)
#define Q6SS_CLAMP_IO		BIT(6)

#define Q6SS_CLK_ENA		BIT(1)
#define Q6SS_SRC_SWITCH_CLK_OVR	BIT(8)

enum q6 {
	Q6_MODEM_FW,
	Q6_MODEM_SW,
	Q6_LPASS,
	NUM_Q6
};

static int q6_start[NUM_Q6];
static void __iomem *q6_reg_base[NUM_Q6];
static void __iomem *mss_enable_reg;

static int init_image_modem_fw_q6_untrusted(const u8 *metadata, size_t size)
{
	const struct elf32_hdr *ehdr = (struct elf32_hdr *)metadata;
	q6_start[Q6_MODEM_FW] = ehdr->e_entry;
	return 0;
}

static int init_image_modem_sw_q6_untrusted(const u8 *metadata, size_t size)
{
	const struct elf32_hdr *ehdr = (struct elf32_hdr *)metadata;
	q6_start[Q6_MODEM_SW] = ehdr->e_entry;
	return 0;
}

static int init_image_lpass_q6_untrusted(const u8 *metadata, size_t size)
{
	const struct elf32_hdr *ehdr = (struct elf32_hdr *)metadata;
	q6_start[Q6_LPASS] = ehdr->e_entry;
	return 0;
}

static int verify_blob(u32 phy_addr, size_t size)
{
	return 0;
}

static int reset_q6_untrusted(enum q6 q6_id)
{
	static const unsigned q6_strap_tcm_base[] = {
		[Q6_MODEM_FW] = MODEM_Q6_STRAP_TCM_BASE,
		[Q6_MODEM_SW] = MODEM_Q6_STRAP_TCM_BASE,
		[Q6_LPASS] =    LPASS_Q6_STRAP_TCM_BASE,
	};
	static const unsigned q6_strap_ahb_upper[] = {
		[Q6_MODEM_FW] = MODEM_Q6_STRAP_AHB_UPPER,
		[Q6_MODEM_SW] = MODEM_Q6_STRAP_AHB_UPPER,
		[Q6_LPASS] =    LPASS_Q6_STRAP_AHB_UPPER,
	};
	static const unsigned q6_strap_ahb_lower[] = {
		[Q6_MODEM_FW] = MODEM_Q6_STRAP_AHB_LOWER,
		[Q6_MODEM_SW] = MODEM_Q6_STRAP_AHB_LOWER,
		[Q6_LPASS] =    LPASS_Q6_STRAP_AHB_LOWER,
	};
	void __iomem *reg_base = q6_reg_base[q6_id];
	u32 reg;

	switch (q6_id) {
	case Q6_MODEM_FW:
	case Q6_MODEM_SW:
		/* Make sure Modem Subsystem is enabled and not in reset. */
		writel(0x0, MSM_CLK_CTL_BASE + MMS_MODEM_RESET);
		writel(0x0, MSM_CLK_CTL_BASE + MMS_RESET);
		writel(0x7, mss_enable_reg);
		break;
	case Q6_LPASS:
	default:
		break;
	}

	/* Program boot address */
	writel((q6_start[q6_id] >> 8) & 0xFFFFFF, reg_base + QDSP6SS_RST_EVB);

	/* Program TCM and AHB address ranges */
	writel(q6_strap_tcm_base[q6_id], reg_base + QDSP6SS_STRAP_TCM);
	writel(q6_strap_ahb_upper[q6_id] | q6_strap_ahb_lower[q6_id],
	       reg_base + QDSP6SS_STRAP_AHB);

	/* Turn off Q6 core clock */
	reg = Q6SS_SRC_SWITCH_CLK_OVR;
	writel(reg, reg_base + QDSP6SS_GFMUX_CTL);

	/* Put Q6 into reset */
	reg = Q6SS_CORE_ARES | Q6SS_ISDB_ARES | Q6SS_ETM_ARES
	    | Q6SS_STOP_CORE_ARES | Q6SS_PRIV_ARES;
	writel(reg, reg_base + QDSP6SS_RESET);

	/* Wait 8 AHB cycles for Q6 to be fully reset (AHB = 1.5Mhz) */
	usleep_range(20, 30);

	/* Turn on Q6 memories */
	reg = Q6SS_L2DATA_SLP_NRET_N | Q6SS_SLP_RET_N | Q6SS_L1TCM_SLP_NRET_N
	    | Q6SS_L2TAG_SLP_NRET_N | Q6SS_ETB_SLEEP_NRET_N | Q6SS_ARR_STBY_N
	    | Q6SS_CLAMP_IO;
	writel(reg, reg_base + QDSP6SS_PWR_CTL);

	/* Turn on Q6 core clock */
	reg = Q6SS_CLK_ENA | Q6SS_SRC_SWITCH_CLK_OVR;
	writel(reg, reg_base + QDSP6SS_GFMUX_CTL);

	/* Remove Q6SS_CLAMP_IO */
	reg = readl(reg_base + QDSP6SS_PWR_CTL);
	reg &= ~Q6SS_CLAMP_IO;
	writel(reg, reg_base + QDSP6SS_PWR_CTL);

	/* Bring Q6 core out of reset and start execution. */
	writel(0x0, reg_base + QDSP6SS_RESET);

	return 0;
}

static int reset_modem_fw_q6_untrusted(void)
{
	return reset_q6_untrusted(Q6_MODEM_FW);
}

static int reset_modem_sw_q6_untrusted(void)
{
	return reset_q6_untrusted(Q6_MODEM_SW);
}

static int reset_lpass_q6_untrusted(void)
{
	return reset_q6_untrusted(Q6_LPASS);
}

static int shutdown_q6_untrusted(enum q6 q6_id)
{
	u32 reg;
	void *reg_base = q6_reg_base[q6_id];

	/* Turn off Q6 core clock */
	reg = Q6SS_SRC_SWITCH_CLK_OVR;
	writel(reg, reg_base + QDSP6SS_GFMUX_CTL);

	/* Put Q6SS into reset */
	reg = Q6SS_SS_ARES | Q6SS_CORE_ARES | Q6SS_ISDB_ARES | Q6SS_ETM_ARES
	    | Q6SS_STOP_CORE_ARES | Q6SS_PRIV_ARES;
	writel(reg, reg_base + QDSP6SS_RESET);

	/* Turn off Q6 memories */
	reg &= ~(Q6SS_L2DATA_SLP_NRET_N | Q6SS_SLP_RET_N | Q6SS_L1TCM_SLP_NRET_N
	    | Q6SS_L2TAG_SLP_NRET_N | Q6SS_ETB_SLEEP_NRET_N | Q6SS_ARR_STBY_N
	    | Q6SS_CLAMP_IO);
	writel(reg, reg_base + QDSP6SS_PWR_CTL);

	return 0;
}

static int shutdown_modem_fw_q6_untrusted(void)
{
	return shutdown_q6_untrusted(Q6_MODEM_FW);
}

static int shutdown_modem_sw_q6_untrusted(void)
{
	return shutdown_q6_untrusted(Q6_MODEM_SW);
}

static int shutdown_lpass_q6_untrusted(void)
{
	return shutdown_q6_untrusted(Q6_LPASS);
}

static struct pil_reset_ops pil_modem_fw_q6_ops = {
	.init_image = init_image_modem_fw_q6_untrusted,
	.verify_blob = verify_blob,
	.auth_and_reset = reset_modem_fw_q6_untrusted,
	.shutdown = shutdown_modem_fw_q6_untrusted,
};

static struct pil_reset_ops pil_modem_sw_q6_ops = {
	.init_image = init_image_modem_sw_q6_untrusted,
	.verify_blob = verify_blob,
	.auth_and_reset = reset_modem_sw_q6_untrusted,
	.shutdown = shutdown_modem_sw_q6_untrusted,
};

static struct pil_reset_ops pil_lpass_q6_ops = {
	.init_image = init_image_lpass_q6_untrusted,
	.verify_blob = verify_blob,
	.auth_and_reset = reset_lpass_q6_untrusted,
	.shutdown = shutdown_lpass_q6_untrusted,
};

static struct pil_device peripherals[] = {
	{
		.name = "q6",
		.pdev = {
			.name = "pil_lpass_q6",
			.id = -1,
		},
		.ops = &pil_lpass_q6_ops,
	},
	{
		.name = "modem_fw",
		.depends_on = "q6",
		.pdev = {
			.name = "pil_modem_fw_q6",
			.id = -1,
		},
		.ops = &pil_modem_fw_q6_ops,
	},
	{
		.name = "modem",
		.depends_on = "modem_fw",
		.pdev = {
			.name = "pil_modem_sw_q6",
			.id = -1,
		},
		.ops = &pil_modem_sw_q6_ops,
	},
};

static int __init msm_peripheral_reset_init(void)
{
	unsigned i;

	mss_enable_reg = ioremap(MSM_MSS_ENABLE_PHYS, 1);
	if (!mss_enable_reg)
		goto err;

	q6_reg_base[Q6_MODEM_FW] = ioremap(MSM_FW_QDSP6SS_PHYS, SZ_256);
	if (!q6_reg_base[Q6_MODEM_FW])
		goto err_modem_fw_q6;

	q6_reg_base[Q6_MODEM_SW] = ioremap(MSM_SW_QDSP6SS_PHYS, SZ_256);
	if (!q6_reg_base[Q6_MODEM_SW])
		goto err_modem_sw_q6;

	q6_reg_base[Q6_LPASS] = ioremap(MSM_LPASS_QDSP6SS_PHYS, SZ_256);
	if (!q6_reg_base[Q6_LPASS])
		goto err_lpass_q6;

	for (i = 0; i < ARRAY_SIZE(peripherals); i++)
		msm_pil_add_device(&peripherals[i]);

	return 0;

err_lpass_q6:
	iounmap(q6_reg_base[Q6_MODEM_SW]);
err_modem_sw_q6:
	iounmap(q6_reg_base[Q6_MODEM_FW]);
err_modem_fw_q6:
	iounmap(mss_enable_reg);
err:
	return -ENOMEM;
}
arch_initcall(msm_peripheral_reset_init);
