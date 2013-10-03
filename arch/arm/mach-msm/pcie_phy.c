/* Copyright (c) 2013, The Linux Foundation. All rights reserved.
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

/*
 * MSM PCIe PHY driver.
 */

#include <linux/io.h>
#include <mach/msm_iomap.h>
#include "pcie.h"
#include "pcie_phy.h"

static void *base0;
static void *base1;

static inline void write_phy(void *base, u32 offset, u32 value)
{
	writel_relaxed(value, base + offset);
	wmb();
}

void pcie_phy_init(u32 rc)
{
	void *base;

	PCIE_DBG("Initializing 28nm QMP phy - 19.2MHz for RC %d\n", rc);

	if (rc == 0) {
		if (!base0) {
			base = ioremap(PCIe0_PHY_BASE, 0x1000);
			base0 = base;
		} else
			base = base0;
	} else {
		if (!base1) {
			base = ioremap(PCIe1_PHY_BASE, 0x1000);
			base1 = base;
		} else
			base = base1;
	}

	write_phy(base, PCIE_PHY_POWER_DOWN_CONTROL,		0x03);
	write_phy(base, QSERDES_COM_SYSCLK_EN_SEL,		0x08);
	write_phy(base, QSERDES_COM_DEC_START1,			0x82);
	write_phy(base, QSERDES_COM_DEC_START2,			0x03);
	write_phy(base, QSERDES_COM_DIV_FRAC_START1,		0xd5);
	write_phy(base, QSERDES_COM_DIV_FRAC_START2,		0xaa);
	write_phy(base, QSERDES_COM_DIV_FRAC_START3,		0x13);
	write_phy(base, QSERDES_COM_PLLLOCK_CMP_EN,		0x01);
	write_phy(base, QSERDES_COM_PLLLOCK_CMP1,		0x2b);
	write_phy(base, QSERDES_COM_PLLLOCK_CMP2,		0x68);
	write_phy(base, QSERDES_COM_PLL_CRCTRL,			0xff);
	write_phy(base, QSERDES_COM_PLL_CP_SETI,		0x3f);
	write_phy(base, QSERDES_COM_PLL_IP_SETP,		0x07);
	write_phy(base, QSERDES_COM_PLL_CP_SETP,		0x03);
	write_phy(base, QSERDES_RX_CDR_CONTROL,			0xf3);
	write_phy(base, QSERDES_RX_CDR_CONTROL2,		0x6b);
	write_phy(base, QSERDES_COM_RESETSM_CNTRL,		0x10);
	write_phy(base, QSERDES_RX_RX_TERM_HIGHZ_CM_AC_COUPLE,	0x87);
	write_phy(base, QSERDES_RX_RX_EQ_GAIN12,		0x54);
	write_phy(base, PCIE_PHY_POWER_STATE_CONFIG1,		0xa3);
	write_phy(base, PCIE_PHY_POWER_STATE_CONFIG2,		0xcb);
	write_phy(base, QSERDES_COM_PLL_RXTXEPCLK_EN,		0x10);
	write_phy(base, PCIE_PHY_ENDPOINT_REFCLK_DRIVE,		0x10);
	write_phy(base, PCIE_PHY_SW_RESET,			0x00);
	write_phy(base, PCIE_PHY_START,				0x03);
}

bool pcie_phy_is_ready(u32 rc)
{
	void *base;

	if (rc == 0)
		base = base0;
	else
		base = base1;

	if (!base)
		return false;

	if (readl_relaxed(base + PCIE_PHY_PCS_STATUS) & BIT(6))
		return false;
	else
		return true;
}
