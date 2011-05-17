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
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/regulator/consumer.h>
#include <mach/msm_xo.h>
#include <mach/msm_iomap.h>

#include "wcnss_riva.h"

static void __iomem *msm_riva_base;

#define MSM_RIVA_PHYS                     0x03204000
#define RIVA_PMU_CFG                      (msm_riva_base + 0x28)
#define RIVA_PMU_CFG_IRIS_XO_CFG          BIT(3)
#define RIVA_PMU_CFG_IRIS_XO_EN           BIT(4)
#define RIVA_PMU_CFG_GC_BUS_MUX_SEL_TOP   BIT(5)
#define RIVA_PMU_CFG_IRIS_XO_CFG_STS      BIT(6) /* 1: in progress, 0: done */

#define RIVA_PMU_CFG_IRIS_XO_MODE         0x6
#define RIVA_PMU_CFG_IRIS_XO_MODE_48      (2 << 1)

#define VREG_NOT_CONFIGURED         0x0000
#define VREG_GET_REGULATOR_MASK     0x0001
#define VREG_SET_VOLTAGE_MASK       0x0002
#define VREG_PIN_CONTROL_MASK       0x0004
#define VREG_ENABLE_MASK            0x0008

static struct vregs_info {
	const char * const name;
	int state;
	const int nominal_min;
	const int low_power_min;
	const int max_voltage;
	const bool is_pin_control;
	struct regulator *regulator;
} vregs[] = {
	{"8921_lvs1", VREG_NOT_CONFIGURED, 1800000, 0, 1800000, 0, NULL},
	{"8921_lvs2", VREG_NOT_CONFIGURED, 1200000, 0, 1200000, 0, NULL},
	{"8921_s2",   VREG_NOT_CONFIGURED, 1300000, 0, 1300000, 0, NULL},
	{"8921_l10",  VREG_NOT_CONFIGURED, 2900000, 0, 2900000, 0, NULL},
};

static struct msm_xo_voter *wlan_clock;
static const char *id = "WLAN";

static int configure_iris_xo(bool use_48mhz_xo, int on)
{
	u32 reg;
	int rc = 0;

	if (on) {
		msm_riva_base = ioremap(MSM_RIVA_PHYS, SZ_256);
		if (!msm_riva_base) {
			pr_err("ioremap MSM_RIVA_PHYS failed\n");
			goto fail;
		}

		/* Enable IRIS XO */
		reg = readl_relaxed(RIVA_PMU_CFG);
		reg |= RIVA_PMU_CFG_GC_BUS_MUX_SEL_TOP |
				RIVA_PMU_CFG_IRIS_XO_EN;
		writel_relaxed(reg, RIVA_PMU_CFG);

		/* Clear XO_MODE[b2:b1] bits. Clear implies 19.2 MHz TCXO */
		reg &= ~(RIVA_PMU_CFG_IRIS_XO_MODE);

		if (use_48mhz_xo)
			reg |= RIVA_PMU_CFG_IRIS_XO_MODE_48;

		writel_relaxed(reg, RIVA_PMU_CFG);

		/* Start IRIS XO configuration */
		reg |= RIVA_PMU_CFG_IRIS_XO_CFG;
		writel_relaxed(reg, RIVA_PMU_CFG);

		/* Wait for XO configuration to finish */
		while (readl_relaxed(RIVA_PMU_CFG) &
						RIVA_PMU_CFG_IRIS_XO_CFG_STS)
			cpu_relax();

		/* Stop IRIS XO configuration */
		reg &= ~(RIVA_PMU_CFG_GC_BUS_MUX_SEL_TOP |
				RIVA_PMU_CFG_IRIS_XO_CFG);
		writel_relaxed(reg, RIVA_PMU_CFG);

		if (!use_48mhz_xo) {
			wlan_clock = msm_xo_get(MSM_XO_TCXO_A0, id);
			if (IS_ERR(wlan_clock)) {
				rc = PTR_ERR(wlan_clock);
				pr_err("Failed to get MSM_XO_TCXO_A0 voter"
							" (%d)\n", rc);
				goto fail;
			}

			rc = msm_xo_mode_vote(wlan_clock, MSM_XO_MODE_ON);
			if (rc < 0) {
				pr_err("Configuring MSM_XO_MODE_ON failed"
							" (%d)\n", rc);
				goto msm_xo_vote_fail;
			}
		}
	}  else {
		if (wlan_clock != NULL && !use_48mhz_xo) {
			rc = msm_xo_mode_vote(wlan_clock, MSM_XO_MODE_OFF);
			if (rc < 0)
				pr_err("Configuring MSM_XO_MODE_OFF failed"
							" (%d)\n", rc);
		}
	}

	return rc;

msm_xo_vote_fail:
	msm_xo_put(wlan_clock);

fail:
	return rc;
}


static void wcnss_wlan_vregs_off(void)
{
	int i, rc = 0;

	for (i = 0; i < ARRAY_SIZE(vregs); i++) {
		if (vregs[i].state == VREG_NOT_CONFIGURED)
			continue;

		/* Remove pin control */
		if (vregs[i].state & VREG_PIN_CONTROL_MASK) {
			rc = regulator_set_mode(vregs[i].regulator,
					REGULATOR_MODE_NORMAL);
			if (rc)
				pr_err("regulator_set_mode(%s) failed (%d)\n",
						vregs[i].name, rc);
		}

		/* Set voltage to lowest level */
		if (vregs[i].state & VREG_SET_VOLTAGE_MASK) {
			rc = regulator_set_voltage(vregs[i].regulator,
					vregs[i].low_power_min,
					vregs[i].max_voltage);
			if (rc)
				pr_err("regulator_set_voltage(%s) failed (%d)\n",
						vregs[i].name, rc);
		}

		/* Disable regulator */
		if (vregs[i].state & VREG_ENABLE_MASK) {
			rc = regulator_disable(vregs[i].regulator);
			if (rc < 0)
				pr_err("vreg %s disable failed (%d)\n",
						vregs[i].name, rc);
		}

		/* Free the regulator source */
		if (vregs[i].state & VREG_GET_REGULATOR_MASK)
			regulator_put(vregs[i].regulator);

		vregs[i].state = VREG_NOT_CONFIGURED;
	}
}

int wcnss_wlan_power(struct device *dev,
		struct wcnss_wlan_config *cfg,
		enum wcnss_opcode on)
{
	int i = 0;
	int rc = 0;

	/* WLAN regulator settings */
	if (on) {
		for (i = 0; i < ARRAY_SIZE(vregs); i++) {
			/* Get regulator source */
			vregs[i].regulator = regulator_get(dev, vregs[i].name);
			if (IS_ERR(vregs[i].regulator)) {
				rc = PTR_ERR(vregs[i].regulator);
				pr_err("regulator get of %s failed (%d)\n",
						vregs[i].name, rc);
				goto fail;
			}
			vregs[i].state |= VREG_GET_REGULATOR_MASK;

			/* Set voltage to nominal level */
			rc = regulator_set_voltage(vregs[i].regulator,
					vregs[i].nominal_min,
					vregs[i].max_voltage);
			if (rc) {
				pr_err("regulator_set_voltage(%s) failed (%d)\n",
						vregs[i].name, rc);
				goto fail;
			}
			vregs[i].state |= VREG_SET_VOLTAGE_MASK;

			/* Vote for pin control (if needed) */
			if (vregs[i].is_pin_control) {
				rc = regulator_set_mode(vregs[i].regulator,
						REGULATOR_MODE_IDLE);
				vregs[i].state |= VREG_PIN_CONTROL_MASK;
			} else {
				rc = regulator_set_mode(vregs[i].regulator,
						REGULATOR_MODE_NORMAL);
			}
			if (rc) {
				pr_err("regulator_set_mode(%s) failed (%d)\n",
						vregs[i].name, rc);
				goto fail;
			}

			/* Enable the regulator */
			rc = regulator_enable(vregs[i].regulator);
			if (rc < 0) {
				pr_err("vreg %s enable failed (%d)\n",
						vregs[i].name, rc);
				goto fail;
			}
			vregs[i].state |= VREG_ENABLE_MASK;
		}
	} else {
		wcnss_wlan_vregs_off();
	}

	/* Configure IRIS XO */
	rc = configure_iris_xo(cfg->use_48mhz_xo, on);
	if (rc && on)
		goto fail;

	return rc;

fail:
	wcnss_wlan_vregs_off();
	return rc;
}
EXPORT_SYMBOL(wcnss_wlan_power);

