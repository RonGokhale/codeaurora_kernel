/* linux/arch/arm/mach-msm/common.c
 *
 * Common setup code for MSM7K Boards
 *
 * Copyright (C) 2007 Google, Inc.
 * Author: San Mehat <san@android.com>
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
#include <linux/platform_device.h>

#include <asm/arch/clock.h>
#include "clock.h"

/*
 * ACPU speed table
 */
struct clkctl_acpu_speed  acpu_freq_tbl[] = {
	{ 19200, ACPU_PLL_TCXO, 0, 0, 19200, 0, 0},
	{ 61440, ACPU_PLL_0, 4, 3, 61440, 0, 0}, /* VDD assumed */
	{ 81920, ACPU_PLL_0, 4, 2, 40960, 1, 0}, /* VDD assumed */
	{ 96000, ACPU_PLL_1, 1, 7, 48000, 1, 0}, /* VDD assumed */
	{ 122880, ACPU_PLL_0, 4, 1, 61440, 1, 3},
	{ 128000, ACPU_PLL_1, 1, 5, 64000, 1, 3}, /* VDD assumed */
	{ 176000, ACPU_PLL_2, 2, 5, 88000, 1, 3}, /* VDD assumed */
	{ 192000, ACPU_PLL_1, 1, 3, 64000, 2, 3}, /* VDD assumed */
	{ 245760, ACPU_PLL_0, 4, 0, 81920, 2, 4},
	{ 256000, ACPU_PLL_1, 1, 2, 85333, 2, 5},
	{ 264000, ACPU_PLL_2, 2, 3, 88000, 2, 5}, /* VDD assumed */
	{ 352000, ACPU_PLL_2, 2, 2, 88000, 3, 5}, /* VDD assumed */
	{ 384000, ACPU_PLL_1, 1, 1, 128000, 2, 6},
	{ 528000, ACPU_PLL_2, 2, 1, 132000, 3, 6},
	{ 0, 0, 0, 0, 0, 0, 0},
};

/*
 * Various clocks in the system
 */

#define MSM_CLOCK_FLAGS(clk_name, clk_id, clk_flags) {	\
	.name = clk_name, \
	.owner = THIS_MODULE, \
	.id = clk_id, \
	.flags = clk_flags, \
	}

#define MSM_CLOCK(name, id) MSM_CLOCK_FLAGS(name, id, 0)
#define MSM_CLOCK_MM(name, id) MSM_CLOCK_FLAGS(name, id, CLKFLAG_USE_MIN_MAX_TO_SET)

static struct clk clocks[] = {
	MSM_CLOCK("acpu_clk", ACPU_CLK),
	MSM_CLOCK("adm_clk", ADM_CLK),
	MSM_CLOCK("adsp_clk", ADSP_CLK),
	MSM_CLOCK("ebi1_clk", EBI1_CLK),
	MSM_CLOCK("ebi2_clk", EBI2_CLK),
	MSM_CLOCK("ecodec_clk", ECODEC_CLK),
	MSM_CLOCK("emdh_clk", EMDH_CLK),
	MSM_CLOCK("gp_clk", GP_CLK),
	MSM_CLOCK("grp_clk", GRP_CLK),
	MSM_CLOCK("i2c_clk", I2C_CLK),
	MSM_CLOCK("icodec_rx_clk", ICODEC_RX_CLK),
	MSM_CLOCK("icodec_tx_clk", ICODEC_TX_CLK),
	MSM_CLOCK("imem_clk", IMEM_CLK),
	MSM_CLOCK("mdc_clk", MDC_CLK),
	MSM_CLOCK("mdp_clk", MDP_CLK),
	MSM_CLOCK("pbus_clk", PBUS_CLK),
	MSM_CLOCK("pcm_clk", PCM_CLK),
	MSM_CLOCK_MM("pmdh_clk", PMDH_CLK),
	MSM_CLOCK("sdac_clk", SDAC_CLK),
	MSM_CLOCK("sdc1_clk", SDC1_CLK),
	MSM_CLOCK("sdc1_pclk", SDC1_PCLK),
	MSM_CLOCK("sdc2_clk", SDC2_CLK),
	MSM_CLOCK("sdc2_pclk", SDC2_PCLK),
	MSM_CLOCK("sdc3_clk", SDC3_CLK),
	MSM_CLOCK("sdc3_pclk", SDC3_PCLK),
	MSM_CLOCK("sdc4_clk", SDC4_CLK),
	MSM_CLOCK("sdc4_pclk", SDC4_PCLK),
	MSM_CLOCK("tsif_clk", TSIF_CLK),
	MSM_CLOCK("tsif_ref_clk", TSIF_REF_CLK),
	MSM_CLOCK("tv_dac_clk", TV_DAC_CLK),
	MSM_CLOCK("tv_enc_clk", TV_ENC_CLK),
	MSM_CLOCK("uart1_clk", UART1_CLK),
	MSM_CLOCK("uart2_clk", UART2_CLK),
	MSM_CLOCK("uart3_clk", UART3_CLK),
	MSM_CLOCK("uart1dm_clk", UART1DM_CLK),
	MSM_CLOCK("uart2dm_clk", UART2DM_CLK),
	MSM_CLOCK("usb_hs_clk", USB_HS_CLK),
	MSM_CLOCK("usb_hs_pclk", USB_HS_PCLK),
	MSM_CLOCK("usb_otg_clk", USB_OTG_CLK),
	MSM_CLOCK_MM("vdc_clk", VDC_CLK),
	MSM_CLOCK("vfe_clk", VFE_CLK),
	MSM_CLOCK("vfe_mdc_clk", VFE_MDC_CLK),
};

struct clk *msm_clock_get_nth(unsigned index)
{
	if (index < ARRAY_SIZE(clocks)) {
		return clocks + index;
	} else {
		return 0;
	}
}

void __init msm_clock_init(struct msm_clock_platform_data *clkdata)
{
	unsigned n;

	/* Start MSM clock driver */
	clock_init(clkdata->acpu_switch_time_us,
		   clkdata->max_speed_delta_khz,
		   clkdata->vdd_switch_time_us);

	/* Register all the clocks for the MSM7k */
	for (n = 0; n < ARRAY_SIZE(clocks); n++)
		clk_register(clocks + n);
}
