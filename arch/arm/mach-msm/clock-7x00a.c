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
	{ 176000, ACPU_PLL_3, 3, 5, 88000, 1, 3}, /* VDD assumed */
	{ 192000, ACPU_PLL_1, 1, 3, 64000, 2, 3}, /* VDD assumed */
	{ 245760, ACPU_PLL_0, 4, 0, 81920, 2, 4},
	{ 256000, ACPU_PLL_1, 1, 2, 85333, 2, 5},
	{ 264000, ACPU_PLL_3, 3, 3, 88000, 2, 5}, /* VDD assumed */
	{ 352000, ACPU_PLL_3, 3, 2, 88000, 3, 5}, /* VDD assumed */
	{ 384000, ACPU_PLL_1, 1, 1, 128000, 2, 6},
	{ 528000, ACPU_PLL_3, 3, 1, 132000, 3, 7},
	{ 0, 0, 0, 0, 0, 0, 0},
};

/*
 * Various clocks in the system
 */

static struct clk clocks[] = {
	{
		.name		= "acpu_clk",
		.owner		= THIS_MODULE,
		.id		= ACPU_CLK,
	},
	{
		.name		= "adm_clk",
		.owner		= THIS_MODULE,
		.id		= ADM_CLK,
	},
	{
		.name		= "adsp_clk",
		.owner		= THIS_MODULE,
		.id		= ADSP_CLK,
	},
	{
		.name		= "ebi1_clk",
		.owner		= THIS_MODULE,
		.id		= EBI1_CLK,
	},
	{
		.name		= "ebi2_clk",
		.owner		= THIS_MODULE,
		.id		= EBI2_CLK,
	},
	{
		.name		= "ecodec_clk",
		.owner		= THIS_MODULE,
		.id		= ECODEC_CLK,
	},
	{
		.name		= "emdh_clk",
		.owner		= THIS_MODULE,
		.id		= EMDH_CLK,
	},
	{
		.name		= "gp_clk",
		.owner		= THIS_MODULE,
		.id		= GP_CLK,
	},
	{
		.name		= "grp_clk",
		.owner		= THIS_MODULE,
		.id		= GRP_CLK,
	},
	{
		.name		= "i2c_clk",
		.owner		= THIS_MODULE,
		.id		= I2C_CLK,
	},
	{
		.name		= "icodec_rx_clk",
		.owner		= THIS_MODULE,
		.id		= ICODEC_RX_CLK,
	},
	{
		.name		= "icodec_tx_clk",
		.owner		= THIS_MODULE,
		.id		= ICODEC_TX_CLK,
	},
	{
		.name		= "imem_clk",
		.owner		= THIS_MODULE,
		.id		= IMEM_CLK,
	},
	{
		.name		= "mdc_clk",
		.owner		= THIS_MODULE,
		.id		= MDC_CLK,
	},
	{
		.name		= "mdp_clk",
		.owner		= THIS_MODULE,
		.id		= MDP_CLK,
	},
	{
		.name		= "pbus_clk",
		.owner		= THIS_MODULE,
		.id		= PBUS_CLK,
	},
	{
		.name		= "pcm_clk",
		.owner		= THIS_MODULE,
		.id		= PCM_CLK,
	},
	{
		.name		= "pmdh_clk",
		.owner		= THIS_MODULE,
		.id		= PMDH_CLK,
	},
	{
		.name		= "sdac_clk",
		.owner		= THIS_MODULE,
		.id		= SDAC_CLK,
	},
	{
		.name		= "sdc1_clk",
		.owner		= THIS_MODULE,
		.id		= SDC1_CLK,
	},
	{
		.name		= "sdc2_clk",
		.owner		= THIS_MODULE,
		.id		= SDC2_CLK,
	},
	{
		.name		= "sdc3_clk",
		.owner		= THIS_MODULE,
		.id		= SDC3_CLK,
	},
	{
		.name		= "sdc4_clk",
		.owner		= THIS_MODULE,
		.id		= SDC4_CLK,
	},
	{
		.name		= "tsif_clk",
		.owner		= THIS_MODULE,
		.id		= TSIF_CLK,
	},
	{
		.name		= "tsif_ref_clk",
		.owner		= THIS_MODULE,
		.id		= TSIF_REF_CLK,
	},
	{
		.name		= "tv_dac_clk",
		.owner		= THIS_MODULE,
		.id		= TV_DAC_CLK,
	},
	{
		.name		= "tv_enc_clk",
		.owner		= THIS_MODULE,
		.id		= TV_ENC_CLK,
	},
	{
		.name		= "uart1_clk",
		.owner		= THIS_MODULE,
		.id		= UART1_CLK,
	},
	{
		.name		= "uart2_clk",
		.owner		= THIS_MODULE,
		.id		= UART2_CLK,
	},
	{
		.name		= "uart3_clk",
		.owner		= THIS_MODULE,
		.id		= UART3_CLK,
	},
	{
		.name		= "uart1dm_clk",
		.owner		= THIS_MODULE,
		.id		= UART1DM_CLK,
	},
	{
		.name		= "uart2dm_clk",
		.owner		= THIS_MODULE,
		.id		= UART2DM_CLK,
	},
	{
		.name		= "usb_hs_clk",
		.owner		= THIS_MODULE,
		.id		= USB_HS_CLK,
	},
	{
		.name		= "usb_otg_clk",
		.owner		= THIS_MODULE,
		.id		= USB_OTG_CLK,
	},
	{
		.name		= "vdc_clk",
		.owner		= THIS_MODULE,
		.id		= VDC_CLK,
	},
	{
		.name		= "vfe_clk",
		.owner		= THIS_MODULE,
		.id		= VFE_CLK,
	},
	{
		.name		= "vfe_mdc_clk",
		.owner		= THIS_MODULE,
		.id		= VFE_MDC_CLK,
	},
};

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
