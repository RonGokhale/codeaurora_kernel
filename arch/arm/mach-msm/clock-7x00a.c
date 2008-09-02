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

/* MSM7201A Levels 3-6 all correspond to 1.2V, level 7 corresponds to 1.325V. */
enum {
	VDD_0 = 0,
	VDD_1 = 1,
	VDD_2 = 2,
	VDD_3 = 3,
	VDD_4 = 3,
	VDD_5 = 3,
	VDD_6 = 3,
	VDD_7 = 7,
	VDD_END
};

/*
 * ACPU speed table. Complete table is shown but certain speeds are commented
 * out to optimized speed switching. Initalize loops_per_jiffy to 0.
 *
 * Table stepping up/down is optimized for 256mhz jumps while staying on the
 * same PLL.
 */
#if (0)
struct clkctl_acpu_speed  acpu_freq_tbl[] = {
	{ 19200, ACPU_PLL_TCXO, 0, 0, 19200, 0, VDD_0, 0, 0, 8 },
	{ 61440, ACPU_PLL_0, 4, 3, 61440, 0, VDD_0, 0, 0, 8 },
	{ 81920, ACPU_PLL_0, 4, 2, 40960, 1, VDD_0, 0, 0, 8 },
	{ 96000, ACPU_PLL_1, 1, 7, 48000, 1, VDD_0, 0, 0, 9 },
	{ 122880, ACPU_PLL_0, 4, 1, 61440, 1, VDD_3, 0, 0, 8 },
	{ 128000, ACPU_PLL_1, 1, 5, 64000, 1, VDD_3, 0, 0, 12 },
	{ 176000, ACPU_PLL_2, 2, 5, 88000, 1, VDD_3, 0, 0, 11 },
	{ 192000, ACPU_PLL_1, 1, 3, 64000, 2, VDD_3, 0, 0, 12 },
	{ 245760, ACPU_PLL_0, 4, 0, 81920, 2, VDD_4, 0, 0, 12 },
	{ 256000, ACPU_PLL_1, 1, 2, 128000, 1, VDD_5, 0, 0, 12 },
	{ 264000, ACPU_PLL_2, 2, 3, 88000, 2, VDD_5, 0, 6, 13 },
	{ 352000, ACPU_PLL_2, 2, 2, 88000, 3, VDD_5, 0, 6, 13 },
	{ 384000, ACPU_PLL_1, 1, 1, 128000, 2, VDD_6, 0, 5, -1 },
	{ 528000, ACPU_PLL_2, 2, 1, 132000, 3, VDD_7, 0, 11, -1 },
	{ 0, 0, 0, 0, 0, 0, 0, 0, 0 ,0},
};
#else /* Table of freq we currently use. */
struct clkctl_acpu_speed  acpu_freq_tbl[] = {
	{ 19200, ACPU_PLL_TCXO, 0, 0, 19200, 0, VDD_0, 0, 0, 3 },
	{ 122880, ACPU_PLL_0, 4, 1, 61440, 1, VDD_3, 0, 0, 3 },
	{ 128000, ACPU_PLL_1, 1, 5, 64000, 1, VDD_3, 0, 0, 4 },
	{ 245760, ACPU_PLL_0, 4, 0, 81920, 2, VDD_4, 0, 0, 4 },
	{ 384000, ACPU_PLL_1, 1, 1, 128000, 2, VDD_6, 0, 2, -1 },
	{ 528000, ACPU_PLL_2, 2, 1, 132000, 3, VDD_7, 0, 3, -1 },
	{ 0, 0, 0, 0, 0, 0, 0},
};
#endif


/*
 * Various clocks in the system
 */

#define CLOCK(clk_name, clk_id, clk_flags) {	\
	.name = clk_name, \
	.owner = THIS_MODULE, \
	.id = clk_id, \
	.flags = clk_flags, \
	}

#define OFF CLKFLAG_AUTO_OFF
#define MINMAX CLKFLAG_USE_MIN_MAX_TO_SET

static struct clk clocks[] = {
	CLOCK("acpu_clk",	ACPU_CLK,	0),
	CLOCK("adm_clk",	ADM_CLK,	0),
	CLOCK("adsp_clk",	ADSP_CLK,	0),
	CLOCK("ebi1_clk",	EBI1_CLK,	0),
	CLOCK("ebi2_clk",	EBI2_CLK,	0),
	CLOCK("ecodec_clk",	ECODEC_CLK,	0),
	CLOCK("emdh_clk",	EMDH_CLK,	OFF),
	CLOCK("gp_clk",		GP_CLK,		0),
	CLOCK("grp_clk",	GRP_CLK,	OFF),
	CLOCK("i2c_clk",	I2C_CLK,	0),
	CLOCK("icodec_rx_clk",	ICODEC_RX_CLK,	0),
	CLOCK("icodec_tx_clk",	ICODEC_TX_CLK,	0),
	CLOCK("imem_clk",	IMEM_CLK,	OFF),
	CLOCK("mdc_clk",	MDC_CLK,	0),
	CLOCK("mdp_clk",	MDP_CLK,	OFF),
	CLOCK("pbus_clk",	PBUS_CLK,	0),
	CLOCK("pcm_clk",	PCM_CLK,	0),
	CLOCK("pmdh_clk",	PMDH_CLK,	OFF | MINMAX),
	CLOCK("sdac_clk",	SDAC_CLK,	OFF),
	CLOCK("sdc1_clk",	SDC1_CLK,	OFF),
	CLOCK("sdc1_pclk",	SDC1_PCLK,	OFF),
	CLOCK("sdc2_clk",	SDC2_CLK,	OFF),
	CLOCK("sdc2_pclk",	SDC2_PCLK,	OFF),
	CLOCK("sdc3_clk",	SDC3_CLK,	OFF),
	CLOCK("sdc3_pclk",	SDC3_PCLK,	OFF),
	CLOCK("sdc4_clk",	SDC4_CLK,	OFF),
	CLOCK("sdc4_pclk",	SDC4_PCLK,	OFF),
	CLOCK("tsif_clk",	TSIF_CLK,	0),
	CLOCK("tsif_ref_clk",	TSIF_REF_CLK,	0),
	CLOCK("tv_dac_clk",	TV_DAC_CLK,	0),
	CLOCK("tv_enc_clk",	TV_ENC_CLK,	0),
	CLOCK("uart1_clk",	UART1_CLK,	OFF),
	CLOCK("uart2_clk",	UART2_CLK,	0),
	CLOCK("uart3_clk",	UART3_CLK,	OFF),
	CLOCK("uart1dm_clk",	UART1DM_CLK,	OFF),
	CLOCK("uart2dm_clk",	UART2DM_CLK,	0),
	CLOCK("usb_hs_clk",	USB_HS_CLK,	OFF),
	CLOCK("usb_hs_pclk",	USB_HS_PCLK,	OFF),
	CLOCK("usb_otg_clk",	USB_OTG_CLK,	0),
	CLOCK("vdc_clk",	VDC_CLK,	OFF | MINMAX),
	CLOCK("vfe_clk",	VFE_CLK,	OFF),
	CLOCK("vfe_mdc_clk",	VFE_MDC_CLK,	OFF),
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
		   clkdata->vdd_switch_time_us,
		   clkdata->power_collapse_khz,
		   clkdata->wait_for_irq_khz);

	/* Register all the clocks for the MSM7k */
	for (n = 0; n < ARRAY_SIZE(clocks); n++)
		clk_register(clocks + n);
}
