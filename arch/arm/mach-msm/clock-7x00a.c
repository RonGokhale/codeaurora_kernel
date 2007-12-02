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
 * Configuration settings for each available ACPU clock speed
 */

const clkctl_acpu_clk_cfg_t acpu_clk_cfg[CLKCTL_NUM_ACPU_SPEEDS] =
{
	{
		19200,          /* A11S Clock Freq = 19.2 MHz   */
		CLKCTL_TCXO,    /* TCXO                         */
		0x0,            /* CLK_SRCx_SEL = TCXO          */
		0x0,            /* CLK_SRCx_DIV = 1             */
		19200,          /* AHB Clock Freq = 19.2 MHz    */
		0x0             /* AHB_CLK_DIV = Divide by 1    */
	},
	{
		61440,          /* A11S Clock Freq = 61.44 MHz  */
		CLKCTL_PLL0,    /* PLL0 (245.76 MHz)            */
		0x4,            /* CLK_SRCx_SEL = PLL0          */
		0x3,            /* CLK_SRCx_DIV = 4             */
		61440,          /* AHB Clock Freq = 61.44 MHz   */
		0x0             /* AHB_CLK_DIV = Divide by 1    */
	},
	{
		81920,          /* A11S Clock Freq = 81.92 MHz  */
		CLKCTL_PLL0,    /* PLL0 (245.76 MHz)            */
		0x4,            /* CLK_SRCx_SEL = Modem PLL     */
		0x2,            /* CLK_SRCx_DIV = 3             */
		40960,          /* AHB Clock Freq = 40.96 MHz   */
		0x1             /* AHB_CLK_DIV = Divide by 2    */
	},
	{
		96000,          /* A11S Clock Freq = 96 MHz     */
		CLKCTL_PLL1,    /* PLL1 (768 MHz)               */
		0x1,            /* CLK_SRCx_SEL = PLL1          */
		0x7,            /* CLK_SRCx_DIV = 8             */
		48000,          /* AHB Clock Freq = 48 MHz      */
		0x1             /* AHB_CLK_DIV = Divide by 2    */
	},
	{
		122880,         /* A11S Clock Freq = 122.88 MHz */
		CLKCTL_PLL0,    /* PLL0 (245.76 MHz)            */
		0x4,            /* CLK_SRCx_SEL = PLL0          */
		0x1,            /* CLK_SRCx_DIV = 2             */
		61440,          /* AHB Clock Freq = 61.44 MHz   */
		0x1             /* AHB_CLK_DIV = Divide by 2    */
	},
	{
		128000,         /* A11S Clock Freq = 128 MHz    */
		CLKCTL_PLL1,    /* PLL1 (768 MHz)               */
		0x1,            /* CLK_SRCx_SEL = PLL1          */
		0x5,            /* CLK_SRCx_DIV = 6             */
		64000,          /* AHB Clock Freq = 64 MHz      */
		0x1             /* AHB_CLK_DIV = Divide by 2    */
	},
	{
		176000,         /* A11S Clock Freq = 176 MHz    */
		CLKCTL_PLL3,    /* PLL3 (1056 MHz)              */
		0x3,            /* CLK_SRCx_SEL = PLL3          */
		0x5,            /* CLK_SRCx_DIV = 6             */
		88000,          /* AHB Clock Freq = 88 MHz      */
		0x1             /* AHB_CLK_DIV = Divide by 2    */
	},
	{
		192000,         /* A11S Clock Freq = 192 MHz    */
		CLKCTL_PLL1,    /* PLL1 (768 MHz)               */
		0x1,            /* CLK_SRCx_SEL = PLL1          */
		0x3,            /* CLK_SRCx_DIV = 4             */
		64000,          /* AHB Clock Freq = 64 MHz      */
		0x2             /* AHB_CLK_DIV = Divide by 3    */
	},
	{
		245760,         /* A11S Clock Freq = 245.76 MHz */
		CLKCTL_PLL0,    /* PLL0 (245.76 MHz)            */
		0x4,            /* CLK_SRCx_SEL = PLL0          */
		0x0,            /* CLK_SRCx_DIV = 1             */
		81920,          /* AHB Clock Freq = 81.92 MHz   */
		0x2             /* AHB_CLK_DIV = Divide by 3    */
	},
	{
		256000,         /* A11S Clock Freq = 256 MHz    */
		CLKCTL_PLL1,    /* PLL1 (768 MHz)               */
		0x1,            /* CLK_SRCx_SEL = PLL1          */
		0x2,            /* CLK_SRCx_DIV = 3             */
		85333,          /* AHB Clock Freq = 85.3 MHz    */
		0x2             /* AHB_CLK_DIV = Divide by 3    */
	},
	{
		264000,         /* A11S Clock Freq = 264 MHz    */
		CLKCTL_PLL3,    /* PLL3 (1056 MHz)              */
		0x3,            /* CLK_SRCx_SEL = PLL3          */
		0x3,            /* CLK_SRCx_DIV = 4             */
		88000,          /* AHB Clock Freq = 88 MHz      */
		0x2             /* AHB_CLK_DIV = Divide by 3    */
	},
	{
		352000,         /* A11S Clock Freq = 352 MHz    */
		CLKCTL_PLL3,    /* PLL3 (1056 MHz)              */
		0x3,            /* CLK_SRCx_SEL = PLL3          */
		0x2,            /* CLK_SRCx_DIV = 3             */
		88000,          /* AHB Clock Freq = 88 MHz      */
		0x3             /* AHB_CLK_DIV = Divide by 4    */
	},
	{
		384000,         /* A11S Clock Freq = 384 MHz    */
		CLKCTL_PLL1,    /* PLL1 (768 MHz)               */
		0x1,            /* CLK_SRCx_SEL = PLL1          */
		0x1,            /* CLK_SRCx_DIV = 2             */
		128000,         /* AHB Clock Freq = 128 MHz     */
		0x2             /* AHB_CLK_DIV = Divide by 3    */
	},
	{
		528000,         /* A11S Clock Freq = 528 MHz    */
		CLKCTL_PLL3,    /* PLL3 (1056 MHz)              */
		0x3,            /* CLK_SRCx_SEL = PLL3          */
		0x1,            /* CLK_SRCx_DIV = 2             */
		132000,         /* AHB Clock Freq = 132 MHz     */
		0x3             /* AHB_CLK_DIV = Divide by 4    */
	}
};


/*
 * Configuration settings for each available ARM11 performance level.
 */
const clkctl_acpu_perf_cfg_t acpu_perf_cfg[CLKCTL_NUM_ACPU_PERF_LEVELS] =
{
	{
		CLKCTL_ACPU_SPEED_TCXO,
		CLKCTL_ACPU_VDD_LEVEL_0
	},
	{
		CLKCTL_ACPU_SPEED_122P88_MHZ,
		CLKCTL_ACPU_VDD_LEVEL_3
	},
	{
		CLKCTL_ACPU_SPEED_245P76_MHZ,
		CLKCTL_ACPU_VDD_LEVEL_4
	},
	{
		CLKCTL_ACPU_SPEED_256_MHZ,
		CLKCTL_ACPU_VDD_LEVEL_5
	},
	{
		CLKCTL_ACPU_SPEED_384_MHZ,
		CLKCTL_ACPU_VDD_LEVEL_6
	},
	{
		CLKCTL_ACPU_SPEED_528_MHZ,
		CLKCTL_ACPU_VDD_LEVEL_7
	}
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
		.a9_controlled	= 1,
	},
	{
		.name		= "adsp_clk",
		.owner		= THIS_MODULE,
		.id		= ADSP_CLK,
		.a9_controlled	= 1,
	},
	{
		.name		= "ebi1_clk",
		.owner		= THIS_MODULE,
		.id		= EBI1_CLK,
		.a9_controlled	= 1,
	},
	{
		.name		= "ebi2_clk",
		.owner		= THIS_MODULE,
		.id		= EBI2_CLK,
		.a9_controlled	= 1,
	},
	{
		.name		= "ecodec_clk",
		.owner		= THIS_MODULE,
		.id		= ECODEC_CLK,
		.a9_controlled	= 1,
	},
	{
		.name		= "emdh_clk",
		.owner		= THIS_MODULE,
		.id		= EMDH_CLK,
		.a9_controlled	= 1,
	},
	{
		.name		= "gp_clk",
		.owner		= THIS_MODULE,
		.id		= GP_CLK,
		.a9_controlled	= 1,
	},
	{
		.name		= "grp_clk",
		.owner		= THIS_MODULE,
		.id		= GRP_CLK,
		.a9_controlled	= 1,
	},
	{
		.name		= "i2c_clk",
		.owner		= THIS_MODULE,
		.id		= I2C_CLK,
		.a9_controlled	= 1,
	},
	{
		.name		= "icodec_rx_clk",
		.owner		= THIS_MODULE,
		.id		= ICODEC_RX_CLK,
		.a9_controlled	= 1,
	},
	{
		.name		= "icodec_tx_clk",
		.owner		= THIS_MODULE,
		.id		= ICODEC_TX_CLK,
		.a9_controlled	= 1,
	},
	{
		.name		= "imem_clk",
		.owner		= THIS_MODULE,
		.id		= IMEM_CLK,
		.a9_controlled	= 1,
	},
	{
		.name		= "mdc_clk",
		.owner		= THIS_MODULE,
		.id		= MDC_CLK,
		.a9_controlled	= 1,
	},
	{
		.name		= "mdp_clk",
		.owner		= THIS_MODULE,
		.id		= MDP_CLK,
		.a9_controlled	= 1,
	},
	{
		.name		= "pbus_clk",
		.owner		= THIS_MODULE,
		.id		= PBUS_CLK,
		.a9_controlled	= 1,
	},
	{
		.name		= "pcm_clk",
		.owner		= THIS_MODULE,
		.id		= PCM_CLK,
		.a9_controlled	= 1,
	},
	{
		.name		= "pmdh_clk",
		.owner		= THIS_MODULE,
		.id		= PMDH_CLK,
		.a9_controlled	= 1,
	},
	{
		.name		= "sdac_clk",
		.owner		= THIS_MODULE,
		.id		= SDAC_CLK,
		.a9_controlled	= 1,
	},
	{
		.name		= "sdc1_clk",
		.owner		= THIS_MODULE,
		.id		= SDC1_CLK,
		.a9_controlled	= 1,
	},
	{
		.name		= "sdc2_clk",
		.owner		= THIS_MODULE,
		.id		= SDC2_CLK,
		.a9_controlled	= 1,
	},
	{
		.name		= "sdc3_clk",
		.owner		= THIS_MODULE,
		.id		= SDC3_CLK,
		.a9_controlled	= 1,
	},
	{
		.name		= "sdc4_clk",
		.owner		= THIS_MODULE,
		.id		= SDC4_CLK,
		.a9_controlled	= 1,
	},
	{
		.name		= "tsif_clk",
		.owner		= THIS_MODULE,
		.id		= TSIF_CLK,
		.a9_controlled	= 1,
	},
	{
		.name		= "tsif_ref_clk",
		.owner		= THIS_MODULE,
		.id		= TSIF_REF_CLK,
		.a9_controlled	= 1,
	},
	{
		.name		= "tv_dac_clk",
		.owner		= THIS_MODULE,
		.id		= TV_DAC_CLK,
		.a9_controlled	= 1,
	},
	{
		.name		= "tv_enc_clk",
		.owner		= THIS_MODULE,
		.id		= TV_ENC_CLK,
		.a9_controlled	= 1,
	},
	{
		.name		= "uart1_clk",
		.owner		= THIS_MODULE,
		.id		= UART1_CLK,
		.a9_controlled	= 1,
	},
	{
		.name		= "uart2_clk",
		.owner		= THIS_MODULE,
		.id		= UART2_CLK,
		.a9_controlled	= 1,
	},
	{
		.name		= "uart3_clk",
		.owner		= THIS_MODULE,
		.id		= UART3_CLK,
		.a9_controlled	= 1,
	},
	{
		.name		= "uart1dm_clk",
		.owner		= THIS_MODULE,
		.id		= UART1DM_CLK,
		.a9_controlled	= 1,
	},
	{
		.name		= "uart2dm_clk",
		.owner		= THIS_MODULE,
		.id		= UART2DM_CLK,
		.a9_controlled	= 1,
	},
	{
		.name		= "usb_hs_clk",
		.owner		= THIS_MODULE,
		.id		= USB_HS_CLK,
		.a9_controlled	= 1,
	},
	{
		.name		= "usb_otg_clk",
		.owner		= THIS_MODULE,
		.id		= USB_OTG_CLK,
		.a9_controlled	= 1,
	},
	{
		.name		= "vdc_clk",
		.owner		= THIS_MODULE,
		.id		= VDC_CLK,
		.a9_controlled	= 1,
	},
	{
		.name		= "vfe_clk",
		.owner		= THIS_MODULE,
		.id		= VFE_CLK,
		.a9_controlled	= 1,
	},
	{
		.name		= "vfe_mdc_clk",
		.owner		= THIS_MODULE,
		.id		= VFE_MDC_CLK,
		.a9_controlled	= 1,
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
