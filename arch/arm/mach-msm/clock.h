/* arch/arm/mach-msm/clock.h
 *
 * MSM architecture clock driver header
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2007 QUALCOMM Incorporated
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

#ifndef __ARCH_ARM_MACH_MSM_CLOCK_H
#define __ARCH_ARM_MACH_MSM_CLOCK_H

#include <linux/list.h>
/*
 * High level clock API support
 */
struct module;
#define CLKFLAG_INVERT   0x00000001 /* invert the clock */
#define CLKFLAG_NOINVERT 0x00000002 /* do not invert */
#define CLKFLAG_NONEST   0x00000004 /* disable nesting of enable/disable */
#define CLKFLAG_NORESET  0x00000008 /* Prohibit resetting clock */

#define CLK_FIRST_AVAILABLE_FLAG 0x00000100

struct clk
{
	struct list_head        list;
	struct clk		*parent;
	struct module           *owner;
	const char              *name;
	uint32_t                id;
	uint32_t                flags;
	int			a9_controlled;
};

#define A11S_CLK_CNTL_ADDR (MSM_CSR_BASE + 0x100)
#define A11S_CLK_SEL_ADDR (MSM_CSR_BASE + 0x104)
#define A11S_VDD_SVS_PLEVEL_ADDR (MSM_CSR_BASE + 0x124)

/*
 * Supported clocks
 */
#define ACPU_CLK           0  /* Applications processor clock */
#define ADM_CLK            1  /* Applications data mover clock */
#define ADSP_CLK           2  /* ADSP clock */
#define EBI1_CLK           3  /* External bus interface 1 clock */
#define EBI2_CLK           4  /* External bus interface 2 clock */
#define ECODEC_CLK         5  /* External CODEC clock */
#define EMDH_CLK           6  /* External MDDI host clock */
#define GP_CLK             7  /* General purpose clock */
#define GRP_CLK            8  /* Graphics clock */
#define I2C_CLK            9  /* I2C clock */
#define ICODEC_RX_CLK     10  /* Internal CODEX RX clock */
#define ICODEC_TX_CLK     11  /* Internal CODEX TX clock */
#define IMEM_CLK          12  /* Internal graphics memory clock */
#define MDC_CLK           13  /* MDDI client clock */
#define MDP_CLK           14  /* Mobile display processor clock */
#define PBUS_CLK          15  /* Peripheral bus clock */
#define PCM_CLK           16  /* PCM clock */
#define PMDH_CLK          17  /* Primary MDDI host clock */
#define SDAC_CLK          18  /* Stereo DAC clock */
#define SDC1_CLK          19  /* Secure Digital Card clocks */
#define SDC2_CLK          20
#define SDC3_CLK          21
#define SDC4_CLK          22
#define TSIF_CLK          23  /* Transport Stream Interface clocks */
#define TSIF_REF_CLK      24
#define TV_DAC_CLK        25  /* TV clocks */
#define TV_ENC_CLK        26
#define UART1_CLK         27  /* UART clocks */
#define UART2_CLK         28
#define UART3_CLK         29
#define UART1DM_CLK       30
#define UART2DM_CLK       31
#define USB_HS_CLK        32  /* High speed USB clock */
#define USB_OTG_CLK       33  /* Full speed USB clock */
#define VDC_CLK           34  /* Video controller clock */
#define VFE_CLK           35  /* Camera / Video Front End clock */
#define VFE_MDC_CLK       36  /* VFE MDDI client clock */

#define NR_CLKS           37


/*
 * Max supported clock rates
 */
#define MDH_CLK_MAX_FREQ    192000
#define VDC_CLK_MAX_FREQ     96000
#define EBI1_CLK_MAX_FREQ   128000
#define PBUS_CLK_MAX_FREQ    64000

/*
 * Supported ACPU clock speeds
 */
typedef enum
{
	CLKCTL_ACPU_SPEED_TCXO,
	CLKCTL_ACPU_SPEED_61P44_MHZ,
	CLKCTL_ACPU_SPEED_81P92_MHZ,
	CLKCTL_ACPU_SPEED_96_MHZ,
	CLKCTL_ACPU_SPEED_122P88_MHZ,
	CLKCTL_ACPU_SPEED_128_MHZ,
	CLKCTL_ACPU_SPEED_176_MHZ,
	CLKCTL_ACPU_SPEED_192_MHZ,
	CLKCTL_ACPU_SPEED_245P76_MHZ,
	CLKCTL_ACPU_SPEED_256_MHZ,
	CLKCTL_ACPU_SPEED_264_MHZ,
	CLKCTL_ACPU_SPEED_352_MHZ,
	CLKCTL_ACPU_SPEED_384_MHZ,
	CLKCTL_ACPU_SPEED_528_MHZ,

	CLKCTL_NUM_ACPU_SPEEDS
} clkctl_acpu_speed_t;


/*
 * Supported ACPU voltage levels
 */
typedef enum
{
	CLKCTL_ACPU_VDD_LEVEL_0,
	CLKCTL_ACPU_VDD_LEVEL_1,
	CLKCTL_ACPU_VDD_LEVEL_2,
	CLKCTL_ACPU_VDD_LEVEL_3,
	CLKCTL_ACPU_VDD_LEVEL_4,
	CLKCTL_ACPU_VDD_LEVEL_5,
	CLKCTL_ACPU_VDD_LEVEL_6,
	CLKCTL_ACPU_VDD_LEVEL_7,

	CLKCTL_NUM_ACPU_VDD_LEVELS
} clkctl_acpu_vdd_t;

/*
 * PLLs in the system
 */

typedef enum
{
	CLKCTL_TCXO,    /* TCXO @ 19.2 MHz   */
	CLKCTL_PLL0,    /* PLL @ 245.76 MHz  */
	CLKCTL_PLL1,    /* PLL @ 768 MHz     */
	CLKCTL_PLL2,    /* PLL @ 864 MHz     */
	CLKCTL_PLL3,    /* PLL @ 1056 MHz    */
	CLKCTL_NUM_SOURCES
} clkctl_source_t;

/*
 * ARM11 clock configuration for specific ACPU speeds
 */
typedef struct
{
	unsigned int     a11s_clk_khz;     /* A11S_CLK frequency in KHz  */
	clkctl_source_t  source;           /* Clock source               */
	unsigned int     a11s_clk_src_sel; /* A11S_CLK_CNTL:CLK_SRCx_SEL */
	unsigned int     a11s_clk_src_div; /* A11S_CLK_CNTL:CLK_SRCx_DIV */
	unsigned int     ahb_clk_khz;      /* AHB_CLK frequency in KHz   */
	unsigned int     ahb_clk_div;      /* A11S_CLK_SEL:AHB_CLK_DIV   */
} clkctl_acpu_clk_cfg_t;

/*
 * Combined speed/voltage pair
 */
typedef struct
{
	clkctl_acpu_speed_t  speed;
	clkctl_acpu_vdd_t    vdd;
} clkctl_acpu_perf_cfg_t;

typedef enum
{
	IDLE_STANDBY,    /* Regular standby mode, wakeup on interrupt */
	IDLE_SLEEP,      /* Sleep mode, wakeup only when ARM9 tells us */
	IDLE_POWERDOWN   /* Power collapse, wakeup only when ARM9 tells us */
} clkctl_idle_t;

/*
 * Supported ACPU perf levels.
 */
typedef enum
{
	CLKCTL_ACPU_PERF_LEVEL_0,
	CLKCTL_ACPU_PERF_LEVEL_1,
	CLKCTL_ACPU_PERF_LEVEL_2,
	CLKCTL_ACPU_PERF_LEVEL_3,
	CLKCTL_ACPU_PERF_LEVEL_4,
	CLKCTL_ACPU_PERF_LEVEL_5,

	CLKCTL_NUM_ACPU_PERF_LEVELS
} clkctl_acpu_perf_level_t;

#endif

