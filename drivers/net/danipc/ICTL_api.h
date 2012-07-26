/*
	All files except if stated otherwise in the begining of the file are under the ISC license:
	-----------------------------------------------------------------------------------

	Copyright (c) 2010-2012 Design Art Networks Ltd.

	Permission to use, copy, modify, and/or distribute this software for any
	purpose with or without fee is hereby granted, provided that the above
	copyright notice and this permission notice appear in all copies.

	THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
	WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
	MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
	ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
	WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
	ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
	OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
*/



#ifndef __ICTL_API_H__
#define __ICTL_API_H__
/*
 * -----------------------------------------------------------
 * Include section
 * -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------
 * MACRO (define) section
 * -----------------------------------------------------------
 */
#define MAX_LOCAL_INTS		(32)

/*
 * -----------------------------------------------------------
 * Type definition section
 * -----------------------------------------------------------
 */

 /* This enum holds all the system interrupts. use it when binding a local interrupt to system interrupt */
 typedef enum
 {
    E_SYS_INT_TU_WDT_INT = 0,                           /*   0 */	                // From timing_unit_i of timing_unit.v
    E_SYS_INT_TU_SPI_SSI_INTR_0,                    /*   1 */	                   // From timing_unit_i of timing_unit.v
    E_SYS_INT_TU_SPI_SSI_INTR_1,                    /*   2 */	                   // From timing_unit_i of timing_unit.v
    E_SYS_INT_TU_SPI_SSI_INTR_2,                    /*   3 */	                   // From timing_unit_i of timing_unit.v
    E_SYS_INT_TU_SPI_SSI_INTR_3,                    /*   4 */	                   // From timing_unit_i of timing_unit.v
    E_SYS_INT_TU_SELF_INT,                          /*   5 */	                // From timing_unit_i of timing_unit.v
    E_SYS_INT_TU_PINT2_INT,                         /*   6 */	                // From timing_unit_i of timing_unit.v
    E_SYS_INT_TU_PINT1_INT,                         /*   7 */	                // From timing_unit_i of timing_unit.v
    E_SYS_INT_TU_PINT0_INT,                         /*   8 */	                // From timing_unit_i of timing_unit.v
    E_SYS_INT_TU_FS_INT,                            /*   9 */	                // From timing_unit_i of timing_unit.v
    E_SYS_INT_TU_1PPS_1588_INT,                     /*  10 */	                // From timing_unit_i of timing_unit.v
    E_SYS_INT_GPIO2_INTRCLK_EN,                     /*  11 */	                // From apb_gpio_2 of apb_gpio_ip_DW_apb_gpio.v
    E_SYS_INT_GPIO2_INTR_FLAG,                      /*  12 */	                // From apb_gpio_2 of apb_gpio_ip_DW_apb_gpio.v
    E_SYS_INT_GPIO1_INTRCLK_EN,                     /*  13 */	                // From apb_gpio_1 of apb_gpio_ip_DW_apb_gpio.v
    E_SYS_INT_GPIO1_INTR_FLAG,                      /*  14 */	                // From apb_gpio_1 of apb_gpio_ip_DW_apb_gpio.v
    E_SYS_INT_GPIO0_INTRCLK_EN,                     /*  15 */	                // From apb_gpio_0 of apb_gpio_ip_DW_apb_gpio.v
    E_SYS_INT_GPIO0_INTR_FLAG,                      /*  16 */	                // From apb_gpio_0 of apb_gpio_ip_DW_apb_gpio.v
    E_SYS_INT_DDR1_ERR,                             /*  17 */	                // From ddr_wrap_1 of ddr3_wrap.v
    E_SYS_INT_DDR1_CAL_DONE,                        /*  18 */	                       // From ddr_wrap_1 of ddr3_wrap.v
    E_SYS_INT_DDR0_PLL_LOCK,                        /*  19 */	                       // From ddr_wrap_0 of ddr3_wrap.v
    E_SYS_INT_DDR0_INIT,                            /*  20 */	                       // From ddr_wrap_0 of ddr3_wrap.v
    E_SYS_INT_DDR0_ERR,                             /*  21 */	                // From ddr_wrap_0 of ddr3_wrap.v
    E_SYS_INT_DDR0_CAL_DONE,                        /*  22 */	                       // From ddr_wrap_0 of ddr3_wrap.v
    E_SYS_INT_TU_CDU_IRQ0_OUT,                      /*  23 */	                // From timing_unit_i of timing_unit.v
    E_SYS_INT_SRIO3_IRQ1_OUT,                       /*  24 */	                // From dan_srio_top_3 of dan_srio_top.v
    E_SYS_INT_SRIO3_IRQ0_OUT,                       /*  25 */	                // From dan_srio_top_3 of dan_srio_top.v
    E_SYS_INT_SRIO2_IRQ1_OUT,                       /*  26 */	                // From dan_srio_top_2 of dan_srio_top.v
    E_SYS_INT_SRIO2_IRQ0_OUT,                       /*  27 */	                // From dan_srio_top_2 of dan_srio_top.v
    E_SYS_INT_SRIO1_IRQ1_OUT,                       /*  28 */	                // From dan_srio_top_1 of dan_srio_top.v
    E_SYS_INT_SRIO1_IRQ0_OUT,                       /*  29 */	                // From dan_srio_top_1 of dan_srio_top.v
    E_SYS_INT_SRIO0_IRQ1_OUT,                       /*  30 */	                // From dan_srio_top_0 of dan_srio_top.v
    E_SYS_INT_SRIO0_IRQ0_OUT,                       /*  31 */	                // From dan_srio_top_0 of dan_srio_top.v
    E_SYS_INT_WDT0_INTR,                            /*  32 */	                // From npu_top_i of npu_top.v
    E_SYS_INT_TIMER3_0,                             /*  33 */	                   // From npu_top_i of npu_top.v
    E_SYS_INT_TIMER3_1,                       /*  34 */	                   // From npu_top_i of npu_top.v
    E_SYS_INT_TIMER3_2,                       /*  35 */	                   // From npu_top_i of npu_top.v
    E_SYS_INT_TIMER3_3,                       /*  36 */	                   // From npu_top_i of npu_top.v
    E_SYS_INT_TIMER3_4,                       /*  37 */	                   // From npu_top_i of npu_top.v
    E_SYS_INT_TIMER3_5,                       /*  38 */	                   // From npu_top_i of npu_top.v
    E_SYS_INT_TIMER3_6,                       /*  39 */	                   // From npu_top_i of npu_top.v
    E_SYS_INT_TIMER3_7,                       /*  40 */	                   // From npu_top_i of npu_top.v
    E_SYS_INT_TIMER2_0,                       /*  41 */	                   // From npu_top_i of npu_top.v
    E_SYS_INT_TIMER2_1,                       /*  42 */	                   // From npu_top_i of npu_top.v
    E_SYS_INT_TIMER2_2,                       /*  43 */	                   // From npu_top_i of npu_top.v
    E_SYS_INT_TIMER2_3,                       /*  44 */	                   // From npu_top_i of npu_top.v
    E_SYS_INT_TIMER2_4,                       /*  45 */	                   // From npu_top_i of npu_top.v
    E_SYS_INT_TIMER2_5,                       /*  46 */	                   // From npu_top_i of npu_top.v
    E_SYS_INT_TIMER2_6,                       /*  47 */	                   // From npu_top_i of npu_top.v
    E_SYS_INT_TIMER2_7,                       /*  48 */	                   // From npu_top_i of npu_top.v
    E_SYS_INT_TIMER1_0,                       /*  49 */	                   // From npu_top_i of npu_top.v
    E_SYS_INT_TIMER1_1,                       /*  50 */	                   // From npu_top_i of npu_top.v
    E_SYS_INT_TIMER1_2,                       /*  51 */	                   // From npu_top_i of npu_top.v
    E_SYS_INT_TIMER1_3,                       /*  52 */	                   // From npu_top_i of npu_top.v
    E_SYS_INT_TIMER1_4,                       /*  53 */	                   // From npu_top_i of npu_top.v
    E_SYS_INT_TIMER1_5,                       /*  54 */	                   // From npu_top_i of npu_top.v
    E_SYS_INT_TIMER1_6,                       /*  55 */	                   // From npu_top_i of npu_top.v
    E_SYS_INT_TIMER1_7,                       /*  56 */	                   // From npu_top_i of npu_top.v
    E_SYS_INT_TIMER0_0,                       /*  57 */	                   // From npu_top_i of npu_top.v
    E_SYS_INT_TIMER0_1,                       /*  58 */	                   // From npu_top_i of npu_top.v
    E_SYS_INT_TIMER0_2,                       /*  59 */	                   // From npu_top_i of npu_top.v
    E_SYS_INT_TIMER0_3,                       /*  60 */	                   // From npu_top_i of npu_top.v
    E_SYS_INT_TIMER0_4,                       /*  61 */	                   // From npu_top_i of npu_top.v
    E_SYS_INT_TIMER0_5,                       /*  62 */	                   // From npu_top_i of npu_top.v
    E_SYS_INT_TIMER0_6,                       /*  63 */	                   // From npu_top_i of npu_top.v
    E_SYS_INT_TIMER0_7,                       /*  64 */	                   // From npu_top_i of npu_top.v
    E_SYS_INT_NPU_UART,                   /*  65 */	                // From npu_top_i of npu_top.v
    E_SYS_INT_NPU_SW_0,                  /*  66 */	                   // From npu_top_i of npu_top.v
    E_SYS_INT_NPU_SW_1,                  /*  67 */	                   // From npu_top_i of npu_top.v
    E_SYS_INT_NPU_SW_2,                  /*  68 */	                   // From npu_top_i of npu_top.v
    E_SYS_INT_NPU_SW_3,                  /*  69 */	                   // From npu_top_i of npu_top.v
    E_SYS_INT_NPU_SW_4,                  /*  70 */	                   // From npu_top_i of npu_top.v
    E_SYS_INT_NPU_SW_5,                  /*  71 */	                   // From npu_top_i of npu_top.v
    E_SYS_INT_NPU_SW_6,                  /*  72 */	                   // From npu_top_i of npu_top.v
    E_SYS_INT_NPU_SW_7,                  /*  73 */	                   // From npu_top_i of npu_top.v
    E_SYS_INT_NPU_SW_8,                  /*  74 */	                   // From npu_top_i of npu_top.v
    E_SYS_INT_NPU_SW_9,                  /*  75 */	                   // From npu_top_i of npu_top.v
    E_SYS_INT_NPU_SW_10,                 /*  76 */	                    // From npu_top_i of npu_top.v
    E_SYS_INT_NPU_SW_11,                 /*  77 */	                    // From npu_top_i of npu_top.v
    E_SYS_INT_NPU_SW_12,                 /*  78 */	                    // From npu_top_i of npu_top.v
    E_SYS_INT_NPU_SW_13,                 /*  79 */	                    // From npu_top_i of npu_top.v
    E_SYS_INT_NPU_SW_14,                 /*  80 */	                    // From npu_top_i of npu_top.v
    E_SYS_INT_NPU_SW_15,                 /*  81 */	                    // From npu_top_i of npu_top.v
    E_SYS_INT_NPU_SW_16,                 /*  82 */	                    // From npu_top_i of npu_top.v
    E_SYS_INT_NPU_SW_17,                 /*  83 */	                    // From npu_top_i of npu_top.v
    E_SYS_INT_NPU_SW_18,                 /*  84 */	                    // From npu_top_i of npu_top.v
    E_SYS_INT_NPU_SW_19,                 /*  85 */	                    // From npu_top_i of npu_top.v
    E_SYS_INT_NPU_SW_20,                 /*  86 */	                    // From npu_top_i of npu_top.v
    E_SYS_INT_NPU_SW_21,                 /*  87 */	                    // From npu_top_i of npu_top.v
    E_SYS_INT_NPU_SW_22,                 /*  88 */	                    // From npu_top_i of npu_top.v
    E_SYS_INT_NPU_SW_23,                 /*  89 */	                    // From npu_top_i of npu_top.v
    E_SYS_INT_NPU_SW_24,                 /*  90 */	                    // From npu_top_i of npu_top.v
    E_SYS_INT_NPU_SW_25,                 /*  91 */	                    // From npu_top_i of npu_top.v
    E_SYS_INT_NPU_SW_26,                 /*  92 */	                    // From npu_top_i of npu_top.v
    E_SYS_INT_NPU_SW_27,                 /*  93 */	                    // From npu_top_i of npu_top.v
    E_SYS_INT_NPU_SW_28,                 /*  94 */	                    // From npu_top_i of npu_top.v
    E_SYS_INT_NPU_SW_29,                 /*  95 */	                    // From npu_top_i of npu_top.v
    E_SYS_INT_NPU_SW_30,                 /*  96 */	                    // From npu_top_i of npu_top.v
    E_SYS_INT_NPU_SW_31,                 /*  97 */	                    // From npu_top_i of npu_top.v
    E_SYS_INT_NPU_SPI,                    /*  98 */	                // From npu_top_i of npu_top.v
    E_SYS_INT_NPU_SEMAPHORE1_0,          /*  99 */	                    // From npu_top_i of npu_top.v
    E_SYS_INT_NPU_SEMAPHORE1_1,          /* 100 */	                    // From npu_top_i of npu_top.v
    E_SYS_INT_NPU_SEMAPHORE1_2,          /* 101 */	                    // From npu_top_i of npu_top.v
    E_SYS_INT_NPU_SEMAPHORE1_3,          /* 102 */	                    // From npu_top_i of npu_top.v
    E_SYS_INT_NPU_SEMAPHORE1_4,          /* 103 */	                    // From npu_top_i of npu_top.v
    E_SYS_INT_NPU_SEMAPHORE1_5,          /* 104 */	                    // From npu_top_i of npu_top.v
    E_SYS_INT_NPU_SEMAPHORE1_6,          /* 105 */	                    // From npu_top_i of npu_top.v
    E_SYS_INT_NPU_SEMAPHORE1_7,          /* 106 */	                    // From npu_top_i of npu_top.v
    E_SYS_INT_NPU_SEMAPHORE0_0,          /* 107 */	                    // From npu_top_i of npu_top.v
    E_SYS_INT_NPU_SEMAPHORE0_1,          /* 108 */	                    // From npu_top_i of npu_top.v
    E_SYS_INT_NPU_SEMAPHORE0_2,          /* 109 */	                    // From npu_top_i of npu_top.v
    E_SYS_INT_NPU_SEMAPHORE0_3,          /* 110 */	                    // From npu_top_i of npu_top.v
    E_SYS_INT_NPU_SEMAPHORE0_4,          /* 111 */	                    // From npu_top_i of npu_top.v
    E_SYS_INT_NPU_SEMAPHORE0_5,          /* 112 */	                    // From npu_top_i of npu_top.v
    E_SYS_INT_NPU_SEMAPHORE0_6,          /* 113 */	                    // From npu_top_i of npu_top.v
    E_SYS_INT_NPU_SEMAPHORE0_7,          /* 114 */	                    // From npu_top_i of npu_top.v
    E_SYS_INT_SPARE_0,                   /* 115 */
    E_SYS_INT_SPARE_1,                   /* 116 */
    E_SYS_INT_SPARE_2,                   /* 117 */
    E_SYS_INT_SPARE_3,                   /* 118 */
    E_SYS_INT_SPARE_4,                   /* 119 */
    E_SYS_INT_SPARE_5,                   /* 120 */
    E_SYS_INT_SPARE_6,                   /* 121 */
    E_SYS_INT_SPARE_7,                   /* 122 */
    E_SYS_INT_SPARE_8,                   /* 123 */
    E_SYS_INT_SPARE_9,                   /* 124 */
    E_SYS_INT_SPARE_10,                  /* 125 */
    E_SYS_INT_SPARE_11,                  /* 126 */
    E_SYS_INT_SPARE_12,                  /* 127 */
    E_SYS_INT_WSC_INT,                   /* 128 */
    E_SYS_INT_NPU_SECURITY1_INT,         /* 129 */
    E_SYS_INT_NPU_SECURITY0_INT,         /* 130 */
    E_SYS_INT_NPU_GP_DMA1_0,             /* 131 */	                   // From npu_top_i of npu_top.v
    E_SYS_INT_NPU_GP_DMA1_1,             /* 132 */	                   // From npu_top_i of npu_top.v
    E_SYS_INT_NPU_GP_DMA1_2,             /* 133 */	                   // From npu_top_i of npu_top.v
    E_SYS_INT_NPU_GP_DMA1_3,             /* 134 */	                   // From npu_top_i of npu_top.v
    E_SYS_INT_NPU_GP_DMA1_4,             /* 135 */	                   // From npu_top_i of npu_top.v
    E_SYS_INT_NPU_GP_DMA1_5,             /* 136 */	                   // From npu_top_i of npu_top.v
    E_SYS_INT_NPU_GP_DMA1_6,             /* 137 */	                   // From npu_top_i of npu_top.v
    E_SYS_INT_NPU_GP_DMA1_7,             /* 138 */	                   // From npu_top_i of npu_top.v
    E_SYS_INT_NPU_GP_DMA0_0,             /* 139 */	                   // From npu_top_i of npu_top.v
    E_SYS_INT_NPU_GP_DMA0_1,             /* 140 */	                   // From npu_top_i of npu_top.v
    E_SYS_INT_NPU_GP_DMA0_2,             /* 141 */	                   // From npu_top_i of npu_top.v
    E_SYS_INT_NPU_GP_DMA0_3,             /* 142 */	                   // From npu_top_i of npu_top.v
    E_SYS_INT_NPU_GP_DMA0_4,             /* 143 */	                   // From npu_top_i of npu_top.v
    E_SYS_INT_NPU_GP_DMA0_5,             /* 144 */	                   // From npu_top_i of npu_top.v
    E_SYS_INT_NPU_GP_DMA0_6,             /* 145 */	                   // From npu_top_i of npu_top.v
    E_SYS_INT_NPU_GP_DMA0_7,             /* 146 */	                   // From npu_top_i of npu_top.v
    E_SYS_INT_NPU_GMAC1_SBD,             /* 147 */	                // From npu_top_i of npu_top.v
    E_SYS_INT_NPU_GMAC1_PMT,             /* 148 */	                // From npu_top_i of npu_top.v
    E_SYS_INT_NPU_GMAC1_LPI,             /* 149 */	                // From npu_top_i of npu_top.v
    E_SYS_INT_NPU_GMAC0_SBD,             /* 150 */	                // From npu_top_i of npu_top.v
    E_SYS_INT_NPU_GMAC0_PMT,             /* 151 */	                // From npu_top_i of npu_top.v
    E_SYS_INT_NPU_GMAC0_LPI,             /* 152 */	                // From npu_top_i of npu_top.v
    E_SYS_INT_NPU_DAN_IPC5_CDU_IRQ1,                /* 153 */	                       // From npu_top_i of npu_top.v
    E_SYS_INT_NPU_DAN_IPC5_CDU_IRQ0,                /* 154 */	                       // From npu_top_i of npu_top.v
    E_SYS_INT_NPU_DAN_IPC4_CDU_IRQ1,                /* 155 */	                // From npu_top_i of npu_top.v
    E_SYS_INT_NPU_DAN_IPC4_CDU_IRQ0,                /* 156 */	                // From npu_top_i of npu_top.v
    E_SYS_INT_NPU_DAN_IPC3_CDU_IRQ1,                /* 157 */	                // From npu_top_i of npu_top.v
    E_SYS_INT_NPU_DAN_IPC3_CDU_IRQ0,                /* 158 */	                // From npu_top_i of npu_top.v
    E_SYS_INT_NPU_DAN_IPC2_CDU_IRQ1,                /* 159 */	                // From npu_top_i of npu_top.v
    E_SYS_INT_NPU_DAN_IPC2_CDU_IRQ0,                /* 160 */	                // From npu_top_i of npu_top.v
    E_SYS_INT_NPU_DAN_IPC1_CDU_IRQ1,                /* 161 */	                // From npu_top_i of npu_top.v
    E_SYS_INT_NPU_DAN_IPC1_CDU_IRQ0,                /* 162 */	                // From npu_top_i of npu_top.v
    E_SYS_INT_NPU_DAN_IPC0_CDU_IRQ1,                /* 163 */	                // From npu_top_i of npu_top.v
    E_SYS_INT_NPU_DAN_IPC0_CDU_IRQ0,                /* 164 */	                // From npu_top_i of npu_top.v
    E_SYS_INT_FMSH_WR_DAN_DMA_CDU_IRQ1_OUT_0,      /* 165 */	                        // From npu_top_i of npu_top.v
    E_SYS_INT_FMSH_WR_DAN_DMA_CDU_IRQ1_OUT_1,      /* 166 */	                        // From npu_top_i of npu_top.v
    E_SYS_INT_FMSH_RD_DAN_DMA_CDU_IRQ1_OUT_0,      /* 167 */	                        // From npu_top_i of npu_top.v
    E_SYS_INT_FMSH_RD_DAN_DMA_CDU_IRQ1_OUT_1,      /* 168 */	                        // From npu_top_i of npu_top.v
    E_SYS_INT_TXF_RD_DAN_DMA_CDU_IRQ1_OUT,          /* 169 */	                    // From npu_top_i of npu_top.v
    E_SYS_INT_TXF_CDU_IRQ1_OUT,                     /* 170 */	                // From npu_top_i of npu_top.v
    E_SYS_INT_CPU_SS7_CDU_IRQ1_OUT,                 /* 171 */	                         // From npu_top_i of npu_top.v
    E_SYS_INT_CPU_SS7_CDU_IRQ0_OUT,                 /* 172 */	                         // From npu_top_i of npu_top.v
    E_SYS_INT_CPU_SS6_CDU_IRQ1_OUT,                 /* 173 */	                         // From npu_top_i of npu_top.v
    E_SYS_INT_CPU_SS6_CDU_IRQ0_OUT,                 /* 174 */	                         // From npu_top_i of npu_top.v
    E_SYS_INT_CPU_SS5_CDU_IRQ1_OUT,                 /* 175 */	                // From npu_top_i of npu_top.v
    E_SYS_INT_CPU_SS5_CDU_IRQ0_OUT,                 /* 176 */	                // From npu_top_i of npu_top.v
    E_SYS_INT_CPU_SS4_CDU_IRQ1_OUT,                 /* 177 */	                // From npu_top_i of npu_top.v
    E_SYS_INT_CPU_SS4_CDU_IRQ0_OUT,                 /* 178 */	                // From npu_top_i of npu_top.v
    E_SYS_INT_CPU_SS3_CDU_IRQ1_OUT,                 /* 179 */	                // From npu_top_i of npu_top.v
    E_SYS_INT_CPU_SS3_CDU_IRQ0_OUT,                 /* 180 */	                // From npu_top_i of npu_top.v
    E_SYS_INT_CPU_SS2_CDU_IRQ1_OUT,                 /* 181 */	                // From npu_top_i of npu_top.v
    E_SYS_INT_CPU_SS2_CDU_IRQ0_OUT,                 /* 182 */	                // From npu_top_i of npu_top.v
    E_SYS_INT_CPU_SS1_CDU_IRQ1_OUT,                 /* 183 */	                // From npu_top_i of npu_top.v
    E_SYS_INT_CPU_SS1_CDU_IRQ0_OUT,                 /* 184 */	                // From npu_top_i of npu_top.v
    E_SYS_INT_CPU_SS0_CDU_IRQ1_OUT,                 /* 185 */	                // From npu_top_i of npu_top.v
    E_SYS_INT_CPU_SS0_CDU_IRQ0_OUT,                 /* 186 */	                // From npu_top_i of npu_top.v
    E_SYS_INT_CPU7_WR_DAN_DMA_CDU_IRQ1_OUT,         /* 187 */	                              // From npu_top_i of npu_top.v
    E_SYS_INT_CPU7_RD_DAN_DMA_CDU_IRQ1_OUT,         /* 188 */	                              // From npu_top_i of npu_top.v
    E_SYS_INT_CPU6_WR_DAN_DMA_CDU_IRQ1_OUT,         /* 189 */	                              // From npu_top_i of npu_top.v
    E_SYS_INT_CPU6_RD_DAN_DMA_CDU_IRQ1_OUT,         /* 190 */	                              // From npu_top_i of npu_top.v
    E_SYS_INT_CPU5_WR_DAN_DMA_CDU_IRQ1_OUT,         /* 191 */	                     // From npu_top_i of npu_top.v
    E_SYS_INT_CPU5_RD_DAN_DMA_CDU_IRQ1_OUT,         /* 192 */	                     // From npu_top_i of npu_top.v
    E_SYS_INT_CPU4_WR_DAN_DMA_CDU_IRQ1_OUT,         /* 193 */	                     // From npu_top_i of npu_top.v
    E_SYS_INT_CPU4_RD_DAN_DMA_CDU_IRQ1_OUT,         /* 194 */	                     // From npu_top_i of npu_top.v
    E_SYS_INT_CPU3_WR_DAN_DMA_CDU_IRQ1_OUT,         /* 195 */	                     // From npu_top_i of npu_top.v
    E_SYS_INT_CPU3_RD_DAN_DMA_CDU_IRQ1_OUT,         /* 196 */	                     // From npu_top_i of npu_top.v
    E_SYS_INT_CPU2_WR_DAN_DMA_CDU_IRQ1_OUT,         /* 197 */	                     // From npu_top_i of npu_top.v
    E_SYS_INT_CPU2_RD_DAN_DMA_CDU_IRQ1_OUT,         /* 198 */	                     // From npu_top_i of npu_top.v
    E_SYS_INT_CPU1_WR_DAN_DMA_CDU_IRQ1_OUT,         /* 199 */	                     // From npu_top_i of npu_top.v
    E_SYS_INT_CPU1_RD_DAN_DMA_CDU_IRQ1_OUT,         /* 200 */	                     // From npu_top_i of npu_top.v
    E_SYS_INT_CPU0_WR_DAN_DMA_CDU_IRQ1_OUT,         /* 201 */	                     // From npu_top_i of npu_top.v
    E_SYS_INT_CPU0_RD_DAN_DMA_CDU_IRQ1_OUT,         /* 202 */	                     // From npu_top_i of npu_top.v
    E_SYS_INT_CB_CDU_IRQ0_OUT,                      /* 203 */	                // From npu_top_i of npu_top.v
    E_SYS_INT_TBUS_STORAGE_CDU_IRQ1_OUT,            /* 204 */	                         // From phy_fe_top_i of phy_fe_top.v
    E_SYS_INT_TBUS_STORAGE_CDU_IRQ0_OUT,            /* 205 */	                  // From phy_fe_top_i of phy_fe_top.v
    E_SYS_INT_PHY_FE_FIFO_CDU_IRQ1_OUT,             /* 206 */	                 // From phy_fe_top_i of phy_fe_top.v
    E_SYS_INT_PHY_FE_FIFO_CDU_IRQ0_OUT,             /* 207 */	                 // From phy_fe_top_i of phy_fe_top.v
    E_SYS_INT_FE_TX_PRI3_CDU_IRQ1_OUT,              /* 208 */	                // From phy_fe_top_i of phy_fe_top.v
    E_SYS_INT_FE_TX_PRI3_CDU_IRQ0_OUT,              /* 209 */	                // From phy_fe_top_i of phy_fe_top.v
    E_SYS_INT_FE_TX_PRI2_CDU_IRQ1_OUT,              /* 210 */	                // From phy_fe_top_i of phy_fe_top.v
    E_SYS_INT_FE_TX_PRI2_CDU_IRQ0_OUT,              /* 211 */	                // From phy_fe_top_i of phy_fe_top.v
    E_SYS_INT_FE_TX_PRI1_CDU_IRQ1_OUT,              /* 212 */	                // From phy_fe_top_i of phy_fe_top.v
    E_SYS_INT_FE_TX_PRI1_CDU_IRQ0_OUT,              /* 213 */	                // From phy_fe_top_i of phy_fe_top.v
    E_SYS_INT_FE_TX_PRI0_CDU_IRQ1_OUT,              /* 214 */	                // From phy_fe_top_i of phy_fe_top.v
    E_SYS_INT_FE_TX_PRI0_CDU_IRQ0_OUT,              /* 215 */	                // From phy_fe_top_i of phy_fe_top.v
    E_SYS_INT_FE_TX_FIFO_WR_DAN_DMA9_CDU_IRQ1_OUT,  /* 216 */	                            // From phy_fe_top_i of phy_fe_top.v
    E_SYS_INT_FE_TX_FIFO_WR_DAN_DMA8_CDU_IRQ1_OUT,  /* 217 */	                            // From phy_fe_top_i of phy_fe_top.v
    E_SYS_INT_FE_TX_FIFO_WR_DAN_DMA7_CDU_IRQ1_OUT,  /* 218 */	                            // From phy_fe_top_i of phy_fe_top.v
    E_SYS_INT_FE_TX_FIFO_WR_DAN_DMA6_CDU_IRQ1_OUT,  /* 219 */	                            // From phy_fe_top_i of phy_fe_top.v
    E_SYS_INT_FE_TX_FIFO_WR_DAN_DMA5_CDU_IRQ1_OUT,  /* 220 */	                            // From phy_fe_top_i of phy_fe_top.v
    E_SYS_INT_FE_TX_FIFO_WR_DAN_DMA4_CDU_IRQ1_OUT,  /* 221 */	                            // From phy_fe_top_i of phy_fe_top.v
    E_SYS_INT_FE_TX_FIFO_WR_DAN_DMA3_CDU_IRQ1_OUT,  /* 222 */	                            // From phy_fe_top_i of phy_fe_top.v
    E_SYS_INT_FE_TX_FIFO_WR_DAN_DMA2_CDU_IRQ1_OUT,  /* 223 */	                            // From phy_fe_top_i of phy_fe_top.v
    E_SYS_INT_FE_TX_FIFO_WR_DAN_DMA1_CDU_IRQ1_OUT,  /* 224 */	                            // From phy_fe_top_i of phy_fe_top.v
    E_SYS_INT_FE_TX_FIFO_WR_DAN_DMA15_CDU_IRQ1_OUT, /* 225 */	                             // From phy_fe_top_i of phy_fe_top.v
    E_SYS_INT_FE_TX_FIFO_WR_DAN_DMA14_CDU_IRQ1_OUT, /* 226 */	                             // From phy_fe_top_i of phy_fe_top.v
    E_SYS_INT_FE_TX_FIFO_WR_DAN_DMA13_CDU_IRQ1_OUT, /* 227 */	                             // From phy_fe_top_i of phy_fe_top.v
    E_SYS_INT_FE_TX_FIFO_WR_DAN_DMA12_CDU_IRQ1_OUT, /* 228 */	                             // From phy_fe_top_i of phy_fe_top.v
    E_SYS_INT_FE_TX_FIFO_WR_DAN_DMA11_CDU_IRQ1_OUT, /* 229 */	                             // From phy_fe_top_i of phy_fe_top.v
    E_SYS_INT_FE_TX_FIFO_WR_DAN_DMA10_CDU_IRQ1_OUT, /* 230 */	                             // From phy_fe_top_i of phy_fe_top.v
    E_SYS_INT_FE_TX_FIFO_WR_DAN_DMA0_CDU_IRQ1_OUT,  /* 231 */	                            // From phy_fe_top_i of phy_fe_top.v
    E_SYS_INT_FE_TX_DAN_DMA3_CDU_IRQ_OUT,           /* 232 */	                   // From phy_fe_top_i of phy_fe_top.v
    E_SYS_INT_FE_TX_DAN_DMA2_CDU_IRQ_OUT,           /* 233 */	                   // From phy_fe_top_i of phy_fe_top.v
    E_SYS_INT_FE_TX_DAN_DMA1_CDU_IRQ_OUT,           /* 234 */	                   // From phy_fe_top_i of phy_fe_top.v
    E_SYS_INT_FE_TX_DAN_DMA0_CDU_IRQ_OUT,           /* 235 */	                   // From phy_fe_top_i of phy_fe_top.v
    E_SYS_INT_FE_TX_CDU_IRQ1_OUT,                   /* 236 */	                // From phy_fe_top_i of phy_fe_top.v
    E_SYS_INT_FE_TX_CDU_IRQ0_OUT,                   /* 237 */	                // From phy_fe_top_i of phy_fe_top.v
    E_SYS_INT_FE_RX_WR_DAN_DMA9_CDU_IRQ_OUT,        /* 238 */	                      // From phy_fe_top_i of phy_fe_top.v
    E_SYS_INT_FE_RX_WR_DAN_DMA8_CDU_IRQ_OUT,        /* 239 */	                      // From phy_fe_top_i of phy_fe_top.v
    E_SYS_INT_FE_RX_WR_DAN_DMA7_CDU_IRQ_OUT,        /* 240 */	                      // From phy_fe_top_i of phy_fe_top.v
    E_SYS_INT_FE_RX_WR_DAN_DMA6_CDU_IRQ_OUT,        /* 241 */	                      // From phy_fe_top_i of phy_fe_top.v
    E_SYS_INT_FE_RX_WR_DAN_DMA5_CDU_IRQ_OUT,        /* 242 */	                      // From phy_fe_top_i of phy_fe_top.v
    E_SYS_INT_FE_RX_WR_DAN_DMA4_CDU_IRQ_OUT,        /* 243 */	                      // From phy_fe_top_i of phy_fe_top.v
    E_SYS_INT_FE_RX_WR_DAN_DMA3_CDU_IRQ_OUT,        /* 244 */	                      // From phy_fe_top_i of phy_fe_top.v
    E_SYS_INT_FE_RX_WR_DAN_DMA2_CDU_IRQ_OUT,        /* 245 */	                      // From phy_fe_top_i of phy_fe_top.v
    E_SYS_INT_FE_RX_WR_DAN_DMA1_CDU_IRQ_OUT,        /* 246 */	                      // From phy_fe_top_i of phy_fe_top.v
    E_SYS_INT_FE_RX_WR_DAN_DMA15_CDU_IRQ_OUT,       /* 247 */	                       // From phy_fe_top_i of phy_fe_top.v
    E_SYS_INT_FE_RX_WR_DAN_DMA14_CDU_IRQ_OUT,       /* 248 */	                       // From phy_fe_top_i of phy_fe_top.v
    E_SYS_INT_FE_RX_WR_DAN_DMA13_CDU_IRQ_OUT,       /* 249 */	                       // From phy_fe_top_i of phy_fe_top.v
    E_SYS_INT_FE_RX_WR_DAN_DMA12_CDU_IRQ_OUT,       /* 250 */	                       // From phy_fe_top_i of phy_fe_top.v
    E_SYS_INT_FE_RX_WR_DAN_DMA11_CDU_IRQ_OUT,       /* 251 */	                       // From phy_fe_top_i of phy_fe_top.v
    E_SYS_INT_FE_RX_WR_DAN_DMA10_CDU_IRQ_OUT,       /* 252 */	                       // From phy_fe_top_i of phy_fe_top.v
    E_SYS_INT_FE_RX_WR_DAN_DMA0_CDU_IRQ_OUT,        /* 253 */	                      // From phy_fe_top_i of phy_fe_top.v
    E_SYS_INT_FE_RX_PRI4_WR1_DAN_DMA_CDU_IRQ1_OUT,  /* 254 */	                            // From phy_fe_top_i of phy_fe_top.v
    E_SYS_INT_FE_RX_PRI4_WR0_DAN_DMA_CDU_IRQ1_OUT,  /* 255 */	                            // From phy_fe_top_i of phy_fe_top.v
    E_SYS_INT_FE_RX_PRI4_CDU_IRQ1_OUT,              /* 256 */	                // From phy_fe_top_i of phy_fe_top.v
    E_SYS_INT_FE_RX_PRI4_CDU_IRQ0_OUT,              /* 257 */	                // From phy_fe_top_i of phy_fe_top.v
    E_SYS_INT_FE_RX_PRI3_WR1_DAN_DMA_CDU_IRQ1_OUT,  /* 258 */	                            // From phy_fe_top_i of phy_fe_top.v
    E_SYS_INT_FE_RX_PRI3_WR0_DAN_DMA_CDU_IRQ1_OUT,  /* 259 */	                            // From phy_fe_top_i of phy_fe_top.v
    E_SYS_INT_FE_RX_PRI3_CDU_IRQ1_OUT,              /* 260 */	                // From phy_fe_top_i of phy_fe_top.v
    E_SYS_INT_FE_RX_PRI3_CDU_IRQ0_OUT,              /* 261 */	                // From phy_fe_top_i of phy_fe_top.v
    E_SYS_INT_FE_RX_PRI2_WR1_DAN_DMA_CDU_IRQ1_OUT,  /* 262 */	                            // From phy_fe_top_i of phy_fe_top.v
    E_SYS_INT_FE_RX_PRI2_WR0_DAN_DMA_CDU_IRQ1_OUT,  /* 263 */	                            // From phy_fe_top_i of phy_fe_top.v
    E_SYS_INT_FE_RX_PRI2_CDU_IRQ1_OUT,              /* 264 */	                // From phy_fe_top_i of phy_fe_top.v
    E_SYS_INT_FE_RX_PRI2_CDU_IRQ0_OUT,              /* 265 */	                // From phy_fe_top_i of phy_fe_top.v
    E_SYS_INT_FE_RX_PRI1_WR1_DAN_DMA_CDU_IRQ1_OUT,  /* 266 */	                            // From phy_fe_top_i of phy_fe_top.v
    E_SYS_INT_FE_RX_PRI1_WR0_DAN_DMA_CDU_IRQ1_OUT,  /* 267 */	                            // From phy_fe_top_i of phy_fe_top.v
    E_SYS_INT_FE_RX_PRI1_CDU_IRQ1_OUT,              /* 268 */	                // From phy_fe_top_i of phy_fe_top.v
    E_SYS_INT_FE_RX_PRI1_CDU_IRQ0_OUT,              /* 269 */	                // From phy_fe_top_i of phy_fe_top.v
    E_SYS_INT_FE_RX_PRI0_WR1_DAN_DMA_CDU_IRQ1_OUT,  /* 270 */	                            // From phy_fe_top_i of phy_fe_top.v
    E_SYS_INT_FE_RX_PRI0_WR0_DAN_DMA_CDU_IRQ1_OUT,  /* 271 */	                            // From phy_fe_top_i of phy_fe_top.v
    E_SYS_INT_FE_RX_PRI0_CDU_IRQ1_OUT,              /* 272 */	                // From phy_fe_top_i of phy_fe_top.v
    E_SYS_INT_FE_RX_PRI0_CDU_IRQ0_OUT,              /* 273 */	                // From phy_fe_top_i of phy_fe_top.v
    E_SYS_INT_FE_RX_CDU_IRQ1_OUT,                   /* 274 */	                // From phy_fe_top_i of phy_fe_top.v
    E_SYS_INT_FE_RX_CDU_IRQ0_OUT,                   /* 275 */	                // From phy_fe_top_i of phy_fe_top.v
    E_SYS_INT_RX_SEMAPHOR_0,                        /* 276 */	                   // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_SEMAPHOR_1,                        /* 277 */	                   // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_SEMAPHOR_2,                        /* 278 */	                   // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_SEMAPHOR_3,                        /* 279 */	                   // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_SEMAPHOR_4,                        /* 280 */	                   // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_SEMAPHOR_5,                        /* 281 */	                   // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_SEMAPHOR_6,                        /* 282 */	                   // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_SEMAPHOR_7,                        /* 283 */	                   // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_GP_DMA1_0,                   /* 284 */	                       // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_GP_DMA1_1,                   /* 285 */	                       // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_GP_DMA1_2,                   /* 286 */	                       // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_GP_DMA1_3,                   /* 287 */	                       // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_GP_DMA1_4,                   /* 288 */	                       // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_GP_DMA1_5,                   /* 289 */	                       // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_GP_DMA1_6,                   /* 290 */	                       // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_GP_DMA1_7,                   /* 291 */	                       // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_GP_DMA0_0,                   /* 292 */	                   // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_GP_DMA0_1,                   /* 293 */	                   // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_GP_DMA0_2,                   /* 294 */	                   // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_GP_DMA0_3,                   /* 295 */	                   // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_GP_DMA0_4,                   /* 296 */	                   // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_GP_DMA0_5,                   /* 297 */	                   // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_GP_DMA0_6,                   /* 298 */	                   // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_GP_DMA0_7,                   /* 299 */	                   // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_FMSH_WR_DAN_DMA_CDU_IRQ1_OUT_0,   /* 300 */	                           // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_FMSH_WR_DAN_DMA_CDU_IRQ1_OUT_1,   /* 301 */	                           // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_FMSH_RD_DAN_DMA_CDU_IRQ1_OUT_0,   /* 302 */	                           // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_FMSH_RD_DAN_DMA_CDU_IRQ1_OUT_1,   /* 303 */	                           // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_FFT_TOP5_WR_DAN_DMA_CDU_IRQ1_OUT,  /* 304 */	                            // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_FFT_TOP5_RD_DAN_DMA_CDU_IRQ1_OUT,  /* 305 */	                            // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_FFT_TOP5_FFT_CDU_IRQ1_OUT,         /* 306 */	                     // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_FFT_TOP5_FFT_CDU_IRQ0_OUT,         /* 307 */	                     // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_FFT_TOP4_WR_DAN_DMA_CDU_IRQ1_OUT,  /* 308 */	                            // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_FFT_TOP4_RD_DAN_DMA_CDU_IRQ1_OUT,  /* 309 */	                            // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_FFT_TOP4_FFT_CDU_IRQ1_OUT,         /* 310 */	                     // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_FFT_TOP4_FFT_CDU_IRQ0_OUT,         /* 311 */	                     // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_FFT_TOP3_WR_DAN_DMA_CDU_IRQ1_OUT,  /* 312 */	                            // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_FFT_TOP3_RD_DAN_DMA_CDU_IRQ1_OUT,  /* 313 */	                            // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_FFT_TOP3_FFT_CDU_IRQ1_OUT,         /* 314 */	                     // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_FFT_TOP3_FFT_CDU_IRQ0_OUT,         /* 315 */	                     // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_FFT_TOP2_WR_DAN_DMA_CDU_IRQ1_OUT,  /* 316 */	                            // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_FFT_TOP2_RD_DAN_DMA_CDU_IRQ1_OUT,  /* 317 */	                            // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_FFT_TOP2_FFT_CDU_IRQ1_OUT,         /* 318 */	                     // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_FFT_TOP2_FFT_CDU_IRQ0_OUT,         /* 319 */	                     // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_FFT_TOP1_WR_DAN_DMA_CDU_IRQ1_OUT,  /* 320 */	                            // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_FFT_TOP1_RD_DAN_DMA_CDU_IRQ1_OUT,  /* 321 */	                            // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_FFT_TOP1_FFT_CDU_IRQ1_OUT,         /* 322 */	                     // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_FFT_TOP1_FFT_CDU_IRQ0_OUT,         /* 323 */	                     // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_FFT_TOP0_WR_DAN_DMA_CDU_IRQ1_OUT,  /* 324 */	                            // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_FFT_TOP0_RD_DAN_DMA_CDU_IRQ1_OUT,  /* 325 */	                            // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_FFT_TOP0_FFT_CDU_IRQ1_OUT,         /* 326 */	                     // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_FFT_TOP0_FFT_CDU_IRQ0_OUT,         /* 327 */	                     // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_DSP_SS3_CDU_IRQ1_OUT,              /* 328 */	                // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_DSP_SS3_CDU_IRQ0_OUT,              /* 329 */	                // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_DSP_SS2_CDU_IRQ1_OUT,              /* 330 */	                // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_DSP_SS2_CDU_IRQ0_OUT,              /* 331 */	                // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_DSP_SS1_CDU_IRQ1_OUT,              /* 332 */	                // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_DSP_SS1_CDU_IRQ0_OUT,              /* 333 */	                // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_DSP_SS0_CDU_IRQ1_OUT,              /* 334 */	                // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_DSP_SS0_CDU_IRQ0_OUT,              /* 335 */	                // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_DSP3_WR_DAN_DMA_CDU_IRQ1_OUT,      /* 336 */	                        // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_DSP3_RD_DAN_DMA_CDU_IRQ1_OUT,      /* 337 */	                        // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_DSP2_WR_DAN_DMA_CDU_IRQ1_OUT,      /* 338 */	                        // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_DSP2_RD_DAN_DMA_CDU_IRQ1_OUT,      /* 339 */	                        // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_DSP1_WR_DAN_DMA_CDU_IRQ1_OUT,      /* 340 */	                        // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_DSP1_RD_DAN_DMA_CDU_IRQ1_OUT,      /* 341 */	                        // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_DSP0_WR_DAN_DMA_CDU_IRQ1_OUT,      /* 342 */	                        // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_DSP0_RD_DAN_DMA_CDU_IRQ1_OUT,      /* 343 */	                        // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_DMSH_WR_DAN_DMA_CDU_IRQ1_OUT_0,   /* 344 */	                           // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_DMSH_WR_DAN_DMA_CDU_IRQ1_OUT_1,   /* 345 */	                           // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_DMSH_WR_DAN_DMA_CDU_IRQ1_OUT_2,   /* 346 */	                           // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_DMSH_WR_DAN_DMA_CDU_IRQ1_OUT_3,   /* 347 */	                           // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_DMSH_RD_DAN_DMA_CDU_IRQ1_OUT_0,   /* 348 */	                           // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_DMSH_RD_DAN_DMA_CDU_IRQ1_OUT_1,   /* 349 */	                           // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_DMSH_RD_DAN_DMA_CDU_IRQ1_OUT_2,   /* 350 */	                           // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_DMSH_RD_DAN_DMA_CDU_IRQ1_OUT_3,   /* 351 */	                           // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_DAN_IPC3_CDU_IRQ1,                 /* 352 */	                       // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_DAN_IPC3_CDU_IRQ0,                 /* 353 */	                       // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_DAN_IPC2_CDU_IRQ1,                 /* 354 */	                // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_DAN_IPC2_CDU_IRQ0,                 /* 355 */	                // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_DAN_IPC1_CDU_IRQ1,                 /* 356 */	                // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_DAN_IPC1_CDU_IRQ0,                 /* 357 */	                // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_DAN_IPC0_CDU_IRQ1,                 /* 358 */	                // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_DAN_IPC0_CDU_IRQ0,                 /* 359 */	                // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_CPU_SS3_CDU_IRQ1_OUT,              /* 360 */	                         // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_CPU_SS3_CDU_IRQ0_OUT,              /* 361 */	                         // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_CPU_SS2_CDU_IRQ1_OUT,              /* 362 */	                         // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_CPU_SS2_CDU_IRQ0_OUT,              /* 363 */	                         // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_CPU_SS1_CDU_IRQ1_OUT,              /* 364 */	                // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_CPU_SS1_CDU_IRQ0_OUT,              /* 365 */	                // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_CPU_SS0_CDU_IRQ1_OUT,              /* 366 */	                // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_CPU_SS0_CDU_IRQ0_OUT,              /* 367 */	                // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_CPU3_WR_DAN_DMA_CDU_IRQ1_OUT,      /* 368 */	                              // From phy_rx_top.v
    E_SYS_INT_RX_CPU3_RD_DAN_DMA_CDU_IRQ1_OUT,      /* 369 */	                              // From phy_rx_top.v
    E_SYS_INT_RX_CPU2_WR_DAN_DMA_CDU_IRQ1_OUT,      /* 370 */	                              // From phy_rx_top.v
    E_SYS_INT_RX_CPU2_RD_DAN_DMA_CDU_IRQ1_OUT,      /* 371 */	                              // From phy_rx_top.v
    E_SYS_INT_RX_CPU1_WR_DAN_DMA_CDU_IRQ1_OUT,      /* 372 */	                        // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_CPU1_RD_DAN_DMA_CDU_IRQ1_OUT,      /* 373 */	                        // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_CPU0_WR_DAN_DMA_CDU_IRQ1_OUT,      /* 374 */	                        // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_CPU0_RD_DAN_DMA_CDU_IRQ1_OUT,      /* 375 */	                        // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_RX_CB_CDU_IRQ0_OUT,                   /* 376 */	                // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_PHY_RX_DEC5_CDU_IRQ1,                 /* 377 */	                // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_PHY_RX_DEC5_CDU_IRQ0,                 /* 378 */	                // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_PHY_RX_DEC4_CDU_IRQ1,                 /* 379 */	                // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_PHY_RX_DEC4_CDU_IRQ0,                 /* 380 */	                // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_PHY_RX_DEC3_CDU_IRQ1,                 /* 381 */	                // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_PHY_RX_DEC3_CDU_IRQ0,                 /* 382 */	                // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_PHY_RX_DEC2_CDU_IRQ1,                 /* 383 */	                // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_PHY_RX_DEC2_CDU_IRQ0,                 /* 384 */	                // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_PHY_RX_DEC1_CDU_IRQ1,                 /* 385 */	                // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_PHY_RX_DEC1_CDU_IRQ0,                 /* 386 */	                // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_PHY_RX_DEC0_CDU_IRQ1,                 /* 387 */	                // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_PHY_RX_DEC0_CDU_IRQ0,                 /* 388 */	                // From phy_rx_top_i of phy_rx_top.v
    E_SYS_INT_TX_SEMAPHOR_0,                        /* 389 */	                   // From phy_tx_top_i of phy_tx_top.v
    E_SYS_INT_TX_SEMAPHOR_1,                        /* 390 */	                   // From phy_tx_top_i of phy_tx_top.v
    E_SYS_INT_TX_SEMAPHOR_2,                        /* 391 */	                   // From phy_tx_top_i of phy_tx_top.v
    E_SYS_INT_TX_SEMAPHOR_3,                        /* 392 */	                   // From phy_tx_top_i of phy_tx_top.v
    E_SYS_INT_TX_SEMAPHOR_4,                        /* 393 */	                   // From phy_tx_top_i of phy_tx_top.v
    E_SYS_INT_TX_SEMAPHOR_5,                        /* 394 */	                   // From phy_tx_top_i of phy_tx_top.v
    E_SYS_INT_TX_SEMAPHOR_6,                        /* 395 */	                   // From phy_tx_top_i of phy_tx_top.v
    E_SYS_INT_TX_SEMAPHOR_7,                        /* 396 */	                   // From phy_tx_top_i of phy_tx_top.v
    E_SYS_INT_TX_GP_DMA_INTR_0,                    /* 397 */	                   // From phy_tx_top_i of phy_tx_top.v
    E_SYS_INT_TX_GP_DMA_INTR_1,                    /* 398 */	                   // From phy_tx_top_i of phy_tx_top.v
    E_SYS_INT_TX_GP_DMA_INTR_2,                    /* 399 */	                   // From phy_tx_top_i of phy_tx_top.v
    E_SYS_INT_TX_GP_DMA_INTR_3,                    /* 400 */	                   // From phy_tx_top_i of phy_tx_top.v
    E_SYS_INT_TX_GP_DMA_INTR_4,                    /* 401 */	                   // From phy_tx_top_i of phy_tx_top.v
    E_SYS_INT_TX_GP_DMA_INTR_5,                    /* 402 */	                   // From phy_tx_top_i of phy_tx_top.v
    E_SYS_INT_TX_GP_DMA_INTR_6,                    /* 403 */	                   // From phy_tx_top_i of phy_tx_top.v
    E_SYS_INT_TX_GP_DMA_INTR_7,                    /* 404 */	                   // From phy_tx_top_i of phy_tx_top.v
    E_SYS_INT_TX_FMSH_WR_DAN_DMA_CDU_IRQ1_OUT_0,   /* 405 */	                           // From phy_tx_top_i of phy_tx_top.v
    E_SYS_INT_TX_FMSH_WR_DAN_DMA_CDU_IRQ1_OUT_1,   /* 406 */	                           // From phy_tx_top_i of phy_tx_top.v
    E_SYS_INT_TX_FMSH_RD_DAN_DMA_CDU_IRQ1_OUT_0,   /* 407 */	                           // From phy_tx_top_i of phy_tx_top.v
    E_SYS_INT_TX_FMSH_RD_DAN_DMA_CDU_IRQ1_OUT_1,   /* 408 */	                           // From phy_tx_top_i of phy_tx_top.v
    E_SYS_INT_TX_FFT_TOP1_WR_DAN_DMA_CDU_IRQ1_OUT,  /* 409 */	                            // From phy_tx_top_i of phy_tx_top.v
    E_SYS_INT_TX_FFT_TOP1_RD_DAN_DMA_CDU_IRQ1_OUT,  /* 410 */	                            // From phy_tx_top_i of phy_tx_top.v
    E_SYS_INT_TX_FFT_TOP1_FFT_CDU_IRQ1_OUT,         /* 411 */	                     // From phy_tx_top_i of phy_tx_top.v
    E_SYS_INT_TX_FFT_TOP1_FFT_CDU_IRQ0_OUT,         /* 412 */	                     // From phy_tx_top_i of phy_tx_top.v
    E_SYS_INT_TX_FFT_TOP0_WR_DAN_DMA_CDU_IRQ1_OUT,  /* 413 */	                            // From phy_tx_top_i of phy_tx_top.v
    E_SYS_INT_TX_FFT_TOP0_RD_DAN_DMA_CDU_IRQ1_OUT,  /* 414 */	                            // From phy_tx_top_i of phy_tx_top.v
    E_SYS_INT_TX_FFT_TOP0_FFT_CDU_IRQ1_OUT,         /* 415 */	                     // From phy_tx_top_i of phy_tx_top.v
    E_SYS_INT_TX_FFT_TOP0_FFT_CDU_IRQ0_OUT,         /* 416 */	                     // From phy_tx_top_i of phy_tx_top.v
    E_SYS_INT_TX_DSP_SS1_CDU_IRQ1_OUT,              /* 417 */	                // From phy_tx_top_i of phy_tx_top.v
    E_SYS_INT_TX_DSP_SS1_CDU_IRQ0_OUT,              /* 418 */	                // From phy_tx_top_i of phy_tx_top.v
    E_SYS_INT_TX_DSP_SS0_CDU_IRQ1_OUT,              /* 419 */	                // From phy_tx_top_i of phy_tx_top.v
    E_SYS_INT_TX_DSP_SS0_CDU_IRQ0_OUT,              /* 420 */	                // From phy_tx_top_i of phy_tx_top.v
    E_SYS_INT_TX_DSP1_WR_DAN_DMA_CDU_IRQ1_OUT,      /* 421 */	                        // From phy_tx_top_i of phy_tx_top.v
    E_SYS_INT_TX_DSP1_RD_DAN_DMA_CDU_IRQ1_OUT,      /* 422 */	                        // From phy_tx_top_i of phy_tx_top.v
    E_SYS_INT_TX_DSP0_WR_DAN_DMA_CDU_IRQ1_OUT,      /* 423 */	                        // From phy_tx_top_i of phy_tx_top.v
    E_SYS_INT_TX_DSP0_RD_DAN_DMA_CDU_IRQ1_OUT,      /* 424 */	                        // From phy_tx_top_i of phy_tx_top.v
    E_SYS_INT_TX_DMSH_WR_DAN_DMA_CDU_IRQ1_OUT_0,   /* 425 */	                           // From phy_tx_top_i of phy_tx_top.v
    E_SYS_INT_TX_DMSH_WR_DAN_DMA_CDU_IRQ1_OUT_1,   /* 426 */	                           // From phy_tx_top_i of phy_tx_top.v
    E_SYS_INT_TX_DMSH_RD_DAN_DMA_CDU_IRQ1_OUT_0,   /* 427 */	                           // From phy_tx_top_i of phy_tx_top.v
    E_SYS_INT_TX_DMSH_RD_DAN_DMA_CDU_IRQ1_OUT_1,   /* 428 */	                           // From phy_tx_top_i of phy_tx_top.v
    E_SYS_INT_TX_DAN_IPC2_CDU_IRQ1,                 /* 429 */	                // From phy_tx_top_i of phy_tx_top.v
    E_SYS_INT_TX_DAN_IPC2_CDU_IRQ0,                 /* 430 */	                // From phy_tx_top_i of phy_tx_top.v
    E_SYS_INT_TX_DAN_IPC1_CDU_IRQ1,                 /* 431 */	                // From phy_tx_top_i of phy_tx_top.v
    E_SYS_INT_TX_DAN_IPC1_CDU_IRQ0,                 /* 432 */	                // From phy_tx_top_i of phy_tx_top.v
    E_SYS_INT_TX_DAN_IPC0_CDU_IRQ1,                 /* 433 */	                // From phy_tx_top_i of phy_tx_top.v
    E_SYS_INT_TX_DAN_IPC0_CDU_IRQ0,                 /* 434 */	                // From phy_tx_top_i of phy_tx_top.v
    E_SYS_INT_TX_CPU_SS3_CDU_IRQ1_OUT,              /* 435 */	                // From phy_tx_top_i of phy_tx_top.v
    E_SYS_INT_TX_CPU_SS3_CDU_IRQ0_OUT,              /* 436 */	                // From phy_tx_top_i of phy_tx_top.v
    E_SYS_INT_TX_CPU_SS2_CDU_IRQ1_OUT,              /* 437 */	                // From phy_tx_top_i of phy_tx_top.v
    E_SYS_INT_TX_CPU_SS2_CDU_IRQ0_OUT,              /* 438 */	                // From phy_tx_top_i of phy_tx_top.v
    E_SYS_INT_TX_CPU_SS1_CDU_IRQ1_OUT,              /* 439 */	                // From phy_tx_top_i of phy_tx_top.v
    E_SYS_INT_TX_CPU_SS1_CDU_IRQ0_OUT,              /* 440 */	                // From phy_tx_top_i of phy_tx_top.v
    E_SYS_INT_TX_CPU_SS0_CDU_IRQ1_OUT,              /* 441 */	                // From phy_tx_top_i of phy_tx_top.v
    E_SYS_INT_TX_CPU_SS0_CDU_IRQ0_OUT,              /* 442 */	                // From phy_tx_top_i of phy_tx_top.v
    E_SYS_INT_TX_CPU3_WR_DAN_DMA_CDU_IRQ1_OUT,      /* 443 */	                        // From phy_tx_top_i of phy_tx_top.v
    E_SYS_INT_TX_CPU3_RD_DAN_DMA_CDU_IRQ1_OUT,      /* 444 */	                        // From phy_tx_top_i of phy_tx_top.v
    E_SYS_INT_TX_CPU2_WR_DAN_DMA_CDU_IRQ1_OUT,      /* 445 */	                        // From phy_tx_top_i of phy_tx_top.v
    E_SYS_INT_TX_CPU2_RD_DAN_DMA_CDU_IRQ1_OUT,      /* 446 */	                        // From phy_tx_top_i of phy_tx_top.v
    E_SYS_INT_TX_CPU1_WR_DAN_DMA_CDU_IRQ1_OUT,      /* 447 */	                        // From phy_tx_top_i of phy_tx_top.v
    E_SYS_INT_TX_CPU1_RD_DAN_DMA_CDU_IRQ1_OUT,      /* 448 */	                        // From phy_tx_top_i of phy_tx_top.v
    E_SYS_INT_TX_CPU0_WR_DAN_DMA_CDU_IRQ1_OUT,      /* 449 */	                        // From phy_tx_top_i of phy_tx_top.v
    E_SYS_INT_TX_CPU0_RD_DAN_DMA_CDU_IRQ1_OUT,      /* 450 */	                        // From phy_tx_top_i of phy_tx_top.v
    E_SYS_INT_TX_CB_CDU_IRQ0_OUT,                   /* 451 */	                // From phy_tx_top_i of phy_tx_top.v
    E_SYS_INT_PHY_TX_ENC5_CDU_IRQ1,                 /* 452 */	                // From phy_tx_top_i of phy_tx_top.v
    E_SYS_INT_PHY_TX_ENC5_CDU_IRQ0,                 /* 453 */	                // From phy_tx_top_i of phy_tx_top.v
    E_SYS_INT_PHY_TX_ENC4_CDU_IRQ1,                 /* 454 */	                // From phy_tx_top_i of phy_tx_top.v
    E_SYS_INT_PHY_TX_ENC4_CDU_IRQ0,                 /* 455 */	                // From phy_tx_top_i of phy_tx_top.v
    E_SYS_INT_PHY_TX_ENC3_CDU_IRQ1,                 /* 456 */	                // From phy_tx_top_i of phy_tx_top.v
    E_SYS_INT_PHY_TX_ENC3_CDU_IRQ0,                 /* 457 */	                // From phy_tx_top_i of phy_tx_top.v
    E_SYS_INT_PHY_TX_ENC2_CDU_IRQ1,                 /* 458 */	                // From phy_tx_top_i of phy_tx_top.v
    E_SYS_INT_PHY_TX_ENC2_CDU_IRQ0,                 /* 459 */	                // From phy_tx_top_i of phy_tx_top.v
    E_SYS_INT_PHY_TX_ENC1_CDU_IRQ1,                 /* 460 */	                // From phy_tx_top_i of phy_tx_top.v
    E_SYS_INT_PHY_TX_ENC1_CDU_IRQ0,                 /* 461 */	                // From phy_tx_top_i of phy_tx_top.v
    E_SYS_INT_PHY_TX_ENC0_CDU_IRQ1,                 /* 462 */	                // From phy_tx_top_i of phy_tx_top.v
    E_SYS_INT_PHY_TX_ENC0_CDU_IRQ0,                 /* 463 */	                // From phy_tx_top_i of phy_tx_top.v
    E_SYS_INT_TX_IC_B_INT,                          /* 464 */ 	                         // From phy_tx_top_wrap_i of phy_tx_top_wrap.v
    E_SYS_INT_TX_IC_A_INT,                          /* 465 */	            // From phy_tx_top_wrap_i of phy_tx_top_wrap.v
    E_SYS_INT_RX_IC_C_INT,                          /* 466 */	            // From phy_rx_top_wrap_i of phy_rx_top_wrap.v
    E_SYS_INT_RX_IC_B_INT,                          /* 467 */	            // From phy_rx_top_wrap_i of phy_rx_top_wrap.v
    E_SYS_INT_RX_IC_A_INT,                          /* 468 */	            // From phy_rx_top_wrap_i of phy_rx_top_wrap.v
    E_SYS_INT_NPU_IC_B_INT,                         /* 469 */	            // From npu_top_wrap_i of npu_top_wrap.v
    E_SYS_INT_NPU_IC_A_INT,                         /* 470 */	            // From npu_top_wrap_i of npu_top_wrap.v
    E_SYS_INT_FE_IC_TX_INT,                         /* 471 */	            // From phy_fe_top_wrap_i of phy_fe_top_wrap.v
    E_SYS_INT_FE_IC_RX_INT,                         /* 472 */	            // From phy_fe_top_wrap_i of phy_fe_top_wrap.v
    E_SYS_INT_SPARE_13,	                                /* 473 */
    E_SYS_INT_SPARE_14,	                                /* 474 */
    E_SYS_INT_SPARE_15,	                                /* 475 */
    E_SYS_INT_SPARE_16,	                                /* 476 */
    E_SYS_INT_SPARE_17,	                                /* 477 */
    E_SYS_INT_SPARE_18,	                                /* 478 */
    E_SYS_INT_SPARE_19,	                                /* 479 */
    E_SYS_INT_SPARE_20,	                                /* 480 */
    E_SYS_INT_SPARE_21,	                                /* 481 */
    E_SYS_INT_SPARE_22,	                                /* 482 */
    E_SYS_INT_SPARE_23,	                                /* 483 */
    E_SYS_INT_SPARE_24,	                                /* 484 */
    E_SYS_INT_SPARE_25,	                                /* 485 */
    E_SYS_INT_SPARE_26,	                                /* 486 */
    E_SYS_INT_SPARE_27,	                                /* 487 */
    E_SYS_INT_SPARE_28,	                                /* 488 */
    E_SYS_INT_SPARE_29,	                                /* 489 */
    E_SYS_INT_SPARE_30,	                                /* 490 */
    E_SYS_INT_SPARE_31,	                                /* 491 */
    E_SYS_INT_SPARE_32,	                                /* 492 */
    E_SYS_INT_SPARE_33,	                                /* 493 */
    E_SYS_INT_SPARE_34,	                                /* 494 */
    E_SYS_INT_SPARE_35,	                                /* 495 */
    E_SYS_INT_SPARE_36,	                                /* 496 */
    E_SYS_INT_SPARE_37,	                                /* 497 */
    E_SYS_INT_SPARE_38,	                                /* 498 */
    E_SYS_INT_SPARE_39,	                                /* 499 */
    E_SYS_INT_SPARE_40,	                                /* 500 */
    E_SYS_INT_SPARE_41,	                                /* 501 */
    E_SYS_INT_SPARE_42,	                                /* 502 */
    E_SYS_INT_SPARE_43,	                                /* 503 */
    E_SYS_INT_SPARE_44,	                                /* 504 */
    E_SYS_INT_SPARE_45,	                                /* 505 */
    E_SYS_INT_SPARE_46,	                                /* 506 */
    E_SYS_INT_SPARE_47,	                                /* 507 */
    E_SYS_INT_SPARE_48,	                                /* 508 */
    E_SYS_INT_SPARE_49,	                                /* 509 */
    E_SYS_INT_SPARE_50,	                                /* 510 */
    E_SYS_INT_SPARE_51 	                                /* 511 */
 }ESystemInt;


/* type of the interrupts */
typedef enum
{
	INT_TYPE_IRQ = 0,
	INT_TYPE_FIQ,
	INT_TYPE_SFIQ,
	INT_TYPE_LAST
}EIntType;

/* ID of the local interrupt */
typedef UINT32 INT_ID;

/* ISR callback prototype */
typedef void (* isr_callback)(UINT32 UserData);

/* Descriptor that contains all the data needed to bind an interrupts */
typedef struct IntDesc_s
{
	UINT32	        System_Int	    :9; //Choose a system int from enum ESystemInt
	UINT32	        Type		    :2; //Choose type of int from enum  EIntType        - NOT IMPLEMENTED!!!!!
	UINT32	        Priority	    :4; //Priority can range from 0 to 15               - NOT IMPLEMENTED!!!!!
	UINT32          UserData;           //A cookie that will be return as a parameter in the ISR callback
	isr_callback	Callback_Func;      //ISR callback to be called when interrupts occurs
}IntDesc_t;

/*
 * -----------------------------------------------------------
 * Static inline functions section
 * -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------
 * Global prototypes section
 * -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------
 * Function:	BindLocalInterrupt
 * Description:	Binds a system interrupt to a local interrupt
 * Input:		pIntDesc:	A pointer to a struct with the interrupt data
 * Output:		INT_ID:     ID of the local interrupt
 * -----------------------------------------------------------
 */
extern INT_ID ICTL_BindLocalInterrupt(IntDesc_t * pIntDesc);


/*
 * -----------------------------------------------------------
 * Function:	UnBindLocalInterrupt
 * Description:	Un Binds the local interrupt
 * Input:		INT_ID:     ID of the local interrupt
 * Output:		None
 * -----------------------------------------------------------
 */
extern void ICTL_UnBindLocalInterrupt(INT_ID id);


/*
 * -----------------------------------------------------------
 * Function:	MaskLocalInterrupts
 * Description:	Mask the local interrupts
 * Input:		mask:     mask of the interrupts to be masked, use ICTL_IdToMask to produce it
 * Output:		UINT32:   Old status of the interrupts, before masking
 * -----------------------------------------------------------
 */
extern UINT32 ICTL_MaskLocalInterrupts(UINT32 mask);


/*
 * -------------------UnMaskLocalInterrupts-------------------------------------
 * Function:	UnBindLocalInterrupt
 * Description:	UnMask the local interrupts
 * Input:		mask:     mask of the interrupts to be masked, use ICTL_IdToMask to produce it
 * Output:		None
 * -----------------------------------------------------------
 */
extern void ICTL_UnMaskLocalInterrupts(UINT32 mask);


/*
 * -----------------------------------------------------------
 * Function:	ForceLocalInterrupts
 * Description:	Forces an interrupt in the local ICTL entry
 * Input:		mask:     mask of the interrupts to be masked, use ICTL_IdToMask to produce it
 * Output:		None
 * -----------------------------------------------------------
 */
extern void ICTL_ForceLocalInterrupts(UINT32 mask);



/*
 * -----------------------------------------------------------
 * Function:	ICTL_EnableLocalInterrupts
 * Description:	Enables the local interrupt
 * Input:		mask:     mask of the interrupts to be enabled, use ICTL_IdToMask to produce it
 * Output:		None
 * -----------------------------------------------------------
 */
extern UINT32 ICTL_EnableLocalInterrupts(UINT32 mask);


/*
 * -------------------UnMaskLocalInterrupts-------------------------------------
 * Function:	ICTL_DisableLocalInterrupts
 * Description:	Disable the local interrupt
 * Input:		mask:     mask of the interrupts to be disabled, use ICTL_IdToMask to produce it
 * Output:		None
 * -----------------------------------------------------------
 */
extern void ICTL_DisableLocalInterrupts(UINT32 mask);




/*
 * -------------------UnMaskLocalInterrupts-------------------------------------
 * Function:	ICTL_IdToMask
 * Description:	Converts INT_ID to a mask that can be used with functions that need mask as input
 * Input:		INT_ID:     ID of the interrupt as returned by ICTL_BindLocalInterrupt function
 * Output:		None
 * -----------------------------------------------------------
 */

static inline UINT32 ICTL_IdToMask(INT_ID id)
{
    ASSERT(id<MAX_LOCAL_INTS);

    return (1<<id);
}



#endif//__ICTL_API_H__
/*
 * -----------------------------------------------------------
 * End of file
 * -----------------------------------------------------------
 */
