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


#ifndef _I_PHY_ADDR_SPACE_H
#define _I_PHY_ADDR_SPACE_H

/*
 * -----------------------------------------------------------
 * Include section
 * -----------------------------------------------------------
 */

//#include "I_sys_utils.h"

/*
 * -----------------------------------------------------------
 * MACRO (define) section
 * -----------------------------------------------------------
 */


/*
 * -----------------------------------------------------------
 * Type definition section
 * -----------------------------------------------------------
 */


#define	ADDR_NO_CACHE(x)			(((unsigned)(x)))

// SoC Addrace Space based on "Dan3400AddressSpace.xls"

// DDR
#define DDR_XC_BASE						ADDR_NO_CACHE(0x00000000)
#define	DDR_NPU							(DDR_XC_BASE+0x00000000)
#define	DDR_PHY							(DDR_XC_BASE+0x20000000)
// PHY_TX
#define	TX_XC_BASE						ADDR_NO_CACHE(0x40000000)
#define	TX_CB							(TX_XC_BASE+0x00000000)
#define	TX_RTA							(TX_XC_BASE+0x02000000)
#define	TX_RTB							(TX_XC_BASE+0x02010000)
#define	TX_CPU0_TCM						(TX_XC_BASE+0x04000000)
#define	TX_CPU1_TCM						(TX_XC_BASE+0x04100000)
#define	TX_CPU2_TCM						(TX_XC_BASE+0x04200000)
#define	TX_CPU3_TCM						(TX_XC_BASE+0x04300000)
#define	TX_DSP0_TCM						(TX_XC_BASE+0x04400000)
#define	TX_DSP1_TCM						(TX_XC_BASE+0x04500000)
#define	TX_DSP0_PM						(TX_XC_BASE+0x04700000)
#define	TX_DSP0_INTERRUPT				(TX_XC_BASE+0x04704000)
#define	TX_DSP0_WRITE_DMA				(TX_XC_BASE+0x04706000)
#define	TX_DSP0_READ_DMA				(TX_XC_BASE+0x04708000)
#define	TX_DSP1_PM						(TX_XC_BASE+0x0470A000)
#define	TX_DSP1_INTERRUPT				(TX_XC_BASE+0x0470E000)
#define	TX_DSP1_WRITE_DMA				(TX_XC_BASE+0x04710000)
#define	TX_DSP1_READ_DMA				(TX_XC_BASE+0x04712000)
#define	TX_CB_PM						(TX_XC_BASE+0x04714000)
#define	TX_IPC0_PM						(TX_XC_BASE+0x04716000)
#define	TX_IPC1_PM						(TX_XC_BASE+0x04718000)
#define	TX_FPGA_USE						(TX_XC_BASE+0x0471C000)		// NC_in_RTL
#define	TX_DEBUG						(TX_XC_BASE+0x0471E000)
#define	TX_CPU0_PM						(TX_XC_BASE+0x04720000)
#define	TX_CPU0_INTERRUPT				(TX_XC_BASE+0x04724000)
#define	TX_CPU0_WRITE_DMA				(TX_XC_BASE+0x04726000)
#define	TX_CPU0_READ_DMA				(TX_XC_BASE+0x04728000)
#define	TX_CPU1_PM						(TX_XC_BASE+0x0472A000)
#define	TX_CPU1_INTERRUPT				(TX_XC_BASE+0x0472E000)
#define	TX_CPU1_WRITE_DMA				(TX_XC_BASE+0x04730000)
#define	TX_CPU1_READ_DMA				(TX_XC_BASE+0x04732000)
#define	TX_FFT_TOP0_PM			   		(TX_XC_BASE+0x04734000)
#define	TX_FFT_TOP0_WRITE_DMA	   		(TX_XC_BASE+0x04736000)
#define	TX_FFT_TOP0_READ_DMA	   		(TX_XC_BASE+0x04738000)
#define	TX_CPU2_PM						(TX_XC_BASE+0x04740000)
#define	TX_CPU2_INTERRUPT				(TX_XC_BASE+0x04744000)
#define	TX_CPU2_WRITE_DMA				(TX_XC_BASE+0x04746000)
#define	TX_CPU2_READ_DMA				(TX_XC_BASE+0x04748000)
#define	TX_CPU3_PM						(TX_XC_BASE+0x0474A000)
#define	TX_CPU3_INTERRUPT				(TX_XC_BASE+0x0474E000)
#define	TX_CPU3_WRITE_DMA				(TX_XC_BASE+0x04750000)
#define	TX_CPU3_READ_DMA				(TX_XC_BASE+0x04752000)
#define	TX_ENC0_PM						(TX_XC_BASE+0x04754000)
#define	TX_ENC1_PM						(TX_XC_BASE+0x04756000)
#define	TX_ENC2_PM						(TX_XC_BASE+0x04758000)
#define	TX_ENC3_PM						(TX_XC_BASE+0x0475A000)
#define	TX_ENC4_PM						(TX_XC_BASE+0x0475C000)
#define	TX_ENC5_PM						(TX_XC_BASE+0x0475E000)
#define	TX_TOP_PM						(TX_XC_BASE+0x04760000)
#define	TX_FFT_TOP1_PM					(TX_XC_BASE+0x04762000)
#define	TX_FFT_TOP1_WRITE_DMA	   		(TX_XC_BASE+0x04764000)
#define	TX_FFT_TOP1_READ_DMA	   		(TX_XC_BASE+0x04766000)
#define	TX_DAN_DMA_WRAP_FMSH_WRITE0_DMA	(TX_XC_BASE+0x04768000)
#define	TX_DAN_DMA_WRAP_FMSH_WRITE1_DMA	(TX_XC_BASE+0x0476A000)
#define	TX_DAN_DMA_WRAP_FMSH_READ0_DMA	(TX_XC_BASE+0x0476C000)
#define	TX_DAN_DMA_WRAP_FMSH_READ1_DMA	(TX_XC_BASE+0x0476E000)
#define	TX_DAN_DMA_WRAP_DMSH_WRITE0_DMA	(TX_XC_BASE+0x04770000)
#define	TX_DAN_DMA_WRAP_DMSH_WRITE1_DMA	(TX_XC_BASE+0x04772000)
#define	TX_DAN_DMA_WRAP_DMSH_READ0_DMA	(TX_XC_BASE+0x04774000)
#define	TX_DAN_DMA_WRAP_DMSH_READ1_DMA	(TX_XC_BASE+0x04776000)
#define	TX_IPC2_PM						(TX_XC_BASE+0x04778000)
#define	TX_CPU0_FIFO_RW					(TX_XC_BASE+0x047A0000)
#define	TX_CPU1_FIFO_RW					(TX_XC_BASE+0x047A0400)
#define	TX_CPU2_FIFO_RW					(TX_XC_BASE+0x047A0800)
#define	TX_CPU3_FIFO_RW					(TX_XC_BASE+0x047A0C00)
#define	TX_DSP0_FIFO_RD					(TX_XC_BASE+0x047A1400)
#define	TX_DSP0_FIFO_WR					(TX_XC_BASE+0x047A1800)
#define	TX_DSP1_FIFO_RD					(TX_XC_BASE+0x047A1C00)
#define	TX_DSP1_FIFO_WR					(TX_XC_BASE+0x047A2000)
#define	TX_SRIO2						(TX_XC_BASE+0x08000000)
#define	TX_SRIO3						(TX_XC_BASE+0x08000000)
#define	TX_GP_DMA						(TX_XC_BASE+0x04800000)
#define	TX_PSEMAPHORE					(TX_XC_BASE+0x04802000) // Was renamed from TX_SEMAPHORE because name collision with THX
// FE
#define	FE_XC_BASE						ADDR_NO_CACHE(0x46000000)
#define	FE_TX_AXIS_WR0					(FE_XC_BASE+0x00000000)
#define	FE_TX_AXIS_WR1					(FE_XC_BASE+0x00000400)
#define	FE_TX_AXIS_WR2					(FE_XC_BASE+0x00000800)
#define	FE_TX_AXIS_WR3					(FE_XC_BASE+0x00000C00)
#define	FE_TX_AXIS_WR4					(FE_XC_BASE+0x00001000)
#define	FE_TX_AXIS_WR5					(FE_XC_BASE+0x00001400)
#define	FE_TX_AXIS_WR6					(FE_XC_BASE+0x00001800)
#define	FE_TX_AXIS_WR7					(FE_XC_BASE+0x00001C00)
#define	FE_TX_AXIS_WR8					(FE_XC_BASE+0x00002000)
#define	FE_TX_AXIS_WR9					(FE_XC_BASE+0x00002400)
#define	FE_TX_AXIS_WR10					(FE_XC_BASE+0x00002800)
#define	FE_TX_AXIS_WR11					(FE_XC_BASE+0x00002C00)
#define	FE_TX_AXIS_WR12					(FE_XC_BASE+0x00003000)
#define	FE_TX_AXIS_WR13					(FE_XC_BASE+0x00003400)
#define	FE_TX_AXIS_WR14					(FE_XC_BASE+0x00003800)
#define	FE_TX_AXIS_WR15					(FE_XC_BASE+0x00003C00)
#define	FE_TX_FIFO_AXIS_WR0				(FE_XC_BASE+0x00004000)
#define	FE_TX_FIFO_AXIS_WR1				(FE_XC_BASE+0x00004400)
#define	FE_TX_PRI0_AXIS_WR0				(FE_XC_BASE+0x00006000)
#define	FE_TX_PRI0_AXIS_WR1				(FE_XC_BASE+0x00006400)
#define	FE_TX_PRI1_AXIS_WR0				(FE_XC_BASE+0x00006800)
#define	FE_TX_PRI1_AXIS_WR1				(FE_XC_BASE+0x00006C00)
#define	FE_TX_PRI2_AXIS_WR0				(FE_XC_BASE+0x00007000)
#define	FE_TX_PRI2_AXIS_WR1				(FE_XC_BASE+0x00007400)
#define	FE_TX_PRI3_AXIS_WR0				(FE_XC_BASE+0x00007800)
#define	FE_TX_PRI3_AXIS_WR1				(FE_XC_BASE+0x00007C00)
#define	FE_TX_PRE_COMB0					(FE_XC_BASE+0x00100000)
#define	FE_TX_PRE_COMB1					(FE_XC_BASE+0x00101000)
#define	FE_TX_PRE_COMB2					(FE_XC_BASE+0x00102000)
#define	FE_TX_PRE_COMB3					(FE_XC_BASE+0x00103000)
#define	FE_TX_PRE_COMB4					(FE_XC_BASE+0x00104000)
#define	FE_TX_PRE_COMB5					(FE_XC_BASE+0x00105000)
#define	FE_TX_PRE_COMB6					(FE_XC_BASE+0x00106000)
#define	FE_TX_PRE_COMB7					(FE_XC_BASE+0x00107000)
#define	FE_TX_PRE_COMB8					(FE_XC_BASE+0x00108000)
#define	FE_TX_PRE_COMB9					(FE_XC_BASE+0x00109000)
#define	FE_TX_PRE_COMB10				(FE_XC_BASE+0x0010A000)
#define	FE_TX_PRE_COMB11				(FE_XC_BASE+0x0010B000)
#define	FE_TX_PRE_COMB12				(FE_XC_BASE+0x0010C000)
#define	FE_TX_PRE_COMB13				(FE_XC_BASE+0x0010D000)
#define	FE_TX_PRE_COMB14				(FE_XC_BASE+0x0010E000)
#define	FE_TX_PRE_COMB15				(FE_XC_BASE+0x0010F000)
#define	FE_TX_POST_COMB0				(FE_XC_BASE+0x00110000)
#define	FE_TX_POST_COMB1				(FE_XC_BASE+0x00111000)
#define	FE_TX_POST_COMB2				(FE_XC_BASE+0x00112000)
#define	FE_TX_POST_COMB3				(FE_XC_BASE+0x00113000)
#define	FE_TX_FE_TX_TOP					(FE_XC_BASE+0x00114000)
#define	FE_TX_PRI0_LUT					(FE_XC_BASE+0x00116000)
#define	FE_TX_PRI1_LUT					(FE_XC_BASE+0x00118000)
#define	FE_TX_PRI2_LUT					(FE_XC_BASE+0x0011A000)
#define	FE_TX_PRI3_LUT					(FE_XC_BASE+0x0011C000)
#define	FE_FIFO							(FE_XC_BASE+0x00120000)
#define	FE_TBUS_STORAGE					(FE_XC_BASE+0x00122000)
#define	FE_IO_CONTROL					(FE_XC_BASE+0x00124000)
#define	FE_GPIO1						(FE_XC_BASE+0x00126000)
#define	FE_DAN3400_TOP					(FE_XC_BASE+0x00128000)
#define	FE_GPIO2						(FE_XC_BASE+0x0012A000)
#define	FE_RX_CORE0						(FE_XC_BASE+0x00130000)
#define	FE_RX_CORE1						(FE_XC_BASE+0x00130800)
#define	FE_RX_CORE2						(FE_XC_BASE+0x00131000)
#define	FE_RX_CORE3						(FE_XC_BASE+0x00131800)
#define	FE_RX_CORE4						(FE_XC_BASE+0x00132000)
#define	FE_RX_CORE5						(FE_XC_BASE+0x00132800)
#define	FE_RX_CORE6						(FE_XC_BASE+0x00133000)
#define	FE_RX_CORE7						(FE_XC_BASE+0x00133800)
#define	FE_RX_CORE8						(FE_XC_BASE+0x00134000)
#define	FE_RX_CORE9						(FE_XC_BASE+0x00134800)
#define	FE_RX_CORE10					(FE_XC_BASE+0x00135000)
#define	FE_RX_CORE11					(FE_XC_BASE+0x00135800)
#define	FE_RX_CORE12					(FE_XC_BASE+0x00136000)
#define	FE_RX_CORE13					(FE_XC_BASE+0x00136800)
#define	FE_RX_CORE14					(FE_XC_BASE+0x00137000)
#define	FE_RX_CORE15					(FE_XC_BASE+0x00137800)
#define	FE_RX_TOP						(FE_XC_BASE+0x00138000)
#define	FE_TX_DAN_DMA_0					(FE_XC_BASE+0x00140000)
#define	FE_TX_DAN_DMA_1					(FE_XC_BASE+0x00142000)
#define	FE_TX_DAN_DMA_2					(FE_XC_BASE+0x00144000)
#define	FE_TX_DAN_DMA_3					(FE_XC_BASE+0x00146000)
#define	FE_TX_FIFO_DAN_DMA_0			(FE_XC_BASE+0x00148000)
#define	FE_TX_FIFO_DAN_DMA_1			(FE_XC_BASE+0x0014A000)
#define	FE_TX_FIFO_DAN_DMA_2			(FE_XC_BASE+0x0014C000)
#define	FE_TX_FIFO_DAN_DMA_3			(FE_XC_BASE+0x0014E000)
#define	FE_TX_FIFO_DAN_DMA_4			(FE_XC_BASE+0x00150000)
#define	FE_TX_FIFO_DAN_DMA_5			(FE_XC_BASE+0x00152000)
#define	FE_TX_FIFO_DAN_DMA_6			(FE_XC_BASE+0x00154000)
#define	FE_TX_FIFO_DAN_DMA_7			(FE_XC_BASE+0x00156000)
#define	FE_TX_FIFO_DAN_DMA_8			(FE_XC_BASE+0x00158000)
#define	FE_TX_FIFO_DAN_DMA_9			(FE_XC_BASE+0x0015A000)
#define	FE_TX_FIFO_DAN_DMA_10			(FE_XC_BASE+0x0015C000)
#define	FE_TX_FIFO_DAN_DMA_11			(FE_XC_BASE+0x0015E000)
#define	FE_TX_FIFO_DAN_DMA_12			(FE_XC_BASE+0x00160000)
#define	FE_TX_FIFO_DAN_DMA_13			(FE_XC_BASE+0x00162000)
#define	FE_TX_FIFO_DAN_DMA_14			(FE_XC_BASE+0x00164000)
#define	FE_TX_FIFO_DAN_DMA_15			(FE_XC_BASE+0x00166000)
#define	FE_RX_PRI0_DAN_DMA_0			(FE_XC_BASE+0x00168000)
#define	FE_RX_PRI0_DAN_DMA_1			(FE_XC_BASE+0x0016A000)
#define	FE_RX_PRI1_DAN_DMA_0			(FE_XC_BASE+0x0016C000)
#define	FE_RX_PRI1_DAN_DMA_1			(FE_XC_BASE+0x0016E000)
#define	FE_RX_PRI2_DAN_DMA_0			(FE_XC_BASE+0x00170000)
#define	FE_RX_PRI2_DAN_DMA_1			(FE_XC_BASE+0x00172000)
#define	FE_RX_PRI3_DAN_DMA_0			(FE_XC_BASE+0x00174000)
#define	FE_RX_PRI3_DAN_DMA_1			(FE_XC_BASE+0x00176000)
#define	FE_RX_PRI4_DAN_DMA_0			(FE_XC_BASE+0x00178000)
#define	FE_RX_PRI4_DAN_DMA_1			(FE_XC_BASE+0x0017A000)
#define	FE_DEBUG_PORT	   				(FE_XC_BASE+0x0017E000)
#define	FE_RTB	  						(FE_XC_BASE+0x001C0000)
#define	FE_RX_AXIS_WR0					(FE_XC_BASE+0x00400000)
#define	FE_RX_AXIS_WR1					(FE_XC_BASE+0x00400400)
#define	FE_RX_AXIS_WR2					(FE_XC_BASE+0x00400800)
#define	FE_RX_AXIS_WR3					(FE_XC_BASE+0x00400C00)
#define	FE_RX_FIFO_AXIS_WR0				(FE_XC_BASE+0x00401000)
#define	FE_RX_FIFO_AXIS_WR1				(FE_XC_BASE+0x00401400)
#define	FE_RX_FIFO_AXIS_WR2				(FE_XC_BASE+0x00401800)
#define	FE_RX_FIFO_AXIS_WR3				(FE_XC_BASE+0x00401C00)
#define	FE_RX_FIFO_AXIS_WR4				(FE_XC_BASE+0x00402000)
#define	FE_RX_FIFO_AXIS_WR5				(FE_XC_BASE+0x00402400)
#define	FE_RX_FIFO_AXIS_WR6				(FE_XC_BASE+0x00402800)
#define	FE_RX_FIFO_AXIS_WR7				(FE_XC_BASE+0x00402C00)
#define	FE_RX_FIFO_AXIS_WR8				(FE_XC_BASE+0x00403000)
#define	FE_RX_FIFO_AXIS_WR9				(FE_XC_BASE+0x00403400)
#define	FE_RX_FIFO_AXIS_WR10			(FE_XC_BASE+0x00403800)
#define	FE_RX_FIFO_AXIS_WR11			(FE_XC_BASE+0x00403C00)
#define	FE_RX_FIFO_AXIS_WR12			(FE_XC_BASE+0x00404000)
#define	FE_RX_FIFO_AXIS_WR13			(FE_XC_BASE+0x00404400)
#define	FE_RX_FIFO_AXIS_WR14			(FE_XC_BASE+0x00404800)
#define	FE_RX_FIFO_AXIS_WR15			(FE_XC_BASE+0x00404C00)
#define	FE_RX_FIFO_AXIS_RD0				(FE_XC_BASE+0x00405000)
#define	FE_RX_FIFO_AXIS_RD1				(FE_XC_BASE+0x00405400)
#define	FE_RX_WR_DAN_DMA_0				(FE_XC_BASE+0x00500000)
#define	FE_RX_WR_DAN_DMA_1				(FE_XC_BASE+0x00502000)
#define	FE_RX_WR_DAN_DMA_2				(FE_XC_BASE+0x00504000)
#define	FE_RX_WR_DAN_DMA_3				(FE_XC_BASE+0x00506000)
#define	FE_RX_WR_DAN_DMA_4				(FE_XC_BASE+0x00508000)
#define	FE_RX_WR_DAN_DMA_5				(FE_XC_BASE+0x0050A000)
#define	FE_RX_WR_DAN_DMA_6				(FE_XC_BASE+0x0050C000)
#define	FE_RX_WR_DAN_DMA_7				(FE_XC_BASE+0x0050E000)
#define	FE_RX_WR_DAN_DMA_8				(FE_XC_BASE+0x00510000)
#define	FE_RX_WR_DAN_DMA_9				(FE_XC_BASE+0x00512000)
#define	FE_RX_WR_DAN_DMA_10				(FE_XC_BASE+0x00514000)
#define	FE_RX_WR_DAN_DMA_11				(FE_XC_BASE+0x00516000)
#define	FE_RX_WR_DAN_DMA_12				(FE_XC_BASE+0x00518000)
#define	FE_RX_WR_DAN_DMA_13				(FE_XC_BASE+0x0051A000)
#define	FE_RX_WR_DAN_DMA_14				(FE_XC_BASE+0x0051C000)
#define	FE_RX_WR_DAN_DMA_15				(FE_XC_BASE+0x0051E000)
#define	FE_TX_PRI0_PM					(FE_XC_BASE+0x00520000)
#define	FE_TX_PRI1_PM					(FE_XC_BASE+0x00522000)
#define	FE_TX_PRI2_PM					(FE_XC_BASE+0x00524000)
#define	FE_TX_PRI3_PM					(FE_XC_BASE+0x00526000)
#define	FE_PHY_FE_TOP_PM	  			(FE_XC_BASE+0x00528000)
#define	FE_RX_PRI0_PM					(FE_XC_BASE+0x0052A000)
#define	FE_RX_PRI1_PM					(FE_XC_BASE+0x0052C000)
#define	FE_RX_PRI2_PM					(FE_XC_BASE+0x0052E000)
#define	FE_RX_PRI3_PM					(FE_XC_BASE+0x00530000)
#define	FE_RX_PRI4_PM					(FE_XC_BASE+0x00532000)
#define	FE_RX_PRI0_LUT					(FE_XC_BASE+0x00534000)
#define	FE_RX_PRI1_LUT					(FE_XC_BASE+0x00536000)
#define	FE_RX_PRI2_LUT					(FE_XC_BASE+0x00538000)
#define	FE_RX_PRI3_LUT					(FE_XC_BASE+0x0053A000)
#define	FE_RX_PRI4_LUT					(FE_XC_BASE+0x0053C000)

// ??
#define	FE_RTA 							(0x65700000)
// PHY_RX
#define	RX_XC_BASE						ADDR_NO_CACHE(0x50000000)
#define	RX_CB							(RX_XC_BASE+0x00000000)
#define	RX_IC_RTA_REGISTERS				(RX_XC_BASE+0x02000000)
#define	RX_IC_RTB_REGISTERS				(RX_XC_BASE+0x02010000)
#define	RX_IC_RTC_REGISTERS				(RX_XC_BASE+0x02020000)
#define	RX_CPU0_TCM						(RX_XC_BASE+0x04100000)
#define	RX_CPU1_TCM						(RX_XC_BASE+0x04300000)
#define	RX_DSP0_TCM						(RX_XC_BASE+0x04400000)
#define	RX_DSP1_TCM						(RX_XC_BASE+0x04500000)
#define	RX_DSP2_TCM						(RX_XC_BASE+0x04600000)
#define	RX_DSP3_TCM						(RX_XC_BASE+0x04700000)
#define	RX_DSP0_PM						(RX_XC_BASE+0x04900000)
#define	RX_DSP0_SLOW_INTERRUPT			(RX_XC_BASE+0x04904000)
#define	RX_DSP0_WRITE_DMA				(RX_XC_BASE+0x04906000)
#define	RX_DSP0_READ_DMA				(RX_XC_BASE+0x04908000)
#define	RX_DSP1_PM						(RX_XC_BASE+0x0490A000)
#define	RX_DSP1_SLOW_INTERRUPT			(RX_XC_BASE+0x0490E000)
#define	RX_DSP1_WRITE_DMA				(RX_XC_BASE+0x04910000)
#define	RX_DSP1_READ_DMA				(RX_XC_BASE+0x04912000)
#define	RX_CPU1_PM						(RX_XC_BASE+0x0491E000)
#define	RX_CPU1_SLOW_INTERRUPT			(RX_XC_BASE+0x04922000)
#define	RX_CPU1_WRITE_DMA				(RX_XC_BASE+0x04924000)
#define	RX_CPU1_READ_DMA				(RX_XC_BASE+0x04926000)
#define	RX_FFT_TOP4_PM					(RX_XC_BASE+0x04928000)
#define	RX_FFT_TOP4_WRITE_DMA			(RX_XC_BASE+0x0492A000)
#define	RX_FFT_TOP4_READ_DMA			(RX_XC_BASE+0x0492C000)
#define	RX_FFT_TOP5_PM					(RX_XC_BASE+0x0492E000)
#define	RX_FFT_TOP5_WRITE_DMA			(RX_XC_BASE+0x04930000)
#define	RX_FFT_TOP5_READ_DMA			(RX_XC_BASE+0x04932000)
#define	RX_DEC0_PM						(RX_XC_BASE+0x04934000)
#define	RX_DEC1_PM						(RX_XC_BASE+0x04936000)
#define	RX_IPC0_PM						(RX_XC_BASE+0x04938000)
#define	RX_IPC1_PM						(RX_XC_BASE+0x0493A000)
#define	RX_FFT_TOP0_PM					(RX_XC_BASE+0x04940000)
#define	RX_FFT_TOP0_WRITE_DMA			(RX_XC_BASE+0x04942000)
#define	RX_FFT_TOP0_READ_DMA			(RX_XC_BASE+0x04944000)
#define	RX_FFT_TOP1_PM					(RX_XC_BASE+0x04946000)
#define	RX_FFT_TOP1_WRITE_DMA			(RX_XC_BASE+0x04948000)
#define	RX_FFT_TOP1_READ_DMA			(RX_XC_BASE+0x0494A000)
#define	RX_DSP2_PM						(RX_XC_BASE+0x0494C000)
#define	RX_DSP2_SLOW_INTERRUPT			(RX_XC_BASE+0x04950000)
#define	RX_DSP2_WRITE_DMA				(RX_XC_BASE+0x04952000)
#define	RX_DSP2_READ_DMA				(RX_XC_BASE+0x04954000)
#define	RX_DSP3_PM						(RX_XC_BASE+0x04956000)
#define	RX_DSP3_SLOW_INTERRUPT			(RX_XC_BASE+0x0495A000)
#define	RX_DSP3_WRITE_DMA				(RX_XC_BASE+0x0495C000)
#define	RX_DSP3_READ_DMA				(RX_XC_BASE+0x0495E000)
#define	RX_DAN_DMA_WRAP_DMSH_WRITE0_DMA	(RX_XC_BASE+0x04960000)
#define	RX_DAN_DMA_WRAP_DMSH_WRITE1_DMA	(RX_XC_BASE+0x04962000)
#define	RX_DAN_DMA_WRAP_DMSH_WRITE2_DMA	(RX_XC_BASE+0x04964000)
#define	RX_DAN_DMA_WRAP_DMSH_WRITE3_DMA	(RX_XC_BASE+0x04966000)
#define	RX_DAN_DMA_WRAP_DMSH_READ0_DMA	(RX_XC_BASE+0x04968000)
#define	RX_DAN_DMA_WRAP_DMSH_READ1_DMA	(RX_XC_BASE+0x0496A000)
#define	RX_DAN_DMA_WRAP_DMSH_READ2_DMA	(RX_XC_BASE+0x0496C000)
#define	RX_DAN_DMA_WRAP_DMSH_READ3_DMA	(RX_XC_BASE+0x0496E000)
#define	RX_DEC2_PM						(RX_XC_BASE+0x04970000)
#define	RX_DEC3_PM						(RX_XC_BASE+0x04972000)
#define	RX_IPC2_PM						(RX_XC_BASE+0x04978000)
#define	RX_CPU0_PM						(RX_XC_BASE+0x0498A000)
#define	RX_CPU0_SLOW_INTERRUPT			(RX_XC_BASE+0x0498E000)
#define	RX_CPU0_WRITE_DMA				(RX_XC_BASE+0x04990000)
#define	RX_CPU0_READ_DMA				(RX_XC_BASE+0x04992000)
#define	RX_FFT_TOP2_PM					(RX_XC_BASE+0x04994000)
#define	RX_FFT_TOP2_WRITE_DMA			(RX_XC_BASE+0x04996000)
#define	RX_FFT_TOP2_READ_DMA			(RX_XC_BASE+0x04998000)
#define	RX_FFT_TOP3_PM					(RX_XC_BASE+0x0499A000)
#define	RX_FFT_TOP3_WRITE_DMA			(RX_XC_BASE+0x0499C000)
#define	RX_FFT_TOP3_READ_DMA			(RX_XC_BASE+0x0499E000)
#define	RX_DAN_DMA_WRAP_FMSH_WRITE0_DMA	(RX_XC_BASE+0x049A0000)
#define	RX_DAN_DMA_WRAP_FMSH_WRITE1_DMA	(RX_XC_BASE+0x049A2000)
#define	RX_DAN_DMA_WRAP_FMSH_READ0_DMA	(RX_XC_BASE+0x049A4000)
#define	RX_DAN_DMA_WRAP_FMSH_READ1_DMA	(RX_XC_BASE+0x049A6000)
#define	RX_DEC4_PM						(RX_XC_BASE+0x049A8000)
#define	RX_DEC5_PM						(RX_XC_BASE+0x049AA000)
#define	RX_TOP_PM 						(RX_XC_BASE+0x049AE000)
#define	RX_CB_PM						(RX_XC_BASE+0x049B0000)
#define	RX_DEBUG_PORT					(RX_XC_BASE+0x049B2000)
#define	RX_CPU0_FIFO_RW					(RX_XC_BASE+0x049D0400)
#define	RX_CPU1_FIFO_RW					(RX_XC_BASE+0x049D0C00)
#define	RX_DSP0_FIFO_RD					(RX_XC_BASE+0x049D1400)
#define	RX_DSP0_FIFO_WR					(RX_XC_BASE+0x049D1800)
#define	RX_DSP1_FIFO_RD					(RX_XC_BASE+0x049D1C00)
#define	RX_DSP1_FIFO_WR					(RX_XC_BASE+0x049D2000)
#define	RX_DSP2_FIFO_RD					(RX_XC_BASE+0x049D2400)
#define	RX_DSP2_FIFO_WR					(RX_XC_BASE+0x049D2800)
#define	RX_DSP3_FIFO_RD					(RX_XC_BASE+0x049D2C00)
#define	RX_DSP3_FIFO_WR					(RX_XC_BASE+0x049D3000)
#define	RX_GP_DMA0						(RX_XC_BASE+0x04D08000)
#define	RX_SEMAPHORE					(RX_XC_BASE+0x04D0C000)

// NPU
#define	NPU_XC_BASE						ADDR_NO_CACHE(0x60000000)

#if 1

#define	NPU_CB							(NPU_XC_BASE+0x00000000)
#define	NPU_CPU0_TCM					(NPU_XC_BASE+0x04000000)
#define	NPU_CPU1_TCM					(NPU_XC_BASE+0x04100000)
#define	NPU_CPU2_TCM					(NPU_XC_BASE+0x04200000)
#define	NPU_CPU3_TCM					(NPU_XC_BASE+0x04300000)
#define	NPU_CPU4_TCM					(NPU_XC_BASE+0x04400000)
#define	NPU_CPU5_TCM					(NPU_XC_BASE+0x04500000)
#define	NPU_ARM0_TCM					(NPU_XC_BASE+0x04800000)
#define	NPU_ARM1_TCM					(NPU_XC_BASE+0x04A00000)
#define	NPU_ARM2_TCM					(NPU_XC_BASE+0x04C00000)
#define	NPU_GP_DMA0						(NPU_XC_BASE+0x05000000)
#define	NPU_GP_DMA1						(NPU_XC_BASE+0x05001000)
#define	NPU_SRIO0						(NPU_XC_BASE+0x05004000)
#define	NPU_SRIO1						(NPU_XC_BASE+0x05005000)
#define	NPU_SRIO2						(NPU_XC_BASE+0x05006000)
#define	NPU_SRIO3						(NPU_XC_BASE+0x05007000)
#define	NPU_SEMAPHORE0					(NPU_XC_BASE+0x05008000)
#define	NPU_SEMAPHORE1					(NPU_XC_BASE+0x05009000)
#define	NPU_WIMAX_SECURITY				(NPU_XC_BASE+0x0500A000)
#define	NPU_SW_INTERRUPTS0				(NPU_XC_BASE+0x05040000)
#define	NPU_SW_INTERRUPTS1				(NPU_XC_BASE+0x05040400)
#define	NPU_SW_INTERRUPTS2				(NPU_XC_BASE+0x05040800)
#define	NPU_SW_INTERRUPTS3				(NPU_XC_BASE+0x05040C00)
#define	NPU_CFG_DDR_NPU					(NPU_XC_BASE+0x05044000)
#define	NPU_CFG_DDR_PHY					(NPU_XC_BASE+0x05046000)
#define	NPU_CPU0_PM						(NPU_XC_BASE+0x05700000)
#define	NPU_CPU0_INT					(NPU_XC_BASE+0x05702000)
#define	NPU_CPU0_DMA_WR					(NPU_XC_BASE+0x05704000)
#define	NPU_CPU0_DMA_RD					(NPU_XC_BASE+0x05706000)
#define	NPU_CPU1_PM						(NPU_XC_BASE+0x05708000)
#define	NPU_CPU1_INT					(NPU_XC_BASE+0x0570A000)
#define	NPU_CPU1_DMA_WR					(NPU_XC_BASE+0x0570C000)
#define	NPU_CPU1_DMA_RD					(NPU_XC_BASE+0x0570E000)
#define	NPU_CPU2_PM						(NPU_XC_BASE+0x05710000)
#define	NPU_CPU2_INT					(NPU_XC_BASE+0x05712000)
#define	NPU_CPU2_DMA_WR					(NPU_XC_BASE+0x05714000)
#define	NPU_CPU2_DMA_RD					(NPU_XC_BASE+0x05716000)
#define	NPU_CPU3_PM						(NPU_XC_BASE+0x05718000)
#define	NPU_CPU3_INT					(NPU_XC_BASE+0x0571A000)
#define	NPU_CPU3_DMA_WR					(NPU_XC_BASE+0x0571C000)
#define	NPU_CPU3_DMA_RD					(NPU_XC_BASE+0x0571E000)
#define	NPU_ARM0_INT					(NPU_XC_BASE+0x05720000)
#define	NPU_ARM0_PM						(NPU_XC_BASE+0x05722000)
#define	NPU_ARM1_INT					(NPU_XC_BASE+0x05724000)
#define	NPU_ARM1_PM						(NPU_XC_BASE+0x05726000)
#define	NPU_IPC0						(NPU_XC_BASE+0x05728000)
#define	NPU_IPC1						(NPU_XC_BASE+0x0572A000)
#define	NPU_IPC2						(NPU_XC_BASE+0x0572C000)
#endif
#define	NPU_UART						(NPU_XC_BASE+0x0572E000)
#define	NPU_TIMER0						(NPU_XC_BASE+0x05730000)
#define	NPU_TOP							(NPU_XC_BASE+0x05732000)
#define	NPU_TXF							(NPU_XC_BASE+0x05734000)
//#define	NPU_TXF							(NPU_XC_BASE+0x05736000)
#define	NPU_GMAC0						(NPU_XC_BASE+0x05738000)
#define	NPU_GPIO0						(NPU_XC_BASE+0x0573A000)
#define	NPU_TIMING_UNIT_PM				(NPU_XC_BASE+0x0573C000)
#define	NPU_CLOCK_GEN_PM				(NPU_XC_BASE+0x0573E000)

#if 1
#define	NPU_CPU4_PM						(NPU_XC_BASE+0x05740000)
#define	NPU_CPU4_INT					(NPU_XC_BASE+0x05742000)
#define	NPU_CPU4_DMA_WR					(NPU_XC_BASE+0x05744000)
#define	NPU_CPU4_DMA_RD					(NPU_XC_BASE+0x05746000)
#define	NPU_CPU5_PM						(NPU_XC_BASE+0x05748000)
#define	NPU_CPU5_INT					(NPU_XC_BASE+0x0574A000)
#define	NPU_CPU5_DMA_WR					(NPU_XC_BASE+0x0574C000)
#define	NPU_CPU5_DMA_RD					(NPU_XC_BASE+0x0574E000)
#define	NPU_ARM2_INT					(NPU_XC_BASE+0x05760000)
#define	NPU_ARM2_PM						(NPU_XC_BASE+0x05762000)
#define	NPU_ARM3_INT					(NPU_XC_BASE+0x05764000)
#define	NPU_ARM3_PM						(NPU_XC_BASE+0x05766000)
#define	NPU_IPC3						(NPU_XC_BASE+0x05768000)
#define	NPU_IPC4						(NPU_XC_BASE+0x0576A000)
//#define	NPU_WIMAX_SECURITY				(NPU_XC_BASE+0x0576C000)
#define	NPU_WDT0						(NPU_XC_BASE+0x0576E000)
#define	NPU_GP_SPI						(NPU_XC_BASE+0x05770000)
//#define	NPU_CB							(NPU_XC_BASE+0x05772000)
#define	NPU_FMSH_DMA_WR0				(NPU_XC_BASE+0x05774000)
#define	NPU_FMSH_DMA_WR1				(NPU_XC_BASE+0x05776000)
#define	NPU_FMSH_DMA_RD0				(NPU_XC_BASE+0x05778000)
#define	NPU_FMSH_DMA_RD1				(NPU_XC_BASE+0x0577A000)
#define	NPU_RXP							(NPU_XC_BASE+0x0577C000)
#endif

#define	NPU_GMAC1						(NPU_XC_BASE+0x0577E000)

#if 0
#define	NPU_SRIO0						(NPU_XC_BASE+0x05780000)
#define	NPU_SRIO1						(NPU_XC_BASE+0x05782000)
#define	NPU_SRIO2						(NPU_XC_BASE+0x05784000)
#define	NPU_SRIO3						(NPU_XC_BASE+0x05786000)
#define	NPU_SRIO0_RAB					(NPU_XC_BASE+0x05788000)
#define	NPU_SRIO1_RAB					(NPU_XC_BASE+0x0578A000)
#define	NPU_SRIO2_RAB					(NPU_XC_BASE+0x0578C000)
#define	NPU_SRIO3_RAB					(NPU_XC_BASE+0x0578E000)
#define	NPU_TIMER1						(NPU_XC_BASE+0x05790000)
#define	NPU_TIMER2						(NPU_XC_BASE+0x05792000)
#define	NPU_TIMER3						(NPU_XC_BASE+0x05794000)
#define	NPU_DEBUG_PORT					(NPU_XC_BASE+0x057BE000)
#define	NPU_SPACC0_AXIS					(NPU_XC_BASE+0x057C0000)
#define	NPU_SPACC1_AXIS					(NPU_XC_BASE+0x05800000)
#define	NPU_CPU0_FIFO_RW				(NPU_XC_BASE+0x05840000)
#define	NPU_CPU1_FIFO_RW				(NPU_XC_BASE+0x05840400)
#define	NPU_CPU2_FIFO_RW				(NPU_XC_BASE+0x05840800)
#define	NPU_CPU3_FIFO_RW				(NPU_XC_BASE+0x05840C00)
#define	NPU_CPU4_FIFO_RW				(NPU_XC_BASE+0x05841000)
#define	NPU_CPU5_FIFO_RW				(NPU_XC_BASE+0x05841400)
#define	NPU_RX_Parser_IRXDF_RD			(NPU_XC_BASE+0x05842000)
#define	NPU_RX_Parser_RXDF_WR			(NPU_XC_BASE+0x05842400)
#define	NPU_TX_Framer_TXDF_RD			(NPU_XC_BASE+0x05842800)
#define	NPU_SRIO0						(NPU_XC_BASE+0x08000000)
#define	NPU_SRIO1						(NPU_XC_BASE+0x10000000)
#define	NPU_BOOT_ROM					(NPU_XC_BASE+0x1fff0000)

#endif


/////////////////////////////////////
// junk - to be cleaned
/////////////////////////////////////

#define FE1_XC_BASE		(0x20000000)


/////////////////////////////////////////////////
/////////////////////////////////////////////////
// FPGA-TX registers
/////////////////////////////////////////////////
/////////////////////////////////////////////////


#define FPGA_TX_TOP_BASE_ADDR		(TX_XC_BASE+0x0471C000)
#define	FPGA_TX_STATUS_CNT_OFFSET		(0x00000030)


/////////////////////////////////////////////////
/////////////////////////////////////////////////
// GMAC registers
/////////////////////////////////////////////////
/////////////////////////////////////////////////


/////////////////////////////////////////////////
/////////////////////////////////////////////////
// FPGA-RX registers
/////////////////////////////////////////////////
/////////////////////////////////////////////////
#define FPGA_RX_TOP_BASE_ADDR			(RX_XC_BASE+0x04920000)
#define	FPGA_RX_STATUS_CNT_OFFSET	(0x00000030)

/////////////////////////////////////////////////
/////////////////////////////////////////////////
// CTC encoder (TX)
/////////////////////////////////////////////////
/////////////////////////////////////////////////

//#include "ctc_encoder_reg.h"

/////////////////////////////////////////////////
/////////////////////////////////////////////////
// CTC decoder (RX)
/////////////////////////////////////////////////
/////////////////////////////////////////////////

/////////////////////////////////////////////////
/////////////////////////////////////////////////
// FFT
/////////////////////////////////////////////////
/////////////////////////////////////////////////
//#include "FFT_reg.h"


/////////////////////////////////////////////////
/////////////////////////////////////////////////
// F-MESH
/////////////////////////////////////////////////
/////////////////////////////////////////////////
#define	F_MESH_BASE_ADDR_0		(TX_XC_BASE+0x04760000)
#define	F_MESH_BASE_ADDR_1		(TX_XC_BASE+0x0476001C)

#define	F_MESH_BASE_ADDR_2		(RX_XC_BASE+0x049AE000)
#define	F_MESH_BASE_ADDR_3		(RX_XC_BASE+0x049AE024)
#define	F_MESH_BASE_ADDR_4		(RX_XC_BASE+0x15772000)


/////////////////////////////////////////////////
/////////////////////////////////////////////////
// DSP
/////////////////////////////////////////////////
/////////////////////////////////////////////////
//#include "DSP_SS_reg.h"


/////////////////////////////////////////////////
/////////////////////////////////////////////////
// DMA
/////////////////////////////////////////////////
/////////////////////////////////////////////////

#define	DMA_STATIC_REGISTER


//#include "xDMA_reg.h"


// RX CPU1 PM
#define RX_CPU0_SS_IF_MODULE_ADDR   (RX_XC_BASE+0x0498a000) //0x5491e000
#define	CPU_SS_IF_DSS_FTRN_OFFSET   0x244

/////////////////////////////////////////////////
/////////////////////////////////////////////////
// IPC
/////////////////////////////////////////////////
/////////////////////////////////////////////////


#define IPC_TX1_ADDR	(TX_XC_BASE+0x04716000)	//TX_IPC0_PM
#define IPC_TX2_ADDR	(TX_XC_BASE+0x04718000)	//TX_IPC1_PM
#define IPC_TX3_ADDR	(TX_XC_BASE+0x04778000)	//TX_IPC2_PM

//#include "IPC_reg.h"

/////////////////////////////////////////////////
/////////////////////////////////////////////////
// CB
/////////////////////////////////////////////////
/////////////////////////////////////////////////


#define	TX_CB_BASE			(TX_CB)
#define	RX_CB_BASE			(RX_CB)

// CB-RX
#define RX_FFT_DATA_IN_BUF_ADDR		(RX_CB+0x00000000)
#define RX_FFT_DATA_OUT_BUF_ADDR	(RX_CB+0x00010000)

/////////////////////////////////////////////////
/////////////////////////////////////////////////
// CLOCKGEN
/////////////////////////////////////////////////
/////////////////////////////////////////////////


//#include "cgen_reg.h"

/////////////////////////////////////////////////
/////////////////////////////////////////////////
// FE (front-end units)
/////////////////////////////////////////////////
/////////////////////////////////////////////////

#define	FE_TX_FIFO_XDMA_BASE	(TX_XC_BASE+0x06148000)
#define FE_FIFO_BASE			(TX_XC_BASE+0x06120000)
#define FE_RX_PRI_BASE			(TX_XC_BASE+FE_RX_PRI0_PM)


//#include "fe_fifo_reg.h"
//#include "fe_rx_core_reg.h"
//#include "fe_rx_pri_reg.h"
//#include "fe_rx_top_reg.h"
//#include "fe_splitter_reg.h"
//#include "fe_tx_poc_reg.h"
//#include "fe_tx_prc_reg.h"
//#include "fe_tx_pri_reg.h"
//#include "fe_tx_top_reg.h"
//#include "fe_pri_lut.h"

/////////////////////////////////////////////////
/////////////////////////////////////////////////
// FE (front-end units)
/////////////////////////////////////////////////
/////////////////////////////////////////////////


/////////////////////////////////////////////////
/////////////////////////////////////////////////
// SPI
/////////////////////////////////////////////////
/////////////////////////////////////////////////
//#include "spi_reg.h"

/////////////////////////////////////////////////
/////////////////////////////////////////////////
// TU
/////////////////////////////////////////////////
/////////////////////////////////////////////////
//#include "tu_reg.h"
//#include "tu_css_reg.h"



#endif	// _I_PHY_ADDR_SPACE_H

/*
 * -----------------------------------------------------------
 * End of file
 * -----------------------------------------------------------
 */
