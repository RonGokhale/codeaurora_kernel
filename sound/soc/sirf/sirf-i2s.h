/*
 * SiRF I2S controllers define
 *
 * Copyright (c) 2011 Cambridge Silicon Radio Limited, a CSR plc group company.
 *
 * Licensed under GPLv2 or later.
 */

#ifndef _SIRF_I2S_H
#define _SIRF_I2S_H

#define AUDIO_CTRL_TX_FIFO_LEVEL_CHECK_MASK     0x3F
#define AUDIO_CTRL_TX_FIFO_SC_OFFSET    0
#define AUDIO_CTRL_TX_FIFO_LC_OFFSET    10
#define AUDIO_CTRL_TX_FIFO_HC_OFFSET    20

#define TX_FIFO_SC(x)           (((x) & AUDIO_CTRL_TX_FIFO_LEVEL_CHECK_MASK) \
				<< AUDIO_CTRL_TX_FIFO_SC_OFFSET)
#define TX_FIFO_LC(x)           (((x) & AUDIO_CTRL_TX_FIFO_LEVEL_CHECK_MASK) \
				<< AUDIO_CTRL_TX_FIFO_LC_OFFSET)
#define TX_FIFO_HC(x)           (((x) & AUDIO_CTRL_TX_FIFO_LEVEL_CHECK_MASK) \
				<< AUDIO_CTRL_TX_FIFO_HC_OFFSET)

#define AUDIO_CTRL_RX_FIFO_LEVEL_CHECK_MASK     0x0F
#define AUDIO_CTRL_RX_FIFO_SC_OFFSET    0
#define AUDIO_CTRL_RX_FIFO_LC_OFFSET    10
#define AUDIO_CTRL_RX_FIFO_HC_OFFSET    20

#define RX_FIFO_SC(x)           (((x) & AUDIO_CTRL_RX_FIFO_LEVEL_CHECK_MASK) \
				<< AUDIO_CTRL_RX_FIFO_SC_OFFSET)
#define RX_FIFO_LC(x)           (((x) & AUDIO_CTRL_RX_FIFO_LEVEL_CHECK_MASK) \
				<< AUDIO_CTRL_RX_FIFO_LC_OFFSET)
#define RX_FIFO_HC(x)           (((x) & AUDIO_CTRL_RX_FIFO_LEVEL_CHECK_MASK) \
				<< AUDIO_CTRL_RX_FIFO_HC_OFFSET)

#define AUDIO_CTRL_MODE_SEL			(0x0000)
#define AUDIO_CTRL_I2S_CTRL			(0x0020)
#define AUDIO_CTRL_I2S_TX_RX_EN			(0x0024)

#define AUDIO_CTRL_I2S_TXFIFO_OP		(0x0040)
#define AUDIO_CTRL_I2S_TXFIFO_LEV_CHK		(0x0044)
#define AUDIO_CTRL_I2S_TXFIFO_STS		(0x0048)
#define AUDIO_CTRL_I2S_TXFIFO_INT		(0x004C)
#define AUDIO_CTRL_I2S_TXFIFO_INT_MSK		(0x0050)

#define AUDIO_CTRL_I2S_RXFIFO_OP		(0x00B8)
#define AUDIO_CTRL_I2S_RXFIFO_LEV_CHK		(0x00BC)
#define AUDIO_CTRL_I2S_RXFIFO_STS		(0x00C0)
#define AUDIO_CTRL_I2S_RXFIFO_INT		(0x00C4)
#define AUDIO_CTRL_I2S_RXFIFO_INT_MSK		(0x00C8)

#define I2S_MODE				(1<<0)

#define I2S_LOOP_BACK				(1<<3)
#define	I2S_MCLK_DIV_SHIFT			15
#define I2S_MCLK_DIV_MASK			(0x1FF<<I2S_MCLK_DIV_SHIFT)
#define I2S_BITCLK_DIV_SHIFT			24
#define I2S_BITCLK_DIV_MASK			(0xFF<<I2S_BITCLK_DIV_SHIFT)

#define I2S_MCLK_EN				(1<<2)
#define I2S_REF_CLK_SEL_EXT			(1<<3)
#define I2S_DOUT_OE				(1<<4)
#define i2s_R2X_LP_TO_TX0			(1<<30)
#define i2s_R2X_LP_TO_TX1			(2<<30)
#define i2s_R2X_LP_TO_TX2			(3<<30)

#define AUDIO_FIFO_START		(1 << 0)
#define AUDIO_FIFO_RESET		(1 << 1)

#define AUDIO_FIFO_FULL			(1 << 0)
#define AUDIO_FIFO_EMPTY		(1 << 1)
#define AUDIO_FIFO_OFLOW		(1 << 2)
#define AUDIO_FIFO_UFLOW		(1 << 3)

#define I2S_RX_ENABLE			(1 << 0)
#define I2S_TX_ENABLE			(1 << 1)

/* Codec I2S Control Register defines */
#define I2S_SLAVE_MODE			(1 << 0)
#define I2S_SIX_CHANNELS		(1 << 1)
#define I2S_L_CHAN_LEN_SHIFT		(4)
#define I2S_L_CHAN_LEN_MASK		(0x1f << I2S_L_CHAN_LEN_SHIFT)
#define I2S_FRAME_LEN_SHIFT		(9)
#define I2S_FRAME_LEN_MASK		(0x3f << I2S_FRAME_LEN_SHIFT)

#define SIRF_I2S_NO_REF_CLK	0x0
#define SIRF_I2S_PWM_REF_CLK	0x1
#define SIRF_I2S_EXT_REF_CLK	0x2
#define SIRF_I2S_OUTPUT_MCLK	0x3

#define SIRF_I2S_MCLK_DIV	0x0

#endif /*__SIRF_I2S_H*/
