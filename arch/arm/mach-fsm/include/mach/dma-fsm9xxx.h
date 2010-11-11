/* DMOV configuration for fsm9xxx.
 *
 * Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of Code Aurora Forum, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __ASM_ARCH_FSM_9xxx_DMA_H

#define DMOV_SD_SIZE 0x1400
#define DMOV_SD_MASTER 0
#define DMOV_SD_AARM 3
#define DMOV_SD_MASTER_ADDR(off, ch) DMOV_ADDR(off, ch, DMOV_SD_MASTER)
#define DMOV_SD_AARM_ADDR(off, ch) DMOV_ADDR(off, ch, DMOV_SD_AARM)

/* all the DMA channels allocated to Scorpion */
#define DMOV_GP_CHAN          4
#define DMOV_CE1_IN_CHAN      5
#define DMOV_CE1_OUT_CHAN     6
#define DMOV_NAND_CHAN        7
#define DMOV_SDC1_CHAN        8
#define DMOV_GP2_CHAN         10
#define DMOV_CE2_IN_CHAN      12
#define DMOV_CE2_OUT_CHAN     13
#define DMOV_CE3_IN_CHAN      14
#define DMOV_CE3_OUT_CHAN     15

/* CRCI  */
#define DMOV_CE1_IN_CRCI       1
#define DMOV_CE1_OUT_CRCI      2
#define DMOV_CE1_HASH_CRCI     3

#define DMOV_NAND_CRCI_DATA    4
#define DMOV_NAND_CRCI_CMD     5

#define DMOV_SDC1_CRCI         6

#define DMOV_HSUART_TX_CRCI    7
#define DMOV_HSUART_RX_CRCI    8

#define DMOV_CE2_IN_CRCI       9
#define DMOV_CE2_OUT_CRCI      10
#define DMOV_CE2_HASH_CRCI     11

#define DMOV_CE3_IN_CRCI       12
#define DMOV_CE3_OUT_CRCI      13
#define DMOV_CE3_HASH_DONE_CRCI 14

/* the following should not be defined here for fsm. To make compile happy */
#define DMOV_SDC2_CRCI         6
#define DMOV_SDC3_CRCI         6
#define DMOV_SDC4_CRCI         6

#endif
