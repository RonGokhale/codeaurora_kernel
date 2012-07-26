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


#ifndef _dan_ipc_if_HW_REGS_H
#define _dan_ipc_if_HW_REGS_H
#ifdef __cplusplus
extern "C" {
#endif


//****************************************
//   dan_ipc_0 (Prototype: dan_ipc)
//****************************************


//define offsets
#define dan_ipc_if_SPARE_OFFSET  (0x0)
#define dan_ipc_if_FIFO_WR_ACCESS_0_OFFSET  (0x4)
#define dan_ipc_if_FIFO_RD_ACCESS_0_OFFSET  (0x8)
#define dan_ipc_if_FIFO_WR_ACCESS_1_OFFSET  (0xC)
#define dan_ipc_if_FIFO_RD_ACCESS_1_OFFSET  (0x10)
#define dan_ipc_if_FIFO_WR_ACCESS_2_OFFSET  (0x14)
#define dan_ipc_if_FIFO_RD_ACCESS_2_OFFSET  (0x18)
#define dan_ipc_if_FIFO_WR_ACCESS_3_OFFSET  (0x1C)
#define dan_ipc_if_FIFO_RD_ACCESS_3_OFFSET  (0x20)
#define dan_ipc_if_FIFO_WR_ACCESS_4_OFFSET  (0x24)
#define dan_ipc_if_FIFO_RD_ACCESS_4_OFFSET  (0x28)
#define dan_ipc_if_FIFO_WR_ACCESS_5_OFFSET  (0x2C)
#define dan_ipc_if_FIFO_RD_ACCESS_5_OFFSET  (0x30)
#define dan_ipc_if_FIFO_WR_ACCESS_6_OFFSET  (0x34)
#define dan_ipc_if_FIFO_RD_ACCESS_6_OFFSET  (0x38)
#define dan_ipc_if_FIFO_WR_ACCESS_7_OFFSET  (0x3C)
#define dan_ipc_if_FIFO_RD_ACCESS_7_OFFSET  (0x40)
#define dan_ipc_if_FIFO_0_STATUS_OFFSET  (0x44)
#define dan_ipc_if_FIFO_1_STATUS_OFFSET  (0x48)
#define dan_ipc_if_FIFO_2_STATUS_OFFSET  (0x4C)
#define dan_ipc_if_FIFO_3_STATUS_OFFSET  (0x50)
#define dan_ipc_if_FIFO_4_STATUS_OFFSET  (0x54)
#define dan_ipc_if_FIFO_5_STATUS_OFFSET  (0x58)
#define dan_ipc_if_FIFO_6_STATUS_OFFSET  (0x5C)
#define dan_ipc_if_FIFO_7_STATUS_OFFSET  (0x60)
#define dan_ipc_if_FIFO_THR_CFG_OFFSET  (0x64)
#define dan_ipc_if_CDU_INT0_PULSE_N_LEVEL_OFFSET  (0x508)
#define dan_ipc_if_CDU_INT1_PULSE_N_LEVEL_OFFSET  (0x50C)
#define dan_ipc_if_CDU_INT0_MASK_OFFSET  (0x510)
#define dan_ipc_if_CDU_INT1_MASK_OFFSET  (0x514)
#define dan_ipc_if_CDU_INT0_ENABLE_OFFSET  (0x518)
#define dan_ipc_if_CDU_INT1_ENABLE_OFFSET  (0x51C)
#define dan_ipc_if_CDU_INT0_STATUS_OFFSET  (0x520)
#define dan_ipc_if_CDU_INT1_STATUS_OFFSET  (0x524)
#define dan_ipc_if_CDU_INT0_RAW_STATUS_OFFSET  (0x528)
#define dan_ipc_if_CDU_INT1_RAW_STATUS_OFFSET  (0x52C)
#define dan_ipc_if_CDU_INT0_CLEAR_OFFSET  (0x530)
#define dan_ipc_if_CDU_INT1_CLEAR_OFFSET  (0x534)


typedef struct dan_ipc_SPARE_s
{
#ifdef BIG_ENDIAN
	UINT32 SPARE:32;
#else
	UINT32 SPARE:32;
#endif
} dan_ipc_SPARE_t;

typedef struct dan_ipc_FIFO_WR_ACCESS_0_s
{
#ifdef BIG_ENDIAN
	UINT32 FIFO_WR_ACCESS:32;
#else
	UINT32 FIFO_WR_ACCESS:32;
#endif
} dan_ipc_FIFO_WR_ACCESS_0_t;

typedef struct dan_ipc_FIFO_RD_ACCESS_0_s
{
#ifdef BIG_ENDIAN
	UINT32 FIFO_RD_ACCESS:32;
#else
	UINT32 FIFO_RD_ACCESS:32;
#endif
} dan_ipc_FIFO_RD_ACCESS_0_t;

typedef struct dan_ipc_FIFO_WR_ACCESS_1_s
{
#ifdef BIG_ENDIAN
	UINT32 FIFO_WR_ACCESS:32;
#else
	UINT32 FIFO_WR_ACCESS:32;
#endif
} dan_ipc_FIFO_WR_ACCESS_1_t;

typedef struct dan_ipc_FIFO_RD_ACCESS_1_s
{
#ifdef BIG_ENDIAN
	UINT32 FIFO_RD_ACCESS:32;
#else
	UINT32 FIFO_RD_ACCESS:32;
#endif
} dan_ipc_FIFO_RD_ACCESS_1_t;

typedef struct dan_ipc_FIFO_WR_ACCESS_2_s
{
#ifdef BIG_ENDIAN
	UINT32 FIFO_WR_ACCESS:32;
#else
	UINT32 FIFO_WR_ACCESS:32;
#endif
} dan_ipc_FIFO_WR_ACCESS_2_t;

typedef struct dan_ipc_FIFO_RD_ACCESS_2_s
{
#ifdef BIG_ENDIAN
	UINT32 FIFO_RD_ACCESS:32;
#else
	UINT32 FIFO_RD_ACCESS:32;
#endif
} dan_ipc_FIFO_RD_ACCESS_2_t;

typedef struct dan_ipc_FIFO_WR_ACCESS_3_s
{
#ifdef BIG_ENDIAN
	UINT32 FIFO_WR_ACCESS:32;
#else
	UINT32 FIFO_WR_ACCESS:32;
#endif
} dan_ipc_FIFO_WR_ACCESS_3_t;

typedef struct dan_ipc_FIFO_RD_ACCESS_3_s
{
#ifdef BIG_ENDIAN
	UINT32 FIFO_RD_ACCESS:32;
#else
	UINT32 FIFO_RD_ACCESS:32;
#endif
} dan_ipc_FIFO_RD_ACCESS_3_t;

typedef struct dan_ipc_FIFO_WR_ACCESS_4_s
{
#ifdef BIG_ENDIAN
	UINT32 FIFO_WR_ACCESS:32;
#else
	UINT32 FIFO_WR_ACCESS:32;
#endif
} dan_ipc_FIFO_WR_ACCESS_4_t;

typedef struct dan_ipc_FIFO_RD_ACCESS_4_s
{
#ifdef BIG_ENDIAN
	UINT32 FIFO_RD_ACCESS:32;
#else
	UINT32 FIFO_RD_ACCESS:32;
#endif
} dan_ipc_FIFO_RD_ACCESS_4_t;

typedef struct dan_ipc_FIFO_WR_ACCESS_5_s
{
#ifdef BIG_ENDIAN
	UINT32 FIFO_WR_ACCESS:32;
#else
	UINT32 FIFO_WR_ACCESS:32;
#endif
} dan_ipc_FIFO_WR_ACCESS_5_t;

typedef struct dan_ipc_FIFO_RD_ACCESS_5_s
{
#ifdef BIG_ENDIAN
	UINT32 FIFO_RD_ACCESS:32;
#else
	UINT32 FIFO_RD_ACCESS:32;
#endif
} dan_ipc_FIFO_RD_ACCESS_5_t;

typedef struct dan_ipc_FIFO_WR_ACCESS_6_s
{
#ifdef BIG_ENDIAN
	UINT32 FIFO_WR_ACCESS:32;
#else
	UINT32 FIFO_WR_ACCESS:32;
#endif
} dan_ipc_FIFO_WR_ACCESS_6_t;

typedef struct dan_ipc_FIFO_RD_ACCESS_6_s
{
#ifdef BIG_ENDIAN
	UINT32 FIFO_RD_ACCESS:32;
#else
	UINT32 FIFO_RD_ACCESS:32;
#endif
} dan_ipc_FIFO_RD_ACCESS_6_t;

typedef struct dan_ipc_FIFO_WR_ACCESS_7_s
{
#ifdef BIG_ENDIAN
	UINT32 FIFO_WR_ACCESS:32;
#else
	UINT32 FIFO_WR_ACCESS:32;
#endif
} dan_ipc_FIFO_WR_ACCESS_7_t;

typedef struct dan_ipc_FIFO_RD_ACCESS_7_s
{
#ifdef BIG_ENDIAN
	UINT32 FIFO_RD_ACCESS:32;
#else
	UINT32 FIFO_RD_ACCESS:32;
#endif
} dan_ipc_FIFO_RD_ACCESS_7_t;

typedef struct dan_ipc_FIFO_0_STATUS_s
{
#ifdef BIG_ENDIAN
	UINT32 rsv0:26;
	UINT32 ERR:1;
	UINT32 FULL:1;
	UINT32 AFULL:1;
	UINT32 HALFULL:1;
	UINT32 AEMPTY:1;
	UINT32 EMPTY:1;
#else
	UINT32 EMPTY:1;
	UINT32 AEMPTY:1;
	UINT32 HALFULL:1;
	UINT32 AFULL:1;
	UINT32 FULL:1;
	UINT32 ERR:1;
	UINT32 rsv0:26;
#endif
} dan_ipc_FIFO_0_STATUS_t;

typedef struct dan_ipc_FIFO_1_STATUS_s
{
#ifdef BIG_ENDIAN
	UINT32 rsv0:26;
	UINT32 ERR:1;
	UINT32 FULL:1;
	UINT32 AFULL:1;
	UINT32 HALFULL:1;
	UINT32 AEMPTY:1;
	UINT32 EMPTY:1;
#else
	UINT32 EMPTY:1;
	UINT32 AEMPTY:1;
	UINT32 HALFULL:1;
	UINT32 AFULL:1;
	UINT32 FULL:1;
	UINT32 ERR:1;
	UINT32 rsv0:26;
#endif
} dan_ipc_FIFO_1_STATUS_t;

typedef struct dan_ipc_FIFO_2_STATUS_s
{
#ifdef BIG_ENDIAN
	UINT32 rsv0:26;
	UINT32 ERR:1;
	UINT32 FULL:1;
	UINT32 AFULL:1;
	UINT32 HALFULL:1;
	UINT32 AEMPTY:1;
	UINT32 EMPTY:1;
#else
	UINT32 EMPTY:1;
	UINT32 AEMPTY:1;
	UINT32 HALFULL:1;
	UINT32 AFULL:1;
	UINT32 FULL:1;
	UINT32 ERR:1;
	UINT32 rsv0:26;
#endif
} dan_ipc_FIFO_2_STATUS_t;

typedef struct dan_ipc_FIFO_3_STATUS_s
{
#ifdef BIG_ENDIAN
	UINT32 rsv0:26;
	UINT32 ERR:1;
	UINT32 FULL:1;
	UINT32 AFULL:1;
	UINT32 HALFULL:1;
	UINT32 AEMPTY:1;
	UINT32 EMPTY:1;
#else
	UINT32 EMPTY:1;
	UINT32 AEMPTY:1;
	UINT32 HALFULL:1;
	UINT32 AFULL:1;
	UINT32 FULL:1;
	UINT32 ERR:1;
	UINT32 rsv0:26;
#endif
} dan_ipc_FIFO_3_STATUS_t;

typedef struct dan_ipc_FIFO_4_STATUS_s
{
#ifdef BIG_ENDIAN
	UINT32 rsv0:26;
	UINT32 ERR:1;
	UINT32 FULL:1;
	UINT32 AFULL:1;
	UINT32 HALFULL:1;
	UINT32 AEMPTY:1;
	UINT32 EMPTY:1;
#else
	UINT32 EMPTY:1;
	UINT32 AEMPTY:1;
	UINT32 HALFULL:1;
	UINT32 AFULL:1;
	UINT32 FULL:1;
	UINT32 ERR:1;
	UINT32 rsv0:26;
#endif
} dan_ipc_FIFO_4_STATUS_t;

typedef struct dan_ipc_FIFO_5_STATUS_s
{
#ifdef BIG_ENDIAN
	UINT32 rsv0:26;
	UINT32 ERR:1;
	UINT32 FULL:1;
	UINT32 AFULL:1;
	UINT32 HALFULL:1;
	UINT32 AEMPTY:1;
	UINT32 EMPTY:1;
#else
	UINT32 EMPTY:1;
	UINT32 AEMPTY:1;
	UINT32 HALFULL:1;
	UINT32 AFULL:1;
	UINT32 FULL:1;
	UINT32 ERR:1;
	UINT32 rsv0:26;
#endif
} dan_ipc_FIFO_5_STATUS_t;

typedef struct dan_ipc_FIFO_6_STATUS_s
{
#ifdef BIG_ENDIAN
	UINT32 rsv0:26;
	UINT32 ERR:1;
	UINT32 FULL:1;
	UINT32 AFULL:1;
	UINT32 HALFULL:1;
	UINT32 AEMPTY:1;
	UINT32 EMPTY:1;
#else
	UINT32 EMPTY:1;
	UINT32 AEMPTY:1;
	UINT32 HALFULL:1;
	UINT32 AFULL:1;
	UINT32 FULL:1;
	UINT32 ERR:1;
	UINT32 rsv0:26;
#endif
} dan_ipc_FIFO_6_STATUS_t;

typedef struct dan_ipc_FIFO_7_STATUS_s
{
#ifdef BIG_ENDIAN
	UINT32 rsv0:26;
	UINT32 ERR:1;
	UINT32 FULL:1;
	UINT32 AFULL:1;
	UINT32 HALFULL:1;
	UINT32 AEMPTY:1;
	UINT32 EMPTY:1;
#else
	UINT32 EMPTY:1;
	UINT32 AEMPTY:1;
	UINT32 HALFULL:1;
	UINT32 AFULL:1;
	UINT32 FULL:1;
	UINT32 ERR:1;
	UINT32 rsv0:26;
#endif
} dan_ipc_FIFO_7_STATUS_t;

typedef struct dan_ipc_FIFO_THR_CFG_s
{
#ifdef BIG_ENDIAN
	UINT32 ALMOST_EMPTY:16;
	UINT32 ALMOST_FULL:16;
#else
	UINT32 ALMOST_FULL:16;
	UINT32 ALMOST_EMPTY:16;
#endif
} dan_ipc_FIFO_THR_CFG_t;

typedef struct dan_ipc_CDU_INT0_PULSE_N_LEVEL_s
{
#ifdef BIG_ENDIAN
	UINT32 SELECT:32;
#else
	UINT32 SELECT:32;
#endif
} dan_ipc_CDU_INT0_PULSE_N_LEVEL_t;

typedef struct dan_ipc_CDU_INT1_PULSE_N_LEVEL_s
{
#ifdef BIG_ENDIAN
	UINT32 SELECT:32;
#else
	UINT32 SELECT:32;
#endif
} dan_ipc_CDU_INT1_PULSE_N_LEVEL_t;

typedef struct dan_ipc_CDU_INT0_MASK_s
{
#ifdef BIG_ENDIAN
	UINT32 int_mask:32;
#else
	UINT32 int_mask:32;
#endif
} dan_ipc_CDU_INT0_MASK_t;

typedef struct dan_ipc_CDU_INT1_MASK_s
{
#ifdef BIG_ENDIAN
	UINT32 int_mask:32;
#else
	UINT32 int_mask:32;
#endif
} dan_ipc_CDU_INT1_MASK_t;

typedef struct dan_ipc_CDU_INT0_ENABLE_s
{
#ifdef BIG_ENDIAN
	UINT32 INT_CTRL_0_EN:32;
#else
	UINT32 INT_CTRL_0_EN:32;
#endif
} dan_ipc_CDU_INT0_ENABLE_t;

typedef struct dan_ipc_CDU_INT1_ENABLE_s
{
#ifdef BIG_ENDIAN
	UINT32 INT_CTRL_1_EN:32;
#else
	UINT32 INT_CTRL_1_EN:32;
#endif
} dan_ipc_CDU_INT1_ENABLE_t;

typedef struct dan_ipc_CDU_INT0_STATUS_s
{
#ifdef BIG_ENDIAN
	UINT32 INT_CTRL_0_STATUS:32;
#else
	UINT32 INT_CTRL_0_STATUS:32;
#endif
} dan_ipc_CDU_INT0_STATUS_t;

typedef struct dan_ipc_CDU_INT1_STATUS_s
{
#ifdef BIG_ENDIAN
	UINT32 INT_CTRL_1_STATUS:32;
#else
	UINT32 INT_CTRL_1_STATUS:32;
#endif
} dan_ipc_CDU_INT1_STATUS_t;

typedef struct dan_ipc_CDU_INT0_RAW_STATUS_s
{
#ifdef BIG_ENDIAN
	UINT32 INT_CTRL_0_RAW_STATUS:32;
#else
	UINT32 INT_CTRL_0_RAW_STATUS:32;
#endif
} dan_ipc_CDU_INT0_RAW_STATUS_t;

typedef struct dan_ipc_CDU_INT1_RAW_STATUS_s
{
#ifdef BIG_ENDIAN
	UINT32 INT_CTRL_1_RAW_STATUS:32;
#else
	UINT32 INT_CTRL_1_RAW_STATUS:32;
#endif
} dan_ipc_CDU_INT1_RAW_STATUS_t;

typedef struct dan_ipc_CDU_INT0_CLEAR_s
{
#ifdef BIG_ENDIAN
	UINT32 INT_CTRL_0_CLEAR:32;
#else
	UINT32 INT_CTRL_0_CLEAR:32;
#endif
} dan_ipc_CDU_INT0_CLEAR_t;

typedef struct dan_ipc_CDU_INT1_CLEAR_s
{
#ifdef BIG_ENDIAN
	UINT32 INT_CTRL_1_CLEAR:32;
#else
	UINT32 INT_CTRL_1_CLEAR:32;
#endif
} dan_ipc_CDU_INT1_CLEAR_t;

//module typedef
typedef struct dan_ipc_if_module_s
{
	dan_ipc_SPARE_t SPARE;
	dan_ipc_FIFO_WR_ACCESS_0_t FIFO_WR_ACCESS_0;
	dan_ipc_FIFO_RD_ACCESS_0_t FIFO_RD_ACCESS_0;
	dan_ipc_FIFO_WR_ACCESS_1_t FIFO_WR_ACCESS_1;
	dan_ipc_FIFO_RD_ACCESS_1_t FIFO_RD_ACCESS_1;
	dan_ipc_FIFO_WR_ACCESS_2_t FIFO_WR_ACCESS_2;
	dan_ipc_FIFO_RD_ACCESS_2_t FIFO_RD_ACCESS_2;
	dan_ipc_FIFO_WR_ACCESS_3_t FIFO_WR_ACCESS_3;
	dan_ipc_FIFO_RD_ACCESS_3_t FIFO_RD_ACCESS_3;
	dan_ipc_FIFO_WR_ACCESS_4_t FIFO_WR_ACCESS_4;
	dan_ipc_FIFO_RD_ACCESS_4_t FIFO_RD_ACCESS_4;
	dan_ipc_FIFO_WR_ACCESS_5_t FIFO_WR_ACCESS_5;
	dan_ipc_FIFO_RD_ACCESS_5_t FIFO_RD_ACCESS_5;
	dan_ipc_FIFO_WR_ACCESS_6_t FIFO_WR_ACCESS_6;
	dan_ipc_FIFO_RD_ACCESS_6_t FIFO_RD_ACCESS_6;
	dan_ipc_FIFO_WR_ACCESS_7_t FIFO_WR_ACCESS_7;
	dan_ipc_FIFO_RD_ACCESS_7_t FIFO_RD_ACCESS_7;
	dan_ipc_FIFO_0_STATUS_t FIFO_0_STATUS;
	dan_ipc_FIFO_1_STATUS_t FIFO_1_STATUS;
	dan_ipc_FIFO_2_STATUS_t FIFO_2_STATUS;
	dan_ipc_FIFO_3_STATUS_t FIFO_3_STATUS;
	dan_ipc_FIFO_4_STATUS_t FIFO_4_STATUS;
	dan_ipc_FIFO_5_STATUS_t FIFO_5_STATUS;
	dan_ipc_FIFO_6_STATUS_t FIFO_6_STATUS;
	dan_ipc_FIFO_7_STATUS_t FIFO_7_STATUS;
	dan_ipc_FIFO_THR_CFG_t FIFO_THR_CFG;
	UINT32 rsv_64_508 [296];
	dan_ipc_CDU_INT0_PULSE_N_LEVEL_t CDU_INT0_PULSE_N_LEVEL;
	dan_ipc_CDU_INT1_PULSE_N_LEVEL_t CDU_INT1_PULSE_N_LEVEL;
	dan_ipc_CDU_INT0_MASK_t CDU_INT0_MASK;
	dan_ipc_CDU_INT1_MASK_t CDU_INT1_MASK;
	dan_ipc_CDU_INT0_ENABLE_t CDU_INT0_ENABLE;
	dan_ipc_CDU_INT1_ENABLE_t CDU_INT1_ENABLE;
	dan_ipc_CDU_INT0_STATUS_t CDU_INT0_STATUS;
	dan_ipc_CDU_INT1_STATUS_t CDU_INT1_STATUS;
	dan_ipc_CDU_INT0_RAW_STATUS_t CDU_INT0_RAW_STATUS;
	dan_ipc_CDU_INT1_RAW_STATUS_t CDU_INT1_RAW_STATUS;
	dan_ipc_CDU_INT0_CLEAR_t CDU_INT0_CLEAR;
	dan_ipc_CDU_INT1_CLEAR_t CDU_INT1_CLEAR;
} dan_ipc_if_module_t;

extern dan_ipc_if_module_t *dan_ipc_if_module;

#ifdef __cplusplus
}
#endif
#endif /* _dan_ipc_if_HW_REGS_H*/



