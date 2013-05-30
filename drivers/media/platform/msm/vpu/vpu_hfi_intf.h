/* Copyright (c) 2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __H_VPU_HFI_INTF_H__
#define __H_VPU_HFI_INTF_H__

#include "vpu_debug.h"

struct hfi_queue_table_header {
	u32 qtbl_version;
	u32 qtbl_size;
	u32 qtbl_qhdr0_offset;
	u32 qtbl_qhdr_size;
	u32 qtbl_num_q;
	u32 qtbl_num_active_q;
};

/* qhdr_status */
#define QHDR_STATUS_INACTIVE		0
#define QHDR_STATUS_ACTIVE		1

/* qhdr_type bitmask */
#define QHDR_TYPE_TX_MODE_SHIFT		24
#define QHDR_TYPE_RX_MODE_SHIFT		16
#define Q_TYPE_INT_ONE			0
#define Q_TYPE_INT_MANY			1
#define Q_TYPE_POLL			2
#define QHDR_TYPE_PRIORITY_SHIFT	8
#define QHDR_TYPE_DIR_TX		0
#define QHDR_TYPE_DIR_RX		1

/* total number of queues */
#define VPU_MAX_TXQ_NUM			4
#define VPU_MAX_RXQ_NUM			4
#define VPU_MAX_QUEUE_NUM		(VPU_MAX_TXQ_NUM + VPU_MAX_RXQ_NUM)

/* qhdr_q_size in bytes */
#define VPU_SYS_QUEUE_SIZE		1024
#define VPU_SESSION_QUEUE_SIZE		2048
#define VPU_LOGGING_QUEUE_SIZE		2048 /* assume this size */
#define VPU_TUNE_QUEUE_SIZE		2048

#define TX_Q_IDX_TO_Q_ID(idx)		(VPU_SYSTEM_CMD_QUEUE_ID + (idx * 2))
#define RX_Q_IDX_TO_Q_ID(idx)		(VPU_SYSTEM_MSG_QUEUE_ID + (idx * 2))

struct hfi_queue_header {
	u32 qhdr_status;
	u32 qhdr_start_addr;
	u32 qhdr_type;
	u32 qhdr_q_size;
	u32 qhdr_pkt_size;
	u32 qhdr_pkt_drop_cnt;		/* ? */
	u32 qhdr_rx_wm;
	u32 qhdr_tx_wm;
	u32 qhdr_rx_req;
	u32 qhdr_tx_req;
	u32 qhdr_rx_irq_status;	/* not used */
	u32 qhdr_tx_irq_status;	/* not used */
	u32 qhdr_read_idx;
	u32 qhdr_write_idx;
};

static inline void vpu_hfi_init_qhdr(struct hfi_queue_header *qhdr, bool tx,
					u32 offset, u32 size)
{
	memset(qhdr, 0, sizeof(*qhdr));

	if (tx)
		qhdr->qhdr_type = QHDR_TYPE_DIR_TX |
				(Q_TYPE_POLL << QHDR_TYPE_TX_MODE_SHIFT) |
				(Q_TYPE_POLL << QHDR_TYPE_RX_MODE_SHIFT);
	else
		qhdr->qhdr_type = QHDR_TYPE_DIR_RX |
				(Q_TYPE_POLL << QHDR_TYPE_TX_MODE_SHIFT) |
				(Q_TYPE_POLL << QHDR_TYPE_RX_MODE_SHIFT);

	qhdr->qhdr_rx_wm = 1;
	qhdr->qhdr_tx_wm = 1;

	qhdr->qhdr_rx_req = 1; /* queue is empty intially */

	/* the queue array */
	qhdr->qhdr_start_addr = offset;
	qhdr->qhdr_q_size = size;

	/* the indices */
	qhdr->qhdr_read_idx = 0;
	qhdr->qhdr_write_idx = 0;

	/* make the queue active */
	qhdr->qhdr_status = QHDR_STATUS_INACTIVE;
}

static inline void vpu_hfi_deinit_qhdr(struct hfi_queue_header *qhdr)
{
}

static inline void vpu_hfi_enable_qhdr(struct hfi_queue_header *qhdr)
{
	qhdr->qhdr_status = QHDR_STATUS_ACTIVE;
}

static inline void vpu_hfi_disable_qhdr(struct hfi_queue_header *qhdr)
{
	qhdr->qhdr_status = QHDR_STATUS_INACTIVE;
}

static inline bool vpu_hfi_q_empty(struct hfi_queue_header *qhdr)
{
	return (qhdr->qhdr_read_idx == qhdr->qhdr_write_idx);
}

static inline u32 vpu_hfi_q_size(int q_id)
{
	/* q_id defined in vpu_hfi.h*/
	switch (q_id) {
	case VPU_SYSTEM_CMD_QUEUE_ID:
	case VPU_SYSTEM_MSG_QUEUE_ID:
		return VPU_SYS_QUEUE_SIZE;
		break;
	case VPU_SESSION_CMD_QUEUE_0_ID:
	case VPU_SESSION_MSG_QUEUE_0_ID:
	case VPU_SESSION_CMD_QUEUE_1_ID:
	case VPU_SESSION_MSG_QUEUE_1_ID:
		return VPU_SESSION_QUEUE_SIZE;
		break;
	case VPU_SYSTEM_LOG_QUEUE_ID:
		return VPU_LOGGING_QUEUE_SIZE;
		break;
	default:
		break;
	}
	return 0;
}

#ifndef VPU_MAPLE_FW_SIM
/*
 * VPU CSR register offsets
 * maple -> apps sgi interrupt:
 * 0x0000B050 MAPLE_CSR_APPS_SGI_STS
 * 0x0000B054 MAPLE_CSR_APPS_SGI_CLR
 *
 * apps->maple sgi interrupts:
 * 0x0000B084 MAPLE_CSR_MAPLE_SGI_EN_SET
 * 0x0000B088 MAPLE_CSR_MAPLE_SGI_EN_CLR
 * 0x0000B08C MAPLE_CSR_MAPLE_SGI_FORCELEVEL
 * 0x0000B090 MAPLE_CSR_MAPLE_SGI_STS
 * 0x0000B094 MAPLE_CSR_MAPLE_SGI_CLR
 * 0x0000B098 MAPLE_CSR_MAPLE_SGI_TRIG
 *
 * scratch register:
 * 0x0000B160 MAPLE_CSR_SW_SCRATCH0	vpu status
 * 0x0000B164 MAPLE_CSR_SW_SCRATCH1	qtable ready
 * 0x0000B168 MAPLE_CSR_SW_SCRATCH2	qtable lower 32 bit phy addr
 * 0x0000B16c MAPLE_CSR_SW_SCRATCH3	unused
 * 0x0000B170 MAPLE_CSR_SW_SCRATCH4	IPC polling register (IRQ bypass)
 */
#define MAPLE_CSR_SEC_RESET			0x000
#define MAPLE_CSR_BOOT_REMAP			0x020

#define MAPLE_CSR_APPS_SGI_STS			0x050
#define MAPLE_CSR_APPS_SGI_CLR			0x054

#define MAPLE_CSR_MAPLE_SGI_EN_SET		0x084
#define MAPLE_CSR_MAPLE_SGI_EN_CLR		0x088
#define MAPLE_CSR_MAPLE_SGI_FORCELEVEL		0x08c
#define MAPLE_CSR_MAPLE_SGI_STS			0x090
#define MAPLE_CSR_MAPLE_SGI_CLR			0x094
#define MAPLE_CSR_MAPLE_SGI_TRIG		0x098
#define MAPLE_CSR_SW_SCRATCH0_STS		0x160
#define MAPLE_CSR_SW_SCRATCH1_QTBL_INFO		0x164
#define MAPLE_CSR_SW_SCRATCH2_QTBL_ADDR		0x168
/*unused: MAPLE_CSR_SW_SCRATCH3			0x16c*/
#define MAPLE_CSR_SW_SCRATCH4_IPC_POLLING	0x170
#define MAPLE_HW_VERSION			0x1A0

/* Fixme: Remove this function once PIL is available */
static inline void raw_hfi_deassert_reset(u32 regbase, u32 phyaddr)
{
	/* clear VPU reset signal */
	wmb();
	writel_relaxed(1, regbase + MAPLE_CSR_SEC_RESET);

	/* make sure this go through */
	mb();
}

/* Fixme: Remove this function once PIL is available */
static inline void raw_hfi_assert_reset(u32 regbase, u32 phyaddr)
{
	/* clear VPU reset signal */
	wmb();
	writel_relaxed(0, regbase + MAPLE_CSR_SEC_RESET);

	/* make sure this go through */
	mb();
}

/* Fixme: Remove this function once PIL is available */
static inline void raw_hfi_vpu_boot_remap(u32 regbase, u32 phyaddr)
{
	/* allow VPU boot remap */
	wmb();
	writel_relaxed(0x00010db0, regbase + MAPLE_CSR_BOOT_REMAP);

	/* make sure this go through */
	mb();
}

static inline void raw_hfi_qtbl_paddr_set(u32 regbase, u32 phyaddr)
{
	/* lower 32 bit of qtable phy address */
	writel_relaxed(phyaddr, regbase + MAPLE_CSR_SW_SCRATCH2_QTBL_ADDR);

	/* barrier then make the qtbl infor ready */
	wmb();
	writel_relaxed(1, regbase + MAPLE_CSR_SW_SCRATCH1_QTBL_INFO);
}

static inline void raw_hfi_int_enable(u32 regbase)
{
	/* use edge interrrupt */
	writel_relaxed(0, regbase + MAPLE_CSR_MAPLE_SGI_FORCELEVEL);

	/* barrier, then enable sgi interrupt */
	wmb();
	writel_relaxed(1, regbase + MAPLE_CSR_MAPLE_SGI_EN_SET);
}

static inline void raw_hfi_int_disable(u32 regbase)
{
	/* disable sgi interrupt */
	wmb();
	writel_relaxed(1, regbase + MAPLE_CSR_MAPLE_SGI_EN_CLR);

	/* make sure this go through */
	mb();
}

static inline void raw_hfi_int_ack(u32 regbase)
{
	/* clear sgi interrupt */
	wmb();
	writel_relaxed(1, regbase + MAPLE_CSR_APPS_SGI_CLR);

	/* make sure this go through */
	mb();
}

static inline void raw_hfi_int_fire(u32 regbase)
{
	/* barrier, then trigger interrupt */
	wmb();
	writel_relaxed(1, regbase + MAPLE_CSR_MAPLE_SGI_TRIG);

	/* no need for barrier after */
}

static inline u32 raw_hfi_status_read(u32 regbase)
{
	u32 val;

	/* read vpu boot status */
	val = readl_relaxed(regbase + MAPLE_CSR_SW_SCRATCH0_STS);
	rmb();

	return val;
}


#define add2buf(dest, dest_size, temp, temp_size, __fmt, arg...) \
	do { \
		snprintf(temp, temp_size, __fmt, ## arg); \
		strlcat(dest, temp, dest_size); \
	} while (0)

static inline void raw_hfi_dump_csr_regs(u32 v_base, u32 p_base,
		char *buf, size_t buf_size)
{
	/* temporary buffer */
	size_t t_s = SZ_128;
	char t[t_s];
	u32 addr;
	u32 last_addr = v_base + MAPLE_HW_VERSION;

	/* read each register (4 per line) */
	for (addr = v_base; addr <= last_addr; addr += 4 * sizeof(u32))
		add2buf(buf, buf_size, t, t_s,
			"@0x%08x - 0x%08x 0x%08x 0x%08x 0x%08x\n",
			(addr - v_base + p_base),
			readl_relaxed(addr + 0 * sizeof(u32)),
			readl_relaxed(addr + 1 * sizeof(u32)),
			readl_relaxed(addr + 2 * sizeof(u32)),
			readl_relaxed(addr + 3 * sizeof(u32)));
}

#else

#define IPC_MEM_ORDER 4
void vpu_sim_start(struct device *dev, int irq, void *priv);
void vpu_sim_stop(struct device *dev);
void raw_hfi_qtbl_paddr_set(u32 regbase, u32 phyaddr);
void raw_hfi_int_enable(u32 regbase);
void raw_hfi_int_disable(u32 regbase);
void raw_hfi_int_ack(u32 regbase);
void raw_hfi_int_fire(u32 regbase);
u32 raw_hfi_status_read(u32 regbase);
#endif /* VPU_MAPLE_FW_SIM */

#endif /* __H_VPU_HFI_INTF_H__ */
