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


#ifndef __DANIPC_LOWLEVEL_H__
#define __DANIPC_LOWLEVEL_H__

#ifdef __KERNEL__

#include <linux/irqflags.h>
#include <linux/spinlock.h>
#include <asm/atomic.h>

#define IPC_DUMMY_ADDR			0
#define PLATFORM_MAX_NUM_OF_NODES	32
#define IPC_BUF_SIZE			((2 * IPC_BUF_COUNT_MAX) *	\
						sizeof(ipc_buffer_t))

typedef uint8_t				ipc_buffer_t[IPC_BUF_SIZE_MAX];

extern ipc_buffer_t			*ipc_buffers;
extern const uint32_t			IPC_array_hw_access_phys[];
extern void				*IPC_array_hw_access[];
extern agentNameEntry_t			*agentTable;
extern long				*semaphore;

#define OS_printf_debug		printk

#define PLATFORM_my_ipc_id	/*NPU_ARM_3*/ 3

typedef struct
{
	/* Physical address of the FIFO data buffer *without* bit 31 set. */
	uint32_t		paddr;

	/* Virtual address of the FIFO data buffer. */
	void __iomem		*vaddr;

	/* How many skbs destined for this core are on delayed_skb list */
	atomic_t		pending_skbs;
} ipc_to_virt_map_t;

extern uint32_t virt_to_ipc(const int cpuid, const unsigned prio, void *v_addr);
extern void *ipc_to_virt(const int cpuid, const unsigned prio, const uint32_t raw_ipc_addr);

#define __IPC_AGENT_ID(cpuid, lid)				\
			(((cpuid&(PLATFORM_MAX_NUM_OF_NODES-1)) << 3) + (0x07 & (lid)))


#define HAL_INTERRUPT_SAVE_AREA		unsigned long flags
#define HAL_INTERRUPT_DISABLE		local_irq_save(flags)
#define HAL_INTERRUPT_RESTORE		local_irq_restore(flags)

extern unsigned	IPC_init(void);
extern void	IPC_trns_fifo_buff_init(uint8_t cpu_id);
extern void	IPC_routeTableInit (IPC_transport_func_t *defTranVecPtr);
extern volatile char *	IPC_trns_fifo_buffer_alloc(uint8_t dest_agent_id,
							IPC_trns_priority_e pri);
extern void	IPC_trns_fifo_buffer_free(VCHAR *ptr, UINT8 dest_agent_id, IPC_trns_priority_e pri);
extern int32_t	IPC_trns_fifo_buf_send(VCHAR *ptr, UINT8 destId, IPC_trns_priority_e pri);
extern volatile char *	IPC_trns_fifo2eth_buffer_alloc(UINT8 dest_agent_id, IPC_trns_priority_e pri);
extern void	IPC_trns_fifo2eth_buffer_free(VCHAR *ptr, UINT8 dest_agent_id, IPC_trns_priority_e pri);
extern int32_t	IPC_trns_fifo2eth_buffer_send(VCHAR *ptr, UINT8 destId, IPC_trns_priority_e pri);
extern char *	IPC_trns_fifo_buf_read(UINT8 agentId, IPC_trns_priority_e pri);
extern void	IPC_agent_table_clean(void);
extern uint8_t	IPC_getOwnNode(void);
extern char *	IPC_getAgentName(UINT8 inx);
extern void	IPC_setAgentName (char *name, UINT8 inx);
extern IPC_transport_func_t *IPC_getUtilFuncVector(UINT8 nodeId);


extern void	handle_incoming_packet(volatile char *const packet, UINT8 cpu_id, IPC_trns_priority_e pri);

extern ipc_to_virt_map_t	ipc_to_virt_map[PLATFORM_MAX_NUM_OF_NODES][2];

#endif /* __KERNEL__ */

#endif /* __DANIPC_LOWLEVEL_H__ */
