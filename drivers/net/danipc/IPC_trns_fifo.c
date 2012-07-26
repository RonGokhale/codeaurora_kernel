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


/*
 * -----------------------------------------------------------
 * Include section
 * -----------------------------------------------------------
 */

#include <asm/delay.h>
#include <asm/page.h>

#include "I_sys_types.h"
#include "I_sys_utils.h"
//#include "I_module_interface.h"
//#include "I_driver_interface.h"
//#include "I_kernel_interface.h"
//#ifndef UNIX
#include "I_phy_addr_space.h"
//#include "modules_import.h"
//#include "drivers_import.h"
//#include "kernels_import.h"
//#else
//#endif
//#include "IPC.h"
#include "IPC_reg.h"
#include "PLATFORM_api.h"
#include "ICTL_api.h"
#include "IPC_api.h"

#include "danipc_lowlevel.h"


#undef CODE_DEFAULT
#define CODE_DEFAULT


#define IPC_IRQ_USED_CF	1

//#define NPU_SEMAPHORE0                        0xe5008000
/* Virtual memory mapping of NPU_SEMAPHORE0 */
long				*semaphore;

#define SEMAPHORE0_SET_OFFSET		( 0x0 / sizeof(long) )
#define SEMAPHORE0_RESET_OFFSET		( 0x100 / sizeof(long) )
#define SEMAPHORE1_STATUS_OFFSET	( 0x40 / sizeof(long) )
#define SPL_MAX_LOCK_TRY		1000

static void IPC_ssp_lock(void)
{
	UINT32 lock_val = PLATFORM_my_ipc_id + 1;
	UINT32 read_semaphore;
	UINT32 try_cnt = 0;

	ASSERT((lock_val > 0) && (lock_val <= PLATFORM_MAX_NUM_OF_NODES));
	do
	{
		outvl(semaphore + SEMAPHORE0_SET_OFFSET , lock_val);
		read_semaphore = invl(semaphore + SEMAPHORE0_SET_OFFSET);
		if(lock_val !=  (read_semaphore & 0xff))
		{
			udelay(1);
		}
		ASSERT(try_cnt++ < SPL_MAX_LOCK_TRY);
	} while ((read_semaphore & 0xff) != lock_val);
}

static void IPC_ssp_unlock(void)
{
        UINT32 lock_val = PLATFORM_my_ipc_id + 1;
        UINT32 read_semaphore;

        read_semaphore = invl(semaphore + SEMAPHORE0_SET_OFFSET);
        ASSERT(read_semaphore == lock_val);
        outvl(semaphore + SEMAPHORE0_RESET_OFFSET , lock_val);
}


/*
 * -----------------------------------------------------------
 * MACRO (define) section
 * -----------------------------------------------------------
 */
//	Node
// TODO: Set relevant address for each node
//#define IPC_DUMMY_ADDR		(0)

#define	IPC_NPU_ARM0_ADDR	(NPU_IPC3)
#define	IPC_NPU_ARM1_ADDR 	(NPU_IPC3)
#define IPC_NPU_ARM2_ADDR	(NPU_IPC4)

/* Vladik, 28.06.2011 */
#define IPC_NPU_ARM3_ADDR	(NPU_IPC4) // FIXME: original code is IPC_DUMMY_ADDR


#define IPC_NPU_CPU0_ADDR	(NPU_IPC0)
#define IPC_NPU_CPU1_ADDR	(NPU_IPC0)
#define IPC_NPU_CPU2_ADDR	(NPU_IPC1)
#define IPC_NPU_CPU3_ADDR	(NPU_IPC1)
#define IPC_NPU_CPU4_ADDR	(NPU_IPC2)
#define IPC_NPU_CPU5_ADDR	(NPU_IPC2)
#define IPC_NPU_CPU6_ADDR	(IPC_DUMMY_ADDR)
#define IPC_NPU_RXP_ADDR	(IPC_DUMMY_ADDR)
#define IPC_TX_CPU0_ADDR	(IPC_DUMMY_ADDR)
#define IPC_TX_CPU1_ADDR	(IPC_TX3_ADDR)
#define IPC_TX_CPU2_ADDR	(IPC_TX1_ADDR)
#define IPC_TX_CPU3_ADDR	(IPC_TX1_ADDR)
#define IPC_TX_DSP0_ADDR	(IPC_TX2_ADDR)
#define IPC_TX_DSP1_ADDR	(IPC_DUMMY_ADDR)
#define IPC_RX_CPU0_ADDR	(RX_IPC0_PM)
#define IPC_RX_CPU1_ADDR	(RX_IPC0_PM)
#define IPC_RX_CPU2_ADDR	(IPC_DUMMY_ADDR)
#define IPC_RX_CPU3_ADDR	(IPC_DUMMY_ADDR)
#define IPC_RX_DSP0_ADDR	(RX_IPC1_PM)
#define IPC_RX_DSP1_ADDR	(RX_IPC1_PM)
#define IPC_RX_DSP2_ADDR	(RX_IPC2_PM)
#define IPC_RX_DSP3_ADDR	(RX_IPC2_PM)


/*
NPU_ARM0 0 0 0 0 0 0 0x1A0 416
NPU_ARM1 0 0 0 0 1 1 0x1A1 417 
NPU_ARM2 0 0 0 1 0 2 0x1A2 418 
NPU_ARM3 0 0 0 1 1 3 0x1A3 419 
NPU_CPU0 0 1 0 0 0 8 0x1C0 448 
NPU_CPU1 0 1 0 0 1 9 0x1C1 449 
NPU_CPU2 0 1 0 1 0 10 0x1C2 450 
NPU_CPU3 0 1 0 1 1 11 0x1C3 451 
NPU_CPU4 0 1 1 0 0 12 0x1C4 452 
NPU_CPU5 0 1 1 0 1 13 0x1C5 453 
NPU_CPU6 0 1 1 1 0 14 0x1C6 454 
NPU_RXP 0 1 1 1 1 15 0x1C7 455 
TX_CPU0 1 0 0 0 0 16 0x2C0 704 
TX_CPU1 1 0 0 0 1 17 0x2C1 705 
TX_CPU2 1 0 0 1 0 18 0x2C2 706 
TX_CPU3 1 0 0 1 1 19 0x2C3 707 
TX_DSP0 1 0 1 0 0 20 0x2D0 720 
TX_DSP1 1 0 1 0 1 21 0x2D1 721 
RX_CPU0 1 1 0 0 0 24 0x3C0 960 
RX_CPU1 1 1 0 0 1 25 0x3C1 961 
RX_CPU2 1 1 0 1 0 26 0x3C2 962 
RX_CPU3 1 1 0 1 1 27 0x3C3 963 
RX_DSP0 1 1 1 0 0 28 0x3D0 976 
RX_DSP1 1 1 1 0 1 29 0x3D1 977 
RX_DSP2 1 1 1 1 0 30 0x3D2 978 
RX_DSP3 1 1 1 1 1 31 0x3D3 979 
*/

#define IPC_FIFO_RD_IN_HIGH_ADDR(_cpuid)	(((_cpuid)&1) ?					\
	((UINT32)IPC_array_hw_access[(_cpuid)] + dan_ipc_if_FIFO_RD_ACCESS_4_OFFSET):	\
	((UINT32)IPC_array_hw_access[(_cpuid)] + dan_ipc_if_FIFO_RD_ACCESS_0_OFFSET))

#define IPC_FIFO_RD_OUT_HIGH_ADDR(_cpuid)	(((_cpuid)&1) ?					\
	((UINT32)IPC_array_hw_access[(_cpuid)] + dan_ipc_if_FIFO_RD_ACCESS_5_OFFSET):	\
	((UINT32)IPC_array_hw_access[(_cpuid)] + dan_ipc_if_FIFO_RD_ACCESS_1_OFFSET))

#define IPC_FIFO_RD_IN_LOW_ADDR(_cpuid)	(((_cpuid)&1) ?						\
	((UINT32)IPC_array_hw_access[(_cpuid)] + dan_ipc_if_FIFO_RD_ACCESS_6_OFFSET):	\
	((UINT32)IPC_array_hw_access[(_cpuid)] + dan_ipc_if_FIFO_RD_ACCESS_2_OFFSET))

#define IPC_FIFO_RD_OUT_LOW_ADDR(_cpuid)	(((_cpuid)&1) ?					\
	((UINT32)IPC_array_hw_access[(_cpuid)] + dan_ipc_if_FIFO_RD_ACCESS_7_OFFSET):	\
	((UINT32)IPC_array_hw_access[(_cpuid)] + dan_ipc_if_FIFO_RD_ACCESS_3_OFFSET))

#define IPC_FIFO_WR_IN_HIGH_ADDR(_cpuid)	(((_cpuid)&1) ?					\
	((UINT32)IPC_array_hw_access[(_cpuid)] + dan_ipc_if_FIFO_WR_ACCESS_4_OFFSET):	\
	((UINT32)IPC_array_hw_access[(_cpuid)] + dan_ipc_if_FIFO_WR_ACCESS_0_OFFSET))

#define IPC_FIFO_WR_OUT_HIGH_ADDR(_cpuid)	(((_cpuid)&1) ?					\
	((UINT32)(IPC_array_hw_access[(_cpuid)]) + dan_ipc_if_FIFO_WR_ACCESS_5_OFFSET):	\
	((UINT32)(IPC_array_hw_access[(_cpuid)]) + dan_ipc_if_FIFO_WR_ACCESS_1_OFFSET))

#define IPC_FIFO_WR_IN_LOW_ADDR(_cpuid)	(((_cpuid)&1) ?						\
	((UINT32)IPC_array_hw_access[(_cpuid)] + dan_ipc_if_FIFO_WR_ACCESS_6_OFFSET):	\
	((UINT32)IPC_array_hw_access[(_cpuid)] + dan_ipc_if_FIFO_WR_ACCESS_2_OFFSET))

#define IPC_FIFO_WR_OUT_LOW_ADDR(_cpuid)	(((_cpuid)&1) ?					\
	((UINT32)IPC_array_hw_access[(_cpuid)] + dan_ipc_if_FIFO_WR_ACCESS_7_OFFSET):	\
	((UINT32)IPC_array_hw_access[(_cpuid)] + dan_ipc_if_FIFO_WR_ACCESS_3_OFFSET))

#define IPC_FIFO_STAT_IN_HIGH_ADDR(_cpuid)	(((_cpuid)&1) ?					\
	((UINT32)IPC_array_hw_access[(_cpuid)] + dan_ipc_if_FIFO_4_STATUS_OFFSET):		\
	((UINT32)IPC_array_hw_access[(_cpuid)] + dan_ipc_if_FIFO_0_STATUS_OFFSET))

#define IPC_FIFO_STAT_OUT_HIGH_ADDR(_cpuid)	(((_cpuid)&1) ?					\
	((UINT32)IPC_array_hw_access[(_cpuid)] + dan_ipc_if_FIFO_5_STATUS_OFFSET):		\
	((UINT32)IPC_array_hw_access[(_cpuid)] + dan_ipc_if_FIFO_1_STATUS_OFFSET))

#define IPC_FIFO_STAT_IN_LOW_ADDR(_cpuid)	(((_cpuid)&1) ?					\
	((UINT32)IPC_array_hw_access[(_cpuid)] + dan_ipc_if_FIFO_6_STATUS_OFFSET):		\
	((UINT32)IPC_array_hw_access[(_cpuid)] + dan_ipc_if_FIFO_2_STATUS_OFFSET))

#define IPC_FIFO_STAT_OUT_LOW_ADDR(_cpuid)	(((_cpuid)&1) ?					\
	((UINT32)IPC_array_hw_access[(_cpuid)] + dan_ipc_if_FIFO_7_STATUS_OFFSET):		\
	((UINT32)IPC_array_hw_access[(_cpuid)] + dan_ipc_if_FIFO_3_STATUS_OFFSET))

#define AGENT_ID2CPU_ID(_cpuid)	(IPC_OwnNode)


#define IPC_FIFO_IRQ_OFFSET			((PLATFORM_my_ipc_id & 1) ? 4 : 0)

#define IPC_FIFO_RD_IN_LOW_IRQ_MASK(_cpuid)	(((_cpuid)&1) ?			\
	(2<<(6*4)):	(2<<(2*4))) /* bit_empty<<fifo_num*4_bits */

#define IPC_FIFO_RD_IN_HIGH_IRQ_MASK(_cpuid)	(((_cpuid)&1) ?		\
	(2<<(4*4)):	(2<<(0*4))) /* bit_empty<<fifo_num*4_bits */

#define IPC_LOCK(_cpuid)	IPC_ssp_lock() // SPINLOCK_lock(spinlock_array[(_cpuid)])
#define IPC_UNLOCK(_cpuid)	IPC_ssp_unlock() // SPINLOCK_unlock(spinlock_array[(_cpuid)])


/*
 * -----------------------------------------------------------
 * Type definition section
 * -----------------------------------------------------------
 */

/* Status mask of ITC FIFO */
typedef enum
{
	ITC_e_fifo_stat_empty	= 0x00000001,
	ITC_e_fifo_stat_full	= 0x00000010,
	ITC_e_fifo_stat_error	= 0x00000020
} ITC_e_fifo_stat_mask;

/*
 * -----------------------------------------------------------
 * Global data section
 * -----------------------------------------------------------
 */
//DECL_GLOBMEM(IPC_RX_BUF);
//DECL_GLOBMEM(IPC_NPU_BUF);
//VOID * IPC_fifo_mq;

/* Linux always runs on ARM3 */
//UINT8	PLATFORM_my_ipc_id = NPU_ARM_3;

//#define IPC_OwnNode	PLATFORM_my_ipc_id

/*
 * -----------------------------------------------------------
 * Type definition section
 * -----------------------------------------------------------
 */

const uint32_t IPC_array_hw_access_phys[PLATFORM_MAX_NUM_OF_NODES] =
{
	IPC_NPU_ARM0_ADDR,	// 0
	IPC_NPU_ARM1_ADDR,	// 1
	IPC_NPU_ARM2_ADDR,	// 2
	IPC_NPU_ARM3_ADDR,	// 3
	IPC_DUMMY_ADDR,		// 4
	IPC_DUMMY_ADDR,		// 5
	IPC_DUMMY_ADDR,		// 6
	IPC_DUMMY_ADDR,		// 7
	IPC_NPU_CPU0_ADDR,	// 8
	IPC_NPU_CPU1_ADDR,	// 9
	IPC_NPU_CPU2_ADDR,	// 10
	IPC_NPU_CPU3_ADDR,	// 11
	IPC_NPU_CPU4_ADDR,	// 12
	IPC_NPU_CPU5_ADDR,	// 13
	IPC_DUMMY_ADDR,		// 14
	IPC_DUMMY_ADDR,		// 15
	IPC_TX_CPU0_ADDR,	// 16
	IPC_TX_CPU1_ADDR,	// 17
	IPC_TX_CPU2_ADDR,	// 18
	IPC_TX_CPU3_ADDR,	// 19
	IPC_TX_DSP0_ADDR,	// 20
	IPC_TX_DSP1_ADDR,	// 21
	IPC_DUMMY_ADDR,		// 22
	IPC_DUMMY_ADDR,		// 23
	IPC_RX_CPU0_ADDR,	// 24
	IPC_RX_CPU1_ADDR,	// 25
	IPC_RX_CPU2_ADDR,	// 26
	IPC_RX_CPU3_ADDR,	// 27
	IPC_RX_DSP0_ADDR,	// 28
	IPC_RX_DSP1_ADDR,	// 29
	IPC_RX_DSP2_ADDR,	// 30
	IPC_RX_DSP3_ADDR	// 31
};

/* Remapped addresses from IPC_array_hw_access_phys */
void *IPC_array_hw_access[PLATFORM_MAX_NUM_OF_NODES];

const UINT32 IPC_array_irq_num[PLATFORM_MAX_NUM_OF_NODES] =
{
	IPC_DUMMY_ADDR,		// 0
	IPC_DUMMY_ADDR,		// 1
	IPC_DUMMY_ADDR,		// 2
	IPC_DUMMY_ADDR,		// 3
	IPC_DUMMY_ADDR,		// 4
	IPC_DUMMY_ADDR,		// 5
	IPC_DUMMY_ADDR,		// 6
	IPC_DUMMY_ADDR,		// 7
    E_SYS_INT_NPU_DAN_IPC0_CDU_IRQ0,	/* 164, 8*/
    E_SYS_INT_NPU_DAN_IPC0_CDU_IRQ1,	/* 164, 9 */
    E_SYS_INT_NPU_DAN_IPC1_CDU_IRQ0,	/* 162, 10 */
    E_SYS_INT_NPU_DAN_IPC1_CDU_IRQ1,	/* 162, 11 */
    E_SYS_INT_NPU_DAN_IPC2_CDU_IRQ0,	/* 160, 12 */
    E_SYS_INT_NPU_DAN_IPC2_CDU_IRQ1,	/* 160, 13 */
	IPC_DUMMY_ADDR,		// 14
	IPC_DUMMY_ADDR,		// 15
    IPC_DUMMY_ADDR,		// 16
    E_SYS_INT_TX_DAN_IPC2_CDU_IRQ0,                 /* 430, 17 */
    E_SYS_INT_TX_DAN_IPC0_CDU_IRQ0,                 /* 434, 18 */
    E_SYS_INT_TX_DAN_IPC0_CDU_IRQ1,                 /* 434, 19 */
    E_SYS_INT_TX_DAN_IPC1_CDU_IRQ1,                 /* 432, 20 */
    IPC_DUMMY_ADDR,		// 21
	IPC_DUMMY_ADDR,		// 22
	IPC_DUMMY_ADDR,		// 23
	IPC_RX_CPU0_ADDR,	// 24
    E_SYS_INT_RX_DAN_IPC0_CDU_IRQ0,                 /* 359, 25 */
    E_SYS_INT_RX_DAN_IPC0_CDU_IRQ1,                 /* 359, 26 */

    IPC_DUMMY_ADDR,		// 27
    E_SYS_INT_RX_DAN_IPC1_CDU_IRQ0,                 /* 357, 28 */
    E_SYS_INT_RX_DAN_IPC1_CDU_IRQ1,                 /* 357, 29 */
    E_SYS_INT_RX_DAN_IPC2_CDU_IRQ0,                 /* 355, 30 */
    E_SYS_INT_RX_DAN_IPC2_CDU_IRQ1,                 /* 355, 31 */
};

INT_ID IPC_irq_id;


/*
 * -----------------------------------------------------------
 * Global prototypes section
 * -----------------------------------------------------------
 */

//static UINT32 IPC_trns_fifo_get_mem_chunk(UINT8 cpu_id);


/* IPC_trns_fifo_buffer_alloc
 *
 * Transport layer buffer allocation API
 * use to allocate buffer when message is to be sent
 * from an agent on the phy to another agent on the
 * phy (i.e. using fifo based transport)
 *
*/
volatile char * CODE_DEFAULT IPC_trns_fifo_buffer_alloc
(
		uint8_t					dest_agent_id,
		IPC_trns_priority_e		pri
)
{
	UINT32	buff_addr;
	UINT32	fifo_addr;
	UINT8	cpu_id = IPC_GetNode(dest_agent_id);

	if (pri == IPC_trns_prio_e_0)
	{
		fifo_addr = IPC_FIFO_RD_OUT_LOW_ADDR(cpu_id);
	}
	else
	{
		fifo_addr = IPC_FIFO_RD_OUT_HIGH_ADDR(cpu_id);
	}
	IPC_LOCK(cpu_id);
	buff_addr = invl(fifo_addr);
	IPC_UNLOCK(cpu_id);

	// Fetch ptr from dest agent Buf FIFO
	// if not available return NULL
	//
#ifdef IPC_DBG_CF
	OS_printf_debug("Allocate buffer 0x%08X on Agent's %02x %u Node %u fifo_addr=0x%08X\n",
			buff_addr, dest_agent_id, dest_agent_id,  cpu_id, fifo_addr);
#endif /* IPC_DBG_CF */
	return  (volatile char *) ((buff_addr) ? ipc_to_virt(cpu_id, pri, buff_addr) : 0);

}



/* ipc_fifo_buffer_free:
 *
*  Transport layer buffer free API
 * use to free buffer when message is receievd
 * from an agent on the phy to another agent on the
 * phy (i.e. using fifo based transport)
 *
*/
void CODE_DEFAULT IPC_trns_fifo_buffer_free(VCHAR *ptr, UINT8 dest_agent_id, IPC_trns_priority_e pri)
{
	UINT32	fifo_addr;
	UINT8	cpu_id = IPC_GetNode(dest_agent_id);

	if (pri == IPC_trns_prio_e_0)
		fifo_addr = IPC_FIFO_WR_OUT_LOW_ADDR(cpu_id);
	else
		fifo_addr = IPC_FIFO_WR_OUT_HIGH_ADDR(cpu_id);

#ifdef IPC_DBG_CF
	OS_printf_debug("[%s]-%u cpu: %u dest_agent_id %u ptr 0x%08X fifo_addr 0x%08X\n",
			__FUNCTION__, __LINE__, cpu_id, dest_agent_id, (unsigned)ptr, fifo_addr);
#endif /* IPC_DBG_CF */

	ASSERT(ptr);
	IPC_LOCK(cpu_id);
	outvl(fifo_addr,virt_to_ipc(cpu_id, pri, (void *)ptr));
	IPC_UNLOCK(cpu_id);
}

/* IPC_trns_fifo_msg_send:
 *
*  Transport layer message sent API
 * use to send message when message is to be sent
 * from an agent on the phy to another agent on the
 * phy (i.e. using fifo based transport)
 *
*/
int32_t CODE_DEFAULT IPC_trns_fifo_buf_send(VCHAR *ptr, UINT8 destId, IPC_trns_priority_e pri)
{
	UINT8	cpu_id = IPC_GetNode(destId);
	UINT32	fifo_addr;
	
	if (pri == IPC_trns_prio_e_0)
		fifo_addr = IPC_FIFO_WR_IN_LOW_ADDR(cpu_id);
	else
		fifo_addr = IPC_FIFO_WR_IN_HIGH_ADDR(cpu_id);

	// Fetch ptr from dest agent Buf FIFO
	// if not available return NULL
	//
#ifdef IPC_DBG_CF
	OS_printf_debug("[%s]-%u Transmit buffer 0x%08X on destId %02X (%u) cpu_id %u FIFO 0x%08X\n",
			__FUNCTION__, __LINE__, (unsigned)ptr, destId, destId,  cpu_id, fifo_addr);
#endif /* IPC_DBG_CF */
	IPC_LOCK(cpu_id);
	outvl(fifo_addr,virt_to_ipc(cpu_id, pri, (void *)ptr));
	IPC_UNLOCK(cpu_id);

	return 0;
}


/* IPC_trns_fifo2eth_buffer_alloc:
 *
 * Transport layer buffer allocation API
 * use to allocate buffer when message is to be sent
 * from an agent on the phy to another agent on the
 * phy (i.e. using fifo based transport)
 *
*/
volatile char * CODE_DEFAULT IPC_trns_fifo2eth_buffer_alloc
(
		UINT8				dest_agent_id,
		IPC_trns_priority_e	pri
)
{
	UINT32	buff_addr;
	UINT32	fifo_addr;
	UINT8	cpu_id = IPC_PROXY_ETH_NODE_ID;

	if (pri == IPC_trns_prio_e_0)
	{
		fifo_addr = IPC_FIFO_RD_OUT_LOW_ADDR(cpu_id);
	}
	else
	{
		fifo_addr = IPC_FIFO_RD_OUT_HIGH_ADDR(cpu_id);
	}
	IPC_LOCK(cpu_id);
	buff_addr = invl(fifo_addr);
	IPC_UNLOCK(cpu_id);
	// Fetch ptr from dest agent Buf FIFO
	// if not available return NULL
	//
#ifdef IPC_DBG_CF
	OS_printf_debug("Allocate buffer 0x%08X on Agent's %02x %u Node %u fifo_addr=0x%08X\n",
			buff_addr, dest_agent_id, dest_agent_id,  cpu_id, fifo_addr);
#endif /* IPC_DBG_CF */
	return  (volatile char *)buff_addr;
}

/* IPC_trns_fifo2eth_buffer_free:
 *
*  Transport layer buffer free API
 * use to free buffer when message is receievd
 * from an agent on the phy to another agent on the
 * phy (i.e. using fifo based transport)
 *
*/
void CODE_DEFAULT IPC_trns_fifo2eth_buffer_free(VCHAR *ptr, UINT8 dest_agent_id, IPC_trns_priority_e pri)
{
	UINT8 cpu_id = IPC_PROXY_ETH_NODE_ID;
	UINT32 fifo_addr;

	if (pri == IPC_trns_prio_e_0)
		fifo_addr = IPC_FIFO_WR_OUT_LOW_ADDR(cpu_id);
	else
		fifo_addr = IPC_FIFO_WR_OUT_HIGH_ADDR(cpu_id);

#ifdef IPC_DBG_CF
	OS_printf_debug("[%s]-%u dest_agent_id %u %08lx fifo_addr 0x%08X\n",
			__FUNCTION__, __LINE__, 	dest_agent_id, (long)ptr, fifo_addr);
#endif /* IPC_DBG_CF */

	ASSERT(ptr);
	IPC_LOCK(cpu_id);
	outvl(fifo_addr,(UINT32)ptr);
	IPC_UNLOCK(cpu_id);
}

/* IPC_trns_fifo_msg_send:
 *
 *  Transport layer message sent API
 * use to send message when message is to be sent
 * from an agent on the phy to an agent on the mac
 * (i.e. using fifo based transport to a predefined proxy)
 *
*/
int32_t CODE_DEFAULT IPC_trns_fifo2eth_buffer_send(VCHAR *ptr, UINT8 destId, IPC_trns_priority_e pri)
{
	UINT8	cpu_id = IPC_PROXY_ETH_NODE_ID;
	UINT32	fifo_addr;

	if (pri == IPC_trns_prio_e_0)
		fifo_addr = IPC_FIFO_WR_IN_LOW_ADDR(cpu_id);
	else
		fifo_addr = IPC_FIFO_WR_IN_HIGH_ADDR(cpu_id);

	// Fetch ptr from dest agent Buf FIFO
	// if not available return NULL
	//
#ifdef IPC_DBG_CF
	OS_printf_debug("[%s]-%u Transmit buffer 0x%08X on destId %02X (%u) cpu_id %u FIFO 0x%08X\n",
			__FUNCTION__, __LINE__, (unsigned)ptr, destId, destId,  cpu_id, fifo_addr);
#endif /* IPC_DBG_CF */
	IPC_LOCK(cpu_id);
	outvl(fifo_addr,((UINT32)ptr));
	IPC_UNLOCK(cpu_id);

	return 0;
}

/*
 * -----------------------------------------------------------
 * Function:	IPC_trns_fifo_buff_init
 * Description:	Initialize IPC buffer for current node
 * Input:		cpu_id:	node ID ()
 * Output:		None
 * -----------------------------------------------------------
 */
void CODE_DEFAULT IPC_trns_fifo_buff_init(uint8_t cpu_id)
{
	UINT8	ix;
	UINT32	fifo_addr;

//#if !defined(DSP) && !defined(CPU)
//#error "IPC: error, unknown target"
//#endif
	UINT32 buf_addr = virt_to_ipc(cpu_id, IPC_trns_prio_e_1, ipc_buffers);

	ASSERT(buf_addr);

	fifo_addr = IPC_FIFO_WR_OUT_HIGH_ADDR(cpu_id);

	for (ix=0; ix<IPC_FIFO_BUF_NUM_HIGH; ix++, buf_addr+=IPC_BUF_SIZE_MAX)
	{
		outvl(fifo_addr,buf_addr);
#ifdef IPC_DBG_CF
		OS_printf_debug("[%s]-%u HIGH: fifo_addr=0x%08X buf_addr 0x%08X cpu_id=%u\n",
				__FUNCTION__, __LINE__, fifo_addr, buf_addr, cpu_id );
#endif /* IPC_DBG_CF */
	}

	fifo_addr = IPC_FIFO_WR_OUT_LOW_ADDR(cpu_id);

	for (ix=0; ix<IPC_FIFO_BUF_NUM_LOW; ix++, buf_addr+=IPC_BUF_SIZE_MAX)
	{
		outvl(fifo_addr,buf_addr);
#ifdef IPC_DBG_CF
		OS_printf_debug("[%s]-%u LOW: fifo_addr=0x%08X buf_addr 0x%08X cpu_id=%u\n",
				__FUNCTION__, __LINE__, fifo_addr, buf_addr, cpu_id );
#endif /* IPC_DBG_CF */
	}
	
#ifdef IPC_DBG_CF
	OS_printf_debug("[%s]-%u cpu_id=%u fifo_addr=0x%08X buf_addr=0x%08X base=0x%p \n",
			__FUNCTION__, __LINE__,
			cpu_id, fifo_addr, buf_addr, IPC_array_hw_access[cpu_id]);
#endif /* IPC_DBG_CF */
}

/*
 * -----------------------------------------------------------
 * Function:	IPC_trns_phy_addr
 * Description:	Print
 * Input:		cpu_id:	node ID ()
 * Output:		None
 * -----------------------------------------------------------
 */
#if 0
void CODE_DEFAULT IPC_trns_fifo_addr(void)
{
	UINT32 ind;
//	UINT32 addr; 
	
	for (ind = 0; ind < PLATFORM_MAX_NUM_OF_NODES; ind++)
	{
		if (IPC_cfg_get_type(ind) != dan3400_e)
			continue;
//		cid = IPC_IDX1(cid_array[ind]);
		OS_printf_debug("[%s]-%u ind =%u RD_IN_HIGH =0x%08X\n",
				__FUNCTION__, __LINE__,	ind, IPC_FIFO_RD_IN_HIGH_ADDR(ind));
		OS_printf_debug("ind =%u RD_OUT_HIGH =0x%08X\n", ind, IPC_FIFO_RD_OUT_HIGH_ADDR(ind));
		OS_printf_debug("ind =%u RD_IN_LOW =0x%08X\n", ind, IPC_FIFO_RD_IN_LOW_ADDR(ind));
		OS_printf_debug("ind =%u RD_OUT_LOW =0x%08X\n", ind, IPC_FIFO_RD_OUT_LOW_ADDR(ind));
		
		OS_printf_debug("ind =%u WR_IN_HIGH =0x%08X\n", ind, IPC_FIFO_WR_IN_HIGH_ADDR(ind));
		OS_printf_debug("ind =%u WR_OUT_HIGH =0x%08X\n", ind, IPC_FIFO_WR_OUT_HIGH_ADDR(ind));
		OS_printf_debug("ind =%u WR_IN_LOW =0x%08X\n", ind, IPC_FIFO_WR_IN_LOW_ADDR(ind));
		OS_printf_debug("ind =%u WR_OUT_LOW =0x%08X\n", ind, IPC_FIFO_WR_OUT_LOW_ADDR(ind));
	}
}
#endif

/*
 * -----------------------------------------------------------
 * Function:	IPC_trns_fifo_buf_read
 * Description:	Get message from node associated FIFO
 * Input:		agentId: NOT USED, current node ID already detected
 * Output:		None
 * -----------------------------------------------------------
 */
char * CODE_DEFAULT IPC_trns_fifo_buf_read(UINT8 agentId, IPC_trns_priority_e pri)
{
	UINT32	fifo_addr;
	UINT32	buff_addr	= 0;
	UINT8	cpu_id 		= IPC_OwnNode;

	UNUSED(agentId);

	if (pri == IPC_trns_prio_e_0)
	{
		fifo_addr = IPC_FIFO_RD_IN_LOW_ADDR(cpu_id);
	}
	else
	{
		fifo_addr = IPC_FIFO_RD_IN_HIGH_ADDR(cpu_id);
	}
	IPC_LOCK(cpu_id);
	buff_addr = invl(fifo_addr);
	IPC_UNLOCK(cpu_id);

	// Fetch ptr from dest agent Buf FIFO
	// if not available return NULL
	//
#ifdef IPC_DBG_CF
	if (buff_addr)
		OS_printf_debug("[%s]-%u Recieve buffer 0x%x\n",
				__FUNCTION__, __LINE__,	buff_addr);
#endif /* IPC_DBG_CF */
	return  (CHAR *) ((buff_addr) ? ipc_to_virt(cpu_id, pri, buff_addr) : 0);
}

/*
 * -----------------------------------------------------------
 * Function:	IPC_trns_fifo_get_mem_chunk
 * Description:	Get memory chunk for current cpu buffer allocation
 * Input:		cpu_id: node ID
 * Output:		None
 * -----------------------------------------------------------
 */
#if 0
static UINT32 CODE_DEFAULT IPC_trns_fifo_get_mem_chunk(UINT8 cpu_id)
{
	/* TBD: buffer should be allocated from CB according cpu_id */
	switch(cpu_id)
	{
/* TX  */
		case TX_CPU_1 /* 17 0x2C1 */:	return (IPC_TX_BUF_START_ADDR+IPC_BUF_SIZE_PER_NODE*0);
		case TX_CPU_2 /* 18 0x2C2 */:	return (IPC_TX_BUF_START_ADDR+IPC_BUF_SIZE_PER_NODE*1);
		case TX_CPU_3 /* 19 0x2C3 */:	return (IPC_TX_BUF_START_ADDR+IPC_BUF_SIZE_PER_NODE*2);
		case TX_DSP_0 /* 20 0x2D0 */:	return (IPC_TX_BUF_START_ADDR+IPC_BUF_SIZE_PER_NODE*3);

/* RX */
		case RX_CPU_0 /* 24 0x3C0 */:	return (IPC_RX_BUF_START_ADDR+IPC_BUF_SIZE_PER_NODE*0);
		case RX_CPU_1 /* 25 0x3C1 */:	return (IPC_RX_BUF_START_ADDR+IPC_BUF_SIZE_PER_NODE*1);
		case RX_DSP_0 /* 28 0x3D0 */:	return (IPC_RX_BUF_START_ADDR+IPC_BUF_SIZE_PER_NODE*2);
		case RX_DSP_1 /* 29 0x3D1 */:	return (IPC_RX_BUF_START_ADDR+IPC_BUF_SIZE_PER_NODE*3);
		case RX_DSP_2 /* 30 0x3D2 */:	return (IPC_RX_BUF_START_ADDR+IPC_BUF_SIZE_PER_NODE*4);
		case RX_DSP_3 /* 31 0x3D3 */:	return (IPC_RX_BUF_START_ADDR+IPC_BUF_SIZE_PER_NODE*5);
/* NPU */

		case NPU_CPU_0 /* 8 0x1C0 */:	return (IPC_NPU_BUF_START_ADDR+IPC_BUF_SIZE_PER_NODE*0);
		case NPU_CPU_1 /* 9 0x1C1 */:	return (IPC_NPU_BUF_START_ADDR+IPC_BUF_SIZE_PER_NODE*1);
		case NPU_CPU_2 /* 10 0x1C2 */:	return (IPC_NPU_BUF_START_ADDR+IPC_BUF_SIZE_PER_NODE*2);
		case NPU_CPU_3 /* 11 0x1C3 */:	return (IPC_NPU_BUF_START_ADDR+IPC_BUF_SIZE_PER_NODE*3);
		case NPU_CPU_4 /* 12 0x1C4 */:	return (IPC_NPU_BUF_START_ADDR+IPC_BUF_SIZE_PER_NODE*4);
		case NPU_CPU_5 /* 13 0x1C5 */:	return (IPC_NPU_BUF_START_ADDR+IPC_BUF_SIZE_PER_NODE*5);

		default:		ASSERT(FALSE);
	}

	return 0;
}
#endif

/*
 * -----------------------------------------------------------
 * Function:	IPC_cfg_irq
 * Description:	Configure IRQ
 * Input:		cpu_id: node ID
 * Output:		None
 * -----------------------------------------------------------
 */
#if 0
void CODE_DEFAULT IPC_fifo_irq_cb(UINT32 arg)
{
	UINT32 irq_mask		= IPC_FIFO_RD_IN_LOW_IRQ_MASK(PLATFORM_my_ipc_id);
	UINT32 irq_offset	= IPC_FIFO_IRQ_OFFSET;
	UINT32 baddr		= IPC_array_hw_access[PLATFORM_my_ipc_id];
	VCHAR *ipcData;

	OS_printf_debug("[%s]-%u IRQ cb node: %u  \n", __FUNCTION__, __LINE__,
			PLATFORM_my_ipc_id);

	while ((ipcData = IPC_trns_fifo_buf_read(IPC_OwnNode, IPC_trns_prio_e_1)))
	{
		ASSERT(OSA_mq_try_send(IPC_fifo_mq, ipcData));
	}

	outvl((baddr+dan_ipc_if_CDU_INT0_CLEAR_OFFSET+irq_offset), irq_mask);
    ICTL_EnableLocalInterrupts(ICTL_IdToMask(IPC_irq_id));
}

/*
 * -----------------------------------------------------------
 * Function:	IPC_fifo_irq_init
 * Description:	Configure IRQ
 * Input:		cpu_id: node ID
 * Output:		None
 * -----------------------------------------------------------
 */
UINT32 CODE_DEFAULT IPC_fifo_irq_init(void)
{
	IntDesc_t			IPC_irq_desc;
	UINT32 irq_num		= IPC_array_irq_num[PLATFORM_my_ipc_id];
	UINT32 irq_offset	= IPC_FIFO_IRQ_OFFSET;
	UINT32 irq_mask		= IPC_FIFO_RD_IN_LOW_IRQ_MASK(PLATFORM_my_ipc_id);
	UINT32 baddr		= IPC_array_hw_access[PLATFORM_my_ipc_id];
	UINT32 irq_stat		= IPC_array_hw_access[PLATFORM_my_ipc_id];

	ASSERT(irq_num);

	IPC_fifo_mq	= OSA_mq_create("IPC_MQ", IPC_FIFO_BUF_NUM_HIGH);

	outvl((baddr+dan_ipc_if_CDU_INT0_ENABLE_OFFSET+irq_offset), irq_mask);
	outvl((baddr+dan_ipc_if_CDU_INT0_CLEAR_OFFSET+irq_offset), irq_mask);
	irq_stat = invl(baddr+dan_ipc_if_CDU_INT0_STATUS_OFFSET+irq_offset);
	// No rised IRQ, on this stage
	ASSERT(!(irq_stat & irq_mask));

	OS_printf_debug("IPC IRQ Mask:  0x%p Enable: 0x%p Clear: 0x%p\n",
			(baddr+dan_ipc_if_CDU_INT0_MASK_OFFSET+irq_offset),
			(baddr+dan_ipc_if_CDU_INT0_ENABLE_OFFSET+irq_offset),
			(baddr+dan_ipc_if_CDU_INT0_CLEAR_OFFSET+irq_offset));

	IPC_irq_desc.Priority		= 0;
	IPC_irq_desc.Type			= INT_TYPE_IRQ;
	IPC_irq_desc.System_Int		= IPC_array_irq_num[PLATFORM_my_ipc_id];
	IPC_irq_desc.Callback_Func	= IPC_fifo_irq_cb;
	IPC_irq_desc.UserData		= 0;

	IPC_irq_id = ICTL_BindLocalInterrupt(&IPC_irq_desc);

	return 0;
}
#endif

