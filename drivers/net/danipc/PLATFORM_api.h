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


#ifndef _PLATFORM_API_H
#define _PLATFORM_API_H

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

#if !defined(UNIX) && !defined(WIN32)
//DECL_GLOBMEM(SYNC_ARRAY)
#endif

//#define PLATFORM_MAX_NUM_OF_NODES			(32)
#define PLATFORM_CACHE_LINE_SIZE			(32)

#define PLATFORM_DELAY_100_msec              (100000) // in Usec
#define PLATFORM_DELAY_1_msec                (1000) // in Usec


#ifdef __cplusplus
extern "C" {
#endif


/* Cache Operation Wrapper */

//#define CACHE_DISABLE_CF

#if defined(FPGA_ST3) || defined(CACHE_DISABLE_CF) || defined(SIMULATION)
#define PLATFORM_STARTUP_OFFSET 0
//Cache is disabled
/* dummy macro, Cache doesn't enabled for ST3 */
#define HAL_icache_region_invalidate(_a,_s)
#define HAL_dcache_region_invalidate(_a,_s)
#define HAL_icache_line_invalidate(_a)
#define HAL_dcache_line_invalidate(_a)
/* write dirty data back */
#define HAL_dcache_region_writeback(_a,_s)
#define HAL_dcache_line_writeback(_a)
/* write dirty data back and invalidate */
#define HAL_dcache_region_writeback_inv(_a,_s)
#define HAL_dcache_line_writeback_inv(_a)

#define PTR_NO_CACHE(_a)	PTR(_a)
#define UINT_NO_CACHE(_a)	((UINT32)(_a) | (1<<31))
#define PTR_CACHE(_a)		PTR(_a)
//Cache is disabled End
#else
//Cache is Enabled
/* invalidate the caches */
#define HAL_icache_region_invalidate(_a,_s)		\
							xthal_icache_region_invalidate(((void *)(_a)),(_s))
#define HAL_dcache_region_invalidate(_a,_s)	\
							xthal_dcache_region_invalidate((void *)(_a),(_s))
#define HAL_icache_line_invalidate(_a)			\
							xthal_icache_line_invalidate((void *)(_a))
#define HAL_dcache_line_invalidate(_a)			\
							xthal_dcache_line_invalidate((void *)(_a))

/* write dirty data back */
#define HAL_dcache_region_writeback(_a,_s)		\
							xthal_dcache_region_writeback(((void *)_a),(_s))
#define HAL_dcache_line_writeback(_a)			\
							xthal_dcache_line_writeback((void *)_a)

/* write dirty data back and invalidate */
#define HAL_dcache_region_writeback_inv(_a,_s)	\
							xthal_dcache_region_writeback_inv(((void *)_a),(_s))
#define HAL_dcache_line_writeback_inv(_a)		\
							xthal_dcache_line_writeback_inv((void *)_a)

#define PTR_NO_CACHE(_a)	PTR(PLATFORM_virt_2_phys((UINT32)(_a)) | (1<<31))
#define UINT_NO_CACHE(_a)	(PLATFORM_virt_2_phys((UINT32)(_a)) | (1<<31))

#define PTR_CACHE(_a)		PTR((UINT32)(_a) & ~(1<<31))
#define PTR_PHYS2VIRT_CACHE(_a)	PLATFORM_phys_2_virt(PTR_CACHE((_a)))
#define PTR_VIRT_CACHE(_a)	PTR_PHYS2VIRT_CACHE(PLATFORM_virt_2_phys((UINT32)(_a)))
#endif /* Cache conditions */

/*
 * -----------------------------------------------------------
 * Type definition section
 * -----------------------------------------------------------
 */

typedef union
{
	struct
	{
		UINT16 id		: 4;	/* node number 0:7							*/
		UINT16 type		: 4;	/* 0xA-ARM, 0xC-CPU, 0xD-DSP				*/
		UINT16 block	: 2;	/* 1-NPU, 2-TX, 3-RX						*/
		UINT16 pad		: 6;
	} bits;
	UINT16 node_id;
} PLATFORM_node_id_t;



/* Locality (TX,RX,NPU) */
typedef enum PLATFORM_locality_e
{
	PLATFORM_locality_NPU	= 1,
	PLATFORM_locality_TX	= 2,
	PLATFORM_locality_RX	= 3,
	PLATFORM_locality_LAST	= 4,
} PLATFORM_locality_t;

/* startup stages */
typedef enum
{
	PLATFROM_e_stage_idle		= 0,
	PLATFROM_e_stage_init,
	PLATFROM_e_stage_cust,
	PLATFROM_e_stage_pre_start,
	PLATFROM_e_stage_start,
	PLATFROM_e_stage_post_start,
	PLATFROM_e_stage_post_run,
	PLATFROM_e_stage_post_last,
} PLATFROM_e_startup_stage ;


/*
 * Available cores (CPUs) in the system.
 */
typedef enum PLATFORM_core_e
{
 	PLATFORM_CORE_START = 0,
 	NPU_ARM_0 = PLATFORM_CORE_START, /* 0 */
 	NPU_ARM_1 = 1,
 	NPU_ARM_2 = 2,
 	NPU_ARM_3 = 3,
	RES_4	  = 4,
	RES_5     = 5,
	RES_6     = 6,
	RES_7     = 7,
 	NPU_CPU_0 = 8,
 	NPU_CPU_1 = 9,
 	NPU_CPU_2 = 10,
 	NPU_CPU_3 = 11,
 	NPU_CPU_4 = 12,
 	NPU_CPU_5 = 13,
 	RES_14	 = 14,
 	RES_15	 = 15,
	TX_CPU_0 = 16,
	TX_CPU_1 = 17,
	TX_CPU_2 = 18,
	TX_CPU_3 = 19,
	TX_DSP_0 = 20,
	TX_DSP_1 = 21,
	RES_22   = 22,
	RES_23   = 23,
	RX_CPU_0 = 24,
	RX_CPU_1 = 25,
	RX_CPU_2 = 26,
	RX_CPU_3 = 27,
	RX_DSP_0 = 28,
	RX_DSP_1 = 29,
	RX_DSP_2 = 30,
	RX_DSP_3 = 31,
	PLATFORM_CORE_MAX = 32,
	PLATFORM_CORE_ALL = PLATFORM_CORE_MAX,
	//PLATFORM_CORE_NUM = PLATFORM_CORE_MAX
 } PLATFORM_core_t;

 /*
  * Available cores (CPUs) in the system.
  */
 typedef enum PLATFORM_node_status_e
 {
	 PLATFORM_node_status_e_IDLE		= 0x00,
	 PLATFORM_node_status_e_ASSERT		= 0x01,
	 PLATFORM_node_status_e_ALL			= 0x02,

 	//PLATFORM_CORE_NUM = PLATFORM_CORE_MAX
  } PLATFORM_node_status_t;

 /*
 * -----------------------------------------------------------
 * Static inline functions section
 * -----------------------------------------------------------
 */

#if !defined(WIN32)

static inline TBOOL PLATFORM_is_locality_NPU (UINT16 node_id)
{
	PLATFORM_node_id_t node_id_u;
	node_id_u.node_id = node_id;
	return (node_id_u.bits.block == PLATFORM_locality_NPU);
}

static inline TBOOL PLATFORM_is_locality_TX(UINT16 node_id)
{
	PLATFORM_node_id_t node_id_u;
	node_id_u.node_id = node_id;
	return (node_id_u.bits.block == PLATFORM_locality_TX);
}

static inline TBOOL PLATFORM_is_locality_RX(UINT16 node_id)
{
	PLATFORM_node_id_t node_id_u;
	node_id_u.node_id = node_id;
	return (node_id_u.bits.block == PLATFORM_locality_RX);
}

static inline TBOOL PLATFORM_is_CB_ptr(void *ptr)
{
	return (TRUE);
}

/*
 * -----------------------------------------------------------
 * Function:	PLATFORM_node_clear_status
 * Description:	Clear all status value for all nodes
 * Output:		none
 * -----------------------------------------------------------
 */
void PLATFORM_node_clear_status(void);

#endif // (WIN32)

/*
 * -----------------------------------------------------------
 * Global prototypes section
 * -----------------------------------------------------------
 */

/*  */
extern UINT16 PLATFORM_my_core_id;
extern PLATFORM_locality_t PLATFORM_my_locality;	/* RX, TX, NPU */
extern const char *PLATFORM_node_name[];
/* system clock in Hz */
extern UINT32 PLATFORM_system_clk;

/*
 * -----------------------------------------------------------
 * Function:	PLATFORM_get_my_ipc_id
 * Description:	Calculate IPC ID according, CORE ID for current Node
 * Input:		None
 * Output:		IPC core ID (0:31)
 * -----------------------------------------------------------
 */
UINT8 PLATFORM_get_my_ipc_id(void);

/*
 * -----------------------------------------------------------
 * Function:	PLATFORM_wait_startup_stage
 * Description:	Waiting when all active nodes of current locality will
 * 				complete desired stage during startup
 * Input:		stage - desired stage
 * Output:		BOOLEAN
 * -----------------------------------------------------------
 */
void PLATFORM_wait_startup_stage(PLATFROM_e_startup_stage stage);

/*
 * -----------------------------------------------------------
 * Function:	PLATFORM_sync_array_clear
 * Description:	Clean sync array, used during startup, call by booter
 * Input:		None
 * Output:		None
 * -----------------------------------------------------------
 */
void PLATFORM_sync_array_clear(void);

/*
 * -----------------------------------------------------------
 * Function:	PLATFORM_get_sync_array_addr
 * Description:	Get address of startup synchronization array
 * Input:		None
 * Output:		address in common buffer
 * -----------------------------------------------------------
 */
UINT32 PLATFORM_get_sync_array_addr(void);

/*
 * -----------------------------------------------------------
 * Function:	PLATFORM_node_status_array
 * Description:	Get address of nodes status array
 * Input:		None
 * Output:		address in common buffer
 * -----------------------------------------------------------
 */
UINT32 PLATFORM_get_node_status_array_addr(void);

void PLATFORM_node_status_set(UINT8 status);
void PLATFORM_node_status_reset(UINT8 status);
UINT8 PLATFORM_node_status_get(void);
TBOOL PLATFORM_node_status_check(UINT8 status);

/*
 * -----------------------------------------------------------
 * Function:	PLATFORM_get_node_active_mask
 * Description:	Get mask with active nodes for current locality
 * Input:		None
 * Output:		mask value
 * -----------------------------------------------------------
 */
UINT32 PLATFORM_get_node_active_mask(void);

/*
 * -----------------------------------------------------------
 * Function:	PLATFORM_nodes_check_assert
 * Description:	Check is anybody got ASSERT before reset
 * Input:		stage - desired stage
 * Output:		status word, each bit set to "1"  represent assert of N node 
 * -----------------------------------------------------------
 */
UINT32 PLATFORM_nodes_check_assert(void);

/*
 * -----------------------------------------------------------
 * Function:	PLATFORM_tick_delay
 * Description:	Simple delay in Node tick
 * Input:		None
 * Output:		address of common buffer
 * -----------------------------------------------------------
 */
void PLATFORM_tick_delay(UINT32 tick);

/*
 * -----------------------------------------------------------
 * Function:	PLATFORM_usec_delay
 * Description:	Simple delay in microseconds
 * Input:		usec - time to delay in microseconds
 * Output:		None
 * -----------------------------------------------------------
 */
void PLATFORM_usec_delay(UINT32 usec);


/*
 * -----------------------------------------------------------
 * Function:	PLATFORM_get_clk
 * Description:	Get system clock
 * Input:		stage - desired stage
 * Output:		current clk value
 * -----------------------------------------------------------
 */
UINT32 PLATFORM_get_clk(void);

/*
 * -----------------------------------------------------------
 * Function:	PLATFORM_virt_2_phys
 * Description:	Get physical address from the virtual
 * Input:		vaddr - virtual address
 * Output:		physical address
 * -----------------------------------------------------------
 */
UINT32 PLATFORM_virt_2_phys(UINT32 vaddr);

/*
 * -----------------------------------------------------------
 * Function:	PLATFORM_phys_2_virt
 * Description:	Get virtual address from the physical
 * Input:		paddr - physical address
 * Output:		virtual address
 * -----------------------------------------------------------
 */
UINT32 PLATFORM_phys_2_virt(UINT32 paddr);

#ifdef __cplusplus
}
#endif

#endif /*_PLATFORM_API_H*/

/*
 * -----------------------------------------------------------
 * End of file
 * -----------------------------------------------------------
 */
