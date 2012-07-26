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

#include <linux/string.h>

#pragma pack(4)
#include "I_sys_types.h"
#include "I_sys_utils.h"
//#if !defined(UNIX) && !defined(WIN32)
//#include "I_module_interface.h"
//#include "I_driver_interface.h"
//#include "I_kernel_interface.h"
//#include "modules_import.h"
//#include "drivers_import.h"
//#include "kernels_import.h"
//#else
//#include <string.h>
#include "PLATFORM_api.h"

#include "IPC_api.h"
#include "IPC_cfg.h"
//#endif

#include "danipc_lowlevel.h"

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
// Entry of Route-How table providing information how
// information is to pass when Source and destination Node location are
// known
typedef struct route_how_s
{
	UINT8 srcNodeLoc;
	UINT8 destNodeLoc;
	IPC_transport_func_t *utilVectorPtr;
} IPC_route_how_t;


/*
 * -----------------------------------------------------------
 * Global data section
 * -----------------------------------------------------------
 */

#undef IPC_AGENT_NAME

#define IPC_AGENT_NAME(_name)	#_name

const char * DATA_DEFAULT IPC_agent_name_array[] =
{
#include "IPC_agent_name.h"
};

#if defined(WIN32)
extern IPC_transport_func_t IPC_ETH_eth2proxy_utils;

const IPC_route_how_t routeHow[] =
{
		{dan3400_e, dan3400_e, 	&IPC_ETH_eth2proxy_utils},
		{dan3400_e,	extEth_e, 	&IPC_ETH_eth2proxy_utils}
};

#else

IPC_transport_func_t IPC_fifo_utils =
{
	IPC_trns_fifo_buffer_alloc,
	IPC_trns_fifo_buffer_free,
	IPC_trns_fifo_buf_send
};

IPC_transport_func_t IPC_fifo2eth_utils =
{
	IPC_trns_fifo2eth_buffer_alloc,
	IPC_trns_fifo2eth_buffer_free,
	IPC_trns_fifo2eth_buffer_send
};

IPC_transport_func_t IPC_proxy2eth_utils =
{
	NULL,
	NULL,
	NULL,
};

IPC_route_how_t routeHow[] =
{
	{dan3400_e,		dan3400_e,	&IPC_fifo_utils},
	{dan3400_e,		extEth_e,	&IPC_fifo2eth_utils},
	{dan3400_eth_e,	extEth_e,	&IPC_proxy2eth_utils}
};

#endif /* WIN32 */

// The static constant table contains information about the location
// of each node.
const IPC_node_type_e nodeAllocation[PLATFORM_MAX_NUM_OF_NODES] = {
		dan3400_e,		// Node #0
		dan3400_e,		// Node #1
		dan3400_e,		// Node #2
		dan3400_e,		// Node #3
		extEth_e,		// Node #4
		extFser_e,		// Node #5
		extUart_e,		// Node #6
		undef_e,		// Node #7
		dan3400_e,		// Node #8
		dan3400_e,		// Node #9
		dan3400_e,		// Node #10
		dan3400_e,		// Node #11
		dan3400_e,		// Node #12
		dan3400_e,		// Node #13
		dan3400_e,		// Node #14
		dan3400_e,		// Node #15
		dan3400_e,		// Node #16
		dan3400_e,		// Node #17
		dan3400_eth_e,	// Node #18
		dan3400_e,		// Node #19
		dan3400_e,		// Node #20
		dan3400_e,		// Node #21
		dan3400_e,		// Node #22
		undef_e,		// Node #23
		dan3400_e,		// Node #24
		dan3400_e,		// Node #25
		dan3400_e,		// Node #26
		dan3400_e,		// Node #27
		dan3400_e,		// Node #28
		dan3400_e,		// Node #29
		dan3400_e,		// Node #30
		dan3400_e,		// Node #31
};




UINT8 numOfRouteHows = sizeof(routeHow)/sizeof(IPC_route_how_t);

//Routing table is maintained globaly per 'board'
static IPC_transport_func_t *IPC_routing[PLATFORM_MAX_NUM_OF_NODES];

agentNameEntry_t *agentTable; 


/*
 * -----------------------------------------------------------
 * Local prototypes section
 * -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------
 * Global prototypes section
 * -----------------------------------------------------------
 */

/*
*===========================================================================
* IPC_getOwnNode
*===========================================================================
* Description:  Get Node ID for current node
*
* Parameters: none
*
* Returns: IPC node ID (0:31)
*
*/
uint8_t CODE_DEFAULT IPC_getOwnNode(void)
{
#if defined(WIN32)
	//#warning TBD: set proc_id to external module
	return (IPC_MAC_EXT_NODE_ID);
#else
    return (PLATFORM_my_ipc_id);
#endif
}


/*
*===========================================================================
* IPC_cfg_get_loc
*===========================================================================
* Description:  Fetch the node location based on the 
*               Node Id (5 lsbs)
*
* Parameters: nodeId
*
* Returns: Location Type
*
*/
IPC_node_type_e CODE_DEFAULT IPC_cfg_get_type (UINT8 nodeId)
{
	return nodeAllocation[nodeId & (PLATFORM_MAX_NUM_OF_NODES-1)];
}

/*
*===========================================================================
* IPC_cfg_get_util_vec
*===========================================================================
* Description:  Fetch the node util function vector  
*
* Parameters: srcNode - source Node Id
* 			  destNode - destination Node Id
*
* Returns: Location Type
*
*/
IPC_transport_func_t * CODE_DEFAULT IPC_cfg_get_util_vec (UINT8 srcNode, UINT8 destNode)
{
	int i;
	
	for (i=0; i < numOfRouteHows; i++)
	{
		if ((routeHow[i].srcNodeLoc == nodeAllocation[srcNode]) &&
			(routeHow[i].destNodeLoc == nodeAllocation[destNode])    )
		{
			return routeHow[i].utilVectorPtr;
		}
	}
	
	return NULL;
}

/*
*===========================================================================
* IPC_setAgentName
*===========================================================================
* Description:  Set agent name in Agent Table 
*
* Parameters: name - Agent Name
* 			  inx  - Index in table
*
* Returns: n/a
*
*/
void CODE_DEFAULT IPC_setAgentName (char *name, UINT8 inx)
{
	UINT32 len = strlen(name);
	if (len < MAX_AGENT_NAME_LEN)
		strcpy(agentTable[inx].agentName, name);
	else
		ASSERT(FALSE);

	if (len)
	{
#if !defined(UNIX) && !defined(WIN32)
		OS_printf_debug("NAME [%s] addr=0x%08X ind=%u\n", agentTable[inx].agentName,
				agentTable[inx].agentName, inx);
#endif
	}
}

/*
*===========================================================================
* IPC_getAgentName
*===========================================================================
* Description:  Get agent name from Agent Table 
*
* Parameters:   inx  - Index in table
*
* Returns: name
*
*/
char * CODE_DEFAULT IPC_getAgentName(UINT8 inx)
{
	return agentTable[inx].agentName;
}



void CODE_DEFAULT IPC_trns_eth_add(IPC_transport_func_t *trn_vec_p)
{

#if defined(UNIX) || defined(WIN32)
//
#else
	if (IPC_OwnNode == IPC_PROXY_ETH_NODE_ID)
	{
		ASSERT(trn_vec_p);
		memcpy ( &IPC_proxy2eth_utils, trn_vec_p,
				sizeof(IPC_transport_func_t));
	}
#endif

}


/*
*===========================================================================
* IPC_agent_table_clean
*===========================================================================
* Description:  Clear agent table, no registered agents on this stage
*
* Parameters:   inx  - Index in table
*
* Returns: name
*
*/
void CODE_DEFAULT IPC_agent_table_clean(void)
{
	int		len = (sizeof(agentNameEntry_t) * MAX_LOCAL_AGENT) / sizeof(uint32_t);
	uint32_t	*p = (uint32_t *)&agentTable[__IPC_AGENT_ID(PLATFORM_my_ipc_id, 0)];

	/* Clean only my part of the global table. This is necessary so I may
	 * register agents but do not hurt other cores.
	 * Vladik, 18.08,2011
	 */
	while (--len >= 0)
	{
		if (*p)
			*p = 0;
		p++;
	}
}

/*
*===========================================================================
* getUtilFuncVector
*===========================================================================
* Description:  Fetch the utility function's vector from the IPC
* 				routing table.
*
* Parameters: CPU-Id
*
* Returns: Pointer to function's pointers structure
*
*/
IPC_transport_func_t *IPC_getUtilFuncVector(UINT8 nodeId)
{
	ASSERT(nodeId < PLATFORM_MAX_NUM_OF_NODES)
	return IPC_routing[nodeId];
}

/*
*===========================================================================
* IPC_routeTableInit
*===========================================================================
* Description:  Initialize routing table .
*
* Parameters: pointer to default transport function
*
* Returns:
*
*   Note: The routing table
*   is unique per CPU as it maintains the information how to communicate
*   with any other CPU from this CPU.
*
*   This is just a preliminary implementation. Need to decide how each CPU
*   maintains the correct routing table
*/
void CODE_DEFAULT IPC_routeTableInit (IPC_transport_func_t *defTranVecPtr)
{
	int i;

#if !defined(UNIX) && !defined(WIN32)
	OS_printf_debug("[%s]-%u prix=%u\n",__FUNCTION__, __LINE__, IPC_OwnNode);
#endif
	//For every potential destination fill the associated function vector
	//or set to the node default transport functions
	for (i=0; i < PLATFORM_MAX_NUM_OF_NODES; i++)
	{
		IPC_routing[i] = (IPC_transport_func_t *)
				IPC_cfg_get_util_vec(IPC_OwnNode, i);
		if (IPC_routing[i] == NULL)
			IPC_routing[i] = defTranVecPtr;
	}
}

