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


#include <linux/string.h>

#pragma pack(4)
#include "I_sys_types.h"
#include "I_sys_utils.h"
//#if !defined(UNIX) && !defined(WIN32)
#include "I_phy_addr_space.h"
//#include "I_module_interface.h"
//#include "I_driver_interface.h"
//#include "I_kernel_interface.h"
//#include "modules_import.h"
//#include "drivers_import.h"
//#include "kernels_import.h"
//#else
//#include <string.h>
//#include "OSA_api.h"
//#include "PLATFORM_api.h"
//#endif
#include "IPC_reg.h"
#include "PLATFORM_api.h"
#include "ICTL_api.h"
#include "IPC_api.h"

#include "danipc_lowlevel.h"

//#include "IPC.h"

/*
 * -----------------------------------------------------------
 * MACRO (define) section
 * -----------------------------------------------------------
 */
/* max. number of local agents per one Node */
#define MAX_LOCAL_ID     (MAX_LOCAL_AGENT-1)

/*
 * -----------------------------------------------------------
 * Type definition section
 * -----------------------------------------------------------
 */
#if !defined(UNIX) && !defined(WIN32)
//#define IPC_USE_TRACE 1
//#define IPC_DBG_CF    1
#endif

#if defined(IPC_USE_TRACE)

#define IPC_TRACE_BUF_SIZE	0x30

static UINT8 IPC_trace_client_id;

typedef enum trace_e
{
	IPC_e_MSG_ALLOC,
	IPC_e_MSG_FREE,
	IPC_e_MSG_SEND,
	IPC_e_MSG_PROC,

} IPC_trace_id_t;

typedef enum trace_type_e
{
	IPC_e_MSG_INFO,
	IPC_e_MSG_WARN,
	IPC_e_MSG_ERR,

} IPC_trace_type_t;

START_TRACE_TABLE(IPC)
    TRACE_TABLE_ENTRY(IPC_e_MSG_ALLOC, IPC_e_MSG_INFO,"IPC ALLOC: 0x%p dst: %u src: %u type: 0x%02X")
    TRACE_TABLE_ENTRY(IPC_e_MSG_FREE, IPC_e_MSG_INFO, "IPC FREE: 0x%p dst: %u src: %u type: 0x%02X")
    TRACE_TABLE_ENTRY(IPC_e_MSG_SEND, IPC_e_MSG_INFO, "IPC SEND: 0x%p dst: %u src: %u type: 0x%02X")
    TRACE_TABLE_ENTRY(IPC_e_MSG_PROC, IPC_e_MSG_INFO, "IPC PROC: 0x%p dst: %u src: %u type: 0x%02X")
END_TRACE_TABLE


TRACE_record_t DATA_DEFAULT_ALIGN(32)	IPC_trace_rec_buf[IPC_TRACE_BUF_SIZE];

#endif

/*
 * -----------------------------------------------------------
 * Global prototypes section
 * -----------------------------------------------------------
 */


/*
 * -----------------------------------------------------------
 * Global data section
 * -----------------------------------------------------------
 */

//DEFINE_MODULE(IPC);

UINT8	ipc_localId; 	// Maintained the next available Local Id
                     	// for Agent registration
UINT8   ipc_reqSeqNume; // Maintain node related sequence number

IPC_agent_t	*IPC_agent_array[MAX_LOCAL_ID];
UINT8		IPC_agent_console_id;
UINT8		IPC_OwnNode;

/*
 * -----------------------------------------------------------
 * Local (static) data section
 * -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------
 * Local (static) and inline functions section
 * -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------
 * Function:	IPC_init
 * Description:	Called during node boot (initialization), assumes
 * 				non-interruptible execution, means that interrupts are disabled
 * 				and multi-thread support is not switched on yet
 * 				(if present at all on the node)
 * Input:		mem_start:			pointer to the start of memory block that
 * 				mem_size:			size of the provided memory block
 * 									module can use
 * Output:		Number of bytes allocated during the initialization
 * 				all the allocations
 * -----------------------------------------------------------
 */
unsigned CODE_DEFAULT IPC_init(void)
{
	IPC_agent_table_clean();
#if 0 // defined(UNIX) || defined(WIN32)
	IPC_appl_init(&IPC_eth2proxy_utils);
#else
	IPC_appl_init(&IPC_fifo_utils);
	return (0);
#endif
}

/*
 * -----------------------------------------------------------
 * Function:	IPC_pre_start
 * Description:	Called during node boot, assumes interruptible execution.
 * 				Pre-Start APIs of all modules are run before Start boot stage
 * Input:		None
 * Output:		None
 * -----------------------------------------------------------
 */
#if 0
static void CODE_DEFAULT IPC_pre_start(void)
{
	int status;
	status = IPC_Get_AgentId(IPC_GET_NAME(TX_CONSOLE), &IPC_agent_console_id);
	ASSERT(status == IPC_SUCCESS);

	IPC_fifo_irq_init();
}
#endif


/*
 * -----------------------------------------------------------
 * Global functions section
 * -----------------------------------------------------------
 */

/*
*===========================================================================
* IPC_init
*===========================================================================
* Description:  This function initializes software/HW during startup
*
* Parameters: defTranVecPtr - pointer to default transport layer
* 							  function vector
*
* Returns: n/a
*
*/
void CODE_DEFAULT IPC_appl_init(IPC_transport_func_t *defTranVecPtr)
{
	IPC_OwnNode = IPC_getOwnNode();
#if defined(UNIX) || defined(WIN32)
	IPC_agent_table_clean(); 
#else
	// fake registration of EXT_AGENT_0 should not be done on EXTERNAL Node
	IPC_fake_register(IPC_GET_NAME(EXT_AGENT_0), IPC_MAC_EXT_NODE_ID, 0);
#endif

	ipc_localId = 0;
	ipc_reqSeqNume = 0;

#if !defined(WIN32)
	IPC_trns_fifo_buff_init(IPC_OwnNode); // add macro GET_CPUID
#endif
	// Initialize IPC routing table (of CPU#)
	IPC_routeTableInit(defTranVecPtr);

#if defined(IPC_USE_TRACE)
	IPC_trace_client_id = TRACE_get_client_id((char *)"IPC");
	TRACE_register_client(IPC_trace_client_id, TRACE_TABLE(IPC),
							IPC_TRACE_BUF_SIZE, IPC_trace_rec_buf);
	OS_printf_debug ("TRACE client registered IPC_trace_client_id %d\n",
			IPC_trace_client_id);
#endif
}



/*
*===========================================================================
* IPC_register
*===========================================================================
* Description:  Registers IPC agent on the calling node, agent name and memory
* 				for agent data should be provided
*
* Parameters: 	name  - Agent unique name
* 				agent - Pointer to Agent structure
*
* Returns: Result code
*
*	Note: This is just a temporary implementation assuming all names
*	      and their corresponding AgentIds are pre-configured globally.
*/
int CODE_DEFAULT IPC_register(char *name, IPC_agent_t *agent)
{
	UINT8 myAgentId;
	UINT32 cpu_id = IPC_getOwnNode();
//	int	  res;

	ASSERT(agent!=NULL);
	agent->cb = NULL;
	ASSERT(ipc_localId < MAX_LOCAL_ID)
	myAgentId = IPC_AGENT_ID(cpu_id, (ipc_localId));

	if (strcmp(IPC_getAgentName(myAgentId), "") == 0) //Not register yet
	{
		IPC_setAgentName (name, myAgentId);
		strcpy(agent->agentName, name);
		agent->agentId = myAgentId;
		IPC_agent_array[ipc_localId] = agent;
		ipc_localId++; //For the next Agent registration in this node
	}

#ifdef IPC_DBG_CF
	OS_printf_debug("[%s]-%u [%s] cpu_id=%u myAgentId=%u (0x%X) ipc_localId=%u\n",
			__FUNCTION__, __LINE__, name, cpu_id, myAgentId, myAgentId,
			ipc_localId);
#endif /* IPC_DBG_CF */
	return IPC_SUCCESS;
}


/*
*===========================================================================
* IPC_fake_register
*===========================================================================
* Description:  Registers IPC agent on the calling node, agent name and memory
* 				for agent data should be provided, used for debugging
*
* Returns: Result code
*
*/
UINT8 CODE_DEFAULT IPC_fake_register(char *name, UINT32 cpu_id, UINT8 loc_id)
{
	UINT8 myAgentId;

	ASSERT(loc_id < MAX_LOCAL_ID)
	myAgentId = IPC_AGENT_ID(cpu_id, loc_id);
	ASSERT(name);
	IPC_setAgentName (name, myAgentId);
#ifdef IPC_DBG_CF
	OS_printf_debug("[%s]-%u [%s] cpu_id=%u myAgentId=%u (0x%X) ipc_localId=%u\n",
			__FUNCTION__, __LINE__, name, cpu_id, myAgentId, myAgentId,
			loc_id);
#endif /* IPC_DBG_CF */
	return myAgentId;
}




/*
*===========================================================================
* IPC_Get_AgentId
*===========================================================================
* Description:  Obtains Agent Id per given name
*
* Parameters: 	name  - Agent unique name
* 				agentIdPtr - (output) returns the AgentId associated
* 							 with the given name
*
* Returns: return Code
*
*/
int CODE_DEFAULT IPC_Get_AgentId(char *name, UINT8 *agentIdPtr)
{
	int i;

	if (agentIdPtr == NULL)
		return IPC_GENERIC_ERROR;

	//Search the static Agents Name/Id tale for the given name
	for (i=0;i<MAX_AGENTS;i++)
	{
		if (strcmp(IPC_getAgentName(i), name) == 0)
		{
			*agentIdPtr = i;
			return IPC_SUCCESS;
		}
	}
	return IPC_GENERIC_ERROR;
}


/*
*===========================================================================
* IPC_Get_AgentStruct
*===========================================================================
* Description:  Obtains data from Agent's IPC_agent_t structure per given
*				Agent ID
*
* Parameters: 	agentId   - Agent local ID
* 				agent_ptr - (output) pointer to preallocated IPC_agent_t
* 							 structure
*
* Returns: return Code
*
*/
int CODE_DEFAULT IPC_Get_AgentPtr (UINT8 agentId, IPC_agent_t** agent_ptr)
{
	int retVal = IPC_GENERIC_ERROR;
#ifdef IPC_DBG_CF
	OS_printf_debug ("IPC_Get_AgentPtr agentId %d\n", agentId);
#endif
	if ((agent_ptr != NULL) && (agentId < MAX_LOCAL_ID))
	{
		if (IPC_agent_array[agentId] != NULL)
		{
			*agent_ptr = IPC_agent_array[agentId];
			retVal = IPC_SUCCESS;
		}
		else
		{
			retVal = IPC_NO_AGENT_ERROR;
		}
	}
	return retVal;
}



/*
*===========================================================================
* IPC_callback_set
*===========================================================================
* Description:  Set the callback for specified agent, required for
* 				agent to receive any IPC messages
*
* Parameters: 	agent - Pointer to Agent structure
* 				cb	  - callback function
*
*
* Returns: Result code
*
*/
int CODE_DEFAULT IPC_callback_set(IPC_agent_t *agent, IPC_agent_cb_t cb)
{
	ASSERT(agent!=NULL);

	agent->cb = cb;
	return IPC_SUCCESS;
}

/*
*===========================================================================
* IPC_forward_callback_set
*===========================================================================
* Description:  Set the forward callback for specified agent, required for
* 				agent to forward IPC messages
*
* Parameters: 	agent - Pointer to Agent structure
* 				cb	  - callback function
*
*
* Returns: Result code
*
*/
int CODE_DEFAULT IPC_forward_callback_set(IPC_agent_t *agent, IPC_forward_cb_t cb)
{
	ASSERT(agent!=NULL);

	agent->forward_cb = cb;
	return IPC_SUCCESS;
}

/*
*===========================================================================
* IPC_buf_alloc
*===========================================================================
* Description:  buffer allocation API, should be called before building new message
*
* Parameters: 	agent 			- Pointer to Agent structure
* 				dest_agent_id	- Message destination AgentId
* 				pri				- Transport priority level
*
*
* Returns: Pointer to a 128 Byte buffer
*
*/
VCHAR * CODE_DEFAULT IPC_buf_alloc
(
		IPC_agent_t			*agent,
		UINT8				dest_agent_id,
		IPC_trns_priority_e	pri
)
{
	UINT8					cpuId;
	VCHAR					*ptr = NULL;
	IPC_transport_func_t	*funcPtr;
	IPC_trns_buf_alloc_t	allocFuncPtr;
	HAL_INTERRUPT_SAVE_AREA;

	ASSERT(agent!=NULL);

	// Allocate buffer of 128 Bytes using the allocation function
	// associated with the given destination agentId
	cpuId = IPC_GetNode(dest_agent_id);

	funcPtr = (void *)IPC_getUtilFuncVector(cpuId);
	ASSERT(funcPtr != NULL);

	allocFuncPtr = funcPtr->trns_buf_alloc_ptr;
	ASSERT(allocFuncPtr != NULL);
	HAL_INTERRUPT_DISABLE;
	ptr = allocFuncPtr(dest_agent_id, pri);
	HAL_INTERRUPT_RESTORE;
#ifdef IPC_DBG_CF
	OS_printf_debug("[%s]-%u Allocated Buffer in 0x%p\n",__FUNCTION__, __LINE__, ptr);
#endif /* IPC_DBG_CF */
	// Clear the 'Next buffer' filed
	if (ptr)
	{
		((volatile IPC_buffer_hdr_t *)ptr)->nextBufPtr = 0;
	}

	return ptr;
}

/*
*===========================================================================
* IPC_buf_free
*===========================================================================
* Description:  Free the buffer, could be called on IPC message receiving node
* 				or on sending node when need to free previously allocated buffers
*
* Parameters: 	agent 			- Pointer to Agent structure
* 				buf_first		- Pointer to first message buffer
* 				pri				- Transport priority level
*
*
* Returns: Result code
*
*/
INT32 CODE_DEFAULT IPC_buf_free(IPC_agent_t *agent, volatile char *buf_first, IPC_trns_priority_e pri)
{
	volatile IPC_next_buf_ptr_t	curBufferPtr;
	volatile IPC_next_buf_ptr_t	nxtBufferPtr;
	IPC_transport_func_t		*trns_ptr;
	IPC_trns_buf_free_t			freeFuncPtr;
	UINT8						destAgentId;
	UINT8						cpuId;
	INT32						res = IPC_SUCCESS;

	HAL_INTERRUPT_SAVE_AREA;

#ifdef IPC_DBG_CF
	OS_printf_debug("[%s]-%u buf_first=0x%p, %s\n", __FUNCTION__, __LINE__, buf_first,
				agent->agentName);
#endif

	ASSERT(agent!=NULL);

	if (buf_first == NULL)
		return IPC_SUCCESS;

	destAgentId 	= (((IPC_first_buffer_t *)buf_first)->msgHdr).destAgentId;
	curBufferPtr    = (IPC_next_buf_ptr_t) buf_first;

	cpuId = IPC_GetNode(destAgentId);
	trns_ptr = IPC_getUtilFuncVector(cpuId);
	ASSERT(trns_ptr != NULL);

	freeFuncPtr = trns_ptr->trns_buf_free_ptr;

	ASSERT(freeFuncPtr != NULL);
	// Now loop all allocated buffers and free them all
	// Last buffer is either a single buffer (type = 0)
	// or the buffer marked as the last one (type = 2)
	// all other buffers have their LSB set (type = 1 or 3)

#if defined(IPC_USE_TRACE)
	TRACE4(IPC_trace_client_id, IPC_e_MSG_FREE, (UINT32)curBufferPtr,
			(UINT32)((IPC_message_hdr_t *)curBufferPtr)->destAgentId,
			(UINT32)((IPC_message_hdr_t *)curBufferPtr)->srcAgentId,
			(UINT32)((IPC_message_hdr_t *)curBufferPtr)->msgType);
#endif

	HAL_INTERRUPT_DISABLE;
	do {
		nxtBufferPtr   = ((volatile IPC_message_hdr_t *)
				IPC_NEXT_PTR_PART(curBufferPtr))->nextBufPtr;
		(freeFuncPtr)(IPC_NEXT_PTR_PART(curBufferPtr), destAgentId, pri);
		curBufferPtr = nxtBufferPtr;
//		OS_printf_debug("Buffer serviced: next: 0x%08X\n", nxtBufferPtr);
	} while (((UINT32)curBufferPtr & IPC_BUF_TYPE_MTC) );
	HAL_INTERRUPT_RESTORE;
//	OS_printf_debug("All buffers serviced\n");

	return res;
}

/*
*===========================================================================
* IPC_buf_link
*===========================================================================
* Description:  Link two buffers, should be called when message does not fit the single buffer
*
* Parameters: 	agent 			- Pointer to Agent structure
* 				buf_prev		- Pointer to a message buffer
* 				buf_next		- Pointer to the next message buffer (to be linked to)
*
*
* Returns: Result code
*
*/
INT32 CODE_DEFAULT IPC_buf_link(IPC_agent_t *agent, VCHAR *buf_prev, VCHAR *buf_next)
{
	ASSERT(agent!=NULL);

	if (buf_prev == NULL || buf_next == NULL)
		return -1;

	//Set the next buffer pointer in place
	*(VUINT32 *)buf_prev |= (UINT32)buf_next & ~IPC_BUF_TYPE_BITS;
	// Set the LSB of the prev buffer to signal there are
	// more to come
	*(VUINT32 *)buf_prev |= IPC_BUF_TYPE_MTC;
	// Mark the next buffer as the last one
	*(VUINT32 *)buf_next |= IPC_BUF_TYPE_END;
	return IPC_SUCCESS;
}

/*
*===========================================================================
* IPC_msg_set_len
*===========================================================================
* Description:  sets message length, first buffer of the message should be provided
*
* Parameters: 	agent 			- Pointer to Agent structure
* 				buf_first		- Pointer to the first message buffer
* 				len				- Message length (bytes)
*
*
* Returns: Result code
*
*/
INT32 CODE_DEFAULT IPC_msg_set_len(IPC_agent_t *agent, VCHAR *buf_first, UINT16 len)
{
	ASSERT(agent!=NULL);

	if (buf_first == NULL)
		return IPC_GENERIC_ERROR;
	(((IPC_first_buffer_t *)buf_first)->msgHdr).msgLen = len;
	return IPC_SUCCESS;
}

/*
*===========================================================================
* IPC_msg_set_type
*===========================================================================
* Description:  sets message type, first buffer of the message should be provided
*
* Parameters: 	agent 			- Pointer to Agent structure
* 				buf_first		- Pointer to the first message buffer
* 				type			- Message type
*
*
* Returns: Result code
*
*/
INT32 CODE_DEFAULT IPC_msg_set_type(IPC_agent_t *agent, VCHAR *buf_first, UINT8 type)
{
	ASSERT(agent!=NULL);

	if (buf_first == NULL)
		return IPC_GENERIC_ERROR;
	(((IPC_first_buffer_t *)buf_first)->msgHdr).msgType = type;
	return IPC_SUCCESS;
}

/*
*===========================================================================
* IPC_msg_set_reply_ptr
*===========================================================================
* Description:  sets message reply buffer pointer
*
* Parameters: 	agent 			- Pointer to Agent structure
* 				buf_first		- Pointer to the first message buffer
* 				buf_rep			- Pointer to the expected replay message
*
*
* Returns: Result code
*
*/
INT32 CODE_DEFAULT IPC_msg_set_reply_ptr
(
	IPC_agent_t		*agent,
	VCHAR			*buf_first,
	VCHAR			*buf_rep
)
{
	ASSERT(agent!=NULL);

	if (buf_first == NULL)
		return IPC_GENERIC_ERROR;
	(((IPC_first_buffer_t *)buf_first)->msgHdr).ReplyMsgPtr = buf_rep;
	return IPC_SUCCESS;
}


/*
*===========================================================================
* IPC_msg_alloc
*===========================================================================
* Description:  Allocate message buffer[s] and set the type and length.
* 				Copy message data into allocated buffers.
*
*
* Parameters: 	agent 			- Pointer to Agent structure
* 				dest_agent_id	- Message destination AgentId
* 				msgPtr			- Pointer to message data
* 				msgLen			- Message length
* 				type			- Message type
* 				repPtr			- Pointer to allocated reply buffer
* 				pri				- Transport priority level
*
*
* Returns: Pointer to the message first buffer
*
*/
VCHAR * CODE_DEFAULT IPC_msg_alloc
(
	IPC_agent_t		*agent,
	UINT8			dest_agent_id,
	CHAR			*msgPtr,
	UINT16			msgLen,
	UINT8			msgType,
	VCHAR			*repPtr,
	IPC_trns_priority_e	pri
)
{
	VCHAR	*firstPtr = NULL;
	VCHAR	*prevPtr = NULL;
	VCHAR	*nextPtr = NULL;
	UINT16	numOfNextBufs = 0;
	UINT16	BufsCnt, tmpSize, dataReminder;
	CHAR	*lastData;

	ASSERT(agent!=NULL);

	if ((msgLen > IPC_MAX_MESSAGE_SIZE) || (msgLen == 0))
		return NULL;

	// Calculate number of 'next' buffers required
	// (i.e. buffers additional to the first buffer)
	if (msgLen > IPC_FIRST_BUF_DATA_SIZE_MAX)
	{
		numOfNextBufs = (msgLen - IPC_FIRST_BUF_DATA_SIZE_MAX)/IPC_NEXT_BUF_DATA_SIZE_MAX;
		if ((msgLen - IPC_FIRST_BUF_DATA_SIZE_MAX)%IPC_NEXT_BUF_DATA_SIZE_MAX)
			numOfNextBufs++;
	}

	firstPtr = prevPtr = IPC_buf_alloc(agent, dest_agent_id, pri);
#ifdef IPC_DBG_CF
	OS_printf_debug("[%s]-%u Allocated ptr=0x%p\n", __FUNCTION__, __LINE__, prevPtr);
#endif /* IPC_DBG_CF */
#ifdef IPC_DBG_CF
	OS_printf_debug("[%s]-%u Number of buf required = %d\n",
			__FUNCTION__, __LINE__, numOfNextBufs + 1);
#endif /* IPC_DBG_CF */
	for (BufsCnt = 0; BufsCnt < numOfNextBufs; BufsCnt++)
	{
		if (prevPtr == NULL)
		{
#if !defined(UNIX) && !defined(WIN32)
#pragma frequency_hint NEVER
#endif
			break;
		}
		nextPtr = IPC_buf_alloc(agent, dest_agent_id, pri);
		if (nextPtr != NULL)
		{
#if !defined(UNIX) && !defined(WIN32)
#pragma frequency_hint NEVER
#endif
			IPC_buf_link(agent, prevPtr, nextPtr);
		}
		prevPtr = nextPtr;
	}

	//If buffer allocation failed free the entire buffers
	if ((prevPtr == NULL) && (firstPtr != NULL))
	{
		IPC_buf_free(agent, firstPtr, pri);
		firstPtr = NULL;
	}
	else if (firstPtr)
	{
		IPC_msg_set_type(agent, firstPtr, msgType);
		IPC_msg_set_len(agent, firstPtr, msgLen);
		IPC_msg_set_reply_ptr (agent, firstPtr, repPtr);
		((IPC_message_hdr_t *)firstPtr)->destAgentId = dest_agent_id;
		((IPC_message_hdr_t *)firstPtr)->srcAgentId  = agent?(agent->agentId):0;
		((IPC_message_hdr_t *)firstPtr)->requestSeqNum = ipc_reqSeqNume;
		ipc_reqSeqNume++;

		if (msgPtr != NULL)
		{
			lastData = msgPtr + msgLen;

			//Now copy the Data
			dataReminder = msgLen;
			tmpSize = MIN(dataReminder, IPC_FIRST_BUF_DATA_SIZE_MAX);

			memcpy((((IPC_first_buffer_t *)firstPtr)->body), lastData - dataReminder, tmpSize);

			dataReminder -= tmpSize;
			prevPtr = firstPtr;

			while (dataReminder > 0)
			{
				nextPtr = IPC_NEXT_PTR_PART(((IPC_message_hdr_t *)prevPtr)->nextBufPtr);
				tmpSize = MIN(dataReminder, IPC_NEXT_BUF_DATA_SIZE_MAX);

				memcpy((((IPC_next_buffer_t *)nextPtr)->body), lastData - dataReminder, tmpSize);

				dataReminder -= tmpSize;
				prevPtr = nextPtr;
			}
		}
	}

#if defined(IPC_USE_TRACE)
	if (firstPtr)
	{
		TRACE4(IPC_trace_client_id, IPC_e_MSG_ALLOC, (UINT32)firstPtr,
				(UINT32)((IPC_message_hdr_t *)firstPtr)->destAgentId,
				(UINT32)((IPC_message_hdr_t *)firstPtr)->srcAgentId,
				(UINT32)((IPC_message_hdr_t *)firstPtr)->msgType);
	}
#endif

	return firstPtr;
}

/*
*===========================================================================
* IPC_msg_send
*===========================================================================
* Description:  Message send, first buffer of the message should be provided,
*
* Parameters: 	agent 			- Pointer to Agent structure
* 				buf_first		- Pointer to the first message buffer
* 				pri				- Transport priority level
*
*
* Returns: Result code
*
*/
INT32 CODE_DEFAULT IPC_msg_send(IPC_agent_t *agent, VCHAR *buf_first, IPC_trns_priority_e pri)
{
	volatile IPC_next_buf_ptr_t	curBufferPtr;
	IPC_transport_func_t		*trns_ptr;
	IPC_trns_buf_send_t			sendFuncPtr;
	UINT8						destAgentId;
	UINT8						cpuId;
	INT32						res = IPC_SUCCESS;
	HAL_INTERRUPT_SAVE_AREA;

	ASSERT(agent!=NULL);

	if (buf_first == NULL)
		return IPC_SUCCESS;

	destAgentId 	= (((IPC_first_buffer_t *)buf_first)->msgHdr).destAgentId;
	cpuId           = IPC_GetNode(destAgentId);
	curBufferPtr    = (IPC_next_buf_ptr_t) buf_first;

	trns_ptr = IPC_getUtilFuncVector(cpuId);
	ASSERT(trns_ptr != NULL);

	sendFuncPtr  = trns_ptr->trns_buf_send_ptr;
	ASSERT(sendFuncPtr != NULL);

#ifdef IPC_SEND_CF
	OS_printf_debug("[%s]-%d destAgentId = %d, \n",
			__FUNCTION__, __LINE__, destAgentId);
	curBufferPtr    = (IPC_next_buf_ptr_t)IPC_NEXT_PTR_PART(curBufferPtr);
#endif
	HAL_INTERRUPT_DISABLE;
	res = (sendFuncPtr)((VCHAR *)curBufferPtr, destAgentId, pri);
	HAL_INTERRUPT_RESTORE;

#if defined(IPC_USE_TRACE)
	TRACE4(IPC_trace_client_id, IPC_e_MSG_SEND, (UINT32)curBufferPtr,
			(UINT32)((IPC_message_hdr_t *)curBufferPtr)->destAgentId,
			(UINT32)((IPC_message_hdr_t *)curBufferPtr)->srcAgentId,
			(UINT32)((IPC_message_hdr_t *)curBufferPtr)->msgType);
#endif

	return res;
}

/*
*===========================================================================
* IPC_chunk_send
*===========================================================================
* Description:  Send separatly all chunks from the message
*
* Parameters: 	agent 			- Pointer to Agent structure
* 				buf_first		- Pointer to the first message buffer
* 				pri				- Transport priority level
*
*
* Returns: Result code
*
*/
INT32 CODE_DEFAULT IPC_chunk_send(IPC_agent_t *agent, VCHAR *buf_first, IPC_trns_priority_e pri)
{
	IPC_transport_func_t		*trns_ptr;
	IPC_trns_buf_send_t			sendFuncPtr;
	UINT8						destAgentId;
	UINT8						cpuId;
	INT32						res = IPC_SUCCESS;
	HAL_INTERRUPT_SAVE_AREA;

	ASSERT(agent);
	ASSERT(buf_first);

	destAgentId 	= (((IPC_first_buffer_t *)buf_first)->msgHdr).destAgentId;
	cpuId           = IPC_GetNode(destAgentId);

	ASSERT(trns_ptr = IPC_getUtilFuncVector(cpuId));
	ASSERT(sendFuncPtr = trns_ptr->trns_buf_send_ptr);

#ifdef IPC_SEND_CF
	OS_printf_debug("[%s]-%d destAgentId = %d, \n",
			__FUNCTION__, __LINE__, destAgentId);
	curBufferPtr    = (IPC_next_buf_ptr_t)IPC_NEXT_PTR_PART(curBufferPtr);
#endif

	HAL_INTERRUPT_DISABLE;
	res = (sendFuncPtr)((VCHAR *)buf_first, destAgentId, pri);
	HAL_INTERRUPT_RESTORE;
	return res;
}

/*
*===========================================================================
* IPC_buf_info
*===========================================================================
* Description:  Get the buffer Data part pointer and next buffer ptr
*
* Parameters: 	bufPtr  - Pointer to a buffer
* 				bodyPtr - Pointer to Data part Pointer (output parameter)
* 				bodyLen - Pointer to data part length (output parameter)
* 				bufType - Pointer to Buffer type bits (output parameter)
*
* Returns: Pointer to the nextBuf
*
*/
VCHAR * CODE_DEFAULT IPC_buf_info
(
	VCHAR	*bufPtr,
	CHAR	**bodyPtr,
	UINT8	*bodyLen,
	UINT32	*bufType
)
{
	IPC_next_buf_ptr_t nextBufferPtr;
	UINT32             bufferType;

	if (bufPtr == NULL)
	{
		if (bodyPtr != NULL)
			*bodyPtr = NULL;
		if (bodyLen != NULL)
			*bodyLen = 0;
		if (bufType != NULL)
			*bufType = 0;
		return NULL;
	}
	//Extract the 'next buffer' filed
	nextBufferPtr   = ((IPC_message_hdr_t *)bufPtr)->nextBufPtr;
	//Extract the buffer type from the 'next-buffer' field
	bufferType      = (UINT32)nextBufferPtr & IPC_BUF_TYPE_BITS;

	//Set the body ptr into the provided output parameter
	if (bufferType == IPC_BUF_TYPE_START || bufferType == IPC_BUF_TYPE_FULL)
	{
		if (bodyPtr != NULL)
			*bodyPtr = ((IPC_first_buffer_t *)bufPtr)->body;
		if (bodyLen != NULL)
			*bodyLen = IPC_FIRST_BUF_DATA_SIZE_MAX;
	}
	else
	{
		if (bodyPtr != NULL)
			*bodyPtr = ((IPC_next_buffer_t *)bufPtr)->body;
		if (bodyLen != NULL)
			*bodyLen = IPC_NEXT_BUF_DATA_SIZE_MAX;
	}
	if (bufType != NULL)
		*bufType = bufferType;
	return IPC_NEXT_PTR_PART(nextBufferPtr);
}

/*
*===========================================================================
* display_message
*===========================================================================
* Description:  Debug function to display message buffer content
*
* Parameters: buf_first - First message buffer
*
* Returns: n/a
*
*/
void CODE_DEFAULT display_message (char *buf_first)
{
#if !defined(UNIX) && !defined(WIN32)
	IPC_next_buf_ptr_t curBufferPtr;
	IPC_next_buf_ptr_t nextBufferPtr;
	UINT32             bufferType;

	if (buf_first == NULL)
		return ;

	curBufferPtr    = (IPC_next_buf_ptr_t) buf_first;
	OS_printf_debug("\nMessage Buffer:\n");
	OS_printf_debug("---------------\n");
	OS_printf_debug("Message (%lx):\n", (unsigned long)curBufferPtr);

	do {
		//Extract the 'next buffer' filed
		nextBufferPtr   = ((IPC_message_hdr_t *)curBufferPtr)->nextBufPtr;
		//Extract the buffer type from the 'next-buffer' field
		bufferType      = (UINT32)nextBufferPtr & IPC_BUF_TYPE_BITS;
		//Clear the type bits
		OS_printf_debug("     |\n");
		OS_printf_debug("     |\n");
		OS_printf_debug("     V\n");
		OS_printf_debug("[%07x|%01x ]\n", (int)IPC_NEXT_PTR_PART(nextBufferPtr), (int)bufferType);
		if (bufferType == IPC_BUF_TYPE_START || bufferType == IPC_BUF_TYPE_FULL)
		{
			//Display message header
			OS_printf_debug("[%02x|%02x|%04x ]\n",  (unsigned char)((IPC_message_hdr_t *)curBufferPtr)->requestSeqNum,
					(unsigned char)((IPC_message_hdr_t *)curBufferPtr)->replySeqNum,
					(unsigned short)((IPC_message_hdr_t *)curBufferPtr)->msgLen);
			OS_printf_debug("[%02x|%02x|%02x|%01d%01d]\n",  ((IPC_message_hdr_t *)curBufferPtr)->destAgentId,
					(unsigned char)((IPC_message_hdr_t *)curBufferPtr)->srcAgentId,
					(unsigned char)((IPC_message_hdr_t *)curBufferPtr)->msgType,
					(unsigned char)(((IPC_message_hdr_t *)curBufferPtr)->flags).reply,
					(unsigned char)(((IPC_message_hdr_t *)curBufferPtr)->flags).request);
			OS_printf_debug("[  %08lx ]\n",  (UINT32)((IPC_message_hdr_t *)curBufferPtr)->ReplyMsgPtr);

			OS_printf_debug("[%02x|%02x|%02x|%02x]\n", (unsigned char)((IPC_first_buffer_t *)curBufferPtr)->body[0],
					(unsigned char)((IPC_first_buffer_t *)curBufferPtr)->body[1],
					(unsigned char)((IPC_first_buffer_t *)curBufferPtr)->body[2],
					(unsigned char)((IPC_first_buffer_t *)curBufferPtr)->body[3]);
			OS_printf_debug("[   ...\t    ]\n");

		}
		else
		{
			OS_printf_debug("[%02x|%02x|%02x|%02x]\n", (unsigned char)((IPC_next_buffer_t *)curBufferPtr)->body[0],
					(unsigned char)((IPC_next_buffer_t *)curBufferPtr)->body[1],
					(unsigned char)((IPC_next_buffer_t *)curBufferPtr)->body[2],
					(unsigned char)((IPC_next_buffer_t *)curBufferPtr)->body[3]);
			OS_printf_debug("[   ...\t    ]\n");
		}
		curBufferPtr    = (IPC_next_buf_ptr_t)IPC_NEXT_PTR_PART(nextBufferPtr);
	} while (bufferType & IPC_BUF_TYPE_MTC);
#endif
}

/*
*===========================================================================
* display_header
*===========================================================================
* Description:  Debug function to display message header content
*
* Parameters: buf_first - First message buffer
*
* Returns: n/a
*
*/
void CODE_DEFAULT display_header (char *buf_first)
{
#if !defined(UNIX) && !defined(WIN32)
	IPC_next_buf_ptr_t curBufferPtr;
	IPC_next_buf_ptr_t nextBufferPtr;
	UINT32             bufferType;

	if (buf_first == NULL)
		return ;

	curBufferPtr    = (IPC_next_buf_ptr_t) buf_first;
	OS_printf_debug("\nMessage Buffer:\n");
	OS_printf_debug("---------------\n");
	OS_printf_debug("Message (%lx):\n", (unsigned long)curBufferPtr);

//	do {
		//Extract the 'next buffer' filed
		nextBufferPtr   = ((IPC_message_hdr_t *)curBufferPtr)->nextBufPtr;
		//Extract the buffer type from the 'next-buffer' field
		bufferType      = (UINT32)nextBufferPtr & IPC_BUF_TYPE_BITS;
		//Clear the type bits
		OS_printf_debug("     |\n");
		OS_printf_debug("     |\n");
		OS_printf_debug("     V\n");
		OS_printf_debug("[%07x|%01x  ]\n", (int)IPC_NEXT_PTR_PART(nextBufferPtr), (int)bufferType);
		if (bufferType == IPC_BUF_TYPE_START || bufferType == IPC_BUF_TYPE_FULL)
		{
			//Display message header
			OS_printf_debug("[%02x|%02x|%04x ]\n",  (unsigned char)((IPC_message_hdr_t *)curBufferPtr)->requestSeqNum,
					(unsigned char)((IPC_message_hdr_t *)curBufferPtr)->replySeqNum,
					(unsigned short)((IPC_message_hdr_t *)curBufferPtr)->msgLen);
			OS_printf_debug("[%02x|%02x|%02x|%01d%01d]\n",  ((IPC_message_hdr_t *)curBufferPtr)->destAgentId,
					(unsigned char)((IPC_message_hdr_t *)curBufferPtr)->srcAgentId,
					(unsigned char)((IPC_message_hdr_t *)curBufferPtr)->msgType,
					(unsigned char)(((IPC_message_hdr_t *)curBufferPtr)->flags).reply,
					(unsigned char)(((IPC_message_hdr_t *)curBufferPtr)->flags).request);
			OS_printf_debug("[  %08lx ]\n",  (UINT32)((IPC_message_hdr_t *)curBufferPtr)->ReplyMsgPtr);

			OS_printf_debug("[%02x|%02x|%02x|%02x]\n", (unsigned char)((IPC_first_buffer_t *)curBufferPtr)->body[0],
					(unsigned char)((IPC_first_buffer_t *)curBufferPtr)->body[1],
					(unsigned char)((IPC_first_buffer_t *)curBufferPtr)->body[2],
					(unsigned char)((IPC_first_buffer_t *)curBufferPtr)->body[3]);
			OS_printf_debug("[   ...\t    ]\n");

		}
		else
		{
			OS_printf_debug("[%02x|%02x|%02x|%02x]\n", (unsigned char)((IPC_next_buffer_t *)curBufferPtr)->body[0],
					(unsigned char)((IPC_next_buffer_t *)curBufferPtr)->body[1],
					(unsigned char)((IPC_next_buffer_t *)curBufferPtr)->body[2],
					(unsigned char)((IPC_next_buffer_t *)curBufferPtr)->body[3]);
			OS_printf_debug("[   ...\t    ]\n");
		}
		curBufferPtr    = (IPC_next_buf_ptr_t)IPC_NEXT_PTR_PART(nextBufferPtr);
//	} while (bufferType & IPC_BUF_TYPE_MTC);
#endif
}

/*
 * -----------------------------------------------------------
 * Function:	IPC_recieve
 * Description:	Processing IPC messages
 * Input:		max_msg_count:	max number processed messages per call
 * Output:		number of processed messages
 * -----------------------------------------------------------
 */
UINT32 CODE_DEFAULT IPC_recieve(UINT32 max_msg_count, IPC_trns_priority_e pri)
{
	UINT32		ix;
	VCHAR		*ipcData;
	HAL_INTERRUPT_SAVE_AREA;

	for (ix=0; ix<max_msg_count; ix++)
	{
#if 0 // defined(UNIX) || defined(WIN32)
		ipcData = NULL; // get message
		ASSERT (FALSE);
#else
		HAL_INTERRUPT_DISABLE;
		ipcData = IPC_trns_fifo_buf_read(IPC_OwnNode, pri);
		HAL_INTERRUPT_RESTORE;
#endif

		if (ipcData)
		{
			//IPC_msg_handler(ipcData);
			handle_incoming_packet(ipcData, IPC_OwnNode, pri);
		}
		else
		{
			break; /* no more messages, queue empty */
		}
	}
	return ix;
}


/*
 * -----------------------------------------------------------
 * Function:	IPC_recieve_queue
 * Description:	Processing IPC messages from High priority queue
 * Input:		max_msg_count:	max number processed messages per call
 * Output:		number of processed messages
 * -----------------------------------------------------------
 */
#if 0
UINT32 CODE_DEFAULT IPC_recieve_queue(UINT32 max_msg_count, IPC_trns_priority_e pri)
{
	UINT32		ix;
	VCHAR		*ipcData;

	for (ix=0; ix<max_msg_count; ix++)
	{
		ipcData = IPC_trns_fifo_buf_read(IPC_OwnNode, pri);

		if (ipcData)
		{
			OSA_mq_receive(IPC_fifo_mq, ipcData);
			IPC_msg_handler(ipcData);
		}
		else
		{
			break; /* no more messages, queue empty */
		}
	}
	return ix;
}
#endif

/*
 * -----------------------------------------------------------
 * Function:	IPC_msg_handler
 * Description:	Handling single IPC message
 * Input:		ipcData - IPC message
 * Output:		None
 * -----------------------------------------------------------
 */
void CODE_DEFAULT IPC_msg_handler(VCHAR *ipcData)
{
	IPC_agent_t	*agent_p;
	UINT16		msgLen;
	UINT8		destAgentId;

	ASSERT(ipcData);
	destAgentId = ((IPC_message_hdr_t *)ipcData)->destAgentId;
	msgLen = ((IPC_message_hdr_t *)ipcData)->msgLen;
#ifdef IPC_DBG_CF
#if 1 //!defined(UNIX) && !defined(WIN32)
	OS_printf_debug("[%s]-%u destAgentId: %02X (%u) msgLen: %u\n",
			__FUNCTION__, __LINE__, destAgentId, destAgentId, msgLen);
#endif
#endif

#if defined(IPC_USE_TRACE)
	TRACE4(IPC_trace_client_id, IPC_e_MSG_PROC, (UINT32)ipcData,
			(UINT32)((IPC_message_hdr_t *)ipcData)->destAgentId,
			(UINT32)((IPC_message_hdr_t *)ipcData)->srcAgentId,
			(UINT32)((IPC_message_hdr_t *)ipcData)->msgType);
#endif

	/* check is the message belongs to current node */
	if(IPC_OwnNode == IPC_GetNode(destAgentId))
	{

		ASSERT(agent_p = IPC_agent_array[IPC_LocalId(destAgentId)]);
		ASSERT(agent_p->cb);
		/* call user registered function */
		agent_p->cb((void *)ipcData);
	}
	else /* forward if possible */
	{
		ASSERT(IPC_agent_array[0]);

		if (IPC_agent_array[0]->forward_cb)
			IPC_agent_array[0]->forward_cb((void *)ipcData, 0);


#if !defined(UNIX) && !defined(WIN32)
		IPC_chunk_send(IPC_agent_array[0], ipcData, IPC_trns_prio_e_0);
#endif
		// actually was used this destination, so revert it
#if defined(UNIX) || defined(WIN32)
// 	Not implemented
#else
		/* Restore destination ID, need for release buffer to proper pool */
		((IPC_message_hdr_t *)ipcData)->destAgentId =
					IPC_agent_array[0]->agentId;
		/* Usually the destination agent shall free the buffers */
		IPC_buf_free(IPC_agent_array[0], ipcData, IPC_trns_prio_e_0);
#endif
	} // forward
}

#if 0
void IPC_msg_handler(CHAR *ipcData)
{
	static IPC_agent_t	*agent_p;
	static VCHAR	*destMessage, *nextDestBuf;
	static UINT16	msgLen;
	static CHAR		*first_ptr;
	static TBOOL	msgForward = FALSE;

	CHAR	*bodyPtr,	*destBodyPtr;
	UINT8	bodyLen,	destBufLen;
	UINT32	bufferType,	destBufferType;
	UINT8	destAgentId;

	//Check received buffer
	IPC_buf_info (ipcData, &bodyPtr, &bodyLen, &bufferType);

	//Print buffer information
	//	OS_printf_debug("Buffer type = %d; Body len=%d\n", (int)bufferType, (int)bodyLen);
	//Its the first buffer
	if ((bufferType==IPC_BUF_TYPE_FULL) || (bufferType==IPC_BUF_TYPE_START))
	{
		destAgentId = ((IPC_message_hdr_t *)ipcData)->destAgentId;
		msgLen = ((IPC_message_hdr_t *)ipcData)->msgLen;
		OS_printf_debug("destAgentId: %02X (%u)\n", destAgentId, destAgentId);
		OS_printf_debug("Total len msgLen: %d\n", msgLen);

		/* check is the message belongs to current node */
		if(IPC_OwnNode == IPC_GetNode(destAgentId))
		{
			msgForward = FALSE;
			ASSERT(agent_p = IPC_agent_array[IPC_LocalId(destAgentId)]);
			ASSERT(agent_p->cb);
			/* call user registered function */
			agent_p->cb(ipcData);
		}
		else /* forward if possible */
		{
			msgForward = TRUE;
			first_ptr = ipcData;
			//Allocate message buffers for the destination agent
			destMessage = nextDestBuf = IPC_msg_alloc (IPC_agent_array[0],
					destAgentId, NULL, msgLen,
						((IPC_message_hdr_t *)ipcData)->msgType, NULL, 0);
			// set original source message ID
			((IPC_message_hdr_t *)destMessage)->srcAgentId =
			((IPC_message_hdr_t *)ipcData)->srcAgentId;
		}
	}

	//Copy the user data from the original buffer into the destination body
	if (msgForward)
	{
		//Get information about the buffer destination allocated buffer
		nextDestBuf = IPC_buf_info(nextDestBuf,	&destBodyPtr,
				&destBufLen, &destBufferType);

		if (bufferType & IPC_BUF_TYPE_MTC)
		{
			bodyPtr = ipcData + sizeof(IPC_message_hdr_t);
		}
		else
		{
			bodyPtr = ipcData + sizeof(ipc_buffer_hdr_t);
		}

		if (destBufLen == bodyLen)
		{
			memcpy(destBodyPtr, bodyPtr, bodyLen);
		}
		else
		{
			OS_printf_debug("Wrong Len destBufLen: %u bodyLen: %u\n",
					destBufLen, bodyLen);
		}
		if ((bufferType == IPC_BUF_TYPE_END) ||
			(bufferType == IPC_BUF_TYPE_FULL))
		{
				IPC_msg_send(IPC_agent_array[0], destMessage, IPC_trns_prio_e_0);
				// actually was used this destination, so revert it
#if defined(UNIX) || defined(WIN32)
//
#else

				((IPC_message_hdr_t *)first_ptr)->destAgentId =
						IPC_agent_array[0]->agentId;
				IPC_buf_free(IPC_agent_array[0], first_ptr, IPC_trns_prio_e_0);
#endif

				//Usually the dest agent shall free the buffers
		}
	}
	else
	{
		/* next buffer received, call user registered function */
		ASSERT(agent_p->cb)
		agent_p->cb(ipcData);
	}
}
#endif

/*
 * -----------------------------------------------------------
 * Function:	IPC_puts_len
 * Description:	Send console messages via IPC
 * Input:		str - user message
 * 				len	- message length in bytes
 * Output:		None
 * -----------------------------------------------------------
 */
void CODE_DEFAULT IPC_puts_len(CHAR *str, INT32 len)
{
	VCHAR *destMessage;

	if (IPC_agent_console_id)
	{
		ASSERT(len<(IPC_BUF_SIZE_MAX-sizeof(IPC_message_hdr_t)));
		destMessage = IPC_msg_alloc(IPC_agent_array[0], IPC_agent_console_id,
				str, (UINT16)len, 0, NULL, IPC_trns_prio_e_0);

		if (destMessage)
		{
			IPC_msg_send(IPC_agent_array[0], destMessage, IPC_trns_prio_e_0);
		}
		else
		{
			return;
		}
	}
}

/*
 * -----------------------------------------------------------
 * Function:	IPC_set_console_agent
 * Description:	Set IPC agent used for sending of console messages
 * Input:		agent - IPC agent
 * Output:		None
 * -----------------------------------------------------------
 */
void CODE_DEFAULT IPC_set_console_agent(UINT8 agent)
{
	IPC_agent_console_id = agent;
}


/*
 * -----------------------------------------------------------
 * Function:	IPC_set_console_agent
 * Description:	Set IPC agent used for sending of console messages
 * Input:		agent - IPC agent
 * Output:		None
 * -----------------------------------------------------------
 */
void CODE_DEFAULT IPC_print_agent_ptr(void)
{
	UINT32 ix;
	for (ix = 0; ix < MAX_LOCAL_AGENT; ix++)
	{
#if !defined(UNIX) && !defined(WIN32)
		OS_printf_debug("[%s]-%u ix:%u, agent_ptr: 0x%08X\n",
				__FUNCTION__, __LINE__, ix,	IPC_agent_array[ix]);
#else
	printk("[%s]-%u ix:%u, agent_ptr: 0x%p\n",
		__FUNCTION__, __LINE__, ix,	IPC_agent_array[ix]);
#endif
	}
}

/*
 * -----------------------------------------------------------
 * End of file
 * -----------------------------------------------------------
 */
