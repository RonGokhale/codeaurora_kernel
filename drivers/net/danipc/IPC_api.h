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



#ifndef _IPC_API_H_
#define _IPC_API_H_


#if defined(WIN32)
#define VCHAR volatile char
#endif /* WIN32 */


// #include "ipc_names.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 ****************************************************************************
 *#                     MACROS
 ****************************************************************************
 */

//#define IPC_DBG_CF 1
#define IPC_REDIRECT_PRINT_CF 1


//Macro Build AgentId from Given CpuId and LocalId
#define IPC_AGENT_ID(cpuid, lid)	\
		(((cpuid&(PLATFORM_MAX_NUM_OF_NODES-1)) << 3) + (0x07 & (lid)))
//Build IPC_IDX from Given CpuId (Node ID) and LocalId
#define IPC_GetNode(_id)		((_id)>>3)
#define IPC_LocalId(_id)		((_id)&0x07)

#define IPC_BUF_SIZE_MAX	   	128		/* in bytes 				*/
//#define IPC_MAX_BUFFER_COUNT	8		/*  PRI HIGH 8, PRI LOW 8	*/
#define IPC_BUF_COUNT_MAX   	128		/*  PRI HIGH 8, PRI LOW 8	*/
#define IPC_MAX_MESSAGE_SIZE   	(IPC_BUF_COUNT_MAX*IPC_BUF_SIZE_MAX)
#define IPC_BUF_SIZE_PER_NODE	(IPC_BUF_COUNT_MAX*IPC_BUF_SIZE_MAX)
#define IPC_FIFO_BUF_NUM_HIGH	(IPC_BUF_COUNT_MAX)
#define IPC_FIFO_BUF_NUM_LOW	(IPC_BUF_COUNT_MAX)

#define IPC_FIRST_BUF_DATA_SIZE_MAX	\
			(IPC_BUF_SIZE_MAX-sizeof(IPC_message_hdr_t))
#define IPC_NEXT_BUF_DATA_SIZE_MAX 	\
			(IPC_BUF_SIZE_MAX-sizeof(IPC_buffer_hdr_t))

#define IPC_BUF_TYPE_FULL	0 /* Single message buffer			*/
#define IPC_BUF_TYPE_START	1 /* First buffer, more to come		*/
#define IPC_BUF_TYPE_END	2 /* Last buffer, no more			*/
#define IPC_BUF_TYPE_MID	3 /* Mid buffer, more to come		*/

#define IPC_BUF_TYPE_MTC	1 /* There are more buffers			*/
#define IPC_BUF_TYPE_BITS	3 /* mask for type bits 			*/
//'Clean' the type part from the next_buffer field (clear 2 LSB bits)
#define IPC_NEXT_PTR_PART(_next) (VCHAR *)(((UINT32)_next) & ~IPC_BUF_TYPE_BITS)

#define MAX_AGENT_NAME_LEN 	32

#ifndef MAX_AGENTS
#define MAX_AGENTS       	256
#endif

#define MAX_LOCAL_AGENT		8

//IPC result codes
#define IPC_SUCCESS			0
#define IPC_GENERIC_ERROR	-1
#define IPC_NO_AGENT_ERROR	-2


#define TX_SYMB     "TX_SYMB"
#define HWM         "HWM"



// Note: currently CLI use infrastructure agent on each node (agent_id 0)
//#define TX_CPU2_CLI	"TX_CPU2_CLI"
//#define TX_CPU3_CLI	"TX_CPU3_CLI"
//#define TX_DSP0_CLI	"TX_DSP0_CLI"
#define NPU_CPU0_CLI	NPU_CPU0_0
#define NPU_CPU1_CLI	NPU_CPU1_0
#define NPU_CPU2_CLI	NPU_CPU2_0
#define NPU_CPU3_CLI	NPU_CPU3_0
#define NPU_CPU4_CLI	NPU_CPU4_0
#define NPU_CPU5_CLI	NPU_CPU5_0

#define TX_CPU1_CLI	   TX_CPU1_0
#define TX_CPU2_CLI	   TX_CPU2_ETH
#define TX_CPU3_CLI	   TX_CPU3_0
#define TX_DSP0_CLI	   TX_DSP0_0

//#define IPC_EXT_CAGE_MAC_ADDR	"\x00\x1B\x21\x89\x3D\xC9"
#define IPC_EXT_CAGE_MAC_ADDR	"\x00\x15\x17\xdb\x66\x6d"
//#define IPC_EXT_CAGE_MAC_ADDR	"\x00\x0E\x04\xB7\x05\x0A"
//#define IPC_EXT_CAGE_MAC_ADDR	"\xFF\xFF\xFF\xFF\xFF\xFF"
//#define IPC_RX_CAGE_MAC_ADDR	"\x00\x55\x7B\xB5\x7D\xF7"
//#define IPC_RX_CAGE_MAC_ADDR	"\x00\x01\x02\x03\x04\x05"
#define IPC_RX_CAGE_MAC_ADDR	"\x00\x0e\x04\xb7\x04\xfd"
//#define IPC_TX_CAGE_MAC_ADDR	"\x00\x11\x22\x33\x00\x0A"
//#define IPC_TX_CAGE_MAC_ADDR	"\x00\x01\x02\x03\x04\x05"
#define IPC_TX_CAGE_MAC_ADDR	"\x00\x0e\x04\xb7\x04\xfd"

#define IPC_PROXY_ETH_NODE_ID	18
#define IPC_MAC_EXT_NODE_ID		4


/*
 ****************************************************************************
 *#                     Debug customization
 ****************************************************************************
 */
#define IPC_BASE_ADDR			(IPC_COMMON_BUF_ADDR)
#define IPC_AGENT_TABLE			(IPC_BASE_ADDR + 0)
#define IPC_AGENT_TABLE_SIZE	(MAX_AGENTS*MAX_AGENT_NAME_LEN)
#define IPC_TX_BUF_START_ADDR 	(IPC_BASE_ADDR)

#define IPC_RX_BUF_START_ADDR	ADDR_NO_CACHE(GLOBMEM_ADDR(IPC_RX_BUF))
#define IPC_RX_BUF_SIZE			GLOBMEM_SIZE(IPC_RX_BUF)

#define IPC_NPU_BUF_START_ADDR	ADDR_NO_CACHE(GLOBMEM_ADDR(IPC_NPU_BUF))
#define IPC_NPU_BUF_SIZE		GLOBMEM_SIZE(IPC_NPU_BUF)

typedef void (*IPC_agent_cb_t) (void *);
typedef void (*IPC_forward_cb_t) (void *, unsigned int);


/*********************************
 *  Agent Structure (Temp)
 *********************************
*/
typedef struct IPC_agent_s
{
	CHAR				agentName[MAX_AGENT_NAME_LEN];
	UINT8				agentId;
	// Receive CB pointer
	IPC_agent_cb_t 		cb;
	IPC_forward_cb_t 	forward_cb;
} IPC_agent_t;

/*********************************
 *  Message header flag field
 *********************************
*/
typedef struct IPC_message_flag_s
{
	UINT8	reserved:	6;
	UINT8	reply:		1;	// Message is replay
	UINT8	request:	1;	// Message is a request
} IPC_message_flag_t;

/********************************************
 *  Message header
 *
 *  This is the header of the first buffer
 *
 ********************************************
*/
typedef struct IPC_buffer_hdr_s *IPC_next_buf_ptr_t;

#if defined(WIN32)
#pragma pack (push)
#pragma pack (1)
typedef struct IPC_message_hdr_s
{
	IPC_next_buf_ptr_t	nextBufPtr;		// Points to the next buffer if
										// exists. 2 LSB are used to identify
										// buffer type (full, start, end, mid)
	UINT8			requestSeqNum;		// Request sequence number
	UINT8			replySeqNum;		// Reply sequence number
	UINT16			msgLen;				// Total message length (Data part)
	UINT8			destAgentId;		// Message destination (Agent Id)
	UINT8			srcAgentId;			// Message source (Agent Id)
	UINT8			msgType;			// Application message type
	IPC_message_flag_t	flags;			// Message flag (request/Reply)
	VCHAR			*ReplyMsgPtr;		// Optional replay message pointer
} IPC_message_hdr_t;
#pragma pack (pop)
#else
typedef struct IPC_message_hdr_s
{
	IPC_next_buf_ptr_t	nextBufPtr;		// Points to the next buffer if
										// exists. 2 LSB are used to identify
										// buffer type (full, start, end, mid)
	UINT8			requestSeqNum;		// Request sequence number
	UINT8			replySeqNum;		// Reply sequence number
	UINT16			msgLen;				// Total message length (Data part)
	UINT8			destAgentId;		// Message destination (Agent Id)
	UINT8			srcAgentId;			// Message source (Agent Id)
	UINT8			msgType;			// Application message type
	IPC_message_flag_t	flags;			// Message flag (request/Reply)
	VCHAR			*ReplyMsgPtr;		// Optional replay message pointer
} __attribute__((packed)) IPC_message_hdr_t;

#endif /* WIN32 */


/********************************************
 *  Buffer header
 *
 *  This is the header of the next buffer[s]
 *
 ********************************************
*/
typedef struct IPC_buffer_hdr_s
{
	IPC_next_buf_ptr_t	nextBufPtr;		// Points to the next buffer if
										// exists. 2 LSB are used to identify
										// buffer type (full, start, end, mid)
} IPC_buffer_hdr_t;

/********************************************
 *  First Buffer structure
 *
 ********************************************
*/
typedef struct IPC_first_buffer_s
{
	IPC_message_hdr_t   msgHdr;
	CHAR 				body[IPC_FIRST_BUF_DATA_SIZE_MAX];
} IPC_first_buffer_t;

/********************************************
 *  Next Buffer[s] structure
 *
 ********************************************
*/
typedef struct IPC_next_buffer_s
{
	IPC_buffer_hdr_t	bufHdr;
	CHAR 				body[IPC_NEXT_BUF_DATA_SIZE_MAX];
} IPC_next_buffer_t;

/********************************************
 *  Priority (for transport)
 *
 ********************************************
*/
typedef enum
{
	IPC_trns_prio_e_0,    //Low
	IPC_trns_prio_e_1,    //..
	IPC_trns_prio_e_2,    //..
	IPC_trns_prio_e_3,    //High
} IPC_trns_priority_e;

typedef VCHAR *(*IPC_trns_buf_alloc_t)
				(UINT8 dest_agent_id, IPC_trns_priority_e pri);
typedef void (*IPC_trns_buf_free_t)
				(VCHAR *buf, UINT8 dest_agent_id, IPC_trns_priority_e pri);
typedef INT32 (*IPC_trns_buf_send_t)
				(VCHAR *buf, UINT8 dest_agent_id, IPC_trns_priority_e pri);

/********************************************
 *  Transport layer utility function vector
 *
 *  The structure hold the transport function
 *  to be used per destination node (alloc, send)
 *  and source (free)
 *
 ********************************************
*/
typedef struct IPC_transport_func_s
{
	IPC_trns_buf_alloc_t 	trns_buf_alloc_ptr;
	IPC_trns_buf_free_t		trns_buf_free_ptr;
	IPC_trns_buf_send_t		trns_buf_send_ptr;
} IPC_transport_func_t;

typedef struct agentNameEntry_s
{
	char 	agentName[MAX_AGENT_NAME_LEN];
} agentNameEntry_t;


#define IPC_AGENT_NAME(_name)	_name

typedef enum
{
#include "IPC_agent_name.h"
} IPC_agent_name_e;

extern const char *IPC_agent_name_array[];

#define IPC_GET_NAME(_n)	((char *)IPC_agent_name_array[_n])

/* Own Node number */
extern 	UINT8	IPC_agent_console_id;
extern	UINT8	IPC_OwnNode;

extern IPC_transport_func_t IPC_eth2proxy_utils;

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
void IPC_appl_init(IPC_transport_func_t *defTranVecPtr);


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
*/
int IPC_register(CHAR *name, IPC_agent_t *agent);

/*
*===========================================================================
* IPC_fake_register
*===========================================================================
* Description:  Registers IPC agent node, used for debugging only
*
* Returns: Result code
*
*/
UINT8 IPC_fake_register(CHAR *name, UINT32 cpu_id, UINT8 loc_id);

/*
 * -----------------------------------------------------------
 * Function:	IPC_msg_handler
 * Description:	Processing IPC messages
 * Input:		max_msg_count:	max number processed messages per call
 * 				pri: priority queue
 * Output:		None
 * -----------------------------------------------------------
 */
UINT32 IPC_recieve(UINT32 max_msg_count, IPC_trns_priority_e pri);


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
int IPC_Get_AgentId(CHAR *name, UINT8 *agentIdPtr);

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
int IPC_Get_AgentPtr (UINT8 agentId, IPC_agent_t** agent_ptr);

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
int IPC_callback_set(IPC_agent_t *agent, IPC_agent_cb_t cb);


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
int IPC_forward_callback_set(IPC_agent_t *agent, IPC_forward_cb_t cb);


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
volatile char *IPC_buf_alloc(IPC_agent_t *agent, UINT8 dest_agent_id, IPC_trns_priority_e pri);

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
INT32 IPC_buf_free(IPC_agent_t *agent, VCHAR *buf_first, IPC_trns_priority_e pri);

/*
*===========================================================================
* IPC_buf_link
*===========================================================================
* Description:  Link two buffers, should be called when message does not fit
* 				 the single buffer
*
* Parameters: 	agent 			- Pointer to Agent structure
* 				buf_prev		- Pointer to a message buffer
* 				buf_next		- Pointer to the next message buffer
* 								(to be linked to)
* Returns: Result code
*
*/
INT32 IPC_buf_link(IPC_agent_t *agent, VCHAR *buf_prev, VCHAR *buf_next);

/*
*===========================================================================
* IPC_msg_set_len
*===========================================================================
* Description:  sets message length, first buffer of the message
* 				should be provided
*
* Parameters: 	agent 			- Pointer to Agent structure
* 				buf_first		- Pointer to the first message buffer
* 				len				- Message length (bytes)
*
*
* Returns: Result code
*
*/
INT32 IPC_msg_set_len(IPC_agent_t *agent, VCHAR *buf_first, UINT16 len);

/*
*===========================================================================
* IPC_msg_set_type
*===========================================================================
* Description:  sets message type, first buffer of the message
* 				should be provided
*
* Parameters: 	agent 			- Pointer to Agent structure
* 				buf_first		- Pointer to the first message buffer
* 				type			- Message type
*
*
* Returns: Result code
*
*/
INT32 IPC_msg_set_type(IPC_agent_t *agent, VCHAR *buf_first, UINT8 type);

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
INT32 IPC_msg_set_reply_ptr
(
	IPC_agent_t		*agent,
	VCHAR			*buf_first,
	VCHAR			*buf_rep
);


/*
*===========================================================================
* IPC_msg_alloc
*===========================================================================
* Description:  Allocate message buffer[s] and set the type and length.
* 				Optionally copy message data into allocated buffers (if msgPtr
* 				is not NULL.
*
*
* Parameters: 	agent 			- Pointer to Agent structure
* 				dest_agent_id	- Message destination AgentId
* 				msgPtr			- Pointer to message data (optional)
* 				msgLen			- Message length
* 				type			- Message type
* 				repPtr			- Pointer to allocated reply buffer
* 				pri				- Transport priority level
*
*
* Returns: Pointer to the message first buffer
*
*/
VCHAR *IPC_msg_alloc
(
	IPC_agent_t		*agent,
	UINT8			dest_agent_id,
	CHAR			*msgPtr,
	UINT16			msgLen,
	UINT8			msgType,
	VCHAR			*repPtr,
	IPC_trns_priority_e pri
);

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
INT32 IPC_msg_send(IPC_agent_t *agent, VCHAR *buf_first, IPC_trns_priority_e pri);

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
INT32 IPC_chunk_send(IPC_agent_t *agent, VCHAR *buf_first, IPC_trns_priority_e pri);

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
VCHAR *IPC_buf_info
(
	VCHAR	*bufPtr,
	CHAR	**bodyPtr,
	UINT8	*bodyLen,
	UINT32	*bufType
);


/*
*===========================================================================
* IPC_trns_eth_add
*===========================================================================
* Description:  Adds Ehernet transport layer function vector
*
* Parameters: trn_vec_p - function vector
*
* Returns: n/a
*
*/
void IPC_trns_eth_add(IPC_transport_func_t *trn_vec_p);

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
void display_message (CHAR *buf_first);

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
void display_header (char *buf_first);


void IPC_msg_handler(VCHAR *ipcData);

UINT32 IPC_recieve(UINT32 max_msg_count, IPC_trns_priority_e pri);

/*
 * -----------------------------------------------------------
 * Function:	IPC_puts_len
 * Description:	Send console messages via IPC
 * Input:		str - user message
 * 				len	- message length in bytes
 * Output:		None
 * -----------------------------------------------------------
 */
void IPC_puts_len(CHAR *str, INT32 len);

/*
 * -----------------------------------------------------------
 * Function:	IPC_set_console_agent
 * Description:	Set IPC agent used for sending of console messages
 * Input:		agent - IPC agent
 * Output:		None
 * -----------------------------------------------------------
 */
void IPC_set_console_agent(UINT8 agent);

#if defined(WIN32)
extern IPC_transport_func_t mac_to_phy_utils;
extern void OS_assert(unsigned cause, char *file, unsigned line);
extern BOOLEAN pcap_send_eth_raw( char *buf_p,  unsigned int len);
#else
extern IPC_transport_func_t IPC_fifo_utils;
#endif /* WIN32 */


#ifdef __cplusplus
}
#endif


#endif /*_IPC_API_H_*/

/*
 ****************************************************************************
 *#                      END OF FILE
 ****************************************************************************
 */
