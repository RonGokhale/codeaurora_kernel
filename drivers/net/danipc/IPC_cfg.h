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


#ifndef IPC_CONFIG_H_
#define IPC_CONFIG_H_

/*
****************************************************************************
*                     MACROS
****************************************************************************
*/
/*
****************************************************************************
*                    TYPES
****************************************************************************
*/
// Node type Enumeration
typedef enum
{
	undef_e,
	dan3400_e, 	//Node is located on the DAN3400 based board 
	dan3400_eth_e, 	//  eth. ppoxy
	extEth_e, 	//Node is located on External board connected via ETh interface
	extFser_e,  //Node is located on External board connected via Fast Serial interface
	extUart_e, 	//Node is located on External board connected via Uart interface
} IPC_node_type_e;

/*
****************************************************************************
*                    PROTOTYPES
****************************************************************************
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
UINT8 IPC_getOwnNode(void);

/*
*===========================================================================
* IPC_cfg_get_type
*===========================================================================
* Description:  Fetch the node location based on the 
*               Node Id (5 lsbs)
*
* Parameters: nodeId
*
* Returns: Location Type
*
*/
IPC_node_type_e IPC_cfg_get_type(UINT8 nodeId);

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
IPC_transport_func_t *IPC_cfg_get_util_vec(UINT8 srcNode, UINT8 destNode);

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
void IPC_setAgentName (char *name, UINT8 inx);

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
char *IPC_getAgentName (UINT8 inx);

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
void IPC_agent_table_clean(void);

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
void IPC_routeTableInit(IPC_transport_func_t *defTranVecPtr);

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
IPC_transport_func_t *IPC_getUtilFuncVector(UINT8 nodeId);

#endif /*IPC_CONFIG_H_*/

/*
 ****************************************************************************
 *#                      END OF FILE
 ****************************************************************************
 */
