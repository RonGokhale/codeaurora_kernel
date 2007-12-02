/** include/asm-arm/arch-msm/msm_rpcrouter.h
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2007 QUALCOMM Incorporated
 * Author: San Mehat <san@android.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 */

#ifndef _ASM_MSM_RPCROUTER_H
#define _ASM_MSM_RPCROUTER_H

#include <linux/types.h>
#include <linux/msm_rpcrouter.h>
#include <asm/arch/msm_smd.h>

#define RPCROUTER_MAX_REMOTE_SERVERS	50

#define RPCROUTER_CLIENT_BCAST_ID	0xffffffff
#define RPCROUTER_ROUTER_ADDRESS	0xfffffffe

#define RPCROUTER_PID_LOCAL 1
#define RPCROUTER_PID_REMOTE 0

#define RPCROUTER_CTRL_CMD_DATA			1
#define RPCROUTER_CTRL_CMD_HELLO		2
#define RPCROUTER_CTRL_CMD_BYE			3
#define RPCROUTER_CTRL_CMD_NEW_SERVER		4
#define RPCROUTER_CTRL_CMD_REMOVE_SERVER	5
#define RPCROUTER_CTRL_CMD_REMOVE_CLIENT	6
#define RPCROUTER_CTRL_CMD_EXIT			7

struct rpcrouter_server
{
	struct list_head list;

	rpcrouter_address addr;
	rpcrouter_program_version pv;

	dev_t device_number;
	struct cdev cdev;
	struct device *device;
};

struct rpcrouter_address_list
{
	struct list_head list;
	rpcrouter_address addr;
	struct rpcrouter_xport *xport;
};

/*
 * Queue of received packets per client
 */

struct rpcrouter_client_read_q
{
	struct list_head list;
	int data_size;
	void *data;
};

struct rpcrouter_client
{
	struct list_head client_list;

	rpcrouter_address addr; /* this process connections router address */

	spinlock_t read_q_lock;
	struct list_head read_q;
	wait_queue_head_t wait_q;

	dev_t dev; /* device node which was used by the client */
};

typedef struct rpcrouter_client rpcrouterclient_t;

struct rpcrouter_xport
{
	struct list_head xport_list;
	struct list_head remote_client_list;
	spinlock_t rcl_lock;
	uint8_t rx_buffer[RPCROUTER_MSGSIZE_MAX];

	int initialized;
	wait_queue_head_t hello_wait;

	rpcrouter_xport_address	xport_address;
	rpcrouter_address peer_router_address;
	union {
		struct {
			smd_channel_t *smd_channel;
			spinlock_t smd_lock;
		} smd;
		struct {
		} local;
	} xport_specific;
};

struct rpcrouter_ctrlmsg_server_args
{
	uint32_t prog;
	uint32_t ver;
	uint32_t pid;
	uint32_t cid;
};

struct rpcrouter_ctrlmsg_client_args
{
	uint32_t pid; /* processor id */
	uint32_t cid; /* client id */
};

struct rpcrouter_control_msg
{
	uint32_t command;
	union {
		struct rpcrouter_ctrlmsg_server_args	arg_s;
		struct rpcrouter_ctrlmsg_client_args	arg_c;
	} args;
};

struct rpcrouter_header
{
	uint32_t version;
	uint32_t msg_type;
	rpcrouter_address src_addr;
};

struct rpcrouter_packet_header
{
	uint32_t msg_size;
	rpcrouter_address addr;
};

struct rpcrouter_complete_header
{
	struct rpcrouter_header rh;
	struct rpcrouter_packet_header ph;
};

struct krpcrouterd_thread
{
	struct task_struct *thread;
	wait_queue_head_t wait;
	spinlock_t lock;
	int command;
	struct rpcrouter_xport *xport;
};

/*
 *  Kernel API for kernel consumers/producers
 */

int rpcrouter_kernapi_openxport(rpcrouter_xport_address *addr);
int rpcrouter_kernapi_open(uint32_t client_id, rpcrouterclient_t **client);
int rpcrouter_kernapi_close(rpcrouterclient_t *client);
int rpcrouter_kernapi_write(rpcrouterclient_t *client,
			    rpcrouter_address *dest,
			    void *buffer,
			    int count);
int rpcrouter_kernapi_read(rpcrouterclient_t *client,
			   void **buffer,
			   long timeout);
uint32_t rpcrouter_kernapi_getnextxid(void);
uint8_t rpcrouter_kernapi_getnextpacmarkid(void);
int rpcrouter_kernapi_getdest(rpcrouterclient_t *client,
			      uint32_t prog,
			      uint32_t vers,
			      long timeout,
			      rpcrouter_address *dest_addr);
#define KTHREAD_CMD_NONE 0
#define KTHREAD_CMD_EXIT 1
#define KTHREAD_CMD_DATA 2

/*
 * Structures for sending / receiving direct RPC requests
 * XXX: Any cred/verif lengths > 0 not supported
 */

/*
 * These elements must be in host byte order
 */
typedef struct
{
	union {
		uint32_t raw;
		struct {
			uint32_t length : 16;
			uint32_t message_id : 8;
			uint32_t reserved : 7;
			uint32_t last_pkt : 1;
		} cooked;
	} data;
	uint32_t padding;
} pacmark_header;

/* =====================
 * Reply data structures
 * =====================
 */
typedef struct
{
	uint32_t xid;
	uint32_t type;	/* 0 */
	uint32_t rpc_vers; /* 2 */
	uint32_t prog;
	uint32_t vers;
	uint32_t proceedure;
	uint32_t cred_flavor;
	uint32_t cred_length;
	uint32_t verf_flavor;
	uint32_t verf_length;
} rpc_request_hdr;

typedef struct
{
	pacmark_header pacmark_hdr;
	rpc_request_hdr rpc_hdr;
} oncrpc_request_hdr;

/* =====================
 * Reply data structures
 * =====================
 */
typedef struct
{
	uint32_t low;
	uint32_t high;
} rpc_reply_progmismatch_data;

typedef struct
{
} rpc_denied_reply_hdr;

typedef struct
{
	uint32_t verf_flavor;
	uint32_t verf_length;
	uint32_t accept_stat;
#define RPC_ACCEPTSTAT_SUCCESS 0
#define RPC_ACCEPTSTAT_PROG_UNAVAIL 1
#define RPC_ACCEPTSTAT_PROG_MISMATCH 2
#define RPC_ACCEPTSTAT_PROC_UNAVAIL 3
#define RPC_ACCEPTSTAT_GARBAGE_ARGS 4
#define RPC_ACCEPTSTAT_SYSTEM_ERR 5
#define RPC_ACCEPTSTAT_PROG_LOCKED 6
	/*
	 * Following data is dependant on accept_stat
	 * If ACCEPTSTAT == PROG_MISMATCH then there is a
	 * 'rpc_reply_progmismatch_data' structure following the header.
	 * Otherwise the data is proceedure specific
	 */
} rpc_accepted_reply_hdr;

typedef struct
{
	uint32_t xid;
	uint32_t type;
	uint32_t reply_stat;
#define RPCMSG_REPLYSTAT_ACCEPTED 0
#define RPCMSG_REPLYSTAT_DENIED 1
	union {
		rpc_accepted_reply_hdr acc_hdr;
		rpc_denied_reply_hdr dny_hdr;
	} data;
} rpc_reply_hdr;

typedef struct
{
	rpcrouter_address src_addr;
	pacmark_header pacmark_hdr;
	rpc_reply_hdr rpc_hdr;
} oncrpc_reply_hdr;
#endif
