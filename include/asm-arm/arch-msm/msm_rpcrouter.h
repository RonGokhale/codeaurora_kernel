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

#ifndef __ASM__ARCH_MSM_RPCROUTER_H
#define __ASM__ARCH_MSM_RPCROUTER_H

#include <linux/types.h>
#include <linux/msm_rpcrouter.h>
#include <asm/arch/msm_smd.h>

struct rpcrouter_client;

typedef struct rpcrouter_client rpcrouterclient_t;

/*
 * Structures for sending / receiving direct RPC requests
 * XXX: Any cred/verif lengths > 0 not supported
 */

/* =====================
 * Reply data structures
 * =====================
 */
struct rpc_request_hdr 
{
	uint32_t xid;
	uint32_t type;	/* 0 */
	uint32_t rpc_vers; /* 2 */
	uint32_t prog;
	uint32_t vers;
	uint32_t procedure;
	uint32_t cred_flavor;
	uint32_t cred_length;
	uint32_t verf_flavor;
	uint32_t verf_length;
};

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

struct rpc_reply_hdr
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
};

/*
 *  Kernel API for kernel consumers/producers
 */

extern int rpcrouter_kernapi_openxport(rpcrouter_xport_address *addr);
extern int rpcrouter_kernapi_open(rpcrouterclient_t **client);
extern int rpcrouter_kernapi_close(rpcrouterclient_t *client);
extern int rpcrouter_kernapi_write(rpcrouterclient_t *client,
			    rpcrouter_address *dest,
			    void *buffer,
			    int count);
extern int rpcrouter_kernapi_read(rpcrouterclient_t *client,
			   void **buffer,
			   unsigned user_len,
			   long timeout);
extern int rpcrouter_kernapi_getdest(rpcrouterclient_t *client,
			      uint32_t prog,
			      uint32_t vers,
			      long timeout,
			      rpcrouter_address *dest_addr);
extern void rpcrouter_kernapi_setup_request(struct rpc_request_hdr *hdr,
				     uint32_t prog,
				     uint32_t vers,
				     uint32_t proc);
extern struct rpcrouter_server *rpcrouter_kernapi_register_server(struct rpcrouter_client *client,
		struct rpcrouter_ioctl_server_args *server_args);
extern int rpcrouter_kernapi_unregister_server(struct rpcrouter_client *client,
		struct rpcrouter_ioctl_server_args *server_args);
#endif
