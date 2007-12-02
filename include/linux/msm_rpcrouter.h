/* include/linux/msm_rpcrouter.h
 *
 * Copyright (c) QUALCOMM Incorporated
 * Copyright (C) 2007 Google, Inc.
 * Author: San Mehat <san@android.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#ifndef __LINUX_MSM_RPCROUTER_H
#define __LINUX_MSM_RPCROUTER_H

#include <linux/types.h>
#include <linux/ioctl.h>

#define RPCROUTER_VERSION 1
#define RPCROUTER_MSGSIZE_MAX 512
#define RPCROUTER_PROCESSORS_MAX 4

typedef struct
{
	uint32_t pid; /* processor id */
	uint32_t cid; /* client id */
} rpcrouter_address;

typedef struct
{
	uint32_t	prog; /* program */
	uint32_t	ver; /* version */
} rpcrouter_program_version;

typedef struct
{
	uint32_t	xp;
	uint32_t	port;
} rpcrouter_xport_address;

#define RPCROUTER_XPORT_NONE	0
#define RPCROUTER_XPORT_LOCAL	1
#define RPCROUTER_XPORT_SMD	2
#define RPCROUTER_XPORT_SMMS	3
#define RPCROUTER_XPORT_SOCKETS	4
#define RPCROUTER_XPORT_ALL	5

/*
 *  Arguments for RPC_ROUTER_IOCTL_OPEN_XPORT and RPC_ROUTER_IOCTL_CLOSE_XPORT
 */
struct rpcrouter_ioctl_xport_args
{
	rpcrouter_xport_address addr;
};

/*
 * Arguments for RPC_ROUTER_IOCTL_REGISTER_SERVER and
 * RPC_ROUTER_IOCTL_UNREGISTER_SERVER
 */
struct rpcrouter_ioctl_server_args
{
	rpcrouter_program_version progver;
};

/*
 * Arguments for RPC_ROUTER_IOCTL_GET_DEST
 */

struct rpcrouter_ioctl_dest_args
{
	union {
		struct {
		rpcrouter_program_version pv;
		uint32_t timeout;
		} input;
	rpcrouter_address output;
	} data;
};

#define RPC_ROUTER_SIGNAL_TIMEOUT_INFINITE (0xFFFFFFFF)
#define RPC_ROUTER_IOCTL_MAGIC (0xC0)

#define RPC_ROUTER_IOCTL_GET_MTU \
   _IOR(RPC_ROUTER_IOCTL_MAGIC, 1, unsigned int)

#define RPC_ROUTER_IOCTL_REGISTER_SERVER \
   _IOWR(RPC_ROUTER_IOCTL_MAGIC, 4, unsigned int)

#define RPC_ROUTER_IOCTL_UNREGISTER_SERVER \
   _IOWR(RPC_ROUTER_IOCTL_MAGIC, 5, unsigned int)

#define RPC_ROUTER_IOCTL_GET_DEST \
   _IOWR(RPC_ROUTER_IOCTL_MAGIC, 6, unsigned int)

#define RPC_ROUTER_IOCTL_GET_TX_QUOTA \
   _IOR(RPC_ROUTER_IOCTL_MAGIC, 2, unsigned int)

#define RPC_ROUTER_IOCTL_OPEN_XPORT \
   _IOWR(RPC_ROUTER_IOCTL_MAGIC, 7, unsigned int)

#define RPC_ROUTER_IOCTL_GET_RX_BUFFER_SIZE \
   _IOR(RPC_ROUTER_IOCTL_MAGIC, 3, unsigned int)

#define RPC_ROUTER_IOCTL_CLOSE_XPORT \
   _IOWR(RPC_ROUTER_IOCTL_MAGIC, 8, unsigned int)

#define RPC_ROUTER_IOCTL_REGISTER_SERVER \
   _IOWR(RPC_ROUTER_IOCTL_MAGIC, 4, unsigned int)

#define RPC_ROUTER_IOCTL_UNREGISTER_SERVER \
   _IOWR(RPC_ROUTER_IOCTL_MAGIC, 5, unsigned int)

#define RPC_ROUTER_IOCTL_GET_DEST \
   _IOWR(RPC_ROUTER_IOCTL_MAGIC, 6, unsigned int)

#define RPC_ROUTER_IOCTL_OPEN_XPORT \
   _IOWR(RPC_ROUTER_IOCTL_MAGIC, 7, unsigned int)

#define RPC_ROUTER_IOCTL_CLOSE_XPORT \
   _IOWR(RPC_ROUTER_IOCTL_MAGIC, 8, unsigned int)

#define RPC_ROUTER_IOCTL_GET_ROUTER_STATS \
   _IOR(RPC_ROUTER_IOCTL_MAGIC, 9, unsigned int)

#define RPC_ROUTER_IOCTL_GET_CLIENT_STATS \
   _IOR(RPC_ROUTER_IOCTL_MAGIC, 10, unsigned int)

#define RPC_ROUTER_IOCTL_GET_ROUTING_TABLE \
   _IOWR(RPC_ROUTER_IOCTL_MAGIC, 11, unsigned int)

#endif
