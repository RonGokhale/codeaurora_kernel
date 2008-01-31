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

#endif
