/** arch/arm/mach-msm/smd_rpcrouter.h
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

#ifndef _ARCH_ARM_MACH_MSM_SMD_RPCROUTER_H
#define _ARCH_ARM_MACH_MSM_SMD_RPCROUTER_H

#include <linux/types.h>
#include <linux/msm_rpcrouter.h>
#include <asm/arch/msm_smd.h>
#include <asm/arch/msm_rpcrouter.h>

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

#define RPCROUTER_CB_WORKAROUND		1
#define RPCROUTER_BW_COMP               1

/*
 *  These elements must be in host byte order
 */
struct pacmark_hdr
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
#if defined(CONFIG_MSM7X00A_6046_COMPAT)
	uint32_t padding;
#endif
};


struct rpcrouter_header
{
	uint32_t version;
	uint32_t msg_type;
	rpcrouter_address src_addr;
};

struct rpcrouter_packet_header
{
#if defined(CONFIG_MSM7X00A_6059_COMPAT)
	uint32_t confirm_rx;
#endif
	uint32_t msg_size;
	rpcrouter_address addr;
};

struct rpcrouter_complete_header
{
	struct rpcrouter_header rh;
	struct rpcrouter_packet_header ph;
};

#if RPCROUTER_CB_WORKAROUND
struct rpcrouter_client;
#endif
struct rpcrouter_server
{
	struct list_head list;

	rpcrouter_address addr;
	rpcrouter_program_version pv;

#if RPCROUTER_CB_WORKAROUND
	struct rpcrouter_client *client;
#endif

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
	rpcrouter_address src_addr;
	struct pacmark_hdr pacmark;
	struct list_head list;
	int data_size;
	void *data;
#if RPCROUTER_BW_COMP
        uint32_t prog;
        uint32_t vers;
#endif
};

struct rpcrouter_client
{
	struct list_head client_list;

	rpcrouter_address addr; /* this process connections router address */

	spinlock_t read_q_lock;
	struct list_head read_q;
	wait_queue_head_t wait_q;

#if RPCROUTER_CB_WORKAROUND
	struct rpcrouter_client *override;
#endif
	int (*notify)(int);

	dev_t dev; /* device node which was used by the client */
	rpcrouter_address bound_dest; /* If we're bound to a destination */

	rpcrouter_address cb_addr;
};

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

struct krpcrouterd_thread
{
	struct task_struct *thread;
	wait_queue_head_t wait;
	spinlock_t lock;
	int command;
	struct rpcrouter_xport *xport;
};

#define KTHREAD_CMD_NONE 0
#define KTHREAD_CMD_EXIT 1
#define KTHREAD_CMD_DATA 2

#endif
