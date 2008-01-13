/* arch/arm/mach-msm/rpc_servers.c
 *
 * Copyright (C) 2007 Google, Inc.
 * Author: Iliyan Malchev <ibm@android.com>
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/cdev.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/kthread.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/msm_rpcrouter.h>
#include <asm/arch/msm_rpcrouter.h>
#include "smd_rpcrouter.h"

/* time_remote_mtoa server definitions. */

#define TIME_REMOTE_MTOAPROG 0x3000005d
#define TIME_REMOTE_MTOAVERS 0
#define RPC_TIME_REMOTE_MTOA_NULL   0
#define RPC_TIME_TOD_SET_APPS_BASES 1

struct rpc_time_tod_set_apps_bases_args {
	uint32_t tick;
	uint64_t stamp;
};

/* dog_keepalive server definitions */

#define DOG_KEEPALIVEPROG 0x30000015
#define DOG_KEEPALIVEVERS 0
#define RPC_DOG_KEEPALIVE_NULL 0
#define RPC_DOG_KEEPALIVE_BEACON 1

/* common definitions */

static rpcrouterclient_t *rpc_client;
static struct rpcrouter_server *rpc_server;

static int rpc_send_accepted_void_reply(rpcrouterclient_t *client, uint32_t xid, uint32_t accept_status)
{
	int rc = 0;
	rpcrouter_address dest_addr;
	uint8_t reply_buf[sizeof(struct rpc_reply_hdr)];
	struct rpc_reply_hdr *reply = (struct rpc_reply_hdr *)reply_buf;

	reply->xid = cpu_to_be32(xid);
	reply->type = cpu_to_be32(1); /* reply */
	reply->reply_stat = cpu_to_be32(RPCMSG_REPLYSTAT_ACCEPTED);

	reply->data.acc_hdr.accept_stat = cpu_to_be32(accept_status);
	reply->data.acc_hdr.verf_flavor = 0;
	reply->data.acc_hdr.verf_length = 0;

	memcpy(&dest_addr, &client->cb_addr, sizeof(dest_addr));

	rc = rpcrouter_kernapi_write(rpc_client, &dest_addr, reply_buf, sizeof(reply_buf));
	if (rc < 0)
		printk(KERN_ERR "%s: could not write response: %d\n", __FUNCTION__, rc);

	return rc;
}

static int check_progvers(struct rpc_request_hdr *req, uint32_t expected_vers)
{
	if (req->vers != expected_vers) {
		printk(KERN_ERR"%s: version %d of program 0x%08x not supported, expecting %d\n",
			__FUNCTION__, req->vers, req->prog, expected_vers);
		rpc_send_accepted_void_reply(rpc_client, req->xid, RPC_ACCEPTSTAT_PROG_MISMATCH);
		return -EINVAL;
	}
	return 0;
}

static int handle_time_remote_mtoa(struct rpc_request_hdr *req)
{
	int rc = check_progvers(req, TIME_REMOTE_MTOAVERS);
	if (!rc) {
		switch (req->procedure) {
		case RPC_TIME_REMOTE_MTOA_NULL:
			rpc_send_accepted_void_reply(rpc_client, req->xid, RPC_ACCEPTSTAT_SUCCESS);
			break;
		case RPC_TIME_TOD_SET_APPS_BASES:
		{
			struct rpc_time_tod_set_apps_bases_args *args;
			args = (struct rpc_time_tod_set_apps_bases_args *)(req + 1);
			args->tick = be32_to_cpu(args->tick);
			args->stamp = be64_to_cpu(args->stamp);
			printk(KERN_INFO"RPC_TIME_TOD_SET_APPS_BASES:\n"
				"\ttick = %d\n"
				"\tstamp = %lld\n",
				args->tick, args->stamp);
			rpc_send_accepted_void_reply(rpc_client, req->xid, RPC_ACCEPTSTAT_SUCCESS);
			/* exit = 1; */ /* this will cause the server to exit. */
		}
		break;
		default:
			printk(KERN_ERR "%s: program 0x%08x:%d: unknown procedure %d\n",
				__FUNCTION__, req->prog, req->vers, req->procedure);
			rpc_send_accepted_void_reply(rpc_client, req->xid, RPC_ACCEPTSTAT_PROC_UNAVAIL);
			break;
		}
	}
	return rc;
}

static int handle_dog_keepalive(struct rpc_request_hdr *req)
{
	int rc = check_progvers(req, DOG_KEEPALIVEVERS);
	if (!rc) {
		switch (req->procedure) {
		case RPC_DOG_KEEPALIVE_NULL:
			rpc_send_accepted_void_reply(rpc_client, req->xid, RPC_ACCEPTSTAT_SUCCESS);
			break;
		case RPC_DOG_KEEPALIVE_BEACON:
			printk(KERN_INFO"DOG KEEPALIVE PING\n");
			rpc_send_accepted_void_reply(rpc_client, req->xid, RPC_ACCEPTSTAT_SUCCESS);
			break;
		default:
			printk(KERN_ERR "%s: program 0x%08x:%d: unknown procedure %d\n",
				__FUNCTION__, req->prog, req->vers, req->procedure);
			rpc_send_accepted_void_reply(rpc_client, req->xid, RPC_ACCEPTSTAT_PROC_UNAVAIL);
			break;
		}
	}
	return rc;
}

static int rpc_servers_thread(void *data)
{
	void *buffer;
	struct rpc_request_hdr *req;
	int rc, exit = 0;

	do {
		rc = rpcrouter_kernapi_read(rpc_client, &buffer, -1, -1);
		if (rc < 0) {
			printk(KERN_ERR "%s: could not read: %d\n", __FUNCTION__, rc);
			break;
		}
		req = (struct rpc_request_hdr *)buffer;

		req->type = be32_to_cpu(req->type);
		req->xid = be32_to_cpu(req->xid);
		req->rpc_vers = be32_to_cpu(req->rpc_vers);
		req->prog = be32_to_cpu(req->prog);
		req->vers = be32_to_cpu(req->vers);
		req->procedure = be32_to_cpu(req->procedure);

#if 0
		printk(KERN_ERR"%s: ++++++++ incoming RPC:\n"
			"\ttype = %d\n"
			"\trpc vers = %d\n"
			"\tprog = 0x%08x\n"
			"\tvers = %d\n"
			"\tproc = %d\n",
			__FUNCTION__,
			req->type, req->rpc_vers, req->prog, req->vers, req->procedure);
#endif

		if (req->type == 0) {
			if (req->rpc_vers != 2) {
				printk(KERN_ERR"%s: (serious error) unknown RPC version %d, expecting 2.\n",
					__FUNCTION__, req->rpc_vers);
			} else {
				switch (req->prog) {
				case TIME_REMOTE_MTOAPROG:
					handle_time_remote_mtoa(req);
					break;
				case DOG_KEEPALIVEPROG:
					handle_dog_keepalive(req);
					break;
				default:
					/* This is a serious routing error: we should not receive calls for other servers
					   onto this server. */
					printk(KERN_ERR"%s: (serious error) RPC call for program 0x%08x:%d routed incorrectly.\n",
						__FUNCTION__, req->prog, req->vers);
					rpc_send_accepted_void_reply(rpc_client, req->xid, RPC_ACCEPTSTAT_PROG_UNAVAIL);
					break;
				}
			}
		} else {
			/* This is a serious routing error; we should not receive replies on a server channel;
			   this reply must have been meant for someone else.
			*/
			printk(KERN_ERR"%s: (serious error) expecting an RPC call on server channel, "
				"but received an RPC reply!\n", __FUNCTION__);
		}
		/* free the read buffer -- it's the responsibility of the caller */
		kfree(buffer);
	} while (!exit);

	/* exit */
	do_exit(0);
}

static int __init rpc_servers_init(void)
{
	rpcrouter_xport_address	xport_addr;
	struct rpcrouter_ioctl_server_args server_args;
	int rc = 0;

	xport_addr.xp = RPCROUTER_XPORT_SMD;
	xport_addr.port = 2;

	rc = rpcrouter_kernapi_openxport(&xport_addr);
	if (rc < 0) {
		printk(KERN_ERR "rpc_servers: Error opening SMD xport (%d)\n", rc);
		goto done;
	}

	rc = rpcrouter_kernapi_open(&rpc_client);
	if (rc < 0) {
		printk(KERN_ERR "rpc_time_remote_mtoa: Error opening SMD client (%d)\n", rc);
		goto done_close_xport;
	}

	server_args.progver.prog = TIME_REMOTE_MTOAPROG;
	server_args.progver.ver  = TIME_REMOTE_MTOAVERS;
	rpc_server = rpcrouter_kernapi_register_server(rpc_client, &server_args);
	if (!rpc_server) {
		printk(KERN_ERR "rpc_servers: error registering server %08x:%d (%d)\n",
			server_args.progver.prog, server_args.progver.ver, rc);
		goto done_close_client;
	}

	server_args.progver.prog = DOG_KEEPALIVEPROG;
	server_args.progver.ver  = DOG_KEEPALIVEVERS;
	rpc_server = rpcrouter_kernapi_register_server(rpc_client, &server_args);
	if (!rpc_server) {
		printk(KERN_ERR "rpc_servers: error registering server %08x:%d (%d)\n",
			server_args.progver.prog, server_args.progver.ver, rc);
		goto done_close_client;
	}

	/* start the kernel thread */
	kthread_run(rpc_servers_thread, NULL, "krpcserversd");

	goto done;
done_close_client:
	rpcrouter_kernapi_close(rpc_client);
done_close_xport:
	/* there's no call to do this yet. */
done:
	return rc;
}

module_init(rpc_servers_init);

MODULE_DESCRIPTION("MSM RPC Servers");
MODULE_AUTHOR("Iliyan Malchev <ibm@android.com>");
MODULE_LICENSE("GPL");
