/* arch/arm/mach-msm/smd_rpcrouter.c
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/cdev.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/err.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/poll.h>
#include <asm/uaccess.h>
#include <asm/byteorder.h>
#include <asm/arch/msm_smd.h>
#include <linux/msm_rpcrouter.h>
#include "smd_rpcrouter.h"

#define MSM_RPCROUTER_DEBUG 0
#define MSM_RPCROUTER_DEBUG_PKT 0
#define MSM_RPCROUTER_R2R_DEBUG 1
#define DUMP_ALL_RECEIVED_HEADERS 0

#if MSM_RPCROUTER_DEBUG
#define D(x...) printk(x)
#else
#define D(x...) do {} while (0)
#endif

static int rpcrouter_open(struct inode *inode, struct file *filp);
static int rpcrouter_release(struct inode *inode, struct file *filp);
static ssize_t rpcrouter_callback_read(struct file *filp, char __user *buf,
				size_t count, loff_t *ppos);
static ssize_t rpcrouter_read(struct file *filp, char __user *buf,
				size_t count, loff_t *ppos);
static ssize_t rpcrouter_callback_write(struct file *filp, const char __user *buf,
				size_t count, loff_t *ppos);
static ssize_t rpcrouter_write(struct file *filp, const char __user *buf,
				size_t count, loff_t *ppos);
static unsigned int rpcrouter_poll(struct file *filp, struct poll_table_struct *pfd);
static long rpcrouter_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);

static dev_t rpcrouter_devno; /* Device number for the router itself */
static int next_minor = 1; /* Next minor # available for a remote server */
static struct cdev rpcrouter_cdev;
static struct class *rpcrouter_class;
static struct device *rpcrouter_device;

static LIST_HEAD(client_list);
static LIST_HEAD(xport_list);
static LIST_HEAD(server_list);
#if RPCROUTER_BW_COMP
#define ONCRPC_BW_COMP_LOCAL_PROG 0x3000FFFE
#define ONCRPC_BW_COMP_LOCAL_VERS 1
static LIST_HEAD(unregistered_server_calls_list);
static DEFINE_SPINLOCK(unregistered_server_calls_list_lock);
static rpcrouterclient_t *bw_client;
static struct rpcrouter_server *bw_server;
#endif

static DEFINE_SPINLOCK(client_list_lock);
static DEFINE_SPINLOCK(xport_list_lock);
static DEFINE_SPINLOCK(server_list_lock);

static struct krpcrouterd_thread worker_thread;
static uint32_t next_xid;
static uint8_t next_pacmarkid;
static wait_queue_head_t newserver_wait;

static struct file_operations rpcrouter_server_fops = {
	.owner	 = THIS_MODULE,
	.open	 = rpcrouter_open,
	.release = rpcrouter_release,
	.read	 = rpcrouter_read,
	.write	 = rpcrouter_write,
	.poll    = rpcrouter_poll,
	.unlocked_ioctl	 = rpcrouter_ioctl,
};

static struct file_operations rpcrouter_router_fops = {
	.owner	 = THIS_MODULE,
	.open	 = rpcrouter_open,
	.release = rpcrouter_release,
	.read	 = rpcrouter_callback_read,
	.write	 = rpcrouter_callback_write,
	.poll    = rpcrouter_poll,
	.unlocked_ioctl = rpcrouter_ioctl,
};

#if MSM_RPCROUTER_DEBUG_PKT
static void dump_pacmark(struct pacmark_hdr *pacmark)
{
	printk(KERN_INFO "  Pacmark: 0x%x (len %d, id %d, lp %d)\n",
	       pacmark->data.raw,
	       pacmark->data.cooked.length,
	       pacmark->data.cooked.message_id,
	       pacmark->data.cooked.last_pkt);
}

static void dump_rpc_req_pkt(struct rpc_request_hdr *rq, int arglen)
{
	uint32_t *p;
	int i;

	printk(KERN_INFO "CALL\n");
	printk(KERN_INFO "  Xid: %x Type: %d Ver: %d Prg: 0x%x Ver: %d Prc: %d Cred: {%d:%d:%d:%d}\n",
		be32_to_cpu(rq->xid), be32_to_cpu(rq->type),
		be32_to_cpu(rq->rpc_vers), be32_to_cpu(rq->prog),
		be32_to_cpu(rq->vers), be32_to_cpu(rq->procedure),
		be32_to_cpu(rq->cred_flavor),
		be32_to_cpu(rq->cred_length), be32_to_cpu(rq->verf_flavor),
		be32_to_cpu(rq->verf_length));

	if (be32_to_cpu(rq->cred_flavor != 0)) {
		printk(KERN_INFO "  (Dump stopped because cred_flavor != 0)\n");
		return;
	}
	if (be32_to_cpu(rq->verf_flavor != 0)) {
		printk(KERN_INFO "  (Dump stopped because verf_flavor != 0)\n");
		return;
	}

	p = ((void *) rq) + sizeof(struct rpc_request_hdr);
	arglen -= sizeof(struct rpc_request_hdr);

	printk(KERN_INFO "  Args(%d):", arglen);

	if (arglen > 0)
		for (i = 0; i < arglen / sizeof(uint32_t); i++)
			printk("  %.8x", be32_to_cpu(p[i]));
	printk("\n");
}

static void dump_rpc_rsp_pkt(struct rpc_reply_hdr *rs, int arglen)
{
	uint32_t *p;
	int i;

	printk(KERN_INFO "REPLY\n");
	printk(KERN_INFO "  Xid: %x Type: %d ReplyStat: %d",
		be32_to_cpu(rs->xid), be32_to_cpu(rs->type),
		be32_to_cpu(rs->reply_stat));

	if (be32_to_cpu(rs->reply_stat) == RPCMSG_REPLYSTAT_ACCEPTED) {
		uint32_t acc_stat = be32_to_cpu(rs->data.acc_hdr.accept_stat);

		printk(" (accepted) Cred: {%d:%d} AcceptStat: %d",
			be32_to_cpu(rs->data.acc_hdr.verf_flavor),
			be32_to_cpu(rs->data.acc_hdr.verf_length),
			be32_to_cpu(rs->data.acc_hdr.accept_stat));
		if (acc_stat == RPC_ACCEPTSTAT_SUCCESS)
			printk(" (success)\n");
		else if (acc_stat == RPC_ACCEPTSTAT_PROG_UNAVAIL)
			printk(" (prog unavail)\n");
		else if (acc_stat == RPC_ACCEPTSTAT_PROG_MISMATCH)
			printk(" (prog mismatch)\n");
		else if (acc_stat == RPC_ACCEPTSTAT_PROC_UNAVAIL)
			printk(" (proc unavail)\n");
		else if (acc_stat == RPC_ACCEPTSTAT_GARBAGE_ARGS)
			printk(" (garbage args)\n");
		else if (acc_stat == RPC_ACCEPTSTAT_SYSTEM_ERR)
			printk(" (system err)\n");
		else if (acc_stat == RPC_ACCEPTSTAT_PROG_LOCKED)
			printk(" (prog locked)\n");
	} else {
		printk(" (denied)\n");
		return;
	}

	p = ((void *) rs) + sizeof(struct rpc_reply_hdr);
	arglen -= sizeof(struct rpc_reply_hdr);
	printk(KERN_INFO "  Args(%d):", arglen);

	if (arglen > 0)
		for (i = 0; i < arglen / sizeof(uint32_t); i++)
			printk("  %.8x", be32_to_cpu(p[i]));
	printk("\n");
}

static void dump_rpc_pkt(void *buf, int arglen)
{
	uint32_t *data = (uint32_t *)buf;
	if (arglen >= 8) {
		if (0 == data[1])
			dump_rpc_req_pkt(buf, arglen);
		else
			dump_rpc_rsp_pkt(buf, arglen);
	} else
		printk(KERN_INFO "  length %d is less than 8 bytes.\n", arglen);
}

#endif

/*
 *  Send a control msg to the remote router
 *  NOTE: xport->lock MUST be held
 */
static int rpcrouter_send_control_msg(struct rpcrouter_xport *xport,
				      struct rpcrouter_control_msg *msg)
{
	struct rpcrouter_complete_header hdr;
	int rc;
	unsigned long flags;

#if MSM_RPCROUTER_R2R_DEBUG
	printk(KERN_INFO "rpcrouter_send_control_msg(): [XP %x P %x] Command 0x%x\n",
			xport->xport_address.xp,
			xport->xport_address.port,
			msg->command);
#endif

	if (xport->xport_address.xp != RPCROUTER_XPORT_SMD) {
		printk(KERN_ERR "rpcrouter: Unsupported xport [XP %x P %x\n",
			xport->xport_address.xp, xport->xport_address.port);
		return -EINVAL;
	}

	if (!(msg->command == RPCROUTER_CTRL_CMD_HELLO)
	 && !xport->initialized) {
		printk(KERN_ERR "rpcrouter_send_control_msg(): Warning, xport "
		       "[XP %x P %x] not yet initialized\n",
			xport->xport_address.xp, xport->xport_address.port);
		return -EINVAL;
	}

	memset(&hdr, 0, sizeof(struct rpcrouter_complete_header));

	hdr.rh.version = RPCROUTER_VERSION;
	hdr.rh.msg_type = msg->command;
	hdr.rh.src_addr.pid = RPCROUTER_PID_LOCAL;
	hdr.rh.src_addr.cid = RPCROUTER_ROUTER_ADDRESS;
	hdr.ph.msg_size = sizeof(struct rpcrouter_control_msg);
	hdr.ph.addr.pid = xport->peer_router_address.pid;
	hdr.ph.addr.cid = xport->peer_router_address.cid;

	spin_lock_irqsave(&xport->xport_specific.smd.smd_lock, flags);
	/* Write header */
	rc = smd_write(xport->xport_specific.smd.smd_channel,
		       &hdr,
		       sizeof(struct rpcrouter_complete_header));
	if (rc < 0) {
		spin_unlock_irqrestore(&xport->xport_specific.smd.smd_lock,
				       flags);
		return rc;
	}

	/* Write data */
	rc = smd_write(xport->xport_specific.smd.smd_channel, msg,
		       hdr.ph.msg_size);
	spin_unlock_irqrestore(&xport->xport_specific.smd.smd_lock, flags);
	return rc;
}

/*  ===================================
 *  Server creation / deletion / lookup
 *  ===================================
 */

static int create_server_chardev(struct rpcrouter_server *server)
{
	int rc;

	if (next_minor == RPCROUTER_MAX_REMOTE_SERVERS) {
		printk(KERN_ERR "rpcrouter: Minor numbers exhausted - Increase "
		       "RPCROUTER_MAX_REMOTE_SERVERS\n");
		return -ENOBUFS;
	}
	server->device_number = MKDEV(MAJOR(rpcrouter_devno), next_minor++);

	server->device = device_create(rpcrouter_class,
					rpcrouter_device,
					server->device_number,
					"%.8x:%d",
					server->pv.prog,
					server->pv.ver);
	if (IS_ERR(server->device)) {
		printk(KERN_ERR "rpcrouter: Unable to create device (%ld)\n",
		       PTR_ERR(server->device));
		return PTR_ERR(server->device);;
	}

	cdev_init(&server->cdev, &rpcrouter_server_fops);
	server->cdev.owner = THIS_MODULE;

	rc = cdev_add(&server->cdev, server->device_number, 1);
	if (rc < 0) {
		printk(KERN_ERR "rpcrouter: Unable to add chrdev (%d)\n", rc);
		device_destroy(rpcrouter_class, server->device_number);
		return rc;
	}
	return 0;
}

static struct rpcrouter_server *rpcrouter_create_server(uint32_t pid,
							uint32_t cid,
							uint32_t prog,
							uint32_t ver)
{
	struct rpcrouter_server *server;
	unsigned long flags;
	int rc;

	server = kmalloc(sizeof(struct rpcrouter_server), GFP_KERNEL);
	if (!server)
		return ERR_PTR(-ENOMEM);

	memset(server, 0, sizeof(struct rpcrouter_server));
	server->addr.pid = pid;
	server->addr.cid = cid;
	server->pv.prog = prog;
	server->pv.ver = ver;

	spin_lock_irqsave(&server_list_lock, flags);
	list_add_tail(&server->list, &server_list);
	spin_unlock_irqrestore(&server_list_lock, flags);

	if (pid == RPCROUTER_PID_REMOTE) {
		rc = create_server_chardev(server);
		if (rc < 0) {
			spin_lock_irqsave(&server_list_lock, flags);
			list_del(&server->list);
			spin_unlock_irqrestore(&server_list_lock, flags);
			kfree(server);
			return ERR_PTR(rc);
		}
	}
	return server;
}

static void rpcrouter_destroy_server(struct rpcrouter_server *server)
{
	unsigned long flags;

	spin_lock_irqsave(&server_list_lock, flags);
	list_del(&server->list);
	spin_unlock_irqrestore(&server_list_lock, flags);
	device_destroy(rpcrouter_class, server->device_number);
	kfree(server);
}

static struct rpcrouter_server *rpcrouter_lookup_server(uint32_t prog,
							uint32_t ver)
{
	struct rpcrouter_server *server;
	unsigned long flags;

	spin_lock_irqsave(&server_list_lock, flags);
	list_for_each_entry(server, &server_list, list) {
		if (server->pv.prog == prog
		 && server->pv.ver == ver) {
			spin_unlock_irqrestore(&server_list_lock, flags);
			return server;
		}
	}
	spin_unlock_irqrestore(&server_list_lock, flags);
	return NULL;
}

static struct rpcrouter_server *rpcrouter_lookup_server_by_dev(dev_t dev)
{
	struct rpcrouter_server *server;
	unsigned long flags;

	spin_lock_irqsave(&server_list_lock, flags);
	list_for_each_entry(server, &server_list, list) {
		if (server->device_number == dev) {
			spin_unlock_irqrestore(&server_list_lock, flags);
			return server;
		}
	}
	spin_unlock_irqrestore(&server_list_lock, flags);
	return NULL;
}


/*  ======================================================
 *  Client creation / deletion / lookup / perms validation
 *  ======================================================
 */

static int client_validate_permission(struct rpcrouter_client *client,
				      uint32_t prog, uint32_t vers)
{
#if RPCROUTER_CB_WORKAROUND
	return 0;
#else
	struct rpcrouter_server *server;

	if (client->dev == rpcrouter_devno)
		return 0; /* If you open minor 0 you get it all */
	if (client->dev == MKDEV(0, 0))
		return 0; /* Kernel API users can do what they want */

	server = rpcrouter_lookup_server(prog, vers);
	if (!server)
		return -ENOENT;

	if (client->dev == server->device_number)
		return 0;
	return -EPERM;
#endif
}

static struct rpcrouter_client *rpcrouter_create_local_client(dev_t dev)
{
	struct rpcrouter_client *client;
	unsigned long flags;

	client = kmalloc(sizeof(struct rpcrouter_client), GFP_KERNEL);
	if (!client)
		return NULL;
	memset(client, 0, sizeof(struct rpcrouter_client));

	client->addr.cid = (uint32_t) client;
	client->addr.pid = RPCROUTER_PID_LOCAL;
	client->dev = dev;

	if ((dev != rpcrouter_devno) && (dev != MKDEV(0, 0))) {
		struct rpcrouter_server *srv;
		/*
		 * This is a userspace client which opened
		 * a program/ver devicenode. Bind the client
		 * to that destination
		 */
		srv = rpcrouter_lookup_server_by_dev(dev);
		BUG_ON(!srv);

		memcpy(&client->bound_dest,
			&srv->addr,
			sizeof(rpcrouter_address));
	}

	init_waitqueue_head(&client->wait_q);
	INIT_LIST_HEAD(&client->read_q);
	spin_lock_init(&client->read_q_lock);

	spin_lock_irqsave(&client_list_lock, flags);
	list_add_tail(&client->client_list, &client_list);
	spin_unlock_irqrestore(&client_list_lock, flags);
	return client;
}

static int rpcrouter_destroy_local_client(struct rpcrouter_client *client)
{
	int rc;

	struct rpcrouter_control_msg msg;
	struct rpcrouter_xport *xport;
	unsigned long flags;

	msg.command = RPCROUTER_CTRL_CMD_REMOVE_CLIENT;
	msg.args.arg_c.pid = client->addr.pid;
	msg.args.arg_c.cid = client->addr.cid;

	/*
	 * This message must be sent over every xport
	 */
	spin_lock_irqsave(&xport_list_lock, flags);
	list_for_each_entry(xport, &xport_list, xport_list) {
		rc = rpcrouter_send_control_msg(xport, &msg);
		if (rc < 0) {
			spin_unlock_irqrestore(&xport_list_lock, flags);
			return rc;
		}
	}
	list_del(&client->client_list);
	spin_unlock_irqrestore(&xport_list_lock, flags);
	kfree(client);
	return 0;
}

static int rpcrouter_create_remote_client(struct rpcrouter_xport *xport,
					  uint32_t cid)
{
	struct rpcrouter_address_list *new_c;
	unsigned long flags;

	new_c = kmalloc(sizeof(struct rpcrouter_address_list), GFP_KERNEL);
	if (!new_c)
		return -ENOMEM;
	memset(new_c, 0, sizeof(struct rpcrouter_address_list));

	new_c->addr.cid = cid;
	new_c->addr.pid = RPCROUTER_PID_REMOTE;
	new_c->xport = xport;

	spin_lock_irqsave(&xport->rcl_lock, flags);
	list_add_tail(&new_c->list, &xport->remote_client_list);
	spin_unlock_irqrestore(&xport->rcl_lock, flags);

	return 0;
}

static struct rpcrouter_client *rpcrouter_lookup_local_client(uint32_t cid)
{
	struct rpcrouter_client *client;
	unsigned long flags;

	spin_lock_irqsave(&client_list_lock, flags);
	list_for_each_entry(client, &client_list, client_list) {
		if (client->addr.cid == cid) {
			spin_unlock_irqrestore(&client_list_lock, flags);
			return client;
		}
	}
	spin_unlock_irqrestore(&client_list_lock, flags);
	return NULL;
}

static struct rpcrouter_address_list *rpcrouter_lookup_remote_client(
						  struct rpcrouter_xport *xport,
						  uint32_t cid)
{
	struct rpcrouter_address_list *element;
	unsigned long flags;

	spin_lock_irqsave(&xport->rcl_lock, flags);
	list_for_each_entry(element, &xport->remote_client_list, list) {
		if (element->addr.cid == cid) {
			spin_unlock_irqrestore(&xport->rcl_lock, flags);
			return element;
			}
	}
	spin_unlock_irqrestore(&xport->rcl_lock, flags);
	return NULL;
}

static struct rpcrouter_xport *rpcrouter_locate_route_to_client(
                                                   rpcrouter_address * addr)
{
	struct rpcrouter_xport *c_xport;
	struct rpcrouter_address_list *c_element;
	unsigned long flags, flags2;


	if (addr->pid > RPCROUTER_PROCESSORS_MAX)
		return NULL;

	spin_lock_irqsave(&xport_list_lock, flags);
	list_for_each_entry(c_xport, &xport_list, xport_list) {
		spin_lock_irqsave(&c_xport->rcl_lock, flags2);
		list_for_each_entry(c_element,
					&c_xport->remote_client_list,
					list) {
			if ((c_element->addr.pid == addr->pid) &&
			    (c_element->addr.cid == addr->cid)) {
				spin_unlock_irqrestore(&c_xport->rcl_lock,
						       flags2);
				spin_unlock_irqrestore(&xport_list_lock, flags);
				return c_xport;
			}
		}
		spin_unlock_irqrestore(&c_xport->rcl_lock, flags2);
	}

	spin_unlock_irqrestore(&xport_list_lock, flags);
	return NULL;
}

/*  =============================================
 *  Router To Router Communication and processing
 *  =============================================
 */


/*
 * Process a msg from the router
 * NOTE: xport->lock must be held
 */
static int rpcrouter_process_routermsg(struct rpcrouter_complete_header *hdr,
				       struct rpcrouter_xport *xport)
{
	struct rpcrouter_server *server;
	struct rpcrouter_control_msg  *cntl;
	struct rpcrouter_address_list *remote_client;
	int rc = 0;
	unsigned long flags;

	if (hdr->ph.msg_size != sizeof(struct rpcrouter_control_msg)) {
		printk(KERN_ERR "rpcrouter: R2R msg size (%d) != sizeof cntl msg (%d)\n",
			hdr->ph.msg_size, sizeof(struct rpcrouter_control_msg));
		return -EINVAL;
	}

	cntl = (struct rpcrouter_control_msg *) xport->rx_buffer;
	switch (hdr->rh.msg_type) {
	case RPCROUTER_CTRL_CMD_HELLO:
		/* Send list of servers one at a time */
		cntl->command = RPCROUTER_CTRL_CMD_NEW_SERVER;

		spin_lock_irqsave(&server_list_lock, flags);
		list_for_each_entry(server, &server_list, list) {
			cntl->args.arg_s.pid = server->addr.pid;
			cntl->args.arg_s.cid = server->addr.cid;
			cntl->args.arg_s.prog = server->pv.prog;
			cntl->args.arg_s.ver = server->pv.ver;

			rc = rpcrouter_send_control_msg(xport, cntl);
			if (rc < 0)
				printk(KERN_ERR
					"rpcrouter: Control msg send "
					"failure (%d)\n", rc);
		}
		spin_unlock_irqrestore(&server_list_lock, flags);
		break;

	case RPCROUTER_CTRL_CMD_NEW_SERVER:
		server = rpcrouter_lookup_server(cntl->args.arg_s.prog,
						 cntl->args.arg_s.ver);
		if (!server) {
			server = rpcrouter_create_server(
						cntl->args.arg_s.pid,
						cntl->args.arg_s.cid,
						cntl->args.arg_s.prog,
						cntl->args.arg_s.ver);
			if (!server)
				return -ENOMEM;
#if MSM_RPCROUTER_R2R_DEBUG
			printk(KERN_INFO
				"rpcrouter: New server [PRG %x VER %x]"
				" registered at [PID %x CID %x]\n",
				cntl->args.arg_s.prog,
				cntl->args.arg_s.ver,
				cntl->args.arg_s.pid,
				cntl->args.arg_s.cid);
#endif
			/*
			 * XXX: Verify that its okay to add the
			 * client to our xports remote client list
			 * if we get a NEW_SERVER notification
			 */
			if (!rpcrouter_lookup_remote_client(xport,
						    cntl->args.arg_s.cid)) {
				D("rpcrouter: Adding remote client "
				  "[PID %x CID %x] to xport [XP %x P %x"
				  "]\n",
				  cntl->args.arg_s.pid,
				  cntl->args.arg_s.cid,
				  xport->xport_address.xp,
				  xport->xport_address.port);

				rc = rpcrouter_create_remote_client(xport,
							  cntl->args.arg_s.cid);
				if (rc < 0)
					printk(KERN_ERR
						"rpcrouter:Client create"
						"error (%d)\n", rc);
			}
			wake_up_interruptible(&newserver_wait);
		} else {
			if ((server->addr.pid == cntl->args.arg_s.pid) &&
			    (server->addr.cid == cntl->args.arg_s.pid)) {
				printk(KERN_ERR "rpcrouter: Duplicate svr\n");
			} else {
				server->addr.pid = cntl->args.arg_s.pid;
				server->addr.cid = cntl->args.arg_s.cid;
			}
		}
		break;

	case RPCROUTER_CTRL_CMD_REMOVE_SERVER:
		server = rpcrouter_lookup_server(cntl->args.arg_s.prog,
						 cntl->args.arg_s.ver);
		if (!server) {
			printk(KERN_WARNING
			       "rpcrouter: Skipping removal of unknown"
			       "  svr [PRG %x VER %x]\n",
			       cntl->args.arg_s.prog,
			       cntl->args.arg_s.ver);
		} else {
			rpcrouter_destroy_server(server);
#if MSM_RPCROUTER_R2R_DEBUG
			printk(KERN_INFO
			       "rpcrouter: Svr [PRG %x VER %x] removed"
			       "  by request from [PID %x CID %x]\n",
			       cntl->args.arg_s.prog,
			       cntl->args.arg_s.ver,
			       hdr->rh.src_addr.pid,
			       hdr->rh.src_addr.cid);
#endif
		}
		break;

	case RPCROUTER_CTRL_CMD_REMOVE_CLIENT:
		if (cntl->args.arg_c.pid != RPCROUTER_PID_REMOTE) {
			printk(KERN_ERR
			       "rpcrouter: Denying remote removal of "
			       "local client\n");
			break;
		}
		remote_client = rpcrouter_lookup_remote_client(xport,
						 cntl->args.arg_c.cid);
		if (!remote_client) {
			printk(KERN_WARNING
			       "rpcrouter: Skipping removal of unknown"
			       " remote client [PID %x CID %x]\n",
			       cntl->args.arg_c.pid,
			       cntl->args.arg_c.cid);
		} else {
			spin_lock_irqsave(&xport->rcl_lock, flags);
			list_del(&remote_client->list);
			spin_unlock_irqrestore(&xport->rcl_lock, flags);
			kfree(remote_client);
#if MSM_RPCROUTER_R2R_DEBUG
			printk(KERN_INFO
			       "rpcrouter: Client [PID %x CID %x] "
			       "removed by request from "
			       "[PID %x CID %x]\n",
				cntl->args.arg_c.pid,
				cntl->args.arg_c.cid,
				hdr->rh.src_addr.pid,
				hdr->rh.src_addr.cid);
#endif
		}

		/* Notify local clients of this event */
		printk(KERN_ERR "rpcrouter: LOCAL NOTIFICATION NOT IMP\n");
		rc = -ENOSYS;

		break;
	default:
		rc = -ENOSYS;
	}

	return rc;
}

static void rpcrouter_smdnotify(void *_dev, unsigned event)
{
	struct rpcrouter_xport *xport = (struct rpcrouter_xport *) _dev;
	unsigned long flags;

	BUG_ON(xport->xport_address.xp != RPCROUTER_XPORT_SMD);

	if (event != SMD_EVENT_DATA)
		return;

	D("rpcrouter_smdnotify(): [XP %x P %x] Event 0x%x\n",
	  xport->xport_address.xp, xport->xport_address.port, event);

	spin_lock_irqsave(&worker_thread.lock, flags);
	worker_thread.command = KTHREAD_CMD_DATA;
	worker_thread.xport = xport;
	spin_unlock_irqrestore(&worker_thread.lock, flags);
	wake_up_interruptible(&worker_thread.wait);
}

static void krpcrouterd_process_msg(struct rpcrouter_xport *xport)
{
	struct rpcrouter_client *client;
	struct rpcrouter_complete_header hdr;
	struct rpcrouter_client_read_q *read_queue;
	struct pacmark_hdr *pacmark;
	int len, rc, retry = 0, brtr = 0;
	unsigned long flags;
	char *p;

	BUG_ON(!xport);
	for (;;) {

		len = smd_read_avail(xport->xport_specific.smd.smd_channel);
		if (!len)
			break;
		else if (len < sizeof(struct rpcrouter_complete_header)) {
			printk(KERN_ERR
			       "rpcrouter: data avail (%d) < hdr size (%d)\n",
			       len, sizeof(struct rpcrouter_complete_header));
			goto exit_flush_smd;
		}
		brtr = sizeof(struct rpcrouter_complete_header);
		p = (char *) &hdr;
		retry = 0;
		while (brtr) {
			rc = smd_read(xport->xport_specific.smd.smd_channel,
				      p, brtr);
			p += rc;
			brtr -= rc;
			if (brtr) {
				mdelay(10);
				retry++;
				if (retry == 5) {
					printk(KERN_ERR
					       "krpcrouterd: Too many short "
					       "reads on hdr\n");
					goto exit_flush_smd;
				}
			}
		}
#if DUMP_ALL_RECEIVED_HEADERS
		{
			uint32_t *p = (uint32_t *) &hdr;
			int k;

			printk("RAW HDR:\n");
			for (k = 0; k <= (sizeof(struct rpcrouter_complete_header)/4); k++) {
				if ((!k % 4))
					printk("\n");
				printk("0x%.8x ", *p++);
			}
			printk("\n");
		}
#endif

#if 0
		D("krpcrouterd: [PID %x CID %x] --> "
		  "[PID %x CID %x] (ver %d, type %d, size %d)\n",
		  hdr.rh.src_addr.pid, hdr.rh.src_addr.cid,
		  hdr.ph.addr.pid, hdr.ph.addr.cid,
		  hdr.rh.version, hdr.rh.msg_type, hdr.ph.msg_size);
#endif

		if (hdr.ph.msg_size > RPCROUTER_MSGSIZE_MAX) {
			printk(KERN_ERR
			       "krpcrouterd: msg size %d > max %d\n",
			       hdr.ph.msg_size, RPCROUTER_MSGSIZE_MAX);
			goto exit_flush_smd;
		}

		brtr = hdr.ph.msg_size;
		p = (char *) xport->rx_buffer;
		retry = 0;
		while (brtr) {
			rc = smd_read(xport->xport_specific.smd.smd_channel,
				      p, brtr);
			p += rc;
			brtr -= rc;

			if (brtr) {
				mdelay(10);
				retry++;
				if (retry == 5) {
					printk(KERN_ERR
					       "krpcrouterd: Too many short "
					       "reads on data\n");
					goto exit_flush_smd;
				}
			}
		}
		pacmark = (struct pacmark_hdr *) xport->rx_buffer;

		if (hdr.rh.version != RPCROUTER_VERSION) {
			printk(KERN_ERR
			       "krpcrouterd: Bad ver in msg (%d != %d)\n",
			       hdr.rh.version, RPCROUTER_VERSION);
			goto exit_flush_smd;
		}

		if (hdr.rh.msg_type == RPCROUTER_CTRL_CMD_HELLO) {
			xport->peer_router_address.pid = hdr.rh.src_addr.pid;
			xport->peer_router_address.cid = hdr.rh.src_addr.cid;
			xport->initialized = 1;
			wake_up_interruptible(&xport->hello_wait);
		}

		D("krpcrouterd: [PID %x CID %x] <- [PID %x CID %x] (size %d)\n",
		  hdr.ph.addr.pid, hdr.ph.addr.cid,
		  hdr.rh.src_addr.pid, hdr.rh.src_addr.cid, hdr.ph.msg_size);
#if defined(CONFIG_MSM7X00A_6059_COMPAT)
		if (hdr.ph.confirm_rx)
			printk(KERN_WARNING "krpcrouterd: confirm_rx = %d!\n",
			       hdr.ph.confirm_rx);
#endif

		if (hdr.ph.addr.cid == RPCROUTER_ROUTER_ADDRESS) {
			rc = rpcrouter_process_routermsg(&hdr, xport);
			if (rc < 0)
				printk(KERN_ERR
				       "krpcrouterd: Failed to process R2R msg"
				       "(%d)\n", rc);
			continue;
		}

		/*
		 * Lookup the destination client for this message
		 * Assumption here is that we only need to do local delivery
		 */
		if (hdr.ph.addr.pid != RPCROUTER_PID_LOCAL) {
			printk(KERN_WARNING
			       "krpcrouterd: Reject non local R2R packet\n");
			continue;
		}
		client = rpcrouter_lookup_local_client(hdr.ph.addr.cid);
		if (!client) {
			printk(KERN_ERR "krpcrouterd: No local client match on address "
			  " [PID %x CID %x], dropping pkt\n",
			  hdr.ph.addr.pid, hdr.ph.addr.cid);
			continue;
		}

		if (!rpcrouter_lookup_remote_client(xport,
						    hdr.rh.src_addr.cid)) {
			D("krpcrouterd: Adding remote client [PID %x CID %x]"
			  " to xport [XP %x P %x]\n",
			  hdr.rh.src_addr.pid, hdr.rh.src_addr.cid,
			  xport->xport_address.xp, xport->xport_address.port);

			rc = rpcrouter_create_remote_client(xport,
						   hdr.rh.src_addr.cid);
			if (rc < 0) {
				printk(KERN_ERR
				       "krpcrouterd: Client create err (%d)\n",
				       rc);
				continue;
			}
		}

		/*
		 *  Push data onto the client queue
		 */

		read_queue = kmalloc(sizeof(struct rpcrouter_client_read_q),
				     GFP_KERNEL);
		if (!read_queue) {
			printk(KERN_ERR "krpcrouterd: Out of memory\n");
			return;
		}

		read_queue->src_addr.pid = hdr.rh.src_addr.pid;
		read_queue->src_addr.cid = hdr.rh.src_addr.cid;

		memcpy(&read_queue->pacmark, pacmark,
			sizeof(struct pacmark_hdr));

		read_queue->data_size =
			hdr.ph.msg_size - sizeof(struct pacmark_hdr);

		read_queue->data = kmalloc(read_queue->data_size, GFP_KERNEL);
		if (!read_queue->data) {
			kfree(read_queue);
			printk(KERN_ERR "krpcrouterd: Out of memory\n");
			return;
		}

		memcpy(read_queue->data,
			&xport->rx_buffer[sizeof(struct pacmark_hdr)],
			read_queue->data_size);

#if MSM_RPCROUTER_DEBUG_PKT
		printk(KERN_INFO
			"INCOMING: Src [PID %x CID %x]\n",
			read_queue->src_addr.pid,
			read_queue->src_addr.cid);

		dump_pacmark(&read_queue->pacmark);
		dump_rpc_pkt(read_queue->data, read_queue->data_size);
#endif

#if RPCROUTER_CB_WORKAROUND
		/* If this is a CALL packet, then chances are the client we looked up
		   is wrong, because the A9 RPC router has a bug.  We need to look up
		   the client by the program number among registered servers.
		*/
		{
			uint32_t *data = (uint32_t *)read_queue->data;
			if (data[1] == 0) { /* RPC call? */
				uint32_t prog;
				uint32_t vers;
				struct rpcrouter_server *server;

				prog = be32_to_cpu(data[3]); /* prog */
				vers = be32_to_cpu(data[4]); /* vers */
				server = rpcrouter_lookup_server(prog, vers);
				if (server && server->client != client) {
					D("rpcrouter: (CALL) client override %p --> %p for %08x:%d\n",
						client, server->client, prog, vers);
					server->client->override = client;
					client = server->client;
				} else if (!server) {
#if RPCROUTER_BW_COMP
					printk(KERN_ERR "rpcrouter: can't find server %08x:%d, "
						"will deliver when server registers\n", prog, vers);
					read_queue->prog = prog;
					read_queue->vers = vers;
					spin_lock_irqsave(&unregistered_server_calls_list_lock, flags);
					list_add_tail(&read_queue->list, &unregistered_server_calls_list);
					spin_unlock_irqrestore(&unregistered_server_calls_list_lock, flags);

					return; /* do not add this to the wrong client's read queue! */
#else
					printk(KERN_ERR "rpcrouter: can't find server %08x:%d, call may be routed incorrectly\n", prog, vers);
#endif
				}
			}
		}
#endif

		/*
		 * Keep track of the source of this msg
		 */
		memcpy(&client->cb_addr,
			&read_queue->src_addr,
			sizeof(rpcrouter_address));

		spin_lock_irqsave(&client->read_q_lock, flags);
		list_add_tail(&read_queue->list, &client->read_q);
		spin_unlock_irqrestore(&client->read_q_lock, flags);

		if (client->notify) {
			D("rpc router notifying client...\n");
			client->notify(RPC_DATA_IN);
		}

		wake_up_interruptible(&client->wait_q);
	}

	return;

exit_flush_smd:
	D(KERN_INFO "krpcrouterd: Flushing transport\n");
	len = smd_read_avail(xport->xport_specific.smd.smd_channel);
	rc = smd_read(xport->xport_specific.smd.smd_channel, NULL, len);
	if (rc < 0)
		printk(KERN_ERR
		       "krpcrouterd: Error clearing out channel (%d)\n", rc);
	return;
}

static int rpcrouter_thread(void *data)
{
	struct krpcrouterd_thread *this = (struct krpcrouterd_thread *) data;
	DEFINE_WAIT(__wait);
	int rc = 0;
	unsigned int flags;
	struct sched_param param = { .sched_priority = 1 };

	sched_setscheduler(this->thread, SCHED_FIFO, &param);

	printk(KERN_INFO "krpcrouterd: Starting up\n");

	while (1) {
		int command = 0;
		struct rpcrouter_xport *xport = NULL;

		for (;;) {
			prepare_to_wait(&this->wait, &__wait,
					TASK_INTERRUPTIBLE);
			spin_lock_irqsave(&this->lock, flags);
			if (this->command != KTHREAD_CMD_NONE) {
				command = this->command;
				xport = this->xport;
				this->command = KTHREAD_CMD_NONE;
				this->xport = NULL;
				spin_unlock_irqrestore(&this->lock, flags);
				break;
			}
			spin_unlock_irqrestore(&this->lock, flags);
			if (!signal_pending(current)) {
				schedule();
				continue;
			}
			break;
		}

		finish_wait(&this->wait, &__wait);
		if (signal_pending(current)) {
			rc = -ERESTARTSYS;
			break;
		}
		if (command == KTHREAD_CMD_EXIT) {
			rc = 0;
			break;
		} else if (command == KTHREAD_CMD_DATA)
			krpcrouterd_process_msg(xport);
		else
			printk(KERN_ERR
			       "krpcrouterd: Unknown cmd (%d)\n", command);
	}
	printk(KERN_INFO "krpcrouterd: Shutting down (%d)\n", rc);
	return rc;
}

/*  ================================
 *  SMD xport creation / destruction
 *  ================================
 */
static int rpcrouter_destroy_smd_xport_channel(uint32_t channel)
{
	struct rpcrouter_xport *c_xport, *n_xport;
	struct rpcrouter_address_list *c_addr, *n_addr;
	struct rpcrouter_control_msg msg;
	int	rc = 0;
	unsigned long flags, flags2;

	D("rpcrouter_destroy_smd_xport_channel(): channel %d\n", channel);

	spin_lock_irqsave(&xport_list_lock, flags);
	list_for_each_entry_safe(c_xport, n_xport, &xport_list, xport_list) {
		if (c_xport->xport_address.xp == RPCROUTER_XPORT_SMD &&
		    c_xport->xport_address.port == channel) {
			memset(&msg, 0, sizeof(struct rpcrouter_control_msg));
			msg.command = RPCROUTER_CTRL_CMD_BYE;

			rc = rpcrouter_send_control_msg(c_xport, &msg);
			if (rc < 0)
				goto out;
			rc = smd_close(c_xport->xport_specific.smd.smd_channel);
			if (rc < 0)
				goto out;

			list_del(&c_xport->xport_list);
			spin_lock_irqsave(&c_xport->rcl_lock, flags2);
			list_for_each_entry_safe(c_addr,
						 n_addr,
						 &c_xport->remote_client_list,
						 list) {
				list_del(&c_addr->list);
				kfree(c_addr);
			}
			spin_unlock_irqrestore(&c_xport->rcl_lock, flags2);

			kfree(c_xport);
			rc = 0;
			goto out;
		}
	}

	/*
	 * Transport not found; Qualcomm returns success in this case - wierdos
	 */
out:
	spin_unlock_irqrestore(&xport_list_lock, flags);
	return rc;
}

static int rpcrouter_create_smd_xport_channel(uint32_t channel)
{
	struct rpcrouter_xport *xport, *c_xport;
	struct rpcrouter_control_msg msg;
	int	rc;
	unsigned long flags;

	/*
	 * Check for duplicate xport
	 */
	spin_lock_irqsave(&xport_list_lock, flags);
	list_for_each_entry(c_xport, &xport_list, xport_list) {
		if (c_xport->xport_address.xp == RPCROUTER_XPORT_SMD &&
			c_xport->xport_address.port == channel) {
			spin_unlock_irqrestore(&xport_list_lock, flags);
			/* Qualcomm returns success on duplicate open  */
			return 0;
		}
	}

	xport = kmalloc(sizeof(struct rpcrouter_xport), GFP_KERNEL);
	if (!xport)
		return -ENOMEM;

	memset(xport, 0, sizeof(struct rpcrouter_xport));
	xport->xport_address.xp = RPCROUTER_XPORT_SMD;
	xport->xport_address.port = channel;
	init_waitqueue_head(&xport->hello_wait);
	spin_lock_init(&xport->rcl_lock);

	INIT_LIST_HEAD(&xport->remote_client_list);

	xport->peer_router_address.cid = RPCROUTER_ROUTER_ADDRESS;
	xport->peer_router_address.pid = 0;
	xport->initialized = 0;

	list_add_tail(&xport->xport_list, &xport_list);
	spin_unlock_irqrestore(&xport_list_lock, flags);

	rc = smd_open(channel,
			&xport->xport_specific.smd.smd_channel,
			xport, rpcrouter_smdnotify);
	if (rc < 0)
		return rc;
	spin_lock_init(&xport->xport_specific.smd.smd_lock);
	/*
	 * We need to wait for the remote to send us a HELLO msg before we
	 * can continue.
	 */
	wait_event_interruptible(xport->hello_wait, (xport->initialized));
	if (signal_pending(current))
		return -ERESTARTSYS;

	memset(&msg, 0, sizeof(struct rpcrouter_control_msg));
	msg.command = RPCROUTER_CTRL_CMD_HELLO;
	rc = rpcrouter_send_control_msg(xport, &msg);
	if (rc < 0) {
		smd_close(xport->xport_specific.smd.smd_channel);
		xport->xport_specific.smd.smd_channel = NULL;
		return rc;
	}

	return 0;
}

/*  ================================
 *  struct file_operations callbacks
 *  ================================
 */
static int rpcrouter_open(struct inode *inode, struct file *filp)
{
	int rc;
	struct rpcrouter_client *client;

	rc = nonseekable_open(inode, filp);
	if (rc < 0)
		return rc;

	client = rpcrouter_create_local_client(inode->i_rdev);
	if (!client)
		return -ENOMEM;

	filp->private_data = client;
	return 0;
}

static int rpcrouter_release(struct inode *inode, struct file *filp)
{
	struct rpcrouter_client *client;
	client = (struct rpcrouter_client *) filp->private_data;

	return rpcrouter_destroy_local_client(client);
}

static ssize_t rpcrouter_callback_read(struct file *filp, char __user *buf,
				size_t count, loff_t *ppos)
{
	struct rpcrouter_client *client;
	struct rpcrouter_client_read_q *read_q_entry;
	DEFINE_WAIT(__wait);
	unsigned long flags;
	int rc = -1;

	client = (struct rpcrouter_client *) filp->private_data;
	if (count < RPCROUTER_MSGSIZE_MAX)
		return -EINVAL;

	for (;;) {
		prepare_to_wait(&client->wait_q, &__wait, TASK_INTERRUPTIBLE);
		spin_lock_irqsave(&client->read_q_lock, flags);
		if (!list_empty(&client->read_q))
			break;
		spin_unlock_irqrestore(&client->read_q_lock, flags);
		if (!signal_pending(current)) {
			schedule();
			continue;
		}
		break;
	}
	finish_wait(&client->wait_q, &__wait);

	if (signal_pending(current))
		return -ERESTARTSYS;

	read_q_entry = list_first_entry(&client->read_q,
					struct rpcrouter_client_read_q, list);
	BUG_ON(!read_q_entry);

	list_del(&read_q_entry->list);
	spin_unlock_irqrestore(&client->read_q_lock, flags);

	if (read_q_entry->data_size < count)
		count = read_q_entry->data_size;

	if (copy_to_user(buf, read_q_entry->data, count))
		rc = -EFAULT;
	else
		rc = count;

	kfree(read_q_entry->data);
	kfree(read_q_entry);
	return rc;
}

static ssize_t rpcrouter_read(struct file *filp, char __user *buf,
				size_t count, loff_t *ppos)
{
	struct rpcrouter_client *client;
	int rc;
	void *buffer;

	client = (struct rpcrouter_client *) filp->private_data;

	/*
	 * XXX: Qualcomm bug here? 'count' may need to be
	 *	> RPCROUTER_MAX_MSG_SIZE + sizeof(rpcrouter_address)
	 */
	if (count < RPCROUTER_MSGSIZE_MAX)
		return -EINVAL;

	rc = rpcrouter_kernapi_read(client, &buffer, count, -1);
	if (rc < 0)
		return rc;

	count = rc;

#if RPCROUTER_CB_WORKAROUND
	{
		/* If this is a REPLY packet in response to a CALL that was overridden
		   then we will route the reply to the original sender. */
		uint32_t type = be32_to_cpu(((uint32_t *)buffer)[1]);
		if (type == 1 && client->override) {
			struct rpcrouter_client *override = client->override;
			client->override = NULL;
			D("rpcrouter: (REPLY) client override %p --> %p\n",
				client, override);
			client = override;
		}
	}
#endif

	if (copy_to_user(buf, buffer, count)) {
		printk(KERN_ERR "rpcrouter: could not copy all read data to user!\n");
		rc = -EFAULT;
	}

	kfree(buffer);
	return rc;
}

static ssize_t rpcrouter_callback_write(struct file *filp, const char __user *buf,
				size_t count, loff_t *ppos)
{
	struct rpcrouter_client	*client;
	rpcrouter_address dest_addr;
	int rc = 0;
	void *k_buffer;

	client = (struct rpcrouter_client *) filp->private_data;

	if (count > RPCROUTER_MSGSIZE_MAX)
		return -EINVAL;

	if (count < sizeof(rpcrouter_address))
		return -EINVAL;

	k_buffer = kmalloc(count, GFP_KERNEL);
	if (!k_buffer)
		return -ENOMEM;

	if (copy_from_user(k_buffer, buf, count)) {
		rc = -EFAULT;
		goto write_out_free;
	}

	memcpy(&dest_addr, &client->cb_addr, sizeof(rpcrouter_address));

	rc = rpcrouter_kernapi_write(client,
				     &dest_addr,
				     k_buffer,
				     count);
	if (rc < 0) {
		D("rpcrouter_write(): Write to client [PID %x CID %x]"
		  "failed (%d)\n",
		  dest_addr.pid, dest_addr.cid, rc);
		goto write_out_free;
	}

	rc = count;
write_out_free:
	kfree(k_buffer);
	return rc;
}

static ssize_t rpcrouter_write(struct file *filp, const char __user *buf,
				size_t count, loff_t *ppos)
{
	struct rpcrouter_client	*client;
	rpcrouter_address dest_addr;
	int rc = 0;
	void *k_buffer;

	client = (struct rpcrouter_client *) filp->private_data;

	if (count > RPCROUTER_MSGSIZE_MAX)
		return -EINVAL;

	if (count < sizeof(rpcrouter_address))
		return -EINVAL;

	k_buffer = kmalloc(count, GFP_KERNEL);
	if (!k_buffer)
		return -ENOMEM;

	if (copy_from_user(k_buffer, buf, count)) {
		rc = -EFAULT;
		goto write_out_free;
	}

	memcpy(&dest_addr, &client->bound_dest, sizeof(rpcrouter_address));

	rc = rpcrouter_kernapi_write(client,
				     &dest_addr,
				     k_buffer,
				     count);
	if (rc < 0) {
		D("rpcrouter_write(): Write to client [PID %x CID %x]"
		  "failed (%d)\n",
		  dest_addr.pid, dest_addr.cid, rc);
		goto write_out_free;
	}

	rc = count;
write_out_free:
	kfree(k_buffer);
	return rc;
}

static unsigned int rpcrouter_poll(struct file *filp, struct poll_table_struct *wait)
{
	struct rpcrouter_client *client;
	unsigned mask = 0;
	client = (struct rpcrouter_client *) filp->private_data;

	/* If there's data already in the read queue, return POLLIN.  Else, wait for the
	   requested amount of time, and check again. */
	if (!list_empty(&client->read_q))
		mask |= POLLIN;

	if (!mask) {
		poll_wait(filp, &client->wait_q, wait);
		if (!list_empty(&client->read_q))
			mask |= POLLIN;
	}

	return mask;
}

static long rpcrouter_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct rpcrouter_client *client;
	struct rpcrouter_ioctl_xport_args xport_args;
	struct rpcrouter_ioctl_server_args server_args;
	struct rpcrouter_ioctl_dest_args dest_args;
	int rc;
	uint32_t mtu = 0;
	long timeout_jiffies;

	client = (struct rpcrouter_client *) filp->private_data;
	switch (cmd) {
	case RPC_ROUTER_IOCTL_OPEN_XPORT:
		rc = copy_from_user(&xport_args, (void *) arg,
				    sizeof(xport_args));
		if (rc < 0)
			break;
		if (xport_args.addr.xp == RPCROUTER_XPORT_NONE) {
			rc = 0;
			break;
		}

		if (xport_args.addr.xp != RPCROUTER_XPORT_ALL &&
		    xport_args.addr.xp != RPCROUTER_XPORT_SMD) {
			rc = -ENOSYS;
			break;
		}

		rc = rpcrouter_create_smd_xport_channel(xport_args.addr.port);
		break;

	case RPC_ROUTER_IOCTL_CLOSE_XPORT:
		rc = copy_from_user(&xport_args, (void *) arg,
				    sizeof(xport_args));
		if (rc < 0)
			break;
		if (xport_args.addr.xp == RPCROUTER_XPORT_NONE) {
			rc = 0;
			break;
		}
		if (xport_args.addr.xp != RPCROUTER_XPORT_ALL &&
		    xport_args.addr.xp != RPCROUTER_XPORT_SMD) {
			rc = -ENOSYS;
			break;
		}
		rc = rpcrouter_destroy_smd_xport_channel(xport_args.addr.port);
		break;

	case RPC_ROUTER_IOCTL_GET_MTU:
		/* Qualcomms code just returns 0 here and copies stack garbage
		 * into the  passed in arg. Lets be a bit saner and return 0;
		 */
		mtu = 0;
		rc = put_user(mtu, (unsigned int *) arg);
		break;

	case RPC_ROUTER_IOCTL_REGISTER_SERVER:
		rc = copy_from_user(&server_args, (void *) arg,
				    sizeof(server_args));
		if (rc < 0)
			break;
		rpcrouter_kernapi_register_server(client, &server_args);
		break;

	case RPC_ROUTER_IOCTL_UNREGISTER_SERVER:
		rc = copy_from_user(&server_args, (void *) arg,
				    sizeof(server_args));
		if (rc < 0)
			break;

		rpcrouter_kernapi_unregister_server(client, &server_args);
		break;

	case RPC_ROUTER_IOCTL_GET_DEST:
		rc = copy_from_user(&dest_args, (void *) arg,
				    sizeof(dest_args));
		if (rc < 0)
			break;

		if (dest_args.data.input.timeout < -1)
			return -EINVAL;
		else if (dest_args.data.input.timeout == -1)
			timeout_jiffies = 0;
		else
			timeout_jiffies = dest_args.data.input.timeout * HZ;
		rc = rpcrouter_kernapi_getdest(client,
						dest_args.data.input.pv.prog,
						dest_args.data.input.pv.ver,
						timeout_jiffies,
						&dest_args.data.output);
		if (rc < 0)
			return rc;

		rc = copy_to_user((void *) arg, &dest_args, sizeof(dest_args));
		break;
	case RPC_ROUTER_IOCTL_GET_ROUTER_STATS:
	case RPC_ROUTER_IOCTL_GET_CLIENT_STATS:
	case RPC_ROUTER_IOCTL_GET_ROUTING_TABLE:
		rc = -ENOSYS;
		break;
	default:
		rc = -EINVAL;
		break;
	}

	return rc;
}

/*  ====================================
 *  API for kernel producers / consumers
 *  ====================================
 */

void rpcrouter_kernapi_setup_request(struct rpc_request_hdr *hdr, uint32_t prog,
                                     uint32_t vers, uint32_t proc)
{
	memset(hdr, 0, sizeof(struct rpc_request_hdr));
	hdr->xid = cpu_to_be32(++next_xid);
	hdr->rpc_vers = cpu_to_be32(2);
	hdr->prog = cpu_to_be32(prog);
	hdr->vers = cpu_to_be32(vers);
	hdr->procedure = cpu_to_be32(proc);
}

int rpcrouter_kernapi_openxport(rpcrouter_xport_address * addr)
{
	if (addr->xp == RPCROUTER_XPORT_NONE)
		return 0;

	if (addr->xp != RPCROUTER_XPORT_ALL &&
	    addr->xp != RPCROUTER_XPORT_SMD)
		return -ENOSYS;

	return rpcrouter_create_smd_xport_channel(addr->port);
}

int rpcrouter_kernapi_open(rpcrouterclient_t **client)
{
	if (!client)
		return -EINVAL;

	*client = rpcrouter_create_local_client(MKDEV(0, 0));
	if (!(*client))
		return -ENOMEM;

	return 0;
}

int rpcrouter_kernapi_close(rpcrouterclient_t *client)
{
	return rpcrouter_destroy_local_client(client);
}

int rpcrouter_kernapi_write(rpcrouterclient_t *client,
			    rpcrouter_address *dest,
			    void *buffer,
			    int count)
{
	struct rpcrouter_xport *xport;
	struct rpcrouter_complete_header hdr;
	struct rpc_request_hdr *rq = buffer;
	struct pacmark_hdr pacmark;
	int rc = 0;
	unsigned long flags;

	if (count > RPCROUTER_MSGSIZE_MAX || !count)
		return -EINVAL;

	/*
	 * Peek at the write request to see if the client is writing to
	 * a server which it has access to. (identified by prog/vers tuple)
	 */

	rc = client_validate_permission(client,
					be32_to_cpu(rq->prog),
					be32_to_cpu(rq->vers));
	if (rc < 0)
		return rc;

	xport = rpcrouter_locate_route_to_client(dest);
	if (!xport) {
		printk(KERN_ERR
			"rpcrouter_kernapi_write(): No route to client "
			"[PID %x CID %x]\n", dest->pid, dest->cid);
		return -EHOSTUNREACH;
	}

	/* Create routing header */
	memset(&hdr, 0, sizeof(struct rpcrouter_complete_header));
	hdr.rh.msg_type = RPCROUTER_CTRL_CMD_DATA;
	hdr.rh.version = RPCROUTER_VERSION;
	hdr.rh.src_addr.pid = client->addr.pid;
	hdr.rh.src_addr.cid = client->addr.cid;

	hdr.ph.addr.pid = dest->pid;
	hdr.ph.addr.cid = dest->cid;
	hdr.ph.msg_size = count + sizeof(struct pacmark_hdr);

	/* Create pacmark header */
	memset(&pacmark, 0, sizeof(pacmark));
	pacmark.data.cooked.length = count;
	pacmark.data.cooked.message_id = ++next_pacmarkid;
	pacmark.data.cooked.last_pkt = 1;

	spin_lock_irqsave(&xport->xport_specific.smd.smd_lock, flags);

	/* Write routing header */
	rc = smd_write(xport->xport_specific.smd.smd_channel,
		       &hdr, sizeof(struct rpcrouter_complete_header));
	if (rc < 0) {
		spin_unlock_irqrestore(&xport->xport_specific.smd.smd_lock,
				       flags);
		return rc;
	}

#if MSM_RPCROUTER_DEBUG_PKT
	printk(KERN_INFO
	       "OUTGOING: Dest [PID %x CID %x]\n", dest->pid, dest->cid);
	dump_pacmark(&pacmark);
	dump_rpc_pkt(buffer, count);
#endif

	/* Write pacmark header */
	rc = smd_write(xport->xport_specific.smd.smd_channel,
			&pacmark,
			sizeof(struct pacmark_hdr));
	if (rc < 0) {
		spin_unlock_irqrestore(&xport->xport_specific.smd.smd_lock,
				       flags);
		return rc;
	}

	/* Write data */
	rc = smd_write(xport->xport_specific.smd.smd_channel,
			buffer, hdr.ph.msg_size - sizeof(struct pacmark_hdr));
	spin_unlock_irqrestore(&xport->xport_specific.smd.smd_lock, flags);
	if (rc < 0)
		return rc;

	return count;
}

/*
 * NOTE: It is the responsibility of the caller to kfree buffer
 */
int rpcrouter_kernapi_read(rpcrouterclient_t *client,
			   void **buffer, unsigned user_len,
			   long timeout)
{
	struct rpcrouter_client_read_q *read_q_entry;
	DEFINE_WAIT(__wait);
	unsigned long flags;
	int rc = -1;

	if (timeout == -1) {
		for (;;) {
			prepare_to_wait(&client->wait_q, &__wait, TASK_INTERRUPTIBLE);
			spin_lock_irqsave(&client->read_q_lock, flags);
			if (!list_empty(&client->read_q))
				break;
			spin_unlock_irqrestore(&client->read_q_lock, flags);
			if (!signal_pending(current)) {
				schedule();
				continue;
			}
			break;
		}
		finish_wait(&client->wait_q, &__wait);
	} else {
		for (;;) {
			prepare_to_wait(&client->wait_q, &__wait, TASK_INTERRUPTIBLE);
			spin_lock_irqsave(&client->read_q_lock, flags);
			if (!list_empty(&client->read_q))
				break;
			spin_unlock_irqrestore(&client->read_q_lock, flags);
			if (!signal_pending(current)) {
				rc = schedule_timeout(timeout);
				if (!rc)
					break;
				continue;
			}
			break;
		}
		finish_wait(&client->wait_q, &__wait);
		if (rc == 0)
			return -ETIMEDOUT;
	}

	if (signal_pending(current))
		return -ERESTARTSYS;

	read_q_entry = list_first_entry(&client->read_q,
					struct rpcrouter_client_read_q, list);

	BUG_ON(!read_q_entry);

	rc = read_q_entry->data_size;
	if (rc > user_len) {
		spin_unlock_irqrestore(&client->read_q_lock, flags);
		return -ETOOSMALL;
	}

	list_del(&read_q_entry->list);

	spin_unlock_irqrestore(&client->read_q_lock, flags);

	*buffer = read_q_entry->data;

	kfree(read_q_entry);
	return rc;
}

int rpcrouter_kernapi_getdest(rpcrouterclient_t *client,
			      uint32_t prog,
			      uint32_t vers,
			      long timeout,
			      rpcrouter_address *dest_addr)
{
	struct rpcrouter_server *server = NULL;
	int rc;

	timeout = wait_event_interruptible_timeout(newserver_wait,
			     (server = rpcrouter_lookup_server(prog, vers)),
						   timeout);
	if (timeout < 0)
		return timeout;
	if (!server)
		return -EHOSTUNREACH;

	rc = client_validate_permission(client, prog, vers);
	if (rc < 0)
		return rc;

	dest_addr->pid = server->addr.pid;
	dest_addr->cid = server->addr.cid;
	return 0;
}

struct rpcrouter_server *rpcrouter_kernapi_register_server(struct rpcrouter_client *client,
		struct rpcrouter_ioctl_server_args *server_args)
{
	int rc;

	struct rpcrouter_control_msg msg;
	struct rpcrouter_xport *xport;
	unsigned long flags;

	struct rpcrouter_server *server;

	server = rpcrouter_create_server(client->addr.pid,
					 client->addr.cid,
					 server_args->progver.prog,
					 server_args->progver.ver);
	if (!server)
		goto done;

#if RPCROUTER_CB_WORKAROUND
	server->client = client;
#endif

#if RPCROUTER_BW_COMP
	{
		struct rpcrouter_client_read_q *read_queue;
		unsigned long flags, unlocked = 0;

		spin_lock_irqsave(&unregistered_server_calls_list_lock, flags);
		list_for_each_entry(read_queue, &unregistered_server_calls_list, list) {
			if (read_queue->prog == server_args->progver.prog &&
			    read_queue->vers == server_args->progver.ver) {
				list_del(&read_queue->list);
				spin_unlock_irqrestore(&unregistered_server_calls_list_lock, flags);

				D("rpcrouter: delivering saved RPC call for server %08x:%d\n",
					read_queue->prog, read_queue->vers);

				/* Keep track of the source of this msg. */
				memcpy(&client->cb_addr, &read_queue->src_addr, sizeof(rpcrouter_address));

				spin_lock_irqsave(&client->read_q_lock, flags);
				list_add_tail(&read_queue->list, &client->read_q);
				spin_unlock_irqrestore(&client->read_q_lock, flags);
				wake_up_interruptible(&client->wait_q);

				unlocked = 1;
				break;
			}
		}
		if (!unlocked)
			spin_unlock_irqrestore(&unregistered_server_calls_list_lock, flags);
	}
#endif

	msg.command = RPCROUTER_CTRL_CMD_NEW_SERVER;
	msg.args.arg_s.pid = client->addr.pid;
	msg.args.arg_s.cid = client->addr.cid;
	msg.args.arg_s.prog = server_args->progver.prog;
	msg.args.arg_s.ver = server_args->progver.ver;

	spin_lock_irqsave(&xport_list_lock, flags);
	list_for_each_entry(xport, &xport_list, xport_list) {
		rc = rpcrouter_send_control_msg(xport, &msg);
		if (rc < 0)
			break;
	}
	spin_unlock_irqrestore(&xport_list_lock, flags);
done:
	return server;
}

int rpcrouter_kernapi_unregister_server(struct rpcrouter_client *client,
		struct rpcrouter_ioctl_server_args *server_args)
{
	struct rpcrouter_server *server;
	server = rpcrouter_lookup_server(server_args->progver.prog,
					 server_args->progver.ver);

	if (!server)
		return -ENOENT;
	rpcrouter_destroy_server(server);
	return 0;
}

int rpcrouter_kernapi_register_notify(struct rpcrouter_client **client,
					int (*func)(int))
{
	(*client)->notify = func;
	return 0;
}

/*  ================================
 *  Module init / exit
 *  ================================
 */

static int __init rpcrouter_init(void)
{
	int rc;
	int major;

	printk(KERN_INFO "rpcrouter_init(): starting\n");

	rpcrouter_class = class_create(THIS_MODULE, "oncrpc");
	if (IS_ERR(rpcrouter_class)) {
		rc = -ENOMEM;
		goto fail_nocleanup;
	}

	rc = alloc_chrdev_region(&rpcrouter_devno, 0,
				 RPCROUTER_MAX_REMOTE_SERVERS + 1,
				 "oncrpc");
	if (rc < 0) {
		printk(KERN_ERR
		       "rpcrouter: Failed to alloc chardev region (%d)\n", rc);
		goto fail_destroy_class;
	}

	major = MAJOR(rpcrouter_devno);
	rpcrouter_device = device_create(rpcrouter_class, NULL,
					 rpcrouter_devno, "%.8x:%d",
					 0, 0);
	if (IS_ERR(rpcrouter_device)) {
		rc = -ENOMEM;
		goto fail_unregister_cdev_region;
	}

	cdev_init(&rpcrouter_cdev, &rpcrouter_router_fops);
	rpcrouter_cdev.owner = THIS_MODULE;

	rc = cdev_add(&rpcrouter_cdev, rpcrouter_devno, 1);
	if (rc < 0)
		goto fail_destroy_device;

	INIT_LIST_HEAD(&client_list);
	INIT_LIST_HEAD(&xport_list);

	init_waitqueue_head(&newserver_wait);
	memset(&worker_thread, 0, sizeof(worker_thread));
	init_waitqueue_head(&worker_thread.wait);
	spin_lock_init(&worker_thread.lock);
	worker_thread.thread = kthread_create(rpcrouter_thread, &worker_thread,
					      "krpcrouterd");
	if (IS_ERR(worker_thread.thread)) {
		printk(KERN_ERR
		       "rpcrouter: Err creating worker thread\n");
		goto fail_remove_cdev;
	}

	wake_up_process(worker_thread.thread);

#if RPCROUTER_BW_COMP
	{
		rpcrouter_xport_address xport_addr;
		struct rpcrouter_ioctl_server_args server_args;

        	xport_addr.xp = RPCROUTER_XPORT_SMD;
	        xport_addr.port = 2;
		rc = rpcrouter_kernapi_openxport(&xport_addr);
		if (rc < 0) {
			printk(KERN_ERR
			       "rpcrouter: Err opening xport\n");
			goto fail_remove_cdev;
		}
		rc = rpcrouter_kernapi_open(&bw_client);
		if (rc < 0) {
			printk(KERN_ERR
			       "rpcrouter: Err creating client\n");
			goto fail_closexport;
		}

		server_args.progver.prog = ONCRPC_BW_COMP_LOCAL_PROG;
		server_args.progver.ver  = ONCRPC_BW_COMP_LOCAL_VERS;
		bw_server = rpcrouter_kernapi_register_server(bw_client, &server_args);
		if (!bw_server) {
			printk(KERN_ERR
				"rpcrouter: Err registering BW server %08x:%d\n",
				server_args.progver.prog, server_args.progver.ver);
			rc = -ENOENT;
			goto fail_close_bw_client;
		}
	}
#endif

	D("rpcrouter_init(): done\n");
	return 0;

#if RPCROUTER_BW_COMP
fail_close_bw_client:
	rpcrouter_kernapi_close(bw_client);
fail_closexport:
	/* nothing yet */
#endif
fail_remove_cdev:
	cdev_del(&rpcrouter_cdev);

fail_unregister_cdev_region:
	unregister_chrdev_region(rpcrouter_devno,
				 RPCROUTER_MAX_REMOTE_SERVERS + 1);

fail_destroy_device:
	device_destroy(rpcrouter_class, rpcrouter_devno);
fail_destroy_class:
	class_destroy(rpcrouter_class);
fail_nocleanup:
	return rc;
}

static void __exit rpcrouter_exit(void)
{
	struct rpcrouter_xport *xp, *n_xport;
	struct rpcrouter_client *client, *n_client;
	struct rpcrouter_server *server, *n_server;
	struct rpcrouter_client_read_q *c_q, *n_q;
	unsigned long flags, flags2;

	cdev_del(&rpcrouter_cdev);
	unregister_chrdev_region(rpcrouter_devno,
				 RPCROUTER_MAX_REMOTE_SERVERS + 1);
	device_destroy(rpcrouter_class, rpcrouter_devno);
	class_destroy(rpcrouter_class);

	spin_lock_irqsave(&client_list_lock, flags2);
	list_for_each_entry_safe(client, n_client, &client_list, client_list) {
		spin_lock_irqsave(&client->read_q_lock, flags);
		list_for_each_entry_safe(c_q, n_q, &client->read_q, list) {
			kfree(c_q->data);
			list_del(&c_q->list);
		}
		spin_unlock_irqrestore(&client->read_q_lock, flags);
		list_del(&client->client_list);
		kfree(client);
	}
	spin_unlock_irqrestore(&client_list_lock, flags2);

	list_for_each_entry_safe(xp, n_xport, &xport_list, xport_list) {
		if (xp->xport_address.xp == RPCROUTER_XPORT_SMD)
			rpcrouter_destroy_smd_xport_channel(xp->xport_address.xp);
	}

	spin_lock_irqsave(&server_list_lock, flags2);
	list_for_each_entry_safe(server, n_server, &server_list, list) {
		list_del(&server->list);
		kfree(server);
	}
	spin_unlock_irqrestore(&server_list_lock, flags2);
}

module_init(rpcrouter_init);
module_exit(rpcrouter_exit);
MODULE_DESCRIPTION("MSM RPC Router");
MODULE_AUTHOR("San Mehat <san@android.com>");
MODULE_LICENSE("GPL");
