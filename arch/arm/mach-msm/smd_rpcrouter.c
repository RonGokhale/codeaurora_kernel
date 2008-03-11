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
#include <linux/platform_device.h>

#include <asm/arch/msm_smd.h>
#include <linux/msm_rpcrouter.h>
#include "smd_rpcrouter.h"

#define MSM_RPCROUTER_DEBUG 0
#define MSM_RPCROUTER_DEBUG_PKT 0
#define MSM_RPCROUTER_R2R_DEBUG 0
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
static int create_platform_device(struct rpcrouter_server *server);

static dev_t rpcrouter_devno; /* Device number for the router itself */
static int next_minor = 1; /* Next minor # available for a remote server */
static struct cdev rpcrouter_cdev;
static struct class *rpcrouter_class;
static struct device *rpcrouter_device;

static LIST_HEAD(local_clients);
static LIST_HEAD(remote_clients);

static LIST_HEAD(server_list);

#if RPCROUTER_BW_COMP
#define ONCRPC_BW_COMP_LOCAL_PROG 0x3000FFFE
#define ONCRPC_BW_COMP_LOCAL_VERS 1
static LIST_HEAD(unregistered_server_calls_list);
static DEFINE_SPINLOCK(unregistered_server_calls_list_lock);
static rpcrouterclient_t *bw_client;
static struct rpcrouter_server *bw_server;
#endif

static smd_channel_t *smd_channel;
static int initialized;
static DECLARE_WAIT_QUEUE_HEAD(hello_wait);

static DEFINE_SPINLOCK(local_clients_lock);
static DEFINE_SPINLOCK(remote_clients_lock);
static DEFINE_SPINLOCK(server_list_lock);
static DEFINE_SPINLOCK(smd_lock);

static struct krpcrouterd_thread worker_thread;
static uint32_t next_xid;
static uint8_t next_pacmarkid;
static wait_queue_head_t newserver_wait;
static uint8_t rx_buffer[RPCROUTER_MSGSIZE_MAX];
static rpcrouter_address remote_router_address;

static struct platform_device rpcrouter_pdev = {
	.name		= "oncrpc_router",
	.id		= -1,
};
static struct work_struct create_pdev_work;
static struct work_struct create_rpcrouter_pdev_work;

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
 */
static int rpcrouter_send_control_msg(struct rpcrouter_control_msg *msg)
{
	struct rpcrouter_complete_header hdr;
	int rc;
	unsigned long flags;

#if MSM_RPCROUTER_R2R_DEBUG
	printk(KERN_INFO "rpcrouter_send_control_msg(): Command 0x%x\n",
			msg->command);
#endif

	if (!(msg->command == RPCROUTER_CTRL_CMD_HELLO) && !initialized) {
		printk(KERN_ERR "rpcrouter_send_control_msg(): Warning, "
		       "router not initialized\n");
		return -EINVAL;
	}

	memset(&hdr, 0, sizeof(struct rpcrouter_complete_header));

	hdr.rh.version = RPCROUTER_VERSION;
	hdr.rh.msg_type = msg->command;
	hdr.rh.src_addr.pid = RPCROUTER_PID_LOCAL;
	hdr.rh.src_addr.cid = RPCROUTER_ROUTER_ADDRESS;
	hdr.ph.msg_size = sizeof(struct rpcrouter_control_msg);
	hdr.ph.addr.pid = remote_router_address.pid;
	hdr.ph.addr.cid = remote_router_address.cid;

	spin_lock_irqsave(&smd_lock, flags);
	/* Write header */
	rc = smd_write(smd_channel, &hdr,
		       sizeof(struct rpcrouter_complete_header));
	if (rc < 0) {
		spin_unlock_irqrestore(&smd_lock, flags);
		return rc;
	}

	/* Write data */
	rc = smd_write(smd_channel, msg, hdr.ph.msg_size);
	spin_unlock_irqrestore(&smd_lock, flags);
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
		if (rc < 0)
			goto out_fail;
	}
	return server;
out_fail:
	spin_lock_irqsave(&server_list_lock, flags);
	list_del(&server->list);
	spin_unlock_irqrestore(&server_list_lock, flags);
	kfree(server);
	return ERR_PTR(rc);
}

static int create_platform_device(struct rpcrouter_server *server)
{
	char pdev_name[21];

	sprintf(pdev_name, "rpcsvr_%.8x:%d", server->pv.prog, server->pv.ver);
	memset(&server->p_device, 0, sizeof(struct platform_device));

	server->p_device.base.id = -1;
	server->p_device.base.name = kmalloc(strlen(pdev_name)+1, GFP_KERNEL);
	if (!server->p_device.base.name) {
		printk(KERN_ERR "Unable to create platform device (OOM)\n");
		return -ENOMEM;
	}
	strcpy(server->p_device.base.name, pdev_name);

	memcpy(&server->p_device.addr, &server->addr, sizeof(rpcrouter_address));
	D("%s: creating device '%s'\n", __FUNCTION__, pdev_name);
	platform_device_register(&server->p_device.base);
	return 0;
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

	spin_lock_irqsave(&local_clients_lock, flags);
	list_add_tail(&client->clients, &local_clients);
	spin_unlock_irqrestore(&local_clients_lock, flags);
	return client;
}

static int rpcrouter_destroy_local_client(struct rpcrouter_client *client)
{
	int rc;
	struct rpcrouter_control_msg msg;

	msg.command = RPCROUTER_CTRL_CMD_REMOVE_CLIENT;
	msg.args.arg_c.pid = client->addr.pid;
	msg.args.arg_c.cid = client->addr.cid;

	rc = rpcrouter_send_control_msg(&msg);
	if (rc < 0)
		return rc;

	list_del(&client->clients);
	kfree(client);
	return 0;
}

static int rpcrouter_create_remote_client(uint32_t cid)
{
	struct rpcrouter_r_client *new_c;
	unsigned long flags;

	new_c = kmalloc(sizeof(struct rpcrouter_r_client), GFP_KERNEL);
	if (!new_c)
		return -ENOMEM;
	memset(new_c, 0, sizeof(struct rpcrouter_r_client));

	new_c->addr.cid = cid;
	new_c->addr.pid = RPCROUTER_PID_REMOTE;
	init_waitqueue_head(&new_c->quota_wait);
	spin_lock_init(&new_c->quota_lock);

	spin_lock_irqsave(&remote_clients_lock, flags);
	list_add_tail(&new_c->list, &remote_clients);
	spin_unlock_irqrestore(&remote_clients_lock, flags);
	return 0;
}

static struct rpcrouter_client *rpcrouter_lookup_local_client(uint32_t cid)
{
	struct rpcrouter_client *client;
	unsigned long flags;

	spin_lock_irqsave(&local_clients_lock, flags);
	list_for_each_entry(client, &local_clients, clients) {
		if (client->addr.cid == cid) {
			spin_unlock_irqrestore(&local_clients_lock, flags);
			return client;
		}
	}
	spin_unlock_irqrestore(&local_clients_lock, flags);
	return NULL;
}

static struct rpcrouter_r_client *rpcrouter_lookup_remote_client(
						  uint32_t cid)
{
	struct rpcrouter_r_client *element;
	unsigned long flags;

	spin_lock_irqsave(&remote_clients_lock, flags);
	list_for_each_entry(element, &remote_clients, list) {
		if (element->addr.cid == cid) {
			spin_unlock_irqrestore(&remote_clients_lock, flags);
			return element;
		}
	}
	spin_unlock_irqrestore(&remote_clients_lock, flags);
	return NULL;
}

/*  =============================================
 *  Router To Router Communication and processing
 *  =============================================
 */


/*
 * Process a msg from the router
 */
static int rpcrouter_process_routermsg(struct rpcrouter_complete_header *hdr)
{
	struct rpcrouter_server *server;
	struct rpcrouter_control_msg  *cntl;
	struct rpcrouter_r_client *r_client;
	int rc = 0;
	unsigned long flags;

	if (hdr->ph.msg_size != sizeof(struct rpcrouter_control_msg)) {
		printk(KERN_ERR "rpcrouter: R2R msg size (%d) != sizeof cntl msg (%d)\n",
			hdr->ph.msg_size, sizeof(struct rpcrouter_control_msg));
		return -EINVAL;
	}

	cntl = (struct rpcrouter_control_msg *) rx_buffer;
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

			rc = rpcrouter_send_control_msg(cntl);
			if (rc < 0)
				printk(KERN_ERR
					"rpcrouter: Control msg send "
					"failure (%d)\n", rc);
		}
		spin_unlock_irqrestore(&server_list_lock, flags);
		break;

	case RPCROUTER_CTRL_CMD_RESUME_TX:
		r_client = rpcrouter_lookup_remote_client(cntl->args.arg_c.cid);
		if (!r_client) {
			printk(KERN_ERR "rpcrouter: Unable to resume client\n");
			break;
		}
		spin_lock_irqsave(&r_client->quota_lock, flags);
		r_client->tx_quota_cntr = 0;
		spin_unlock_irqrestore(&r_client->quota_lock, flags);
		wake_up_interruptible(&r_client->quota_wait);
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
			 * client to our remote client list
			 * if we get a NEW_SERVER notification
			 */
			if (!rpcrouter_lookup_remote_client(
						    cntl->args.arg_s.cid)) {
				D("rpcrouter: Adding remote client "
				  "[PID %x CID %x]\n",
				  cntl->args.arg_s.pid,
				  cntl->args.arg_s.cid);

				rc = rpcrouter_create_remote_client(
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
		r_client = rpcrouter_lookup_remote_client(
						 cntl->args.arg_c.cid);
		if (!r_client) {
			printk(KERN_WARNING
			       "rpcrouter: Skipping removal of unknown"
			       " remote client [PID %x CID %x]\n",
			       cntl->args.arg_c.pid,
			       cntl->args.arg_c.cid);
		} else {
			spin_lock_irqsave(&remote_clients_lock, flags);
			list_del(&r_client->list);
			spin_unlock_irqrestore(&remote_clients_lock, flags);
			kfree(r_client);
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

static void create_rpcrouter_pdev_worker(struct work_struct *work)
{
	platform_device_register(&rpcrouter_pdev);
}

static void create_pdev_worker(struct work_struct *work)
{
	struct rpcrouter_server *server;
	int rc;

	list_for_each_entry(server, &server_list, list) {
		if (server->addr.pid == RPCROUTER_PID_REMOTE) {
			rc = create_platform_device(server);
			if (rc < 0)
				printk(KERN_ERR "Err creating pdev\n");
		}
	}
}

static void rpcrouter_smdnotify(void *_dev, unsigned event)
{
	unsigned long flags;

	if (event != SMD_EVENT_DATA)
		return;

	D("%s: Event 0x%x\n", __FUNCTION__, event);

	spin_lock_irqsave(&worker_thread.lock, flags);
	worker_thread.command = KTHREAD_CMD_DATA;
	spin_unlock_irqrestore(&worker_thread.lock, flags);
	D("%s: Waking up worker\n", __FUNCTION__);
	wake_up_interruptible(&worker_thread.wait);
}

static void krpcrouterd_process_msg(void)
{
	struct rpcrouter_client *client;
	struct rpcrouter_complete_header hdr;
	struct rpcrouter_client_read_q *read_queue;
	struct pacmark_hdr *pacmark;
	int len, rc, retry = 0, brtr = 0;
	unsigned long flags;
	char *p;

	for (;;) {

		len = smd_read_avail(smd_channel);
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
			rc = smd_read(smd_channel, p, brtr);
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
		p = (char *) rx_buffer;
		retry = 0;
		while (brtr) {
			rc = smd_read(smd_channel, p, brtr);
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
		pacmark = (struct pacmark_hdr *) rx_buffer;

		if (hdr.rh.version != RPCROUTER_VERSION) {
			printk(KERN_ERR
			       "krpcrouterd: Bad ver in msg (%d != %d)\n",
			       hdr.rh.version, RPCROUTER_VERSION);
			goto exit_flush_smd;
		}

		if (hdr.rh.msg_type == RPCROUTER_CTRL_CMD_HELLO) {
			remote_router_address.pid = hdr.rh.src_addr.pid;
			remote_router_address.cid = hdr.rh.src_addr.cid;
			initialized = 1;
			INIT_WORK(&create_rpcrouter_pdev_work, create_rpcrouter_pdev_worker);
			schedule_work(&create_rpcrouter_pdev_work);
			wake_up_interruptible(&hello_wait);
		}

		D("krpcrouterd: [PID %x CID %x] <- [PID %x CID %x] (size %d)\n",
		  hdr.ph.addr.pid, hdr.ph.addr.cid,
		  hdr.rh.src_addr.pid, hdr.rh.src_addr.cid, hdr.ph.msg_size);

		if (hdr.ph.addr.cid == RPCROUTER_ROUTER_ADDRESS) {
			rc = rpcrouter_process_routermsg(&hdr);
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

		if (!rpcrouter_lookup_remote_client(
						    hdr.rh.src_addr.cid)) {
			D("krpcrouterd: Adding remote client [PID %x CID %x]\n"
			  hdr.rh.src_addr.pid, hdr.rh.src_addr.cid);

			rc = rpcrouter_create_remote_client(
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
		read_queue->confirm_rx = hdr.ph.confirm_rx;

		memcpy(read_queue->data,
			&rx_buffer[sizeof(struct pacmark_hdr)],
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
	len = smd_read_avail(smd_channel);
	rc = smd_read(smd_channel, NULL, len);
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
	int created_pdevs = 0;

	INIT_WORK(&create_pdev_work, create_pdev_worker);

	sched_setscheduler(this->thread, SCHED_FIFO, &param);

	printk(KERN_INFO "krpcrouterd: Starting up\n");

	while (1) {
		int command = 0;

		rc = 0;
		for (;;) {
			prepare_to_wait(&this->wait, &__wait,
					TASK_INTERRUPTIBLE);
			spin_lock_irqsave(&this->lock, flags);
			if (this->command != KTHREAD_CMD_NONE) {
				command = this->command;
				this->command = KTHREAD_CMD_NONE;
				D("%s: Woken up with cmd = %d\n", __FUNCTION__, command);
				spin_unlock_irqrestore(&this->lock, flags);
				break;
			}
			spin_unlock_irqrestore(&this->lock, flags);
			if (!signal_pending(current)) {
				if (created_pdevs)
					schedule();
				else {
					rc = schedule_timeout(1 * HZ);
					if (!rc) {
						rc = -ETIMEDOUT;
						break;
					}
				}
				continue;
			}
			break;
		}

		finish_wait(&this->wait, &__wait);
		D("%s: Cmd = %d, rc = %d, sig_pend = %d\n",
		  __FUNCTION__, command, rc, signal_pending(current));
		if (rc == -ETIMEDOUT) {
			if (!created_pdevs) {
				created_pdevs = 1;
				schedule_work(&create_pdev_work);
			}
			continue;
		}
		if (signal_pending(current)) {
			rc = -ERESTARTSYS;
			break;
		}
		if (command == KTHREAD_CMD_EXIT) {
			rc = 0;
			break;
		} else if (command == KTHREAD_CMD_DATA)
			krpcrouterd_process_msg();
		else
			printk(KERN_ERR
			       "krpcrouterd: Unknown cmd (%d)\n", command);
	}
	printk(KERN_INFO "krpcrouterd: Shutting down (%d)\n", rc);
	return rc;
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

	if (signal_pending(current)) {
		spin_unlock_irqrestore(&client->read_q_lock, flags);
		return -ERESTARTSYS;
	}

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
	struct rpcrouter_ioctl_server_args server_args;
	struct rpcrouter_ioctl_dest_args dest_args;
	int rc;
	uint32_t mtu = 0;
	long timeout_jiffies;

	client = (struct rpcrouter_client *) filp->private_data;
	switch (cmd) {

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
	struct rpcrouter_complete_header hdr;
	struct rpc_request_hdr *rq = buffer;
	struct pacmark_hdr pacmark;
	struct rpcrouter_r_client *r_client;
	int rc = 0;
	unsigned long flags;
	DEFINE_WAIT(__wait);

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

	if (dest->pid != RPCROUTER_PID_REMOTE)
		return -EHOSTUNREACH;
	
	r_client = rpcrouter_lookup_remote_client(dest->cid);

	if (!r_client) {
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

	for (;;) {
		prepare_to_wait(&r_client->quota_wait, &__wait, TASK_INTERRUPTIBLE);
		spin_lock_irqsave(&r_client->quota_lock, flags);
		if (r_client->tx_quota_cntr < RPCROUTER_DEFAULT_RX_QUOTA)
			break;
		spin_unlock_irqrestore(&r_client->quota_lock, flags);
		if (!signal_pending(current)) {
			schedule();
			continue;
		}
		break;
	}
	finish_wait(&r_client->quota_wait, &__wait);

	if (signal_pending(current)) {
		spin_unlock_irqrestore(&r_client->quota_lock, flags);
		return -ERESTARTSYS;
	}

	r_client->tx_quota_cntr++;
	if (r_client->tx_quota_cntr == RPCROUTER_DEFAULT_RX_QUOTA) 
		hdr.ph.confirm_rx = 1;

	spin_unlock_irqrestore(&r_client->quota_lock, flags);

	spin_lock_irqsave(&smd_lock, flags);

	/* Write routing header */
	rc = smd_write(smd_channel, &hdr,
		       sizeof(struct rpcrouter_complete_header));
	if (rc < 0) {
		spin_unlock_irqrestore(&smd_lock, flags);
		return rc;
	}

#if MSM_RPCROUTER_DEBUG_PKT
	printk(KERN_INFO
	       "OUTGOING: Dest [PID %x CID %x]\n", dest->pid, dest->cid);
	dump_pacmark(&pacmark);
	dump_rpc_pkt(buffer, count);
#endif

	/* Write pacmark header */
	rc = smd_write(smd_channel, &pacmark,
			sizeof(struct pacmark_hdr));
	if (rc < 0) {
		spin_unlock_irqrestore(&smd_lock, flags);
		return rc;
	}

	/* Write data */
	rc = smd_write(smd_channel,
			buffer, hdr.ph.msg_size - sizeof(struct pacmark_hdr));
	spin_unlock_irqrestore(&smd_lock, flags);
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

	if (read_q_entry->confirm_rx) {
		struct rpcrouter_control_msg msg;

		memset(&msg, 0, sizeof(msg));
		msg.command = RPCROUTER_CTRL_CMD_RESUME_TX;

		/* if we were redirected, make sure we ACK based on
		 * the original target, not the redirected target
		 */
		if (client->override) {
			msg.args.arg_c.pid = client->override->addr.pid;
			msg.args.arg_c.cid = client->override->addr.cid;
		} else {
			msg.args.arg_c.pid = client->addr.pid;
			msg.args.arg_c.cid = client->addr.cid;
		}
		printk("%s: confirming rx\n", __FUNCTION__);
		rc = rpcrouter_send_control_msg(&msg);
	}

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

	rc = rpcrouter_send_control_msg(&msg);
	if (rc < 0)
		return ERR_PTR(rc);
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

static int msm_rpcrouter_probe(struct platform_device *pdev)
{
	struct rpcrouter_control_msg msg;
	int rc;
	int major;

	/* Initialize what we need to start processing */
	INIT_LIST_HEAD(&local_clients);
	INIT_LIST_HEAD(&remote_clients);

	remote_router_address.cid = RPCROUTER_ROUTER_ADDRESS;
	remote_router_address.pid = 0;

	init_waitqueue_head(&newserver_wait);
	memset(&worker_thread, 0, sizeof(worker_thread));
	init_waitqueue_head(&worker_thread.wait);
	spin_lock_init(&worker_thread.lock);
	worker_thread.thread = kthread_create(rpcrouter_thread, &worker_thread,
					      "krpcrouterd");
	if (IS_ERR(worker_thread.thread)) {
		printk(KERN_ERR
		       "rpcrouter: Err creating worker thread\n");
		return PTR_ERR(worker_thread.thread);
	}

	wake_up_process(worker_thread.thread);

	/* Create the device nodes */
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

	/* Open up SMD channel 2 */
	initialized = 0;
	rc = smd_open(2, &smd_channel, NULL, rpcrouter_smdnotify);
	if (rc < 0)
		goto fail_remove_cdev;

	/*
	 * We need to wait for the remote to send us a HELLO msg before we
	 * can continue. Opening channel 2 will kick this all off
	 */
	wait_event_interruptible(hello_wait, (initialized));
	if (signal_pending(current)) {
		rc = -ERESTARTSYS;
		goto fail_close_smd;
	}

	memset(&msg, 0, sizeof(struct rpcrouter_control_msg));
	msg.command = RPCROUTER_CTRL_CMD_HELLO;
	rc = rpcrouter_send_control_msg(&msg);
	if (rc < 0)
		goto fail_close_smd;

#if RPCROUTER_BW_COMP
	{
		struct rpcrouter_ioctl_server_args server_args;

		rc = rpcrouter_kernapi_open(&bw_client);
		if (rc < 0) {
			printk(KERN_ERR
			       "rpcrouter: Err creating client\n");
			goto fail_close_smd;
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

	return 0;

#if RPCROUTER_BW_COMP
fail_close_bw_client:
	rpcrouter_kernapi_close(bw_client);
#endif
fail_close_smd:
	smd_close(smd_channel);
	smd_channel = NULL;
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

static struct platform_driver msm_smd_channel2_driver = {
	.probe		= msm_rpcrouter_probe,
	.driver		= {
			.name	= "smd_channel_02",
			.owner	= THIS_MODULE,
	},
};

static int __init rpcrouter_init(void)
{
	return platform_driver_register(&msm_smd_channel2_driver);
}

module_init(rpcrouter_init);
MODULE_DESCRIPTION("MSM RPC Router");
MODULE_AUTHOR("San Mehat <san@android.com>");
MODULE_LICENSE("GPL");
