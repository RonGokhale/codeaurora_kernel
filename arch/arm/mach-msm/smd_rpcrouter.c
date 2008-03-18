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

/* TODO: reassembly */
/* TODO: handle cases where smd_write() will tempfail due to full fifo */
/* TODO: better pacmark handling on read to avoid memmove() yuck yuck yuck! */
/* TODO: thread priority? schedule a work to bump it? */
/* TODO: maybe make server_list_lock a mutex */

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
#include <linux/poll.h>
#include <asm/uaccess.h>
#include <asm/byteorder.h>
#include <linux/platform_device.h>

#include <asm/arch/msm_smd.h>
#include "smd_rpcrouter.h"

#define TRACE_R2R_MSG 0
#define TRACE_R2R_RAW 0
#define TRACE_RPC_MSG 0

#define MSM_RPCROUTER_DEBUG 0
#define MSM_RPCROUTER_DEBUG_PKT 0
#define MSM_RPCROUTER_R2R_DEBUG 0
#define DUMP_ALL_RECEIVED_HEADERS 0

#if MSM_RPCROUTER_DEBUG
#define D(x...) printk(x)
#else
#define D(x...) do {} while (0)
#endif

#if TRACE_R2R_MSG
#define RR(x...) printk("[RR] "x)
#else
#define RR(x...) do {} while (0)
#endif

#if TRACE_RPC_MSG
#define IO(x...) printk("[RPC] "x)
#else
#define IO(x...) do {} while (0)
#endif

static LIST_HEAD(local_endpoints);
static LIST_HEAD(remote_endpoints);

static LIST_HEAD(server_list);

static smd_channel_t *smd_channel;
static int initialized;
static wait_queue_head_t newserver_wait;

static DEFINE_SPINLOCK(local_endpoints_lock);
static DEFINE_SPINLOCK(remote_endpoints_lock);
static DEFINE_SPINLOCK(server_list_lock);
static DEFINE_SPINLOCK(smd_lock);

static struct workqueue_struct *rpcrouter_workqueue;

static uint32_t next_xid;
static uint8_t next_pacmarkid;

static void do_read_data(struct work_struct *work);
static void do_create_pdevs(struct work_struct *work);
static void do_create_rpcrouter_pdev(struct work_struct *work);

static DECLARE_WORK(work_read_data, do_read_data);
static DECLARE_WORK(work_create_pdevs, do_create_pdevs);
static DECLARE_WORK(work_create_rpcrouter_pdev, do_create_rpcrouter_pdev);

#define RR_STATE_IDLE    0
#define RR_STATE_HEADER  1
#define RR_STATE_BODY    2
#define RR_STATE_ERROR   3

struct rr_context {
	struct rr_packet *pkt;
	uint8_t *ptr;
	uint32_t state; /* current assembly state */
	uint32_t count; /* bytes needed in this state */
};

struct rr_context the_rr_context;

static struct platform_device rpcrouter_pdev = {
	.name		= "oncrpc_router",
	.id		= -1,
};


static int rpcrouter_send_control_msg(union rr_control_msg *msg)
{
	struct rr_header hdr;
	unsigned long flags;
	int need;

	if (!(msg->cmd == RPCROUTER_CTRL_CMD_HELLO) && !initialized) {
		printk(KERN_ERR "rpcrouter_send_control_msg(): Warning, "
		       "router not initialized\n");
		return -EINVAL;
	}

	hdr.version = RPCROUTER_VERSION;
	hdr.type = msg->cmd;
	hdr.src_pid = RPCROUTER_PID_LOCAL;
	hdr.src_cid = RPCROUTER_ROUTER_ADDRESS;
	hdr.confirm_rx = 0;
	hdr.size = sizeof(*msg);
	hdr.dst_pid = 0;
	hdr.dst_cid = RPCROUTER_ROUTER_ADDRESS;

	/* TODO: what if channel is full? */

	need = sizeof(hdr) + hdr.size;
	spin_lock_irqsave(&smd_lock, flags);
	while (smd_write_avail(smd_channel) < need) {
		spin_unlock_irqrestore(&smd_lock, flags);
		msleep(250);
		spin_lock_irqsave(&smd_lock, flags);
	}
	smd_write(smd_channel, &hdr, sizeof(hdr));
	smd_write(smd_channel, msg, hdr.size);
	spin_unlock_irqrestore(&smd_lock, flags);
	return 0;
}

static struct rr_server *rpcrouter_create_server(uint32_t pid,
							uint32_t cid,
							uint32_t prog,
							uint32_t ver)
{
	struct rr_server *server;
	unsigned long flags;
	int rc;

	server = kmalloc(sizeof(struct rr_server), GFP_KERNEL);
	if (!server)
		return ERR_PTR(-ENOMEM);

	memset(server, 0, sizeof(struct rr_server));
	server->pid = pid;
	server->cid = cid;
	server->prog = prog;
	server->vers = ver;

	spin_lock_irqsave(&server_list_lock, flags);
	list_add_tail(&server->list, &server_list);
	spin_unlock_irqrestore(&server_list_lock, flags);

	if (pid == RPCROUTER_PID_REMOTE) {
		rc = msm_rpcrouter_create_server_cdev(server);
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

static void rpcrouter_destroy_server(struct rr_server *server)
{
	unsigned long flags;

	spin_lock_irqsave(&server_list_lock, flags);
	list_del(&server->list);
	spin_unlock_irqrestore(&server_list_lock, flags);
	device_destroy(msm_rpcrouter_class, server->device_number);
	kfree(server);
}

static struct rr_server *rpcrouter_lookup_server(uint32_t prog,
							uint32_t ver)
{
	struct rr_server *server;
	unsigned long flags;

	spin_lock_irqsave(&server_list_lock, flags);
	list_for_each_entry(server, &server_list, list) {
		if (server->prog == prog
		 && server->vers == ver) {
			spin_unlock_irqrestore(&server_list_lock, flags);
			return server;
		}
	}
	spin_unlock_irqrestore(&server_list_lock, flags);
	return NULL;
}

static struct rr_server *rpcrouter_lookup_server_by_dev(dev_t dev)
{
	struct rr_server *server;
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


struct msm_rpc_endpoint *msm_rpcrouter_create_local_endpoint(dev_t dev)
{
	struct msm_rpc_endpoint *ept;
	unsigned long flags;

	ept = kmalloc(sizeof(struct msm_rpc_endpoint), GFP_KERNEL);
	if (!ept)
		return NULL;
	memset(ept, 0, sizeof(struct msm_rpc_endpoint));

	/* mark no reply outstanding */
	ept->reply_pid = 0xffffffff;

	ept->cid = (uint32_t) ept;
	ept->pid = RPCROUTER_PID_LOCAL;
	ept->dev = dev;

	if ((dev != msm_rpcrouter_devno) && (dev != MKDEV(0, 0))) {
		struct rr_server *srv;
		/*
		 * This is a userspace client which opened
		 * a program/ver devicenode. Bind the client
		 * to that destination
		 */
		srv = rpcrouter_lookup_server_by_dev(dev);
		/* TODO: bug? really? */
		BUG_ON(!srv);

		ept->dst_pid = srv->pid;
		ept->dst_cid = srv->cid;
		ept->dst_prog = cpu_to_be32(srv->prog);
		ept->dst_vers = cpu_to_be32(srv->vers);
	} else {
		/* mark not connected */
		ept->dst_pid = 0xffffffff;
	}

	init_waitqueue_head(&ept->wait_q);
	INIT_LIST_HEAD(&ept->read_q);
	spin_lock_init(&ept->read_q_lock);

	spin_lock_irqsave(&local_endpoints_lock, flags);
	list_add_tail(&ept->list, &local_endpoints);
	spin_unlock_irqrestore(&local_endpoints_lock, flags);
	return ept;
}

int msm_rpcrouter_destroy_local_endpoint(struct msm_rpc_endpoint *ept)
{
	int rc;
	union rr_control_msg msg;

	msg.cmd = RPCROUTER_CTRL_CMD_REMOVE_CLIENT;
	msg.cli.pid = ept->pid;
	msg.cli.cid = ept->cid;

	RR("x REMOVE_CLIENT id=%d:%08x\n", ept->pid, ept->cid);
	rc = rpcrouter_send_control_msg(&msg);
	if (rc < 0)
		return rc;

	list_del(&ept->list);
	kfree(ept);
	return 0;
}

static int rpcrouter_create_remote_endpoint(uint32_t cid)
{
	struct rr_remote_endpoint *new_c;
	unsigned long flags;

	new_c = kmalloc(sizeof(struct rr_remote_endpoint), GFP_KERNEL);
	if (!new_c)
		return -ENOMEM;
	memset(new_c, 0, sizeof(struct rr_remote_endpoint));

	new_c->cid = cid;
	new_c->pid = RPCROUTER_PID_REMOTE;
	init_waitqueue_head(&new_c->quota_wait);
	spin_lock_init(&new_c->quota_lock);

	spin_lock_irqsave(&remote_endpoints_lock, flags);
	list_add_tail(&new_c->list, &remote_endpoints);
	spin_unlock_irqrestore(&remote_endpoints_lock, flags);
	return 0;
}

static struct msm_rpc_endpoint *rpcrouter_lookup_local_endpoint(uint32_t cid)
{
	struct msm_rpc_endpoint *ept;
	unsigned long flags;

	spin_lock_irqsave(&local_endpoints_lock, flags);
	list_for_each_entry(ept, &local_endpoints, list) {
		if (ept->cid == cid) {
			spin_unlock_irqrestore(&local_endpoints_lock, flags);
			return ept;
		}
	}
	spin_unlock_irqrestore(&local_endpoints_lock, flags);
	return NULL;
}

static struct rr_remote_endpoint *rpcrouter_lookup_remote_endpoint(uint32_t cid)
{
	struct rr_remote_endpoint *ept;
	unsigned long flags;

	spin_lock_irqsave(&remote_endpoints_lock, flags);
	list_for_each_entry(ept, &remote_endpoints, list) {
		if (ept->cid == cid) {
			spin_unlock_irqrestore(&remote_endpoints_lock, flags);
			return ept;
		}
	}
	spin_unlock_irqrestore(&remote_endpoints_lock, flags);
	return NULL;
}

static int process_control_msg(struct rr_packet *pkt)
{
	union rr_control_msg *msg = pkt->data;
	union rr_control_msg ctl;
	struct rr_server *server;
	struct rr_remote_endpoint *r_ept;
	int rc = 0;
	unsigned long flags;

	if (pkt->data_len != sizeof(*msg)) {
		printk(KERN_ERR "rpcrouter: r2r msg size %d != %d\n",
		       pkt->data_len, sizeof(*msg));
		return -EINVAL;
	}

	switch (msg->cmd) {
	case RPCROUTER_CTRL_CMD_HELLO:
		RR("o HELLO\n");

		RR("x HELLO\n");
		memset(&ctl, 0, sizeof(ctl));
		ctl.cmd = RPCROUTER_CTRL_CMD_HELLO;
		rpcrouter_send_control_msg(&ctl);

		initialized = 1;

		/* Send list of servers one at a time */
		ctl.cmd = RPCROUTER_CTRL_CMD_NEW_SERVER;

		/* TODO: long time to hold a spinlock... */
		spin_lock_irqsave(&server_list_lock, flags);
		list_for_each_entry(server, &server_list, list) {
			ctl.srv.pid = server->pid;
			ctl.srv.cid = server->cid;
			ctl.srv.prog = server->prog;
			ctl.srv.vers = server->vers;

			RR("x NEW_SERVER id=%d:%08x prog=%08x:%d\n",
			   server->pid, server->cid,
			   server->prog, server->vers);

			rpcrouter_send_control_msg(&ctl);
		}
		spin_unlock_irqrestore(&server_list_lock, flags);

		queue_work(rpcrouter_workqueue, &work_create_rpcrouter_pdev);
		break;

	case RPCROUTER_CTRL_CMD_RESUME_TX:
		RR("o RESUME_TX id=%d:%08x\n", msg->cli.pid, msg->cli.cid);

		r_ept = rpcrouter_lookup_remote_endpoint(msg->cli.cid);
		if (!r_ept) {
			printk(KERN_ERR
			       "rpcrouter: Unable to resume client\n");
			break;
		}
		spin_lock_irqsave(&r_ept->quota_lock, flags);
		r_ept->tx_quota_cntr = 0;
		spin_unlock_irqrestore(&r_ept->quota_lock, flags);
		wake_up_interruptible(&r_ept->quota_wait);
		break;

	case RPCROUTER_CTRL_CMD_NEW_SERVER:
		RR("o NEW_SERVER id=%d:%08x prog=%08x:%d\n",
		   msg->srv.pid, msg->srv.cid, msg->srv.prog, msg->srv.vers);

		server = rpcrouter_lookup_server(msg->srv.prog, msg->srv.vers);

		if (!server) {
			server = rpcrouter_create_server(
				msg->srv.pid, msg->srv.cid,
				msg->srv.prog, msg->srv.vers);
			if (!server)
				return -ENOMEM;
			/*
			 * XXX: Verify that its okay to add the
			 * client to our remote client list
			 * if we get a NEW_SERVER notification
			 */
			if (!rpcrouter_lookup_remote_endpoint(msg->srv.cid)) {
				rc = rpcrouter_create_remote_endpoint(
					msg->srv.cid);
				if (rc < 0)
					printk(KERN_ERR
						"rpcrouter:Client create"
						"error (%d)\n", rc);
			}
			schedule_work(&work_create_pdevs);
			wake_up_interruptible(&newserver_wait);
		} else {
			if ((server->pid == msg->srv.pid) &&
			    (server->cid == msg->srv.cid)) {
				printk(KERN_ERR "rpcrouter: Duplicate svr\n");
			} else {
				server->pid = msg->srv.pid;
				server->cid = msg->srv.cid;
			}
		}
		break;

	case RPCROUTER_CTRL_CMD_REMOVE_SERVER:
		RR("o REMOVE_SERVER prog=%08x:%d\n",
		   msg->srv.prog, msg->srv.vers);
		server = rpcrouter_lookup_server(msg->srv.prog, msg->srv.vers);
		if (server)
			rpcrouter_destroy_server(server);
		break;

	case RPCROUTER_CTRL_CMD_REMOVE_CLIENT:
		RR("o REMOVE_CLIENT id=%d:%08x\n", msg->cli.pid, msg->cli.cid);
		if (msg->cli.pid != RPCROUTER_PID_REMOTE) {
			printk(KERN_ERR
			       "rpcrouter: Denying remote removal of "
			       "local client\n");
			break;
		}
		r_ept = rpcrouter_lookup_remote_endpoint(msg->cli.cid);
		if (r_ept) {
			spin_lock_irqsave(&remote_endpoints_lock, flags);
			list_del(&r_ept->list);
			spin_unlock_irqrestore(&remote_endpoints_lock, flags);
			kfree(r_ept);
		}

		/* Notify local clients of this event */
		printk(KERN_ERR "rpcrouter: LOCAL NOTIFICATION NOT IMP\n");
		rc = -ENOSYS;

		break;
	default:
		RR("o UNKNOWN(%08x)\n", msg->cmd);
		rc = -ENOSYS;
	}

	return rc;
}

static void do_create_rpcrouter_pdev(struct work_struct *work)
{
	platform_device_register(&rpcrouter_pdev);
}

static void do_create_pdevs(struct work_struct *work)
{
	unsigned long flags;
	struct rr_server *server;

	/* TODO: race if destroyed while being registered */
	spin_lock_irqsave(&server_list_lock, flags);
	list_for_each_entry(server, &server_list, list) {
		if (server->pid == RPCROUTER_PID_REMOTE) {
			if (server->pdev_name[0] == 0) {
				spin_unlock_irqrestore(&server_list_lock,
						       flags);
				msm_rpcrouter_create_server_pdev(server);
				schedule_work(&work_create_pdevs);
				return;
			}
		}
	}
	spin_unlock_irqrestore(&server_list_lock, flags);
}

static void rpcrouter_smdnotify(void *_dev, unsigned event)
{
	if (event != SMD_EVENT_DATA)
		return;

	queue_work(rpcrouter_workqueue, &work_read_data);
}

static void *rr_malloc(unsigned sz)
{
	void *ptr = kmalloc(sz, GFP_KERNEL);
	if (ptr)
		return ptr;

	printk(KERN_ERR "rpcrouter: kmalloc of %d failed, retrying...\n", sz);
	do {
		ptr = kmalloc(sz, GFP_KERNEL);
	} while (!ptr);

	return ptr;
}

static void process_packet(struct rr_packet *pkt)
{
	unsigned long flags;
	struct msm_rpc_endpoint *ept;
	uint32_t n;

	if (pkt->hdr.type != RPCROUTER_CTRL_CMD_DATA) {
		process_control_msg(pkt);
		return;
	}

	if (pkt->hdr.dst_pid != RPCROUTER_PID_LOCAL) {
		printk(KERN_WARNING "rpcrouter: rejecting nonlocal packet\n");
		goto fail;
	}

	RR("o DATA %d bytes\n", pkt->data_len);

	ept = rpcrouter_lookup_local_endpoint(pkt->hdr.dst_cid);
	if (!ept) {
		printk(KERN_ERR "rpcrouter: no local ept for cid=%08x\n",
		       pkt->hdr.dst_cid);
		goto fail;
	}

	if (!rpcrouter_lookup_remote_endpoint(pkt->hdr.src_cid)) {
		int rc = rpcrouter_create_remote_endpoint(pkt->hdr.src_cid);
		if (rc < 0) {
			printk(KERN_ERR
			       "rpcrouter: cannot create remote ept\n");
			goto fail;
		}
	}

	/* validate data packet */
	if (pkt->data_len < sizeof(n)) {
		printk(KERN_ERR "rpcrouter: runt packet (%d bytes)\n",
		       pkt->data_len);
		goto fail;
	}
	memcpy(&n, pkt->data, sizeof(n));

	if (!PACMARK_LAST(n)) {
		printk(KERN_ERR "rpcrouter: fragments not supported\n");
		goto fail;
	}
	if (PACMARK_LEN(n) != (pkt->data_len - sizeof(n))) {
		printk(KERN_ERR "rpcrouter: headers mismatched (%d != %d)\n",
		       PACMARK_LEN(n), pkt->data_len - sizeof(n));
		goto fail;
	}

	RR("o DATA %d bytes to ept=%p %d:%08x\n", pkt->data_len,
	   ept, pkt->hdr.dst_pid, pkt->hdr.dst_cid);
	/* enqueue packet */
	spin_lock_irqsave(&ept->read_q_lock, flags);
	list_add_tail(&pkt->list, &ept->read_q);
	spin_unlock_irqrestore(&ept->read_q_lock, flags);

	wake_up_interruptible(&ept->wait_q);
	return;
fail:
	kfree(pkt->data);
	kfree(pkt);
}

static int is_invalid_header(struct rr_header *hdr)
{
#if TRACE_R2R_RAW
	RR("- ver=%d type=%d src=%d:%08x crx=%d siz=%d dst=%d:%08x\n",
	   hdr->version, hdr->type, hdr->src_pid, hdr->src_cid,
	   hdr->confirm_rx, hdr->size, hdr->dst_pid, hdr->dst_cid);
#endif
	if (hdr->size > RPCROUTER_MSGSIZE_MAX) {
		printk(KERN_ERR "rpcrouter: msg size %d > max %d\n",
		       hdr->size, RPCROUTER_MSGSIZE_MAX);
		return 1;
	}
	if (hdr->version != RPCROUTER_VERSION) {
		printk(KERN_ERR "rpcrouter: version %d != %d\n",
		       hdr->version, RPCROUTER_VERSION);
		return 1;
	}
	return 0;
}

/* process data from the SMD channel and assemble packets */
static void do_read_data(struct work_struct *work)
{
	struct rr_context *ctxt = &the_rr_context;
	struct rr_packet *pkt = ctxt->pkt;
	int rc;

	for (;;) {
		switch (ctxt->state) {
		case RR_STATE_IDLE:
			ctxt->pkt = pkt = rr_malloc(sizeof(struct rr_packet));
			ctxt->ptr = (void *) &pkt->hdr;
			ctxt->count = sizeof(pkt->hdr);
			ctxt->state = RR_STATE_HEADER;
			/* fall through */

		case RR_STATE_HEADER:
		case RR_STATE_BODY:
			/* TODO is this safe in non-irq context? */
			rc = smd_read(smd_channel, ctxt->ptr, ctxt->count);
			if (rc < 1) {
				if (rc == 0)
					return;

				printk(KERN_ERR
				       "rpcrouter: smd_read err %d\n", rc);
				ctxt->state = RR_STATE_ERROR;
				continue;
			}
			ctxt->ptr += rc;
			ctxt->count -= rc;
			if (ctxt->count != 0)
				continue;

			if (ctxt->state == RR_STATE_HEADER) {
				if (is_invalid_header(&pkt->hdr)) {
					ctxt->state = RR_STATE_ERROR;
					continue;
				}
				pkt->data_len = pkt->hdr.size;
				pkt->data = rr_malloc(pkt->data_len);
				ctxt->count = pkt->data_len;
				ctxt->ptr = pkt->data;
				ctxt->state = RR_STATE_BODY;
			} else {
				process_packet(pkt);
				ctxt->pkt = NULL;
				ctxt->state = RR_STATE_IDLE;
			}
			continue;

		case RR_STATE_ERROR:
			/* TODO: restart if we can */
			return;

		default:
			printk(KERN_ERR "inavlid rr ctxt state\n");
			BUG();
		}
	}
}

void msm_rpc_setup_req(struct rpc_request_hdr *hdr, uint32_t prog,
		       uint32_t vers, uint32_t proc)
{
	memset(hdr, 0, sizeof(struct rpc_request_hdr));
	hdr->xid = cpu_to_be32(++next_xid);
	hdr->rpc_vers = cpu_to_be32(2);
	hdr->prog = cpu_to_be32(prog);
	hdr->vers = cpu_to_be32(vers);
	hdr->procedure = cpu_to_be32(proc);
}

int msm_rpc_open(struct msm_rpc_endpoint **ept)
{
	if (!ept)
		return -EINVAL;

	*ept = msm_rpcrouter_create_local_endpoint(MKDEV(0, 0));
	if (!(*ept))
		return -ENOMEM;

	return 0;
}

int msm_rpc_close(struct msm_rpc_endpoint *ept)
{
	return msm_rpcrouter_destroy_local_endpoint(ept);
}

int msm_rpc_write(struct msm_rpc_endpoint *ept, void *buffer, int count)
{
	struct rr_header hdr;
	uint32_t pacmark;
	struct rpc_request_hdr *rq = buffer;
	struct rr_remote_endpoint *r_ept;
	unsigned long flags;
	int needed;
	DEFINE_WAIT(__wait);

	if (count > RPCROUTER_MSGSIZE_MAX || !count)
		return -EINVAL;

	/* snoop the RPC packet and enforce permissions */

	/* has to have at least the xid and type fields */
	if (count < (sizeof(uint32_t) * 2)) {
		printk(KERN_ERR "rr_write: rejecting runt packet\n");
		return -EINVAL;
	}

	if (rq->type == 0) {
		/* RPC CALL */
		if (count < (sizeof(uint32_t) * 6)) {
			printk(KERN_ERR
			       "rr_write: rejecting runt call packet\n");
			return -EINVAL;
		}
		if (ept->dst_pid == 0xffffffff) {
			printk(KERN_ERR "rr_write: not connected\n");
			return -ENOTCONN;
		}
		if ((ept->dst_prog != rq->prog) ||
		    (ept->dst_vers != rq->vers)) {
			printk(KERN_ERR
			       "rr_write: cannot write to %08x:%d "
			       "(bound to %08x:%d)\n",
			       be32_to_cpu(rq->prog), be32_to_cpu(rq->vers),
			       be32_to_cpu(ept->dst_prog),
			       be32_to_cpu(ept->dst_vers));
			return -EINVAL;
		}
		hdr.dst_pid = ept->dst_pid;
		hdr.dst_cid = ept->dst_cid;
		IO("CALL to %08x:%d @ %d:%08x (%d bytes)\n",
		   be32_to_cpu(rq->prog), be32_to_cpu(rq->vers),
		   ept->dst_pid, ept->dst_cid, count);
	} else {
		/* RPC REPLY */
		/* TODO: locking */
		if (ept->reply_pid == 0xffffffff) {
			printk(KERN_ERR
			       "rr_write: rejecting unexpected reply\n");
			return -EINVAL;
		}
		if (ept->reply_xid != rq->xid) {
			printk(KERN_ERR
			       "rr_write: rejecting packet w/ bad xid\n");
			return -EINVAL;
		}

		hdr.dst_pid = ept->reply_pid;
		hdr.dst_cid = ept->reply_cid;

		/* consume this reply */
		ept->reply_pid = 0xffffffff;

		IO("REPLY to xid=%d @ %d:%08x (%d bytes)\n",
		   be32_to_cpu(rq->xid), hdr.dst_pid, hdr.dst_cid, count);
	}

	r_ept = rpcrouter_lookup_remote_endpoint(hdr.dst_cid);

	if (!r_ept) {
		printk(KERN_ERR
			"msm_rpc_write(): No route to ept "
			"[PID %x CID %x]\n", hdr.dst_pid, hdr.dst_cid);
		return -EHOSTUNREACH;
	}

	/* Create routing header */
	hdr.type = RPCROUTER_CTRL_CMD_DATA;
	hdr.version = RPCROUTER_VERSION;
	hdr.src_pid = ept->pid;
	hdr.src_cid = ept->cid;
	hdr.confirm_rx = 0;
	hdr.size = count + sizeof(uint32_t);

	/* Create pacmark header */
	pacmark = PACMARK(count, ++next_pacmarkid, 1);

	for (;;) {
		prepare_to_wait(&r_ept->quota_wait, &__wait,
				TASK_INTERRUPTIBLE);
		spin_lock_irqsave(&r_ept->quota_lock, flags);
		if (r_ept->tx_quota_cntr < RPCROUTER_DEFAULT_RX_QUOTA)
			break;
		spin_unlock_irqrestore(&r_ept->quota_lock, flags);
		if (!signal_pending(current)) {
			schedule();
			continue;
		}
		break;
	}
	finish_wait(&r_ept->quota_wait, &__wait);

	if (signal_pending(current)) {
		spin_unlock_irqrestore(&r_ept->quota_lock, flags);
		return -ERESTARTSYS;
	}

	r_ept->tx_quota_cntr++;
	if (r_ept->tx_quota_cntr == RPCROUTER_DEFAULT_RX_QUOTA)
		hdr.confirm_rx = 1;

	spin_unlock_irqrestore(&r_ept->quota_lock, flags);

	spin_lock_irqsave(&smd_lock, flags);

	needed = sizeof(hdr) + hdr.size;
	while (smd_write_avail(smd_channel) < needed) {
		spin_unlock_irqrestore(&smd_lock, flags);
		msleep(250);
		spin_lock_irqsave(&smd_lock, flags);
	}

	/* TODO: deal with full fifo */
	smd_write(smd_channel, &hdr, sizeof(hdr));
	smd_write(smd_channel, &pacmark, sizeof(pacmark));
	smd_write(smd_channel, buffer, count);

	spin_unlock_irqrestore(&smd_lock, flags);

	return count;
}

static inline int ept_packet_available(struct msm_rpc_endpoint *ept)
{
	unsigned long flags;
	int ret;
	spin_lock_irqsave(&ept->read_q_lock, flags);
	ret = !list_empty(&ept->read_q);
	spin_unlock_irqrestore(&ept->read_q_lock, flags);
	return ret;
}

/*
 * NOTE: It is the responsibility of the caller to kfree buffer
 */
int msm_rpc_read(struct msm_rpc_endpoint *ept, void **buffer,
		 unsigned user_len, long timeout)
{
	struct rr_packet *pkt;
	struct rpc_request_hdr *rq;
	DEFINE_WAIT(__wait);
	unsigned long flags;
	int rc;

	IO("READ on ept %p\n", ept);
	if (timeout < 0) {
		rc = wait_event_interruptible(
			ept->wait_q, ept_packet_available(ept));
	} else {
		rc = wait_event_interruptible_timeout(
			ept->wait_q, ept_packet_available(ept),
			timeout);
		if ((rc == 0) && !ept_packet_available(ept))
			return -ETIMEDOUT;
	}

	if (rc < 0)
		return rc;

	spin_lock_irqsave(&ept->read_q_lock, flags);
	if (list_empty(&ept->read_q)) {
		spin_unlock_irqrestore(&ept->read_q_lock, flags);
		return -EAGAIN;
	}
	pkt = list_first_entry(&ept->read_q, struct rr_packet, list);
	if (pkt->data_len > user_len) {
		spin_unlock_irqrestore(&ept->read_q_lock, flags);
		return -ETOOSMALL;
	}
	list_del(&pkt->list);
	spin_unlock_irqrestore(&ept->read_q_lock, flags);

	/* XXX THIS SUUUUUCKS - FIX DURING READ */
	rc = pkt->data_len - sizeof(uint32_t);
	memmove(pkt->data, ((char *) pkt->data) + sizeof(uint32_t), rc);
	*buffer = pkt->data;

	rq = pkt->data;
	if ((rc >= (sizeof(uint32_t) * 3)) && (rq->type == 0)) {
		/* RPC CALL */
		if (ept->reply_pid != 0xffffffff) {
			printk(KERN_WARNING
			       "rr_read: lost previous reply xid...\n");
		}
		/* TODO: locking? */
		ept->reply_pid = pkt->hdr.src_pid;
		ept->reply_cid = pkt->hdr.src_cid;
		ept->reply_xid = rq->xid;
	}

	if (pkt->hdr.confirm_rx) {
		union rr_control_msg msg;

		msg.cmd = RPCROUTER_CTRL_CMD_RESUME_TX;
		msg.cli.pid = ept->pid;
		msg.cli.cid = ept->cid;

		RR("x RESUME_TX id=%d:%08x\n", msg.cli.pid, msg.cli.cid);
		rpcrouter_send_control_msg(&msg);
	}

	kfree(pkt);
	IO("READ on ept %p (%d bytes)\n", ept, rc);
	return rc;
}

int msm_rpc_connect(struct msm_rpc_endpoint *ept,
		    uint32_t prog, uint32_t vers,
		    long timeout)
{
	struct rr_server *server;

	/* can only bind once */
	if (ept->dst_pid != 0xffffffff)
		return -EISCONN;

	server = rpcrouter_lookup_server(prog, vers);
	if (!server)
		return -EHOSTUNREACH;

	ept->dst_pid = server->pid;
	ept->dst_cid = server->cid;
	ept->dst_prog = cpu_to_be32(prog);
	ept->dst_vers = cpu_to_be32(vers);

	return 0;
}

/* TODO: permission check? */
int msm_rpc_register_server(struct msm_rpc_endpoint *ept,
			    uint32_t prog, uint32_t vers)
{
	int rc;
	union rr_control_msg msg;
	struct rr_server *server;

	server = rpcrouter_create_server(ept->pid, ept->cid,
					 prog, vers);
	if (!server)
		return -ENODEV;

	msg.srv.cmd = RPCROUTER_CTRL_CMD_NEW_SERVER;
	msg.srv.pid = ept->pid;
	msg.srv.cid = ept->cid;
	msg.srv.prog = prog;
	msg.srv.vers = vers;

	RR("x NEW_SERVER id=%d:%08x prog=%08x:%d\n",
	   ept->pid, ept->cid, prog, vers);

	rc = rpcrouter_send_control_msg(&msg);
	if (rc < 0)
		return rc;

	return 0;
}

/* TODO: permission check -- disallow unreg of somebody else's server */
int msm_rpc_unregister_server(struct msm_rpc_endpoint *ept,
			      uint32_t prog, uint32_t vers)
{
	struct rr_server *server;
	server = rpcrouter_lookup_server(prog, vers);

	if (!server)
		return -ENOENT;
	rpcrouter_destroy_server(server);
	return 0;
}

static int msm_rpcrouter_probe(struct platform_device *pdev)
{
	int rc;

	/* Initialize what we need to start processing */
	INIT_LIST_HEAD(&local_endpoints);
	INIT_LIST_HEAD(&remote_endpoints);

	init_waitqueue_head(&newserver_wait);

	rpcrouter_workqueue = create_singlethread_workqueue("rpcrouter");
	if (!rpcrouter_workqueue)
		return -ENOMEM;

	rc = msm_rpcrouter_init_devices();
	if (rc < 0)
		goto fail_destroy_workqueue;

	/* Open up SMD channel 2 */
	initialized = 0;
	rc = smd_open(2, &smd_channel, NULL, rpcrouter_smdnotify);
	if (rc < 0)
		goto fail_remove_devices;

	return 0;

fail_remove_devices:
	msm_rpcrouter_exit_devices();
fail_destroy_workqueue:
	destroy_workqueue(rpcrouter_workqueue);
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
