/* arch/arm/mach-msm/smd_qmi.c
 *
 * QMI Control Driver -- Manages network data connections.
 *
 * Copyright (C) 2007 Google, Inc.
 * Author: Brian Swetland <swetland@google.com>
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
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/wait.h>
#include <linux/miscdevice.h>
#include <linux/workqueue.h>

#include <asm/uaccess.h>
#include <asm/arch/msm_smd.h>

#define QMI_CTL 0x00
#define QMI_WDS 0x01
#define QMI_DMS 0x02
#define QMI_NAS 0x03

#define QMI_RESULT_SUCCESS 0x0000
#define QMI_RESULT_FAILURE 0x0001

struct qmi_msg
{
	unsigned char service;
	unsigned char client_id;
	unsigned short txn_id;
	unsigned short type;
	unsigned short size;
	unsigned char *tlv;
};

static DEFINE_MUTEX(qmi_lock);
static smd_channel_t *qmi_ch;

static struct workqueue_struct *qmi_wq;

#define qmi_ctl_client_id 0
static unsigned char qmi_ctl_txn_id = 1;
static unsigned char qmi_wds_client_id;
static unsigned short qmi_wds_txn_id = 1;

#define STATE_OFFLINE    0
#define STATE_QUERYING   1
#define STATE_ONLINE     2

static unsigned qmi_wds_busy = 1;
static unsigned qmi_wds_handle;
static unsigned qmi_state_dirty;
static unsigned qmi_state = STATE_OFFLINE;

static unsigned char qmi_addr[4];
static unsigned char qmi_mask[4];
static unsigned char qmi_gateway[4];
static unsigned char qmi_dns1[4];
static unsigned char qmi_dns2[4];

static int verbose;

/* anyone waiting for a state change waits here */
static DECLARE_WAIT_QUEUE_HEAD(qmi_wait_queue);


static void qmi_dump_msg(struct qmi_msg *msg, const char *prefix)
{
	unsigned sz, n;
	unsigned char *x;

	if (!verbose) return;

	printk(KERN_INFO
	       "qmi: %s: svc=%02x cid=%02x tid=%04x type=%04x size=%04x\n",
	       prefix, msg->service, msg->client_id,
	       msg->txn_id, msg->type, msg->size);

	x = msg->tlv;
	sz = msg->size;

	while (sz >= 3) {
		sz -= 3;

		n = x[1] | (x[2] << 8);
		if (n > sz) break;

		printk(KERN_INFO "qmi: %s: tlv: %02x %04x { ",
		       prefix, x[0], n);
		x += 3;
		sz -= n;
		while (n-- > 0)
			printk("%02x ", *x++);
		printk("}\n");
	}
}

int qmi_add_tlv(struct qmi_msg *msg,
		unsigned type, unsigned size, const void *data)
{
	unsigned char *x = msg->tlv + msg->size;

	x[0] = type;
	x[1] = size;
	x[2] = size >> 8;

	memcpy(x + 3, data, size);

	msg->size += (size + 3);

	return 0;
}

/* Extract a tagged item from a qmi message buffer,
** taking care not to overrun the buffer.
*/
static int qmi_get_tlv(struct qmi_msg *msg,
		       unsigned type, unsigned size, void *data)
{
	unsigned char *x = msg->tlv;
	unsigned len = msg->size;
	unsigned n;

	while (len >= 3) {
		len -= 3;

		/* size of this item */
		n = x[1] | (x[2] << 8);
		if (n > len) break;

		if (x[0] == type) {
			if (n != size) return -1;
			memcpy(data, x + 3, size);
			return 0;
		}

		x += (n + 3);
		len -= n;
	}

	return -1;
}

static unsigned qmi_get_status(struct qmi_msg *msg, unsigned *error)
{
	unsigned short status[2];
	if (qmi_get_tlv(msg, 0x02, sizeof(status), status)) {
		*error = 0;
		return QMI_RESULT_FAILURE;
	} else {
		*error = status[1];
		return status[0];
	}
}

/* 0x01 <qmux-header> <payload> 0x0b 0x00 0x00 0x00 */

#define QMUX_HEADER    13
#define QMUX_FOOTER    4

/* should be >= HEADER + FOOTER */
#define QMUX_OVERHEAD  20

static int qmi_send(struct qmi_msg *msg)
{
	unsigned char *data;
	unsigned hlen;
	unsigned len;
	int r;

	qmi_dump_msg(msg, "send");

	if (msg->service == QMI_CTL) {
		hlen = QMUX_HEADER - 1;
	} else {
		hlen = QMUX_HEADER;
	}

	/* QMUX length is total header + total payload - IFC selector */
	len = hlen + msg->size - 1;
	if (len > 0xffff) return -1;

	data = msg->tlv + msg->size;

	/* append encap footer */
	data[0] = 0x0b;
	data[1] = 0x00;
	data[2] = 0x00;
	data[3] = 0x00;

	data = msg->tlv - hlen;

	/* prepend encap and qmux header */
	*data++ = 0x01; /* ifc selector */

	/* qmux header */
	*data++ = len;
	*data++ = len >> 8;
	*data++ = 0x00; /* flags: client */
	*data++ = msg->service;
	*data++ = msg->client_id;

	/* qmi header */
	*data++ = 0x00; /* flags: send */
	*data++ = msg->txn_id;
	if (msg->service != QMI_CTL)
		*data++ = msg->txn_id >> 8;

	*data++ = msg->type;
	*data++ = msg->type >> 8;
	*data++ = msg->size;
	*data++ = msg->size >> 8;

	/* add in the ifc selector and smd footer */
	len += 5;

	r = smd_write(qmi_ch, msg->tlv - hlen, len);

	if (r != len) {
		return -1;
	} else {
		return 0;
	}
}

static void qmi_process_ctl_msg(struct qmi_msg *msg)
{
	unsigned err;
	if (msg->type == 0x0022) {
		unsigned char n[2];
		if (qmi_get_status(msg, &err))
			return;
		if (qmi_get_tlv(msg, 0x01, sizeof(n), n))
			return;
		if (n[0] == QMI_WDS) {
			printk(KERN_INFO
			       "qmi: ctl: wds use client_id 0x%02x\n", n[1]);
			qmi_wds_client_id = n[1];
			qmi_wds_busy = 0;
		}
	}
}

static int qmi_network_get_profile(void);

static void swapaddr(unsigned char *src, unsigned char *dst)
{
	dst[0] = src[3];
	dst[1] = src[2];
	dst[2] = src[1];
	dst[3] = src[0];
}

static unsigned char zero[4];
static void qmi_read_runtime_profile(struct qmi_msg *msg)
{
	unsigned char tmp[4];
	unsigned r;

	r = qmi_get_tlv(msg, 0x1e, 4, tmp);
	swapaddr(r ? zero : tmp, qmi_addr);
	r = qmi_get_tlv(msg, 0x21, 4, tmp);
	swapaddr(r ? zero : tmp, qmi_mask);
	r = qmi_get_tlv(msg, 0x20, 4, tmp);
	swapaddr(r ? zero : tmp, qmi_gateway);
	r = qmi_get_tlv(msg, 0x15, 4, tmp);
	swapaddr(r ? zero : tmp, qmi_dns1);
	r = qmi_get_tlv(msg, 0x16, 4, tmp);
	swapaddr(r ? zero : tmp, qmi_dns2);
}

static void qmi_process_unicast_wds_msg(struct qmi_msg *msg)
{
	unsigned err;
	switch (msg->type) {
	case 0x0020:
		if (qmi_get_status(msg, &err)) {
			printk(KERN_ERR "qmi: wds: network start failed (%04x)\n", err);
		} else if (qmi_get_tlv(msg, 0x01, sizeof(qmi_wds_handle), &qmi_wds_handle)) {
			printk(KERN_ERR "qmi: wds no handle?\n");
		} else {
			printk(KERN_INFO "qmi: wds: got handle 0x%08x\n", qmi_wds_handle);
		}
		break;
	case 0x002D:
		printk(KERN_INFO "qmi: got network profile\n");
		if (qmi_state == STATE_QUERYING) {
			qmi_read_runtime_profile(msg);
			qmi_state = STATE_ONLINE;
			qmi_state_dirty = 1;
		}
		break;
	default:
		printk(KERN_ERR
		       "qmi_process_unicast_msg unknown type %04x\n",
		       msg->type);
	}
	qmi_wds_busy = 0;
}

static void qmi_process_broadcast_wds_msg(struct qmi_msg *msg)
{
	if (msg->type == 0x0022) {
		unsigned char n[2];
		if (qmi_get_tlv(msg, 0x01, sizeof(n), n))
			return;
		switch (n[0]) {
		case 1:
			printk(KERN_INFO "qmi: wds: DISCONNECTED\n");
			qmi_state = STATE_OFFLINE;
			break;
		case 2:
			printk(KERN_INFO "qmi: wds: CONNECTED\n");
			qmi_state = STATE_QUERYING;
			qmi_network_get_profile();
			break;
		case 3:
			printk(KERN_INFO "qmi: wds: SUSPENDED\n");
			qmi_state = STATE_OFFLINE;
			break;
		}
	} else {
		printk(KERN_ERR
		       "qmi_process_broadcast_msg unknown type %04x\n",
		       msg->type);
	}
}

static void qmi_process_wds_msg(struct qmi_msg *msg)
{
	if (msg->client_id == qmi_wds_client_id) {
		qmi_process_unicast_wds_msg(msg);
	} else if (msg->client_id == 0xff) {
		qmi_process_broadcast_wds_msg(msg);
	} else {
		printk(KERN_ERR
		       "qmi_process_wds_msg client id 0x%02x unknown\n",
			msg->client_id);
	}
}

static void qmi_process_qmux(unsigned char *buf, unsigned sz)
{
	struct qmi_msg msg;

	/* require a full header */
	if (sz < 5) return;

	/* require a size that matches the buffer size */
	if (sz != (buf[0] | (buf[1] << 8))) return;

	/* only messages from a service (bit7=1) are allowed */
	if (buf[2] != 0x80) return;

	msg.service = buf[3];
	msg.client_id = buf[4];

	/* annoyingly, CTL messages have a shorter TID */
	if (buf[3] == 0) {
		if (sz < 7) return;
		msg.txn_id = buf[6];
		buf += 7;
		sz -= 7;
	} else {
		if (sz < 8) return;
		msg.txn_id = buf[6] | (buf[7] << 8);
		buf += 8;
		sz -= 8;
	}

	/* no type and size!? */
	if (sz < 4) return;
	sz -= 4;

	msg.type = buf[0] | (buf[1] << 8);
	msg.size = buf[2] | (buf[3] << 8);
	msg.tlv = buf + 4;

	if (sz != msg.size) return;

	qmi_dump_msg(&msg, "recv");

	mutex_lock(&qmi_lock);
	switch (msg.service) {
	case QMI_CTL:
		qmi_process_ctl_msg(&msg);
		break;
	case QMI_WDS:
		qmi_process_wds_msg(&msg);
		break;
	default:
		printk("qmi: msg from unknown svc 0x%02x\n", msg.service);
		break;
	}
	mutex_unlock(&qmi_lock);

	wake_up(&qmi_wait_queue);
}

#define QMI_MAX_PACKET (256 + QMUX_OVERHEAD)

static void qmi_work_func(struct work_struct *ws)
{
	unsigned char buf[QMI_MAX_PACKET];
	int sz;

	for (;;) {
		sz = smd_cur_packet_size(qmi_ch);
		if (sz == 0)
			break;
		if (sz < smd_read_avail(qmi_ch))
			break;
		if (sz > QMI_MAX_PACKET) {
			smd_read(qmi_ch, 0, sz);
			continue;
		}
		if (smd_read(qmi_ch, buf, sz) != sz) {
			printk(KERN_ERR "qmi: not enough data?!\n");
			continue;
		}

		/* require the 0x01 header and 0x0b,0x00,0x00,0x00 footer */
		if (sz < 5) continue;
		if (buf[0] != 0x01) continue;
		if (buf[sz - 4] != 0x0b) continue;
		if (buf[sz - 3] != 0x00) continue;
		if (buf[sz - 2] != 0x00) continue;
		if (buf[sz - 1] != 0x00) continue;

		qmi_process_qmux(buf + 1, sz - 5);
	}
}

static int qmi_request_wds_cid(void);

void qmi_init_work_func(struct work_struct *work)
{
	mutex_lock(&qmi_lock);
	qmi_request_wds_cid();
	mutex_unlock(&qmi_lock);
}

static DECLARE_WORK(qmi_work, qmi_work_func);
static DECLARE_WORK(qmi_opened_work, qmi_init_work_func);

static void qmi_notify(void *priv, unsigned event)
{
	switch (event) {
	case SMD_EVENT_DATA: {
		int sz;
		sz = smd_cur_packet_size(qmi_ch);
		if ((sz > 0) && (sz <= smd_read_avail(qmi_ch)))
			queue_work(qmi_wq, &qmi_work);
		break;
	}
	case SMD_EVENT_OPEN:
		printk(KERN_INFO "qmi: smd opened\n");
		queue_work(qmi_wq, &qmi_opened_work);
		break;
	case SMD_EVENT_CLOSE:
		printk(KERN_INFO "qmi: smd closed\n");
		break;
	}
}

static int qmi_request_wds_cid(void)
{
	unsigned char data[64 + QMUX_OVERHEAD];
	struct qmi_msg msg;
	unsigned char n;

	msg.service = QMI_CTL;
	msg.client_id = qmi_ctl_client_id;
	msg.txn_id = qmi_ctl_txn_id;
	msg.type = 0x0022;
	msg.size = 0;
	msg.tlv = data + QMUX_HEADER;

	qmi_ctl_txn_id += 2;

	n = QMI_WDS;
	qmi_add_tlv(&msg, 0x01, 0x01, &n);

	return qmi_send(&msg);
}

static int qmi_network_get_profile(void)
{
	unsigned char data[96 + QMUX_OVERHEAD];
	struct qmi_msg msg;

	msg.service = QMI_WDS;
	msg.client_id = qmi_wds_client_id;
	msg.txn_id = qmi_wds_txn_id;
	msg.type = 0x002D;
	msg.size = 0;
	msg.tlv = data + QMUX_HEADER;

	qmi_wds_txn_id += 2;

	return qmi_send(&msg);
}

static int qmi_network_up(const char *apn)
{
	unsigned char data[96 + QMUX_OVERHEAD];
	struct qmi_msg msg;

	msg.service = QMI_WDS;
	msg.client_id = qmi_wds_client_id;
	msg.txn_id = qmi_wds_txn_id;
	msg.type = 0x0020;
	msg.size = 0;
	msg.tlv = data + QMUX_HEADER;

	qmi_wds_txn_id += 2;

	qmi_add_tlv(&msg, 0x14, strlen(apn), apn);

	return qmi_send(&msg);
}

static int qmi_network_down(void)
{
	unsigned char data[16 + QMUX_OVERHEAD];
	struct qmi_msg msg;

	msg.service = QMI_WDS;
	msg.client_id = qmi_wds_client_id;
	msg.txn_id = qmi_wds_txn_id;
	msg.type = 0x0021;
	msg.size = 0;
	msg.tlv = data + QMUX_HEADER;

	qmi_wds_txn_id += 2;

	qmi_add_tlv(&msg, 0x01, sizeof(qmi_wds_handle), &qmi_wds_handle);

	return qmi_send(&msg);
}

static int qmi_print_state(char *buf, int max)
{
	int i;

	if (qmi_state != STATE_ONLINE)
		return scnprintf(buf, max, "STATE=down\n");

	i = scnprintf(buf, max, "STATE=up\n");

	i += scnprintf(buf + i, max - i, "ADDR=%d.%d.%d.%d\n",
		qmi_addr[0], qmi_addr[1], qmi_addr[2], qmi_addr[3]);
	i += scnprintf(buf + i, max - i, "MASK=%d.%d.%d.%d\n",
		qmi_mask[0], qmi_mask[1], qmi_mask[2], qmi_mask[3]);
	i += scnprintf(buf + i, max - i, "GATEWAY=%d.%d.%d.%d\n",
		qmi_gateway[0], qmi_gateway[1], qmi_gateway[2],
		qmi_gateway[3]);
	i += scnprintf(buf + i, max - i, "DNS1=%d.%d.%d.%d\n",
		qmi_dns1[0], qmi_dns1[1], qmi_dns1[2], qmi_dns1[3]);
	i += scnprintf(buf + i, max - i, "DNS2=%d.%d.%d.%d\n",
		qmi_dns2[0], qmi_dns2[1], qmi_dns2[2], qmi_dns2[3]);

	return i;
}

static ssize_t qmi_read(struct file *fp, char __user *buf,
			size_t count, loff_t *pos)
{
	char msg[256];
	int len;
	int r;

	mutex_lock(&qmi_lock);
	for (;;) {
		if (qmi_state_dirty) {
			qmi_state_dirty = 0;
			len = qmi_print_state(msg, 256);
			break;
		}
		mutex_unlock(&qmi_lock);
		r = wait_event_interruptible(qmi_wait_queue, qmi_state_dirty);
		if (r < 0)
			return r;
		mutex_lock(&qmi_lock);
	}
	mutex_unlock(&qmi_lock);

	if (len > count)
		len = count;

	if (copy_to_user(buf, msg, len))
		return -EFAULT;

	return len;
}


static ssize_t qmi_write(struct file *fp, const char __user *buf,
			 size_t count, loff_t *pos)
{
	unsigned char cmd[64];
	int len;
	int r;

	if (count < 1) return 0;

	len = count > 63 ? 63 : count;

	if (copy_from_user(cmd, buf, len))
		return -EFAULT;

	cmd[len] = 0;

	/* lazy */
	if (cmd[len-1] == '\n') {
		cmd[len-1] = 0;
		len--;
	}

	if (!strncmp(cmd, "verbose", 7)) {
		verbose = 1;
	} else if (!strncmp(cmd, "terse", 5)) {
		verbose = 0;
	} else if (!strncmp(cmd, "poll", 4)) {
		qmi_state_dirty = 1;
		wake_up(&qmi_wait_queue);
	} else if (!strncmp(cmd, "down", 4)) {
retry_down:
		mutex_lock(&qmi_lock);
		if (qmi_wds_busy) {
			mutex_unlock(&qmi_lock);
			r = wait_event_interruptible(qmi_wait_queue, !qmi_wds_busy);
			if (r < 0)
				return r;
			goto retry_down;
		}
		qmi_wds_busy = 1;
		qmi_network_down();
		mutex_unlock(&qmi_lock);
	} else if (!strncmp(cmd, "up:", 3)) {
retry_up:
		mutex_lock(&qmi_lock);
		if (qmi_wds_busy) {
			mutex_unlock(&qmi_lock);
			r = wait_event_interruptible(qmi_wait_queue, !qmi_wds_busy);
			if (r < 0)
				return r;
			goto retry_up;
		}
		qmi_wds_busy = 1;
		qmi_network_up(cmd+3);
		mutex_unlock(&qmi_lock);
	} else {
		return -EINVAL;
	}

	return count;
}

static int qmi_open(struct inode *ip, struct file *fp)
{
	int r = 0;

	mutex_lock(&qmi_lock);
	if (qmi_ch == 0)
		r = smd_open(5, &qmi_ch, 0, qmi_notify);
	qmi_state_dirty = 1;
	mutex_unlock(&qmi_lock);

	wake_up(&qmi_wait_queue);

	return r;
}

static int qmi_release(struct inode *ip, struct file *fp)
{
	return 0;
}


static struct file_operations qmi_fops = {
	.owner = THIS_MODULE,
	.read = qmi_read,
	.write = qmi_write,
	.open = qmi_open,
	.release = qmi_release,
};

static struct miscdevice qmi_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "qmi",
	.fops = &qmi_fops,
};

static int __init qmi_init(void)
{
	printk(KERN_INFO "qmi_init()\n");

	qmi_wq = create_singlethread_workqueue("qmi");
	if (qmi_wq == 0)
		return -ENOMEM;

	return misc_register(&qmi_device);
}

module_init(qmi_init);
