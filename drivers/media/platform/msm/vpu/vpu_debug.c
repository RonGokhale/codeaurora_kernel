/* Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/types.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/sizes.h>

#include "vpu_debug.h"
#include "vpu_v4l2.h"
#include "vpu_ioctl_internal.h"
#include "vpu_channel.h"
#include "vpu_hfi.h"
#include "vpu_ipc.h"

#define BUF_SIZE	(SZ_4K)
#define RW_MODE		(S_IRUSR | S_IWUSR)

u8 vpu_debug_level = VPU_ERR | VPU_WARN;
int vpu_debug_out_type = VPU_OUT_PRINTK;

struct maple_log_info {
	/* wq woken by hfi layer when maple log msg received */
	wait_queue_head_t wq;
	/* wq only woken by hfi layer if this flag set */
	int wake_up_request;
	/* buf used for formatting log msgs */
	char *fmt_buf;
};

/* SMEM controller data */
struct smem_ctrl_data {
	/* number of bytes to read */
	u32 size;
	/* offset from shared memory base address */
	u32 offset;
};

static struct smem_ctrl_data smem_ctrl = {
	.size = 1024,
	.offset = 0x00000000
};


struct maple_log_info maple_log;

void vpu_wakeup_maple_logging_wq(void)
{
	if (maple_log.wake_up_request) {
		maple_log.wake_up_request = 0;
		wake_up_interruptible_all(&maple_log.wq);
	}
}
static int open_maple_log(struct inode *inode, struct file *file)
{
	char *fmt_buf;

	maple_log.wake_up_request = 0;
	init_waitqueue_head(&maple_log.wq);

	fmt_buf = kzalloc(BUF_SIZE, GFP_KERNEL);
	if (unlikely(!fmt_buf)) {
		dprintk(VPU_ERR, "Failed to allocated fmt_buf\n");
		return -ENOMEM;
	}

	maple_log.fmt_buf = fmt_buf;
	return 0;
}


/*
 * To be called when the logging queue is active (after vpu_hw_sys_start call)
 * Read the content of the VPU logging queue into buf (up to buf_size bytes)
 *
 * @buf:	logging buffer to write into
 * @buf_size:	maximum size to read, in bytes
 *
 * @return	the number of bytes read
 */
static size_t print_log(char __user *user_buf, char *fmt_buf, int buf_size)
{
	int read_data = 0;
	size_t size = 0;
	int bytes_unread;
	struct vpu_ipc_log_header_packet *log_hdr;
	int msg_size;
	int offset = 0;

	/* get the full (unread) logging buffer */
	read_data = vpu_hfi_read_log_data(VPU_LOGGING_CHANNEL_ID, fmt_buf,
			buf_size);

	if (read_data <= 0) {
		dprintk(VPU_DBG, "%s: no data read from logging queue (%d).\n",
			__func__, read_data);
		return read_data;
	}

	dprintk(VPU_DBG, "%s: size of full logging buffer %d\n",
			__func__, read_data);

	log_hdr = (struct vpu_ipc_log_header_packet *)fmt_buf;
	/* read the local buffer and interpret the data inside */
	do {
		char *log_msg;

		/* point to the next log header packet*/
		if (!log_hdr)
			/* we reach the end of the logging */
			break;

		/* locate the logging message and its size */
		log_msg = (char *)(log_hdr->msg_addr ? log_hdr->msg_addr :
			(u32)(log_hdr + 1));
		msg_size = log_hdr->msg_size;

		/*
		 * For debug purpose only:
		 *
		 * dprintk(VPU_DBG, "%s: log_msg=%p | msg_size=%d\n",
		 *		__func__, log_msg, msg_size);
		 * dprintk(VPU_DBG, "%s: log_msg='%s'\n", __func__, log_msg);
		 */

		/* if too big, shorten log_msg to fit user buffer */
		if ((offset + msg_size) > buf_size) {
			msg_size = buf_size - offset;
			dprintk(VPU_WARN, "%s: Loosing data", __func__);
		}

		/* finally concatenate this log msg to the user buf passed in */
		bytes_unread = copy_to_user((char *)(user_buf + offset),
				log_msg, msg_size);
		if (bytes_unread > 0) {
			dprintk(VPU_ERR,
				"copying to user failed, bytes unread %d\n",
				bytes_unread);
			return size;
		}
		offset += msg_size;
		size += msg_size;

		/* read the next message */
		log_hdr = (struct vpu_ipc_log_header_packet *)((u32)log_hdr +
				log_hdr->size);

	} while ((u32)log_hdr < ((u32)fmt_buf + read_data - sizeof(*log_hdr))
			&& size < buf_size);

	VPU_EXIT_FUNC("return %d", size);
	return size;
}

static ssize_t read_maple_log(struct file *file, char __user *user_buf,
	size_t len, loff_t *ppos)
{
	int ret;
	size_t bytes_read = 0;

	do {
		/* read data into user buffer */
		bytes_read = print_log(user_buf, maple_log.fmt_buf, len);

		/*
		 * The Logging queue might not be ready yet. If that is the case
		 * try again after queue init.
		 * Also, when we read 0 bytes we wait until Maple writes
		 * something in the logging queue.
		 */
		if ((bytes_read == -EAGAIN) || (bytes_read == 0)) {
			maple_log.wake_up_request = 1;
			ret = wait_event_interruptible(maple_log.wq,
					maple_log.wake_up_request == 0);
			if (ret < 0) {
				/* could be a signal, return */
				dprintk(VPU_ERR, "%s, Error: ret=%d\n",
						__func__, ret);
				return ret;
			}
		} else if (bytes_read < 0) {
			dprintk(VPU_ERR, "%s, Error: bytes_read=%d\n",
					__func__, bytes_read);
			return bytes_read;
		}

	} while ((bytes_read == -EAGAIN) || (bytes_read == 0));

	VPU_EXIT_FUNC("return %d", bytes_read);
	return bytes_read;
}

static int release_maple_log(struct inode *inode, struct file *file)
{
	kfree(maple_log.fmt_buf);
	maple_log.fmt_buf = NULL;

	return 0;
}

static const struct file_operations maple_logging_ops = {
	.open = open_maple_log,
	.read = read_maple_log,
	.release = release_maple_log,
};

static ssize_t read_queue_state(struct file *file, char __user *user_buf,
	size_t len, loff_t *ppos)
{
	char *dbg_buf;
	size_t size, ret;

	dbg_buf = kzalloc(BUF_SIZE, GFP_KERNEL);
	if (!dbg_buf) {
		dprintk(VPU_ERR, "%s failed to alloc\n", __func__);
		return 0;
	}

	size = vpu_hfi_print_queues(dbg_buf, BUF_SIZE);
	ret = simple_read_from_buffer(user_buf, len, ppos, dbg_buf, size);

	kfree(dbg_buf);
	return ret;
}

static const struct file_operations queue_state_ops = {
	.open = simple_open,
	.read = read_queue_state,
};

static ssize_t read_csr_regs(struct file *file, char __user *user_buf,
	size_t len, loff_t *ppos)
{
	char *dbg_buf;
	size_t size, ret;

	dbg_buf = kzalloc(BUF_SIZE, GFP_KERNEL);
	if (!dbg_buf) {
		dprintk(VPU_ERR, "%s failed to alloc\n", __func__);
		return 0;
	}

	size = vpu_hfi_dump_csr_regs(dbg_buf, BUF_SIZE);

	ret = simple_read_from_buffer(user_buf, len, ppos, dbg_buf, size);

	kfree(dbg_buf);
	return ret;
}

static const struct file_operations csr_regs_ops = {
	.open = simple_open,
	.read = read_csr_regs,
};

static ssize_t write_cmd(struct file *file, const char __user *user_buf,
	size_t len, loff_t *ppos)
{
	int ret = len;
	char str[] =
		"Usage: echo <cmd> > <this_file> to send a command to VPU\n";

	if (strcmp(user_buf, "crash\n") == 0) {
		vpu_hw_sys_cmd_ext(VPU_SYS_CMD_DEBUG_CRASH, NULL, 0);
	} else {
		dprintk(VPU_WARN, "%s, Error: %s is an unknown cmd\n",
				__func__, user_buf);
		dprintk(VPU_INFO, "%s\n", str);
		ret = -EINVAL;
	}

	return ret;
}

static ssize_t read_cmd(struct file *file, char __user *user_buf,
	size_t len, loff_t *ppos)
{
	char str[] =
		"Usage: echo <cmd> > <this_file> to send a command to VPU\n";

	return simple_read_from_buffer(user_buf, len, ppos, str, sizeof(str));
}

static const struct file_operations vpu_cmd_ops = {
	.open = simple_open,
	.write = write_cmd,
	.read = read_cmd,
};

static int smem_data_show(struct seq_file *m, void *private)
{
	char cbuf[SZ_64];
	struct smem_ctrl_data *smem = m->private;
	u32 offset = smem->offset;

	if (((offset >> 2) << 2) != offset) {
		seq_printf(m, "Error: offset (0x%x) must be a multiple of 4!\n",
				offset);
		goto smem_exit;
	}
	/*
	 * Print each memory line (4 32-bit words) containing the incremented
	 * offset. Stop reading if lower layer does not print anymore (or error)
	 */
	for (; offset <= smem->offset + smem->size; offset += 4 * sizeof(u32)) {
		int ret;
		ret = vpu_hfi_dump_smem_line(cbuf, sizeof(cbuf), offset);
		if (ret > 0)
			seq_printf(m, "%s", cbuf);
		else {
			if (ret == -EACCES)
				dprintk(VPU_WARN,
					"Cannot read outside of VPU mem!\n");
			if (ret == -ENOMEM)
				dprintk(VPU_ERR, "cbuf too small!\n");

			/* break 'for' loop if ret <= 0 */
			break;
		}
	}

smem_exit:
	return 0;
}

static int smem_data_open(struct inode *inode, struct file *file)
{
	return single_open(file, smem_data_show, inode->i_private);
}

static const struct file_operations smem_data_ops = {
	.open = smem_data_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release
};

struct dentry *init_smem_dir(struct dentry *root)
{
	struct dentry *smem, *attr;

	smem = debugfs_create_dir("smem", root);
	if (IS_ERR_OR_NULL(root)) {
		dprintk(VPU_ERR, "Failed to create smem directory\n");
		goto smem_dir_err;
	}

	attr = debugfs_create_x32("offset", RW_MODE, smem, &smem_ctrl.offset);
	if (IS_ERR_OR_NULL(attr)) {
		dprintk(VPU_ERR, "Failed to create smem/offset entry\n");
		goto smem_dir_err;
	}

	attr = debugfs_create_u32("size", RW_MODE, smem, &smem_ctrl.size);
	if (IS_ERR_OR_NULL(attr)) {
		dprintk(VPU_ERR, "Failed to create smem/size entry\n");
		goto smem_dir_err;
	}

	attr = debugfs_create_file("data", RW_MODE, smem, &smem_ctrl,
			&smem_data_ops);
	if (IS_ERR_OR_NULL(attr)) {
		dprintk(VPU_ERR, "Failed to create smem/data entry\n");
		goto smem_dir_err;
	}

	return smem;
smem_dir_err:
	return NULL;
}

static struct vpu_client *debug_client;

static ssize_t write_client(struct file *file, const char __user *user_buf,
	size_t len, loff_t *ppos)
{
	int ret = len;

	if (strcmp(user_buf, "get\n") == 0) {
		if (!debug_client)
			debug_client = vpu_open_kernel_client();
	} else if (strcmp(user_buf, "put\n") == 0) {
		if (debug_client) {
			vpu_close_client(debug_client);
			debug_client = NULL;
		}
	} else
		ret = -EINVAL;

	return ret;
}

static ssize_t read_client(struct file *file, char __user *user_buf,
	size_t len, loff_t *ppos)
{
	char str[] =
		"Usage: echo get/put > <this_file> to inc/dec VPU client\n";

	return simple_read_from_buffer(user_buf, len, ppos, str, sizeof(str));
}

static const struct file_operations vpu_client_ops = {
	.open = simple_open,
	.write = write_client,
	.read = read_client,
};

struct dentry *init_vpu_debugfs(struct vpu_dev_core *core)
{
	struct dentry *attr;
	struct dentry *root = debugfs_create_dir(VPU_DRV_NAME, NULL);

	if (IS_ERR_OR_NULL(root)) {
		dprintk(VPU_ERR, "Failed to create debugfs directory\n");
		goto failed_create_root;
	}

	/* create debug level file */
	attr = debugfs_create_x8("debug_level", S_IRUGO | S_IWUSR,
			root, &vpu_debug_level);
	if (IS_ERR_OR_NULL(attr)) {
		dprintk(VPU_ERR, "Failed to create debug_level file\n");
		goto failed_create_attr;
	}

	/* create maple_log file */
	attr = debugfs_create_file("maple_log", S_IRUGO, root, NULL,
			&maple_logging_ops);
	if (IS_ERR_OR_NULL(attr)) {
		dprintk(VPU_ERR, "Failed to create maple logging attribute\n");
		goto failed_create_attr;
	}

	/* create queue state file */
	attr = debugfs_create_file("queue_state", S_IRUGO, root, NULL,
			&queue_state_ops);
	if (IS_ERR_OR_NULL(attr)) {
		dprintk(VPU_ERR, "Failed to create queue state attribute\n");
		goto failed_create_attr;
	}

	/* create csr regs file */
	attr = debugfs_create_file("csr_regs", S_IRUGO, root, NULL,
			&csr_regs_ops);
	if (IS_ERR_OR_NULL(attr)) {
		dprintk(VPU_ERR, "Failed to create csr regs attribute\n");
		goto failed_create_attr;
	}

	/* create vpu regs file */
	/* TODO: create debugfs entry once IPC command is defined */

	/* create cmd entry */
	attr = debugfs_create_file("cmd", RW_MODE, root, NULL,
			&vpu_cmd_ops);
	if (IS_ERR_OR_NULL(attr)) {
		dprintk(VPU_ERR, "Failed to create cmd attribute\n");
		goto failed_create_attr;
	}

	/* create shared mem entry (smem dir + files) */
	attr = init_smem_dir(root);
	if (IS_ERR_OR_NULL(attr)) {
		dprintk(VPU_ERR, "Failed to create smem dir\n");
		goto failed_create_attr;
	}

	/* create client entry */
	attr = debugfs_create_file("client", RW_MODE, root, core,
			&vpu_client_ops);
	if (IS_ERR_OR_NULL(attr)) {
		dprintk(VPU_ERR, "Failed to create client attribute\n");
		goto failed_create_attr;
	}

	return root;

failed_create_attr:
	cleanup_vpu_debugfs(root);
	return attr ? attr : ERR_PTR(-ENOMEM);
failed_create_root:
	return root ? root : ERR_PTR(-ENOMEM);
}

void cleanup_vpu_debugfs(struct dentry *root)
{
	debugfs_remove_recursive(root);
}
