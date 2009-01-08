/* arch/arm/mach-msm/htc_acoustic.c
 *
 * Copyright (C) 2007-2008 HTC Corporation
 * Author: Laurence Chen <Laurence_Chen@htc.com>
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
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/uaccess.h>
#include <linux/mutex.h>

#include <mach/msm_smd.h>
#include <mach/msm_rpcrouter.h>
#include <mach/msm_iomap.h>

#include "smd_private.h"

#define ACOUSTIC_IOCTL_MAGIC 'p'
#define ACOUSTIC_ARM11_DONE	_IOW(ACOUSTIC_IOCTL_MAGIC, 22, unsigned int)
#define ACOUSTIC_ALLOC_SMEM	_IOW(ACOUSTIC_IOCTL_MAGIC, 23, unsigned int)

#define HTCRPOG 0x30100002
#define HTCVERS 0
#define ONCRPC_SET_MIC_BIAS_PROC       (1)
#define ONCRPC_ACOUSTIC_INIT_PROC      (5)
#define ONCRPC_ALLOC_ACOUSTIC_MEM_PROC (6)

#define HTC_ACOUSTIC_TABLE_SIZE        (0x10000)
#define ACOUSTICE(x...) printk(KERN_ERR "[ACOUSTIC] " x)

struct set_smem_req {
	struct rpc_request_hdr hdr;
	uint32_t size;
};

struct set_smem_rep {
	struct rpc_reply_hdr hdr;
	int n;
};

struct set_acoustic_req {
	struct rpc_request_hdr hdr;
} req;

struct set_acoustic_rep {
	struct rpc_reply_hdr hdr;
	int n;
} rep;

static uint32_t htc_acoustic_phy_addr;
static uint32_t htc_acoustic_vir_addr;
static struct msm_rpc_endpoint *endpoint;
struct mutex rpc_connect_mutex;

static int is_rpc_connect()
{
	mutex_lock(&rpc_connect_mutex);
	if (endpoint == NULL) {
		endpoint = msm_rpc_connect(HTCRPOG, HTCVERS, 0);
		if (IS_ERR(endpoint)) {
			ACOUSTICE("%s: init rpc failed! rc = %ld\n",
				__func__, PTR_ERR(endpoint));
			mutex_unlock(&rpc_connect_mutex);
			return 0;
		}
	}
	mutex_unlock(&rpc_connect_mutex);
	return 1;
}


int turn_mic_bias_on(int on)
{
	struct mic_bias_req {
		struct rpc_request_hdr hdr;
		uint32_t on;
	} req;

	if (!is_rpc_connect())
		return -1;

	req.on = cpu_to_be32(on);
	return msm_rpc_call(endpoint, ONCRPC_SET_MIC_BIAS_PROC,
			    &req, sizeof(req), 5 * HZ);
}
EXPORT_SYMBOL(turn_mic_bias_on);

static int acoustic_mmap(struct file *file, struct vm_area_struct *vma)
{
	unsigned long pgoff;
	size_t size = vma->vm_end - vma->vm_start;
	if (vma->vm_pgoff != 0)
		return -EINVAL;

	if (size <= HTC_ACOUSTIC_TABLE_SIZE)
		pgoff = htc_acoustic_phy_addr >> PAGE_SHIFT;
	else
		return -EINVAL;

	vma->vm_flags |= VM_IO | VM_RESERVED;

	if (io_remap_pfn_range(vma, vma->vm_start, pgoff,
			      size, vma->vm_page_prot))
		return -EAGAIN;

	return 0;
}

static int acoustic_open(struct inode *inode, struct file *file)
{
	if (!is_rpc_connect())
		return -1;
	return 0;
}

static int acoustic_release(struct inode *inode, struct file *file)
{
	return 0;
}

static long acoustic_ioctl(struct file *file, unsigned int cmd,
			   unsigned int arg)
{
	int rc, reply_value;
	struct set_smem_req req_smem;
	struct set_smem_rep rep_smem;
	struct set_acoustic_req req;
	struct set_acoustic_rep rep;

	switch (cmd) {
	case ACOUSTIC_ALLOC_SMEM:
#if 0
		AACOUSTICE("ioctl ACOUSTIC_ALLOC_SMEM %d called.\n", rc);
#endif
		req_smem.size = cpu_to_be32(HTC_ACOUSTIC_TABLE_SIZE);
		rc = msm_rpc_call_reply(endpoint,
					ONCRPC_ALLOC_ACOUSTIC_MEM_PROC,
					&req_smem, sizeof(req_smem),
					&rep_smem, sizeof(rep_smem),
					5 * HZ);

		reply_value = be32_to_cpu(rep_smem.n);
		if (reply_value != 0 || rc < 0) {
			ACOUSTICE("ALLOC_ACOUSTIC_MEM_PROC failed %d.\n", rc);
			return rc;
		}

		htc_acoustic_vir_addr =
				(uint32_t)smem_alloc(SMEM_ID_VENDOR1, HTC_ACOUSTIC_TABLE_SIZE);
		htc_acoustic_phy_addr = MSM_SHARED_RAM_PHYS +
					(htc_acoustic_vir_addr -
						(uint32_t)MSM_SHARED_RAM_BASE);
		htc_acoustic_phy_addr = (htc_acoustic_phy_addr + 4095 & ~4095);

		if (htc_acoustic_phy_addr <= 0) {
			ACOUSTICE("htc_acoustic_phy_addr wrong.\n");
			return -EFAULT;
		}

		return 0;
		break;

	case ACOUSTIC_ARM11_DONE:
#if 0
		pr_info("ioctl ACOUSTIC_ARM11_DONE called %d.\n", current->pid);
#endif
		rc = msm_rpc_call_reply(endpoint,
					ONCRPC_ACOUSTIC_INIT_PROC, &req,
					sizeof(req), &rep, sizeof(rep),
					5 * HZ);

		reply_value = be32_to_cpu(rep.n);
		if (reply_value != 0 || rc < 0) {
			ACOUSTICE("ONCRPC_ACOUSTIC_INIT_PROC failed %d.\n", rc);
			return reply_value;
		} else {
#if 0
			pr_info("ONCRPC_ACOUSTIC_INIT_PROC success.\n");
#endif
			return 0;
		}
		break;

	default:
		rc = -EINVAL;
	}
	return 0;
}


static struct file_operations acoustic_fops = {
	.owner = THIS_MODULE,
	.open = acoustic_open,
	.release = acoustic_release,
	.mmap = acoustic_mmap,
	.unlocked_ioctl = acoustic_ioctl,
};

static struct miscdevice acoustic_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "htc-acoustic",
	.fops = &acoustic_fops,
};

static int __init acoustic_init(void)
{
	int ret;

	ret = misc_register(&acoustic_misc);
	if (ret < 0) {
		ACOUSTICE("failed to register misc device!\n");
		return ret;
	}
	mutex_init(&rpc_connect_mutex);

	return 0;
}

static void __exit acoustic_exit(void)
{
	int ret;

	ret = misc_deregister(&acoustic_misc);
	if (ret < 0)
		ACOUSTICE("failed to unregister misc device!\n");
}

module_init(acoustic_init);
module_exit(acoustic_exit);

MODULE_AUTHOR("Laurence Chen <Laurence_Chen@htc.com>");
MODULE_DESCRIPTION("HTC acoustic driver");
MODULE_LICENSE("GPL");
