/* arch/arm/mach-msm/hw3d.c
 *
 * Register/Interrupt access for userspace 3D library.
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
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/poll.h>
#include <linux/time.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/mm.h>
#include <linux/clk.h>

static DEFINE_SPINLOCK(hw3d_lock);
static DECLARE_WAIT_QUEUE_HEAD(hw3d_queue);
static int hw3d_pending;
static int hw3d_disabled;

static struct clk *grp_clk;
static struct clk *imem_clk;

static irqreturn_t hw3d_irq_handler(int irq, void *data)
{
	unsigned long flags;

	spin_lock_irqsave(&hw3d_lock, flags);
	disable_irq(INT_GRAPHICS);
	hw3d_pending = 1;
	hw3d_disabled = 1;
	spin_unlock_irqrestore(&hw3d_lock, flags);

	wake_up(&hw3d_queue);

	return IRQ_HANDLED;
}

static ssize_t hw3d_read(struct file *file, char __user *buf,
			 size_t count, loff_t *pos)
{
	unsigned long flags;
	int ret;

	for (;;) {
		spin_lock_irqsave(&hw3d_lock, flags);
		if (hw3d_pending) {
			hw3d_pending = 0;
			spin_unlock_irqrestore(&hw3d_lock, flags);
			return 0;
		}
		if (hw3d_disabled) {
			hw3d_disabled = 0;
			enable_irq(INT_GRAPHICS);
		}
		spin_unlock_irqrestore(&hw3d_lock, flags);

		ret = wait_event_interruptible(hw3d_queue, hw3d_pending);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int hw3d_mmap(struct file *file, struct vm_area_struct *vma)
{
	/* don't bother allowing any fancy mapping */

	if (vma->vm_pgoff != 0)
		return -EINVAL;

	if ((vma->vm_end - vma->vm_start) > (1024*1024))
		return -EINVAL;

	vma->vm_flags |= VM_IO | VM_RESERVED;

	/* vma->vm_pgoff,*/
	if (io_remap_pfn_range(vma, vma->vm_start, 0xA0000000 >> PAGE_SHIFT,
			      vma->vm_end - vma->vm_start, vma->vm_page_prot))
		return -EAGAIN;

	return 0;
}

static int hw3d_open(struct inode *inode, struct file *file)
{
	clk_enable(imem_clk);
	clk_enable(grp_clk);
	return 0;
}

static int hw3d_release(struct inode *inode, struct file *file)
{
	return 0;
}

static struct file_operations hw3d_fops = {
	.owner = THIS_MODULE,
	.read = hw3d_read,
	.open = hw3d_open,
	.mmap = hw3d_mmap,
	.release = hw3d_release,
};

static struct miscdevice hw3d_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "hw3d",
	.fops = &hw3d_fops,
};

static int __init hw3d_init(void)
{
	int ret;

	grp_clk = clk_get(NULL, "grp_clk");
	if (IS_ERR(grp_clk))
		return PTR_ERR(grp_clk);
	
	imem_clk = clk_get(NULL, "imem_clk");
	if (IS_ERR(imem_clk)) {
		clk_put(grp_clk);
		return PTR_ERR(imem_clk);
	}
	ret = request_irq(INT_GRAPHICS, hw3d_irq_handler,
			  IRQF_TRIGGER_HIGH, "hw3d", 0);
	if (ret) {
		clk_put(grp_clk);
		clk_put(imem_clk);
		return ret;
	}

	return misc_register(&hw3d_device);
}

device_initcall(hw3d_init);
