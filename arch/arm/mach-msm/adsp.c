/* arch/arm/mach-msm/adsp.c
 *
 * Register/Interrupt access for userspace aDSP library.
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
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/poll.h>
#include <linux/time.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/mm.h>
#include <asm/arch/msm_iomap.h>

#define ADSP_IOCTL_MAGIC 'q'
#define ADSP_IOCTL_GET_PHYS _IOWR(ADSP_IOCTL_MAGIC, 0, uint32_t *)

#define ADSP_MMAP_OFFSET_BUF0 (MSM_AD5_SIZE >> PAGE_SHIFT)
#define ADSP_MMAP_OFFSET_BUF1 ((MSM_AD5_SIZE + SZ_1M) >> PAGE_SHIFT)
#define ADSP_MMAP_OFFSET_BUF2 ((MSM_AD5_SIZE + SZ_2M) >> PAGE_SHIFT)

static DEFINE_SPINLOCK(adsp_lock);
static DECLARE_WAIT_QUEUE_HEAD(adsp_queue);
static int adsp_pending;
static int adsp_disabled;

#define REQUEST_IRQ_ON_OPEN (0)

static irqreturn_t adsp_irq_handler(int irq, void *data)
{
	unsigned long flags;

	spin_lock_irqsave(&adsp_lock, flags);
	adsp_pending++;
//	disable_irq(INT_ADSP_A11);
//	adsp_disabled = 1;
	spin_unlock_irqrestore(&adsp_lock, flags);

	wake_up(&adsp_queue);

	return IRQ_HANDLED;
}

static ssize_t adsp_read(struct file *file, char __user *buf,
			 size_t count, loff_t *pos)
{
	unsigned long flags;
	int ret;

	for (;;) {
		spin_lock_irqsave(&adsp_lock, flags);
		if (adsp_disabled) {
			adsp_disabled = 0;
			enable_irq(INT_ADSP_A11);
		}
		if (adsp_pending) {
			adsp_pending--;
			spin_unlock_irqrestore(&adsp_lock, flags);
			return 0;
		}
		spin_unlock_irqrestore(&adsp_lock, flags);

		ret = wait_event_interruptible(adsp_queue, adsp_pending);
		if (ret < 0)
			return ret;
	}

	return 0;
}

struct adsp_buf_info {
    uint32_t proc_virt;
    uint32_t kern_virt;
    uint32_t phys;
    uint32_t order;
};

static struct adsp_buf_info adsp_buf[3] = {
    { .kern_virt = -1 },
    { .kern_virt = -1 },
    { .kern_virt = -1 }
};

static int alloc_adsp_buf(int index, struct vm_area_struct *vma)
{
    int rc = 0, order;
    uint32_t virt, phys;
    uint32_t size = vma->vm_end - vma->vm_start;
    if (size > SZ_1M) {
        rc = -EINVAL;
        printk(KERN_ERR "adsp: dma buffer too big (%d bytes, max is SZ_1M)\n", size);
        goto done;
    }
    
    order = get_order(size);
    printk(KERN_INFO "adsp: allocating 2^%d (size %d) contiguous pages.\n", 
           order, size);

    virt = __get_dma_pages(GFP_USER, order);    
    if (!virt) {
        rc = -ENOMEM;
        printk(KERN_ERR "adsp: could not allocate 2^%d pages for user buffer.\n", order);
        goto done;
    }

    phys = virt_to_phys((void *)virt);   
    vma->vm_flags |= VM_IO | VM_RESERVED;
    if (io_remap_pfn_range(vma, vma->vm_start, 
                           phys >> PAGE_SHIFT,
                           vma->vm_end - vma->vm_start, 
                           vma->vm_page_prot)) {
        printk(KERN_ERR "adsp: could not remap DMA pages into process address space.\n");
        __free_pages((void *)virt, order);
        rc = -EAGAIN;
        goto done;
    }

    adsp_buf[index].proc_virt = vma->vm_start;
    adsp_buf[index].kern_virt = virt;
    adsp_buf[index].phys = phys;
    adsp_buf[index].order = order;
done:
    return rc;
}    

static int adsp_mmap(struct file *file, struct vm_area_struct *vma)
{
    int rc = 0;
	/* don't bother allowing any fancy mapping */

    switch (vma->vm_pgoff) {
    case 0:
        if ((vma->vm_end - vma->vm_start) > MSM_AD5_SIZE) {
            rc = -EINVAL;
            goto done;
        }
        vma->vm_flags |= VM_IO | VM_RESERVED;
        /* vma->vm_pgoff,*/
        if (io_remap_pfn_range(vma, vma->vm_start, MSM_AD5_PHYS >> PAGE_SHIFT,
                               vma->vm_end - vma->vm_start, vma->vm_page_prot)) {
            printk(KERN_ERR "adsp: could not remap DSP pages into process address space.\n");
            rc = -EAGAIN;
        }
        break;
    case ADSP_MMAP_OFFSET_BUF0:
        rc = alloc_adsp_buf(0, vma);
        break;
    case ADSP_MMAP_OFFSET_BUF1:
        rc = alloc_adsp_buf(1, vma);
        break;
    case ADSP_MMAP_OFFSET_BUF2: 
        rc = alloc_adsp_buf(2, vma);
        break;
    default:
        rc = -EINVAL;
    }

done:
	return rc;
}

static long adsp_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    int rc = -EINVAL;
    if (cmd == ADSP_IOCTL_GET_PHYS) {
        uint32_t addr; int c;
        rc = copy_from_user(&addr, (void *)arg, sizeof(addr));
        if (rc < 0)
            goto done;
        printk(KERN_INFO "adsp: ADSP_IOCTL_GET_PHYS for 0x%08x\n", addr);
        for (c = 0; c < sizeof(adsp_buf)/sizeof(adsp_buf[0]); c++) {
            if (adsp_buf[c].proc_virt == addr) {
                printk(KERN_INFO "adsp: get_page: virt 0x%08x, kern virt 0x%08x, phys 0x%08x, order 0x%08x\n",
                       adsp_buf[c].proc_virt,
                       adsp_buf[c].kern_virt,
                       adsp_buf[c].phys,
                       adsp_buf[c].order);
                addr = adsp_buf[c].phys;
                rc = copy_to_user((void *)arg, &addr, sizeof(addr));
                break;
            }
        }
    }
    else
        printk(KERN_ERR "adsp: unknown ioctl 0x%x\n", cmd);
done:
    return rc;
}

static int adsp_open(struct inode *inode, struct file *file)
{
#if REQUEST_IRQ_ON_OPEN
	return request_irq(INT_ADSP_A11, adsp_irq_handler,
                       IRQF_TRIGGER_RISING, "adsp", 0);
#else
    return 0;
#endif
}

static int adsp_release(struct inode *inode, struct file *file)
{
    int c;
    for (c = 0; c < sizeof(adsp_buf)/sizeof(adsp_buf[0]); c++) {
        if (adsp_buf[c].kern_virt >= 0) {
            printk(KERN_INFO "adsp: releasing buffer %d.\n", c);
            free_pages(adsp_buf[c].kern_virt, adsp_buf[c].order);
        }
    }

#if REQUEST_IRQ_ON_OPEN
	free_irq(INT_ADSP_A11, NULL);
#endif
	return 0;
}
    
static unsigned int adsp_poll(struct file *filp, struct poll_table_struct *wait) {
    unsigned mask = 0;

    if (adsp_pending)
        mask |= POLLIN;

    if (!mask) {
        poll_wait(filp, &adsp_queue, wait);
        if (adsp_pending)
            mask |= POLLIN;
    }

    return mask;
}

static struct file_operations adsp_fops = {
	.owner = THIS_MODULE,
	.read = adsp_read,
	.open = adsp_open,
	.mmap = adsp_mmap,
    .poll = adsp_poll,
    .unlocked_ioctl = adsp_ioctl,
	.release = adsp_release,
};

static struct miscdevice adsp_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "adsp",
	.fops = &adsp_fops,
};

static int __init adsp_init(void)
{
#if !REQUEST_IRQ_ON_OPEN
	int rc = request_irq(INT_ADSP_A11, adsp_irq_handler, IRQF_TRIGGER_RISING, "adsp", 0);
    if (rc < 0) 
        return rc;
#endif
	return misc_register(&adsp_device);
}

device_initcall(adsp_init);
