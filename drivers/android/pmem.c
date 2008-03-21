/* drivers/android/pmem.c
 *
 * Copyright (C) 2007 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
/* TODO: do I need vm ops to cover the fork case?
 *
 */

#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/mm.h>
#include <linux/list.h>
#include <linux/debugfs.h>
#include <asm/io.h>

#define PMEM_BASE 0x4c00000
#define PMEM_SIZE 0x800000

#define PMEM_TOTAL_ORDER 11		/* log2(size/4k) */
#define PMEM_MAX_ORDER 15
#define PMEM_MIN_ALLOC PAGE_SIZE

#define PMEM_DEBUG 0

unsigned long pmem_base = 0;

#define PMEM_FLAGS_ALLOCATED 0x1
#define PMEM_FLAGS_RELOCATABLE 0x1 << 1
#define PMEM_FLAGS_RESTRICTED 0x1 << 2

#if PMEM_DEBUG
struct pmem_pid {
	struct list_head list;
	pid_t pid;
};
#endif

struct pmem_data {
	struct list_head list;  /* lets keep a linked list of these so
				 * need be we can access them all */
	struct pmem_bits *bits;
#if PMEM_DEBUG
	struct pmem_pid *pid;
#endif
};

struct pmem_bits {
	unsigned order:4;		/* size of the region in pmem space */
	unsigned flags:4;		/* size of the region in pmem space */
};


#define PMEM_TOTAL_ENTRIES (1 << PMEM_TOTAL_ORDER)
struct pmem_bits pbits[PMEM_TOTAL_ENTRIES];

static LIST_HEAD(data_list);

#define PMEM_ENTRIES(bits) (1 << (*bits).order)

#define PMEM_ORDER(index) pbits[index].order
#define PMEM_FLAGS(index) pbits[index].flags

#define PMEM_FREE(index) !(PMEM_FLAGS(index) & PMEM_FLAGS_ALLOCATED)

#define PMEM_INDEX(bits) (bits - pbits)

#define PMEM_BUDDY_INDEX(index) (index ^ (1 << PMEM_ORDER(index)))
#define PMEM_NEXT_INDEX(index) (index + (1 << PMEM_ORDER(index)))

#define PMEM_START_ADDR(bits) (PMEM_INDEX(bits) * PMEM_MIN_ALLOC) + \
	pmem_base
#define PMEM_END_ADDR(bits) PMEM_START_ADDR(bits) + \
	(PMEM_ENTRIES(bits) * PMEM_MIN_ALLOC)

DECLARE_MUTEX(pmem_sem);


static int valid_pmem_file (struct file *file)
{
	if (unlikely(!file->private_data))
		return 0;
	/* XXX: fix me, i'm not that safe... */
	if (unlikely(((struct pmem_data*)file->private_data)->bits < pbits ||
		     ((struct pmem_data*)file->private_data)->bits > pbits +
				     PMEM_TOTAL_ENTRIES))
		return 0;
	return 1;
}

static int pmem_free(struct pmem_data* data)
{
	int buddy, curr = PMEM_INDEX(data->bits);

	/* clean up the bitmap, merging any buddies */
	PMEM_FLAGS(curr) = 0;
	/* find a slots buddy Buddy# = Slot# ^ (1 << order)
	 * if the buddy is also free merge them
	 * repeat until the buddy is not free or end of the bitmap is reached
	 */
	do {
		buddy = PMEM_BUDDY_INDEX(curr);
		if (PMEM_FREE(buddy) &&
		    PMEM_ORDER(buddy) == PMEM_ORDER(curr)) {
			PMEM_ORDER(buddy)++;
			PMEM_ORDER(curr)++;
			curr = min(buddy, curr);
		} else {
			break;
		}
	} while (curr < PMEM_TOTAL_ENTRIES);

	return 0;
}

static int pmem_release(struct inode* inode, struct file *file)
{
	struct pmem_data *data;
	int ret;
	if (!valid_pmem_file(file))
		return -1;
	data = (struct pmem_data *)file->private_data;
	down(&pmem_sem);
	ret = pmem_free(data);
	up(&pmem_sem);
	list_del(&data->list);
#if PMEM_DEBUG
	{
		struct list_head *elt, *elt2;
		struct pmem_pid *pid;
		list_for_each_safe(elt, elt2, &data->pid->list) {
			pid = list_entry(elt, struct pmem_pid, list);
			list_del(&pid->list);
			kfree(pid);
		}
	}
#endif
	kfree(data);
	file->private_data = NULL;
	return ret;
}

static int pmem_open(struct inode *inode, struct file *file)
{
	struct pmem_data* data;
	/* setup file->private_data to indicate its unmapped */
	/*  you can only open a pmem device one time */
	if (file->private_data != NULL)
		return -1;
	/* XXX: use vmalloc? */
	data = kmalloc(sizeof(struct pmem_data), GFP_KERNEL);
	INIT_LIST_HEAD(&data->list);
	list_add(&data->list, &data_list);
#if PMEM_DEBUG
	data->pid = kmalloc(sizeof(struct pmem_pid), GFP_KERNEL);
	data->pid->pid = current->pid;
	INIT_LIST_HEAD(&data->pid->list);
#endif
	data->bits = NULL;
	file->private_data = data;
	return 0;
}

static unsigned long pmem_order(unsigned long len) {
	int i;

	len /= PMEM_MIN_ALLOC;
	len--;
	for (i = 0; i < sizeof(len); i++)
		if (len >> i == 0)
			break;
	return i;
}

static struct pmem_bits *pmem_allocate(unsigned long len)
{
	/* return the corresponding pdata[] entry */
	int curr = 0;
	int end = PMEM_TOTAL_ENTRIES;
	int best_fit = -1;
	unsigned long order = pmem_order(len);

	if (order > PMEM_MAX_ORDER)
		return NULL;

	/* look through the bitmap:
	 * 	if you find a free slot of the correct order use it
	 * 	otherwise, use the best fit (smallest with size > order) slot
	 */
	while (curr < end) {
		if (PMEM_FREE(curr)) {
			if (PMEM_ORDER(curr) == (unsigned char)order) {
				/* set the not free bit and clear others */
				best_fit = curr;
				break;
			}
			if (PMEM_ORDER(curr) > (unsigned char)order &&
			    (best_fit < 0 ||
			     PMEM_ORDER(curr) < PMEM_ORDER(best_fit)))
				best_fit = curr;
		}
		curr = PMEM_NEXT_INDEX(curr);
	}

	/* if best_fit < 0, there are no suitable slots,
	 * return an error
	 */
	if (best_fit < 0)
		return NULL;

	/* now partition the best fit:
	 * 	split the slot into 2 buddies of order - 1
	 * 	repeat until the slot is of the correct order
	 */
	while (PMEM_ORDER(best_fit) > (unsigned char)order) {
		int buddy;
		PMEM_ORDER(best_fit) -= 1;
		buddy = PMEM_BUDDY_INDEX(best_fit);
		PMEM_ORDER(buddy) = PMEM_ORDER(best_fit);
		PMEM_FLAGS(buddy) = 0;
	}
	PMEM_FLAGS(best_fit) = PMEM_FLAGS_ALLOCATED;
	return &pbits[best_fit];
}

static pgprot_t phys_mem_access_prot(struct file *file, unsigned long addr,
				     unsigned long size, pgprot_t vma_prot)
{
	/* check this i think we want this noncached by default */
#ifdef pgprot_noncached
        if (file->f_flags & O_SYNC || addr >= virt_to_phys(high_memory))
		return pgprot_noncached(vma_prot);
#endif
	return vma_prot;
}

static int pmem_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct pmem_data *data = (struct pmem_data*)file->private_data;
	struct pmem_bits *bits;
	unsigned long size =  vma->vm_end - vma->vm_start;
	unsigned long offset = vma->vm_pgoff;
	unsigned long pfn;

	/* if file->private_data == unalloced, alloc*/
	if (data && data->bits == NULL) {
		down(&pmem_sem);
		bits = pmem_allocate(vma->vm_end - vma->vm_start);
		up(&pmem_sem);
		if (!bits)
			return -EINVAL;
		data->bits = bits;
	}

	if (!valid_pmem_file(file))
		return -EINVAL;

	/* check that the offset + size is still within the allocated region */
	if (PMEM_START_ADDR(data->bits) + offset + size >
	    PMEM_END_ADDR(data->bits))
		return -EINVAL;

	pfn = (PMEM_START_ADDR(data->bits) + offset) >> PAGE_SHIFT;
	vma->vm_pgoff = pfn;
	vma->vm_page_prot = phys_mem_access_prot(file, pfn, size,
                                                 vma->vm_page_prot);
	if (remap_pfn_range(vma, vma->vm_start, pfn, size, vma->vm_page_prot)) {
		pmem_free(data);
                return -EAGAIN;
	}
#if PMEM_DEBUG
	{
		struct pmem_pid *pid;
		pid = kmalloc(sizeof(struct pmem_pid), GFP_KERNEL);
		pid->pid = current->pid;
		INIT_LIST_HEAD(&pid->list);
		list_add(&pid->list, &data->pid->list);
	}
#endif
	return 0;
}

int get_pmem_file(unsigned long fd, unsigned long *start, unsigned long *end)
{
	struct file *file;
	struct pmem_data *data;

	file = fget(fd);
	if (file == NULL || !valid_pmem_file(file))
		return -1;

	data = (struct pmem_data*)file->private_data;
	if (!data->bits)
		return -1;

	*start = PMEM_START_ADDR(data->bits);
	*end = PMEM_END_ADDR(data->bits);
	return 0;
}

void put_pmem_file(unsigned long fd)
{
	struct file *file;

	file = fget(fd);
	if (file == NULL)
		return;
	if(valid_pmem_file(file))
		fput(file);
	fput(file);
}

static long pmem_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	switch(cmd) {
		default:
			return -EINVAL;
	}
	return 0;
}

struct file_operations pmem_fops = {
	.release = pmem_release,
	.mmap = pmem_mmap,
	.open = pmem_open,
	.unlocked_ioctl = pmem_ioctl,
};

static struct miscdevice pmem_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "pmem",
	.fops = &pmem_fops,
};

#if PMEM_DEBUG
#define DEBUG_BUFMAX 4096
static char buffer[DEBUG_BUFMAX];
static int n;

static ssize_t debug_open(struct inode *inode, struct file *file)
{
	struct list_head *elt, *elt2;
	struct pmem_data *data;
	struct pmem_pid *pid;
	n = 0;

	n = sprintf(buffer, "from\t\tto\t\tpids (opener, mappers....)\n");
	list_for_each(elt, &data_list) {
		data = list_entry(elt, struct pmem_data, list);
		if (!data->bits)
			continue;
		n += sprintf(buffer + n, "%lx\t%lx\t",
			     PMEM_START_ADDR(data->bits),
			     PMEM_END_ADDR(data->bits));
		list_for_each(elt2, &data->pid->list) {
			pid = list_entry(elt2, struct pmem_pid, list);
			n += sprintf(buffer + n, "%u ", pid->pid);
		}
		n += sprintf(buffer + n, "\n");
	}
	n++;
	buffer[n] = 0;
	return 0;
}


static ssize_t debug_read(struct file *file, char __user *buf, size_t count,
			   loff_t *ppos)
{
	return simple_read_from_buffer(buf, count, ppos, buffer, n);
}

static struct file_operations debug_fops = {
	.read = debug_read,
	.open = debug_open,
};
#endif

static int __init pmem_init(void)
{
	int err;
	unsigned char *pmem;
	err = misc_register(&pmem_dev);
	if (err) {
		printk(KERN_ALERT "Unable to register pmem driver!\n");
		return err;
	}
/*
	pmem_base = virt_to_phys((void*)__get_free_pages(GFP_KERNEL,
							 PMEM_TOTAL_ORDER));
*/
	pmem = ioremap(PMEM_BASE, PMEM_SIZE);
	if (pmem == 0)
		return -1;
	pmem_base = PMEM_BASE;
	PMEM_ORDER(0) = PMEM_TOTAL_ORDER;
	PMEM_FLAGS(0) = 0;

#if PMEM_DEBUG
	debugfs_create_file("pmem", S_IFREG | S_IRUGO, NULL, NULL, &debug_fops);
#endif
	return 0;
}

static void __exit pmem_exit(void)
{
	misc_deregister(&pmem_dev);
	free_pages(pmem_base, PMEM_TOTAL_ORDER);
}

module_init(pmem_init);

module_exit(pmem_exit);
