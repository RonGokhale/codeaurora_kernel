/* drivers/android/pmem.c
 *
 * Copyright (C) 2007 Google, Inc.
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

#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/mm.h>
#include <linux/list.h>
#include <linux/debugfs.h>
#include <linux/android_pmem.h>
#include <asm/io.h>
#include <asm/uaccess.h>

#define PMEM_BASE 0x4c00000
#define PMEM_SIZE 0x800000

#define PMEM_TOTAL_ORDER 11		/* log2(size/4k) */
#define PMEM_MAX_ORDER 128
#define PMEM_MIN_ALLOC PAGE_SIZE

#define PMEM_DEBUG 0

unsigned long pmem_base = 0;

#define PMEM_FLAGS_RELOCATABLE 0x1
#define PMEM_FLAGS_RESTRICTED 0x1 << 1

#if PMEM_DEBUG
struct pmem_pid {
	struct list_head list;
	pid_t pid;
};
#endif

struct pmem_data {
	//struct pmem_bits *bits;
	int index;
	unsigned int flags;
	pid_t creator;
	unsigned long restrict_start;
	unsigned long restrict_end;
#if PMEM_DEBUG
	struct list_head list;  /* lets keep a linked list of these so
				 * need be we can access them all */
	struct pmem_pid *pid;
#endif
};

#if PMEM_DEBUG
static LIST_HEAD(data_list);
#endif

struct pmem_bits {
	unsigned allocated:1;		/* 1 if allocated, 0 if free */
	unsigned order:7;		/* size of the region in pmem space */
};

#define PMEM_TOTAL_ENTRIES (1 << PMEM_TOTAL_ORDER)
struct pmem_bits pbits[PMEM_TOTAL_ENTRIES];


#define PMEM_IS_FREE(index) !(pbits[index].allocated)
#define PMEM_ORDER(index) pbits[index].order

#define PMEM_INDEX(bits) (bits - pbits)
#define PMEM_ENTRIES(index) (1 << PMEM_ORDER(index))

#define PMEM_BUDDY_INDEX(index) (index ^ (1 << PMEM_ORDER(index)))
#define PMEM_NEXT_INDEX(index) (index + (1 << PMEM_ORDER(index)))

#define PMEM_START_ADDR(index) (index * PMEM_MIN_ALLOC) + pmem_base
#define PMEM_BYTES(index) PMEM_ENTRIES(index) * PMEM_MIN_ALLOC
#define PMEM_END_ADDR(index) PMEM_START_ADDR(index) + PMEM_BYTES(index)

#define PMEM_RESTRICTED(data) (data->flags & PMEM_FLAGS_RESTRICTED)

/* pmem_sem protects the pbits array
 * a write lock should be held when modifying entries in pbits
 * a read lock should be held when reading data from bits or dereferencing
 * a pointer into pbits (ie by PMEM_ENTRIES, PMEM_ORDER, PMEM_FREE etc.)
 *
 * pmem_data_sem protects the pmem data fields so they are updated
 * atomically.  There are only 3 functions that modify file->private_data
 * mmap, release and restrict.  Release need not take the data sem as it
 * can only be called once for a given file.
 *
 * IF YOU TAKE BOTH LOCKS TAKE THEM IN THIS ORDER:
 * down(pmem_data_sem)->down(pmem_sem)->up(pem_sem)->up(pmem_data_sem)
 */
DECLARE_RWSEM(pmem_sem);
DECLARE_RWSEM(pmem_data_sem);


static int pmem_release(struct inode *, struct file *);
static int pmem_mmap(struct file *, struct vm_area_struct *);
static int pmem_open(struct inode *, struct file *);
static long pmem_ioctl(struct file *, unsigned int, unsigned long);

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

static int is_pmem_file (struct file *file)
{
	if (unlikely(file->f_dentry->d_inode->i_rdev !=
	    MKDEV(MISC_MAJOR, pmem_dev.minor)))
		return 0;
	return 1;
}

static int has_allocation (struct file *file)
{
	struct pmem_data *data;
	/* check is_pmem_file first if not accessed via pmem_file_ops */
	if (unlikely(!file->private_data))
		return 0;
	data = (struct pmem_data *)file->private_data;
	if (unlikely(data->index < 0 ||
		     data->index > PMEM_TOTAL_ENTRIES))
		return 0;
	return 1;
}

static int pmem_free(struct pmem_data* data)
{
	/* caller should hold the write lock on pmem_sem! */

	int buddy, curr = data->index;

	/* clean up the bitmap, merging any buddies */
	pbits[curr].allocated = 0;
	/* find a slots buddy Buddy# = Slot# ^ (1 << order)
	 * if the buddy is also free merge them
	 * repeat until the buddy is not free or end of the bitmap is reached
	 */
	do {
		buddy = PMEM_BUDDY_INDEX(curr);
		if (PMEM_IS_FREE(buddy) &&
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

	if (unlikely(!has_allocation(file)))
		return 0;
	data = (struct pmem_data *)file->private_data;

	down_write(&pmem_sem);
	ret = pmem_free(data);
	up_write(&pmem_sem);

#if PMEM_DEBUG
	{
		struct list_head *elt, *elt2;
		struct pmem_pid *pid;
		list_del(&data->list);
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
#if PMEM_DEBUG
	INIT_LIST_HEAD(&data->list);
	list_add(&data->list, &data_list);
	data->pid = kmalloc(sizeof(struct pmem_pid), GFP_KERNEL);
	data->pid->pid = current->pid;
	INIT_LIST_HEAD(&data->pid->list);
#endif
	data->flags = 0;
	data->index = -1;
	data->creator = current;

	down_write(&pmem_data_sem);
	file->private_data = data;
	up_write(&pmem_data_sem);
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

static int pmem_allocate(unsigned long len)
{
	/* caller should hold the write lock on pmem_sem! */
	/* return the corresponding pdata[] entry */
	int curr = 0;
	int end = PMEM_TOTAL_ENTRIES;
	int best_fit = -1;
	unsigned long order = pmem_order(len);

	if (order > PMEM_MAX_ORDER)
		return -1;

	/* look through the bitmap:
	 * 	if you find a free slot of the correct order use it
	 * 	otherwise, use the best fit (smallest with size > order) slot
	 */
	while (curr < end) {
		if (PMEM_IS_FREE(curr)) {
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
		return -1;

	/* now partition the best fit:
	 * 	split the slot into 2 buddies of order - 1
	 * 	repeat until the slot is of the correct order
	 */
	while (PMEM_ORDER(best_fit) > (unsigned char)order) {
		int buddy;
		PMEM_ORDER(best_fit) -= 1;
		buddy = PMEM_BUDDY_INDEX(best_fit);
		PMEM_ORDER(buddy) = PMEM_ORDER(best_fit);
	}
	pbits[best_fit].allocated = 1;
	return best_fit;
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

static unsigned long pmem_start_addr(struct pmem_data *data)
{
	if (!PMEM_RESTRICTED(data))
		return PMEM_START_ADDR(data->index);
	return  PMEM_START_ADDR(data->index) + data->restrict_start;
}

static unsigned long pmem_end_addr(struct pmem_data *data)
{
	if (!PMEM_RESTRICTED(data))
		return PMEM_END_ADDR(data->index);
	return  PMEM_START_ADDR(data->index) + data->restrict_end;
}

static int pmem_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct pmem_data *data;
	int index;
	unsigned long size =  vma->vm_end - vma->vm_start;
	unsigned long offset = vma->vm_pgoff;
	unsigned long pfn;

	down_write(&pmem_data_sem);
	data = (struct pmem_data*)file->private_data;
	/* if file->private_data == unalloced, alloc*/
	if (data && data->index == -1) {
		down_write(&pmem_sem);
		index = pmem_allocate(vma->vm_end - vma->vm_start);
		up_write(&pmem_sem);
		if (index != -1)
			data->index = index;
	}
	up_write(&pmem_data_sem);

	if (!has_allocation(file))
		return -EINVAL;

	if (PMEM_RESTRICTED(data))
		size = data->restrict_end - data->restrict_start;

	/* check that the offset + size is still within the allocated region */
	if (pmem_start_addr(data) + offset + size > pmem_end_addr(data)) {
		return -EOVERFLOW;
	}

	pfn = (pmem_start_addr(data) + offset) >> PAGE_SHIFT;
	vma->vm_pgoff = pfn;
	vma->vm_page_prot = phys_mem_access_prot(file, pfn, size,
                                                 vma->vm_page_prot);
	if (remap_pfn_range(vma, vma->vm_start, pfn, size, vma->vm_page_prot))
                return -EAGAIN;
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
	if (file == NULL || !is_pmem_file(file) || !has_allocation(file))
		return -1;

	data = (struct pmem_data*)file->private_data;
	if (data->index == -1)
		return -1;

	*start = pmem_start_addr(data);
	*end = pmem_end_addr(data);

	return 0;
}

void put_pmem_file(unsigned long fd)
{
	struct file *file;

	file = fget(fd);
	if (file == NULL || !is_pmem_file(file))
		return;
	if (has_allocation(file))
		fput(file);
	fput(file);
}

static int pmem_restrict(struct pmem_addr* addr, struct file *file)
{
	struct pmem_data* data = (struct pmem_data*)file->private_data;
	int ret = 0;

	if (unlikely(!has_allocation(file)))
		return -EINVAL;

	if (unlikely(((unsigned long)addr->start > PMEM_BYTES(data->index)) ||
	    ((unsigned long)addr->end > PMEM_BYTES(data->index))))
		return -EINVAL;

	/* only the creator can make restrictions */
	if (unlikely(current != data->creator))
		return -EINVAL;

	down_write(&pmem_data_sem);
		/* round start down to the nearest page and end up */
		data->restrict_start = (unsigned long)addr->start&PAGE_MASK;
		data->restrict_end = PAGE_ALIGN((unsigned long)addr->end);
		data->flags |= PMEM_FLAGS_RESTRICTED;
	up_write(&pmem_data_sem);
	return ret;
}

static void pmem_get_size(struct pmem_addr* addr, struct file *file)
{
	struct pmem_data* data = (struct pmem_data*)file->private_data;

	if (!has_allocation(file)) {
		addr->start = NULL;
		addr->end = NULL;
		return;
	}

	if (PMEM_RESTRICTED(data)) {
		addr->start = (void*)data->restrict_start;
		addr->end = (void*)data->restrict_end;
	} else {
		addr->start = 0;
		addr->end = (void*)(PMEM_BYTES(data->index));
	}

	return;
}

static long pmem_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct pmem_data* data;
	struct pmem_addr addr;

	switch(cmd) {
		case PMEM_GET_PHYS:
			if (!has_allocation(file)) {
				addr.start = NULL;
				addr.end = NULL;
			} else { 
				data = (struct pmem_data*)file->private_data;
				addr.start = (void*)pmem_start_addr(data);
				addr.end = (void*)pmem_end_addr(data);
			}
			if (copy_to_user((void __user *)arg, &addr,
					  sizeof(struct pmem_addr)))
				return -EFAULT;
			break;
		case PMEM_RESTRICT:
			if (copy_from_user(&addr, (void __user *)arg,
					   sizeof(struct pmem_addr)))
				return -EFAULT;
			return pmem_restrict(&addr, file);
			break;
		case PMEM_GET_SIZE:
			pmem_get_size(&addr, file);
			if (copy_to_user((void __user *)arg, &addr,
					  sizeof(struct pmem_addr)))
				return -EFAULT;
			break;
		default:
			return -EINVAL;
	}
	return 0;
}

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
		if (!data->index)
			continue;
		n += sprintf(buffer + n, "%lx\t%lx\t",
			     PMEM_START_ADDR(data->index),
			     PMEM_END_ADDR(data->index));
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
	pmem = ioremap(PMEM_BASE, PMEM_SIZE);
	if (pmem == 0)
		return -1;
	pmem_base = PMEM_BASE;
	memset(pbits, 0, sizeof(struct pmem_bits) * PMEM_TOTAL_ENTRIES);
	PMEM_ORDER(0) = PMEM_TOTAL_ORDER;

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
