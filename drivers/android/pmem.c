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
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/mm.h>
#include <linux/list.h>
#include <linux/debugfs.h>
#include <linux/android_pmem.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/cacheflush.h>
#include <asm/arch/board.h>

#define PMEM_MAX_ORDER 128
#define PMEM_MIN_ALLOC PAGE_SIZE

#define PMEM_DEBUG 1

#define PMEM_FLAGS_RELOCATABLE 0x1
#define PMEM_FLAGS_SUBALLOC 0x1 << 1
#define PMEM_FLAGS_REVOKED 0x1 << 2

struct pmem_vma {
	struct list_head list;
	struct vm_area_struct *vma;
#if PMEM_DEBUG
	pid_t pid;
#endif
};

struct pmem_data {
	int index;
	unsigned int flags;
	unsigned long restrict_start;
	unsigned long restrict_len;
	struct list_head vma_list;
#if PMEM_DEBUG
	struct list_head list;  /* lets keep a linked list of these so
				 * need be we can access them all */
#endif
};

#if PMEM_DEBUG
static LIST_HEAD(data_list);
#endif

struct pmem_bits {
	unsigned allocated:1;		/* 1 if allocated, 0 if free */
	unsigned order:7;		/* size of the region in pmem space */
};

unsigned long pmem_base;
unsigned char __iomem *pmem_vbase;
unsigned long pmem_size;
unsigned long pmem_num_entries; 
struct page *garbage_page;
unsigned long garbage_pfn;
struct pmem_bits *pbits;

#define PMEM_IS_FREE(index) !(pbits[index].allocated)
#define PMEM_ORDER(index) pbits[index].order

#define PMEM_BUDDY_INDEX(index) (index ^ (1 << PMEM_ORDER(index)))
#define PMEM_NEXT_INDEX(index) (index + (1 << PMEM_ORDER(index)))

#define PMEM_START_ADDR(index) ((index * PMEM_MIN_ALLOC) + pmem_base)
#define PMEM_LEN(index) ((1 << PMEM_ORDER(index)) * PMEM_MIN_ALLOC)
#define PMEM_END_ADDR(index) (PMEM_START_ADDR(index) + PMEM_LEN(index))

#define PMEM_START_VADDR(index) ((index * PMEM_MIN_ALLOC) + pmem_vbase)
#define PMEM_END_VADDR(index) (PMEM_START_VADDR(index) + PMEM_LEN(index))


#define PMEM_SUBALLOC(data) (data->flags & PMEM_FLAGS_SUBALLOC)
#define PMEM_REVOKED(data) (data->flags & PMEM_FLAGS_REVOKED)

#define PMEM_IS_PAGE_ALIGNED(addr) (!((addr) & (~PAGE_MASK)))

/* pmem_sem protects the pbits array
 * a write lock should be held when modifying entries in pbits
 * a read lock should be held when reading data from bits or dereferencing
 * a pointer into pbits
 *
 * pmem_data_sem protects the pmem data fields so they are updated
 * atomically.  There are only 3 functions that modify file->private_data
 * mmap, release and restrict.  Release need not take the data sem as it
 * can only be called once for a given file.
 *
 * pmem_vma_sem protects the list of vma's hanging off each data sem.
 * this lock should be held in pmem_vma_open and pmem_vma_close and when
 * walking the vma list for revoke or to display debug info
 *
 * IF YOU TAKE BOTH LOCKS TAKE THEM IN THIS ORDER:
 * down(pmem_data_sem) => down(pmem_sem) => up(pem_sem) => up(pmem_data_sem)
 */
DECLARE_RWSEM(pmem_sem);
DECLARE_RWSEM(pmem_data_sem);
DECLARE_RWSEM(pmem_vma_sem);
unsigned have_pmem_data_sem_write = 0;
#define DOWN_WRITE_PMEM_DATA_SEM \
do { down_write(&pmem_data_sem); \
     have_pmem_data_sem_write = 1; } while (0)

#define UP_WRITE_PMEM_DATA_SEM \
do { up_write(&pmem_data_sem); \
     have_pmem_data_sem_write = 0; } while (0)

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
	if (unlikely(data->index < 0 || data->index > pmem_num_entries))
		return 0;
	return 1;
}

static int pmem_free(struct pmem_data* data)
{
	/* caller should hold the write lock on pmem_sem! */

	int buddy, curr = data->index;
	memset(PMEM_START_VADDR(curr), 0, PMEM_LEN(curr));

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
	} while (curr < pmem_num_entries);

	return 0;
}

static int pmem_release(struct inode* inode, struct file *file)
{
	struct pmem_data *data = (struct pmem_data *)file->private_data;
	struct pmem_vma *vma;
	struct list_head *elt, *elt2;
	int ret = 0;

#if PMEM_DEBUG
	list_del(&data->list);
#endif
	if (unlikely(!has_allocation(file)))
		return 0;

	file->private_data = NULL;

	down_write(&pmem_sem);
	if (!PMEM_SUBALLOC(data)) {
		ret = pmem_free(data);
	}
	up_write(&pmem_sem);

	list_for_each_safe(elt, elt2, &data->vma_list) {
		vma = list_entry(elt, struct pmem_vma, list);
		list_del(elt);
		kfree(vma);
	}
	kfree(data);
	return ret;
}

static int pmem_open(struct inode *inode, struct file *file)
{
	struct pmem_data* data;
	/* setup file->private_data to indicate its unmapped */
	/*  you can only open a pmem device one time */
	if (file->private_data != NULL)
		return -1;
	data = kmalloc(sizeof(struct pmem_data), GFP_KERNEL);
#if PMEM_DEBUG
	INIT_LIST_HEAD(&data->list);
	list_add(&data->list, &data_list);
#endif
	data->flags = 0;
	data->index = -1;
	INIT_LIST_HEAD(&data->vma_list);

	DOWN_WRITE_PMEM_DATA_SEM;
	file->private_data = data;
	UP_WRITE_PMEM_DATA_SEM;
	return 0;
}

static unsigned long pmem_order(unsigned long len) {
	int i;

	len = (len + PMEM_MIN_ALLOC - 1)/PMEM_MIN_ALLOC;
	len--;
	for (i = 0; i < sizeof(len)*8; i++)
		if (len >> i == 0)
			break;
	return i;
}

static int pmem_allocate(unsigned long len)
{
	/* caller should hold the write lock on pmem_sem! */
	/* return the corresponding pdata[] entry */
	int curr = 0;
	int end = pmem_num_entries;
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

static pgprot_t phys_mem_access_prot(struct file *file, pgprot_t vma_prot)
{
	/* check this i think we want this noncached by default */
#ifdef pgprot_noncached
	if (file->f_flags & O_SYNC)
		return pgprot_noncached(vma_prot);
#endif
	return vma_prot;
}

static unsigned long pmem_start_addr(struct pmem_data *data)
{
	unsigned long ret;

	if (!have_pmem_data_sem_write)
		down_read(&pmem_data_sem);
	if (!PMEM_SUBALLOC(data))
		ret = PMEM_START_ADDR(data->index);
	else
		ret = PMEM_START_ADDR(data->index) + data->restrict_start;
	if (!have_pmem_data_sem_write)
		up_read(&pmem_data_sem);
	return ret;
}

static unsigned long pmem_len(struct pmem_data *data)
{
	unsigned long ret;

	if (!have_pmem_data_sem_write)
		down_read(&pmem_data_sem);
	if (!PMEM_SUBALLOC(data))
		ret = PMEM_LEN(data->index);
	else
		ret = data->restrict_len;
	if (!have_pmem_data_sem_write)
		up_read(&pmem_data_sem);
	return ret;
}


static int pmem_remap_pfn_range(struct vm_area_struct *vma,
				unsigned long start,
				unsigned long size,
				unsigned unmap)
{
	/* hold the mm semp for the vma you are modifying when you call this */
	int i;
	unsigned long vma_size = vma->vm_end - vma->vm_start;
	unsigned long garbage_pages = 0;

	BUG_ON(!PMEM_IS_PAGE_ALIGNED(vma_size));
	size = min(vma_size, size);
	garbage_pages = (vma_size - size) >> PAGE_SHIFT;
	if (unmap)
		zap_page_range(vma, vma->vm_start, vma->vm_end - vma->vm_start,
				NULL);
	if (size)
		if (remap_pfn_range(vma, vma->vm_start, start >> PAGE_SHIFT,
				    size, vma->vm_page_prot))
			return -EAGAIN;
	for (i = 0; i < garbage_pages; i++)
		if (vm_insert_pfn(vma, vma->vm_start + size + (i * PAGE_SIZE),
					garbage_pfn))
			return -EAGAIN;
	return 0;
}

static int pmem_remap_file(struct file *file, unsigned long start,
			   unsigned long size)
{
	/* walk the list of vmas, zap_page_range in them */
	struct pmem_data *data = file->private_data;
	struct pmem_vma *pmem_vma;
	struct vm_area_struct *vma;
	struct list_head *elt;

	DOWN_WRITE_PMEM_DATA_SEM;
	//down_write(&pmem_data_sem);
	list_for_each(elt, &data->vma_list) {
		pmem_vma = list_entry(elt, struct pmem_vma, list);
		vma = pmem_vma->vma;
		down_write(&vma->vm_mm->mmap_sem);
		/* unmap the pages and remap the garbage page */
		pmem_remap_pfn_range(vma, start, size, 1);
		up_write(&vma->vm_mm->mmap_sem);
	}
	UP_WRITE_PMEM_DATA_SEM;
	//up_write(&pmem_data_sem);
	return 0;
}

#if 0
static int pmem_fault(struct vm_area_struct *vma, struct vm_fault *vmf)
{
	struct pmem_data *data;
	int ret = 0;

	if (!is_pmem_file(vma->vm_file) || !has_allocation(vma->vm_file))
		return VM_FAULT_SIGBUS;

	printk("pmem: process %d attempting to access revoked memory!\n",
			current->pid);
	/* check that the request is inside the mmaped region or it should 
	 * actually fault */
	down_read(&pmem_data_sem);
	data = (struct pmem_data*)vma->vm_file->private_data;
	if (vmf->pgoff > PMEM_START_ADDR(data->index) << PAGE_SHIFT &&
			vmf->pgoff < PMEM_END_ADDR(data->index) << PAGE_SHIFT) {
		ret = VM_FAULT_SIGBUS;
		goto end;
	}

	get_page(garbage_page);
	vmf->page = garbage_page;
end:
	up_read(&pmem_data_sem);
	return ret;
}
#endif

static void pmem_vma_open(struct vm_area_struct *vma)
{
	struct file* file = vma->vm_file;
	struct pmem_data *data = file->private_data;
	struct pmem_vma *pmem_vma;

	down_write(&pmem_vma_sem);
	BUG_ON(!has_allocation(file));

	pmem_vma = kmalloc(sizeof(struct pmem_vma), GFP_KERNEL);
	pmem_vma->vma = vma;
#if PMEM_DEBUG
	pmem_vma->pid = current->pid;
#endif

	list_add(&pmem_vma->list, &data->vma_list);
	up_write(&pmem_vma_sem);
}

static void pmem_vma_close(struct vm_area_struct *vma)
{
	struct file* file = vma->vm_file;
	struct pmem_data *data = file->private_data;
	struct pmem_vma *pmem_vma;
	struct list_head *elt, *elt2;

	down_write(&pmem_vma_sem);
	if (unlikely(!file || !is_pmem_file(file) || !data))
		goto end;
	BUG_ON(!has_allocation(file));
	BUG_ON(list_empty(&data->vma_list));
	list_for_each_safe(elt, elt2, &data->vma_list) {
		pmem_vma = list_entry(elt, struct pmem_vma, list);
		if (pmem_vma->vma == vma) {
			list_del(elt);
			kfree(pmem_vma);
			break;
		}
	}
end:
	up_write(&pmem_vma_sem);
}

static struct vm_operations_struct vm_ops = {
	.open = pmem_vma_open,
	.close = pmem_vma_close,
};

static int pmem_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct pmem_data *data;
	int index;
	unsigned long vma_size =  vma->vm_end - vma->vm_start;
	unsigned long map_start, map_size;
	int ret = 0;

	if (vma->vm_pgoff) {
#if PMEM_DEBUG
		printk("pmem: mmaps must be at zero offset.\n");
#endif
		return -EINVAL;
	}
	if (!PMEM_IS_PAGE_ALIGNED(vma_size)) {
#if PMEM_DEBUG
		printk("pmem: mmaps must be a multiple of pages_size.\n");
#endif
		return -EINVAL;
	}
	DOWN_WRITE_PMEM_DATA_SEM;
	//down_write(&pmem_data_sem);
	data = (struct pmem_data*)file->private_data;
	/* no new mappings on revoked files! */
	if (data && PMEM_REVOKED(data)) {
		ret = -EINVAL;
		goto error;
	}
	/* if file->private_data == unalloced, alloc*/
	if (data && data->index == -1) {
		down_write(&pmem_sem);
		index = pmem_allocate(vma->vm_end - vma->vm_start);
		up_write(&pmem_sem);
		if (index != -1)
			data->index = index;
	}
	/* either no space was available or an error occured */
	if (!has_allocation(file)) {
		ret = -EINVAL;
		goto error;
	}
	map_start = pmem_start_addr(data);
	map_size = pmem_len(data);
	/* request may be to map less than the total allocation */
	vma->vm_pgoff = map_start >> PAGE_SHIFT;
	vma->vm_page_prot = phys_mem_access_prot(file, vma->vm_page_prot);
	if (pmem_remap_pfn_range(vma, map_start, map_size, 0)) {
		ret = -EAGAIN;
		goto error;
	}
	pmem_vma_open(vma);
	vma->vm_ops = &vm_ops;
error:
	UP_WRITE_PMEM_DATA_SEM;
	//up_write(&pmem_data_sem);
	return ret;
}

/* the following are the api for accessing pmem regions by other drivers
 * from inside the kernel */
int get_pmem_file(unsigned long fd, unsigned long *start, unsigned long *len)
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
	*len = pmem_len(data);

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

void flush_pmem_file(unsigned long fd, unsigned long start, unsigned long len)
{
	struct pmem_vma *pvma;
	struct pmem_data *data;
	unsigned long flush_start, flush_end;
	struct list_head *elt;
	struct file *file = fget(fd);

	if (file == NULL || !is_pmem_file(file) || !has_allocation(file))
		return;

	data = (struct pmem_data *)file->private_data;

	list_for_each(elt, &data->vma_list) {
		pvma = list_entry(elt, struct pmem_vma, list);
		flush_start = start + pvma->vma->vm_start;
		flush_end = len + flush_start;
		flush_cache_user_range(pvma->vma, flush_start, flush_end);
	}
	fput(file);
}

static int pmem_suballocate(struct pmem_suballoc* suballoc, struct file *file)
{
	struct pmem_data* data = (struct pmem_data*)file->private_data;
	struct pmem_data* src_data;
	struct file* src_file;
	unsigned long src_len;
	unsigned is_mapped;
	int ret = 0;

	if (unlikely(!PMEM_IS_PAGE_ALIGNED(suballoc->addr.start) ||
		     !PMEM_IS_PAGE_ALIGNED(suballoc->addr.len))) {
#if PMEM_DEBUG
		printk("pmem: request for unaligned pmem suballocation\n");
#endif
		return -EINVAL;
	}

	down_read(&pmem_data_sem);
	/* retrieve the src file and check it is a pmem file with an alloc */
	src_file = fget(suballoc->fd);
	if (unlikely(!is_pmem_file(src_file) || !has_allocation(src_file))) {
		ret = -EINVAL;
		goto end;
	}
	src_data = (struct pmem_data*)src_file->private_data;
	is_mapped = data->index != -1;
	/* check old allocations map into the specified src file and are marked
	 * as suballocations */
	if (is_mapped && (data->index != src_data->index ||
			  !(data->flags & PMEM_FLAGS_SUBALLOC))) {
		ret = -EINVAL;
		goto end;
	}
	/* check that the requested range is within the src allocation */
	src_len = PMEM_LEN(src_data->index);
	if (unlikely((suballoc->addr.start > src_len) ||
		     (suballoc->addr.start + suballoc->addr.len > src_len))) {
		ret = -EINVAL;
		goto end;
	}
	data->index = src_data->index;
	data->restrict_start = suballoc->addr.start;
	data->restrict_len = suballoc->addr.len;
	data->flags |= PMEM_FLAGS_SUBALLOC;
	/* remap if it's mapped, otherwise let the mmap handle it */
	if (is_mapped)
		pmem_remap_file(file, pmem_start_addr(data),
			        data->restrict_len);
end:
	fput(src_file);
	up_read(&pmem_data_sem);
	return ret;
}


static void pmem_get_size(struct pmem_addr* addr, struct file *file)
{
	struct pmem_data* data = (struct pmem_data*)file->private_data;

	if (!has_allocation(file)) {
		addr->start = 0;
		addr->len = 0;
		return;
	}
	if (PMEM_SUBALLOC(data)) {
		addr->start = data->restrict_start;
		addr->len = data->restrict_len;
	} else {
		addr->start = 0;
		addr->len = PMEM_LEN(data->index);
	}
}

static long pmem_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct pmem_data* data;
	struct pmem_addr addr;

	switch(cmd) {
		case PMEM_GET_PHYS:
			if (!has_allocation(file)) {
				addr.start = 0;
				addr.len = 0;
			} else { 
				data = (struct pmem_data*)file->private_data;
				addr.start = pmem_start_addr(data);
				addr.len = pmem_len(data);
			}
			if (copy_to_user((void __user *)arg, &addr,
					  sizeof(struct pmem_addr)))
				return -EFAULT;
			break;
		case PMEM_SUBALLOCATE:
			{
			struct pmem_suballoc suballoc;
			if (copy_from_user(&suballoc, (void __user *)arg,
					   sizeof(struct pmem_suballoc)))
				return -EFAULT;
			return pmem_suballocate(&suballoc, file);
			}
			break;
		case PMEM_GET_SIZE:
			pmem_get_size(&addr, file);
			if (copy_to_user((void __user *)arg, &addr,
					  sizeof(struct pmem_addr)))
				return -EFAULT;
			break;
		case PMEM_REVOKE:
			if (!has_allocation(file))
				return 0;
			return pmem_remap_file(file, 0, 0);
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
	struct pmem_vma *vma;
	n = 0;

	n = scnprintf(buffer, DEBUG_BUFMAX,
		      "from\tto\t(mappers....)\n");
	down_read(&pmem_data_sem);
	down_read(&pmem_vma_sem);
	list_for_each(elt, &data_list) {
		data = list_entry(elt, struct pmem_data, list);
		if (!data->index)
			continue;
		n += scnprintf(buffer + n, DEBUG_BUFMAX - n,
			       "%lx\t%lx\t",
			       PMEM_START_ADDR(data->index),
			       PMEM_END_ADDR(data->index));
		list_for_each(elt2, &data->vma_list) {
			vma = list_entry(elt2, struct pmem_vma, list);
			n += scnprintf(buffer + n, DEBUG_BUFMAX - n,
			               "%u ", vma->pid);
		}
		n += scnprintf(buffer + n, DEBUG_BUFMAX - n, "\n");
	}
	up_read(&pmem_vma_sem);
	up_read(&pmem_data_sem);
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

static int pmem_probe(struct platform_device *pdev)
{
	int err = 0;
	int i, index = 0;
	struct android_pmem_platform_data *pdata;

	if (!pdev || !pdev->dev.platform_data) {
		printk(KERN_ALERT "Unable to probe pmem!\n");
		return -1;
	}
	pdata = pdev->dev.platform_data;

	
	err = misc_register(&pmem_dev);
	if (err) {
		printk(KERN_ALERT "Unable to register pmem driver!\n");
		return err;
	}
        pmem_base = pdata->start;
        pmem_size = pdata->size;
	pmem_num_entries = pmem_size / PMEM_MIN_ALLOC;

	pbits = kmalloc(pmem_num_entries * sizeof(struct pmem_bits), 
		GFP_KERNEL);
	if (!pbits) {
		err = -1;
		goto error;
	}
	memset(pbits, 0, sizeof(struct pmem_bits) * pmem_num_entries);


	for (i = sizeof(pmem_num_entries)*8 - 1; i >=0; i--) {
		if ((pmem_num_entries) &  1<<i) {
			PMEM_ORDER(index) = i;
			index = PMEM_NEXT_INDEX(index);
		}
	}

	pmem_vbase = ioremap(pmem_base, pmem_size);
	if (pmem_vbase == 0) {
		err = -1;
		goto error1;
	}
	garbage_page = alloc_page(GFP_KERNEL);
	if (!garbage_page) {
		goto error2;
	}
	garbage_pfn = PMEM_START_ADDR(pmem_allocate(PAGE_SIZE)) >> PAGE_SHIFT;

#if PMEM_DEBUG
	debugfs_create_file("pmem", S_IFREG | S_IRUGO, NULL, NULL, &debug_fops);
#endif
	return 0;
error2:
	__free_page(garbage_page);
error1:
	kfree(pbits);
error:
	misc_deregister(&pmem_dev);
	return err;
}

static int pmem_remove(struct platform_device *pdev)
{
	__free_page(garbage_page);
	misc_deregister(&pmem_dev);
	return 0;
}

static struct platform_driver pmem_driver = {
	.probe = pmem_probe,
	.remove = pmem_remove,
	.driver = { .name = "android_pmem" }
};


static int __init pmem_init(void) {
	return platform_driver_register(&pmem_driver);
}

static void __exit pmem_exit(void)
{
	platform_driver_unregister(&pmem_driver);
}

module_init(pmem_init);
module_exit(pmem_exit);
 
