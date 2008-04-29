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
#include <linux/mempolicy.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/cacheflush.h>
#include <asm/arch/board.h>

#define PMEM_MAX_DEVICES 10
#define PMEM_MAX_ORDER 128
#define PMEM_MIN_ALLOC PAGE_SIZE

#define PMEM_DEBUG 1

/* indicates that a refernce to this file has been taken via get_pmem_file,
 * the file should not be released until put_pmem_file is called */
#define PMEM_FLAGS_BUSY 0x1
/* indicates that this is a suballocation of a larger master range */
#define PMEM_FLAGS_SUBALLOC 0x1 << 1
/* indicates this is a master and not a sub allocation and that it is mmaped */
#define PMEM_FLAGS_MASTERMAP 0x1 << 2
/* submap and unsubmap flags indicate:
 * 00: subregion has never been mmaped
 * 10: subregion has been mmaped, reference to the mm was taken
 * 11: subretion has ben released, refernece to the mm still held
 * 01: subretion has been released, reference to the mm has been released
 */
#define PMEM_FLAGS_SUBMAP 0x1 << 3
#define PMEM_FLAGS_UNSUBMAP 0x1 << 4

struct pmem_data {
	/* in alloc mode: an index into the bitmap
	 * in no_alloc mode: the size of the allocation */
	int index;
	/* see flags above for descriptions */
	unsigned int flags;
	/* protects this data field, if the mm_mmap sem will be held at the
	 * same time as this sem, the mm sem must be taken first (as this is
	 * the order for vma_open and vma_close ops */
	struct rw_semaphore sem;
	/* info about the mmaping process */
	struct vm_area_struct *vma;
	/* task struct of the mapping process */
	struct task_struct *task;
	/* process id of teh mapping process */
	pid_t pid;
	/* file descriptor of the master */
	int master_fd;
	/* a list of currently available regions if this is a suballocation */
	struct list_head region_list;
#if PMEM_DEBUG
	/* a linked list of data so we can access them for debugging */
	struct list_head list;  
#endif
};

struct pmem_bits {
	unsigned allocated:1;		/* 1 if allocated, 0 if free */
	unsigned order:7;		/* size of the region in pmem space */
};

struct pmem_region_list {
	struct pmem_region region;
	struct list_head list;
};

#define PMEM_DEBUG_MSGS 0
#if PMEM_DEBUG_MSGS
#define DLOG(fmt,args...) \
	do { printk("[%s:%s:%d] "fmt, __FILE__, __func__, __LINE__,##args); }\
	while(0)
#else
#define DLOG(x...) do {} while(0)
#endif

struct pmem_info {
	struct miscdevice dev;
	/* physical start address of the remaped pmem space */
	unsigned long base;
	/* vitual start address of the remaped pmem space */
	unsigned char __iomem *vbase;
	/* total size of the pmem space */
	unsigned long size;
	/* number of entries in the pmem space */
	unsigned long num_entries;
	/* pfn of the garbage page in memory */
	unsigned long garbage_pfn;
	/* index of the garbage page in the pmem space */
	int garbage_index;
	/* the bitmap for the region indicating which entries are allocated
	 * and which are free */
	struct pmem_bits *bitmap;
	/* indicates the region should not be managed with an allocator */
	unsigned no_allocator;
	/* indicates maps of this region should be cached, if a mix of
	 * cached and uncached is desired, set this and open the device with
	 * O_SYNC to get an uncached region */
	unsigned cached;
	/* in no_allocator mode the first mapper gets the whole space and sets
	 * this flag */
	unsigned allocated;
#if PMEM_DEBUG
	/* for debugging, creates a list of pmem file structs, the
	 * data_list_sem should be taken before pmem_data->sem if both are
	 * needed */ 
	struct rw_semaphore data_list_sem;
	struct list_head data_list;
#endif
	/* pmem_sem protects the bitmap array
	 * a write lock should be held when modifying entries in bitmap
	 * a read lock should be held when reading data from bits or
	 * dereferencing a pointer into bitmap
	 *
	 * pmem_data->sem protects the pmem data of a particular file
	 * Many of the function that require the pmem_data->sem have a non-
	 * locking version for when the caller is already holding that sem.
	 *
	 * IF YOU TAKE BOTH LOCKS TAKE THEM IN THIS ORDER:
	 * down(pmem_data->sem) => down(bitmap_sem)
	 */
	struct rw_semaphore bitmap_sem;
};

static struct pmem_info pmem[PMEM_MAX_DEVICES];

#define PMEM_IS_FREE(id, index) !(pmem[id].bitmap[index].allocated)
#define PMEM_ORDER(id, index) pmem[id].bitmap[index].order
#define PMEM_BUDDY_INDEX(id, index) (index ^ (1 << PMEM_ORDER(id, index)))
#define PMEM_NEXT_INDEX(id, index) (index + (1 << PMEM_ORDER(id, index)))
#define PMEM_OFFSET(index) (index * PMEM_MIN_ALLOC)
#define PMEM_START_ADDR(id, index) (PMEM_OFFSET(index) + pmem[id].base)
#define PMEM_LEN(id, index) ((1 << PMEM_ORDER(id, index)) * PMEM_MIN_ALLOC)
#define PMEM_END_ADDR(id, index) (PMEM_START_ADDR(id, index) + \
	PMEM_LEN(id, index))
#define PMEM_START_VADDR(id, index) (PMEM_OFFSET(id, index) + pmem[id].vbase)
#define PMEM_END_VADDR(id, index) (PMEM_START_VADDR(id, index) + \
	PMEM_LEN(id, index))
#define PMEM_REVOKED(data) (data->flags & PMEM_FLAGS_REVOKED)
#define PMEM_IS_PAGE_ALIGNED(addr) (!((addr) & (~PAGE_MASK)))
#define PMEM_IS_SUBMAP(data) ((data->flags & PMEM_FLAGS_SUBMAP) && \
	(!(data->flags & PMEM_FLAGS_UNSUBMAP)))

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

static int get_id(struct file *file) {
	return MINOR(file->f_dentry->d_inode->i_rdev);
}

static int is_pmem_file (struct file *file)
{
	int id;

	if (unlikely(!file->f_dentry || !file->f_dentry->d_inode))
		return 0;
	if (unlikely((id = get_id(file)) >= PMEM_MAX_DEVICES))
		return 0;
	if (unlikely(file->f_dentry->d_inode->i_rdev !=
	     MKDEV(MISC_MAJOR, pmem[id].dev.minor)))
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
	if (unlikely(data->index < 0 /*|| data->index > pmem_num_entries*/))
		return 0;
	return 1;
}

static int pmem_free(int id, int index)
{
	/* caller should hold the write lock on pmem_sem! */
	int buddy, curr = index;
	DLOG("index %d\n", index);

	if (pmem[id].no_allocator) {
		pmem[id].allocated = 0;
		return 0;
	}
	/* clean up the bitmap, merging any buddies */
	pmem[id].bitmap[curr].allocated = 0;
	/* find a slots buddy Buddy# = Slot# ^ (1 << order)
	 * if the buddy is also free merge them
	 * repeat until the buddy is not free or end of the bitmap is reached
	 */
	do {
		buddy = PMEM_BUDDY_INDEX(id, curr);
		if (PMEM_IS_FREE(id, buddy) &&
				PMEM_ORDER(id, buddy) == PMEM_ORDER(id, curr)) {
			PMEM_ORDER(id, buddy)++;
			PMEM_ORDER(id, curr)++;
			curr = min(buddy, curr);
		} else {
			break;
		}
	} while (curr < pmem[id].num_entries);

	return 0;
}

static int pmem_release(struct inode* inode, struct file *file)
{
	struct pmem_data *data = (struct pmem_data *)file->private_data;
	struct pmem_region_list *region_list;
	struct list_head *elt, *elt2;
	int id = get_id(file), ret = 0;


#if PMEM_DEBUG
	down_write(&pmem[id].data_list_sem);
	list_del(&data->list);
	up_write(&pmem[id].data_list_sem);
#endif
	DLOG("file %p\n", file);
	down_write(&data->sem);

	if (!(PMEM_FLAGS_SUBALLOC & data->flags) && has_allocation(file)) {
		down_write(&pmem[id].bitmap_sem);
		ret = pmem_free(id, data->index);
		up_write(&pmem[id].bitmap_sem);
	}
	if (PMEM_FLAGS_SUBMAP & data->flags) {
		if (data->task) {
			put_task_struct(data->task);
			data->task = NULL;
		}
	}

	file->private_data = NULL;

	list_for_each_safe(elt, elt2, &data->region_list) {
		region_list = list_entry(elt, struct pmem_region_list, list);
		list_del(elt);
		kfree(region_list);
	}
	BUG_ON(!list_empty(&data->region_list));

	up_write(&data->sem);
	kfree(data);
	return ret;
}

static int pmem_open(struct inode *inode, struct file *file)
{
	struct pmem_data* data;
	int id = get_id(file);

	DLOG("current %u file %p(%d)\n", current->pid, file, file_count(file));
	/* setup file->private_data to indicate its unmapped */
	/*  you can only open a pmem device one time */
	if (file->private_data != NULL)
		return -1;
	data = kmalloc(sizeof(struct pmem_data), GFP_KERNEL);
	if (!data) {
		printk("pmem: unable to allocate memory for pmem metadata.");
		return -1;
	}
	data->flags = 0;
	data->index = -1;
	data->task = NULL;
	data->vma = NULL;
	data->pid = 0;
	INIT_LIST_HEAD(&data->region_list);
	init_rwsem(&data->sem);

	file->private_data = data;
#if PMEM_DEBUG
	INIT_LIST_HEAD(&data->list);
	down_write(&pmem[id].data_list_sem);
	list_add(&data->list, &pmem[id].data_list);
	up_write(&pmem[id].data_list_sem);
#endif
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

static int pmem_allocate(int id, unsigned long len)
{
	/* caller should hold the write lock on pmem_sem! */
	/* return the corresponding pdata[] entry */
	int curr = 0;
	int end = pmem[id].num_entries;
	int best_fit = -1;
	unsigned long order = pmem_order(len);

	if (pmem[id].no_allocator) {
		DLOG("no allocator");
		if ((len > pmem[id].size - PAGE_SIZE) || pmem[id].allocated)
			return -1;
		pmem[id].allocated = 1;
		return len;
	}

	if (order > PMEM_MAX_ORDER)
		return -1;
	DLOG("order %lx\n", order);

	/* look through the bitmap:
	 * 	if you find a free slot of the correct order use it
	 * 	otherwise, use the best fit (smallest with size > order) slot
	 */
	while (curr < end) {
		if (PMEM_IS_FREE(id, curr)) {
			if (PMEM_ORDER(id, curr) == (unsigned char)order) {
				/* set the not free bit and clear others */
				best_fit = curr;
				break;
			}
			if (PMEM_ORDER(id, curr) > (unsigned char)order &&
			    (best_fit < 0 ||
			     PMEM_ORDER(id, curr) < PMEM_ORDER(id, best_fit)))
				best_fit = curr;
		}
		curr = PMEM_NEXT_INDEX(id, curr);
	}

	/* if best_fit < 0, there are no suitable slots,
	 * return an error
	 */
	if (best_fit < 0) {
		printk("pmem: no space left to allocate!\n");
		return -1;
	}

	/* now partition the best fit:
	 * 	split the slot into 2 buddies of order - 1
	 * 	repeat until the slot is of the correct order
	 */
	while (PMEM_ORDER(id, best_fit) > (unsigned char)order) {
		int buddy;
		PMEM_ORDER(id, best_fit) -= 1;
		buddy = PMEM_BUDDY_INDEX(id, best_fit);
		PMEM_ORDER(id, buddy) = PMEM_ORDER(id, best_fit);
	}
	pmem[id].bitmap[best_fit].allocated = 1;
	return best_fit;
}

static pgprot_t phys_mem_access_prot(struct file *file, pgprot_t vma_prot)
{
#ifdef pgprot_noncached
	if (!pmem[get_id(file)].cached || file->f_flags & O_SYNC)
		return pgprot_noncached(vma_prot);
#endif
	return vma_prot;
}

static unsigned long pmem_start_addr(int id, struct pmem_data *data)
{
	if (pmem[id].no_allocator)
		return PMEM_START_ADDR(id, 0);
	else
		return PMEM_START_ADDR(id, data->index);

}

static void *pmem_start_vaddr(int id, struct pmem_data *data)
{
	return pmem_start_addr(id, data) - pmem[id].base + pmem[id].vbase;
}

static unsigned long pmem_len(int id, struct pmem_data *data)
{
	if (pmem[id].no_allocator)
		return data->index;
	else
		return PMEM_LEN(id, data->index);
}

static int pmem_unmap_pfn_range(int id, struct vm_area_struct *vma,
				struct pmem_data *data, unsigned long offset,
				unsigned long len)
{
	int i, garbage_pages;
	DLOG("unmap offset %lx len %lx\n", offset, len);

	BUG_ON(!PMEM_IS_PAGE_ALIGNED(len));

	garbage_pages = len >> PAGE_SHIFT;
	zap_page_range(vma, vma->vm_start + offset, len, NULL);
	for (i = 0; i < garbage_pages; i++) {
		vma->vm_flags |= VM_IO | VM_RESERVED | VM_PFNMAP | VM_SHARED | 					 VM_WRITE;
		if (vm_insert_pfn(vma, vma->vm_start + offset + (i * PAGE_SIZE),
		    pmem[id].garbage_pfn))
			return -EAGAIN;
	}
	return 0;
}

static int pmem_map_pfn_range(int id, struct vm_area_struct *vma,
			      struct pmem_data *data, unsigned long offset,
			      unsigned long len)
{
	/* hold the mm semp for the vma you are modifying when you call this */

	DLOG("map offset %lx len %lx\n", offset, len);
	BUG_ON(!PMEM_IS_PAGE_ALIGNED(vma->vm_start));
	BUG_ON(!PMEM_IS_PAGE_ALIGNED(vma->vm_end));
	BUG_ON(!PMEM_IS_PAGE_ALIGNED(len));
	BUG_ON(!PMEM_IS_PAGE_ALIGNED(offset));

	zap_page_range(vma, vma->vm_start + offset, len, NULL);
	if (remap_pfn_range(vma, vma->vm_start + offset,
			    (pmem_start_addr(id, data) + offset) >> PAGE_SHIFT,
			    len, vma->vm_page_prot)) {
		DLOG("not ok!");
		return -EAGAIN;
	}
	return 0;
}

static void pmem_vma_open(struct vm_area_struct *vma)
{
	struct file* file = vma->vm_file;
	struct pmem_data *data = file->private_data;
	int id = get_id(file);
	/* this should never be called as we don't support copying pmem
	 * ranges via fork */
	DLOG("file %p\n", file);
	BUG_ON(!has_allocation(file));
	down_write(&data->sem);
	/* remap the garbage pages, forkers don't get access to the data */
	pmem_unmap_pfn_range(id, vma, data, 0, vma->vm_end - vma->vm_end);
	up_write(&data->sem);
}

static void pmem_vma_close(struct vm_area_struct *vma)
{
	struct file* file = vma->vm_file;
	struct pmem_data *data = file->private_data;

	DLOG("current %u ppid %u file %p count %d\n", current->pid,
	     current->parent->pid, file, file_count(file));
	if (unlikely(!is_pmem_file(file) || !has_allocation(file))) {
		printk("pmem: something is very wrong, you are closing a"
			"vm backing an allocation that doesn't exist!\n");
		return;
	}
	down_write(&data->sem);
	if ((data->flags & PMEM_FLAGS_SUBALLOC) &&
	    (data->flags & PMEM_FLAGS_SUBMAP) &&
	     data->vma == vma) {
		data->flags |= PMEM_FLAGS_UNSUBMAP;
	}
	up_write(&data->sem);
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
	int ret = 0, id = get_id(file);

	if (vma->vm_pgoff || !PMEM_IS_PAGE_ALIGNED(vma_size)) {
#if PMEM_DEBUG
		printk(KERN_ERR "pmem: mmaps must be at offset zero, aligned"
				" and a multiple of pages_size.\n");
#endif
		return -EINVAL;
	}

	data = (struct pmem_data*)file->private_data;
	down_write(&data->sem);
	/* check this file isn't already mmaped, for submaps check this file
	 * has never been mmaped */
	if ((data->flags & PMEM_FLAGS_MASTERMAP) ||
	    (data->flags & PMEM_FLAGS_SUBMAP) ||
	    (data->flags & PMEM_FLAGS_UNSUBMAP)) {
#if PMEM_DEBUG
		printk(KERN_ERR "pmem: you can only mmap a pmem file once, "
		       "this file is already mmaped. %x\n", data->flags);
#endif
		ret = -EINVAL;
		goto error;
	}
	/* if file->private_data == unalloced, alloc*/
	if (data && data->index == -1) {
		down_write(&pmem[id].bitmap_sem);
		index = pmem_allocate(id, vma->vm_end - vma->vm_start);
		up_write(&pmem[id].bitmap_sem);
		data->index = index;
	}
	/* either no space was available or an error occured */
	if (!has_allocation(file)) {
		ret = -EINVAL;
		printk("pmem: could not find allocation for map.\n");
		goto error;
	}

	if (pmem_len(id, data) < vma_size) {
#if PMEM_DEBUG
		printk("pmem: mmap size [%lu] does not match size of backing "
			"region [%lu]!\n", vma_size, pmem_len(id, data));
#endif
		ret = -EINVAL;
		goto error;
	}

	vma->vm_pgoff = pmem_start_addr(id, data) >> PAGE_SHIFT;
	vma->vm_page_prot = phys_mem_access_prot(file, vma->vm_page_prot);

	if (data->flags & PMEM_FLAGS_SUBALLOC) {
		struct pmem_region_list *region_list;
		struct list_head *elt;
		if (pmem_unmap_pfn_range(id, vma, data, 0, vma_size)) {
			printk("pmem: mmap failed in kernel!\n");
			ret = -EAGAIN;
			goto error;
		}
		list_for_each(elt, &data->region_list) {
			region_list = list_entry(elt, struct pmem_region_list,
						 list);
			DLOG("remapping file: %p %lx %lx\n", file,
				region_list->region.offset,
				region_list->region.len);
			if (pmem_map_pfn_range(id, vma, data,
					       region_list->region.offset,
					       region_list->region.len)) {
				ret = -EAGAIN;
				goto error;
			}
		}
		data->flags |= PMEM_FLAGS_SUBMAP;
		get_task_struct(current);
		data->task = current;
		data->vma = vma;
#if PMEM_DEBUG
		data->pid = current->pid;
#endif
		DLOG("submmapped file %p vma %p pid %u\n", file, vma, current->pid);
	} else {
		if (pmem_map_pfn_range(id, vma, data, 0, vma_size)) {
			printk("pmem: mmap failed in kernel!\n");
			ret = -EAGAIN;
			goto error;
		}
		data->flags |= PMEM_FLAGS_MASTERMAP;
		data->pid = current->pid;
#if PMEM_DEBUG
		DLOG("mmapped file %p(%d) vma %p pid %u\n", file, file_count(file), vma, current->pid);
#endif
	}
	vma->vm_ops = &vm_ops;
error:
	up_write(&data->sem);
	return ret;
}

/* the following are the api for accessing pmem regions by other drivers
 * from inside the kernel */
int get_pmem_file(unsigned int fd, unsigned long *start, unsigned long *len,
		  struct file **filp)
{
	struct file *file;
	struct pmem_data *data;
	int id;

	file = fget(fd);
	if (unlikely(file == NULL)) {
#if PMEM_DEBUG
		printk("pmem: lookup fd=%d failed, file not found in fd table.\n", fd);
#endif
		return -1;
	}

	if(!is_pmem_file(file) || !has_allocation(file)) {
#if PMEM_DEBUG
		printk("pmem: requested pmem data from invalid file.\n");
#endif
		goto end;
	}

	data = (struct pmem_data*)file->private_data;
	if (data->index == -1) {
#if PMEM_DEBUG
		printk("pmem: requested pmem data from file with no allocation.\n");
		goto end;
#endif
	}
	id = get_id(file);

	down_read(&data->sem);
	*start = pmem_start_addr(id, data);
	*len = pmem_len(id, data);
	up_read(&data->sem);
	
	if (filp)
		*filp = file;

	if (filp != NULL)
		*filp = file;

	return 0;
end:
	fput(file);
	return -1;
}

int get_pmem_fd(unsigned int fd, unsigned long *start, unsigned long *len)
{
	return get_pmem_file(fd, start, len, NULL);
}

void put_pmem_file(struct file* file)
{
        struct pmem_data *data;
        int id;

	if (!is_pmem_file(file))
		return;
	id = get_id(file);
	data = (struct pmem_data *)file->private_data;
	fput(file);
}

void put_pmem_fd(unsigned int fd)
{
	struct file *file;
	int put_needed;

	file = fget_light(fd, &put_needed);
	if (file == NULL)
		return;
	put_pmem_file(file);
	fput_light(file, put_needed);
}

void flush_pmem_fd(unsigned int fd, unsigned long offset, unsigned long len)
{
	struct pmem_data *data;
	struct file *file;
	int id;
	void *vaddr;
	struct pmem_region_list *region_list;
	struct list_head *elt;
	void *flush_start, *flush_end;
	int fput_needed;

	file = fget_light(fd, &fput_needed);
	if (file == NULL)
		return;

	if (!is_pmem_file(file) || !has_allocation(file)) {
		fput_light(file, fput_needed);
		return;
	}

	id = get_id(file);
	data = (struct pmem_data *)file->private_data;
	fput_light(file, fput_needed);

	down_read(&data->sem);
	vaddr = pmem_start_vaddr(id, data);
	/* if this isn't a submmapped file, flush the whole thing */
	if (unlikely(!(data->flags & PMEM_FLAGS_SUBALLOC))) {
		dmac_flush_range(vaddr, vaddr + pmem_len(id, data));
		goto end;
	}
	/* otherwise, flush the region of the file we are drawing */
	list_for_each(elt, &data->region_list) {
		region_list = list_entry(elt, struct pmem_region_list, list);
		if ((offset >= region_list->region.offset) &&
		    ((offset + len) <= (region_list->region.offset +
			region_list->region.len))) {
			flush_start = vaddr + region_list->region.offset;
			flush_end = flush_start + region_list->region.len;
			dmac_flush_range(flush_start, flush_end);
			break;
		}
	}
end:
	up_read(&data->sem);
}

static int pmem_connect(unsigned long connect, struct file *file)
{
	struct pmem_data *data = (struct pmem_data*)file->private_data;
	struct pmem_data* src_data;
	struct file* src_file;
	int ret = 0, put_needed;

	down_write(&data->sem);
	/* retrieve the src file and check it is a pmem file with an alloc */
	src_file = fget_light(connect, &put_needed);
	DLOG("connect %p to %p\n", file, src_file);
	if (!src_file) {
		printk("pmem: src file not found!\n");
		ret = -EINVAL;
		goto end2;
	}
	if (unlikely(!is_pmem_file(src_file) || !has_allocation(src_file))) {
		printk("pmem: src file is not a pmem file or has no alloc!\n");
		ret = -EINVAL;
		goto end;
	}
	src_data = (struct pmem_data*)src_file->private_data;

	if (has_allocation(file) && (data->index != src_data->index)) {
		printk("pmem: file is already mapped but doesn't match this"
		       " src_file!\n");
		ret = -EINVAL;
		goto end;
	}
	data->index = src_data->index;
	data->flags |= PMEM_FLAGS_SUBALLOC;
	data->master_fd = connect;
end:
	fput_light(src_file, put_needed);
end2:
	up_write(&data->sem);
	return ret;
}

static int pmem_remap(struct pmem_region *region, struct file *file,
		      unsigned operation)
{
	struct pmem_data* data = (struct pmem_data*)file->private_data;
	struct pmem_region_list *region_list;
	struct list_head *elt, *elt2;
	struct file *master_file;
	struct mm_struct *mm = 0;
	int ret = 0, id = get_id(file), is_submmapped = 0, fput;

	if (unlikely(!PMEM_IS_PAGE_ALIGNED(region->offset) ||
		     !PMEM_IS_PAGE_ALIGNED(region->len))) {
#if PMEM_DEBUG
		printk("pmem: request for unaligned pmem suballocation "
		       "%lx %lx\n", region->offset, region->len);
#endif
		return -EINVAL;
	}

lock_mm:
	down_read(&data->sem);
	if (PMEM_IS_SUBMAP(data)) {
		is_submmapped = 1;
		mm = get_task_mm(data->task);
		if (!mm) {
			is_submmapped = 0;
			ret = -EINVAL;
			goto end2;
		}
	}
	up_read(&data->sem);

	if (is_submmapped)
		down_write(&mm->mmap_sem);

	down_write(&data->sem);
	master_file = fget_light(data->master_fd, &fput);
	if (unlikely(!master_file)) {
		ret = -EINVAL;
		goto end;
	}
	fput_light(master_file, fput);
	/* check that the file didn't get mmaped before we could take the
	 * data sem, this should be safe b/c you can only submap each file
	 * once */
	if (PMEM_IS_SUBMAP(data) && !is_submmapped) {
		up_write(&data->sem);
		goto lock_mm;
	}
	/* now check that vma.mm is still there, it could have been
	 * deleted by vma_close before we could get the data->sem */
	if ((data->flags & PMEM_FLAGS_UNSUBMAP) && is_submmapped) {
		is_submmapped = 0;
		/* might as well release this */
		up_write(&mm->mmap_sem);
		mmput(mm);
		if (data->flags & PMEM_FLAGS_SUBMAP) {
			put_task_struct(data->task);
			data->task = NULL;
			/* lower the submap flag to show the mm is gone */
			data->flags &= ~(PMEM_FLAGS_SUBMAP);
		}
	}
	if (region->len == 0)
		goto end;

	/* check that the requested range is within the src allocation */
	if (unlikely((region->offset > pmem_len(id, data)) ||
		     (region->offset + region->len > pmem_len(id, data)))) {
#if PMEM_DEBUG
		printk("pmem: suballocation doesn't fit in src_file!\n");
#endif
		ret = -EINVAL;
		goto end;
	}

	if (is_submmapped) {
		if (operation == PMEM_MAP)
			ret = pmem_map_pfn_range(id, data->vma, data,
			      region->offset, region->len);
		else if (operation == PMEM_UNMAP)
			ret = pmem_unmap_pfn_range(id, data->vma, data,
						   region->offset, region->len);
	}

	if (operation == PMEM_MAP) {
		region_list = kmalloc(sizeof(struct pmem_region_list),
			      GFP_KERNEL);
		if (!region_list) {
			ret= -EINVAL;
#if PMEM_DEBUG
			printk("No space to allocate metadata!");
#endif
			goto end;
		}
		region_list->region = *region;
		list_add(&region_list->list, &data->region_list);
	} else if (operation == PMEM_UNMAP) {
		int found = 0;
		list_for_each_safe(elt, elt2, &data->region_list) {
			region_list = list_entry(elt, struct pmem_region_list,
				      list);
			if (region->len == 0 ||
			    (region_list->region.offset == region->offset &&
			    region_list->region.len == region->len)) {
				list_del(elt);
				kfree(region_list);
				found = 1;
			}
		}
		if (!found) {
#if PMEM_DEBUG
			printk("pmem: Unmap region does not map any mapped "
				"region!");
#endif
			ret = -EINVAL;
			goto end;
		}
	}
end:
	up_write(&data->sem);
end2:
	if (is_submmapped) {
		up_write(&mm->mmap_sem);
		mmput(mm);
	}
	return ret;
}


static void pmem_get_size(struct pmem_region *region, struct file *file)
{
	struct pmem_data* data = (struct pmem_data*)file->private_data;
	int id = get_id(file);

	if (!has_allocation(file)) {
		region->offset = 0;
		region->len = 0;
		return;
	} else {
		region->offset = pmem_start_addr(id, data);
		region->len = pmem_len(id, data);
	}
	DLOG("offset %lx len %lx\n", region->offset, region->len);
}

static long pmem_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct pmem_data* data;
	int id = get_id(file);

	switch(cmd) {
		case PMEM_GET_PHYS:
			{
			struct pmem_region region;
			DLOG("get_phys\n");
			if (!has_allocation(file)) {
				region.offset = 0;
				region.len = 0;
			} else {
				data = (struct pmem_data*)file->private_data;
				region.offset = pmem_start_addr(id, data);
				region.len = pmem_len(id, data);
			}
			if (copy_to_user((void __user *)arg, &region,
					  sizeof(struct pmem_region)))
				return -EFAULT;
			break;
			}
		case PMEM_MAP:
			{
			struct pmem_region region;
			if (copy_from_user(&region, (void __user *)arg,
					   sizeof(struct pmem_region)))
				return -EFAULT;
			return pmem_remap(&region, file, PMEM_MAP);
			}
			break;
		case PMEM_UNMAP:
			{
			struct pmem_region region;
			if (copy_from_user(&region, (void __user *)arg,
					   sizeof(struct pmem_region)))
				return -EFAULT;
			return pmem_remap(&region, file, PMEM_UNMAP);
			break;
			}
		case PMEM_GET_SIZE:
			{
			struct pmem_region region;
			DLOG("get_size\n");
			pmem_get_size(&region, file);
			if (copy_to_user((void __user *)arg, &region,
					  sizeof(struct pmem_region)))
				return -EFAULT;
			break;
			}
		case PMEM_GET_TOTAL_SIZE:
			{
			struct pmem_region region;
			DLOG("get total size\n");
			region.offset = 0;
			get_id(file);
			region.len = pmem[id].size;
			if (copy_to_user((void __user *)arg, &region,
					  sizeof(struct pmem_region)))
				return -EFAULT;
			break;
			}
		case PMEM_ALLOCATE:
			{
			if (has_allocation(file))
				return -EINVAL;
			data = (struct pmem_data*)file->private_data;
			data->index = pmem_allocate(id, arg);
			break;
			}
		case PMEM_CONNECT:
			DLOG("connect\n");
			return pmem_connect(arg, file);
			break;
		default:
			return -EINVAL;
	}
	return 0;
}

#if PMEM_DEBUG
static ssize_t debug_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t debug_read(struct file *file, char __user *buf, size_t count,
			  loff_t *ppos)
{
	struct list_head *elt, *elt2;
	struct pmem_data *data;
	struct pmem_region_list *region_list;
	int id = (int)file->private_data;
	const int debug_bufmax = 4096;
	static char buffer[4096];
	int n = 0;

	DLOG("debug open\n");
	n = scnprintf(buffer, debug_bufmax,
		      "pid #: mapped regions (offset, len) (offset,len)...\n");

	down_write(&pmem[id].data_list_sem);
	list_for_each(elt, &pmem[id].data_list) {
		data = list_entry(elt, struct pmem_data, list);
		down_read(&data->sem);
		n += scnprintf(buffer + n, debug_bufmax - n, "pid %u:",
				data->pid);
		list_for_each(elt2, &data->region_list) {
			region_list = list_entry(elt2, struct pmem_region_list,
				      list);
			n += scnprintf(buffer + n, debug_bufmax - n,
					"(%lx,%lx) ",
					region_list->region.offset,
					region_list->region.len);
		}
		n += scnprintf(buffer + n, debug_bufmax - n, "\n");
		up_read(&data->sem);
	}
	up_write(&pmem[id].data_list_sem);

	n++;
	buffer[n] = 0;
	return simple_read_from_buffer(buf, count, ppos, buffer, n);
}

static struct file_operations debug_fops = {
	.read = debug_read,
	.open = debug_open,
};
#endif

#if 0
static struct miscdevice pmem_dev = {
	.name = "pmem",
	.fops = &pmem_fops,
};
#endif

static int pmem_probe(struct platform_device *pdev)
{
	int err = 0;
	int i, index = 0;
	struct android_pmem_platform_data *pdata;
	int id;

	if (!pdev || !pdev->dev.platform_data) {
		printk(KERN_ALERT "Unable to probe pmem!\n");
		return -1;
	}

	id = pdev->id;
	pdata = pdev->dev.platform_data;
	pmem[id].no_allocator = pdata->no_allocator;
	pmem[id].cached = pdata->cached;
	pmem[id].base = pdata->start;
	pmem[id].size = pdata->size;
	init_rwsem(&pmem[id].bitmap_sem);
#if PMEM_DEBUG
	init_rwsem(&pmem[id].data_list_sem);
	INIT_LIST_HEAD(&pmem[id].data_list);
#endif
	pmem[id].dev.name = pdata->name;
	pmem[id].dev.minor = id;
	pmem[id].dev.fops = &pmem_fops;
	printk(KERN_INFO "%s: init\n", pdata->name);

	err = misc_register(&pmem[id].dev);
	if (err) {
		printk(KERN_ALERT "Unable to register pmem driver!\n");
		return err;
	}
	pmem[id].num_entries = pmem[id].size / PMEM_MIN_ALLOC;

	pmem[id].bitmap = kmalloc(pmem[id].num_entries *
				  sizeof(struct pmem_bits), GFP_KERNEL);
	if (!pmem[id].bitmap) {
		err = -1;
		goto error;
	}
	memset(pmem[id].bitmap, 0, sizeof(struct pmem_bits) *
					  pmem[id].num_entries);

	for (i = sizeof(pmem[id].num_entries)*8 - 1; i >=0; i--) {
		if ((pmem[id].num_entries) &  1<<i) {
			PMEM_ORDER(id, index) = i;
			index = PMEM_NEXT_INDEX(id, index);
		}
	}

	pmem[id].vbase = ioremap_cached(pmem[id].base, pmem[id].size);
	if (pmem[id].vbase == 0) {
		err = -1;
		goto error1;
	}

	pmem[id].garbage_pfn = page_to_pfn(alloc_page(GFP_KERNEL));

#if PMEM_DEBUG
	debugfs_create_file(pdata->name, S_IFREG | S_IRUGO, NULL, (void*)id,
			&debug_fops);
#endif
	return 0;
error1:
	kfree(pmem[id].bitmap);
error:
	misc_deregister(&pmem[id].dev);
	return err;
}

static int pmem_remove(struct platform_device *pdev)
{
	int id = pdev->id;
	__free_page(pfn_to_page(pmem[id].garbage_pfn));
	misc_deregister(&pmem[id].dev);
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
 
