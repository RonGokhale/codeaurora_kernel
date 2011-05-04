/*
 *  ChromeOS platform support code. Glue layer between higher level functions
 *  and per-platform firmware interfaces.
 *
 *  Copyright (C) 2011 The Chromium OS Authors
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/types.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/spinlock.h>
#include <linux/mutex.h>
#include <linux/nvram.h>
#include <linux/seq_file.h>
#include <linux/miscdevice.h>
#include <linux/blkdev.h>
#include <linux/bio.h>
#include <asm/uaccess.h>

#define MY_LOGPREFIX "cros_nvram: "
#define MY_ERR KERN_ERR MY_LOGPREFIX
#define MY_NOTICE KERN_NOTICE MY_LOGPREFIX
#define MY_INFO KERN_INFO MY_LOGPREFIX

#define BLKNV_MAJOR		MMC_BLOCK_MAJOR
#define BLKNV_MINOR		0
#define BLKNV_READ		0
#define BLKNV_WRITE		1
#define SECTOR_SIZE		512

#define NVRAM_WRITE		1
#define NVRAM_EXCL		2

static DEFINE_MUTEX(nvram_mutex);
static DEFINE_SPINLOCK(nvram_state_lock);
static unsigned char *nvram_cache = NULL;
static unsigned char *nvram_cache_dirty = NULL;
static int nvram_open_cnt = 0;
static int nvram_open_mode = 0;

static int is_nvram_pre_init    = 0;
static int is_nvram_module_init = 0;
static int is_nvram_post_init   = 0;

extern sector_t get_blknv_sector(void);
extern void *get_firmware_nvram_addr(void);
extern unsigned get_firmware_nvram_size(void);
extern int cros_acpi_init(void);
extern void cros_acpi_release(void);

/* nvram on block device */
static void blknv_endio(struct bio *bio, int err)
{
	complete(bio->bi_private);
	bio->bi_private = (void *)err;
}

static void blknv_submit_bio(struct bio *bio, int rq)
{
	DECLARE_COMPLETION_ONSTACK(wait);

	bio->bi_end_io	= blknv_endio;
	bio->bi_private = &wait;
	submit_bio(rq, bio);
	wait_for_completion(&wait);
}

static int blknv_readwrite_sector(unsigned char *buf, int is_write)
{
	struct block_device *bdev	= NULL;
	struct bio *bio			= NULL;
	struct page *page		= NULL;
	dev_t mdev;
	fmode_t devmode;
	int rq, ret;
	sector_t sector;
	char holder[0];

	/* get sector offset from firmware */
	sector = get_blknv_sector();
	if (sector == (sector_t)-1) {
		printk(MY_ERR "could not get lba offset\n");
		ret = -EFAULT;
		goto out;
	}
	mdev = MKDEV(BLKNV_MAJOR, BLKNV_MINOR);
	devmode = is_write ? FMODE_WRITE : FMODE_READ;
	bdev = blkdev_get_by_dev(mdev, devmode, holder);
	if (IS_ERR(bdev)) {
		printk(MY_ERR "could not open mmcblk0 dev=[%d:0]\n",
			MMC_BLOCK_MAJOR);
		ret = -EFAULT;
		goto out;
	}
	/* map the sector to page */
	bio = bio_alloc(GFP_NOIO, 1);
	if (!bio) {
		ret = -ENOMEM;
		goto out;
	}
	page = alloc_page(GFP_NOIO);
	if (!page) {
		ret = -ENOMEM;
		goto out;
	}
	bio->bi_bdev	= bdev;
	bio->bi_sector	= get_blknv_sector();
	bio->bi_vcnt	= 1;
	bio->bi_idx	= 0;
	bio->bi_size	= SECTOR_SIZE;
	bio->bi_io_vec[0].bv_page	= page;
	bio->bi_io_vec[0].bv_len	= SECTOR_SIZE;
	bio->bi_io_vec[0].bv_offset	= 0;
	/* submit bio */
	rq = REQ_SYNC | REQ_SOFTBARRIER | REQ_UNPLUG | REQ_NOIDLE;
	if (is_write) {
		rq |= REQ_WRITE;
		memcpy(page_address(page), buf, SECTOR_SIZE);
	}
	blknv_submit_bio(bio, rq);
	if (bio->bi_private) {
		ret = (int)bio->bi_private;
		goto out;
	}
	if (!is_write) {
		memcpy(buf, page_address(page), SECTOR_SIZE);
	}
	ret = 0;
out:
	if (page)
		__free_page(page);
	if (bio)
		bio_put(bio);
	if (bdev)
		blkdev_put(bdev, devmode);
	return ret;
}

/* nvram api */

unsigned char nvram_read_byte(int i)
{
	unsigned char sector[SECTOR_SIZE];
	if (is_nvram_post_init) {
		/* after post init
		 * nvram will be mapped to block device
		 */
		if (i >= SECTOR_SIZE)
			return 0;
		memset(sector, 0, SECTOR_SIZE);
		if (blknv_readwrite_sector(sector, BLKNV_READ))
			return 0;
		return sector[i];
	}

	if (!is_nvram_pre_init) {
		/* no access before pre_init */
		printk(MY_ERR "nvread - before pre init\n");
		return 0;
	}

	if (i < SECTOR_SIZE) {
		return nvram_cache[i];
	}
	printk(MY_ERR "nvread - out of range\n");
	return 0;
}
EXPORT_SYMBOL(nvram_read_byte);

void nvram_write_byte(unsigned char c, int i)
{
	unsigned char sector[SECTOR_SIZE];
	if (is_nvram_post_init) {
		/* after post init
		 * nvram will be mapped to block device
		 */
		if (i >= SECTOR_SIZE)
			return;
		memset(sector, 0, SECTOR_SIZE);
		if (blknv_readwrite_sector(sector, BLKNV_READ))
			return;
		if (sector[i] != c) {
			sector[i] = c;
			blknv_readwrite_sector(sector, BLKNV_WRITE);
		}
		return;
	}

	if (!is_nvram_pre_init) {
		/* no access before pre_init */
		printk(MY_ERR "nvwrite - before pre init\n");
		return;
	}

	if (i < SECTOR_SIZE){
		nvram_cache[i] = c;
		nvram_cache_dirty[i] = 1;
		return;
	}
	printk(MY_ERR "nvwrite - out of range\n");
}
EXPORT_SYMBOL(nvram_write_byte);

/* File ops for /dev/nvram */
static loff_t nvram_llseek(struct file *file, loff_t offset, int origin)
{
	switch(origin) {
	case 0:
		break;
	case 1:
		offset += file->f_pos;
		break;
	case 2:
		offset += SECTOR_SIZE;
		break;
	}

	return (offset >= 0) ? (file->f_pos = offset) : -EINVAL;
}

static ssize_t nvram_read(struct file *file, char __user *buf,
						size_t count, loff_t *ppos)
{
	unsigned char sector[SECTOR_SIZE];
	unsigned i = *ppos;

	memset(sector, 0, SECTOR_SIZE);
	if(blknv_readwrite_sector(sector, BLKNV_READ))
		return -EFAULT;

	if (i >= SECTOR_SIZE)
		return 0;
	if ((i + count) > SECTOR_SIZE)
		count = SECTOR_SIZE - i;
	if (copy_to_user(buf, sector + i, count))
		return -EFAULT;

	i += count;
	*ppos = i;
	return count;
}

static ssize_t nvram_write(struct file *file, const char __user *buf,
						size_t count, loff_t *ppos)
{
	unsigned char sector[SECTOR_SIZE];
	const char __user *p = buf;
	int dirty = 0;
	unsigned int i;
	int ret;
	char c;

	if (!access_ok(VERIFY_READ, buf, count))
		return -EFAULT;
	if (*ppos >= SECTOR_SIZE)
		return 0;
	ret = blknv_readwrite_sector(sector, BLKNV_READ);
	if (ret)
		return ret;
	for (i = *ppos; count > 0 && i < SECTOR_SIZE; ++i, ++p, --count) {
		if (__get_user(c, p))
			return -EFAULT;
		if (c != sector[i]) {
			dirty = 1;
			sector[i] = c;
		}
	}
	if (dirty) {
		ret = blknv_readwrite_sector(sector, BLKNV_WRITE);
		if (ret)
			return ret;
	}

	*ppos = i;
	return p - buf;
}

static long nvram_ioctl(struct file *file, unsigned int cmd,
			unsigned long arg)
{
	unsigned char sector[SECTOR_SIZE];

	switch (cmd) {
	case NVRAM_INIT:
		if (!capable(CAP_SYS_ADMIN))
			return -EACCES;
		mutex_lock(&nvram_mutex);
		memset(sector, 0, SECTOR_SIZE);
		blknv_readwrite_sector(sector, BLKNV_WRITE);
		mutex_unlock(&nvram_mutex);
		return 0;
	case NVRAM_SETCKS:
		return 0;
	default:
		return -ENOTTY;
	}
}

static int nvram_open(struct inode *inode, struct file *file)
{
	spin_lock(&nvram_state_lock);

	if ((nvram_open_cnt && (file->f_flags & O_EXCL)) ||
	    (nvram_open_mode & NVRAM_EXCL) ||
	    ((file->f_mode & FMODE_WRITE) && (nvram_open_mode & NVRAM_WRITE))) {
	    	spin_unlock(&nvram_state_lock);
		return -EBUSY;
	}

	if (file->f_flags & O_EXCL)
		nvram_open_mode |= NVRAM_EXCL;
	if (file->f_mode & FMODE_WRITE)
		nvram_open_mode |= NVRAM_WRITE;
	nvram_open_cnt++;

	spin_unlock(&nvram_state_lock);
	return 0;
}

static int nvram_release(struct inode *inode, struct file *file)
{
	spin_lock(&nvram_state_lock);

	nvram_open_cnt--;

	if (nvram_open_mode & NVRAM_EXCL)
		nvram_open_mode &= ~NVRAM_EXCL;
	if (file->f_mode & FMODE_WRITE)
		nvram_open_mode &= ~NVRAM_WRITE;

	spin_unlock(&nvram_state_lock);
	return 0;
}

/* procfs /proc/driver/nvram */
#ifndef CONFIG_PROC_FS
static int nvram_add_proc_fs(void)
{
	return 0;
}

#else

static void proc_infos(unsigned char *nvram, struct seq_file *seq, void *offset)
{
	unsigned int value;
	seq_printf(seq, "NVRAM status:\n");
	/* header */
	value = nvram[0];
	seq_printf(seq, "  %02x:header\n", value);
	seq_printf(seq, "    %d - signature\n",
		(value & 0x40) ? 1 : 0);
	seq_printf(seq, "    %d - fw settings reset\n",
		(value & 0x20) ? 1 : 0);
	seq_printf(seq, "    %d - kernel settings reset\n",
		(value & 0x10) ? 1 : 0);
	/* boot */
	value = nvram[1];
	seq_printf(seq, "  %02x:boot\n", value);
	seq_printf(seq, "    %d - debug reset mode\n",
		(value & 0x80) ? 1 : 0);
	seq_printf(seq, "    %d - try b count\n",
		(value & 0xf));
	/* recovery */
	value = nvram[2];
	seq_printf(seq, "  %02x:recovery\n", value);
	/* localization */
	value = nvram[3];
	seq_printf(seq, "  %02x:localization\n", value);
	/* firmware */
	value = nvram[5];
	seq_printf(seq, "  %02x:firmware\n", value);
	seq_printf(seq, "    %d - func\n", (value & 0x38) >> 3);
	seq_printf(seq, "    %d - num\n", value & 7);
	/* kernel field */
	seq_printf(seq, "  %02x %02x %02x %02x :kernel\n",
		nvram[14], nvram[13], nvram[12], nvram[11]);
}

static int nvram_proc_read(struct seq_file *seq, void *offset)
{
	unsigned char sector[SECTOR_SIZE];

	memset(sector, 0, SECTOR_SIZE);
	if(blknv_readwrite_sector(sector, BLKNV_READ))
		return -EFAULT;
	proc_infos(sector, seq, offset);

	return 0;
}

static int nvram_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, nvram_proc_read, NULL);
}

static const struct file_operations nvram_proc_fops = {
	.owner		= THIS_MODULE,
	.open		= nvram_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int nvram_add_proc_fs(void)
{
	if (!proc_create("driver/nvram", 0, NULL, &nvram_proc_fops))
		return -ENOMEM;
	return 0;
}

#endif /* CONFIG_PROC_FS */

static const struct file_operations nvram_fops = {
	.owner		= THIS_MODULE,
	.llseek		= nvram_llseek,
	.read		= nvram_read,
	.write		= nvram_write,
	.unlocked_ioctl	= nvram_ioctl,
	.open		= nvram_open,
	.release	= nvram_release,
};

static struct miscdevice nvram_dev = {
	.minor		= NVRAM_MINOR,
	.name		= "nvram",
	.fops		= &nvram_fops
};

static int __init nvram_pre_init(void)
{
	int ret;
	/* initialize nvram cache  */
	is_nvram_post_init	= 0;
	is_nvram_module_init	= 0;
	is_nvram_pre_init	= 0;
	nvram_cache		= kzalloc(SECTOR_SIZE, GFP_KERNEL);
	if (!nvram_cache) {
		ret = -ENOMEM;
		goto out;
	}
	nvram_cache_dirty	= kzalloc(SECTOR_SIZE, GFP_KERNEL);
	if (!nvram_cache_dirty) {
		ret = -ENOMEM;
		goto out;
	}
	/* get nvram from cookie */
	if (cros_acpi_init()) {
		ret = -EFAULT;
		goto out;
	}
	memcpy(nvram_cache, get_firmware_nvram_addr(),
		get_firmware_nvram_size());

	is_nvram_pre_init = 1;
	return 0;
out:
	if (nvram_cache) {
		kfree(nvram_cache);
		nvram_cache = NULL;
	}
	if (nvram_cache_dirty) {
		kfree(nvram_cache_dirty);
		nvram_cache_dirty = NULL;
	}
	return ret;
}
arch_initcall_sync(nvram_pre_init);

static int __init nvram_init(void)
{
	int ret;

	if (!is_nvram_pre_init)
		return -EFAULT;
	/* register /dev/nvram and /proc/driver/nvram */
	ret = misc_register(&nvram_dev);
	if (ret) {
		printk(KERN_ERR "nvram: can't misc_register on minor=%d\n",
			NVRAM_MINOR);
		goto out;
	}
	ret = nvram_add_proc_fs();
	if (ret) {
		printk(KERN_ERR "nvram: can't create /proc/driver/nvram\n");
		goto outmisc;
	}
	printk(KERN_INFO "Non-volatile memory driver\n");
	is_nvram_module_init = 1;
	goto out;
outmisc:
	misc_deregister(&nvram_dev);
out:
	return ret;
}
module_init(nvram_init);

static __init int nvram_post_init(void)
{
	unsigned char sector[SECTOR_SIZE];
	int i, dirty = 0;

	if (!is_nvram_module_init)
		return -EFAULT;

	if(blknv_readwrite_sector(sector, BLKNV_READ))
		return -EFAULT;
	for (i = 0; i < get_firmware_nvram_size(); i++) {
		if (nvram_cache_dirty[i]) {
			dirty = 1;
			sector[i] = nvram_cache[i];
		}
	}
	if (dirty) {
		printk(MY_INFO "nvram cache write back\n");
		if (blknv_readwrite_sector(sector, BLKNV_WRITE))
			return -EFAULT;
	}
	is_nvram_post_init = 1;
	kfree(nvram_cache);
	nvram_cache = NULL;
	kfree(nvram_cache_dirty);
	nvram_cache_dirty = NULL;
	printk(MY_INFO "nvram cache released\n");
	return 0;
}
late_initcall_sync(nvram_post_init);

