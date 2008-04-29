/* include/linux/android_pmem.h
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

#ifndef _ANDROID_PMEM_H_
#define _ANDROID_PMEM_H_

#define PMEM_IOCTL_MAGIC 'p'
#define PMEM_GET_PHYS		_IOW(PMEM_IOCTL_MAGIC, 1, unsigned int)
#define PMEM_MAP		_IOW(PMEM_IOCTL_MAGIC, 2, unsigned int)
#define PMEM_GET_SIZE		_IOW(PMEM_IOCTL_MAGIC, 3, unsigned int)
#define PMEM_UNMAP		_IOW(PMEM_IOCTL_MAGIC, 4, unsigned int)
/* This ioctl will allocate pmem space, backing the file, it will fail
 * if the file already has an allocation, pass it the len as the argument
 * to the ioctl */
#define PMEM_ALLOCATE		_IOW(PMEM_IOCTL_MAGIC, 5, unsigned int)
/* This will connect a one pmem file to another, pass the file that is already
 * backed in memory as the argument to the ioctl
 */
#define PMEM_CONNECT		_IOW(PMEM_IOCTL_MAGIC, 6, unsigned int)
#define PMEM_GET_TOTAL_SIZE	_IOW(PMEM_IOCTL_MAGIC, 7, unsigned int)

int get_pmem_file(unsigned int fd, unsigned long *start, unsigned long *end, struct file **filp);
int get_pmem_fd(unsigned int fd, unsigned long *start, unsigned long *end);
void put_pmem_file(struct file* file);
void put_pmem_fd(unsigned int fd);
void flush_pmem_fd(unsigned int fd, unsigned long start, unsigned long len);

struct pmem_region {
	unsigned long offset;
	unsigned long len;
};

#endif //_ANDROID_PPP_H_

