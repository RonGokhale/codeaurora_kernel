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
#define PMEM_GET_PHYS	_IOW(PMEM_IOCTL_MAGIC, 1, unsigned int)
#define PMEM_RESTRICT	_IOW(PMEM_IOCTL_MAGIC, 2, unsigned int)
#define PMEM_GET_SIZE	_IOW(PMEM_IOCTL_MAGIC, 3, unsigned int)

int get_pmem_file(unsigned long fd, unsigned long *start, unsigned long *end);
void put_pmem_file(unsigned long fd);

struct pmem_addr {
	void *start;
	void *end;
};

#endif //_ANDROID_PPP_H_

