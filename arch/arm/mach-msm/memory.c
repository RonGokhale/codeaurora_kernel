/* arch/arm/mach-msm/memory.c
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

#include <linux/mm.h>
#include <asm/cacheflush.h>

int io_remap_pfn_range(struct vm_area_struct *vma, unsigned long addr,
		       unsigned long pfn, unsigned long size,
		       pgprot_t prot)
{
	unsigned long pfn_addr = pfn << PAGE_SHIFT;
	if ((pfn_addr >= 0x88000000) && (pfn_addr < 0xD0000000))
		prot = pgprot_device(prot);
	return remap_pfn_range(vma, addr, pfn, size, prot);
}

EXPORT_SYMBOL(io_remap_pfn_range);
