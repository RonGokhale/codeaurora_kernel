/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *    * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *    * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *    * Neither the name of Code Aurora Forum nor
 *      the names of its contributors may be used to endorse or promote
 *      products derived from this software without specific prior written
 *      permission.
 *
 * Alternatively, provided that this notice is retained in full, this software
 * may be relicensed by the recipient under the terms of the GNU General Public
 * License version 2 ("GPL") and only version 2, in which case the provisions of
 * the GPL apply INSTEAD OF those given above.  If the recipient relicenses the
 * software under the GPL, then the identification text in the MODULE_LICENSE
 * macro must be changed to reflect "GPLv2" instead of "Dual BSD/GPL".  Once a
 * recipient changes the license terms to the GPL, subsequent recipients shall
 * not relicense under alternate licensing terms, including the BSD or dual
 * BSD/GPL terms.  In addition, the following license statement immediately
 * below and between the words START and END shall also then apply when this
 * software is relicensed under the GPL:
 *
 * START
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 and only version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * END
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
/**
 * Cache routines for different DMA operations on various kinds of memories.
 * arch/arm/mach-msm/8kcacheroutines.S
 *
 */
#include <linux/linkage.h>
#include <linux/init.h>
#include <asm/assembler.h>
#include "../mm/proc-macros.S"

/**
 * msm_dma_nb_inv_range(start,end) - Clean and Invalidate the data cache
 * within the specified region
 * @start: virtual start address of region
 * @end: virtual end address of region
 *
 * No barriers used
 **/
ENTRY(msm_dma_nb_inv_range)
	dcache_line_size r2, r3
	sub r3, r2, #1
	tst r0, r3
	bic r0, r0, r3
	mcrne p15, 0, r0, c7, c14, 1	@ clean & invalidate D / U line

	tst r1, r3
	bic r1, r1, r3
	mcrne p15, 0, r1, c7, c14, 1	@ clean & invalidate D / U line
1:
	mcr p15, 0, r0, c7, c6, 1	@ invalidate D / U line
	add r0, r0, r2
	cmp r0, r1
	blo 1b
	mov pc, lr
ENDPROC(msm_dma_nb_inv_range)

/**
 * msm_dma_nb_clean_range(start,end) - Clean the specified region
 * @start: virtual start address of region
 * @end: virtual end address of region
 *
 * No barriers used
 **/
ENTRY(msm_dma_nb_clean_range)
	dcache_line_size r2, r3
	sub r3, r2, #1
	bic r0, r0, r3
1:
	mcr p15, 0, r0, c7, c10, 1	@ clean D / U line
	add r0, r0, r2
	cmp r0, r1
	blo 1b
	mov pc, lr
ENDPROC(msm_dma_nb_clean_range)

/**
 * msm_dma_nb_flush_range(start,end) - cache flush the specified region
 * @start: virtual start address of region
 * @end: virtual end address of region
 *
 * No barriers used
 **/
ENTRY(msm_dma_nb_flush_range)
	dcache_line_size r2, r3
	sub r3, r2, #1
	bic r0, r0, r3
1:
	mcr p15, 0, r0, c7, c14, 1	@ clean & invalidate D / U line
	add r0, r0, r2
	cmp r0, r1
	blo 1b
	mov pc, lr
ENDPROC(msm_dma_nb_flush_range)

/**
 * msm_dma_nb_noclean_inv_range(start,end) - Invalidate the data cache
 * within the specified region
 * @start: virtual start address of region
 * @end: virtual end address of region
 *
 * No barriers used
 **/
ENTRY(msm_dma_nb_noclean_inv_range)
	dcache_line_size r2, r3
	sub r3, r2, #1
	bic r0, r0, r3
1:
	mcr p15, 0, r0, c7, c6, 1	@ invalidate D / U line
	add r0, r0, r2
	cmp r0, r1
	blo 1b
	mov pc, lr
ENDPROC(msm_dma_nb_noclean_inv_range)

/**
 * msm_dma_cb_fromdevice_wt_post(start,end) - Done doing DMA operation on
 * writethrough memory. Invalidate cache and do a sync.
 * @start: virtual start address of region
 * @end: virtual end address of region
 *
 **/
ENTRY(msm_dma_cb_fromdevice_wt_post)
	dcache_line_size r2, r3
	sub r3, r2, #1
	bic r0, r0, r3
1:
	mcr p15, 0, r0, c7, c6, 1	@ invalidate D / U line
	add r0, r0, r2
	cmp r0, r1
	blo 1b
	dsb				@ sync barrier
	mov pc, lr
ENDPROC(msm_dma_cb_fromdevice_wt_post)

/**
 * msm_dma_cb_fromdevice_wb_pre(start,end) - Clean and invalidate plus sync
 * barrier before performing DMA op from device on writeback memory.
 * @start: virtual start address of region
 * @end: virtual end address of region
 *
 **/
ENTRY(msm_dma_cb_fromdevice_wb_pre)
	dcache_line_size r2, r3
	sub r3, r2, #1
	tst r0, r3
	bic r0, r0, r3
	mcrne p15, 0, r0, c7, c14, 1	@ clean & invalidate D / U line

	tst r1, r3
	bic r1, r1, r3
	mcrne p15, 0, r1, c7, c14, 1	@ clean & invalidate D / U line
1:
	mcr p15, 0, r0, c7, c6, 1	@ invalidate D / U line
	add r0, r0, r2
	cmp r0, r1
	blo 1b
	dsb				@ sync barrier
	mov pc, lr
ENDPROC(msm_dma_cb_fromdevice_wb_pre)

/**
 * msm_dma_cb_fromdevice_wb_post(start,end) - Invalidate and sync
 * barrier after performing DMA op from device on writeback memory.
 * @start: virtual start address of region
 * @end: virtual end address of region
 *
 **/
ENTRY(msm_dma_cb_fromdevice_wb_post)
	dcache_line_size r2, r3
	sub r3, r2, #1
	bic r0, r0, r3
1:
	mcr p15, 0, r0, c7, c6, 1	@ invalidate D / U line
	add r0, r0, r2
	cmp r0, r1
	blo 1b
	dsb				@ sync barrier
	mov pc, lr
ENDPROC(msm_dma_cb_fromdevice_wb_post)

/**
 * msm_dma_cb_todevice_wb_pre(start,end) - Clean and sync barrier
 * before performing DMA op to device on writeback memory.
 * @start: virtual start address of region
 * @end: virtual end address of region
 *
 **/
ENTRY(msm_dma_cb_todevice_wb_pre)
	dcache_line_size r2, r3
	sub r3, r2, #1
	bic r0, r0, r3
1:
	mcr p15, 0, r0, c7, c10, 1	@ clean D / U line
	add r0, r0, r2
	cmp r0, r1
	blo 1b
	dsb				@ sync barrier
	mov pc, lr
ENDPROC(msm_dma_cb_todevice_wb_pre)
