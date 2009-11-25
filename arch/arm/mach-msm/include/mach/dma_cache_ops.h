/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of Code Aurora nor
 *     the names of its contributors may be used to endorse or promote
 *     products derived from this software without specific prior written
 *     permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NON-INFRINGEMENT ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * Routines for cache operations before and after DMA using different kinds
 * of memories.
 * arch/arm/mach-msm/include/mach/dma_cache_ops.h
 *
 * Notations used:
 * nb    --> no barriers used but cache operations performed
 * cb    --> cache operations performed and barriers used
 * pre   --> performed before DMA transfer
 * post  --> performed after DMA transfer
 * nc    --> non cacheable memory
 * wb    --> write-back memory
 * wt    --> write-through memory
 * wbwa  --> write-back write allocate
 * wbnwa --> write-back no write allocate
 *
 **/

/**
 * Clean and invalidate cache range within the start and end address passed as
 * input parameters in that order. This function uses no barriers.
 *
 **/
extern void msm_dma_nb_inv_range(unsigned long, unsigned long);

/**
 * Clean cache range within the start and end address passed as
 * input parameters in that order. This function uses no barriers.
 *
 **/
extern void msm_dma_nb_clean_range(unsigned long, unsigned long);

/**
 * Flush cache range within the start and end address passed as
 * input parameters in that order. This function uses no barriers.
 *
 **/
extern void msm_dma_nb_flush_range(unsigned long, unsigned long);

/**
 * Invalidate cache range within the start and end address passed as
 * input parameters in that order. This function uses no barriers.
 *
 **/
extern void msm_dma_nb_noclean_inv_range(unsigned long, unsigned long);

/**
 * Clean and invalidate cache range within the start and end address passed
 * as input parameters in that order plus dsb before performing DMA operation
 * from device on writeback memory.
 *
 **/
extern void msm_dma_cb_fromdevice_wb_pre(unsigned long, unsigned long);

/**
 * Invalidate cache range within the start and end address passed as
 * input parameters in that order and dsb after performing DMA operation
 * from device on writeback memory.
 *
 **/
extern void msm_dma_cb_fromdevice_wb_post(unsigned long, unsigned long);

/**
 * Clean cache range within the start and end address passed as
 * input parameters in that order and dsb before performing DMA operation
 * to device on writeback memory.
 *
 **/
extern void msm_dma_cb_todevice_wb_pre(unsigned long, unsigned long);

/**
 * Invalidate cache range within the start and end address passed as
 * input parameters in that order and dsb after DMA operation on
 * writethrough memory.
 *
 **/
extern void msm_dma_cb_fromdevice_wt_post(unsigned long, unsigned long);

/**
 * Perform 'dsb' operation.
 *
 **/
static inline void
msm_dma_dsb(void)
{
 __asm__ __volatile__ ("dsb" : : : "memory");
}

/**
 * Perform 'dsb st' operation.
 *
 **/
static inline void
msm_dma_dsb_st(void)
{
 __asm__ __volatile__ ("dsb st" : : : "memory");
}

/**
 * Perform 'dmb' operation.
 *
 **/
static inline void
msm_dma_dmb(void)
{
 __asm__ __volatile__ ("dmb" : : : "memory");
}

/**
 * Perform 'dmb' operation before DMA operation from device on non-cacheable
 * memory.
 *
 **/
static inline void
msm_dma_fromdevice_nc_pre(void)
{
	msm_dma_dmb();
}

/**
 * Perform 'dmb' operation after DMA operation from device on non-cacheable
 * memory.
 *
 **/
static inline void
msm_dma_fromdevice_nc_post(void)
{
	msm_dma_dmb();
}

/**
 * Perform 'dsb st' operation before DMA operation to device on non-cacheable
 * memory.
 *
 **/
static inline void
msm_dma_todevice_nc_pre(void)
{
	msm_dma_dsb_st();
}

/**
 * Perform 'dmb' operation after DMA operation to device on non-cacheable
 * memory.
 *
 **/
static inline void
msm_dma_todevice_nc_post(void)
{
	msm_dma_dmb();
}

/**
 * Perform 'dmb' operation after DMA operation to device for write-back
 * memory.
 *
 **/
static inline void
msm_dma_todevice_wb_post(void)
{
	msm_dma_dmb();
}

/**
 * Perform 'dsb st' operation before DMA operation from device for write-through
 * memory.
 *
 **/
static inline void
msm_dma_fromdevice_wt_pre(void)
{
	msm_dma_dsb_st();
}

/**
 * Perform 'dsb st' operation before DMA operation to device for write-through
 * memory.
 *
 **/
static inline void
msm_dma_todevice_wt_pre(void)
{
	msm_dma_dsb_st();
}

/**
 * Perform 'dmb' operation after DMA operation to device for write-through
 * memory.
 *
 **/
static inline void
msm_dma_todevice_wt_post(void)
{
	msm_dma_dmb();
}

/**
 * Performed before DMA operation from device on write-back write allocate memory.
 * Same as msm_dma_cb_fromdevice_wb_pre.
 *
 **/
static inline void
msm_dma_cb_fromdevice_wbwa_pre(unsigned long start, unsigned long end)
{
	msm_dma_cb_fromdevice_wb_pre(start, end);
}

/**
 * Performed after DMA operation from device on write-back write allocate memory.
 * Same as msm_dma_cb_fromdevice_wb_post.
 *
 **/
static inline void
msm_dma_cb_fromdevice_wbwa_post(unsigned long start, unsigned long end)
{
	msm_dma_cb_fromdevice_wb_post(start, end);
}

/**
 * Performed before DMA operation to device on write-back write allocate memory.
 * Same as msm_dma_cb_fromdevice_wb_pre.
 *
 **/
static inline void
msm_dma_cb_todevice_wbwa_pre(unsigned long start, unsigned long end)
{
	msm_dma_cb_fromdevice_wb_pre(start, end);
}

/**
 * Performed after DMA operation to device on write-back write allocate memory.
 * Same as msm_dma_cb_fromdevice_wb_post.
 *
 **/
static inline void
msm_dma_cb_todevice_wbwa_post(unsigned long start, unsigned long end)
{
	msm_dma_cb_fromdevice_wb_post(start, end);
}

/**
 * Performed before DMA operation from device on write-back no write allocate memory.
 * Same as msm_dma_cb_fromdevice_wb_pre.
 *
 **/
static inline void
msm_dma_cb_fromdevice_wbnwa_pre(unsigned long start, unsigned long end)
{
	msm_dma_cb_fromdevice_wb_pre(start, end);
}

/**
 * Performed after DMA operation from device on write-back no write allocate memory.
 * Same as msm_dma_cb_fromdevice_wb_post.
 *
 **/
static inline void
msm_dma_cb_fromdevice_wbnwa_post(unsigned long start, unsigned long end)
{
	msm_dma_cb_fromdevice_wb_post(start, end);
}

/**
 * Performed before DMA operation to device on write-back no write allocate memory.
 * Same as msm_dma_cb_todevice_wb_pre.
 *
 **/
static inline void
msm_dma_cb_todevice_wbnwa_pre(unsigned long start, unsigned long end)
{
	msm_dma_cb_todevice_wb_pre(start, end);
}

/**
 * Performed after DMA operation to device on write-back no write allocate memory.
 * Same as msm_dma_todevice_wb_post.
 *
 **/
static inline void
msm_dma_todevice_wbnwa_post(void)
{
	msm_dma_todevice_wb_post();
}
