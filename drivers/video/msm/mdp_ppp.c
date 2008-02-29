/* drivers/video/msm/mdp_ppp.c
 *
 * Copyright (C) 2007 QUALCOMM Incorporated
 * Copyright (C) 2007 Google Incorporated
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/fb.h>
#include <linux/delay.h>
#include <linux/msm_mdp.h>
#include <linux/android_pmem.h>
#include <asm/arch/msm_fb.h>

#include "mdp_hw.h"

struct mdp_regs
{
	uint32_t src0;
	uint32_t src1;
	uint32_t dst0;
	uint32_t dst1;
	uint32_t src_cfg;
	uint32_t dst_cfg;
	uint32_t src_ystride;
	uint32_t dst_ystride;
	uint32_t op;
	uint32_t src_bpp;
	uint32_t dst_bpp;
};

static uint32_t pack_pattern[] = {
	PPP_ARRAY0(PACK_PATTERN)
};

static uint32_t src_img_cfg[] = {
	PPP_ARRAY1(CFG, SRC)
};

static uint32_t dst_img_cfg[] = {
	PPP_ARRAY1(CFG, DST)
};

static uint32_t bytes_per_pixel[] = {
	[MDP_RGB_565] = 2,
	[MDP_RGB_888] = 3,
	[MDP_XRGB_8888] = 4,
	[MDP_ARGB_8888] = 4,
	[MDP_Y_CBCR_H2V1] = 1,
	[MDP_Y_CBCR_H2V2] = 1,
	[MDP_Y_CRCB_H2V1] = 1,
	[MDP_Y_CRCB_H2V2] = 1,
	[MDP_YCRYCB_H2V1] = 2
};

static uint32_t dst_op_chroma[] = {
	PPP_ARRAY1(CHROMA_SAMP, DST)
};

static uint32_t src_op_chroma[] = {
	PPP_ARRAY1(CHROMA_SAMP, SRC)
};

static uint32_t bg_op_chroma[] = {
	PPP_ARRAY1(CHROMA_SAMP, BG)
};

static void rotate_dst_addr_x(struct mdp_blit_req *req, struct mdp_regs *regs)
{
	regs->dst0 += (req->dst_rect.w -
		       min((uint32_t)16, req->dst_rect.w)) * regs->dst_bpp;
	regs->dst1 += (req->dst_rect.w -
		       min((uint32_t)16, req->dst_rect.w)) * regs->dst_bpp;
}

static void rotate_dst_addr_y(struct mdp_blit_req *req, struct mdp_regs *regs)
{
	regs->dst0 += (req->dst_rect.h -
		       min((uint32_t)16, req->dst_rect.h)) *
		       regs->dst_ystride;
	regs->dst1 += (req->dst_rect.h -
		       min((uint32_t)16, req->dst_rect.h)) *
		       regs->dst_ystride;
}

static void blit_rotate(struct mdp_blit_req *req,
			struct mdp_regs *regs)
{
	regs->op |= PPP_OP_ROT_ON;
	if ((req->rotation & MDP_ROT_90 || req->rotation & MDP_FLIP_LR) &&
	    !(req->rotation & MDP_ROT_90 && req->rotation & MDP_FLIP_LR))
		rotate_dst_addr_x(req, regs);
	if (req->rotation & MDP_ROT_90) {
		regs->op |= PPP_OP_ROT_90;
	}
	if (req->rotation & MDP_FLIP_UD) {
		regs->op |= PPP_OP_FLIP_UD;
		rotate_dst_addr_y(req, regs);
	}
	if (req->rotation & MDP_FLIP_LR) {
		regs->op |= PPP_OP_FLIP_LR;
	}
}

static void blit_convert(struct mdp_blit_req *req, struct mdp_regs *regs)
{
	if (IS_RGB(req->src.format) && IS_YCRCB(req->dst.format)) {
		regs->op |= PPP_OP_CONVERT_RGB2YCBCR | PPP_OP_CONVERT_ON;
	} else if (IS_YCRCB(req->src.format) && IS_RGB(req->dst.format)) {
		regs->op |= PPP_OP_CONVERT_YCBCR2RGB | PPP_OP_CONVERT_ON;
		if (req->dst.format == MDP_RGB_565)
			regs->op |= PPP_OP_CONVERT_MATRIX_SECONDARY;
	}
}

#define GET_BIT_RANGE(value, start, end) \
	((1 << (start - end+ 1)) - 1) & (value >> start)
static uint32_t transp_convert(struct mdp_blit_req *req)
{
	uint32_t transp = 0;
	if (req->src.format == MDP_RGB_565) {
		/* pad each value to 8 bits by copying the high bits into the
		 * low end, convert RGB to RBG by switching low 2 components */
		transp |= ((GET_BIT_RANGE(req->transp_mask, 15, 11) << 3) |
			  (GET_BIT_RANGE(req->transp_mask, 15, 13))) << 16;

		transp |= ((GET_BIT_RANGE(req->transp_mask, 4, 0) << 3) |
			   (GET_BIT_RANGE(req->transp_mask, 4, 2))) << 8;

		transp |= (GET_BIT_RANGE(req->transp_mask, 10, 5) << 2) |
			   (GET_BIT_RANGE(req->transp_mask, 10, 9));

	} else {
		/* convert RGB to RBG */
		transp |= (GET_BIT_RANGE(req->transp_mask, 15, 8)) |
			  (GET_BIT_RANGE(req->transp_mask, 23, 16) << 16) |
			  (GET_BIT_RANGE(req->transp_mask, 7, 0) << 8);
	}
	return transp;
}
#undef GET_BIT_RANGE

static void blit_blend(struct mdp_blit_req *req, struct mdp_regs *regs)
{
	if (req->alpha > 0xff)
		return;

	if (unlikely(req->src.format == MDP_ARGB_8888))
		return;

	regs->op |= bg_op_chroma[req->dst.format];

	if (req->transp_mask != MDP_TRANSP_NOP) {
		req->transp_mask = transp_convert(req);
		if (req->alpha != MDP_ALPHA_NOP) {
			/* use blended transparancy mode
			 * pixel = (src == transp) ? dst : blend
			 * blend is combo of blend_eq_sel and
			 * blend_alpha_sel */
			regs->op |= PPP_OP_ROT_ON | PPP_OP_BLEND_ON |
				PPP_OP_BLEND_ALPHA_BLEND_NORMAL |
				PPP_OP_BLEND_CONSTANT_ALPHA |
				PPP_BLEND_ALPHA_TRANSP;
		} else {
			/* simple transparancy mode
			 * pixel = (src == transp) ? dst : src */
			regs->op |= PPP_OP_ROT_ON | PPP_OP_BLEND_ON |
				PPP_OP_BLEND_SRCPIXEL_TRANSP;
		}
	} else {
		/* just blend by alpha */
		regs->op |= PPP_OP_ROT_ON | PPP_OP_BLEND_ON |
			PPP_OP_BLEND_ALPHA_BLEND_NORMAL |
			PPP_OP_BLEND_CONSTANT_ALPHA;
	}
}

static int valid_src_dst(unsigned long src_start, unsigned long src_len,
			 unsigned long dst_start, unsigned long dst_len,
			 struct mdp_blit_req *req, struct mdp_regs *regs)
{
	unsigned long src_min_ok = src_start;
	unsigned long src_max_ok = src_start + src_len - 1;
	unsigned long dst_min_ok = dst_start;
	unsigned long dst_max_ok = dst_start + dst_len - 1;

	uint32_t src_size = ((req->src.height - 1) * req->src.width +
			    req->src_rect.w) * regs->src_bpp;
	uint32_t dst_size = ((req->dst.height - 1) * req->dst.width +
			    req->dst_rect.w) * regs->dst_bpp;

	if (regs->src0 < src_min_ok || regs->src0 > src_max_ok ||
	    regs->src0 + src_size > src_max_ok) {
		return 0;
	}
	if (regs->src_cfg & PPP_SRC_PLANE_PSEUDOPLNR) {
		if (regs->src1 < src_min_ok || regs->src1 > src_max_ok ||
		    regs->src1 + src_size > src_max_ok) {
			return 0;
		}
	}
	if (regs->dst0 < dst_min_ok || regs->dst0 > dst_max_ok ||
	    regs->dst0 + dst_size > dst_max_ok) {
		return 0;
	}
	if (regs->dst_cfg & PPP_SRC_PLANE_PSEUDOPLNR) {
		if (regs->dst1 < dst_min_ok || regs->dst1 > dst_max_ok ||
		    regs->dst1 + dst_size > dst_max_ok) {
			return 0;
		}
	}
	return 1;
}

int get_img(struct mdp_img *img, struct fb_info *info, unsigned long *start,
	    unsigned long *len)
{
	switch(img->memory_type) {
		case FB_IMG:
			*start = info->fix.smem_start;
			*len = info->fix.smem_len;
			break;
		case PMEM_IMG:
			get_pmem_file(img->memory_id, start, len);
			break;
		default:
			return -1;
	}
	return 0;
}

void mdp_ppp_put_img(struct mdp_blit_req *req)
{
	if (req->src.memory_type == PMEM_IMG)
		put_pmem_file(req->src.memory_id);
	if (req->dst.memory_type == PMEM_IMG)
		put_pmem_file(req->dst.memory_id);
}

#define WRITEL(v, a) do { writel(v,a); /*printk(#a "[%x]=%x\n", a, v);*/ }\
		     while (0)
int mdp_ppp_blit(struct fb_info *info, struct mdp_blit_req *req)
{
	struct mdp_regs regs;
	unsigned long src_start, src_len;
	unsigned long dst_start, dst_len;

#if 0
	printk("BLIT recieved img %p\n"
		" src:%d\n"
		" src_type: %d\n"
		" src_w: %d\n"
		" src_h: %d\n"
		" src_roi_x: %u\n"
		" src_roi_y: %u\n"
		" src_roi_w: %u\n"
		" src_roi_h: %u\n"

		" dst:%d\n"
		" dst_type: %d\n"
		" dst_w: %d\n"
		" dst_h: %d\n"
		" dst_roi_x: %u\n"
		" dst_roi_y: %u\n"
		" dst_roi_w: %u\n"
		" dst_roi_h: %u\n"

		" rotate: %u\n"
		" transp: %u\n"
		" alpha: %u\n",
		req, 
		req->src.offset, req->src.format,
		req->src.width, req->src.height,
		req->src_rect.x, req->src_rect.y,
		req->src_rect.w, req->src_rect.h,
		req->dst.offset, req->dst.format,
		req->dst.width, req->dst.height,
		req->dst_rect.x, req->dst_rect.y,
		req->dst_rect.w, req->dst_rect.h,
		req->rotation, req->transp_mask, req->alpha);
#endif
	/* do this first so that if this fails, the caller can always
	 * safely call put_img */
	if (unlikely(get_img(&req->src, info, &src_start, &src_len)) ||
		    (get_img(&req->dst, info, &dst_start, &dst_len)))
		return -1;

	if (unlikely(req->src.format >= MDP_IMGTYPE_LIMIT ||
	    req->dst.format >= MDP_IMGTYPE_LIMIT))
		return -1;


	if (unlikely(req->src_rect.x > req->src.width ||
		     req->src_rect.y > req->src.height ||
		     req->dst_rect.x > req->dst.width ||
		     req->dst_rect.y > req->dst.height))
		return -1;

	regs.src_cfg = src_img_cfg[req->src.format];
	regs.src_cfg |= (req->src_rect.x % 2) ? PPP_SRC_BPP_ROI_ODD_X : 0;
	regs.src_cfg |= (req->src_rect.y % 2) ? PPP_SRC_BPP_ROI_ODD_Y : 0;

	regs.dst_cfg = dst_img_cfg[req->dst.format] | PPP_DST_OUT_SEL_AXI;

	regs.src_bpp = bytes_per_pixel[req->src.format];
	regs.src0 = src_start + req->src.offset;
	regs.src0 += (req->src_rect.x + (req->src_rect.y * req->src.width)) *
		      regs.src_bpp;
	regs.src_ystride = req->src.width * regs.src_bpp;

	if (regs.src_cfg & PPP_SRC_PLANE_PSEUDOPLNR) {
		regs.src1 = regs.src0 + (req->src.width * req->src.height *
					 regs.src_bpp);
		regs.src_ystride |= regs.src_ystride << 16;
	} else {
		regs.src1 = 0;
	}

	regs.dst_bpp = info->var.bits_per_pixel / 8;
	regs.dst0 = dst_start + req->dst.offset;
	regs.dst0 += (req->dst_rect.x + (req->dst_rect.y * req->dst.width)) *
		     regs.dst_bpp;
	regs.dst_ystride = req->dst.width * regs.dst_bpp;
	if (regs.dst_cfg & PPP_DST_PLANE_PSEUDOPLNR) {
		regs.dst1 = regs.dst0 + (req->dst.width * req->dst.height *
					 regs.dst_bpp);
		regs.dst_ystride |= regs.dst_ystride << 16;
	} else {
		regs.dst1 = 0;
	}

	if (!valid_src_dst(src_start, src_len, dst_start, dst_len, req, &regs)) 
		return -1;

	regs.op = 0;
	if (req->rotation != MDP_ROT_NOP)
		blit_rotate(req, &regs);
	if (req->src.format != req->dst.format)
		blit_convert(req, &regs);
	if (req->transp_mask != MDP_TRANSP_NOP || req->alpha != MDP_ALPHA_NOP)
		blit_blend(req, &regs);

	if (unlikely(req->src.format == MDP_ARGB_8888))
		regs.op |= PPP_OP_ROT_ON | PPP_OP_BLEND_ON |
			   PPP_OP_BLEND_SRCPIXEL_ALPHA;

	regs.op |= dst_op_chroma[req->dst.format] |
		   src_op_chroma[req->src.format];
#if 0
	if (req->src.w != dst->src.w ||
	    req->src.h != dst->src.h)
		mdp_blt_scale(req);
#endif


	if (unlikely(req->src.format == MDP_YCRYCB_H2V1)) {
		// the x and w must be even
		req->src_rect.x = (req->src_rect.x / 2) * 2;
		req->src_rect.w = (req->src_rect.w / 2) * 2;
		req->dst_rect.x = (req->dst_rect.x / 2) * 2;
		req->dst_rect.w = (req->dst_rect.w / 2) * 2;
	}

	WRITEL(1, MSM_MDP_BASE+0x060);
	WRITEL((req->src_rect.h << 16) | req->src_rect.w,
		PPP_ADDR_SRC_ROI);
	WRITEL(0, MDP_FULL_BYPASS_WORD46);
	WRITEL(regs.src0, PPP_ADDR_SRC0);
	WRITEL(regs.src1, PPP_ADDR_SRC1);
	WRITEL(regs.src_ystride, PPP_ADDR_SRC_YSTRIDE);
	WRITEL(regs.src_cfg, PPP_ADDR_SRC_CFG);
	WRITEL(pack_pattern[req->src.format], PPP_ADDR_SRC_PACK_PATTERN);

	WRITEL(regs.op, PPP_ADDR_OPERATION);
	// Scale registers here
	WRITEL((req->alpha << 24) | (req->transp_mask & 0xffffff),
		PPP_ADDR_ALPHA_TRANSP);
	WRITEL(regs.dst_cfg, PPP_ADDR_DST_CFG);
	WRITEL(pack_pattern[req->dst.format], PPP_ADDR_DST_PACK_PATTERN);
	WRITEL((req->dst_rect.h << 16) | req->dst_rect.w,
		 PPP_ADDR_DST_ROI);
	WRITEL(regs.dst0, PPP_ADDR_DST0);
	WRITEL(regs.dst1, PPP_ADDR_DST1);
	WRITEL(regs.dst_ystride, PPP_ADDR_DST_YSTRIDE);

	if (regs.op & PPP_OP_BLEND_ON) {
		WRITEL(regs.dst0, PPP_ADDR_BG0);
		WRITEL(regs.dst1, PPP_ADDR_BG1);
		WRITEL(regs.dst_ystride, PPP_ADDR_BG_YSTRIDE);
		WRITEL(src_img_cfg[req->dst.format], PPP_ADDR_BG_CFG);
		WRITEL(pack_pattern[req->dst.format], PPP_ADDR_BG_PACK_PATTERN);
	}

	WRITEL(0x1000, MDP_DISPLAY0_START);
	return 0;
}
