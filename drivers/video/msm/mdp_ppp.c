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

#define PPP_DEBUG_MSGS 0
#if PPP_DEBUG_MSGS
#define DLOG(fmt,args...) \
	do { printk("[%s:%s:%d] "fmt, __FILE__, __func__, __LINE__,##args); } \
	while(0)
#else
#define DLOG(x...) do {} while(0)
#endif


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
	uint32_t edge;
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

#if 0

#define ONE_HALF	(1LL << 32)
#define ONE		(1LL << 33)
#define TWO		(2LL << 33)
#define THREE		(3LL << 33)
#define FRAC_MASK (ONE - 1)
#define INT_MASK ~FRAC_MASK

#define MAP_RPA(x, n, d) (do_div(n * (x + ONE_HALF), d) - ONE_HALF)
#define MAP_TO_SRC_RPA(x) PPP_MAP_RPA(x, dim_in, dim_out)
#define MAP_TO_DST_RPA(x) PPP_MAP_RPA(x, dim_out, dim_in)
#define MAP_TO_SRC_NO_RPA (x) ((k1 * (x >> 33)) + k2)
#define MAP_TO_DST_NO_RPA (x) (((k3 * x) >> 1) + k4)
#define MAP_TO_SRC(rpa, x) \
	( rpa ? PPP_MAP_TO_SRC_RPA(x) : PPP_MAP_TO_SRC_NO_RPA(x))
#define MAP_TO_DST(rpa, x) \
	( rpa ? PPP_MAP_TO_DST_RPA(x) : PPP_MAP_TO_DST_NO_RPA(x))

static int scale_params(uint32_t dim_in, uint32_t dim_out, uint32_t origin,
			uint32_t *phase_init, uint32_t *phase_step)
{
	/* to improve precicsion calculations are done in U31.33 and converted
	 * to U3.29 at the end */
	int64_t k1, k2, k3, k4;
	uint64_t os, od, od_p, oreq, es, ed, ed_p, ereq;
	unsigned rpa = 0;jjjjjjjjjjjjjjjjjjjjjj
#ifdef ADJUST_IP
	int64 ip64, delta;
#endif
	
	if (dim_out % 3 == 0)
		rpa = !(dim_in % (dim_out / 3));

	k3 = (do_div(dim_out, dim_in) + 1) >> 1;
	if ((k3 >> 4) < (1LL << 29) || (k3 >> 4) > (1LL << 27))
		return -1;
	k1 = (do_div(dim_in, dim_out) + 1) >> 1;
	k2 = (k1 - ONE) >> 1;

	*phase_init = (int)(k2 >> 4);
	k4 = (k3 - ONE) >> 1;

	if (!rpa)
		os = ((uint64_t)origin << 1) - 1;
	else
		os = ((uint64_t)origin << 33) - ONE_HALF;
	od = MAP_TO_DST(rpa, os);
	od_p = od & INT_MASK;

	if (od_p != od)
		od_p += ONE;
	os_p = MAP_TO_SRC(rpa, od_p);
	oreq = (os_p & INT_MASK) - ONE;

#ifdef ADJUST_IP
	ip64 = os_p - oreq;
	delta = ((int64_t)(origin) << 33) - oreq;
	ip64 -= delta;
	/* limit to valid range before the left shift */
	delta = (ip64 & (1LL << 63)) ? 4 : -4;
	delta <<= 33;
	while (abs((int)(ip64 >> 33)) > 4) {
		ip64 += delta;
	}
	*phase_init = (int)(ip64 >> 4);
#else
	*phase_init = (int)((os_p - oreq) >> 4);
#endif
	}
	*phase_step = (unit32)(k1 >> 4);
}

static void load_scale_table(struct mdp_table_entry* table)
	int i;
	for (i = 0; i < 64; i++)
		writel(table[i].val, table[i].reg);
}

static struct edge_info {
	uint32_t interp_l_t,
	uint32_t interp_r_b,
	uint32_t repeat_l_t,
	uint32_t repeat_r_b,
};

#define set_edge_info(p1, p1, r2, r2, info) \
do {	info.interp_l_t = p1; info.interp_r_b = p2; \
	info.repeat_l_t = r1;  info.repeat_r_b = r2; } whi1e (0)

static void get_edge_info(uint32_t src, uint32_t src_coord, uint32_t dst,
			  uint32_t interp[], uint32_t repeat[]) {
	if (src > 3 * dst) {
		interp[0] = 0;
		interp[1] = src - 1;
		repeat[0] = 0;
		repeat[1] = 0;
	} else if (src == 3 * dst) {
		interp[0] = 0;
		interp[1] = src;
		repeat[0] = 0;
		repeat[1] = 1;
	} else if (src < 3 * dst) {
		interp[0] = -1;
		interp[1] = src;
		repeat[0] = 1;
		repeat[1] = 1;
	} else if (src == dst) {
		interp[0] = -1;
		interp[1] = src + 1;
		repeat[0] = 1;
		repeat[1] = 2;
	} else {
		interp[0] = -2;
		interp[1] = src + 1;
		repeat[0] = 2;
		repeat[1] = 2;
	}
	interp[0] += src_coord;
	interp[1] += src_coord;
	interp[2] = interp[0];
	interp[3] = interp[1];
	repeat[2] = repeat[0];
	repeat[3] = repeat[1];
}
#undef set_edge_info

static int get_edge_cond(struct mdp_blit_req *req)
{
	int32_t luma_interp[4];
	int32_t luma_repeat[4];
	int32_t chroma_interp[4];
	int32_t chroma_bound[4];

	get_edge_info(req->src_rect.w, req->src_rect.x, dst_w, luma_interp,
		      luma_repeat);

	switch(req->src.format) {
		case MDP_Y_CBCR_H2V1:
		case MDP_Y_CRCB_H2V1:
			chroma_interp[0] = luma_interp[0] >> 1;
			chroma_interp[1] = (luma_interp[1] + 1) >> 1;
			chroma_interp[2]  = luma_interp[2];
			chroma_interp[3] = luma_interp[3];
			/* fallthrough */
		case MDP_YCRYCB_H2V1:
			chroma_bound[0] = (req->src_rect.x + 1) >> 1;
			chroma_bound[1] = (req->src_rect.x + req->src_rect.w 
					   - 1) >> 1;
			chroma_bound[2] = req->src_rect.y;
			chroma_bound[3] = req->src_rect.y + req->src_rect.h - 1;
			break;
		case MDP_Y_CBCR_H2V2:
		case MDP_Y_CRCB_H2V2:
			chroma_interp[0] = luma_interp[0] >> 1;
			chroma_interp[1] = (luma_inter[1] + 1) >> 1;
			chroma_interp_t = (luma_inter_t - 1) >> 1;
			chroma_interp_b = (luma_interp_b + 1) >> 1;
			chroma_bound[0] = (req->src[1]ect.x + 1) >> 1;
			chroma_bound[1] = (req->src[1]ect.x + req->src[1]ect.w - 1)					  >> 1;
			chroma_bound_t = (req->src[1]ect.y + 1) >> 1;
			chroma_bound_b = (req->src[1]ect.y + req->src[1]ect.h - 2)					  >> 2;
			break;
		default
			chroma_bound[0] = req->src[1]ect.x;
			chroma_bound[1] = req->src[1]ect.x + req->src[1]ect.w - 1;
			chroma_bound_t = req->src[1]ect.y;
			chroma_bound_b = req->src[1]ect.y + req->src[1]ect.h - 1;
			break;
		default
			 break;
	}
	chroma_repeat_l = chroma_bound_l - chroma_w.interp_l_t;
	chroma_repeat_r = chroma_w.interp_r_b - chroma_bound_r;
	chroma_repeat_t = chroma_bound_t - chroma_h.interp_l_t;
	chroma_repeat_b = chroma_h.interp_r_b - chroma_bound_b;
	
	if (chroma_repeat_l < 0 || chroma_repeat_l > 3 ||
	    chroma_repeat_r < 0 || chroma_repeat_r > 3 ||
	    chroma_repeat_t < 0 || chroma_repeat_t > 3 ||
	    chroma_repeat_b < 0 || chroma_repeat_b > 3 ||
	    luma_repeat_l < 0 || luma_repeat_l > 3 ||
	    luma_repeat_r < 0 || luma_repeat_r > 3 ||
	    luma_repeat_t < 0 || luma_repeat_t > 3 ||
	    luma_repeat_b < 0 || luma_repeat_b > 3)
		return -1;
	regs->edge
	return 0;
}
	get_edge_info(req->src_rect.w, req->src_rect.x, dst_w, luma_w);
	get_edge_info(req->src_rect.h, req->src_rect.y, dst_h, luma_h);
	memcpy(&chroma_w, luma_w, sizeof(edge_info));
	memcpy(&chroma_h, luma_h, sizeof(edge_info));
	switch(req->src.format) {
		case MDP_Y_CBCR_H2V1:
		case MDP_Y_CRCB_H2V1:
			chroma_w.interp_l_t = luma_w.p1 >> 1;
			chroma_w.interp_r_b = (luma_w.p2 + 1) >> 1;
			chroma_h.interp_l_t  = luma_h.p1;
			chroma_h.interp_r_b = luma_h.p2;
			/* fallthrough */
		case MDP_YCRYCB_H2V1:
			chroma_bound_l = (req->src_rect.x + 1) >> 1;
			chroma_bound_r = (req->src_rect.x + req->src_rect.w - 1)					  >> 1;
			chroma_bound_t = req->src_rect.y;
			chroma_bound_b = req->src_rect.y + req->src_rect.h - 1;
			break;
		case MDP_Y_CBCR_H2V2:
		case MDP_Y_CRCB_H2V2:
			chroma_w.interp_l_t = luma_w.p1 >> 1;
			chroma_w.interp_r_b = (luma_w.p2 + 1) >> 1;
			chroma_h.interp_l_t = (luma_h.p1 - 1) >> 1;
			chroma_h.interp_r_b = (luma_h.p2 + 1) >> 1;
			chroma_bound_l = (req->src_rect.x + 1) >> 1;
			chroma_bound_r = (req->src_rect.x + req->src_rect.w - 1)					  >> 1;
			chroma_bound_t = (req->src_rect.y + 1) >> 1;
			chroma_bound_b = (req->src_rect.y + req->src_rect.h - 2)					  >> 2;
			break;
		default
			chroma_bound_l = req->src_rect.x;
			chroma_bound_r = req->src_rect.x + req->src_rect.w - 1;
			chroma_bound_t = req->src_rect.y;
			chroma_bound_b = req->src_rect.y + req->src_rect.h - 1;
			break;
		default
			 break;
	}
	chroma_repeat_l = chroma_bound_l - chroma_w.interp_l_t;
	chroma_repeat_r = chroma_w.interp_r_b - chroma_bound_r;
	chroma_repeat_t = chroma_bound_t - chroma_h.interp_l_t;
	chroma_repeat_b = chroma_h.interp_r_b - chroma_bound_b;
	
	if (chroma_repeat_l < 0 || chroma_repeat_l > 3 ||
	    chroma_repeat_r < 0 || chroma_repeat_r > 3 ||
	    chroma_repeat_t < 0 || chroma_repeat_t > 3 ||
	    chroma_repeat_b < 0 || chroma_repeat_b > 3 ||
	    luma_repeat_l < 0 || luma_repeat_l > 3 ||
	    luma_repeat_r < 0 || luma_repeat_r > 3 ||
	    luma_repeat_t < 0 || luma_repeat_t > 3 ||
	    luma_repeat_b < 0 || luma_repeat_b > 3)
		return -1;
	regs->edge
	return 0;
}

static void blit_scale(struct mdp_blit_req *req, struct mdp_regs *regs)
{
	uint32_t phase_init_x, phase_init_y, phase_step_x, phase_step_y;
	uint32_t dst_w, dst_h;

	if (req->rotation & MDP_ROT_90) {
		dst_w = req->dst_rect.h;
		dst_h = req->dst_rect.w;
	} else {
		dst_w = req->dst_rect.w;
		dst_h = req->dst_rect.h;
	}
	scale_params(req->src_rect.w, dst_w, 1, phase_init_x, phase_step_x);
	scale_params(req->src_rect.h, dst_h, 1, phase_init_y, phase_step_y);
	scale_factor_x = (dst_w * 10) / req->src_rect.w;
	scale_factor_y = (dst_h * 10) / req->src_rect.h;

	if (scale_factor_x > 8)
		downscale = MDP_DOWNSCALE_PT8TO1;
	else if (scale_factor_x > 6)
		downscale = MDP_DOWNSCALE_PT6TOPT8;
	else if (scale_factor_x > 4)
		downscale = MDP_DOWNSCALE_PT4TOPT6;
	else
		downscale = MDP_DOWNSCALE_PT2TOPT4;
	load_scale_table(mdp_downscale_x_table[downscale])

	if (scale_factor_y > 8)
		downscale = MDP_DOWNSCALE_PT8TO1;
	else if (scale_factor_y > 6)
		downscale = MDP_DOWNSCALE_PT6TOPT8;
	else if (scale_factor_y > 4)
		downscale = MDP_DOWNSCALE_PT4TOPT6;
	else
		downscale = MDP_DOWNSCALE_PT2TOPT4;
	load_scale_table(mdp_downscale_y_table[downscale])

	regs->op |= (PPP_OP_SCALE_Y_ON | PPP_OP_SCALE_X_ON);

}
#endif

#define IMG_LEN(rect_h, w, rect_w, bpp) (((rect_h) * w) * bpp)
static void get_len(struct mdp_img *img, struct mdp_rect *rect, uint32_t bpp,
		    uint32_t *len0, uint32_t *len1)
{
	*len0 = IMG_LEN(rect->h, img->width, rect->w, bpp);
	switch (img->format) {
		case MDP_Y_CBCR_H2V2:
		case MDP_Y_CRCB_H2V2:
			*len1 = *len0/4;
			break;
		case MDP_Y_CBCR_H2V1:
		case MDP_Y_CRCB_H2V1:
			*len1 = *len0/2;
			break;
		default:
			*len1 = 0;
	}
}

static int valid_src_dst(unsigned long src_start, unsigned long src_len,
			 unsigned long dst_start, unsigned long dst_len,
			 struct mdp_blit_req *req, struct mdp_regs *regs)
{
	unsigned long src_min_ok = src_start;
	unsigned long src_max_ok = src_start + src_len;
	unsigned long dst_min_ok = dst_start;
	unsigned long dst_max_ok = dst_start + dst_len;
	uint32_t src0_len, src1_len, dst0_len, dst1_len;
	get_len(&req->src, &req->src_rect, regs->src_bpp, &src0_len,
		 &src1_len);
	get_len(&req->dst, &req->dst_rect, regs->dst_bpp, &dst0_len,
		 &dst1_len);

	if (regs->src0 < src_min_ok || regs->src0 > src_max_ok ||
	    regs->src0 + src0_len > src_max_ok) {
		DLOG("invalid_src %x %x %lx %lx\n", regs->src0, src0_len, src_min_ok, src_max_ok);
		return 0;
	}
	if (regs->src_cfg & PPP_SRC_PLANE_PSEUDOPLNR) {
		if (regs->src1 < src_min_ok || regs->src1 > src_max_ok ||
		    regs->src1 + src1_len > src_max_ok) {
			DLOG("invalid_src1");
			return 0;
		}
	}
	if (regs->dst0 < dst_min_ok || regs->dst0 > dst_max_ok ||
	    regs->dst0 + dst0_len > dst_max_ok) {
		DLOG("invalid_dst");
		return 0;
	}
	if (regs->dst_cfg & PPP_SRC_PLANE_PSEUDOPLNR) {
		if (regs->dst1 < dst_min_ok || regs->dst1 > dst_max_ok ||
		    regs->dst1 + dst1_len > dst_max_ok) {
			DLOG("invalid_dst1");
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
			if (get_pmem_file(img->memory_id, start, len))
				return -1;
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

static void flush_imgs(struct mdp_blit_req *req, struct mdp_regs *regs)
{
	uint32_t src0_len, src1_len, dst0_len, dst1_len;

	if (req->src.memory_type == PMEM_IMG) {
		get_len(&req->src, &req->src_rect, regs->src_bpp, &src0_len,
			 &src1_len);
		flush_pmem_file(req->src.memory_id, req->src.offset, src0_len);
		if (regs->src_cfg & PPP_SRC_PLANE_PSEUDOPLNR)
			flush_pmem_file(req->src.memory_id,
					req->src.offset + src0_len,
					src1_len);
	}
	if (req->dst.memory_type == PMEM_IMG) {
		get_len(&req->dst, &req->dst_rect, regs->dst_bpp, &dst0_len, 
			 &dst1_len);
		flush_pmem_file(req->dst.memory_id, req->dst.offset, dst0_len);
		if (regs->dst_cfg & PPP_SRC_PLANE_PSEUDOPLNR)
			flush_pmem_file(req->dst.memory_id,
					req->dst.offset + dst0_len,
					dst1_len);
	}
}

#define WRITEL(v, a) do { writel(v,a); /*printk(#a "[%x]=%x\n", a, v);*/ }\
		     while (0)
int mdp_ppp_blit(struct fb_info *info, struct mdp_blit_req *req)
{
	struct mdp_regs regs;
	unsigned long src_start, src_len;
	unsigned long dst_start, dst_len;
	ktime_t t1, t2;

#if 0
	DLOG("BLIT recieved img %p\n"
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
	t1 = ktime_get();
	if (unlikely(get_img(&req->src, info, &src_start, &src_len)) ||
		    (get_img(&req->dst, info, &dst_start, &dst_len))) {
		printk("mpd_ppp: could not retrieve image from memory\n");
		return -1;
	}
	t2 = ktime_get();
	DLOG("get_img time %lld\n", ktime_to_ns(ktime_sub(t2, t1)));

	if (unlikely(req->src.format >= MDP_IMGTYPE_LIMIT ||
	    req->dst.format >= MDP_IMGTYPE_LIMIT)) {
		printk("mpd_ppp: img is of wrong format\n");
		return -EINVAL;
		}

	if (unlikely(req->src_rect.x > req->src.width ||
		     req->src_rect.y > req->src.height ||
		     req->dst_rect.x > req->dst.width ||
		     req->dst_rect.y > req->dst.height)) {
		printk("mpd_ppp: img rect is outside of img!\n");
		return -1;
	}

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

	if (!valid_src_dst(src_start, src_len, dst_start, dst_len, req, 
			   &regs)) {
		printk("mpd_ppp: final src or dst location is invalid, are you "
			"trying to make an image too large or to place it "
			"outside the screen?\n");
		return -1;
	}

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
	if (((req->rotation & MDP_ROT_90) &&
	    ((req->src_rect.w != req->dst_rect.h) ||
	     (req->src_rect.h != req->dst_rect.w))) ||
	    ((req->src_rect.w != req->dst_rect.w) ||
	     (req->src_rect.h != req->dst_rect.h))) {
		printk("mdp_ppp: src and destination rects don't match "
			"and scaling is not implemented %x %x %x %x %x\n",
			req->rotation & MDP_ROT_90,
			req->src_rect.w, req->src_rect.h,
			req->dst_rect.w, req->dst_rect.h);
		return -1;
		/* mdp_blt_scale(req);  */
	}
#endif


	if (unlikely(req->src.format == MDP_YCRYCB_H2V1)) {
		/* the x and w must be even */
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
	t1 = ktime_get();
	flush_imgs(req, &regs);
	t2 = ktime_get();
	DLOG("flush img time %lld\n", ktime_to_ns(ktime_sub(t2, t1)));
	WRITEL(0x1000, MDP_DISPLAY0_START);
	return 0;
}
