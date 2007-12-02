/* drivers/video/msm_fb/mdp_scale_tables.h
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

static struct {
	uint32_t reg;
	uint32_t val;
} mdp_upscale_table[] = {
	{ MSM_MDP_BASE + 0x5fffc, 0x0 },
	{ MSM_MDP_BASE + 0x50200, 0x7fc00000 },
	{ MSM_MDP_BASE + 0x5fffc, 0xff80000d },
	{ MSM_MDP_BASE + 0x50204, 0x7ec003f9 },
	{ MSM_MDP_BASE + 0x5fffc, 0xfec0001c },
	{ MSM_MDP_BASE + 0x50208, 0x7d4003f3 },
	{ MSM_MDP_BASE + 0x5fffc, 0xfe40002b },
	{ MSM_MDP_BASE + 0x5020c, 0x7b8003ed },
	{ MSM_MDP_BASE + 0x5fffc, 0xfd80003c },
	{ MSM_MDP_BASE + 0x50210, 0x794003e8 },
	{ MSM_MDP_BASE + 0x5fffc, 0xfcc0004d },
	{ MSM_MDP_BASE + 0x50214, 0x76c003e4 },
	{ MSM_MDP_BASE + 0x5fffc, 0xfc40005f },
	{ MSM_MDP_BASE + 0x50218, 0x73c003e0 },
	{ MSM_MDP_BASE + 0x5fffc, 0xfb800071 },
	{ MSM_MDP_BASE + 0x5021c, 0x708003de },
	{ MSM_MDP_BASE + 0x5fffc, 0xfac00085 },
	{ MSM_MDP_BASE + 0x50220, 0x6d0003db },
	{ MSM_MDP_BASE + 0x5fffc, 0xfa000098 },
	{ MSM_MDP_BASE + 0x50224, 0x698003d9 },
	{ MSM_MDP_BASE + 0x5fffc, 0xf98000ac },
	{ MSM_MDP_BASE + 0x50228, 0x654003d8 },
	{ MSM_MDP_BASE + 0x5fffc, 0xf8c000c1 },
	{ MSM_MDP_BASE + 0x5022c, 0x610003d7 },
	{ MSM_MDP_BASE + 0x5fffc, 0xf84000d5 },
	{ MSM_MDP_BASE + 0x50230, 0x5c8003d7 },
	{ MSM_MDP_BASE + 0x5fffc, 0xf7c000e9 },
	{ MSM_MDP_BASE + 0x50234, 0x580003d7 },
	{ MSM_MDP_BASE + 0x5fffc, 0xf74000fd },
	{ MSM_MDP_BASE + 0x50238, 0x534003d8 },
	{ MSM_MDP_BASE + 0x5fffc, 0xf6c00112 },
	{ MSM_MDP_BASE + 0x5023c, 0x4e8003d8 },
	{ MSM_MDP_BASE + 0x5fffc, 0xf6800126 },
	{ MSM_MDP_BASE + 0x50240, 0x494003da },
	{ MSM_MDP_BASE + 0x5fffc, 0xf600013a },
	{ MSM_MDP_BASE + 0x50244, 0x448003db },
	{ MSM_MDP_BASE + 0x5fffc, 0xf600014d },
	{ MSM_MDP_BASE + 0x50248, 0x3f4003dd },
	{ MSM_MDP_BASE + 0x5fffc, 0xf5c00160 },
	{ MSM_MDP_BASE + 0x5024c, 0x3a4003df },
	{ MSM_MDP_BASE + 0x5fffc, 0xf5c00172 },
	{ MSM_MDP_BASE + 0x50250, 0x354003e1 },
	{ MSM_MDP_BASE + 0x5fffc, 0xf5c00184 },
	{ MSM_MDP_BASE + 0x50254, 0x304003e3 },
	{ MSM_MDP_BASE + 0x5fffc, 0xf6000195 },
	{ MSM_MDP_BASE + 0x50258, 0x2b0003e6 },
	{ MSM_MDP_BASE + 0x5fffc, 0xf64001a6 },
	{ MSM_MDP_BASE + 0x5025c, 0x260003e8 },
	{ MSM_MDP_BASE + 0x5fffc, 0xf6c001b4 },
	{ MSM_MDP_BASE + 0x50260, 0x214003eb },
	{ MSM_MDP_BASE + 0x5fffc, 0xf78001c2 },
	{ MSM_MDP_BASE + 0x50264, 0x1c4003ee },
	{ MSM_MDP_BASE + 0x5fffc, 0xf80001cf },
	{ MSM_MDP_BASE + 0x50268, 0x17c003f1 },
	{ MSM_MDP_BASE + 0x5fffc, 0xf90001db },
	{ MSM_MDP_BASE + 0x5026c, 0x134003f3 },
	{ MSM_MDP_BASE + 0x5fffc, 0xfa0001e5 },
	{ MSM_MDP_BASE + 0x50270, 0xf0003f6 },
	{ MSM_MDP_BASE + 0x5fffc, 0xfb4001ee },
	{ MSM_MDP_BASE + 0x50274, 0xac003f9 },
	{ MSM_MDP_BASE + 0x5fffc, 0xfcc001f5 },
	{ MSM_MDP_BASE + 0x50278, 0x70003fb },
	{ MSM_MDP_BASE + 0x5fffc, 0xfe4001fb },
	{ MSM_MDP_BASE + 0x5027c, 0x34003fe },
};
