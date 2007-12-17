/* include/asm-arm/arch-msm/rpc_clkctl.h
 *
 * Public interface for RPC clock control
 *
 * Copyright (c) 2007 QUALCOMM Incorporated
 * Copyright (C) 2007 Google, Inc.
 * Author: San Mehat <san@android.com>
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

#ifndef _INCLUDE_ASM_ARM_ARCH_MSM_RPC_CLKCTL_H
#define _INCLUDE_ASM_ARM_ARCH_MSM_RPC_CLKCTL_H

struct clkctl_rpc_ops
{
	int  (*enable)(uint32_t clk);
	int (*disable)(uint32_t clk);
	int  (*reset)(uint32_t clk);
	int  (*set_flags)(uint32_t clk, uint32_t flags);
	int  (*set_rate)(uint32_t clk, uint32_t rate);
	int  (*set_min_rate)(uint32_t clk, uint32_t rate);
	int  (*set_max_rate)(uint32_t clk, uint32_t rate);
	uint32_t (*get_rate)(uint32_t clk);
	int (*pll_request)(uint32_t pll, int enable);
};

#endif
