/* arch/arm/mach-msm/rpc_clkctl.h
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2007 QUALCOMM Incorporated
 * Author: San Mehat <san@android.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 */

#ifndef __ARCH_ARM_MACH_MSM_RPC_CLKCTL_H
#define __ARCH_ARM_MACH_MSM_RPC_CLKCTL_H


/* ===========================
 * Low level RPC clock control
 * ===========================
 */
#define APP_CLKCTL_PDEV_NAME "rpcsvr_30000075:0"
#define APP_CLKCTL_PROG 0x30000075
#define APP_CLKCTL_VER 0

#define CLKCTL_PROCEEDURE_NULL 0
#define CLKCTL_PROCEEDURE_ENABLE 1
#define CLKCTL_PROCEEDURE_DISABLE 2
#define CLKCTL_PROCEEDURE_RESET 3
#define CLKCTL_PROCEEDURE_SETFLAGS 4
#define CLKCTL_PROCEEDURE_SETRATE 5
#define CLKCTL_PROCEEDURE_SETMINRATE 6
#define CLKCTL_PROCEEDURE_SETMAXRATE 7
#define CLKCTL_PROCEEDURE_GETRATE 8
#define CLKCTL_PROCEEDURE_PLLREQUEST 9

/*
 *  The RPC request/response structures aren't combined as one might
 *  stylistically hope because this data is derrived from XDR definitions.
 */

/*
 * CLKCTL_PROCEEDURE_GETRATE
 */
struct clkctlrpc_getrate_req
{
	struct rpc_request_hdr hdr;
	uint32_t clock;
};

struct clkctlrpc_getrate_rsp
{
	struct rpc_reply_hdr hdr;
	uint32_t rate;
};

/*
 * CLKCTL_PROCEEDURE_SETRATE
 */

struct clkctlrpc_setrate_req
{
	struct rpc_request_hdr hdr;
	uint32_t clock;
	uint32_t rate;
};

struct clkctlrpc_setrate_rsp
{
	struct rpc_reply_hdr hdr;
	int result;
};

/*
 * CLKCTL_PROCEEDURE_ENABLE
 */

struct clkctlrpc_enable_req
{
	struct rpc_request_hdr hdr;
	uint32_t clock;
};

/*
 * CLKCTL_PROCEEDURE_DISABLE
 */

struct clkctlrpc_disable_req
{
	struct rpc_request_hdr hdr;
	uint32_t clock;
};

/*
 * CLKCTL_PROCEEDURE_PLLREQUEST
 */

struct clkctlrpc_pllrequest_req
{
	struct rpc_request_hdr hdr;
	uint32_t pll;
	int enable;
};

#endif
