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
#define RPCCLKCTL_CLIENT_ID 0x80000001
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
typedef struct
{
	uint32_t clock;
} clkctl_rpc_getrate_req_args;

typedef struct
{
	uint32_t rate;
} clkctl_rpc_getrate_rets;

typedef struct
{
	oncrpc_request_hdr hdr;
	clkctl_rpc_getrate_req_args args;
} clkctl_rpc_getrate_msg;


/*
 * CLKCTL_PROCEEDURE_SETRATE
 */

typedef struct
{
	uint32_t clock;
	uint32_t rate;
} clkctl_rpc_setrate_req_args;

typedef struct
{
	int result;
} clkctl_rpc_setrate_rets;

typedef struct
{
	oncrpc_request_hdr hdr;
	clkctl_rpc_setrate_req_args args;
} clkctl_rpc_setrate_msg;

/*
 * CLKCTL_PROCEEDURE_ENABLE
 */
typedef struct
{
	uint32_t clock;
} clkctl_rpc_enable_req_args;

typedef struct
{
	oncrpc_request_hdr hdr;
	clkctl_rpc_enable_req_args args;
} clkctl_rpc_enable_msg;

/*
 * CLKCTL_PROCEEDURE_DISABLE
 */
typedef struct
{
	uint32_t clock;
} clkctl_rpc_disable_req_args;

typedef struct
{
	int result;
} clkctl_rpc_disable_rets;

typedef struct
{
	oncrpc_request_hdr hdr;
	clkctl_rpc_disable_req_args args;
} clkctl_rpc_disable_msg;

/*
 * CLKCTL_PROCEEDURE_PLLREQUEST
 */
typedef struct
{
	uint32_t pll;
	int enable;

} clkctl_rpc_pllrequest_req_args;

typedef struct
{
	oncrpc_request_hdr hdr;
	clkctl_rpc_pllrequest_req_args args;
} clkctl_rpc_pllrequest_msg;
#endif
