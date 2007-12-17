/* arch/arm/mach-msm/rpc_pm.h
**
** Copyright (C) 2007 Google, Inc.
** Copyright (c) 2007 QUALCOMM Incorporated
**
** This software is licensed under the terms of the GNU General Public
** License version 2, as published by the Free Software Foundation, and
** may be copied, distributed, and modified under those terms.
**
** This program is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU General Public License for more details.
**
** Author: San Mehat <san@google.com>
**
*/

#ifndef __ARCH_ARM_MACH_MSM_RPC_PM_H
#define __ARCH_ARM_MACH_MSM_RPC_PM_H


/* ===========================
 * Low level RPC PM control
 * ===========================
 */
#define APP_PM_PROG 0x30000060
#define APP_PM_VER 0

#define PM_PROCEEDURE_NULL 0
#define PM_PROCEEDURE_SETPWRKEYDUR 1
#define PM_PROCEEDURE_VOTEVREGSWITCH 2

/*
 *  The RPC request/response structures aren't combined as one might
 *  stylistically hope because this data is derrived from XDR definitions.
 */

/*
 * PM_PROCEEDURE_VREGSWITCH
 */

typedef struct
{
	uint32_t	cmd; /* 0 == off, 1 = on */
	uint32_t	vreg_id;
	uint32_t	app_mask;
} pm_rpc_vregswitch_req_args;

typedef struct
{
        oncrpc_request_hdr hdr;
        pm_rpc_vregswitch_req_args args;
} pm_rpc_vregswitch_msg;

#endif
