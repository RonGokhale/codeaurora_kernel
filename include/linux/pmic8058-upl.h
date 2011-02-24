/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, and the entire permission notice in its entirety,
 *    including the disclaimer of warranties.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote
 *    products derived from this software without specific prior
 *    written permission.
 *
 * ALTERNATIVELY, this product may be distributed under the terms of
 * the GNU General Public License, version 2, in which case the provisions
 * of the GPL version 2 are required INSTEAD OF the BSD license.
 *
 * THIS SOFTWARE IS PROVIDED ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE, ALL OF
 * WHICH ARE HEREBY DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 * USE OF THIS SOFTWARE, EVEN IF NOT ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 *
 */
#ifndef __PMIC8058_UPL_H__
#define __PMIC8058_UPL_H__

struct pm8058_upl_device;

/* control masks and flags */
#define PM8058_UPL_MOD_ENABLE_MASK	(0x10)
#define PM8058_UPL_MOD_ENABLE		(0x10)
#define PM8058_UPL_MOD_DISABLE		(0x00)

#define PM8058_UPL_OUT_DTEST_MASK	(0xE0)
#define PM8058_UPL_OUT_GPIO_ONLY	(0x00)
#define PM8058_UPL_OUT_DTEST_1		(0x80)
#define PM8058_UPL_OUT_DTEST_2		(0xA0)
#define PM8058_UPL_OUT_DTEST_3		(0xC0)
#define PM8058_UPL_OUT_DTEST_4		(0xE0)

#define PM8058_UPL_IN_A_MASK		(0x01)
#define PM8058_UPL_IN_A_GPIO		(0x00)
#define PM8058_UPL_IN_A_DTEST		(0x01)
#define PM8058_UPL_IN_B_MASK		(0x02)
#define PM8058_UPL_IN_B_GPIO		(0x00)
#define PM8058_UPL_IN_B_DTEST		(0x02)
#define PM8058_UPL_IN_C_MASK		(0x04)
#define PM8058_UPL_IN_C_GPIO		(0x00)
#define PM8058_UPL_IN_C_DTEST		(0x04)
#define PM8058_UPL_IN_D_MASK		(0x08)
#define PM8058_UPL_IN_D_GPIO		(0x00)
#define PM8058_UPL_IN_D_DTEST		(0x08)

/*
 * pm8058_upl_request - request a handle to access UPL device
 */
struct pm8058_upl_device *pm8058_upl_request(void);

int pm8058_upl_read_truthtable(struct pm8058_upl_device *upldev,
				u16 *truthtable);

int pm8058_upl_write_truthtable(struct pm8058_upl_device *upldev,
				u16 truthtable);

/*
 * pm8058_upl_config - configure UPL I/O settings and UPL enable/disable
 *
 * @upldev: the UPL device
 * @mask: setting mask to configure
 * @flags: setting flags
 */
int pm8058_upl_config(struct pm8058_upl_device *upldev, u32 mask, u32 flags);

#endif /* __PMIC8058_UPL_H__ */
