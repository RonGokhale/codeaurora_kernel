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
#ifndef __PMIC8058_NFC_H__
#define __PMIC8058_NFC_H__

struct pm8058_nfc_device;

/* masks, flags and status */
#define	PM_NFC_VDDLDO_MON_LEVEL		0x0003
#define	PM_NFC_VPH_PWR_EN		0x0008
#define	PM_NFC_EXT_VDDLDO_EN		0x0010
#define	PM_NFC_EN			0x0020
#define	PM_NFC_LDO_EN			0x0040
#define	PM_NFC_SUPPORT_EN		0x0080

#define	PM_NFC_EXT_EN_HIGH		0x0100
#define	PM_NFC_MBG_EN_HIGH		0x0200
#define	PM_NFC_VDDLDO_OK_HIGH		0x0400
#define	PM_NFC_DTEST1_MODE		0x2000
#define	PM_NFC_ATEST_EN			0x4000
#define	PM_NFC_VDDLDO_MON_EN		0x8000

#define	PM_NFC_CTRL_REQ			(PM_NFC_SUPPORT_EN |\
					PM_NFC_LDO_EN |\
					PM_NFC_EN |\
					PM_NFC_EXT_VDDLDO_EN |\
					PM_NFC_VPH_PWR_EN |\
					PM_NFC_VDDLDO_MON_LEVEL)

#define	PM_NFC_TEST_REQ			(PM_NFC_VDDLDO_MON_EN |\
					PM_NFC_DTEST1_MODE |\
					PM_NFC_ATEST_EN)

#define	PM_NFC_TEST_STATUS		(PM_NFC_EXT_EN_HIGH |\
					PM_NFC_MBG_EN_HIGH |\
					PM_NFC_VDDLDO_OK_HIGH)

/*
 * pm8058_nfc_request - request a handle to access NFC device
 */
struct pm8058_nfc_device *pm8058_nfc_request(void);

/*
 * pm8058_nfc_config - configure NFC signals
 *
 * @nfcdev: the NFC device
 * @mask: signal mask to configure
 * @flags: control flags
 */
int pm8058_nfc_config(struct pm8058_nfc_device *nfcdev, u32 mask, u32 flags);

/*
 * pm8058_nfc_get_status - get NFC status
 *
 * @nfcdev: the NFC device
 * @mask: of status mask to read
 * @status: pointer to the status variable
 */
int pm8058_nfc_get_status(struct pm8058_nfc_device *nfcdev,
			  u32 mask, u32 *status);

/*
 * pm8058_nfc_free - free the NFC device
 */
void pm8058_nfc_free(struct pm8058_nfc_device *nfcdev);

#endif /* __PMIC8058_NFC_H__ */
