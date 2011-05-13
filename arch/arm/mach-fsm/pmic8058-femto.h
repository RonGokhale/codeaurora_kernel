/* Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of Code Aurora Forum, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _PMIC8058_FEMTO_H_
#define _PMIC8058_FEMTO_H_

#include <linux/types.h>
#include <linux/ioctl.h>

#define PM8058_FEMTO_IOC_MAGIC       0x93

#define PM8058_FEMTO_IOC_1V25		_IO(PM8058_FEMTO_IOC_MAGIC, 1)
#define PM8058_FEMTO_IOC_DIE_TEMP	_IO(PM8058_FEMTO_IOC_MAGIC, 2)
#define PM8058_FEMTO_IOC_THERM_PA	_IO(PM8058_FEMTO_IOC_MAGIC, 3)
#define PM8058_FEMTO_IOC_THERM_FSM	_IO(PM8058_FEMTO_IOC_MAGIC, 4)
#define PM8058_FEMTO_IOC_THERM_VCTCXO	_IO(PM8058_FEMTO_IOC_MAGIC, 5)
#define PM8058_FEMTO_IOC_CLKBUF		_IO(PM8058_FEMTO_IOC_MAGIC, 6)
#define PM8058_FEMTO_IOC_BOARD_RESET	_IO(PM8058_FEMTO_IOC_MAGIC, 7)

#define PMIC_DEVICE_READY           0x01
#define PMIC_DEVICE_OFF             0x00

#define FEMTO_GPIO_PS_HOLD          161

/* enum for TCXO clock output buffer definition */
enum clk_buffer_type {
    XO_BUFFER_A0 = 0,
    XO_BUFFER_A1 = 1,
    XO_BUFFER_LAST
};

/*
 * This user request structure is used to exchange the pmic device data
 * requested to user space applications.  The pointer to this structure is
 * passed to the the ioctl function.
 * data     = pointer to the 16 bit word denoting user space data.
*/
struct pm8058_femto_req {
	u16                    *data;
	enum clk_buffer_type   clkBuffer;
	u8                     clkBufEnable;
};

#endif
