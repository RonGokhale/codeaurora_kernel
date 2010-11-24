/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
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
 *
 */

#ifndef _FSM_RFIC_FTR_H_
#define _FSM_RFIC_FTR_H_

#include <linux/ioctl.h>

/*
 * Device interface
 */

#define RFIC_FTR_DEVICE_NAME		"rfic_ftr"

/*
 * IOCTL interface
 */

/*!
    Macro to associate the "bus" and "address" pair when accessing the RFIC.
    Using a 32 bit address, reserve the upper 8 bits for the bus value, and
    the lower 24 bits for the address.
 */
#define RFIC_FTR_ADDR(bus, addr)	(((bus&0x03)<<24)|(addr&0xFFFFFF))
#define RFIC_FTR_GET_ADDR(busAddr)	(busAddr&0xFFFFFF)
#define RFIC_FTR_GET_BUS(busAddr)	((busAddr>>24)&0x03)

struct rfic_write_register_param {
	unsigned int rficAddr;
	unsigned int value;
};

struct rfic_write_register_mask_param {
	unsigned int rficAddr;
	unsigned int value;
	unsigned int mask;
};

struct rfic_grfc_param {
	unsigned int grfcId;
	unsigned int maskValue;
	unsigned int ctrlValue;
};

#define RFIC_IOCTL_MAGIC				'f'
#define RFIC_IOCTL_IS_UMTS \
	_IOC(_IOC_READ, RFIC_IOCTL_MAGIC, 0x00, \
		0)
#define RFIC_IOCTL_READ_REGISTER \
	_IOC(_IOC_READ, RFIC_IOCTL_MAGIC, 0x01, \
		sizeof(unsigned int *))
#define RFIC_IOCTL_WRITE_REGISTER \
	_IOC(_IOC_WRITE, RFIC_IOCTL_MAGIC, 0x02, \
		sizeof(struct rfic_write_register_param *))
#define RFIC_IOCTL_WRITE_REGISTER_WITH_MASK \
	_IOC(_IOC_WRITE, RFIC_IOCTL_MAGIC, 0x03, \
		sizeof(struct rfic_write_register_mask_param *))
#define RFIC_IOCTL_GET_GRFC \
	_IOC(_IOC_WRITE, RFIC_IOCTL_MAGIC, 0x10, \
		sizeof(struct rfic_grfc_param *))
#define RFIC_IOCTL_SET_GRFC \
	_IOC(_IOC_WRITE, RFIC_IOCTL_MAGIC, 0x11, \
		sizeof(struct rfic_grfc_param *))

#endif /* _FSM_RFIC_FTR_H_ */
