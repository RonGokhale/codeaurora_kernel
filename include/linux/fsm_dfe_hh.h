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

#ifndef _FSM_DFE_HH_H_
#define _FSM_DFE_HH_H_

#include <linux/ioctl.h>

/*
 * Device interface
 */

#define DFE_HH_DEVICE_NAME		"dfe_hh"

/*
 * IOCTL interface
 */

enum {
	DFE_IOCTL_COMMAND_CODE_WRITE,
	DFE_IOCTL_COMMAND_CODE_WRITE_WITH_MASK,
};

struct dfe_write_register_param {
	unsigned int offset;
	unsigned int value;
};

struct dfe_write_register_mask_param {
	unsigned int offset;
	unsigned int value;
	unsigned int mask;
};

struct dfe_read_write_array_param {
	unsigned int offset;
	unsigned int num; /* number of 16 bit registers */
	unsigned int *pArray;
};

struct dfe_command_entry {
	unsigned int code;
	unsigned int offset;
	unsigned int value;
	unsigned int mask; /* DFE_IOCTL_COMMAND_CODE_WRITE_WITH_MASK only */
};

struct dfe_command_param {
	unsigned int num;
	struct dfe_command_entry *pEntry;
};

#define DFE_IOCTL_MAGIC				'h'
#define DFE_IOCTL_IS_UMTS \
	_IOC(_IOC_READ, DFE_IOCTL_MAGIC, 0x00, \
		0)
#define DFE_IOCTL_READ_REGISTER \
	_IOC(_IOC_READ, DFE_IOCTL_MAGIC, 0x01, \
		sizeof(unsigned int *))
#define DFE_IOCTL_WRITE_REGISTER \
	_IOC(_IOC_WRITE, DFE_IOCTL_MAGIC, 0x02, \
		sizeof(struct dfe_write_register_param *))
#define DFE_IOCTL_WRITE_REGISTER_WITH_MASK \
	_IOC(_IOC_WRITE, DFE_IOCTL_MAGIC, 0x03, \
		sizeof(struct dfe_write_register_mask_param *))
#define DFE_IOCTL_READ_REGISTER_ARRAY \
	_IOC(_IOC_READ, DFE_IOCTL_MAGIC, 0x04, \
		sizeof(struct dfe_read_write_array_param *))
#define DFE_IOCTL_WRITE_REGISTER_ARRAY \
	_IOC(_IOC_WRITE, DFE_IOCTL_MAGIC, 0x05, \
		sizeof(struct dfe_read_write_array_param *))
#define DFE_IOCTL_COMMAND \
	_IOC(_IOC_WRITE, DFE_IOCTL_MAGIC, 0x10, \
		sizeof(struct dfe_command_param *))

#endif /* _FSM_DFE_HH_H_ */
