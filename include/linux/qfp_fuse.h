/*
 * Copyright (c) 2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 */

#ifndef _QFP_FUSE_H_
#define _QFP_FUSE_H_

#include <linux/types.h>
#include <linux/ioctl.h>

#define QFP_FUSE_IOC_MAGIC          0x92

#define QFP_FUSE_IOC_WRITE          _IO(QFP_FUSE_IOC_MAGIC, 1)
#define QFP_FUSE_IOC_READ           _IO(QFP_FUSE_IOC_MAGIC, 2)

/*
 * QFPROM size = 32K bytes
 */
#define QFPROM_BASE                 qfp_qfprom_base
#define QFPROM_END                  qfp_qfprom_end

/*
 * Time QFPROM requires to reliably burn a fuse.
 */
#define QFPROM_BLOW_TIMEOUT_US      10
#define QFPROM_BLOW_TIMER_OFFSET    (QFPROM_BASE + 0x2038)
/*
 * Denotes number of cycles required to blow the fuse.
 */
#define QFPROM_BLOW_TIMER_VALUE     (QFPROM_BLOW_TIMEOUT_US * 83)

#define QFPROM_BLOW_STATUS_OFFSET   (QFPROM_BASE + 0x204C)
#define QFPROM_BLOW_STATUS_BUSY     0x01
#define QFPROM_BLOW_STATUS_ERROR    0x02

#define PM8058_LVS0_REGULATOR       "8058_lvs0"

#define QFP_FUSE_READY              0x01
#define QFP_FUSE_OFF                0x00

/*
 * This structure is used to exchange the fuse parameters with the user
 * space application. The pointer to this structure is passed to the ioctl
 * function.
 * offset   = offset from the QFPROM base for the data to be read/written.
 * size     = number of 32-bit words to be read/written.
 * data     = pointer to the 32 bit word denoting userspace data.
 */
struct qfp_fuse_req {
    u32 offset;
    u32 size;
    u32 *data;
};

#endif
