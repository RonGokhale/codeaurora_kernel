/*===========================================================================
FILE:
   structs.h

DESCRIPTION:
   Declaration of structures used by the Qualcomm Linux USB Network driver
   
FUNCTIONS:
   none

Copyright (c) 2010, Code Aurora Forum. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of Code Aurora Forum nor
      the names of its contributors may be used to endorse or promote
      products derived from this software without specific prior written
      permission.

Alternatively, provided that this notice is retained in full, this software
may be relicensed by the recipient under the terms of the GNU General Public
License version 2 ("GPL") and only version 2, in which case the provisions of
the GPL apply INSTEAD OF those given above.  If the recipient relicenses the
software under the GPL, then the identification text in the MODULE_LICENSE
macro must be changed to reflect "GPLv2" instead of "Dual BSD/GPL".  Once a
recipient changes the license terms to the GPL, subsequent recipients shall
not relicense under alternate licensing terms, including the BSD or dual
BSD/GPL terms.  In addition, the following license statement immediately
below and between the words START and END shall also then apply when this
software is relicensed under the GPL:

START

This program is free software; you can redistribute it and/or modify it under
the terms of the GNU General Public License version 2 and only version 2 as
published by the Free Software Foundation.

This program is distributed in the hope that it will be useful, but WITHOUT
ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
details.

You should have received a copy of the GNU General Public License along with
this program; if not, write to the Free Software Foundation, Inc.,
51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.

END

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
===========================================================================*/

#ifndef QCUSBNET_STRUCTS_H
#define QCUSBNET_STRUCTS_H

#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/mii.h>
#include <linux/usb.h>
#include <linux/version.h>
#include <linux/cdev.h>
#include <linux/kthread.h>

#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,24)
	#include "usbnet.h"
#else
	#include <linux/usb/usbnet.h>
#endif

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,25)
	#include <linux/fdtable.h>
#else
	#include <linux/file.h>
#endif

#define DBG(format, arg...) \
	if (debug == 1) { \
		printk(KERN_INFO "QCUSBNet2k::%s " format, __FUNCTION__, ## arg); \
	}

struct qcusbnet;

struct urbreq {
	struct list_head node;
	struct urb *urb;
};

#define DEFAULT_READ_URB_LENGTH 0x1000

struct worker {
	struct task_struct *thread;
	struct completion work;
	struct list_head urbs;
	spinlock_t urbs_lock;
	struct urb *active;
	spinlock_t active_lock;
	struct usb_interface *iface;
};

struct qmidev {
	dev_t devnum;
	struct class *devclass;
	struct cdev cdev;
	struct urb *readurb;
	struct urbsetup *readsetup;
	void *readbuf;
	struct urb *inturb;
	void *intbuf;
	struct list_head clients;
	spinlock_t clients_lock;
	atomic_t qmitid;
};

enum {
	DOWN_NO_NDIS_CONNECTION = 0,
	DOWN_CDC_CONNECTION_SPEED = 1,
	DOWN_DRIVER_SUSPENDED = 2,
	DOWN_NET_IFACE_STOPPED = 3,
};

struct qcusbnet {
	struct usbnet *usbnet;
	struct usb_interface *iface;
	int (*open)(struct net_device *);
	int (*stop)(struct net_device *);
	unsigned long down;
	bool valid;
	struct qmidev qmi;
	char meid[14];
	struct worker worker;
};

#endif /* !QCUSBNET_STRUCTS_H */
