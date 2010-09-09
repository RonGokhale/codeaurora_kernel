/*===========================================================================
FILE:
   qmi.h

DESCRIPTION:
   Qualcomm QMI driver header
   
FUNCTIONS:
   Generic QMUX functions
      qmux_parse
      qmux_fill

   Get sizes of buffers needed by QMI requests
      QMIWDSGetPKGSRVCStatusReqSize
      QMIDMSGetMEIDReqSize

   Fill Buffers with QMI requests
      QMIWDSGetPKGSRVCStatusReq
      QMIDMSGetMEIDReq
      
   Parse data from QMI responses
      qmictl_alloccid_resp
      qmictl_freecid_resp
      qmiwds_event_resp
      qmidms_meid_resp

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

#ifndef QCUSBNET_QMI_H
#define QCUSBNET_QMI_H

#include <linux/types.h>

#define QMICTL 0
#define QMIWDS 1
#define QMIDMS 2

#define true      1
#define false     0

#define ENOMEM    12
#define EFAULT    14
#define EINVAL    22
#define ENOMSG    42
#define ENODATA   61

int qmux_parse(u16 *cid, void *buf, size_t size);
int qmux_fill(u16 cid, void *buf, size_t size);

extern const size_t qmux_size;

void *qmictl_new_getcid(u8 tid, u8 svctype, size_t *size);
void *qmictl_new_releasecid(u8 tid, u16 cid, size_t *size);
void *qmictl_new_ready(u8 tid, size_t *size);
void *qmiwds_new_seteventreport(u8 tid, size_t *size);
void *qmiwds_new_getpkgsrvcstatus(u8 tid, size_t *size);
void *qmidms_new_getmeid(u8 tid, size_t *size);

struct qmiwds_stats {
	u32 txok;
	u32 rxok;
	u32 txerr;
	u32 rxerr;
	u32 txofl;
	u32 rxofl;
	u64 txbytesok;
	u64 rxbytesok;
	bool linkstate;
	bool reconfigure;
};

int qmictl_alloccid_resp(void *buf, u16 size, u16 *cid);
int qmictl_freecid_resp(void *buf, u16 size);
int qmiwds_event_resp(void *buf, u16 size, struct qmiwds_stats *stats);
int qmidms_meid_resp(void *buf, u16 size, char *meid, int meidsize);

#endif /* !QCUSBNET_QMI_H */
