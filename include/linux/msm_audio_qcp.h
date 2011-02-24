/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
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

#ifndef __MSM_AUDIO_QCP_H
#define __MSM_AUDIO_QCP_H

#include <linux/msm_audio.h>

#define AUDIO_SET_QCELP_ENC_CONFIG  _IOW(AUDIO_IOCTL_MAGIC, \
	0, struct msm_audio_qcelp_enc_config)

#define AUDIO_GET_QCELP_ENC_CONFIG  _IOR(AUDIO_IOCTL_MAGIC, \
	1, struct msm_audio_qcelp_enc_config)

#define AUDIO_SET_EVRC_ENC_CONFIG  _IOW(AUDIO_IOCTL_MAGIC, \
	2, struct msm_audio_evrc_enc_config)

#define AUDIO_GET_EVRC_ENC_CONFIG  _IOR(AUDIO_IOCTL_MAGIC, \
	3, struct msm_audio_evrc_enc_config)

#define CDMA_RATE_BLANK		0x00
#define CDMA_RATE_EIGHTH	0x01
#define CDMA_RATE_QUARTER	0x02
#define CDMA_RATE_HALF		0x03
#define CDMA_RATE_FULL		0x04
#define CDMA_RATE_ERASURE	0x05

struct msm_audio_qcelp_enc_config {
	uint32_t cdma_rate;
	uint32_t min_bit_rate;
	uint32_t max_bit_rate;
};

struct msm_audio_evrc_enc_config {
	uint32_t cdma_rate;
	uint32_t min_bit_rate;
	uint32_t max_bit_rate;
};

#endif /* __MSM_AUDIO_QCP_H */
