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

#ifndef __MSM_AUDIO_SBC_H
#define __MSM_AUDIO_SBC_H

#include <linux/msm_audio.h>

#define AUDIO_SET_SBC_ENC_CONFIG  _IOW(AUDIO_IOCTL_MAGIC, \
  (AUDIO_MAX_COMMON_IOCTL_NUM+0), struct msm_audio_sbc_enc_config)

#define AUDIO_GET_SBC_ENC_CONFIG  _IOR(AUDIO_IOCTL_MAGIC, \
  (AUDIO_MAX_COMMON_IOCTL_NUM+1), struct msm_audio_sbc_enc_config)

#define AUDIO_SBC_BA_LOUDNESS		0x0
#define AUDIO_SBC_BA_SNR		0x1

#define AUDIO_SBC_MODE_MONO		0x0
#define AUDIO_SBC_MODE_DUAL		0x1
#define AUDIO_SBC_MODE_STEREO		0x2
#define AUDIO_SBC_MODE_JSTEREO		0x3

#define AUDIO_SBC_BANDS_8		0x1

#define AUDIO_SBC_BLOCKS_4		0x0
#define AUDIO_SBC_BLOCKS_8		0x1
#define AUDIO_SBC_BLOCKS_12		0x2
#define AUDIO_SBC_BLOCKS_16		0x3

struct msm_audio_sbc_enc_config {
	uint32_t channels;
	uint32_t sample_rate;
	uint32_t bit_allocation;
	uint32_t number_of_subbands;
	uint32_t number_of_blocks;
	uint32_t bit_rate;
	uint32_t mode;
};
#endif /* __MSM_AUDIO_SBC_H */
