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
#ifndef __LINUX_MFD_MSM_MARIMBA_CODEC_H
#define __LINUX_MFD_MSM_MARIMBA_CODEC_H

#include <mach/qdsp5v2/adie_marimba.h>

struct adie_codec_register {
	u8 reg;
	u8 mask;
	u8 val;
};

struct adie_codec_register_image {
	struct adie_codec_register *regs;
	u32 img_sz;
};

struct adie_codec_path {
	struct adie_codec_dev_profile *profile;
	struct adie_codec_register_image img;
	u32 hwsetting_idx;
	u32 stage_idx;
	u32 curr_stage;
};

int adie_codec_open(struct adie_codec_dev_profile *profile,
	struct adie_codec_path **path_pptr);
int adie_codec_setpath(struct adie_codec_path *path_ptr,
	u32 freq_plan, u32 osr);
int adie_codec_proceed_stage(struct adie_codec_path *path_ptr, u32 state);
int adie_codec_close(struct adie_codec_path *path_ptr);
u32 adie_codec_freq_supported(struct adie_codec_dev_profile *profile,
							u32 requested_freq);
int adie_codec_enable_sidetone(struct adie_codec_path *rx_path_ptr, u32 enable);

int adie_codec_set_device_digital_volume(struct adie_codec_path *path_ptr,
		u32 num_channels, u32 vol_percentage /* in percentage */);

int adie_codec_set_device_analog_volume(struct adie_codec_path *path_ptr,
		u32 num_channels, u32 volume /* in percentage */);
#endif
