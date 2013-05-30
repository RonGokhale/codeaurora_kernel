/* Copyright (c) 2013, The Linux Foundation. All rights reserved.
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
 */

#ifndef _H_VPU_TRANSLATE_H_
#define _H_VPU_TRANSLATE_H_

#include <linux/videodev2.h>

#include "vpu_v4l2.h"
#include "vpu_property.h"

/*
 * Translations between API parameters and HFI configuration parameters
 */

enum translation_direction {
	API_2_HFI = 0,
	HFI_2_API,
};

static inline u32 translate_input_source(unsigned int in)
{
	return in;
}
static inline u32 translate_output_destination(unsigned int out)
{
	return out;
}

u32 trans_pixelformat_to_hfi(u32 api_pix_fmt);
u32 trans_pixelformat_to_api(u32 hfi_pix_fmt);

void translate_input_format_to_hfi(const struct vpu_port_info *port_info,
		struct vpu_prop_session_input *in);
void translate_output_format_to_hfi(const struct vpu_port_info *port_info,
		struct vpu_prop_session_output *out);

void translate_input_format_to_api(const struct vpu_prop_session_input *in,
		struct v4l2_format *fmt);
void translate_output_format_to_api(const struct vpu_prop_session_output *out,
		struct v4l2_format *fmt);

u32 translate_timeperframe_to_xfps(const struct v4l2_fract *timeperframe);
void translate_xfps_to_timeperframe(u32 framerate,
		struct v4l2_fract *timeperframe);

void translate_roi_rect_to_hfi(const struct v4l2_rect *crop,
		struct rect *roi);
void translate_roi_rect_to_api(const struct rect *roi,
		struct v4l2_rect *crop);

static inline u32 translate_3d_mode_to_hfi(u32 api_mode)
{
	return api_mode;
}
static inline u32 translate_3d_mode_to_api(u32 hfi_mode)
{
	return hfi_mode;
}

u32 translate_field_to_hfi(enum v4l2_field api_field);
u32 translate_field_to_api(u32 hfi_field);

u32 translate_v4l2_scan_mode(enum v4l2_field field);

#endif /* _H_VPU_TRANSLATE_H_ */
