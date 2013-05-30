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

#include <linux/videodev2.h>
#include <media/msm_vpu.h>

#include "vpu_translate.h"
#include "vpu_property.h"
#include "vpu_debug.h"
#include "vpu_ipc.h"

/*
 * Translations between API parameters and HFI configuration parameters *
 */

static void __trans_resolution_to_hfi(const struct v4l2_pix_format_mplane *fmt,
		struct frame_resolution *resolution)
{
	resolution->width = fmt->width;
	resolution->height = fmt->height;
	dprintk(VPU_DBG, "%s received width = %d, height = %d\n",
			__func__, resolution->width, resolution->height);
}

static void __trans_resolution_to_api(const struct frame_resolution *resolution,
		struct v4l2_format *fmt)
{
	fmt->fmt.pix_mp.width = resolution->width;
	fmt->fmt.pix_mp.height = resolution->height;
}

static u32 __trans_scan_mode_to_hfi(enum v4l2_field field)
{
	u32 scan_mode;

	if (field == V4L2_FIELD_NONE || field == V4L2_FIELD_ANY)
		scan_mode = LINESCANPROGRESSIVE;
	else
		scan_mode = LINESCANINTERLACED;
	dprintk(VPU_DBG, "%s received scan mode = %d\n",
			__func__, scan_mode);

	return scan_mode;
}

static enum v4l2_field __trans_scan_mode_to_api(u32 scan_mode)
{
	enum v4l2_field field;

	if (scan_mode == LINESCANPROGRESSIVE)
		field = V4L2_FIELD_NONE;
	else if (scan_mode == LINESCANINTERLACED)
		field = V4L2_FIELD_INTERLACED;
	else
		field = 100; /* invalid */

	return field;
}

u32 trans_pixelformat_to_hfi(u32 api_pix_fmt)
{
	u32 hfi_pix_fmt;

	switch (api_pix_fmt) {
	case VPU_PIX_FMT_RGB888:
		hfi_pix_fmt = PIXEL_FORMAT_RGB888;
		break;
	case VPU_PIX_FMT_XRGB8888:
		hfi_pix_fmt = PIXEL_FORMAT_XRGB8888;
		break;
	case VPU_PIX_FMT_XRGB2:
		hfi_pix_fmt = PIXEL_FORMAT_XRGB2;
		break;
	case VPU_PIX_FMT_BGR888:
		hfi_pix_fmt = PIXEL_FORMAT_BGR888;
		break;
	case VPU_PIX_FMT_BGRX8888:
		hfi_pix_fmt = PIXEL_FORMAT_BGRX8888;
		break;
	case VPU_PIX_FMT_XBGR2:
		hfi_pix_fmt = PIXEL_FORMAT_XBGR2;
		break;
	case VPU_PIX_FMT_NV12:
		hfi_pix_fmt = PIXEL_FORMAT_NV12;
		break;
	case VPU_PIX_FMT_NV21:
		hfi_pix_fmt = PIXEL_FORMAT_NV21;
		break;
	case VPU_PIX_FMT_YUYV:
		hfi_pix_fmt = PIXEL_FORMAT_YUYV;
		break;
	case VPU_PIX_FMT_YVYU:
		hfi_pix_fmt = PIXEL_FORMAT_YVYU;
		break;
	case VPU_PIX_FMT_VYUY:
		hfi_pix_fmt = PIXEL_FORMAT_VYUY;
		break;
	case VPU_PIX_FMT_UYVY:
		hfi_pix_fmt = PIXEL_FORMAT_UYVY;
		break;
	case VPU_PIX_FMT_YUYV10:
		hfi_pix_fmt = PIXEL_FORMAT_YUYV_LOOSE;
		break;
	case VPU_PIX_FMT_YUV_COMP:
		hfi_pix_fmt = PIXEL_FORMAT_COMPRESSED_YUYV422;
		break;
	default:
		hfi_pix_fmt = PIXEL_FORMAT_MAX;
		dprintk(VPU_ERR,
			"%s, Unsupported api pixel format: %d (%4c)\n",
			__func__, api_pix_fmt,
			(char)api_pix_fmt);
		break;
	}

	dprintk(VPU_DBG, "%s received pixel format = %d\n",
		__func__, hfi_pix_fmt);

	return hfi_pix_fmt;
}

u32 trans_pixelformat_to_api(u32 hfi_pix_fmt)
{
	u32 api_pix_fmt;

	switch (hfi_pix_fmt) {
	case PIXEL_FORMAT_RGB888:
		api_pix_fmt = VPU_PIX_FMT_RGB888;
		break;
	case PIXEL_FORMAT_XRGB8888:
		api_pix_fmt = VPU_PIX_FMT_XRGB8888;
		break;
	case PIXEL_FORMAT_XRGB2:
		api_pix_fmt = VPU_PIX_FMT_XRGB2;
		break;
	case PIXEL_FORMAT_BGR888:
		api_pix_fmt = VPU_PIX_FMT_BGR888;
		break;
	case PIXEL_FORMAT_BGRX8888:
		api_pix_fmt = VPU_PIX_FMT_BGRX8888;
		break;
	case PIXEL_FORMAT_XBGR2:
		api_pix_fmt = VPU_PIX_FMT_XBGR2;
		break;
	case PIXEL_FORMAT_NV12:
		api_pix_fmt = VPU_PIX_FMT_NV12;
		break;
	case PIXEL_FORMAT_NV21:
		api_pix_fmt = VPU_PIX_FMT_NV21;
		break;
	case PIXEL_FORMAT_YUYV:
		api_pix_fmt = VPU_PIX_FMT_YUYV;
		break;
	case PIXEL_FORMAT_YVYU:
		api_pix_fmt = VPU_PIX_FMT_YVYU;
		break;
	case PIXEL_FORMAT_VYUY:
		api_pix_fmt = VPU_PIX_FMT_VYUY;
		break;
	case PIXEL_FORMAT_UYVY:
		api_pix_fmt = VPU_PIX_FMT_UYVY;
		break;
	case PIXEL_FORMAT_YUYV_LOOSE:
		api_pix_fmt = VPU_PIX_FMT_YUYV10;
		break;
	case PIXEL_FORMAT_COMPRESSED_YUYV422:
		api_pix_fmt = VPU_PIX_FMT_YUV_COMP;
		break;
	default:
		api_pix_fmt = 0;
		dprintk(VPU_ERR,
			"%s, Unsupported hfi pixel format: %d\n",
			__func__, hfi_pix_fmt);
		break;
	}

	return api_pix_fmt;
}

static void __trans_stride_to_hfi(const struct v4l2_pix_format_mplane *fmt,
		struct vpu_frame_info *frame_info)
{
	frame_info->plane0_stride = fmt->plane_fmt[0].bytesperline;
	frame_info->plane1_stride = fmt->plane_fmt[1].bytesperline;
	frame_info->plane2_stride = fmt->plane_fmt[2].bytesperline;
}

static void __trans_stride_to_api(const struct vpu_frame_info *frame_info,
		struct v4l2_format *fmt)
{
	fmt->fmt.pix_mp.plane_fmt[0].bytesperline =
			frame_info->plane0_stride;
	fmt->fmt.pix_mp.plane_fmt[1].bytesperline =
			frame_info->plane1_stride;
	fmt->fmt.pix_mp.plane_fmt[2].bytesperline =
			frame_info->plane2_stride;
}

void translate_input_format_to_hfi(const struct vpu_port_info *port_info,
		struct vpu_prop_session_input *in)
{
	memset(in, 0, sizeof(*in));

	__trans_resolution_to_hfi(&port_info->format,
			&in->frame_info.resolution);
	in->frame_info.pixel_format =
			trans_pixelformat_to_hfi(port_info->format.pixelformat);
	__trans_stride_to_hfi(&port_info->format, &in->frame_info);

	in->video_format = translate_3d_mode_to_hfi(port_info->video_fmt);
	in->input_source = translate_input_source(port_info->source);
	in->scan_mode = port_info->scan_mode;
	in->frame_rate = port_info->framerate;

	translate_roi_rect_to_hfi(&port_info->roi, &in->region_interest);

	in->flags = (port_info->secure_content) ?
			VPU_CHANNEL_FLAG_SECURE_CONTENT : 0;
}

void translate_output_format_to_hfi(const struct vpu_port_info *port_info,
		struct vpu_prop_session_output *out)
{
	memset(out, 0, sizeof(*out));

	__trans_resolution_to_hfi(&port_info->format,
			&out->frame_info.resolution);
	out->frame_info.pixel_format =
			trans_pixelformat_to_hfi(port_info->format.pixelformat);
	__trans_stride_to_hfi(&port_info->format, &out->frame_info);

	out->video_format = translate_3d_mode_to_hfi(port_info->video_fmt);
	out->output_dest = translate_input_source(port_info->destination);
	out->frame_rate = port_info->framerate;

	translate_roi_rect_to_hfi(&port_info->roi, &out->dest_rect);
	translate_roi_rect_to_hfi(&port_info->roi, &out->target_rect);

	out->flags = (port_info->secure_content) ?
			VPU_CHANNEL_FLAG_SECURE_CONTENT : 0;
}

void translate_input_format_to_api(const struct vpu_prop_session_input *in,
		struct v4l2_format *fmt)
{
	__trans_resolution_to_api(&in->frame_info.resolution, fmt);
	fmt->fmt.pix_mp.field = __trans_scan_mode_to_api(in->scan_mode);
	fmt->fmt.pix_mp.pixelformat =
			trans_pixelformat_to_api(in->frame_info.pixel_format);
	__trans_stride_to_api(&in->frame_info, fmt);
}

void translate_output_format_to_api(const struct vpu_prop_session_output *out,
		struct v4l2_format *fmt)
{
	__trans_resolution_to_api(&out->frame_info.resolution, fmt);
	fmt->fmt.pix_mp.pixelformat =
			trans_pixelformat_to_api(out->frame_info.pixel_format);
	__trans_stride_to_api(&out->frame_info, fmt);
}

u32 translate_timeperframe_to_xfps(const struct v4l2_fract *timeperframe)
{
	uint64_t temp;
	u32 framerate;
	if (timeperframe->denominator == 0 || timeperframe->numerator == 0) {
		framerate = 0;
	} else {
		/* store frame rate in 16.16 format */
		temp = timeperframe->denominator << 16;
		framerate = do_div(temp, timeperframe->numerator);
	}

	return framerate;
}

void translate_xfps_to_timeperframe(u32 framerate,
		struct v4l2_fract *timeperframe)
{
	timeperframe->numerator = 1 << 16;
	timeperframe->denominator = framerate;
}

void translate_roi_rect_to_hfi(const struct v4l2_rect *crop,
		struct rect *roi)
{
	roi->left = (u32) crop->left;
	roi->top = (u32) crop->top;
	roi->right = (u32) (crop->width + crop->left - 1);
	roi->bottom = (u32) (crop->top + crop->height - 1);
	dprintk(VPU_DBG,
			"%s received left = %d, top = %d, right = %d, bottom = %d\n",
			__func__, roi->left, roi->top, roi->right, roi->bottom);
}


void translate_roi_rect_to_api(const struct rect *roi,
		struct v4l2_rect *crop)
{
	crop->left = roi->left;
	crop->top = roi->top;
	crop->width = roi->right - roi->left + 1;
	crop->height = roi->bottom - roi->top + 1;
}

u32 translate_field_to_hfi(enum v4l2_field api_field)
{
	u32 hfi_field;

	switch (api_field) {
	case V4L2_FIELD_ANY:
		/* fall through to progressive */
	case V4L2_FIELD_NONE:
		hfi_field = BUFFER_PKT_FLAG_PROGRESSIVE_FRAME;
		break;
	case V4L2_FIELD_INTERLACED:
		/* fall through to interlaced_tb */
	case V4L2_FIELD_INTERLACED_TB:
		hfi_field = BUFFER_PKT_FLAG_INTERLEAVED_FRAME_TOP_FIRST;
		break;
	case V4L2_FIELD_INTERLACED_BT:
		hfi_field = BUFFER_PKT_FLAG_INTERLEAVED_FRAME_BOT_FIRST;
		break;
	case V4L2_FIELD_SEQ_TB:
		hfi_field = BUFFER_PKT_FLAG_INTERLACED_TOP_FIRST;
		break;
	case V4L2_FIELD_SEQ_BT:
		hfi_field = BUFFER_PKT_FLAG_INTERLACED_BOT_FIRST;
		break;
	case V4L2_FIELD_TOP:
		hfi_field = BUFFER_PKT_FLAG_SINGLE_FIELD_TOP;
		break;
	case V4L2_FIELD_BOTTOM:
		hfi_field = BUFFER_PKT_FLAG_SINGLE_FIELD_BOT;
		break;
	case V4L2_FIELD_ALTERNATE:
		/* fall through to default */
	default:
		dprintk(VPU_ERR,
			"Unsupported api field type (enum v4l2_field=%d)",
			api_field);
		hfi_field = 0;
		break;
	}

	return hfi_field;
}

u32 translate_field_to_api(u32 hfi_field)
{
	enum v4l2_field api_field;

	switch (hfi_field & BUFFER_PKT_FLAG_BUFFER_TYPE_MASK) {
	case BUFFER_PKT_FLAG_PROGRESSIVE_FRAME:
		api_field = (u32) V4L2_FIELD_NONE;
		break;
	case BUFFER_PKT_FLAG_INTERLEAVED_FRAME_TOP_FIRST:
		api_field = (u32) V4L2_FIELD_INTERLACED_TB;
		break;
	case BUFFER_PKT_FLAG_INTERLEAVED_FRAME_BOT_FIRST:
		api_field = (u32) V4L2_FIELD_INTERLACED_BT;
		break;
	case BUFFER_PKT_FLAG_INTERLACED_TOP_FIRST:
		api_field = (u32) V4L2_FIELD_SEQ_TB;
		break;
	case BUFFER_PKT_FLAG_INTERLACED_BOT_FIRST:
		api_field = (u32) V4L2_FIELD_SEQ_BT;
		break;
	case BUFFER_PKT_FLAG_SINGLE_FIELD_TOP:
		api_field = (u32) V4L2_FIELD_TOP;
		break;
	case BUFFER_PKT_FLAG_SINGLE_FIELD_BOT:
		api_field = (u32) V4L2_FIELD_BOTTOM;
		break;
	default:
		dprintk(VPU_ERR, "Unsupported hfi field type (%d)\n",
				hfi_field);
		api_field = 100; /* invalid */
		break;
	}

	return api_field;
}

u32 translate_v4l2_scan_mode(enum v4l2_field field)
{
	return __trans_scan_mode_to_hfi(field);
}
