/* Copyright (c) 2012, The Linux Foundation. All rights reserved.
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

#ifndef VCAP_FMT_H
#define VCAP_FMT_H
#include <linux/videodev2.h>

#define V4L2_BUF_TYPE_INTERLACED_IN_DECODER (V4L2_BUF_TYPE_PRIVATE)

#define VCAP_GENERIC_NOTIFY_EVENT 0
#define VCAP_VC_PIX_ERR_EVENT 1
#define VCAP_VC_LINE_ERR_EVENT 2
#define VCAP_VC_VSYNC_ERR_EVENT 3
#define VCAP_VC_NPL_OFLOW_ERR_EVENT 4
#define VCAP_VC_LBUF_OFLOW_ERR_EVENT 5
#define VCAP_VC_BUF_OVERWRITE_EVENT 6
#define VCAP_VC_VSYNC_SEQ_ERR 7
#define VCAP_VP_REG_R_ERR_EVENT 8
#define VCAP_VP_REG_W_ERR_EVENT 9
#define VCAP_VP_IN_HEIGHT_ERR_EVENT 10
#define VCAP_VP_IN_WIDTH_ERR_EVENT 11
#define VCAP_VC_UNEXPECT_BUF_DONE 12
#define VCAP_MAX_NOTIFY_EVENT 13

enum hal_vcap_mode {
	HAL_VCAP_MODE_PRO = 0,
	HAL_VCAP_MODE_INT,
};

enum hal_vcap_polar {
	HAL_VCAP_POLAR_POS = 0,
	HAL_VCAP_POLAR_NEG,
};

enum hal_vcap_color {
	HAL_VCAP_YUV = 0,
	HAL_VCAP_RGB,
};

enum nr_threshold_mode {
	NR_THRESHOLD_STATIC = 0,
	NR_THRESHOLD_DYNAMIC,
};

enum nr_mode {
	NR_DISABLE = 0,
	NR_AUTO,
	NR_MANUAL,
};

enum nr_decay_ratio {
	NR_Decay_Ratio_26 = 0,
	NR_Decay_Ratio_25,
	NR_Decay_Ratio_24,
	NR_Decay_Ratio_23,
	NR_Decay_Ratio_22,
	NR_Decay_Ratio_21,
	NR_Decay_Ratio_20,
	NR_Decay_Ratio_19,
};

enum tune_balance_mode {
	VCAP_TUNE_AUTO = 0,
	VCAP_TUNE_SPACE_TIME = 1,
	VCAP_TUNE_SPATIAL = 2,
	VCAP_TUNE_TEMPORAL = 3,
	VCAP_TUNE_WEAVE_FM = 5,
	VCAP_TUNE_WEAVE_T1 = 6,
	VCAP_TUNE_WEAVE_TM1 = 7,
};

enum tune_film_state {
	VCAP_TUNE_VIDEO = 0,
	VCAP_TUNE_UNSURE = 1,
	VCAP_TUNE_MIXED = 2,
	VCAP_TUNE_FILM = 3,
};

enum tune_split_mode {
	VCAP_TUNE_SPLIT_OFF = 0,
	VCAP_TUNE_SPLIT_WEAVE_L = 1,
	VCAP_TUNE_SPLIT_WEAVE_R = 3,
	VCAP_TUNE_SPLIT_BOB_L = 5,
	VCAP_TUNE_SPLIT_BOB_R = 7,
};

struct nr_config {
	uint8_t max_blend_ratio;
	uint8_t scale_diff_ratio;
	uint8_t diff_limit_ratio;
	uint8_t scale_motion_ratio;
	uint8_t blend_limit_ratio;
	uint16_t scale_diff;
	uint16_t diff_limit;
	uint16_t scale_motion;
	uint16_t blend_limit;
};

struct nr_param {
	enum nr_threshold_mode threshold;
	enum nr_mode mode;
	enum nr_decay_ratio decay_ratio;
	uint8_t window;
	struct nr_config luma;
	struct nr_config chroma;
};

struct tuning_param {
	enum tune_balance_mode bal_mode;
	enum tune_film_state film_state;
	enum tune_split_mode split_mode;
};

#define VCAPIOC_NR_S_PARAMS _IOWR('V', (BASE_VIDIOC_PRIVATE+0), struct nr_param)
#define VCAPIOC_NR_G_PARAMS _IOWR('V', (BASE_VIDIOC_PRIVATE+1), struct nr_param)
#define VCAPIOC_S_NUM_VC_BUF _IOWR('V', (BASE_VIDIOC_PRIVATE+2), int)
#define VCAPIOC_TUNE_S_PARAMS _IOWR('V', (BASE_VIDIOC_PRIVATE+3), \
	struct tuning_param)
#define VCAPIOC_TUNE_G_PARAMS _IOWR('V', (BASE_VIDIOC_PRIVATE+4), \
	struct tuning_param)
#define VCAPIOC_PAUSE _IO('V', (BASE_VIDIOC_PRIVATE+5))
#define VCAPIOC_RESUME _IO('V', (BASE_VIDIOC_PRIVATE+6))
#define VCAPIOC_RESET_S_FMT _IO('V', (BASE_VIDIOC_PRIVATE+7))

struct v4l2_format_vc_ext {
	enum hal_vcap_mode     mode;
	enum hal_vcap_polar    h_polar;
	enum hal_vcap_polar    v_polar;
	enum hal_vcap_polar    d_polar;
	enum hal_vcap_color    color_space;

	uint32_t clk_freq;
	uint32_t frame_rate;
	uint32_t vtotal;
	uint32_t htotal;
	uint32_t hactive_start;
	uint32_t hactive_end;
	uint32_t vactive_start;
	uint32_t vactive_end;
	uint32_t vsync_start;
	uint32_t vsync_end;
	uint32_t hsync_start;
	uint32_t hsync_end;
	uint32_t f2_vactive_start;
	uint32_t f2_vactive_end;
	uint32_t f2_vsync_h_start;
	uint32_t f2_vsync_h_end;
	uint32_t f2_vsync_v_start;
	uint32_t f2_vsync_v_end;
	uint32_t sizeimage;
	uint32_t bytesperline;
};

enum vcap_type {
	VC_TYPE,
	VP_IN_TYPE,
	VP_OUT_TYPE,
};

enum vcap_stride {
	VC_STRIDE_16,
	VC_STRIDE_32,
};

struct vcap_priv_fmt {
	enum vcap_type type;
	enum vcap_stride stride;
	union {
		struct v4l2_format_vc_ext timing;
		struct v4l2_pix_format pix;
		/* Once VP is created there will be another type in here */
	} u;
};
#endif
