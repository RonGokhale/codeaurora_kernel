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

#ifndef _H_VPU_CONFIGURATION_H_
#define _H_VPU_CONFIGURATION_H_

#include <linux/videodev2.h>
#include <media/msm_vpu.h>
#include "vpu_v4l2.h"


/*
 * VPU private controls
 */
#define NUM_NR_BUFFERS 2

struct vpu_controller {
	/* controls cache */
	u32 cache_size;
	void *cache;

	/* Control specific metadata */
	void *nr_buffers[NUM_NR_BUFFERS];
};

void setup_vpu_controls(void);

struct vpu_controller *init_vpu_controller(struct vpu_platform_resources *res);

void deinit_vpu_controller(struct vpu_controller *controller);

int apply_vpu_control(struct vpu_dev_session *session, int cmd,
		struct vpu_control *control);

int apply_vpu_control_extended(struct vpu_client *client, int cmd,
		struct vpu_control_extended *control);

void *get_control(struct vpu_controller *controller, u32 id);


/*
 * Configuration commit functions.
 * First function is used for initial commit when both ports streamon.
 *
 * The other two are only used in runtime config after intial commit is done.
 * commit_port_config: sets and commits a speciifc port config in runtime
 * commit_control: used by controls
 *
 * @new_load: indicates that hardawre load needs to be recalculated.
 */
int commit_initial_config(struct vpu_dev_session *session);

int commit_port_config(struct vpu_dev_session *session, int port, int new_load);

int commit_control(struct vpu_dev_session *session, int new_load);


/*
 * Port configuration
 */
struct vpu_format_desc {
	uint8_t description[32];
	uint32_t fourcc;
	int num_planes;
	struct {
		int bitsperpixel;
		int heightfactor;
	} plane[VPU_MAX_PLANES];
};

const struct vpu_format_desc *query_supported_formats(int index);

u32 get_bytesperline(u32 width, u32 bitsperpixel, u32 input_bytesperline);

static inline u32 get_sizeimage(u32 bytesperline, u32 height, u32 heightfactor)
{
	return bytesperline * height / heightfactor;
}

int is_format_valid(struct v4l2_format *fmt);

struct vpu_io_desc {
	uint8_t description[32];
};

const struct vpu_io_desc *query_inputs_outputs(int port, int index);

#endif /* _H_VPU_CONFIGURATION_H_ */
