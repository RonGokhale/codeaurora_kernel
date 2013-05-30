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

#ifndef __H_VPU_CHANNEL_H__
#define __H_VPU_CHANNEL_H__

#include <media/videobuf2-core.h>
#include "vpu_resources.h"

/*
 * custom VPU buffer structure
 */
#define ADDR_INDEX_VPU  0
#define ADDR_INDEX_VCAP 1
#define ADDR_INDEX_MDP  2

#define ADDR_VALID_VPU	(1 << ADDR_INDEX_VPU)
#define ADDR_VALID_VCAP (1 << ADDR_INDEX_VCAP)
#define ADDR_VALID_MDP  (1 << ADDR_INDEX_MDP)

struct vpu_buffer {
	struct vb2_buffer vb;
	u32 valid_addresses_mask;
	struct list_head  buffers_entry;

	struct vpu_plane {
		/* use ADDR_INDEX_xxxx to get the device address */
		u32 mapped_address[3];

		/* internal to v4l2 driver, not used by chan layer */
		u32 new_plane;
		u32 user_fd;
		u32 length;
		u32 data_offset;
		void *mem_cookie;
	} planes[VIDEO_MAX_PLANES];
};

#define to_vpu_buffer(vb_buf) container_of(vb_buf, struct vpu_buffer, vb)

/*
 * Buffer callback
 * clients need to provide this callback for streaming
 * channel will deliver a processed buffer
 *    (empty input buffer or filled output buffer) to client
 * channel does not interpret priv
 */
typedef void (*channel_buffer_handler)(u32 sid, struct vpu_buffer *pbuf,
	u32 data, void *priv);

/* event type */
enum vpu_hw_event {
	VPU_SYS_EVENT_NONE = 0,
	VPU_HW_EVENT_WATCHDOG_BITE,
	VPU_HW_EVENT_BOOT_FAIL,
	VPU_HW_EVENT_IPC_ERROR,
	VPU_HW_EVENT_ACTIVE_REGION_CHANGED,
};

/*
 * event callback
 * clients need to provide this callback for event handling
 * channel does not interpret priv
 * @event:	see enum vpu_chan_event
 */
typedef void (*channel_event_handler)(u32 sid, u32 event, u32 data, void *priv);

/*
 * static initialization
 * to be called at driver probe time
 * IO resources will be mapped, but VPU is not powered on
 */
int vpu_hw_sys_init(struct vpu_platform_resources *res);

/*
 * static cleanup
 * to be called at driver removal time
 * IO resources will be freed
 */
void vpu_hw_sys_cleanup(void);

/*
 * dynamic initialization
 * to be called when device is to be used
 * VPU will be powered on, and interrupt and IPC communication enabled
 * @event_cb:	event callback (to handle system + session events)
 * @buffer_cb:	buffer callback (to handle session buffer flow)
 * @priv:	private argument that will be sent back to callback
 */
int vpu_hw_sys_start(channel_event_handler event_cb,
		channel_buffer_handler buffer_cb,
		void *priv);

/*
 * dynamic cleanup
 * to be called when device is to be used or not to be used
 * VPU will be powered off, and interrupt and IPC communication disabled
 */
void vpu_hw_sys_stop(void);

/*
 * VPU extended system command
 * sends an extended system command to VPU
 * @cmd: extended command
 * @data: pointer to control data, sent as is
 * @data_size: size of data payload
 */
enum vpu_sys_cmd_ext {
	VPU_SYS_CMD_DEBUG_CRASH,
	VPU_SYS_CMD_EXT_UNKNOWN
};
int vpu_hw_sys_cmd_ext(enum vpu_sys_cmd_ext cmd,
		void *data, u32 data_size);

/*
 * Set/get system property parameters
 * Pproperties are identified by their prop_id, and data is passed as a pointer.
 * @prop_id:	porperty ID as per vpu_property.h definition (VPU_PROP_xxx)
 * @data: pointer to control data corresponding to HFI structs defintion
 * @data_size:	size of data payload
 * @buf: address of buffer to receive property data
 * @buf_size: size of the buffer to receive property data
 */
int vpu_hw_sys_s_property(u32 prop_id, void *data, u32 data_size);
int vpu_hw_sys_g_property(u32 prop_id, void *buf, u32 buf_size);

/*
 * Set/get extended property parameters .
 * Controls are identified by their prop_id, and data is passed as a pointer.
 * @prop_id:	porperty ID as per vpu_property.h definition (VPU_PROP_xxx)
 * @data:	pointer to control data corresponding to HFI structs defintion
 * @data_size:	size of data payload
 * @buf: address of buffer to receive property data
 * @buf_size: size of the buffer to receive property data
 */
int vpu_hw_sys_s_property_ext(void __user *data, u32 data_size);
int vpu_hw_sys_g_property_ext(void __user *data, u32 data_size,
		void __user *buf, u32 buf_size);

/*
 * session open
 * To allocate the session for streaming
 */
int vpu_hw_session_open(u32 sid, u32 flag);

/*
 * session close
 * To free the session from streaming
 */
void vpu_hw_session_close(u32 sid);

/*
 * session stream state controls
 * to start/stop/pause/resume streaming
 */
int vpu_hw_session_start(u32 sid);
int vpu_hw_session_stop(u32 sid);
int vpu_hw_session_pause(u32 sid);
int vpu_hw_session_resume(u32 sid);

/*
 * Set input/output port configuration. Channel *does not* commit new settings.
 * returns 0 on success, -ve value on failure
 */
int vpu_hw_session_s_input_params(u32 sid,
		const struct vpu_prop_session_input *inp);
int vpu_hw_session_s_output_params(u32 sid,
		const struct vpu_prop_session_output *outp);

/*
 * Get input/output port configuration.
 * Channel copies current hardware configuration into *param.
 * returns 0 on success, -ve value on failure
 */
int vpu_hw_session_g_input_params(u32 sid,
		struct vpu_prop_session_input *inp);
int vpu_hw_session_g_output_params(u32 sid,
		struct vpu_prop_session_output *outp);

/*
 * Config/release Noise Reduction buffer. Channel layer does not know the nature
 * of these buffer (secure or non-secure).
 * returns 0 on success, -ve value on failure
 * @in_addr:	buffer address for temporal input buffer
 * @out_addr:	buffer address for temporal output buffer
 */
int vpu_hw_session_nr_buffer_config(u32 sid, u32 in_addr, u32 out_addr);
int vpu_hw_session_nr_buffer_release(u32 sid);

/*
 * Set/get property parameters (non-port specifc).
 * Controls are identified by their prop_id, and data is passed as a pointer.
 * @prop_id:	porperty ID as per vpu_property.h definition (VPU_PROP_xxx)
 * @data:	pointer to control data corresponding to HFI structs defintion
 * @data_size:	size of data payload
 * @buf: address of buffer to receive property data
 * @buf_size: size of the buffer to receive property data
 */
int vpu_hw_session_s_property(u32 sid, u32 prop_id, void *data, u32 data_size);
int vpu_hw_session_g_property(u32 sid, u32 prop_id, void *buf, u32 buf_size);

/*
 * Set/get extended property parameters .
 * Controls are identified by their prop_id, and data is passed as a pointer.
 * @prop_id:	porperty ID as per vpu_property.h definition (VPU_PROP_xxx)
 * @data:	pointer to control data corresponding to HFI structs defintion
 * @data_size:	size of data payload
 * @buf: address of buffer to receive property data
 * @buf_size: size of the buffer to receive property data
 */
int vpu_hw_session_s_property_ext(u32 sid, void __user *data, u32 data_size);
int vpu_hw_session_g_property_ext(u32 sid, void __user *data, u32 data_size,
		void __user *buf, u32 buf_size);

/*
 * VPU extended session command
 * sends an extended session command to VPU, data sent as is
 * @sid: session ID
 * @cmd: command ID
 * @data:	pointer to control data corresponding to HFI structs defintion
 * @data_size:	size of data payload
 */
int vpu_hw_session_cmd_ext(u32 sid, u32 cmd, void *data, u32 data_size);

/*
 * commit all VPU setting/change sent previously
 * specifies how this setting shall be applied
 * CH_COMMIT_AT_ONCE : applied immediately
 * CH_COMMIT_IN_ORDER: applied in the order in the queue
 * @load is the VPU load in bits per second
 */
enum commit_type {
	CH_COMMIT_AT_ONCE,
	CH_COMMIT_IN_ORDER,
};
int vpu_hw_session_commit(u32 sid, enum commit_type type, u32 load);

/* register session buffers
 * pass a list of buffers to session for use in tunnel case
 */
int vpu_hw_session_register_buffers(u32 sid, bool input,
				struct vpu_buffer **vb, u32 num);

/*
 * release session buffers
 * to tell a session to stop using all buffers,
 *    whether processing finished or not
 * VPU does not return buffers because of this call
 */
enum release_buf_type {
	CH_RELEASE_IN_BUF,
	CH_RELEASE_OUT_BUF,
	CH_RELEASE_NR_BUF
};
int vpu_hw_session_release_buffers(u32 sid, enum release_buf_type);

/*
 * fill an output buffer
 * pass an empty output buffer to the session
 */
int vpu_hw_session_fill_buffer(u32 sid, struct vpu_buffer*);

/*
 * empty an input buffer
 * pass a filled input buffer to the session to process
 */
int vpu_hw_session_empty_buffer(u32 sid, struct vpu_buffer*);

/*
 * session flush
 * to tell a session to return all buffers back,
 *    whether processing finished or not
 * buffers will be returned via buffer callback
 */
enum flush_buf_type {
	CH_FLUSH_IN_BUF = 0,
	CH_FLUSH_OUT_BUF,
	CH_FLUSH_ALL_BUF
};
int vpu_hw_session_flush(u32 sid, enum flush_buf_type);

#endif /* __H_VPU_CHANNEL_H__ */
