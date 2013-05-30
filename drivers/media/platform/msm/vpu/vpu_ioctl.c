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

#include <media/v4l2-event.h>

#include "vpu_ioctl_internal.h"
#include "vpu_configuration.h"
#include "vpu_translate.h"
#include "vpu_channel.h"
#include "vpu_debug.h"

/*
 * Events/Callbacks handling
 */
static void __prepare_v4l2_event(struct v4l2_event *event,
		u32 type, u8 *data, u32 size)
{
	event->type = type;
	if (data && size > sizeof(event->u.data)) {
		dprintk(VPU_WARN, "%s Payload size (%d) is too large\n",
				__func__, size);
		size = sizeof(event->u.data);
	}
	if (data)
		memcpy(event->u.data, data, size);
}

void notify_vpu_event_client(struct vpu_client *client,
		u32 type, u8 *data, u32 size)
{
	struct v4l2_event event = {0};
	if (!client)
		return;

	__prepare_v4l2_event(&event, type, data, size);
	v4l2_event_queue_fh(&client->vfh, &event);
}

void notify_vpu_event_session(struct vpu_dev_session *session,
		u32 type, u8 *data, u32 size)
{
	struct v4l2_event event = {0};
	struct vpu_client *clnt, *n;
	if (!session)
		return;

	__prepare_v4l2_event(&event, type, data, size);
	list_for_each_entry_safe(clnt, n,
				      &session->clients_list, clients_entry)
		v4l2_event_queue_fh(&clnt->vfh, &event);
}

/* __system_callback_handler
 *
 * event:	@see enum vpu_chan_event defined in vpu_channel.h
 */
static void __sys_event_callback_handler(u32 sid, u32 event,
		u32 data, void *pcore)
{
	struct vpu_dev_core *core = (struct vpu_dev_core *)pcore;
	int i;

	if (sid < VPU_NUM_SESSIONS) {
		/* session event */
		struct vpu_dev_session *session = core->sessions[sid];

		if (event == VPU_HW_EVENT_IPC_ERROR) {
			dprintk(VPU_WARN, "%s: error in session %d (%d, %d)!\n",
					__func__, session->id, event, data);

			notify_vpu_event_session(session, VPU_EVENT_HW_ERROR,
					(u8 *) &data, sizeof(data));
		}

		if (event == VPU_HW_EVENT_ACTIVE_REGION_CHANGED) {
			struct v4l2_rect roi_result, *cache;
			struct rect *roi_rect = (struct rect *)data;

			translate_roi_rect_to_api(roi_rect, &roi_result);

			cache = get_control(session->controller,
					VPU_CTRL_ACTIVE_REGION_RESULT);
			if (cache)
				memcpy(cache, &roi_result, sizeof(roi_result));

			notify_vpu_event_session(session,
					VPU_EVENT_ACTIVE_REGION_CHANGED,
					(u8 *)&roi_result, sizeof(roi_result));
		}
	} else {
		/* system event */
		if ((event == VPU_HW_EVENT_WATCHDOG_BITE) ||
			(event == VPU_HW_EVENT_IPC_ERROR)) {

			dprintk(VPU_WARN, "%s: VPU HW error (%d)\n",
					__func__, event);

			/* notify all sessions */
			for (i = 0; i < VPU_NUM_SESSIONS; i++)
				notify_vpu_event_session(core->sessions[i],
						VPU_EVENT_HW_ERROR, NULL, 0);

		}
	}
}

static void __sys_buffer_callback_handler(u32 sid, struct vpu_buffer *pbuf,
				    u32 data, void *client_data)
{
	int port, i;
	struct vpu_dev_session *session;
	struct vpu_port_info *port_info;
	if (!pbuf)
		return;

	port = get_port_number(pbuf->vb.vb2_queue->type);
	session = vb2_get_drv_priv(pbuf->vb.vb2_queue);

	if (data) {
		dprintk(VPU_DBG, "ERROR (%d) buffer callback port %d buff %d\n",
				data, port, pbuf->vb.v4l2_buf.index);
		vb2_buffer_done(&pbuf->vb, VB2_BUF_STATE_ERROR);
	} else {
		dprintk(VPU_DBG, "GOOD buffer callback for port %d buff %d\n",
				port, pbuf->vb.v4l2_buf.index);

		/* update bytesused */
		port_info = &session->port_info[port];
		for (i = 0; i < pbuf->vb.num_planes; i++)
			pbuf->vb.v4l2_planes[i].bytesused =
				port_info->format.plane_fmt[i].sizeimage;

		vb2_buffer_done(&pbuf->vb, VB2_BUF_STATE_DONE);
	}
}

/*
 * Session/Client management
 */

/*
 * Dynamic switch (on/off) of VPU hardware on first/last global client.
 * Function must be called with core->lock mutex held.
 */
static int __dynamic_vpu_hw_switch(struct vpu_dev_core *core, int on)
{
	int ret = 0;

	if (on) {

		if (core->global_client_count++ == 0) {

			ret = vpu_hw_sys_start(__sys_event_callback_handler,
					__sys_buffer_callback_handler,
					(void *)core);
			if (ret) {
				dprintk(VPU_ERR,
				"%s:failed to start IPC system\n", __func__);
				core->global_client_count--;
				goto exit_hfi_init;
			}

			ret = attach_vpu_iommus(&core->resources);
			if (ret) {
				dprintk(VPU_ERR,
				"%s:could not attach VPU IOMMUs\n", __func__);
				core->global_client_count--;
				goto err_stop_sys;
			}
		}

	} else { /*deinit */

		if (--core->global_client_count == 0)
			goto hfi_deinit;
	}

	goto exit_hfi_init; /* hfi system stays on */

hfi_deinit:
	dprintk(VPU_INFO, "%s:Shutting down hfi IPC\n", __func__);
	detach_vpu_iommus(&core->resources);
err_stop_sys:
	vpu_hw_sys_stop();
exit_hfi_init:
	return ret;
}

struct vpu_client *__create_client(struct vpu_dev_core *core, struct file *file)
{
	struct vpu_client *client;
	int ret = 0;

	mutex_lock(&core->lock);

	if (core->global_client_count >= VPU_MAX_CLIENTS) {
		ret = -EBUSY;
		goto err_client_create;
	}

	/* create client struct */
	client = devm_kzalloc(core->dev, sizeof(*client), GFP_KERNEL);
	if (!client) {
		ret = -ENOMEM;
		goto err_client_create;
	}
	client->core = core;
	client->type = file ? VPU_USERSPACE_CLIENT : VPU_KERNEL_SUBDEV_CLIENT;
	INIT_LIST_HEAD(&client->clients_entry);

	/* Initialize HFI on first client open */
	ret = __dynamic_vpu_hw_switch(core, 1);
	if (ret) {
		devm_kfree(core->dev, client);
		goto err_client_create;
	}

	list_add_tail(&client->clients_entry, &core->unattached_list);
	mutex_unlock(&core->lock);

	/* initialize v4l2_fh */
	v4l2_fh_init(&client->vfh, &core->vdev);
	if (file)
		file->private_data = &client->vfh;
	v4l2_fh_add(&client->vfh);

	dprintk(VPU_DBG, "Client creation successful\n");
	return client;

err_client_create:
	mutex_unlock(&core->lock);
	return ERR_PTR(ret);
}

int vpu_open_user_client(struct file *file)
{
	struct vpu_dev_core *core = video_drvdata(file);
	struct vpu_client *client;

	dprintk(VPU_DBG, "%s, New userspace client\n", __func__);

	client = __create_client(core, file);
	if (IS_ERR_OR_NULL(client))
		return PTR_ERR(client);

	return 0;
}

struct vpu_client *vpu_open_kernel_client(void)
{
	struct vpu_dev_core *core = 0;
	struct vpu_client *client;
	if (!core)
		return NULL; /* not enabled for now */

	dprintk(VPU_DBG, "%s, New kernel client\n", __func__);

	client = __create_client(core, NULL);
	if (IS_ERR_OR_NULL(client))
		return NULL;

	return client;
}

int vpu_close_client(struct vpu_client *client)
{
	if (!client)
		return 0; /* should never happen */
	dprintk(VPU_DBG, "Enter function %s\n", __func__);

	if (client->session)
		vpu_detach_client(client);

	mutex_lock(&client->core->lock);
	__dynamic_vpu_hw_switch(client->core, 0);
	list_del_init(&client->clients_entry);
	mutex_unlock(&client->core->lock);

	v4l2_fh_del(&client->vfh);
	v4l2_fh_exit(&client->vfh);
	devm_kfree(client->core->dev, client);
	return 0;
}

int vpu_attach_client(struct vpu_client *client, int num_session)
{
	struct vpu_dev_core *core;
	struct vpu_dev_session *session;
	int ret = 0;
	if (!client || num_session <= 0 || num_session > VPU_NUM_SESSIONS) {
		dprintk(VPU_ERR, "invalid session attach # %d\n", num_session);
		return -EINVAL;
	}

	core = client->core;
	session = core->sessions[num_session-1];

	dprintk(VPU_DBG,
		"Attach client %p to session %d\n", client, num_session);

	if (client->session == session)
		return 0; /* client already attached to this session */
	else if (client->session) {
		dprintk(VPU_ERR, "Client already attached to a session\n");
		return -EINVAL;
	}

	if (session->client_count >= VPU_MAX_CLIENTS_PER_SESSION)
		return -EBUSY;

	mutex_lock(&session->lock);

	if (session->client_count++ == 0) {

		/* Initialize controller */
		session->controller =
				init_vpu_controller(&session->core->resources);
		if (!session->controller) {
			dprintk(VPU_DBG, "init_vpu_controller failed\n");
			goto err_dec_count;
		}

		/* Open hw session & IPC channel */
		ret = vpu_hw_session_open(session->id, 0);
		if (ret) {
			dprintk(VPU_DBG, "could not open IPC channel\n");
			goto err_deinit_controller;
		}
	}

	list_del_init(&client->clients_entry); /* remove from unattached_list */
	list_add_tail(&client->clients_entry, &session->clients_list);
	client->session = session;
	mutex_unlock(&session->lock);

	dprintk(VPU_DBG, "Attach to session %d successful\n", num_session);
	return 0;

err_deinit_controller:
	deinit_vpu_controller(session->controller);
	session->controller = NULL;
err_dec_count:
	session->client_count--;
	mutex_unlock(&session->lock);
	return ret;
}

void vpu_detach_client(struct vpu_client *client)
{
	struct vpu_dev_session *session;
	int port = 0;
	if (!client || !client->session)
		return;
	session = client->session;
	dprintk(VPU_DBG, "Enter function %s\n", __func__);

	mutex_lock(&session->lock);
	for (port = 0; port < NUM_VPU_PORTS; ++port) {
		mutex_lock(&session->que_lock[port]);
		/* Stream off and return buffers if client was doing io ops */
		if (client == session->io_client[port]) {
			vb2_queue_release(&session->vbqueue[port]);
			session->io_client[port] = NULL;
		}
		mutex_unlock(&session->que_lock[port]);
	}

	list_del_init(&client->clients_entry);
	if (--session->client_count == 0) {
		/* close hw session on last detach */
		vpu_hw_session_close(session->id);

		/* reset configuration data */
		deinit_vpu_controller(session->controller);
		session->controller = NULL;
		memset(&session->port_info[INPUT_PORT], 0,
				sizeof(session->port_info[INPUT_PORT]));
		memset(&session->port_info[OUTPUT_PORT], 0,
				sizeof(session->port_info[OUTPUT_PORT]));
	}

	client->session = NULL;
	mutex_unlock(&session->lock);

	list_add_tail(&client->core->unattached_list, &client->clients_entry);
}


/*
 * Format/parameters configuration
 */
int vpu_enum_fmt(struct v4l2_fmtdesc *f)
{
	const struct vpu_format_desc *fmt;
	int port = get_port_number(f->type);
	if (port < 0)
		return -EINVAL;

	dprintk(VPU_DBG, "Enter function %s\n", __func__);
	fmt = query_supported_formats(f->index);
	if (!fmt)
		return -EINVAL;

	memset(f->reserved, 0 , sizeof(f->reserved));
	strlcpy(f->description, fmt->description, sizeof(f->description));
	f->pixelformat = fmt->fourcc;
	return 0;
}

int vpu_get_fmt(struct vpu_client *client, struct v4l2_format *f)
{
	struct vpu_prop_session_input    in_param;
	struct vpu_prop_session_output   out_param;
	struct vpu_dev_session *session = client ? client->session : 0;
	int ret = 0, port;
	if (!session)
		return -EPERM;

	port = get_port_number(f->type);
	if (port < 0)
		return -EINVAL;

	if (port == INPUT_PORT) {
		ret = vpu_hw_session_g_input_params(session->id, &in_param);
		translate_input_format_to_api(&in_param, f);
	} else {
		ret = vpu_hw_session_g_output_params(session->id, &out_param);
		translate_output_format_to_api(&out_param, f);
	}
	return ret;
}

int vpu_try_fmt(struct vpu_client *client, struct v4l2_format *f)
{
	int i;
	u32 hfi_pixelformat;
	const struct vpu_format_desc *vpu_format;

	hfi_pixelformat = trans_pixelformat_to_hfi(f->fmt.pix_mp.pixelformat);
	vpu_format = query_supported_formats(hfi_pixelformat);
	if (!vpu_format)
		return -EINVAL;

	if (!is_format_valid(f))
		return -EINVAL;

	f->fmt.pix_mp.num_planes = vpu_format->num_planes;
	for (i = 0; i < vpu_format->num_planes; i++) {
		f->fmt.pix_mp.plane_fmt[i].bytesperline =
			get_bytesperline(f->fmt.pix_mp.width,
				vpu_format->plane[i].bitsperpixel,
				f->fmt.pix_mp.plane_fmt[i].bytesperline);
		f->fmt.pix_mp.plane_fmt[i].sizeimage =
			get_sizeimage(f->fmt.pix_mp.plane_fmt[i].bytesperline,
				f->fmt.pix_mp.height,
				vpu_format->plane[i].heightfactor);
	}

	return 0;
}

int vpu_set_fmt(struct vpu_client *client, struct v4l2_format *f)
{
	struct vpu_dev_session *session = client ? client->session : 0;
	int ret = 0;
	int port;
	struct v4l2_rect def_roi;
	struct v4l2_pix_format_mplane *pix_mp = &f->fmt.pix_mp;
	if (!session) {
		dprintk(VPU_ERR, "%s, invalid session\n", __func__);
		return -EPERM;
	}
	port = get_port_number(f->type);
	if (port < 0) {
		dprintk(VPU_ERR, "%s, invalid port (%d)\n", __func__, port);
		return -EINVAL;
	}
	ret = vpu_try_fmt(client, f);
	if (ret < 0) {
		dprintk(VPU_ERR, "%s try_fmt failed (err %d)\n", __func__, ret);
		return -EINVAL;
	}

	/* by default, ROI is the full frame */
	def_roi.left = 0;
	def_roi.top = 0;
	def_roi.width = pix_mp->width;
	def_roi.height = pix_mp->height;

	mutex_lock(&session->lock);
	memcpy(&session->port_info[port].format, pix_mp, sizeof(*pix_mp));
	memcpy(&session->port_info[port].roi, &def_roi, sizeof(def_roi));
	if (port == INPUT_PORT)
		session->port_info[INPUT_PORT].scan_mode =
				translate_v4l2_scan_mode(pix_mp->field);

	ret = commit_port_config(session, port, 1);
	if (ret) {
		dprintk(VPU_ERR, "%s, commit_port_config failed (err %d)\n",
				__func__, ret);
		return -EINVAL;
	}
	mutex_unlock(&session->lock);
	return ret;
}

int vpu_get_framerate(struct vpu_client *client,
		struct v4l2_fract *timeperframe, enum v4l2_buf_type type)
{
	struct vpu_prop_session_input    in_param;
	struct vpu_prop_session_output   out_param;
	struct vpu_dev_session *session = client ? client->session : 0;
	int ret = 0, port = get_port_number(type);
	if (!session)
		return -EPERM;

	if (port < 0)
		return -EINVAL;

	if (port == INPUT_PORT) {
		ret = vpu_hw_session_g_input_params(session->id, &in_param);
		translate_xfps_to_timeperframe(in_param.frame_rate,
				timeperframe);
	} else {
		ret = vpu_hw_session_g_output_params(session->id, &out_param);
		translate_xfps_to_timeperframe(out_param.frame_rate,
				timeperframe);
	}
	return ret;
}

int vpu_set_framerate(struct vpu_client *client,
		struct v4l2_fract *timeperframe, enum v4l2_buf_type type)
{
	struct vpu_dev_session *session = client ? client->session : 0;
	int ret = 0, port = get_port_number(type);
	if (!session)
		return -EPERM;

	if (port < 0)
		return -EINVAL;

	if (timeperframe->numerator == 0) {
		dprintk(VPU_ERR, "Invalid timeperframe setting of 0\n");
		return -EINVAL;
	}

	mutex_lock(&session->lock);
	session->port_info[port].framerate =
			translate_timeperframe_to_xfps(timeperframe);

	ret = commit_port_config(session, port, 1);
	mutex_unlock(&session->lock);
	return ret;
}

int vpu_get_3d_mode(struct vpu_client *client,
		u32 *mode_3d, enum v4l2_buf_type type)
{
	struct vpu_prop_session_input    in_param;
	struct vpu_prop_session_output   out_param;
	struct vpu_dev_session *session = client ? client->session : 0;
	int ret = 0, port = get_port_number(type);
	if (!session)
		return -EPERM;

	if (port < 0)
		return -EINVAL;

	if (port == INPUT_PORT) {
		ret = vpu_hw_session_g_input_params(session->id, &in_param);
		*mode_3d = translate_3d_mode_to_api(in_param.video_format);
	} else {
		ret = vpu_hw_session_g_output_params(session->id, &out_param);
		*mode_3d = translate_3d_mode_to_api(out_param.video_format);
	}
	return ret;
}

int vpu_set_3d_mode(struct vpu_client *client,
		u32 *mode_3d, enum v4l2_buf_type type)
{
	struct vpu_dev_session *session = client ? client->session : 0;
	int ret = 0, port = get_port_number(type);
	if (!session)
		return -EPERM;
	if (port < 0)
		return -EINVAL;

	mutex_lock(&session->lock);
	session->port_info[port].video_fmt = *mode_3d;
	ret = commit_port_config(session, port, 0);
	mutex_unlock(&session->lock);
	return ret;
}

int vpu_get_region_of_intereset(struct vpu_client *client,
		struct v4l2_crop *crop)
{
	struct vpu_prop_session_input    in_param;
	struct vpu_prop_session_output   out_param;
	struct vpu_dev_session *session = client ? client->session : 0;
	int ret = 0, port;
	if (!session)
		return -EPERM;

	port = get_port_number(crop->type);
	if (port < 0)
		return -EINVAL;

	if (port == INPUT_PORT) {
		ret = vpu_hw_session_g_input_params(session->id, &in_param);
		translate_roi_rect_to_api(&in_param.region_interest,
				&crop->c);
	} else {
		ret = vpu_hw_session_g_output_params(session->id, &out_param);
	translate_roi_rect_to_api(&out_param.dest_rect,
			&crop->c);
	}
	return ret;
}

int vpu_set_region_of_intereset(struct vpu_client *client,
		const struct v4l2_crop *crop)
{
	struct vpu_dev_session *session = client ? client->session : 0;
	int ret = 0, port = get_port_number(crop->type);
	if (!session)
		return -EPERM;
	if (port < 0)
		return -EINVAL;

	mutex_lock(&session->lock);
	memcpy(&session->port_info[port].roi, &crop->c, sizeof(crop->c));
	ret = commit_port_config(session, port, 0);

	mutex_unlock(&session->lock);
	return ret;
}

int vpu_enum_input(struct v4l2_input *inp)
{
	const struct vpu_io_desc *io;
	u32 index = inp->index;
	io = query_inputs_outputs(INPUT_PORT, index);
	if (!io)
		return -EINVAL;
	memset(inp, 0 , sizeof(*inp));
	inp->index = index;
	strlcpy(inp->name, io->description, sizeof(inp->name));
	return 0;
}

int vpu_get_input(struct vpu_client *client, unsigned int *i)
{
	if (!client || !client->session)
		return -EPERM;
	*i = client->session->port_info[INPUT_PORT].source;
	return 0;
}

int vpu_set_input(struct vpu_client *client, unsigned int i)
{
	int ret = 0;
	struct vpu_dev_session *session = client ? client->session : 0;
	if (!session)
		return -EPERM;

	if (i >= VPU_IN_SRC_MAX)
		return -EINVAL;

	/* Changing input/output only allowed if port is not streaming */
	mutex_lock(&session->lock);
	if (session->streaming_state == (0x1 << INPUT_PORT)) {
		ret = -EBUSY;
		goto exit_s_input;
	}

	session->port_info[INPUT_PORT].destination = i;

exit_s_input:
	mutex_unlock(&session->lock);
	return ret;
}

int vpu_enum_output(struct v4l2_output *out)
{
	const struct vpu_io_desc *io;
	u32 index = out->index;
	io = query_inputs_outputs(OUTPUT_PORT, index);
	if (!io)
		return -EINVAL;

	memset(out, 0 , sizeof(*out));
	out->index = index;
	strlcpy(out->name, io->description, sizeof(out->name));
	return 0;
}

int vpu_get_output(struct vpu_client *client, unsigned int *i)
{
	if (!client || !client->session)
		return -EPERM;
	*i = client->session->port_info[OUTPUT_PORT].destination;
	return 0;
}

int vpu_set_output(struct vpu_client *client, unsigned int i)
{
	int ret = 0;
	struct vpu_dev_session *session = client ? client->session : 0;
	if (!session)
		return -EPERM;

	if (i >= VPU_OUT_DEST_MAX)
		return -EINVAL;

	mutex_lock(&session->lock);
	if (session->streaming_state == (0x1 << OUTPUT_PORT)) {
		ret = -EBUSY;
		goto exit_s_output;
	}

	session->port_info[OUTPUT_PORT].destination = i;

exit_s_output:
	mutex_unlock(&session->lock);
	return ret;
}

int vpu_get_control(struct vpu_client *client, struct vpu_control *control)
{
	struct vpu_dev_session *session = client ? client->session : 0;
	if (!session)
		return -EPERM;

	return apply_vpu_control(session, 0, control);
}

int vpu_set_control(struct vpu_client *client, struct vpu_control *control)
{
	struct vpu_dev_session *session = client ? client->session : 0;
	if (!session)
		return -EPERM;

	return apply_vpu_control(session, 1, control);
}

int vpu_get_control_extended(struct vpu_client *client,
		struct vpu_control_extended *control)
{
	/* Don't check for session, since ioctl can address system */
	return apply_vpu_control_extended(client, 0, control);
}

int vpu_set_control_extended(struct vpu_client *client,
		struct vpu_control_extended *control)
{
	/* Don't check for session, since ioctl can address system */
	return apply_vpu_control_extended(client, 1, control);
}

/*
 * Streaming I/O operations
 */
static inline int __is_tunneling_client(struct vpu_dev_session *session,
		int port)
{
	/*
	 * Checks if this is a tunneling client
	 * Clients doing tunneling I/O can only call streamon and streamoff
	 */
	return (session->port_info[port].source != 0);
}

int vpu_reqbufs(struct vpu_client *client, struct v4l2_requestbuffers *req)
{
	struct vpu_dev_session *session = client ? client->session : 0;
	int ret = 0, port;
	if (!session)
		return -EPERM;

	port = get_port_number(req->type);
	if (port < 0) {
		dprintk(VPU_ERR, "%s: Invalid buffer type %d\n", __func__,
				req->type);
		return -EINVAL;
	}

	VPU_ENTER_FUNC("count = %d", req->count);

	mutex_lock(&session->que_lock[port]);
	if (session->io_client[port] != client && session->io_client[port])
		ret = -EBUSY;
	else if (__is_tunneling_client(session, port)) {
		dprintk(VPU_ERR, "%s: tunneling client can't request buffers\n",
				__func__);
		ret = -EINVAL;
	} else {
		ret = vb2_reqbufs(&session->vbqueue[port], req);
		if (!ret) {
			if (req->count == 0)
				session->io_client[port] = NULL;
			else
				session->io_client[port] = client;
		} else
			dprintk(VPU_ERR, "%s: reqbufs returned error code %d\n",
				__func__, ret);
	}
	mutex_unlock(&session->que_lock[port]);
	return ret;
}

void __check_user_planes(struct vb2_queue *vbq, struct v4l2_buffer *b)
{
	/*
	 * check content of user planes array is mapped.
	 * If not then set flag for buf_init callback to map.
	 */
	struct vpu_buffer *vpu_buf;
	struct v4l2_plane *plane;
	int i;
	if (b->m.planes &&
	    b->type == vbq->type &&
	    b->index < vbq->num_buffers &&
	    b->length == vbq->bufs[b->index]->num_planes) {

		vpu_buf = to_vpu_buffer(vbq->bufs[b->index]);
		if (vpu_buf->vb.state != VB2_BUF_STATE_DEQUEUED)
			return;

		for (i = 0; i < b->length; i++) {
			plane = &b->m.planes[i];
			if (plane->m.fd == vpu_buf->planes[i].user_fd &&
				plane->length == vpu_buf->planes[i].length &&
				plane->data_offset ==
						vpu_buf->planes[i].data_offset)
				continue;
			else {
				vpu_buf->planes[i].new_plane = 1;
				vpu_buf->planes[i].user_fd = plane->m.fd;
				vpu_buf->planes[i].length = plane->length;
				vpu_buf->planes[i].data_offset =
						plane->data_offset;
			}
		}
	}
}

int vpu_qbuf(struct vpu_client *client, struct v4l2_buffer *b)
{
	struct vpu_dev_session *session = client ? client->session : 0;
	int ret = 0, port;
	if (!session)
		return -EPERM;

	port = get_port_number(b->type);
	if (port < 0) {
		dprintk(VPU_ERR, "%s: Invalid type %d\n", __func__, b->type);
		return -EINVAL;
	}

	dprintk(VPU_DBG, "%s, queuing buffer #%d on port %d\n",
			__func__, b->index, port);

	mutex_lock(&session->que_lock[port]);
	if (session->io_client[port] != client ||
			__is_tunneling_client(session, port))
		ret = -EINVAL;
	else {
		__check_user_planes(&session->vbqueue[port], b);
		ret = vb2_qbuf(&session->vbqueue[port], b);
	}
	mutex_unlock(&session->que_lock[port]);

	return ret;
}

int vpu_dqbuf(struct vpu_client *client, struct v4l2_buffer *b,
		bool nonblocking)
{
	struct vpu_dev_session *session = client ? client->session : 0;
	int ret = 0, port;
	if (!session)
		return -EPERM;

	port = get_port_number(b->type);
	if (port < 0) {
		dprintk(VPU_ERR, "%s: Invalid type %d\n", __func__, b->type);
		return -EINVAL;
	}

	mutex_lock(&session->que_lock[port]);
	if (session->io_client[port] != client ||
			__is_tunneling_client(session, port))
		ret = -EINVAL;
	else {
		ret = vb2_dqbuf(&session->vbqueue[port], b, nonblocking);
		if (!ret && b->m.planes) {
			int i;
			struct vpu_buffer *vpu_buf = to_vpu_buffer(
				session->vbqueue[port].bufs[b->index]);
			for (i = 0; i < b->length; i++)
				b->m.planes[i].data_offset =
					vpu_buf->planes[i].data_offset;
		}
	}
	mutex_unlock(&session->que_lock[port]);

	if (!ret)
		dprintk(VPU_DBG, "%s, buffer #%d dequeued on port %d\n",
			__func__, b->index , port);
	return ret;
}

static int __queue_pending_buffers(struct vpu_dev_session *session)
{
	int port, ret = 0;
	struct vpu_buffer *buff, *n;

	dprintk(VPU_DBG, "Enter function %s\n", __func__);
	BUG_ON(session->streaming_state != ALL_STREAMING);

	for (port = 0; port < NUM_VPU_PORTS; port++) {
		mutex_lock(&session->que_lock[port]);
		list_for_each_entry_safe(buff, n,
			&session->pending_list[port], buffers_entry)
		{
			if (port == INPUT_PORT)
				ret = vpu_hw_session_empty_buffer(session->id,
						buff);
			else
				ret = vpu_hw_session_fill_buffer(session->id,
						buff);

			if (ret) {
				dprintk(VPU_ERR, "%s, returning buffer\n",
						__func__);
				vb2_buffer_done(&buff->vb, VB2_BUF_STATE_ERROR);
			}
			list_del(&buff->buffers_entry);
		}
		mutex_unlock(&session->que_lock[port]);
	}
	return 0;
}

int vpu_flush_bufs(struct vpu_client *client, enum v4l2_buf_type type)
{
	struct vpu_dev_session *session = client ? client->session : 0;
	int ret = 0, port;
	if (!session)
		return -EPERM;

	port = get_port_number(type);
	if (port < 0) {
		dprintk(VPU_ERR, "%s: Invalid type %d\n", __func__, type);
		return -EINVAL;
	}

	dprintk(VPU_DBG, "%s on port %d\n", __func__, port);

	mutex_lock(&session->que_lock[port]);
	if (session->io_client[port] != client) {
		ret = -EINVAL;
		goto exit_flush;
	} else {
		/* pause, if session is streaming */
		if (session->streaming_state == ALL_STREAMING) {
			ret = vpu_hw_session_pause(session->id);
			if (ret) {
				dprintk(VPU_ERR, "Critical!!! Pause failed\n");
				goto exit_flush;
			}
		}

		ret = vpu_vb2_flush_buffers(session, port);

		if (session->streaming_state == ALL_STREAMING)
			ret = vpu_hw_session_resume(session->id);
	}

exit_flush:
	mutex_unlock(&session->que_lock[port]);
	return ret;
}

int vpu_streamon(struct vpu_client *client, enum v4l2_buf_type type)
{
	struct vpu_dev_session *session = client ? client->session : 0;
	int ret = 0, port;
	u32 temp_streaming = 0;
	if (!session)
		return -EPERM;

	port = get_port_number(type);
	if (port < 0) {
		dprintk(VPU_ERR, "%s: Invalid type %d\n", __func__, type);
		return -EINVAL;
	}

	mutex_lock(&session->lock); /* needed to sync streamon from two ports */

	mutex_lock(&session->que_lock[port]);
	if (session->io_client[port] != client &&
		(session->io_client[port] ||
		!__is_tunneling_client(session, port))) {
		ret = -EBUSY;
		goto early_exit_streamon;
	}
	dprintk(VPU_DBG, "Port %d streaming on\n", port);
	temp_streaming = session->streaming_state;

	ret = vb2_streamon(&session->vbqueue[port], type);
	if (ret)
		goto early_exit_streamon;

	if (temp_streaming & (0x1 << port))
		goto early_exit_streamon; /* This port already streaming */
	else {
		temp_streaming |= (0x1 << port);
		/* lock port if tunneling */
		if (__is_tunneling_client(session, port))
			session->io_client[port] = client;
	}

	if (temp_streaming != ALL_STREAMING)
		goto early_exit_streamon; /* wait for other port to streamon */

	mutex_unlock(&session->que_lock[port]);

	/* both ports are ready. commit configuration */
	ret = commit_initial_config(session);
	if (ret)
		goto late_exit_streamon;

	/* Start end-to-end session streaming */
	ret = vpu_hw_session_start(session->id);
	if (ret) {
		dprintk(VPU_ERR, "%s, channel start failed\n", __func__);
		goto late_exit_streamon;
	}

	session->streaming_state = temp_streaming; /* ALL_STREAMING */
	__queue_pending_buffers(session);

	dprintk(VPU_INFO, "Session streaming started successfully\n");

late_exit_streamon:
	if (ret) {
		/* TODO: How do we notify the streamed on sessions? */
		if (__is_tunneling_client(session, port))
			session->io_client[port] = NULL;
	}
	mutex_unlock(&session->lock);
	return ret;

early_exit_streamon:
	session->streaming_state = temp_streaming;
	mutex_unlock(&session->que_lock[port]);
	mutex_unlock(&session->lock);
	return ret;
}

int vpu_streamoff(struct vpu_client *client, enum v4l2_buf_type type)
{
	struct vpu_dev_session *session = client ? client->session : 0;
	int ret = 0, port;
	if (!session)
		return -EPERM;

	port = get_port_number(type);
	if (port < 0) {
		dprintk(VPU_ERR, "%s: Invalid type %d\n", __func__, type);
		return -EINVAL;
	}

	/* session lock needed to protect actions inside vb2_stream_off */
	mutex_lock(&session->lock);

	mutex_lock(&session->que_lock[port]);
	if (session->io_client[port] != client) {
		ret = -EINVAL;
		goto exit_stream_off;
	}
	dprintk(VPU_DBG, "Port %d streaming off\n", port);

	ret = vb2_streamoff(&session->vbqueue[port],
			session->vbqueue[port].type);
	if (ret)
		dprintk(VPU_ERR, "Critical!!! vb2_streamoff failed\n");

	/* unlock port if tunneling client */
	if (__is_tunneling_client(session, port))
		session->io_client[port] = NULL;

exit_stream_off:
	mutex_unlock(&session->que_lock[port]);
	mutex_unlock(&session->lock);
	return ret;
}

