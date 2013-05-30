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

#include <linux/module.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-event.h>

#include "vpu_v4l2.h"
#include "vpu_ioctl_internal.h"
#include "vpu_configuration.h"
#include "vpu_channel.h"
#include "vpu_debug.h"

/*
 * V4l2 interface *
 */
static int v4l2_vpu_open(struct file *file)
{
	return vpu_open_user_client(file);
}

static int v4l2_vpu_close(struct file *file)
{
	struct vpu_client *client = get_vpu_client(file->private_data);

	return vpu_close_client(client);
}

static unsigned int v4l2_vpu_poll(struct file *file,
				struct poll_table_struct *wait)
{
	unsigned int mask = 0;
	struct vpu_client *client = get_vpu_client(file->private_data);
	struct vpu_dev_session *session = client->session;

	dprintk(VPU_DBG, "Enter function %s\n", __func__);

	if (v4l2_event_pending(&client->vfh)) {
		dprintk(VPU_DBG, "event ready\n");
		mask |= POLLPRI;
	}
	poll_wait(file, &(client->vfh.wait), wait);

	if (!session)
		return mask;

	/* Poll vbqueues if client is doing streaming i/o */
	if (client == session->io_client[INPUT_PORT] &&
		!list_empty(&session->vbqueue[INPUT_PORT].queued_list))
		mask |= vb2_poll(&session->vbqueue[INPUT_PORT],	file, wait);

	if (client == session->io_client[OUTPUT_PORT] &&
		!list_empty(&session->vbqueue[OUTPUT_PORT].queued_list))
		mask |= vb2_poll(&session->vbqueue[OUTPUT_PORT], file, wait);

	return mask;
}

static const struct v4l2_file_operations v4l2_vpu_fops = {
	.owner = THIS_MODULE,
	.open = v4l2_vpu_open,
	.release = v4l2_vpu_close,
	.unlocked_ioctl = video_ioctl2,
	.poll = v4l2_vpu_poll,
};


/*
 * V4l2 IOCTLs
 */
static long v4l2_vpu_private_ioctls(struct file *file, void *fh,
		bool valid_prio, unsigned int cmd, void *arg)
{
	struct vpu_client *client = get_vpu_client(file->private_data);
	int num_session = 0, ret = 0;

	switch (cmd) {
	case VPU_QUERY_SESSIONS:
		dprintk(VPU_DBG, "Received ioctl VPU_QUERY_SESSIONS\n");
		ret = get_vpu_num_sessions((unsigned *)arg);
		break;

	case VPU_ATTACH_TO_SESSION:
		dprintk(VPU_DBG, "Received ioctl VPU_ATTACH_TO_SESSION\n");
		num_session = *((unsigned *)arg);
		ret = vpu_attach_client(client, num_session);
		break;

	case VPU_FLUSH_BUFS:
		dprintk(VPU_DBG, "Received ioctl VPU_FLUSH_BUFS\n");
		ret = vpu_flush_bufs(client, *((enum v4l2_buf_type *) arg));
		break;

	case VPU_G_CONTROL:
		dprintk(VPU_DBG, "Received ioctl VPU_G_CONTROL\n");
		ret = vpu_get_control(client, (struct vpu_control *) arg);
		break;

	case VPU_S_CONTROL:
		dprintk(VPU_DBG, "Received ioctl VPU_S_CONTROL\n");
		ret = vpu_set_control(client, (struct vpu_control *) arg);
		break;

	case VPU_G_CONTROL_EXTENDED:
		dprintk(VPU_DBG, "Received ioctl VPU_G_CONTROL_EXTENDED\n");
		ret = vpu_get_control_extended(client,
				(struct vpu_control_extended *) arg);
		break;

	case VPU_S_CONTROL_EXTENDED:
		dprintk(VPU_DBG, "Received ioctl VPU_S_CONTROL_EXTENDED\n");
		ret = vpu_set_control_extended(client,
				(struct vpu_control_extended *) arg);
		break;

	default:
		return -ENOTTY;
		break;
	}
	return (long) ret;
}

static int v4l2_vpu_querycap(struct file *file, void *fh,
		struct v4l2_capability *cap)
{
	struct vpu_dev_core *core = video_drvdata(file);
	VPU_ENTER_FUNC();

	strlcpy(cap->driver, VPU_DRV_NAME, sizeof(cap->driver));
	snprintf(cap->card, 32, "%s0", VPU_DRV_NAME);
	strlcpy(cap->bus_info, core->v4l2_dev.name, sizeof(cap->bus_info));
	cap->version = KERNEL_VERSION(0, 0, 1);
	cap->capabilities = V4L2_CAP_STREAMING | V4L2_CAP_VIDEO_M2M_MPLANE |
		 V4L2_CAP_VIDEO_CAPTURE_MPLANE | V4L2_CAP_VIDEO_OUTPUT_MPLANE;
	return 0;
}

int v4l2_vpu_enum_fmt(struct file *file, void *fh, struct v4l2_fmtdesc *f)
{
	return vpu_enum_fmt(f);
}

int v4l2_vpu_g_fmt(struct file *file, void *fh, struct v4l2_format *f)
{
	struct vpu_client *client = get_vpu_client(file->private_data);

	return vpu_get_fmt(client, f);
}

int v4l2_vpu_s_fmt(struct file *file, void *fh, struct v4l2_format *f)
{
	struct vpu_client *client = get_vpu_client(file->private_data);

	dprintk(VPU_DBG, "Received ioctl VIDIOC_S_FMT on port %d\n",
			get_port_number(f->type));

	return vpu_set_fmt(client, f);
}

int v4l2_vpu_try_fmt(struct file *file, void *fh, struct v4l2_format *f)
{
	struct vpu_client *client = get_vpu_client(file->private_data);

	dprintk(VPU_DBG, "Received ioctl VIDIOC_TRY_FMT on port %d\n",
				get_port_number(f->type));

	return vpu_try_fmt(client, f);
}

static int v4l2_vpu_g_parm(struct file *file, void *fh,
		struct v4l2_streamparm *parm)
{
	struct vpu_client *client = get_vpu_client(file->private_data);
	int ret = 0;

	ret |= vpu_get_framerate(client, &parm->parm.capture.timeperframe,
				parm->type);
	ret |= vpu_get_3d_mode(client, &parm->parm.capture.extendedmode,
				parm->type);
	return ret;
}

static int v4l2_vpu_s_parm(struct file *file, void *fh,
		struct v4l2_streamparm *parm)
{
	struct vpu_client *client = get_vpu_client(file->private_data);
	int ret = 0;

	dprintk(VPU_DBG, "Received ioctl VIDIOC_S_PARM on port %d\n",
			get_port_number(parm->type));

	if (parm->parm.capture.timeperframe.numerator != 0 &&
			parm->parm.capture.timeperframe.denominator != 0)
		ret |= vpu_set_framerate(client,
				&parm->parm.capture.timeperframe, parm->type);

	if (parm->parm.capture.extendedmode)
		ret |= vpu_set_3d_mode(client, &parm->parm.capture.extendedmode,
				parm->type);
	return ret;
}

static int v4l2_vpu_g_crop(struct file *file, void *fh, struct v4l2_crop *c)
{
	struct vpu_client *client = get_vpu_client(file->private_data);

	dprintk(VPU_DBG, "Received ioctl VIDIOC_G_CROP on port %d\n",
			get_port_number(c->type));

	return vpu_get_region_of_intereset(client, c);
}

static int v4l2_vpu_s_crop(struct file *file, void *fh,
			const struct v4l2_crop *c)
{
	struct vpu_client *client = get_vpu_client(file->private_data);

	dprintk(VPU_DBG, "Received ioctl VIDIOC_S_CROP on port %d\n",
			get_port_number(c->type));

	return vpu_set_region_of_intereset(client, c);
}

static int v4l2_vpu_enum_input(struct file *file, void *fh,
		struct v4l2_input *inp)
{
	return vpu_enum_input(inp);
}

static int v4l2_vpu_g_input(struct file *file, void *fh, unsigned int *i)
{
	struct vpu_client *client = get_vpu_client(file->private_data);

	return vpu_get_input(client, i);
}
static int v4l2_vpu_s_input(struct file *file, void *fh, unsigned int i)
{
	struct vpu_client *client = get_vpu_client(file->private_data);

	return vpu_set_input(client, i);
}

static int v4l2_vpu_enum_output(struct file *file, void *fh,
		struct v4l2_output *out)
{
	return vpu_enum_output(out);
}

static int v4l2_vpu_g_output(struct file *file, void *fh, unsigned int *i)
{
	struct vpu_client *client = get_vpu_client(file->private_data);

	return vpu_get_output(client, i);
}
static int v4l2_vpu_s_output(struct file *file, void *fh, unsigned int i)
{
	struct vpu_client *client = get_vpu_client(file->private_data);

	return vpu_set_output(client, i);
}

static int v4l2_vpu_reqbufs(struct file *file, void *fh,
			struct v4l2_requestbuffers *rb)
{
	struct vpu_client *client = get_vpu_client(file->private_data);

	dprintk(VPU_DBG, "Received ioctl VIDIOC_REQBUFS on port %d\n",
			get_port_number(rb->type));

	return vpu_reqbufs(client, rb);
}

static int v4l2_vpu_qbuf(struct file *file, void *fh, struct v4l2_buffer *b)
{
	struct vpu_client *client = get_vpu_client(file->private_data);

	dprintk(VPU_DBG, "Received ioctl VIDIOC_QBUF on port %d\n",
			get_port_number(b->type));

	return vpu_qbuf(client, b);
}

static int v4l2_vpu_dqbuf(struct file *file, void *fh, struct v4l2_buffer *b)
{
	struct vpu_client *client = get_vpu_client(file->private_data);

	dprintk(VPU_DBG, "Received ioctl VIDIOC_DQBUF on port %d\n",
			get_port_number(b->type));

	return vpu_dqbuf(client, b, file->f_flags & O_NONBLOCK);
}

static int v4l2_vpu_streamon(struct file *file, void *fh, enum v4l2_buf_type i)
{
	struct vpu_client *client = get_vpu_client(file->private_data);

	dprintk(VPU_DBG, "Received ioctl VIDIOC_STREAMON on port %d\n",
			get_port_number(i));

	return vpu_streamon(client, i);
}

static int v4l2_vpu_streamoff(struct file *file, void *fh, enum v4l2_buf_type i)
{
	struct vpu_client *client = get_vpu_client(file->private_data);

	dprintk(VPU_DBG, "Received ioctl VIDIOC_STREAMOFF on port %d\n",
			get_port_number(i));

	return vpu_streamoff(client, i);
}

static int v4l2_vpu_subscribe_event(struct v4l2_fh *fh,
				const struct v4l2_event_subscription *sub)
{
	int ret = 0;
	if (!fh)
		return -EPERM; /* client was not attached */

	dprintk(VPU_DBG, "Received ioctl VIDIOC_SUBSCRIBE_EVENT\n");
	dprintk(VPU_DBG, "Enter function %s\n", __func__);

	if (sub->type == V4L2_EVENT_ALL) {
		struct v4l2_event_subscription each_sub;
		memcpy(&each_sub, sub, sizeof(each_sub));

		each_sub.type = VPU_EVENT_START;
		do {
			ret = v4l2_event_subscribe(fh, &each_sub,
					VPU_EVENT_Q_SIZE, NULL);
			if (ret < 0) {
				v4l2_event_unsubscribe(fh, sub);
				return ret;
			}
			each_sub.type++;
		} while (each_sub.type < VPU_EVENT_END);
	} else if (sub->type < VPU_EVENT_START ||
				sub->type >= VPU_EVENT_END)
		ret = -EINVAL;
	else
		ret = v4l2_event_subscribe(fh, sub, VPU_EVENT_Q_SIZE, NULL);

	return ret;
}

static const struct v4l2_ioctl_ops v4l2_vpu_ioctl_ops = {
	.vidioc_querycap = v4l2_vpu_querycap,

	/* video format setting */
	.vidioc_enum_fmt_vid_cap_mplane = v4l2_vpu_enum_fmt,
	.vidioc_enum_fmt_vid_out_mplane = v4l2_vpu_enum_fmt,
	.vidioc_g_fmt_vid_cap_mplane = v4l2_vpu_g_fmt,
	.vidioc_g_fmt_vid_out_mplane = v4l2_vpu_g_fmt,
	.vidioc_s_fmt_vid_cap_mplane = v4l2_vpu_s_fmt,
	.vidioc_s_fmt_vid_out_mplane = v4l2_vpu_s_fmt,
	.vidioc_try_fmt_vid_cap_mplane = v4l2_vpu_try_fmt,
	.vidioc_try_fmt_vid_out_mplane = v4l2_vpu_try_fmt,

	/* framerate and 2D/3D mode */
	.vidioc_g_parm = v4l2_vpu_g_parm,
	.vidioc_s_parm = v4l2_vpu_s_parm,

	/* Cropping ioctls (region of interest) */
	.vidioc_cropcap = NULL,
	.vidioc_g_crop = v4l2_vpu_g_crop,
	.vidioc_s_crop = v4l2_vpu_s_crop,

	/* ioctls to set/get session source and destination */
	.vidioc_enum_input = v4l2_vpu_enum_input,
	.vidioc_g_input = v4l2_vpu_g_input,
	.vidioc_s_input = v4l2_vpu_s_input,
	.vidioc_enum_output = v4l2_vpu_enum_output,
	.vidioc_g_output = v4l2_vpu_g_output,
	.vidioc_s_output = v4l2_vpu_s_output,

	/* streaming I/O ioctls */
	.vidioc_reqbufs = v4l2_vpu_reqbufs,
	.vidioc_prepare_buf = NULL,
	.vidioc_qbuf = v4l2_vpu_qbuf,
	.vidioc_dqbuf = v4l2_vpu_dqbuf,
	.vidioc_streamon = v4l2_vpu_streamon,
	.vidioc_streamoff = v4l2_vpu_streamoff,

	/* Subscribe/unsubscribe to events (use VIDIOC_DQEVENT to get events) */
	.vidioc_subscribe_event = v4l2_vpu_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe, /* generic unsub */

	/* Custom ioctls */
	.vidioc_default = v4l2_vpu_private_ioctls,
};


/*
 * Module init / Probe *
 */
static void vpu_video_device_release(struct video_device *vdev) {}

static void deinit_vpu_sessions(struct vpu_dev_core *core)
{
	int i;
	struct vpu_dev_session *session = 0;
	struct vpu_client *client, *n;
	for (i = 0; i < VPU_NUM_SESSIONS; i++) {
		session = core->sessions[i];
		if (!session)
			break;

		/* free all attached clients */
		list_for_each_entry_safe(client, n,
				&session->clients_list, clients_entry) {
			vpu_close_client(client);
		}

		devm_kfree(core->dev, session);
		core->sessions[i] = 0;
	}

	/* free any other unattached clients */
	list_for_each_entry_safe(client, n,
				&core->unattached_list, clients_entry) {
		vpu_close_client(client);
	}
}

static int init_vpu_sessions(struct vpu_dev_core *core)
{
	int ret = 0, i;
	struct vpu_dev_session *session = 0;
	for (i = 0; i < VPU_NUM_SESSIONS; i++) {
		session = devm_kzalloc(core->dev, sizeof(*session), GFP_KERNEL);
		if (!session) {
			ret = -ENOMEM;
			goto err_enomem;
		}
		session->id = i;
		session->core = core;

		mutex_init(&session->lock);
		INIT_LIST_HEAD(&session->clients_list);

		mutex_init(&session->que_lock[INPUT_PORT]);
		mutex_init(&session->que_lock[OUTPUT_PORT]);
		INIT_LIST_HEAD(&session->pending_list[INPUT_PORT]);
		INIT_LIST_HEAD(&session->pending_list[OUTPUT_PORT]);

		ret = vpu_vb2_queue_init(&session->vbqueue[INPUT_PORT],
				V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE, session);
		ret |= vpu_vb2_queue_init(&session->vbqueue[OUTPUT_PORT],
				V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE, session);
		if (ret)
			goto err_free_mem;

		core->sessions[i] = session;
	}
	return ret;

err_free_mem:
	devm_kfree(core->dev, session);
err_enomem:
	deinit_vpu_sessions(core); /* free successfully loaded sessions */
	dprintk(VPU_ERR, "init_vpu_sessions failed\n");
	return ret;
}

static int vpu_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct vpu_dev_core *core = 0;

	dprintk(VPU_DBG, "Enter function %s\n", __func__);

	/* Allocate global VPU core struct */
	core = devm_kzalloc(&pdev->dev, sizeof(*core), GFP_KERNEL);
	if (!core)
		return -ENOMEM;

	/* Initialize core's data */
	INIT_LIST_HEAD(&core->unattached_list);
	mutex_init(&core->lock);
	core->dev = &pdev->dev;
	platform_set_drvdata(pdev, core);

	/* read/get platform resources from Device Tree */
	ret = read_vpu_platform_resources(&core->resources, pdev);
	if (ret) {
		dprintk(VPU_ERR, "%s, Failed to read platform resources\n",
				__func__);
		goto err_free_core;
	}

	ret = register_vpu_iommu_domains(&core->resources);
	if (ret) {
		dprintk(VPU_ERR, "%s, Failed to register iommu domains\n",
				__func__);
		goto err_free_platform_resources;
	}

	/* init IPC system */
	ret = vpu_hw_sys_init(&core->resources);
	if (ret) {
		dprintk(VPU_ERR, "%s, VPU system init failed\n", __func__);
		goto err_unregister_iommu_domains;
	}

	/* Initialize VPU sessions */
	ret = init_vpu_sessions(core);
	if (ret)
		goto err_deinit_vpu_system;

	setup_vpu_controls();

	/* register VPU v4l2 device */
	ret = v4l2_device_register(&pdev->dev, &core->v4l2_dev);
	if (ret)
		goto err_deinit_sessions;

	/* register video_device */
	strlcpy(core->vdev.name, VPU_DRV_NAME, sizeof(core->vdev.name));
	core->vdev.v4l2_dev = &core->v4l2_dev;
	core->vdev.fops = &v4l2_vpu_fops;
	core->vdev.ioctl_ops = &v4l2_vpu_ioctl_ops;
	core->vdev.release = vpu_video_device_release;
	/* mem2mem device: can transmit using the same video device. */
	core->vdev.vfl_dir = VFL_DIR_M2M;
	video_set_drvdata(&core->vdev, core);
	ret = video_register_device(&core->vdev, VFL_TYPE_GRABBER, -1);
	if (ret) {
		dprintk(VPU_ERR, "%s, Failed to register video_device\n",
				__func__);
		goto err_unregister_v4l2_device;
	}

	/* initialize debugfs */
	core->debugfs_root = init_vpu_debugfs(core);
	if (IS_ERR(core->debugfs_root)) {
		ret = PTR_ERR(core->debugfs_root);
		goto err_unregister_video_device;
	}

	dprintk(VPU_DBG, "%s Successful\n", __func__);
	return 0;

err_unregister_video_device:
	video_unregister_device(&core->vdev);
err_unregister_v4l2_device:
	v4l2_device_unregister(&core->v4l2_dev);
err_deinit_sessions:
	deinit_vpu_sessions(core);
err_deinit_vpu_system:
	vpu_hw_sys_cleanup();
err_unregister_iommu_domains:
	unregister_vpu_iommu_domains(&core->resources);
err_free_platform_resources:
	free_vpu_platform_resources(&core->resources);
err_free_core:
	devm_kfree(&pdev->dev, core);
	return ret;
}

static int vpu_remove(struct platform_device *pdev)
{
	struct vpu_dev_core *core = platform_get_drvdata(pdev);
	dprintk(VPU_DBG, "Enter function %s\n", __func__);

	cleanup_vpu_debugfs(core->debugfs_root);
	video_unregister_device(&core->vdev);
	v4l2_device_unregister(&core->v4l2_dev);
	deinit_vpu_sessions(core);
	vpu_hw_sys_cleanup();
	unregister_vpu_iommu_domains(&core->resources);
	free_vpu_platform_resources(&core->resources);
	devm_kfree(&pdev->dev, core);

	return 0;
}


static const struct of_device_id vpu_dt_match[] = {
	{.compatible = "qcom,vpu"},
	{}
};

MODULE_DEVICE_TABLE(of, msm_csid_dt_match);

static struct platform_driver vpu_platform_driver = {
	.driver   = {
		.name = VPU_DRV_NAME,
		.owner  = THIS_MODULE,
		.of_match_table = vpu_dt_match,
	},
	.probe    = vpu_probe,
	.remove   = vpu_remove,
};

static int __init vpu_init_module(void)
{
	dprintk(VPU_DBG, "Enter function %s\n", __func__);
	return platform_driver_register(&vpu_platform_driver);
}

static void __exit vpu_exit_module(void)
{
	dprintk(VPU_DBG, "Enter function %s\n", __func__);
	platform_driver_unregister(&vpu_platform_driver);
}

module_init(vpu_init_module);
module_exit(vpu_exit_module);
MODULE_DESCRIPTION("MSM VPU driver");
MODULE_LICENSE("GPL v2");

