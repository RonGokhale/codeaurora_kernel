/* Copyright (c) 2011, Code Aurora Forum. All rights reserved.
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

#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/list.h>
#include <linux/ioctl.h>
#include <linux/spinlock.h>
#include <linux/videodev2.h>
#include <linux/proc_fs.h>
#include <linux/vmalloc.h>

#include <media/v4l2-dev.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-device.h>

#include <linux/android_pmem.h>

#include "msm.h"
#include "msm_ispif.h"

#ifdef CONFIG_MSM_CAMERA_DEBUG
#define D(fmt, args...) printk(KERN_DEBUG "msm_mctl: " fmt, ##args)
#else
#define D(fmt, args...) do {} while (0)
#endif

#define MAX_VIDEO_MEM 16
/* VFE required buffer number for streaming */
#define VFE_OUT1_BUF 3
static struct msm_isp_color_fmt msm_isp_formats[] = {
	{
	.name	   = "NV21YUV",
	.depth	  = 12,
	.bitsperpxl = 8,
	.fourcc	 = V4L2_PIX_FMT_NV21,
	.pxlcode	= V4L2_MBUS_FMT_YUYV8_2X8, /* YUV sensor */
	.colorspace = V4L2_COLORSPACE_JPEG,
	},
	{
	.name	   = "NV21BAYER",
	.depth	  = 8,
	.bitsperpxl = 8,
	.fourcc	 = V4L2_PIX_FMT_NV21,
	.pxlcode	= V4L2_MBUS_FMT_SBGGR8_1X8, /* Bayer sensor */
	.colorspace = V4L2_COLORSPACE_JPEG,
	},
};
/* master controller instance counter
static atomic_t mctl_instance = ATOMIC_INIT(0);
*/
/*
 *  Videobuf operations
 */
static void free_buffer(struct videobuf_queue *vq,
			struct msm_frame_buffer *buf)
{
	struct videobuf_buffer *vb = &buf->vidbuf;

	BUG_ON(in_interrupt());

	D("%s (vb=0x%p) 0x%08lx %d\n", __func__,
			vb, vb->baddr, vb->bsize);

	/* This waits until this buffer is out of danger, i.e.,
	 * until it is no longer in STATE_QUEUED or STATE_ACTIVE */
	videobuf_waiton(vq, vb, 0, 0);
	videobuf_pmem_contig_free(vq, vb);
	vb->state = VIDEOBUF_NEEDS_INIT;
}

/* Setup # of buffers and size of each buffer for the videobuf_queue.
   This is called when videobuf_reqbufs() is called, so this function
   should tell how many buffer should be used and how big the size is.

   The caller will allocate the real buffers, either in user space or
   in kernel */
static int msm_vidbuf_setup(struct videobuf_queue *vq, unsigned int *count,
							unsigned int *size)
{
	/* get the video device */
	struct msm_cam_v4l2_device *pcam = vq->priv_data;

	D("%s\n", __func__);
	if (!pcam || !count || !size) {
		D("%s error : invalid input\n", __func__);
		return -EINVAL;
	}

	/* at least we should provide 3 buffers to VFE and 1 in buffer queue*/
	if (*count <= VFE_OUT1_BUF)
		*count = VFE_OUT1_BUF + 1;

	/* we support only NV21 format for any input mediabus format */
	D("%s width = %d\n", __func__, pcam->vid_fmt.fmt.pix.width);
	D("%s height = %d\n", __func__, pcam->vid_fmt.fmt.pix.height);
	*size = pcam->vid_fmt.fmt.pix.width *
		pcam->vid_fmt.fmt.pix.height * 3/2;
	/* if the total mem size is bigger than available memory,
	 * reduce the count*/
	while ((*size) * (*count) > MAX_VIDEO_MEM * 1024 * 1024)
		(*count)--;

	/* if there are not buffers for VFE then fail*/
	if (*count < VFE_OUT1_BUF + 1) {
		D("%s error : out of memory input\n", __func__);
		return -ENOMEM;
	}

	D("count=%d, size=%d\n", *count, *size);

	return 0;
}

/* Prepare the buffer before it is put into the videobuf_queue for streaming.
   This is called when videobuf_qbuf() is called, so this function should
   setup the video buffer to receieve the VFE output. */
static int msm_vidbuf_prepare(struct videobuf_queue *vq,
	struct videobuf_buffer *vb, enum v4l2_field field)
{
	int rc = 0;
	/*struct msm_cam_v4l2_device *pcam = vq->priv_data;*/
	struct msm_cam_v4l2_device *pcam = NULL;
	struct msm_frame_buffer *buf = NULL;

	D("%s\n", __func__);
	if (!vb || !vq) {
		D("%s error : input is NULL\n", __func__);
		return -EINVAL;
	}
	pcam = vq->priv_data;
	buf = container_of(vb, struct msm_frame_buffer, vidbuf);

	if (!pcam || !buf) {
		D("%s error : pointer is NULL\n", __func__);
		return -EINVAL;
	}

	D("%s (vb=0x%p) 0x%08lx %d\n", __func__,
		vb, vb->baddr, vb->bsize);

	/* by this time vid_fmt should be already set */
	/* return error if it is not */
	if ((pcam->vid_fmt.fmt.pix.width == 0) ||
		(pcam->vid_fmt.fmt.pix.height == 0)) {
		D("%s error : pcam vid_fmt is not set\n", __func__);
		return -EINVAL;
	}

	buf->inuse = 1;

	D("buf->pxlcode=%d, pcam->sensor_pxlcode=%d, vb->width=%d,"
		"pcam->vid_fmt.fmt.pix.width = %d, vb->height = %d,"
		"pcam->vid_fmt.fmt.pix.height=%d, vb->field=%d, field=%d\n",
		buf->pxlcode, pcam->sensor_pxlcode, vb->width,
		pcam->vid_fmt.fmt.pix.width, vb->height,
		pcam->vid_fmt.fmt.pix.height, vb->field, field);

	if (buf->pxlcode != pcam->sensor_pxlcode ||
		vb->width   != pcam->vid_fmt.fmt.pix.width ||
		vb->height	!= pcam->vid_fmt.fmt.pix.height ||
		vb->field   != field) {
		buf->pxlcode  = pcam->sensor_pxlcode;
		vb->width = pcam->vid_fmt.fmt.pix.width;
		vb->height  = pcam->vid_fmt.fmt.pix.height;
		vb->field = field;
		vb->state = VIDEOBUF_NEEDS_INIT;
		D("VIDEOBUF_NEEDS_INIT\n");
	}

	/* For us, output will always be in NV21 format at least for now */
	vb->size = pcam->vid_fmt.fmt.pix.width * vb->height * 3/2;

	D("vb->size=%lu, vb->bsize=%u, vb->baddr=0x%x\n",
		vb->size, vb->bsize, (uint32_t)vb->baddr);

	if (0 != vb->baddr && vb->bsize < vb->size) {
		D("Something wrong vb->size=%lu, vb->bsize=%u,\
					vb->baddr=0x%x\n",
					vb->size, vb->bsize,
					(uint32_t)vb->baddr);
		rc = -EINVAL;
		goto out;
	}

	if (vb->state == VIDEOBUF_NEEDS_INIT) {
		rc = videobuf_iolock(vq, vb, NULL);
		if (rc)
			goto fail;
		D("%s: setting buffer state to prepared\n", __func__);
		vb->state = VIDEOBUF_PREPARED;
	}

	buf->inuse = 0;

	/* finally if everything is oK, set the VIDEOBUF_PREPARED state*/
	if (0 == rc)
		vb->state = VIDEOBUF_PREPARED;
	return rc;

fail:
	free_buffer(vq, buf);

out:
	buf->inuse = 0;
	return rc;
}

/* Called under spin_lock_irqsave(q->irqlock, flags) in videobuf-core.c*/
static void msm_vidbuf_queue(struct videobuf_queue *vq,
				struct videobuf_buffer *vb)
{
	/*struct msm_cam_v4l2_device *pcam = vq->priv_data;*/
	struct msm_cam_v4l2_device *pcam = NULL;
	unsigned long phyaddr = 0;
	int rc;

	D("%s\n", __func__);
	if (!vb || !vq) {
		D("%s error : input is NULL\n", __func__);
		return ;
	}
	pcam = vq->priv_data;
	if (!pcam) {
		D("%s error : pcam is NULL\n", __func__);
		return;
	}
	D("%s (vb=0x%p) 0x%08lx %d\n", __func__, vb, vb->baddr, vb->bsize);


	vb->state = VIDEOBUF_QUEUED;
	if (vq->streaming) {
		struct msm_frame frame;
		/* we are returning a buffer to the queue */
		struct videobuf_contig_pmem *mem = vb->priv;
		/* get the physcial address of the buffer */
		phyaddr = (unsigned long) videobuf_to_pmem_contig(vb);

		D("%s buffer type is %d\n", __func__, mem->buffer_type);
		frame.path = mem->buffer_type;
		frame.buffer = 0;
		frame.y_off = mem->y_off;
		frame.cbcr_off = mem->cbcr_off;
		/* now release frame to vfe */
		{
			struct msm_vfe_cfg_cmd cfgcmd;
			cfgcmd.cmd_type = CMD_FRAME_BUF_RELEASE;
			cfgcmd.value	= (void *)&frame;
			/* yyan: later change to mctl APIs*/
			rc = pcam->mctl.sync.vfefn.vfe_config
			(&cfgcmd, &phyaddr);
		}
	}
}

/* This will be called when streamingoff is called. */
static void msm_vidbuf_release(struct videobuf_queue *vq,
				struct videobuf_buffer *vb)
{
	struct msm_cam_v4l2_device *pcam = vq->priv_data;
	struct msm_frame_buffer *buf = container_of(vb, struct msm_frame_buffer,
									vidbuf);

	D("%s\n", __func__);
	if (!pcam || !vb || !vq) {
		D("%s error : input is NULL\n", __func__);
		return ;
	}
#ifdef DEBUG
	D("%s (vb=0x%p) 0x%08lx %d\n", __func__,
		vb, vb->baddr, vb->bsize);

	switch (vb->state) {
	case VIDEOBUF_ACTIVE:
		D("%s (active)\n", __func__);
		break;
	case VIDEOBUF_QUEUED:
		D("%s (queued)\n", __func__);
		break;
	case VIDEOBUF_PREPARED:
		D("%s (prepared)\n", __func__);
		break;
	default:
		D("%s (unknown) state = %d\n", __func__, vb->state);
		break;
	}
#endif

	/* free the buffer */
	free_buffer(vq, buf);
}


static struct videobuf_queue_ops msm_vidbuf_ops = {
	.buf_setup  = msm_vidbuf_setup,
	.buf_prepare  = msm_vidbuf_prepare,
	.buf_queue  = msm_vidbuf_queue,
	.buf_release  = msm_vidbuf_release,
};

/* prepare a video buffer queue for a vl42 device*/
static int msm_vidbuf_init(struct msm_cam_v4l2_device *pcam,
						   struct videobuf_queue *q)
{
	int rc = 0;
	struct resource *res;
	struct platform_device *pdev = NULL;

	D("%s\n", __func__);
	if (!pcam || !q) {
		D("%s error : input is NULL\n", __func__);
		return -EINVAL;
	} else
		pdev = pcam->mctl.sync.pdev;

	if (!pdev) {
		D("%s error : pdev is NULL\n", __func__);
		return -EINVAL;
	}

	/* first check if we have resources */
	res = platform_get_resource(pdev, IORESOURCE_DMA, 0);
	if (res) {
		D("res->start = 0x%x\n", (u32)res->start);
		D("res->size = 0x%x\n", (u32)resource_size(res));
		D("res->end = 0x%x\n", (u32)res->end);
		rc = dma_declare_coherent_memory(&pdev->dev, res->start,
			res->start,
			resource_size(res),
			DMA_MEMORY_MAP |
			DMA_MEMORY_EXCLUSIVE);
		if (!rc) {
			D("%s: Unable to declare coherent memory.\n", __func__);
			rc = -ENXIO;
			return rc;
		}

		/*pcam->memsize = resource_size(res);*/
		D("%s: found DMA capable resource\n", __func__);
	} else {
		D("%s: no DMA capable resource\n", __func__);
		return -ENOMEM;
	}

	spin_lock_init(&pcam->vb_irqlock);

	videobuf_queue_pmem_contig_init(q, &msm_vidbuf_ops, &pdev->dev,
		&pcam->vb_irqlock,
		V4L2_BUF_TYPE_VIDEO_CAPTURE,
		V4L2_FIELD_NONE,
		sizeof(struct msm_frame_buffer), pcam, NULL);


	return 0;
}

/*
 *  V4l2 subdevice operations
 */
static	int mctl_subdev_log_status(struct v4l2_subdev *sd)
{
	return -EINVAL;
}

static long mctl_subdev_ioctl(struct v4l2_subdev *sd,
				 unsigned int cmd, void *arg)
{
	struct msm_cam_media_controller *pmctl = NULL;
	if (!sd) {
		D("%s: param is NULL", __func__);
		return -EINVAL;
	} else
		pmctl = (struct msm_cam_media_controller *)
		v4l2_get_subdevdata(sd);


	return -EINVAL;
}


static int mctl_subdev_g_mbus_fmt(struct v4l2_subdev *sd,
					 struct v4l2_mbus_framefmt *mf)
{
	return -EINVAL;
}

static struct v4l2_subdev_core_ops mctl_subdev_core_ops = {
	.log_status = mctl_subdev_log_status,
	.ioctl = mctl_subdev_ioctl,
};

static struct v4l2_subdev_video_ops mctl_subdev_video_ops = {
	.g_mbus_fmt = mctl_subdev_g_mbus_fmt,
};

static struct v4l2_subdev_ops mctl_subdev_ops = {
	.core = &mctl_subdev_core_ops,
	.video  = &mctl_subdev_video_ops,
};

static int msm_get_sensor_info(struct msm_sync *sync, void __user *arg)
{
	int rc = 0;
	struct msm_camsensor_info info;
	struct msm_camera_sensor_info *sdata;

	if (copy_from_user(&info,
			arg,
			sizeof(struct msm_camsensor_info))) {
		ERR_COPY_FROM_USER();
		return -EFAULT;
	}

	sdata = sync->pdev->dev.platform_data;
	D("%s: sensor_name %s\n", __func__, sdata->sensor_name);

	memcpy(&info.name[0], sdata->sensor_name, MAX_SENSOR_NAME);
	info.flash_enabled = sdata->flash_data->flash_type !=
					MSM_CAMERA_FLASH_NONE;

	/* copy back to user space */
	if (copy_to_user((void *)arg,
				&info,
				sizeof(struct msm_camsensor_info))) {
		ERR_COPY_TO_USER();
		rc = -EFAULT;
	}

	return rc;
}

/* called by other subdev to notify any changes*/

static int msm_mctl_notify(struct msm_cam_media_controller *p_mctl,
			unsigned int notification, void *arg)
{
	int rc = -EINVAL;
	struct msm_ispif_params ispif_params;

	/* if the CID changed, then reconfig the ISPIF*/
	if (NOTIFY_CID_CHANGE == notification) {
		if (p_mctl->ispif_fns->ispif_config) {
			ispif_params.intftype = PIX0;
			ispif_params.cid_mask = 0x0001;
			ispif_params.csid = 0x01;

			rc = p_mctl->ispif_fns->ispif_config(&ispif_params, 1);
			if (rc < 0)
				return rc;
			rc = p_mctl->ispif_fns->ispif_start_intf_transfer
					(&ispif_params);
			if (rc < 0)
				return rc;
			msleep(20);
		}
	}

	return rc;
}

/* called by the server or the config nodes to handle user space
  commands*/
static int msm_mctl_cmd(struct msm_cam_media_controller *p_mctl,
			unsigned int cmd, unsigned long arg)
{
	int rc = -EINVAL;
	void __user *argp = (void __user *)arg;
	if (!p_mctl) {
		D("%s: param is NULL", __func__);
		return -EINVAL;
	}
	D("%s start cmd = %d\n", __func__, _IOC_NR(cmd));

	/* ... call sensor, ISPIF or VEF subdev*/
	switch (cmd) {
		/* sensor config*/
	case MSM_CAM_IOCTL_GET_SENSOR_INFO:
			rc = msm_get_sensor_info(&p_mctl->sync, argp);
			break;

	case MSM_CAM_IOCTL_SENSOR_IO_CFG:
			rc = p_mctl->sync.sctrl.s_config(argp);
			break;

			/* ISFIF config*/

	default:
		/* ISP config*/
		rc = p_mctl->isp_sdev->isp_config(p_mctl, cmd, arg);
		break;
	}
	D("%s: !!! cmd = %d, rc = %d\n", __func__, _IOC_NR(cmd), rc);
	return rc;
}

static int msm_sync_init(struct msm_sync *sync,
	struct platform_device *pdev, struct msm_sensor_ctrl *sctrl)
{
	int rc = 0;
	if (!sync) {
		D("%s: param is NULL", __func__);
		return -EINVAL;
	}

	sync->sdata = pdev->dev.platform_data;

	wake_lock_init(&sync->wake_lock, WAKE_LOCK_IDLE, "msm_camera");

	sync->pdev = pdev;
	sync->sctrl = *sctrl;
	sync->opencnt = 0;
	mutex_init(&sync->lock);
	D("%s: initialized %s\n", __func__, sync->sdata->sensor_name);
	return rc;
}

static int msm_mctl_open(struct msm_cam_media_controller *p_mctl,
				 const char *const apps_id)
{
	int rc = 0;
	struct msm_sync *sync = NULL;
	D("%s\n", __func__);
	if (!p_mctl) {
		D("%s: param is NULL", __func__);
		return -EINVAL;
	}

	/* msm_sync_init() muct be called before*/
	sync = &(p_mctl->sync);

	mutex_lock(&sync->lock);
	/* open sub devices - once only*/
	if (!sync->opencnt) {
		wake_lock(&sync->wake_lock);

		/* turn on clock */
		rc = msm_camio_sensor_clk_on(sync->pdev);
		if (rc < 0) {
			pr_err("%s: msm_camio_sensor_clk_on failed:%d\n",
			 __func__, rc);
			goto msm_open_done;
		}

		/* ISP first*/
		if (p_mctl->isp_sdev && p_mctl->isp_sdev->isp_open)
			rc = p_mctl->isp_sdev->isp_open(sync);

		if (rc < 0) {
			pr_err("%s: isp init failed: %d\n", __func__, rc);
			goto msm_open_done;
		}

		/* then sensor - move sub dev later*/
		if (sync->sctrl.s_init)
			rc = sync->sctrl.s_init(sync->sdata);

		if (rc < 0) {
			pr_err("%s: isp init failed: %d\n", __func__, rc);
			goto msm_open_done;
		}

		sync->apps_id = apps_id;
		sync->opencnt++;
	}

msm_open_done:
	mutex_unlock(&sync->lock);
	return rc;
}

static int msm_mctl_release(struct msm_cam_media_controller *p_mctl)
{
	if (p_mctl->sync.vfefn.vfe_release)
		p_mctl->sync.vfefn.vfe_release(p_mctl->sync.pdev);

	if (p_mctl->sync.sctrl.s_release)
		p_mctl->sync.sctrl.s_release();

	return 0;
}

int msm_mctl_init_user_formats(struct msm_cam_v4l2_device *pcam)
{
	struct v4l2_subdev *sd = pcam->isp.sdev;
	enum v4l2_mbus_pixelcode pxlcode;
	int numfmt = 0;
	int rc = 0;
	int i, j;

	D("%s\n", __func__);
	while (!v4l2_subdev_call(sd, video, enum_mbus_fmt, numfmt, &pxlcode))
		numfmt++;

	D("%s, numfmt = %d\n", __func__, numfmt);
	if (!numfmt)
		return -ENXIO;

	pcam->usr_fmts =
		vmalloc(numfmt * sizeof(struct msm_isp_color_fmt));
	if (!pcam->usr_fmts)
		return -ENOMEM;

	pcam->num_fmts = numfmt;

	D("Found %d supported formats.\n", pcam->num_fmts);

	/* from sensor to ISP.. fill the data structure */
	for (i = 0; i < numfmt; i++) {
		rc = v4l2_subdev_call(sd, video, enum_mbus_fmt, i, &pxlcode);
		D("rc is  %d\n", rc);
		if (rc < 0) {
			vfree(pcam->usr_fmts);
			return rc;
		}

		for (j = 0; j < ARRAY_SIZE(msm_isp_formats); j++) {
			/* find the corresponding format */
			if (pxlcode == msm_isp_formats[j].pxlcode) {
				pcam->usr_fmts[i] = msm_isp_formats[j];
				D("pcam->usr_fmts=0x%x\n", (u32)pcam->usr_fmts);
				D("format pxlcode 0x%x (0x%x) found\n",
					  pcam->usr_fmts[i].pxlcode,
					  pcam->usr_fmts[i].fourcc);
				break;
			}
		}
		if (j == ARRAY_SIZE(msm_isp_formats)) {
			D("format pxlcode 0x%x not found\n", pxlcode);
			vfree(pcam->usr_fmts);
			return -EINVAL;
		}
	}

	/* set the default pxlcode, in any case, it will be set through
	 * setfmt */
	pcam->sensor_pxlcode = pcam->usr_fmts[0].pxlcode;

	return 0;
}

/* this function plug in the implementation of a v4l2_subdev */
int msm_mctl_init_module(struct msm_cam_v4l2_device *pcam)
{

	struct msm_cam_media_controller *pmctl = NULL;
	D("%s\n", __func__);
	if (!pcam) {
		D("%s: param is NULL", __func__);
		return -EINVAL;
	} else
		pmctl = &pcam->mctl;

	/* init module sync object*/
	msm_sync_init(&pmctl->sync, pcam->pdev, &pcam->sctrl);

	/* init module operations*/
	pmctl->mctl_open = msm_mctl_open;
	pmctl->mctl_cmd = msm_mctl_cmd;
	pmctl->mctl_notify = msm_mctl_notify;
	pmctl->mctl_vidbuf_init = msm_vidbuf_init;
	pmctl->mctl_release = msm_mctl_release;

	/* init sub device*/
	v4l2_subdev_init(&(pmctl->mctl_sdev), &mctl_subdev_ops);
	v4l2_set_subdevdata(&(pmctl->mctl_sdev), pmctl);

	return 0;
}
