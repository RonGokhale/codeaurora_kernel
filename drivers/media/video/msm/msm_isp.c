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
#include "msm_vfe31.h"

#ifdef CONFIG_MSM_CAMERA_DEBUG
#define D(fmt, args...) printk(KERN_DEBUG "msm_isp: " fmt, ##args)
#else
#define D(fmt, args...) do {} while (0)
#endif
#define ERR_USER_COPY(to) pr_err("%s(%d): copy %s user\n", \
				__func__, __LINE__, ((to) ? "to" : "from"))
#define ERR_COPY_FROM_USER() ERR_USER_COPY(0)
#define ERR_COPY_TO_USER() ERR_USER_COPY(1)


#define PAD_TO_WORD(a)	  (((a) + 3) & ~3)

#define __CONTAINS(r, v, l, field) ({			   \
	typeof(r) __r = r;				  \
	typeof(v) __v = v;				  \
	typeof(v) __e = __v + l;				\
	int res = __v >= __r->field &&			  \
		__e <= __r->field + __r->len;		   \
	res;							\
})

#define CONTAINS(r1, r2, field) ({			  \
	typeof(r2) __r2 = r2;				   \
	__CONTAINS(r1, __r2->field, __r2->len, field);	  \
})

#define IN_RANGE(r, v, field) ({				\
	typeof(r) __r = r;				  \
	typeof(v) __vv = v;				 \
	int res = ((__vv >= __r->field) &&		  \
		(__vv < (__r->field + __r->len)));	  \
	res;							\
})

#define OVERLAPS(r1, r2, field) ({			  \
	typeof(r1) __r1 = r1;				   \
	typeof(r2) __r2 = r2;				   \
	typeof(__r2->field) __v = __r2->field;		  \
	typeof(__v) __e = __v + __r2->len - 1;		  \
	int res = (IN_RANGE(__r1, __v, field) ||		\
		IN_RANGE(__r1, __e, field));				 \
	res;							\
})

/* This will enqueue ISP events or signal buffer completion */
static int msm_isp_enqueue(struct msm_cam_media_controller *pmctl,
				struct msm_vfe_resp *data,
				enum msm_queue qtype)
{
	struct v4l2_event v4l2_evt;

	struct videobuf_queue *q;
	struct videobuf_buffer *buf = NULL;
	uint32_t buf_phyaddr = 0;
	struct msm_stats_buf stats;

	struct msm_isp_stats_event_ctrl *isp_event;
	/* struct msm_stats_buf stats; */
	int i;
	unsigned long flags = 0;
	isp_event = (struct msm_isp_stats_event_ctrl *)v4l2_evt.u.data;
	if (!data) {
		D("%s !!!!data = 0x%p\n", __func__, data);
		return -EINVAL;
	}

	D("%s data->type = %d\n", __func__, data->type);

	switch (qtype) {
	case MSM_CAM_Q_VFE_EVT:
	case MSM_CAM_Q_VFE_MSG:
		/* adsp event and message */
		v4l2_evt.type = V4L2_EVENT_PRIVATE_START +
					MSM_CAM_RESP_STAT_EVT_MSG;

		isp_event->resptype = MSM_CAM_RESP_STAT_EVT_MSG;

		/* 0 - msg from aDSP, 1 - event from mARM */
		isp_event->isp_data.isp_msg.type   = data->evt_msg.type;
		isp_event->isp_data.isp_msg.msg_id = data->evt_msg.msg_id;
		isp_event->isp_data.isp_msg.len	= data->evt_msg.len;

		D("%s: qtype %d length %d msd_id %d\n", __func__,
					qtype,
					isp_event->isp_data.isp_msg.len,
					isp_event->isp_data.isp_msg.msg_id);

		if ((data->type >= VFE_MSG_STATS_AEC) &&
			(data->type <=  VFE_MSG_STATS_WE)) {

			D("%s data->phy.sbuf_phy = 0x%x\n", __func__,
						data->phy.sbuf_phy);
			stats.buffer = msm_pmem_stats_ptov_lookup(&pmctl->sync,
							data->phy.sbuf_phy,
							&(stats.fd));
			if (!stats.buffer) {
				pr_err("%s: msm_pmem_stats_ptov_lookup error\n",
								__func__);
				isp_event->isp_data.isp_msg.len = 0;
			} else {
				memcpy((void *)isp_event->isp_data.isp_msg.data,
						&stats,
						sizeof(struct msm_stats_buf));
				isp_event->isp_data.isp_msg.len =
						sizeof(struct msm_stats_buf);
			}

		} else if ((data->evt_msg.len > 0) &&
			(data->evt_msg.len <= 48) && /* only 48 bytes */
			(data->type == VFE_MSG_GENERAL)) {
			memcpy((void *)isp_event->isp_data.isp_msg.data,
						data->evt_msg.data,
						data->evt_msg.len);
		} else if (data->type == VFE_MSG_OUTPUT_P) {
			q = &(pmctl->sync.pcam_sync->vid_bufq);

			D("q=0x%x\n", (u32)q);

			/* find the videobuf which is done */
			for (i = 0; i < VIDEO_MAX_FRAME; i++) {
				if (NULL == q->bufs[i])
					continue;
				buf = q->bufs[i];
				buf_phyaddr = videobuf_to_pmem_contig(buf);
				D("buf_phyaddr=0x%x\n", (u32)buf_phyaddr);
				D("data->phy.y_phy=0x%x\n",
							(u32)data->phy.y_phy);
				D("buf = 0x%x\n", (u32)buf);
				if (buf_phyaddr == data->phy.y_phy)
					break;
			}

			/* signal that buffer is done */
			/* get the buf lock first */
			spin_lock_irqsave(q->irqlock, flags);
			buf->state = VIDEOBUF_DONE;
			D("queuedequeue video_buffer 0x%x,"
					"phyaddr = 0x%x\n",
					(u32)buf, (u32)data->phy.y_phy);

			do_gettimeofday(&buf->ts);
			buf->field_count++;
			wake_up(&buf->done);
			spin_unlock_irqrestore(q->irqlock, flags);
		}
		break;
	default:
		break;
	}

	/* now queue the event */
	v4l2_event_queue(pmctl->config_device->config_stat_event_queue.pvdev,
					  &v4l2_evt);
	return 0;
}

/*
 * This function executes in interrupt context.
 */

static void *msm_vfe_sync_alloc(int size,
	  void *syncdata __attribute__((unused)),
	  gfp_t gfp)
{
	struct msm_queue_cmd *qcmd =
		kmalloc(sizeof(struct msm_queue_cmd) + size, gfp);

	if (qcmd) {
		atomic_set(&qcmd->on_heap, 1);
		return qcmd + 1;
	}
	return NULL;
}

static void msm_vfe_sync_free(void *ptr)
{
	if (ptr) {
		struct msm_queue_cmd *qcmd =
			(struct msm_queue_cmd *)ptr;
		qcmd--;
		if (atomic_read(&qcmd->on_heap))
			kfree(qcmd);
	}
}

/*
 * This function executes in interrupt context.
 */

static void msm_vfe_sync(struct msm_vfe_resp *vdata,
		enum msm_queue qtype, void *syncdata,
		gfp_t gfp)
{
	struct msm_queue_cmd *qcmd = NULL;
	struct msm_sync *sync1 = (struct msm_sync *)syncdata;

	if (!sync1) {
		pr_err("%s: no context in dsp callback.\n", __func__);
		return;
	}

	qcmd = ((struct msm_queue_cmd *)vdata) - 1;
	qcmd->type = qtype;
	qcmd->command = vdata;

	if (qtype != MSM_CAM_Q_VFE_MSG)
		goto for_config;

	D("%s: vdata->type %d\n", __func__, vdata->type);
	switch (vdata->type) {
	case VFE_MSG_STATS_AWB:
		D("%s: qtype %d, AWB stats, enqueue event_q.\n",
					__func__, vdata->type);
		break;

	case VFE_MSG_STATS_AEC:
		D("%s: qtype %d, AEC stats, enqueue event_q.\n",
					__func__, vdata->type);
		break;

	case VFE_MSG_STATS_IHIST:
		D("%s: qtype %d, ihist stats, enqueue event_q.\n",
					__func__, vdata->type);
		break;

	case VFE_MSG_STATS_RS:
		D("%s: qtype %d, rs stats, enqueue event_q.\n",
					__func__, vdata->type);
		break;

	case VFE_MSG_STATS_CS:
		D("%s: qtype %d, cs stats, enqueue event_q.\n",
					__func__, vdata->type);
	break;

	case VFE_MSG_GENERAL:
		D("%s: qtype %d, general msg, enqueue event_q.\n",
					__func__, vdata->type);
		break;
	default:
		D("%s: qtype %d not handled\n", __func__, vdata->type);
		/* fall through, send to config. */
	}

for_config:
	D("%s: msm_enqueue event_q\n", __func__);
	msm_isp_enqueue(&sync1->pcam_sync->mctl, vdata, qtype);

	msm_vfe_sync_free(vdata);
}

static struct msm_vfe_callback msm_vfe_s = {
	.vfe_resp = msm_vfe_sync,
	.vfe_alloc = msm_vfe_sync_alloc,
	.vfe_free = msm_vfe_sync_free,
};

/* This function is called by open() function, so we need to init HW*/
/*static int msm_isp_init(struct msm_cam_v4l2_device *pcam)*/
static int msm_isp_open(struct msm_sync *sync)
{
	/* init vfe and senor, register sync callbacks for init*/
	int rc = 0;
	D("%s\n", __func__);
	if (!sync) {
		D("%s: param is NULL", __func__);
		return -EINVAL;
	}
	/* yyan: use old code for vfe31 for now*/
	/*rc = __msm_open(psync, MSM_APPS_ID_V4L2);*/
	msm_camvfe_fn_init(&sync->vfefn, sync);
	if (sync->vfefn.vfe_init) {
		sync->get_pic_abort = 0;
		/*rc = msm_camio_sensor_clk_on(sync->pdev);*/
		if (rc < 0) {
			D("%s: setting sensor clocks failed: %d\n",
							__func__, rc);
			goto msm_isp_open_done;
		}
		rc = sync->vfefn.vfe_init(&msm_vfe_s, sync->pdev);
		if (rc < 0) {
			pr_err("%s: vfe_init failed at %d\n",
						__func__, rc);
			goto msm_isp_open_done;
		}

	} else {
		pr_err("%s: no VFE init func\n", __func__);
		rc = -ENODEV;
		goto msm_isp_open_done;
	}

msm_isp_open_done:
	return rc;
}

static int msm_isp_release(struct msm_sync *psync)
{
	D("%s\n", __func__);
	return 0;
}

static int msm_config_vfe(struct msm_sync *sync, void __user *arg)
{
	struct msm_vfe_cfg_cmd cfgcmd;
	struct msm_pmem_region region[8];
	struct axidata axi_data;

	if (!sync->vfefn.vfe_config) {
		pr_err("%s: no vfe_config!\n", __func__);
		return -EIO;
	}

	if (copy_from_user(&cfgcmd, arg, sizeof(cfgcmd))) {
		ERR_COPY_FROM_USER();
		return -EFAULT;
	}

	memset(&axi_data, 0, sizeof(axi_data));
	CDBG("%s: cmd_type %d\n", __func__, cfgcmd.cmd_type);
	switch (cfgcmd.cmd_type) {
	case CMD_STATS_ENABLE:
		axi_data.bufnum1 =
			msm_pmem_region_lookup(&sync->pmem_stats,
					MSM_PMEM_AEC_AWB, &region[0],
					NUM_STAT_OUTPUT_BUFFERS);
		axi_data.bufnum2 =
			msm_pmem_region_lookup(&sync->pmem_stats,
					MSM_PMEM_AF, &region[axi_data.bufnum1],
					NUM_STAT_OUTPUT_BUFFERS);
		if (!axi_data.bufnum1 || !axi_data.bufnum2) {
			pr_err("%s: pmem region lookup error\n", __func__);
			return -EINVAL;
		}
		axi_data.region = &region[0];
		return sync->vfefn.vfe_config(&cfgcmd, &axi_data);
	case CMD_STATS_AF_ENABLE:
		axi_data.bufnum1 =
			msm_pmem_region_lookup(&sync->pmem_stats,
					MSM_PMEM_AF, &region[0],
					NUM_STAT_OUTPUT_BUFFERS);
		if (!axi_data.bufnum1) {
			pr_err("%s %d: pmem region lookup error\n",
				__func__, __LINE__);
			return -EINVAL;
		}
		axi_data.region = &region[0];
		return sync->vfefn.vfe_config(&cfgcmd, &axi_data);
	case CMD_STATS_AEC_AWB_ENABLE:
		axi_data.bufnum1 =
			msm_pmem_region_lookup(&sync->pmem_stats,
			MSM_PMEM_AEC_AWB, &region[0],
			NUM_STAT_OUTPUT_BUFFERS);
		if (!axi_data.bufnum1) {
			pr_err("%s %d: pmem region lookup error\n",
				__func__, __LINE__);
			return -EINVAL;
		}
		axi_data.region = &region[0];
		return sync->vfefn.vfe_config(&cfgcmd, &axi_data);
	case CMD_STATS_AEC_ENABLE:
		axi_data.bufnum1 =
			msm_pmem_region_lookup(&sync->pmem_stats,
			MSM_PMEM_AEC, &region[0],
			NUM_STAT_OUTPUT_BUFFERS);
		if (!axi_data.bufnum1) {
			pr_err("%s %d: pmem region lookup error\n",
				__func__, __LINE__);
			return -EINVAL;
		}
		axi_data.region = &region[0];
		return sync->vfefn.vfe_config(&cfgcmd, &axi_data);
	case CMD_STATS_AWB_ENABLE:
		axi_data.bufnum1 =
			msm_pmem_region_lookup(&sync->pmem_stats,
			MSM_PMEM_AWB, &region[0],
			NUM_STAT_OUTPUT_BUFFERS);
		if (!axi_data.bufnum1) {
			pr_err("%s %d: pmem region lookup error\n",
				__func__, __LINE__);
			return -EINVAL;
		}
		axi_data.region = &region[0];
		return sync->vfefn.vfe_config(&cfgcmd, &axi_data);


	case CMD_STATS_IHIST_ENABLE:
		axi_data.bufnum1 =
			msm_pmem_region_lookup(&sync->pmem_stats,
			MSM_PMEM_IHIST, &region[0],
			NUM_STAT_OUTPUT_BUFFERS);
		if (!axi_data.bufnum1) {
			pr_err("%s %d: pmem region lookup error\n",
				__func__, __LINE__);
			return -EINVAL;
		}
		axi_data.region = &region[0];
		return sync->vfefn.vfe_config(&cfgcmd, &axi_data);

	case CMD_STATS_RS_ENABLE:
		axi_data.bufnum1 =
			msm_pmem_region_lookup(&sync->pmem_stats,
			MSM_PMEM_RS, &region[0],
			NUM_STAT_OUTPUT_BUFFERS);
		if (!axi_data.bufnum1) {
			pr_err("%s %d: pmem region lookup error\n",
				__func__, __LINE__);
			return -EINVAL;
		}
		axi_data.region = &region[0];
		return sync->vfefn.vfe_config(&cfgcmd, &axi_data);

	case CMD_STATS_CS_ENABLE:
		axi_data.bufnum1 =
			msm_pmem_region_lookup(&sync->pmem_stats,
			MSM_PMEM_CS, &region[0],
			NUM_STAT_OUTPUT_BUFFERS);
		if (!axi_data.bufnum1) {
			pr_err("%s %d: pmem region lookup error\n",
				__func__, __LINE__);
			return -EINVAL;
		}
		axi_data.region = &region[0];
		return sync->vfefn.vfe_config(&cfgcmd, &axi_data);

	case CMD_GENERAL:
	case CMD_STATS_DISABLE:
		return sync->vfefn.vfe_config(&cfgcmd, NULL);
	default:
		pr_err("%s: unknown command type %d\n",
			__func__, cfgcmd.cmd_type);
	}

	return -EINVAL;
}

static int msm_vpe_frame_cfg(struct msm_sync *sync,
				void *cfgcmdin)
{
	int rc = -EIO;
	struct axidata axi_data;
	void *data = &axi_data;
	struct msm_pmem_region region[8];
	int pmem_type;

	struct msm_vpe_cfg_cmd *cfgcmd;
	cfgcmd = (struct msm_vpe_cfg_cmd *)cfgcmdin;

	memset(&axi_data, 0, sizeof(axi_data));
	CDBG("In vpe_frame_cfg cfgcmd->cmd_type = %d\n",
		cfgcmd->cmd_type);
	switch (cfgcmd->cmd_type) {
	case CMD_AXI_CFG_VPE:
		pmem_type = MSM_PMEM_VIDEO_VPE;
		axi_data.bufnum1 =
			msm_pmem_region_lookup_2(&sync->pmem_frames, pmem_type,
								&region[0], 8);
		CDBG("axi_data.bufnum1 = %d\n", axi_data.bufnum1);
		if (!axi_data.bufnum1) {
			pr_err("%s %d: pmem region lookup error\n",
				__func__, __LINE__);
			return -EINVAL;
		}
		pmem_type = MSM_PMEM_VIDEO;
		break;
	default:
		pr_err("%s: unknown command type %d\n",
			__func__, cfgcmd->cmd_type);
		break;
	}
	axi_data.region = &region[0];
	CDBG("out vpe_frame_cfg cfgcmd->cmd_type = %d\n",
		cfgcmd->cmd_type);
	/* send the AXI configuration command to driver */
	if (sync->vpefn.vpe_config)
		rc = sync->vpefn.vpe_config(cfgcmd, data);
	return rc;
}

static int msm_stats_axi_cfg(struct msm_sync *sync,
		struct msm_vfe_cfg_cmd *cfgcmd)
{
	int rc = -EIO;
	struct axidata axi_data;
	void *data = &axi_data;

	struct msm_pmem_region region[3];
	int pmem_type = MSM_PMEM_MAX;

	memset(&axi_data, 0, sizeof(axi_data));

	switch (cfgcmd->cmd_type) {
	case CMD_STATS_AXI_CFG:
		pmem_type = MSM_PMEM_AEC_AWB;
		break;
	case CMD_STATS_AF_AXI_CFG:
		pmem_type = MSM_PMEM_AF;
		break;
	case CMD_GENERAL:
		data = NULL;
		break;
	default:
		pr_err("%s: unknown command type %d\n",
			__func__, cfgcmd->cmd_type);
		return -EINVAL;
	}

	if (cfgcmd->cmd_type != CMD_GENERAL) {
		axi_data.bufnum1 =
			msm_pmem_region_lookup(&sync->pmem_stats, pmem_type,
				&region[0], NUM_STAT_OUTPUT_BUFFERS);
		if (!axi_data.bufnum1) {
			pr_err("%s %d: pmem region lookup error\n",
				__func__, __LINE__);
			return -EINVAL;
		}
	axi_data.region = &region[0];
	}

	/* send the AEC/AWB STATS configuration command to driver */
	if (sync->vfefn.vfe_config)
		rc = sync->vfefn.vfe_config(cfgcmd, &axi_data);

	return rc;
}

static int msm_frame_axi_cfg(struct msm_sync *sync,
		struct msm_vfe_cfg_cmd *cfgcmd)
{
	int rc = -EIO;
	struct axidata axi_data;
	void *data = &axi_data;
	struct msm_pmem_region region[8];
	int pmem_type;
	int i = 0;

	memset(&axi_data, 0, sizeof(axi_data));

	switch (cfgcmd->cmd_type) {

	case CMD_AXI_CFG_PREVIEW:
		axi_data.bufnum2 =
			msm_pmem_region_lookup_3(sync->pcam_sync,
				&region[0], 0, 4);
		if (!axi_data.bufnum2) {
			pr_err("%s %d: pmem region 3 lookup error\n",
				__func__, __LINE__);
			return -EINVAL;
		}
		D("%s __func__ axi_data.bufnum2 = %d\n", __func__,
						axi_data.bufnum2);
		break;

	case CMD_AXI_CFG_VIDEO:
		pmem_type = MSM_PMEM_PREVIEW;
		axi_data.bufnum1 =
			msm_pmem_region_lookup_3(sync->pcam_sync,
				&region[0], 0, 3);
		D("%s bufnum1 = %d\n", __func__, axi_data.bufnum1);
		if (!axi_data.bufnum1) {
			pr_err("%s %d: pmem region lookup error\n",
				__func__, __LINE__);
			return -EINVAL;
		}

		pmem_type = MSM_PMEM_VIDEO;
		axi_data.bufnum2 =
			msm_pmem_region_lookup_3(sync->pcam_sync,
				&region[axi_data.bufnum1],
				4, 7);
		D("%s bufnum2 = %d\n", __func__, axi_data.bufnum2);
		if (!axi_data.bufnum2) {
			pr_err("%s %d: pmem region lookup error\n",
				__func__, __LINE__);
			return -EINVAL;
		}
		break;


	case CMD_AXI_CFG_SNAP:
		pmem_type = MSM_PMEM_THUMBNAIL;
		axi_data.bufnum1 =
			msm_pmem_region_lookup_3(sync->pcam_sync,
				&region[0], 0, 4);
		if (!axi_data.bufnum1) {
			pr_err("%s %d: pmem region lookup error\n",
				__func__, __LINE__);
			return -EINVAL;
		}

		pmem_type = MSM_PMEM_MAINIMG;
		axi_data.bufnum2 =
		msm_pmem_region_lookup_3(sync->pcam_sync,
				&region[axi_data.bufnum1],
				axi_data.bufnum1, 8);
		if (!axi_data.bufnum2) {
			pr_err("%s %d: pmem region lookup error\n",
				__func__, __LINE__);
			return -EINVAL;
		}
		break;

	case CMD_RAW_PICT_AXI_CFG:
		pmem_type = MSM_PMEM_RAW_MAINIMG;
		axi_data.bufnum2 =
			msm_pmem_region_lookup_3(sync->pcam_sync,
				&region[0], 0, 4);
		if (!axi_data.bufnum2) {
			pr_err("%s %d: pmem region lookup error\n",
				__func__, __LINE__);
			return -EINVAL;
		}
		break;

	case CMD_GENERAL:
		data = NULL;
		break;

	default:
		pr_err("%s: unknown command type %d\n",
			__func__, cfgcmd->cmd_type);
		return -EINVAL;
	}

	axi_data.region = &region[0];
	D("%s bufnum1 = %d, bufnum2 = %d\n", __func__,
	  axi_data.bufnum1, axi_data.bufnum2);
	for (i = 0; i < 8; i++) {
		D("%s region %d paddr = 0x%p\n", __func__, i,
					(void *)region[i].paddr);
		D("%s region y_off = %d cbcr_off = %d\n", __func__,
			region[i].info.y_off, region[i].info.cbcr_off);
	}
	/* send the AXI configuration command to driver */
	if (sync->vfefn.vfe_config)
		rc = sync->vfefn.vfe_config(cfgcmd, data);

	return rc;
}

static int msm_axi_config(struct msm_sync *sync, void __user *arg)
{
	struct msm_vfe_cfg_cmd cfgcmd;

	if (copy_from_user(&cfgcmd, arg, sizeof(cfgcmd))) {
		ERR_COPY_FROM_USER();
		return -EFAULT;
	}

	switch (cfgcmd.cmd_type) {
	case CMD_AXI_CFG_VIDEO:
	case CMD_AXI_CFG_PREVIEW:
	case CMD_AXI_CFG_SNAP:
	case CMD_RAW_PICT_AXI_CFG:
		return msm_frame_axi_cfg(sync, &cfgcmd);
	case CMD_AXI_CFG_VPE:
		return 0;
		return msm_vpe_frame_cfg(sync, (void *)&cfgcmd);

	case CMD_STATS_AXI_CFG:
	case CMD_STATS_AF_AXI_CFG:
		return msm_stats_axi_cfg(sync, &cfgcmd);

	default:
		pr_err("%s: unknown command type %d\n",
			__func__,
			cfgcmd.cmd_type);
		return -EINVAL;
	}

	return 0;
}

static int msm_set_crop(struct msm_sync *sync, void __user *arg)
{
	struct crop_info crop;

	if (copy_from_user(&crop,
				arg,
				sizeof(struct crop_info))) {
		ERR_COPY_FROM_USER();
		return -EFAULT;
	}

	if (!sync->croplen) {
		sync->cropinfo = kmalloc(crop.len, GFP_KERNEL);
		if (!sync->cropinfo)
			return -ENOMEM;
	} else if (sync->croplen < crop.len)
		return -EINVAL;

	if (copy_from_user(sync->cropinfo,
				crop.info,
				crop.len)) {
		ERR_COPY_FROM_USER();
		kfree(sync->cropinfo);
		return -EFAULT;
	}

	sync->croplen = crop.len;

	return 0;
}

static int msm_put_stats_buffer(struct msm_sync *sync, void __user *arg)
{
	int rc = -EIO;

	struct msm_stats_buf buf;
	unsigned long pphy;
	struct msm_vfe_cfg_cmd cfgcmd;

	if (copy_from_user(&buf, arg,
				sizeof(struct msm_stats_buf))) {
		ERR_COPY_FROM_USER();
		return -EFAULT;
	}

	CDBG("%s\n", __func__);
	pphy = msm_pmem_stats_vtop_lookup(sync, buf.buffer, buf.fd);

	if (pphy != 0) {
		if (buf.type == STAT_AEAW)
			cfgcmd.cmd_type = CMD_STATS_BUF_RELEASE;
		else if (buf.type == STAT_AF)
			cfgcmd.cmd_type = CMD_STATS_AF_BUF_RELEASE;
		else if (buf.type == STAT_AEC)
			cfgcmd.cmd_type = CMD_STATS_AEC_BUF_RELEASE;
		else if (buf.type == STAT_AWB)
			cfgcmd.cmd_type = CMD_STATS_AWB_BUF_RELEASE;
		else if (buf.type == STAT_IHIST)
			cfgcmd.cmd_type = CMD_STATS_IHIST_BUF_RELEASE;
		else if (buf.type == STAT_RS)
			cfgcmd.cmd_type = CMD_STATS_RS_BUF_RELEASE;
		else if (buf.type == STAT_CS)
			cfgcmd.cmd_type = CMD_STATS_CS_BUF_RELEASE;

		else {
			pr_err("%s: invalid buf type %d\n",
				__func__,
				buf.type);
			rc = -EINVAL;
			goto put_done;
		}

		cfgcmd.value = (void *)&buf;

		if (sync->vfefn.vfe_config) {
			rc = sync->vfefn.vfe_config(&cfgcmd, &pphy);
			if (rc < 0)
				pr_err("%s: vfe_config error %d\n",
					__func__, rc);
		} else
			pr_err("%s: vfe_config is NULL\n", __func__);
	} else {
		pr_err("%s: NULL physical address\n", __func__);
		rc = -EINVAL;
	}

put_done:
	return rc;
}

/* config function simliar to origanl msm_ioctl_config*/
static int msm_isp_config(struct msm_cam_media_controller *pmctl,
			 unsigned int cmd, unsigned long arg)
{
	int rc = -EINVAL;
	void __user *argp = (void __user *)arg;

	D("%s: cmd %d\n", __func__, _IOC_NR(cmd));
	switch (cmd) {
/*yyan todo: move to mctl or sensor code
	case MSM_CAM_IOCTL_GET_SENSOR_INFO:
		rc = msm_get_sensor_info(pmsm->sync, argp);
		D("%s rc = %d\n", __func__, rc);
		break;

	case MSM_CAM_IOCTL_SENSOR_IO_CFG:
		rc = pmsm->sync->sctrl.s_config(argp);
		break;
*/
	case MSM_CAM_IOCTL_PICT_PP_DONE:
		/* Release the preview of snapshot frame
		 * that was grabbed.
		 */
		/*rc = msm_pp_release(pmsm->sync, arg);*/
		break;

	case MSM_CAM_IOCTL_CONFIG_VFE:
		/* Coming from config thread for update */
		rc = msm_config_vfe(&pmctl->sync, argp);
		break;

	case MSM_CAM_IOCTL_CONFIG_VPE:
		/* Coming from config thread for update */
		/*rc = msm_config_vpe(pmsm->sync, argp);*/
		rc = 0;
		break;

	case MSM_CAM_IOCTL_AXI_CONFIG:
	case MSM_CAM_IOCTL_AXI_VPE_CONFIG:
		D("Received MSM_CAM_IOCTL_AXI_CONFIG\n");
		rc = msm_axi_config(&pmctl->sync, argp);
		break;

	case MSM_CAM_IOCTL_SET_CROP:
		rc = msm_set_crop(&pmctl->sync, argp);
		break;

	case MSM_CAM_IOCTL_RELEASE_STATS_BUFFER:
		rc = msm_put_stats_buffer(&pmctl->sync, argp);
		break;

	default:
		break;
	}

	D("%s: cmd %d DONE\n", __func__, _IOC_NR(cmd));

	return rc;
}

static struct msm_isp_ops isp_subdev[MSM_MAX_CAMERA_CONFIGS];

/**/
int msm_isp_init_module(int g_num_config_nodes)
{
	int i = 0;

	for (i = 0; i < g_num_config_nodes; i++) {
		isp_subdev[i].isp_open = msm_isp_open;
		isp_subdev[i].isp_config = msm_isp_config;
		isp_subdev[i].isp_release  = msm_isp_release;
		isp_subdev[i].isp_enqueue = msm_isp_enqueue;
	}
	return 0;
}
EXPORT_SYMBOL(msm_isp_init_module);

/*
int msm_isp_register(struct msm_cam_v4l2_device *pcam)
*/

int msm_isp_register(struct msm_cam_server_dev *psvr)
{
	int rc = -EINVAL;
	int i = 0;

	D("%s\n", __func__);

	if (!psvr) {
		D("%s psvr = %x\n", __func__, (unsigned int)psvr);
		return rc;
	}

	for (i = 0; i < psvr->config_info.num_config_nodes; i++)
		psvr->isp_subdev[i] = &(isp_subdev[i]);

	return 0;
}

EXPORT_SYMBOL(msm_isp_register);

