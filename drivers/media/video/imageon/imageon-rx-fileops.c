/*
 * Driver for the IMAGEON-FMC board
 *
 * Copyright 2012-2013 Analog Devices Inc.
 *  Author: Lars-Peter Clausen <lars@metafoo.de>
 *
 * based on cobalt-fileops.c
 *
 * Licensed under the GPL-2.
 */

#include <media/v4l2-event.h>
#include <linux/v4l2-dv-timings.h>
#include <media/videobuf2-dma-contig.h>
#include <linux/dmaengine.h>
#include <linux/spinlock.h>

#include "imageon-rx-driver.h"
#include "imageon-rx-ioctl.h"

struct imageon_rx_buffer {
	struct vb2_buffer vb;
	struct list_head head;
};

static struct imageon_rx_buffer *vb2_buf_to_imageon_buf(struct vb2_buffer *vb)
{
	return container_of(vb, struct imageon_rx_buffer, vb);
}

static const struct v4l2_file_operations imageon_rx_fops = {
	.owner = THIS_MODULE,
	.open = v4l2_fh_open,
	.release = vb2_fop_release,
	.unlocked_ioctl = video_ioctl2,
	.read = vb2_fop_read,
	.poll = vb2_fop_poll,
	.mmap = vb2_fop_mmap,
};

static int imageon_rx_queue_setup(struct vb2_queue *q, const struct v4l2_format *fmt,
			unsigned int *num_buffers, unsigned int *num_planes,
			unsigned int sizes[], void *alloc_ctxs[])
{
	struct imageon_rx *imageon_rx = vb2_get_drv_priv(q);
	struct imageon_rx_stream *s = &imageon_rx->stream;

	if (*num_buffers < 3)
		*num_buffers = 3;
	if (*num_buffers > 10)
		*num_buffers = 10;
	*num_planes = 1;

	sizes[0] = s->stride * s->height;

	alloc_ctxs[0] = imageon_rx->alloc_ctx;

	return 0;
}

static int imageon_rx_buf_prepare(struct vb2_buffer *vb)
{
	struct imageon_rx *imageon_rx = vb2_get_drv_priv(vb->vb2_queue);
	struct imageon_rx_stream *s = &imageon_rx->stream;
	unsigned size;

	size = s->stride * s->height;
	if (vb2_plane_size(vb, 0) < size) {
		pr_info("data will not fit into plane (%lu < %u)\n",
					vb2_plane_size(vb, 0), size);
		return -EINVAL;
	}

	vb2_set_plane_payload(vb, 0, size);
	return 0;
}

static void imageon_rx_dma_done(void *arg)
{
	struct imageon_rx_buffer *buf = arg;
	struct vb2_queue *q = buf->vb.vb2_queue;
	struct imageon_rx *imageon_rx = vb2_get_drv_priv(q);
	struct imageon_rx_stream *s = &imageon_rx->stream;
	unsigned long flags;

	spin_lock_irqsave(&s->spinlock, flags);
	list_del(&buf->head);
	spin_unlock_irqrestore(&s->spinlock, flags);

	if (!vb2_is_streaming(q))
		return;

	v4l2_get_timestamp(&buf->vb.v4l2_buf.timestamp);
	vb2_buffer_done(&buf->vb, VB2_BUF_STATE_DONE);
}

static void imageon_rx_buf_queue(struct vb2_buffer *vb)
{
	struct imageon_rx_buffer *buf = vb2_buf_to_imageon_buf(vb);
	struct dma_async_tx_descriptor *desc;
	struct vb2_queue *q = vb->vb2_queue;
	struct imageon_rx *imageon_rx = vb2_get_drv_priv(q);
	struct imageon_rx_stream *s = &imageon_rx->stream;
	struct dma_interleaved_template *xt;
	unsigned long size;
	unsigned long flags;
	dma_addr_t addr;
	dma_cookie_t cookie;

	addr = vb2_dma_contig_plane_dma_addr(vb, 0);
	size = vb2_get_plane_payload(vb, 0);

	xt = kzalloc(sizeof(struct dma_async_tx_descriptor) +
				sizeof(struct data_chunk), GFP_KERNEL);

	xt->dst_start = addr;
	xt->src_inc = false;
	xt->dst_inc = false;
	xt->src_sgl = false;
	xt->dst_sgl = true;
	xt->frame_size = 1;
	xt->numf = s->height;
	xt->sgl[0].size = s->width * s->bpp;
	xt->sgl[1].icg = s->stride - (s->width * s->bpp);
	xt->dir = DMA_DEV_TO_MEM;

	desc = s->chan->device->device_prep_interleaved_dma(s->chan, xt, 0);
	if (!desc) {
		vb2_buffer_done(vb, VB2_BUF_STATE_ERROR);
		return;
	}
	desc->callback = imageon_rx_dma_done;
	desc->callback_param = buf;

	spin_lock_irqsave(&s->spinlock, flags);
	list_add_tail(&buf->head, &s->queued_buffers);
	spin_unlock_irqrestore(&s->spinlock, flags);

	cookie = dmaengine_submit(desc);
	if (cookie < 0) {
		vb2_buffer_done(vb, VB2_BUF_STATE_ERROR);
		return;
	}

	if (vb2_is_streaming(q))
		dma_async_issue_pending(s->chan);

	kfree(xt);
}

static void imageon_rx_wait_prepare(struct vb2_queue *q)
{
	struct imageon_rx *imageon_rx = vb2_get_drv_priv(q);
	struct imageon_rx_stream *s = &imageon_rx->stream;

	mutex_unlock(&s->lock);
}

static void imageon_rx_wait_finish(struct vb2_queue *q)
{
	struct imageon_rx *imageon_rx = vb2_get_drv_priv(q);
	struct imageon_rx_stream *s = &imageon_rx->stream;

	mutex_lock(&s->lock);
}

static int imageon_rx_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct imageon_rx *imageon_rx = vb2_get_drv_priv(q);
	struct imageon_rx_stream *s = &imageon_rx->stream;

	dma_async_issue_pending(s->chan);
	return 0;
}

static int imageon_rx_stop_streaming(struct vb2_queue *q)
{
	struct imageon_rx *imageon_rx = vb2_get_drv_priv(q);
	struct imageon_rx_stream *s = &imageon_rx->stream;
	struct imageon_rx_buffer *buf;
	unsigned long flags;

	dmaengine_terminate_all(s->chan);

	spin_lock_irqsave(&s->spinlock, flags);

	list_for_each_entry(buf, &s->queued_buffers, head)
		vb2_buffer_done(&buf->vb, VB2_BUF_STATE_ERROR);

	INIT_LIST_HEAD(&s->queued_buffers);

	spin_unlock_irqrestore(&s->spinlock, flags);

	vb2_wait_for_all_buffers(q);
	return 0;
}

static const struct vb2_ops imageon_rx_qops = {
	.queue_setup = imageon_rx_queue_setup,
	.wait_prepare = imageon_rx_wait_prepare,
	.wait_finish = imageon_rx_wait_finish,

	.buf_prepare = imageon_rx_buf_prepare,
	.buf_queue = imageon_rx_buf_queue,
	.start_streaming = imageon_rx_start_streaming,
	.stop_streaming = imageon_rx_stop_streaming,
};

int imageon_rx_nodes_register(struct imageon_rx *imageon_rx)
{
	struct v4l2_dv_timings dv1080p60 = V4L2_DV_BT_DMT_800X600P60;
	struct imageon_rx_stream *s = &imageon_rx->stream;
	struct video_device *vdev = &s->vdev;
	struct vb2_queue *q = &s->q;
	int ret;

	mutex_init(&s->lock);
	snprintf(vdev->name, sizeof(vdev->name),
		 "%s", imageon_rx->v4l2_dev.name);
	vdev->v4l2_dev = &imageon_rx->v4l2_dev;
	vdev->fops = &imageon_rx_fops;
	vdev->release = video_device_release_empty;
	vdev->ctrl_handler = s->sd_adv7611->ctrl_handler;
	set_bit(V4L2_FL_USE_FH_PRIO, &vdev->flags);
	vdev->lock = &s->lock;
	vdev->queue = q;

	INIT_LIST_HEAD(&s->queued_buffers);
	spin_lock_init(&s->spinlock);

	s->width = 800;
	s->height = 600;
	s->bpp = IMAGEON_RX_BYTES_PER_PIXEL_RGB32;
	s->pack_fmt = IMAGEON_RX_VID_PACK_FMT_RGB32;
	s->stride = s->width * s->bpp;

	v4l2_subdev_call(s->sd_adv7611, video, s_dv_timings, &dv1080p60);
	imageon_rx_set_funcs(vdev);

	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->io_modes = VB2_MMAP | VB2_USERPTR | VB2_READ;
	q->drv_priv = imageon_rx;
	q->buf_struct_size = sizeof(struct imageon_rx_buffer);
	q->ops = &imageon_rx_qops;
	q->mem_ops = &vb2_dma_contig_memops;

	ret = vb2_queue_init(q);
	if (ret)
		return ret;

	s->pad.flags = MEDIA_PAD_FL_SINK;
	ret = media_entity_init(&vdev->entity, 1, &s->pad, 0);
	if (ret < 0)
		goto err_cleanup;

	ret = media_entity_create_link(&s->sd_adv7611->entity, 0,
			&vdev->entity, 0,
			MEDIA_LNK_FL_ENABLED | MEDIA_LNK_FL_IMMUTABLE);
	if (ret < 0)
		goto err_cleanup;
	
	ret = video_register_device(vdev, VFL_TYPE_GRABBER, -1);
	if (ret < 0)
		goto err_cleanup;

	return 0;

err_cleanup:
	media_entity_cleanup(&vdev->entity);
	return ret;
}

void imageon_rx_nodes_unregister(struct imageon_rx *imageon_rx)
{
	struct video_device *vdev = &imageon_rx->stream.vdev;
	video_unregister_device(vdev);
}
