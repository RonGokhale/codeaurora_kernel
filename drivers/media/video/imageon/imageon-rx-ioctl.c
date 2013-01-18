/*
 * Driver for the IMAGEON-FMC board
 *
 * Copyright 2012-2013 Analog Devices Inc.
 *  Author: Lars-Peter Clausen <lars@metafoo.de>
 *
 * based on cobalt-ioctl.c
 *
 * Licensed under the GPL-2.
 */

#include "imageon-rx-driver.h"
#include "imageon-rx-ioctl.h"

#include <media/v4l2-chip-ident.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ctrls.h>
#include <media/adv7604.h>
#include <linux/v4l2-dv-timings.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>

#define IMAGEON_RX_REG_CONFIG 0x04
#define IMAGEON_RX_REG_TIMING 0x08

#define IMAGEON_RX_CONFIG_TPG		BIT(3)
#define IMAGEON_RX_CONFIG_CSC_BYPASS	BIT(2)
#define IMAGEON_RX_CONFIG_ENABLE	BIT(0)

static const struct v4l2_dv_timings cea1080p60 = V4L2_DV_BT_DMT_800X600P60;

static void imageon_rx_update_cfg(struct imageon_rx *imageon_rx,
	uint32_t set, uint32_t clr)
{
	uint32_t val;

	val = ioread32(imageon_rx->base + IMAGEON_RX_REG_CONFIG);
	val &= ~clr;
	val |= set;
	iowrite32(val, imageon_rx->base + IMAGEON_RX_REG_CONFIG);
}

#ifdef CONFIG_VIDEO_ADV_DEBUG

static int imageon_rx_g_register(struct file *file, void *priv_fh,
	struct v4l2_dbg_register *reg)
{
	struct imageon_rx_stream *s = imageon_rx_file_to_stream(file);
	struct imageon_rx *imageon_rx = imageon_rx_stream_to_imageon_rx(s);

	if (v4l2_chip_match_host(&reg->match)) {
		if (reg->reg > IMAGEON_RX_REG_TIMING + 0x20)
		    return -EINVAL;
		reg->val = ioread32(imageon_rx->base + reg->reg);
		reg->size = 4;
		return 0;
	} else {
		v4l2_device_call_all(&imageon_rx->v4l2_dev, 0, core, g_register,
		    reg);
		return v4l2_subdev_call(s->sd_adv7611, core, g_register, reg);
	}
}

static int imageon_rx_s_register(struct file *file, void *priv_fh,
	struct v4l2_dbg_register *reg)
{
	struct imageon_rx_stream *s = imageon_rx_file_to_stream(file);
	struct imageon_rx *imageon_rx = imageon_rx_stream_to_imageon_rx(s);

	if (!capable(CAP_SYS_ADMIN))
		return -EPERM;

	if (v4l2_chip_match_host(&reg->match)) {
		if (reg->reg > IMAGEON_RX_REG_TIMING + 0x30)
		    return -EINVAL;
		iowrite32(reg->val, imageon_rx->base + reg->reg);
		reg->size = 4;
		return 0;
	}
	
	return v4l2_subdev_call(s->sd_adv7611, core, s_register, reg);
}
#endif

static int imageon_rx_log_status(struct file *file, void *priv)
{
	struct imageon_rx_stream *s = imageon_rx_file_to_stream(file);
	struct imageon_rx *imageon_rx = imageon_rx_stream_to_imageon_rx(s);

	v4l2_device_call_all(&imageon_rx->v4l2_dev, 0, core, log_status);
	return 0;
}


static int imageon_rx_g_chip_ident(struct file *file, void *priv_fh,
	struct v4l2_dbg_chip_ident *chip)
{
	struct imageon_rx_stream *s = imageon_rx_file_to_stream(file);

	chip->ident = V4L2_IDENT_NONE;
	chip->revision = 0;
	if (chip->match.type == V4L2_CHIP_MATCH_HOST) {
		if (v4l2_chip_match_host(&chip->match))
			chip->ident = 0;
		return 0;
	}
	if (chip->match.type != V4L2_CHIP_MATCH_I2C_DRIVER &&
		chip->match.type != V4L2_CHIP_MATCH_I2C_ADDR)
		return -EINVAL;

	return v4l2_subdev_call(s->sd_adv7611, core, g_chip_ident, chip);
}

static int imageon_rx_querycap(struct file *file, void *priv_fh,
				struct v4l2_capability *vcap)
{
	struct imageon_rx *imageon_rx = video_drvdata(file);

	strlcpy(vcap->driver, "imageon_rx", sizeof(vcap->driver));
	strlcpy(vcap->card, "imageon_rx", sizeof(vcap->card));
	snprintf(vcap->bus_info, sizeof(vcap->bus_info), "platform:%s",
		dev_name(imageon_rx->mdev.dev));
	vcap->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
	vcap->capabilities = vcap->device_caps | V4L2_CAP_DEVICE_CAPS;

	return 0;
}

static void imageon_rx_select_input(struct imageon_rx_stream *s)
{
	struct imageon_rx *imageon_rx = imageon_rx_stream_to_imageon_rx(s);

	switch (s->pack_fmt) {
/*
	case IMAGEON_RX_VID_PACK_FMT_YUYV:
		imageon_rx_update_cfg(imageon_rx, IMAGEON_RX_CONFIG_CSC_BYPASS, 0);
		break;
*/
	case IMAGEON_RX_VID_PACK_FMT_RGB32:
		imageon_rx_update_cfg(imageon_rx, 0, IMAGEON_RX_CONFIG_CSC_BYPASS);
		break;
	default:
		break;
	}
}

static int imageon_rx_streamon(struct file *file, void *priv_fh,
	enum v4l2_buf_type buffer_type)
{
	struct imageon_rx *imageon_rx = video_drvdata(file);
	struct imageon_rx_stream *s = imageon_rx_file_to_stream(file);

	if (buffer_type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	imageon_rx_select_input(s);
	imageon_rx_update_cfg(imageon_rx, IMAGEON_RX_CONFIG_ENABLE, 0);

	return vb2_streamon(&s->q, buffer_type);
}

static int imageon_rx_streamoff(struct file *file, void *priv_fh,
	enum v4l2_buf_type buffer_type)
{
	struct imageon_rx *imageon_rx = video_drvdata(file);
	struct imageon_rx_stream *s = imageon_rx_file_to_stream(file);

	if (buffer_type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	imageon_rx_update_cfg(imageon_rx, 0, IMAGEON_RX_CONFIG_ENABLE);

	return vb2_streamoff(&s->q, buffer_type);
}

static int imageon_rx_s_dv_timings(struct file *file, void *priv_fh,
	struct v4l2_dv_timings *timings)
{
	struct imageon_rx *imageon_rx = video_drvdata(file);
	struct imageon_rx_stream *s = &imageon_rx->stream;
	int err;

	if (s->input == 1) {
		*timings = cea1080p60;
		return 0;
	}

	err = v4l2_subdev_call(s->sd_adv7611,
			video, s_dv_timings, timings);
	if (!err) {
		s->width = timings->bt.width;
		s->height = timings->bt.height;
		s->stride = timings->bt.width * s->bpp;
	}
	return err;
}

static int imageon_rx_g_dv_timings(struct file *file, void *priv_fh,
	struct v4l2_dv_timings *timings)
{
	struct imageon_rx *imageon_rx = video_drvdata(file);
	struct imageon_rx_stream *s = &imageon_rx->stream;

	if (s->input == 1) {
		*timings = cea1080p60;
		return 0;
	}

	return v4l2_subdev_call(s->sd_adv7611,
			video, g_dv_timings, timings);
}

static int imageon_rx_enum_dv_timings(struct file *file, void *priv_fh,
				    struct v4l2_enum_dv_timings *timings)
{
	struct imageon_rx *imageon_rx = video_drvdata(file);
	struct imageon_rx_stream *s = &imageon_rx->stream;

	if (s->input == 1) {
		if (timings->index)
			return -EINVAL;
		memset(timings->reserved, 0, sizeof(timings->reserved));
		timings->timings = cea1080p60;
		return 0;
	}

	return v4l2_subdev_call(s->sd_adv7611,
			video, enum_dv_timings, timings);
}

static int imageon_rx_query_dv_timings(struct file *file, void *priv_fh,
				    struct v4l2_dv_timings *timings)
{
	struct imageon_rx *imageon_rx = video_drvdata(file);
	struct imageon_rx_stream *s = &imageon_rx->stream;

	if (s->input == 1) {
		*timings = cea1080p60;
		return 0;
	}

	return v4l2_subdev_call(s->sd_adv7611,
			video, query_dv_timings, timings);
}

static int imageon_rx_dv_timings_cap(struct file *file, void *priv_fh,
				    struct v4l2_dv_timings_cap *cap)
{
	struct imageon_rx *imageon_rx = video_drvdata(file);
	struct imageon_rx_stream *s = &imageon_rx->stream;

	return v4l2_subdev_call(s->sd_adv7611,
			video, dv_timings_cap, cap);
}

static int imageon_rx_enum_fmt_vid_cap(struct file *file, void *priv_fh,
	struct v4l2_fmtdesc *f)
{
	switch (f->index) {
	case 0:
		strlcpy(f->description, "RGB32", sizeof(f->description));
		f->pixelformat = V4L2_PIX_FMT_RGB32;
		break;
/*
	case 0:
		strlcpy(f->description, "YUV 4:2:2", sizeof(f->description));
		f->pixelformat = V4L2_PIX_FMT_YUYV;
		break;*/
	default:
		return -EINVAL;
	}

	return 0;
}

static int imageon_rx_g_fmt_vid_cap(struct file *file, void *priv_fh,
		struct v4l2_format *f)
{
	struct imageon_rx *imageon_rx = video_drvdata(file);
	struct imageon_rx_stream *s = &imageon_rx->stream;
	struct v4l2_pix_format *pix = &f->fmt.pix;

	pix->width = s->width;
	pix->height = s->height;
	pix->bytesperline = s->stride;
	pix->field = V4L2_FIELD_NONE;

	switch (s->pack_fmt) {
/*
	case IMAGEON_RX_VID_PACK_FMT_YUYV:
		pix->colorspace = V4L2_COLORSPACE_REC709;
		pix->pixelformat = V4L2_PIX_FMT_YUYV;
		break;
*/
	case IMAGEON_RX_VID_PACK_FMT_RGB32:
		pix->colorspace = V4L2_COLORSPACE_SRGB;
		pix->pixelformat = V4L2_PIX_FMT_RGB32;
		break;
	default:
		return -EINVAL;
	}

	pix->sizeimage = pix->bytesperline * pix->height;

	return 0;
}

static int imageon_rx_try_fmt_vid_cap(struct file *file, void *priv_fh,
		struct v4l2_format *f)
{
	struct imageon_rx *imageon_rx = video_drvdata(file);
	struct imageon_rx_stream *s = &imageon_rx->stream;
	struct v4l2_pix_format *pix = &f->fmt.pix;
	struct v4l2_mbus_framefmt mbus_fmt;

	v4l_bound_align_image(&pix->width, 176, 1920, 0,
				 &pix->height, 144, 1080, 0, 0);

	if (s->input == 0) {
		v4l2_subdev_call(s->sd_adv7611, video, g_mbus_fmt, &mbus_fmt);
		v4l2_fill_pix_format(pix, &mbus_fmt);
	}

	switch (pix->pixelformat) {
	default:
		pix->pixelformat = V4L2_PIX_FMT_RGB32;
		pix->colorspace = V4L2_COLORSPACE_SRGB;
		pix->bytesperline = ALIGN(pix->width * IMAGEON_RX_BYTES_PER_PIXEL_RGB32, 4);
		break;
/*
	default:
		pix->pixelformat = V4L2_PIX_FMT_YUYV;
		pix->colorspace = V4L2_COLORSPACE_REC709;
		pix->bytesperline = ALIGN(pix->width *
		IMAGEON_RX_BYTES_PER_PIXEL_YUYV, 4);
		break;
*/
	}

	pix->sizeimage = pix->bytesperline * pix->height;
	pix->field = V4L2_FIELD_NONE;
	pix->priv = 0;

	return 0;
}

static int imageon_rx_s_fmt_vid_cap(struct file *file, void *priv_fh,
		struct v4l2_format *f)
{
	struct imageon_rx *imageon_rx = video_drvdata(file);
	struct imageon_rx_stream *s = &imageon_rx->stream;
	struct v4l2_pix_format *pix = &f->fmt.pix;

	if (imageon_rx_try_fmt_vid_cap(file, priv_fh, f))
		return -EINVAL;

	s->width = pix->width;
	s->height = pix->height;
	s->stride = pix->bytesperline;
	switch (pix->pixelformat) {
/*
	case V4L2_PIX_FMT_YUYV:
		s->bpp = IMAGEON_RX_BYTES_PER_PIXEL_YUYV;
		s->pack_fmt = IMAGEON_RX_VID_PACK_FMT_YUYV;
		break;
*/
	case V4L2_PIX_FMT_RGB32:
		s->bpp = IMAGEON_RX_BYTES_PER_PIXEL_RGB32;
		s->pack_fmt = IMAGEON_RX_VID_PACK_FMT_RGB32;
		break;
	default:
		return -EINVAL;
	}
	imageon_rx_select_input(s);
	s->dma_config.hsize = s->stride;
	s->dma_config.vsize = s->height;
	s->dma_config.stride = s->stride;

	dmaengine_device_control(s->chan, DMA_SLAVE_CONFIG,
        (unsigned long)&s->dma_config);

	iowrite32((s->height << 16) | s->width, imageon_rx->base + IMAGEON_RX_REG_TIMING);

	return 0;
}

static int imageon_rx_enum_input(struct file *file, void *priv_fh,
				 struct v4l2_input *inp)
{
	struct imageon_rx *imageon_rx = video_drvdata(file);
	struct imageon_rx_stream *s = &imageon_rx->stream;

	switch (inp->index) {
	case 0:
		snprintf(inp->name, sizeof(inp->name), "HDMI-0");
		break;
	case 1:
		snprintf(inp->name, sizeof(inp->name), "Generator-0");
		break;
	default:
		return -EINVAL;
	}

	inp->type = V4L2_INPUT_TYPE_CAMERA;
	if (inp->index == 1)
		return 0;
	return v4l2_subdev_call(s->sd_adv7611,
			video, g_input_status, &inp->status);
}

static int imageon_rx_g_input(struct file *file, void *priv_fh, unsigned int *i)
{
	struct imageon_rx *imageon_rx = video_drvdata(file);
	struct imageon_rx_stream *s = &imageon_rx->stream;

	*i = s->input;
	return 0;
}

static int imageon_rx_s_input(struct file *file, void *priv_fh, unsigned int i)
{
	struct imageon_rx_stream *s = imageon_rx_file_to_stream(file);

	if (i >= 2)
		return -EINVAL;

	s->input = i;
	imageon_rx_select_input(s);

	if (s->input == 1) /* Test Pattern Generator */
		return 0;

	return v4l2_subdev_call(s->sd_adv7611, video, s_routing,
		ADV7604_MODE_HDMI, 0, 0);
}

static int imageon_rx_g_parm(struct file *file, void *fh,
	struct v4l2_streamparm *a)
{
	if (a->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	a->parm.capture.timeperframe.numerator = 1;
	a->parm.capture.timeperframe.denominator = 60;
	return 0;
}

static const struct v4l2_ioctl_ops imageon_rx_ioctl_ops = {
	.vidioc_querycap		= imageon_rx_querycap,
	.vidioc_g_parm			= imageon_rx_g_parm,
	.vidioc_log_status		= imageon_rx_log_status,
	.vidioc_streamon		= imageon_rx_streamon,
	.vidioc_streamoff		= imageon_rx_streamoff,
	.vidioc_g_chip_ident		= imageon_rx_g_chip_ident,
	.vidioc_enum_input		= imageon_rx_enum_input,
	.vidioc_g_input			= imageon_rx_g_input,
	.vidioc_s_input			= imageon_rx_s_input,
	.vidioc_enum_fmt_vid_cap	= imageon_rx_enum_fmt_vid_cap,
	.vidioc_g_fmt_vid_cap		= imageon_rx_g_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap		= imageon_rx_s_fmt_vid_cap,
	.vidioc_try_fmt_vid_cap		= imageon_rx_try_fmt_vid_cap,
	.vidioc_s_dv_timings		= imageon_rx_s_dv_timings,
	.vidioc_g_dv_timings		= imageon_rx_g_dv_timings,
	.vidioc_query_dv_timings	= imageon_rx_query_dv_timings,
	.vidioc_enum_dv_timings		= imageon_rx_enum_dv_timings,
	.vidioc_dv_timings_cap		= imageon_rx_dv_timings_cap,
	.vidioc_subscribe_event		= v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event	= v4l2_event_unsubscribe,
	.vidioc_reqbufs			= vb2_ioctl_reqbufs,
	.vidioc_querybuf		= vb2_ioctl_querybuf,
	.vidioc_qbuf			= vb2_ioctl_qbuf,
	.vidioc_dqbuf			= vb2_ioctl_dqbuf,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.vidioc_g_register		= imageon_rx_g_register,
	.vidioc_s_register		= imageon_rx_s_register,
#endif
};

void imageon_rx_set_funcs(struct video_device *vdev)
{
	vdev->ioctl_ops = &imageon_rx_ioctl_ops;
}
