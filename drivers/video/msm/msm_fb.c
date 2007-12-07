/* drivers/video/msm_fb/msm_fb.c
 *
 * Core MSM framebuffer driver.
 *
 * Copyright (C) 2007 Google Incorporated
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/fb.h>
#include <linux/delay.h>

#include <linux/freezer.h>
#include <linux/wait.h>

#include <asm/io.h>

#include "msm_fb.h"

#define PRINT_FPS 0

static int msmfb_open(struct fb_info *info, int user)
{
	return 0;
}

static int msmfb_release(struct fb_info *info, int user)
{
	return 0;
}

static unsigned yoffset;
static volatile unsigned frame;
static volatile int running;
static volatile struct {
	int need_update;
	int left;
	int top;
	int eright; /* exclusive */
	int ebottom; /* exclusive */
} msmfb_update_info;
static DEFINE_SPINLOCK(msmfb_update_lock);

static DECLARE_WAIT_QUEUE_HEAD(update_wq);
static DECLARE_WAIT_QUEUE_HEAD(frame_wq);

static int updater(void *_pi)
{
	struct mddi_panel_info *pi = _pi;
	uint32_t x, y, w, h;
	unsigned addr;
	unsigned long irq_flags;
#if PRINT_FPS
	ktime_t t1 = ktime_get();
	ktime_t t2, t3, t4, t5;
	int64_t dt;
	int rel_frame_count = 0;
#endif

	running = 1;

	daemonize("msm_fb");
	set_user_nice(current, -10);
	set_freezable();

	for (;;) {
		wait_event_interruptible(update_wq, msmfb_update_info.need_update);
		if(try_to_freeze())
			continue;

		spin_lock_irqsave(&msmfb_update_lock, irq_flags);
		msmfb_update_info.need_update = 0;
		x = msmfb_update_info.left;
		y = msmfb_update_info.top;
		msmfb_update_info.left = pi->width + 1;
		msmfb_update_info.top = pi->height + 1;
		w = msmfb_update_info.eright - x;
		h = msmfb_update_info.ebottom - y;
		msmfb_update_info.eright = 0;
		msmfb_update_info.ebottom = 0;
		spin_unlock_irqrestore(&msmfb_update_lock, irq_flags);
#if PRINT_FPS
		t2 = ktime_get();
#endif
		if (pi->panel_ops->wait_vsync)
			pi->panel_ops->wait_vsync(pi);
		else
			msleep(16);
#if PRINT_FPS
		t3 = ktime_get();
#endif
		if (w > pi->width || h > pi->height || w == 0 || h == 0) {
			printk(KERN_INFO "invalid update: %d %d %d %d\n",
			       x, y, w, h);
			mddi_activate_link(pi->mddi);
			/* some clients clear their vsync interrupt
			 * when the link activates
			 */
		} else {
			addr = ((pi->width * (yoffset + y) + x) * 2);
			mdp_dma_to_mddi(addr, pi->width * 2, w, h, x, y);
			mdp_dma_wait();
		}
#if PRINT_FPS
		rel_frame_count++;
		t4 = ktime_get();
		dt = ktime_to_ns(ktime_sub(t4, t1));
		if (dt > NSEC_PER_SEC) {
			if (dt < UINT_MAX) {
				int64_t fps = (int64_t)rel_frame_count * NSEC_PER_SEC * 100;
				int64_t vsync = (int64_t)NSEC_PER_SEC * 100;
				int64_t last = (int64_t)NSEC_PER_SEC * 100;
				int64_t dma = (int64_t)NSEC_PER_SEC * 100;
				do_div(fps, dt);
				do_div(vsync, ktime_to_ns(ktime_sub(t3, t5)));
				do_div(last, ktime_to_ns(ktime_sub(t4, t2)));
				do_div(dma, ktime_to_ns(ktime_sub(t4, t3)));
				printk(KERN_INFO "msm_fb: fps*100 = %lld, "
				       "vsync %lld, last %lld, dma %lld\n",
				       fps, vsync, last, dma);
			}
			t1 = t4;
			rel_frame_count = 0;
		}
		t5 = t3;
#endif
		frame++;
		wake_up(&frame_wq);
	}

	return 0;
}

static void msmfb_update(struct fb_info *info, uint32_t left, uint32_t top, uint32_t eright, uint32_t ebottom)
{
	unsigned long irq_flags;
	spin_lock_irqsave(&msmfb_update_lock, irq_flags);
	if (left < msmfb_update_info.left)
		msmfb_update_info.left = left;
	if (top < msmfb_update_info.top)
		msmfb_update_info.top = top;
	if (eright > msmfb_update_info.eright)
		msmfb_update_info.eright = eright;
	if (ebottom > msmfb_update_info.ebottom)
		msmfb_update_info.ebottom = ebottom;
	msmfb_update_info.need_update = 1;
	spin_unlock_irqrestore(&msmfb_update_lock, irq_flags);
	wake_up(&update_wq);
}

static int msmfb_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
	if (var->xres != info->var.xres) return -EINVAL;
	if (var->yres != info->var.yres) return -EINVAL;
	if (var->xres_virtual != info->var.xres_virtual) return -EINVAL;
	if (var->yres_virtual != info->var.yres_virtual) return -EINVAL;
	if (var->xoffset != info->var.xoffset) return -EINVAL;
	if (var->bits_per_pixel != info->var.bits_per_pixel) return -EINVAL;
	if (var->grayscale != info->var.grayscale) return -EINVAL;
	return 0;
}

int msmfb_pan_display(struct fb_var_screeninfo *var, struct fb_info *info)
{
	unsigned thisframe;
	yoffset = var->yoffset;

	if (!running) return 0;

	/* don't return until we start painting this frame */
	thisframe = frame;
	if (var->reserved[0] == 0x54445055) { /* "UPDT" */
#if 0
		printk("pan frame %d-%d, rect %d %d %d %d\n",
		       thisframe, frame, var->reserved[1] & 0xffff,
		       var->reserved[1] >> 16, var->reserved[2] & 0xffff,
		       var->reserved[2] >> 16);
#endif
		msmfb_update(info, var->reserved[1] & 0xffff,
			     var->reserved[1] >> 16,
			     var->reserved[2] & 0xffff,
			     var->reserved[2] >> 16);
	} else {
		msmfb_update(info, 0, 0, info->var.xres, info->var.yres);
	}
	if(wait_event_interruptible_timeout(frame_wq, thisframe != frame, HZ) == 0) {
		printk("msmfb_pan_display timeout waiting for frame change, %d %d\n", thisframe, frame);
	}
	return 0;
}

static void msmfb_fillrect(struct fb_info *p, const struct fb_fillrect *rect)
{
	cfb_fillrect(p, rect);
	msmfb_update(p, rect->dx, rect->dy, rect->dx + rect->width, rect->dy + rect->height);
}

static void msmfb_copyarea(struct fb_info *p, const struct fb_copyarea *area)
{
	cfb_copyarea(p, area);
	msmfb_update(p, area->dx, area->dy, area->dx + area->width, area->dy + area->height);
}

static void msmfb_imageblit(struct fb_info *p, const struct fb_image *image)
{
	cfb_imageblit(p, image);
	msmfb_update(p, image->dx, image->dy, image->dx + image->width, image->dy + image->height);
}

static struct fb_ops msmfb_ops = {
	.owner =       THIS_MODULE,
	.fb_open =     msmfb_open,
	.fb_release =  msmfb_release,
	.fb_check_var = msmfb_check_var,
	.fb_pan_display = msmfb_pan_display,
	.fb_fillrect	= msmfb_fillrect,
	.fb_copyarea	= msmfb_copyarea,
	.fb_imageblit	= msmfb_imageblit,
};

static unsigned PP[16];

static int msmfb_probe(struct platform_device *pdev)
{
	unsigned char *fbram;
	struct fb_info *info;
	struct mddi_panel_info *pi = pdev->dev.platform_data;
	int r;

	printk(KERN_INFO "msmfb_probe() installing %d x %d panel\n",
	       pi->width, pi->height);

	fbram = ioremap(0, 8 * 1024 * 1024);

	if (fbram == 0) {
		printk(KERN_ERR "cannot allocate fbram!\n");
		return -ENOMEM;
	}

	info = framebuffer_alloc(0, &pdev->dev);

	info->screen_base = fbram;
	info->fix.smem_start = 0;
	info->fix.smem_len = 8 * 1024 * 1024;
	info->fix.ypanstep = 1;

	info->fbops = &msmfb_ops;
	info->flags = FBINFO_DEFAULT;

	info->fix.type = FB_TYPE_PACKED_PIXELS;
	info->fix.visual = FB_VISUAL_TRUECOLOR;
	info->fix.line_length = pi->width * 2;

	info->var.xres = pi->width;
	info->var.yres = pi->height;
	info->var.xres_virtual = pi->width;
	info->var.yres_virtual = pi->height * 2;
	info->var.bits_per_pixel = 16;
	info->var.accel_flags = 0;

	info->var.yoffset = 0;
	info->var.reserved[0] = 0x54445055;
	info->var.reserved[1] = 0;
	info->var.reserved[2] = (uint16_t)pi->width | ((uint32_t)pi->height << 16);

	info->var.red.offset = 11;
	info->var.red.length = 5;
	info->var.red.msb_right = 0;
	info->var.green.offset = 5;
	info->var.green.length = 6;
	info->var.green.msb_right = 0;
	info->var.blue.offset = 0;
	info->var.blue.length = 5;
	info->var.blue.msb_right = 0;

	r = fb_alloc_cmap(&info->cmap, 16, 0);
	info->pseudo_palette = PP;

	PP[0] = 0;
	for (r = 1; r < 16; r++)
		PP[r] = 0xffffffff;

	r = register_framebuffer(info);
	if (r) return r;

	kernel_thread(updater, pi, 0);
	return 0;
}

static int msmfb_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver mddi_panel_driver = {
	.probe   = msmfb_probe,
	.remove  = msmfb_remove,
	.driver  = { .name = "mddi_panel" },
};

extern int mdp_init(void);

static int __init msmfb_init(void)
{
	int ret;

	ret = mdp_init();
	if (ret)
		return ret;

	return platform_driver_register(&mddi_panel_driver);
}

module_init(msmfb_init);
