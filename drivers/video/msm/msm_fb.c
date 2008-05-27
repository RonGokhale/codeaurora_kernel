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
#ifdef CONFIG_ANDROID_POWER
#include <linux/android_power.h>
#endif
#include <linux/msm_mdp.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/arch/msm_fb.h>
#include <linux/workqueue.h>

#define PRINT_FPS 0
#define PRINT_BLIT_TIME 0

#define SLEEPING 0x4
#define UPDATING 0x3
#define FULL_UPDATE_DONE 0x2
#define WAKING 0x1
#define AWAKE 0x0

struct msmfb_info {
	struct fb_info *fb_info;
	struct mddi_panel_info *panel_info;
	unsigned yoffset;
	unsigned need_update;
	unsigned frame_requested;
	unsigned frame_done;
	int sleeping;
	struct {
		int left;
		int top;
		int eright; /* exclusive */
		int ebottom; /* exclusive */
	} update_info;

#ifdef CONFIG_ANDROID_POWER
	android_early_suspend_t early_suspend;
	android_early_suspend_t slightly_earlier_suspend;
#endif
	spinlock_t update_lock;
	wait_queue_head_t frame_wq;
	char* black;
	struct work_struct resume_work;
	struct msmfb_callback dma_callback;
	struct msmfb_callback vsync_callback;
	struct hrtimer fake_vsync;
};

static int msmfb_open(struct fb_info *info, int user)
{
	return 0;
}

static int msmfb_release(struct fb_info *info, int user)
{
	return 0;
}

/* Called from dma interrupt handler, must not sleep */
static void msmfb_handle_dma_interrupt(struct msmfb_callback *callback)
{
	unsigned long irq_flags;
	struct msmfb_info *par  = container_of(callback, struct msmfb_info,
					       dma_callback);
#if PRINT_FPS
	int64_t dt;
	ktime_t now;
	static int64_t frame_count;
	static ktime_t last_sec;
#endif

	spin_lock_irqsave(&par->update_lock, irq_flags);
	par->frame_done = par->frame_requested;
	if (unlikely(par->sleeping == UPDATING)) {
		par->sleeping = AWAKE;
		schedule_work(&par->resume_work);
	}
#if PRINT_FPS
	now = ktime_get();
	dt = ktime_to_ns(ktime_sub(now, last_sec));
	frame_count++;
	if (dt > NSEC_PER_SEC) {
		int64_t fps = frame_count * NSEC_PER_SEC * 100;
		frame_count = 0;
		last_sec = ktime_get();
		do_div(fps, dt);
		printk(KERN_INFO "fps * 100: %llu\n", fps);
	}
#endif
	spin_unlock_irqrestore(&par->update_lock, irq_flags);
	wake_up(&par->frame_wq);
}

static int msmfb_start_dma(struct msmfb_info *par)
{
	uint32_t x, y, w, h;
	unsigned addr;
	unsigned long irq_flags;
	uint32_t yoffset;
	struct mddi_panel_info *pi = par->panel_info;

	spin_lock_irqsave(&par->update_lock, irq_flags);
	if (par->frame_done == par->frame_requested) {
		spin_unlock_irqrestore(&par->update_lock, irq_flags);
		return -1;
	}
	x = par->update_info.left;
	y = par->update_info.top;
	w = par->update_info.eright - x;
	h = par->update_info.ebottom - y;
	yoffset = par->yoffset;
	par->update_info.left = pi->width + 1;
	par->update_info.top = pi->height + 1;
	par->update_info.eright = 0;
	par->update_info.ebottom = 0;
	spin_unlock_irqrestore(&par->update_lock, irq_flags);

	if (unlikely(w > pi->width || h > pi->height || w == 0 || h == 0)) {
		printk(KERN_INFO "invalid update: %d %d %d "
				"%d\n", x, y, w, h);
		mddi_activate_link(pi->mddi);
		return 0;
		/* some clients clear their vsync interrupt
		 * when the link activates
		 */
	}
	addr = ((pi->width * (yoffset + y) + x) * 2);
	mdp_dma_to_mddi(addr + par->fb_info->fix.smem_start,
			pi->width * 2, w, h, x, y, &par->dma_callback);
	return 0;
}

/* Called from esync interrupt handler, must not sleep */
static void msmfb_handle_vsync_interrupt(struct msmfb_callback *callback)
{
	struct msmfb_info *par  = container_of(callback, struct msmfb_info,
					       vsync_callback);
	msmfb_start_dma(par);
}

static enum hrtimer_restart msmfb_fake_vsync(struct hrtimer *timer)
{
	struct msmfb_info *par  = container_of(timer, struct msmfb_info,
					       fake_vsync);
	if (msmfb_start_dma(par))
		return HRTIMER_RESTART;
	return HRTIMER_NORESTART;
}

static void msmfb_pan_update(struct fb_info *info, uint32_t left, uint32_t top,
			 uint32_t eright, uint32_t ebottom, uint32_t yoffset,
			 int pan_display)
{
	struct msmfb_info *par = info->par;
	struct mddi_panel_info *pi = par->panel_info;
	unsigned long irq_flags;
#if PRINT_FPS
	ktime_t t1, t2;
	static uint64_t pans;
	static uint64_t dt;
	t1 = ktime_get();
#endif

restart:
	spin_lock_irqsave(&par->update_lock, irq_flags);
	/* if we are sleeping, on a pan_display wait 10ms (to throttle back
	 * drawing otherwise return */
	if (par->sleeping == SLEEPING) {
		spin_unlock_irqrestore(&par->update_lock, irq_flags);
		if (pan_display)
			wait_event_interruptible_timeout(par->frame_wq,
				par->sleeping == AWAKE, HZ/10);
		return;
	}

	/* on a full update, if the last frame has not completed, wait for it */
	if (pan_display && par->frame_requested != par->frame_done) {
		spin_unlock_irqrestore(&par->update_lock, irq_flags);
		if (wait_event_interruptible_timeout(par->frame_wq,
		    par->frame_done == par->frame_requested, HZ) <= 0) {
			printk(KERN_WARNING "msmfb_pan_display timeout waiting "
					    "for frame start, %d %d\n",
					    par->frame_requested,
					    par->frame_done);
			return;
		}
		goto restart;
	}

#if PRINT_FPS
	t2 = ktime_get();
	if (pan_display) {
		uint64_t temp = ktime_to_ns(ktime_sub(t2, t1));
		do_div(temp, 1000);
		dt += temp;
		pans++;
		if (pans > 1000) {
			do_div(dt, pans);
			printk("ave_wait_time: %lld\n", dt);
			dt = 0;
			pans = 0;
		}
	}
#endif

	/* if necessary, update the y offset, if this is the
	 * first full update on resume, set the sleeping state */
	if (pan_display) {
		par->yoffset = yoffset;
		if (par->sleeping == WAKING &&
		    left == 0 && top == 0 && eright == info->var.xres &&
		    ebottom == info->var.yres)
			par->sleeping = UPDATING;
	}

	/* set the update request */
	if (left < par->update_info.left)
		par->update_info.left = left;
	if (top < par->update_info.top)
		par->update_info.top = top;
	if (eright > par->update_info.eright)
		par->update_info.eright = eright;
	if (ebottom > par->update_info.ebottom)
		par->update_info.ebottom = ebottom;
	par->frame_requested++;
	spin_unlock_irqrestore(&par->update_lock, irq_flags);

	/* if the panel is all the way on wait for vsync, otherwise sleep
	 * for 16 ms (long enough for the dma to panel) and then begin dma */
	if (pi->panel_ops->request_vsync && (par->sleeping == AWAKE))
		pi->panel_ops->request_vsync(pi, &par->vsync_callback);
	else {
		if (!hrtimer_active(&par->fake_vsync)) {
			hrtimer_start(&par->fake_vsync,
				      ktime_set(0, NSEC_PER_SEC/60),
				      HRTIMER_MODE_REL);
		}
	}
}

static void msmfb_update(struct fb_info *info, uint32_t left, uint32_t top,
			 uint32_t eright, uint32_t ebottom)
{
	msmfb_pan_update(info, left, top, eright, ebottom, 0, 0);
}

#ifdef CONFIG_ANDROID_POWER
static void power_on_panel(struct work_struct *work)
{
	struct msmfb_info *par = container_of(work, struct msmfb_info,
					      resume_work);
	struct mddi_panel_info *pi = par->panel_info;
	pi->panel_ops->power(pi, 1);
}

static void msmfb_slightly_earlier_suspend(android_early_suspend_t *h)
{
	struct msmfb_info *par = container_of(h, struct msmfb_info,
					      slightly_earlier_suspend);
	struct mddi_panel_info *pi = par->panel_info;

	par->sleeping = SLEEPING;
	/* blank the screen */
	msmfb_update(par->fb_info, 0, 0, par->fb_info->var.xres,
		     par->fb_info->var.yres);
	mdp_dma_to_mddi(virt_to_phys(par->black), 0,
			par->fb_info->var.xres, par->fb_info->var.yres, 0, 0,
			NULL);
	mdp_dma_wait();
	/* turn off the backlight and the panel */
	pi->panel_ops->power(pi, 0);
}

static void msmfb_early_suspend(android_early_suspend_t *h)
{
}

static void msmfb_early_resume(android_early_suspend_t *h)
{
	struct msmfb_info *par = container_of(h, struct msmfb_info,
				 early_suspend);
	par->sleeping = WAKING;
}

static void msmfb_slightly_later_resume(android_early_suspend_t *h)
{
}
#endif

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
	/* "UPDT" */
	if (var->reserved[0] == 0x54445055) {
#if 0
		printk(KERN_INFO "pan frame %d-%d, rect %d %d %d %d\n",
		       par->frame_requested, par->frame_done,
		       var->reserved[1] & 0xffff,
		       var->reserved[1] >> 16, var->reserved[2] & 0xffff,
		       var->reserved[2] >> 16);
#endif
		msmfb_pan_update(info, var->reserved[1] & 0xffff,
				 var->reserved[1] >> 16,
				 var->reserved[2] & 0xffff,
				 var->reserved[2] >> 16, var->yoffset, 1);
	} else {
		msmfb_pan_update(info, 0, 0, info->var.xres, info->var.yres,
		var->yoffset, 1);
	}
	return 0;
}

static void msmfb_fillrect(struct fb_info *p, const struct fb_fillrect *rect)
{
	cfb_fillrect(p, rect);
	msmfb_update(p, rect->dx, rect->dy, rect->dx + rect->width,
		     rect->dy + rect->height);
}

static void msmfb_copyarea(struct fb_info *p, const struct fb_copyarea *area)
{
	cfb_copyarea(p, area);
	msmfb_update(p, area->dx, area->dy, area->dx + area->width,
		     area->dy + area->height);
}

static void msmfb_imageblit(struct fb_info *p, const struct fb_image *image)
{
	cfb_imageblit(p, image);
	msmfb_update(p, image->dx, image->dy, image->dx + image->width,
		     image->dy + image->height);
}


static int msmfb_blit(struct fb_info *info, void __user *p)
{
	struct mdp_blit_req req;
	struct mdp_blit_req_list req_list;
	int i;
	int ret;

	if (copy_from_user(&req_list, p, sizeof(req_list)))
		return -EFAULT;

	for (i = 0; i < req_list.count; i++) {
		struct mdp_blit_req_list *list =
			(struct mdp_blit_req_list *)p;
		if (copy_from_user(&req, &list->req[i], sizeof(req)))
			return -EFAULT;
		ret = mdp_blit(info, &req);
		if (ret)
			return ret;
	}
	return 0;
}


DECLARE_MUTEX(mdp_ppp_sem);

static int msmfb_ioctl(struct fb_info *p, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int ret;
#if PRINT_BLIT_TIME
	ktime_t t1, t2;
#endif

	switch (cmd)
	{
		case MSMFB_GRP_DISP:
			mdp_set_grp_disp(arg);
			break;
		case MSMFB_BLIT:
#if PRINT_BLIT_TIME
			t1 = ktime_get();
#endif
			down(&mdp_ppp_sem);
			ret = msmfb_blit(p, argp);
			up(&mdp_ppp_sem);
			if (ret)
				return ret;
#if PRINT_BLIT_TIME
			t2 = ktime_get();
			printk(KERN_INFO "total %lld\n",
			       ktime_to_ns(t2) - ktime_to_ns(t1));
#endif
			break;
		default:
			printk(KERN_INFO "msmfb unknown ioctl: %d\n", cmd);
			return -EINVAL;
	}
	return 0;
}

static struct fb_ops msmfb_ops = {
	.owner = THIS_MODULE,
	.fb_open = msmfb_open,
	.fb_release = msmfb_release,
	.fb_check_var = msmfb_check_var,
	.fb_pan_display = msmfb_pan_display,
	.fb_fillrect = msmfb_fillrect,
	.fb_copyarea = msmfb_copyarea,
	.fb_imageblit = msmfb_imageblit,
	.fb_ioctl = msmfb_ioctl,
};

static unsigned PP[16];

static int msmfb_probe(struct platform_device *pdev)
{
	unsigned char *fbram;
	struct fb_info *info;
	struct msmfb_info *par;
	struct mddi_panel_info *pi = pdev->dev.platform_data;
	int r;

	printk(KERN_INFO "msmfb_probe() installing %d x %d panel\n",
	       pi->width, pi->height);

	fbram = ioremap(pi->fb_base, pi->fb_size);

	if (fbram == 0) {
		printk(KERN_ERR "cannot allocate fbram!\n");
		return -ENOMEM;
	}

	info = framebuffer_alloc(sizeof(struct msmfb_info), &pdev->dev);
	par = info->par;
	par->fb_info = info;
	par->panel_info = pi;
	par->dma_callback.func = msmfb_handle_dma_interrupt;
	par->vsync_callback.func = msmfb_handle_vsync_interrupt;
	hrtimer_init(&par->fake_vsync, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	par->fake_vsync.function = msmfb_fake_vsync;
	spin_lock_init(&par->update_lock);
	init_waitqueue_head(&par->frame_wq);

	info->screen_base = fbram;
	strncpy(info->fix.id, "msmfb", 16);
	info->fix.smem_start = pi->fb_base;
	info->fix.smem_len = pi->fb_size;
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
	info->var.reserved[2] = (uint16_t)pi->width |
				((uint32_t)pi->height << 16);

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

	INIT_WORK(&par->resume_work, power_on_panel);
	par->black = kmalloc(2*par->fb_info->var.xres, GFP_KERNEL);
	memset(par->black, 0, 2*par->fb_info->var.xres);

	r = register_framebuffer(info);
	if (r)
		return r;

#ifdef CONFIG_ANDROID_POWER
	par->slightly_earlier_suspend.suspend = msmfb_slightly_earlier_suspend;
	par->slightly_earlier_suspend.resume = msmfb_slightly_later_resume;
	par->slightly_earlier_suspend.level =
		ANDROID_EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
	android_register_early_suspend(&par->slightly_earlier_suspend);
	par->early_suspend.suspend = msmfb_early_suspend;
	par->early_suspend.resume = msmfb_early_resume;
	par->early_suspend.level = ANDROID_EARLY_SUSPEND_LEVEL_DISABLE_FB - 1;
	android_register_early_suspend(&par->early_suspend);
#endif

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
