/*
 * drivers/video/backlight/tegra-pwm.c
 *
 * Tegra pulse-width-modulation backlight control
 *
 * Copyright (c) 2009-2010, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/backlight.h>
#include <linux/clk.h>
#include <linux/fb.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <mach/pwm-bl.h>

#define MAX_INTENSITY 0xff

struct tegra_pwm_bl {
	int		intensity;
	int		clk_enb;
	int		mode;
	int		gpio;
	struct device	*dev;
	struct clk	*clk;
	void __iomem	*reg;
};

static void pwmbl_send_intensity(struct tegra_pwm_bl *pwm, int intensity)
{
	unsigned long rate;
	unsigned long divisor;
	u32 val = 0;

	if (intensity && !pwm->clk_enb) {
		clk_enable(pwm->clk);
		pwm->clk_enb = 1;
	}

	rate = clk_get_rate(pwm->clk);
	divisor = (rate + 100) / 200;

	if (intensity < MAX_INTENSITY) {
		unsigned int jog = (intensity <= 0x7f) ?: intensity + 1;	
		val = 0x80000000ul | (jog << 16);
	}

	val |= divisor;
	writel(divisor, pwm->reg);

	if (!intensity && pwm->clk_enb) {
		clk_disable(pwm->clk);
		pwm->clk_enb = 0;
	}
}

static void pwmbl_set_power(struct tegra_pwm_bl *pwm, int mode)
{
	if (mode != FB_BLANK_UNBLANK)
		mode = FB_BLANK_POWERDOWN;

	if (mode == FB_BLANK_UNBLANK) {
		pwmbl_send_intensity(pwm, pwm->intensity);
		if (pwm->gpio >= -1)
			gpio_set_value(pwm->gpio, 1);
	} else {
		if (pwm->gpio >= -1)
			gpio_set_value(pwm->gpio, 0);
		pwmbl_send_intensity(pwm, 0);
	}
}

static int pwmbl_update_status(struct backlight_device *dev)
{
	struct tegra_pwm_bl *pwm = dev_get_drvdata(&dev->dev);
	int mode;

	if (pwm->intensity != dev->props.brightness) {
		if (pwm->mode == FB_BLANK_UNBLANK) {
			pwmbl_send_intensity(pwm, dev->props.brightness);
			if (dev->props.brightness == 0) {
				pwmbl_set_power(pwm, FB_BLANK_POWERDOWN);
				pwm->mode = FB_BLANK_POWERDOWN;
			}
		}
		pwm->intensity = dev->props.brightness;
	}

	mode = dev->props.fb_blank;
	if ((mode != FB_BLANK_UNBLANK) || (pwm->intensity == 0))
		mode = FB_BLANK_POWERDOWN;

	if (pwm->mode != mode) {
		pwmbl_set_power(pwm, dev->props.fb_blank);
		pwm->mode = mode;
	}

	return 0;
}

static int pwmbl_get_intensity(struct backlight_device *dev)
{
	struct tegra_pwm_bl *bl = dev_get_drvdata(&dev->dev);
	return bl->intensity;
}

static const struct backlight_ops pwmbl_ops = {
	.get_brightness	= pwmbl_get_intensity,
	.update_status	= pwmbl_update_status,
};

static int pwmbl_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct backlight_device *dev = platform_get_drvdata(pdev);
	struct tegra_pwm_bl *pwm = dev_get_drvdata(&dev->dev);

	if (pwm->mode != FB_BLANK_POWERDOWN)
		pwmbl_set_power(pwm, FB_BLANK_POWERDOWN);

	return 0;
}

static int pwmbl_resume(struct platform_device *pdev)
{
	struct backlight_device *dev = platform_get_drvdata(pdev);
	struct tegra_pwm_bl *pwm = dev_get_drvdata(&dev->dev);

	if (pwm->mode != FB_BLANK_POWERDOWN)
		pwmbl_set_power(pwm, pwm->mode);

	return 0;
}

static int pwmbl_probe(struct platform_device *pdev)
{
	struct tegra_pwm_bl_platform_data *pdata = pdev->dev.platform_data;
	struct backlight_properties props;
	struct backlight_device *dev = NULL;
	struct tegra_pwm_bl *pwm;
	struct resource *r;
	int err;

	if (!pdata) {
		dev_err(&pdev->dev, "no platform data?\n");
		return -ENXIO;
	}

	pwm = kzalloc(sizeof(*pwm), GFP_KERNEL);
	if (!pwm) {
		dev_err(&pdev->dev, "out of memory\n");
		return -ENOMEM;
	}

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (r == NULL) {
		dev_err(&pdev->dev, "no memory resources?\n");
		err = -ENODEV;
		goto out_early;
	}

	if (!request_mem_region(r->start, resource_size(r),
				dev_name(&pdev->dev))) {
		dev_err(&pdev->dev, "couldn't request memory region\n");
		err = -EBUSY;
		goto out_early;
	}

	pwm->reg = ioremap(r->start, resource_size(r));
	if (!pwm->reg) {
		dev_err(&pdev->dev, "couldn't ioremap registers\n");
		err = -ENOMEM;
		goto out;
	}

	pwm->gpio = -1;
	if (pdata->pwr_gpio != -1) {
		err = gpio_request(pdata->pwr_gpio, "tegra-pwm-bl pwr");
		if (err < 0) {
			dev_err(&pdev->dev, "couldn't request power GPIO\n");
			goto out;
		}
		pwm->gpio = pdata->pwr_gpio;
	}

	err = gpio_direction_output(pwm->gpio, (pdata->init_intensity) ? 1 : 0);
	if (err) {
		dev_err(&pdev->dev, "couldn't configure GPIO as output\n");
		goto out;
	}

	pwm->clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR_OR_NULL(pwm->clk)) {
		dev_err(&pdev->dev, "couldn't get PWM clock\n");
		err = PTR_ERR(pwm->clk);
		goto out;
	}

	memset(&props, 0, sizeof(props));
	props.max_brightness = MAX_INTENSITY;
	dev = backlight_device_register("tegra-pwm-bl", &pdev->dev, pwm,
					&pwmbl_ops, &props);

	if (IS_ERR(dev)) {
		err = PTR_ERR(dev);
		goto out;
	}

	pwm->mode = (pdata->init_intensity) ? FB_BLANK_UNBLANK : FB_BLANK_POWERDOWN;

	pwm->dev = &pdev->dev;
	platform_set_drvdata(pdev, dev);

	dev_info(&pdev->dev, "initialized\n");

	return 0;
out:
	if (!IS_ERR_OR_NULL(pwm->clk))
		clk_put(pwm->clk);
	if (pwm->reg)
		iounmap(pwm->reg);
	if (pwm->gpio != -1)
		gpio_free(pwm->gpio);
	if (!IS_ERR_OR_NULL(dev))
		backlight_device_unregister(dev);
	release_mem_region(r->start, resource_size(r));
out_early:
	kfree(pwm);

	return err;
}

static int pwmbl_remove(struct platform_device *pdev)
{
	struct backlight_device *dev = platform_get_drvdata(pdev);
	struct tegra_pwm_bl *pwm = dev_get_drvdata(&dev->dev);
	struct resource *r = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	backlight_device_unregister(dev);
	pwmbl_set_power(pwm, FB_BLANK_POWERDOWN);
	clk_put(pwm->clk);
	if (pwm->gpio >= 0)
		gpio_free(pwm->gpio);
	iounmap(pwm->reg);
	release_mem_region(r->start, resource_size(r));
	kfree(pwm);
	return 0;
}

static struct platform_driver pwmbl_driver = {
	.probe		= pwmbl_probe,
	.remove		= pwmbl_remove,
	.suspend	= pwmbl_suspend,
	.resume		= pwmbl_resume,
	.driver		= {
		.name	= "tegra-pwm-bl",
	},
};

static int __init pwmbl_init(void)
{
	return platform_driver_register(&pwmbl_driver);
}

static void __exit pwmbl_exit(void)
{
	platform_driver_unregister(&pwmbl_driver);
}

module_init(pwmbl_init);
module_exit(pwmbl_exit);
MODULE_AUTHOR("NVIDIA Corporation");
MODULE_DESCRIPTION("Tegra PWM LCD Backlight driver");
MODULE_LICENSE("GPL");
