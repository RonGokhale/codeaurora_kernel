/*
 * Copyright (C) 2013, NVIDIA Corporation.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sub license,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the
 * next paragraph) shall be included in all copies or substantial portions
 * of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include <linux/backlight.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>

#include <drm/drm_crtc.h>
#include <drm/drm_panel.h>

struct panel_desc {
	const struct drm_display_mode *modes;
	unsigned int num_modes;

	struct {
		unsigned int width;
		unsigned int height;
	} size;
};

/* TODO: convert to gpiod_*() API once it's been merged */
#define GPIO_ACTIVE_LOW	(1 << 0)

struct panel_simple {
	struct drm_panel base;
	bool enabled;

	const struct panel_desc *desc;

	struct backlight_device *backlight;
	struct regulator *supply;

	unsigned long enable_gpio_flags;
	int enable_gpio;
};

static inline struct panel_simple *to_panel_simple(struct drm_panel *panel)
{
	return container_of(panel, struct panel_simple, base);
}

static void panel_simple_disable(struct drm_panel *panel)
{
	struct panel_simple *p = to_panel_simple(panel);

	if (!p->enabled)
		return;

	if (p->backlight) {
		p->backlight->props.power = FB_BLANK_POWERDOWN;
		backlight_update_status(p->backlight);
	}

	if (gpio_is_valid(p->enable_gpio)) {
		if (p->enable_gpio_flags & GPIO_ACTIVE_LOW)
			gpio_set_value(p->enable_gpio, 1);
		else
			gpio_set_value(p->enable_gpio, 0);
	}

	regulator_disable(p->supply);
	p->enabled = false;
}

static void panel_simple_enable(struct drm_panel *panel)
{
	struct panel_simple *p = to_panel_simple(panel);
	int err;

	if (p->enabled)
		return;

	err = regulator_enable(p->supply);
	if (err < 0)
		dev_err(panel->dev, "failed to enable supply: %d\n", err);

	if (gpio_is_valid(p->enable_gpio)) {
		if (p->enable_gpio_flags & GPIO_ACTIVE_LOW)
			gpio_set_value(p->enable_gpio, 0);
		else
			gpio_set_value(p->enable_gpio, 1);
	}

	if (p->backlight) {
		p->backlight->props.power = FB_BLANK_UNBLANK;
		backlight_update_status(p->backlight);
	}

	p->enabled = true;
}

static int panel_simple_get_modes(struct drm_panel *panel)
{
	struct panel_simple *p = to_panel_simple(panel);
	struct drm_display_mode *mode;
	unsigned int i;

	for (i = 0; i < p->desc->num_modes; i++) {
		mode = drm_mode_duplicate(panel->drm, &p->desc->modes[i]);
		if (!mode)
			return -ENOMEM;

		drm_mode_set_name(mode);

		drm_mode_probed_add(panel->connector, mode);
	}

	return p->desc->num_modes;
}

static const struct drm_panel_funcs panel_simple_funcs = {
	.disable = panel_simple_disable,
	.enable = panel_simple_enable,
	.get_modes = panel_simple_get_modes,
};

static const struct drm_display_mode auo_b101aw03_mode = {
	.clock = 51450,
	.hdisplay = 1024,
	.hsync_start = 1024 + 156,
	.hsync_end = 1024 + 156 + 8,
	.htotal = 1024 + 156 + 8 + 156,
	.vdisplay = 600,
	.vsync_start = 600 + 16,
	.vsync_end = 600 + 16 + 6,
	.vtotal = 600 + 16 + 6 + 16,
	.vrefresh = 60,
};

static const struct panel_desc auo_b101aw03 = {
	.modes = &auo_b101aw03_mode,
	.num_modes = 1,
	.size = {
		.width = 223,
		.height = 125,
	},
};

static const struct drm_display_mode chunghwa_claa101wb01_mode = {
	.clock = 69300,
	.hdisplay = 1366,
	.hsync_start = 1366 + 48,
	.hsync_end = 1366 + 48 + 32,
	.htotal = 1366 + 48 + 32 + 20,
	.vdisplay = 768,
	.vsync_start = 768 + 16,
	.vsync_end = 768 + 16 + 8,
	.vtotal = 768 + 16 + 8 + 16,
	.vrefresh = 60,
};

static const struct panel_desc chunghwa_claa101wb01 = {
	.modes = &chunghwa_claa101wb01_mode,
	.num_modes = 1,
	.size = {
		.width = 223,
		.height = 125,
	},
};

static const struct drm_display_mode panasonic_vvx10f004b00_mode = {
	.clock = 154700,
	.hdisplay = 1920,
	.hsync_start = 1920 + 154,
	.hsync_end = 1920 + 154 + 16,
	.htotal = 1920 + 154 + 16 + 32,
	.vdisplay = 1200,
	.vsync_start = 1200 + 17,
	.vsync_end = 1200 + 17 + 2,
	.vtotal = 1200 + 17 + 2 + 16,
	.vrefresh = 60,
};

static const struct panel_desc panasonic_vvx10f004b00 = {
	.modes = &panasonic_vvx10f004b00_mode,
	.num_modes = 1,
	.size = {
		.width = 217,
		.height = 136,
	},
};

static const struct of_device_id panel_simple_of_match[] = {
	{
		.compatible = "auo,b101aw03",
		.data = &auo_b101aw03,
	}, {
		.compatible = "chunghwa,claa101wb01",
		.data = &chunghwa_claa101wb01
	}, {
		.compatible = "panasonic,vvx10f004b00",
		.data = &panasonic_vvx10f004b00
	}, {
		/* sentinel */
	}
};
MODULE_DEVICE_TABLE(of, panel_simple_of_match);

static int panel_simple_probe(struct platform_device *pdev)
{
	const struct of_device_id *id;
	struct device_node *backlight;
	struct panel_simple *panel;
	enum of_gpio_flags flags;
	int err;

	panel = devm_kzalloc(&pdev->dev, sizeof(*panel), GFP_KERNEL);
	if (!panel)
		return -ENOMEM;

	id = of_match_node(panel_simple_of_match, pdev->dev.of_node);
	if (!id)
		return -ENODEV;

	panel->desc = id->data;
	panel->enabled = false;

	panel->supply = devm_regulator_get(&pdev->dev, "power");
	if (IS_ERR(panel->supply))
		return PTR_ERR(panel->supply);

	panel->enable_gpio = of_get_named_gpio_flags(pdev->dev.of_node,
						     "enable-gpios", 0,
						     &flags);
	if (gpio_is_valid(panel->enable_gpio)) {
		unsigned int value;

		if (flags & OF_GPIO_ACTIVE_LOW)
			panel->enable_gpio_flags |= GPIO_ACTIVE_LOW;

		err = gpio_request(panel->enable_gpio, "enable");
		if (err < 0) {
			dev_err(&pdev->dev, "failed to request GPIO#%u: %d\n",
				panel->enable_gpio, err);
			return err;
		}

		value = (panel->enable_gpio_flags & GPIO_ACTIVE_LOW) != 0;

		err = gpio_direction_output(panel->enable_gpio, value);
		if (err < 0) {
			dev_err(&pdev->dev, "failed to setup GPIO%u: %d\n",
				panel->enable_gpio, err);
			goto free_gpio;
		}
	}

	backlight = of_parse_phandle(pdev->dev.of_node, "backlight", 0);
	if (backlight) {
		panel->backlight = of_find_backlight_by_node(backlight);
		if (!panel->backlight) {
			err = -EPROBE_DEFER;
			goto free_gpio;
		}

		of_node_put(backlight);
	}

	drm_panel_init(&panel->base);
	panel->base.dev = &pdev->dev;
	panel->base.funcs = &panel_simple_funcs;

	err = drm_panel_add(&panel->base);
	if (err < 0)
		goto free_gpio;

	platform_set_drvdata(pdev, panel);

	return 0;

free_gpio:
	if (gpio_is_valid(panel->enable_gpio))
		gpio_free(panel->enable_gpio);

	return err;
}

static int panel_simple_remove(struct platform_device *pdev)
{
	struct panel_simple *panel = platform_get_drvdata(pdev);

	drm_panel_detach(&panel->base);
	drm_panel_remove(&panel->base);

	panel_simple_disable(&panel->base);

	if (panel->backlight)
		put_device(&panel->backlight->dev);

	if (gpio_is_valid(panel->enable_gpio))
		gpio_free(panel->enable_gpio);

	regulator_disable(panel->supply);

	return 0;
}

static struct platform_driver panel_simple_driver = {
	.driver = {
		.name = "panel-simple",
		.owner = THIS_MODULE,
		.of_match_table = panel_simple_of_match,
	},
	.probe = panel_simple_probe,
	.remove = panel_simple_remove,
};
module_platform_driver(panel_simple_driver);

MODULE_DESCRIPTION("DRM Driver for Simple Panels");
MODULE_AUTHOR("Thierry Reding <treding@nvidia.com>");
MODULE_LICENSE("GPL v2");
