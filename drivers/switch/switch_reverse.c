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
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/switch.h>
#include <linux/reverse.h>
#include <linux/input.h>

#include "pic.h"
#include "A.h"
#include "B.h"

struct reverse_reverse_data {
	struct switch_dev sdev;
	struct input_dev *idev;
	unsigned gpio;
	unsigned int key_code;
	unsigned debounce;
	const char *name_on;
	const char *name_off;
	const char *state_on;
	const char *state_off;
	int irq;
	struct work_struct work;
	struct delayed_work detect_delayed_work;
};

static int reverse_continue;

#if 0
static void gpio_switch_work(struct work_struct *work)
{
	int state;
	struct reverse_reverse_data	*data =
		container_of(work, struct reverse_reverse_data, work);

	state = gpio_get_value(data->gpio);
	switch_set_state(&data->sdev, state);
}
#endif

extern int show_a;
extern int show_b;
extern int camera_preview_ready;
extern int init_camera_kthread(void);
extern int camera_preview_exit;
extern void exit_camera_kthread(void);

enum PicStatus{
	PIC_SHOWING,
	SPLASH_LOGO_SHOWING,
	ALL_CLEAR,
};

enum PicStatus pic_status = ALL_CLEAR;

enum AndroidUIStatus{
	SHOWING,
	NOT_SHOWING,
};

enum AndroidUIStatus android_ui_status = NOT_SHOWING;

static int camera_thread_enable;

void show_pic(void);
void shutdown_pic(void);
void pic_update(unsigned char *pic, unsigned int pos_x, unsigned int pos_y, unsigned int image_w, unsigned int image_h);
int recovery_splash_logo(void);
void pic_pan_display(void);

static struct delayed_work camera_detect_dwork;

enum reverse_flags {
	ENUM_ALL	= 0x00,
	ENUM_BG_A	= 0x01,
	ENUM_BG_B	= 0x02,
	ENUM_BG		= 0x03,
};

void show_reverse_pic(int flags)
{
	unsigned char *data = gImage + 4;
	int w = (*(gImage + 1) << 8) + *(gImage + 0);
	int h = (*(gImage + 3) << 8) + *(gImage + 2);

	unsigned char *data_a = a + 4;
	int w_a = (*(a + 1) << 8) + *(a + 0);
	int h_a = (*(a + 3) << 8) + *(a + 2);

	unsigned char *data_b = b + 4;
	int w_b = (*(b + 1) << 8) + *(b + 0);
	int h_b = (*(b + 3) << 8) + *(b + 2);

	switch (flags) {
	case ENUM_ALL:
		pic_update(data, 0, 0, w, h);			/* bg */
		pic_update(data_a, 170, 340, w_a, h_a);	/* 'A' */
		pic_update(data_b, 500, 340, w_b, h_b);	/* 'B' */
		pic_pan_display();
		break;
	case ENUM_BG_A:
		pic_update(data, 0, 0, w, h);			/* bg */
		pic_update(data_a, 170, 340, w_a, h_a);	/* 'A' */
		pic_pan_display();
		break;
	case ENUM_BG_B:
		pic_update(data, 0, 0, w, h);			/* bg */
		pic_update(data_b, 500, 340, w_b, h_b);	/* 'B' */
		pic_pan_display();
		break;
	case ENUM_BG:
		pic_update(data, 0, 0, w, h);				/* bg */
		pic_pan_display();
		break;
	}

	if (pic_status == ALL_CLEAR) {
		show_pic();
		pic_status = PIC_SHOWING;
	} else if (pic_status == SPLASH_LOGO_SHOWING)
		pic_status = PIC_SHOWING;
}

static void show_pic_exit(void)
{
	if ((pic_status == PIC_SHOWING)) {
		if (android_ui_status == SHOWING) {
			show_a = show_b = 1;
			shutdown_pic();
			pic_status = ALL_CLEAR;
		} else {
			recovery_splash_logo();
			pic_status = SPLASH_LOGO_SHOWING;
		}
	}
}

static void camera_detect_workqueue(struct work_struct *work)
{
	if (camera_preview_ready && ((pic_status == SPLASH_LOGO_SHOWING) || (pic_status == ALL_CLEAR)))
/*	if (((pic_status == SPLASH_LOGO_SHOWING) || (pic_status == ALL_CLEAR))) */
		show_reverse_pic(ENUM_ALL);
	else
		schedule_delayed_work(&camera_detect_dwork, msecs_to_jiffies(50));
}

static void reverse_detection_work(struct work_struct *work)
{
	int state;
	int rc = 0;
	struct reverse_reverse_data	*data =
		container_of(work, struct reverse_reverse_data, detect_delayed_work.work);

	state = gpio_get_value(data->gpio);
	switch_set_state(&data->sdev, !state);

	input_report_key(data->idev, data->key_code, !state);
	input_sync(data->idev);

	if (!state) {
		if (camera_thread_enable == 0) {
			printk("CAMERA_TEST, start camera preview thread ~~~~~~~~~~~~~~~~~ 00 %s \n", __func__);
			camera_thread_enable = 1;
			rc = init_camera_kthread();
		}
		schedule_delayed_work(&camera_detect_dwork, msecs_to_jiffies(50));
	} else {
		if (camera_thread_enable == 1) {
			camera_thread_enable = 0;
			printk("CAMERA_TEST ~~~~~~~~~~exit_camera_kthread~~~~~~~11  %s \n", __func__);
			exit_camera_kthread();
		}
		cancel_delayed_work_sync(&camera_detect_dwork);
		show_pic_exit();
	}

}

static irqreturn_t gpio_irq_handler(int irq, void *dev_id)
{
	struct reverse_reverse_data *data =
		(struct reverse_reverse_data *)dev_id;

/*	cancel_work_sync(&data->detect_delayed_work); */
	schedule_delayed_work(&data->detect_delayed_work, msecs_to_jiffies(data->debounce));
/*	schedule_work(&data->work); */
	return IRQ_HANDLED;
}

static ssize_t switch_gpio_print_state(struct switch_dev *sdev, char *buf)
{
	struct reverse_reverse_data	*data =
		container_of(sdev, struct reverse_reverse_data, sdev);
	const char *state;
	if (switch_get_state(sdev))
		state = data->state_on;
	else
		state = data->state_off;

	if (state)
		return snprintf(buf, sizeof(state), "%s\n", state);
	return -EPERM;
}

static ssize_t continues_show(struct device *dev,
						  struct device_attribute *attr,
						  char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", reverse_continue);
}

static ssize_t continues_store(struct device *dev,
						  struct device_attribute *attr,
						  const char *buf, size_t count)
{
	char *last = NULL;

	if (strcmp(buf, "show") == 0) {
		android_ui_status = SHOWING;
		if (pic_status == SPLASH_LOGO_SHOWING) { /* Now the splash logo is showing, so need shutdown it */
			shutdown_pic();
			pic_status = ALL_CLEAR;
		}
	}

	reverse_continue = simple_strtoul(buf, &last, 0);
	printk("reverse_continue = %d\n", reverse_continue);
	return count;
}

static DEVICE_ATTR(continues, S_IRUGO | S_IWUSR, continues_show, continues_store);

static int switch_reverse_probe(struct platform_device *pdev)
{
	struct reverse_switch_platform_data *pdata = pdev->dev.platform_data;
	struct reverse_reverse_data *reverse_data;
	unsigned long irq_flags;
	int ret = 0;

	if (!pdata)
		return -EBUSY;

	reverse_data = kzalloc(sizeof(struct reverse_reverse_data), GFP_KERNEL);
	if (!reverse_data)
		return -ENOMEM;

	reverse_data->sdev.name = pdata->name;
	reverse_data->gpio = pdata->gpio;
	reverse_data->key_code = pdata->key_code;
	reverse_data->debounce = pdata->debounce_time;
	reverse_data->name_on = pdata->name_on;
	reverse_data->name_off = pdata->name_off;
	reverse_data->state_on = pdata->state_on;
	reverse_data->state_off = pdata->state_off;
	reverse_data->sdev.print_state = switch_gpio_print_state;

	ret = switch_dev_register(&reverse_data->sdev);
	if (ret < 0)
		goto err_switch_dev_register;

	reverse_data->idev = input_allocate_device();
	if (!reverse_data->idev) {
		printk("Failed to allocate input dev\n");
		ret = -ENOMEM;
		goto err_input_dev_register;
	}

	reverse_data->idev->name = pdata->name;
	reverse_data->idev->phys = "reverse_keys/input0";
	reverse_data->idev->id.bustype = BUS_HOST;
	reverse_data->idev->dev.parent = &pdev->dev;
	reverse_data->idev->evbit[0] = BIT_MASK(EV_KEY);
	reverse_data->idev->keybit[BIT_WORD(pdata->key_code)] = BIT_MASK(pdata->key_code);

	ret = input_register_device(reverse_data->idev);
	if (ret) {
		printk("Can't register input device: %d\n", ret);
		goto err_input_reg;
	}

	ret = gpio_request(reverse_data->gpio, pdev->name);
	if (ret < 0)
		goto err_request_gpio;

	ret = gpio_direction_input(reverse_data->gpio);
	if (ret < 0)
		goto err_set_gpio_input;

	reverse_data->irq = gpio_to_irq(reverse_data->gpio);
	if (reverse_data->irq < 0) {
		ret = reverse_data->irq;
		goto err_detect_irq_num_failed;
	}

	irq_flags = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT;
	ret = request_irq(reverse_data->irq, gpio_irq_handler,
			  irq_flags, pdev->name, reverse_data);
	if (ret < 0)
		goto err_request_irq;

	disable_irq(reverse_data->irq);

	ret = device_create_file(reverse_data->sdev.dev, &dev_attr_continues);

/*	INIT_WORK(&reverse_data->work, gpio_switch_work); */
	INIT_DELAYED_WORK(&reverse_data->detect_delayed_work, reverse_detection_work);
	INIT_DELAYED_WORK(&camera_detect_dwork, camera_detect_workqueue);

	/* Perform initial detection */
/*	gpio_switch_work(&reverse_data->work); */
	reverse_detection_work(&reverse_data->detect_delayed_work.work);

	enable_irq(reverse_data->irq);

	return 0;

err_request_irq:
err_detect_irq_num_failed:
err_set_gpio_input:
	gpio_free(reverse_data->gpio);
err_request_gpio:
	input_unregister_device(reverse_data->idev);
err_input_reg:
	input_free_device(reverse_data->idev);
err_input_dev_register:
	switch_dev_unregister(&reverse_data->sdev);
err_switch_dev_register:
	kfree(reverse_data);

	return ret;
}

static int __devexit switch_reverse_remove(struct platform_device *pdev)
{
	struct reverse_reverse_data *reverse_data = platform_get_drvdata(pdev);

	cancel_delayed_work_sync(&reverse_data->detect_delayed_work);
	gpio_free(reverse_data->gpio);
	input_unregister_device(reverse_data->idev);
	switch_dev_unregister(&reverse_data->sdev);
	kfree(reverse_data);

	return 0;
}

static struct platform_driver switch_reverse_driver = {
	.probe		= switch_reverse_probe,
	.remove		= __devexit_p(switch_reverse_remove),
	.driver		= {
		.name	= "switch-reverse",
		.owner	= THIS_MODULE,
	},
};

static int __init switch_reverse_init(void)
{
	return platform_driver_register(&switch_reverse_driver);
}

static void __exit switch_reverse_exit(void)
{
	platform_driver_unregister(&switch_reverse_driver);
}

module_init(switch_reverse_init);
module_exit(switch_reverse_exit);

MODULE_DESCRIPTION("Switch Reverse driver");
MODULE_LICENSE("GPL");
