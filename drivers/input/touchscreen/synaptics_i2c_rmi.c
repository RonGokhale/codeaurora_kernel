/* drivers/input/keyboard/synaptics_i2c_rmi.c
 *
 * Copyright (C) 2007 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/hrtimer.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <asm/io.h>

struct synaptics_ts_pdev {
	unsigned short force[3];
	unsigned short *forces[2];
	struct i2c_client_address_data addr_data;
	int irq;
};

struct synaptics_ts_data {
	uint16_t addr;
	struct i2c_client *client;
	struct input_dev *input_dev;
	int use_irq;
	struct hrtimer timer;
	struct work_struct  work;
	uint16_t max_y;
};

static void synaptics_ts_work_func(struct work_struct *work)
{
	int i;
	int ret;
	struct i2c_msg msg[2];
	uint8_t start_reg;
	uint8_t buf[15];
	struct synaptics_ts_data *ts = container_of(work, struct synaptics_ts_data, work);

	msg[0].addr = ts->client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &start_reg;
	start_reg = 0x00;
	msg[1].addr = ts->client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = sizeof(buf);
	msg[1].buf = buf;

	/* printk("synaptics_ts_work_func\n"); */
	for (i = 0; i < (ts->use_irq ? 1 : 10); i++) {
		ret = i2c_transfer(ts->client->adapter, msg, 2);
		if (ret < 0) {
			printk(KERN_ERR "synaptics_ts_work_func: i2c_transfer failed\n");
		} else {
			/* printk("synaptics_ts_work_func: %x %x %x %x %x %x %x %x %x\n", */
			/*       buf[0], buf[1], buf[2], buf[3], */
			/*       buf[4], buf[5], buf[6], buf[7], buf[8]); */
			if ((buf[14] & 1) == 0) {
				/* printk("read %d coordinates\n", i); */
				break;
			} else {
				int x = buf[3] | (uint16_t)(buf[2] & 0x1f) << 8;
				int y = buf[5] | (uint16_t)(buf[4] & 0x1f) << 8;
				int z = buf[1];
				int w = buf[0] >> 4;
				int finger = buf[0] & 7;

				int x2 = buf[3+6] | (uint16_t)(buf[2+6] & 0x1f) << 8;
				int y2 = buf[5+6] | (uint16_t)(buf[4+6] & 0x1f) << 8;
				/* int z2 = buf[1+6]; */
				/* int w2 = buf[0+6] >> 4; */
				/* int finger2 = buf[0+6] & 7; */

				/* int dx = (int8_t)buf[12]; */
				/* int dy = (int8_t)buf[13]; */
				int finger2_pressed;

				/* printk("x %4d, y %4d, z %3d, w %2d, F %d, 2nd: x %4d, y %4d, z %3d, w %2d, F %d, dx %4d, dy %4d\n", */
				/*	x, y, z, w, finger, */
				/*	x2, y2, z2, w2, finger2, */
				/*	dx, dy); */
				if (z) {
					input_report_abs(ts->input_dev, ABS_X, x);
					input_report_abs(ts->input_dev, ABS_Y, ts->max_y - y);
				}
				input_report_abs(ts->input_dev, ABS_PRESSURE, z);
				input_report_abs(ts->input_dev, ABS_TOOL_WIDTH, w);
				input_report_key(ts->input_dev, BTN_TOUCH, finger);
				finger2_pressed = finger > 1 && finger != 7;
				input_report_key(ts->input_dev, BTN_2, finger2_pressed);
				if (finger2_pressed) {
					input_report_abs(ts->input_dev, ABS_HAT0X, x2);
					input_report_abs(ts->input_dev, ABS_HAT0Y, ts->max_y - y2);
				}
				input_sync(ts->input_dev);
			}
		}
	}
	if (ts->use_irq)
		enable_irq(ts->client->irq);
}

static enum hrtimer_restart synaptics_ts_timer_func(struct hrtimer *timer)
{
	struct synaptics_ts_data *ts = container_of(timer, struct synaptics_ts_data, timer);
	/* printk("synaptics_ts_timer_func\n"); */

	schedule_work(&ts->work);

	hrtimer_start(&ts->timer, ktime_set(0, 12500000), HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}

static irqreturn_t synaptics_ts_irq_handler(int irq, void *dev_id)
{
	struct synaptics_ts_data *ts = dev_id;

	/* printk("synaptics_ts_irq_handler\n"); */
	disable_irq(ts->client->irq);
	schedule_work(&ts->work);
	return IRQ_HANDLED;
}

static int synaptics_ts_probe(struct i2c_client *client)
{
	struct synaptics_ts_data *ts;
	uint8_t buf0[4];
	uint8_t buf1[8];
	struct i2c_msg msg[2];
	int ret = 0;
	uint16_t max_x, max_y;
	/* todo: Move to platform data */
	int inactive_area_left = ((4674 - 4334) / 2 + 200) * 0x10000 / 4334;
	int inactive_area_right = ((4674 - 4334) / 2 + 200) * 0x10000 / 4334;
	int inactive_area_top = ((6946 - 6696) / 2) * 0x10000 / 6696;
	int inactive_area_bottom = ((6946 - 6696) / 2) * 0x10000 / 6696;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_ERR "synaptics_ts_probe: need I2C_FUNC_I2C\n");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}

	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL) {
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}
	INIT_WORK(&ts->work, synaptics_ts_work_func);
	ts->client = client;
	i2c_set_clientdata(client, ts);

	ret = i2c_smbus_write_byte_data(ts->client, 0xf4, 0x01); /* device command = reset */
	if (ret < 0) {
		printk(KERN_ERR "i2c_smbus_write_byte_data failed\n");
		/* fail? */
	}
	{
		int retry = 10;
		while (retry-- > 0) {
			ret = i2c_smbus_read_byte_data(ts->client, 0xe4);
			if (ret >= 0)
				break;
			msleep(100);
		}
	}
	if (ret < 0) {
		printk(KERN_ERR "i2c_smbus_read_byte_data failed\n");
		goto err_detect_failed;
	}
	printk(KERN_INFO "synaptics_ts_probe: Product Major Version %x\n", ret);
	ret = i2c_smbus_read_byte_data(ts->client, 0xe5);
	if (ret < 0) {
		printk(KERN_ERR "i2c_smbus_read_byte_data failed\n");
		goto err_detect_failed;
	}
	printk(KERN_INFO "synaptics_ts_probe: Product Minor Version %x\n", ret);

	ret = i2c_smbus_read_byte_data(ts->client, 0xe3);
	if (ret < 0) {
		printk(KERN_ERR "i2c_smbus_read_byte_data failed\n");
		goto err_detect_failed;
	}
	printk(KERN_INFO "synaptics_ts_probe: product property %x\n", ret);

	ret = i2c_smbus_read_byte_data(ts->client, 0xf0);
	if (ret < 0) {
		printk(KERN_ERR "i2c_smbus_read_byte_data failed\n");
		goto err_detect_failed;
	}
	printk(KERN_INFO "synaptics_ts_probe: device control %x\n", ret);

	ret = i2c_smbus_read_byte_data(ts->client, 0xf1);
	if (ret < 0) {
		printk(KERN_ERR "i2c_smbus_read_byte_data failed\n");
		goto err_detect_failed;
	}
	printk(KERN_INFO "synaptics_ts_probe: interrupt enable %x\n", ret);

	ret = i2c_smbus_write_byte_data(ts->client, 0xf1, 0); /* disable interrupt */
	if (ret < 0) {
		printk(KERN_ERR "i2c_smbus_write_byte_data failed\n");
		goto err_detect_failed;
	}

	msg[0].addr = ts->client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = buf0;
	buf0[0] = 0xe0;
	msg[1].addr = ts->client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 8;
	msg[1].buf = buf1;
	ret = i2c_transfer(ts->client->adapter, msg, 2);
	if (ret < 0) {
		printk(KERN_ERR "i2c_transfer failed\n");
		goto err_detect_failed;
	}
	printk(KERN_INFO "synaptics_ts_probe: 0xe0: %x %x %x %x %x %x %x %x\n",
	       buf1[0], buf1[1], buf1[2], buf1[3],
	       buf1[4], buf1[5], buf1[6], buf1[7]);

	ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x10); /* page select = 0x10 */
	if (ret < 0) {
		printk(KERN_ERR "i2c_smbus_write_byte_data failed for page select\n");
		goto err_detect_failed;
	}
	ret = i2c_smbus_read_word_data(ts->client, 0x04);
	if (ret < 0) {
		printk(KERN_ERR "i2c_smbus_read_word_data failed\n");
		goto err_detect_failed;
	}
	max_x = (ret >> 8 & 0xff) | ((ret & 0x1f) << 8);
	ret = i2c_smbus_read_word_data(ts->client, 0x06);
	if (ret < 0) {
		printk(KERN_ERR "i2c_smbus_read_word_data failed\n");
		goto err_detect_failed;
	}
	ts->max_y = max_y = (ret >> 8 & 0xff) | ((ret & 0x1f) << 8);

	ret = i2c_smbus_write_byte_data(ts->client, 0x41, 0x04); /* Set "No Clip Z" */
	if (ret < 0) {
		printk(KERN_ERR "i2c_smbus_write_byte_data failed for No Clip Z\n");
		goto err_detect_failed;
	}

	ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x04); /* page select = 0x04 */
	if (ret < 0) {
		printk(KERN_ERR "i2c_smbus_write_byte_data failed for page select\n");
		goto err_detect_failed;
	}

	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		ret = -ENOMEM;
		printk(KERN_ERR "synaptics_ts_probe: Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}
	ts->input_dev->name = "synaptics-rmi-touchscreen";
	set_bit(EV_SYN, ts->input_dev->evbit);
	set_bit(EV_KEY, ts->input_dev->evbit);
	set_bit(BTN_TOUCH, ts->input_dev->keybit);
	set_bit(BTN_2, ts->input_dev->keybit);
	set_bit(EV_ABS, ts->input_dev->evbit);
	inactive_area_left = inactive_area_left * max_x / 0x10000;
	inactive_area_right = inactive_area_right * max_x / 0x10000;
	inactive_area_top = inactive_area_top * max_y / 0x10000;
	inactive_area_bottom = inactive_area_bottom * max_y / 0x10000;
	printk(KERN_INFO "synaptics_ts_probe: max_x %d, max_y %d\n", max_x, max_y);
	printk(KERN_INFO "synaptics_ts_probe: inactive_x %d %d, inactive_y %d %d\n",
	       inactive_area_left, inactive_area_right,
	       inactive_area_top, inactive_area_bottom);
	input_set_abs_params(ts->input_dev, ABS_X, -inactive_area_left, max_x + inactive_area_right, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_Y, -inactive_area_top, max_y + inactive_area_bottom, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_TOOL_WIDTH, 0, 15, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_HAT0X, -inactive_area_left, max_x + inactive_area_right, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_HAT0Y, -inactive_area_top, max_y + inactive_area_bottom, 0, 0);
	/* ts->input_dev->name = ts->keypad_info->name; */
	ret = input_register_device(ts->input_dev);
	if (ret) {
		printk(KERN_ERR "synaptics_ts_probe: Unable to register %s input device\n", ts->input_dev->name);
		goto err_input_register_device_failed;
	}
	if (client->irq) {
		ret = request_irq(client->irq, synaptics_ts_irq_handler, 0, client->name, ts);
		if (ret == 0) {
			ret = i2c_smbus_write_byte_data(ts->client, 0xf1, 0x01); /* enable abs int */
			if (ret)
				free_irq(client->irq, ts);
		}
		if (ret == 0)
			ts->use_irq = 1;
		else
			dev_err(&client->dev, "request_irq failed\n");
	}
	if (!ts->use_irq) {
		hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ts->timer.function = synaptics_ts_timer_func;
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	}

	printk(KERN_INFO "synaptics_ts_probe: Start touchscreen %s in %s mode\n", ts->input_dev->name, ts->use_irq ? "interrupt" : "polling");

	return 0;

err_input_register_device_failed:
	input_free_device(ts->input_dev);

err_input_dev_alloc_failed:
err_detect_failed:
	kfree(ts);
err_alloc_data_failed:
err_check_functionality_failed:
	return ret;
}

static int synaptics_ts_remove(struct i2c_client *client)
{
	struct synaptics_ts_data *ts = i2c_get_clientdata(client);
	if (ts->use_irq)
		free_irq(client->irq, ts);
	else
		hrtimer_cancel(&ts->timer);
	input_unregister_device(ts->input_dev);
	kfree(ts);
	return 0;
}

static int synaptics_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int ret;
	ret = i2c_smbus_write_byte_data(client, 0xf0, 0x86); /* deep sleep */
	if (ret < 0)
		printk(KERN_ERR "synaptics_ts_suspend: i2c_smbus_write_byte_data failed\n");
	return 0;
}

static int synaptics_ts_resume(struct i2c_client *client)
{
	int ret;
	ret = i2c_smbus_write_byte_data(client, 0xf0, 0x81); /* normal operation, 80 reports per second */
	if (ret < 0)
		printk(KERN_ERR "synaptics_ts_resume: i2c_smbus_write_byte_data failed\n");
	return 0;
}

static struct i2c_driver synaptics_ts_driver = {
	.probe		= synaptics_ts_probe,
	.remove		= synaptics_ts_remove,
	.suspend	= synaptics_ts_suspend,
	.resume		= synaptics_ts_resume,
	.driver = {
		.name	= "synaptics-rmi-ts",
	},
};

static int __devinit synaptics_ts_init(void)
{
	return i2c_add_driver(&synaptics_ts_driver);
}

static void __exit synaptics_ts_exit(void)
{
	i2c_del_driver(&synaptics_ts_driver);
}

module_init(synaptics_ts_init);
module_exit(synaptics_ts_exit);

MODULE_DESCRIPTION("Synaptics Touchscreen Driver");
MODULE_LICENSE("GPL");
