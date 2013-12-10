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

#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/delay.h>

#include "pic.h"
#include "A.h"
#include "B.h"

void show_pic(void);
void shutdown_pic(void);
void pic_update(unsigned char *pic, unsigned int pos_x, unsigned int pos_y, unsigned int image_w, unsigned int image_h);

static int __init show_pic_init(void)
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

	pic_update(data, 0, 0, w, h);
	pic_update(data_a, 170, 340, w_a, h_a);
	pic_update(data_b, 500, 340, w_b, h_b);

	show_pic();

	return 0;
}

static void __exit show_pic_exit(void)
{
	shutdown_pic();
}

module_init(show_pic_init);
module_exit(show_pic_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Show boot picture Driver");
