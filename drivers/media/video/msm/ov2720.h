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

#ifndef OV2720_H
#define OV2720_H
#include <linux/types.h>
#include <mach/board.h>
extern struct ov2720_reg ov2720_regs;

struct ov2720_i2c_reg_conf {
	unsigned short waddr;
	unsigned short wdata;
};

struct ov2720_i2c_conf_array {
	struct ov2720_i2c_reg_conf *conf;
	unsigned short size;
};

enum ov2720_test_mode_t {
	TEST_OFF,
	TEST_1,
	TEST_2,
	TEST_3
};

enum ov2720_resolution_t {
	QTR_2D_SIZE,
	FULL_2D_SIZE,
	QTR_3D_SIZE,
	FULL_3D_SIZE,
	INVALID_SIZE
};
enum ov2720_setting {
	RES_PREVIEW,
	RES_CAPTURE,
	RES_3D_PREVIEW,
	RES_3D_CAPTURE
};
enum ov2720_cam_mode_t {
	MODE_2D_RIGHT,
	MODE_2D_LEFT,
	MODE_3D,
	MODE_INVALID
};
enum ov2720_reg_update {
	/* Sensor egisters that need to be updated during initialization */
	REG_INIT,
	/* Sensor egisters that needs periodic I2C writes */
	UPDATE_PERIODIC,
	/* All the sensor Registers will be updated */
	UPDATE_ALL,
	/* Not valid update */
	UPDATE_INVALID
};

enum ov2720_reg_mode {
	OV2720_LINE_LENGTH_PCK_HI = 12,
	OV2720_LINE_LENGTH_PCK_LO,
	OV2720_FRAME_LENGTH_LINES_HI,
	OV2720_FRAME_LENGTH_LINES_LO,
};

struct ov2720_reg {
	const struct ov2720_i2c_reg_conf *rec_settings;
	const unsigned short rec_size;
	const struct ov2720_i2c_conf_array *conf_array;
};
#endif /* OV2720_H */
