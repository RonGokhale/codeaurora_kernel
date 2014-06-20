/*
 * max77802.h - Driver for the Maxim 77802
 *
 * Copyright (c) 2013-2014 Google, Inc
 *
 *  Copyright (C) 2012 Samsung Electrnoics
 *  Chiwoong Byun <woong.byun@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * This driver is based on max8997.h
 *
 * MAX77802 has PMIC, RTC devices.
 * The devices share the same I2C bus and included in
 * this mfd driver.
 */

#ifndef __LINUX_MFD_MAX77802_H
#define __LINUX_MFD_MAX77802_H

#include <linux/regulator/consumer.h>

/* MAX77802 regulator IDs - LDOS must come before BUCKs */
enum max77802_regulators {
	MAX77802_LDO1 = 0,
	MAX77802_LDO2,
	MAX77802_LDO3,
	MAX77802_LDO4,
	MAX77802_LDO5,
	MAX77802_LDO6,
	MAX77802_LDO7,
	MAX77802_LDO8,
	MAX77802_LDO9,
	MAX77802_LDO10,
	MAX77802_LDO11,
	MAX77802_LDO12,
	MAX77802_LDO13,
	MAX77802_LDO14,
	MAX77802_LDO15,
	MAX77802_LDO16,
	MAX77802_LDO17,
	MAX77802_LDO18,
	MAX77802_LDO19,
	MAX77802_LDO20,
	MAX77802_LDO21,
	MAX77802_LDO22,
	MAX77802_LDO23,
	MAX77802_LDO24,
	MAX77802_LDO25,
	MAX77802_LDO26,
	MAX77802_LDO27,
	MAX77802_LDO28,
	MAX77802_LDO29,
	MAX77802_LDO30,
	MAX77802_LDO31,
	MAX77802_LDO32,
	MAX77802_LDO33,
	MAX77802_LDO34,
	MAX77802_LDO35,
	MAX77802_BUCK1,
	MAX77802_BUCK2,
	MAX77802_BUCK3,
	MAX77802_BUCK4,
	MAX77802_BUCK5,
	MAX77802_BUCK6,
	MAX77802_BUCK7,
	MAX77802_BUCK8,
	MAX77802_BUCK9,
	MAX77802_BUCK10,

	MAX77802_REG_MAX,
};

struct max77802_regulator_data {
	int id;
	int opmode;
	struct regulator_init_data *initdata;
	struct device_node *of_node;
};

enum max77802_opmode {
	MAX77802_OPMODE_OFF,
	MAX77802_OPMODE_STANDBY,
	MAX77802_OPMODE_LP,
	MAX77802_OPMODE_NORMAL,
};

struct max77802_opmode_data {
	int id;
	int mode;
};

struct max77802_platform_data {
	/* IRQ */
	int irq_gpio;
	int ono;
	int wakeup;

	/* ---- PMIC ---- */
	struct max77802_regulator_data *regulators;
	int num_regulators;

	struct max77802_opmode_data *opmode_data;

	/*
	 * GPIO-DVS feature is not fully enabled with the current version of
	 * MAX77802 driver, but the driver does support using a DVS index other
	 * than the default of 0.
	 */
	struct gpio_desc *buck_gpio_dvs[3]; /* GPIO of [0]DVS1, [1]DVS2, [2]DVS3 */
	int buck_default_idx; /* Default value of DVS1, 2, 3 */

	struct gpio_desc *buck_gpio_selb[5]; /* 77802: 1, 2, 3, 4, 6 */
};

#endif /* __LINUX_MFD_MAX77802_H */
