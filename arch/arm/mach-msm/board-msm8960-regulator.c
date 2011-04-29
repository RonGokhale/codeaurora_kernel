/*
 * Copyright (c) 2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/regulator/pm8921-regulator.h>

#include "board-msm8960.h"

#define VREG_CONSUMERS(_id) \
	static struct regulator_consumer_supply \
	vreg_consumers_##_id[] __devinitdata

/*
 * Consumer specific regulator names:
 *			 regulator name		consumer dev_name
 */
VREG_CONSUMERS(L1) = {
	REGULATOR_SUPPLY("8921_l1",		NULL),
};
VREG_CONSUMERS(L2) = {
	REGULATOR_SUPPLY("8921_l2",		NULL),
};
VREG_CONSUMERS(L3) = {
	REGULATOR_SUPPLY("8921_l3",		NULL),
	REGULATOR_SUPPLY("HSUSB_3p3",		"msm_otg"),
};
VREG_CONSUMERS(L4) = {
	REGULATOR_SUPPLY("8921_l4",		NULL),
	REGULATOR_SUPPLY("HSUSB_1p8",		"msm_otg"),
};
VREG_CONSUMERS(L5) = {
	REGULATOR_SUPPLY("8921_l5",		NULL),
	REGULATOR_SUPPLY("sdc_vdd",		"msm_sdcc.1"),
};
VREG_CONSUMERS(L6) = {
	REGULATOR_SUPPLY("8921_l6",		NULL),
	REGULATOR_SUPPLY("sdc_vdd",		"msm_sdcc.3"),
};
VREG_CONSUMERS(L7) = {
	REGULATOR_SUPPLY("8921_l7",		NULL),
	REGULATOR_SUPPLY("sdc_vddp",		"msm_sdcc.3"),
};
VREG_CONSUMERS(L8) = {
	REGULATOR_SUPPLY("8921_l8",		NULL),
};
VREG_CONSUMERS(L9) = {
	REGULATOR_SUPPLY("8921_l9",		NULL),
};
VREG_CONSUMERS(L10) = {
	REGULATOR_SUPPLY("8921_l10",		NULL),
};
VREG_CONSUMERS(L11) = {
	REGULATOR_SUPPLY("8921_l11",		NULL),
};
VREG_CONSUMERS(L12) = {
	REGULATOR_SUPPLY("8921_l12",		NULL),
};
VREG_CONSUMERS(L14) = {
	REGULATOR_SUPPLY("8921_l14",		NULL),
};
VREG_CONSUMERS(L15) = {
	REGULATOR_SUPPLY("8921_l15",		NULL),
};
VREG_CONSUMERS(L16) = {
	REGULATOR_SUPPLY("8921_l16",		NULL),
};
VREG_CONSUMERS(L17) = {
	REGULATOR_SUPPLY("8921_l17",		NULL),
};
VREG_CONSUMERS(L18) = {
	REGULATOR_SUPPLY("8921_l18",		NULL),
};
VREG_CONSUMERS(L21) = {
	REGULATOR_SUPPLY("8921_l21",		NULL),
};
VREG_CONSUMERS(L22) = {
	REGULATOR_SUPPLY("8921_l22",		NULL),
};
VREG_CONSUMERS(L23) = {
	REGULATOR_SUPPLY("8921_l23",		NULL),
};
VREG_CONSUMERS(L24) = {
	REGULATOR_SUPPLY("8921_l24",		NULL),
};
VREG_CONSUMERS(L25) = {
	REGULATOR_SUPPLY("8921_l25",		NULL),
};
VREG_CONSUMERS(L26) = {
	REGULATOR_SUPPLY("8921_l26",		NULL),
};
VREG_CONSUMERS(L27) = {
	REGULATOR_SUPPLY("8921_l27",		NULL),
};
VREG_CONSUMERS(L28) = {
	REGULATOR_SUPPLY("8921_l28",		NULL),
};
VREG_CONSUMERS(L29) = {
	REGULATOR_SUPPLY("8921_l29",		NULL),
};
VREG_CONSUMERS(S1) = {
	REGULATOR_SUPPLY("8921_s1",		NULL),
};
VREG_CONSUMERS(S2) = {
	REGULATOR_SUPPLY("8921_s2",		NULL),
};
VREG_CONSUMERS(S3) = {
	REGULATOR_SUPPLY("8921_s3",		NULL),
	REGULATOR_SUPPLY("HSUSB_VDDCX",		"msm_otg"),
};
VREG_CONSUMERS(S4) = {
	REGULATOR_SUPPLY("8921_s4",		NULL),
	REGULATOR_SUPPLY("sdc_vccq",		"msm_sdcc.1"),
};
VREG_CONSUMERS(S5) = {
	REGULATOR_SUPPLY("8921_s5",		NULL),
};
VREG_CONSUMERS(S6) = {
	REGULATOR_SUPPLY("8921_s6",		NULL),
};
VREG_CONSUMERS(S7) = {
	REGULATOR_SUPPLY("8921_s7",		NULL),
};
VREG_CONSUMERS(S8) = {
	REGULATOR_SUPPLY("8921_s8",		NULL),
};
VREG_CONSUMERS(LVS1) = {
	REGULATOR_SUPPLY("8921_lvs1",		NULL),
	REGULATOR_SUPPLY("sdc_vdd",		"msm_sdcc.4"),
};
VREG_CONSUMERS(LVS2) = {
	REGULATOR_SUPPLY("8921_lvs2",		NULL),
};
VREG_CONSUMERS(LVS3) = {
	REGULATOR_SUPPLY("8921_lvs3",		NULL),
};
VREG_CONSUMERS(LVS4) = {
	REGULATOR_SUPPLY("8921_lvs4",		NULL),
};
VREG_CONSUMERS(LVS5) = {
	REGULATOR_SUPPLY("8921_lvs5",		NULL),
};
VREG_CONSUMERS(LVS6) = {
	REGULATOR_SUPPLY("8921_lvs6",		NULL),
};
VREG_CONSUMERS(LVS7) = {
	REGULATOR_SUPPLY("8921_lvs7",		NULL),
};
VREG_CONSUMERS(USB_OTG) = {
	REGULATOR_SUPPLY("8921_usb_otg",	NULL),
	REGULATOR_SUPPLY("vbus_otg",		"msm_otg"),
};
VREG_CONSUMERS(HDMI_MVS) = {
	REGULATOR_SUPPLY("8921_hdmi_mvs",	NULL),
};
VREG_CONSUMERS(NCP) = {
	REGULATOR_SUPPLY("8921_ncp",		NULL),
};

#define PM8921_VREG_INIT(_id, _min_uV, _max_uV, _modes, _ops, _apply_uV, \
			 _pull_down, _always_on, _supply_regulator) \
	{ \
		.init_data = { \
			.constraints = { \
				.valid_modes_mask	= _modes, \
				.valid_ops_mask		= _ops, \
				.min_uV			= _min_uV, \
				.max_uV			= _max_uV, \
				.input_uV		= _max_uV, \
				.apply_uV		= _apply_uV, \
				.always_on		= _always_on, \
			}, \
			.num_consumer_supplies	= \
					ARRAY_SIZE(vreg_consumers_##_id), \
			.consumer_supplies	= vreg_consumers_##_id, \
			.supply_regulator	= _supply_regulator, \
		}, \
		.id			= PM8921_VREG_ID_##_id, \
		.pull_down_enable	= _pull_down, \
	}

#define PM8921_VREG_INIT_LDO(_id, _always_on, _pull_down, _min_uV, _max_uV, \
		_supply_regulator) \
	PM8921_VREG_INIT(_id, _min_uV, _max_uV, REGULATOR_MODE_NORMAL \
		| REGULATOR_MODE_IDLE, REGULATOR_CHANGE_VOLTAGE | \
		REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_MODE | \
		REGULATOR_CHANGE_DRMS, 0, _pull_down, _always_on, \
		_supply_regulator)

#define PM8921_VREG_INIT_NLDO1200(_id, _always_on, _pull_down, _min_uV, \
		_max_uV, _supply_regulator) \
	PM8921_VREG_INIT(_id, _min_uV, _max_uV, REGULATOR_MODE_NORMAL \
		| REGULATOR_MODE_IDLE, REGULATOR_CHANGE_VOLTAGE | \
		REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_MODE | \
		REGULATOR_CHANGE_DRMS, 0, _pull_down, _always_on, \
		_supply_regulator)

#define PM8921_VREG_INIT_SMPS(_id, _always_on, _pull_down, _min_uV, _max_uV, \
		_supply_regulator) \
	PM8921_VREG_INIT(_id, _min_uV, _max_uV, REGULATOR_MODE_NORMAL \
		| REGULATOR_MODE_IDLE, REGULATOR_CHANGE_VOLTAGE | \
		REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_MODE | \
		REGULATOR_CHANGE_DRMS, 0, _pull_down, _always_on, \
		_supply_regulator)

#define PM8921_VREG_INIT_FTSMPS(_id, _always_on, _pull_down, _min_uV, _max_uV, \
		_supply_regulator) \
	PM8921_VREG_INIT(_id, _min_uV, _max_uV, REGULATOR_MODE_NORMAL, \
		REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS \
		| REGULATOR_CHANGE_MODE, 0, _pull_down, _always_on, \
		_supply_regulator)

#define PM8921_VREG_INIT_VS(_id, _always_on, _pull_down, _supply_regulator) \
	PM8921_VREG_INIT(_id, 0, 0, 0, REGULATOR_CHANGE_STATUS, 0, _pull_down, \
		_always_on, _supply_regulator)

#define PM8921_VREG_INIT_VS300(_id, _always_on, _pull_down, _supply_regulator) \
	PM8921_VREG_INIT(_id, 0, 0, 0, REGULATOR_CHANGE_STATUS, 0, _pull_down, \
		_always_on, _supply_regulator)

#define PM8921_VREG_INIT_NCP(_id, _always_on, _min_uV, _max_uV, \
		_supply_regulator) \
	PM8921_VREG_INIT(_id, _min_uV, _max_uV, 0, REGULATOR_CHANGE_VOLTAGE | \
		REGULATOR_CHANGE_STATUS, 0, 0, _always_on, _supply_regulator)

/* Pin control initialization */
#define PM8921_PC_INIT(_id, _always_on, _pin_fn, _pin_ctrl, _supply_regulator) \
	{ \
		.init_data = { \
			.constraints = { \
				.valid_ops_mask	= REGULATOR_CHANGE_STATUS, \
				.always_on	= _always_on, \
			}, \
			.num_consumer_supplies	= \
					ARRAY_SIZE(vreg_consumers_##_id), \
			.consumer_supplies	= vreg_consumers_##_id, \
			.supply_regulator  = _supply_regulator, \
		}, \
		.id	  = PM8921_VREG_ID_##_id##_PC, \
		.pin_fn	  = PM8921_VREG_PIN_FN_##_pin_fn, \
		.pin_ctrl = _pin_ctrl, \
	}

/* Regulator constraints */
struct pm8921_regulator_platform_data
msm_pm8921_regulator_pdata[] __devinitdata = {
	/*		      ID  always_on pd min_uV   max_uV   supply */
	PM8921_VREG_INIT_SMPS(S1,	 0, 1, 1225000, 1225000, NULL),
	PM8921_VREG_INIT_SMPS(S2,	 0, 1, 1300000, 1300000, NULL),
	PM8921_VREG_INIT_SMPS(S3,	 0, 1, 1050000, 1150000, NULL),
	PM8921_VREG_INIT_SMPS(S4,	 0, 1, 1800000, 1800000, NULL),
	PM8921_VREG_INIT_FTSMPS(S5,	 0, 1, 1050000, 1150000, NULL),
	PM8921_VREG_INIT_FTSMPS(S6,	 0, 1, 1050000, 1150000, NULL),
	PM8921_VREG_INIT_SMPS(S7,	 0, 1, 1150000, 1150000, NULL),
	PM8921_VREG_INIT_SMPS(S8,	 0, 1, 2200000, 2200000, NULL),

	PM8921_VREG_INIT_LDO(L1,	 0, 1, 1050000, 1050000, "8921_s4"),
	PM8921_VREG_INIT_LDO(L2,	 0, 1, 1200000, 1200000, "8921_s4"),
	PM8921_VREG_INIT_LDO(L3,	 0, 1, 3075000, 3075000, NULL),
	PM8921_VREG_INIT_LDO(L4,	 0, 1, 1800000, 1800000, NULL),
	PM8921_VREG_INIT_LDO(L5,	 0, 1, 2950000, 2950000, NULL),
	PM8921_VREG_INIT_LDO(L6,	 0, 1, 2950000, 2950000, NULL),
	PM8921_VREG_INIT_LDO(L7,	 0, 1, 2950000, 2950000, NULL),
	PM8921_VREG_INIT_LDO(L8,	 0, 1, 3000000, 3000000, NULL),
	PM8921_VREG_INIT_LDO(L9,	 0, 1, 2850000, 2850000, NULL),
	PM8921_VREG_INIT_LDO(L10,	 0, 1, 2900000, 2900000, NULL),
	PM8921_VREG_INIT_LDO(L11,	 0, 1, 2850000, 2850000, NULL),
	PM8921_VREG_INIT_LDO(L12,	 0, 1, 1200000, 1200000, "8921_s4"),
	PM8921_VREG_INIT_LDO(L14,	 0, 1, 1800000, 1800000, NULL),
	PM8921_VREG_INIT_LDO(L15,	 0, 1, 1800000, 2950000, NULL),
	PM8921_VREG_INIT_LDO(L16,	 0, 1, 2800000, 2800000, NULL),
	PM8921_VREG_INIT_LDO(L17,	 0, 1, 1800000, 2950000, NULL),
	PM8921_VREG_INIT_LDO(L18,	 0, 1, 1300000, 1300000, "8921_s4"),
	PM8921_VREG_INIT_LDO(L21,	 0, 1, 1900000, 1900000, "8921_s8"),
	PM8921_VREG_INIT_LDO(L22,	 0, 1, 2750000, 2750000, NULL),
	PM8921_VREG_INIT_LDO(L23,	 0, 1, 1800000, 1800000, "8921_s8"),
	PM8921_VREG_INIT_NLDO1200(L24,	 0, 1, 1050000, 1150000, "8921_s1"),
	PM8921_VREG_INIT_NLDO1200(L25,	 0, 1, 1225000, 1225000, "8921_s1"),
	PM8921_VREG_INIT_NLDO1200(L26,	 0, 1, 1050000, 1050000, "8921_s7"),
	PM8921_VREG_INIT_NLDO1200(L27,	 0, 1, 1050000, 1050000, "8921_s7"),
	PM8921_VREG_INIT_NLDO1200(L28,	 0, 1, 1050000, 1050000, "8921_s7"),
	PM8921_VREG_INIT_LDO(L29,	 0, 1, 2050000, 2100000, "8921_s8"),

	PM8921_VREG_INIT_VS(LVS1,	 0, 1,			 "8921_s4"),
	PM8921_VREG_INIT_VS300(LVS2,	 0, 1,			 "8921_s1"),
	PM8921_VREG_INIT_VS(LVS3,	 0, 1,			 "8921_s4"),
	PM8921_VREG_INIT_VS(LVS4,	 0, 1,			 "8921_s4"),
	PM8921_VREG_INIT_VS(LVS5,	 0, 1,			 "8921_s4"),
	PM8921_VREG_INIT_VS(LVS6,	 0, 1,			 "8921_s4"),
	PM8921_VREG_INIT_VS(LVS7,	 0, 1,			 "8921_s4"),

	PM8921_VREG_INIT_VS300(USB_OTG,  0, 1,			 NULL),
	PM8921_VREG_INIT_VS300(HDMI_MVS, 0, 1,			 NULL),

	PM8921_VREG_INIT_NCP(NCP,	 0,    1800000, 1800000, "8921_l6"),
};

int msm_pm8921_regulator_pdata_len __devinitdata =
	ARRAY_SIZE(msm_pm8921_regulator_pdata);
