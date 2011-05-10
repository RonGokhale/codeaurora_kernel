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
/*
 * Qualcomm PMIC 8921 ADC driver header file
 *
 */

#ifndef __MFD_PM8921_ADC_H
#define __MFD_PM8921_ADC_H

#include <linux/kernel.h>
#include <linux/list.h>

enum pm8921_adc_channels {
	CHANNEL_VCOIN = 0,
	CHANNEL_VBAT,
	CHANNEL_DCIN,
	CHANNEL_ICHG,
	CHANNEL_VPH_PWR,
	CHANNEL_IBAT,
	CHANNEL_MPP_1,
	CHANNEL_MPP_2,
	CHANNEL_BATT_THERM,
	CHANNEL_BATT_ID,
	CHANNEL_USBIN,
	CHANNEL_DIE_TEMP,
	CHANNEL_625MV,
	CHANNEL_125V,
	CHANNEL_CHG_TEMP,
	CHANNEL_MUXOFF,
	CHANNEL_NONE,
};

enum pm8921_adc_mpp_channels {
	ADC_MPP_ATEST_8 = 0,
	ADC_MPP_USB_SNS_DIV20,
	ADC_MPP_DCIN_SNS_DIV20,
	ADC_MPP_AMUX3,
	ADC_MPP_AMUX4,
	ADC_MPP_AMUX5,
	ADC_MPP_AMUX6,
	ADC_MPP_AMUX7,
	ADC_MPP_AMUX8,
	ADC_MPP_ATEST_1,
	ADC_MPP_ATEST_2,
	ADC_MPP_ATEST_3,
	ADC_MPP_ATEST_4,
	ADC_MPP_ATEST_5,
	ADC_MPP_ATEST_6,
	ADC_MPP_ATEST_7,
	ADC_MPP_CHANNEL_NONE,
};

#define PM8921_ADC_PMIC_0	0x0

#define PM8921_CHANNEL_ADC_625_MV	625

#define PM8921_AMUX_MPP_3	0x3
#define PM8921_AMUX_MPP_4	0x4
#define PM8921_AMUX_MPP_5	0x5
#define PM8921_AMUX_MPP_6	0x6

#define PM8921_ADC_DEV_NAME	"pm8921-adc"

enum pm8921_adc_decimation_type {
	ADC_DECIMATION_TYPE1 = 0,
	ADC_DECIMATION_TYPE2,
	ADC_DECIMATION_NONE,
};

enum pm8921_adc_calib_type {
	ADC_CALIB_ABSOLUTE = 0,
	ADC_CALIB_RATIOMETRIC,
	ADC_CALIB_CONFIG_NONE,
};

enum pm8921_adc_channel_scaling_param {
	CHAN_PATH_SCALING1 = 0,
	CHAN_PATH_SCALING2,
	CHAN_PATH_SCALING3,
	CHAN_PATH_SCALING4,
	CHAN_PATH_SCALING_NONE,
};

enum pm8921_adc_amux_input_rsv {
	AMUX_RSV0 = 0,
	AMUX_RSV1,
	AMUX_RSV2,
	AMUX_RSV3,
	AMUX_RSV4,
	AMUX_RSV5,
	AMUX_NONE,
};

enum pm8921_adc_premux_mpp_scale_type {
	PREMUX_MPP_SCALE_0 = 0,
	PREMUX_MPP_SCALE_1,
	PREMUX_MPP_SCALE_1_DIV3,
	PREMUX_MPP_NONE,
};

enum pm8921_adc_scale_fn_type {
	ADC_SCALE_DEFAULT = 0,
	ADC_SCALE_MSM_THERM,
	ADC_SCALE_BATT_THERM,
	ADC_SCALE_PMIC_THERM,
	ADC_SCALE_XTERN_CHGR_CUR,
	ADC_SCALE_NONE,
};

struct pm8921_adc_linear_graph {
	int32_t offset;
	int32_t dy; /* Slope numerator */
	int32_t dx; /* Slope denominator */
};

struct pm8921_adc_map_pt {
	int32_t x;
	int32_t y;
};

struct pm8921_adc_scaling_ratio {
	int32_t num;
	int32_t den;
};

struct pm8921_adc_properties {
	uint32_t	adc_reference; /* milli-voltage for this adc */
	uint32_t	bitresolution;
	bool		bipolar;
	uint32_t	conversiontime;
};

struct pm8921_adc_chan_properties {
	uint32_t			offset_gain_numerator;
	uint32_t			offset_gain_denominator;
	struct pm8921_adc_linear_graph	adc_graph[2];
};

struct pm8921_adc_chan_result {
	uint32_t	chan;
	int32_t		adc_code;
	int64_t		measurement;
	int64_t		physical;
};

#if defined(CONFIG_MFD_PM8921_ADC) || defined(CONFIG_MFD_PM8921_ADC_MODULE)
int32_t pm8921_adc_scale_default(int32_t adc_code,
			const struct pm8921_adc_properties *,
			const struct pm8921_adc_chan_properties *,
			struct pm8921_adc_chan_result *);
int32_t pm8921_adc_tdkntcg_therm(int32_t adc_code,
			const struct pm8921_adc_properties *,
			const struct pm8921_adc_chan_properties *,
			struct pm8921_adc_chan_result *);
int32_t pm8921_adc_scale_msm_therm(int32_t adc_code,
			const struct pm8921_adc_properties *,
			const struct pm8921_adc_chan_properties *,
			struct pm8921_adc_chan_result *);
int32_t pm8921_adc_scale_batt_therm(int32_t adc_code,
			const struct pm8921_adc_properties *,
			const struct pm8921_adc_chan_properties *,
			struct pm8921_adc_chan_result *);
int32_t pm8921_adc_scale_pmic_therm(int32_t adc_code,
			const struct pm8921_adc_properties *,
			const struct pm8921_adc_chan_properties *,
			struct pm8921_adc_chan_result *);
int32_t pm8921_adc_scale_xtern_chgr_cur(int32_t adc_code,
			const struct pm8921_adc_properties *,
			const struct pm8921_adc_chan_properties *,
			struct pm8921_adc_chan_result *);

#else
static inline int32_t pm8921_adc_scale_default(int32_t adc_code,
			const struct pm8921_adc_properties *adc_prop,
			const struct pm8921_adc_chan_properties *chan_prop,
			struct pm8921_adc_chan_result *chan_rslt)
{ return -ENXIO; }
static inline int32_t pm8921_adc_tdkntcg_therm(int32_t adc_code,
			const struct pm8921_adc_properties *adc_prop,
			const struct pm8921_adc_chan_properties *chan_prop,
			struct pm8921_adc_chan_result *chan_rslt)
{ return -ENXIO; }
static inline int32_t pm8921_adc_scale_msm_therm(int32_t adc_code,
			const struct pm8921_adc_properties *adc_prop,
			const struct pm8921_adc_chan_properties *chan_prop,
			struct pm8921_adc_chan_result *chan_rslt)
{ return -ENXIO; }
static inline int32_t pm8921_adc_scale_batt_therm(int32_t adc_code,
			const struct pm8921_adc_properties *adc_prop,
			const struct pm8921_adc_chan_properties *chan_prop,
			struct pm8921_adc_chan_result *chan_rslt)
{ return -ENXIO; }
static inline int32_t pm8921_adc_scale_pmic_therm(int32_t adc_code,
			const struct pm8921_adc_properties *adc_prop,
			const struct pm8921_adc_chan_properties *chan_prop,
			struct pm8921_adc_chan_result *chan_rslt)
{ return -ENXIO; }
static inline int32_t pm8921_adc_scale_xtern_chgr_cur(int32_t adc_code,
			const struct pm8921_adc_properties *adc_prop,
			const struct pm8921_adc_chan_properties *chan_prop,
			struct pm8921_adc_chan_result *chan_rslt)
{ return -ENXIO; }
#endif

struct pm8921_adc_scale_fn {
	int32_t (*chan) (int32_t,
		const struct pm8921_adc_properties *,
		const struct pm8921_adc_chan_properties *,
		struct pm8921_adc_chan_result *);
};

struct pm8921_adc_amux {
	char					*name;
	enum pm8921_adc_channels		channel_name;
	enum pm8921_adc_channel_scaling_param	chan_path_prescaling;
	enum pm8921_adc_amux_input_rsv		adc_rsv;
	enum pm8921_adc_decimation_type		adc_decimation;
	enum pm8921_adc_scale_fn_type		adc_scale_fn;
};

struct pm8921_adc_arb_btm {
	uint32_t					interval;
	struct pm8921_adc_btm_prop			*btm_prop;
	struct pm8921_adc_arb_btm_param			*btm_param;
	struct pm8921_adc_btm_channel_properties	*btm_channel_prop;
};

struct pm8921_adc_btm_channel_properties {
	enum pm8921_adc_channels		btm_channel;
	enum pm8921_adc_decimation_type		decimation;
	enum pm8921_adc_amux_input_rsv		btm_rsv;
	struct pm8921_adc_chan_properties	*chan_prop;
};

struct pm8921_adc_btm_prop {
	uint32_t rs_1;
	uint32_t rs_2;
	uint32_t r_thm;
	uint32_t vref_thm;
};

struct pm8921_adc_arb_btm_param {
	uint32_t	low_thr_temp;
	uint32_t	high_thr_temp;
	uint32_t	low_thr_voltage;
	uint32_t	high_thr_voltage;
	int32_t		interval;
	void		(*btm_warm_fn) (void);
	void		(*btm_cold_fn) (void);
};

int32_t pm8921_adc_batt_scaler(struct pm8921_adc_arb_btm_param *);

struct pm8921_adc_platform_data {
	struct pm8921_adc_properties	*adc_prop;
	u32				(*adc_setup) (void);
	void				(*adc_shutdown) (void);
	int				(*adc_vreg_set) (int);
	int				(*adc_vreg_setup) (void);
	void				(*adc_vreg_shutdown) (void);
	int				adc_num;
	u32				adc_wakeup;
	struct pm8921_adc_amux		*adc_channel;
	uint32_t			adc_num_channel;
};

/* Public API */
#if defined(CONFIG_MFD_PM8921_ADC) || defined(CONFIG_MFD_PM8921_ADC_MODULE)
/**
 * pm8921_adc_read() - Performs ADC read on the channel.
 * @channel:	Input channel to perform the ADC read.
 * @result:	Structure pointer of type adc_chan_result
 *		in which the ADC read results are stored.
 */
uint32_t pm8921_adc_read(enum pm8921_adc_channels channel,
				struct pm8921_adc_chan_result *result);
/**
 * pm8921_mpp_adc_read() - Performs ADC read on the channel.
 * @channel:	Input channel to perform the ADC read.
 * @result:	Structure pointer of type adc_chan_result
 *		in which the ADC read results are stored.
 * @mpp_scale:	The pre scale value to be performed to the input signal
 *		passed. Currently the pre-scale support is for 1 and 1/3.
 */
uint32_t pm8921_adc_mpp_read(enum pm8921_adc_mpp_channels channel,
			struct pm8921_adc_chan_result *result,
			enum pm8921_adc_premux_mpp_scale_type);
/**
 * pm8921_adc_btm_start() - Configure the BTM registers and start
			monitoring the BATT_THERM channel for
			threshold warm/cold temperature set
			by the Battery client. The btm_start
			api is to be used after calling the
			pm8921_btm_configure() api which sets
			the temperature thresholds, interval
			and functions to call when warm/cold
			events are triggered.
 * @param:	none.
 */
uint32_t pm8921_adc_btm_start(void);

/**
 * pm8921_adc_btm_end() - Configures the BTM registers to stop
 *			monitoring the BATT_THERM channel for
 *			warm/cold events and disables the
 *			interval timer.
 * @param:	none.
 */
uint32_t pm8921_adc_btm_end(void);

/**
 * pm8921_adc_btm_configure() - Configures the BATT_THERM channel
 *			parameters for warm/cold thresholds.
 *			Sets the interval timer for perfoming
 *			reading the temperature done by the HW.
 * @btm_param:		Structure pointer of type adc_arb_btm_param *
 *			which client provides for threshold warm/cold,
 *			interval and functions to call when warm/cold
 *			events are triggered.
 */
uint32_t pm8921_adc_btm_configure(struct pm8921_adc_arb_btm_param *);
#else
static inline uint32_t pm8921_adc_read(uint32_t channel,
				struct pm8921_adc_chan_result *result)
{ return -ENXIO; }
static inline uint32_t pm8921_mpp_adc_read(uint32_t channel,
		struct pm8921_adc_chan_result *result,
		enum pm8921_adc_premux_mpp_scale_type scale)
{ return -ENXIO; }
static inline uint32_t pm8921_adc_btm_start(void)
{ return -ENXIO; }
static inline uint32_t pm8921_adc_btm_end(void)
{ return -ENXIO; }
static inline uint32_t pm8921_adc_btm_configure(
		struct pm8921_adc_arb_btm_param *param)
{ return -ENXIO; }
#endif

#endif /* MFD_PM8921_ADC_H */
