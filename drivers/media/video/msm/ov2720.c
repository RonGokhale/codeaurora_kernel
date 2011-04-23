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

#include <linux/delay.h>
#include <linux/debugfs.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <media/msm_camera.h>
#include <mach/gpio.h>
#include <mach/camera.h>
#include "ov2720.h"
/*=============================================================
	SENSOR REGISTER DEFINES
==============================================================*/
#define REG_GROUPED_PARAMETER_HOLD		0x3208
#define GROUPED_PARAMETER_HOLD_OFF		0x01
#define GROUPED_PARAMETER_HOLD			0x00
/* Integration Time */
#define REG_COARSE_INTEGRATION_TIME		0x3012
/* Gain */
#define REG_GLOBAL_GAIN					0x305E
#define REG_GR_GAIN						0x3056
#define REG_R_GAIN						0x3058
#define REG_B_GAIN						0x305A
#define REG_GB_GAIN						0x305C

/* PLL registers */
#define REG_FRAME_LENGTH_LINES			0x300A
#define REG_LINE_LENGTH_PCK				0x300C
/* Test Pattern */
#define REG_TEST_PATTERN_MODE			0x0601
#define REG_VCM_NEW_CODE				0x30F2
#define AF_ADDR							0x18
#define BRIDGE_ADDR						0x80
/*============================================================================
							 TYPE DECLARATIONS
============================================================================*/

/* 16bit address - 8 bit context register structure */
#define Q8  0x00000100
#define Q10 0x00000400
#define OV2720_MASTER_CLK_RATE 24000000
#define OV2720_OFFSET			8

/* AF Total steps parameters */
#define OV2720_TOTAL_STEPS_NEAR_TO_FAR    32

uint16_t ov2720_step_position_table[OV2720_TOTAL_STEPS_NEAR_TO_FAR+1];
uint16_t ov2720_nl_region_boundary1 = 3;
uint16_t ov2720_nl_region_code_per_step1 = 30;
uint16_t ov2720_l_region_code_per_step = 4;
uint16_t ov2720_damping_threshold = 10;
uint16_t ov2720_sw_damping_time_wait = 1;

struct ov2720_work_t {
	struct work_struct work;
};

static struct ov2720_work_t *ov2720_sensorw;
static struct i2c_client *ov2720_client;

struct ov2720_ctrl_t {
	const struct  msm_camera_sensor_info *sensordata;

	uint32_t sensormode;
	uint32_t fps_divider;/* init to 1 * 0x00000400 */
	uint32_t pict_fps_divider;/* init to 1 * 0x00000400 */
	uint16_t fps;

	uint16_t curr_lens_pos;
	uint16_t curr_step_pos;
	uint16_t my_reg_gain;
	uint32_t my_reg_line_count;
	uint16_t total_lines_per_frame;

	enum ov2720_resolution_t prev_res;
	enum ov2720_resolution_t pict_res;
	enum ov2720_resolution_t curr_res;
	enum ov2720_test_mode_t  set_test;
	enum ov2720_cam_mode_t cam_mode;
};

static uint16_t prev_line_length_pck;
static uint16_t prev_frame_length_lines;
static uint16_t snap_line_length_pck;
static uint16_t snap_frame_length_lines;

static bool CSI_CONFIG;
static struct ov2720_ctrl_t *ov2720_ctrl;
static DECLARE_WAIT_QUEUE_HEAD(ov2720_wait_queue);
DEFINE_MUTEX(ov2720_mut);

static int cam_debug_init(void);
static struct dentry *debugfs_base;
/*=============================================================*/

static int ov2720_i2c_rxdata(unsigned short saddr,
	unsigned char *rxdata, int length)
{
	struct i2c_msg msgs[] = {
		{
			.addr  = saddr,
			.flags = 0,
			.len   = length,
			.buf   = rxdata,
		},
		{
			.addr  = saddr,
			.flags = I2C_M_RD,
			.len   = length,
			.buf   = rxdata,
		},
	};
	if (i2c_transfer(ov2720_client->adapter, msgs, 2) < 0) {
		CDBG("ov2720_i2c_rxdata faild 0x%x\n", saddr);
		return -EIO;
	}
	return 0;
}

static int32_t ov2720_i2c_txdata(unsigned short saddr,
				unsigned char *txdata, int length)
{
	struct i2c_msg msg[] = {
		{
			.addr = saddr,
			.flags = 0,
			.len = length,
			.buf = txdata,
		 },
	};
	if (i2c_transfer(ov2720_client->adapter, msg, 1) < 0) {
		CDBG("ov2720_i2c_txdata faild 0x%x\n", saddr);
		return -EIO;
	}

	return 0;
}

static int32_t ov2720_i2c_read(unsigned short raddr,
	unsigned short *rdata, int rlen)
{
	int32_t rc = 0;
	unsigned char buf[2];
	if (!rdata)
		return -EIO;
	memset(buf, 0, sizeof(buf));
	buf[0] = (raddr & 0xFF00) >> 8;
	buf[1] = (raddr & 0x00FF);
	rc = ov2720_i2c_rxdata(ov2720_client->addr>>1, buf, rlen);
	if (rc < 0) {
		CDBG("ov2720_i2c_read 0x%x failed!\n", raddr);
		return rc;
	}
	*rdata = (rlen == 2 ? buf[0] << 8 | buf[1] : buf[0]);
	CDBG("ov2720_i2c_read 0x%x val = 0x%x!\n", raddr, *rdata);
	return rc;
}

static int32_t ov2720_i2c_write_w_sensor(unsigned short waddr,
	uint16_t wdata)
{
	int32_t rc = -EFAULT;
	unsigned char buf[4];
	memset(buf, 0, sizeof(buf));
	buf[0] = (waddr & 0xFF00) >> 8;
	buf[1] = (waddr & 0x00FF);
	buf[2] = (wdata & 0xFF00) >> 8;
	buf[3] = (wdata & 0x00FF);
	CDBG("i2c_write_b addr = 0x%x, val = 0x%x\n", waddr, wdata);
	rc = ov2720_i2c_txdata(ov2720_client->addr>>1, buf, 4);
	if (rc < 0) {
		CDBG("i2c_write_b failed, addr = 0x%x, val = 0x%x!\n",
			waddr, wdata);
	}
	return rc;
}

static int32_t ov2720_i2c_write_b_sensor(unsigned short waddr,
	uint8_t bdata)
{
	int32_t rc = -EFAULT;
	unsigned char buf[3];
	memset(buf, 0, sizeof(buf));
	buf[0] = (waddr & 0xFF00) >> 8;
	buf[1] = (waddr & 0x00FF);
	buf[2] = bdata;
	CDBG("i2c_write_b addr = 0x%x, val = 0x%x\n", waddr, bdata);
	rc = ov2720_i2c_txdata(ov2720_client->addr>>1, buf, 3);
	if (rc < 0) {
		CDBG("i2c_write_b failed, addr = 0x%x, val = 0x%x!\n",
			waddr, bdata);
	}
	return rc;
}

static int32_t ov2720_i2c_write_w_table(struct ov2720_i2c_reg_conf const
					 *reg_conf_tbl, int num)
{
	int i;
	int32_t rc = -EIO;
	for (i = 0; i < num; i++) {
		rc = ov2720_i2c_write_b_sensor(reg_conf_tbl->waddr,
			reg_conf_tbl->wdata);
		if (rc < 0)
			break;
		reg_conf_tbl++;
	}
	return rc;
}

static void ov2720_group_hold_on(void)
{
	ov2720_i2c_write_b_sensor(REG_GROUPED_PARAMETER_HOLD,
						GROUPED_PARAMETER_HOLD);
}

static void ov2720_group_hold_off(void)
{
	ov2720_i2c_write_b_sensor(REG_GROUPED_PARAMETER_HOLD,
						GROUPED_PARAMETER_HOLD_OFF);
	ov2720_i2c_write_b_sensor(REG_GROUPED_PARAMETER_HOLD,
						0xA0);
}

static void ov2720_start_stream(void)
{
	ov2720_i2c_write_b_sensor(0x0100, 0x01);
}

static void ov2720_stop_stream(void)
{
	ov2720_i2c_write_b_sensor(0x0100, 0x00);
}

static void ov2720_get_pict_fps(uint16_t fps, uint16_t *pfps)
{
	/* input fps is preview fps in Q8 format */
	uint32_t divider, d1, d2;

	d1 = prev_frame_length_lines * 0x00000400 / snap_frame_length_lines;
	d2 = prev_line_length_pck * 0x00000400 / snap_line_length_pck;
	divider = d1 * d2 / 0x400;

	/*Verify PCLK settings and frame sizes.*/
	*pfps = (uint16_t) (fps * divider / 0x400);
	/* 2 is the ratio of no.of snapshot channels
	to number of preview channels */
}

static uint16_t ov2720_get_prev_lines_pf(void)
{
	return prev_frame_length_lines;
}

static uint16_t ov2720_get_prev_pixels_pl(void)
{
	return prev_line_length_pck;
}

static uint16_t ov2720_get_pict_lines_pf(void)
{
	return snap_frame_length_lines;
}

static uint16_t ov2720_get_pict_pixels_pl(void)
{
	return snap_line_length_pck;
}

static uint32_t ov2720_get_pict_max_exp_lc(void)
{
	return snap_frame_length_lines  * 24;
}

static int32_t ov2720_set_fps(struct fps_cfg   *fps)
{
	uint16_t total_lines_per_frame;
	int32_t rc = 0;
	total_lines_per_frame = (uint16_t)
		((prev_frame_length_lines) *
		ov2720_ctrl->fps_divider/0x400);
	ov2720_ctrl->fps_divider = fps->fps_div;
	ov2720_ctrl->pict_fps_divider = fps->pict_fps_div;

	ov2720_group_hold_on();
	rc = ov2720_i2c_write_w_sensor(REG_FRAME_LENGTH_LINES,
							total_lines_per_frame);
	ov2720_group_hold_off();
	return rc;
}

static int32_t ov2720_write_exp_gain(struct sensor_3d_exp_cfg exp_cfg)
{
	uint16_t max_legal_gain = 0xE7F;
	uint16_t gain = exp_cfg.gain;
	uint32_t line = exp_cfg.line;
	int32_t rc = 0;
	if (gain > max_legal_gain) {
		CDBG("Max legal gain Line:%d\n", __LINE__);
		gain = max_legal_gain;
	}
	ov2720_group_hold_on();
	rc = ov2720_i2c_write_w_sensor(REG_GLOBAL_GAIN, gain|0x1000);
	rc = ov2720_i2c_write_w_sensor(REG_COARSE_INTEGRATION_TIME, line);

	ov2720_group_hold_off();
	return rc;
}

static int32_t ov2720_set_pict_exp_gain(struct sensor_3d_exp_cfg exp_cfg)
{
	int32_t rc = 0;
	rc = ov2720_write_exp_gain(exp_cfg);
	return rc;
}

static int32_t ov2720_test(enum ov2720_test_mode_t mo)
{
	int32_t rc = 0;
	if (mo == TEST_OFF)
		return rc;
	else {
		/* REG_0x30D8[4] is TESBYPEN: 0: Normal Operation,
		1: Bypass Signal Processing
		REG_0x30D8[5] is EBDMASK: 0:
		Output Embedded data, 1: No output embedded data */
		if (ov2720_i2c_write_b_sensor(REG_TEST_PATTERN_MODE,
			(uint8_t) mo) < 0) {
			return rc;
		}
	}
	return rc;
}

static int32_t ov2720_sensor_setting(int update_type, int rt)
{

	int32_t rc = 0;
	struct msm_camera_csi_params ov2720_csi_params;
	ov2720_stop_stream();
	msleep(30);
	if (update_type == REG_INIT) {
		CSI_CONFIG = 0;
		ov2720_i2c_write_w_table(ov2720_regs.rec_settings,
			ov2720_regs.rec_size);
		ov2720_i2c_write_w_table(
			ov2720_regs.conf_array[rt].conf,
			ov2720_regs.conf_array[rt].size);
	} else if (update_type == UPDATE_PERIODIC) {
		cam_debug_init();

		msleep(20);
		if (!CSI_CONFIG) {
			ov2720_csi_params.lane_cnt = 2;
			ov2720_csi_params.data_format = CSI_10BIT;
			ov2720_csi_params.lane_assign = 0xe4;
			ov2720_csi_params.dpcm_scheme = 0;
			ov2720_csi_params.settle_cnt = 0x18;
			msm_camio_vfe_clk_rate_set(266667000);
			rc = msm_camio_csi_config(&ov2720_csi_params);
			msleep(20);
			CSI_CONFIG = 1;
		}
		ov2720_start_stream();
		msleep(30);
	}
	return rc;
}

static int32_t ov2720_video_config(int mode)
{

	int32_t rc = 0;
	/* change sensor resolution if needed */
	if (ov2720_sensor_setting(UPDATE_PERIODIC,
		ov2720_ctrl->prev_res) < 0)
		return rc;
	if (ov2720_ctrl->set_test) {
		if (ov2720_test(ov2720_ctrl->set_test) < 0)
			return  rc;
	}

	ov2720_ctrl->curr_res = ov2720_ctrl->prev_res;
	ov2720_ctrl->sensormode = mode;
	return rc;
}

static int32_t ov2720_snapshot_config(int mode)
{
	int32_t rc = 0;
	/*change sensor resolution if needed */
	if (ov2720_ctrl->curr_res != ov2720_ctrl->pict_res) {
		if (ov2720_sensor_setting(UPDATE_PERIODIC,
					ov2720_ctrl->pict_res) < 0)
			return rc;
	}

	ov2720_ctrl->curr_res = ov2720_ctrl->pict_res;
	ov2720_ctrl->sensormode = mode;
	return rc;
} /*end of ov2720_snapshot_config*/

static int32_t ov2720_raw_snapshot_config(int mode)
{
	int32_t rc = 0;
	/* change sensor resolution if needed */
	if (ov2720_ctrl->curr_res != ov2720_ctrl->pict_res) {
		if (ov2720_sensor_setting(UPDATE_PERIODIC,
					ov2720_ctrl->pict_res) < 0)
			return rc;
	}

	ov2720_ctrl->curr_res = ov2720_ctrl->pict_res;
	ov2720_ctrl->sensormode = mode;
	return rc;
} /*end of ov2720_raw_snapshot_config*/
static int32_t ov2720_mode_init(int mode, struct sensor_init_cfg init_info)
{
	int32_t rc = 0;
	CDBG("%s: %d\n", __func__, __LINE__);
	if (mode != ov2720_ctrl->cam_mode) {
		ov2720_ctrl->prev_res = init_info.prev_res;
		ov2720_ctrl->pict_res = init_info.pict_res;
		ov2720_ctrl->cam_mode = mode;

		prev_frame_length_lines =
			ov2720_regs.conf_array[ov2720_ctrl->prev_res].
			conf[OV2720_FRAME_LENGTH_LINES_HI].wdata << 8 |
			ov2720_regs.conf_array[ov2720_ctrl->prev_res].
			conf[OV2720_FRAME_LENGTH_LINES_LO].wdata;
		prev_line_length_pck =
			ov2720_regs.conf_array[ov2720_ctrl->prev_res].
			conf[OV2720_LINE_LENGTH_PCK_HI].wdata << 8 |
			ov2720_regs.conf_array[ov2720_ctrl->prev_res].
			conf[OV2720_LINE_LENGTH_PCK_LO].wdata;
		snap_frame_length_lines = prev_frame_length_lines;
		snap_line_length_pck = prev_line_length_pck;

		rc = ov2720_sensor_setting(REG_INIT,
			ov2720_ctrl->prev_res);
	}
	return rc;
}

static int32_t ov2720_set_sensor_mode(int mode,
	int res)
{
	int32_t rc = 0;

	switch (mode) {
	case SENSOR_PREVIEW_MODE:
		ov2720_ctrl->prev_res = res;
		rc = ov2720_video_config(mode);
		break;
	case SENSOR_SNAPSHOT_MODE:
		ov2720_ctrl->pict_res = res;
		rc = ov2720_snapshot_config(mode);
		break;
	case SENSOR_RAW_SNAPSHOT_MODE:
		ov2720_ctrl->pict_res = res;
		rc = ov2720_raw_snapshot_config(mode);
		break;
	default:
		rc = -EINVAL;
		break;
	}
	return rc;
}

static int32_t ov2720_power_down(void)
{
	return 0;
}

static int ov2720_probe_init_done(const struct msm_camera_sensor_info *data)
{
	CDBG("probe done\n");
	gpio_free(data->sensor_reset);
	return 0;
}

static int ov2720_probe_init_sensor(
	const struct msm_camera_sensor_info *data)
{
	int32_t rc = 0;
	uint16_t chipid = 0;
	CDBG("%s: %d\n", __func__, __LINE__);
	rc = gpio_request(data->sensor_reset, "ov2720");
	CDBG(" ov2720_probe_init_sensor\n");
	if (!rc) {
		CDBG("sensor_reset = %d\n", rc);
		gpio_direction_output(data->sensor_reset, 0);
		msleep(50);
		gpio_set_value_cansleep(data->sensor_reset, 1);
		msleep(20);
	} else {
		goto init_probe_done;
	}

	CDBG(" ov2720_probe_init_sensor is called\n");
	rc = ov2720_i2c_read(0x300A, &chipid, 2);
	CDBG("ID: %d\n", chipid);
	/* 4. Compare sensor ID to OV2720 ID: */
	if (chipid != 0x2720) {
		rc = -ENODEV;
		CDBG("ov2720_probe_init_sensor chip id doesnot match\n");
		goto init_probe_fail;
	}
	goto init_probe_done;
init_probe_fail:
	CDBG(" ov2720_probe_init_sensor fails\n");
	gpio_set_value_cansleep(data->sensor_reset, 0);
	ov2720_probe_init_done(data);
init_probe_done:
	CDBG(" ov2720_probe_init_sensor finishes\n");
	return rc;
}
/* camsensor_ov2720_reset */

int ov2720_sensor_open_init(const struct msm_camera_sensor_info *data)
{
	int32_t rc = 0;
	CDBG("%s: %d\n", __func__, __LINE__);
	CDBG("Calling ov2720_sensor_open_init\n");

	ov2720_ctrl = kzalloc(sizeof(struct ov2720_ctrl_t), GFP_KERNEL);
	if (!ov2720_ctrl) {
		CDBG("ov2720_init failed!\n");
		rc = -ENOMEM;
		goto init_done;
	}
	ov2720_ctrl->fps_divider = 1 * 0x00000400;
	ov2720_ctrl->pict_fps_divider = 1 * 0x00000400;
	ov2720_ctrl->set_test = TEST_OFF;
	ov2720_ctrl->cam_mode = MODE_INVALID;

	if (data)
		ov2720_ctrl->sensordata = data;
	if (rc < 0) {
		CDBG("Calling ov2720_sensor_open_init fail1\n");
		return rc;
	}
	CDBG("%s: %d\n", __func__, __LINE__);
	/* enable mclk first */
	msm_camio_clk_rate_set(OV2720_MASTER_CLK_RATE);
	rc = ov2720_probe_init_sensor(data);
	if (rc < 0)
		goto init_fail;

	ov2720_ctrl->fps = 30*Q8;
	if (rc < 0) {
		gpio_set_value_cansleep(data->sensor_reset, 0);
		goto init_fail;
	} else
		goto init_done;
init_fail:
	CDBG("init_fail\n");
	ov2720_probe_init_done(data);
init_done:
	CDBG("init_done\n");
	return rc;
} /*endof ov2720_sensor_open_init*/

static int ov2720_init_client(struct i2c_client *client)
{
	/* Initialize the MSM_CAMI2C Chip */
	init_waitqueue_head(&ov2720_wait_queue);
	return 0;
}

static const struct i2c_device_id ov2720_i2c_id[] = {
	{"ov2720", 0},
	{ }
};

static int ov2720_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int rc = 0;
	CDBG("ov2720_probe called!\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		CDBG("i2c_check_functionality failed\n");
		goto probe_failure;
	}

	ov2720_sensorw = kzalloc(sizeof(struct ov2720_work_t),
			GFP_KERNEL);
	if (!ov2720_sensorw) {
		CDBG("kzalloc failed.\n");
		rc = -ENOMEM;
		goto probe_failure;
	}

	i2c_set_clientdata(client, ov2720_sensorw);
	ov2720_init_client(client);
	ov2720_client = client;

	msleep(50);

	CDBG("ov2720_probe successed! rc = %d\n", rc);
	return 0;

probe_failure:
	CDBG("ov2720_probe failed! rc = %d\n", rc);
	return rc;
}

static int ov2720_send_wb_info(struct wb_info_cfg *wb)
{
	return 0;

} /*end of ov2720_snapshot_config*/

static int __exit ov2720_remove(struct i2c_client *client)
{
	struct ov2720_work_t_t *sensorw = i2c_get_clientdata(client);
	free_irq(client->irq, sensorw);
	ov2720_client = NULL;
	kfree(sensorw);
	return 0;
}

static struct i2c_driver ov2720_i2c_driver = {
	.id_table = ov2720_i2c_id,
	.probe  = ov2720_i2c_probe,
	.remove = __exit_p(ov2720_i2c_remove),
	.driver = {
		.name = "ov2720",
	},
};

int ov2720_sensor_config(void __user *argp)
{
	struct sensor_cfg_data cdata;
	long   rc = 0;
	if (copy_from_user(&cdata,
		(void *)argp,
		sizeof(struct sensor_cfg_data)))
		return -EFAULT;
	mutex_lock(&ov2720_mut);
	CDBG("ov2720_sensor_config: cfgtype = %d\n",
		 cdata.cfgtype);
	switch (cdata.cfgtype) {
	case CFG_GET_PICT_FPS:
		ov2720_get_pict_fps(
			cdata.cfg.gfps.prevfps,
			&(cdata.cfg.gfps.pictfps));

		if (copy_to_user((void *)argp,
			&cdata,
			sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;

	case CFG_GET_PREV_L_PF:
		cdata.cfg.prevl_pf =
		ov2720_get_prev_lines_pf();

		if (copy_to_user((void *)argp,
			&cdata,
			sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;

	case CFG_GET_PREV_P_PL:
		cdata.cfg.prevp_pl =
			ov2720_get_prev_pixels_pl();

		if (copy_to_user((void *)argp,
			&cdata,
			sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;

	case CFG_GET_PICT_L_PF:
		cdata.cfg.pictl_pf =
			ov2720_get_pict_lines_pf();

		if (copy_to_user((void *)argp,
			&cdata,
			sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;

	case CFG_GET_PICT_P_PL:
		cdata.cfg.pictp_pl =
			ov2720_get_pict_pixels_pl();

		if (copy_to_user((void *)argp,
			&cdata,
			sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;

	case CFG_GET_PICT_MAX_EXP_LC:
		cdata.cfg.pict_max_exp_lc =
			ov2720_get_pict_max_exp_lc();

		if (copy_to_user((void *)argp,
			&cdata,
			sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;

	case CFG_SET_FPS:
	case CFG_SET_PICT_FPS:
		rc = ov2720_set_fps(&(cdata.cfg.fps));
		break;

	case CFG_SET_EXP_GAIN:
		rc = ov2720_write_exp_gain(
			cdata.cfg.sensor_3d_exp);
		break;

	case CFG_SET_PICT_EXP_GAIN:
		rc = ov2720_set_pict_exp_gain(
			cdata.cfg.sensor_3d_exp);
		break;

	case CFG_SET_MODE:
		rc = ov2720_set_sensor_mode(cdata.mode, cdata.rs);
		break;

	case CFG_PWR_DOWN:
		rc = ov2720_power_down();
		break;

	case CFG_MOVE_FOCUS:
		break;

	case CFG_SET_DEFAULT_FOCUS:
		break;

	case CFG_GET_AF_MAX_STEPS:
		cdata.max_steps = OV2720_TOTAL_STEPS_NEAR_TO_FAR;
		if (copy_to_user((void *)argp,
			&cdata,
			sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;

	case CFG_SET_EFFECT:
		break;

	case CFG_SEND_WB_INFO:
		rc = ov2720_send_wb_info(
			&(cdata.cfg.wb_info));
	break;

	case CFG_SENSOR_INIT:
		rc = ov2720_mode_init(cdata.mode,
				cdata.cfg.init_info);
	break;

	default:
		rc = -EFAULT;
		break;
	}

	mutex_unlock(&ov2720_mut);

	return rc;
}

static int ov2720_sensor_release(void)
{
	int rc = -EBADF;
	mutex_lock(&ov2720_mut);
	ov2720_power_down();
	gpio_set_value_cansleep(ov2720_ctrl->sensordata->sensor_reset, 0);
	msleep(20);
	gpio_free(ov2720_ctrl->sensordata->sensor_reset);
	kfree(ov2720_ctrl);
	ov2720_ctrl = NULL;
	CDBG("ov2720_release completed\n");
	mutex_unlock(&ov2720_mut);

	return rc;
}

static int ov2720_sensor_probe(const struct msm_camera_sensor_info *info,
		struct msm_sensor_ctrl *s)
{
	int rc = 0;
	rc = i2c_add_driver(&ov2720_i2c_driver);
	if (rc < 0 || ov2720_client == NULL) {
		rc = -ENOTSUPP;
		CDBG("I2C add driver failed");
		goto probe_fail;
	}
	msm_camio_clk_rate_set(OV2720_MASTER_CLK_RATE);
	rc = ov2720_probe_init_sensor(info);
	if (rc < 0)
		goto probe_fail;
	s->s_init = ov2720_sensor_open_init;
	s->s_release = ov2720_sensor_release;
	s->s_config  = ov2720_sensor_config;
	s->s_mount_angle = 270;

	gpio_set_value_cansleep(info->sensor_reset, 0);
	ov2720_probe_init_done(info);
	return rc;

probe_fail:
	CDBG("ov2720_sensor_probe: SENSOR PROBE FAILS!\n");
	return rc;
}

static int __ov2720_probe(struct platform_device *pdev)
{
	return msm_camera_drv_start(pdev, ov2720_sensor_probe);
}

static struct platform_driver msm_camera_driver = {
	.probe = __ov2720_probe,
	.driver = {
		.name = "msm_camera_ov2720",
		.owner = THIS_MODULE,
	},
};

static int __init ov2720_init(void)
{
	return platform_driver_register(&msm_camera_driver);
}

module_init(ov2720_init);
void ov2720_exit(void)
{
	i2c_del_driver(&ov2720_i2c_driver);
}
MODULE_DESCRIPTION("Aptina 8 MP Bayer sensor driver");
MODULE_LICENSE("GPL v2");

static bool streaming = 1;

static int cam_debug_stream_set(void *data, u64 val)
{
	int rc = 0;

	if (val) {
		ov2720_start_stream();
		streaming = 1;
	} else {
		ov2720_stop_stream();
		streaming = 0;
	}

	return rc;
}

static int cam_debug_stream_get(void *data, u64 *val)
{
	*val = streaming;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(cam_stream, cam_debug_stream_get,
			cam_debug_stream_set, "%llu\n");


static int cam_debug_init(void)
{
	struct dentry *cam_dir;
	debugfs_base = debugfs_create_dir("sensor", NULL);
	if (!debugfs_base)
		return -ENOMEM;

	cam_dir = debugfs_create_dir("ov2720", debugfs_base);
	if (!cam_dir)
		return -ENOMEM;

	if (!debugfs_create_file("stream", S_IRUGO | S_IWUSR, cam_dir,
							 NULL, &cam_stream))
		return -ENOMEM;
	return 0;
}



