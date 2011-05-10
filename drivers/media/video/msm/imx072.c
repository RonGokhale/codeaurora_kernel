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
#include "imx072.h"

/* SENSOR REGISTER DEFINES */
#define REG_GROUPED_PARAMETER_HOLD		0x0104
#define GROUPED_PARAMETER_HOLD_OFF		0x00
#define GROUPED_PARAMETER_HOLD			0x01
/* Integration Time */
#define REG_COARSE_INTEGRATION_TIME		0x0202
/* Gain */
#define REG_GLOBAL_GAIN					0x0204

/* PLL registers */
#define REG_FRAME_LENGTH_LINES			0x0340
#define REG_LINE_LENGTH_PCK				0x0342

/* 16bit address - 8 bit context register structure */
#define Q8  0x00000100
#define Q10 0x00000400
#define IMX072_MASTER_CLK_RATE 24000000
#define IMX072_OFFSET		3

/* AF Total steps parameters */
#define IMX072_TOTAL_STEPS_NEAR_TO_FAR    32

struct imx072_work_t {
	struct work_struct work;
};

static struct imx072_work_t *imx072_sensorw;
static struct i2c_client *imx072_client;

struct imx072_ctrl_t {
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

	enum imx072_resolution_t prev_res;
	enum imx072_resolution_t pict_res;
	enum imx072_resolution_t curr_res;
	enum imx072_test_mode_t  set_test;
	enum imx072_cam_mode_t cam_mode;
};

static uint16_t prev_line_length_pck;
static uint16_t prev_frame_length_lines;
static uint16_t snap_line_length_pck;
static uint16_t snap_frame_length_lines;

static bool CSI_CONFIG;
static struct imx072_ctrl_t *imx072_ctrl;
static DECLARE_WAIT_QUEUE_HEAD(imx072_wait_queue);
DEFINE_MUTEX(imx072_mut);

#ifdef CONFIG_DEBUG_FS
static int cam_debug_init(void);
static struct dentry *debugfs_base;
#endif

static int imx072_i2c_rxdata(unsigned short saddr,
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
	if (i2c_transfer(imx072_client->adapter, msgs, 2) < 0) {
		pr_err("imx072_i2c_rxdata faild 0x%x\n", saddr);
		return -EIO;
	}
	return 0;
}

static int32_t imx072_i2c_txdata(unsigned short saddr,
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
	if (i2c_transfer(imx072_client->adapter, msg, 1) < 0) {
		pr_err("imx072_i2c_txdata faild 0x%x\n", saddr);
		return -EIO;
	}

	return 0;
}

static int32_t imx072_i2c_read(unsigned short raddr,
	unsigned short *rdata, int rlen)
{
	int32_t rc = 0;
	unsigned char buf[2];
	if (!rdata)
		return -EIO;
	memset(buf, 0, sizeof(buf));
	buf[0] = (raddr & 0xFF00) >> 8;
	buf[1] = (raddr & 0x00FF);
	rc = imx072_i2c_rxdata(imx072_client->addr>>1, buf, rlen);
	if (rc < 0) {
		pr_err("imx072_i2c_read 0x%x failed!\n", raddr);
		return rc;
	}
	*rdata = (rlen == 2 ? buf[0] << 8 | buf[1] : buf[0]);
	return rc;
}

static int32_t imx072_i2c_write_w_sensor(unsigned short waddr,
	uint16_t wdata)
{
	int32_t rc = -EFAULT;
	unsigned char buf[4];
	memset(buf, 0, sizeof(buf));
	buf[0] = (waddr & 0xFF00) >> 8;
	buf[1] = (waddr & 0x00FF);
	buf[2] = (wdata & 0xFF00) >> 8;
	buf[3] = (wdata & 0x00FF);
	pr_err("i2c_write_b addr = 0x%x, val = 0x%x\n", waddr, wdata);
	rc = imx072_i2c_txdata(imx072_client->addr>>1, buf, 4);
	if (rc < 0) {
		pr_err("i2c_write_b failed, addr = 0x%x, val = 0x%x!\n",
			waddr, wdata);
	}
	return rc;
}

static int32_t imx072_i2c_write_b_sensor(unsigned short waddr,
	uint8_t bdata)
{
	int32_t rc = -EFAULT;
	unsigned char buf[3];
	memset(buf, 0, sizeof(buf));
	buf[0] = (waddr & 0xFF00) >> 8;
	buf[1] = (waddr & 0x00FF);
	buf[2] = bdata;
	pr_err("i2c_write_b addr = 0x%x, val = 0x%x\n", waddr, bdata);
	rc = imx072_i2c_txdata(imx072_client->addr>>1, buf, 3);
	if (rc < 0) {
		pr_err("i2c_write_b failed, addr = 0x%x, val = 0x%x!\n",
			waddr, bdata);
	}
	return rc;
}

static int32_t imx072_i2c_write_w_table(struct imx072_i2c_reg_conf const
					 *reg_conf_tbl, int num)
{
	int i;
	int32_t rc = -EIO;
	for (i = 0; i < num; i++) {
		rc = imx072_i2c_write_b_sensor(reg_conf_tbl->waddr,
			reg_conf_tbl->wdata);
		if (rc < 0)
			break;
		reg_conf_tbl++;
	}
	return rc;
}

static void imx072_group_hold_on(void)
{
	imx072_i2c_write_b_sensor(REG_GROUPED_PARAMETER_HOLD,
						GROUPED_PARAMETER_HOLD);
}

static void imx072_group_hold_off(void)
{
	imx072_i2c_write_b_sensor(REG_GROUPED_PARAMETER_HOLD,
						GROUPED_PARAMETER_HOLD_OFF);
}

static void imx072_start_stream(void)
{
	imx072_i2c_write_b_sensor(0x0100, 0x01);
}

static void imx072_stop_stream(void)
{
	imx072_i2c_write_b_sensor(0x0100, 0x00);
}

static void imx072_get_pict_fps(uint16_t fps, uint16_t *pfps)
{
	/* input fps is preview fps in Q8 format */
	uint32_t divider, d1, d2;

	d1 = prev_frame_length_lines * 0x00000400 / snap_frame_length_lines;
	d2 = prev_line_length_pck * 0x00000400 / snap_line_length_pck;
	divider = d1 * d2 / 0x400;

	/*Verify PCLK settings and frame sizes.*/
	*pfps = (uint16_t) (fps * divider / 0x400);
}

static uint16_t imx072_get_prev_lines_pf(void)
{
	return prev_frame_length_lines;
}

static uint16_t imx072_get_prev_pixels_pl(void)
{
	return prev_line_length_pck;
}

static uint16_t imx072_get_pict_lines_pf(void)
{
	return snap_frame_length_lines;
}

static uint16_t imx072_get_pict_pixels_pl(void)
{
	return snap_line_length_pck;
}

static uint32_t imx072_get_pict_max_exp_lc(void)
{
	return snap_frame_length_lines  * 24;
}

static int32_t imx072_set_fps(struct fps_cfg   *fps)
{
	uint16_t total_lines_per_frame;
	int32_t rc = 0;
	total_lines_per_frame = (uint16_t)
		((prev_frame_length_lines *
		imx072_ctrl->fps_divider)/0x400);
	imx072_ctrl->fps_divider = fps->fps_div;
	imx072_ctrl->pict_fps_divider = fps->pict_fps_div;

	imx072_group_hold_on();
	rc = imx072_i2c_write_w_sensor(REG_FRAME_LENGTH_LINES,
							total_lines_per_frame);
	imx072_group_hold_off();
	return rc;
}

static int32_t imx072_write_exp_gain(uint16_t gain, uint32_t line)
{
	uint32_t fl_lines;
	uint8_t offset;
	int32_t rc = 0;
	fl_lines = prev_frame_length_lines;
	line = (line * imx072_ctrl->fps_divider) / Q10;
	offset = IMX072_OFFSET;
	if (line > (fl_lines - offset))
		fl_lines = line + offset;

	imx072_group_hold_on();
	rc = imx072_i2c_write_w_sensor(REG_FRAME_LENGTH_LINES, fl_lines);
	rc = imx072_i2c_write_w_sensor(REG_COARSE_INTEGRATION_TIME, line);
	rc = imx072_i2c_write_w_sensor(REG_GLOBAL_GAIN, gain);
	imx072_group_hold_off();
	return rc;
}

static int32_t imx072_set_pict_exp_gain(uint16_t gain, uint32_t line)
{
	int32_t rc = 0;
	rc = imx072_write_exp_gain(gain, line);
	return rc;
}

static int32_t imx072_sensor_setting(int update_type, int rt)
{

	int32_t rc = 0;
	struct msm_camera_csi_params imx072_csi_params;

	imx072_stop_stream();
	msleep(30);
	if (update_type == REG_INIT) {
		msleep(20);
		CSI_CONFIG = 0;
		imx072_i2c_write_w_table(imx072_regs.rec_settings,
			imx072_regs.rec_size);
	} else if (update_type == UPDATE_PERIODIC) {
#ifdef CONFIG_DEBUG_FS
		cam_debug_init();
#endif
		msleep(20);
		if (!CSI_CONFIG) {
			imx072_csi_params.lane_cnt = 2;
			imx072_csi_params.data_format = CSI_10BIT;
			imx072_csi_params.lane_assign = 0xe4;
			imx072_csi_params.dpcm_scheme = 0;
			imx072_csi_params.settle_cnt = 0x18;
			msm_camio_vfe_clk_rate_set(192000000);
			rc = msm_camio_csi_config(&imx072_csi_params);
			msleep(100);
			CSI_CONFIG = 1;
		}
		imx072_i2c_write_w_table(
			imx072_regs.conf_array[rt].conf,
			imx072_regs.conf_array[rt].size);
		imx072_start_stream();
		msleep(30);
	}
	return rc;
}

static int32_t imx072_video_config(int mode)
{

	int32_t rc = 0;
	/* change sensor resolution if needed */
	if (imx072_sensor_setting(UPDATE_PERIODIC,
		imx072_ctrl->prev_res) < 0)
		return rc;

	imx072_ctrl->curr_res = imx072_ctrl->prev_res;
	imx072_ctrl->sensormode = mode;
	return rc;
}

static int32_t imx072_snapshot_config(int mode)
{
	int32_t rc = 0;
	/*change sensor resolution if needed */
	if (imx072_ctrl->curr_res != imx072_ctrl->pict_res) {
		if (imx072_sensor_setting(UPDATE_PERIODIC,
					imx072_ctrl->pict_res) < 0)
			return rc;
	}

	imx072_ctrl->curr_res = imx072_ctrl->pict_res;
	imx072_ctrl->sensormode = mode;
	return rc;
}

static int32_t imx072_raw_snapshot_config(int mode)
{
	int32_t rc = 0;
	/* change sensor resolution if needed */
	if (imx072_ctrl->curr_res != imx072_ctrl->pict_res) {
		if (imx072_sensor_setting(UPDATE_PERIODIC,
					imx072_ctrl->pict_res) < 0)
			return rc;
	}

	imx072_ctrl->curr_res = imx072_ctrl->pict_res;
	imx072_ctrl->sensormode = mode;
	return rc;
}

static int32_t imx072_mode_init(int mode, struct sensor_init_cfg init_info)
{
	int32_t rc = 0;
	pr_err("%s: %d\n", __func__, __LINE__);
	if (mode != imx072_ctrl->cam_mode) {
		imx072_ctrl->prev_res = init_info.prev_res;
		imx072_ctrl->pict_res = init_info.pict_res;
		imx072_ctrl->cam_mode = mode;

		prev_frame_length_lines =
			imx072_regs.conf_array[imx072_ctrl->prev_res].
			conf[IMX072_FRAME_LENGTH_LINES_HI].wdata << 8 |
			imx072_regs.conf_array[imx072_ctrl->prev_res].
			conf[IMX072_FRAME_LENGTH_LINES_LO].wdata;
		prev_line_length_pck =
			imx072_regs.conf_array[imx072_ctrl->prev_res].
			conf[IMX072_LINE_LENGTH_PCK_HI].wdata << 8 |
			imx072_regs.conf_array[imx072_ctrl->prev_res].
			conf[IMX072_LINE_LENGTH_PCK_LO].wdata;
		snap_frame_length_lines = prev_frame_length_lines;
		snap_line_length_pck = prev_line_length_pck;

		rc = imx072_sensor_setting(REG_INIT,
			imx072_ctrl->prev_res);
	}
	return rc;
}

static int32_t imx072_set_sensor_mode(int mode,
	int res)
{
	int32_t rc = 0;

	switch (mode) {
	case SENSOR_PREVIEW_MODE:
		imx072_ctrl->prev_res = res;
		rc = imx072_video_config(mode);
		break;
	case SENSOR_SNAPSHOT_MODE:
		imx072_ctrl->pict_res = res;
		rc = imx072_snapshot_config(mode);
		break;
	case SENSOR_RAW_SNAPSHOT_MODE:
		imx072_ctrl->pict_res = res;
		rc = imx072_raw_snapshot_config(mode);
		break;
	default:
		rc = -EINVAL;
		break;
	}
	return rc;
}

static int32_t imx072_power_down(void)
{
	return 0;
}

static int imx072_probe_init_done(const struct msm_camera_sensor_info *data)
{
	pr_err("probe done\n");
	gpio_free(data->sensor_reset);
	return 0;
}

static int imx072_probe_init_sensor(
	const struct msm_camera_sensor_info *data)
{
	int32_t rc = 0;
	uint16_t chipid = 0;
	pr_err("%s: %d\n", __func__, __LINE__);
	rc = gpio_request(data->sensor_reset, "imx072");
	pr_err(" imx072_probe_init_sensor\n");
	if (!rc) {
		pr_err("sensor_reset = %d\n", rc);
		gpio_direction_output(data->sensor_reset, 0);
		msleep(50);
		gpio_set_value_cansleep(data->sensor_reset, 1);
		msleep(20);
	} else {
		goto init_probe_done;
	}

	pr_err(" imx072_probe_init_sensor is called\n");
	rc = imx072_i2c_read(0x0, &chipid, 2);
	pr_err("ID: %d\n", chipid);
	/* 4. Compare sensor ID to IMX072 ID: */
	if (chipid != 0x0045) {
		rc = -ENODEV;
		pr_err("imx072_probe_init_sensor chip id doesnot match\n");
		goto init_probe_fail;
	}
	goto init_probe_done;
init_probe_fail:
	pr_err(" imx072_probe_init_sensor fails\n");
	gpio_set_value_cansleep(data->sensor_reset, 0);
	imx072_probe_init_done(data);
init_probe_done:
	pr_err(" imx072_probe_init_sensor finishes\n");
	return rc;
}

int imx072_sensor_open_init(const struct msm_camera_sensor_info *data)
{
	int32_t rc = 0;
	pr_err("%s: %d\n", __func__, __LINE__);
	pr_err("Calling imx072_sensor_open_init\n");

	imx072_ctrl = kzalloc(sizeof(struct imx072_ctrl_t), GFP_KERNEL);
	if (!imx072_ctrl) {
		pr_err("imx072_init failed!\n");
		rc = -ENOMEM;
		goto init_done;
	}
	imx072_ctrl->fps_divider = 1 * 0x00000400;
	imx072_ctrl->pict_fps_divider = 1 * 0x00000400;
	imx072_ctrl->set_test = TEST_OFF;
	imx072_ctrl->cam_mode = MODE_INVALID;

	if (data)
		imx072_ctrl->sensordata = data;
	if (rc < 0) {
		pr_err("Calling imx072_sensor_open_init fail1\n");
		return rc;
	}
	pr_err("%s: %d\n", __func__, __LINE__);
	/* enable mclk first */
	msm_camio_clk_rate_set(IMX072_MASTER_CLK_RATE);
	rc = imx072_probe_init_sensor(data);
	if (rc < 0)
		goto init_fail;

	imx072_ctrl->fps = 30*Q8;
	if (rc < 0) {
		gpio_set_value_cansleep(data->sensor_reset, 0);
		goto init_fail;
	} else
		goto init_done;
init_fail:
	pr_err("init_fail\n");
	imx072_probe_init_done(data);
init_done:
	pr_err("init_done\n");
	return rc;
}

static int imx072_init_client(struct i2c_client *client)
{
	/* Initialize the MSM_CAMI2C Chip */
	init_waitqueue_head(&imx072_wait_queue);
	return 0;
}

static const struct i2c_device_id imx072_i2c_id[] = {
	{"imx072", 0},
	{ }
};

static int imx072_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int rc = 0;
	pr_err("imx072_probe called!\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("i2c_check_functionality failed\n");
		goto probe_failure;
	}

	imx072_sensorw = kzalloc(sizeof(struct imx072_work_t),
			GFP_KERNEL);
	if (!imx072_sensorw) {
		pr_err("kzalloc failed.\n");
		rc = -ENOMEM;
		goto probe_failure;
	}

	i2c_set_clientdata(client, imx072_sensorw);
	imx072_init_client(client);
	imx072_client = client;

	msleep(50);

	pr_err("imx072_probe successed! rc = %d\n", rc);
	return 0;

probe_failure:
	pr_err("imx072_probe failed! rc = %d\n", rc);
	return rc;
}

static int imx072_send_wb_info(struct wb_info_cfg *wb)
{
	return 0;

}

static int __exit imx072_remove(struct i2c_client *client)
{
	struct imx072_work_t_t *sensorw = i2c_get_clientdata(client);
	free_irq(client->irq, sensorw);
	imx072_client = NULL;
	kfree(sensorw);
	return 0;
}

static struct i2c_driver imx072_i2c_driver = {
	.id_table = imx072_i2c_id,
	.probe  = imx072_i2c_probe,
	.remove = __exit_p(imx072_i2c_remove),
	.driver = {
		.name = "imx072",
	},
};

int imx072_sensor_config(void __user *argp)
{
	struct sensor_cfg_data cdata;
	long   rc = 0;
	if (copy_from_user(&cdata,
		(void *)argp,
		sizeof(struct sensor_cfg_data)))
		return -EFAULT;
	mutex_lock(&imx072_mut);
	pr_err("imx072_sensor_config: cfgtype = %d\n",
		 cdata.cfgtype);
	switch (cdata.cfgtype) {
	case CFG_GET_PICT_FPS:
		imx072_get_pict_fps(
			cdata.cfg.gfps.prevfps,
			&(cdata.cfg.gfps.pictfps));

		if (copy_to_user((void *)argp,
			&cdata,
			sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;
	case CFG_GET_PREV_L_PF:
		cdata.cfg.prevl_pf =
		imx072_get_prev_lines_pf();

		if (copy_to_user((void *)argp,
			&cdata,
			sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;
	case CFG_GET_PREV_P_PL:
		cdata.cfg.prevp_pl =
			imx072_get_prev_pixels_pl();

		if (copy_to_user((void *)argp,
			&cdata,
			sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;
	case CFG_GET_PICT_L_PF:
		cdata.cfg.pictl_pf =
			imx072_get_pict_lines_pf();

		if (copy_to_user((void *)argp,
			&cdata,
			sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;
	case CFG_GET_PICT_P_PL:
		cdata.cfg.pictp_pl =
			imx072_get_pict_pixels_pl();

		if (copy_to_user((void *)argp,
			&cdata,
			sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;
	case CFG_GET_PICT_MAX_EXP_LC:
		cdata.cfg.pict_max_exp_lc =
			imx072_get_pict_max_exp_lc();

		if (copy_to_user((void *)argp,
			&cdata,
			sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;
	case CFG_SET_FPS:
	case CFG_SET_PICT_FPS:
		rc = imx072_set_fps(&(cdata.cfg.fps));
		break;
	case CFG_SET_EXP_GAIN:
		rc = imx072_write_exp_gain(
			cdata.cfg.exp_gain.gain,
			cdata.cfg.exp_gain.line);
		break;
	case CFG_SET_PICT_EXP_GAIN:
		rc = imx072_set_pict_exp_gain(
			cdata.cfg.exp_gain.gain,
			cdata.cfg.exp_gain.line);
		break;
	case CFG_SET_MODE:
		rc = imx072_set_sensor_mode(cdata.mode, cdata.rs);
		break;
	case CFG_PWR_DOWN:
		rc = imx072_power_down();
		break;
	case CFG_MOVE_FOCUS:
		break;
	case CFG_SET_DEFAULT_FOCUS:
		break;
	case CFG_GET_AF_MAX_STEPS:
		cdata.max_steps = IMX072_TOTAL_STEPS_NEAR_TO_FAR;
		if (copy_to_user((void *)argp,
			&cdata,
			sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;
	case CFG_SET_EFFECT:
		break;
	case CFG_SEND_WB_INFO:
		rc = imx072_send_wb_info(
			&(cdata.cfg.wb_info));
	break;
	case CFG_SENSOR_INIT:
		rc = imx072_mode_init(cdata.mode,
				cdata.cfg.init_info);
	break;
	case CFG_SET_LENS_SHADING:
		break;
	default:
		rc = -EFAULT;
		break;
	}

	mutex_unlock(&imx072_mut);

	return rc;
}

static int imx072_sensor_release(void)
{
	int rc = -EBADF;
	mutex_lock(&imx072_mut);
	imx072_power_down();
	gpio_set_value_cansleep(imx072_ctrl->sensordata->sensor_reset, 0);
	msleep(20);
	gpio_free(imx072_ctrl->sensordata->sensor_reset);
	kfree(imx072_ctrl);
	imx072_ctrl = NULL;
	pr_err("imx072_release completed\n");
	mutex_unlock(&imx072_mut);

	return rc;
}

static int imx072_sensor_probe(const struct msm_camera_sensor_info *info,
		struct msm_sensor_ctrl *s)
{
	int rc = 0;
	rc = i2c_add_driver(&imx072_i2c_driver);
	if (rc < 0 || imx072_client == NULL) {
		rc = -ENOTSUPP;
		pr_err("I2C add driver failed");
		goto probe_fail;
	}
	msm_camio_clk_rate_set(IMX072_MASTER_CLK_RATE);
	rc = imx072_probe_init_sensor(info);
	if (rc < 0)
		goto probe_fail;
	s->s_init = imx072_sensor_open_init;
	s->s_release = imx072_sensor_release;
	s->s_config  = imx072_sensor_config;
	s->s_mount_angle = 0;

	gpio_set_value_cansleep(info->sensor_reset, 0);
	imx072_probe_init_done(info);
	pr_info("imx072_sensor_probe : SUCCESS\n");
	return rc;

probe_fail:
	pr_err("imx072_sensor_probe: SENSOR PROBE FAILS!\n");
	return rc;
}

static int __imx072_probe(struct platform_device *pdev)
{
	return msm_camera_drv_start(pdev, imx072_sensor_probe);
}

static struct platform_driver msm_camera_driver = {
	.probe = __imx072_probe,
	.driver = {
		.name = "msm_camera_imx072",
		.owner = THIS_MODULE,
	},
};

static int __init imx072_init(void)
{
	return platform_driver_register(&msm_camera_driver);
}

module_init(imx072_init);
void imx072_exit(void)
{
	i2c_del_driver(&imx072_i2c_driver);
}
MODULE_DESCRIPTION("Aptina 8 MP Bayer sensor driver");
MODULE_LICENSE("GPL v2");

#ifdef CONFIG_DEBUG_FS
static bool streaming = 1;

static int cam_debug_stream_set(void *data, u64 val)
{
	int rc = 0;

	if (val) {
		imx072_start_stream();
		streaming = 1;
	} else {
		imx072_stop_stream();
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

	cam_dir = debugfs_create_dir("imx072", debugfs_base);
	if (!cam_dir)
		return -ENOMEM;

	if (!debugfs_create_file("stream", S_IRUGO | S_IWUSR, cam_dir,
							 NULL, &cam_stream))
		return -ENOMEM;
	return 0;
}
#endif
