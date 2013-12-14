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

/************************
****camera_test.h********
*************************/
#ifndef _MSM_CAMERA_TEST_H
#define _MSM_CAMERA_TEST_H

#include <linux/kthread.h>
#include <linux/sched.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/msm_mdp.h>
#include <media/msm_camera.h>
#include <media/msm_isp.h>
#include <media/videobuf2-core.h>
#include "csi/msm_csiphy.h"
#include "csi/msm_csid.h"
#include "sensors/msm_sensor_common.h"
#include "msm.h"
#include "vfe/msm_vfe32.h"
#include "../../../video/msm/msm_fb.h"
#define PREVIEW_BUFFER_COUNT 3
int PREVIEW_HEIGHT = 507;
int PREVIEW_WIDTH = 720;
int PREVIEW_BUFFER_LENGTH;
int PREVIEW_BUFFER_SIZE;
enum camera_preview_buffer_state {
	CAMERA_PREVIEW_BUFFER_STATE_UNUSED,
	CAMERA_PREVIEW_BUFFER_STATE_INITIALIZED, /* free, can be used */
	CAMERA_PREVIEW_BUFFER_STATE_QUEUED_TO_PINGPONG,/* used by ping pong */
	CAMERA_PREVIEW_BUFFER_STATE_DEQUEUED_FROM_PINGPONG, /* preview data dequeue from ping pong */
	CAMERA_PREVIEW_BUFFER_STATE_QUEUED_TO_DISPLAY,/* used by display */
	CAMERA_PREVIEW_BUFFER_STATE_DEQUEUED_FROM_DISPLAY /* dequeued by display, shall be set to initialized and re-use */
};
/* use msm_free_buf, for easily called by ping pong buffer configuration */
struct preview_mem {
	struct msm_free_buf cam_preview;
	struct list_head list;
	enum camera_preview_buffer_state state;
	uint32_t offset;
};
struct msm_camera_preview_data {
	struct ion_client *ion_client;
	struct ion_handle *ion_handle;
	struct preview_mem preview_buffer[PREVIEW_BUFFER_COUNT];
	struct list_head camera_preview_list;
	struct mutex preview_lock;
};
/* add for axi bus configuration */
#define AXI_MAX_WM_NUM	      7
#define MAX_OUTPUT_SUPPORTED 5 /* Max Number of scalers in VFE */
struct output_ch {
	int32_t   ch0:16;
	int32_t   ch1:16;
	int32_t   ch2:16;
	int32_t   /* reserved */ : 16;
	int32_t   inst_handle:32;
} __attribute__((packed, aligned(4)));

typedef struct VFE_PixelIfCfg {
	uint32_t inputSelect:2;
	uint32_t rdiEnable:1;
	uint32_t /* reserved */ : 1;
	uint32_t rdiM0Select:4;
	uint32_t rdiM1Select:4;
	uint32_t rdiM2Select:4;
	uint32_t rdiM0FrameBasedEnable:1;
	uint32_t rdiM1FrameBasedEnable:1;
	uint32_t rdiM2FrameBasedEnable:1;
	uint32_t /* reserved */ : 1;
	uint32_t rdiFrameSkip:4;
	uint32_t rdiFrameSkipEnable:1;
	uint32_t /* reserved */ : 3;
	uint32_t rdiStreamSelect:4;
} __attribute__((packed, aligned(4))) VFE_PixelIfCfg;

typedef struct VFE_RdiCfg0 {
	uint32_t /*reserved*/ : 2;
	uint32_t rdiEnable:1;
	uint32_t /* reserved */ : 1;
	uint32_t rdiM3Select:4;
	uint32_t rdiM4Select:4;
	uint32_t rdiM5Select:4;
	uint32_t /* reserved */ : 4;
	uint32_t rdiFrameSkip:4;
	uint32_t rdiFrameSkipEnable:1;
	uint32_t /*reserved*/ : 3;
	uint32_t rdiStreamSelect1:4;
} __attribute__((packed, aligned(4))) VFE_RdiCfg0;

typedef struct VFE_RdiCfg1 {
	uint32_t /*reserved*/ : 2;
	uint32_t rdiEnable:1;
	uint32_t /* reserved */ : 1;
	uint32_t rdiM6Select:4;
	uint32_t rdiM7Select:4;
	uint32_t rdiM8Select:4;
	uint32_t /* reserved */ : 4;
	uint32_t rdiFrameSkip:4;
	uint32_t rdiFrameSkipEnable:1;
	uint32_t /*reserved*/ : 3;
	uint32_t rdiStreamSelect2:4;
} __attribute__((packed, aligned(4))) VFE_RdiCfg1;


typedef struct output_path {
	struct output_ch out0;
	struct output_ch out1;
	struct output_ch out2;
	struct output_ch out3;
	struct output_ch out4;
} __attribute__((packed, aligned(4)))output_path;

typedef struct vfe_wm_config {
	uint32_t wmEnable:1;
	uint32_t /* reserved */ : 31;
	uint32_t busPingAddr:32;
	uint32_t busPongAddr:32;
	uint32_t busUbDepth:10;
	uint32_t /* reserved */ : 6;
	uint32_t busUbOffset:10;
	uint32_t /* reserved */ : 6;
	uint32_t buslinesPerImage:12;
	uint32_t /* reserved */ : 4;
	uint32_t busdwordsPerLine:10;
	uint32_t /* reserved */ : 6;
	uint32_t busburstLength:2;
	uint32_t /* reserved */ : 2;
	uint32_t busbufferNumRows:12;
	uint32_t busrowIncrement:13;
	uint32_t /* reserved */ : 3;
} __attribute__((packed, aligned(4))) vfe_wm_config;

typedef struct VFE_AXIOutputConfigCmdType {
	uint32_t busioFormat:32;
	uint32_t busCmd:32;
	/* busCfg's 31st bit config is as following
	* 7x30: OOO_WRITE_ENABLE  ---  Not Used
	* 8x60: Reserved          ---  Not Used
	* 8960: IMEM Mode disable (For Inline-JPEG only)
	*/
	uint32_t busCfg:32;
	uint32_t xbarCfg0:32;
	uint32_t xbarCfg1:32;
	uint32_t busWrSkipCfg:32;
	vfe_wm_config wm[AXI_MAX_WM_NUM];
	output_path outpath;
	VFE_PixelIfCfg pixelIfCfg;
	VFE_RdiCfg0 rdiCfg0;
	VFE_RdiCfg1 rdiCfg1;
} __attribute__((packed, aligned(4))) VFE_AXIOutputConfigCmdType;

extern struct csiphy_device *lsh_csiphy_dev;
extern struct csid_device *lsh_csid_dev;
extern struct ispif_device *lsh_ispif;
extern struct msm_sensor_ctrl_t mt9m114_s_ctrl;
extern struct msm_sensor_ctrl_t adv7481_s_ctrl;
extern struct v4l2_subdev *lsh_axi_ctrl;
extern struct axi_ctrl_t *my_axi_ctrl;
extern struct msm_fb_data_type *cam_preview_mfd;
extern struct fb_info *cam_preview_fbi;
extern struct mdp4_overlay_pipe *pipe;
extern  int msm_ispif_init(struct ispif_device *ispif, const uint32_t *csid_version);
extern int msm_ispif_config(struct ispif_device *ispif, struct msm_ispif_params_list *params_list);
extern int msm_csiphy_init(struct csiphy_device *csiphy_dev);
extern int msm_csiphy_lane_config(struct csiphy_device *csiphy_dev, struct msm_camera_csiphy_params *csiphy_params);
extern int msm_csid_init(struct csid_device *csid_dev, uint32_t *csid_version);
extern int msm_csid_config(struct csid_device *csid_dev, struct msm_camera_csid_params *csid_params);
extern int32_t msm_sensor_power_up(struct msm_sensor_ctrl_t *s_ctrl);
extern int32_t msm_sensor_power_up_adv7481(struct msm_sensor_ctrl_t *s_ctrl);
extern int32_t msm_sensor_power_down_adv7481(struct msm_sensor_ctrl_t *s_ctrl);
extern int32_t msm_sensor_mode_init(struct msm_sensor_ctrl_t *s_ctrl, int mode, struct sensor_init_cfg *init_info);
extern int32_t msm_sensor_set_sensor_mode(struct msm_sensor_ctrl_t *s_ctrl,
	int mode, int res);
extern void msm_sensor_start_stream(struct msm_sensor_ctrl_t *s_ctrl);
extern void msm_sensor_stop_stream(struct msm_sensor_ctrl_t *s_ctrl);
extern int32_t msm_sensor_power_down(struct msm_sensor_ctrl_t *s_ctrl);
extern void axi_start(struct msm_cam_media_controller *pmctl, struct axi_ctrl_t *axi_ctrl, struct msm_camera_vfe_params_t vfe_params);
extern int vfe32_config_axi(struct axi_ctrl_t *axi_ctrl, int mode, uint32_t *ao);
extern int msm_axi_subdev_init_rdi_only(struct v4l2_subdev *sd,
	uint8_t dual_enabled, struct msm_sensor_ctrl_t *s_ctrl);
extern int msm_ispif_subdev_video_s_stream_rdi_only(struct ispif_device *ispif, int enable);
extern int axi_reset_rdi1_only(struct axi_ctrl_t *axi_ctrl,	struct msm_camera_vfe_params_t vfe_params);
extern int configure_pingpong_buffers_rdi1_only(int id, int path, struct axi_ctrl_t *axi_ctrl);
extern void msm_axi_process_irq(struct v4l2_subdev *sd, void *arg);
extern void axi_start_rdi1_only(struct axi_ctrl_t *axi_ctrl, struct msm_sensor_ctrl_t *s_ctrl);
extern int vfe32_config_axi_rdi_only(struct axi_ctrl_t *axi_ctrl, int mode, uint32_t *ao);
extern void axi_stop_rdi1_only(struct axi_ctrl_t *axi_ctrl);
extern void mdp4_v4l2_overlay_clear(struct mdp4_overlay_pipe *pipe);
extern int msm_fb_v4l2_enable(struct mdp_overlay *req, bool enable, struct mdp4_overlay_pipe **ppipe);
extern int msm_fb_v4l2_update(struct mdp4_overlay_pipe *ppipe,
	unsigned long srcp0_addr, unsigned long srcp0_size,
	unsigned long srcp1_addr, unsigned long srcp1_size,
	unsigned long srcp2_addr, unsigned long srcp2_size);
extern int msm_fb_blank_sub_for_camera_preview(int blank_mode, struct fb_info *info,
				bool op_enable);
extern int mdp_bus_scale_update_request(u64 ab_p0, u64 ib_p0, u64 ab_p1, u64 ib_p1);
extern int msm_csid_release(struct csid_device *csid_dev);
extern int msm_csiphy_release(struct csiphy_device *csiphy_dev, void *arg);
extern void msm_ispif_release(struct ispif_device *ispif);
extern void msm_axi_subdev_release_rdi_only(struct v4l2_subdev *sd, struct msm_sensor_ctrl_t *s_ctrl);
extern int msm_axi_subdev_s_crystal_freq(struct v4l2_subdev *sd,
						u32 freq, u32 flags);
extern void *ion_handle_kmap_get(struct ion_handle *handle);
extern void msm_sensor_detect_std_adv7481(struct msm_sensor_ctrl_t *s_ctrl);

extern void msm_sensor_start_stream_adv7481(struct msm_sensor_ctrl_t *s_ctrl);

void preview_set_data_pipeline(void);
struct preview_mem *preview_buffer_find_free_for_ping_pong(void);
struct preview_mem *preview_buffer_find_free_for_ping_pong_mdp(void);
static void preview_configure_bufs(void);
static int camera_test(void);

#endif /* _MSM_CAMERA_TEST_H */
