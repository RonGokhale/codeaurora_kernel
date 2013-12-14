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

#include <linux/msm_ion.h>
#include <linux/mutex.h>
#include <linux/io.h>
#include <asm/mach-types.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/swab.h>
#include "camera_test.h"

static void *k_addr;
static int alloc_overlay_pipe_flag;
static struct mdp_overlay overlay_req;
static int rc = -ENOIOCTLCMD;
static uint8_t dual_enabled;
static struct msm_sensor_ctrl_t *s_ctrl;
static struct msm_ispif_params_list params_list;
struct sensor_init_cfg *init_info;
static uint32_t csid_version;
static int rdi1_irq_count;
extern struct msm_fb_data_type *cam_preview_mfd;
struct mdp4_overlay_pipe *pipe;
static struct VFE_AXIOutputConfigCmdType vfe_axi_cmd_para;
static struct msm_camera_vfe_params_t vfe_para;

struct msm_camera_csiphy_params csiphy_params = {
	.lane_cnt = 1,
	.settle_cnt = 0x14,
	.lane_mask = 0x1,
 };
struct msm_camera_csi_lane_params csi_lane_params = {
	.csi_lane_assign = 0xE4,
	.csi_lane_mask = 0x1,
	.csi_phy_sel = 0,
};
struct msm_camera_csid_vc_cfg mt9m114_cid_cfg1[] = {
	{0, CSI_YUV422_8, CSI_DECODE_8BIT},
	{1, CSI_EMBED_DATA, CSI_DECODE_8BIT},
};
struct msm_camera_csid_params csid_params = {
	.lane_cnt = 1,
	.lane_assign = 0xE4,
	.lut_params = {
		.num_cid = ARRAY_SIZE(mt9m114_cid_cfg1),
		.vc_cfg = mt9m114_cid_cfg1,
	},
};
static struct msm_camera_preview_data preview_data;
static struct msm_camera_preview_data preview_data_mdp;
static int camera_preview_exit = 1;
int camera_preview_ready;
static struct ion_handle *display_handle;
int axi_vfe_config_cmd_para(VFE_AXIOutputConfigCmdType *cmd)
{
	/* configure the axi bus parameters here */
	int config;
	int ch_wm;
	int axiOutputPpw;
	int ch0_wm;
	int imageWidth;
	int imageHeight;
	int burstLength;
	imageWidth = PREVIEW_WIDTH*2;
	imageHeight = PREVIEW_HEIGHT;
	cmd->busioFormat = 0;
	cmd->busCmd = 0;
	cmd->busCfg = 0x2aaa771;
	cmd->busWrSkipCfg  = 0;
	cmd->rdiCfg0.rdiEnable = 0x1;
	cmd->rdiCfg0.rdiStreamSelect1 = 0x3;
	cmd->rdiCfg0.rdiM3Select = 0x0;
	config = 0x01 | (0x06 << 5);
	ch_wm = 3;
	cmd->xbarCfg0 = 0;
	cmd->xbarCfg0 = (cmd->xbarCfg0) & ~(0x000000FF << (8 * (ch_wm % 4)));
	cmd->xbarCfg0 = (cmd->xbarCfg0) | (config << (8 * (ch_wm % 4)));
	ch0_wm = 3;
	axiOutputPpw = 8;
	burstLength = 3;
	cmd->wm[ch0_wm].busdwordsPerLine = 89;
	cmd->wm[ch0_wm].busrowIncrement =
		(imageWidth+(axiOutputPpw-1))/(axiOutputPpw);
	cmd->wm[ch0_wm].buslinesPerImage = imageHeight - 1;
	cmd->wm[ch0_wm].busbufferNumRows = imageHeight - 1;
	cmd->wm[ch0_wm].busburstLength = burstLength;
	cmd->wm[ch0_wm].busUbOffset = 0;
	cmd->wm[ch0_wm].busUbDepth = 0x3F0;

	return 0;
}

void preview_set_data_pipeline()
{
	int ispif_stream_enable;
	u32 freq = 128000000;
	u32 flags = 0;
	dual_enabled = 0; /* set according to log */
	/* power on and enable  clock for vfe and axi, before csi */
	rc = msm_axi_subdev_init_rdi_only(lsh_axi_ctrl, dual_enabled, s_ctrl);
	msm_axi_subdev_s_crystal_freq(lsh_axi_ctrl, freq,  flags);
	csid_version = 1;
	rc = msm_csid_init(lsh_csid_dev, &csid_version); /* CSID_INIT */
	rc = msm_csiphy_init(lsh_csiphy_dev); /* CSIPHY_INIT */
	rc = msm_ispif_init(lsh_ispif, &csid_version); /* ISPIF_INIT */
	params_list.params[0].intftype =  RDI1; /* RDI1 */
	params_list.params[0].csid = 1;
	params_list.params[0].vfe_intf =  VFE0;
	params_list.params[0].cid_mask = (1 << 0);
	params_list.len = 1;
	/* ISPIF_CFG,use csid 1,rdi1 */
	rc = msm_ispif_config(lsh_ispif, &params_list);
	rc = msm_csid_config(lsh_csid_dev, &csid_params); /* CSID_CFG */
	/* CSIPHY_CFG */
	rc = msm_csiphy_lane_config(lsh_csiphy_dev, &csiphy_params);
	/*
	TODO: Call ADV7481 sensor power up function once ADV7481 sensor driver
	code is added.
	*/

	ispif_stream_enable = 129;  /* configure to select RDI 1, VFE0 */
	msm_ispif_subdev_video_s_stream_rdi_only(lsh_ispif, ispif_stream_enable);
	pr_debug(" CAMERA_TEST: %s begin axi reset!!!\n", __func__);
	axi_reset_rdi1_only(my_axi_ctrl, vfe_para);
	pr_debug(" CAMERA_TEST: %s vfe32_config_axi now!!!\n", __func__);
	axi_vfe_config_cmd_para(&vfe_axi_cmd_para);
	rc = vfe32_config_axi_rdi_only(my_axi_ctrl, OUTPUT_TERT2, (uint32_t *)&vfe_axi_cmd_para);
}

void preview_buffer_alloc(void)
{
	int i, result;
	int offset = 0;
	int mem_len;
	unsigned long paddr;
	unsigned long display_paddr;
	int fd;

	memset(&preview_data, 0, sizeof(struct msm_camera_preview_data));
	pr_debug("%s\n", __func__);
	preview_data.ion_client = msm_ion_client_create(-1, "camera");
	if (IS_ERR_OR_NULL((void *)preview_data.ion_client)) {
		pr_err("%s: ION create client failed\n", __func__);
		goto err;
	}
	/* ION_CP_MM_HEAP_ID size is 0x7800000 */
	preview_data.ion_handle = ion_alloc(preview_data.ion_client,
	/* add 32 to align with 8 for physical address align to 8 */
				(PREVIEW_BUFFER_SIZE + 32),
				SZ_4K, (0x1 << ION_CP_MM_HEAP_ID | 0x1 << ION_IOMMU_HEAP_ID), 0);
	if (IS_ERR_OR_NULL((void *) preview_data.ion_handle)) {
		pr_err("%s: CAMERA_TEST: ION memory allocation failed\n", __func__);
		goto err_ion_client;
	}
	 k_addr = ion_handle_kmap_get(preview_data.ion_handle);
	result = ion_map_iommu(preview_data.ion_client, preview_data.ion_handle,
			CAMERA_DOMAIN, GEN_POOL, SZ_4K, 0,
			(unsigned long *)&paddr,
			(unsigned long *)&mem_len, 0, 1);
	if (result < 0) {
			pr_err("%s Could not get  address\n", __func__);
			goto err_ion_handle;
	}
		fd = ion_share_dma_buf(preview_data.ion_client, preview_data.ion_handle);

		display_handle = ion_import_dma_buf((cam_preview_mfd->iclient), fd);
		result = ion_map_iommu((cam_preview_mfd->iclient), display_handle,
			DISPLAY_READ_DOMAIN, GEN_POOL, SZ_4K, 0,
			(unsigned long *)&display_paddr,
			(unsigned long *)&mem_len, 0, 1);
	if (result < 0) {
		pr_err("%s Could not get  address\n", __func__);
		goto err_ion_handle;
	}

	pr_debug("CAMERA_TEST: %s ION allocate buffer finished, mem_len is %d\n",
			__func__, mem_len);
	paddr = ((paddr + 7) & 0xFFFFFFF8); /* to align with 8 */
	/* Make all phys point to the correct address */
	for (i = 0; i < PREVIEW_BUFFER_COUNT; i++) {
		/* Y plane */
		preview_data.preview_buffer[i].cam_preview.ch_paddr[0] =
			(uint32_t)(paddr + offset);
		/* add offset for display use */
		/* if paddr last byte is 5, we need to add 3 to align to 8 */
		preview_data.preview_buffer[i].offset = offset + (8 - (paddr & 7));
		offset += PREVIEW_BUFFER_LENGTH;
		/* this the offset from start address ion_handle, as align to 8	*/
	}
	offset = 0;
	display_paddr = ((display_paddr + 7) & 0xFFFFFFF8); /* to align with 8 */
	for (i = 0; i < PREVIEW_BUFFER_COUNT; i++) {
				/* Y plane */
				preview_data_mdp.preview_buffer[i].cam_preview.ch_paddr[0] =
					(uint32_t)(display_paddr + offset);
				/* add offset for display use */
				/* if paddr last byte is 5, we need to add 3 to align to 8 */
				preview_data_mdp.preview_buffer[i].offset =
					offset + (8 - (display_paddr & 7));
				offset += PREVIEW_BUFFER_LENGTH;
				/* this the offset from start address ion_handle, as align to 8 */
				pr_debug("%s:  mdp phys addr = 0x%x\n",
					__func__,
					preview_data_mdp.preview_buffer[i].cam_preview.ch_paddr[0]);
				pr_debug("%s: mdp  buffer offset = 0x%x\n",
					__func__,
					preview_data_mdp.preview_buffer[i].offset);
	}
	pr_debug("CAMERA_TEST :%s ION allocate buffer finished\n", __func__);
	return;
err_ion_handle:
	ion_free(preview_data.ion_client, preview_data.ion_handle);
err_ion_client:
	ion_client_destroy(preview_data.ion_client);
err:
	return;
}

int preview_buffer_init(void)
{
	int i ;
	/* initialized the list head and add all nodes */
	pr_debug("CAMERA_TEST :%s begin to setup buffer list\n", __func__);

	INIT_LIST_HEAD(&preview_data.camera_preview_list);

	for (i = 0; i < PREVIEW_BUFFER_COUNT; i++) {
		list_add(&(preview_data.preview_buffer[i].list), &(preview_data.camera_preview_list));
		preview_data.preview_buffer[i].state = CAMERA_PREVIEW_BUFFER_STATE_INITIALIZED;
	}

	INIT_LIST_HEAD(&preview_data_mdp.camera_preview_list);

	for (i = 0; i < PREVIEW_BUFFER_COUNT; i++) {
		list_add(&(preview_data_mdp.preview_buffer[i].list),
				&(preview_data_mdp.camera_preview_list));
		preview_data_mdp.preview_buffer[i].state =
				CAMERA_PREVIEW_BUFFER_STATE_INITIALIZED;
	}

	pr_debug("CAMERA_TEST :%s setup buffer list\n", __func__);
	return 0;
}

void preview_buffer_return(uint32_t paddr)
{
	/* search in the 4 buffer list, if the physical address matches,
	   set the buffer state as initialized and can be reused */
	int i;
	for (i = 0; i < PREVIEW_BUFFER_COUNT; i++) {
		if (preview_data.preview_buffer[i].cam_preview.ch_paddr[0] ==
				(uint32_t)(paddr)) {
			preview_data.preview_buffer[i].state =
					CAMERA_PREVIEW_BUFFER_STATE_INITIALIZED;
			pr_debug("CAMERA_TEST: %s find the buffer and set \
			the buf state as initialized, phys addr = 0x%x\n",
			__func__,
			preview_data.preview_buffer[i].cam_preview.ch_paddr[0]);
			break;
		}
	}
	return;
}

void preview_buffer_return_mdp(uint32_t paddr)
{
	/* search in the 4 buffer list, if the physical address matches,
		set the buffer state as initialized and can be reused */
	int i;
	for (i = 0; i < PREVIEW_BUFFER_COUNT; i++) {
		if (preview_data_mdp.preview_buffer[i].cam_preview.ch_paddr[0] == (uint32_t)(paddr)) {
			preview_data_mdp.preview_buffer[i].state = CAMERA_PREVIEW_BUFFER_STATE_INITIALIZED;
			pr_debug("CAMERA_TEST %s: find the buffer and set the \
				buf state as initialized, phys addr = 0x%x\n",
				__func__,
				preview_data.preview_buffer[i].\
				cam_preview.ch_paddr[0]);
			break;
		}
	}
	return;
}

struct preview_mem *preview_find_buffer_by_paddr(uint32_t paddr)
{
	/* search in the 4 buffer list, if the physical address matches,
		set the buffer state as initialized and can be reused */
	int i;
	struct preview_mem *buf = NULL;
	for (i = 0; i < PREVIEW_BUFFER_COUNT; i++) {
		if (preview_data.preview_buffer[i].cam_preview.ch_paddr[0] == (uint32_t)(paddr)) {
			/* or we can return offset here directly we need not to define buffer here */
			buf = &(preview_data.preview_buffer[i]);
			break;
			}
		}
	return buf;
}

int preview_find_buffer_index_by_paddr(uint32_t paddr)
{
	/* search in the 4 buffer list, if the physical address matches,
	   set the buffer state as initialized and can be reused */
	int i;
	int index = 0;
	for (i = 0; i < PREVIEW_BUFFER_COUNT; i++)	{
		if (preview_data.preview_buffer[i].cam_preview.ch_paddr[0] == (uint32_t)(paddr)) {
			index = i;
			break;
		}
	}
	return index;
}

int preview_find_buffer_index_by_paddr_mdp(uint32_t paddr)
{
	/* search in the 4 buffer list, if the physical address matches,
	   set the buffer state as initialized and can be reused */
	int i;
	int index = 0;
	for (i = 0; i < PREVIEW_BUFFER_COUNT; i++) {
		if (preview_data_mdp.preview_buffer[i].cam_preview.ch_paddr[0] == (uint32_t)(paddr)) {
			index = i;
			break;
		}
	}
	return index;
}

struct preview_mem *preview_find_buffer_by_paddr_mdp(uint32_t paddr)
{
	/* search in the 4 buffer list, if the physical address matches,
		set the buffer state as initialized and can be reused */
	int i;
	struct preview_mem *buf = NULL;

	for (i = 0; i < PREVIEW_BUFFER_COUNT; i++) {
		if (preview_data_mdp.preview_buffer[i].cam_preview.ch_paddr[0] == (uint32_t)(paddr)) {
			/* or we can return offset here directly we need not to define buffer here */
			buf = &(preview_data_mdp.preview_buffer[i]);
			break;
		}
	}
	return buf;
}

struct preview_mem *preview_buffer_find_free_for_ping_pong()
{
	int ret = 0;
	struct preview_mem *buf = NULL;

	pr_debug("CAMERA_TEST %s begin to find free buffer for ping pong\n",
			__func__);
	list_for_each_entry(buf, &(preview_data.camera_preview_list), list) {
		if (buf->state == CAMERA_PREVIEW_BUFFER_STATE_INITIALIZED)
			break;
		ret = 1;
	}
	if (IS_ERR_OR_NULL((void *) buf)) {
		pr_err("%s: CAMERA_TEST: can not find free buffer\n", __func__);
	}

	pr_debug("CAMERA_TEST %s free buff find finished, physical addr is %x\n",
		__func__, buf->cam_preview.ch_paddr[0]);
	return buf;
}

struct preview_mem *preview_buffer_find_free_for_ping_pong_mdp()
{
	int ret = 0;
	struct preview_mem *buf = NULL;

	pr_debug("CAMERA_TEST %s begin to find free buffer for ping pong\n",
			__func__);
	list_for_each_entry(buf, &(preview_data_mdp.camera_preview_list), list) {
		if (buf->state == CAMERA_PREVIEW_BUFFER_STATE_INITIALIZED)
			break;
		ret = 1;
	}
	if (IS_ERR_OR_NULL((void *) buf)) {
		pr_err("%s: CAMERA_TEST: can not find free buffer\n",
			__func__);
	}

	pr_debug("CAMERA_TEST %s free buff find finished, physical addr is %x\n",
		__func__, buf->cam_preview.ch_paddr[0]);
	return buf;
}

static int preview_configure_ping_pong_buffer(struct preview_mem *ping_buf,
		struct preview_mem *pong_buf, struct preview_mem *free_buffer)
{
	int message_id;
	int path;
	message_id = 0;
	path = VFE_MSG_OUTPUT_TERTIARY2;
	pr_debug("%s CAMERA_TEST: ping/pong configure enter!!, init the address\n",
			__func__);
	/* get three buffer from the list, use them to configure
	ping pong buffer and free buffer */
	my_axi_ctrl->share_ctrl->outpath.out3.ping.ch_paddr[0] =
			ping_buf->cam_preview.ch_paddr[0];
	my_axi_ctrl->share_ctrl->outpath.out3.pong.ch_paddr[0] =
			pong_buf->cam_preview.ch_paddr[0];
	my_axi_ctrl->share_ctrl->outpath.out3.free_buf.ch_paddr[0] =
			free_buffer->cam_preview.ch_paddr[0];
	pr_debug("%s CAMERA_TEST: ping/pong configure enter~~~\n",
				__func__);
	configure_pingpong_buffers_rdi1_only(message_id, path, my_axi_ctrl);
	return 0;
}

static int preview_buffer_free(void)
{
	int ret;
	ion_unmap_iommu(preview_data.ion_client, preview_data.ion_handle,
			CAMERA_DOMAIN, GEN_POOL);
	ion_unmap_iommu((cam_preview_mfd->iclient), display_handle,
			DISPLAY_READ_DOMAIN, GEN_POOL);
	ion_free(preview_data.ion_client, preview_data.ion_handle);
	ion_client_destroy(preview_data.ion_client);
	return ret = 0;
}

static void preview_configure_bufs()
{
	/* step 1, alloc ION buffer */
	pr_debug(" CAMERA_TEST: %s preview buffer allocate\n", __func__);
	preview_buffer_alloc();

	/*step2, buffer linked list */
	preview_buffer_init();
}

static void set_overlay_init(struct mdp_overlay *overlay)
{
	/*
	set_overlay;
	TODO: re-check and change below values based on Auto Platform # 1
	board's display
	*/
	overlay->id = MSMFB_NEW_REQUEST;
	overlay->src.width  = PREVIEW_WIDTH*2;
	overlay->src.height = PREVIEW_HEIGHT;
	overlay->src.format = MDP_YCRYCB_H2V1;/* MDP_Y_CB_CR_H2V2 */
	overlay->src_rect.x = 0;
	overlay->src_rect.y = 0;
	overlay->src_rect.w = PREVIEW_WIDTH;
	overlay->src_rect.h = PREVIEW_HEIGHT/2;
	overlay->dst_rect.x = 0;
	overlay->dst_rect.y = 0;
	overlay->dst_rect.w = 800;
	overlay->dst_rect.h = 480;
	overlay->z_order = 2; /* 0; FB_OVERLAY_VID_0; */
	overlay->alpha = MDP_ALPHA_NOP;
	overlay->transp_mask = MDP_TRANSP_NOP; /* 0xF81F */
	overlay->flags = 0;
	overlay->is_fg = 1;
	return;
}

void format_convert(int index)
{
	int i;
	int8 *data;
	data = (int8 *)k_addr;
	data += PREVIEW_WIDTH*PREVIEW_HEIGHT*2*index;
	for (i = 0; i < PREVIEW_WIDTH*PREVIEW_HEIGHT/4 ; i++)
		__swab16s((uint16 *)(data+4*i));
}

void vfe32_process_output_path_irq_rdi1_only(struct axi_ctrl_t *axi_ctrl)
{
	uint32_t ping_pong;
	uint32_t ch0_paddr = 0;
	uint32_t mdp_addr = 0;
	/* this must be rdi image output. */
	struct preview_mem *free_buf = NULL;
	int buffer_index;

	pr_debug(" CAMERA_TEST %s enter into rdi1 irq now, rdi1_irq_count is %d!!!\n",
			__func__, rdi1_irq_count);
	rdi1_irq_count++;
	/* added test code, to stop axi for debug */
	/*RDI1*/
	if (axi_ctrl->share_ctrl->operation_mode & VFE_OUTPUTS_RDI1) {
		/* check if there is free buffer, if there is, then put it to ping pong */
		free_buf = preview_buffer_find_free_for_ping_pong();
		if (axi_ctrl->share_ctrl->outpath.out3.capture_cnt > 0 || free_buf) {
			ping_pong = msm_camera_io_r(axi_ctrl->share_ctrl->vfebase +
				VFE_BUS_PING_PONG_STATUS);
			/* Y channel */
			/* read preview data from pingpong buffer */
			ch0_paddr = vfe32_get_ch_addr(ping_pong,
				axi_ctrl->share_ctrl->vfebase,
				axi_ctrl->share_ctrl->outpath.out3.ch0);
			/* convert the camera domain address to mdp domain address */
			buffer_index = preview_find_buffer_index_by_paddr(ch0_paddr);
			format_convert(buffer_index);
			mdp_addr =
			preview_data_mdp.preview_buffer[buffer_index].\
				cam_preview.ch_paddr[0];
			pr_debug("%s CAMERA_TEST: mdp domain ch0 = 0x%x\n",
			__func__, mdp_addr);
			msm_fb_v4l2_update(pipe, mdp_addr, 0, 0, 0, 0, 0);
			camera_preview_ready = 1;
			if (free_buf) {
				vfe32_put_ch_addr(ping_pong,
					axi_ctrl->share_ctrl->vfebase,
					axi_ctrl->share_ctrl->outpath.out3.ch0,
					free_buf->cam_preview.ch_paddr[0]);
				/* shall add function to change the buffer state
				to be queued to ping pong
				preview_buffer_update_status_to_pingpong
				preview_free_buf() is needed? */
				free_buf->state =
				CAMERA_PREVIEW_BUFFER_STATE_QUEUED_TO_PINGPONG;
			}
			preview_buffer_return(ch0_paddr);
			if (axi_ctrl->share_ctrl->
					outpath.out3.capture_cnt == 1)
				axi_ctrl->share_ctrl->
				outpath.out3.capture_cnt = 0;
		} else {
			axi_ctrl->share_ctrl->outpath.out3.frame_drop_cnt++;
			pr_err("CAMERA_TEST %s: no free buffer for rdi1!\n",
					__func__);
		}
	}
}

static int camera_test()
{
	u64 mdp_max_bw_test = 2000000000;
	struct preview_mem *ping_buffer, *pong_buffer, *free_buffer;
	my_axi_ctrl->share_ctrl->current_mode = 4096;
	vfe_para.operation_mode = VFE_OUTPUTS_RDI1;
	camera_preview_exit = 0;
	/* Detect NTSC or PAL, get the preview width and height */

	/*
	TODO: Call ADV7481 sensor detect function ADV7481 sensor driver code
	is added
	*/
	PREVIEW_BUFFER_LENGTH =  PREVIEW_WIDTH * PREVIEW_HEIGHT*2;
	PREVIEW_BUFFER_SIZE  =  PREVIEW_BUFFER_COUNT * PREVIEW_BUFFER_LENGTH;
	preview_configure_bufs();
	preview_set_data_pipeline();
	/* step 1, find free buffer from the list, then use it to
	configure ping/pong */
	ping_buffer = preview_buffer_find_free_for_ping_pong();
	ping_buffer->state = CAMERA_PREVIEW_BUFFER_STATE_QUEUED_TO_PINGPONG;
	pong_buffer = preview_buffer_find_free_for_ping_pong();
	pong_buffer->state = CAMERA_PREVIEW_BUFFER_STATE_QUEUED_TO_PINGPONG;
	free_buffer = preview_buffer_find_free_for_ping_pong();
	pr_debug(" CAMERA_TEST: %s find ping pong buffer end!!!\n", __func__);

	/* step 2, configure the free buffer to ping pong buffer */
	preview_configure_ping_pong_buffer(ping_buffer, pong_buffer,
		free_buffer);
	if (msm_fb_blank_sub_for_camera_preview(FB_BLANK_UNBLANK,
		cam_preview_fbi, true))
		pr_err("CAMERA_TEST: %s: can't turn on display!\n", __func__);
	mdp_bus_scale_update_request(mdp_max_bw_test, mdp_max_bw_test,
		mdp_max_bw_test, mdp_max_bw_test);
	set_overlay_init(&overlay_req);
	alloc_overlay_pipe_flag = msm_fb_v4l2_enable(&overlay_req, 1, &pipe);
	if (alloc_overlay_pipe_flag != 0) {
		pr_err("CAMERA_TEST %s: msm_fb_v4l2_enable error!1\n",
			__func__);
		return 0;
	}
	my_axi_ctrl->share_ctrl->current_mode = 4096; /* BIT(12) */
	my_axi_ctrl->share_ctrl->operation_mode = 4096;
	axi_start_rdi1_only(my_axi_ctrl, s_ctrl);
	return 0;
}

int  init_camera_kthread(void)
{
	int ret;
	ret = camera_test();
	return 0;
}

void  exit_camera_kthread(void)
{
	pr_debug("CAMERA_TEST stop camera test thread!!!\n");
	axi_stop_rdi1_only(my_axi_ctrl);

	msm_csid_release(lsh_csid_dev);
	msm_csiphy_release(lsh_csiphy_dev, &csi_lane_params);
	msm_ispif_release(lsh_ispif);
	pr_debug(" CAMERA_TEST: begin axi release\n");
	msm_axi_subdev_release_rdi_only(lsh_axi_ctrl, s_ctrl);
	if (alloc_overlay_pipe_flag == 0) {
		pr_debug(" CAMERA_TEST: begin overlay pipe free\n");
		msm_fb_v4l2_enable(&overlay_req, 0, &pipe);
		pr_err("CAMERA_TEST %s: msm_fb_v4l2_enable free pipe !\n",
			__func__);
	}
	preview_buffer_free();
	camera_preview_exit = 1;
	camera_preview_ready = 0;
}
