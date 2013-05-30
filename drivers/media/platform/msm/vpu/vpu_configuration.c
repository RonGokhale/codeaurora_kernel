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

#include <linux/videodev2.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-ctrls.h>
#include <linux/stddef.h>

#include <media/msm_vpu.h>
#include "vpu_configuration.h"
#include "vpu_v4l2.h"
#include "vpu_ioctl_internal.h"
#include "vpu_translate.h"
#include "vpu_property.h"
#include "vpu_channel.h"
#include "vpu_debug.h"


/*
 * Private controls support
 */
enum vpu_ctrl_type {
	CTRL_TYPE_NONE = 0,
	CTRL_TYPE_BOOLEAN,
	CTRL_TYPE_SIMPLE,
	CTRL_TYPE_STANDARD,
	CTRL_TYPE_AUTO_MANUAL,
};

struct vpu_ctrl_desc;

#define call_op(d, op, args...) (((d)->op) ? ((d)->op(args)) : 0)
typedef int (*get_custom_size)(const void *api_arg);
typedef int (*bounds_check)(const struct vpu_ctrl_desc *desc,
		const void *api_arg);
typedef int (*translate_to_hfi) (const void *api_data, void *hfi_data);
typedef int (*translate_to_api) (const void *hfi_data, void *api_data);
typedef int (*pre_commit_handler)(struct vpu_dev_session *session,
		const void *api_arg);
typedef void (*post_commit_handler)(struct vpu_dev_session *session,
		const void *api_arg);
typedef int (*custom_set_function)(struct vpu_dev_session *session,
		const void *api_arg);
typedef int (*custom_get_function)(struct vpu_dev_session *session,
		void *ret_arg);

struct ctrl_bounds {
	s32 min;
	s32 max;
	s32 def;
	u32 step;
};

/**
 * struct vpu_ctrl_desc - struct to completely describe VPU controls
 *
 * @name: [Optional] Control name used for logging purposes.
 * @type: Can be set if control is a generic type to avoid setting other fields.
 *		If set, then size, hfi_size, bounds_check, trans_to_hfi,
 *		and trans_to_api are automatically set accordingly.
 * @hfi_id: [Required] Identifies corresponding HFI property ID.
 *		If value is zero, then control is not sent to HFI.
 * @size: [Required] Size of API struct used with this control.
 * @hfi_size: [Required] Size of data sent to HFI. Auto set if type is valid.
 * @cached: Set to 1 if control cannot be read from HFI and needs to be
 *		returned from cache instead.
 * @cache_offset: [Don't set]. Calculated offset where control cache is stored.
 * @bounds: [Optional] Set if control has bounds.
 * @bounds2: [Optional] A second set of bounds if needed.
 * @custom_size: Custom function to calculate hfi_size if it is dynamic.
 * @bound_check: [Optional] Function that performs the bound checking.
 *		Auto set if type is valid.
 * @trans_to_hfi: [Required for set control]. Translates from API to HFI.
 *		Auto set if type is valid.
 * @trans_to_api: [Required for get control]. Translates from HFI to API.
 *		Auto set if type is valid.
 * @pre_commit: [Optional] Special control handling before committing.
 * @post_commit: [Optional] Special control handling before committing.
 */
struct vpu_ctrl_desc {
	const char *name;
	enum vpu_ctrl_type type;
	u32 hfi_id;
	u32 size;
	u32 hfi_size;
	u32 cached;
	u32 cache_offset;
	struct ctrl_bounds bounds;
	struct ctrl_bounds bounds2;
	/* function pointers */
	get_custom_size custom_size;
	bounds_check bounds_check;
	translate_to_hfi trans_to_hfi;
	translate_to_api trans_to_api;
	pre_commit_handler pre_commit;
	post_commit_handler post_commit;
	custom_set_function set_function;
	custom_get_function get_function;
};


static int translate_ctrl_simple_to_hfi(const void *api_data, void *hfi_data)
{
	const __s32 *value = api_data;
	struct vpu_s_data_value *hfi = hfi_data;
	hfi->flags = PROP_FALSE;
	hfi->value = *value;
	return 0;
}

static int translate_ctrl_simple_to_api(const void *hfi_data, void *api_data)
{
	const struct vpu_s_data_value *hfi = hfi_data;
	__s32 *value = api_data;
	*value = hfi->value;
	return 0;
}

static int translate_ctrl_standard_to_hfi(const void *api_data, void *hfi_data)
{
	const struct vpu_ctrl_standard *api = api_data;
	struct vpu_s_data_value *hfi = hfi_data;

	hfi->flags = api->enable ? PROP_TRUE : PROP_FALSE;
	hfi->value = api->value;
	dprintk(VPU_INFO, "%s: flags = %d, value = %d\n",
			__func__, hfi->flags, hfi->value);
	return 0;
}

static int translate_ctrl_standard_to_api(const void *hfi_data, void *api_data)
{
	struct vpu_ctrl_standard *api = api_data;
	const struct vpu_s_data_value *hfi = hfi_data;

	api->enable = hfi->flags ? PROP_TRUE : PROP_FALSE;
	api->value = hfi->value;
	dprintk(VPU_INFO, "%s: enable = %d, value = %d\n",
			__func__, api->enable, api->value);
	return 0;
}

static int translate_ctrl_auto_manual_to_hfi(const void *api_data,
		void *hfi_data)
{
	const struct vpu_ctrl_auto_manual *api = api_data;
	struct vpu_s_data_value *hfi = hfi_data;

	if (!api->enable)
		hfi->flags = PROP_MODE_DISABLED;
	else if (api->auto_mode)
		hfi->flags = PROP_MODE_AUTO;
	else
		hfi->flags = PROP_MODE_MANUAL;
	hfi->value = api->value;
	dprintk(VPU_INFO, "%s: flags = %d, value = %d\n",
			__func__, hfi->flags, hfi->value);
	return 0;
}

static int translate_ctrl_auto_manual_to_api(const void *hfi_data,
		void *api_data)
{
	struct vpu_ctrl_auto_manual *api = api_data;
	const struct vpu_s_data_value *hfi = hfi_data;

	api->enable = PROP_TRUE;
	if (hfi->flags & PROP_MODE_AUTO)
		api->auto_mode = PROP_TRUE;
	else if (hfi->flags & PROP_MODE_MANUAL)
		api->auto_mode = PROP_FALSE;
	else
		api->enable = PROP_FALSE;
	api->value = hfi->value;
	dprintk(VPU_INFO, "%s: enable = %d, auto_mode = %d, value = %d\n",
			__func__, api->enable, api->auto_mode, api->value);
	return 0;
}

static int __check_bounds(const struct ctrl_bounds *bounds, s32 val)
{
	if (!bounds->min && !bounds->max && !bounds->step)
		return 0; /* no bounds specified */

	if (val < bounds->min || val > bounds->max || !bounds->step)
		return -EINVAL;

	val = val - bounds->min;
	if (((val / bounds->step) * bounds->step) != val)
		return -EINVAL;
	return 0;
}

static int simple_bounds_check(const struct vpu_ctrl_desc *desc,
		const void *api_arg)
{
	const __s32 *value = api_arg;
	return __check_bounds(&desc->bounds, *value);
}

static int standard_bounds_check(const struct vpu_ctrl_desc *desc,
		const void *api_arg)
{
	const struct vpu_ctrl_standard *arg = api_arg;

	if (arg->enable)
		return __check_bounds(&desc->bounds, arg->value);
	else
		return 0; /* ctrl not enabled, no need to check value */
}

static int auto_manual_bounds_check(const struct vpu_ctrl_desc *desc,
		const void *api_arg)
{
	const struct vpu_ctrl_auto_manual *arg = api_arg;
	const struct ctrl_bounds *bounds;

	if (arg->enable && arg->auto_mode)
		bounds = &desc->bounds;
	else if (arg->enable && !arg->auto_mode)
		bounds = &desc->bounds2;
	else
		return 0; /* ctrl not enabled, no need to check value */

	return __check_bounds(bounds, arg->value);
}


static int __config_nr_buffers(struct vpu_dev_session *session,
		const void *api_arg)
{
	struct vpu_controller *controller = session->controller;
	const struct vpu_ctrl_auto_manual *arg = api_arg;
	struct v4l2_pix_format_mplane *in_fmt;
	const struct vpu_format_desc *vpu_format;
	u32 buf_size = 0;
	int ret = 0, i;

	if (arg->enable) {
		/* Get NR buffer size. NR buffers are YUV422 format */
		vpu_format = query_supported_formats(PIXEL_FORMAT_YUYV);
		in_fmt = &session->port_info[INPUT_PORT].format;
		buf_size = get_bytesperline(in_fmt->width,
				vpu_format->plane[0].bitsperpixel, 0);
		buf_size *= in_fmt->height;

		for (i = 0; i < NUM_NR_BUFFERS; i++) {
			ret = vpu_iommu_alloc_kernel_mem(
				controller->nr_buffers[i], buf_size,
				session->port_info[INPUT_PORT].secure_content);
			if (ret) {
				dprintk(VPU_ERR,
				"Failed to allocate NR buffers size (%d)\n",
				buf_size);
				goto exit_nr_bufs;
			}
		}

		/* send buffers to maple */
		ret = vpu_hw_session_nr_buffer_config(session->id,
			vpu_iommu_get_addr(controller->nr_buffers[0]),
			vpu_iommu_get_addr(controller->nr_buffers[1]));
		if (ret) {
			dprintk(VPU_ERR, "Fail to send NR buffers to maple\n");
			goto exit_nr_bufs;
		}
	}

exit_nr_bufs:
	return ret;
}

static void __release_nr_buffers(struct vpu_dev_session *session,
		const void *api_arg)
{
	const struct vpu_ctrl_auto_manual *arg = api_arg;

	if (!arg->enable) {
		if (vpu_hw_session_nr_buffer_release(session->id))
			dprintk(VPU_WARN,
				"Critical!!! Failed to release NR buffers\n");
	}
}


static int translate_range_mapping_to_hfi(const void *api_data, void *hfi_data)
{
	const struct vpu_ctrl_range_map *api = api_data;
	struct vpu_prop_session_range_mapping *hfi = hfi_data;
	hfi->enabled = api->enable ? PROP_TRUE : PROP_FALSE;
	hfi->y_map_range = api->y_range;
	hfi->uv_map_range = api->uv_range;
	return 0;
}

static int translate_range_mapping_to_api(const void *hfi_data, void *api_data)
{
	const struct vpu_prop_session_range_mapping *hfi = hfi_data;
	struct vpu_ctrl_range_map *api = api_data;
	api->enable = hfi->enabled;
	api->y_range = hfi->y_map_range;
	api->uv_range = hfi->uv_map_range;
	return 0;
}


static int __act_reg_parm_get_size(const void *api_arg)
{
	const struct vpu_ctrl_active_region_param *arg = api_arg;

	int size = sizeof(struct vpu_prop_session_active_region_detect);
	/* add the size of exclusion regions */
	if (arg->num_exclusions <= VPU_ACTIVE_REGION_N_EXCLUSIONS)
		size += arg->num_exclusions * sizeof(struct rect);
	return size;
}

static int __act_reg_parm_bounds_check(const struct vpu_ctrl_desc *desc,
		const void *api_arg)
{
	const struct vpu_ctrl_active_region_param *arg = api_arg;
	return (arg->num_exclusions > VPU_ACTIVE_REGION_N_EXCLUSIONS) ?
			-EINVAL : 0;
}

static int translate_act_reg_parm_to_hfi(const void *api_data, void *hfi_data)
{
	const struct vpu_ctrl_active_region_param *api = api_data;
	struct vpu_prop_session_active_region_detect *hfi = hfi_data;
	struct rect *hfi_rect;
	int i = 0;

	hfi->enabled = api->enable ? PROP_TRUE : PROP_FALSE;
	hfi->num_exclusion_regions = api->num_exclusions;
	translate_roi_rect_to_hfi(
			(struct v4l2_rect *) &api->detection_region,
			&hfi->region_of_interest);

	hfi_rect = (struct rect *) (hfi_data + sizeof(*hfi));
	for (i = 0; i < api->num_exclusions; i++)
		translate_roi_rect_to_hfi(
				(struct v4l2_rect *) &api->excluded_regions[i],
				(hfi_rect++));
	return 0;
}


static int translate_act_reg_res_to_api(const void *hfi_data, void *api_data)
{
	const struct rect *hfi = hfi_data;
	struct v4l2_rect *api = api_data;
	translate_roi_rect_to_api((struct rect *) hfi, api);
	return 0;
}

int vpu_set_content_protection(struct vpu_dev_session *session,
		const void *api_arg)
{
	int i;
	__u32 cp = *((__u32 *)api_arg);
	if (cp)
		cp = 1;

	mutex_lock(&session->lock);
	for (i = 0; i < NUM_VPU_PORTS; i++) {
		/* only change CP status if all ports are in reset */
		if (session->io_client[i] != NULL) {
			mutex_unlock(&session->lock);
			return -EBUSY;
		}
	}
	for (i = 0; i < NUM_VPU_PORTS; i++)
		session->port_info[i].secure_content = cp;

	mutex_unlock(&session->lock);
	return 0;
}

int vpu_get_content_protection(struct vpu_dev_session *session, void *ret_arg)
{
	__u32 *cp = (__u32 *) ret_arg;
	*cp = session->port_info[INPUT_PORT].secure_content;
	return 0;
}

/* ctrl descriptions array */
static struct vpu_ctrl_desc ctrl_descriptors[VPU_CTRL_ID_MAX] = {
	[VPU_CTRL_NOISE_REDUCTION] = {
		.name = "Noise Reduction",
		.type = CTRL_TYPE_AUTO_MANUAL,
		.hfi_id = VPU_PROP_SESSION_NOISE_REDUCTION,
		.bounds = {
				.min = VPU_AUTO_NR_LEVEL_MIN,
				.max = VPU_AUTO_NR_LEVEL_MAX,
				.step = VPU_AUTO_NR_LEVEL_STEP,
				.def = VPU_AUTO_NR_LEVEL_MIN,
		},
		.bounds2 = {
				.min = VPU_NOISE_REDUCTION_LEVEL_MIN,
				.max = VPU_NOISE_REDUCTION_LEVEL_MAX,
				.step = VPU_NOISE_REDUCTION_STEP,
				.def = VPU_NOISE_REDUCTION_LEVEL_MIN,
		},
		.pre_commit = __config_nr_buffers,
		.post_commit = __release_nr_buffers,
	},
	[VPU_CTRL_IMAGE_ENHANCEMENT] = {
		.name = "Image Enhancement",
		.type = CTRL_TYPE_AUTO_MANUAL,
		.hfi_id = VPU_PROP_SESSION_IMAGE_ENHANCEMENT,
		.bounds = {
				.min = VPU_AUTO_IE_LEVEL_MIN,
				.max = VPU_AUTO_IE_LEVEL_MAX,
				.step = VPU_AUTO_IE_LEVEL_STEP,
				.def = VPU_AUTO_IE_LEVEL_MIN,
		},
		.bounds2 = {
				.min = VPU_IMAGE_ENHANCEMENT_LEVEL_MIN,
				.max = VPU_IMAGE_ENHANCEMENT_LEVEL_MAX,
				.step = VPU_IMAGE_ENHANCEMENT_STEP,
				.def = VPU_IMAGE_ENHANCEMENT_LEVEL_MIN,
		},
	},
	[VPU_CTRL_ANAMORPHIC_SCALING] = {
		.name = "Anamorphic Scaling",
		.type = CTRL_TYPE_STANDARD,
		.hfi_id = VPU_PROP_SESSION_ANAMORPHIC_SCALING,
		.bounds = {
				.min = VPU_ANAMORPHIC_SCALE_VALUE_MIN,
				.max = VPU_ANAMORPHIC_SCALE_VALUE_MAX,
				.step = VPU_ANAMORPHIC_SCALE_VALUE_STEP,
				.def = VPU_ANAMORPHIC_SCALE_VALUE_DEF,
		},
	},
	[VPU_CTRL_DIRECTIONAL_INTERPOLATION] = {
		.name = "Directional Interpolation",
		.type = CTRL_TYPE_STANDARD,
		.hfi_id = VPU_PROP_SESSION_DI,
		.bounds = {
				.min = VPU_DI_VALUE_MIN,
				.max = VPU_DI_VALUE_MAX,
				.step = 1,
				.def = VPU_DI_VALUE_MIN,
		},
	},
	[VPU_CTRL_RANGE_MAPPING] = {
		.name = "Y/UV Range Mapping",
		.hfi_id = VPU_PROP_SESSION_RANGE_MAPPING,
		.size = sizeof(struct vpu_ctrl_range_map),
		.hfi_size = sizeof(struct vpu_prop_session_range_mapping),
		.trans_to_hfi = translate_range_mapping_to_hfi,
		.trans_to_api = translate_range_mapping_to_api,
	},
	[VPU_CTRL_ACTIVE_REGION_PARAM] = {
		.name = "Active Region Detection Parameters",
		.hfi_id = VPU_PROP_SESSION_ACTIVE_REGION_DETECT,
		.size = sizeof(struct vpu_ctrl_active_region_param),
		.custom_size = __act_reg_parm_get_size,
		.cached = 1,
		.bounds_check = __act_reg_parm_bounds_check,
		.trans_to_hfi = translate_act_reg_parm_to_hfi,
	},
	[VPU_CTRL_ACTIVE_REGION_RESULT] = {
		.name = "Active Region Detection Result",
		.size = sizeof(struct v4l2_rect),
		.hfi_size = sizeof(struct rect),
		.cached = 1,
		.trans_to_api = translate_act_reg_res_to_api,
	},
	[VPU_CTRL_PRIORITY] = {
		.name = "Session Priority",
		.type = CTRL_TYPE_SIMPLE,
		.hfi_id = VPU_PROP_SESSION_PRIORITY,
	},
	[VPU_CTRL_CONTENT_PROTECTION] = {
		.name = "Content Protection",
		.set_function = vpu_set_content_protection,
		.get_function = vpu_get_content_protection,
	},
};

static u32 controller_cache_size;

/*
 * Initializes some control descriptors if type has been set.
 * And calculates required cache size and offests for controls caches
 */
void setup_vpu_controls(void)
{
	struct vpu_ctrl_desc *desc, *prev_desc;
	int i;
	controller_cache_size = 0;

	/* First entry is an invalid control. Ensure it's zeroed out */
	memset(&ctrl_descriptors[0], 0, sizeof(struct vpu_ctrl_desc));

	for (i = 1; i < VPU_CTRL_ID_MAX; i++) {
		desc = &ctrl_descriptors[i];
		prev_desc = &ctrl_descriptors[i - 1];

		switch (desc->type) {
		case CTRL_TYPE_BOOLEAN:
		case CTRL_TYPE_SIMPLE:
			desc->size = sizeof(__s32);
			desc->hfi_size = sizeof(struct vpu_s_data_value);
			desc->bounds_check = simple_bounds_check;
			desc->trans_to_api = translate_ctrl_simple_to_api;
			desc->trans_to_hfi = translate_ctrl_simple_to_hfi;
			break;
		case CTRL_TYPE_STANDARD:
			desc->size = sizeof(struct vpu_ctrl_standard);
			desc->hfi_size = sizeof(struct vpu_s_data_value);
			desc->bounds_check = standard_bounds_check;
			desc->trans_to_api = translate_ctrl_standard_to_api;
			desc->trans_to_hfi = translate_ctrl_standard_to_hfi;
			break;
		case CTRL_TYPE_AUTO_MANUAL:
			desc->size = sizeof(struct vpu_ctrl_auto_manual);
			desc->hfi_size = sizeof(struct vpu_s_data_value);
			desc->bounds_check = auto_manual_bounds_check;
			desc->trans_to_api = translate_ctrl_auto_manual_to_api;
			desc->trans_to_hfi = translate_ctrl_auto_manual_to_hfi;
			break;
		default:
			break;
		}

		controller_cache_size += desc->size;
		desc->cache_offset = prev_desc->cache_offset + prev_desc->size;
	}
}

struct vpu_controller *init_vpu_controller(struct vpu_platform_resources *res)
{
	int i;
	struct vpu_controller *controller =
			kzalloc(sizeof(*controller), GFP_KERNEL);
	if (!controller) {
		dprintk(VPU_ERR, "%s, Failed memory allocation\n", __func__);
		return NULL;
	}

	controller->cache_size = controller_cache_size;
	if (controller->cache_size) {
		controller->cache = kzalloc(controller->cache_size, GFP_KERNEL);
		if (!controller->cache) {
			dprintk(VPU_ERR, "%s, Failed cache allocation\n",
					__func__);
			deinit_vpu_controller(controller);
			return NULL;
		}
	}

	for (i = 0; i < NUM_NR_BUFFERS; i++)
		controller->nr_buffers[i] = vpu_iommu_create_handle(res);

	if (i < NUM_NR_BUFFERS) {
		deinit_vpu_controller(controller);
		return NULL;
	}
	return controller;
}

void deinit_vpu_controller(struct vpu_controller *controller)
{
	int i;
	if (!controller)
		return;

	for (i = 0; i < NUM_NR_BUFFERS; i++) {
		vpu_iommu_destroy_handle(controller->nr_buffers[i]);
		controller->nr_buffers[i] = NULL;
	}

	kfree(controller->cache);
	controller->cache = NULL;
	kfree(controller);
}

void *get_control(struct vpu_controller *controller, u32 id)
{
	struct vpu_ctrl_desc *desc;
	if (!controller || !controller->cache  ||
		id <= VPU_CTRL_ID_MIN || id >= VPU_CTRL_ID_MAX)
		return NULL;

	desc = &ctrl_descriptors[id];

	if (!desc->size || desc->cache_offset > controller->cache_size ||
	    (desc->cache_offset + desc->size) > controller->cache_size)
		return NULL;
	else
		return controller->cache + desc->cache_offset;
}

int apply_vpu_control(struct vpu_dev_session *session, int set,
		struct vpu_control *control)
{
	struct vpu_controller *controller = session->controller;
	struct vpu_ctrl_desc *desc;
	void *cache = NULL;
	int ret = 0;

	u8 hfi_data[128]; /* temporary store translated to HFI */
	u32 hfi_size = 0; /* size of HFI data */
	memset(hfi_data, 0, 128);

	if (!controller || !control)
		return -ENOMEM;

	if (!control->control_id || control->control_id >= VPU_CTRL_ID_MAX) {
		dprintk(VPU_ERR, "%s, control id %d is invalid\n",
				__func__, control->control_id);
		return -EINVAL;
	}

	desc = &ctrl_descriptors[control->control_id];
	if (!desc->hfi_id && !desc->size &&
			!desc->set_function && !desc->get_function) {
		dprintk(VPU_ERR, "%s, control id %d is not supported\n",
				__func__, control->control_id);
		return -EINVAL;
	}

	dprintk(VPU_INFO, "%s, %s control %s\n",
			__func__, set ? "Set" : "Get", desc->name);

	if (set && desc->set_function)
		return call_op(desc, set_function, session, &control->data);
	else if (!set && desc->get_function)
		return call_op(desc, get_function, session, &control->data);

	cache = get_control(controller, control->control_id);
	if (!cache) {
		dprintk(VPU_ERR, "%s, control %s has no cache\n",
				__func__, desc->name);
		return -ENOMEM;
	}

	hfi_size = (desc->custom_size) ?
		call_op(desc, custom_size, &control->data) : desc->hfi_size;

	if (set) {
		ret = call_op(desc, bounds_check, desc, &control->data);
		if (ret) {
			dprintk(VPU_ERR, "%s, bounds check failed for %s\n",
					__func__, desc->name);
			return ret;
		}

		if (!desc->hfi_id) {
			memcpy(cache, &control->data, desc->size);
			return 0;
		}

		if (desc->trans_to_hfi)
			call_op(desc, trans_to_hfi, &control->data, hfi_data);
		else {
			hfi_size = desc->size;
			memcpy(hfi_data, &control->data, desc->size);
		}

		mutex_lock(&session->lock);
		ret = vpu_hw_session_s_property(session->id, desc->hfi_id,
				hfi_data, hfi_size);

		if (!ret)
			ret = call_op(desc, pre_commit, session,
					&control->data);

		if (!ret)
			ret = commit_control(session, 0);

		if (!ret)
			call_op(desc, post_commit, session, &control->data);

		/* update controls cache */
		if (!ret)
			memcpy(cache, &control->data, desc->size);
		mutex_unlock(&session->lock);
	} else {
		/* For cached controls, get control data from cache */
		if (desc->cached) {
			memcpy(&control->data, cache, desc->size);
			return 0;
		}

		ret = vpu_hw_session_g_property(session->id, desc->hfi_id,
				hfi_data, hfi_size);
		if (ret)
			return ret;

		if (desc->trans_to_api)
			call_op(desc, trans_to_api, &control->data, hfi_data);
		else
			memcpy(&control->data, hfi_data, desc->size);
	}

	return ret;
}

int apply_vpu_control_extended(struct vpu_client *client, int set,
		struct vpu_control_extended *control)
{
	struct vpu_dev_session *session = client->session;
	int ret = 0;

	if (!control)
		return -ENOMEM;
	if (control->data_len > VPU_MAX_EXT_DATA_SIZE ||
			control->buf_size > VPU_MAX_EXT_DATA_SIZE) {
		dprintk(VPU_ERR, "%s, data_len or buf_size > than max (%d)\n",
				__func__, VPU_MAX_EXT_DATA_SIZE);
		return -EINVAL;
	}

	if (control->type == 0) { /* system */
		if (set)
			ret = vpu_hw_sys_s_property_ext(control->data_ptr,
					control->data_len);
		else
			ret = vpu_hw_sys_g_property_ext(control->data_ptr,
					control->data_len, control->buf_ptr,
					control->buf_size);
	} else if (control->type == 1) { /* session */
		if (!session)
			return -EPERM;

		if (set)
			ret = vpu_hw_session_s_property_ext(session->id,
					control->data_ptr, control->data_len);
		else
			ret = vpu_hw_session_g_property_ext(session->id,
					control->data_ptr, control->data_len,
					control->buf_ptr, control->buf_size);
	} else {
		dprintk(VPU_ERR, "%s, control type %d is invalid\n",
						__func__, control->type);
		return -EINVAL;
	}

	return ret;
}

/*
 * Port parameters configuration
 */

/* List of supported formats */
static const struct vpu_format_desc vpu_port_formats[] = {
	[PIXEL_FORMAT_RGB888] = {
		.description = "RGB-8-8-8",
		.fourcc = VPU_PIX_FMT_RGB888,
		.num_planes = 1,
		.plane[0] = { .bitsperpixel = 24, .heightfactor = 1},
	},
	[PIXEL_FORMAT_XRGB8888] = {
		.description = "ARGB-8-8-8-8",
		.fourcc = VPU_PIX_FMT_XRGB8888,
		.num_planes = 1,
		.plane[0] = { .bitsperpixel = 32, .heightfactor = 1},
	},
	[PIXEL_FORMAT_XRGB2] = {
		.description = "ARGB-2-10-10-10",
		.fourcc = VPU_PIX_FMT_XRGB2,
		.num_planes = 1,
		.plane[0] = { .bitsperpixel = 32, .heightfactor = 1},
	},
	[PIXEL_FORMAT_BGR888] = {
		.description = "BGR-8-8-8",
		.fourcc = VPU_PIX_FMT_BGR888,
		.num_planes = 1,
		.plane[0] = { .bitsperpixel = 24, .heightfactor = 1},
	},
	[PIXEL_FORMAT_BGRX8888] = {
		.description = "BGRX-8-8-8-8",
		.fourcc = VPU_PIX_FMT_BGRX8888,
		.num_planes = 1,
		.plane[0] = { .bitsperpixel = 32, .heightfactor = 1},
	},
	[PIXEL_FORMAT_XBGR2] = {
		.description = "XBGR-2-10-10-10",
		.fourcc = VPU_PIX_FMT_XBGR2,
		.num_planes = 1,
		.plane[0] = { .bitsperpixel = 32, .heightfactor = 1},
	},
	[PIXEL_FORMAT_NV12] = {
		.description = "YUV 4:2:0 semi-planar",
		.fourcc = VPU_PIX_FMT_NV12,
		.num_planes = 2,
		.plane[0] = { .bitsperpixel = 8, .heightfactor = 1},
		.plane[1] = { .bitsperpixel = 8, .heightfactor = 2},
	},
	[PIXEL_FORMAT_NV21] = {
		.description = "YVU 4:2:0 semi-planar",
		.fourcc = VPU_PIX_FMT_NV21,
		.num_planes = 2,
		.plane[0] = { .bitsperpixel = 8, .heightfactor = 1},
		.plane[1] = { .bitsperpixel = 8, .heightfactor = 2},
	},
	[PIXEL_FORMAT_YUYV] = {
		.description = "YUYV 4:2:2 intlvd",
		.fourcc = VPU_PIX_FMT_YUYV,
		.num_planes = 1,
		.plane[0] = { .bitsperpixel = 16, .heightfactor = 1},
	},
	[PIXEL_FORMAT_YVYU] = {
		.description = "YVYU 4:2:2 intlvd",
		.fourcc = VPU_PIX_FMT_YVYU,
		.num_planes = 1,
		.plane[0] = { .bitsperpixel = 16, .heightfactor = 1},
	},
	[PIXEL_FORMAT_VYUY] = {
		.description = "VYUY 4:2:2 intlvd",
		.fourcc = VPU_PIX_FMT_VYUY,
		.num_planes = 1,
		.plane[0] = { .bitsperpixel = 16, .heightfactor = 1},
	},
	[PIXEL_FORMAT_UYVY] = {
		.description = "UYVY 4:2:2 intlvd",
		.fourcc = VPU_PIX_FMT_UYVY,
		.num_planes = 1,
		.plane[0] = { .bitsperpixel = 16, .heightfactor = 1},
	},
	[PIXEL_FORMAT_YUYV_LOOSE] = {
		.description = "YUYV 4:2:2 10bit packed loose",
		.fourcc = VPU_PIX_FMT_YUYV10,
		.num_planes = 2,
		.plane[0] = { .bitsperpixel = 10, .heightfactor = 1},
		.plane[1] = { .bitsperpixel = 10, .heightfactor = 1},
	},
	[PIXEL_FORMAT_YUV_8BIT_INTERLEAVED_DENSE] = {
		.description = "YUV 4:4:4 8bit intlvd dense",
		.fourcc = 0,
		.num_planes = 1,
		.plane[0] = { .bitsperpixel = 24, .heightfactor = 1},
	},
	[PIXEL_FORMAT_YUV_10BIT_INTERLEAVED_LOOSE] = {
		.description = "YUV 4:4:4 10bit intlvd loose",
		.fourcc = 0,
		.num_planes = 1,
		.plane[0] = { .bitsperpixel = 30, .heightfactor = 1},
	},
	[PIXEL_FORMAT_COMPRESSED_YUYV422] = {
		.description = "YUYV422 Compressed",
		.fourcc = VPU_PIX_FMT_YUV_COMP,
		.num_planes = 1,
		.plane[0] = { .bitsperpixel = 10, .heightfactor = 1},
	},
};

#define PADDING 128
#define CEIL(x, y) (((x) + ((y)-1)) / (y))
u32 get_bytesperline(u32 width, u32 bitsperpixel, u32 input_bytesperline)
{
	u32 bytesperline = CEIL(width * bitsperpixel, 8);
	u32 padding_factor = CEIL(bytesperline, PADDING);
	u32 min_bytesperline = PADDING * padding_factor;

	if (input_bytesperline < min_bytesperline) /* input too low */
		return min_bytesperline;
	else if (input_bytesperline % PADDING) /* must be multiple of padding */
		return min_bytesperline;
	else if (input_bytesperline > SZ_32K) /* not too big */
		return min_bytesperline;
	else
		return input_bytesperline; /* input bytesperline was fine */
}

int is_format_valid(struct v4l2_format *fmt)
{
#define VPU_HEIGHT_MINIMUM 72
#define VPU_HEIGHT_MULTIPLE 8

	u32 height = fmt->fmt.pix_mp.height;
	u32 width = fmt->fmt.pix_mp.width;

	if (height < VPU_HEIGHT_MINIMUM)
		return 0;
	if ((height / VPU_HEIGHT_MULTIPLE) * VPU_HEIGHT_MULTIPLE != height)
		return 0;
	if (height & 1) /* must be even */
		return 0;
	if (width & 1) /* must be even */
		return 0;

	return 1;
}

const struct vpu_format_desc *query_supported_formats(int index)
{
	if (index < 0 || index >= ARRAY_SIZE(vpu_port_formats))
		return NULL;
	if (vpu_port_formats[index].fourcc == 0)
		return NULL; /* fourcc of 0 indicates no support in API */
	else
		return &vpu_port_formats[index];
}

static const struct vpu_io_desc vpu_input_ports[] = {
	{ .description = "Non-Tunneling Buffer input"},
	{ .description = "Tunneling from VCAP port 0"},
	{ .description = "Tunneling from VCAP port 1"},
};

static const struct vpu_io_desc vpu_output_ports[] = {
	{ .description = "Non-Tunneling Buffer output"},
	{ .description = "Tunneling to Display port 0"},
	{ .description = "Tunneling to Display port 1"},
};

const struct vpu_io_desc *query_inputs_outputs(int port, int index)
{
	if ((port < 0) || (index < 0) ||
		(port == INPUT_PORT && index >= ARRAY_SIZE(vpu_input_ports)) ||
		(port == OUTPUT_PORT && index >= ARRAY_SIZE(vpu_output_ports))
	   )
		return NULL;

	return port ? &vpu_output_ports[index] : &vpu_input_ports[index];
}

static int __get_bits_per_pixel(u32 api_pix_fmt)
{
	int i, bpp = 0;
	const struct vpu_format_desc *vpu_format;
	u32 hfi_pix_fmt = trans_pixelformat_to_hfi(api_pix_fmt);

	vpu_format = query_supported_formats(hfi_pix_fmt);
	if (!vpu_format)
		return -EINVAL;

	for (i = 0; i < vpu_format->num_planes; i++)
		bpp  += vpu_format->plane[i].bitsperpixel /
			vpu_format->plane[i].heightfactor;

	return bpp;
}

/* compute the average load value for all VPU sessions (in bits per second) */
static u32 __get_vpu_load(struct vpu_dev_core *core)
{
	int i;
	u32 load_bps = 0;
	struct vpu_dev_session *session = NULL;

	for (i = 0; i < VPU_NUM_SESSIONS; i++) {
		session = core->sessions[i];
		if (!session)
			break;
		dprintk(VPU_DBG, "%s, session %d load: %dbps",
				__func__, i, session->load);
		load_bps = max(load_bps, session->load);
	}

	VPU_EXIT_FUNC("(%d bps)", load_bps);
	return /* TODO replace by load */ 500000;
}

/* returns the session load in bits per second */
int __calculate_session_load(struct vpu_dev_session *session)
{
	u32 bits_per_stream;
	u32 load_bits_per_sec;
	u32 frame_rate;
	bool b_interlaced, b_nr, b_bbroi;
	u32 bpp_in, bpp_vpu, h_in, w_in, bpp_out, h_out, w_out;
	struct vpu_port_info *in = &session->port_info[INPUT_PORT];
	struct vpu_port_info *out = &session->port_info[OUTPUT_PORT];
	struct vpu_controller *controller = session->controller;
	struct vpu_ctrl_auto_manual *nr_cache =
		(struct vpu_ctrl_auto_manual *) get_control(controller,
				VPU_CTRL_NOISE_REDUCTION);

	/*
	 * get required info before computation
	 */
	b_interlaced = (in->scan_mode == LINESCANINTERLACED);
	b_nr = nr_cache ? (nr_cache->enable == PROP_TRUE) : false;
	b_bbroi = false; /*TODO: how to calculate? */
	bpp_in = __get_bits_per_pixel(in->format.pixelformat);
	if (bpp_in <= 0)
		dprintk(VPU_ERR, "%s, bad value for bpp_in (%d)",
				__func__, bpp_in);
	bpp_vpu = 16; /* 8bpc422 pixel format */
	h_in = in->format.height;
	w_in = in->format.width;
	bpp_out = __get_bits_per_pixel(out->format.pixelformat);
	if (bpp_out <= 0)
		dprintk(VPU_ERR, "%s, bad value for bpp_out (%d)",
				__func__, bpp_out);
	h_out = out->format.height;
	w_out = out->format.width;
	frame_rate = max(in->framerate, out->framerate);

	/*
	 * compute the current session's load
	 */
	bits_per_stream =  ((1 + b_interlaced) * (bpp_in)
			+ (2 * b_nr + 2 * b_bbroi) * (bpp_vpu)) * (h_in * w_in)
			+ (bpp_out) * (h_out * w_out);
	load_bits_per_sec = bits_per_stream * frame_rate;

	VPU_EXIT_FUNC("(%d bps)", load_bits_per_sec);
	return load_bits_per_sec;
}

int commit_initial_config(struct vpu_dev_session *session)
{
	struct vpu_prop_session_input in_param;
	struct vpu_prop_session_output out_param;
	int ret = 0;

	VPU_ENTER_FUNC();

	translate_input_format_to_hfi(&session->port_info[INPUT_PORT],
			&in_param);
	ret = vpu_hw_session_s_input_params(session->id, &in_param);
	if (ret) {
		dprintk(VPU_ERR, "%s, Failed to set port 0 config\n", __func__);
		goto exit_commit;
	}

	translate_output_format_to_hfi(&session->port_info[OUTPUT_PORT],
			&out_param);
	ret = vpu_hw_session_s_output_params(session->id, &out_param);
	if (ret) {
		dprintk(VPU_ERR, "%s, Failed to set port 1 config\n", __func__);
		goto exit_commit;
	}

	/* calculate and store the newly computed session load */
	session->load = __calculate_session_load(session);

	ret = vpu_hw_session_commit(session->id, CH_COMMIT_AT_ONCE,
			__get_vpu_load(session->core));
	if (ret) {
		dprintk(VPU_ERR, "%s, Commit Failed (err %d)\n", __func__, ret);
		notify_vpu_event_session(session, VPU_EVENT_INVALID_CONFIG,
				NULL, 0);
		goto exit_commit;
	}

	session->commit_state = COMMITED;
	dprintk(VPU_INFO, "Initial configuration committed successfully\n");

exit_commit:
	return ret;
}

int commit_port_config(struct vpu_dev_session *session,	int port, int new_load)
{
	struct vpu_prop_session_input in_param;
	struct vpu_prop_session_output out_param;
	int ret = 0;

	if (session->commit_state != COMMITED)
		return 0; /* wait for initial configuration */

	VPU_ENTER_FUNC();

	if (port == INPUT_PORT) {
		translate_input_format_to_hfi(&session->port_info[INPUT_PORT],
				&in_param);
		ret = vpu_hw_session_s_input_params(session->id, &in_param);
	} else if (port == OUTPUT_PORT) {
		translate_output_format_to_hfi(&session->port_info[OUTPUT_PORT],
				&out_param);
		ret = vpu_hw_session_s_output_params(session->id, &out_param);
	} else
		return -EINVAL;

	if (ret) {
		dprintk(VPU_ERR, "%s, Failed to set port config\n", __func__);
		goto exit_commit;
	}

	if (new_load)
		session->load = __calculate_session_load(session);

	ret = vpu_hw_session_commit(session->id, CH_COMMIT_IN_ORDER,
			__get_vpu_load(session->core));
	if (ret) {
		dprintk(VPU_ERR, "%s, Commit Failed\n", __func__);
		if (ret == -EIO)
			ret = -EAGAIN; /* special runtime commit fail retval */
		goto exit_commit;
	}

exit_commit:
	return ret;
}

int commit_control(struct vpu_dev_session *session, int new_load)
{
	int ret = 0;

	if (session->commit_state != COMMITED)
		return 0; /* wait for initial configuration */

	VPU_ENTER_FUNC();

	if (new_load)
		session->load = __calculate_session_load(session);

	ret = vpu_hw_session_commit(session->id, CH_COMMIT_AT_ONCE,
			__get_vpu_load(session->core));
	if (ret) {
		dprintk(VPU_ERR, "%s, Commit Failed\n", __func__);
		if (ret == -EIO)
			ret = -EAGAIN; /* special runtime commit fail retval */
		goto exit_commit;
	}

exit_commit:
	return ret;
}
