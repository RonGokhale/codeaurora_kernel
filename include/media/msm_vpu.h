#ifndef _H_MSM_VPU_H_
#define _H_MSM_VPU_H_

#include <linux/videodev2.h>


/*
 * M I S C   D E F I N E S   A N D   E N U M S
 */

/*
 * VPU supported pixel format fourcc codes (use in s_fmt pixelformat field)
 */
/* 24 bit RGB-8-8-8 */
#define VPU_PIX_FMT_RGB888    V4L2_PIX_FMT_RGB24
/* 32 bit XRGB-8-8-8-8 */
#define VPU_PIX_FMT_XRGB8888  V4L2_PIX_FMT_RGB32
/* 32 bit XRGB-2-10-10-10*/
#define VPU_PIX_FMT_XRGB2     v4l2_fourcc('X', '2', 'R', 'G')
/* 24 bit BGR-8-8-8 */
#define VPU_PIX_FMT_BGR888    V4L2_PIX_FMT_BGR24
/* 32 bit BGRX-8-8-8-8 */
#define VPU_PIX_FMT_BGRX8888  V4L2_PIX_FMT_BGR32
/* 32 bit XBGR-2-10-10-10*/
#define VPU_PIX_FMT_XBGR2     v4l2_fourcc('X', '2', 'B', 'G')
/* 12 bit YUV 4:2:0  semi-planar*/
#define VPU_PIX_FMT_NV12      V4L2_PIX_FMT_NV12
/* 12 bit YVU 4:2:0  semi-planar*/
#define VPU_PIX_FMT_NV21      V4L2_PIX_FMT_NV21
/* 16 bit YUYV 4:2:2 interleaved*/
#define VPU_PIX_FMT_YUYV      V4L2_PIX_FMT_YUYV
/* 16 bit YVYU 4:2:2 interleaved*/
#define VPU_PIX_FMT_YVYU      V4L2_PIX_FMT_YVYU
/* 16 bit VYUY 4:2:2 interleaved*/
#define VPU_PIX_FMT_VYUY      V4L2_PIX_FMT_VYUY
/* 16 bit UYVY 4:2:2 interleaved*/
#define VPU_PIX_FMT_UYVY      V4L2_PIX_FMT_UYVY
/* 20 bit YUYV 4:2:2 10bit per component (packed loose)*/
#define VPU_PIX_FMT_YUYV10    v4l2_fourcc('Y', 'U', 'V', 'L')
/* 10 bit YUYV422 Compressed*/
#define VPU_PIX_FMT_YUV_COMP  v4l2_fourcc('Y', 'U', 'V', 'C')


/*
 * V P U   E V E N T S :   I D s   A N D   D A T A   P A Y L O A D S
 */

/*
 * Event IDs: use in type field with VIDIOC_(SUBSCRIBE/UNSUBSCRIBE/DQEVENT)
 * Payload data: returned in data array of struct v4l2_event (VIDIOC_DQEVENT)
 *
 * VPU_EVENT_HW_ERROR: a hardware error occured in VPU
 * payload data: NULL
 *
 * VPU_EVENT_INVALID_CONFIG: invalid VPU session configuration
 * payload data: NULL
 *
 * VPU_EVENT_ACTIVE_REGION_CHANGED: New Active Region Detected
 * payload data: new active region (struct v4l2_rect)
 *
 * VPU_EVENT_SESSION_TIMESTAMP: New Session timestamp
 * payload data: NULL
 *
 */
#define VPU_PRIVATE_EVENT_BASE (V4L2_EVENT_PRIVATE_START + 6 * 1000)
enum VPU_PRIVATE_EVENT {
	VPU_EVENT_START = VPU_PRIVATE_EVENT_BASE + 1,

	VPU_EVENT_HW_ERROR = VPU_EVENT_START,
	VPU_EVENT_INVALID_CONFIG,
	VPU_EVENT_ACTIVE_REGION_CHANGED,
	VPU_EVENT_SESSION_TIMESTAMP,

	VPU_EVENT_END
};


/*
 * V P U   CO N T R O L S :   S T R U C T S   A N D   I D s
 *
 * Controls are video processing parameters
 */

/*
 * Standard VPU Controls
 */
struct vpu_ctrl_standard {
	__u32 enable;		/* boolean: 0=disable, else=enable */
	__s32 value;
};

struct vpu_ctrl_auto_manual {
	__u32 enable;		/* boolean: 0=disable, else=enable */
	__u32 auto_mode;	/* boolean: 0=manual, else=automatic */
	__s32 value;
};

struct vpu_ctrl_range_map {
	__u32 enable;		/* boolean: 0=disable, else=enable */
	__u32 y_range;		/* the range mapping set for Y */
	__u32 uv_range;		/* the range mapping set for UV */
};

#define VPU_ACTIVE_REGION_N_EXCLUSIONS 1
struct vpu_ctrl_active_region_param {
	__u32               enable; /* boolean: 0=disable, else=enable */
	/* number of exclusion regions */
	__u32               num_exclusions;
	/* roi where active region detection is applied */
	struct v4l2_rect    detection_region;
	/* roi(s) excluded from active region detection*/
	struct v4l2_rect    excluded_regions[VPU_ACTIVE_REGION_N_EXCLUSIONS];
};

struct vpu_control {
	__u32 control_id;
	union control_data {
		__s32 value;
		struct vpu_ctrl_standard standard;
		struct vpu_ctrl_auto_manual auto_manual;
		struct vpu_ctrl_range_map range_map;
		struct vpu_ctrl_active_region_param active_region_param;
		struct v4l2_rect active_region_result;
	} data;
};

/*
 * IDs for standard controls (use in control_id field of struct vpu_control)
 *
 * VPU_CTRL_NOISE_REDUCTION: noise reduction level, data: auto_manual
 *
 * VPU_CTRL_IMAGE_ENHANCEMENT: image enhancement level, data: auto_manual
 *
 * VPU_CTRL_ANAMORPHIC_SCALING: anamorphic scaling config, data: standard
 *
 * VPU_CTRL_DIRECTIONAL_INTERPOLATION: directional interpolation config
 * data: standard
 *
 * VPU_CTRL_BACKGROUND_COLOR:
 *
 * VPU_CTRL_RANGE_MAPPING: Y/UV range mapping, data: range_map
 *
 * VPU_CTRL_ACTIVE_REGION_PARAM: active region detection parameters (set only)
 * data: active_region_param
 *
 * VPU_CTRL_ACTIVE_REGION_RESULT: detected active region roi (get only)
 * data: active_region_result
 *
 * VPU_CTRL_PRIORITY: Session priority, data: value
 *
 * VPU_CTRL_CONTENT_PROTECTION: Session content protection status, data: value
 *
 * VPU_CTRL_FPS_INPUT: input fps
 * VPU_CTRL_FPS_OUTPUT: output fps
 * VPU_CTRL_FPS_DISPLAY: display fps
 * data: value (set to __u32 16.16 format)
 *
 * VPU_CTRL_ACE: , data: value
 *
 * VPU_CTRL_ACE_BRIGHTNESS: , data: value
 *
 * VPU_CTRL_ACE_CONTRAST: , data: value
 *
 * VPU_CTRL_2D3D: , data: value
 *
 * VPU_CTRL_2D3D_DEPTH: , data: value
 *
 * VPU_CTRL_TIMESTAMP_MODE: , data: value
 *
 * VPU_CTRL_FRC: , data: value
 *
 * VPU_CTRL_FRC_MOTION_SMOOTHNESS: , data: value
 *
 * VPU_CTRL_FRC_MOTION_CLEAR: , data: value
 *
 * VPU_CTRL_LATENCY: session latency (get only), data: value
 */
#define VPU_CTRL_ID_MIN						0

#define VPU_CTRL_NOISE_REDUCTION				1
#define VPU_CTRL_IMAGE_ENHANCEMENT				2
#define VPU_CTRL_ANAMORPHIC_SCALING				3
#define VPU_CTRL_DIRECTIONAL_INTERPOLATION			4
#define VPU_CTRL_BACKGROUND_COLOR				5
#define VPU_CTRL_RANGE_MAPPING					6
#define VPU_CTRL_ACTIVE_REGION_PARAM				8
#define VPU_CTRL_ACTIVE_REGION_RESULT				9

#define	VPU_CTRL_PRIORITY					10
#define	VPU_CTRL_CONTENT_PROTECTION				11
#define	VPU_CTRL_FPS_INPUT					12
#define	VPU_CTRL_FPS_OUTPUT					13
#define	VPU_CTRL_FPS_DISPLAY					14

#define	VPU_CTRL_HQV						20
#define	VPU_CTRL_HQV_SHARPEN					21
#define	VPU_CTRL_HQV_AUTONR					22
#define	VPU_CTRL_ACE						23
#define	VPU_CTRL_ACE_BRIGHTNESS					24
#define	VPU_CTRL_ACE_CONTRAST					25
#define	VPU_CTRL_2D3D						26
#define	VPU_CTRL_2D3D_DEPTH					27
#define	VPU_CTRL_TIMESTAMP_MODE					28
#define	VPU_CTRL_FRC						29
#define	VPU_CTRL_FRC_MOTION_SMOOTHNESS				30
#define	VPU_CTRL_FRC_MOTION_CLEAR				31
#define	VPU_CTRL_LATENCY					32

#define VPU_CTRL_ID_MAX						33


/*
 * Extended VPU Controls (large data payloads)
 */
#define VPU_MAX_EXT_DATA_SIZE	720
struct vpu_control_extended {
	/*
	 * extended control type
	 * 0: system
	 * 1: session
	 */
	__u32 type;

	/*
	 * size and ptr of the data to send
	 * maximum VPU_MAX_EXT_DATA_SIZE bytes
	 */
	__u32 data_len;
	void *data_ptr;

	/*
	 * size and ptr of the buffer to recv data
	 * maximum VPU_MAX_EXT_DATA_SIZE bytes
	 */
	__u32 buf_size;
	void *buf_ptr;
};


/*
 * V P U   D E V I C E   P R I V A T E   I O C T L   C O D E S
 */

/* VPU Session ioctls */
#define VPU_QUERY_SESSIONS	_IOR('V', (BASE_VIDIOC_PRIVATE + 0), int)
#define VPU_ATTACH_TO_SESSION _IOW('V', (BASE_VIDIOC_PRIVATE + 1), int)

/* Explicit commit of session configuration */
#define VPU_COMMIT_CONFIGURATION    _IO('V', (BASE_VIDIOC_PRIVATE + 10))

/* Flush all buffers of given type (port) */
#define VPU_FLUSH_BUFS    _IOW('V', (BASE_VIDIOC_PRIVATE + 15), \
		enum v4l2_buf_type)

/* VPU controls get/set ioctls (for most controls with small data) */
#define VPU_G_CONTROL	_IOWR('V', (BASE_VIDIOC_PRIVATE + 20), \
						struct vpu_control)
#define VPU_S_CONTROL	_IOW('V', (BASE_VIDIOC_PRIVATE + 21), \
						struct vpu_control)

/* extended control set/get ioctls (large data payloads) */
#define VPU_G_CONTROL_EXTENDED	_IOWR('V', (BASE_VIDIOC_PRIVATE + 22), \
		struct vpu_control_extended)
#define VPU_S_CONTROL_EXTENDED	_IOW('V', (BASE_VIDIOC_PRIVATE + 23), \
		struct vpu_control_extended)


#endif /* _H_MSM_VPU_H_ */

