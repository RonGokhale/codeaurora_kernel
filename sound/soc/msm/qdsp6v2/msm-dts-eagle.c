/* Copyright (c) 2014, The Linux Foundation. All rights reserved.
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

#define DEBUG

#include <linux/init.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/msm_ion.h>
#include <linux/mm.h>
#include <linux/msm_audio_ion.h>
#include <sound/core.h>
#include <sound/soc.h>
#include <sound/pcm.h>
#include <sound/q6adm-v2.h>
#include <sound/q6asm-v2.h>
#include <sound/apr_audio-v2.h>
#include <sound/q6audio-v2.h>
#include <sound/audio_effects.h>
#include <sound/hwdep.h>

#include "msm-pcm-routing-v2.h"
#include "msm-dts-eagle.h"
#include "q6core.h"

#define ION_MEM_SIZE  131072
#define DEPC_MAX_SIZE 524288

#define MPST				AUDPROC_MODULE_ID_DTS_HPX_POSTMIX
#define MPRE				AUDPROC_MODULE_ID_DTS_HPX_PREMIX

enum {
	AUDIO_DEVICE_OUT_EARPIECE = 0,
	AUDIO_DEVICE_OUT_SPEAKER,
	AUDIO_DEVICE_OUT_WIRED_HEADSET,
	AUDIO_DEVICE_OUT_WIRED_HEADPHONE,
	AUDIO_DEVICE_OUT_BLUETOOTH_SCO,
	AUDIO_DEVICE_OUT_BLUETOOTH_SCO_HEADSET,
	AUDIO_DEVICE_OUT_BLUETOOTH_SCO_CARKIT,
	AUDIO_DEVICE_OUT_BLUETOOTH_A2DP,
	AUDIO_DEVICE_OUT_BLUETOOTH_A2DP_HEADPHONES,
	AUDIO_DEVICE_OUT_BLUETOOTH_A2DP_SPEAKER,
	AUDIO_DEVICE_OUT_AUX_DIGITAL,
	AUDIO_DEVICE_OUT_ANLG_DOCK_HEADSET,
	AUDIO_DEVICE_OUT_DGTL_DOCK_HEADSET,
	AUDIO_DEVICE_OUT_USB_ACCESSORY,
	AUDIO_DEVICE_OUT_USB_DEVICE,
	AUDIO_DEVICE_OUT_REMOTE_SUBMIX,
	AUDIO_DEVICE_OUT_ANC_HEADSET,
	AUDIO_DEVICE_OUT_ANC_HEADPHONE,
	AUDIO_DEVICE_OUT_PROXY,
	AUDIO_DEVICE_OUT_FM,
	AUDIO_DEVICE_OUT_FM_TX,

	AUDIO_DEVICE_OUT_COUNT
};

#define AUDIO_DEVICE_COMBO 0x400000 /* bit 23 */

enum { /* cache block */
	CB_0 = 0,
	CB_1,
	CB_2,
	CB_3,
	CB_4,
	CB_5,
	CB_6,
	CB_7,

	CB_COUNT
};

enum { /* cache block description */
	CBD_DEV_MASK = 0,
	CBD_OFFSG,
	CBD_CMD0,
	CBD_SZ0,
	CBD_OFFS1,
	CBD_CMD1,
	CBD_SZ1,
	CBD_OFFS2,
	CBD_CMD2,
	CBD_SZ2,
	CBD_OFFS3,
	CBD_CMD3,
	CBD_SZ3,

	CBD_COUNT,
};

static __s32 _fx_logN(__s32 x)
{
	__s32 t, y = 0xa65af;
	if (x < 0x00008000) {
		x <<= 16; y -= 0xb1721; }
	if (x < 0x00800000) {
		x <<= 8; y -= 0x58b91; }
	if (x < 0x08000000) {
		x <<= 4; y -= 0x2c5c8; }
	if (x < 0x20000000) {
		x <<= 2; y -= 0x162e4; }
	if (x < 0x40000000) {
		x <<= 1; y -= 0x0b172; }
	t = x + (x >> 1);
	if ((t & 0x80000000) == 0) {
		x = t; y -= 0x067cd; }
	t = x + (x >> 2);
	if ((t & 0x80000000) == 0) {
		x = t; y -= 0x03920; }
	t = x + (x >> 3);
	if ((t & 0x80000000) == 0) {
		x = t; y -= 0x01e27; }
	t = x + (x >> 4);
	if ((t & 0x80000000) == 0) {
		x = t; y -= 0x00f85; }
	t = x + (x >> 5);
	if ((t & 0x80000000) == 0) {
		x = t; y -= 0x007e1; }
	t = x + (x >> 6);
	if ((t & 0x80000000) == 0) {
		x = t; y -= 0x003f8; }
	t = x + (x >> 7);
	if ((t & 0x80000000) == 0) {
		x = t; y -= 0x001fe; }
	x = 0x80000000 - x;
	y -= x >> 15;
	return y;
}

static inline void *_getd(struct dts_eagle_param_desc *depd)
{
	return (void *)(((char *)depd) + sizeof(struct dts_eagle_param_desc));
}

static int _ref_cnt;
/* dts eagle parameter cache */
static char *_depc;
static __s32 _depc_size;
static __s32 _c_bl[CB_COUNT][CBD_COUNT];
static __u32 _device_primary;
static __u32 _device_all;
/* ION states */
static struct ion_client *_ion_client;
static struct ion_handle *_ion_handle;
static struct param_outband _po;
static struct audio_client *_ac_NT;
static struct ion_client *_ion_client_NT;
static struct ion_handle *_ion_handle_NT;
static struct param_outband _po_NT;

#define SEC_BLOB_MAX_CNT 10
#define SEC_BLOB_MAX_SIZE 0x4004 /*extra 4 for size*/
static char *_sec_blob[SEC_BLOB_MAX_CNT];

/* multi-copp support */
static int _cidx[AFE_MAX_PORTS] = {-1};

/* volume controls */
#define VOL_CMD_CNT_MAX 10
static __s32 _vol_cmd_cnt;
static __s32 **_vol_cmds;
struct vol_cmds_d {
	__s32 d[4];
};
static struct vol_cmds_d *_vol_cmds_d;
static const __s32 _log10_10_inv_x20 = 0x0008af84;

/* hpx master control */
static __u32 _is_hpx_enabled;

static void _volume_cmds_free(void)
{
	int i;
	for (i = 0; i < _vol_cmd_cnt; i++)
		kfree(_vol_cmds[i]);
	_vol_cmd_cnt = 0;
	kfree(_vol_cmds);
	kfree(_vol_cmds_d);
	_vol_cmds = NULL;
	_vol_cmds_d = NULL;
}

static __s32 _volume_cmds_alloc1(__s32 size)
{
	_volume_cmds_free();
	_vol_cmd_cnt = size;
	_vol_cmds = kzalloc(_vol_cmd_cnt * sizeof(int *), GFP_KERNEL);
	if (_vol_cmds) {
		_vol_cmds_d = kzalloc(_vol_cmd_cnt * sizeof(struct vol_cmds_d),
					GFP_KERNEL);
	}
	if (_vol_cmds_d)
		return 0;
	_volume_cmds_free();
	return -ENOMEM;
}

/* assumes size is equal or less than 0xFFF */
static __s32 _volume_cmds_alloc2(__s32 idx, __s32 size)
{
	kfree(_vol_cmds[idx]);
	_vol_cmds[idx] = kzalloc(size, GFP_KERNEL);
	if (_vol_cmds[idx])
		return 0;
	_vol_cmds_d[idx].d[0] = 0;
	return -ENOMEM;
}

static void _init_cb_descs(void)
{
	int i;
	for (i = 0; i < CB_COUNT; i++) {
		_c_bl[i][CBD_DEV_MASK] = 0;
		_c_bl[i][CBD_OFFSG] = _c_bl[i][CBD_OFFS1] =
		_c_bl[i][CBD_OFFS2] = _c_bl[i][CBD_OFFS3] =
		0xFFFFFFFF;
		_c_bl[i][CBD_CMD0] = _c_bl[i][CBD_SZ0] =
		_c_bl[i][CBD_CMD1] = _c_bl[i][CBD_SZ1] =
		_c_bl[i][CBD_CMD2] = _c_bl[i][CBD_SZ2] =
		_c_bl[i][CBD_CMD3] = _c_bl[i][CBD_SZ3] = 0;
	}
}

static __u32 _get_dev_mask_for_pid(int pid)
{
	switch (pid) {
	case SLIMBUS_0_RX:
		return (1 << AUDIO_DEVICE_OUT_EARPIECE) |
			(1 << AUDIO_DEVICE_OUT_SPEAKER) |
			(1 << AUDIO_DEVICE_OUT_WIRED_HEADSET) |
			(1 << AUDIO_DEVICE_OUT_WIRED_HEADPHONE) |
			(1 << AUDIO_DEVICE_OUT_ANC_HEADSET) |
			(1 << AUDIO_DEVICE_OUT_ANC_HEADPHONE);
	case INT_BT_SCO_RX:
		return (1 << AUDIO_DEVICE_OUT_BLUETOOTH_SCO) |
			(1 << AUDIO_DEVICE_OUT_BLUETOOTH_SCO_HEADSET) |
			(1 << AUDIO_DEVICE_OUT_BLUETOOTH_SCO_CARKIT);
	case RT_PROXY_PORT_001_RX:
		return (1 << AUDIO_DEVICE_OUT_BLUETOOTH_A2DP) |
			(1 << AUDIO_DEVICE_OUT_BLUETOOTH_A2DP_HEADPHONES) |
			(1 << AUDIO_DEVICE_OUT_BLUETOOTH_A2DP_SPEAKER) |
			(1 << AUDIO_DEVICE_OUT_ANLG_DOCK_HEADSET) |
			(1 << AUDIO_DEVICE_OUT_DGTL_DOCK_HEADSET) |
			(1 << AUDIO_DEVICE_OUT_USB_ACCESSORY) |
			(1 << AUDIO_DEVICE_OUT_USB_DEVICE) |
			(1 << AUDIO_DEVICE_OUT_PROXY);
	case HDMI_RX:
		return 1 << AUDIO_DEVICE_OUT_AUX_DIGITAL;
	case INT_FM_RX:
		return 1 << AUDIO_DEVICE_OUT_FM;
	case INT_FM_TX:
		return 1 << AUDIO_DEVICE_OUT_FM_TX;
	default:
		return 0;
	}
}

static int _get_pid_from_dev(__u32 device)
{
	if (device & (1 << AUDIO_DEVICE_OUT_EARPIECE) ||
	    device & (1 << AUDIO_DEVICE_OUT_SPEAKER) ||
	    device & (1 << AUDIO_DEVICE_OUT_WIRED_HEADSET) ||
	    device & (1 << AUDIO_DEVICE_OUT_WIRED_HEADPHONE) ||
	    device & (1 << AUDIO_DEVICE_OUT_ANC_HEADSET) ||
	    device & (1 << AUDIO_DEVICE_OUT_ANC_HEADPHONE)) {
		return SLIMBUS_0_RX;
	} else if (device & (1 << AUDIO_DEVICE_OUT_BLUETOOTH_SCO) ||
		   device & (1 << AUDIO_DEVICE_OUT_BLUETOOTH_SCO_HEADSET) ||
		   device & (1 << AUDIO_DEVICE_OUT_BLUETOOTH_SCO_CARKIT)) {
		return INT_BT_SCO_RX;
	} else if (device & (1 << AUDIO_DEVICE_OUT_BLUETOOTH_A2DP) ||
		   device & (1 << AUDIO_DEVICE_OUT_BLUETOOTH_A2DP_HEADPHONES) ||
		   device & (1 << AUDIO_DEVICE_OUT_BLUETOOTH_A2DP_SPEAKER) ||
		   device & (1 << AUDIO_DEVICE_OUT_ANLG_DOCK_HEADSET) ||
		   device & (1 << AUDIO_DEVICE_OUT_DGTL_DOCK_HEADSET) ||
		   device & (1 << AUDIO_DEVICE_OUT_USB_ACCESSORY) ||
		   device & (1 << AUDIO_DEVICE_OUT_USB_DEVICE) ||
		   device & (1 << AUDIO_DEVICE_OUT_PROXY)) {
		return RT_PROXY_PORT_001_RX;
	} else if (device & (1 << AUDIO_DEVICE_OUT_AUX_DIGITAL)) {
		return HDMI_RX;
	} else if (device & (1 << AUDIO_DEVICE_OUT_FM)) {
		return INT_FM_RX;
	} else if (device & (1 << AUDIO_DEVICE_OUT_FM_TX)) {
		return INT_FM_TX;
	}
	return 0;
}

static __u32 _get_cb_for_dev(int device)
{
	__u32 i;
	if (device & AUDIO_DEVICE_COMBO) {
		for (i = 0; i < CB_COUNT; i++) {
			if ((_c_bl[i][CBD_DEV_MASK] & device) == device)
				return i;
		}
	} else {
		for (i = 0; i < CB_COUNT; i++) {
			if ((_c_bl[i][CBD_DEV_MASK] & device) &&
			    !(_c_bl[i][CBD_DEV_MASK] & AUDIO_DEVICE_COMBO))
				return i;
		}
	}
	pr_err("DTS_EAGLE_DRIVER: %s - device %i not found, returning 0\n",
		   __func__, device);
	return 0;
}

static int _is_port_open_and_eagle(int pid)
{
	if (msm_routing_check_backend_enabled(pid))
		return 1;
	return 1;
}

static int _isNTDevice(__u32 device)
{
	if (device >= (1 << AUDIO_DEVICE_OUT_BLUETOOTH_SCO) &&
	    device <= (1 << AUDIO_DEVICE_OUT_BLUETOOTH_A2DP_SPEAKER)) {
		return 1;
	}
	return 0;
}

static void _reg_ion_mem(void)
{
	msm_audio_ion_alloc("DTS_EAGLE", &_ion_client, &_ion_handle,
			    ION_MEM_SIZE, &_po.paddr, &_po.size, &_po.kvaddr);
}

static void _unreg_ion_mem(void)
{
	msm_audio_ion_free(_ion_client, _ion_handle);
}

static void _reg_ion_mem_NT(void)
{
	int rc = -EINVAL;

	pr_debug("DTS_EAGLE_DRIVER: %s\n", __func__);
	rc = msm_audio_ion_alloc("DTS_EAGLE", &_ion_client_NT,
				 &_ion_handle_NT, ION_MEM_SIZE,
				 &_po_NT.paddr, &_po_NT.size, &_po_NT.kvaddr);
	if (rc) {
		pr_err("DTS_EAGLE_DRIVER: %s - msm audio ion alloc failed\n",
			__func__);
		return;
	}

	rc = q6asm_memory_map(_ac_NT, _po_NT.paddr,
			      IN, _po_NT.size, 1);
	if (rc < 0) {
		pr_err("DTS_EAGLE_DRIVER: %s - memory map failed\n", __func__);
		msm_audio_ion_free(_ion_client_NT, _ion_handle_NT);
	}
	return;
}

static void _unreg_ion_mem_NT(void)
{
	int rc = -EINVAL;
	rc = q6asm_memory_unmap(_ac_NT,	(uint32_t) _po_NT.paddr, IN);
	if (rc < 0)
		pr_err("%s: mem unmap failed\n", __func__);
	rc = msm_audio_ion_free(_ion_client_NT, _ion_handle_NT);
	if (rc < 0)
		pr_err("%s: mem free failed\n", __func__);
}

static struct audio_client *_getNTDeviceAC(void)
{
	return _ac_NT;
}

static void _set_audioclient(struct audio_client *ac)
{
	_ac_NT = ac;
	_reg_ion_mem_NT();
}

static void _clear_audioclient(void)
{
	_unreg_ion_mem_NT();
	_ac_NT = NULL;
}


static int _sendcache_pre(struct audio_client *ac)
{
	int offset, cidx, size, cmd;
	cidx = _get_cb_for_dev(_device_primary);
	if (cidx < 0) {
		pr_err("DTS_EAGLE_DRIVER_SENDCACHE_PRE: in %s, no cache for primary device %i found.\n",
			__func__, _device_primary);
		return -EINVAL;
	}
	offset = _c_bl[cidx][CBD_OFFSG];
	cmd = _c_bl[cidx][CBD_CMD0];
	size = _c_bl[cidx][CBD_SZ0];

	if (_depc_size == 0 || !_depc || offset < 0 || size <= 0 || cmd == 0 ||
	    (offset + size) > _depc_size) {
		pr_err("DTS_EAGLE_DRIVER_SENDCACHE_PRE: in %s, primary device %i cache index %i general error - cache size = %u, cache ptr = %p, offset = %i, size = %i, cmd = %i\n",
			__func__, _device_primary, cidx, _depc_size, _depc,
			offset, size, cmd);
		return -EINVAL;
	}

	pr_debug("DTS_EAGLE_DRIVER_SENDCACHE_PRE: first 6 integers %i %i %i %i %i %i (30th %i)\n",
		  *((int *)&_depc[offset]), *((int *)&_depc[offset+4]),
		  *((int *)&_depc[offset+8]), *((int *)&_depc[offset+12]),
		  *((int *)&_depc[offset+16]), *((int *)&_depc[offset+20]),
		  *((int *)&_depc[offset+120]));
	pr_debug("DTS_EAGLE_DRIVER_SENDCACHE_PRE: sending full data block to port, with cache index = %d device mask 0x%X, param = 0x%X, offset = %d, and size = %d\n",
		  cidx, _c_bl[cidx][CBD_DEV_MASK], cmd, offset, size);

	if (q6asm_dts_eagle_set(ac, cmd, size, (void *)&_depc[offset],
				NULL, MPRE)) {
		pr_err("DTS_EAGLE_DRIVER_SENDCACHE_PRE: in %s, q6asm_dts_eagle_set failed with id = %d and size = %d\n",
			__func__, cmd, size);
	} else {
		pr_debug("DTS_EAGLE_DRIVER_SENDCACHE_PRE: in %s, q6asm_dts_eagle_set succeeded with id = %d and size = %d\n",
			 __func__, cmd, size);
	}
	return 0;
}

static int _sendcache_post(int port_id, int copp_idx, int topology)
{
	int offset, cidx = -1, size, cmd, mask, index;

	if (port_id == -1) {
		cidx = _get_cb_for_dev(_device_primary);
		goto NT_MODE_GOTO;
	}

	index = adm_validate_and_get_port_index(port_id);
	if (index < 0) {
		pr_err("DTS_EAGLE_DRIVER_SENDCACHE_POST :%s: Invalid port idx %d port_id %#x\n",
			__func__, index, port_id);
	} else {
		pr_debug("DTS_EAGLE_DRIVER_SENDCACHE_POST : %s valid port idx %d for port_id %#x set to %i",
			 __func__, index, port_id, copp_idx);
	}
	_cidx[index] = copp_idx;

	mask = _get_dev_mask_for_pid(port_id);
	if (mask & _device_primary) {
		cidx = _get_cb_for_dev(_device_primary);
		if (cidx < 0) {
			pr_err("DTS_EAGLE_DRIVER_SENDCACHE_POST: in %s, no cache for primary device %i found. Port id was 0x%X.\n",
				__func__, _device_primary, port_id);
			return -EINVAL;
		}
	} else if (mask & _device_all) {
		cidx = _get_cb_for_dev(_device_all);
		if (cidx < 0) {
			pr_err("DTS_EAGLE_DRIVER_SENDCACHE_POST: in %s, no cache for combo device %i found. Port id was 0x%X.\n",
				__func__, _device_primary, port_id);
			return -EINVAL;
		}
	} else {
		pr_err("DTS_EAGLE_DRIVER_SENDCACHE_POST: in %s, port id 0x%X not for primary or combo device %i.\n",
			__func__, port_id, _device_primary);
		return -EINVAL;
	}

NT_MODE_GOTO:
	offset = _c_bl[cidx][CBD_OFFSG] + _c_bl[cidx][CBD_OFFS2];
	cmd = _c_bl[cidx][CBD_CMD2];
	size = _c_bl[cidx][CBD_SZ2];

	if (_depc_size == 0 || !_depc || offset < 0 || size <= 0 || cmd == 0 ||
	    (offset + size) > _depc_size) {
		pr_err("DTS_EAGLE_DRIVER_SENDCACHE_POST: in %s, primary device %i cache index %i port_id 0x%X general error - cache size = %u, cache ptr = %p, offset = %i, size = %i, cmd = %i\n",
			__func__, _device_primary, cidx, port_id,
			_depc_size, _depc, offset, size, cmd);
		return -EINVAL;
	}

	pr_debug("DTS_EAGLE_DRIVER_SENDCACHE_POST: first 6 integers %i %i %i %i %i %i\n",
		  *((int *)&_depc[offset]), *((int *)&_depc[offset+4]),
		  *((int *)&_depc[offset+8]), *((int *)&_depc[offset+12]),
		  *((int *)&_depc[offset+16]), *((int *)&_depc[offset+20]));
	pr_debug("DTS_EAGLE_DRIVER_SENDCACHE_POST: sending full data block to port, with cache index = %d device mask 0x%X, port_id = 0x%X, param = 0x%X, offset = %d, and size = %d\n",
		  cidx, _c_bl[cidx][CBD_DEV_MASK], port_id, cmd, offset, size);

	if (_ac_NT) {
		pr_debug("DTS_EAGLE_DRIVER_SENDCACHE_POST: NT Route detected\n");
		if (q6asm_dts_eagle_set(_getNTDeviceAC(), cmd, size,
					(void *)&_depc[offset],
					&_po_NT, MPST)) {
			pr_err("DTS_EAGLE_DRIVER_SENDCACHE_POST: in %s, q6asm_dts_eagle_set failed with id = 0x%X and size = %d\n",
			__func__, cmd, size);
		}
	} else if (adm_dts_eagle_set(port_id, copp_idx, cmd,
			      (void *)&_depc[offset], size) < 0) {
		pr_err("DTS_EAGLE_DRIVER_SENDCACHE_POST: in %s, adm_dts_eagle_set failed with id = 0x%X and size = %d\n",
			__func__, cmd, size);
	} else {
		pr_debug("DTS_EAGLE_DRIVER_SENDCACHE_POST: in %s, adm_dts_eagle_set succeeded with id = 0x%X and size = %d\n",
			 __func__, cmd, size);
	}
	return 0;
}

static int _enable_post_get_control(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = _is_hpx_enabled;
	return 0;
}

static int _enable_post_put_control(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	int idx = 0, be_index = 0, port_id, topology;
	int flag = ucontrol->value.integer.value[0];
	struct msm_pcm_routing_bdai_data msm_bedai;
	pr_debug("%s flag %d\n", __func__, flag);

	if ((_is_hpx_enabled && flag) || (!_is_hpx_enabled && !flag))
		return 0;

	_is_hpx_enabled = flag ? true : false;
	msm_pcm_routing_acquire_lock();
	/* send cache postmix params when hpx is set On */
	for (be_index = 0; be_index < MSM_BACKEND_DAI_MAX; be_index++) {
		msm_pcm_routing_get_bedai_info(be_index, &msm_bedai);
		port_id = msm_bedai.port_id;
		if (!(((port_id == SLIMBUS_0_RX) ||
		      (port_id == RT_PROXY_PORT_001_RX)) &&
		      msm_bedai.active))
			continue;
		for (idx = 0; idx < MAX_COPPS_PER_PORT; idx++) {
			topology = adm_get_topology_for_port_from_copp_id(
								port_id, idx);
			if (topology ==
				ADM_CMD_COPP_OPEN_TOPOLOGY_ID_DTS_HPX_0) {
				msm_dts_eagle_enable_adm(port_id, idx,
							 _is_hpx_enabled);
			}
		}
	}
	msm_pcm_routing_release_lock();
	return 0;
}

static const struct snd_kcontrol_new _hpx_enabled_controls[] = {
	SOC_SINGLE_EXT("Set HPX OnOff", SND_SOC_NOPM, 0, 1, 0,
	_enable_post_get_control, _enable_post_put_control)
};

/*//////////////////*/
/* EAGLE KERNEL API */
/*//////////////////*/

void msm_dts_ion_memmap(struct param_outband *po_)
{
	po_->size = ION_MEM_SIZE;
	po_->kvaddr = _po.kvaddr;
	po_->paddr = _po.paddr;
}
int msm_dts_eagle_enable_asm(struct audio_client *ac, __u32 enable, int module)
{
	int ret = 0;
	pr_debug("DTS_EAGLE_ENABLE: %s - enable = %i on module %i\n",
		 __func__, enable, module);
	_is_hpx_enabled = enable;
	ret = q6asm_dts_eagle_set(ac, AUDPROC_PARAM_ID_ENABLE,
				      sizeof(enable), &enable,
				      NULL, module);
	if (_is_hpx_enabled) {
		if (module == MPRE)
			_sendcache_pre(ac);
		else if (module == MPST)
			_sendcache_post(-1, 0, 0);
	}
	return ret;
}

int msm_dts_eagle_enable_adm(int port_id, int copp_idx, __u32 enable)
{
	int ret = 0;
	pr_debug("DTS_EAGLE_ENABLE: %s - enable = %i\n", __func__, enable);
	_is_hpx_enabled = enable;
	ret = adm_dts_eagle_set(port_id, copp_idx, AUDPROC_PARAM_ID_ENABLE,
			     (char *)&enable, sizeof(enable));
	if (_is_hpx_enabled)
		_sendcache_post(port_id, copp_idx, MPST);
	return ret;
}

int msm_dts_eagle_enable_master(struct audio_client *ac, __u32 enable)
{
	int ret = msm_dts_eagle_enable_asm(ac, enable, MPRE);
	if (ret >= 0)
		ret = msm_dts_eagle_enable_asm(ac, enable, MPST);
	return ret;
}

void msm_dts_eagle_add_controls(struct snd_soc_platform *platform)
{
	snd_soc_add_platform_controls(platform, _hpx_enabled_controls,
				      ARRAY_SIZE(_hpx_enabled_controls));
}

int msm_dts_eagle_set_stream_gain(struct audio_client *ac, int lgain, int rgain)
{
	__s32 i, val;
	__u32 idx;

	pr_debug("DTS_EAGLE_DRIVER_VOLUME: %s - entry: vol_cmd_cnt = %i, lgain = %i, rgain = %i",
		 __func__, _vol_cmd_cnt, lgain, rgain);

	if (_depc_size == 0) {
		pr_err("DTS_EAGLE_DRIVER_VOLUME: driver cache not initialized.\n");
		return -EINVAL;
	}

	for (i = 0; i < _vol_cmd_cnt; i++) {
		if (_vol_cmds_d[i].d[0] & 0x8000) {
			idx = (sizeof(struct dts_eagle_param_desc)/sizeof(int))
				+ (_vol_cmds_d[i].d[0] & 0x3FF);
			val = _fx_logN(((__s32)(lgain+rgain)) << 2);
			val = ((long long)val * _log10_10_inv_x20) >> 16;
			_vol_cmds[i][idx] = (__s32)clamp((int)(((long long)val *
						    _vol_cmds_d[i].d[1]) >> 16),
						    _vol_cmds_d[i].d[2],
						    _vol_cmds_d[i].d[3]);
			pr_debug("DTS_EAGLE_DRIVER_VOLUME: loop %i cmd desc found %i, idx = %i. volume info: lgain = %i, rgain = %i, volume = %i (scale %i, min %i, max %i)\n",
				 i, _vol_cmds_d[i].d[0], idx, lgain,
				 rgain, _vol_cmds[i][idx], _vol_cmds_d[i].d[1],
				 _vol_cmds_d[i].d[2], _vol_cmds_d[i].d[3]);
		}
		idx = _get_cb_for_dev(_device_primary);
		val = _c_bl[idx][CBD_OFFSG] + _vol_cmds[i][2];
		if ((val + _vol_cmds[i][1]) > _depc_size) {
			pr_err("DTS_EAGLE_DRIVER_VOLUME: volume size (%i) + offset (%i) out of bounds %i.\n",
				val, _vol_cmds[i][1], _depc_size);
			return -EINVAL;
		}
		memcpy((void *)&_depc[val], &_vol_cmds[i][4], _vol_cmds[i][1]);
		if (q6asm_dts_eagle_set(ac, _vol_cmds[i][0],
			_vol_cmds[i][1], (void *)&_depc[val], NULL, MPRE)) {
			pr_err("DTS_EAGLE_DRIVER_VOLUME: loop %i - volume set failed with id 0x%X, size %i, offset %i, cmd_desc %i, scale %i, min %i, max %i, data(...) %i\n",
				i, _vol_cmds[i][0], _vol_cmds[i][1],
				_vol_cmds[i][2], _vol_cmds_d[i].d[0],
				_vol_cmds_d[i].d[1], _vol_cmds_d[i].d[2],
				_vol_cmds_d[i].d[3], _vol_cmds[i][4]);
		} else {
			pr_debug("DTS_EAGLE_DRIVER_VOLUME: loop %i - volume set succeeded with id 0x%X, size %i, offset %i, cmd_desc %i, scale %i, min %i, max %i, data(...) %i\n",
				 i, _vol_cmds[i][0], _vol_cmds[i][1],
				 _vol_cmds[i][2], _vol_cmds_d[i].d[0],
				 _vol_cmds_d[i].d[1], _vol_cmds_d[i].d[2],
				 _vol_cmds_d[i].d[3], _vol_cmds[i][4]);
		}
	}
	return 0;
}

int msm_dts_eagle_handle_asm(struct dts_eagle_param_desc *depd, char *buf,
			     bool for_pre, bool get, struct audio_client *ac,
			     struct param_outband *po)
{
	struct dts_eagle_param_desc depd_;
	__s32 offset, ret = 0, i, mod = for_pre ? MPRE : MPST;

	pr_debug("DTS_EAGLE_DRIVER_ASM: %s called (set pre-param)\n", __func__);

	/* special handling for ALSA route, to accommodate 64 bit platforms */
	if (depd == NULL) {
		long *arg_ = (long *)buf;
		depd = &depd_;
		depd->id = (__u32)*arg_++;
		depd->size = (__s32)*arg_++;
		depd->offset = (__s32)*arg_++;
		depd->device = (__u32)*arg_++;
		buf = (char *)arg_;
	}

	if (depd->size & 3) {
		pr_err("DTS_EAGLE_DRIVER_ASM: parameter size %i is not a multiple of 4\n",
			depd->size);
		return -EINVAL;
	}

	if (get) {
		void *buf, *buf_m = NULL;
		pr_debug("DTS_EAGLE_DRIVER_ASM: get requested\n");
		if (depd->offset == -1) {
			pr_debug("DTS_EAGLE_DRIVER_ASM: get from dsp requested\n");
			buf = buf_m = kzalloc(depd->size, GFP_KERNEL);
			if (!buf_m) {
				pr_err("DTS_EAGLE_DRIVER_ASM: out of memory\n");
				return -ENOMEM;
			} else if (q6asm_dts_eagle_get(ac, depd->id,
						       depd->size, buf_m,
						       po, mod) < 0) {
				pr_debug("DTS_EAGLE_DRIVER_ASM: asm failed, trying core\n");
				if (core_dts_eagle_get(depd->id, depd->size,
						       buf) < 0) {
					pr_err("DTS_EAGLE_DRIVER_ASM: get from qdsp failed (topology not eagle or closed)\n");
					ret = -EFAULT;
					goto DTS_EAGLE_IOCTL_GET_PARAM_PRE_EXIT;
				}
			}
		} else {
			__u32 tgt = _get_cb_for_dev(depd->device);
			offset = _c_bl[tgt][CBD_OFFSG] + depd->offset;
			if ((offset + depd->size) > _depc_size) {
				pr_err("DTS_EAGLE_DRIVER_ASM: invalid size %d and/or offset %d\n",
					depd->size, offset);
				return -EINVAL;
			}
			buf = (__u32 *)&_depc[offset];
		}
		if (depd == NULL) {
			__u32 *pbuf = (__u32 *)buf;
			for (i = 0; i < (depd->size >> 2); i++)
				*(long *)buf++ = (long)*pbuf++;
		} else {
			memcpy(buf, &_depc[offset], depd->size);
		}
DTS_EAGLE_IOCTL_GET_PARAM_PRE_EXIT:
		kfree(buf_m);
		return (int)ret;
	} else {
		__u32 tgt = _get_cb_for_dev(depd->device);
		offset = _c_bl[tgt][CBD_OFFSG] + depd->offset;
		if ((offset + depd->size) > _depc_size) {
			pr_err("DTS_EAGLE_DRIVER_ASM: invalid size %i and/or offset %i for parameter (cache is size %u)\n",
					depd->size, offset, _depc_size);
			return -EINVAL;
		}
		if (depd == NULL) {
			__u32 *pbuf = (__u32 *)(&_depc[offset]);
			for (i = 0; i < (depd->size >> 2); i++)
				*pbuf++ = (__u32)*(long *)buf++;
		} else {
			memcpy(&_depc[offset], buf, depd->size);
		}
		pr_debug("DTS_EAGLE_DRIVER_ASM: param info: param = 0x%X, size = %i, offset = %i, device = %i, cache block %i, global offset = %i, first bytes as integer = %i.\n",
			  depd->id, depd->size, depd->offset, depd->device,
			  tgt, offset, *(int *)&_depc[offset]);
		if (q6asm_dts_eagle_set(ac, depd->id, depd->size,
					(void *)&_depc[offset], po, mod)) {
			pr_err("DTS_EAGLE_DRIVER_ASM: q6asm_dts_eagle_set failed with id = 0x%X, size = %d, offset = %d\n",
				depd->id, depd->size, depd->offset);
		} else {
			pr_debug("DTS_EAGLE_DRIVER_ASM: q6asm_dts_eagle_set succeeded with id = 0x%X, size = %d, offset = %d\n",
				 depd->id, depd->size, depd->offset);
		}
	}
	return (int)ret;
}

int msm_dts_eagle_handle_adm(struct dts_eagle_param_desc *depd, char *buf,
			     bool for_pre, bool get)
{
	__u32 pid = _get_pid_from_dev(depd->device), cidx;
	__s32 ret = 0;
	if (_isNTDevice(depd->device)) {
		pr_debug("DTS_EAGLE_DRIVER_ADM: NT Route detected\n");
		ret = msm_dts_eagle_handle_asm(depd, buf, for_pre, get,
					       _getNTDeviceAC(), &_po_NT);
		if (ret < 0) {
			pr_debug("DTS_EAGLE_DRIVER_ADM: NT Route set failed with id = 0x%X, size = %i, offset = %i, device = %i\n",
				depd->id, depd->size, depd->offset,
				depd->device);
		}
	} else if (get) {
		__u32 pid = _get_pid_from_dev(depd->device), cidx;
		cidx = adm_validate_and_get_port_index(pid);
		pr_debug("DTS_EAGLE_DRIVER_ADM: get from qdsp requested (port id 0x%X)\n",
			 pid);
		if (adm_dts_eagle_get(pid, _cidx[cidx], depd->id,
				      buf, depd->size) < 0) {
			pr_debug("DTS_EAGLE_DRIVER_ADM: get from qdsp via adm with port id 0x%X failed, trying core\n",
				 pid);
			if (core_dts_eagle_get(depd->id, depd->size,
						buf) < 0) {
				pr_err("DTS_EAGLE_DRIVER_ADM: get from qdsp failed\n");
				ret = -EFAULT;
				return ret;
			}
		}
	} else if (_is_port_open_and_eagle(pid)) {
		cidx = adm_validate_and_get_port_index(pid);
		ret = adm_dts_eagle_set(pid, _cidx[cidx], depd->id,
					(void *)buf, depd->size);
		if (ret < 0) {
			pr_err("DTS_EAGLE_DRIVER_ADM: adm_dts_eagle_set failed with id = 0x%X, size = %i, offset = %i, device = %i, port id = %u, copp index = %u\n",
				depd->id, depd->size, depd->offset,
				depd->device, pid, cidx);
		} else {
			pr_debug("DTS_EAGLE_DRIVER_ADM: adm_dts_eagle_set succeeded with id = 0x%X, size = %i, offset = %i, device = %i, port id = %u, copp index = %u\n",
				depd->id, depd->size, depd->offset,
				depd->device, pid, cidx);
		}
	} else {
		ret = -EINVAL;
		pr_debug("DTS_EAGLE_DRIVER_ADM: port id 0x%X not active or not Eagle\n",
			 pid);
	}
	return (int)ret;
}

int msm_dts_eagle_ioctl(unsigned int cmd, unsigned long arg)
{
	__s32 ret = 0;
	switch (cmd) {
	case DTS_EAGLE_IOCTL_GET_CACHE_SIZE: {
		pr_debug("DTS_EAGLE_DRIVER_IOCTL: %s called with control 0x%X (get param cache size)\n",
			__func__, cmd);
		if (copy_to_user((void *)arg, &_depc_size,
				 sizeof(_depc_size))) {
			pr_err("DTS_EAGLE_DRIVER_IOCTL: error writing size\n");
			return -EFAULT;
		}
		break;
	}
	case DTS_EAGLE_IOCTL_SET_CACHE_SIZE: {
		__s32 size = 0;
		pr_debug("DTS_EAGLE_DRIVER_IOCTL: %s called with control 0x%X (allocate param cache)\n",
			__func__, cmd);
		if (copy_from_user((void *)&size, (void *)arg, sizeof(size))) {
			pr_err("DTS_EAGLE_DRIVER_IOCTL: error copying size (src:%p, tgt:%p, size:%zu)\n",
				(void *)arg, &size, sizeof(size));
			return -EFAULT;
		} else if (size < 0 || size > DEPC_MAX_SIZE) {
			pr_err("DTS_EAGLE_DRIVER_IOCTL: cache size %d not allowed (min 0, max %d)\n",
				size, DEPC_MAX_SIZE);
			return -EINVAL;
		}
		if (_depc) {
			pr_debug("DTS_EAGLE_DRIVER_IOCTL: previous param cache of size %u freed\n",
				_depc_size);
			_depc_size = 0;
			kfree(_depc);
			_depc = NULL;
		}
		if (size) {
			_depc = kzalloc(size, GFP_KERNEL);
		} else {
			pr_debug("DTS_EAGLE_DRIVER_IOCTL: %d bytes requested for param cache, nothing allocated\n",
				size);
		}
		if (_depc) {
			pr_debug("DTS_EAGLE_DRIVER_IOCTL: %d bytes allocated for param cache\n",
				size);
			_depc_size = size;
		} else {
			pr_err("DTS_EAGLE_DRIVER_IOCTL: error allocating param cache (kzalloc failed on %d bytes)\n",
				size);
			_depc_size = 0;
			return -ENOMEM;
		}
		break;
	}
	case DTS_EAGLE_IOCTL_GET_PARAM: {
		struct dts_eagle_param_desc depd;
		__s32 offset = 0, for_pre = 0;
		void *buf, *buf_m = NULL;
		pr_debug("DTS_EAGLE_DRIVER_IOCTL: %s called, control 0x%X (get param)\n",
			__func__, cmd);
		if (copy_from_user((void *)&depd, (void *)arg, sizeof(depd))) {
			pr_err("DTS_EAGLE_DRIVER_IOCTL: error copying dts_eagle_param_desc (src:%p, tgt:%p, size:%zu)\n",
				(void *)arg, &depd, sizeof(depd));
			return -EFAULT;
		}
		if (depd.device & DTS_EAGLE_FLAG_IOCTL_PRE) {
			pr_debug("DTS_EAGLE_DRIVER_IOCTL: using for premix.\n");
			for_pre = 1;
		}
		depd.device &= DTS_EAGLE_FLAG_IOCTL_MASK;
		if (depd.offset == -1) {
			buf = buf_m = kzalloc(depd.size, GFP_KERNEL);
			if (!buf_m) {
				pr_err("DTS_EAGLE_DRIVER_IOCTL: out of memory\n");
				return -ENOMEM;
			}
			ret = msm_dts_eagle_handle_adm(&depd, buf,
						       for_pre, true);
		} else {
			__u32 cb = _get_cb_for_dev(depd.device);
			offset = _c_bl[cb][CBD_OFFSG] + depd.offset;
			if ((offset + depd.size) > _depc_size) {
				pr_err("DTS_EAGLE_DRIVER_IOCTL: invalid size %d and/or offset %d\n",
					depd.size, offset);
				return -EINVAL;
			}
			buf = (void *)&_depc[offset];
		}
		if (copy_to_user((void *)(((char *)arg)+sizeof(depd)),
						  buf, depd.size)) {
			pr_err("DTS_EAGLE_DRIVER_IOCTL: error getting param\n");
			ret = -EFAULT;
		}
		kfree(buf_m);
		break;
	}
	case DTS_EAGLE_IOCTL_SET_PARAM: {
		struct dts_eagle_param_desc depd;
		__s32 offset = 0, just_set_cache = 0, for_pre = 0;
		__u32 tgt;
		pr_debug("DTS_EAGLE_DRIVER_IOCTL: %s called, control 0x%X (set param)\n",
			__func__, cmd);
		if (copy_from_user((void *)&depd, (void *)arg, sizeof(depd))) {
			pr_err("DTS_EAGLE_DRIVER_IOCTL: error copying dts_eagle_param_desc (src:%p, tgt:%p, size:%zu)\n",
				(void *)arg, &depd, sizeof(depd));
			return -EFAULT;
		}
		if (depd.device & DTS_EAGLE_FLAG_IOCTL_PRE) {
			pr_debug("DTS_EAGLE_DRIVER_IOCTL: using for premix.\n");
			for_pre = 1;
		}
		if (depd.device & DTS_EAGLE_FLAG_IOCTL_JUSTSETCACHE) {
			pr_debug("DTS_EAGLE_DRIVER_IOCTL: 'just set cache' requested.\n");
			just_set_cache = 1;
		}
		depd.device &= DTS_EAGLE_FLAG_IOCTL_MASK;
		tgt = _get_cb_for_dev(depd.device);
		offset = _c_bl[tgt][CBD_OFFSG] + depd.offset;
		if ((offset + depd.size) > _depc_size) {
			pr_err("DTS_EAGLE_DRIVER_IOCTL: invalid size %i and/or offset %i for parameter (target cache block %i with offset %i, global cache is size %u)\n",
				depd.size, offset, tgt,
				_c_bl[tgt][CBD_OFFSG], _depc_size);
			return -EINVAL;
		}
		if (copy_from_user((void *)&_depc[offset],
				   (void *)(((char *)arg)+sizeof(depd)),
					depd.size)) {
			pr_err("DTS_EAGLE_DRIVER_IOCTL: error copying param to cache (src:%p, tgt:%p, size:%i)\n",
				((char *)arg)+sizeof(depd),
				&_depc[offset], depd.size);
			return -EFAULT;
		}
		pr_debug("DTS_EAGLE_DRIVER_IOCTL: param info: param = 0x%X, size = %i, offset = %i, device = %i, cache block %i, global offset = %i, first bytes as integer = %i.\n",
			  depd.id, depd.size, depd.offset, depd.device,
			  tgt, offset, *(int *)&_depc[offset]);
		if (!just_set_cache) {
			ret = msm_dts_eagle_handle_adm(&depd, &_depc[offset],
						       for_pre, false);
		}
		break;
	}
	case DTS_EAGLE_IOCTL_SET_CACHE_BLOCK: {
		__u32 b_[CBD_COUNT+1], *b = &b_[1], cb;
		pr_debug("DTS_EAGLE_DRIVER_IOCTL: %s called with control 0x%X (set param cache block)\n",
			 __func__, cmd);
		if (copy_from_user((void *)b_, (void *)arg, sizeof(b_))) {
			pr_err("DTS_EAGLE_DRIVER_IOCTL: error copying cache block data (src:%p, tgt:%p, size:%zu)\n",
				(void *)arg, b_, sizeof(b_));
			return -EFAULT;
		}
		cb = b_[0];
		if (cb >= CB_COUNT) {
			pr_err("DTS_EAGLE_DRIVER_IOCTL: cache block %u out of range (max %u)\n",
			cb, CB_COUNT-1);
			return -EINVAL;
		}
		if ((b[CBD_OFFSG]+b[CBD_OFFS1]+b[CBD_SZ1]) >= _depc_size ||
			(b[CBD_OFFSG]+b[CBD_OFFS2]+b[CBD_SZ2]) >= _depc_size ||
			(b[CBD_OFFSG]+b[CBD_OFFS3]+b[CBD_SZ3]) >= _depc_size) {
			pr_err("DTS_EAGLE_DRIVER_IOCTL: cache block bounds out of range\n");
			return -EINVAL;
		}
		memcpy(_c_bl[cb], b, sizeof(_c_bl[cb]));
		pr_debug("DTS_EAGLE_DRIVER_IOCTL: cache block %i set: devices 0x%X, global offset %u, offsets 1:%u 2:%u 3:%u, cmds/sizes 0:0x%X %u 1:0x%X %u 2:0x%X %u 3:0x%X %u\n",
		cb, _c_bl[cb][CBD_DEV_MASK], _c_bl[cb][CBD_OFFSG],
		_c_bl[cb][CBD_OFFS1], _c_bl[cb][CBD_OFFS2],
		_c_bl[cb][CBD_OFFS3], _c_bl[cb][CBD_CMD0], _c_bl[cb][CBD_SZ0],
		_c_bl[cb][CBD_CMD1], _c_bl[cb][CBD_SZ1], _c_bl[cb][CBD_CMD2],
		_c_bl[cb][CBD_SZ2], _c_bl[cb][CBD_CMD3], _c_bl[cb][CBD_SZ3]);
		break;
	}
	case DTS_EAGLE_IOCTL_SET_ACTIVE_DEVICE: {
		__u32 data[2];
		pr_debug("DTS_EAGLE_DRIVER_IOCTL: %s called with control 0x%X (set active device)\n",
			 __func__, cmd);
		if (copy_from_user((void *)data, (void *)arg, sizeof(data))) {
			pr_err("DTS_EAGLE_DRIVER_IOCTL: error copying active device data (src:%p, tgt:%p, size:%zu)\n",
				(void *)arg, data, sizeof(data));
			return -EFAULT;
		}
		if (data[1] != 0) {
			_device_primary = data[0];
			pr_debug("DTS_EAGLE_DRIVER_IOCTL: primary device %i\n",
				 data[0]);
		} else {
			_device_all = data[0];
			pr_debug("DTS_EAGLE_DRIVER_IOCTL: all devices 0x%X\n",
				 data[0]);
		}
		break;
	}
	case DTS_EAGLE_IOCTL_GET_LICENSE: {
		__u32 target = 0;
		__s32 size = 0, size_only;
		pr_debug("DTS_EAGLE_DRIVER_IOCTL: %s called with control 0x%X (get license)\n",
			 __func__, cmd);
		if (copy_from_user((void *)&target, (void *)arg,
				   sizeof(target))) {
			pr_err("DTS_EAGLE_DRIVER_IOCTL: error reading license index. (src:%p, tgt:%p, size:%zu)\n",
				(void *)arg, &target, sizeof(target));
			return -EFAULT;
		}
		size_only = target & (1<<31) ? 1 : 0;
		target &= 0x7FFFFFFF;
		if (target < 0 || target >= SEC_BLOB_MAX_CNT) {
			pr_err("DTS_EAGLE_DRIVER_IOCTL: license index %i out of bounds (max index is %i)\n",
				   target, SEC_BLOB_MAX_CNT);
			return -EINVAL;
		}
		if (_sec_blob[target] == NULL) {
			pr_err("DTS_EAGLE_DRIVER_IOCTL: license index %i never initialized.\n",
				   target);
			return -EINVAL;
		}
		size = ((__s32 *)_sec_blob[target])[0];
		if (size <= 0 || size > SEC_BLOB_MAX_SIZE) {
			pr_err("DTS_EAGLE_DRIVER_IOCTL: license size %i for index %i invalid (min size is 1, max size is %i).\n",
				   size, target, SEC_BLOB_MAX_SIZE);
			return -EINVAL;
		}
		if (size_only) {
			pr_debug("DTS_EAGLE_DRIVER_IOCTL: reporting size of license data only\n");
			if (copy_to_user((void *)(((char *)arg)+sizeof(target)),
				 (void *)&size, sizeof(size))) {
				pr_err("DTS_EAGLE_DRIVER_IOCTL: error copying license size.\n");
				return -EFAULT;
			}
		} else if (copy_to_user((void *)(((char *)arg)+sizeof(target)),
			   (void *)&(((__s32 *)_sec_blob[target])[1]), size)) {
			pr_err("DTS_EAGLE_DRIVER_IOCTL: error copying license data.\n");
			return -EFAULT;
		} else {
			pr_debug("DTS_EAGLE_DRIVER_IOCTL: license file %i bytes long from license index %i returned to user.\n",
				  size, target);
		}
		break;
	}
	case DTS_EAGLE_IOCTL_SET_LICENSE: {
		__s32 target[2] = {0, 0};
		pr_debug("DTS_EAGLE_DRIVER_IOCTL: %s called with control 0x%X (set license)\n",
			 __func__, cmd);
		if (copy_from_user((void *)target, (void *)arg,
				   sizeof(target))) {
			pr_err("DTS_EAGLE_DRIVER_IOCTL: error reading license index (src:%p, tgt:%p, size:%zu)\n",
				(void *)arg, target, sizeof(target));
			return -EFAULT;
		}
		if (target[0] < 0 || target[0] >= SEC_BLOB_MAX_CNT) {
			pr_err("DTS_EAGLE_DRIVER_IOCTL: license index %i out of bounds (max index is %i)\n",
				   target[0], SEC_BLOB_MAX_CNT-1);
			return -EINVAL;
		}
		if (target[1] == 0) {
			pr_debug("DTS_EAGLE_DRIVER_IOCTL: request to free license index %i\n",
				 target[0]);
			kfree(_sec_blob[target[0]]);
			_sec_blob[target[0]] = NULL;
			break;
		}
		if (target[1] <= 0 || target[1] >= SEC_BLOB_MAX_SIZE) {
			pr_err("DTS_EAGLE_DRIVER_IOCTL: license size %i for index %i invalid (min size is 1, max size is %i).\n",
				   target[1], target[0], SEC_BLOB_MAX_SIZE);
			return -EINVAL;
		}
		if (_sec_blob[target[0]] != NULL) {
			if (((__s32 *)_sec_blob[target[0]])[1] != target[1]) {
				pr_debug("DTS_EAGLE_DRIVER_IOCTL: request new size for already allocated license index %i\n",
					 target[0]);
				kfree(_sec_blob[target[0]]);
				_sec_blob[target[0]] = NULL;
			}
		}
		pr_debug("DTS_EAGLE_DRIVER_IOCTL: allocating %i bytes for license index %i\n",
				  target[1], target[0]);
		_sec_blob[target[0]] = kzalloc(target[1] + 4, GFP_KERNEL);
		if (!_sec_blob[target[0]]) {
			pr_err("DTS_EAGLE_DRIVER_IOCTL: error allocating license index %i (kzalloc failed on %i bytes)\n",
					target[0], target[1]);
			return -ENOMEM;
		}
		((__s32 *)_sec_blob[target[0]])[0] = target[1];
		if (copy_from_user(
				(void *)&(((__s32 *)_sec_blob[target[0]])[1]),
				(void *)(((char *)arg)+sizeof(target)),
				target[1])) {
			pr_err("DTS_EAGLE_DRIVER_IOCTL: error copying license to index %i, size %i (src:%p, tgt:%p, size:%i)\n",
					target[0], target[1],
					((char *)arg)+sizeof(target),
					&(((__s32 *)_sec_blob[target[0]])[1]),
					target[1]);
			return -EFAULT;
		} else {
			pr_debug("DTS_EAGLE_DRIVER_IOCTL: license file %i bytes long copied to index license index %i\n",
				  target[1], target[0]);
		}
		break;
	}
	case DTS_EAGLE_IOCTL_SEND_LICENSE: {
		__s32 target = 0;
		pr_debug("DTS_EAGLE_DRIVER_IOCTL: %s called with control 0x%X (send license)\n",
			 __func__, cmd);
		if (copy_from_user((void *)&target, (void *)arg,
				   sizeof(target))) {
			pr_err("DTS_EAGLE_DRIVER_IOCTL: error reading license index (src:%p, tgt:%p, size:%zu)\n",
				(void *)arg, &target, sizeof(target));
			return -EFAULT;
		}
		if (target >= SEC_BLOB_MAX_CNT) {
			pr_err("DTS_EAGLE_DRIVER_IOCTL: license index %i out of bounds (max index is %i)\n",
					target, SEC_BLOB_MAX_CNT-1);
			return -EINVAL;
		}
		if (!_sec_blob[target] ||
		    ((__s32 *)_sec_blob[target])[0] <= 0) {
			pr_err("DTS_EAGLE_DRIVER_IOCTL: license index %i is invalid\n",
				target);
			return -EINVAL;
		}
		if (core_dts_eagle_set(((__s32 *)_sec_blob[target])[0],
				(char *)&((__s32 *)_sec_blob[target])[1]) < 0) {
			pr_err("DTS_EAGLE_DRIVER_IOCTL: core_dts_eagle_set failed with id = %i\n",
				target);
		} else {
			pr_debug("DTS_EAGLE_DRIVER_IOCTL: core_dts_eagle_set succeeded with id = %i\n",
				 target);
		}
		break;
	}
	case DTS_EAGLE_IOCTL_SET_VOLUME_COMMANDS: {
		__s32 spec = 0;
		pr_debug("DTS_EAGLE_DRIVER_IOCTL: %s called with control 0x%X (set volume commands)\n",
			 __func__, cmd);
		if (copy_from_user((void *)&spec, (void *)arg,
					sizeof(spec))) {
			pr_err("DTS_EAGLE_DRIVER_IOCTL: error reading volume command specifier (src:%p, tgt:%p, size:%zu)\n",
				(void *)arg, &spec, sizeof(spec));
			return -EFAULT;
		}
		if (spec & 0x80000000) {
			__u32 idx = (spec & 0x0000F000) >> 12;
			__s32 size = spec & 0x00000FFF;
			pr_debug("DTS_EAGLE_DRIVER_IOCTL: setting volume command %i size: %i\n",
				 idx, size);
			if (idx >= _vol_cmd_cnt) {
				pr_err("DTS_EAGLE_DRIVER_IOCTL: volume command index %i out of bounds (only %i allocated).\n",
					idx, _vol_cmd_cnt);
				return -EINVAL;
			}
			if (_volume_cmds_alloc2(idx, size) < 0) {
				pr_err("DTS_EAGLE_DRIVER_IOCTL: error allocating memory for volume controls.\n");
				return -ENOMEM;
			}
			if (copy_from_user((void *)&_vol_cmds_d[idx],
					(void *)(((char *)arg) + sizeof(int)),
					sizeof(struct vol_cmds_d))) {
				pr_err("DTS_EAGLE_DRIVER_IOCTL: error reading volume command descriptor (src:%p, tgt:%p, size:%zu)\n",
					((char *)arg) + sizeof(int),
					&_vol_cmds_d[idx],
					sizeof(struct vol_cmds_d));
				return -EFAULT;
			}
			pr_debug("DTS_EAGLE_DRIVER_IOCTL: setting volume command %i spec (size %zu): %i %i %i %i\n",
				  idx, sizeof(struct vol_cmds_d),
				  _vol_cmds_d[idx].d[0], _vol_cmds_d[idx].d[1],
				  _vol_cmds_d[idx].d[2], _vol_cmds_d[idx].d[3]);
			if (copy_from_user((void *)_vol_cmds[idx],
					(void *)(((char *)arg) + (sizeof(int) +
					sizeof(struct vol_cmds_d))), size)) {
				pr_err("DTS_EAGLE_DRIVER_IOCTL: error reading volume command string (src:%p, tgt:%p, size:%i)\n",
					((char *)arg) + (sizeof(int) +
					sizeof(struct vol_cmds_d)),
					_vol_cmds[idx], size);
				return -EFAULT;
			}
		} else {
			pr_debug("DTS_EAGLE_DRIVER_IOCTL: setting volume command size\n");
			if (spec < 0 || spec > VOL_CMD_CNT_MAX) {
				pr_err("DTS_EAGLE_DRIVER_IOCTL: volume command count %i out of bounds (min 0, max %i).\n",
				spec, VOL_CMD_CNT_MAX);
				return -EINVAL;
			} else if (spec == 0) {
				pr_debug("DTS_EAGLE_DRIVER_IOCTL: request to free volume commands.\n");
				_volume_cmds_free();
				break;
			}
			pr_debug("DTS_EAGLE_DRIVER_IOCTL: setting volume command size requested = %i\n",
				  spec);
			if (_volume_cmds_alloc1(spec) < 0) {
				pr_err("DTS_EAGLE_DRIVER_IOCTL: error allocating memory for volume controls.\n");
				return -ENOMEM;
			}
		}
		break;
	}
	default: {
		pr_err("DTS_EAGLE_DRIVER_IOCTL: %s called, control 0x%X (invalid control)\n",
			 __func__, cmd);
		ret = -EINVAL;
	}
	}
	return (int)ret;
}

int msm_dts_eagle_init_pre(struct audio_client *ac)
{
	return msm_dts_eagle_enable_asm(ac, _is_hpx_enabled,
				 AUDPROC_MODULE_ID_DTS_HPX_PREMIX);
}

int msm_dts_eagle_deinit_pre(struct audio_client *ac)
{
	return 0;
}

int msm_dts_eagle_init_post(int port_id, int copp_idx, int topology)
{
	return msm_dts_eagle_enable_adm(port_id, copp_idx, _is_hpx_enabled);
}

int msm_dts_eagle_deinit_post(int port_id, int topology)
{
	return 0;
}

int msm_dts_eagle_init_master_module(struct audio_client *ac)
{
	_set_audioclient(ac);
	msm_dts_eagle_enable_asm(ac, _is_hpx_enabled,
				 AUDPROC_MODULE_ID_DTS_HPX_PREMIX);
	msm_dts_eagle_enable_asm(ac, _is_hpx_enabled,
				 AUDPROC_MODULE_ID_DTS_HPX_POSTMIX);
	return 0;
}

int msm_dts_eagle_deinit_master_module(struct audio_client *ac)
{
	msm_dts_eagle_deinit_pre(ac);
	msm_dts_eagle_deinit_post(-1, 0);
	_clear_audioclient();
	return 0;
}

int msm_dts_eagle_pcm_new(struct snd_soc_pcm_runtime *runtime)
{
	if (!_ref_cnt++) {
		_init_cb_descs();
		_reg_ion_mem();
	}

	return 0;
}

void msm_dts_eagle_pcm_free(struct snd_pcm *pcm)
{
	if (!--_ref_cnt)
		_unreg_ion_mem();
}

MODULE_DESCRIPTION("DTS EAGLE platform driver");
MODULE_LICENSE("GPL v2");
