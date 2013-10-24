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
 */

#define DEBUG

#include <linux/init.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/kernel.h>
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
#include <sound/asound.h>
#include <sound/hwdep.h>

#include "msm-pcm-routing-v2.h"
#include "msm-dts-eagle.h"
#include "q6core.h"

#define ION_MEM_SIZE 131072
#define DEPC_MAX_SIZE 0x200000

/* backend cache links */
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
enum {
    BETD_ID = 0,
    BETD_OFFSET,
    BETD_COMMAND,
    BETD_SIZE,

    BETD_COUNT,
};

static int fx_logN(int x) {
  int t, y=0xa65af;
  if(x<0x00008000){x<<=16,y-=0xb1721;} if(x<0x00800000){x<<= 8,y-=0x58b91;}
  if(x<0x08000000){x<<= 4,y-=0x2c5c8;} if(x<0x20000000){x<<= 2,y-=0x162e4;}
  if(x<0x40000000){x<<= 1,y-=0x0b172;}
  t=x+(x>>1); if((t&0x80000000)==0){x=t, y-=0x067cd;}
  t=x+(x>>2); if((t&0x80000000)==0){x=t, y-=0x03920;}
  t=x+(x>>3); if((t&0x80000000)==0){x=t, y-=0x01e27;}
  t=x+(x>>4); if((t&0x80000000)==0){x=t, y-=0x00f85;}
  t=x+(x>>5); if((t&0x80000000)==0){x=t, y-=0x007e1;}
  t=x+(x>>6); if((t&0x80000000)==0){x=t, y-=0x003f8;}
  t=x+(x>>7); if((t&0x80000000)==0){x=t, y-=0x001fe;}
  x=0x80000000-x; y-=x>>15; return y;
}

static int ref_cnt = 0;
/* dts eagle parameter cache */
static char *_depc = NULL;
static unsigned int _depc_size = 0;
static unsigned int _cache_targets[AUDIO_DEVICE_OUT_COUNT][BETD_COUNT];
static unsigned int _pre_cmd = 0;
static unsigned int _pre_size = 0;
/* ION states */
static struct ion_client        *ion_client = NULL;
static struct ion_handle        *ion_handle = NULL;
static ion_phys_addr_t paddr = 0;
static size_t pa_len = 0;
static void *vaddr = NULL;

#define SEC_BLOB_MAX_CNT 10
#define SEC_BLOB_MAX_SIZE 0x4004 /*extra 4 for size*/
static char* sec_blob[SEC_BLOB_MAX_CNT] = {NULL};

// volume controls
#define VOL_CMD_CNT_MAX 10
static int vol_cmd_cnt = 0;
static int** vol_cmds = NULL;
typedef int vol_cmds_desc_[4];
static vol_cmds_desc_* vol_cmds_desc = NULL;

static void volume_cmds_free(void) {
    int i;
    for(i = 0; i < vol_cmd_cnt; i++) {
        if(vol_cmds[i])
            kfree(vol_cmds[i]);
    }
    vol_cmd_cnt = 0;
    if(vol_cmds)
        kfree(vol_cmds);
    if(vol_cmds_desc)
        kfree(vol_cmds_desc);
    vol_cmds = NULL;
    vol_cmds_desc = NULL;
}

static int volume_cmds_alloc1(int size) {
    volume_cmds_free();
    vol_cmd_cnt = size;
    vol_cmds = kzalloc(vol_cmd_cnt * sizeof(int*), GFP_KERNEL);
    if(vol_cmds)
        vol_cmds_desc = kzalloc(vol_cmd_cnt * sizeof(vol_cmds_desc_), GFP_KERNEL);
    if(vol_cmds_desc)
        return 0;
    return -1;
}

// assumes size is equal or less than 0xFFF
static int volume_cmds_alloc2(int idx, int size) {
    if(vol_cmds[idx])
        kfree(vol_cmds[idx]);
    vol_cmds[idx] = kzalloc(size, GFP_KERNEL);
    if(vol_cmds[idx])
        return 0;
    vol_cmds_desc[idx][0] = 0;
    return -1;
}

static void _setup_cache_target_mapping(void)
{
    int i;
    pr_info("DTS_EAGLE_DRIVER: %s called\n", __func__);
    for (i = 0; i < AUDIO_DEVICE_OUT_COUNT; i++) {
        _cache_targets[i][BETD_OFFSET] = 0xFFFFFFFF;
        _cache_targets[i][BETD_COMMAND] = _cache_targets[i][BETD_SIZE] = 0;
    }

    _cache_targets[AUDIO_DEVICE_OUT_EARPIECE][BETD_ID] =
    _cache_targets[AUDIO_DEVICE_OUT_SPEAKER][BETD_ID] =
    _cache_targets[AUDIO_DEVICE_OUT_WIRED_HEADSET][BETD_ID] =
    _cache_targets[AUDIO_DEVICE_OUT_WIRED_HEADPHONE][BETD_ID] =
    _cache_targets[AUDIO_DEVICE_OUT_ANC_HEADSET][BETD_ID] =
    _cache_targets[AUDIO_DEVICE_OUT_ANC_HEADPHONE][BETD_ID] =
                                SLIMBUS_0_RX;

    _cache_targets[AUDIO_DEVICE_OUT_BLUETOOTH_SCO][BETD_ID] =
    _cache_targets[AUDIO_DEVICE_OUT_BLUETOOTH_SCO_HEADSET][BETD_ID] =
    _cache_targets[AUDIO_DEVICE_OUT_BLUETOOTH_SCO_CARKIT][BETD_ID] =
                                INT_BT_SCO_RX;

    _cache_targets[AUDIO_DEVICE_OUT_BLUETOOTH_A2DP][BETD_ID] =
    _cache_targets[AUDIO_DEVICE_OUT_BLUETOOTH_A2DP_HEADPHONES][BETD_ID] =
    _cache_targets[AUDIO_DEVICE_OUT_BLUETOOTH_A2DP_SPEAKER][BETD_ID] =
    _cache_targets[AUDIO_DEVICE_OUT_ANLG_DOCK_HEADSET][BETD_ID] =
    _cache_targets[AUDIO_DEVICE_OUT_DGTL_DOCK_HEADSET][BETD_ID] =
    _cache_targets[AUDIO_DEVICE_OUT_USB_ACCESSORY][BETD_ID] =
    _cache_targets[AUDIO_DEVICE_OUT_USB_DEVICE][BETD_ID] =
    _cache_targets[AUDIO_DEVICE_OUT_PROXY][BETD_ID] =
                            RT_PROXY_PORT_001_RX;

    _cache_targets[AUDIO_DEVICE_OUT_AUX_DIGITAL][BETD_ID] = HDMI_RX;

    _cache_targets[AUDIO_DEVICE_OUT_REMOTE_SUBMIX][BETD_ID] = 0;

    _cache_targets[AUDIO_DEVICE_OUT_FM][BETD_ID] = INT_FM_RX;

    _cache_targets[AUDIO_DEVICE_OUT_FM_TX][BETD_ID] = INT_FM_TX;
}

static void reg_ion_mem(void)
{
    msm_audio_ion_alloc("DTS_EAGLE", &ion_client, &ion_handle, ION_MEM_SIZE,
                 &paddr, &pa_len, &vaddr);
}

void dts_ion_memmap(struct param_outband *po)
{
    po->size = ION_MEM_SIZE;
    po->kvaddr = (uint32_t)vaddr;
    po->paddr = (uint32_t)paddr;
}

static void unreg_ion_mem(void)
{
    msm_audio_ion_free(ion_client, ion_handle);
}

int dts_eagle_ioctl_pre(struct audio_client *ac, void *arg)
{
    struct dts_eagle_param_desc depd;
    pr_info("DTS_EAGLE_DRIVER_PRE: %s called (set [pre] param)\n",
        __func__);

    if(_depc_size == 0) {
        pr_err("DTS_EAGLE_DRIVER_VOLUME: driver cache not initialized.\n");
        return -EFAULT;
    }

    if (copy_from_user((void *)&depd, (void *)arg, sizeof(depd))) {
        pr_err("DTS_EAGLE_DRIVER_PRE: error copying dts_eagle_param_desc\n");
        return -EFAULT;
    }
    if ((depd.offset + depd.size) > _depc_size) {
        pr_err("DTS_EAGLE_DRIVER_PRE: invalid size %d and/or offset %d\n",
            depd.size, depd.offset);
        return -EINVAL;
    }
    if (copy_from_user((void *)&_depc[depd.offset],
               (void *)(((char *)arg)+sizeof(depd)), depd.size)) {
        pr_err("DTS_EAGLE_DRIVER_PRE: error copying param to cache\n");
        return -EFAULT;
    }
    if (dts_eagle_open_pre(ac, depd.id, depd.size,
                   (void *)&_depc[depd.offset])) {
        pr_err("DTS_EAGLE_DRIVER_PRE: dts_eagle_open_pre failed with id = 0x%X, size = %d, offset = %d\n",
            depd.id, depd.size, depd.offset);
    } else {
        pr_debug("DTS_EAGLE_DRIVER_PRE: dts_eagle_open_pre succeeded with id = 0x%X, size = %d, offset = %d\n",
             depd.id, depd.size, depd.offset);
    }
    return 0;
}

static const int log10_10_inv_x20 = 0x0008af84;
int msm_dts_eagle_set_volume(struct audio_client *ac, int lgain, int rgain) {
    int i, idx, val;

    if(_depc_size == 0) {
        pr_err("DTS_EAGLE_DRIVER_VOLUME: driver cache not initialized.\n");
        return -1;
    }

    for(i = 0; i < vol_cmd_cnt; i++) {
        if(vol_cmds_desc[i][0] & 0x8000) {
            idx = (sizeof(struct dts_eagle_param_desc)/sizeof(int)) +
                      (vol_cmds_desc[i][0]&0x3FF);
            val = fx_logN((lgain+rgain) << 2);
            val = ((long long)val * log10_10_inv_x20) >> 16;
            vol_cmds[i][idx] =
                clamp((int)(((long long)val * vol_cmds_desc[i][1]) >> 16),
                      vol_cmds_desc[i][2], vol_cmds_desc[i][3]);
            pr_debug("DTS_EAGLE_DRIVER_VOLUME: loop %i cmd desc found %i, idx = %i. volume info: lgain = %i, rgain = %i, volume = %i (scale %i, min %i, max %i)\n",
                      i, vol_cmds_desc[i][0], idx, lgain, rgain, vol_cmds[i][idx], vol_cmds_desc[i][1], vol_cmds_desc[i][2], vol_cmds_desc[i][3]);
        }
        memcpy((void *)&_depc[vol_cmds[i][2]], &vol_cmds[i][4],
                vol_cmds[i][1]);
        if (dts_eagle_open_pre(ac, vol_cmds[i][0], vol_cmds[i][1],
                   (void *)&_depc[vol_cmds[i][2]])) {
            pr_err("DTS_EAGLE_DRIVER_VOLUME: loop %i - volume set failed with id 0x%X, size %i, offset %i, cmd_desc %i, scale %i, min %i, max %i, data(...) %i\n",
                 i, vol_cmds[i][0], vol_cmds[i][1], vol_cmds[i][2], vol_cmds_desc[i][0], vol_cmds_desc[i][1], vol_cmds_desc[i][2], vol_cmds_desc[i][3], vol_cmds[i][4]);
        } else {
            pr_debug("DTS_EAGLE_DRIVER_VOLUME: loop %i - volume set succeeded with id 0x%X, size %i, offset %i, cmd_desc %i, scale %i, min %i, max %i, data(...) %i\n",
                 i, vol_cmds[i][0], vol_cmds[i][1], vol_cmds[i][2], vol_cmds_desc[i][0], vol_cmds_desc[i][1], vol_cmds_desc[i][2], vol_cmds_desc[i][3], vol_cmds[i][4]);
        }
    }
    return 0;
}

static int msm_pcm_routing_hwdep_open(struct snd_hwdep *hw, struct file *file)
{
    pr_info("%s called\n", __func__);
    return 0;
}

static int msm_pcm_routing_hwdep_release(struct snd_hwdep *hw,
                     struct file *file)
{
    pr_info("%s called\n", __func__);
    return 0;
}

static int msm_pcm_routing_hwdep_ioctl(struct snd_hwdep *hw, struct file *file,
                       unsigned int cmd, unsigned long arg)
{
    switch (cmd) {
    case DTS_EAGLE_IOCTL_GET_CACHE_SIZE: {
        pr_info("DTS_EAGLE_DRIVER_POST: %s called with control 0x%X (get param cache size)\n",
            __func__, cmd);
        if (copy_to_user((void *)arg, &_depc_size,
                 sizeof(_depc_size))) {
            pr_err("DTS_EAGLE_DRIVER_POST: error writing size\n");
            return -EFAULT;
        }
        break;
    }
    case DTS_EAGLE_IOCTL_SET_CACHE_SIZE: {
        int size = 0;
        pr_info("DTS_EAGLE_DRIVER_POST: %s called with control 0x%X (allocate param cache)\n",
            __func__, cmd);
        if (copy_from_user((void *)&size, (void *)arg, sizeof(size))) {
            pr_err("DTS_EAGLE_DRIVER_POST: error copying size\n");
            return -EFAULT;
        } else if (size < 0 || size > DEPC_MAX_SIZE) {
            pr_err("DTS_EAGLE_DRIVER_POST: cache size %d not allowed (min 0, max %d)\n",
                size, DEPC_MAX_SIZE);
            return -EFAULT;
        }
        if (_depc) {
            pr_info("DTS_EAGLE_DRIVER_POST: previous param cache of size %u freed\n",
                _depc_size);
            _depc_size = 0;
            kfree(_depc);
            _depc = NULL;
        }
        if (size) {
            _depc = kzalloc(size, GFP_KERNEL);
        } else {
            pr_info("DTS_EAGLE_DRIVER_POST: %d bytes requested for param cache, nothing allocated\n",
                size);
        }
        if (_depc) {
            pr_info("DTS_EAGLE_DRIVER_POST: %d bytes allocated for param cache\n",
                size);
            _depc_size = (unsigned int)size;
        } else {
            pr_err("DTS_EAGLE_DRIVER_POST: error allocating param cache (kzalloc failed on %d bytes)\n",
                size);
            _depc_size = 0;
        }
        break;
    }
    case DTS_EAGLE_IOCTL_GET_PARAM: {
        struct dts_eagle_param_desc depd;
        int offset = 0;
        void *buf, *buf_m = NULL;
        pr_info("DTS_EAGLE_DRIVER_POST: %s called, control 0x%X (get param)\n",
            __func__, cmd);
        if (copy_from_user((void *)&depd, (void *)arg, sizeof(depd))) {
            pr_err("DTS_EAGLE_DRIVER_POST: error copying dts_eagle_param_desc\n");
            return -EFAULT;
        }        
        if (depd.offset == -1) {
            pr_info("DTS_EAGLE_DRIVER_POST: get from qdsp requested\n");
            buf = buf_m = kzalloc(depd.size, GFP_KERNEL);            
            if(!buf_m) {
                pr_err("DTS_EAGLE_DRIVER_POST: out of memory\n");
                return -ENOMEM;
            } else if(dts_eagle_open_get(_cache_targets[depd.target][BETD_ID],
                      depd.id, buf, depd.size) < 0) {
                pr_err("DTS_EAGLE_DRIVER_POST: get from qdsp failed\n");
                return -EFAULT;        
            }
        } else {
            offset = _cache_targets[depd.target][BETD_OFFSET] + depd.offset;
            if ((offset + depd.size) > _depc_size) {
                pr_err("DTS_EAGLE_DRIVER_POST: invalid size %d and/or offset %d\n",
                    depd.size, offset);
                return -EFAULT;
            }
            buf = (void *)&_depc[offset];
        }
        if (copy_to_user((void *)(((char *)arg)+sizeof(depd)),
                          buf, depd.size)) {
            pr_err("DTS_EAGLE_DRIVER_POST: error getting param\n");
            return -EFAULT;
        }
        if(buf_m)
            kfree(buf_m);
        break;
    }
    case DTS_EAGLE_IOCTL_SET_PARAM: {
        struct dts_eagle_param_desc depd;
        int offset = 0, just_set_cache = 0, target_pre = 0;
        pr_info("DTS_EAGLE_DRIVER_POST: %s called, control 0x%X (set param)\n",
            __func__, cmd);
        if (copy_from_user((void *)&depd, (void *)arg, sizeof(depd))) {
            pr_err("DTS_EAGLE_DRIVER_POST: error copying dts_eagle_param_desc\n");
            return -EFAULT;
        }

        if(depd.target & (1<<31)) {
            pr_info("DTS_EAGLE_DRIVER_POST: 'just set cache' requested.\n");
            just_set_cache = 1;
        }
        if(depd.target & (1<<30)) {
            if(just_set_cache == 0) {
                pr_err("DTS_EAGLE_DRIVER_POST: invalid preproc param for cache request without 'just set cache' requested.\n");
                return -EFAULT;
            }
            pr_info("DTS_EAGLE_DRIVER_POST: preproc param for cache requested.\n");
            target_pre = 1;
        }
        depd.target &= 0x3FFFFFFF;

        offset = target_pre ? depd.offset :
                     depd.offset + _cache_targets[depd.target][BETD_OFFSET];
        if ((offset + depd.size) > _depc_size) {
            pr_err("DTS_EAGLE_DRIVER_POST: invalid size %d and/or offset %d\n",
                depd.size, offset);
            return -EFAULT;
        }
        if (copy_from_user((void *)&_depc[offset],
                   (void *)(((char *)arg)+sizeof(depd)),
                    depd.size)) {
            pr_err("DTS_EAGLE_DRIVER_POST: error copying param to cache\n");
            return -EFAULT;
        }
        pr_debug("DTS_EAGLE_DRIVER_POST: param info: param = 0x%X, size = %i, offset = %i, target = %i, global offset = %i, first 4 bytes as int = %i.\n",
                  depd.id, depd.size, depd.offset, depd.target, offset, *((int *)&_depc[offset]));

        if(!just_set_cache) {
            pr_debug("DTS_EAGLE_DRIVER_POST: checking for active backend: %i (target was %i)\n",
                _cache_targets[depd.target][BETD_ID], depd.target);
            if (get_is_backend_active(
                    _cache_targets[depd.target][BETD_ID])) {
                if (dts_eagle_open_post(_cache_targets[depd.target][BETD_ID],
                        depd.id, (void *)&_depc[offset], depd.size) < 0) {
                    pr_err("DTS_EAGLE_DRIVER_POST: dts_eagle_open_post failed with id = 0x%X, size = %d, offset = %d, target = %d, global offset = %d\n",
                        depd.id, depd.size, depd.offset, depd.target, offset);
                } else {
                    pr_debug("DTS_EAGLE_DRIVER_POST: dts_eagle_open_post succeeded with id = 0x%X, size = %d, offset = %d, target = %d, global offset = %d\n",
                        depd.id, depd.size, depd.offset, depd.target, offset);
                }
            } else {
                pr_debug("DTS_EAGLE_DRIVER_POST: backend not active");
            }
        }
        break;
    }
    case DTS_EAGLE_IOCTL_SET_CACHE_TARGET: {
        unsigned int target[3] = {0, 0, 0}, device_idx = 0;
        pr_info("DTS_EAGLE_DRIVER_POST: %s called with control 0x%X (set param cache target)\n",
            __func__, cmd);
        if (copy_from_user((void *)target, (void *)arg,
                   sizeof(target))) {
            pr_err("DTS_EAGLE_DRIVER_POST: error copying cache target data\n");
            return -EFAULT;
        }
        device_idx = (target[0] & 0xFF000000) >> 24;
        if (device_idx & 0x40) {
            _pre_cmd = target[1];
            _pre_size = target[2];
            pr_info("DTS_EAGLE_DRIVER_POST: setting preproc target.\n");
            break;
        }
        if (device_idx & 0x80) {
            device_idx &= 0x3F;
            _setup_cache_target_mapping();
            pr_info("DTS_EAGLE_DRIVER_POST: cache target reset requested.\n");
        }
        if (device_idx >= AUDIO_DEVICE_OUT_COUNT) {
            pr_err("DTS_EAGLE_DRIVER_POST: target backend index out of range (index = %u, max is %u)\n",
                device_idx, AUDIO_DEVICE_OUT_COUNT);
            return -EFAULT;
        }
        target[0] &= 0x00FFFFFF;
        if (target[0] + target[2] >= _depc_size || !target[0] ||
            !target[1] || !target[2]) {
            pr_err("DTS_EAGLE_DRIVER_POST: target offset and/or size out of range or zero, or invalid parameter sent (offset %u, size %u, cache size limit is %u, parameter sent 0x%X)\n",
                target[0], target[2], _depc_size, target[1]);
            return -EFAULT;
        }
        _cache_targets[device_idx][BETD_OFFSET] = target[0];
        _cache_targets[device_idx][BETD_COMMAND] = target[1];
        _cache_targets[device_idx][BETD_SIZE] = target[2];
        break;
    }
    case DTS_EAGLE_IOCTL_GET_LICENSE: {
        int target = 0, size = 0, size_only;
        pr_info("DTS_EAGLE_DRIVER_POST: %s called with control 0x%X (get license)\n",
            __func__, cmd);
        if (copy_from_user((void *)&target, (void *)arg,
                   sizeof(target))) {
            pr_err("DTS_EAGLE_DRIVER_POST: error reading license index.\n");
            return -EFAULT;
        }
        size_only = target & (1<<31) ? 1 : 0;
        target &= 0x7FFFFFFF;
        if(target < 0 || target >= SEC_BLOB_MAX_CNT) {
            pr_err("DTS_EAGLE_DRIVER_POST: license index %i out of bounds (max index is %i)\n",
                   target, SEC_BLOB_MAX_CNT);
            return -EFAULT;
        }
        if(sec_blob[target] == NULL) {
            pr_err("DTS_EAGLE_DRIVER_POST: license index %i never initialized.\n",
                   target);
            return -EFAULT;
        }
        size = ((int*)sec_blob[target])[0];
        if(size <= 0 || size > SEC_BLOB_MAX_SIZE) {
            pr_err("DTS_EAGLE_DRIVER_POST: license size %i for index %i invalid (min size is 1, max size is %i).\n",
                   size, target, SEC_BLOB_MAX_SIZE);
            return -EFAULT;
        }
        if(size_only) {
            pr_info("DTS_EAGLE_DRIVER_POST: reporting size of license data only");
            if (copy_to_user((void *)(((char *)arg)+sizeof(target)),
                 (void *)&size, sizeof(size))) {
                pr_err("DTS_EAGLE_DRIVER_POST: error copying license size.\n");
                return -EFAULT;
            }
        } else if (copy_to_user((void *)(((char *)arg)+sizeof(target)),
                 (void *)&(((int*)sec_blob[target])[1]), size)) {
            pr_err("DTS_EAGLE_DRIVER_POST: error copying license data.\n");
            return -EFAULT;
        } else {
            pr_debug("DTS_EAGLE_DRIVER_POST: license file %i bytes long from license index %i returned to user.\n",
                  size, target);
        }
        break;
    }
    case DTS_EAGLE_IOCTL_SET_LICENSE: {
        int target[2] = {0, 0};
        pr_info("DTS_EAGLE_DRIVER_POST: %s called with control 0x%X (set license)\n",
            __func__, cmd);
        if (copy_from_user((void *)target, (void *)arg,
                   sizeof(target))) {
            pr_err("DTS_EAGLE_DRIVER_POST: error reading license index.\n");
            return -EFAULT;
        }
        if(target[0] < 0 || target[0] >= SEC_BLOB_MAX_CNT) {
            pr_err("DTS_EAGLE_DRIVER_POST: license index %i out of bounds (max index is %i)\n",
                   target[0], SEC_BLOB_MAX_CNT-1);
            return -EFAULT;
        }
        if(target[1] == 0) {
            pr_info("DTS_EAGLE_DRIVER_POST: request to free license index %i", target[0]);
            kfree(sec_blob[target[0]]);
            sec_blob[target[0]] = NULL;
            break;
        }
        if(target[1] <= 0 || target[1] >= SEC_BLOB_MAX_SIZE) {
            pr_err("DTS_EAGLE_DRIVER_POST: license size %i for index %i invalid (min size is 1, max size is %i).\n",
                   target[1], target[0], SEC_BLOB_MAX_SIZE);
            return -EFAULT;
        }
        if(sec_blob[target[0]] != NULL) {
            if(((int*)sec_blob[target[0]])[1] != target[1]) {
                pr_info("DTS_EAGLE_DRIVER_POST: request new size for already allocated license index %i",
                        target[0]);
                kfree(sec_blob[target[0]]);
                sec_blob[target[0]] = NULL;
            }
        }
        pr_debug("DTS_EAGLE_DRIVER_POST: allocating %i bytes for license index %i\n",
                  target[1], target[0]);
        sec_blob[target[0]] = kzalloc(target[1] + 4, GFP_KERNEL);
        if(!sec_blob[target[0]]) {
            pr_err("DTS_EAGLE_DRIVER_POST: error allocating license index %i (kzalloc failed on %i bytes)\n",
                    target[0], target[1]);
            return -EFAULT;
        }
        ((int*)sec_blob[target[0]])[0] = target[1];
        if (copy_from_user((void *)&(((int*)sec_blob[target[0]])[1]),
                (void *)(((char *)arg)+sizeof(target)),
                target[1])) {
            pr_err("DTS_EAGLE_DRIVER_POST: error copying license to index %i, size %i\n",
                    target[0], target[1]);
            return -EFAULT;
        } else {
            pr_debug("DTS_EAGLE_DRIVER_POST: license file %i bytes long copied to index license index %i\n",
                  target[1], target[0]);
        }
        break;
    }
    case DTS_EAGLE_IOCTL_SEND_LICENSE: {
        int target = 0;
        pr_info("DTS_EAGLE_DRIVER_POST: %s called with control 0x%X (send license)\n",
            __func__, cmd);
        if (copy_from_user((void *)&target, (void *)arg,
                   sizeof(int))) {
            pr_err("DTS_EAGLE_DRIVER_POST: error reading license index.\n");
            return -EFAULT;
        }
        if (target >= SEC_BLOB_MAX_CNT) {
            pr_err("DTS_EAGLE_DRIVER_POST: license index %i out of bounds (max index is %i)\n",
                    target, SEC_BLOB_MAX_CNT-1);
            return -EINVAL;
        }
        if (!sec_blob[target] || ((int*)sec_blob[target])[0] <= 0) {
            pr_err("DTS_EAGLE_DRIVER_POST: license index %i is invalid\n", target);
            return -EINVAL;
        }
        if (core_set_dts_eagle(((int*)sec_blob[target])[0],
                               (char*)&((int*)sec_blob[target])[1]) < 0) {
            pr_err("DTS_EAGLE_DRIVER_POST: core_set_dts_eagle failed with id = %i\n",
                    target);
        } else {
            pr_debug("DTS_EAGLE_DRIVER_POST: core_set_dts_eagle succeeded with id = %i\n",
                      target);
        }
        break;
    }
    case DTS_EAGLE_IOCTL_SET_VOLUME_COMMANDS: {
        int spec = 0;
        pr_info("DTS_EAGLE_DRIVER_POST: %s called with control 0x%X (set volume commands)\n",
            __func__, cmd);
        if (copy_from_user((void *)&spec, (void *)arg,
                    sizeof(int))) {
            pr_err("DTS_EAGLE_DRIVER_POST: error reading volume command specifier.\n");
            return -EFAULT;
        }
        if (spec & 0x80000000) {
            int idx = (spec & 0x0000F000) >> 12,
                size = spec & 0x00000FFF;
            pr_debug("DTS_EAGLE_DRIVER_POST: setting volume command %i size: %i\n", idx, size);
            if (idx >= vol_cmd_cnt) {
                pr_err("DTS_EAGLE_DRIVER_POST: volume command index %i out of bounds (only %i allocated).\n", idx, vol_cmd_cnt);
                return -EINVAL;
            }
            if (volume_cmds_alloc2(idx, size) < 0) {
                pr_err("DTS_EAGLE_DRIVER_POST: error allocating memory for volume controls.\n");
                return -ENOMEM;
            }
            if (copy_from_user((void*)&vol_cmds_desc[idx],
                               (void*)(((char*)arg)+sizeof(int)),
                               sizeof(vol_cmds_desc_))) {
                pr_err("DTS_EAGLE_DRIVER_POST: error reading volume command descriptor.\n");
                return -EFAULT;
            }
            pr_debug("DTS_EAGLE_DRIVER_POST: setting volume command %i spec (size %i): %i %i %i %i\n",
                      idx, sizeof(vol_cmds_desc_), vol_cmds_desc[idx][0],
                      vol_cmds_desc[idx][1], vol_cmds_desc[idx][2], vol_cmds_desc[idx][3]);
            if (copy_from_user((void*)vol_cmds[idx],
                               (void*)(((char*)arg) + (sizeof(int) +
                               sizeof(vol_cmds_desc_))), size)) {
                pr_err("DTS_EAGLE_DRIVER_POST: error reading volume command string.\n");
                return -EFAULT;
            }
        } else {
            pr_debug("DTS_EAGLE_DRIVER_POST: setting volume command size\n");
            if (spec < 0 || spec > VOL_CMD_CNT_MAX) {
                pr_err("DTS_EAGLE_DRIVER_POST: volume command count %i out of bounds (min 0, max %i).\n", spec, VOL_CMD_CNT_MAX);
                return -EFAULT;
            } else if (spec == 0) {
                pr_info("DTS_EAGLE_DRIVER_POST: request to free volume commands.\n");
                volume_cmds_free();
                break;
            }
            pr_debug("DTS_EAGLE_DRIVER_POST: setting volume command size requested = %i\n", spec);
            if (volume_cmds_alloc1(spec) < 0) {
                pr_err("DTS_EAGLE_DRIVER_POST: error allocating memory for volume controls.\n");
                return -ENOMEM;
            }
        }
        break;
    }
    default: {
        pr_info("DTS_EAGLE_DRIVER_POST: %s called, control 0x%X (invalid control)\n",
            __func__, cmd);
    }
    }
    return 0;
}

static void msm_pcm_routing_hwdep_free(struct snd_hwdep *hwdep)
{
    pr_info("%s called\n", __func__);
}

int msm_dts_eagle_send_cache_pre(struct audio_client *ac)
{
    if (_depc_size == 0 || !_depc || _pre_size == 0 || _pre_cmd == 0 ||
        _pre_size > _depc_size) {
        pr_err("DTS_EAGLE_DRIVER_SENDCACHE_PRE: in %s, general error - cache size = %u, cache ptr = %p, size = %u, cmd = %u\n",
            __func__, _depc_size, _depc, _pre_size, _pre_cmd);
        return -EINVAL;
    }

    pr_debug("DTS_EAGLE_DRIVER_SENDCACHE_PRE: first 6 integers %i %i %i %i %i %i (30th %i)\n",
          *((int *)&_depc[0]), *((int *)&_depc[4]),
          *((int *)&_depc[8]), *((int *)&_depc[12]),
          *((int *)&_depc[16]), *((int *)&_depc[20]), *((int *)&_depc[120]));
    pr_debug("DTS_EAGLE_DRIVER_SENDCACHE_PRE: sending full data block to stream, with param = 0x%X and size = %d\n",
          _pre_cmd, _pre_size);

    if (dts_eagle_open_pre(ac, _pre_cmd, _pre_size, (void *)_depc)) {
        pr_err("DTS_EAGLE_DRIVER_SENDCACHE_PRE: in %s, dts_eagle_open_pre failed with id = %d and size = %d\n",
            __func__, _pre_cmd, _pre_size);
    } else {
        pr_debug("DTS_EAGLE_DRIVER_SENDCACHE_PRE: in %s, dts_eagle_open_pre succeeded with id = %d and size = %d\n",
             __func__, _pre_cmd, _pre_size);
    }
    return 0;
}

int msm_dts_eagle_send_cache_post(int be_idx)
{
    int offset, id = -1, i, size, cmd;

    /* find active device */
    for (i = 0; i < AUDIO_DEVICE_OUT_COUNT; i++) {
        if (_cache_targets[i][BETD_OFFSET] != 0xFFFFFFFF &&
            _cache_targets[i][BETD_ID] == be_idx) {
            id = i;
            break;
        }
    }
    if (id == -1) {
        pr_err("DTS_EAGLE_DRIVER_SENDCACHE_POST: in %s, active device not found for back end %i\n",
            __func__, be_idx);
        return -EINVAL;
    }
    offset = _cache_targets[id][BETD_OFFSET];
    cmd = _cache_targets[id][BETD_COMMAND];
    size = _cache_targets[id][BETD_SIZE];

    if (_depc_size == 0 || !_depc || offset <= 0 || size <= 0 || cmd == 0 ||
        (offset + size) > _depc_size) {
        pr_err("DTS_EAGLE_DRIVER_SENDCACHE_POST: in %s, id %i be_id %i general error - cache size = %u, cache ptr = %p, offset = %i, size = %i, cmd = %i\n",
            __func__, id, be_idx, _depc_size, _depc, offset, size, cmd);
        return -EINVAL;
    }

    pr_debug("DTS_EAGLE_DRIVER_SENDCACHE_POST: first 6 integers %i %i %i %i %i %i\n",
          *((int *)&_depc[offset]), *((int *)&_depc[offset+4]),
          *((int *)&_depc[offset+8]), *((int *)&_depc[offset+12]),
          *((int *)&_depc[offset+16]), *((int *)&_depc[offset+20]));
    pr_debug("DTS_EAGLE_DRIVER_SENDCACHE_POST: sending full data block to backend, with device id = %d, be_id = %d, param = 0x%X, offset = %d, and size = %d\n",
          id, be_idx, cmd, offset, size);

    if (dts_eagle_open_post(be_idx, cmd,
        (void *)&_depc[offset], size) < 0) {
        pr_err("DTS_EAGLE_DRIVER_SENDCACHE_POST: in %s, dts_eagle_open_post failed with id = 0x%X and size = %d\n",
            __func__, cmd, size);
    } else {
        pr_debug("DTS_EAGLE_DRIVER_SENDCACHE_POST: in %s, dts_eagle_open_post succeeded with id = 0x%X and size = %d\n",
             __func__, cmd, size);
    }

    return 0;
}

int msm_dts_eagle_pcm_new(struct snd_soc_pcm_runtime *runtime,
              struct msm_pcm_routing_bdai_data *msm_bedais)
{
    struct snd_hwdep *hwdep;
    struct snd_soc_dai_link *dai_link = runtime->dai_link;
    int rc;
    dev_info(runtime->platform->dev, "DTS_eagle: %s entered be ID %d\n",
         __func__, dai_link->be_id);
    rc = snd_hwdep_new(runtime->card->snd_card,
               msm_bedais[dai_link->be_id].name,
               dai_link->be_id, &hwdep);

    if (IS_ERR_VALUE(rc)) {
        dev_err(runtime->platform->dev,
            "%s: DTS_EAGLE hwdep intf failed to create %s\n",
            __func__, msm_bedais[dai_link->be_id].name);
        return rc;
    }

    hwdep->iface = SNDRV_HWDEP_IFACE_QC_AUDIO;
    hwdep->private_data = &msm_bedais[dai_link->be_id];
    hwdep->private_free = msm_pcm_routing_hwdep_free;
    hwdep->ops.open = msm_pcm_routing_hwdep_open;
    hwdep->ops.ioctl = msm_pcm_routing_hwdep_ioctl;
    hwdep->ops.release = msm_pcm_routing_hwdep_release;

    if (!ref_cnt) {
        _setup_cache_target_mapping();
        reg_ion_mem();
    }
    ref_cnt++;

    return 0;
}

void msm_dts_eagle_pcm_free(struct snd_pcm *pcm)
{
    /* TODO: Remove hwdep interface */
    ref_cnt--;
    if (!ref_cnt)
        unreg_ion_mem();
}

MODULE_DESCRIPTION("DTS EAGLE platform driver");
MODULE_LICENSE("GPL v2");
