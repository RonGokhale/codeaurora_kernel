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

#ifndef __MSM_DTS_EAGLE_H__
#define __MSM_DTS_EAGLE_H__

struct snd_soc_pcm_runtime;
struct msm_pcm_routing_bdai_data;
struct snd_pcm;
struct audio_client;

struct param_outband {
	uint32_t        size;
	uint32_t        kvaddr;
	uint32_t        paddr;
};

#ifdef CONFIG_DTS_EAGLE
int msm_dts_eagle_send_cache_post(int port_id);

int msm_dts_eagle_send_cache_pre(struct audio_client *ac);

int msm_dts_eagle_pcm_new(struct snd_soc_pcm_runtime *runtime,
				struct msm_pcm_routing_bdai_data *msm_bedais);

void msm_dts_eagle_pcm_free(struct snd_pcm *pcm);

int msm_dts_eagle_set_volume(struct audio_client *ac, int lgain, int rgain);

int msm_dts_eagle_ioctl_pre(struct audio_client *ac, void *arg);

void msm_dts_ion_memmap(struct param_outband *po);
#else
static inline int msm_dts_eagle_send_cache_post(int port_id)
{
	return 0;
}

static inline int msm_dts_eagle_send_cache_pre(struct audio_client *ac)
{
	return 0;
}

static inline int msm_dts_eagle_pcm_new(struct snd_soc_pcm_runtime *runtime,
				struct msm_pcm_routing_bdai_data *msm_bedais)
{
	pr_debug("%s\n", __func__);
	return 0;
}

static inline void msm_dts_eagle_pcm_free(struct snd_pcm *pcm)
{
	pr_debug("%s\n", __func__);
}

static inline int msm_dts_eagle_set_volume(struct audio_client *ac,
					   int lgain, int rgain) {
	pr_debug("%s\n", __func__);
	return 0;
}

static inline int msm_dts_eagle_ioctl_pre(struct audio_client *ac, void *arg)
{
	pr_debug("%s\n", __func__);
	return -EFAULT;
}

static inline void msm_dts_ion_memmap(struct param_outband *po)
{
	pr_debug("%s\n", __func__);
}
#endif

#endif
