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

#ifndef __MSM_DTS_EAGLE_H__
#define __MSM_DTS_EAGLE_H__

struct snd_soc_pcm_runtime;
struct snd_pcm;
struct audio_client;
struct dts_eagle_param_desc;
struct snd_soc_platform;

struct param_outband {
	size_t       size;
	void        *kvaddr;
	phys_addr_t  paddr;
};

#ifdef CONFIG_DTS_EAGLE
void msm_dts_ion_memmap(struct param_outband *po_);
void msm_dts_eagle_set_audioclient(struct audio_client *ac);
void msm_dts_eagle_clear_audioclient(void);
int msm_dts_eagle_enable_asm(struct audio_client *ac, __u32 enable, int module);
int msm_dts_eagle_enable_adm(int port_id, int copp_idx, __u32 enable);
int msm_dts_eagle_enable_master(struct audio_client *ac, __u32 enable);
void msm_dts_eagle_add_controls(struct snd_soc_platform *platform);
int msm_dts_eagle_set_stream_gain(struct audio_client *ac,
				  int lgain, int rgain);
int msm_dts_eagle_handle_asm(struct dts_eagle_param_desc *depd, char *buf,
			     bool for_pre, bool get, struct audio_client *ac,
			     struct param_outband *po);
int msm_dts_eagle_handle_adm(struct dts_eagle_param_desc *depd, char *buf,
			     bool for_pre, bool get);
int msm_dts_eagle_ioctl(unsigned int cmd, unsigned long arg);
int msm_dts_eagle_sendcache_pre(struct audio_client *ac);
int msm_dts_eagle_sendcache_post(int port_id, int copp_idx, int topology);
int msm_dts_eagle_init_pre(struct audio_client *ac);
int msm_dts_eagle_deinit_pre(struct audio_client *ac);
int msm_dts_eagle_init_post(int port_id, int copp_id, int topology);
int msm_dts_eagle_deinit_post(int port_id, int topology);
int msm_dts_eagle_init_master_module(struct audio_client *ac);
int msm_dts_eagle_deinit_master_module(struct audio_client *ac);
int msm_dts_eagle_pcm_new(struct snd_soc_pcm_runtime *runtime);
void msm_dts_eagle_pcm_free(struct snd_pcm *pcm);
#else
static inline void msm_dts_ion_memmap(struct param_outband *po_)
{
	pr_debug("%s\n", __func__);
}
static inline void msm_dts_eagle_set_audioclient(struct audio_client *ac) {}
static inline void msm_dts_eagle_clear_audioclient(void) {}
static inline int msm_dts_eagle_enable_asm(struct audio_client *ac,
					   __u32 enable, int module)
{
	return 0;
}
static inline int msm_dts_eagle_enable_adm(int port_id, int copp_idx,
					   __u32 enable)
{
	return 0;
}
static inline int msm_dts_eagle_enable_master(struct audio_client *ac,
					      __u32 enable)
{
	return 0;
}
static inline void msm_dts_eagle_add_controls(struct snd_soc_platform *platform)
{
}
static inline int msm_dts_eagle_set_stream_gain(struct audio_client *ac,
						int lgain, int rgain)
{
	pr_debug("%s\n", __func__);
	return 0;
}
static inline int msm_dts_eagle_handle_asm(struct dts_eagle_param_desc *depd,
					   char *buf, bool for_pre, bool get,
					   struct audio_client *ac,
					   struct param_outband *po)
{
	return 0;
}
static inline int msm_dts_eagle_handle_adm(struct dts_eagle_param_desc *depd,
					   char *buf, bool for_pre, bool get)
{
	return 0;
}
static inline int msm_dts_eagle_ioctl(unsigned int cmd, unsigned long arg)
{
	return -EPERM;
}
static inline int msm_dts_eagle_sendcache_pre(struct audio_client *ac)
{
	return 0;
}
static inline int msm_dts_eagle_sendcache_post(int port_id, int copp_idx,
					       int topology)
{
	return 0;
}
static inline int msm_dts_eagle_init_pre(struct audio_client *ac)
{
	return 0;
}
static inline int msm_dts_eagle_deinit_pre(struct audio_client *ac)
{
	return 0;
}
static inline int msm_dts_eagle_init_post(int port_id, int coppid, int topology)
{
	return 0;
}
static inline int msm_dts_eagle_deinit_post(int port_id, int topology)
{
	return 0;
}
static inline int msm_dts_eagle_init_master_module(struct audio_client *ac)
{
	return 0;
}
static inline int msm_dts_eagle_deinit_master_module(struct audio_client *ac)
{
	return 0;
}
static inline int msm_dts_eagle_pcm_new(struct snd_soc_pcm_runtime *runtime)
{
	pr_debug("%s\n", __func__);
	return 0;
}
static inline void msm_dts_eagle_pcm_free(struct snd_pcm *pcm)
{
	pr_debug("%s\n", __func__);
}
#endif

#endif
