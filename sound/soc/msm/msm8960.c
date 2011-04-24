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
 */

#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/soc-dsp.h>
#include <sound/pcm.h>
#include "msm-pcm-routing.h"

/* 8960 machine driver */

static const char *mm_be[] = {
	LPASS_BE_PRI_I2S_RX,
	LPASS_BE_PRI_I2S_TX,
	LPASS_BE_HDMI,
};

static struct snd_soc_dsp_link fe_media = {
	.supported_be = mm_be,
	.num_be = ARRAY_SIZE(mm_be),
	.fe_playback_channels = 2,
	.fe_capture_channels = 1,
	.trigger = {
		SND_SOC_DSP_TRIGGER_POST,
		SND_SOC_DSP_TRIGGER_POST
	},
};

/* Digital audio interface glue - connects codec <---> CPU */
static struct snd_soc_dai_link msm8960_dai[] = {
	/* FrontEnd DAI Links */
	{
		.name = "MSM8960 Media1",
		.stream_name = "MultiMedia1",
		.cpu_dai_name	= "MultiMedia1",
		.platform_name  = "msm-pcm-dsp",
		.dynamic = 1,
		.dsp_link = &fe_media,
		.be_id = MSM_FRONTEND_DAI_MULTIMEDIA1
	},
	{
		.name = "MSM8960 Media2",
		.stream_name = "MultiMedia2",
		.cpu_dai_name	= "MultiMedia2",
		.platform_name  = "msm-pcm-dsp",
		.dynamic = 1,
		.dsp_link = &fe_media,
		.be_id = MSM_FRONTEND_DAI_MULTIMEDIA2,
	},
	{
		.name = "Circuit-Switch Voice",
		.stream_name = "CS-Voice",
		.cpu_dai_name   = "CS-VOICE",
		.platform_name  = "msm-pcm-voice",
		.dynamic = 1,
		.dsp_link = &fe_media,
		.be_id = 0,
	},
	/* Backend DAI Links */
	{
		.name = LPASS_BE_PRI_I2S_RX,
		.stream_name = "Primary I2S Playback",
		.cpu_dai_name = "msm-dai-q6.0",
		.platform_name = "msm-pcm-routing",
		.codec_name     = "tabla_codec",
		.codec_dai_name	= "tabla_rx1",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_PRI_I2S_RX
	},
	{
		.name = LPASS_BE_PRI_I2S_TX,
		.stream_name = "Primary I2S Capture",
		.cpu_dai_name = "msm-dai-q6.1",
		.platform_name = "msm-pcm-routing",
		.codec_name     = "tabla_codec",
		.codec_dai_name	= "tabla_tx1",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_PRI_I2S_TX
	},
};

struct snd_soc_card snd_soc_card_msm8960 = {
	.name		= "msm8960-snd-card",
	.dai_link	= msm8960_dai,
	.num_links	= ARRAY_SIZE(msm8960_dai),
};

static struct platform_device *msm8960_snd_device;

static int __init msm8960_audio_init(void)
{
	int ret;

	msm8960_snd_device = platform_device_alloc("soc-audio", 0);
	if (!msm8960_snd_device) {
		pr_err("Platform device allocation failed\n");
		return -ENOMEM;
	}

	platform_set_drvdata(msm8960_snd_device, &snd_soc_card_msm8960);
	ret = platform_device_add(msm8960_snd_device);
	if (ret) {
		platform_device_put(msm8960_snd_device);
		return ret;
	}

	return ret;

}
module_init(msm8960_audio_init);

static void __exit msm8960_audio_exit(void)
{
	platform_device_unregister(msm8960_snd_device);
}
module_exit(msm8960_audio_exit);

MODULE_DESCRIPTION("ALSA SoC MSM8960");
MODULE_LICENSE("GPL v2");
