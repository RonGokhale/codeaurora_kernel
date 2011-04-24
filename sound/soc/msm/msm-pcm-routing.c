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


#include <linux/init.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/bitops.h>
#include <sound/core.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/pcm.h>
#include <sound/initval.h>
#include <sound/control.h>
#include <sound/q6adm.h>

#include "msm-pcm-routing.h"

struct audio_mixer_data {
	u32 port_id; /* AFE port ID for Rx, FE DAI ID for TX */
	unsigned long dai_sessions; /* Rx: FE DAIs Tx: BE DAI */
	u32 mixer_type; /* playback or capture */
};

#define INVALID_SESSION -1

enum {
	AUDIO_MIXER_PRI_I2S_RX = 0,
	AUDIO_MIXER_HDMI_RX,
	AUDIO_MIXER_MM_UL1,
	AUDIO_MIXER_MAX,
};

/* Tx mixer session is stored based on BE DAI ID
 * Need to map to actual AFE port ID since AFE port
 * ID can get really large.
 * The table convert DAI back to AFE port ID
 */
static int bedai_port_map[MSM_BACKEND_DAI_MAX] = {
	PRIMARY_I2S_RX,
	PRIMARY_I2S_TX,
	HDMI_RX,
};

/* Track ASM playback & capture sessions of DAI */
static int fe_dai_map[MSM_FRONTEND_DAI_MAX][2] = {
	/* MULTIMEDIA1 */
	{INVALID_SESSION, INVALID_SESSION},
	/* MULTIMEDIA2 */
	{INVALID_SESSION, INVALID_SESSION},
};

static struct audio_mixer_data audio_mixers[AUDIO_MIXER_MAX] = {
	/* AUDIO_MIXER_PRI_I2S_RX */
	{PRIMARY_I2S_RX, 0, SNDRV_PCM_STREAM_PLAYBACK},
	/* AUDIO_MIXER_PRI_I2S_TX */
	{HDMI_RX, 0, SNDRV_PCM_STREAM_PLAYBACK},
	/* AUDIO_MIXER_MM_UL1 */
	{MSM_FRONTEND_DAI_MULTIMEDIA1, 0, SNDRV_PCM_STREAM_CAPTURE},
};

void msm_pcm_routing_reg_phy_stream(int fedai_id, int dspst_id, int stream_type)
{
	int i, be_id;

	if (stream_type == SNDRV_PCM_STREAM_PLAYBACK) {
		fe_dai_map[fedai_id][0] = dspst_id;
		for (i = 0; i < AUDIO_MIXER_MAX; i++) {
			if ((audio_mixers[i].mixer_type == stream_type) &&
			(test_bit(fedai_id, &audio_mixers[i].dai_sessions)))
				/* To do: multiple devices case */
				adm_route_session(audio_mixers[i].port_id,
				dspst_id, 1);
		}
	} else {
		fe_dai_map[fedai_id][1] = dspst_id;
		for (i = 0; i < AUDIO_MIXER_MAX; i++) {
			if ((audio_mixers[i].mixer_type == stream_type) &&
				(fedai_id == audio_mixers[i].port_id)) {
				/* To-do: Handle mixing of inputs */
				be_id = find_next_bit(
					&audio_mixers[i].dai_sessions,
					MSM_BACKEND_DAI_MAX, 0);
				if (be_id < MSM_BACKEND_DAI_MAX)
					adm_route_session(bedai_port_map[be_id],
					dspst_id, 1);
				else
					pr_err("%s: no routing\n", __func__);
			}
		}
	}
}

void msm_pcm_routing_dereg_phy_stream(int fedai_id, int stream_type)
{
	int i, be_id;

	if (stream_type == SNDRV_PCM_STREAM_PLAYBACK) {
		for (i = 0; i < AUDIO_MIXER_MAX; i++) {
			if ((audio_mixers[i].mixer_type == stream_type) &&
			(test_bit(fedai_id, &audio_mixers[i].dai_sessions))) {
				/* To do: multiple devices case */
				adm_route_session(audio_mixers[i].port_id,
				fe_dai_map[fedai_id][0], 0);
			}
		}
		fe_dai_map[fedai_id][0] = INVALID_SESSION;
	} else {
		for (i = 0; i < AUDIO_MIXER_MAX; i++) {
			if ((audio_mixers[i].mixer_type == stream_type) &&
				(fedai_id == audio_mixers[i].port_id)) {
				/* To-do: Handle mixing of inputs */
				be_id = find_next_bit(
					&audio_mixers[i].dai_sessions,
					MSM_BACKEND_DAI_MAX, 0);
				if (be_id < MSM_BACKEND_DAI_MAX)
					adm_route_session(bedai_port_map[be_id],
					fe_dai_map[fedai_id][1], 0);
				else
					pr_err("%s: no routing\n", __func__);
			}
		}
		fe_dai_map[fedai_id][1] = INVALID_SESSION;
	}
}

static void msm_pcm_routing_process_audio(u16 reg, u16 val, int set)
{

	pr_debug("%s: reg %x val %x set %x\n", __func__, reg, val, set);

	if (set)
		set_bit(val, &audio_mixers[reg].dai_sessions);
	 else
		clear_bit(val, &audio_mixers[reg].dai_sessions);

	if (audio_mixers[reg].mixer_type == SNDRV_PCM_STREAM_PLAYBACK) {
		if (fe_dai_map[val][0] != INVALID_SESSION)
			adm_route_session(audio_mixers[reg].port_id,
			fe_dai_map[val][0], set);
	} else {
		int fe_id = audio_mixers[reg].port_id;
		if (fe_dai_map[fe_id][1] != INVALID_SESSION)
			adm_route_session(bedai_port_map[val],
			fe_dai_map[fe_id][1], set);
	}
}

static int msm_routing_get_audio_mixer(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
	(struct soc_mixer_control *)kcontrol->private_value;

	if (test_bit(mc->shift, &audio_mixers[mc->reg].dai_sessions))
		ucontrol->value.integer.value[0] = 1;
	else
		ucontrol->value.integer.value[0] = 0;

	pr_debug("%s: reg %x shift %x val %ld\n", __func__, mc->reg, mc->shift,
	ucontrol->value.integer.value[0]);

	return 0;
}

static int msm_routing_put_audio_mixer(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_widget *widget = snd_kcontrol_chip(kcontrol);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;

	if (ucontrol->value.integer.value[0]) {
		msm_pcm_routing_process_audio(mc->reg, mc->shift, 1);
		snd_soc_dapm_mixer_update_power(widget, kcontrol, 1);
	} else {
		msm_pcm_routing_process_audio(mc->reg, mc->shift, 0);
		snd_soc_dapm_mixer_update_power(widget, kcontrol, 0);
	}

	return 1;
}

static const struct snd_kcontrol_new pri_rx_mixer_controls[] = {
	SOC_SINGLE_EXT("MultiMedia1", AUDIO_MIXER_PRI_I2S_RX ,
	MSM_FRONTEND_DAI_MULTIMEDIA1, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_SINGLE_EXT("MultiMedia2", AUDIO_MIXER_PRI_I2S_RX,
	MSM_FRONTEND_DAI_MULTIMEDIA2, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
};

static const struct snd_kcontrol_new hdmi_mixer_controls[] = {
	SOC_SINGLE_EXT("MultiMedia1", AUDIO_MIXER_HDMI_RX,
	MSM_FRONTEND_DAI_MULTIMEDIA1, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_SINGLE_EXT("MultiMedia2", AUDIO_MIXER_HDMI_RX,
	MSM_FRONTEND_DAI_MULTIMEDIA2, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
};

static const struct snd_kcontrol_new mmul1_mixer_controls[] = {
	SOC_SINGLE_EXT("PRI_TX", AUDIO_MIXER_MM_UL1,
	MSM_BACKEND_DAI_PRI_I2S_TX, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
};

static const struct snd_soc_dapm_widget msm_qdsp6_widgets[] = {
	/* Frontend AIF */
	/* Widget name equals to Front-End DAI name<Need confirmation>,
	 * Stream name must contains substring of front-end dai name
	 */
	SND_SOC_DAPM_AIF_IN("MM_DL1", "MultiMedia1 Playback", 0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("MM_DL2", "MultiMedia2 Playback", 0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("MM_UL1", "MultiMedia1 Capture", 0, 0, 0, 0),
	/* Backend AIF */
	/* Stream name equals to backend dai link stream name
	 */
	SND_SOC_DAPM_AIF_OUT("PRI_I2S_RX", "Primary I2S Playback", 0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("HDMI", "HDMI Playback", 0, 0, 0 , 0),
	SND_SOC_DAPM_AIF_IN("PRI_I2S_TX", "Primary I2S Capture", 0, 0, 0, 0),
	/* Mixer definitions */
	SND_SOC_DAPM_MIXER("PRI_RX Audio Mixer", SND_SOC_NOPM, 0, 0,
	pri_rx_mixer_controls, ARRAY_SIZE(pri_rx_mixer_controls)),
	SND_SOC_DAPM_MIXER("HDMI Mixer", SND_SOC_NOPM, 0, 0,
	hdmi_mixer_controls, ARRAY_SIZE(hdmi_mixer_controls)),
	SND_SOC_DAPM_MIXER("MultiMedia1 Mixer", SND_SOC_NOPM, 0, 0,
	mmul1_mixer_controls, ARRAY_SIZE(mmul1_mixer_controls)),
};

static const struct snd_soc_dapm_route intercon[] = {
	{"PRI_RX Audio Mixer", "MultiMedia1", "MM_DL1"},
	{"PRI_RX Audio Mixer", "MultiMedia2", "MM_DL2"},
	{"HDMI Mixer", "MultiMedia1", "MM_DL1"},
	{"HDMI Mixer", "MultiMedia2", "MM_DL2"},
	{"HDMI", NULL, "HDMI Mixer"},
	{"PRI_I2S_RX", NULL, "PRI_RX Audio Mixer"},
	{"MultiMedia1 Mixer", "PRI_TX", "PRI_I2S_TX"},
	{"MM_UL1", NULL, "MultiMedia1 Mixer"},
};

static struct snd_pcm_ops msm_routing_pcm_ops = {};

static unsigned int msm_routing_read(struct snd_soc_platform *platform,
				 unsigned int reg)
{
	dev_dbg(platform->dev, "reg %x\n", reg);
	return 0;
}

/* Not used but frame seems to require it */
static int msm_routing_write(struct snd_soc_platform *platform,
	unsigned int reg, unsigned int val)
{
	dev_dbg(platform->dev, "reg %x val %x\n", reg, val);
	return 0;
}

/* Not used but frame seems to require it */
static int msm_routing_probe(struct snd_soc_platform *platform)
{
	snd_soc_dapm_new_controls(&platform->dapm, msm_qdsp6_widgets,
			    ARRAY_SIZE(msm_qdsp6_widgets));
	snd_soc_dapm_add_routes(&platform->dapm, intercon,
		ARRAY_SIZE(intercon));

	snd_soc_dapm_new_widgets(&platform->dapm);

	return 0;
}

static struct snd_soc_platform_driver msm_soc_routing_platform = {
	.ops		= &msm_routing_pcm_ops,
	.probe		= msm_routing_probe,
	.read		= msm_routing_read,
	.write		= msm_routing_write,
};

static __devinit int msm_routing_pcm_probe(struct platform_device *pdev)
{
	dev_dbg(&pdev->dev, "dev name %s\n", dev_name(&pdev->dev));
	return snd_soc_register_platform(&pdev->dev,
				   &msm_soc_routing_platform);
}

static int msm_routing_pcm_remove(struct platform_device *pdev)
{
	snd_soc_unregister_platform(&pdev->dev);
	return 0;
}

static struct platform_driver msm_routing_pcm_driver = {
	.driver = {
		.name = "msm-pcm-routing",
		.owner = THIS_MODULE,
	},
	.probe = msm_routing_pcm_probe,
	.remove = __devexit_p(msm_routing_pcm_remove),
};

static int __init msm_soc_routing_platform_init(void)
{
	return platform_driver_register(&msm_routing_pcm_driver);
}
module_init(msm_soc_routing_platform_init);

static void __exit msm_soc_routing_platform_exit(void)
{
	platform_driver_unregister(&msm_routing_pcm_driver);
}
module_exit(msm_soc_routing_platform_exit);

MODULE_DESCRIPTION("MSM routing platform driver");
MODULE_LICENSE("GPL v2");
