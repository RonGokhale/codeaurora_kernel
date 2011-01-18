/*
 * harmony.c - Harmony machine ASoC driver
 *
 * Author: Stephen Warren <swarren@nvidia.com>
 * Copyright (C) 2010 - NVIDIA, Inc.
 *
 * Based on code copyright/by:
 *
 * (c) 2009, 2010 Nvidia Graphics Pvt. Ltd.
 *
 * Copyright 2007 Wolfson Microelectronics PLC.
 * Author: Graeme Gregory
 *         graeme.gregory@wolfsonmicro.com or linux@wolfsonmicro.com
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include <asm/mach-types.h>
#include <linux/module.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include "tegra_das.h"
#include "tegra_i2s.h"
#include "tegra_pcm.h"
#include "tegra_asoc_utils.h"

#include "../codecs/wm8903.h"

#define PREFIX "ASoC Harmony: "

#define TEGRA_HP        0
#define TEGRA_MIC       1
#define TEGRA_LINE      2
#define TEGRA_HEADSET   3
#define TEGRA_HP_OFF    4
#define TEGRA_SPK_ON    0
#define TEGRA_SPK_OFF   1

/* codec register values */
#define B07_INEMUTE                     7
#define B06_VOL_M3DB            6
#define B00_IN_VOL                      0
#define B00_INR_ENA                     0
#define B01_INL_ENA                     1
#define R06_MICBIAS_CTRL_0      6
#define B07_MICDET_HYST_ENA     7
#define B04_MICDET_THR          4
#define B02_MICSHORT_THR        2
#define B01_MICDET_ENA          1
#define B00_MICBIAS_ENA         0
#define B15_DRC_ENA                     15
#define B03_DACL_ENA            3
#define B02_DACR_ENA            2
#define B01_ADCL_ENA            1
#define B00_ADCR_ENA            0
#define B06_IN_CM_ENA           6
#define B04_IP_SEL_N            4
#define B02_IP_SEL_P            2
#define B00_MODE                        0
#define B06_AIF_ADCL            7
#define B06_AIF_ADCR            6
#define B05_ADC_HPF_CUT         5
#define B04_ADC_HPF_ENA         4
#define B01_ADCL_DATINV         1
#define B00_ADCR_DATINV         0
#define R20_SIDETONE_CTRL       32
#define R29_DRC_1                       41
#define SET_REG_VAL(r, m, l, v)         (((r)&(~((m)<<(l))))|(((v)&(m))<<(l)))

static struct platform_device *harmony_snd_device;
static int tegra_jack_func;
static int tegra_spk_func;

static void tegra_ext_control(struct snd_soc_codec *codec)
{
	/* set up jack connection */
	switch (tegra_jack_func) {
	case TEGRA_HP:
		/* set = unmute headphone */
		snd_soc_dapm_enable_pin(codec, "Mic Jack");
		snd_soc_dapm_disable_pin(codec, "Line Jack");
		snd_soc_dapm_enable_pin(codec, "Headphone Jack");
		snd_soc_dapm_disable_pin(codec, "Headset Jack");
		break;
	case TEGRA_MIC:
		/* reset = mute headphone */
		snd_soc_dapm_enable_pin(codec, "Mic Jack");
		snd_soc_dapm_disable_pin(codec, "Line Jack");
		snd_soc_dapm_disable_pin(codec, "Headphone Jack");
		snd_soc_dapm_disable_pin(codec, "Headset Jack");
		break;
	case TEGRA_LINE:
		snd_soc_dapm_disable_pin(codec, "Mic Jack");
		snd_soc_dapm_enable_pin(codec, "Line Jack");
		snd_soc_dapm_disable_pin(codec, "Headphone Jack");
		snd_soc_dapm_disable_pin(codec, "Headset Jack");
		break;
	case TEGRA_HEADSET:
		snd_soc_dapm_enable_pin(codec, "Mic Jack");
		snd_soc_dapm_disable_pin(codec, "Line Jack");
		snd_soc_dapm_disable_pin(codec, "Headphone Jack");
		snd_soc_dapm_enable_pin(codec, "Headset Jack");
		break;
	}

	if (tegra_spk_func == TEGRA_SPK_ON)
		snd_soc_dapm_enable_pin(codec, "Ext Spk");
	else
		snd_soc_dapm_disable_pin(codec, "Ext Spk");

	/* signal a DAPM event */
	snd_soc_dapm_sync(codec);
}

static int harmony_asoc_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_codec *codec = codec_dai->codec;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int srate, mclk, mclk_change;
	int err;
	unsigned int ctrl_reg;

	srate = params_rate(params);
	switch (srate) {
	case 64000:
	case 88200:
	case 96000:
		mclk = 128 * srate;
		break;
	default:
		mclk = 256 * srate;
		break;
	}
	/* FIXME: Codec only requires >= 3MHz if OSR==0 */
	while (mclk < 6000000)
		mclk *= 2;

	err = tegra_asoc_utils_set_rate(srate, mclk, &mclk_change);
	if (err < 0) {
		pr_err(PREFIX "Can't configure clocks\n");
		return err;
	}

	err = snd_soc_dai_set_fmt(codec_dai,
					SND_SOC_DAIFMT_I2S |
					SND_SOC_DAIFMT_NB_NF |
					SND_SOC_DAIFMT_CBS_CFS);
	if (err < 0) {
		pr_err(PREFIX "codec_dai fmt not set\n");
		return err;
	}

	err = snd_soc_dai_set_fmt(cpu_dai,
					SND_SOC_DAIFMT_I2S |
					SND_SOC_DAIFMT_NB_NF |
					SND_SOC_DAIFMT_CBS_CFS);
	if (err < 0) {
		pr_err(PREFIX "cpu_dai fmt not set\n");
		return err;
	}

	if (mclk_change) {
	    err = snd_soc_dai_set_sysclk(codec_dai, 0, mclk, SND_SOC_CLOCK_IN);
	    if (err < 0) {
		    pr_err(PREFIX "codec_dai clock not set\n");
		    return err;
	    }
	}

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		return 0;

	snd_soc_write(codec, WM8903_ANALOGUE_LEFT_INPUT_0, 0x7);
	snd_soc_write(codec, WM8903_ANALOGUE_RIGHT_INPUT_0, 0x7);

	/* Mic Bias enable */
	ctrl_reg = (0x1 << B00_MICBIAS_ENA) | (0x1 << B01_MICDET_ENA);
	snd_soc_write(codec, WM8903_MIC_BIAS_CONTROL_0, ctrl_reg);

	/* Enable DRC */
	ctrl_reg = snd_soc_read(codec, WM8903_DRC_0);
	ctrl_reg |= (1 << B15_DRC_ENA);
	snd_soc_write(codec, WM8903_DRC_0, ctrl_reg);

	/* Single Ended Mic */
	ctrl_reg = (0x0 << B06_IN_CM_ENA) | (0x0 << B00_MODE) |
			(0x0 << B04_IP_SEL_N) | (0x1 << B02_IP_SEL_P);
	/* Mic Setting */
	snd_soc_write(codec, WM8903_ANALOGUE_LEFT_INPUT_1, ctrl_reg);
	snd_soc_write(codec, WM8903_ANALOGUE_RIGHT_INPUT_1, ctrl_reg);

	/* voulme for single ended mic */
	ctrl_reg = (0x5 << B00_IN_VOL);
	snd_soc_write(codec, WM8903_ANALOGUE_LEFT_INPUT_0, ctrl_reg);
	snd_soc_write(codec, WM8903_ANALOGUE_RIGHT_INPUT_0, ctrl_reg);

	/* replicate mic setting on both channels */
	ctrl_reg = snd_soc_read(codec, WM8903_AUDIO_INTERFACE_0);
	ctrl_reg = SET_REG_VAL(ctrl_reg, 0x1, B06_AIF_ADCR, 0x1);
	ctrl_reg = SET_REG_VAL(ctrl_reg, 0x1, B06_AIF_ADCL, 0x1);
	snd_soc_write(codec, WM8903_AUDIO_INTERFACE_0, ctrl_reg);

	/* Enable analog inputs */
	ctrl_reg = (0x1 << B01_INL_ENA) | (0x1 << B00_INR_ENA);
	snd_soc_write(codec, WM8903_POWER_MANAGEMENT_0, ctrl_reg);

	/* ADC Settings */
	ctrl_reg = snd_soc_read(codec, WM8903_ADC_DIGITAL_0);
	ctrl_reg |= (0x1<<B04_ADC_HPF_ENA);
	snd_soc_write(codec, WM8903_ADC_DIGITAL_0, ctrl_reg);

	ctrl_reg = 0;
	snd_soc_write(codec, R20_SIDETONE_CTRL, ctrl_reg);

	/* Enable ADC */
	ctrl_reg = snd_soc_read(codec, WM8903_POWER_MANAGEMENT_6);
	ctrl_reg |= (0x1<<B00_ADCR_ENA)|(0x1<<B01_ADCL_ENA);
	snd_soc_write(codec, WM8903_POWER_MANAGEMENT_6, ctrl_reg);

	/* Enable Sidetone */
	ctrl_reg = (0x1 << 2) | (0x2 << 0);
	/* sidetone 0 db */
	ctrl_reg |= (12 << 8) | (12 << 4);
	snd_soc_write(codec, R20_SIDETONE_CTRL, ctrl_reg);

	ctrl_reg = snd_soc_read(codec, R29_DRC_1);
	ctrl_reg |= 0x3; /* mic volume 18 db */
	snd_soc_write(codec, R29_DRC_1, ctrl_reg);

	return 0;
}

static int tegra_get_jack(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = tegra_jack_func;
	return 0;
}

static int tegra_set_jack(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

	if (tegra_jack_func == ucontrol->value.integer.value[0])
		return 0;

	tegra_jack_func = ucontrol->value.integer.value[0];
	tegra_ext_control(codec);
	return 1;
}

static int tegra_get_spk(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = tegra_spk_func;
	return 0;
}

static int tegra_set_spk(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec =  snd_kcontrol_chip(kcontrol);


	if (tegra_spk_func == ucontrol->value.integer.value[0])
		return 0;

	tegra_spk_func = ucontrol->value.integer.value[0];
	tegra_ext_control(codec);
	return 1;
}

/*tegra machine dapm widgets */
static const struct snd_soc_dapm_widget wm8903_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
	SND_SOC_DAPM_MIC("Mic Jack", NULL),
	SND_SOC_DAPM_SPK("Ext Spk", NULL),
	SND_SOC_DAPM_LINE("Line Jack", NULL),
	SND_SOC_DAPM_HP("Headset Jack", NULL),
};

/* Tegra machine audio map (connections to the codec pins) */
static const struct snd_soc_dapm_route audio_map[] = {
	/* headset Jack  - in = micin, out = LHPOUT*/
	{"Headset Jack", NULL, "HPOUTL"},

	/* headphone connected to LHPOUT1, RHPOUT1 */
	{"Headphone Jack", NULL, "HPOUTR"}, {"Headphone Jack", NULL, "HPOUTL"},

	/* speaker connected to LOUT, ROUT */
	{"Ext Spk", NULL, "LINEOUTR"}, {"Ext Spk", NULL, "LINEOUTL"},

	/* mic is connected to MICIN (via right channel of headphone jack) */
	{"IN1R", NULL, "Mic Jack"},

	/* Same as the above but no mic bias for line signals */
	{"IN2L", NULL, "Line Jack"},
};

static const char *jack_function[] = {"Headphone", "Mic", "Line", "Headset",
					"Off"
};
static const char *spk_function[] = {"On", "Off"};
static const struct soc_enum tegra_enum[] = {
	SOC_ENUM_SINGLE_EXT(5, jack_function),
	SOC_ENUM_SINGLE_EXT(2, spk_function),
};

static const struct snd_kcontrol_new wm8903_tegra_controls[] = {
	SOC_ENUM_EXT("Jack Function", tegra_enum[0], tegra_get_jack,
			tegra_set_jack),
	SOC_ENUM_EXT("Speaker Function", tegra_enum[1], tegra_get_spk,
			tegra_set_spk),
};

static int harmony_asoc_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	int err;

	/* Add tegra specific controls */
	err = snd_soc_add_controls(codec, wm8903_tegra_controls,
					ARRAY_SIZE(wm8903_tegra_controls));
	if (err < 0)
		return err;

	/* Add tegra specific widgets */
	snd_soc_dapm_new_controls(codec, wm8903_dapm_widgets,
					ARRAY_SIZE(wm8903_dapm_widgets));

	/* Set up tegra specific audio path audio_map */
	snd_soc_dapm_add_routes(codec, audio_map, ARRAY_SIZE(audio_map));

	/* Default to HP output */
	tegra_jack_func = TEGRA_HP;
	tegra_spk_func = TEGRA_SPK_ON;
	tegra_ext_control(codec);

	snd_soc_dapm_sync(codec);

	return 0;
}

static struct snd_soc_ops harmony_asoc_ops = {
	.hw_params = harmony_asoc_hw_params,
};

static struct snd_soc_dai_link harmony_wm8903_dai = {
	.name = "WM8903",
	.stream_name = "WM8903 PCM",
	.codec_name = "wm8903-codec.0-001a",
	.platform_name = "tegra-pcm-audio",
	.cpu_dai_name = "tegra-i2s.0",
	.codec_dai_name = "wm8903-hifi",
	.init = harmony_asoc_init,
	.ops = &harmony_asoc_ops,
};

static struct snd_soc_card snd_soc_harmony = {
	.name = "tegra-harmony",
	.dai_link = &harmony_wm8903_dai,
	.num_links = 1,
};

static int __init harmony_soc_modinit(void)
{
	int ret;

	if (!machine_is_harmony() && !machine_is_seaboard() &&
		!machine_is_ventana()) {
		pr_err(PREFIX "Not running on Tegra Harmony/Seaboard/Ventana!\n");
		return -ENODEV;
	}

	ret = tegra_asoc_utils_init();
	if (ret) {
		return ret;
	}

	/*
	 * Create and register platform device
	 */
	harmony_snd_device = platform_device_alloc("soc-audio", -1);
	if (harmony_snd_device == NULL) {
		pr_err(PREFIX "platform_device_alloc failed\n");
		ret = -ENOMEM;
		goto err_clock_utils;
	}

	platform_set_drvdata(harmony_snd_device, &snd_soc_harmony);

	ret = platform_device_add(harmony_snd_device);
	if (ret) {
		pr_err(PREFIX "platform_device_add failed (%d)\n",
			ret);
		goto err_device_put;
	}

	return 0;

err_device_put:
	platform_device_put(harmony_snd_device);
err_clock_utils:
	tegra_asoc_utils_fini();
	return ret;
}
module_init(harmony_soc_modinit);

static void __exit harmony_soc_modexit(void)
{
	platform_device_unregister(harmony_snd_device);

	tegra_asoc_utils_fini();
}
module_exit(harmony_soc_modexit);

MODULE_AUTHOR("Stephen Warren <swarren@nvidia.com>");
MODULE_DESCRIPTION("Harmony machine ASoC driver");
MODULE_LICENSE("GPL");
