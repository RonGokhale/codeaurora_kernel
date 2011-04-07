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
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/mfd/wcd9310/core.h>
#include <linux/mfd/wcd9310/pdata.h>
#include <linux/mfd/wcd9310/registers.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/tlv.h>
#include <linux/delay.h>
#include "wcd9310.h"

static const DECLARE_TLV_DB_SCALE(pga_tlv, -8400, 28, 0);

enum tabla_bandgap_type {
	TABLA_BANDGAP_OFF = 0,
	TABLA_BANDGAP_AUDIO_MODE,
};

struct tabla_priv { /* member undecided */
	struct snd_soc_codec *codec;
	u32 ref_cnt;
	struct tabla_pdata *pdata;
	enum tabla_bandgap_type bandgap_type;
	bool clock_active;
};

static int tabla_codec_enable_charge_pump(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;
	struct tabla_priv *tabla = snd_soc_codec_get_drvdata(codec);

	pr_debug("%s %d\n", __func__, event);
	if ((tabla->bandgap_type != TABLA_BANDGAP_AUDIO_MODE) ||
		(!tabla->clock_active)) {
		pr_err("%s: Error, Tabla must have clocks enabled for charge"
			"pump\n", __func__);
		return -EINVAL;
	}
	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		snd_soc_update_bits(codec, TABLA_A_CP_EN, 0x01, 0x01);
		snd_soc_update_bits(codec, TABLA_A_CDC_CLK_OTHR_CTL, 0x01,
			0x01);
		snd_soc_update_bits(codec, TABLA_A_CDC_CLSG_CTL, 0x08, 0x08);
		usleep_range(200, 200);
		snd_soc_update_bits(codec, TABLA_A_CP_STATIC, 0x10, 0x00);
		break;
	case SND_SOC_DAPM_POST_PMD:
		snd_soc_update_bits(codec, TABLA_A_CDC_CLK_OTHR_RESET_CTL, 0x10,
			0x10);
		usleep_range(20, 20);
		snd_soc_update_bits(codec, TABLA_A_CP_STATIC, 0x08, 0x08);
		snd_soc_update_bits(codec, TABLA_A_CP_STATIC, 0x10, 0x10);
		snd_soc_update_bits(codec, TABLA_A_CDC_CLSG_CTL, 0x08, 0x00);
		snd_soc_update_bits(codec, TABLA_A_CDC_CLK_OTHR_CTL, 0x01,
			0x00);

		/* TODO disable PA */

		snd_soc_update_bits(codec, TABLA_A_CP_STATIC, 0x08, 0x00);
		snd_soc_update_bits(codec, TABLA_A_CP_EN, 0x01, 0x00);
		break;
	}
	return 0;
}

static const struct snd_kcontrol_new tabla_snd_controls[] = {
	SOC_SINGLE_TLV("RX1 Digital Volume", TABLA_A_CDC_RX1_VOL_CTL_B2_CTL, 4,
		127, 0, pga_tlv),
	SOC_SINGLE_TLV("TX6 Digital Volume", TABLA_A_TX_1_2_EN, 4, 127, 0,
		pga_tlv),
	/* TODO make the gain follow the correct scale */

	SOC_SINGLE("MICBIAS1 CAPLESS Switch", TABLA_A_MICB_1_CTL, 4, 1, 1),
};
static const char *rx_mix1_text[] = {
	"ZERO", "SRC1", "SRC2", "IIR1", "IIR2", "RX1", "RX2", "RX3", "RX4",
		"RX5", "RX6", "RX7"
};

static const char *sb_tx6_mux_text[] = {
	"ZERO", "RMIX1", "RMIX2", "RMIX3", "RMIX4", "RMIX5", "RMIX6", "RMIX7",
		"DEC6"
};
static const char *dec6_mux_text[] = {
	"ZERO", "DMIC1", "DMIC6", "ADC1", "ADC6", "RSVD1", "RSVD2"
};

static const struct soc_enum rx_mix1_inp1_chain_enum =
	SOC_ENUM_SINGLE(TABLA_A_CDC_CONN_RX1_B1_CTL, 0, 12, rx_mix1_text);

static const struct soc_enum sb_tx6_mux_enum =
	SOC_ENUM_SINGLE(TABLA_A_CDC_CONN_TX_SB_B6_CTL, 0, 9, sb_tx6_mux_text);

static const struct soc_enum dec6_mux_enum =
	SOC_ENUM_SINGLE(TABLA_A_CDC_CONN_TX_B2_CTL, 0, 7, dec6_mux_text);

static const struct snd_kcontrol_new rx_mix1_inp1_mux =
	SOC_DAPM_ENUM("RX MIX1 INP1 Mux", rx_mix1_inp1_chain_enum);

static const struct snd_kcontrol_new sb_tx6_mux =
	SOC_DAPM_ENUM("TX6 MUX Mux", sb_tx6_mux_enum);

static const struct snd_kcontrol_new dec6_mux =
	SOC_DAPM_ENUM("DEC6 MUX Mux", dec6_mux_enum);

static int tabla_codec_enable_adc1(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;

	pr_debug("%s %d\n", __func__, event);
	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		snd_soc_update_bits(codec, TABLA_A_TX_1_2_TEST_CTL, 0x80, 0x80);
		usleep_range(1000, 1000);
		snd_soc_update_bits(codec, TABLA_A_TX_1_2_TEST_CTL, 0x80, 0x00);
		usleep_range(1000, 1000);
		break;
	}
	return 0;
}

static int tabla_codec_enable_adc_block(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;

	pr_debug("%s %d\n", __func__, event);
	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		snd_soc_update_bits(codec, TABLA_A_TX_COM_BIAS, 0xE0, 0xE0);
		break;
	case SND_SOC_DAPM_POST_PMD:
		snd_soc_update_bits(codec, TABLA_A_TX_COM_BIAS, 0xE0, 0x00);
		break;
	}
	return 0;
}

static int tabla_codec_enable_pamp_gain(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;

	pr_debug("%s %d\n", __func__, event);
	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		snd_soc_update_bits(codec, TABLA_A_RX_EAR_GAIN, 0x80, 0x80);
		break;
	case SND_SOC_DAPM_POST_PMD:
		snd_soc_update_bits(codec, TABLA_A_RX_EAR_GAIN, 0x80, 0x00);
		break;
	}
	return 0;
}

static const struct snd_soc_dapm_widget tabla_dapm_widgets[] = {
	/*RX stuff */
	SND_SOC_DAPM_OUTPUT("EAR"),

	SND_SOC_DAPM_PGA_E("EAR PA", TABLA_A_RX_EAR_EN, 3, 0, NULL, 0,
		tabla_codec_enable_pamp_gain, SND_SOC_DAPM_POST_PMU |
			SND_SOC_DAPM_PRE_PMD),

	SND_SOC_DAPM_PGA("EAR PA Input", TABLA_A_CDC_CLSG_CTL, 2, 0, NULL, 0),

	SND_SOC_DAPM_DAC("DAC1 REF", NULL, TABLA_A_RX_EAR_EN, 5, 0),
	SND_SOC_DAPM_DAC("DAC1", NULL, TABLA_A_RX_EAR_EN, 6, 0),

	SND_SOC_DAPM_SUPPLY("RX1 CP", TABLA_A_CDC_RX1_B6_CTL, 5, 0,
		tabla_codec_enable_charge_pump, SND_SOC_DAPM_POST_PMU |
			SND_SOC_DAPM_PRE_PMD),

	SND_SOC_DAPM_SUPPLY("RX1 CLOCK", TABLA_A_CDC_CLK_RX_B1_CTL, 0, 0, NULL,
		0),
	SND_SOC_DAPM_SUPPLY("RX1 BIAS", TABLA_A_RX_COM_BIAS, 7, 0, NULL, 0),

	SND_SOC_DAPM_MUX("RX MIX1 INP1", SND_SOC_NOPM, 0, 0, &rx_mix1_inp1_mux),
	SND_SOC_DAPM_AIF_IN("RX1", "AIF1 Playback", 0, TABLA_A_RX_COM_BIAS, 7,
		0),

	/* TX */
	SND_SOC_DAPM_INPUT("AMIC1"),
	/* TODO Enable CFILTER */
	SND_SOC_DAPM_MICBIAS("MIC BIAS1", TABLA_A_MICB_1_CTL, 7, 0),
	SND_SOC_DAPM_ADC_E("ADC1", NULL, TABLA_A_TX_1_2_EN, 7, 0,
		tabla_codec_enable_adc1, SND_SOC_DAPM_PRE_PMU),

	SND_SOC_DAPM_ADC_E("ADC BLOCK", NULL, TABLA_A_CDC_CLK_OTHR_CTL, 1, 0,
		tabla_codec_enable_adc_block, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_MUX("DEC6 MUX", TABLA_A_CDC_CLK_OTHR_CTL, 2, 0, &dec6_mux),
	SND_SOC_DAPM_MUX("TX6 MUX", 0x308, 5, 0, &sb_tx6_mux),
	SND_SOC_DAPM_AIF_OUT("TX6", "AIF1 Capture", NULL, TABLA_A_TX_1_2_EN, 7,
		0),

};

static const struct snd_soc_dapm_route audio_map[] = {
	/* Earpiece route */
	{"EAR", NULL, "EAR PA"},
	{"EAR PA", NULL, "EAR PA Input"},
	{"EAR PA Input", NULL, "DAC1"},
	{"DAC1", NULL, "DAC1 REF"},
	{"DAC1 REF", NULL, "RX MIX1 INP1"},
	{"RX MIX1 INP1", "RX1", "RX1 CLOCK"},
	{"RX1 CLOCK", NULL, "RX1 CP"},
	{"RX1 CP", NULL, "RX1 BIAS"},
	{"RX1 BIAS", NULL, "RX1"},

	/* Handset TX route */
	{"TX6", NULL, "TX6 MUX"},
	{"TX6 MUX", "DEC6", "DEC6 MUX"},
	{"DEC6 MUX", "ADC1", "ADC1"},
	{"ADC1", NULL, "ADC BLOCK"},
	{"ADC BLOCK", NULL, "MIC BIAS1"},
	{"MIC BIAS1", NULL, "AMIC1"},

};

static int tabla_readable(unsigned int reg)
{
	return tabla_reg_readable[reg];
}

static int tabla_volatile(unsigned int reg)
{
	switch (reg) {
	default:
		break;
	}
	return 1;
}

#define TABLA_FORMATS (SNDRV_PCM_FMTBIT_S16_LE \
			| SNDRV_PCM_FMTBIT_S24_LE \
			| SNDRV_PCM_FMTBIT_S32_LE)
static int tabla_write(struct snd_soc_codec *codec, unsigned int reg,
	unsigned int value)
{
	int ret;
	pr_debug("%s: write reg %x val %x\n", __func__, reg, value);

	BUG_ON(reg > TABLA_MAX_REGISTER);

	if (!tabla_volatile(codec)) {
		pr_debug("writing to cache\n");
		ret = snd_soc_cache_write(codec, reg, value);
		if (ret != 0)
			dev_err(codec->dev, "Cache write to %x failed: %d\n",
				reg, ret);
	}

	return tabla_reg_write(codec->control_data, reg, value);
}
static unsigned int tabla_read(struct snd_soc_codec *codec,
				unsigned int reg)
{
	unsigned int val;
	int ret;

	BUG_ON(reg > TABLA_MAX_REGISTER);

	if (!tabla_volatile(reg) && tabla_readable(reg) &&
		reg < codec->driver->reg_cache_size) {
		pr_debug("reading from cache\n");
		ret = snd_soc_cache_read(codec, reg, &val);
		if (ret >= 0) {
			pr_debug("register %d, value %d\n", reg, val);
			return val;
		} else
			dev_err(codec->dev, "Cache read from %x failed: %d\n",
				reg, ret);
	}

	val = tabla_reg_read(codec->control_data, reg);
	pr_debug("%s: read reg %x val %x\n", __func__, reg, val);
	return val;
}

static int tabla_codec_enable_bandgap(struct snd_soc_codec *codec,
	enum tabla_bandgap_type choice)
{
	struct tabla_priv *tabla = snd_soc_codec_get_drvdata(codec);

	pr_debug("%s, choice is %d, current is %d\n", __func__, choice,
		tabla->bandgap_type);

	if (tabla->bandgap_type == choice)
		return 0;

	if ((tabla->bandgap_type == TABLA_BANDGAP_OFF) &&
		(choice == TABLA_BANDGAP_AUDIO_MODE)) {
		snd_soc_update_bits(codec, TABLA_A_BIAS_CENTRAL_BG_CTL, 0x80,
			0x80);
		snd_soc_update_bits(codec, TABLA_A_BIAS_CENTRAL_BG_CTL, 0x04,
			0x04);
		snd_soc_update_bits(codec, TABLA_A_BIAS_CENTRAL_BG_CTL, 0x01,
			0x01);
		usleep_range(1000, 1000);
		snd_soc_update_bits(codec, TABLA_A_BIAS_CENTRAL_BG_CTL, 0x80,
			0x00);
	} else if (choice == TABLA_BANDGAP_OFF) {
		snd_soc_write(codec, TABLA_A_BIAS_CENTRAL_BG_CTL, 0x05);
	} else {
		pr_err("%s: Error, Invalid bandgap settings\n", __func__);
			return -EINVAL;
	}
	tabla->bandgap_type = choice;

	return 0;
}
static int tabla_codec_enable_clock_block(struct snd_soc_codec *codec)
{
	struct tabla_priv *tabla = snd_soc_codec_get_drvdata(codec);

	pr_debug("%s\n", __func__);

	/* Check that codec is in audio mode */
	if ((tabla->clock_active) || (tabla->bandgap_type !=
		TABLA_BANDGAP_AUDIO_MODE)) {
		pr_err("%s: Error, Tabla must be in audio mode when configuring"
			"clocks\n", __func__);
		return -EINVAL;
	}

	snd_soc_update_bits(codec, TABLA_A_CLK_BUFF_EN1, 1 << 4, 1 << 4);
	usleep_range(1000, 1000);
	snd_soc_update_bits(codec, TABLA_A_CLK_BUFF_EN2, 0x02, 0x00);
	snd_soc_update_bits(codec, TABLA_A_CLK_BUFF_EN2, 0x04, 0x04);
	snd_soc_update_bits(codec, TABLA_A_CDC_CLK_MCLK_CTL, 0x01, 0x01);
	usleep_range(50, 50);
	tabla->clock_active = true;
	return 0;
}
static void tabla_codec_disable_clock_block(struct snd_soc_codec *codec)
{
	struct tabla_priv *tabla = snd_soc_codec_get_drvdata(codec);
	pr_debug("%s\n", __func__);
	snd_soc_update_bits(codec, TABLA_A_CLK_BUFF_EN2, 0x04, 0x00);
	ndelay(160);
	snd_soc_update_bits(codec, TABLA_A_CLK_BUFF_EN2, 0x02, 0x02);
	snd_soc_update_bits(codec, TABLA_A_CLK_BUFF_EN1, 0x04, 0x00);
	tabla->clock_active = false;
}
static void tabla_codec_bring_up(struct snd_soc_codec *codec)
{
	pr_debug("%s\n", __func__);

	snd_soc_update_bits(codec, TABLA_A_LEAKAGE_CTL, 0x4, 0x4);
	snd_soc_update_bits(codec, TABLA_A_CDC_CTL, 1, 0);
	usleep_range(5000, 5000);
	snd_soc_update_bits(codec, TABLA_A_CDC_CTL, 1, 1);
}
static void tabla_codec_bring_down(struct snd_soc_codec *codec)
{
	pr_debug("%s\n", __func__);

	snd_soc_write(codec, TABLA_A_LEAKAGE_CTL, 0x7);
	snd_soc_write(codec, TABLA_A_LEAKAGE_CTL, 0x6);
	snd_soc_write(codec, TABLA_A_LEAKAGE_CTL, 0xe);
	snd_soc_write(codec, TABLA_A_LEAKAGE_CTL, 0x8);
}

static int tabla_startup(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct tabla_priv *tabla = snd_soc_codec_get_drvdata(codec);
	int ret = 0;

	pr_debug("%s()\n", __func__);

	if (!codec) {
		pr_err("Error, no codec found\n");
		return -EINVAL;
	}
	if (!tabla->ref_cnt)
		tabla_codec_bring_up(codec);

	tabla->ref_cnt++;

	if (tabla->bandgap_type != TABLA_BANDGAP_AUDIO_MODE) {
		ret = tabla_codec_enable_bandgap(codec,
			TABLA_BANDGAP_AUDIO_MODE);
		if (ret) {
			pr_err("%s: Enabling bandgap failed\n", __func__);
			goto err_bandgap;
		}
	}
	if (!tabla->clock_active)
		tabla_codec_enable_clock_block(codec);


	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		/* Enable LDO */
		snd_soc_update_bits(codec, TABLA_A_LDO_H_MODE_1, 0xC, 0x0);
		snd_soc_update_bits(codec, TABLA_A_MICB_CFILT_1_VAL, 0xFC,
			0xA0);
		snd_soc_update_bits(codec, TABLA_A_LDO_H_MODE_1, 0x80, 0x80);
		usleep_range(1000, 1000);
	}

	return ret;
err_bandgap:
	tabla->ref_cnt--;
	if (!tabla->ref_cnt)
		tabla_codec_bring_down(codec);

	return ret;
}

static void tabla_shutdown(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct tabla_priv *tabla = snd_soc_codec_get_drvdata(codec);

	pr_debug("%s()\n", __func__);

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		/* Disable LDO */
		snd_soc_update_bits(codec, TABLA_A_LDO_H_MODE_1, 0x80, 0x00);
		usleep_range(1000, 1000);
	}

	if (!tabla->ref_cnt) {
		pr_err("Error, trying to shutdown codec when already down\n");
		return;
	}
	tabla->ref_cnt--;

	if (!tabla->ref_cnt) {
		tabla_codec_disable_clock_block(codec);
		tabla_codec_enable_bandgap(codec, TABLA_BANDGAP_OFF);
	}
}

static int tabla_digital_mute(struct snd_soc_dai *codec_dai, int mute)
{
	struct snd_soc_codec *codec = codec_dai->codec;

	pr_debug("%s %d\n", __func__, mute);

	/* TODO mute TX */
	if (mute)
		snd_soc_update_bits(codec, TABLA_A_CDC_RX1_B6_CTL, 0x01, 0x01);
	else
		snd_soc_update_bits(codec, TABLA_A_CDC_RX1_B6_CTL, 0x01, 0x00);

	return 0;
}

static struct snd_soc_dai_ops tabla_dai_ops = {
	.startup = tabla_startup,
	.shutdown = tabla_shutdown,
	.digital_mute = tabla_digital_mute,
};
static struct snd_soc_dai_driver tabla_dai[] = {
	{
		.name = "tabla_rx1",
		.id = 1,
		.playback = {
			.stream_name = "AIF1 Playback",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = TABLA_FORMATS,
			.rate_max = 48000,
			.rate_min = 8000,
			.channels_min = 1,
			.channels_max = 2,
		},
		.ops = &tabla_dai_ops,
	},
	{
		.name = "tabla_tx1",
		.id = 2,
		.capture = {
			.stream_name = "AIF1 Capture",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = TABLA_FORMATS,
			.rate_max = 48000,
			.rate_min = 8000,
			.channels_min = 1,
			.channels_max = 2,
		},
		.ops = &tabla_dai_ops,
	},
};

static int tabla_codec_probe(struct snd_soc_codec *codec)
{
	struct tabla *control;
	struct tabla_priv *tabla;
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	int ret = 0;

	codec->control_data = dev_get_drvdata(codec->dev->parent);
	control = codec->control_data;

	tabla = kzalloc(sizeof(struct tabla_priv), GFP_KERNEL);
	if (!tabla) {
		dev_err(codec->dev, "Failed to allocate private data\n");
		return -ENOMEM;
	}

	snd_soc_codec_set_drvdata(codec, tabla);
	tabla->pdata = dev_get_platdata(codec->dev->parent);
	if (!tabla->pdata)
		pr_info("%s: No Tabla Platform Data\n", __func__);

	tabla->ref_cnt = 0;
	tabla->bandgap_type = TABLA_BANDGAP_OFF;
	tabla->clock_active = false;
	tabla->codec = codec;

	snd_soc_add_controls(codec, tabla_snd_controls,
		ARRAY_SIZE(tabla_snd_controls));
	snd_soc_dapm_new_controls(dapm, tabla_dapm_widgets,
		ARRAY_SIZE(tabla_dapm_widgets));
	snd_soc_dapm_add_routes(dapm, audio_map, ARRAY_SIZE(audio_map));

	return ret;
}
static int tabla_codec_remove(struct snd_soc_codec *codec)
{
	struct tabla_priv *tabla = snd_soc_codec_get_drvdata(codec);
	kfree(tabla);
	return 0;
}
static struct snd_soc_codec_driver soc_codec_dev_tabla = {
	.probe	= tabla_codec_probe,
	.remove	= tabla_codec_remove,
	.read = tabla_read,
	.write = tabla_write,

	.readable_register = tabla_readable,
	.volatile_register = tabla_volatile,

	.reg_cache_size = TABLA_CACHE_SIZE,
	.reg_cache_default = tabla_reg_defaults,
	.reg_word_size = 1,
};
static int __devinit tabla_probe(struct platform_device *pdev)
{
	return snd_soc_register_codec(&pdev->dev, &soc_codec_dev_tabla,
		tabla_dai, ARRAY_SIZE(tabla_dai));
}
static int __devexit tabla_remove(struct platform_device *pdev)
{
	snd_soc_unregister_codec(&pdev->dev);
	return 0;
}
static struct platform_driver tabla_codec_driver = {
	.probe = tabla_probe,
	.remove = tabla_remove,
	.driver = {
		.name = "tabla_codec",
		.owner = THIS_MODULE,
	},
};

static int __init tabla_codec_init(void)
{
	return platform_driver_register(&tabla_codec_driver);
}

static void __exit tabla_codec_exit(void)
{
	platform_driver_unregister(&tabla_codec_driver);
}

module_init(tabla_codec_init);
module_exit(tabla_codec_exit);

MODULE_DESCRIPTION("Tabla codec driver");
MODULE_VERSION("1.0");
MODULE_LICENSE("GPL v2");
