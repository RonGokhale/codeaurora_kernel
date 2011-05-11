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
	enum tabla_bandgap_type bandgap_type;
	bool clock_active;
};

static int tabla_codec_enable_charge_pump(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;
	struct tabla_priv *tabla = snd_soc_codec_get_drvdata(codec);

	pr_debug("%s %d\n", __func__, event);
	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		if ((tabla->bandgap_type != TABLA_BANDGAP_AUDIO_MODE) ||
			(!tabla->clock_active)) {
			pr_err("%s: Error, Tabla must have clocks enabled for "
				"charge pump\n", __func__);
			return -EINVAL;
		}

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
		snd_soc_update_bits(codec, TABLA_A_CP_STATIC, 0x08, 0x00);
		snd_soc_update_bits(codec, TABLA_A_CP_EN, 0x01, 0x00);
		break;
	}
	return 0;
}

static const struct snd_kcontrol_new tabla_snd_controls[] = {
	SOC_SINGLE_TLV("LINEOUT1 Volume", TABLA_A_RX_LINE_1_GAIN, 0, 127, 0,
		pga_tlv),
	SOC_SINGLE_TLV("LINEOUT3 Volume", TABLA_A_RX_LINE_3_GAIN, 0, 127, 0,
		pga_tlv),
	SOC_SINGLE_TLV("HPHL Volume", TABLA_A_RX_HPH_L_GAIN, 0, 127, 0,
		pga_tlv),
	SOC_SINGLE_TLV("HPHR Volume", TABLA_A_RX_HPH_R_GAIN, 0, 127, 0,
		pga_tlv),

	SOC_SINGLE_TLV("SLIM RX1 Digital Volume",
		TABLA_A_CDC_RX1_VOL_CTL_B2_CTL, 4, 127, 0, pga_tlv),
	SOC_SINGLE_TLV("SLIM TX6 Digital Volume", TABLA_A_TX_1_2_EN, 4, 127, 0,
		pga_tlv),
	/* TODO make the gain follow the correct scale */

	SOC_SINGLE("MICBIAS1 CAPLESS Switch", TABLA_A_MICB_1_CTL, 4, 1, 1),
};

static const char *rx_mix1_text[] = {
	"ZERO", "SRC1", "SRC2", "IIR1", "IIR2", "RX1", "RX2", "RX3", "RX4",
		"RX5", "RX6", "RX7"
};

static const char *sb_tx1_mux_text[] = {
	"ZERO", "RMIX1", "RMIX2", "RMIX3", "RMIX4", "RMIX5", "RMIX6", "RMIX7",
		"DEC1"
};

static const char *sb_tx6_mux_text[] = {
	"ZERO", "RMIX1", "RMIX2", "RMIX3", "RMIX4", "RMIX5", "RMIX6", "RMIX7",
		"DEC6"
};

static const char *dec1_mux_text[] = {
	"ZERO", "DMIC1", "ADC6",
};

static const char *dec6_mux_text[] = {
	"ZERO", "DMIC6", "ADC1",
};

static const char *iir1_inp1_text[] = {
	"ZERO", "DEC1", "DEC2", "DEC3", "DEC4", "DEC5", "DEC6", "DEC7", "DEC8",
	"DEC9", "DEC10", "RX1", "RX2", "RX3", "RX4", "RX5", "RX6", "RX7"
};

static const struct soc_enum rx_mix1_inp1_chain_enum =
	SOC_ENUM_SINGLE(TABLA_A_CDC_CONN_RX1_B1_CTL, 0, 12, rx_mix1_text);

static const struct soc_enum rx2_mix1_inp1_chain_enum =
	SOC_ENUM_SINGLE(TABLA_A_CDC_CONN_RX2_B1_CTL, 0, 12, rx_mix1_text);

static const struct soc_enum rx3_mix1_inp1_chain_enum =
	SOC_ENUM_SINGLE(TABLA_A_CDC_CONN_RX3_B1_CTL, 0, 12, rx_mix1_text);

static const struct soc_enum rx4_mix1_inp1_chain_enum =
	SOC_ENUM_SINGLE(TABLA_A_CDC_CONN_RX4_B1_CTL, 0, 12, rx_mix1_text);

static const struct soc_enum sb_tx6_mux_enum =
	SOC_ENUM_SINGLE(TABLA_A_CDC_CONN_TX_SB_B6_CTL, 0, 9, sb_tx6_mux_text);

static const struct soc_enum sb_tx1_mux_enum =
	SOC_ENUM_SINGLE(TABLA_A_CDC_CONN_TX_SB_B7_CTL, 0, 9, sb_tx1_mux_text);

static const struct soc_enum dec1_mux_enum =
	SOC_ENUM_SINGLE(TABLA_A_CDC_CONN_TX_B1_CTL, 0, 3, dec1_mux_text);

static const struct soc_enum dec6_mux_enum =
	SOC_ENUM_SINGLE(TABLA_A_CDC_CONN_TX_B2_CTL, 3, 3, dec6_mux_text);

static const struct soc_enum iir1_inp1_mux_enum =
	SOC_ENUM_SINGLE(TABLA_A_CDC_CONN_EQ1_B1_CTL, 0, 18, iir1_inp1_text);

static const struct snd_kcontrol_new rx_mix1_inp1_mux =
	SOC_DAPM_ENUM("RX1 MIX1 INP1 Mux", rx_mix1_inp1_chain_enum);

static const struct snd_kcontrol_new rx2_mix1_inp1_mux =
	SOC_DAPM_ENUM("RX2 MIX1 INP1 Mux", rx2_mix1_inp1_chain_enum);

static const struct snd_kcontrol_new rx3_mix1_inp1_mux =
	SOC_DAPM_ENUM("RX3 MIX1 INP1 Mux", rx3_mix1_inp1_chain_enum);

static const struct snd_kcontrol_new rx4_mix1_inp1_mux =
	SOC_DAPM_ENUM("RX4 MIX1 INP1 Mux", rx4_mix1_inp1_chain_enum);

static const struct snd_kcontrol_new sb_tx6_mux =
	SOC_DAPM_ENUM("SLIM TX6 MUX Mux", sb_tx6_mux_enum);

static const struct snd_kcontrol_new sb_tx1_mux =
	SOC_DAPM_ENUM("SLIM TX1 MUX Mux", sb_tx1_mux_enum);

static const struct snd_kcontrol_new dec1_mux =
	SOC_DAPM_ENUM("DEC1 MUX Mux", dec1_mux_enum);

static const struct snd_kcontrol_new dec6_mux =
	SOC_DAPM_ENUM("DEC6 MUX Mux", dec6_mux_enum);

static const struct snd_kcontrol_new iir1_inp1_mux =
	SOC_DAPM_ENUM("IIR1 INP1 Mux", iir1_inp1_mux_enum);

static const struct snd_kcontrol_new dac1_control =
	SOC_DAPM_SINGLE("Switch", TABLA_A_RX_EAR_EN, 5, 1, 0);

static const struct snd_kcontrol_new hphl_switch =
	SOC_DAPM_SINGLE("Switch", TABLA_A_RX_HPH_L_DAC_CTL, 6, 1, 0);

static const struct snd_kcontrol_new hphr_switch =
	SOC_DAPM_SINGLE("Switch", TABLA_A_RX_HPH_R_DAC_CTL, 6, 1, 0);

static const struct snd_kcontrol_new lineout1_switch =
	SOC_DAPM_SINGLE("Switch", TABLA_A_RX_LINE_1_DAC_CTL, 6, 1, 0);

static const struct snd_kcontrol_new lineout3_switch =
	SOC_DAPM_SINGLE("Switch", TABLA_A_RX_LINE_3_DAC_CTL, 6, 1, 0);

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
static int tabla_codec_delay_40ms(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	pr_debug("%s %d\n", __func__, event);
	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		usleep_range(40, 40);
		break;
	}
	return 0;
}

static int tabla_codec_enable_dmic1(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;

	pr_debug("%s %d\n", __func__, event);
	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		snd_soc_update_bits(codec, TABLA_A_CDC_TX1_MUX_CTL, 0x1, 0x1);
		snd_soc_update_bits(codec, TABLA_A_CDC_CLK_DMIC_CTL, 0x2, 0x2);
		snd_soc_update_bits(codec, TABLA_A_CDC_TX1_DMIC_CTL, 0x1, 0x1);
		snd_soc_update_bits(codec, TABLA_A_CDC_CLK_DMIC_CTL, 0x1, 0x1);
		break;
	case SND_SOC_DAPM_POST_PMD:
		snd_soc_update_bits(codec, TABLA_A_CDC_TX1_DMIC_CTL, 0x1, 0);
		snd_soc_update_bits(codec, TABLA_A_CDC_CLK_DMIC_CTL, 0x3, 0);
		break;
	}
	return 0;
}

static int tabla_codec_enable_micbias1(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;

	pr_debug("%s %d\n", __func__, event);
	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		snd_soc_update_bits(codec, TABLA_A_MICB_CFILT_1_CTL, 0x40,
			0x40);
		snd_soc_update_bits(codec, TABLA_A_MICB_CFILT_1_CTL, 0x80,
			0x80);
		break;
	case SND_SOC_DAPM_POST_PMD:
		snd_soc_update_bits(codec, TABLA_A_MICB_CFILT_2_CTL, 0x80, 0);
		snd_soc_update_bits(codec, TABLA_A_MICB_CFILT_2_CTL, 0x40, 0);
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

	SND_SOC_DAPM_SWITCH("DAC1", TABLA_A_RX_EAR_EN, 6, 0, &dac1_control),
	SND_SOC_DAPM_PGA_E("RX1 CP", TABLA_A_CDC_RX1_B6_CTL, 5, 0, NULL, 0,
		tabla_codec_enable_charge_pump, SND_SOC_DAPM_POST_PMU |
			SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_PGA("RX BIAS", TABLA_A_RX_COM_BIAS, 7, 0, NULL, 0),
	SND_SOC_DAPM_MUX("RX1 MIX1 INP1", TABLA_A_CDC_CLK_RX_B1_CTL, 0, 0,
		&rx_mix1_inp1_mux),
	SND_SOC_DAPM_AIF_IN("SLIM RX1", "AIF1 Playback", 0, SND_SOC_NOPM, 0, 0),

	/* RX 2 path */
	SND_SOC_DAPM_PGA_E("RX2 CP", TABLA_A_CDC_RX2_B6_CTL, 5, 0, NULL, 0,
		tabla_codec_enable_charge_pump, SND_SOC_DAPM_POST_PMU |
			SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_MUX("RX2 MIX1 INP1", TABLA_A_CDC_CLK_RX_B1_CTL, 1, 0,
		&rx2_mix1_inp1_mux),
	SND_SOC_DAPM_AIF_IN("SLIM RX2", "AIF1 Playback", 0, SND_SOC_NOPM, 0, 0),

	/* Headphone */
	SND_SOC_DAPM_OUTPUT("HEADPHONE"),
	SND_SOC_DAPM_PGA("HPHL", TABLA_A_RX_HPH_CNP_EN, 5, 0, NULL, 0),
	SND_SOC_DAPM_SWITCH("HPHL DAC", TABLA_A_RX_HPH_L_DAC_CTL, 7, 0,
		&hphl_switch),

	SND_SOC_DAPM_PGA("HPHR", TABLA_A_RX_HPH_CNP_EN, 4, 0, NULL, 0),
	SND_SOC_DAPM_SWITCH("HPHR DAC", TABLA_A_RX_HPH_R_DAC_CTL, 7, 0,
		&hphr_switch),

	/* Speaker */
	SND_SOC_DAPM_OUTPUT("LINEOUT"),
	SND_SOC_DAPM_PGA("LINEOUT1", TABLA_A_RX_LINE_CNP_EN, 0, 0, NULL, 0),

	SND_SOC_DAPM_SWITCH("LINEOUT1 DAC", TABLA_A_RX_LINE_1_DAC_CTL, 7, 0,
		&lineout1_switch),
	SND_SOC_DAPM_MUX("RX3 MIX1 INP1", TABLA_A_CDC_CLK_RX_B1_CTL, 2, 0,
		&rx3_mix1_inp1_mux),

	SND_SOC_DAPM_PGA_E("LINEOUT3", TABLA_A_RX_LINE_CNP_EN, 2, 0, NULL, 0,
		tabla_codec_delay_40ms, SND_SOC_DAPM_PRE_PMU),

	SND_SOC_DAPM_SWITCH("LINEOUT3 DAC", TABLA_A_RX_LINE_3_DAC_CTL, 7, 0,
		&lineout3_switch),
	SND_SOC_DAPM_MUX("RX4 MIX1 INP1", TABLA_A_CDC_CLK_RX_B1_CTL, 3, 0,
		&rx4_mix1_inp1_mux),

	/* TX */
	SND_SOC_DAPM_INPUT("AMIC1"),
	SND_SOC_DAPM_MICBIAS_E("MIC BIAS1", TABLA_A_MICB_1_CTL, 7, 0,
		tabla_codec_enable_micbias1, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_ADC_E("ADC1", NULL, TABLA_A_TX_1_2_EN, 7, 0,
		tabla_codec_enable_adc1, SND_SOC_DAPM_PRE_PMU),

	SND_SOC_DAPM_ADC_E("ADC BLOCK", NULL, TABLA_A_CDC_CLK_OTHR_CTL, 1, 0,
		tabla_codec_enable_adc_block, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_MUX("DEC6 MUX", TABLA_A_CDC_CLK_OTHR_CTL, 2, 0, &dec6_mux),
	SND_SOC_DAPM_MUX("SLIM TX6 MUX", TABLA_A_CDC_CLK_TX_CLK_EN_B1_CTL, 5, 0,
		&sb_tx6_mux),
	SND_SOC_DAPM_AIF_OUT("SLIM TX6", "AIF1 Capture", NULL,
		TABLA_A_TX_1_2_EN, 7, 0),

	/* Digital Mic */
	SND_SOC_DAPM_INPUT("DMIC OUT"),
	SND_SOC_DAPM_MIC("DMIC1", &tabla_codec_enable_dmic1),
	SND_SOC_DAPM_MUX("DEC1 MUX", TABLA_A_CDC_CLK_TX_CLK_EN_B1_CTL, 0, 0,
		&dec1_mux),
	SND_SOC_DAPM_MUX("SLIM TX1 MUX", SND_SOC_NOPM, 0, 0, &sb_tx1_mux),
	SND_SOC_DAPM_AIF_OUT("SLIM TX1", "AIF1 Capture", NULL, SND_SOC_NOPM, 0,
		0),


	/* Sidetone */
	SND_SOC_DAPM_MUX("IIR1 INP1 MUX", SND_SOC_NOPM, 0, 0, &iir1_inp1_mux),
	SND_SOC_DAPM_PGA("IIR1", TABLA_A_CDC_CLK_SD_CTL, 0, 0, NULL, 0),

};

static const struct snd_soc_dapm_route audio_map[] = {
	/* SLIMBUS Connections */
	{"RX BIAS", NULL, "SLIM RX1"},
	{"RX BIAS", NULL, "SLIM RX2"},

	{"SLIM TX1", NULL, "SLIM TX1 MUX"},
	{"SLIM TX1 MUX", "DEC1", "DEC1 MUX"},
	{"SLIM TX6", NULL, "SLIM TX6 MUX"},
	{"SLIM TX6 MUX", "DEC6", "DEC6 MUX"},

	/* Earpiece (RX MIX1) */
	{"EAR", NULL, "EAR PA"},
	{"EAR PA", NULL, "EAR PA Input"},
	{"EAR PA Input", NULL, "DAC1"},
	{"DAC1", "Switch", "RX1 CP"},
	{"RX1 CP", NULL, "RX1 MIX1 INP1"},
	{"RX1 MIX1 INP1", "RX1", "RX BIAS"},

	/* Headset (RX MIX1 and RX MIX2) */
	{"HEADPHONE", NULL, "HPHL"},
	{"HPHL", NULL, "HPHL DAC"},
	{"HPHL DAC", "Switch", "RX1 MIX1 INP1"},

	{"HEADPHONE", NULL, "HPHR"},
	{"HPHR", NULL, "HPHR DAC"},
	{"HPHR DAC", "Switch", "RX2 CP"},
	{"RX2 CP", NULL, "RX2 MIX1 INP1"},
	{"RX2 MIX1 INP1", "RX2", "RX BIAS"},

	/* Speaker (RX MIX3 and RX MIX4) */
	{"LINEOUT", NULL, "LINEOUT1"},
	{"LINEOUT1", NULL, "LINEOUT1 DAC"},
	{"LINEOUT1 DAC", "Switch", "RX3 MIX1 INP1"},
	{"RX3 MIX1 INP1", "RX1", "RX BIAS"},

	{"LINEOUT", NULL, "LINEOUT3"},
	{"LINEOUT3", NULL, "LINEOUT3 DAC"},
	{"LINEOUT3 DAC", "Switch", "RX4 MIX1 INP1"},
	{"RX4 MIX1 INP1", "RX2", "RX BIAS"},

	/* Handset TX */
	{"DEC6 MUX", "ADC1", "ADC1"},
	{"ADC1", NULL, "ADC BLOCK"},
	{"ADC BLOCK", NULL, "MIC BIAS1"},
	{"MIC BIAS1", NULL, "AMIC1"},

	/* Digital Mic */
	{"DEC1 MUX", "DMIC1", "DMIC1"},
	{"DMIC1", NULL, "DMIC OUT"},

	/* Sidetone (IIR1) */
	{"RX1 MIX1 INP1", "IIR1", "IIR1"},
	{"IIR1", NULL, "IIR1 INP1 MUX"},
	{"IIR1 INP1 MUX", "DEC6", "DEC6 MUX"},

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

	if (!tabla_volatile(reg)) {
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

static int tabla_set_dai_sysclk(struct snd_soc_dai *dai,
		int clk_id, unsigned int freq, int dir)
{
	pr_debug("%s\n", __func__);
	return 0;
}

static int tabla_set_dai_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	pr_debug("%s\n", __func__);
	return 0;
}

static int tabla_hw_params(struct snd_pcm_substream *substream,
			    struct snd_pcm_hw_params *params,
			    struct snd_soc_dai *dai)
{
	pr_debug("%s: DAI-ID %x\n", __func__, dai->id);
	return 0;
}

static struct snd_soc_dai_ops tabla_dai_ops = {
	.startup = tabla_startup,
	.shutdown = tabla_shutdown,
	.hw_params = tabla_hw_params,
	.set_sysclk = tabla_set_dai_sysclk,
	.set_fmt = tabla_set_dai_fmt,
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

	tabla->ref_cnt = 0;
	tabla->bandgap_type = TABLA_BANDGAP_OFF;
	tabla->clock_active = false;
	tabla->codec = codec;

	/* Initialize gain registers to use register gain */
	snd_soc_update_bits(codec, TABLA_A_RX_HPH_L_GAIN, 0x10, 0x10);
	snd_soc_update_bits(codec, TABLA_A_RX_HPH_R_GAIN, 0x10, 0x10);
	snd_soc_update_bits(codec, TABLA_A_RX_LINE_1_GAIN, 0x10, 0x10);
	snd_soc_update_bits(codec, TABLA_A_RX_LINE_3_GAIN, 0x10, 0x10);

	/* Initialize mic biases to differential mode */
	snd_soc_update_bits(codec, TABLA_A_MICB_1_INT_RBIAS, 0x24, 0x24);
	snd_soc_update_bits(codec, TABLA_A_MICB_2_INT_RBIAS, 0x24, 0x24);
	snd_soc_update_bits(codec, TABLA_A_MICB_3_INT_RBIAS, 0x24, 0x24);
	snd_soc_update_bits(codec, TABLA_A_MICB_4_INT_RBIAS, 0x24, 0x24);

	/* Set headset CFILT to fast mode */
	snd_soc_update_bits(codec, TABLA_A_MICB_CFILT_1_CTL, 0x00, 0x00);
	snd_soc_update_bits(codec, TABLA_A_MICB_CFILT_2_CTL, 0x00, 0x00);
	snd_soc_update_bits(codec, TABLA_A_MICB_CFILT_3_CTL, 0x00, 0x00);

	snd_soc_add_controls(codec, tabla_snd_controls,
		ARRAY_SIZE(tabla_snd_controls));
	snd_soc_dapm_new_controls(dapm, tabla_dapm_widgets,
		ARRAY_SIZE(tabla_dapm_widgets));
	snd_soc_dapm_add_routes(dapm, audio_map, ARRAY_SIZE(audio_map));
	snd_soc_dapm_sync(dapm);

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
