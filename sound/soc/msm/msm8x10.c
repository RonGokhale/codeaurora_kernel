 /* Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
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

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/mfd/pm8xxx/pm8921.h>
#include <linux/qpnp/clkdiv.h>
#include <linux/io.h>
#include <sound/core.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/pcm.h>
#include <sound/jack.h>
#include <asm/mach-types.h>
#include <mach/socinfo.h>
#include <qdsp6v2/msm-pcm-routing-v2.h>
#include <sound/q6afe-v2.h>
#include <linux/module.h>
#include "../codecs/msm8x10-wcd.h"
#define DRV_NAME "msm8x10-asoc-wcd"

#define MSM8X10_DINO_LPASS_AUDIO_CORE_DIG_CODEC_CLK_SEL	0xFE03B004
#define MSM8X10_DINO_LPASS_DIGCODEC_CMD_RCGR			0xFE02C000
#define MSM8X10_DINO_LPASS_DIGCODEC_CFG_RCGR			0xFE02C004
#define MSM8X10_DINO_LPASS_DIGCODEC_M				0xFE02C008
#define MSM8X10_DINO_LPASS_DIGCODEC_N				0xFE02C00C
#define MSM8X10_DINO_LPASS_DIGCODEC_D				0xFE02C010
#define MSM8X10_DINO_LPASS_DIGCODEC_CBCR			0xFE02C014
#define MSM8X10_DINO_LPASS_DIGCODEC_AHB_CBCR			0xFE02C018

/*
 * There is limitation for the clock root selection from
 * either MI2S or DIG_CODEC.
 * If DIG_CODEC root can only provide 9.6MHz clock
 * to codec while MI2S only can provide
 * 12.288MHz.
 */
enum {
	DIG_CDC_CLK_SEL_DIG_CODEC,
	DIG_CDC_CLK_SEL_PRI_MI2S,
	DIG_CDC_CLK_SEL_SEC_MI2S,
};

static struct afe_clk_cfg mi2s_rx_clk = {
	AFE_API_VERSION_I2S_CONFIG,
	Q6AFE_LPASS_IBIT_CLK_1_P536_MHZ,
	Q6AFE_LPASS_OSR_CLK_12_P288_MHZ,
	Q6AFE_LPASS_CLK_SRC_INTERNAL,
	Q6AFE_LPASS_CLK_ROOT_DEFAULT,
	Q6AFE_LPASS_MODE_BOTH_VALID,
	0,
};

static struct afe_clk_cfg mi2s_tx_clk = {
	AFE_API_VERSION_I2S_CONFIG,
	Q6AFE_LPASS_IBIT_CLK_1_P536_MHZ,
	Q6AFE_LPASS_OSR_CLK_12_P288_MHZ,
	Q6AFE_LPASS_CLK_SRC_INTERNAL,
	Q6AFE_LPASS_CLK_ROOT_DEFAULT,
	Q6AFE_LPASS_MODE_BOTH_VALID,
	0,
};

static struct afe_digital_clk_cfg digital_cdc_clk = {
	AFE_API_VERSION_I2S_CONFIG,
	9600000,
	5,  /* Digital Codec root */
	0,
};

static atomic_t aud_init_rsc_ref;

/*
 * This function will be replaced by
 * afe_set_lpass_internal_digital_codec_clock(port_id, cfg)
 * in the future after LPASS API fix
 */
static int msm_enable_lpass_mclk(void)
{
	/* Select the codec root */
	iowrite32(DIG_CDC_CLK_SEL_DIG_CODEC,
		  ioremap(MSM8X10_DINO_LPASS_AUDIO_CORE_DIG_CODEC_CLK_SEL,
		  4));
	/* Div-2 */
	iowrite32(0x3, ioremap(MSM8X10_DINO_LPASS_DIGCODEC_CFG_RCGR, 4));
	iowrite32(0x0, ioremap(MSM8X10_DINO_LPASS_DIGCODEC_M, 4));
	iowrite32(0x0, ioremap(MSM8X10_DINO_LPASS_DIGCODEC_N, 4));
	iowrite32(0x0, ioremap(MSM8X10_DINO_LPASS_DIGCODEC_D, 4));
	/* Digital codec clock enable */
	iowrite32(0x1, ioremap(MSM8X10_DINO_LPASS_DIGCODEC_CBCR, 4));
	/* AHB clock enable */
	iowrite32(0x1, ioremap(MSM8X10_DINO_LPASS_DIGCODEC_AHB_CBCR, 4));
	/* Set the update bit to make the settings go through */
	iowrite32(0x1, ioremap(MSM8X10_DINO_LPASS_DIGCODEC_CMD_RCGR, 4));

	return 0;
}

static int msm_enable_mclk_root(u16 port_id, struct afe_digital_clk_cfg *cfg)
{
	int ret = 0;
	/*
	  * msm_enable_lpass_mclk() function call will be replaced by
	  * ret =  afe_set_lpass_internal_digital_codec_clock(port_id, cfg)
	  * in the future. Currentlt there is a bug in LPASS plan which
	  * doesn't consider the digital codec clock. It will be fixed soon
	  * in new Q6 image
	  */
	msm_enable_lpass_mclk();
	pr_err("%s(): return = %d\n", __func__, ret);
	return ret;
}

static int msm_config_mclk(u16 port_id, struct afe_digital_clk_cfg *cfg)
{
	/* Select the codec root */
	iowrite32(DIG_CDC_CLK_SEL_DIG_CODEC,
		  ioremap(MSM8X10_DINO_LPASS_AUDIO_CORE_DIG_CODEC_CLK_SEL,
		  4));
	/* Div-2 */
	iowrite32(0x3, ioremap(MSM8X10_DINO_LPASS_DIGCODEC_CFG_RCGR, 4));
	iowrite32(0x0, ioremap(MSM8X10_DINO_LPASS_DIGCODEC_M, 4));
	iowrite32(0x0, ioremap(MSM8X10_DINO_LPASS_DIGCODEC_N, 4));
	iowrite32(0x0, ioremap(MSM8X10_DINO_LPASS_DIGCODEC_D, 4));
	/* Digital codec clock enable */
	if(cfg->clk_val == 0) {
		iowrite32(0x0, ioremap(MSM8X10_DINO_LPASS_DIGCODEC_CBCR, 4));
		pr_err("%s(line %d)\n", __func__, __LINE__);
	} else {
		iowrite32(0x1, ioremap(MSM8X10_DINO_LPASS_DIGCODEC_CBCR, 4));
		pr_err("%s(line %d)\n", __func__, __LINE__);
	}
	iowrite32(0x1, ioremap(MSM8X10_DINO_LPASS_DIGCODEC_CBCR, 4));
	/* AHB clock enable */
	iowrite32(0x1, ioremap(MSM8X10_DINO_LPASS_DIGCODEC_AHB_CBCR, 4));
	/* Set the update bit to make the settings go through */
	iowrite32(0x1, ioremap(MSM8X10_DINO_LPASS_DIGCODEC_CMD_RCGR, 4));

	return 0;

}

static int msm_config_mi2s_clk(int enable)
{
	int ret = 0;
	pr_err("%s(line %d):enable = %x\n", __func__, __LINE__, enable);
	if (enable) {
		digital_cdc_clk.clk_val = 9600000;
		//if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			mi2s_rx_clk.clk_val2 = Q6AFE_LPASS_OSR_CLK_12_P288_MHZ;
			mi2s_rx_clk.clk_val1 = Q6AFE_LPASS_IBIT_CLK_1_P536_MHZ;
			ret = afe_set_lpass_clock(AFE_PORT_ID_SECONDARY_MI2S_RX,
						  &mi2s_rx_clk);
		//} else if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
			mi2s_tx_clk.clk_val2 = Q6AFE_LPASS_OSR_CLK_12_P288_MHZ;
			mi2s_tx_clk.clk_val1 = Q6AFE_LPASS_IBIT_CLK_1_P536_MHZ;
			ret = afe_set_lpass_clock(AFE_PORT_ID_PRIMARY_MI2S_RX,
						  &mi2s_tx_clk);
		//} else
		//	pr_err("%s:Not valid substream.\n", __func__);

		if (ret < 0)
			pr_err("%s:afe_set_lpass_clock failed\n", __func__);

	} else {
		digital_cdc_clk.clk_val = 0;
		//if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			mi2s_rx_clk.clk_val2 = Q6AFE_LPASS_OSR_CLK_DISABLE;
			mi2s_rx_clk.clk_val1 = Q6AFE_LPASS_IBIT_CLK_DISABLE;
			ret = afe_set_lpass_clock(AFE_PORT_ID_SECONDARY_MI2S_RX,
						  &mi2s_rx_clk);
		//} else if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
			mi2s_tx_clk.clk_val2 = Q6AFE_LPASS_OSR_CLK_DISABLE;
			mi2s_tx_clk.clk_val1 = Q6AFE_LPASS_IBIT_CLK_DISABLE;
			ret = afe_set_lpass_clock(AFE_PORT_ID_PRIMARY_MI2S_RX,
						  &mi2s_tx_clk);
		//} else
		//	pr_err("%s:Not valid substream.\n", __func__);

		if (ret < 0)
			pr_err("%s:afe_set_lpass_clock failed\n", __func__);

	}
	ret = msm_config_mclk(AFE_PORT_ID_SECONDARY_MI2S_RX, &digital_cdc_clk);
	return ret;
}


static int msm_be_hw_params_fixup(struct snd_soc_pcm_runtime *rtd,
				struct snd_pcm_hw_params *params)
{
	struct snd_interval *rate = hw_param_interval(params,
					SNDRV_PCM_HW_PARAM_RATE);

	pr_debug("%s()\n", __func__);
	rate->min = rate->max = 48000;

	return 0;
}

static int msm_mi2s_snd_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params)
{
	pr_debug("%s(): substream = %s  stream = %d\n", __func__,
		 substream->name, substream->stream);
	return 0;
}

static int mi2s_clk_ctl(struct snd_pcm_substream *substream, bool enable)
{
	int ret = 0;
	if (enable) {
		digital_cdc_clk.clk_val = 9600000;
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			mi2s_rx_clk.clk_val2 = Q6AFE_LPASS_OSR_CLK_12_P288_MHZ;
			mi2s_rx_clk.clk_val1 = Q6AFE_LPASS_IBIT_CLK_1_P536_MHZ;
			ret = afe_set_lpass_clock(AFE_PORT_ID_SECONDARY_MI2S_RX,
						  &mi2s_rx_clk);
		} else if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
			mi2s_tx_clk.clk_val2 = Q6AFE_LPASS_OSR_CLK_12_P288_MHZ;
			mi2s_tx_clk.clk_val1 = Q6AFE_LPASS_IBIT_CLK_1_P536_MHZ;
			ret = afe_set_lpass_clock(AFE_PORT_ID_PRIMARY_MI2S_RX,
						  &mi2s_tx_clk);
		} else
			pr_err("%s:Not valid substream.\n", __func__);

		if (ret < 0)
			pr_err("%s:afe_set_lpass_clock failed\n", __func__);

	} else {
		digital_cdc_clk.clk_val = 0;
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			mi2s_rx_clk.clk_val2 = Q6AFE_LPASS_OSR_CLK_DISABLE;
			mi2s_rx_clk.clk_val1 = Q6AFE_LPASS_IBIT_CLK_DISABLE;
			ret = afe_set_lpass_clock(AFE_PORT_ID_SECONDARY_MI2S_RX,
						  &mi2s_rx_clk);
		} else if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
			mi2s_tx_clk.clk_val2 = Q6AFE_LPASS_OSR_CLK_DISABLE;
			mi2s_tx_clk.clk_val1 = Q6AFE_LPASS_IBIT_CLK_DISABLE;
			ret = afe_set_lpass_clock(AFE_PORT_ID_PRIMARY_MI2S_RX,
						  &mi2s_tx_clk);
		} else
			pr_err("%s:Not valid substream.\n", __func__);

		if (ret < 0)
			pr_err("%s:afe_set_lpass_clock failed\n", __func__);

	}
	ret = msm_config_mclk(AFE_PORT_ID_SECONDARY_MI2S_RX, &digital_cdc_clk);
	return ret;
}

static int msm8x10_enable_codec_ext_clk(struct snd_soc_codec *codec,
					int enable, bool dapm)
{
	int ret = 0;

	pr_err("%s: enable = %d  codec name %s enable %x\n",
		   __func__, enable, codec->name, enable);
	if (enable) {
		digital_cdc_clk.clk_val = 9600000;
		msm_config_mi2s_clk(1);
		ret = msm_config_mclk(AFE_PORT_ID_SECONDARY_MI2S_RX,
					   &digital_cdc_clk);
		msm8x10_wcd_mclk_enable(codec, 1, dapm);
	} else {
		//digital_cdc_clk.clk_val = 0;
		msm8x10_wcd_mclk_enable(codec, 0, dapm);
		ret = msm_config_mclk(AFE_PORT_ID_SECONDARY_MI2S_RX,
					   &digital_cdc_clk);
		msm_config_mi2s_clk(0);
	}
	return ret;
}

static int msm8x10_mclk_event(struct snd_soc_dapm_widget *w,
			      struct snd_kcontrol *kcontrol, int event)
{
	pr_err("%s: event = %d\n", __func__, event);
	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		return msm8x10_enable_codec_ext_clk(w->codec, 1, true);
	case SND_SOC_DAPM_POST_PMD:
		return msm8x10_enable_codec_ext_clk(w->codec, 0, true);
	}
	return 0;
}

static void msm_mi2s_snd_shutdown(struct snd_pcm_substream *substream)
{
	int ret;

	pr_debug("%s(): substream = %s  stream = %d\n", __func__,
		 substream->name, substream->stream);
	ret = mi2s_clk_ctl(substream, false);
	if (ret < 0)
		pr_err("%s:clock disable failed\n", __func__);
}

static int msm_mi2s_snd_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int ret = 0;

	pr_debug("%s(): substream = %s  stream = %d\n", __func__,
		 substream->name, substream->stream);

	ret = mi2s_clk_ctl(substream, true);

	ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_CBS_CFS);
	if (ret < 0)
		pr_err("set fmt cpu dai failed\n");

	return ret;
}

static const struct snd_soc_dapm_route msm8x10_common_audio_map[] = {
	{"RX_BIAS", NULL, "MCLK"},
	{"INT_LDO_H", NULL, "MCLK"},
	{"MIC BIAS External", NULL, "Handset Mic"},
	{"MIC BIAS Internal2", NULL, "Headset Mic"},
	{"AMIC1", NULL, "MIC BIAS External"},
	{"AMIC2", NULL, "MIC BIAS Internal2"},

};

static const struct snd_soc_dapm_widget msm8x10_dapm_widgets[] = {

	SND_SOC_DAPM_SUPPLY("MCLK",  SND_SOC_NOPM, 0, 0,
	msm8x10_mclk_event, SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MIC("Handset Mic", NULL),
	SND_SOC_DAPM_MIC("Headset Mic", NULL),

};

static int msm_audrx_init(struct snd_soc_pcm_runtime *rtd)
{

	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int ret = 0;
	pr_err("%s(),dev_name%s\n", __func__, dev_name(cpu_dai->dev));

	pr_debug("%s(): aud_init_rsc_ref counter = %d\n",
		__func__, atomic_read(&aud_init_rsc_ref));
	if (atomic_inc_return(&aud_init_rsc_ref) != 1)
		goto exit;


	snd_soc_dapm_new_controls(dapm, msm8x10_dapm_widgets,
				ARRAY_SIZE(msm8x10_dapm_widgets));

	snd_soc_dapm_add_routes(dapm, msm8x10_common_audio_map,
		ARRAY_SIZE(msm8x10_common_audio_map));

	snd_soc_dapm_sync(dapm);
	ret =  msm_enable_mclk_root(AFE_PORT_ID_SECONDARY_MI2S_RX,
				    &digital_cdc_clk);
exit:
	return ret;
}

static struct snd_soc_ops msm8x10_mi2s_be_ops = {
	.startup = msm_mi2s_snd_startup,
	.hw_params = msm_mi2s_snd_hw_params,
	.shutdown = msm_mi2s_snd_shutdown,
};

/* Digital audio interface glue - connects codec <---> CPU */
static struct snd_soc_dai_link msm8x10_dai[] = {
	/* FrontEnd DAI Links */
	{
		.name = "MSM8X10 Media1",
		.stream_name = "MultiMedia1",
		.cpu_dai_name	= "MultiMedia1",
		.platform_name  = "msm-pcm-dsp.0",
		.dynamic = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.ignore_suspend = 1,
		/* this dainlink has playback support */
		.ignore_pmdown_time = 1,
		.be_id = MSM_FRONTEND_DAI_MULTIMEDIA1
	},
	{
		.name = "MSM8X10 Media2",
		.stream_name = "MultiMedia2",
		.cpu_dai_name   = "MultiMedia2",
		.platform_name  = "msm-pcm-dsp.0",
		.dynamic = 1,
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.ignore_suspend = 1,
		/* this dainlink has playback support */
		.ignore_pmdown_time = 1,
		.be_id = MSM_FRONTEND_DAI_MULTIMEDIA2,
	},
	/* Backend I2S DAI Links */
	{
		.name = LPASS_BE_SEC_MI2S_RX,
		.stream_name = "Secondary MI2S Playback",
		.cpu_dai_name = "msm-dai-q6-mi2s.1",
		.platform_name = "msm-pcm-routing",
		.codec_name     = "msm8x10-wcd-i2c-core.5-000d",
		.codec_dai_name = "msm8x10_wcd_i2s_rx1",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_SECONDARY_MI2S_RX,
		.init = &msm_audrx_init,
		.be_hw_params_fixup = msm_be_hw_params_fixup,
		.ops = &msm8x10_mi2s_be_ops,
		.ignore_suspend = 1,
	},
	{
		.name = LPASS_BE_PRI_MI2S_TX,
		.stream_name = "Primary MI2S Capture",
		.cpu_dai_name = "msm-dai-q6-mi2s.0",
		.platform_name = "msm-pcm-routing",
		.codec_name     = "msm8x10-wcd-i2c-core.5-000d",
		.codec_dai_name = "msm8x10_wcd_i2s_tx1",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_PRI_MI2S_TX,
		.init = &msm_audrx_init,
		.be_hw_params_fixup = msm_be_hw_params_fixup,
		.ops = &msm8x10_mi2s_be_ops,
		.ignore_suspend = 1,
	},
};

struct snd_soc_card snd_soc_card_msm8x10 = {
	.name		= "msm8x10-snd-card",
	.dai_link	= msm8x10_dai,
	.num_links	= ARRAY_SIZE(msm8x10_dai),
};


static __devinit int msm8x10_asoc_machine_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &snd_soc_card_msm8x10;
	int ret;

	dev_dbg(&pdev->dev, "%s\n", __func__);
	if (!pdev->dev.of_node) {
		dev_err(&pdev->dev, "No platform supplied from device tree\n");
		return -EINVAL;
	}

	card->dev = &pdev->dev;
	platform_set_drvdata(pdev, card);


	ret = snd_soc_register_card(card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n",
			ret);
		goto err;
	}

	atomic_set(&aud_init_rsc_ref, 0);

	return 0;
err:
	return ret;
}

static int __devexit msm8x10_asoc_machine_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);

	snd_soc_unregister_card(card);

	return 0;
}

static const struct of_device_id msm8x10_asoc_machine_of_match[]  = {
	{ .compatible = "qcom,msm8x10-audio-codec", },
	{},
};

static struct platform_driver msm8x10_asoc_machine_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.pm = &snd_soc_pm_ops,
		.of_match_table = msm8x10_asoc_machine_of_match,
	},
	.probe = msm8x10_asoc_machine_probe,
	.remove = __devexit_p(msm8x10_asoc_machine_remove),
};
module_platform_driver(msm8x10_asoc_machine_driver);

MODULE_DESCRIPTION("ALSA SoC msm");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRV_NAME);
MODULE_DEVICE_TABLE(of, msm8x10_asoc_machine_of_match);
