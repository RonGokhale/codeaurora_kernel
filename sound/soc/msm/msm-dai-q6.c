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
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/mfd/wcd9310/core.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/apr_audio.h>
#include <sound/q6afe.h>
#include <sound/q6adm.h>

static int msm_dai_q6_cdc_hw_params(struct snd_pcm_hw_params *params,
				    struct snd_soc_dai *dai, int stream)
{
	union afe_port_config port_config;
	int sample_rate, channels;
	int rc;

	channels = params_channels(params);
	switch (channels) {
	case 2:
		port_config.mi2s.channel = MSM_AFE_STEREO;
		break;
	case 1:
	default:
		port_config.mi2s.channel = MSM_AFE_MONO;
		break;
	}
	sample_rate = params_rate(params);

	dev_dbg(dai->dev, " channel %d sample rate %d entered\n",
	channels, sample_rate);

	port_config.mi2s.bitwidth = 16; /* Q6 only supports 16 as now */
	port_config.mi2s.line = 1;
	port_config.mi2s.ws = 1; /* I2S master mode for now */

	rc = afe_open(dai->id, &port_config, sample_rate);

	if (IS_ERR_VALUE(rc)) {
		dev_err(dai->dev, "fail to open AFE port\n");
		goto failed_cmd;
	}
	if (stream == SNDRV_PCM_STREAM_PLAYBACK)
		rc = adm_open_mixer(dai->id, ADM_PATH_PLAYBACK, sample_rate,
		channels, DEFAULT_COPP_TOPOLOGY);
	else
		rc = adm_open_mixer(dai->id, ADM_PATH_LIVE_REC, sample_rate,
		channels, DEFAULT_COPP_TOPOLOGY);

	if (IS_ERR_VALUE(rc)) {
		dev_err(dai->dev, "fail to open AFE port\n");
		goto failed_cmd;
	}

	return 0;

failed_cmd:
	return rc;
}

static int msm_dai_q6_hdmi_hw_params(struct snd_pcm_hw_params *params,
	struct snd_soc_dai *dai)
{
	union afe_port_config port_config;
	int sample_rate;
	int rc;

	dev_dbg(dai->dev, "%s start HDMI port\n", __func__);

	switch (params_channels(params)) {
	case 2:
		port_config.hdmi.channel_mode = 0; /* Put in macro */
		break;
	default:
		return -EINVAL;
		break;
	}

	port_config.hdmi.bitwidth = 16; /* Q6 only supports 16 as now */
	port_config.hdmi.data_type = 0;
	sample_rate = params_rate(params);

	rc = afe_open(dai->id, &port_config, sample_rate);

	if (IS_ERR_VALUE(rc)) {
		dev_err(dai->dev, "fail to open AFE port\n");
		goto failed_cmd;
	}

	rc = adm_open_mixer(dai->id, ADM_PATH_PLAYBACK,
		sample_rate, 2, DEFAULT_COPP_TOPOLOGY);

	if (IS_ERR_VALUE(rc)) {
		dev_err(dai->dev, "fail to open AFE port\n");
		afe_close(dai->id);
		goto failed_cmd;
	}

	return 0;

failed_cmd:
	return rc;
}

static int msm_dai_q6_slim_bus_hw_params(struct snd_pcm_hw_params *params,
				    struct snd_soc_dai *dai, int stream)
{
	union afe_port_config port_config;
	int sample_rate, channels;
	u8 pgd_la, inf_la;
	int rc;

	channels = params_channels(params);
	sample_rate = params_rate(params);
	tabla_get_logical_addresses(&pgd_la, &inf_la);

	port_config.slimbus.slimbus_dev_id =  AFE_SLIMBUS_DEVICE_1;
	port_config.slimbus.slave_dev_pgd_la = pgd_la;
	port_config.slimbus.slave_dev_intfdev_la = inf_la;
	port_config.slimbus.bit_width = 16; /* Q6 only supports 16 as now */
	port_config.slimbus.data_format = 0;
	port_config.slimbus.num_channels = channels;

	memset(port_config.slimbus.slave_port_mapping, 0,
			sizeof(port_config.slimbus.slave_port_mapping));

	switch (channels) {
	case 2:
		if (dai->id == SLIMBUS_0_RX) {
			port_config.slimbus.slave_port_mapping[0] = 1;
			port_config.slimbus.slave_port_mapping[1] = 2;
		} else {
			port_config.slimbus.slave_port_mapping[0] = 5;
			port_config.slimbus.slave_port_mapping[1] = 6;
		}
		break;
	case 1:
	default:
		if (dai->id == SLIMBUS_0_RX)
			port_config.slimbus.slave_port_mapping[0] = 1;
		else
			port_config.slimbus.slave_port_mapping[0] = 6;

		break;
	}

	port_config.slimbus.reserved = 0;

	dev_dbg(dai->dev, "slimbus_dev_id  %hu  slave_dev_pgd_la 0x%hx\n"
		"slave_dev_intfdev_la 0x%hx   bit_width %hu   data_format %hu\n"
		"num_channel %hu  slave_port_mapping[0]  %hu\n"
		"slave_port_mapping[1]  %hu slave_port_mapping[2]  %hu\n"
		"sample_rate %d\n", port_config.slimbus.slimbus_dev_id,
		port_config.slimbus.slave_dev_pgd_la,
		port_config.slimbus.slave_dev_intfdev_la,
		port_config.slimbus.bit_width, port_config.slimbus.data_format,
		port_config.slimbus.num_channels,
		port_config.slimbus.slave_port_mapping[0],
		port_config.slimbus.slave_port_mapping[1],
		port_config.slimbus.slave_port_mapping[2],
		sample_rate);

	rc = afe_open(dai->id, &port_config, sample_rate);

	if (IS_ERR_VALUE(rc)) {
		dev_err(dai->dev, "fail to open AFE port\n");
		goto failed_cmd;
	}
	if (stream == SNDRV_PCM_STREAM_PLAYBACK)
		rc = adm_open_mixer(dai->id, ADM_PATH_PLAYBACK, sample_rate,
		channels, DEFAULT_COPP_TOPOLOGY);
	else
		rc = adm_open_mixer(dai->id, ADM_PATH_LIVE_REC, sample_rate,
		channels, DEFAULT_COPP_TOPOLOGY);

	if (IS_ERR_VALUE(rc)) {
		dev_err(dai->dev, "fail to open AFE port\n");
		goto failed_cmd;
	}

	return 0;

failed_cmd:
	return rc;
}



/* Current implementation assumes hw_param is called once
 * This may not be the case but what to do when ADM and AFE
 * port are already opened and parameter changes
 */
static int msm_dai_q6_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	int rc = 0;

	switch (dai->id) {
	case PRIMARY_I2S_TX:
	case PRIMARY_I2S_RX:
		rc = msm_dai_q6_cdc_hw_params(params, dai, substream->stream);
		break;
	case HDMI_RX:
		rc = msm_dai_q6_hdmi_hw_params(params, dai);
		break;

	case SLIMBUS_0_RX:
	case SLIMBUS_0_TX:
		rc = msm_dai_q6_slim_bus_hw_params(params, dai,
				substream->stream);
		break;
	default:
		dev_err(dai->dev, "invalid AFE port ID\n");
		rc = -EINVAL;
		break;
	}

	return rc;
}

static int msm_dai_q6_hw_free(struct snd_pcm_substream *substream,
				 struct snd_soc_dai *dai)
{
	int rc;

	rc = adm_close(dai->id);

	if (IS_ERR_VALUE(rc))
		dev_err(dai->dev, "fail to close ADM COPP\n");

	rc = afe_close(dai->id);

	if (IS_ERR_VALUE(rc))
		dev_err(dai->dev, "fail to close AFE port\n");

	return rc;
};

static struct snd_soc_dai_ops msm_dai_q6_ops = {
	/* AFE port are not generic port. There is no run-time
	 * assignment of generic port
	 * startup and shutdown are not necessary
	 * trigger is called in interrupt context
	 * afe_open and adm_start can put process to sleep
	 * workaround is to put afe_open and adm start in
	 * hw_params
	 * DSP only handles 16-bit and support only I2S
	 * master mode for now. leave set_fmt function
	 * unimplemented for now.
	 */
	.hw_params	= msm_dai_q6_hw_params,
	.hw_free	= msm_dai_q6_hw_free,
};

static struct snd_soc_dai_driver msm_dai_q6_i2s_rx_dai = {
	.playback = {
		.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
		SNDRV_PCM_RATE_16000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
		.channels_min = 1,
		.channels_max = 2,
		.rate_min =     8000,
		.rate_max =	48000,
	},
	.ops = &msm_dai_q6_ops,
};

static struct snd_soc_dai_driver msm_dai_q6_i2s_tx_dai = {
	.capture = {
		.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
		SNDRV_PCM_RATE_16000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
		.channels_min = 1,
		.channels_max = 2,
		.rate_min =     8000,
		.rate_max =	48000,
	},
	.ops = &msm_dai_q6_ops,
};

static struct snd_soc_dai_driver msm_dai_q6_hdmi_rx_dai = {
	.playback = {
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
		.channels_min = 2,
		.channels_max = 2,
		.rate_max =     48000,
		.rate_min =	48000,
	},
	.ops = &msm_dai_q6_ops,
};


static struct snd_soc_dai_driver msm_dai_q6_slimbus_rx_dai = {
	.playback = {
		.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
		SNDRV_PCM_RATE_16000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
		.channels_min = 1,
		.channels_max = 2,
		.rate_min =     8000,
		.rate_max =	48000,
	},
	.ops = &msm_dai_q6_ops,
};

static struct snd_soc_dai_driver msm_dai_q6_slimbus_tx_dai = {
	.capture = {
		.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
		SNDRV_PCM_RATE_16000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
		.channels_min = 1,
		.channels_max = 2,
		.rate_min =     8000,
		.rate_max =	48000,
	},
	.ops = &msm_dai_q6_ops,
};


/* To do: change to register DAIs as batch */
static __devinit int msm_dai_q6_dev_probe(struct platform_device *pdev)
{
	int rc = 0;

	dev_dbg(&pdev->dev, "dev name %s\n", dev_name(&pdev->dev));

	switch (pdev->id) {
	case PRIMARY_I2S_RX:
		rc = snd_soc_register_dai(&pdev->dev, &msm_dai_q6_i2s_rx_dai);
		break;
	case PRIMARY_I2S_TX:
		rc = snd_soc_register_dai(&pdev->dev, &msm_dai_q6_i2s_tx_dai);
		break;
	case HDMI_RX:
		rc = snd_soc_register_dai(&pdev->dev, &msm_dai_q6_hdmi_rx_dai);
		break;
	case SLIMBUS_0_RX:
		rc = snd_soc_register_dai(&pdev->dev,
				&msm_dai_q6_slimbus_rx_dai);
		break;
	case SLIMBUS_0_TX:
		rc = snd_soc_register_dai(&pdev->dev,
				&msm_dai_q6_slimbus_tx_dai);
		break;
	default:
		rc = -ENODEV;
		break;
	}
	return rc;
}

static __devexit int msm_dai_q6_dev_remove(struct platform_device *pdev)
{
	snd_soc_unregister_dai(&pdev->dev);
	return 0;
}

static struct platform_driver msm_dai_q6_driver = {
	.probe  = msm_dai_q6_dev_probe,
	.remove = msm_dai_q6_dev_remove,
	.driver = {
		.name = "msm-dai-q6",
		.owner = THIS_MODULE,
	},
};

static int __init msm_dai_q6_init(void)
{
	return platform_driver_register(&msm_dai_q6_driver);
}
module_init(msm_dai_q6_init);

static void __exit msm_dai_q6_exit(void)
{
	platform_driver_unregister(&msm_dai_q6_driver);
}
module_exit(msm_dai_q6_exit);

/* Module information */
MODULE_DESCRIPTION("MSM DSP DAI driver");
MODULE_LICENSE("GPL v2");
