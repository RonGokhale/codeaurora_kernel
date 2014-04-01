/*
 * SiRF I2S driver
 *
 * Copyright (c) 2011 Cambridge Silicon Radio Limited, a CSR plc group company.
 *
 * Licensed under GPLv2 or later.
 */
#include <linux/module.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/pm_runtime.h>
#include <linux/clk.h>
#include <linux/regmap.h>
#include <sound/soc.h>
#include <sound/pcm_params.h>
#include <sound/dmaengine_pcm.h>

#include "sirf-i2s.h"

struct sirf_i2s {
	struct regmap *regmap;
	struct clk *clk;
	u32 i2s_ctrl;
	u32 i2s_ctrl_tx_rx_en;
	bool master;
	bool mclk_out;
	int mclk_rate;
	/* Reference clock. If I2S is master mode or need I2S controller
	 * enable mclk output. It must be set to SIRF_I2S_PWM_REF_CLK or
	 * SIRF_I2S_EXT_REF_CLK. If I2S is salve mode and no need enable
	 * mclk output. It can be set to SIRF_I2S_NO_REF_CLK.
	 */
	int ref_clk;
	int ref_clk_rate;
	struct snd_dmaengine_dai_dma_data playback_dma_data;
	struct snd_dmaengine_dai_dma_data capture_dma_data;
};

static int sirf_i2s_dai_probe(struct snd_soc_dai *dai)
{
	struct sirf_i2s *i2s = snd_soc_dai_get_drvdata(dai);
	snd_soc_dai_init_dma_data(dai, &i2s->playback_dma_data,
			&i2s->capture_dma_data);
	return 0;
}

static void sirf_i2s_tx_enable(struct sirf_i2s *i2s)
{
	/* First start the FIFO, then enable the tx/rx */
	regmap_update_bits(i2s->regmap, AUDIO_CTRL_I2S_TXFIFO_OP,
		AUDIO_FIFO_RESET, AUDIO_FIFO_RESET);
	regmap_update_bits(i2s->regmap, AUDIO_CTRL_I2S_TXFIFO_OP,
		AUDIO_FIFO_RESET, ~AUDIO_FIFO_RESET);
	regmap_update_bits(i2s->regmap, AUDIO_CTRL_I2S_TXFIFO_OP,
		AUDIO_FIFO_START, AUDIO_FIFO_START);
	regmap_update_bits(i2s->regmap, AUDIO_CTRL_I2S_TX_RX_EN,
		I2S_TX_ENABLE | I2S_DOUT_OE,
		I2S_TX_ENABLE | I2S_DOUT_OE);
}

static void sirf_i2s_tx_disable(struct sirf_i2s *i2s)
{
	regmap_update_bits(i2s->regmap, AUDIO_CTRL_I2S_TX_RX_EN,
		I2S_TX_ENABLE, ~I2S_TX_ENABLE);
	/* First disable the tx/rx, then stop the FIFO */
	regmap_write(i2s->regmap, AUDIO_CTRL_I2S_TXFIFO_OP, 0);
}

static void sirf_i2s_rx_enable(struct sirf_i2s *i2s)
{
	/* First start the FIFO, then enable the tx/rx */
	regmap_update_bits(i2s->regmap, AUDIO_CTRL_I2S_RXFIFO_OP,
		AUDIO_FIFO_RESET, AUDIO_FIFO_RESET);
	regmap_update_bits(i2s->regmap, AUDIO_CTRL_I2S_RXFIFO_OP,
		AUDIO_FIFO_RESET, ~AUDIO_FIFO_RESET);
	regmap_update_bits(i2s->regmap, AUDIO_CTRL_I2S_RXFIFO_OP,
		AUDIO_FIFO_START, AUDIO_FIFO_START);
	regmap_update_bits(i2s->regmap, AUDIO_CTRL_I2S_TX_RX_EN,
		I2S_RX_ENABLE, I2S_RX_ENABLE);
}

static void sirf_i2s_rx_disable(struct sirf_i2s *i2s)
{
	regmap_update_bits(i2s->regmap, AUDIO_CTRL_I2S_TX_RX_EN,
		I2S_RX_ENABLE, ~I2S_RX_ENABLE);
	/* First disable the tx/rx, then stop the FIFO */
	regmap_write(i2s->regmap, AUDIO_CTRL_I2S_RXFIFO_OP, 0);
}

static int sirf_i2s_trigger(struct snd_pcm_substream *substream,
		int cmd, struct snd_soc_dai *dai)
{
	struct sirf_i2s *i2s = snd_soc_dai_get_drvdata(dai);
	int playback = (substream->stream == SNDRV_PCM_STREAM_PLAYBACK);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		if (playback)
			sirf_i2s_tx_enable(i2s);
		else
			sirf_i2s_rx_enable(i2s);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		if (playback)
			sirf_i2s_tx_disable(i2s);
		else
			sirf_i2s_rx_disable(i2s);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int sirf_i2s_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	struct sirf_i2s *i2s = snd_soc_dai_get_drvdata(dai);
	u32 i2s_ctrl;
	u32 i2s_tx_rx_ctrl;
	u32 left_len, frame_len;
	int channels = params_channels(params);
	u32 bitclk;
	u32 bclk_div;
	int mclk_div = 0;

	/*
	 * SiRFSoC I2S controller only support 2 and 6 channells output.
	 * I2S_SIX_CHANNELS bit clear: select 2 channels mode.
	 * I2S_SIX_CHANNELS bit set: select 6 channels mode.
	 */
	regmap_read(i2s->regmap, AUDIO_CTRL_I2S_CTRL, &i2s_ctrl);
	regmap_read(i2s->regmap, AUDIO_CTRL_I2S_TX_RX_EN, &i2s_tx_rx_ctrl);
	switch (channels) {
	case 2:
		i2s_ctrl &= ~I2S_SIX_CHANNELS;
		break;
	case 6:
		i2s_ctrl |= I2S_SIX_CHANNELS;
		break;
	default:
		dev_err(dai->dev, "%d channels unsupported\n", channels);
		return -EINVAL;
	}

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S8:
		left_len = 8;
		break;
	case SNDRV_PCM_FORMAT_S16_LE:
		left_len = 16;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		left_len = 24;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		left_len = 32;
		break;
	default:
		dev_err(dai->dev, "Format unsupported\n");
		return -EINVAL;
	}

	frame_len = left_len * 2;
	i2s_ctrl &= ~(I2S_L_CHAN_LEN_MASK | I2S_FRAME_LEN_MASK);
	/* Fill the actual len - 1 */
	i2s_ctrl |= ((frame_len - 1) << I2S_FRAME_LEN_SHIFT)
		| ((left_len - 1) << I2S_L_CHAN_LEN_SHIFT);

	if (i2s->mclk_out || i2s->master) {
		if (i2s->mclk_rate || i2s->ref_clk_rate)
			mclk_div = i2s->ref_clk_rate / i2s->mclk_rate;

		/*
		 * The mclk_div is a minimum of 2, that means MCLK is
		 * a minimum of half of reference clk
		 */
		if (mclk_div < 2)
			mclk_div = 2;
		i2s_ctrl |= (((mclk_div / 2) - 1) << I2S_MCLK_DIV_SHIFT);
	}

	if (i2s->mclk_out)
		i2s_tx_rx_ctrl |= I2S_MCLK_EN;
	else
		i2s_tx_rx_ctrl &= ~I2S_MCLK_EN;

	if (i2s->master) {
		i2s_ctrl &= ~I2S_SLAVE_MODE;
		bitclk = params_rate(params) * frame_len;
		bclk_div = i2s->ref_clk_rate / mclk_div / bitclk;
		i2s_ctrl |= (((bclk_div / 2) - 1) << I2S_BITCLK_DIV_SHIFT);
	} else
		i2s_ctrl |= I2S_SLAVE_MODE;

	if (i2s->ref_clk == SIRF_I2S_EXT_REF_CLK)
		i2s_tx_rx_ctrl |= I2S_REF_CLK_SEL_EXT;
	else if (i2s->ref_clk == SIRF_I2S_PWM_REF_CLK)
		i2s_tx_rx_ctrl &= ~I2S_REF_CLK_SEL_EXT;

	regmap_write(i2s->regmap, AUDIO_CTRL_I2S_CTRL, i2s_ctrl);
	regmap_write(i2s->regmap, AUDIO_CTRL_I2S_TX_RX_EN, i2s_tx_rx_ctrl);
	regmap_update_bits(i2s->regmap, AUDIO_CTRL_MODE_SEL,
		I2S_MODE, I2S_MODE);

	return 0;
}

static int sirf_i2s_set_dai_fmt(struct snd_soc_dai *dai,
		unsigned int fmt)
{
	struct sirf_i2s *i2s = snd_soc_dai_get_drvdata(dai);

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		i2s->master = false;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		i2s->master = true;
		break;
	default:
		return -EINVAL;
	}

	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		break;
	default:
		dev_err(dai->dev, "Only I2S format supported\n");
		return -EINVAL;
	}

	/* clock inversion */
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		break;
	default:
		dev_err(dai->dev, "Only normal bit clock, normal frame clock supported\n");
		return -EINVAL;
	}

	return 0;
}

static int sirf_i2s_set_sysclk(struct snd_soc_dai *dai, int clk_id,
	unsigned int clk_rate, int dir)
{
	struct sirf_i2s *i2s = snd_soc_dai_get_drvdata(dai);

	switch (clk_id) {
	case SIRF_I2S_NO_REF_CLK:
		break;
	case SIRF_I2S_EXT_REF_CLK:
	case SIRF_I2S_PWM_REF_CLK:
		i2s->ref_clk = clk_id;
		i2s->ref_clk_rate = clk_rate;
		break;
	case SIRF_I2S_OUTPUT_MCLK:
		if (dir == SND_SOC_CLOCK_OUT) {
			i2s->mclk_out = true;
			i2s->mclk_rate = clk_rate;
		}
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

struct snd_soc_dai_ops sirfsoc_i2s_dai_ops = {
	.trigger	= sirf_i2s_trigger,
	.hw_params	= sirf_i2s_hw_params,
	.set_fmt	= sirf_i2s_set_dai_fmt,
	.set_sysclk	= sirf_i2s_set_sysclk,
};

static struct snd_soc_dai_driver sirf_i2s_dai = {
	.probe = sirf_i2s_dai_probe,
	.id = 0,
	.playback = {
		.stream_name = "SiRF I2S Playback",
		.channels_min = 2,
		.channels_max = 6,
		.rates = SNDRV_PCM_RATE_8000_96000,
		.formats = SNDRV_PCM_FMTBIT_S8 |
			SNDRV_PCM_FMTBIT_S16_LE |
			SNDRV_PCM_FMTBIT_S24_LE |
			SNDRV_PCM_FMTBIT_S32_LE,
	},
	.capture = {
		.stream_name = "SiRF I2S Capture",
		.channels_min = 2,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_96000,
		.formats = SNDRV_PCM_FMTBIT_S8 |
			SNDRV_PCM_FMTBIT_S16_LE |
			SNDRV_PCM_FMTBIT_S24_LE |
			SNDRV_PCM_FMTBIT_S32_LE,
	},
	.ops = &sirfsoc_i2s_dai_ops,
};

#ifdef CONFIG_PM_RUNTIME
static int sirf_i2s_runtime_suspend(struct device *dev)
{
	struct sirf_i2s *i2s = dev_get_drvdata(dev);
	clk_disable_unprepare(i2s->clk);

	return 0;
}

static int sirf_i2s_runtime_resume(struct device *dev)
{
	struct sirf_i2s *i2s = dev_get_drvdata(dev);
	return clk_prepare_enable(i2s->clk);
}
#endif

#ifdef CONFIG_PM_SLEEP
static int sirf_i2s_suspend(struct device *dev)
{
	struct sirf_i2s *i2s = dev_get_drvdata(dev);

	if (!pm_runtime_status_suspended(dev)) {
		regmap_read(i2s->regmap, AUDIO_CTRL_I2S_CTRL, &i2s->i2s_ctrl);
		regmap_read(i2s->regmap, AUDIO_CTRL_I2S_TX_RX_EN,
			&i2s->i2s_ctrl_tx_rx_en);
		sirf_i2s_runtime_suspend(dev);
	}
	return 0;
}

static int sirf_i2s_resume(struct device *dev)
{
	struct sirf_i2s *i2s = dev_get_drvdata(dev);
	int ret;
	if (!pm_runtime_status_suspended(dev)) {
		ret = sirf_i2s_runtime_resume(dev);
		if (ret)
			return ret;
		regmap_update_bits(i2s->regmap, AUDIO_CTRL_MODE_SEL,
			I2S_MODE, I2S_MODE);
		regmap_write(i2s->regmap, AUDIO_CTRL_I2S_CTRL, i2s->i2s_ctrl);
		regmap_write(i2s->regmap, AUDIO_CTRL_I2S_TX_RX_EN,
			i2s->i2s_ctrl_tx_rx_en);
	}

	return 0;
}
#endif

static const struct snd_soc_component_driver sirf_i2s_component = {
	.name       = "sirf-i2s",
};

static const struct regmap_config sirf_i2s_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.max_register = AUDIO_CTRL_I2S_RXFIFO_INT_MSK,
	.cache_type = REGCACHE_NONE,
};

static int sirf_i2s_probe(struct platform_device *pdev)
{
	int ret;
	struct sirf_i2s *i2s;
	void __iomem *base;
	struct resource *mem_res;

	i2s = devm_kzalloc(&pdev->dev, sizeof(struct sirf_i2s),
			GFP_KERNEL);
	if (!i2s)
		return -ENOMEM;

	mem_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(&pdev->dev, mem_res);
	if (IS_ERR(base))
		return PTR_ERR(base);

	i2s->regmap = devm_regmap_init_mmio(&pdev->dev, base,
			&sirf_i2s_regmap_config);
	if (IS_ERR(i2s->regmap))
		return PTR_ERR(i2s->regmap);

	i2s->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(i2s->clk)) {
		dev_err(&pdev->dev, "Get clock failed.\n");
		ret = PTR_ERR(i2s->clk);
		return ret;
	}

	pm_runtime_enable(&pdev->dev);

	ret = devm_snd_soc_register_component(&pdev->dev, &sirf_i2s_component,
			&sirf_i2s_dai, 1);
	if (ret) {
		dev_err(&pdev->dev, "Register Audio SoC dai failed.\n");
		return ret;
	}

	platform_set_drvdata(pdev, i2s);
	return devm_snd_dmaengine_pcm_register(&pdev->dev, NULL, 0);
}

static int sirf_i2s_remove(struct platform_device *pdev)
{
	pm_runtime_disable(&pdev->dev);
	return 0;
}

static const struct of_device_id sirf_i2s_of_match[] = {
	{ .compatible = "sirf,prima2-i2s", },
	{}
};
MODULE_DEVICE_TABLE(of, sirf_i2s_of_match);

static const struct dev_pm_ops sirf_i2s_pm_ops = {
	SET_RUNTIME_PM_OPS(sirf_i2s_runtime_suspend, sirf_i2s_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(sirf_i2s_suspend, sirf_i2s_resume)
};

static struct platform_driver sirf_i2s_driver = {
	.driver = {
		.name = "sirf-i2s",
		.owner = THIS_MODULE,
		.of_match_table = sirf_i2s_of_match,
		.pm = &sirf_i2s_pm_ops,
	},
	.probe = sirf_i2s_probe,
	.remove = sirf_i2s_remove,
};

module_platform_driver(sirf_i2s_driver);

MODULE_DESCRIPTION("SiRF SoC I2S driver");
MODULE_AUTHOR("RongJun Ying <Rongjun.Ying@csr.com>");
MODULE_LICENSE("GPL v2");
