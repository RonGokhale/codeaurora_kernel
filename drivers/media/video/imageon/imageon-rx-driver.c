/*
 * Driver for the IMAGEON-FMC board
 *
 * Copyright 2012-2013 Analog Devices Inc.
 *  Author: Lars-Peter Clausen <lars@metafoo.de>
 *
 * based on cobalt-driver.c
 *
 * Licensed under the GPL-2.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/of_i2c.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/dmaengine.h>
#include <linux/firmware.h>
#include <linux/platform_device.h>

#include <media/adv7604.h>
#include <media/videobuf2-dma-contig.h>
#include <media/v4l2-event.h>

#include "imageon-rx-driver.h"
#include "imageon-rx-ioctl.h"

static void imageon_rx_notify(struct v4l2_subdev *sd, unsigned int notification,
	void *arg)
{
	struct imageon_rx *imageon_rx = to_imageon_rx(sd->v4l2_dev);
	long hotplug = (long)arg;

	switch (notification) {
	case ADV7604_HOTPLUG:
		gpio_set_value_cansleep(imageon_rx->hotplug_gpio, hotplug);
		break;
	default:
		break;
	}
}

static irqreturn_t imageon_rx_irq_handler(int irq, void *dev_id)
{
	struct imageon_rx *imageon_rx = dev_id;

	v4l2_subdev_call(imageon_rx->stream.sd_adv7611, core,
				interrupt_service_routine, 0, NULL);

	return IRQ_HANDLED;
}

static int imageon_rx_subdevs_init(struct imageon_rx *imageon_rx)
{
	static struct adv7604_platform_data adv7611_pdata = {
		.disable_pwrdnb = 1,
		.op_ch_sel = ADV7604_OP_CH_SEL_RGB,
		.blank_data = 1,
		.op_656_range = 1,
		.rgb_out = 0,
		.alt_data_sat = 1,
		.op_format_sel = ADV7604_OP_FORMAT_SEL_SDR_ITU656_16,
		.int1_config = ADV7604_INT1_CONFIG_OPEN_DRAIN,
		.connector_hdmi = 1,
		.insert_av_codes = 1,
		.i2c_cec = 0x40,
		.i2c_infoframe = 0x3e,
		.i2c_afe = 0x26,
		.i2c_repeater = 0x32,
		.i2c_edid = 0x36,
		.i2c_hdmi = 0x34,
		.i2c_cp = 0x22,
	};
	static struct i2c_board_info adv7611_info = {
		.type = "adv7611",
		.addr = 0x4c,
		.platform_data = &adv7611_pdata,
	};
	struct v4l2_subdev_edid edid = {
		.pad = 0,
		.start_block = 0,
		.blocks = 1,
		.edid = imageon_rx->edid_data,
	};

	struct imageon_rx_stream *s = &imageon_rx->stream;

	s->sd_adv7611 = v4l2_i2c_new_subdev_board(&imageon_rx->v4l2_dev,
		s->i2c_adap, &adv7611_info, NULL);
	if (!s->sd_adv7611)
		return -ENODEV;

	return v4l2_subdev_call(s->sd_adv7611, pad, set_edid, &edid);
}

struct xlnx_pcm_dma_params {
       struct device_node *of_node;
       int chan_id;
};

static bool xlnx_pcm_filter(struct dma_chan *chan, void *param)
{
       struct xlnx_pcm_dma_params *p = param;

       return chan->device->dev->of_node == p->of_node &&
               chan->chan_id == p->chan_id;
}

static int imageon_rx_load_edid(struct platform_device *pdev,
	struct imageon_rx *imageon_rx)
{
	const struct firmware *fw;
	int ret;

	ret = request_firmware(&fw, "imageon_edid.bin", &pdev->dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to load firmware: %d\n", ret);
		return ret;
	}

	if (fw->size > 256) {
		dev_err(&pdev->dev, "EDID firmware data too large.\n");
		release_firmware(fw);
		return -EINVAL;
	}

	memcpy(imageon_rx->edid_data, fw->data, fw->size);

	release_firmware(fw);

	return 0;
}

static int imageon_rx_probe(struct platform_device *pdev)
{
	struct xlnx_pcm_dma_params dma_params;
	struct device_node *of_node;
	struct i2c_adapter *adap;
	struct imageon_rx *imageon_rx;
	struct of_phandle_args dma_spec;
	struct resource *res;
	dma_cap_mask_t mask;
	int irq;
	int ret;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return -ENXIO;

	of_node = of_parse_phandle(pdev->dev.of_node, "slave_adapter", 0);
	if (!of_node)
		return -ENXIO;

	adap = of_find_i2c_adapter_by_node(of_node);
	of_node_put(of_node);
	if (!adap)
		return -EPROBE_DEFER;

	ret = of_parse_phandle_with_args(pdev->dev.of_node,
		"dma-request", "#dma-cells", 0, &dma_spec);
	if (ret)
		return ret;

	dma_params.of_node = dma_spec.np;
	dma_params.chan_id = dma_spec.args[0];

	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE, mask);
	dma_cap_set(DMA_PRIVATE, mask);

	imageon_rx = devm_kzalloc(&pdev->dev, sizeof(struct imageon_rx), GFP_KERNEL);
	if (imageon_rx == NULL) {
		dev_err(&pdev->dev, "Failed to allocate device\n");
		ret = -ENOMEM;
		goto err_i2c_put_adapter;
	}

	imageon_rx->hotplug_gpio = of_get_gpio(pdev->dev.of_node, 0);
	if (!gpio_is_valid(imageon_rx->hotplug_gpio)) {
		return imageon_rx->hotplug_gpio;
	}
	
	ret = devm_gpio_request_one(&pdev->dev, 
		imageon_rx->hotplug_gpio, GPIOF_OUT_INIT_LOW, "HPD");
	if (ret < 0)
		return ret;

	imageon_rx->stream.chan = dma_request_channel(mask, xlnx_pcm_filter, &dma_params);
	if (!imageon_rx->stream.chan) {
		ret = -EPROBE_DEFER;
		goto err_i2c_put_adapter;
	}

	imageon_rx->stream.i2c_adap = adap;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	imageon_rx->base = devm_request_and_ioremap(&pdev->dev, res);
	if (!imageon_rx->base) {
		ret = -ENXIO;
		goto err_i2c_put_adapter;
	}

	ret = imageon_rx_load_edid(pdev, imageon_rx);
	if (ret)
		goto err_i2c_put_adapter;

	imageon_rx->alloc_ctx = vb2_dma_contig_init_ctx(&pdev->dev);
	if (IS_ERR(imageon_rx->alloc_ctx)) {
		ret = PTR_ERR(imageon_rx->alloc_ctx);
		dev_err(&pdev->dev, "Failed to init dma ctx: %d\n", ret);
		goto err_i2c_put_adapter;
	}

	ret = v4l2_device_register(&pdev->dev, &imageon_rx->v4l2_dev);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register card: %d\n", ret);
		goto err_dma_cleanup_ctx;
	}
	snprintf(imageon_rx->v4l2_dev.name, sizeof(imageon_rx->v4l2_dev.name),
		"imageon_rx");
	imageon_rx->v4l2_dev.notify = imageon_rx_notify;

	imageon_rx->mdev.dev = &pdev->dev;
	strlcpy(imageon_rx->mdev.model, imageon_rx->v4l2_dev.name,
		sizeof(imageon_rx->mdev.model));
	snprintf(imageon_rx->mdev.bus_info, sizeof(imageon_rx->mdev.bus_info),
		"Platform");
	imageon_rx->mdev.hw_revision = 0;
	imageon_rx->mdev.driver_version = 0;
	imageon_rx->v4l2_dev.mdev = &imageon_rx->mdev;
	if (media_device_register(&imageon_rx->mdev) < 0)
		goto err_device_unregister;

	ret = imageon_rx_subdevs_init(imageon_rx);
	if (ret)
		goto err_media_device_unregister;

	ret = v4l2_device_register_subdev_nodes(&imageon_rx->v4l2_dev);
	if (ret)
		goto err_media_device_unregister;

	ret = imageon_rx_nodes_register(imageon_rx);
	if (ret) {
		dev_err(&pdev->dev, "Error %d registering device nodes\n", ret);
		goto err_media_device_unregister;
	}

#if 0
	ret = request_threaded_irq(irq, NULL, imageon_rx_irq_handler,
		IRQF_ONESHOT | IRQF_TRIGGER_HIGH, dev_name(&pdev->dev), imageon_rx);
	if (ret < 0)
		goto err_media_device_unregister;
#endif

	video_set_drvdata(&imageon_rx->stream.vdev, imageon_rx);

	return 0;

err_media_device_unregister:
	media_device_unregister(&imageon_rx->mdev);
err_device_unregister:
	v4l2_device_unregister(&imageon_rx->v4l2_dev);
err_dma_cleanup_ctx:
	vb2_dma_contig_cleanup_ctx(imageon_rx->alloc_ctx);
err_i2c_put_adapter:
	i2c_put_adapter(adap);
	return ret;
}

static int imageon_rx_remove(struct platform_device *pdev)
{
	struct imageon_rx *imageon_rx = platform_get_drvdata(pdev);

	free_irq(platform_get_irq(pdev, 0), imageon_rx);

	media_device_unregister(&imageon_rx->mdev);
	v4l2_device_unregister(&imageon_rx->v4l2_dev);
	vb2_dma_contig_cleanup_ctx(imageon_rx->alloc_ctx);
	i2c_put_adapter(imageon_rx->stream.i2c_adap);

	return 0;
}

static const struct of_device_id imageon_rx_of_match[] = {
	{ .compatible = "adi,imageon-rx", },
	{},
};
MODULE_DEVICE_TABLE(of, imageon_rx_of_match);

static struct platform_driver imageon_rx_driver = {
	.driver = {
		.name = "imageon-rx",
		.owner = THIS_MODULE,
		.of_match_table = imageon_rx_of_match,
	},
	.probe = imageon_rx_probe,
	.remove = imageon_rx_remove,
};
module_platform_driver(imageon_rx_driver);
