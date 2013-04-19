/*
 *  Copyright (C) 2012, Analog Devices Inc.
 *	Author: Lars-Peter Clausen <lars@metafoo.de>
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under  the terms of the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the License, or (at your
 *  option) any later version.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/dmaengine_pcm.h>
#include <linux/dma-mapping.h>

#include <linux/dmaengine.h>

static const struct snd_pcm_hardware xlnx_pcm_hardware = {
	.info = SNDRV_PCM_INFO_MMAP |
		SNDRV_PCM_INFO_MMAP_VALID |
		SNDRV_PCM_INFO_INTERLEAVED |
		SNDRV_PCM_INFO_BLOCK_TRANSFER,
	.period_bytes_min = 1024,
	.period_bytes_max = (1 << 20) - 1,
	.periods_min	  = 2,
	.periods_max	  = UINT_MAX,
	.buffer_bytes_max = ULONG_MAX,
};

static const struct snd_dmaengine_pcm_config xilinx_dmaengine_pcm_config = {
	.pcm_hardware = &xlnx_pcm_hardware,
	.prealloc_buffer_size = 1 * 1024 * 1024,
};

int xlnx_pcm_register(struct device *dev)
{
	return snd_dmaengine_pcm_register(dev, &xilinx_dmaengine_pcm_config, 0);
}
EXPORT_SYMBOL_GPL(xlnx_pcm_register);

void xlnx_pcm_unregister(struct device *dev)
{
	snd_dmaengine_pcm_unregister(dev);
}
EXPORT_SYMBOL_GPL(xlnx_pcm_unregister);

MODULE_AUTHOR("Lars-Peter Clausen <lars@metafoo.de>");
MODULE_DESCRIPTION("Xilinx DMA engine based audio DMA driver");
MODULE_LICENSE("GPL");
