/*
 * Load firmware files from Analog Devices SigmaStudio
 *
 * Copyright 2009-2011 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#ifndef __SIGMA_FIRMWARE_H__
#define __SIGMA_FIRMWARE_H__

#include <linux/device.h>
#include <linux/regmap.h>
#include <linux/list.h>

struct sigmadsp;
struct snd_soc_codec;

struct sigmadsp_ops {
	int (*safeload)(struct sigmadsp *sigmadsp, unsigned int addr,
			const uint8_t *data, size_t len);
};

typedef int (*sigma_fw_write_fn)(void *, unsigned int, const uint8_t *, size_t);

struct sigmadsp {
	const struct sigmadsp_ops *ops;

	struct list_head ctrl_list;

	void *control_data;
	int (*write)(void *, unsigned int, const uint8_t *, size_t);
	int (*read)(void *, unsigned int, uint8_t *, size_t);
};

struct i2c_client;

void sigmadsp_init_regmap(struct sigmadsp *sigmadsp,
	const struct sigmadsp_ops *ops,	struct regmap *regmap);
void sigmadsp_init_i2c(struct sigmadsp *sigmadsp,
	const struct sigmadsp_ops *ops,	struct i2c_client *client);

int sigmadsp_firmware_load(struct snd_soc_codec *codec,
	struct sigmadsp *sigmadsp, const char *name,
	unsigned int sample_rate);
void sigmadsp_firmware_release(struct snd_soc_codec *codec,
	struct sigmadsp *sigmadsp);

#endif
