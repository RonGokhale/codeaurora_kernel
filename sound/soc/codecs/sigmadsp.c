/*
 * Load Analog Devices SigmaStudio firmware files
 *
 * Copyright 2009-2013 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#include <linux/crc32.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/module.h>
#include <linux/slab.h>

#include <sound/control.h>
#include <sound/soc.h>

#include <asm/unaligned.h>

#include "sigmadsp.h"

#define SIGMA_MAGIC "ADISIGM"

#define SIGMA_FW_CHUNK_TYPE_DATA 0
#define SIGMA_FW_CHUNK_TYPE_CONTROL 1

struct sigmadsp_control {
	struct list_head head;
	unsigned int addr;
	unsigned int num_bytes;
	struct snd_kcontrol *kcontrol;
	bool cached;
	u8 cache[];
};

struct sigma_fw_chunk {
	__le32 length;
	__le32 tag;
	__le32 samplerates;
} __packed;

struct sigma_fw_chunk_data {
	struct sigma_fw_chunk chunk;
	__be16 addr;
	uint8_t data[];
} __packed;

struct sigma_fw_chunk_control {
	struct sigma_fw_chunk chunk;
	__le16 type;
	__le16 addr;
	__le16 num_bytes;
	const char name[];
} __packed;

struct sigma_firmware_header {
	unsigned char magic[7];
	u8 version;
	__le32 crc;
} __packed;

enum {
	SIGMA_ACTION_WRITEXBYTES = 0,
	SIGMA_ACTION_WRITESINGLE,
	SIGMA_ACTION_WRITESAFELOAD,
	SIGMA_ACTION_DELAY,
	SIGMA_ACTION_PLLWAIT,
	SIGMA_ACTION_NOOP,
	SIGMA_ACTION_END,
};

struct sigma_action {
	u8 instr;
	u8 len_hi;
	__le16 len;
	__be16 addr;
	unsigned char payload[];
} __packed;

static int sigmadsp_write(struct sigmadsp *sigmadsp, unsigned int addr,
	const uint8_t data[], size_t len)
{
	return sigmadsp->write(sigmadsp->control_data, addr, data, len);
}

static int sigmadsp_read(struct sigmadsp *sigmadsp, unsigned int addr,
	uint8_t data[], size_t len)
{
	return sigmadsp->read(sigmadsp->control_data, addr, data, len);
}

static int sigma_fw_ctrl_info(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_info *info)
{
	struct sigmadsp_control *ctrl = (void *)kcontrol->private_value;

	info->type = SNDRV_CTL_ELEM_TYPE_BYTES;
	info->count = ctrl->num_bytes;

	return 0;
}

static int sigma_fw_ctrl_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct sigmadsp_control *ctrl = (void *)kcontrol->private_value;
	struct sigmadsp *sigmadsp = snd_kcontrol_chip(kcontrol);
	uint8_t *data;
	int ret;

	data = ucontrol->value.bytes.data;

	if (ctrl->num_bytes <= 4 || ctrl->num_bytes > 20 || !sigmadsp->ops)
		ret = sigmadsp_write(sigmadsp, ctrl->addr, data,
			ctrl->num_bytes);
	else
		ret = sigmadsp->ops->safeload(sigmadsp, ctrl->addr, data,
			ctrl->num_bytes);

	if (ret == 0)
		memcpy(ctrl->cache, ucontrol->value.bytes.data, ctrl->num_bytes);

	return ret;
}

static int sigma_fw_ctrl_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct sigmadsp_control *ctrl = (void *)kcontrol->private_value;
	struct sigmadsp *sigmadsp = snd_kcontrol_chip(kcontrol);

	if (!ctrl->cached) {
		sigmadsp_read(sigmadsp, ctrl->addr, ctrl->cache,
			ctrl->num_bytes);
		ctrl->cached = true;
	}

	memcpy(ucontrol->value.bytes.data, ctrl->cache, ctrl->num_bytes);

	return 0;
}

static int sigma_fw_load_data(struct sigmadsp *sigmadsp,
	const struct sigma_fw_chunk *chunk)
{
	const struct sigma_fw_chunk_data *data;

	data = (struct sigma_fw_chunk_data *)chunk;

	return sigmadsp_write(sigmadsp, le16_to_cpu(data->addr), data->data,
		data->chunk.length - sizeof(*data));
}

static void sigma_fw_control_free(struct snd_kcontrol *kcontrol)
{
	struct sigmadsp_control *ctrl = (void *)kcontrol->private_value;

	kfree(ctrl);
}

static int sigma_fw_create_control(struct snd_soc_codec *codec,
	struct sigmadsp *sigmadsp, const struct sigma_fw_chunk *chunk)
{
	const struct sigma_fw_chunk_control *data;
	struct snd_kcontrol_new template;
	struct sigmadsp_control *ctrl;
	struct snd_kcontrol *kcontrol;
	unsigned int num_bytes;
	size_t name_len;
	char *name;
	int ret;

	data = (struct sigma_fw_chunk_control *)chunk;

	name_len = chunk->length - sizeof(*data);
	name = kzalloc(name_len + 1, GFP_KERNEL);
	if (!name)
		return -ENOMEM;
	
	strncpy(name, data->name, name_len);

	num_bytes = le16_to_cpu(data->num_bytes);
	ctrl = kzalloc(sizeof(*ctrl) + num_bytes, GFP_KERNEL);
	if (!ctrl) {
		ret = -ENOMEM;
		goto err_free_name;
	}

	ctrl->addr = le16_to_cpu(data->addr);
	ctrl->num_bytes = num_bytes;

	memset(&template, 0, sizeof(template));
	template.iface = SNDRV_CTL_ELEM_IFACE_MIXER;
	template.name = name;
	template.info = sigma_fw_ctrl_info;
	template.get = sigma_fw_ctrl_get;
	template.put = sigma_fw_ctrl_put;
	template.private_value = (unsigned long)ctrl;

	kcontrol = snd_ctl_new1(&template, sigmadsp);
	if (!kcontrol) {
		ret = -ENOMEM;
		goto err_free_ctrl;
	}

	kcontrol->private_free = sigma_fw_control_free;

	ret = snd_ctl_add(codec->card->snd_card, kcontrol);
	if (ret)
		goto err_free_ctrl;

	ctrl->kcontrol = kcontrol;
	list_add_tail(&ctrl->head, &sigmadsp->ctrl_list);

	kfree(name);

	return 0;

err_free_ctrl:
	kfree(ctrl);
err_free_name:
	kfree(name);

	return ret;
}

static int sigmadsp_fw_load_v2(struct snd_soc_codec *codec,
	struct sigmadsp *sigmadsp, const struct firmware *fw)
{
	struct sigma_fw_chunk *chunk;
	size_t pos;
	int ret;

	pos = sizeof(struct sigma_firmware_header);

	while (pos + sizeof(*chunk) < fw->size) {
		chunk = (struct sigma_fw_chunk *)(fw->data + pos);

		if (chunk->length + pos >= fw->size || chunk->length == 0)
			return -EINVAL;

		switch (chunk->tag) {
		case SIGMA_FW_CHUNK_TYPE_DATA:
			ret = sigma_fw_load_data(sigmadsp, chunk);
			break;
		case SIGMA_FW_CHUNK_TYPE_CONTROL:
			ret = sigma_fw_create_control(codec, sigmadsp, chunk);
			break;
		default:
			dev_warn(codec->dev, "Unkown chunk type: %d\n",
				chunk->tag);
			break;
		}

		if (ret)
			return ret;

		pos += ALIGN(chunk->length, sizeof(__le32));
	}

	return 0;
}

static inline u32 sigma_action_len(struct sigma_action *sa)
{
	return (sa->len_hi << 16) | le16_to_cpu(sa->len);
}

static size_t sigma_action_size(struct sigma_action *sa)
{
	size_t payload = 0;

	switch (sa->instr) {
	case SIGMA_ACTION_WRITEXBYTES:
	case SIGMA_ACTION_WRITESINGLE:
	case SIGMA_ACTION_WRITESAFELOAD:
		payload = sigma_action_len(sa);
		break;
	default:
		break;
	}

	payload = ALIGN(payload, 2);

	return payload + sizeof(struct sigma_action);
}

/*
 * Returns a negative error value in case of an error, 0 if processing of
 * the firmware should be stopped after this action, 1 otherwise.
 */
static int process_sigma_action(struct sigmadsp *sigmadsp,
	struct sigma_action *sa)
{
	size_t len = sigma_action_len(sa);
	int ret;

	pr_debug("%s: instr:%i addr:%#x len:%zu\n", __func__,
		sa->instr, sa->addr, len);

	switch (sa->instr) {
	case SIGMA_ACTION_WRITEXBYTES:
	case SIGMA_ACTION_WRITESINGLE:
	case SIGMA_ACTION_WRITESAFELOAD:
		ret = sigmadsp_write(sigmadsp, be16_to_cpu(sa->addr),
				sa->payload, len - 2);
		if (ret < 0)
			return -EINVAL;
		break;
	case SIGMA_ACTION_DELAY:
		udelay(len);
		len = 0;
		break;
	case SIGMA_ACTION_END:
		return 0;
	default:
		return -EINVAL;
	}

	return 1;
}

static int sigmadsp_fw_load_v1(struct sigmadsp *sigmadsp,
	const struct firmware *fw)
{
	struct sigma_action *sa;
	size_t size, pos;
	int ret;

	pos = sizeof(struct sigma_firmware_header);

	while (pos + sizeof(*sa) <= fw->size) {
		sa = (struct sigma_action *)(fw->data + pos);

		size = sigma_action_size(sa);
		pos += size;
		if (pos > fw->size || size == 0)
			break;

		ret = process_sigma_action(sigmadsp, sa);

		pr_debug("%s: action returned %i\n", __func__, ret);

		if (ret <= 0)
			return ret;
	}

	if (pos != fw->size)
		return -EINVAL;

	return 0;
}

void sigmadsp_firmware_release(struct snd_soc_codec *codec,
	struct sigmadsp *sigmadsp)
{
	struct sigmadsp_control *ctrl, *_ctrl;

	list_for_each_entry_safe(ctrl, _ctrl, &sigmadsp->ctrl_list, head)
		snd_ctl_remove(codec->card->snd_card, ctrl->kcontrol);

	INIT_LIST_HEAD(&sigmadsp->ctrl_list);
}
EXPORT_SYMBOL_GPL(sigmadsp_firmware_release);

int sigmadsp_firmware_load(struct snd_soc_codec *codec,
	struct sigmadsp *sigmadsp, const char *name,
	unsigned int sample_rate)
{
	const struct sigma_firmware_header *ssfw_head;
	const struct firmware *fw;
	int ret;
	u32 crc;

	sigmadsp_firmware_release(codec, sigmadsp);

	/* first load the blob */
	ret = request_firmware(&fw, name, codec->dev);
	if (ret) {
		pr_debug("%s: request_firmware() failed with %i\n", __func__, ret);
		goto done;
	}

	/* then verify the header */
	ret = -EINVAL;

	/*
	 * Reject too small or unreasonable large files. The upper limit has been
	 * chosen a bit arbitrarily, but it should be enough for all practical
	 * purposes and having the limit makes it easier to avoid integer
	 * overflows later in the loading process.
	 */
	if (fw->size < sizeof(*ssfw_head) || fw->size >= 0x4000000) {
		dev_err(codec->dev, "Failed to load firmware: Invalid size\n");
		goto done;
	}

	ssfw_head = (void *)fw->data;
	if (memcmp(ssfw_head->magic, SIGMA_MAGIC, ARRAY_SIZE(ssfw_head->magic))) {
		dev_err(codec->dev, "Failed to load firmware: Invalid magic\n");
		goto done;
	}

	crc = crc32(0, fw->data + sizeof(*ssfw_head),
			fw->size - sizeof(*ssfw_head));
	pr_debug("%s: crc=%x\n", __func__, crc);
	if (crc != le32_to_cpu(ssfw_head->crc)) {
		dev_err(codec->dev, "Failed to load firmware: Wrong crc checksum: expected %x got %x\n",
			le32_to_cpu(ssfw_head->crc), crc);
		goto done;
	}

	switch (ssfw_head->version) {
	case 1:
		ret = sigmadsp_fw_load_v1(sigmadsp, fw);
		break;
	case 2:
		ret = sigmadsp_fw_load_v2(codec, sigmadsp, fw);
		break;
	default:
		dev_err(codec->dev, "Unknown firmware version: %d\n",
			ssfw_head->version);
		ret = -EINVAL;
		break;
	}

done:
	release_firmware(fw);

	return ret;
}
EXPORT_SYMBOL_GPL(sigmadsp_firmware_load);

static void sigmadsp_init(struct sigmadsp *sigmadsp,
	const struct sigmadsp_ops *ops)
{
	sigmadsp->ops = ops;
	INIT_LIST_HEAD(&sigmadsp->ctrl_list);
}

#if IS_ENABLED(CONFIG_I2C)

static int sigmadsp_write_i2c(void *control_data,
	unsigned int addr, const uint8_t data[], size_t len)
{
	uint8_t *buf;
	int ret;

	buf = kzalloc(2 + len, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	put_unaligned_be16(addr, buf);
	memcpy(buf + 2, data, len);

	ret = i2c_master_send(control_data, buf, len + 2);

	kfree(buf);

	return ret;
}

static int sigmadsp_read_i2c(void *control_data,
	unsigned int addr, uint8_t data[], size_t len)
{
	struct i2c_client *client = control_data;
	struct i2c_msg msgs[2];
	uint8_t buf[2];
	int ret;

	put_unaligned_be16(addr, buf);

	msgs[0].addr = client->addr;
	msgs[0].len = sizeof(buf);
	msgs[0].buf = buf;
	msgs[0].flags = 0;

	msgs[1].addr = client->addr;
	msgs[1].len = len;
	msgs[1].buf = data;
	msgs[1].flags = I2C_M_RD;

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret < 0)
		return ret;
	else if (ret != ARRAY_SIZE(msgs))
		return -EIO;
	return 0;
}

void sigmadsp_init_i2c(struct sigmadsp *sigmadsp,
	const struct sigmadsp_ops *ops, struct i2c_client *client)
{
	sigmadsp->control_data = client;
	sigmadsp->write = sigmadsp_write_i2c;
	sigmadsp->read = sigmadsp_read_i2c;
	sigmadsp_init(sigmadsp, ops);
}
EXPORT_SYMBOL_GPL(sigmadsp_init_i2c);

#endif

#if IS_ENABLED(CONFIG_REGMAP)

static int sigmadsp_write_regmap(void *control_data,
	unsigned int addr, const uint8_t data[], size_t len)
{
	return regmap_raw_write(control_data, addr,
		data, len);
}

static int sigmadsp_read_regmap(void *control_data,
	unsigned int addr, uint8_t data[], size_t len)
{
	return regmap_raw_read(control_data, addr,
		data, len);
}

void sigmadsp_init_regmap(struct sigmadsp *sigmadsp,
	const struct sigmadsp_ops *ops, struct regmap *regmap)
{
	sigmadsp->control_data = regmap;
	sigmadsp->write = sigmadsp_write_regmap;
	sigmadsp->read = sigmadsp_read_regmap;
	sigmadsp_init(sigmadsp, ops);
}
EXPORT_SYMBOL_GPL(sigmadsp_init_regmap);

#endif

MODULE_LICENSE("GPL");
