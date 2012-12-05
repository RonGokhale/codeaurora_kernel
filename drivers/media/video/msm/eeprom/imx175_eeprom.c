/* Copyright (c) 2012, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <msm.h>
#include "msm_camera_eeprom.h"
#include <media/v4l2-subdev.h>

#define SPI_PROTOCOL_OFFSET 4
#define SPI_PROTOCOL_READ_CODE 0x03
#define SPI_PROTOCOL_WRITE_ENABLE_CODE 0x06
#define SPI_PROTOCOL_SECTOR_ERASE_CODE 0x20
#define SPI_PROTOCOL_READ_STATUS0_CODE 0x05
#define SPI_PROTOCOL_PAGE_PROGRAM_CODE 0x02

#define sizeof_member(type, member) sizeof(((type *)0)->member)

#define CHROMATIX_ADDRESS 0x000000
#define CHROMATIX_SIZE    10240
#define SPI_PAGE_SIZE     256

DEFINE_MUTEX(imx175_eeprom_mutex);
static struct msm_eeprom_ctrl_t imx175_eeprom_t;

int32_t imx175_spi_txfr(struct spi_device *spi,
	char *tx_buf, char *rx_buf, uint32_t num_byte)
{
	struct spi_transfer t;
	struct spi_message m;
	int ret;

	spi->bits_per_word = 8;
	spi->mode = SPI_MODE_0;
	memset(&t, 0, sizeof(t));

	t.tx_buf = tx_buf;
	t.rx_buf = rx_buf;
	t.len = num_byte;
	t.speed_hz = 10800000;//supported rates can be obtained from clock-8960.c
	spi_setup(spi);
	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	ret = spi_sync(spi, &m);
	if (ret)
		pr_err("%s: Error spi transfer: %d\n", __func__, ret);
	return ret;
}

int32_t imx175_spi_write_enable(struct msm_eeprom_ctrl_t *eeprom_ctrl)
{
	struct spi_device *spi = eeprom_ctrl->spi;
	char tx_buf;
	char rx_buf;

	tx_buf = SPI_PROTOCOL_WRITE_ENABLE_CODE;

	return imx175_spi_txfr(spi, &tx_buf, &rx_buf, 1);
}

int32_t imx175_spi_read_status0(struct msm_eeprom_ctrl_t *eeprom_ctrl,
	uint8_t *read_status)
{
	char tx_buf[2] = {0};
	char rx_buf[2] = {0};
	int ret;

	tx_buf[0] = SPI_PROTOCOL_READ_STATUS0_CODE;
	ret = imx175_spi_txfr(eeprom_ctrl->spi, &tx_buf[0], &rx_buf[0], 2);
	*read_status = rx_buf[1];
	return ret;
}

int32_t imx175_spi_sector_erase(struct msm_eeprom_ctrl_t *eeprom_ctrl,
	uint32_t sector_addr)
{
	char tx_buf[4] = {0};
	char rx_buf[4] = {0};
	char read_status = 0xFF;
	int ret, i = 0;

	ret = imx175_spi_write_enable(eeprom_ctrl);
	if (ret) {
		pr_err("%s: Error write enable: %d\n", __func__, ret);
		return ret;
	}

	tx_buf[0] = SPI_PROTOCOL_SECTOR_ERASE_CODE;
	tx_buf[1] = (char)((sector_addr >> 16) & 0xFF);
	tx_buf[2] = (char)((sector_addr >> 8) & 0xFF);
	tx_buf[3] = (char)(sector_addr & 0xFF);
	ret = imx175_spi_txfr(eeprom_ctrl->spi, &tx_buf[0], &rx_buf[0], 4);
	if (ret) {
		pr_err("%s: Error sector erase: %d\n", __func__, ret);
		return ret;
	}

	do {
		if (i > 10) {
			pr_err("%s: Sector erase timeout\n", __func__);
			return -ETIMEDOUT;
		}
		ret = imx175_spi_read_status0(eeprom_ctrl, &read_status);
		if (ret) {
			pr_err("%s: Error read status0: %d\n", __func__, ret);
			return ret;
		}
		i++;
		msleep(40);
	} while(read_status != 0);
	return ret;
}

int32_t imx175_spi_page_program(struct msm_eeprom_ctrl_t *eeprom_ctrl,
	uint32_t reg_addr, char *tx_buf, char *rx_buf, uint32_t num_byte)
{
	char read_status = 0xFF;
	int ret, i = 0;

	ret = imx175_spi_write_enable(eeprom_ctrl);
	if (ret) {
		pr_err("%s: Error write enable: %d\n", __func__, ret);
		return ret;
	}

	tx_buf[0] = SPI_PROTOCOL_PAGE_PROGRAM_CODE;
	tx_buf[1] = (char)((reg_addr >> 16) & 0xFF);
	tx_buf[2] = (char)((reg_addr >> 8) & 0xFF);
	tx_buf[3] = (char)(reg_addr & 0xFF);
	ret = imx175_spi_txfr(eeprom_ctrl->spi, &tx_buf[0], &rx_buf[0],
		num_byte + SPI_PROTOCOL_OFFSET);
	if (ret) {
		pr_err("%s: Error page program: %d\n", __func__, ret);
		return ret;
	}

	do {
		if (i > 10) {
			pr_err("%s: Page program timeout\n", __func__);
			return -ETIMEDOUT;
		}
		ret = imx175_spi_read_status0(eeprom_ctrl, &read_status);
		if (ret) {
			pr_err("%s: Error read status0: %d\n", __func__, ret);
			return ret;
		}
		i++;
		msleep(40);
	} while(read_status != 0);
	return ret;
}

int32_t imx175_spi_read(struct msm_eeprom_ctrl_t *eeprom_ctrl,
	uint32_t reg_addr, void *inbuf, uint32_t num_byte)
{
	char *tx_buf;
	int ret;
	char *data = (char *)inbuf;

	tx_buf = kmalloc(num_byte + SPI_PROTOCOL_OFFSET, GFP_KERNEL);
	if (NULL == tx_buf) {
		pr_err("%s: Error allocating tx_buf.\n", __func__);
		return -ENOMEM;
	}

	tx_buf[0] = SPI_PROTOCOL_READ_CODE;
	tx_buf[1] = (char)((reg_addr >> 16) & 0xFF);
	tx_buf[2] = (char)((reg_addr >> 8) & 0xFF);
	tx_buf[3] = (char)(reg_addr & 0xFF);
	ret = imx175_spi_txfr(eeprom_ctrl->spi, &tx_buf[0], data,
		num_byte + SPI_PROTOCOL_OFFSET);
	kfree(tx_buf);
	return ret;
}

int32_t imx175_direct_read(struct msm_eeprom_ctrl_t *ectrl,
	struct eeprom_data_access_t *data_access)
{
	char *rx_buf;
	int ret;

	if (NULL == data_access->data) {
		pr_err("%s: Invalid read buffer.\n", __func__);
		return -EINVAL;
	}

	rx_buf = kmalloc(data_access->num_bytes + SPI_PROTOCOL_OFFSET,
		GFP_KERNEL);
	if (NULL == rx_buf) {
		pr_err("%s: Error allocating rx_buf.\n", __func__);
		return -ENOMEM;
	}

	ret = imx175_spi_read(ectrl, data_access->addr, rx_buf,
		data_access->num_bytes);
	if (!ret) {
		pr_err("%s: spi_read is success\n", __func__);
		if (copy_to_user(data_access->data, &rx_buf[SPI_PROTOCOL_OFFSET],
			data_access->num_bytes)) {
			pr_err("%s: Error copy to userspace\n", __func__);
			ret = -EFAULT;
		}
	}
	kfree(rx_buf);
	return ret;
}

int32_t imx175_direct_write(struct msm_eeprom_ctrl_t *ectrl,
	struct eeprom_data_access_t *data_access)
{
	char *rx_buf;
	char *tx_buf;
	int ret;

	if (NULL == data_access->data) {
		pr_err("%s: Invalid read buffer.\n", __func__);
		return -EINVAL;
	}

	tx_buf = kmalloc(data_access->num_bytes + SPI_PROTOCOL_OFFSET,
		GFP_KERNEL);
	if (NULL == tx_buf) {
		pr_err("%s: Error allocating tx_buf.\n", __func__);
		return -ENOMEM;
	}

	rx_buf = kmalloc(data_access->num_bytes + SPI_PROTOCOL_OFFSET,
		GFP_KERNEL);
	if (NULL == rx_buf) {
		pr_err("%s: Error allocating rx_buf.\n", __func__);
		kfree(tx_buf);
		return -ENOMEM;
	}

	if (copy_from_user(&tx_buf[SPI_PROTOCOL_OFFSET], data_access->data,
		data_access->num_bytes)) {
		pr_err("%s: Error copy from userspace\n", __func__);
		kfree(tx_buf);
		kfree(rx_buf);
		return -EFAULT;
	}

	ret = imx175_spi_page_program(ectrl, data_access->addr, tx_buf,
		rx_buf, data_access->num_bytes);
	kfree(tx_buf);
	kfree(rx_buf);
	return ret;
}

int32_t imx175_direct_erase(struct msm_eeprom_ctrl_t *ectrl,
	struct eeprom_data_access_t *data_access)
{
	int ret;
	ret = imx175_spi_sector_erase(ectrl, data_access->addr);
	if (ret)
		pr_err("%s: Error sector erase: %d\n", __func__, ret);
	return ret;
}

static struct v4l2_subdev_core_ops imx175_eeprom_subdev_core_ops = {
	.ioctl = msm_eeprom_subdev_ioctl,
};

static struct v4l2_subdev_ops imx175_eeprom_subdev_ops = {
	.core = &imx175_eeprom_subdev_core_ops,
};


/*
 * AWB, AF, LSC
 */

/* data after formatting */
static struct msm_calib_af imx175_af_data;
static struct msm_calib_wb imx175_wb_data;
static struct msm_calib_lsc imx175_lsc_data;
static struct msm_calib_wb imx175_gld_wb_data;
static struct msm_calib_lsc imx175_gld_lsc_data;

/* based on F-Rom definition */
typedef struct {
	uint32_t inf1;
	uint32_t inf2;
	uint32_t macro1;
	uint32_t macro2;
	uint32_t position;
} FRom_af;
typedef struct {
	uint32_t rg;
	uint32_t bg;
	uint32_t gr_gb;
} FRom_wb;
typedef struct {
	uint16_t r_gain[221];
	uint16_t gr_gain[221];
	uint16_t gb_gain[221];
	uint16_t b_gain[221];
} FRom_lsc;

/* buffer for receiving data from F-Rom */
static uint8_t imx175_afcalib_data[SPI_PROTOCOL_OFFSET + sizeof(FRom_af)];
static uint8_t imx175_wbcalib_data[SPI_PROTOCOL_OFFSET + sizeof(FRom_wb)];
static uint8_t imx175_lsccalib_data[SPI_PROTOCOL_OFFSET + sizeof(FRom_lsc)];
static uint8_t imx175_gld_wbcalib_data[SPI_PROTOCOL_OFFSET + sizeof(FRom_wb)];
static uint8_t imx175_gld_lsccalib_data[SPI_PROTOCOL_OFFSET + sizeof(FRom_lsc)];
static uint8_t imx175_mod_info_data[SPI_PROTOCOL_OFFSET + 4];

static struct msm_camera_eeprom_info_t imx175_calib_supp_info = {
	{TRUE, sizeof(imx175_af_data), 0, 1},
	{TRUE, sizeof(imx175_wb_data), 1, 1},
	{TRUE, sizeof(imx175_lsc_data), 2, 1},
	{FALSE, 0, 0, 1},
	{FALSE, 0, 0, 1},
	{TRUE, sizeof(imx175_gld_wb_data), 3, 1},
	{TRUE, sizeof(imx175_gld_lsc_data), 4, 1},
};

static struct msm_camera_eeprom_read_t imx175_eeprom_read_tbl[] = {
	{0x000040, &imx175_mod_info_data[0], sizeof(imx175_mod_info_data), 0},
	{0x001000, &imx175_afcalib_data[0], sizeof(FRom_af), 0},
	{0x002000, &imx175_wbcalib_data[0], sizeof(FRom_wb), 0},
	{0x00300C, &imx175_lsccalib_data[0], sizeof(FRom_lsc), 0},
	{0x001200, &imx175_gld_wbcalib_data[0], sizeof(FRom_wb), 0},
	{0x001218, &imx175_gld_lsccalib_data[0], sizeof(FRom_lsc), 0},
};

static struct msm_camera_eeprom_data_t imx175_eeprom_data_tbl[] = {
	{&imx175_af_data, sizeof(imx175_af_data)},
	{&imx175_wb_data, sizeof(imx175_wb_data)},
	{&imx175_lsc_data, sizeof(imx175_lsc_data)},
	{&imx175_gld_wb_data, sizeof(imx175_wb_data)},
	{&imx175_gld_lsc_data, sizeof(imx175_lsc_data)},
};

static void imx175_format_afdata(void)
{
	FRom_af* af = (FRom_af*)&imx175_afcalib_data[SPI_PROTOCOL_OFFSET];

	imx175_af_data.inf_dac = 0;
	imx175_af_data.macro_dac = 0;
	imx175_af_data.start_dac = 0;
	imx175_af_data.pan_dac = (uint16_t)af->position;
}
static void imx175_format_wbdata(void)
{
	FRom_wb* wb = (FRom_wb*)&imx175_wbcalib_data[SPI_PROTOCOL_OFFSET];
	FRom_wb* gld_wb = (FRom_wb*)&imx175_gld_wbcalib_data[SPI_PROTOCOL_OFFSET];

	imx175_wb_data.r_over_g = (uint16_t)wb->rg;
	imx175_wb_data.b_over_g = (uint16_t)wb->bg;
	imx175_wb_data.gr_over_gb = 1;/*(uint16_t)wb->gr_gb;*/

	imx175_gld_wb_data.r_over_g = (uint16_t)gld_wb->rg;
	imx175_gld_wb_data.b_over_g = (uint16_t)gld_wb->bg;
	imx175_gld_wb_data.gr_over_gb = 1;/*(uint16_t)gld_wb->gr_gb;*/
}
static void imx175_format_lscdata(void)
{
	int i;
	uint16_t* r = (uint16_t*)&imx175_lsccalib_data[SPI_PROTOCOL_OFFSET];
	uint16_t* gr = (uint16_t*)((uint8_t*)r + sizeof_member(FRom_lsc, r_gain));
	uint16_t* gb = (uint16_t*)((uint8_t*)gr + sizeof_member(FRom_lsc, gr_gain));
	uint16_t* b = (uint16_t*)((uint8_t*)gb + sizeof_member(FRom_lsc, gb_gain));

	uint16_t* gld_r = (uint16_t*)&imx175_gld_lsccalib_data[SPI_PROTOCOL_OFFSET];
	uint16_t* gld_gr = (uint16_t*)((uint8_t*)gld_r + sizeof_member(FRom_lsc, r_gain));
	uint16_t* gld_gb = (uint16_t*)((uint8_t*)gld_gr + sizeof_member(FRom_lsc, gr_gain));
	uint16_t* gld_b = (uint16_t*)((uint8_t*)gld_gb + sizeof_member(FRom_lsc, gb_gain));

	for (i = 0; i < 221; i++) {
		imx175_lsc_data.r_gain[i] = r[i];
	}
	for (i = 0; i < 221; i++) {
		imx175_lsc_data.b_gain[i] = b[i];
	}
	for (i = 0; i < 221; i++) {
		imx175_lsc_data.gr_gain[i] = gr[i];
	}
	for (i = 0; i < 221; i++) {
		imx175_lsc_data.gb_gain[i] = gb[i];
	}

	for (i = 0; i < 221; i++) {
		imx175_gld_lsc_data.r_gain[i] = gld_r[i];
	}
	for (i = 0; i < 221; i++) {
		imx175_gld_lsc_data.b_gain[i] = gld_b[i];
	}
	for (i = 0; i < 221; i++) {
		imx175_gld_lsc_data.gr_gain[i] = gld_gr[i];
	}
	for (i = 0; i < 221; i++) {
		imx175_gld_lsc_data.gb_gain[i] = gld_gb[i];
	}
}

uint8_t imx175_check_module_info(void)
{
	uint8_t *ptr = &imx175_mod_info_data[SPI_PROTOCOL_OFFSET];
	if (ptr[0] != 0x4F)
		return 0;
	if ((ptr[1] != 0x30) &&
		(ptr[1] != 0x38))
		return 0;
	if ((ptr[2] != 0x30) &&
		(ptr[2] != 0x38))
		return 0;
	if (ptr[3] != 0x51)
		return 0;
	return 1;
}

void imx175_format_calibrationdata(void)
{
	uint8_t is_all_supported = 0;
	is_all_supported = imx175_check_module_info();
	if (is_all_supported) {
		imx175_format_afdata();
		imx175_format_wbdata();
		imx175_format_lscdata();
	} else {
		imx175_calib_supp_info.af.is_supported = FALSE;
		imx175_calib_supp_info.wb.is_supported = FALSE;
		imx175_calib_supp_info.lsc.is_supported = FALSE;
		imx175_calib_supp_info.gld_wb.is_supported = FALSE;
		imx175_calib_supp_info.gld_lsc.is_supported = FALSE;
	}
}


/*
 * CHROMATIX
 */

typedef struct {
	uint8_t *data;
	uint8_t size;
} chromatix_raw_data_type;

static chromatix_raw_data_type *chromatix_data;
static uint8_t chromatix_array_size;

static void alloc_chromatix_memory(void)
{
	chromatix_array_size = CHROMATIX_SIZE / SPI_PAGE_SIZE;
	if (CHROMATIX_SIZE % SPI_PAGE_SIZE > 0)
		chromatix_array_size++;

	chromatix_data = kmalloc(
		sizeof(chromatix_raw_data_type) * chromatix_array_size, GFP_KERNEL);
}
static void imx175_read_choromatix(struct msm_eeprom_ctrl_t *eeprom_ctrl)
{
	int32_t size = CHROMATIX_SIZE;
	uint32_t address = CHROMATIX_ADDRESS;
	uint32_t num_byte = SPI_PAGE_SIZE;
	uint32_t offset = 0;

	alloc_chromatix_memory();

	while (size > 0) {
		if (size < SPI_PAGE_SIZE)
			num_byte = size;

		chromatix_data[offset].data = kmalloc(num_byte + SPI_PROTOCOL_OFFSET,
			GFP_KERNEL);
		chromatix_data[offset].data[SPI_PROTOCOL_OFFSET] = 0xAA;
		chromatix_data[offset].size = num_byte;

#ifdef READ_CHROMATIX_FROM
		if (imx175_spi_read(eeprom_ctrl, address,
							chromatix_data[offset].data, num_byte) < 0) {
			printk("%s : read fail\n", __func__);
		}
#endif
		size -= num_byte;
		address += num_byte;
		offset++;
	};
}

int32_t imx175_get_chromatix(struct msm_eeprom_ctrl_t *ectrl, void *edata)
{
	int32_t rc = 0;
	int i;
	struct chromatix_params *chr_params = (struct chromatix_params *)edata;
	uint8_t *offset = chr_params->chromatix;

	mutex_lock(ectrl->eeprom_mutex);

	if (chr_params->chromatix_size == CHROMATIX_SIZE) {
		for (i = 0; i < chromatix_array_size; i++) {
			if (copy_to_user(offset,
					chromatix_data[i].data + SPI_PROTOCOL_OFFSET,
					chromatix_data[i].size)) {
				rc = -EFAULT;
			}
			offset += chromatix_data[i].size;
		}
	}

	mutex_unlock(ectrl->eeprom_mutex);

	return rc;
}

static int __devinit imx175_spi_probe(struct spi_device *spi)
{
	int rc = 0;
	struct msm_cam_subdev_info sd_info;
	struct msm_eeprom_ctrl_t *eeprom_ctrl =
		(struct msm_eeprom_ctrl_t *)spi_get_device_id(spi)->driver_data;

	eeprom_ctrl->spi = spi;
	printk("imx175 spi driver probe\n");

	if (eeprom_ctrl->func_tbl.eeprom_init != NULL) {
		rc = eeprom_ctrl->func_tbl.eeprom_init(eeprom_ctrl,
			eeprom_ctrl->i2c_client.client->adapter);
	}
	msm_camera_eeprom_read_tbl(eeprom_ctrl,
		eeprom_ctrl->read_tbl,
		eeprom_ctrl->read_tbl_size);

	imx175_read_choromatix(eeprom_ctrl);

	if (eeprom_ctrl->func_tbl.eeprom_format_data != NULL)
		eeprom_ctrl->func_tbl.eeprom_format_data();

	if (eeprom_ctrl->func_tbl.eeprom_release != NULL)
		rc = eeprom_ctrl->func_tbl.eeprom_release(eeprom_ctrl);

	/* Initialize sub device */
	v4l2_spi_subdev_init(&eeprom_ctrl->sdev,
		spi, eeprom_ctrl->eeprom_v4l2_subdev_ops);

	sd_info.sdev_type = EEPROM_DEV;
	eeprom_ctrl->sdev.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	msm_cam_register_subdev_node(&eeprom_ctrl->sdev, &sd_info);
	media_entity_init(&eeprom_ctrl->sdev.entity, 0, NULL, 0);
	eeprom_ctrl->sdev.entity.type = MEDIA_ENT_T_DEVNODE_V4L;
	eeprom_ctrl->sdev.entity.group_id = EEPROM_DEV;
	eeprom_ctrl->sdev.entity.name = "chromatix_eeprom";
	eeprom_ctrl->sdev.entity.revision = eeprom_ctrl->sdev.devnode->num;

	return 0;
}

static const struct spi_device_id imx175_spi_id[] = {
	{"imx175_spi", (kernel_ulong_t)&imx175_eeprom_t},
	{ }
};

static struct spi_driver imx175_spi_driver = {
	.id_table = imx175_spi_id,
	.driver = {
		.name	= "imx175_spi",
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
	},
	.probe		= imx175_spi_probe,
};

void imx175_eeprom_init(void)
{
	spi_register_driver(&imx175_spi_driver);
}

static struct msm_eeprom_ctrl_t imx175_eeprom_t = {
	.eeprom_v4l2_subdev_ops = &imx175_eeprom_subdev_ops,
	.eeprom_mutex = &imx175_eeprom_mutex,

	.func_tbl = {
		.eeprom_init = NULL,
		.eeprom_release = NULL,
		.eeprom_get_info = msm_camera_eeprom_get_info,
		.eeprom_get_data = msm_camera_eeprom_get_data,
		.eeprom_set_dev_addr = NULL,
		.eeprom_format_data = imx175_format_calibrationdata,
		.eeprom_read = imx175_spi_read,
		.eeprom_direct_data_read = imx175_direct_read,
		.eeprom_direct_data_write = imx175_direct_write,
		.eeprom_direct_data_erase = imx175_direct_erase,
		.eeprom_get_chromatix = imx175_get_chromatix,
	},
	.info = &imx175_calib_supp_info,
	.info_size = sizeof(struct msm_camera_eeprom_info_t),
	.read_tbl = imx175_eeprom_read_tbl,
	.read_tbl_size = ARRAY_SIZE(imx175_eeprom_read_tbl),
	.data_tbl = imx175_eeprom_data_tbl,
	.data_tbl_size = ARRAY_SIZE(imx175_eeprom_data_tbl),
};
