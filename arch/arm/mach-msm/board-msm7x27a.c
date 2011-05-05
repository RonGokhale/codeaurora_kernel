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
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/gpio_event.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/board.h>
#include <mach/msm_iomap.h>
#include <mach/msm_hsusb.h>
#include <mach/rpc_hsusb.h>
#include <mach/rpc_pmapp.h>
#include <mach/usbdiag.h>
#include <mach/usb_gadget_fserial.h>
#include <mach/msm_memtypes.h>
#include <mach/msm_serial_hs.h>
#include <linux/usb/android_composite.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <mach/vreg.h>
#include <asm/hardware/cache-l2x0.h>
#include <mach/pmic.h>
#include <mach/socinfo.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <asm/mach/mmc.h>
#include <linux/i2c.h>
#include <linux/i2c/sx150x.h>
#include <linux/gpio.h>
#include <linux/android_pmem.h>
#include <linux/bootmem.h>
#include <linux/mfd/marimba.h>
#include <mach/vreg.h>
#include <linux/power_supply.h>
#include <mach/rpc_pmapp.h>

#include <mach/msm_battery.h>
#include <linux/smsc911x.h>
#include <linux/atmel_maxtouch.h>
#include "devices.h"
#include "timer.h"
#include "devices-msm7x2xa.h"
#include "pm.h"
#include <mach/rpc_server_handset.h>

#define PMEM_KERNEL_EBI1_SIZE	0x1C000
#define MSM_PMEM_AUDIO_SIZE	0x5B000
#define BAHAMA_SLAVE_ID_FM_ADDR         0x2A
#define BAHAMA_SLAVE_ID_QMEMBIST_ADDR   0x7B
#define FM_GPIO	83

enum {
	GPIO_EXPANDER_IRQ_BASE	= NR_MSM_IRQS + NR_GPIO_IRQS,
	GPIO_EXPANDER_GPIO_BASE	= NR_MSM_GPIOS,
	/* SURF expander */
	GPIO_SURF_EXPANDER_BASE	= GPIO_EXPANDER_GPIO_BASE,
	GPIO_BT_SYS_REST_EN	= GPIO_SURF_EXPANDER_BASE,
	GPIO_WLAN_EXT_POR_N,
	GPIO_DISPLAY_PWR_EN,
	GPIO_BACKLIGHT_EN,
	GPIO_PRESSURE_XCLR,
	GPIO_VREG_S3_EXP,
	GPIO_UBM2M_PWRDWN,
	GPIO_ETM_MODE_CS_N,
	GPIO_HOST_VBUS_EN,
	GPIO_SPI_MOSI,
	GPIO_SPI_MISO,
	GPIO_SPI_CLK,
	GPIO_SPI_CS0_N,
	GPIO_SURF_EXPANDER_IO13,
	GPIO_SURF_EXPANDER_IO14,
	GPIO_SURF_EXPANDER_IO15,
	/* Camera expander on SURF */
	GPIO_SURF_CAM_EXPANDER_BASE	= GPIO_SURF_EXPANDER_BASE + 16,
	GPIO_SURF_CAM_GP_STROBE_READY	= GPIO_SURF_CAM_EXPANDER_BASE,
	GPIO_SURF_CAM_GP_AFBUSY,
	GPIO_SURF_CAM_GP_CAM_PWDN,
	GPIO_SURF_CAM_GP_CAM1MP_XCLR,
	GPIO_SURF_CAM_GP_CAMIF_RESET_N,
	GPIO_SURF_CAM_GP_STROBE_CE,
	GPIO_SURF_CAM_GP_LED_EN1,
	GPIO_SURF_CAM_GP_LED_EN2,
	/* FFA expander */
	GPIO_FFA_EXPANDER_BASE	= GPIO_SURF_CAM_EXPANDER_BASE + 8,
	GPIO_FFA_BT_SYS_REST_EN	= GPIO_FFA_EXPANDER_BASE,
	GPIO_FFA_WLAN_EXT_POR_N,
	GPIO_FFA_LCD_PWR_EN_N,
	GPIO_FFA_GPIO_EXP_03,
	GPIO_FFA_ALTIMETER_XCLR_N,
	GPIO_FFA_GPIO_EXP_05,
	GPIO_FFA_GPIO_EXP_06,
	GPIO_FFA_GPIO_EXP_07,
	GPIO_FFA_5V_BOOST_EN,
	GPIO_FFA_EXPANDER_IO09,
	GPIO_FFA_EXPANDER_IO10,
	GPIO_FFA_EXPANDER_IO11,
	GPIO_FFA_EXPANDER_IO12,
	GPIO_FFA_EXPANDER_IO13,
	GPIO_FFA_EXPANDER_IO14,
	GPIO_FFA_EXPANDER_IO15,
	/* Camera expander on FFA */
	GPIO_FFA_CAM_EXPANDER_BASE	= GPIO_FFA_EXPANDER_BASE + 16,
	GPIO_FFA_CAM_EXP_GPIO_00	= GPIO_FFA_CAM_EXPANDER_BASE,
	GPIO_FFA_CAM_EXP_GPIO_O1,
	GPIO_FFA_CAM_5MP_PWDN_VCM,
	GPIO_FFA_CAM_1MP_XCLR,
	GPIO_FFA_CAM_5MP_SHDN_N,
	GPIO_FFA_CAM_EXP_GPIO_05,
	GPIO_FFA_LED_DRV_EN1,
	GPIO_FFA_LED_DRV_EN2,
};

#if defined(CONFIG_GPIO_SX150X)
enum {
	SX150X_SURF,
	SX150X_SURF_CAM,
	SX150X_FFA,
	SX150X_FFA_CAM,
};

static struct sx150x_platform_data sx150x_data[] __initdata = {
	[SX150X_SURF]	= {
		.gpio_base		= GPIO_SURF_EXPANDER_BASE,
		.oscio_is_gpo		= false,
		.io_pullup_ena		= 0,
		.io_pulldn_ena		= 0,
		.io_open_drain_ena	= 0,
		.irq_summary		= -1,
		.irq_base		= GPIO_EXPANDER_IRQ_BASE,
	},
	[SX150X_SURF_CAM]	= {
		.gpio_base		= GPIO_SURF_CAM_EXPANDER_BASE,
		.oscio_is_gpo		= false,
		.io_pullup_ena		= 0,
		.io_pulldn_ena		= 0,
		.io_open_drain_ena	= 0,
		.irq_summary		= -1,
		.irq_base		= GPIO_EXPANDER_IRQ_BASE +
					  GPIO_SURF_CAM_EXPANDER_BASE -
					  GPIO_EXPANDER_GPIO_BASE,
	},
	[SX150X_FFA]	= {
		.gpio_base		= GPIO_FFA_EXPANDER_BASE,
		.oscio_is_gpo		= false,
		.io_pullup_ena		= 0,
		.io_pulldn_ena		= 0,
		.io_open_drain_ena	= 0,
		.irq_summary		= -1,
		.irq_base		= GPIO_EXPANDER_IRQ_BASE +
					  GPIO_FFA_EXPANDER_BASE -
					  GPIO_EXPANDER_GPIO_BASE,
	},
	[SX150X_FFA_CAM]	= {
		.gpio_base		= GPIO_FFA_CAM_EXPANDER_BASE,
		.oscio_is_gpo		= false,
		.io_pullup_ena		= 0,
		.io_pulldn_ena		= 0,
		.io_open_drain_ena	= 0,
		.irq_summary		= -1,
		.irq_base		= GPIO_EXPANDER_IRQ_BASE +
					  GPIO_FFA_CAM_EXPANDER_BASE -
					  GPIO_EXPANDER_GPIO_BASE,
	},

};
#endif

	/* FM Platform power and shutdown routines */

static struct vreg *fm_regulator;
static int fm_radio_setup(struct marimba_fm_platform_data *pdata)
{
	int rc = 0;
	const char *id = "FMPW";
	uint32_t irqcfg;

	/* Voting for 1.8V Regulator */
	fm_regulator = vreg_get(NULL , "msme1");
	if (IS_ERR(fm_regulator)) {
		pr_err("%s: vreg get failed with : (%ld)\n",
			__func__, PTR_ERR(fm_regulator));
		return -EINVAL;
	}

	/* Set the voltage level to 1.8V */
	rc = vreg_set_level(fm_regulator, 1800);
	if (rc < 0) {
		pr_err("%s: set regulator level failed with :(%d)\n",
			__func__, rc);
		goto fm_vreg_fail;
	}

	/* Enabling the 1.8V regulator */
	rc = vreg_enable(fm_regulator);
	if (rc) {
		pr_err("%s: enable regulator failed with :(%d)\n",
			__func__, rc);
		goto fm_vreg_fail;
	}

	/* Voting for 19.2MHz clock */
	rc = pmapp_clock_vote(id, PMAPP_CLOCK_ID_D1,
			PMAPP_CLOCK_VOTE_ON);
	if (rc < 0) {
		pr_err("%s: clock vote failed with :(%d)\n",
			 __func__, rc);
		goto fm_clock_vote_fail;
	}

	/* Configuring the FM GPIO */
	irqcfg = GPIO_CFG(FM_GPIO, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL,
			GPIO_CFG_2MA);

	rc = gpio_tlmm_config(irqcfg, GPIO_CFG_ENABLE);
	if (rc) {
		pr_err("%s: gpio_tlmm_config(%#x)=%d\n",
			 __func__, irqcfg, rc);
		goto fm_gpio_config_fail;
	}

	return 0;

fm_gpio_config_fail:
	pmapp_clock_vote(id, PMAPP_CLOCK_ID_D1,
		PMAPP_CLOCK_VOTE_OFF);

fm_clock_vote_fail:
	vreg_disable(fm_regulator);

fm_vreg_fail:
	vreg_put(fm_regulator);

	return rc;

};

static void fm_radio_shutdown(struct marimba_fm_platform_data *pdata)
{
	int rc;
	const char *id = "FMPW";

	/* Releasing the GPIO line used by FM */
	uint32_t irqcfg = GPIO_CFG(FM_GPIO, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP,
		GPIO_CFG_2MA);

	rc = gpio_tlmm_config(irqcfg, GPIO_CFG_ENABLE);
	if (rc)
		pr_err("%s: gpio_tlmm_config(%#x)=%d\n",
			 __func__, irqcfg, rc);

	/* Releasing the 1.8V Regulator */
	if (fm_regulator != NULL) {
		rc = vreg_disable(fm_regulator);

		if (rc)
			pr_err("%s: disable regulator failed:(%d)\n",
				__func__, rc);
		fm_regulator = NULL;
	}

	/* Voting off the clock */
	rc = pmapp_clock_vote(id, PMAPP_CLOCK_ID_D1,
		PMAPP_CLOCK_VOTE_OFF);

	if (rc < 0)
		pr_err("%s: voting off failed with :(%d)\n",
			__func__, rc);
}

static struct marimba_fm_platform_data marimba_fm_pdata = {
	.fm_setup = fm_radio_setup,
	.fm_shutdown = fm_radio_shutdown,
	.irq = MSM_GPIO_TO_INT(FM_GPIO),
	.vreg_s2 = NULL,
	.vreg_xo_out = NULL,
	/* Configuring the FM SoC as I2S Master */
	.is_fm_soc_i2s_master = true,
};

#if defined(CONFIG_BT) && defined(CONFIG_MARIMBA_CORE)

static struct platform_device msm_bt_power_device = {
	.name = "bt_power",
};
	struct bahama_config_register {
		u8 reg;
		u8 value;
		u8 mask;
	};
static const char * const vregs_bahama_name[] = {
	"msme1",
	"bt",
};
static struct vreg *vregs_bahama[ARRAY_SIZE(vregs_bahama_name)];

static unsigned bt_config_power_on[] = {
	/*RFR*/
	GPIO_CFG(43, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	/*CTS*/
	GPIO_CFG(44, 2, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	/*RX*/
	GPIO_CFG(45, 2, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	/*TX*/
	GPIO_CFG(46, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
};
static unsigned bt_config_power_off[] = {
	/*RFR*/
	GPIO_CFG(43, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	/*CTS*/
	GPIO_CFG(44, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	/*RX*/
	GPIO_CFG(45, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	/*TX*/
	GPIO_CFG(46, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
};

static int bahama_bt(int on)
{

	int rc = 0;
	int i;

	struct marimba config = { .mod_id =  SLAVE_ID_BAHAMA};

	struct bahama_variant_register {
		const size_t size;
		const struct bahama_config_register *set;
	};

	const struct bahama_config_register *p;

	u8 version;

	const struct bahama_config_register v10_bt_on[] = {
		{ 0xE9, 0x00, 0xFF },
		{ 0xF4, 0x80, 0xFF },
		{ 0xE4, 0x00, 0xFF },
		{ 0xE5, 0x00, 0x0F },
#ifdef CONFIG_WLAN
		{ 0xE6, 0x38, 0x7F },
		{ 0xE7, 0x06, 0xFF },
#endif
		{ 0xE9, 0x21, 0xFF },
		{ 0x01, 0x0C, 0x1F },
		{ 0x01, 0x08, 0x1F },
	};

	const struct bahama_config_register v20_bt_on_fm_off[] = {
		{ 0x11, 0x0C, 0xFF },
		{ 0x13, 0x01, 0xFF },
		{ 0xF4, 0x80, 0xFF },
		{ 0xF0, 0x00, 0xFF },
		{ 0xE9, 0x00, 0xFF },
#ifdef CONFIG_WLAN
		{ 0x81, 0x00, 0x7F },
		{ 0x82, 0x00, 0xFF },
		{ 0xE6, 0x38, 0x7F },
		{ 0xE7, 0x06, 0xFF },
#endif
		{ 0x8E, 0x15, 0xFF },
		{ 0x8F, 0x15, 0xFF },
		{ 0x90, 0x15, 0xFF },

		{ 0xE9, 0x21, 0xFF },
	};

	const struct bahama_config_register v20_bt_on_fm_on[] = {
		{ 0x11, 0x0C, 0xFF },
		{ 0x13, 0x01, 0xFF },
		{ 0xF4, 0x86, 0xFF },
		{ 0xF0, 0x06, 0xFF },
		{ 0xE9, 0x00, 0xFF },
#ifdef CONFIG_WLAN
		{ 0x81, 0x00, 0x7F },
		{ 0x82, 0x00, 0xFF },
		{ 0xE6, 0x38, 0x7F },
		{ 0xE7, 0x06, 0xFF },
#endif
		{ 0xE9, 0x21, 0xFF },
	};

	const struct bahama_config_register v10_bt_off[] = {
		{ 0xE9, 0x00, 0xFF },
	};

	const struct bahama_config_register v20_bt_off_fm_off[] = {
		{ 0xF4, 0x84, 0xFF },
		{ 0xF0, 0x04, 0xFF },
		{ 0xE9, 0x00, 0xFF }
	};

	const struct bahama_config_register v20_bt_off_fm_on[] = {
		{ 0xF4, 0x86, 0xFF },
		{ 0xF0, 0x06, 0xFF },
		{ 0xE9, 0x00, 0xFF }
	};
	const struct bahama_variant_register bt_bahama[2][3] = {
	{
		{ ARRAY_SIZE(v10_bt_off), v10_bt_off },
		{ ARRAY_SIZE(v20_bt_off_fm_off), v20_bt_off_fm_off },
		{ ARRAY_SIZE(v20_bt_off_fm_on), v20_bt_off_fm_on }
	},
	{
		{ ARRAY_SIZE(v10_bt_on), v10_bt_on },
		{ ARRAY_SIZE(v20_bt_on_fm_off), v20_bt_on_fm_off },
		{ ARRAY_SIZE(v20_bt_on_fm_on), v20_bt_on_fm_on }
	}
	};

	u8 offset = 0; /* index into bahama configs */
	on = on ? 1 : 0;
	version = marimba_read_bahama_ver(&config);

	if (version == BAHAMA_VER_UNSUPPORTED) {
		dev_err(&msm_bt_power_device.dev,
			"%s: unsupported version\n",
			__func__);
		return -EIO;
	}

	if (version == BAHAMA_VER_2_0) {
		if (marimba_get_fm_status(&config))
			offset = 0x01;
	}

	p = bt_bahama[on][version + offset].set;

	dev_info(&msm_bt_power_device.dev,
		"%s: found version %d\n", __func__, version);

	for (i = 0; i < bt_bahama[on][version + offset].size; i++) {
		u8 value = (p+i)->value;
		rc = marimba_write_bit_mask(&config,
			(p+i)->reg,
			&value,
			sizeof((p+i)->value),
			(p+i)->mask);
		if (rc < 0) {
			dev_err(&msm_bt_power_device.dev,
				"%s: reg %x write failed: %d\n",
				__func__, (p+i)->reg, rc);
			return rc;
		}
		dev_info(&msm_bt_power_device.dev,
			"%s: reg 0x%02x write value 0x%02x mask 0x%02x\n",
				__func__, (p+i)->reg,
				value, (p+i)->mask);
		value = 0;
		rc = marimba_read_bit_mask(&config,
				(p+i)->reg, &value,
				sizeof((p+i)->value), (p+i)->mask);
		if (rc < 0)
			dev_err(&msm_bt_power_device.dev, "%s marimba_read_bit_mask- error",
					__func__);
		dev_info(&msm_bt_power_device.dev,
			"%s: reg 0x%02x read value 0x%02x mask 0x%02x\n",
				__func__, (p+i)->reg,
				value, (p+i)->mask);
	}
	/* Update BT Status */
	if (on)
		marimba_set_bt_status(&config, true);
	else
		marimba_set_bt_status(&config, false);
	return rc;
}
static int bluetooth_switch_regulators(int on)
{
	int i, rc = 0;

	for (i = 0; i < ARRAY_SIZE(vregs_bahama_name); i++) {
		if (!vregs_bahama[i]) {
			pr_err("%s: vreg_get %s failed(%d)\n",
			__func__, vregs_bahama_name[i], rc);
			goto vreg_fail;
		}
		rc = on ? vreg_set_level(vregs_bahama[i], i ? 2900 :
			1800) : 0;

		if (rc < 0) {
			pr_err("%s: vreg set level failed (%d)\n",
					__func__, rc);
			goto vreg_set_level_fail;
		}

		rc = on ? vreg_enable(vregs_bahama[i]) :
			  vreg_disable(vregs_bahama[i]);

		if (rc < 0) {
			pr_err("%s: vreg %s %s failed(%d)\n",
				__func__, vregs_bahama_name[i],
			       on ? "enable" : "disable", rc);
			goto vreg_fail;
			}
	}
	return rc;

vreg_fail:
	while (i) {
		if (on)
			vreg_disable(vregs_bahama[--i]);
		}
vreg_set_level_fail:
	vreg_put(vregs_bahama[0]);
	vreg_put(vregs_bahama[1]);
	return rc;
}

static unsigned int msm_bahama_setup_power(void)
{
	int rc = 0;
	struct vreg *vreg_s3 = NULL;

	vreg_s3 = vreg_get(NULL, "msme1");
	if (IS_ERR(vreg_s3)) {
		pr_err("%s: vreg get failed (%ld)\n",
			__func__, PTR_ERR(vreg_s3));
		return PTR_ERR(vreg_s3);
	}
	rc = vreg_set_level(vreg_s3, 1800);
	if (rc < 0) {
		pr_err("%s: vreg set level failed (%d)\n",
				__func__, rc);
		goto vreg_fail;
	}
	rc = vreg_enable(vreg_s3);
	if (rc < 0) {
		pr_err("%s: vreg enable failed (%d)\n",
		       __func__, rc);
		goto vreg_fail;
	}
	/*setup Bahama_sys_reset_n*/
	rc = gpio_request(GPIO_BT_SYS_REST_EN, "bahama sys_rst_n");
	if (rc < 0) {
		pr_err("%s: gpio_request %d = %d\n", __func__,
			GPIO_BT_SYS_REST_EN, rc);
		goto vreg_fail;
	}
	rc = gpio_direction_output(GPIO_BT_SYS_REST_EN, 1);
	if (rc < 0) {
		pr_err("%s: gpio_direction_output %d = %d\n", __func__,
			GPIO_BT_SYS_REST_EN, rc);
		goto gpio_fail;
	}
	return rc;
gpio_fail:
	gpio_free(GPIO_BT_SYS_REST_EN);
vreg_fail:
	vreg_put(vreg_s3);
	return rc;
}

static unsigned int msm_bahama_shutdown_power(int value)
{
	int rc = 0;
	struct vreg *vreg_s3 = NULL;

	vreg_s3 = vreg_get(NULL, "msme1");
	if (IS_ERR(vreg_s3)) {
		pr_err("%s: vreg get failed (%ld)\n",
			__func__, PTR_ERR(vreg_s3));
		return PTR_ERR(vreg_s3);
	}
	rc = vreg_disable(vreg_s3);
	if (rc) {
		pr_err("%s: vreg disable failed (%d)\n",
		       __func__, rc);
		vreg_put(vreg_s3);
		return rc;
	}

	return rc;
}


static unsigned int msm_bahama_core_config(int type)
{
	int rc = 0;

	if (type == BAHAMA_ID) {
		int i;
		struct marimba config = { .mod_id = SLAVE_ID_BAHAMA };
		const struct bahama_config_register v20_init[] = {
			/* reg, value, mask */
			{ 0xF4, 0x84, 0xFF }, /* AREG */
			{ 0xF0, 0x04, 0xFF } /* DREG */
		};
		if (marimba_read_bahama_ver(&config) == BAHAMA_VER_2_0) {
			for (i = 0; i < ARRAY_SIZE(v20_init); i++) {
				u8 value = v20_init[i].value;
				rc = marimba_write_bit_mask(&config,
					v20_init[i].reg,
					&value,
					sizeof(v20_init[i].value),
					v20_init[i].mask);
				if (rc < 0) {
					pr_err("%s: reg %d write failed: %d\n",
						__func__, v20_init[i].reg, rc);
					return rc;
				}
				pr_debug("%s: reg 0x%02x value 0x%02x"
					" mask 0x%02x\n",
					__func__, v20_init[i].reg,
					v20_init[i].value, v20_init[i].mask);
			}
		}
	}
	pr_debug("core type: %d\n", type);
	return rc;
}

static int bluetooth_power(int on)
{
	int pin, rc = 0;
	const char *id = "BTPW";
	if (on) {
		/*setup power for BT SOC*/
		rc = bluetooth_switch_regulators(on);
		if (rc < 0) {
			pr_err("%s: bluetooth_switch_regulators rc = %d",
			__func__, rc);
			goto exit;
		}
		/*setup BT UART lines*/
		for (pin = 0; pin < ARRAY_SIZE(bt_config_power_on);
			pin++) {
			rc = gpio_tlmm_config(bt_config_power_on[pin],
					GPIO_CFG_ENABLE);
			if (rc < 0) {
				pr_err("%s: gpio_tlmm_config(%#x)=%d\n",
						__func__,
						bt_config_power_on[pin],
						rc);
				goto fail_power;
			}
		}
		/*Setup BT clocks*/
		rc = pmapp_clock_vote(id, PMAPP_CLOCK_ID_D1,
			PMAPP_CLOCK_VOTE_ON);
		if (rc < 0) {
			pr_err("Failed to vote for TCXO_D1 ON\n");
			goto fail_clock;
		}
		msleep(20);

		/*I2C config for Bahama*/
		rc = bahama_bt(1);
		if (rc < 0) {
			pr_err("%s: bahama_bt rc = %d", __func__, rc);
			goto fail_i2c;
		}
		msleep(20);
		rc = pmapp_clock_vote(id, PMAPP_CLOCK_ID_D1,
				  PMAPP_CLOCK_VOTE_PIN_CTRL);
		if (rc < 0)
			pr_err("%s:Pin Control Failed, rc = %d", __func__, rc);

	} else {
		rc = bahama_bt(0);
		if (rc < 0)
			pr_err("%s: bahama_bt rc = %d", __func__, rc);
fail_i2c:
		rc = pmapp_clock_vote(id, PMAPP_CLOCK_ID_D1,
				  PMAPP_CLOCK_VOTE_OFF);
		if (rc < 0)
			pr_err("%s: Failed to vote Off D1\n", __func__);
fail_clock:
		for (pin = 0; pin < ARRAY_SIZE(bt_config_power_off);
			pin++) {
				rc = gpio_tlmm_config(bt_config_power_off[pin],
					GPIO_CFG_ENABLE);
				if (rc < 0) {
					pr_err("%s: gpio_tlmm_config(%#x)=%d\n",
					__func__, bt_config_power_off[pin], rc);
				}
			}
fail_power:
		rc = bluetooth_switch_regulators(0);
		if (rc < 0) {
			pr_err("%s: switch_regulators : rc = %d",\
					__func__, rc);
			goto exit;
		}
	}
	return rc;
exit:
	pr_err("%s: failed with rc = %d", __func__, rc);
	return rc;
}

static int __init bt_power_init(void)
{
	int i, rc = 0;
	for (i = 0; i < ARRAY_SIZE(vregs_bahama_name); i++) {
			vregs_bahama[i] = vreg_get(NULL,
						vregs_bahama_name[i]);
			if (IS_ERR(vregs_bahama[i])) {
				pr_err("%s: vreg get %s failed (%ld)\n",
				       __func__, vregs_bahama_name[i],
				       PTR_ERR(vregs_bahama[i]));
				rc = PTR_ERR(vregs_bahama[i]);
				goto vreg_get_fail;
			}
		}

	msm_bt_power_device.dev.platform_data = &bluetooth_power;

	return rc;

vreg_get_fail:
	while (i)
		vreg_put(vregs_bahama[--i]);
	return rc;
}

static struct marimba_platform_data marimba_pdata = {
	.slave_id[SLAVE_ID_BAHAMA_FM]        = BAHAMA_SLAVE_ID_FM_ADDR,
	.slave_id[SLAVE_ID_BAHAMA_QMEMBIST]  = BAHAMA_SLAVE_ID_QMEMBIST_ADDR,
	.bahama_setup                        = msm_bahama_setup_power,
	.bahama_shutdown                     = msm_bahama_shutdown_power,
	.bahama_core_config                  = msm_bahama_core_config,
	.fm				     = &marimba_fm_pdata,
};

#endif

#if defined(CONFIG_I2C) && defined(CONFIG_GPIO_SX150X)
static struct i2c_board_info surf_core_exp_i2c_info[] __initdata = {
	{
		I2C_BOARD_INFO("sx1509q", 0x3e),
		.platform_data =  &sx150x_data[SX150X_SURF],
	},
};
static struct i2c_board_info surf_cam_exp_i2c_info[] __initdata = {
	{
		I2C_BOARD_INFO("sx1508q", 0x22),
		.platform_data	= &sx150x_data[SX150X_SURF_CAM],
	},
};
static struct i2c_board_info ffa_core_exp_i2c_info[] __initdata = {
	{
		I2C_BOARD_INFO("sx1509q", 0x3e),
		.platform_data =  &sx150x_data[SX150X_FFA],
	},
};
static struct i2c_board_info ffa_cam_exp_i2c_info[] __initdata = {
	{
		I2C_BOARD_INFO("sx1508q", 0x22),
		.platform_data	= &sx150x_data[SX150X_FFA_CAM],
	},

};
#endif
#if defined(CONFIG_BT) && defined(CONFIG_MARIMBA_CORE)
static struct i2c_board_info bahama_devices[] = {
{
	I2C_BOARD_INFO("marimba", 0xc),
	.platform_data = &marimba_pdata,
},
};
#endif

#if defined(CONFIG_I2C) && defined(CONFIG_GPIO_SX150X)
static void __init register_i2c_devices(void)
{
	if (machine_is_msm7x27a_surf()) {
		i2c_register_board_info(MSM_GSBI1_QUP_I2C_BUS_ID,
				surf_core_exp_i2c_info,
				ARRAY_SIZE(surf_core_exp_i2c_info));
		i2c_register_board_info(MSM_GSBI0_QUP_I2C_BUS_ID,
				surf_cam_exp_i2c_info,
				ARRAY_SIZE(surf_cam_exp_i2c_info));
#if defined(CONFIG_BT) && defined(CONFIG_MARIMBA_CORE)
		i2c_register_board_info(MSM_GSBI1_QUP_I2C_BUS_ID,
				bahama_devices,
				ARRAY_SIZE(bahama_devices));
#endif

	} else if (machine_is_msm7x27a_ffa()) {
		i2c_register_board_info(MSM_GSBI1_QUP_I2C_BUS_ID,
				ffa_core_exp_i2c_info,
				ARRAY_SIZE(ffa_core_exp_i2c_info));
		i2c_register_board_info(MSM_GSBI0_QUP_I2C_BUS_ID,
				ffa_cam_exp_i2c_info,
				ARRAY_SIZE(ffa_cam_exp_i2c_info));
#if defined(CONFIG_BT) && defined(CONFIG_MARIMBA_CORE)
		i2c_register_board_info(MSM_GSBI0_QUP_I2C_BUS_ID,
				bahama_devices,
				ARRAY_SIZE(bahama_devices));
#endif
	}
}
#endif

static struct msm_gpio qup_i2c_gpios_io[] = {
	{ GPIO_CFG(60, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
		"qup_scl" },
	{ GPIO_CFG(61, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
		"qup_sda" },
	{ GPIO_CFG(131, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
		"qup_scl" },
	{ GPIO_CFG(132, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
		"qup_sda" },
};

static struct msm_gpio qup_i2c_gpios_hw[] = {
	{ GPIO_CFG(60, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
		"qup_scl" },
	{ GPIO_CFG(61, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
		"qup_sda" },
	{ GPIO_CFG(131, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
		"qup_scl" },
	{ GPIO_CFG(132, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
		"qup_sda" },
};

static void gsbi_qup_i2c_gpio_config(int adap_id, int config_type)
{
	int rc;

	if (adap_id < 0 || adap_id > 1)
		return;

	/* Each adapter gets 2 lines from the table */
	if (config_type)
		rc = msm_gpios_request_enable(&qup_i2c_gpios_hw[adap_id*2], 2);
	else
		rc = msm_gpios_request_enable(&qup_i2c_gpios_io[adap_id*2], 2);
	if (rc < 0)
		pr_err("QUP GPIO request/enable failed: %d\n", rc);
}

static struct msm_i2c_platform_data msm_gsbi0_qup_i2c_pdata = {
	.clk_freq		= 100000,
	.clk			= "gsbi_qup_clk",
	.pclk			= "gsbi_qup_pclk",
	.msm_i2c_config_gpio	= gsbi_qup_i2c_gpio_config,
};

static struct msm_i2c_platform_data msm_gsbi1_qup_i2c_pdata = {
	.clk_freq		= 100000,
	.clk			= "gsbi_qup_clk",
	.pclk			= "gsbi_qup_pclk",
	.msm_i2c_config_gpio	= gsbi_qup_i2c_gpio_config,
};

#ifdef CONFIG_ARCH_MSM7X27A
#define MSM_PMEM_MDP_SIZE       0x1DD1000
#define MSM_PMEM_ADSP_SIZE      0x1000000
#define MSM_FB_SIZE             0x195000
#endif

static char *usb_functions_default[] = {
	"diag",
	"modem",
	"nmea",
	"rmnet",
	"usb_mass_storage",
};

static char *usb_functions_default_adb[] = {
	"diag",
	"adb",
	"modem",
	"nmea",
	"rmnet",
	"usb_mass_storage",
};

static char *usb_functions_rndis[] = {
	"rndis",
};

static char *usb_functions_rndis_adb[] = {
	"rndis",
	"adb",
};

static char *usb_functions_all[] = {
#ifdef CONFIG_USB_ANDROID_RNDIS
	"rndis",
#endif
#ifdef CONFIG_USB_ANDROID_DIAG
	"diag",
#endif
	"adb",
#ifdef CONFIG_USB_F_SERIAL
	"modem",
	"nmea",
#endif
#ifdef CONFIG_USB_ANDROID_RMNET
	"rmnet",
#endif
	"usb_mass_storage",
};

static struct android_usb_product usb_products[] = {
	{
		.product_id     = 0x9026,
		.num_functions	= ARRAY_SIZE(usb_functions_default),
		.functions      = usb_functions_default,
	},
	{
		.product_id	= 0x9025,
		.num_functions	= ARRAY_SIZE(usb_functions_default_adb),
		.functions	= usb_functions_default_adb,
	},
	{
		.product_id	= 0xf00e,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis),
		.functions	= usb_functions_rndis,
	},
	{
		.product_id	= 0x9024,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis_adb),
		.functions	= usb_functions_rndis_adb,
	},
};

static struct usb_mass_storage_platform_data mass_storage_pdata = {
	.nluns		= 1,
	.vendor		= "Qualcomm Incorporated",
	.product	= "Mass storage",
	.release	= 0x0100,

};

static struct platform_device usb_mass_storage_device = {
	.name	= "usb_mass_storage",
	.id	= -1,
	.dev	= {
		.platform_data = &mass_storage_pdata,
	},
};

static struct android_usb_platform_data android_usb_pdata = {
	.vendor_id	= 0x05C6,
	.product_id	= 0x9026,
	.version	= 0x0100,
	.product_name	= "Qualcomm HSUSB Device",
	.manufacturer_name = "Qualcomm Incorporated",
	.num_products = ARRAY_SIZE(usb_products),
	.products = usb_products,
	.num_functions = ARRAY_SIZE(usb_functions_all),
	.functions = usb_functions_all,
	.serial_number = "1234567890ABCDEF",
};

static struct platform_device android_usb_device = {
	.name	= "android_usb",
	.id	= -1,
	.dev	= {
		.platform_data = &android_usb_pdata,
	},
};

static struct usb_ether_platform_data rndis_pdata = {
	.vendorID	= 0x05C6,
	.vendorDescr	= "Qualcomm Incorporated",
};

static struct platform_device rndis_device = {
	.name	= "rndis",
	.id	= -1,
	.dev	= {
		.platform_data = &rndis_pdata,
	},
};

static int __init board_serialno_setup(char *serialno)
{
	int i;
	char *src = serialno;

	/* create a fake MAC address from our serial number.
	 * first byte is 0x02 to signify locally administered.
	 */
	rndis_pdata.ethaddr[0] = 0x02;
	for (i = 0; *src; i++) {
		/* XOR the USB serial across the remaining bytes */
		rndis_pdata.ethaddr[i % (ETH_ALEN - 1) + 1] ^= *src++;
	}

	android_usb_pdata.serial_number = serialno;
	return 1;
}
__setup("androidboot.serialno=", board_serialno_setup);

#ifdef CONFIG_USB_EHCI_MSM_72K
static int  msm_hsusb_vbus_init(int on)
{
	int rc = 0;
	rc = gpio_request(GPIO_HOST_VBUS_EN, "i2c_host_vbus_en");
	if (rc < 0) {
		pr_err("failed to request %d GPIO\n", GPIO_HOST_VBUS_EN);
		return rc;
	}
	gpio_direction_output(GPIO_HOST_VBUS_EN, 1);
	return rc;
}
static void msm_hsusb_vbus_power(unsigned phy_info, int on)
{
	gpio_set_value(GPIO_HOST_VBUS_EN, !!on);
}

static struct msm_usb_host_platform_data msm_usb_host_pdata = {
	.phy_info       = (USB_PHY_INTEGRATED | USB_PHY_MODEL_45NM),
	.vbus_init	= msm_hsusb_vbus_init,
	.vbus_power	= msm_hsusb_vbus_power,
};

static void __init msm7x2x_init_host(void)
{
	msm_add_host(0, &msm_usb_host_pdata);
}
#endif

#ifdef CONFIG_USB_MSM_OTG_72K
static int hsusb_rpc_connect(int connect)
{
	if (connect)
		return msm_hsusb_rpc_connect();
	else
		return msm_hsusb_rpc_close();
}

static struct vreg *vreg_3p3;
static int msm_hsusb_ldo_init(int init)
{
	if (init) {
		vreg_3p3 = vreg_get(NULL, "usb");
		if (IS_ERR(vreg_3p3))
			return PTR_ERR(vreg_3p3);
	} else
		vreg_put(vreg_3p3);

	return 0;
}

static int msm_hsusb_ldo_enable(int enable)
{
	static int ldo_status;

	if (!vreg_3p3 || IS_ERR(vreg_3p3))
		return -ENODEV;

	if (ldo_status == enable)
		return 0;

	ldo_status = enable;

	if (enable)
		return vreg_enable(vreg_3p3);

	return vreg_disable(vreg_3p3);
}

static struct msm_otg_platform_data msm_otg_pdata = {
#ifndef CONFIG_USB_EHCI_MSM_72K
	.pmic_vbus_notif_init	 = msm_hsusb_pmic_notif_init,
#else
	.vbus_power		 = msm_hsusb_vbus_power,
#endif
	.rpc_connect		 = hsusb_rpc_connect,
	.core_clk		 = 1,
	.pemp_level		 = PRE_EMPHASIS_WITH_20_PERCENT,
	.cdr_autoreset		 = CDR_AUTO_RESET_DISABLE,
	.drv_ampl		 = HS_DRV_AMPLITUDE_DEFAULT,
	.se1_gating		 = SE1_GATING_DISABLE,
	.ldo_init		 = msm_hsusb_ldo_init,
	.ldo_enable		 = msm_hsusb_ldo_enable,
};
#endif

static struct resource smc91x_resources[] = {
	[0] = {
		.start = 0x90000300,
		.end   = 0x900003ff,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = MSM_GPIO_TO_INT(4),
		.end   = MSM_GPIO_TO_INT(4),
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device smc91x_device = {
	.name           = "smc91x",
	.id             = 0,
	.num_resources  = ARRAY_SIZE(smc91x_resources),
	.resource       = smc91x_resources,
};

#if (defined(CONFIG_MMC_MSM_SDC1_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC2_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC3_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC4_SUPPORT))

static unsigned long vreg_sts, gpio_sts;
static struct vreg *vreg_mmc;
static struct vreg *vreg_emmc;

struct sdcc_vreg {
	struct vreg *vreg_data;
	unsigned level;
};

static struct sdcc_vreg sdcc_vreg_data[4];

struct sdcc_gpio {
	struct msm_gpio *cfg_data;
	uint32_t size;
	struct msm_gpio *sleep_cfg_data;
};

static struct msm_gpio sdc1_cfg_data[] = {
	{GPIO_CFG(51, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
								"sdc1_dat_3"},
	{GPIO_CFG(52, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
								"sdc1_dat_2"},
	{GPIO_CFG(53, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
								"sdc1_dat_1"},
	{GPIO_CFG(54, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
								"sdc1_dat_0"},
	{GPIO_CFG(55, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
								"sdc1_cmd"},
	{GPIO_CFG(56, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
								"sdc1_clk"},
};

static struct msm_gpio sdc2_cfg_data[] = {
	{GPIO_CFG(62, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
								"sdc2_clk"},
	{GPIO_CFG(63, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
								"sdc2_cmd"},
	{GPIO_CFG(64, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
								"sdc2_dat_3"},
	{GPIO_CFG(65, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
								"sdc2_dat_2"},
	{GPIO_CFG(66, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
								"sdc2_dat_1"},
	{GPIO_CFG(67, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
								"sdc2_dat_0"},
};

static struct msm_gpio sdc2_sleep_cfg_data[] = {
	{GPIO_CFG(62, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
								"sdc2_clk"},
	{GPIO_CFG(63, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
								"sdc2_cmd"},
	{GPIO_CFG(64, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
								"sdc2_dat_3"},
	{GPIO_CFG(65, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
								"sdc2_dat_2"},
	{GPIO_CFG(66, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
								"sdc2_dat_1"},
	{GPIO_CFG(67, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
								"sdc2_dat_0"},
};
static struct msm_gpio sdc3_cfg_data[] = {
	{GPIO_CFG(88, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
								"sdc3_clk"},
	{GPIO_CFG(89, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
								"sdc3_cmd"},
	{GPIO_CFG(90, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
								"sdc3_dat_3"},
	{GPIO_CFG(91, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
								"sdc3_dat_2"},
	{GPIO_CFG(92, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
								"sdc3_dat_1"},
	{GPIO_CFG(93, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
								"sdc3_dat_0"},
#ifdef CONFIG_MMC_MSM_SDC3_8_BIT_SUPPORT
	{GPIO_CFG(19, 3, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
								"sdc3_dat_7"},
	{GPIO_CFG(20, 3, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
								"sdc3_dat_6"},
	{GPIO_CFG(21, 3, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
								"sdc3_dat_5"},
	{GPIO_CFG(108, 3, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
								"sdc3_dat_4"},
#endif
};

static struct msm_gpio sdc4_cfg_data[] = {
	{GPIO_CFG(19, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
								"sdc4_dat_3"},
	{GPIO_CFG(20, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
								"sdc4_dat_2"},
	{GPIO_CFG(21, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
								"sdc4_dat_1"},
	{GPIO_CFG(107, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
								"sdc4_cmd"},
	{GPIO_CFG(108, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
								"sdc4_dat_0"},
	{GPIO_CFG(109, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
								"sdc4_clk"},
};

static struct sdcc_gpio sdcc_cfg_data[] = {
	{
		.cfg_data = sdc1_cfg_data,
		.size = ARRAY_SIZE(sdc1_cfg_data),
	},
	{
		.cfg_data = sdc2_cfg_data,
		.size = ARRAY_SIZE(sdc2_cfg_data),
		.sleep_cfg_data = sdc2_sleep_cfg_data,
	},
	{
		.cfg_data = sdc3_cfg_data,
		.size = ARRAY_SIZE(sdc3_cfg_data),
	},
	{
		.cfg_data = sdc4_cfg_data,
		.size = ARRAY_SIZE(sdc4_cfg_data),
	},
};

static int msm_sdcc_setup_gpio(int dev_id, unsigned int enable)
{
	int rc = 0;
	struct sdcc_gpio *curr;

	curr = &sdcc_cfg_data[dev_id - 1];
	if (!(test_bit(dev_id, &gpio_sts)^enable))
		return rc;

	if (enable) {
		set_bit(dev_id, &gpio_sts);
		rc = msm_gpios_request_enable(curr->cfg_data, curr->size);
		if (rc)
			pr_err("%s: Failed to turn on GPIOs for slot %d\n",
					__func__,  dev_id);
	} else {
		clear_bit(dev_id, &gpio_sts);
		if (curr->sleep_cfg_data) {
			rc = msm_gpios_enable(curr->sleep_cfg_data, curr->size);
			msm_gpios_free(curr->sleep_cfg_data, curr->size);
			return rc;
		}
		msm_gpios_disable_free(curr->cfg_data, curr->size);
	}
	return rc;
}

static int msm_sdcc_setup_vreg(int dev_id, unsigned int enable)
{
	int rc = 0;
	struct sdcc_vreg *curr;

	curr = &sdcc_vreg_data[dev_id - 1];

	if (!(test_bit(dev_id, &vreg_sts)^enable))
		return rc;

	if (enable) {
		set_bit(dev_id, &vreg_sts);
		rc = vreg_set_level(curr->vreg_data, curr->level);
		if (rc)
			pr_err("%s: vreg_set_level() = %d\n", __func__, rc);

		rc = vreg_enable(curr->vreg_data);
		if (rc)
			pr_err("%s: vreg_enable() = %d\n", __func__, rc);
	} else {
		clear_bit(dev_id, &vreg_sts);
		rc = vreg_disable(curr->vreg_data);
		if (rc)
			pr_err("%s: vreg_disable() = %d\n", __func__, rc);
	}
	return rc;
}

static uint32_t msm_sdcc_setup_power(struct device *dv, unsigned int vdd)
{
	int rc = 0;
	struct platform_device *pdev;

	pdev = container_of(dv, struct platform_device, dev);

	rc = msm_sdcc_setup_gpio(pdev->id, !!vdd);
	if (rc)
		goto out;

	rc = msm_sdcc_setup_vreg(pdev->id, !!vdd);
out:
	return rc;
}

#define GPIO_SDC1_HW_DET 85

#if defined(CONFIG_MMC_MSM_SDC1_SUPPORT) \
	&& defined(CONFIG_MMC_MSM_CARD_HW_DETECTION)
static unsigned int msm7x2xa_sdcc_slot_status(struct device *dev)
{
	int status;

	status = gpio_tlmm_config(GPIO_CFG(GPIO_SDC1_HW_DET, 2, GPIO_CFG_INPUT,
			GPIO_CFG_PULL_UP, GPIO_CFG_8MA), GPIO_CFG_ENABLE);
	if (status)
		pr_err("%s:Failed to configure tlmm for GPIO %d\n", __func__,
				GPIO_SDC1_HW_DET);

	status = gpio_request(GPIO_SDC1_HW_DET, "SD_HW_Detect");
	if (status) {
		pr_err("%s:Failed to request GPIO %d\n", __func__,
				GPIO_SDC1_HW_DET);
	} else {
		status = gpio_direction_input(GPIO_SDC1_HW_DET);
		if (!status)
			status = gpio_get_value(GPIO_SDC1_HW_DET);
		gpio_free(GPIO_SDC1_HW_DET);
	}
	return status;
}
#endif

#ifdef CONFIG_MMC_MSM_SDC1_SUPPORT
static struct mmc_platform_data sdc1_plat_data = {
	.ocr_mask	= MMC_VDD_28_29,
	.translate_vdd  = msm_sdcc_setup_power,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
	.msmsdcc_fmin	= 144000,
	.msmsdcc_fmid	= 24576000,
	.msmsdcc_fmax	= 49152000,
#ifdef CONFIG_MMC_MSM_CARD_HW_DETECTION
	.status      = msm7x2xa_sdcc_slot_status,
	.status_irq  = MSM_GPIO_TO_INT(GPIO_SDC1_HW_DET),
	.irq_flags   = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
#endif
};
#endif

#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
static struct mmc_platform_data sdc2_plat_data = {
	.ocr_mask	= MMC_VDD_28_29,
	.translate_vdd  = msm_sdcc_setup_power,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
#ifdef CONFIG_MMC_MSM_SDIO_SUPPORT
	.sdiowakeup_irq = MSM_GPIO_TO_INT(66),
#endif
	.msmsdcc_fmin	= 144000,
	.msmsdcc_fmid	= 24576000,
	.msmsdcc_fmax	= 49152000,
#ifdef CONFIG_MMC_MSM_SDC2_DUMMY52_REQUIRED
	.dummy52_required = 1,
#endif
};
#endif

#ifdef CONFIG_MMC_MSM_SDC3_SUPPORT
static struct mmc_platform_data sdc3_plat_data = {
	.ocr_mask	= MMC_VDD_28_29,
	.translate_vdd  = msm_sdcc_setup_power,
#ifdef CONFIG_MMC_MSM_SDC3_8_BIT_SUPPORT
	.mmc_bus_width  = MMC_CAP_8_BIT_DATA,
#else
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
#endif
	.msmsdcc_fmin	= 144000,
	.msmsdcc_fmid	= 24576000,
	.msmsdcc_fmax	= 49152000,
	.nonremovable	= 1,
};
#endif

#if (defined(CONFIG_MMC_MSM_SDC4_SUPPORT)\
		&& !defined(CONFIG_MMC_MSM_SDC3_8_BIT_SUPPORT))
static struct mmc_platform_data sdc4_plat_data = {
	.ocr_mask	= MMC_VDD_28_29,
	.translate_vdd  = msm_sdcc_setup_power,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
	.msmsdcc_fmin	= 144000,
	.msmsdcc_fmid	= 24576000,
	.msmsdcc_fmax	= 49152000,
};
#endif
#endif

#ifdef CONFIG_SERIAL_MSM_HS
static struct msm_serial_hs_platform_data msm_uart_dm1_pdata = {
	.inject_rx_on_wakeup	= 1,
	.rx_to_inject		= 0xFD,
};
#endif
static struct msm_pm_platform_data msm7x27a_pm_data[MSM_PM_SLEEP_MODE_NR] = {
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE] = {
					.supported = 1,
					.suspend_enabled = 1,
					.idle_enabled = 1,
					.latency = 16000,
					.residency = 20000,
	},
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN] = {
					.supported = 1,
					.suspend_enabled = 1,
					.idle_enabled = 1,
					.latency = 12000,
					.residency = 20000,
	},
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT] = {
					.supported = 1,
					.suspend_enabled = 1,
					.idle_enabled = 0,
					.latency = 2000,
					.residency = 0,
	},
	[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT] = {
					.supported = 1,
					.suspend_enabled = 1,
					.idle_enabled = 1,
					.latency = 2,
					.residency = 0,
	},
};

static struct android_pmem_platform_data android_pmem_adsp_pdata = {
	.name = "pmem_adsp",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 0,
	.memory_type = MEMTYPE_EBI1,
};

static struct platform_device android_pmem_adsp_device = {
	.name = "android_pmem",
	.id = 1,
	.dev = { .platform_data = &android_pmem_adsp_pdata },
};

static unsigned pmem_mdp_size = MSM_PMEM_MDP_SIZE;
static int __init pmem_mdp_size_setup(char *p)
{
	pmem_mdp_size = memparse(p, NULL);
	return 0;
}

early_param("pmem_mdp_size", pmem_mdp_size_setup);

static unsigned pmem_adsp_size = MSM_PMEM_ADSP_SIZE;
static int __init pmem_adsp_size_setup(char *p)
{
	pmem_adsp_size = memparse(p, NULL);
	return 0;
}

early_param("pmem_adsp_size", pmem_adsp_size_setup);

static unsigned fb_size = MSM_FB_SIZE;
static int __init fb_size_setup(char *p)
{
	fb_size = memparse(p, NULL);
	return 0;
}

early_param("fb_size", fb_size_setup);


#define LCDC_CONFIG_PROC          21
#define LCDC_UN_CONFIG_PROC       22
#define LCDC_API_PROG             0x30000066
#define LCDC_API_VERS             0x00010001

static struct msm_rpc_endpoint *lcdc_ep;

static int msm_fb_lcdc_config(int on)
{
	int rc = 0;
	struct rpc_request_hdr hdr;

	if (on)
		pr_info("lcdc config\n");
	else
		pr_info("lcdc un-config\n");

	lcdc_ep = msm_rpc_connect_compatible(LCDC_API_PROG, LCDC_API_VERS, 0);

	if (IS_ERR(lcdc_ep)) {
		printk(KERN_ERR "%s: msm_rpc_connect failed! rc = %ld\n",
			__func__, PTR_ERR(lcdc_ep));
		return -EINVAL;
	}

	rc = msm_rpc_call(lcdc_ep,
		(on) ? LCDC_CONFIG_PROC : LCDC_UN_CONFIG_PROC,
		&hdr, sizeof(hdr), 5 * HZ);

	if (rc)
		printk(KERN_ERR
			"%s: msm_rpc_call failed! rc = %d\n", __func__, rc);

	msm_rpc_close(lcdc_ep);
	return rc;
}

static uint32_t lcdc_gpio_initialized;

static void lcdc_toshiba_gpio_init(void)
{
	if (!lcdc_gpio_initialized) {
		if (gpio_request(GPIO_SPI_CLK, "spi_clk")) {
			pr_err("failed to request gpio spi_clk\n");
			goto fail_gpio;
		}
		if (gpio_request(GPIO_SPI_CS0_N, "spi_cs")) {
			pr_err("failed to request gpio spi_cs0_N\n");
			goto fail_gpio;
		}
		if (gpio_request(GPIO_SPI_MOSI, "spi_mosi")) {
			pr_err("failed to request gpio spi_mosi\n");
			goto fail_gpio;
		}
		if (gpio_request(GPIO_SPI_MISO, "spi_miso")) {
			pr_err("failed to request gpio spi_miso\n");
			goto fail_gpio;
		}
		if (gpio_request(GPIO_DISPLAY_PWR_EN, "gpio_disp_pwr")) {
			pr_err("failed to request gpio_disp_pwr\n");
			goto fail_gpio;
		}
		if (gpio_request(GPIO_BACKLIGHT_EN, "gpio_bkl_en")) {
			pr_err("failed to request gpio_bkl_en\n");
			goto fail_gpio;
		}
		lcdc_gpio_initialized = 1;
		return;
	}
fail_gpio:
	lcdc_gpio_initialized = 0;
}

static uint32_t lcdc_gpio_table[] = {
	GPIO_SPI_CLK,
	GPIO_SPI_CS0_N,
	GPIO_SPI_MOSI,
	GPIO_DISPLAY_PWR_EN,
	GPIO_BACKLIGHT_EN,
	GPIO_SPI_MISO,
};

static void config_lcdc_gpio_table(uint32_t *table, int len, unsigned enable)
{
	int n;

	if (lcdc_gpio_initialized) {
		/* All are IO Expander GPIOs */
		for (n = 0; n < (len - 1); n++)
			gpio_direction_output(table[n], 1);
	}
}

static void lcdc_toshiba_config_gpios(int enable)
{
	config_lcdc_gpio_table(lcdc_gpio_table,
		ARRAY_SIZE(lcdc_gpio_table), enable);
}

static int msm_fb_lcdc_power_save(int on)
{
	/* struct vreg *vreg[ARRAY_SIZE(msm_fb_lcdc_vreg)]; */
	int rc = 0;

	/* Doing the init of the LCDC GPIOs very late as they are from
		an I2C-controlled IO Expander */
	lcdc_toshiba_gpio_init();

	if (lcdc_gpio_initialized) {
		gpio_set_value_cansleep(GPIO_DISPLAY_PWR_EN, on);
		gpio_set_value_cansleep(GPIO_BACKLIGHT_EN, on);
	}

	pmapp_disp_backlight_init();
	rc = pmapp_disp_backlight_set_brightness(100);

	return 0;
}

static struct lcdc_platform_data lcdc_pdata = {
	.lcdc_gpio_config = msm_fb_lcdc_config,
	.lcdc_power_save   = msm_fb_lcdc_power_save,
};

static int lcd_panel_spi_gpio_num[] = {
		GPIO_SPI_MOSI,  /* spi_sdi */
		GPIO_SPI_MISO,  /* spi_sdoi */
		GPIO_SPI_CLK,   /* spi_clk */
		GPIO_SPI_CS0_N, /* spi_cs  */
};

static struct msm_panel_common_pdata lcdc_toshiba_panel_data = {
	.panel_config_gpio = lcdc_toshiba_config_gpios,
	.gpio_num	  = lcd_panel_spi_gpio_num,
};

static struct platform_device lcdc_toshiba_panel_device = {
	.name   = "lcdc_toshiba_fwvga_pt",
	.id     = 0,
	.dev    = {
		.platform_data = &lcdc_toshiba_panel_data,
	}
};

static struct resource msm_fb_resources[] = {
	{
		.flags  = IORESOURCE_DMA,
	}
};

static int msm_fb_detect_panel(const char *name)
{
	int ret = -EPERM;

	if (machine_is_msm7x27a_surf()) {
		if (!strncmp(name, "lcdc_toshiba_fwvga_pt", 21))
			ret = 0;
	} else {
		ret = -ENODEV;
	}

	return ret;
}

static struct msm_fb_platform_data msm_fb_pdata = {
	.detect_client = msm_fb_detect_panel,
};

static struct platform_device msm_fb_device = {
	.name   = "msm_fb",
	.id     = 0,
	.num_resources  = ARRAY_SIZE(msm_fb_resources),
	.resource       = msm_fb_resources,
	.dev    = {
		.platform_data = &msm_fb_pdata,
	}
};

#ifdef CONFIG_FB_MSM_MIPI_DSI
static struct platform_device mipi_dsi_renesas_panel_device = {
	.name = "mipi_renesas",
	.id = 0,
};
#endif

static void __init msm7x27a_init_mmc(void)
{
	vreg_emmc = vreg_get(NULL, "emmc");
	if (IS_ERR(vreg_emmc)) {
		pr_err("%s: vreg get failed (%ld)\n",
				__func__, PTR_ERR(vreg_emmc));
		return;
	}

	vreg_mmc = vreg_get(NULL, "mmc");
	if (IS_ERR(vreg_mmc)) {
		pr_err("%s: vreg get failed (%ld)\n",
				__func__, PTR_ERR(vreg_mmc));
		return;
	}

	/* Micro-SD slot */
#ifdef CONFIG_MMC_MSM_SDC1_SUPPORT
	sdcc_vreg_data[0].vreg_data = vreg_mmc;
	sdcc_vreg_data[0].level = 2850;
	msm_add_sdcc(1, &sdc1_plat_data);
#endif
	/* SDIO WLAN slot */
#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
	sdcc_vreg_data[1].vreg_data = vreg_mmc;
	sdcc_vreg_data[1].level = 2850;
	msm_add_sdcc(2, &sdc2_plat_data);
#endif
	/* eMMC slot */
#ifdef CONFIG_MMC_MSM_SDC3_SUPPORT
	sdcc_vreg_data[2].vreg_data = vreg_emmc;
	sdcc_vreg_data[2].level = 1800;
	msm_add_sdcc(3, &sdc3_plat_data);
#endif
	/* Not Used */
#if (defined(CONFIG_MMC_MSM_SDC4_SUPPORT)\
		&& !defined(CONFIG_MMC_MSM_SDC3_8_BIT_SUPPORT))
	sdcc_vreg_data[3].vreg_data = vreg_mmc;
	sdcc_vreg_data[3].level = 2850;
	msm_add_sdcc(4, &sdc4_plat_data);
#endif
}
#define SND(desc, num) { .name = #desc, .id = num }
static struct snd_endpoint snd_endpoints_list[] = {
	SND(HANDSET, 0),
	SND(MONO_HEADSET, 2),
	SND(HEADSET, 3),
	SND(SPEAKER, 6),
	SND(TTY_HEADSET, 8),
	SND(TTY_VCO, 9),
	SND(TTY_HCO, 10),
	SND(BT, 12),
	SND(IN_S_SADC_OUT_HANDSET, 16),
	SND(IN_S_SADC_OUT_SPEAKER_PHONE, 25),
	SND(CURRENT, 27),
};
#undef SND

static struct msm_snd_endpoints msm_device_snd_endpoints = {
	.endpoints = snd_endpoints_list,
	.num = sizeof(snd_endpoints_list) / sizeof(struct snd_endpoint)
};

static struct platform_device msm_device_snd = {
	.name = "msm_snd",
	.id = -1,
	.dev    = {
		.platform_data = &msm_device_snd_endpoints
	},
};

#define DEC0_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))
#define DEC1_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))
#define DEC2_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))
#define DEC3_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))
#define DEC4_FORMAT (1<<MSM_ADSP_CODEC_MIDI)

static unsigned int dec_concurrency_table[] = {
	/* Audio LP */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DMA)), 0,
	0, 0, 0,

	/* Concurrency 1 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC4_FORMAT),

	 /* Concurrency 2 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC4_FORMAT),

	/* Concurrency 3 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC4_FORMAT),

	/* Concurrency 4 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC4_FORMAT),

	/* Concurrency 5 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC4_FORMAT),

	/* Concurrency 6 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	0, 0, 0, 0,

	/* Concurrency 7 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC4_FORMAT),
};

#define DEC_INFO(name, queueid, decid, nr_codec) { .module_name = name, \
	.module_queueid = queueid, .module_decid = decid, \
	.nr_codec_support = nr_codec}

static struct msm_adspdec_info dec_info_list[] = {
	DEC_INFO("AUDPLAY0TASK", 13, 0, 11), /* AudPlay0BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY1TASK", 14, 1, 11),  /* AudPlay1BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY2TASK", 15, 2, 11),  /* AudPlay2BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY3TASK", 16, 3, 11),  /* AudPlay3BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY4TASK", 17, 4, 1),  /* AudPlay4BitStreamCtrlQueue */
};

static struct msm_adspdec_database msm_device_adspdec_database = {
	.num_dec = ARRAY_SIZE(dec_info_list),
	.num_concurrency_support = (ARRAY_SIZE(dec_concurrency_table) / \
					ARRAY_SIZE(dec_info_list)),
	.dec_concurrency_table = dec_concurrency_table,
	.dec_info_list = dec_info_list,
};

static struct platform_device msm_device_adspdec = {
	.name = "msm_adspdec",
	.id = -1,
	.dev    = {
		.platform_data = &msm_device_adspdec_database
	},
};

static struct android_pmem_platform_data android_pmem_audio_pdata = {
	.name = "pmem_audio",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 0,
	.memory_type = MEMTYPE_EBI1,
};

static struct platform_device android_pmem_audio_device = {
	.name = "android_pmem",
	.id = 2,
	.dev = { .platform_data = &android_pmem_audio_pdata },
};

static struct android_pmem_platform_data android_pmem_pdata = {
	.name = "pmem",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 1,
	.memory_type = MEMTYPE_EBI1,
};
static struct platform_device android_pmem_device = {
	.name = "android_pmem",
	.id = 0,
	.dev = { .platform_data = &android_pmem_pdata },
};

static u32 msm_calculate_batt_capacity(u32 current_voltage);

static struct msm_psy_batt_pdata msm_psy_batt_data = {
	.voltage_min_design     = 2800,
	.voltage_max_design     = 4300,
	.avail_chg_sources      = AC_CHG | USB_CHG ,
	.batt_technology        = POWER_SUPPLY_TECHNOLOGY_LION,
	.calculate_capacity     = &msm_calculate_batt_capacity,
};

static u32 msm_calculate_batt_capacity(u32 current_voltage)
{
	u32 low_voltage	 = msm_psy_batt_data.voltage_min_design;
	u32 high_voltage = msm_psy_batt_data.voltage_max_design;

	return (current_voltage - low_voltage) * 100
			/ (high_voltage - low_voltage);
}

static struct platform_device msm_batt_device = {
	.name               = "msm-battery",
	.id                 = -1,
	.dev.platform_data  = &msm_psy_batt_data,
};

static struct smsc911x_platform_config smsc911x_config = {
	.irq_polarity	= SMSC911X_IRQ_POLARITY_ACTIVE_HIGH,
	.irq_type	= SMSC911X_IRQ_TYPE_PUSH_PULL,
	.flags		= SMSC911X_USE_16BIT,
};

static struct resource smsc911x_resources[] = {
	[0] = {
		.start	= 0x90000000,
		.end	= 0x90007fff,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= MSM_GPIO_TO_INT(48),
		.end	= MSM_GPIO_TO_INT(48),
		.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL,
	},
};

static struct platform_device smsc911x_device = {
	.name		= "smsc911x",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(smsc911x_resources),
	.resource	= smsc911x_resources,
	.dev		= {
		.platform_data	= &smsc911x_config,
	},
};

static struct msm_gpio smsc911x_gpios[] = {
	{ GPIO_CFG(48, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_6MA),
							 "smsc911x_irq"  },
	{ GPIO_CFG(49, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_6MA),
							 "eth_fifo_sel" },
};

#define ETH_FIFO_SEL_GPIO	49
static void msm7x27a_cfg_smsc911x(void)
{
	int res;

	res = msm_gpios_request_enable(smsc911x_gpios,
				 ARRAY_SIZE(smsc911x_gpios));
	if (res) {
		pr_err("%s: unable to enable gpios for SMSC911x\n", __func__);
		return;
	}

	/* ETH_FIFO_SEL */
	res = gpio_direction_output(ETH_FIFO_SEL_GPIO, 0);
	if (res) {
		pr_err("%s: unable to get direction for gpio %d\n", __func__,
							 ETH_FIFO_SEL_GPIO);
		msm_gpios_disable_free(smsc911x_gpios,
						 ARRAY_SIZE(smsc911x_gpios));
		return;
	}
	gpio_set_value(ETH_FIFO_SEL_GPIO, 0);
}

#ifdef CONFIG_MSM_CAMERA
static uint32_t camera_off_gpio_table[] = {
	GPIO_CFG(61, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
	GPIO_CFG(60, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
	GPIO_CFG(15, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
};

static uint32_t camera_on_gpio_table[] = {
	GPIO_CFG(61, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
	GPIO_CFG(60, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
	GPIO_CFG(15, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
};

#ifdef CONFIG_MSM_CAMERA_FLASH
static struct msm_camera_sensor_flash_src msm_flash_src = {
	.flash_sr_type = MSM_CAMERA_FLASH_SRC_CURRENT_DRIVER,
	._fsrc.current_driver_src.led1 = GPIO_SURF_CAM_GP_LED_EN1,
	._fsrc.current_driver_src.led2 = GPIO_SURF_CAM_GP_LED_EN2,
};
#endif

static struct vreg *vreg_gp2;
static struct vreg *vreg_gp3;
static void msm_camera_vreg_config(int vreg_en)
{
	int rc;

	if (vreg_gp2 == NULL) {
		vreg_gp2 = vreg_get(NULL, "gp2");
		if (IS_ERR(vreg_gp2)) {
			pr_err("%s: vreg_get(%s) failed (%ld)\n",
				__func__, "gp2", PTR_ERR(vreg_gp2));
			return;
		}

		rc = vreg_set_level(vreg_gp2, 2850);
		if (rc) {
			pr_err("%s: GP2 set_level failed (%d)\n",
				__func__, rc);
		}
	}

	if (vreg_gp3 == NULL) {
		vreg_gp3 = vreg_get(NULL, "usb2");
		if (IS_ERR(vreg_gp3)) {
			pr_err("%s: vreg_get(%s) failed (%ld)\n",
				__func__, "gp3", PTR_ERR(vreg_gp3));
			return;
		}

		rc = vreg_set_level(vreg_gp3, 1800);
		if (rc) {
			pr_err("%s: GP3 set level failed (%d)\n",
				__func__, rc);
		}
	}

	if (vreg_en) {
		rc = vreg_enable(vreg_gp2);
		if (rc) {
			pr_err("%s: GP2 enable failed (%d)\n",
				__func__, rc);
		}

		rc = vreg_enable(vreg_gp3);
		if (rc) {
			pr_err("%s: GP3 enable failed (%d)\n",
				__func__, rc);
		}
	} else {
		rc = vreg_disable(vreg_gp2);
		if (rc) {
			pr_err("%s: GP2 disable failed (%d)\n",
				__func__, rc);
		}

		rc = vreg_disable(vreg_gp3);
		if (rc) {
			pr_err("%s: GP3 disable failed (%d)\n",
				__func__, rc);
		}
	}
}

static int config_gpio_table(uint32_t *table, int len)
{
	int rc = 0, i = 0;

	for (i = 0; i < len; i++) {
		rc = gpio_tlmm_config(table[i], GPIO_CFG_ENABLE);
		if (rc) {
			pr_err("%s not able to get gpio\n", __func__);
			for (i--; i >= 0; i--)
				gpio_tlmm_config(camera_off_gpio_table[i],
							GPIO_CFG_ENABLE);
			break;
		}
	}
	return rc;
}

static struct msm_camera_sensor_info msm_camera_sensor_s5k4e1_data;
static struct msm_camera_sensor_info msm_camera_sensor_ov9726_data;
static int config_camera_on_gpios_rear(void)
{
	int rc = 0;

	if (machine_is_msm7x27a_ffa())
		msm_camera_vreg_config(1);

	rc = config_gpio_table(camera_on_gpio_table,
			ARRAY_SIZE(camera_on_gpio_table));
	if (rc < 0) {
		pr_err("%s: CAMSENSOR gpio table request"
		"failed\n", __func__);
		return rc;
	}

	if (machine_is_msm7x27a_ffa()) {
		gpio_tlmm_config(GPIO_CFG(GPIO_FFA_CAM_5MP_SHDN_N, 1,
					GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN,
					GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		msm_flash_src._fsrc.current_driver_src.led1 =
						GPIO_FFA_LED_DRV_EN1;
		msm_flash_src._fsrc.current_driver_src.led2 =
						GPIO_FFA_LED_DRV_EN2;
		msm_camera_sensor_s5k4e1_data.sensor_reset =
						GPIO_FFA_CAM_5MP_SHDN_N;
		msm_camera_sensor_s5k4e1_data.vcm_pwd =
						GPIO_FFA_CAM_5MP_PWDN_VCM;
	}
	return rc;
}

static void config_camera_off_gpios_rear(void)
{
	if (machine_is_msm7x27a_ffa())
		msm_camera_vreg_config(0);

	config_gpio_table(camera_off_gpio_table,
			ARRAY_SIZE(camera_off_gpio_table));

	if (machine_is_msm7x27a_ffa())
		gpio_tlmm_config(GPIO_CFG(GPIO_FFA_CAM_5MP_SHDN_N, 0,
					GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN,
					GPIO_CFG_2MA), GPIO_CFG_ENABLE);
}

static int config_camera_on_gpios_front(void)
{
	int rc = 0;

	if (machine_is_msm7x27a_ffa())
		msm_camera_vreg_config(1);

	rc = config_gpio_table(camera_on_gpio_table,
			ARRAY_SIZE(camera_on_gpio_table));
	if (rc < 0) {
		pr_err("%s: CAMSENSOR gpio table request"
			"failed\n", __func__);
		return rc;
	}

	if (machine_is_msm7x27a_ffa()) {
		gpio_tlmm_config(GPIO_CFG(GPIO_FFA_CAM_1MP_XCLR, 1,
				GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP,
				GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		msm_flash_src._fsrc.current_driver_src.led1 =
						GPIO_FFA_LED_DRV_EN1;
		msm_flash_src._fsrc.current_driver_src.led2 =
						GPIO_FFA_LED_DRV_EN2;
		msm_camera_sensor_ov9726_data.sensor_reset =
						GPIO_FFA_CAM_1MP_XCLR;
	}
	return rc;
}

static void config_camera_off_gpios_front(void)
{
	if (machine_is_msm7x27a_ffa())
		msm_camera_vreg_config(0);

	config_gpio_table(camera_off_gpio_table,
			ARRAY_SIZE(camera_off_gpio_table));

	if (machine_is_msm7x27a_ffa())
		gpio_tlmm_config(GPIO_CFG(GPIO_FFA_CAM_1MP_XCLR, 0,
				GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP,
				GPIO_CFG_2MA), GPIO_CFG_ENABLE);
}

struct msm_camera_device_platform_data msm_camera_device_data_rear = {
	.camera_gpio_on  = config_camera_on_gpios_rear,
	.camera_gpio_off = config_camera_off_gpios_rear,
	.ioext.csiphy = 0xA1000000,
	.ioext.csisz  = 0x00100000,
	.ioext.csiirq = INT_CSI_IRQ_1,
	.ioclk.mclk_clk_rate = 24000000,
	.ioclk.vfe_clk_rate  = 192000000,
	.ioext.appphy = MSM_CLK_CTL_PHYS,
	.ioext.appsz  = MSM_CLK_CTL_SIZE,
};

struct msm_camera_device_platform_data msm_camera_device_data_front = {
	.camera_gpio_on  = config_camera_on_gpios_front,
	.camera_gpio_off = config_camera_off_gpios_front,
	.ioext.csiphy = 0xA0F00000,
	.ioext.csisz  = 0x00100000,
	.ioext.csiirq = INT_CSI_IRQ_0,
	.ioclk.mclk_clk_rate = 24000000,
	.ioclk.vfe_clk_rate  = 192000000,
	.ioext.appphy = MSM_CLK_CTL_PHYS,
	.ioext.appsz  = MSM_CLK_CTL_SIZE,
};

#ifdef CONFIG_S5K4E1
static struct msm_camera_sensor_platform_info s5k4e1_sensor_7627a_info = {
	.mount_angle = 0
};

static struct msm_camera_sensor_flash_data flash_s5k4e1 = {
	.flash_type             = MSM_CAMERA_FLASH_LED,
	.flash_src              = &msm_flash_src
};

static struct msm_camera_sensor_info msm_camera_sensor_s5k4e1_data = {
	.sensor_name    = "s5k4e1",
	.sensor_reset   = GPIO_SURF_CAM_GP_CAMIF_RESET_N,
	.sensor_reset_enable = 1,
	.sensor_pwd             = 85,
	.vcm_pwd                = GPIO_SURF_CAM_GP_CAM_PWDN,
	.vcm_enable             = 1,
	.pdata                  = &msm_camera_device_data_rear,
	.flash_data             = &flash_s5k4e1,
	.sensor_platform_info   = &s5k4e1_sensor_7627a_info,
	.csi_if                 = 1
};

static struct platform_device msm_camera_sensor_s5k4e1 = {
	.name   = "msm_camera_s5k4e1",
	.dev    = {
		.platform_data = &msm_camera_sensor_s5k4e1_data,
	},
};
#endif

#ifdef CONFIG_IMX072
static struct msm_camera_sensor_platform_info imx072_sensor_7627a_info = {
	.mount_angle = 0
};

static struct msm_camera_sensor_flash_data flash_imx072 = {
	.flash_type             = MSM_CAMERA_FLASH_LED,
	.flash_src              = &msm_flash_src
};

static struct msm_camera_sensor_info msm_camera_sensor_imx072_data = {
	.sensor_name    = "imx072",
	.sensor_reset_enable = 1,
	.sensor_reset   = GPIO_SURF_CAM_GP_CAMIF_RESET_N, /* TODO 106,*/
	.sensor_pwd             = 85,
	.vcm_pwd                = GPIO_SURF_CAM_GP_CAM_PWDN,
	.vcm_enable             = 1,
	.pdata                  = &msm_camera_device_data_rear,
	.flash_data             = &flash_imx072,
	.sensor_platform_info = &imx072_sensor_7627a_info,
	.csi_if                 = 1
};

static struct platform_device msm_camera_sensor_imx072 = {
	.name   = "msm_camera_imx072",
	.dev    = {
		.platform_data = &msm_camera_sensor_imx072_data,
	},
};
#endif

#ifdef CONFIG_WEBCAM_OV9726
static struct msm_camera_sensor_platform_info ov9726_sensor_7627a_info = {
	.mount_angle = 90
};

static struct msm_camera_sensor_flash_data flash_ov9726 = {
	.flash_type             = MSM_CAMERA_FLASH_LED,
	.flash_src              = &msm_flash_src
};

static struct msm_camera_sensor_info msm_camera_sensor_ov9726_data = {
	.sensor_name    = "ov9726",
	.sensor_reset   = GPIO_SURF_CAM_GP_CAM1MP_XCLR,
	.sensor_reset_enable = 0,
	.sensor_pwd             = 85,
	.vcm_pwd                = 1,
	.vcm_enable             = 0,
	.pdata                  = &msm_camera_device_data_front,
	.flash_data             = &flash_ov9726,
	.sensor_platform_info   = &ov9726_sensor_7627a_info,
	.csi_if                 = 1
};

static struct platform_device msm_camera_sensor_ov9726 = {
	.name   = "msm_camera_ov9726",
	.dev    = {
		.platform_data = &msm_camera_sensor_ov9726_data,
	},
};
#endif

#ifdef CONFIG_MT9E013
static struct msm_camera_sensor_platform_info mt9e013_sensor_7627a_info = {
	.mount_angle = 0
};

static struct msm_camera_sensor_flash_data flash_mt9e013 = {
	.flash_type = MSM_CAMERA_FLASH_LED,
	.flash_src  = &msm_flash_src
};

static struct msm_camera_sensor_info msm_camera_sensor_mt9e013_data = {
	.sensor_name    = "mt9e013",
	.sensor_reset   = 0,
	.sensor_reset_enable = 1,
	.sensor_pwd     = 85,
	.vcm_pwd        = 1,
	.vcm_enable     = 0,
	.pdata          = &msm_camera_device_data_rear,
	.flash_data     = &flash_mt9e013,
	.sensor_platform_info   = &mt9e013_sensor_7627a_info,
	.csi_if         = 1
};

static struct platform_device msm_camera_sensor_mt9e013 = {
	.name      = "msm_camera_mt9e013",
	.dev       = {
		.platform_data = &msm_camera_sensor_mt9e013_data,
	},
};
#endif

static struct i2c_board_info i2c_camera_devices[] = {
	#ifdef CONFIG_S5K4E1
	{
		I2C_BOARD_INFO("s5k4e1", 0x36),
	},
	{
		I2C_BOARD_INFO("s5k4e1_af", 0x8c >> 1),
	},
	#endif
	#ifdef CONFIG_WEBCAM_OV9726
	{
		I2C_BOARD_INFO("ov9726", 0x10),
	},
	#endif
	#ifdef CONFIG_IMX072
	{
		I2C_BOARD_INFO("imx072", 0x34),
	},
	#endif
	#ifdef CONFIG_MT9E013
	{
		I2C_BOARD_INFO("mt9e013", 0x6C >> 2),
	},
	#endif
	{
		I2C_BOARD_INFO("sc628a", 0x37),
	},
};
#endif

static struct platform_device *rumi_sim_devices[] __initdata = {
	&msm_device_dmov,
	&msm_device_smd,
	&smc91x_device,
	&msm_device_uart1,
	&msm_device_nand,
	&msm_device_uart_dm1,
	&msm_gsbi0_qup_i2c_device,
	&msm_gsbi1_qup_i2c_device,
};

static struct platform_device *surf_ffa_devices[] __initdata = {
	&msm_device_dmov,
	&msm_device_smd,
	&msm_device_uart1,
	&msm_device_uart_dm1,
	&msm_device_nand,
	&msm_gsbi0_qup_i2c_device,
	&msm_gsbi1_qup_i2c_device,
	&msm_device_otg,
	&msm_device_gadget_peripheral,
	&android_usb_device,
	&android_pmem_device,
	&android_pmem_adsp_device,
	&usb_mass_storage_device,
	&rndis_device,
	&usb_diag_device,
	&usb_gadget_fserial_device,
	&android_pmem_audio_device,
	&msm_device_snd,
	&msm_device_adspdec,
	&msm_fb_device,
	&lcdc_toshiba_panel_device,
	&msm_batt_device,
	&smsc911x_device,
#ifdef CONFIG_S5K4E1
	&msm_camera_sensor_s5k4e1,
#endif
#ifdef CONFIG_IMX072
	&msm_camera_sensor_imx072,
#endif
#ifdef CONFIG_WEBCAM_OV9726
	&msm_camera_sensor_ov9726,
#endif
#ifdef CONFIG_MT9E013
	&msm_camera_sensor_mt9e013,
#endif
#ifdef CONFIG_FB_MSM_MIPI_DSI
	&mipi_dsi_renesas_panel_device,
#endif
	&msm_kgsl_3d0,
#ifdef CONFIG_BT
	&msm_bt_power_device,
#endif
};

static unsigned pmem_kernel_ebi1_size = PMEM_KERNEL_EBI1_SIZE;
static int __init pmem_kernel_ebi1_size_setup(char *p)
{
	pmem_kernel_ebi1_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_kernel_ebi1_size", pmem_kernel_ebi1_size_setup);

static unsigned pmem_audio_size = MSM_PMEM_AUDIO_SIZE;
static int __init pmem_audio_size_setup(char *p)
{
	pmem_audio_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_audio_size", pmem_audio_size_setup);

static void __init msm_msm7x2x_allocate_memory_regions(void)
{
	void *addr;
	unsigned long size;

	size = fb_size ? : MSM_FB_SIZE;
	addr = alloc_bootmem_align(size, 0x1000);
	msm_fb_resources[0].start = __pa(addr);
	msm_fb_resources[0].end = msm_fb_resources[0].start + size - 1;
	pr_info("allocating %lu bytes at %p (%lx physical) for fb\n",
		size, addr, __pa(addr));
}

static struct memtype_reserve msm7x27a_reserve_table[] __initdata = {
	[MEMTYPE_SMI] = {
	},
	[MEMTYPE_EBI0] = {
		.flags	=	MEMTYPE_FLAGS_1M_ALIGN,
	},
	[MEMTYPE_EBI1] = {
		.flags	=	MEMTYPE_FLAGS_1M_ALIGN,
	},
};

static void __init size_pmem_devices(void)
{
#ifdef CONFIG_ANDROID_PMEM
	android_pmem_adsp_pdata.size = pmem_adsp_size;
	android_pmem_pdata.size = pmem_mdp_size;
	android_pmem_audio_pdata.size = pmem_audio_size;
#endif
}

static void __init reserve_memory_for(struct android_pmem_platform_data *p)
{
	msm7x27a_reserve_table[p->memory_type].size += p->size;
}

static void __init reserve_pmem_memory(void)
{
#ifdef CONFIG_ANDROID_PMEM
	reserve_memory_for(&android_pmem_adsp_pdata);
	reserve_memory_for(&android_pmem_pdata);
	reserve_memory_for(&android_pmem_audio_pdata);
	msm7x27a_reserve_table[MEMTYPE_EBI1].size += pmem_kernel_ebi1_size;
#endif
}

static void __init msm7x27a_calculate_reserve_sizes(void)
{
	size_pmem_devices();
	reserve_pmem_memory();
}

static int msm7x27a_paddr_to_memtype(unsigned int paddr)
{
	return MEMTYPE_EBI1;
}

static struct reserve_info msm7x27a_reserve_info __initdata = {
	.memtype_reserve_table = msm7x27a_reserve_table,
	.calculate_reserve_sizes = msm7x27a_calculate_reserve_sizes,
	.paddr_to_memtype = msm7x27a_paddr_to_memtype,
};

static void __init msm7x27a_reserve(void)
{
	reserve_info = &msm7x27a_reserve_info;
	msm_reserve();
}

static void __init msm_device_i2c_init(void)
{
	msm_gsbi0_qup_i2c_device.dev.platform_data = &msm_gsbi0_qup_i2c_pdata;
	msm_gsbi1_qup_i2c_device.dev.platform_data = &msm_gsbi1_qup_i2c_pdata;
}

static struct msm_panel_common_pdata mdp_pdata = {
	.gpio = 97,
};

#define GPIO_LCDC_BRDG_PD	128
#define GPIO_LCDC_BRDG_RESET_N	129
static unsigned mipi_dsi_gpio[] = {
	GPIO_CFG(GPIO_LCDC_BRDG_RESET_N, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
		GPIO_CFG_2MA),       /* LCDC_BRDG_RESET_N */
};

static int msm_fb_dsi_client_reset(void)
{
	int rc = 0;

	rc = gpio_request(GPIO_LCDC_BRDG_RESET_N, "lcdc_brdg_reset_n");

	if (rc < 0) {
		pr_err("failed to request lcd brdg reset\n");
		return rc;
	}

	rc = gpio_tlmm_config(mipi_dsi_gpio[0], GPIO_CFG_ENABLE);
	if (rc) {
		pr_err("Failed to enable LCDC Bridge reset enable\n");
		goto gpio_error;
	}

	rc = gpio_direction_output(GPIO_LCDC_BRDG_RESET_N, 1);
	if (!rc) {
		gpio_set_value_cansleep(GPIO_LCDC_BRDG_RESET_N, 0);
		msleep(20);
		gpio_set_value_cansleep(GPIO_LCDC_BRDG_RESET_N, 1);
		return rc;
	} else {
		goto gpio_error;
	}

gpio_error:
	pr_err("Failed GPIO bridge reset\n");
	gpio_free(GPIO_LCDC_BRDG_RESET_N);
	return rc;
}

static int dsi_gpio_initialized;

static int mipi_dsi_panel_power(int on)
{
	int rc = 0;

	if (!on)
		return rc;

	/* I2C-controlled GPIO Expander -init of the GPIOs very late */
	if (!dsi_gpio_initialized) {
		pmapp_disp_backlight_init();

		if (pmapp_disp_backlight_set_brightness(100))
			pr_err("display backlight set brightness failed\n");

		if (machine_is_msm7x27a_surf()) {
			rc = gpio_request(GPIO_DISPLAY_PWR_EN, "gpio_disp_pwr");
			if (rc < 0) {
				pr_err("failed to request gpio_disp_pwr\n");
				return rc;
			}

			if (gpio_request(GPIO_BACKLIGHT_EN, "gpio_bkl_en")) {
				pr_err("failed to request gpio_bkl_en\n");
				goto fail_gpio1;
			}

			if (!gpio_direction_output(GPIO_DISPLAY_PWR_EN, 1)) {
				gpio_set_value_cansleep(GPIO_DISPLAY_PWR_EN,
					on);
			} else {
				pr_err("failed to enable display pwr\n");
				goto fail_gpio2;
			}

			if (!gpio_direction_output(GPIO_BACKLIGHT_EN, 1)) {
				gpio_set_value_cansleep(GPIO_BACKLIGHT_EN, on);
				return rc;
			} else {
				pr_err("failed to enable backlight\n");
				goto fail_gpio2;
			}

fail_gpio2:
		gpio_free(GPIO_BACKLIGHT_EN);
fail_gpio1:
		gpio_free(GPIO_DISPLAY_PWR_EN);

		} else {
			rc = gpio_request(GPIO_FFA_LCD_PWR_EN_N,
				"gpio_disp_pwr");
			if (rc < 0) {
				pr_err("failed to request lcd pwr\n");
				return rc;
			}
			if (!gpio_direction_output(GPIO_FFA_LCD_PWR_EN_N,
				1)) {
				gpio_set_value_cansleep(GPIO_FFA_LCD_PWR_EN_N,
					on);
				return rc;
			} else {
				pr_err("failed to enable lcd pwr\n");
				return rc;
			}

		}
	}

	return rc;
}

#define MDP_303_VSYNC_GPIO 97

#ifdef CONFIG_FB_MSM_MDP303
static struct mipi_dsi_platform_data mipi_dsi_pdata = {
	.vsync_gpio = MDP_303_VSYNC_GPIO,
	.dsi_power_save   = mipi_dsi_panel_power,
	.dsi_client_reset = msm_fb_dsi_client_reset,
};
#endif

static void __init msm_fb_add_devices(void)
{
	msm_fb_register_device("mdp", &mdp_pdata);
	msm_fb_register_device("lcdc", &lcdc_pdata);
	msm_fb_register_device("mipi_dsi", &mipi_dsi_pdata);
}

#define MSM_EBI2_PHYS			0xa0d00000
#define MSM_EBI2_XMEM_CS2_CFG1		0xa0d10030

static void __init msm7x27a_init_ebi2(void)
{
	uint32_t ebi2_cfg;
	void __iomem *ebi2_cfg_ptr;

	ebi2_cfg_ptr = ioremap_nocache(MSM_EBI2_PHYS, sizeof(uint32_t));
	if (!ebi2_cfg_ptr)
		return;

	ebi2_cfg = readl(ebi2_cfg_ptr);
	if (machine_is_msm7x27a_rumi3() || machine_is_msm7x27a_surf())
		ebi2_cfg |= (1 << 4); /* CS2 */

	writel(ebi2_cfg, ebi2_cfg_ptr);
	iounmap(ebi2_cfg_ptr);

	/* Enable A/D MUX[bit 31] from EBI2_XMEM_CS2_CFG1 */
	ebi2_cfg_ptr = ioremap_nocache(MSM_EBI2_XMEM_CS2_CFG1,
							 sizeof(uint32_t));
	if (!ebi2_cfg_ptr)
		return;

	ebi2_cfg = readl(ebi2_cfg_ptr);
	if (machine_is_msm7x27a_surf())
		ebi2_cfg |= (1 << 31);

	writel(ebi2_cfg, ebi2_cfg_ptr);
	iounmap(ebi2_cfg_ptr);
}

static void __init msm7x2x_init_irq(void)
{
	msm_init_irq();
}

#define ATMEL_TS_I2C_NAME "maXTouch"
static struct vreg *vreg_l2;
static struct vreg *vreg_s3;

#define ATMEL_TS_GPIO_IRQ 82

static int atmel_ts_platform_init(struct i2c_client *client)
{
	int rc;

	vreg_l2 = vreg_get(NULL, "rfrx2");
	if (IS_ERR(vreg_l2)) {
		pr_err("%s: vreg_get for L2 failed\n", __func__);
		return PTR_ERR(vreg_l2);
	}

	rc = vreg_set_level(vreg_l2, 2850);
	if (rc) {
		pr_err("%s: vreg set level failed (%d) for l2\n",
		       __func__, rc);
		goto vreg_put_l2;
	}

	rc = vreg_enable(vreg_l2);
	if (rc) {
		pr_err("%s: vreg enable failed (%d)\n",
		       __func__, rc);
		goto vreg_put_l2;
	}

	vreg_s3 = vreg_get(NULL, "msme1");
	if (IS_ERR(vreg_s3)) {
		pr_err("%s: vreg_get for S3 failed\n", __func__);
		rc = PTR_ERR(vreg_s3);
		goto vreg_disable_l2;
	}

	rc = vreg_set_level(vreg_s3, 1800);
	if (rc) {
		pr_err("%s: vreg set level failed (%d) for S3\n",
		       __func__, rc);
		goto vreg_put_s3;
	}

	rc = vreg_enable(vreg_s3);
	if (rc) {
		pr_err("%s: vreg enable failed (%d) for S3\n",
		       __func__, rc);
		goto vreg_put_s3;
	}

	rc = gpio_tlmm_config(GPIO_CFG(ATMEL_TS_GPIO_IRQ, 0,
				GPIO_CFG_INPUT, GPIO_CFG_PULL_UP,
				GPIO_CFG_8MA), GPIO_CFG_ENABLE);
	if (rc) {
		pr_err("%s: gpio_tlmm_config for %d failed\n",
			__func__, ATMEL_TS_GPIO_IRQ);
		goto vreg_disable_s3;
	}

	/* configure touchscreen interrupt gpio */
	rc = gpio_request(ATMEL_TS_GPIO_IRQ, "atmel_maxtouch_gpio");
	if (rc) {
		pr_err("%s: unable to request gpio %d\n",
			__func__, ATMEL_TS_GPIO_IRQ);
		goto ts_gpio_tlmm_unconfig;
	}

	rc = gpio_direction_input(ATMEL_TS_GPIO_IRQ);
	if (rc < 0) {
		pr_err("%s: unable to set the direction of gpio %d\n",
			__func__, ATMEL_TS_GPIO_IRQ);
		goto free_ts_gpio;
	}
	return 0;

free_ts_gpio:
	gpio_free(ATMEL_TS_GPIO_IRQ);
ts_gpio_tlmm_unconfig:
	gpio_tlmm_config(GPIO_CFG(ATMEL_TS_GPIO_IRQ, 0,
				GPIO_CFG_INPUT, GPIO_CFG_NO_PULL,
				GPIO_CFG_2MA), GPIO_CFG_DISABLE);
vreg_disable_s3:
	vreg_disable(vreg_s3);
vreg_put_s3:
	vreg_put(vreg_s3);
vreg_disable_l2:
	vreg_disable(vreg_l2);
vreg_put_l2:
	vreg_put(vreg_l2);
	return rc;
}

static u8 atmel_ts_read_chg(void)
{
	return gpio_get_value(ATMEL_TS_GPIO_IRQ);
}

static u8 atmel_ts_valid_interrupt(void)
{
	return !atmel_ts_read_chg();
}

static struct mxt_platform_data atmel_ts_pdata = {
	.numtouch = 10,
	.init_platform_hw = atmel_ts_platform_init,
	.max_x = 506,
	.max_y = 864,
	.valid_interrupt = atmel_ts_valid_interrupt,
	.read_chg = atmel_ts_read_chg,
};

static struct i2c_board_info atmel_ts_i2c_info[] __initdata = {
	{
		I2C_BOARD_INFO(ATMEL_TS_I2C_NAME, 0x4a),
		.platform_data = &atmel_ts_pdata,
		.irq = MSM_GPIO_TO_INT(ATMEL_TS_GPIO_IRQ),
	},
};

#define KP_INDEX(row, col) ((row)*ARRAY_SIZE(kp_col_gpios) + (col))

static unsigned int kp_row_gpios[] = {31, 32, 33, 34, 35};
static unsigned int kp_col_gpios[] = {36, 37, 38, 39, 40};

static const unsigned short keymap[ARRAY_SIZE(kp_col_gpios) *
					  ARRAY_SIZE(kp_row_gpios)] = {
	[KP_INDEX(0, 0)] = KEY_7,
	[KP_INDEX(0, 1)] = KEY_DOWN,
	[KP_INDEX(0, 2)] = KEY_UP,
	[KP_INDEX(0, 3)] = KEY_RIGHT,
	[KP_INDEX(0, 4)] = KEY_ENTER,

	[KP_INDEX(1, 0)] = KEY_LEFT,
	[KP_INDEX(1, 1)] = KEY_SEND,
	[KP_INDEX(1, 2)] = KEY_1,
	[KP_INDEX(1, 3)] = KEY_4,
	[KP_INDEX(1, 4)] = KEY_CLEAR,

	[KP_INDEX(2, 0)] = KEY_6,
	[KP_INDEX(2, 1)] = KEY_5,
	[KP_INDEX(2, 2)] = KEY_8,
	[KP_INDEX(2, 3)] = KEY_3,
	[KP_INDEX(2, 4)] = KEY_NUMERIC_STAR,

	[KP_INDEX(3, 0)] = KEY_9,
	[KP_INDEX(3, 1)] = KEY_NUMERIC_POUND,
	[KP_INDEX(3, 2)] = KEY_0,
	[KP_INDEX(3, 3)] = KEY_2,
	[KP_INDEX(3, 4)] = KEY_SLEEP,

	[KP_INDEX(4, 0)] = KEY_BACK,
	[KP_INDEX(4, 1)] = KEY_HOME,
	[KP_INDEX(4, 2)] = KEY_MENU,
	[KP_INDEX(4, 3)] = KEY_VOLUMEUP,
	[KP_INDEX(4, 4)] = KEY_VOLUMEDOWN,
};

/* SURF keypad platform device information */
static struct gpio_event_matrix_info kp_matrix_info = {
	.info.func	= gpio_event_matrix_func,
	.keymap		= keymap,
	.output_gpios	= kp_row_gpios,
	.input_gpios	= kp_col_gpios,
	.noutputs	= ARRAY_SIZE(kp_row_gpios),
	.ninputs	= ARRAY_SIZE(kp_col_gpios),
	.settle_time.tv.nsec = 40 * NSEC_PER_USEC,
	.poll_time.tv.nsec = 20 * NSEC_PER_MSEC,
	.flags		= GPIOKPF_LEVEL_TRIGGERED_IRQ | GPIOKPF_DRIVE_INACTIVE |
			  GPIOKPF_PRINT_UNMAPPED_KEYS,
};

static struct gpio_event_info *kp_info[] = {
	&kp_matrix_info.info
};

static struct gpio_event_platform_data kp_pdata = {
	.name		= "7x27a_kp",
	.info		= kp_info,
	.info_count	= ARRAY_SIZE(kp_info)
};

static struct platform_device kp_pdev = {
	.name	= GPIO_EVENT_DEV_NAME,
	.id	= -1,
	.dev	= {
		.platform_data	= &kp_pdata,
	},
};

static struct msm_handset_platform_data hs_platform_data = {
	.hs_name = "7k_handset",
	.pwr_key_delay_ms = 500, /* 0 will disable end key */
};

static struct platform_device hs_pdev = {
	.name   = "msm-handset",
	.id     = -1,
	.dev    = {
		.platform_data = &hs_platform_data,
	},
};
#define UART1DM_RX_GPIO		45
static void __init msm7x2x_init(void)
{
	if (socinfo_init() < 0)
		printk(KERN_ERR "%s: socinfo_init() failed!\n",
		       __func__);

	msm_clock_init(msm_clocks_7x27a, msm_num_clocks_7x27a);
	/* Common functions for SURF/FFA/RUMI3 */
	msm_device_i2c_init();
	msm7x27a_init_mmc();
	msm7x27a_init_ebi2();
#ifdef CONFIG_SERIAL_MSM_HS
	msm_uart_dm1_pdata.wakeup_irq = gpio_to_irq(UART1DM_RX_GPIO);
	msm_device_uart_dm1.dev.platform_data = &msm_uart_dm1_pdata;
#endif

	if (machine_is_msm7x27a_rumi3()) {
		platform_add_devices(rumi_sim_devices,
				ARRAY_SIZE(rumi_sim_devices));
	}
	if (machine_is_msm7x27a_surf() || machine_is_msm7x27a_ffa()) {
#ifdef CONFIG_USB_MSM_OTG_72K
		msm_otg_pdata.swfi_latency =
			msm7x27a_pm_data
		[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].latency;
		msm_device_otg.dev.platform_data = &msm_otg_pdata;
#endif
		msm7x27a_cfg_smsc911x();
		platform_add_devices(surf_ffa_devices,
				ARRAY_SIZE(surf_ffa_devices));
		msm_fb_add_devices();
#ifdef CONFIG_USB_EHCI_MSM_72K
		msm7x2x_init_host();
#endif
	}

	msm_pm_set_platform_data(msm7x27a_pm_data,
				ARRAY_SIZE(msm7x27a_pm_data));

#if defined(CONFIG_I2C) && defined(CONFIG_GPIO_SX150X)
	register_i2c_devices();
#endif
#if defined(CONFIG_BT) && defined(CONFIG_MARIMBA_CORE)
	bt_power_init();
#endif
	i2c_register_board_info(MSM_GSBI1_QUP_I2C_BUS_ID,
		atmel_ts_i2c_info,
		ARRAY_SIZE(atmel_ts_i2c_info));

	i2c_register_board_info(MSM_GSBI0_QUP_I2C_BUS_ID,
			i2c_camera_devices,
			ARRAY_SIZE(i2c_camera_devices));
	platform_device_register(&kp_pdev);
	platform_device_register(&hs_pdev);
}

#ifdef CONFIG_CACHE_L2X0
static void msm7x27x_l2_cache_init(void)
{
	int aux_ctrl = 0;

	/* Way Size 010(0x2) 32KB */
	aux_ctrl = (0x1 << L2X0_AUX_CTRL_SHARE_OVERRIDE_SHIFT) | \
		   (0x2 << L2X0_AUX_CTRL_WAY_SIZE_SHIFT) | \
		   (0x1 << L2X0_AUX_CTRL_EVNT_MON_BUS_EN_SHIFT);

	l2x0_init(MSM_L2CC_BASE, aux_ctrl, L2X0_AUX_CTRL_MASK);
}
#endif

static void __init msm7x2x_init_early(void)
{
	msm_msm7x2x_allocate_memory_regions();
}

static void __init msm7x2x_map_io(void)
{
	msm_map_common_io();
#ifdef CONFIG_CACHE_L2X0
	msm7x27x_l2_cache_init();
#endif
}

MACHINE_START(MSM7X27A_RUMI3, "QCT MSM7x27a RUMI3")
	.boot_params	= PHYS_OFFSET + 0x100,
	.map_io		= msm7x2x_map_io,
	.reserve	= msm7x27a_reserve,
	.init_irq	= msm7x2x_init_irq,
	.init_machine	= msm7x2x_init,
	.timer		= &msm_timer,
	.init_early     = msm7x2x_init_early,
MACHINE_END
MACHINE_START(MSM7X27A_SURF, "QCT MSM7x27a SURF")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io        = MSM_DEBUG_UART_PHYS,
	.io_pg_offst    = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.boot_params	= PHYS_OFFSET + 0x100,
	.map_io		= msm7x2x_map_io,
	.reserve	= msm7x27a_reserve,
	.init_irq	= msm7x2x_init_irq,
	.init_machine	= msm7x2x_init,
	.timer		= &msm_timer,
	.init_early     = msm7x2x_init_early,
MACHINE_END
MACHINE_START(MSM7X27A_FFA, "QCT MSM7x27a FFA")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io        = MSM_DEBUG_UART_PHYS,
	.io_pg_offst    = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.boot_params	= PHYS_OFFSET + 0x100,
	.map_io		= msm7x2x_map_io,
	.reserve	= msm7x27a_reserve,
	.init_irq	= msm7x2x_init_irq,
	.init_machine	= msm7x2x_init,
	.timer		= &msm_timer,
	.init_early     = msm7x2x_init_early,
MACHINE_END
