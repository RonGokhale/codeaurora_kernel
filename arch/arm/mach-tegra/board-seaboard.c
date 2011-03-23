/*
 * arch/arm/mach-tegra/board-seaboard.c
 *
 * Copyright (c) 2010, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/platform_device.h>
#include <linux/serial_8250.h>
#include <linux/i2c.h>
#include <linux/i2c-tegra.h>
#include <linux/i2c/atmel_mxt_ts.h>
#include <linux/dma-mapping.h>
#include <linux/fsl_devices.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/gpio_keys.h>
#include <linux/power/bq20z75.h>
#include <linux/nct1008.h>
#include <linux/cyapa.h>

#include <sound/wm8903.h>

#include <mach/seaboard_audio.h>
#include <mach/iomap.h>
#include <mach/irqs.h>
#include <mach/pinmux.h>
#include <mach/iomap.h>
#include <mach/io.h>
#include <mach/gpio.h>
#include <mach/clk.h>
#include <mach/usb_phy.h>
#include <mach/kbc.h>
#include <mach/suspend.h>
#include <linux/tegra_usb.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include "board.h"
#include "board-seaboard.h"
#include "clock.h"
#include "devices.h"
#include "gpio-names.h"

#define	PMC_CTRL		0x0
#define	PMC_DPD_PADS_ORIDE	0x1c
#define	PMC_BLINK_TIMER		0x40
#define	PMC_CTRL_BLINK_EN	(1<<7)
#define	PMC_DPD_PADS_ORIDE_BLINK_ENABLE	(1<<20)

static struct plat_serial8250_port debug_uart_platform_data[] = {
	{
		/* Memory and IRQ filled in before registration */
		.flags		= UPF_BOOT_AUTOCONF,
		.iotype		= UPIO_MEM,
		.regshift	= 2,
		.uartclk	= 216000000,
	}, {
		.flags		= 0,
	}
};

static struct platform_device debug_uart = {
	.name = "serial8250",
	.id = PLAT8250_DEV_PLATFORM,
	.dev = {
		.platform_data = debug_uart_platform_data,
	},
};

static __initdata struct tegra_clk_init_table seaboard_clk_init_table[] = {
	/* name		parent		rate		enabled */
	{ "clk_m",	NULL,		12000000,	true},
	{ "pll_m",	"clk_m",	600000000,	true},
	{ "3d",		"pll_m",	300000000,	false},
	{ "2d",		"pll_m",	300000000,	false},
	{ "vi",		"pll_m",	150000000,	false},
	{ "vi_sensor",	"pll_m",	18200000,	false},
	{ "epp",	"pll_m",	300000000,	false},
	{ "mpe",	"pll_m",	100000000,	false},
	{ "emc",	"pll_m",	600000000,	true},
	{ "pll_c",	"clk_m",	600000000,	true},
	{ "pll_c_out1",	"pll_c",	108000000,	true},
	{ "vde",	"pll_c",	240000000,	false},
	{ "pll_p",	"clk_m",	216000000,	true},
	{ "pll_p_out1",	"pll_p",	28800000,	true},
	{ "pll_a",	"pll_p_out1",	56448000,	true},
	{ "pll_a_out0",	"pll_a",	11289600,	true},
	{ "cdev1",	"pll_a_out0",	11289600,	true},
	{ "i2s1",	"pll_a_out0",	11289600,	false},
	{ "audio",	"pll_a_out0",	11289600,	false},
	{ "audio_2x",	"audio",	22579200,	false},
	{ "pll_p_out2",	"pll_p",	48000000,	true},
	{ "pll_p_out3",	"pll_p",	72000000,	true},
	{ "i2c1_i2c",	"pll_p_out3",	72000000,	true},
	{ "i2c2_i2c",	"pll_p_out3",	72000000,	true},
	{ "i2c3_i2c",	"pll_p_out3",	72000000,	true},
	{ "dvc_i2c",	"pll_p_out3",	72000000,	true},
	{ "csi",	"pll_p_out3",	72000000,	false},
	{ "pll_p_out4",	"pll_p",	24000000,	true},
	{ "sclk",	"pll_c_out1",	108000000,	true},
	{ "hclk",	"sclk",		108000000,	true},
	{ "pclk",	"hclk",		54000000,	true},
	{ "spdif_in",	"pll_p",	36000000,	false},
	{ "csite",	"pll_p",	144000000,	true},
	{ "host1x",	"pll_p",	144000000,	false},
	{ "disp1",	"pll_p",	216000000,	false},
	{ "pll_d",	"clk_m",	1000000,	false},
	{ "pll_d_out0",	"pll_d",	500000,		false},
	{ "dsi",	"pll_d",	1000000,	false},
	{ "pll_u",	"clk_m",	480000000,	true},
	{ "clk_d",	"clk_m",	24000000,	true},
	{ "timer",	"clk_m",	12000000,	true},
	{ "i2s2",	"clk_m",	12000000,	false},
	{ "spdif_out",	"pll_a_out0",	11289600,	false},
	{ "spi",	"clk_m",	12000000,	false},
	{ "xio",	"clk_m",	12000000,	false},
	{ "twc",	"clk_m",	12000000,	false},
	{ "sbc1",	"clk_m",	12000000,	false},
	{ "sbc2",	"clk_m",	12000000,	false},
	{ "sbc3",	"clk_m",	12000000,	false},
	{ "sbc4",	"clk_m",	12000000,	false},
	{ "ide",	"clk_m",	12000000,	false},
	{ "ndflash",	"clk_m",	12000000,	false},
	{ "vfir",	"clk_m",	12000000,	false},
	{ "la",		"clk_m",	12000000,	false},
	{ "owr",	"clk_m",	12000000,	false},
	{ "nor",	"clk_m",	12000000,	false},
	{ "mipi",	"clk_m",	12000000,	false},
	{ "i2c1",	"clk_m",	3000000,	false},
	{ "i2c2",	"clk_m",	3000000,	false},
	{ "i2c3",	"clk_m",	3000000,	false},
	{ "dvc",	"clk_m",	3000000,	false},
	{ "uarta",	"clk_m",	12000000,	false},
	{ "uartb",	"pll_p",	216000000,	true},
	{ "uartc",	"clk_m",	12000000,	false},
	{ "uartd",	"pll_p",	216000000,	true},
	{ "uarte",	"clk_m",	12000000,	false},
	{ "cve",	"clk_m",	12000000,	false},
	{ "tvo",	"clk_m",	12000000,	false},
	{ "hdmi",	"clk_m",	12000000,	false},
	{ "tvdac",	"clk_m",	12000000,	false},
	{ "disp2",	"clk_m",	12000000,	false},
	{ "usbd",	"clk_m",	12000000,	true},
	{ "usb2",	"clk_m",	12000000,	false},
	{ "usb3",	"clk_m",	12000000,	true},
	{ "isp",	"clk_m",	12000000,	false},
	{ "csus",	"clk_m",	12000000,	false},
	{ "pwm",	"clk_32k",	32768,		false},
	{ "clk_32k",	NULL,		32768,		true},
	{ "pll_s",	"clk_32k",	32768,		false},
	{ "rtc",	"clk_32k",	32768,		true},
	{ "kbc",	"clk_32k",	32768,		true},
	{ "blink",      "clk_32k",      32768,          true},
	{ NULL,		NULL,		0,		0},
};

static struct tegra_utmip_config utmi_phy_config[] = {
	[0] = {
		.hssync_start_delay = 0,
		.idle_wait_delay = 17,
		.elastic_limit = 16,
		.term_range_adj = 6,
		.xcvr_setup = 15,
		.xcvr_lsfslew = 2,
		.xcvr_lsrslew = 2,
	},
	[1] = {
		.hssync_start_delay = 0,
		.idle_wait_delay = 17,
		.elastic_limit = 16,
		.term_range_adj = 6,
		.xcvr_setup = 8,
		.xcvr_lsfslew = 2,
		.xcvr_lsrslew = 2,
	},
};

static struct tegra_ulpi_config ulpi_phy_config = {
	.reset_gpio = TEGRA_GPIO_PV1,
	.clk = "cdev2",
};

static struct tegra_ehci_platform_data tegra_ehci_pdata[] = {
	[0] = {
		.phy_config = &utmi_phy_config[0],
		.operating_mode = TEGRA_USB_HOST,
		.power_down_on_bus_suspend = 1,
	},
	[1] = {
		.phy_config = &ulpi_phy_config,
		.operating_mode = TEGRA_USB_HOST,
		.power_down_on_bus_suspend = 1,
	},
	[2] = {
		.phy_config = &utmi_phy_config[1],
		.operating_mode = TEGRA_USB_HOST,
		.power_down_on_bus_suspend = 1,
	},
};


static struct cyapa_platform_data cyapa_i2c_platform_data = {
	.flag				= 0,
	.gen				= CYAPA_GEN2,
	.power_state			= CYAPA_PWR_ACTIVE,
	.use_absolute_mode		= false,
	.use_polling_mode		= false,
	.polling_interval_time_active	= CYAPA_ACTIVE_POLLING_INTVAL_TIME,
	.polling_interval_time_lowpower	= CYAPA_LOWPOWER_POLLING_INTVAL_TIME,
	.active_touch_timeout		= CYAPA_ACTIVE_TOUCH_TIMEOUT,
	.name				= CYAPA_I2C_NAME,
	.irq_gpio			= TEGRA_GPIO_CYTP_INT,
	.report_rate			= CYAPA_REPORT_RATE,
};

static struct tegra_i2c_platform_data seaboard_i2c1_platform_data = {
	.adapter_nr	= 0,
	.bus_count	= 1,
	.bus_clk_rate	= { 400000, 0 },
};

static const struct tegra_pingroup_config i2c2_ddc = {
	.pingroup	= TEGRA_PINGROUP_DDC,
	.func		= TEGRA_MUX_I2C2,
};

static const struct tegra_pingroup_config i2c2_gen2 = {
	.pingroup	= TEGRA_PINGROUP_PTA,
	.func		= TEGRA_MUX_I2C2,
};

static struct tegra_i2c_platform_data seaboard_i2c2_platform_data = {
	.adapter_nr	= 1,
	.bus_count	= 2,
	.bus_clk_rate	= { 400000, 100000 },
	.bus_mux	= { &i2c2_ddc, &i2c2_gen2 },
	.bus_mux_len	= { 1, 1 },
};

static struct tegra_i2c_platform_data seaboard_i2c3_platform_data = {
	.adapter_nr	= 3,
	.bus_count	= 1,
	.bus_clk_rate	= { 400000, 0 },
};

static struct tegra_i2c_platform_data seaboard_dvc_platform_data = {
	.adapter_nr	= 4,
	.bus_count	= 1,
	.bus_clk_rate	= { 400000, 0 },
	.is_dvc		= true,
};

static int cros_kbd_keycode[] = {
	/* Row 0 */	KEY_RESERVED,	KEY_RESERVED,	KEY_LEFTCTRL,	KEY_RESERVED,	KEY_RIGHTCTRL,	KEY_RESERVED,	KEY_RESERVED,	KEY_RESERVED,
	/* Row 1 */	KEY_SEARCH,	KEY_ESC,	KEY_TAB,	KEY_GRAVE,	KEY_A,		KEY_Z,		KEY_1,		KEY_Q,
	/* Row 2 */	KEY_BACK,	KEY_RESERVED,	KEY_REFRESH,	KEY_FORWARD,	KEY_D,		KEY_C,		KEY_3,		KEY_E,
	/* Row 3 */	KEY_RESERVED,	KEY_RESERVED,	KEY_RESERVED,	KEY_RESERVED,	KEY_RESERVED,	KEY_RESERVED,	KEY_RESERVED,	KEY_RESERVED,
	/* Row 4 */	KEY_B,		KEY_G,		KEY_T,		KEY_5,		KEY_F,		KEY_V,		KEY_4,		KEY_R,
	/* Row 5 */	KEY_VOLUMEUP,	KEY_BRIGHTNESSUP,	KEY_BRIGHTNESSDOWN,	KEY_RESERVED,	KEY_S,	KEY_X,	KEY_2,		KEY_W,
	/* Row 6 */	KEY_RESERVED,	KEY_RESERVED,	KEY_RIGHTBRACE,	KEY_RESERVED,	KEY_K,		KEY_COMMA,	KEY_8,		KEY_I,
	/* Row 7 */	KEY_RESERVED,	KEY_RESERVED,	KEY_RESERVED,	KEY_RESERVED,	KEY_RESERVED,	KEY_RESERVED,	KEY_RESERVED,	KEY_RESERVED,
	/* Row 8 */	KEY_N,		KEY_H,		KEY_Y,		KEY_6,		KEY_J,		KEY_M,		KEY_7,		KEY_U,
	/* Row 9 */	KEY_RESERVED,	KEY_RESERVED,	KEY_RESERVED,	KEY_RESERVED,	KEY_RESERVED,	KEY_LEFTSHIFT,	KEY_RESERVED,	KEY_RIGHTSHIFT,
	/* Row A */	KEY_EQUAL,	KEY_APOSTROPHE,	KEY_LEFTBRACE,	KEY_MINUS,	KEY_SEMICOLON,	KEY_SLASH,	KEY_0,		KEY_P,
	/* Row B */	KEY_RESERVED,	KEY_VOLUMEDOWN,	KEY_MUTE,	KEY_RESERVED,	KEY_L,		KEY_DOT,	KEY_9,		KEY_O,
	/* Row C */	KEY_RESERVED,	KEY_RESERVED,	KEY_RESERVED,	KEY_RESERVED,	KEY_RESERVED,	KEY_RESERVED,	KEY_RESERVED,	KEY_RESERVED,
	/* Row D */	KEY_RIGHTALT,	KEY_RESERVED,	KEY_RESERVED,	KEY_RESERVED,	KEY_RESERVED,	KEY_RESERVED,	KEY_LEFTALT,	KEY_RESERVED,
	/* Row E */	KEY_RESERVED,	KEY_BACKSPACE,	KEY_RESERVED,	KEY_BACKSLASH,	KEY_ENTER,	KEY_SPACE,	KEY_DOWN,	KEY_UP,
	/* Row F */	KEY_RESERVED,	KEY_RESERVED,	KEY_RESERVED,	KEY_RESERVED,	KEY_RESERVED,	KEY_RESERVED,	KEY_RIGHT,	KEY_LEFT
};


static void seaboard_isl29018_init(void)
{
	tegra_gpio_enable(TEGRA_GPIO_ISL29018_IRQ);
	gpio_request(TEGRA_GPIO_ISL29018_IRQ, "isl29018");
	gpio_direction_input(TEGRA_GPIO_ISL29018_IRQ);
}

static struct wm8903_platform_data wm8903_pdata = {
	.irq_active_low = 0,
	.micdet_cfg = 0,
	.micdet_delay = 100,
	.gpio_base = GPIO_WM8903(0),
	.gpio_cfg = {
		WM8903_GPIO_NO_CONFIG,
		WM8903_GPIO_NO_CONFIG,
		0,
		WM8903_GPIO_NO_CONFIG,
		WM8903_GPIO_NO_CONFIG,
	},
};

static void seaboard_nct1008_init(void)
{
	tegra_gpio_enable(TEGRA_GPIO_NCT1008_THERM2_IRQ);
	gpio_request(TEGRA_GPIO_NCT1008_THERM2_IRQ, "temp_alert");
	gpio_direction_input(TEGRA_GPIO_NCT1008_THERM2_IRQ);
}

static struct nct1008_platform_data nct1008_pdata = {
	.supported_hwrev	= true,
	.ext_range		= false,
	.conv_rate		= 0x08,
	.offset			= 0,
	.hysteresis		= 0,
	.shutdown_ext_limit	= 115,
	.shutdown_local_limit	= 120,
	.throttling_ext_limit	= 90,
	.alarm_fn		= tegra_throttling_enable,
};

static struct i2c_board_info __initdata wm8903_device = {
	I2C_BOARD_INFO("wm8903", 0x1a),
	.platform_data = &wm8903_pdata,
	.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PX3),
};

static struct i2c_board_info __initdata isl29018_device = {
	I2C_BOARD_INFO("isl29018", 0x44),
	.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_ISL29018_IRQ),
};

static struct bq20z75_platform_data bq20z75_pdata = {
	.i2c_retry_count	= 2,
	.battery_detect		= -1,
};

static struct i2c_board_info __initdata bq20z75_device = {
	I2C_BOARD_INFO("bq20z75", 0x0b),
	.platform_data	= &bq20z75_pdata,
};

static struct i2c_board_info __initdata nct1008_device = {
	I2C_BOARD_INFO("nct1008", 0x4c),
	.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_NCT1008_THERM2_IRQ),
	.platform_data = &nct1008_pdata,
};

static struct i2c_board_info __initdata ak8975_device = {
	I2C_BOARD_INFO("ak8975", 0x0c),
	.irq		= TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_MAGNETOMETER),
};

static struct i2c_board_info __initdata cyapa_device = {
	I2C_BOARD_INFO("cypress_i2c_apa", 0x67),
	.irq		= TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_CYTP_INT),
	.platform_data	= &cyapa_i2c_platform_data,
};

static struct i2c_board_info __initdata mpu3050_device = {
	I2C_BOARD_INFO("mpu3050", 0x68),
	.irq		= TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_MPU3050_IRQ),
};

static const u8 mxt_config_data[] = {
	/* MXT_GEN_COMMAND(6) */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	/* MXT_GEN_POWER(7) */
	0xFF, 0xff, 0x32,
	/* MXT_GEN_ACQUIRE(8) */
	0x0a, 0x00, 0x14, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	/* MXT_TOUCH_MULTI(9) */
	0x0F, 0x00, 0x00, 0x1b, 0x2a, 0x00, 0x10, 0x32, 0x02, 0x05,
	0x00, 0x02, 0x01, 0x00, 0x0a, 0x0a, 0x0a, 0x0a, 0x00, 0x03,
	0x56, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x0a, 0x00, 0x00, 0x00,
	/* MXT_TOUCH_KEYARRAY(15) */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00,
	/* MXT_PROCG_NOISE(22) */
	0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x19, 0x00,
	0x00, 0x00, 0x05, 0x0a, 0x14, 0x1e, 0x00,
	/* MXT_PROCI_ONETOUCH(24) */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	/* MXT_PROCI_TWOTOUCH(27) */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	/* MXT_SPT_SELFTEST(25) */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
	/* MXT_SPT_CTECONFIG(28) */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	/* MXT_PROCI_GRIP(40) */
	0x00, 0x00, 0x00, 0x00, 0x00,
	/* MXT_PROCI_PALM(41) */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	/* MXT_SPT_DIGITIZER(43) */
	0x00, 0x00, 0x00, 0x00,
};

static struct mxt_platform_data mxt_platform_data = {
	.x_line		= 27,
	.y_line		= 42,
	.x_size		= 768,
	.y_size		= 1386,
	.blen		= 0x16,
	.threshold	= 0x28,
	.voltage	= 3300000,              /* 3.3V */
	.orient		= MXT_DIAGONAL,
	.irqflags       = IRQF_TRIGGER_FALLING,
	.config         = mxt_config_data,
	.config_length  = sizeof(mxt_config_data),
};

static struct i2c_board_info __initdata mxt_device = {
	I2C_BOARD_INFO("atmel_mxt_ts", 0x5A),
	.platform_data = &mxt_platform_data,
};

static __initdata struct tegra_pingroup_config mxt_pinmux_config[] = {
	{TEGRA_PINGROUP_LVP0,  TEGRA_MUX_RSVD4,         TEGRA_PUPD_NORMAL,    TEGRA_TRI_NORMAL},
};

static void seaboard_mxt_init(void)
{
	int gpio;

	tegra_pinmux_config_table(mxt_pinmux_config, ARRAY_SIZE(mxt_pinmux_config));

	gpio = TEGRA_GPIO_MXT_RST;
	gpio_request(gpio, "TSP_LDO_ON");
	tegra_gpio_enable(gpio);
	gpio_direction_output(gpio, 1);
	gpio_export(gpio, 0);

	gpio = TEGRA_GPIO_MXT_IRQ;
	gpio_request(gpio, "TSP_INT");
	tegra_gpio_enable(gpio);
	gpio_direction_input(gpio);
	mxt_device.irq = TEGRA_GPIO_TO_IRQ(gpio);
}

static void seaboard_mpu3050_init(void)
{
	int gpio;

	gpio = TEGRA_GPIO_MPU3050_IRQ;
	gpio_request(gpio, "mpu_int");
	tegra_gpio_enable(gpio);
	gpio_direction_input(gpio);
}

static void __init common_i2c_init(void)
{
	tegra_i2c_device1.dev.platform_data = &seaboard_i2c1_platform_data;
	tegra_i2c_device2.dev.platform_data = &seaboard_i2c2_platform_data;
	tegra_i2c_device3.dev.platform_data = &seaboard_i2c3_platform_data;
	tegra_i2c_device4.dev.platform_data = &seaboard_dvc_platform_data;

	platform_device_register(&tegra_i2c_device1);
	platform_device_register(&tegra_i2c_device2);
	platform_device_register(&tegra_i2c_device3);
	platform_device_register(&tegra_i2c_device4);
}

static void __init seaboard_i2c_init(void)
{
	tegra_gpio_enable(TEGRA_GPIO_MAGNETOMETER);

	seaboard_isl29018_init();
	seaboard_mxt_init();
	seaboard_nct1008_init();
	seaboard_mpu3050_init();

	i2c_register_board_info(0, &wm8903_device, 1);
	i2c_register_board_info(0, &isl29018_device, 1);
	i2c_register_board_info(0, &mxt_device, 1);
	i2c_register_board_info(0, &mpu3050_device, 1);

	i2c_register_board_info(2, &bq20z75_device, 1);

	i2c_register_board_info(4, &nct1008_device, 1);
	i2c_register_board_info(4, &ak8975_device, 1);

	common_i2c_init();
}

static void __init kaen_i2c_init(void)
{
	seaboard_isl29018_init();
	seaboard_nct1008_init();

	tegra_gpio_enable(TEGRA_GPIO_CYTP_INT);
	gpio_request(TEGRA_GPIO_CYTP_INT, "gpio_cytp_int");
	gpio_direction_input(TEGRA_GPIO_CYTP_INT);

	i2c_register_board_info(0, &wm8903_device, 1);
	i2c_register_board_info(0, &isl29018_device, 1);
	i2c_register_board_info(0, &cyapa_device, 1);

	bq20z75_pdata.battery_detect = TEGRA_GPIO_BATT_DETECT;
	/* battery present is low */
	bq20z75_pdata.battery_detect_present = 0;
	i2c_register_board_info(2, &bq20z75_device, 1);

	i2c_register_board_info(4, &nct1008_device, 1);
	i2c_register_board_info(4, &ak8975_device, 1);

	common_i2c_init();
}

static void __init aebl_i2c_init(void)
{
	seaboard_isl29018_init();
	seaboard_nct1008_init();

	tegra_gpio_enable(TEGRA_GPIO_CYTP_INT);
	gpio_request(TEGRA_GPIO_CYTP_INT, "gpio_cytp_int");
	gpio_direction_input(TEGRA_GPIO_CYTP_INT);

	i2c_register_board_info(0, &wm8903_device, 1);
	i2c_register_board_info(0, &isl29018_device, 1);
	i2c_register_board_info(0, &cyapa_device, 1);

	i2c_register_board_info(4, &nct1008_device, 1);
	i2c_register_board_info(4, &ak8975_device, 1);

	common_i2c_init();
}

static void __init wario_i2c_init(void)
{
	seaboard_nct1008_init();

	tegra_gpio_enable(TEGRA_GPIO_CYTP_INT);
	gpio_request(TEGRA_GPIO_CYTP_INT, "gpio_cytp_int");
	gpio_direction_input(TEGRA_GPIO_CYTP_INT);

	i2c_register_board_info(0, &wm8903_device, 1);
	i2c_register_board_info(0, &isl29018_device, 1);
	i2c_register_board_info(0, &cyapa_device, 1);

	i2c_register_board_info(2, &bq20z75_device, 1);

	i2c_register_board_info(4, &nct1008_device, 1);
	i2c_register_board_info(4, &ak8975_device, 1);

	common_i2c_init();
}

static struct resource tegra_rtc_resources[] = {
	[0] = {
		.start = TEGRA_RTC_BASE,
		.end = TEGRA_RTC_BASE + TEGRA_RTC_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = INT_RTC,
		.end = INT_RTC,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device tegra_rtc_device = {
	.name = "tegra_rtc",
	.id   = -1,
	.resource = tegra_rtc_resources,
	.num_resources = ARRAY_SIZE(tegra_rtc_resources),
};

static struct gpio_keys_button seaboard_gpio_keys_buttons[] = {
	{
		.code		= SW_LID,
		.gpio		= TEGRA_GPIO_LIDSWITCH,
		.active_low	= 0,
		.desc		= "Lid",
		.type		= EV_SW,
		.wakeup		= 1,
		.debounce_interval = 1,
	},
	{
		.code		= KEY_POWER,
		.gpio		= TEGRA_GPIO_POWERKEY,
		.active_low	= 1,
		.desc		= "Power",
		.type		= EV_KEY,
		.wakeup		= 1,
	},
};

static struct gpio_keys_platform_data seaboard_gpio_keys = {
	.buttons	= seaboard_gpio_keys_buttons,
	.nbuttons	= ARRAY_SIZE(seaboard_gpio_keys_buttons),
};

static struct platform_device seaboard_gpio_keys_device = {
	.name		= "gpio-keys",
	.id		= -1,
	.dev		= {
		.platform_data = &seaboard_gpio_keys,
	}
};

static struct tegra_kbc_wake_key seaboard_wake_cfg[] = {
	[0] = {
		.row = 1,
		.col = 7,
	},
	[1] = {
		.row = 15,
		.col = 0,
	},
};

static struct resource seaboard_kbc_resources[] = {
	[0] = {
		.start = TEGRA_KBC_BASE,
		.end   = TEGRA_KBC_BASE + TEGRA_KBC_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = INT_KBC,
		.end   = INT_KBC,
		.flags = IORESOURCE_IRQ,
	},
};

static struct tegra_kbc_platform_data seaboard_kbc_platform_data = {
	.debounce_cnt = 2,
	.repeat_cnt = 5 * 32,
	.wake_cnt = 2,
	.wake_cfg = &seaboard_wake_cfg[0],
};

static struct platform_device seaboard_kbc_device = {
	.name = "tegra-kbc",
	.id = -1,
	.resource = seaboard_kbc_resources,
	.num_resources = ARRAY_SIZE(seaboard_kbc_resources),
	.dev = {
		.platform_data = &seaboard_kbc_platform_data,
	},

};

static void seaboard_kbc_init(void)
{
	struct tegra_kbc_platform_data *data = &seaboard_kbc_platform_data;
	int i, j;

	BUG_ON((KBC_MAX_ROW + KBC_MAX_COL) > KBC_MAX_GPIO);
	/*
	 * Setup the pin configuration information.
	 */
	for (i = 0; i < KBC_MAX_ROW; i++) {
		data->pin_cfg[i].num = i;
		data->pin_cfg[i].is_row = true;
		data->pin_cfg[i].is_col = false;
	}

	for (j = 0; j < KBC_MAX_COL; j++) {
		data->pin_cfg[i + j].num = j;
		data->pin_cfg[i + j].is_row = false;
		data->pin_cfg[i + j].is_col = true;
	}

	platform_device_register(&seaboard_kbc_device);
}

static struct seaboard_audio_platform_data audio_pdata = {
	.gpio_spkr_en = GPIO_WM8903(2),
	.gpio_hp_det = TEGRA_GPIO_PX1,
};

static struct platform_device audio_device = {
	.name = "tegra-snd-seaboard",
	.id   = 0,
	.dev  = {
		.platform_data = &audio_pdata,
	},
};

static struct platform_device spdif_dit_device = {
	.name   = "spdif-dit",
	.id     = -1,
};

static struct platform_device *seaboard_devices[] __initdata = {
	&debug_uart,
	&tegra_rtc_device,
	&pmu_device,
	&seaboard_gpio_keys_device,
	&tegra_gart_device,
	&tegra_spdif_device,
	&spdif_dit_device,
	&tegra_i2s_device1,
	&tegra_das_device,
	&tegra_pcm_device,
	&audio_device,
	&tegra_avp_device,
};

static void __init seaboard_wlan_init(void)
{
	/* set wifi power/reset gpio */
	tegra_gpio_enable(TEGRA_GPIO_PK6);
	gpio_request(TEGRA_GPIO_PK6, "wlan_pwr_rst");
	gpio_direction_output(TEGRA_GPIO_PK6, 1);
}

static struct tegra_suspend_platform_data seaboard_suspend = {
	.cpu_timer = 5000,
	.cpu_off_timer = 5000,
	.core_timer = 0x7e7e,
	.core_off_timer = 0x7f,
	.separate_req = true,
	.corereq_high = false,
	.sysclkreq_high = true,
	.suspend_mode = TEGRA_SUSPEND_LP0,
};

static int seaboard_ehci_init(void)
{
	int gpio_status;

	gpio_status = gpio_request(TEGRA_GPIO_USB1, "VBUS_USB1");
	if (gpio_status < 0) {
		pr_err("VBUS_USB1 request GPIO FAILED\n");
		WARN_ON(1);
	}
	tegra_gpio_enable(TEGRA_GPIO_USB1);
	gpio_status = gpio_direction_output(TEGRA_GPIO_USB1, 1);
	if (gpio_status < 0) {
		pr_err("VBUS_USB1 request GPIO DIRECTION FAILED\n");
		WARN_ON(1);
	}
	gpio_set_value(TEGRA_GPIO_USB1, 1);

	tegra_ehci1_device.dev.platform_data = &tegra_ehci_pdata[0];
	tegra_ehci2_device.dev.platform_data = &tegra_ehci_pdata[1];
	tegra_ehci3_device.dev.platform_data = &tegra_ehci_pdata[2];

	platform_device_register(&tegra_ehci1_device);
	platform_device_register(&tegra_ehci2_device);
	platform_device_register(&tegra_ehci3_device);

	return 0;
}

static void __init __tegra_seaboard_init(void)
{
	seaboard_pinmux_init();

	tegra_common_init();
	tegra_init_suspend(&seaboard_suspend);

	tegra_clk_init_from_table(seaboard_clk_init_table);

	tegra_gpio_enable(audio_pdata.gpio_hp_det);

	platform_add_devices(seaboard_devices, ARRAY_SIZE(seaboard_devices));

	seaboard_ehci_init();
	seaboard_panel_init();
	seaboard_sdhci_init();
	seaboard_power_init();
	seaboard_kbc_init();

	seaboard_wlan_init();
	seaboard_emc_init();

	tegra_gpio_enable(TEGRA_GPIO_LIDSWITCH);
	tegra_gpio_enable(TEGRA_GPIO_POWERKEY);
}

static void __init tegra_seaboard_init(void)
{
	/* Seaboard uses UARTD for the debug port. */
	debug_uart_platform_data[0].membase = IO_ADDRESS(TEGRA_UARTD_BASE);
	debug_uart_platform_data[0].mapbase = TEGRA_UARTD_BASE;
	debug_uart_platform_data[0].irq = INT_UARTD;

	__tegra_seaboard_init();

	seaboard_i2c_init();
}

static void __init tegra_kaen_init(void)
{
	/* Kaen uses UARTB for the debug port. */
	debug_uart_platform_data[0].membase = IO_ADDRESS(TEGRA_UARTB_BASE);
	debug_uart_platform_data[0].mapbase = TEGRA_UARTB_BASE;
	debug_uart_platform_data[0].irq = INT_UARTB;

	seaboard_kbc_platform_data.plain_keycode = cros_kbd_keycode;
	seaboard_kbc_platform_data.fn_keycode = cros_kbd_keycode;

	__tegra_seaboard_init();

	kaen_i2c_init();
}

static void __init tegra_aebl_init(void)
{
	/* Aebl uses UARTB for the debug port. */
	debug_uart_platform_data[0].membase = IO_ADDRESS(TEGRA_UARTB_BASE);
	debug_uart_platform_data[0].mapbase = TEGRA_UARTB_BASE;
	debug_uart_platform_data[0].irq = INT_UARTB;

	seaboard_kbc_platform_data.plain_keycode = cros_kbd_keycode;
	seaboard_kbc_platform_data.fn_keycode = cros_kbd_keycode;

	__tegra_seaboard_init();

	aebl_i2c_init();
}

static void __init tegra_wario_init(void)
{
	struct clk *c, *p;

	/* Wario uses UARTB for the debug port. */
	debug_uart_platform_data[0].membase = IO_ADDRESS(TEGRA_UARTB_BASE);
	debug_uart_platform_data[0].mapbase = TEGRA_UARTB_BASE;
	debug_uart_platform_data[0].irq = INT_UARTB;

	seaboard_kbc_platform_data.plain_keycode = cros_kbd_keycode;
	seaboard_kbc_platform_data.fn_keycode = cros_kbd_keycode;

	__tegra_seaboard_init();

	/* Temporary hack to keep eMMC controller at 24MHz */
	c = tegra_get_clock_by_name("sdmmc4");
	p = tegra_get_clock_by_name("pll_p");
	if (c && p) {
		clk_set_parent(c, p);
		clk_set_rate(c, 24000000);
		clk_enable(c);
	}

	wario_i2c_init();
}


MACHINE_START(SEABOARD, "seaboard")
	.boot_params    = 0x00000100,
	.init_irq       = tegra_init_irq,
	.init_machine   = tegra_seaboard_init,
	.map_io         = tegra_map_common_io,
	.timer          = &tegra_timer,
MACHINE_END

MACHINE_START(KAEN, "kaen")
	.boot_params    = 0x00000100,
	.init_irq       = tegra_init_irq,
	.init_machine   = tegra_kaen_init,
	.map_io         = tegra_map_common_io,
	.timer          = &tegra_timer,
MACHINE_END

MACHINE_START(AEBL, "aebl")
	.boot_params    = 0x00000100,
	.init_irq       = tegra_init_irq,
	.init_machine   = tegra_aebl_init,
	.map_io         = tegra_map_common_io,
	.timer          = &tegra_timer,
MACHINE_END

MACHINE_START(WARIO, "wario")
	.boot_params    = 0x00000100,
	.init_irq       = tegra_init_irq,
	.init_machine   = tegra_wario_init,
	.map_io         = tegra_map_common_io,
	.timer          = &tegra_timer,
MACHINE_END
