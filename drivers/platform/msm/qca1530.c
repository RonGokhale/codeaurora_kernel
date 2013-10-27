/* Copyright (c) 2013, The Linux Foundation. All rights reserved.
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
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>
#include <linux/types.h>

#define QCA1530_LOGD(msg, ...) \
	pr_notice("%s:%d] " msg "\n", __func__, __LINE__, ##__VA_ARGS__)
#define QCA1530_LOGI(msg, ...) \
	pr_info("%s:%d] " msg "\n" , __func__, __LINE__, ##__VA_ARGS__)
#define QCA1530_LOGW(msg, ...) \
	pr_warn("%s:%d] " msg "\n", __func__, __LINE__, ##__VA_ARGS__)
#define QCA1530_LOGE(msg, ...) \
	pr_err("%s:%d] " msg "\n", __func__, __LINE__, ##__VA_ARGS__)

#define QCA1530_RTC_CLK_ID		"rtc_clk"
#define QCA1530_RTC_CLK_RATE		(32 * 1000)
#define QCA1530_TCXO_CLK_ID		"tcxo_clk"
#define QCA1530_OF_PWR_REG_NAME		"pwr"
#define QCA1530_OF_PWR_REG2_NAME	"pwr2"
#define QCA1530_OF_PWR_GPIO_NAME	"pwr-gpio"
#define QCA1530_OF_CLK_GPIO_NAME	"clk-gpio"
#define QCA1530_OF_XLNA_REG_NAME	"xlna"
#define QCA1530_OF_XLNA_GPIO_NAME	"xlna-gpio"

#define QCA1530_PWR_MIN_UV		1800000
#define QCA1530_PWR_MAX_UV		1800000
#define QCA1530_PWR_LOAD_UA		90000

/**
 * \struct qca1530_static
 * \brief Structure to keep all driver variables
 *
 * \var qpio_reset    Number of GPIO pin for reset control
 * \var pdev          Driver data
 * \var rtc_clk       RTC clock handle (32KHz)
 * \var rtc_clk_state RTC clock state
 * \var pwr           Power controller
 *
 * \var qca1530_data
 * \brief Holds driver data
 *
 * \var qca1530_of_match
 * \brief Driver's device tree compatibility identifiers
 *
 * \var qca1530_driver
 * \brief Driver descriptor
 *
 * \fn qca1530_probe(struct platform_device *pdev)
 * \brief Driver's probing
 *
 * The driver probing includes initialization of the subsystems in the
 * following order:
 * - Reset control
 * - RTC Clock control
 * - Power control
 * - FS control interface
 '
 * \fn qca1530_remove(struct platform_device *pdev)
 * \brief Driver's removal
 *
 * Driver's removal stops subsystems in the following order:
 * - FS control interface
 * - Power control
 * - RTC Clock control
 * - Reset control
 *
 * \fn qca1530_init(void)
 * \brief Registers driver
 *
 * \fn qca1530_exit(void)
 * \brief Unregisters driver
 */

struct qca1530_static {
	struct platform_device	*pdev;
	int			gpio_reset;
	struct clk		*rtc_clk;
	int                     rtc_clk_state;
	int			rtc_clk_gpio;
	struct clk		*tcxo_clk;
	struct regulator	*pwr_reg;
	struct regulator	*pwr_reg2;
	int			pwr_gpio;
	struct regulator	*xlna_reg;
	int			xlna_gpio;
};

static int qca1530_probe(struct platform_device *pdev);
static int qca1530_remove(struct platform_device *pdev);

static void qca1530_deinit_gpio(int *pgpio);
static void qca1530_deinit_regulator(struct regulator **ppwr);

static int qca1530_pwr_init(struct platform_device *pdev);
static void qca1530_pwr_deinit(void);
static int qca1530_pwr_set(int status);
static int qca1530_pwr_set_regulator(struct regulator *pwr, int mode);
static void qca1530_pwr_set_gpio(int mode);
static int qca1530_pwr_init_regulator(
	struct platform_device *pdev,
	const char *name,
	struct regulator **ppwr);

static int qca1530_reset_init(struct platform_device *pdev);
static void qca1530_reset_deinit(void);

static int qca1530_clk_init(struct platform_device *pdev);
static void qca1530_clk_deinit(struct platform_device *pdev);
static int qca1530_clk_set(int status);
static void qca1530_clk_set_gpio(int mode);
static int qca1530_clk_set_rtc(struct clk *clk, int mode);
static void qca1530_clk_deinit_rtc(struct platform_device *pdev);

static int qca1530_xlna_init(struct platform_device *pdev);
static void qca1530_xlna_deinit(void);
static int qca1530_xlna_set(int status);

static struct qca1530_static	qca1530_data = {
	.gpio_reset = -1,
	.rtc_clk_state = -1,
	.rtc_clk_gpio = -1,
	.pwr_gpio = -1,
	.xlna_gpio = -1,
};

static struct of_device_id qca1530_of_match[] = {
	{.compatible = "qcom,qca1530", },
	{ },
};

static struct platform_driver qca1530_driver = {
	.probe		= qca1530_probe,
	.remove         = qca1530_remove,
	.driver         = {
		.name = "qca1530",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(qca1530_of_match),
	},
};

static int qca1530_probe(struct platform_device *pdev)
{
	int ret = -1;
	ret = qca1530_reset_init(pdev);
	if (ret < 0) {
		QCA1530_LOGE("failed to init reset: %d", ret);
		goto err_reset_init;
	}

	ret = qca1530_clk_init(pdev);
	if (ret) {
		QCA1530_LOGE("failed to initialize clock: %d", ret);
		goto err_clk_init;
	}

	ret = qca1530_xlna_init(pdev);
	if (ret) {
		QCA1530_LOGE("failed to initialize xLNA: %d", ret);
		goto err_xlna_init;
	}

	ret = qca1530_pwr_init(pdev);
	if (ret < 0) {
		QCA1530_LOGE("failed to init power: %d", ret);
		goto err_pwr_init;
	}

	qca1530_data.pdev = pdev;

	QCA1530_LOGI("Probe OK");

	return ret;

err_pwr_init:
	qca1530_xlna_deinit();
err_xlna_init:
	qca1530_clk_deinit(pdev);
err_clk_init:
	qca1530_reset_deinit();
err_reset_init:
	return ret;
}

static int qca1530_remove(struct platform_device *pdev)
{
	qca1530_pwr_deinit();
	qca1530_xlna_deinit();
	qca1530_clk_deinit(pdev);
	qca1530_reset_deinit();
	qca1530_data.pdev = NULL;
	return 0;
}

static int __init qca1530_init(void)
{
	return platform_driver_register(&qca1530_driver);
}

static void __exit qca1530_exit(void)
{
	platform_driver_unregister(&qca1530_driver);
}

module_init(qca1530_init);
module_exit(qca1530_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("qca1530 SoC chip driver");
MODULE_ALIAS("qca1530");

static void qca1530_deinit_gpio(int *pgpio)
{
	int gpio = *pgpio;

	if (gpio >= 0) {
		QCA1530_LOGI("Releasing GPIO: %d", gpio);

		gpio_unexport(gpio);
		gpio_direction_input(gpio);
		gpio_free(gpio);
		*pgpio = -1;
	}
}

static void qca1530_deinit_regulator(struct regulator **ppwr)
{
	if (*ppwr) {
		regulator_put(*ppwr);
		*ppwr = NULL;
	}
}

/*
 * \fn qca1530_clk_set(int status)
 * \brief Starts/stops RTC clock
 *
 * \param status Control code
 * \retval 0  On sucess
 * \retval <0 On error
 */
static int qca1530_clk_set(int mode)
{
	int ret = 0;

	if (qca1530_data.rtc_clk_gpio < 0 && !qca1530_data.rtc_clk)
		ret = -ENOSYS;
	else if (mode) {
		if (qca1530_data.rtc_clk_gpio >= 0)
			qca1530_clk_set_gpio(1);
		if (qca1530_data.rtc_clk)
			ret = qca1530_clk_set_rtc(qca1530_data.rtc_clk, 1);
		if (!ret && qca1530_data.tcxo_clk)
			ret = qca1530_clk_set_rtc(qca1530_data.tcxo_clk, 1);
		qca1530_data.rtc_clk_state = ret ? 0 : 1;
	} else {
		if (qca1530_data.tcxo_clk)
			ret = qca1530_clk_set_rtc(qca1530_data.tcxo_clk, 0);
		if (!ret && qca1530_data.rtc_clk)
			ret = qca1530_clk_set_rtc(qca1530_data.rtc_clk, 0);
		if (!ret && qca1530_data.rtc_clk_gpio >= 0)
			qca1530_clk_set_gpio(0);
		qca1530_data.rtc_clk_state = 0;
	}

	QCA1530_LOGI("Configured clk: mode=%d ret=%d", mode, ret);

	return ret;
}

static int qca1530_clk_set_rtc(struct clk *clk, int mode)
{
	int ret = 0;

	if (mode)
		ret = clk_prepare_enable(clk);
	else
		clk_disable_unprepare(clk);

	QCA1530_LOGI("Configured clk (%p): mode=%d ret=%d", clk, mode, ret);

	return ret;
}

static void qca1530_clk_set_gpio(int mode)
{
	gpio_set_value(qca1530_data.rtc_clk_gpio, mode ? 1 : 0);

	QCA1530_LOGI("Configured clk (GPIO): mode=%d", mode);
}

static void qca1530_clk_deinit_rtc(struct platform_device *pdev)
{
	if (qca1530_data.rtc_clk) {
		QCA1530_LOGI("Unregistering CLK: device=%s name=%s",
			dev_name(&pdev->dev), QCA1530_RTC_CLK_ID);
		qca1530_data.rtc_clk = NULL;
	}
}

static int qca1530_clk_init_gpio(struct platform_device *pdev)
{
	int ret;

	ret = of_get_named_gpio(pdev->dev.of_node, QCA1530_OF_CLK_GPIO_NAME, 0);
	if (ret == -ENOENT) {
		qca1530_data.rtc_clk_gpio = ret;
		ret = 0;
		QCA1530_LOGI("GPIO is not defined");
	} else if (ret < 0) {
		QCA1530_LOGE("GPIO error: %d", ret);
	} else {
		qca1530_data.rtc_clk_gpio = ret;
		QCA1530_LOGI("GPIO registered: gpio=%d", ret);
		ret = 0;
	}
	return ret;
}

static int qca1530_clk_init(struct platform_device *pdev)
{
	int ret = 0;

	QCA1530_LOGI("Clock initializing");

	qca1530_data.rtc_clk = clk_get(&pdev->dev, QCA1530_RTC_CLK_ID);
	if (IS_ERR_OR_NULL(qca1530_data.rtc_clk)) {
		ret = PTR_ERR(qca1530_data.rtc_clk);
		QCA1530_LOGE("error: device=%s clock=%s ret=%d",
			dev_name(&pdev->dev), QCA1530_RTC_CLK_ID, ret);
		qca1530_data.rtc_clk = NULL;
		goto err_0;
	}
	qca1530_data.tcxo_clk = clk_get(&pdev->dev, QCA1530_TCXO_CLK_ID);
	if (!qca1530_data.tcxo_clk)
		QCA1530_LOGI("No TCXO clock controller");
	else if (IS_ERR(qca1530_data.tcxo_clk)) {
		ret = PTR_ERR(qca1530_data.tcxo_clk);
		qca1530_data.tcxo_clk = NULL;
		if (ret == -ENOENT || ret == -EINVAL) {
			QCA1530_LOGI("No TCXO clock controller");
			ret = 0;
		} else {
			QCA1530_LOGE("error: device=%s clock=%s ret=%d",
				dev_name(&pdev->dev), QCA1530_TCXO_CLK_ID,
				ret);
			goto err_0;
		}
	}
	ret = qca1530_clk_init_gpio(pdev);
	if (ret)
		goto err_1;

	ret = qca1530_clk_set(1);
	if (ret < 0) {
		QCA1530_LOGE("error: ret=%d", ret);
		goto err_2;
	}

	QCA1530_LOGI("init done: GPIO=%s RTC=%s TCXO=%s",
		qca1530_data.rtc_clk_gpio >= 0 ? "ok" : "unused",
		qca1530_data.rtc_clk ? "ok" : "unused",
		qca1530_data.tcxo_clk ? "ok" : "unused");

	return 0;
err_2:
	qca1530_deinit_gpio(&qca1530_data.rtc_clk_gpio);
err_1:
	qca1530_clk_deinit_rtc(pdev);
err_0:
	qca1530_data.rtc_clk_state = -1;

	QCA1530_LOGE("init error: ret=%d", ret);

	return ret;
}

static void qca1530_clk_deinit(struct platform_device *pdev)
{
	qca1530_clk_set(0);
	qca1530_deinit_gpio(&qca1530_data.rtc_clk_gpio);
	qca1530_clk_deinit_rtc(pdev);
	qca1530_data.rtc_clk_state = -1;
}

/*
 * \fn  qca1530_power_set(int status)
 * \brief Sets current power status
 *
 * \param status New power status
 *
 * \retval 0  On success
 * \retval <0 On error
 */

static int qca1530_pwr_set(int mode)
{
	int ret = 0;
	if (!qca1530_data.pwr_reg && !qca1530_data.pwr_reg2
		&& qca1530_data.pwr_gpio < 0)
		ret = -ENOSYS;
	else if (mode) {
		if (qca1530_data.pwr_reg)
			ret = qca1530_pwr_set_regulator(
				qca1530_data.pwr_reg, mode);
		if (!ret && qca1530_data.pwr_reg2)
			ret = qca1530_pwr_set_regulator(
				qca1530_data.pwr_reg2, mode);
		if (!ret && qca1530_data.pwr_gpio >= 0)
			qca1530_pwr_set_gpio(mode);
	} else {
		if (qca1530_data.pwr_gpio >= 0)
			qca1530_pwr_set_gpio(mode);
		if (qca1530_data.pwr_reg2)
			ret = qca1530_pwr_set_regulator(
				qca1530_data.pwr_reg2, mode);
		if (!ret && qca1530_data.pwr_reg)
			ret = qca1530_pwr_set_regulator(
				qca1530_data.pwr_reg, mode);
	}
	return ret;
}

static void qca1530_pwr_set_gpio(int mode)
{
	gpio_set_value(qca1530_data.pwr_gpio, mode ? 1 : 0);

	QCA1530_LOGI("Configuring power(GPIO): mode=%d", mode);
}

static int qca1530_pwr_set_regulator(struct regulator *reg, int mode)
{
	int ret;

	QCA1530_LOGI("Setting regulator: mode=%d regulator=%p",
		mode, reg);

	if (mode) {
		if (regulator_is_enabled(reg)) {
			ret = 0;
			goto done;
		}

		ret = regulator_set_mode(reg, REGULATOR_MODE_NORMAL);
		if (ret)
			QCA1530_LOGW(
				"failed to set regulator mode, ret=%d", ret);

		ret = regulator_enable(reg);
		if (ret) {
			QCA1530_LOGE(
				"failed to enable regulator, rc=%d", ret);
			goto err;
		}
	} else {
		if (!regulator_is_enabled(reg)) {
			ret = 0;
			goto done;
		}

		ret = regulator_disable(reg);
		if (ret) {
			QCA1530_LOGE(
				"failed to disable regulator, rc=%d", ret);
			goto done;
		}
	}
done:
err:
	QCA1530_LOGI("Regulator result: regulator=%p mode=%d ret=%d",
		reg, mode, ret);

	return ret;
}

static int qca1530_pwr_init_gpio(struct platform_device *pdev)
{
	int ret;

	ret = of_get_named_gpio(pdev->dev.of_node, QCA1530_OF_PWR_GPIO_NAME, 0);
	if (ret == -ENOENT) {
		qca1530_data.pwr_gpio = ret;
		ret = 0;
		QCA1530_LOGI("Power control GPIO is not defined");
	} else if (ret < 0) {
		QCA1530_LOGE("Power control GPIO error: %d", ret);
	} else {
		qca1530_data.pwr_gpio = ret;
		ret = 0;
	}
	return ret;
}

static int qca1530_pwr_init_regulator(
	struct platform_device *pdev, const char *name, struct regulator **ppwr)
{
	int ret;
	struct regulator *pwr;

	pwr = regulator_get(&pdev->dev, name);
	if (IS_ERR_OR_NULL(pwr)) {
		ret = PTR_ERR(pwr);
		*ppwr = NULL;
		if (ret == -ENODEV) {
			QCA1530_LOGI("Power regulator %s is not defined",
				name);
			ret = 0;
		} else
			QCA1530_LOGE("Failed to get regulator, ret=%d", ret);
	} else {
		*ppwr = pwr;
		ret = 0;
	}

	return ret;
}


static int qca1530_pwr_init(struct platform_device *pdev)
{
	int ret = 0;

	QCA1530_LOGI("Initializing power control");

	ret = qca1530_pwr_init_regulator(
		pdev, QCA1530_OF_PWR_REG_NAME, &qca1530_data.pwr_reg);
	if (ret)
		goto err_0;

	ret = qca1530_pwr_init_regulator(
		pdev, QCA1530_OF_PWR_REG2_NAME, &qca1530_data.pwr_reg2);
	if (ret)
		goto err_1;


	ret = qca1530_pwr_init_gpio(pdev);
	if (ret)
		goto err_2;

	if (qca1530_data.pwr_reg || qca1530_data.pwr_reg2
		|| qca1530_data.pwr_gpio >= 0) {
		ret = qca1530_pwr_set(1);
		if (ret) {
			QCA1530_LOGE("Failed to enable power, rc=%d", ret);
			goto err_3;
		}
		QCA1530_LOGI("Configured: reg=%p reg2=%p gpio=%d",
			qca1530_data.pwr_reg,
			qca1530_data.pwr_reg2,
			qca1530_data.pwr_gpio);
	} else {
		QCA1530_LOGI("Power control is not available");
	}

	return ret;
err_3:
	qca1530_deinit_gpio(&qca1530_data.pwr_gpio);
err_2:
	qca1530_deinit_regulator(&qca1530_data.pwr_reg2);
err_1:
	qca1530_deinit_regulator(&qca1530_data.pwr_reg);
err_0:
	return ret;
}

static void qca1530_pwr_deinit(void)
{
	qca1530_pwr_set(0);
	qca1530_deinit_gpio(&qca1530_data.pwr_gpio);
	qca1530_deinit_regulator(&qca1530_data.pwr_reg2);
	qca1530_deinit_regulator(&qca1530_data.pwr_reg);
}

static int qca1530_reset_init(struct platform_device *pdev)
{
	int ret;
	struct device_node *np;

	QCA1530_LOGI("reset control: initializing");

	np = pdev->dev.of_node;
	ret = of_get_gpio(np, 0);
	if (ret < 0) {
		QCA1530_LOGE("failed to get gpio from config: %d", ret);
		goto err_gpio_get;
	}

	qca1530_data.gpio_reset = ret;
	ret = gpio_request(qca1530_data.gpio_reset, "qca1530-reset");

	if (ret < 0) {
		QCA1530_LOGE("failed to request gpio-%d",
			qca1530_data.gpio_reset);
		goto err_gpio_get;
	}

	ret = gpio_direction_output(qca1530_data.gpio_reset, 0);
	if (ret < 0) {
		QCA1530_LOGE("failed to change direction for gpio-%d",
			qca1530_data.gpio_reset);
		goto err_gpio_configure;
	}

	ret = gpio_export(qca1530_data.gpio_reset, false);
	if (ret < 0) {
		QCA1530_LOGE("failed to export gpio-%d for user",
			qca1530_data.gpio_reset);
		goto err_gpio_configure;
	}

	return 0;
err_gpio_configure:
	qca1530_deinit_gpio(&qca1530_data.gpio_reset);
err_gpio_get:
	return ret;
}

static void qca1530_reset_deinit(void)
{
	QCA1530_LOGI("reset control: releasing");
	qca1530_deinit_gpio(&qca1530_data.gpio_reset);
}

static int qca1530_xlna_init_gpio(struct platform_device *pdev)
{
	int ret;

	ret = of_get_named_gpio(pdev->dev.of_node, QCA1530_OF_XLNA_GPIO_NAME,
				0);
	if (ret == -ENOENT) {
		qca1530_data.xlna_gpio = -1;
		ret = 0;
		QCA1530_LOGI("xLNA control: GPIO is not defined");
	} else if (ret < 0) {
		QCA1530_LOGE("xLNA control: GPIO error: %d", ret);
	} else {
		qca1530_data.xlna_gpio = ret;
		ret = 0;
	}
	return ret;
}

static int qca1530_xlna_init(struct platform_device *pdev)
{
	int ret = 0;

	QCA1530_LOGI("xLNA control: initializing");

	ret = qca1530_pwr_init_regulator(
		pdev, QCA1530_OF_XLNA_REG_NAME, &qca1530_data.xlna_reg);
	if (ret)
		goto err_0;

	ret = qca1530_xlna_init_gpio(pdev);
	if (ret)
		goto err_1;

	if (qca1530_data.xlna_reg || qca1530_data.xlna_gpio >= 0) {
		ret = qca1530_xlna_set(1);
		if (ret) {
			QCA1530_LOGE("Failed to enable xLNA, rc=%d", ret);
			goto err_2;
		}
		QCA1530_LOGI("Configured: reg=%p gpio=%d",
			qca1530_data.xlna_reg,
			qca1530_data.xlna_gpio);
	} else {
		QCA1530_LOGI("xLNA control is not available");
	}

	return ret;
err_2:
	qca1530_deinit_gpio(&qca1530_data.xlna_gpio);
err_1:
	qca1530_deinit_regulator(&qca1530_data.xlna_reg);
err_0:
	return ret;
}

static void qca1530_xlna_deinit(void)
{
	qca1530_xlna_set(0);
	qca1530_deinit_gpio(&qca1530_data.xlna_gpio);
	qca1530_deinit_regulator(&qca1530_data.xlna_reg);
}

static int qca1530_xlna_set(int mode)
{
	int ret = 0;
	if (!qca1530_data.xlna_reg && qca1530_data.xlna_gpio < 0)
		ret = -ENOSYS;
	else if (mode) {
		if (qca1530_data.xlna_reg)
			ret = qca1530_pwr_set_regulator(
				qca1530_data.xlna_reg, mode);
		if (!ret && qca1530_data.xlna_gpio >= 0)
			gpio_set_value(qca1530_data.xlna_gpio, 1);
	} else {
		if (qca1530_data.xlna_gpio >= 0)
			gpio_set_value(qca1530_data.xlna_gpio, 0);
		if (qca1530_data.xlna_reg)
			ret = qca1530_pwr_set_regulator(
				qca1530_data.xlna_reg, mode);
	}
	return ret;
}

