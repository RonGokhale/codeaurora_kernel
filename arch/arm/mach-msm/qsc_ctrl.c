/* Copyright (c) 2011-2012, Code Aurora Forum. All rights reserved.
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
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/irq.h>
#include <linux/ioctl.h>
#include <linux/delay.h>
#include <linux/reboot.h>
#include <linux/debugfs.h>
#include <linux/completion.h>
#include <linux/workqueue.h>
#include <linux/clk.h>
#include <linux/mfd/pmic8058.h>
#include <linux/msm_charm.h>
#include <asm/mach-types.h>
#include <asm/uaccess.h>
#include <mach/mdm2.h>
#include <mach/restart.h>
#include <mach/subsystem_notif.h>
#include <mach/subsystem_restart.h>
#include <mach/rpm.h>
#include <mach/gpiomux.h>
#include "msm_watchdog.h"
#include "sysmon.h"

#define MDM_DEBUG_MASK_VDDMIN_SETUP (0x00000002)
#define MDM_DEBUG_MASK_SHDN_LOG     (0x00000004)
#define GPIO_IS_VALID(gpio) \
	(gpio != -1)
struct qsc_modem_drv;

struct mdm_ops {
	void (*power_on_mdm_cb)(struct qsc_modem_drv *mdm_drv);
	void (*reset_mdm_cb)(struct qsc_modem_drv *mdm_drv);
	void (*atomic_reset_mdm_cb)(struct qsc_modem_drv *mdm_drv);
	void (*normal_boot_done_cb)(struct qsc_modem_drv *mdm_drv);
	void (*power_down_mdm_cb)(struct qsc_modem_drv *mdm_drv);
	void (*debug_state_changed_cb)(int value);
	void (*status_cb)(struct qsc_modem_drv *mdm_drv, int value);
	void (*image_upgrade_cb)(struct qsc_modem_drv *mdm_drv, int type);
};

/* Private mdm2 data structure */
struct qsc_modem_drv {
	unsigned mdm2ap_errfatal_gpio;
	unsigned ap2mdm_errfatal_gpio;
	unsigned mdm2ap_status_gpio;
	unsigned ap2mdm_status_gpio;
	unsigned mdm2ap_wakeup_gpio;
	unsigned ap2mdm_wakeup_gpio;
	unsigned ap2mdm_kpdpwr_n_gpio;
	unsigned ap2mdm_soft_reset_gpio;
	unsigned ap2mdm_pmic_pwr_en_gpio;
	unsigned mdm2ap_pblrdy;
	unsigned usb_switch_gpio;

	int mdm_errfatal_irq;
	int mdm_status_irq;
	int mdm_ready;
	int mdm_boot_status;
	int mdm_ram_dump_status;
	enum charm_boot_type boot_type;
	int mdm_debug_on;
	int mdm_unexpected_reset_occurred;
	int disable_status_check;

	struct mdm_ops *ops;
	struct mdm_platform_data *pdata;
};

#define MDM_MODEM_TIMEOUT	6000
#define MDM_MODEM_DELTA	100
#define MDM_BOOT_TIMEOUT	60000L
#define MDM_RDUMP_TIMEOUT	120000L
#define MDM2AP_STATUS_TIMEOUT_MS 60000L
#define MDM_PBLRDY_CNT		20

static int power_on_count;
static int hsic_peripheral_status;
static DEFINE_MUTEX(hsic_status_lock);

static unsigned int qsc_debug_mask;
static struct workqueue_struct *qsc_queue;
static struct workqueue_struct *qsc_sfr_queue;
static unsigned int dump_timeout_ms;
static int vddmin_gpios_sent;

#define EXTERNAL_MODEM "qsc_modem"

static struct qsc_modem_drv *mdm_drv;

DECLARE_COMPLETION(qsc_needs_reload);
DECLARE_COMPLETION(qsc_boot);
DECLARE_COMPLETION(qsc_ram_dumps);

static int first_boot = 1;

#define RD_BUF_SIZE			100
#define SFR_MAX_RETRIES		10
#define SFR_RETRY_INTERVAL	1000

enum gpio_update_config {
	GPIO_UPDATE_BOOTING_CONFIG = 1,
	GPIO_UPDATE_RUNNING_CONFIG,
};
static int mdm2ap_status_valid_old_config;
static struct gpiomux_setting mdm2ap_status_old_config;

static irqreturn_t qsc_vddmin_change(int irq, void *dev_id)
{
	int value = gpio_get_value(
		mdm_drv->pdata->vddmin_resource->mdm2ap_vddmin_gpio);

	if (value == 0)
		pr_info("External Modem entered Vddmin\n");
	else
		pr_info("External Modem exited Vddmin\n");

	return IRQ_HANDLED;
}

static void qsc_setup_vddmin_gpios(void)
{
	struct msm_rpm_iv_pair req;
	struct mdm_vddmin_resource *vddmin_res;
	int irq, ret;

	/* This resource may not be supported by some platforms. */
	vddmin_res = mdm_drv->pdata->vddmin_resource;
	if (!vddmin_res)
		return;

	pr_info("Enabling vddmin logging\n");
	req.id = vddmin_res->rpm_id;
	req.value = ((uint32_t)vddmin_res->ap2mdm_vddmin_gpio & 0x0000FFFF)
							<< 16;
	req.value |= ((uint32_t)vddmin_res->modes & 0x000000FF) << 8;
	req.value |= (uint32_t)vddmin_res->drive_strength & 0x000000FF;

	msm_rpm_set(MSM_RPM_CTX_SET_0, &req, 1);

	/* Start monitoring low power gpio from mdm */
	irq = MSM_GPIO_TO_INT(vddmin_res->mdm2ap_vddmin_gpio);
	if (irq < 0) {
		pr_err("%s: could not get LPM POWER IRQ resource.\n",
			__func__);
		goto error_end;
	}

	ret = request_threaded_irq(irq, NULL, qsc_vddmin_change,
		IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
		"mdm lpm", NULL);

	if (ret < 0)
		pr_err("%s: MDM LPM IRQ#%d request failed with error=%d",
			__func__, irq, ret);
error_end:
	return;
}

static void qsc_restart_reason_fn(struct work_struct *work)
{
	int ret, ntries = 0;
	char sfr_buf[RD_BUF_SIZE];

	do {
		msleep(SFR_RETRY_INTERVAL);
		ret = sysmon_get_reason(SYSMON_SS_EXT_MODEM,
					sfr_buf, sizeof(sfr_buf));
		if (ret) {
			/*
			 * The sysmon device may not have been probed as yet
			 * after the restart.
			 */
			pr_err("%s: Error retrieving qsc restart reason, ret = %d\n",
					__func__, ret);
		} else {
			pr_err("qsc restart reason: %s\n", sfr_buf);
			break;
		}
	} while (++ntries < SFR_MAX_RETRIES);
}

static DECLARE_WORK(sfr_reason_work, qsc_restart_reason_fn);

static void qsc2ap_status_check(struct work_struct *work)
{
	/*
	 * If the mdm modem did not pull the MDM2AP_STATUS gpio
	 * high then call subsystem_restart.
	 */
	if (!mdm_drv->disable_status_check) {
		if (gpio_get_value(mdm_drv->mdm2ap_status_gpio) == 0) {
			pr_err("%s: QSC2AP_STATUS gpio did not go high\n",
					__func__);
			mdm_drv->mdm_ready = 0;
			subsystem_restart(EXTERNAL_MODEM);
		}
	}
}

static DECLARE_DELAYED_WORK(qsc2ap_status_check_work, qsc2ap_status_check);

static void qsc_update_gpio_configs(enum gpio_update_config gpio_config)
{
	/* Some gpio configuration may need updating after modem bootup.*/
	switch (gpio_config) {
	case GPIO_UPDATE_RUNNING_CONFIG:
		if (mdm_drv->pdata->mdm2ap_status_gpio_run_cfg) {
			if (msm_gpiomux_write(mdm_drv->mdm2ap_status_gpio,
				GPIOMUX_ACTIVE,
				mdm_drv->pdata->mdm2ap_status_gpio_run_cfg,
				&mdm2ap_status_old_config))
				pr_err("%s: failed updating running gpio config\n",
					   __func__);
			else
				mdm2ap_status_valid_old_config = 1;
		}
		break;
	case GPIO_UPDATE_BOOTING_CONFIG:
		if (mdm2ap_status_valid_old_config) {
			msm_gpiomux_write(mdm_drv->mdm2ap_status_gpio,
					GPIOMUX_ACTIVE,
					&mdm2ap_status_old_config,
					NULL);
			mdm2ap_status_valid_old_config = 0;
		}
		break;
	default:
		pr_err("%s: called with no config\n", __func__);
		break;
	}
}

long qsc_modem_ioctl(struct file *filp, unsigned int cmd,
				unsigned long arg)
{
	int status, ret = 0;

	if (_IOC_TYPE(cmd) != CHARM_CODE) {
		pr_err("%s: invalid ioctl code\n", __func__);
		return -EINVAL;
	}

	pr_debug("%s: Entering ioctl cmd = %d\n", __func__, _IOC_NR(cmd));
	switch (cmd) {
	case WAKE_CHARM:
		pr_info("%s: Powering on qsc\n", __func__);
		mdm_drv->ops->power_on_mdm_cb(mdm_drv);
		break;
	case CHECK_FOR_BOOT:
		if (gpio_get_value(mdm_drv->mdm2ap_status_gpio) == 0)
			put_user(1, (unsigned long __user *) arg);
		else
			put_user(0, (unsigned long __user *) arg);
		break;
	case NORMAL_BOOT_DONE:
		pr_debug("%s: check if qsc is booted up\n", __func__);
		get_user(status, (unsigned long __user *) arg);
		if (status) {
			pr_debug("%s: normal boot failed\n", __func__);
			mdm_drv->mdm_boot_status = -EIO;
		} else {
			pr_info("%s: normal boot done\n", __func__);
			mdm_drv->mdm_boot_status = 0;
		}
		mdm_drv->mdm_ready = 1;

		if (mdm_drv->ops->normal_boot_done_cb != NULL)
			mdm_drv->ops->normal_boot_done_cb(mdm_drv);

		if (!first_boot)
			complete(&qsc_boot);
		else
			first_boot = 0;

		/* If successful, start a timer to check that the mdm2ap_status
		 * gpio goes high.
		 */
		if (!status && gpio_get_value(mdm_drv->mdm2ap_status_gpio) == 0)
			schedule_delayed_work(&qsc2ap_status_check_work,
				msecs_to_jiffies(MDM2AP_STATUS_TIMEOUT_MS));
		break;
	case RAM_DUMP_DONE:
		pr_debug("%s: qsc done collecting RAM dumps\n", __func__);
		get_user(status, (unsigned long __user *) arg);
		if (status)
			mdm_drv->mdm_ram_dump_status = -EIO;
		else {
			pr_info("%s: ramdump collection completed\n", __func__);
			mdm_drv->mdm_ram_dump_status = 0;
		}
		complete(&qsc_ram_dumps);
		break;
	case WAIT_FOR_RESTART:
		pr_debug("%s: wait for qsc to need images reloaded\n",
				__func__);
		ret = wait_for_completion_interruptible(&qsc_needs_reload);
		if (!ret)
			put_user(mdm_drv->boot_type,
					 (unsigned long __user *) arg);
		INIT_COMPLETION(qsc_needs_reload);
		break;
	case GET_DLOAD_STATUS:
		pr_debug("getting status of qsc2ap_errfatal_gpio\n");
		if (gpio_get_value(mdm_drv->mdm2ap_errfatal_gpio) == 1 &&
			!mdm_drv->mdm_ready)
			put_user(1, (unsigned long __user *) arg);
		else
			put_user(0, (unsigned long __user *) arg);
		break;
	case IMAGE_UPGRADE:
		pr_debug("%s Image upgrade ioctl recieved\n", __func__);
		if (mdm_drv->pdata->image_upgrade_supported &&
				mdm_drv->ops->image_upgrade_cb) {
			get_user(status, (unsigned long __user *) arg);
			mdm_drv->ops->image_upgrade_cb(mdm_drv, status);
		} else
			pr_debug("%s Image upgrade not supported\n", __func__);
		break;
	case SHUTDOWN_CHARM:
		if (!mdm_drv->pdata->send_shdn)
			break;
		mdm_drv->mdm_ready = 0;
		if (qsc_debug_mask & MDM_DEBUG_MASK_SHDN_LOG)
			pr_info("Sending shutdown request to mdm\n");
		ret = sysmon_send_shutdown(SYSMON_SS_EXT_MODEM);
		if (ret)
			pr_err("%s: Graceful shutdown of the external modem failed, ret = %d\n",
				   __func__, ret);
		break;
	default:
		pr_err("%s: invalid ioctl cmd = %d\n", __func__, _IOC_NR(cmd));
		ret = -EINVAL;
		break;
	}

	return ret;
}

static void qsc_status_fn(struct work_struct *work)
{
	int value = gpio_get_value(mdm_drv->mdm2ap_status_gpio);

	pr_debug("%s: status:%d\n", __func__, value);
	if (mdm_drv->mdm_ready && mdm_drv->ops->status_cb)
		mdm_drv->ops->status_cb(mdm_drv, value);

	/* Update gpio configuration to "running" config. */
	qsc_update_gpio_configs(GPIO_UPDATE_RUNNING_CONFIG);
}

static DECLARE_WORK(qsc_status_work, qsc_status_fn);

static void qsc_disable_irqs(void)
{
	disable_irq_nosync(mdm_drv->mdm_errfatal_irq);
	disable_irq_nosync(mdm_drv->mdm_status_irq);
}

static irqreturn_t qsc_errfatal(int irq, void *dev_id)
{
	pr_debug("%s: qsc got errfatal interrupt\n", __func__);
	if (mdm_drv->mdm_ready &&
		(gpio_get_value(mdm_drv->mdm2ap_status_gpio) == 1)) {
		pr_info("%s: Reseting the qsc due to an errfatal\n", __func__);
		mdm_drv->mdm_ready = 0;
		subsystem_restart(EXTERNAL_MODEM);
	}
	return IRQ_HANDLED;
}

static int qsc_modem_open(struct inode *inode, struct file *file)
{
	return 0;
}

static const struct file_operations qsc_modem_fops = {
	.owner		= THIS_MODULE,
	.open		= qsc_modem_open,
	.unlocked_ioctl	= qsc_modem_ioctl,
};


static struct miscdevice qsc_modem_misc = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "mdm1",
	.fops	= &qsc_modem_fops
};

static int qsc_panic_prep(struct notifier_block *this,
				unsigned long event, void *ptr)
{
	int i;

	pr_debug("%s: setting AP2QSC_ERRFATAL high for a non graceful reset\n",
			 __func__);
	qsc_disable_irqs();
	gpio_set_value(mdm_drv->ap2mdm_errfatal_gpio, 1);

	for (i = MDM_MODEM_TIMEOUT; i > 0; i -= MDM_MODEM_DELTA) {
		pet_watchdog();
		mdelay(MDM_MODEM_DELTA);
		if (gpio_get_value(mdm_drv->mdm2ap_status_gpio) == 0)
			break;
	}
	if (i <= 0) {
		pr_err("%s: QSC2AP_STATUS never went low\n", __func__);
		/* Reset the modem so that it will go into download mode. */
		if (mdm_drv && mdm_drv->ops->atomic_reset_mdm_cb)
			mdm_drv->ops->atomic_reset_mdm_cb(mdm_drv);
	}
	return NOTIFY_DONE;
}

static struct notifier_block qsc_panic_blk = {
	.notifier_call  = qsc_panic_prep,
};

static irqreturn_t qsc_status_change(int irq, void *dev_id)
{
	int value = gpio_get_value(mdm_drv->mdm2ap_status_gpio);

	if ((qsc_debug_mask & MDM_DEBUG_MASK_SHDN_LOG) && (value == 0))
		pr_info("%s: qsc2ap_status went low\n", __func__);

	pr_debug("%s: qscs sent status change interrupt\n", __func__);
	if (value == 0 && mdm_drv->mdm_ready == 1) {
		pr_info("%s: unexpected reset external modem\n", __func__);
		mdm_drv->mdm_unexpected_reset_occurred = 1;
		mdm_drv->mdm_ready = 0;
		subsystem_restart(EXTERNAL_MODEM);
	} else if (value == 1) {
		cancel_delayed_work(&qsc2ap_status_check_work);
		pr_info("%s: status = 1: qsc is now ready\n", __func__);
		queue_work(qsc_queue, &qsc_status_work);
	}
	return IRQ_HANDLED;
}

static irqreturn_t qsc_pblrdy_change(int irq, void *dev_id)
{
	pr_info("%s: pbl ready:%d\n", __func__,
			gpio_get_value(mdm_drv->mdm2ap_pblrdy));

	return IRQ_HANDLED;
}

static int qsc_subsys_shutdown(const struct subsys_data *crashed_subsys)
{
	mdm_drv->mdm_ready = 0;
	cancel_delayed_work(&qsc2ap_status_check_work);
	gpio_direction_output(mdm_drv->ap2mdm_errfatal_gpio, 1);
	if (mdm_drv->pdata->ramdump_delay_ms > 0) {
		/* Wait for the external modem to complete
		 * its preparation for ramdumps.
		 */
		msleep(mdm_drv->pdata->ramdump_delay_ms);
	}
	if (!mdm_drv->mdm_unexpected_reset_occurred) {
		mdm_drv->ops->reset_mdm_cb(mdm_drv);
		/* Update gpio configuration to "booting" config. */
		qsc_update_gpio_configs(GPIO_UPDATE_BOOTING_CONFIG);
	} else {
		mdm_drv->mdm_unexpected_reset_occurred = 0;
	}
	return 0;
}

static int qsc_subsys_powerup(const struct subsys_data *crashed_subsys)
{
	gpio_direction_output(mdm_drv->ap2mdm_errfatal_gpio, 0);
	gpio_direction_output(mdm_drv->ap2mdm_status_gpio, 1);
	mdm_drv->ops->power_on_mdm_cb(mdm_drv);
	mdm_drv->boot_type = CHARM_NORMAL_BOOT;
	complete(&qsc_needs_reload);
	if (!wait_for_completion_timeout(&qsc_boot,
			msecs_to_jiffies(MDM_BOOT_TIMEOUT))) {
		mdm_drv->mdm_boot_status = -ETIMEDOUT;
		pr_info("%s: qsc modem restart timed out.\n", __func__);
	} else {
		pr_info("%s: qsc modem has been restarted\n", __func__);

		/* Log the reason for the restart */
		if (mdm_drv->pdata->sfr_query)
			queue_work(qsc_sfr_queue, &sfr_reason_work);
	}
	INIT_COMPLETION(qsc_boot);
	return mdm_drv->mdm_boot_status;
}

static int qsc_subsys_ramdumps(int want_dumps,
				const struct subsys_data *crashed_subsys)
{
	mdm_drv->mdm_ram_dump_status = 0;
	cancel_delayed_work(&qsc2ap_status_check_work);
	if (want_dumps) {
		mdm_drv->boot_type = CHARM_RAM_DUMPS;
		complete(&qsc_needs_reload);
		if (!wait_for_completion_timeout(&qsc_ram_dumps,
				msecs_to_jiffies(dump_timeout_ms))) {
			mdm_drv->mdm_ram_dump_status = -ETIMEDOUT;
			pr_info("%s: qsc modem ramdumps timed out.\n",
					__func__);
		} else
			pr_info("%s: qsc modem ramdumps completed.\n",
					__func__);
		INIT_COMPLETION(qsc_ram_dumps);
		if (!mdm_drv->pdata->no_powerdown_after_ramdumps) {
			mdm_drv->ops->power_down_mdm_cb(mdm_drv);
			/* Update gpio configuration to "booting" config. */
			qsc_update_gpio_configs(GPIO_UPDATE_BOOTING_CONFIG);
		}
	}
	return mdm_drv->mdm_ram_dump_status;
}

static struct subsys_data qsc_subsystem = {
	.shutdown = qsc_subsys_shutdown,
	.ramdump = qsc_subsys_ramdumps,
	.powerup = qsc_subsys_powerup,
	.name = EXTERNAL_MODEM,
};

/* Once the gpios are sent to RPM and debugging
 * starts, there is no way to stop it without
 * rebooting the device.
 */
static int qsc_debug_mask_set(void *data, u64 val)
{
	if (!vddmin_gpios_sent &&
		(val & MDM_DEBUG_MASK_VDDMIN_SETUP)) {
		qsc_setup_vddmin_gpios();
		vddmin_gpios_sent = 1;
	}

	qsc_debug_mask = val;
	if (mdm_drv->ops->debug_state_changed_cb)
		mdm_drv->ops->debug_state_changed_cb(qsc_debug_mask);
	return 0;
}

static int qsc_debug_mask_get(void *data, u64 *val)
{
	*val = qsc_debug_mask;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(qsc_debug_mask_fops,
			qsc_debug_mask_get,
			qsc_debug_mask_set, "%llu\n");

static int qsc_debugfs_init(void)
{
	struct dentry *dent;

	dent = debugfs_create_dir("mdm_dbg", 0);
	if (IS_ERR(dent))
		return PTR_ERR(dent);

	debugfs_create_file("debug_mask", 0644, dent, NULL,
			&qsc_debug_mask_fops);
	return 0;
}

static void qsc_modem_initialize_data(struct platform_device  *pdev,
				struct mdm_ops *mdm_ops)
{
	struct resource *pres;

	/* MDM2AP_ERRFATAL */
	pres = platform_get_resource_byname(pdev, IORESOURCE_IO,
							"MDM2AP_ERRFATAL");
	mdm_drv->mdm2ap_errfatal_gpio = pres ? pres->start : -1;

	/* AP2MDM_ERRFATAL */
	pres = platform_get_resource_byname(pdev, IORESOURCE_IO,
							"AP2MDM_ERRFATAL");
	mdm_drv->ap2mdm_errfatal_gpio = pres ? pres->start : -1;

	/* MDM2AP_STATUS */
	pres = platform_get_resource_byname(pdev, IORESOURCE_IO,
							"MDM2AP_STATUS");
	mdm_drv->mdm2ap_status_gpio = pres ? pres->start : -1;

	/* AP2MDM_STATUS */
	pres = platform_get_resource_byname(pdev, IORESOURCE_IO,
							"AP2MDM_STATUS");
	mdm_drv->ap2mdm_status_gpio = pres ? pres->start : -1;

	/* MDM2AP_WAKEUP */
	pres = platform_get_resource_byname(pdev, IORESOURCE_IO,
							"MDM2AP_WAKEUP");
	mdm_drv->mdm2ap_wakeup_gpio = pres ? pres->start : -1;

	/* AP2MDM_WAKEUP */
	pres = platform_get_resource_byname(pdev, IORESOURCE_IO,
							"AP2MDM_WAKEUP");
	mdm_drv->ap2mdm_wakeup_gpio = pres ? pres->start : -1;

	/* AP2MDM_SOFT_RESET */
	pres = platform_get_resource_byname(pdev, IORESOURCE_IO,
							"AP2MDM_SOFT_RESET");
	mdm_drv->ap2mdm_soft_reset_gpio = pres ? pres->start : -1;

	/* AP2MDM_KPDPWR_N */
	pres = platform_get_resource_byname(pdev, IORESOURCE_IO,
							"AP2MDM_KPDPWR_N");
	mdm_drv->ap2mdm_kpdpwr_n_gpio = pres ? pres->start : -1;

	/* AP2MDM_PMIC_PWR_EN */
	pres = platform_get_resource_byname(pdev, IORESOURCE_IO,
							"AP2MDM_PMIC_PWR_EN");
	mdm_drv->ap2mdm_pmic_pwr_en_gpio = pres ? pres->start : -1;

	/* MDM2AP_PBLRDY */
	pres = platform_get_resource_byname(pdev, IORESOURCE_IO,
							"MDM2AP_PBLRDY");
	mdm_drv->mdm2ap_pblrdy = pres ? pres->start : -1;

	/*USB_SW*/
	pres = platform_get_resource_byname(pdev, IORESOURCE_IO,
							"USB_SW");
	mdm_drv->usb_switch_gpio = pres ? pres->start : -1;

	mdm_drv->boot_type                  = CHARM_NORMAL_BOOT;

	mdm_drv->ops      = mdm_ops;
	mdm_drv->pdata    = pdev->dev.platform_data;
	dump_timeout_ms = mdm_drv->pdata->ramdump_timeout_ms > 0 ?
		mdm_drv->pdata->ramdump_timeout_ms : MDM_RDUMP_TIMEOUT;
}

int qsc_common_create(struct platform_device  *pdev,
					  struct mdm_ops *p_mdm_cb)
{
	int ret = -1, irq;

	mdm_drv = kzalloc(sizeof(struct qsc_modem_drv), GFP_KERNEL);
	if (mdm_drv == NULL) {
		pr_err("%s: kzalloc fail.\n", __func__);
		goto alloc_err;
	}

	qsc_modem_initialize_data(pdev, p_mdm_cb);
	if (mdm_drv->ops->debug_state_changed_cb)
		mdm_drv->ops->debug_state_changed_cb(qsc_debug_mask);

	gpio_request(mdm_drv->ap2mdm_status_gpio, "AP2MDM_STATUS");
	gpio_request(mdm_drv->ap2mdm_errfatal_gpio, "AP2MDM_ERRFATAL");
	if (GPIO_IS_VALID(mdm_drv->ap2mdm_kpdpwr_n_gpio))
		gpio_request(mdm_drv->ap2mdm_kpdpwr_n_gpio, "AP2MDM_KPDPWR_N");
	gpio_request(mdm_drv->mdm2ap_status_gpio, "MDM2AP_STATUS");
	gpio_request(mdm_drv->mdm2ap_errfatal_gpio, "MDM2AP_ERRFATAL");
	if (GPIO_IS_VALID(mdm_drv->mdm2ap_pblrdy))
		gpio_request(mdm_drv->mdm2ap_pblrdy, "MDM2AP_PBLRDY");

	if (GPIO_IS_VALID(mdm_drv->ap2mdm_pmic_pwr_en_gpio))
		gpio_request(mdm_drv->ap2mdm_pmic_pwr_en_gpio,
					 "AP2MDM_PMIC_PWR_EN");
	if (GPIO_IS_VALID(mdm_drv->ap2mdm_soft_reset_gpio))
		gpio_request(mdm_drv->ap2mdm_soft_reset_gpio,
					 "AP2MDM_SOFT_RESET");

	if (GPIO_IS_VALID(mdm_drv->ap2mdm_wakeup_gpio))
		gpio_request(mdm_drv->ap2mdm_wakeup_gpio, "AP2MDM_WAKEUP");

	if (GPIO_IS_VALID(mdm_drv->usb_switch_gpio)) {
		if (gpio_request(mdm_drv->usb_switch_gpio, "USB_SW")) {
			pr_err("%s Failed to get usb switch gpio\n", __func__);
			mdm_drv->usb_switch_gpio = -1;
		}
	}

	gpio_direction_output(mdm_drv->ap2mdm_status_gpio, 1);
	gpio_direction_output(mdm_drv->ap2mdm_errfatal_gpio, 0);

	if (GPIO_IS_VALID(mdm_drv->ap2mdm_wakeup_gpio))
		gpio_direction_output(mdm_drv->ap2mdm_wakeup_gpio, 0);

	gpio_direction_input(mdm_drv->mdm2ap_status_gpio);
	gpio_direction_input(mdm_drv->mdm2ap_errfatal_gpio);

	qsc_queue = create_singlethread_workqueue("qsc_queue");
	if (!qsc_queue) {
		pr_err("%s: could not create workqueue.\n", __func__);
		ret = -ENOMEM;
		goto fatal_err;
	}

	qsc_sfr_queue = alloc_workqueue("qsc_sfr_queue", 0, 0);
	if (!qsc_sfr_queue) {
		pr_err("%s: could not create workqueue mdm_sfr_queue\n",
			   __func__);
		ret = -ENOMEM;
		destroy_workqueue(qsc_queue);
		goto fatal_err;
	}

	atomic_notifier_chain_register(&panic_notifier_list, &qsc_panic_blk);
	qsc_debugfs_init();

	/* Register subsystem handlers */
	ssr_register_subsystem(&qsc_subsystem);

	/* ERR_FATAL irq. */
	irq = MSM_GPIO_TO_INT(mdm_drv->mdm2ap_errfatal_gpio);
	if (irq < 0) {
		pr_err("%s: could not get QSC2AP_ERRFATAL IRQ resource, error=%d\n",
			__func__, irq);
		goto errfatal_err;
	}
	ret = request_irq(irq, qsc_errfatal,
		IRQF_TRIGGER_RISING , "mdm errfatal", NULL);

	if (ret < 0) {
		pr_err("%s: QSC2AP_ERRFATAL IRQ#%d request failed, error=%d\n",
			__func__, irq, ret);
		goto errfatal_err;
	}
	mdm_drv->mdm_errfatal_irq = irq;

errfatal_err:

	/* status irq */
	irq = MSM_GPIO_TO_INT(mdm_drv->mdm2ap_status_gpio);
	if (irq < 0) {
		pr_err("%s: could not get QSC2AP_STATUS IRQ resource, error=%d\n",
			__func__, irq);
		goto status_err;
	}

	ret = request_threaded_irq(irq, NULL, qsc_status_change,
		IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_SHARED,
		"mdm status", mdm_drv);

	if (ret < 0) {
		pr_err("%s: QSC2AP_STATUS IRQ#%d request failed, error=%d\n",
			__func__, irq, ret);
		goto status_err;
	}
	mdm_drv->mdm_status_irq = irq;

status_err:
	if (GPIO_IS_VALID(mdm_drv->mdm2ap_pblrdy)) {
		irq = MSM_GPIO_TO_INT(mdm_drv->mdm2ap_pblrdy);
		if (irq < 0) {
			pr_err("%s: could not get QSC2AP_PBLRDY IRQ resource",
				__func__);
			goto pblrdy_err;
		}

		ret = request_threaded_irq(irq, NULL, qsc_pblrdy_change,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING |
			IRQF_SHARED,
			"mdm pbl ready", mdm_drv);

		if (ret < 0) {
			pr_err("%s: QSC2AP_PBL IRQ#%d request failed error=%d",
				__func__, irq, ret);
			goto pblrdy_err;
		}
	}

pblrdy_err:
	/*
	 * If AP2MDM_PMIC_PWR_EN gpio is used, pull it high. It remains
	 * high until the whole phone is shut down.
	 */
	if (GPIO_IS_VALID(mdm_drv->ap2mdm_pmic_pwr_en_gpio))
		gpio_direction_output(mdm_drv->ap2mdm_pmic_pwr_en_gpio, 1);

	/* Perform early powerup of the external modem in order to
	 * allow tabla devices to be found.
	 */
	if (mdm_drv->pdata->early_power_on)
		mdm_drv->ops->power_on_mdm_cb(mdm_drv);

	pr_info("%s: Registering qsc modem\n", __func__);
	return misc_register(&qsc_modem_misc);

fatal_err:
	gpio_free(mdm_drv->ap2mdm_status_gpio);
	gpio_free(mdm_drv->ap2mdm_errfatal_gpio);
	if (GPIO_IS_VALID(mdm_drv->ap2mdm_kpdpwr_n_gpio))
		gpio_free(mdm_drv->ap2mdm_kpdpwr_n_gpio);
	if (GPIO_IS_VALID(mdm_drv->ap2mdm_pmic_pwr_en_gpio))
		gpio_free(mdm_drv->ap2mdm_pmic_pwr_en_gpio);
	gpio_free(mdm_drv->mdm2ap_status_gpio);
	gpio_free(mdm_drv->mdm2ap_errfatal_gpio);
	if (GPIO_IS_VALID(mdm_drv->ap2mdm_soft_reset_gpio))
		gpio_free(mdm_drv->ap2mdm_soft_reset_gpio);

	if (GPIO_IS_VALID(mdm_drv->ap2mdm_wakeup_gpio))
		gpio_free(mdm_drv->ap2mdm_wakeup_gpio);

	kfree(mdm_drv);
	ret = -ENODEV;

alloc_err:
	return ret;
}

int qsc_common_modem_remove(struct platform_device *pdev)
{
	int ret;

	gpio_free(mdm_drv->ap2mdm_status_gpio);
	gpio_free(mdm_drv->ap2mdm_errfatal_gpio);
	if (GPIO_IS_VALID(mdm_drv->ap2mdm_kpdpwr_n_gpio))
		gpio_free(mdm_drv->ap2mdm_kpdpwr_n_gpio);
	if (GPIO_IS_VALID(mdm_drv->ap2mdm_pmic_pwr_en_gpio))
		gpio_free(mdm_drv->ap2mdm_pmic_pwr_en_gpio);
	gpio_free(mdm_drv->mdm2ap_status_gpio);
	gpio_free(mdm_drv->mdm2ap_errfatal_gpio);
	if (GPIO_IS_VALID(mdm_drv->ap2mdm_soft_reset_gpio))
		gpio_free(mdm_drv->ap2mdm_soft_reset_gpio);

	if (GPIO_IS_VALID(mdm_drv->ap2mdm_wakeup_gpio))
		gpio_free(mdm_drv->ap2mdm_wakeup_gpio);

	kfree(mdm_drv);

	ret = misc_deregister(&qsc_modem_misc);
	return ret;
}

void qsc_common_modem_shutdown(struct platform_device *pdev)
{
	qsc_disable_irqs();

	mdm_drv->ops->power_down_mdm_cb(mdm_drv);
	if (GPIO_IS_VALID(mdm_drv->ap2mdm_pmic_pwr_en_gpio))
		gpio_direction_output(mdm_drv->ap2mdm_pmic_pwr_en_gpio, 0);
}

static void qsc_peripheral_connect(struct qsc_modem_drv *mdm_drv)
{
	if (!mdm_drv->pdata->peripheral_platform_device)
		return;

	mutex_lock(&hsic_status_lock);
	if (hsic_peripheral_status)
		goto out;
	platform_device_add(mdm_drv->pdata->peripheral_platform_device);
	hsic_peripheral_status = 1;
out:
	mutex_unlock(&hsic_status_lock);
}

static void qsc_peripheral_disconnect(struct qsc_modem_drv *mdm_drv)
{
	if (!mdm_drv->pdata->peripheral_platform_device)
		return;

	mutex_lock(&hsic_status_lock);
	if (!hsic_peripheral_status)
		goto out;
	platform_device_del(mdm_drv->pdata->peripheral_platform_device);
	hsic_peripheral_status = 0;
out:
	mutex_unlock(&hsic_status_lock);
}

/* This function can be called from atomic context. */
static void qsc_toggle_soft_reset(struct qsc_modem_drv *mdm_drv)
{
	int soft_reset_direction_assert = 0,
	    soft_reset_direction_de_assert = 1;

	if (mdm_drv->pdata->soft_reset_inverted) {
		soft_reset_direction_assert = 1;
		soft_reset_direction_de_assert = 0;
	}
	gpio_direction_output(mdm_drv->ap2mdm_soft_reset_gpio,
			soft_reset_direction_assert);
	/* Use mdelay because this function can be called from atomic
	 * context.
	 */
	mdelay(10);
	gpio_direction_output(mdm_drv->ap2mdm_soft_reset_gpio,
			soft_reset_direction_de_assert);
}

/* This function can be called from atomic context. */
static void qsc_atomic_soft_reset(struct qsc_modem_drv *mdm_drv)
{
	qsc_toggle_soft_reset(mdm_drv);
}

static void qsc_power_down_common(struct qsc_modem_drv *mdm_drv)
{
	int i;
	int soft_reset_direction =
		mdm_drv->pdata->soft_reset_inverted ? 1 : 0;

	qsc_peripheral_disconnect(mdm_drv);

	/* Wait for the modem to complete its power down actions. */
	for (i = 20; i > 0; i--) {
		if (gpio_get_value(mdm_drv->mdm2ap_status_gpio) == 0) {
			if (qsc_debug_mask & MDM_DEBUG_MASK_SHDN_LOG)
				pr_info("%s: qsc2ap_status went low, i = %d\n",
					__func__, i);
			break;
		}
		msleep(100);
	}
	if (i == 0) {
		pr_err("%s: QSC2AP_STATUS never went low. Doing a hard reset\n",
			   __func__);
		gpio_direction_output(mdm_drv->ap2mdm_soft_reset_gpio,
					soft_reset_direction);
		/*
		* Currently, there is a debounce timer on the charm PMIC. It is
		* necessary to hold the PMIC RESET low for ~3.5 seconds
		* for the reset to fully take place. Sleep here to ensure the
		* reset has occured before the function exits.
		*/
		msleep(4000);
	}
}

static void qsc_do_first_power_on(struct qsc_modem_drv *mdm_drv)
{
	int i;
	int pblrdy;
	if (power_on_count != 1) {
		pr_err("%s: Calling fn when power_on_count != 1\n",
			   __func__);
		return;
	}

	pr_err("%s: Powering on modem for the first time\n", __func__);
	qsc_peripheral_disconnect(mdm_drv);

	/* If this is the first power-up after a panic, the modem may still
	 * be in a power-on state, in which case we need to toggle the gpio
	 * instead of just de-asserting it. No harm done if the modem was
	 * powered down.
	 */
	qsc_toggle_soft_reset(mdm_drv);
	/* If the device has a kpd pwr gpio then toggle it. */
	if (GPIO_IS_VALID(mdm_drv->ap2mdm_kpdpwr_n_gpio)) {
		/* Pull AP2MDM_KPDPWR gpio high and wait for PS_HOLD to settle,
		 * then	pull it back low.
		 */
		pr_debug("%s: Pulling AP2QSC_KPDPWR gpio high\n", __func__);
		gpio_direction_output(mdm_drv->ap2mdm_kpdpwr_n_gpio, 1);
		msleep(1000);
		gpio_direction_output(mdm_drv->ap2mdm_kpdpwr_n_gpio, 0);
	}

	if (!GPIO_IS_VALID(mdm_drv->mdm2ap_pblrdy))
		goto start_qsc_peripheral;

	for (i = 0; i  < MDM_PBLRDY_CNT; i++) {
		pblrdy = gpio_get_value(mdm_drv->mdm2ap_pblrdy);
		if (pblrdy)
			break;
		usleep_range(5000, 5000);
	}
	pr_debug("%s: i:%d\n", __func__, i);

start_qsc_peripheral:
	qsc_peripheral_connect(mdm_drv);
	msleep(200);
}

static void qsc_do_soft_power_on(struct qsc_modem_drv *mdm_drv)
{
	int i;
	int pblrdy;

	pr_err("%s: soft resetting qsc modem\n", __func__);
	qsc_peripheral_disconnect(mdm_drv);
	qsc_toggle_soft_reset(mdm_drv);

	if (!GPIO_IS_VALID(mdm_drv->mdm2ap_pblrdy))
		goto start_qsc_peripheral;

	for (i = 0; i  < MDM_PBLRDY_CNT; i++) {
		pblrdy = gpio_get_value(mdm_drv->mdm2ap_pblrdy);
		if (pblrdy)
			break;
		usleep_range(5000, 5000);
	}

	pr_debug("%s: i:%d\n", __func__, i);

start_qsc_peripheral:
	qsc_peripheral_connect(mdm_drv);
	msleep(200);
}

static void qsc_power_on_common(struct qsc_modem_drv *mdm_drv)
{
	power_on_count++;

	/* this gpio will be used to indicate apq readiness,
	 * de-assert it now so that it can be asserted later.
	 * May not be used.
	 */
	if (GPIO_IS_VALID(mdm_drv->ap2mdm_wakeup_gpio))
		gpio_direction_output(mdm_drv->ap2mdm_wakeup_gpio, 0);

	/*
	 * If we did an "early power on" then ignore the very next
	 * power-on request because it would the be first request from
	 * user space but we're already powered on. Ignore it.
	 */
	if (mdm_drv->pdata->early_power_on &&
			(power_on_count == 2))
		return;

	if (power_on_count == 1)
		qsc_do_first_power_on(mdm_drv);
	else
		qsc_do_soft_power_on(mdm_drv);
}

static void debug_state_changed(int value)
{
	qsc_debug_mask = value;
}

static void qsc_status_changed(struct qsc_modem_drv *mdm_drv, int value)
{
	pr_debug("%s: value:%d\n", __func__, value);

	if (value) {
		qsc_peripheral_disconnect(mdm_drv);
		qsc_peripheral_connect(mdm_drv);
		if (GPIO_IS_VALID(mdm_drv->ap2mdm_wakeup_gpio))
			gpio_direction_output(mdm_drv->ap2mdm_wakeup_gpio, 1);
	}
}

static void qsc_image_upgrade(struct qsc_modem_drv *mdm_drv, int type)
{
	switch (type) {
	case APQ_CONTROLLED_UPGRADE:
		pr_debug("%s APQ controlled modem image upgrade\n", __func__);
		mdm_drv->mdm_ready = 0;
		qsc_toggle_soft_reset(mdm_drv);
		break;
	case MDM_CONTROLLED_UPGRADE:
		pr_debug("%s QSC controlled modem image upgrade\n", __func__);
		mdm_drv->mdm_ready = 0;
		/*
		 * If we have no image currently present on the modem, then we
		 * would be in PBL, in which case the status gpio would not go
		 * high.
		 */
		mdm_drv->disable_status_check = 1;
		if (GPIO_IS_VALID(mdm_drv->usb_switch_gpio)) {
			pr_info("%s Switching usb control to QSC\n", __func__);
			gpio_direction_output(mdm_drv->usb_switch_gpio, 1);
		} else
			pr_err("%s usb switch gpio unavailable\n", __func__);
		break;
	default:
		pr_err("%s invalid upgrade type\n", __func__);
	}
}
static struct mdm_ops mdm_cb = {
	.power_on_mdm_cb = qsc_power_on_common,
	.reset_mdm_cb = qsc_power_on_common,
	.atomic_reset_mdm_cb = qsc_atomic_soft_reset,
	.power_down_mdm_cb = qsc_power_down_common,
	.debug_state_changed_cb = debug_state_changed,
	.status_cb = qsc_status_changed,
	.image_upgrade_cb = qsc_image_upgrade,
};

static int __init qsc_modem_probe(struct platform_device *pdev)
{
	return qsc_common_create(pdev, &mdm_cb);
}

static int __devexit qsc_modem_remove(struct platform_device *pdev)
{
	return qsc_common_modem_remove(pdev);
}

static void qsc_modem_shutdown(struct platform_device *pdev)
{
	qsc_common_modem_shutdown(pdev);
}

static struct platform_driver qsc_modem_driver = {
	.remove         = qsc_modem_remove,
	.shutdown	= qsc_modem_shutdown,
	.driver         = {
		.name = "qsc_modem",
		.owner = THIS_MODULE
	},
};

static int __init qsc_modem_init(void)
{
	return platform_driver_probe(&qsc_modem_driver, qsc_modem_probe);
}

static void __exit qsc_modem_exit(void)
{
	platform_driver_unregister(&qsc_modem_driver);
}

module_init(qsc_modem_init);
module_exit(qsc_modem_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("qsc modem driver");
MODULE_VERSION("1.0");
MODULE_ALIAS("qsc_modem");
