/* drivers/power/ds2784_battery.c
 *
 * Copyright (C) 2009 HTC Corporation
 * Copyright (C) 2009 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/pm.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/spinlock.h>

#include <linux/android_alarm.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/wakelock.h>
#include <asm/gpio.h>

#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/mutex.h>

#include "../w1/w1.h"
#include "../w1/slaves/w1_ds2784.h"

extern int is_ac_power_supplied(void);

struct battery_status {
	int timestamp;

	int voltage_uV;		/* units of uV */
	int current_uA;		/* units of uA */
	int current_avg_uA;
	int charge_uAh;

	int temp_C;		/* units of 0.1 C */

	u8 percentage;		/* battery percentage */
	u8 charging_source;
	u8 status_reg;
	u8 battery_full;	/* battery full (don't charge) */
};

#define BATTERY_LOG_MAX 4096
#define BATTERY_LOG_MASK (BATTERY_LOG_MAX - 1)

static DEFINE_MUTEX(battery_log_lock);
static struct battery_status battery_log[BATTERY_LOG_MAX];
static unsigned battery_log_head;
static unsigned battery_log_tail;

void battery_log_status(struct battery_status *s)
{
	unsigned n;
	mutex_lock(&battery_log_lock);
	n = battery_log_head;
	memcpy(battery_log + n, s, sizeof(struct battery_status));
	n = (n + 1) & BATTERY_LOG_MASK;
	if (n == battery_log_tail)
		battery_log_tail = (battery_log_tail + 1) & BATTERY_LOG_MASK;
	battery_log_head = n;
	mutex_unlock(&battery_log_lock);
}

static const char *battery_source[3] = { "none", " usb", "  ac" };

static int battery_log_print(struct seq_file *sf, void *private)
{
	unsigned n;
	mutex_lock(&battery_log_lock);
	seq_printf(sf, "timestamp    mV     mA avg mA      uAh   dC   %%   src   reg full\n");
	for (n = battery_log_tail; n != battery_log_head; n = (n + 1) & BATTERY_LOG_MASK) {
		struct battery_status *s = battery_log + n;
		seq_printf(sf, "%9d %5d %6d %6d %8d %4d %3d  %s  0x%02x %d\n",
			   s->timestamp, s->voltage_uV / 1000,
			   s->current_uA / 1000, s->current_avg_uA / 1000,
			   s->charge_uAh, s->temp_C,
			   s->percentage,
			   battery_source[s->charging_source],
			   s->status_reg, s->battery_full);
	}
	mutex_unlock(&battery_log_lock);
	return 0;
}


struct ds2784_device_info {
	struct device *dev;

	/* DS2784 data, valid after calling ds2784_battery_read_status() */
	unsigned long update_time;	/* jiffies when data read */
	char raw[DS2784_DATA_SIZE];	/* raw DS2784 data */

	struct battery_status status;

	struct power_supply bat;
	struct device *w1_dev;
	struct workqueue_struct *monitor_wqueue;
	struct work_struct monitor_work;
	struct alarm alarm;
	struct wake_lock work_wake_lock;

	int dummy;
};

#define psy_to_dev_info(x) container_of((x), struct ds2784_device_info, bat)

static struct wake_lock vbus_wake_lock;

#define BATT_NO_SOURCE        (0)  /* 0: No source battery */
#define BATT_FIRST_SOURCE     (1)  /* 1: Main source battery */
#define BATT_SECOND_SOURCE    (2)  /* 2: Second source battery */
#define BATT_THIRD_SOURCE     (3)  /* 3: Third source battery */
#define BATT_FOURTH_SOURCE    (4)  /* 4: Fourth source battery */
#define BATT_FIFTH_SOURCE     (5)  /* 5: Fifth source battery */
#define BATT_UNKNOWN        (255)  /* Other: Unknown battery */

#define BATT_RSNSP			(67)	/*Passion battery source 1*/

#define GPIO_BATTERY_DETECTION		39
#define GPIO_BATTERY_CHARGER_EN		22
#define GPIO_BATTERY_CHARGER_CURRENT	16

static enum power_supply_property battery_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
};

static int battery_initial;

typedef enum {
	DISABLE = 0,
	ENABLE_SLOW_CHG,
	ENABLE_FAST_CHG
} batt_ctl_t;

typedef enum {
	CHARGER_BATTERY = 0,
	CHARGER_USB,
	CHARGER_AC
} charger_type_t;

static int battery_get_property(struct power_supply *psy,
				    enum power_supply_property psp,
				    union power_supply_propval *val);

static void battery_ext_power_changed(struct power_supply *psy);

#define to_ds2784_device_info(x) container_of((x), struct ds2784_device_info, \
					      bat);

static void ds2784_parse_data(u8 *raw, struct battery_status *s)
{
	short n;

	/* Get status reg */
	s->status_reg = raw[DS2784_REG_STS];

	/* Get Level */
	s->percentage = raw[DS2784_REG_RARC];

	/* Get Voltage: Unit=4.886mV, range is 0V to 4.99V */
	n = (((raw[DS2784_REG_VOLT_MSB] << 8) |
	      (raw[DS2784_REG_VOLT_LSB])) >> 5);

	s->voltage_uV = n * 4886;

	/* Get Current: Unit= 1.5625uV x Rsnsp(67)=104.68 */
	n = ((raw[DS2784_REG_CURR_MSB]) << 8) |
		raw[DS2784_REG_CURR_LSB];
	s->current_uA = ((n * 15625) / 10000) * 67;

	n = ((raw[DS2784_REG_AVG_CURR_MSB]) << 8) |
		raw[DS2784_REG_AVG_CURR_LSB];
	s->current_avg_uA = ((n * 15625) / 10000) * 67;

	/* Get Temperature:
	 * Unit=0.125 degree C,therefore, give up LSB ,
	 * just caculate MSB for temperature only.
	 */
	n = (((signed char)raw[DS2784_REG_TEMP_MSB]) << 3) |
		(raw[DS2784_REG_TEMP_LSB] >> 5);

	s->temp_C = n + (n / 4);

	/* RAAC is in units of 1.6mAh */
	s->charge_uAh = ((raw[DS2784_REG_RAAC_MSB] << 8) |
			  raw[DS2784_REG_RAAC_LSB]) * 1600;
}

static int ds2784_battery_read_status(struct ds2784_device_info *di)
{
	int ret, start, count;

	/* The first time we read the entire contents of SRAM/EEPROM,
	 * but after that we just read the interesting bits that change. */
	if (di->raw[DS2784_REG_RSNSP] == 0x00) {
		start = 0;
		count = DS2784_DATA_SIZE;
	} else {
		start = DS2784_REG_PORT;
		count = DS2784_REG_CURR_LSB - start + 1;
	}

	ret = w1_ds2784_read(di->w1_dev, di->raw + start, start, count);
	if (ret != count) {
		dev_warn(di->dev, "call to w1_ds2784_read failed (0x%p)\n",
			 di->w1_dev);
		return 1;
	}
	di->update_time = jiffies;

	if (battery_initial == 0) {
		if (!memcmp(di->raw + 0x20, "DUMMY!", 6)) {
			unsigned char acr[2];

			di->dummy = 1;
			pr_info("batt: dummy battery detected\n");

			/* reset ACC register to ~500mAh, since it may have zeroed out */
			acr[0] = 0x05;
			acr[1] = 0x06;
			w1_ds2784_write(di->w1_dev, acr,DS2784_REG_ACCUMULATE_CURR_MSB, 2);
		}
		battery_initial = 1;
	}

	pr_info("batt: %02x %02x %02x%02x %02x%02x %02x%02x %02x%02x %02x%02x %02x%02x %02x%02x\n",
		di->raw[0x00], di->raw[0x01], di->raw[0x02], di->raw[0x03], 
		di->raw[0x04], di->raw[0x05], di->raw[0x06], di->raw[0x07], 
		di->raw[0x08], di->raw[0x09], di->raw[0x0a], di->raw[0x0b], 
		di->raw[0x0c], di->raw[0x0d], di->raw[0x0e], di->raw[0x0f]
		);

	ds2784_parse_data(di->raw, &di->status);

	pr_info("batt: %3d%%, %d mV, %d mA (%d avg), %d C, %d mAh\n",
		di->status.percentage,
		di->status.voltage_uV / 1000, di->status.current_uA / 1000,
		di->status.current_avg_uA / 1000,
		di->status.temp_C, di->status.charge_uAh / 1000);
	
	return 0;
}

static int battery_get_property(struct power_supply *psy,
				    enum power_supply_property psp,
				    union power_supply_propval *val)
{
	struct ds2784_device_info *di = psy_to_dev_info(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		switch (di->status.charging_source) {
		case CHARGER_BATTERY:
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
			break;
		case CHARGER_USB:
		case CHARGER_AC:
			if (di->status.battery_full)
				val->intval = POWER_SUPPLY_STATUS_FULL;
			else
				val->intval = POWER_SUPPLY_STATUS_CHARGING;
			break;
		default:
			val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
			break;
		}
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = POWER_SUPPLY_HEALTH_GOOD;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		/* XXX todo */
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		if (di->dummy) {
			val->intval = POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
		} else {
			val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		}
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		if (di->dummy) {
			val->intval = 75;
		} else {
			val->intval = di->status.percentage;
		}
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = di->status.voltage_uV;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = di->status.temp_C;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = di->status.current_uA;
		break;
	case POWER_SUPPLY_PROP_CURRENT_AVG:
		val->intval = di->status.current_avg_uA;
		break;
	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
		val->intval = di->status.charge_uAh;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static void ds2784_battery_update_status(struct ds2784_device_info *di)
{
	u8 last_level;
	last_level = di->status.percentage;

	ds2784_battery_read_status(di);

#if 0
	if (htc_batt_info.rep.batt_id == BATT_UNKNOWN) {
		htc_batt_info.rep.level = 0;
		printk("Not support battery %d, power down system\n",htc_batt_info.rep.batt_id);
		power_supply_changed(&di->bat);
	}
#endif

	if (last_level != di->status.percentage)
		power_supply_changed(&di->bat);
}

static unsigned last_source = 0xffffffff;
static unsigned last_status = 0xffffffff;
static spinlock_t charge_state_lock;

static int battery_adjust_charge_state(struct ds2784_device_info *di)
{
	unsigned source;
	unsigned status;
	unsigned long flags;
	int charging_full;
	int rc = 0;

	spin_lock_irqsave(&charge_state_lock, flags);

	source = di->status.charging_source;
	status = di->status.status_reg;

	if ((status & 0x80) && (source > 0) &&
		(di->status.current_uA <= 80000) &&
		(di->status.percentage == 100))
		charging_full = 1;
	else
		charging_full = 0;

	if ((source == last_source) && (charging_full == di->status.battery_full))
		goto done;

	last_source = source;
	last_status = status;
	di->status.battery_full = charging_full;

	if (charging_full)
		/* ignore actual source if we're in full-dont-charge mode */
		source = CHARGER_BATTERY;
	else
		source = di->status.charging_source;

	switch (source) {
	case CHARGER_BATTERY:
		/* CHARGER_EN is active low.  Set to 1 to disable. */
		gpio_direction_output(GPIO_BATTERY_CHARGER_EN, 1);
		pr_info("batt: charging OFF%s\n",
			charging_full ? " [FULL]" : "");
		break;
	case CHARGER_USB:
		/* slow charge mode */
		gpio_direction_output(GPIO_BATTERY_CHARGER_CURRENT, 0);
		gpio_direction_output(GPIO_BATTERY_CHARGER_EN, 0);
		pr_info("batt: charging SLOW\n");
		break;
	case CHARGER_AC:
		/* fast charge mode */
		gpio_direction_output(GPIO_BATTERY_CHARGER_CURRENT, 1);
		gpio_direction_output(GPIO_BATTERY_CHARGER_EN, 0);
		pr_info("batt: charging FAST\n");
		break;
	}
	rc = 1;
done:
	spin_unlock_irqrestore(&charge_state_lock, flags);
	return rc;
}

static void ds2784_battery_work(struct work_struct *work)
{
	struct ds2784_device_info *di =
		container_of(work, struct ds2784_device_info, monitor_work);
	const ktime_t low_interval = ktime_set(50, 0);
	const ktime_t slack = ktime_set(20, 0);
	ktime_t now;
	ktime_t next_alarm;
	struct timespec ts;
	unsigned long flags;

	ds2784_battery_update_status(di);
	now = alarm_get_elapsed_realtime();

	if (battery_adjust_charge_state(di))
		power_supply_changed(&di->bat);

	next_alarm = ktime_add(now, low_interval);

	ts = ktime_to_timespec(now);
	di->status.timestamp = ts.tv_sec;
	battery_log_status(&di->status);

	/* prevent suspend before starting the alarm */
	local_irq_save(flags);

	wake_unlock(&di->work_wake_lock);
	alarm_start_range(&di->alarm, next_alarm, ktime_add(next_alarm, slack));
	local_irq_restore(flags);
}

static void ds2784_battery_alarm(struct alarm *alarm)
{
	struct ds2784_device_info *di =
		container_of(alarm, struct ds2784_device_info, alarm);
	wake_lock(&di->work_wake_lock);
	queue_work(di->monitor_wqueue, &di->monitor_work);
}

static void battery_ext_power_changed(struct power_supply *psy)
{
	struct ds2784_device_info *di;
	int got_power;

	di = psy_to_dev_info(psy);
	got_power = power_supply_am_i_supplied(psy);

	pr_info("*** batt ext power changed (%d) ***\n", got_power);

	if (got_power) {
		if (is_ac_power_supplied())
			di->status.charging_source = CHARGER_AC;
		else
			di->status.charging_source = CHARGER_USB;
		wake_lock(&vbus_wake_lock);
	} else {
		di->status.charging_source = CHARGER_BATTERY;
		/* give userspace some time to see the uevent and update
		 * LED state or whatnot...
		 */
		wake_lock_timeout(&vbus_wake_lock, HZ / 2);
	}
	battery_adjust_charge_state(di);
	power_supply_changed(psy);
}

static int ds2784_battery_probe(struct platform_device *pdev)
{
	int rc;
	struct ds2784_device_info *di;
	struct ds2784_platform_data *pdata;

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di)
		return -ENOMEM;

	di->update_time = jiffies;
	platform_set_drvdata(pdev, di);

	pdata = pdev->dev.platform_data;
	di->dev = &pdev->dev;
	di->w1_dev = pdev->dev.parent;

	di->bat.name = "battery";
	di->bat.type = POWER_SUPPLY_TYPE_BATTERY;
	di->bat.properties = battery_properties;
	di->bat.num_properties = ARRAY_SIZE(battery_properties);
	di->bat.external_power_changed = battery_ext_power_changed;
	di->bat.get_property = battery_get_property;

	rc = power_supply_register(&pdev->dev, &di->bat);
	if (rc)
		goto fail_register;

	INIT_WORK(&di->monitor_work, ds2784_battery_work);
	di->monitor_wqueue = create_singlethread_workqueue(dev_name(&pdev->dev));
	if (!di->monitor_wqueue) {
		rc = -ESRCH;
		goto fail_workqueue;
	}
	wake_lock_init(&di->work_wake_lock, WAKE_LOCK_SUSPEND,
			"ds2784-battery");
	alarm_init(&di->alarm, ANDROID_ALARM_ELAPSED_REALTIME_WAKEUP,
			ds2784_battery_alarm);
	wake_lock(&di->work_wake_lock);
	queue_work(di->monitor_wqueue, &di->monitor_work);
	return 0;

fail_workqueue:
	power_supply_unregister(&di->bat);
fail_register:
	kfree(di);
	return rc;
}

static struct platform_driver ds2784_battery_driver = {
	.driver = {
		.name = "ds2784-battery",
	},
	.probe	  = ds2784_battery_probe,
};

static int battery_log_open(struct inode *inode, struct file *file)
{
	return single_open(file, battery_log_print, NULL);
}

static struct file_operations battery_log_fops = {
	.open = battery_log_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int __init ds2784_battery_init(void)
{
	debugfs_create_file("battery_log", 0444, NULL, NULL, &battery_log_fops);
	spin_lock_init(&charge_state_lock);
	wake_lock_init(&vbus_wake_lock, WAKE_LOCK_SUSPEND, "vbus_present");
	return platform_driver_register(&ds2784_battery_driver);
}

module_init(ds2784_battery_init);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Justin Lin <Justin_lin@htc.com>");
MODULE_DESCRIPTION("ds2784 battery driver");
