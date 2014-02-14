/*
 * TI LP855x Backlight Driver
 *
 *			Copyright (C) 2011 Texas Instruments
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/backlight.h>
#include <linux/err.h>
#include <linux/lp855x.h>
#include <linux/delay.h>

#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>

/* LP8550/1/2/3/6 Registers */
#define LP855X_BRIGHTNESS_CTRL		0x00
#define LP855X_DEVICE_CTRL		0x01
#define LP855X_EEPROM_START		0xA0
#define LP855X_EEPROM_END		0xA7
#define LP8556_EPROM_START		0xA0
#define LP8556_EPROM_END		0xAF

/* LP8557 Registers */
#define LP8557_BL_CMD			0x00
#define LP8557_BL_MASK			0x01
#define LP8557_BL_ON			0x01
#define LP8557_BL_OFF			0x00
#define LP8557_BRIGHTNESS_CTRL		0x04
#define LP8557_CONFIG			0x10
#define LP8557_EPROM_START		0x10
#define LP8557_EPROM_END		0x1E

#define BUF_SIZE		20
#define DEFAULT_BL_NAME		"lcd-backlight"
#define MAX_BRIGHTNESS		255

//If these callbacks are defined, call the defined suspend/resume
// handlers such that the touch turns off before the display blanks
// and the touch turns on after the display unblanks
static int (*touch_suspend) (void* dev_context) = NULL;
static int (*touch_resume) (void* dev_context) = NULL;
static void* touch_dev_context = NULL;

struct lp855x;

static int backlight_en;

struct lp855x_device_config {
	u8 reg_brightness;
	u8 reg_devicectrl;
};

struct lp855x {
	const char *chipname;
	enum lp855x_chip_id chip_id;
	struct lp855x_device_config *cfg;
	struct i2c_client *client;
	struct backlight_device *bl;
	struct device *dev;
	struct lp855x_platform_data *pdata;
	struct pwm_device *pwm;
	struct mutex bl_lock;
	int last_brightness;
};

struct lp855x *lp855x_ctx = NULL; /* a global pointer for other module's use */

static int lp855x_write_byte(struct lp855x *lp, u8 reg, u8 data)
{
	return i2c_smbus_write_byte_data(lp->client, reg, data);
}

static int lp855x_update_bit(struct lp855x *lp, u8 reg, u8 mask, u8 data)
{
	int ret;
	u8 tmp;

	ret = i2c_smbus_read_byte_data(lp->client, reg);
	if (ret < 0) {
		dev_err(lp->dev, "failed to read 0x%.2x\n", reg);
		return ret;
	}

	tmp = (u8)ret;
	tmp &= ~mask;
	tmp |= data & mask;

	return lp855x_write_byte(lp, reg, tmp);
}

static int lp855x_read_byte(struct lp855x *lp, u8 reg, u8 *data)
{
	int ret;

	ret = i2c_smbus_read_byte_data(lp->client, reg);
	if (ret < 0) {
		dev_err(lp->dev, "failed to read 0x%.2x\n", reg);
		return ret;
	}

	*data = (u8)ret;
	return 0;
}

static bool lp855x_is_valid_rom_area(struct lp855x *lp, u8 addr)
{
	u8 start, end;

	switch (lp->chip_id) {
	case LP8550:
	case LP8551:
	case LP8552:
	case LP8553:
		start = LP855X_EEPROM_START;
		end = LP855X_EEPROM_END;
		break;
	case LP8556:
		start = LP8556_EPROM_START;
		end = LP8556_EPROM_END;
		break;
	case LP8557:
		start = LP8557_EPROM_START;
		end = LP8557_EPROM_END;
		break;
	default:
		return false;
	}

	return (addr >= start && addr <= end);
}

static int lp855x_init_device(struct lp855x *lp)
{
	struct lp855x_platform_data *pd = lp->pdata;
	int i;
	int ret;
	u8 val;
	u8 addr;

	val = pd->device_control;
	ret = lp855x_write_byte(lp, lp->cfg->reg_devicectrl, val);
	if (ret)
		goto err;

	if (pd->load_new_rom_data && pd->size_program) {
		for (i = 0; i < pd->size_program; i++) {
			addr = pd->rom_data[i].addr;
			val = pd->rom_data[i].val;
			if (!lp855x_is_valid_rom_area(lp, addr))
				continue;

			ret = lp855x_write_byte(lp, addr, val);
			if (ret)
				goto err;
		}
	}

err:
	return ret;
}

static int lp8557_bl_off(struct lp855x *lp)
{
	dev_info(lp->dev, "bl_OFF\n");

	/* BL_ON = 0 before updating EPROM settings */
	lp855x_update_bit(lp, LP8557_BL_CMD, LP8557_BL_MASK, LP8557_BL_OFF);

	gpio_set_value(lp->pdata->gpio_enable, 0);

	return 0;
}

DECLARE_WAIT_QUEUE_HEAD(panel_on_waitqueue);
int lcd_panel_enabled = 1; /* first power on do not need to wait */

static int lp8557_bl_on(struct lp855x *lp)
{
	long timeWaited=0;
	struct timeval  tv1, tv2;

	dev_info(lp->dev, "bl_ON\n");
 
	do_gettimeofday(&tv1);
	if (!lcd_panel_enabled) {		
		/* wait until LCD panel is ready or end of timeout */
		timeWaited = wait_event_timeout(panel_on_waitqueue, lcd_panel_enabled, msecs_to_jiffies(1000));
	}
	do_gettimeofday(&tv2);
	dev_info(lp->dev, "time usec %ld -> %ld, diff %ld, timeWaited=%ld\n", 
		 tv1.tv_usec, tv2.tv_usec, tv2.tv_usec - tv1.tv_usec, timeWaited);

	lcd_panel_enabled = 0;

	/* turn on lp855x power and init */
	gpio_set_value(lp->pdata->gpio_enable, 1);
	msleep(20);
	lp855x_init_device(lp);

	/* BL_ON = 1 after updating EPROM settings */
	lp855x_update_bit(lp, LP8557_BL_CMD, LP8557_BL_MASK, LP8557_BL_ON);

	return 0;
}

static struct lp855x_device_config lp8550to6_cfg = {
	.reg_brightness = LP855X_BRIGHTNESS_CTRL,
	.reg_devicectrl = LP855X_DEVICE_CTRL,
};

static struct lp855x_device_config lp8557_cfg = {
	.reg_brightness = LP8557_BRIGHTNESS_CTRL,
	.reg_devicectrl = LP8557_CONFIG,
};

static int lp855x_configure(struct lp855x *lp)
{
	u8 ret = 0;

	switch (lp->chip_id) {
	case LP8550:
	case LP8551:
	case LP8552:
	case LP8553:
	case LP8556:
		lp->cfg = &lp8550to6_cfg;
		break;
	case LP8557:
		lp->cfg = &lp8557_cfg;
		break;
	default:
		return -EINVAL;
	}

	if (lp->pdata->cont_splash_enabled == 0) {
		/* BL_ON = 0 before updating EPROM settings */
		lp855x_update_bit(lp, LP8557_BL_CMD, LP8557_BL_MASK, LP8557_BL_OFF);

		lp855x_init_device(lp);

		/* BL_ON = 1 after updating EPROM settings */
		lp855x_update_bit(lp, LP8557_BL_CMD, LP8557_BL_MASK, LP8557_BL_ON);
	}
	else {
		lp->last_brightness = 0;
		ret = lp855x_read_byte(lp, lp->cfg->reg_brightness, (u8 *)&lp->last_brightness);
		if (ret) {
			dev_err(lp->dev, "%s read from chip failed\n", __func__);
			lp->last_brightness = 0;
		}
	}

	return ret;
}

//These callbacks will be used for in-cell touch systems where the touch
// driver needs to be disabled before cutting the display's power, and
// enabled after the display is up and running
int lp855x_bl_add_touch_pm_calls(
	int (*new_touch_suspend) (void* dev_context),
	int (*new_touch_resume) (void* dev_context),
	void* new_dev_context
)
{
	//Check to see if there are already touch callbacks defined
	if (touch_suspend)
	{
		pr_err("%s(): Touch power callbacks already defined\n",
		       __func__);
		return -ENOMEM;
	}

	touch_suspend = new_touch_suspend;
	touch_resume = new_touch_resume;
	touch_dev_context = new_dev_context;

	return 0;
}
EXPORT_SYMBOL(lp855x_bl_add_touch_pm_calls);

//TODO: Grab console mutex, or otherwise synchronize with the rest of the
// driver to prevent race conditons from loading/unloading touch while
// blanking/unblanking display (shouldn't be possible due to the way our
// full system initializes, but it couldn't hurt to be careful)
int lp855x_bl_rm_touch_pm_calls(void)
{
	touch_suspend = NULL;
	touch_resume = NULL;
	touch_dev_context = NULL;

	return 0;
}
EXPORT_SYMBOL(lp855x_bl_rm_touch_pm_calls);

static void lp855x_bl_update_register(struct lp855x *lp, int val)
{
	if (lp == NULL) return;

	if ((lp->last_brightness == 0) && (val != 0)) {
		lp8557_bl_on(lp);
		if (touch_resume)
			touch_resume(touch_dev_context);
	}
	if ((lp->last_brightness != 0) && (val == 0)) {
		if (touch_suspend)
			touch_suspend(touch_dev_context);
		lp8557_bl_off(lp);
	}

	if (val != lp->last_brightness) {
		lp855x_write_byte(lp, lp->cfg->reg_brightness, val);
		lp->last_brightness = val;
		dev_dbg(lp->dev, "%s set brightness to %d\n", __func__, val);
	}
}

void lp855x_bl_set(int val)
{
	if (lp855x_ctx) {
		mutex_lock(&lp855x_ctx->bl_lock); /* lock it in this function instead of update_register */
		lp855x_bl_update_register(lp855x_ctx, val);
		mutex_unlock(&lp855x_ctx->bl_lock);
	}
}
EXPORT_SYMBOL(lp855x_bl_set);

static int lp855x_bl_update_status(struct backlight_device *bl)
{
	struct lp855x *lp = bl_get_data(bl);

	/* Move backlight setting to /sys/class/leds/lcd-backlight/brightness
	   This is because ALS is handled there (scaled backlight) */
	dev_err(lp->dev, "%s: backlight should be set through leds interface\n", __func__);
#if 0
	enum lp855x_brightness_ctrl_mode mode = lp->pdata->mode;

	if (mode == PWM_BASED) {
		struct lp855x_pwm_data *pd = &lp->pdata->pwm_data;
		int br = bl->props.brightness;
		int max_br = bl->props.max_brightness;

		if (pd->pwm_set_intensity)
			pd->pwm_set_intensity(br, max_br);

	} else if (mode == REGISTER_BASED) {
		u8 val = bl->props.brightness;
		lp855x_bl_update_register(lp, val);
	}
#endif
	return 0; /* the return value is not checked in calling function */
}

static int lp855x_bl_get_brightness(struct backlight_device *bl)
{
	struct lp855x *lp = bl_get_data(bl);
	u8 val=0, ret;

	ret = lp855x_read_byte(lp, lp->cfg->reg_brightness, &val);
	if (ret) {
		dev_err(lp->dev, "%s read from chip failed, use last set value\n", __func__);
		val = bl->props.brightness;
	}
	return val;
}

static const struct backlight_ops lp855x_bl_ops = {
	.options = BL_CORE_SUSPENDRESUME,
	.update_status = lp855x_bl_update_status,
	.get_brightness = lp855x_bl_get_brightness,
};

static int lp855x_backlight_register(struct lp855x *lp)
{
	struct backlight_device *bl;
	struct backlight_properties props;
	struct lp855x_platform_data *pdata = lp->pdata;
	char *name = pdata->name ? : DEFAULT_BL_NAME;

	props.type = BACKLIGHT_PLATFORM;
	props.max_brightness = MAX_BRIGHTNESS;

	if (pdata->initial_brightness > props.max_brightness)
		pdata->initial_brightness = props.max_brightness;

	props.brightness = pdata->initial_brightness;

	bl = backlight_device_register(name, lp->dev, lp,
				       &lp855x_bl_ops, &props);
	if (IS_ERR(bl))
		return PTR_ERR(bl);

	lp->bl = bl;

	return 0;
}

static void lp855x_backlight_unregister(struct lp855x *lp)
{
	if (lp->bl)
		backlight_device_unregister(lp->bl);
}

static ssize_t lp855x_get_chip_id(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct lp855x *lp = dev_get_drvdata(dev);
	return scnprintf(buf, BUF_SIZE, "%s\n", lp->chipname);
}

static ssize_t lp855x_get_bl_ctl_mode(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct lp855x *lp = dev_get_drvdata(dev);
	enum lp855x_brightness_ctrl_mode mode = lp->pdata->mode;
	char *strmode = NULL;

	if (mode == PWM_BASED)
		strmode = "pwm based";
	else if (mode == REGISTER_BASED)
		strmode = "register based";

	return scnprintf(buf, BUF_SIZE, "%s\n", strmode);
}

static ssize_t lp855x_dump_registers(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct lp855x *lp = dev_get_drvdata(dev);
	int ret;
	u8 addr;
	u8 val = 0;

	for (addr = 0x00 ; addr < 0x20 ; addr++) {
		ret = lp855x_read_byte(lp, addr, &val);
		if (ret)
			break;
		dev_info(lp->dev, "R: [0x%.2x] = 0x%.2x\n", addr, val);
	}

	return 0;
}

static DEVICE_ATTR(chip_id, S_IRUGO, lp855x_get_chip_id, NULL);
static DEVICE_ATTR(bl_ctl_mode, S_IRUGO, lp855x_get_bl_ctl_mode, NULL);
static DEVICE_ATTR(registers, S_IRUGO, lp855x_dump_registers, NULL);

static struct attribute *lp855x_attributes[] = {
	&dev_attr_chip_id.attr,
	&dev_attr_bl_ctl_mode.attr,
	&dev_attr_registers.attr,
	NULL,
};

static const struct attribute_group lp855x_attr_group = {
	.attrs = lp855x_attributes,
};

#ifdef CONFIG_OF
static int lp855x_parse_dt(struct device *dev, struct lp855x_platform_data *pdata)
{
	struct device_node *np = dev->of_node;
	struct lp855x_rom_data *rom_data;
	u32  *ptmp;
	u32 tmp;
	int rc, i, size_rom_data;

	pdata->gpio_enable = of_get_named_gpio(np, "ti,lp855x-en-gpio", 0);
	if (!gpio_is_valid(pdata->gpio_enable)) {
		dev_err(dev, "%s:%d, backlight gpio not specified\n",
						__func__, __LINE__);
		return -ENODEV;
	}

	rc = gpio_request(pdata->gpio_enable, "backlight_en");
	if (rc) {
		dev_err(dev, "request reset gpio failed, rc=%d\n",
			rc);
		gpio_free(backlight_en);
		return -ENODEV;
	}
	rc = gpio_direction_output(pdata->gpio_enable, 1);
	if (rc) {
		dev_err(dev, "set_direction for disp_en gpio failed, rc=%d\n",
			rc);
		gpio_free(backlight_en);
		return -ENODEV;
	}
	
	rc = of_property_read_u32(np, "ti,lp855x-device-control", &tmp);
	if (rc) {
		dev_err(dev, "%s:%d, device control not specified\n",
						__func__, __LINE__);
		return -EINVAL;
	}	
	pdata->device_control = tmp;

	rc = of_property_read_u32(np, "ti,lp855x-mode", &tmp);
	if (rc) {
		dev_err(dev, "%s:%d, mode not specified\n",
						__func__, __LINE__);
		return -EINVAL;
	}	
	pdata->mode = tmp;
	
	tmp = 0;
	rc = of_property_read_u32(np, "ti,lp855x-cont-splash-enabled", &tmp);
	pdata->cont_splash_enabled = tmp;

	rc = of_property_read_u32(np, "ti,lp855x-initial-brightness", &tmp);
	if (rc) {
		dev_err(dev, "%s:%d, initial brightness not specified\n",
						__func__, __LINE__);
		return -EINVAL;
	}	
	pdata->initial_brightness = tmp;
	
	rc = of_property_read_u32(np, "ti,lp855x-load-new-rom-data", &tmp);
	if (rc) {
		dev_err(dev, "%s:%d, initial brightness not specified\n",
						__func__, __LINE__);
		return -EINVAL;
	}	
	pdata->load_new_rom_data = tmp;		
	
	rc = of_property_read_u32(np, "ti,lp855x-size-program", &tmp);
	if (rc) {
		dev_err(dev, "%s:%d, initial brightness not specified\n",
						__func__, __LINE__);
		return -EINVAL;
	}	
	pdata->size_program = tmp;
	size_rom_data = sizeof(struct lp855x_rom_data);
	rom_data = devm_kzalloc(dev, pdata->size_program*size_rom_data, GFP_KERNEL);
	if (!rom_data) {
		dev_err(dev, "%s: Failed to allocate memory for pdata\n", __func__);
		return -ENOMEM;
	}
	ptmp = devm_kzalloc(dev, pdata->size_program*size_rom_data, GFP_KERNEL);
	if (!ptmp) {
		dev_err(dev, "%s: Failed to allocate memory for pdata\n", __func__);
		goto out;
	}
	
	rc = of_property_read_u32_array(np,
		"ti,lp855x-rom-data", ptmp, pdata->size_program*size_rom_data);

	for (i=0; i<pdata->size_program; i++){
		rom_data[i].addr = (u8)(*(ptmp+i*size_rom_data));
		rom_data[i].val  = (u8)(*(ptmp+i*size_rom_data+1));
	}
	pdata->rom_data = rom_data;
	
	devm_kfree(dev, ptmp);
	
	dev_info(dev, "%s: backlight_en=%d device-control=%d mode=%d \
		initial-brightness=%d load-new-rom-data=%d size_program=%d rom-data-addr=%d rom-data-val=%d", \
		__func__, backlight_en, pdata->device_control, pdata->mode,  \
		pdata->initial_brightness, pdata->load_new_rom_data, pdata->size_program, \
		pdata->rom_data->addr, pdata->rom_data->val);

	return 0;
	
out:
	devm_kfree(dev, rom_data);
	return -ENOMEM;
}
#else
static int lp855x_parse_dt(struct device *dev, struct lp855x_platform_data *pdata)
{
	return -ENODEV;
}
#endif

static int lp855x_probe(struct i2c_client *cl, const struct i2c_device_id *id)
{
	struct lp855x *lp;
	struct lp855x_platform_data *pdata;
	enum lp855x_brightness_ctrl_mode mode;
	int ret;
	struct regulator *vcc_i2c;

	if (!i2c_check_functionality(cl->adapter, I2C_FUNC_SMBUS_I2C_BLOCK))
		return -EIO;

	pdata = devm_kzalloc(&cl->dev, sizeof(struct lp855x_platform_data), GFP_KERNEL);
	if (!pdata) {
		dev_err(&cl->dev, "%s: Failed to allocate memory for pdata\n", __func__);
		return -ENOMEM;
	}
	if (cl->dev.of_node) {
		ret = lp855x_parse_dt(&cl->dev, pdata);
		if (ret)
			return ret;
	}

	lp = devm_kzalloc(&cl->dev, sizeof(struct lp855x), GFP_KERNEL);
	if (!lp)
		return -ENOMEM;

	mode = pdata->mode;
	lp->client = cl;
	lp->dev = &cl->dev;
	lp->pdata = pdata;
	lp->chipname = "lp8557";
	lp->chip_id = LP8557;
	lp->last_brightness = 0;
	mutex_init(&lp->bl_lock);
	i2c_set_clientdata(cl, lp);
	
	// i2c_pull_up
	vcc_i2c = regulator_get(&cl->dev, "vcc_i2c");
	if (IS_ERR(vcc_i2c)) {
		ret = PTR_ERR(vcc_i2c);
		dev_err(&cl->dev, "Regulator get failed rc=%d\n", ret);
		dev_err(&cl->dev, "lp855x vcc_i2c NOT ok \n");
		goto err_dev;
	}
	ret = regulator_enable(vcc_i2c);
	if (ret) {
		dev_err(lp->dev, "regulator enable err: %d\n", ret);
		goto err_dev;
	}
	msleep(3);	

	ret = lp855x_configure(lp);
	if (ret) {
		dev_err(lp->dev, "device config err: %d\n", ret);
		goto err_dev;
	}

	ret = lp855x_backlight_register(lp);
	if (ret) {
		dev_err(lp->dev,
			"failed to register backlight. err: %d\n", ret);
		goto err_dev;
	}

	ret = sysfs_create_group(&lp->dev->kobj, &lp855x_attr_group);
	if (ret) {
		dev_err(lp->dev, "failed to register sysfs. err: %d\n", ret);
		goto err_sysfs;
	}

	lp855x_ctx = lp;
/* This is a hack to update initial brightness, will remove after Android setting is right */
#if defined(CONFIG_ARCH_APQ8084_LOKI)
	lp855x_bl_update_register(lp, pdata->initial_brightness);
#endif	
	return 0;

err_sysfs:
	lp855x_backlight_unregister(lp);
err_dev:
	return ret;
}

static int lp855x_remove(struct i2c_client *cl)
{
	struct lp855x *lp = i2c_get_clientdata(cl);

	lp->bl->props.brightness = 0;
	backlight_update_status(lp->bl);
	sysfs_remove_group(&lp->dev->kobj, &lp855x_attr_group);
	lp855x_backlight_unregister(lp);
	lp855x_ctx = NULL;

	return 0;
}

static const struct i2c_device_id lp855x_ids[] = {
        {"lp8550", LP8550},
        {"lp8551", LP8551},
        {"lp8552", LP8552},
        {"lp8553", LP8553},
        {"lp8556", LP8556},
        {"lp8557", LP8557},
	{ }
};
MODULE_DEVICE_TABLE(i2c, lp855x_ids);

#ifdef CONFIG_OF
static struct of_device_id lp855x_match_table[] = {
	{ .compatible = "ti,lp855x",},
	{ },
};
#else
#define lp855x_match_table NULL
#endif

static struct i2c_driver lp855x_driver = {
	.driver = {
		.name = "lp855x",
		.owner	= THIS_MODULE,
		.of_match_table = lp855x_match_table,
	},
	.probe = lp855x_probe,
	.remove = lp855x_remove,
	.id_table = lp855x_ids,
};

module_i2c_driver(lp855x_driver);

MODULE_DESCRIPTION("Texas Instruments LP855x Backlight driver");
MODULE_AUTHOR("Milo Kim <milo.kim@ti.com>");
MODULE_LICENSE("GPL");
