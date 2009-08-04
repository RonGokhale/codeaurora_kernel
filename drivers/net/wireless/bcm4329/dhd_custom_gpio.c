/*
* Customer code to add GPIO control during WLAN start/stop
* Copyright (C) 1999-2009, Broadcom Corporation
* 
*      Unless you and Broadcom execute a separate written software license
* agreement governing use of this software, this software is licensed to you
* under the terms of the GNU General Public License version 2 (the "GPL"),
* available at http://www.broadcom.com/licenses/GPLv2.php, with the
* following added to such license:
* 
*      As a special exception, the copyright holders of this software give you
* permission to link this software with independent modules, and to copy and
* distribute the resulting executable under terms of your choice, provided that
* you also meet, for each linked independent module, the terms and conditions of
* the license of that module.  An independent module is a module which is not
* derived from this software.  The special exception does not apply to any
* modifications of the software.
* 
*      Notwithstanding the above, under no circumstances may you combine this
* software in any way with any other Broadcom software provided under a license
* other than the GPL, without Broadcom's express prior written consent.
*
* $Id: dhd_custom_gpio.c,v 1.1.4.2 2009/04/13 06:09:38 Exp $
*/


#include <typedefs.h>
#include <linuxver.h>
#include <osl.h>
#include <bcmutils.h>

#include <wlioctl.h>
#include <wl_iw.h>

#include <linux/platform_device.h>
#include <linux/wifi_tiwlan.h>

static struct wifi_platform_data *wifi_control_data = NULL;
static struct resource *wifi_irqres = NULL;
#ifdef MODULE
DECLARE_COMPLETION(sdio_wait);
#endif

static int wifi_set_carddetect( int on )
{
	printk("%s = %d\n", __FUNCTION__, on);
	if (wifi_control_data && wifi_control_data->set_carddetect) {
		wifi_control_data->set_carddetect(on);
	}
	return 0;
}

static int wifi_set_power( int on, unsigned long msec )
{
	printk("%s = %d\n", __FUNCTION__, on);
	if (wifi_control_data && wifi_control_data->set_power) {
		wifi_control_data->set_power(on);
	}
	if (msec)
		mdelay(msec);
	return 0;
}

static int wifi_set_reset( int on, unsigned long msec )
{
	printk("%s = %d\n", __FUNCTION__, on);
	if (wifi_control_data && wifi_control_data->set_reset) {
		wifi_control_data->set_reset(on);
	}
	if (msec)
		mdelay(msec);
	return 0;
}

static int wifi_probe( struct platform_device *pdev )
{
	struct wifi_platform_data *wifi_ctrl = (struct wifi_platform_data *)(pdev->dev.platform_data);

	printk("%s\n", __FUNCTION__);
	wifi_irqres = platform_get_resource_byname(pdev, IORESOURCE_IRQ, "device_wifi_irq");
	if (wifi_irqres) {
		printk("wifi_irqres->start = %lu\n", (unsigned long)(wifi_irqres->start));
		printk("wifi_irqres->flags = %lx\n", wifi_irqres->flags);
	}
	wifi_control_data = wifi_ctrl;
	wifi_set_power(1, 0);
	wifi_set_reset(0, 0);
	wifi_set_carddetect(1);
	return 0;
}

static int wifi_remove( struct platform_device *pdev )
{
	struct wifi_platform_data *wifi_ctrl = (struct wifi_platform_data *)(pdev->dev.platform_data);

	printk("%s\n", __FUNCTION__);
	wifi_control_data = wifi_ctrl;
	wifi_set_carddetect(0);
	wifi_set_reset(1, 0);
	wifi_set_power(0, 0);
	return 0;
}

static struct platform_driver bcm4329_wlan_device = {
	.probe          = wifi_probe,
	.remove         = wifi_remove,
	.suspend        = NULL,
	.resume         = NULL,
	.driver         = {
		.name   = "bcm4329_wlan",
	},
};

void dhd_customer_wifi_complete( void )
{
#ifdef MODULE
	complete(&sdio_wait);
#endif
}

int dhd_customer_wifi_add_dev( void )
{
	printk("%s\n", __FUNCTION__);

	if (platform_driver_register( &bcm4329_wlan_device ))
		return -ENODEV;

#ifdef MODULE
	if (!wait_for_completion_timeout(&sdio_wait, msecs_to_jiffies(10000))) {
		printk(KERN_ERR "%s: Timed out waiting for device detect\n", __FUNCTION__);
		return -ENODEV;
	}
#endif
	return 0;
}

void dhd_customer_wifi_del_dev( void )
{
	printk("%s\n", __FUNCTION__);
	platform_driver_unregister( &bcm4329_wlan_device );
}

/* Customer specific function to insert/remove wlan reset gpio pin */
void dhd_customer_gpio_wlan_reset( bool onoff )
{
	if (onoff == G_WLAN_SET_OFF) {
		printk("%s: assert WLAN RESET\n", __FUNCTION__);
		wifi_set_reset(1, 0);
		wifi_set_power(0, 0);
	}
	else {
		printk("%s: remove WLAN RESET\n", __FUNCTION__);
		wifi_set_power(1, 0);
		wifi_set_reset(0, 0);
	}
}
