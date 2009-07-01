/* linux/arch/arm/mach-msm/board-mahimahi-wifi.c
*/
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <asm/mach-types.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <linux/wifi_tiwlan.h>

#include "board-mahimahi.h"

int mahimahi_wifi_power(int on);
int mahimahi_wifi_reset(int on);
int mahimahi_wifi_set_carddetect(int on);

static struct resource mahimahi_wifi_resources[] = {
	[0] = {
		.name		= "bcm4329_wlan_irq",
		.start		= MSM_GPIO_TO_INT(MAHIMAHI_GPIO_WIFI_IRQ),
		.end		= MSM_GPIO_TO_INT(MAHIMAHI_GPIO_WIFI_IRQ),
		.flags          = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL,
	},
};

static struct wifi_platform_data mahimahi_wifi_control = {
	.set_power      = mahimahi_wifi_power,
	.set_reset      = mahimahi_wifi_reset,
	.set_carddetect = mahimahi_wifi_set_carddetect,
};

static struct platform_device mahimahi_wifi_device = {
        .name           = "bcm4329_wlan",
        .id             = 1,
        .num_resources  = ARRAY_SIZE(mahimahi_wifi_resources),
        .resource       = mahimahi_wifi_resources,
        .dev            = {
                .platform_data = &mahimahi_wifi_control,
        },
};

static int __init mahimahi_wifi_init(void)
{
	int ret;

	if (!machine_is_mahimahi())
		return 0;

	printk("%s: start\n", __func__);
	ret = platform_device_register(&mahimahi_wifi_device);
        return ret;
}

late_initcall(mahimahi_wifi_init);
