/* drivers/rtc/rtc-msm7x00a.c
 *
 * Copyright (C) 2008 Google, Inc.
 * Author: San Mehat <san@google.com>
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
#include <linux/version.h>

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/rtc.h>
#include <linux/msm_rpcrouter.h>

#include <asm/arch/msm_rpcrouter.h>

extern void msm_pm_set_max_sleep_time(int64_t sleep_time_ns);

#define APP_PMLIB_PDEV_NAME_V1 "rs30000061:00000000" /* AMSS6066 */
#define APP_PMLIB_PDEV_NAME_V2 "rs30000061:a3887453" /* AMSS6074 */

/* these procedure numbers happen to be identical between versions */
#define PMLIB_PROCEEDURE_RTC_START	6
#define PMLIB_PROCEEDURE_RTC_GET_TIME	8

static struct msm_rpc_endpoint *ep;
static struct rtc_device *rtc;
static unsigned long rtcalarm_time;

static int
msmrtc_pmlib_set_time(struct device *dev, struct rtc_time *tm)
{
	int rc;
	unsigned long unix_time;

	struct pmlib_rtc_start_req {
		struct rpc_request_hdr hdr;
		uint32_t opt_arg;
		uint32_t seconds;
	} req;

	struct pmlib_rtc_start_rep {
		struct rpc_reply_hdr hdr;
		uint32_t err_flag;
	} rep;

	if (rtc_valid_tm(tm))
		return -EINVAL;

	rtc_tm_to_time(tm, &unix_time);
	req.opt_arg = cpu_to_be32(1);
	req.seconds = cpu_to_be32(unix_time);

	rc = msm_rpc_call_reply(ep, PMLIB_PROCEEDURE_RTC_START,
				&req, sizeof(req),
				&rep, sizeof(rep),
				5 * HZ);
	if (rc < 0)
		return rc;

	return be32_to_cpu(rep.err_flag) ? -EREMOTEIO : 0;
}

static int
msmrtc_pmlib_read_time(struct device *dev, struct rtc_time *tm)
{
	int rc;

	struct pmlib_rtc_get_time_req {
		struct rpc_request_hdr hdr;
		uint32_t time_ptr_not_null;
	} req;

	struct pmlib_rtc_get_time_rep {
		struct rpc_reply_hdr hdr;
		uint32_t err_flag;
		uint32_t opt_arg;
		uint32_t seconds;
	} rep;

	memset(&rep, 0, sizeof(rep));
	req.time_ptr_not_null = cpu_to_be32(1);
	rc = msm_rpc_call_reply(ep, PMLIB_PROCEEDURE_RTC_GET_TIME,
				&req, sizeof(req),
				&rep, sizeof(rep),
				5 * HZ);
	if (rc < 0)
		return rc;


	if (be32_to_cpu(rep.err_flag))
		return -EREMOTEIO;

	if (!be32_to_cpu(rep.opt_arg))
		return -ENODATA;

	rtc_time_to_tm(be32_to_cpu(rep.seconds), tm);

	return 0;
}

static int
msmrtc_virtual_alarm_set(struct device *dev, struct rtc_wkalrm *a)
{
	unsigned long now = get_seconds();

	if (!a->enabled) {
		rtcalarm_time = 0;
		return 0;
	}
	else
		rtc_tm_to_time(&a->time, &rtcalarm_time);

	if (now > rtcalarm_time) {
		printk("%s: Attempt to set alarm in the past\n", __func__);
		rtcalarm_time = 0;
		return -EINVAL;
	}

	return 0;
}

static struct rtc_class_ops msm_rtc_ops = {
	.read_time	= msmrtc_pmlib_read_time,
	.set_time	= msmrtc_pmlib_set_time,
	.set_alarm	= msmrtc_virtual_alarm_set,
};

static void
msmrtc_alarmtimer_expired(unsigned long _data)
{
	printk("RTC_ALARM: GENERATING ALARM EVENT (source = %lu)\n", _data);
	rtc_update_irq(rtc, 1, RTC_IRQF | RTC_AF);
	rtcalarm_time = 0;
}

static int
msmrtc_probe(struct platform_device *pdev)
{
	struct rpcsvr_platform_device *rdev =
		container_of(pdev, struct rpcsvr_platform_device, base);

	ep = msm_rpc_connect(rdev->prog, rdev->vers, 0);
	if (IS_ERR(ep)) {
		printk(KERN_ERR "%s: init rpc failed! rc = %ld\n",
		       __func__, PTR_ERR(ep));
		return PTR_ERR(ep);
	}

	rtc = rtc_device_register("msm_rtc",
				  &pdev->dev,
				  &msm_rtc_ops,
				  THIS_MODULE);
	if (IS_ERR(rtc)) {
		printk(KERN_ERR "%s: Can't register RTC device (%ld)\n",
		       pdev->name, PTR_ERR(rtc));
		return PTR_ERR(rtc);
	}
	return 0;
}

static int
msmrtc_suspend(struct platform_device *dev, pm_message_t state)
{
	if (rtcalarm_time) {
		unsigned long now = get_seconds();
		int diff = rtcalarm_time - now;

		if (diff <=0) {
			msmrtc_alarmtimer_expired(1);
			msm_pm_set_max_sleep_time(0);
			return 0;
		}
		msm_pm_set_max_sleep_time((int64_t) ((int64_t) diff * NSEC_PER_SEC));
	}
	else
		msm_pm_set_max_sleep_time(0);
	return 0;
}

static int
msmrtc_resume(struct platform_device *dev)
{
	if (rtcalarm_time) {
		struct rtc_time tm;
		unsigned long now;
		int diff;

		msmrtc_pmlib_read_time(NULL, &tm);
		rtc_tm_to_time(&tm, &now);
		diff = rtcalarm_time - now;

		printk(KERN_INFO
		       "%s: Alarm in %d secs (we slept for %lu whole secs)\n",
		       __func__, diff, (now - get_seconds()));

		if (diff <=0)
			msmrtc_alarmtimer_expired(2);
	}
	return 0;
}

static struct platform_driver msmrtc_driver_v1 = {
	.probe		= msmrtc_probe,
	.suspend	= msmrtc_suspend,
	.resume		= msmrtc_resume,
	.driver	= {
		.name	= APP_PMLIB_PDEV_NAME_V1,
		.owner	= THIS_MODULE,
	},
};

static struct platform_driver msmrtc_driver_v2 = {
	.probe		= msmrtc_probe,
	.suspend	= msmrtc_suspend,
	.resume		= msmrtc_resume,
	.driver	= {
		.name	= APP_PMLIB_PDEV_NAME_V2,
		.owner	= THIS_MODULE,
	},
};

static int __init msmrtc_init(void)
{
	int ret;

	rtcalarm_time = 0;

	ret = platform_driver_register(&msmrtc_driver_v1);
	if (ret < 0)
		return ret;
	return platform_driver_register(&msmrtc_driver_v2);
}

module_init(msmrtc_init);

MODULE_DESCRIPTION("RTC driver for Qualcomm MSM7x00a chipsets");
MODULE_AUTHOR("San Mehat <san@android.com>");
MODULE_LICENSE("GPL");
