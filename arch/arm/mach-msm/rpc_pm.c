/* arch/arm/mach-msm/rpc_pmapp.c
**
** Copyright (C) 2007 Google, Inc.
**
** This software is licensed under the terms of the GNU General Public
** License version 2, as published by the Free Software Foundation, and
** may be copied, distributed, and modified under those terms.
**
** This program is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU General Public License for more details.
**
** Author: San Mehat <san@google.com>
**
*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/cdev.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/platform_device.h>

#include <linux/msm_rpcrouter.h>
#include <asm/arch/msm_rpcrouter.h>
#include <asm/arch/rpc_pm.h>
#include "rpc_pm.h"

static rpcrouterclient_t *rpc_client;
static rpcrouter_address pmsvc_addr;

static int rpc_pm_vote_vreg_switch(int enable, uint32_t vreg_id,
				   uint32_t app_mask)
{
	struct pmrpc_votevregswitch_req req;
	void *rsp;
	int rc;

	rpcrouter_kernapi_setup_request(&req.hdr, APP_PM_PROG, APP_PM_VER,
					PM_PROCEEDURE_VOTEVREGSWITCH);

	if (enable)
		req.cmd = cpu_to_be32(1);
	else
		req.cmd = cpu_to_be32(0);
	req.vreg_id = cpu_to_be32(vreg_id);
	req.app_mask = cpu_to_be32(app_mask);

	rc = rpcrouter_kernapi_write(rpc_client, &pmsvc_addr, &req, sizeof(req));
	if (rc < 0)
		return rc;

	rc = rpcrouter_kernapi_read(rpc_client, &rsp, -1, (5*HZ));
	if (rc < 0)
		return rc;

	kfree(rsp);
	return 0;
}

static int rpc_pm_probe(struct platform_device *data)
{
	int rc;
	struct rpcsvr_platform_device *pdev;

	printk(KERN_INFO "%s:\n", __FUNCTION__);
	pdev  = (struct rpcsvr_platform_device *) data;

	rc = rpcrouter_kernapi_open(&rpc_client);
	if (rc < 0) {
		printk(KERN_ERR "rpc_pm: Error opening SMD client (%d)\n", rc);
		return rc;
	}

	memcpy(&pmsvc_addr, &pdev->addr, sizeof(rpcrouter_address));

	return 0;
}

static struct platform_driver rpc_pm_driver = {
	.probe	= rpc_pm_probe,
	.driver	= {
		.name	= APP_PM_PDEV_NAME,
		.owner	= THIS_MODULE,
	},
};

static int __init rpc_pm_init(void)
{
	return platform_driver_register(&rpc_pm_driver);
}

module_init(rpc_pm_init);

MODULE_DESCRIPTION("RPC Power Management");
MODULE_AUTHOR("San Mehat <san@android.com>");
MODULE_LICENSE("GPL");
