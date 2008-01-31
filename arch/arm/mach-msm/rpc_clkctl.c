/* arch/arm/mach-msm/clkctl.c
 *
 * Copyright (C) 2007 Google, Inc.
 * Author: San Mehat <san@android.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
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
#include <asm/arch/rpc_clkctl.h>
#include <asm/arch/msm_rpcrouter.h>
#include "rpc_clkctl.h"
#include "clock.h"

extern void clock_register_rpc(struct clkctl_rpc_ops *rpc_ops);

static rpcrouterclient_t *rpc_client;
static rpcrouter_address clkctlsvc_addr;

static int rpc_clkctl_set_flags(uint32_t clock, uint32_t flags)
{
	return -ENOSYS;
}

static int rpc_clkctl_enable(uint32_t clock)
{
	struct clkctlrpc_enable_req req;
	void *rsp;
	int rc;

	rpcrouter_kernapi_setup_request(&req.hdr, APP_CLKCTL_PROG,
					APP_CLKCTL_VER,
					CLKCTL_PROCEEDURE_ENABLE);

	req.clock = cpu_to_be32(clock);

	rc = rpcrouter_kernapi_write(rpc_client,
				     &clkctlsvc_addr,
				     &req,
				     sizeof(req));
	if (rc < 0)
		return rc;

	rc = rpcrouter_kernapi_read(rpc_client, &rsp, -1, (5 * HZ));
	if (rc < 0)
		return rc;

	kfree(rsp);
	return 0;
}

static int rpc_clkctl_disable(uint32_t clock)
{
	struct clkctlrpc_disable_req req;
	void *rsp;
	int rc;

	rpcrouter_kernapi_setup_request(&req.hdr, APP_CLKCTL_PROG,
					APP_CLKCTL_VER,
					CLKCTL_PROCEEDURE_DISABLE);

	req.clock = cpu_to_be32(clock);

	rc = rpcrouter_kernapi_write(rpc_client,
				     &clkctlsvc_addr,
				     &req,
				     sizeof(req));
	if (rc < 0)
		return rc;

	rc = rpcrouter_kernapi_read(rpc_client, &rsp, -1, (5 * HZ));
	if (rc < 0)
		return rc;

	kfree(rsp);
	return 0;
}

static int rpc_clkctl_reset(uint32_t clock)
{
	return -ENOSYS;
}

static int rpc_clkctl_set_min_rate(uint32_t clock, uint32_t min_rate)
{
	return -ENOSYS;
}

static int rpc_clkctl_pll_request(uint32_t pll, int enable)
{
	struct clkctlrpc_pllrequest_req req;
	void *rsp;
	int rc;

	printk(KERN_INFO "%s: PLL %d, Ena = %d\n", __FUNCTION__, pll, enable);

	rpcrouter_kernapi_setup_request(&req.hdr, APP_CLKCTL_PROG,
					APP_CLKCTL_VER,
					CLKCTL_PROCEEDURE_PLLREQUEST);

	req.pll = cpu_to_be32(pll);
	req.enable = cpu_to_be32(enable);

	rc = rpcrouter_kernapi_write(rpc_client,
				     &clkctlsvc_addr,
				     &req,
				     sizeof(req));
	if (rc < 0)
		return rc;

	rc = rpcrouter_kernapi_read(rpc_client, &rsp, -1, (5 * HZ));
	if (rc < 0)
		return rc;

	kfree(rsp);
	return 0;
}

static int rpc_clkctl_set_max_rate(uint32_t clock, uint32_t max_rate)
{
	return -ENOSYS;
}

static int rpc_clkctl_set_rate(uint32_t clock, uint32_t rate)
{
	struct clkctlrpc_setrate_req req;
	struct clkctlrpc_setrate_rsp *rsp;
	int rc;

	rpcrouter_kernapi_setup_request(&req.hdr, APP_CLKCTL_PROG,
					APP_CLKCTL_VER,
					CLKCTL_PROCEEDURE_SETRATE);

	req.clock = cpu_to_be32(clock);
	req.rate = cpu_to_be32(rate);

	rc = rpcrouter_kernapi_write(rpc_client,
				     &clkctlsvc_addr,
				     &req,
				     sizeof(req));
	if (rc < 0)
		return rc;

	rc = rpcrouter_kernapi_read(rpc_client, (void **) &rsp, -1, (5 * HZ));
	if (rc < 0)
		return rc;

	rc = (be32_to_cpu(rsp->result) == 0) ? 0 : -EREMOTEIO;
	kfree(rsp);
	return rc;
}

static uint32_t rpc_clkctl_get_rate(uint32_t clock)
{
	struct clkctlrpc_getrate_req req;
	struct clkctlrpc_getrate_rsp *rsp;
	int rc;
	uint32_t rate;

	rpcrouter_kernapi_setup_request(&req.hdr, APP_CLKCTL_PROG,
					APP_CLKCTL_VER,
					CLKCTL_PROCEEDURE_GETRATE);

	req.clock = cpu_to_be32(clock);
	rc = rpcrouter_kernapi_write(rpc_client,
				     &clkctlsvc_addr,
				     &req,
				     sizeof(req));
	if (rc < 0)
		return 0;

	rc = rpcrouter_kernapi_read(rpc_client, (void **) &rsp, -1, (5 * HZ));
	if (rc < 0)
		return 0;

	rate = be32_to_cpu(rsp->rate);
	kfree(rsp);
	return rate;
}

static struct clkctl_rpc_ops rpc_ops = {
	.enable = &rpc_clkctl_enable,
	.disable = &rpc_clkctl_disable,
	.reset = &rpc_clkctl_reset,
	.set_flags = &rpc_clkctl_set_flags,
	.set_rate = &rpc_clkctl_set_rate,
	.set_min_rate = &rpc_clkctl_set_min_rate,
	.set_max_rate = &rpc_clkctl_set_max_rate,
	.get_rate = &rpc_clkctl_get_rate,
	.pll_request = &rpc_clkctl_pll_request,
};

static int rpc_clkctl_probe(struct platform_device *data)
{
	struct rpcsvr_platform_device *pdev;
	int rc;

	printk(KERN_INFO "%s:\n", __FUNCTION__);
	pdev = (struct rpcsvr_platform_device *) data;

	rc = rpcrouter_kernapi_open(&rpc_client);
	if (rc < 0) {
		printk(KERN_ERR "rpc_clkctl: Error opening SMD client (%d)\n", rc);
		return rc;
	}

	memcpy(&clkctlsvc_addr, &pdev->addr, sizeof(rpcrouter_address));

	/*
	 * Pass our ops structure to the arch clock driver
	 */
	clock_register_rpc(&rpc_ops);

	printk(KERN_INFO "%s: Done\n", __FUNCTION__);
	return 0;
}

static struct platform_driver rpc_clkctl_driver = {
	.probe	= rpc_clkctl_probe,
	.driver	= {
			.name	= APP_CLKCTL_PDEV_NAME,
			.owner	= THIS_MODULE,
	},
};

static int __init rpc_clkctl_init(void)
{
	return platform_driver_register(&rpc_clkctl_driver);
}

module_init(rpc_clkctl_init);

MODULE_DESCRIPTION("RPC Clock Control");
MODULE_AUTHOR("San Mehat <san@android.com>");
MODULE_LICENSE("GPL");
