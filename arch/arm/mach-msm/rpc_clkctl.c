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
#include <linux/msm_rpcrouter.h>
#include <asm/arch/rpc_clkctl.h>
#include <asm/arch/msm_rpcrouter.h>
#include "rpc_clkctl.h"

extern void clock_register_rpc(struct clkctl_rpc_ops *rpc_ops);

static rpcrouterclient_t *rpc_client;
static rpcrouter_address clkctlsvc_addr;

static int rpc_clkctl_set_flags(uint32_t clock, uint32_t flags)
{
	return -ENOSYS;
}

static int rpc_clkctl_enable(uint32_t clock)
{
	clkctl_rpc_enable_msg msg;
	oncrpc_reply_hdr *rep;
	int rc;

	rpcrouter_kernapi_setup_request(&msg.hdr, APP_CLKCTL_PROG,
					APP_CLKCTL_VER,
					CLKCTL_PROCEEDURE_ENABLE,
					sizeof(clkctl_rpc_enable_req_args));

	msg.args.clock = cpu_to_be32(clock);

	rc = rpcrouter_kernapi_write(rpc_client,
				     &clkctlsvc_addr,
				     &msg,
				     sizeof(msg));
	if (rc < 0)
		return rc;

	rc = rpcrouter_kernapi_read(rpc_client,
				    (void **) &rep,
				    (5*HZ));
	if (rc < 0)
		return rc;

	kfree(rep);
	return 0;
}

static void rpc_clkctl_disable(uint32_t clock)
{
	clkctl_rpc_disable_msg msg;
	oncrpc_reply_hdr *rep;
	int rc;

	rpcrouter_kernapi_setup_request(&msg.hdr, APP_CLKCTL_PROG,
					APP_CLKCTL_VER,
					CLKCTL_PROCEEDURE_DISABLE,
					sizeof(clkctl_rpc_disable_req_args));

	msg.args.clock = cpu_to_be32(clock);

	rc = rpcrouter_kernapi_write(rpc_client,
				     &clkctlsvc_addr,
				     &msg,
				     sizeof(msg));
	if (rc < 0)
		return;

	rc = rpcrouter_kernapi_read(rpc_client,
				    (void **) &rep,
				    (5*HZ));
	if (rc < 0)
		return;

	kfree(rep);
	return;
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
	clkctl_rpc_pllrequest_msg msg;
	oncrpc_reply_hdr *rep;
	int rc;

	printk(KERN_INFO "rpc_clkctl_pll_request(): PLL %d, Enable = %d\n",
	       pll, enable);

	rpcrouter_kernapi_setup_request(&msg.hdr, APP_CLKCTL_PROG,
					APP_CLKCTL_VER,
					CLKCTL_PROCEEDURE_PLLREQUEST,
					sizeof(clkctl_rpc_pllrequest_req_args));

	msg.args.pll = cpu_to_be32(pll);
	msg.args.enable = cpu_to_be32(enable);

	rc = rpcrouter_kernapi_write(rpc_client,
				     &clkctlsvc_addr,
				     &msg,
				     sizeof(msg));
	if (rc < 0)
		return rc;

	rc = rpcrouter_kernapi_read(rpc_client,
				    (void **) &rep,
				    (5*HZ));
	if (rc < 0)
		return rc;

	kfree(rep);
	return rc;
}

static int rpc_clkctl_set_max_rate(uint32_t clock, uint32_t max_rate)
{
	return -ENOSYS;
}

static int rpc_clkctl_set_rate(uint32_t clock, uint32_t rate)
{
	clkctl_rpc_setrate_msg msg;
	clkctl_rpc_setrate_rets *rets;
	oncrpc_reply_hdr *rep;
	int rc;

	rpcrouter_kernapi_setup_request(&msg.hdr, APP_CLKCTL_PROG,
					APP_CLKCTL_VER,
					CLKCTL_PROCEEDURE_SETRATE,
					sizeof(clkctl_rpc_setrate_req_args));

	msg.args.clock = cpu_to_be32(clock);
	msg.args.rate = cpu_to_be32(rate);

	rc = rpcrouter_kernapi_write(rpc_client,
				     &clkctlsvc_addr,
				     &msg,
				     sizeof(msg));
	if (rc < 0)
		return rc;

	rc = rpcrouter_kernapi_read(rpc_client,
				    (void **) &rep,
				    (5*HZ));
	if (rc < 0)
		return rc;

	rets = (clkctl_rpc_setrate_rets *)
		((void *) rep + sizeof(oncrpc_reply_hdr));

	rc = (be32_to_cpu(rets->result) == 0) ? 0 : -EIO;
	kfree(rep);
	return rc;
}

static uint32_t rpc_clkctl_get_rate(uint32_t clock)
{
	uint32_t rate;
	clkctl_rpc_getrate_msg msg;
	clkctl_rpc_getrate_rets *rets;
	oncrpc_reply_hdr *rep;
	int rc;

	rpcrouter_kernapi_setup_request(&msg.hdr, APP_CLKCTL_PROG,
					APP_CLKCTL_VER,
					CLKCTL_PROCEEDURE_GETRATE,
					sizeof(clkctl_rpc_getrate_req_args));

	msg.args.clock = cpu_to_be32(clock);
	rc = rpcrouter_kernapi_write(rpc_client,
				     &clkctlsvc_addr,
				     &msg,
				     sizeof(msg));
	if (rc < 0)
		return 0;

	rc = rpcrouter_kernapi_read(rpc_client,
				    (void **) &rep,
				    (5*HZ));
	if (rc < 0)
		return 0;

	rets = (clkctl_rpc_getrate_rets *)
		((void *) rep + sizeof(oncrpc_reply_hdr));

	rate = be32_to_cpu(rets->rate);
	kfree(rep);
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


static int __init rpc_clkctl_init(void)
{
	rpcrouter_xport_address	xport_addr;
	int rc;

	printk(KERN_INFO "rpc_clkctl: Initializing\n");

	xport_addr.xp = RPCROUTER_XPORT_SMD;
	xport_addr.port = 2;

	rc = rpcrouter_kernapi_openxport(&xport_addr);
	if (rc < 0) {
		printk(KERN_ERR "rpc_clkctl: Error opening SMD xport (%d)\n", rc);
		return rc;
	}

	rc = rpcrouter_kernapi_open(RPCCLKCTL_CLIENT_ID, &rpc_client);
	if (rc < 0) {
		printk(KERN_ERR "rpc_clkctl: Error opening SMD client (%d)\n", rc);
		return rc;
	}

	rc = rpcrouter_kernapi_getdest(rpc_client,
				       APP_CLKCTL_PROG,
				       APP_CLKCTL_VER,
				       (5 * HZ),
				       &clkctlsvc_addr);
	if (rc < 0) {
		printk(KERN_ERR "rpc_clkctl: Unable to locate clkctl service (%d)\n", rc);
		return rc;
	}
	/*
	 * Pass our ops structure to the arch clock driver
	 */
	clock_register_rpc(&rpc_ops);
	return 0;
}

module_init(rpc_clkctl_init);

MODULE_DESCRIPTION("RPC Clock Control");
MODULE_AUTHOR("San Mehat <san@android.com>");
MODULE_LICENSE("GPL");
