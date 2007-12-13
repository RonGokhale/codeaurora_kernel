/* arch/arm/mach-msm/rpc_tlmm.c
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
#include <linux/msm_rpcrouter.h>
#include <asm/arch/msm_rpcrouter.h>
#include <asm/mach-types.h>
#include "rpc_tlmm.h"

static rpcrouterclient_t *rpc_client;
static rpcrouter_address tlmmsvc_addr;

static int rpc_gpio_tlmm_get_config(uint32_t gpio_number, uint32_t *config)
{
	tlmm_gpio_tlmm_get_config_msg msg;
	tlmm_gpio_tlmm_get_config_rets *rets;
	oncrpc_reply_hdr *rep;
	int rc;

	rpcrouter_kernapi_setup_request(&msg.hdr, APP_TLMM_PROG, APP_TLMM_VER,
					TLMM_PROCEEDURE_GPIO_TLMM_GET_CONFIG,
				    sizeof(tlmm_gpio_tlmm_get_config_req_args));

	msg.args.gpio_number = cpu_to_be32(gpio_number);

	rc = rpcrouter_kernapi_write(rpc_client, &tlmmsvc_addr, &msg,
					sizeof(msg));
	if (rc < 0)
		return rc;

	rc = rpcrouter_kernapi_read(rpc_client, (void **) &rep, (5*HZ));
	if (rc < 0)
		return rc;

	rets = (tlmm_gpio_tlmm_get_config_rets *) ((void *) rep +
						sizeof(oncrpc_reply_hdr));
	*config = rets->gpio_config_data;
	kfree(rep);
	return 0;
}

static int rpc_gpio_tlmm_config_remote(uint32_t config)
{
	tlmm_gpio_tlmm_config_remote_msg msg;
	tlmm_gpio_tlmm_config_remote_rets *rets;
	oncrpc_reply_hdr *rep;
	int rc;

	rpcrouter_kernapi_setup_request(&msg.hdr, APP_TLMM_PROG, APP_TLMM_VER,
					TLMM_PROCEEDURE_GPIO_TLMM_CONFIG_REMOTE,
				 sizeof(tlmm_gpio_tlmm_config_remote_req_args));

	msg.args.gpio_config_data = cpu_to_be32(config);

	rc = rpcrouter_kernapi_write(rpc_client, &tlmmsvc_addr, &msg,
				sizeof(msg));
	if (rc < 0)
		return rc;

	rc = rpcrouter_kernapi_read(rpc_client, (void **) &rep, (5*HZ));
	if (rc < 0)
		return rc;

	rets = (tlmm_gpio_tlmm_config_remote_rets *) ((void *) rep +
						sizeof(oncrpc_reply_hdr));

	rc = (be32_to_cpu(rets->result) == 1) ? 0 : -EREMOTEIO;
	kfree(rep);
	return rc;
}

static int rpc_tlmm_program_gpio_table(uint32_t *table)
{
	int i, rc;

	for (i = 0; table[i] != 0xffffffff; i++) {
		rc = rpc_gpio_tlmm_config_remote(table[i]);
		if (rc < 0)
			return rc;
	}
	return 0;
}

static int __init rpc_tlmm_init(void)
{
	rpcrouter_xport_address	xport_addr;
	int rc;


	xport_addr.xp = RPCROUTER_XPORT_SMD;
	xport_addr.port = 2;

	rc = rpcrouter_kernapi_openxport(&xport_addr);
	if (rc < 0) {
		printk(KERN_ERR "rpc_tlmm: Error opening SMD xport (%d)\n", rc);
		return rc;
	}

	rc = rpcrouter_kernapi_open(RPCTLMM_CLIENT_ID, &rpc_client);
	if (rc < 0) {
		printk(KERN_ERR "rpc_tlmm: Error opening SMD client (%d)\n",
			rc);
		return rc;
	}

	rc = rpcrouter_kernapi_getdest(rpc_client, APP_TLMM_PROG, APP_TLMM_VER,
					(5 * HZ), &tlmmsvc_addr);
	if (rc < 0) {
		printk(KERN_ERR "rpc_tlmm: Can't find tlmm service (%d)\n",
			 rc);
		return rc;
	}

	return 0;
}

module_init(rpc_tlmm_init);

MODULE_DESCRIPTION("RPC TLMM driver");
MODULE_AUTHOR("San Mehat <san@android.com>");
MODULE_LICENSE("GPL");
