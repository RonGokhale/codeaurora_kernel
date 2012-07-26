/*
	All files except if stated otherwise in the begining of the file are under the ISC license:
	-----------------------------------------------------------------------------------

	Copyright (c) 2010-2012 Design Art Networks Ltd.

	Permission to use, copy, modify, and/or distribute this software for any
	purpose with or without fee is hereby granted, provided that the above
	copyright notice and this permission notice appear in all copies.

	THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
	WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
	MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
	ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
	WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
	ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
	OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
*/


#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>

#include "danipc_k.h"
#include "danipc_ioctl.h"
#include "I_sys_types.h"
#include "IPC_api.h"
#include "danipc_lowlevel.h"

MODULE_LICENSE("GPL");

/* FIXME: need this in order not to pass `struct net_device' to
 * IPC low level code: it does not understand it.
 */
/*static*/ struct net_device	*danipc_dev;

#define DANIPC_VERSION		"v0.1"


#define TIMER_INTERVAL		10


static void
danipc_timer(unsigned long data)
{
	struct net_device	*dev = (struct net_device *)data;
	danipc_priv_t		*priv = netdev_priv(dev);

	danipc_poll(dev);

	mod_timer(&priv->timer, jiffies + TIMER_INTERVAL);
}


static void danipc_init_timer(struct net_device *dev, danipc_priv_t *priv)
{
	init_timer(&priv->timer);
	priv->timer.expires =	jiffies +  TIMER_INTERVAL;
	priv->timer.data =	(unsigned long)dev;
	priv->timer.function =	danipc_timer;
	add_timer(&priv->timer);
}

static int danipc_open(struct net_device *dev)
{
	danipc_priv_t		*priv = netdev_priv(dev);
	int			rc;

	danipc_init_irq(dev, priv);
	rc = request_irq(dev->irq, danipc_interrupt, 0, dev->name, dev);
	if (rc == 0) {
		danipc_init_timer(dev, priv);
		tasklet_init(&priv->rx_task, high_prio_rx, (unsigned long)dev);

		netif_start_queue(dev);
	}
	return rc;
}


static int danipc_close(struct net_device *dev)
{
	danipc_priv_t		*priv = netdev_priv(dev);

	netif_stop_queue(dev);

	tasklet_kill(&priv->rx_task);
	del_timer_sync(&priv->timer);
	free_irq(dev->irq, dev);
	return 0;
}


static const struct net_device_ops danipc_netdev_ops = {
	.ndo_open		= danipc_open,
	.ndo_stop		= danipc_close,
	.ndo_start_xmit		= danipc_hard_start_xmit,
	.ndo_do_ioctl		= danipc_ioctl,
	.ndo_change_mtu		= eth_change_mtu,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_set_mac_address	= eth_mac_addr,
};


static void __init
danipc_set_dev_addr(uint8_t *dev_addr)
{
	/* DAN's OUI: 0x002486 */
	dev_addr[0] = 0x00;
	dev_addr[1] = 0x24;
	dev_addr[2] = 0x86;
	dev_addr[3] = 0x00;
	dev_addr[4] = 0x00;
	dev_addr[5] = 0x00;
}


static void __init
danipc_dev_priv_init(struct net_device *dev)
{
	danipc_priv_t		*priv = netdev_priv(dev);
	mutex_init(&priv->lock);
}

static void __exit
danipc_dev_priv_cleanup(struct net_device *dev)
{
	danipc_priv_t		*priv = netdev_priv(dev);
	(void)priv;
	mutex_destroy(&priv->lock);
}

static void danipc_exit_module(void)
{
	if (danipc_dev) {
		if (danipc_dev->reg_state == NETREG_REGISTERED) {
			danipc_dev_priv_cleanup(danipc_dev);
			unregister_netdev(danipc_dev);
		}
		free_netdev(danipc_dev);
	}
	danipc_ll_cleanup();

	printk(KERN_INFO "DAN IPC driver %s unregistered.\n", DANIPC_VERSION);
}

/* Our vision of L2 header: it is of type danipc_pair_t
 * it is stored at address skb->cb[HADDR_CB_OFFSET].
 */

static int danipc_header_parse(const struct sk_buff *skb, unsigned char *haddr)
{
	danipc_pair_t	*pair = (danipc_pair_t *)&(skb->cb[HADDR_CB_OFFSET]);
	memcpy(haddr, &pair->src, sizeof(danipc_addr_t));
	return sizeof(danipc_addr_t);
}

int danipc_header(struct sk_buff *skb, struct net_device *dev,
               unsigned short type,
               const void *daddr, const void *saddr, unsigned len)
{
	danipc_pair_t	*pair = (danipc_pair_t *)&(skb->cb[HADDR_CB_OFFSET]);
	const uint8_t	*addr = daddr;

	pair->src = COOKIE_TO_AGENTID(type);
	pair->prio = COOKIE_TO_PRIO(type);
	if (addr)
		pair->dst = *addr;
	return 0;
}

static const struct header_ops danipc_header_ops ____cacheline_aligned = {
	.create		= danipc_header,
	.parse		= danipc_header_parse,
};



static int __init danipc_init_module(void)
{
	int		rc = danipc_ll_init();

	if (rc == 0) {
		struct net_device *dev = alloc_etherdev(sizeof(danipc_priv_t));
		if (dev) {
			danipc_dev = dev;
			strcpy(dev->name, "danipc");
			dev->netdev_ops		= &danipc_netdev_ops;
			dev->header_ops		= &danipc_header_ops;
			dev->irq		= IRQ_IPC;
			danipc_set_dev_addr(dev->dev_addr);

			rc = register_netdev(dev);
			if (rc == 0) {
				danipc_dev_priv_init(dev);
				printk(KERN_INFO "DAN IPC driver %s registered.\n",
					DANIPC_VERSION);
			} else
				printk(KERN_ERR "%s: register_netdev failure\n",
					__func__);
		} else {
			rc = -ENOMEM;
		}
	} else
		printk(KERN_ERR "%s: cannot initialize DAN IPC\n", __func__);

	if (rc)
		danipc_exit_module();

	return rc;
}


module_init(danipc_init_module);
module_exit(danipc_exit_module);
