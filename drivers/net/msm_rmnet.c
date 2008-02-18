/* linux/drivers/net/msm_rmnet.c
 *
 * Virtual Ethernet Interface for MSM7K Networking
 *
 * Copyright (C) 2007 Google, Inc.
 * Author: Brian Swetland <swetland@google.com>
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
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>

#include <asm/arch/msm_smd.h>

/* XXX should come from smd headers */
#define SMD_PORT_ETHER0 11

struct rmnet_private
{
	smd_channel_t *ch;
	struct net_device_stats stats;
};

static void smd_net_notify(void *_dev, unsigned event)
{
	struct net_device *dev = _dev;
	struct rmnet_private *p = netdev_priv(dev);
	struct sk_buff *skb;
	void *ptr = 0;
	int sz;

	if (event != SMD_EVENT_DATA)
		return;

	for (;;) {
		sz = smd_cur_packet_size(p->ch);
		if (sz == 0) break;
		if (smd_read_avail(p->ch) < sz) break;

		if (sz > 1514) {
			printk(KERN_ERR
			       "rmnet_recv() discarding %d len\n", sz);
		} else {
			skb = dev_alloc_skb(sz + NET_IP_ALIGN);
			if (skb == NULL) {
				printk(KERN_ERR
				       "rmnet_recv() cannot allocate skb\n");
			} else {
				skb->dev = dev;
				skb_reserve(skb, NET_IP_ALIGN);
				ptr = skb_put(skb, sz);
				if (smd_read(p->ch, ptr, sz) != sz) {
					printk(KERN_ERR
					       "rmnet_recv() smd lied about avail?!");
					ptr = 0;
					dev_kfree_skb_irq(skb);
				} else {
					skb->protocol = eth_type_trans(skb, dev);
					p->stats.rx_packets++;
					p->stats.rx_bytes += skb->len;
					netif_rx(skb);
				}
				continue;
			}
		}
		if (smd_read(p->ch, ptr, sz) != sz)
			printk(KERN_ERR "rmnet_recv() smd lied about avail?!");
	}
}

static int rmnet_open(struct net_device *dev)
{
	int r;
	struct rmnet_private *p = netdev_priv(dev);

	printk(KERN_INFO "rmnet_open()\n");
	r = smd_open(SMD_PORT_ETHER0, &p->ch, dev, smd_net_notify);

	if (r < 0)
		return -ENODEV;

	netif_start_queue(dev);
	return 0;
}

static int rmnet_stop(struct net_device *dev)
{
	struct rmnet_private *p = netdev_priv(dev);

	printk(KERN_INFO "rmnet_stop()\n");
	smd_close(p->ch);
	p->ch = 0;

	netif_stop_queue(dev);
	return 0;
}

static int rmnet_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct rmnet_private *p = netdev_priv(dev);
	smd_channel_t *ch = p->ch;

	if (smd_write(ch, skb->data, skb->len) != skb->len) {
		printk(KERN_ERR "rmnet fifo full, dropping packet\n");
	} else {
		p->stats.tx_packets++;
		p->stats.tx_bytes += skb->len;
	}

	dev_kfree_skb_irq(skb);
	return 0;
}

static struct net_device_stats *rmnet_get_stats(struct net_device *dev)
{
	struct rmnet_private *p = netdev_priv(dev);
	return &p->stats;
}

static void rmnet_set_multicast_list(struct net_device *dev)
{
}

static void rmnet_tx_timeout(struct net_device *dev)
{
	printk(KERN_INFO "rmnet_tx_timeout()\n");
}

static void __init rmnet_setup(struct net_device *dev)
{
	dev->open = rmnet_open;
	dev->stop = rmnet_stop;
	dev->hard_start_xmit = rmnet_xmit;
	dev->get_stats = rmnet_get_stats;
	dev->set_multicast_list = rmnet_set_multicast_list;
	dev->tx_timeout = rmnet_tx_timeout;

	dev->watchdog_timeo = 20; /* ??? */

	ether_setup(dev);

	dev->change_mtu = 0; /* ??? */

	random_ether_addr(dev->dev_addr);
}


static int __init rmnet_init(void)
{
	int ret;
	struct net_device *dev;
	struct rmnet_private *p;

	dev = alloc_netdev(sizeof(struct rmnet_private),
			   "rmnet%d", rmnet_setup);

	printk(KERN_INFO "rmnet_init() %p\n", dev);

	if (!dev)
		return -ENOMEM;

	p = netdev_priv(dev);

	ret = register_netdev(dev);
	if (ret)
		free_netdev(dev);
	return ret;
}


module_init(rmnet_init);
