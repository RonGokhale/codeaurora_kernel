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
#include <linux/skbuff.h>
#include <linux/netdevice.h>

#include "I_sys_types.h"
#include "IPC_api.h"

#include "danipc_k.h"
#include "danipc_lowlevel.h"

extern struct net_device	*danipc_dev;
extern int			registered;


void send_pkt(struct sk_buff *skb)
{
	danipc_pair_t	*pair = (danipc_pair_t *)&(skb->cb[HADDR_CB_OFFSET]);
	const IPC_agent_t	agent = {
		agentId:	pair->src
	};
	VCHAR		*msg;

	netdev_dbg(skb->dev, "%s: pair={dst=0x%x src=0x%x}\n", __func__,
		pair->dst, pair->src);

	msg = IPC_msg_alloc((IPC_agent_t *)&agent,
			pair->dst,
			skb->data,
			skb->len,
			0x12,
			NULL,
			pair->prio
		);

	if (msg)
		IPC_msg_send((IPC_agent_t *)&agent, msg, pair->prio);
	else
		printk(KERN_ERR "%s: IPC_msg_alloc failed!", __func__);

	dev_kfree_skb(skb);
}


static int delay_skb(struct sk_buff *skb, ipc_to_virt_map_t *map)
{
	int			rc;
	delayed_skb_t		*dskb = (delayed_skb_t *)kmalloc(sizeof(*dskb),
								GFP_ATOMIC);

	if (dskb) {
		unsigned long	flags;
		dskb->skb = skb;
		INIT_LIST_HEAD(&dskb->list);

		spin_lock_irqsave(&skbs_lock, flags);
		list_add_tail(&delayed_skbs, &dskb->list);
		atomic_inc(&map->pending_skbs);
		spin_unlock_irqrestore(&skbs_lock, flags);

		schedule_work(&delayed_skbs_work);
		rc = NETDEV_TX_OK;
	}
	else {
		netdev_err(skb->dev, "cannot allocate delayed_skb_t\n");
		rc = NETDEV_TX_BUSY;	/* Try again sometime */
	}
	return rc;
}

int danipc_hard_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	danipc_pair_t	*pair = (danipc_pair_t *)&(skb->cb[HADDR_CB_OFFSET]);
	ipc_to_virt_map_t *map = &ipc_to_virt_map[IPC_GetNode(pair->dst)][pair->prio];
	int			rc = NETDEV_TX_OK;

	if ( map->paddr && atomic_read(&map->pending_skbs) == 0)
		send_pkt(skb);
	else
		rc = delay_skb(skb, map);

	return rc;
}

static void danipc_print_payload(const char *p, int len)
{
	int i;

	printk("%s:%d\n", __func__, __LINE__);
	for (i = 0; i < len; i++)
		printk("%d 0x%x\n", i, p[i]);
	printk("\n");
}


static void
read_ipc_message(volatile char *const packet, char *buf,
		IPC_message_hdr_t *const first_hdr, const unsigned len,
		u8 cpu_id,
		IPC_trns_priority_e pri)
{
	const IPC_agent_t	agent;
	unsigned		data_len = IPC_FIRST_BUF_DATA_SIZE_MAX;
	unsigned		total_len = 0;
	unsigned		rest_len = len;
	uint8_t			*data_ptr = (uint8_t*)(first_hdr) +
						sizeof(IPC_message_hdr_t);
	IPC_next_buf_ptr_t	next_ptr = NULL;

	if (first_hdr->nextBufPtr)
		first_hdr->nextBufPtr = ipc_to_virt(cpu_id, pri, (u32)first_hdr->nextBufPtr);
	next_ptr = first_hdr->nextBufPtr;

	do
	{
		if (total_len != 0)
		{
			data_len = IPC_NEXT_BUF_DATA_SIZE_MAX;
			data_ptr = (uint8_t*)(next_ptr) + sizeof(IPC_buffer_hdr_t);
			if (next_ptr->nextBufPtr)
				next_ptr->nextBufPtr = ipc_to_virt(cpu_id, pri, (u32)next_ptr->nextBufPtr);
			next_ptr = next_ptr->nextBufPtr;
		}

		/* Clean 2 last bits (service information) */
		next_ptr = (IPC_next_buf_ptr_t)(((uint32_t)next_ptr) & (~IPC_BUF_TYPE_BITS));
		data_len = min(rest_len, data_len);
		rest_len -= data_len;
		memcpy(buf + total_len, data_ptr, data_len);
#if 0
		/*print first x bytes of each buffer*/
		danipc_print_payload(buf + total_len, 20);
#endif
		total_len += data_len;
	} while ((next_ptr != NULL) && (rest_len != 0));

	IPC_buf_free((IPC_agent_t *)&agent, packet, pri);
}

void
handle_incoming_packet(volatile char *const packet, u8 cpu_id, IPC_trns_priority_e pri)
{
	IPC_message_hdr_t *const	first_hdr = (IPC_message_hdr_t *)packet;
	const unsigned			msg_len = first_hdr->msgLen;

	struct sk_buff *skb = netdev_alloc_skb(danipc_dev, msg_len);

	if (skb)
	{
		danipc_pair_t	*pair = (danipc_pair_t *)&(skb->cb[HADDR_CB_OFFSET]);

		pair->dst = first_hdr->destAgentId;
		pair->src = first_hdr->srcAgentId;

		read_ipc_message(packet, skb->data, first_hdr, msg_len, cpu_id, pri);

		netdev_dbg(danipc_dev, "%s() pair={dst=0x%x src=0x%x}\n",
			__func__, pair->dst, pair->src);

		skb_put(skb, msg_len);
		skb_reset_mac_header(skb);

		skb->protocol = cpu_to_be16(AGENTID_TO_COOKIE(pair->dst, pri));

		netif_rx(skb);
	}
}

