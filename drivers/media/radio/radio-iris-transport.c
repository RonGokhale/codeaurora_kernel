
/*
 *  Qualcomm's FM Shared Memory Transport Driver
 *
 *  FM HCI_SMD ( FM HCI Shared Memory Driver) is Qualcomm's Shared memory driver
 *  for the HCI protocol. This file is based on drivers/bluetooth/hci_vhci.c
 *
 *  Copyright (c) 2000-2001, 2011 Code Aurora Forum. All rights reserved.
 *
 *  Copyright (C) 2002-2003  Maxim Krasnyansky <maxk@qualcomm.com>
 *  Copyright (C) 2004-2006  Marcel Holtmann <marcel@holtmann.org>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2
 *  as published by the Free Software Foundation
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/skbuff.h>

#include <media/radio-iris.h>
#include <mach/msm_smd.h>

struct radio_data {
	struct hci_dev *hdev;
	void *data;
};

struct radio_data hs_radio;
struct smd_channel *fm_channel;

static void radio_hci_smd_notify_event(void *, unsigned int);

static int radio_hci_smd_open(struct hci_dev *hdev)
{
	int rc;

	/* Open the SMD Channel and device and register the callback function*/
	rc =  smd_named_open_on_edge("APPS_FM", SMD_APPS_WCNSS,
		&fm_channel, &hs_radio, radio_hci_smd_notify_event);

	if (rc < 0) {
		FMDERR("Cannot open the SMD Channel\n");
		kfree(fm_channel);
		return -ENODEV;
	}

	/* Disabling interupts when RIVA reads something */
	smd_disable_read_intr(fm_channel);

	return 0;
}


static int radio_hci_smd_close(struct hci_dev *hdev)
{

	return smd_close(fm_channel);
}


static int radio_hci_smd_recv_frame(struct hci_dev *hdev)
{
	int len;
	struct sk_buff *skb;
	struct smd_channel *channel;
	unsigned  char *buf;
	int rc;

	channel = fm_channel;

	len = smd_cur_packet_size(fm_channel);

	while (len) {
		skb = alloc_skb(len, GFP_KERNEL);
		if (!skb)
			return -ENOMEM;

		buf = kmalloc(len, GFP_KERNEL);
		if (!buf)
			return -ENOMEM;

		rc = smd_read(channel, (void *)buf, len);
		if (rc < 0)
			return -EINVAL;

		if (memcpy(skb_put(skb, len), buf, len)) {
			kfree_skb(skb);
			return -EFAULT;
		}

		skb->dev = (void *)hdev;

		radio_hci_recv_frame(skb);

		kfree(skb);
		kfree(buf);

		len = smd_cur_packet_size(fm_channel);
	}

	return 0;
}

static int radio_hci_smd_send_frame(struct sk_buff *skb)
{
	struct hci_dev *hdev = (struct hci_dev *)skb->dev;
	int rc;

	if (!hdev) {
		FMDERR("Frame for unknown HCI device (hdev=NULL)");
		return -ENODEV;
	}

	rc = smd_write(fm_channel, skb->data, skb->len);
	if (rc < 0) {
		FMDERR("Not enough memory in SMD Channel");
		return -ENOMEM;
	}

	return 0;
}


static void radio_hci_smd_notify_event(void *data, unsigned int event)
{
	struct hci_dev *hdev = ((struct radio_data *)data)->hdev;

	if (!hdev) {
		FMDERR("Frame for unknown HCI device (hdev=NULL)");
		return;
	}

	switch (event) {
	case SMD_EVENT_DATA:
		radio_hci_smd_recv_frame(hdev);
		break;
	case SMD_EVENT_OPEN:
		radio_hci_smd_open(hdev);
		break;
	case SMD_EVENT_CLOSE:
		radio_hci_smd_close(hdev);
		break;
	default:
		break;
	}

}

static int radio_hci_smd_register_dev(struct radio_data *hs_radio)
{
	struct hci_dev *hdev;
	int len;

	len = sizeof(struct hci_dev);
	hdev = kmalloc(len, GFP_KERNEL);
	if (!hdev) {
		FMDERR("Error in allocating memory\n");
		return -ENOMEM;
	}

	hs_radio->hdev = hdev;
	hdev->open  = radio_hci_smd_open;
	hdev->close = radio_hci_smd_close;
	hdev->send  = radio_hci_smd_send_frame;

	if (radio_hci_register_dev(hdev) < 0) {
		FMDERR("Can't register HCI device");
		kfree(hdev);
		return -ENODEV;
	}

	return 0;
}

static void radio_hci_smd_deregister(void)
{
	smd_close(fm_channel);
}

static int __init radio_hci_smd_init(void)
{
	return radio_hci_smd_register_dev(&hs_radio);
}
module_init(radio_hci_smd_init);

static void __exit radio_hci_smd_exit(void)
{
	radio_hci_smd_deregister();
}
module_exit(radio_hci_smd_exit);

MODULE_DESCRIPTION("Qualcomm FM SMD Driver");
MODULE_AUTHOR("Ankur Nandwani <ankurn@codeaurora.org>");
MODULE_LICENSE("GPL v2");
