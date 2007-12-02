/* drivers/usb/function/usb_function.h
 *
 * USB Function Device Interface
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

#ifndef _DRIVERS_USB_FUNCTION_USB_FUNCTION_H_
#define _DRIVERS_USB_FUNCTION_USB_FUNCTION_H_

#include <linux/list.h>

#define EPT_BULK_IN   1
#define EPT_BULK_OUT  2

struct usb_endpoint;

struct usb_request
{
	void *buf;          /* pointer to associated data buffer */
	unsigned length;    /* requested transfer length */
	int status;         /* status upon completion */
	unsigned actual;    /* actual bytes transferred */

	void (*complete)(struct usb_endpoint *ep, struct usb_request *req);
	void *context;

	struct list_head list;
};

struct usb_function
{
	/* bind() is called once when the function has had its endpoints
	** allocated, but before the bus is active.
	**
	** might be a good place to allocate some usb_request objects
	*/
	void (*bind)(struct usb_endpoint **ept, void *context);

	/* configure() is called when the usb client has been configured
	** by the host and again when the device is unconfigured (or
	** when the client is detached)
	**
	** currently called from interrupt context.
	*/
	void (*configure)(int configured, void *context);

	/* driver name */
	const char *name;
	void *context;

	/* interface class/subclass/protocol for descriptor */
	unsigned char ifc_class;
	unsigned char ifc_subclass;
	unsigned char ifc_protocol;

	/* name string for descriptor */
	const char *ifc_name;

	/* number of needed endpoints and their types */
	unsigned char ifc_ept_count;
	unsigned char ifc_ept_type[8];
};

int usb_function_register(struct usb_function *driver);

/* should not be called from interrupt context */
struct usb_request *usb_ept_alloc_req(struct usb_endpoint *ept, unsigned bufsize);
void usb_ept_free_req(struct usb_endpoint *ept, struct usb_request *req);

/* safely callable from any context
** returns 0 if successfully queued and sets req->status = -EBUSY
** req->status will change to a different value upon completion
** (0 for success, -EIO, -ENODEV, etc for error)
*/
int usb_ept_queue_xfer(struct usb_endpoint *ept, struct usb_request *req);
int usb_ept_cancel_xfer(struct usb_request *req);
#endif
