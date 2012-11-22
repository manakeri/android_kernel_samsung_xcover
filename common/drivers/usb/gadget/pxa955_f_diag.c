/*
 * pxa955_f_diag.c -- diag function driver
 *
 * Copyright (C) 2003 Al Borchers (alborchers@steinerpoint.com)
 * Copyright (C) 2008 by David Brownell
 * Copyright (C) 2008 by Nokia Corporation
 * Copyright (C) 2010 by Marvell Corporation
 *
 * This software is distributed under the terms of the GNU General
 * Public License ("GPL") as published by the Free Software Foundation,
 * either version 2 of that License or (at your option) any later version.
 */

/* #define VERBOSE_DEBUG */

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/usb/android_composite.h>

#include "pxa955_diag_serial.h"
#include "gadget_chips.h"

struct diag_ep_descs {
	struct usb_endpoint_descriptor	*in;
	struct usb_endpoint_descriptor	*out;
};

struct f_diag {
	struct diag_serial			port;
	u8							ctrl_id;
	u8							port_num;

	u8							pending;

	/* lock is mostly for pending... they get accessed
	 * by callbacks both from tty (open/close/break) under its spinlock,
	 * which can't use that lock.
	 */
	spinlock_t					lock;

	struct diag_ep_descs		fs;
	struct diag_ep_descs		hs;
};

static inline struct f_diag *func_to_diag(struct usb_function *f)
{
	return container_of(f, struct f_diag, port.func);
}

static inline struct f_diag *port_to_diag(struct diag_serial *p)
{
	return container_of(p, struct f_diag, port);
}

/* interface and class descriptors: */

static struct usb_interface_descriptor diag_interface_desc __initdata = {
	.bLength =		USB_DT_INTERFACE_SIZE,
	.bDescriptorType =	USB_DT_INTERFACE,
	/* .bInterfaceNumber =  DYNAMIC */
	.bNumEndpoints =	2,
//dh0318.lee 101227 FOR_ACM_DIAG 
	.bInterfaceClass =	0xFF,//USB_CLASS_CDC_DATA,
	.bInterfaceSubClass =	0x20,//0,
	.bInterfaceProtocol =	0x01,//0xff,
	/* .iInterface = DYNAMIC */
};

/* full speed support: */

static struct usb_endpoint_descriptor diag_fs_in_desc __initdata = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
};

static struct usb_endpoint_descriptor diag_fs_out_desc __initdata = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_OUT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
};

static struct usb_descriptor_header *diag_fs_function[] __initdata = {
	(struct usb_descriptor_header *) &diag_interface_desc,
	(struct usb_descriptor_header *) &diag_fs_in_desc,
	(struct usb_descriptor_header *) &diag_fs_out_desc,
	NULL,
};

/* high speed support: */

static struct usb_endpoint_descriptor diag_hs_in_desc __initdata = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	cpu_to_le16(512),
};

static struct usb_endpoint_descriptor diag_hs_out_desc __initdata = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	cpu_to_le16(512),
};

static struct usb_descriptor_header *diag_hs_function[] __initdata = {
	(struct usb_descriptor_header *) &diag_interface_desc,
	(struct usb_descriptor_header *) &diag_hs_in_desc,
	(struct usb_descriptor_header *) &diag_hs_out_desc,
	NULL,
};

/* string descriptors: */

#define DIAG_IDX	0

/* static strings, in UTF-8 */
static struct usb_string diag_string_defs[] = {
	[DIAG_IDX].s = "Marvell Diagnostics",
	{  /* ZEROES END LIST */ },
};

static struct usb_gadget_strings diag_string_table = {
	.language =		0x0409,	/* en-us */
	.strings =		diag_string_defs,
};

static struct usb_gadget_strings *diag_strings[] = {
	&diag_string_table,
	NULL,
};

/***********************************************************************
* Function: diag_setup
************************************************************************
* Description:  Handles usb setup requests
*		 Not relevant for diag function, as it is currently
*		 based on vendor specific class
*
* Parameters:   f          - pointer to usb_function
*		ctrl       - usb controll request
*
* Return value: Operation result
*
* Notes:
*
***********************************************************************/
static int diag_setup(struct usb_function *f,
	const struct usb_ctrlrequest *ctrl)
{
	return 0;
}

/***********************************************************************
* Function: diag_set_alt
************************************************************************
* Description:  Set alternate function
*
* Parameters:   f          - pointer to usb_function
*		intf       - required interface number
*		alt        -
*
* Return value: Operation result
*
* Notes:
*
***********************************************************************/
static int diag_set_alt(struct usb_function *f, unsigned intf, unsigned alt)
{
	struct f_diag		*diag = func_to_diag(f);
	struct usb_composite_dev *cdev = f->config->cdev;
	int ret;

	/* we know alt == 0, so this is an activation or a reset */
	if (diag->port.in->driver_data) {
		DBG(cdev, "reset diag ttyDIAG%d\n", diag->port_num);
		diag_serial_disconnect(&diag->port);
	} else {
		DBG(cdev, "activate diag ttyDIAG%d\n", diag->port_num);
	}
	diag->port.in->driver_data = NULL;
	diag->port.in_desc = ep_choose(cdev->gadget,
			diag->hs.in, diag->fs.in);
	diag->port.out_desc = ep_choose(cdev->gadget,
			diag->hs.out, diag->fs.out);
	ret = diag_serial_connect(&diag->port, diag->port_num);
	return ret;
}

static void diag_set_first_interfrace(struct usb_function *f, unsigned intf)
{
	struct f_diag		*diag = func_to_diag(f);
	diag->ctrl_id = intf;
}


/***********************************************************************
* Function: diag_disable
************************************************************************
* Description:  Disables diag function
*
* Parameters:   f          - pointer to usb_function
*
* Return value: None
*
* Notes:
*
***********************************************************************/
static void diag_disable(struct usb_function *f)
{
	struct f_diag	*diag = func_to_diag(f);
	struct usb_composite_dev *cdev = f->config->cdev;

	DBG(cdev, "diag ttyDIAG%d deactivated\n", diag->port_num);
	diag_serial_disconnect(&diag->port);
}

/*-------------------------------------------------------------------------*/

/* connect == the TTY link is open */

/***********************************************************************
* Function: diag_connect
************************************************************************
* Description:  Callback function upon diag connection
*
* Parameters:   port       - pointer to diag_serial
*
* Return value: None
*
* Notes:
*
***********************************************************************/
static void diag_connect(struct diag_serial *port)
{
}

/***********************************************************************
* Function: diag_disconnect
************************************************************************
* Description:  Callback function upon diag disconnect
*
* Parameters:   port       - pointer to diag_serial
*
* Return value: None
*
* Notes:
*
***********************************************************************/
static void diag_disconnect(struct diag_serial *port)
{
}

/***********************************************************************
* Function: diag_send_break
************************************************************************
* Description:  Callback function upon diag disconnect
*
* Parameters:   port       - pointer to diag_serial
*               duration   - duration period
*
* Return value: Operation result
*
* Notes:
*
***********************************************************************/
static int diag_send_break(struct diag_serial *port, int duration)
{
	return 0;
}

/*-------------------------------------------------------------------------*/

/***********************************************************************
* Function: diag_bind
************************************************************************
* Description:  Diag function setup	/ bind
*
* Parameters:   c      - pointer to usb_configuration
*               f	- pointer to usb_function
*
* Return value: Operation result
*
* Notes:
*
***********************************************************************/
static int __init
diag_bind(struct usb_configuration *c, struct usb_function *f)
{
	struct usb_composite_dev *cdev = c->cdev;
	struct f_diag		*diag = func_to_diag(f);
	int			status;
	struct usb_ep		*ep;

	/* allocate instance-specific interface IDs, and patch descriptors */
	status = usb_interface_id(c, f);
	if (status < 0)
		goto fail;
	diag->ctrl_id = status;

	diag_interface_desc.bInterfaceNumber = status;
	status = -ENODEV;

	/* allocate instance-specific endpoints */
	ep = usb_ep_autoconfig(cdev->gadget, &diag_fs_in_desc);
	if (!ep)
		goto fail;
	diag->port.in = ep;
	ep->driver_data = cdev;	/* claim */

	ep = usb_ep_autoconfig(cdev->gadget, &diag_fs_out_desc);
	if (!ep)
		goto fail;
	diag->port.out = ep;
	ep->driver_data = cdev;	/* claim */

	/* copy descriptors, and track endpoint copies */
	f->descriptors = usb_copy_descriptors(diag_fs_function);
	if (!f->descriptors)
		goto fail;

	diag->fs.in = usb_find_endpoint(diag_fs_function,
			f->descriptors, &diag_fs_in_desc);
	diag->fs.out = usb_find_endpoint(diag_fs_function,
			f->descriptors, &diag_fs_out_desc);

	/* support all relevant hardware speeds... we expect that when
	 * hardware is dual speed, all bulk-capable endpoints work at
	 * both speeds
	 */
	if (gadget_is_dualspeed(c->cdev->gadget)) {
		diag_hs_in_desc.bEndpointAddress =
				diag_fs_in_desc.bEndpointAddress;
		diag_hs_out_desc.bEndpointAddress =
				diag_fs_out_desc.bEndpointAddress;

		/* copy descriptors, and track endpoint copies */
		f->hs_descriptors = usb_copy_descriptors(diag_hs_function);

		diag->hs.in = usb_find_endpoint(diag_hs_function,
				f->hs_descriptors, &diag_hs_in_desc);
		diag->hs.out = usb_find_endpoint(diag_hs_function,
				f->hs_descriptors, &diag_hs_out_desc);
	}

	DBG(cdev, "diag ttyDIAG%d: %s speed IN/%s OUT/%s\n",
			diag->port_num,
			gadget_is_dualspeed(c->cdev->gadget) ? "dual" : "full",
			diag->port.in->name, diag->port.out->name);
	return 0;
fail:
	/* we might as well release our claims on endpoints */
	if (diag->port.out)
		diag->port.out->driver_data = NULL;
	if (diag->port.in)
		diag->port.in->driver_data = NULL;

	ERROR(cdev, "%s/%p: can't bind, err %d\n", f->name, f, status);

	return status;
}

static void
diag_unbind(struct usb_configuration *c, struct usb_function *f)
{
	struct f_diag		*diag = func_to_diag(f);

	if (gadget_is_dualspeed(c->cdev->gadget))
		usb_free_descriptors(f->hs_descriptors);
	usb_free_descriptors(f->descriptors);
	kfree(diag);
}

/***********************************************************************
* Function: diag_bind_config
************************************************************************
* Description:  Add a diag function to a configuration
*
* Parameters:   c          - pointer to usb_configuration
*               port_num   - /dev/ttyDIAG* port this interface will use
*
* Return value: Operation result
*
* Notes:
*		Context: single threaded during gadget setup
*		Caller must have called diag_serial_setup() with
*		enough ports to handle all the ones it binds.  Caller
*		is also responsible for calling diag_serial_cleanup()
*		before module unload.
***********************************************************************/
int __init diag_bind_config(struct usb_configuration *c, u8 port_num)
{
	struct f_diag	*diag;
	int		status;

	/* maybe allocate device-global string IDs, and patch descriptors */
	if (diag_string_defs[DIAG_IDX].id == 0) {
		status = usb_string_id(c->cdev);
		if (status < 0)
			return status;
		diag_string_defs[DIAG_IDX].id = status;

		diag_interface_desc.iInterface = status;
	}

	/* allocate and initialize one new instance */
	diag = kzalloc(sizeof *diag, GFP_KERNEL);
	if (!diag)
		return -ENOMEM;

	spin_lock_init(&diag->lock);

	diag->port_num = port_num;

	diag->port.connect = diag_connect;
	diag->port.disconnect = diag_disconnect;
	diag->port.send_break = diag_send_break;

	diag->port.func.name = "diag";
	diag->port.func.strings = diag_strings;
	/* descriptors are per-instance copies */
	diag->port.func.bind = diag_bind;
	diag->port.func.unbind = diag_unbind;
	diag->port.func.set_alt = diag_set_alt;
	diag->port.func.setup = diag_setup;
	diag->port.func.disable = diag_disable;
#ifdef CONFIG_DYNAMIC_INTERFACE_DESC_INDEX
	diag->port.func.set_first_interfrace = diag_set_first_interfrace;
#endif
	/* start disabled */
	diag->port.func.disabled = 1;

	status = usb_add_function(c, &diag->port.func);
	if (status)
		kfree(diag);
	return status;
}

#ifdef CONFIG_USB_ANDROID_PXA955_DIAG

int diag_function_bind_config(struct usb_configuration *c)
{
	int ret = diag_bind_config(c, 0);
	if (ret == 0)
		diag_serial_setup(c->cdev->gadget, 1);
	return ret;
}

static struct android_usb_function diag_function = {
	.name = "diag",
	.bind_config = diag_function_bind_config,
};

static int __init init(void)
{
	printk(KERN_INFO "f_diag init\n");
	android_register_function(&diag_function);
	return 0;
}
module_init(init);

#endif /* CONFIG_USB_ANDROID_PXA955_DIAG */
