/*
 * pxa955_diag_serial.h - interface to USB gadget "serial port"/TTY utilities
 *
 * Copyright (C) 2008 David Brownell
 * Copyright (C) 2008 by Nokia Corporation
 * Copyright (C) 2010 by Marvell Corporation
 *
 * This software is distributed under the terms of the GNU General
 * Public License ("GPL") as published by the Free Software Foundation,
 * either version 2 of that License or (at your option) any later version.
 */

#ifndef __DIAG_SERIAL_H
#define __DIAG_SERIAL_H

#include <linux/usb/composite.h>

/*
 * One non-multiplexed "serial" I/O port ... there can be several of these
 * on any given USB peripheral device, if it provides enough endpoints.
 *
 * The "diag_serial" utility component exists to do one thing:  manage TTY
 * style I/O using the USB peripheral endpoints listed here, including
 * hookups to sysfs and /dev for each logical "tty" device.
 */
struct diag_serial {
	struct usb_function				func;

	/* port is managed by diag_serial_{connect,disconnect} */
	struct diag_port				*ioport;

	struct usb_ep					*in;
	struct usb_ep					*out;
	struct usb_endpoint_descriptor	*in_desc;
	struct usb_endpoint_descriptor	*out_desc;

	/* notification callbacks */
	void (*connect)(struct diag_serial *p);
	void (*disconnect)(struct diag_serial *p);
	int (*send_break)(struct diag_serial *p, int duration);
};

/* utilities to allocate/free request and buffer */
struct usb_request *diag_alloc_req(struct usb_ep *ep,
	unsigned len, gfp_t flags);
void diag_free_req(struct usb_ep *, struct usb_request *req);

/* port setup/teardown is handled by gadget driver */
int diag_serial_setup(struct usb_gadget *g, unsigned n_ports);
void diag_serial_cleanup(void);

/* connect/disconnect is handled by individual functions */
int diag_serial_connect(struct diag_serial *, u8 port_num);
void diag_serial_disconnect(struct diag_serial *);

/* functions are bound to configurations by a config or gadget driver */
int diag_bind_config(struct usb_configuration *c, u8 port_num);
int gser_bind_config(struct usb_configuration *c, u8 port_num);
int obex_bind_config(struct usb_configuration *c, u8 port_num);

#endif /* __DIAG_SERIAL_H */
