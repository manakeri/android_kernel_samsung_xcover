/*
 * f_serial.c - generic USB serial function driver
 *
 * Copyright (C) 2003 Al Borchers (alborchers@steinerpoint.com)
 * Copyright (C) 2008 by David Brownell
 * Copyright (C) 2008 by Nokia Corporation
 *
 * This software is distributed under the terms of the GNU General
 * Public License ("GPL") as published by the Free Software Foundation,
 * either version 2 of that License or (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/utsname.h>
#include <linux/device.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/usb/android_composite.h>

#include "pxa910_u_serial.c"

/*
 * This function packages a simple "generic serial" port with no real
 * control mechanisms, just raw data transfer over two bulk endpoints.
 *
 * Because it's not standardized, this isn't as interoperable as the
 * CDC ACM driver.  However, for many purposes it's just as functional
 * if you can arrange appropriate host side drivers.
 */

struct gser_descs {
	struct usb_endpoint_descriptor *in;
	struct usb_endpoint_descriptor *out;
};

struct f_gser {
	struct gserial port;
	u8 data_id;
	u8 port_num;

	struct gser_descs fs;
	struct gser_descs hs;
};

static inline struct f_gser *func_to_gser(struct usb_function *f)
{
	return container_of(f, struct f_gser, port.func);
}

/*-------------------------------------------------------------------------*/

/* interface descriptor: */

static struct usb_interface_descriptor gser_interface_desc __initdata = {
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	/* .bInterfaceNumber = DYNAMIC */
	.bNumEndpoints = 2,
	.bInterfaceClass = 0x0a,
	.bInterfaceSubClass = 0x0,
	.bInterfaceProtocol = 0xff,
	/* .iInterface = DYNAMIC */
};

/* full speed support: */

static struct usb_endpoint_descriptor gser_fs_in_desc __initdata = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = USB_DIR_IN,
	.bmAttributes = USB_ENDPOINT_XFER_BULK,
};

static struct usb_endpoint_descriptor gser_fs_out_desc __initdata = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = USB_DIR_OUT,
	.bmAttributes = USB_ENDPOINT_XFER_BULK,
};

static struct usb_descriptor_header *gser_fs_function[] __initdata = {
	(struct usb_descriptor_header *)&gser_interface_desc,
	(struct usb_descriptor_header *)&gser_fs_in_desc,
	(struct usb_descriptor_header *)&gser_fs_out_desc,
	NULL,
};

/* high speed support: */

static struct usb_endpoint_descriptor gser_hs_in_desc __initdata = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bmAttributes = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize = cpu_to_le16(512),
};

static struct usb_endpoint_descriptor gser_hs_out_desc __initdata = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bmAttributes = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize = cpu_to_le16(512),
};

static struct usb_descriptor_header *gser_hs_function[] __initdata = {
	(struct usb_descriptor_header *)&gser_interface_desc,
	(struct usb_descriptor_header *)&gser_hs_in_desc,
	(struct usb_descriptor_header *)&gser_hs_out_desc,
	NULL,
};

/* string descriptors: */

static struct usb_string gser_string_defs[] = {
	[0].s = "Generic Serial",
	{}			/* end of list */
};

static struct usb_gadget_strings gser_string_table = {
	.language = 0x0409,	/* en-us */
	.strings = gser_string_defs,
};

static struct usb_gadget_strings *gser_strings[] = {
	&gser_string_table,
	NULL,
};

/*-------------------------------------------------------------------------*/

static int gser_set_alt(struct usb_function *f, unsigned intf, unsigned alt)
{
	struct f_gser *gser = func_to_gser(f);
	struct usb_composite_dev *cdev = f->config->cdev;

	/* we know alt == 0, so this is an activation or a reset */

	if (gser->port.in->driver_data) {
		DBG(cdev, "reset generic ttyGS%d\n", gser->port_num);
		gserial_disconnect(&gser->port);
	} else {
		DBG(cdev, "activate generic ttyGS%d\n", gser->port_num);
	}
	gser->port.in_desc = ep_choose(cdev->gadget, gser->hs.in, gser->fs.in);
	gser->port.out_desc = ep_choose(cdev->gadget,
					gser->hs.out, gser->fs.out);
	gserial_connect(&gser->port, gser->port_num);
	return 0;
}

static void gser_disable(struct usb_function *f)
{
	struct f_gser *gser = func_to_gser(f);

	gserial_disconnect(&gser->port);
}

static void gser_set_first_interfrace(struct usb_function *f, unsigned intf)
{
	struct f_gser	*gser = func_to_gser(f);
	gser->data_id = intf;
}

/*-------------------------------------------------------------------------*/

/* serial function driver setup/binding */

static int __init gser_bind(struct usb_configuration *c, struct usb_function *f)
{
	struct usb_composite_dev *cdev = c->cdev;
	struct f_gser *gser = func_to_gser(f);
	int status;
	struct usb_ep *ep;

	/* allocate instance-specific interface IDs */
	status = usb_interface_id(c, f);
	if (status < 0)
		goto fail;
	gser->data_id = status;
	gser_interface_desc.bInterfaceNumber = status;

	status = -ENODEV;

	/* allocate instance-specific endpoints */
	ep = usb_ep_autoconfig(cdev->gadget, &gser_fs_in_desc);
	if (!ep)
		goto fail;
	gser->port.in = ep;
	ep->driver_data = cdev;	/* claim */

	ep = usb_ep_autoconfig(cdev->gadget, &gser_fs_out_desc);
	if (!ep)
		goto fail;
	gser->port.out = ep;
	ep->driver_data = cdev;	/* claim */

	/* copy descriptors, and track endpoint copies */
	f->descriptors = usb_copy_descriptors(gser_fs_function);

	gser->fs.in = usb_find_endpoint(gser_fs_function,
					f->descriptors, &gser_fs_in_desc);
	gser->fs.out = usb_find_endpoint(gser_fs_function,
					 f->descriptors, &gser_fs_out_desc);

	/* support all relevant hardware speeds... we expect that when
	 * hardware is dual speed, all bulk-capable endpoints work at
	 * both speeds
	 */
	if (gadget_is_dualspeed(c->cdev->gadget)) {
		gser_hs_in_desc.bEndpointAddress =
		    gser_fs_in_desc.bEndpointAddress;
		gser_hs_out_desc.bEndpointAddress =
		    gser_fs_out_desc.bEndpointAddress;

		/* copy descriptors, and track endpoint copies */
		f->hs_descriptors = usb_copy_descriptors(gser_hs_function);

		gser->hs.in = usb_find_endpoint(gser_hs_function,
						f->hs_descriptors,
						&gser_hs_in_desc);
		gser->hs.out =
		    usb_find_endpoint(gser_hs_function, f->hs_descriptors,
				      &gser_hs_out_desc);
	}

	DBG(cdev, "generic ttyGS%d: %s speed IN/%s OUT/%s\n",
	    gser->port_num,
	    gadget_is_dualspeed(c->cdev->gadget) ? "dual" : "full",
	    gser->port.in->name, gser->port.out->name);
	return 0;

fail:
	/* we might as well release our claims on endpoints */
	if (gser->port.out)
		gser->port.out->driver_data = NULL;
	if (gser->port.in)
		gser->port.in->driver_data = NULL;

	ERROR(cdev, "%s: can't bind, err %d\n", f->name, status);

	return status;
}

static void gser_unbind(struct usb_configuration *c, struct usb_function *f)
{
	if (gadget_is_dualspeed(c->cdev->gadget))
		usb_free_descriptors(f->hs_descriptors);
	usb_free_descriptors(f->descriptors);
	kfree(func_to_gser(f));
}

/**
 * gser_bind_config - add a generic serial function to a configuration
 * @c: the configuration to support the serial instance
 * @port_num: /dev/ttyGS* port this interface will use
 * Context: single threaded during gadget setup
 *
 * Returns zero on success, else negative errno.
 *
 * Caller must have called @gserial_setup() with enough ports to
 * handle all the ones it binds.  Caller is also responsible
 * for calling @gserial_cleanup() before module unload.
 */
int __init gser_bind_config(struct usb_configuration *c, u8 port_num)
{
	struct f_gser *gser;
	int status;

	/* REVISIT might want instance-specific strings to help
	 * distinguish instances ...
	 */

	/* maybe allocate device-global string ID */
	if (gser_string_defs[0].id == 0) {
		status = usb_string_id(c->cdev);
		if (status < 0)
			return status;
		gser_string_defs[0].id = status;
	}

	/* allocate and initialize one new instance */
	gser = kzalloc(sizeof *gser, GFP_KERNEL);
	if (!gser)
		return -ENOMEM;

	gser->port_num = port_num;

	gser->port.func.name = "diag";
	gser->port.func.strings = gser_strings;
	gser->port.func.bind = gser_bind;
	gser->port.func.unbind = gser_unbind;
	gser->port.func.set_alt = gser_set_alt;
	gser->port.func.disable = gser_disable;
	gser->port.func.set_first_interfrace = gser_set_first_interfrace;

	gser->port.func.disabled = 0;

	init_waitqueue_head(&gser->port.port_send);
	status = usb_add_function(c, &gser->port.func);
	if (status)
		kfree(gser);
	return status;
}

static int gs_marvell_diag_write(struct tty_struct *tty,
				 const unsigned char *buf, int count)
{
	struct gs_port *port = tty->driver_data;
	unsigned long flags;
	int status;

	pr_vdebug("gs_write: ttyGS%d (%p) writing %d bytes\n",
		  port->port_num, tty, count);

	if (port == NULL)
		goto out;
	spin_lock_irqsave(&port->port_lock, flags);
	if (count)
		count = gs_buf_put(&port->port_write_buf, buf, count);

	/* treat count == 0 as flush_chars() */
	if (port->port_usb)
		status = gs_start_tx(port);
	spin_unlock_irqrestore(&port->port_lock, flags);
out:
	if (count == 0)
		return -EAGAIN;
	return count;
}

int gs_marvell_diag_send(const unsigned char *buf, int count)
{
	struct tty_struct *tty;
	struct gs_port *port;
	int c, retval = 0;
	const unsigned char *b = buf;
	unsigned long flags;
	wait_queue_head_t *p_port_send;

	/* Assume that we have only one port */
	port = ports[0].port;
	tty = port->port_tty;

	if (tty == NULL || tty->driver_data == NULL || port->port_usb == NULL)
		return count;
	while (count > 0) {
		c = gs_marvell_diag_write(tty, b, count);

		if (c == count) {
			b += c;
			break;	/* send all bytes successfully */
		} else if (c >= 0) {
			b += c;
			count -= c;
			continue;
		} else if (c < 0) {
			retval = c;
			if (c != -EAGAIN)
				goto break_out;
		}

		/* retry to wait enough buffer to send the rest bytes. */
		spin_lock_irqsave(&port->port_lock, flags);

		if (port->port_usb) {
			p_port_send = &port->port_usb->port_send;
		} else {
			p_port_send = NULL;
			printk(KERN_INFO "%s: usb port is released.\n",
					__func__);
		}

		spin_unlock_irqrestore(&port->port_lock, flags);

		if (p_port_send)
			interruptible_sleep_on_timeout(p_port_send,
						       10 * HZ / 1000);

		port = ports[0].port;
		tty = port->port_tty;
		if (tty == NULL || tty->driver_data == NULL
		    || port->port_usb == NULL)
			return count;
	}
break_out:
	return (b - buf) ? b - buf : retval;
}
EXPORT_SYMBOL(gs_marvell_diag_send);

typedef int (*marvell_diag_rx_callback) (char *packet, int len);

marvell_diag_rx_callback gs_marvell_diag_rx_callback =
    (marvell_diag_rx_callback) NULL;
EXPORT_SYMBOL(gs_marvell_diag_rx_callback);

typedef int (*marvell_diag_ioctl) (unsigned int cmd, unsigned long arg);

marvell_diag_ioctl gs_marvell_diag_ioctl = (marvell_diag_ioctl) NULL;
EXPORT_SYMBOL(gs_marvell_diag_ioctl);
/*
 * RX tasklet takes data out of the RX queue and hands it up to the TTY
 * layer until it refuses to take any more data (or is throttled back).
 * Then it issues reads for any further data.
 *
 * If the RX queue becomes full enough that no usb_request is queued,
 * the OUT endpoint may begin NAKing as soon as its FIFO fills up.
 * So QUEUE_SIZE packets plus however many the FIFO holds (usually two)
 * can be buffered before the TTY layer's buffers (currently 64 KB).
 */
static void gs_marvell_diag_rx_push(unsigned long _port)
{
	struct gs_port *port = (void *)_port;
	struct tty_struct *tty;
	struct list_head *queue = &port->read_queue;
	bool disconnect = false;
	bool do_push = false;
	struct timespec now;

	/* hand any queued data to the tty */
	spin_lock_irq(&port->port_lock);
	tty = port->port_tty;
	while (!list_empty(queue)) {
		struct usb_request *req;

		now = CURRENT_TIME;
		req = list_first_entry(queue, struct usb_request, list);

		/* discard data if tty was closed */
		if (!tty)
			goto recycle;

		/* leave data queued if tty was rx throttled */
		if (test_bit(TTY_THROTTLED, &tty->flags))
			break;

		switch (req->status) {
		case -ESHUTDOWN:
			disconnect = true;
			pr_vdebug(PREFIX "%d: shutdown\n", port->port_num);
			break;

		default:
			/* presumably a transient fault */
			pr_warning(PREFIX "%d: unexpected RX status %d\n",
				   port->port_num, req->status);
			/* FALLTHROUGH */
		case 0:
			/* normal completion */
			break;
		}

		/* push data to (open) tty */
		if (req->actual) {
			char *packet = req->buf;
			unsigned size = req->actual;
			unsigned n;
			int count;

			/* we may have pushed part of this packet already... */
			n = port->n_read;
			if (n) {
				packet += n;
				size -= n;
			}

			if (gs_marvell_diag_rx_callback !=
			    (marvell_diag_rx_callback) NULL) {
				int filtered;
				filtered =
				    gs_marvell_diag_rx_callback(packet, size);
				if (filtered)
					goto recycle;
			}

			count = tty_insert_flip_string(tty, packet, size);
			if (count)
				do_push = true;
			if (count != size) {
				/* stop pushing; TTY layer can't handle more */
				port->n_read += count;
				pr_vdebug(PREFIX "%d: rx block %d/%d\n",
					  port->port_num, count, req->actual);
				break;
			}
			port->n_read = 0;
		}
recycle:
		list_move(&req->list, &port->read_pool);
	}

	/* Push from tty to ldisc; this is immediate with low_latency, and
	 * may trigger callbacks to this driver ... so drop the spinlock.
	 */
	if (tty && do_push) {
		spin_unlock_irq(&port->port_lock);
		tty_flip_buffer_push(tty);
		wake_up_interruptible(&tty->read_wait);
		spin_lock_irq(&port->port_lock);

		/* tty may have been closed */
		tty = port->port_tty;
	}

	/* We want our data queue to become empty ASAP, keeping data
	 * in the tty and ldisc (not here).  If we couldn't push any
	 * this time around, there may be trouble unless there's an
	 * implicit tty_unthrottle() call on its way...
	 *
	 * REVISIT we should probably add a timer to keep the tasklet
	 * from starving ... but it's not clear that case ever happens.
	 */
	if (!list_empty(queue) && tty) {
		if (!test_bit(TTY_THROTTLED, &tty->flags)) {
			if (do_push)
				tasklet_schedule(&port->push);
			else
				pr_warning(PREFIX "%d: RX not scheduled?\n",
					   port->port_num);
		}
	}

	/* If we're still connected, refill the USB RX queue. */
	if (!disconnect && port->port_usb)
		gs_start_rx(port);

	spin_unlock_irq(&port->port_lock);
}

static int __init
gs_marvell_diag_port_alloc(unsigned port_num,
			   struct usb_cdc_line_coding *coding)
{
	struct gs_port *port;

	port = kzalloc(sizeof(struct gs_port), GFP_KERNEL);
	if (port == NULL)
		return -ENOMEM;

	spin_lock_init(&port->port_lock);
	init_waitqueue_head(&port->close_wait);
	init_waitqueue_head(&port->drain_wait);

	tasklet_init(&port->push, gs_marvell_diag_rx_push, (unsigned long)port);

	INIT_LIST_HEAD(&port->read_pool);
	INIT_LIST_HEAD(&port->read_queue);
	INIT_LIST_HEAD(&port->write_pool);

	port->port_num = port_num;
	port->port_line_coding = *coding;

	ports[port_num].port = port;

	return 0;
}

#ifdef DIAG_USB_TEST
static void diagUsbTest3()
{
	int i;
	int ret;
	unsigned long start, end, timeuse;

	memset(testbuf, 'b', sizeof(testbuf));
	printk(KERN_INFO "diag usb test start...\n");
	msleep_interruptible(1000);

	start = jiffies;
	for (i = 0; i < lpcunt; i++) {
		ret = gs_usb_write(gs_diag_tty, testbuf, sizeof(testbuf));
		if (ret != sizeof(testbuf))
			printk(KERN_ERR "usb write error!ret=%d\n", ret);

		while (!writecomplete) {
			if (wait_event_interruptible(wcwaitQ, \
						writecomplete == 1))
				continue;
		}
		writecomplete = 0;
	}
	end = jiffies;
	timeuse = (end - start) * 1000 / HZ;
	printk(KERN_INFO "sending %d Kbytes to use take %d ms\n",
	       lpcunt * sizeof(testbuf) / 1024, timeuse);
}

static void diagUsbTest(int caseNo)
{

	init_waitqueue_head(&wcwaitQ);
	switch (caseNo) {
	case 1:
		/* diagUsbTest1(); */
		break;
	case 2:
		/* diagUsbTest2(); */
		break;
	case 3:
		diagUsbTest3();
		break;
	}
}
#endif

/*
 * gs_ioctl
 */
static int gs_ioctl(struct tty_struct *tty, struct file *file, unsigned int cmd,
		    unsigned long arg)
{
	struct gs_port *port = tty->driver_data;
	int ret = 0;

	if (port == NULL) {
		printk(KERN_ERR "gs_ioctl: NULL port pointer\n");
		return -EIO;
	}

	/* handle ioctls */
	if (gs_marvell_diag_ioctl)
		ret = gs_marvell_diag_ioctl(cmd, arg);

	return ret;
}

static const struct tty_operations gs_marvell_diag_tty_ops = {
	.open = gs_open,
	.close = gs_close,
	.write = gs_write,
	.put_char = gs_put_char,
	.flush_chars = gs_flush_chars,
	.write_room = gs_write_room,
	.chars_in_buffer = gs_chars_in_buffer,
	.ioctl = gs_ioctl,
	.unthrottle = gs_unthrottle,
	.break_ctl = gs_break_ctl,
};

int __init marvell_diag_gserial_setup(struct usb_gadget *g, unsigned count)
{

#define GS_MARVELL_DIAG_MAJOR		126
#define GS_MARVELL_DIAG_MINOR_START	16

	unsigned i;
	struct usb_cdc_line_coding coding;
	int status;

	if (count == 0 || count > N_PORTS)
		return -EINVAL;

	gs_tty_driver = alloc_tty_driver(count);
	if (!gs_tty_driver)
		return -ENOMEM;

	gs_tty_driver->owner = THIS_MODULE;
	gs_tty_driver->driver_name = "g_serial";
	gs_tty_driver->name = "ttydiag";
	/* uses dynamically assigned dev_t values */

	gs_tty_driver->type = TTY_DRIVER_TYPE_SERIAL;
	gs_tty_driver->subtype = SERIAL_TYPE_NORMAL;
	gs_tty_driver->flags = TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV;
	gs_tty_driver->init_termios = tty_std_termios;

	/* 9600-8-N-1 ... matches defaults expected by "usbser.sys" on
	 * MS-Windows.  Otherwise, most of these flags shouldn't affect
	 * anything unless we were to actually hook up to a serial line.
	 */
	gs_tty_driver->init_termios.c_cflag =
	    B9600 | CS8 | CREAD | HUPCL | CLOCAL;
	gs_tty_driver->init_termios.c_ispeed = 9600;
	gs_tty_driver->init_termios.c_ospeed = 9600;

	gs_tty_driver->major = GS_MARVELL_DIAG_MAJOR;
	gs_tty_driver->minor_start = GS_MARVELL_DIAG_MINOR_START;

	coding.dwDTERate = cpu_to_le32(9600);
	coding.bCharFormat = 8;
	coding.bParityType = USB_CDC_NO_PARITY;
	coding.bDataBits = USB_CDC_1_STOP_BITS;

	tty_set_operations(gs_tty_driver, &gs_marvell_diag_tty_ops);

	/* make devices be openable */
	for (i = 0; i < count; i++) {
		mutex_init(&ports[i].lock);
		status = gs_marvell_diag_port_alloc(i, &coding);
		if (status) {
			count = i;
			goto fail;
		}
	}
	n_ports = count;

	/* export the driver ... */
	status = tty_register_driver(gs_tty_driver);
	if (status) {
		pr_err("%s: cannot register, err %d\n", __func__, status);
		goto fail;
	}

	/* ... and sysfs class devices, so mdev/udev make /dev/ttyGS* */
	for (i = 0; i < count; i++) {
		struct device *tty_dev;

		tty_dev = tty_register_device(gs_tty_driver, i, &g->dev);
		if (IS_ERR(tty_dev))
			pr_warning("%s: no classdev for port %d, err %ld\n",
				   __func__, i, PTR_ERR(tty_dev));
	}

	pr_debug("%s: registered %d ttyGS* device%s\n", __func__,
		 count, (count == 1) ? "" : "s");
	return status;
fail:
	while (count--)
		kfree(ports[count].port);
	put_tty_driver(gs_tty_driver);
	gs_tty_driver = NULL;
	return status;
}

int marvell_diag_function_bind_config(struct usb_configuration *c)
{
	int ret = gser_bind_config(c, 0);
	if (ret == 0)
		marvell_diag_gserial_setup(c->cdev->gadget, 1);
	return ret;
}

static struct android_usb_function marvell_diag_function = {
	.name = "diag",
	.bind_config = marvell_diag_function_bind_config,
};

static int __init init(void)
{
	printk(KERN_INFO "f_marvell_diag init\n");
	android_register_function(&marvell_diag_function);
	return 0;
}

module_init(init);

