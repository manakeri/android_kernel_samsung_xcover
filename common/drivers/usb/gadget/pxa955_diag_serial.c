
/*
 * pxa955_diag_serial.c - utilities for USB gadget "serial port"/TTY support
 *
 * Copyright (C) 2003 Al Borchers (alborchers@steinerpoint.com)
 * Copyright (C) 2008 David Brownell
 * Copyright (C) 2008 by Nokia Corporation
 * Copyright (C) 2010 by Marvell Corporation
 *
 * This code also borrows from usbserial.c, which is
 * Copyright (C) 1999 - 2002 Greg Kroah-Hartman (greg@kroah.com)
 * Copyright (C) 2000 Peter Berger (pberger@brimson.com)
 * Copyright (C) 2000 Al Borchers (alborchers@steinerpoint.com)
 *
 * This software is distributed under the terms of the GNU General
 * Public License ("GPL") as published by the Free Software Foundation,
 * either version 2 of that License or (at your option) any later version.
 */

/* #define VERBOSE_DEBUG */

#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/sched.h>

#include "pxa955_diag_serial.h"


/*
 * This component encapsulates the TTY layer glue needed to provide basic
 * "serial port" functionality through the USB gadget stack.  Each such
 * port is exposed through a /dev/ttyDIAG* node.
 *
 * After initialization (diag_serial_setup), these TTY port devices stay
 * available until they are removed (diag_serial_cleanup).  Each one may be
 * connected to a USB function (diag_serial_connect), or disconnected (with
 * diag_serial_disconnect) when the USB host issues a config change event.
 * Data can only flow when the port is connected to the host.
 */

#define PREFIX	"ttyDIAG"

/*
 * diag_serial is the lifecycle interface, used by USB functions
 * diag_port is the I/O nexus, used by the tty driver
 * tty_struct links to the tty/filesystem framework
 *
 * diag_serial <---> diag_port ... links will be null when the USB link is
 * inactive; managed by diag_serial_{connect,disconnect}().  each diag_serial
 * instance can wrap its own USB control protocol.
 *	diag_serial->ioport == usb_ep->driver_data ... diag_port
 *	diag_port->port_usb ... diag_serial
 *
 * diag_port <---> tty_struct ... links will be null when the TTY file
 * isn't opened; managed by diag_open()/diag_close()
 *	diag_serial->port_tty ... tty_struct
 *	tty_struct->driver_data ... diag_serial
 */

/* RX and TX queues can buffer QUEUE_SIZE packets before they hit the
 * next layer of buffering.  For TX that's a circular buffer; for RX
 * consider it a NOP.  A third layer is provided by the TTY code.
 */
#define QUEUE_SIZE		8


#define MTXBUF 0x400
#define MAX_REQ_SIZE 512

static int windex, rindex;
static char *tx_buffers[MTXBUF];
static int tx_buffers_len[MTXBUF];
static int gs_kmallocs;
static int gs_kfrees;


static int txbuf_add(char *buf, int len)
{
	char *kbuf;

	if (((windex+1) & (MTXBUF-1)) != rindex) {

		kbuf =  kmalloc(len, GFP_ATOMIC);
		if (kbuf) {
			memcpy(kbuf, buf, len);
			tx_buffers[windex] = kbuf;
			tx_buffers_len[windex] = len;
			windex++;
			windex &= (MTXBUF-1);
			gs_kmallocs++;
			return 1;
		}
	}

	return 0;
}

static int txbuf_get(char *buf, int *len, int bufsz)
{
	while (rindex != windex) {
		if (tx_buffers[rindex] == NULL)
			return 0;

		if (bufsz < tx_buffers_len[rindex])
			return *len;

		memcpy(buf+(*len), tx_buffers[rindex], tx_buffers_len[rindex]);
		*len += tx_buffers_len[rindex];
		bufsz -= tx_buffers_len[rindex];
		kfree(tx_buffers[rindex]);
		gs_kfrees++;
		tx_buffers[rindex] = NULL;
		rindex++;
		rindex &= (MTXBUF-1);
		break;
	}
	return *len;
}

static int txbuf_room(void)
{
	return ((MTXBUF + rindex - windex - 1) & (MTXBUF-1)) * MAX_REQ_SIZE;
}


static int txbuf_has_data(void)
{

	return (int) (windex != rindex);
}

static int txbuf_data_count(void)
{
	int t = rindex;
	int total = 0;

	while (t != windex) {
		total += tx_buffers_len[t];
		t++;
		t &= (MTXBUF-1);
	}
	return total;
}


static void txbuf_free(void)
{
	while (rindex != windex) {
		if (tx_buffers[rindex] == NULL) {
			if (printk_ratelimit())
				printk(KERN_ERR "txbuf_free NULL pointer\n");
		} else {
			kfree(tx_buffers[rindex]);
			rindex++;
			rindex &= (MTXBUF-1);
		}
	}

}

/*
 * The port structure holds info for each port, one for each minor number
 * (and thus for each /dev/ node).
 */

struct diag_port {
	/* guard port_* access */
	spinlock_t				port_lock;

	struct diag_serial			*port_usb;
	struct tty_struct			*port_tty;

	unsigned				open_count;
	/* open/close in progress */
	bool					openclose;
	u8					port_num;

	/* wait for last close */
	wait_queue_head_t			close_wait;

	struct list_head			read_pool;
	struct list_head			read_queue;
	unsigned				n_read;
	struct tasklet_struct			push;

	struct list_head			write_pool;
	/* wait while writes drain */
	wait_queue_head_t			drain_wait;
};

/* increase N_PORTS if you need more */
#define N_PORTS				1
static struct portmaster {
	struct mutex		lock;		/* protect open/close */
	struct diag_port	*port;
} ports[N_PORTS];
static unsigned	n_ports;

#define DIAG_CLOSE_TIMEOUT	15		/* seconds */

#ifdef VERBOSE_DEBUG
#define pr_vdebug(fmt, arg...) \
	pr_debug(fmt, ##arg)
#else
#define pr_vdebug(fmt, arg...) \
	({ if (0) pr_debug(fmt, ##arg); })
#endif

/*---------------------------------------------------------------------*/

/* I/O glue between TTY (upper) and USB function (lower) driver layers */

/***********************************************************************
* Function: diag_alloc_req
************************************************************************
* Description:  Allocate a usb_request and its buffer.
*
* Parameters:   ep            - assosiated usb end point
*               len           - request length
*               kmalloc_flags - flags to use for memory allocation
*
* Return value: pointer to the usb_request or NULL if there is an error
*
* Notes:
*
***********************************************************************/
struct usb_request *
diag_alloc_req(struct usb_ep *ep, unsigned len, gfp_t kmalloc_flags)
{
	struct usb_request *req;

	req = usb_ep_alloc_request(ep, kmalloc_flags);

	if (req != NULL) {
		req->length = len;
		req->buf = kmalloc(len, kmalloc_flags);
		if (req->buf == NULL) {
			usb_ep_free_request(ep, req);
			return NULL;
		}
	}

	return req;
}

/***********************************************************************
* Function: diag_free_req
************************************************************************
* Description:  Free a usb_request and its buffer.
*
* Parameters:   ep      - assosiated usb end point
*               req     - pointer to usb request
*
* Return value: None
*
* Notes:
*
***********************************************************************/
void diag_free_req(struct usb_ep *ep, struct usb_request *req)
{
	kfree(req->buf);
	usb_ep_free_request(ep, req);
}

/***********************************************************************
* Function: diag_start_tx
************************************************************************
* Description:  This function finds available write requests, calls
*               diag_send_packet to fill these packets with data, and
*               continues until either there are no more write requests
*		 available or no more data to send.  This function is
*		 run whenever data arrives or write requests are
*		 available.
*
* Parameters:   port    - pointer to diag_port
*
* Return value: Operation status
*
* Notes:
*		 Context: caller owns port_lock; port_usb is non-null.
***********************************************************************/
static int diag_start_tx(struct diag_port *port)
/*
__releases(&port->port_lock)
__acquires(&port->port_lock)
*/
{
	struct list_head	*pool = &port->write_pool;
	struct usb_ep		*in;
	int			status = 0;
	bool			do_tty_wake = false;

	while (!list_empty(pool)) {
		struct usb_request	*req;
		int			len;

		/* abort immediately after disconnect */
		if (!port->port_usb)
			break;

		in = port->port_usb->in;

		req = list_entry(pool->next, struct usb_request, list);
		req->length = 0;
		if (!txbuf_get(req->buf, &req->length, MAX_REQ_SIZE)) {

			wake_up_interruptible(&port->drain_wait);
			break;
		}
		do_tty_wake = true;

		list_del(&req->list);

		req->zero = 0;

		pr_vdebug(PREFIX "%d: tx len=%d, 0x%02x 0x%02x 0x%02x ...\n",
				port->port_num, len, *((u8 *)req->buf),
				*((u8 *)req->buf+1), *((u8 *)req->buf+2));

		/* Drop lock while we call out of driver; completions
		 * could be issued while we do so.  Disconnection may
		 * happen too; maybe immediately before we queue this!
		 *
		 * NOTE that we may keep sending data for a while after
		 * the TTY closed (dev->ioport->port_tty is NULL).
		 */
		/* spin_unlock(&port->port_lock); */
		status = usb_ep_queue(in, req, GFP_ATOMIC);
		/* spin_lock(&port->port_lock); */

		if (status) {
			pr_debug("%s: %s %s err %d\n",
					__func__, "queue", in->name, status);
			list_add(&req->list, pool);
			break;
		}
	}

	if (do_tty_wake && port->port_tty)
		tty_wakeup(port->port_tty);
	return status;
}

/***********************************************************************
* Function: diag_start_rx                                              *
************************************************************************
* Description:  Receive usb request entries
*
* Parameters:   port    - pointer to diag_port
*
* Return value: Number of entries received
*
* Notes:
*		    Context: caller owns port_lock, and port_usb is set
***********************************************************************/
static unsigned diag_start_rx(struct diag_port *port)
/*
__releases(&port->port_lock)
__acquires(&port->port_lock)
*/
{
	struct list_head	*pool = &port->read_pool;
	struct usb_ep		*out = NULL;
	unsigned		started = 0;

	if(!port || !port->port_usb)
		return -1;

        out = port->port_usb->out;	

	while (!list_empty(pool)) {
		struct usb_request	*req;
		int			status;
		struct tty_struct	*tty;

		/* no more rx if closed */
		tty = port->port_tty;
		if (!tty)
			break;

		req = list_entry(pool->next, struct usb_request, list);
		list_del(&req->list);
		req->length = out->maxpacket;

		/* drop lock while we call out; the controller driver
		 * may need to call us back (e.g. for disconnect)
		 */
		/* spin_unlock(&port->port_lock); */
		status = usb_ep_queue(out, req, GFP_ATOMIC);
		/* spin_lock(&port->port_lock); */

		if (status) {
			pr_debug("%s: %s %s err %d\n",
					__func__, "queue", out->name, status);
			list_add(&req->list, pool);
			break;
		}
		started++;

		/* abort immediately after disconnect */
		if (!port->port_usb)
			break;
	}
	return started;
}

/***********************************************************************
* Function: diag_rx_push
************************************************************************
* Description: RX tasklet takes data out of the RX queue and hands
*		it up to the TTY ayer until it refuses to take any
*		more data (or is throttled back). Then it issues reads
*		for any further data.
*
* Parameters:   port    - pointer to diag_port
*
* Return value: None
*
* Notes:
*		If the RX queue becomes full enough that no usb_request
*		is queued, the OUT endpoint may begin NAKing as soon as
*		its FIFO fills up. So QUEUE_SIZE packets plus however
*		many the FIFO holds (usually two) can be buffered
*		before the TTY layer's buffers (currently 64 KB).
***********************************************************************/
static void diag_rx_push(unsigned long _port)
{
	struct diag_port		*port = (void *)_port;
	struct tty_struct		*tty;
	struct list_head		*queue = &port->read_queue;
	bool					disconnect = false;
	bool					do_push = false;

	/* hand any queued data to the tty */
	spin_lock_irq(&port->port_lock);
	tty = port->port_tty;
	while (!list_empty(queue)) {
		struct usb_request	*req;

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
			char		*packet = req->buf;
			unsigned	size = req->actual;
			unsigned	n;
			int		count;

			/* we may have pushed part of this packet already... */
			n = port->n_read;
			if (n) {
				packet += n;
				size -= n;
			}

			count = tty_insert_flip_string(tty, packet, size);
			if (count)
				do_push = true;
			if (count != size) {
				/* stop pushing; TTY layer can't handle more */
				port->n_read += count;
				pr_vdebug(PREFIX "%d: rx block %d/%d\n",
						port->port_num,
						count, req->actual);
				break;
			}
			port->n_read = 0;
		}
recycle:
		list_move(&req->list, &port->read_pool);
	}

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
		diag_start_rx(port);

	spin_unlock_irq(&port->port_lock);
}

/***********************************************************************
* Function: diag_read_complete
************************************************************************
* Description: Callback function upon read complete. schedules tasklet
*		to push the data to tty layer
*
* Parameters:  ep      - assosiated usb endpoint
*		req     - assosiated usb request
*
* Return value: None
*
* Notes:
*
***********************************************************************/
static void diag_read_complete(struct usb_ep *ep,
		struct usb_request *req)
{
	struct diag_port	*port = ep->driver_data;

	/* Queue all received data until the tty layer is ready for it. */
	spin_lock(&port->port_lock);
	list_add_tail(&req->list, &port->read_queue);
	tasklet_schedule(&port->push);
	spin_unlock(&port->port_lock);
}

/***********************************************************************
* Function: diag_write_complete
************************************************************************
* Description: Callback function upon write complete. starts requests
*		transmission
*
* Parameters:   ep      - assosiated usb endpoint
*		 req     - assosiated usb request
*
* Return value: None
*
* Notes:
*
***********************************************************************/
static void diag_write_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct diag_port	*port = ep->driver_data;

	spin_lock(&port->port_lock);
	list_add(&req->list, &port->write_pool);

	switch (req->status) {
	default:
		/* presumably a transient fault */
		pr_warning("%s: unexpected %s status %d\n",
				__func__, ep->name, req->status);
		/* FALL THROUGH */
	case 0:
		/* normal completion */
		diag_start_tx(port);
		break;

	case -ESHUTDOWN:
		/* disconnect */
		pr_vdebug("%s: %s shutdown\n", __func__, ep->name);
		break;
	}

	spin_unlock(&port->port_lock);
}

/***********************************************************************
* Function: diag_free_requests
************************************************************************
* Description:  Free all requests associated to specific usb end point
*
* Parameters:   ep      - assosiated usb endpoint
*		 head    - assosiated list head
*
* Return value: None
*
* Notes:
*
***********************************************************************/
static void diag_free_requests(struct usb_ep *ep,
		struct list_head *head)
{
	struct usb_request	*req;

	while (!list_empty(head)) {
		req = list_entry(head->next, struct usb_request, list);
		list_del(&req->list);
		diag_free_req(ep, req);
	}
}

/***********************************************************************
* Function: diag_alloc_requests
************************************************************************
* Description:  Allocated QUEUE_SIZE usb requests placeholders
*
* Parameters:   ep      - assosiated usb endpoint
*		 head    - assosiated list head
*		 fn      - assosiated complete callback function
*
* Return value: Operation result
*
* Notes:
*		 if we cannot allocate QUEUE_SIZE transfers, then
*		 allocates as much as it can.
***********************************************************************/
static int diag_alloc_requests(struct usb_ep *ep, struct list_head *head,
		void (*fn)(struct usb_ep *, struct usb_request *))
{
	int			i;
	struct usb_request	*req;

	/* Pre-allocate up to QUEUE_SIZE transfers, but if we can't
	 * do quite that many this time, don't fail ... we just won't
	 * be as speedy as we might otherwise be.
	 */
	for (i = 0; i < QUEUE_SIZE; i++) {
		req = diag_alloc_req(ep, MAX_REQ_SIZE, GFP_ATOMIC);
		if (!req)
			return list_empty(head) ? -ENOMEM : 0;
		req->complete = fn;
		list_add_tail(&req->list, head);
	}
	return 0;
}

/***********************************************************************
* Function: diag_start_io
************************************************************************
* Description:  starts USB I/O streams
*
* Parameters:   port    - pointer to diag_port
*
* Return value: Operation result
*
* Notes:
*		@dev: encapsulates endpoints to use
*		Context: holding port_lock. port_tty and port_usb
*		are non-null.
*		We only start I/O when something is connected to both
*		sides of this port.  If nothing is listening on the
*		host side, we may be pointlessly filling up our TX
*		buffers and FIFO.
*
***********************************************************************/
static int diag_start_io(struct diag_port *port)
{
	struct list_head	*head = &port->read_pool;
	struct usb_ep		*ep = port->port_usb->out;
	int			status;
	unsigned		started;

	/* Allocate RX and TX I/O buffers.  We can't easily do this much
	 * earlier (with GFP_KERNEL) because the requests are coupled to
	 * endpoints, as are the packet sizes we'll be using.  Different
	 * configurations may use different endpoints with a given port;
	 * and high speed vs full speed changes packet sizes too.
	 */
	status = diag_alloc_requests(ep, head, diag_read_complete);
	if (status)
		return status;

	status = diag_alloc_requests(port->port_usb->in, &port->write_pool,
			diag_write_complete);
	if (status) {
		diag_free_requests(ep, head);
		return status;
	}

	/* queue read requests */
	port->n_read = 0;
	started = diag_start_rx(port);

	/* unblock any pending writes into our circular buffer */
	if (started) {
		tty_wakeup(port->port_tty);
	} else {
		diag_free_requests(ep, head);
		diag_free_requests(port->port_usb->in, &port->write_pool);
		status = -EIO;
	}

	return status;
}

/*-------------------------------------------------------------------------*/

/* TTY Driver */

/***********************************************************************
* Function: diag_open
************************************************************************
* Description:  sets up the link between a diag_port and its associated
*		TTY. That link is broken *only* by diag_close(), and
*		all driver methods know that.
*
* Parameters:   tty     - assosiated tty_struct to use in diag_port
*		 file    - pointer to file
*
* Return value: Operation result
*
* Notes:
*
***********************************************************************/
static int diag_open(struct tty_struct *tty, struct file *file)
{
	int		port_num = tty->index;
	struct diag_port	*port;
	int		status;

	if (port_num < 0 || port_num >= n_ports)
		return -ENXIO;

	do {
		mutex_lock(&ports[port_num].lock);
		port = ports[port_num].port;
		if (!port)
			status = -ENODEV;
		else {
			spin_lock_irq(&port->port_lock);

			/* already open?  Great. */
			if (port->open_count) {
				status = 0;
				port->open_count++;

			/* currently opening/closing? wait ... */
			} else if (port->openclose) {
				status = -EBUSY;

			/* ... else we do the work */
			} else {
				status = -EAGAIN;
				port->openclose = true;
			}
			spin_unlock_irq(&port->port_lock);
		}
		mutex_unlock(&ports[port_num].lock);

		switch (status) {
		default:
			/* fully handled */
			return status;
		case -EAGAIN:
			/* must do the work */
			break;
		case -EBUSY:
			/* wait for EAGAIN task to finish */
			msleep(1);
			/* REVISIT could have a waitchannel here, if
			 * concurrent open performance is important
			 */
			break;
		}
	} while (status != -EAGAIN);

	/* Do the "real open" */
	spin_lock_irq(&port->port_lock);

	/* allocate circular buffer on first open */

	/* REVISIT if REMOVED (ports[].port NULL), abort the open
	 * to let rmmod work faster (but this way isn't wrong).
	 */

	tty->driver_data = port;
	port->port_tty = tty;

	port->open_count = 1;
	port->openclose = false;

	/* low_latency must be kept unset, as setting this flag causing
	 * spin_lock_irqsave - which is forbidden in irq context.
	 */
	tty->low_latency = 0;

	/* if connected, start the I/O stream */
	if (port->port_usb) {
		struct diag_serial	*gser = port->port_usb;

		pr_debug("diag_open: start ttyDIAG%d\n", port->port_num);
		diag_start_io(port);

		if (gser->connect)
			gser->connect(gser);
	}

	pr_debug("diag_open: ttyDIAG%d (%p,%p)\n", port->port_num, tty, file);

	status = 0;

	spin_unlock_irq(&port->port_lock);
	return status;
}

/***********************************************************************
* Function: diag_writes_finished
************************************************************************
* Description:  check if writes were finished
*
* Parameters:   p       - pointer to diag_port
*
* Return value: True if no write request pending, 0 otherwise
*
* Notes:
*
***********************************************************************/
static int diag_writes_finished(struct diag_port *p)
{
	int cond;

	/* return true on disconnect or empty buffer */
	spin_lock_irq(&p->port_lock);
	cond = (p->port_usb == NULL) || !txbuf_has_data();
	spin_unlock_irq(&p->port_lock);

	return cond;
}

/***********************************************************************
* Function: diag_close
************************************************************************
* Description:  Disconnect the link between diag_port and its
*		 associated TTY driver
*
* Parameters:   tty     - assosiated tty_struct
*		 file    - pointer to file
*
* Return value: None
*
* Notes:
*
***********************************************************************/
static void diag_close(struct tty_struct *tty, struct file *file)
{
	struct diag_port *port = tty->driver_data;
	struct diag_serial	*gser;

	spin_lock_irq(&port->port_lock);

	if (port->open_count != 1) {
		if (port->open_count == 0)
			WARN_ON(1);
		else
			--port->open_count;
		goto exit;
	}

	pr_debug("diag_close: ttyDIAG%d (%p,%p) ...\n",
		port->port_num, tty, file);

	/* mark port as closing but in use; we can drop port lock
	 * and sleep if necessary
	 */
	port->openclose = true;
	port->open_count = 0;

	gser = port->port_usb;
	if (gser && gser->disconnect)
		gser->disconnect(gser);

	/* wait for circular write buffer to drain, disconnect, or at
	 * most DIAG_CLOSE_TIMEOUT seconds; then discard the rest
	 */
	if (txbuf_has_data() && gser) {
		spin_unlock_irq(&port->port_lock);
		wait_event_interruptible_timeout(port->drain_wait,
					diag_writes_finished(port),
					DIAG_CLOSE_TIMEOUT * HZ);
		spin_lock_irq(&port->port_lock);
		gser = port->port_usb;
	}

	txbuf_free();

	tty->driver_data = NULL;
	port->port_tty = NULL;

	port->openclose = false;

	pr_debug("diag_close: ttyDIAG%d (%p,%p) done!\n",
			port->port_num, tty, file);

	wake_up_interruptible(&port->close_wait);
exit:
	spin_unlock_irq(&port->port_lock);
}

/***********************************************************************
* Function: diag_write
************************************************************************
* Description:  Receives data buffer from assosiated TTY driver to
*		 circulat buffer
*
* Parameters:   tty     - assosiated tty_struct
*		 buf     - pointer to data buffer
*		 count   - size in bytes of the source buffer
*
* Return value: Actual received bytes
*
* Notes:
*
***********************************************************************/
static int diag_write(struct tty_struct *tty,
	const unsigned char *buf, int count)
{
	struct diag_port	*port = tty->driver_data;
	unsigned long	flags;
	int		status;

	pr_vdebug("diag_write: ttyDIAG%d (%p) writing %d bytes\n",
			port->port_num, tty, count);


	if (!count)
		return 0;

	spin_lock_irqsave(&port->port_lock, flags);

	if (txbuf_room() >= count) {
		int temp = count;
		int i = 0;
		while (temp > 0) {
			txbuf_add((char *)buf+(i*MAX_REQ_SIZE),
				temp > MAX_REQ_SIZE ? MAX_REQ_SIZE : temp);
			temp -= MAX_REQ_SIZE;
			i++;

		}
	} else

		count = 0;

	/* treat count == 0 as flush_chars() */
	if (port->port_usb)
		status = diag_start_tx(port);
	spin_unlock_irqrestore(&port->port_lock, flags);

	return count;
}

/***********************************************************************
* Function: diag_put_char
************************************************************************
* Description:  Copies one byte from TTY driver into circular buffer
*
* Parameters:   tty     - assosiated tty_struct
*		 ch      - pointer to data buffer
*
* Return value: Actual copied bytes	(1 if succeed, 0 otherwise)
*
* Notes:
*
***********************************************************************/
static int diag_put_char(struct tty_struct *tty, unsigned char ch)
{
	struct diag_port	*port = tty->driver_data;
	unsigned long	flags;
	int		status = 0;

	pr_vdebug("diag_put_char: (%d,%p) char=0x%x, called from %p\n",
		port->port_num, tty, ch, __builtin_return_address(0));

	spin_lock_irqsave(&port->port_lock, flags);
	if (port->port_usb) {
		status = txbuf_add((char *)&ch, 1);
		diag_start_tx(port);
	}
	spin_unlock_irqrestore(&port->port_lock, flags);

	return status;
}

/***********************************************************************
* Function: diag_flush_chars
************************************************************************
* Description:  Flush all pending tx requests
*
* Parameters:   tty     - assosiated tty_struct
*
* Return value: None
*
* Notes:
*
***********************************************************************/
static void diag_flush_chars(struct tty_struct *tty)
{
	struct diag_port	*port = tty->driver_data;
	unsigned long	flags;

	pr_vdebug("diag_flush_chars: (%d,%p)\n", port->port_num, tty);

	spin_lock_irqsave(&port->port_lock, flags);
	if (port->port_usb)
		diag_start_tx(port);
	spin_unlock_irqrestore(&port->port_lock, flags);
}

/***********************************************************************
* Function: diag_write_room
************************************************************************
* Description:  checks for buffer available space (for write)
*
* Parameters:   tty     - assosiated tty_struct
*
* Return value: size in bytes of free space (in circular buffer)
*
* Notes:
*
***********************************************************************/
static int diag_write_room(struct tty_struct *tty)
{
	struct diag_port	*port = tty->driver_data;
	unsigned long	flags;
	int		room = 0;

	spin_lock_irqsave(&port->port_lock, flags);
	if (port->port_usb)
		room = txbuf_room();
	spin_unlock_irqrestore(&port->port_lock, flags);

	pr_vdebug("diag_write_room: (%d,%p) room=%d\n",
		port->port_num, tty, room);

	return room;
}

/***********************************************************************
* Function: diag_chars_in_buffer
************************************************************************
* Description:  checks for pending data in circular buffer
*
* Parameters:   tty     - assosiated tty_struct
*
* Return value: size of pending bytes (in circular buffer)
*
* Notes:
*
***********************************************************************/
static int diag_chars_in_buffer(struct tty_struct *tty)
{
	struct diag_port	*port = tty->driver_data;
	unsigned long	flags;
	int		chars = 0;

	spin_lock_irqsave(&port->port_lock, flags);
	chars = txbuf_data_count();
	spin_unlock_irqrestore(&port->port_lock, flags);

	pr_vdebug("diag_chars_in_buffer: (%d,%p) chars=%d\n",
		port->port_num, tty, chars);

	return chars;
}

/***********************************************************************
* Function: diag_unthrottle
************************************************************************
* Description:  undo side effects of setting TTY_THROTTLED
*		 starts rx_push tasklet
*
* Parameters:   tty     - assosiated tty_struct
*
* Return value: None
*
* Notes:
*
***********************************************************************/
static void diag_unthrottle(struct tty_struct *tty)
{
	struct diag_port		*port = tty->driver_data;
	unsigned long		flags;

	spin_lock_irqsave(&port->port_lock, flags);
	if (port->port_usb) {
		/* Kickstart read queue processing.  We don't do xon/xoff,
		 * rts/cts, or other handshaking with the host, but if the
		 * read queue backs up enough we'll be NAKing OUT packets.
		 */
		tasklet_schedule(&port->push);
		pr_vdebug(PREFIX "%d: unthrottle\n", port->port_num);
	}
	spin_unlock_irqrestore(&port->port_lock, flags);
}

/***********************************************************************
* Function: diag_break_ctl
************************************************************************
* Description:  Sends break to assosiate diag_serial
*
* Parameters:   tty     - assosiated tty_struct
*		 duration- duration time
*
* Return value: Operation result
*
* Notes:
*
***********************************************************************/
static int diag_break_ctl(struct tty_struct *tty, int duration)
{
	struct diag_port	*port = tty->driver_data;
	int			status = 0;
	struct diag_serial	*gser;

	pr_vdebug("diag_break_ctl: ttyDIAG%d, send break (%d)\n",\
		port->port_num, duration);

	spin_lock_irq(&port->port_lock);
	gser = port->port_usb;
	if (gser && gser->send_break)
		status = gser->send_break(gser, duration);
	spin_unlock_irq(&port->port_lock);

	return status;
}

static const struct tty_operations diag_tty_ops = {
	.open =			diag_open,
	.close =		diag_close,
	.write =		diag_write,
	.put_char =		diag_put_char,
	.flush_chars =		diag_flush_chars,
	.write_room =		diag_write_room,
	.chars_in_buffer =	diag_chars_in_buffer,
	.unthrottle =		diag_unthrottle,
	.break_ctl =		diag_break_ctl,
};

/*--------------------------------------------------------------------*/

static struct tty_driver *diag_tty_driver;

/***********************************************************************
* Function: diag_port_alloc
************************************************************************
* Description:  Allocates diag_port
*
* Parameters:   port_num   - port number to allocate
*
* Return value: Operation result
*
* Notes:
*
***********************************************************************/
static int __init
diag_port_alloc(unsigned port_num)
{
	struct diag_port	*port;

	port = kzalloc(sizeof(struct diag_port), GFP_KERNEL);
	if (port == NULL)
		return -ENOMEM;

	spin_lock_init(&port->port_lock);
	init_waitqueue_head(&port->close_wait);
	init_waitqueue_head(&port->drain_wait);

	tasklet_init(&port->push, diag_rx_push, (unsigned long) port);

	INIT_LIST_HEAD(&port->read_pool);
	INIT_LIST_HEAD(&port->read_queue);
	INIT_LIST_HEAD(&port->write_pool);

	port->port_num = port_num;

	ports[port_num].port = port;

	return 0;
}

/***********************************************************************
* Function: diag_serial_setup
************************************************************************
* Description:  Initialize TTY driver for one or more ports
*
* Parameters:   g          - gadget to associate with these ports
*		 count      - how many ports to support
*
* Return value: Operation result
*
* Notes:
*
***********************************************************************/
int __init diag_serial_setup(struct usb_gadget *g, unsigned count)
{
	unsigned	i;
	int		status;

	if (count == 0 || count > N_PORTS)
		return -EINVAL;

	diag_tty_driver = alloc_tty_driver(count);
	if (!diag_tty_driver)
		return -ENOMEM;

	diag_tty_driver->owner = THIS_MODULE;
	diag_tty_driver->driver_name = "g_serial";
	diag_tty_driver->name = PREFIX;
	/* uses dynamically assigned dev_t values */

	diag_tty_driver->type = TTY_DRIVER_TYPE_SERIAL;
	diag_tty_driver->subtype = SERIAL_TYPE_NORMAL;
	diag_tty_driver->flags = TTY_DRIVER_REAL_RAW |
				 TTY_DRIVER_DYNAMIC_DEV;
	diag_tty_driver->init_termios = tty_std_termios;

	/* 9600-8-N-1 ... matches defaults expected by "usbser.sys" on
	 * MS-Windows.  Otherwise, most of these flags shouldn't affect
	 * anything unless we were to actually hook up to a serial line.
	 */
	diag_tty_driver->init_termios.c_cflag =
			B9600 | CS8 | CREAD | HUPCL | CLOCAL;
	diag_tty_driver->init_termios.c_ispeed = 9600;
	diag_tty_driver->init_termios.c_ospeed = 9600;

	tty_set_operations(diag_tty_driver, &diag_tty_ops);

	/* make devices be openable */
	for (i = 0; i < count; i++) {
		mutex_init(&ports[i].lock);
		status = diag_port_alloc(i);
		if (status) {
			count = i;
			goto fail;
		}
	}
	n_ports = count;

	/* export the driver ... */
	status = tty_register_driver(diag_tty_driver);
	if (status) {
		pr_err("%s: cannot register, err %d\n",
				__func__, status);
		goto fail;
	}

	/* ... and sysfs class devices, so mdev/udev make /dev/ttyDIAG* */
	for (i = 0; i < count; i++) {
		struct device	*tty_dev;

		tty_dev = tty_register_device(diag_tty_driver, i, &g->dev);
		if (IS_ERR(tty_dev))
			pr_warning("%s: no classdev for port %d, err %ld\n",
				__func__, i, PTR_ERR(tty_dev));
	}

	pr_debug("%s: registered %d ttyDIAG* device%s\n", __func__,
			count, (count == 1) ? "" : "s");

	return status;
fail:
	while (count--)
		kfree(ports[count].port);
	put_tty_driver(diag_tty_driver);
	diag_tty_driver = NULL;
	return status;
}

/***********************************************************************
* Function: diag_closed
************************************************************************
* Description:  Checks if the driver is closed
*
* Parameters:   port       - pointer to diag_port
*
* Return value: True if port is closed, 0 otherwise
*
* Notes:
*
***********************************************************************/
static int diag_closed(struct diag_port *port)
{
	int cond;

	spin_lock_irq(&port->port_lock);
	cond = (port->open_count == 0) && !port->openclose;
	spin_unlock_irq(&port->port_lock);
	return cond;
}

/***********************************************************************
* Function: diag_serial_cleanup
************************************************************************
* Description:  Remove TTY-over-USB driver and devices
*
* Parameters:   port       - pointer to diag_port
*
* Return value: None
*
* Notes:
*		This is called to free all resources allocated by
*		diag_serial_setup(). Accordingly, it may need to wait
*		until some open /dev/ files have closed.
*
*		The caller must have issued diag_serial_disconnect()
*		for any ports that had previously been connected, so
*		that there is never any I/O pending when it's called.
***********************************************************************/
void diag_serial_cleanup(void)
{
	unsigned	i;
	struct diag_port	*port;

	if (!diag_tty_driver)
		return;

	/* start sysfs and /dev/ttyDIAG* node removal */
	for (i = 0; i < n_ports; i++)
		tty_unregister_device(diag_tty_driver, i);

	for (i = 0; i < n_ports; i++) {
		/* prevent new opens */
		mutex_lock(&ports[i].lock);
		port = ports[i].port;
		ports[i].port = NULL;
		mutex_unlock(&ports[i].lock);

		tasklet_kill(&port->push);

		/* wait for old opens to finish */
		wait_event(port->close_wait, diag_closed(port));

		WARN_ON(port->port_usb != NULL);

		kfree(port);
	}
	n_ports = 0;

	tty_unregister_driver(diag_tty_driver);
	diag_tty_driver = NULL;

	pr_debug("%s: cleaned up ttyDIAG* support\n", __func__);
}

/***********************************************************************
* Function: diag_serial_connect
************************************************************************
* Description:  Notify TTY I/O glue that USB link is active
*		 On success, ep->driver_data will be overwritten.
*
* Parameters:   gser       - the function, set up with endpoints and
*			      descriptors
*		 port_num   - which port is active
*
* Return value: Operation result
*
* Notes:
*		This is called activate endpoints and let the TTY
*		layer know that the connection is active ... not unlike
*		"carrier detect".  It won't necessarily start I/O
*		queues; unless the TTY is held open by any task, there
*		would be no point.  However, the endpoints will be
*		activated so the USB host can perform I/O, subject to
*		basic USB hardware flow control.
*
*		Caller needs to have set up the endpoints and USB
*		function in @dev before calling this, as well as the
*		appropriate (speed-specific) endpoint descriptors, and
*		also have set up the TTY driver by calling
*		diag_serial_setup().
*
***********************************************************************/
int diag_serial_connect(struct diag_serial *gser, u8 port_num)
{
	struct diag_port	*port;
	unsigned long	flags;
	int		status;

	if (!diag_tty_driver || port_num >= n_ports)
		return -ENXIO;

	/* we "know" diag_serial_cleanup() hasn't been called */
	port = ports[port_num].port;

	/* activate the endpoints */
	gser->in->zlp = 1;
	status = usb_ep_enable(gser->in, gser->in_desc);
	if (status < 0)
		return status;
	gser->in->driver_data = port;

	status = usb_ep_enable(gser->out, gser->out_desc);
	if (status < 0)
		goto fail_out;
	gser->out->driver_data = port;

	/* then tell the tty glue that I/O can work */
	spin_lock_irqsave(&port->port_lock, flags);
	gser->ioport = port;
	port->port_usb = gser;

	/* REVISIT if waiting on "carrier detect", signal. */

	/* if it's already open, start I/O ... and notify the serial
	 * protocol about open/close status (connect/disconnect).
	 */
	if (port->open_count) {
		pr_debug("diag_serial_connect: start ttyDIAG%d\n",
			port->port_num);
		diag_start_io(port);
		if (gser->connect)
			gser->connect(gser);
	} else {
		if (gser->disconnect)
			gser->disconnect(gser);
	}

	spin_unlock_irqrestore(&port->port_lock, flags);

	return status;

fail_out:
	usb_ep_disable(gser->in);
	gser->in->driver_data = NULL;
	return status;
}

/***********************************************************************
* Function: diag_serial_disconnect
************************************************************************
* Description:  Notify TTY I/O glue that USB link is inactive
*		 On success, ep->driver_data will be overwritten.
*
* Parameters:   gser       - the function, on which
*			      diag_serial_connect() was called
*
* Return value: Operation result
*
* Notes:
*		This is called to deactivate endpoints and let the TTY
*		layer know that the connection went inactive ... not
*		unlike "hangup".
*
*		On return, the state is as if diag_serial_connect()
*		had never been called. there is no active USB I/O on
*		these endpoints.
***********************************************************************/
void diag_serial_disconnect(struct diag_serial *gser)
{
	struct diag_port	*port = gser->ioport;
	unsigned long	flags;

	if (!port)
		return;

	/* tell the TTY glue not to do I/O here any more */
	spin_lock_irqsave(&port->port_lock, flags);

	port->port_usb = NULL;
	gser->ioport = NULL;
	if (port->open_count > 0 || port->openclose) {
		wake_up_interruptible(&port->drain_wait);
		if (port->port_tty)
			tty_hangup(port->port_tty);
	}
	spin_unlock_irqrestore(&port->port_lock, flags);

	/* disable endpoints, aborting down any active I/O */
	usb_ep_disable(gser->out);
	gser->out->driver_data = NULL;

	usb_ep_disable(gser->in);
	gser->in->driver_data = NULL;

	/* finally, free any unused/unusable I/O buffers */
	spin_lock_irqsave(&port->port_lock, flags);
	if (port->open_count == 0 && !port->openclose)
		txbuf_free();
	diag_free_requests(gser->out, &port->read_pool);
	diag_free_requests(gser->out, &port->read_queue);
	diag_free_requests(gser->in, &port->write_pool);
	spin_unlock_irqrestore(&port->port_lock, flags);
}
