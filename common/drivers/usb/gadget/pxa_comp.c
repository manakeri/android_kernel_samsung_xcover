//#define	DEBUG
//#define	VERBOSE

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/ioport.h>
#include <linux/types.h>
#include <linux/version.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/proc_fs.h>
#include <linux/mm.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/clk.h>

#include <asm/byteorder.h>
#include <asm/dma.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/system.h>
#include <asm/mach-types.h>
#include <asm/unaligned.h>
#include <asm/uaccess.h>
#include <linux/usb.h>
#include <linux/usb/ch9.h>
#include <linux/usb/cdc.h>
#include <linux/usb/gadget.h>
#include <linux/usb/otg.h>

#include <plat/pxausb_comp.h>

#ifdef CONFIG_USB_COMPOSITE
static int gadget_info_init(struct pxa3xx_comp *dev, struct usb_gadget *gadget,
			    struct usb_gadget_driver *driver);
static int gadget_get_device_desc(struct pxa3xx_comp *dev,
				  struct usb_gadget *gadget,
				  struct usb_gadget_driver *driver);
static void set_cdc_desc(struct pxa3xx_comp *dev);
static int gadget_info_uninit(struct pxa3xx_comp *dev,
			      struct usb_gadget *gadget,
			      struct usb_gadget_driver **slf_drver,
			      struct usb_gadget_driver *driver);
static int gadget_get_config_desc_hs(struct pxa3xx_comp *dev,
				     struct usb_gadget *gadget,
				     struct usb_gadget_driver *driver);
#endif

/*
 * get_extra_descriptor() finds a descriptor of specific type in the
 * extra field of the interface and endpoint descriptor structs.
 */
int get_extra_descriptor(char *buffer, unsigned size,
	unsigned char type, void **ptr)
{
	struct usb_descriptor_header *header;

	*ptr = buffer;
	while (size >= sizeof(struct usb_descriptor_header)) {
		header = (struct usb_descriptor_header *)buffer;

		if (header->bLength < 2) {
			DMSG("%s: descriptor, type %d length %d not found\n",
				__FUNCTION__,
				header->bDescriptorType,
				header->bLength);
			return -ENODATA;
		}

		if (header->bDescriptorType == type) {
			*ptr = header;
			return size;
		}

		buffer += header->bLength;
		size -= header->bLength;
	}
	return -ENODATA;
}

void get_fake_config(struct pxa3xx_comp *dev, struct usb_request *req, int spd)
{
#ifndef CONFIG_USB_COMPOSITE
	memcpy(dev->configs, req->buf, req->length);
#else
	if (((__u8 *)(req->buf))[1] == USB_DT_CONFIG) {
		if (spd == USB_SPEED_HIGH)
			memcpy(dev->active_gadget->config_desc_hs, req->buf,
			       req->length);
		else
			memcpy(dev->active_gadget->config_desc, req->buf,
			       req->length);
	} else if (((__u8 *)(req->buf))[1] == USB_DT_DEVICE)
		memcpy(dev->active_gadget->device_desc, req->buf, req->length);
#endif
}

void comp_val_init(struct pxa3xx_comp *dev)
{
#ifdef CONFIG_USB_COMPOSITE
	dev->configuration = 0;
	dev->interface = 0;
	dev->alternate = 0;
#endif
}

#ifdef CONFIG_USB_COMPOSITE
static struct pxa3xx_comp_ep *find_ep_intf(struct pxa3xx_comp_ep *head,
					   int assigned_interface)
{
	struct pxa3xx_comp_ep *p = head;
	while (p) {
		if (p->assigned_interface == assigned_interface)
			break;
		p = p->next;
	}
	return p;
}
#endif

struct pxa3xx_comp_ep *find_ep_num(struct pxa3xx_comp_ep *head, int num)
{
	struct pxa3xx_comp_ep *p = head;
	while (p) {
		if (p->log_ep_num == num)
			break;
		p = p->next;
	}
	return p;
}

static void delete_comp_ep_list(struct pxa3xx_comp_ep **head)
{
	struct pxa3xx_comp_ep *p = *head;
	struct pxa3xx_comp_ep *p_cur;
	while (p) {
		p_cur = p;
		p = p->next;
		kfree(p_cur);
		p_cur = NULL;
	}
	*head = NULL;
}

void comp_set_ep(struct pxa3xx_comp *dev, int ep_num, int config, int interface)
{
	struct pxa3xx_comp_ep *ep = kzalloc(sizeof(struct pxa3xx_comp_ep),
					    GFP_KERNEL);
	struct pxa3xx_comp_ep *p;
	if (!ep) {
		pr_err("%s: cannot alloc mem for ep!\n", __func__);
		return;
	}

	ep->config = config;
	ep->interface = interface;
	ep->log_ep_num = ep_num;
#ifdef CONFIG_USB_COMPOSITE
	ep->assigned_interface = dev->interface_count;
	ep->driver_info = dev->active_gadget;
#endif
	if (!dev->first_ep)
		dev->first_ep = ep;
	else {
		p = dev->first_ep;
		while (p->next) p = p->next;
		p->next = ep;
	}
}

/* After driver is bound, send a fake get configuration command to
 * gadget driver to get the configuration information */
static int gadget_get_config_desc(struct pxa3xx_comp *dev,
				  struct usb_gadget *gadget,
				  struct usb_gadget_driver *driver)
{
	struct usb_ctrlrequest  req;
	struct usb_config_descriptor *config_desc;
	struct usb_interface_descriptor *interface_desc;
	unsigned config;
	int i;
	struct usb_config_descriptor *p_config_desc;
	int config_desc_length, ret;
#ifdef CONFIG_USB_COMPOSITE
	struct usb_endpoint_descriptor *ep_desc;
	__u8 num_itfs;
	__u8 cur_intf = 0, last_intf = 0;
#endif

	DMSG("----------%s------------\n", __FUNCTION__);
	req.bRequestType = USB_RECIP_DEVICE | USB_DIR_IN;
	req.bRequest = USB_REQ_GET_DESCRIPTOR;
	req.wValue = (USB_DT_CONFIG<<8);
	req.wIndex = 0;
	req.wLength = MAX_CONFIG_LENGTH;

	gadget->speed = USB_SPEED_FULL;
	dev->ep0state = EP0_IN_FAKE;
	i = driver->setup(gadget, &req);
#ifdef CONFIG_USB_PXA3XX_U2D
	gadget->speed = USB_SPEED_UNKNOWN;
#endif

#ifndef CONFIG_USB_COMPOSITE
	config_desc = (struct usb_config_descriptor *)dev->configs;
#else
	config_desc = (struct usb_config_descriptor *)
		       dev->active_gadget->config_desc;
#endif
	p_config_desc = config_desc;
	config_desc_length = config_desc->wTotalLength;

	if (config_desc->bDescriptorType == USB_DT_CONFIG) {
		config = config_desc->bConfigurationValue;
	} else {
		DMSG("wrong configuration\n");
		return -EFAULT;
	}

#ifndef CONFIG_USB_COMPOSITE
	while (config_desc_length >= 0) {
		ret = get_extra_descriptor((char *)p_config_desc,
					  config_desc_length,
					  USB_DT_INTERFACE,
					  (void **) &interface_desc);
		if (ret >= 0) {
			/* search eps and fill the pxa27x_ep_config struct */
			if (interface_desc->bNumEndpoints) {
				set_eps(interface_desc->bNumEndpoints,
					(struct usb_endpoint_descriptor *)
					 interface_desc,
					config_desc_length, config,
					interface_desc->bInterfaceNumber,
					interface_desc->bAlternateSetting);

				DMSG("config=%d, intf assigned=%d,"
				     " alt=%d\n", config, 
				     interface_desc->bInterfaceNumber,
				     interface_desc->bAlternateSetting);
			}
		} else {
			DMSG("interface config not find\n");
			return -EFAULT;
		}

		p_config_desc = (struct usb_config_descriptor *)
				((struct usb_interface_descriptor *)
				 interface_desc + 1);

		config_desc_length -= interface_desc->bLength;  /* yfw */
	}
#else
	num_itfs = config_desc->bNumInterfaces;

	DMSG("parse the config desc, assigned_intf_start=%d, num of intfs=%d\n",
	     dev->interface_count, num_itfs);

	dev->active_gadget->assigned_intf_start = dev->interface_count;
	dev->active_gadget->config = config;
	dev->active_gadget->num_intfs = num_itfs;

	/* get every interface desc, fill the gadget_driver_info structure */
	for (i = 0; i < num_itfs; i++) {
		DMSG("\nparse interface %d, p_config_desc=%p,"
		     " config_desc_length=%d\n", i, p_config_desc,
		     config_desc_length);

		while (config_desc_length >= 0) {
			ret = get_extra_descriptor((char *)p_config_desc,
						   config_desc_length,
						   USB_DT_INTERFACE,
						   (void **)&interface_desc);
			if (ret >= 0) {
				cur_intf = interface_desc->bInterfaceNumber;

				config_desc_length -= (u32)interface_desc -
						      (u32)p_config_desc;

				if (cur_intf != last_intf) {
					p_config_desc =
						(struct usb_config_descriptor *)
						interface_desc;
					goto next_intf;
				}

				/* set interface number to assigned one */
				interface_desc->bInterfaceNumber = i +
					dev->active_gadget->assigned_intf_start;

				interface_desc->iInterface = 0;

				/* search eps and fill the pxa27x_ep_config*/
				ep_desc = (struct usb_endpoint_descriptor *)
					   interface_desc;
				if (interface_desc->bNumEndpoints) {
					set_eps(interface_desc->bNumEndpoints,
						ep_desc, config_desc_length,
						config, cur_intf,
						interface_desc->bAlternateSetting);
				}

				DMSG("config=%d, intf=%d(assigned=%d),"
				     " alt=%d\n", config, cur_intf,
				     interface_desc->bInterfaceNumber,
				     interface_desc->bAlternateSetting);
			} else {
				DMSG("no more alt interfaces,"
				     " config_desc_length=%d, goto next_intf\n",
				config_desc_length);
				goto next_intf;
			} /* if */

			p_config_desc = (struct usb_config_descriptor *)
					((struct usb_interface_descriptor *)
					 interface_desc + 1);

			config_desc_length -= interface_desc->bLength;/* yfw */
			DMSG("  p_config_desc=%p, interface_desc=%p, "
			     "config_desc_length=%d\n",
			     p_config_desc, interface_desc, config_desc_length);
		} /* while */

next_intf:
		last_intf = cur_intf;
		dev->interface_count++;

		DMSG("parse interface %d finished, dev->interface_count=%d\n",
		     i, dev->interface_count);
	} /* for */

	/* set CDC union descriptors */
	set_cdc_desc(dev);
#endif
	return 0;
}

int comp_calc_config(struct pxa3xx_comp *dev, int ep_num)
{
#ifndef CONFIG_USB_COMPOSITE
	struct pxa3xx_comp_ep *ep = find_ep_num(dev->first_ep, ep_num);
	if (!ep) {
		printk(KERN_ERR "config can't find ep %d"
			" first_ep %p ???\n\n", ep_num, dev->first_ep);
		return 0;
	}
	return ep->config;
#else
	return ((struct usb_config_descriptor *)
		dev->first_gadget->config_desc)->bConfigurationValue;
#endif
}

int comp_calc_interface(struct pxa3xx_comp *dev, int ep_num)
{
	struct pxa3xx_comp_ep *ep = find_ep_num(dev->first_ep, ep_num);
	if (!ep) {
		printk(KERN_ERR "intf can't find ep %d???\n\n", ep_num);
		return 0;
	}
#ifndef CONFIG_USB_COMPOSITE
	return ep->interface;
#else
	return ep->assigned_interface;
#endif
}

static void
#ifndef CONFIG_USB_COMPOSITE
stop_activity(struct usb_gadget *gadget, struct usb_gadget_driver *driver)
{
#else
stop_activity(struct usb_gadget *gadget, struct gadget_driver_info *p_info)
{
	struct usb_gadget_driver *driver=NULL;
#endif

	DMSG("Trace path 1\n");
	stop_udc(driver);

	/* report disconnect; the driver is already quiesced */
#ifndef CONFIG_USB_COMPOSITE
	if (driver)
		driver->disconnect(gadget);
#else
	if (!p_info->stopped)
		p_info->driver->disconnect(gadget);
	p_info->stopped = 1;
#endif

	/* re-init driver-visible data structures */
	udc_reinit();
}

#ifdef CONFIG_USB_COMPOSITE
struct gadget_driver_info *get_driver_info(struct pxa3xx_comp *dev,
					   struct usb_gadget_driver *driver)
{
	struct gadget_driver_info *p_info = dev->first_gadget;

	while (p_info && (p_info->driver != driver)) p_info = p_info->next;

	return p_info;
}
#endif

int comp_check_driver(struct pxa3xx_comp *dev,
		      struct usb_gadget_driver *slf_drver,
		      struct usb_gadget_driver *driver)
{
#ifdef CONFIG_USB_OTG
	if (dev->transceiver && dev->transceiver->default_a) {
		printk(KERN_ERR "Mini-A connected!  "
		       "This operation may cause unexpected error!!!\n");
	}
#endif

#ifdef CONFIG_USB_COMPOSITE
	{
	struct gadget_driver_info *p_info = get_driver_info(dev, driver);

	if (!driver || NULL == p_info) {
		printk(KERN_ERR "%s, can't find driver!\n", __FUNCTION__);
		return 0;
	}
	}
	return 1;
#else
	if (!driver || driver != slf_drver) {
		printk(KERN_ERR "%s, can't find driver!\n", __FUNCTION__);
		return 0;
	}
	return 1;
#endif
}

int comp_is_dev_busy(struct pxa3xx_comp *dev, struct usb_gadget_driver *driver)
{
#ifndef CONFIG_USB_COMPOSITE
	if (driver)
		return 1;
#else
	/* FIXME remove all modules before insert again */
	if ((dev->rm_flag) && dev->first_gadget) {
		printk(KERN_ERR "left modules may not work!  "
		       "please remove all and insert again!!!\n");
		return 1;
	}
#ifdef CONFIG_USB_OTG
	if(dev->transceiver && dev->transceiver->default_a) {
		printk(KERN_ERR "Mini-A connected!  "
		       "please unplug it and insert module again!!!\n");
		return 1;
	}
#endif
#endif
	return 0;
}

int stop_cur_gadget(struct pxa3xx_comp *dev, struct usb_gadget *gadget,
		    struct usb_gadget_driver *driver)
{
#ifdef CONFIG_USB_COMPOSITE
	struct gadget_driver_info *p_info = get_driver_info(dev, driver);

	set_gadget_data(gadget, p_info->driver_data);
	stop_activity(gadget, p_info);
	return 0;
#else
	stop_activity(gadget, driver);
	return 0;
#endif
}
void comp_register_driver(struct pxa3xx_comp *dev, struct usb_gadget *gadget,
			    struct usb_gadget_driver *driver)
{
#ifdef CONFIG_USB_COMPOSITE
	/* allocate gadget_driver_info and attach it to controller */
	gadget_info_init(dev, gadget, driver);
	dev->active_gadget->driver_data = get_gadget_data(gadget);
#ifdef MULTI_P3
	gadget_get_device_desc(dev, gadget, driver);
#endif /* MULTI_P3 */
#endif
	/* After driver is bound, send a fake get configuration command to
	 * gadget driver to get the configuration information */
	gadget_get_config_desc(dev, gadget, driver);
#if defined(CONFIG_USB_COMPOSITE) && (defined(CONFIG_USB_PXA3XX_U2D) \
	|| defined(CONFIG_USB_PXA_U2O))
	gadget_get_config_desc_hs(dev, gadget, driver);
#endif
}

void comp_unregister_driver(struct pxa3xx_comp *dev,
			    struct usb_gadget *gadget,
			    struct usb_gadget_driver **slf_drver,
			    struct usb_gadget_driver *driver)
{
#ifndef CONFIG_USB_COMPOSITE
	delete_comp_ep_list(&dev->first_ep);
#else
	gadget_info_uninit(dev, gadget, slf_drver, driver);

	memset(dev->configs, 0, MAX_CONFIG_LENGTH);
	if (dev->driver_count != 0) {
		dev->rm_flag = 1;
		printk(KERN_WARNING "left modules may not work!  "
		       "please remove all and insmod again!!!\n");
	} else {
		delete_comp_ep_list(&dev->first_ep);
		dev->rm_flag = 0;
	}
#endif
}

void comp_driver_suspend(struct pxa3xx_comp *dev, struct usb_gadget *gadget,
			 struct usb_gadget_driver *driver)
{
#ifndef CONFIG_USB_COMPOSITE
	if (driver->suspend)
		driver->suspend(gadget);
#else
	struct gadget_driver_info *p_info = dev->first_gadget;

	do {
		set_gadget_data(gadget, p_info->driver_data);
		if (p_info->driver->suspend)
			p_info->driver->suspend(gadget);
		p_info = p_info->next;
	} while (p_info);
#endif
}

void comp_driver_resume(struct pxa3xx_comp *dev, struct usb_gadget *gadget,
			struct usb_gadget_driver *driver)
{
#ifndef CONFIG_USB_COMPOSITE
	if (driver->resume)
		driver->resume(gadget);
#else
	struct gadget_driver_info *p_info = dev->first_gadget;

	do {
		set_gadget_data(gadget, p_info->driver_data);
		if (p_info->driver->resume)
			p_info->driver->resume(gadget);
		p_info = p_info->next;
	} while (p_info);
#endif
}

int comp_change_config(struct pxa3xx_comp *dev, struct usb_gadget *gadget,
			struct usb_gadget_driver *driver,
			struct usb_ctrlrequest *req)
{
	int ret;
#ifndef CONFIG_USB_COMPOSITE
	ret = driver->setup(gadget, req);
#else
	struct gadget_driver_info *p_info = dev->first_gadget;

	do {
		set_gadget_data(gadget, p_info->driver_data);
#ifndef MULTIPLE_CONFIGURATION
		/* switch to gadget driver's configuration */
		comp_print("set config %d orig %d ===============\n", 
				req->wValue, p_info->config);
		req->wValue = p_info->config;
#endif
		if ((ret = p_info->driver->setup(gadget, req))) {
			printk(KERN_DEBUG "set %s config %d fail %d ?\n", 
				p_info->driver->function, p_info->config, ret);
		}
		p_info->stopped = 0;
		p_info = p_info->next;
	} while (p_info);
#endif
	return ret;
}

struct usb_gadget_driver
*comp_change_interface(struct pxa3xx_comp *dev, int active_interface,
		       struct usb_ctrlrequest *req, struct usb_gadget *gadget,
		       struct usb_gadget_driver *driver, int *ret)
{
#ifndef CONFIG_USB_COMPOSITE
	dev->ep0state = EP0_IN_DATA_PHASE;
	driver->setup(gadget, req);
	return driver;
#else
	struct gadget_driver_info *p_info = NULL;
	struct pxa3xx_comp_ep *ep;

	/* change the assigned interface to gadget interface */
	ep = find_ep_intf(dev->first_ep, active_interface);
	if (ep) {
		DMSG("dev->ep[%d]:assigned_interface = %d, driver_info=0x%x\n",
		     ep->log_ep_num, ep->assigned_interface,
		     (unsigned)(ep->driver_info));
		p_info = ep->driver_info;
		req->wIndex = ep->interface;
		DMSG("	req.wValue = %d, req.wIndex = %d\n",
		     req->wValue, req->wIndex);
	}

	if (p_info == NULL) {
		printk(KERN_ERR "active interface not found, error\n");
		return NULL;
	} else {
		dev->active_gadget = p_info;

		set_gadget_data(gadget, dev->active_gadget->driver_data);

		dev->interface = active_interface;

		dev->ep0state = EP0_IN_DATA_PHASE;

		*ret = p_info->driver->setup(gadget, req);
		if (*ret == -EOPNOTSUPP)
			DMSG(" ret EOPNOTSUPP\n");

		return p_info->driver;
	}
#endif
}

void stop_gadget(struct pxa3xx_comp *dev, struct usb_gadget *gadget,
		 struct usb_gadget_driver *driver)
{
#ifndef CONFIG_USB_COMPOSITE
	stop_activity(gadget, driver);
#else
	struct gadget_driver_info *pInfo = dev->first_gadget;

	while (pInfo) {
		set_gadget_data(gadget, pInfo->driver_data);
		stop_activity(gadget, pInfo);
		pInfo = pInfo->next;
	}
#endif
}

void udc_stop(struct pxa3xx_comp *dev, struct usb_gadget *gadget,
	      struct usb_gadget_driver *driver, int state)
{
#ifdef CONFIG_USB_COMPOSITE
	struct gadget_driver_info *pInfo = dev->first_gadget;

	if (driver) {
		if (state == 0) {
			do {
				pInfo->stopped = 0;
				pInfo = pInfo->next;
			} while (pInfo);
		} else if (state == 1) {
			stop_gadget(dev, gadget, driver);
		} else
			printk(KERN_ERR "stop state %d error\n", state);

	}
#else
	if (state == 1)
	stop_gadget(dev, gadget, driver);
#endif
}

#ifdef CONFIG_USB_COMPOSITE
#ifdef DEBUG
/*  dump the gadget_driver_info structure
 */
static void gadget_info_dump(struct pxa3xx_comp *dev)
{
	struct gadget_driver_info *p_info = dev->first_gadget;
	int i = 1;

	DMSG("%s, dev->interface_count= 0x%x\n", __FUNCTION__,
	     dev->interface_count);
	while (p_info) {

		DMSG("i=%d, p_info=%p\n", i, p_info);
		DMSG("   next = 0x%x\n", (unsigned)p_info->next);
		DMSG("   config = 0x%x\n", p_info->config);
		DMSG("   assigned_intf_start = 0x%x\n",
		     p_info->assigned_intf_start);
		DMSG("   num_intfs = 0x%x\n", p_info->num_intfs);
		DMSG("   config_desc = 0x%x\n", (unsigned)p_info->config_desc);
		DMSG("   driver = 0x%x\n", (unsigned)p_info->driver);
		DMSG("   driver_data = 0x%x\n", (unsigned)p_info->driver_data);

		p_info = p_info->next;
		i++;
	}
	DMSG("dev->first_gadget = %p\n", dev->first_gadget);
	DMSG("dev->active_gadget = %p\n", dev->active_gadget);
}
#endif

static void udc_setup_complete(struct usb_ep *ep, struct usb_request *req);
/* gadget_info_init
 * init the gadget_driver_info structure when the driver is registered
 * combined from several gadget driver, should be lager ??
 */
#define REQ_BUFSIZ 256
static int gadget_info_init(struct pxa3xx_comp *dev, struct usb_gadget *gadget,
			    struct usb_gadget_driver *driver)
{
	struct gadget_driver_info *info;
	struct gadget_driver_info *p_info;

	/* set up the new gadget driver info */
	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info) {
		printk(KERN_ERR "kmalloc gadget_driver_info error\n");
		return -EBUSY;
	}

	info->driver = driver;
	info->next = NULL;

	info->num_intfs = 0;
	info->stopped = 1;

	if (dev->first_gadget) {
		/* find the last element */
		p_info = dev->first_gadget;
		while (p_info->next)
			p_info = p_info->next;
		/* set up the struct.
		 * the last registered driver is always the active one
		 * before receive the set_interface request
		 */
		p_info->next = info;
		dev->active_gadget = info;
	} else {
		dev->first_gadget = dev->active_gadget = info;
		dev->interface_count = 0;
		dev->driver_count = 0;

		/* init ep0 control request queueand buffer */
		memset((void *)&dev->ep0_req, 0, sizeof(dev->ep0_req));
		INIT_LIST_HEAD(&dev->ep0_req.queue);

		dev->ep0_req.req.complete = udc_setup_complete;
		dev->ep0_req.req.buf = kzalloc(REQ_BUFSIZ, GFP_KERNEL);
		if (!dev->ep0_req.req.buf) {
			usb_ep_free_request(gadget->ep0, &dev->ep0_req.req);
			DMSG("%s, dev->ep0_req.req.buf malloc error\n",
			     __FUNCTION__);
		}
	}

	return 0;
}

static int gadget_info_uninit(struct pxa3xx_comp *dev,
			      struct usb_gadget *gadget,
			      struct usb_gadget_driver **slf_drver,
			      struct usb_gadget_driver *driver)
{
	struct gadget_driver_info *p_info = dev->first_gadget;
	struct gadget_driver_info *info = NULL;

	do {
		/* find the gadget driver info to p_info */
		if (p_info->driver == driver) {
			/* for the first driver is being removed*/
			if (!info)
				info = p_info->next;
			break;
		}
		/* save the previous one */
		info = p_info;
		p_info = p_info->next;
	} while (p_info);

	if (NULL == p_info) {
		printk(KERN_ERR "%s, can't find driver!\n", __FUNCTION__);
		return -EINVAL;
	}

	/* put the active one to the previous one */
	if (dev->first_gadget == p_info) {
		if (info)
			dev->first_gadget = info;
		else
			dev->first_gadget = p_info->next;
	}
	if (dev->active_gadget == p_info) {
		if (info)
			dev->active_gadget = info;
		else
			dev->active_gadget = p_info->next;
	}
	if ((info) && (info != p_info->next))
		info->next = p_info->next;

	if (dev->active_gadget) {
		*slf_drver = dev->active_gadget->driver;
		gadget->dev.driver = &dev->active_gadget->driver->driver;
	} else
		*slf_drver = 0;/* no drivers left */

	dev->interface_count -= p_info->num_intfs;

	kfree(p_info);
	return 0;
}

#ifdef MULTI_P3
struct usb_interface_assoc_descriptor
iad_desc = {
	.bLength                = sizeof iad_desc,
	.bDescriptorType        = USB_DT_INTERFACE_ASSOCIATION,

	.bFirstInterface        = 0,
	.bInterfaceCount        = 0,
	.bFunctionClass         = 0,
	.bFunctionSubClass      = 0,
	.bFunctionProtocol      = 0,
	.iFunction              = 0,
};

static void  set_iad_desc(struct usb_device_descriptor *device_desc,
			  __u8 first_intf, __u8 num_intfs)
{
	iad_desc.bFirstInterface = first_intf;
	iad_desc.bInterfaceCount = num_intfs;

	iad_desc.bFunctionClass = device_desc->bDeviceClass;
	iad_desc.bFunctionSubClass = device_desc->bDeviceSubClass;
	iad_desc.bFunctionProtocol = device_desc->bDeviceProtocol;
}

static int gadget_get_device_desc(struct pxa3xx_comp *dev,
				  struct usb_gadget *gadget,
				  struct usb_gadget_driver *driver)
{
	struct usb_ctrlrequest req;
	int i;

	DMSG(KERN_DEBUG "%s\n", __FUNCTION__);
	req.bRequestType = USB_RECIP_DEVICE | USB_DIR_IN;
	req.bRequest = USB_REQ_GET_DESCRIPTOR;
	req.wValue = (USB_DT_DEVICE << 8);
	req.wIndex = 0;
	req.wLength = sizeof(struct usb_device_descriptor);

	dev->ep0state = EP0_IN_FAKE;
	i = driver->setup(gadget, &req);

	return 0;
}
#endif

/* combine_configuration
 * Combine the configuration descriptors for all gadget drivers registered.
 * Add the IAD descriptor if there are more than one interfaces within one
 * function.
 */
static int combine_configuration(struct pxa3xx_comp *dev, int speed)
{
	struct gadget_driver_info *p_info = dev->first_gadget;
	struct usb_config_descriptor *config_desc;
	struct usb_device_descriptor *device_desc;

#ifdef MULTI_P3
	struct usb_interface_assoc_descriptor *p_iad_desc;
#endif
	struct usb_config_descriptor *configs =
		(struct usb_config_descriptor *)dev->configs;
	int desc_length;

	DMSG("%s\n", __FUNCTION__);

	/* config desc, may diff between gadget drivers */
	configs->bLength = USB_DT_CONFIG_SIZE;
	configs->bDescriptorType = USB_DT_CONFIG;

	configs->wTotalLength = USB_DT_CONFIG_SIZE;
	configs->bNumInterfaces = 0;
	config_desc = (struct usb_config_descriptor *)(p_info->config_desc);
	configs->bConfigurationValue = config_desc->bConfigurationValue;
	configs->iConfiguration = 0;
	configs->bmAttributes = USB_CONFIG_ATT_ONE | USB_CONFIG_ATT_SELFPOWER;
	configs->bMaxPower = 1;

	do {
		if (speed ==  USB_SPEED_HIGH)
			config_desc = (struct usb_config_descriptor *)
				(p_info->config_desc_hs);
		else
			config_desc = (struct usb_config_descriptor *)
				(p_info->config_desc);
		device_desc = (struct usb_device_descriptor *)
			(p_info->device_desc);
		configs->bNumInterfaces += (u8)(config_desc->bNumInterfaces);

#ifdef MULTI_P3
		/* add the IAD descriptor if there are multiple interface in one
		   gadget driver */
		if (config_desc->bNumInterfaces > 1) {

			if ((get_extra_descriptor((char *)config_desc,
						  config_desc->wTotalLength,
						  USB_DT_INTERFACE_ASSOCIATION,
						  (void **)&p_iad_desc)) >= 0)
				p_iad_desc->bFirstInterface =
					p_info->assigned_intf_start;
			else {
				/* fill the iad_desc
				 * functionClass/subclass/protocol fields */
				set_iad_desc((struct usb_device_descriptor *)
					     p_info->device_desc,
					     p_info->assigned_intf_start,
					     config_desc->bNumInterfaces);
				memcpy((u8 *)configs + configs->wTotalLength,
				       (u8 *)&iad_desc, sizeof iad_desc);
				configs->wTotalLength += sizeof iad_desc;
			}
		}
#endif

		/* copy all descriptors except config desc */
		memcpy((u8 *)configs + configs->wTotalLength,
		       (u8 *)config_desc + USB_DT_CONFIG_SIZE,
		       config_desc->wTotalLength - USB_DT_CONFIG_SIZE);

		/* modify the interface number to assigned interface number */
		desc_length = config_desc->wTotalLength - USB_DT_CONFIG_SIZE;

		configs->wTotalLength += (config_desc->wTotalLength -
					  USB_DT_CONFIG_SIZE);
		p_info = p_info->next;
		DMSG("configs->wTotalLength = 0x%x\n", configs->wTotalLength);
	} while (p_info != NULL);

#ifdef DEBUG
	gadget_info_dump(dev);
#endif

	return configs->wTotalLength;
}

/* set_cdc_desc
 * modify the cdc union descriptor which include the master/slave interface
 * number
 */
static void set_cdc_desc(struct pxa3xx_comp *dev)
{
	struct usb_device_descriptor *device_desc =
		(struct usb_device_descriptor *)dev->active_gadget->device_desc;
	struct usb_config_descriptor *config_desc =
		(struct usb_config_descriptor *)dev->active_gadget->config_desc;
	struct usb_cdc_union_desc *union_desc;
	int config_desc_len = config_desc->wTotalLength, ret = 0;

	if (device_desc->bDeviceClass != USB_CLASS_COMM)
		return;

	while ((config_desc_len > 0) && (ret >= 0)) {
		ret = get_extra_descriptor((char *)config_desc,	config_desc_len,
					   USB_DT_CS_INTERFACE,
					   (void **)&union_desc);
		if (ret >= 0) {
			if (union_desc->bDescriptorSubType ==
				USB_CDC_UNION_TYPE) {
				DMSG("found cdc union desc, change to %d\n",
				     dev->active_gadget->assigned_intf_start);
				union_desc->bMasterInterface0 =
					dev->active_gadget->assigned_intf_start;
				union_desc->bSlaveInterface0 = 1 +
					dev->active_gadget->assigned_intf_start;
			}

			if (union_desc->bDescriptorSubType
				== USB_CDC_CALL_MANAGEMENT_TYPE) {
				DMSG("found cdc call mgt desc, change to %d\n",
				     dev->active_gadget->assigned_intf_start
				     + 1);
				((struct usb_cdc_call_mgmt_descriptor *)
				 union_desc)->bDataInterface = 1 +
					dev->active_gadget->assigned_intf_start;
			}
			config_desc_len -= ((unsigned)union_desc -
					    (unsigned)config_desc
					    + union_desc->bLength);
			config_desc = (struct usb_config_descriptor *)
				      ((unsigned)union_desc
				       + union_desc->bLength);
		}
	}
}

/* get the hs configuration desc, change the interface number */
static int gadget_get_config_desc_hs(struct pxa3xx_comp *dev,
				     struct usb_gadget *gadget,
				     struct usb_gadget_driver *driver)
{
	struct usb_ctrlrequest req;
	struct usb_config_descriptor *config_desc;
	struct usb_interface_descriptor *interface_desc;
	unsigned config;
	int i;
	__u8 num_itfs;
	__u8 cur_intf = 0, last_intf = 0;
	struct usb_config_descriptor *p_config_desc;
	int config_desc_length, ret;

	DMSG("----------%s------------\n", __FUNCTION__);
	req.bRequestType = USB_RECIP_DEVICE | USB_DIR_IN;
	req.bRequest = USB_REQ_GET_DESCRIPTOR;
	req.wValue = (USB_DT_CONFIG << 8);
	req.wIndex = 0;
	req.wLength = MAX_CONFIG_LENGTH;

	dev->ep0state = EP0_IN_FAKE;
	gadget->speed = USB_SPEED_HIGH;
	i = driver->setup(gadget, &req);
	gadget->speed = USB_SPEED_UNKNOWN;

	config_desc = (struct usb_config_descriptor *)
		       dev->active_gadget->config_desc_hs;

	if (config_desc->bDescriptorType == USB_DT_CONFIG)
		config = config_desc->bConfigurationValue;
	else {
		DMSG("wrong configuration\n");
		return -EFAULT;
	}

	num_itfs = config_desc->bNumInterfaces;
	p_config_desc = config_desc;
	config_desc_length = config_desc->wTotalLength;

	DMSG("parse the config desc, assigned_intf_start=%d, num of intfs=%d\n",
	     dev->interface_count, num_itfs);

	/* get every interface desc, fill the gadget_driver_info structure */
	for (i = 0; i < num_itfs; i++) {
		DMSG("\nparse interface %d, config_desc_length=%d\n",
		     i, config_desc_length);

		while (config_desc_length >= 0) {
			ret = get_extra_descriptor((char *)p_config_desc,
						   config_desc_length,
						   USB_DT_INTERFACE,
						   (void **)&interface_desc);
			if (ret >= 0) {
				cur_intf = interface_desc->bInterfaceNumber;
				DMSG("  cur_intf=%d, last_intf=%d,"
				     " config_desc_length=%d\n", cur_intf,
				     last_intf, config_desc_length);

				config_desc_length -= (u32)interface_desc -
						      (u32)p_config_desc;

				if (cur_intf != last_intf) {
					p_config_desc =
						(struct usb_config_descriptor *)
						interface_desc;
					goto next_intf;
				}

				/* set interface number to assigned one */
				interface_desc->bInterfaceNumber = i +
					dev->active_gadget->assigned_intf_start;

				interface_desc->iInterface = 0;
			} else {
				DMSG("no alt interfaces, config_desc_length=%d,"
				     " goto next_intf\n", config_desc_length);
				goto next_intf;
			} /* if */

			p_config_desc = (struct usb_config_descriptor *)
					((struct usb_interface_descriptor *)
					 interface_desc + 1);

			config_desc_length -= interface_desc->bLength; /* yfw */
		} /* while */

next_intf:
		last_intf = cur_intf;

		DMSG("parse interface %d finished, dev->interface_count=%d\n",
		     i, dev->interface_count);
	}

	/* set CDC union descriptors */
	set_cdc_desc(dev);

	return 0;
}

/* string desc not supported yet */
#define UDC_STRING_MANUFACTURER	0
#define UDC_STRING_PRODUCT		0

static struct usb_device_descriptor
udc_device_desc = {
	.bLength =		sizeof udc_device_desc,
	.bDescriptorType =	USB_DT_DEVICE,

#if defined(CONFIG_USB_PXA27X_UDC)
	.bcdUSB =		__constant_cpu_to_le16(0x0110),
#elif defined(CONFIG_USB_PXA3XX_U2D)
	.bcdUSB =		__constant_cpu_to_le16(0x0200),
#endif
	/*  USB_CLASS_COMM, for rndis */
	.bDeviceClass =		USB_CLASS_PER_INTERFACE,
	.bDeviceSubClass =	0,
	.bDeviceProtocol =	0,
#ifndef CONFIG_USB_PXA_U2O
	.bMaxPacketSize0 = 	EP0_FIFO_SIZE, /*  for pxa3xx */
#else
	.bMaxPacketSize0 = 	64, /*  for pxa9xx u2o, xj check */
#endif
	.idVendor =		__constant_cpu_to_le16(UDC_VENDOR_NUM),
	.idProduct =		__constant_cpu_to_le16(UDC_PRODUCT_NUM),
	.iManufacturer =	UDC_STRING_MANUFACTURER,
	.iProduct =		UDC_STRING_PRODUCT,
	.bNumConfigurations =	1,
};

static struct usb_qualifier_descriptor
usb_qualifier_desc = {
	.bLength                = sizeof usb_qualifier_desc,
	.bDescriptorType        = USB_DT_DEVICE_QUALIFIER,

	.bcdUSB                 = __constant_cpu_to_le16(0x0200),
	/* USB_CLASS_COMM, for rndis */
	.bDeviceClass           = USB_CLASS_PER_INTERFACE,
	.bDeviceSubClass        = 0,
	.bDeviceProtocol        = 0,
#ifndef CONFIG_USB_PXA_U2O
	.bMaxPacketSize0 	= EP0_FIFO_SIZE, /*  for pxa3xx */
#else
	.bMaxPacketSize0 = 	64, /*  for pxa9xx u2o, xj check */
#endif
	.bNumConfigurations     = 1,
};

static void udc_setup_complete(struct usb_ep *ep, struct usb_request *req)
{
	if (req->status || req->actual != req->length)
		DMSG("pseudo setup complete --> %d, %d/%d\n",
				req->status, req->actual, req->length);
}

static int udc_do_specific_requests(struct pxa3xx_comp *dev,
				    struct usb_gadget *gadget,
				    struct usb_ctrlrequest *ctrl,
				    struct gadget_driver_info **gadget_info)
{
	struct gadget_driver_info *p_info = dev->first_gadget;

	if (((ctrl->bRequestType == 0x21) && (ctrl->bRequest == 0x00)) ||
	    ((ctrl->bRequestType == 0xa1) && (ctrl->bRequest == 0x01))) {
		while (p_info && (strcmp(p_info->driver->driver.name,
			       "g_ether")))
			p_info = p_info->next;
		if (p_info == NULL) {
			printk(KERN_ERR "%s,eth not found????\n", __func__);
			return -1;
		} else
			set_gadget_data(gadget, p_info->driver_data);
	}

	if (((ctrl->bRequestType == 0xa1) && (ctrl->bRequest == 0xfe)) ||
	    ((ctrl->bRequestType == 0x21) && (ctrl->bRequest == 0xff))) {
		p_info = dev->first_gadget;
		while (p_info && (strcmp(p_info->driver->driver.name,
			       "g_file_storage")))
			p_info = p_info->next;
		if (p_info == NULL) {
			printk(KERN_ERR "%s, mass not found????\n", __func__);
			return -1;
		} else
			set_gadget_data(gadget, p_info->driver_data);
	}

	if (((ctrl->bRequestType == 0xa1) && (ctrl->bRequest == 0x21)) ||
	    ((ctrl->bRequestType == 0x21) && (ctrl->bRequest == 0x20)) ||
	    ((ctrl->bRequestType == 0x21) && (ctrl->bRequest == 0x22))) {
		p_info = dev->first_gadget;
		while (p_info && (strcmp(p_info->driver->driver.name,
			       "gs_modem")))
			p_info = p_info->next;
		if (p_info == NULL) {
			printk(KERN_DEBUG "%s, gs_modem not found????\n\n", __func__);
			return -1;
		} else
			set_gadget_data(gadget, p_info->driver_data);
	}

	*gadget_info = p_info;
	return 0;
}

static int udc_do_request(struct pxa3xx_comp *dev, struct usb_ctrlrequest *ctrl,
			  struct usb_ep *ep, int speed)
{
	struct usb_request *usb_req = &dev->ep0_req.req;
	int value = -EOPNOTSUPP;
	int pseudo = 0, ret = 0;

	switch (ctrl->bRequest) {

	case USB_REQ_GET_DESCRIPTOR:
		if (ctrl->bRequestType != USB_DIR_IN)
			break;
		switch (ctrl->wValue >> 8) {

		case USB_DT_DEVICE:
			DMSG("%s, get device desc\n", __FUNCTION__);
			pseudo = 1;
			/* send the pseudo device desc */
			value = min(ctrl->wLength, (u16)sizeof udc_device_desc);
			memcpy(usb_req->buf, &udc_device_desc, value);
			break;

		case USB_DT_CONFIG:
			DMSG("%s, get conf desc\n", __FUNCTION__);
			pseudo = 1;
			/* send the pseudo configuration desc */
			value = combine_configuration(dev, speed);
			value = min((int)ctrl->wLength, value);
			memcpy(usb_req->buf, &dev->configs, value);
			break;

		case USB_DT_OTHER_SPEED_CONFIG:
			DMSG("%s, get other speed conf desc\n", __FUNCTION__);
			/* send the pseudo configuration desc */
			if (speed != USB_SPEED_HIGH &&
				speed != USB_SPEED_FULL) {
				pr_err("%s, unknown speed, error\n", __func__);
				break;
			}
			value = combine_configuration(dev, speed);
			pseudo = 1;
			value = min((int)ctrl->wLength, value);
			memcpy(usb_req->buf, &dev->configs, value);
			break;

		case USB_DT_DEVICE_QUALIFIER:
			DMSG("%s, get device qualifier desc\n", __FUNCTION__);
			pseudo = 1;
			/* send the pseudo device desc */
			value = min(ctrl->wLength,
				    (u16)sizeof(usb_qualifier_desc));
			memcpy(usb_req->buf, &usb_qualifier_desc, value);
			break;

		default:
			break;
		}
	default:
		break;
	}

	if (pseudo) {
		usb_req->length = value;
		usb_req->no_interrupt = 0;
		usb_req->zero = value < ctrl->wLength
				&& (value % ep->maxpacket) == 0;
		usb_req->complete = udc_setup_complete;

		ret = ep->ops->queue(ep, usb_req, GFP_KERNEL);
		if (!(ret == 0))
			DMSG("%s, ep_queue error = 0x%x", __FUNCTION__, ret);
		return value;
	} else {
		return -1;
	}
}
#endif

int comp_ep0_req(struct pxa3xx_comp *dev, struct usb_gadget *gadget,
		 struct usb_ep *ep0, struct usb_ctrlrequest *usb_req)
{
#ifdef CONFIG_USB_COMPOSITE
	int i;
	struct pxa3xx_comp_ep *ep;
	struct gadget_driver_info *p_info = dev->active_gadget;
	struct gadget_driver_info *p_cur_info = dev->active_gadget;

	i = udc_do_request(dev, usb_req, ep0, gadget->speed);

	/* class specfic requests needed to be set up */
	if (i < 0) {
		if ((usb_req->bRequestType & USB_RECIP_MASK) ==
			USB_RECIP_INTERFACE) {
			ep = find_ep_intf(dev->first_ep, usb_req->wIndex);
			if (ep) {
				usb_req->wIndex = ep->interface;
				goto set_gd_data;
			}
		}
		if ((usb_req->bRequestType & USB_RECIP_MASK) ==
			USB_RECIP_ENDPOINT) {
			i = usb_req->wIndex & 0xf;
			ep = find_ep_num(dev->first_ep, i);
			if (!ep)
				return 0;
set_gd_data:
			p_info = ep->driver_info;
			if (p_info == NULL) {
				pr_err("wrong req!\n");
				p_info = p_cur_info;
			}
			set_gadget_data(gadget, p_info->driver_data);
		}

		udc_do_specific_requests(dev, gadget, usb_req, &p_info);

		i = p_info->driver->setup(gadget, usb_req);

		if (i < 0) {
			p_info = dev->first_gadget;
			do {
				set_gadget_data(gadget, p_info->driver_data);
				i = p_info->driver->setup(gadget, usb_req);
				p_info = p_info->next;
			} while ((i == -EOPNOTSUPP) && (p_info));
			if (i == -EOPNOTSUPP)
				DMSG("%s, no correct driver found!\n",
				     __FUNCTION__);
			set_gadget_data(gadget, p_cur_info->driver_data);
		} /* if(i) */
	} /* if(!i) */
	return i;
#endif
	return 0;
}
