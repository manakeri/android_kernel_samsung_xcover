/*
 * Gadget Driver for Android
 *
 * Copyright (C) 2008 Google, Inc.
 * Author: Mike Lockwood <lockwood@android.com>
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

/* #define DEBUG */
/* #define VERBOSE_DEBUG */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>

#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/utsname.h>
#include <linux/platform_device.h>

#include <linux/usb/android_composite.h>
#include <linux/usb/ch9.h>
#include <linux/usb/composite.h>
#include <linux/usb/gadget.h>

#include "gadget_chips.h"

/*
 * Kbuild is not very cooperative with respect to linking separately
 * compiled library objects into one module.  So for now we won't use
 * separate compilation ... ensuring init/exit sections work to shrink
 * the runtime footprint, and giving us at least some parts of what
 * a "gcc --combine ... part1.c part2.c part3.c ... " build would.
 */
#include "usbstring.c"
#include "config.c"
#include "epautoconf.c"
#include "composite.c"

MODULE_AUTHOR("Mike Lockwood");
MODULE_DESCRIPTION("Android Composite USB Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");

static const char longname[] = "Gadget Android";

/* Default vendor and product IDs, overridden by platform data */
//dh0318.lee 110310 FOR_DEVGURU change_vid

#if defined(CONFIG_MACH_GFORCE) || defined(CONFIG_MACH_ALKON) || defined(CONFIG_MACH_JETTA)
#define VENDOR_ID		0x04E8
#else
#define VENDOR_ID		0x1286
#endif
#define PRODUCT_ID		0x0001


struct android_dev {
	struct usb_composite_dev *cdev;
	struct usb_configuration *config;
	int num_products;
	struct android_usb_product *products;
	int num_functions;
	char **functions;

	int vendor_id;
	struct android_usb_product *active_product;
	struct android_usb_product *prev_product;
	int product_id;
	int version;
};

static struct android_dev *_android_dev;

/* string IDs are assigned dynamically */

#define STRING_MANUFACTURER_IDX		0
#define STRING_PRODUCT_IDX		1
#define STRING_SERIAL_IDX		2

/* String Table */
static struct usb_string strings_dev[] = {
	/* These dummy values should be overridden by platform data */
	[STRING_MANUFACTURER_IDX].s = "Android",
	[STRING_PRODUCT_IDX].s = "Android",
	[STRING_SERIAL_IDX].s = "0123456789ABCDEF",
	{  }			/* end of list */
};

static struct usb_gadget_strings stringtab_dev = {
	.language	= 0x0409,	/* en-us */
	.strings	= strings_dev,
};

static struct usb_gadget_strings *dev_strings[] = {
	&stringtab_dev,
	NULL,
};

static struct usb_device_descriptor device_desc = {
	.bLength              = sizeof(device_desc),
	.bDescriptorType      = USB_DT_DEVICE,
	.bcdUSB               = __constant_cpu_to_le16(0x0200),
#if (defined CONFIG_USB_ANDROID_RNDIS) || (defined CONFIG_USB_ANDROID_ACM) || \
	(defined CONFIG_USB_ANDROID_PXA910_MODEM) || \
	(defined CONFIG_USB_ANDROID_PXA955_ACM)
	.bDeviceClass         = USB_CLASS_MISC,
	.bDeviceSubClass      = 0x02,
	.bDeviceProtocol      = 0x01,
#else
	.bDeviceClass         = USB_CLASS_PER_INTERFACE,
#endif
	.idVendor             = __constant_cpu_to_le16(VENDOR_ID),
	.idProduct            = __constant_cpu_to_le16(PRODUCT_ID),
	.bcdDevice            = __constant_cpu_to_le16(0xffff),
	.bNumConfigurations   = 1,
};

static struct list_head _functions = LIST_HEAD_INIT(_functions);
static bool _are_functions_bound;
static void android_force_reset(struct android_dev *dev);

#if defined(CONFIG_MACH_GFORCE) || defined(CONFIG_MACH_ALKON) || defined(CONFIG_MACH_JETTA)
void android_usb_set_connected(int connected)
{
	if (_android_dev && _android_dev->cdev && _android_dev->cdev->gadget) {
		if (connected)
			usb_gadget_connect(_android_dev->cdev->gadget);
		else
			usb_gadget_disconnect(_android_dev->cdev->gadget);
	}
}
#endif 

static struct android_usb_function *get_function(const char *name)
{
	struct android_usb_function	*f;
	list_for_each_entry(f, &_functions, list) {
		if (!strcmp(name, f->name))
			return f;
	}
	return 0;
}

static bool are_functions_registered(struct android_dev *dev)
{
	char **functions = dev->functions;
	int i;

	/* Look only for functions required by the board config */
	for (i = 0; i < dev->num_functions; i++) {
		char *name = *functions++;
		bool is_match = false;
		/* Could reuse get_function() here, but a reverse search
		 * should yield less comparisons overall */
		struct android_usb_function *f;
		list_for_each_entry_reverse(f, &_functions, list) {
			if (!strcmp(name, f->name)) {
				is_match = true;
				break;
			}
		}
		if (is_match)
			continue;
		else
			return false;
	}

	return true;
}

static bool should_bind_functions(struct android_dev *dev)
{
	/* Don't waste time if the main driver hasn't bound */
	if (!dev->config)
		return false;

	/* Don't waste time if we've already bound the functions */
	if (_are_functions_bound)
		return false;

	/* This call is the most costly, so call it last */
	if (!are_functions_registered(dev))
		return false;

	return true;
}

static void bind_functions(struct android_dev *dev)
{
	struct android_usb_function	*f;
	char **functions = dev->functions;
	int i;

	for (i = 0; i < dev->num_functions; i++) {
		char *name = *functions++;
		f = get_function(name);
//#if defined(CONFIG_MACH_GFORCE) || defined(CONFIG_MACH_ALKON) || defined(CONFIG_MACH_JETTA)
//dh0318.lee 101220 COMPOSITE_GADGET
		mdelay(15);	//delay 15ms for completing the initiation of each functions.
//#endif
		if (f)
			f->bind_config(dev->config);
		else
			printk(KERN_ERR "function %s not found in bind_functions\n", name);
	}

	_are_functions_bound = true;
}

static int android_bind_config(struct usb_configuration *c)
{
	struct android_dev *dev = _android_dev;

	printk(KERN_DEBUG "android_bind_config\n");
	dev->config = c;

	if (should_bind_functions(dev))
		bind_functions(dev);

	return 0;
}

static int android_setup_config(struct usb_configuration *c,
		const struct usb_ctrlrequest *ctrl);

static struct usb_configuration android_config_driver = {
	.label		= "android",
	.bind		= android_bind_config,
	.setup		= android_setup_config,
	.bConfigurationValue = 1,
	.bmAttributes	= USB_CONFIG_ATT_ONE | USB_CONFIG_ATT_SELFPOWER,
	/*dh0318.lee 110823 FOR_USING_DEVGURU_DRIVER*/
	.bMaxPower	= 0x30, /*96ma*///0xFA, /* 500ma */
};

static int android_setup_config(struct usb_configuration *c,
		const struct usb_ctrlrequest *ctrl)
{
	int i;
	int ret = -EOPNOTSUPP;

	for (i = 0; i < android_config_driver.next_interface_id; i++) {
		if (android_config_driver.interface[i]->setup) {
			ret = android_config_driver.interface[i]->setup(
				android_config_driver.interface[i], ctrl);
			if (ret >= 0)
				return ret;
		}
	}
	return ret;
}

static int product_has_function(struct android_usb_product *p,
		struct usb_function *f)
{
	char **functions = p->functions;
	int count = p->num_functions;
	const char *name = f->name;
	int i;

	for (i = 0; i < count; i++) {
		/* For functions with multiple instances, usb_function.name
		 * will have an index appended to the core name (ex: acm0),
		 * while android_usb_product.functions[i] will only have the
		 * core name (ex: acm). So, only compare up to the length of
		 * android_usb_product.functions[i].
		 */
		if (!strncmp(name, functions[i], strlen(functions[i])))
			return 1;
	}
	return 0;
}

static int product_matches_functions(struct android_usb_product *p)
{
	struct usb_function		*f;
	list_for_each_entry(f, &android_config_driver.functions, list) {
		if (product_has_function(p, f) == !!f->disabled)
			return 0;
	}
	return 1;
}

static int get_vendor_id(struct android_dev *dev)
{
	struct android_usb_product *p = dev->products;
	int count = dev->num_products;
	int i;

	if (p) {
		for (i = 0; i < count; i++, p++) {
			if (p->vendor_id && product_matches_functions(p))
				return p->vendor_id;
		}
	}
	/* use default vendor ID */
	return dev->vendor_id;
}

static int get_product_id(struct android_dev *dev)
{
	struct android_usb_product *p = dev->products;
	int count = dev->num_products;
	int i;

	if (p) {
		for (i = 0; i < count; i++, p++) {
			if (product_matches_functions(p))
				return p->product_id;
		}
	}
	/* use default product ID */
	return dev->product_id;
}

static int android_bind(struct usb_composite_dev *cdev)
{
	struct android_dev *dev = _android_dev;
	struct usb_gadget	*gadget = cdev->gadget;
	int			gcnum, id, ret;

	printk(KERN_INFO "android_bind\n");

	/* Allocate string descriptor numbers ... note that string
	 * contents can be overridden by the composite_dev glue.
	 */
	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_dev[STRING_MANUFACTURER_IDX].id = id;
	device_desc.iManufacturer = id;

	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_dev[STRING_PRODUCT_IDX].id = id;
	device_desc.iProduct = id;

	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_dev[STRING_SERIAL_IDX].id = id;
	device_desc.iSerialNumber = id;

	/* register our configuration */
	ret = usb_add_config(cdev, &android_config_driver);
	if (ret) {
		printk(KERN_ERR "usb_add_config failed\n");
		return ret;
	}

	gcnum = usb_gadget_controller_number(gadget);
	if (gcnum >= 0)
		device_desc.bcdDevice = cpu_to_le16(0x0200 + gcnum);
	else {
		/* gadget zero is so simple (for now, no altsettings) that
		 * it SHOULD NOT have problems with bulk-capable hardware.
		 * so just warn about unrcognized controllers -- don't panic.
		 *
		 * things like configuration and altsetting numbering
		 * can need hardware-specific attention though.
		 */
		pr_warning("%s: controller '%s' not recognized\n",
			longname, gadget->name);
		device_desc.bcdDevice = __constant_cpu_to_le16(0x9999);
	}

	usb_gadget_set_selfpowered(gadget);
	dev->cdev = cdev;
	device_desc.idVendor = __constant_cpu_to_le16(get_vendor_id(dev));
	device_desc.idProduct = __constant_cpu_to_le16(get_product_id(dev));
	cdev->desc.idVendor = device_desc.idVendor;
	cdev->desc.idProduct = device_desc.idProduct;

	return 0;
}

static struct usb_composite_driver android_usb_driver = {
	.name		= "android_usb",
	.dev		= &device_desc,
	.strings	= dev_strings,
	.bind		= android_bind,
	.enable_function = android_enable_function,
};

void android_register_function(struct android_usb_function *f)
{
	struct android_dev *dev = _android_dev;

	printk(KERN_INFO "android_register_function %s\n", f->name);
	list_add_tail(&f->list, &_functions);

	if (dev && should_bind_functions(dev))
		bind_functions(dev);
}

int android_show_function(char *buf)
{
	unsigned length = 0;
	struct usb_function		*f;
	list_for_each_entry(f, &android_config_driver.functions, list) {
		if (!(f->disabled))
			length += sprintf(buf + length, "%s ", f->name);
	}
	length += sprintf(buf + length, "\n");
	return length;
}
EXPORT_SYMBOL(android_show_function);

static struct android_usb_product *get_product_by_configuration(char *func_list)
{
	struct android_dev *dev = _android_dev;
	struct usb_function f;
	struct android_usb_product *p = dev->products;
	char function_list[200], *func_list_curr;

	char *token;
	int i, token_counter;

	for (i = 0; i < _android_dev->num_products; i++, p++) {	/* scan all possible configuration */
		if(strlen(func_list)>200)	/*prevent*/
			return NULL;
		func_list_curr = strcpy(function_list, func_list);
		token_counter = 0;
		while ((token = strsep(&func_list_curr, ","))) {
			f.name = token;
			if (!product_has_function(p, &f))
				break;
			token_counter++;
		}
		/* finished parsing all func_list &
		func_list is not a subset of currently scanned product function list (i.e. func_list=acm,diag product functions=rndis,adb,acm,mass,diag) */
		if (!token && (token_counter == p->num_functions)) {
			//printk(KERN_ERR "%s: product#%d is found suitable, p->functions=%p\n", __func__, i, p->functions);
			return p;
		}
	}
	return NULL;
}
static void android_force_reset(struct android_dev *dev)
{
	struct usb_function *f;
	char enabled_func_list[200] = {0};
	struct android_usb_product *p;


	if (!dev) {
		printk(KERN_ERR "usb android_force_reset, invalid dev=%p\n", dev);
		return;
	}
	list_for_each_entry(f, &dev->config->functions, list) { /* scan all functions */
		if (!f->disabled) {
			if(strlen(f->name)>200) /*prevent*/
				return;
			if (!enabled_func_list[0])
				strcat(enabled_func_list, f->name);
			else {
				strcat(enabled_func_list, ",");
				strcat(enabled_func_list, f->name);
			}
		}
	}
	p = get_product_by_configuration(enabled_func_list);
	if (!p) {
		dev->cdev->gadget->is_all_functions_enabled = 0;
		printk(KERN_ERR "enabled_func_list: invalid usb function list \"%s\"", enabled_func_list);

	} else {
		//printk(KERN_ERR "enabled_func_list: applying new usb configuration \"%s\"", enabled_func_list);
		dev->cdev->gadget->is_all_functions_enabled = 1;
		dev->active_product = p;

		usb_composite_force_reset(dev->cdev);
	}
}

int android_switch_function(char *func_list)
{
	struct usb_function		*f;
	struct android_dev *dev = _android_dev;
	int product_id;
	if (dev->cdev==NULL || !func_list || !get_product_by_configuration(func_list)) {
		printk(KERN_ERR "illegal usb composite configuration string \"%s\"\n", func_list);
		return 0;
	}
		if (dev->cdev==NULL) {
		printk(KERN_ERR "illegal dev->cdev \n");
		return 0;
	}
	dev->cdev->desc.idVendor = __constant_cpu_to_le16(VENDOR_ID);
	/*dh0318.lee 110823 FOR_USING_DEVGURU_DRIVER*/
	dev->cdev->desc.bDeviceClass = 0xEF;//USB_CLASS_PER_INTERFACE;
	dev->cdev->desc.bDeviceSubClass = 0x02;//0x00;
	dev->cdev->desc.bDeviceProtocol = 0x01;//0x00;
	dev->cdev->desc.bcdDevice = __constant_cpu_to_le16(0x0400/*0x9999*/);
	printk(KERN_DEBUG "%s: composite configuration is %s\n", \
		__func__, func_list);
	list_for_each_entry(f, &android_config_driver.functions, list) {
		if (strstr(func_list, "acm") &&
			!strcmp(f->name, "acm")) {
			f->disabled = 0;
		} else if (strstr(func_list, "adb") && \
		!strcmp(f->name, "adb")) {
			f->disabled = 0;

			/* ADB doesn't function properly when vendor ID is
			not included in it list (see ADB host transport_usb.c),
			therefore using in this case
			vid,pid = 0x0bb4 */
#if !defined(CONFIG_MACH_GFORCE) && !defined(CONFIG_MACH_ALKON) && !defined(CONFIG_MACH_JETTA)
			dev->cdev->desc.idVendor = \
			__constant_cpu_to_le16(0x0BB4);
#endif
		} else if (strstr(func_list, "rndis") && \
		!strcmp(f->name, "rndis")) {
			f->disabled = 0;
#ifdef CONFIG_USB_ANDROID_RNDIS_WCEIS
			dev->cdev->desc.bDeviceClass = 0xEF;
#else
			dev->cdev->desc.bDeviceClass = USB_CLASS_COMM;
#endif
			dev->cdev->desc.bDeviceSubClass = 0x02;
			dev->cdev->desc.bDeviceProtocol = 0x01;
		} else if (strstr(func_list, "usb_mass_storage") &&
			!strcmp(f->name, "usb_mass_storage"))
			f->disabled = 0;
		else if (strstr(func_list, "diag") &&
			!strcmp(f->name, "diag")) {
			f->disabled = 0;
//#if defined(CONFIG_MACH_GFORCE) || defined(CONFIG_MACH_ALKON) || defined(CONFIG_MACH_JETTA)
			//dh0318.lee 101227 FOR_ACM_DIAG
			dev->cdev->desc.bDeviceClass = 0x02;
			dev->cdev->desc.bDeviceSubClass = 0x00;
			dev->cdev->desc.bDeviceProtocol = 0x00;		
//#endif			
		} else if (strstr(func_list, "mtp") &&
			!strcmp(f->name, "mtp"))
			f->disabled = 0;
		else
			f->disabled = 1;
	}

	product_id = get_product_id(dev);
	device_desc.idProduct = __constant_cpu_to_le16(product_id);
	if (dev->cdev)
		dev->cdev->desc.idProduct = device_desc.idProduct;
	android_force_reset(dev);
	return 0;
}
EXPORT_SYMBOL(android_switch_function);

static int calc_next_config(struct android_dev *dev, struct usb_function *f, int enable)
{ /*
Brief: this function implements simple state machine of usb configuration transition.
it's uses predefined hard coded transitions and should be adjusted by customers.
it handles modification in usb configuration done via Android GUI, as follows:
Enable Tethering (RNDIS): (1) Save last working configuration (2) switch to <RNDIS>
Disable Tethering: Restore previous working configuration.
Exceptions When Tethering is enabled, switch to any other usb configuration is blocked until Disable Tethering is triggered.

Enable Debugging (ADB):  Switch to <ADB, ACM, MASS>
Disable Debugging : Switch to <ACM, MASS>

Enable MASS:  If ADB is enabled switch to: <ADB, ACM, MASS> else switch to: <ACM, MASS>
Disable MASS: No actual switch/re-enumeration is done (it just disconnect the media from the usb_mass_storage).

return: 0 - active configuration was not modified
	1 - active configuration was  modified
	EINVAL - illegal request (e.g. see Exceptions in RNDIS section).
*/
	struct android_usb_product *p = NULL;
	struct usb_function *func;
	static int rndis_active;

	char *next_config = "";

	if (rndis_active && strcmp(f->name, "rndis"))
		return -EINVAL;

	printk(KERN_INFO "%s: f=%s, enable=%d\n", __func__, f->name, enable);
	/* RNDIS */
	if (!strcmp(f->name, "rndis")) {
		if (enable) {
			rndis_active = 1;
			next_config = "rndis";
			dev->prev_product = dev->active_product;
		} else { /*disable rndis*/
			rndis_active = 0;
			if (dev->prev_product)
				dev->active_product = dev->prev_product;
			else {
				printk(KERN_ERR "%s: illegal previous product value\n", __func__);
				/*default configuration in case previous configuration is fault
				(prevent stay in rndis only configuration)*/
				next_config = "adb,acm,usb_mass_storage";
			}
		}
	/* ADB */
	} else if (!strcmp(f->name, "adb")) {
		if (enable)
			next_config = "adb,acm,usb_mass_storage";
		else /*disable adb*/
			next_config = "acm,usb_mass_storage";
	/* MASS */
	} else if (!strcmp(f->name, "usb_mass_storage")) {
		struct usb_function adb_func;
		adb_func.name = "adb";
		/*check if ADB included in active confiugration*/
		if (product_has_function(dev->active_product, &adb_func))
			next_config = "adb,acm,usb_mass_storage";
		else
			next_config = "acm,usb_mass_storage";
	} else
		return 0;

	p = get_product_by_configuration(next_config);
	if (p)
		dev->active_product = p;

	/*enable all funcitons inlcuded in the new confiugration*/
	list_for_each_entry(func, &android_config_driver.functions, list) {
		if (product_has_function(dev->active_product, func))
			func->disabled = 0;
		else
			func->disabled = 1;
	}
	return 1;
}

void android_enable_function(struct usb_function *f, int enable)
{
	struct android_dev *dev = _android_dev;
	int disable = !enable;
	int res; /*protect from moving to any other configuration when tethering is enagled.*/

	res = calc_next_config(dev, f, enable);
	if (!res) { /*if not one of configuration trigged from Android GUI (not enable RNDIS, ADB or MASS)*/
		if (!product_has_function(dev->active_product, f)) {
			printk(KERN_INFO "android_enable_function: will not enable %s (usb function), it's not included in active usb composite configuration\n", f->name);
			return;
		}
		if (!!f->disabled == disable)
			return;
		usb_function_set_enabled(f, !disable);
	}
	if (res < 0) {
		printk(KERN_INFO "unresolved next USB configuration: %s %s function\n", (enable ? "enable" : "disable"), f->name);
		return;
	}

#ifdef CONFIG_USB_ANDROID_RNDIS
		if (!strcmp(f->name, "rndis")) {
			struct usb_function		*func;

			/* We need to specify the COMM class in the device descriptor
			 * if we are using RNDIS.
			 */
			if (enable){
#ifdef CONFIG_USB_ANDROID_RNDIS_WCEIS
				dev->cdev->desc.bDeviceClass = USB_CLASS_WIRELESS_CONTROLLER;
#else
				dev->cdev->desc.bDeviceClass = USB_CLASS_COMM;
#endif
				//dh0318.lee 111116 usb_uevent bConfigurationValue = 2 for the RNDIS configuration
			        android_config_driver.bConfigurationValue = 2;
			}else{
				dev->cdev->desc.bDeviceClass = USB_CLASS_PER_INTERFACE;
				//dh0318.lee 111116 usb_uevent
			       android_config_driver.bConfigurationValue = 1;
			}
	}
#endif
#ifdef CONFIG_USB_ANDROID_ACCESSORY
	if (!strcmp(f->name, "accessory") && enable) {
		struct usb_function		*func;

		/* disable everything else (and keep adb for now) */
			list_for_each_entry(func, &android_config_driver.functions, list) {
			if (strcmp(func->name, "accessory")
				&& strcmp(func->name, "adb")) {
				usb_function_set_enabled(func, 0);
				}
			}
		}
#endif

	device_desc.idVendor = __constant_cpu_to_le16(get_vendor_id(dev));
	device_desc.idProduct = __constant_cpu_to_le16(get_product_id(dev));
	if (dev->cdev) {
		dev->cdev->desc.idVendor = device_desc.idVendor;
			dev->cdev->desc.idProduct = device_desc.idProduct;
	}
	android_force_reset(dev);

}

static int android_probe(struct platform_device *pdev)
{
	struct android_usb_platform_data *pdata = pdev->dev.platform_data;
	struct android_dev *dev = _android_dev;

	printk(KERN_INFO "android_probe pdata: %p\n", pdata);

	if (pdata) {
		dev->products = pdata->products;
		dev->num_products = pdata->num_products;
		dev->functions = pdata->functions;
		dev->num_functions = pdata->num_functions;
		if (pdata->vendor_id) {
			dev->vendor_id = pdata->vendor_id;
			device_desc.idVendor =
				__constant_cpu_to_le16(pdata->vendor_id);
		}
		if (pdata->product_id) {
			dev->product_id = pdata->product_id;
			device_desc.idProduct =
				__constant_cpu_to_le16(pdata->product_id);
		}
		if (pdata->version)
			dev->version = pdata->version;

		if (pdata->product_name)
			strings_dev[STRING_PRODUCT_IDX].s = pdata->product_name;
		if (pdata->manufacturer_name)
			strings_dev[STRING_MANUFACTURER_IDX].s =
					pdata->manufacturer_name;
		if (pdata->serial_number)
			strings_dev[STRING_SERIAL_IDX].s = pdata->serial_number;
			/*prevent*/
		_android_dev->active_product = &pdata->products[0]; /*default active product is the first one */
	}
	
	return usb_composite_register(&android_usb_driver);
}

static struct platform_driver android_platform_driver = {
	.driver = { .name = "android_usb", },
	.probe = android_probe,
};

static int __init init(void)
{
	struct android_dev *dev;

	printk(KERN_INFO "android init\n");

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	/* set default values, which should be overridden by platform data */
	dev->product_id = PRODUCT_ID;
	_android_dev = dev;

	return platform_driver_register(&android_platform_driver);
}
module_init(init);

static void __exit cleanup(void)
{
	usb_composite_unregister(&android_usb_driver);
	platform_driver_unregister(&android_platform_driver);
	kfree(_android_dev);
	_android_dev = NULL;
}
module_exit(cleanup);
