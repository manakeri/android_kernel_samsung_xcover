
#ifndef __LINUX_USB_GADGET_PXA_COMP_H
#define __LINUX_USB_GADGET_PXA_COMP_H

#include <plat/pxausb_common.h>

#ifdef DEBUG
#define comp_print(args...) printk(args)
#else
#define comp_print(args...)
#endif

#define MULTI_P3	/* add iad descriptor */
#define UDC_DEFAULT_CONFIG 1	/* FIXME for rndis */
#define UDC_VENDOR_NUM 			0x1286
#define UDC_PRODUCT_NUM 		0x8002

#define MAX_CONFIG_LENGTH 256

struct pxa3xx_comp_ep {
	struct pxa3xx_comp_ep *next;
	unsigned config;
	unsigned interface;
//#ifdef CONFIG_USB_COMPOSITE xj del
	int log_ep_num;
	unsigned assigned_interface; /* actual interface number report to the host */
	struct gadget_driver_info *driver_info;/* pointer to gadget_driver_info */
//#endif
};

#ifdef CONFIG_USB_COMPOSITE
struct gadget_driver_info {
	struct gadget_driver_info *next;/* point to next gadget_driver_info */
	unsigned config;		/* configuration number used by the driver */
	unsigned assigned_intf_start;	/* the first assigned interface number */
	unsigned num_intfs;		/* total interface number for the driver */
	/*struct usb_descriptor_header *desc;	// point to configuration from gadget */
	/*int		desc_length;	// length of the configuration above */
#ifdef MULTI_P3
	unsigned char 	device_desc[18];/* device desc for the driver */
#endif
	unsigned char	config_desc[MAX_CONFIG_LENGTH];	/* configuration desc for th driver */
	unsigned char config_desc_hs[MAX_CONFIG_LENGTH];
	/*unsigned 	config_length; */

	struct	usb_gadget_driver *driver;	/* store the driver pointer of gadget */
	void 	*driver_data;		/* pointer to the driver private data */
	unsigned stopped;		/* driver disconnected or not */
};
#endif

struct pxa_comp_request {
	struct usb_request			req;
	struct list_head			queue;
};

struct pxa3xx_comp {
	int	driver_count;
	struct pxa3xx_comp_ep		*first_ep;
#ifdef CONFIG_USB_COMPOSITE
	struct gadget_driver_info 	*first_gadget;	/* head of a gadget_driver_info queue */
	struct gadget_driver_info 	*active_gadget;	/* currently active gadget */
	int	interface_count;
	/* FIXME, should be usb_request */
	struct pxa_comp_request		ep0_req;	/* include the usb_req to respond to the get_desc request, etc */
	struct t_str_id 		*str_id;
	int	rm_flag;
#endif
	enum ep0_state				ep0state;
	unsigned 				req_config;
	unsigned char				configs[MAX_CONFIG_LENGTH];
	unsigned				config_length;
	unsigned				configuration,
						interface,
						alternate;
#ifdef CONFIG_USB_OTG
	struct otg_transceiver      		*transceiver;
#endif
};


/* for controller driver */
extern int get_extra_descriptor(char *buffer, unsigned size, unsigned char type, void **ptr);
extern void get_fake_config(struct pxa3xx_comp *dev, struct usb_request *req, int spd);
extern void comp_val_init(struct pxa3xx_comp *dev);
extern void comp_set_ep(struct pxa3xx_comp *dev, int ep_num, int config, int interface);
extern struct pxa3xx_comp_ep *find_ep_num(struct pxa3xx_comp_ep *head, int num);
extern int comp_calc_config(struct pxa3xx_comp *dev, int ep_num);
extern int comp_calc_interface(struct pxa3xx_comp *dev, int ep_num);
extern int stop_cur_gadget(struct pxa3xx_comp *dev, struct usb_gadget *gadget,
		    struct usb_gadget_driver *driver);
extern int comp_check_driver(struct pxa3xx_comp *dev,
		      struct usb_gadget_driver *slf_drver,
		      struct usb_gadget_driver *driver);
extern void comp_register_driver(struct pxa3xx_comp *dev, struct usb_gadget *gadget, struct usb_gadget_driver *driver);
extern void comp_unregister_driver(struct pxa3xx_comp *dev, struct usb_gadget *gadget, 
		struct usb_gadget_driver **slf_drver, struct usb_gadget_driver *driver);
extern void comp_driver_suspend(struct pxa3xx_comp *dev, struct usb_gadget *gadget, struct usb_gadget_driver *driver);
extern void comp_driver_resume(struct pxa3xx_comp *dev, struct usb_gadget *gadget, struct usb_gadget_driver *driver);
extern int comp_change_config(struct pxa3xx_comp *dev, struct usb_gadget *gadget, struct usb_gadget_driver *driver, struct usb_ctrlrequest *req);
extern struct usb_gadget_driver *comp_change_interface(struct pxa3xx_comp *dev, int active_interface, struct usb_ctrlrequest *req, \
		struct usb_gadget *gadget, struct usb_gadget_driver *driver, int *ret);
extern void stop_gadget(struct pxa3xx_comp *dev, struct usb_gadget *gadget, struct usb_gadget_driver *driver);
extern void udc_stop(struct pxa3xx_comp *dev, struct usb_gadget *gadget, struct usb_gadget_driver *driver, int state);
extern int comp_ep0_req(struct pxa3xx_comp *dev, struct usb_gadget *gadget, struct usb_ep *ep0, struct usb_ctrlrequest *usb_req);
extern int comp_is_dev_busy(struct pxa3xx_comp *dev, struct usb_gadget_driver *driver);

extern int stop_udc(struct usb_gadget_driver *driver);
extern int set_eps(__u8 num_eps, struct usb_endpoint_descriptor *p_ep_desc, int len,
		                int config, int interface, int alt);
extern void udc_reinit(void);


#endif /* __LINUX_USB_GADGET_PXA_COMP_H */
