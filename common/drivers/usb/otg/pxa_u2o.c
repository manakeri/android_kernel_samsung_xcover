#ifdef CONFIG_USB_OTG
#include <linux/usb.h>
#include <linux/usb/ch9.h>
#include <linux/workqueue.h>
#include <linux/usb/otg.h>
#include <linux/usb/gadget.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/irq.h>
#include <linux/clk.h>
#include <linux/workqueue.h>

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

#include <linux/usb.h>
#include <linux/usb/ch9.h>
#include <linux/usb/cdc.h>
#include <linux/usb/gadget.h>
#include <linux/usb/otg.h>

#include <asm/irq.h>
#include <asm/uaccess.h>
#include <asm/uaccess.h>
#include <asm/mach-types.h>

#include <plat/pxausb_comp.h>
#include <plat/pxa_u2o.h>
#include <plat/pxa3xx_otg.h>
#include <plat/vbus.h>
#include "../gadget/mv/mvUsbDevApi.h"
#include "../gadget/mvUsb.h"

#ifdef CONFIG_PXA3xx_DVFM
#include <asm/arch/dvfm.h>
#endif

//#define DEBUG

#define DRIVER_VERSION "05-May-2008"
#define DRIVER_DESC "Marvell USB 2.0 OTG Controller"
static const char driver_name [] = "pxa-u2otg";
static const char driver_desc [] = DRIVER_DESC;


struct pxa_u2otg
{
    struct device               *dev;
    struct clk			*clk;
    struct pxa_u2o_mach_info    *mach;
    unsigned			regbase;
    unsigned			phybase;

    int (*vbus_detect) (void *func, int enable);
    int (*usbid_detect)(struct otg_transceiver *);
    int (*phy_init)    (unsigned base);
    int (*phy_deinit)  (unsigned base);
};


static struct pxa_u2otg		*u2otg = 0;
static struct work_struct	pxa_otgc_work;
static struct workqueue_struct 	*pxa_otgc_work_queue = 0;

static void pxa_u2otg_init(void);

extern struct mv_usb_dev *get_the_controller(void);
extern void mv_clock_enable(struct mv_usb_dev *, int);
extern void pxa3xx_otg_require_bus(int require);

/*----------------------------------------------------------------*/
#if 0
static int u2otg_proc_read(char *page, char **start, off_t off, int count,
		int *eof, void *_dev)
{
	char			*buf = page;
	char			*next = buf;
	unsigned		size = count;
	unsigned long		flags;
	int			t;

	local_irq_save(flags);
	t = scnprintf(next, size, DRIVER_DESC "\n"
		"%s\n\tU2xOTGSC = 0x%x\n",
		driver_name, u2o_get(u2otg->regbase, U2xOTGSC));
	size -= t;
	next += t;

	local_irq_restore(flags);
	*eof = 1;
	return count - size;
}

static int u2otg_proc_write(struct file *filp, const char *buffer,
		unsigned long count, void *data)
{
	char kbuf[8];
	int index;

	if (count >= 8)
		return -EINVAL;
	if (copy_from_user(kbuf, buffer, count))
		return -EFAULT;
	index = (int)simple_strtoul(kbuf, NULL, 10);

	switch (index) {
	default:
		break;
	}
	return count;
}
#endif

#define create_proc_files() \
	do {	struct proc_dir_entry *ent;\
		ent = create_proc_entry("driver/u2otg", 0, NULL);\
		if (ent) { \
			ent->data = u2otg_dev; \
			ent->read_proc = u2otg_proc_read; \
			ent->write_proc = u2otg_proc_write; \
		} \
	} while (0);
#define remove_proc_files() \
	remove_proc_entry("driver/u2otg", NULL)


/******************************************************************
 * Pxa9xx OTG transceiver functions
 ******************************************************************
 */
static int xceiv_mode = USB_OTG_OFF;
static int pxa9xx_otgx_get_mode(void)
{
	return xceiv_mode;
}
int otg_is_client(void)
{
	struct mv_usb_dev *mv_dev = get_the_controller();
	return (mv_dev->transceiver->default_a != 1);
}
int otg_is_host(void)
{
	struct mv_usb_dev *mv_dev = get_the_controller();
	int ret = mv_dev->transceiver->state == OTG_STATE_A_WAIT_BCON;
	struct pxa_otg *pxa_otg = container_of(mv_dev->transceiver, \
                                               struct pxa_otg, otg);

	ret |= mv_dev->transceiver->state == OTG_STATE_A_HOST;
	ret |= (mv_dev->transceiver->state == OTG_STATE_A_IDLE) && pxa_otg->otg_ctrl->a_bus_req;
	ret |= mv_dev->transceiver->state == OTG_STATE_A_WAIT_VFALL;
	ret |= mv_dev->transceiver->state == OTG_STATE_A_WAIT_VRISE;
	return ret;
}

/* Configures USB 2.0 OTG controller to the desired mode */
extern void pxa9xx_gadget_init(void);
extern void pxa9xx_ehci_set(int);

static void pxa_u2o_host_remove(struct work_struct *work)
{
	pxa9xx_ehci_set(0);
}
static void pxa_u2o_host_init(struct work_struct *work)
{
	pxa9xx_ehci_set(1);
}

static int pxa9xx_otgx_set_mode(int mode)
{
	int status = 0;

	pr_debug("%s set mode %d\n", __func__, mode);
	switch (mode) {

	case USB_OTG_LP:
		break;
	case USB_INT_OTG_CLIENT_DP:
		pxa_u2otg_init();
		u2otg->phy_init(u2otg->phybase);
		pxa_set_usbdev(USB_B_DEVICE);
		INIT_WORK(&pxa_otgc_work,
			(work_func_t)pxa_u2o_host_remove);
		queue_work(pxa_otgc_work_queue, &pxa_otgc_work);
		pxa9xx_gadget_init();
		break;

	case USB_INT_OTG_HOST_DP:
		pxa_u2otg_init();
		u2otg->phy_init(u2otg->phybase);
		pxa_set_usbdev(USB_A_DEVICE);
		INIT_WORK(&pxa_otgc_work,
			(work_func_t)pxa_u2o_host_init);
		queue_work(pxa_otgc_work_queue, &pxa_otgc_work);
		break;
	case USB_OTG_PRE_SYNCH:
		break;
	default:
		status = OTG_INVALID_PARAMETER;
		break;
	}

	xceiv_mode = mode;
	return status;
}


static void pxa9xx_otgx_init(void)
{
	printk("%s not implemented\n", __func__);
}

static int pxa9xx_otgx_dp_session(void)
{
	printk("%s not implemented\n", __func__);
	return 0;
}
static int pxa9xx_otgx_vbus_session(struct pxa_otg *pOtgHandle)
{
	printk("%s not implemented\n", __func__);
	return 0;
}
static int pxa9xx_otgx_check_b_hnp(void)
{
	printk("%s not implemented\n", __func__);
	return 0;
}

static int pxa9xx_otgx_check_vbus(void)
{
	return pxa_query_vbus();
}
static int pxa9xx_otgx_start_autoresume(void)
{
	printk("%s not implemented\n", __func__);
	return 0;
}
static int pxa9xx_otgx_drive_resume(void)
{
	printk("%s not implemented\n", __func__);
	return 0;
}

static enum otg_function pxa9xx_otgx_detect_default_func(void)
{
	struct mv_usb_dev *mv_dev;
#ifdef CONFIG_PXA95x
	if (pxa_query_usbid() & OTG_INT_IDR) {
		return OTG_B_DEVICE;
	}

	return OTG_A_DEVICE;
#endif
	mv_dev = get_the_controller();
	if (u2otg->usbid_detect) {
		if (u2otg->usbid_detect(mv_dev->transceiver)) {
			return OTG_B_DEVICE;
		}
		return OTG_A_DEVICE;
	}

	if (rely_on_vbus)
		mv_clock_enable(mv_dev, 1);

	/* enable OTG ID pullup register */
	u2o_set(u2otg->regbase, U2xOTGSC, U2xOTGSC_IDPU);
	mdelay(3);

	/* ID detect */
	if (u2o_get(u2otg->regbase, U2xOTGSC) & U2xOTGSC_ID) {
		if (rely_on_vbus)
			mv_clock_enable(mv_dev, 0);
		return OTG_B_DEVICE;
	}

	return OTG_A_DEVICE;
}

static struct otg_xceiv_ops pxa9xx_otg_xceiv_ops = {
	.otgx_get_mode                  = pxa9xx_otgx_get_mode,
	.otgx_set_mode                  = pxa9xx_otgx_set_mode,
	.otgx_init                      = pxa9xx_otgx_init,
	.otgx_detect_default_func	= pxa9xx_otgx_detect_default_func,
	.otgx_dp_session                = pxa9xx_otgx_dp_session,
	.otgx_vbus_session              = pxa9xx_otgx_vbus_session,
	.otgx_check_b_hnp               = pxa9xx_otgx_check_b_hnp,
	.otgx_check_vbus                = pxa9xx_otgx_check_vbus,
	.otgx_start_autoresume          = pxa9xx_otgx_start_autoresume,
	.otgx_drive_resume              = pxa9xx_otgx_drive_resume,
};

struct otg_xceiv_ops *init_pxa9xx_otg_xceiv_ops(void)
{
	return &pxa9xx_otg_xceiv_ops;
}

/******************************************************************
 * Pxa9xx OTG controller functions
 ******************************************************************
 */
static void pxa_otgc_interrupt_init(void)
{

}

static int pxa_otgc_reset(void)
{
	int delay, tmp;

	pr_debug("%s line %d\n", __func__, __LINE__);
	u2o_clear(u2otg->regbase, U2xUSBCMD, U2xUSBCMD_RS);
	u2o_set(u2otg->regbase, U2xUSBCMD, U2xUSBCMD_RST);
	delay = 10000;
	while ((u2o_get(u2otg->regbase, U2xUSBCMD) & U2xUSBCMD_RST) && delay--);
	if (delay <= 0) {
		printk("%s reset timeout\n", __func__);
	}

	u2o_write(u2otg->regbase, U2xUSBINTR, 0);
	u2o_clear(u2otg->regbase, U2xUSBMODE, 0x3);
	tmp = u2o_get(u2otg->regbase, U2xUSBSTS);
	u2o_write(u2otg->regbase, U2xUSBSTS, tmp);
	pr_debug("%s USBCMD %x USBSTS %x USBINTR %x\n",
		__func__, u2o_get(u2otg->regbase, U2xUSBCMD),
		u2o_get(u2otg->regbase, U2xUSBSTS), u2o_get(u2otg->regbase, U2xUSBINTR));
	return 0;
}

static int pxa_otgc_interrupt_handle(struct pxa_otg *pOtgHandle)
{
	unsigned long flags;
	u8 int_type = OTG_INT_INIT;

	if (!u2otg) {
		printk("u2otg not inited yet\n");
		return OTG_INT_INIT;
	}
	pr_debug("%s line %d\n", __func__, __LINE__);

	local_irq_save(flags);
	pxa_otgc_reset();
#ifdef CONFIG_CPU_MMP3
	if (u2otg->vbus_detect) {
		int_type |= pxa_query_session();
		int_type |= pxa_query_usbid();
	}
	else {
		u32 otgsc = u2o_get(u2otg->regbase, U2xOTGSC);
		if (otgsc & U2xOTGSC_AVV)
			int_type |= OTG_INT_RVV;
		else
			int_type |= OTG_INT_FVV;

		if (otgsc & U2xOTGSC_ID)
			int_type |= OTG_INT_IDR;
		else
			int_type |= OTG_INT_IDF;
	}
#else
	int_type |= pxa_query_session();
	int_type |= pxa_query_usbid();
#endif
	local_irq_restore(flags);

	pr_debug("%s, int_type %x\n", __func__, int_type);
	return int_type;
}

static void pxa_otgc_init_gadget(void)
{
	u32 tmp;

/*	if (pxa_query_vbus() == VBUS_LOW) {
		printk("%s  vbus low, return\n", __func__);
		return;
	}*/
	u2o_clear(u2otg->regbase, U2xOTGSC, U2xOTGSC_IE_MASK);
	tmp = u2o_get(u2otg->regbase, U2xOTGSC);
	u2o_set(u2otg->regbase, U2xOTGSC, tmp);
}

static void pxa_u2otg_init(void)
{
#ifndef CONFIG_CPU_MMP3
	if (pxa_query_vbus() == VBUS_LOW) {
		printk("%s  vbus low, return\n", __func__);
		return;
	}
#endif
	pr_debug("%s U2xUSBCMD %x U2xOTGSC %x\n", __func__,
		u2o_get(u2otg->regbase, U2xUSBCMD),
		u2o_get(u2otg->regbase, U2xOTGSC));
	pxa_otgc_reset();

	if (u2otg->vbus_detect) {
		u2o_set(u2otg->regbase, U2xOTGSC, U2xOTGSC_IDIE | U2xOTGSC_IDPU);
	} else {
		u2o_set(u2otg->regbase, U2xOTGSC, U2xOTGSC_IDIE | U2xOTGSC_IDPU |
			U2xOTGSC_BSVIE | U2xOTGSC_BSEIE |
			U2xOTGSC_AVVIE | U2xOTGSC_ASVIE);
	}
}

static int pxa_otgc_init(void *_data)
{
	struct pxa_usb_plat_info *info = _data;

	if (!u2otg) {
		u2otg = kmalloc(sizeof(struct pxa_u2otg), GFP_ATOMIC);
		u2otg->regbase = info->regbase;
		u2otg->phybase = info->phybase;
		u2otg->phy_init = info->phy_init;
		u2otg->phy_deinit = info->phy_deinit;
		u2otg->usbid_detect = info->usbid_detect;
		u2otg->vbus_detect = info->vbus_detect;
	}

	printk("u2otg->regbase 0x%x\n", u2otg->regbase);
	pxa_otgc_init_gadget();
	pxa_u2otg_init();

	if(!pxa_otgc_work_queue)
		pxa_otgc_work_queue = create_workqueue("pxa_otgc_work_queue");

	/* create_proc_files(); */
	return 0;
}

static void pxa_otgc_deinit(void)
{
	printk("%s\n", __func__);
}

static struct otg_ctrl_ops pxa_otg_ctrl_ops = {
	.otgc_init              = pxa_otgc_init,
	.otgc_deinit            = pxa_otgc_deinit,
	.otgc_interrupt_init    = pxa_otgc_interrupt_init,
	.otgc_interrupt_handle  = pxa_otgc_interrupt_handle,
	.otgc_init_gadget       = pxa_otgc_init_gadget,
};

struct  otg_ctrl_ops *init_pxa9xx_otg_ctrl_ops(void)
{
	return &pxa_otg_ctrl_ops;
}

#endif /* CONFIG_CPU_PXA935 */
