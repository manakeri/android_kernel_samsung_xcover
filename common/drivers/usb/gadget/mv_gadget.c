/*******************************************************************************
Copyright (C) Marvell International Ltd. and its affiliates

This software file (the "File") is owned and distributed by Marvell 
International Ltd. and/or its affiliates ("Marvell") under the following
alternative licensing terms.  Once you have made an election to distribute the
File under one of the following license alternatives, please (i) delete this
introductory statement regarding license alternatives, (ii) delete the two
license alternatives that you have not elected to use and (iii) preserve the
Marvell copyright notice above.

********************************************************************************
Marvell Commercial License Option

If you received this File from Marvell and you have entered into a commercial
license agreement (a "Commercial License") with Marvell, the File is licensed
to you under the terms of the applicable Commercial License.

********************************************************************************
Marvell GPL License Option

If you received this File from Marvell, you may opt to use, redistribute and/or 
modify this File in accordance with the terms and conditions of the General 
Public License Version 2, June 1991 (the "GPL License"), a copy of which is 
available along with the File in the license.txt file or by writing to the Free 
Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 or 
on the worldwide web at http://www.gnu.org/licenses/gpl.txt. 

THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE IMPLIED 
WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY 
DISCLAIMED.  The GPL License provides additional details about this warranty 
disclaimer.
********************************************************************************
Marvell BSD License Option

If you received this File from Marvell, you may opt to use, redistribute and/or 
modify this File under the following licensing terms. 
Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    *   Redistributions of source code must retain the above copyright notice,
        this list of conditions and the following disclaimer. 

    *   Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution. 

    *   Neither the name of Marvell nor the names of its contributors may be 
        used to endorse or promote products derived from this software without 
        specific prior written permission. 
    
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND 
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR 
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON 
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*******************************************************************************/

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
#include <linux/workqueue.h>
#include <linux/mm.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/clk.h>
#ifdef CONFIG_ANDROID
#include <linux/wakelock.h>
#endif
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
#include <linux/usb/composite.h>
#include <linux/mfd/88pm860x.h>

#include "mv/mvUsbDevApi.h"
#include "mv/mvUsbCh9.h"
#include <mach/hardware.h>
#include <mach/regs-ost.h>
#include <plat/vbus.h>
#include <plat/pxausb_comp.h>
#include <plat/pxa3xx_otg.h>
#include <plat/pxa_u2o.h>
#include "mvUsb.h"

#if defined(CONFIG_MV645xx) || defined(CONFIG_MV646xx)
#   include "marvell_pic.h"
#endif /* CONFIG_MV645xx */

#ifdef CONFIG_SMP
#define MV_SPIN_LOCK_IRQSAVE(spinlock,flags) \
if(!in_interrupt())  \
spin_lock_irqsave(spinlock, flags)

#define MV_SPIN_UNLOCK_IRQRESTORE(spinlock,flags) \
if(!in_interrupt())  \
spin_unlock_irqrestore(spinlock, flags)

#else /* CONFIG_SMP */

#define MV_SPIN_LOCK_IRQSAVE(spinlock,flags) spin_lock_irqsave(spinlock, flags)
#define MV_SPIN_UNLOCK_IRQRESTORE(spinlock,flags) spin_unlock_irqrestore(spinlock, flags)

#endif /* CONFIG_SMP */

#define mvOsMalloc(buf)	kmalloc(buf, GFP_KERNEL)
#define mvOsFree	kfree

#define MV_USB_VERSION	0
#define MV_BOARD_MAX_USB_IF	2

#define L2_CACHE_LINE_SIZE	32
#define L2_CACHE_ADDR_MASK	((L2_CACHE_LINE_SIZE << 1) - 1)

#define MAX_DELAY_BETWEEN_ENUMERATIONS          220   /* in msec */
#define OSCR4_TICKS_PER_MS                      (32768 / 1000)

/*  
 *  Enable/Disable Streaming mode for USB Core
 */
/*#if (defined(CONFIG_MV88F6082) || defined(CONFIG_MV645xx) || defined(USB_UNDERRUN_WA))*/
#if (MV_USB_VERSION >= 1) || defined(USB_UNDERRUN_WA)
static int streaming = 1;  
#else
static int streaming = 0;  
#endif
module_param_named(streaming, streaming, int, S_IRUGO);
MODULE_PARM_DESC(streaming, "0 - Streaming Disable, 1 - Streaming Enable");

static struct mv_usb_dev*   the_controllers[MV_BOARD_MAX_USB_IF] = {NULL, NULL};

unsigned long temp_remove_count = 0;
unsigned long send_not_empty = 0;
int IRQ_USB_CTRL[1];

#define U2xPORTSC_DP	(1<<11)
#define U2xPORTSC_DM	(1<<10)
#define CHARGER_TIMER_DELAY 300
static struct timer_list	charger_timer;
static struct work_struct 	set_charger_type_work;
static void charger_timer_event(unsigned long data);
/*identifies if the last enumeration succeeded.*/
enum enum_result  enumeration_result = ENUMERATION_START;
EXPORT_SYMBOL(enumeration_result);
static int reset_time_stamp;

/***************************************************************************
 * PMIC related operations
 */
#include <mach/dvfm.h>

#define ENABLE_CABLE_DETECT
#define USB_TIMER_TIMEOUT (3*HZ)

extern int pm860x_set_vbus(int enable, int srp);
int connected;
static int conn_check = 1;
int rely_on_vbus;

#ifdef CONFIG_PXA95x
static int dvfm_dev_idx;
#elif defined(CONFIG_ANDROID)
static struct wake_lock idle_lock;
#endif
#ifdef CONFIG_ANDROID
static struct wake_lock suspend_lock;
#endif

/*is_enabled insure that we call clk_enable
only if the clock was previously disabled*/
unsigned int is_enabled = FALSE;
#ifdef CONFIG_ARCH_PXA
void mv_clock_enable(struct mv_usb_dev *dev, int enable)
{
        unsigned long flags;
        local_irq_save(flags);
        if (enable && (!is_enabled)) {
                is_enabled = TRUE;
                clk_enable(dev->bus_clk);
                clk_enable(dev->island_clk);
                clk_enable(dev->clk);
        } else if ((!enable) && is_enabled) {
                is_enabled = FALSE;
                clk_disable(dev->clk);
                clk_disable(dev->island_clk);
                clk_disable(dev->bus_clk);
        }
        local_irq_restore(flags);
}
#else
void mv_clock_enable(struct mv_usb_dev *dev, int enable)
{
        static int enabled;

        pr_debug("%s enable %d enabled %d\n", __func__, enable, enabled);
        if (enable == enabled)
                return;

        if (enable) {
                clk_enable(dev->clk);
                if (dev->info && dev->info->phy_init)
                        dev->info->phy_init(dev->info->phybase);
        } else {
/* Now we use OTGSC to detect USB cable, so we can not disable the clock*/
#ifndef CONFIG_CPU_MMP3
                clk_disable(dev->clk);
                if (dev->info && dev->info->phy_deinit)
                        dev->info->phy_deinit(dev->info->phybase);
#endif
        }
        enabled = enable;
}
#endif	/* CONFIG_ARCH_PXA */

/* detect USB cable attach and detach by PMIC
 * 1 -- cable attached; 0 -- cable detached
 */
int is_cable_attached(void)
{
	struct mv_usb_dev *dev = the_controllers[0];

	if (!dev->driver)
		connected = 0;
// VBUS #ifdef CONFIG_PXA_VBUS
	if (pxa_query_vbus() & VBUS_HIGH)
 		connected = 1;
	else
		connected = 0;
// #endif

	return connected;
}


static int distinguish_ac_charger_type(void)
{
	struct mv_usb_dev *mv_dev = the_controllers[0];
	unsigned int val=0;

	val = u2o_get(mv_dev->regbase, U2xPORTSC);
	val &=U2xPORTSC_DP|U2xPORTSC_DM;
	if((val&U2xPORTSC_DP)&&(val&U2xPORTSC_DM))
		return AC_STANDARD_CHARGER;
	else
		return AC_OTHER_CHARGER;
}

static void do_set_charger_type(struct work_struct *work)
{
	struct mv_usb_dev *mv_dev = the_controllers[0];
	enum enum_charger_type type = USB_CHARGER;
	if (enumeration_result == ENUMERATION_500MA) {
		enumeration_result = ENUMERATION_START;
		printk(KERN_ALERT \
			"USB charger-mv_gadget:do_set_charger_type\n");
		if(mv_dev->set_charger_type)
			mv_dev->set_charger_type(USB_CHARGER);
	} else {
		enumeration_result = ENUMERATION_START;
		printk(KERN_ALERT \
			"AC charger-mv_gadget:do_set_charger_type\n");
		type = distinguish_ac_charger_type();
		/*AC_STANDARD_CHARGER in our hand is 500ma,AC_OTHER_CHARGER is 1000ma*/
		if(type == AC_STANDARD_CHARGER){
			if(mv_dev->set_charger_type)
				mv_dev->set_charger_type(AC_STANDARD_CHARGER);
		} else if(type == AC_OTHER_CHARGER){
			if(mv_dev->set_charger_type)
				mv_dev->set_charger_type(AC_OTHER_CHARGER);
		}
	}
}

static void charger_timer_event(unsigned long data)
{
	schedule_work(&set_charger_type_work);
}

#ifdef ENABLE_CABLE_DETECT
static int is_set_conf(SETUP_STRUCT_PTR ctrl_req)
{
	if ((ctrl_req->REQUESTTYPE & REQ_TYPE_MASK) == REQ_TYPE_STANDARD) {
	    if (ctrl_req->REQUEST == REQ_SET_CONFIGURATION) {
		return 1;
	    }
	}
	return 0;
}

static void uevent_worker(struct work_struct *work)
{
	struct mv_usb_dev *dev = the_controllers[0];

	if (connected) 
		kobject_uevent(&dev->dev->kobj, KOBJ_ADD);
	else 
		kobject_uevent(&dev->dev->kobj, KOBJ_REMOVE);
}

static void mv_usb_clock_disable(unsigned long data)
{
	struct mv_usb_dev *dev = the_controllers[0];

	if (connected)
		return;

	pr_info("USB: no cable connected\n");
	mv_clock_enable(dev, 0);
	if (dev->info && dev->info->set_power)
		dev->info->set_power(0);
}

void mv_usb_device_start(_usb_device_handle handle)
{
	struct mv_usb_dev *dev = the_controllers[0];

	if (dev->gadget.is_all_functions_enabled)
		_usb_device_start(handle);
}

extern void _usb_dci_vusb20_chip_initialize(_usb_device_handle);
static uint_8 mv_usb_start_ep0(struct mv_usb_dev *mv_dev);
static void mv_usb_connect_change(int status)
{
	struct mv_usb_dev *dev = the_controllers[0];
	struct otg_transceiver *otg;
	struct pxa_otg *pxa_otg;
	struct usb_gadget gadget;
	struct usb_composite_dev *cdev;
	unsigned long flags;

	if (!dev || !dev->driver)
		return;

	gadget = dev->gadget;
	cdev = get_gadget_data(&gadget);
	
	pr_debug("%s status %d\n", __func__, status);
	if (status & (VBUS_HIGH | VBUS_SRP)) {

		connected = 1;
		
		if (timer_pending(&charger_timer) == 0)
			mod_timer(&charger_timer, jiffies + 600);

#ifdef CONFIG_PXA95x
		dvfm_disable_lowpower(dvfm_dev_idx);
#elif defined(CONFIG_ANDROID)
		wake_lock(&idle_lock);
#endif
#ifdef CONFIG_ANDROID
		pr_info("%s: charger insertion, request wakelock forever\n", __func__);
		wake_lock(&suspend_lock);
#endif

		mv_clock_enable(dev, 1);
		if (dev->info && dev->info->set_power)
			dev->info->set_power(1);

#ifdef CONFIG_USB_OTG
		/* ID detect */
		otg = dev->transceiver;
		pxa_otg = container_of(otg, struct pxa_otg, otg);
#ifndef CONFIG_CPU_MMP3 
//dh0318.lee 110714 TODO:pm860x_set_vbus_output function is not defined under DIALOG PMIC. need to fix.
//dh0318.lee 110720 Below code is to supply power if the USB-OTG function is supported. we don't need it at this time.
#if !defined(CONFIG_MACH_JETTA)
		if (pxa_otg->xceiv_ops->otgx_detect_default_func() == \
								OTG_A_DEVICE)								
			pm860x_set_vbus_output(1);
		else
		{
			if (timer_pending(&charger_timer) == 0)
				mod_timer(&charger_timer, jiffies + CHARGER_TIMER_DELAY);

			pm860x_set_vbus_output(0);
		}
#endif //CONFIG_MACH_JETTA			
#endif
       		if (dev->transceiver) {
			pxa3xx_otg_require_bus(USBOTG_VBUS_VALID);
			/* fix the issue of composite mode, when insert 2nd
			 * module, it won't init usb in require_bus, init
			 * usb client mode here instead */
			if (dev->transceiver->state
				== OTG_STATE_B_PERIPHERAL) {
				mv_usb_device_start(dev->mv_usb_handle);
				mv_usb_start_ep0(dev);
			}
		} else 
#endif
      		{
			udc_stop(&dev->cdev, &dev->gadget, dev->driver, 1);
       			_usb_dci_vusb20_chip_initialize(dev->mv_usb_handle);

		/* start device here */
			mv_usb_device_start(dev->mv_usb_handle);
			mv_usb_start_ep0(dev);
		}
		//dh0318.lee 111116 usb_uevent send the cable connection UEVENT to the platform layer
		MV_SPIN_LOCK_IRQSAVE(&dev->lock, flags);
		cdev->connected = 1;
		schedule_work(&cdev->switch_work);
		MV_SPIN_UNLOCK_IRQRESTORE(&dev->lock, flags);

	} else { /* Cable detached */	

		connected = 0;

		enumeration_result = ENUMERATION_START;
		/*check pending before del 0-is not , 1-it is*/
		if (timer_pending(&charger_timer) == 1)
			del_timer(&charger_timer);

		schedule_work(&dev->work);
#ifdef CONFIG_USB_OTG
       		if (dev->transceiver) {
			pxa3xx_otg_require_bus(0);
		} else 
#endif
		if (!rely_on_vbus) {
			udc_stop(&dev->cdev, &dev->gadget, dev->driver, 1);
			_usb_device_stop(dev->mv_usb_handle);
		}

		/* add some delay to avoid some thread that may access
		 * usb stuff, like hub_thread when OTG enabled */
		mod_timer(&dev->timer, jiffies + USB_TIMER_TIMEOUT);
#ifdef CONFIG_PXA95x
		dvfm_enable_lowpower(dvfm_dev_idx);
#elif defined(CONFIG_ANDROID)
		wake_unlock(&idle_lock);
#endif
#ifdef CONFIG_ANDROID
		/* taking a wakelock since we want to explicitly inform
		   the user about this change in battery/charging status */
		if (wake_lock_active(&suspend_lock)) {
			pr_info("%s: clean previous wakelock request\n", __func__);
			wake_unlock(&suspend_lock);
		}
			pr_info("%s: charger removing or battery event - request a wake lock for time\n", __func__);
		wake_lock_timeout(&suspend_lock, 15 * HZ);
#endif
		//dh0318.lee 111116 usb_uevent  send the cable disconnection UEVENT to the platform layer
		MV_SPIN_LOCK_IRQSAVE(&dev->lock, flags);
		cdev->connected = 0;
		schedule_work(&cdev->switch_work);
		MV_SPIN_UNLOCK_IRQRESTORE(&dev->lock, flags);
	}
}

void mv_usb_vbus_change(int status)
{
	mv_usb_connect_change(status);
}
EXPORT_SYMBOL (mv_usb_vbus_change);
#else
static void mv_usb_vbus_change(int status) {}
#endif



/***************************************************************************/

#ifdef DEBUG
int mv_debug = 1;
#else
int mv_debug = 0;
#endif

static void mvOsPrintf(const char *fmt, ...)
{
#if 1
	va_list args;
	int r;

	if (mv_debug) {
        	va_start(args, fmt);
        	r = vprintk(fmt, args);
        	va_end(args);
	}
#endif
}

int mvCtrlUsbMaxGet(void)
{
	return 1;
}

#if defined(USB_UNDERRUN_WA)
#include "mvIdma.h" 

extern int              mv_idma_usage_get(int* free_map);
extern unsigned char*   mv_sram_usage_get(int* sram_size_ptr);


#define USB_IDMA_CTRL_LOW_VALUE       ICCLR_DST_BURST_LIM_128BYTE   \
                                    | ICCLR_SRC_BURST_LIM_128BYTE   \
                                    | ICCLR_BLOCK_MODE              \
                                    | ICCLR_DESC_MODE_16M

/*  
 *  0..3   - use specifed IDMA engine. If the engine is busy - find free one.
 *  Other  - don't use IDMA (use memcpy instead)
 */
static int idma = 1;  
module_param_named(idma, idma, int, S_IRUGO);
MODULE_PARM_DESC(idma, "IDMA engine used for copy from DRAM to SRAM [0..3]");

/*  
 */
static int wa_sram_parts  = 2;  
module_param_named(wa_sram_parts, wa_sram_parts, int, S_IRUGO);
MODULE_PARM_DESC(wa_sram_parts, "");

static int wa_sram_descr  = 1;  
module_param_named(wa_sram_descr, wa_sram_descr, int, S_IRUGO);
MODULE_PARM_DESC(wa_sram_descr, "");

/*  
 */
static int wa_threshold = 64;  
module_param_named(wa_threshold, wa_threshold, int, S_IRUGO);
MODULE_PARM_DESC(wa_threshold, "");


static char*    sramBase = (char*)NULL;
static int      sramSize = 0;

u32     mvUsbSramGet(u32* pSize)
{
    char*   pBuf;

    /* Align address to 64 bytes */
    pBuf = (char*)MV_ALIGN_UP((u32)sramBase, 64);

    if(pSize != NULL)
    {
        *pSize = sramSize - (pBuf - sramBase);
    }

    mvOsPrintf("mvUsbSramGet: Base=%p (%p), Size=%d (%d)\n", 
                sramBase, pBuf, sramSize, *pSize);

    return (u32)pBuf;
}

void        mvUsbIdmaToSramCopy(void* sram_buf, void* src_buf, unsigned int size)
{
    unsigned long phys_addr;


    mvOsPrintf("IdmaToSramCopy: idma=%d, sram=%p, src=%p (0x%x)\n", 
                    usbWaIdma, sram_buf, src_buf, phys_addr);

    if( (idma >= 0) && (idma < MV_IDMA_MAX_CHAN) )
    {
        phys_addr = pci_map_single(NULL, src_buf, size, PCI_DMA_TODEVICE );

        /* !!!! SRAM Uncached */
        /*mvOsCacheInvalidate(NULL, sram_buf, size);*/

        mvDmaTransfer(idma, (MV_U32)phys_addr, 
                      (MV_U32)sram_buf, size, 0);

        /* Wait until copy is finished */
        while( mvDmaStateGet(idma) != MV_IDLE );
    }
    else
    {
         mvOsPrintf("usbWA: copy to SRAM %d bytes: \n", size); 
        memcpy(sram_buf, src_buf, size);
        /* !!!! SRAM Uncached */
        /*mvOsCacheFlush(NULL, sram_buf, size);*/
    }
}

USB_WA_FUNCS    usbWaFuncs = 
{
    mvUsbSramGet,
    mvUsbIdmaToSramCopy
};

int mv_usb_find_idma_engine(int idma_no)
{
    int idma, free = 0;
    int free_map[MV_IDMA_MAX_CHAN];

    if( (idma_no < 0) || (idma_no >= MV_IDMA_MAX_CHAN) )
    {
        mvOsPrintf("Wrong IDMA number (%d): Valid range [0..%d]\n", 
                    idma_no, (MV_IDMA_MAX_CHAN-1) );
        return -1;
    }
    free = mv_idma_usage_get(free_map);
    if(free == 0)
    {
        mvOsPrintf("No free IDMAs for USB Underrun WA: use memcpy\n");
        return -1;
    }
    /* First of all check user idma_no */
    if(free_map[idma_no] != 0)
        return idma_no;

    /* User idma_no is Busy. Look for free IDMA engine */
    for(idma=0; idma<MV_IDMA_MAX_CHAN; idma++)
    {
        if(free_map[idma] != 0)
            break;
    }
    mvOsPrintf("IDMA engine #%d is Busy. Use IDMA engine #%d instead\n", idma_no, idma);
    return idma;
}
#endif /* USB_UNDERRUN_WA */

#ifdef DEBUG
#define DBGMSG(fmt,args...)    \
             mvOsPrintf(fmt , ## args)
#else
#   define DBGMSG(fmt,args...)
#endif /* DEBUG */

#define DRIVER_VERSION  "05-July-2006"
#define DRIVER_DESC "Marvell Gadget USB Peripheral Controller"

void    mv_usb_show(struct mv_usb_dev* mv_dev, unsigned int mode);

void mv_udelay(int delay)
{
	udelay(delay);
}

extern void   _usb_dci_vusb20_isr(pointer);

static void* mvUsbMalloc(unsigned int size)
{
    return kmalloc(size,GFP_ATOMIC);
}

static void mvUsbFree(void* buf)
{
    return kfree(buf);
}

void usb_dump(void)
{
	struct mv_usb_dev *mv_dev = the_controllers[0];
	unsigned offset, base;

	base = mv_dev->phybase;
	printk("UTMI regs\n");
	for (offset=4;offset < 0x40;offset+=4) {
		printk("\t%2X: 0x%8X\n", offset, u2o_get(base, offset));
	}

	base = mv_dev->regbase;
	printk("\nU2O regs\n");
	for (offset=0;offset < 0x1c4;offset+=4) {
		printk("\t %2X: 0x%8X\n", offset, u2o_get(base, offset));
	}
}

static void MV_REG_WRITE(unsigned offset, unsigned int value)
{
	struct mv_usb_dev *mv_dev = the_controllers[0];
	unsigned int base = mv_dev->regbase;
	unsigned long flags;
	unsigned int addr;
	
	addr = (unsigned int)(base + offset);
	local_irq_save(flags);
	writel(value, addr);
	__raw_readl(addr);
	local_irq_restore(flags);
}

static unsigned int MV_REG_READ(unsigned int offset)
{
	struct mv_usb_dev *mv_dev = the_controllers[0];
	unsigned int base = mv_dev->regbase;
	unsigned int addr;

	addr = (unsigned int)(base + offset);
	return __raw_readl(addr);
}

static int MV_BIT_CHECK(unsigned a, unsigned b)
{
	if (a & (1<<b))
		return 1;

	return 0; 
}

static void* mvUsbIoUncachedMalloc( void* pDev, unsigned int size, unsigned int alignment, unsigned long* pPhyAddr )
{
#if defined(USB_UNDERRUN_WA)
    if(wa_sram_descr != 0)
    {
        char*   pBuf;

        pBuf = (char*)MV_ALIGN_UP((MV_U32)sramBase, alignment);
        size += (pBuf - sramBase);
        if( (sramSize < size) )
        {
            mvOsPrintf("SRAM malloc failed: Required %d bytes - Free %d bytes\n", 
                        size, sramSize);
            return NULL;
        }
        if(pPhyAddr != NULL)
            *pPhyAddr = (MV_ULONG)sramBase;

        mvOsPrintf("usbUncachedMalloc: , pBuf=%p (%p), size=%d, align=%d\n", 
                sramBase, pBuf, size, alignment);

        pBuf = sramBase;

        sramSize -= size;
        sramBase += size;

        return pBuf;
    }
#endif 

#ifdef CONFIG_PCI
    return pci_alloc_consistent( pDev, size+alignment, (dma_addr_t *)pPhyAddr );
#else	
    return dma_alloc_coherent(pDev, size+alignment, (dma_addr_t *)pPhyAddr, GFP_KERNEL);
#endif
}
static void mvUsbIoUncachedFree( void* pDev, unsigned int size, unsigned long phyAddr, void* pVirtAddr )
{
#if defined(USB_UNDERRUN_WA)
    if(wa_sram_descr != 0)
    {
        return;
    }
#endif 
#ifdef CONFIG_PCI
    return pci_free_consistent( pDev, size, pVirtAddr, (dma_addr_t)phyAddr );
#else
    return dma_free_coherent( pDev, size, pVirtAddr, (dma_addr_t)phyAddr );
#endif
} 

static MV_ULONG mvUsbCacheInvalidate( void* pDev, void* p, int size )
{
#if defined(USB_UNDERRUN_WA)
    if( ((char*)p >= sramBase) && 
        ((char*)p < (sramBase + sramSize)) )
        return (unsigned long)p;
#endif

#ifdef CONFIG_PCI
    return pci_map_single( pDev, p, size, PCI_DMA_FROMDEVICE );
#else
    return dma_map_single(pDev, p, size, DMA_FROM_DEVICE);
#endif
}

static MV_ULONG mvUsbCacheFlush( void* pDev, void* p, int size )
{
#if defined(USB_UNDERRUN_WA)
    if( ((char*)p >= sramBase) && 
        ((char*)p < (sramBase + sramSize)) )
        return (unsigned long)p;
#endif

#ifdef CONFIG_PCI
    return pci_map_single( pDev, p, size, PCI_DMA_TODEVICE );
#else
    return dma_map_single( pDev, p, size, DMA_TO_DEVICE );
#endif
}

static unsigned long mvUsbVirtToPhys(void* pDev, void* pVirtAddr)
{
#if defined(USB_UNDERRUN_WA)
    if( ((char*)pVirtAddr >= sramBase) && 
        ((char*)pVirtAddr < (sramBase + sramSize)) )
        return (unsigned long)pVirtAddr;
#endif 

    return virt_to_phys(pVirtAddr);
}

static void   usbDevResetComplete(int devNo)
{
    MV_U32  regVal; 

    regVal = MV_USB_CORE_MODE_DEVICE | MV_USB_CORE_SETUP_LOCK_DISABLE_MASK;
    if(streaming == 0)    
        regVal |= MV_USB_CORE_STREAM_DISABLE_MASK; 

    /* Set USB_MODE register */
    MV_REG_WRITE(MV_USB_CORE_MODE_REG(devNo), regVal); 
}

static MV_U32   mvUsbGetCapRegAddr(int devNo)
{
	struct mv_usb_dev *mv_dev = the_controllers[0];
	return (unsigned int)((unsigned)mv_dev->regbase + 0x100);
//	return &(MV_USB_CORE_CAP_LENGTH_REG(devNo));
}

static unsigned long flags;
int mv_lock(void)
{
	local_irq_save(flags);
	return flags;
}
void mv_unlock(int flags)
{
	local_irq_restore((unsigned long)flags);
}

USB_IMPORT_FUNCS    usbImportFuncs =
{
#ifdef DEBUG
	.bspPrintf =            mvOsPrintf,
#else
//	.bspPrintf =            mvOsPrintf,
#endif
	.bspSprintf =           sprintf, 
	.bspUncachedMalloc = 	mvUsbIoUncachedMalloc,
	.bspUncachedFree = 	mvUsbIoUncachedFree,
	.bspMalloc =            mvUsbMalloc,
	.bspFree =              mvUsbFree,
	.bspMemset =            memset,
	.bspMemcpy =            memcpy,
	.bspCacheFlush = 	mvUsbCacheFlush,
	.bspCacheInv =		mvUsbCacheInvalidate,
	.bspVirtToPhys =	mvUsbVirtToPhys,
	.bspLock =              mv_lock,
	.bspUnlock =            mv_unlock,
	.bspGetCapRegAddr =     mvUsbGetCapRegAddr,
	.bspResetComplete =     usbDevResetComplete
};



static const char driver_name[] = USB_DRV_NAME;
static const char driver_desc [] = DRIVER_DESC;

static char ep_name [2*ARC_USB_MAX_ENDPOINTS][10] = 
{
    "ep0out", "ep0in",
};

#ifdef CONFIG_USB_OTG
struct mv_usb_dev* get_the_controller(void)
{
	return the_controllers[0];
}
#endif

static struct usb_ep_ops mv_usb_ep_ops;

static void mv_usb_ep_cancel_all_req(struct mv_usb_ep *mv_ep)
{
    struct mv_usb_dev*      mv_dev = mv_ep->usb_dev;
    struct usb_request*     usb_req;
    int                     req_cntr, tr_cntr;

    req_cntr = tr_cntr = 0;

    /* Cancel all transfers */
    while(_usb_device_get_transfer_status(mv_dev->mv_usb_handle, mv_ep->num, 
           mv_ep->is_in ? ARC_USB_SEND : ARC_USB_RECV) != ARC_USB_STATUS_IDLE)
   {
        tr_cntr++;
       _usb_device_cancel_transfer(mv_dev->mv_usb_handle, mv_ep->num, 
                           mv_ep->is_in ? ARC_USB_SEND : ARC_USB_RECV);
    }

    if(tr_cntr > 0)
    {
        mvOsPrintf("Cancel ALL transfers: ep=%d-%s, %d transfers\n", 
                        mv_ep->num, mv_ep->is_in ? "in" : "out", tr_cntr);
    }

    while (!list_empty (&mv_ep->req_list)) 
    {
        usb_req = list_entry (mv_ep->req_list.next, struct usb_request, list);

        /* Dequeue request and call complete function */
        list_del_init (&usb_req->list);
	mv_ep->processing = 0;

        if (usb_req->status == -EINPROGRESS)
            usb_req->status = -ESHUTDOWN;

        /* Only copy when OUT and no errors */
	if (!mv_ep->is_in && usb_req->status == 0) {
		if (((unsigned)usb_req->buf & L2_CACHE_ADDR_MASK)
			|| ((unsigned)(usb_req->buf + usb_req->length) & L2_CACHE_ADDR_MASK)) {
			struct mv_usb_request *req = container_of(usb_req, struct mv_usb_request, req);
			memcpy(usb_req->buf, req->align_buffer, usb_req->length);
		}
	}
        usb_req->complete (&mv_ep->ep, usb_req);
        req_cntr++;
        if(req_cntr >= MAX_XDS_FOR_TR_CALLS)
            break;
    }

    if(req_cntr > 0)
    {
        mvOsPrintf("Cancel ALL Requests: ep=%d-%s, %d requests\n", 
                        mv_ep->num, mv_ep->is_in ? "in" : "out", req_cntr);
        _usb_stats(mv_dev->mv_usb_handle);
    }

}

static uint_8 mv_usb_start_ep0(struct mv_usb_dev *mv_dev)
{
    DBGMSG("%s: mv_dev=%p, mv_usb_handle=%p, mv_ep=%p, usb_ep=%p\n", 
           __FUNCTION__, mv_dev, mv_dev->mv_usb_handle, &mv_dev->ep[0], &mv_dev->ep[0].ep);

    /* Init ep0 IN and OUT */
    mv_dev->ep[0].is_enabled = 1;

    _usb_device_init_endpoint(mv_dev->mv_usb_handle, 0, mv_dev->ep[0].ep.maxpacket, 
                                ARC_USB_SEND,  ARC_USB_CONTROL_ENDPOINT, 0);

    _usb_device_init_endpoint(mv_dev->mv_usb_handle, 0, mv_dev->ep[0].ep.maxpacket, 
                                ARC_USB_RECV, ARC_USB_CONTROL_ENDPOINT, 0);

    return USB_OK;
}

static uint_8 mv_usb_reinit (struct mv_usb_dev *usb_dev)
{
    int                 i, num;
    MV_BOOL             is_in;
    struct mv_usb_ep    *ep;
    unsigned long	flags;

    DBGMSG("%s: mv_dev=%p, mv_usb_handle=%p\n", 
           __FUNCTION__, usb_dev, usb_dev->mv_usb_handle);

    INIT_LIST_HEAD (&usb_dev->gadget.ep_list);
    num = 0;
    for(i=0; i<2*ARC_USB_MAX_ENDPOINTS; i++)
    {
        is_in = i % 2;
        ep = &usb_dev->ep[i];
        if (num != 0)
        {
            INIT_LIST_HEAD(&ep->ep.ep_list);
            list_add_tail (&ep->ep.ep_list, &usb_dev->gadget.ep_list);
            sprintf(&ep_name[i][0], "ep%d%s", num, is_in ? "in" : "out");
        }

        ep->ep.name = &ep_name[i][0];
	ep->ep.zlp = 0;
        ep->usb_dev = usb_dev;
        ep->num = num;
        ep->is_in = is_in;
        ep->is_enabled = 0;

	spin_lock_init (&ep->req_lock);
	spin_lock_irqsave(&ep->req_lock, flags);
        INIT_LIST_HEAD (&ep->req_list);
	spin_unlock_irqrestore(&ep->req_lock, flags);
    
        ep->ep.maxpacket = ~0;
        ep->ep.ops = &mv_usb_ep_ops;
        if(is_in)
            num++;
    }
    usb_dev->ep[0].ep.maxpacket = 64;
    usb_dev->gadget.ep0 = &usb_dev->ep[0].ep;
    INIT_LIST_HEAD (&usb_dev->gadget.ep0->ep_list);
    return USB_OK;
}

static int mv_usb_recv_data(struct mv_usb_dev *usb_dev, struct mv_usb_ep *usb_ep, struct usb_request *_req)
{
	struct mv_usb_request *req = container_of(_req, struct mv_usb_request, req);
	int error;
	u_int real_size;

	/* DMA from Device should be 64 bytes align, end address can be 32 bytes aligned */
	if (((unsigned)_req->buf & L2_CACHE_ADDR_MASK)
		|| ((unsigned)(_req->buf + _req->length) & L2_CACHE_ADDR_MASK)) {
		real_size = req->alloc_buffer + req->align_length - (uint_32)req->align_buffer;
		if (req->alloc_buffer == 0 || real_size < _req->length) {
			if (req->alloc_buffer && real_size < _req->length) {
				kfree((void *)req->alloc_buffer);
				req->alloc_buffer = 0;
				req->align_buffer = NULL;
				req->align_length = 0;
			}
			/* We can not make sure that the kmalloc will return 64bytes align address,
			   so allocate addtional 64 bytes */
			req->align_length = ALIGN(_req->length, L2_CACHE_LINE_SIZE) + L2_CACHE_LINE_SIZE;
			req->alloc_buffer = (uint_32)kmalloc(req->align_length, GFP_KERNEL);
			if (req->alloc_buffer == 0) {
				error = USBERR_ALLOC;
				printk("mv_usb_ep_queue: Can't RCV data (err=%d): ep_num=%d, pBuf=0x%x, size=%d\n because alignment",
					error, usb_ep->num, (unsigned)_req->buf, _req->length);
				goto out;
			}
			req->align_buffer = (void *)ALIGN(req->alloc_buffer, L2_CACHE_LINE_SIZE);
		}
		real_size = req->alloc_buffer + req->align_length - (uint_32)req->align_buffer;
		BUG_ON(real_size < _req->length);
		error = _usb_device_recv_data(usb_dev->mv_usb_handle, usb_ep->num, req->align_buffer, _req->length, real_size);

	}
	else
		error = _usb_device_recv_data(usb_dev->mv_usb_handle, usb_ep->num, _req->buf, _req->length, _req->length);
out:
	return error;
}

static int mv_usb_start_transfer(struct mv_usb_dev *usb_dev,
		struct mv_usb_ep *usb_ep)
{
	struct usb_request *_req;
	uint_8              error = 0;

	if (!list_empty(&usb_ep->req_list) && !usb_ep->processing)
		_req = list_entry(usb_ep->req_list.next,
				struct usb_request, list);
	else {
		DBGMSG("%s list not empty, go out\n", __func__);
		goto out;
	}

	DBGMSG("\n%s: %s data (err=%d): ep_num=%d, pBuf=0x%x, send_size=%d\n",
			__func__, usb_ep->is_in ? "SEND" : "RCV", error,
			usb_ep->num, (unsigned)_req->buf, _req->length);

	usb_ep->processing = 1;
	/* Add request to list */
	if (((usb_ep->num == 0) && (_req->length == 0)) || (usb_ep->is_in)) {
		error = _usb_device_send_data(usb_dev->mv_usb_handle,
				usb_ep->num, _req->buf, _req->length);
	} else {
		error = mv_usb_recv_data(usb_dev, usb_ep, _req);
	}

	if (error != USB_OK) {
		printk(KERN_INFO "ep_queue: Can't %s data (err=%d): ep_num=%d, "
			"pBuf=0x%x, send_size=%d\n", usb_ep->is_in ? "in" : "out",
			error, usb_ep->num, (unsigned)_req->buf, _req->length);
		list_del_init(&_req->list);
		usb_ep->processing = 0;
	}

out:
	return (int)error;
}

void mv_usb_bus_reset_service(void*      handle, 
                               uint_8     type, 
                               boolean    setup,
                               uint_8     direction, 
                               uint_8_ptr buffer,
                               uint_32    length, 
                               uint_8     error)
{
    int                     i, dev_no = _usb_device_get_dev_num(handle);
    struct mv_usb_dev       *mv_dev = the_controllers[dev_no];
    struct mv_usb_ep        *mv_ep;
	struct usb_gadget gadget;
	struct usb_composite_dev *cdev;

	reset_time_stamp = OSCR4;
    if(setup == 0)
    {
        /* mv_usb_show(mv_dev, 0x3ff); */

        /* Stop Hardware and cancel all pending requests */
        for (i=0; i<2*(ARC_USB_MAX_ENDPOINTS); i++)
        {
            mv_ep = &mv_dev->ep[i];

            if(mv_ep->is_enabled == 0)
                continue;

            mv_usb_ep_cancel_all_req(mv_ep);
        }
        /* If connected call Function disconnect callback */
        if( (mv_dev->gadget.speed != USB_SPEED_UNKNOWN) && 
            (mv_dev->driver != NULL) &&
            (mv_dev->driver->disconnect != NULL) )

        {

            USB_printf("USB gadget device disconnect or port reset: frindex=0x%x\n",
                    MV_REG_READ(MV_USB_CORE_FRAME_INDEX_REG(dev_no)) );    

		gadget = mv_dev->gadget;
		cdev = get_gadget_data(&gadget);
		cdev->mute_switch = 1;

		mv_dev->driver->disconnect(&mv_dev->gadget);
        }
        mv_dev->gadget.speed = USB_SPEED_UNKNOWN;

        /* Reinit all endpoints */
        mv_usb_reinit(mv_dev);
    }
    else
    {
        USB_printf("device start, ep0 start\n");    
        mv_usb_device_start(mv_dev->mv_usb_handle);
        /* Restart Control Endpoint #0 */
        mv_usb_start_ep0(mv_dev);
    }
}

void mv_usb_speed_service(void*      handle, 
                           uint_8     type, 
                           boolean    setup,
                           uint_8     direction, 
                           uint_8_ptr buffer,
                           uint_32    length, 
                           uint_8     error)
{
    int                     dev_no = _usb_device_get_dev_num(handle);
    struct mv_usb_dev       *mv_dev = the_controllers[dev_no];

    DBGMSG("Speed = %s\n", (length == ARC_USB_SPEED_HIGH) ? "High" : "Full");

    if(length == ARC_USB_SPEED_HIGH)
        mv_dev->gadget.speed = USB_SPEED_HIGH;
    else
        mv_dev->gadget.speed = USB_SPEED_FULL;

    return;
}

void mv_usb_suspend_service(void*      handle, 
                            uint_8     type, 
                            boolean    setup,
                            uint_8     direction, 
                            uint_8_ptr buffer,
                            uint_32    length, 
                            uint_8     error)
{
    int                     dev_no = _usb_device_get_dev_num(handle);
    struct mv_usb_dev       *mv_dev = the_controllers[dev_no];

    DBGMSG("%s\n", __FUNCTION__);

    if( (mv_dev->driver != NULL) &&
        (mv_dev->driver->suspend != NULL) )
	comp_driver_suspend(&mv_dev->cdev, &mv_dev->gadget, mv_dev->driver);
}

void mv_usb_resume_service(void*      handle, 
                            uint_8     type, 
                            boolean    setup,
                            uint_8     direction, 
                            uint_8_ptr buffer,
                            uint_32    length, 
                            uint_8     error)
{
    int                     dev_no = _usb_device_get_dev_num(handle);
    struct mv_usb_dev       *mv_dev = the_controllers[dev_no];

    DBGMSG("%s\n", __FUNCTION__);

    if( (mv_dev->driver != NULL) &&
        (mv_dev->driver->resume != NULL) )
	comp_driver_resume(&mv_dev->cdev, &mv_dev->gadget, mv_dev->driver);
}

void mv_usb_tr_complete_service(void*      handle, 
                                 uint_8     type, 
                                 boolean    setup,
                                 uint_8     direction, 
                                 uint_8_ptr buffer,
                                 uint_32    length, 
                                 uint_8     error)
{
    int                     dev_no = _usb_device_get_dev_num(handle);
    struct mv_usb_dev       *mv_dev = the_controllers[dev_no];
    struct mv_usb_ep       *mv_ep;
    struct usb_request      *usb_req;
    struct mv_usb_request   *req;
    int                     ep_num = (type*2) + direction;
    unsigned long	    flags;

    DBGMSG("%s: ep_num=%d, setup=%s, direction=%s, pBuf=0x%x, length=%d, error=0x%x\n", 
             __FUNCTION__, type, setup ? "YES" : "NO", 
             (direction == ARC_USB_RECV) ? "RECV" : "SEND", 
             (unsigned)buffer, (int)length, error);

    mv_ep = &mv_dev->ep[ep_num];
    if( !list_empty(&mv_ep->req_list) )
    {
        usb_req = list_entry (mv_ep->req_list.next, struct usb_request, list);
        req = container_of(usb_req, struct mv_usb_request, req);
        if((usb_req->buf != buffer) && (req->align_buffer != buffer))
        {
                printk("ep=%d-%s: req=%p, Unexpected buffer pointer: %p, len=%d, expected=%p\n", 
                    ep_num, (direction == ARC_USB_RECV) ? "out" : "in",
                    usb_req, buffer, length, usb_req->buf);
                return;       
        }

        /* Dequeue request and call complete function */
	spin_lock_irqsave(&mv_ep->req_lock, flags);
        list_del_init (&usb_req->list);
	spin_unlock_irqrestore(&mv_ep->req_lock, flags);
	mv_ep->processing = 0;

        if (!mv_ep->is_in && buffer != usb_req->buf) {
            if (buffer != req->align_buffer) {
                printk("request buffer 0x%x is not equal to align buffer 0x%x\n",
                    (unsigned)buffer, (unsigned)req->align_buffer);
                BUG_ON(1);
            }
	    dma_unmap_single(NULL, __pa(req->align_buffer), req->align_length, DMA_FROM_DEVICE);
            memcpy(usb_req->buf, buffer, length);
        } else
	    dma_unmap_single(NULL,
		__pa(usb_req->buf),
		usb_req->length,
		mv_ep->is_in?DMA_TO_DEVICE:DMA_FROM_DEVICE);

        usb_req->actual += length;
        usb_req->status = error;

        usb_req->complete (&mv_ep->ep, usb_req);
    }
    else
        mvOsPrintf("ep=%p, epName=%s, epNum=%d - reqList EMPTY\n", 
                mv_ep, mv_ep->ep.name, mv_ep->num);

	if ((mv_dev->in_single && mv_ep->is_in) ||
			(mv_dev->out_single && !mv_ep->is_in))
		mv_usb_start_transfer(mv_dev, mv_ep);
}

unsigned set_config_flag = 0;

void mv_usb_ep0_complete_service(void*      handle, 
                                 uint_8     type, 
                                 boolean    setup,
                                 uint_8     direction, 
                                 uint_8_ptr buffer,
                                 uint_32    length, 
                                 uint_8     error)
{ /* Body */
    int                     dev_no = _usb_device_get_dev_num(handle);
    struct mv_usb_dev       *mv_dev = the_controllers[dev_no];
    struct mv_usb_ep       *mv_ep;
    struct usb_request*     usb_req;
    int                     rc;
    boolean                 is_delegate = FALSE;
    static SETUP_STRUCT     mv_ctrl_req;
   
    DBGMSG("%s: EP0(%d), setup=%s, direction=%s, pBuf=0x%x, length=%d, error=0x%x\n", 
                __FUNCTION__, type, setup ? "YES" : "NO", 
                (direction == ARC_USB_RECV) ? "RECV" : "SEND", 
                (unsigned)buffer, (int)length, error);

    mv_ep = &mv_dev->ep[type];

    if (setup) 
    {
        _usb_device_read_setup_data(handle, type, (u8 *)&mv_ctrl_req);
        le16_to_cpus (&mv_ctrl_req.VALUE);
        le16_to_cpus (&mv_ctrl_req.INDEX);
        le16_to_cpus (&mv_ctrl_req.LENGTH);

        while(_usb_device_get_transfer_status(handle, mv_ep->num, 
                ARC_USB_SEND) != ARC_USB_STATUS_IDLE)
        {
            _usb_device_cancel_transfer(mv_dev->mv_usb_handle, mv_ep->num, 
                           ARC_USB_SEND);
        }
        while(_usb_device_get_transfer_status(handle, mv_ep->num, 
                ARC_USB_RECV) != ARC_USB_STATUS_IDLE)
        {
            _usb_device_cancel_transfer(mv_dev->mv_usb_handle, mv_ep->num, 
                           ARC_USB_RECV);
        }
        /* make sure any leftover request state is cleared */
        while (!list_empty (&mv_ep->req_list)) 
        {
            usb_req = list_entry (mv_ep->req_list.next, struct usb_request, list);

            /* Dequeue request and call complete function */
            list_del_init (&usb_req->list);
	    mv_ep->processing = 0;

            if (usb_req->status == -EINPROGRESS)
                usb_req->status = -EPROTO;

            usb_req->complete (&mv_ep->ep, usb_req);
        }
    }
    /* Setup request direction */
    mv_ep->is_in = (mv_ctrl_req.REQUESTTYPE & REQ_DIR_IN) != 0;     

    if(setup)
        DBGMSG("*** Setup ***: dir=%s, reqType=0x%x, req=0x%x, value=0x%02x, index=0x%02x, length=0x%02x\n", 
                (direction == ARC_USB_SEND) ? "In" : "Out",
                mv_ctrl_req.REQUESTTYPE, mv_ctrl_req.REQUEST, mv_ctrl_req.VALUE,
                mv_ctrl_req.INDEX, mv_ctrl_req.LENGTH); 

    /* Handle most lowlevel requests;
     * everything else goes uplevel to the gadget code.
     */
    if( (mv_ctrl_req.REQUESTTYPE & REQ_TYPE_MASK) == REQ_TYPE_STANDARD)
    {
        switch (mv_ctrl_req.REQUEST) 
        {
            case REQ_GET_STATUS: 
                mvUsbCh9GetStatus(handle, setup, &mv_ctrl_req);
                break;

            case REQ_CLEAR_FEATURE:
                mvUsbCh9ClearFeature(handle, setup, &mv_ctrl_req);
                break;

            case REQ_SET_FEATURE:
                mvUsbCh9SetFeature(handle, setup, &mv_ctrl_req);
                break;

            case REQ_SET_ADDRESS:
                mvUsbCh9SetAddress(handle, setup, &mv_ctrl_req);
                break;

            default:
                /* All others delegate call up-layer gadget code */
                is_delegate = TRUE;
        }
    }
    else
        is_delegate = TRUE;

    /* delegate call up-layer gadget code */
    if(is_delegate)
    {
        if(setup)
        {

	    mv_ep->ack_recv = 0;
	    mv_ep->ack_sent = 0;
    
	    /* when only one driver is registered, do as original */
	    if (mv_dev->cdev.driver_count == 1)
            	rc = mv_dev->driver->setup (&mv_dev->gadget, 
				(struct usb_ctrlrequest*)&mv_ctrl_req);
	    else if (mv_dev->cdev.driver_count <= 0) {
	    	pr_err("%s, error: dev->cdev.driver_count = %d\n",
	    	       __FUNCTION__, mv_dev->cdev.driver_count);
	    	return;
	    } else {

		if ((mv_ctrl_req.REQUESTTYPE & REQ_TYPE_MASK) == REQ_TYPE_STANDARD) {
        	    switch (mv_ctrl_req.REQUEST)
		    {
            	    case REQ_SET_CONFIGURATION:
	    		mv_dev->cdev.configuration = mv_ctrl_req.VALUE;
			comp_print("----------------set_configuration %d\n", 
					mv_ctrl_req.VALUE);
                	rc = comp_change_config(&mv_dev->cdev, &mv_dev->gadget, 
				mv_dev->driver, (struct usb_ctrlrequest *) &mv_ctrl_req);
			is_delegate = FALSE;

                    	break;

	    	    case REQ_SET_INTERFACE:
	    		mv_dev->cdev.interface = mv_ctrl_req.INDEX;
			mv_dev->cdev.alternate = mv_ctrl_req.VALUE;
			comp_print("----------------set_intf %d alt %d\n", 
					mv_ctrl_req.INDEX, mv_ctrl_req.VALUE);
                	comp_change_interface(&mv_dev->cdev, mv_ctrl_req.INDEX,
				(struct usb_ctrlrequest *) &mv_ctrl_req, &mv_dev->gadget, 
				mv_dev->driver, &rc);
			printk("rc = %d\n", rc);
			is_delegate = FALSE;
                	break;
		    }
		} 
		if (is_delegate)
		    rc = comp_ep0_req(&mv_dev->cdev, &mv_dev->gadget, 
				&mv_ep->ep, (struct usb_ctrlrequest *) &mv_ctrl_req);
	    }

	    if (is_set_conf(&mv_ctrl_req)) {
		    schedule_work(&mv_dev->work);
	    }

            if(rc < 0)
            {
                mvOsPrintf("Setup is failed: rc=%d, req=0x%02x, reqType=0x%x, value=0x%04x, index=0x%04x\n", 
                    rc, mv_ctrl_req.REQUEST, mv_ctrl_req.REQUESTTYPE, 
                    mv_ctrl_req.VALUE, mv_ctrl_req.INDEX);
                _usb_device_stall_endpoint(handle, 0, ARC_USB_RECV);
                return;
            }
            /* Acknowledge  */
            if( mv_ep->is_in ) {
		mv_ep->ack_recv = 1;
                _usb_device_recv_data(handle, 0, NULL, 0, 0);
            } 
            else if( mv_ctrl_req.LENGTH ) {
		mv_ep->ack_sent = 1;
                _usb_device_send_data(handle, 0, NULL, 0);
            }
        }
    }

    if(!setup)
    {
	int is_ack = 0;
	if( mv_ep->ack_sent && (direction == ARC_USB_SEND) ) {
		mv_ep->ack_sent = 0;
		is_ack = 1;
	}
	if( mv_ep->ack_recv && (direction != ARC_USB_SEND) ) {
		mv_ep->ack_recv = 0;
		is_ack = 1;
	}

        if( !list_empty(&mv_ep->req_list) && !is_ack)
        {
            usb_req = list_entry (mv_ep->req_list.next, struct usb_request, list);

            /* Dequeue request and call complete function */
            list_del_init (&usb_req->list);
	    mv_ep->processing = 0;

            /* For EP0 no setup OUT transfer, need consider the aligment*/
            if (direction != ARC_USB_SEND) {
                if (((unsigned)usb_req->buf & L2_CACHE_ADDR_MASK)
                    || ((unsigned)(usb_req->buf + usb_req->length) & L2_CACHE_ADDR_MASK)) {
                    struct mv_usb_request *req = container_of(usb_req, struct mv_usb_request, req);
		    dma_unmap_single(NULL, __pa(req->align_buffer), req->align_length, DMA_FROM_DEVICE);
                    memcpy(usb_req->buf, req->align_buffer, usb_req->length);
                } else
		    dma_unmap_single(NULL, __pa(usb_req->buf), usb_req->length, DMA_FROM_DEVICE);
            } else
		dma_unmap_single(NULL, __pa(usb_req->buf), usb_req->length, DMA_TO_DEVICE);

            usb_req->actual = length;
            usb_req->status = error;
            usb_req->complete (&mv_ep->ep, usb_req);
        }
        DBGMSG("Setup complete: dir=%s, is_in=%d, length=%d, is_ack=%d\n", 
                (direction == ARC_USB_SEND) ? "In" : "Out",
                mv_ep->is_in, length, is_ack);
    }
}

#ifdef MV_USB_VOLTAGE_FIX

static irqreturn_t mv_usb_vbus_irq (int irq, void *_dev)
{
    struct mv_usb_dev       *mv_dev = _dev;
    int                     vbus_change;

    vbus_change = mvUsbBackVoltageUpdate(mv_dev->dev_no, (int)mv_dev->vbus_gpp_no);
    if(vbus_change == 2)
    {
        if( (mv_dev->gadget.speed != USB_SPEED_UNKNOWN) &&
            (mv_dev->driver != NULL)                    &&
            (mv_dev->driver->disconnect != NULL) )
            mv_dev->driver->disconnect (&mv_dev->gadget);
    }

    return IRQ_HANDLED;
}
#endif /* MV_USB_VOLTAGE_FIX */

unsigned int mv_usb_otgsc = 0;
irqreturn_t mv_usb_dev_irq (int irq, void *_dev)
{
    struct mv_usb_dev       *mv_dev = the_controllers[0];
    unsigned base = mv_dev->regbase;
    u32 status;

    mv_usb_otgsc = u2o_get(base, U2xOTGSC);
    u2o_set(base, U2xOTGSC, mv_usb_otgsc);
 
/*
 * Now we use OTGSC to tect USB cable, but u2o_vbus driver can not
 * handle irq well, so add the tection here
 */
#ifdef CONFIG_USB_OTG
    if (irq == IRQ_USB_CTRL[0]) {
        status = (mv_usb_otgsc & U2xOTGSC_IS_MASK)
		& ((mv_usb_otgsc & U2xOTGSC_IE_MASK) >> 8);
        if (!mv_dev->transceiver)
	    goto client_handle;

        if (status) {
	    pr_debug("OTGSC %x\n", u2o_get(base, U2xOTGSC));
	    otg_interrupt(mv_dev->transceiver);
        }
    } else
        otg_interrupt(mv_dev->transceiver);

    if (!otg_is_client())
    {
    	return IRQ_HANDLED;
    }

client_handle:
#endif

    /* handle ARC USB Device interrupts */
    _usb_dci_vusb20_isr(mv_dev->mv_usb_handle);

    return IRQ_HANDLED;
}

static int mv_usb_ioctl(struct usb_gadget *g,
	       unsigned code, unsigned long param)
{
	switch (code) {

	default:
		printk("%s unknown ioctl %d\n", __func__, code);
		break;
	}

	return 0;
}

/* when a driver is successfully registered, it will receive
 * control requests including set_configuration(), which enables
 * non-control requests.  then usb traffic follows until a
 * disconnect is reported.  then a host may connect again, or
 * the driver might get unbound.
 */
void ep_list_update(struct mv_usb_dev *mv_dev, int restore);
int usb_gadget_register_driver (struct usb_gadget_driver *driver)
{
    int                 retval = 0, dev_no;
    struct mv_usb_dev   *mv_dev = NULL;
    uint_8              error = 0;

    mvOsPrintf("ENTER usb_gadget_register_driver: \n");

    /* Find USB Gadget device controller */
    for(dev_no=0; dev_no<mvCtrlUsbMaxGet(); dev_no++)
    {
        mv_dev = the_controllers[dev_no];
        if(mv_dev != NULL)
            break;
    }

    if ( (driver == NULL)
//            || (driver->speed != USB_SPEED_HIGH)
            || !driver->bind
//            || !driver->unbind
            || !driver->setup)
    {
	if(driver)
		mvOsPrintf("ERROR: speed=%d, bind=%p,  unbind=%p, setup=%p\n",
			driver->speed, driver->bind, driver->unbind, driver->setup);
        return -EINVAL;
    }

    if (!mv_dev)
    {
        mvOsPrintf("ERROR: max_dev=%d, mv_dev=%p\n", mvCtrlUsbMaxGet(), mv_dev);
        return -ENODEV;
    }

    if (comp_is_dev_busy(&mv_dev->cdev, mv_dev->driver))
    {
        mvOsPrintf("ERROR: driver=%p is busy\n", mv_dev->driver);
        return -EBUSY;
    }

    mvOsPrintf("usb_gadget_register_driver: dev=%d, mv_dev=%p, pDriver=%p\n", 
                dev_no, mv_dev, driver);

    mv_clock_enable(mv_dev, 1);

    /* composite preparation */
    ep_list_update(mv_dev, 0);
    local_irq_disable();
    _usb_device_stop(mv_dev->mv_usb_handle);
    stop_gadget(&mv_dev->cdev, &mv_dev->gadget, mv_dev->driver);
    local_irq_enable();

    /* first hook up the driver ... */
    mv_dev->driver = driver;
    mv_dev->gadget.dev.driver = &driver->driver;

    if (mv_dev->cdev.driver_count == 0) {
    	retval = device_add (&mv_dev->gadget.dev);
    	if (retval) {
		mv_dev->driver = NULL;
        	mv_dev->gadget.dev.driver = NULL;
        	return retval;
    	}
    }

    retval = driver->bind (&mv_dev->gadget);
    if (retval) {
        mvOsPrintf("bind to driver %s --> %d\n",
                driver->driver.name, retval);
        mv_dev->driver = 0;
        mv_dev->gadget.dev.driver = 0;
        return retval;
    }

    /* get information for composite device */
    /* get information for composite device */
    comp_register_driver(&mv_dev->cdev, &mv_dev->gadget, driver);

    mv_dev->cdev.driver_count++;

    if (mv_dev->cdev.driver_count == 1) {
        /* request_irq */
		if (request_irq (IRQ_USB_CTRL[dev_no], mv_usb_dev_irq, IRQF_DISABLED | IRQF_SHARED, 
                         driver_name, mv_dev) != 0) 
        {
            mvOsPrintf("%s register: request interrupt %d failed\n", 
					driver->driver.name, IRQ_USB_CTRL[dev_no]);
            return -EBUSY;
        }

#if defined(ENABLE_CABLE_DETECT)
// VBUS #ifdef CONFIG_PXA_VBUS
	pxa_register_vbus_event(mv_usb_vbus_change);
// #endif
#else
	mv_usb_device_start(mv_dev->mv_usb_handle);
	mv_usb_start_ep0(mv_dev);
#endif
    }

#ifdef CONFIG_USB_OTG
    if (mv_dev->transceiver)
    	otg_set_peripheral(mv_dev->transceiver, &mv_dev->gadget);
#endif

#if defined(ENABLE_CABLE_DETECT) || defined(CONFIG_USB_OTG)
    if (!is_cable_attached())
	    mv_clock_enable(mv_dev, 0);
#if !defined(CONFIG_MACH_ALKON)	
    if (is_cable_attached()) {
	mv_usb_connect_change(VBUS_HIGH);
    }
#endif /* CONFIG_MACH_ALKON */

#endif
#ifdef CONFIG_CPU_MMP3
	if (mv_dev->transceiver) {
		u32 otgsc = u2o_get(mv_dev->regbase, U2xOTGSC);

		if (otgsc & (U2xOTGSC_AVV | U2xOTGSC_ASV | U2xOTGSC_ID))
			otg_interrupt(mv_dev->transceiver);
	}
#endif
    mvOsPrintf("registered Marvell USB-%d gadget driver %s\n", dev_no, driver->driver.name);
    return error;
}
EXPORT_SYMBOL (usb_gadget_register_driver);

int usb_gadget_unregister_driver (struct usb_gadget_driver *driver)
{
    int                 i, dev_no;
    struct mv_usb_ep    *mv_ep;
    struct mv_usb_dev   *mv_dev = NULL;
    unsigned long       flags = 0;

    /* Find USB Gadget device controller */
    for(dev_no=0; dev_no<mvCtrlUsbMaxGet(); dev_no++)
    {
#ifndef CONFIG_USB_COMPOSITE
        if( (the_controllers[dev_no] != NULL) && (the_controllers[dev_no]->driver == driver) )
#else
        if (the_controllers[dev_no] != NULL)
#endif
        {
            mv_dev = the_controllers[dev_no];
            break;
        }
    }
    
    if (!mv_dev)
    {
        mvOsPrintf("usb_gadget_unregister_driver FAILED: no such device\n");
        return -ENODEV;
    }

    if (!comp_check_driver(&mv_dev->cdev, mv_dev->driver, driver))
    {
        mvOsPrintf("usb_gadget_unregister_driver FAILED: no such driver, dev_no=%d\n", dev_no);
        return -EINVAL;
    }

#if defined(ENABLE_CABLE_DETECT) || defined(CONFIG_USB_OTG)
	if (is_cable_attached()) {
		mv_usb_connect_change(0);
	}
#endif
    del_timer(&mv_dev->timer);
    mv_clock_enable(mv_dev, 1);
    /* Stop and Disable ARC USB device */
    MV_SPIN_LOCK_IRQSAVE(&mv_dev->lock, flags);

#ifdef CONFIG_USB_OTG
    if (mv_dev->transceiver && (mv_dev->cdev.driver_count == 1)) {
    	otg_set_peripheral(mv_dev->transceiver, NULL);
    }
#endif

    /* Stop Endpoints */
    for (i=0; i<2*(ARC_USB_MAX_ENDPOINTS); i++)
    {
        mv_ep = &mv_dev->ep[i];
#ifdef CONFIG_USB_COMPOSITE
        mv_ep->assigned = 0;
#endif
        if(mv_ep->is_enabled == 0)
            continue;

        mv_ep->is_enabled = 0;
        mv_usb_ep_cancel_all_req(mv_ep);
    
	if (!rely_on_vbus || connected)
		_usb_device_deinit_endpoint(mv_dev->mv_usb_handle,
			mv_ep->num, mv_ep->is_in ? ARC_USB_SEND : ARC_USB_RECV);
    }
    MV_SPIN_UNLOCK_IRQRESTORE(&mv_dev->lock, flags);

    local_irq_disable();
    if (!rely_on_vbus || connected)
	_usb_device_stop(mv_dev->mv_usb_handle);
    stop_cur_gadget(&mv_dev->cdev, &mv_dev->gadget, driver);
    local_irq_enable();

    driver->unbind (&mv_dev->gadget);

    mv_dev->cdev.driver_count--;
    comp_unregister_driver(&mv_dev->cdev, &mv_dev->gadget, 
		    &mv_dev->driver, driver);

    if (mv_dev->cdev.driver_count != 0)
	goto out;

    /* free_irq */
	free_irq (IRQ_USB_CTRL[dev_no], mv_dev);
    
    mv_dev->gadget.dev.driver = 0;
    mv_dev->driver = 0;
    device_del (&mv_dev->gadget.dev);
    
    mv_dev->gadget.speed = USB_SPEED_UNKNOWN;
    
    /* Reinit all endpoints */
    mv_usb_reinit(mv_dev);
//VBUS  #if defined(ENABLE_CABLE_DETECT) && defined(CONFIG_PXA_VBUS)
 #if defined(ENABLE_CABLE_DETECT) // VBUS && defined(CONFIG_PXA_VBUS)
    pxa_unregister_vbus_event(mv_usb_vbus_change);
    mv_clock_enable(mv_dev, 0);
#endif

out:
    /*device_remove_file(dev->dev, &dev_attr_function); ?????*/
    mvOsPrintf("unregistered Marvell USB %d gadget driver %s\n", dev_no, driver->driver.name);

    return 0;
}
EXPORT_SYMBOL (usb_gadget_unregister_driver);

static void _usb_ep_req(struct mv_usb_dev* mv_dev, int num)
{
        struct mv_usb_ep *ep = &mv_dev->ep[num];
	struct usb_request *usb_req;

	printk("ep %d requests:\n", num);
	list_for_each_entry(usb_req, &ep->req_list, list) {
		printk("req (0x%p): buf %p len %d\n", usb_req, usb_req->buf, usb_req->length);
	}
	printk("\n");
}

void    mv_usb_show(struct mv_usb_dev* mv_dev, unsigned int mode)
{
    int     i;

    if( MV_BIT_CHECK(mode, 0) )
        _usb_regs(mv_dev->mv_usb_handle);

    if( MV_BIT_CHECK(mode, 1) )
        _usb_status(mv_dev->mv_usb_handle);

    if( MV_BIT_CHECK(mode, 2) )
        _usb_stats(mv_dev->mv_usb_handle);

    if( MV_BIT_CHECK(mode, 3) )
        _usb_debug_print_trace_log();

    for(i=0; i<ARC_USB_MAX_ENDPOINTS; i++)
    {
	int recv=1, send=1;
#ifdef CONFIG_USB_COMPOSITE
        {
                recv = send = 0;
                if (!i)
                        send = recv = 1;
                if (mv_dev->ep[i*2].assigned) {
                        recv = 1;
                }
                if (mv_dev->ep[i*2+1].assigned) {
                        send = 1;
                }
         }
#endif
        if( MV_BIT_CHECK(mode, (8+i)) )
        {
            if(send) {
            	_usb_ep_status(mv_dev->mv_usb_handle, i, ARC_USB_SEND);
		_usb_ep_req(mv_dev, i);
	    }
	    if(recv) {
	        _usb_ep_status(mv_dev->mv_usb_handle, i, ARC_USB_RECV);
		_usb_ep_req(mv_dev, i);
	    }
        }
    }
}
EXPORT_SYMBOL (mv_usb_show);

static int  mv_usb_ep_enable(struct usb_ep *_ep, 
                              const struct usb_endpoint_descriptor *desc)
{
    struct mv_usb_dev* usb_dev;
    struct mv_usb_ep*  usb_ep;
    uint_16             maxSize;
    uint_8              epType; 
    unsigned long       flags = 0;

    usb_ep = container_of (_ep, struct mv_usb_ep, ep);
    if( (_ep == NULL) || (desc == NULL) )
        return -EINVAL;
    
    usb_dev = usb_ep->usb_dev;

    mvOsPrintf("USB Enable %s: type=%d, maxPktSize=%d\n",
                _ep->name, desc->bmAttributes & 0x3, desc->wMaxPacketSize);

    if(usb_ep->num == 0)
    {
        mvOsPrintf("mv_usb: ep0 is reserved\n");
        return -EINVAL;
    }

    if(desc->bDescriptorType != USB_DT_ENDPOINT)
    {
        mvOsPrintf("mv_usb: wrong descriptor type %d\n", desc->bDescriptorType);
        return -EINVAL;
    }

    if(usb_ep->is_enabled)
    {
        mvOsPrintf("mv_usb: %d%s Endpoint (%s) is already in use\n", 
                    usb_ep->num, usb_ep->is_in ? "In" : "Out", usb_ep->ep.name);
        return 0;
    }

    MV_SPIN_LOCK_IRQSAVE(&usb_dev->lock, flags);

    usb_dev = usb_ep->usb_dev;
    if( (usb_dev->driver == NULL) || 
        (usb_dev->gadget.speed == USB_SPEED_UNKNOWN) )
    {
        MV_SPIN_UNLOCK_IRQRESTORE(&usb_dev->lock, flags);
        return -ESHUTDOWN;
    }
    /* Max packet size */
    maxSize = le16_to_cpu (desc->wMaxPacketSize);

    /* Endpoint type */
    if( (desc->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) == USB_ENDPOINT_XFER_CONTROL)
        epType = ARC_USB_CONTROL_ENDPOINT;
    else if( (desc->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) == USB_ENDPOINT_XFER_ISOC) 
        epType = ARC_USB_ISOCHRONOUS_ENDPOINT;
    else if( (desc->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) == USB_ENDPOINT_XFER_BULK) 
        epType = ARC_USB_BULK_ENDPOINT;
    else
        epType = ARC_USB_INTERRUPT_ENDPOINT;

    _ep->maxpacket = maxSize & 0x7ff;
    usb_ep->is_enabled = 1;

    _usb_device_init_endpoint(usb_dev->mv_usb_handle, usb_ep->num, maxSize, 
            usb_ep->is_in ? ARC_USB_SEND : ARC_USB_RECV, epType,
		((epType == ARC_USB_BULK_ENDPOINT) && !_ep->zlp) ? ARC_USB_DEVICE_DONT_ZERO_TERMINATE : 0);

    MV_SPIN_UNLOCK_IRQRESTORE(&usb_dev->lock, flags);
    return 0;
}

static int  mv_usb_ep_disable (struct usb_ep *_ep)
{
    struct mv_usb_dev*  mv_dev;
    struct mv_usb_ep*   mv_ep;
    unsigned long       flags = 0;
    uint_8              direction;

    mv_ep = container_of (_ep, struct mv_usb_ep, ep);
    if( (_ep == NULL) || (mv_ep->is_enabled == 0) || (mv_ep->num == 0))
        return -EINVAL;

    mv_dev = mv_ep->usb_dev;

    mvOsPrintf("mv_usb_ep_disable: mv_dev=%p, ep=0x%x (%d-%s), name=%s\n", 
                mv_dev, (unsigned)_ep, mv_ep->num, mv_ep->is_in ? "in" : "out", 
                _ep->name);
    
    MV_SPIN_LOCK_IRQSAVE(&mv_dev->lock, flags);

    direction = mv_ep->is_in ? ARC_USB_SEND : ARC_USB_RECV;

    mv_ep->is_enabled = 0;

    /* Cancell all requests */
    mv_usb_ep_cancel_all_req(mv_ep);
	
    if (!rely_on_vbus || connected)
	/* Disable endpoint */
	_usb_device_deinit_endpoint(mv_dev->mv_usb_handle,
			mv_ep->num, direction);

    MV_SPIN_UNLOCK_IRQRESTORE(&mv_dev->lock, flags);
    return 0;
}


static struct usb_request* mv_usb_ep_alloc_request(struct usb_ep *_ep, gfp_t gfp_flags)
{
    struct mv_usb_request* req;

    if (!_ep)
        return NULL;

    req = kmalloc (sizeof *req, gfp_flags);
    if (!req)
        return NULL;

    memset (req, 0, sizeof *req);
    INIT_LIST_HEAD (&req->req.list);

    return &req->req;
}

static void     mv_usb_ep_free_request(struct usb_ep *_ep, struct usb_request *_req)
{
    struct mv_usb_request* req;

    if (!_ep || !_req)
    {
        mvOsPrintf("ep_free_request Error: _ep=%p, _req=%p\n", _ep, _req);
        return;
    }
    req = container_of(_req, struct mv_usb_request, req);
    if (req->alloc_buffer)
        kfree((void *)req->alloc_buffer);
    kfree (req);
    mvOsPrintf("ep_kfree_request\n");
}

/*************************** composite **************************************/

#ifdef CONFIG_USB_COMPOSITE

void ep_list_update(struct mv_usb_dev *mv_dev, int restore)
{
    int i;

    if (restore) {
	    mv_usb_reinit(mv_dev);
	    return;
    }

    for (i=1;i<2*ARC_USB_MAX_ENDPOINTS;i++) {
	if (mv_dev->ep[i].assigned) {
		comp_print("\t\tep %d is_in %d, list del\n", 
				i, mv_dev->ep[i].is_in);
		list_del_init(&mv_dev->ep[i].ep.ep_list);
	}
    }
}

int stop_udc(struct usb_gadget_driver *driver)
{
	struct mv_usb_dev *dev = the_controllers[0];

#ifndef CONFIG_USB_COMPOSITE
	/* don't disconnect drivers more than once */
	if (dev->gadget.speed == USB_SPEED_UNKNOWN)
		driver = 0;
#endif
	dev->gadget.speed = USB_SPEED_UNKNOWN;
	return 0;
}

/*
 * 	udc_reinit - initialize software state
 */
void udc_reinit(void)
{
	struct mv_usb_dev *dev = the_controllers[0];

	/* device/ep0 records init */
	dev->cdev.ep0state = EP0_IDLE;

	dev->cdev.configuration = 0;
	dev->cdev.interface = 0;
	dev->cdev.alternate = 0;
}


void comp_do_fake_req(struct mv_usb_dev *dev, 
    struct mv_usb_ep *mv_ep, struct usb_request *req)
{			
	comp_print("%s: in EP0_IN_FAKE\n", __func__);
		dev->cdev.config_length = req->length;
		get_fake_config(&dev->cdev, req, dev->gadget.speed);
		dev->cdev.ep0state = EP0_IDLE;
		req->actual = req->length;
		//done(ep, req, 0);


	/* from mv_usb_ep0_complete_service */
        /* make sure any leftover request state is cleared */
        while (!list_empty (&mv_ep->req_list)) 
        {
	    struct usb_request *usb_req;
	
            usb_req = list_entry (mv_ep->req_list.next, struct usb_request, list);

            /* Dequeue request and call complete function */
            list_del_init (&req->list);
	    mv_ep->processing = 0;

            if (req->status == -EINPROGRESS)
                req->status = -EPROTO;

            req->complete (&mv_ep->ep, req);
        }

}

static void set_ep(struct usb_endpoint_descriptor *ep_desc, int ep_num,
		   int config, int interface, int alt)
{
	struct mv_usb_dev *dev = the_controllers[0];
	int k = ep_num;

	
/*	dev->ep[k].desc = ep_desc;
	dev->ep[k].assigned = 1;
	if (!(ep_desc->wMaxPacketSize))
		ep_desc->wMaxPacketSize = dev->ep[k].ep.maxpacket;
	else
		dev->ep[k].ep.maxpacket = ep_desc->wMaxPacketSize;
	dev->ep[k].dir_in = (ep_desc->bEndpointAddress & USB_DIR_IN) ? 1 : 0;
	dev->ep[k].ep_type = ep_desc->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK;
	dev->ep[k].stopped = 1;
	dev->ep[k].aisn = alt;*/
	dev->ep[k].assigned = 1;
	comp_set_ep(&dev->cdev, k, config, interface);
}


/* set_eps
 * assign a physical ep to logic one, and
 * fill pxa_ep structure with their configuration, interface, alternate
 * settings, assigned interface number
 */
int set_eps(__u8 num_eps, struct usb_endpoint_descriptor *p_ep_desc, int len,
		int config, int interface, int alt)
{
	struct usb_endpoint_descriptor *ep_desc = p_ep_desc;
	struct mv_usb_dev *dev = the_controllers[0];
	int ep_desc_length = len;
	int j, k, ret;

	comp_print("  ----%s----\n", __FUNCTION__);
	for (j = 0; j < num_eps; j++) {
		/* find the ep */
		ret = get_extra_descriptor((char *)p_ep_desc, ep_desc_length,
			USB_DT_ENDPOINT, (void **) &ep_desc);
		if (ret >= 0) {
			/* compare with the ep in controller, if match,
			 * fill the config, interface and asin number fields */
			for (k = 2; k < 2*ARC_USB_MAX_ENDPOINTS; k++) {
				comp_print("search ep 0x%x addr 0x%x is_in %d\n", k/2, 
					ep_desc->bEndpointAddress, dev->ep[k].is_in);
				if (((ep_desc->bEndpointAddress>>7) == dev->ep[k].is_in) \
				 	&& ((k/2) == (ep_desc->bEndpointAddress & 0x0f))) {
					comp_print("found match at %d\n", k);
        				list_del_init (&dev->ep[k].ep.ep_list);
					set_ep(ep_desc, k, config, interface,
					       alt);
					break;
				}
			}
		} else {
			comp_print("  ep desc not find, ep_desc_length=0x%x,"
			     " p_ep_desc=0x%x\n",
			     ep_desc_length, (int)p_ep_desc);
			return -EFAULT;
		}

		ep_desc_length -= (int)ep_desc - (int)p_ep_desc +
				  ep_desc->bLength;
		p_ep_desc = (struct usb_endpoint_descriptor *)
			    ((unsigned)ep_desc + ep_desc->bLength);
	}/* for(j=0;j<num_eps;j++) */
	return 0;
}

#else

int set_eps(__u8 num_eps, struct usb_endpoint_descriptor *p_ep_desc, int len,
		int config, int interface, int alt) {return 0;}
void ep_list_update(struct mv_usb_dev *mv_dev, int restore) {}
int stop_udc(struct usb_gadget_driver *driver) { return 0; }
void udc_reinit(void) {}
#endif

/*************************** composite end **********************************/

static int      mv_usb_ep_queue (struct usb_ep *_ep, struct usb_request *_req, 
                                 gfp_t gfp_flags)
{
    struct mv_usb_dev* usb_dev;
    struct mv_usb_ep*  usb_ep;
    unsigned long       flags = 0, req_flags = 0;
    uint_8              error = 0;
    int			is_first_req;

    usb_ep = container_of (_ep, struct mv_usb_ep, ep);
    /* check parameters */
    if( (_ep == NULL) || (_req == NULL) )
    {
        printk("ep_queue Failed: _ep=%p, _req=%p\n", _ep, _req);
        return -EINVAL;
    }
    usb_dev = usb_ep->usb_dev;

#ifdef CONFIG_USB_COMPOSITE
    if (usb_dev->cdev.ep0state == EP0_IN_FAKE) {
	comp_print("USB FAKE request for composite\n");
	comp_do_fake_req(usb_dev, usb_ep, _req);
	return 0;
    }
#endif

    if ( (usb_dev->driver == NULL) || (usb_dev->gadget.speed == USB_SPEED_UNKNOWN) )
        return -ESHUTDOWN;

    if(usb_ep->is_enabled == 0)
    {
        printk("ep_queue Failed - %s is disabled: usb_ep=%p\n", _ep->name, usb_ep);
        return -EINVAL;
    }

    DBGMSG("%s: num=%d-%s, name=%s, _req=%p, buf=%p, length=%d\n", 
                __FUNCTION__, usb_ep->num, usb_ep->is_in ? "in" : "out", 
                _ep->name, _req, _req->buf, _req->length);

    MV_SPIN_LOCK_IRQSAVE(&usb_dev->lock, flags);
                
    _req->status = -EINPROGRESS;
    _req->actual = 0;

	if ((usb_dev->in_single && usb_ep->is_in) ||
			(usb_dev->out_single && !usb_ep->is_in)) {
		is_first_req = list_empty(&usb_ep->req_list);
		DBGMSG("queue req %p(first=%s), len %d buf %p\n", _req,
			is_first_req ? "yes" : "no", _req->length, _req->buf);

		spin_lock_irqsave(&usb_ep->req_lock, req_flags);
		list_add_tail(&_req->list, &usb_ep->req_list);
		spin_unlock_irqrestore(&usb_ep->req_lock, req_flags);
		if (is_first_req)
			error = mv_usb_start_transfer(usb_dev, usb_ep);

		goto out;
	}

    /* Add request to list */
    if( ((usb_ep->num == 0) && (_req->length == 0)) || (usb_ep->is_in) )
    {
	spin_lock_irqsave(&usb_ep->req_lock, req_flags);
        list_add_tail(&_req->list, &usb_ep->req_list);
	spin_unlock_irqrestore(&usb_ep->req_lock, req_flags);

        error = _usb_device_send_data(usb_dev->mv_usb_handle, usb_ep->num, _req->buf, _req->length);
        if(error != USB_OK)
        {
            printk("ep_queue: Can't SEND data (err=%d): ep_num=%d, pBuf=0x%x, send_size=%d\n",
                    error, usb_ep->num, (unsigned)_req->buf, _req->length);
            list_del_init (&_req->list);
        }
    }
    else
    {
	error = mv_usb_recv_data(usb_dev, usb_ep, _req);
        if(error != USB_OK)
        {
            printk("mv_usb_ep_queue: Can't RCV data (err=%d): ep_num=%d, pBuf=0x%x, size=%d\n",
                        error, usb_ep->num, (unsigned)_req->buf, _req->length);
        }
        else {
	    spin_lock_irqsave(&usb_ep->req_lock, req_flags);
            list_add_tail(&_req->list, &usb_ep->req_list);
	    spin_unlock_irqrestore(&usb_ep->req_lock, req_flags);
	}
    }

out:
    DBGMSG("%s end\n\n", __func__);
    MV_SPIN_UNLOCK_IRQRESTORE(&usb_dev->lock, flags);

    return (int)error;
}

/* Cancell request */
static int      mv_usb_ep_dequeue (struct usb_ep *_ep, struct usb_request *_req)
{
    struct mv_usb_dev* usb_dev;
    struct usb_request *usb_req;
    struct mv_usb_ep*  usb_ep;
    unsigned long       flags = 0;
    int                 status = 0;

    usb_ep = container_of (_ep, struct mv_usb_ep, ep);
    /* check parameters */
    if( (_ep == NULL) || (_req == NULL) || (usb_ep->is_enabled == 0) )
        return -EINVAL;

    usb_dev = usb_ep->usb_dev;
        
    if ( (usb_dev->driver == NULL) || (usb_dev->gadget.speed == USB_SPEED_UNKNOWN) )
    {
        mvOsPrintf("mv_usb_ep_dequeue: ep=0x%x, num=%d-%s, name=%s, driver=0x%x, speed=%d\n", 
                (unsigned)_ep, usb_ep->num, usb_ep->is_in ? "in" : "out", 
                _ep->name, (unsigned)usb_dev->driver, usb_dev->gadget.speed);

        return -ESHUTDOWN;
    }

    MV_SPIN_LOCK_IRQSAVE(&usb_dev->lock, flags);

    /* ????? Currently supported only dequeue request from the HEAD of List */
    if( !list_empty(&usb_ep->req_list) )
    {
        usb_req = list_entry (usb_ep->req_list.next, struct usb_request, list);

        if(usb_req == _req)
        {
            /* Cancel transfer */
            _usb_device_cancel_transfer(usb_dev->mv_usb_handle, usb_ep->num, 
                            usb_ep->is_in ? ARC_USB_SEND : ARC_USB_RECV);
            /* Dequeue request and call complete function */
            list_del_init (&_req->list);
	    usb_ep->processing = 0;

            if (_req->status == -EINPROGRESS)
                _req->status = -ECONNRESET;

            /* Onlt copy when OUT and no errors */
	    if (!usb_ep->is_in && _req->status == 0) {
                if (((unsigned)_req->buf & L2_CACHE_ADDR_MASK)
                    || ((unsigned)(_req->buf + _req->length) & L2_CACHE_ADDR_MASK)) {
                    struct mv_usb_request *req = container_of(_req, struct mv_usb_request, req);
                    memcpy(_req->buf, req->align_buffer, _req->length);
                }
            }

            /* ????? what about enable interrupts */
            _req->complete (&usb_ep->ep, _req);
        }
        else
        {
            mvOsPrintf("Cancell request failed: ep=%p, usb_req=%p, req=%p\n", 
                        _ep, usb_req, _req);
            status = EINVAL;
        }
    }
    else
        mvOsPrintf("%s: ep=%p, epName=%s, epNum=%d - reqList EMPTY\n", 
                    __FUNCTION__, usb_ep, usb_ep->ep.name, usb_ep->num);
    
    MV_SPIN_UNLOCK_IRQRESTORE(&usb_dev->lock, flags);

    return status;
}

static int      mv_usb_ep_set_halt (struct usb_ep *_ep, int value)
{
    struct mv_usb_ep*   mv_ep;
    unsigned long       flags = 0;
    int                 retval = 0;

    mv_ep = container_of (_ep, struct mv_usb_ep, ep);
    if (_ep == NULL)
        return -EINVAL;
    if( (mv_ep->usb_dev->driver == NULL) || 
        (mv_ep->usb_dev->gadget.speed == USB_SPEED_UNKNOWN) )
        return -ESHUTDOWN;

    mvOsPrintf("%s - %s \n", 
                _ep->name, value ? "Stalled" : "Unstalled");

    MV_SPIN_LOCK_IRQSAVE(&mv_ep->usb_dev->lock, flags);
    if (!list_empty (&mv_ep->req_list))
        retval = -EAGAIN;
    else 
    {
        /* set/clear, then synch memory views with the device */
        if (value) 
        {
            if (mv_ep->num == 0)
                mv_ep->usb_dev->protocol_stall = 1;
            else
                _usb_device_stall_endpoint(mv_ep->usb_dev->mv_usb_handle, mv_ep->num,
                mv_ep->is_in ? ARC_USB_SEND : ARC_USB_RECV);
        } 
        else
        {
            _usb_device_unstall_endpoint(mv_ep->usb_dev->mv_usb_handle, mv_ep->num, 
                                    mv_ep->is_in ? ARC_USB_SEND : ARC_USB_RECV);
        }
    }
    MV_SPIN_UNLOCK_IRQRESTORE(&mv_ep->usb_dev->lock, flags);

    return retval;
}


static void     mv_usb_ep_fifo_flush (struct usb_ep *_ep)
{
    DBGMSG("%s: ep=%p, ep_name=%s - NOT supported\n", __FUNCTION__, _ep, _ep->name);
}

static struct usb_ep_ops mv_usb_ep_ops = 
{
    .enable         = mv_usb_ep_enable,
    .disable        = mv_usb_ep_disable,

    .alloc_request  = mv_usb_ep_alloc_request,
    .free_request   = mv_usb_ep_free_request,

    .queue          = mv_usb_ep_queue,
    .dequeue        = mv_usb_ep_dequeue,

    .set_halt       = mv_usb_ep_set_halt,
    .fifo_flush     = mv_usb_ep_fifo_flush,
    /*.fifo_status    =  Not supported */
};

static int mv_usb_get_frame (struct usb_gadget *_gadget)
{
    DBGMSG("Call mv_usb_get_frame - NOT supported\n");
    return 0;
}

static int mv_usb_set_selfpowered (struct usb_gadget *_gadget, int value)
{
    DBGMSG("Call mv_usb_set_selfpowered - NOT supported\n");
    return 0;
}

static int mv_usb_vbus_session (struct usb_gadget *_gadget, int is_active)
{
    struct mv_usb_dev *mv_dev = the_controllers[0];

    if (rely_on_vbus && !connected)
	    return 0;

    if (!is_active)
	udc_stop(&mv_dev->cdev, &mv_dev->gadget, mv_dev->driver, 1);
    return 0;
}

/**
 * calc_delay_time() - calculates the time (in msec) needed to wait
 *					   before the usb controller can be started
 *
 */
static int calc_delay_time(void)
{
	unsigned int delay_time;
	if (OSCR4 > reset_time_stamp)
		delay_time = OSCR4 - reset_time_stamp;
	else
		delay_time = 4294967295 - reset_time_stamp + OSCR4;
	if (delay_time > (MAX_DELAY_BETWEEN_ENUMERATIONS * OSCR4_TICKS_PER_MS))
		return 0;
	else {
		delay_time /= OSCR4_TICKS_PER_MS;
		delay_time = MAX_DELAY_BETWEEN_ENUMERATIONS - delay_time;
		return (delay_time);
	}
}

static int mv_usb_pullup (struct usb_gadget *_gadget, int is_on)
{
    struct mv_usb_dev *mv_dev = the_controllers[0];

    if (rely_on_vbus && !connected)
	    return 0;

    pr_debug("%s is_on %d\n", __func__, is_on);
    if (is_on) {
		int delay_time_ms = calc_delay_time();
		if (delay_time_ms > 0)
			msleep(delay_time_ms);
   	mv_usb_device_start(mv_dev->mv_usb_handle);
    } else {
    	udc_stop(&mv_dev->cdev, &mv_dev->gadget, mv_dev->driver, 1);
   	_usb_device_stop(mv_dev->mv_usb_handle);
    }

    return 0;
}
static const struct usb_gadget_ops mv_usb_ops = 
{
    .get_frame       = mv_usb_get_frame,
    .wakeup          = 0, /* remote wakeup is not supported */
    .set_selfpowered = mv_usb_set_selfpowered,
    .ioctl           = mv_usb_ioctl,
    .vbus_session    = mv_usb_vbus_session,
    .pullup          = mv_usb_pullup,
};

static void mv_usb_gadget_release (struct device *_dev)
{
    struct mv_usb_dev   *usb_dev = dev_get_drvdata (_dev);

    mvOsPrintf("Call mv_usb_gadget_release \n");
    mvOsFree(usb_dev);
}

static ssize_t show_connected(struct device *dev, \
				struct device_attribute *attr, char *buf)
{
	if (connected)
		return sprintf(buf, "%d\n", PXA_GADGET_CABLE_CONNECTED);
	else
		return sprintf(buf, "%d\n", PXA_GADGET_CABLE_DISCONNECTED);
}

static ssize_t show_mode(struct device *dev, \
				struct device_attribute *attr, char *buf)
{
/* FIX ME: need to use new pmic API to distinguish
between device and host modes */
/*	if (pxa3xx_pmic_get_usbid() > 0x10)*/
		return sprintf(buf, "%d\n", PXA_GADGET_MODE_DEVICE);
/*	else
		return sprintf(buf, "%d\n", PXA_GADGET_MODE_HOST); */
}

#ifdef CONFIG_USB_ANDROID
extern int		android_show_function(char *buf);
extern int		android_switch_function(char *func);

static ssize_t show_composite(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return android_show_function(buf);
}

static ssize_t store_composite(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t  ret;
	char usbcfg[100];

	sscanf(buf, "%s", usbcfg);
	printk(KERN_DEBUG "store new composite configuration => %s\n", usbcfg);
	ret = android_switch_function(usbcfg);

	if (ret == 0)
		return count;
	else
		return 0;
}
static DEVICE_ATTR(composite, S_IRUGO|S_IWUSR, show_composite, store_composite);
#endif
static DEVICE_ATTR(connected, 0644, show_connected, NULL);
static DEVICE_ATTR(mode, 0644, show_mode, NULL);
static struct attribute *mvgadget_attributes[] = {
	&dev_attr_connected.attr,
	&dev_attr_mode.attr,
#ifdef CONFIG_USB_ANDROID
	&dev_attr_composite.attr,
#endif
	NULL,
};
static struct attribute_group mvgadget_attr_group = {
	.attrs = mvgadget_attributes,
};

#if 0
static void enable_phy(void)
{
	volatile unsigned tmp;
//	PLL VCO and TX Impedance Calibration Timing:

//	PU   			__________|----------------------------------|
//	VOCAL STAR		___________________|--|______________________|
//	REG_RCAL_START	________________________________|--|_________|
//					          | 200us  |40| 200us   |40| 200us   | USB PHY READY

//	ACCR1 = ACCR1 | ACCR1_PU_OTG | ACCR1_PU_PLL | ACCR1_PU;

	/* U2PPLL */
	tmp = MV_PHYREG_READ(MV_USB_PHY_PLL_CTRL_REG);
	tmp &= ~(MV_USB_PHY_PLL_ICP_MASK | MV_USB_PHY_PLL_KVCO_MASK);
	MV_PHYREG_WRITE(MV_USB_PHY_PLL_CTRL_REG, tmp);

	tmp = MV_PHYREG_READ(MV_USB_PHY_PLL_CTRL_REG);
	tmp |= (0x6<<MV_USB_PHY_PLL_ICP_SHIFT) | (0x4<<MV_USB_PHY_PLL_KVCO_SHIFT) | MV_USB_PHY_PLL_KVCO_EXT | MV_USB_PHY_PLL_CLK_BLK_EN;
	MV_PHYREG_WRITE(MV_USB_PHY_PLL_CTRL_REG, tmp);
	tmp = MV_PHYREG_READ(MV_USB_PHY_PLL_CTRL_REG);

	/* IVREF */
	tmp = MV_PHYREG_READ(MV_USB_PHY_IVREF_CTRL_REG);
	tmp &= ~(MV_USB_PHY_IVREF_BG_VSEL_MASK | MV_USB_PHY_IVREF_RXVDD18_MASK);
	MV_PHYREG_WRITE(MV_USB_PHY_IVREF_CTRL_REG, tmp);

	/* U2DRX */
	tmp = MV_PHYREG_READ(MV_USB_PHY_RX_CTRL_REG);
	tmp &= ~(U2PRX_SQ_THRESH_MASK);
	MV_PHYREG_WRITE(MV_USB_PHY_RX_CTRL_REG, tmp);
	tmp = MV_PHYREG_READ(MV_USB_PHY_RX_CTRL_REG);

	tmp = MV_PHYREG_READ(MV_USB_PHY_RX_CTRL_REG);
	tmp |= (0x7<<U2PRX_SQ_THRESH_SHIFT);
	MV_PHYREG_WRITE(MV_USB_PHY_RX_CTRL_REG, tmp);
	tmp = MV_PHYREG_READ(MV_USB_PHY_RX_CTRL_REG);

	/* wait U2PPLL ready */
	do {
		tmp = MV_PHYREG_READ(MV_USB_PHY_PLL_CTRL_REG);
	} while (!(tmp & MV_USB_PHY_PLL_READY));

	// SET USB_REG_ARC_DPDM_MODE BIT OF U2PT0
	tmp = MV_PHYREG_READ(MV_USB_PHY_TEST_CTRL0_REG);
	tmp |= MV_USB_PHY_TEST_CTRL0_DPDM_MODE; 
	MV_PHYREG_WRITE(MV_USB_PHY_TEST_CTRL0_REG, tmp);
	tmp = MV_PHYREG_READ(MV_USB_PHY_TEST_CTRL0_REG);
	//printk("U2PT0 0x%x\n", tmp);

	// Toggle VCOCAL_START bit of U2PLL
	udelay(200);
	tmp = MV_PHYREG_READ(MV_USB_PHY_PLL_CTRL_REG);
	tmp |= MV_USB_PHY_PLL_VCOCAL_START;
	MV_PHYREG_WRITE(MV_USB_PHY_PLL_CTRL_REG, tmp);
	tmp = MV_PHYREG_READ(MV_USB_PHY_PLL_CTRL_REG);
//	printk("U2PPLL = 0x%x\n", tmp);
	udelay(200);
	// Write VCOCAL_START bit back to 0
	tmp &= ~MV_USB_PHY_PLL_VCOCAL_START;
	MV_PHYREG_WRITE(MV_USB_PHY_PLL_CTRL_REG, tmp);
	tmp = MV_PHYREG_READ(MV_USB_PHY_PLL_CTRL_REG);
//	printk("2 U2PPLL = 0x%x\n", tmp);

	// Toggle REG_RCAL_START bit of U2PTX
	udelay(200);
	tmp = MV_PHYREG_READ(MV_USB_PHY_TX_CTRL_REG);
	tmp |= MV_USB_PHY_TX_RCAL_START; // Write REG_RCAL_START bit to 1
	MV_PHYREG_WRITE(MV_USB_PHY_TX_CTRL_REG, tmp);
	tmp = MV_PHYREG_READ(MV_USB_PHY_TX_CTRL_REG);
//	printk("U2PTX = 0x%x\n", tmp);
	udelay(200);
	tmp &= ~MV_USB_PHY_TX_RCAL_START; // Write REG_RCAL_START back to 0
	MV_PHYREG_WRITE(MV_USB_PHY_TX_CTRL_REG, tmp);
	tmp = MV_PHYREG_READ(MV_USB_PHY_TX_CTRL_REG);
//	printk("2 U2PTX = 0x%x\n", tmp);
	udelay(200);

	// Make sure PHY is ready
  	while(MV_PHYREG_READ(MV_USB_PHY_PLL_CTRL_REG & MV_USB_PHY_PLL_READY) == 0);

	// Disable Dynamic Clock Gating
	tmp = MV_PHYREG_READ(MV_USB_OTG_CLK_GATING_REG);
//	printk("1 U2O_CG 0x%x\n", tmp);
	tmp = 0xffffffff; // Disable Clock Gating
	MV_PHYREG_WRITE(MV_USB_OTG_CLK_GATING_REG, tmp);
	tmp = MV_PHYREG_READ(MV_USB_OTG_CLK_GATING_REG);
//	printk("2 U2O_CG 0x%x\n", tmp);
}
#endif


static int __init mv_usb_gadget_probe(struct platform_device *_dev) 
{
    struct mv_usb_dev       *mv_dev;
    int                     dev_no = 0, retval, i;
    uint_8                  error;
    struct resource	    *res;

    if(dev_no >= mvCtrlUsbMaxGet())
    {
        mvOsPrintf("mv_gadget_probe: device is not found\n");
        return -EINVAL;
    }
    mvOsPrintf("USB-%d Gadget driver probed\n", dev_no);

    if (the_controllers[dev_no]) 
    {
		mvOsPrintf("mv_dev_load: USB-%d controller is BUSY\n", dev_no);
		return -EBUSY;
    }

    /* alloc, and start init */
    mv_dev = mvOsMalloc (sizeof(struct mv_usb_dev));
    if (mv_dev == NULL)
    {
        mvOsPrintf("mv_dev_load: malloc failed\n");
        return -ENOMEM;
    }

    memset (mv_dev, 0, sizeof *mv_dev);

#ifdef CONFIG_PXA95x
	dvfm_register("U2O", &dvfm_dev_idx);
#elif defined(CONFIG_ANDROID)
	wake_lock_init(&idle_lock, WAKE_LOCK_IDLE, "mv_gadget_idle");
#endif
#ifdef CONFIG_ANDROID
	wake_lock_init(&suspend_lock, WAKE_LOCK_SUSPEND, "mv_gadget");
#endif

#if defined(CONFIG_ARCH_PXA)
    if (cpu_is_pxa935() || cpu_is_pxa95x() || cpu_is_pxa970()) {
        mv_dev->bus_clk = clk_get(&_dev->dev, "AXICLK");
        if (IS_ERR(mv_dev->bus_clk)) {
            retval = PTR_ERR(mv_dev->bus_clk);
            mvOsPrintf("mv_usb_gadget_probe: get AXICLK failed %d\n", retval);
            mvOsFree (mv_dev);
            return -ENODEV;
        }
        mv_dev->island_clk = clk_get(&_dev->dev, "IMUCLK");
        if (IS_ERR(mv_dev->island_clk)) {
            retval = PTR_ERR(mv_dev->island_clk);
            mvOsPrintf("mv_usb_gadget_probe: get IMUCLK failed %d\n", retval);
            clk_put(mv_dev->clk);
            mvOsFree(mv_dev);
            return -ENODEV;
        }
    }
#endif	/* CONFIG_ARCH_PXA */

    mv_dev->clk = clk_get(NULL, "U2OCLK");
    if (IS_ERR(mv_dev->clk)) {
	retval = PTR_ERR(mv_dev->clk);
	mvOsFree (mv_dev);
	return -ENODEV;
    }

    spin_lock_init (&mv_dev->lock);
    mv_dev->dev = &_dev->dev; 
    mv_dev->gadget.ops = &mv_usb_ops;
    mv_dev->gadget.is_dualspeed = 1;
	mv_dev->gadget.is_all_functions_enabled = 0;

    device_initialize(&mv_dev->gadget.dev);
    dev_set_name(&mv_dev->gadget.dev, "gadget");	
    mv_dev->gadget.dev.parent = &_dev->dev;
    mv_dev->gadget.dev.dma_mask = _dev->dev.dma_mask;

    mv_dev->gadget.dev.release = mv_usb_gadget_release ;
    mv_dev->gadget.name = driver_name;
    mv_dev->mv_usb_handle = NULL;
    mv_dev->dev_no = dev_no;

    dev_set_drvdata(&_dev->dev, mv_dev);
    the_controllers[dev_no] = mv_dev;

#ifdef MV_USB_VOLTAGE_FIX

    mv_dev->vbus_gpp_no = mvUsbGppInit(dev_no);
    mvOsPrintf("USB gadget device #%d: vbus_gpp_no = %d\n", 
                    dev_no, mv_dev->vbus_gpp_no);

    if(mv_dev->vbus_gpp_no != (MV_U8)N_A)
    {
        if (request_irq (IRQ_GPP_START + mv_dev->vbus_gpp_no, mv_usb_vbus_irq, IRQF_DISABLED, 
                     driver_name, mv_dev) != 0) 
        {
            mvOsPrintf("%s probe: request interrupt %d failed\n", 
                    driver_name, IRQ_GPP_START + mv_dev->vbus_gpp_no);
            return -EBUSY;
        }
    }

#endif /* MV_USB_VOLTAGE_FIX */

    /* Reset ARC USB device ????? */
    /* Reinit ARC USB device ????? */

    /* First of all. */
    _usb_device_set_bsp_funcs(&usbImportFuncs);

#if defined(USB_UNDERRUN_WA)

    if(wa_sram_parts > USB_SRAM_MAX_PARTS)
    {
        mvOsPrintf("Wrong <wa_sram_parts> param (%d): Valid range [1 .. %d]\n",
                wa_sram_parts, USB_SRAM_MAX_PARTS);
        return -EINVAL;
    }

    global_wa_funcs = &usbWaFuncs;
    global_wa_threshold = wa_threshold;
    global_wa_sram_parts = wa_sram_parts;

    sramBase = mv_sram_usage_get(&sramSize);
    if(sramBase == NULL)
    {
        mvOsPrintf("USB Underrun WA: No free SRAM space\n");
        return -ENOMEM;
    }
    memset(sramBase, 0, sramSize);
    
    idma = mv_usb_find_idma_engine(idma);
    if(idma != -1)
    {
        MV_REG_WRITE(IDMA_BYTE_COUNT_REG(idma), 0);
        MV_REG_WRITE(IDMA_CURR_DESC_PTR_REG(idma), 0);
        MV_REG_WRITE(IDMA_CTRL_HIGH_REG(idma), ICCHR_ENDIAN_LITTLE | ICCHR_DESC_BYTE_SWAP_EN);
        MV_REG_WRITE(IDMA_CTRL_LOW_REG(idma), USB_IDMA_CTRL_LOW_VALUE);
    }
    mvOsPrintf("USB underrun WA: idma=%d, streaming=%d, threshold=%d, SRAM: base=%p, size=%d, parts=%d\n", 
                idma, streaming, global_wa_threshold, sramBase, sramSize, global_wa_sram_parts);
#endif /* USB_UNDERRUN_WA */

    /*_usb_debug_set_flags(MV_USB_GADGET_DEBUG_FLAGS);*/

    res = platform_get_resource_byname(_dev, IORESOURCE_MEM, "u2o");
    if (res)
	mv_dev->regbase = (unsigned int) ioremap_nocache(res->start, res_size(res));
    if (!res || !mv_dev->regbase) {
            printk(KERN_ERR "Cannot get regbase 0x%x\n", 
		mv_dev->regbase);
            return -ENOMEM;
    }

    res = platform_get_resource_byname(_dev, IORESOURCE_MEM, "u2ophy");
    if (res)
	mv_dev->phybase = (unsigned int) ioremap_nocache(res->start, res_size(res));
    if (!res || !mv_dev->phybase) {
            printk(KERN_ERR "Cannot get phybase 0x%x\n",
		mv_dev->phybase);
            return -ENODEV;
    }

    IRQ_USB_CTRL[dev_no] = platform_get_irq(_dev, 0);
    if (!IRQ_USB_CTRL[dev_no]) {
            printk( KERN_ERR "Cannot get irq %x\n",
		    IRQ_USB_CTRL[dev_no]);
            return -ENODEV;
    }
    printk("u2o regbase 0x%x phybase 0x%x irq %d\n", mv_dev->regbase,
		    mv_dev->phybase, IRQ_USB_CTRL[dev_no]);

    mv_dev->info = _dev->dev.platform_data;
    if (mv_dev->info) {
	mv_dev->info->regbase = mv_dev->regbase;
	mv_dev->info->phybase = mv_dev->phybase;
    /* FIXME some board has the limitation that can't access
    * U2O if VBUS not valid, e.g. TTC DKB rev1.0 board */
    rely_on_vbus = mv_dev->info->rely_on_vbus;
    if (rely_on_vbus)
	mv_dev->info->clk_gating = 0;
    mv_dev->in_single = mv_dev->info->in_single;
    mv_dev->out_single = mv_dev->info->out_single;
    }
    mv_clock_enable(mv_dev, 1);

    /* Enable ARC USB device */
    retval = (int)_usb_device_init(dev_no, &mv_dev->mv_usb_handle);
    if (retval != USB_OK) 
    {
        mvOsPrintf("\nUSB Initialization failed. Error: %x", retval);
        return -EINVAL;
    } /* Endif */

    /* Self Power, Remote wakeup disable */
    _usb_device_set_status(mv_dev->mv_usb_handle, ARC_USB_STATUS_DEVICE, (1 << DEVICE_SELF_POWERED));

    /* Register all ARC Services */  
    error = _usb_device_register_service(mv_dev->mv_usb_handle, 
                                ARC_USB_SERVICE_BUS_RESET, mv_usb_bus_reset_service);
    if (error != USB_OK) 
    {
        mvOsPrintf("\nUSB BUS_RESET Service Registration failed. Error: 0x%x", error);
        return -EINVAL;
    } /* Endif */
   
    error = _usb_device_register_service(mv_dev->mv_usb_handle, 
                        ARC_USB_SERVICE_SPEED_DETECTION, mv_usb_speed_service);
    if (error != USB_OK) 
    {
        mvOsPrintf("\nUSB SPEED_DETECTION Service Registration failed. Error: 0x%x", 
                        error);
        return -EINVAL;
    } /* Endif */
         
    error = _usb_device_register_service(mv_dev->mv_usb_handle, 
                                ARC_USB_SERVICE_SUSPEND, mv_usb_suspend_service);
    if (error != USB_OK) 
    {
        mvOsPrintf("\nUSB SUSPEND Service Registration failed. Error: 0x%x", error);
        return -EINVAL;
    } /* Endif */

    error = _usb_device_register_service(mv_dev->mv_usb_handle, 
                                ARC_USB_SERVICE_SLEEP, mv_usb_suspend_service);
    if (error != USB_OK) 
    {
        mvOsPrintf("\nUSB SUSPEND Service Registration failed. Error: 0x%x", error);
        return -EINVAL;
    } /* Endif */    

    error = _usb_device_register_service(mv_dev->mv_usb_handle, 
                                ARC_USB_SERVICE_RESUME, mv_usb_resume_service);
    if (error != USB_OK) 
    {
        mvOsPrintf("\nUSB RESUME Service Registration failed. Error: 0x%x", error);
        return -EINVAL;
    } /* Endif */

    error = _usb_device_register_service(mv_dev->mv_usb_handle, 0, 
                                            mv_usb_ep0_complete_service);   
    if (error != USB_OK) 
    {
        mvOsPrintf("\nUSB ep0 TR_COMPLETE Service Registration failed. Error: 0x%x", error);
        return error;
    } /* Endif */

    for (i=1; i<ARC_USB_MAX_ENDPOINTS; i++)
    {
        error = _usb_device_register_service(mv_dev->mv_usb_handle, i, 
                                                    mv_usb_tr_complete_service);   
        if (error != USB_OK) 
        {
            mvOsPrintf("\nUSB ep0 TR_COMPLETE Service Registration failed. Error: 0x%x", error);
            return error;
        } /* Endif */
    }

    mv_dev->gadget.speed = USB_SPEED_UNKNOWN;

    if( mv_usb_reinit (mv_dev) != USB_OK)
        return -EINVAL;

#ifdef CONFIG_USB_OTG
    if (mv_dev->info->is_otg) {
    	mv_dev->transceiver = otg_get_transceiver();
    	mv_dev->cdev.transceiver = otg_get_transceiver();
    	mv_dev->gadget.is_otg = 1;
	if (!mv_dev->transceiver)
   		printk("failed to get otg transceiver\n");
    } else {
	printk("no otg support yet\n");
    }
#else
	pxa_set_usbdev(USB_B_DEVICE);
	/* Query status in order to clear existing status */
	pxa_query_vbus();
#endif
    mv_clock_enable(mv_dev, 0);
    INIT_WORK(&mv_dev->work, uevent_worker);
    init_timer(&mv_dev->timer);
    mv_dev->timer.function = mv_usb_clock_disable;

	INIT_WORK(&set_charger_type_work, do_set_charger_type);
	init_timer(&charger_timer);
	charger_timer.function = charger_timer_event;
#ifdef CONFIG_CHARGER_88PM860X
	mv_dev->set_charger_type = pm860x_set_charger_type;
#endif

	device_init_wakeup(&_dev->dev, 0);

    retval = sysfs_create_group(&_dev->dev.kobj, &mvgadget_attr_group);
    return retval; 
}

static int __exit mv_usb_gadget_remove(struct device *_dev)
{
    int                 i;
    struct mv_usb_dev   *mv_dev = dev_get_drvdata(_dev); 

    sysfs_remove_group(&_dev->kobj, &mvgadget_attr_group);
    mvOsPrintf("mv_usb_gadget_remove: mv_dev=%p, driver=%p\n", 
                mv_dev, mv_dev->driver);
    /* start with the driver above us */
    if (mv_dev->driver) 
    {
        /* should have been done already by driver model core */
        mvOsPrintf("pci remove, driver '%s' is still registered\n",
                    mv_dev->driver->driver.name);
        usb_gadget_unregister_driver (mv_dev->driver);
    }

    spin_lock (&mv_dev->lock);

    for (i=0; i<ARC_USB_MAX_ENDPOINTS; i++)
        _usb_device_unregister_service(mv_dev->mv_usb_handle, i);   

    /* Deregister all other services */
    _usb_device_unregister_service(mv_dev->mv_usb_handle, ARC_USB_SERVICE_BUS_RESET);   
    _usb_device_unregister_service(mv_dev->mv_usb_handle, ARC_USB_SERVICE_SPEED_DETECTION);

    _usb_device_unregister_service(mv_dev->mv_usb_handle, ARC_USB_SERVICE_SUSPEND);

    _usb_device_unregister_service(mv_dev->mv_usb_handle, ARC_USB_SERVICE_RESUME);

    _usb_device_shutdown(mv_dev->mv_usb_handle);

    spin_unlock (&mv_dev->lock);

#ifdef MV_USB_VOLTAGE_FIX

    if(mv_dev->vbus_gpp_no != (MV_U8)N_A)
    {
        free_irq (IRQ_GPP_START + mv_dev->vbus_gpp_no, mv_dev); 
    }

#endif /* MV_USB_VOLTAGE_FIX */

    the_controllers[mv_dev->dev_no] = 0;
    device_unregister (&mv_dev->gadget.dev);

    kfree(mv_dev);

    dev_set_drvdata(_dev, 0);

#ifdef CONFIG_PXA95x
	dvfm_unregister("U2O", &dvfm_dev_idx);
#elif defined(CONFIG_ANDROID)
	wake_lock_destroy(&idle_lock);
#endif
#ifdef CONFIG_ANDROID
	wake_lock_destroy(&suspend_lock);
#endif
    return 0;
}
 
#ifdef CONFIG_USB_OTG

void pxa9xx_gadget_init(void)
{
       struct mv_usb_dev* mv_dev = the_controllers[0];

       if (!mv_dev->transceiver) {
	       pr_err("no otg transceiver!\n");
	       return;
       }
       pr_debug("%s otg state %d\n", __func__, mv_dev->transceiver->state);
       if (rely_on_vbus)
		mv_usb_vbus_session(&mv_dev->gadget, 0);

       if (mv_dev->transceiver->state == OTG_STATE_B_PERIPHERAL) {
       		_usb_dci_vusb20_chip_initialize(mv_dev->mv_usb_handle);

       		mv_usb_device_start(mv_dev->mv_usb_handle);
       		mv_usb_start_ep0(mv_dev);
       }
}
#endif

/* dump IN endpoint status */
static int dump_ep = 3;
void dump_ep_req(int ep_num, int is_in)
{
	struct mv_usb_dev *mv_dev = the_controllers[0];
	int	tmp = mv_debug;
	void 	*print = usbImportFuncs.bspPrintf;
	usbImportFuncs.bspPrintf = mvOsPrintf;
	mv_debug = 1;
	if (is_in)
		_usb_ep_status(mv_dev->mv_usb_handle, ep_num, ARC_USB_SEND);
	else
		_usb_ep_status(mv_dev->mv_usb_handle, ep_num, ARC_USB_RECV);
	mv_debug = tmp;
	usbImportFuncs.bspPrintf = print;
}

/* global variables from 'regdump' */
static struct proc_dir_entry *usb_resource_dump;
static u32  usb_resource_dump_result;

int usb_resource_dump_read (char *buffer, char **buffer_location, off_t offset,
                            int buffer_length, int *zero, void *ptr);
int usb_resource_dump_write (struct file *file, const char *buffer,
                      unsigned long count, void *data) 
{
	char kbuf[8], vol[8];
	int index;

	if (count >= 8)
		return -EINVAL;
	if (copy_from_user(kbuf, buffer, count))
		return -EFAULT;
	index = (int)simple_strtoul(kbuf, NULL, 10);


	if ('-' == kbuf[0]) {
		/* set the ep num */
		memcpy(vol, kbuf+1, count-1);
		dump_ep = (int) simple_strtoul(vol, NULL, 16);
	}

	switch (index) {
	case 1:
            	usbImportFuncs.bspPrintf = mvOsPrintf;
		mv_debug = 1;
		break;
	case 2:
            	usbImportFuncs.bspPrintf = NULL;
		mv_debug = 0;
		break;
	case 3:
		usb_dump();
		break;
	case 4:
// VBUS #ifdef CONFIG_PXA_VBUS
		pr_info("%s\n", pxa_query_vbus() ? "VBUS HIGH" : "VBUS LOW");
//#endif
		break;
	case 5:
		dump_ep_req(dump_ep, 0);
		usb_resource_dump_read(kbuf, NULL, 0, 0, 0, NULL);
		break;
	case 6:
		conn_check = 0;
		break;

	default:
		return -EINVAL;
	}

    return count;
}

int usb_resource_dump_read (char *buffer, char **buffer_location, off_t offset,
                            int buffer_length, int *zero, void *ptr) 
{
    int         i;
    static int  count = 0;
    int		tmp = mv_debug;
    void 	*print = usbImportFuncs.bspPrintf;

    if(offset > 0)
        return 0;

    usbImportFuncs.bspPrintf = mvOsPrintf;
    mv_debug = 1;

    count++;
    usb_resource_dump_result = count;

    for(i=0; i<mvCtrlUsbMaxGet(); i++)
    {
        if(the_controllers[i] != NULL)
            mv_usb_show(the_controllers[i], 0xffff);
    }

    mv_debug = tmp;
    usbImportFuncs.bspPrintf = print;
 
    return sprintf(buffer, "%x\n", usb_resource_dump_result);
}

int usb_start_resource_dump(void)
{
  usb_resource_dump = create_proc_entry ("usb_dump" , 0666 , NULL);
  usb_resource_dump->read_proc = usb_resource_dump_read;
  usb_resource_dump->write_proc = usb_resource_dump_write;
  usb_resource_dump->nlink = 1;
  return 0;
}

static void mv_usb_gadget_shutdown(struct platform_device *_dev)
{
	printk("%s\n\n", __func__);
}

#ifdef CONFIG_PXA95x_SUSPEND
static int mv_usb_gadget_suspend(struct platform_device *_dev, pm_message_t state)
{
	if (is_cable_attached())
		printk(KERN_ALERT \
		"Warning: USB cable still connected while suspending!\n");

	return 0;
}

static int mv_usb_gadget_resume(struct platform_device *_dev)
{
	return 0;
}

#else
static int mv_usb_gadget_suspend(struct platform_device *_dev, pm_message_t state)
{
	struct mv_usb_dev *dev = the_controllers[0];

	if (conn_check && is_cable_attached()) {
		printk(KERN_ERR "USB cable connected, please try again!\n");
		return -EAGAIN;
	}
#ifdef CONFIG_USB_OTG
       	if (dev->transceiver) {
		pxa3xx_otg_require_bus(0);
		otg_set_peripheral(dev->transceiver, NULL);
	}
#endif
	udc_stop(&dev->cdev, &dev->gadget, dev->driver, 1);
   	_usb_device_stop(dev->mv_usb_handle);

	if (device_may_wakeup(&_dev->dev))
		enable_irq_wake(platform_get_irq(_dev, 0));

	return 0;
}

static int mv_usb_gadget_resume(struct platform_device *_dev)
{
	struct mv_usb_dev *dev = the_controllers[0];

	if (device_may_wakeup(&_dev->dev))
		disable_irq_wake(platform_get_irq(_dev, 0));

	mv_clock_enable(dev, 1);
#ifdef CONFIG_ARCH_PXA
	dev->info->phy_init(dev->phybase);
#endif
	_usb_dci_vusb20_chip_initialize(dev->mv_usb_handle);
#ifdef CONFIG_USB_OTG
    if (dev->transceiver)
		otg_set_peripheral(dev->transceiver, &dev->gadget);
#endif
   	mv_usb_device_start(dev->mv_usb_handle);
    	mv_usb_start_ep0(dev);

	if (is_cable_attached()) {
		mv_usb_connect_change(VBUS_HIGH);
	} else
		mv_clock_enable(dev, 0);
	return 0;
}
#endif
static struct platform_driver udc_driver = {
	.shutdown	= mv_usb_gadget_shutdown,
	.remove		= __exit_p(mv_usb_gadget_remove),
	.suspend	= mv_usb_gadget_suspend,
	.resume		= mv_usb_gadget_resume,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "pxa-u2o",
	},
};

MODULE_VERSION (DRIVER_VERSION);
MODULE_DESCRIPTION (DRIVER_DESC);
MODULE_AUTHOR ("Dima Epshtein");
MODULE_LICENSE ("GPL");

static int __init init (void)
{
    mvOsPrintf("%s: version %s loaded\n", driver_name, DRIVER_VERSION);
    usb_start_resource_dump();
    return platform_driver_probe(&udc_driver, mv_usb_gadget_probe);
}
module_init (init);

static void __exit cleanup (void)
{
    mvOsPrintf("%s: version %s unloaded\n", driver_name, DRIVER_VERSION);
    platform_driver_unregister(&udc_driver);
}
module_exit (cleanup);

