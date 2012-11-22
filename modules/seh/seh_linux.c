/*
 * (C) Copyright 2010 Marvell International Ltd.
 * All Rights Reserved
 */
 /*******************************************************************
  *
  *  FILE:	 seh_linux.c
  *
  *  DESCRIPTION: This file either serve as an entry point or a function
  * 				for writing or reading to/from the Linux SEH
  * 				Device Driver.
  *
  *
  *  HISTORY:
  *    April, 2008 - Rovin Yu
  *
  *
  *******************************************************************/

#include <linux/version.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/ioport.h>
#include <linux/fs.h>
#include <linux/pagemap.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/proc_fs.h>
#include <linux/fcntl.h>
#include <linux/poll.h>
#include <linux/aio.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/moduleparam.h>
#include <linux/miscdevice.h>

#include <mach/irqs.h>
#include <linux/version.h>
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,32)	
#include <mach/pxa-regs.h>
#else
#include <mach/regs-ost.h>
#endif

#ifdef CONFIG_PXA95x_DVFM
#include <mach/dvfm.h>
#endif
#include <mach/hardware.h>
#include <asm/system.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <linux/delay.h>
#include <linux/mtd/mtd.h>
#include <linux/platform_device.h>
#include <linux/mm.h>
#include <linux/vmalloc.h>
#include <linux/mman.h>
#include <linux/spinlock.h>
#include <asm/pgtable.h>
#include <linux/mfd/88pm860x.h>


#ifdef CONFIG_ANDROID_POWER
#include <linux/wakelock.h>
#endif

#ifdef CONFIG_PXA_MIPSRAM
#include <linux/mipsram.h>
#include <linux/time.h>
#endif /* CONFIG_PXA_MIPSRAM */

#define ACI_LNX_KERNEL
#include "seh_linux_kernel.h"
#include "seh_assert_notify.h"

#include <mach/mfp.h>
#include <asm/gpio.h>
#ifdef CONFIG_PXA_RAMDUMP
#include <asm/ptrace.h> /*pt_regs*/
#include <mach/ramdump.h>
#include <mach/ramdump_defs.h>
#define MMC_SD_DETECT_GPIO             MFP_PIN_GPIO47
#define MMC_SD_DETECT_INSERTED(VAL)   ((0x8000 & VAL)==0)
#ifdef RAMDUMP_PHASE_1_1 /* comes from ramdump h-files above: backwards compatibility with older kernel */
#define SEH_RAMDUMP_ENABLED 
#endif
#endif

#define read_reg(cwsbr) __raw_readl(cwsbr)
#define set_cwsbr(cwsbr, bitno)	__raw_writel(((__raw_readl(cwsbr)) | (bitno)), cwsbr)
#define clr_cwsbr(cwsbr, bitno)	__raw_writel(((__raw_readl(cwsbr)) & ~(bitno)), cwsbr)

#define REG_BIT_SET(BRN_CWSBR_REG,BITno)  { BRN_CWSBR_REG |=  (BITno); }
#define REG_BIT_CLR(BRN_CWSBR_REG,BITno)  { BRN_CWSBR_REG &= ~(BITno); }

extern void pxa9xx_platform_rfic_reset(void);

unsigned short seh_open_count = 0;
spinlock_t seh_init_lock = SPIN_LOCK_UNLOCKED;

#ifdef CONFIG_PXA_MIPSRAM
spinlock_t seh_mipsram_lock = SPIN_LOCK_UNLOCKED;
#endif/* CONFIG_PXA_MIPSRAM */

struct workqueue_struct *seh_int_wq = NULL;
struct work_struct seh_int_request;
struct seh_dev *seh_dev = NULL;

static int bStressTest = 0;
static int bStartRecovery = 0;

static int seh_open(struct inode *inode, struct file *filp);
static int seh_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg);
static void seh_dev_release(struct device *dev);
static int seh_remove(struct platform_device *dev);
static int seh_probe(struct platform_device *dev);
static ssize_t seh_read(struct file *filp, char *buf, size_t count,	loff_t *f_pos);
static unsigned int seh_poll(struct file *filp, poll_table *wait);
static int seh_mmap(struct file *file, struct vm_area_struct *vma);
static int seh_release(struct inode *inode, struct file *filp);
static void seh_dont_sleep(int dontSleepTrue);

void tel_assert_util(char* file, char* function, int line, int param);

#ifdef CONFIG_PXA_MIPSRAM
static void mipsram_vma_open(struct vm_area_struct *vma);
static void mipsram_vma_close(struct vm_area_struct *vma);
int mipsram_mmap(struct file *file, struct vm_area_struct *vma);
#endif/* CONFIG_PXA_MIPSRAM */

static const char* const seh_name="seh";
static struct file_operations seh_fops={
	.open = seh_open,
	.read = seh_read,
	.release	= seh_release,
	.ioctl = seh_ioctl,
	.poll = seh_poll,
	.mmap 		= seh_mmap,
	.owner = THIS_MODULE
};

static struct platform_device seh_device = {
       .name            = "seh",
       .id              = 0,
       .dev             = {
	      .release = seh_dev_release,
       },
};

static struct platform_driver seh_driver = {
	.probe	= seh_probe,
	.remove	= seh_remove,
	.driver = {
		.name	= "seh",
		.owner	= THIS_MODULE,
	},
};

static struct miscdevice seh_miscdev = {
	MISC_DYNAMIC_MINOR,
	"seh",
	&seh_fops,
};


#ifdef CONFIG_PXA_MIPSRAM
/*#define MIPSRAM_DEBUG*/

/* These are mostly for debug: do nothing useful otherwise*/
static struct vm_operations_struct mipsram_vm_ops = {
	.open = mipsram_vma_open,
	.close = mipsram_vma_close
};

static const struct file_operations mipsram_fops = {
	.owner = THIS_MODULE,
	.mmap  = mipsram_mmap,
};

static struct miscdevice mipsram_miscdev = {
	.minor		= MISC_DYNAMIC_MINOR,
	.name		= "mipsram",
	.fops		= &mipsram_fops,
};
#endif/* CONFIG_PXA_MIPSRAM */

static void EehSaveErrorInfo(EehErrorInfo* info);
#ifdef SEH_RAMDUMP_ENABLED
static int ramfile_mmap(struct file *file, struct vm_area_struct *vma);

static const struct file_operations ramfile_fops = {
	.owner = THIS_MODULE,
	.mmap  = ramfile_mmap,
};

static struct miscdevice ramfile_miscdev = {
	.minor		= MISC_DYNAMIC_MINOR,
	.name		= "ramfile",
	.fops		= &ramfile_fops,
};

#endif

#ifdef CONFIG_ANDROID_POWER
#define PM_TIMEOUT                 (60 * 10)/*sec*/
#define SEH_PM_WAKE_LOCK_TIME()    wake_lock_timeout(&eeh_assert_wakelock,(PM_TIMEOUT)*HZ);
#define SEH_PM_WAKE_LOCK()         wake_lock(&eeh_assert_wakelock);
#define SEH_PM_WAKE_UNLOCK()       wake_unlock(&eeh_assert_wakelock);
#define SEH_PM_WAKE_INIT()         wake_lock_init(&eeh_assert_wakelock, WAKE_LOCK_SUSPEND, EEH_DUMP_ASSERT_LOCK_NAME);
#define SEH_PM_WAKE_DESTROY()        wake_lock_destroy(&eeh_assert_wakelock);
static struct wake_lock eeh_assert_wakelock;
static long eeh_assert_wakelock_timeout = PM_TIMEOUT;
#else//CONFIG_ANDROID_POWER
#define SEH_PM_WAKE_LOCK_TIME()    /**/
#define SEH_PM_WAKE_LOCK()         /**/
#define SEH_PM_WAKE_UNLOCK()       /**/
#define SEH_PM_WAKE_INIT()         /**/
#define SEH_PM_WAKE_DESTROY()      /**/
#endif//CONFIG_ANDROID_POWER


//#define DEBUG_SEH_LINUX
#ifdef DEBUG_SEH_LINUX
#define DBGMSG(fmt,args...)	printk("SEH: "fmt,##args)
#define ERRMSG(fmt,args...) printk(KERN_ERR"SEH:"fmt, ##args)
#define	ENTER()			printk("SEH: ENTER %s\n",__FUNCTION__)
#define	LEAVE()			printk("SEH: LEAVE %s\n",__FUNCTION__)
#define	FUNC_EXIT()			printk("SEH: EXIT %s\n",__FUNCTION__)
#define DPRINT(fmt, args...) printk(KERN_INFO"SEH:"fmt, ##args)
#else
#define	DBGMSG(fmt,args...)	do{}while(0)
#define ERRMSG(fmt,args...) printk(KERN_ERR"SEH:"fmt, ##args)
#define	ENTER()		do{}while(0)
#define	LEAVE()		do{}while(0)
#define	FUNC_EXIT()	do{}while(0)
#define DPRINT(fmt, args...) printk(KERN_DEBUG"SEH:"fmt, ##args)
#endif

#if (LINUX_VERSION_CODE == KERNEL_VERSION(2, 6, 35))
#define PLATFORM_RFIC_RESET_LOCAL /*YANM: initially not implemented in K35*/
#endif
#if defined (PLATFORM_RFIC_RESET_LOCAL)
static unsigned int rfic_reset_gpio_pin = MFP_PIN_GPIO112;
static void pxa9xx_platform_rfic_reset_LOCAL(void)
{
	if (gpio_request(rfic_reset_gpio_pin, "RFIC reset")) {
		printk(KERN_ERR "RFIC gpio_request: failed!\n");
		return;
	}
	gpio_direction_output(rfic_reset_gpio_pin, 0);
	msleep(1);
	gpio_direction_output(rfic_reset_gpio_pin, 1);
	gpio_free(rfic_reset_gpio_pin);
}
#define pxa9xx_platform_rfic_reset pxa9xx_platform_rfic_reset_LOCAL
#endif//PLATFORM_RFIC_RESET_LOCAL


static void seh_int_assert_simulate_exec(unsigned long d)
{
	printk("KERNEL INTERRUPT ASSERT simulation\n");
	tel_assert_util((char*)__FILE__,(char*)__FUNCTION__,(int)__LINE__,0); //== ASSERT(0);
}

static void seh_int_assert_simulate_init(void)
{
	struct timer_list 	*ts_timer =
		(struct timer_list *)kmalloc(sizeof(struct timer_list), GFP_KERNEL);
	init_timer(ts_timer);
	ts_timer->function = seh_int_assert_simulate_exec;
	ts_timer->expires = jiffies+15;
	ts_timer->data = (unsigned long)0;
	add_timer(ts_timer);
}

void seh_cp_assert_simulate(void)
{
	//Uses 2 ACIPC registers WDR and ISRW to harm with invalid data 0xFFFFFFFF
	resource_size_t  ACIPC_ADDR = 0x42403004;
	volatile void __iomem * va_acipc;
	va_acipc = ioremap_nocache((resource_size_t)ACIPC_ADDR, 2*(sizeof(unsigned long)));
	if (va_acipc == NULL)
		printk("COMM ASSERT simulation - FAILED to map ACIPC\n");
	else
	{
		printk("COMM ASSERT simulation over ACIPC\n");
		__raw_writel(0xFFFFFFFF, va_acipc);
		__raw_writel(0xFFFFFFFF, va_acipc+sizeof(unsigned int));
		iounmap((void *)va_acipc);
	}
}
EXPORT_SYMBOL(seh_cp_assert_simulate);


static void seh_mipsram_mark_end_of_log(void)
{
#ifdef CONFIG_PXA_MIPSRAM
	struct timeval tv;

	/* Mark end of log + time of day - this function is unsafe to call from ISR */
	do_gettimeofday(&tv);
	MIPS_RAM_MARK_END_OF_LOG(MIPSRAM_get_descriptor(), (unsigned long)tv.tv_sec);
#endif/*CONFIG_PXA_MIPSRAM*/
}

static void seh_mipsram_stop(bool sign_log)
{
#ifdef CONFIG_PXA_MIPSRAM
	unsigned long flags;

	spin_lock_irqsave(&seh_mipsram_lock, flags);

	/* Mark end of log */
	MIPSRAM_Trace(MIPSRAM_LOG_END_MARK_EVENT);

	/* stop MIPSRAM logger */
	MIPSRAM_get_descriptor()->buffer = NULL;

	if (sign_log)
	{
		seh_mipsram_mark_end_of_log();
	}

	spin_unlock_irqrestore(&seh_mipsram_lock, flags);
#endif/* CONFIG_PXA_MIPSRAM */
}

static void seh_mipsram_start(void)
{
#ifdef CONFIG_PXA_MIPSRAM
	/* Start MIPSRAM buffer */
	MIPSRAM_clear();
	/* start MIPSRAM logger */
	MIPSRAM_get_descriptor()->buffer = MIPSRAM_get_descriptor()->bufferVirt;
#endif/*CONFIG_PXA_MIPSRAM*/
}

void seh_mipsram_add_trace(unsigned long trace)
{
#ifdef CONFIG_PXA_MIPSRAM
	unsigned long flags;

	spin_lock_irqsave(&seh_mipsram_lock, flags);

	/* Add custom trace */
	MIPSRAM_Trace(trace);

	spin_unlock_irqrestore(&seh_mipsram_lock, flags);
#endif/*CONFIG_PXA_MIPSRAM*/
}


//Set CWSBR to capture WDT as interrupt
/*ENABLE_COMM_WDT_INTERRUPT
* capturing the interrupt on BICU and not generating reset to Comm
*  -------------------------
* CWSBR[CWSB] = 0
* CWSBR[CWRE] = 0
*/
void CommWDTInterruptSet(void)
{
	ENTER();

	spin_lock(&seh_init_lock);

	set_cwsbr(seh_dev->cwsbr, XLLP_CWSBR_CWSB);

	enable_irq(IRQ_COMM_WDT);

	clr_cwsbr(seh_dev->cwsbr, XLLP_CWSBR_CWRE); //disable next GB WDT int to generate auto comm reset

	clr_cwsbr(seh_dev->cwsbr, XLLP_CWSBR_CWSB); //enable src GB WDT int

	spin_unlock(&seh_init_lock);

	LEAVE();
}




//Clear CWSBR from capture WDT as interrupt , make it ready for next event
//Should be called after Interrupt is handled
/*CLEAR_REENABLE_COMM_WDT_INTERRUPT after serving (handling)
 *
 */

void CommWDTInterruptClearDuringSilentReset(void)
{
	ENTER();

	spin_lock(&seh_init_lock);

	set_cwsbr(seh_dev->cwsbr, XLLP_CWSBR_CWSB); //clr GB WDT int src

	//REG_BIT_SET(ICMR2, XLLP_ICMR2_BIT3); //Unmask GW WTD Interrupt
	enable_irq(IRQ_COMM_WDT);

	clr_cwsbr(seh_dev->cwsbr, XLLP_CWSBR_CWRE); //disable next GB WDT int to generate auto comm reset

	clr_cwsbr(seh_dev->cwsbr, XLLP_CWSBR_CWSB); //enable src GB WDT int

	bStartRecovery = 0;

	spin_unlock(&seh_init_lock);

	LEAVE();
}


/*
 * Copy Arbel and Grayback Image from Flash to DDR
 */
#if defined(RENEW_COMM_DDR_IMAGE_FROM_FLASH)
static void CopyCommImageFromFlashtoDDR(EehCopyCommImageFromFlashParam* param)
{
	struct mtd_info *mtd = NULL;
	unsigned int length = 0;
	static unsigned int va_arbel;
	static unsigned int va_gb;
	void __iomem *tai;

	tai = ioremap_nocache(TAI,IORESOURCE_IO);
	if( tai == NULL)
		return;

	if ((read_reg(tai) & 0xFFF) != 0)
	{
		ERRMSG("TAI: 0x%x. DDR blocked: cannot copy Comm Image from Flash to DDR!\n", read_reg(tai));
		iounmap(tai);
		return;
	}
	iounmap(tai);

#ifdef CONFIG_ANDROID_POWER
	printk("[JK]: use mount point 1 instead of 2\n");
	mtd = get_mtd_device(NULL, 1);
#else
	mtd = get_mtd_device(NULL, 2);
#endif
	// Copy Arbel Image from flash to DDR address: 0xBFA00000
	va_arbel = (unsigned int)ioremap_nocache(param->ArbelAddr, param->ArbelSize);
	if (va_arbel)
	{
		mtd->read(mtd, 0x0, param->ArbelSize, &length, (void*)va_arbel);
		iounmap((void *)va_arbel);
	}


	// Copy Grayback Image from flash to DDR address: 0xBFF00000
	va_gb = (unsigned int)ioremap_nocache(param->GraybackAddr, param->GraybackSize);
	if (va_gb)
	{
		mtd->read(mtd, param->ArbelGraybackOffset, param->GraybackSize, &length, (void*)va_gb);
		iounmap((void *)va_gb);
	}

	DPRINT("%s: successfully\n", __FUNCTION__);
}
#endif //RENEW_COMM_DDR_IMAGE_FROM_FLASH

/*
  * The top part for SEH interrupt handler.
  */
irqreturn_t seh_int_handler_low(int irq, void *dev_id/*, struct pt_regs *regs*/)
{
    //disable_irq(IRQ_COMM_WDT);
	disable_irq_nosync(IRQ_COMM_WDT);

	/* stop MIPSRAM */
	seh_mipsram_stop(FALSE);
    queue_work(seh_int_wq, &seh_int_request);
    //SEH_PM_WAKE_LOCK();  //SEH_PM_WAKE_LOCK_TIME();
    return IRQ_HANDLED;
}

/*
 * The bottom part for SEH interrupt handler
 */
void seh_int_handler_high(struct work_struct *data)
{
        (void)data; /*unused*/

	if (seh_dev)
	{
		if (down_interruptible(&seh_dev->read_sem))
		{
			ERRMSG("%s: fail to down semaphore\n", __FUNCTION__);
			enable_irq(IRQ_COMM_WDT);
			return;
		}
		SEH_PM_WAKE_LOCK();  //SEH_PM_WAKE_LOCK_TIME();

		/* Mark MIPS_RAM end of log with EPOCH timestamp */
		seh_mipsram_mark_end_of_log();

		/* Config PMIC WatchDog to guaranty reset in 16sec */
		/*pm860x_reg_write(pm8607_i2c_client, 0x0B, 0x5E);*/
		pm860x_codec_reg_write(0x0B, 0x5A);

		seh_dev->msg.msgId = EEH_WDT_INT_MSG;
		up(&seh_dev->read_sem);
		wake_up_interruptible(&seh_dev->readq);
	}

}

int seh_api_ioctl_handler(unsigned long arg)
{
    EehApiParams   params;
    EEH_STATUS       status = EEH_SUCCESS;

    if (copy_from_user(&params, (EehApiParams*)arg, sizeof(EehApiParams)))
        return -EFAULT;

    DPRINT("seh_api_ioctl_handler %d\n ",params.eehApiId);

    switch (params.eehApiId)// specific EEH API handler
    {

    case _EehInit:


        DBGMSG("Kernel Space EehInit Params:No params\n");

         /* Set CWSBR to capture WDT as interrupt  */
		CommWDTInterruptSet();

        if (copy_to_user(&( (EehApiParams*)arg)->status,&status, sizeof(UINT32)))
            return -EFAULT;


        break;

    case _EehDeInit:

        DBGMSG("Kernel Space EehDeInit Params:No params\n");

        if (copy_to_user(&( (EehApiParams*)arg)->status,&status, sizeof(UINT32)))
            return -EFAULT;

        break;

	case _EehExecRFICReset:
		/* Reset RF module */
		DBGMSG("Resetting RFIC...\n");
		pxa9xx_platform_rfic_reset();
		break;

    case _EehInsertComm2Reset:
    {
    	EehInsertComm2ResetParam resetParams;

		if (copy_from_user(&resetParams, params.params,sizeof(EehInsertComm2ResetParam)))
			return -EFAULT;

    	DBGMSG("Kernel Space EehInsertComm2Reset asserttype: %d\n", resetParams.AssertType);

		if(resetParams.AssertType == EEH_CP_WDT)
		{
			LoadTableType *pLoadTable;
			pLoadTable = ioremap_nocache((UINT32)ADDR_CONVERT(LOAD_TABLE_FIX_ADDR), sizeof(LoadTableType));          //-+ GQ00000270 telephony: fix error handling mmap +-
			if(pLoadTable != NULL)
			{
				UINT32 *resetTypeAddr = (UINT32*)&(pLoadTable->apps2commDataBuf);
				*resetTypeAddr = RESET_BASIC_2;
				iounmap((void*)pLoadTable);
				/* go ahead like EEH_CP_ASSERT reset */
	    	   	set_cwsbr(seh_dev->cwsbr, XLLP_CWSBR_CWRE);
			}
			return 0;
		}
		/* verify if !PENDING in bit_2 */
		spin_lock(&seh_init_lock);

		if(bStressTest || resetParams.AssertType == EEH_AP_ASSERT)
		{
			disable_irq(IRQ_COMM_WDT);
			set_cwsbr(seh_dev->cwsbr, XLLP_CWSBR_CWRE);
		}
		else
		{
		#if 0
	    	if((read_reg(seh_dev->cwsbr) & XLLP_CWSBR_CWSBV) == 0)
	    	{
	    		/* Can't reset */
	    		ERRMSG("%s: cannot reset COMM now!\n", __FUNCTION__);
	    		DPRINT("seh_dev->cwsbr: 0x%x\n", read_reg(seh_dev->cwsbr));
	    		enable_irq(IRQ_COMM_WDT);
	    		status = EEH_ERROR;
	    	}
	    	else
		#endif
	    	   	set_cwsbr(seh_dev->cwsbr, XLLP_CWSBR_CWRE);
    	}

    	bStartRecovery = 1;

		spin_unlock(&seh_init_lock);

        if (copy_to_user(&( (EehApiParams*)arg)->status,&status, sizeof(UINT32)))
            return -EFAULT;

		/* Reset RF module */
		DBGMSG("Resetting RFIC...\n");
		pxa9xx_platform_rfic_reset();

        break;
	}
    case _EehReleaseCommFromReset:

    	DBGMSG("Kernel Space EehReleaseCommFromReset Params:No params\n");

    	CommWDTInterruptClearDuringSilentReset();

        if (copy_to_user(&( (EehApiParams*)arg)->status,&status, sizeof(UINT32)))
            return -EFAULT;

        break;

    case _EehExecReset:

    	DBGMSG("Kernel Space EehExecReset Params:No params\n");

    	{
    		UINT32 matchvalue;

			spin_lock(&seh_init_lock);

    		/* Configure the OSMR3: plus 1000 agaist current oscr0   */
    		matchvalue = OSCR + 1000;

    		/* Disable interrupts on the specified Match register: OSMR3 */

    		OIER &= ~(OIER_E3 | XLLP_OIER_RESERVED_BITS);

		    /*
		       * Clear any interrupt on the specified Match register: OSMR3
		       */
		    OSSR = OSSR_M3;

		    /*
			* Set up the match register to expire when the oscr0 reaches
		       * the next match interval.
			*/
		    OSMR3 = matchvalue;

		    /*
		       * Enable the Match register interrupt on
		       */
		    REG_BIT_SET(OIER,OIER_E3);

		    /* Enable OS Timer Watchdog Match Enable Register: OWER */
		    REG_BIT_SET(OWER,OWER_WME);

		    spin_unlock(&seh_init_lock);

    	}

        if (copy_to_user(&( (EehApiParams*)arg)->status,&status, sizeof(UINT32)))
            return -EFAULT;

        break;

    case _EehCopyCommImageFromFlash:
#if defined(RENEW_COMM_DDR_IMAGE_FROM_FLASH)
    	{
			EehCopyCommImageFromFlashParam CopyParam;

			if (copy_from_user(&CopyParam, params.params, sizeof(EehCopyCommImageFromFlashParam)))
				return -EFAULT;

			DBGMSG("Kernel Space EehCopyCommImageFromFlash Params: \
					ArbelAddr = 0x%x, ArbelSize = %d, \
					GraybackAddr = 0x%x, GraybackSize = %d \
					ArbelGraybackOffset = 0x%x\n",
				   CopyParam.ArbelAddr, CopyParam.ArbelSize,
				   CopyParam.GraybackAddr, CopyParam.GraybackSize,
				   CopyParam.ArbelGraybackOffset);

			spin_lock(&seh_init_lock);

	    	CopyCommImageFromFlashtoDDR(&CopyParam);

			spin_unlock(&seh_init_lock);

	        if (copy_to_user(&( (EehApiParams*)arg)->status,&status, sizeof(UINT32)))
	            return -EFAULT;
        }
#endif //RENEW_COMM_DDR_IMAGE_FROM_FLASH
        break;
    case _EehGetTid:
        if ((((EehApiParams*)arg)->params==NULL) || copy_to_user(((EehApiParams*)arg)->params, &current->pid, sizeof(long)))
            return -EFAULT;
        break;

    case _EehTestAssertKernel:
        tel_assert_util((char*)__FILE__,(char*)__FUNCTION__,(int)__LINE__,0);
	break;                                                                 //-+ GQ00000296 telephony: coverity fixes in osa hwmap nvms eeh fake +-

    case _EehTestAssertKernelIRQ:
        seh_int_assert_simulate_init();
        break;

    case _EehTestCpAssert:
        seh_cp_assert_simulate();
        break;

    default:
        ERRMSG("WRONG Api = %d (params.eehApiId)\n",params.eehApiId);
        return -EFAULT;
    }
    return 0;
}

static int seh_probe(struct platform_device *dev)
{
	int ret;

	ENTER();

    seh_int_wq = create_workqueue("seh_rx_wq");

    INIT_WORK(&seh_int_request, seh_int_handler_high);

	seh_dev = (struct seh_dev*)kzalloc(sizeof(struct seh_dev), GFP_KERNEL);
	if (seh_dev == NULL)
	{
		ERRMSG("seh_probe: unable to allocate memory\n");
		return -ENOMEM;
	}

	seh_dev->cwsbr = ioremap_nocache(GCWSBR, IORESOURCE_IO);
	if (seh_dev->cwsbr == NULL) {
		ERRMSG("seh_probe: failed to ioremap registers\n");
		kfree(seh_dev);
		return -ENXIO;
	}

	init_waitqueue_head(&(seh_dev->readq));
	sema_init(&seh_dev->read_sem, 1);
	seh_dev->dev = dev;

	ret = misc_register(&seh_miscdev);
	if (ret)
		ERRMSG("seh_probe: failed to call misc_register\n");

	LEAVE();

	return ret;
}

static int __init seh_init(void)
{
	int ret;

	ENTER();

	ret = platform_device_register(&seh_device);
	if(!ret)
	{
		ret = platform_driver_register(&seh_driver);
		if(ret)
		{
			ERRMSG("Cannot register SEH platform driver\n");
			platform_device_unregister(&seh_device);
		}
	}else{
		ERRMSG("Cannot register SEH platform device\n");
	}

#ifdef CONFIG_PXA_MIPSRAM
	/* register MIPSRAM device */
	ret = misc_register(&mipsram_miscdev);
    if (ret) {
		printk(KERN_ERR "%s can't register device, err=%d\n", __func__, ret);
    } else {
		printk(KERN_DEBUG "mipsram init successful\n");
	}
#endif /* CONFIG_PXA_MIPSRAM */

#ifdef SEH_RAMDUMP_ENABLED
	/* register MIPSRAM device */
	ret = misc_register(&ramfile_miscdev);
	if (ret)
		printk(KERN_ERR "%s can't register device, err=%d\n", __func__, ret);
#endif /* SEH_RAMDUMP_ENABLED */

    SEH_PM_WAKE_INIT();
	LEAVE();

	return ret;
}

static void __exit seh_exit(void)
{
	ENTER();

	platform_driver_unregister(&seh_driver);
	platform_device_unregister(&seh_device);

	SEH_PM_WAKE_DESTROY();
	LEAVE();

}

static int seh_open(struct inode *inode, struct file *filp)
{
	int status;

	ENTER();

	/*  Save the device pointer */
	if (seh_dev)
		filp->private_data = seh_dev;
	else
		return -ERESTARTSYS;
	/*
	 * Only to prevent kernel preemption.
	 */
	spin_lock(&seh_init_lock);


	/* if seh driver is opened already. Just increase the count */
	if (seh_open_count) {
		seh_open_count++;
		spin_unlock(&seh_init_lock);
		return 0;
	}

	status = request_irq(IRQ_COMM_WDT, seh_int_handler_low, 0,
			seh_name, NULL);
	if(status)
	{
		DBGMSG("seh_open: cannot register the COMM WDT interrupt\n");
		spin_unlock(&seh_init_lock);
		return status;
	}
	disable_irq(IRQ_COMM_WDT);

	seh_open_count = 1;

	spin_unlock(&seh_init_lock);

	LEAVE();
	return 0;
}

static ssize_t seh_read(struct file *filp, char *buf, size_t count,
		loff_t *f_pos)
{
	struct seh_dev *dev;
	//static int client_count =0; - to be used for the CLIENT: "Assert Message Display"

	ENTER();

	//client_count ++;
	dev = (struct seh_dev *) filp->private_data;
	if (dev == NULL)
		return -ERESTARTSYS;

	if (down_interruptible(&dev->read_sem))
		return -ERESTARTSYS;

	while (dev->msg.msgId == EEH_INVALID_MSG)
	{ /* nothing to read */
		up(&dev->read_sem); /* release the lock */
		if (filp->f_flags & O_NONBLOCK)
			return -EAGAIN;
		DPRINT("\"%s\" reading: going to sleep\n", current->comm);
		if (wait_event_interruptible(dev->readq, (dev->msg.msgId != EEH_INVALID_MSG)))
			return -ERESTARTSYS; /* signal: tell the fs layer to handle it */
		/* otherwise loop, but first reacquire the lock */
		if (down_interruptible(&dev->read_sem))
			return -ERESTARTSYS;
	}

	/* ok, data is there, return something */
	count = min(count, sizeof(EehMsgStruct));
	if (copy_to_user(buf, (void*)&(dev->msg), count))
	{
		up (&dev->read_sem);
		return -EFAULT;
	}

	DPRINT("\"%s\" did read %li bytes, msgId: %d\n",current->comm, (long)count, dev->msg.msgId);
	//--client_count;

	//if(client_count ==0) {
		/* reset the msg info */
		memset(&(dev->msg), 0, sizeof(EehMsgStruct));
	//}
	//else {
	//	wake_up_interruptible(&seh_dev->readq);
	//}
	up(&dev->read_sem);
	LEAVE();
	return count;
}


static int seh_ioctl(struct inode *inode, struct file *filp,
		unsigned int cmd, unsigned long arg)
{
	int ret;
	struct seh_dev *dev;

	if(_IOC_TYPE(cmd) != SEH_IOC_MAGIC) {
		ERRMSG("seh_ioctl: seh magic number is wrong!\n");
		return -ENOTTY;
	}

	dev = (struct seh_dev *) filp->private_data;
	if (dev == NULL)
		return -EFAULT;

	DBGMSG("seh_ioctl,cmd=0x%x\n",cmd);

	ret = 0;

	switch(cmd)
	{
		case SEH_IOCTL_API:
            ret = seh_api_ioctl_handler(arg);
            break;
		case SEH_IOCTL_TEST:
			DPRINT("SEH_IOCTL_TEST\n");
			if (down_interruptible(&seh_dev->read_sem))
				return -EFAULT;

			bStressTest = 1;

			seh_dev->msg.msgId = EEH_WDT_INT_MSG;
			up(&seh_dev->read_sem);
			wake_up_interruptible(&seh_dev->readq);
			break;
		case SEH_IOCTL_ATC_OK:
			DPRINT("Receive SEH_IOCTL_ATC_OK\n");
			if (down_interruptible(&seh_dev->read_sem))
				return -EFAULT;

			seh_dev->msg.msgId = EEH_ATC_OK_MSG;
			up(&seh_dev->read_sem);
			wake_up_interruptible(&seh_dev->readq);
			break;
		case SEH_IOCTL_APP_ASSERT:
		{
			EehAppAssertParam param;

			/* First of all, stop MIPSRAM logging */
			seh_mipsram_stop(TRUE);

			/* Config PMIC WatchDog to guaranty reset in 16sec */
			/*pm860x_reg_write(pm8607_i2c_client, 0x0B, 0x5E);*/
			pm860x_codec_reg_write(0x0B, 0x5A);

			DPRINT("Receive SEH_IOCTL_APP_ASSERT\n");
			memset(&param, 0, sizeof(EehAppAssertParam));
			if (copy_from_user(&param, (EehAppAssertParam*)arg, sizeof(EehAppAssertParam)))
        		return -EFAULT;

			if (down_interruptible(&seh_dev->read_sem))
				return -EFAULT;

			seh_dev->msg.msgId = EEH_AP_ASSERT_MSG;
			strncpy(seh_dev->msg.msgDesc, param.msgDesc, sizeof(seh_dev->msg.msgDesc));          //-+ GQ00000296 telephony: coverity fixes in osa hwmap nvms eeh fake +-
			seh_dev->msg.msgDesc[sizeof(seh_dev->msg.msgDesc) - 1] = 0;          //-+ GQ00000296 telephony: coverity fixes in osa hwmap nvms eeh fake +-
			up(&seh_dev->read_sem);
			wake_up_interruptible(&seh_dev->readq);

			break;
		}
		case SEH_IOCTL_AUDIOSERVER_REL_COMP:
			DPRINT("Receive SEH_IOCTL_AUDIOSERVER_REL_COMP\n");
			if (down_interruptible(&seh_dev->read_sem))
				return -EFAULT;

			seh_dev->msg.msgId = EEH_AUDIOSERVER_REL_COMP_MSG;
			up(&seh_dev->read_sem);
			wake_up_interruptible(&seh_dev->readq);
			break;

		case SEH_IOCTL_MIPSRAM_START:
			seh_mipsram_start();
			break;

		case SEH_IOCTL_MIPSRAM_STOP:
			seh_mipsram_stop(TRUE);
			break;

		case SEH_IOCTL_MIPSRAM_ADD_TRACE:
			seh_mipsram_add_trace(arg);
			break;

		case SEH_IOCTL_MIPSRAM_DUMP:
			/* First of all, stop MIPS_RAM logging */
			seh_mipsram_stop(TRUE);

			/* Trigger dump */
			seh_dev->msg.msgId = EEH_MIPSRAM_DUMP;
			up(&seh_dev->read_sem);
			wake_up_interruptible(&seh_dev->readq);
			break;

		case SEH_IOCTL_DONT_SLEEP:
            seh_dont_sleep(TRUE);
			break;
		case SEH_IOCTL_SLEEP_ALLOWED:
            seh_dont_sleep(FALSE);
			break;
		case SEH_IOCTL_EEH_TIMER_EXPIRED:
            //Msg from EEH-timer obtained
            //Send it back into the EEH-Task for the handling
			if (down_interruptible(&seh_dev->read_sem))  return -EFAULT;
			seh_dev->msg.msgId = EEH_TIMER_EXPIRED_MSG;
			seh_dev->msg.msgDesc[0] = 77;
			up(&seh_dev->read_sem);
			wake_up_interruptible(&seh_dev->readq);
			break;
		case SEH_IOCTL_EMULATE_PANIC:
			panic("EEH emulated panic\n");
			break;
		case SEH_IOCTL_SET_ERROR_INFO:
		{
			EehErrorInfo param;
			if (copy_from_user(&param, (EehErrorInfo*)arg, sizeof(EehErrorInfo)))
        			return -EFAULT;
			EehSaveErrorInfo(&param);
			break;
		}

		case SEH_IOCTL_GET_GPIO_SD_DETECT_INSERTED:
#ifdef CONFIG_PXA_RAMDUMP
			//!!! Keep in sync with the saarb.c::saarb_init_mmc::gpio_cd=MFP_PIN_GPIO47 !!!
			ret = gpio_get_value( mfp_to_gpio(MMC_SD_DETECT_GPIO) );
			ret = MMC_SD_DETECT_INSERTED(ret);
#else
			ret = 0; //run like there is no sd-card => no RAMDUMP
#endif
			break;

		default:
			ret = -ENOTTY;
			break;
	}

	return ret;
}

static unsigned int seh_poll(struct file *filp, poll_table *wait)
{
	struct seh_dev *dev = filp->private_data;
	unsigned int mask = 0;

	ENTER();

	poll_wait(filp, &dev->readq, wait);

	if(dev->msg.msgId != EEH_INVALID_MSG)   /* read finished */
		mask |= POLLIN | POLLRDNORM;

	LEAVE();

	return mask;
}

/* device memory map method */
static void seh_vma_open(struct vm_area_struct *vma)
{
  DBGMSG("SEH OPEN 0x%lx -> 0x%lx (0x%lx)\n",vma->vm_start, vma->vm_pgoff<<PAGE_SHIFT, vma->vm_page_prot);
}
static void seh_vma_close(struct vm_area_struct *vma)
{
  DBGMSG("SEH CLOSE 0x%lx -> 0x%lx\n",vma->vm_start, vma->vm_pgoff<<PAGE_SHIFT);
}

static struct vm_operations_struct vm_ops =
{
  .open=seh_vma_open,
  .close=seh_vma_close
};

/*
 vma->vm_end, vma->vm_start : specify the user space process address range assigned when mmap has been called;
 vma->vm_pgoff - is the physical address supplied by user to mmap in the last argument (off)
             However, mmap restricts the offset, so we pass this shifted 12 bits right.
 */
static int seh_mmap(struct file *file, struct vm_area_struct *vma)
{
    unsigned long size = vma->vm_end - vma->vm_start;
    unsigned long pa   = __phys_to_pfn((unsigned long)ADDR_CONVERT(__pfn_to_phys(vma->vm_pgoff)));

    vma->vm_pgoff = pa;

    /* we do not want to have this area swapped out, lock it */
    vma->vm_flags |= (VM_RESERVED|VM_IO);
    vma->vm_page_prot=pgprot_noncached(vma->vm_page_prot);

    if(io_remap_pfn_range(vma,
           vma->vm_start,
           pa,/* physical page index */
           size,
	   vma->vm_page_prot))
    {
            ERRMSG("remap page range failed\n");
            return -ENXIO;
    }
    vma->vm_ops=&vm_ops;
    seh_vma_open(vma);
    return(0);
}

static int seh_release(struct inode *inode, struct file *filp)
{
	ENTER();

	spin_lock(&seh_init_lock);
	/* Just release the resource when access count is 0 */
	if (!--seh_open_count) {
		free_irq(IRQ_COMM_WDT, NULL);
	}
	spin_unlock(&seh_init_lock);

	return 0;
}


static void seh_dev_release(struct device *dev)
{
	return;
}

static int seh_remove(struct platform_device *dev)
{
	ENTER();

	misc_deregister(&seh_miscdev);
	if (seh_dev->cwsbr)
		iounmap(seh_dev->cwsbr);
	kfree(seh_dev);
	destroy_workqueue(seh_int_wq);

	LEAVE();
	return 0;
}

int seh_draw_panic_text(const char *panic_text, size_t panic_len, int do_timer)
{
	int status = 0;
	/* stop MIPSRAM logging */
	seh_mipsram_stop(TRUE);
	printk("Enter %s\n", __FUNCTION__);
	printk(KERN_CRIT"%s:%d:%d\n", panic_text, panic_len, do_timer);

	{
		/*Customer should add draw_panic_text function here */
	}

	seh_dev->msg.msgId = EEH_AP_ASSERT_MSG;
	strncpy(seh_dev->msg.msgDesc, panic_text, sizeof(seh_dev->msg.msgDesc));          //-+ GQ00000296 telephony: coverity fixes in osa hwmap nvms eeh fake +-
	seh_dev->msg.msgDesc[sizeof(seh_dev->msg.msgDesc) - 1] = 0;            //-+ GQ00000296 telephony: coverity fixes in osa hwmap nvms eeh fake +-
	up(&seh_dev->read_sem);
	wake_up_interruptible(&seh_dev->readq);

	if(in_interrupt() || in_irq())
		printk("ASSERT on INT or IRQ context/n");
	else
	{
		while(!bStartRecovery)
			msleep_interruptible(10);
	}
	return status;
}

/* Kernel mode callstack implementation
* Depends also upon CONFIG_FRAME_POINTER and CONFIG_PRINTK presence
 */
#include <linux/kallsyms.h>

typedef struct
{
  char* string;
  int   len;
  int   index;
} StrDesc;

/* Overloaded implementation is to feed the output into a string buffer (original code does printk)*/
#if defined (BACKTRACE_TEL_ENABLE)
extern void BackTrace_k(StrDesc* sd);
#else
#define BackTrace_k(X)  /**/
#endif

/* The original code is in linux/lib/vsprintf.c */
int print_bk(StrDesc* sd, const char *fmt, ...)
{
	va_list args;
	int i;
	va_start(args, fmt);
	i=vsnprintf(&sd->string[sd->index], sd->len-sd->index, fmt, args);
	va_end(args);
        if((sd->index+=i)>sd->len) sd->index=sd->len;
	return i;
}

/* The original code is linux/kernel/kallsyms.c */
static void print_symbol_bk(StrDesc* sd, const char *fmt, unsigned long address)
{
	char buffer[KSYM_SYMBOL_LEN];

	sprint_symbol(buffer, address);
	print_bk(sd, fmt, buffer);
}

/* The original code is linux/arch/arm/kernel/traps.c */
void dump_backtrace_entry_bk(StrDesc* sd, unsigned long where, unsigned long from, unsigned long frame)
{
#ifdef CONFIG_KALLSYMS
	print_bk(sd, "[<%08lx>] ", where);
	print_symbol_bk(sd, "(%s) ", where);
	print_bk(sd, "from [<%08lx>] ", from);
	print_symbol_bk(sd, "(%s)\n", from);
#else
	print_bk(sd, "Function entered at [<%08lx>] from [<%08lx>]\n", where, from);
#endif
}


void tel_assert_util(char* file, char* function, int line, int param)
{
    char buffer[512];
    char* shfile;
    StrDesc sd={buffer,sizeof(buffer),0};

    /* Remove the path from file name: normally long, not mandatory, and costs missing backtrace info */
    if((shfile=strrchr(file,'/'))!=NULL) file=shfile+1; /* next char after '/' */

    print_bk(&sd, "KERNEL ASSERT at %s #%d (%s)\nin tid [%d]\n",file,line,function,current?current->pid:0);
    BackTrace_k(&sd);
    seh_draw_panic_text(buffer, strlen(buffer),0);
}

static void seh_dont_sleep(int dontSleepTrue)
{
#ifdef CONFIG_PXA95x_DVFM
    static int  seh_PowerHandler = (int)NULL;
    if(seh_PowerHandler == (int)NULL)
    {
	    dvfm_register("EEH", &seh_PowerHandler);
        if(seh_PowerHandler == (int)NULL)
        {
            printk("PXA Power driver - fail to init EEH\n\r");
            return;
        }
    }
#endif


	if(dontSleepTrue == TRUE)
	{
#ifdef CONFIG_PXA95x_DVFM
		dvfm_disable_lowpower(seh_PowerHandler);
#endif
        SEH_PM_WAKE_LOCK();
    }
    else
    {
        SEH_PM_WAKE_UNLOCK();
#ifdef CONFIG_PXA95x_DVFM
		dvfm_enable_lowpower(seh_PowerHandler);
#endif
    }
}

#ifdef CONFIG_PXA_MIPSRAM
static void mipsram_vma_open(struct vm_area_struct *vma)
{
#if defined(MIPSRAM_DEBUG)
	printk(KERN_DEBUG "MIPSRAM OPEN 0x%lx -> 0x%lx (0x%lx)\n",
				vma->vm_start,
				vma->vm_pgoff<<PAGE_SHIFT,
				vma->vm_page_prot);
#endif
}
static void mipsram_vma_close(struct vm_area_struct *vma)
{
#if defined(MIPSRAM_DEBUG)
	printk(KERN_DEBUG "MIPSRAM CLOSE 0x%lx -> 0x%lx\n",
					vma->vm_start,
					vma->vm_pgoff<<PAGE_SHIFT);
#endif
}

/* device memory map method */
/*
 * vma->vm_end, vma->vm_start : specify the user space process address
 * 	 range assigned when mmap has been called;
 * vma->vm_pgoff - is the physical address supplied by user to
 * 				mmap in the last argument (off)
 *		 	However, mmap restricts the offset, so we pass this
 * 		 		shifted 12 bits right.
 */
int mipsram_mmap(struct file *file, struct vm_area_struct *vma)
{
	int ret_val = 0;
	unsigned long size = 0;
	unsigned long pfn;

    /* we do not want to have this area swapped out, lock it */
    vma->vm_flags |= (VM_RESERVED|VM_IO);
	/* see linux/drivers/char/mem.c */
    vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

   /* TBD: without this vm_page_prot=0x53 and write seem to not
	* reach the destination:
    *	- no fault on write
    *	- read immediately (same user process) return the new value
    *	- read after a second by another user process instance
	* 	  return original value
    * Why PROT_WRITE specified by mmap caller does not take effect?
    * MAP_PRIVATE used by app results in copy-on-write behaviour, which is
	* irrelevant for this application
	* vma->vm_page_prot|=L_PTE_WRITE; */

	/* Map MIPSRAM buffer to userspace */
	pfn = MIPSRAM_get_descriptor()->buffer_phys_ptr;
	size = (MIPS_RAM_BUFFER_SZ * sizeof(unsigned int));

	/* re-map page to user-space, we assume the following:
	 * - MIPSRAM buffer is consecutive in physical memory */
	ret_val = remap_pfn_range(vma,
						  (unsigned int)vma->vm_start,
						  (pfn>>PAGE_SHIFT),
						  size,
						  vma->vm_page_prot);
	if (ret_val < 0) {
		printk(KERN_DEBUG "%s failed to re-map!\n", __func__);
		return ret_val;
	}

    vma->vm_ops = &mipsram_vm_ops;
    mipsram_vma_open(vma);
    return(0);
}
#endif /* CONFIG_PXA_MIPSRAM */

#ifdef SEH_RAMDUMP_ENABLED
#include <linux/swap.h> /*nr_free_buffer_pages and similiar*/
#include <linux/vmstat.h> /*global_page_state*/

/*#define RAMFILE_DEBUG*/
static void ramfile_vma_close(struct vm_area_struct *vma);
static struct vm_operations_struct ramfile_vm_ops = {
	.close = ramfile_vma_close
};
#define RAMFILE_LOW_WATERMARK 0x100000
/* NOTE: the size requested by user already accounts for ramfile header */
static int ramfile_mmap(struct file *file, struct vm_area_struct *vma)
{
	int ret_val = 0;
	unsigned long usize = vma->vm_end - vma->vm_start;
	void* pbuf;

	/* Check we don't exhaust all system memory to prevent crash before EEH
		is done with saving logs. Use the total free for now */
	
	unsigned int avail_mem = global_page_state(NR_FREE_PAGES)*PAGE_SIZE;
	printk(KERN_ERR "ramfile_mmap(0x%x), available 0x%x\n", usize, avail_mem);
	if (avail_mem < RAMFILE_LOW_WATERMARK) {
		printk(KERN_ERR "Rejected\n");
		return -ENOMEM;
	}	
	
	/* Note: kmalloc allocates physically continous memory.
	vmalloc would allocate potentially physically discontinuous memory.
	The advantage of vmalloc is that it would be able to allocate more 
	memory when physical memory available is fragmented */
	pbuf = kmalloc(usize, GFP_KERNEL);
#ifdef RAMFILE_DEBUG	
	printk(KERN_ERR "ramfile_mmap(0x%x): ka=%.8x ua=%.8x\n", usize, pbuf, (unsigned int)vma->vm_start);
#endif	
	if (!pbuf)
		return -ENOMEM;

	/* Allocated. Map this to user space and let it fill in the data.
		We do not want to waste a whole page for the ramfile_desc header,
		so we map all the buffer to user space, which should reserved the
		header area.
		We will fill the header and link it into the ramdump when user
		space is done and calls unmap. This way user mistake corrupting
		the header will not compromise the kernel operation.*/
	vma->vm_pgoff = __phys_to_pfn(__virt_to_phys((unsigned)pbuf)); /* needed during unmap/close */
		
	vma->vm_flags |= (VM_RESERVED|VM_IO);
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	ret_val = remap_pfn_range(vma, (unsigned int)vma->vm_start,
				vma->vm_pgoff, usize, vma->vm_page_prot);
	if (ret_val < 0) {
		kfree(pbuf);
		return ret_val;
	}
	vma->vm_ops = &ramfile_vm_ops;
	return(0);
}
static void ramfile_vma_close(struct vm_area_struct *vma)
{
	struct ramfile_desc *prf;
	unsigned long usize = vma->vm_end - vma->vm_start;
					
	/* Fill in the ramfile desc (header) */
	prf = (struct ramfile_desc *)__phys_to_virt(__pfn_to_phys(vma->vm_pgoff));
	prf->payload_size = usize;
	prf->flags = RAMFILE_PHYCONT;
	memset((void*)&prf->reserved[0], 0, sizeof(prf->reserved));
	ramdump_attach_ramfile(prf);
#ifdef RAMFILE_DEBUG	
	printk(KERN_ERR "ramfile close 0x%x - linked into RDC\n", (unsigned)prf);
#endif
}

/* EehSaveErrorInfo: save the error ID/string into ramdump */
static void EehSaveErrorInfo(EehErrorInfo* info)
{
	char str[RAMDUMP_ERR_STR_LEN];
	char *s = 0;
	struct pt_regs regs;
	struct pt_regs *p = 0;
	unsigned err;
	err = (info->err == ERR_EEH_CP) ? RAMDUMP_ERR_EEH_CP : RAMDUMP_ERR_EEH_AP;

	if (info->str && !copy_from_user(str, info->str, sizeof(str))) {
		str[sizeof(str)-1]=0;                                          //-+ GQ00000296 telephony: coverity fixes in osa hwmap nvms eeh fake +-
		if ((s=strchr(str, '\n'))!=0) *s=0;
		s = &str[0];
	}
	memset(&regs, 0, sizeof(struct pt_regs));
	
	if (info->regs && !copy_from_user(&regs, (struct pt_regs *)info->regs, sizeof(regs)))
		p = &regs;
	ramdump_save_dynamic_context(s, (int)err, NULL, p);
	printk(KERN_ERR "SEH saved error info: %.8x (%s)\n", err, s);
}

#else
static void EehSaveErrorInfo(EehErrorInfo* info) {}
#endif

module_init(seh_init);
module_exit(seh_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Marvell");
MODULE_DESCRIPTION("Marvell System Error Handler."); 
EXPORT_SYMBOL(seh_draw_panic_text);
EXPORT_SYMBOL(tel_assert_util);

