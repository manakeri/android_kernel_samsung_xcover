/*
 * arch/arm/mach-pxa/include/mach/pxa95x_pm.c
 *
 * PXA95x Power Management Routines
 *
 * Copyright (C) 2010 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#undef DEBUG
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/vmalloc.h>
#include <linux/kobject.h>
#include <linux/suspend.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <asm/cacheflush.h>
#include <asm/io.h>
#include <asm/mach-types.h>
#include <mach/hardware.h>
#include <mach/pxa95x-regs.h>
#include <mach/regs-rtc.h>
#include <mach/regs-intc.h>
#include <mach/regs-ost.h>
#include <mach/mfp.h>
#include <mach/mfp-pxa3xx.h>
#include <mach/gpio.h>
#include <mach/debug_pm.h>
#ifdef CONFIG_ISPT
#include <mach/pxa_ispt.h>
#endif
#include <mach/pxa95x_pm.h>
#include <mach/soc_vmeta.h>
#include <mach/ipmc.h>
#ifdef CONFIG_ARMV7_OS_SAVE_AND_RESTORE
#include <asm/hardware/armv7_jtag.h>
#endif
#ifdef CONFIG_PXA_MIPSRAM
#include <mach/pxa_mips_ram.h>
#endif
#include <mach/pxa9xx_pm_logger.h> /* for pm debug tracing */
#include <mach/dma.h>
#include <mach/spa.h>

/* mtd.h declares another DEBUG macro definition */
#undef DEBUG
#include <linux/mtd/mtd.h>

/* The first 32KB is reserved and can't be accessed by kernel.
 * This restrict is only valid on BootROM V2.
 */
#define ISRAM_START	0x5c000000

/* for OVERHEATING */
#define FRQ_TEMP	(HZ / 100)

enum {
	OVH_TEMP_40C = 0,
	OVH_TEMP_80C,
	OVH_TEMP_85C,
	OVH_TEMP_90C,
	OVH_TEMP_95C,
	OVH_TEMP_100C,
	OVH_TEMP_105C,
	OVH_TEMP_110C,
};

#define OVH_OTIS_DEFAULT	OVH_TEMP_100C
#define OVH_OVWF_DEFAULT	OVH_TEMP_105C
#define TSS_THRESHOLD		OVH_OTIS_DEFAULT
#define OVH_TO_TEMP_CONVERT(x) (((x - 1) * 5) + 80)
static struct timer_list temp_detecting_timer;
static int temp_of_core;

static int isram_size;
unsigned int is_wkr_mg1_1274_value;
EXPORT_SYMBOL(is_wkr_mg1_1274_value); /*this is used in LPM entry and exit */

/* Counter Structure for Debugging ENTER/EXIT D2/CGM/D0CS */
extern pxa95x_DVFM_LPM_Global_Count DVFMLPMGlobalCount;
extern int d2_led_toggle_flag;

extern int ForceVCTCXO_EN;
extern int EnableD2VoltageChange;
extern unsigned int D2voltageLevelValue;
extern int cur_op;
int RepeatMode;

extern int suspend_forbidden;
#ifdef CONFIG_PXA9XX_ACIPC
extern u32 set_DDR_avail_flag(void);
extern u32 clear_DDR_avail_flag(void);
extern u32 get_acipc_pending_events(void);
extern void acipc_start_cp_constraints(void);
#endif

#define CHECK_APPS_COMM_SYNC						\
{									\
	u32 iir_tmp;							\
	iir_tmp = get_acipc_pending_events();				\
	if (iir_tmp != 0)						\
		printk(KERN_ERR "CHECK_APPS_COMM_SYNC %x\n", iir_tmp);	\
}

enum pxa95x_pm_mode {
	PXA95x_PM_RUN = 0,
	PXA95x_PM_IDLE = 1,
	PXA95x_PM_LCDREFRESH = 2,
	PXA95x_PM_STANDBY = 3,
	PXA95x_PM_D0CS = 5,
	PXA95x_PM_SLEEP = 6,
	PXA95x_PM_DEEPSLEEP = 7,
	PXA95x_PM_CG = 8,
};

extern struct kobject *power_kobj;

pm_wakeup_src_t wakeup_src;	/* interface to set up wakeup_src */
EXPORT_SYMBOL(wakeup_src);

static pm_wakeup_src_t waked;	/* It records the latest wakeup source */

static struct pxa95x_peripheral_wakeup_ops *wakeup_ops;

/* How long we will in sleep mode if duty cycle. */
unsigned int pm_sleeptime = 58;	/* In seconds. */
EXPORT_SYMBOL(pm_sleeptime);
unsigned int pm_msleeptime;	/* In miliseconds. */

/* Flag of reseting CP */
unsigned int pm_cp;

#ifdef CONFIG_ISPT
#define ispt_power_state_d2() ispt_power_msg(CT_P_PWR_STATE_ENTRY_D2)
#define ispt_power_state_d1() ispt_power_msg(CT_P_PWR_STATE_ENTRY_D1)
#define ispt_power_state_cgm() ispt_power_msg(CT_P_PWR_MODE_ENTRY_CGM)
#define ispt_power_state_exit_lpm() ispt_power_msg(CT_P_PWR_STATE_ENTRY_C0)
#else
static int __attribute__ ((unused)) ispt_power_state_d2(void)
{
	return 0;
}

static int __attribute__ ((unused)) ispt_power_state_d1(void)
{
	return 0;
}

static int __attribute__ ((unused)) ispt_power_state_cgm(void)
{
	return 0;
}

static int __attribute__ ((unused)) ispt_power_state_exit_lpm(void)
{
	return 0;
}
#endif

#ifdef CONFIG_IPM
int enable_deepidle;	/* IDLE_D0 -- 0 */
int save_deepidle;

void (*event_notify) (int, int, void *, unsigned int);
EXPORT_SYMBOL(event_notify);

static void (*orig_poweroff) (void);
#endif

/* low level stanby and lcdrefresh routine need to access DMC regs */
unsigned char __iomem *dmc_membase;
EXPORT_SYMBOL(dmc_membase);
unsigned char __iomem *ost_membase;
EXPORT_SYMBOL(ost_membase);
unsigned char __iomem *pm_membase;

extern void pxa95x_cpu_sleep(unsigned int, unsigned int);
extern void pxa95x_cpu_resume(void);
extern void pxa95x_init_standby(unsigned int);
extern void enable_oscc_pout(void);
extern void disable_oscc_pout(void);
extern void enable_oscc_tout_s0(void);
extern void disable_oscc_tout_s0(void);
extern void pxa95x_pm_clear_Dcache_L2Cache(void);
extern void pxa95x_pm_invalidate_Dcache_L2Cache(void);

static struct pxa95x_pm_regs pxa95x_pm_regs;

static unsigned long pm_state;

static u32 disable_sram_use;
int get_pm_state(void)
{
	return pm_state;
}

/*************************************************************************/
/* workaround for bug JIRA MG1-1358 */
int is_wkr_mg1_1358(void)
{
	unsigned int cpuid;
	cpuid = read_cpuid(CPUID_ID);
	/*
	 * This WA is relevant to PXA955 as it is changing the GC bits
	 * and can go to D2 from High PP. PXA950 does not change these bits
	 * because they are reserved but this JIRA is relevant for PXA950 as
	 * well since bit 21 is controlling 13M D2 enable functionality
	 */

	/* It's PXA955 */
	if ((cpuid & 0x0000FFF0) == 0x5810)
		return 1;
	/* It's PXA950 */
	if ((cpuid & 0x0000FFF0) == 0x6970)
		return 1;
	return 0;
}


/* workaround for bug JIRA MG2-388 */
int is_wkr_mg2_388(void)
{
	int wkr = ((cpu_is_pxa968() && !(cpu_is_pxa968_Ax())));

	return wkr;
}

/* workaround for bug JIRA MG1-1677 */
int is_wkr_mg1_1677(void)
{
	return cpu_is_pxa955_E0();
}

static u32 ddadr[32], dtadr[32], dsadr[32], dcmd[32];
void save_dma_registers(void)
{
	int i;
	for (i = 0; i < 32; i++) {
		ddadr[i] = DDADR(i);
		dtadr[i] = DTADR(i);
		dsadr[i] = DSADR(i);
		dcmd[i] = DCMD(i);
	}
}

void restore_dma_registers(void)
{
	int i;
	for (i = 0; i < 32; i++) {
		if (DCSR(i) & DCSR_NODESC) {
			DTADR(i) = dtadr[i];
			DSADR(i) = dsadr[i];
			DCMD(i) = dcmd[i];
		} else {
			DDADR(i) = ddadr[i];
		}
	}
}

/*************************************************************************/
/* workaround for bug JIRA MG1-1274 */
int is_wkr_mg1_1274(void)
{
	return 0;
}

static unsigned int lpm_with_axi_on;
void enable_axi_lpm_entry(void)
{
	if ((CKENC & ((1 << (CKEN_AXI - 64)) |
					(1 << (CKEN_AXI_2X - 64)))) == 0x0) {
		/* notify WA is used */
		lpm_with_axi_on = 1;
		/* turning on the AXI clocks */
		CKENC |= ((1 << (CKEN_AXI - 64)) | (1 << (CKEN_AXI_2X - 64)));
	}
}

void enable_axi_lpm_exit(void)
{
	if (lpm_with_axi_on) {
		lpm_with_axi_on = 0;
		/* turning off the AXI clocks */
		CKENC &= ~((1 << (CKEN_AXI - 64)) | (1 << (CKEN_AXI_2X - 64)));
	}
}


int is_wkr_mg1_1468(void)
{
	return cpu_is_pxa95x();
}

static void pxa95x_intc_save(struct intc_regs *context)
{
	unsigned int temp, i;

	context->iccr = ICCR;
	for (i = 0; i < 32; i++) {
		context->ipr[i] = __raw_readl(&IPR(i));
	}
	for (i = 0; i < 21; i++) {
		context->ipr2[i] = __raw_readl(&IPR(i));
	}

	/* load registers by accessing co-processor */
	__asm__("mrc\tp6, 0, %0, c1, c0, 0" : "=r"(temp));
	context->icmr = temp;
	__asm__("mrc\tp6, 0, %0, c7, c0, 0" : "=r"(temp));
	context->icmr2 = temp;
	__asm__("mrc\tp6, 0, %0, c2, c0, 0" : "=r"(temp));
	context->iclr = temp;
	__asm__("mrc\tp6, 0, %0, c8, c0, 0" : "=r"(temp));
	context->iclr2 = temp;
}

static void pxa95x_intc_restore(struct intc_regs *context)
{
	unsigned int temp, i;

	ICCR = context->iccr;
	for (i = 0; i < 32; i++) {
		__raw_writel(context->ipr[i], &IPR(i));
	}
	for (i = 0; i < 21; i++) {
		__raw_writel(context->ipr2[i], &IPR(i));
	}

	temp = context->icmr;
	__asm__("mcr\tp6, 0, %0, c1, c0, 0" : : "r"(temp));
	temp = context->icmr2;
	__asm__("mcr\tp6, 0, %0, c7, c0, 0" : : "r"(temp));
	temp = context->iclr;
	__asm__("mcr\tp6, 0, %0, c2, c0, 0" : : "r"(temp));
	temp = context->iclr2;
	__asm__("mcr\tp6, 0, %0, c8, c0, 0" : : "r"(temp));
}

static void pxa95x_clk_save(struct clock_regs *context)
{
	context->aicsr = AICSR;
	context->ckena = CKENA;
	context->ckenb = CKENB;
	context->oscc = OSCC;
	/* Disable the processor to use the ring oscillator output clock
	 * as a clock source when transitioning from any low-power mode
	 * to D0 mode.
	 */
	ACCR &= ~ACCR_PCCE;
}

static void pxa95x_clk_restore(struct clock_regs *context)
{
	context->aicsr &= (AICSR_PCIE | AICSR_TCIE | AICSR_FCIE);
	AICSR = context->aicsr;
	CKENA = context->ckena;
	CKENB = context->ckenb;
	OSCC = context->oscc;
}

static void pxa95x_ost_save(struct ost_regs *context)
{
	context->oscr4 = OSCR4;
	context->omcr4 = OMCR4;
	context->osmr4 = OSMR4;
	context->oier = OIER;
}

static void pxa95x_ost_restore(struct ost_regs *context)
{
	OSCR4 = context->oscr4;
	OMCR4 = context->omcr4;
	OSMR4 = context->osmr4;
	OIER = context->oier;
}

static void pxa95x_mfp_save(struct mfp_regs *context)
{
	int i;

	for (i = 0; i < MAX_MFP_PINS; i++)
		context->mfp[i] = __raw_readl(&__REG(MFPR_BASE) + (i << 2));
}

static void pxa95x_mfp_restore(struct mfp_regs *context)
{
	int i;

	for (i = 0; i < MAX_MFP_PINS; i++)
		__raw_writel(context->mfp[i], &__REG(MFPR_BASE) + (i << 2));
}

/* The setting of GPIO will be restored.
 * The status of GPIO Edge Status will be lost.
 */
static void pxa95x_gpio_save(struct gpio_regs *context)
{
	context->gpdr0 = GPDR0;
	context->gpdr1 = GPDR1;
	context->gpdr2 = GPDR2;
	context->gpdr3 = GPDR3;

	context->gplr0 = GPLR0;
	context->gplr1 = GPLR1;
	context->gplr2 = GPLR2;
	context->gplr3 = GPLR3;

	context->grer0 = GRER0;
	context->grer1 = GRER1;
	context->grer2 = GRER2;
	context->grer3 = GRER3;

	context->gfer0 = GFER0;
	context->gfer1 = GFER1;
	context->gfer2 = GFER2;
	context->gfer3 = GFER3;
}

static void pxa95x_gpio_restore(struct gpio_regs *context)
{
	GPDR0 = context->gpdr0;
	GPDR1 = context->gpdr1;
	GPDR2 = context->gpdr2;
	GPDR3 = context->gpdr3;

	GPSR0 = context->gplr0;
	GPSR1 = context->gplr1;
	GPSR2 = context->gplr2;
	GPSR3 = context->gplr3;
	GPCR0 = ~(context->gplr0);
	GPCR1 = ~(context->gplr1);
	GPCR2 = ~(context->gplr2);
	GPCR3 = ~(context->gplr3);

	GRER0 = context->grer0;
	GRER1 = context->grer1;
	GRER2 = context->grer2;
	GRER3 = context->grer3;

	GFER0 = context->gfer0;
	GFER1 = context->gfer1;
	GFER2 = context->gfer2;
	GFER3 = context->gfer3;
}

static void pxa95x_sysbus_init(struct pxa95x_pm_regs *context)
{
	context->smc.membase = ioremap(SMC_START, SMC_END - SMC_START + 1);
	context->arb.membase = ioremap(ARB_START, ARB_END - ARB_START + 1);

	dmc_membase = ioremap(DMC_START, DMC_END - DMC_START + 1);
	ost_membase = ioremap(OST_START, OST_END - OST_START + 1);
	pm_membase = ioremap(PM_START, PM_END - PM_START + 1);

	isram_size = (128 * 1024);

	context->sram_map = ioremap(ISRAM_START, isram_size);
	context->sram = vmalloc(isram_size);
	/* Two words begun from 0xC0000000 are used to store key information.
	 */
	context->data_pool = (unsigned char *) 0xC0000000;
}

static void pxa95x_sysbus_save(struct pxa95x_pm_regs *context)
{
	unsigned char __iomem *base = NULL;
	unsigned int tmp;

	/* static memory controller */
	base = context->smc.membase;
	context->smc.msc0 = readl(base + MSC0_OFF);
	context->smc.msc1 = readl(base + MSC1_OFF);
	context->smc.sxcnfg = readl(base + SXCNFG_OFF);
	context->smc.memclkcfg = readl(base + MEMCLKCFG_OFF);
	context->smc.cscfg0 = readl(base + CSADRCFG0_OFF);
	context->smc.cscfg1 = readl(base + CSADRCFG1_OFF);
	context->smc.cscfg2 = readl(base + CSADRCFG2_OFF);
	context->smc.cscfg3 = readl(base + CSADRCFG3_OFF);
	context->smc.csmscfg = readl(base + CSMSADRCFG_OFF);

	/* system bus arbiters */
	base = context->arb.membase;
	context->arb.ctl1 = readl(base + ARBCTL1_OFF);
	context->arb.ctl2 = readl(base + ARBCTL2_OFF);

	/* pmu controller */
	if (!cpu_is_pxa970())
		context->pmu.pecr = PECR;

	context->pmu.pvcr = PVCR;
	/* clear PSR */
	tmp = PSR;
	tmp &= 0x07;
	PSR = tmp;

	pxa95x_intc_save(&(context->intc));
	pxa95x_clk_save(&(context->clock));
	pxa95x_ost_save(&(context->ost));
	pxa95x_mfp_save(&(context->mfp));
	pxa95x_gpio_save(&(context->gpio));
}

static void pxa95x_sysbus_restore(struct pxa95x_pm_regs *context)
{
	unsigned char __iomem *base = NULL;

	pxa95x_mfp_restore(&(context->mfp));
	pxa95x_gpio_restore(&(context->gpio));
	pxa95x_ost_restore(&(context->ost));
	pxa95x_intc_restore(&(context->intc));
	pxa95x_clk_restore(&(context->clock));

	/* PMU controller */
	if (!cpu_is_pxa970()) {
		/* status information will be lost in PECR */
		PECR = 0xA0000000;
		PECR = (context->pmu.pecr | PECR_E1IS | PECR_E0IS);
	}
	PVCR = context->pmu.pvcr;

	/* system bus arbiters */
	base = context->arb.membase;
	writel(context->arb.ctl1, base + ARBCTL1_OFF);
	writel(context->arb.ctl2, base + ARBCTL2_OFF);

	/* static memory controller */
	base = context->smc.membase;
	writel(context->smc.msc0, base + MSC0_OFF);
	writel(context->smc.msc1, base + MSC1_OFF);
	writel(context->smc.sxcnfg, base + SXCNFG_OFF);
	writel(context->smc.memclkcfg, base + MEMCLKCFG_OFF);
	writel(context->smc.cscfg0, base + CSADRCFG0_OFF);
	writel(context->smc.cscfg1, base + CSADRCFG1_OFF);
	writel(context->smc.cscfg2, base + CSADRCFG2_OFF);
	writel(context->smc.cscfg3, base + CSADRCFG3_OFF);
	writel(context->smc.csmscfg, base + CSMSADRCFG_OFF);
}

static void pxa95x_pm_set_clk(char *id, int enable)
{
	struct clk *clk;

	clk = clk_get(NULL, id);
	if (IS_ERR(clk)) {
		printk(KERN_INFO "clk_get failed when getting \"%s\"\n", id);
		return;
	}

	if (enable)
		clk_enable(clk);
	else
		clk_disable(clk);
}

/* This function is used to set unit clock before system enters sleep.
 */
static void pxa95x_pm_set_cken(void)
{
	/*
	 * turn off SMC, GPIO,INTC clocks to save power in sleep mode.
	 * they will be turn on by BLOB during wakeup
	 */
	pxa95x_pm_set_clk("SMCCLK", 0);
	pxa95x_pm_set_clk("GPIOCLK", 0);
	/*
	 * turn on clocks used by bootrom during wakeup
	 * they will be turn off by BLOB during wakeup
	 * D0CKEN_A clocks: bootrom, No.19
	 */
	pxa95x_pm_set_clk("BOOTCLK", 1);
	/* Trusted parts */
	pxa95x_pm_set_clk("SMCCLK", 1);
}

/* This function is used to restore unit clock after system resumes.
 */
static void pxa95x_pm_restore_cken(void)
{
	pxa95x_pm_set_clk("SMCCLK", 1);
	pxa95x_pm_set_clk("GPIOCLK", 1);
	pxa95x_pm_set_clk("BOOTCLK", 0);
}

/* This function is used to clear power manager status.
 */
static void pxa95x_clear_pm_status(int sys_level)
{
	unsigned int tmp;

	if (sys_level && !cpu_is_pxa970()) {
		/* clear power manager status */
		tmp = PSR;
		tmp &= PSR_MASK;
		PSR = tmp;
	}
	/* clear application system status */
	tmp = ASCR;
	tmp &= ASCR_MASK;
	ASCR = tmp;
	/* clear all application subsystem reset status */
	tmp = ARSR;
	ARSR = tmp;
}

/* This function is used to set RTC time.
 * When it timeouts, it will wakeup system from low power mode.
 * There's limitation that only 65 seconds sleep time can be set by this way.
 * And user should avoid to use PIAR because it will be used as wakeup timer.
 *
 * Notice:
 * User can also choice use another RTC register to trigger wakeup event.
 * If so, keep pm_sleeptime as 0. Otherwise, those RTC registers event
 * will make user confused. System will only serve the first RTC event.
 */
static void pxa95x_set_wakeup_sec(int sleeptime)
{
	unsigned int tmp;
	if (sleeptime) {
		/* PIAR can not more than 65535 */
		if (sleeptime > 65)
			sleeptime = 65;
		pr_debug("Set RTC to wakeup system after %d sec\n", sleeptime);
		tmp = RTSR;
		tmp &= ~(RTSR_PICE | RTSR_PIALE);
		RTSR = tmp;
		/* set PIAR to sleep time, in ms */
		PIAR = sleeptime * 1000;

		tmp = RTSR;
		tmp |= RTSR_PICE;
		RTSR = tmp;
	}
}

/* This function is used to set OS Timer4 time.
 * The time interval may not be accurate. Because it's derived from 32.768kHz
 * oscillator.
 */
static void pxa95x_set_wakeup_msec(int msleeptime)
{
	unsigned int tmp;
	if (msleeptime) {
		pr_debug("Set OS Timer4 to wakeup system after %d msec\n",
			 msleeptime);
		/* set the time interval of sleep */
		tmp = OSCR4;
		OSSR = 0x10;
		OSMR4 = tmp + msleeptime * 32768 / 1000;
	}
}

/*
 * Clear the wakeup source event.
 */
static void pm_clear_wakeup_src(pm_wakeup_src_t src)
{
	if (wakeup_ops->ext)
		wakeup_ops->ext(src, 0);
	if (wakeup_ops->key)
		wakeup_ops->key(src, 0);
	if (wakeup_ops->mmc1)
		wakeup_ops->mmc1(src, 0);
	if (wakeup_ops->mmc3)
		wakeup_ops->mmc3(src, 0);
	if (wakeup_ops->uart)
		wakeup_ops->uart(src, 0);
	if (wakeup_ops->eth)
		wakeup_ops->eth(src, 0);
	if (wakeup_ops->tsi)
		wakeup_ops->tsi(src, 0);
	if(wakeup_ops->usbdetect)
		wakeup_ops->usbdetect(src,0);
	if(wakeup_ops->headset)
		wakeup_ops->headset(src,0);
	if(wakeup_ops->hookswitch)
		wakeup_ops->hookswitch(src,0);
	if(wakeup_ops->proximate_sensor)
		wakeup_ops->proximate_sensor(src,0);
}

static void pm_select_wakeup_src(enum pxa95x_pm_mode lp_mode,
				 pm_wakeup_src_t src)
{
	unsigned int tmp, reg_src = 0;

	if (!wakeup_ops)
		printk(KERN_ERR "ERROR: wakeup_ops is still NULL!\n");
	if (wakeup_ops->ext)
		reg_src |= wakeup_ops->ext(src, 1);
	if (wakeup_ops->key)
		reg_src |= wakeup_ops->key(src, 1);
	if (wakeup_ops->mmc1)
		reg_src |= wakeup_ops->mmc1(src, 1);
	if (wakeup_ops->mmc3)
		reg_src |= wakeup_ops->mmc3(src, 1);
	if (wakeup_ops->uart)
		reg_src |= wakeup_ops->uart(src, 1);
	if (wakeup_ops->eth)
		reg_src |= wakeup_ops->eth(src, 1);
	if (wakeup_ops->headset)
		reg_src |= wakeup_ops->headset(src, 1);
	if (wakeup_ops->hookswitch)
		reg_src |= wakeup_ops->hookswitch(src, 1);
if (lp_mode == PXA95x_PM_CG) {
	if (wakeup_ops->proximate_sensor)
		reg_src |= wakeup_ops->proximate_sensor(src, 1);
}
	if (wakeup_ops->tsi)
		reg_src |= wakeup_ops->tsi(src, 1);
	if(wakeup_ops->usbdetect) // added by JIIN_101201 for fix the wakeup 2ms issue
		reg_src |= wakeup_ops->usbdetect(src,1);
	if (src.bits.rtc) {
		pxa95x_set_wakeup_sec(pm_sleeptime);
		reg_src |= PXA95x_PM_WE_RTC;
	}
	if (src.bits.ost) {
		pxa95x_set_wakeup_msec(pm_msleeptime);
		reg_src |= PXA95x_PM_WE_OST;
	}
	if (src.bits.msl)
		reg_src |= PXA95x_PM_WE_MSL0;

	/* set wakeup register */
	if (lp_mode == PXA95x_PM_SLEEP) {
		if (!cpu_is_pxa970()) {
			PWSR = 0xFFFFFFFF;
			PWER = 0;

			tmp = PWER;
			if (src.bits.rtc)
				tmp |= PWER_WERTC;
			if (src.bits.ext0)
				tmp |= (PWER_WER0 | PWER_WEF0);
			if (src.bits.ext1)
				tmp |= (PWER_WER1 | PWER_WEF1);
			PWER = tmp;
		}

		AD3SR = 0xFFFFFFFF;
		AD3ER = 0;
		AD3ER = reg_src;
	}
	if (lp_mode == PXA95x_PM_DEEPSLEEP) {
		if (!cpu_is_pxa970()) {
			PWSR = 0xFFFFFFFF;
			PWER = 0;

			tmp = PWER;
			/* RTC cause exit from S3 mode too shortly - TO FIX */
			/*
			   if (src.bits.rtc)
			   tmp |= PWER_WERTC;
			   */
			if (src.bits.ext0)
				tmp |= (PWER_WER0 | PWER_WEF0);
			/* on MG1 there is only one external wake up */
			/*
			   if (src.bits.ext1)
			   tmp |= (PWER_WER1 | PWER_WEF1);
			   */
			PWER = tmp;
		}
	}
	if (lp_mode == PXA95x_PM_STANDBY) {
		AD2D0SR = 0xFFFFFFFF;
		AD2D0ER = 0;
		AD2D0ER = reg_src;
	}
	if (lp_mode == PXA95x_PM_LCDREFRESH) {
		AD1D0SR = 0xFFFFFFFF;
		AD1D0ER = 0;
		/* add the minilcd wakeup event */
		AD1D0ER = reg_src | PXA95x_PM_WE_MLCD;
	}
	if (lp_mode == PXA95x_PM_CG) {
		ACGD0SR = 0xFFFFFFFF;
		/* add the interrupt and dmemc wakeup event */
		ACGD0ER = reg_src | PXA95x_PM_WE_INTC | PXA95x_PM_WE_DMC;
	}
}

static unsigned int pm_query_wakeup_src(void)
{
	unsigned int data;

	memset(&waked, 0, sizeof(pm_wakeup_src_t));

	if (ASCR & 0x07) {
		data = ASCR & 0x07;
		ASCR = data;
		switch (data) {
		case 4:
			/* check D1 wakeup source */
			data = AD1D0SR;
			AD1D0SR = data;
			if (wakeup_ops->query)
				wakeup_ops->query(data, &waked);
			break;
		case 2:
			/* check D2 wakeup source */
			data = AD2D0SR;
			AD2D0SR = data;
			if (wakeup_ops->query)
				wakeup_ops->query(data, &waked);
			break;
		case 1:
			/* check D3 wakeup source */
			data = AD3SR;
			AD3SR = data;
			if (wakeup_ops->query)
				wakeup_ops->query(data, &waked);
			if (!cpu_is_pxa970()) {
				data = PWSR;
				PWSR = data;
				if (data & PWSR_EERTC)
					waked.bits.rtc = 1;
				if (data & PWSR_EDR0)
					waked.bits.ext0 = 1;
				if (data & PWSR_EDR1)
					waked.bits.ext1 = 1;
			}
			break;
		}
	} else if (ARSR & 0x04) {
		/* check S3 wakeup source */
		data = ARSR & 0x04;
		ARSR = data;
		if (!cpu_is_pxa970()) {
			data = PWSR;
			PWSR = data;
			if (data & PWSR_EERTC)
				waked.bits.rtc = 1;
			if (data & PWSR_EDR0)
				waked.bits.ext0 = 1;
			if (data & PWSR_EDR1)
				waked.bits.ext1 = 1;
		}
	} else {
		/* check clock gate mode wakeup source */
		data = ACGD0SR;
		ACGD0SR = data;
		if (wakeup_ops->query)
			wakeup_ops->query(data, &waked);
	}

	return data;
}

static void dump_wakeup_src(pm_wakeup_src_t *src)
{
	printk(KERN_DEBUG "wakeup source: ");
	if (src->bits.rtc)
		printk(KERN_DEBUG "rtc, ");
	if (src->bits.ost)
		printk(KERN_DEBUG "ost, ");
	if (src->bits.msl)
		printk(KERN_DEBUG "msl, ");
	if (src->bits.wifi)
		printk(KERN_DEBUG "wifi, ");
	if (src->bits.uart1)
		printk(KERN_DEBUG "uart1, ");
	if (src->bits.uart2)
		printk(KERN_DEBUG "uart2, ");
	if (src->bits.uart3)
		printk(KERN_DEBUG "uart3, ");
	if (src->bits.mkey)
		printk(KERN_DEBUG "mkey, ");
	if (src->bits.dkey)
		printk(KERN_DEBUG "dkey, ");
	if (src->bits.mlcd)
		printk(KERN_DEBUG "mlcd, ");
	if (src->bits.tsi)
		printk(KERN_DEBUG "tsi, ");
	if (src->bits.ext0)
		printk(KERN_DEBUG "ext0, ");
	if (src->bits.ext1)
		printk(KERN_DEBUG "ext1, ");
	if (src->bits.mmc1_cd)
		printk(KERN_DEBUG "mmc1 card detect, ");
	if (src->bits.mmc2_cd)
		printk(KERN_DEBUG "mmc2 card detect, ");
	if (src->bits.mmc3_cd)
		printk(KERN_DEBUG "mmc3 card detect, ");
	if (src->bits.mmc1_dat1)
		printk(KERN_DEBUG "mmc1 dat1, ");
	if (src->bits.mmc2_dat1)
		printk(KERN_DEBUG "mmc2 dat1, ");
	if (src->bits.mmc3_dat1)
		printk(KERN_DEBUG "mmc3 dat1, ");
	if (src->bits.eth)
		printk(KERN_DEBUG "eth, ");
	if (src->bits.cmwdt)
		printk(KERN_DEBUG "comm watchdog, ");
	if(src->bits.usbdetect)  // added by JIIN_101201 for fix the wakeup 2ms issue
		printk(KERN_DEBUG "usb card detect, ");
}

void get_wakeup_source(pm_wakeup_src_t *src)
{
	memset(src, 0, sizeof(pm_wakeup_src_t));
	memcpy(src, &waked, sizeof(pm_wakeup_src_t));
}
EXPORT_SYMBOL(get_wakeup_source);

int pxa95x_wakeup_register(struct pxa95x_peripheral_wakeup_ops *ops)
{
	wakeup_ops = ops;

	/* set default wakeup source */
	if (wakeup_ops->init)
		wakeup_ops->init(&wakeup_src);

	/* clear the related wakeup source */
	pm_clear_wakeup_src(wakeup_src);

	return 0;
}
EXPORT_SYMBOL(pxa95x_wakeup_register);

void pxa95x_wakeup_unregister(void)
{
	wakeup_ops = NULL;
}
EXPORT_SYMBOL(pxa95x_wakeup_unregister);

/*************************************************************************/
static void flush_cpu_cache(void)
{
	__cpuc_flush_kern_all();
	/* __cpuc_flush_l2cache_all(); */
}

struct os_header {
	int version;
	int identifier;
	int address;
	int size;
	int reserved;
};

int pxa95x_pm_enter_sleep(struct pxa95x_pm_regs *pm_regs)
{
	pxa95x_sysbus_save(pm_regs);

	pm_select_wakeup_src(PXA95x_PM_SLEEP, wakeup_src);

	pxa95x_pm_set_cken();

	/* should set:modeSaveFlags, areaAddress, flushFunc, psprAddress,
	 * extendedChecksumByteCount */
	pm_regs->pm_data.modeSaveFlags = 0x3f;	/* PM_MODE_SAVE_FLAG_SVC; */
	pm_regs->pm_data.flushFunc = flush_cpu_cache;
	pm_regs->pm_data.areaAddress = (unsigned int) &(pm_regs->pm_data);
	pm_regs->pm_data.psprAddress = (unsigned int) &PSPR;
	pm_regs->pm_data.extendedChecksumByteCount =
	    sizeof(struct pxa95x_pm_regs) - sizeof(struct pm_save_data);
	pr_debug("ext size:%d, save size%d\n",
		 pm_regs->pm_data.extendedChecksumByteCount,
		 sizeof(struct pm_save_data));

	/* save the resume back address into SDRAM */
	pm_regs->word0 = __raw_readl(pm_regs->data_pool);
	pm_regs->word1 = __raw_readl(pm_regs->data_pool + 4);
	__raw_writel(virt_to_phys(pxa95x_cpu_resume), pm_regs->data_pool);
	__raw_writel(virt_to_phys(&(pm_regs->pm_data)),
		     pm_regs->data_pool + 4);

	pxa95x_clear_pm_status(1);

	pr_debug("ready to sleep:0x%lx\n",
		 virt_to_phys(&(pm_regs->pm_data)));

#if defined(CONFIG_PXA9XX_ACIPC)
	if (0 == clear_DDR_avail_flag()) {
		CHECK_APPS_COMM_SYNC
#endif
		    /* go to Zzzz */
		    pxa95x_cpu_sleep((unsigned int) &(pm_regs->pm_data),
				     virt_to_phys(&(pm_regs->pm_data)));
#if defined(CONFIG_PXA9XX_ACIPC)
		/* set the DDR_AVAIL flag for comm */
		set_DDR_avail_flag();

	} else
		printk(KERN_ERR "******EDDR WARNING: %s DDR_req=1 shared "
				"flag=1.should not happen. check the apps-comm "
				"sync\n", __func__);
#endif

	/* come back */
	__raw_writel(pm_regs->word0, pm_regs->data_pool);
	__raw_writel(pm_regs->word1, pm_regs->data_pool + 4);

	pxa95x_pm_restore_cken();

	pm_query_wakeup_src();
	dump_wakeup_src(&waked);
	pxa95x_sysbus_restore(pm_regs);

	pxa95x_clear_pm_status(1);

	/* clear RDH */
	ASCR &= ~ASCR_RDH;

	pm_clear_wakeup_src(wakeup_src);

#ifdef CONFIG_IPM
	/* Need to post event to policy maker.  */
	if (event_notify) {
		event_notify(IPM_EVENT_SUSPEND_WAKEUP, PM_SUSPEND_MEM,
			     &waked, sizeof(pm_wakeup_src_t));
	}
#endif

	pr_debug("*** made it back from sleep\n");

	return 0;
}

#ifdef CONFIG_IPM
#ifdef CONFIG_PMIC_D1980
extern void d1980_power_off(void);
#endif 
static void pxa95x_pm_poweroff(void)
{
	unsigned int mode = PXA95x_PM_DEEPSLEEP;
	unsigned long cser;

	spa_Lfs_Access(DEBUG_WRITE, POWEROFF_PXA95X_PM);

	printk(KERN_INFO "Enter pxa95x_pm_poweroff\n");

	if (!cpu_is_pxa970())
		PMCR |= 0x01;		/* Workaround of Micco reset */

	/* Disable sleeptime interface at here */
	pm_sleeptime = 0;
	pm_select_wakeup_src(mode, wakeup_src);
	/* No need to set CKEN bits in PowerOff path */
	/* pxa95x_pm_set_cken(); */
	pxa95x_clear_pm_status(1);
	__raw_writel(0x0, pm_membase + CSER_OFF);
	while ((cser = __raw_readl(pm_membase + CSER_OFF)) != 0)
		;
#ifdef CONFIG_PMIC_D1980
    d1980_power_off();
#endif
	PWRMODE = mode | PXA95x_PM_I_Q_BIT;
	while (PWRMODE != (mode | PXA95x_PM_I_Q_BIT))
		;
	__asm__("dsb");
	__asm__("wfi");
}
#endif

static void pm_preset_standby(void)
{
	pxa95x_clear_pm_status(0);
}

static unsigned int pm_postset_standby(void)
{
	unsigned int wakeup_data;

	wakeup_data = pm_query_wakeup_src();
	/*dump_wakeup_src(&waked); */

	/* clear RDH */
	ASCR &= ~ASCR_RDH;

	pxa95x_clear_pm_status(0);
	pm_clear_wakeup_src(wakeup_src);

	return wakeup_data;
}

int pxa95x_pm_enter_standby(struct pxa95x_pm_regs *pm_regs)
{
	unsigned long ticks = 1;
	unsigned int power_state = PXA95x_PM_S0D2C2;

	pm_select_wakeup_src(PXA95x_PM_STANDBY, wakeup_src);
	pm_preset_standby();
#if defined(CONFIG_PXA9XX_ACIPC)
	if (0 == clear_DDR_avail_flag()) {
		CHECK_APPS_COMM_SYNC
#endif
		pxa95x_cpu_standby((unsigned int) pm_regs->sram_map + 0x8000,
				(unsigned int) pm_regs->sram_map + 0xa000 - 4,
				(unsigned int) &ticks, power_state);
#if defined(CONFIG_PXA9XX_ACIPC)
		set_DDR_avail_flag();
	} else
		printk(KERN_ERR "******EDDR WARNING: %s DDR_req=1 shared "
			       "flag=1. should not happen. check the apps-comm"
			       " sync\n", __func__);
#endif
	pm_postset_standby();

#ifdef CONFIG_IPM
	/* Need to post event to policy maker.  */
	if (event_notify) {
		event_notify(IPM_EVENT_STANDBY_WAKEUP, PM_SUSPEND_STANDBY,
			     &waked, sizeof(pm_wakeup_src_t));
	}
#endif
	pr_debug("*** made it back from standby\n");

	return 0;
}

unsigned int pm_core_pwdn(unsigned int powerState)
{
	unsigned int cpupwr = 0;

	if (cpu_is_pxa955() || cpu_is_pxa968()) {
		/*This function is called before & after LPM
		and before and after D0CS
		Normal functionallity is that outside of these functions
		the register will be set to 0x0001 -
		this will allow C2 with 0x0 counter (see JIRA 1495).
		when entring D2 this will be set to C2 with 0x3F counter.
		When entring D0CS it will configure C1*/
		switch (powerState) {
		case CPU_PDWN_LPM_ENTRY:
			cpupwr = CPU_PDWN_SETALLWAYS | CPU_PDWN_ENABLE;
			break;
		case CPU_PDWN_D0CS_ENTRY:
			cpupwr = CPU_PDWN_SETALLWAYS | CPU_PDWN_DISABLE;
			break;
		case CPU_PDWN_LPM_EXIT:
		case CPU_PDWN_D0CS_EXIT:
			cpupwr = CPU_PDWN_SETALLWAYS | CPU_PDWN_ENABLE;
			break;
		}
		/*configuring direction*/
		__raw_writel(cpupwr, (void *)&(CPUPWR));
	}
	return cpupwr;
}

unsigned int user_index;
void vmeta_pwr(unsigned int enableDisable)
{
	unsigned int vmpwr = 0;
	static unsigned int onetime;
	vmpwr = VMPWR;

	if (onetime == 0) {
		onetime = 1;
		dvfm_enable_op_name("208M_HF", user_index);
		dvfm_enable_op_name("416M_VGA", user_index);
	}
	if (VMETA_PWR_ENABLE == enableDisable) {
		if (vmpwr & VMPWR_PWR_ST)
			return;	/*Pwr is already on */
		VMPWR = VMPWR_SETALLWAYS | VMPWR_PWON;
		do {
			vmpwr = VMPWR;
		} while ((vmpwr & VMPWR_PWR_ST) != VMPWR_PWR_ST);
	} else if (VMETA_PWR_DISABLE == enableDisable) {
		if ((vmpwr & VMPWR_PWR_ST) != VMPWR_PWR_ST)
			return;	/*Pwr is already off */
		VMPWR = VMPWR_SETALLWAYS;
	}
}

void gc_pwr(int enableDisable)
{
	unsigned int gcpwr = 0;

	gcpwr = GCPWR;
	if (GC_PWR_ENABLE == enableDisable) {
		if (gcpwr & GCPWR_PWR_ST)
			return;	/*Pwr is already on */
		GCPWR = GCPWR_SETALLWAYS | GCPWR_PWON;
		do {
			gcpwr = GCPWR;
		} while ((gcpwr & GCPWR_PWR_ST) != GCPWR_PWR_ST);
		gcpwr = GCPWR;
		gcpwr |= GCPWR_RST_N;
		GCPWR = gcpwr;
	} else if (GC_PWR_DISABLE == enableDisable) {
		if ((gcpwr & GCPWR_PWR_ST) != GCPWR_PWR_ST)
			return;	/*Pwr is already off */
		/* GCPWR_RST_N,GCPWR_PWON = 0 */
		GCPWR = GCPWR_SETALLWAYS;
		do {
			gcpwr = GCPWR;
		} while ((gcpwr & GCPWR_PWR_ST) == GCPWR_PWR_ST);
	}
}
EXPORT_SYMBOL(gc_pwr);

#ifdef CONFIG_PXA95x_DVFM
/* masks of the reserved bits */
unsigned int ckena_rsvd_bit_mask;
unsigned int ckenb_rsvd_bit_mask;
unsigned int ckenc_rsvd_bit_mask;
static void cken_rsvd_bit_mask_setup(void)
{
	ckena_rsvd_bit_mask = 0x00080001;
	ckenb_rsvd_bit_mask = 0x97FCF040;
	ckenc_rsvd_bit_mask = 0x00c00000;
}

extern int calc_switchtime(unsigned int, unsigned int);

static unsigned int pm_postset_clockgate(void)
{
	unsigned int wakeup_data;

	wakeup_data = pm_query_wakeup_src();
	/*dump_wakeup_src(&waked); */

	pxa95x_clear_pm_status(0);
	pm_clear_wakeup_src(wakeup_src);

	return wakeup_data;
}

extern unsigned long uboot_DefaultAvcrValue;

void enter_lowpower_mode(int state)
{
	unsigned int start_tick = 0, end_tick = 0;
	unsigned int cken[3] = {0, 0, 0 }, icmr[3] = {0, 0, 0};
	unsigned int reg, sram, pollreg,aicsr;
	unsigned int accr_gcfs = 0;
	unsigned int accr, acsr;
	unsigned int power_state;
	unsigned int wakeup_data, cpupwr;
	static unsigned int last_d2exit_time;
	static unsigned int first_d2_entry = 1;
	if (is_wkr_mg1_1274()) {
		/*
		 * Mask all interrupts except R2B20 BCCU
		 */
		icmr[0] = ICMR;
		icmr[1] = ICMR2;
		icmr[2] = ICMR3;
		ICMR = 0x00000000;
		ICMR2 = 0x00100000;
		ICMR3 = 0x00100000;
		/* go to LPM only if there are no pending interrupts */
		if (((ICPR & icmr[0]) != 0x0) ||
		    ((ICPR2 & icmr[1]) != 0x0) ||
		    ((ICPR3 & icmr[2]) != 0x0)) {
			ICMR = icmr[0];
			ICMR2 = icmr[1];
			ICMR3 = icmr[2];
			return;
		}
	}
	/*C2 is entered in power-down states */
	cpupwr = pm_core_pwdn(CPU_PDWN_LPM_ENTRY);
	/* the counter must be disabeld and it value saved before enterinf
	 * lpm */
	mipsram_disable_counter();

#ifdef CONFIG_ARMV7_OS_SAVE_AND_RESTORE
	/* Save JTAG registers before going to low power mode */
	armv7_jtag_registers_save();
#endif

	if (state == POWER_MODE_D1) {
		power_state = PXA95x_PM_S0D1C2;
		/* D1 is represented by LCDREFRESH */
		pm_select_wakeup_src(PXA95x_PM_LCDREFRESH, wakeup_src);

		/* need the same action as for D2 */
		if (PXA9xx_Force_D1 == ForceLPM) {
			LastForceLPM = PXA9xx_Force_D1;
			AD1D0ER = ForceLPMWakeup;
		}
		pm_preset_standby();
		end_tick = OSCR4;

		if (is_wkr_mg1_1468())
			enable_axi_lpm_entry();

		if (is_wkr_mg1_1677())
			save_dma_registers();

		/* PWRMODE = D1 */
		PWRMODE = (PXA95x_PM_S0D1C2 | PXA95x_PM_I_Q_BIT);
		do {
			pollreg = PWRMODE;
		} while (pollreg != (PXA95x_PM_S0D1C2 | PXA95x_PM_I_Q_BIT));

#if defined(CONFIG_PXA9XX_ACIPC)
		if (0 == clear_DDR_avail_flag()) {
			CHECK_APPS_COMM_SYNC
#endif
#ifdef CONFIG_PXA_MIPSRAM
			MIPS_RAM_ADD_32K_TIME_STAMP(end_tick);
			MIPS_RAM_ADD_PM_TRACE(ENTER_D1_MIPS_RAM);
#endif
			ispt_power_state_d1();
			pm_logger_app_add_trace(6, PM_D1_ENTRY, end_tick,
					pollreg, CKENA, CKENB, CKENC,
					get_mipi_reference_control(), cpupwr);

			sram = (unsigned int) pxa95x_pm_regs.sram_map;
			pxa95x_cpu_standby(sram + 0x8000, sram + 0xa000 - 4,
					(unsigned int) &start_tick,
					power_state);
#if defined(CONFIG_PXA9XX_ACIPC)
			set_DDR_avail_flag();
		} else
			printk(KERN_WARNING "******EDDR WARNING: %s DDR_req=1 \
					shared flag=1. should not happen. \
					check the apps-comm sync \n", __func__);
#endif

		if (is_wkr_mg1_1677())
			restore_dma_registers();

		if (is_wkr_mg1_1468())
			enable_axi_lpm_exit();
#ifdef CONFIG_PXA_MIPSRAM
		MIPS_RAM_ADD_PM_TRACE(EXIT_D1_MIPS_RAM);
		MIPS_RAM_ADD_32K_TIME_STAMP(start_tick);
#endif
#ifdef CONFIG_PXA95x_DVFM_STATS
		calc_switchtime(end_tick, start_tick);
#endif

		wakeup_data = pm_postset_standby();

		pm_logger_app_add_trace(1, PM_D1_EXIT, start_tick, wakeup_data);

	} else if (state == POWER_MODE_D2) {
		power_state = PXA95x_PM_S0D2C2;
		pm_select_wakeup_src(PXA95x_PM_STANDBY, wakeup_src);
		pm_preset_standby();
		end_tick = OSCR4;
		
		/* pm logger debug - print when long wakeup */
		debug_check_active_time(end_tick, last_d2exit_time);
		/* if VCTCXO_EN is forced we aill not enable shutdown when
		 * entering D2 */
//#ifndef CONFIG_SND_SOC_D1981		 
		if (likely(!ForceVCTCXO_EN))
			disable_oscc_pout();
//#endif	
		if (PXA9xx_Force_D2 == ForceLPM) {
			/*Setting to forcedD2wakeups */
			LastForceLPM = PXA9xx_Force_D2;
			AD2D0ER = ForceLPMWakeup;
		}
#if defined(CONFIG_PXA9XX_ACIPC)
		if (0 == clear_DDR_avail_flag()) {
			CHECK_APPS_COMM_SYNC
#endif
#ifdef CONFIG_PXA_MIPSRAM
			MIPS_RAM_ADD_32K_TIME_STAMP(end_tick);
			MIPS_RAM_ADD_PM_TRACE(ENTER_D2_MIPS_RAM);
#endif
			ispt_power_state_d2();

			if (is_wkr_mg1_1358()) {
				accr_gcfs = ((ACCR & ACCR_GCFS_MASK) >>
				     ACCR_GCFS_OFFSET);
				accr = ACCR;
				accr &= ~ACCR_GCFS_MASK;
				ACCR = accr;
				do {
					acsr = ACSR;
				} while (((accr & ACCR_GCFS_MASK) >>
					  ACCR_GCFS_OFFSET !=
					  ((acsr & ACSR_GCFS_MASK) >>
					   ACSR_GCFS_OFFSET)));
			}
			if (is_wkr_mg1_1468())
				enable_axi_lpm_entry();

			if (is_wkr_mg1_1677())
				save_dma_registers();

			/*if (d2_led_toggle_flag) {
			    //configuring direction
			   __raw_writel(0x00100000, (void *)&(__REG(0x40e00404)));
			   //clearing gpio 52  (turning on D10 led)
			   __raw_writel(0x00100000, (void *)&(__REG(0x40e00028)));
			   } */
			
			/*For the first D2 only we will set AVCR to
			 * the defualt value which is given by OBM
			 * becuase OBM will increase this value by
			 * 25mV for ALVL1,ALVL2,ALVL3*/
			if (first_d2_entry == 1) {
				first_d2_entry = 0;
				AVCR = uboot_DefaultAvcrValue;
			}
			/* PWRMODE = D2 */
			PWRMODE = (PXA95x_PM_S0D2C2 | PXA95x_PM_I_Q_BIT);
			do {
				pollreg = PWRMODE;
			} while (pollreg != (PXA95x_PM_S0D2C2 |
						PXA95x_PM_I_Q_BIT));

			pm_logger_app_add_trace(7, PM_D2_ENTRY, end_tick,
					pollreg, CKENA, CKENB, CKENC, OSCC,
					get_mipi_reference_control(), cpupwr);

			sram = (unsigned int) pxa95x_pm_regs.sram_map;
			pxa95x_cpu_standby(sram + 0x8000, sram + 0xa000 - 4,
					   (unsigned int) &start_tick,
					   power_state);

			if (is_wkr_mg1_1468())
				enable_axi_lpm_exit();

			if (is_wkr_mg1_1677())
				restore_dma_registers();

			if (is_wkr_mg1_1358()) {
				accr = ACCR;
				accr &= ~ACCR_GCFS_MASK;
				accr |= ((accr_gcfs << ACCR_GCFS_OFFSET) &
				     ACCR_GCFS_MASK);
				ACCR = accr;
				do {
					acsr = ACSR;
				} while (((accr & ACCR_GCFS_MASK) >>
					  ACCR_GCFS_OFFSET !=
					  ((acsr & ACSR_GCFS_MASK) >>
					   ACSR_GCFS_OFFSET)));
			}

			/* for pm logger debug */
			turn_on_pm_logger_print();
			
			/* if (d2_led_toggle_flag) {
			   //setting gpio 52 to high (turning on D10 led)
			   __raw_writel(0x00100000, (void *)&(__REG(0x40e0001C)));
			   } */
#if defined(CONFIG_PXA9XX_ACIPC)
			set_DDR_avail_flag();
#endif
			DVFMLPMGlobalCount.D2_Enter_Exit_count++;
#ifdef CONFIG_PXA_MIPSRAM
			MIPS_RAM_ADD_PM_TRACE(EXIT_D2_MIPS_RAM);
			MIPS_RAM_ADD_32K_TIME_STAMP(start_tick);
#endif
#if defined(CONFIG_PXA9XX_ACIPC)
		} else
			printk(KERN_ERR "******EDDR WARNING: %s DDR_req=1 "
					"shared flag=1. should not happen. "
					"check the apps-comm sync\n",
					__func__);
#endif
//#ifndef CONFIG_SND_SOC_D1981
		/* if forced we will not enabled/disable VCTCXO shutdown */
		if (likely(!ForceVCTCXO_EN))
			enable_oscc_pout();
//#endif
#ifdef CONFIG_PXA95x_DVFM_STATS
		calc_switchtime(end_tick, start_tick);
#endif
		last_d2exit_time = start_tick;
		wakeup_data = pm_postset_standby();
		pm_logger_app_add_trace(1, PM_D2_EXIT, start_tick, wakeup_data);

		// temp fix: At syssleep command prevent suspend
		// need renable suspend at end of AT sysleep test, temporary set here 
		// should be done via user mode command at end of AT syssleep test 
	} else if (state == POWER_MODE_CG) {

		ispt_power_state_cgm();

		pm_select_wakeup_src(PXA95x_PM_CG, wakeup_src);


		/* Enable ACCU internal interrupt */		
		ICMR2 |= 0x00100000;		
		/* Turn on wakeup to core during idle mode */		
		aicsr = AICSR;		
		/* enable bit 10 - wakeup during core idle */		
		aicsr |= AICSR_WEIDLE;		
		/* do not write to status bits (write to clear) */		
		aicsr &= ~AICSR_STATUS_BITS;		
		AICSR = aicsr;

		if (PXA9xx_Force_CGM == ForceLPM) {
			LastForceLPM = PXA9xx_Force_CGM;
			ACGD0ER = ForceLPMWakeup;
		}
		end_tick = OSCR4;

#ifdef CONFIG_PXA_MIPSRAM
		MIPS_RAM_ADD_32K_TIME_STAMP(end_tick);
		MIPS_RAM_ADD_PM_TRACE(ENTER_CGM_MIPS_RAM);
#endif

		/* PWRMODE = C1 */
		PWRMODE = (PXA95x_PM_S0D0C1 | PXA95x_PM_I_Q_BIT);
		do {
			pollreg = PWRMODE;
		} while (pollreg != (PXA95x_PM_S0D0C1 | PXA95x_PM_I_Q_BIT));

		cken[0] = CKENA;
		cken[1] = CKENB;
		cken[2] = CKENC;

		pm_logger_app_add_trace(6, PM_CGM_ENTRY, end_tick, pollreg,
				cken[0], cken[1], cken[2],
				get_mipi_reference_control(), cpupwr);

		/* writing 1 to AGENP[2] to enable the wakeup detection
		 * window */
		AGENP |= AGENP_SAVE_WK;

		/*
		 * Turn off all clocks except PX1 bus clock,
		 * DMEMC clock and reserved bits
		 */
		CKENA = ckena_rsvd_bit_mask | (1 << 8);
		reg = CKENA;
		CKENB = ckenb_rsvd_bit_mask | (1 << 29);
		reg = CKENB;
		CKENC = ckenc_rsvd_bit_mask;
		reg = CKENC;

		/*
		 * making sure that registers are written with the correct value
		 * before moving on
		 */
		while ((CKENA != (ckena_rsvd_bit_mask | (1 << 8))) ||
				(CKENB != (ckenb_rsvd_bit_mask | (1 << 29))) ||
				(CKENC != ckenc_rsvd_bit_mask)) {
		};
		/* Turn off PX1 bus clock */
		CKENB &= ~(1 << 29);

		if (is_wkr_mg1_1274()) {
			pxa95x_pm_clear_Dcache_L2Cache();
		}
		/* enter clock gated mode by entering core idle */
		/*
		Since timer functional clock is disable on CGM exit, we can't use the original workaround (JIRA )
		Instead we will use constant delay at CGM exist to insure L2 is ready
		loop was calibrated to create ~25uSec delay.
		therefor for PP 2-7 set 6000 loop count and for PP1 set 1500
		*/
		if (cur_op < 2)
			pm_enter_cgm_deepidle(CPU_LOOP_COUNT_ON_EXIT_CGM_LOW_PP);
		else
			pm_enter_cgm_deepidle(CPU_LOOP_COUNT_ON_EXIT_CGM_HIGH_PP);
		if (is_wkr_mg1_1274()) {
			pxa95x_pm_invalidate_Dcache_L2Cache();
		}
		/* restore clocks after exiting from clock gated mode */
		CKENA = cken[0];
		reg = CKENA;
		CKENB = cken[1];
		reg = CKENB;
		CKENC = cken[2];
		reg = CKENC;

		/* clear AICSR wakeup status */		
		aicsr = AICSR;		
		/*do not write to status bits (write to clear) */		
		aicsr &= ~(AICSR_STATUS_BITS | AICSR_WEIDLE);		
		/*enable bit 10 - wakeup during core idle */		
		aicsr |= AICSR_WSIDLE;		
		AICSR = aicsr;


		start_tick = OSCR4;
#ifdef CONFIG_PXA95x_DVFM_STATS
		calc_switchtime(end_tick, start_tick);
#endif
		wakeup_data = pm_postset_clockgate();
		DVFMLPMGlobalCount.CGM_Enter_Exit_count++;
#ifdef CONFIG_PXA_MIPSRAM
		MIPS_RAM_ADD_PM_TRACE(EXIT_CGM_MIPS_RAM);
		MIPS_RAM_ADD_32K_TIME_STAMP(start_tick);
		pm_logger_app_add_trace(1, PM_CGM_EXIT, start_tick,
				wakeup_data);
#endif
	}
	/* this indicates exit d2 or cgm */
	ispt_power_state_exit_lpm();

	if (is_wkr_mg1_1274()) {
		/* Unmask all masked interrupts within LPM entry */
		ICMR = icmr[0];
		ICMR2 = icmr[1];
		ICMR3 = icmr[2];
	}
#ifdef CONFIG_ARMV7_OS_SAVE_AND_RESTORE
	/* Restore JTAG registers */
	armv7_jtag_registers_restore();
#endif

	/* restoring the performance counter and compensating its value */
	mipsram_reinit_counter();
	/* C2 is entered in power-down states - returning to normal C1 */
	pm_core_pwdn(CPU_PDWN_LPM_EXIT);
	if (!RepeatMode) {
		if (ForceLPM && LastForceLPM == ForceLPM)
			LastForceLPM = ForceLPM = PXA9xx_Force_None;
	}
}
#endif

static int pxa95x_pm_enter(suspend_state_t state)
{
	if (state == PM_SUSPEND_MEM)
		return pxa95x_pm_enter_sleep(&pxa95x_pm_regs);
	else if (state == PM_SUSPEND_STANDBY)
		return pxa95x_pm_enter_standby(&pxa95x_pm_regs);
	else
		return -EINVAL;
}

/*
 * Called after processes are frozen, but before we shut down devices.
 */
static int pxa95x_pm_prepare(void)
{
#ifdef CONFIG_IPM
	/* Disable deep idle when system enters low power mode */
	save_deepidle = enable_deepidle;
	enable_deepidle = 0;
#endif
	if (pm_state == PM_SUSPEND_MEM) {
		/* backup data in ISRAM */
		memcpy(pxa95x_pm_regs.sram, pxa95x_pm_regs.sram_map,
		       isram_size);
	} else if (pm_state == PM_SUSPEND_STANDBY) {
		/* FIXME: allocat SRAM to execute D1/D2 entry/exit code.
		 * Try not to use it in the future.
		 */
		/* backup data in ISRAM */
		memcpy(pxa95x_pm_regs.sram, pxa95x_pm_regs.sram_map, 1024);
	}

	return 0;
}

/*
 * Called after devices are re-setup, but before processes are thawed.
 */
static void pxa95x_pm_finish(void)
{
#ifdef CONFIG_IPM
	enable_deepidle = save_deepidle;
#endif
	if (pm_state == PM_SUSPEND_MEM) {
		/* restore data in ISRAM */
		memcpy(pxa95x_pm_regs.sram_map, pxa95x_pm_regs.sram,
		       isram_size);
	} else if (pm_state == PM_SUSPEND_STANDBY) {
		/* restore data in ISRAM */
		memcpy(pxa95x_pm_regs.sram_map, pxa95x_pm_regs.sram, 1024);
	}
	pm_state = PM_SUSPEND_ON;
}

static int pxa95x_pm_valid(suspend_state_t state)
{
	int ret = 1;

	if (state == PM_SUSPEND_MEM) {
		pm_state = PM_SUSPEND_MEM;
	} else if (state == PM_SUSPEND_STANDBY) {
		pm_state = PM_SUSPEND_STANDBY;
	} else {
		ret = 0;
	}
	return ret;
}

/*
 * Set to PM_DISK_FIRMWARE so we can quickly veto suspend-to-disk.
 */
static struct platform_suspend_ops pxa95x_pm_ops = {
	.valid = pxa95x_pm_valid,
	.prepare = pxa95x_pm_prepare,
	.enter = pxa95x_pm_enter,
	.finish = pxa95x_pm_finish,
};

#define pm_attr(_name, object)						\
static ssize_t _name##_store(struct kobject *kobj,			\
		struct kobj_attribute *attr,				\
		const char *buf, size_t n)				\
{									\
	sscanf(buf, "%u", &object);					\
	return n;							\
}									\
static ssize_t _name##_show(struct kobject *kobj,			\
		struct kobj_attribute *attr,				\
		char *buf)						\
{									\
	return sprintf(buf, "%u\n", object);				\
}									\
static struct kobj_attribute _name##_attr = {				\
	.attr	= {							\
		.name = __stringify(_name),				\
		.mode = 0644,						\
	},								\
	.show	= _name##_show,						\
	.store	= _name##_store,					\
}

pm_attr(sleeptime, pm_sleeptime);
pm_attr(msleeptime, pm_msleeptime);
#ifdef CONFIG_IPM

static int tokenizer(char **tbuf, const char *userbuf, ssize_t n,
		     char **tokptrs, int maxtoks)
{
	char *cp, *tok;
	char *whitespace = " \t\r\n";
	int ntoks = 0;

	cp = kmalloc(n + 1, GFP_KERNEL);
	if (!cp)
		return -ENOMEM;

	*tbuf = cp;
	memcpy(cp, userbuf, n);
	cp[n] = '\0';

	do {
		cp = cp + strspn(cp, whitespace);
		tok = strsep(&cp, whitespace);
		if (tok == NULL)
			break;
		if ((*tok == '\0') || (ntoks == maxtoks))
			break;
		tokptrs[ntoks++] = tok;
	} while (cp);

	return ntoks;
}

static ssize_t deepidle_show(struct kobject *kobj,
			     struct kobj_attribute *attr, char *buf)
{
	int len = 0;

	if (enable_deepidle & IDLE_D0CS)
		len += sprintf(buf + len, "D0CSIDLE, ");
	if (enable_deepidle & IDLE_D1)
		len += sprintf(buf + len, "D1IDLE, ");
	if (enable_deepidle & IDLE_D2)
		len += sprintf(buf + len, "D2IDLE, ");
	if (enable_deepidle & IDLE_CG)
		len += sprintf(buf + len, "CGIDLE, ");
	len += sprintf(buf + len, "D0IDLE\n");
	len += sprintf(buf + len, "Command: echo [set|unset] [d0cs|d1|d2|cg] "
		    "> deepidle\n");
	return len;
}

#define MAXTOKENS	80

static ssize_t deepidle_store(struct kobject *kobj,
			      struct kobj_attribute *attr, const char *buf,
			      size_t len)
{
	int error = 0;
	char *tbuf = NULL;
	char *token[MAXTOKENS];
	int ntoks = tokenizer(&tbuf, buf, len, (char **) &token, MAXTOKENS);

	if (ntoks <= 0) {
		error = ntoks;
		goto out;
	}

	if (strcmp(token[0], "set") == 0) {
		if (strcmp(token[1], "d0cs") == 0)
			enable_deepidle |= IDLE_D0CS;
		else if (strcmp(token[1], "d1") == 0)
			enable_deepidle |= IDLE_D1;
		else if (strcmp(token[1], "d2") == 0)
			enable_deepidle |= IDLE_D2;
		else if (strcmp(token[1], "cg") == 0)
			enable_deepidle |= IDLE_CG;
		else
			error = -EINVAL;
	} else if (strcmp(token[0], "unset") == 0) {
		if (strcmp(token[1], "d0cs") == 0)
			enable_deepidle &= ~IDLE_D0CS;
		else if (strcmp(token[1], "d1") == 0)
			enable_deepidle &= ~IDLE_D1;
		else if (strcmp(token[1], "d2") == 0)
			enable_deepidle &= ~IDLE_D2;
		else if (strcmp(token[1], "cg") == 0)
			enable_deepidle &= ~IDLE_CG;
		else
			error = -EINVAL;
	} else {
		if (strcmp(token[0], "1") == 0)
			enable_deepidle = IDLE_D0CS;
		else if (strcmp(token[0], "0") == 0)
			enable_deepidle = IDLE_D0;
		else
			error = -EINVAL;
	}
out:
	kfree(tbuf);
	return error ? error : len;
}

static struct kobj_attribute deepidle_attr = {
	.attr = {
		.name = __stringify(deepidle),
		.mode = 0644,
	},
	.show = deepidle_show,
	.store = deepidle_store,
};
#endif

static ssize_t cp_show(struct kobject *kobj, struct kobj_attribute *attr,
		       char *buf)
{
	return sprintf(buf, "%u\n", pm_cp);
}

static int pm_cp_disabled;
static int __init pm_nocp_setup(char *this_opt)
{
	/* Once "nocp" token is present disable CP regardless of the value */
	pm_cp_disabled = 1;
	return 1;
}

__setup("tavorcfg_nocp", pm_nocp_setup);

static ssize_t cp_store(struct kobject *kobj, struct kobj_attribute *attr,
			const char *buf, size_t len)
{
	if (!pm_cp_disabled)
		sscanf(buf, "%u", &pm_cp);
#if defined(CONFIG_PXA9XX_ACIPC)
	/* Must be called prior to acipc_start_cp_constraints
	 * This will update acipc driver on cp status */
	set_acipc_cp_enable(pm_cp);
#endif
	if (pm_cp) {
		/* release CP */
		__raw_writel(0x11, pm_membase + CSER_OFF);
#if defined(CONFIG_PXA9XX_ACIPC)
		acipc_start_cp_constraints();
#endif
	} else {
		/* reset CP */
		__raw_writel(0x0, pm_membase + CSER_OFF);
	}
	return len;
}

static struct kobj_attribute cp_attr = {
	.attr = {
		.name = __stringify(cp),
		.mode = 0644,
	},
	.show = cp_show,
	.store = cp_store,
};

static ssize_t temp_show(struct kobject *kobj, struct kobj_attribute *attr,
			 char *buf)
{
	int len = 0;
	len += sprintf(buf + len, "OVH %x ", OVH);
	len += sprintf(buf + len, "PSR %x ", PSR);
	len += sprintf(buf + len, "temp_of_core %x\n", temp_of_core);
	return len;
}

static struct kobj_attribute temp_attr = {
	.attr = {
		.name = __stringify(temp),
		.mode = 0644,
	},
	.show = temp_show,
};

static ssize_t reg_show(struct kobject *kobj, struct kobj_attribute *attr,
			char *buf)
{
	int len = 0;

	len += sprintf(buf + len, "ARSR:0x%08x\n", ARSR);
	len += sprintf(buf + len, "PSR:0x%08x\n", PSR);
	len += sprintf(buf + len, "AD1D0SR:0x%08x\n", AD1D0SR);
	len += sprintf(buf + len, "AD2D0SR:0x%08x\n", AD2D0SR);
	len += sprintf(buf + len, "AD3SR:0x%08x\n", AD3SR);
	if (!cpu_is_pxa970())
		len += sprintf(buf + len, "PWSR:0x%08x\n", PWSR);
	len += sprintf(buf + len, "OVH:0x%08x\n", OVH);
	len += sprintf(buf + len, "PMCR:0x%08x\n", PMCR);

	return len;
}

static struct kobj_attribute reg_attr = {
	.attr = {
		.name = __stringify(reg),
		.mode = 0644,
	},
	.show = reg_show,
};

static struct attribute *g[] = {
	&sleeptime_attr.attr,
	&msleeptime_attr.attr,
#ifdef CONFIG_IPM
	&deepidle_attr.attr,
#endif
	&cp_attr.attr,
	&temp_attr.attr,
	&reg_attr.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = g,
};

static void log_overheating_event(unsigned char *str_log)
{
	struct timeval now;

	do_gettimeofday(&now);
	pr_warning("[%10lu]%s PSR.TSS 0x%x\n",
		   now.tv_sec, str_log, temp_of_core);
	/* TODO: record the warning into NVM and nodify the user */
}

static void detect_core_temp(unsigned long data)
{
	struct timer_list *timer = &temp_detecting_timer;
	static int conscutive_counter, average_meas;
	/* detect the temp of core */
	if (cpu_is_pxa970())
		temp_of_core = (PSR >> PSR_TSS_OFF) & 0x1ff;
	else
		temp_of_core = (PSR >> PSR_TSS_OFF) & 0x7;

	if (temp_of_core == 0)
		temp_of_core = 80;
	else
		temp_of_core = OVH_TO_TEMP_CONVERT(temp_of_core);
	average_meas += (temp_of_core / 3);
	conscutive_counter++;
	if (conscutive_counter == 3) {
		if (average_meas < OVH_TO_TEMP_CONVERT(TSS_THRESHOLD)
			&& !(PMCR & PMCR_TIE)) {
			log_overheating_event("INFO: AP core cooled down!");
			PMCR |= PMCR_TIE;
			temperture_sensor_int_high_freq_pp_callback(CORE_COLLING_DETECTED);
		} else {
			/* reset the timer */
			mod_timer(timer, jiffies + FRQ_TEMP);
		}

		pr_debug("%s:PSR 0x%x PMCR 0x%x OVH 0x%x\n", __func__, PSR, PMCR, OVH);
		average_meas = conscutive_counter = 0;
	} else { /*waiting for 3 consecutive reads*/
		/* reset the timer */
		mod_timer(timer, jiffies + FRQ_TEMP);
	}
}

static irqreturn_t core_overhearting_irq(int irq, void *data)
{
	struct timer_list *timer = &temp_detecting_timer;

	temperture_sensor_int_high_freq_pp_callback(CORE_OVERHEATING_DETECTED);
	pr_debug("%s:PSR 0x%x PMCR 0x%x OVH 0x%x\n", __func__, PSR, PMCR, OVH);
	if (PMCR & PMCR_TIS) {
		/* disable the interrupt & clear the status bit */
		PMCR &= ~PMCR_TIE;
		PMCR |= PMCR_TIS;

		log_overheating_event("WARNING: AP core is OVERHEATING!");

		/* start the timer for measuring the temp of core */
		mod_timer(timer, jiffies + FRQ_TEMP);
	}
	return IRQ_HANDLED;
}

static void overheating_init(void)
{
	struct timer_list *timer = &temp_detecting_timer;
	int retval;

	/* irq setup after old hardware state is cleaned up */
	retval = request_irq(IRQ_SGP, core_overhearting_irq,
			     IRQF_DISABLED, "Overheating", NULL);
	if (retval != 0) {
		printk(KERN_ERR "%s: can't get irq %i, err %d\n",
		       "Overheating", IRQ_SGP, retval);
		return;
	}

	OVH = (OVH_OTIS_DEFAULT) << OVH_OTIF_OFF;
	OVH |= (OVH_OVWF_DEFAULT) << OVH_OVWF_OFF;
	PMCR |= (PMCR_TIE | PMCR_TIS);
	OVH |= OVH_TEMP_EN;
	OVH |= OVH_OWM;

	/* initialize the timer for measuring the temp of core */
	init_timer(timer);
	timer->function = detect_core_temp;
}

static unsigned char __iomem *bpb_membase;

/*
 * Query whether the specified pin waked up system.
 *
 * return 1 -- yes, 0 -- no, negative -- failure
 */
int pxa95x_query_gwsr(int pin)
{
	int off, ret;

	if (pin < 0)
		return -EINVAL;
	off = pin / 32 + 1;
	/* read data from GWSR(x), x is [1..4] */
	ret = __raw_readl(bpb_membase + GWSR(off));
	ret = (ret >> (pin - (off - 1) * 32)) & 1;
	return ret;
}

static int __init pxa95x_pm_init(void)
{
	int rc = 0;
#ifdef CONFIG_IPM
	unsigned int oscc, dmemvlr;

	suspend_set_ops(&pxa95x_pm_ops);

#ifdef CONFIG_IPM_DEEPIDLE
	enable_deepidle |= IDLE_D0CS;
#endif
#ifdef CONFIG_IPM_D2IDLE
	enable_deepidle |= IDLE_D2;
#endif
#ifdef CONFIG_IPM_D1IDLE
	enable_deepidle |= IDLE_D1;
#endif
#ifdef CONFIG_IPM_CGIDLE
	enable_deepidle |= IDLE_CG;
#endif
	orig_poweroff = pm_power_off;
	pm_power_off = pxa95x_pm_poweroff;

	/* if nowhere use tout_s0, it would be disable */
	enable_oscc_tout_s0();
	disable_oscc_tout_s0();
	oscc = OSCC;
	if (!cpu_is_pxa970())
		/* Disable CLK_TOUT in S0/S2/S3 state */
		oscc &= ~0x600;
	else
		/* Clear all clearalways bits */
		oscc &= ~0x6600;
	/* Disable TD bit */
	oscc &= ~0x10000;
	/* configuring VCTCXO_EN to be used by PMIC
	 * as a trigger for volateg change */
	oscc |= OSCC_VCTVCEN;
	/* wait time for voltage change completion
	 * is set to 1.5 32Khz cycles */
	oscc &= ~(0x7 << OSCC_VCTVSTB_OFFSET);
	oscc |= (0x1 << OSCC_VCTVSTB_OFFSET);

	if (!cpu_is_pxa970()) {
		dmemvlr = DMEMVLR;
		dmemvlr &= ~(0x3 << DMEMVLR_DMCHV_OFFSET);
		dmemvlr |= (0x1 << DMEMVLR_DMCHV_OFFSET);
		DMEMVLR = dmemvlr;
	}

	{
		static u32 *base;
#define GEN_REG3	0x42404008	/* general Register 3 */
#define GEN_REG3_SPLGEN	   (1 << 19)
		base = ioremap_nocache(GEN_REG3, 4);
		/* system pll auto gating set */
		*base |= GEN_REG3_SPLGEN;
		iounmap(base);
	}
	/* set VCTSTB as 0x11 ~ 0.5ms */
	OSCC = (oscc & ~(0xFF << 24)) | (0x11 << 24);
#ifndef CONFIG_SND_SOC_D1981	
	/* Enable CLK_POUT */
	enable_oscc_pout();
#endif
#endif
	overheating_init();

	pxa95x_pm_set_clk("GPIOCLK", 1);

	if (sysfs_create_group(power_kobj, &attr_group))
		return -1;

	bpb_membase = ioremap(BPB_START, BPB_END - BPB_START + 1);

#ifdef CONFIG_PXA95x_DVFM
	/* Init debugfs folder and files. */
	pxa_9xx_power_init_debugfs();
#endif

	pxa95x_sysbus_init(&pxa95x_pm_regs);

	/* make sure that sram bank 0 is not off in D1
	 * base on JIRA MG1-1021, it should be set to 0x01
	 */
	AD1R = 0x01;

	/* setting bit 2 (SV_WK) to enable wakeup event detection in clock
	 * gated mode entery sequecne */
	AGENP |= 0x4;

	/* setting bit 4 to avoid hangs in C2 entry (MG2-388) */
	if (is_wkr_mg2_388())
		AGENP |= 0x10;

		/* Setting C2 as default */
#ifdef CONFIG_PXA95x_DVFM
		pm_core_pwdn(CPU_PDWN_LPM_EXIT);
#endif

	/* Enabling ACCU and BPMU interrupts */
	ICMR2 |= 0x100000;
	ICMR3 |= 0x100000;

	/* Clearing RDH bit in Startup */
	ASCR &= ~ASCR_RDH;

	/* if SRAM is allocate to GB by uboot param, the lpm code will not be
	 * copied to SRAM. note, power cannot be enabled if sram allocate to
	 * GB */
	if (!disable_sram_use)
		pxa95x_init_standby((unsigned int) pxa95x_pm_regs.sram_map +
				0x8000);

#ifdef CONFIG_PXA95x_DVFM
	cken_rsvd_bit_mask_setup();
#endif

	/* Reading ProductID Full Layer info from FuseReg */
	is_wkr_mg1_1274_value = is_wkr_mg1_1274();

	if (dvfm_find_index("User", &user_index) == 0){
		rc |= dvfm_enable_op_name("806M", user_index);
		if (rc)
			printk(KERN_ERR "Error disable op\n");
	}
	else
		pr_err("Can't get \"User\" index for DVFM in %s.\n", __func__);

	return 0;
}

late_initcall(pxa95x_pm_init);

/* uboot parameters handling */

static int uboot_sram_allocation(char *s)
{
	disable_sram_use = 1;
	/* no SRAM so pm must be disabled */
	PowerDisabled = 1;
	return 1;
}

__setup("tavorcfg_bsram2gb=", uboot_sram_allocation);

/* uboot parameters handling code end */
