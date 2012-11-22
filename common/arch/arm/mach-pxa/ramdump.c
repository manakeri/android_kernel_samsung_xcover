/*
 *  linux/arch/arm/mach-pxa/ramdump.c
 *
 *  Support for the Marvell PXA RAMDUMP error handling capability.
 *
 *  Author:     Anton Eidelman (anton.eidelman@marvell.com)
 *  Created:    May 20, 2010
 *  Copyright:  (C) Copyright 2006 Marvell International Ltd.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  publishhed by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/errno.h>
#include <linux/notifier.h>
#include <linux/reboot.h>
#include <linux/crc32.h>

#include <asm/system.h>
#include <asm/ptrace.h> /*pt_regs*/
#include <linux/sched.h> /* task_struct */
#include <asm/cacheflush.h>

#include <mach/ramdump.h>
#include <mach/hardware.h>
#include <mach/pxa95x-regs.h>
#ifdef CONFIG_PXA_MIPSRAM
#include <linux/mipsram.h>
#endif

#include <mach/ramdump_defs.h> /* common definitions reused in OSL */

/* CPU mode registers */
struct mode_regs {
	unsigned spsr;
	unsigned sp;
	unsigned lr;
};
struct usr_regs {
	unsigned sp;
	unsigned lr;
};
struct fiq_regs {
	unsigned spsr;
	unsigned r8;
	unsigned r9;
	unsigned r10;
	unsigned r11;
	unsigned r12;
	unsigned sp;
	unsigned lr;
};

/* CP15 registers */
struct cp15_regs {
	unsigned id;		/* CPU ID */
	unsigned cr;		/* Control */
	unsigned aux_cr;	/* Auxiliary Control */
	unsigned ttb;		/* TTB; PJ4: ttb0 */
	unsigned da_ctrl;	/* Domain Access Control */
	unsigned cpar;		/* Co-processor access control */
	unsigned fsr;		/* PJ4: DFSR */
	unsigned far;		/* PJ4: DFAR */
	unsigned procid;	/* Process ID; PJ4: Context ID */
};

/* CP14 registers */
struct cp14_regs {
	unsigned ccnt;
	unsigned pmnc;
};

/* CP7 registers (L2C/BIU errors)*/
struct cp7_regs {
	unsigned errlog;
	unsigned erradrl;
	unsigned erradru;
};

/* CP6 registers (Interrupt Controller) */
struct cp6_regs {
	unsigned icip;
	unsigned icmr;
	unsigned iclr;
	unsigned icfp;
	unsigned icpr;
	unsigned ichp;
	unsigned icip2;
	unsigned icmr2;
	unsigned iclr2;
	unsigned icfp2;
	unsigned icpr2;
	unsigned icip3;
	unsigned icmr3;
	unsigned iclr3;
	unsigned icfp3;
	unsigned icpr3;
};

/* PJ4 specific cp15 regs. DONT EXTEND, ADD NEW STRUCTURE TO ramdump_state */
struct cp15_regs_pj4 {
	unsigned seccfg;	/* Secure Configuration */
	unsigned secdbg;	/* Secure Debug Enable */
	unsigned nsecac;	/* Non-secure Access Control */
	unsigned ttb1;		/* TTB1; TTB0 is cp15_regs.ttb */
	unsigned ttbc;		/* TTB Control */
	unsigned ifsr;		/* Instruction FSR; Data FSR is cp15_regs.fsr */
	unsigned ifar;		/* Instruction FAR; Data FAR is cp15_regs.far */
	unsigned auxdfsr;	/* Auxiliary DFSR */
	unsigned auxifsr;	/* Auxiliary IFSR */
	unsigned pa;		/* PA: physical address after translation */
	unsigned prremap;	/* Primary Region Remap */
	unsigned nmremap;	/* Normal Memory Remap */
	unsigned istat;		/* Interrupt Status */
	unsigned fcsepid;	/* FCSE PID */
	unsigned urwpid;	/* User Read/Write Thread and Process ID */
	unsigned uropid;	/* User Read/Only Thread and Process ID */
	unsigned privpid;	/* Priviledged Only Thread and Process ID */
	unsigned auxdmc0;	/* Auxiliary Debug Modes Control 0 */
	unsigned auxdmc1;	/* Auxiliary Debug Modes Control 1 */
	unsigned auxfmc;	/* Auxiliary Functional Modes Control */
	unsigned idext;		/* CPU ID code extension */
};

struct l2c_pj4_regs {
	unsigned l2errcnt;	/* L2 Cache Error Counter */
	unsigned l2errth;	/* L2 Cache Error Threshold */
	unsigned l2errcapt;	/* L2 Cache Error Capture */
};

/* PJ4 performance monitor */
struct pfm_pj4_regs {
	unsigned ctrl;
	unsigned ceset;
	unsigned ceclr;
	unsigned ovf;
	unsigned softinc;
	unsigned csel;
	unsigned ccnt;
	unsigned evsel;
	unsigned pmcnt;
	unsigned uen;
	unsigned ieset;
	unsigned ieclr;
};

/* AP Clock Control registers */
struct acc_regs {
	unsigned accr;
	unsigned acsr;
	unsigned aicsr;
	unsigned d0cken_a;
	unsigned d0cken_b;
	unsigned accr1;
	unsigned d0cken_c;
	unsigned cfgreg0;	/* MG1 Arbitration Control */
};

/* Other SoC registers */
struct soc_regs {
	unsigned avcr;
	unsigned ser_fuse_reg2;
	union {
		struct {
			unsigned mdtac;
			unsigned rcomp;
		}; /* Tavor DMC */
	};
};

/* Main RAMDUMP data structure */
struct ramdump_state {
	unsigned rdc_va;	/* RDC header virtual addres */
	unsigned rdc_pa;	/* RDC header physical addres */
	char text[100];
	unsigned err;
	struct pt_regs regs;	/* saved context */
	struct thread_info *thread;
	struct mode_regs svc;
	struct usr_regs usr;
	struct mode_regs abt;
	struct mode_regs und;
	struct mode_regs irq;
	struct fiq_regs fiq;
	struct cp15_regs cp15;
	/* Up to this point same structure for XSC and PJ4 */
	union {
		struct {			/* 21 total */
			struct cp14_regs cp14;	/* 2 */
			struct cp6_regs cp6;	/* 16 */
			struct cp7_regs cp7;	/* 3 */
		}; /* XSC */
		struct {			/* 21 total */
			struct cp15_regs_pj4 cp15pj4;
		}; /* PJ4 */
	};
	struct acc_regs acc;
	struct l2c_pj4_regs l2cpj4;
	struct pfm_pj4_regs pfmpj4;
	struct soc_regs soc;
} ramdump_data;

static int debug_level_param=0;

static void *isram_va; /* ioremapped va for ISRAM access */
//static struct rdc_area *rdc_va;/* ioremapped va for rdc_area access */
/* SAMSUNG_PPS: changed this to skip i2c call in interrupt context(ramdump) becuase i2c is using mutex so it should not sleep */
struct rdc_area *rdc_va;/* ioremapped va for rdc_area access */
/************************************************************************/
/*				Internal prototypes			*/
/************************************************************************/
static void ramdump_save_static_context(struct ramdump_state *d);
static void ramdump_save_current_context(struct ramdump_state *d);
static void ramdump_save_isram(void);
static void ramdump_flush_caches(void);
static void ramdump_fill_rdc(void);
static void save_peripheral_regs(struct ramdump_state *d);
extern void arch_reset(char mode, const char *cmd);

/************************************************************************/
/*				RDC address setup			*/
/************************************************************************/
static unsigned rdc_pa;

static int __init ramdump_rdc_setup(char *str)
{
	rdc_pa = simple_strtoul(str, NULL, 16);
	return 1;
}
__setup("RDCA=", ramdump_rdc_setup);

/************************************************************************/
/*				RAMDUMP panic notifier			*/
/************************************************************************/
static int
ramdump_panic(struct notifier_block *this, unsigned long event, void *ptr)
{
//#ifdef CONFIG_MACH_GFORCE
	extern unsigned int samsung_debug_enable_flag;
	if(samsung_debug_enable_flag !=0 && debug_level_param > 0) {
//#endif
		printk(KERN_ERR "RAMDUMP STARTED\n");
		ramdump_fill_rdc();
		ramdump_save_current_context(&ramdump_data);
		ramdump_save_static_context(&ramdump_data);
		ramdump_save_isram();
// + SAMSUNG_SSENI
		{
			extern void sec_debug_dump(void *, int);
			sec_debug_dump(ptr, ramdump_data.err);
		}
// - SAMSUNG_SSENI
		ramdump_flush_caches();
		printk(KERN_ERR "RAMDUMP DONE\n");
		/* Reset right away, do not return. Two issues with reset done
		by the main panic() implementation:
		1) The setup_mm_for_reboot() called from arm_machine_restart()
		corrupts the current MM page tables,
		which is bad for offline debug.
		2) The current kernel stack is corrupted by other functions
		invoked after this returns:
		the stack frame becomes invalid in offline debug.
		*/
		arch_reset('h', NULL);
//#if defined(CONFIG_MACH_GFORCE) || defined(CONFIG_MACH_ALKON) || defined(CONFIG_MACH_JETTA)
	} else {
		printk(KERN_ERR "DEBUG LEVEL = LOW\n");
		printk(KERN_ERR "RAMDUMP SKIPPED due to samsung_debug setting\n");
		printk(KERN_ERR "SILENT RESET IN PROGRESS\n");
	}
//#endif
	return NOTIFY_DONE;
}

static struct notifier_block panic_block = {
	.notifier_call = ramdump_panic,
};

/*
*/

static int sec_debug_normal_reboot_handler(struct notifier_block *nb,
					   unsigned long l, void *p)
{
	rdc_va->header.signature = 0x0;

	return 0;
}

static struct notifier_block nb_reboot_block = {
	.notifier_call = sec_debug_normal_reboot_handler
};

/************************************************************************/
/*				inline asm helpers			*/
/************************************************************************/
#define get_reg_asm(instruction) ({	\
	unsigned reg;			\
	asm(instruction : "=r" (reg) : );	\
	reg; })

static inline void get_banked_regs(unsigned *dest, unsigned mode)
{
	register unsigned *rdest asm("r0") = dest;
	register unsigned rmode asm("r1") = mode;
	register unsigned cpsr asm("r2");
	register unsigned scr asm("r3");
	asm volatile(
		"mrs	r2, cpsr\n"
		"bic	r3, r2, #0x1f @ clear mode\n"
		"orr	r3, r3, r1 @ set target mode\n"
		"msr	cpsr, r3\n"
		"mrs	r3,spsr\n"
		"cmp	r2, #0x11\n"
		"stmne	r0, {r3,r13-r14}\n"
		"stmeq	r0, {r3,r8-r14}\n"
		"msr	cpsr, r2 @ restore original mode\n"
		: "=r" (cpsr), "=r" (scr)
		: "r" (rdest), "r" (rmode), "r" (cpsr), "r" (scr)
		: "memory", "cc");
}
static inline void get_usr_regs(unsigned *dest, unsigned mode)
{
	register unsigned *rdest asm("r0") = dest;
	register unsigned rmode asm("r1") = mode;
	register unsigned cpsr asm("r2");
	register unsigned scr asm("r3");
	asm volatile(
		"mrs	r2, spsr\n"
		"bic	r3, r2, #0x1f @ clear mode\n"
		"orr	r3, r3, r1 @ set usr mode\n"
		"msr	spsr, r3\n"
		"stm	r0, {r13-r14}^\n"
		"msr	spsr, r2 @ restore original spsr\n"
		: "=r" (cpsr), "=r" (scr)
		: "r" (rdest), "r" (rmode), "r" (cpsr), "r" (scr)
		: "memory", "cc");
}

static void save_peripheral_regs(struct ramdump_state *d)
{
	d->acc.accr = ACCR;
	d->acc.acsr = ACSR;
	d->acc.aicsr = AICSR;
	d->acc.accr1 = ACCR1;
	d->acc.d0cken_a = CKENA;
	d->acc.d0cken_b = CKENB;
	d->acc.d0cken_c = CKENC;
#ifdef CONFIG_CPU_PJ4
	d->acc.cfgreg0 = __REG(0x48100f10);
#endif
#ifdef CONFIG_PXA95x
	d->soc.avcr = __REG(0x40f50094);
	d->soc.ser_fuse_reg2 = __REG(0x40f50208);
	if (cpu_is_pxa968() || cpu_is_pxa955()) {
		d->soc.mdtac = __REG(0x48100010);
		d->soc.rcomp = __REG(0x48100100);
	}
#endif
}

/************************************************************************/
/*				RAMDUMP state save			*/
/************************************************************************/
/*
	ramdump_save_static_context
	Saves general CPU registers state into the ramdump.
*/
static void ramdump_save_static_context(struct ramdump_state *d)
{
	/* mode banked regs */
	get_banked_regs(&d->abt.spsr, ABT_MODE);
	get_banked_regs(&d->und.spsr, UND_MODE);
	get_banked_regs(&d->irq.spsr, IRQ_MODE);
	get_banked_regs(&d->fiq.spsr, FIQ_MODE);
	get_banked_regs(&d->svc.spsr, SVC_MODE);

	/* USR mode banked regs */
	get_usr_regs(&d->usr.sp, USR_MODE);

	/* cp15 */
	d->cp15.id	=	get_reg_asm("mrc p15, 0, %0, c0, c0, 0");
	d->cp15.cr	=	get_reg_asm("mrc p15, 0, %0, c1, c0, 0");
	d->cp15.aux_cr	=	get_reg_asm("mrc p15, 0, %0, c1, c0, 1");
	d->cp15.ttb	=	get_reg_asm("mrc p15, 0, %0, c2, c0, 0");
	d->cp15.da_ctrl	=	get_reg_asm("mrc p15, 0, %0, c3, c0, 0");
#ifndef CONFIG_CPU_PJ4 /* XSC */
	d->cp15.cpar	=	get_reg_asm("mrc p15, 0, %0, c15,c1, 0");
#else /* PJ4 */
	d->cp15.cpar	=	get_reg_asm("mrc p15, 0, %0, c1, c0, 2");
#endif
	d->cp15.fsr	=	get_reg_asm("mrc p15, 0, %0, c5, c0, 0");
	d->cp15.far	=	get_reg_asm("mrc p15, 0, %0, c6, c0, 0");
	d->cp15.procid	=	get_reg_asm("mrc p15, 0, %0, c13,c0, 0"); /* PJ4: context id */

#ifndef CONFIG_CPU_PJ4 /* XSC */
	/* cp14 */
	d->cp14.ccnt	=	get_reg_asm("mrc p14, 0, %0, c1, c1, 0");
	d->cp14.pmnc	=	get_reg_asm("mrc p14, 0, %0, c0, c1, 0");

	/* cp6 */
	d->cp6.icip	=	get_reg_asm("mrc p6, 0, %0, c0, c0, 0");
	d->cp6.icmr	=	get_reg_asm("mrc p6, 0, %0, c1, c0, 0");
	d->cp6.iclr	=	get_reg_asm("mrc p6, 0, %0, c2, c0, 0");
	d->cp6.icfp	=	get_reg_asm("mrc p6, 0, %0, c3, c0, 0");
	d->cp6.icpr	=	get_reg_asm("mrc p6, 0, %0, c4, c0, 0");
	d->cp6.ichp	=	get_reg_asm("mrc p6, 0, %0, c5, c0, 0");
	d->cp6.icip2	=	get_reg_asm("mrc p6, 0, %0, c6, c0, 0");
	d->cp6.icmr2	=	get_reg_asm("mrc p6, 0, %0, c7, c0, 0");
	d->cp6.iclr2	=	get_reg_asm("mrc p6, 0, %0, c8, c0, 0");
	d->cp6.icfp2	=	get_reg_asm("mrc p6, 0, %0, c9, c0, 0");
	d->cp6.icpr2	=	get_reg_asm("mrc p6, 0, %0, c10, c0, 0");
	d->cp6.icip3	=	get_reg_asm("mrc p6, 0, %0, c11, c0, 0");
	d->cp6.icmr3	=	get_reg_asm("mrc p6, 0, %0, c12, c0, 0");
	d->cp6.iclr3	=	get_reg_asm("mrc p6, 0, %0, c13, c0, 0");
	d->cp6.icfp3	=	get_reg_asm("mrc p6, 0, %0, c14, c0, 0");
	d->cp6.icpr3	=	get_reg_asm("mrc p6, 0, %0, c15, c0, 0");

	/* cp7 */
	d->cp7.errlog	=	get_reg_asm("mrc p7, 0, %0, c0, c2, 0");
	d->cp7.erradrl	=	get_reg_asm("mrc p7, 0, %0, c1, c2, 0");
	d->cp7.erradru	=	get_reg_asm("mrc p7, 0, %0, c2, c2, 0");
#else /* PJ4 */
	d->cp15pj4.seccfg	= get_reg_asm("mrc p15, 0, %0, c1, c1, 0");
	d->cp15pj4.secdbg	= get_reg_asm("mrc p15, 0, %0, c1, c1, 1");
	d->cp15pj4.nsecac	= get_reg_asm("mrc p15, 0, %0, c1, c1, 2");
	d->cp15pj4.ttb1		= get_reg_asm("mrc p15, 0, %0, c2, c0, 1");
	d->cp15pj4.ttbc		= get_reg_asm("mrc p15, 0, %0, c2, c0, 2");
	d->cp15pj4.ifsr		= get_reg_asm("mrc p15, 0, %0, c5, c0, 1");
	d->cp15pj4.ifar		= get_reg_asm("mrc p15, 0, %0, c6, c0, 2");
	d->cp15pj4.auxdfsr	= get_reg_asm("mrc p15, 0, %0, c5, c1, 0");
	d->cp15pj4.auxifsr	= get_reg_asm("mrc p15, 0, %0, c5, c1, 1");
	d->cp15pj4.pa		= get_reg_asm("mrc p15, 0, %0, c7, c4, 0");
	d->cp15pj4.prremap	= get_reg_asm("mrc p15, 0, %0, c10, c2, 0");
	d->cp15pj4.nmremap	= get_reg_asm("mrc p15, 0, %0, c10, c2, 1");
	d->cp15pj4.istat	= get_reg_asm("mrc p15, 0, %0, c12, c1, 0");
	d->cp15pj4.fcsepid	= get_reg_asm("mrc p15, 0, %0, c13, c0, 0");
	d->cp15pj4.urwpid	= get_reg_asm("mrc p15, 0, %0, c13, c0, 2");
	d->cp15pj4.uropid	= get_reg_asm("mrc p15, 0, %0, c13, c0, 3");
	d->cp15pj4.privpid	= get_reg_asm("mrc p15, 0, %0, c13, c0, 4");
	d->cp15pj4.auxdmc0	= get_reg_asm("mrc p15, 1, %0, c15, c1, 0");
	d->cp15pj4.auxdmc1	= get_reg_asm("mrc p15, 1, %0, c15, c1, 1");
	d->cp15pj4.auxfmc	= get_reg_asm("mrc p15, 1, %0, c15, c2, 0");
	d->cp15pj4.idext	= get_reg_asm("mrc p15, 1, %0, c15, c12, 0");
	d->l2cpj4.l2errcnt	= get_reg_asm("mrc p15, 1, %0, c15, c9, 6");
	d->l2cpj4.l2errth	= get_reg_asm("mrc p15, 1, %0, c15, c9, 7");
	d->l2cpj4.l2errcapt	= get_reg_asm("mrc p15, 1, %0, c15, c11, 7");
	d->pfmpj4.ctrl		= get_reg_asm("mrc p15, 0, %0, c9, c12, 0");
	d->pfmpj4.ceset		= get_reg_asm("mrc p15, 0, %0, c9, c12, 1");
	d->pfmpj4.ceclr		= get_reg_asm("mrc p15, 0, %0, c9, c12, 2");
	d->pfmpj4.ovf		= get_reg_asm("mrc p15, 0, %0, c9, c12, 3");
	d->pfmpj4.softinc	= get_reg_asm("mrc p15, 0, %0, c9, c12, 4");
	d->pfmpj4.csel		= get_reg_asm("mrc p15, 0, %0, c9, c12, 5");
	d->pfmpj4.ccnt		= get_reg_asm("mrc p15, 0, %0, c9, c13, 0");
	d->pfmpj4.evsel		= get_reg_asm("mrc p15, 0, %0, c9, c13, 1");
	d->pfmpj4.pmcnt		= get_reg_asm("mrc p15, 0, %0, c9, c13, 2");
	d->pfmpj4.uen		= get_reg_asm("mrc p15, 0, %0, c9, c14, 0");
	d->pfmpj4.ieset		= get_reg_asm("mrc p15, 0, %0, c9, c14, 1");
	d->pfmpj4.ieclr		= get_reg_asm("mrc p15, 0, %0, c9, c14, 2");

#endif
	save_peripheral_regs(d);
}

/*
	Save current register context if no dynamic context has been filled in
*/
static void ramdump_save_current_context(struct ramdump_state *d)
{
	/* check if panic was called directly, then regs will be empty */
	if (d->regs.uregs[16] == 0) {
		/* let's fill up regs as current */
		d->regs.uregs[0] = 0xDEADDEAD; /* use DEADDEAD as a marking */
		d->regs.uregs[1] = get_reg_asm("mov %0, r1");
		d->regs.uregs[2] = get_reg_asm("mov %0, r2");
		d->regs.uregs[3] = get_reg_asm("mov %0, r3");
		d->regs.uregs[4] = get_reg_asm("mov %0, r4");
		d->regs.uregs[5] = get_reg_asm("mov %0, r5");
		d->regs.uregs[6] = get_reg_asm("mov %0, r6");
		d->regs.uregs[7] = get_reg_asm("mov %0, r7");
		d->regs.uregs[8] = get_reg_asm("mov %0, r8");
		d->regs.uregs[9] = get_reg_asm("mov %0, r9");
		d->regs.uregs[10] = get_reg_asm("mov %0, r10");
		d->regs.uregs[11] = get_reg_asm("mov %0, r11");
		d->regs.uregs[12] = get_reg_asm("mov %0, r12");
		d->regs.uregs[13] = get_reg_asm("mov %0, r13");
		d->regs.uregs[14] = get_reg_asm("mov %0, r14");
		d->regs.uregs[15] = get_reg_asm("mov %0, r15");
		d->regs.uregs[16] = get_reg_asm("mrs %0, cpsr");
	}

	if (d->thread == 0)
		d->thread = current_thread_info();
	if (strlen(d->text) == 0) {
		strncat(d->text, "[KR] Panic in ", sizeof(d->text) - 1);
		if (d->thread && d->thread->task)
			strncat(d->text, d->thread->task->comm, sizeof(d->text) - 1 - strlen(d->text)); /* 0 is always appended even after n chars copied */
	}
}

/*
	ramdump_save_dynamic_context
	Saves register context (regs) into the ramdump.
*/
void ramdump_save_dynamic_context(const char *str, int err,
		struct thread_info *thread, struct pt_regs *regs)
{
	int len;
	const char *idtext = "??"; /* error type string */
	if (regs)
		ramdump_data.regs = *regs;
	if (err == RAMDUMP_ERR_EEH_CP)
		idtext = "CP";
	else if (err == RAMDUMP_ERR_EEH_AP)
		idtext = "AP";
	else if ((err & RAMDUMP_ERR_EEH_CP) == 0)
		idtext = "KR";

	len = snprintf(ramdump_data.text, sizeof(ramdump_data.text),
			"[%s] ", idtext);
	if (len >= sizeof(ramdump_data.text)) /* beware: snprintf retvalue */
		len = sizeof(ramdump_data.text) - 1;

	if (str)
		strncat(ramdump_data.text, str, sizeof(ramdump_data.text) - 1 - len); /* 0 is always appended even after n chars copied */
	ramdump_data.err = (unsigned)err;
	ramdump_data.thread = thread;
	if ((err&RAMDUMP_ERR_EEH_CP) == 0) {
		/* For kernel oops/panic add more data to text */
		char info[30];
		len = strlen(ramdump_data.text);
		snprintf(info, sizeof(info),
			" at 0x%.8x in ", (unsigned)regs->ARM_pc);

		if (thread && thread->task)
			strncat(info, thread->task->comm, sizeof(info) - 1 - strlen(info)); /* 0 is always appended even after n chars copied */

		len = sizeof(ramdump_data.text) - len;
		if (len > 0)
			strncat(ramdump_data.text, info, len - 1); /* 0 is always appended even after n chars copied */
	}
#ifdef CONFIG_PXA_MIPSRAM
	MIPS_RAM_ADD_TRACE(MIPSRAM_LOG_END_MARK_EVENT);
#endif
}
EXPORT_SYMBOL(ramdump_save_dynamic_context);

/*
	ramdump_save_isram
	Saves the ISRAM contents into the RDC.
	Caches should be flushed prior to calling this (so SRAM is in sync).
	Cacheable access to both SRAM and DDR is used for performance reasons.
	Caches are flushed also by this function to sync the DDR.
*/
static void ramdump_save_isram(void)
{
	void *rdc_isram_va = RDC_ISRAM_START(rdc_va);
	ramdump_flush_caches();
#ifdef RAMDUMP_ISRAM_CRC_CHECK
	rdc_va->header.isram_crc32 = crc32_le(0, (unsigned char *)isram_va,
		rdc_va->header.isram_size);
#else
	rdc_va->header.isram_crc32 = 0;
#endif
	memcpy(rdc_isram_va, isram_va, rdc_va->header.isram_size);
}

static void ramdump_flush_caches(void)
{
	flush_cache_all();
	outer_flush_range(0, -1ul);
}

static void ramdump_fill_rdc(void)
{
	/* Some RDC fields are already set at this point: retain these */
//#ifdef CONFIG_MACH_GFORCE
	extern unsigned int samsung_debug_mode_flag;	/*Smn-c: for RAMDUMP collection method*/
//#endif
	rdc_va->header.signature = RDC_SIGNATURE;
	rdc_va->header.kernel_build_id = 0; /* TODO */
	rdc_va->header.error_id = ramdump_data.err;
	rdc_va->header.ramdump_data_addr = __virt_to_phys((unsigned)&ramdump_data);
	rdc_va->header.isram_pa = (unsigned)RDC_ISRAM_START(rdc_pa);
	rdc_va->header.isram_size = ISRAM_SIZE;
	ramdump_data.rdc_pa = rdc_pa;
	ramdump_data.rdc_va = (unsigned)rdc_va;
//#ifdef CONFIG_MACH_GFORCE
	/* smn-c: RDC reserved area is filled with SD card status
	 * 	  0- SD card 1- USB RDX.
	 */
	if ( 4 == (samsung_debug_mode_flag & (1<<2)))
		rdc_va->header.reserved[0] = 0;     
	else
		rdc_va->header.reserved[0] = 1;      
//#endif
#ifdef CONFIG_PXA_MIPSRAM
	rdc_va->header.mipsram_pa = mipsram_desc.buffer_phys_ptr;
	rdc_va->header.mipsram_size = MIPS_RAM_BUFFER_SZ_BYTES;
#endif
}

static int sec_debug_set_debug_level(char *str)
{
        if (!str)
                return -EINVAL;
        if (strcmp(str, "HIGH") == 0)
        {   
                debug_level_param =2 ;
        }   
        else if (strcmp(str, "MID") == 0)
        {   
                debug_level_param =1 ;
        }   
        else 
        {   
                debug_level_param =0 ;
        }   
        return 0;
}
early_param("DEBUG_LEVEL", sec_debug_set_debug_level);


/************************************************************************/
/*			RAMDUMP RAMFILE support				*/
/************************************************************************/
int ramdump_attach_ramfile(struct ramfile_desc *desc)
{
	unsigned pa = rdc_va->header.ramfile_addr;
	struct ramfile_desc *va;
	unsigned *link = &rdc_va->header.ramfile_addr;
	/* Caller may supply buffers allocated with vmalloc, so VA should be used */
	unsigned use_phy = desc->flags & RAMFILE_PHYCONT;
	if (pa) {
		/* Search through the linked list */
		va = (struct ramfile_desc *)(use_phy ? __phys_to_virt(pa) : pa);
		while (va->next)
			va = (struct ramfile_desc *)(use_phy ? __phys_to_virt(va->next) : va->next);;
		link = &va->next;
	}

	/* Link in the new desc */
	*link = use_phy ? __virt_to_phys((unsigned)desc) : (unsigned)desc;
	/* Make sure the new desc is properly ending the list */
	desc->next = 0;
	return 0;
}
EXPORT_SYMBOL(ramdump_attach_ramfile);

/************************************************************************/
/*				RAMDUMP init				*/
/************************************************************************/

static int __init ramdump_init(void)
{
	extern unsigned cp_area_addr(void);
	if (!rdc_pa) /* not overriden on CMDLINE */
		rdc_pa = cp_area_addr() + RDC_OFFSET;
	isram_va = ioremap_cached(ISRAM_PA, ISRAM_SIZE);
	if (!isram_va)
		return -ENOMEM;
	rdc_va = (struct rdc_area *)ioremap_cached((unsigned)RDC_START(rdc_pa),
				sizeof(struct rdc_area)+RDC_HEADROOM);
	if (!rdc_va) {
		iounmap(isram_va);
		return -ENOMEM;
	} else
		rdc_va = RDC_HEAD(rdc_va);
	memset((void *)rdc_va, 0, sizeof(*rdc_va)); /* zero reserved fields */

	if(debug_level_param > 0)
		ramdump_fill_rdc();

	register_reboot_notifier(&nb_reboot_block);
	atomic_notifier_chain_register(&panic_notifier_list, &panic_block);
	return 0;
}
core_initcall(ramdump_init); /*TBD: option early_initcall*/
