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
#include <linux/crc32.h>
#include <linux/bootmem.h>

#include <asm/system.h>
#include <asm/ptrace.h> /*pt_regs*/
#include <asm/cacheflush.h>

#define RAMDUMP_FOR_TD
#include <mach/ramdump.h>
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

/* PJ4 specific cp15 regs. DONT EXTEND, ADD NEW STRUCTURE TO ramdump_state */
struct cp15_regs_pj1 {
	unsigned cntcfg0;	/* counter operation 0 */
	unsigned cntcfg1;	/* counter operation 1 */
	unsigned cntcfg2;	/* counter operation 2 */
	unsigned cntcfg3;	/* counter operation 3 */
	unsigned count0;	/* coutner 0 */
	unsigned count1;	/* coutner 1 */
	unsigned count2;	/* coutner 2 */
	unsigned count3;	/* coutner 3 */
	unsigned count4;	/* coutner 4 */
	unsigned count5;	/* coutner 5 */
	unsigned count6;	/* coutner 6 */
	unsigned count7;	/* coutner 7 */
	unsigned perfint_en;	/* performance interrupt enable */
	unsigned overflow_st;	/* overflow status flag */
	unsigned cr_c;		/* control configuration */
	unsigned pri_access;	/* privilledge access */
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
	struct {			/* 16 total */
		struct cp15_regs_pj1 cp15pj1;
	}; /* PJ4 */
	struct acc_regs acc;
	struct l2c_pj4_regs l2cpj4;
	struct pfm_pj4_regs pfmpj4;
} ramdump_data;

static void *isram_va; /* ioremapped va for ISRAM access */
static struct rdc_area *rdc_va;/* ioremapped va for rdc_area access */
/************************************************************************/
/*				Internal prototypes			*/
/************************************************************************/
static void ramdump_save_static_context(struct ramdump_state *d);
static void ramdump_save_isram(void);
static void ramdump_flush_caches(void);
static void ramdump_fill_rdc(void);

/************************************************************************/
/*				RDC address setup			*/
/************************************************************************/
static unsigned rdc_pa = 0;

static int __init ramdump_rdc_setup(char *str)
{
	rdc_pa = simple_strtoul(str, NULL, 16);
	BUG_ON(reserve_bootmem(rdc_pa, RDC_SIZE, BOOTMEM_EXCLUSIVE) != 0);
	printk("Reserved RDCA+obm+uboot part  %dM at %.8x\n",
			(unsigned)RDC_SIZE/0x100000, (unsigned)rdc_pa);
	return 1;
}
__setup("RDCA=", ramdump_rdc_setup);

/*
 * ramdump_status: 0: ramdump is off
 * 		   1: ramdump is on
 * flag: >0 set ramdump_status to 1
 * 	 <0 set ramdump_status to 0
 * 	 =0 inquire the ramdump_status
 */
static ramdump_status = 1;
int ramdump_open(int flag)
{
	if (flag > 0)
		ramdump_status = 1;
	else if (flag < 0)
		ramdump_status = 0;
	return ramdump_status;
}
EXPORT_SYMBOL(ramdump_open);

/************************************************************************/
/*				RAMDUMP panic notifier			*/
/************************************************************************/
static int
ramdump_panic(struct notifier_block *this, unsigned long event, void *ptr)
{
	if (ramdump_status) {
		printk(KERN_ERR "RAMDUMP STARTED\n");
		ramdump_fill_rdc();
		ramdump_save_static_context(&ramdump_data);
		ramdump_save_isram();
		ramdump_flush_caches();
		printk(KERN_ERR "RAMDUMP DONE\n");
	}
	return NOTIFY_DONE;
}

static struct notifier_block panic_block = {
	.notifier_call = ramdump_panic,
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
	d->cp15.id 	= 	get_reg_asm("mrc p15, 0, %0, c0, c0, 0");
	d->cp15.cr 	= 	get_reg_asm("mrc p15, 0, %0, c1, c0, 0");
	d->cp15.ttb	=	get_reg_asm("mrc p15, 0, %0, c2, c0, 0");
	d->cp15.da_ctrl	=	get_reg_asm("mrc p15, 0, %0, c3, c0, 0");
	d->cp15.cpar	=	get_reg_asm("mrc p15, 0, %0, c15, c1, 0");
	d->cp15.fsr	=	get_reg_asm("mrc p15, 0, %0, c5, c0, 0");
	d->cp15.far	=	get_reg_asm("mrc p15, 0, %0, c6, c0, 0");
	d->cp15.procid	=	get_reg_asm("mrc p15, 0, %0, c13,c0, 0"); /* PJ4: context id */

	d->cp15pj1.cntcfg0	= get_reg_asm("mrc p15, 0, %0, c15, c12, 0");
	d->cp15pj1.cntcfg1	= get_reg_asm("mrc p15, 0, %0, c15, c12, 1");
	d->cp15pj1.cntcfg2	= get_reg_asm("mrc p15, 0, %0, c15, c12, 2");
	d->cp15pj1.cntcfg3	= get_reg_asm("mrc p15, 0, %0, c15, c12, 3");
	d->cp15pj1.count0	= get_reg_asm("mrc p15, 0, %0, c15, c13, 0");
	d->cp15pj1.count1	= get_reg_asm("mrc p15, 0, %0, c15, c13, 1");
	d->cp15pj1.count2	= get_reg_asm("mrc p15, 0, %0, c15, c13, 2");
	d->cp15pj1.count3	= get_reg_asm("mrc p15, 0, %0, c15, c13, 3");
	d->cp15pj1.count4	= get_reg_asm("mrc p15, 0, %0, c15, c13, 4");
	d->cp15pj1.count5	= get_reg_asm("mrc p15, 0, %0, c15, c13, 5");
	d->cp15pj1.count6	= get_reg_asm("mrc p15, 0, %0, c15, c13, 6");
	d->cp15pj1.count7	= get_reg_asm("mrc p15, 0, %0, c15, c13, 7");
	d->cp15pj1.perfint_en	= get_reg_asm("mrc p15, 0, %0, c15, c14, 0");
	d->cp15pj1.overflow_st	= get_reg_asm("mrc p15, 0, %0, c15, c14, 1");
	d->cp15pj1.cr_c		= get_reg_asm("mrc p15, 1, %0, c15, c1, 1");
	d->cp15pj1.pri_access	= get_reg_asm("mrc p15, 0, %0, c15, c9, 0");

	d->l2cpj4.l2errcnt      = get_reg_asm("mrc p15, 1, %0, c15, c9, 6");
	d->l2cpj4.l2errth       = get_reg_asm("mrc p15, 1, %0, c15, c9, 7");
	d->l2cpj4.l2errcapt     = get_reg_asm("mrc p15, 1, %0, c15, c11, 7");

}

/*
	ramdump_save_dynamic_context
	Saves register context (regs) into the ramdump.
*/
void ramdump_save_dynamic_context(const char *str, int err,
		struct thread_info *thread, struct pt_regs *regs)
{
	if (regs)
		ramdump_data.regs = *regs;
	if (str)
		strncpy(ramdump_data.text, str, (sizeof(ramdump_data.text) - 1));
	ramdump_data.err = (unsigned)err;
	ramdump_data.thread = thread;
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

#define CACHE_WAY_PER_SET	8
#define CACHE_LINE_SIZE		32
#define CACHE_LINE_SHIFT	5
#define CACHE_WAY_SIZE(l2ctype)	(8192 << (((l2ctype) >> 8) & 0xf))
#define CACHE_SET_SIZE(l2ctype)	(CACHE_WAY_SIZE(l2ctype) >> CACHE_LINE_SHIFT)

static void ramdump_flush_caches(void)
{
	unsigned long l2ctype, set_way;
	int set, way;
	flush_cache_all();
	/* Clean L2 cache */
	__asm__("mrc p15, 1, %0, c0, c0, 1" : "=r" (l2ctype));

	for (set = 0; set < CACHE_SET_SIZE(l2ctype); set++) {
		for (way = 0; way < CACHE_WAY_PER_SET; way++) {
			set_way = (way << 29) | (set << 5);
			__asm__("mcr p15, 1, %0, c7, c15, 2" : : "r"(set_way));
		}
	}
}

static void ramdump_fill_rdc(void)
{
	/* Some RDC fields are already set at this point: retain these */
	rdc_va->header.signature = RDC_SIGNATURE;
	rdc_va->header.kernel_build_id = 0; /* TODO */
	rdc_va->header.error_id = ramdump_data.err;
	rdc_va->header.ramdump_data_addr = __virt_to_phys((unsigned)&ramdump_data);
	rdc_va->header.isram_pa = (unsigned)RDC_ISRAM_START(rdc_pa);
	rdc_va->header.isram_size = ISRAM_SIZE;
	ramdump_data.rdc_pa = rdc_pa;
	ramdump_data.rdc_va = (unsigned)rdc_va;
#ifdef CONFIG_PXA_MIPSRAM
	rdc_va->header.mipsram_pa = mipsram_desc.buffer_phys_ptr;
	rdc_va->header.mipsram_size = MIPS_RAM_BUFFER_SZ_BYTES;
#endif
}

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
	/* we don't enable ramdump when the RDCA is not passed */
	if (!rdc_pa)
		return 0;
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
	atomic_notifier_chain_register(&panic_notifier_list, &panic_block);
	return 0;
}
core_initcall(ramdump_init); /*TBD: option early_initcall*/
