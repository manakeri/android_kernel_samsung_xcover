/*
 *  kernel/mipsram.c
 *
 *  Support for the Marvell MIPSRAM over PXAxxx
 *
 *  Author:	Shuki Zanyovka
 *  Created:	Feb 9, 2009
 *  Copyright:	(C) Copyright 2009 Marvell International Ltd.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  publishhed by the Free Software Foundation.
 */

#include <linux/errno.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/wait.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/serial.h>
#include <linux/uaccess.h>

#include <linux/tty_ldisc.h>
#include <linux/version.h>
#include <linux/module.h>

#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/module.h>
#include <linux/errno.h>

#include <linux/mm.h>
#include <linux/vmalloc.h>
#include <linux/mman.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/device.h>
#include <linux/moduleparam.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <asm/pgtable.h>
#ifndef CONFIG_CPU_PJ4
#include <mach/pmu.h>
#endif
#include <linux/mipsram.h>

/*#define MIPSRAM_DEBUG*/

#define DRIVER_VERSION "v1.0"
#define DRIVER_AUTHOR "MARVELL"
#define DRIVER_DESC "Pseudo tty driver for MIPSRAM"

/* Module information */
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");

__attribute__ ((aligned(PAGE_SIZE)))
	unsigned int g_MipsRam_buffer[MIPS_RAM_BUFFER_SZ];

unsigned int mips_counter_value;

struct mipsram_descriptor mipsram_desc;

struct mipsram_descriptor *MIPSRAM_get_descriptor(void)
{
	return &mipsram_desc;
}
EXPORT_SYMBOL(MIPSRAM_get_descriptor);

void MIPSRAM_Trace(unsigned long id)
{
  MIPS_RAM_ADD_TRACE(id);
}
EXPORT_SYMBOL(MIPSRAM_Trace);

void MIPSRAM_clear(void)
{
	/* Clear MIPS RAM buffer */
	if (mipsram_desc.bufferVirt)
		memset(mipsram_desc.bufferVirt, 0x00, MIPS_RAM_BUFFER_SZ_BYTES);

	mipsram_desc.current_entry = 0;
	mipsram_desc.compensation_ticks = 0;
}
EXPORT_SYMBOL(MIPSRAM_clear);

static void mipsram_alloc_buffer(void)
{
	unsigned long pfn;

	/* get physical base address of buffer */
	pfn = virt_to_phys((void *)g_MipsRam_buffer);

	/* re-map memory to gain non-cache attributes for the buffer */
	/* At this point, mipsram_desc.buffer is:
	 * 1) MMU PAGE_SIZE aligned (4KB)
	 * 2) Uncached
	 * 3) Consecutive in physical memory */
	mipsram_desc.buffer = ioremap_nocache(pfn, MIPS_RAM_BUFFER_SZ_BYTES);
	mipsram_desc.bufferVirt = mipsram_desc.buffer;
	mipsram_desc.buffer_phys_ptr = pfn;
	mipsram_desc.current_pp_msk = 0;
}

static void mipsram_cycle_counter_init(void)
{
	unsigned int PMNC_val;

	/* read/modify/write for the PMNC register - start CP14
	 * Performance Monitor counter */
#ifndef CONFIG_CPU_PJ4
	PMNC_val = pmu_read_reg(PMU_PMNC);
	pmu_write_reg(PMU_PMNC, PMNC_val | PMU_COUNTERS_ENABLE);
#else
	/* Bit#0: enables all counters */
	PMNC_val = 0x1;
	asm volatile ("mcr p15, 0, %0, c9, c12, 0" : : "r" (PMNC_val));

	/* 1. Bit#31: Enable Cycle Counter */
	/* 2. Bit#0: Enable counter */
	PMNC_val = 0x80000000;
	asm volatile ("mcr p15, 0, %0, c9, c12, 1" : : "r" (PMNC_val));
#endif
}

static int __init
mipsram_init(void)
{
	/* Allocate buffer that is:
	 * 1) PAGE aligned
	 * 2) Uncacheable
	 * 3) Consecutive in physical memory */
	mipsram_alloc_buffer();

	/* initialize MIPSRAM counter */
	mipsram_cycle_counter_init();

    return 0;
}

/* the two functions mipsram_reinit_counter and mipsram_disable_counter
must be called in the same sequence while interrupts are disabled. */
void mipsram_reinit_counter()
{
#ifdef CONFIG_CPU_PJ4
	/* the counter is restarted and the last value read is
	componsated in mips ram internal calculations */
	unsigned int counter;

	/* re-initialize MIPSRAM counter */
	mipsram_cycle_counter_init();

	counter = mips_counter_value;
	mips_counter_value = 0;
	MIPS_RAM_ADD_TIMESTAMP_COMPENSATION(counter);
#endif
}

void mipsram_disable_counter()
{
#ifdef CONFIG_CPU_PJ4
	/* we must stop the counter before entering LPM
	and save the counter value */
	unsigned int counter = 0;
	unsigned int PMNC_val;
	asm volatile ("mrc p15, 0, %0, c9, c13, 0" : : "r" (counter));
	PMNC_val = 0x00000000;
	asm volatile ("mcr p15, 0, %0, c9, c12, 1" : : "r" (PMNC_val));
	/* the arm macros must use a local variable,
		so it is copied to a global */
	mips_counter_value = counter;
#endif
}

static void __exit
mipsram_exit(void)
{
	/* First, stop logging, to prevent system
	 * crash during reboot */
	mipsram_desc.buffer = NULL;

	/* free re-map */
	iounmap(mipsram_desc.bufferVirt);
	mipsram_desc.bufferVirt = NULL;
}

subsys_initcall(mipsram_init);
