/*
 *  arch/arm/plat-pxa/pmem.c
 *
 *  Buffer Management Module
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.

 *(C) Copyright 2009 Marvell International Ltd.
 * All Rights Reserved
 */

//#define DEBUG
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/android_pmem.h>
#include <linux/bootmem.h>

#ifdef CONFIG_ANDROID_PMEM
#include <plat/pmem.h>

size_t __initdata reserving_size = 0x1000000;/*default reserve size:16MB*/

/* Note that we have to call it at where is located
 * between bootmem_init() and mm_init().
 */
static unsigned long __initdata pmem_reserve_pa = 0;
int __init pxa_reserve_early_dram(char *arg)
{
	void *va_reserve;

	reserving_size = memparse(arg, NULL);
	va_reserve= alloc_bootmem_pages(reserving_size);
	if (WARN_ON(!va_reserve))
		return -ENOMEM;
	pmem_reserve_pa = virt_to_phys(va_reserve);
	return 0;
}
__setup("reserve_pmem=", pxa_reserve_early_dram);

void __init pxa_add_pmem(char *name, size_t size, int no_allocator, int cached, int buffered)
{
	struct platform_device *android_pmem_device;
	struct android_pmem_platform_data *android_pmem_pdata;
	static int id;

	if (size > PAGE_SIZE && size > reserving_size)
		return;

	if (pmem_reserve_pa == 0)
		return;

	android_pmem_device = kzalloc(sizeof(struct platform_device), GFP_KERNEL);
	if (android_pmem_device == NULL)
		return ;

	android_pmem_pdata = kzalloc(sizeof(struct android_pmem_platform_data), GFP_KERNEL);
	if (android_pmem_pdata == NULL) {
		kfree(android_pmem_device);
		return ;
	}

	if (size > PAGE_SIZE) {
		android_pmem_pdata->start = pmem_reserve_pa;
		android_pmem_pdata->size = size;
		pmem_reserve_pa += size;
		reserving_size -= size;
	} else {
		android_pmem_pdata->start = size;
		android_pmem_pdata->size = 0;
	}

	android_pmem_pdata->name = name;
	android_pmem_pdata->no_allocator = no_allocator;
	android_pmem_pdata->cached = cached;
	android_pmem_pdata->buffered = buffered;

	android_pmem_device->name = "android_pmem";
	android_pmem_device->id = id++;
	android_pmem_device->dev.platform_data = android_pmem_pdata;

	platform_device_register(android_pmem_device);
	printk(KERN_INFO "pmem register %s reserve pa(0x%lx), request size=0x%x\n", name, pmem_reserve_pa, size);
}

#endif
