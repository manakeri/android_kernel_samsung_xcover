/*
 *  arch/arm/plat-pxa/include/plat/pmem.h
 *
 *  Buffer Management Module
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.

 *(C) Copyright 2009 Marvell International Ltd.
 * All Rights Reserved
 */

#ifndef _PXA_PMEM_H_
#define _PXA_PMEM_H_

void pxa_add_pmem(char *name, size_t size, int no_allocator, int cached, int buffered);
int __init pxa_reserve_early_dram(char *arg);

extern size_t __initdata reserving_size;

#endif
