/*
 *  arch/arm/mach-pxa/include/mach/memory.h
 *
 * Author:	Nicolas Pitre
 * Copyright:	(C) 2001 MontaVista Software Inc.
 * Copyright:	(C) 2010 Marvell Semiconductor Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_ARCH_MEMORY_H
#define __ASM_ARCH_MEMORY_H

/* Override the ARM default */
/* Must be multiple of 2MB in [2MB..14MB] */
#define CONSISTENT_DMA_SIZE	(12 * 1024 * 1024)

/*
 * Physical DRAM offset.
 */
#define PHYS_OFFSET	UL(0x80000000)

#if !defined(__ASSEMBLY__) && defined(CONFIG_MACH_ARMCORE) && defined(CONFIG_PCI)
void cmx2xx_pci_adjust_zones(int node, unsigned long *size,
			     unsigned long *holes);

#define arch_adjust_zones(node, size, holes) \
	cmx2xx_pci_adjust_zones(node, size, holes)

#define ISA_DMA_THRESHOLD	(PHYS_OFFSET + SZ_64M - 1)
#define MAX_DMA_ADDRESS		(PAGE_OFFSET + SZ_64M)
#endif


#ifdef CONFIG_SPARSEMEM

#if defined(CONFIG_MACH_SAARB) || defined(CONFIG_MACH_SAARB_MG1) || defined(CONFIG_MACH_NEVOEVB3) || defined(CONFIG_MACH_GFORCE) || defined(CONFIG_MACH_ALKON) || defined(CONFIG_MACH_JETTA)
/*
 * There're two DRAM banks in PXA955 Saarb board.
 *
 * The macros below define sections with 256MB size and a non-linear virtual to
 * physical mapping:
 *
 * node 0: 0x80000000-0x8fffffff -> 0xc0000000-0xcfffffff
 * node 1: 0xc0000000-0xcfffffff -> 0xd0000000-0xdfffffff
 *
 * Since DRAM can be wrapped, 0xa0000000 equals to 0x80000000.
 */
#define MAX_PHYSMEM_BITS	32
#define SECTION_SIZE_BITS	28

/* bank page offsets */
#define BANK_PAGE_OFFSET	(PAGE_OFFSET + 0x10000000)

#define __phys_to_virt(phys)						\
	((phys) >= 0xc0000000 ? (phys) - 0xc0000000 + BANK_PAGE_OFFSET :\
	 (phys) >= 0xa0000000 ? (phys) - 0xa0000000 + PAGE_OFFSET :	\
	 (phys) - 0x80000000 + PAGE_OFFSET)				\

#define __virt_to_phys(virt)						\
	((virt) >= BANK_PAGE_OFFSET ? (virt) - BANK_PAGE_OFFSET + 0xc0000000 :\
	 (virt) - PAGE_OFFSET + PHYS_OFFSET)
#endif	/* CONFIG_MACH_SAARB || CONFIG_MACH_SAARB_MG1 */

#endif	/* CONFIG_SPARSEMEM */

#endif	/* __ASM_ARCH_MEMORY_H */
