/*
 * linux/arch/arm/mach-mmp/include/mach/addr-map.h
 *
 *   Common address map definitions
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_MACH_ADDR_MAP_H
#define __ASM_MACH_ADDR_MAP_H

/* APB - Application Subsystem Peripheral Bus
 *
 * NOTE: the DMA controller registers are actually on the AXI fabric #1
 * slave port to AHB/APB bridge, due to its close relationship to those
 * peripherals on APB, let's count it into the ABP mapping area.
 */
#define APB_PHYS_BASE		0xd4000000
#define APB_VIRT_BASE		0xfe000000
#define APB_PHYS_SIZE		0x00200000

#define AXI_PHYS_BASE		0xd4200000
#define AXI_VIRT_BASE		0xfe200000
#define AXI_PHYS_SIZE		0x00200000

#define BOOTROM_PHYS_BASE	0xffe00000
#define BOOTROM_VIRT_BASE	0xfe600000
#define BOOTROM_PHYS_SIZE	0x00200000

#ifdef CONFIG_CPU_MMP3
#define PGU_PHYS_BASE		0xe0000000
#define PGU_VIRT_BASE		0xfe400000
#define PGU_PHYS_SIZE		0x00100000

#define SCU_VIRT_BASE		(PGU_VIRT_BASE)
#define GIC_DIST_VIRT_BASE	(PGU_VIRT_BASE + 0x1000)
#define GIC_CPU_VIRT_BASE	(PGU_VIRT_BASE + 0x0100)
#define TWD_VIRT_BASE		(PGU_VIRT_BASE + 0x600)

#define SL2C_PHYS_BASE		0xd0020000
#define SL2C_VIRT_BASE		0xfe800000
#define SL2C_PHYS_SIZE		SZ_8K

#define AUD_PHYS_BASE		0xc0ffd000
#define AUD_VIRT_BASE		0xfeffd000
#define AUD_PHYS_SIZE		0x00003000
#define AUD_PHYS_BASE2		0xc0140000
#define AUD_VIRT_BASE2		0xfef40000
#define AUD_PHYS_SIZE2		0x00010000
#endif

#define BOOTROM_PHYS_BASE       0xffe00000
#define BOOTROM_VIRT_BASE       0xfe600000
#define BOOTROM_PHYS_SIZE       0x00200000

/* Static Memory Controller - Chip Select 0 and 1 */
#define SMC_CS0_PHYS_BASE	0x80000000
#define SMC_CS0_PHYS_SIZE	0x10000000
#define SMC_CS1_PHYS_BASE	0x90000000
#define SMC_CS1_PHYS_SIZE	0x10000000

#endif /* __ASM_MACH_ADDR_MAP_H */
