/*
 * linux/arch/arm/mach-mmp/include/mach/regs-ciu.h
 *
 *  CPU Interface Unit Registers
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_MACH_REGS_CIU_H
#define __ASM_MACH_REGS_CIU_H

#include <mach/addr-map.h>

#define CIU_VIRT_BASE		(AXI_VIRT_BASE + 0x82c00)
#define CIU_REG(x)		(CIU_VIRT_BASE + (x))

#define CIU_CHIP_ID		CIU_REG(0x0000)
#define CIU_CPU_CONF		CIU_REG(0x0008)
#define CIU_CPU_SRAM_SPD	CIU_REG(0x0010)
#define CIU_CPU_L2C_SRAM_SPD	CIU_REG(0x0018)
#define CIU_MCB_CONF		CIU_REG(0x001c)
#define CIU_SYS_BOOT_CNTRL	CIU_REG(0x0020)
#define CIU_SW_BRANCH_ADDR	CIU_REG(0x0024)
#define CIU_PERF_COUNT0_CNTRL	CIU_REG(0x0028)
#define CIU_PERF_COUNT1_CNTRL	CIU_REG(0x002c)
#define CIU_PERF_COUNT2_CNTRL	CIU_REG(0x0030)
#define CIU_PERF_COUNT0		CIU_REG(0x0034)
#define CIU_PERF_COUNT1		CIU_REG(0x0038)
#define CIU_PERF_COUNT2		CIU_REG(0x003c)
#define CIU_MC_CONF		CIU_REG(0x0040)
#define CIU_MCB_SRAM_SPD	CIU_REG(0x0044)
#define CIU_AXI_SRAM_SPD	CIU_REG(0x0048)

#endif /* __ASM_MACH_REGS_CIU_H */
