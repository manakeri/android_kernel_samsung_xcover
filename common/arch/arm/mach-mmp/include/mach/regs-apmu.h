/*
 * linux/arch/arm/mach-mmp/include/mach/regs-apmu.h
 *
 *   Application Subsystem Power Management Unit
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_MACH_REGS_APMU_H
#define __ASM_MACH_REGS_APMU_H

#include <mach/addr-map.h>

#define APMU_VIRT_BASE	(AXI_VIRT_BASE + 0x82800)
#define APMU_REG(x)	(APMU_VIRT_BASE + (x))

/* Clock Reset Control */
#define APMU_IRE	APMU_REG(0x048)
#define APMU_LCD	APMU_REG(0x04c)
#define APMU_CCIC_RST	APMU_REG(0x050)
#define APMU_SDH0	APMU_REG(0x054)
#define APMU_SDH1	APMU_REG(0x058)
#define APMU_USB	APMU_REG(0x05c)
#define APMU_NAND	APMU_REG(0x060)
#define APMU_DMA	APMU_REG(0x064)
#define APMU_GEU	APMU_REG(0x068)
#define APMU_BUS	APMU_REG(0x06c)
#define APMU_GC	APMU_REG(0x0cc)
#define APMU_GC_PD	APMU_REG(0x0d0)
#define APMU_SDH2	APMU_REG(0x0e8)
#define APMU_SDH3	APMU_REG(0x0ec)
#define APMU_CCIC_GATE APMU_REG(0x028)
#define APMU_CCIC_DBG	APMU_REG(0x088)
#define APMU_VMETA	APMU_REG(0x0A4)
#define APMU_SMC	APMU_REG(0x0d4)

#define APMU_CP_CCR             APMU_REG(0x0000)
#define APMU_CCR                APMU_REG(0x0004)
#define APMU_CP_CCSR            APMU_REG(0x0008)
#define APMU_CCSR               APMU_REG(0x000c)
#define APMU_IDLE_CFG           APMU_REG(0x0018)
#define APMU_SQU_CLK_GATE_CTRL	APMU_REG(0x001c)
#define APMU_LCD_CLK_RES_CTRL   APMU_REG(0x004c)
#define APMU_DEBUG              APMU_REG(0x0088)
#define APMU_IMR                APMU_REG(0x0098)
#define APMU_ISR                APMU_REG(0x00a0)
#define APMU_DX8_CLK_RES_CTRL   APMU_REG(0x00a4)
#define APMU_MC_HW_SLP_TYPE     APMU_REG(0x00b0)
#define APMU_PLL_SEL_STATUS     APMU_REG(0x00c4)
#define APMU_SMC_CLK_RES_CTRL   APMU_REG(0x00d4)
#define APMU_PWR_CTRL_REG		APMU_REG(0x00d8)
#define APMU_GC_CLK_RES_CTRL	APMU_REG(0x00cc)
#define APMU_PWR_BLK_TMR_REG	APMU_REG(0x00dc)
#define APMU_PWR_STATUS_REG     APMU_REG(0x00f0)

#define APMU_FSIC3_CLK_RES_CTRL APMU_REG(0x0100)

#define APMU_FNCLK_EN	(1 << 4)
#define APMU_AXICLK_EN	(1 << 3)
#define APMU_FNRST_DIS	(1 << 1)
#define APMU_AXIRST_DIS	(1 << 0)

/* Wake Clear Register */
#define APMU_WAKE_CLR	APMU_REG(0x07c)

#define APMU_PXA168_KP_WAKE_CLR		(1 << 7)
#define APMU_PXA168_CFI_WAKE_CLR	(1 << 6)
#define APMU_PXA168_XD_WAKE_CLR		(1 << 5)
#define APMU_PXA168_MSP_WAKE_CLR	(1 << 4)
#define APMU_PXA168_SD4_WAKE_CLR	(1 << 3)
#define APMU_PXA168_SD3_WAKE_CLR	(1 << 2)
#define APMU_PXA168_SD2_WAKE_CLR	(1 << 1)
#define APMU_PXA168_SD1_WAKE_CLR	(1 << 0)

#define APMU_PXA910_KP_WAKE_CLR		(1 << 3)

#define APMU_GC_156M		0x0
#define APMU_GC_312M		0x40
#define APMU_GC_PLL2		0x80
#define APMU_GC_PLL2_DIV2	0xc0
#define APMU_GC_624M		0xc0 /* added according to Aspen SW spec v2.8*/

#define APMU_VMETA_CLK_RES_CTRL	APMU_VMETA
/* VMeta Technology Power Mode */
#define APMU_VMETA_CLK_RES_CTRL_VMETA_PWR_CTRL	(1 << 11)
/* VMeta Technology Power Up */
#define APMU_VMETA_CLK_RES_CTRL_VMETA_PWR_ON	(3 << 9)
#define APMU_VMETA_CLK_RES_CTRL_VMETA_PWR_SLOW_RAMP_UP	(1 << 9)
/* VMeta Technology Isolation Enable */
#define APMU_VMETA_CLK_RES_CTRL_VMETA_ISB	(1 << 8)
/* VMeta Technology Clock Select */
#define APMU_VMETA_CLK_RES_CTRL_VMETA_CLK_SEL	(1 << 6)
/* Bit(s) PMUA_VMETA_CLK_RES_CTRL_RSRV_5 reserved */
/* VMeta Technology Peripheral Clock Enable */
#define APMU_VMETA_CLK_RES_CTRL_VMETA_CLK_EN	(1 << 4)
/* VMeta Technology AXI Clock Enable */
#define APMU_VMETA_CLK_RES_CTRL_VMETA_AXICLK_EN	(1 << 3)
/* Bit(s) PMUA_VMETA_CLK_RES_CTRL_RSRV_2 reserved */
/* VMeta Technology Peripheral Reset 1 */
#define APMU_VMETA_CLK_RES_CTRL_VMETA_RST1	(1 << 1)
/* VMeta Technology AXI Reset */
#define APMU_VMETA_CLK_RES_CTRL_VMETA_AXI_RST	(1 << 0)

/* USB HSIC/FSIC*/
#define APMU_USBHSIC1	APMU_REG(0x0f8)
#define APMU_USBHSIC2	APMU_REG(0x0fc)
#define APMU_USBFSIC	APMU_REG(0x100)

#endif /* __ASM_MACH_REGS_APMU_H */
