/*
 * linux/arch/arm/mach-mmp/include/mach/regs-pmu.h
 *
 *   Power Management Unit
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_MACH_REGS_PMU_H
#define __ASM_MACH_REGS_PMU_H

#include <mach/addr-map.h>

#define PMUM_VIRT_BASE		(APB_VIRT_BASE + 0x50000)
#define PMUM_REG(x)		(PMUM_VIRT_BASE + (x))

#define MMP2_PMUM_PCR_SP	PMUM_REG(0x0000)	/* Secure Processor Power Control Register */
#define MMP2_PMUM_PSR_SP	PMUM_REG(0x0004)	/* Secure Processor Power Status Register */
#define MMP2_PMUM_FCCR		PMUM_REG(0x0008)	/* Frequency Change Control Register */
#define MMP2_PMUM_POCR		PMUM_REG(0x000c)	/* PLL & Oscillator Control Register */
#define MMP2_PMUM_POSR		PMUM_REG(0x0010)	/* PLL & Oscillator Status Register */
#define MMP2_PMUM_SUCCR		PMUM_REG(0x0014)	/* UART Clock Generation Control Register */
#define MMP2_PMUM_VRCR		PMUM_REG(0x0018)	/* VCXO Request Control Register */
#define MMP2_PMUM_PRR_SP	PMUM_REG(0x0020)	/* Secure Processor Programmable Reset Register */
#define MMP2_PMUM_CGR_SP	PMUM_REG(0x0024)	/* Secure Processor Clock Gating Register */
#define MMP2_PMUM_RSR_SP	PMUM_REG(0x0028)	/* Secure Processor Reset Status Register */
#define MMP2_PMUM_GPCR		PMUM_REG(0x0030)	/* General Purpose Clock Generation Control Register */
#define MMP2_PMUM_PLL2CR	PMUM_REG(0x0034)	/* PLL2 Control Register */
#define MMP2_PMUM_SCCR		PMUM_REG(0x0038)	/* Slow Clock Control Register */
#define MMP2_PMUM_ISCCR1	PMUM_REG(0x0040)	/* I2S Clock Generation Control Register 1 */
#define MMP2_PMUM_ISCCR2	PMUM_REG(0x0044)	/* I2S Clock Generation Control Register 2 */
#define MMP2_PMUM_WUCRS_SP	PMUM_REG(0x0048)	/* Secure Processor Wakeup & Clock Resume Lines Status Register */
#define MMP2_PMUM_WUCRM_SP	PMUM_REG(0x004c)	/* Secure Processor Wakeup & Clock Resume Lines Mask Register */
#define MMP2_PMUM_WDTPCR	PMUM_REG(0x0200)	/* WDT Control Register */
#define MMP2_PMUM_PLL2_CTRL1	PMUM_REG(0x0414)	/* PLL2 Control Register 1 */
#define MMP2_PMUM_SRAM_PWRDWN	PMUM_REG(0x0420)	/* SRAM Power Down Control Register */
#define MMP2_PMUM_PCR_PJ	PMUM_REG(0x1000)	/* PJ4 Power Control Register */
#define MMP2_PMUM_PSR_PJ	PMUM_REG(0x1004)	/* PJ4 Power Status Register */
#define MMP2_PMUM_PRR_PJ	PMUM_REG(0x1020)	/* PJ4 Programmable Reset Register */
#define MMP2_PMUM_CGR_PJ	PMUM_REG(0x1024)	/* PJ4 Clock Gating Register */
#define MMP2_PMUM_RSR_PJ	PMUM_REG(0x1028)	/* PJ4 Reset Status Register */
#define MMP2_PMUM_WUCRS_PJ	PMUM_REG(0x1048)	/* PJ4 Wakeup & Clock Resume Lines Status Register */
#define MMP2_PMUM_WUCRM_PJ	PMUM_REG(0x104c)	/* PJ4 Wakeup & Clock Resume Lines Mask Register */


/* bit definition of MMP2_PMUM_PCR_PJ */
#define MMP2_PCR_SLPWP7		(1 << 15)
#define MMP2_PCR_SLPWP6		(1 << 16)
#define MMP2_PCR_SLPWP5		(1 << 17)
#define MMP2_PCR_SLPWP4		(1 << 18)
#define MMP2_PCR_VCXOSD		(1 << 19)
#define MMP2_PCR_SLPWP3		(1 << 20)
#define MMP2_PCR_SLPWP2		(1 << 21)
#define MMP2_PCR_SLPWP1		(1 << 22)
#define MMP2_PCR_SLPWP0		(1 << 23)
#define MMP2_PCR_INTCLR		(1 << 24)
#define MMP2_PCR_APBSD		(1 << 26)
#define MMP2_PCR_DDRCORSD	(1 << 27)
#define MMP2_PCR_SPSD		(1 << 28)
#define MMP2_PCR_SLPEN		(1 << 29)
#define MMP2_PCR_AXISD		(1 << 31)
#define MMP2_PCR_SLPWPALL	(MMP2_PCR_SLPWP7 | MMP2_PCR_SLPWP6	\
				| MMP2_PCR_SLPWP5 | MMP2_PCR_SLPWP4	\
				| MMP2_PCR_SLPWP3 | MMP2_PCR_SLPWP2	\
				| MMP2_PCR_SLPWP1 | MMP2_PCR_SLPWP0)

/* bit definition of MMP2_PMUM_WUCRS_PJ */
#define MMP2_WAKEUP0		(1 << 0)	/* MIPI */
#define MMP2_WAKEUP1		(1 << 1)	/* Audio island */
#define MMP2_WAKEUP2		(1 << 2)	/* GPIO */
#define MMP2_WAKEUP3		(1 << 3)	/* Key, Trackball, Rotary */
#define MMP2_WAKEUP4		(1 << 4)	/* Timer, RTC, WDT */
#define MMP2_WAKEUP5		(1 << 5)	/* USB PHY */
#define MMP2_WAKEUP6		(1 << 6)	/* SDH, MSP_INS */
#define MMP2_WAKEUP7		(1 << 7)	/* PMIC */
#define MMP2_WAKEUP_TIMER1G1	(1 << 8)
#define MMP2_WAKEUP_TIMER1G2	(1 << 9)
#define MMP2_WAKEUP_TIMER1G3	(1 << 10)
#define MMP2_WAKEUP_MSP		(1 << 11)
#define MMP2_WAKEUP_WDT1	(1 << 13)
#define MMP2_WAKEUP_TIMER2G1	(1 << 14)
#define MMP2_WAKEUP_TIMER2G2	(1 << 15)
#define MMP2_WAKEUP_TIMER2G3	(1 << 16)
#define MMP2_WAKEUP_RTC		(1 << 17)
#define MMP2_WAKEUP_WDT2	(1 << 18)
#define MMP2_WAKEUP_ROTARY	(1 << 19)
#define MMP2_WAKEUP_TBALL	(1 << 20)
#define MMP2_WAKEUP_KPC		(1 << 21)
#define MMP2_WAKEUP_SDH3	(1 << 22)
#define MMP2_WAKEUP_SDH1	(1 << 23)
#define MMP2_WAKEUP_FULL_IDLE	(1 << 24)
#define MMP2_WAKEUP_ICU_ASYNC	(1 << 25)
#define MMP2_WAKEUP_SSP1	(1 << 26)
#define MMP2_WAKEUP_CAWAKE	(1 << 27)
#define MMP2_WAKEUP_SSP3	(1 << 29)
#define MMP2_WAKEUP_ALL		(1 << 30)

#define PMUA_VIRT_BASE		(AXI_VIRT_BASE + 0x82800)
#define PMUA_REG(x)		(PMUA_VIRT_BASE + (x))

#define MMP2_PMUA_CC_SP		PMUA_REG(0x0000)	/* Secure Processor Clock Control Register */
#define MMP2_PMUA_CC_PJ		PMUA_REG(0x0004)	/* PJ4 Clock Control Register */
#define MMP2_PMUA_DD_CC_SP	PMUA_REG(0x0008)	/* Secure Processor Clock Control Status Register */
#define MMP2_PMUA_DD_CC_PJ	PMUA_REG(0x000c)	/* PJ4 Clock Control Status Register */
#define MMP2_PMUA_FC_TIMER	PMUA_REG(0x0010)	/* Frequency Change Timer Register */
#define MMP2_PMUA_SP_IDLE_CFG	PMUA_REG(0x0014)	/* Secure Processor Idle Configuration Register */
#define MMP2_PMUA_PJ_IDLE_CFG	PMUA_REG(0x0018)	/* PJ4 Idle Configuration Register */
#define MMP2_PMUA_CCIC_GATE	PMUA_REG(0x0028)	/* CCIC Dynamic Clock Gate Control Register */
#define MMP2_PMUA_PMU_GATE	PMUA_REG(0x0040)	/* PMU Dynamic Clock Gate Control Register */
#define MMP2_PMUA_IRE_RESET	PMUA_REG(0x0048)	/* IRE Clock/Reset Control Register */
#define MMP2_PMUA_DIS1_RESET	PMUA_REG(0x004c)	/* Display Controller 1 Clock/Reset Control Register */
#define MMP2_PMUA_CCIC_RESET	PMUA_REG(0x0050)	/* CCIC Clock/Reset Control Register */
#define MMP2_PMUA_SDH0_RESET	PMUA_REG(0x0054)	/* SDIO Host 0 Clock/Reset Control Register */
#define MMP2_PMUA_SDH1_RESET	PMUA_REG(0x0058)	/* SDIO Host 1 Clock/Reset Control Register */
#define MMP2_PMUA_USB_RESET	PMUA_REG(0x005c)	/* USB Clock/Reset Control Register */
#define MMP2_PMUA_NAND_RESET	PMUA_REG(0x0060)	/* NAND Clock/Reset Control Register */
#define MMP2_PMUA_DMA_RESET	PMUA_REG(0x0064)	/* DMA Clock/Reset Control Register */
#define MMP2_PMUA_WTM_RESET	PMUA_REG(0x0068)	/* WTM Clock/Reset Control Register */
#define MMP2_PMUA_BUS_RESET	PMUA_REG(0x006c)	/* Bus Clock/Reset Control Register */
#define MMP2_PMUA_WAKE_CLR	PMUA_REG(0x007c)	/* Wake Clear/Mask Register */
#define MMP2_PMUA_STBL_TIMER	PMUA_REG(0x0084)	/* Power Stable Timer Register */
#define MMP2_PMUA_SRAM_DWN	PMUA_REG(0x008c)	/* Core SRAM Power Down Register */
#define MMP2_PMUA_CORE_STATUS	PMUA_REG(0x0090)	/* Core Status Register */
#define MMP2_PMUA_RES_CLR	PMUA_REG(0x0094)	/* Resume from Sleep Clear Register */
#define MMP2_PMUA_PJ_IMR	PMUA_REG(0x0098)	/* PMU PJ4 Interrupt Mask Register */
#define MMP2_PMUA_PJ_IRWC	PMUA_REG(0x009c)	/* PMU PJ4 Interrupt READ/WRITE Clear Register */
#define MMP2_PMUA_PJ_ISR	PMUA_REG(0x00a0)	/* PMU PJ4 Interrupt Status Register */
#define MMP2_PMUA_VMETA_RES	PMUA_REG(0x00a4)	/* VMeta Clock/Reset Control Register */
#define MMP2_PMUA_MC_HW_SLP	PMUA_REG(0x00b0)	/* Memory Controller Hardware Sleep Type Register */
#define MMP2_PMUA_MC_SLP_REQ	PMUA_REG(0x00b4)	/* Memory Controller PJ4 Sleep Request Register */
#define MMP2_PMUA_MC_SW_SLP	PMUA_REG(0x00c0)	/* Memory Controller Software Sleep Type Register */
#define MMP2_PMUA_PLL_SEL	PMUA_REG(0x00c4)	/* PLL Clock Select Status Register */
#define MMP2_PMUA_GC_RES	PMUA_REG(0x00cc)	/* 2D/3D Graphics Clock/Reset Control Register */
#define MMP2_PMUA_SMC_RES	PMUA_REG(0x00d4)	/* SMC Clock/Reset Control Register */
#define MMP2_PMUA_MSPRO_RES	PMUA_REG(0x00d8)	/* Memory Stick PRO Clock/Reset Control Register */
#define MMP2_PMUA_GLB_CTRL	PMUA_REG(0x00dc)	/* Global Clock Control Register */
#define MMP2_PMUA_ISL_CTRL	PMUA_REG(0x00e0)	/* Power Island On/Off Control Register */
#define MMP2_PMUA_ISL_TIMER	PMUA_REG(0x00e4)	/* Power Island Timer Register */
#define MMP2_PMUA_SDH2_RES	PMUA_REG(0x00e8)	/* SDIO Host 2 Clock/Reset Control Register */
#define MMP2_PMUA_SDH3_RES	PMUA_REG(0x00ec)	/* SDIO Host 3 Clock/Reset Control Register */
#define MMP2_PMUA_CCIC2_RES	PMUA_REG(0x00f4)	/* CCIC2 Clock/Reset Control Register */
#define MMP2_PMUA_HSIC1_RES	PMUA_REG(0x00f8)	/* HSIC1 Clock/Reset Control Register */
#define MMP2_PMUA_HSIC2_RES	PMUA_REG(0x00fc)	/* HSIC2 Clock/Reset Control Register */
#define MMP2_PMUA_FSIC3_RES	PMUA_REG(0x0100)	/* FSIC3 Clock/Reset Control Register */
#define MMP2_PMUA_SLIM_RES	PMUA_REG(0x0104)	/* Slimbus Clock/Reset Control Register */
#define MMP2_PMUA_HSI_RES	PMUA_REG(0x0108)	/* HSI Bus Clock/Reset control Register */
#define MMP2_PMUA_AUDIO_RES	PMUA_REG(0x010c)	/* Audio Bus Clock/Reset Control Register */
#define MMP2_PMUA_DIS2_RES	PMUA_REG(0x0110)	/* Display Controller 2 Clock/Reset Control Register */
#define MMP2_PMUA_CCIC2_GATE	PMUA_REG(0x0118)	/* CCIC2 Dynamic Clock Gate Control Register */
#define MMP2_PMUA_MC_PAR_CTRL	PMUA_REG(0x011c)	/* Memory Controller Parameter Table Control Register */
#define MMP2_PMUA_APB2_RES	PMUA_REG(0x0134)	/* APB2 Clock/Reset Control Register */

/* bit definition of MMP2_PMUA_PJ_IDLE_CFG */
#define MMP2_IDLE_PJ_IDLE	(1 << 1)
#define MMP2_IDLE_PJ_DWN	(1 << 5)
#define MMP2_IDLE_PJ_SRAMDWN	(1 << 6)
#define MMP2_IDLE_PJ_SW		(3 << 16)
#define MMP2_IDLE_PJ_L2SW	(3 << 18)
#define MMP2_IDLE_PJ_MC_WAKE	(1 << 20)
#define MMP2_IDLE_PJ_NO_MCREQ	(1 << 21)
#define MMP2_IDLE_PJ_ISO	(3 << 28)

/* bit definition of MMP2_PMUA_MC_HW_SLP */
#define MMP2_MC_HWSLP_SELFREF	(0)	/* self-refresh power down */
#define MMP2_MC_HWSLP_ACT_DWN	(1)	/* active power down */
#define MMP2_MC_HWSLP_PRE_DWN	(2)	/* precharge power down */
#define MMP2_MC_HWSLP_DEEP_DWN	(3)	/* deep power down */

#endif	/* __ASM_MACH_REGS_PMU_H */
