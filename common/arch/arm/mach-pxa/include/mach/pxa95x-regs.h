/*
 * arch/arm/mach-pxa/include/mach/pxa95x-regs.h
 *
 * PXA95x specific register definitions
 *
 * Copyright (C) 2010 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_ARCH_PXA95X_REGS_H
#define __ASM_ARCH_PXA95X_REGS_H

#include <mach/hardware.h>

/*
 * Static Chip Selects
 */

#define PXA95X_CS0_PHYS		(0x00000000)
#define PXA95X_CS1_PHYS		(0x30000000)
#define PXA95X_CS2_PHYS		(0x10000000)
#define PXA95X_CS3_PHYS		(0x14000000)

/*
 * Oscillator Configuration Register (OSCC)
 */
#define OSCC		__REG(0x41350000)	/* Oscillator Configuration Register */
#define OSCC_VCTVSTB_OFFSET (20)
#define OSCC_VCTVCEN	(1 << 23)
#define OSCC_VCTVSTB	(1 << OSCC_VCTVSTB_OFFSET)
#define OSCC_PEN	(1 << 11)		/* 13MHz POUT */
#define OSCC_TENS3	(1 << 10)
#define OSCC_TENS2	(1 << 9)
#define OSCC_TENS0	(1 << 8)

/*
 * Main Clock Control Unit Registers
 */
#define DMEMVLR		__REG(0x4135000C)	/* DMC Voltage Level Configuration Register */

#define DMEMVLR_DMCHV_OFFSET (0)

/*
 * Service Power Management Unit (MPMU)
 */
#define PMCR		__REG(0x40F50000)	/* Power Manager Control Register */
#define PSR		__REG(0x40F50004)	/* Power Manager S2 Status Register */
#define PSPR		__REG(0x40F50008)	/* Power Manager Scratch Pad Register */
#define PCFR		__REG(0x40F5000C)	/* Power Manager General Configuration Register */
#define PWER		__REG(0x40F50010)	/* Power Manager Wake-up Enable Register */
#define PWSR		__REG(0x40F50014)	/* Power Manager Wake-up Status Register */
#define PECR		__REG(0x40F50018)	/* Power Manager EXT_WAKEUP[1:0] Control Register */
#define OVH		__REG(0x40F50020)	/* Overheating Control Register */
#define DCDCSR		__REG(0x40F50080)	/* DC-DC Controller Status Register */
#define SDCR		__REG(0x40F5008C)	/* SRAM State-retentive Control Register */
#define AVCR		__REG(0x40F50094)	/* VCC_MAIN Voltage Control Register */
#define SVCR		__REG(0x40F50098)	/* VCC_SRAM Voltage Control Register */
#define PSBR		__REG(0x40F500A0)	/* Power Manager Safty Bits Register */
#define PVCR		__REG(0x40F50100)	/* Power Manager Voltage Change Control Register */

#define PMCR_BIE	(1 << 0)		/* Interrupt Enable for nBATT_FAULT */
#define PMCR_BIS	(1 << 1)		/* Interrupt Status for nBATT_FAULT */
#define PMCR_TIE	(1 << 10)		/* Interrupt Enable for XScale Core Frequency Change */
#define PMCR_TIS	(1 << 11)		/* Interrupt Status for XScale Core Frequency Change */
#define PMCR_VIE	(1 << 12)		/* Interrupt Enable for VCC_APPS and VCC_SRAM Voltage Change */
#define PMCR_VIS	(1 << 13)		/* Interrupt Status for VCC_APPS and VCC_SRAM Voltage Change */
#define PMCR_SWGR	(1 << 31)		/* Software GPIO Reset */
#define PSR_TSS_OFF	(12)
#define OVH_TEMP_EN	(1 << 0)		/* Enable for Temperature Sensor */
#ifdef CONFIG_CPU_PXA970
#define OVH_OWM		(1 << 19)		/* Over-heating WDT Enable */
#define OVH_OVWF_OFF	(10)			/* WDT Reset Temperature Over-heating Threshold */
#else
#define OVH_OWM		(1 << 7)		/* Over-heating WDT Enable */
#define OVH_OVWF_OFF	(4)			/* WDT Reset Temperature Over-heating Threshold */
#endif
#define OVH_OTIF_OFF	(1)			/* Over-heating Treshold Value for Generating TIS Software Interrupt */
#define PVCR_VCSA	(1 << 14)

#define AVCR_ALVL3_OFFSET 24
#define AVCR_ALVL3_MASK_5bit	(0x1F << AVCR_ALVL3_OFFSET)
#define AVCR_ALVL3_MASK		(0xFF << AVCR_ALVL3_OFFSET)
#define CORE_OVERHEATING_DETECTED 1
#define CORE_COLLING_DETECTED 0

/*
 * Slave Power Managment Unit
 */
#define ASCR		__REG(0x40F40000)	/* Application Subsystem Power Status/Configuration */
#define ARSR		__REG(0x40F40004)	/* Application Subsystem Reset Status */
#define AD3ER		__REG(0x40F40008)	/* Application Subsystem Wake-Up from D3 Enable */
#define AD3SR		__REG(0x40F4000C)	/* Application Subsystem Wake-Up from D3 Status */
#define AD2D0ER		__REG(0x40F40010)	/* Application Subsystem Wake-Up from D2 to D0 Enable */
#define AD2D0SR		__REG(0x40F40014)	/* Application Subsystem Wake-Up from D2 to D0 Status */
#define AD1D0ER		__REG(0x40F40020)	/* Application Subsystem Wake-Up from D1 to D0 Enable */
#define AD1D0SR		__REG(0x40F40024)	/* Application Subsystem Wake-Up from D1 to D0 Status */
#define AGENP		__REG(0x40F4002C)	/* Application Subsystem General Purpose */
#define AD1R		__REG(0x40F40038)	/* Application Subsystem D1 Configuration */
#define ACGD0ER		__REG(0x40F40040)	/* Application Subsystem CG to D0 state Wakeup Status Register */
#define ACGD0SR		__REG(0x40F40044)	/* Application Subsystem CG to D0 state Wakeup Status Register */
#define PWRMODE		__REG(0x40F40080)	/* Application Subsystem Power Mode Register */
#define CPUPWR		__REG(0x40F40084)	/* Application Subsystem CPU Power Mode Register */
#define VMPWR		__REG(0x40F40090)	/* Application Subsystem VMeta Power Mode Register */
#define GCPWR		__REG(0x40F40094)	/* Application Subsystem GCU Power Mode Register */

#define VMPWR_PWON		(1 << 0)
#define VMPWR_PWR_ST		(1 << 2)
#define VMPWR_SETALLWAYS	(0xFF00)
#define GCPWR_PWON		(1 << 0)
#define GCPWR_RST_N		(1 << 1)
#define GCPWR_PWR_ST		(1 << 2)
#define GCPWR_SETALLWAYS	(0xFF00)

/*
 * Application Subsystem Configuration bits.
 */
#define ASCR_RDH		(1 << 31)
#define ASCR_D1S		(1 << 2)
#define ASCR_D2S		(1 << 1)
#define ASCR_D3S		(1 << 0)

/*
 * Application Reset Status bits.
 */
#define ARSR_GPR		(1 << 3)
#define ARSR_LPMR		(1 << 2)
#define ARSR_WDT		(1 << 1)
#define ARSR_HWR		(1 << 0)

/*
 * Application Subsystem Wake-Up bits.
 */
#define ADXER_WRTC		(1 << 31)	/* RTC */
#define ADXER_WOST		(1 << 30)	/* OS Timer */
#define ADXER_WDMC		(1 << 27)	/* DMC (only in clock-gated mode) */
#define ADXER_WUSB2		(1 << 26)	/* USB client 2.0 */
#define ADXER_WHSI		(1 << 25)	/* HSI RX */
#define ADXER_WACIPC		(1 << 24)	/* ACIPC */
#define ADXER_WIC_USB		(1 << 23)	/* IC_USB */
#define ADXER_WMMC3_DAT		(1 << 22)	/* MMC3_DAT[7:0] */
#define ADXER_WMKIN		(1 << 21)	/* Matrix Keypad MKIN[7:0] */
#define ADXER_WSSP3		(1 << 20)	/* SSP3 clock, frame and Rx */
#define ADXER_WTRBALL		(1 << 19)	/* Trackball or Enhanced-rotary key */
#define ADXER_WABU		(1 << 18)	/* ABU */
#define ADXER_WIRQ		(1 << 17)	/* Pending IRQ or FIQ before entering CG */
#define ADXER_WGPIO		(1 << 15)	/* Except for GPIO83 */
#define ADXER_WCOMM_WDT		(1 << 14)	/* Comm Watchdog Timer interrupt */
#define ADXER_WSSP2		(1 << 13)	/* SSP2 clock, frame and Rx */
#define ADXER_WCI2C		(1 << 12)	/* Common I2C */
#define ADXER_WFFUART		(1 << 11)	/* FFUART CTS, RX, DCD, DSR, RI */
#define ADXER_WMMC4_DAT		(1 << 10)	/* MMC4_DAT[3:0] */
#define ADXER_WSSP1		(1 << 9)	/* SSP1 clock, frame and Rx */
#define ADXER_WMMC2_DAT		(1 << 8)	/* MMC2_DAT[3:0] */
#define ADXER_WMMC1_DAT		(1 << 7)	/* MMC1_DAT[3:0] */
#define ADXER_WNAND		(1 << 6)	/* NAND Ready/Busy */
#define ADXER_WPMIC		(1 << 5)	/* PMIC (GPIO83) */
#define ADXER_WBTST_UART	(1 << 4)	/* BTUART or STUART */
#define ADXER_WDKIN0		(1 << 3)	/* Direct Keypad DKIN[3:0] */
#define ADXER_WDKIN4		(1 << 2)	/* Direct Keypad DKIN[7:4] */
#define ADXER_WEXTWAKE		(1 << 0)	/* External Wake 0 */

/*
 * Values for PWRMODE register
 */
#define PXA95X_PM_S3D4C4	0x07	/* aka power off */
#define PXA95X_PM_S2D3C4	0x06	/* Not used */
#define PXA95x_PM_S0D0CS	0x05	/* aka D0CS */
#define PXA95X_PM_S0D2C2	0x03	/* aka D2 */
#define PXA95X_PM_S0D1C2	0x02	/* aka D1 */
#define PXA95X_PM_S0D0C1	0x01	/* aka core idle */

/*
 * Application Subsystem Clock
 */
#define ACCR		__REG(0x41340000)	/* Application Subsystem Clock Configuration Register */
#define ACSR		__REG(0x41340004)	/* Application Subsystem Clock Status Register */
#define AICSR		__REG(0x41340008)	/* Application Subsystem Interrupt Control/Status Register */
#define CKENA		__REG(0x4134000C)	/* A Clock Enable Register */
#define CKENB		__REG(0x41340010)	/* B Clock Enable Register */
#define ACCR1		__REG(0x41340020)	/* Application Subsystem Clock Configuration Register 1 */
#define CKENC		__REG(0x41340024)	/* C Clock Enable Register */
#define CCLKCFG		__REG(0x41340040)	/* Core Clock Configuration Register */

#define ACCR_XPDIS		(1 << 31)	/* Core PLL Output Disable */
#define ACCR_SPDIS		(1 << 30)	/* System PLL Output Disable */
#define ACCR_D0CS		(1 << 26)	/* D0 Mode Clock Select */
#define ACCR_PCCE		(1 << 11)	/* Power Mode Change Clock Enable */
#define ACCR_DDR_D0CS		(1 << 7)	/* DDR SDRAM clock frequency in D0CS */
#define ACCR_DMCFS_312		(1 << 6)	/* DDR SDRAM clock frequency 312MHz */

#define ACCR_XPDIS_MASK		(0x1 << 31)	/* Core PLL Output Disable */
#define ACCR_SPDIS_MASK		(0x1 << 30)	/* System PLL Output Disable */
#define ACCR_AXIFS_MASK		(0x3 << 28)	/* AXI Bus Frequency Select */
#define ACCR_D0CS_MASK		(0x1 << 26)	/* D0 Mode Clock Select */
#define ACCR_SMCFS_MASK		(0x7 << 23)	/* Static Memory Controller Frequency Select */
#define ACCR_GCFS_MASK		(0x3 << 20)	/* Graphics Controller Frequency Select */
#define ACCR_SFLFS_MASK		(0x3 << 18)	/* Frequency Select for Internal Memory Controller */
#define ACCR_XSPCLK_MASK	(0x3 << 16)	/* Core Frequency during Frequency Change */
#define ACCR_HSS_MASK		(0x3 << 14)	/* System Bus-Clock Frequency Select */
#define ACCR_DMCFS_MASK		(0x3 << 12)	/* Dynamic Memory Controller Clock Frequency Select */
#define ACCR_XN_MASK		(0x7 << 8)	/* Core PLL Turbo-Mode-to-Run-Mode Ratio */
#define ACCR_DDR_D0CS_MASK	(0x1 << 7)	/* Memory Clock in D0CS */
#define ACCR_DMCFS_312_MASK	(0x1 << 6)	/* DMC PLL Select */
#define ACCR_XL_MASK		(0x3f)		/* Core PLL Run-Mode-to-Oscillator Ratio */

#define ACCR_AXI(x)		(((x) & 0x3) << 28)
#define ACCR_SMCFS(x)		(((x) & 0x7) << 23)
#define ACCR_GCFS(x)		(((x) & 0x3) << 20)
#define ACCR_SFLFS(x)		(((x) & 0x3) << 18)
#define ACCR_XSPCLK(x)		(((x) & 0x3) << 16)
#define ACCR_HSS(x)		(((x) & 0x3) << 14)
#define ACCR_DMCFS(x)		(((x) & 0x3) << 12)
#define ACCR_XN(x)		(((x) & 0x7) << 8)
#define ACCR_XL(x)		((x) & 0x3f)

#define ACCR1_DIS_DRX		(1 << 31)	/* Disable DRX */
#define ACCR1_VMETA_156_312	(1 << 21)	/* VMeta Frequency Control: 0 = 156Mhz, 1 = 312Mhz */
#define ACCR1_PU_OTG		(1 << 12)	/* USB 2.0 PHY OTG power up */
#define ACCR1_PU_PLL		(1 << 11)	/* USB 2.0 PHY PLL power up */
#define ACCR1_PU		(1 << 10)	/* USB 2.0 PHY power up */
#define ACCR1_I2C_33_52		(1 << 8)	/* I2C frequency control: 0 = 624/19 Mhz, 1 = 624/12 Mhz */
#define ACCR1_MMC6_48_52	(1 << 6)	/* MMC6 frequency control: 0 = 624/13 Mhz, 1 = 624/12 Mhz */
#define ACCR1_MMC5_48_52	(1 << 4)	/* MMC5 frequency control: 0 = 624/13 Mhz, 1 = 624/12 Mhz */
#define ACCR1_MMC4_48_52	(1 << 2)	/* MMC4 frequency control: 0 = 624/13 Mhz, 1 = 624/12 Mhz */
#define ACCR1_MMC3_48_52	(1 << 0)	/* MMC3 frequency control: 0 = 624/13 Mhz, 1 = 624/12 Mhz */

/*
 * Clock Enable Bit
 */
#define CKEN_NAND	4	/* < NAND Flash Controller Clock Enable */
#define CKEN_DMC	8	/* < Dynamic Memory Controller clock enable */
#define CKEN_SMC	9	/* < Static Memory Controller clock enable */
#define CKEN_ISC	10	/* < Internal SRAM Controller clock enable */
#define CKEN_BOOT	11	/* < Boot rom clock enable */
#define CKEN_KEYPAD	14	/* < Keypand Controller Clock Enable */
#define CKEN_CIR	15	/* < Consumer IR Clock Enable */
#define CKEN_TPM	19	/* < TPM clock enable */
#define CKEN_BTUART	21	/* < BTUART clock enable */
#define CKEN_FFUART	22	/* < FFUART clock enable */
#define CKEN_STUART	23	/* < STUART clock enable */
#define CKEN_SSP1	26	/* < SSP1 clock enable */
#define CKEN_SSP2	27	/* < SSP2 clock enable */
#define CKEN_SSP3	28	/* < SSP3 clock enable */
#define CKEN_ABU_SSI	29	/* < ABU SSI clock enable */
#define CKEN_MSL0	30	/* < MSL0 clock enable */
#define CKEN_PWM0	32	/* < PWM[0] clock enable */
#define CKEN_PWM1	33	/* < PWM[1] clock enable */
#define CKEN_HSI	34	/* < HSI clock enable */
#define CKEN_VMETA	35	/* < VMeta clock enable */
#define CKEN_I2C1	36	/* < I2C1 clock enable */
#define CKEN_GPIO	39	/* < GPIO clock enable */
#define CKEN_1WIRE	40	/* < 1-wire clock enable */
#define CKEN_ABU	59	/* < ABU clock enable */
#define CKEN_HSIO	61	/* < System Bus (HSIO) clock enable */
#define CKEN_CSI_TX	64	/* < CSI TX Escape clock enable */
#define CKEN_MMC1	65	/* < MMC1 clock enable */
#define CKEN_MMC2	66	/* < MMC2 clock enable */
#define CKEN_MMC3	67	/* < MMC3 clock enable */
#define CKEN_MMC4	68	/* < MMC4 clock enable */
#define CKEN_AXI_2X	69	/* < AXI 2X clock enable */
#define CKEN_USB_PRL	70	/* < USB2(OTG) Peripheral clock enable */
#define CKEN_USBH_PRL	71	/* < USB2 Host1 Peripheral clock enable */
#define CKEN_USB_BUS	74	/* < USB2(OTG) System Bus clock enable */
#define CKEN_USBH_BUS	75	/* < USB2 Host1 System Bus clock enable */
#define CKEN_MMC1_BUS	78	/* < MMC1 System Bus clock enable */
#define CKEN_MMC2_BUS	79	/* < MMC2 System Bus clock enable */
#define CKEN_MMC3_BUS	80	/* < MMC3 System Bus clock enable */
#define CKEN_MMC4_BUS	81	/* < MMC4 System Bus clock enable */
#define CKEN_IMU	82	/* < ISLAND MMC USB System Bus clock enable */
#define CKEN_AXI	83	/* < AXI clock enable */
#define CKEN_DISPLAY	86	/* < Display clock enable */
#define CKEN_PIXEL	87	/* < Pixel clock enable */
#define CKEN_I2C2	88	/* < I2C2 clock enable */
#define CKEN_I2C3	89	/* < I2C3 clock enable */
#define CKEN_SCI1	90	/* < Camera SCI1 clock enable */
#define CKEN_SCI2	91	/* < Camera SCI2 clock enable */
#define CKEN_GC_1X	92	/* < Graphics 1x clock enable */
#define CKEN_GC_2X	93	/* < Graphics 2x clock enable */
#define CKEN_DSI_TX1	94	/* < DSI TX1 Escape clock enable */
#define CKEN_DSI_TX2	95	/* < DSI TX2 Escape clock enable */

#define CKEN_PWM4	0	/* < PWMCCR4 offset  */
#define CKEN_PWM5	4	/* < PWMCCR4 offset  */
#define CKEN_PWM6	8	/* < PWMCCR4 offset  */
#define CKEN_PWM7	12	/* < PWMCCR4 offset  */
#define PWMCLKEN_SLOW	(1 << 6)	/* PWM SLOW CLK enble Reset */

#endif /* __ASM_ARCH_PXA95X_REGS_H */
