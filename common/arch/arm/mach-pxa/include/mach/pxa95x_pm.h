/*
 * arch/arm/mach-pxa/include/mach/pxa95x_pm.h
 *
 * PXA95x Power Management Routines Head File
 *
 * Copyright (C) 2010 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __PXA95X_PM_H__
#define __PXA95X_PM_H__

#include <asm/types.h>

/* clock manager registers */
#define ACCR_OFF		0x00
#define ACSR_OFF		0x04
#define AICSR_OFF		0x08
#define D0CKEN_A_OFF		0x0c
#define D0CKEN_B_OFF		0x10
#define AC97_DIV_OFF		0x14
#define ACCR1_OFF		0x20
#define OSCC_OFF		0x10000

/* service power management uinit */
#define PSR_OFF			0x004
#define PSPR_OFF		0x008
#define PCFR_OFF		0x00C
#define PWER_OFF		0x010
#define PWSR_OFF		0x014
#define PECR_OFF		0x018
#define CSER_OFF		0x01C
#define DCDCSR_OFF		0x080
#define AVCR_OFF		0x094
#define SVCR_OFF		0x098
#define CVCR_OFF		0x09C
#define PSBR_OFF		0x0A0
#define PVCR_OFF		0x100
#define SDCR_OFF		0x08C

/* slave power management unit */
#define ASCR_OFF		0x00
#define ARSR_OFF		0x04
#define AD3ER_OFF		0x08
#define AD3SR_OFF		0x0c
#define AD2D0ER_OFF		0x10
#define AD2D0SR_OFF		0x14
#define AD2D1ER_OFF		0x18
#define AD2D1SR_OFF		0x1c
#define AD1D0ER_OFF		0x20
#define AD1D0SR_OFF		0x24
#define ASDCNT_OFF		0x28
#define AGENP_OFF		0x2c
#define AD3R_OFF		0x30
#define AD2R_OFF		0x34
#define AD1R_OFF		0x38

/* dynamic memory controller registers */
#define MDCNFG_OFF		0x0000
#define MDREFR_OFF		0x0004
#define FLYCNFG_OFF		0x0020
#define MDMRS_OFF		0x0040
#define DDR_SCAL_OFF		0x0050
#define DDR_HCAL_OFF		0x0060
#define DDR_WCAL_OFF		0x0068
#define DMCIER_OFF		0x0070
#define DMCISR_OFF		0x0078
#define DMCISR2_OFF		0x007C
#define DDR_DLS_OFF		0x0080
#define EMPI_OFF		0x0090
#define RCOMP_OFF		0x0100
#define PAD_MA_OFF		0x0110
#define PAD_MDMSB_OFF		0x0114
#define PAD_MDLSB_OFF		0x0118
#define PAD_SDRAM_OFF		0x011C
#define PAD_SDCLK_OFF		0x0120
#define PAD_SDCS_OFF		0x0124
#define PAD_SMEM_OFF		0x0128
#define PAD_SCLK_OFF		0x012C

/* static memory controller registers */
#define MSC0_OFF		0x0008
#define MSC1_OFF		0x000C
#define MECR_OFF		0x0014
#define SXCNFG_OFF		0x001C
#define MCMEM0_OFF		0x0028
#define MCATT0_OFF		0x0030
#define MCIO0_OFF		0x0038
#define MEMCLKCFG_OFF		0x0068
#define CSADRCFG0_OFF		0x0080
#define CSADRCFG1_OFF		0x0084
#define CSADRCFG2_OFF		0x0088
#define CSADRCFG3_OFF		0x008C
#define CSADRCFG_P_OFF		0x0090
#define CSMSADRCFG_OFF		0x00A0
#define CLK_RET_DEL_OFF		0x00B0
#define ADV_RET_DEL_OFF		0x00B4

/* OS Timer address space */
#define OST_START		0x40a00000
#define OST_END			0x40a000df

/* System Bus Arbiter address space */
#define ARB_START		0x4600fe00
#define ARB_END			0x4600fe07

/* Registers offset within ARB space */
#define ARBCTL1_OFF		0x0000
#define ARBCTL2_OFF		0x0004

/* Dynamic memory controll address space */
#define DMC_START		0x48100000
#define DMC_END			0x48100fff

/* static memory controll address space */
#define SMC_START		0x4a000000
#define SMC_END			0x4a0000ff

/* Power Management Unit address space */
#define PM_START		0x40f50000
#define PM_END			0x40f5018f

/* Bits definition for Clock Control Register */
#define ACCR_PCCE	(1 << 11)

#define ACSR_XPLCK	(1 << 29)
#define ACSR_SPLCK	(1 << 28)

#if defined(CONFIG_PXA95x)
#define AICSR_WSIDLE (1 << 11)
#define AICSR_WEIDLE (1 << 10)
#define AICSR_WSD0CS (1 << 9)
#define AICSR_WED0CS (1 << 8)
#else
#define AICSR_WSIDLE 0
#define AICSR_WEIDLE 0
#define AICSR_WSD0CS 0
#define AICSR_WED0CS 0
#endif
#define AICSR_PCIS	(1 << 5)
#define AICSR_PCIE	(1 << 4)
#define AICSR_TCIS	(1 << 3)
#define AICSR_TCIE	(1 << 2)
#define AICSR_FCIS	(1 << 1)
#define AICSR_FCIE	(1 << 0)
#define AICSR_STATUS_BITS	\
	(AICSR_PCIS | AICSR_FCIS | AICSR_WSD0CS | AICSR_TCIS | AICSR_WSIDLE)

/* Bits definition for RTC Register */
#define RTSR_PICE	(1 << 15)
#define RTSR_PIALE	(1 << 14)

/* Bits definition for Power Control Register */
#define ASCR_RDH	(1 << 31)
#define ASCR_D1S	(1 << 2)
#define ASCR_D2S	(1 << 1)
#define ASCR_D3S	(1 << 0)
#define ASCR_MASK	(ASCR_D1S | ASCR_D2S | ASCR_D3S)
#define PSR_MASK	0x07
#define PCFR_L1DIS	(1 << 13)
#define PCFR_L0EN	(1 << 12)
#define PECR_E1IS	(1 << 31)
#define PECR_E1IE	(1 << 30)
#define PECR_E0IS	(1 << 29)
#define PECR_E0IE	(1 << 28)
#define PECR_DIR1	(1 << 5)
#define PECR_DIR0	(1 << 4)

/* Bits definition for Oscillator Configuration Register */
#define OSCC_GPRM		(1 << 18)	/* GB PLL Request Mask */
#define OSCC_GPLS		(1 << 17)	/* GB PLL Lock Status */

/* Bits definition for Application Subsystem General Purpose Register */
#define AGENP_GBPLL_CTRL	(1 << 29)
#define AGENP_GBPLL_DATA	(1 << 28)	/* Turn on/off GB PLL */
#define AGENP_SAVE_WK		(1 << 2)	/* Save wakeup */

/* Registers offset within ARB space */
#define BPB_START		0x42404100
#define BPB_END			0x4240414B

/* GPIO Wakeup Status Register */
#define GWSR(x)			((x << 2) + 0x38)
#define GWSR1			0x3C
#define GWSR2			0x40
#define GWSR3			0x44
#define GWSR4			0x48

/* bits definitions */
#define ASCR_MTS_OFFSET		12
#define ASCR_MTS_S_OFFSET	8

#define ACCR_XL_OFFSET		0
#define ACCR_DMCFS_312_OFFSET	6
#define ACCR_XN_OFFSET		8
#define ACCR_DMCFS_OFFSET	12
#define ACCR_HSS_OFFSET		14
#define ACCR_XSPCLK_OFFSET	16
#define ACCR_SFLFS_OFFSET	18
#define ACCR_GCFS_OFFSET	20
#define ACCR_SMCFS_OFFSET	23
#define ACCR_D0CS_OFFSET	26
#define ACCR_AXIFS_OFFSET	28
#define ACCR_SPDIS_OFFSET	30
#define ACCR_XPDIS_OFFSET	31

#define ACSR_AXIFS_OFFSET	21
#define ACSR_GCFS_OFFSET	6

#define ACCR1_DSI_1_OFFSET	13
#define ACCR1_DSI_2_OFFSET	16
#define ACCR1_VMFC_OFFSET	21

#define ACCR1_XPDIS_MASK		(0x01 << ACCR_XPDIS_OFFSET)
#define ACSR_AXIFS_MASK		(0x03 << ACSR_AXIFS_OFFSET)
#define ACSR_GCFS_MASK		(0x03 << ACSR_GCFS_OFFSET)
#define ACCR1_DSI_1_MASK		(0x07 << ACCR1_DSI_1_OFFSET)
#define ACCR1_DSI_2_MASK		(0x07 << ACCR1_DSI_2_OFFSET)
#define ACCR1_VMFC_MASK		(0x01 << ACCR1_VMFC_OFFSET)

#define MEMCLKCFG_EMPI_OFFSET	0
#define MEMCLKCFG_DF_OFFSET	16

#define HCAL_HCEN_OFFSET	31

#define MDCNFG_HWNOPHD_OFFSET	28
#define MDCNFG_HWFREQ_OFFSET	29
#define MDCNFG_DMCEN_OFFSET	30
#define MDCNFG_DMAP_OFFSET	31

#define CPU_PDWN_ENABLE		0x1
#define CPU_PDWN_DISABLE	0x0
#define CPU_PDWN_LPM_ENTRY	0x10
#define CPU_PDWN_LPM_EXIT	0x11
#define CPU_PDWN_D0CS_ENTRY	0x12
#define CPU_PDWN_D0CS_EXIT	0x13
#define	CPU_PDWN_SETALLWAYS  0x00003F00
#define	CPU_PDWN_3_25M_CYCLES	0x13 /* WA see JIRA 1495. Increased, 0x10 was marginal */
#define CPU_LOOP_COUNT_ON_EXIT_CGM_LOW_PP  1500
#define CPU_LOOP_COUNT_ON_EXIT_CGM_HIGH_PP 6000

/* mode save flags */
#define PM_MODE_SAVE_FLAG_SYS	0x1
#define PM_MODE_SAVE_FLAG_IRQ	0x2
#define PM_MODE_SAVE_FLAG_FIQ	0x4
#define PM_MODE_SAVE_FLAG_ABT	0x8
#define PM_MODE_SAVE_FLAG_UND	0x10
#define PM_MODE_SAVE_FLAG_SVC	0x20

/* value for PWRMODE register */
#define PXA95x_PM_S2D3C4	0x06
#define PXA95x_PM_S0D2C2	0x03
#define PXA95x_PM_S3D4C4	0x07
#define PXA95x_PM_S0D1C2	0x02
#define PXA95x_PM_S0D0C1	0x01
#if defined(CONFIG_PXA95x)
#define PXA95x_PM_I_Q_BIT	0x20
#else
#define PXA95x_PM_I_Q_BIT	0x00
#endif

/* CPSR Processor constants */
#define CPSR_Mode_MASK		(0x0000001F)
#define CPSR_Mode_USR		(0x10)
#define CPSR_Mode_FIQ		(0x11)
#define CPSR_Mode_IRQ		(0x12)
#define CPSR_Mode_SVC		(0x13)
#define CPSR_Mode_ABT		(0x17)
#define CPSR_Mode_UND		(0x1B)
#define CPSR_Mode_SYS		(0x1F)
#define CPSR_I_Bit		(0x80)
#define CPSR_F_Bit		(0x40)


/****************************************************************************/
#define PXA95x_PM_WE_EXTERNAL0	(0x1UL << 0)
#define PXA95x_PM_WE_EXTERNAL1	(0x1UL << 1)
#define PXA95x_PM_WE_GENERIC(x)	(0x1UL << (x + 2))	/* x is in [0,13] */
#define PXA95x_PM_WE_OTG	(0x1UL << 16)
#define PXA95x_PM_WE_INTC	(0x1UL << 17)
#define PXA95x_PM_WE_MLCD	(0x1UL << 18)
#define PXA95x_PM_WE_USIM0	(0x1UL << 19)
#define PXA95x_PM_WE_USIM1	(0x1UL << 20)
#define PXA95x_PM_WE_KP		(0x1UL << 21)
#define PXA95x_PM_WE_MMC3	(0x1UL << 22)
#define PXA95x_PM_WE_MUX3	(0x1UL << 23)
#define PXA95x_PM_WE_MSL0	(0x1UL << 24)
#define PXA95x_PM_WE_RESERVE1	(0x1UL << 25)
#define PXA95x_PM_WE_USB2	(0x1UL << 26)
#define PXA95x_PM_WE_DMC	(0x1UL << 27)
#define PXA95x_PM_WE_USBH	(0x1UL << 28)
#define PXA95x_PM_WE_TSI	(0x1UL << 29)
#define PXA95x_PM_WE_OST	(0x1UL << 30)
#define PXA95x_PM_WE_RTC	(0x1UL << 31)


#define PWSR_EDR0		(0x1 << 0)
#define PWSR_EDR1		(0x1 << 1)
#define PWSR_EDF0		(0x1 << 2)
#define PWSR_EDF1		(0x1 << 3)
#define PWSR_EERTC		(0x1 << 31)

#define PWER_WER0		(0x1 << 0)
#define PWER_WER1		(0x1 << 1)
#define PWER_WEF0		(0x1 << 2)
#define PWER_WEF1		(0x1 << 3)
#define PWER_WERTC		(0x1 << 31)


#define WORD_SIZE 4

/* the position of each data memeber */
#define SleepState_begin		0x0
#define SleepState_checksum		0x0
#define SleepState_wordCount		(SleepState_checksum + WORD_SIZE)
#define SleepState_areaAddress		(SleepState_wordCount + WORD_SIZE)
#define SleepState_modeSaveFlags	(SleepState_areaAddress + WORD_SIZE)

/* save ARM registers */
#define SleepState_ENTRY_REGS		(SleepState_modeSaveFlags + WORD_SIZE)
#define SleepState_ENTRY_CPSR		(SleepState_ENTRY_REGS)
#define SleepState_ENTRY_SPSR		(SleepState_ENTRY_CPSR + WORD_SIZE)
#define SleepState_ENTRY_R0		(SleepState_ENTRY_SPSR + WORD_SIZE)
#define SleepState_ENTRY_R1		(SleepState_ENTRY_R0 + WORD_SIZE)
#define SleepState_SYS_REGS		(SleepState_ENTRY_REGS + 17*WORD_SIZE)
#define SleepState_FIQ_REGS		(SleepState_SYS_REGS + 2*WORD_SIZE)
#define SleepState_IRQ_REGS		(SleepState_FIQ_REGS + 8*WORD_SIZE)
#define SleepState_ABT_REGS		(SleepState_IRQ_REGS + 3*WORD_SIZE)
#define SleepState_UND_REGS		(SleepState_ABT_REGS + 3*WORD_SIZE)
#define SleepState_SVC_REGS		(SleepState_UND_REGS + 3*WORD_SIZE)

/* save MMU settings */
#define SleepState_Cp15_ACR_MMU		(SleepState_SVC_REGS + 3*WORD_SIZE)
#define SleepState_Cp15_AUXCR_MMU	(SleepState_Cp15_ACR_MMU + WORD_SIZE)
#define SleepState_Cp15_TTBR_MMU	(SleepState_Cp15_AUXCR_MMU + WORD_SIZE)
#define SleepState_Cp15_DACR_MMU	(SleepState_Cp15_TTBR_MMU + WORD_SIZE)
#define SleepState_Cp15_PID_MMU		(SleepState_Cp15_DACR_MMU + WORD_SIZE)
#define SleepState_Cp15_CPAR		(SleepState_Cp15_PID_MMU + WORD_SIZE)

#define SleepState_extendedChecksumByteCount	(SleepState_Cp15_CPAR + WORD_SIZE)
#define SleepState_psprAddress		(SleepState_extendedChecksumByteCount + WORD_SIZE)
#define SleepState_flushFunc		(SleepState_psprAddress + WORD_SIZE)
#define SleepState_end			(SleepState_flushFunc + WORD_SIZE)
#define SleepState_size			(SleepState_end - SleepState_begin)

#ifndef __ASSEMBLY__

typedef struct {
	unsigned long value;
	struct {
		unsigned ext0:1;
		unsigned ext1:1;
		unsigned uart1:1;
		unsigned uart2:1;
		unsigned uart3:1;
		/* wifi use UART1's pin as wakeup source */
		unsigned wifi:1;
		unsigned mmc1_cd:1;
		unsigned mmc2_cd:1;
		unsigned mmc3_cd:1;
		unsigned mmc1_dat1:1;
		unsigned mmc2_dat1:1;
		unsigned mmc3_dat1:1;
		unsigned mkey:1;
		unsigned usbotg:1;
		unsigned mlcd:1;
		unsigned dkey:1;
		unsigned usb2:1;	/* USB 2.0 client */
		unsigned usbh:1;	/* USB Host Port 1 */
		unsigned msl:1;
		unsigned tsi:1;
		unsigned ost:1;
		unsigned rtc:1;
		unsigned eth:1;
		unsigned cmwdt:1;
		unsigned usbdetect :1;
		unsigned headset :1;
		unsigned hookswitch :1;
		unsigned proximate_sensor :1;
	} bits;
} pm_wakeup_src_t;


#ifdef __KERNEL__
struct intc_regs {
	unsigned int iccr;
	unsigned int ipr[32];
	unsigned int ipr2[21];
	unsigned int icmr;
	unsigned int icmr2;
	unsigned int iclr;
	unsigned int iclr2;
};

struct clock_regs {
	unsigned int aicsr;
	unsigned int ckena;
	unsigned int ckenb;
	unsigned int oscc;
};

struct ost_regs {
	unsigned int ossr;
	unsigned int oier;
	unsigned int oscr;
	unsigned int oscr4;
	unsigned int osmr4;
	unsigned int omcr4;
};

struct rtc_regs {
	unsigned int rtsr;
	unsigned int piar;
};

struct smc_regs {
	unsigned char __iomem *membase;
	unsigned int msc0;
	unsigned int msc1;
	unsigned int mecr;
	unsigned int sxcnfg;
	unsigned int mcmem0;
	unsigned int mcatt0;
	unsigned int mcio0;
	unsigned int memclkcfg;
	unsigned int cscfg0;
	unsigned int cscfg1;
	unsigned int cscfg2;
	unsigned int cscfg3;
	unsigned int cscfg_p;
	unsigned int csmscfg;
};

struct arb_regs {
	unsigned char __iomem *membase;
	unsigned int ctl1;
	unsigned int ctl2;
};

struct pmu_regs {
	unsigned int pcfr;
	unsigned int pecr;
	unsigned int pvcr;
};

#define MAX_MFP_PINS 419

struct mfp_regs {
	unsigned int mfp[MAX_MFP_PINS];
};

struct gpio_regs {
	unsigned int gplr0;
	unsigned int gplr1;
	unsigned int gplr2;
	unsigned int gplr3;
	unsigned int gpdr0;
	unsigned int gpdr1;
	unsigned int gpdr2;
	unsigned int gpdr3;
	unsigned int grer0;
	unsigned int grer1;
	unsigned int grer2;
	unsigned int grer3;
	unsigned int gfer0;
	unsigned int gfer1;
	unsigned int gfer2;
	unsigned int gfer3;
};

struct pm_save_data {
	u32 checksum;
	u32 wordCount;
	u32 areaAddress;
	u32 modeSaveFlags;
	/* current mode registers cpsr, sprsr, r0-r12, lr, sp */
	u32 ENTRY_REGS[17];
	/* SYS mode registers:sp, lr */
	u32 SYS_REGS[2];
	/* FIQ mode registers:spsr, r8-r12, sp, lr */
	u32 FIQ_REGS[8];
	/* IRQ mode registers:spsr, sp, lr */
	u32 IRQ_REGS[3];
	/* ABT mode registers:spsr, sp, lr */
	u32 ABT_REGS[3];
	/* UND mode registers:spsr, sp, lr */
	u32 UND_REGS[3];
	/* SVC mode registers:spsr, sp, lr */
	u32 SVC_REGS[3];
	/* MMU registers */
	u32 CP15_ACR_MMU;
	u32 CP15_AUXCR_MMU;
	u32 CP15_TTBR_MMU;
	u32 CP15_DACR_MMU;
	u32 CP15_PID_MMU;
	u32 CP15_CPAR;

	u32 extendedChecksumByteCount;
	u32 psprAddress;
	void (*flushFunc) (void);
	/* the parameter is the reserved bytes from 0x5c010000 */
	/* It returns the physical address of initlization code in SRAM */
};

struct pxa95x_pm_regs {
	/* It's used to save core registers. */
	struct pm_save_data pm_data;
	struct mfp_regs mfp;
	struct gpio_regs gpio;
	struct intc_regs intc;
	struct clock_regs clock;
	struct ost_regs ost;
	struct rtc_regs rtc;
	struct smc_regs smc;
	struct arb_regs arb;
	struct pmu_regs pmu;
	/* It's the virtual address of ISRAM that can be accessed by kernel.
	 */
	void *sram_map;
	/* It's used to save ISRAM data. */
	void *sram;
	/* It's the address of DDR that stores key information.
	 * Two words are used from the address.
	 */
	void *data_pool;
	unsigned int word0;
	unsigned int word1;
};

extern pm_wakeup_src_t wakeup_src;

struct pxa95x_peripheral_wakeup_ops {
	int (*init) (pm_wakeup_src_t *src);
	int (*query) (unsigned int reg, pm_wakeup_src_t *src);
	int (*ext) (pm_wakeup_src_t src, int enable);
	int (*key) (pm_wakeup_src_t src, int enable);
	int (*mmc1) (pm_wakeup_src_t src, int enable);
	int (*mmc3) (pm_wakeup_src_t src, int enable);
	int (*uart) (pm_wakeup_src_t src, int enable);
	int (*eth) (pm_wakeup_src_t src, int enable);
	int (*tsi) (pm_wakeup_src_t src, int enable);
	int (*cmwdt) (pm_wakeup_src_t src, int enable);
	int (*usbdetect) (pm_wakeup_src_t src, int enable);
	int (*headset) (pm_wakeup_src_t src, int enable);
	int (*hookswitch) (pm_wakeup_src_t src, int enable);
	int (*proximate_sensor) (pm_wakeup_src_t src, int enable);
};

#define GC_PWR_ENABLE		(1)
#define GC_PWR_DISABLE		(0)
extern int temperture_sensor_int_high_freq_pp_callback(int highTempDetected);
extern int pxa95x_wakeup_register(struct pxa95x_peripheral_wakeup_ops *);
extern void gc_pwr(int enableDisable);
extern int set_acipc_cp_enable(unsigned int pm_cp);
extern void pm_enter_idle(void);
void pxa95x_cpu_standby(unsigned int, unsigned int, unsigned int, unsigned int);
extern unsigned int pm_core_pwdn(unsigned int powerState);
extern unsigned int pm_enter_deepidle(unsigned int);
extern unsigned int pm_enter_cgm_deepidle(unsigned int);
extern int pxa95x_query_gwsr(int);
extern u32 get_mipi_reference_control(void);

#endif
#endif
#endif
