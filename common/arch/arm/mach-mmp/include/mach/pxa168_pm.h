/*
 * PXA168 Power Management Routines
 *
 * Copyright (C) 2004, Intel Corporation(chao.xie@intel.com).
 *
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html
 *
 * (C) Copyright 2009 Marvell International Ltd.
 * All Rights Reserved
 */

#ifndef __PXA3xx_PM_H__
#define __PXA3xx_PM_H__

#include <asm/types.h>

/* mode save flags */
#define PM_MODE_SAVE_FLAG_SYS	0x1
#define PM_MODE_SAVE_FLAG_IRQ	0x2
#define PM_MODE_SAVE_FLAG_FIQ	0x4
#define PM_MODE_SAVE_FLAG_ABT	0x8
#define PM_MODE_SAVE_FLAG_UND	0x10
#define PM_MODE_SAVE_FLAG_SVC	0x20

/* value for PWRMODE register */
#define PXA3xx_PM_S2D3C4	0x06
#define PXA3xx_PM_S0D2C2	0x03
#define PXA3xx_PM_S3D4C4	0x07
#define PXA3xx_PM_S0D1C2	0x02
#define PXA3xx_PM_S0D0C1	0x01

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

/* Timer definitions */
#define TMRnm(n, m)		__REG_PXA910(0xd4014004+(n)*12+(m)*4)
#define TIERn(n)		__REG_PXA910(0xd4014040+(n)*4)
#define TICRn(n)		__REG_PXA910(0xd4014074+(n)*4)
#define TCVWRRn(n)		__REG_PXA910(0xd40140a4+(n)*4)

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

#define SleepState_extendedChecksumByteCount	\
	(SleepState_Cp15_CPAR + WORD_SIZE)
#define SleepState_flushFunc		\
	(SleepState_extendedChecksumByteCount + WORD_SIZE)
#define SleepState_end			(SleepState_flushFunc + WORD_SIZE)
#define SleepState_size			(SleepState_end - SleepState_begin)

#ifndef __ASSEMBLY__

#ifdef __KERNEL__
struct intc_regs {
	unsigned int		icu_int_conf[63];
	unsigned int		icu_fiq_sel_int_num;
	unsigned int		icu_irq_sel_int_num;
	unsigned int		icu_gbl_irq_msk;
	unsigned int		icu_dma_int_msk;
	unsigned int		icu_dma_int_status;
	unsigned int		icu_int_status_0;
	unsigned int		icu_int_status_1;
	unsigned int		icu_ddr_arm_l2_int_msk;
	unsigned int		icu_ddr_arm_l2_int_status;
};

struct mpmu_regs {
	unsigned int		fccr;
	unsigned int		pocr;
	unsigned int		posr;
	unsigned int		succr;
	unsigned int		ohcr;
	unsigned int		gpcr;
	unsigned int		pll2cr;
	unsigned int		sccr;
	unsigned int		pll1_reg1;
	unsigned int		pll1_reg2;
	unsigned int		pll1_ssc;
	unsigned int		pll2_reg1;
	unsigned int		pll2_reg2;
	unsigned int		pll2_ssc;
	unsigned int		ts;
	unsigned int		wdtpcr;
	unsigned int		apcr;
	unsigned int		apsr;
	unsigned int		aprr;
	unsigned int		acgr;
	unsigned int		arsr;
	unsigned int		awucrs;
	unsigned int		awucrm;
};

struct apbclk_regs {
	unsigned int		uart0;
	unsigned int		uart1;
	unsigned int		gpio;
	unsigned int		pwm0;
	unsigned int		pwm1;
	unsigned int		pwm2;
	unsigned int		pwm3;
	unsigned int		rtc;
	unsigned int		twsi0;
	unsigned int		twsi1;
	unsigned int		kpc;
	unsigned int		timers;
	unsigned int		aib;
	unsigned int		sw_jtag;
	unsigned int		timer1;
	unsigned int		onewire;
	unsigned int		asfar;
	unsigned int		assar;
	unsigned int		uart2;
	unsigned int		timer2;
	unsigned int		ac97;
	unsigned int		ssp0;
	unsigned int		ssp1;
	unsigned int		ssp2;
	unsigned int		ssp3;
	unsigned int		ssp4;
};

struct apmu_regs {
	unsigned int		ccr;
	unsigned int		ccsr;
	unsigned int		fc_timer;
	unsigned int		idle_cfg;
	unsigned int		lcd_clk_res_ctrl;
	unsigned int		ccic_clk_res_ctrl;
	unsigned int		sdh0_clk_res_ctrl;
	unsigned int		sdh1_clk_res_ctrl;
	unsigned int		usb_clk_res_ctrl;
	unsigned int		nfc_clk_res_ctrl;
	unsigned int		dma_clk_res_ctrl;
	unsigned int		bus_clk_res_ctrl;
	unsigned int		wake_clk;
	unsigned int		core_status;
	unsigned int		res_frm_slp_clr;
	unsigned int		imr;
	unsigned int		irwc;
	unsigned int		isr;
	unsigned int		dtc_clk_res_ctrl;
	unsigned int		mc_hw_slp_type;
	unsigned int		mc_slp_req;
	unsigned int		mc_sw_slp_type;
	unsigned int		pll_sel_status;
	unsigned int		gc_clk_res_ctrl;
	unsigned int		smc_clk_res_ctrl;
	unsigned int		xd_clk_res_ctrl;
	unsigned int		sdh2_clk_res_ctrl;
	unsigned int		sdh3_clk_res_ctrl;
	unsigned int		cf_clk_res_ctrl;
	unsigned int		msp_clk_res_ctrl;
	unsigned int		cmu_clk_res_ctrl;
	unsigned int		mfu_clk_res_ctrl;
};

#define MAX_MFP_PINS 131

struct mfp_regs {
	unsigned int		mfp[MAX_MFP_PINS];
};

struct gpio_regs {
	unsigned int		gpdr[4];
	unsigned int		grer[4];
	unsigned int		gfer[4];
	unsigned int		gedr[4];
	unsigned int		gapmask[4];

};

struct ciu_regs {
	unsigned int		chip_id;
	unsigned int		cpu_conf;
	unsigned int		cpu_sram_spd;
	unsigned int		cpu_l2c_sram_spd;
	unsigned int		mcb_conf;
	unsigned int		sys_boot_cntrl;
	unsigned int		sw_branch_addr;
	unsigned int		perf_count0_cntrl;
	unsigned int		perf_count1_cntrl;
	unsigned int		perf_count2_cntrl;
	unsigned int		perf_count0;
	unsigned int		perf_count1;
	unsigned int		perf_count2;
	unsigned int		mc_conf;
	unsigned int		mcb_sram_spd;
	unsigned int		axi_sram_spd;
};

struct axifab_regs {
	unsigned int		timeout;
	unsigned int		timeout_status;
	unsigned int		port[22];
};

struct smc_regs {
	unsigned int		msc0;
	unsigned int		msc1;
	unsigned int		sxcnfg0;
	unsigned int		sxcnfg1;
	unsigned int		memclkcfg;
	unsigned int		csdficfg0;
	unsigned int		csdficfg1;
	unsigned int		clk_ret_del;
	unsigned int		adv_ret_del;
	unsigned int		csadrmap0;
	unsigned int		csadrmap1;
	unsigned int		we_ap0;
	unsigned int		we_ap1;
	unsigned int		oe_ap0;
	unsigned int		oe_ap1;
	unsigned int		adv_ap0;
	unsigned int		adv_ap1;
};

struct squ_regs {
	unsigned int		ctrl_0;
	unsigned int		ctrl_1;
	unsigned int		ctrl_2;
	unsigned int		fmbist_ctrl_0;
	unsigned int		fmbist_ctrl_1;
	unsigned int		fmbist_status_0;
	unsigned int		rsvd;
	unsigned int		perf_count_cntrl;
	unsigned int		perf_count_s1;
	unsigned int		perf_count_s4;
	unsigned int		perf_count_s8;
	unsigned int		cam_ent_bank0;
	unsigned int		cam_ent_bank1;
	unsigned int		cam_ent_bank2;
	unsigned int		cam_ent_bank3;
	unsigned int		cam_ent_bank4;
	unsigned int		cam_ent_bank5;
	unsigned int		logger_ent;
	unsigned int		chan_0_byte_cnt;
	unsigned int		chan_1_byte_cnt;
	unsigned int		chan_0_src_addr;
	unsigned int		chan_1_src_addr;
	unsigned int		chan_0_dest_addr;
	unsigned int		chan_1_dest_addr;
	unsigned int		chan_0_next_desc_ptr;
	unsigned int		chan_1_next_desc_ptr;
	unsigned int		chan_0_ctrl;
	unsigned int		chan_1_ctrl;
	unsigned int		chan_pri;
	unsigned int		chan_0_curr_desc_ptr;
	unsigned int		chan_1_curr_desc_ptr;
	unsigned int		chan_0_int_mask;
	unsigned int		chan_1_int_mask;
	unsigned int		chan_0_int_rst_sel;
	unsigned int		chan_1_int_rst_sel;
	unsigned int		chan_0_int_status;
	unsigned int		chan_1_int_status;
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
	void (*flushFunc)(void);
};

struct pxa168_pm_regs {
	/* It's used to save core registers. */
	struct pm_save_data	pm_data;
	struct mfp_regs		mfp;
	struct gpio_regs	gpio;
	struct intc_regs	intc;
	struct apmu_regs	apmu;
	struct mpmu_regs	mpmu;
	struct apbclk_regs	apbclk;
	struct ciu_regs		ciu;
	struct squ_regs		squ;
	struct axifab_regs	axifab;
	struct smc_regs		smc;
	/* It's the address of DDR that stores key information.
	 * Two words are used from the address.
	 */
	void *data_pool;
	unsigned int word0;
	unsigned int word1;
	unsigned int word2;

};


enum {
	POWER_MODE_ACTIVE = 0,
	POWER_MODE_CORE_INTIDLE,
	POWER_MODE_CORE_EXTIDLE,
	POWER_MODE_APPS_IDLE,
	POWER_MODE_APPS_SLEEP,
	POWER_MODE_SYS_SLEEP,
	POWER_MODE_HIBERNATE,
	POWER_MODE_UDR,
};

enum {
	IDLE_ACTIVE = 0,
	IDLE_CORE_INTIDLE = 1,
	IDLE_CORE_EXTIDLE = 2,
	IDLE_APPS_IDLE = 4,
	IDLE_APPS_SLEEP = 8,
	IDLE_SYS_SLEEP = 16,
};

extern int enable_deepidle;
extern struct kobject *power_kobj;
extern void pxa168_cpu_sleep(unsigned int, unsigned int);
extern void pxa168_cpu_resume(void);
extern void pxa168_pm_swi(void);
extern void pxa168_pm_enter_lowpower_mode(int state);
extern void set_idle_op(int, int);
extern void mspm_idle_load(void);
extern void mspm_idle_clean(void);
extern unsigned int read_timer(void);
#endif
#endif

#endif

