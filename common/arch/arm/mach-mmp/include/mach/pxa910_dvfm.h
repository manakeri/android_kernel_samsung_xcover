/*
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html
 *
 * (C) Copyright 2007 Marvell International Ltd.
 * All Rights Reserved
 */

#ifndef PXA910_DVFM_H
#define PXA910_DVFM_H

#include <mach/dvfm.h>

#define OP_NAME_LEN		16

struct pxa910_md_opt {
	int	power_mode;
	int	vcc_core;	/* core voltage */
	int	pclk;		/* core clock */
	int	pdclk;		/* DDR interface clock */
	int	baclk;		/* bus interface clock */
	int	xpclk;		/* L2 cache interface clock */
	int	dclk;		/* DDR clock */
	int	aclk;		/* bus clock */
	int	cp_pclk;
	int	cp_pdclk;
	int	cp_baclk;
	int	cp_xpclk;
	int	cp_clk_src;
	int	ap_clk_src;
	int	ddr_clk_src;
	int	axi_clk_src;
	int	gc_clk_src;
	int	pll2freq;
	int	lpj;
	char	name[OP_NAME_LEN];
};

typedef union {
	struct {
		unsigned int pll1fbd:		9;
		unsigned int pll1refd:		5;
		unsigned int pll1cen:		1;
		unsigned int mfc:		1;
		unsigned int reserved0:		1;
		unsigned int gcaclksel:		2;
		unsigned int axiclksel0:	1;
		unsigned int reserved1:		3;
		unsigned int ddrclksel:		2;
		unsigned int axiclksel1:	1;
		unsigned int seaclksel:		3;
		unsigned int mohclksel:		3;
	} b;
	unsigned int v;
} pmum_fccr;

typedef union {
	struct {
		unsigned int reserved0:		6;
		unsigned int reserved1:		2;
		unsigned int en:		1;
		unsigned int ctrl:		1;
		unsigned int pll2fbd:		9;
		unsigned int pll2refd:		5;
		unsigned int reserved2:		8;
	} b;
	unsigned int v;
} pmum_pll2cr;

typedef union {
	struct {
		unsigned int cpclksel:		2;
		unsigned int apclksel:		2;
		unsigned int ddrclksel:		2;
		unsigned int axiclksel:		2;
		unsigned int gcaclksel:		2;
		unsigned int reserved0:		22;
	} b;
	unsigned int v;
} pmua_pllsel;

typedef union {
	struct {
		unsigned int core_clk_div:	3;
		unsigned int bus_mc_clk_div:	3;
		unsigned int biu_clk_div:	3;
		unsigned int xp_clk_div:	3;
		unsigned int ddr_clk_div:	3;
		unsigned int bus_clk_div:	3;
		unsigned int async1:		1;
		unsigned int async2:		1;
		unsigned int async3:		1;
		unsigned int async3_1:		1;
		unsigned int async4:		1;
		unsigned int async5:		1;
		unsigned int core_freq_chg_req:	1;
		unsigned int ddr_freq_chg_req:	1;
		unsigned int bus_freq_chg_req:	1;
		unsigned int core_allow_spd_chg:1;
		unsigned int core_dyn_fc:	1;
		unsigned int dclk_dyn_fc:	1;
		unsigned int aclk_dyn_fc:	1;
		unsigned int core_rd_st_clear:	1;
	} b;
	unsigned int v;
} pmua_cc;

typedef union {
	struct {
		unsigned int core_clk_div:	3;
		unsigned int bus_mc_clk_div:	3;
		unsigned int biu_clk_div:	3;
		unsigned int xp_clk_div:	3;
		unsigned int ddr_clk_div:	3;
		unsigned int bus_clk_div:	3;
		unsigned int async1:		1;
		unsigned int async2:		1;
		unsigned int async3:		1;
		unsigned int async3_1:		1;
		unsigned int async4:		1;
		unsigned int async5:		1;
		unsigned int sea_rd_status:	1;
		unsigned int moh_rd_status:	1;
		unsigned int cp_fc_done:	1;
		unsigned int ap_fc_done:	1;
		unsigned int dclk_fc_done:	1;
		unsigned int aclk_fc_done:	1;
		unsigned int reserved:		2;
	} b;
	unsigned int v;
} pmua_dm_cc;

#define CLK_SRC_VCTCXO		(1u<<0)
#define CLK_SRC_PLL1_312	(1u<<1)
#define CLK_SRC_PLL1_624	(1u<<2)
#define CLK_SRC_PLL2		(1u<<3)

#define BIT_0 (1 << 0)
#define BIT_1 (1 << 1)
#define BIT_2 (1 << 2)
#define BIT_3 (1 << 3)
#define BIT_4 (1 << 4)
#define BIT_5 (1 << 5)
#define BIT_6 (1 << 6)
#define BIT_7 (1 << 7)
#define BIT_8 (1 << 8)
#define BIT_9 (1 << 9)
#define BIT_10 (1 << 10)
#define BIT_11 (1 << 11)
#define BIT_12 (1 << 12)
#define BIT_13 (1 << 13)
#define BIT_14 (1 << 14)
#define BIT_15 (1 << 15)
#define BIT_16 (1 << 16)
#define BIT_17 (1 << 17)
#define BIT_18 (1 << 18)
#define BIT_19 (1 << 19)
#define BIT_20 (1 << 20)
#define BIT_21 (1 << 21)
#define BIT_22 (1 << 22)
#define BIT_23 (1 << 23)
#define BIT_24 (1 << 24)
#define BIT_25 (1 << 25)
#define BIT_26 (1 << 26)
#define BIT_27 (1 << 27)
#define BIT_28 (1 << 28)
#define BIT_29 (1 << 29)
#define BIT_30 (1 << 30)
#define BIT_31 ((unsigned)1 << 31)

#define SHIFT0(Val)  (Val)
#define SHIFT1(Val)  ((Val) << 1)
#define SHIFT2(Val)  ((Val) << 2)
#define SHIFT3(Val)  ((Val) << 3)
#define SHIFT4(Val)  ((Val) << 4)
#define SHIFT5(Val)  ((Val) << 5)
#define SHIFT6(Val)  ((Val) << 6)
#define SHIFT7(Val)  ((Val) << 7)
#define SHIFT8(Val)  ((Val) << 8)
#define SHIFT9(Val)  ((Val) << 9)
#define SHIFT10(Val) ((Val) << 10)
#define SHIFT11(Val) ((Val) << 11)
#define SHIFT12(Val) ((Val) << 12)
#define SHIFT13(Val) ((Val) << 13)
#define SHIFT14(Val) ((Val) << 14)
#define SHIFT15(Val) ((Val) << 15)
#define SHIFT16(Val) ((Val) << 16)
#define SHIFT17(Val) ((Val) << 17)
#define SHIFT18(Val) ((Val) << 18)
#define SHIFT19(Val) ((Val) << 19)
#define SHIFT20(Val) ((Val) << 20)
#define SHIFT21(Val) ((Val) << 21)
#define SHIFT22(Val) ((Val) << 22)
#define SHIFT23(Val) ((Val) << 23)
#define SHIFT24(Val) ((Val) << 24)
#define SHIFT25(Val) ((Val) << 25)
#define SHIFT26(Val) ((Val) << 26)
#define SHIFT27(Val) ((Val) << 27)
#define SHIFT28(Val) ((Val) << 28)
#define SHIFT29(Val) ((Val) << 29)
#define SHIFT30(Val) ((Val) << 30)
#define SHIFT31(Val) ((Val) << 31)

/*
 * pmua registers and bits definition
 */
#define	CC_SEA_OFF				0x0000
#define	CC_MOH_OFF				0x0004
#define	DM_CC_SEA_OFF				0x0008
#define	DM_CC_MOH_OFF				0x000C
#define	MOH_IMR_OFF				0x0098
#define	MOH_ISR_OFF				0x00A0

#define	PMUA_CC_SEA_SEA_RD_ST_CLEAR		BIT_31
#define	PMUA_CC_SEA_ACLK_DYN_FC			BIT_30
#define	PMUA_CC_SEA_DCLK_DYN_FC			BIT_29
#define	PMUA_CC_SEA_CORE_DYN_FC			BIT_28
#define	PMUA_CC_SEA_SEA_ALLOW_SPD_CHG		BIT_27
#define	PMUA_CC_SEA_BUS_FREQ_CHG_REQ		BIT_26
#define	PMUA_CC_SEA_DDR_FREQ_CHG_REQ		BIT_25
#define	PMUA_CC_SEA_SEA_FREQ_CHG_REQ		BIT_24
#define	PMUA_CC_SEA_ASYNC5			BIT_23
#define	PMUA_CC_SEA_ASYNC4			BIT_22
#define	PMUA_CC_SEA_ASYNC3_1			BIT_21
#define	PMUA_CC_SEA_ASYNC3			BIT_20
#define	PMUA_CC_SEA_ASYNC2			BIT_19
#define	PMUA_CC_SEA_ASYNC1			BIT_18
#define	PMUA_CC_SEA_BUS_CLK_DIV_MSK		SHIFT15(0x7)
#define	PMUA_CC_SEA_BUS_CLK_DIV_BASE		15
#define	PMUA_CC_SEA_DDR_CLK_DIV_MSK		SHIFT12(0x7)
#define	PMUA_CC_SEA_DDR_CLK_DIV_BASE		12
#define	PMUA_CC_SEA_XP_CLK_DIV_MSK		SHIFT9(0x7)
#define	PMUA_CC_SEA_XP_CLK_DIV_BASE		9
#define	PMUA_CC_SEA_BIU_CLK_DIV_MSK		SHIFT6(0x7)
#define	PMUA_CC_SEA_BIU_CLK_DIV_BASE		6
#define	PMUA_CC_SEA_BUS_MC_CLK_DIV_MSK		SHIFT3(0x7)
#define	PMUA_CC_SEA_BUS_MC_CLK_DIV_BASE		3
#define	PMUA_CC_SEA_CORE_CLK_DIV_MSK		SHIFT0(0x7)
#define	PMUA_CC_SEA_CORE_CLK_DIV_BASE		0

#define	PMUA_CC_MOH_MOH_RD_ST_CLEAR		BIT_31
#define	PMUA_CC_MOH_ACLK_DYN_FC			BIT_30
#define	PMUA_CC_MOH_DCLK_DYN_FC			BIT_29
#define	PMUA_CC_MOH_CORE_DYN_FC			BIT_28
#define	PMUA_CC_MOH_MOH_ALLOW_SPD_CHG		BIT_27
#define	PMUA_CC_MOH_BUS_FREQ_CHG_REQ		BIT_26
#define	PMUA_CC_MOH_DDR_FREQ_CHG_REQ		BIT_25
#define	PMUA_CC_MOH_MOH_FREQ_CHG_REQ		BIT_24
#define	PMUA_CC_MOH_ASYNC5			BIT_23
#define	PMUA_CC_MOH_ASYNC4			BIT_22
#define	PMUA_CC_MOH_ASYNC3_1			BIT_21
#define	PMUA_CC_MOH_ASYNC3			BIT_20
#define	PMUA_CC_MOH_ASYNC2			BIT_19
#define	PMUA_CC_MOH_ASYNC1			BIT_18
#define	PMUA_CC_MOH_BUS_2_CLK_DIV_BASE		18
#define	PMUA_CC_MOH_BUS_CLK_DIV_MSK		SHIFT15(0x7)
#define	PMUA_CC_MOH_BUS_CLK_DIV_BASE		15
#define	PMUA_CC_MOH_DDR_CLK_DIV_MSK		SHIFT12(0x7)
#define	PMUA_CC_MOH_DDR_CLK_DIV_BASE		12
#define	PMUA_CC_MOH_XP_CLK_DIV_MSK		SHIFT9(0x7)
#define	PMUA_CC_MOH_XP_CLK_DIV_BASE		9
#define	PMUA_CC_MOH_BIU_CLK_DIV_MSK		SHIFT6(0x7)
#define	PMUA_CC_MOH_BIU_CLK_DIV_BASE		6
#define	PMUA_CC_MOH_BUS_MC_CLK_DIV_MSK		SHIFT3(0x7)
#define	PMUA_CC_MOH_BUS_MC_CLK_DIV_BASE		3
#define	PMUA_CC_MOH_CORE_CLK_DIV_MSK		SHIFT0(0x7)
#define	PMUA_CC_MOH_CORE_CLK_DIV_BASE		0

#define	PMUA_DM_CC_SEA_MOH_RD_STATUS		BIT_25
#define	PMUA_DM_CC_SEA_SEA_RD_STATUS		BIT_24
#define	PMUA_DM_CC_SEA_ASYNC5			BIT_23
#define	PMUA_DM_CC_SEA_ASYNC4			BIT_22
#define	PMUA_DM_CC_SEA_ASYNC3_1			BIT_21
#define	PMUA_DM_CC_SEA_ASYNC3			BIT_20
#define	PMUA_DM_CC_SEA_ASYNC2			BIT_19
#define	PMUA_DM_CC_SEA_ASYNC1			BIT_18
#define	PMUA_DM_CC_SEA_BUS_CLK_DIV_MSK		SHIFT15(0x7)
#define	PMUA_DM_CC_SEA_BUS_CLK_DIV_BASE		15
#define	PMUA_DM_CC_SEA_DDR_CLK_DIV_MSK		SHIFT12(0x7)
#define	PMUA_DM_CC_SEA_DDR_CLK_DIV_BASE		12
#define	PMUA_DM_CC_SEA_XP_CLK_DIV_MSK		SHIFT9(0x7)
#define	PMUA_DM_CC_SEA_XP_CLK_DIV_BASE		9
#define	PMUA_DM_CC_SEA_BIU_CLK_DIV_MSK		SHIFT6(0x7)
#define	PMUA_DM_CC_SEA_BIU_CLK_DIV_BASE		6
#define	PMUA_DM_CC_SEA_BUS_MC_CLK_DIV_MSK	SHIFT3(0x7)
#define	PMUA_DM_CC_SEA_BUS_MC_CLK_DIV_BASE	3
#define	PMUA_DM_CC_SEA_CORE_CLK_DIV_MSK		SHIFT0(0x7)
#define	PMUA_DM_CC_SEA_CORE_CLK_DIV_BASE	0

#define	PMUA_DM_CC_MOH_MOH_RD_STATUS		BIT_25
#define	PMUA_DM_CC_MOH_SEA_RD_STATUS		BIT_24
#define	PMUA_DM_CC_MOH_ASYNC5			BIT_23
#define	PMUA_DM_CC_MOH_ASYNC4			BIT_22
#define	PMUA_DM_CC_MOH_ASYNC3_1			BIT_21
#define	PMUA_DM_CC_MOH_ASYNC3			BIT_20
#define	PMUA_DM_CC_MOH_ASYNC2			BIT_19
#define	PMUA_DM_CC_MOH_ASYNC1			BIT_18
#define	PMUA_DM_CC_MOH_BUS_CLK_DIV_MSK		SHIFT15(0x7)
#define	PMUA_DM_CC_MOH_BUS_CLK_DIV_BASE		15
#define	PMUA_DM_CC_MOH_DDR_CLK_DIV_MSK		SHIFT12(0x7)
#define	PMUA_DM_CC_MOH_DDR_CLK_DIV_BASE		12
#define	PMUA_DM_CC_MOH_XP_CLK_DIV_MSK		SHIFT9(0x7)
#define	PMUA_DM_CC_MOH_XP_CLK_DIV_BASE		9
#define	PMUA_DM_CC_MOH_BIU_CLK_DIV_MSK		SHIFT6(0x7)
#define	PMUA_DM_CC_MOH_BIU_CLK_DIV_BASE		6
#define	PMUA_DM_CC_MOH_BUS_MC_CLK_DIV_MSK	SHIFT3(0x7)
#define	PMUA_DM_CC_MOH_BUS_MC_CLK_DIV_BASE	3
#define	PMUA_DM_CC_MOH_CORE_CLK_DIV_MSK		SHIFT0(0x7)
#define	PMUA_DM_CC_MOH_CORE_CLK_DIV_BASE	0

#define	PMUA_MOH_IMR_MOH_FC_INTR_MASK		BIT_1
#define	PMUA_MOH_IMR_SEA_FC_INTR_MASK		BIT_0

#define	PMUA_MOH_ISR_MOH_FC_ISR			BIT_1
#define	PMUA_MOH_ISR_SEA_FC_ISR			BIT_0

#define PMUA_MOH_DIS_MC_SW_REQ			BIT_21
#define PMUA_MOH_MC_WAKE_EN			BIT_20
#define PMUA_MOH_SRAM_PWRDWN			BIT_6
#define PMUA_MOH_PWRDWN				BIT_5
#define PMUA_MOH_IDLE				BIT_1

/*
 * pmum registers and bits definition
 */
#define	FCCR_OFF				0x0008

#define	PMUM_FCCR_MOHCLKSEL_MSK			SHIFT29(0x7)
#define	PMUM_FCCR_MOHCLKSEL_BASE		29
#define	PMUM_FCCR_SEAGCLKSEL_MSK		SHIFT26(0x7)
#define	PMUM_FCCR_SEAGCLKSEL_BASE		26
#define	PMUM_FCCR_AXICLKSEL_MSK			SHIFT23(0x7)
#define	PMUM_FCCR_AXICLKSEL_BASE		23
#define	PMUM_FCCR_MFC				BIT_15
#define	PMUM_FCCR_PLL1CEN			BIT_14
#define	PMUM_FCCR_PLL1REFD_MSK			SHIFT9(0x1f)
#define	PMUM_FCCR_PLL1REFD_BASE			9
#define	PMUM_FCCR_PLL1FBD_MSK			SHIFT0(0x1ff)
#define	PMUM_FCCR_PLL1FBD_BASE			0

#define PMUM_AXISD				BIT_31
#define PMUM_DSPSD				BIT_30
#define PMUM_SLPEN				BIT_29
#define PMUM_DTCMSD				BIT_28
#define PMUM_DDRCORSD				BIT_27
#define PMUM_APBSD				BIT_26
#define PMUM_BBSD				BIT_25
#define PMUM_INTCLR				BIT_24
#define PMUM_SLPWP0				BIT_23
#define PMUM_SLPWP1				BIT_22
#define PMUM_SLPWP2				BIT_21
#define PMUM_SLPWP3				BIT_20
#define PMUM_VCTCXOSD				BIT_19
#define PMUM_SLPWP4				BIT_18
#define PMUM_SLPWP5				BIT_17
#define PMUM_SLPWP6				BIT_16
#define PMUM_SLPWP7				BIT_15
#define PMUM_MSASLPEN				BIT_14
#define PMUM_STBYEN				BIT_13

#define PMUM_GSM_WAKEUPWMX			BIT_29
#define PMUM_WCDMA_WAKEUPX			BIT_28
#define PMUM_GSM_WAKEUPWM			BIT_27
#define PMUM_WCDMA_WAKEUPWM			BIT_26
#define PMUM_AP_ASYNC_INT			BIT_25
#define PMUM_AP_FULL_IDLE			BIT_24
#define PMUM_SDH1				BIT_23
#define PMUM_SDH2				BIT_22
#define PMUM_KEYPRESS				BIT_21
#define PMUM_TRACKBALL				BIT_20
#define PMUM_NEWROTARY				BIT_19
#define PMUM_WDT				BIT_18
#define PMUM_RTC_ALARM				BIT_17
#define PMUM_CP_TIMER_3				BIT_16
#define PMUM_CP_TIMER_2				BIT_15
#define PMUM_CP_TIMER_1				BIT_14
#define PMUM_AP2_TIMER_3			BIT_13
#define PMUM_AP2_TIMER_2			BIT_12
#define PMUM_AP2_TIMER_1			BIT_11
#define PMUM_AP1_TIMER_3			BIT_10
#define PMUM_AP1_TIMER_2			BIT_9
#define PMUM_AP1_TIMER_1			BIT_8
#define PMUM_WAKEUP7				BIT_7
#define PMUM_WAKEUP6				BIT_6
#define PMUM_WAKEUP5				BIT_5
#define PMUM_WAKEUP4				BIT_4
#define PMUM_WAKEUP3				BIT_3
#define PMUM_WAKEUP2				BIT_2
#define PMUM_WAKEUP1				BIT_1
#define PMUM_WAKEUP0				BIT_0

#endif
