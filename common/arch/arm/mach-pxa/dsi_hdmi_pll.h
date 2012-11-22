/*
 * linux/arch/arm/mach-pxa/dsi_hdmi_pll.h
 *
 *  Copyright (C) 2011 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_ARCH_DSI_HDMI_PLL_H
#define __ASM_ARCH_DSI_HDMI_PLL_H

#define DSI_BASE_ADDRESS			0x42404154L
#define DSI_BASE_ADDRESS_NEVO			0x42404160L
#define HDMI_BASE_ADDRESS			0x424041B0L

#define ADDRESS_SPACE				0x10

#define DSI_REGS_MEM_SIZE			0x18
#define DSI_REG1_OFFSET				0x0
#define DSI_REG2_OFFSET				0x4

/*DSI_REG1 bits*/
#define DSI_REG1_DSIx_PLL_LOCK_SEL		(0x1u<<0)
#define DSI_REG1_DSIx_PLL_ON			(0x1u<<1)

/*DSI_REG2 bits*/
#define DSI_REG2_CK_SEL(n)			((n)<<0)
#define DSI_REG2_OUTDIV(n)			((n)<<13)
#define DSI_REG2_FBDIV(n)			((n)<<17)
#define DSI_REG2_REFDIV(n)			((n)<<26)
#define DSI_REG2_DEFAULT			0x178A

/*DSI_NEVO_PU bits*/
#define DSI_NEVO_PU_PLL_ON			(0x1u<<0)

/*DSI_NEVO_CONFIG bits*/
#define DSI_NEVO_CONFIG_VCODIV_SEL_SE(n)	((n)<<0)
#define DSI_NEVO_CONFIG_VCO_VRNG(n)		((n)<<4)
#define DSI_NEVO_CONFIG_KVCO(n)			((n)<<7)
#define DSI_NEVO_CONFIG_OUTDIV(n)		((n)<<11)
#define DSI_NEVO_CONFIG_FBDIV(n)		((n)<<12)
#define DSI_NEVO_CONFIG_REFDIV(n)		((n)<<21)
#define DSI_NEVO_CONFIG_PLL_READY		(0x1u<<31)

/* HDMI_PLL_PU bits */
#define HDMI_PLL_PWR_PLL_ON			(0x1u<<0)

/* HDMI_PLL_CONFIG1 bits */
#define HDMI_PLL_CONFIG1_INTPI(n)		((n)<<0)
#define HDMI_PLL_CONFIG1_KVCO(n)		((n)<<4)
#define HDMI_PLL_CONFIG1_POSTDIV_SEL(n)		((n)<<8)
#define HDMI_PLL_CONFIG1_FBDIV(n)		((n)<<11)
#define HDMI_PLL_CONFIG1_REFDIV(n)		((n)<<20)
#define HDMI_PLL_CONFIG1_EN_HDMI		(0x1u<<25)
#define HDMI_PLL_CONFIG1_EN_PANEL		(0x1u<<26)
#define HDMI_PLL_CONFIG1_VPLL_CAL_START		(0x1u<<28)
#define HDMI_PLL_CONFIG1_CLOCK_READY		(0x1u<<31)

/* HDMI_PLL_CONFIG2 bits */
#define HDMI_PLL_CONFIG2_FREQ_OFFSET_ADJ(n)	((n)<<0)
#define HDMI_PLL_CONFIG2_RESET_OFFSET(n)	((n)<<17)
#define HDMI_PLL_CONFIG2_RESET_INTP_EXT(n)	((n)<<18)
#define HDMI_PLL_CONFIG2_MODE(n)		((n)<<19)

/* HDMI_PLL_CONFIG3 bits */
#define HDMI_PLL_CONFIG3_FREQ_OFFSET_INNER(n) ((n)<<0)

/* HDMI_PLL_DEBUG1_REG bits */
#define HDMI_PLL_DEBUG1_REG_CLK_DET_EN(n)	((n)<<0)
#define HDMI_PLL_DEBUG1_REG_VPLL_CALCLK_DIV(n)	((n)<<1)
#define HDMI_PLL_DEBUG1_REG_VTH_VPLL_CAL(n)	((n)<<3)
#define HDMI_PLL_DEBUG1_REG_KVCO_EXT_EN(n)	((n)<<5)
#define HDMI_PLL_DEBUG1_REG_CTUNE(n)		((n)<<6)
#define HDMI_PLL_DEBUG1_REG_ICP(n)		((n)<<9)
#define HDMI_PLL_DEBUG1_REG_VREG_IVREF(n)	((n)<<13)
#define HDMI_PLL_DEBUG1_REG_VDDL(n)		((n)<<15)
#define HDMI_PLL_DEBUG1_REG_VDDM(n)		((n)<<19)
#define HDMI_PLL_DEBUG1_REG_RESET(n)		((n)<<21)

/* HDMI_PLL_DEBUG2_REG bits */
#define HDMI_PLL_DEBUG2_REG_RESERVE_IN(n)	((n)<<0)
#define HDMI_PLL_DEBUG2_REG_TEST_MON(n)		((n)<<4)
#define HDMI_PLL_DEBUG2_REG_CLKOUT_TST_EN(n)	((n)<<8)
#define HDMI_PLL_DEBUG2_REG_PLL_CTRL_RSTN(n)	((n)<<9)
#define HDMI_PLL_DEBUG2_REG_PANEL_CG_OVRD(n)	((n)<<10)

/* HDMI_PHY_CTL1_REG bits */
#define HDMI_PHY_CTL1_REG_RESET_TX_OFF		0x1
#define HDMI_PHY_CTL1_REG_RESET_TX_ON		0xFFFFFFFE
#define HDMI_PHY_CTL1_REG_PD_IREF(n)		((n)<<1)
#define HDMI_PHY_CTL1_REG_PD_TX(n)		((n)<<2)
#define HDMI_PHY_CTL1_REG_SYNC(n)		((n)<<6)
#define HDMI_PHY_CTL1_REG_EAMP(n)		((n)<<7)
#define HDMI_PHY_CTL1_REG_DAMP(n)		((n)<<19)
#define HDMI_PHY_CTL1_REG_AUX(n)		((n)<<31)

/* HDMI_PHY_CTL2_REG bits */
#define HDMI_PHY_CTL2_REG_IDRV(n)		((n)<<0)
#define HDMI_PHY_CTL2_REG_TXDRVX2(n)		((n)<<16)
#define HDMI_PHY_CTL2_REG_CP(n)			((n)<<20)
#define HDMI_PHY_CTL2_REG_PU_ARC(n)		((n)<<28)
#define HDMI_PHY_CTL2_REG_ENABLE_ARC(n)		((n)<<29)
#define HDMI_PHY_CTL2_REG_COMMON_MODE_ARC(n)	((n)<<30)
#define HDMI_PHY_CTL2_REG_CLOCK_IN_ARC(n)	((n)<<31)

/* HDMI_PHY_CTL3_REG bits */
#define HDMI_PHY_CTL3_REG_POLSWAP_TX(n)		((n)<<0)
#define HDMI_PHY_CTL3_REG_INV_CK20T(n)		((n)<<4)
#define HDMI_PHY_CTL3_REG_HYST0_ARC(n)		((n)<<8)
#define HDMI_PHY_CTL3_REG_HYST1_ARC(n)		((n)<<9)

/* HDMI_PLL_CONFIG3 bits */
#define HDMI_PLL_CONFIG3_FREQ_OFFSET_INNER(n)	((n)<<0)

/* HDMI_PLL_DEBUG2 bits */
#define HDMI_PLL_DEBUG2_PLL_CTRL_RSTN		(0x1<<9)

struct DSIRegisters {
	u32 DSI_REG1;
	u32 DSI_REG2;
	u32 DSI_REG3;
};

struct DSI_PLL_PARAMETERS_NEVO {
	u32 refdiv;
	u32 fbdiv;
	u32 vcodiv_sel_se;
	u32 vco_vrng;
	u32 kvco_range;
};

struct DSIRegisters_Nevo {
	u32 DSI_NEVO_DELAY;
	u32 DSI_NEVO_PU;
	u32 DSI_NEVO_CONFIG;
	u32 DSI_NEVO_DEBUG;
};

struct HDMI_PLL_PARAMETERS {
	u32 intpi;
	u32 kvco;
	u32 postdiv_sel;
	u32 fbdiv;
	u32 refdiv;
	u32 freq_offset_inner;
};

struct HDMIRegisters {
	u32 HDMI_PLL_DELAY;
	u32 HDMI_PLL_PU;
	u32 HDMI_PLL_CONFIG1;
	u32 HDMI_PLL_CONFIG2;
	u32 HDMI_PLL_CONFIG3;
	u32 HDMI_PLL_DEBUG1;
	u32 HDMI_PLL_DEBUG2;
	u32 HDMI_PHY_CTL1;
	u32 HDMI_PHY_CTL2;
	u32 HDMI_PHY_CTL3;
	u32 HDMI_PHY_DEBUG1;
	u32 HDMI_PHY_DEBUG2;
};

static inline struct DSIRegisters *get_dsi_pll(int conv)
{
	struct DSIRegisters *p_Regs;
	static void *MMCUbase;

	if (!MMCUbase)
		MMCUbase = ioremap_nocache(DSI_BASE_ADDRESS,
					   DSI_REGS_MEM_SIZE);

	p_Regs = (struct DSIRegisters *)
		(MMCUbase + conv * ADDRESS_SPACE);

	return p_Regs;
}

static inline struct DSIRegisters_Nevo *get_dsi_pll_nevo(void)
{
	static struct DSIRegisters_Nevo *MMCUbase;

	if (NULL == MMCUbase)
		MMCUbase = (struct DSIRegisters_Nevo *)
			ioremap_nocache(DSI_BASE_ADDRESS_NEVO,
				DSI_REGS_MEM_SIZE);

	return MMCUbase;
}

static inline struct HDMIRegisters *get_ihdmi_pll(void)
{
	static struct HDMIRegisters *p_Regs;

	/* Calc  HDMI address */
	if (NULL == p_Regs)
		p_Regs = (struct HDMIRegisters *)
			ioremap_nocache(HDMI_BASE_ADDRESS,
				sizeof(struct HDMIRegisters));

	return p_Regs;
}

/*PLL formulas
  DSIbitclock  = PLL(CKOUT)/OUTDIV
 PLL(CKOUT) = (13MHz FBDIV + 2))/(3 *(CKSEL+1))
 */
static u32 dsi_calc_fbdiv(u16 dsi_clk, u8 outdiv, u8 cksel)
{
	u32 ckout, fbdiv;

	/*calc actual outdiv */
	outdiv = 16 - outdiv;

	/*calc actual PLL(CKOUT) */
	ckout = dsi_clk / outdiv;

	/*Calc fbdiv */
	fbdiv = ((ckout * (3 * (cksel + 1))) / 13) - 2;

	return (u32) fbdiv;
}

static u32 dsi_get_kvco_range(u32 dsi_clk)
{
	if (dsi_clk < 1200)
		return 1;
	else if (dsi_clk < 1360)
		return 1;
	else if (dsi_clk < 1530)
		return 2;
	else if (dsi_clk < 1700)
		return 3;
	else if (dsi_clk < 1900)
		return 4;
	else if (dsi_clk < 2100)
		return 5;
	else if (dsi_clk < 2300)
		return 6;
	else if (dsi_clk < 2400)
		return 7;
	else
		return 1;
}

static u32 dsi_calc_kvco(u32 dsi_clk, u32 *kvco_range, u32 *vcodiv)
{
	int i = 0;
	u32 temp_clk = 0;
	u32 mul_array[] = { 2, 3, 4, 5, 6, 8, 10, 12, 16, 0 };

	/* Try to multiply dsi_clk according to vcodiv values
	   until we get above 1200 */
	while (mul_array[i] != 0) {
		temp_clk = (u32) (dsi_clk * mul_array[i] / 2);
		if (temp_clk >= 1200)
			break;
		i++;
	}

	*vcodiv = i;

	*kvco_range = dsi_get_kvco_range(temp_clk);

	return temp_clk;

}

static u32 dsi_calc_fbdiv_Nevo(u32 refdiv, u32 kvco)
{
	u32 fref = 26;		/* 26Mhz reference clock */
	return (kvco * refdiv) / fref;

}

static int dsi_pll_locked_Nevo(int timeout,
			struct DSIRegisters_Nevo *p_Regs)
{
	/* wait until status bit is set */
	while (!(p_Regs->DSI_NEVO_CONFIG
				& DSI_NEVO_CONFIG_PLL_READY)
			&& --timeout)
		mdelay(1);

	if (timeout <= 0)
		printk(KERN_ERR "hdmi pll not enabled correctly\n");
	return (timeout > 0);
}

static int hdmi_pll_locked(int timeout,
			struct HDMIRegisters *p_Regs)
{
	/* wait until status bit is set */
	while (!(p_Regs->HDMI_PLL_CONFIG1
				& HDMI_PLL_CONFIG1_CLOCK_READY)
			&& --timeout)
		mdelay(1);

	if (timeout <= 0)
		printk(KERN_ERR "hdmi pll not enabled correctly\n");
	return (timeout > 0);
}

#endif
