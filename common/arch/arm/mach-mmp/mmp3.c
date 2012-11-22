/*
 * linux/arch/arm/mach-mmp/mmp3.c
 *
 * code name MMP3
 *
 * Copyright (C) 2009 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/delay.h>

#include <asm/smp_twd.h>
#include <asm/mach/time.h>
#include <asm/hardware/gic.h>
#include <asm/hardware/cache-l2x0.h>

#include <mach/addr-map.h>
#include <mach/regs-apbc.h>
#include <mach/regs-apmu.h>
#include <mach/regs-mpmu.h>
#include <mach/cputype.h>
#include <mach/irqs.h>
#include <mach/gpio.h>
#include <mach/dma.h>
#include <mach/devices.h>
#include <mach/mmp2_dma.h>
#include <mach/soc_vmeta.h>

#include <linux/platform_device.h>

#include <plat/mfp.h>

#include "common.h"
#include "clock.h"

#define MFPR_VIRT_BASE	(APB_VIRT_BASE + 0x1e000)

#define APMASK(i)	(GPIO_REGS_VIRT + BANK_OFF(i) + 0x9c)

static struct mfp_addr_map mmp3_addr_map[] __initdata = {
	MFP_ADDR_X(GPIO0, GPIO58, 0x54),
	MFP_ADDR_X(GPIO59, GPIO73, 0x280),
	MFP_ADDR_X(GPIO74, GPIO101, 0x170),
	MFP_ADDR_X(GPIO102, GPIO103, 0x0),
	MFP_ADDR_X(GPIO115, GPIO122, 0x260),
	MFP_ADDR_X(GPIO124, GPIO141, 0xc),
	MFP_ADDR_X(GPIO143, GPIO151, 0x220),
	MFP_ADDR_X(GPIO152, GPIO153, 0x248),
	MFP_ADDR_X(GPIO154, GPIO155, 0x254),

	MFP_ADDR(GPIO142, 0x8),
	MFP_ADDR(GPIO114, 0x164),
	MFP_ADDR(GPIO123, 0x148),

	MFP_ADDR(GPIO168, 0x1e0),
	MFP_ADDR(GPIO167, 0x1e4),
	MFP_ADDR(GPIO166, 0x1e8),
	MFP_ADDR(GPIO165, 0x1ec),
	MFP_ADDR(GPIO107, 0x1f0),
	MFP_ADDR(GPIO106, 0x1f4),
	MFP_ADDR(GPIO105, 0x1f8),
	MFP_ADDR(GPIO104, 0x1fc),
	MFP_ADDR(GPIO111, 0x200),
	MFP_ADDR(GPIO164, 0x204),
	MFP_ADDR(GPIO163, 0x208),
	MFP_ADDR(GPIO162, 0x20c),
	MFP_ADDR(GPIO161, 0x210),
	MFP_ADDR(GPIO110, 0x214),
	MFP_ADDR(GPIO109, 0x218),
	MFP_ADDR(GPIO108, 0x21c),
	MFP_ADDR(GPIO110, 0x214),
	MFP_ADDR(GPIO112, 0x244),
	MFP_ADDR(GPIO160, 0x250),
	MFP_ADDR(GPIO113, 0x25c),
	/* FIXME: Zx does not have this pin, define here will not impact */
	MFP_ADDR(GPIO171, 0x2c8),

	MFP_ADDR_X(TWSI1_SCL, TWSI1_SDA, 0x140),
	MFP_ADDR_X(TWSI4_SCL, TWSI4_SDA, 0x2bc),
	MFP_ADDR(PMIC_INT, 0x2c4),
	MFP_ADDR(CLK_REQ, 0x160),

	MFP_ADDR_END,
};

void mmp3_clear_pmic_int(void)
{
	unsigned long mfpr_pmic, data;

	mfpr_pmic = APB_VIRT_BASE + 0x1e000 + 0x2c4;
	data = __raw_readl(mfpr_pmic);
	__raw_writel(data | (1 << 6), mfpr_pmic);
	__raw_writel(data, mfpr_pmic);
}

static void __init mmp3_init_gpio(void)
{
	int i;

	/* enable GPIO clock */
	__raw_writel(APBC_APBCLK | APBC_FNCLK, APBC_MMP2_GPIO);

	/* unmask GPIO edge detection for all 6 banks -- APMASKx */
	for (i = 0; i < 6; i++)
		__raw_writel(0xffffffff, APMASK(i));

	pxa_init_gpio(IRQ_MMP3_GPIO, 0, 167, NULL);
}

static void sdhc_clk_enable(struct clk *clk)
{
	uint32_t clk_rst;

	clk_rst  =  __raw_readl(clk->clk_rst);
	clk_rst |= clk->enable_val;
	__raw_writel(clk_rst, clk->clk_rst);
}

static void sdhc_clk_disable(struct clk *clk)
{
	uint32_t clk_rst;

	clk_rst  =  __raw_readl(clk->clk_rst);
	clk_rst &= ~clk->enable_val;
	__raw_writel(clk_rst, clk->clk_rst);
}

struct clkops sdhc_clk_ops = {
	.enable		= sdhc_clk_enable,
	.disable	= sdhc_clk_disable,
};

static void uart_clk_enable(struct clk *clk)
{
	uint32_t clk_rst;

	clk_rst = __raw_readl(clk->clk_rst);
	clk_rst |= APBC_FNCLK;
	__raw_writel(clk_rst, clk->clk_rst);
	mdelay(1);

	clk_rst |= APBC_APBCLK;
	__raw_writel(clk_rst, clk->clk_rst);
	mdelay(1);

	clk_rst &= ~(APBC_RST);
	__raw_writel(clk_rst, clk->clk_rst);
}

static void uart_clk_disable(struct clk *clk)
{
	__raw_writel(0, clk->clk_rst);
	mdelay(1);
}

static int uart_clk_setrate(struct clk *clk, unsigned long val)
{
	uint32_t clk_rst;

	if (val == clk->rate) {
		/* choose vctcxo */
		clk_rst = __raw_readl(clk->clk_rst);
		clk_rst &= ~(APBC_FNCLKSEL(0x7));
		clk_rst |= APBC_FNCLKSEL(0x1);
		__raw_writel(clk_rst, clk->clk_rst);

	} else if (val > clk->rate) {
		/* set m/n for high speed */
		unsigned int numer = 27;
		unsigned int denom = 16;

		/*
		 *      n/d = base_clk/(2*out_clk)
		 *      base_clk = 199.33M, out_clk=199.33*16/27/2=59.06M
		 *      buadrate = clk/(16*divisor)
		 */

		/* Bit(s) PMUM_SUCCR_RSRV_31_29 reserved */
		/* UART Clock Generation Programmable Divider Numerator Value */
#define PMUM_SUCCR_UARTDIVN_MSK                 (0x1fff << 16)
#define PMUM_SUCCR_UARTDIVN_BASE                16
		/* Bit(s) PMUM_SUCCR_RSRV_15_13 reserved */
		/* UART Clock Generation Programmable Divider Denominator Value */
#define PMUM_SUCCR_UARTDIVD_MSK                 (0x1fff)
#define PMUM_SUCCR_UARTDIVD_BASE                0

		clk_rst = __raw_readl(MPMU_SUCCR);
		clk_rst &=
		    ~(PMUM_SUCCR_UARTDIVN_MSK + PMUM_SUCCR_UARTDIVD_MSK);
		clk_rst |=
		    (numer << PMUM_SUCCR_UARTDIVN_BASE) | (denom <<
							   PMUM_SUCCR_UARTDIVD_BASE);
		__raw_writel(clk_rst, MPMU_SUCCR);

		/* choose programmable clk */
		clk_rst = __raw_readl(clk->clk_rst);
		clk_rst &= ~(APBC_FNCLKSEL(0x7));
		__raw_writel(clk_rst, clk->clk_rst);
	}


	return 0;
}

struct clkops uart_clk_ops = {
	.enable = uart_clk_enable,
	.disable = uart_clk_disable,
	.setrate = uart_clk_setrate,
};

static void turn_on_pll3(void)
{
	u32 tmp = __raw_readl(PMUM_PLL3_CTRL2);

	/* set SEL_VCO_CLK_SE in PMUM_PLL3_CTRL2 register*/
	__raw_writel(tmp | 0x00000001, PMUM_PLL3_CTRL2);

	/* PLL3 control register 1 - program VCODIV_SEL_SE = 2,
	 * ICP = 4, KVCO = 5 and VCRNG = 4*/
	__raw_writel(0x05290499, PMUM_PLL3_CTRL1);

	/*MPMU_PLL3CR: Program PLL3 VCO for 2.0Ghz -REFD = 3;*/
	tmp = (__raw_readl(APMU_FSIC3_CLK_RES_CTRL) >> 8) & 0xF;
	if (tmp == 0xD)
		/* 26MHz ref clock to HDMI\DSI\USB PLLs,
		 * MPMU_PLL3CR ;FBD = 0xE6
		 */
		__raw_writel(0x001B9A00, PMUM_PLL3_CR);
	else
		/* 25MHz ref clock to HDMI\DSI\USB PLLs,
		 * MPMU_PLL3CR ;FBD = 0xF6
		 */
		__raw_writel(0x001BdA00, PMUM_PLL3_CR);

	/* PLL3 Control register -Enable SW PLL3*/
	tmp = __raw_readl(PMUM_PLL3_CR);
	__raw_writel(tmp | 0x00000100, PMUM_PLL3_CR);

	/* wait for PLLs to lock*/
	udelay(500);

	/* PMUM_PLL3_CTRL1: take PLL3 out of reset*/
	tmp = __raw_readl(PMUM_PLL3_CTRL1);
	__raw_writel(tmp | 0x20000000, PMUM_PLL3_CTRL1);

	udelay(500);
}

static void turn_off_pll3(void)
{
	u32 tmp = __raw_readl(PMUM_PLL3_CR);

	/* PLL3 Control register -disable SW PLL3*/
	__raw_writel(tmp & ~0x00000100, PMUM_PLL3_CR);

	/* wait for PLLs to lock*/
	udelay(500);

	/* PMUM_PLL3_CTRL1: put PLL3 into reset*/
	tmp = __raw_readl(PMUM_PLL3_CTRL1);
	__raw_writel(tmp & ~0x20000000, PMUM_PLL3_CTRL1);

	udelay(500);
}

static void lcd_pn1_clk_enable(struct clk *clk)
{
	u32 tmp;

	/* DSI clock enable*/
	turn_on_pll3();

	tmp = __raw_readl(clk->clk_rst);
	tmp |= 0x103f;
	__raw_writel(tmp, clk->clk_rst);
}

static void lcd_pn1_clk_disable(struct clk *clk)
{
	u32 tmp = __raw_readl(clk->clk_rst);
	/* tmp &= ~0xf9fff; */
	tmp &= ~0x1038; /* release from reset to keep register setting */
	__raw_writel(tmp, clk->clk_rst);

	/* DSI clock disable*/
	turn_off_pll3();
}

static int lcd_clk_setrate(struct clk *clk, unsigned long val)
{
	u32 tmp = __raw_readl(clk->clk_rst);
	/* u32 pll2clk = pll2_get_clk(); */

	tmp &= ~((0x1f << 15) | (0xf << 8) | (0x3 << 6));
	tmp |= (0x1a << 15);

	switch (val) {
	case 400000000:
		/* DSP1clk = PLL1/2 = 400MHz */
		tmp |= (0x2 << 8) | (0x0 << 6);
		break;
	case 500000000:
		/* DSP1clk = PLL2/x, about 500MHz, temply workaround */
		tmp |= (0x2 << 8) | (0x2 << 6);
		break;
	default:
		printk(KERN_ERR"%s %d not supported\n", __func__, (int) val);
		return -1;
	}

	__raw_writel(tmp, clk->clk_rst);
	return 0;
}

static unsigned long lcd_clk_getrate(struct clk *clk)
{
	u32 lcdclk = clk->rate;

	return lcdclk;
}

struct clkops lcd_pn1_clk_ops = {
	.enable = lcd_pn1_clk_enable,
	.disable = lcd_pn1_clk_disable,
	.setrate = lcd_clk_setrate,
	.getrate = lcd_clk_getrate,
};

static void rtc_clk_enable(struct clk *clk)
{
	uint32_t clk_rst;

	clk_rst = APBC_APBCLK | APBC_FNCLK | APBC_FNCLKSEL(clk->fnclksel);
	clk_rst |= 1 << 7;
	__raw_writel(clk_rst, clk->clk_rst);
}

static void rtc_clk_disable(struct clk *clk)
{
	__raw_writel(0, clk->clk_rst);
}

struct clkops rtc_clk_ops = {
	.enable         = rtc_clk_enable,
	.disable        = rtc_clk_disable,
};

static void gc1000_clk_enable(struct clk *clk)
{
	u32 tmp;

	/* [GC_PWRUP]=1, GC module slow ramp */
	tmp = (1 << 9);
	__raw_writel(tmp, clk->clk_rst);

	tmp = __raw_readl(clk->clk_rst);
	/* [GC_PWRUP]=3, GC module power on */
	tmp |= (3 << 9);
	__raw_writel(tmp, clk->clk_rst);

	tmp = __raw_readl(clk->clk_rst);
	/* [GC_ACLK_SEL]=0, PLL1 divided by 4 */
	tmp &= ~(3 << 4);
	/* [CLK_GC_SRC_SEL]=0, clk_gc source select PLL1 */
	tmp &= ~(3 << 6);
	/* [GC_CLK_DIV]=2, input pll clock to gc clock ratio,
		divide by 2, 800/2 = 400MHz */
	tmp &= ~(0xF << 24);
	tmp |= (2 << 24);
	__raw_writel(tmp, clk->clk_rst);

	tmp = __raw_readl(clk->clk_rst);
	/* [GC_CLK_EN]=1, Peripheral clock enable */
	tmp |= (1 << 3);
	__raw_writel(tmp, clk->clk_rst);

	udelay(100);

	tmp = __raw_readl(clk->clk_rst);
	/* [GC_AXICLK_EN]=1, AXI clock enable */
	tmp |= (1 << 2);
	__raw_writel(tmp, clk->clk_rst);

	udelay(100);

	tmp = __raw_readl(clk->clk_rst);
	/* [GC_ISB]=1, Isolation disable (normal mode) */
	tmp |= (1 << 8);
	__raw_writel(tmp, clk->clk_rst);

	tmp = __raw_readl(clk->clk_rst);
	/* [GC_RST]=1, release GC controller from reset */
	tmp |= (1 << 1);
	__raw_writel(tmp, clk->clk_rst);

	tmp = __raw_readl(clk->clk_rst);
	/* [GC_AXI_RST]=1, release GC controller AXI from reset */
	tmp |= (1 << 0);
	__raw_writel(tmp, clk->clk_rst);
}

static void gc1000_clk_disable(struct clk *clk)
{
	u32 tmp;

	tmp = __raw_readl(clk->clk_rst);
	/* [GC_ISB]=0, Isolation enable (power down mode) */
	tmp &= ~(1 << 8);
	__raw_writel(tmp, clk->clk_rst);

	tmp = __raw_readl(clk->clk_rst);
	/* [GC_RST]=0, hold GC controller in reset */
	/* [GC_AXI_RST]=0, hold GC controller AXI in reset */
	tmp &= ~((1 << 0) | (1 << 1));
	__raw_writel(tmp, clk->clk_rst);

	tmp = __raw_readl(clk->clk_rst);
	/* [GC_CLK_EN]=0, Peripheral clock disable */
	/* [GC_AXICLK_EN]=0, AXI clock disable */
	tmp &= ~((1 << 2) | (1 << 3));
	__raw_writel(tmp, clk->clk_rst);

	tmp = __raw_readl(clk->clk_rst);
	/* [GC_PWRUP]=0, GC module power off */
	tmp &= ~(3 << 9);
	__raw_writel(tmp, clk->clk_rst);
}

static int gc1000_clk_setrate(struct clk *clk, unsigned long target_rate)
{
	return 0;
}

static unsigned long gc1000_clk_getrate(struct clk *clk)
{
	u32 tmp, gc_clk, div;

	tmp = __raw_readl(clk->clk_rst);
	div = (tmp >> 24) & 0xF;

	if (div == 0) {
		printk(KERN_ERR "gc clock div 0, error!!\n");
		return 0;
	}

	/* clock input from PLL1, it's 800MHz */
	gc_clk = 800000000 / div;

	return gc_clk;
}

struct clkops gc1000_clk_ops = {
	.enable		= gc1000_clk_enable,
	.disable	= gc1000_clk_disable,
	.setrate	= gc1000_clk_setrate,
	.getrate	= gc1000_clk_getrate,
};

void __init mmp3_init_irq(void)
{
	gic_dist_init(0, (void __iomem *) GIC_DIST_VIRT_BASE, 29);
	gic_cpu_init(0, (void __iomem *) GIC_CPU_VIRT_BASE);

	if (cpu_is_mmp3_z1() || cpu_is_mmp3_z0())
		mmp3_init_icu();
	else /* mmp3 a0*/
		mmp3_init_gic();
	mmp3_init_gpio();
}

#ifdef CONFIG_UIO_VMETA
long vmeta_delay(long tick)
{
	long count;
	long ret;
	for (ret = 0, count = 0; count < tick; count++) {
		if (count%2)
			ret++;
	}
	return ret;
}

static void vmeta_clk_enable(struct clk *clk)
{
	int reg;

	reg = readl(APMU_VMETA_CLK_RES_CTRL);
	reg |= APMU_VMETA_CLK_RES_CTRL_VMETA_AXICLK_EN;
	writel(reg, APMU_VMETA_CLK_RES_CTRL);
	vmeta_delay(1000);

	/*reg |= APMU_VMETA_CLK_RES_CTRL_VMETA_CLK_SEL;*/
	reg &= ~APMU_VMETA_CLK_RES_CTRL_VMETA_CLK_SEL;
	writel(reg, APMU_VMETA_CLK_RES_CTRL);

	reg |= APMU_VMETA_CLK_RES_CTRL_VMETA_CLK_EN;
	writel(reg, APMU_VMETA_CLK_RES_CTRL);
	vmeta_delay(1000);

	if (cpu_is_mmp3_a0()) {
		reg |= APMU_VMETA_CLK_RES_CTRL_VMETA_ISB;
		writel(reg, APMU_VMETA_CLK_RES_CTRL);
	}

	reg &= ~APMU_VMETA_CLK_RES_CTRL_VMETA_AXI_RST;
	writel(reg, APMU_VMETA_CLK_RES_CTRL);
	reg |= APMU_VMETA_CLK_RES_CTRL_VMETA_AXI_RST;
	writel(reg, APMU_VMETA_CLK_RES_CTRL);

	reg |= APMU_VMETA_CLK_RES_CTRL_VMETA_RST1;
	writel(reg, APMU_VMETA_CLK_RES_CTRL);
}

static void vmeta_clk_disable(struct clk *clk)
{
	int reg;

	if (cpu_is_mmp3_a0()) {
		reg = readl(APMU_VMETA_CLK_RES_CTRL);
		reg &= ~APMU_VMETA_CLK_RES_CTRL_VMETA_ISB;
		writel(reg, APMU_VMETA_CLK_RES_CTRL);

		reg = readl(APMU_VMETA_CLK_RES_CTRL);
		reg &= ~APMU_VMETA_CLK_RES_CTRL_VMETA_AXI_RST;
		writel(reg, APMU_VMETA_CLK_RES_CTRL);
		reg &= ~APMU_VMETA_CLK_RES_CTRL_VMETA_RST1;
		writel(reg, APMU_VMETA_CLK_RES_CTRL);

		reg = readl(APMU_VMETA_CLK_RES_CTRL);
		reg &= ~APMU_VMETA_CLK_RES_CTRL_VMETA_AXICLK_EN;
		writel(reg, APMU_VMETA_CLK_RES_CTRL);
		reg &= ~APMU_VMETA_CLK_RES_CTRL_VMETA_CLK_EN;
		writel(reg, APMU_VMETA_CLK_RES_CTRL);
	} else {
		reg = readl(APMU_VMETA_CLK_RES_CTRL);
		reg &= ~APMU_VMETA_CLK_RES_CTRL_VMETA_CLK_EN;
		writel(reg, APMU_VMETA_CLK_RES_CTRL);
		reg &= ~APMU_VMETA_CLK_RES_CTRL_VMETA_AXICLK_EN;
		writel(reg, APMU_VMETA_CLK_RES_CTRL);

		reg = readl(APMU_VMETA_CLK_RES_CTRL);
		reg &= ~APMU_VMETA_CLK_RES_CTRL_VMETA_CLK_EN;
		writel(reg, APMU_VMETA_CLK_RES_CTRL);
		reg &= ~APMU_VMETA_CLK_RES_CTRL_VMETA_AXICLK_EN;
		writel(reg, APMU_VMETA_CLK_RES_CTRL);
		reg &= ~APMU_VMETA_CLK_RES_CTRL_VMETA_RST1;
		writel(reg, APMU_VMETA_CLK_RES_CTRL);
		reg &= ~APMU_VMETA_CLK_RES_CTRL_VMETA_AXI_RST;
		writel(reg, APMU_VMETA_CLK_RES_CTRL);
	}
}

static int vmeta_clk_setrate(struct clk *clk, unsigned long rate)
{
	return 0;
}

struct clkops vmeta_clk_ops = {
	.enable         = vmeta_clk_enable,
	.disable        = vmeta_clk_disable,
	.setrate        = vmeta_clk_setrate,
};
#endif

/* usb: hsic clock */
static void hsic_clk_enable(struct clk *clk)
{
	uint32_t clk_rst;

	clk_rst  =  __raw_readl(clk->clk_rst);
	clk_rst |= 0x1b;
	__raw_writel(clk_rst, clk->clk_rst);

}

static void hsic_clk_disable(struct clk *clk)
{
	uint32_t clk_rst;

	clk_rst  =  __raw_readl(clk->clk_rst);
	clk_rst &= ~0x18;
	__raw_writel(clk_rst, clk->clk_rst);
}

struct clkops hsic_clk_ops = {
	.enable		= hsic_clk_enable,
	.disable	= hsic_clk_disable,
};

/* APB peripheral clocks */
static APBC_CLK_OPS(uart1, MMP2_UART1, 1, 26000000, &uart_clk_ops);

static APBC_CLK(twsi1, MMP2_TWSI1, 0, 26000000);
static APBC_CLK(twsi2, MMP2_TWSI2, 0, 26000000);
static APBC_CLK(twsi3, MMP2_TWSI3, 0, 26000000);
static APBC_CLK(twsi4, MMP2_TWSI4, 0, 26000000);
static APBC_CLK(twsi5, MMP2_TWSI5, 0, 26000000);
static APBC_CLK(twsi6, MMP2_TWSI6, 0, 26000000);
static APBC_CLK(pwm1, MMP2_PWM0, 0, 26000000);
static APBC_CLK(pwm2, MMP2_PWM1, 0, 26000000);
static APBC_CLK(pwm3, MMP2_PWM2, 0, 26000000);
static APBC_CLK(pwm4, MMP2_PWM3, 0, 26000000);
static APBC_CLK(keypad, MMP2_KPC, 0, 32768);
static APMU_CLK(nand, NAND, 0xbf, 100000000);
static APMU_CLK(u2o, USB, 0x9, 48000000);
static APMU_CLK_OPS(gc, GC, 0, 0, &gc1000_clk_ops);
#ifdef CONFIG_UIO_VMETA
static APMU_CLK_OPS(vmeta, VMETA, 0, 0, &vmeta_clk_ops);
#endif
static APMU_CLK_OPS(lcd, LCD_CLK_RES_CTRL, 0, 500000000, &lcd_pn1_clk_ops);
static APMU_CLK_OPS(sdh0, SDH0, 0x1b, 200000000, &sdhc_clk_ops);
static APMU_CLK_OPS(sdh1, SDH1, 0x1b, 200000000, &sdhc_clk_ops);
static APMU_CLK_OPS(sdh2, SDH2, 0x1b, 200000000, &sdhc_clk_ops);
static APMU_CLK_OPS(sdh3, SDH3, 0x1b, 200000000, &sdhc_clk_ops);
static APBC_CLK_OPS(rtc, MMP2_RTC, 0, 32768, &rtc_clk_ops);
static APMU_CLK_OPS(hsic1, USBHSIC1, 0x1b, 480000000, &hsic_clk_ops);

static struct clk_lookup mmp3_clkregs[] = {
	INIT_CLKREG(&clk_uart1, "pxa2xx-uart.0", NULL),
	INIT_CLKREG(&clk_twsi1, "pxa2xx-i2c.0", NULL),
	INIT_CLKREG(&clk_twsi2, "pxa2xx-i2c.1", NULL),
	INIT_CLKREG(&clk_twsi3, "pxa2xx-i2c.2", NULL),
	INIT_CLKREG(&clk_twsi4, "pxa2xx-i2c.3", NULL),
	INIT_CLKREG(&clk_twsi5, "pxa2xx-i2c.4", NULL),
	INIT_CLKREG(&clk_twsi6, "pxa2xx-i2c.5", NULL),
	INIT_CLKREG(&clk_nand, "pxa3xx-nand", NULL),
	INIT_CLKREG(&clk_gc, NULL, "GCCLK"),
	INIT_CLKREG(&clk_vmeta, NULL, "VMETA_CLK"),
	INIT_CLKREG(&clk_pwm1, "mmp2-pwm.0", NULL),
	INIT_CLKREG(&clk_pwm2, "mmp2-pwm.1", NULL),
	INIT_CLKREG(&clk_pwm3, "mmp2-pwm.2", NULL),
	INIT_CLKREG(&clk_pwm4, "mmp2-pwm.3", NULL),
	INIT_CLKREG(&clk_lcd, NULL, "LCDCLK"),
	INIT_CLKREG(&clk_sdh0, "sdhci-pxa.0", "PXA-SDHCLK"),
	INIT_CLKREG(&clk_sdh1, "sdhci-pxa.1", "PXA-SDHCLK"),
	INIT_CLKREG(&clk_sdh2, "sdhci-pxa.2", "PXA-SDHCLK"),
	INIT_CLKREG(&clk_sdh3, "sdhci-pxa.3", "PXA-SDHCLK"),
	INIT_CLKREG(&clk_rtc, "mmp-rtc", NULL),
	INIT_CLKREG(&clk_keypad, "pxa27x-keypad", NULL),
	INIT_CLKREG(&clk_u2o, NULL, "U2OCLK"),
	INIT_CLKREG(&clk_hsic1, NULL, "HSIC1CLK"),
};

static void __init mmp3_timer_init(void)
{
	uint32_t clk_rst;

#ifdef CONFIG_LOCAL_TIMERS
	twd_base = (void __iomem *)TWD_VIRT_BASE;
#endif
	/* this is early, we have to initialize the CCU registers by
	 * ourselves instead of using clk_* API. Clock rate is defined
	 * by APBC_TIMERS_FNCLKSEL and enabled free-running
	 */
	__raw_writel(APBC_APBCLK | APBC_RST, APBC_MMP2_TIMERS);

	/* 6.5MHz, bus/functional clock enabled, release reset */
	clk_rst = APBC_APBCLK | APBC_FNCLK | APBC_FNCLKSEL(1);
	__raw_writel(clk_rst, APBC_MMP2_TIMERS);

	timer_init(IRQ_MMP3_TIMER1, IRQ_MMP3_TIMER2);
}

struct sys_timer mmp3_timer = {
	.init   = mmp3_timer_init,
};

#ifdef CONFIG_CACHE_L2X0
static void mmp3_init_l2x0(void)
{
	void __iomem *l2x0_base = ioremap(SL2C_PHYS_BASE, SZ_4K);
	if (IS_ERR(l2x0_base)) {
		printk(KERN_ERR "L2 map failed %ld\n", PTR_ERR(l2x0_base));
	} else {
		/* Auxiliary Control:
		   TODO: According to the manual, this register should be
			written in secure access, we may need to move the
			configuration in early stage of boot if TZ enabled

		   [ 0.. 2]	cycles of latency of data RAM read
		   [ 3.. 5]	cycles of latency of data RAM write
		   [ 6.. 8]	cycles of latency of tag RAM
		   [ 9..11]	cycles of latency of dirty RAM
		   [12]		exclusive cache op, 0:disable,1:enable
		   [13..16]	associativity
		   [17..19]	way-size
		   [20]		event monitor bus enable
		   [21]		parity enable
		   [22]		shared attribute override enable
		   [23..24]	force write allocate
				0: use AWCACHE
				1: force no WA
				2: force WA on
				3: internal mapped
		   [25]		reserved, SBO/RAO
		   [26]		Non-secure lockdown enable
		   [27]		Non-secure interrupt access enable
		   [28..31]	reserved, SBZ
		*/
		/*
		   forece NO WA, for A0 memory performance, bug in WA
		   64KB way-size
		   clear bit[16] to make sure l2x0_init call take it as 8-way
		*/
		l2x0_init(l2x0_base, 0x00860000, 0xE200FFFF);
	}
}
#else
static void mmp3_init_l2x0(void) {}
#endif

#define PJ4B_WCB_MIN_MSK	(0x3f)
#define PJ4B_WCB_MIN_SHFT	(1)
#define PJ4B_WCB_MAX_MSK	(0x3f)
#define PJ4B_WCB_MAX_SHFT	(7)
#define PJ4B_WCB_EVCT_MSK	(0x7fff)
#define PJ4B_WCB_EVCT_SHFT	(13)
#define OMITFLD			((unsigned long)-1)
#define UPDATE_ON_VALID(lval, rval, msk, shft)		\
	do if (rval != OMITFLD) {			\
		lval &= ~((msk) << (shft));		\
		lval |= (((rval) & (msk)) << (shft));	\
	} while (0)

static unsigned long pj4b_wcb_config(unsigned long min, unsigned long max,
			unsigned long evct)
{
	register unsigned long regval;
	__asm__("mrc p15, 1, %0, c15, c2, 1" : "=r" (regval));
	UPDATE_ON_VALID(regval, min, PJ4B_WCB_MIN_MSK, PJ4B_WCB_MIN_SHFT);
	UPDATE_ON_VALID(regval, max, PJ4B_WCB_MAX_MSK, PJ4B_WCB_MAX_SHFT);
	UPDATE_ON_VALID(regval, evct, PJ4B_WCB_EVCT_MSK, PJ4B_WCB_EVCT_SHFT);
	__asm__("mcr p15, 1, %0, c15, c2, 1" : : "r" (regval));
	return regval;
}

static int __init mmp3_init(void)
{
	/*
	  let's make minimum WCB open entries to 2 to boost memory access
	*/
	pj4b_wcb_config(2, OMITFLD, OMITFLD);

	mmp3_init_l2x0();

	mfp_init_base(MFPR_VIRT_BASE);
	mfp_init_addr(mmp3_addr_map);

	pxa_init_dma(IRQ_MMP3_DMA_RIQ, 16);
	pxa688_init_dma();
	clkdev_add_table(ARRAY_AND_SIZE(mmp3_clkregs));

	return 0;
}

postcore_initcall(mmp3_init);

/* on-chip devices */
MMP3_DEVICE(uart1, "pxa2xx-uart", 0, UART1, 0xd4030000, 0x30, 4, 5);
MMP3_DEVICE(nand, "pxa3xx-nand", -1, NAND, 0xd4283000, 0x100, 28, 29);
MMP3_DEVICE(twsi1, "pxa2xx-i2c", 0, TWSI1, 0xd4011000, 0x70);
MMP3_DEVICE(twsi2, "pxa2xx-i2c", 1, TWSI2, 0xd4031000, 0x70);
MMP3_DEVICE(twsi3, "pxa2xx-i2c", 2, TWSI3, 0xd4032000, 0x70);
MMP3_DEVICE(twsi4, "pxa2xx-i2c", 3, TWSI4, 0xd4033000, 0x70);
MMP3_DEVICE(twsi5, "pxa2xx-i2c", 4, TWSI5, 0xd4033800, 0x70);
MMP3_DEVICE(twsi6, "pxa2xx-i2c", 5, TWSI6, 0xd4034000, 0x70);
MMP3_DEVICE(pwm1, "mmp2-pwm", 0, NONE, 0xd401a000, 0x10);
MMP3_DEVICE(pwm2, "mmp2-pwm", 1, NONE, 0xd401a400, 0x10);
MMP3_DEVICE(pwm3, "mmp2-pwm", 2, NONE, 0xd401a800, 0x10);
MMP3_DEVICE(pwm4, "mmp2-pwm", 3, NONE, 0xd401ac00, 0x10);
MMP3_DEVICE(fb, "pxa168-fb", 0, LCD, 0xd420b000, 0x500);
MMP3_DEVICE(fb_ovly, "pxa168fb_ovly", 0, LCD, 0xd420b000, 0x500);
MMP3_DEVICE(v4l2_ovly, "pxa168-v4l2_ovly", 0, LCD, 0xd420b000, 0x500);
MMP3_DEVICE(sdh0, "sdhci-pxa", 0, MMC, 0xd4280000, 0x120);
MMP3_DEVICE(sdh1, "sdhci-pxa", 1, MMC2, 0xd4280800, 0x120);
MMP3_DEVICE(sdh2, "sdhci-pxa", 2, MMC3, 0xd4281000, 0x120);
MMP3_DEVICE(sdh3, "sdhci-pxa", 3, MMC4, 0xd4281800, 0x120);
MMP3_DEVICE(sspa1, "mmp2-sspa", 0, SSPA1, 0xc0ffdc00, 0xb0, ADMA1_CH_1, ADMA1_CH_0);
MMP3_DEVICE(sspa2, "mmp2-sspa", 1, SSPA2, 0xc0ffdd00, 0xb0, ADMA2_CH_1, ADMA2_CH_0);
MMP3_DEVICE(keypad, "pxa27x-keypad", -1, KEYPAD, 0xd4012000, 0x4c);

static struct resource mmp3_resource_audiosram[] = {
    [0] = {
		.name   = "audio_sram",
		.start  = 0xd1000000 + SZ_128K + SZ_64K,
		.end    = 0xd1000000 + SZ_256K + SZ_64K - 1,
		.flags  = IORESOURCE_MEM,
	},
};

struct platform_device mmp3_device_audiosram = {
	.name           = "mmp2-audiosram",
	.id             = -1,
	.num_resources  = ARRAY_SIZE(mmp3_resource_audiosram),
	.resource       = mmp3_resource_audiosram,
};

static struct resource mmp3_resource_audiosram_A0[] = {
    [0] = {
		.name   = "audio_sram",
		.start  = 0xe0000000,
		.end    = 0xe0000000 + SZ_16K - 1,
		.flags  = IORESOURCE_MEM,
	},
};

struct platform_device mmp3_device_audiosram_A0 = {
	.name           = "mmp2-audiosram",
	.id             = -1,
	.num_resources  = ARRAY_SIZE(mmp3_resource_audiosram_A0),
	.resource       = mmp3_resource_audiosram_A0,
};

static struct resource mmp3_resource_rtc[] = {
	[0] = {
		.start  = 0xd4010000,
		.end    = 0xD40100ff,
		.flags  = IORESOURCE_MEM,
	},
	[1] = {
		.start  = IRQ_MMP3_RTC,
		.end    = IRQ_MMP3_RTC,
		.flags  = IORESOURCE_IRQ,
		.name   = "RTC_1HZ",
	},

	[2] = {
		.start  = IRQ_MMP3_RTC_ALARM,
		.end    = IRQ_MMP3_RTC_ALARM,
		.flags  = IORESOURCE_IRQ,
		.name   = "RTC_ALARM",
	},

};

struct platform_device mmp3_device_rtc = {
	.name           = "mmp-rtc",
	.id             = -1,
	.resource       = mmp3_resource_rtc,
	.num_resources  = ARRAY_SIZE(mmp3_resource_rtc),
};

#ifdef CONFIG_UIO_VMETA
/* vmeta soc specific functions */
int mmp__vmeta_set_dvfm_constraint(int idx)
{
	return 0;
	/*dvfm_disable(idx);*/
}

int mmp__vmeta_unset_dvfm_constraint(int idx)
{
	return 0;
	/*dvfm_enable(idx);*/
}
void vmeta_pwr(unsigned int enableDisable)
{
	unsigned int reg_vmpwr = 0;
	reg_vmpwr = readl(APMU_VMETA_CLK_RES_CTRL);
	if (VMETA_PWR_ENABLE == enableDisable) {
		if (reg_vmpwr & (APMU_VMETA_CLK_RES_CTRL_VMETA_PWR_ON|APMU_VMETA_CLK_RES_CTRL_VMETA_ISB))
			return; /*Pwr is already on*/

		reg_vmpwr |= APMU_VMETA_CLK_RES_CTRL_VMETA_PWR_SLOW_RAMP_UP;
		writel(reg_vmpwr, APMU_VMETA_CLK_RES_CTRL);

		reg_vmpwr |= APMU_VMETA_CLK_RES_CTRL_VMETA_PWR_ON;
		writel(reg_vmpwr, APMU_VMETA_CLK_RES_CTRL);

		if (cpu_is_mmp3_z1() || cpu_is_mmp3_z0()) {
			reg_vmpwr &= ~APMU_VMETA_CLK_RES_CTRL_VMETA_PWR_SLOW_RAMP_UP;
			writel(reg_vmpwr, APMU_VMETA_CLK_RES_CTRL);

			reg_vmpwr |= APMU_VMETA_CLK_RES_CTRL_VMETA_ISB;
			writel(reg_vmpwr, APMU_VMETA_CLK_RES_CTRL);
		}

	} else if (VMETA_PWR_DISABLE == enableDisable) {
		if ((reg_vmpwr & (APMU_VMETA_CLK_RES_CTRL_VMETA_PWR_ON|APMU_VMETA_CLK_RES_CTRL_VMETA_ISB)) == 0)
			return; /*Pwr is already off*/

		if (cpu_is_mmp3_z1() || cpu_is_mmp3_z0()) {
			reg_vmpwr &= ~APMU_VMETA_CLK_RES_CTRL_VMETA_ISB;
			writel(reg_vmpwr, APMU_VMETA_CLK_RES_CTRL);
		}
		reg_vmpwr &= ~APMU_VMETA_CLK_RES_CTRL_VMETA_PWR_ON;
		writel(reg_vmpwr, APMU_VMETA_CLK_RES_CTRL);
	}
}

void mmp3_clear_keypad_wakeup(void)
{
	uint32_t val;
	uint32_t mask = (1 << 5);

	/* wake event clear is needed in order to clear keypad interrupt */
	val = __raw_readl(APMU_WAKE_CLR);
	__raw_writel(val | mask, APMU_WAKE_CLR);
}

#endif
