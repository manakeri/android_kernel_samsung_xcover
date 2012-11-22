/*
 * linux/arch/arm/mach-pxa/pxa93x.c
 *
 * code specific to pxa93x aka Tavor
 *
 * Copyright (C) 2006 Marvell International Ltd.
 *
 * 2007-09-02: eric miao <eric.miao@marvell.com>
 *             initial version
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/pm.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/sysdev.h>

#include <mach/hardware.h>
#include <mach/gpio.h>
#include <mach/pxa3xx-regs.h>
#include <mach/pxa930.h>
#include <mach/reset.h>
#include <mach/pm.h>
#include <mach/dma.h>
#include <mach/regs-intc.h>
#include <plat/i2c.h>

#include "generic.h"
#include "devices.h"
#include "clock.h"

/* Crystal clock: 13MHz */
#define BASE_CLK	13000000

/* Ring Oscillator Clock: 60MHz */
#define RO_CLK		60000000

#define ACCR_D0CS	(1 << 26)
#define ACCR_PCCE	(1 << 11)

#define PECR_IE(n)	((1 << ((n) * 2)) << 28)
#define PECR_IS(n)	((1 << ((n) * 2)) << 29)

/* crystal frequency to static memory controller multiplier (SMCFS) */
static unsigned char smcfs_mult[8] = { 6, 0, 8, 0, 0, 16, };

/* crystal frequency to HSIO bus frequency multiplier (HSS) */
static unsigned char hss_mult[4] = { 8, 12, 16, 24 };

static struct mfp_addr_map pxa930_mfp_addr_map[] __initdata = {

	MFP_ADDR(GPIO0, 0x02e0),
	MFP_ADDR(GPIO1, 0x02dc),
	MFP_ADDR(GPIO2, 0x02e8),
	MFP_ADDR(GPIO3, 0x02d8),
	MFP_ADDR(GPIO4, 0x02e4),
	MFP_ADDR(GPIO5, 0x02ec),
	MFP_ADDR(GPIO6, 0x02f8),
	MFP_ADDR(GPIO7, 0x02fc),
	MFP_ADDR(GPIO8, 0x0300),
	MFP_ADDR(GPIO9, 0x02d4),
	MFP_ADDR(GPIO10, 0x02f4),
	MFP_ADDR(GPIO11, 0x02f0),
	MFP_ADDR(GPIO12, 0x0304),
	MFP_ADDR(GPIO13, 0x0310),
	MFP_ADDR(GPIO14, 0x0308),
	MFP_ADDR(GPIO15, 0x030c),
	MFP_ADDR(GPIO16, 0x04e8),
	MFP_ADDR(GPIO17, 0x04f4),
	MFP_ADDR(GPIO18, 0x04f8),
	MFP_ADDR(GPIO19, 0x04fc),
	MFP_ADDR(GPIO20, 0x0518),
	MFP_ADDR(GPIO21, 0x051c),
	MFP_ADDR(GPIO22, 0x04ec),
	MFP_ADDR(GPIO23, 0x0500),
	MFP_ADDR(GPIO24, 0x04f0),
	MFP_ADDR(GPIO25, 0x0504),
	MFP_ADDR(GPIO26, 0x0510),
	MFP_ADDR(GPIO27, 0x0514),
	MFP_ADDR(GPIO28, 0x0520),
	MFP_ADDR(GPIO29, 0x0600),
	MFP_ADDR(GPIO30, 0x0618),
	MFP_ADDR(GPIO31, 0x0610),
	MFP_ADDR(GPIO32, 0x060c),
	MFP_ADDR(GPIO33, 0x061c),
	MFP_ADDR(GPIO34, 0x0620),
	MFP_ADDR(GPIO35, 0x0628),
	MFP_ADDR(GPIO36, 0x062c),
	MFP_ADDR(GPIO37, 0x0630),
	MFP_ADDR(GPIO38, 0x0634),
	MFP_ADDR(GPIO39, 0x0638),
	MFP_ADDR(GPIO40, 0x063c),
	MFP_ADDR(GPIO41, 0x0614),
	MFP_ADDR(GPIO42, 0x0624),
	MFP_ADDR(GPIO43, 0x0608),
	MFP_ADDR(GPIO44, 0x0604),
	MFP_ADDR(GPIO45, 0x050c),
	MFP_ADDR(GPIO46, 0x0508),
	MFP_ADDR(GPIO47, 0x02bc),
	MFP_ADDR(GPIO48, 0x02b4),
	MFP_ADDR(GPIO49, 0x02b8),
	MFP_ADDR(GPIO50, 0x02c8),
	MFP_ADDR(GPIO51, 0x02c0),
	MFP_ADDR(GPIO52, 0x02c4),
	MFP_ADDR(GPIO53, 0x02d0),
	MFP_ADDR(GPIO54, 0x02cc),
	MFP_ADDR(GPIO55, 0x029c),
	MFP_ADDR(GPIO56, 0x02a0),
	MFP_ADDR(GPIO57, 0x0294),
	MFP_ADDR(GPIO58, 0x0298),
	MFP_ADDR(GPIO59, 0x02a4),
	MFP_ADDR(GPIO60, 0x02a8),
	MFP_ADDR(GPIO61, 0x02b0),
	MFP_ADDR(GPIO62, 0x02ac),
	MFP_ADDR(GPIO63, 0x0640),
	MFP_ADDR(GPIO64, 0x065c),
	MFP_ADDR(GPIO65, 0x0648),
	MFP_ADDR(GPIO66, 0x0644),
	MFP_ADDR(GPIO67, 0x0674),
	MFP_ADDR(GPIO68, 0x0658),
	MFP_ADDR(GPIO69, 0x0654),
	MFP_ADDR(GPIO70, 0x0660),
	MFP_ADDR(GPIO71, 0x0668),
	MFP_ADDR(GPIO72, 0x0664),
	MFP_ADDR(GPIO73, 0x0650),
	MFP_ADDR(GPIO74, 0x066c),
	MFP_ADDR(GPIO75, 0x064c),
	MFP_ADDR(GPIO76, 0x0670),
	MFP_ADDR(GPIO77, 0x0678),
	MFP_ADDR(GPIO78, 0x067c),
	MFP_ADDR(GPIO79, 0x0694),
	MFP_ADDR(GPIO80, 0x069c),
	MFP_ADDR(GPIO81, 0x06a0),
	MFP_ADDR(GPIO82, 0x06a4),
	MFP_ADDR(GPIO83, 0x0698),
	MFP_ADDR(GPIO84, 0x06bc),
	MFP_ADDR(GPIO85, 0x06b4),
	MFP_ADDR(GPIO86, 0x06b0),
	MFP_ADDR(GPIO87, 0x06c0),
	MFP_ADDR(GPIO88, 0x06c4),
	MFP_ADDR(GPIO89, 0x06ac),
	MFP_ADDR(GPIO90, 0x0680),
	MFP_ADDR(GPIO91, 0x0684),
	MFP_ADDR(GPIO92, 0x0688),
	MFP_ADDR(GPIO93, 0x0690),
	MFP_ADDR(GPIO94, 0x068c),
	MFP_ADDR(GPIO95, 0x06a8),
	MFP_ADDR(GPIO96, 0x06b8),
	MFP_ADDR(GPIO97, 0x0410),
	MFP_ADDR(GPIO98, 0x0418),
	MFP_ADDR(GPIO99, 0x041c),
	MFP_ADDR(GPIO100, 0x0414),
	MFP_ADDR(GPIO101, 0x0408),
	MFP_ADDR(GPIO102, 0x0324),
	MFP_ADDR(GPIO103, 0x040c),
	MFP_ADDR(GPIO104, 0x0400),
	MFP_ADDR(GPIO105, 0x0328),
	MFP_ADDR(GPIO106, 0x0404),

	MFP_ADDR(nXCVREN, 0x0204),
	MFP_ADDR(DF_CLE_nOE, 0x020c),
	MFP_ADDR(DF_nADV1_ALE, 0x0218),
	MFP_ADDR(DF_SCLK_E, 0x0214),
	MFP_ADDR(DF_SCLK_S, 0x0210),
	MFP_ADDR(nBE0, 0x021c),
	MFP_ADDR(nBE1, 0x0220),
	MFP_ADDR(DF_nADV2_ALE, 0x0224),
	MFP_ADDR(DF_INT_RnB, 0x0228),
	MFP_ADDR(DF_nCS0, 0x022c),
	MFP_ADDR(DF_nCS1, 0x0230),
	MFP_ADDR(nLUA, 0x0254),
	MFP_ADDR(nLLA, 0x0258),
	MFP_ADDR(DF_nWE, 0x0234),
	MFP_ADDR(DF_nRE_nOE, 0x0238),
	MFP_ADDR(DF_ADDR0, 0x024c),
	MFP_ADDR(DF_ADDR1, 0x0250),
	MFP_ADDR(DF_ADDR2, 0x025c),
	MFP_ADDR(DF_ADDR3, 0x0260),
	MFP_ADDR(DF_IO0, 0x023c),
	MFP_ADDR(DF_IO1, 0x0240),
	MFP_ADDR(DF_IO2, 0x0244),
	MFP_ADDR(DF_IO3, 0x0248),
	MFP_ADDR(DF_IO4, 0x0264),
	MFP_ADDR(DF_IO5, 0x0268),
	MFP_ADDR(DF_IO6, 0x026c),
	MFP_ADDR(DF_IO7, 0x0270),
	MFP_ADDR(DF_IO8, 0x0274),
	MFP_ADDR(DF_IO9, 0x0278),
	MFP_ADDR(DF_IO10, 0x027c),
	MFP_ADDR(DF_IO11, 0x0280),
	MFP_ADDR(DF_IO12, 0x0284),
	MFP_ADDR(DF_IO13, 0x0288),
	MFP_ADDR(DF_IO14, 0x028c),
	MFP_ADDR(DF_IO15, 0x0290),

	MFP_ADDR(GSIM_UIO, 0x0314),
	MFP_ADDR(GSIM_UCLK, 0x0318),
	MFP_ADDR(GSIM_UDET, 0x031c),
	MFP_ADDR(GSIM_nURST, 0x0320),

	MFP_ADDR(PMIC_INT, 0x06c8),

	MFP_ADDR(RDY, 0x0200),

	MFP_ADDR_END,
};

static struct mfp_addr_map pxa935_mfp_addr_map[] __initdata = {
	MFP_ADDR(GPIO159, 0x0524),
	MFP_ADDR(GPIO163, 0x0534),
	MFP_ADDR(GPIO167, 0x0544),
	MFP_ADDR(GPIO168, 0x0548),
	MFP_ADDR(GPIO169, 0x054c),
	MFP_ADDR(GPIO170, 0x0550),
	MFP_ADDR(GPIO171, 0x0554),
	MFP_ADDR(GPIO172, 0x0558),
	MFP_ADDR(GPIO173, 0x055c),

	MFP_ADDR_END,
};

void pxa93x_clear_reset_status(unsigned int mask)
{
	/* RESET_STATUS_* has a 1:1 mapping with ARSR */
	ARSR = mask;
}

/*
 * Return the current HSIO bus clock frequency
 */
static unsigned long clk_pxa93x_hsio_getrate(struct clk *clk)
{
	unsigned long acsr;
	unsigned int hss, hsio_clk;

	acsr = ACSR;

	hss = (acsr >> 14) & 0x3;
	hsio_clk = (acsr & ACCR_D0CS) ? RO_CLK : hss_mult[hss] * BASE_CLK;

	return hsio_clk;
}

void clk_pxa93x_cken_enable(struct clk *clk)
{
	unsigned long mask = 1ul << (clk->cken & 0x1f);

	if (clk->cken < 32)
		CKENA |= mask;
	else
		CKENB |= mask;
}

void clk_pxa93x_cken_disable(struct clk *clk)
{
	unsigned long mask = 1ul << (clk->cken & 0x1f);

	if (clk->cken < 32)
		CKENA &= ~mask;
	else
		CKENB &= ~mask;
}

const struct clkops clk_pxa93x_cken_ops = {
	.enable		= clk_pxa93x_cken_enable,
	.disable	= clk_pxa93x_cken_disable,
};

static const struct clkops clk_pxa93x_hsio_ops = {
	.enable		= clk_pxa93x_cken_enable,
	.disable	= clk_pxa93x_cken_disable,
	.getrate	= clk_pxa93x_hsio_getrate,
};

static void clk_pout_enable(struct clk *clk)
{
	OSCC |= OSCC_PEN;
}

static void clk_pout_disable(struct clk *clk)
{
	OSCC &= ~OSCC_PEN;
}

static const struct clkops clk_pout_ops = {
	.enable		= clk_pout_enable,
	.disable	= clk_pout_disable,
};

static void clk_dummy_enable(struct clk *clk)
{
}

static void clk_dummy_disable(struct clk *clk)
{
}

static const struct clkops clk_dummy_ops = {
	.enable		= clk_dummy_enable,
	.disable	= clk_dummy_disable,
};

static struct clk clk_pxa93x_pout = {
	.ops		= &clk_pout_ops,
	.rate		= 13000000,
	.delay		= 70,
};

static struct clk clk_dummy = {
	.ops		= &clk_dummy_ops,
};

static DEFINE_PXA93_CK(pxa93x_lcd, LCD, &clk_pxa93x_hsio_ops);
static DEFINE_PXA93_CK(pxa93x_camera, CAMERA, &clk_pxa93x_hsio_ops);
static DEFINE_PXA93_CKEN(pxa93x_ffuart, FFUART, 14857000, 1);
static DEFINE_PXA93_CKEN(pxa93x_btuart, BTUART, 14857000, 1);
static DEFINE_PXA93_CKEN(pxa93x_stuart, STUART, 14857000, 1);
static DEFINE_PXA93_CKEN(pxa93x_i2c, I2C, 32842000, 0);
static DEFINE_PXA93_CKEN(pxa93x_keypad, KEYPAD, 32768, 0);
static DEFINE_PXA93_CKEN(pxa93x_ssp1, SSP1, 13000000, 0);
static DEFINE_PXA93_CKEN(pxa93x_ssp2, SSP2, 13000000, 0);
static DEFINE_PXA93_CKEN(pxa93x_ssp3, SSP3, 13000000, 0);
static DEFINE_PXA93_CKEN(pxa93x_ssp4, SSP4, 13000000, 0);
static DEFINE_PXA93_CKEN(pxa93x_pwm0, PWM0, 13000000, 0);
static DEFINE_PXA93_CKEN(pxa93x_pwm1, PWM1, 13000000, 0);
static DEFINE_PXA93_CKEN(pxa93x_mmc1, MMC1, 19500000, 0);
static DEFINE_PXA93_CKEN(pxa93x_mmc2, MMC2, 19500000, 0);

static struct clk_lookup pxa93x_clkregs[] = {
	INIT_CLKREG(&clk_pxa93x_pout, NULL, "CLK_POUT"),
	/* Power I2C clock is always on */
	INIT_CLKREG(&clk_dummy, "pxa3xx-pwri2c.1", NULL),
	INIT_CLKREG(&clk_pxa93x_lcd, "pxa2xx-fb", NULL),
	INIT_CLKREG(&clk_pxa93x_camera, NULL, "CAMCLK"),
	INIT_CLKREG(&clk_pxa93x_ffuart, "pxa2xx-uart.0", NULL),
	INIT_CLKREG(&clk_pxa93x_btuart, "pxa2xx-uart.1", NULL),
	INIT_CLKREG(&clk_pxa93x_stuart, "pxa2xx-uart.2", NULL),
	INIT_CLKREG(&clk_pxa93x_stuart, "pxa2xx-ir", "UARTCLK"),
	INIT_CLKREG(&clk_pxa93x_i2c, "pxa2xx-i2c.0", NULL),
	INIT_CLKREG(&clk_pxa93x_keypad, "pxa27x-keypad", NULL),
	INIT_CLKREG(&clk_pxa93x_ssp1, "pxa27x-ssp.0", NULL),
	INIT_CLKREG(&clk_pxa93x_ssp2, "pxa27x-ssp.1", NULL),
	INIT_CLKREG(&clk_pxa93x_ssp3, "pxa27x-ssp.2", NULL),
	INIT_CLKREG(&clk_pxa93x_ssp4, "pxa27x-ssp.3", NULL),
	INIT_CLKREG(&clk_pxa93x_pwm0, "pxa27x-pwm.0", NULL),
	INIT_CLKREG(&clk_pxa93x_pwm1, "pxa27x-pwm.1", NULL),
};

static struct clk_lookup pxa930_clkregs[] = {
	INIT_CLKREG(&clk_pxa93x_mmc1, "pxa2xx-mci.0", NULL),
	INIT_CLKREG(&clk_pxa93x_mmc2, "pxa2xx-mci.1", NULL),
};

static void pxa_ack_ext_wakeup(unsigned int irq)
{
	PECR |= PECR_IS(irq - IRQ_WAKEUP0);
}

static void pxa_mask_ext_wakeup(unsigned int irq)
{
	ICMR2 &= ~(1 << ((irq - PXA_IRQ(0)) & 0x1f));
	PECR &= ~PECR_IE(irq - IRQ_WAKEUP0);
}

static void pxa_unmask_ext_wakeup(unsigned int irq)
{
	ICMR2 |= 1 << ((irq - PXA_IRQ(0)) & 0x1f);
	PECR |= PECR_IE(irq - IRQ_WAKEUP0);
}

static int pxa_set_ext_wakeup_type(unsigned int irq, unsigned int flow_type)
{
	if (flow_type & IRQ_TYPE_EDGE_RISING)
		PWER |= 1 << (irq - IRQ_WAKEUP0);

	if (flow_type & IRQ_TYPE_EDGE_FALLING)
		PWER |= 1 << (irq - IRQ_WAKEUP0 + 2);

	return 0;
}

static struct irq_chip pxa_ext_wakeup_chip = {
	.name		= "WAKEUP",
	.ack		= pxa_ack_ext_wakeup,
	.mask		= pxa_mask_ext_wakeup,
	.unmask		= pxa_unmask_ext_wakeup,
	.set_type	= pxa_set_ext_wakeup_type,
};

static void __init pxa_init_ext_wakeup_irq(set_wake_t fn)
{
	set_irq_chip(IRQ_WAKEUP0, &pxa_ext_wakeup_chip);
	set_irq_handler(IRQ_WAKEUP0, handle_edge_irq);
	set_irq_flags(IRQ_WAKEUP0, IRQF_VALID);

	pxa_ext_wakeup_chip.set_wake = fn;
}

void __init pxa93x_init_irq(void)
{
	/* enable CP6 access */
	u32 value;
	__asm__ __volatile__("mrc p15, 0, %0, c15, c1, 0\n": "=r"(value));
	value |= (1 << 6);
	__asm__ __volatile__("mcr p15, 0, %0, c15, c1, 0\n": :"r"(value));

	pxa_init_irq(96, NULL);
	pxa_init_ext_wakeup_irq(NULL);
	pxa_init_gpio(IRQ_GPIO_2_x, 2, 127, NULL);
}

/*
 * device registration specific to PXA93x.
 */

void __init pxa93x_set_i2c_power_info(struct i2c_pxa_platform_data *info)
{
	pxa_register_device(&pxa3xx_device_i2c_power, info);
}

static struct platform_device *devices[] __initdata = {
	&sa1100_device_rtc,
	&pxa_device_rtc,
	&pxa27x_device_ssp1,
	&pxa27x_device_ssp2,
	&pxa27x_device_ssp3,
	&pxa3xx_device_ssp4,
	&pxa27x_device_pwm0,
	&pxa27x_device_pwm1,
};

static int __init pxa93x_init(void)
{
	int ret = 0;

	if (cpu_is_pxa93x()) {
		mfp_init_base(io_p2v(MFPR_BASE));
		mfp_init_addr(pxa930_mfp_addr_map);

		if (cpu_is_pxa935())
			mfp_init_addr(pxa935_mfp_addr_map);

		reset_status = ARSR;

		/*
		 * clear RDH bit every time after reset
		 *
		 * Note: the last 3 bits DxS are write-1-to-clear so carefully
		 * preserve them here in case they will be referenced later
		 */
		ASCR &= ~(ASCR_RDH | ASCR_D1S | ASCR_D2S | ASCR_D3S);

		clkdev_add_table(pxa93x_clkregs, ARRAY_SIZE(pxa93x_clkregs));
		if (cpu_is_pxa930())
			clkdev_add_table(pxa930_clkregs,
				ARRAY_SIZE(pxa930_clkregs));

		if ((ret = pxa_init_dma(IRQ_DMA, 32)))
			return ret;

		ret = platform_add_devices(devices, ARRAY_SIZE(devices));
	}

	return ret;
}

postcore_initcall(pxa93x_init);
