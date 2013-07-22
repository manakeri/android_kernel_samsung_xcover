/*
 *  linux/arch/arm/mach-pxa/generic.c
 *
 *  Author:	Nicolas Pitre
 *  Created:	Jun 15, 2001
 *  Copyright:	MontaVista Software Inc.
 *
 * Code common to all PXA machines.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Since this file should be linked before any other machine specific file,
 * the __initcall() here will be executed first.  This serves as default
 * initialization stuff for PXA machines which can be overridden later if
 * need be.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/onenand.h>

#include <mach/hardware.h>
#include <asm/system.h>
#include <asm/pgtable.h>
#include <asm/mach/map.h>
#include <asm/mach-types.h>

#include <mach/reset.h>
#include <mach/gpio.h>
#include <mach/part_table.h>

#include <plat/pxa3xx_onenand.h>
#include <plat/pxa3xx_nand.h>
#include <plat/mfp.h>
#include <linux/delay.h>
#include "generic.h"

#include <mach/audio.h>
#if defined(CONFIG_FB_PXA95x)
#include <mach/pxa95xfb.h>
#endif
#include <mach/pxa95x_pm.h>
#include <linux/platform_device.h>

void (*abu_mfp_init_func)(bool);
void (*ssp3_mfp_init_func)(bool);

/* chip id is introduced from PXA95x */
unsigned int pxa_chip_id;
EXPORT_SYMBOL(pxa_chip_id);

void clear_reset_status(unsigned int mask)
{
	if (cpu_is_pxa2xx())
		pxa2xx_clear_reset_status(mask);
	else if (cpu_is_pxa3xx())
		pxa3xx_clear_reset_status(mask);
	else if (cpu_is_pxa93x())
		pxa93x_clear_reset_status(mask);
	else
		pxa95x_clear_reset_status(mask);
}

unsigned long get_clock_tick_rate(void)
{
	unsigned long clock_tick_rate;

	if (cpu_is_pxa25x())
		clock_tick_rate = 3686400;
	else if (machine_is_mainstone())
		clock_tick_rate = 3249600;
	else
		clock_tick_rate = 3250000;

	return clock_tick_rate;
}
EXPORT_SYMBOL(get_clock_tick_rate);

/*
 * Get the clock frequency as reflected by CCCR and the turbo flag.
 * We assume these values have been applied via a fcs.
 * If info is not 0 we also display the current settings.
 */
unsigned int get_clk_frequency_khz(int info)
{
	if (cpu_is_pxa25x())
		return pxa25x_get_clk_frequency_khz(info);
	else if (cpu_is_pxa27x())
		return pxa27x_get_clk_frequency_khz(info);
	return 0;
}
EXPORT_SYMBOL(get_clk_frequency_khz);

/*
 * Return the current memory clock frequency in units of 10kHz
 * Only pxa2xx_pcmcia device needs this API.
 */
unsigned int get_memclk_frequency_10khz(void)
{
	if (cpu_is_pxa25x())
		return pxa25x_get_memclk_frequency_10khz();
	else if (cpu_is_pxa27x())
		return pxa27x_get_memclk_frequency_10khz();
	return 0;
}
EXPORT_SYMBOL(get_memclk_frequency_10khz);

/*
 * Intel PXA2xx internal register mapping.
 *
 * Note 1: not all PXA2xx variants implement all those addresses.
 *
 * Note 2: virtual 0xfffe0000-0xffffffff is reserved for the vector table
 *         and cache flush area.
 *
 * Note 3: virtual 0xfb000000-0xfb00ffff is reserved for PXA95x
 *
 */
static struct map_desc standard_io_desc[] __initdata = {
  	{	/* Devs */
		.virtual	=  0xf2000000,
		.pfn		= __phys_to_pfn(0x40000000),
		.length		= 0x02000000,
		.type		= MT_DEVICE
	}, {	/* Mem Ctl */
		.virtual	=  0xf6000000,
		.pfn		= __phys_to_pfn(0x48000000),
		.length		= 0x00200000,
		.type		= MT_DEVICE
	}, {	/* Camera */
		.virtual	=  0xfa000000,
		.pfn		= __phys_to_pfn(0x50000000),
		.length		= 0x00100000,
		.type		= MT_DEVICE
	}, {	/* Sys */
		.virtual	= 0xfb000000,
		.pfn		= __phys_to_pfn(0x46000000),
		.length		= 0x00010000,
		.type		= MT_DEVICE,
	}, {	/* IMem ctl */
		.virtual	=  0xfe000000,
		.pfn		= __phys_to_pfn(0x58000000),
		.length		= 0x00100000,
		.type		= MT_DEVICE
	}, {	/* UNCACHED_PHYS_0 */
		.virtual	= 0xff000000,
		.pfn		= __phys_to_pfn(0x00000000),
		.length		= 0x00100000,
		.type		= MT_DEVICE
	}
};

#if defined(CONFIG_MTD_NAND)
static struct pxa3xx_nand_platform_data nand_info = {
	.controller_attrs = PXA3XX_ARBI_EN | PXA3XX_NAKED_CMD_EN | PXA3XX_DMA_EN | PXA3XX_ADV_TIME_TUNING | PXA3XX_POLLING_MODE,
	.parts[0] = android_512m_4k_page_partitions,
	.nr_parts[0] =   ARRAY_SIZE(android_512m_4k_page_partitions),
};


void nand_init(void)
{
	pxa3xx_set_nand_info(&nand_info);
}
#else
void nand_init(void) {}
#endif /* CONFIG_MTD_NAND */


#define	CONFIG_FSR_ONENAND

#if (defined(CONFIG_MTD_ONENAND) || defined(CONFIG_MTD_ONENAND_MODULE))

extern void onenand_mmcontrol_smc_cfg(void);
extern void onenand_sync_clk_cfg(void);

static void __attribute__ ((unused)) onenand_mmcontrol(struct mtd_info *mtd, int sync_read)
{
        struct onenand_chip *this = mtd->priv;
        unsigned int syscfg;

        if (sync_read) {
		onenand_mmcontrol_smc_cfg();
		syscfg = this->read_word(this->base + ONENAND_REG_SYS_CFG1);
		syscfg &= (~(0x07<<9));
		/* 16 words for one burst */
		syscfg |= 0x03<<9;
		this->write_word((syscfg | sync_read ), this->base + ONENAND_REG_SYS_CFG1);
	} else {
		syscfg = this->read_word(this->base + ONENAND_REG_SYS_CFG1);
		this->write_word((syscfg & ~sync_read),this->base + ONENAND_REG_SYS_CFG1);
	}
}

static struct pxa3xx_onenand_platform_data tavorevb3_onenand_info;
static int set_partition_info(u32 flash_size, u32 page_size, struct pxa3xx_onenand_platform_data *pdata)
{
// SAMSUNG, hanjung.park@samsung.com, #5448
/*
	int found = -EINVAL;
	if (256 == flash_size) {
		pdata->parts = android_256m_4k_page_partitions;
		pdata->nr_parts = ARRAY_SIZE(android_256m_4k_page_partitions);
		found = 0;
	}
	else if (512 == flash_size) {
		pdata->parts = android_512m_4k_page_partitions;
		pdata->nr_parts = ARRAY_SIZE(android_512m_4k_page_partitions);
		found = 0;
	}

	if (0 != found){
		printk(KERN_ERR"***************no proper partition table *************\n");
	}
	return found;
*/
// SAMSUNG, hanjung.park@samsung.com, #5448
}
void onenand_init(int sync_enable)
{

	if(sync_enable){
		onenand_sync_clk_cfg();
		tavorevb3_onenand_info.mmcontrol = onenand_mmcontrol;
	}
	tavorevb3_onenand_info.set_partition_info = set_partition_info;
	pxa3xx_set_onenand_info(&tavorevb3_onenand_info);
}
#elif defined(CONFIG_FSR_ONENAND)
void onenand_init(int sync_enable)
{
	unsigned char __iomem *ACCR_REG_base;
	unsigned char __iomem *smc_base;
	unsigned char __iomem *ONDctrl_base;
	unsigned int csadrcfg2, msc1, sxcnfg;
	unsigned short syscfg;
	u32 temp;



	printk("[OneNAND Init] setting the related register\r\n");



	ONDctrl_base = ioremap(0x10000000,  0x00020000);

	syscfg = readw(ONDctrl_base + ONENAND_REG_SYS_CFG1);
	if( sync_enable )
	{
		syscfg &= (~(0x01 << 15));
		syscfg &= (~(0x07 << 12));
		syscfg &= (~(0x07 << 9));
		syscfg &= (~(0x01 << 7));
		syscfg &= (~(0x01 << 3));
		syscfg &= (~(0x01 << 2));
		syscfg &= (~(0x01 << 1));
		syscfg |= (0x01 << 15);	// Read Mode : 1 (sync)
		syscfg |= (0x06 << 12);	// Burst Read Write Latency : 6 (6 cycle)
		syscfg |= (0x03 << 9);	// Burst Length : 3 (16 words)
		syscfg |= (0x01 << 7);	// RDY signal polarity : 1 (High for ready)
		syscfg |= (0x00 << 3);	// VHF : 0
		syscfg |= (0x01 << 2);	// HF : 1
		syscfg |= (0x01 << 1);	// Write Mode : 1 (sync)
	}
	writew(syscfg, (ONDctrl_base + ONENAND_REG_SYS_CFG1));
	printk("[OneNAND Init] OneNAND system configuration register : 0x%08x\r\n", syscfg);
	 
	iounmap(ONDctrl_base);


	
	#define ACCR_REG 0x41340000
	ACCR_REG_base = ioremap(ACCR_REG, 4);
	if( ACCR_REG_base )
	{
		temp  = ioread32(ACCR_REG_base);
		temp &= (~(7 << 23));
		temp |= (0x04 << 23);	// SMCFS : 4 (156MHz)
		iowrite32(temp, ACCR_REG_base);
		printk("[OneNAND Init] Appication subsystem Clock Configuration Register : 0x%08x\r\n", temp);
	}
	iounmap(ACCR_REG_base);


	smc_base = ioremap(SMC_START, SMC_END - SMC_START + 1);

	temp = ioread32(smc_base + MEMCLKCFG_OFF);
	temp &= (~(0x03 << 24));
	temp &= (~(0x07 << 16));
	temp |= (0x01 << 24);	// DF_CLKCTL : 1 (Clock is olny on when MSMMC configured for sync operation)
	temp |= (0x02 << 16);	// DF_CLKDIV : 2 (SMC freq/2)
	iowrite32(temp, smc_base + MEMCLKCFG_OFF);
	printk("[OneNAND Init] MEMCLKCFG Register : 0x%08x\r\n", temp);

	if( sync_enable )
	{
		csadrcfg2 = readl(smc_base + CSADRCFG2_OFF);
		csadrcfg2 &= (~(0x03 << 20));
		csadrcfg2 &= (~(0x07 << 17));
		csadrcfg2 &= (~(0x07 << 14));
		csadrcfg2 &= (~(0x0F << 8));
		csadrcfg2 &= (~(0x03 << 4));
		csadrcfg2 &= (~(0x0F << 0));
		csadrcfg2 |= (0x03 << 20);	// Address Latch Timing : 3 (One DF_SCLK of setup, One DF_SCLK of hold)
		csadrcfg2 |= (0x01 << 17);	// Address Latch Width : 1 
		csadrcfg2 |= (0x00 << 14);	// Address Configuration : 0 (Full-latch mode)
		csadrcfg2 |= (0x09 << 8);	// Address Split : 9 (Byte address bit 17)
		csadrcfg2 |= (0x01 << 4);	// Address Base : 1 (Byte address bit 1)
		csadrcfg2 |= (0x0E << 0);	// Interface & Addressing Type : 0xE (DFI, AA/D multiplexing sync device, read and write)
		writel(csadrcfg2, smc_base + CSADRCFG2_OFF);
		printk("[OneNAND Init] CSADRCFG2 Register : 0x%08x\r\n", csadrcfg2);


		msc1 = readl(smc_base + MSC1_OFF);
		msc1 &= (~(0x0F << 12));
		msc1 &= (~(0x0F << 8));
		msc1 &= (~(0x0F << 4));
		msc1 &= (~(0x01 << 3));
		msc1 &= (~(0x07 << 0));
		msc1 |= (0x07 << 12);	// Reserved
		msc1 |= (0x0F << 8);	// Return Delay Next : 0xf
		msc1 |= (0x01 << 4);	// Return Delay First : 0x1
		msc1 |= (0x01 << 3);	// Set with "1" always
		msc1 |= (0x00 << 0);	// Memory Type : 0 (Flash memory)
		writel(msc1, smc_base + MSC1_OFF);
		printk("[OneNAND Init] MSC1 Register : 0x%08x\r\n", msc1);


		sxcnfg = readl(smc_base + SXCNFG_OFF);
		sxcnfg &= (~(0x0F << 28));
		sxcnfg &= (~(0x0F << 21));
		sxcnfg &= (~(0x07 << 18));
		sxcnfg &= (~(0x03 << 16));
		sxcnfg |= (0x00 << 30);	// Alternate Addressing mode bit : 0 (Normal operation)
		sxcnfg |= (0x00 << 29);	// Sync Wite Mode bit for CS<3:2> : 0 (Assert nWE only address phase)
		sxcnfg |= (0x01 << 28);	// Set with "1" always
		sxcnfg |= (0x06 << 21);	// Write Latency for Sync Write on CS<3:2> : 6 (7 Clock)
		sxcnfg |= (0x06 << 18);	// Read Latency for Sync Read on CS<3:2> : 6 (7 Clock)
		sxcnfg |= (0x01 << 16);	// Sync Mode Enable Bit for CS<3:2> : 1 (CS2 is sync mode enable)
		writel(sxcnfg, smc_base + SXCNFG_OFF);
		printk("[OneNAND Init] SXCNFG Register : 0x%08x\r\n", sxcnfg);


		writel(0x04, smc_base + CLK_RET_DEL_OFF);
		writel(0x00, smc_base + ADV_RET_DEL_OFF);
	}
	else
	{
		writel(sxcnfg, smc_base + SXCNFG_OFF);
		writel(msc1, smc_base + MSC1_OFF);
		writel(csadrcfg2, smc_base + CSADRCFG2_OFF);
	}

	iounmap(smc_base);
		

	printk("[OneNAND Init] setting the related register was done\r\n");
}
#else
void onenand_init(int sync_enable){}
#endif

/* Board ID based on BOAR= cmdline token get from OBM */
static long g_board_id = -1;
static int __init set_board_id(char *p)
{
	int ret;
	ret = strict_strtol(p, 16, &g_board_id);
	if (ret < 0) {
		printk(KERN_ERR "%s g_board_id is not right\n", __func__);
		return ret;
	}
	printk(KERN_INFO "%s g_board_id = %ld\n", __func__, g_board_id);
	return 1;
}
__setup("BOAR=", set_board_id);

long get_board_id(void)
{
	return g_board_id;
}
EXPORT_SYMBOL(get_board_id);

/* Board ID based on lcdid= cmdline token get from OBM */
static long g_lcd_id = -1;
static int __init set_lcd_id(char *p)
{
        int ret;
        ret = strict_strtol(p, 16, &g_lcd_id);
        if (ret < 0) {
                printk(KERN_ERR "%s g_lcd_id is not right\n", __func__);
                return ret;
        }
        printk(KERN_INFO "%s g_lcd_id = %ld\n", __func__, g_lcd_id);
        return 1;
}
__setup("lcdid=", set_lcd_id);

long get_lcd_id(void)
{
        return g_lcd_id;
}
EXPORT_SYMBOL(get_lcd_id);

void __init pxa_map_io(void)
{
	iotable_init(standard_io_desc, ARRAY_SIZE(standard_io_desc));
	get_clk_frequency_khz(1);

	if (!cpu_is_pxa2xx() || !cpu_is_pxa3xx() || !cpu_is_pxa93x())
		pxa_chip_id = __raw_readl(0xfb00ff80);
}

void __init set_abu_init_func(void (*func)(bool))
{
	if (func)
		abu_mfp_init_func = func;
	else
		printk(KERN_ERR "%s called with NULL pointer\n", __func__);
}

void pxa95x_abu_mfp_init(bool abu)
{
	if (abu_mfp_init_func)
		abu_mfp_init_func(abu);
	else
		panic("pxa95x_abu_mfp_init called with NULL pointer!\n");
}

void __init set_ssp_init_func(void (*func)(bool))
{
	if (func)
		ssp3_mfp_init_func = func;
	else
		printk(KERN_ERR "%s called with NULL pointer\n", __func__);
}

void pxa95x_ssp_mfp_init(bool bssp)
{
	if (ssp3_mfp_init_func)
		ssp3_mfp_init_func(bssp);
	else
		panic("pxa95x_abu_mfp_init called with NULL pointer!\n");
}

#if defined(CONFIG_FB_PXA95x)

int modify_lcd_pclk(unsigned long int pclk_val, unsigned int blw, unsigned int elw, unsigned int hsw)
{
	return pxa95xfb_pclk_modification(pclk_val, blw, elw, hsw);
}
EXPORT_SYMBOL(modify_lcd_pclk);

#endif

static unsigned int rfic_reset_gpio_pin = MFP_PIN_GPIO112;
void pxa9xx_platform_rfic_reset(
		unsigned short in_len,  void * in_buf, 
		unsigned short out_len, void *out_buf )
{
	int uDelay = *((int*)in_buf);
	printk(KERN_ERR "%s: RFIC RPC gpio_request recieved, delay=%d!\n", __func__, uDelay);
	
	if (gpio_request(rfic_reset_gpio_pin, "RFIC reset RPC")) {

		printk(KERN_ERR "RFIC RPC gpio_request: failed!\n");
		return;
	}	
	gpio_direction_output(rfic_reset_gpio_pin, 0);
	udelay(uDelay);
	gpio_direction_output(rfic_reset_gpio_pin, 1);
	gpio_free(rfic_reset_gpio_pin);
}
EXPORT_SYMBOL(pxa9xx_platform_rfic_reset);

#ifdef CONFIG_SUSPEND
extern int dirty_writeback_suspend(void);
extern int dirty_writeback_resume(void);

/* generic suspend call backs */
static struct platform_device android_device_generic_suspend_cbs = {
	.name   = "android_generic_suspend_cbs",
	.id     = -1,
};

static int android_generic_suspend_func(struct platform_device *_dev, pm_message_t state)
{
	dirty_writeback_suspend();
	return 0;
}
static int android_generic_resume_func(struct platform_device *_dev)
{
	dirty_writeback_resume();
	return 0;
}
static struct platform_driver android_driver_generic_suspend_cbs = {
	.suspend = android_generic_suspend_func,
	.resume	= android_generic_resume_func,
	.driver	= {
		.owner	= THIS_MODULE,
		.name	= "android_generic_suspend_cbs",
	},
};

int platform_generic_suspend_cbs_register(void)
{
	int ret;
	ret = platform_device_register(&android_device_generic_suspend_cbs);
	if (ret) {
		printk(KERN_ERR "%s error: platform_device_register failed ret = %d !!!\n", __func__, ret);
		BUG_ON(1);
	}
	ret = platform_driver_register(&android_driver_generic_suspend_cbs);
	if (ret) {
		printk(KERN_ERR "%s error: platform_driver_register failed ret = %d !!!\n", __func__, ret);
		BUG_ON(1);
	}
}

static int __init generic_init(void)
{
	unsigned int ret;
	ret = platform_generic_suspend_cbs_register();
	if (ret) {
		printk(KERN_ERR "%s failed ret = %d !!!\n", __func__, ret);
		BUG_ON(1);
	}
}
#else
static inline __init generic_init(void) {}
#endif /* CONFIG_SUSPEND */

static int __init generic_exit(void)
{
}

module_init(generic_init);
module_exit(generic_exit);
