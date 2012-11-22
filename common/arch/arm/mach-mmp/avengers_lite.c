/*
 *  linux/arch/arm/mach-mmp/avengers_lite.c
 *
 *  Support for the Marvell PXA168-based Avengers lite Development Platform.
 *
 *  Copyright (C) 2009-2010 Marvell International Ltd.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  publishhed by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/addr-map.h>
#include <mach/mfp-pxa168.h>
#include <mach/pxa168.h>
#include <mach/irqs.h>


#include "common.h"
#include <linux/delay.h>

/* Avengers lite MFP configurations */
static unsigned long avengers_lite_pin_config_V16F[] __initdata = {
	/* DEBUG_UART */
	GPIO88_UART2_TXD,
	GPIO89_UART2_RXD,
};

static struct mtd_partition avengers_nand_partitions_0[] = {
	{
		.name		= "bootloader",
		.offset		= 0,
		.size		= SZ_1M,
		.mask_flags	= MTD_WRITEABLE,
	}, {
		.name		= "kernel",
		.offset		= MTDPART_OFS_APPEND,
		.size		= (SZ_2M + SZ_1M),
	}, {
		.name		= "mass0",
		.offset		= MTDPART_OFS_APPEND,
		.size		= SZ_48M,
	}
};

static struct mtd_partition avengers_nand_partitions_1[] = {
	{
		.name		= "reserved",
		.offset		= 0,
		.size		= SZ_2M,
		.mask_flags	= MTD_WRITEABLE,
	}, {
		.name		= "filesystem",
		.offset		= MTDPART_OFS_APPEND,
		.size		= SZ_512M,
	}, {
		.name		= "mass1",
		.offset		= MTDPART_OFS_APPEND,
		.size		= SZ_16M,
	}, {
		.name		= "mass2",
		.offset		= MTDPART_OFS_APPEND,
		.size		= SZ_256M,
	}
};

static struct pxa3xx_nand_platform_data avengers_nand_info;
static void __init avengers_init_flash(void)
{
	avengers_nand_info.parts[0] = avengers_nand_partitions_0;
	avengers_nand_info.nr_parts[0] = ARRAY_SIZE(avengers_nand_partitions_0);
	avengers_nand_info.parts[1] = avengers_nand_partitions_1;
	avengers_nand_info.nr_parts[1] = ARRAY_SIZE(avengers_nand_partitions_1);
	avengers_nand_info.controller_attrs = PXA3XX_ARBI_EN | PXA3XX_NAKED_CMD_EN
		| PXA3XX_DMA_EN | PXA3XX_ADV_TIME_TUNING | PXA3XX_TWO_CHIP_EN;
	pxa168_add_nand(&avengers_nand_info);
}

static void __init avengers_lite_init(void)
{
	mfp_config(ARRAY_AND_SIZE(avengers_lite_pin_config_V16F));

	/* on-chip devices */
	pxa168_add_uart(2);
	avengers_init_flash();
}

MACHINE_START(AVENGERS_LITE, "PXA168 Avengers lite Development Platform")
	.map_io		= pxa_map_io,
	.init_irq       = pxa168_init_irq,
	.timer          = &pxa168_timer,
	.init_machine   = avengers_lite_init,
MACHINE_END
