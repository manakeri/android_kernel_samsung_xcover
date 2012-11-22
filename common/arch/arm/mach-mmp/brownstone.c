/*
 *  linux/arch/arm/mach-mmp/brownstone.c
 *
 *  Support for the Marvell Brownstone Development Platform.
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
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/max8649.h>
#include <linux/mfd/max8925.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/addr-map.h>
#include <mach/mfp-mmp2.h>
#include <mach/mmp2.h>
#include <mach/irqs.h>
#include <plat/misc.h>

#include "common.h"

#define BROWNSTONE_NR_IRQS		(IRQ_BOARD_START + 48)

static unsigned long brownstone_pin_config[] __initdata = {
	/* UART1 */
	GPIO29_UART1_RXD,
	GPIO30_UART1_TXD,

	/* UART3 */
	GPIO51_UART3_RXD,
	GPIO52_UART3_TXD,

	/* DFI */
	GPIO168_DFI_D0,
	GPIO167_DFI_D1,
	GPIO166_DFI_D2,
	GPIO165_DFI_D3,
	GPIO107_DFI_D4,
	GPIO106_DFI_D5,
	GPIO105_DFI_D6,
	GPIO104_DFI_D7,
	GPIO111_DFI_D8,
	GPIO164_DFI_D9,
	GPIO163_DFI_D10,
	GPIO162_DFI_D11,
	GPIO161_DFI_D12,
	GPIO110_DFI_D13,
	GPIO109_DFI_D14,
	GPIO108_DFI_D15,
	GPIO143_ND_nCS0,
	GPIO144_ND_nCS1,
	GPIO147_ND_nWE,
	GPIO148_ND_nRE,
	GPIO150_ND_ALE,
	GPIO149_ND_CLE,
	GPIO112_ND_RDY0,
	GPIO160_ND_RDY1,

	/* PMIC */
	PMIC_PMIC_INT | MFP_LPM_EDGE_FALL,

	/* board version */
	GPIO126_GPIO126,
	GPIO127_GPIO127,
	GPIO128_GPIO128,
};

static void __init check_board_version(void)
{
	struct gpio ver_gpios[] = {
		{mfp_to_gpio(GPIO126_GPIO126), GPIOF_DIR_IN, "Version Bit0"},
		{mfp_to_gpio(GPIO127_GPIO127), GPIOF_DIR_IN, "Version Bit1"},
		{mfp_to_gpio(GPIO128_GPIO128), GPIOF_DIR_IN, "Version Bit2"},
	};
	int version;

	if (gpio_request_array(ver_gpios, ARRAY_SIZE(ver_gpios))) {
		pr_info("Failed to request GPIOs for board version!\n");
		return;
	}
	version = !!gpio_get_value(ver_gpios[0].gpio)
		| !!gpio_get_value(ver_gpios[1].gpio) << 1
		| !!gpio_get_value(ver_gpios[2].gpio) << 2;

	gpio_free_array(ver_gpios, ARRAY_SIZE(ver_gpios));
	set_board_version(version);
}

static struct regulator_consumer_supply max8649_supply[] = {
	REGULATOR_SUPPLY("vcc_core", NULL),
};

static struct regulator_init_data max8649_init_data = {
	.constraints	= {
		.name		= "vcc_core range",
		.min_uV		= 1150000,
		.max_uV		= 1280000,
		.always_on	= 1,
		.boot_on	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max8649_supply[0],
};

static struct max8649_platform_data brownstone_max8649_info = {
	.mode		= 2,	/* VID1 = 1, VID0 = 0 */
	.extclk		= 0,
	.ramp_timing	= MAX8649_RAMP_32MV,
	.regulator	= &max8649_init_data,
};

static struct max8925_platform_data brownstone_max8925_info = {
	.irq_base		= IRQ_BOARD_START,
};

static struct i2c_board_info brownstone_twsi1_info[] = {
	[0] = {
		.type		= "max8649",
		.addr		= 0x60,
		.platform_data	= &brownstone_max8649_info,
	},
	[1] = {
		.type		= "max8925",
		.addr		= 0x3c,
		.irq		= IRQ_MMP2_PMIC,
		.platform_data	= &brownstone_max8925_info,
	},
};

static void __init brownstone_init(void)
{
	mfp_config(ARRAY_AND_SIZE(brownstone_pin_config));
	check_board_version();

	/* on-chip devices */
	mmp2_add_uart(1);
	mmp2_add_uart(3);
	mmp2_add_twsi(1, NULL, ARRAY_AND_SIZE(brownstone_twsi1_info));
}

MACHINE_START(BROWNSTONE, "Brownstone Development Platform")
	.map_io		= pxa_map_io,
	.nr_irqs	= BROWNSTONE_NR_IRQS,
	.init_irq	= mmp2_init_irq,
	.timer          = &mmp2_timer,
	.init_machine   = brownstone_init,
MACHINE_END
