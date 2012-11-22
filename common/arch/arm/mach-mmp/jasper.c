/*
 *  linux/arch/arm/mach-mmp/jasper.c
 *
 *  Support for the Marvell Jasper Development Platform.
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
#include <linux/interrupt.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/addr-map.h>
#include <mach/mfp-mmp2.h>
#include <mach/mmp2.h>
#include <plat/misc.h>

#include "common.h"

#ifdef CONFIG_SD8XXX_RFKILL
#include <linux/sd8x_rfkill.h>
#endif

#define JASPER_NR_IRQS		(IRQ_BOARD_START + 48)

static unsigned long jasper_pin_config[] __initdata = {
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

	/* MMC0 */
	GPIO131_MMC1_DAT3,
	GPIO132_MMC1_DAT2,
	GPIO133_MMC1_DAT1,
	GPIO134_MMC1_DAT0,
	GPIO136_MMC1_CMD,
	GPIO139_MMC1_CLK,
	GPIO140_MMC1_CD,
	GPIO141_MMC1_WP,

	/* MMC1 */
	GPIO37_MMC2_DAT3,
	GPIO38_MMC2_DAT2,
	GPIO39_MMC2_DAT1,
	GPIO40_MMC2_DAT0,
	GPIO41_MMC2_CMD,
	GPIO42_MMC2_CLK,

	/* MMC2 */
	GPIO165_MMC3_DAT7,
	GPIO162_MMC3_DAT6,
	GPIO166_MMC3_DAT5,
	GPIO163_MMC3_DAT4,
	GPIO167_MMC3_DAT3,
	GPIO164_MMC3_DAT2,
	GPIO168_MMC3_DAT1,
	GPIO111_MMC3_DAT0,
	GPIO112_MMC3_CMD,
	GPIO151_MMC3_CLK,

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

static struct max8649_platform_data jasper_max8649_info = {
	.mode		= 2,	/* VID1 = 1, VID0 = 0 */
	.extclk		= 0,
	.ramp_timing	= MAX8649_RAMP_32MV,
	.regulator	= &max8649_init_data,
};

static struct regulator_consumer_supply max8925_regulator_supply[] = {
	[0] = {
		.supply	= "DBVDD",
	},
	[1] = {
		.supply	= "AVDD2",
	},
	[2] = {
		.supply = "CPVDD",
	},
};


static struct regulator_consumer_supply regulator_supply[] = {
	[MAX8925_ID_SD1]	= REGULATOR_SUPPLY("v_sd1", NULL),
	[MAX8925_ID_SD2]	= REGULATOR_SUPPLY("v_sd2", NULL),
	[MAX8925_ID_SD3]	= REGULATOR_SUPPLY("v_sd3", NULL),
	[MAX8925_ID_LDO1]	= REGULATOR_SUPPLY("v_ldo1", NULL),
	[MAX8925_ID_LDO2]	= REGULATOR_SUPPLY("v_ldo2", NULL),
	[MAX8925_ID_LDO3]	= REGULATOR_SUPPLY("v_ldo3", NULL),
	[MAX8925_ID_LDO4]	= REGULATOR_SUPPLY("v_ldo4", NULL),
	[MAX8925_ID_LDO5]	= REGULATOR_SUPPLY("v_ldo5", NULL),
	[MAX8925_ID_LDO6]	= REGULATOR_SUPPLY("v_ldo6", NULL),
	[MAX8925_ID_LDO7]	= REGULATOR_SUPPLY("v_ldo7", NULL),
	[MAX8925_ID_LDO8]	= REGULATOR_SUPPLY("v_ldo8", NULL),
	[MAX8925_ID_LDO9]	= REGULATOR_SUPPLY("v_ldo9", NULL),
	[MAX8925_ID_LDO10]	= REGULATOR_SUPPLY("v_ldo10", NULL),
	[MAX8925_ID_LDO11]	= REGULATOR_SUPPLY("v_ldo11", NULL),
	[MAX8925_ID_LDO12]	= REGULATOR_SUPPLY("v_ldo12", NULL),
	[MAX8925_ID_LDO13]	= REGULATOR_SUPPLY("v_ldo13", NULL),
	[MAX8925_ID_LDO14]	= REGULATOR_SUPPLY("v_ldo14", NULL),
	[MAX8925_ID_LDO15]	= REGULATOR_SUPPLY("v_ldo15", NULL),
	[MAX8925_ID_LDO16]	= REGULATOR_SUPPLY("v_ldo16", NULL),
	[MAX8925_ID_LDO17]	= REGULATOR_SUPPLY("v_ldo17", NULL),
	[MAX8925_ID_LDO18]	= REGULATOR_SUPPLY("v_ldo18", NULL),
	[MAX8925_ID_LDO19]	= REGULATOR_SUPPLY("v_ldo19", NULL),
	[MAX8925_ID_LDO20]	= REGULATOR_SUPPLY("v_ldo20", NULL),
};

#define REG_INIT(_name, _min, _max, _always, _boot)		\
{								\
	.constraints = {					\
		.name		= __stringify(_name),		\
		.min_uV		= _min,				\
		.max_uV		= _max,				\
		.always_on	= _always,			\
		.boot_on	= _boot,			\
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE	\
				| REGULATOR_CHANGE_STATUS,	\
	},							\
	.num_consumer_supplies	= 1,				\
	.consumer_supplies	= &regulator_supply[MAX8925_ID_##_name], \
}

static struct regulator_init_data regulator_data[] = {
	[MAX8925_ID_SD1] = REG_INIT(SD1, 637500, 1425000, 0, 0),
	[MAX8925_ID_SD2] = {
		.constraints	= {
			.name		= "SD2",
			.min_uV		= 650000,
			.max_uV		= 2225000,
			.always_on	= 1,
			.boot_on	= 1,
		},
		.num_consumer_supplies	= ARRAY_SIZE(max8925_regulator_supply),
		.consumer_supplies	= max8925_regulator_supply,
	},
	[MAX8925_ID_SD3] = REG_INIT(SD3, 750000, 3900000, 1, 1),
	[MAX8925_ID_LDO1] = REG_INIT(LDO1, 750000, 3900000, 1, 1),
	[MAX8925_ID_LDO2] = REG_INIT(LDO2, 650000, 2250000, 1, 1),
	[MAX8925_ID_LDO3] = REG_INIT(LDO3, 650000, 2250000, 0, 1),
	[MAX8925_ID_LDO4] = REG_INIT(LDO4, 750000, 3900000, 1, 1),
	[MAX8925_ID_LDO5] = REG_INIT(LDO5, 750000, 3900000, 0, 0),
	[MAX8925_ID_LDO6] = REG_INIT(LDO6, 750000, 3900000, 0, 0),
	[MAX8925_ID_LDO7] = REG_INIT(LDO7, 750000, 3900000, 1, 1),
	[MAX8925_ID_LDO8] = REG_INIT(LDO8, 750000, 3900000, 1, 1),
	[MAX8925_ID_LDO9] = REG_INIT(LDO9, 750000, 3900000, 1, 1),
	[MAX8925_ID_LDO10] = REG_INIT(LDO10, 750000, 3900000, 0, 0),
	[MAX8925_ID_LDO11] = REG_INIT(LDO11, 750000, 3900000, 1, 1),
	[MAX8925_ID_LDO12] = REG_INIT(LDO12, 750000, 3900000, 0, 0),
	[MAX8925_ID_LDO13] = REG_INIT(LDO13, 750000, 3900000, 0, 0),
	[MAX8925_ID_LDO14] = REG_INIT(LDO14, 750000, 3900000, 1, 1),
	[MAX8925_ID_LDO15] = REG_INIT(LDO15, 750000, 3900000, 1, 1),
	[MAX8925_ID_LDO16] = REG_INIT(LDO16, 750000, 3900000, 0, 0),
	[MAX8925_ID_LDO17] = REG_INIT(LDO17, 650000, 2250000, 1, 1),
	[MAX8925_ID_LDO18] = REG_INIT(LDO18, 650000, 2250000, 1, 1),
	[MAX8925_ID_LDO19] = REG_INIT(LDO19, 750000, 3900000, 0, 0),
	[MAX8925_ID_LDO20] = REG_INIT(LDO20, 750000, 3900000, 1, 1),
};

static struct max8925_backlight_pdata jasper_backlight_data = {
	.dual_string	= 0,
};

static struct max8925_power_pdata jasper_power_data = {
	.batt_detect		= 0,	/* can't detect battery by ID pin */
	.topoff_threshold	= MAX8925_TOPOFF_THR_10PER,
	.fast_charge		= MAX8925_FCHG_1000MA,
};

static struct max8925_platform_data jasper_max8925_info = {
	.backlight		= &jasper_backlight_data,
	.power			= &jasper_power_data,
	.irq_base		= IRQ_BOARD_START,

	.regulator[MAX8925_ID_SD1] = &regulator_data[MAX8925_ID_SD1],
	.regulator[MAX8925_ID_SD2] = &regulator_data[MAX8925_ID_SD2],
	.regulator[MAX8925_ID_SD3] = &regulator_data[MAX8925_ID_SD3],
	.regulator[MAX8925_ID_LDO1] = &regulator_data[MAX8925_ID_LDO1],
	.regulator[MAX8925_ID_LDO2] = &regulator_data[MAX8925_ID_LDO2],
	.regulator[MAX8925_ID_LDO3] = &regulator_data[MAX8925_ID_LDO3],
	.regulator[MAX8925_ID_LDO4] = &regulator_data[MAX8925_ID_LDO4],
	.regulator[MAX8925_ID_LDO5] = &regulator_data[MAX8925_ID_LDO5],
	.regulator[MAX8925_ID_LDO6] = &regulator_data[MAX8925_ID_LDO6],
	.regulator[MAX8925_ID_LDO7] = &regulator_data[MAX8925_ID_LDO7],
	.regulator[MAX8925_ID_LDO8] = &regulator_data[MAX8925_ID_LDO8],
	.regulator[MAX8925_ID_LDO9] = &regulator_data[MAX8925_ID_LDO9],
	.regulator[MAX8925_ID_LDO10] = &regulator_data[MAX8925_ID_LDO10],
	.regulator[MAX8925_ID_LDO11] = &regulator_data[MAX8925_ID_LDO11],
	.regulator[MAX8925_ID_LDO12] = &regulator_data[MAX8925_ID_LDO12],
	.regulator[MAX8925_ID_LDO13] = &regulator_data[MAX8925_ID_LDO13],
	.regulator[MAX8925_ID_LDO14] = &regulator_data[MAX8925_ID_LDO14],
	.regulator[MAX8925_ID_LDO15] = &regulator_data[MAX8925_ID_LDO15],
	.regulator[MAX8925_ID_LDO16] = &regulator_data[MAX8925_ID_LDO16],
	.regulator[MAX8925_ID_LDO17] = &regulator_data[MAX8925_ID_LDO17],
	.regulator[MAX8925_ID_LDO18] = &regulator_data[MAX8925_ID_LDO18],
	.regulator[MAX8925_ID_LDO19] = &regulator_data[MAX8925_ID_LDO19],
	.regulator[MAX8925_ID_LDO20] = &regulator_data[MAX8925_ID_LDO20],
};

static struct i2c_board_info jasper_twsi1_info[] = {
	[0] = {
		.type		= "max8649",
		.addr		= 0x60,
		.platform_data	= &jasper_max8649_info,
	},
	[1] = {
		.type		= "max8925",
		.addr		= 0x3c,
		.irq		= IRQ_MMP2_PMIC,
		.platform_data	= &jasper_max8925_info,
	},
};

static struct mtd_partition jasper_nand_partitions[] = {
	[0] = {
		.name		= "Bootloader",
		.offset		= 0,
		.size		= 0x100000,
		.mask_flags	= MTD_WRITEABLE,
	},
	[1] = {
		.name		= "Reserve",
		.offset		= 0x100000,
		.size		= 0x080000,
	},
	[2] = {
		.name		= "Reserve",
		.offset		= 0x180000,
		.size		= 0x800000,
		.mask_flags	= MTD_WRITEABLE,
	},
	[3] = {
		.name		= "Kernel",
		.offset		= 0x980000,
		.size		= 0x300000,
		.mask_flags	= MTD_WRITEABLE,
	},
	[4] = {
		.name		= "system",
		.offset		= 0x0c80000,
		.size		= 0x7000000,
	},
	[5] = {
		.name		= "userdata",
		.offset		= 0x7c80000,
		.size		= 0x7000000,
	},
	[6] = {
		.name		= "filesystem",
		.offset		= 0x0ec80000,
		.size		= MTDPART_SIZ_FULL,
	}
};

static struct pxa3xx_nand_platform_data jasper_nand_info;
static void __init jasper_init_flash(void)
{
	jasper_nand_info.parts[0] = jasper_nand_partitions;
	jasper_nand_info.nr_parts[0] = ARRAY_SIZE(jasper_nand_partitions);
	jasper_nand_info.controller_attrs = PXA3XX_ARBI_EN | PXA3XX_NAKED_CMD_EN
		| PXA3XX_DMA_EN | PXA3XX_ADV_TIME_TUNING;
	mmp2_add_nand(&jasper_nand_info);
}

#if defined(CONFIG_MMC_SDHCI_PXA)

/* MMC1 controller for SD-MMC */
static struct sdhci_pxa_platdata mmp2_sdh_platdata_mmc0 = {
	.max_speed	= 25000000,
};

static struct sdhci_pxa_platdata mmp2_sdh_platdata_mmc1 = {
	.max_speed	= 50000000,
	.pxa_quirk	= PXA_QUIRK_BROKEN_CARD_DETECTION,
};

static struct sdhci_pxa_platdata mmp2_sdh_platdata_mmc2 = {
	.max_speed	= 50000000,
	.pxa_quirk	= PXA_QUIRK_BROKEN_CARD_DETECTION |
				PXA_QUIRK_DISABLE_CLOCK_GATING,
};

static void __init mmp2_init_mmc(void)
{
#ifdef CONFIG_SD8XXX_RFKILL
	int WIB_PDn;
	int WIB_RESETn;

	WIB_PDn = mfp_to_gpio(GPIO57_GPIO57);
	WIB_RESETn = mfp_to_gpio(GPIO58_GPIO58);

	add_sd8x_rfkill_device(WIB_PDn, WIB_RESETn,
			&mmp2_sdh_platdata_mmc1.pmmc, NULL);
#endif
	mmp2_add_sdh(0, &mmp2_sdh_platdata_mmc0); /* SD/MMC */
	mmp2_add_sdh(1, &mmp2_sdh_platdata_mmc1); /* wifi */
#ifndef CONFIG_MTD_NAND_PXA3xx
	/*eMMC (MMC3) pins are conflict with NAND*/
	mmp2_add_sdh(2, &mmp2_sdh_platdata_mmc2); /* eMMC */
#endif
}

#endif

static void __init jasper_init(void)
{
	mfp_config(ARRAY_AND_SIZE(jasper_pin_config));
	check_board_version();

	/* on-chip devices */
	mmp2_add_uart(1);
	mmp2_add_uart(3);
	mmp2_add_twsi(1, NULL, ARRAY_AND_SIZE(jasper_twsi1_info));

#if defined(CONFIG_MMC_SDHCI_PXA)
	mmp2_init_mmc();
#endif

	regulator_has_full_constraints();
	jasper_init_flash();
}

MACHINE_START(MARVELL_JASPER, "Jasper Development Platform")
	.map_io		= pxa_map_io,
	.nr_irqs	= JASPER_NR_IRQS,
	.init_irq       = mmp2_init_irq,
	.timer          = &mmp2_timer,
	.init_machine   = jasper_init,
MACHINE_END
