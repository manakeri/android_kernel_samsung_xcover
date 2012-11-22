/*
 *  linux/arch/arm/mach-mmp/abilene.c
 *
 *  Support for the Marvell MMP3 Abilene Development Platform.
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
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/ds4432.h>
#include <linux/mfd/max8925.h>
#include <linux/pwm_backlight.h>
#include <linux/mfd/wm8994/pdata.h>
#include <linux/regulator/fixed.h>
#include <linux/smc91x.h>
#include <linux/i2c/tpk_r800.h>
#include <linux/usb/android_composite.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/setup.h>
#include <asm/irq.h>
#include <mach/addr-map.h>
#include <mach/mfp-mmp2.h>
#include <mach/mmp3.h>
#include <mach/irqs.h>
#include <mach/regs-mpmu.h>
#include <mach/soc_vmeta.h>
#include <mach/tc35876x.h>
#include <plat/pmem.h>
#include <plat/vbus.h>
#include <plat/pxa_u2o.h>

#include "common.h"
#include "onboard.h"

#define ABILENE_NR_IRQS			(IRQ_BOARD_START + 64)

#define MMP_VENDOR_ID				0x0BB4
#define MMP_ALL_PRODUCT_ID			0x4E20
#define MMP_MODEM_DIAG_UMS_PRODUCT_ID		0x4E21
#define MMP_MODEM_DIAG_UMS_ADB_PRODUCT_ID	0x4E22
#define MMP_RNDIS_MODEM_DIAG_PRODUCT_ID		0x4E23
#define MMP_RNDIS_MODEM_DIAG_ADB_PRODUCT_ID	0x4E24
#define MMP_RNDIS_PRODUCT_ID			0x4E25
#define MMP_UMS_PRODUCT_ID			0x4E26
#define MMP_MODEM_DIAG_PRODUCT_ID		0x4E27
#define MMP_UMS_ADB_PRODUCT_ID			0x4E28
#define MMP_RNDIS_ADB_PRODUCT_ID		0x4E29
#define MMP_MODEM_UMS_ADB_PRODUCT_ID		0x4E2A
#define MMP_MODEM_UMS_PRODUCT_ID		0x4E2B
#define MMP_DIAG_PRODUCT_ID			0x4E2C

#ifdef CONFIG_UIO_VMETA
static struct vmeta_plat_data mmp_vmeta_plat_data = {
	.bus_irq_handler = NULL,
	.set_dvfm_constraint = NULL,
	.unset_dvfm_constraint = NULL,
	.axi_clk_available = 0,
};

static void __init mmp_init_vmeta(void)
{
	mmp_set_vmeta_info(&mmp_vmeta_plat_data);
}
#endif

static unsigned long abilene_pin_config[] __initdata = {
	/* UART1 */
	GPIO29_UART1_RXD,
	GPIO30_UART1_TXD,
	/* TWSI5 */
	GPIO99_TWSI5_SCL,
	GPIO100_TWSI5_SDA,

	/* TWSI6 */
	GPIO97_TWSI6_SCL,
	GPIO98_TWSI6_SDA,

	/* TWSI1 */
	TWSI1_SCL | MFP_LPM_PULL_LOW,
	TWSI1_SDA | MFP_LPM_PULL_LOW,

	/* TWSI2 */
	GPIO43_TWSI2_SCL,
	GPIO44_TWSI2_SDA,
	/* TWSI3 */
	GPIO71_TWSI3_SCL,
	GPIO72_TWSI3_SDA,

	/* SSPA1 (I2S) */
	GPIO23_GPIO23,
	GPIO24_I2S_SYSCLK,
	GPIO25_I2S_BITCLK,
	GPIO26_I2S_SYNC,
	GPIO27_I2S_DATA_OUT,
	GPIO28_I2S_SDATA_IN,

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

	/* ETHERNET */
	GPIO146_SMC_nCS1,
	GPIO152_SMC_BE0,
	GPIO153_SMC_BE1,
	GPIO154_SM_ADVMUX,
	GPIO155_GPIO155,	/* interrupt */

	/* Keypad */
	GPIO16_KP_DKIN0 | MFP_PULL_HIGH,
	GPIO17_KP_DKIN1 | MFP_PULL_HIGH,
	GPIO18_KP_DKIN2 | MFP_PULL_HIGH,
	GPIO19_KP_DKIN3 | MFP_PULL_HIGH,

	PMIC_PMIC_INT | MFP_LPM_EDGE_FALL,
	GPIO45_WM8994_LDOEN,

	/* HSIC1 */
	GPIO62_VBUS_EN,
	GPIO96_HSIC_RESET,
};

static unsigned long lcd_pin_config_zx[] __initdata = {
	GPIO135_LCD_RST,
};

static unsigned long lcd_pin_config_a0[] __initdata = {
	GPIO128_LCD_RST,
};

static unsigned long mmc1_pin_config_zx[] __initdata = {
	/* MMC0 */
	GPIO131_MMC1_DAT3,
	GPIO132_MMC1_DAT2,
	GPIO133_MMC1_DAT1,
	GPIO134_MMC1_DAT0,
	GPIO136_MMC1_CMD,
	GPIO139_MMC1_CLK,
	GPIO140_MMC1_CD | MFP_PULL_HIGH,
	GPIO141_MMC1_WP | MFP_PULL_HIGH,
};

static unsigned long mmc1_pin_config_a0[] __initdata = {
	/* MMC0 */
	GPIO131_MMC1_DAT3,
	GPIO132_MMC1_DAT2,
	GPIO133_MMC1_DAT1,
	GPIO134_MMC1_DAT0,
	GPIO136_MMC1_CMD,
	GPIO135_MMC1_CLK,
	GPIO140_MMC1_CD | MFP_PULL_HIGH,
	GPIO141_MMC1_WP | MFP_PULL_HIGH,
};

static unsigned long emmc_pin_config_zx[] __initdata = {
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
};

static unsigned long emmc_pin_config_a0[] __initdata = {
	GPIO161_MMC3_DAT7,
	GPIO145_MMC3_DAT6,
	GPIO162_MMC3_DAT5,
	GPIO146_MMC3_DAT4,
	GPIO163_MMC3_DAT3,
	GPIO108_MMC3_DAT2,
	GPIO164_MMC3_DAT1,
	GPIO109_MMC3_DAT0,
	GPIO111_MMC3_CMD,
	GPIO110_MMC3_CLK,
};

static int emmc_boot = 0;
static int __init emmc_setup(char *__unused)
{
#if defined(CONFIG_MMC_SDHCI_PXA)
	emmc_boot = 1;
#endif
	return 1;
}
__setup("emmc_boot", emmc_setup);

#if defined(CONFIG_SMC91X)
static struct resource smc91x_resources[] = {
	[0] = {
		.start	= SMC_CS1_PHYS_BASE + 0x300,
		.end	= SMC_CS1_PHYS_BASE + 0xfffff,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_GPIO(mfp_to_gpio(MFP_PIN_GPIO155)),
		.end	= IRQ_GPIO(mfp_to_gpio(MFP_PIN_GPIO155)),
		.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE,
	}
};

static struct smc91x_platdata abilene_smc91x_info = {
	.flags	= SMC91X_USE_16BIT | SMC91X_NOWAIT,
};

static struct platform_device smc91x_device = {
	.name		= "smc91x",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(smc91x_resources),
	.resource	= smc91x_resources,
	.dev		= {
		.platform_data = &abilene_smc91x_info,
	},
};
#endif

#ifdef CONFIG_MTD_NAND_PXA3xx
static struct mtd_partition abilene_nand_partitions[] = {
	[0] = {
		.name = "Bootloader",
		.offset = 0x100000,
		.size = 0x100000,
		.mask_flags = MTD_WRITEABLE,
		},
	[1] = {
		.name = "Reserve",
		.offset = 0x200000,
		.size = 0x0100000,
		},
	[2] = {
		.name = "Reserve",
		.offset = 0x300000,
		.size = 0x700000,
		.mask_flags = MTD_WRITEABLE,
		},
	[3] = {
		.name = "Kernel",
		.offset = 0xa00000,
		.size = 0x400000,
		.mask_flags = MTD_WRITEABLE,
		},
	[4] = {
		.name = "Filesystem",
		.offset = 0x0e00000,
		.size = 0x10000000,	/* 256 MB */
		},
	[5] = {
		.name = "MassStorage",
		.offset = 0x10e00000,
		.size = 0x29200000,
		},
	[6] = {
		.name = "BBT",
		.offset = 0x3a000000,
		.size = MTDPART_SIZ_FULL,
		.mask_flags  = MTD_WRITEABLE,  /* force read-only */ \
		}
};

static struct pxa3xx_nand_platform_data abilene_nand_info;
static void __init abilene_init_flash(void)
{
	if (emmc_boot)
		return;

	abilene_nand_info.parts[0] = abilene_nand_partitions;
	abilene_nand_info.nr_parts[0] =
		ARRAY_SIZE(abilene_nand_partitions);
	abilene_nand_info.controller_attrs = PXA3XX_ARBI_EN | PXA3XX_NAKED_CMD_EN;
	mmp3_add_nand(&abilene_nand_info);
}
#endif

extern int pxa_usb_phy_init(unsigned int base);
static struct pxa_usb_plat_info mmp3_u2o_info = {
	.phy_init	= pxa_usb_phy_init,
	.is_otg		= 1,
	.in_single	= 1,
	.vbus_detect	= NULL,
};

static int mmp3_hsic1_reset(void)
{
	int reset;
	reset = mfp_to_gpio(GPIO96_HSIC_RESET);

	if (gpio_request(reset, "hsic reset")) {
		pr_err("Failed to request hsic reset gpio\n");
		return -EIO;
	}

	gpio_direction_output(reset, 0);
	mdelay(100);
	gpio_direction_output(reset, 1);

	gpio_free(reset);
	return 0;
}

static int mmp3_hsic1_set_vbus(int on)
{
	static struct regulator *ldo5;
	int gpio = mfp_to_gpio(GPIO62_VBUS_EN);

	printk("%s: set %d\n", __func__, on);
	if (!ldo5) {
		ldo5 = regulator_get(NULL, "v_ldo5");
		if (IS_ERR(ldo5)) {
			printk(KERN_INFO "ldo5 not found\n");
			return -EIO;
		}
		regulator_set_voltage(ldo5, 1200000, 1200000);
		regulator_enable(ldo5);
		printk("%s: enable regulator\n", __func__);
	}

	/* 5V power supply to external port */
	if (gpio_request(gpio, "HSIC1 VBUS Enable")) {
		printk(KERN_INFO "gpio %d request failed\n", gpio);
		return -1;
	}

	if (on) {
		gpio_direction_output(gpio, 1);
	}
	else {
		gpio_direction_output(gpio, 0);
	}

	gpio_free(gpio);

	mmp3_hsic1_reset();
	return 0;
}

extern int mmp3_hsic_phy_init(unsigned int base);
static struct pxa_usb_plat_info mmp3_hsic_info = {
	.phy_init	= mmp3_hsic_phy_init,
	.vbus_set	= mmp3_hsic1_set_vbus
};

#ifdef CONFIG_USB_ANDROID
#if defined(CONFIG_USB_ANDROID_MASS_STORAGE) || defined(CONFIG_USB_FILE_STORAGE)
static struct usb_mass_storage_platform_data mass_storage_pdata = {
       .nluns          = 1,
       .vendor         = "Marvell",
       .product        = "Android",
       .release        = 0x0100,
};

static struct platform_device usb_mass_storage_device = {
       .name   = "usb_mass_storage",
       .id     = -1,
       .dev    = {
		.platform_data  = &mass_storage_pdata,
       },
};
#endif

#if defined(CONFIG_USB_ANDROID_RNDIS)
static struct usb_ether_platform_data usb_rndis_pdata = {
       .ethaddr        = {11, 22, 33, 44, 55, 66},
       .vendorID       = MMP_VENDOR_ID,
       .vendorDescr    = "Marvell Rndis function"
};

static struct platform_device usb_rndis_device = {
       .name   = "rndis",
       .id     = -1,
       .dev    = {
		.platform_data  = &usb_rndis_pdata,
       },
};
#endif

/* include all existing functions in default function list,
thus all are binded at initialization */
static char *usb_functions_all[] = {
#ifdef CONFIG_USB_ANDROID_RNDIS
	"rndis",
#endif
#if defined(CONFIG_USB_ANDROID_MASS_STORAGE) || defined(CONFIG_USB_FILE_STORAGE)
	"usb_mass_storage",
#endif
#ifdef CONFIG_USB_ANDROID_ADB
	"adb",
#endif
};

static struct android_usb_product usb_products[] = {
	{
		.product_id = MMP_ALL_PRODUCT_ID,
		.num_functions = ARRAY_SIZE(usb_functions_all),
		.functions = usb_functions_all,
	},
};

static struct android_usb_platform_data android_usb_pdata = {
	.vendor_id              = MMP_VENDOR_ID,
	.product_id             = MMP_ALL_PRODUCT_ID,
	.version                = 0x0100,
	.product_name           = "Android",
	.manufacturer_name      = "Marvell",
	.num_functions          = ARRAY_SIZE(usb_functions_all),
	.functions              = usb_functions_all,
	.num_products		= ARRAY_SIZE(usb_products),
	.products		= usb_products,
};

static struct platform_device android_device_usb = {
       .name   = "android_usb",
       .id     = -1,
       .dev    = {
               .platform_data  = &android_usb_pdata,
       },
};

void __init mmp3_android_add_usb_devices(void)
{
#if defined(CONFIG_USB_ANDROID_MASS_STORAGE) || defined(CONFIG_USB_FILE_STORAGE)
       platform_device_register(&usb_mass_storage_device);
#endif

#if defined(CONFIG_USB_ANDROID_RNDIS)
       platform_device_register(&usb_rndis_device);
#endif
       platform_device_register(&android_device_usb);
}

#endif

static struct regulator_consumer_supply max8925_regulator_supply[] = {
	[0] = {
		.supply = "DBVDD",
		},
	[1] = {
		.supply = "AVDD2",
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
				.constraints = {
						.name = "SD2",
						.min_uV = 650000,
						.max_uV = 2225000,
						.always_on = 1,
						.boot_on = 1,
						},
				.num_consumer_supplies =
				ARRAY_SIZE(max8925_regulator_supply),
				.consumer_supplies = max8925_regulator_supply,
				},
	[MAX8925_ID_SD3] = REG_INIT(SD3, 750000, 3900000, 1, 1),
	[MAX8925_ID_LDO1] = REG_INIT(LDO1, 750000, 3900000, 1, 1),
	[MAX8925_ID_LDO2] = REG_INIT(LDO2, 650000, 2250000, 1, 1),
	[MAX8925_ID_LDO3] = REG_INIT(LDO3, 1000000, 1500000, 0, 0),
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
	[MAX8925_ID_LDO17] = REG_INIT(LDO17, 1000000, 1500000, 0, 0),
	[MAX8925_ID_LDO18] = REG_INIT(LDO18, 650000, 2250000, 1, 1),
	[MAX8925_ID_LDO19] = REG_INIT(LDO19, 750000, 3900000, 0, 0),
	[MAX8925_ID_LDO20] = REG_INIT(LDO20, 750000, 3900000, 1, 1),
};

static struct max8925_platform_data abilene_max8925_info = {
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

static struct i2c_board_info abilene_twsi1_info[] = {
	{
		.type		= "max8925",
		.addr		= 0x3c,
		.irq		= IRQ_MMP2_PMIC,
		.platform_data	= &abilene_max8925_info,
	},
};

static int max17083_ds4432_convert(int path, int mode,
			int iparam, int *oparam) {
	if (mode == DS4432_DCDC_VOLTAGE_TO_CURRENT)
		*oparam = (iparam - 1273000) / 100; /* vV -> 10 nA */
	else
		*oparam = iparam * 100 + 1273000; /* 10 nA -> vV */
	return 0;
}

static struct regulator_consumer_supply ds4432_supply[] = {
	REGULATOR_SUPPLY("vcc_main", NULL),
};

static struct ds4432_dac_data ds4432_data[] = {
	[0] = {
		.initdat = {
			.constraints    = {
				.name           = "vcc_main range",
				.min_uV         = 876280,
				.max_uV         = 1474000,
				.always_on      = 1,
				.boot_on        = 1,
				.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
			},
			.num_consumer_supplies  = 1,
			.consumer_supplies      = &ds4432_supply[0],
		},
		.name = "max17083+ds4432",
		.type = REGULATOR_VOLTAGE,
		.dac_path = 1,
		.cstep_10nA = 62, /* (0.997/(16*100000))*100000000 10nA */
		.param_convert = max17083_ds4432_convert,
	},
	/* ds4432 has two paths, we may register two here, however
	   the two seems to be tied together on current board. we need to
	   keep one unused and the other to do real control
	*/
};

static struct ds4432_platform_data abilene_ds4432_info = {
	.regulator_count = sizeof(ds4432_data)/sizeof(ds4432_data[0]),
	.regulators = ds4432_data,

};

static struct i2c_board_info abilene_twsi6_info[] = {
	{
		.type		= "ds4432",
		.addr		= 0x48,
		.platform_data	= &abilene_ds4432_info,
	},
};

#if defined(CONFIG_REGULATOR_WM8994)
static int wm8994_ldoen(void)
{
	int gpio = mfp_to_gpio(GPIO45_WM8994_LDOEN);

	if (gpio_request(gpio, "wm8994 ldoen gpio")) {
		printk(KERN_INFO "gpio %d request failed\n", gpio);
		return -1;
	}

	gpio_direction_output(gpio, 1);
	mdelay(1);
	gpio_free(gpio);

	return 0;
}

static struct regulator_consumer_supply abilene_wm8994_regulator_supply[] = {
	[0] = {
		.supply = "AVDD1",
		},
	[1] = {
		.supply = "DCVDD",
		},
};

struct regulator_init_data abilene_wm8994_regulator_init_data[] = {
	[0] = {
		.constraints = {
				.name = "wm8994-ldo1",
				.min_uV = 2400000,
				.max_uV = 3100000,
				.always_on = 1,
				.boot_on = 1,
				},
		.num_consumer_supplies = 1,
		.consumer_supplies = &abilene_wm8994_regulator_supply[0],
		},
	[1] = {
		.constraints = {
				.name = "wm8994-ldo2",
				.min_uV = 900000,
				.max_uV = 1200000,
				.always_on = 1,
				.boot_on = 1,
				},
		.num_consumer_supplies = 1,
		.consumer_supplies = &abilene_wm8994_regulator_supply[1],
		},
};

struct wm8994_pdata abilene_wm8994_pdata = {
	.ldo[0] = {
			.enable = 0,
			.init_data = &abilene_wm8994_regulator_init_data[0],
			.supply = "AVDD1",

		},
	.ldo[1] = {
		.enable = 0,
		.init_data = &abilene_wm8994_regulator_init_data[1],
		.supply = "DCVDD",

		},
};

static struct regulator_consumer_supply abilene_fixed_regulator_supply[] = {
	[0] = {
		.supply = "SPKVDD1",
		},
	[1] = {
		.supply = "SPKVDD2",
		},
};

static struct i2c_board_info abilene_twsi3_info[] = {
	{
	 .type = "wm8994",
	 .addr = 0x1a,
	 .platform_data = &abilene_wm8994_pdata,
	 },
};

struct regulator_init_data abilene_fixed_regulator_init_data[] = {
	[0] = {
		.constraints = {
				.name = "wm8994-SPK1",
				.always_on = 1,
				.boot_on = 1,
				},
		.num_consumer_supplies = 1,
		.consumer_supplies = &abilene_fixed_regulator_supply[0],
		},
	[1] = {
		.constraints = {
				.name = "wm8994-SPK2",
				.always_on = 1,
				.boot_on = 1,
				},
		.num_consumer_supplies = 1,
		.consumer_supplies = &abilene_fixed_regulator_supply[1],
		},
};

struct fixed_voltage_config abilene_fixed_pdata[2] = {
	[0] = {
		.supply_name = "SPKVDD1",
		.microvolts = 3700000,
		.init_data = &abilene_fixed_regulator_init_data[0],
		.gpio = -1,
		},
	[1] = {
		.supply_name = "SPKVDD2",
		.microvolts = 3700000,
		.init_data = &abilene_fixed_regulator_init_data[1],
		.gpio = -1,
		},
};

static struct platform_device fixed_device[] = {
	[0] = {
		.name = "reg-fixed-voltage",
		.id = 0,
		.dev = {
			.platform_data = &abilene_fixed_pdata[0],
			},
		.num_resources = 0,
		},
	[1] = {
		.name = "reg-fixed-voltage",
		.id = 1,
		.dev = {
			.platform_data = &abilene_fixed_pdata[1],
			},
		.num_resources = 0,
		},
};

static struct platform_device *fixed_rdev[] __initdata = {
	&fixed_device[0],
	&fixed_device[1],
};

static void abilene_fixed_regulator(void)
{
	platform_add_devices(fixed_rdev, ARRAY_SIZE(fixed_rdev));
}

#endif

#if defined(CONFIG_TC35876X)
/* force LDO3 & LDO17 always on */
int tc358765_init(void)
{
	struct regulator *vcc = NULL;

	/* enable LDO for MIPI bridge */
	vcc = regulator_get(NULL, "v_ldo17");
	if (IS_ERR(vcc))
		vcc = NULL;
	else {
		regulator_enable(vcc);
		regulator_set_voltage(vcc, 1200000, 1200000);
	}
	vcc = regulator_get(NULL, "v_ldo3");
	if (IS_ERR(vcc))
		vcc = NULL;
	else {
		regulator_enable(vcc);
		regulator_set_voltage(vcc, 1200000, 1200000);
	}

	return 0;
}

static struct tc35876x_platform_data tc358765_data = {
	.platform_init = tc358765_init,
	.id = TC358765_CHIPID,
	.id_reg = TC358765_CHIPID_REG,
};
#endif

#if defined(CONFIG_TOUCHSCREEN_TPK_R800)
static int tpk_r800_set_power(int on)
{
	struct regulator *vcc = NULL;

	vcc = regulator_get(NULL, "v_ldo8");
	if (IS_ERR(vcc)) {
		pr_err("%s can't open!\n", "v_ldo8");
		return -EIO;
	}

	if (on) {
		regulator_enable(vcc);
		regulator_set_voltage(vcc, 2800000, 2800000);
	} else
		regulator_force_disable(vcc);

	regulator_put(vcc);
	return 1;
}

static struct touchscreen_platform_data tpk_r800_data = {
	.set_power	= tpk_r800_set_power,
};
#endif

#ifdef CONFIG_FB_PXA168
static struct platform_pwm_backlight_data abilene_lcd_backlight_data = {
	/* primary backlight */
	.pwm_id = 2,
	.max_brightness = 100,
	.dft_brightness = 50,
	.pwm_period_ns = 2000000,
};

static struct platform_device abilene_lcd_backlight_devices = {
	.name = "pwm-backlight",
	.id = 2,
	.dev = {
		.platform_data = &abilene_lcd_backlight_data,
	},
};

static void __init abilene_init_lcd_pin(void)
{
	if (cpu_is_mmp3_z1() || cpu_is_mmp3_z0())
		mfp_config(ARRAY_AND_SIZE(lcd_pin_config_zx));
	else
		mfp_config(ARRAY_AND_SIZE(lcd_pin_config_a0));
}

#endif
static struct i2c_board_info abilene_twsi5_info[] = {
#if defined(CONFIG_TC35876X)
	{
		.type		= "tc35876x",
		.addr		= 0x0f,
		.platform_data	= &tc358765_data,
	},
#endif
#if defined(CONFIG_TOUCHSCREEN_TPK_R800)
	{
		.type		= "tpk_r800",
		.addr		= 0x10,
		.irq		= IRQ_GPIO(101),
		.platform_data	= &tpk_r800_data,
	},
#endif
};

#if defined(CONFIG_MMC_SDHCI_PXA)
/* MMC1 controller for SD-MMC */
static struct sdhci_pxa_platdata mmp3_sdh_platdata_mmc0 = {
	.max_speed	= 25000000,
	.quirks		= SDHCI_QUIRK_BROKEN_ADMA,
};

static struct sdhci_pxa_platdata mmp3_sdh_platdata_mmc2 = {
	.max_speed	= 50000000,
	.quirks		= SDHCI_QUIRK_BROKEN_ADMA,
	.flags		= PXA_FLAG_CARD_PERMANENT
				| PXA_FLAG_SD_8_BIT_CAPABLE_SLOT,
};

static void __init abilene_init_mmc(void)
{
	mmp3_add_sdh(0, &mmp3_sdh_platdata_mmc0); /* SD/MMC */
	if (cpu_is_mmp3_z1() || cpu_is_mmp3_z0())
		mfp_config(ARRAY_AND_SIZE(mmc1_pin_config_zx));
	else
		mfp_config(ARRAY_AND_SIZE(mmc1_pin_config_a0));

	if (emmc_boot) {
		if (cpu_is_mmp3_z1() || cpu_is_mmp3_z0())
			mfp_config(ARRAY_AND_SIZE(emmc_pin_config_zx));
		else
			mfp_config(ARRAY_AND_SIZE(emmc_pin_config_a0));
		mmp3_add_sdh(2, &mmp3_sdh_platdata_mmc2); /* eMMC */
	}
}
#endif

static struct pxa27x_keypad_platform_data mmp3_keypad_info = {
	.direct_key_map = {
		KEY_BACK,
		KEY_MENU,
		KEY_HOME,
		KEY_SEARCH,
	},
	.direct_key_num = 4,
	.debounce_interval = 30,
	.active_low = 1,
};

static void __init abilene_init(void)
{
	mfp_config(ARRAY_AND_SIZE(abilene_pin_config));

	/* on-chip devices */
	mmp3_add_uart(1);
	mmp3_add_twsi(1, NULL, ARRAY_AND_SIZE(abilene_twsi1_info));
	mmp3_add_twsi(5, NULL, ARRAY_AND_SIZE(abilene_twsi5_info));
	mmp3_add_twsi(6, NULL, ARRAY_AND_SIZE(abilene_twsi6_info));
	mmp3_add_keypad(&mmp3_keypad_info);

	mmp3_add_rtc();

#if defined(CONFIG_REGULATOR_WM8994)
	mmp3_add_twsi(3, NULL, ARRAY_AND_SIZE(abilene_twsi3_info));
	abilene_fixed_regulator();
	wm8994_ldoen();
#endif

	/* audio sspa support */
	mmp3_add_sspa(1);
	mmp3_add_sspa(2);
	mmp3_add_audiosram();

#ifdef CONFIG_FB_PXA168
	abilene_init_lcd_pin();
	abilene_add_lcd_mipi();
	/* backlight */
	mmp3_add_pwm(3);
	platform_device_register(&abilene_lcd_backlight_devices);
#endif
#ifdef CONFIG_MTD_NAND_PXA3xx
	abilene_init_flash();
#endif
#if defined(CONFIG_MMC_SDHCI_PXA)
	abilene_init_mmc();
#endif

#if defined(CONFIG_SMC91X)
	platform_device_register(&smc91x_device);
#endif

#ifdef CONFIG_ANDROID_PMEM
	pxa_add_pmem("pmem", reserving_size, 0, 1, 1);
	pxa_add_pmem("pmem_adsp", 0, 0, 0, 0);
#endif

#ifdef CONFIG_UIO_VMETA
	mmp_init_vmeta();
#endif

#ifdef CONFIG_USB_GADGET
	pxa_device_u2o.dev.platform_data = (void *)&mmp3_u2o_info;
	platform_device_register(&pxa_device_u2o);
#endif

#ifdef CONFIG_USB_OTG
	pxa_device_u2ootg.dev.platform_data = (void *)&mmp3_u2o_info;
	platform_device_register(&pxa_device_u2ootg);
	pxa_device_u2oehci.dev.platform_data = (void *)&mmp3_u2o_info;
	platform_device_register(&pxa_device_u2oehci);
#endif
#ifdef CONFIG_USB_EHCI_PXA_U2H
	mmp3_hsic1_device.dev.platform_data = (void *)&mmp3_hsic_info;
	platform_device_register(&mmp3_hsic1_device);
#endif

#ifdef CONFIG_USB_ANDROID
	mmp3_android_add_usb_devices();
#endif
}

MACHINE_START(ABILENE, "Abilene")
	.map_io		= pxa_map_io,
	.nr_irqs	= ABILENE_NR_IRQS,
	.init_irq	= mmp3_init_irq,
	.timer		= &mmp3_timer,
	.init_machine	= abilene_init,
MACHINE_END
