/*
 *  linux/arch/arm/mach-pxa/alkon.c
 *
 *  Support for the Marvell Handheld Platform (aka ALKON)
 *
 *  Copyright (C) 2007-2010 Marvell International Ltd.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  publishhed by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/mfd/88pm860x.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/usb/android_composite.h>
#include <linux/mmc/sdhci.h>
#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/machine.h>
#include <linux/sd8x_rfkill.h>
#include <linux/smp_lock.h>

#ifdef CONFIG_PROC_FS
#include <linux/proc_fs.h>
#endif

#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include <mach/hardware.h>
#include <mach/spa.h>
#ifdef CONFIG_FUELGAUGE_STC3105
#include <mach/stc3105_battery.h>
#endif
#include <mach/mfp.h>
#include <mach/gpio.h>
#include <mach/mfp-pxa9xx.h>
#include <mach/pxa95xfb.h>
#include <mach/pxa95x_dvfm.h>
#include <mach/pmu.h>
#include <mach/pmic_id.h>
#include <mach/camera.h>
#include <mach/soc_vmeta.h>

#include <plat/sdhci.h>
#include <plat/i2c.h>
#include <plat/ssp.h>
#include <plat/pxa27x_keypad.h>
#include <plat/pxa_u2o.h>
#include <plat/pxa_uart.h>
#include <plat/vbus.h>
#include <plat/pmem.h>
#include <plat/pxa9xx_sim.h>

#include "devices.h"
#include "generic.h"
#include <media/soc_camera.h>
#include <linux/wakelock.h>
#include <mach/audio.h>
#include <linux/i2c-gpio.h>

#ifdef CONFIG_PMIC_D1980
#include <linux/d1982/pmic.h>
#include <linux/d1982/core.h>
#endif

#define ALKON_NR_IRQS	(IRQ_BOARD_START + 40)
/*   SD-Card GPIO-Detection defines
* NOTE: The appropriated MFPR register should be configured in OBM
*/
#define SD_CARD_NO_DETECT
#if defined(SD_CARD_NO_DETECT)
volatile static int sd_card_present = 1; /*may be used to force Out/In*/
#else
volatile static int sd_card_present = -1; /* should be DETECTED */
#define SD_CARD_DETECT_GPIO_NUM        130
#define GPIO_CD_GPIO       MFP_CFG(GPIO130, AF0)
#define MFP_PIN_GPIO_CD    MFP_PIN_GPIO130
#endif

#define GPIO_NUM(x)     (pins_data->sdhci_gpios[x].gpio_num)
#define PIN_TYPE(x)     (pins_data->sdhci_gpios[x].pin_type)
#define GPIO_DESCR(x)   (pins_data->sdhci_gpios[x].description)
#define MFP_DRIVE_MASK		0x00001800 /* bits 12:11 */
#define MFP_DRIVE_SLOW		0x00000000
#define MFP_DRIVE_MEDIUM	0x00001000
#define MFP_DRIVE_FAST		0x00001800

static void cd_wakelock_op(unsigned int sec);
static struct wake_lock wifi_delayed_work_wake_lock;
static struct wake_lock cd_wake_lock;

static struct pm860x_touch_pdata touch = {
	.gpadc_prebias	= 1,
	.slot_cycle	= 1,
	.tsi_prebias	= 6,
	.pen_prebias	= 16,
	.pen_prechg	= 2,
	.res_x		= 300,
};

#if defined(CONFIG_TOUCHSCREEN_VNC)
static struct platform_device vnc_device = {
	.name	= "vnc-ts",
	.id	= -1,
};
#endif

static struct pm860x_backlight_pdata backlight[] = {
	{
		/*backlight data*/
		.id	= PM8606_ID_BACKLIGHT,
		.iset	= PM8606_WLED_CURRENT(0x08),
		.flags	= PM8606_BACKLIGHT1,
	},
	{
		/*keypad backlight data*/
		.id	= PM8606_ID_BACKLIGHT,
		.iset	= PM8606_WLED_CURRENT(0x08),
		.flags	= PM8606_BACKLIGHT2,
	},
};

static struct pm860x_led_pdata led[] = {
	{
		.id	= PM8606_ID_LED,
		.iset	= PM8606_LED_CURRENT(12),
		.flags	= PM8606_LED1_RED,
	}, {
		.id	= PM8606_ID_LED,
		.iset	= PM8606_LED_CURRENT(12),
		.flags	= PM8606_LED1_GREEN,
	}, {
		.id	= PM8606_ID_LED,
		.iset	= PM8606_LED_CURRENT(12),
		.flags	= PM8606_LED1_BLUE,
	}, {
		.id	= PM8606_ID_LED,
		.iset	= PM8606_LED_CURRENT(12),
		.flags	= PM8606_LED2_RED,
	}, {
		.id	= PM8606_ID_LED,
		.iset	= PM8606_LED_CURRENT(12),
		.flags	= PM8606_LED2_GREEN,
	}, {
		.id	= PM8606_ID_LED,
		.iset	= PM8606_LED_CURRENT(12),
		.flags	= PM8606_LED2_BLUE,
	},
};

static struct pm860x_vbus_pdata vbus = {
        .supply         = PM860X_GPIO1_SUPPLY_VBUS,
        .idpin          = PM860X_IDPIN_NO_USE,
        .reg_base       = PXA935_U2O_REGBASE,
        .reg_end        = PXA935_U2O_REGBASE + USB_REG_RANGE,
};


/* +  add touch device I2C bit banging*/
#if defined(CONFIG_TSP_ABOVE_ALKON_REV03)
#define CY8CTMA340_I2C_SDA_GPIO         42
#define CY8CTMA340_I2C_SCL_GPIO          41
#else
#define CY8CTMA340_I2C_SDA_GPIO          129
#define CY8CTMA340_I2C_SCL_GPIO          128
#endif

#if 0
static mfp_cfg_t qt602240_mfp_cfg[] __initdata = {
	/* i2c-gpio */
	GPIO48_GPIO,    /* TSP_RST*/
	GPIO49_GPIO,    /* TSP_INT*/
	GPIO51_GPIO,    /* TSP_SCL*/
	GPIO52_GPIO,    /* TSP_SDA*/
};
#endif

static struct i2c_gpio_platform_data i2c_bus_data = {
	.sda_pin = CY8CTMA340_I2C_SDA_GPIO,
	.scl_pin = CY8CTMA340_I2C_SCL_GPIO,
	.udelay  = 3,  /* brian :3*/
	.timeout = 100,
};

static struct platform_device i2c_bus_device = {
	.name		= "i2c-gpio",
	.id		= 4, /* pxa95x-i2c are bus 0, 1, 2, so start at 3 */
	.dev = {
		.platform_data = &i2c_bus_data,
	}
};

static struct i2c_board_info __initdata cy8ctma340_i2c_devices[] = {
	{
#if defined(CONFIG_TSP_ABOVE_ALKON_REV03)
	     I2C_BOARD_INFO("zinitix_touch", 0x20),
             .irq = IRQ_GPIO(mfp_to_gpio(MFP_PIN_GPIO23)), /* SKC on110729 TSP I2C client */
#else
		I2C_BOARD_INFO("cy8ctma340", 0x48),
             .irq = IRQ_GPIO(mfp_to_gpio(MFP_PIN_GPIO132)), /* SKC on110215 TSP I2C client */
#endif
	},
};
#if defined(CONFIG_PMIC_D1980)
static struct resource cy8ctma340_resources[] = {
	[0] = {
#if defined(CONFIG_TSP_ABOVE_ALKON_REV03)
		.start	= IRQ_GPIO(mfp_to_gpio(MFP_PIN_GPIO23)),
		.end	= IRQ_GPIO(mfp_to_gpio(MFP_PIN_GPIO23)),
#else
		.start	= IRQ_GPIO(mfp_to_gpio(MFP_PIN_GPIO132)),
		.end	= IRQ_GPIO(mfp_to_gpio(MFP_PIN_GPIO132)),
#endif
		.flags	= IORESOURCE_IRQ,
	}
};

static struct platform_device cy8ctma340_ts_device = {
	.name 		= "atmel_qt602240",
	.id 		= -1,
	.num_resources	= ARRAY_SIZE(cy8ctma340_resources),
	.resource	= cy8ctma340_resources,
};

static struct platform_device android_hsdetect_device = {
	.name               = "android-headset",
	.id                 = -1,
};

static struct platform_device android_vibrator_device = {
	.name               = "android-vibrator",
	.id                 = -1,
};
#endif
static struct platform_device backlight_device = {
       .name           = "backlight-0",
	.id 		= -1,
};

static struct platform_device kp_bl_device = {
	.name 		= "keypad-led",
	.id 		= -1,
};

static struct platform_device torch_flash_device = {
	.name 		= "torch-flash",
	.id 		= -1,
};

static void init_tsp(void)
{
	int intr_pin;

	msleep(1);
       printk("[TSP] init_tsp\n");
#if defined(CONFIG_TSP_ABOVE_ALKON_REV03)
       intr_pin = mfp_to_gpio(MFP_PIN_GPIO23);
#else
	intr_pin = mfp_to_gpio(MFP_PIN_GPIO132);
#endif
	if (gpio_request(intr_pin, "tsp interrupt")) {
		printk(KERN_ERR "QT602240 Request GPIO_%d failed!\n", intr_pin);
}
	gpio_direction_input(intr_pin);
	gpio_free(intr_pin);

}
EXPORT_SYMBOL(init_tsp);

/* - add touch device I2c bit banging*/



static void disable_rf(void)
{
}

static struct pm860x_power_pdata power = {
	.disable_rf_fn	= disable_rf,
};

static struct pxa95x_freq_mach_info freq_mach_info = {
#if defined(CONFIG_PMIC_D1980)
        .flags = 0,
#else
	.flags = PXA95x_USE_POWER_I2C,
#endif
};

#if defined(CONFIG_SENSORS_CM3601)
static int cm3601_request_resource(unsigned char gpio_num, char *name){
       int ret = 0;
       ret = gpio_request(mfp_to_gpio(gpio_num), name);
       if (ret) {
               printk(KERN_ERR "%s: can't request GPIO %d.\n", __func__,gpio_num);
               return -1;
       }
       return ret;
}

static void cm3601_release_resource(unsigned char gpio_num){
       gpio_free(gpio_num);
}

static struct pm860x_cm3601_pdata cm3601_platform_info = {
       .gpio_en = MFP_PIN_GPIO51,
       .gpio_out = MFP_PIN_GPIO50,
       .request_source = cm3601_request_resource,
       .release_source = cm3601_release_resource,
};

#endif

/* TODO: check the regulator data carefully */
static struct regulator_consumer_supply regulator_supply[PM8607_ID_RG_MAX];
static struct regulator_init_data regulator_data[PM8607_ID_RG_MAX];

#define REG_SUPPLY_INIT(_id, _name, _dev_name)		\
{							\
	int _i = _id;				\
	regulator_supply[_i].supply		= _name;		\
	regulator_supply[_i].dev_name	= _dev_name;	\
}

#define REG_INIT(_id, _name, _min, _max, _always, _boot)\
{									\
	int _i = _id;				\
	regulator_data[_i].constraints.name	=		\
		__stringify(_name);\
	regulator_data[_i].constraints.min_uV		= _min;	\
	regulator_data[_i].constraints.max_uV		= _max;	\
	regulator_data[_i].constraints.always_on	= _always;\
	regulator_data[_i].constraints.boot_on	= _boot;	\
	regulator_data[_i].constraints.valid_ops_mask	=	\
		REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS;	\
	regulator_data[_i].num_consumer_supplies	= 1;	\
	regulator_data[_i].consumer_supplies	=			\
		&regulator_supply[PM8607_ID_##_name];	\
}

static struct pm860x_rtc_pdata rtc = {
	.vrtc		= 1,
	.rtc_wakeup	= 0,
};

static struct pm860x_platform_data pm8607_info = {
/*	.touch		= &touch,*/
/*	.backlight	= &backlight[0],*/
/*	.led		= &led[0],*/
	.vbus		= &vbus,
	.power		= &power,
	.rtc		= &rtc,
	.regulator	= regulator_data,
#if defined(CONFIG_SENSORS_CM3601)
	.cm3601		= &cm3601_platform_info,
#endif
	.companion_addr	= 0x10,
	.irq_mode	= 0,
	.irq_base	= IRQ_BOARD_START,
	.headset_flag	= 0,

	.i2c_port	= PI2C_PORT,
	.batt_det	= 1,

};

/*
 * Switch from SIM1 to SIM2
 *		Set RF_IF_17..19 MFPRs to AF_0
 *		Set GSIM_XXX MFPRs to AF_3
 *
 * Switch from SIM2 to SIM1
 *		Set GSIM_XXX MFPRs to AF_0
 *		Set RF_IF_17..19 MFPRs to AF_4
 */
static struct sim_mfp_reg sim_mfprs[] = {
	{.reg_num = MFP_PIN_GSIM_UCLK,	.new_val = 0x124b,},
	{.reg_num = MFP_PIN_GSIM_UIO,	.new_val = 0x12cb },
	{.reg_num = MFP_PIN_GSIM_nURST,	.new_val = 0x134b },
	{.reg_num = MFP_PIN_RF_IF17,	.new_val = 0x124c },
	{.reg_num = MFP_PIN_RF_IF18,	.new_val = 0x12cc },
	{.reg_num = MFP_PIN_RF_IF19,	.new_val = 0x134c },
};

static struct sim_platform_data sim_plat_data = {
	.sim_mfp_cfg	= sim_mfprs,
	.sim_cards		= 1,
};

static struct platform_device sim_dev = {
	.name	= "pxa-sim",
	.id		= -1,
	.dev.platform_data = &sim_plat_data,
};


static void regulator_init(void)
{
	int i = 0;
//	if (PXA95x_USE_POWER_I2C != freq_mach_info.flags) {
	REG_SUPPLY_INIT(PM8607_ID_BUCK1, "v_buck1", NULL);
	REG_INIT(i++, BUCK1, 1200000, 1200000, 0, 0);
//	}
	REG_SUPPLY_INIT(PM8607_ID_LDO1, "v_gps", NULL);
	REG_SUPPLY_INIT(PM8607_ID_LDO2, "v_cam", NULL);
	REG_SUPPLY_INIT(PM8607_ID_LDO5, "v_ldo5", NULL);

#if defined(CONFIG_TSP_ABOVE_ALKON_REV03) //MARVELL>> LDO for HEADSET - above 0.3 = Alkon0.4 or Alkon0.5

       REG_SUPPLY_INIT(PM8607_ID_LDO6, "v_touch2", NULL);
	
	//MARVELL >>> <<<< LDO for Headset 
	/* In Alkon 0.4 LDO8 is free so we can init 
	 * it alwase (in 0.4 & 0.5)  to v_mic_bias*/
	//REG_SUPPLY_INIT(PM8607_ID_LDO8, "v_touch1", NULL); 
	REG_SUPPLY_INIT(PM8607_ID_LDO8, "v_mic_bias", NULL); 
	//MARVELL >>> >>>>LDO for Headset 

#else   /* MARVELL >>> Alkon 0.3 different build*/

	REG_SUPPLY_INIT(PM8607_ID_LDO6, "v_kpled", NULL);
	REG_SUPPLY_INIT(PM8607_ID_LDO8, "v_touch", NULL);
#endif
	REG_SUPPLY_INIT(PM8607_ID_LDO9, "v_lcd", NULL);
	REG_SUPPLY_INIT(PM8607_ID_LDO10, "v_ldo10", NULL);
	REG_SUPPLY_INIT(PM8607_ID_LDO12, "v_ldo12", NULL);
	REG_SUPPLY_INIT(PM8607_ID_LDO13, "v_sdcard", "sdhci-pxa.1");
	REG_SUPPLY_INIT(PM8607_ID_LDO14, "v_gps_txco", NULL);

	REG_INIT(i++, LDO1, 1800000, 1800000, 0, 0);
	REG_INIT(i++, LDO2, 2800000, 2800000, 0, 0);
	REG_INIT(i++, LDO5, 3300000, 3300000, 0, 0);
#if defined(CONFIG_TSP_ABOVE_ALKON_REV03)
       REG_INIT(i++, LDO6, 3300000, 3300000, 0, 0);
	REG_INIT(i++, LDO8, 1800000, 1800000, 0, 0);
	REG_INIT(i++, LDO9, 3300000, 3300000, 0, 0);
#else
	REG_INIT(i++, LDO6, 3300000, 3300000, 0, 0);
	REG_INIT(i++, LDO8, 2800000, 2800000, 0, 0);
	REG_INIT(i++, LDO9, 2850000, 2850000, 0, 0);
#endif
	
	REG_INIT(i++, LDO10, 3300000, 3300000, 0, 0);
	REG_INIT(i++, LDO12, 2900000, 2900000, 0, 0);
	REG_INIT(i++, LDO13, 2800000, 2800000, 0, 0);
	REG_INIT(i++, LDO14, 1800000, 1800000, 0, 0);

	pm8607_info.num_regulators = i;

}

static int vbus_detect(void *func, int enable)
{
	return 0;
}

#if defined(CONFIG_PMIC_D1980)
static int d1980_init_irq(void)
{
	mfp_cfg_t config[] = {
		MFP_PIN_GPIO83 | MFP_LPM_EDGE_FALL,
	}; 
	if(gpio_request(mfp_to_gpio(MFP_PIN_GPIO83), "D1980_IRQ_GPIO")) {
		return -1;
	}
	gpio_direction_input(mfp_to_gpio(MFP_PIN_GPIO83));
	mfp_config(ARRAY_AND_SIZE(config));
	gpio_free(mfp_to_gpio(MFP_PIN_GPIO83));
	return 0;
}

static int __init pmic_d1980_init(struct d1980 *d1980)
{        
    return 0;
}

static struct d1980_platform_data d1980_info = {
    .init =  pmic_d1980_init,
    .irq_init = d1980_init_irq,
};
#endif

#ifdef CONFIG_USB_GADGET_PXA_U2O
static int vbus_status(unsigned base)
{
	int status = VBUS_LOW;

	if (spa_is_usb_connect() == 1) /* +SAMSUNG_YOUNGJUN : for ACAT connection*/
	{
		status = VBUS_HIGH;
	}
	return status;
}

static struct pxa_usb_plat_info u2o_info = {
       .phy_init       = pxa9xx_usb_phy_init,
       .vbus_status    = NULL,
       .vbus_detect    = vbus_detect,
       .usbid_detect   = NULL,
       .is_otg         = 1,
       .in_single	   = 1,
};
#endif

#ifdef CONFIG_USB_ANDROID

#if defined(CONFIG_USB_ANDROID_MASS_STORAGE) || defined(CONFIG_USB_FILE_STORAGE)
static struct usb_mass_storage_platform_data mass_storage_pdata = {
       .nluns          = 1,
       .vendor         = "SAMSUNG ",
       .product        = "GT-S5690 Card ",
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
       .vendorID       = 0x04E8,
/*dh0318.lee 110310 FOR_DEVGURU change_string*/
       .vendorDescr    = "Samsung Rndis function"
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
static char *usb_functions0[] = {
#if defined(CONFIG_USB_ANDROID_MASS_STORAGE) || defined(CONFIG_USB_FILE_STORAGE)
       "usb_mass_storage",
#endif
#ifdef CONFIG_USB_ANDROID_PXA955_ACM
       "acm",
#endif
#ifdef CONFIG_USB_ANDROID_ADB
       "adb",
#endif
#ifdef CONFIG_USB_ANDROID_PXA955_DIAG
	"diag",
#endif
#ifdef CONFIG_USB_ANDROID_RNDIS
       "rndis",
#endif
#ifdef CONFIG_USB_ANDROID_MTP
		"mtp",
#endif
};

/* following usb_functionsX include functions for
 specific usb composite configurations */
static char *usb_functions1[] = {
#if defined(CONFIG_USB_ANDROID_MASS_STORAGE) || defined(CONFIG_USB_FILE_STORAGE)
	"usb_mass_storage",
#endif
#ifdef CONFIG_USB_ANDROID_PXA955_ACM
	"acm",
#endif
#ifdef CONFIG_USB_ANDROID_ADB
	"adb",
#endif
};

static char *usb_functions2[] = {
#ifdef CONFIG_USB_ANDROID_PXA955_ACM
	"acm",
#endif
#if defined(CONFIG_USB_ANDROID_MASS_STORAGE) || defined(CONFIG_USB_FILE_STORAGE)
	"usb_mass_storage",
#endif
};

static char *usb_functions3[] = {
#if defined(CONFIG_USB_ANDROID_MASS_STORAGE) || defined(CONFIG_USB_FILE_STORAGE)
		"usb_mass_storage",
#endif
};

static char *usb_functions4[] = {
#ifdef CONFIG_USB_ANDROID_RNDIS
		"rndis",
#endif
};

static char *usb_functions5[] = {
#ifdef CONFIG_USB_ANDROID_PXA955_DIAG
	"diag",
#endif
};

static char *usb_functions6[] = {
#ifdef CONFIG_USB_ANDROID_PXA955_ACM
	"acm",
#endif
#ifdef CONFIG_USB_ANDROID_PXA955_DIAG
	"diag",
#endif
};

static char *usb_functions7[] = {
#if defined(CONFIG_USB_ANDROID_MASS_STORAGE) || defined(CONFIG_USB_FILE_STORAGE)
		"usb_mass_storage",
#endif
#ifdef CONFIG_USB_ANDROID_PXA955_ACM
		"acm",
#endif
#ifdef CONFIG_USB_ANDROID_ADB
		"adb",
#endif

#ifdef CONFIG_USB_ANDROID_PXA955_DIAG
		"diag",
#endif
};
static struct android_usb_product usb_products[] = {
	{
		.product_id = 0x685E,
		.num_functions = ARRAY_SIZE(usb_functions0),
		.functions = usb_functions0,
	},
	{
		.product_id = 0x685E,
		.num_functions = ARRAY_SIZE(usb_functions1),
		.functions = usb_functions1,
	},
	{
		.product_id = 0x685E,
		.num_functions = ARRAY_SIZE(usb_functions2),
		.functions = usb_functions2,
	},
	{
		.product_id = 0x685B,
		.num_functions = ARRAY_SIZE(usb_functions3),
		.functions = usb_functions3,
	},
	{
		.product_id = 0x6863,
		.num_functions = ARRAY_SIZE(usb_functions4),
		.functions = usb_functions4,
	},
	{
		.product_id = 0x8108,
		.num_functions = ARRAY_SIZE(usb_functions5),
		.functions = usb_functions5,
	},
	{
		.product_id = 0x685D,
		.num_functions = ARRAY_SIZE(usb_functions6),
		.functions = usb_functions6,
	},
	{
		.product_id = 0x685E,
		.num_functions = ARRAY_SIZE(usb_functions7),
		.functions = usb_functions7,
	}
};

static struct android_usb_platform_data android_usb_pdata = {
       .vendor_id              = 0x04E8,
       .product_id             = 0x685E,
       .version                = 0x0100,
       .product_name           = "Android",
/*dh0318.lee 110310 FOR_DEVGURU change_string*/
       .manufacturer_name      = "Samsung",
		/* serial_number is required to assure that in case of ACM
		the same COM port number will be taken at each SW reboot*/
	.serial_number		= "0x0100",
	.num_products		= ARRAY_SIZE(usb_products),
	.products		= usb_products,
	.num_functions		= ARRAY_SIZE(usb_functions0),
	.functions		= usb_functions0,
};

static struct platform_device android_device_usb = {
       .name   = "android_usb",
       .id     = -1,
       .dev    = {
               .platform_data  = &android_usb_pdata,
       },
};

static struct platform_device android_device_yaffs = {
	.name   = "yaffs",
	.id     = -1,
};

static void __init android_add_usb_devices(void)
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

/*static struct pxa95x_freq_mach_info req_mach_info = {
	.flags = PXA95x_USE_POWER_I2C,
};*/

static struct i2c_board_info i2c1_info[] = {
#if defined(CONFIG_PMIC_D1980)
	{
        I2C_BOARD_INFO(D1980_I2C, D1980_I2C_ADDR),
		.platform_data	= &d1980_info,
		.irq		= IRQ_PMIC_INT,
	},
#if defined(CONFIG_SND_SOC_D1981)
 	{
		.type	    = "d1981",
		.addr		= 0x1a,
	},
#endif /* CONFIG_SND_SOC_D1981 */
#else
	{
		.type		= "88PM860x",
		.addr		= 0x34,
		.platform_data	= &pm8607_info,
		.irq		= IRQ_PMIC_INT,
	},
#endif /* CONFIG_PMIC_d1980 */
};
#ifndef CONFIG_I2C_PXA95x
static struct i2c_pxa_platform_data i2c1_pdata = {
	.mode = I2C_PXA_MODE_FIFO_INT,
	.freq = I2C_PXA_FREQ_HS_FAST,
	.master_code = 0x0e,
	.ilcr = 0x082CA356,/* ilcr: hs mode: 0x082CA356 normal: 0x082C36A3 */
	.iwcr = 0x0000143A,/* ilcr: hs mode: 0x0000143A normal: 0x00001431*/
};

static struct i2c_pxa_platform_data i2c2_pdata = {
	.mode = I2C_PXA_MODE_FIFO_INT,
	.freq = I2C_PXA_FREQ_FAST,
};

static struct i2c_pxa_platform_data i2c3_pdata = {
	.mode = I2C_PXA_MODE_FIFO_INT,
	.freq = I2C_PXA_FREQ_FAST,
};

#else
static struct i2c_pxa_platform_data i2c1_pdata = {
#if defined(CONFIG_PMIC_D1980)
	.use_pio     = 0,
	.flags		 = PXA_I2C_FAST_MODE, /* | PXA_I2C_USING_FIFO_PIO_MODE, */
	.master_code = (0x08 | 0x06), /*8 -highest, 0xF -lowest arbitration*/
#else
	.use_pio        = 0,
	.flags		= PXA_I2C_HIGH_MODE | PXA_I2C_FAST_MODE | PXA_I2C_USING_FIFO_PIO_MODE,
	.master_code	= (0x08 | 0x06), /*8 -highest, 0xF -lowest arbitration*/
#endif
};

static struct i2c_pxa_platform_data i2c2_pdata = {
	.use_pio	= 0,
	.flags		= PXA_I2C_FAST_MODE | PXA_I2C_USING_FIFO_PIO_MODE,
};

static struct i2c_pxa_platform_data i2c3_pdata = {
	.use_pio        = 0,
	.flags          = PXA_I2C_FAST_MODE | PXA_I2C_USING_FIFO_PIO_MODE,
};

#endif

#if defined(CONFIG_FB_PXA95x)
struct ssp_device * ssp_lcd_init(void)
{
	uint32_t sscr0;
	struct ssp_device * ssp;

	ssp = pxa_ssp_request(2, "SSP");
	if (ssp == NULL){
		printk(KERN_ERR "SSP2 for lcd init failed\n");
		return NULL;
	}

	clk_enable(ssp->clk);

	/*disable SSP*/
	sscr0 = pxa_ssp_read_reg(ssp, SSCR0);
	sscr0 &= ~SSCR0_SSE;
	pxa_ssp_write_reg(ssp, SSCR0, sscr0);

	/* set up port type, speed, port settings */
	pxa_ssp_write_reg(ssp, SSCR1, 0x18);
	pxa_ssp_write_reg(ssp, SSPSP, 0);
	pxa_ssp_write_reg(ssp, SSCR0, 0x00000588);

	return ssp;
}

void ssp_lcd_deinit(struct ssp_device *ssp)
{
	uint32_t sscr0;

	/*disable SSP*/
	sscr0 = pxa_ssp_read_reg(ssp, SSCR0);
	sscr0 &= ~SSCR0_SSE;
	pxa_ssp_write_reg(ssp, SSCR0, sscr0);

	clk_disable(ssp->clk);
	pxa_ssp_free(ssp);
}

int ssp_lcd_send_cmd_para(struct ssp_device *ssp, u16 *cmd, int num)
{
	int i;
	for (i = 0; i < num; i++, cmd++) {
		int timeout = 100000;
		while (!(pxa_ssp_read_reg(ssp, SSSR) & SSSR_TNF)) {
				if (!--timeout)
					return -ETIMEDOUT;
			cpu_relax();
		}
		pxa_ssp_write_reg(ssp, SSDR, *cmd & 0x1ff);

		/* ensure TX FIFO is empty instead of not full */
		timeout = 100000*2;

		do {
			while (pxa_ssp_read_reg(ssp, SSSR) & SSSR_RNE) {
					if (!--timeout)
						return -ETIMEDOUT;
				(void)pxa_ssp_read_reg(ssp, SSDR);
			}
			if (!--timeout)
				return -ETIMEDOUT;
		} while (pxa_ssp_read_reg(ssp, SSSR) & SSSR_BSY);
	}
	return 0;
}
extern  void  alkon_lcd_init(void);
static void __init init_lcd(void)
{
	alkon_lcd_init();
}

#endif

#ifdef CONFIG_PM
static int init_wakeup(pm_wakeup_src_t *src)
{
	memset(src, 0, sizeof(pm_wakeup_src_t));
	src->bits.rtc = 1;
	src->bits.ost = 1;
#ifdef CONFIG_PXA9XX_ACIPC
	src->bits.msl = 1;
#endif
	src->bits.ext0 = 1;
	src->bits.uart1 = 1;
	src->bits.mkey = 1;
	src->bits.eth = 1;
	src->bits.tsi = 1;
	src->bits.cmwdt = 1;
	src->bits.mmc1_cd = 1;
	src->bits.mmc3_dat1 = 1;
	src->bits.usbdetect = 1;
	src->bits.headset = 1;
	src->bits.hookswitch = 1;
	src->bits.proximate_sensor = 1;
	return 0;
}

static int query_wakeup(unsigned int reg, pm_wakeup_src_t *src)
{
	memset(src, 0, sizeof(pm_wakeup_src_t));
	if (reg & PXA95x_PM_WE_RTC)
		src->bits.rtc = 1;
	if (reg & PXA95x_PM_WE_OST)
		src->bits.ost = 1;
	if (reg & PXA95x_PM_WE_MSL0)
		src->bits.msl = 1;
	if (reg & PXA95x_PM_WE_EXTERNAL0)
		src->bits.ext0 = 1;
	if (reg & PXA95x_PM_WE_KP)
		src->bits.mkey = 1;
	if (reg & PXA95x_PM_WE_GENERIC(3))
		src->bits.tsi = 1;
	if (reg & PXA95x_PM_WE_GENERIC(9)) {
		if (!is_uart_gpio())
			src->bits.uart1 = 1;
	}
	if (reg & PXA95x_PM_WE_GENERIC(2)) {
		if (!is_uart_gpio())
			src->bits.uart2 = 1;
	}
	if (reg & PXA95x_PM_WE_GENERIC(12))
		src->bits.cmwdt = 1;
	if (reg & PXA95x_PM_WE_GENERIC(13)) {
		if (pxa95x_query_gwsr(97))
			src->bits.eth = 1;
		if (pxa95x_query_gwsr(53))
			src->bits.uart1 = 1;
#if !defined(SD_CARD_NO_DETECT)
		if (pxa95x_query_gwsr(SD_CARD_DETECT_GPIO_NUM))
			src->bits.mmc1_cd = 1;
#endif
		if (pxa95x_query_gwsr(79))
			src->bits.headset = 1;
		if (pxa95x_query_gwsr(80))
			src->bits.hookswitch = 1;
		if (pxa95x_query_gwsr(85))
			src->bits.proximate_sensor = 1;
	}

	return 0;
}

static int ext_wakeup(pm_wakeup_src_t src, int enable)
{
	unsigned int ret = 0;
	if (enable) {
		if (src.bits.ext0)
			ret |= PXA95x_PM_WE_EXTERNAL0;
		if (src.bits.ext1)
			ret |= PXA95x_PM_WE_EXTERNAL1;
	}
	return ret;
}
static int headset_wakeup(pm_wakeup_src_t src, int enable)
{
	unsigned int ret = 0;
	mfp_cfg_t mfp_c;
	if (enable) {
		mfp_c = GPIO79_GPIO | MFP_LPM_EDGE_BOTH;
		pxa3xx_mfp_config(&mfp_c, 1);
		ret |= PXA95x_PM_WE_GENERIC(13);
	} else {
		mfp_c = GPIO79_GPIO | MFP_LPM_EDGE_NONE;
		pxa3xx_mfp_config(&mfp_c, 1);
	}
	return ret;
}
static int hookswitch_wakeup(pm_wakeup_src_t src, int enable)
{
	unsigned int ret = 0;
	mfp_cfg_t mfp_c;
	if (enable) {
		mfp_c = GPIO80_GPIO| MFP_LPM_EDGE_BOTH;
		pxa3xx_mfp_config(&mfp_c, 1);
		ret |= PXA95x_PM_WE_GENERIC(13);
	} else {
		mfp_c = GPIO80_GPIO | MFP_LPM_EDGE_NONE;
		pxa3xx_mfp_config(&mfp_c, 1);
	}
	return ret;
}
static int proximate_sensor_wakeup(pm_wakeup_src_t src, int enable)
{
	unsigned int ret = 0;
	mfp_cfg_t mfp_c;
	if (enable) {
		mfp_c = GPIO85_GPIO| MFP_LPM_EDGE_BOTH;
		pxa3xx_mfp_config(&mfp_c, 1);
		ret |= PXA95x_PM_WE_GENERIC(13);
	} else {
		mfp_c = GPIO85_GPIO | MFP_LPM_EDGE_NONE;
		pxa3xx_mfp_config(&mfp_c, 1);
	}
	return ret;
}

static int key_wakeup(pm_wakeup_src_t src, int enable)
{
	unsigned int ret = 0;
	if (enable) {
		if (src.bits.mkey) {
			static mfp_cfg_t key_edgeboth_cfg[] = {
				GPIO4_KP_MKIN_2 | MFP_LPM_EDGE_BOTH,
				GPIO6_KP_MKIN_3 | MFP_LPM_EDGE_BOTH,
			};
			pxa3xx_mfp_config(ARRAY_AND_SIZE(key_edgeboth_cfg));
			ret |= PXA95x_PM_WE_KP;
		}
	} else {
		if (src.bits.mkey) {
			static mfp_cfg_t key_edgenone_cfg[] = {
				GPIO4_KP_MKIN_2 | MFP_LPM_EDGE_NONE,
				GPIO6_KP_MKIN_3 | MFP_LPM_EDGE_NONE,
			};
			pxa3xx_mfp_config(ARRAY_AND_SIZE(key_edgenone_cfg));
		}
	}
	return ret;
}

static int mmc1_wakeup(pm_wakeup_src_t src, int enable)
{
	unsigned int ret = 0;
#if !defined(SD_CARD_NO_DETECT)
	mfp_cfg_t mfp_c;
	if (enable) {
		if (src.bits.mmc1_cd) {
			mfp_c = GPIO_CD_GPIO | MFP_LPM_EDGE_BOTH;
			pxa3xx_mfp_config(&mfp_c, 1);
			ret |= PXA95x_PM_WE_GENERIC(13);
		}
	} else {
		mfp_c = GPIO_CD_GPIO | MFP_LPM_EDGE_NONE;
		pxa3xx_mfp_config(&mfp_c, 1);
	}
#endif
	return ret;
}

static int mmc3_wakeup(pm_wakeup_src_t src, int enable)
{
	unsigned int ret = 0;
	mfp_cfg_t mfp_c;
	if (enable) {
		mfp_c = GPIO88_MMC3_DAT1 | MFP_LPM_EDGE_BOTH;
		pxa3xx_mfp_config(&mfp_c, 1);
		ret |= PXA95x_PM_WE_MMC3;
	} else {
		mfp_c = GPIO88_MMC3_DAT1 | MFP_LPM_EDGE_NONE;
		pxa3xx_mfp_config(&mfp_c, 1);
	}

	return ret;
}

static int uart_wakeup(pm_wakeup_src_t src, int enable)
{
	unsigned int ret = 0;
	mfp_cfg_t m;

	if (enable) {
		if (src.bits.uart1) {
			if (is_uart_gpio()) {
				m = MFP_CFG(GPIO53, AF0) | MFP_LPM_EDGE_FALL;
				pxa3xx_mfp_config(&m, 1);
				ret |= PXA95x_PM_WE_GENERIC(13);
			} else {
				m = GPIO53_UART1_RXD | MFP_LPM_EDGE_FALL;
				pxa3xx_mfp_config(&m, 1);
				ret |= PXA95x_PM_WE_GENERIC(9);
			}
		}
		if (src.bits.uart2) {
			if (is_uart_gpio()) {
				m = MFP_CFG(GPIO45, AF0) | MFP_LPM_EDGE_FALL;
				pxa3xx_mfp_config(&m, 1);
				ret |= PXA95x_PM_WE_GENERIC(13);
			} else {
				m = GPIO45_UART3_RXD | MFP_LPM_EDGE_FALL;
				pxa3xx_mfp_config(&m, 1);
				/* note: on pxa930, uart2 use this bit */
				ret |= PXA95x_PM_WE_GENERIC(2);
			}
		}
	} else {
		if (src.bits.uart1) {
			m = GPIO53_UART1_RXD | MFP_LPM_EDGE_NONE;
			pxa3xx_mfp_config(&m, 1);
		}
		if (src.bits.uart2) {
			m = GPIO45_UART3_RXD | MFP_LPM_EDGE_NONE;
			pxa3xx_mfp_config(&m, 1);
		}
	}
	return ret;
}

static int tsi_wakeup(pm_wakeup_src_t src, int enable)
{
	unsigned int ret = 0;
	unsigned long m;
	if (enable) {
		if (src.bits.tsi) {
			m = mfp_read(MFP_PIN_PMIC_INT);
			m &= ~(1 << 6);
			m |= 1 << 5;
			mfp_write(MFP_PIN_PMIC_INT, m);
			ret |= PXA95x_PM_WE_GENERIC(3);
		}
	} else {
		if (src.bits.tsi) {
			m = mfp_read(MFP_PIN_PMIC_INT);
			m &= ~(1 << 5);
			m |= 1 << 6;
			mfp_write(MFP_PIN_PMIC_INT, m);
		}
	}
	return ret;
}

static int comm_wdt_wakeup(pm_wakeup_src_t src, int enable)
{
	unsigned int ret = 0;
	if (enable) {
		if (src.bits.cmwdt)
			ret |= PXA95x_PM_WE_GENERIC(12);
	}
	return ret;
}

static int usb_wakeup(pm_wakeup_src_t src, int enable)
{
	unsigned int ret = 0;
	mfp_cfg_t mfp_c;
	if (enable) {
		if (src.bits.usbdetect) {
			mfp_c = GPIO92_GPIO/* | MFP_LPM_FLOAT */| MFP_LPM_EDGE_FALL; 
			pxa3xx_mfp_config(&mfp_c,1);
			ret |= PXA95x_PM_WE_GENERIC(13);
		}
	} else {
		mfp_c = GPIO92_GPIO/* | MFP_LPM_FLOAT */| MFP_LPM_EDGE_NONE;
		pxa3xx_mfp_config(&mfp_c,1);
	}

	return ret;
}

static struct pxa95x_peripheral_wakeup_ops wakeup_ops = {
	.init   = init_wakeup,
	.query  = query_wakeup,
	.ext    = ext_wakeup,
	.key    = key_wakeup,
	.mmc1   = mmc1_wakeup,
	.mmc3   = mmc3_wakeup,
	.uart   = uart_wakeup,
	.tsi    = tsi_wakeup,
	.cmwdt  = comm_wdt_wakeup,
	.usbdetect = usb_wakeup,
	.headset = headset_wakeup,
	.hookswitch = hookswitch_wakeup,
	.proximate_sensor = proximate_sensor_wakeup,
};
#endif

static struct pxa_uart_mach_info ffuart_info = {
	.stay_awake_in_suspend = 1,
};

static struct pxa_uart_mach_info stuart_info = {
	.stay_awake_in_suspend = 0,
};

#if defined(CONFIG_SND_PXA95X_SOC)
static mfp_cfg_t pxa95x_abu_mfp_cfg[] = {
	/* ABU of MG1 */
	GPIO68_ABU_RXD,
	GPIO69_ABU_TXD,
	GPIO70_GPIO,	/* no use for ABU/SSI, and configure GPIO70 to AF0 to save power when using ABU (~0.5mA) */
	GPIO71_ABU_FRM,
	GPIO72_ABU_CLK,
};

static mfp_cfg_t pxa95x_bssp2_mfp_cfg[] = {
	/* BSSP2 of MG1 */
	GPIO68_SSP2_RXD,
	GPIO69_SSP2_TXD,
	GPIO70_SSP2_SYSCLK,
	GPIO71_SSP2_FRM,
	GPIO72_SSP2_CLK,
};

static mfp_cfg_t bssp3_mfp_cfg[] = {
	/* BSSP3 of MG1*/
	GPIO63_BSSP3_CLK,
	GPIO64_BSSP3_FRM,
	GPIO65_BSSP3_TXD,
	GPIO66_BSSP3_RXD,
};

static mfp_cfg_t gssp1_mfp_cfg[] = {
	/* BSSP3 of MG1*/
	GPIO63_GSSP1_CLK,
	GPIO64_GSSP1_FRM,
	GPIO65_GSSP1_TXD,
	GPIO66_GSSP1_RXD,
};

void alkon_pxa95x_abu_mfp_init(bool abu)
{
	if (abu)
		pxa3xx_mfp_config(ARRAY_AND_SIZE(pxa95x_abu_mfp_cfg));
	else
		pxa3xx_mfp_config(ARRAY_AND_SIZE(pxa95x_bssp2_mfp_cfg));
}

static void ssp3_mfp_init(bool bssp)
{
	if (bssp)
		pxa3xx_mfp_config(ARRAY_AND_SIZE(bssp3_mfp_cfg));
	else
		pxa3xx_mfp_config(ARRAY_AND_SIZE(gssp1_mfp_cfg));
}
#endif

#if defined(CONFIG_MMC_SDHCI_PXA)

extern void sdhci_pxa_notify_change(struct platform_device *pdev, int state);
extern void sdhci_pxa_set_con_clock(struct sdhci_host *host, int clock);

static inline int ext_cd_val(int gpio,int invert)
{
	int status;

	//    refer SD_CARD_NO_DETECT
	// sd_card_present<0 : detect-GPIO used, return its status
	// sd_card_present>=0: no GPIO, return the sd_card_present 
	if(sd_card_present >= 0) {
		status = (sd_card_present != 0);
	} else {
		status = gpio_get_value(gpio);
		status &= (1 << (gpio%32));
		status = !!status;

		if (invert)
			status = !status;
	}
	return status;
}

static int ext_cd_status(struct sdhci_host *host)
{
	struct sdhci_pxa *pxa = sdhci_priv(host);
	volatile int status;

	if (pxa->pdata->cd_force_status != 0) {
		status = (pxa->pdata->cd_force_status > 0);
		if ((pxa->pdata->cd_force_status == -1) ||
			(pxa->pdata->cd_force_status == 1))
			pxa->pdata->cd_force_status = 0; /*return to default*/
		/* Force requested: -2/-1/+1 = Removed/Inserted */
	} else {
		status = ext_cd_val(pxa->pdata->ext_cd_gpio,pxa->pdata->ext_cd_gpio_invert);
	}
	return status;
}

static irqreturn_t sdhci_pxa_cd_irq_thread(int irq, void *dev_id)
{
	struct platform_device *pdev = dev_id;
	struct sdhci_pxa_platdata *pdata = pdev->dev.platform_data;
	int status;

	msleep(600);
	status = ext_cd_val(pdata->ext_cd_gpio,pdata->ext_cd_gpio_invert);
	cd_wakelock_op(0);
	sdhci_pxa_notify_change(pdev, status);

	return IRQ_HANDLED;
}

int sdhci_pxa_check_short_circuit(struct platform_device *pdev)
{
	struct sdhci_pxa_platdata *pdata = pdev->dev.platform_data;
	struct sdhci_pins_data *pins_data;
	unsigned int status = 0;
	unsigned int *mfpr;
	int i = 0;

	if (!pdata)
		return 0;

	pins_data = pdata->pins_data;
	if (!pins_data)
		return 0;

	mfpr = kzalloc(sizeof(unsigned int) +
				pins_data->number_of_pins, GFP_KERNEL);
	if (!mfpr)
		return -ENOMEM;

	/* Switch MMC pins to GPIO */
	for (i = 0; i < pins_data->number_of_pins; i++) {
		if (PIN_TYPE(i) == SDHCI_PIN_CLK)
			continue;
		mfpr[i] = pxa3xx_mfp_read(GPIO_NUM(i));
		pxa3xx_mfp_write(GPIO_NUM(i), \
				(mfpr[i] & 0xFFFFFFF8));
	}
	udelay(200);
	/* if MMC_DATAx and MMC_CMD gpios are high then
	 * there's no short circuit */
	for (i = 0; i < pins_data->number_of_pins; i++)	{
		if (PIN_TYPE(i) == SDHCI_PIN_CLK)
			continue;
		if (gpio_is_valid(GPIO_NUM(i))) {
			gpio_request(GPIO_NUM(i), GPIO_DESCR(i));
			gpio_direction_input(GPIO_NUM(i));
			if (!gpio_get_value(GPIO_NUM(i))) {
				pr_info("%s is short circuited\n",
						GPIO_DESCR(i));
				status = -1;
			}
		} else {
			pr_info("mmc gpio[%d] is not valid\n", i);
			status = -1;
		}
		gpio_free(GPIO_NUM(i));
	}

	for (i = 0; i < pins_data->number_of_pins; i++)
		if (PIN_TYPE(i) != SDHCI_PIN_CLK)
			pxa3xx_mfp_write(GPIO_NUM(i), mfpr[i]);

	kfree(mfpr);

	return status;
}

/*
* Switch MMC/SD pins drive
*  Pins may have DIFFERENT drives but we set all of them upon "master"
*  The master is SDHCI_PIN_DAT0
*  NOTE: the procedure change all MMC/SD pins
*   but doesn't make any mutex or irq protection
*/
u32 sdhci_pxa_mfpr_drive(struct sdhci_pxa_platdata *pdata, u32 fast0_medium1_slow2)
{
	struct sdhci_pins_data *pins_data;
	u32 mfpr, mfpr_drive;
	int i;
	u32 drive[3] = {MFP_DRIVE_FAST, MFP_DRIVE_MEDIUM, MFP_DRIVE_SLOW};

	if (!pdata)
		return 0;

	pins_data = pdata->pins_data;
	if (!pins_data)
		return 0;

	mfpr = pxa3xx_mfp_read(GPIO_NUM(SDHCI_PIN_DAT0));

	if (fast0_medium1_slow2 == 0xFFFFFFF0)
		return mfpr; /* Just read the mfpr */

	if (!pdata->ext_cd_init)
		return 0; /* limit for SD only */

	/* Go over all pins and modify according to
	* master and  "fast0_medium1_slow2" */
	mfpr_drive = mfpr & MFP_DRIVE_MASK;

	if (fast0_medium1_slow2 == 0xFFFFFFFF) {
		/* toggle fast/medium */
		mfpr_drive = ((mfpr_drive == MFP_DRIVE_FAST) ?
			MFP_DRIVE_MEDIUM : MFP_DRIVE_FAST);
	} else if (fast0_medium1_slow2 <= 2) {
		/* set by index */
		mfpr_drive = drive[fast0_medium1_slow2];
	} else {
		/* Row data provided */
		mfpr_drive = fast0_medium1_slow2 & MFP_DRIVE_MASK;
	}
	mfpr &= ~MFP_DRIVE_MASK;
	mfpr |= mfpr_drive;

	for (i = 0; i < pins_data->number_of_pins; i++)
		pxa3xx_mfp_write(GPIO_NUM(i), mfpr);

	udelay(50);
	return mfpr;
}
EXPORT_SYMBOL(sdhci_pxa_mfpr_drive);

static void mci_setpower(struct device *dev, unsigned int vdd)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct sdhci_host *host = platform_get_drvdata(pdev);
	static unsigned long bitmap = 0;
	unsigned long oldbitmap = 0;
	int devbit = 1;

	oldbitmap = bitmap;

	if (pdev->id >= 0)
		devbit = (1 << pdev->id);

	if (vdd)
		bitmap |= devbit;
	else
		bitmap &= ~devbit;

	if (bitmap && !oldbitmap) {
		if (IS_ERR(host->vmmc)) {
			host->vmmc = NULL;
			printk(KERN_ERR "LDO is not allocated for SD\n");
		} else {
			pr_info("mmc power on vmmc=0x%x\n", (int)host->vmmc);
			regulator_enable(host->vmmc);

			if (sdhci_pxa_check_short_circuit(pdev)) {
				pr_info("mmc short cirquit occured, power off\n");
				sdhci_pxa_notify_change(pdev, 0);
			}
		}
	}
	else if (!bitmap && oldbitmap) {
		if (IS_ERR(host->vmmc)) {
			host->vmmc = NULL;
			printk(KERN_ERR "LDO is not allocated for SD\n");
		} else {
			pr_info("mmc power off vmmc=0x%x\n", (int)host->vmmc);
			regulator_disable(host->vmmc);
		}
	}
}

static int ext_cd_init(void (*notify_func)(struct platform_device *, int state), void *data)
{
	struct platform_device *pdev = data;
	struct sdhci_pxa_platdata *pdata = pdev->dev.platform_data;
	struct sdhci_host *host = platform_get_drvdata(pdev);
	int err, cd_irq, ext_cd_gpio;
	int status;

	/* Catch wake lock when card is inserted or removed */
	wake_lock_init(&cd_wake_lock, WAKE_LOCK_SUSPEND, "sd_card_detect");

#ifdef CONFIG_REGULATOR
#ifdef CONFIG_PMIC_D1980
	host->vmmc = regulator_get(&pdev->dev, REGULATOR_TFLASH);
#else
	host->vmmc = regulator_get(&pdev->dev, "v_sdcard");
#endif

	if (IS_ERR(host->vmmc)) {
		host->vmmc = NULL;
		printk(KERN_ERR "Cannot allocate LDO for SD\n");
	} else {
		mmc_regulator_get_ocrmask(host->vmmc);
		pr_info("success allocate LDO for SD/MMC vmmc=0x%x\n",
			(int)host->vmmc);
	}
#else
	printk(KERN_ERR "Cannot find the power supply for SD\n");
#endif

	if(sd_card_present >= 0) {
		notify_func(pdev,(sd_card_present != 0));
		return 0;
	}

	cd_irq = gpio_to_irq(pdata->ext_cd_gpio);
	ext_cd_gpio = pdata->ext_cd_gpio;

	/*
	 * setup GPIO for MMC controller
	 */
	err = gpio_request(ext_cd_gpio, "mmc card detect");
	if (err) {
		printk(KERN_ERR "gpio_request err =%d\n", err);
		goto err_request_cd;
	}
	gpio_direction_input(ext_cd_gpio);

	err = request_threaded_irq(cd_irq, NULL, sdhci_pxa_cd_irq_thread,
			  IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			  "MMC card detect", pdev);
	if (err) {
		printk(KERN_ERR "%s: MMC/SD/SDIO: "
				"can't request card detect IRQ\n", __func__);
		goto err_request_irq;
	}

	status = ext_cd_val(pdata->ext_cd_gpio,pdata->ext_cd_gpio_invert);
	notify_func(pdev,status);

	return 0;

err_request_irq:
	gpio_free(ext_cd_gpio);
err_request_cd:
	return -1;
}

#if !defined(SD_CARD_NO_DETECT)
static int ext_cd_cleanup(void (*notify_func)(struct platform_device *, int state), void *data)
{
	struct platform_device *pdev = data;
	struct sdhci_pxa_platdata *pdata = pdev->dev.platform_data;
	struct sdhci_host *host = platform_get_drvdata(pdev);
	int cd_irq;

	cd_irq = gpio_to_irq(pdata->ext_cd_gpio);
	if(cd_irq)
		free_irq(cd_irq,pdev);

	if(gpio_is_valid(pdata->ext_cd_gpio))
		gpio_free(pdata->ext_cd_gpio);

#ifdef CONFIG_REGULATOR
	if(host->vmmc)
		regulator_put(host->vmmc);
#endif

	mci_setpower(&pdev->dev,0);

	return 0;
}
#endif

void enable_oscc_tout_s0(void);
void disable_oscc_tout_s0(void);

/* specific 8787 power on/off setting for GFORCE */
void wifi_set_power(unsigned int on)
{
	unsigned long wlan_pd_mfp = 0;
	int gpio_power_down = mfp_to_gpio(MFP_PIN_GPIO77);
	static unsigned int dvfm_ind, first_time;
	unsigned int ret;

	if (is_wkr_mg2_462()){
		if (first_time == 0) {
			first_time = 1;
			ret = dvfm_register("8787 Power", &dvfm_ind);
			if (ret) {
				pr_err("Error %d: Fails to register 8787 into dvfm.\n",
					ret);
				}
		}
	}

	/* Set correct pin status when soc enters low power mode*/
	wlan_pd_mfp = pxa3xx_mfp_read(gpio_power_down);

	if (on) {
		/*do not allow to go to D2 from PP1 as
		 * it will cause WiFi issues with MMC
		 * controller. This should be fixed by MG2-C1
		 * or Alkon rev 0.2*/
		if (is_wkr_mg2_462()){
			set_d2_from_high_pp_8787 = 1;
			if (dvfm_ind) {
				dvfm_disable_op_name("156M", dvfm_ind);
				dvfm_disable_op_name("156M_HF", dvfm_ind);
				dvfm_disable_op_name("208M", dvfm_ind);
				dvfm_disable_op_name("208M_HF", dvfm_ind);
				dvfm_disable_op_name("416M_VGA", dvfm_ind);
				dvfm_disable_op_name("416M", dvfm_ind);
			}
		}
		/* set wlan_pd pin to output high in low power
			mode to ensure 8787 is not power off in low power mode*/
		//wlan_pd_mfp |= 0x100;
		/* when setting output to 1 set also PU: pulling takes effect */
		wlan_pd_mfp = (wlan_pd_mfp | 0x4100) & ~0x2000;
		pxa3xx_mfp_write(gpio_power_down, wlan_pd_mfp & 0xffff);

		/* set LDO10 and VBUCK2's sleep mode to be ACTIVE
			when 8787 is powered ON */
		pm860x_codec_reg_set_bits(PM8607_SLEEP_MODE1,
							(1 << 3) | (1 << 2), (3 << 2));
		pm860x_codec_reg_set_bits(PM8607_SLEEP_MODE4,
							(1 << 1) | (1 << 0), (3 << 0));

		/* enable 32KHz TOUT */
		enable_oscc_tout_s0();
		}
	else {
		/* Allow to go to D2 from PP1.
		 * This should be fixed by MG2-C1
		 * or Alkon rev 0.2*/
		if (is_wkr_mg2_462()){
			set_d2_from_high_pp_8787 = 0;
			if (dvfm_ind) {
				dvfm_enable_op_name("156M", dvfm_ind);
				dvfm_enable_op_name("156M_HF", dvfm_ind);
				dvfm_enable_op_name("208M", dvfm_ind);
				dvfm_enable_op_name("208M_HF", dvfm_ind);
				dvfm_enable_op_name("416M_VGA", dvfm_ind);
				dvfm_enable_op_name("416M", dvfm_ind);
			}
		}
		/*set wlan_pd pin to output low in low power
			mode to save power in low power mode */
		//wlan_pd_mfp &= ~0x100;
		/* when setting output to 0 set also PD: pulling takes effect */
		wlan_pd_mfp = (wlan_pd_mfp | 0x2000) & ~0x4100;
		pxa3xx_mfp_write(gpio_power_down, wlan_pd_mfp & 0xffff);

		/* set LDO10 and VBUCK2's sleep mode to be SLEEP
			when 8787 is powered OFF */
		pm860x_codec_reg_set_bits(PM8607_SLEEP_MODE1,
							(1 << 3) | (1 << 2), (2 << 2));
		pm860x_codec_reg_set_bits(PM8607_SLEEP_MODE4,
							(1 << 1) | (1 << 0), (2 << 0));

		/* disable 32KHz TOUT */
		disable_oscc_tout_s0();
	}

}

static void cd_wakelock_op(unsigned int sec)
{
	if (sec == 0)
		sec = WAKE_LOCK_CD_TIMEOUT;
	wake_lock_timeout(&cd_wake_lock, sec);
}

static void get_sdio_wakelock(unsigned int sec)
{
	wake_lock_timeout(&wifi_delayed_work_wake_lock, sec*HZ);
}

static void mmc_set_ops(struct sdhci_pxa *pxa)
{
	pxa->ops->set_con_clock = sdhci_pxa_set_con_clock;
	pxa->ops->ext_cd_status = ext_cd_status;
}

static int mmc1_lp_switch(unsigned int on, int with_card)
{
	struct mmc_card *card = (struct mmc_card *) with_card;
	struct mmc_host *mmc;
	struct sdhci_host *host;
	struct sdhci_pxa *pxa;
	
	if (card) {
		mmc = card->host;
		host = mmc_priv(mmc);
		pxa = sdhci_priv(host);
	} else
		goto exit;
	
	if (pxa->pdata->setpower) {
		if (on)
			pxa->pdata->setpower(card->host->parent,0);/*enter suspend */
		else
			pxa->pdata->setpower(card->host->parent,1);/*exit suspend */
	}
	
exit:
	return 0;
}

static struct sdhci_gpio sdhci_gpios[SDHCI_PIN_MAX] = {
{	.pin_type = SDHCI_PIN_CMD,
	.gpio_num = MFP_PIN_GPIO55,
	.description = "MMC CMD",
},
{	.pin_type = SDHCI_PIN_CLK,
	.gpio_num = MFP_PIN_GPIO56,
	.description = "MMC CLK",
},
{	.pin_type = SDHCI_PIN_DAT0,
	.gpio_num = MFP_PIN_GPIO57,
	.description = "MMC DATA 0",
},
{	.pin_type = SDHCI_PIN_DAT1,
	.gpio_num = MFP_PIN_GPIO58,
	.description = "MMC DATA 1"
},
{	.pin_type = SDHCI_PIN_DAT2,
	.gpio_num = MFP_PIN_GPIO59,
	.description = "MMC DATA 2"
},
{	.pin_type = SDHCI_PIN_DAT3,
	.gpio_num = MFP_PIN_GPIO60,
	.description = "MMC DATA 3"
},
};

struct sdhci_pins_data sdhci_pins_info = {
	.sdhci_gpios = sdhci_gpios,
	.number_of_pins = SDHCI_PIN_MAX,
};

static struct sdhci_pxa_platdata mci0_platform_data = {
	.quirks = SDHCI_QUIRK_NO_ENDATTR_IN_NOPDESC,
	.flags	= PXA_FLAG_CARD_PERMANENT | PXA_FLAG_ACITVE_IN_SUSPEND | PXA_FLAG_SD_8_BIT_CAPABLE_SLOT,
	.soc_set_ops = mmc_set_ops,
};

static struct sdhci_pxa_platdata mci1_platform_data = {
	.quirks = SDHCI_QUIRK_NO_ENDATTR_IN_NOPDESC,
#if defined(SD_CARD_NO_DETECT)
#else
	.ext_cd_gpio = mfp_to_gpio(MFP_PIN_GPIO_CD),
	.ext_cd_gpio_invert = 1,
	.ext_cd_cleanup = ext_cd_cleanup,
#endif
	.ext_cd_init = ext_cd_init,
	.setpower =  mci_setpower,
	.lp_switch = mmc1_lp_switch,
	.soc_set_ops = mmc_set_ops,
	.cd_wakelock = cd_wakelock_op,
/* 	.max_speed = 24000000,   */ /*down to 24MHZ */
	.pins_data = &sdhci_pins_info,
	.MM4_RETCLK_DEL_paddr = 0x5550A204, /*default value 0x20002000 vs 0x20003124 */
};

static struct sdhci_pxa_platdata mci2_platform_data = {
	.flags  = PXA_FLAG_DISABLE_CLOCK_GATING |
						PXA_FLAG_SDIO_RESUME |
						PXA_FLAG_CARD_PERMANENT |
						// start marvell johnryu
						//PXA_FLAG_SDIO_IRQ_ALWAYS_ON,
						PXA_FLAG_SDIO_IRQ_ALWAYS_ON |
						PXA_FLAG_DISABLE_PROBE_CDDET,
						// end marvell johnryu
	.quirks = SDHCI_QUIRK_NO_ENDATTR_IN_NOPDESC,
	.get_sdio_wakelock      = get_sdio_wakelock,
};

static void __init init_mmc(void)
{
	/*add emmc only, need to add sdcard and sdio later*/
	/* disable emmc */
	/*pxa95x_set_mci_info(0, &mci0_platform_data);*/
	pxa95x_set_mci_info(1, &mci1_platform_data);
#ifdef CONFIG_SD8XXX_RFKILL
	add_sd8x_rfkill_device(mfp_to_gpio(MFP_PIN_GPIO77),
			mfp_to_gpio(MFP_PIN_GPIO67),
			&mci2_platform_data.pmmc,
			wifi_set_power);

	pxa95x_set_mci_info(2, &mci2_platform_data);
#endif
}

#endif

static struct vmeta_plat_data vmeta_plat_data = {
	.bus_irq_handler = pxa95x_vmeta_bus_irq_handler,
	.set_dvfm_constraint = pxa95x_vmeta_set_dvfm_constraint,
	.unset_dvfm_constraint = pxa95x_vmeta_unset_dvfm_constraint,
	.init_dvfm_constraint = pxa95x_vmeta_init_dvfm_constraint,
	.clean_dvfm_constraint = pxa95x_vmeta_clean_dvfm_constraint,
	.axi_clk_available = 1,
	.decrease_core_freq = pxa95x_vmeta_decrease_core_freq,
	.increase_core_freq = pxa95x_vmeta_increase_core_freq,
};

static void __init init_vmeta(void)
{
	pxa95x_set_vmeta_info(&vmeta_plat_data);
}

static struct platform_device *devices[] __initdata = {
	&pxa95x_device_i2c1,
	&pxa95x_device_i2c2,
	&pxa95x_device_i2c3,
#if defined(CONFIG_TOUCHSCREEN_VNC)
	&vnc_device,
#endif
};

#if defined(CONFIG_KEYBOARD_PXA27x) || defined(CONFIG_KEYBOARD_PXA27x_MODULE)
static unsigned int matrix_key_map[] = {
	/* KEY(row, col, key_code) */
#if defined(CONFIG_KEY_FOR_ALKON_REV02)
	KEY(2,2,KEY_VOLUMEUP),
	KEY(2,3,KEY_VOLUMEDOWN),
	KEY(3,1,KEY_BACK),
	KEY(3,2,KEY_MENU),
	KEY(3,3,KEY_HOME)
#else
	KEY(2,1,KEY_SEARCH),
	KEY(2,2,KEY_VOLUMEUP),
	KEY(2,3,KEY_VOLUMEDOWN),
	KEY(3,1,KEY_BACK),
	KEY(3,2,KEY_MENU),
	KEY(3,3,KEY_HOME)
#endif


#if 0        
	/* KEY(row, col, key_code) */
	KEY(1, 3, KEY_0), KEY(0, 0, KEY_1), KEY(1, 0, KEY_2), KEY(2, 0, KEY_3),
	KEY(0, 1, KEY_4), KEY(1, 1, KEY_5), KEY(2, 1, KEY_6), KEY(0, 2, KEY_7),
	KEY(1, 2, KEY_8), KEY(2, 2, KEY_9),

	KEY(0, 3, KEY_KPASTERISK), 	/* * */
	KEY(2, 3, KEY_KPDOT),   	/* # */

	KEY(4, 0, KEY_HOME),
	KEY(3, 3, KEY_END),
	KEY(4, 1, KEY_BACK),

	KEY(3, 2, KEY_SEND),

	KEY(4, 4, KEY_SELECT),    /* volume rocker push */
	KEY(3, 4, KEY_VOLUMEUP),
	KEY(2, 4, KEY_VOLUMEDOWN),

	KEY(3, 0, KEY_F22),	/* soft1 */
	KEY(3, 1, KEY_F23),	/* soft2 */

	KEY(1, 4, KEY_CAMERA),      /* camera full push */
	KEY(0, 4, KEY_ZOOM),		/* camera half push */

	KEY(4, 3, KEY_WWW),		/* surf button */
	KEY(4, 2, KEY_OK),		/* ok button */

	KEY(0, 5, KEY_ENTER),       /* joystick push */
	KEY(4, 5, KEY_LEFT),
	KEY(3, 5, KEY_RIGHT),
	KEY(2, 5, KEY_UP),
	KEY(1, 5, KEY_DOWN),
#endif
};

static struct pxa27x_keypad_platform_data keypad_info = {
	.matrix_key_rows	= 5,
	.matrix_key_cols	= 6,
	.matrix_key_map		= matrix_key_map,
	.matrix_key_map_size	= ARRAY_SIZE(matrix_key_map),
	.debounce_interval	= 30,
};
#endif /* CONFIG_KEYBOARD_PXA27x || CONFIG_KEYBOARD_PXA27x_MODULE */

static int al25_init_irq(void)
{
	int ret;

	ret = gpio_request(mfp_to_gpio(MFP_PIN_GPIO92), "Micro USB Detection");
	
	if (ret)
		goto err_request_mUSB;
		gpio_direction_input(mfp_to_gpio(MFP_PIN_GPIO92));
	
	/*pxa3xx_mfp_set_edge(MFP_CFG_PIN(MFP_PIN_GPIO96),MFP_EDGE_FALL);*/
	return 0;

	err_request_mUSB:
	return ret;
}

static int al25_ack_irq(void)
{
	return 0;
}

static struct AL25_platform_data  al25_data = {
    .init_irq = al25_init_irq,
    .ack_irq = al25_ack_irq,
};

#ifdef CONFIG_FUELGAUGE_STC3105
static int stc3105_init_irq(void)
{
	int ret;

	ret = gpio_request(mfp_to_gpio(MFP_PIN_GPIO110), "FuelGauge Alert Detection");
	
	if (ret)
		goto err_request_mUSB;

	gpio_direction_input(mfp_to_gpio(MFP_PIN_GPIO110));
	
	return 0;

	err_request_mUSB:
	return ret;
}

static int stc3105_ack_irq(void)
{
	return 0;
}

static struct stc31xx_platform_data  stc3105_data = {
    .init_irq = stc3105_init_irq,
    .ack_irq = stc3105_ack_irq,
};
#endif


#if defined(CONFIG_PXA95X_CAMERA)
static int cam_power_set(int flag)
{
	static struct clk *axi_clk = NULL;
	static struct clk *sci1_clk = NULL;
	static struct clk *sci2_clk = NULL;
	static struct clk *csi_tx_esc = NULL;

//    printk(KERN_ERR "cam_power_set");

	if (sci1_clk == NULL) {
		sci1_clk = clk_get(NULL, "SCI1CLK");
		if (IS_ERR(sci1_clk)) {
			printk(KERN_ERR "unable to get SCI1CLK\n");
			return PTR_ERR(sci1_clk);
		};
	};

	if (sci2_clk == NULL) {
		sci2_clk = clk_get(NULL, "SCI2CLK");
		if (IS_ERR(sci2_clk)) {
			printk(KERN_ERR "unable to get SCI2CLK\n");
			return PTR_ERR(sci2_clk);
		};
	};

	if (csi_tx_esc == NULL) {
		csi_tx_esc = clk_get(NULL, "CSI_TX_ESC");
		if (IS_ERR(csi_tx_esc)) {
			printk(KERN_ERR "unable to get CSI_TX_ESC\n");
			return PTR_ERR(csi_tx_esc);
		};
	};

	if (axi_clk == NULL) {
		axi_clk = clk_get(NULL, "AXICLK");
		if (IS_ERR(axi_clk)) {
			printk(KERN_ERR "unable to get AXICLK\n");
			return PTR_ERR(axi_clk);
		};
	};

	if (flag) {

		/* The order of enabling matters! AXI must be the 1st one */
		clk_enable(axi_clk);
		clk_enable(csi_tx_esc);
		clk_enable(sci1_clk);
		clk_enable(sci2_clk);
	} else {
		clk_disable(csi_tx_esc);
		clk_disable(sci2_clk);
		clk_disable(sci1_clk);
		clk_disable(axi_clk);
	}

	clk_put(sci1_clk);
	clk_put(sci2_clk);
	clk_put(csi_tx_esc);
	clk_put(axi_clk);

	return 0;
}
/* sensor init */
static int sensor_power_set(int res, int flag)
{
#if 0
	/*
	 * RES, 0, LOW RESOLUTION, 1 HIGH RESOLUTION
	 * FLAG, 1: ON, 0: OFF
	 * GPIO93 LOW FOR HIGH
	 * GPIO14 LOW FOR LOW
	 */
	int high_pin = 0;
	int low_pin = mfp_to_gpio(MFP_PIN_GPIO108);
 	static struct regulator *vcamera;
	static int vcamera_inited;

	high_pin = mfp_to_gpio(MFP_PIN_GPIO1O9);
	cam_power_set(flag);

	if (gpio_request(high_pin, "CAM_EANBLE_HI_SENSOR")) {
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d\n", high_pin);
		return -EIO;
	}

		if (!vcamera_inited) {
			vcamera = regulator_get(NULL, "v_cam");
			if (!IS_ERR(vcamera))
				vcamera_inited = 1;
			else
				printk(KERN_ERR "cam: failed to get regulator\n");
		}
	if (res) {
		if (flag) {
			gpio_direction_output(high_pin, 0);	/* enable */
			msleep(1);
			if (!IS_ERR(vcamera))
				regulator_enable(vcamera);
			else
				printk(KERN_ERR "cam: failed to get regulator\n");
		} else {
			if (!IS_ERR(vcamera))
				regulator_disable(vcamera);
			else
				printk(KERN_ERR "cam: failed to get regulator\n");
			gpio_direction_output(high_pin, 1);	/* disable */
		}
	} else{
		if (gpio_request(low_pin, "CAM_EANBLE_LOW_SENSOR")) {
			printk(KERN_ERR "Request GPIO failed,"
					"gpio: %d\n", low_pin);
			return -EIO;
		}
		if (flag) {
			/* high resolution is bridge */
			gpio_direction_output(high_pin, 0);	/* enable */
			gpio_direction_output(low_pin, 0);	/* enable */

		} else {
			gpio_direction_output(high_pin, 1);	/* disable */
			gpio_direction_output(low_pin, 1);	/* disable */
		}
		gpio_free(low_pin);
	}


	gpio_free(high_pin);
#endif
	return 0;
}

/* sensor init over */
static struct cam_platform_data cam_ops = {
	.power_set = cam_power_set,
};

#if 0
static struct sensor_platform_data ov5642_sensor_data = {
	.id = SENSOR_HIGH,
	.power_set = sensor_power_set,
};

static struct sensor_platform_data ov7690_sensor_data = {
	.id = SENSOR_LOW,
	.power_set = sensor_power_set,
};


static struct sensor_platform_data isx006_sensor_data = {
	.id = SENSOR_HIGH,
	.power_set = sensor_power_set,
};
#endif

#if defined(CONFIG_CAMERA_S5K4ECGX)
static struct sensor_platform_data s5k4ecgx_sensor_data = {
	.id = SENSOR_HIGH,
	.power_set = sensor_power_set,
};
#endif

#if defined(CONFIG_CAMERA_MT9V114)
static struct sensor_platform_data mt9v114_sensor_data = {
	.id = SENSOR_LOW,
	.power_set = sensor_power_set,
};
#endif

#if defined(CONFIG_CAMERA_S5K5CCGX)
static struct sensor_platform_data s5k5ccgx_sensor_data = {
	.id = SENSOR_HIGH,
	.power_set = sensor_power_set,
};
#endif

#endif/*CONFIG_PXA95X_CAMERA*/

#if defined(CONFIG_HDMI_SI9226)
static void SI9226_hdmi_reset(void)
{
	static struct regulator *vhdmi;
	static int vhdmi_inited;

	if (!vhdmi_inited) {
		vhdmi = regulator_get(dev, "v_hdmi");
		if (!IS_ERR(vhdmi))
			vhdmi_inited = 1;
		else
			printk(KERN_ERR "hdmi: failed to get regulator\n");
	}

	if (gpio_request(mfp_to_gpio(MFP_PIN_GPIO43), "hdmi reset"))
		printk(KERN_ERR "hdmi gpio_request: failed!\n");

	printk(KERN_INFO "SI9226 hdmi power %s\n", on?"on":"off");
	if (on) {
		if (!IS_ERR(vhdmi))
			regulator_enable(vhdmi);
		else
			printk(KERN_ERR "hdmi: regulator is not correctly get\n");
		msleep(10);
		gpio_direction_output(mfp_to_gpio(MFP_PIN_GPIO43), 0);
		msleep(1);
		gpio_direction_output(mfp_to_gpio(MFP_PIN_GPIO43), 1);
	} else {
		gpio_direction_output(mfp_to_gpio(MFP_PIN_GPIO43), 0);
		msleep(10);
		if (!IS_ERR(vhdmi))
			regulator_disable(vhdmi);
		else
			printk(KERN_ERR "hdmi: regulator is not correctly get\n");
	}
	gpio_free(mfp_to_gpio(MFP_PIN_GPIO43));
	msleep(10);
}
#endif

static unsigned long lis33ldl_min_delay = 20;
static struct i2c_board_info i2c2_info[] = {  
#if defined(CONFIG_PXA95X_CAMERA)
#if defined(CONFIG_CAMERA_S5K4ECGX)
    {
    	.type           = "s5k4ecgx",   /* gforce main sensor*/
    	.addr           = 0x56,             /* 0x5A, */
    	.platform_data	= &s5k4ecgx_sensor_data,
    },
#endif
#if defined(CONFIG_CAMERA_MT9V114)
    {
    	.type           = "mt9v114",   /* gforce sub sensor*/
    	.addr           = 0x3C,             /* 0x7A */
    	.platform_data	= &mt9v114_sensor_data,
    },
#endif
#if defined(CONFIG_CAMERA_S5K5CCGX)
    {
    	.type           = "s5k5ccgx",   /* gforce main sensor*/
    	.addr           = 0x3C,         /* 0x78 */
    	.platform_data	= &s5k5ccgx_sensor_data,
    },
#endif

#if 0 /* old*/

    {
    	.type           = "isx006",         /* beni main sensor*/
    	.addr           = 0x1a,             /* 0x3c, */
    	.platform_data	= &isx006_sensor_data,
    },
#endif /*old*/
#if 0
#if defined(CONFIG_VIDEO_PXA95X_OV5642)
	{
		.type		= "ov5642",
		.addr		= 0x3c,
		.platform_data	= &ov5642_sensor_data,
	},
#endif

#if defined(CONFIG_VIDEO_PXA95X_OV7690)
	{
		.type		= "ov7690",
		.addr		= 0x21,
		.platform_data	= &ov7690_sensor_data,
	},
#endif
#endif



#endif

#if defined(CONFIG_HDMI_SI9226)
	{
		.type		= "SI9226",
		.addr		= 0x3B,
		.platform_data = SI9226_hdmi_power,
	},
	{
		.type		= "SI9226-ctrl",
		.addr		= 0x66,
	},
#endif

};

static struct i2c_board_info i2c3_info[] = {
	    {
			.type   		= "AL25",
			.addr           = 0x25, /*7Bit Slave Address */
			.platform_data  = &al25_data,
			.irq            = IRQ_GPIO(mfp_to_gpio(MFP_PIN_GPIO92)),
        },

#if defined(CONFIG_SENSORS_BMA222)
	{
		.type		= "bma222",    /*G-Sensor*/
		.addr		= 0x08,
	},
#endif

#if defined(CONFIG_SENSORS_MMC328X)
	{
		.type		= "mmc328x",
		.addr		= 0x30,
	},
#endif

#if defined(CONFIG_SENSORS_GP2A)
	{
		.type		= "gp2a_prox",  /*Proximity Sensor*/
		.addr		= 0x44,         
	},
#endif

#if defined(CONFIG_SENSORS_TAOS)
	{
		.type		= "taos",  /*Proximity Sensor*/
		.addr		= 0x39,         
	},
#endif

};

#ifdef CONFIG_FUELGAUGE_STC3105
#define STC3105_I2C_SDA_GPIO	121
#define STC3105_I2C_SCL_GPIO	120

static struct i2c_gpio_platform_data i2c_stc3105_bus_data = {
	.sda_pin = STC3105_I2C_SDA_GPIO,
	.scl_pin = STC3105_I2C_SCL_GPIO,
	.udelay  = 3,  //// brian :3
	.timeout = 100,
};

static struct platform_device i2c_stc3105_bus_device = {
	.name	= "i2c-gpio",
	.id		= 5, /* pxa95x-i2c are bus 0, 1, 2, 3, 4 so start at 5 */
	.dev		= {
		.platform_data = &i2c_stc3105_bus_data,
	}
};


static struct i2c_board_info __initdata stc3105_i2c_devices[] = {
	{
		I2C_BOARD_INFO("fuelgauge_stc3105", 0x70),
		.platform_data  = &stc3105_data,
		.irq = IRQ_GPIO(mfp_to_gpio(MFP_PIN_GPIO110)),
	},
};
#endif

#if defined(CONFIG_VIDEO_PXA955)
static int camera0_power(struct device *dev, int flag)
{
	int gpio_pin = mfp_to_gpio(MFP_PIN_GPIO81);

	if (gpio_request(gpio_pin, "CAM_EANBLE_HI_SENSOR")) {
		printk(KERN_ERR "cam: Request GPIO failed,"
				"gpio: %d\n", gpio_pin);
		return -EIO;
	}

	if (flag) {
		gpio_direction_output(gpio_pin, 0);	/* enable */
		msleep(1);
	} else {
		gpio_direction_output(gpio_pin, 1);	/* disable */
	}

	gpio_free(gpio_pin);
	return 0;
}

static int camera1_power(struct device *dev, int flag)
{
#if 0
	int gpio_pin = mfp_to_gpio(MFP_PIN_GPIO14);

	if (gpio_request(gpio_pin, "CAM_EANBLE_LOW_SENSOR")) {
		printk(KERN_ERR "cam: Request GPIO failed,"
				"gpio: %d \n", gpio_pin);
		return -EIO;
	}
	if (flag) {
		gpio_direction_output(gpio_pin, 0);	/* enable */
		msleep(1);
	} else {
		gpio_direction_output(gpio_pin, 1);	/* disable */
	}
	gpio_free(gpio_pin);
#endif
	return 0;
}

static struct i2c_board_info camera_i2c[] = {
	{
		I2C_BOARD_INFO("ov5642", 0x3c),
	}, {
		I2C_BOARD_INFO("ov7690", 0x21),
	},
};

static struct soc_camera_link iclink[] = {
	{
		.bus_id			= 0, /* Must match with the camera ID */
		.board_info		= &camera_i2c[0],
		.i2c_adapter_id		= 1,
		.power = camera0_power,
		.module_name		= "ov5642",
	}, {
		.bus_id			= 0, /* Must match with the camera ID */
		.board_info		= &camera_i2c[1],
		.i2c_adapter_id		= 1,
		.power = camera1_power,
		.module_name		= "ov7690",
	},
};

static struct platform_device camera[] = {
	{
		.name	= "soc-camera-pdrv",
		.id	= 0,
		.dev	= {
			.platform_data = &iclink[0],
		},
	}, {
		.name	= "soc-camera-pdrv",
		.id	= 1,
		.dev	= {
			.platform_data = &iclink[1],
		},
	},
};

struct pxa95x_camera_pdata {
	unsigned long flags;
	unsigned long mclk_10khz;
	struct device *dma_dev;
};

/* TODO: add platform data into this struct*/
static struct pxa95x_camera_pdata cam_pdata = {
	.mclk_10khz	= 2000,
};

#endif

static void __init init_cam(void)
{
#if defined(CONFIG_PXA95X_CAMERA)
	pxa95x_device_cam0.dev.platform_data = &cam_ops;
	platform_device_register(&pxa95x_device_cam0);
	platform_device_register(&pxa95x_device_csi);
#endif

#if defined(CONFIG_VIDEO_PXA955)
	platform_device_register(&pxa95x_device_csi);

#if defined(CONFIG_SOC_CAMERA_OV5642)
	platform_device_register(camera[0]);
#endif
#if defined(CONFIG_SOC_CAMERA_OV7690)
	platform_device_register(&camera[1]);
#endif

	pxa95x_device_cam0.dev.platform_data = &cam_pdata;
	platform_device_register(&pxa95x_device_cam0);

	/* TODO: add sencond camera controller */
	/*pxa95x_device_cam1.dev.platform_data = &cam_pdata;*/
	/*platform_device_register(&pxa95x_device_cam1);*/
#endif
}

#ifdef CONFIG_PROC_FS

#define GEN_REG3		__REG(0x42404008)

static struct regulator *g_gps_regulator, *g_gps_ext_lna_regulator;
static int g_gps_reset, g_gps_on;
static int g_is_gps_on;

static void gps_eclk(int flag);

/* GPS: power on/off control */
static void gps_power_on(void)
{
	if (g_is_gps_on) {
		pr_err("%s: gps driver already on\n", __func__);
		return;
	}

	g_gps_reset = mfp_to_gpio(MFP_PIN_GPIO61);
	if (gpio_request(g_gps_reset, "gpio_gps_reset")) {
		pr_err("%s: g_gps_reset request failed: %d\n", __func__, g_gps_reset);
		return;
	}
	pr_info("%s: g_gps_reset ok: %d\n", __func__, g_gps_reset);

	g_gps_on = mfp_to_gpio(MFP_PIN_GPIO62);
	if (gpio_request(g_gps_on, "gpio_gps_on")) {
		pr_err("%s: gpio_gps_on request failed: %d\n", __func__, g_gps_on);
		goto exit1;
	}
	pr_info("%s: gpio_gps_on ok: %d\n", __func__, g_gps_on);

	g_gps_regulator = regulator_get(NULL, "v_gps");
	if (!g_gps_regulator) {
		pr_err("%s: regulator_get failed: v_gps\n", __func__);
		goto exit2;
	}
	pr_info("%s: regulator_get ok: v_gps\n", __func__);

	g_gps_ext_lna_regulator = regulator_get(NULL, "v_gps_txco");
	if (!g_gps_ext_lna_regulator) {
		pr_err("%s: regulator_get failed: v_gps_txco\n", __func__);
		goto exit3;
	}
	pr_info("%s: regulator_get ok: v_gps_txco\n", __func__);

	gpio_direction_output(g_gps_reset, 0);
	gpio_direction_output(g_gps_on, 0);
	gps_eclk(0);
	enable_oscc_tout_s0();

	if (regulator_enable(g_gps_regulator) < 0) {
		pr_err("%s: regulator_enable failed: v_gps\n", __func__);
		goto exit4;
	}

	if (regulator_enable(g_gps_ext_lna_regulator) < 0) {
		pr_err("%s: regulator_enable failed: v_gps_txco\n", __func__);
		goto exit4;
	}

	g_is_gps_on = 1;
	pr_info("%s: sirf gps chip powered on\n", __func__);
	return;

exit4:
	regulator_put(g_gps_ext_lna_regulator);
	g_gps_ext_lna_regulator = NULL;
exit3:
	regulator_put(g_gps_regulator);
	g_gps_regulator = NULL;
exit2:
	gpio_free(g_gps_on);
	g_gps_on = (int) NULL;
exit1:
	gpio_free(g_gps_reset);
	g_gps_reset = (int) NULL;
}

static void gps_power_off(void)
{
	if (!g_is_gps_on) {
		pr_warning("%s: gps driver already off\n", __func__);
		/* In this case, do not return, release all resources. */
	}

	gps_eclk(0);
	disable_oscc_tout_s0();

	if (g_gps_reset) {
		gpio_direction_input(g_gps_reset);
		gpio_free(g_gps_reset);
		g_gps_reset = (int) NULL;
	}

	if (g_gps_on) {
		gpio_direction_input(g_gps_on);
		gpio_free(g_gps_on);
		g_gps_on = (int) NULL;
	}

	if (g_gps_regulator) {
		if (regulator_disable(g_gps_regulator) < 0)
			pr_err("%s: regulator_disable failed: g_gps_regulator\n", __func__);
		regulator_put(g_gps_regulator);
		g_gps_regulator = NULL;
	}

	if (g_gps_ext_lna_regulator) {
		if (regulator_disable(g_gps_ext_lna_regulator) < 0)
			pr_err("%s: regulator_disable failed: g_gps_ext_lna_regulator\n", __func__);
		regulator_put(g_gps_ext_lna_regulator);
		g_gps_ext_lna_regulator = NULL;
	}

	g_is_gps_on = 0;
	pr_info("%s: sirf gps chip powered off\n", __func__);
}

static void gps_reset(int flag)
{
	if (!g_gps_reset) {
		pr_err("%s: illegal handle, g_gps_reset: %d\n", __func__, g_gps_reset);
		return;
	}
	if ((flag != 0) && (flag != 1)) {
		pr_err("%s: illegal value, flag: %d\n", __func__, flag);
		return;
	}
	gpio_direction_output(g_gps_reset, flag);
	/*pr_info("%s: sirf gps chip reset\n", __func__);*/
}

static void gps_on_off(int flag)
{
	if (!g_gps_on) {
		pr_err("%s: illegal handle, g_gps_on: %d\n", __func__, g_gps_on);
		return;
	}
	if ((flag != 0) && (flag != 1)) {
		pr_err("%s: illegal value, flag: %d\n", __func__, flag);
		return;
	}
	gpio_direction_output(g_gps_on, flag);
	/*pr_info("%s: sirf gps chip offon\n", __func__);*/
}

static void gps_eclk(int flag)
{
	unsigned long reg_image;
	unsigned long cpsr;

	if ((flag != 0) && (flag != 1)) {
		pr_err("%s: illegal value, flag: %d\n", __func__, flag);
		return;
	}

	/* lock interrupts */
	local_irq_save(cpsr);

	/* read GEN_REG3 register */
	reg_image = GEN_REG3;

	/* ignore (clear) all bits, accept bits 0 & 1 () */
	reg_image &= 0x00000003;

	/* modify the value of CKRSW1 bit */
	if (flag)
		reg_image |= (1 << 16); /* set bit 16 (CKRSW1) */
	else
		reg_image &= ~(1 << 16); /* clear bit 16 (CKRSW1) */

	/* write GEN_REG3 register */
	GEN_REG3 = reg_image;

	/* unlock interrupts */
	local_irq_restore(cpsr);

	/*pr_info("%s: sirf gps chip eclk\n", __func__);*/
}

#define SIRF_STATUS_LEN	16
static char sirf_status[SIRF_STATUS_LEN] = "off";

static ssize_t sirf_read_proc(char *page, char **start, off_t off,
							  int count, int *eof, void *data)
{
	int len = strlen(sirf_status);

	sprintf(page, "%s\n", sirf_status);
	return len + 1;
}

static ssize_t sirf_write_proc(struct file *filp,
							   const char *buff, size_t len, loff_t *off)
{
	char messages[256];
	int flag, ret;
	char buffer[7];

	if (len > 255)
		len = 255;

	memset(messages, 0, sizeof(messages));

	if (!buff || copy_from_user(messages, buff, len))
		return -EFAULT;

	if (strlen(messages) > (SIRF_STATUS_LEN - 1)) {
		pr_warning("%s: messages too long, (%d) %s\n",
			__func__, strlen(messages), messages);
		return -EFAULT;
	}

	if (strncmp(messages, "off", 3) == 0) {
		strcpy(sirf_status, "off");
		gps_power_off();
	} else if (strncmp(messages, "on", 2) == 0) {
		strcpy(sirf_status, "on");
		gps_power_on();
	} else if (strncmp(messages, "reset", 5) == 0) {
		strcpy(sirf_status, messages);
		ret = sscanf(messages, "%s %d", buffer, &flag);
		if (ret == 2)
			gps_reset(flag);
	} else if (strncmp(messages, "sirfon", 5) == 0) {
		strcpy(sirf_status, messages);
		ret = sscanf(messages, "%s %d", buffer, &flag);
		if (ret == 2)
			gps_on_off(flag);
	} else if (strncmp(messages, "eclk", 4) == 0) {
		strcpy(sirf_status, messages);
		ret = sscanf(messages, "%s %d", buffer, &flag);
		if (ret == 2)
			gps_eclk(flag);
	} else
		pr_info("usage: echo [off|on|reset|sirfon|eclk] [0|1] > /proc/driver/sirf\n");

	return len;
}

static void create_sirf_proc_file(void)
{
	struct proc_dir_entry *sirf_proc_file = NULL;

	sirf_proc_file = create_proc_entry("driver/sirf", 0644, NULL);
	if (!sirf_proc_file) {
		pr_err("%s: create proc file failed\n", __func__);
		return;
	}

	sirf_proc_file->read_proc = sirf_read_proc;
	sirf_proc_file->write_proc = (write_proc_t *)sirf_write_proc;
}
#endif

#ifdef CONFIG_PROC_FS
#define PROC_FILE	"driver/pmic_id"
#define PROC_AUDIO_POWER_FILE	"driver/audio_power"

static struct proc_dir_entry *proc_file;
static struct proc_dir_entry *proc_audio_power_file;

static long pmic_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	/*struct pmic_ops *ops = file->private_data;*/
	void __user *uarg = (void __user *)arg;
	int ret = -EINVAL;
	static u8 icotl_pmic_id;  /* static is '0' initialized */

	lock_kernel();

	switch (cmd) {
	case IOCTL_PMIC_ID_GET:
		if (icotl_pmic_id == 0)
#if defined(CONFIG_PMIC_D1980)
            /* TODO: 12Apr2011. DLG Check.*/
            icotl_pmic_id = 0x58;
#else
			icotl_pmic_id = pm860x_codec_reg_read(PM8607_CHIP_ID);
#endif
		ret = put_user(icotl_pmic_id, (unsigned long __user *)uarg);
		pr_info("-------> icotl_pmic_id %x\n", icotl_pmic_id);
		if (ret)
			ret = -EFAULT;
		break;

	default:
		/*if (ops->ioctl)
		  ret = ops->ioctl(cmd, arg);*/
		panic("pmic_ioctl");
		break;
	}

	unlock_kernel();

	return ret;
}

static ssize_t proc_read(struct file *filp, char __user *buffer,
				size_t length, loff_t *offset)
{
	printk(KERN_ALERT "proc_read - no action done\n");

	return 0;
}

static ssize_t proc_write(struct file *filp, const char *buff,
				size_t len, loff_t *off)
{
	printk(KERN_ALERT "proc_write - no action done\n");

	return len;
}

static const struct file_operations proc_ops = {
	.read   = proc_read,
	.write  = proc_write,
	.unlocked_ioctl  = pmic_ioctl,
};

static ssize_t proc_audio_power_read(struct file *filp, char __user *buffer,
				size_t length, loff_t *offset)
{
	printk(KERN_ALERT "proc_audio_power_read - no action done\n");

	return 0;
}

static ssize_t proc_audio_power_write(struct file *filp, const char *buff,
				size_t len, loff_t *off)
{
	static unsigned int dvfm_ind, index_init=0;
	unsigned int ret;

    if(len<1) return 0;
    
	if (index_init == 0) {
		index_init = 1;
		ret = dvfm_register("Audio Power", &dvfm_ind);
		if (ret) {
			pr_err("Error %d: Fails to register Audio into dvfm.\n",
				ret);
			}
	}

    if(buff[0]=='1') {
		dvfm_disable_op_name("D2", dvfm_ind);
		dvfm_disable_op_name("CG", dvfm_ind);
		printk(KERN_ALERT "proc_audio_power_write - constraint enabled\n");
	} else {
		dvfm_enable_op_name("D2", dvfm_ind);
		dvfm_enable_op_name("CG", dvfm_ind);
		printk(KERN_ALERT "proc_audio_power_write - constraint removed\n");		
	}

	return len;
}

static const struct file_operations proc_audio_power_ops = {
	.read   = proc_audio_power_read,
	.write  = proc_audio_power_write,
	/*.unlocked_ioctl  = NULL,*/
};

static void create_proc_file(void)
{
	proc_file = create_proc_entry(PROC_FILE, 0664, NULL);
	if (proc_file) {
		proc_file->proc_fops = &proc_ops;
		printk(KERN_ALERT "PMIC_ID proc file created!\n");
	} else
		printk(KERN_ALERT "PMIC_ID proc file create failed!\n");
		
	proc_audio_power_file = create_proc_entry(PROC_AUDIO_POWER_FILE, 0664, NULL);
	if (proc_audio_power_file) {
		proc_audio_power_file->proc_fops = &proc_audio_power_ops;
		printk(KERN_ALERT "Audio Power proc file created!\n");
	} else
		printk(KERN_ALERT "Audio Power proc file create failed!\n");
}
#endif

static void __init init(void)
{
	regulator_init();

	set_abu_init_func(alkon_pxa95x_abu_mfp_init);
	set_ssp_init_func(ssp3_mfp_init);
	pxa_set_ffuart_info(&ffuart_info);
	pxa_set_stuart_info(&stuart_info);
	platform_device_add_data(&pxa95x_device_i2c1, &i2c1_pdata,
				 sizeof(i2c1_pdata));
	platform_device_add_data(&pxa95x_device_i2c2, &i2c2_pdata,
				 sizeof(i2c2_pdata));
	platform_device_add_data(&pxa95x_device_i2c3, &i2c3_pdata,
				 sizeof(i2c3_pdata));

	platform_add_devices(ARRAY_AND_SIZE(devices));
	i2c_register_board_info(0, ARRAY_AND_SIZE(i2c1_info));
	i2c_register_board_info(1, ARRAY_AND_SIZE(i2c2_info));
	i2c_register_board_info(2, ARRAY_AND_SIZE(i2c3_info));
	

      /* + touch init */
       platform_device_register(&i2c_bus_device);
       i2c_register_board_info(4, ARRAY_AND_SIZE(cy8ctma340_i2c_devices));
	init_tsp();
#if defined(CONFIG_PMIC_D1980)
        platform_device_register(&cy8ctma340_ts_device);
#endif
      /* - touch init */

	platform_device_register(&backlight_device);
       platform_device_register(&kp_bl_device);
      platform_device_register(&torch_flash_device);
       
#if defined(CONFIG_PMIC_D1980)
    platform_device_register(&android_hsdetect_device);
    platform_device_register(&android_vibrator_device);
#endif

#ifdef CONFIG_FUELGAUGE_STC3105
       platform_device_register(&i2c_stc3105_bus_device);
       i2c_register_board_info(5, ARRAY_AND_SIZE(stc3105_i2c_devices));
#endif

#ifdef CONFIG_USB_GADGET_PXA_U2O
        pxa9xx_device_u2o.dev.platform_data = &u2o_info;
        platform_device_register(&pxa9xx_device_u2o);
#endif

#ifdef CONFIG_USB_OTG
        pxa9xx_device_u2ootg.dev.platform_data = &u2o_info;
        platform_device_register(&pxa9xx_device_u2ootg);

        pxa9xx_device_u2oehci.dev.platform_data = &u2o_info;
        platform_device_register(&pxa9xx_device_u2oehci);
#endif

#ifdef CONFIG_USB_ANDROID
       android_add_usb_devices();
#endif
	platform_device_register(&android_device_yaffs);
	/* dvfm device */
#ifdef CONFIG_PXA95x_DVFM
	set_pxa95x_freq_info(&freq_mach_info);
#endif

	/* performance monitor unit */
	pxa95x_set_pmu_info(NULL);

    init_cam();

#if defined(CONFIG_FB_PXA95x)
	init_lcd();
#endif

#if defined(CONFIG_MMC_SDHCI_PXA)
	init_mmc(); 
	wake_lock_init(&wifi_delayed_work_wake_lock, WAKE_LOCK_SUSPEND, "wifi_delayed_work");
#endif

	/* Init boot flash - sync mode in case of ONENAND */
	pxa_boot_flash_init(1);


#ifdef CONFIG_PM
	pxa95x_wakeup_register(&wakeup_ops);
#endif

#ifdef CONFIG_ANDROID_PMEM
        pxa_add_pmem("pmem", reserving_size, 0, 1, 1);
        pxa_add_pmem("pmem_adsp", 0, 0, 0, 0);
#endif

#if defined(CONFIG_UIO_VMETA)
	init_vmeta();
#endif

#if defined(CONFIG_KEYBOARD_PXA27x) || defined(CONFIG_KEYBOARD_PXA27x_MODULE)
	pxa_set_keypad_info(&keypad_info);
#endif

#if defined(CONFIG_SENSORS_ORIENTATION)
	platform_device_register_simple("orientation", 0, NULL, 0);
#endif


#ifdef CONFIG_PROC_FS
	create_proc_file();
#endif

#ifdef CONFIG_PROC_FS
	/* create proc for SiRF GPS control */
	create_sirf_proc_file();
#endif

	platform_device_register(&sim_dev);
}


/*
Details on struct machine_desc, arch/arm/include/asm/mach/arch.h:
	.phys_io, .io_pg_offst: used to create basic MMU mapping in
		arch/arm/kernel/head.S. Needed for CONFIG_DEBUG_LL to work.
*/
MACHINE_START(ALKON_MG2, "PXA968 Handheld Platform (aka ALKON)")
	.phys_io	= 0x40000000,
	.io_pg_offst	= (io_p2v(0x40000000) >> 18) & 0xfffc,
	.map_io         = pxa_map_io,
	.nr_irqs	= ALKON_NR_IRQS,
	.init_irq       = pxa95x_init_irq,
	.timer          = &pxa_timer,
	.init_machine   = init,
MACHINE_END

