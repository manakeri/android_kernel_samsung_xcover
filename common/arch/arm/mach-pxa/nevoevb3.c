/*
 *  linux/arch/arm/mach-pxa/nevoevb3.c
 *
 *  Support for the Marvell Handheld Platform (aka EVB3)
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
#include <mach/mfp.h>
#include <mach/gpio.h>
#include <mach/mfp-pxa9xx.h>
#include <mach/pxa95xfb.h>
#include <mach/pxa95x_dvfm.h>
#include <mach/pmu.h>
#include <mach/pmic_id.h>
#include <mach/camera.h>
#include <mach/soc_vmeta.h>
#include <mach/pxa970_hdmitx.h>

#include <plat/sdhci.h>
#include <plat/i2c.h>
#include <plat/ssp.h>
#include <plat/pxa27x_keypad.h>
#include <plat/pxa_u2o.h>
#include <plat/pxa_uart.h>
#include <plat/pmem.h>

#include "devices.h"
#include "generic.h"
#include <media/soc_camera.h>
#include <linux/wakelock.h>
#include <mach/audio.h>

#define EVB3_NR_IRQS	(IRQ_BOARD_START + 40)

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
        .idpin          = PM860X_IDPIN_USE_GPADC2,
        .reg_base       = PXA935_U2O_REGBASE,
        .reg_end        = PXA935_U2O_REGBASE + USB_REG_RANGE,
};

static void disable_rf(void)
{
}

static struct pm860x_power_pdata power = {
	.disable_rf_fn	= disable_rf,
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

static struct regulator_consumer_supply regulator_supply[] = {
	/* For NEVO EVB Sanremo C1 Don't handle LDOs (HW bug) !!!! */
	[PM8607_ID_BUCK1]	= REGULATOR_SUPPLY("v_buck1", NULL),
	[PM8607_ID_BUCK3]	= REGULATOR_SUPPLY("v_buck3",NULL),
	[PM8607_ID_LDO1]	= REGULATOR_SUPPLY("v_ldo1", NULL),
	[PM8607_ID_LDO2]	= REGULATOR_SUPPLY("v_ldo2", NULL),
	[PM8607_ID_LDO3]	= REGULATOR_SUPPLY("v_ldo3", NULL),
	[PM8607_ID_LDO4]	= REGULATOR_SUPPLY("v_ldo4", NULL),
	[PM8607_ID_LDO5]	= REGULATOR_SUPPLY("v_ldo5", NULL),
	[PM8607_ID_LDO6]	= REGULATOR_SUPPLY("v_ldo6", NULL),
	[PM8607_ID_LDO7]	= REGULATOR_SUPPLY("v_ldo7", NULL),
	[PM8607_ID_LDO8]	= REGULATOR_SUPPLY("v_ldo8", NULL),
	[PM8607_ID_LDO9]	= REGULATOR_SUPPLY("v_ldo9", NULL),
	[PM8607_ID_LDO10]	= REGULATOR_SUPPLY("v_ldo10", NULL),
	[PM8607_ID_LDO12]	= REGULATOR_SUPPLY("v_ldo12", NULL),
	[PM8607_ID_LDO13]	= REGULATOR_SUPPLY("v_ldo13", NULL),
	[PM8607_ID_LDO14]	= REGULATOR_SUPPLY("v_ldo14", NULL),
};

#define REG_INIT(_name, _min, _max, _always, _boot)			\
{									\
	.constraints = {						\
		.name		= __stringify(_name),			\
		.min_uV		= _min,					\
		.max_uV		= _max,					\
		.always_on	= _always,				\
		.boot_on	= _boot,				\
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,		\
	},								\
	.num_consumer_supplies	= 1,					\
	.consumer_supplies	= &regulator_supply[PM8607_ID_##_name],	\
}

/* TODO: check the regulator data carefully */
static struct regulator_init_data regulator_data[] = {
	/* For NEVO EVB Sanremo C1 Don't handle LDOs (HW bug)!!!! */
	REG_INIT(BUCK1, 0, 1500000, 0, 0),
	REG_INIT(BUCK3, 0, 3000000, 0, 0),
	REG_INIT(LDO1, 1200000, 2800000, 1, 1),
	REG_INIT(LDO2, 1800000, 3300000, 1, 1),
	REG_INIT(LDO3, 1800000, 3300000, 1, 1),
	REG_INIT(LDO4, 1800000, 3300000, 0, 0),
	REG_INIT(LDO5, 2900000, 3300000, 1, 1),
	REG_INIT(LDO6, 1800000, 3300000, 1, 1),
	REG_INIT(LDO7, 1800000, 2900000, 1, 1),
	REG_INIT(LDO8, 1800000, 2900000, 0, 0),
	REG_INIT(LDO9, 1800000, 3300000, 1, 1),
	REG_INIT(LDO10, 1200000, 3300000, 1, 1),
	REG_INIT(LDO12, 1200000, 3300000, 1, 1),
	REG_INIT(LDO13, 1200000, 3300000, 1, 1),
	REG_INIT(LDO14, 1800000, 3300000, 0, 0),
};

static struct pm860x_rtc_pdata rtc = {
	.vrtc		= 1,
	.rtc_wakeup	= 0,
};

static struct pm860x_platform_data pm8607_info = {
	.touch		= &touch,
	.backlight	= &backlight[0],
	.led		= &led[0],
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
	.batt_det = 1,

	.num_regulators = ARRAY_SIZE(regulator_data),
};

static int vbus_detect(void *func, int enable)
{
	return 0;
}

#ifdef CONFIG_USB_GADGET_PXA_U2O
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
       .vendorID       = 0x0bb4,
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
static char *usb_functions0[] = {
#ifdef CONFIG_USB_ANDROID_RNDIS
       "rndis",
#endif
#ifdef CONFIG_USB_ANDROID_ADB
       "adb",
#endif
#ifdef CONFIG_USB_ANDROID_PXA955_ACM
       "acm",
#endif
#if defined(CONFIG_USB_ANDROID_MASS_STORAGE) || defined(CONFIG_USB_FILE_STORAGE)
       "usb_mass_storage",
#endif
#ifdef CONFIG_USB_ANDROID_PXA955_DIAG
	"diag",
#endif
#ifdef CONFIG_USB_ANDROID_MTP
		"mtp",
#endif
};

/* following usb_functionsX include functions for
 specific usb composite configurations */
static char *usb_functions1[] = {
#ifdef CONFIG_USB_ANDROID_ADB
	"adb",
#endif
#ifdef CONFIG_USB_ANDROID_PXA955_ACM
	"acm",
#endif
#if defined(CONFIG_USB_ANDROID_MASS_STORAGE) || defined(CONFIG_USB_FILE_STORAGE)
	"usb_mass_storage",
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
#ifdef CONFIG_USB_ANDROID_ADB
		"adb",
#endif
#ifdef CONFIG_USB_ANDROID_PXA955_ACM
		"acm",
#endif
#if defined(CONFIG_USB_ANDROID_MASS_STORAGE) || defined(CONFIG_USB_FILE_STORAGE)
		"usb_mass_storage",
#endif
#ifdef CONFIG_USB_ANDROID_PXA955_DIAG
		"diag",
#endif
};
static struct android_usb_product usb_products[] = {
	{
		.product_id = 0x0C03,
		.num_functions = ARRAY_SIZE(usb_functions0),
		.functions = usb_functions0,
	},
	{
		.product_id = 0x0C02,
		.num_functions = ARRAY_SIZE(usb_functions1),
		.functions = usb_functions1,
	},
	{
		.product_id = 0x811e,
		.num_functions = ARRAY_SIZE(usb_functions2),
		.functions = usb_functions2,
	},
	{
		.product_id = 0x811F,
		.num_functions = ARRAY_SIZE(usb_functions3),
		.functions = usb_functions3,
	},
	{
		.product_id = 0x8121,
		.num_functions = ARRAY_SIZE(usb_functions4),
		.functions = usb_functions4,
	},
	{
		.product_id = 0x8108,
		.num_functions = ARRAY_SIZE(usb_functions5),
		.functions = usb_functions5,
	},
	{
		.product_id = 0x8122,
		.num_functions = ARRAY_SIZE(usb_functions6),
		.functions = usb_functions6,
	},
	{
		.product_id = 0x0C02,
		.num_functions = ARRAY_SIZE(usb_functions7),
		.functions = usb_functions7,
	}
};

static struct android_usb_platform_data android_usb_pdata = {
       .vendor_id              = 0x0bb4,
       .product_id             = 0x0c03,
       .version                = 0x0100,
       .product_name           = "Android",
       .manufacturer_name      = "Marvell",
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

static struct pxa95x_freq_mach_info freq_mach_info = {
	.flags = PXA95x_USE_POWER_I2C,
};

static struct i2c_board_info i2c1_info[] = {
	{
		.type		= "88PM860x",
		.addr		= 0x34,
		.platform_data	= &pm8607_info,
		.irq		= IRQ_PMIC_INT,
	},
};
#ifndef CONFIG_I2C_PXA95x
static struct i2c_pxa_platform_data i2c1_pdata = {
	.mode = I2C_PXA_MODE_FIFO_INT,
	.freq = I2C_PXA_FREQ_HS_FAST,
	.master_code = 0x0e,
	.ilcr = 0x082CA356,/* ilcr: hs mode: 0x082CA356 normal: 0x082C45A3 */
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
	.use_pio        = 0,
	.flags		= PXA_I2C_HIGH_MODE | PXA_I2C_FAST_MODE | PXA_I2C_USING_FIFO_PIO_MODE,
	.master_code	= (0x08 | 0x06), /*8 -highest, 0xF -lowest arbitration*/
};

static struct i2c_pxa_platform_data i2c2_pdata = {
	.use_pio	= 0,
	.flags		= PXA_I2C_FAST_MODE | PXA_I2C_USING_FIFO_PIO_MODE,
};

#endif

#if defined(CONFIG_FB_PXA95x)
#if defined(CONFIG_HDMI_ADV7533)
int adv7533_hdmi_reset(void)
{
	int pin0 = mfp_to_gpio(MFP_PIN_GPIO139); /*CP*/
	int pin1 = mfp_to_gpio(MFP_PIN_GPIO18); /*PD*/
	printk(KERN_INFO "hdmi: adv7533_hdmi_reset+\n");

	if (gpio_request(pin0, "hdmi-reset0")) {
		printk(KERN_ERR "hdmi: gpio_request0: failed!\n");
	}
	if (gpio_request(pin1, "hdmi-reset1")) {
		printk(KERN_ERR "hdmi: gpio_request1: failed!\n");
	}

	gpio_direction_output(pin0, 1);
	msleep(10);
	gpio_direction_output(pin1, 0);
	msleep(10);
	gpio_direction_output(pin1, 1);
	msleep(10);
	gpio_direction_output(pin1, 0);

	gpio_free(pin0);
	gpio_free(pin1);

	printk(KERN_INFO "hdmi: adv7533_hdmi_reset-\n");
	return 0;
}

static struct fb_videomode adv_hdmi_video_modes[] = {
	[0] = {
		.pixclock       = 41701,
		.refresh        = 60,
		.xres           = 640,
		.yres           = 480,
		.hsync_len      = 96, /*HSW*/
		.left_margin    = 48, /*BLW*/
		.right_margin   = 16, /*ELW*/
		.vsync_len      = 2, /*VSW*/
		.upper_margin   = 33, /*BFW*/
		.lower_margin   = 10, /*EFW*/
		.sync           = 0,/*(FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT),*/
	},

};

static struct pxa95xfb_mach_info adv_hdmi_base_info __initdata = {
	.id                     = "HDMI-base",
	.modes                  = adv_hdmi_video_modes,
	.num_modes              = ARRAY_SIZE(adv_hdmi_video_modes),
	.pix_fmt_in             = PIX_FMTIN_RGB_16,
	.pix_fmt_out            = PIX_FMTOUT_24_RGB888,
	.panel_type             = LCD_Controller_Active,
	/*as hdmi-ovly use same win4 with lcd-ovly, they should not open at the same time*/
	.window					= 0,
	.mixer_id				= 1,
	.zorder 				= 1,
	.converter				= LCD_M2DSI1,
	.output					= OUTPUT_HDMI,
};

static struct pxa95xfb_mach_info adv_hdmi_ovly_info __initdata = {
	.id                     = "HDMI-Ovly",
	.modes                  = adv_hdmi_video_modes,
	.num_modes              = ARRAY_SIZE(adv_hdmi_video_modes),
	.pix_fmt_in             = PIX_FMTIN_YUV420,
	.pix_fmt_out            = PIX_FMTOUT_24_RGB888,
	.panel_type             = LCD_Controller_Active,
	/*as hdmi-ovly use same win4 with lcd-ovly, they should not open at the same time*/
	.window					= 4,
	.mixer_id				= 1,
	.zorder					= 0,
	.converter				= LCD_M2DSI1,
	.output					= OUTPUT_HDMI,
	.active					= 1,
	.invert_pixclock		= 1,
};
#endif

#if defined(CONFIG_PXA970_IHDMI)
static struct pxa95xfb_mach_info  __attribute__((unused)) adv_hdmi_base_info;
static struct pxa95xfb_mach_info  __attribute__((unused)) adv_hdmi_ovly_info;

static struct hdtx_plat_data hdtx_data = {
	.boot_en = 1,
	.format = 4,
};

static void __init init_hdtx(void)
{
	pxa970_set_ihdmi_info(&hdtx_data);
}


static struct fb_videomode ihdmi_video_modes[] = {
	[0] = {
		.pixclock       = 41701,
		.refresh        = 60,
		.xres           = 1280,
		.yres           = 720,
		.hsync_len      = 40, /*HSW*/
		.left_margin    = 220, /*BLW*/
		.right_margin   = 110, /*ELW*/
		.vsync_len      = 5, /*VSW*/
		.upper_margin   = 20, /*BFW*/
		.lower_margin   = 5, /*EFW*/
		/*(FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT),*/
		.sync           = 0,
	},

};

static struct pxa95xfb_mach_info ihdmi_base_info __initdata = {
	.id                     = "HDMI-base",
	.modes                  = ihdmi_video_modes,
	.num_modes              = ARRAY_SIZE(ihdmi_video_modes),
	.pix_fmt_in             = PIX_FMTIN_RGB_16,
	.pix_fmt_out            = PIX_FMTOUT_24_RGB888,
	.panel_type             = LCD_Controller_TV_HDMI,
	/*as hdmi-ovly use same win4 with lcd-ovly,
	 * they should not open at the same time*/
	.window			= 0,
	.mixer_id		= 2,
	.zorder			= 1,
	.converter		= LCD_M2HDMI,
	.output			= OUTPUT_HDMI,
};

static struct pxa95xfb_mach_info ihdmi_ovly_info __initdata = {
	.id                     = "HDMI-Ovly",
	.modes                  = ihdmi_video_modes,
	.num_modes              = ARRAY_SIZE(ihdmi_video_modes),
	.pix_fmt_in             = PIX_FMTIN_YUV420,
	.pix_fmt_out            = PIX_FMTOUT_24_RGB888,
	.panel_type             = LCD_Controller_TV_HDMI,
	/*as hdmi-ovly use same win4 with lcd-ovly,
	 * they should not open at the same time*/
	.window			= 4,
	.mixer_id		= 2,
	.zorder			= 0,
	.converter		= LCD_M2HDMI,
	.output			= OUTPUT_HDMI,
	.active			= 1,
	.invert_pixclock	= 1,
};
#endif
static void __init init_lcd(void)
{
	/* if both defined internal hdmi will cover */
#if defined(CONFIG_PXA970_IHDMI)
	init_hdtx();
	set_pxa95x_fb_info(&ihdmi_base_info);
	set_pxa95x_fb_ovly_info(&ihdmi_ovly_info, 0);

#elif defined(CONFIG_HDMI_ADV7533)
	set_pxa95x_fb_info(&adv_hdmi_base_info);
	set_pxa95x_fb_ovly_info(&adv_hdmi_ovly_info, 0);
#endif

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
		if (pxa95x_query_gwsr(105))
			src->bits.mmc1_cd = 1;
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

static int key_wakeup(pm_wakeup_src_t src, int enable)
{
	unsigned int ret = 0;
	if (enable) {
		if (src.bits.mkey) {
			static mfp_cfg_t key_edgeboth_cfg[] = {
				GPIO6_KP_MKIN_3 | MFP_LPM_EDGE_BOTH,
				GPIO8_KP_MKIN_4 | MFP_LPM_EDGE_BOTH,
				GPIO10_KP_MKIN_5 | MFP_LPM_EDGE_BOTH,
				GPIO12_KP_MKIN_6 | MFP_LPM_EDGE_BOTH,
				GPIO14_KP_MKIN_7 | MFP_LPM_EDGE_BOTH,
			};
			pxa3xx_mfp_config(ARRAY_AND_SIZE(key_edgeboth_cfg));
			ret |= PXA95x_PM_WE_KP;
		}
	} else {
		if (src.bits.mkey) {
			static mfp_cfg_t key_edgenone_cfg[] = {
				GPIO6_KP_MKIN_3 | MFP_LPM_EDGE_NONE,
				GPIO8_KP_MKIN_4 | MFP_LPM_EDGE_NONE,
				GPIO10_KP_MKIN_5 | MFP_LPM_EDGE_NONE,
				GPIO12_KP_MKIN_6 | MFP_LPM_EDGE_NONE,
				GPIO14_KP_MKIN_7 | MFP_LPM_EDGE_NONE,
			};
			pxa3xx_mfp_config(ARRAY_AND_SIZE(key_edgenone_cfg));
		}
	}
	return ret;
}

static int mmc1_wakeup(pm_wakeup_src_t src, int enable)
{
	unsigned int ret = 0;
	mfp_cfg_t mfp_c;
	if (enable) {
		if (src.bits.mmc1_cd) {
			mfp_c = GPIO105_GPIO | MFP_LPM_EDGE_BOTH;
			pxa3xx_mfp_config(&mfp_c, 1);
			ret |= PXA95x_PM_WE_GENERIC(13);
		}
	} else {
		mfp_c = GPIO105_GPIO | MFP_LPM_EDGE_NONE;
		pxa3xx_mfp_config(&mfp_c, 1);
	}

	return ret;
}

static int mmc3_wakeup(pm_wakeup_src_t src, int enable)
{
	unsigned int ret = 0;
	mfp_cfg_t mfp_c;
	if (enable) {
		mfp_c = GPIO80_MMC3_DAT1 | MFP_LPM_EDGE_BOTH;
		pxa3xx_mfp_config(&mfp_c, 1);
		ret |= PXA95x_PM_WE_MMC3;
	} else {
		mfp_c = GPIO80_MMC3_DAT1 | MFP_LPM_EDGE_NONE;
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
				m = MFP_CFG(GPIO131, AF0) | MFP_LPM_EDGE_FALL;
				pxa3xx_mfp_config(&m, 1);
				ret |= PXA95x_PM_WE_GENERIC(13);
			} else {
				m = GPIO131_UART1_RXD | MFP_LPM_EDGE_FALL;
				pxa3xx_mfp_config(&m, 1);
				ret |= PXA95x_PM_WE_GENERIC(9);
			}
		}
		if (src.bits.uart2) {
			if (is_uart_gpio()) {
				m = MFP_CFG(GPIO94, AF0) | MFP_LPM_EDGE_FALL;
				pxa3xx_mfp_config(&m, 1);
				ret |= PXA95x_PM_WE_GENERIC(13);
			} else {
				m = GPIO94_UART3_RXD | MFP_LPM_EDGE_FALL;
				pxa3xx_mfp_config(&m, 1);
				/* note: on pxa930, uart2 use this bit */
				ret |= PXA95x_PM_WE_GENERIC(2);
			}
		}
	} else {
		if (src.bits.uart1) {
			m = GPIO131_UART1_RXD | MFP_LPM_EDGE_NONE;
			pxa3xx_mfp_config(&m, 1);
		}
		if (src.bits.uart2) {
			m = GPIO94_UART3_RXD | MFP_LPM_EDGE_NONE;
			pxa3xx_mfp_config(&m, 1);
		}
	}
	return ret;
}

static int tsi_wakeup(pm_wakeup_src_t src, int enable)
{
	unsigned int ret = 0;
	mfp_cfg_t m;
	if (enable) {
		if (src.bits.tsi) {
			m = PMIC_INT_GPIO83 | MFP_LPM_FLOAT | MFP_LPM_EDGE_FALL;
			pxa3xx_mfp_config(&m, 1);
			ret |= PXA95x_PM_WE_GENERIC(3);
		}
	} else {
		if (src.bits.tsi) {
			m = PMIC_INT_GPIO83 | MFP_LPM_FLOAT | MFP_LPM_EDGE_NONE;
			pxa3xx_mfp_config(&m, 1);
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

static struct pxa95x_peripheral_wakeup_ops wakeup_ops = {
	.init   = init_wakeup,
	.query  = query_wakeup,
	.ext    = ext_wakeup,
	.key    = key_wakeup,
	.mmc1    = mmc1_wakeup,
	.mmc3    = mmc3_wakeup,
	.uart   = uart_wakeup,
	.tsi    = tsi_wakeup,
	.cmwdt  = comm_wdt_wakeup,
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
	GPIO63_ABU_RXD,
	GPIO64_ABU_TXD,
	GPIO65_GPIO,	/* no use for ABU/SSI, and configure GPIO70 to AF0 to save power when using ABU (~0.5mA) */
	GPIO66_ABU_FRM,
	GPIO67_ABU_CLK,
};

static mfp_cfg_t pxa95x_bssp2_mfp_cfg[] = {
	/* BSSP2 of MG1 */
	GPIO63_SSP2_RXD,
	GPIO64_SSP2_TXD,
	GPIO65_SSP2_SYSCLK,
	GPIO66_SSP2_FRM,
	GPIO67_SSP2_CLK,
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

static void abu_mfp_init(bool abu)
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

	status = gpio_get_value(gpio);
	status &= (1 << (gpio%32));
	status = !!status;

	if (invert)
		status = !status;

	return status;
}

static int ext_cd_status(struct sdhci_host *host)
{
       struct sdhci_pxa *pxa = sdhci_priv(host);

       return ext_cd_val(pxa->pdata->ext_cd_gpio,pxa->pdata->ext_cd_gpio_invert);
}

static irqreturn_t sdhci_pxa_cd_irq_thread(int irq, void *dev_id)
{
	struct platform_device *pdev = dev_id;
	struct sdhci_pxa_platdata *pdata = pdev->dev.platform_data;
	int status;

	msleep(600);
	status = ext_cd_val(pdata->ext_cd_gpio,pdata->ext_cd_gpio_invert);
	wake_lock_timeout(&cd_wake_lock, WAKE_LOCK_CD_TIMEOUT);
	sdhci_pxa_notify_change(pdev, status);

	return IRQ_HANDLED;
}

static void mci_setpower(struct device *dev, unsigned int vdd)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct sdhci_host *host = platform_get_drvdata(pdev);
	static unsigned long bitmap = 0;
	unsigned long oldbitmap = 0;
	int devbit = 1;

	pr_info("mci_setpower - mmc ldo on/off\n");

	oldbitmap = bitmap;

	if (pdev->id >= 0)
		devbit = (1 << pdev->id);

	if (vdd)
		bitmap |= devbit;
	else
		bitmap &= ~devbit;

	if (bitmap && !oldbitmap) {
		if (IS_ERR(host->vmmc)) {
			pr_info("mmc power on fail\n");
		} else {
			pr_info("mmc power on vmmc=0x%x\n", (int)host->vmmc);
			regulator_enable(host->vmmc);
		}
	}
	else if (!bitmap && oldbitmap) {
			if (IS_ERR(host->vmmc)) {
				pr_info("mmc power off fail\n");
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
	host->vmmc = regulator_get(&pdev->dev, "v_mmc");

	if (IS_ERR(host->vmmc)) {
		printk(KERN_ERR "Cannot allocate LDO for SD\n");
	} else {
		mmc_regulator_get_ocrmask(host->vmmc);
		pr_info("success allocate LDO for SD/MMC vmmc=0x%x\n",
		(int)host->vmmc);
	}
#else
	printk(KERN_ERR "Cannot find the power supply for SD\n");
#endif

	cd_irq = gpio_to_irq(pdata->ext_cd_gpio);
	ext_cd_gpio = pdata->ext_cd_gpio;

	/*
	 * setup GPIO for nevoevb3 MMC controller
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

void enable_oscc_tout_s0(void);
void disable_oscc_tout_s0(void);

/* specific 8787 power on/off setting */
static void wifi_set_power(unsigned int on)
{
	unsigned long wlan_pd_mfp = 0;
	int gpio_power_down = mfp_to_gpio(MFP_PIN_GPIO77);

	wlan_pd_mfp = pxa3xx_mfp_read(gpio_power_down);

	if (on) {
		/* set wlan_pd pin to output high in low power
			mode to ensure 8787 is not power off in low power mode*/
		wlan_pd_mfp |= 0x100;
		pxa3xx_mfp_write(gpio_power_down, wlan_pd_mfp & 0xffff);

		/* enable 32KHz TOUT */
		enable_oscc_tout_s0();
		}
	else {
		/*set wlan_pd pin to output low in low power
			mode to save power in low power mode */
		wlan_pd_mfp &= ~0x100;
		pxa3xx_mfp_write(gpio_power_down, wlan_pd_mfp & 0xffff);

		/* disable 32KHz TOUT */
		disable_oscc_tout_s0();
	}

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

static struct sdhci_pxa_platdata mci0_platform_data = {
	.quirks = SDHCI_QUIRK_NO_ENDATTR_IN_NOPDESC,
	.flags	= PXA_FLAG_CARD_PERMANENT | PXA_FLAG_ACITVE_IN_SUSPEND | PXA_FLAG_SD_8_BIT_CAPABLE_SLOT,
	.soc_set_ops = mmc_set_ops,
};

static struct sdhci_pxa_platdata mci1_platform_data = {
	.quirks = SDHCI_QUIRK_NO_ENDATTR_IN_NOPDESC,
	.ext_cd_gpio = mfp_to_gpio(MFP_PIN_GPIO105),
	.ext_cd_gpio_invert = 1,
	.ext_cd_init = ext_cd_init,
	.ext_cd_cleanup = ext_cd_cleanup,
	.lp_switch = mmc1_lp_switch,
	.soc_set_ops = mmc_set_ops,
};

static struct sdhci_pxa_platdata mci2_platform_data = {
	.flags  = PXA_FLAG_DISABLE_CLOCK_GATING |
						PXA_FLAG_SDIO_RESUME |
						PXA_FLAG_CARD_PERMANENT |
						PXA_FLAG_SDIO_IRQ_ALWAYS_ON |
						PXA_FLAG_DISABLE_PROBE_CDDET,
	.quirks = SDHCI_QUIRK_NO_ENDATTR_IN_NOPDESC,
	.get_sdio_wakelock      = get_sdio_wakelock,
};

static void __init init_mmc(void)
{
	/*add emmc only, need to add sdcard and sdio later*/
	pxa95x_set_mci_info(0, &mci0_platform_data);
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
#if defined(CONFIG_TOUCHSCREEN_VNC)
	&vnc_device,
#endif
};

#if defined(CONFIG_KEYBOARD_PXA27x) || defined(CONFIG_KEYBOARD_PXA27x_MODULE)
static unsigned int matrix_key_map[] = {
	/* KEY(row, col, key_code) */
	KEY(3, 4, KEY_HOME),
	KEY(3, 5, KEY_MENU),

	KEY(4, 4, KEY_SEND),
	KEY(4, 5, KEY_END),

	KEY(5, 4, KEY_OK),		/* ok button */
	KEY(5, 5, KEY_LEFT),

	KEY(6, 4, KEY_UP),
	KEY(6, 5, KEY_DOWN),

	KEY(7, 4, KEY_BACK),    /* volume rocker push */
	KEY(7, 5, KEY_RIGHT),
};

static struct pxa27x_keypad_platform_data keypad_info = {
	.matrix_key_rows	= 8,
	.matrix_key_cols	= 6,
	.matrix_key_map		= matrix_key_map,
	.matrix_key_map_size	= ARRAY_SIZE(matrix_key_map),
	.debounce_interval	= 30,
};
#endif /* CONFIG_KEYBOARD_PXA27x || CONFIG_KEYBOARD_PXA27x_MODULE */

#if defined(CONFIG_PXA95X_CAMERA)
static int cam_power_set(int flag)
{
	static struct clk *axi_clk = NULL;
	static struct clk *sci1_clk = NULL;
	static struct clk *sci2_clk = NULL;
	static struct clk *csi_tx_esc = NULL;

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
	/*
	 * RES, 0, LOW RESOLUTION, 1 HIGH RESOLUTION
	 * FLAG, 1: ON, 0: OFF
	 * GPIO93 LOW FOR HIGH
	 * GPIO14 LOW FOR LOW
	 */
	int high_pin = 0;
	int low_pin = mfp_to_gpio(MFP_PIN_GPIO14);
	struct regulator *vcamera;

	high_pin = mfp_to_gpio(MFP_PIN_GPIO15);
	cam_power_set(flag);

	if (gpio_request(high_pin, "CAM_EANBLE_HI_SENSOR")) {
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d\n", high_pin);
		return -EIO;
	}

	if (res) {
		if (flag) {
			gpio_direction_output(high_pin, 0);	/* enable */
			msleep(1);
			if (get_board_id() >= OBM_SAAR_B_MG2_A0_V13_BOARD) {
				vcamera = regulator_get(NULL, "v_ldo3");
				if (vcamera != NULL) {
					regulator_enable(vcamera);
					regulator_put(vcamera);
				} else
					printk(KERN_ERR "cam: failed to get regulator\n");
				}
		} else {
			if (get_board_id() >= OBM_SAAR_B_MG2_A0_V13_BOARD) {
				vcamera = regulator_get(NULL, "v_ldo3");
				if (vcamera != NULL) {
					regulator_disable(vcamera);
					regulator_put(vcamera);
				} else
					printk(KERN_ERR "cam: failed to get regulator\n");
			}
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
	return 0;
}

/* sensor init over */
static struct cam_platform_data cam_ops = {
	.power_set = cam_power_set,
};
static struct sensor_platform_data ov5642_sensor_data = {
	.id = SENSOR_HIGH,
	.power_set = sensor_power_set,
};

static struct sensor_platform_data  __attribute__((unused)) ov7690_sensor_data = {
	.id = SENSOR_LOW,
	.power_set = sensor_power_set,
};
#endif/*CONFIG_PXA95X_CAMERA*/

#if defined(CONFIG_SENSORS_LIS331DL)
static unsigned long lis33ldl_min_delay = 20;
#endif

static struct i2c_board_info i2c2_info[] = {
#if defined(CONFIG_PXA95X_CAMERA)

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

#if defined(CONFIG_SENSORS_LIS331DL)
	{
		.type		= "lis331dl",
		.addr		= 0x1c,
		.platform_data	= &lis33ldl_min_delay,
	},
#endif


#if defined(CONFIG_SENSORS_YAS529)
	{
		.type		= "yas529",
		.addr		= 0x2e,
	},
#endif
#if defined(CONFIG_HDMI_ADV7533)
	{
		.type		= "adv7533-packet",
		.addr		= 0x38,
	},
	{
		.type		= "adv7533-main",
		.addr		= 0x39,
	},
	{
		.type		= "adv7533-cec-dsi",
		.addr		= 0x2c,
	},
	{
		.type		= "adv7533-edid",
		.addr		= 0x3f,
	},
#endif
};

#if defined(CONFIG_VIDEO_PXA955)
static int camera0_power(struct device *dev, int flag)
{
	int gpio_pin = mfp_to_gpio(MFP_PIN_GPIO15);

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
	int gpio_pin = mfp_to_gpio(MFP_PIN_GPIO14);
	int gpio_pin_mipi = mfp_to_gpio(MFP_PIN_GPIO81);

	if (gpio_request(gpio_pin, "CAM_EANBLE_LOW_SENSOR")) {
		printk(KERN_ERR "cam: Request GPIO failed,"
				"gpio: %d\n", gpio_pin);
		return -EIO;
	}

	if (gpio_request(gpio_pin_mipi, "CAM_EANBLE_HI_SENSOR")) {
		printk(KERN_ERR "cam: Request GPIO failed,"
				"gpio: %d\n", gpio_pin_mipi);
		return -EIO;
	}

	if (flag) {
		gpio_direction_output(gpio_pin_mipi, 0);
		gpio_direction_output(gpio_pin, 0);
		msleep(1);
	} else {
		gpio_direction_output(gpio_pin_mipi, 1);
		gpio_direction_output(gpio_pin, 1);
	}

	gpio_free(gpio_pin_mipi);
	gpio_free(gpio_pin);
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

static struct pxa95x_csi_dev csidev[] = {
	{
		.irq_num	= 71,
		.reg_start	= 0x50020000,
	},
	/*TODO: if there is 2 csi controller, add its info here*/
	/*
	{
		.irq_num	= 71,
		.reg_base	= 0x50020000,
	},
	*/
};

/*
* TODO: combine csi controller and camera controller, now we only have one
* csi controller on pxa95x, and two camera controllers
*/
static struct pxa95x_cam_pdata cam_pdata[] = {
	{
		.mclk_mhz	= 26,/*mclk - 26Mhz*/
		.csidev		= &csidev[0],
	},
	/*TODO: if there is 2 sci controller, add its info here*/
	/*
	{
		.mclk_mhz	= 26,
		.csidev		= &csidev[1],
	},
	*/
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

#if defined(CONFIG_SOC_CAMERA_OV5642)
	platform_device_register(&camera[0]);
#endif
#if defined(CONFIG_SOC_CAMERA_OV7690)
	platform_device_register(&camera[1]);
#endif

	pxa95x_device_cam0.dev.platform_data = &cam_pdata[0];
	platform_device_register(&pxa95x_device_cam0);

	/* TODO: add sencond camera controller */
	/*pxa95x_device_cam1.dev.platform_data = &cam_pdata[1];*/
	/*platform_device_register(&pxa95x_device_cam1);*/
#endif
}

#ifdef CONFIG_PROC_FS
#define PROC_FILE	"driver/pmic_id"

static struct proc_dir_entry *proc_file;

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
			icotl_pmic_id = pm860x_codec_reg_read(PM8607_CHIP_ID);
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

static void create_proc_file(void)
{
	proc_file = create_proc_entry(PROC_FILE, 0644, NULL);
	if (proc_file) {
		proc_file->proc_fops = &proc_ops;
		printk(KERN_ALERT "PMIC proc file created!\n");
	} else
		printk(KERN_ALERT "PMIC proc file create failed!\n");
}
#endif

static void __init init(void)
{
#if defined(CONFIG_SND_PXA95X_SOC)
	set_abu_init_func(abu_mfp_init);
	set_ssp_init_func(ssp3_mfp_init);
#endif
	pxa_set_ffuart_info(&ffuart_info);
	pxa_set_stuart_info(&stuart_info);
	platform_device_add_data(&pxa95x_device_i2c1, &i2c1_pdata,
				 sizeof(i2c1_pdata));
	platform_device_add_data(&pxa95x_device_i2c2, &i2c2_pdata,
				 sizeof(i2c2_pdata));

	platform_add_devices(ARRAY_AND_SIZE(devices));
	i2c_register_board_info(0, ARRAY_AND_SIZE(i2c1_info));
	i2c_register_board_info(1, ARRAY_AND_SIZE(i2c2_info));

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

#ifdef CONFIG_MTD_ONENAND
	onenand_init(1);
#endif

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

}

MACHINE_START(NEVOEVB3, "PXA970 Handheld Platform (aka EVB3)")
	.phys_io        = 0x40000000,
	.io_pg_offst    = (io_p2v(0x40000000) >> 18) & 0xfffc,
	.map_io         = pxa_map_io,
	.nr_irqs		= EVB3_NR_IRQS,
	.init_irq       = pxa95x_init_irq,
	.timer          = &pxa_timer,
	.init_machine   = init,
MACHINE_END

