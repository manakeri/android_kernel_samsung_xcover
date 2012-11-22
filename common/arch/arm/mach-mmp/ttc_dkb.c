/*
 *  linux/arch/arm/mach-mmp/ttc_dkb.c
 *
 *  Support for the Marvell PXA910-based TTC_DKB Development Platform.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  publishhed by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/onenand.h>
#include <linux/mfd/88pm860x.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/i2c/pca9575.h>
#include <linux/i2c/pca953x.h>
#include <linux/i2c/elan_touch.h>
#include <linux/sd8x_rfkill.h>
#include <linux/proc_fs.h>

#include <linux/regulator/machine.h>
#include <linux/regulator/consumer.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/flash.h>
#include <mach/addr-map.h>
#include <mach/mfp-pxa910.h>
#include <mach/pxa910.h>
#include <mach/camera.h>
#include <mach/gpio.h>
#include <mach/regs-apbc.h>
#include <mach/pxa910_pm.h>
#include <mach/cputype.h>
#include <media/soc_camera.h>
#include <plat/pxa_u2o.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <linux/spi/cmmb.h>
#include <linux/usb/android_composite.h>

#include <plat/pmem.h>

#include <linux/nfc/pn544.h>
#include <linux/regulator/vpmic.h>

#include "common.h"
#include "onboard.h"

#define TTCDKB_NR_IRQS		(IRQ_BOARD_START + 24)

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

static int is_td_dkb;
static int __init td_dkb_setup(char *__unused)
{
	return is_td_dkb = 1;
}
__setup("td_dkb", td_dkb_setup);

static unsigned long ttc_dkb_pin_config[] __initdata = {
	/*UART2 GPS UART */
	GPIO43_UART2_RXD,
	GPIO44_UART2_TXD,

	/* GPS GPIO */
	GPIO45_GPIO45, /*share with TPO reset*/

	/* UART0 FFUART */
	GPIO47_UART0_RXD,
	GPIO48_UART0_TXD,

	/* UART1 BT_UART */
	GPIO29_UART1_CTS,
	GPIO30_UART1_RTS,
	GPIO31_UART1_TXD,
	GPIO32_UART1_RXD,

	/*GPIO*/
	GPIO15_GPIO15,
	GPIO16_GPIO16,
	GPIO18_GPIO18,
	GPIO19_GPIO19,

	/* NFC(pn544) irq */
	GPIO17_GPIO17,

	/* DFI */
	DF_IO0_ND_IO0,
	DF_IO1_ND_IO1,
	DF_IO2_ND_IO2,
	DF_IO3_ND_IO3,
	DF_IO4_ND_IO4,
	DF_IO5_ND_IO5,
	DF_IO6_ND_IO6,
	DF_IO7_ND_IO7,
	DF_IO8_ND_IO8,
	DF_IO9_ND_IO9,
	DF_IO10_ND_IO10,
	DF_IO11_ND_IO11,
	DF_IO12_ND_IO12,
	DF_IO13_ND_IO13,
	DF_IO14_ND_IO14,
	DF_IO15_ND_IO15,
	DF_nCS0_SM_nCS2_nCS0,
	DF_ALE_SM_WEn_ND_ALE,
	DF_CLE_SM_OEn_ND_CLE,
	DF_WEn_DF_WEn,
	DF_REn_DF_REn,
	DF_RDY0_DF_RDY0,

	/* I2C */
	GPIO53_CI2C_SCL,
	GPIO54_CI2C_SDA,

	/*sdh MMC0*/
	MMC1_DAT7_MMC1_DAT7,
	MMC1_DAT6_MMC1_DAT6,
	MMC1_DAT5_MMC1_DAT5,
	MMC1_DAT4_MMC1_DAT4,
	MMC1_DAT3_MMC1_DAT3,
	MMC1_DAT2_MMC1_DAT2,
	MMC1_DAT1_MMC1_DAT1,
	MMC1_DAT0_MMC1_DAT0,
	MMC1_CMD_MMC1_CMD,
	MMC1_CLK_MMC1_CLK,
	MMC1_CD_MMC1_CD | MFP_PULL_HIGH,
	MMC1_WP_MMC1_WP,

	MMC2_DAT3_GPIO_37,
	MMC2_DAT2_GPIO_38,
	MMC2_DAT1_GPIO_39,
	MMC2_DAT0_GPIO_40,
	MMC2_CMD_GPIO_41,
	MMC2_CLK_GPIO_42,

	/*wlan_bt*/
	WLAN_PD_GPIO_14,
	WLAN_RESET_GPIO_20,
	WIB_EN_GPIO_33,
	WLAN_BT_RESET_GPIO_34,
	WLAN_MAC_WAKEUP_GPIO_35,
	WLAN_LHC_GPIO_36,

	/* one wire */
	ONEWIRE_CLK_REQ,

	/* SSP1 (I2S) */
	GPIO24_SSP1_SDATA_IN,
	GPIO21_SSP1_BITCLK,
	GPIO22_SSP1_SYNC,
	GPIO23_SSP1_DATA_OUT,

	/*keypad*/
	GPIO00_KP_MKIN0,
	GPIO01_KP_MKOUT0,
	GPIO02_KP_MKIN1,
	GPIO03_KP_MKOUT1,
	GPIO04_KP_MKIN2,
	GPIO05_KP_MKOUT2,
	GPIO06_KP_MKIN3,
	GPIO07_KP_MKOUT3,
	GPIO08_KP_MKIN4,
	GPIO09_KP_MKOUT4,
	GPIO12_KP_MKIN6,

	/* AGPS GPIO */
	/* RDA8207 XOUT2_EN enable signal for AGPS clock */
	GPIO113_GPIO113 | MFP_PULL_HIGH,
};

static unsigned long emmc_pin_config[] __initdata = {
	MFP_CFG(DF_IO0, AF1),
	MFP_CFG(DF_IO1, AF1),
	MFP_CFG(DF_IO2, AF1),
	MFP_CFG(DF_IO3, AF1),
	MFP_CFG(DF_IO4, AF1),
	MFP_CFG(DF_IO5, AF1),
	MFP_CFG(DF_IO6, AF1),
	MFP_CFG(DF_IO7, AF1),
	MFP_CFG(DF_CLE_SM_OEn, AF1),
	MFP_CFG(SM_SCLK, AF1),
};

static unsigned long lcd_tpo_pin_config[] __initdata = {
	GPIO81_LCD_FCLK,
	GPIO82_LCD_LCLK,
	GPIO83_LCD_PCLK,
	GPIO84_LCD_DENA,
	GPIO85_LCD_DD0,
	GPIO86_LCD_DD1,
	GPIO87_LCD_DD2,
	GPIO88_LCD_DD3,
	GPIO89_LCD_DD4,
	GPIO90_LCD_DD5,
	GPIO91_LCD_DD6,
	GPIO92_LCD_DD7,
	GPIO93_LCD_DD8,
	GPIO94_LCD_DD9,
	GPIO95_LCD_DD10,
	GPIO96_LCD_DD11,
	GPIO97_LCD_DD12,
	GPIO98_LCD_DD13,
	GPIO100_LCD_DD14,
	GPIO101_LCD_DD15,
	GPIO102_LCD_DD16,
	GPIO103_LCD_DD17,
#if 1
	GPIO104_LCD_SPIDOUT,
	GPIO105_LCD_SPIDIN,
	GPIO106_GPIO106,
	GPIO107_LCD_CS1,
	GPIO108_LCD_DCLK,
#else
	GPIO104_GPIO104,  // Data out
//	GPIO105_LCD_SPIDIN,
	GPIO106_GPIO106,  // Reset
	GPIO107_GPIO107,  // CS
	GPIO108_GPIO108,  // SCLK
#endif
};

#if defined(CONFIG_PXA910_CAMERA) || defined(CONFIG_VIDEO_PXA910)
static unsigned long ccic_dvp_pin_config[] __initdata = {

	GPIO67_CCIC_IN7,
	GPIO68_CCIC_IN6,
	GPIO69_CCIC_IN5,
	GPIO70_CCIC_IN4,
	GPIO71_CCIC_IN3,
	GPIO72_CCIC_IN2,
	GPIO73_CCIC_IN1,
	GPIO74_CCIC_IN0,
	GPIO75_CAM_HSYNC,
	GPIO76_CAM_VSYNC,
	GPIO77_CAM_MCLK,
	GPIO78_CAM_PCLK,
};
#endif
static unsigned long ttc_rf_pin_config[] = {
	/* GSM */
	GPIO110_GPIO110,
	GPIO111_GPIO111,
	GPIO112_GPIO112,
	GPIO113_GPIO113,
	GPIO114_GPIO114,
	GPIO115_GPIO115,
	GPIO116_GPIO116,
	/*TDS-CDMA*/
	GPIO61_GPIO61,
	GPIO62_GPIO62,
	GPIO63_GPIO63,
	GPIO64_GPIO64,
	GPIO65_GPIO65,
	GPIO66_GPIO66,
};

static unsigned long tds_pin_config[] __initdata = {
	GPIO55_TDS_LNACTRL,
	GPIO57_TDS_TRXSW,
	GPIO58_TDS_RXREV,
	GPIO59_TDS_TXREV,
	GPIO60_GPIO60 | MFP_PULL_HIGH,
};

static unsigned int ttc_dkb_matrix_key_map[] = {
	KEY(0, 0, KEY_BACKSPACE),
	KEY(0, 1, KEY_END),
	KEY(0, 2, KEY_RIGHTCTRL),
	KEY(0, 3, KEY_0),
	KEY(0, 4, KEY_1),

	KEY(1, 0, KEY_MENU),
	KEY(1, 1, KEY_HOME),
	KEY(1, 2, KEY_SEND),
	KEY(1, 3, KEY_8),
	KEY(1, 4, KEY_9),

	KEY(2, 0, KEY_OK),
	KEY(2, 1, KEY_2),
	KEY(2, 2, KEY_3),
	KEY(2, 3, KEY_4),
	KEY(2, 4, KEY_5),

	KEY(3, 0, KEY_6),
	KEY(3, 1, KEY_VOLUMEUP),
	KEY(3, 2, KEY_7),
	KEY(3, 3, KEY_VOLUMEDOWN),
	KEY(3, 4, KEY_RECORD),

	KEY(4, 0, KEY_KPASTERISK),
	KEY(4, 1, KEY_KPDOT),
	KEY(4, 2, KEY_F2),
	KEY(4, 3, KEY_CAMERA),
	KEY(4, 4, KEY_CAMERA),

	KEY(6, 0, KEY_F1),
	KEY(6, 1, KEY_UP),
	KEY(6, 2, KEY_DOWN),
	KEY(6, 3, KEY_LEFT),
	KEY(6, 4, KEY_RIGHT),
};

static struct pxa27x_keypad_platform_data ttc_dkb_keypad_info __initdata = {
	.matrix_key_rows	= 7,
	.matrix_key_cols	= 5,
	.matrix_key_map		= ttc_dkb_matrix_key_map,
	.matrix_key_map_size	= ARRAY_SIZE(ttc_dkb_matrix_key_map),
	.debounce_interval	= 30,
};

static int emmc_boot;
int is_pxa921;
static int __init emmc_setup(char *__unused)
{
#if defined(CONFIG_MMC_SDHCI_PXA)
	emmc_boot = 1;
#endif
	is_pxa921 = 1;
	return 1;
}
__setup("emmc_boot", emmc_setup);

static int ttc_dkb_pm860x_fixup(struct pm860x_chip *chip,
			struct pm860x_platform_data *pdata)
{
	int data;
	/*
	Check testpage 0xD7:bit[0~1],if it is b00 or b11, that's to say
	2LSB of 0xD7 is maybe broken, will reset 0xD0~0xD7 to its default
	in test page by set 0xE1:b[7~6]=b00 for loading OTP;
	Besides, 0xE1:b[5~0] work as a counter to record times of D7 broken
	*/
	data = pm860x_page_reg_read(chip->client, 0xD7);
	data &= 0x3;
	if(data ==0x0 || data == 0x3)
	{
		data = pm860x_page_reg_read(chip->client, 0xE1);
		data &= 0x3F;
		if(data < 0x3F)
			data+=1;
		pm860x_page_reg_write(chip->client, 0xE1, data);
		data = pm860x_page_reg_read(chip->client, 0xE1);
		dev_dbg(chip->dev, "detect 0xD7 broken counter: %d", data);
	}
	/*confirm the interrupt mask*/
	pm860x_reg_write(chip->client, PM8607_INT_MASK_1, 0x00);
	pm860x_reg_write(chip->client, PM8607_INT_MASK_2, 0x00);
	pm860x_reg_write(chip->client, PM8607_INT_MASK_3, 0x00);

	pm860x_reg_write(chip->client, PM8607_INT_STATUS1, 0x3f);
	pm860x_reg_write(chip->client, PM8607_INT_STATUS1, 0xff);
	pm860x_reg_write(chip->client, PM8607_INT_STATUS1, 0xff);

	/* disable LDO5 turn on/off by LDO3_EN */
	pm860x_reg_write(chip->client, PM8607_MISC2,
	pm860x_reg_read(chip->client, PM8607_MISC2)|0x80);
	/* enable LDO5 for AVDD_USB */
	pm860x_reg_write(chip->client, PM8607_SUPPLIES_EN11,
	pm860x_reg_read(chip->client, PM8607_SUPPLIES_EN11)|0x80);

	/* init GPADC*/
	pm860x_reg_write(chip->client, PM8607_GPADC_MISC1, 0x0b);
	/* init power mode*/
	pm860x_reg_write(chip->client, PM8607_SLEEP_MODE1, 0xaa);
	pm860x_reg_write(chip->client, PM8607_SLEEP_MODE2, 0xaa);
	pm860x_reg_write(chip->client, PM8607_SLEEP_MODE3, 0xa2);
	/* set LDO14_SLP to be active in sleep mode */
	if (is_td_dkb)
		pm860x_reg_write(chip->client, PM8607_SLEEP_MODE4, 0x38);
	else
		pm860x_reg_write(chip->client, PM8607_SLEEP_MODE4, 0x3a);

	/* set vbuck1 0.9v in sleep*/
	pm860x_reg_write(chip->client, PM8607_SLEEP_BUCK1, 0x24);
	pm860x_reg_write(chip->client, PM8607_SLEEP_BUCK2, 0x24);
	/*RTC to use ext 32k clk*/
	pm860x_set_bits(chip->client, PM8607_RTC1, 1<<6, 1<<6);
	/*Enable RTC to use ext 32k clk*/
	pm860x_set_bits(chip->client, PM8607_RTC_MISC2, 0x7, 0x2);

	if (emmc_boot)
		/* on PXA921, turn on LDO13 for SD/MMC, 2.8V */
		pm860x_reg_write(chip->client, PM8607_VIBRA_SET, 0x0b);
	else
		/* shut down LDO13 for no use  */
		pm860x_reg_write(chip->client, PM8607_VIBRA_SET, 0x0c);

	/* audio save power */
	pm860x_reg_write(chip->client, PM8607_LP_CONFIG1, 0x40);
	/*to save pmic leakage*/
	pm860x_reg_write(chip->client, PM8607_LP_CONFIG3, 0x80);
	pm860x_reg_write(chip->client, PM8607_B0_MISC1, 0x80);
	pm860x_reg_write(chip->client, PM8607_MEAS_OFF_TIME1, 0x2);
	/* config sanremo Buck Controls Register to its default value
	to save 0.04mA in suspend. */
	pm860x_reg_write(chip->client, PM8607_BUCK_CONTROLS, 0x2b);
	pm860x_reg_write(chip->client, PM8607_LP_CONFIG2, 0x98);

	/* force LDO4 be active in sleep mode, required by CP */
	pm860x_set_bits(chip->client, PM8607_SLEEP_MODE2, 3 << 4, 3 << 4);
	return 0;
}

static struct pm860x_backlight_pdata ttc_dkb_backlight[] = {
	{
		.id	= PM8606_ID_BACKLIGHT,
		.iset	= PM8606_WLED_CURRENT(4),
		.flags	= PM8606_BACKLIGHT1,
	},
	{},
};
#if defined(CONFIG_I2C_PXA) || defined(CONFIG_I2C_PXA_MODULE)

static struct regulator_consumer_supply ttc_dkb_regulator_supply[] = {
	[PM8607_ID_BUCK1]	= REGULATOR_SUPPLY("v_buck1", NULL),
	[PM8607_ID_BUCK3]	= REGULATOR_SUPPLY("v_buck3", NULL),
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

#define DKB_REG_INIT(_name, _min, _max, _always, _boot)			\
{									\
	.constraints = {						\
		.name		= __stringify(_name),			\
		.min_uV		= _min,					\
		.max_uV		= _max,					\
		.always_on	= _always,				\
		.boot_on	= _boot,				\
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE		\
				| REGULATOR_CHANGE_STATUS, 		\
	},								\
	.num_consumer_supplies	= 1,					\
	.consumer_supplies	= &ttc_dkb_regulator_supply[PM8607_ID_##_name],	\
}

static struct regulator_init_data ttc_dkb_regulator_init_data[] = {
	DKB_REG_INIT(BUCK1, 1000000, 1500000, 1, 1),
	DKB_REG_INIT(BUCK3, 1000000, 3000000, 1, 1),
	DKB_REG_INIT(LDO1, 1200000, 2800000, 1, 1),
	DKB_REG_INIT(LDO2, 1800000, 3300000, 1, 1),
	DKB_REG_INIT(LDO3, 1800000, 3300000, 1, 1),
	DKB_REG_INIT(LDO4, 1800000, 3300000, 1, 1),
	DKB_REG_INIT(LDO5, 2900000, 3300000, 1, 1),
	DKB_REG_INIT(LDO6, 1800000, 3300000, 1, 1),
	DKB_REG_INIT(LDO7, 1800000, 2900000, 1, 1),
	DKB_REG_INIT(LDO8, 1800000, 2900000, 1, 1),
	DKB_REG_INIT(LDO9, 1800000, 3300000, 1, 1),
	DKB_REG_INIT(LDO10, 1200000, 3300000, 1, 1),
	DKB_REG_INIT(LDO12, 1200000, 3300000, 0, 1),
	DKB_REG_INIT(LDO13, 1200000, 3000000, 0, 1),
	DKB_REG_INIT(LDO14, 1800000, 3300000, 0, 1),
};

static struct pm860x_led_pdata ttc_dkb_led[] = {
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
	},
	{},
};

struct pm860x_vbus_pdata ttc_dkb_vbus = {
	.supply         = PM860X_GPIO2_SUPPLY_VBUS,
	.idpin          = PM860X_IDPIN_NO_USE,
	.reg_base       = PXA168_U2O_REGBASE,
	.reg_end        = PXA168_U2O_REGBASE + USB_REG_RANGE,
};
/*RF has leak current when battery calibration*/
void ttc_disable_rf(void){
	/* disable rf */
	mfp_config(ARRAY_AND_SIZE(ttc_rf_pin_config));
	mfp_write(MFP_PIN_GPIO110, 0xb0c0);
	mfp_write(MFP_PIN_GPIO111, 0xb0c0);
	mfp_write(MFP_PIN_GPIO112, 0xb0c0);
	mfp_write(MFP_PIN_GPIO113, 0xb0c0);
	mfp_write(MFP_PIN_GPIO114, 0xb881);
	mfp_write(MFP_PIN_GPIO115, 0xb881);
	mfp_write(MFP_PIN_GPIO116, 0xb881);

	mfp_write(MFP_PIN_GPIO60, 0xa880);
	mfp_write(MFP_PIN_GPIO61, 0x1085);
	mfp_write(MFP_PIN_GPIO62, 0x1085);
	mfp_write(MFP_PIN_GPIO63, 0x1085);
	mfp_write(MFP_PIN_GPIO64, 0x1085);
	mfp_write(MFP_PIN_GPIO65, 0x1085);
	mfp_write(MFP_PIN_GPIO66, 0x1085);
}
struct pm860x_power_pdata ttc_dkb_power = {
	.disable_rf_fn  = ttc_disable_rf,
};

struct pm860x_rtc_pdata ttc_dkb_rtc = {
	.vrtc		= 1,
	.rtc_wakeup	= 0,
};

static struct pm860x_platform_data ttc_dkb_pm8607_info = {
	.backlight	= &ttc_dkb_backlight[0],
	.led		= &ttc_dkb_led[0],
	.vbus		= &ttc_dkb_vbus,
	.power		= &ttc_dkb_power,
	.rtc		= &ttc_dkb_rtc,
	.regulator	= &ttc_dkb_regulator_init_data[0],
	.fixup		= ttc_dkb_pm860x_fixup,
	.companion_addr	= 0x11,
	.irq_mode	= 0,
	.irq_base	= IRQ_BOARD_START,
	.headset_flag	= 0x1,

	.i2c_port	= GI2C_PORT,
	.batt_det	= 0x0,

	.num_regulators	= ARRAY_SIZE(ttc_dkb_regulator_init_data),
};
#endif

#if defined(CONFIG_GPIO_PCA9575)
static struct pca9575_platform_data pca9575_data[] = {
	[0] = {
		.gpio_base      = GPIO_EXT1(0),
	},
};
#endif

static void mfp_gpio2_power_up(void)
{
	__raw_writel(FIRST_SECURITY_VALUE,APBC_PXA910_ASFAR);
	__raw_writel(SECOND_SECURITY_VALUE,APBC_PXA910_ASSAR);
	__raw_writel(AIB_POWER_TURNON,AIB_GPIO2_IO);
}

static void mfp_gpio2_power_down(void)
{
	__raw_writel(FIRST_SECURITY_VALUE,APBC_PXA910_ASFAR);
	__raw_writel(SECOND_SECURITY_VALUE,APBC_PXA910_ASSAR);
	__raw_writel(AIB_POWER_SHUTDOWN,AIB_GPIO2_IO);
}

static void mfp_gpio3_power_up(void)
{
	__raw_writel(FIRST_SECURITY_VALUE,APBC_PXA910_ASFAR);
	__raw_writel(SECOND_SECURITY_VALUE,APBC_PXA910_ASSAR);
	__raw_writel(AIB_POWER_TURNON,AIB_GPIO3_IO);
}

static void mfp_gpio3_power_down(void)
{
	__raw_writel(FIRST_SECURITY_VALUE,APBC_PXA910_ASFAR);
	__raw_writel(SECOND_SECURITY_VALUE,APBC_PXA910_ASSAR);
	__raw_writel(AIB_POWER_SHUTDOWN,AIB_GPIO3_IO);
}

#if defined(CONFIG_PXA910_CAMERA) || defined(CONFIG_VIDEO_PXA910)
static int cam_ldo12_1p2v_enable(int on)
{
	static struct regulator *r_vcam;
	static int f_enabled;
	if (on && (!f_enabled)) {
		r_vcam = regulator_get(NULL, "v_ldo12");
		if (IS_ERR(r_vcam)) {
			r_vcam = NULL;
			return EIO;
		} else {
			regulator_set_voltage(r_vcam, 1200000, 1200000);
			regulator_enable(r_vcam);
			f_enabled = 1;
		}
	}

	if (f_enabled && (!on)) {
		if (r_vcam) {
			regulator_disable(r_vcam);
			regulator_put(r_vcam);
			f_enabled = 0;
			r_vcam = NULL;
		}
	}
	return 0;
}

/* soc  camera */
static int camera_sensor_power(struct device *dev, int on)
{
	unsigned int cam_pwr;
	unsigned int cam_reset;
	unsigned int cam_afen;
	int sensor = 1; // main sensor
	/* actually, no small power down pin needed */
	cam_pwr = sensor ? GPIO_EXT0(6):0;
	cam_reset = sensor ? GPIO_EXT0(4):GPIO_EXT0(14);
	cam_afen = mfp_to_gpio(MFP_PIN_GPIO49);

	if(cam_pwr)
	if (gpio_request(cam_pwr, "CAM_PWR")) {
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d \n", cam_pwr);
		return -EIO;
	}

	if (gpio_request(cam_reset, "CAM_RESET")) {
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d \n", cam_reset);
		return -EIO;
	}
	if (gpio_request(cam_afen, "CAM_RESET")) {
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d \n", cam_afen);
		return -EIO;
	}

	if(on){
		gpio_direction_output(cam_afen, 1);
		mdelay(1);
		if(cam_pwr)
			gpio_direction_output(cam_pwr, 0);
		mdelay(1);
		gpio_direction_output(cam_reset, 0);
		mdelay(1);
		gpio_direction_output(cam_reset, 1);
		mdelay(1);
		/* set MIPI_AVDD1P2V for MIPI IO */
		cam_ldo12_1p2v_enable(1);
		mdelay(1);
	}else{
		gpio_direction_output(cam_reset, 0);
		mdelay(1);
		gpio_direction_output(cam_reset, 1);
		if(cam_pwr)
			gpio_direction_output(cam_pwr, 1);
		gpio_direction_output(cam_afen, 0);
		cam_ldo12_1p2v_enable(0);
	}
	if(cam_pwr)
		gpio_free(cam_pwr);
	gpio_free(cam_reset);
	gpio_free(cam_afen);
	return 0;
}

static struct i2c_board_info dkb_i2c_camera[] = {
	{
		I2C_BOARD_INFO("ov5642", 0x3c),
	},
};

static struct soc_camera_link iclink_ov5642_dvp = {
        .bus_id         = 0,            /* Must match with the camera ID */
        .power          = camera_sensor_power,
        .board_info     = &dkb_i2c_camera[0],
        .i2c_adapter_id = 0,
     /* .flag = SOCAM_MIPI; */
		.module_name    = "ov5642",
		.priv	= "pxa910-dvp",
};

static struct platform_device dkb_ov5642_dvp = {
        .name   = "soc-camera-pdrv",
        .id     = 0,
        .dev    = {
                .platform_data = &iclink_ov5642_dvp,
        },
};
/* sensor init */
static int sensor_power_onoff(int on, int sensor)
{
	unsigned int cam_pwr;
	unsigned int cam_reset;
	unsigned int cam_afen;

	/* actually, no small power down pin needed */
	cam_pwr = sensor ? GPIO_EXT0(6) : 0;
	cam_reset = sensor ? GPIO_EXT0(4) : GPIO_EXT0(14);
	cam_afen = mfp_to_gpio(MFP_PIN_GPIO49);

	if (cam_pwr)
		if (gpio_request(cam_pwr, "CAM_PWR")) {
			printk(KERN_ERR "Request GPIO failed,\
					gpio: %d\n", cam_pwr);
			return -EIO;
		}

	if (gpio_request(cam_reset, "CAM_RESET")) {
		printk(KERN_ERR "Request GPIO failed, gpio: %d\n", cam_reset);
		return -EIO;
	}
	if (gpio_request(cam_afen, "CAM_RESET")) {
		printk(KERN_ERR "Request GPIO failed, gpio: %d\n", cam_afen);
		return -EIO;
	}

	if (on) {
		gpio_direction_output(cam_afen, 1);
		msleep(1);
		if (cam_pwr)
			gpio_direction_output(cam_pwr, 0);
		msleep(1);
		gpio_direction_output(cam_reset, 0);
		msleep(1);
		gpio_direction_output(cam_reset, 1);
		msleep(1);
		/* set MIPI_AVDD1P2V for MIPI IO */
		cam_ldo12_1p2v_enable(1);
		msleep(1);
	} else {
		gpio_direction_output(cam_reset, 0);
		msleep(1);
		gpio_direction_output(cam_reset, 1);
		if (cam_pwr)
			gpio_direction_output(cam_pwr, 1);
		gpio_direction_output(cam_afen, 0);
		cam_ldo12_1p2v_enable(0);
	}

	if (cam_pwr)
		gpio_free(cam_pwr);
	gpio_free(cam_reset);
	gpio_free(cam_afen);
	return 0;
}

static struct sensor_platform_data ov7670_sensor_data = {
	.id = SENSOR_LOW,
	.power_on = sensor_power_onoff,
};

static struct sensor_platform_data ov3640_sensor_data = {
	.id = SENSOR_HIGH,
	.power_on = sensor_power_onoff,
};

/* sensor init over */
#endif

#if defined(CONFIG_GPIO_PCA953X)
static struct pca953x_platform_data max7312_data[] = {
	[0] = {
		.gpio_base      = GPIO_EXT0(0),
	},
};
#endif

#if defined(CONFIG_TOUCHSCREEN_ELAN)
static int touch_io_power_onoff(int on)
{
	unsigned int tp_logic_en;
	tp_logic_en = GPIO_EXT0(MFP_PIN_GPIO15);

	if (gpio_request(tp_logic_en, "TP_LOGIC_EN")) {
		printk(KERN_ERR "Request GPIO failed,"
		       "gpio: %d \n", tp_logic_en);
		return -EIO;
	}

	if (on) {
		gpio_direction_output(tp_logic_en, 1);
	} else {
		gpio_direction_output(tp_logic_en, 0);
	}

	gpio_free(tp_logic_en);
	return 0;
}
static struct elan_touch_platform_data elan_touch_data ={
	.power = touch_io_power_onoff,
};
#endif

/* workaround for reset i2c bus by GPIO53 -SCL, GPIO54 -SDA */
static void i2c_pxa_bus_reset(void)
{
	unsigned long i2c_mfps[] = {
		GPIO53_GPIO53,		/* SCL */
		GPIO54_GPIO54,		/* SDA */
	};
	unsigned long mfp_pin[ARRAY_SIZE(i2c_mfps)];
	int ccnt;

	if (gpio_request(MFP_PIN_GPIO53, "SCL")) {
		pr_err("Failed to request GPIO for SCL pin!\n");
		goto out;
	}
	if (gpio_request(MFP_PIN_GPIO54, "SDA")) {
		pr_err("Failed to request GPIO for SDA pin!\n");
		goto out_sda;
	}
	pr_info("\t<<<i2c bus reseting>>>\n");
	/* set mfp pins to gpio */
	mfp_pin[0] = mfp_read(MFP_PIN_GPIO53);
	mfp_pin[1] = mfp_read(MFP_PIN_GPIO54);
	mfp_config(ARRAY_AND_SIZE(i2c_mfps));

	gpio_direction_input(MFP_PIN_GPIO54);
	for (ccnt = 20; ccnt; ccnt--) {
		gpio_direction_output(MFP_PIN_GPIO53, ccnt & 0x01);
		udelay(4);
	}
	gpio_direction_output(MFP_PIN_GPIO53, 0);
	udelay(4);
	gpio_direction_output(MFP_PIN_GPIO54, 0);
	udelay(4);
	/* stop signal */
	gpio_direction_output(MFP_PIN_GPIO53, 1);
	udelay(4);
	gpio_direction_output(MFP_PIN_GPIO54, 1);
	udelay(4);

	mfp_write(MFP_PIN_GPIO53, mfp_pin[0]);
	mfp_write(MFP_PIN_GPIO54, mfp_pin[1]);
	gpio_free(MFP_PIN_GPIO54);
out_sda:
	gpio_free(MFP_PIN_GPIO53);
out:
	return;
}

/* The following structure is for VPMIC regulator */
#if defined(CONFIG_REGULATOR_VPMIC)
static struct regulator_consumer_supply vpmic_regulator_supply[] = {
	[VPMIC_ID_Vdd_IO]	= REGULATOR_SUPPLY("Vdd_IO", NULL),
	[VPMIC_ID_VBat]		= REGULATOR_SUPPLY("VBat", NULL),
	[VPMIC_ID_VSim]		= REGULATOR_SUPPLY("VSim", NULL),
};

#define REGULATOR_INIT(_name, _min, _max, _always, _boot)		\
{									\
	.constraints = {						\
		.name		= __stringify(_name),			\
		.min_uV		= _min,					\
		.max_uV		= _max,					\
		.always_on	= _always,				\
		.boot_on	= _boot,				\
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE		\
				| REGULATOR_CHANGE_STATUS,		\
	},								\
	.num_consumer_supplies	= 1,					\
	.consumer_supplies	= &vpmic_regulator_supply[VPMIC_ID_##_name],\
}

static struct regulator_init_data vpmic_regulator_init_data[] = {
	REGULATOR_INIT(Vdd_IO, 1000000, 1500000, 1, 1),
	REGULATOR_INIT(VBat, 1000000, 3000000, 1, 1),
	REGULATOR_INIT(VSim, 1200000, 2800000, 1, 1),
};

static struct platform_device vpmic_regulator_vdd = {
	.name   = "vpmic-regulator",
	.id     = 0,
	.dev    = {
		.platform_data = &vpmic_regulator_init_data[VPMIC_ID_Vdd_IO],
	},
};

static struct platform_device vpmic_regulator_vbat = {
	.name   = "vpmic-regulator",
	.id     = 1,
	.dev    = {
		.platform_data = &vpmic_regulator_init_data[VPMIC_ID_VBat],
	},
};

static struct platform_device vpmic_regulator_vsim = {
	.name   = "vpmic-regulator",
	.id     = 2,
	.dev    = {
		.platform_data = &vpmic_regulator_init_data[VPMIC_ID_VSim],
	},
};

#endif

/* The following structure is for pn544 I2C device */
#if defined(CONFIG_PN544_NFC)
static int pn544_request_resources(struct i2c_client *client)
{
	int ret = 0;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s : need I2C_FUNC_I2C\n", __func__);
		return  -ENODEV;
	}

	ret = gpio_request(MFP_PIN_GPIO17, "gpio used as irq for pn544");
	if (ret)
		return  -ENODEV;

	gpio_direction_input(MFP_PIN_GPIO17);

	return 0;
}

static void pn544_free_resources(void)
{
	gpio_free(MFP_PIN_GPIO17);
}

static int pn544_test(void)
{
	return MFP_PIN_GPIO17;
}

static struct pn544_nfc_platform_data pn544_data = {
	.request_resources	= pn544_request_resources,
	.free_resources		= pn544_free_resources,
	.test			= pn544_test,
};
#endif


static unsigned long lis33ldl_min_delay = 50;
static struct i2c_board_info ttc_dkb_i2c_info[] = {
	{
		.type		= "88PM860x",
		.addr		= 0x34,
		.platform_data	= &ttc_dkb_pm8607_info,
		.irq		= IRQ_PXA910_PMIC_INT,
	},
#if defined(CONFIG_GPIO_PCA9575)
	{
		.type           = "pca9575",
		.addr           = 0x20,
		.irq            = IRQ_GPIO(19),
		.platform_data  = &pca9575_data,
	},
#endif
#if defined(CONFIG_GPIO_PCA953X)
	{
		.type           = "max7312",
		.addr           = 0x23,
		.irq            = IRQ_GPIO(80),
		.platform_data  = &max7312_data,
	},
#endif
#if defined(CONFIG_TOUCHSCREEN_ELAN)
	{
		.type			= "elan_touch",
		.addr			=  0x8,
		.irq			= gpio_to_irq(45),
		.platform_data	= &elan_touch_data,
	},
#endif
#if defined(CONFIG_TOUCHSCREEN_TPO)
	{
		.type		= "tpo_touch",
		.addr		=  0x18,
		.irq		= gpio_to_irq(45),
	},
#endif
#if defined(CONFIG_SENSORS_LIS331DL)
	{
		.type		= "lis331dl",
		.addr		=  0x1c,
		.platform_data	= &lis33ldl_min_delay,
	},
#endif
#if defined(CONFIG_PXA910_CAMERA)
	{
		.type           = "ov7670",
		.addr           = 0x21,
		.platform_data  = &ov7670_sensor_data,
	},
#if defined(CONFIG_VIDEO_OV3640)
	{
		.type		= "ov3640",
		.addr           = 0x3C,
		.platform_data  = &ov3640_sensor_data,
	},
#endif
#if defined(CONFIG_VIDEO_PXA910_OV5642)
	{
		.type		= "ov5642",
		.addr           = 0x3d,
		.platform_data  = &ov3640_sensor_data,
	},
#endif
#endif
#if defined(CONFIG_PN544_NFC)
	{
		.type		= "pn544",
		.addr           = 0x28,
		.irq		= gpio_to_irq(MFP_PIN_GPIO17),
		.platform_data  = &pn544_data,
	},
#endif
};

/*For pxa921 board*/
static struct i2c_board_info ttc_dkb_pwr_i2c_info[] = {
#if defined(CONFIG_GPIO_PCA9575)
	{
		.type           = "pca9575",
		.addr           = 0x20,
		.irq            = IRQ_GPIO(19),
		.platform_data  = &pca9575_data,
	},
#endif
#if defined(CONFIG_VIDEO_PXA910_OV5642)
	{
		.type		= "ov5642",
		.addr           = 0x3c,
		.platform_data  = &ov3640_sensor_data,
	},
#endif
#if defined(CONFIG_SENSORS_LIS331DL)
	{
		.type		= "lis331dl",
		.addr		=  0x1c,
		.platform_data	= &lis33ldl_min_delay,
	},
#endif
};

static struct i2c_pxa_platform_data ttc_dkb_i2c_pdata = {
	.mode = I2C_PXA_MODE_INT,
	.freq = I2C_PXA_FREQ_FAST,
	.ilcr = 0x082C447E,/*ilcr:fs mode b17~9=0x22,about 380K,
			     standard mode b8~0=0x7E,100K*/
	.iwcr = 0x0000142A,/*iwcr:b5~0=b01010 recommended value*/
	.hardware_lock = ripc_get,
	.hardware_unlock = ripc_release,
	.i2c_bus_reset = i2c_pxa_bus_reset,
};
static struct i2c_pxa_platform_data ttc_dkb_pwr_i2c_pdata = {
	.mode = I2C_PXA_MODE_INT,
	.freq = I2C_PXA_FREQ_FAST,
	.ilcr = 0x082C447E,/*ilcr:fs mode b17~9=0x22,about 380K,
			     standard mode b8~0=0x7E,100K*/
	.iwcr = 0x0000142A,/*iwcr:b5~0=b01010 recommended value*/
};


static struct mtd_partition ttc_dkb_onenand_partitions[] = {
	{
		.name		= "bootloader",
		.offset		= 0,
		.size		= SZ_1M,
		.mask_flags	= MTD_WRITEABLE,
	}, {
		.name		= "reserved",
		.offset		= MTDPART_OFS_APPEND,
		.size		= SZ_128K,
		.mask_flags	= MTD_WRITEABLE,
	}, {
		.name		= "reserved",
		.offset		= MTDPART_OFS_APPEND,
		.size		= SZ_8M,
		.mask_flags	= MTD_WRITEABLE,
	}, {
		.name		= "kernel",
		.offset		= MTDPART_OFS_APPEND,
		.size		= (SZ_2M + SZ_1M),
		.mask_flags	= 0,
	}, {
		.name		= "filesystem",
		.offset		= MTDPART_OFS_APPEND,
		.size		= SZ_48M,
		.mask_flags	= 0,
	}
};

static struct onenand_platform_data ttc_dkb_onenand_info = {
	.parts		= ttc_dkb_onenand_partitions,
	.nr_parts	= ARRAY_SIZE(ttc_dkb_onenand_partitions),
};

static struct resource ttc_dkb_resource_onenand[] = {
	[0] = {
		.start	= SMC_CS0_PHYS_BASE,
		.end	= SMC_CS0_PHYS_BASE + SZ_1M,
		.flags	= IORESOURCE_MEM,
	},
};

static struct platform_device ttc_dkb_device_onenand = {
	.name		= "onenand-flash",
	.id		= -1,
	.resource	= ttc_dkb_resource_onenand,
	.num_resources	= ARRAY_SIZE(ttc_dkb_resource_onenand),
	.dev		= {
		.platform_data	= &ttc_dkb_onenand_info,
	},
};

static struct pxa3xx_nand_platform_data ttc_dkb_nand_info = {
	.controller_attrs = PXA3XX_ARBI_EN | PXA3XX_NAKED_CMD_EN | PXA3XX_DMA_EN
				| PXA3XX_ADV_TIME_TUNING | PXA3XX_POLLING_MODE,
	.parts[0]	= ttc_dkb_onenand_partitions,
	.nr_parts[0]	= ARRAY_SIZE(ttc_dkb_onenand_partitions),
};

static int ttc_dkb_vbus_detect(void *func, int enable) {
	return 0;
}

static struct pxa_usb_plat_info pxa910_u2o_info = {
	.phy_init	= pxa_usb_phy_init,
	.is_otg		= 1,
	.in_single	= 1,
	.vbus_detect	= ttc_dkb_vbus_detect,
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
#ifdef CONFIG_USB_ANDROID_PXA910_MODEM
	"acm",
#endif
#ifdef CONFIG_USB_ANDROID_PXA910_DIAG
	"diag",
#endif
#if defined(CONFIG_USB_ANDROID_MASS_STORAGE) || defined(CONFIG_USB_FILE_STORAGE)
	"usb_mass_storage",
#endif
#ifdef CONFIG_USB_ANDROID_ADB
	"adb",
#endif
};

/* following usb_functionsX include functions for
 specific usb composite configurations */
static char *usb_functions_modem_diag_ums_adb[] = {
#ifdef CONFIG_USB_ANDROID_PXA910_MODEM
	"acm",
#endif
#ifdef CONFIG_USB_ANDROID_PXA910_DIAG
	"diag",
#endif
#if defined(CONFIG_USB_ANDROID_MASS_STORAGE) || defined(CONFIG_USB_FILE_STORAGE)
	"usb_mass_storage",
#endif
#ifdef CONFIG_USB_ANDROID_ADB
	"adb",
#endif
};

static char *usb_functions_modem_diag_ums[] = {
#ifdef CONFIG_USB_ANDROID_PXA910_MODEM
	"acm",
#endif
#ifdef CONFIG_USB_ANDROID_PXA910_DIAG
	"diag",
#endif
#if defined(CONFIG_USB_ANDROID_MASS_STORAGE) || defined(CONFIG_USB_FILE_STORAGE)
	"usb_mass_storage",
#endif
};

static char *usb_functions_rndis_modem_diag[] = {
#ifdef CONFIG_USB_ANDROID_RNDIS
	"rndis",
#endif
#ifdef CONFIG_USB_ANDROID_PXA910_MODEM
	"acm",
#endif
#ifdef CONFIG_USB_ANDROID_PXA910_DIAG
	"diag",
#endif
};

static char *usb_functions_rndis_modem_diag_adb[] = {
#ifdef CONFIG_USB_ANDROID_RNDIS
	"rndis",
#endif
#ifdef CONFIG_USB_ANDROID_PXA910_MODEM
	"acm",
#endif
#ifdef CONFIG_USB_ANDROID_PXA910_DIAG
	"diag",
#endif
#ifdef CONFIG_USB_ANDROID_ADB
	"adb",
#endif
};

static char *usb_functions_rndis[] = {
#ifdef CONFIG_USB_ANDROID_RNDIS
	"rndis",
#endif
};

static char *usb_functions_ums[] = {
#if defined(CONFIG_USB_ANDROID_MASS_STORAGE) || defined(CONFIG_USB_FILE_STORAGE)
	"usb_mass_storage",
#endif
};

static char *usb_functions_modem_diag[] = {
#ifdef CONFIG_USB_ANDROID_PXA910_MODEM
	"acm",
#endif
#ifdef CONFIG_USB_ANDROID_PXA910_DIAG
	"diag",
#endif
};

static char *usb_functions_ums_adb[] = {
#if defined(CONFIG_USB_ANDROID_MASS_STORAGE) || defined(CONFIG_USB_FILE_STORAGE)
	"usb_mass_storage",
#endif
#ifdef CONFIG_USB_ANDROID_ADB
	"adb",
#endif
};

static char *usb_functions_rndis_adb[] = {
#ifdef CONFIG_USB_ANDROID_RNDIS
	"rndis",
#endif
#ifdef CONFIG_USB_ANDROID_ADB
	"adb",
#endif
};

static char *usb_functions_modem_ums_adb[] = {
#ifdef CONFIG_USB_ANDROID_PXA910_MODEM
	"acm",
#endif
#if defined(CONFIG_USB_ANDROID_MASS_STORAGE) || defined(CONFIG_USB_FILE_STORAGE)
	"usb_mass_storage",
#endif
#ifdef CONFIG_USB_ANDROID_ADB
	"adb",
#endif
};

static char *usb_functions_modem_ums[] = {
#ifdef CONFIG_USB_ANDROID_PXA910_MODEM
	"acm",
#endif
#if defined(CONFIG_USB_ANDROID_MASS_STORAGE) || defined(CONFIG_USB_FILE_STORAGE)
	"usb_mass_storage",
#endif
};

static char *usb_functions_diag[] = {
#ifdef CONFIG_USB_ANDROID_PXA910_DIAG
	"diag",
#endif
};

static struct android_usb_product usb_products[] = {
	{
		.product_id = MMP_ALL_PRODUCT_ID,
		.num_functions = ARRAY_SIZE(usb_functions_all),
		.functions = usb_functions_all,
	},
	{
		.product_id = MMP_MODEM_DIAG_UMS_PRODUCT_ID,
		.num_functions = ARRAY_SIZE(usb_functions_modem_diag_ums),
		.functions = usb_functions_modem_diag_ums,
	},
	{
		.product_id = MMP_MODEM_DIAG_UMS_ADB_PRODUCT_ID,
		.num_functions = ARRAY_SIZE(usb_functions_modem_diag_ums_adb),
		.functions = usb_functions_modem_diag_ums_adb,
	},
	{
		.product_id = MMP_RNDIS_MODEM_DIAG_PRODUCT_ID,
		.num_functions = ARRAY_SIZE(usb_functions_rndis_modem_diag),
		.functions = usb_functions_rndis_modem_diag,
	},
	{
		.product_id = MMP_RNDIS_MODEM_DIAG_ADB_PRODUCT_ID,
		.num_functions = ARRAY_SIZE(usb_functions_rndis_modem_diag_adb),
		.functions = usb_functions_rndis_modem_diag_adb,
	},
	{
		.product_id = MMP_RNDIS_PRODUCT_ID,
		.num_functions = ARRAY_SIZE(usb_functions_rndis),
		.functions = usb_functions_rndis,
	},
	{
		.product_id = MMP_UMS_PRODUCT_ID,
		.num_functions = ARRAY_SIZE(usb_functions_ums),
		.functions = usb_functions_ums,
	},
	{
		.product_id = MMP_MODEM_DIAG_PRODUCT_ID,
		.num_functions = ARRAY_SIZE(usb_functions_modem_diag),
		.functions = usb_functions_modem_diag,
	},
	{
		.product_id = MMP_UMS_ADB_PRODUCT_ID,
		.num_functions = ARRAY_SIZE(usb_functions_ums_adb),
		.functions = usb_functions_ums_adb,
	},
	{
		.product_id = MMP_RNDIS_ADB_PRODUCT_ID,
		.num_functions = ARRAY_SIZE(usb_functions_rndis_adb),
		.functions = usb_functions_rndis_adb,
	},
	{
		.product_id = MMP_MODEM_UMS_ADB_PRODUCT_ID,
		.num_functions = ARRAY_SIZE(usb_functions_modem_ums_adb),
		.functions = usb_functions_modem_ums_adb,
	},
	{
		.product_id = MMP_MODEM_UMS_PRODUCT_ID,
		.num_functions = ARRAY_SIZE(usb_functions_modem_ums),
		.functions = usb_functions_modem_ums,
	},
	{
		.product_id = MMP_DIAG_PRODUCT_ID,
		.num_functions = ARRAY_SIZE(usb_functions_diag),
		.functions = usb_functions_diag,
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

void __init pxa910_android_add_usb_devices(void)
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

static struct platform_device *ttc_dkb_devices[] = {
	&pxa910_device_twsi0,
	&ttc_dkb_device_onenand,
#if defined(CONFIG_VIDEO_PXA910)
	&dkb_ov5642_dvp,
#endif

#if defined(CONFIG_REGULATOR_VPMIC)
	&vpmic_regulator_vdd,
	&vpmic_regulator_vbat,
	&vpmic_regulator_vsim,
#endif
};

/*
* for wvga panel:
* 1: truly wvga panel
* 2: sharp wvga panel
*/
#define TRULY_WVGA_PANEL 1
#define SHARP_WVGA_PANEL 2
static int wvga_lcd = 0;
static int __init wvga_lcd_setup(char *str)
{
	int n;
	if (!get_option(&str, &n))
		return 0;
	wvga_lcd = n;
	return 1;
}
__setup("wvga_lcd=", wvga_lcd_setup);

static int is_wvga_lcd(void)
{
        return wvga_lcd;
}

#if defined(CONFIG_MMC_SDHCI_PXA)
#define HOST_SLEEP_EN 1

#if !(HOST_SLEEP_EN)
int mmc1_gpio_switch(unsigned int on, int with_card) {return 0}
#else
#include <linux/wakelock.h>
static struct wake_lock gpio_wakeup;

static irqreturn_t dat1_gpio_irq(int irq, void *data)
{
	unsigned int sec = 10;

	printk(KERN_INFO "%s: set wakelock, timout after %d seconds\n",
		__FUNCTION__, sec);

	wake_lock_timeout(&gpio_wakeup, HZ * sec);

	return IRQ_HANDLED;
}

static int gpio_wakeup_setup(u32 w_gpio)
{
	int ret;

	if (gpio_request(w_gpio, "SDIO dat1 GPIO Wakeup")) {
		printk(KERN_ERR "Failed to request GPIO %d "
				"for SDIO DATA1 GPIO Wakeup\n", w_gpio);
		return -EIO;
	}
	gpio_direction_input(w_gpio);

	ret = request_irq(gpio_to_irq(w_gpio), dat1_gpio_irq,
		IRQF_NO_SUSPEND | IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
		"SDIO data1 irq", NULL);
	if (ret) {
		printk(KERN_ERR "Request SDIO data1 GPIO irq failed %d\n", ret);
		return -EIO;
	}
	/* detect whether 8787 has interrupted PXA before we request gpio irq */
	ret = gpio_get_value(w_gpio);
	gpio_free(w_gpio);

	if(!ret)
		return -EIO;

	return 0;
}

static int mmc1_gpio_switch(unsigned int on, int with_card)
{
	int ret;

	mfp_cfg_t mfp_cfg_dat1 = MMC2_DAT1_GPIO_39 | MFP_LPM_EDGE_NONE;
	mfp_cfg_t mfp_cfg_gpio = MMC2_DAT1_IRQ_GPIO_39 | MFP_LPM_EDGE_BOTH \
				| MFP_PULL_HIGH;

	if (!with_card)
		return 0;

	if (on) {
		mfp_config(&mfp_cfg_gpio, 1);
		ret = gpio_wakeup_setup(mfp_to_gpio(mfp_cfg_gpio));
		if(ret){
			return ret;
		}
		enable_irq_wake(gpio_to_irq(mfp_to_gpio(mfp_cfg_gpio)));
	} else {
		disable_irq_wake(gpio_to_irq(mfp_to_gpio(mfp_cfg_gpio)));
		free_irq(gpio_to_irq(mfp_to_gpio(mfp_cfg_gpio)), NULL);
		mfp_config(&mfp_cfg_dat1, 1);
	}
	return 0;
}
#endif //HOST_SLEEP_EN

static unsigned int mmc0_get_ro(struct sdhci_host *host)
{
	//on TTC_TD platform, Micro SD does not support write-protect feature
	//So, always return 0
	return 0;
}

static void mmc0_set_ops(struct sdhci_pxa *pxa)
{
	pxa->ops->get_ro = mmc0_get_ro;
	pxa->ops->platform_8bit_width = pxa910_mmc_set_width;
}

static int mmc0_lp_switch(unsigned int on, int with_card)
{
	static struct regulator *regulator_sd_pins = NULL;
	static struct regulator *regulator_sd_slot = NULL;
	static int sd_pwr_en = 0;
	int error = 0;
	int ret = 0;

	if (!regulator_sd_pins) {
		/* LDO14, power supply of MMC0 pins */
		regulator_sd_pins = regulator_get(NULL, "v_ldo14");
		if (IS_ERR(regulator_sd_pins)) {
			regulator_sd_pins = NULL;
		} else {
			ret = regulator_enable(regulator_sd_pins);
			if (ret < 0) {
				printk(KERN_ERR "Failed to enable LDO14, "
					"SD may not work, ret = %d\n", ret);
				regulator_sd_pins = NULL;
			}
		}
	}

	if (emmc_boot) {
		if (!regulator_sd_slot) {
			/* on PXA921, LDO13, power supply to SD slot */
			regulator_sd_slot = regulator_get(NULL, "v_ldo13");
			if (IS_ERR(regulator_sd_slot)) {
				regulator_sd_slot = NULL;
			} else {
				ret = regulator_enable(regulator_sd_slot);
				if (ret < 0) {
					printk(KERN_ERR "Failed to enable LDO13, "
						"SD may not work, ret = %d\n", ret);
					regulator_sd_slot = NULL;
				}
			}
		}
	} else {
		if (is_td_dkb) {
			sd_pwr_en = mfp_to_gpio(GPIO15_GPIO15);
			if (gpio_request(sd_pwr_en, "SD Power Ctrl")) {
				printk(KERN_ERR "Failed to request SD_PWR_EN(gpio %d), "
					"SD card might not work\n", sd_pwr_en);
				sd_pwr_en = 0;
			}
		} else {
			sd_pwr_en = GPIO_EXT1(5);
			if (gpio_request(sd_pwr_en, "SD Power Ctrl")) {
				printk(KERN_ERR "Failed to request SD_PWR_EN(gpio %d), "
					"SD card might not work\n", sd_pwr_en);
				sd_pwr_en = 0;
			}
		}
	}

	if (!regulator_sd_pins) {
		error = 1;
	} else {
		if (emmc_boot) {
			if (!regulator_sd_slot)
				error = 1;
		} else {
			if (!sd_pwr_en)
				error = 1;
		}
	}

	if (error) {
		printk(KERN_ERR "Failed to get power control of SD\n");
		return -EIO;
	}

	if (on) {
		if (emmc_boot) {
			ret = regulator_disable(regulator_sd_slot);
			if (ret < 0)
				printk(KERN_ERR "Failed to turn off LDO13 "
					"for SD slot, ret = %d\n", ret);
		} else {
			gpio_direction_output(sd_pwr_en, 0);
			gpio_free(sd_pwr_en);
		}

		ret = regulator_disable(regulator_sd_pins);
		if (ret < 0)
			printk(KERN_ERR "Failed to turn off LDO14 "
				"for SD slot, ret = %d\n", ret);
	} else {
		if (emmc_boot) {
			ret = regulator_enable(regulator_sd_slot);
			if (ret < 0)
				printk(KERN_ERR "Failed to turn on LDO13, "
					"SD may not work, ret = %d\n", ret);
		} else {
			gpio_direction_output(sd_pwr_en, 1);
			gpio_free(sd_pwr_en);
		}

		ret = regulator_enable(regulator_sd_pins);
		if (ret < 0)
			printk(KERN_ERR "Failed to turn on LDO14, "
				"SD may not work, ret = %d\n", ret);
	}

	return 0;
}

static unsigned int mmc2_get_ro(struct sdhci_host *host)
{
	/* eMMC on mmc2 always writable */
	return 0;
}

static void mmc2_set_ops(struct sdhci_pxa *pxa)
{
	pxa->ops->get_ro = mmc2_get_ro;
	pxa->ops->platform_8bit_width = pxa910_mmc_set_width;
}

/* MMC0 controller for SD-MMC */
static struct sdhci_pxa_platdata pxa910_sdh_platdata_mmc0 = {
	.quirks			= SDHCI_QUIRK_BROKEN_ADMA
		| SDHCI_QUIRK_CAP_CLOCK_BASE_BROKEN,
	.clk_delay_sel		= 1,
	.clk_delay_cycles	= 2,
	.soc_set_ops		= mmc0_set_ops,
	.lp_switch		= mmc0_lp_switch,
	.soc_set_timing		= pxa910_sdh_specific_ctrl,
};

/* MMC1 controller for SDIO */
static struct sdhci_pxa_platdata pxa910_sdh_platdata_mmc1 = {
	.quirks			= SDHCI_QUIRK_BROKEN_ADMA
		| SDHCI_QUIRK_CAP_CLOCK_BASE_BROKEN,
	.flags			= PXA_FLAG_DISABLE_CLOCK_GATING |
					PXA_FLAG_SDIO_RESUME |
					PXA_FLAG_CARD_PERMANENT,
	.lp_switch		= mmc1_gpio_switch,
	.soc_set_timing		= pxa910_sdh_specific_ctrl,
};

/* MMC2 controller for EMMC */
static struct sdhci_pxa_platdata pxa910_sdh_platdata_mmc2 = {
	.quirks			= SDHCI_QUIRK_BROKEN_ADMA
		| SDHCI_QUIRK_CAP_CLOCK_BASE_BROKEN,
	.flags			= PXA_FLAG_CARD_PERMANENT
					| PXA_FLAG_SD_8_BIT_CAPABLE_SLOT,
	.clk_delay_sel		= 1,
	.clk_delay_cycles	= 2,
	.soc_set_ops		= mmc2_set_ops,
	.soc_set_timing		= pxa910_sdh_specific_ctrl,
};

static void ttc_dkb_wifi_set_power(unsigned int on)
{
	static int WIB_EN = 0;
	static int WLAN_LHC = 0;

	if (!WIB_EN || !WLAN_LHC) {
		if (is_td_dkb) {
			WIB_EN = GPIO_EXT1(14);
			WLAN_LHC = GPIO_EXT1(2);
		} else {
			WIB_EN = mfp_to_gpio(WIB_EN_GPIO_33);
			WLAN_LHC = mfp_to_gpio(WLAN_LHC_GPIO_36);
		}

		if (gpio_request(WIB_EN, "WIB_EN")) {
			printk(KERN_INFO "gpio %d request failed\n", WIB_EN);
			WIB_EN = WLAN_LHC = 0;
			return;
		}
		if (gpio_request(WLAN_LHC, "WLAN_LHC")) {
			printk(KERN_INFO "gpio %d request failed\n", WLAN_LHC);
			gpio_free(WIB_EN);
			WIB_EN = WLAN_LHC = 0;
			return;
		}

#ifdef CONFIG_GPIO_SYSFS
		gpio_export(WIB_EN, false);
		gpio_export(WLAN_LHC, false);
#endif
	}
	BUG_ON(!WIB_EN || !WLAN_LHC);

	pr_debug("%s: on=%d\n", __FUNCTION__, on);
	if (on) {
		if (WIB_EN) gpio_direction_output(WIB_EN, 1);
		if (WLAN_LHC) gpio_direction_output(WLAN_LHC, 1);
	} else {
		if (WIB_EN) gpio_direction_output(WIB_EN, 0);
		if (WLAN_LHC) gpio_direction_output(WLAN_LHC, 0);
	}
}


static void __init pxa910_init_mmc(void)
{
	/* enable power for SD slot on DKB3.1 for TD */
	unsigned long sd_pwr_cfg = GPIO15_GPIO15;
	int sd_pwr_en = 0;

#ifdef CONFIG_SD8XXX_RFKILL
	int WIB_PDn;
	int WIB_RESETn;

	if(is_td_dkb) {
		WIB_PDn = GPIO_EXT1(0);
		WIB_RESETn = GPIO_EXT1(1);
	} else {
		WIB_PDn = mfp_to_gpio(WLAN_PD_GPIO_14);
		WIB_RESETn = mfp_to_gpio(WLAN_RESET_GPIO_20);
	}

	add_sd8x_rfkill_device(WIB_PDn, WIB_RESETn,
		&pxa910_sdh_platdata_mmc1.pmmc, ttc_dkb_wifi_set_power);
#endif

	if (!emmc_boot) {
		if (is_td_dkb) {
			mfp_config(&sd_pwr_cfg, 1);
			sd_pwr_en = mfp_to_gpio(sd_pwr_cfg);

			if (gpio_request(sd_pwr_en, "SD Power Ctrl")) {
				printk(KERN_ERR "Failed to request SD_PWR_EN(gpio %d)\n", sd_pwr_en);
				sd_pwr_en = 0;
			} else {
				gpio_direction_output(sd_pwr_en, 1);
				gpio_free(sd_pwr_en);
			}
		}
	}

	if (emmc_boot) {
		mfp_config(ARRAY_AND_SIZE(emmc_pin_config));
	}

	/* Always register SDHC2 as we need to support both PXA920 (no eMMC)
	 * and PXA921 (with eMMC). Otherwise the device number will be different
	 * on two platform, which causes Android cannot mount SD card correctly */
	pxa910_add_sdh(2, &pxa910_sdh_platdata_mmc2);

	pxa910_add_sdh(0, &pxa910_sdh_platdata_mmc0); /* SD/MMC */
	pxa910_add_sdh(1, &pxa910_sdh_platdata_mmc1);
#if (HOST_SLEEP_EN)
	wake_lock_init(&gpio_wakeup, WAKE_LOCK_SUSPEND, "hs_wakeups");
#endif
}
#endif

#ifdef CONFIG_PM
static unsigned long GPIO[110];
static int ttc_pin_lpm_config(void)
{
	unsigned int pin_index = 0, i = 0;
	for(pin_index = MFP_PIN_GPIO0;pin_index <= MFP_PIN_GPIO109;pin_index++)
		GPIO[i++] = mfp_read(pin_index);
	/*save VCC_IO_GPIO1 0.25mA*/
	mfp_write(MFP_PIN_GPIO1,0xd081);
	mfp_write(MFP_PIN_GPIO3,0xd081);
	mfp_write(MFP_PIN_GPIO5,0xd081);
	mfp_write(MFP_PIN_GPIO7,0xd081);
	mfp_write(MFP_PIN_GPIO9,0xd081);
	mfp_write(MFP_PIN_GPIO10,0xb080);
	mfp_write(MFP_PIN_GPIO46,0xb0c0);
	mfp_write(MFP_PIN_GPIO49,0xb080);
	mfp_write(MFP_PIN_GPIO51,0xb080);
	mfp_write(MFP_PIN_GPIO52,0xb0c0);

         /* save VCC_IO_GPIO3 0.14mA */
	mfp_write(MFP_PIN_GPIO81,0xb081);
	mfp_write(MFP_PIN_GPIO82,0xb081);
	mfp_write(MFP_PIN_GPIO83,0xb081);
	mfp_write(MFP_PIN_GPIO84,0xb081);
	mfp_write(MFP_PIN_GPIO85,0xb081);
	mfp_write(MFP_PIN_GPIO86,0xb081);
	mfp_write(MFP_PIN_GPIO87,0xb081);
	mfp_write(MFP_PIN_GPIO88,0xb081);
	mfp_write(MFP_PIN_GPIO89,0xb081);
	mfp_write(MFP_PIN_GPIO90,0xb081);
	mfp_write(MFP_PIN_GPIO91,0xb081);
	mfp_write(MFP_PIN_GPIO92,0xb081);
	mfp_write(MFP_PIN_GPIO93,0xb081);
	mfp_write(MFP_PIN_GPIO94,0xb081);
	mfp_write(MFP_PIN_GPIO95,0xb081);
	mfp_write(MFP_PIN_GPIO96,0xb081);
	mfp_write(MFP_PIN_GPIO97,0xb081);
	mfp_write(MFP_PIN_GPIO98,0xb081);
	mfp_write(MFP_PIN_GPIO100,0xb081);
	mfp_write(MFP_PIN_GPIO101,0xb081);
	mfp_write(MFP_PIN_GPIO102,0xb081);
	mfp_write(MFP_PIN_GPIO103,0xb081);
	mfp_write(MFP_PIN_GPIO104,0xb083);
	mfp_write(MFP_PIN_GPIO105,0xb083);
	mfp_write(MFP_PIN_GPIO106,0xb080);
	mfp_write(MFP_PIN_GPIO107,0xb083);
	mfp_write(MFP_PIN_GPIO108,0xb083);

	/*turn off GPIO3 power domain*/
	mfp_gpio3_power_down();

	return 0;
}

static int ttc_pin_restore(void)
{
	unsigned int pin_index=0,i=0;
	for(pin_index = MFP_PIN_GPIO0;pin_index <= MFP_PIN_GPIO109;pin_index++)
		mfp_write(pin_index,GPIO[i++]);
	/*turn on GPIO3 power domain*/
	mfp_gpio3_power_up();
	return 0;
}

static struct pxa910_peripheral_config_ops config_ops = {
	.pin_lpm_config	= ttc_pin_lpm_config,
	.pin_restore	= ttc_pin_restore,
};
#endif
/* GPS: power on/off control */
static void gps_power_on(void)
{
	int gps_ldo, gps_rst_n;

	gps_ldo = (is_td_dkb) ? GPIO_EXT1(8) : GPIO_EXT1(7);
	if (gpio_request(gps_ldo, "gpio_gps_ldo")) {
		pr_err("Request GPIO failed, gpio: %d\n", gps_ldo);
		return;
	}

	gps_rst_n = (is_td_dkb) ? GPIO_EXT1(11) : mfp_to_gpio(MFP_PIN_GPIO15);
	if (gpio_request(gps_rst_n, "gpio_gps_rst")) {
		pr_err("Request GPIO failed, gpio: %d\n", gps_rst_n);
		goto out;
	}

	gpio_direction_output(gps_ldo, 0);
	gpio_direction_output(gps_rst_n, 0);
	mdelay(1);
	gpio_direction_output(gps_ldo, 1);

	pr_info("sirf gps chip powered on\n");

	gpio_free(gps_rst_n);
out:
	gpio_free(gps_ldo);
	return;
}

static void gps_power_off(void)
{
	int gps_ldo, gps_rst_n, gps_on;

	gps_ldo = (is_td_dkb) ? GPIO_EXT1(8) : GPIO_EXT1(7);
	if (gpio_request(gps_ldo, "gpio_gps_ldo")) {
		pr_err("Request GPIO failed, gpio: %d\n", gps_ldo);
		return;
	}

	gps_on = (is_td_dkb) ? GPIO_EXT1(10) : GPIO_EXT1(1);
	if (gpio_request(gps_on, "gpio_gps_on")) {
		pr_err("Request GPIO failed,gpio: %d\n", gps_on);
		goto out1;
	}

	gps_rst_n = (is_td_dkb) ? GPIO_EXT1(11) : mfp_to_gpio(MFP_PIN_GPIO15);
	if (gpio_request(gps_rst_n, "gpio_gps_rst")) {
		pr_debug("Request GPIO failed, gpio: %d\n", gps_rst_n);
		goto out2;
	}

	gpio_direction_output(gps_ldo, 0);
	gpio_direction_output(gps_rst_n, 0);
	gpio_direction_output(gps_on, 0);

	pr_info("sirf gps chip powered off\n");

	gpio_free(gps_rst_n);
out2:
	gpio_free(gps_on);
out1:
	gpio_free(gps_ldo);
	return;
}

static void gps_reset(int flag)
{
	int gps_rst_n;

	gps_rst_n = (is_td_dkb) ? GPIO_EXT1(11) : mfp_to_gpio(MFP_PIN_GPIO15);
	if (gpio_request(gps_rst_n, "gpio_gps_rst")) {
		pr_err("Request GPIO failed, gpio: %d\n", gps_rst_n);
		return;
	}

	gpio_direction_output(gps_rst_n, flag);
	gpio_free(gps_rst_n);
	pr_info("sirf gps chip reset\n");
}

static void gps_on_off(int flag)
{
	int gps_on;

	gps_on = (is_td_dkb) ? GPIO_EXT1(10) : GPIO_EXT1(1);
	if (gpio_request(gps_on, "gpio_gps_on")) {
		pr_err("Request GPIO failed, gpio: %d\n", gps_on);
		return;
	}
	gpio_direction_output(gps_on, flag);
	gpio_free(gps_on);
	pr_info("sirf gps chip offon\n");
}

#if defined(CONFIG_PROC_FS)

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
		pr_warning("[ERROR] messages too long! (%d) %s\n",
			strlen(messages), messages);
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
	} else
		pr_info("usage: echo {on/off} > /proc/driver/sirf\n");

	return len;
}

static void create_sirf_proc_file(void)
{
	struct proc_dir_entry *sirf_proc_file = NULL;

	sirf_proc_file = create_proc_entry("driver/sirf", 0644, NULL);
	if (!sirf_proc_file) {
		pr_err("proc file create failed!\n");
		return;
	}

	sirf_proc_file->read_proc = sirf_read_proc;
	sirf_proc_file->write_proc = (write_proc_t  *)sirf_write_proc;
}
#endif

static void ttc_dkb_restart(char mode, const char *cmd)
{
	/* enable watchdog reset */
	arm_machine_restart('w', cmd);
}

#if (defined CONFIG_CMMB)

static unsigned long cmmb_pin_config[] = {
	GPIO33_SSP0_CLK,
	GPIO35_SSP0_RXD,
	GPIO36_SSP0_TXD,
};

static struct pxa2xx_spi_master pxa_ssp_master_info = {
	.num_chipselect = 1,
	.enable_dma = 1,
};

static int cmmb_power_reset(void)
{
	int cmmb_rst;

	cmmb_rst = GPIO_EXT1(7);

	if (gpio_request(cmmb_rst, "cmmb rst")) {
		pr_warning("failed to request GPIO for CMMB RST\n");
		return -EIO;
	}

	/* reset cmmb, keep low for about 1ms */
	gpio_direction_output(cmmb_rst, 0);
	msleep(100);

	/* get cmmb go out of reset state */
	gpio_direction_output(cmmb_rst, 1);
	gpio_free(cmmb_rst);

	return 0;
}

static int cmmb_power_on(void)
{
	int cmmb_en, cmmb_rst;

	cmmb_en = GPIO_EXT1(6);
	if (gpio_request(cmmb_en, "cmmb power")) {
		pr_warning("[ERROR] failed to request GPIO for CMMB POWER\n");
		return -EIO;
	}
	gpio_direction_output(cmmb_en, 0);
	msleep(100);

	gpio_direction_output(cmmb_en, 1);
	gpio_free(cmmb_en);

	msleep(100);

	cmmb_power_reset();

	return 0;
}

static int cmmb_power_off(void)
{
	int cmmb_en;

	cmmb_en = GPIO_EXT1(6);

	if (gpio_request(cmmb_en, "cmmb power")) {
		pr_warning("failed to request GPIO for CMMB POWER\n");
		return -EIO;
	}

	gpio_direction_output(cmmb_en, 0);
	gpio_free(cmmb_en);
	msleep(100);

	return 0;
}
/*.
 ** Add two functions: cmmb_cs_assert and cmmb_cs_deassert.
 ** Provide the capbility that
 ** cmmb driver can handle the SPI_CS by itself.
 **/
static int cmmb_cs_assert(void)
{
	int cs;
	cs = mfp_to_gpio(GPIO34_SSP0_FRM);
	gpio_direction_output(cs, 0);
	return 0;
}

static int cmmb_cs_deassert(void)
{
	int cs;
	cs = mfp_to_gpio(GPIO34_SSP0_FRM);
	gpio_direction_output(cs, 1);
	return 0;
}

static struct cmmb_platform_data cmmb_info = {
	.power_on = cmmb_power_on,
	.power_off = cmmb_power_off,
	.power_reset = cmmb_power_reset,
	.cs_assert = cmmb_cs_assert,
	.cs_deassert = cmmb_cs_deassert,

	.gpio_power = GPIO_EXT1(6),
	.gpio_reset = GPIO_EXT1(7),
	.gpio_cs = mfp_to_gpio(GPIO34_SSP0_FRM),
	.gpio_defined = 1,
};

static void cmmb_if101_cs(u32 cmd)
{
/* Because in CMMB read/write,the max data size is more than 8kB
 * 8k = max data length per dma transfer for pxaxxx
 * But till now,The spi_read/write driver doesn't support muti DMA cycles
 *
 * Here the spi_read/write will not affect the SPI_CS,but provides
 * cs_assert and cs_deassert in the struct cmmb_platform_data
 *
 * And cmmb driver can/should control SPI_CS by itself
 */
}

static struct pxa2xx_spi_chip cmmb_if101_chip = {
	.rx_threshold   = 1,
	.tx_threshold   = 1,
	.cs_control     = cmmb_if101_cs,
};

/* bus_num must match id in pxa2xx_set_spi_info() call */
static struct spi_board_info spi_board_info[] __initdata = {
	{
		.modalias		= "cmmb_if",
		.platform_data	= &cmmb_info,
		.controller_data	= &cmmb_if101_chip,
		.irq			= gpio_to_irq(mfp_to_gpio(GPIO14)),
		.max_speed_hz	= 8000000,
		.bus_num		= 1,
		.chip_select	= 0,
		.mode			= SPI_MODE_0,
	},
};

static void __init ttc_dkb_init_spi(void)
{
	int err;
	int cmmb_int, cmmb_cs;

	mfp_config(ARRAY_AND_SIZE(cmmb_pin_config));
	cmmb_cs = mfp_to_gpio(GPIO34_SSP0_FRM);
	err = gpio_request(cmmb_cs, "cmmb cs");
	if (err) {
		pr_warning("[ERROR] failed to request GPIO for CMMB CS\n");
		return;
	}
	gpio_direction_output(cmmb_cs, 1);

	cmmb_int = mfp_to_gpio(GPIO14);

	err = gpio_request(cmmb_int, "cmmb irq");
	if (err) {
		pr_warning("[ERROR] failed to request GPIO for CMMB IRQ\n");
		return;
	}
	gpio_direction_input(cmmb_int);

	pxa910_add_ssp(0);
	pxa910_add_spi(1, &pxa_ssp_master_info);
	if (spi_register_board_info(spi_board_info,
			ARRAY_SIZE(spi_board_info))) {
		pr_warning("[ERROR] failed to register spi device.\n");
		return;
	}
}

#endif /*defined CONFIG_CMMB*/


static void __init tds_init(void)
{
	if (is_td_dkb) {
		mfp_config(ARRAY_AND_SIZE(tds_pin_config));
		mfp_write(MFP_PIN_GPIO55, 0x00c0);
		mfp_write(MFP_PIN_GPIO57, 0x00c0);
		mfp_write(MFP_PIN_GPIO58, 0x00c0);
		mfp_write(MFP_PIN_GPIO59, 0x00c0);
	}
}

static void __init ttc_dkb_init(void)
{
	arm_pm_restart = ttc_dkb_restart;

	mfp_config(ARRAY_AND_SIZE(ttc_dkb_pin_config));
	/* Reconfig the GPIO113 mfpr as mfp_config doesn't support bit 7/6*/
	mfp_write(MFP_PIN_GPIO113, 0xC0C0);

	if (!emmc_boot)
		pxa910_add_nand(&ttc_dkb_nand_info);
	/* on-chip devices */
	pxa910_add_uart(0);
	pxa910_add_uart(1);
	pxa910_add_uart(2);

	pxa910_add_ssp(1);
	pxa910_add_keypad(&ttc_dkb_keypad_info);
	pxa910_add_cnm();

	pxa910_add_acipc();
#ifdef CONFIG_PXA910_IRE
	pxa910_add_ire();
#endif
	/* off-chip devices */
	platform_device_add_data(&pxa910_device_twsi0, &ttc_dkb_i2c_pdata,
				 sizeof(struct i2c_pxa_platform_data));
	platform_device_add_data(&pxa910_device_twsi1, &ttc_dkb_pwr_i2c_pdata,
				 sizeof(struct i2c_pxa_platform_data));

	platform_add_devices(ARRAY_AND_SIZE(ttc_dkb_devices));
	i2c_register_board_info(0, ARRAY_AND_SIZE(ttc_dkb_i2c_info));

	if (emmc_boot) {
		platform_device_register(&pxa910_device_twsi1);
		i2c_register_board_info(1, ARRAY_AND_SIZE(ttc_dkb_pwr_i2c_info));
	}

	platform_device_register(&pxa910_device_rtc);
#ifdef CONFIG_USB_GADGET
	pxa_device_u2o.dev.platform_data = (void *)&pxa910_u2o_info;
	platform_device_register(&pxa_device_u2o);
#endif

#ifdef CONFIG_USB_OTG
	pxa_device_u2ootg.dev.platform_data = (void *)&pxa910_u2o_info;
	platform_device_register(&pxa_device_u2ootg);
	pxa_device_u2oehci.dev.platform_data = (void *)&pxa910_u2o_info;
	platform_device_register(&pxa_device_u2oehci);
#endif

#ifdef CONFIG_USB_ANDROID
	pxa910_android_add_usb_devices();
#endif

#ifdef CONFIG_ANDROID_PMEM
	pxa_add_pmem("pmem", reserving_size, 0, 1, 1);
	pxa_add_pmem("pmem_adsp", 0, 0, 0, 0);
#endif

#ifdef CONFIG_FB_PXA168
	mfp_config(ARRAY_AND_SIZE(lcd_tpo_pin_config));
	if (TRULY_WVGA_PANEL == is_wvga_lcd()) {
		dkb_add_lcd_truly();
		printk(KERN_INFO "LCD: truly WVGA panel selected.\n");
	} else if (SHARP_WVGA_PANEL == is_wvga_lcd()) {
		dkb_add_lcd_sharp();
		printk(KERN_INFO "LCD: sharp WVGA panel selected.\n");
	} else
		dkb_add_lcd_tpo();
#endif

#if defined(CONFIG_MMC_SDHCI_PXA)
	pxa910_init_mmc();
#endif
	pxa910_add_freq();
	pxa910_add_1wire();

#if defined(CONFIG_PXA910_CAMERA) || defined(CONFIG_VIDEO_PXA910)
	mfp_config(ARRAY_AND_SIZE(ccic_dvp_pin_config));
	pxa910_add_cam();
#endif

#if defined(CONFIG_PROC_FS)
	/* create proc for sirf GPS control */
	create_sirf_proc_file();
#endif

#if (defined CONFIG_CMMB)
	 /* spi device */
	if (is_td_dkb)
		ttc_dkb_init_spi();
#endif

#ifdef CONFIG_PM
	pxa910_power_config_register(&config_ops);
#endif

	tds_init();
}

MACHINE_START(TTC_DKB, "PXA910-based TTC_DKB Development Platform")
	.map_io		= pxa_map_io,
	.nr_irqs	= TTCDKB_NR_IRQS,
	.init_irq       = pxa910_init_irq,
	.timer          = &pxa910_timer,
	.init_machine   = ttc_dkb_init,
MACHINE_END
