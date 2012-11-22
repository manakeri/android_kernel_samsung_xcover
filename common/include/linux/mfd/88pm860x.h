/*
 * Marvell 88PM860x Interface
 *
 * Copyright (C) 2009 Marvell International Ltd.
 * 	Haojian Zhuang <haojian.zhuang@marvell.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __LINUX_MFD_88PM860X_H
#define __LINUX_MFD_88PM860X_H

#include <linux/interrupt.h>

#define MFD_NAME_SIZE		(40)

enum {
	CHIP_INVALID = 0,
	CHIP_PM8606,
	CHIP_PM8607,
	CHIP_MAX,
};

enum {
	PM8606_ID_INVALID,
	PM8606_ID_BACKLIGHT,
	PM8606_ID_LED,
	PM8606_ID_VIBRATOR,
	PM8606_ID_TOUCH,
	PM8606_ID_SOUND,
	PM8606_ID_CHARGER,
	PM8606_ID_MAX,
};

enum {
	PM8606_BACKLIGHT1 = 0,
	PM8606_BACKLIGHT2,
	PM8606_BACKLIGHT3,
};

enum {
	PM8606_LED1_RED = 0,
	PM8606_LED1_GREEN,
	PM8606_LED1_BLUE,
	PM8606_LED2_RED,
	PM8606_LED2_GREEN,
	PM8606_LED2_BLUE,
	PM8607_LED_VIBRATOR,
};

/* 8606 Registers */
#define PM8606_DCM_BOOST		(0x00)
#define PM8606_PWM			(0x01)

/* Backlight Registers */
#define PM8606_WLED1A			(0x02)
#define PM8606_WLED1B			(0x03)
#define PM8606_WLED2A			(0x04)
#define PM8606_WLED2B			(0x05)
#define PM8606_WLED3A			(0x06)
#define PM8606_WLED3B			(0x07)

/* LED Registers */
#define PM8606_RGB2A			(0x08)
#define PM8606_RGB2B			(0x09)
#define PM8606_RGB2C			(0x0A)
#define PM8606_RGB2D			(0x0B)
#define PM8606_RGB1A			(0x0C)
#define PM8606_RGB1B			(0x0D)
#define PM8606_RGB1C			(0x0E)
#define PM8606_RGB1D			(0x0F)

#define PM8606_PREREGULATORA		(0x10)
#define PM8606_PREREGULATORB		(0x11)
#define PM8606_VIBRATORA		(0x12)
#define PM8606_VIBRATORB		(0x13)
#define PM8606_VCHG			(0x14)
#define PM8606_VSYS			(0x15)
#define PM8606_MISC			(0x16)
#define PM8606_CHIP_ID			(0x17)
#define PM8606_STATUS			(0x18)
#define PM8606_FLAGS			(0x19)
#define PM8606_PROTECTA			(0x1A)
#define PM8606_PROTECTB			(0x1B)
#define PM8606_PROTECTC			(0x1C)

/* Bit definitions of PM8606 registers */
#define PM8606_DCM_500MA		(0x0)	/* current limit */
#define PM8606_DCM_750MA		(0x1)
#define PM8606_DCM_1000MA		(0x2)
#define PM8606_DCM_1250MA		(0x3)
#define PM8606_DCM_250MV		(0x0 << 2)
#define PM8606_DCM_300MV		(0x1 << 2)
#define PM8606_DCM_350MV		(0x2 << 2)
#define PM8606_DCM_400MV		(0x3 << 2)

#define PM8606_PWM_31200HZ		(0x0)
#define PM8606_PWM_15600HZ		(0x1)
#define PM8606_PWM_7800HZ		(0x2)
#define PM8606_PWM_3900HZ		(0x3)
#define PM8606_PWM_1950HZ		(0x4)
#define PM8606_PWM_976HZ		(0x5)
#define PM8606_PWM_488HZ		(0x6)
#define PM8606_PWM_244HZ		(0x7)
#define PM8606_PWM_FREQ_MASK		(0x7)

#define PM8606_WLED_ON			(1 << 0)
#define PM8606_WLED_CURRENT(x)		((x & 0x1F) << 1)

#define PM8606_LED_CURRENT(x)		(((x >> 2) & 0x07) << 5)

#define PM8606_VSYS_EN			(1 << 1)

#define PM8606_MISC_OSC_EN		(1 << 4)

enum {
	PM8607_ID_BUCK1 = 0,
	PM8607_ID_BUCK2,
	PM8607_ID_BUCK3,

	PM8607_ID_LDO1,
	PM8607_ID_LDO2,
	PM8607_ID_LDO3,
	PM8607_ID_LDO4,
	PM8607_ID_LDO5,
	PM8607_ID_LDO6,
	PM8607_ID_LDO7,
	PM8607_ID_LDO8,
	PM8607_ID_LDO9,
	PM8607_ID_LDO10,
	PM8607_ID_LDO11,
	PM8607_ID_LDO12,
	PM8607_ID_LDO13,
	PM8607_ID_LDO14,
	PM8607_ID_LDO15,

	PM8607_ID_RG_MAX,
};

#define PM8607_VERSION_MASK		(0xFF)	/* 8607 chip ID mask */

/* Interrupt Registers */
#define PM8607_STATUS_1			(0x01)
#define PM8607_STATUS_2			(0x02)
#define PM8607_INT_STATUS1		(0x03)
#define PM8607_INT_STATUS2		(0x04)
#define PM8607_INT_STATUS3		(0x05)
#define PM8607_INT_MASK_1		(0x06)
#define PM8607_INT_MASK_2		(0x07)
#define PM8607_INT_MASK_3		(0x08)

#define PM8607_INT_STS3_AS		(1 << 0)
#define PM8607_INT_EN_AS		(1 << 0)
#define PM8607_INT_STS3_PEN		(1 << 1)
#define PM8607_INT_EN_PEN		(1 << 1)
#define PM8607_INT_EN_HEADSET		(1 << 2)
#define PM8607_INT_EN_HOOK		(1 << 3)
#define PM8607_INT_EN_MICIN		(1 << 4)
#define PM8607_INT_EN_CHG_FAIL		(1 << 5)
#define PM8607_INT_EN_CHG_DONE		(1 << 6)
#define PM8607_INT_EN_CHG_IOVER		(1 << 7)

/* Wakeup Registers */
#define PM8607_RESET_OUT		(0x09)

/* Regulator Control Registers */
#define PM8607_LDO1			(0x10)
#define PM8607_LDO2			(0x11)
#define PM8607_LDO3			(0x12)
#define PM8607_LDO4			(0x13)
#define PM8607_LDO5			(0x14)
#define PM8607_LDO6			(0x15)
#define PM8607_LDO7			(0x16)
#define PM8607_LDO8			(0x17)
#define PM8607_LDO9			(0x18)
#define PM8607_LDO10			(0x19)
#define PM8607_LDO12			(0x1A)
#define PM8607_LDO14			(0x1B)
#define PM8607_SLEEP_MODE1		(0x1C)
#define PM8607_SLEEP_MODE2		(0x1D)
#define PM8607_SLEEP_MODE3		(0x1E)
#define PM8607_SLEEP_MODE4		(0x1F)
#define PM8607_GO			(0x20)
#define PM8607_SLEEP_BUCK1		(0x21)
#define PM8607_SLEEP_BUCK2		(0x22)
#define PM8607_SLEEP_BUCK3		(0x23)
#define PM8607_BUCK1			(0x24)
#define PM8607_BUCK2			(0x25)
#define PM8607_BUCK3			(0x26)
#define PM8607_BUCK_CONTROLS		(0x27)
#define PM8607_VIBRA_SET		(0x28)
#define PM8607_SUPPLIES_EN11		(0x2B)
#define PM8607_SUPPLIES_EN12		(0x2C)
#define PM8607_GROUP1			(0x2D)
#define PM8607_GROUP2			(0x2E)
#define PM8607_GROUP3			(0x2F)
#define PM8607_GROUP4			(0x30)
#define PM8607_GROUP5			(0x31)
#define PM8607_GROUP6			(0x32)
#define PM8607_SUPPLIES_EN21		(0x33)
#define PM8607_SUPPLIES_EN22		(0x34)
#define PM8607_LP_CONFIG1		(0x35)
#define PM8607_LP_CONFIG2		(0x36)
#define PM8607_LP_CONFIG3		(0x39)

/* headset/mic detection Control Registers */
#define PM8607_MIC_DECTION		(0x37)
#define PM8607_HEADSET_DECTION	(0x38)

#define PM8607_MISC_REG0        (0x3A)

/* bit definitions of  MEAS_EN1*/
#define PM8607_MIC_DET_EN_MIC_DET	(1 << 0)
#define PM8607_MIC_PERIOD		(0x3 << 1) //MARVEL <<< Alkon detection
#define PM8607_HEADSET_EN_HS_DET	(1 << 0)
#define PM8607_ADC_EN_MIC2_BIAS		(0x3 << 5)
#define PM8607_HEADSET_BTN_DBNC		(0x3 << 3)
#define PM8607_HEADSET_PERIOD		(0x3 << 1)
#define PM8607_MICBAIS_CURRENT	        (0x3 << 0) //MARVEL <<< Alkon detection

/* Vibrator Control Registers */
#define PM8607_VIBRATOR_SET		(0x28)
#define PM8607_VIBRATOR_PWM		(0x29)

#define PM8607_MISC2			(0x42)

/* power up log register*/
#define PM8607_POWER_UP_LOG		(0x3F)

/* Charger Control Registers */
#define PM8607_CCNT			(0x47)
#define PM8607_CHG_CTRL1		(0x48)
#define PM8607_CHG_CTRL2		(0x49)
#define PM8607_CHG_CTRL3		(0x4A)
#define PM8607_CHG_CTRL4		(0x4B)
#define PM8607_CHG_CTRL5		(0x4C)
#define PM8607_CHG_CTRL6		(0x4D)
#define PM8607_CHG_CTRL7		(0x4E)

/* GPADC Registers */
#define PM8607_GP_BIAS1			(0x4F)
#define PM8607_MEAS_EN1			(0x50)
#define PM8607_MEAS_EN2			(0x51)
#define PM8607_MEAS_EN3			(0x52)
#define PM8607_MEAS_OFF_TIME1		(0x53)
#define PM8607_MEAS_OFF_TIME2		(0x54)
#define PM8607_TSI_PREBIAS		(0x55)	/* prebias time */
#define PM8607_PD_PREBIAS		(0x56)	/* prebias time */
#define PM8607_GPADC_MISC1		(0x57)
#define PM8607_GPADC_MISC1_GPFSM_EN		(1 << 0)
#define PM8607_GPPADC_GP_PREBIAS_TIME(x)	(x << 1)
#define PM8607_GPPADC_SLOW_MODE(x)			(x << 3)
#define PM8607_GPADC_MISC2			(0x59)
#define PM8607_GPADC_GP_BIAS_EN0	(1 << 0)
#define PM8607_GPADC_GP_BIAS_EN1	(1 << 1)
#define PM8607_GPADC_GP_BIAS_EN2	(1 << 2)
#define PM8607_GPADC_GP_BIAS_EN3	(1 << 3)
#define PM8607_GPADC3_GP_BIAS_A3	(1 << 3)
#define PM8607_GPADC2_GP_BIAS_OUT2	(1 << 6)


/* bit definitions of  MEAS_EN1*/
#define PM8607_MEAS_EN1_VBAT           (1 << 0)
#define PM8607_MEAS_EN1_VCHG           (1 << 1)
#define PM8607_MEAS_EN1_VSYS           (1 << 2)
#define PM8607_MEAS_EN1_TINT           (1 << 3)
#define PM8607_MEAS_EN1_RFTMP          (1 << 4)
#define PM8607_MEAS_EN1_TBAT           (1 << 5)
#define PM8607_MEAS_EN1_GPADC2        	(1 << 6)
#define PM8607_MEAS_EN1_GPADC3         (1 << 7)


/* bit definitions of touch meas enable register 3 */
#define PM8607_MEAS_EN3_PENDET	(1 << 3)
#define PM8607_MEAS_EN3_TSIX	(1 << 4)
#define PM8607_MEAS_EN3_TSIY	(1 << 5)
#define PM8607_MEAS_EN3_TSIZ1	(1 << 6)
#define PM8607_MEAS_EN3_TSIZ2	(1 << 7)

/* Touch Registers */
#define PM8607_MEAS_TSIX_1		(0x8D)
#define PM8607_MEAS_TSIX_2		(0x8E)
#define PM8607_MEAS_TSIY_1		(0x8F)
#define PM8607_MEAS_TSIY_2		(0x90)
#define PM8607_MEAS_TSIZ1_1		(0x91)
#define PM8607_MEAS_TSIZ1_2		(0x92)
#define PM8607_MEAS_TSIZ2_1		(0x93)
#define PM8607_MEAS_TSIZ2_2		(0x94)

/* Battery Monitor Registers */
#define PM8607_GP_BIAS2			(0x5A)
#define PM8607_VBAT_LOWTH		(0x5B)
#define PM8607_VCHG_LOWTH		(0x5C)
#define PM8607_VSYS_LOWTH		(0x5D)
#define PM8607_TINT_LOWTH		(0x5E)
#define PM8607_GPADC0_LOWTH		(0x5F)
#define PM8607_GPADC1_LOWTH		(0x60)
#define PM8607_GPADC2_LOWTH		(0x61)
#define PM8607_GPADC3_LOWTH		(0x62)
#define PM8607_VBAT_HIGHTH		(0x63)
#define PM8607_VCHG_HIGHTH		(0x64)
#define PM8607_VSYS_HIGHTH		(0x65)
#define PM8607_TINT_HIGHTH		(0x66)
#define PM8607_GPADC0_HIGHTH		(0x67)
#define PM8607_GPADC1_HIGHTH		(0x68)
#define PM8607_GPADC2_HIGHTH		(0x69)
#define PM8607_GPADC3_HIGHTH		(0x6A)
#define PM8607_IBAT_MEAS1		(0x6B)
#define PM8607_IBAT_MEAS2		(0x6C)
#define PM8607_VBAT_MEAS1		(0x6D)
#define PM8607_VBAT_MEAS2		(0x6E)
#define PM8607_VCHG_MEAS1		(0x6F)
#define PM8607_VCHG_MEAS2		(0x70)
#define PM8607_VSYS_MEAS1		(0x71)
#define PM8607_VSYS_MEAS2		(0x72)
#define PM8607_TINT_MEAS1		(0x73)
#define PM8607_TINT_MEAS2		(0x74)
#define PM8607_GPADC0_MEAS1		(0x75)
#define PM8607_GPADC0_MEAS2		(0x76)
#define PM8607_GPADC1_MEAS1		(0x77)
#define PM8607_GPADC1_MEAS2		(0x78)
#define PM8607_GPADC2_MEAS1		(0x79)
#define PM8607_GPADC2_MEAS2		(0x7A)
#define PM8607_GPADC3_MEAS1		(0x7B)
#define PM8607_GPADC3_MEAS2		(0x7C)
#define PM8607_VRTC_MEAS1		(0x7D)
#define PM8607_VRTC_MEAS2		(0x7E)
#define PM8607_CCNT_MEAS1		(0x95)
#define PM8607_CCNT_MEAS2		(0x96)
#define PM8607_VBAT_AVG			(0x97)
#define PM8607_VCHG_AVG			(0x98)
#define PM8607_VSYS_AVG			(0x99)
#define PM8607_VBAT_MIN			(0x9A)
#define PM8607_VCHG_MIN			(0x9B)
#define PM8607_VSYS_MIN			(0x9C)
#define PM8607_VBAT_MAX			(0x9D)
#define PM8607_VCHG_MAX			(0x9E)
#define PM8607_VSYS_MAX			(0x9F)

/* RTC Control Registers */
#define PM8607_RTC1			(0xA0)
#define PM8607_RTC_COUNTER1		(0xA1)
#define PM8607_RTC_COUNTER2		(0xA2)
#define PM8607_RTC_COUNTER3		(0xA3)
#define PM8607_RTC_COUNTER4		(0xA4)
#define PM8607_RTC_EXPIRE1		(0xA5)
#define PM8607_RTC_EXPIRE2		(0xA6)
#define PM8607_RTC_EXPIRE3		(0xA7)
#define PM8607_RTC_EXPIRE4		(0xA8)
#define PM8607_RTC_TRIM1		(0xA9)
#define PM8607_RTC_TRIM2		(0xAA)
#define PM8607_RTC_TRIM3		(0xAB)
#define PM8607_RTC_TRIM4		(0xAC)
#define PM8607_RTC_MISC1		(0xAD)
#define PM8607_RTC_MISC2		(0xAE)
#define PM8607_RTC_MISC3		(0xAF)

/*Audio*/
#define PM8607_AUDIO_REG_BASE 0xb0
#define PM8607_AUDIO_REG_LEN  0x3b
#define PM8607_AUDIO_PCM_INTERFACE_1	0x00
#define PM8607_AUDIO_PCM_INTERFACE_2		0x01
#define PM8607_AUDIO_PCM_INTERFACE_3		0x02
#define PM8607_AUDIO_ADC_PCM		0x03
#define PM8607_AUDIO_ADC_1		0x04
#define PM8607_AUDIO_ADC_2		0x05
#define PM8607_AUDIO_ADC_3		0x06
#define PM8607_AUDIO_ADC_4		0x07
#define PM8607_AUDIO_ADC_5		0x08
#define PM8607_AUDIO_ADC_6		0x09
#define PM8607_AUDIO_ADC_7		0x0a
#define PM8607_AUDIO_I2S_INTERFACE_1		0x0b
#define PM8607_AUDIO_I2S_INTERFACE_2		0x0c
#define PM8607_AUDIO_I2S_INTERFACE_3		0x0d
#define PM8607_AUDIO_Equalizer_N0_1		0x0e
#define PM8607_AUDIO_Equalizer_N0_2		0x0f
#define PM8607_AUDIO_Equalizer_N1_1		0x11
#define PM8607_AUDIO_Equalizer_N1_2		0x12
#define PM8607_AUDIO_Equalizer_D1_1		0x13
#define PM8607_AUDIO_Equalizer_D1_2		0x14
#define PM8607_AUDIO_Side_Tone_1		0x15
#define PM8607_AUDIO_Side_Tone_2		0x16
#define PM8607_AUDIO_Left_Gain1		0x17
#define PM8607_AUDIO_Left_Gain2		0x18
#define PM8607_AUDIO_Right_Gain1		0x19
#define PM8607_AUDIO_Right_Gain2		0x1a
#define PM8607_AUDIO_DWA_OFFSET		0x1b
#define PM8607_AUDIO_OFFSET_LEFT1		0x1c
#define PM8607_AUDIO_OFFSET_LEFT2		0x1d
#define PM8607_AUDIO_OFFSET_RIGHT1		0x1e
#define PM8607_AUDIO_OFFSET_RIGHT2		0x1f
#define PM8607_AUDIO_ADC_ANALOG_PROGRAM1		0x20
#define PM8607_AUDIO_ADC_ANALOG_PROGRAM2		0x21
#define PM8607_AUDIO_ADC_ANALOG_PROGRAM3		0x22
#define PM8607_AUDIO_ADC_ANALOG_PROGRAM4		0x23
#define PM8607_AUDIO_A2A_PATH_PROGRAMMING		0x24
#define PM8607_AUDIO_DAC_HS1_CTRL		0x25
#define PM8607_AUDIO_DAC_HS2_CTRL		0x26
#define PM8607_AUDIO_DAC_LO1_CTRL		0x27
#define PM8607_AUDIO_DAC_LO2_CTRL		0x28
#define PM8607_AUDIO_DAC_EAR_SPKRPHNE_GAIN		0x29
#define PM8607_AUDIO_MISC_AUDIO		0x2a
#define PM8607_AUDIO_AUDIO_SUPPLIES1		0x2b
#define PM8607_AUDIO_AUDIO_SUPPLIES2		0x2c
#define PM8607_AUDIO_ADC_ANALOG_ENABLES		0x2d
#define PM8607_AUDIO_ADC_DIGITAL_ENABLES		0x2e
#define PM8607_AUDIO_DAC_ANALOG_ENABLES	0x2f
#define PM8607_AUDIO_DAC_DIGITAL_ENABLES		0x31
#define PM8607_AUDIO_AUDIO_CAL1		0x32
#define PM8607_AUDIO_AUDIO_CAL2		0x33
#define PM8607_AUDIO_AUDIO_CAL3		0x34
#define PM8607_AUDIO_AUDIO_CAL4		0x35
#define PM8607_AUDIO_AUDIO_CAL5		0x36
#define PM8607_AUDIO_ANALOG_INPUT_SEL1		0x37
#define PM8607_AUDIO_ANALOG_INPUT_SEL2		0x38
#define PM8607_AUDIO_MIC_BUTTON_DETECTION		0x39
#define PM8607_AUDIO_HEADSET_DETECTION		0x3a
#define PM8607_AUDIO_SHORTS		0x3b

#define PM8607_GENERAL_USE	0x3e

/* Misc Registers */
#define PM8607_CHIP_ID			(0x00)
#define PM8607_B0_MISC1			(0x0C)
#define PM8607_LDO1			(0x10)
#define PM8607_DVC3			(0x26)
#define PM8607_A1_MISC1			(0x40)

/* bit definitions of Status Query Interface */
#define PM8607_STATUS_CC		(1 << 3)
#define PM8607_STATUS_PEN		(1 << 4)
#define PM8607_STATUS_HEADSET		(1 << 5)
#define PM8607_STATUS_HOOK		(1 << 6)
#define PM8607_STATUS_MICIN		(1 << 7)
#define PM8607_STATUS_ONKEY		(1 << 8)
#define PM8607_STATUS_EXTON		(1 << 9)
#define PM8607_STATUS_CHG		(1 << 10)
#define PM8607_STATUS_BAT		(1 << 11)
#define PM8607_STATUS_VBUS		(1 << 12)
#define PM8607_STATUS_OV		(1 << 13)

/* bit definitions of BUCK3 */
#define PM8607_BUCK3_DOUBLE		(1 << 6)

/* bit definitions of Misc1 */
#define PM8607_A1_MISC1_PI2C		(1 << 0)
#define PM8607_B0_MISC1_INV_INT		(1 << 0)
#define PM8607_B0_MISC1_INT_CLEAR	(1 << 1)
#define PM8607_B0_MISC1_INT_MASK	(1 << 2)
#define PM8607_B0_MISC1_PI2C		(1 << 3)
#define PM8607_B0_MISC1_RESET		(1 << 6)

/* bits definitions of GPADC */
#define PM8607_GPADC_EN			(1 << 0)
#define PM8607_GPADC_PREBIAS_MASK	(3 << 1)
#define PM8607_GPADC_SLOT_CYCLE_MASK	(3 << 3)	/* slow mode */
#define PM8607_GPADC_OFF_SCALE_MASK	(3 << 5)	/* GP sleep mode */
#define PM8607_GPADC_SW_CAL_MASK	(1 << 7)

#define PM8607_PD_PREBIAS_MASK		(0x1F << 0)
#define PM8607_PD_PRECHG_MASK		(7 << 5)

#define PM8607_DEBOUNCE_REG		0x0A
#define PM8607_DEBOUNCE_PEN_DET(x) (x << 0)
#define PM8607_DEBOUNCE_CHG(x)     (x << 2)
#define PM8607_DEBOUNCE_EXTON(x)   (x << 4)
#define PM8607_DEBOUNCE_ONKEY(x)   (x << 6)

#define PM8607_PEN_DEBOUNCE_MASK	0x03

#define PM8607_MEASOFFTIME1_BAT_DET_EN_A1 	(1 << 0)
#define PM8607_MEASOFFTIME1MEAS_OFFTIME1_A1(x)	(x << 1)
#define PM8607_MEASOFFTIME1MEAS_DOUBLE_TSI_B	(1 << 0)
#define PM8607_MEASOFFTIME1MEAS_EN_SLP_B	(1 << 1)
#define PM8607_MEASOFFTIME1MEAS_OFFTIME1_B(x)	(x << 2)

/* bit define RTC_ALARM_WU */
#define PM8607_RTC_ALARM_WU		(1 << 4)

#define PM860X_TEMP_TINT		(0)
#define PM860X_TEMP_TBAT		(1)

enum levante_ref_gp_and_osc_clients {
        REF_GP_NO_CLIENTS       = 0,
        WLED1_DUTY              = (1<<0),
        WLED2_DUTY              = (1<<1),
        WLED3_DUTY              = (1<<2),
        RGB1_ENABLE             = (1<<3),
        RGB2_ENABLE             = (1<<4),
        RGB2_R_PWM_DUTY         = (1<<5),
        RGB2_G_PWM_DUTY         = (1<<6),
        RGB2_B_PWM_DUTY         = (1<<7),
        RGB1_R_PWM_DUTY         = (1<<8),
        RGB1_G_PWM_DUTY         = (1<<9),
        RGB1_B_PWM_DUTY         = (1<<10),
        LDO_VBR_EN              = (1<<11),
        REF_GP_MAX_CLIENT       = 0xFFFF
};

/**************************
 * customer configuration *
***************************/
/* */
#define ALTERNATE_CHARGER
#define PM8607_GPADC_TBAT PM8607_GPADC1_MEAS1

/* for disabling the use of PM8606*/
/*#define NO_PM8606_CHIP*/

/* for choosing the TBAT GPADC*/
#define PM8607_GPADC_TBAT PM8607_GPADC1_MEAS1

/*set accroding to charger_resources[]*/
#define PM860X_CHARGER_RESOURCES (7)

/* this value should be calculate
 as part of the complete table -
  do not modify only this value!*/
#define PM860X_POWER_OFF 3470

/*********************************
 * end of customer configuration *
**********************************/

/* Interrupt Number in 88PM8607 */
enum {
	PM8607_IRQ_ONKEY,
	PM8607_IRQ_EXTON,
	PM8607_IRQ_CHG,
	PM8607_IRQ_BAT,
	PM8607_IRQ_RTC,
	PM8607_IRQ_CC,
	PM8607_IRQ_VBAT,
	PM8607_IRQ_VCHG,
	PM8607_IRQ_VSYS,
	PM8607_IRQ_TINT,
	PM8607_IRQ_GPADC0,
	PM8607_IRQ_GPADC1,
	PM8607_IRQ_GPADC2,
	PM8607_IRQ_GPADC3,
	PM8607_IRQ_AUDIO_SHORT,
	PM8607_IRQ_PEN,
	PM8607_IRQ_HEADSET,
	PM8607_IRQ_HOOK,
	PM8607_IRQ_MICIN,
	PM8607_IRQ_CHG_FAIL,
	PM8607_IRQ_CHG_DONE,
	PM8607_IRQ_CHG_FAULT,
	PM8607_MAX_IRQ,
};

enum {
	PM8607_CHIP_A0 = 0x40,
	PM8607_CHIP_A1 = 0x41,
	PM8607_CHIP_B0 = 0x48,
	PM8607_CHIP_C0 = 0x50,
	PM8607_CHIP_C1 = 0x51,
	PM8607_CHIP_D0 = 0x58,
	PM8607_CHIP_D1 = 0x59,
	PM8607_CHIP_END = PM8607_CHIP_D1
};

enum enum_result {
	ENUMERATION_START	= 0,
	ENUMERATION_500MA,
};

enum enum_charger_type {
	USB_CHARGER		= 0,
	AC_STANDARD_CHARGER,
	AC_OTHER_CHARGER,
};

struct pm860x_chip {
	struct device		*dev;
	struct mutex		io_lock;
	struct mutex		irq_lock;
	struct i2c_client	*client;
	struct i2c_client	*companion;	/* companion chip client */
	struct workqueue_struct	*monitor_wqueue;

	int			buck3_double;	/* DVC ramp slope double */
	unsigned short		companion_addr;
	int			id;
	int			irq_mode;
	int			irq_base;
	int			core_irq;
	unsigned char		chip_version;

};

#define PM8607_MAX_REGULATOR	PM8607_ID_RG_MAX	/* 3 Bucks, 13 LDOs */

enum {
	GI2C_PORT = 0,
	PI2C_PORT,
};

struct pm860x_backlight_pdata {
	int		id;
	int		pwm;
	int		iset;
	int		flags;
};

struct pm860x_led_pdata {
	int		id;
	int		iset;
	int		flags;
};

struct pm860x_touch_pdata {
	int		gpadc_prebias;
	int		slot_cycle;
	int		off_scale;
	int		sw_cal;
	int		tsi_prebias;	/* time, slot */
	int		pen_prebias;	/* time, slot */
	int		pen_prechg;	/* time, slot */
	int		res_x;		/* resistor of Xplate */
	unsigned long	flags;
};

struct pm860x_rtc_pdata {
	int		vrtc;
	int		rtc_wakeup;
};

struct pm860x_power_pdata {
	void (*disable_rf_fn)(void);/* disable rf for battery calibration */
};

enum {
	PM860X_GPIO1_SUPPLY_VBUS = 1,
	PM860X_GPIO2_SUPPLY_VBUS = 2,
};

enum {
	PM860X_IDPIN_NO_USE = 0,
	PM860X_IDPIN_USE_GPADC0,
	PM860X_IDPIN_USE_GPADC1,
	PM860X_IDPIN_USE_GPADC2,
	PM860X_IDPIN_USE_GPADC3,
};

struct pm860x_vbus_pdata {
	int             supply;
	int             idpin;
	unsigned int    reg_base;	/* Physical address */
	unsigned int    reg_end;	/* Physical address end */
};

struct pm860x_cm3601_pdata
{
	unsigned char gpio_en; /*gpio number*/
	unsigned char gpio_out; /*gpio number*/
	int (*request_source)(unsigned char gpio_num, char *name);
	void (*release_source)(unsigned char gpio_num);
};

struct pm860x_platform_data {
	struct pm860x_backlight_pdata	*backlight;
	struct pm860x_led_pdata		*led;
	struct pm860x_touch_pdata	*touch;
	struct pm860x_rtc_pdata		*rtc;
	struct pm860x_power_pdata	*power;
	struct pm860x_vbus_pdata	*vbus;
	struct pm860x_cm3601_pdata	*cm3601;
	struct regulator_init_data	*regulator;

	unsigned short	companion_addr;	/* I2C address of companion chip */
	int		i2c_port;	/* Controlled by GI2C or PI2C */
	int		irq_mode;	/* Clear interrupt by read/write(0/1) */
	int		irq_base;	/* IRQ base number of 88pm860x */
	int		headset_flag;	/* headset detect flag, 1 for PXA910, 0 for others */
	int 	batt_det;	/* battery detect is enabled , 0 - disable for PXA910 , 1 enable*/
	int		num_regulators;
	int	(*fixup)(struct pm860x_chip *, struct pm860x_platform_data *);
};
extern int pm8607_irq_base;

extern int pm8606_ref_gp_and_osc_get(struct pm860x_chip *, u16);
extern int pm8606_ref_gp_and_osc_release(struct pm860x_chip *, u16);

extern char pm860x_backlight_name[][MFD_NAME_SIZE];
extern char pm860x_led_name[][MFD_NAME_SIZE];

typedef int (*pm860xStatusCbFunc)(void);
typedef int (*pm860xHealthCbFunc)(void); 
typedef int (*pm860xCapacityCbFunc)(void);
typedef int (*pm860x_control_cb_func)(int);
typedef irqreturn_t (*pm860xCoulombCbFunc)(int irq, void *data); 
typedef irqreturn_t (*pm860xBattDetCbFunc)(int irq, void *data); 
typedef irqreturn_t (*pm860xChargerEventsCbFunc)(int irq, void *data);
typedef irqreturn_t (*pm860xVbusCbFunc)(int irq, void *data); 

#ifdef ALTERNATE_CHARGER
extern int pm860x_registerStatusCb(pm860xStatusCbFunc callBack);
extern int pm860x_registerHealthCb(pm860xHealthCbFunc callBack);
extern int pm860x_registerCapacityCb(pm860xCapacityCbFunc callBack);
extern int pm860x_registerCoulombCb(pm860xCoulombCbFunc callBack);
extern int pm860x_registerBattDetCb(pm860xBattDetCbFunc callBack);
extern int pm860x_registerChargerEventsCb(pm860xChargerEventsCbFunc callBack,int event);
extern int pm860x_registerVbusCb(pm860xVbusCbFunc callBack);
extern void pm860x_set_vbatt_threshold(int min, int max);
extern void pm860x_set_vchg_threshold(int min, int max);
extern void pm860x_set_temp_threshold(int min, int max);
extern void pm860x_read_vbat(int *vbat);
extern void pm860x_read_tbat_adc(int gpadc, int *tabt);
extern int pm860x_set_general_user(unsigned char data);
extern void pm860x_get_general_user(int *data);
#endif

extern int pm860x_reg_read(struct i2c_client *, int);
extern int pm860x_reg_write(struct i2c_client *, int, unsigned char);
extern int pm860x_codec_reg_read(int reg);
extern int pm860x_codec_reg_write(int reg, unsigned char data);
extern int pm860x_codec_reg_set_bits(int reg, unsigned char mask, unsigned char data);
extern int pm860x_bulk_read(struct i2c_client *, int, int, unsigned char *);
extern int pm860x_bulk_write(struct i2c_client *, int, int, unsigned char *);
extern int pm860x_set_bits(struct i2c_client *, int, unsigned char,
			   unsigned char);
extern int pm860x_page_reg_read(struct i2c_client *, int);
extern int pm860x_page_reg_write(struct i2c_client *, int, unsigned char);
extern int pm860x_page_bulk_read(struct i2c_client *, int, int,
			unsigned char *);
extern int pm860x_page_bulk_write(struct i2c_client *, int, int,
			unsigned char *);
extern int pm860x_page_set_bits(struct i2c_client *, int, unsigned char,
			unsigned char);

extern int pm860x_device_init(struct pm860x_chip *chip,
			      struct pm860x_platform_data *pdata) __devinit ;
extern void pm860x_device_exit(struct pm860x_chip *chip) __devexit ;

extern int pm860x_calc_resistor(void);
extern void pm860x_system_poweroff(void);
extern void pm860x_set_charger_type(enum enum_charger_type type );
extern int pm860x_battery_update_soc(void);
extern void pm860x_set_vbus_output(int);

#if defined(CONFIG_RTC_NVM_88PM860X)
extern int rtc_hctosys(void);
#endif

#endif /* __LINUX_MFD_88PM860X_H */
