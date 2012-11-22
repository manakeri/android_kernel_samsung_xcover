/*
 * Base driver for Marvell 88PM8607
 *
 * Copyright (C) 2009 Marvell International Ltd.
 *	Haojian Zhuang <haojian.zhuang@marvell.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/mfd/core.h>
#include <linux/mfd/88pm860x.h>
#include <linux/switch.h>
#include <linux/regulator/machine.h>

#define INT_STATUS_NUM			3

int pm8607_irq_base;

char pm860x_backlight_name[][MFD_NAME_SIZE] = {
	"backlight-0",
	"backlight-1",
	"backlight-2",
};
EXPORT_SYMBOL(pm860x_backlight_name);

char pm860x_led_name[][MFD_NAME_SIZE] = {
	"led0-red",
	"led0-green",
	"led0-blue",
	"led1-red",
	"led1-green",
	"led1-blue",
};
EXPORT_SYMBOL(pm860x_led_name);

#define PM8606_BACKLIGHT_RESOURCE(_i, _x)		\
{							\
	.name	= pm860x_backlight_name[_i],		\
	.start	= PM8606_##_x,				\
	.end	= PM8606_##_x,				\
	.flags	= IORESOURCE_IO,			\
}

static struct resource backlight_resources[] = {
	PM8606_BACKLIGHT_RESOURCE(PM8606_BACKLIGHT1, WLED1A),
	PM8606_BACKLIGHT_RESOURCE(PM8606_BACKLIGHT2, WLED2A),
	PM8606_BACKLIGHT_RESOURCE(PM8606_BACKLIGHT3, WLED3A),
};

#define PM8606_BACKLIGHT_DEVS(_i)			\
{							\
	.name		= "88pm860x-backlight",		\
	.num_resources	= 1,				\
	.resources	= &backlight_resources[_i],	\
	.id		= _i,				\
}

static struct mfd_cell backlight_devs[] = {
	PM8606_BACKLIGHT_DEVS(PM8606_BACKLIGHT1),
	PM8606_BACKLIGHT_DEVS(PM8606_BACKLIGHT2),
	PM8606_BACKLIGHT_DEVS(PM8606_BACKLIGHT3),
};


static struct resource vibrator_resources[] = {
	{
		.name		= "android-vibrator",
		.start		= PM8606_VIBRATORA,
		.end			= PM8606_VIBRATORB,
		.flags		= IORESOURCE_IO,
	},
};

static struct mfd_cell vibrator_devs[] = {
	{
		.name		= "android-vibrator",
		.num_resources	= 1,
		.resources	= &vibrator_resources[0],
		.id		= -1,
	},
};


#define PM8606_LED_RESOURCE(_i, _x)			\
{							\
	.name	= pm860x_led_name[_i],			\
	.start	= PM8606_##_x,				\
	.end	= PM8606_##_x,				\
	.flags	= IORESOURCE_IO,			\
}

static struct resource led_resources[] = {
	PM8606_LED_RESOURCE(PM8606_LED1_RED, RGB1B),
	PM8606_LED_RESOURCE(PM8606_LED1_GREEN, RGB1C),
	PM8606_LED_RESOURCE(PM8606_LED1_BLUE, RGB1D),
	PM8606_LED_RESOURCE(PM8606_LED2_RED, RGB2B),
	PM8606_LED_RESOURCE(PM8606_LED2_GREEN, RGB2C),
	PM8606_LED_RESOURCE(PM8606_LED2_BLUE, RGB2D),
};

#define PM8606_LED_DEVS(_i)				\
{							\
	.name		= "88pm860x-led",		\
	.num_resources	= 1,				\
	.resources	= &led_resources[_i],		\
	.id		= _i,				\
}

static struct mfd_cell led_devs[] = {
	PM8606_LED_DEVS(PM8606_LED1_RED),
	PM8606_LED_DEVS(PM8606_LED1_GREEN),
	PM8606_LED_DEVS(PM8606_LED1_BLUE),
	PM8606_LED_DEVS(PM8606_LED2_RED),
	PM8606_LED_DEVS(PM8606_LED2_GREEN),
	PM8606_LED_DEVS(PM8606_LED2_BLUE),
};

static struct resource touch_resources[] = {
	{
		.start	= PM8607_IRQ_PEN,
		.end	= PM8607_IRQ_PEN,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct mfd_cell touch_devs[] = {
	{
		.name		= "88pm860x-touch",
		.num_resources	= 1,
		.resources	= &touch_resources[0],
	},
};

#define PM8607_REG_RESOURCE(_start, _end)		\
{							\
	.start	= PM8607_##_start,			\
	.end	= PM8607_##_end,			\
	.flags	= IORESOURCE_IO,			\
}

static struct resource battery_resources[] = {
	{
		.name		= "88pm860x-battery",
		.start		= PM8607_IRQ_CC,
		.end		= PM8607_IRQ_CC,
		.flags		= IORESOURCE_IRQ,
	}, {
		.name		= "88pm860x-battery",
		.start		= PM8607_IRQ_BAT,
		.end		= PM8607_IRQ_BAT,
		.flags		= IORESOURCE_IRQ,
	},
};

static struct resource charger_resources[] = {
	{
		.name		= "charger detect",
		.start		= PM8607_IRQ_CHG,
		.end		= PM8607_IRQ_CHG,
		.flags		= IORESOURCE_IRQ,
	}, {
		.name		= "charging done",
		.start		= PM8607_IRQ_CHG_DONE,
		.end		= PM8607_IRQ_CHG_DONE,
		.flags		= IORESOURCE_IRQ,
	}, {
		.name		= "charging timeout",
		.start		= PM8607_IRQ_CHG_FAIL,
		.end		= PM8607_IRQ_CHG_FAIL,
		.flags		= IORESOURCE_IRQ,
	}, {
		.name		= "charging fault",
		.start		= PM8607_IRQ_CHG_FAULT,
		.end		= PM8607_IRQ_CHG_FAULT,
		.flags		= IORESOURCE_IRQ,
	}, {
		.name		= "battery temperature",
		.start		= PM8607_IRQ_GPADC1,
		.end		= PM8607_IRQ_GPADC1,
		.flags		= IORESOURCE_IRQ,
	}, {
		.name		= "battery voltage",
		.start		= PM8607_IRQ_VBAT,
		.end		= PM8607_IRQ_VBAT,
		.flags		= IORESOURCE_IRQ,
	}, {
		.name		= "vchg voltage",
		.start		= PM8607_IRQ_VCHG,
		.end		= PM8607_IRQ_VCHG,
		.flags		= IORESOURCE_IRQ,
	},
};

static struct mfd_cell power_devs[] = {
	{
		.name		= "88pm860x-battery",
		.num_resources	= ARRAY_SIZE(battery_resources),
		.resources	= &battery_resources[0],
		.id		= 0,
	}, {
		.name		= "88pm860x-charger",
		.num_resources	= ARRAY_SIZE(charger_resources),
		.resources	= &charger_resources[0],
		.id		= 1,
	},
};

static struct resource rtc_resources[] = {
	{
		.name		= "88pm860x-rtc",
		.start		= PM8607_IRQ_RTC,
		.end		= PM8607_IRQ_RTC,
		.flags		= IORESOURCE_IRQ,
	},
};

static struct mfd_cell rtc_devs[] = {
	{
		.name		= "88pm860x-rtc",
		.num_resources	= ARRAY_SIZE(rtc_resources),
		.resources	= &rtc_resources[0],
		.id		= -1,
	},
};

static struct resource vbus_resources[] = {
	{
		.name		= "88pm860x-vbus",
		.start		= PM8607_IRQ_CHG,
		.end		= PM8607_IRQ_CHG,
		.flags		= IORESOURCE_IRQ,
	},
};

static struct mfd_cell vbus_devs[] = {
	{
		.name		= "88pm860x-vbus",
		.num_resources	= 1,
		.resources	= &vbus_resources[0],
		.id		= -1,
	},
};

static struct resource onkey_resources[] = {
	{
		.name		= "88pm860x-onkey",
		.start		= PM8607_IRQ_ONKEY,
		.end		= PM8607_IRQ_ONKEY,
		.flags		= IORESOURCE_IRQ,
	},
};

static struct mfd_cell onkey_devs[] = {
	{
		.name		= "88pm860x-onkey",
		.num_resources	= 1,
		.resources	= &onkey_resources[0],
		.id		= -1,
	},
};

static struct resource cm3601_resources[] = {
	{
		.name = "cm3601",
	},
};

static struct mfd_cell cm3601_devs[] = {
	{
		.name		= "cm3601",
		.num_resources	= 1,
		.resources	= &cm3601_resources[0],
		.id		= -1,
	},
};

static struct resource codec_resources[] = {
	{
		/* Headset microphone insertion or removal */
		.name		= "micin",
		.start		= PM8607_IRQ_MICIN,
		.end		= PM8607_IRQ_MICIN,
		.flags		= IORESOURCE_IRQ,
	}, {
		/* Audio short */
		.name		= "audio-short",
		.start		= PM8607_IRQ_AUDIO_SHORT,
		.end		= PM8607_IRQ_AUDIO_SHORT,
		.flags		= IORESOURCE_IRQ,
	},
};

static struct resource headset_resources[] = {
	{
		/* Headset insertion or removal */
		.name		= "headset",
		.start		= PM8607_IRQ_HEADSET,
		.end		= PM8607_IRQ_HEADSET,
		.flags		= IORESOURCE_IRQ,
	}, {
		/* Hook-switch press or release */
		.name		= "hook",
		.start		= PM8607_IRQ_HOOK,
		.end		= PM8607_IRQ_HOOK,
		.flags		= IORESOURCE_IRQ,
	},
};

static struct gpio_switch_platform_data headset_switch_device_data[] = {
	{
		/* headset switch */
		.name = "h2w",
		.gpio = 0,
		.name_on = NULL,
		.name_off = NULL,
		.state_on = NULL,
		.state_off = NULL,
	}, {
		/* hook switch */
		.name = "h3w",
		.gpio = 0,
		.name_on = NULL,
		.name_off = NULL,
		.state_on = NULL,
		.state_off = NULL,
	},
};


static struct mfd_cell codec_devs[] = {
	{
		.name		= "88pm860x-codec",
		.num_resources	= ARRAY_SIZE(codec_resources),
		.resources	= &codec_resources[0],
		.id		= -1,
	},
};

static struct mfd_cell headset_devs[] = {
	{
		.name		= "88pm860x-headset",
		.num_resources	= ARRAY_SIZE(headset_resources),
		.resources	= &headset_resources[0],
		.id		= -1,
		.platform_data	= headset_switch_device_data,
		.data_size = sizeof(headset_switch_device_data),
	},
};

static struct resource regulator_resources[] = {
	{PM8607_ID_BUCK1, PM8607_ID_BUCK1, "buck-1", IORESOURCE_IO,},
	{PM8607_ID_BUCK2, PM8607_ID_BUCK2, "buck-2", IORESOURCE_IO,},
	{PM8607_ID_BUCK3, PM8607_ID_BUCK3, "buck-3", IORESOURCE_IO,},
	{PM8607_ID_LDO1,  PM8607_ID_LDO1,  "ldo-01", IORESOURCE_IO,},
	{PM8607_ID_LDO2,  PM8607_ID_LDO2,  "ldo-02", IORESOURCE_IO,},
	{PM8607_ID_LDO3,  PM8607_ID_LDO3,  "ldo-03", IORESOURCE_IO,},
	{PM8607_ID_LDO4,  PM8607_ID_LDO4,  "ldo-04", IORESOURCE_IO,},
	{PM8607_ID_LDO5,  PM8607_ID_LDO5,  "ldo-05", IORESOURCE_IO,},
	{PM8607_ID_LDO6,  PM8607_ID_LDO6,  "ldo-06", IORESOURCE_IO,},
	{PM8607_ID_LDO7,  PM8607_ID_LDO7,  "ldo-07", IORESOURCE_IO,},
	{PM8607_ID_LDO8,  PM8607_ID_LDO8,  "ldo-08", IORESOURCE_IO,},
	{PM8607_ID_LDO9,  PM8607_ID_LDO9,  "ldo-09", IORESOURCE_IO,},
	{PM8607_ID_LDO10, PM8607_ID_LDO10, "ldo-10", IORESOURCE_IO,},
	{PM8607_ID_LDO11, PM8607_ID_LDO11, "ldo-11", IORESOURCE_IO,},
	{PM8607_ID_LDO12, PM8607_ID_LDO12, "ldo-12", IORESOURCE_IO,},
	{PM8607_ID_LDO13, PM8607_ID_LDO13, "ldo-13", IORESOURCE_IO,},
	{PM8607_ID_LDO14, PM8607_ID_LDO14, "ldo-14", IORESOURCE_IO,},
	{PM8607_ID_LDO15, PM8607_ID_LDO15, "ldo-15", IORESOURCE_IO,},
};

static struct mfd_cell regulator_devs[] = {
	{"88pm860x-regulator", 0,},
	{"88pm860x-regulator", 1,},
	{"88pm860x-regulator", 2,},
	{"88pm860x-regulator", 3,},
	{"88pm860x-regulator", 4,},
	{"88pm860x-regulator", 5,},
	{"88pm860x-regulator", 6,},
	{"88pm860x-regulator", 7,},
	{"88pm860x-regulator", 8,},
	{"88pm860x-regulator", 9,},
	{"88pm860x-regulator", 10,},
	{"88pm860x-regulator", 11,},
	{"88pm860x-regulator", 12,},
	{"88pm860x-regulator", 13,},
	{"88pm860x-regulator", 14,},
	{"88pm860x-regulator", 15,},
	{"88pm860x-regulator", 16,},
	{"88pm860x-regulator", 17,},
};

static struct regulator_init_data regulator_pdata[ARRAY_SIZE(regulator_devs)];

struct pm860x_irq_data {
	int	reg;
	int	mask_reg;
	int	enable;		/* enable or not */
	int	offs;		/* bit offset in mask register */
};

static struct pm860x_irq_data pm860x_irqs[] = {
	[PM8607_IRQ_ONKEY] = {
		.reg		= PM8607_INT_STATUS1,
		.mask_reg	= PM8607_INT_MASK_1,
		.offs		= 1 << 0,
	},
	[PM8607_IRQ_EXTON] = {
		.reg		= PM8607_INT_STATUS1,
		.mask_reg	= PM8607_INT_MASK_1,
		.offs		= 1 << 1,
	},
	[PM8607_IRQ_CHG] = {
		.reg		= PM8607_INT_STATUS1,
		.mask_reg	= PM8607_INT_MASK_1,
		.offs		= 1 << 2,
	},
	[PM8607_IRQ_BAT] = {
		.reg		= PM8607_INT_STATUS1,
		.mask_reg	= PM8607_INT_MASK_1,
		.offs		= 1 << 3,
	},
	[PM8607_IRQ_RTC] = {
		.reg		= PM8607_INT_STATUS1,
		.mask_reg	= PM8607_INT_MASK_1,
		.offs		= 1 << 4,
	},
	[PM8607_IRQ_CC] = {
		.reg		= PM8607_INT_STATUS1,
		.mask_reg	= PM8607_INT_MASK_1,
		.offs		= 1 << 5,
	},
	[PM8607_IRQ_VBAT] = {
		.reg		= PM8607_INT_STATUS2,
		.mask_reg	= PM8607_INT_MASK_2,
		.offs		= 1 << 0,
	},
	[PM8607_IRQ_VCHG] = {
		.reg		= PM8607_INT_STATUS2,
		.mask_reg	= PM8607_INT_MASK_2,
		.offs		= 1 << 1,
	},
	[PM8607_IRQ_VSYS] = {
		.reg		= PM8607_INT_STATUS2,
		.mask_reg	= PM8607_INT_MASK_2,
		.offs		= 1 << 2,
	},
	[PM8607_IRQ_TINT] = {
		.reg		= PM8607_INT_STATUS2,
		.mask_reg	= PM8607_INT_MASK_2,
		.offs		= 1 << 3,
	},
	[PM8607_IRQ_GPADC0] = {
		.reg		= PM8607_INT_STATUS2,
		.mask_reg	= PM8607_INT_MASK_2,
		.offs		= 1 << 4,
	},
	[PM8607_IRQ_GPADC1] = {
		.reg		= PM8607_INT_STATUS2,
		.mask_reg	= PM8607_INT_MASK_2,
		.offs		= 1 << 5,
	},
	[PM8607_IRQ_GPADC2] = {
		.reg		= PM8607_INT_STATUS2,
		.mask_reg	= PM8607_INT_MASK_2,
		.offs		= 1 << 6,
	},
	[PM8607_IRQ_GPADC3] = {
		.reg		= PM8607_INT_STATUS2,
		.mask_reg	= PM8607_INT_MASK_2,
		.offs		= 1 << 7,
	},
	[PM8607_IRQ_AUDIO_SHORT] = {
		.reg		= PM8607_INT_STATUS3,
		.mask_reg	= PM8607_INT_MASK_3,
		.offs		= 1 << 0,
	},
	[PM8607_IRQ_PEN] = {
		.reg		= PM8607_INT_STATUS3,
		.mask_reg	= PM8607_INT_MASK_3,
		.offs		= 1 << 1,
	},
	[PM8607_IRQ_HEADSET] = {
		.reg		= PM8607_INT_STATUS3,
		.mask_reg	= PM8607_INT_MASK_3,
		.offs		= 1 << 2,
	},
	[PM8607_IRQ_HOOK] = {
		.reg		= PM8607_INT_STATUS3,
		.mask_reg	= PM8607_INT_MASK_3,
		.offs		= 1 << 3,
	},
	[PM8607_IRQ_MICIN] = {
		.reg		= PM8607_INT_STATUS3,
		.mask_reg	= PM8607_INT_MASK_3,
		.offs		= 1 << 4,
	},
	[PM8607_IRQ_CHG_FAIL] = {
		.reg		= PM8607_INT_STATUS3,
		.mask_reg	= PM8607_INT_MASK_3,
		.offs		= 1 << 5,
	},
	[PM8607_IRQ_CHG_DONE] = {
		.reg		= PM8607_INT_STATUS3,
		.mask_reg	= PM8607_INT_MASK_3,
		.offs		= 1 << 6,
	},
	[PM8607_IRQ_CHG_FAULT] = {
		.reg		= PM8607_INT_STATUS3,
		.mask_reg	= PM8607_INT_MASK_3,
		.offs		= 1 << 7,
	},
};

static inline struct pm860x_irq_data *irq_to_pm860x(struct pm860x_chip *chip,
						    int irq)
{
	return &pm860x_irqs[irq - chip->irq_base];
}

static irqreturn_t pm860x_irq(int irq, void *data)
{
	struct pm860x_chip *chip = data;
	struct pm860x_irq_data *irq_data;
	struct i2c_client *i2c;
	int read_reg = -1, value = 0;
	int i;

	i2c = (chip->id == CHIP_PM8607) ? chip->client : chip->companion;
	for (i = 0; i < ARRAY_SIZE(pm860x_irqs); i++) {
		irq_data = &pm860x_irqs[i];
		if (read_reg != irq_data->reg) {
			read_reg = irq_data->reg;
			value = pm860x_reg_read(i2c, irq_data->reg);
		}
		if (value & irq_data->enable)
			handle_nested_irq(chip->irq_base + i);
	}
	return IRQ_HANDLED;
}

static void pm860x_irq_lock(unsigned int irq)
{
	struct pm860x_chip *chip = get_irq_chip_data(irq);

	mutex_lock(&chip->irq_lock);
}

static void pm860x_irq_sync_unlock(unsigned int irq)
{
	struct pm860x_chip *chip = get_irq_chip_data(irq);
	struct pm860x_irq_data *irq_data;
	struct i2c_client *i2c;
	static unsigned char cached[3] = {0x0, 0x0, 0x0};
	unsigned char mask[3];
	int i;

	i2c = (chip->id == CHIP_PM8607) ? chip->client : chip->companion;
	/* Load cached value. In initial, all IRQs are masked */
	for (i = 0; i < 3; i++)
		mask[i] = cached[i];
	for (i = 0; i < ARRAY_SIZE(pm860x_irqs); i++) {
		irq_data = &pm860x_irqs[i];
		switch (irq_data->mask_reg) {
		case PM8607_INT_MASK_1:
			mask[0] &= ~irq_data->offs;
			mask[0] |= irq_data->enable;
			break;
		case PM8607_INT_MASK_2:
			mask[1] &= ~irq_data->offs;
			mask[1] |= irq_data->enable;
			break;
		case PM8607_INT_MASK_3:
			mask[2] &= ~irq_data->offs;
			mask[2] |= irq_data->enable;
			break;
		default:
			dev_err(chip->dev, "wrong IRQ\n");
			break;
		}
	}
	/* update mask into registers */
	for (i = 0; i < 3; i++) {
		if (mask[i] != cached[i]) {
			cached[i] = mask[i];
			pm860x_reg_write(i2c, PM8607_INT_MASK_1 + i, mask[i]);
		}
	}

	mutex_unlock(&chip->irq_lock);
}

static void pm860x_irq_enable(unsigned int irq)
{
	struct pm860x_chip *chip = get_irq_chip_data(irq);
	pm860x_irqs[irq - chip->irq_base].enable
		= pm860x_irqs[irq - chip->irq_base].offs;
}

static void pm860x_irq_disable(unsigned int irq)
{
	struct pm860x_chip *chip = get_irq_chip_data(irq);
	pm860x_irqs[irq - chip->irq_base].enable = 0;
}

static struct irq_chip pm860x_irq_chip = {
	.name		= "88pm860x",
	.bus_lock	= pm860x_irq_lock,
	.bus_sync_unlock = pm860x_irq_sync_unlock,
	.enable		= pm860x_irq_enable,
	.disable	= pm860x_irq_disable,
};

static int __devinit device_gpadc_init(struct pm860x_chip *chip,
				       struct pm860x_platform_data *pdata)
{
	struct i2c_client *i2c = (chip->id == CHIP_PM8607) ? chip->client \
				: chip->companion;
	int use_gpadc = 0, data, ret = 0;

	/* initialize GPADC without activating it */

	if (pdata && pdata->touch) {
		/* set GPADC MISC1 register */
		data = 0;
		data |= (pdata->touch->gpadc_prebias << 1)
			& PM8607_GPADC_PREBIAS_MASK;
		data |= (pdata->touch->slot_cycle << 3)
			& PM8607_GPADC_SLOT_CYCLE_MASK;
		data |= (pdata->touch->off_scale << 5)
			& PM8607_GPADC_OFF_SCALE_MASK;
		data |= (pdata->touch->sw_cal << 7)
			& PM8607_GPADC_SW_CAL_MASK;
		if (data) {
			ret = pm860x_reg_write(i2c, PM8607_GPADC_MISC1, data);
			if (ret < 0)
				goto out;
		}
		/* set tsi prebias time */
		if (pdata->touch->tsi_prebias) {
			data = pdata->touch->tsi_prebias;
			ret = pm860x_reg_write(i2c, PM8607_TSI_PREBIAS, data);
			if (ret < 0)
				goto out;
		}
		/* set prebias & prechg time of pen detect */
		data = 0;
		data |= pdata->touch->pen_prebias & PM8607_PD_PREBIAS_MASK;
		data |= (pdata->touch->pen_prechg << 5)
			& PM8607_PD_PRECHG_MASK;
		if (data) {
			ret = pm860x_reg_write(i2c, PM8607_PD_PREBIAS, data);
			if (ret < 0)
				goto out;
		}

		use_gpadc = 1;
	}

	/* turn on GPADC */
	if (use_gpadc) {
		ret = pm860x_set_bits(i2c, PM8607_GPADC_MISC1,
					PM8607_GPADC_EN, PM8607_GPADC_EN);
	}

	/*
	This function configures the ADC as requires for
	CP implementation.CP does not "own" the ADC configuration
	registers and relies on AP.
	Reason: enable automatic ADC measurements needed
	for CP to get VBAT and RF temperature readings.
	*/
	data = pm860x_reg_read(i2c, PM8607_MEAS_EN1);
	/* Enable auto ADC (RF temprature, and VBAT)*/
	data |= (PM8607_MEAS_EN1_TINT | PM8607_MEAS_EN1_RFTMP);
	pm860x_reg_write(i2c, PM8607_MEAS_EN1, data);

	data = 0 ;
	data = (PM8607_GPADC_MISC1_GPFSM_EN				|
			PM8607_GPPADC_GP_PREBIAS_TIME(0x01)		|
			PM8607_GPPADC_SLOW_MODE(0x01));
	pm860x_reg_write(i2c, PM8607_GPADC_MISC1, data);

	/* set for Temprature mesurmants*/
	data = 0 ;
	
	if (pdata && (pdata->batt_det == 0) ) {
		data = (PM8607_GPADC_GP_BIAS_EN0		|
					PM8607_GPADC_GP_BIAS_EN1	|
					PM8607_GPADC_GP_BIAS_EN2	|
					PM8607_GPADC_GP_BIAS_EN3);
	} else { 
		data = (PM8607_GPADC_GP_BIAS_EN0		|
					PM8607_GPADC_GP_BIAS_EN2	|
					PM8607_GPADC_GP_BIAS_EN3);	
	}
	pm860x_reg_write(i2c, PM8607_GPADC_MISC2, data);
	pr_info( \
	"device_gpadc_init:initialize GPADC!!!\n");
out:
	return ret;
}

static int __devinit device_irq_init(struct pm860x_chip *chip,
				     struct pm860x_platform_data *pdata)
{
	struct i2c_client *i2c = (chip->id == CHIP_PM8607) ? chip->client \
				: chip->companion;
	unsigned char status_buf[INT_STATUS_NUM];
	unsigned long flags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT;
	struct irq_desc *desc;
	int i, data, mask, ret = -EINVAL;
	int __irq;

	if (!pdata || !pdata->irq_base) {
		dev_warn(chip->dev, "No interrupt support on IRQ base\n");
		return -EINVAL;
	}

	mask = PM8607_B0_MISC1_INV_INT | PM8607_B0_MISC1_INT_CLEAR
		| PM8607_B0_MISC1_INT_MASK;
	data = 0;
	chip->irq_mode = 0;
	if (pdata && pdata->irq_mode) {
		/*
		 * irq_mode defines the way of clearing interrupt. If it's 1,
		 * clear IRQ by write. Otherwise, clear it by read.
		 * This control bit is valid from 88PM8607 B0 steping.
		 */
		data |= PM8607_B0_MISC1_INT_CLEAR;
		chip->irq_mode = 1;
	}
	ret = pm860x_set_bits(i2c, PM8607_B0_MISC1, mask, data);
	if (ret < 0)
		goto out;

	/* mask all IRQs */
	memset(status_buf, 0, INT_STATUS_NUM);
	ret = pm860x_bulk_write(i2c, PM8607_INT_MASK_1,
				INT_STATUS_NUM, status_buf);
	if (ret < 0)
		goto out;

	if (chip->irq_mode) {
		/* clear interrupt status by write */
		memset(status_buf, 0xFF, INT_STATUS_NUM);
		ret = pm860x_bulk_write(i2c, PM8607_INT_STATUS1,
					INT_STATUS_NUM, status_buf);
	} else {
		/* clear interrupt status by read */
		ret = pm860x_bulk_read(i2c, PM8607_INT_STATUS1,
					INT_STATUS_NUM, status_buf);
	}
	if (ret < 0)
		goto out;

	mutex_init(&chip->irq_lock);
	chip->irq_base = pdata->irq_base;
	chip->core_irq = i2c->irq;
	pm8607_irq_base = chip->irq_base;
	if (!chip->core_irq)
		goto out;

	desc = irq_to_desc(chip->core_irq);
	pm860x_irq_chip.set_wake = desc->chip->set_wake;

	/* register IRQ by genirq */
	for (i = 0; i < ARRAY_SIZE(pm860x_irqs); i++) {
		__irq = i + chip->irq_base;
		set_irq_chip_data(__irq, chip);
		set_irq_chip_and_handler(__irq, &pm860x_irq_chip,
					 handle_edge_irq);
		set_irq_nested_thread(__irq, 1);
#ifdef CONFIG_ARM
		set_irq_flags(__irq, IRQF_VALID);
#else
		set_irq_noprobe(__irq);
#endif
	}

	ret = request_threaded_irq(chip->core_irq, NULL, pm860x_irq, flags,
				   "88pm860x", chip);
	if (ret) {
		dev_err(chip->dev, "Failed to request IRQ: %d\n", ret);
		chip->core_irq = 0;
	}

	return 0;
out:
	chip->core_irq = 0;
	return ret;
}

static void device_irq_exit(struct pm860x_chip *chip)
{
	if (chip->core_irq)
		free_irq(chip->core_irq, chip);
}

static u16 pm8606_ref_gp_and_osc_vote;
static u8 pm8606_ref_gp_and_osc_status;
#define PM8606_REF_GP_OSC_OFF         0
#define PM8606_REF_GP_OSC_ON          1
#define PM8606_REF_GP_OSC_UNKNOWN     2
static u8 pm8606_not_used;

int pm8606_ref_gp_and_osc_get(struct pm860x_chip *chip, u16 client)
{
	int ret = -EIO;
	struct i2c_client *i2c = (chip->id == CHIP_PM8606) ?
		chip->client : chip->companion;

	dev_dbg(chip->dev, "%s(B): client=0x%x\n", __func__, client);
	dev_dbg(chip->dev, "%s(B): vote=0x%x status=%d\n",
			__func__, pm8606_ref_gp_and_osc_vote,
			pm8606_ref_gp_and_osc_status);

	/* Update voting status */
	pm8606_ref_gp_and_osc_vote |= client;
	/* If reference group is off - turn on*/
	if ((pm8606_ref_gp_and_osc_status != PM8606_REF_GP_OSC_ON) &&
		!pm8606_not_used) {
		do {
			pm8606_ref_gp_and_osc_status =
				PM8606_REF_GP_OSC_UNKNOWN;

			/* Enable Reference group Vsys */
			if (pm860x_set_bits(i2c, PM8606_VSYS,
					PM8606_VSYS_EN, PM8606_VSYS_EN))
				break;
			/*Enable Internal Oscillator */
			if (pm860x_set_bits(i2c, PM8606_MISC,
					PM8606_MISC_OSC_EN, PM8606_MISC_OSC_EN))
				break;

			/* Update status (only if writes succeed) */
			pm8606_ref_gp_and_osc_status = PM8606_REF_GP_OSC_ON;
			ret = 0;
		} while (0);
	} else
		ret = 0;

	dev_dbg(chip->dev, "%s(A): vote=0x%x status=%d ret=%d\n",
			__func__, pm8606_ref_gp_and_osc_vote,
			pm8606_ref_gp_and_osc_status, ret);
	return ret;
}

int pm8606_ref_gp_and_osc_release(struct pm860x_chip *chip, u16 client)
{
	int ret = -EIO;
	struct i2c_client *i2c = (chip->id == CHIP_PM8606) ?
		chip->client : chip->companion;

	dev_dbg(chip->dev, "%s(B): client=0x%x\n", __func__, client);
	dev_dbg(chip->dev, "%s(B): vote=0x%x status=%d\n",
			__func__, pm8606_ref_gp_and_osc_vote,
			pm8606_ref_gp_and_osc_status);

	/*Update voting status */
	pm8606_ref_gp_and_osc_vote &= ~(client);
	/* If reference group is off and this is the last client to release
	 * - turn off */
	if ((pm8606_ref_gp_and_osc_status != PM8606_REF_GP_OSC_OFF) &&
		(pm8606_ref_gp_and_osc_vote == REF_GP_NO_CLIENTS) &&
		!pm8606_not_used) {
		do {
			pm8606_ref_gp_and_osc_status =
				PM8606_REF_GP_OSC_UNKNOWN;
			/* Disable Reference group Vsys */
			if (pm860x_set_bits(i2c, PM8606_VSYS,
					PM8606_VSYS_EN, 0))
				break;
			/* Disable Internal Oscillator */
			if (pm860x_set_bits(i2c, PM8606_MISC,
					PM8606_MISC_OSC_EN, 0))
				break;
			pm8606_ref_gp_and_osc_status = PM8606_REF_GP_OSC_OFF;
			ret = 0;
		} while (0);
	} else
		ret = 0;

	dev_dbg(chip->dev, "%s(A): vote=0x%x status=%d ret=%d\n",
			__func__, pm8606_ref_gp_and_osc_vote,
			pm8606_ref_gp_and_osc_status, ret);
	return ret;
}

static void __devinit device_8606_init(struct pm860x_chip *chip,
				       struct i2c_client *i2c,
				       struct pm860x_platform_data *pdata)
{
	int ret;

	if (pdata && pdata->backlight) {
		ret = mfd_add_devices(chip->dev, 0, &backlight_devs[0],
				      ARRAY_SIZE(backlight_devs),
				      &backlight_resources[0], 0);
		if (ret < 0) {
			dev_err(chip->dev, "Failed to add backlight "
				"subdev\n");
			goto out_dev;
		}
	}

	ret = mfd_add_devices(chip->dev, 0, &vibrator_devs[0],
			      ARRAY_SIZE(vibrator_devs),
			      &vibrator_resources[0], 0);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to add vibrator subdev\n");
		goto out_dev;
	}

	if (pdata && pdata->led) {
		ret = mfd_add_devices(chip->dev, 0, &led_devs[0],
				      ARRAY_SIZE(led_devs),
				      &led_resources[0], 0);
		if (ret < 0) {
			dev_err(chip->dev, "Failed to add led "
				"subdev\n");
			goto out_dev;
		}
	}

	if (pdata && pdata->vbus) {
		ret = mfd_add_devices(chip->dev, 0, &vbus_devs[0],
				ARRAY_SIZE(vbus_devs),
				&vbus_resources[0], chip->irq_base);
		if (ret < 0) {
			dev_err(chip->dev, "Failed to add vbus subdev\n");
			goto out_dev;
		}
	}
#if defined (CONFIG_MACH_ALKON)
	if (!pdata->touch && !pdata->backlight && !pdata->led) {
		pm8606_not_used = 1;
	} else
#endif
	{
		pm8606_not_used = 0;
		/* init portofino reference group voting and status */
		/* Disable Reference group Vsys */
		pm860x_set_bits(i2c, PM8606_VSYS, PM8606_VSYS_EN, 0);
		/* Disable Internal Oscillator */
		pm860x_set_bits(i2c, PM8606_MISC, PM8606_MISC_OSC_EN, 0);
	}
	pm8606_ref_gp_and_osc_vote = REF_GP_NO_CLIENTS;
	pm8606_ref_gp_and_osc_status = PM8606_REF_GP_OSC_OFF;

	return;
out_dev:
	mfd_remove_devices(chip->dev);
	device_irq_exit(chip);
}

static void __devinit device_regulator_init(struct pm860x_chip *chip,
					    struct i2c_client *i2c,
					    struct pm860x_platform_data *pdata)
{
	struct regulator_init_data *initdata;
	int ret;
	int i, j;

	if ((pdata == NULL) || (pdata->regulator == NULL))
		return;

	if (pdata->num_regulators > ARRAY_SIZE(regulator_devs))
		pdata->num_regulators = ARRAY_SIZE(regulator_devs);

	for (i = 0, j = -1; i < pdata->num_regulators; i++) {
		initdata = &pdata->regulator[i];
		if (strstr(initdata->constraints.name, "BUCK")) {
			sscanf(initdata->constraints.name, "BUCK%d", &j);
			/* BUCK1 ~ BUCK3 */
			if ((j < 1) || (j > 3)) {
				dev_err(chip->dev, "Failed to add constraint "
					"(%s)\n", initdata->constraints.name);
				goto out;
			}
			j = (j - 1) + PM8607_ID_BUCK1;
		}
		if (strstr(initdata->constraints.name, "LDO")) {
			sscanf(initdata->constraints.name, "LDO%d", &j);
			/* LDO1 ~ LDO15 */
			if ((j < 1) || (j > 15)) {
				dev_err(chip->dev, "Failed to add constraint "
					"(%s)\n", initdata->constraints.name);
				goto out;
			}
			j = (j - 1) + PM8607_ID_LDO1;
		}
		if (j == -1) {
			dev_err(chip->dev, "Failed to add constraint (%s)\n",
				initdata->constraints.name);
			goto out;
		}
		memcpy(&regulator_pdata[i], &pdata->regulator[i],
			sizeof(struct regulator_init_data));
		regulator_devs[i].platform_data = &regulator_pdata[i];
		regulator_devs[i].driver_data = &regulator_pdata[i];
		regulator_devs[i].num_resources = 1;
		regulator_devs[i].resources = &regulator_resources[j];

		ret = mfd_add_devices(chip->dev, 0, &regulator_devs[i], 1,
				      &regulator_resources[j], 0);
		if (ret < 0) {
			dev_err(chip->dev, "Failed to add regulator subdev\n");
			goto out;
		}
	}
out:
	return;
}

static void __devinit device_8607_init(struct pm860x_chip *chip,
				       struct i2c_client *i2c,
				       struct pm860x_platform_data *pdata)
{
	int data, ret, pmic_id;

	ret = pm860x_reg_read(i2c, PM8607_CHIP_ID);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to read CHIP ID: %d\n", ret);
		goto out;
	}

	pmic_id = ret & PM8607_VERSION_MASK;

	if  ((pmic_id >= PM8607_CHIP_A0) && (pmic_id <= PM8607_CHIP_END)) {
		chip->chip_version = ret;
		dev_info(chip->dev, "Marvell 88PM8607 (ID: %02x) detected\n",
			 ret);
	} else {
		dev_err(chip->dev, "Failed to detect Marvell 88PM8607:Chip ID: %02x\n",
		ret);
		goto out;
	}

	ret = pm860x_reg_read(i2c, PM8607_RTC1);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to read RTC register: %d\n", ret);
		goto out;
	}
	if (ret & PM8607_RTC_ALARM_WU) {
		if (pdata && pdata->rtc) {
			pdata->rtc->rtc_wakeup = 1;
			rtc_devs[0].platform_data = pdata->rtc;
			rtc_devs[0].data_size = sizeof(pdata->rtc);
		}
	}

	ret = pm860x_reg_read(i2c, PM8607_BUCK3);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to read BUCK3 register: %d\n", ret);
		goto out;
	}
	if (ret & PM8607_BUCK3_DOUBLE)
		chip->buck3_double = 1;

	ret = pm860x_reg_read(i2c, PM8607_B0_MISC1);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to read MISC1 register: %d\n", ret);
		goto out;
	}

	if (pdata && (pdata->i2c_port == PI2C_PORT))
		data = PM8607_B0_MISC1_PI2C;
	else
		data = 0;
	ret = pm860x_set_bits(i2c, PM8607_B0_MISC1, PM8607_B0_MISC1_PI2C, data);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to access MISC1:%d\n", ret);
		goto out;
	}

	ret = device_gpadc_init(chip, pdata);
	if (ret < 0)
		goto out;

	ret = device_irq_init(chip, pdata);
	if (ret < 0)
		goto out;

	chip->monitor_wqueue = create_singlethread_workqueue("88pm860x");
	if (!chip->monitor_wqueue) {
		ret = -ESRCH;
		goto out_work;
	}

	device_regulator_init(chip, i2c, pdata);

	if (pdata && pdata->touch) {
		ret = mfd_add_devices(chip->dev, 0, &touch_devs[0],
				      ARRAY_SIZE(touch_devs),
				      &touch_resources[0], 0);
		if (ret < 0) {
			dev_err(chip->dev, "Failed to add touch "
				"subdev\n");
			goto out_dev;
		}
	}

	ret = mfd_add_devices(chip->dev, 0, &onkey_devs[0],
			      ARRAY_SIZE(onkey_devs),
			      &onkey_resources[0], 0);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to add onkey subdev\n");
		goto out_dev;
	}

	ret = mfd_add_devices(chip->dev, 0, &cm3601_devs[0],
			      ARRAY_SIZE(cm3601_devs),
			      &cm3601_resources[0], 0);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to add cm3601 subdev\n");
		goto out_dev;
	}

	ret = mfd_add_devices(chip->dev, 0, &codec_devs[0],
			      ARRAY_SIZE(codec_devs),
			      &codec_resources[0], 0);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to add codec subdev\n");
		goto out_dev;
	}

	ret = mfd_add_devices(chip->dev, 0, &headset_devs[0],
			      ARRAY_SIZE(headset_devs),
			      &headset_resources[0], 0);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to add headset subdev\n");
		goto out_dev;
	}

	if (pdata && pdata->power) {
		power_devs[0].platform_data = pdata->power;
		power_devs[0].data_size = sizeof(pdata->power);
		ret = mfd_add_devices(chip->dev, 0, &power_devs[0],
				      ARRAY_SIZE(power_devs),
				      NULL, chip->irq_base);
		if (ret < 0) {
			dev_err(chip->dev, "Failed to add power supply "
				"subdev\n");
			goto out_dev;
		}
	}

	ret = mfd_add_devices(chip->dev, 0, &rtc_devs[0],
			      ARRAY_SIZE(rtc_devs),
			      NULL, chip->irq_base);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to add rtc subdev\n");
		goto out_dev;
	}

	return;
out_dev:
	mfd_remove_devices(chip->dev);
	destroy_workqueue(chip->monitor_wqueue);
out_work:
	device_irq_exit(chip);
out:
	return;
}

int __devinit pm860x_device_init(struct pm860x_chip *chip,
		       struct pm860x_platform_data *pdata)
{
	chip->core_irq = 0;

	switch (chip->id) {
	case CHIP_PM8606:
		device_8606_init(chip, chip->client, pdata);
		break;
	case CHIP_PM8607:
		device_8607_init(chip, chip->client, pdata);
		break;
	}

	if (chip->companion) {
		switch (chip->id) {
		case CHIP_PM8607:
			device_8606_init(chip, chip->companion, pdata);
			break;
		case CHIP_PM8606:
			device_8607_init(chip, chip->companion, pdata);
			break;
		}
	}

	return 0;
}

void __devexit pm860x_device_exit(struct pm860x_chip *chip)
{
	flush_workqueue(chip->monitor_wqueue);
	destroy_workqueue(chip->monitor_wqueue);
	device_irq_exit(chip);
	mfd_remove_devices(chip->dev);
}

MODULE_DESCRIPTION("PMIC Driver for Marvell 88PM860x");
MODULE_AUTHOR("Haojian Zhuang <haojian.zhuang@marvell.com>");
MODULE_LICENSE("GPL");
