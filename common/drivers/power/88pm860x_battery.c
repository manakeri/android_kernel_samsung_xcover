/*
 * Battery driver for Marvell 88PM860x PMIC
 *
 * Copyright (c) 2009-2010 Marvell International Ltd.
 * Author:	Jett Zhou <jtzhou@marvell.com>
 *		Haojian Zhuang <haojian.zhuang@marvell.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/string.h>
#include <linux/power_supply.h>
#include <linux/mfd/88pm860x.h>
#include <linux/delay.h>

#include <mach/spa.h>

#define VBAT_CALIB_OTHER

/* bit definitions of Status Query Interface 2 */
#define STATUS2_CHG			(1 << 2)
#define STATUS2_BAT			(1 << 3)
#define STATUS2_VBUS			(1 << 4)

/* bit definitions of Measurement Enable 1 Register */
#define MEAS1_TINT			(1 << 3)
#define MEAS1_GP1			(1 << 5)

/* bit definitions of Measurement Enable 3 Register */
#define MEAS3_IBAT			(1 << 0)
#define MEAS3_BAT_DET			(1 << 1)
#define MEAS3_CC			(1 << 2)

/* bit definitions of Measurement Off Time Register */
#define MEAS_OFF_SLEEP_EN		(1 << 1)

/* bit definitions of GPADC Bias 2 Register */
#define GPBIAS2_GPADC1_15UA		(2 << 4)
#define GPBIAS2_GPADC1_60UA		(0xb << 4)

/* bit definitions of GPADC Misc 1 Register */
#define GPMISC1_GPADC_EN		(1 << 0)

/* bit definitions of Charger Control 6 Register */
#define CC6_BAT_DET_GPADC1		1

/* bit definitions of Coulomb Counter Reading Register */
#define CCNT_AVG_SEL			(4 << 3)

/* bit definitions of RTC miscellaneous Register1 */
#define RTC_SOC_5LSB		(0x1F << 3)
/* bit definitions of RTC Register1 */
#define RTC_SOC_3MSB		(0x7)

/* bit definitions of Power up Log register */
#define BAT_WU_LOG			(1<<6)

#define CCNT_POS1			0
#define CCNT_POS2			1
#define CCNT_NEG1			2
#define CCNT_NEG2			3
#define CCNT_SPOS			4
#define CCNT_SNEG			5

/* OCV -- Open Circuit Voltage */
#define OCV_MODE_ACTIVE			0
#define OCV_MODE_SLEEP			1

#define LOW_BAT_THRESHOLD		3600
#define VBATT_RESISTOR_MIN		3800
#define VBATT_RESISTOR_MAX		4100

#define MONITOR_INTERVAL		(HZ * 100)

/* tbat measurement*/

enum tbat_curr_mode {
	NOT_SET,
	TBAT_CUR_MODE_LOW,
	TBAT_CUR_MODE_HIGH,
};

/*
 * TBAT Threshold
 * There are 2 possible modes for measuring TBAT. The motivation for the
 * 2 modes is to increase the resolution of measurement in the related range
 * of temperatures. Since measured current of TBAT is different from each other
 * in both mode, it had better to have different threshold value.
 */

/* Thresholds for low current mode (15uA)  --  mv / degree */
#define TBAT_LC_NEG_25D			1647	/* -25 */
#define TBAT_LC_NEG_20D			1227	/* -20 */
#define TBAT_LC_NEG_17D			1024	/* -17 */
#define TBAT_LC_NEG_15D			910	/* -15 */
#define TBAT_LC_5D			307	/* 5 */

/* Thresholds for high current mode (60uA) -- mv / degree */
#define TBAT_HC_0D			1589	/* 0 */
#define TBAT_HC_40D			256	/* 40 */
#define TBAT_HC_42D	   		325	/* 42 */
#define TBAT_HC_45D			237	/* 45 */
#define TBAT_HC_55D			145	/* 55 */
#define TBAT_HC_57D	  		135	/* 57 */
#define TBAT_HC_60D			121	/* 60 */
#define TBAT_HC_100D			34	/* 100 */

/* If GPADC1 bias current is different, thresholds are also different */
#define TBAT_LC_2D			358	/*2 degree with low current*/
#define TBAT_HC_2D			1432	/*2 degree with high curent*/

/*These definition are for pm860x Ibat,vbat calibration*/
struct pm860x_nvram
{
	unsigned char d4value;
	unsigned char d5value;
	unsigned char d7value;
	signed short ibat_offset;
	signed short vbat_offset;
	unsigned short  vbat_slope_low;
	unsigned short  vbat_slope_high;
};
typedef void (*disable_rf_fn)(void);

struct pm860x_calibrate
{
	unsigned char ibat_4c;
	unsigned char ibat_48;
	unsigned char ibat_49;
	unsigned char vbat_4c[3];
	unsigned char vbat_48[3];
	unsigned char vbat_temp[3];
	unsigned short vbat_input;
};

struct pm860x_battery_info {
	struct pm860x_chip	*chip;
	struct i2c_client	*i2c;
	struct device		*dev;

	struct power_supply	battery;
	struct delayed_work	monitor_work;
	struct delayed_work	changed_work;
	struct mutex		lock;
	struct pm860x_nvram	nvram_data;
	struct pm860x_calibrate calibrate_val;
	disable_rf_fn 		disable_rf;
	unsigned int		irq_base;
	int			tbat_mode;
	int			status;
	int			irq_cc;
	int			irq_batt;
	int			max_capacity;
	int			resistor;	/* Battery Internal Resistor */
	int			last_capacity;
	int			start_soc;
	int			offset_ibat;/*ibat offset in calibration*/
	unsigned		present : 1;
	unsigned		temp_type : 1;	/* TINT or TBAT */
};

struct ccnt {
	unsigned long long int	pos;
	unsigned long long int	neg;
	unsigned int		spos;
	unsigned int		sneg;

	int			total_chg;	/* mAh(3.6C) */
	int			total_dischg;	/* mAh(3.6C) */
};

/*
 * State of Charge.
 * The first number is mAh(=3.6C), and the second number is percent point.
 */
#if defined(CONFIG_MACH_SAARB_MG1) || defined(CONFIG_GFORCE) || defined(CONFIG_MACH_ALKON) || defined(CONFIG_MACH_JETTA)
/*array_soc should be created for each different battery.
  don't change one value in the table.In order to change
  need to re-calculate the complete table according to the
  battery charge/discharge curve*/

int array_soc[][2] = {  {4180, 100},
			{4170, 99}, {4150, 98}, {4140, 97}, {4130, 96},
			{4120, 95}, {4110, 94}, {4090, 93}, {4080, 92},
			{4070, 91}, {4060, 90}, {4060, 89}, {4050, 88},
			{4040, 87}, {4030, 86}, {4010, 85}, {4000, 84},
			{3990, 83}, {3980, 82}, {3970, 81}, {3960, 80},
			{3950, 79}, {3940, 78}, {3940, 77}, {3930, 76},
			{3930, 75}, {3920, 74}, {3910, 73}, {3900, 72},
			{3890, 71}, {3890, 70}, {3880, 69}, {3870, 68},
			{3870, 67}, {3860, 66}, {3850, 65}, {3840, 64},
			{3840, 63}, {3830, 62}, {3820, 61}, {3810, 60},
			{3810, 59}, {3800, 58}, {3800, 57}, {3790, 56},
			{3780, 55}, {3780, 54}, {3780, 53}, {3770, 52},
			{3770, 51}, {3770, 50}, {3760, 49}, {3760, 48},
			{3760, 47}, {3760, 46}, {3760, 45}, {3750, 44},
			{3750, 43}, {3750, 42}, {3750, 41}, {3750, 40},
			{3750, 39}, {3750, 38}, {3750, 37}, {3750, 36},
			{3750, 35}, {3740, 34}, {3740, 33}, {3740, 32},
			{3740, 31}, {3740, 30}, {3730, 29}, {3730, 28},
			{3720, 27}, {3710, 26}, {3700, 25}, {3690, 24},
			{3680, 23}, {3670, 22}, {3660, 21}, {3650, 20},
			{3640, 19}, {3630, 18}, {3620, 17}, {3610, 16},
			{3600, 15}, {3600, 14}, {3590, 13}, {3580, 12},
			{3570, 11}, {3560, 10}, {3550, 9},  {3540, 8},
			{3530, 7},  {3520, 6},  {3510, 5},  {3500, 4},
			{3490, 3},  {3490, 2},  {3480, 1},  {PM860X_POWER_OFF, 0} };
#else
int array_soc[][2] = {  {4170, 100},
			{4154, 99}, {4136, 98}, {4122, 97}, {4107, 96},
			{4102, 95}, {4088, 94}, {4081, 93}, {4070, 92},
			{4060, 91}, {4053, 90}, {4044, 89}, {4035, 88},
			{4028, 87}, {4019, 86}, {4013, 85}, {4006, 84},
			{3995, 83}, {3987, 82}, {3982, 81}, {3976, 80},
			{3968, 79}, {3962, 78}, {3954, 77}, {3946, 76},
			{3941, 75}, {3934, 74}, {3929, 73}, {3922, 72},
			{3916, 71}, {3910, 70}, {3904, 69}, {3898, 68},
			{3892, 67}, {3887, 66}, {3880, 65}, {3874, 64},
			{3868, 63}, {3862, 62}, {3854, 61}, {3849, 60},
			{3843, 59}, {3840, 58}, {3833, 57}, {3829, 56},
			{3824, 55}, {3818, 54}, {3815, 53}, {3810, 52},
			{3808, 51}, {3804, 50}, {3801, 49}, {3798, 48},
			{3796, 47}, {3792, 46}, {3789, 45}, {3785, 44},
			{3784, 43}, {3782, 42}, {3780, 41}, {3777, 40},
			{3776, 39}, {3774, 38}, {3772, 37}, {3771, 36},
			{3769, 35}, {3768, 34}, {3764, 33}, {3763, 32},
			{3760, 31}, {3760, 30}, {3754, 29}, {3750, 28},
			{3749, 27}, {3744, 26}, {3740, 25}, {3734, 24},
			{3732, 23}, {3728, 22}, {3726, 21}, {3720, 20},
			{3716, 19}, {3709, 18}, {3703, 17}, {3698, 16},
			{3692, 15}, {3683, 14}, {3675, 13}, {3670, 12},
			{3665, 11}, {3661, 10}, {3657, 9},  {3649, 8},
			{3637, 7},  {3622, 8},	{3609, 6},  {3580, 4},
			{3658, 3},  {3540, 2},  {3510, 1},  {3429, 0} };
#endif

static struct ccnt ccnt_data;
static struct pm860x_battery_info *ginfo;

static int calc_ccnt(struct pm860x_battery_info *info, struct ccnt *ccnt);
static int calc_soc(struct pm860x_battery_info *info, int state, int *soc);
static int clear_ccnt(struct pm860x_battery_info *info, struct ccnt *ccnt);
static int measure_current(struct pm860x_battery_info *info, int *data);
#ifdef VBAT_CALIB_OTHER
static long	vbatCalibOther;
static long	vbatNumAvg;
static long	vbatGain;
static long	vbatOffset;
#endif

#ifdef CONFIG_ALTERNATE_CHARGER
static pm860xHealthCbFunc healthCbFun = NULL; 
static pm860xStatusCbFunc statusCbFun = NULL; /*charger status charging/dis/full*/
static pm860xCapacityCbFunc capacityCbFun = NULL;
#endif
static pm860xCoulombCbFunc coulombCbFun = NULL;
static pm860xBattDetCbFunc battDetCbFun = NULL;

static irqreturn_t pm860x_coulomb_handler(int irq, void *data)
{
	struct pm860x_battery_info *info = data;
	
	pr_info("pm860x_coulomb_handler\n");
	
	calc_ccnt(info, &ccnt_data);
	return IRQ_HANDLED;
}

static irqreturn_t pm860x_batt_handler(int irq, void *data)
{
	struct pm860x_battery_info *info = data;
	int ret;
	
	pr_info("pm860x_battdet_handler\n");
	
	mutex_lock(&info->lock);
	ret = pm860x_reg_read(info->i2c, PM8607_STATUS_2);
	if (ret & STATUS2_BAT) {
		info->present = 1;
		info->temp_type = PM860X_TEMP_TBAT;
	} else {
		info->present = 0;
		info->temp_type = PM860X_TEMP_TINT;
	}
	mutex_unlock(&info->lock);
	/* clear ccnt since battery is attached or dettached */
	clear_ccnt(info, &ccnt_data);
	return IRQ_HANDLED;
}
static void pm860x_battery_work(struct work_struct *work)
{
	struct pm860x_battery_info *info = container_of(work,
		struct pm860x_battery_info, monitor_work.work);

	//power_supply_changed(&info->battery);
	queue_delayed_work(info->chip->monitor_wqueue, &info->monitor_work,
			   MONITOR_INTERVAL);
}

static void pm860x_changed_work(struct work_struct *work)
{
	struct pm860x_battery_info *info = container_of(work,
		struct pm860x_battery_info, changed_work.work);
	int ret, data;

#ifdef CONFIG_ALTERNATE_CHARGER
		mutex_lock(&info->lock);
		if (statusCbFun)
			info->status = statusCbFun();
		mutex_unlock(&info->lock);
#else	
	ret = pm860x_reg_read(info->i2c, PM8607_STATUS_2);
	if (ret & STATUS2_BAT) {
		ret = measure_current(info, &data);
		if (ret < 0)
			goto out;
		mutex_lock(&info->lock);
		info->present = 1;
		if (data > 0)
			info->status = POWER_SUPPLY_STATUS_CHARGING;
		else
			info->status = POWER_SUPPLY_STATUS_DISCHARGING;
		mutex_unlock(&info->lock);
	} else {
		mutex_lock(&info->lock);
		info->present = 0;
		info->status = POWER_SUPPLY_STATUS_UNKNOWN;
		mutex_unlock(&info->lock);
	}
#endif	
	power_supply_changed(&info->battery);

out:
	return;
}

/* Calculate start_soc */
static void pm860x_init_battery(struct pm860x_battery_info *info)
{
	unsigned char buf[2];
	int ret, data ;
	int bat_remove, soc;

	/* measure enable on GPADC1 */
	data = MEAS1_GP1;
	if (info->temp_type == PM860X_TEMP_TINT)
		data |= MEAS1_TINT;
	ret = pm860x_set_bits(info->i2c, PM8607_MEAS_EN1, data, data);
	if (ret)
		goto out;

	/* detect battery via GPADC1 */
	ret = pm860x_set_bits(info->i2c, PM8607_CHG_CTRL6,
			      CC6_BAT_DET_GPADC1, CC6_BAT_DET_GPADC1);
	if (ret)
		goto out;

	/* measure enable on IBAT, BAT_DET, CC. IBAT is depend on CC. */
	data = MEAS3_IBAT | MEAS3_BAT_DET | MEAS3_CC;
	ret = pm860x_set_bits(info->i2c, PM8607_MEAS_EN3, data, data);
	if (ret)
		goto out;

	/* measure disable CC in sleep time  */
	ret = pm860x_reg_write(info->i2c, PM8607_MEAS_OFF_TIME1, 0x82);
	if (ret)
		goto out;
	ret = pm860x_reg_write(info->i2c, PM8607_MEAS_OFF_TIME2, 0x6c);
	if (ret)
		goto out;

	/* enable GPADC */
	ret = pm860x_set_bits(info->i2c, PM8607_GPADC_MISC1, GPMISC1_GPADC_EN,
			      GPMISC1_GPADC_EN);
	if (ret < 0)
		goto out;

	/* detect battery via GPADC1 */
	ret = pm860x_set_bits(info->i2c, PM8607_CHG_CTRL6, CC6_BAT_DET_GPADC1,
			      CC6_BAT_DET_GPADC1);
	if (ret < 0)
		goto out;

	ret = pm860x_set_bits(info->i2c, PM8607_CCNT, 7 << 3,
			      CCNT_AVG_SEL);
	if (ret < 0)
		goto out;

	/* set GPADC1 bias*/
	ret = pm860x_set_bits(info->i2c, PM8607_GP_BIAS2, 0xF << 4,
			      GPBIAS2_GPADC1_15UA);
	if (ret < 0)
		goto out;

	/* hw issue fix from pm860x C0 unexpected BAT interrupt when chip-sleep */
	ret = pm860x_set_bits(info->i2c, 0x3a, 0xF, 0x2);
	if (ret < 0)
		goto out;

	/* check whether battery present) */
	mutex_lock(&info->lock);
	ret = pm860x_reg_read(info->i2c, PM8607_STATUS_2);
	if (ret < 0) {
		mutex_unlock(&info->lock);
		goto out;
	}
	if (ret & STATUS2_BAT) {
		info->present = 1;
		info->temp_type = PM860X_TEMP_TBAT;
	} else {
		info->present = 0;
		info->temp_type = PM860X_TEMP_TINT;
	}
	mutex_unlock(&info->lock);

	/*set default battery calibration data*/
	info->nvram_data.ibat_offset = 0;
	info->nvram_data.vbat_offset = 0;
	info->nvram_data.vbat_slope_high = 1000;
	info->nvram_data.vbat_slope_low = 1000;

	calc_soc(info, OCV_MODE_ACTIVE, &soc);

	/*check if battery plug in/out or not*/
	if(info->chip->chip_version <= PM8607_CHIP_C1 && info->present) {
		data = pm860x_reg_read(info->i2c, PM8607_RTC_MISC3);
		bat_remove = (data >> 4)&0xF;
		dev_dbg(info->dev,"new battery insert? %s\n",
			bat_remove != 0 ? "yes":"no");
		/* restore SOC from RTC domain register*/
		if( bat_remove == 0) {
			buf[0] = pm860x_reg_read(info->i2c, PM8607_RTC_MISC2);
			buf[1] = pm860x_reg_read(info->i2c, PM8607_RTC1);
			data = ((buf[1] & 0x3)<<5) |((buf[0] >> 3) & 0x1F);
			if(data > soc + 15)
				info->start_soc = soc;
			else if(data < soc - 15)
				info->start_soc = soc;
			else
				info->start_soc = data;
			dev_dbg(info->dev,"soc_rtc %d, soc_ocv :%d \n",data,soc);
		} else {
			pm860x_set_bits(info->i2c, PM8607_RTC_MISC3, 0xF << 4, 0);
			info->start_soc = soc;
		}
	}else if(info->chip->chip_version >= PM8607_CHIP_D0 && info->present){
		data = pm860x_reg_read(info->i2c, PM8607_POWER_UP_LOG);
		bat_remove = data & BAT_WU_LOG;
		dev_dbg(info->dev,"--battery wake up? %s\n",
			bat_remove != 0 ? "yes":"no");
		/* restore SOC from RTC domain register*/
		if( bat_remove == 0) {
			buf[0] = pm860x_reg_read(info->i2c, PM8607_RTC_MISC2);
			buf[1] = pm860x_reg_read(info->i2c, PM8607_RTC1);
			data = ((buf[1] & 0x3)<<5) |((buf[0] >> 3) & 0x1F);
			if(data > soc + 15)
				info->start_soc = soc;
			else if(data < soc - 15)
				info->start_soc = soc;
			else
				info->start_soc = data;
			dev_dbg(info->dev,"soc_rtc %d, soc_ocv :%d \n",data,soc);
		} else {
			pm860x_set_bits(info->i2c, PM8607_POWER_UP_LOG, BAT_WU_LOG, BAT_WU_LOG);
			info->start_soc = soc;
		}
	} else {
		info->start_soc = soc;
	}
	info->last_capacity = info->start_soc;
	dev_dbg(info->dev, "init soc : %d\n", info->last_capacity);
out:
	return;
}

/*
 * register 1 bit[7:0] -- bit[11:4] of measured value of voltage
 * register 0 bit[3:0] -- bit[3:0] of measured value of voltage
 */
static int measure_12bit_voltage(struct pm860x_battery_info *info,
				 int offset, int *data)
{
	unsigned char buf[2];
	int ret;

	if (!data)
		return -EINVAL;

	ret = pm860x_bulk_read(info->i2c, offset, 2, buf);
	if (ret < 0)
		return ret;

	*data = ((buf[0] & 0xff) << 4) | (buf[1] & 0x0f);
	/* V_MEAS(mV) = data * 1.8 * 1000 / (2^12) */
	*data = ((*data & 0xfff) * 9 * 25) >> 9;
	return 0;
}

#ifdef CONFIG_ALTERNATE_CHARGER
extern void pm860x_read_tbat_adc(int gpadc, int *tbat)
{
	int ret, data;

	mutex_lock(&ginfo->lock);
	ret = measure_12bit_voltage(ginfo, gpadc, &data);
	if (ret) {
		printk(KERN_ALERT \
		"pm860x_read_tbat[GPADC[%x]]->fail!!!\n", gpadc);
		*tbat = 0;
		return;
	}
	*tbat = data;
	mutex_unlock(&ginfo->lock);
} EXPORT_SYMBOL(pm860x_read_tbat_adc);
#endif

#if 0
static int measure_8bit_voltage(struct pm860x_battery_info *info,
				int offset, int *data)
{
	int ret;

	if (!data)
		return -EINVAL;

	ret = pm860x_reg_read(info->i2c, offset);
	if (ret < 0)
		return ret;

	/* V_MEAS(mV) = data * 1.8 * 1000 / (2^8) */
	*data = ((ret & 0xff) * 9 * 25) >> 5;
	return 0;
}
#endif

static int measure_vbatt(struct pm860x_battery_info *info, int state, int *data)
{
	unsigned char buf[5];
	int ret=0,vbat_base;

	switch (state) {
	case OCV_MODE_ACTIVE:
		ret = measure_12bit_voltage(info, PM8607_VBAT_MEAS1, data);
		if (ret)
			return ret;
		/* V_BATT_MEAS(mV) = value * 3 * 1.8 * 1000 / (2^12) */
		*data *= 3;
		break;
	case OCV_MODE_SLEEP:
		/*
		 * voltage value of VBATT in sleep mode is saved in different
		 * registers.
		 * bit[11:10] -- bit[7:6] of LDO9(0x18)
		 * bit[9:8] -- bit[7:6] of LDO8(0x17)
		 * bit[7:6] -- bit[7:6] of LDO7(0x16)
		 * bit[5:4] -- bit[7:6] of LDO6(0x15)
		 * bit[3:0] -- bit[7:4] of LDO5(0x14)
		 */
		ret = pm860x_bulk_read(info->i2c, PM8607_LDO5, 5, buf);
		if (ret < 0)
			return ret;
		ret = ((buf[4] >> 6) << 10) | ((buf[3] >> 6) << 8)
			| ((buf[2] >> 6) << 6) | ((buf[1] >> 6) << 4)
			| (buf[0] >> 4);
		/* V_BATT_MEAS(mV) = data * 3 * 1.8 * 1000 / (2^12) */
		*data = ((*data & 0xff) * 27 * 25) >> 9;
		break;
	default:
		return -EINVAL;
	}
	/* battery calibration must act also upon VBAT in sleep meas*/
#ifdef VBAT_CALIB_OTHER
		if (vbatCalibOther) {
			dev_dbg(info->dev, "measure_vbatt other before[%d]\n", \
			*data);
			if (vbatGain > 0) {
				*data = (((*data << 12) - (vbatOffset << 8))/(vbatGain));
				dev_dbg(info->dev, "measure_vbatt other after[%d]G[%ld]O[%ld]\n", \
				*data, vbatGain, vbatOffset);
			}
			/* if gain <= 0 then don't use calibration*/
		} else
#endif
		{
			dev_dbg(info->dev, "measure_vbatt [%d]\n", *data);
			vbat_base =  3700 - info->nvram_data.vbat_offset;
			if (*data <= 3700)
				*data = (3700 * 1000  - (vbat_base - *data) * info->nvram_data.vbat_slope_low)/1000;
			else
				*data = (3700 * 1000  + (*data - vbat_base) * info->nvram_data.vbat_slope_high)/1000;
		}

	return 0;
}

#ifdef CONFIG_ALTERNATE_CHARGER
/*return calibrated vbat if calib data exist */
extern void pm860x_read_vbat(int *vbat)
{
	int ret, data;

	mutex_lock(&ginfo->lock);
	ret = measure_vbatt(ginfo, OCV_MODE_ACTIVE, &data);
	if (ret) {
		printk(KERN_ALERT "pm860x_read_vbat->fail!!!\n");
		*vbat = 0;
		return;
	}
	*vbat = data;
	mutex_unlock(&ginfo->lock);
} EXPORT_SYMBOL(pm860x_read_vbat);
#endif

static void set_temp_threshold(struct pm860x_battery_info *info,
			       int min, int max)
{
	int data;

	/* (tmp << 8) / 1800 */
	if (min <= 0)
		data = 0;
	else
		data = (min << 8) / 1800;
	pm860x_reg_write(info->i2c, PM8607_GPADC1_HIGHTH, data);
	dev_dbg(info->dev, "TEMP_HIGHTH : min: %d, 0x%x\n", min, data);

	if (max <= 0)
		data = 0xff;
	else
		data = (max << 8) / 1800;
	pm860x_reg_write(info->i2c, PM8607_GPADC1_LOWTH, data);
	dev_dbg(info->dev, "TEMP_LOWTH:max : %d, 0x%x\n", max, data);
}

void pm860x_set_temp_threshold(int min, int max)
{
	struct pm860x_battery_info *info = ginfo;
	set_temp_threshold(info, min, max);
	pr_info( \
	"[%s][%s]min[%d]max[%d]\n", 
	__FILE__, __FUNCTION__, min, max);
	
}EXPORT_SYMBOL(pm860x_set_temp_threshold);

static int measure_temp(struct pm860x_battery_info *info, int *data)
{
	int ret, temp;

	if (!data)
		return -EINVAL;
	if (info->temp_type == PM860X_TEMP_TINT) {
		ret = measure_12bit_voltage(info, PM8607_TINT_MEAS1, data);
		if (ret)
			return ret;
		*data = (*data - 884) * 1000 / 3611;
	} else {
		ret = measure_12bit_voltage(info, PM8607_GPADC_TBAT, data);
		if (ret)
			return ret;
		switch (info->tbat_mode) {
		case TBAT_CUR_MODE_LOW:
			if (*data > TBAT_LC_NEG_25D)
				temp = -30;	/* -30 degree */
			else if (*data > TBAT_LC_NEG_17D)
				temp = -15;	/* -15 degree */
			else if (*data > TBAT_LC_5D)
				temp = 0;	/* in range of (-17, 5) */
			else 
				temp = 26;
			break;
		case TBAT_CUR_MODE_HIGH:
			if (*data < TBAT_HC_60D)
				temp = 100;	/* 100 degree */
			else if (*data < TBAT_HC_57D)
				temp = 60;	/* 60 degree */
			else if (*data < TBAT_HC_42D)
				temp = 50;	/* 50 degree */
			else if (*data < TBAT_HC_0D)
				temp = 26;	/* 26 degree */
			else
				temp = 0; 
			break;
		}
		dev_dbg(info->dev, "TBAT mode:%d, temp_C:%d C,temp_mv:%d mv\n", \
			info->tbat_mode,temp,*data);
		*data = temp;
	}
	return 0;
}

/*
 * Return value is signed data.
 * Negative value means discharging, and positive value means charging.
 */
static int measure_current(struct pm860x_battery_info *info, int *data)
{
	unsigned char buf[2];
	short s;
	int ret;

	if (!data)
		return -EINVAL;

	ret = pm860x_bulk_read(info->i2c, PM8607_IBAT_MEAS1, 2, buf);
	if (ret < 0)
		return ret;

	s = ((buf[0] & 0xff) << 8) | (buf[1] & 0xff);
	/* current(mA) = value * 0.125 */
	*data = s >> 3;
	*data += info->nvram_data.ibat_offset;
	return 0;
}

static int set_charger_current(struct pm860x_battery_info *info, int data,
			       int *old)
{
	int ret;

	if (data < 50 || data > 1600 || !old)
		return -EINVAL;

	data = ((data - 50) / 50) & 0x1f;
	*old = pm860x_reg_read(info->i2c, PM8607_CHG_CTRL2);
	*old = (*old & 0x1f) * 50 + 50;
	ret = pm860x_set_bits(info->i2c, PM8607_CHG_CTRL2, 0x1f, data);
	if (ret < 0)
		return ret;
	return 0;
}

static int calc_resistor(struct pm860x_battery_info *info)
{
	int ret, i, data;
	int vbatt_sum1, vbatt_sum2, chg_current;
	int ibatt_sum1, ibatt_sum2;
	
	ret = measure_vbatt(info, OCV_MODE_ACTIVE, &data);
	if (ret)
		goto out;
	if (data < VBATT_RESISTOR_MIN || data > VBATT_RESISTOR_MAX)
		goto out;

	/* current is saved */
	if (set_charger_current(info, 500, &chg_current))
		goto out;
	msleep(1000);

	for (i = 0, vbatt_sum1 = 0, ibatt_sum1 = 0; i < 10; i++) {
		ret = measure_vbatt(info, OCV_MODE_ACTIVE, &data);
		if (ret)
			goto out_meas;
		vbatt_sum1 += data;
		ret = measure_current(info, &data);
		if (ret)
			goto out_meas;

		if (data < 0)
			ibatt_sum1 = ibatt_sum1 - data;	/* discharging */
		else
			ibatt_sum1 = ibatt_sum1 + data;	/* charging */
	}

	if (set_charger_current(info, 100, &ret))
		goto out_meas;
	msleep(1000);

	for (i = 0, vbatt_sum2 = 0, ibatt_sum2 = 0; i < 10; i++) {
		ret = measure_vbatt(info, OCV_MODE_ACTIVE, &data);
		if (ret)
			goto out_meas;
		vbatt_sum2 += data;
		ret = measure_current(info, &data);
		if (ret)
			goto out_meas;

		if (data < 0)
			ibatt_sum2 = ibatt_sum2 - data;	/* discharging */
		else
			ibatt_sum2 = ibatt_sum2 + data;	/* charging */
	}

	/* restore current setting */
	if (set_charger_current(info, chg_current, &ret))
		goto out_meas;

	if ((vbatt_sum1 > vbatt_sum2) && (ibatt_sum1 > ibatt_sum2)
		&& (ibatt_sum2 > 0)) {
		/* calculate resistor in discharging case */
		data = 1000 * (vbatt_sum1 - vbatt_sum2)
			/ (ibatt_sum1 - ibatt_sum2);
		if ((data - info->resistor > 0)
			&& (data - info->resistor < info->resistor))
			info->resistor = data;
		if ((info->resistor - data > 0)
			&& (info->resistor - data < data))
			info->resistor = data;
	}
	return 0;

out_meas:
	set_charger_current(info, chg_current, &ret);
out:
	return -EINVAL;
}

int pm860x_calc_resistor(void)
{
	return calc_resistor(ginfo);
}
EXPORT_SYMBOL(pm860x_calc_resistor);

static int query_health(struct pm860x_battery_info *info)
{
	int ret, mv, border, state;

	ret = measure_12bit_voltage(info, PM8607_GPADC_TBAT, &mv);
	if (ret)
		goto out;
	dev_dbg(info->dev, "TBAT (before switch current mode): %dmv\n", mv);

	/* check threshold */
	switch (info->tbat_mode) {
	case TBAT_CUR_MODE_LOW:
		border = TBAT_LC_2D;
		break;
	case TBAT_CUR_MODE_HIGH:
		border = TBAT_HC_2D;
		break;
	default:
		goto out;
	}

	if (mv > border) {
		/* current temperature is lower than 2 degree */
		info->tbat_mode = TBAT_CUR_MODE_LOW;
		ret = pm860x_set_bits(info->i2c, PM8607_GP_BIAS2, 0xf << 4,
				      GPBIAS2_GPADC1_15UA);
		if (ret)
			goto out;
		dev_dbg(info->dev, "Switch to Low battery current mode\n");
	} else {
		/* current temperature is higher than 2 degree */
		info->tbat_mode = TBAT_CUR_MODE_HIGH;
		ret = pm860x_set_bits(info->i2c, PM8607_GP_BIAS2, 0xf << 4,
				      GPBIAS2_GPADC1_60UA);
		if (ret)
			goto out;
		dev_dbg(info->dev, "Switch to High battery current mode\n");
	}
	msleep(200);
	dev_dbg(info->dev, "delayed 200msec after switch\n");

	/* measure current temperature again for high resolution */
	ret = measure_12bit_voltage(info, PM8607_GPADC_TBAT, &mv);
	if (ret)
		goto out;
	dev_dbg(info->dev, "TBAT (after switch current mode): %dmv\n", mv);

	switch (info->tbat_mode) {
	case TBAT_CUR_MODE_LOW:
		if (mv > TBAT_LC_NEG_17D) {
			set_temp_threshold(info, TBAT_LC_NEG_25D,
					   TBAT_LC_NEG_15D);
			dev_dbg(info->dev, "TBAT is -17 degree\n");
			dev_dbg(info->dev, "System should be power off\n");
			state = POWER_SUPPLY_HEALTH_DEAD;
		} else if (mv > TBAT_LC_2D) {
			set_temp_threshold(info, TBAT_LC_NEG_20D, TBAT_LC_5D);
			dev_dbg(info->dev, "Charger isn't allowed for cold\n");
			state = POWER_SUPPLY_HEALTH_COLD;
		} else {
			set_temp_threshold(info, TBAT_LC_2D, 0);
			state = POWER_SUPPLY_HEALTH_GOOD;
		}
		break;
	case TBAT_CUR_MODE_HIGH:
		if (mv > TBAT_HC_2D) {
			set_temp_threshold(info, 0, TBAT_HC_0D);
			state = POWER_SUPPLY_HEALTH_COLD;
		} else if (mv > TBAT_HC_42D) {
			set_temp_threshold(info, TBAT_HC_0D, TBAT_HC_45D);
			state = POWER_SUPPLY_HEALTH_GOOD;
		} else if (mv > TBAT_HC_57D) {
			set_temp_threshold(info, TBAT_HC_40D, TBAT_HC_60D);
			state = POWER_SUPPLY_HEALTH_OVERHEAT;
		} else {
			set_temp_threshold(info, TBAT_HC_55D, TBAT_HC_100D);
			dev_dbg(info->dev, "TBAT is higher than 57 degree\n");
			dev_dbg(info->dev, "System should be power off\n");
			state = POWER_SUPPLY_HEALTH_DEAD;
		}
		break;
	}
	return state;
out:
	return POWER_SUPPLY_HEALTH_UNKNOWN;
}

static int read_ccnt(struct pm860x_battery_info *info, int offset,
		     int *ccnt)
{
	unsigned char buf[2];
	int ret;

	if (!ccnt)
		return -EINVAL;

	ret = pm860x_set_bits(info->i2c, PM8607_CCNT, 7, offset & 7);
	if (ret < 0)
		goto out;
	ret = pm860x_bulk_read(info->i2c, PM8607_CCNT_MEAS1, 2, buf);
	if (ret < 0)
		goto out;
	*ccnt = ((buf[0] & 0xff) << 8) | (buf[1] & 0xff);
	return 0;
out:
	return ret;
}

static int calc_ccnt(struct pm860x_battery_info *info, struct ccnt *ccnt)
{
	unsigned int sum;
	int ret, data;

	ret = read_ccnt(info, CCNT_POS1, &data);
	if (ret)
		goto out;
	sum = data & 0xffff;
	ret = read_ccnt(info, CCNT_POS2, &data);
	if (ret)
		goto out;
	sum |= (data & 0xffff) << 16;
	ccnt->pos += sum;

	ret = read_ccnt(info, CCNT_NEG1, &data);
	if (ret)
		goto out;
	sum = data & 0xffff;
	ret = read_ccnt(info, CCNT_NEG2, &data);
	if (ret)
		goto out;
	sum |= (data & 0xffff) << 16;
	sum = ~sum + 1;		/* since it's negative */
	ccnt->neg += sum;

	ret = read_ccnt(info, CCNT_SPOS, &data);
	if (ret)
		goto out;
	ccnt->spos += data;
	ret = read_ccnt(info, CCNT_SNEG, &data);
	if (ret)
		goto out;

	/*
	 * charge(mAh)  = count * 1.6984 * 1e(-8)
	 *		= count * 16984 * 1.024 * 1.024 * 1.024 / (2 ^ 40)
	 *		= count * 18236 / (2 ^ 40)
	 */
	ccnt->total_chg = (int)((ccnt->pos * 18236) >> 40);
	ccnt->total_dischg = (int)((ccnt->neg * 18236) >> 40);
	return 0;
out:
	return ret;
}

static int clear_ccnt(struct pm860x_battery_info *info, struct ccnt *ccnt)
{
	int data;

	memset(ccnt, 0, sizeof(struct ccnt));
	/* read to clear ccnt */
	read_ccnt(info, CCNT_POS1, &data);
	read_ccnt(info, CCNT_POS2, &data);
	read_ccnt(info, CCNT_NEG1, &data);
	read_ccnt(info, CCNT_NEG2, &data);
	read_ccnt(info, CCNT_SPOS, &data);
	read_ccnt(info, CCNT_SNEG, &data);
	return 0;
}

/* Calculate Open Circuit Voltage */
#define NUM_OF_MEAS 10
static int calc_ocv(struct pm860x_battery_info *info, int *ocv)
{
	int ret, i, data;
	int vbatt_avg, vbatt_sum, ibatt_avg, ibatt_sum;

	if (!ocv)
		return -EINVAL;

	for (i = 0, ibatt_sum = 0, vbatt_sum = 0; i < NUM_OF_MEAS; i++) {
		ret = measure_vbatt(info, OCV_MODE_ACTIVE, &data);
		if (ret)
			goto out;
		vbatt_sum += data;
		ret = measure_current(info, &data);
		if (ret)
			goto out;
		ibatt_sum += data;
	}
	vbatt_avg = vbatt_sum / NUM_OF_MEAS;
	ibatt_avg = ibatt_sum / NUM_OF_MEAS;

	mutex_lock(&info->lock);
	if (info->present)
		*ocv = vbatt_avg - ibatt_avg * info->resistor / 1000;
	else
		*ocv = vbatt_avg;
	mutex_unlock(&info->lock);
	dev_dbg(info->dev, "VBAT average:%d, OCV:%d\n", vbatt_avg, *ocv);
	return 0;
out:
	return ret;
}

/* Calculate State of Charge (percent points) */
static int calc_soc(struct pm860x_battery_info *info, int state, int *soc)
{
	int i, ocv=0, count, ret = -EINVAL;

	if (!soc)
		return -EINVAL;

	switch (state) {
	case OCV_MODE_ACTIVE:
		ret = calc_ocv(info, &ocv);
		break;
	case OCV_MODE_SLEEP:
		ret = measure_vbatt(info, OCV_MODE_SLEEP, &ocv);
		break;
	}
	if (ret)
		goto out;

	count = ARRAY_SIZE(array_soc);
	if (ocv < array_soc[count - 1][0]) {
		*soc = 0;
		return 0;
	}

	for (i = 0; i < count; i++) {
		if (ocv >= array_soc[i][0]) {
			*soc = array_soc[i][1];
			break;
		}
	}
	return 0;
out:
	return ret;
}

static int calc_capacity(struct pm860x_battery_info *info, int *cap)
{
	int ret, data;
	int cap_ocv = 0, cap_cc = 0;

	ret = calc_ccnt(info, &ccnt_data);
	if (ret)
		goto out;
soc:
	data = info->max_capacity * info->start_soc / 100;
	if (ccnt_data.total_dischg - ccnt_data.total_chg <= data) {
		cap_cc = data + ccnt_data.total_chg - ccnt_data.total_dischg;
	} else {
		clear_ccnt(info, &ccnt_data);
		calc_soc(info, OCV_MODE_ACTIVE, &info->start_soc);
		dev_dbg(info->dev, "restart soc = %d !\n", info->start_soc);
		goto soc;
	}

	cap_cc = cap_cc * 100 / info->max_capacity;

	if (cap_cc < 0)
		cap_cc = 0;
	else if (cap_cc > 100)
		cap_cc = 100;
	dev_dbg(info->dev, "%s, last cap : %d", __func__, info->last_capacity);

	if (info->status == POWER_SUPPLY_STATUS_DISCHARGING) {
		ret = calc_soc(info, OCV_MODE_ACTIVE, &cap_ocv);
		if (ret)
			cap_ocv = info->last_capacity;
		measure_vbatt(info, OCV_MODE_ACTIVE, &data);
		if (data <= LOW_BAT_THRESHOLD) {
			/* choose the lower capacity value to report
			 * between vbat and CC when vbat < 3.6v;
			 * than 3.6v;
			*/
			*cap = min(cap_ocv, cap_cc);
		} else {
			/* when detect vbat > 3.6v, but cap_cc < 15,and
			 * cap_ocv is 10% larger than cap_cc, we can think
			 * CC have some accumulation error, switch to OCV
			 * to estimate capacity;
			 * */
			if (cap_cc < 15 && cap_ocv - cap_cc > 10)
				*cap = cap_ocv;
			else
				*cap = cap_cc;
		}
		/* when discharging, make sure current capacity
		 * is lower than last*/
		if (*cap > info->last_capacity)
			*cap = info->last_capacity;
	} else {
		*cap = cap_cc;
	}
	info->last_capacity = *cap;

	dev_dbg(info->dev,"%s, cap_ocv:%d cap_cc:%d,cap:%d \n",
		(info->status == POWER_SUPPLY_STATUS_DISCHARGING) ? "discharging" :
		"charging", cap_ocv, cap_cc, *cap);
	/* store the current capacity to RTC domain register, after next power up ,
	 * it will be restored
	 * */
	pm860x_set_bits(info->i2c, PM8607_RTC_MISC2, RTC_SOC_5LSB,(*cap&0x1F)<<3);
	pm860x_set_bits(info->i2c, PM8607_RTC1, RTC_SOC_3MSB,((*cap>>5)&0x3));
	return 0;
out:
	return ret;
}

static void pm860x_external_power_changed(struct power_supply *psy)
{
	struct pm860x_battery_info *info;

	info = container_of(psy, struct pm860x_battery_info, battery);
	queue_delayed_work(info->chip->monitor_wqueue,
			   &info->changed_work, HZ / 2);
	return;
}

int pm860x_battery_update_soc(void)
{
	struct pm860x_battery_info *info = ginfo;

	clear_ccnt(info, &ccnt_data);
	info->start_soc = 100;
	dev_dbg(info->dev, "chg done, update soc = %d\n",
		info->start_soc);
	return 0;
}

static int pm860x_batt_get_prop(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct pm860x_battery_info *info = dev_get_drvdata(psy->dev->parent);
	int data, ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		mutex_lock(&info->lock);
#ifdef CONFIG_ALTERNATE_CHARGER
		if (statusCbFun)
			val->intval = statusCbFun();
#else
		val->intval = info->status;
#endif
		mutex_unlock(&info->lock);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		mutex_lock(&info->lock);
#ifdef CONFIG_ALTERNATE_CHARGER
		if (healthCbFun)
			val->intval = healthCbFun();
#else
	    val->intval = query_health(info);
#endif
		mutex_unlock(&info->lock);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		mutex_lock(&info->lock);
		val->intval = info->present;
		mutex_unlock(&info->lock);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
#ifdef CONFIG_ALTERNATE_CHARGER
		if (capacityCbFun)
			val->intval = capacityCbFun();
#else
		ret = measure_vbatt(info, OCV_MODE_ACTIVE, &data); 
		if (ret)
			goto out; 	
		
		if (data <= PM860X_POWER_OFF) {
			data = 0;
			pr_info("POWER_SUPPLY_PROP_CAPACITY:PM860X_POWER_OFF vbat[%d] force capacity=0\n",
			data);
		} else
		ret = calc_capacity(info, &data);

		if (ret)
			goto out;
		if(data < 0)
			data=0;
		else if(data > 100)
			data = 100;
		val->intval = data;
#endif 
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		/* return real vbatt Voltage */
		ret = measure_vbatt(info, OCV_MODE_ACTIVE, &data);
		if (ret)
			goto out;
#ifdef CONFIG_ALTERNATE_CHARGER
		val->intval = data;
#else
 		val->intval = data * 1000;
#endif
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_AVG:
		/* return Open Circuit Voltage (not measured voltage) */
		ret = calc_ocv(info, &data);
		if (ret)
			goto out;
#ifdef CONFIG_ALTERNATE_CHARGER
		val->intval = data;
#else
 		val->intval = data * 1000;
#endif
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = measure_current(info, &data);
		if (ret)
			goto out;
		val->intval = data;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		if(info->present){
			ret = measure_temp(info, &data);
			if (ret)
				goto out;
#ifdef CONFIG_ALTERNATE_CHARGER
			/*no action*/
#else
 			data*=10;
#endif
		} else {
			/*fake temp 25C without battery*/
#ifdef CONFIG_ALTERNATE_CHARGER
			data = 25;
#else
 			data = 250;
#endif
		}
		val->intval = data;
		break;
	default:
		return -ENODEV;
	}
	return 0;
out:
	return ret;
}

static enum power_supply_property pm860x_batt_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_AVG,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_TEMP,
};

int ibat_offset_d4_trim1(void)
{
	int vbat;
	u8  temp;
	struct pm860x_battery_info *info = ginfo;

	/*restore default battery calibration paras before calibration*/
	info->nvram_data.ibat_offset = 0;
	info->nvram_data.vbat_offset = 0;
	info->nvram_data.vbat_slope_high = 1000;
	info->nvram_data.vbat_slope_low = 1000;

	info->disable_rf();
	msleep(1);
	info->calibrate_val.ibat_4c = pm860x_reg_read(info->i2c,PM8607_CHG_CTRL5);
	/*Force system to be supplied from charger*/
	pm860x_reg_write(info->i2c,PM8607_CHG_CTRL5, (info->calibrate_val.ibat_4c & 0xF0) | 0x8);
	info->calibrate_val.ibat_48 = pm860x_reg_read(info->i2c,PM8607_CHG_CTRL1);
	/*Charge off*/
	pm860x_reg_write(info->i2c,PM8607_CHG_CTRL1,( info->calibrate_val.ibat_48 & 0xFC));
	measure_vbatt(ginfo, OCV_MODE_ACTIVE, &vbat);
	if((4100 > vbat) && (vbat > 3700))
	{
		temp =(14 - ((vbat - 3200)/100)) << 4;
		/*Set Portofino preregulator output.*/
		pm860x_reg_write(info->chip->companion, PM8606_PREREGULATORA,temp | 0x0E);
	}
	return 0;
}

int ibat_offset_d4_trim2(int i_offset)
{
	int ibat;
	u8 temp_d4, ibat_trim=0, value;
	struct pm860x_battery_info *info = ginfo;

	while(1)
	{
		measure_current(ginfo, &ibat);
		temp_d4 = pm860x_page_reg_read(info->i2c,0xD4);
		dev_dbg(info->dev, "init  d4  = 0x%x ,ibat = %d\n", temp_d4,ibat);
		msleep(100);
		if(i_offset == ibat)
			return info->offset_ibat = 0;
		else if(i_offset > ibat){
			ibat_trim = (i_offset - ibat)>>1;
			value = temp_d4 + ibat_trim;
			pm860x_page_reg_write(info->i2c, 0xD4 , value);
			msleep(100);
		}
		else{
			ibat_trim = (ibat - i_offset)>>1;
			value = temp_d4 - ibat_trim;
			pm860x_page_reg_write(info->i2c, 0xD4 , value);
			msleep(100);
		}
		measure_current(ginfo, &ibat);
		dev_dbg(info->dev, "sys read ibat2 = %d\n",ibat);
		if(ibat < 2 && ibat > -2){
			info->offset_ibat = i_offset - ibat ;
			dev_dbg(info->dev, "info->offset_ibat = %d \n", info->offset_ibat);
			dev_dbg(info->dev, "trim value d4  = 0x%x \n", value);
			return info->offset_ibat;	//D4 trim value
		}
	}
}

int ibat_gain_d5d7_trim1(void)
{
	int ibat1,ibat2;
	int ibat_delta;
	int vbat,ret;
	struct pm860x_battery_info *info = ginfo;

	/* disable chg done interrupt */
	pm860x_set_bits(info->i2c, 0x08, 1<<6,0<<6);

	ret = pm860x_reg_read(info->i2c, PM8607_STATUS_2);
	if(ret & STATUS2_BAT && ret & STATUS2_CHG)
	{
		measure_vbatt(ginfo, OCV_MODE_ACTIVE, &vbat);
		dev_dbg(info->dev, "vbat = %d \n", vbat);
		if((4100 > vbat) && (vbat > 3700))
		{
			while(1)
			{
				/*Set Portofino preregulator output to 4.5V.*/
				pm860x_reg_write(info->chip->companion, PM8606_PREREGULATORA,0x0E);
				info->calibrate_val.ibat_49 = pm860x_reg_read(info->i2c,PM8607_CHG_CTRL2);
				/*Set fast charge current to 300mA*/
				pm860x_reg_write(info->i2c,PM8607_CHG_CTRL2, 0x5);
				info->calibrate_val.ibat_48 = pm860x_reg_read(info->i2c,PM8607_CHG_CTRL1);
				pm860x_reg_write(info->i2c,PM8607_CHG_CTRL1, info->calibrate_val.ibat_48|0x2);

				msleep(1000);
				while(1)
				{
					msleep(100);/*Wait 100mS*/
					measure_current(ginfo, &ibat1);
					msleep(100);/*Wait 100mS*/
					measure_current(ginfo, &ibat2);

					if(ibat1 == ibat2)
						return info->offset_ibat;
					else if(ibat1 < ibat2)
						ibat_delta = ibat2 - ibat1;
					else
						ibat_delta = ibat1 - ibat2;
					if(ibat_delta <= 3)
						return info->offset_ibat;
				}
			}
		}
	}
	return info->offset_ibat;
}

int ibat_gain_d5d7_trim2(u16 i_trim)
{
	u8  temp;
	u16 ibat_gain;
	u16 temp_i=0;
	u8 gain2;
	int vbat,ibat=0,ret;
	struct pm860x_battery_info *info = ginfo;

	ret = pm860x_reg_read(info->i2c, PM8607_STATUS_2);
	if(ret & STATUS2_BAT && ret & STATUS2_CHG)
	{
		measure_vbatt(ginfo, OCV_MODE_ACTIVE, &vbat);
		dev_dbg(info->dev, "%s:vbat=%d\n",__func__,vbat);
		if((4100 > vbat) && (vbat > 3700))
		{
			msleep(100);
			measure_current(ginfo, &ibat);
			temp = pm860x_page_reg_read(info->i2c,0xD7);
			ibat_gain = (u16)temp;
			ibat_gain &=0xF0;
			gain2 = pm860x_page_reg_read(info->i2c,0xD5);
			ibat_gain = (ibat_gain << 4) |gain2;

			if(info->offset_ibat < 0)
				ibat = ibat + (info->offset_ibat & 0xFFFF);
			else
				ibat = ibat - (info->offset_ibat & 0xFFFF);
			if(ibat <= i_trim){
				temp_i = 256*(i_trim - ibat)/34;

				if(ibat_gain <= temp_i){
					temp_i = temp_i - ibat_gain;
					ibat_gain = 0xFFF - temp_i;
				} else {
					ibat_gain = ibat_gain + temp_i;
			}
			} else {
				temp_i = 256*(ibat - i_trim)/34;
				if(ibat_gain <= temp_i){
					temp_i = temp_i - ibat_gain;
					ibat_gain = 0xFFF - temp_i;
				} else {
				ibat_gain = ibat_gain - temp_i;
			}
			}

			temp = ibat_gain & 0xFF;
			pm860x_page_reg_write(info->i2c, 0xD5 , temp);
			temp = pm860x_page_reg_read(info->i2c,0xD7);
			temp = (temp & 0xF)  ;
			temp |= ((ibat_gain & 0xF00) >>4);
			pm860x_page_reg_write(info->i2c, 0xD7 , temp);
			msleep(100);
			temp= pm860x_reg_read(info->i2c,PM8607_IBAT_MEAS1);
			temp_i = temp << 8;
			temp = pm860x_reg_read(info->i2c,PM8607_IBAT_MEAS2);
			temp_i = temp_i | temp;
			if(temp_i <= (i_trim<<3)){
				while(temp_i <= (i_trim>>3)){
					ibat_gain = ibat_gain +1;
					temp = ibat_gain & 0xFF;
					pm860x_page_reg_write(info->i2c, 0xD5 , temp);
					temp = (ibat_gain & 0xF00) >>4;
					temp = pm860x_page_reg_read(info->i2c,0xD7);
					temp = (temp & 0xF);
					temp |= ((ibat_gain & 0xF00) >>4);
					pm860x_page_reg_write(info->i2c, 0xD7 , temp);
					msleep(100);
					temp = pm860x_reg_read(info->i2c,PM8607_IBAT_MEAS1);
					temp_i = temp << 8;
					temp = pm860x_reg_read(info->i2c,PM8607_IBAT_MEAS2);
					temp_i = temp_i | temp;
				}
			}
			else{
				while(temp_i > (i_trim<<3)){
					if(ibat_gain >= 1)
						ibat_gain = ibat_gain -1;
					else
						ibat_gain = 0xFFF;

					temp = ibat_gain & 0xFF;

					pm860x_page_reg_write(info->i2c, 0xD5 , temp);
					temp = pm860x_page_reg_read(info->i2c,0xD7);
					temp = (temp & 0xF)  ;
					temp |= ((ibat_gain & 0xF00) >>4);
					pm860x_page_reg_write(info->i2c, 0xD7 , temp);
					msleep(100);
					temp = pm860x_reg_read(info->i2c,PM8607_IBAT_MEAS1);
					temp_i = temp << 8;
					temp = pm860x_reg_read(info->i2c,PM8607_IBAT_MEAS2);
					temp_i = temp_i | temp;
				}
			}

			measure_current(ginfo, &ibat);
			dev_dbg(info->dev, "final ibat  = %d \n", ibat);
			temp = pm860x_page_reg_read(info->i2c,0xD7);
			dev_dbg(info->dev, "final d7 = 0x%x \n",temp);
			temp = pm860x_page_reg_read(info->i2c,0xD5);
			dev_dbg(info->dev, "final d5 = 0x%x \n",temp);
			dev_dbg(info->dev, "Ibat_gain = %d \n", ibat_gain);
			dev_dbg(info->dev, "Ibat_gain= %d\n", ibat_gain);
		} else {
			dev_dbg(info->dev, "Currently can not do Ibat trim!\n");
		}
	} else {
		dev_dbg(info->dev, "Unknow PMIC!Can't do Ibat trim!\n");
	}

	/*restore ox4C 0x49 0x48 register*/
	pm860x_reg_write(info->i2c,PM8607_CHG_CTRL5, info->calibrate_val.ibat_4c);
	pm860x_reg_write(info->i2c,PM8607_CHG_CTRL2, info->calibrate_val.ibat_49);
	pm860x_reg_write(info->i2c,PM8607_CHG_CTRL1, info->calibrate_val.ibat_48);

	return 1;
}
#ifdef VBAT_CALIB_OTHER
#ifdef CONFIG_SYSFS
static ssize_t battery_show_vbat_calib_other(struct device *dev,
				struct device_attribute *attr, char *buf)
{
		pr_info( \
		"\battery_show_vbat_calib_other vbatCalibOther[%ld]\n", vbatCalibOther);
		return sprintf(buf, "%ld\n", vbatCalibOther);
}

static ssize_t battery_show_vbat_avg(struct device *dev,
				struct device_attribute *attr, char *buf)
{
		int ret, i, vbat = 0;
		u32 sum = 0;
		struct pm860x_battery_info *info = ginfo;

		if (vbatNumAvg > 0) {
			for (i = 0; i < vbatNumAvg; i++) {
				ret = measure_vbatt(info, OCV_MODE_ACTIVE, &vbat);
				if (ret)
					dev_err(info->dev, \
					"battery_show_vbat_avg:ret val wrong");
				pr_info( \
				"battery_show_vbat_avg vbat[%d]\n", vbat);
				sum += vbat;
			}
			vbat = sum/vbatNumAvg;
		}
		pr_info( \
		"\nbattery_show_vbat_avg vbatAvg[%d]from[%ld]measurments\n",\
		vbat, vbatNumAvg);
		return sprintf(buf, "%d\n", vbat);
}
static ssize_t battery_show_vbat_num_avg(struct device *dev,
				struct device_attribute *attr, char *buf)
{
		printk(
		"\nbattery_show_vbat_num_avg vbatNumAvg[%ld]\n", vbatNumAvg);
		return sprintf(buf, "%ld\n", vbatNumAvg);
}

static ssize_t battery_set_vbat_num_avg(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
		strict_strtol(buf, 10, &vbatNumAvg);
		/* if set gain then other vbat calib,will happen on calib*/
		vbatCalibOther = 1;
		printk(
		"\nbattery_set_vbat_num_avg vbatNumAvg[%ld]\n", vbatNumAvg);
		return count;
}

static ssize_t battery_show_vbat_gain(struct device *dev,
				struct device_attribute *attr, char *buf)
{
		printk(
		"\nbattery_show_vbat_gain vbatGain[%ld]\n", vbatGain);
		return sprintf(buf, "%ld\n", vbatGain);
}

static ssize_t battery_set_vbat_gain(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
		strict_strtol(buf, 10, &vbatGain);
		/* if set gain then other vbat calib,will happen on init*/
		vbatCalibOther = 1;
		printk(
		"\nbattery_set_vbat_gain vbatGain[%ld]\n", vbatGain);
		return count;
}

static ssize_t battery_show_vbat_offset(struct device *dev,
				struct device_attribute *attr, char *buf)
{
		printk(
		"\nbattery_show_vbat_offset vbatOffset[%ld]\n", vbatOffset);
		return sprintf(buf, "%ld\n", vbatOffset);
}

static ssize_t battery_set_vbat_offset(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
		strict_strtol(buf, 10, &vbatOffset);
		printk(
		"\nbattery_set_vbat_offset vbatOffset[%ld]\n", vbatOffset);
		return count;
}
#endif
#endif

static ssize_t battery_cali_show_attrs(struct device *dev,
					      struct device_attribute *attr,
					      char *buf)
{
	u8 val;
	int len;
	val = ibat_gain_d5d7_trim1();
	len = sprintf(buf, "%d\n", val);
	return len;
}

static ssize_t battery_cali_store_attrs(struct device *dev,
					      struct device_attribute *attr,
					      const char *buf, size_t count)
{
	u16 input;
	u16 value;
	char *last = NULL;
	struct pm860x_battery_info *info = dev_get_drvdata(dev);

	input = simple_strtoul(buf, &last, 0);
	value = ibat_gain_d5d7_trim2(input);
	dev_dbg(info->dev, "input value = %d\n",input);
	return count;
}

static ssize_t battery_info_show_attrs(struct device *dev,
					      struct device_attribute *attr,
					      char *buf)
{
	struct pm860x_battery_info *info = dev_get_drvdata(dev);
	u8 val = 0;
	int len = 0;

	dev_dbg(info->dev, "%s \n",__func__);
	ibat_offset_d4_trim1();
	len = sprintf(buf, "%d\n", val);
	return len;

}
static ssize_t battery_info_store_attrs(struct device *dev,
					      struct device_attribute *attr,
					      const char *buf, size_t count)
{
	int input;
	char *last = NULL;
	struct pm860x_battery_info *info = dev_get_drvdata(dev);

	input = simple_strtoul(buf, &last, 0);
	dev_dbg(info->dev, "%s, input=%d \n",__func__,input);
	ibat_offset_d4_trim2(input);

	return count;
}

static ssize_t battery_trimreturn_show_attrs(struct device *dev,
					      struct device_attribute *attr,
					      char *buf)
{
	u8 temp1,temp2,temp3;
	u32 value;
	int len;
	struct pm860x_battery_info *info = dev_get_drvdata(dev);
	dev_dbg(info->dev, "%s \n",__func__);

	temp1 = pm860x_page_reg_read(info->i2c,0xD4);
	temp2 = pm860x_page_reg_read(info->i2c,0xD5);
	temp3 = pm860x_page_reg_read(info->i2c,0xD7);
	value = temp1 + (temp2<<8) + (temp3<<16);
	dev_dbg(info->dev, "d4=0x%x, d5=0x%x,d7=0x%x \n", temp1,temp2,temp3);
	len = sprintf(buf, "%d\n", value);
	return len;
}
static ssize_t battery_trimreturn_store_attrs(struct device *dev,
					      struct device_attribute *attr,
					      const char *buf, size_t count)
{
	return count;
}
static ssize_t battery_vol_show_attrs(struct device *dev,
					      struct device_attribute *attr,
					      char *buf)
{
	u16 value;
	u8 i;
	int vbat1 = 0,vbat2 = 0, len = 0;

	struct pm860x_battery_info *info = dev_get_drvdata(dev);

	for (i = 0; i < 10; i++) {
		measure_vbatt(ginfo, OCV_MODE_ACTIVE, &vbat2);
		vbat1 += vbat2;
	}
	value = vbat1/10;
	dev_dbg(info->dev, "%s,value = %d \n",__func__,value);
	if( 3195 < info->calibrate_val.vbat_input && info->calibrate_val.vbat_input < 3205){
		pm860x_reg_write(info->i2c,PM8607_CHG_CTRL5, info->calibrate_val.vbat_4c[0]);
		pm860x_reg_write(info->i2c,PM8607_CHG_CTRL1, info->calibrate_val.vbat_48[0]);
		pm860x_reg_write(info->chip->companion, PM8606_PREREGULATORA,info->calibrate_val.vbat_temp[0]);
	}
	if( 3695 < info->calibrate_val.vbat_input && info->calibrate_val.vbat_input < 3705){
		pm860x_reg_write(info->i2c,PM8607_CHG_CTRL5, info->calibrate_val.vbat_4c[1]);
        pm860x_reg_write(info->i2c,PM8607_CHG_CTRL1, info->calibrate_val.vbat_48[1]);
		pm860x_reg_write(info->chip->companion, PM8606_PREREGULATORA,info->calibrate_val.vbat_temp[1]);
        }
	if( 4195  < info->calibrate_val.vbat_input && info->calibrate_val.vbat_input < 4205){
		pm860x_reg_write(info->i2c,PM8607_CHG_CTRL5, info->calibrate_val.vbat_4c[2]);
		pm860x_reg_write(info->i2c,PM8607_CHG_CTRL1, info->calibrate_val.vbat_48[2]);
		pm860x_reg_write(info->chip->companion, PM8606_PREREGULATORA,info->calibrate_val.vbat_temp[2]);
	}
	len = sprintf(buf, "%d\n", value);
	return len;
}
static ssize_t trim_ibat_remain_show_attrs(struct device *dev,
					      struct device_attribute *attr,
					      char *buf)
{
	struct pm860x_battery_info *info = dev_get_drvdata(dev);
	int len;

	len = sprintf(buf, "%d\n", info->offset_ibat);
	return len;
}

static ssize_t battery_vol_store_attrs(struct device *dev,
					      struct device_attribute *attr,
					      const char *buf, size_t count)
{
	char *last = NULL;
	struct pm860x_battery_info *info = dev_get_drvdata(dev);

	info->calibrate_val.vbat_input = simple_strtoul(buf, &last, 0);
	dev_dbg(info->dev, "%s,vol input = %d \n", __func__,info->calibrate_val.vbat_input);
	info->disable_rf();
	if( 3195 < info->calibrate_val.vbat_input && info->calibrate_val.vbat_input < 3205){
		info->calibrate_val.vbat_4c[0] = pm860x_reg_read(info->i2c,PM8607_CHG_CTRL5);
		pm860x_reg_write(info->i2c,PM8607_CHG_CTRL5,(info->calibrate_val.vbat_4c[0] &0xF0)|0x8);//force supply from charger
		info->calibrate_val.vbat_48[0] = pm860x_reg_read(info->i2c,PM8607_CHG_CTRL1);
		pm860x_reg_write(info->i2c,PM8607_CHG_CTRL1, info->calibrate_val.vbat_48[0]&0xFC);
		info->calibrate_val.vbat_temp[0] = pm860x_reg_read(info->chip->companion, PM8606_PREREGULATORA);
		pm860x_reg_write(info->chip->companion, PM8606_PREREGULATORA,(0xF0)|0x0E);
	}
	if(3695< info->calibrate_val.vbat_input && info->calibrate_val.vbat_input < 3705){
		info->calibrate_val.vbat_4c[1] = pm860x_reg_read(info->i2c,PM8607_CHG_CTRL5);
        pm860x_reg_write(info->i2c,PM8607_CHG_CTRL5,(info->calibrate_val.vbat_4c[1] &0xF0)|0x8);
		info->calibrate_val.vbat_48[1] = pm860x_reg_read(info->i2c,PM8607_CHG_CTRL1);
        pm860x_reg_write(info->i2c,PM8607_CHG_CTRL1, info->calibrate_val.vbat_48[1]&0xFC);
		info->calibrate_val.vbat_temp[1] = pm860x_reg_read(info->chip->companion, PM8606_PREREGULATORA);
		pm860x_reg_write(info->chip->companion, PM8606_PREREGULATORA,(0xA0)|0x0E);

	}
	if( 4195 < info->calibrate_val.vbat_input && info->calibrate_val.vbat_input < 4205){
		info->calibrate_val.vbat_4c[2] = pm860x_reg_read(info->i2c,PM8607_CHG_CTRL5);
		pm860x_reg_write(info->i2c,PM8607_CHG_CTRL5, (info->calibrate_val.vbat_4c[2]&0xF0)|0x8);
		info->calibrate_val.vbat_48[2] = pm860x_reg_read(info->i2c,PM8607_CHG_CTRL1);
        pm860x_reg_write(info->i2c,PM8607_CHG_CTRL1, info->calibrate_val.vbat_48[2]&0xFC);
		info->calibrate_val.vbat_temp[2] = pm860x_reg_read(info->chip->companion, PM8606_PREREGULATORA);
		pm860x_reg_write(info->chip->companion, PM8606_PREREGULATORA,(0x50)|0x0E);
	}
	return count;
}

static ssize_t calibration_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct pm860x_battery_info *info = dev_get_drvdata(dev);
	int len;
	len = sprintf(buf, "Battery Max Capacity: %d(mAh)\n", info->max_capacity);
	len += sprintf(buf + len, "Battery Internal Resistor: %d(omh)\n",
			info->resistor);
	len += sprintf(buf + len, "d4: 0x%x, d5: 0x%x, d7: 0x%x\n",
		info->nvram_data.d4value,info->nvram_data.d5value,
		info->nvram_data.d7value);
	len += sprintf(buf + len, "ibat_offset: %d, vbat_offset: %d\n",
		info->nvram_data.ibat_offset,info->nvram_data.vbat_offset);
	len += sprintf(buf + len, "vbat_slope_low %d, vbat_slope_high :%d\n",
		info->nvram_data.vbat_slope_low,info->nvram_data.vbat_slope_high);
	return len;
}

static ssize_t calibration_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct pm860x_battery_info *info = dev_get_drvdata(dev);
	char *s, *sub, flag = 0;
	unsigned long data = 0;
	unsigned long d7_trim = 0,d7_temp = 0 ;
	long value;

	for (s = (char *)buf; s;) {
		sub = strsep(&s, " \t");
		if (!sub)
			break;
		if (flag == 'c') {
			flag = 0;
			if (strict_strtoul(sub, 10, &data))
				dev_warn(dev, "Wrong max capacity is "
					"assigned!\n");
			mutex_lock(&info->lock);
			info->max_capacity = data;
			mutex_unlock(&info->lock);
			break;
		}
		if (flag == 'r') {
			flag = 0;
			if (strict_strtoul(sub, 10, &data))
				dev_warn(dev, "Wrong resistor is assigned!\n");
			mutex_lock(&info->lock);
			info->resistor = data;
			mutex_unlock(&info->lock);
			break;
		}
		if (flag == '1') {
			flag = 0;
			if (strict_strtoul(sub, 10, &data))
				dev_warn(dev, "Wrong d4value is assigned!\n");
			info->nvram_data.d4value = data;
			break;
		}
		if (flag == '2') {
			flag = 0;
			if (strict_strtoul(sub, 10, &data))
				dev_warn(dev, "Wrong d5value is assigned!\n");
			info->nvram_data.d5value = data;
			break;
		}
		if (flag == '3') {
			flag = 0;
			if (strict_strtoul(sub, 10, &d7_trim))
				dev_warn(dev, "Wrong d7value is assigned!\n");
				d7_temp = pm860x_page_reg_read(info->i2c,0xD7);
			d7_temp = d7_temp&0x0F;
			info->nvram_data.d7value = (d7_trim&0xf0)|d7_temp; //D7 lsb 4bit don't change
			pm860x_page_reg_write(info->i2c, 0xD4 , info->nvram_data.d4value);
			pm860x_page_reg_write(info->i2c, 0xD5 , info->nvram_data.d5value);
			pm860x_page_reg_write(info->i2c, 0xD7 , info->nvram_data.d7value);
			break;
		}
		if (flag == '4') {
			flag = 0;
			if (strict_strtol(sub, 10, &value))
				dev_warn(dev, "Wrong ibat_offset is assigned!\n");
			info->nvram_data.ibat_offset = value;
			break;
		}
		if (flag == '5') {
			flag = 0;
			if (strict_strtol(sub, 10, &value))
				dev_warn(dev, "Wrong vbat_offsetis assigned!\n");
			info->nvram_data.vbat_offset = value;
			break;
		}
		if (flag == '6') {
			flag = 0;
			if (strict_strtoul(sub, 10, &data))
				dev_warn(dev, "Wrong vbat_slope_low is assigned!\n");
			info->nvram_data.vbat_slope_low = data;
			break;
		}
		if (flag == '7') {
			flag = 0;
			if (strict_strtoul(sub, 10, &data))
				dev_warn(dev, "Wrong vbat_slope_high is assigned!\n");
			info->nvram_data.vbat_slope_high = data;
			break;
		}
		if (!strcasecmp(sub, "d4value")) {
			flag = '1';
			continue;
		}
		if (!strcasecmp(sub, "d5value")) {
			flag = '2';
			continue;
		}
		if (!strcasecmp(sub, "d7value")) {
			flag = '3';
			continue;
		}
		if (!strcasecmp(sub, "ibat_offset")) {
			flag = '4';
			continue;
		}
		if (!strcasecmp(sub, "vbat_offset")) {
			flag = '5';
			continue;
		}
		if (!strcasecmp(sub, "vbat_slope_low")) {
			flag = '6';
			continue;
		}
		if (!strcasecmp(sub, "vbat_slope_high")) {
			flag = '7';
			continue;
		}
	}
	return count;
}

static ssize_t calibartion_vbat_show_attrs(struct device *dev,
					      struct device_attribute *attr,
					      char *buf)
{
	u16 value;
	int vbat1 = 0 ,vbat2 = 0;
	unsigned char temp_4c,temp_48,i;
	int len;
	struct pm860x_battery_info *info = dev_get_drvdata(dev);

	temp_4c = pm860x_reg_read(info->i2c,PM8607_CHG_CTRL5);
	pm860x_reg_write(info->i2c,PM8607_CHG_CTRL5,(temp_4c &0xF0)|0x8);//force supply from charger
	temp_48 = pm860x_reg_read(info->i2c,PM8607_CHG_CTRL1);
	pm860x_reg_write(info->i2c,PM8607_CHG_CTRL1, temp_48&0xFC);
	/*for read vbat when power supplied from charger*/
	msleep(500);
	for (i = 0; i < 10; i++) {
		measure_vbatt(info, OCV_MODE_ACTIVE, &vbat2);
		vbat1 += vbat2;
	}
	value = vbat1/10;

	pm860x_reg_write(info->i2c,PM8607_CHG_CTRL5, temp_4c);
	pm860x_reg_write(info->i2c,PM8607_CHG_CTRL1, temp_48);

	len = sprintf(buf, "%d\n", value);
	return len;
}

static ssize_t calibartion_ibat_show_attrs(struct device *dev,
					      struct device_attribute *attr,
					      char *buf)
{
	int value, len;
	struct pm860x_battery_info *info = dev_get_drvdata(dev);
	measure_current(info, &value);
	len = sprintf(buf, "%d\n", value);
	return len;
}

static DEVICE_ATTR(trim_d5d7, S_IRUSR|S_IWUSR, battery_cali_show_attrs, battery_cali_store_attrs);
static DEVICE_ATTR(trim_d4, S_IRUSR|S_IWUSR, battery_info_show_attrs, battery_info_store_attrs);
static DEVICE_ATTR(trim_vol, S_IRUSR|S_IWUSR, battery_vol_show_attrs, battery_vol_store_attrs);
static DEVICE_ATTR(trim_return, S_IRUSR|S_IWUSR, battery_trimreturn_show_attrs, battery_trimreturn_store_attrs);
static DEVICE_ATTR(trim_ibat_remain, S_IRUSR, trim_ibat_remain_show_attrs, NULL);
static DEVICE_ATTR(calibration, S_IRUSR|S_IWUSR, calibration_show, calibration_store);
static DEVICE_ATTR(calibration_vbat, S_IRUSR, calibartion_vbat_show_attrs, NULL);
static DEVICE_ATTR(calibration_ibat, S_IRUSR, calibartion_ibat_show_attrs, NULL);
#ifdef VBAT_CALIB_OTHER
static DEVICE_ATTR(vbatcalibother, S_IRUSR|S_IWUSR,
					battery_show_vbat_calib_other,
					NULL);
static DEVICE_ATTR(numavg, S_IRUSR|S_IWUSR,
					battery_show_vbat_num_avg,
					battery_set_vbat_num_avg);
static DEVICE_ATTR(vbatavg, S_IRUSR|S_IWUSR,
					battery_show_vbat_avg,
					NULL);
static DEVICE_ATTR(vbatgain, S_IRUSR|S_IWUSR,
					battery_show_vbat_gain,
					battery_set_vbat_gain);
static DEVICE_ATTR(vbatoffset, S_IRUSR|S_IWUSR,
					battery_show_vbat_offset,
					battery_set_vbat_offset);
#endif


static struct attribute *battery_attributes[] = {
	&dev_attr_trim_d5d7.attr,
	&dev_attr_trim_d4.attr,
	&dev_attr_trim_vol.attr,
	&dev_attr_trim_return.attr,
	&dev_attr_trim_ibat_remain.attr,
	&dev_attr_calibration.attr,
	&dev_attr_calibration_vbat.attr,
	&dev_attr_calibration_ibat.attr,
#ifdef VBAT_CALIB_OTHER
	&dev_attr_vbatcalibother.attr,
	&dev_attr_numavg.attr,
	&dev_attr_vbatavg.attr,
	&dev_attr_vbatgain.attr,
	&dev_attr_vbatoffset.attr,
#endif
	NULL,
};
static struct attribute_group battery_attr_group = {
	.attrs = battery_attributes,
};

#ifdef CONFIG_ALTERNATE_CHARGER

int pm860x_default_bat1_handler(void)
{
	pr_info( \
	"[%s]pm860x_default_bat1_handler\n",__FILE__);
	return 1;
}

static irqreturn_t pm860x_default_bat2_handler(int irq, void *data)
{
	pr_info( \
	"[%s]pm860x_default_bat2_handler\n",__FILE__);
	return IRQ_HANDLED;
}
int pm860x_registerHealthCb(pm860xHealthCbFunc callBack)
{
	if ( callBack ) {
		healthCbFun = callBack;
		pr_info( \
		"88pm860x_battery.c:pm860x_registerHealthCb[%x]\n",(int)healthCbFun);
		return 0;
	} else { 
		pr_info( \
		"88pm860x_battery.c:pm860x_registerHealthCb fail\n");
		return 1;
	}
}
int pm860x_registerStatusCb(pm860xStatusCbFunc callBack)
{
	if ( callBack ) {
		statusCbFun = callBack;
		pr_info( \
		"88pm860x_battery.c:pm860x_registerStatusCb[%x]\n",(int)statusCbFun);
		return 0;
	} else { 
		pr_info( \
		"88pm860x_battery.c:pm860x_registerStatusCb fail\n");
		return 1;
	}
}
int pm860x_registerCapacityCb(pm860xCapacityCbFunc callBack)
{
	if ( callBack ) {
		capacityCbFun = callBack;
		pr_info( \
		"88pm860x_battery.c:pm860x_registerCapacityCb[%x]\n",(int)capacityCbFun);
		return 0;
	} else { 
		pr_info( \
		"88pm860x_battery.c:pm860x_registerCapacityCb fail\n");
		return 1;
	}
}
int pm860x_registerCoulombCb(pm860xCoulombCbFunc callBack)
{
	if ( callBack ) {
		coulombCbFun = callBack;
		pr_info( \
		"88pm860x_battery.c:pm860x_registerCoulombCb[%x]\n",(int)coulombCbFun);
		return 0;
	} else { 
		pr_info( \
		"88pm860x_battery.c:pm860x_registerCoulombCb fail\n");
		return 1;
	}
}
int pm860x_registerBattDetCb(pm860xBattDetCbFunc callBack)
{
	if ( callBack ) {
		battDetCbFun = callBack;
		pr_info( \
		"88pm860x_battery.c:pm860x_registerBattDetCb[%x]\n",(int)battDetCbFun);
		return 0;
	} else { 
		pr_info( \
		"88pm860x_battery.c:pm860x_registerBattDetCb fail\n");
		return 1;
	}
}
#endif 
static __devinit int pm860x_battery_probe(struct platform_device *pdev)
{
	struct pm860x_chip *chip = dev_get_drvdata(pdev->dev.parent);
	struct pm860x_battery_info *info;
	struct pm860x_power_pdata *pdata;
	int ret;

#ifdef CONFIG_ALTERNATE_CHARGER
	/*pm860x_registerHealthCb(pm860x_default_bat1_handler);*/
	/*pm860x_registerStatusCb(pm860x_default_bat1_handler);*/
	/*pm860x_registerCapacityCb(pm860x_default_bat1_handler);*/
	/*pm860x_registerCoulombCb(pm860x_default_bat2_handler);*/
	/*pm860x_registerBattDetCb(pm860x_default_bat2_handler);*/
	pm860x_registerHealthCb(spa_CallBack_Health);
	pm860x_registerStatusCb(spa_CallBack_Status);
	pm860x_registerCapacityCb(spa_CallBack_Capacity);
#else
    coulombCbFun = pm860x_coulomb_handler;
    battDetCbFun = pm860x_batt_handler;
#endif 
	
	info = kzalloc(sizeof(struct pm860x_battery_info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;
	info->irq_cc = platform_get_irq(pdev, 0);
	if (info->irq_cc < 0) {
		dev_err(&pdev->dev, "No IRQ resource!\n");
		ret = -EINVAL;
		goto out;
	}
	info->irq_batt = platform_get_irq(pdev, 1);
	if (info->irq_batt < 0) {
		dev_err(&pdev->dev, "No IRQ resource!\n");
		ret = -EINVAL;
		goto out;
	}

	info->chip = chip;
	info->i2c = (chip->id == CHIP_PM8607) ? chip->client : chip->companion;
	info->dev = &pdev->dev;
	info->status = POWER_SUPPLY_STATUS_UNKNOWN;
	info->tbat_mode = TBAT_CUR_MODE_LOW;
	pdata = pdev->dev.platform_data;
	info->disable_rf = pdata->disable_rf_fn;
	ginfo = info;

	if (coulombCbFun) {
		ret = request_threaded_irq(info->irq_cc, NULL, coulombCbFun,
					   IRQF_ONESHOT, "coulomb", info);
		if (ret < 0) {
			dev_err(chip->dev, "Failed to request IRQ: #%d: %d\n",
				info->irq_cc, ret);
			goto out;
		}
	} else
		pr_info( \
		"[%s]:no callback coulombCbFun\n",
		__FILE__);
	
	if (battDetCbFun) { 
		ret = request_threaded_irq(info->irq_batt, NULL, battDetCbFun,
					   IRQF_ONESHOT, "battery", info);
		if (ret < 0) {
			dev_err(chip->dev, "Failed to request IRQ: #%d: %d\n",
				info->irq_batt, ret);
			goto out_coulomb;
		}
	} else
		pr_info( \
		"[%s]:no callback battDetCbFun\n",
		__FILE__);

	mutex_init(&info->lock);
	platform_set_drvdata(pdev, info);

	pm860x_init_battery(info);

	info->battery.name = "88pm860x_battery";
	info->battery.type = POWER_SUPPLY_TYPE_BATTERY;
	info->battery.properties = pm860x_batt_props;
	info->battery.num_properties = ARRAY_SIZE(pm860x_batt_props);
	info->battery.get_property = pm860x_batt_get_prop;
	info->battery.external_power_changed = pm860x_external_power_changed;
	info->max_capacity = 1500;	/* set default capacity */
	info->resistor = 300;		/* set default internal resistor */
	ret = power_supply_register(&pdev->dev, &info->battery);
	if (ret)
		goto out_attr;
	info->battery.dev->parent = &pdev->dev;

	INIT_DELAYED_WORK_DEFERRABLE(&info->monitor_work, pm860x_battery_work);
	INIT_DELAYED_WORK(&info->changed_work, pm860x_changed_work);
	queue_delayed_work(chip->monitor_wqueue, &info->monitor_work,
			   MONITOR_INTERVAL);

	ret=sysfs_create_group(&pdev->dev.kobj, &battery_attr_group);
	if (ret < 0) {
		goto out_work;
	}

	device_init_wakeup(&pdev->dev, 1);
	return 0;

out_work:
	power_supply_unregister(&info->battery);
out_attr:
	device_remove_file(&pdev->dev, &dev_attr_calibration);
	free_irq(info->irq_batt, info);
out_coulomb:
	free_irq(info->irq_cc, info);
out:
	kfree(info);
	return ret;
}

static int __devexit pm860x_battery_remove(struct platform_device *pdev)
{
	struct pm860x_battery_info *info = platform_get_drvdata(pdev);

	cancel_delayed_work(&info->monitor_work);
	cancel_delayed_work(&info->changed_work);
	flush_workqueue(info->chip->monitor_wqueue);
	power_supply_unregister(&info->battery);
	device_remove_file(info->dev, &dev_attr_calibration);
	sysfs_remove_group(&pdev->dev.kobj, &battery_attr_group);
	free_irq(info->irq_batt, info);
	free_irq(info->irq_cc, info);
	kfree(info);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

#ifdef CONFIG_PM
static int pm860x_battery_suspend(struct device *dev)
{
	struct pm860x_battery_info *info = dev_get_drvdata(dev);

	calc_ccnt(info, &ccnt_data);
	cancel_delayed_work_sync(&info->monitor_work);

	if (device_may_wakeup(dev)) {
		enable_irq_wake(info->chip->core_irq);
		enable_irq_wake(info->irq_batt);
		enable_irq_wake(info->irq_cc);
	}
	return 0;
}

static int pm860x_battery_resume(struct device *dev)
{
	struct pm860x_battery_info *info = dev_get_drvdata(dev);

	if (device_may_wakeup(dev)) {
		disable_irq_wake(info->chip->core_irq);
		disable_irq_wake(info->irq_batt);
		disable_irq_wake(info->irq_cc);
	}
	calc_ccnt(info, &ccnt_data);
	queue_delayed_work(info->chip->monitor_wqueue, &info->monitor_work,
			   MONITOR_INTERVAL);
	return 0;
}

static struct dev_pm_ops pm860x_battery_pm_ops = {
	.suspend	= pm860x_battery_suspend,
	.resume		= pm860x_battery_resume,
};
#endif

static struct platform_driver pm860x_battery_driver = {
	.driver		= {
		.name	= "88pm860x-battery",
		.owner	= THIS_MODULE,
#ifdef CONFIG_PM
		.pm	= &pm860x_battery_pm_ops,
#endif
	},
	.probe		= pm860x_battery_probe,
	.remove		= __devexit_p(pm860x_battery_remove),
};

static int __init pm860x_battery_init(void)
{
	return platform_driver_register(&pm860x_battery_driver);
}
module_init(pm860x_battery_init);

static void __exit pm860x_battery_exit(void)
{
	platform_driver_unregister(&pm860x_battery_driver);
}
module_exit(pm860x_battery_exit);

MODULE_DESCRIPTION("Marvell 88PM860x Battery driver");
MODULE_LICENSE("GPL");
