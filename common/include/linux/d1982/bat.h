/*
 * d1982 Battery/Power module declarations.
  *
 * Copyright(c) 2011 Dialog Semiconductor Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 *
 */

#ifndef __LINUX_D1982_BAT_H
#define __LINUX_D1982_BAT_H

#include <linux/power_supply.h>
#include <linux/mutex.h>


/* STATIC CONFIGURATION */
#define BAT_MANUFACTURER			"Samsung"
#define BAT_TYPE				POWER_SUPPLY_TECHNOLOGY_LION
#define D1980_LOOK_UP_TABLE_SIZE		68
#define D1980_LOOKUP_TABLE_MAX			3

#define D1980_NUMBER_OF_STORE_CURENT_READING	4
#define D1980_ILLEGAL_BATTERY_DETECT		1


#define BAT_CAPACITY_LIMIT_LOW			4
#define BAT_CAPACITY_FULL			100
#define BAT_CAPACITY_LIMIT_HIGH			70
#define BAT_VOLTAGE_CUT_OFF			2800
#define BAT_WITH_NO_RESISTOR			62
/*
 * BAT Retries.
 */
#define D1980_MANUAL_READ_RETRIES	          5



static const u16 tbat_lookup_table[D1980_LOOKUP_TABLE_MAX] = {10, 25, 40};
static u32 const vbat_vs_capacity_look_up[D1980_LOOKUP_TABLE_MAX]
					[D1980_LOOK_UP_TABLE_SIZE][2] = {
	/* For temperature 10 degree celisus*/
	{
		{4102, 100}, {4065, 98},
		{4048, 96}, {4034, 95},
		{4021, 93}, {4011, 92},
		{4001, 90}, {3986, 88},
		{3968, 87}, {3952, 85},
		{3938, 84}, {3926, 82},
		{3916, 81}, {3908, 79},
		{3900, 77}, {3892, 76},
		{3883, 74}, {3774, 73},
		{3764, 71}, {3755, 70},
		{3646, 68}, {3636, 67},
		{3627, 65}, {3619, 64},
		{3610, 62}, {3601, 61},
		{3593, 59}, {3586, 58},
		{3578, 56}, {3562, 55},
		{3555, 53}, {3549, 52},
		{3530, 50}, {3528, 49},
		{3523, 47}, {3508, 46},
		{3493, 44}, {3488, 43},
		{3474, 41}, {3460, 40},
		{3456, 38}, {3442, 37},
		{3439, 35}, {3426, 34},
		{3413, 33}, {3401, 31},
		{3398, 30}, {3396, 28},
		{3393, 27}, {3330, 25},
		{3287, 24}, {3255, 22},
		{3245, 21}, {3222, 19},
		{3199, 18}, {3184, 17},
		{3171, 15}, {3166, 14},
		{3147, 12}, {3139, 11},
		{3130, 9}, {3121, 8},
		{3113, 6}, {3106, 5},
		{3097, 4}, {2982, 2},
		{2846, 1}, {2747, 0}
	},
	/* For temperature 25 degree celisus */
	{
		{4102, 100}, {4065, 98},
		{4048, 96}, {4034, 95},
		{4021, 93}, {4011, 92},
		{4001, 90}, {3986, 88},
		{3968, 87}, {3952, 85},
		{3938, 84}, {3926, 82},
		{3916, 81}, {3908, 79},
		{3900, 77}, {3892, 76},
		{3883, 74}, {3774, 73},
		{3764, 71}, {3755, 70},
		{3646, 68}, {3636, 67},
		{3627, 65}, {3619, 64},
		{3610, 62}, {3601, 61},
		{3593, 59}, {3586, 58},
		{3578, 56}, {3562, 55},
		{3555, 53}, {3549, 52},
		{3530, 50}, {3528, 49},
		{3523, 47}, {3508, 46},
		{3493, 44}, {3488, 43},
		{3474, 41}, {3460, 40},
		{3456, 38}, {3442, 37},
		{3439, 35}, {3426, 34},
		{3413, 33}, {3401, 31},
		{3398, 30}, {3396, 28},
		{3393, 27}, {3330, 25},
		{3287, 24}, {3255, 22},
		{3245, 21}, {3222, 19},
		{3199, 18}, {3184, 17},
		{3171, 15}, {3166, 14},
		{3147, 12}, {3139, 11},
		{3130, 9}, {3121, 8},
		{3113, 6}, {3106, 5},
		{3097, 4}, {2982, 2},
		{2846, 1}, {2747, 0}
	},
	/* For temperature 40 degree celisus*/
	{
		{4102, 100}, {4065, 98},
		{4048, 96}, {4034, 95},
		{4021, 93}, {4011, 92},
		{4001, 90}, {3986, 88},
		{3968, 87}, {3952, 85},
		{3938, 84}, {3926, 82},
		{3916, 81}, {3908, 79},
		{3900, 77}, {3892, 76},
		{3883, 74}, {3774, 73},
		{3764, 71}, {3755, 70},
		{3646, 68}, {3636, 67},
		{3627, 65}, {3619, 64},
		{3610, 62}, {3601, 61},
		{3593, 59}, {3586, 58},
		{3578, 56}, {3562, 55},
		{3555, 53}, {3549, 52},
		{3530, 50}, {3528, 49},
		{3523, 47}, {3508, 46},
		{3493, 44}, {3488, 43},
		{3474, 41}, {3460, 40},
		{3456, 38}, {3442, 37},
		{3439, 35}, {3426, 34},
		{3413, 33}, {3401, 31},
		{3398, 30}, {3396, 28},
		{3393, 27}, {3330, 25},
		{3287, 24}, {3255, 22},
		{3245, 21}, {3222, 19},
		{3199, 18}, {3184, 17},
		{3171, 15}, {3166, 14},
		{3147, 12}, {3139, 11},
		{3130, 9}, {3121, 8},
		{3113, 6}, {3106, 5},
		{3097, 4}, {2982, 2},
		{2846, 1}, {2747, 0}
	}
};

enum charge_status_enum {
	D1980_NONE = 1,
	D1980_CHARGING,
	D1980_DISCHARGING_WITH_CHARGER,
	D1980_DISCHARGING_WITHOUT_CHARGER
};

enum charger_type_enum {
	D1980_NOCHARGER = 1,
	D1980_USB_CHARGER,
	D1980_DCIN_CHARGER
};


struct d1980_bat_event_registration {
	u8	d1980_event_vddlow:1;
	u8	d1980_event_tbat:1;
};

struct d1980_bat_threshold {
	u16		vbatlow_mon_thr;
	u16		tbat_thr_min;
	u16		tbat_thr_max;
	u16		tbat_thr_limit;
	u16		tjunc_thr_limit;
};


struct d1980_bat_status {
	struct 		d1980_bat_platform_data  *bat_pdata;
	u8		cal_capacity;
	u8		charging_mode;
	u8		charger_type;
	u8		health;
	u8		status;
	u8		illegalbattery;
	u16 		vddout;
	u16		bat_voltage;
	u16		bat_temp;
};

struct d1980_nvram
{
    signed short    ibat_offset;
    signed short    vbat_offset;
    unsigned char   d4value;
    unsigned char   d5value;
    unsigned char   d7value;
    unsigned short  vbat_slope_low;
    unsigned short  vbat_slope_high;
};

struct d1980_calibrate
{
	unsigned char   ibat_4c;
	unsigned char   ibat_48;
	unsigned char   ibat_49;
	unsigned char   vbat_4c[3];
	unsigned char   vbat_48[3];
	unsigned char   vbat_temp[3];
	unsigned short  vbat_input;
};

struct d1980_power {
	int max_capacity;
	int resistor;
	struct platform_device 		*pdev;
	struct power_supply 		battery;
	struct power_supply 		usb;
	struct power_supply 		dcin;
	
	struct d1980_bat_status 	bat_status;
	struct d1980_bat_threshold	threshold; 	

	// For battery calibration
	struct d1980_nvram	        nvram_data;
	struct d1980_calibrate      calibrate_val;
};



static inline  u8 bat_temp_reg_to_C(u16 value) { return (55 - value); }
static inline  u8 bat_mV_to_reg(u16 value) { return (((value-4100)/100)<<4); }
static inline  u8 bat_drop_mV_to_reg(u16 value)
		{ return (((value-100)/100)<<6); }
static inline  u16 bat_reg_to_mV(u8 value) { return ((value*100) + 4100); }
static inline  u16 bat_drop_reg_to_mV(u8 value) { return ((value*100)+100); }


/* VDD OUT mV to register 8 bit resolution*/
static inline  u8 vbat_thr_mV_to_reg(u16 value) { return ((value - 2500) / 8); }
/* VDD OUT register to mV 8 bit resolution*/		
static inline  u16 vddout_reg_to_mV(u8 value) { return ((value * 8) + 2500); }
/* Result register to mV 10 bit resolution*/
static inline  u16 volt_bit_reg_to_mV(u16 value) { return ((value * 2) + 2500); }

// For Panther 
static inline  u16 volt_reg_to_mV(u16 value) { return ((((value * 610 * 100)/115) + 2400000)/1000); }
static inline  u16 volt_10bit_reg_to_mV(u16 value) { return ((((value * 630 * 100)/115) + 2400000)/1000); }
//static inline  u16 volt_10bit_reg_to_mV(u16 value) { return ((((value * 1510 * 100)/75) + 2400000)/1000); }


/* register 8 bit resolution*/
static inline  u8 bit8_mV_to_reg(u16 value) { return ((value * 2500) / 256000); }


#define D1980_BAT_DEBUG 			0

#define D1980_BAT_PROFILE			0
#define SUCCESS		                0
#define FAILURE		                1

#define TRUE		                1
#define FALSE		                0

#define set_bits(value, mask)		(value | mask)
#define clear_bits(value, mask)		(value & ~(mask))

#undef D1980_DEBUG
#if D1980_BAT_DEBUG
#define D1980_DEBUG( fmt, args... ) printk( KERN_CRIT "" fmt, ##args )
#else
#define D1980_DEBUG( fmt, args... )
#endif


/* SSC Read or Write Error */
#define D1980_SSC_FAIL				150

/* To enable debug output for your module, set this to 1 */
#define		D1980_SSC_DEBUG	0

#undef D1980_DEBUG
#if D1980_SSC_DEBUG
#define D1980_DEBUG( fmt, args... ) printk( KERN_CRIT "" fmt, ##args )
#else
#define D1980_DEBUG( fmt, args... )
#endif


#endif /* __LINUX_D1982_BAT_H */
