/*
 * Battery driver for Dialog D1980
 *   
 * Copyright(c) 2011 Dialog Semiconductor Ltd.
 *  
 * Author: Dialog Semiconductor Ltd. D. Chen, A Austin, D. Patel
 *  
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include <linux/delay.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/d1982/bat.h>
#include <linux/d1982/core.h>
#include <linux/d1982/d1980_reg.h>
#include <linux/d1982/batUtil.h>
#include <linux/kthread.h>

#include <mach/spa.h>

#define D1980_BATT_SUPPLY	                1
#define D1980_USB_SUPPLY	                2
#define D1980_DCIN_SUPPLY	                4

#define D1980_TBAT_OFFSET                   1

#define D1980_VBATLOW_DEFAULT_LIMIT         (3700) // (3380)
#define D1980_TBATMAX_DEFAULT_LIMIT         (25)
#define D1980_TBATMIN_DEFAULT_LIMIT         (118)

#define D1980_ACTIVE_THREAD_PRID            (10 * HZ)
#define D1980_SUSPEND_THREAD_PRID           (30 * HZ)

#ifdef CONFIG_SPA
#define CONFIG_BATTERY_D1980_OTHER_PSY
#endif
#define VBAT_CALIB_OTHER
#undef  USE_TEMP_THRESHOLD

// Read ADC ++
//typedef enum {
//    D1980_ADC_READ_AUTO = 0,
//    D1980_ADC_READ_MANUAL
//} adc_read_type;
// Read ADC --

static const char  __initdata banner[] = KERN_INFO "D1980 Power, (c) \
2011 Dialog semiconductor Ltd.\n";

static u16 BattVolt;    // Voltage
static u16 BattTBat;    // Temperature
static u16 BattVFBat;   // VF detection
static u8  TAdetect;    // TA detection
// Read ADC ++
static adc_read_type    adc_mode;
// Read ADC --

#ifdef VBAT_CALIB_OTHER
static long	vbatCalibOther;
static long	vbatNumAvg;
static long	vbatGain;
static long	vbatOffset;
#endif

#ifdef CONFIG_BATTERY_D1980_OTHER_PSY
typedef int (*charging_ctrl_cb_func)(int);

static charging_ctrl_cb_func charger_ctrl_cb;
#endif

static u32 threadPeriod = D1980_ACTIVE_THREAD_PRID;
static struct d1980 *d1980_g_client = NULL;

static char *supplied_to[] = {
	"battery",
};

#define AVERAGE_SIZE				3

#define DIFF(x, y, result)  				\
	do {                    				\
		if ((x) > (y)) 						\
			result = (x)-(y); 				\
		else 								\
			result = (y)-(x);   			\
	} while (0)

#define D1980_TBAT_MAX_THD(s, n)                    \
    ((n <=  (s->tbat_thr_max + D1980_TBAT_OFFSET))  \
        && (n >= (s->tbat_thr_max - D1980_TBAT_OFFSET)))
    
#define D1980_TBAT_MIN_THD(s, n)                    \
    ((n <=  (s->tbat_thr_min + D1980_TBAT_OFFSET))  \
        && (n >= (s->tbat_thr_min - D1980_TBAT_OFFSET)))

#undef D1982_DEBUG
#define USE_ADC_MANUAL_MODE
// Read ADC ++
#define ADC_USE_BOTH_MODE
// Read ADC --

#ifdef VBAT_STABILIZATION
static u8   d1980_power_suspended;
static void d1980_init_vbat_sampler(struct d1980 *d1980);
#endif


static void d1980_adjust_battery_voltage(u16 *pData, u16 *result)
{
	u8 i;
	u16 diff[3];

	DIFF(pData[1], pData[0], diff[0]);
	DIFF(pData[2], pData[0], diff[1]);
	DIFF(pData[2], pData[1], diff[2]);

	i = 0;
	if (diff[1] < diff[0]) {
		i++; diff[0] = diff[1];
	}
	if (diff[2] < diff[0])
		i += 2;

	if (i == 0) { 						/* Select diff[0] */
		*result = (pData[0]+pData[1])/2;
	} else if (i == 1) { 				/* Select diff[1] */
		*result = (pData[0]+pData[2])/2;
	} else { 							/* Select diff[2] */
		*result = (pData[1]+pData[2])/2;
	}
}

static int d1980_adc_read_battery_uvolts(struct d1980 * d1980)
{
	u16 Vbat_raw = 0;
	int vbat_base, data = 0;
    int vbatActiveOffset = 0;   // 02/Sep/2011. Additory calibration
	struct d1980_power *info = &d1980->power;

	if (d1980_adc_manual_conversion(d1980 , D1980_ADCMAN_MUXSEL_VBAT) == -EIO) {
        dev_warn(d1980->dev, "\n\nManual ADC Conversion ERROR \n");
        return -EIO;
	}

#ifdef VBAT_CALIB_OTHER
	data = volt_reg_to_mV(d1980->adc_res[D1980_ADCMAN_MUXSEL_VBAT].adc_buf);

	if (vbatCalibOther) {
		if (vbatGain > 0) {

		    // ++ 02/Sep/2011. Additory calibration
		    if(data >= 4100)
		        vbatActiveOffset = (((((vbatGain * 1000) / data) / 10 ) * 25)/1000);
            else if(data >= 4000 && data < 4100)
                vbatActiveOffset = (((((vbatGain * 1000) / data) / 10 ) * 35)/1000);
            else if(data >= 3800 && data < 4000)
		        vbatActiveOffset = (((((vbatGain * 1000) / data) / 10 ) * 48)/1000);
            else if(data >= 3600 && data < 3800)
		        vbatActiveOffset = (((((vbatGain * 1000) / data) / 10 ) * 109)/1000);
            else if(data >= 3500 && data < 3600)
		        vbatActiveOffset = (((((vbatGain * 1000) / data) / 10 ) * 121)/1000);
            else if(data >= 3400 && data < 3500)
                vbatActiveOffset = (((((vbatGain * 1000) / data) / 10 ) * 128)/1000);
            else if(data >= 3300 && data < 3400)
                vbatActiveOffset = (((((vbatGain * 1000) / data) / 10 ) * 138)/1000);
            else
		        vbatActiveOffset = (((((vbatGain * 1000) / data) / 10 ) * 205)/1000);
		    // ++ 02/Sep/2011. Additory calibration
            //printk(KERN_INFO "[D1980-POWER] MANUAL mode. vbatActiveOffset = %d \n", vbatActiveOffset);

			data = (((data << 12) - (vbatOffset << 8))/(vbatGain)) - vbatActiveOffset;
		}
		/* if gain <= 0 then don't use calibration*/
	}
	else
	{
		vbat_base =  3700 - info->nvram_data.vbat_offset;
		if (data <= 3700)
			data = (3700 * 1000  - (vbat_base - data) * info->nvram_data.vbat_slope_low)/1000;
		else
			data = (3700 * 1000  + (data - vbat_base) * info->nvram_data.vbat_slope_high)/1000;
	}

    Vbat_raw = data;
#else
	Vbat_raw = volt_reg_to_mV(d1980->adc_res[D1980_ADCMAN_MUXSEL_VBAT].adc_buf);
#endif


#ifdef D1982_DEBUG
	printk(KERN_INFO "[%s] Battery Read Raw data = %d mV -> Result = %d mV\n", "Power(Battery)", d1980->adc_res[D1980_ADCMAN_MUXSEL_VBAT].adc_buf, Vbat_raw);
#endif
	return (Vbat_raw);	
}


static int d1980_adc_read_battery_temperature_auto(struct d1980 * d1980)
{
	u16 readVal, temp_raw;

    	readVal = d1980_reg_read(d1980, D1980_TEMPRES_REG_MSB);
	if(readVal < 0)
	    return -EIO;
	temp_raw = ((readVal & 0x00FF) << 2);
	readVal = d1980_reg_read(d1980, D1980_AUTORES_REG_1);
	if(readVal < 0)
        return -EIO;
	temp_raw = temp_raw | ((readVal & 0x00C0) >> 6);

#ifdef D1982_DEBUG
	printk(KERN_INFO "[%s] Battery Temperature(auto) = %d \n", "Power(Battery)", temp_raw);
#endif

	return (temp_raw);	
}


static int d1980_adc_read_battery_uvolts_auto(struct d1980 * d1980)
{
    	u16 readVal, temp_raw, Vbat_raw = 0;
	int vbat_base, data = 0;
    int vbatActiveOffset = 0;   // 02/Sep/2011. Additory calibration
	struct d1980_power *info = &d1980->power;

	readVal = d1980_reg_read(d1980, D1980_VBATRES_REG_MSB);
	if(readVal < 0)
	    return -EIO;
	temp_raw = ((readVal & 0x00FF) << 2);
	readVal = d1980_reg_read(d1980, D1980_AUTORES_REG_1);
	if(readVal < 0)
        return -EIO;
	temp_raw = temp_raw | ((readVal & 0x000C) >> 2);

#ifdef VBAT_CALIB_OTHER
    temp_raw = (temp_raw << 2);
    //data = Vbat_raw = volt_10bit_reg_to_mV(temp_raw);
    data = Vbat_raw = volt_reg_to_mV(temp_raw);

	if (vbatCalibOther) {
		if (vbatGain > 0) {

		    // ++ 02/Sep/2011. Additory calibration
		    if(data >= 4000)
		        vbatActiveOffset = (((((vbatGain * 1000) / data) / 10 ) * 3)/100);
            else if(data >= 3800 && data < 4000)
		        vbatActiveOffset = (((((vbatGain * 1000) / data) / 10 ) * 6)/100);
            else if(data >= 3600 && data < 3800)
		        vbatActiveOffset = (((((vbatGain * 1000) / data) / 10 ) * 10)/100);
            else if(data >= 3400 && data < 3600)
		        vbatActiveOffset = (((((vbatGain * 1000) / data) / 10 ) * 12)/100);
            else if(data >= 3200 && data < 3400)
		        vbatActiveOffset = (((((vbatGain * 1000) / data) / 10 ) * 14)/100);
		    // -- 02/Sep/2011. Additory calibration
            //printk(KERN_INFO "[D1980-POWER] AUTO mode. vbatActiveOffset = %d \n", vbatActiveOffset);

			data = (((data << 12) - (vbatOffset << 8))/(vbatGain)) - vbatActiveOffset;
		}
		/* if gain <= 0 then don't use calibration*/
	}
	else
	{
		vbat_base =  3700 - info->nvram_data.vbat_offset;
		if (data <= 3700)
			data = (3700 * 1000  - (vbat_base - data) * info->nvram_data.vbat_slope_low)/1000;
		else
			data = (3700 * 1000  + (data - vbat_base) * info->nvram_data.vbat_slope_high)/1000;
	}

    Vbat_raw = data;

#ifdef D1982_DEBUG
    printk(KERN_INFO "[%s] Battery vbatCalibOther(%d). Voltage(auto) = %d mV, Vbat_raw = %d mV\n", "Power(Battery)", vbatCalibOther, temp_raw, Vbat_raw);
#endif
#else
	Vbat_raw = volt_10bit_reg_to_mV(temp_raw);
#endif
	return (Vbat_raw);	
}


static int d1980_read_battery_uvolts(struct d1980 *d1980)
{
	struct d1980_power *power = &d1980->power;
	struct d1980_bat_status *bat_status = &power->bat_status;
	u16 val, avg_value[AVERAGE_SIZE], i;
	int vBatRaw = 0;
	
	for (i = 0; i < AVERAGE_SIZE; i++) {
		if ((vBatRaw = d1980_adc_read_battery_uvolts(d1980)) == -EIO) {
			dev_warn(d1980->dev, "\n\n Manual d1980_read_battery_uvolts ERROR \n");
			return -EIO;
		}			
		avg_value[i] = (u16)vBatRaw;
	}
	d1980_adjust_battery_voltage(avg_value, &val);
	BattVolt = bat_status->bat_voltage = val;
#ifdef D1982_DEBUG
	printk(KERN_INFO "[%s] BattVolt(manual) Result = %d mV\n", "Power(Battery)", BattVolt);
#endif
	return (0);	
}


static int d1980_read_battery_uvolts_auto(struct d1980 *d1980)
{
	struct d1980_power *power = &d1980->power;
	struct d1980_bat_status *bat_status = &power->bat_status;
	u16 val, avg_value[AVERAGE_SIZE], i;
	int vBatRaw = 0;
	
	d1980_set_bits(d1980_g_client, D1980_ADCCONT_REG, (D1980_ADCCONT_AUTOADCEN | D1980_ADCCONT_AUTOVBATEN)); 
    msleep(1);

	for(i = 0; i < AVERAGE_SIZE; i++) {
		if((vBatRaw = d1980_adc_read_battery_uvolts_auto(d1980)) == -EIO) {
			dev_warn(d1980->dev, "\n\n Auto d1980_read_battery_uvolts_auto ERROR \n");
			return -EIO;
		}			
		avg_value[i] = (u16)vBatRaw;
		//printk(KERN_INFO "[D1980-POWER] ADC read voltage avg_value[%d] = %d\n", i, avg_value[i]);
		msleep(1);
	}
	d1980_adjust_battery_voltage(avg_value, &val);
	BattVolt  = bat_status->bat_voltage = val;
#ifdef D1982_DEBUG
	printk(KERN_INFO "[%s] BattVolt(auto) Result = %d mV\n", "Power(Battery)", BattVolt);
#endif

	return 0;	
}


// ++ 02/Sep/2011. Additory calibration
static int d1980_read_battery_calibration_auto(struct d1980 * d1980)
{
    	u16 readVal, temp_raw, Vbat_raw = 0;
	int vbat_base, data = 0;
	struct d1980_power *info = &d1980->power;

	readVal = d1980_reg_read(d1980, D1980_VBATRES_REG_MSB);
	if(readVal < 0)
	    return -EIO;
	temp_raw = ((readVal & 0x00FF) << 2);
	readVal = d1980_reg_read(d1980, D1980_AUTORES_REG_1);
	if(readVal < 0)
        return -EIO;
	temp_raw = temp_raw | ((readVal & 0x000C) >> 2);

#ifdef VBAT_CALIB_OTHER
    temp_raw = (temp_raw << 2);
    //data = Vbat_raw = volt_10bit_reg_to_mV(temp_raw);
    data = Vbat_raw = volt_reg_to_mV(temp_raw);

	if (vbatCalibOther) {
		if (vbatGain > 0) {
			data = (((data << 12) - (vbatOffset << 8))/(vbatGain));
		}
		/* if gain <= 0 then don't use calibration*/
	}
	else
	{
		vbat_base =  3700 - info->nvram_data.vbat_offset;
		if (data <= 3700)
			data = (3700 * 1000  - (vbat_base - data) * info->nvram_data.vbat_slope_low)/1000;
		else
			data = (3700 * 1000  + (data - vbat_base) * info->nvram_data.vbat_slope_high)/1000;
	}

    Vbat_raw = data;

#ifdef D1982_DEBUG
    printk(KERN_INFO "[%s] Battery vbatCalibOther(%d). Voltage(auto) = %d mV, Vbat_raw = %d mV\n", "Power(Battery)", vbatCalibOther, temp_raw, Vbat_raw);
#endif
#else
	Vbat_raw = volt_10bit_reg_to_mV(temp_raw);
#endif

	return (Vbat_raw);	
}


static int d1980_battery_calibration_auto(struct d1980 *d1980)
{
	struct d1980_power *power = &d1980->power;
	struct d1980_bat_status *bat_status = &power->bat_status;
	u16 val, avg_value[AVERAGE_SIZE], i;
	int vBatRaw = 0;
	
	d1980_set_bits(d1980_g_client, D1980_ADCCONT_REG, (D1980_ADCCONT_AUTOADCEN | D1980_ADCCONT_AUTOVBATEN)); 
    msleep(1);

	for(i = 0; i < AVERAGE_SIZE; i++) {
		if((vBatRaw = d1980_read_battery_calibration_auto(d1980)) == -EIO) {
			dev_warn(d1980->dev, "\n\n Auto d1980_read_battery_uvolts_auto ERROR \n");
			return -EIO;
		}			
		avg_value[i] = (u16)vBatRaw;
		//printk(KERN_INFO "[D1980-POWER] ADC read voltage avg_value[%d] = %d\n", i, avg_value[i]);
		msleep(1);
	}
	d1980_adjust_battery_voltage(avg_value, &val);
	BattVolt  = bat_status->bat_voltage = val;
#ifdef D1982_DEBUG
	printk(KERN_INFO "[%s] BattVolt(auto) Result = %d mV\n", "Power(Battery)", BattVolt);
#endif

	return 0;	
}


static int d1980_read_battery_calibration_manual(struct d1980 * d1980)
{
	u16 Vbat_raw = 0;
	int vbat_base, data = 0;
	struct d1980_power *info = &d1980->power;

	if (d1980_adc_manual_conversion(d1980 , D1980_ADCMAN_MUXSEL_VBAT) == -EIO) {
        dev_warn(d1980->dev, "\n\nManual ADC Conversion ERROR \n");
        return -EIO;
	}

#ifdef VBAT_CALIB_OTHER
	data = volt_reg_to_mV(d1980->adc_res[D1980_ADCMAN_MUXSEL_VBAT].adc_buf);

    printk(KERN_INFO "[D1980-POWER] vbatClibOther = %d\n", vbatCalibOther);
    
	if (vbatCalibOther) {
		if (vbatGain > 0) {
			data = (((data << 12) - (vbatOffset << 8))/(vbatGain));
		}
		/* if gain <= 0 then don't use calibration*/
	}
	else
	{
		vbat_base =  3700 - info->nvram_data.vbat_offset;
		if (data <= 3700)
			data = (3700 * 1000  - (vbat_base - data) * info->nvram_data.vbat_slope_low)/1000;
		else
			data = (3700 * 1000  + (data - vbat_base) * info->nvram_data.vbat_slope_high)/1000;
	}

    Vbat_raw = data;
#else
	Vbat_raw = volt_reg_to_mV(d1980->adc_res[D1980_ADCMAN_MUXSEL_VBAT].adc_buf);
#endif


#ifdef D1982_DEBUG
	printk(KERN_INFO "[%s] Battery Read Raw data = %d mV -> Result = %d mV\n", "Power(Battery)", d1980->adc_res[D1980_ADCMAN_MUXSEL_VBAT].adc_buf, Vbat_raw);
#endif
	return (Vbat_raw);	
}


static int d1980_battery_calibration_manual(struct d1980 *d1980)
{
	struct d1980_power *power = &d1980->power;
	struct d1980_bat_status *bat_status = &power->bat_status;
	u16 val, avg_value[AVERAGE_SIZE], i;
	int vBatRaw = 0;
	
	for (i = 0; i < AVERAGE_SIZE; i++) {
		if ((vBatRaw = d1980_read_battery_calibration_manual(d1980)) == -EIO) {
			dev_warn(d1980->dev, "\n\n Manual d1980_read_battery_uvolts ERROR \n");
			return -EIO;
		}			
		avg_value[i] = (u16)vBatRaw;
	}
	d1980_adjust_battery_voltage(avg_value, &val);
	BattVolt = bat_status->bat_voltage = val;
#ifdef D1982_DEBUG
	printk(KERN_INFO "[%s] BattVolt(manual) Result = %d mV\n", "Power(Battery)", BattVolt);
#endif
	return (0);	
}
// -- 02/Sep/2011. Additory calibration


void d1980_power_off(void)
{ 
   // printk(KERN_ERR "[D1980-POWER] start d1980_power_off...!\n");     

     d1980_reg_write(d1980_g_client, 0x46, 0x10);
    d1980_reg_write(d1980_g_client, 0x49, 0x00);
    d1980_set_bits(d1980_g_client, D1980_CONTROLB_REG, D1980_CONTROLB_SHUTDOWN); 
}

void d1980_ldo15A_set(int V16)
{
    if(V16)
    {
        d1980_set_bits(d1980_g_client, D1980_SUPPLY_REG, 1<<6);
    }
    else
    {
        d1980_clear_bits(d1980_g_client, D1980_SUPPLY_REG, 1<<6);
    }

}

void pass_battery_voltage(u16 * val)
{
    if(d1980_g_client)
    {
        d1980_read_battery_uvolts_auto(d1980_g_client);
    	*val = BattVolt;
#ifdef D1982_DEBUG
	    printk(KERN_INFO "[D1980-POWER] get voltage : %d\n", BattVolt);
#endif
	}
	else
	{
	    printk(KERN_ERR "[D1980-POWER] get voltage failed...!\n");
	}
}


// Read ADC ++
#ifdef ADC_USE_BOTH_MODE
u16 d1980_get_Voltage(void)
{
    int ret = 0;

    if(d1980_g_client)
    {
        if(adc_mode == D1980_ADC_READ_MANUAL) {
        ret = d1980_read_battery_uvolts(d1980_g_client);
        } else {
            ret = d1980_read_battery_uvolts_auto(d1980_g_client);
        }

        if(!ret)
        return BattVolt;
    }
    printk(KERN_ERR "[D1980-POWER] No device data pointer or failed get voltage ADC raw data!\n");
    
    return 0;
}
#else
// Read ADC --
u16 d1980_get_Voltage(void)
{
    int ret = 0;

    if(d1980_g_client)
    {
#ifdef USE_ADC_MANUAL_MODE
        ret = d1980_read_battery_uvolts(d1980_g_client);
#else
        ret = d1980_read_battery_uvolts_auto(d1980_g_client);
#endif /* USE_ADC_MANUAL_MODE */

        if(!ret)
            return BattVolt;
    }
    printk(KERN_ERR "[D1980-POWER] Get Voltage(AUTO) failed...!\n");
    
    return 0;
}
// Read ADC ++
#endif
// Read ADC --
EXPORT_SYMBOL(d1980_get_Voltage);

u8 d1980_get_chargerDetect(void)
{
    u8  readValue = 0;
    
    if(d1980_g_client)
    {
        readValue = d1980_reg_read(d1980_g_client, 0x1C);
        return readValue;
    }

    return 0;
}
EXPORT_SYMBOL(d1980_get_chargerDetect);

/* SAMSUNG_PPS: this is required for power off charging */
int d1980_set_chargerDetect(u8 data)
{
    int ret = 0;
    if(d1980_g_client)
    {
        ret = d1980_reg_write(d1980_g_client, 0x1C,data);
    }

    return ret;
}
EXPORT_SYMBOL(d1980_set_chargerDetect);

static u16 avg_calculation(u16 *avg_value)
{
	u16 tmp = 0, i;

	for (i = 0; i < AVERAGE_SIZE; i++)
		tmp = tmp + *(avg_value + i);

	return  (tmp/AVERAGE_SIZE);
}

static int d1980_battery_temperature_tbat(struct d1980 *d1980)
{
	struct d1980_power *power = &d1980->power;
	struct d1980_bat_status *bat_status = &power->bat_status;
	u16 avg_value[AVERAGE_SIZE], i;

	d1980_set_bits(d1980, D1980_ADCCONT_REG, D1980_ADCCONT_TEMPISRCEN);
	msleep(1);
	
	for(i = 0; i < AVERAGE_SIZE; i++) {
		if (d1980_adc_manual_conversion(d1980 , D1980_ADCMAN_MUXSEL_TEMP) == -EIO)
		{
            printk(KERN_INFO "D1980 Temperature ADC ERROR \n");
			return -EIO;		
		}
		avg_value[i] = d1980->adc_res[D1980_ADCMAN_MUXSEL_TEMP].adc_buf;
	}

	/* Average calculation */
	BattTBat = bat_status->bat_temp = avg_calculation(avg_value);

#ifdef D1982_DEBUG
	printk(KERN_INFO "[%s] BattTBat(manual) Result = %d mV\n", "Power(Battery)", BattTBat);
#endif

	return 0;
}


static int d1980_battery_temperature_tbat_auto(struct d1980 *d1980)
{
	struct d1980_power *power = &d1980->power;
	struct d1980_bat_status *bat_status = &power->bat_status;
	u16 avg_value[AVERAGE_SIZE], i;
	int tBatRaw = 0;

	d1980_set_bits(d1980, D1980_ADCCONT_REG, D1980_ADCCONT_TEMPISRCEN);
	msleep(1);

	for(i = 0; i < AVERAGE_SIZE; i++) {
		if((tBatRaw = d1980_adc_read_battery_temperature_auto(d1980)) == -EIO)
		{
            printk(KERN_INFO "D1980 Temperature ADC ERROR \n");
			return -EIO;		
		}
		avg_value[i] = tBatRaw;
		msleep(1);
	}

	BattTBat = bat_status->bat_temp = avg_calculation(avg_value);

#ifdef D1982_DEBUG
	printk(KERN_INFO "[%s] BattTBat(auto) Result = %d mV\n", "Power(Battery)", BattTBat);
#endif

	return 0;
}

// Read ADC ++
#ifdef ADC_USE_BOTH_MODE
u16 d1980_get_Temperature(void)
{
    int ret = 0;

    if(d1980_g_client)
    {
        if(adc_mode == D1980_ADC_READ_MANUAL) {
            ret = d1980_battery_temperature_tbat(d1980_g_client);
        } else {
            ret = d1980_battery_temperature_tbat_auto(d1980_g_client);
	}

        if(!ret)
            return BattTBat;
	}
    printk(KERN_ERR "[D1980-POWER] No device data pointer or failed get temperature ADC raw data!\n");
    
    return 0;
}
#else
// Read ADC --
u16 d1980_get_Temperature(void)
{
    int ret = 0;

    if(d1980_g_client)
    {
#ifdef USE_ADC_MANUAL_MODE
        ret = d1980_battery_temperature_tbat(d1980_g_client);
#else
        ret = d1980_battery_temperature_tbat_auto(d1980_g_client);
#endif
        if(!ret)
            return BattTBat;
    }
    printk(KERN_ERR "[D1980-POWER] Failure get temperature...!\n");
    
    return 0;
}
// Read ADC ++
#endif
// Read ADC --
EXPORT_SYMBOL(d1980_get_Temperature);


static int  d1980_battery_detection_vf(struct d1980 *d1980)
{
	u16 avg_value[AVERAGE_SIZE], i, val = 0;

	d1980_set_bits(d1980, D1980_ADCCONT_REG, D1980_ADCCONT_VFISRCEN);
	d1980_clear_bits(d1980, D1980_ADCCONT_REG, D1980_ADCCONT_AUTOVFEN);
	msleep(1);

	for(i = 0; i < AVERAGE_SIZE; i++){
		if (d1980_adc_manual_conversion(d1980 , D1980_ADCMAN_MUXSEL_VF) == -EIO)
			return -EIO;
		avg_value[i] = d1980->adc_res[D1980_ADCMAN_MUXSEL_VF].adc_buf;
	}

	/* Average calculation */
	d1980_adjust_battery_voltage(avg_value, &val);
	BattVFBat = val;

#ifdef D1982_DEBUG
	printk(KERN_INFO "[%s] BattVF(manual) Result = %d mV\n", "Power(Battery)", BattVFBat);
#endif

	return 0;
}


static int  d1980_adc_read_battery_detection_vf_auto(struct d1980 *d1980)
{
	u16 readVal, temp_raw;

	readVal = d1980_reg_read(d1980, D1980_VREF_REG_MSB);
	if(readVal < 0)
		return -EIO;		
	temp_raw = ((readVal & 0x00FF) << 2);	
	readVal = d1980_reg_read(d1980, D1980_AUTORES_REG_2);	
	if(readVal < 0)
       	return -EIO;
	temp_raw = temp_raw | ((readVal & 0x000C) >> 2);
	
	return (temp_raw);
}

static int  d1980_battery_detection_vf_auto(struct d1980 *d1980)
{
	struct d1980_power *power = &d1980->power;
	struct d1980_bat_status *bat_status = &power->bat_status;
	u16 avg_value[AVERAGE_SIZE], i, val = 0;
	int vfBatRaw = 0;

	d1980_set_bits(d1980, D1980_ADCCONT_REG, D1980_ADCCONT_AUTOVFEN  |  D1980_ADCCONT_VFISRCEN );
	msleep(1);

	for(i = 0; i < AVERAGE_SIZE; i++) {
		if((vfBatRaw = d1980_adc_read_battery_detection_vf_auto(d1980)) == -EIO)
		{
            dev_err(d1980->dev, " Battery detection VF  ADC ERROR \n", avg_value[i]);
            return -EIO;		
		}
		avg_value[i] = vfBatRaw;
	}

	/* Average calculation */
	d1980_adjust_battery_voltage(avg_value, &val);
	BattVFBat = val;

#ifdef D1982_DEBUG
	printk(KERN_INFO "[%s] BattVF(auto) Result = %d mV\n", "Power(Battery)", BattVFBat);
#endif

	return 0;
}

// Read ADC ++
#ifdef ADC_USE_BOTH_MODE
u16 d1980_get_VFdetection(void)
{
    int ret;

    if(d1980_g_client)
    {
        if(adc_mode == D1980_ADC_READ_MANUAL) {
        ret = d1980_battery_detection_vf(d1980_g_client);
        } else {
            ret = d1980_battery_detection_vf_auto(d1980_g_client);
        }

        if(!ret)
            return BattVFBat;
    }

    printk(KERN_ERR "[D1980-POWER] No device data pointer or Failed get VFdetection ADC raw data...!\n");

    return 0;
}
#else
// Read ADC --
u16 d1980_get_VFdetection(void)
{
    int ret;

    if(d1980_g_client)
    {
#ifdef USE_ADC_MANUAL_MODE
        ret = d1980_battery_detection_vf(d1980_g_client);
#else
        ret = d1980_battery_detection_vf_auto(d1980_g_client);
#endif
        if(!ret)
            return BattVFBat;

    }
    printk(KERN_ERR "[D1980-POWER] Get Battery VF detection(AUTO) failed...!\n");

    return 0;
}
#endif /* ADC_USE_BOTH_MODE */
// Read ADC --
EXPORT_SYMBOL(d1980_get_VFdetection);


static void d1980_set_TAdetect(const u8 state)
{
    TAdetect = state;
#ifdef D1982_DEBUG
    printk(KERN_INFO "- TA Detection state is %s\n", state ? "TRUE" : "FALSE");
#endif
}

u8 d1980_get_TAdetect(void)
{
    u8  value;
 
    if(d1980_g_client)
    {
        printk(KERN_INFO "-Get Travel Adapter Status\n");
        value = d1980_reg_read(d1980_g_client, D1980_STATUSC_REG);
        value = (value & D1980_STATUSC_TA_LEVEL);

        printk(KERN_INFO "-Get Travel Adapter Status. Read value = %d, Tadetect value = %d\n", value, TAdetect);

        if(value != TAdetect)
            TAdetect = value;
    }

    return TAdetect;
}
EXPORT_SYMBOL(d1980_get_TAdetect);

u8 d1980_set_adcIn_enable(u8 on)
{
    u8  value;
 
    if(d1980_g_client)
    {
        if(on)
        {
       	    d1980_set_bits(d1980_g_client, D1980_ADCCONT_REG, D1980_ADCCONT_AUTOAINEN);
       	}
       	else
       	{
            d1980_clear_bits(d1980_g_client, D1980_ADCCONT_REG, D1980_ADCCONT_AUTOAINEN);
       	}
    }
	return 1;
}
EXPORT_SYMBOL(d1980_set_adcIn_enable);

u8 d1980_set_nSleep_enable(u8 on)
{
    u8  value;

    if(d1980_g_client)
    {
        if(on) {
       	    d1980_set_bits(d1980_g_client, D1980_POWERCONT_REG, D1980_POWERCONT_nSLEEP);
       	} else {
       	    d1980_clear_bits(d1980_g_client, D1980_POWERCONT_REG, D1980_POWERCONT_nSLEEP);
       	}
}
    return 1;
}
EXPORT_SYMBOL(d1980_set_nSleep_enable);

int d1980_adc_in_read_auto(void)
{
	u16 readVal, temp_raw;

	readVal = d1980_reg_read(d1980_g_client, D1980_ADCINRES_REG_MSB);
	if(readVal < 0)
		return -EIO;		
	temp_raw = ((readVal & 0x00FF) << 2);	
	readVal = d1980_reg_read(d1980_g_client, D1980_AUTORES_REG_2);	
	if(readVal < 0)
       	return -EIO;
	temp_raw = temp_raw | ((readVal & 0x000C) >> 2);
#ifdef D1982_DEBUG
    printk(KERN_INFO "[%s] ADC IN (auto) = %d \n", "Auido", temp_raw);
#endif
	return (temp_raw);
}
EXPORT_SYMBOL(d1980_adc_in_read_auto);

static u32 interpolated( u32 vbat_lower, u32  vbat_upper, u32  level_lower,
				u32  level_upper, u32 vbat)
{
	s32 temp;

	/*apply formula y= yk + (x ?xk) * (yk+1 ?yk)/(xk+1 - xk) */
	temp = ((level_upper - level_lower) * 1000)/(vbat_upper - vbat_lower);
	temp = level_lower + (((vbat -vbat_lower) * temp)/1000);

	return temp;
}

static u8 select_temperature(u8 temp_index, u16 tbat)
{
	u16 temp_temp = 0;
	temp_temp = (tbat_lookup_table[temp_index] +
				tbat_lookup_table[temp_index+1]) / 2;

	if(tbat >= temp_temp)
		return (temp_index + 1);
	else
		return (temp_index);
}

// Read ADC
static s32 d1980_get_bat_level(struct d1980 *d1980)
{
	struct d1980_power *power = &d1980->power;
	struct d1980_bat_status *bat_status = &power->bat_status;
	u16 avg_value[AVERAGE_SIZE];
	int ret;
	
	u16 tbat;
	u16 vbat;
	u32 vbat_lower = 0, vbat_upper = 0, level_upper = 0, level_lower,level = 0;
	u8 access_index = 0, index = 0, flag = FALSE;

	// Read ADC
	if(adc_mode == D1980_ADC_READ_AUTO)
	{
    	ret = d1980_read_battery_uvolts_auto(d1980); /* value in mV */
    	if (ret < 0) 
            return ret;

    	ret = d1980_battery_temperature_tbat_auto(d1980);
    	if (ret < 0)
    	    return ret;
	}
	else
	{
    	ret = d1980_read_battery_uvolts(d1980); /* value in mV */
    	if (ret < 0) 
            return ret;

    	ret = d1980_battery_temperature_tbat(d1980);
    	if (ret < 0) 
            return ret;
	}

	//tbat = bat_temp_reg_to_C(bat_status->bat_temp);
	vbat = BattVolt;
	tbat = BattTBat;

	for (index = 0; index < (D1980_LOOKUP_TABLE_MAX-1); index++) {
		if (tbat <= tbat_lookup_table[0]) {
			access_index=0;
			break;
		}
		else if (tbat > tbat_lookup_table[D1980_LOOKUP_TABLE_MAX-1]){
			access_index = D1980_LOOKUP_TABLE_MAX -1;
			break;
		}
		else if ((tbat >= tbat_lookup_table[index])
			&& (tbat >= tbat_lookup_table[index+1])){
			access_index = select_temperature(index,tbat);
			break;
		}
	}
	if (vbat >= vbat_vs_capacity_look_up[access_index][0][0]) {
		bat_status->cal_capacity =100;
		//dev_warn(d1980->dev, " Voltage CAPACITY =  100\n");
		return 0;
	}
	if (vbat <=
		vbat_vs_capacity_look_up[access_index][D1980_LOOK_UP_TABLE_SIZE-1][0]){
		//dev_warn(d1980->dev, " Voltage CAPACITY =  0\n");
		bat_status->cal_capacity =0;
			return 0;
	}
	flag = FALSE;

	for (index = 0; index < (D1980_LOOK_UP_TABLE_SIZE - 1); index++) {
		if ((vbat <= vbat_vs_capacity_look_up[access_index][index][0]) &&
			(vbat >= vbat_vs_capacity_look_up[access_index][index+1][0])) {
			vbat_upper = vbat_vs_capacity_look_up[access_index][index][0];
			vbat_lower = vbat_vs_capacity_look_up[access_index][index+1][0];
			level_upper = vbat_vs_capacity_look_up[access_index][index][1];
			level_lower = vbat_vs_capacity_look_up[access_index][index+1][1];
			flag = TRUE;
			break;
		}
	}
	if (!flag)
		return -EIO;

	level = interpolated(vbat_lower, vbat_upper, level_lower, level_upper, vbat);
	bat_status->cal_capacity = level;
#ifdef D1982_DEBUG
	printk(KERN_INFO " TOTAl_BAT_CAPACITY : %d\n", bat_status->cal_capacity);
#endif

	return 0;
}


static int d1980_batt_status(struct d1980 *d1980)
{
	struct d1980_power *power = &d1980->power;
	struct d1980_bat_status *bat_status = &power->bat_status;
#ifdef CONFIG_BATTERY_D1980_OTHER_PSY
	struct power_supply *psy = NULL;
	union power_supply_propval psy_data;

	psy = power_supply_get_by_name(supplied_to[0]);
	if(psy == NULL)
		return -ENODATA;

	if(psy->get_property(psy, POWER_SUPPLY_PROP_STATUS, &psy_data))
		return -ENODATA;

    bat_status->status = psy_data.intval;

    if (bat_status->status == POWER_SUPPLY_STATUS_CHARGING) {
		return POWER_SUPPLY_STATUS_CHARGING;
	} else { 
		return POWER_SUPPLY_STATUS_DISCHARGING;
	}
#else
	if (bat_status->status == D1980_CHARGING) {
		return POWER_SUPPLY_STATUS_CHARGING;
	} else { 
		return POWER_SUPPLY_STATUS_DISCHARGING;
	}
#endif
}


#ifdef CONFIG_SYSFS
//++ Charger control 04/Aug/2011
#ifdef CONFIG_BATTERY_D1980_OTHER_PSY
static ssize_t control_charging(struct device *dev,
	            struct device_attribute *attr, const char *buf, size_t count)
{
	struct d1980 *d1980 = dev_get_drvdata(dev);
	struct d1980_power *power = &d1980->power;
	int input;

	if (!sscanf(buf, "%d", &input))
		return strnlen(buf, PAGE_SIZE);

	mutex_lock(&d1980->charger_lock);
	charger_ctrl_cb(input);
	mutex_unlock(&d1980->charger_lock);
	dev_dbg(d1980->dev, \
        		"alternate control charger: %s\n",\
        		(input) ? "stop" : "start");

	return strnlen(buf, PAGE_SIZE);
}
#endif /* CONFIG_BATTERY_D1980_OTHER_PSY */
//-- Charger control 04/Aug/2011

static ssize_t charger_state_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct d1980 *d1980 = dev_get_drvdata(dev);	
	struct d1980_power *power = &d1980->power;
	struct d1980_bat_status *bat_status = &power->bat_status;
	char *charge;

#ifdef CONFIG_BATTERY_D1980_OTHER_PSY
	return sprintf(buf, "%d\n", bat_status->status);
#else
	if (bat_status->status == D1980_CHARGING) {
		charge = "Charger Connected";
	} else { 
		charge = "Charger Off";
	}
	
	return sprintf(buf, "%s\n", charge);
#endif
}

static ssize_t calibration_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	//struct pm860x_battery_info *info = dev_get_drvdata(dev);
	struct d1980 *d1980 = dev_get_drvdata(dev);
	struct d1980_power *info = &d1980->power;
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
	//struct pm860x_battery_info *info = dev_get_drvdata(dev);
	struct d1980 *d1980 = dev_get_drvdata(dev);
	struct d1980_power *info = &d1980->power;
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
			mutex_lock(&d1980->battery_lock);
			info->max_capacity = data;
			mutex_unlock(&d1980->battery_lock);
			break;
		}
		if (flag == 'r') {
			flag = 0;
			if (strict_strtoul(sub, 10, &data))
				dev_warn(dev, "Wrong resistor is assigned!\n");
			mutex_lock(&d1980->battery_lock);
			info->resistor = data;
			mutex_unlock(&d1980->battery_lock);
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
            // TODO: Check...
			//d7_temp = pm860x_page_reg_read(info->i2c,0xD7);
			//d7_temp = d7_temp&0x0F;
			//info->nvram_data.d7value = (d7_trim&0xf0)|d7_temp; //D7 lsb 4bit don't change
			//pm860x_page_reg_write(info->i2c, 0xD4 , info->nvram_data.d4value);
			//pm860x_page_reg_write(info->i2c, 0xD5 , info->nvram_data.d5value);
			//pm860x_page_reg_write(info->i2c, 0xD7 , info->nvram_data.d7value);
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
	struct d1980 *d1980 = dev_get_drvdata(dev);
	adc_read_type temp_buf = adc_mode;

#ifdef D1982_DEBUG
    printk(KERN_INFO "[D1980(Power)] calibartion_vbat_show_attrs \n");
#endif

    d1980_set_adc_mode(D1980_ADC_READ_MANUAL);

#ifdef CONFIG_BATTERY_D1980_OTHER_PSY
    charger_ctrl_cb(0);
#endif

    // ++ 02/Sep/2011. Additory calibration
    if(adc_mode == D1980_ADC_READ_AUTO) {
    	for (i = 0; i < 6; i++) {
		    d1980_battery_calibration_auto(d1980);
            vbat1 += BattVolt;
        }
	} else {
	    for (i = 0; i < 6; i++) {
            d1980_battery_calibration_manual(d1980);
		    vbat1 += BattVolt;
	    }
	}
	value = vbat1/6;
	// -- 02/Sep/2011. Additory calibration

#ifdef CONFIG_BATTERY_D1980_OTHER_PSY
    charger_ctrl_cb(1);
#endif

    d1980_set_adc_mode(temp_buf);

	len = sprintf(buf, "%d\n", value);

	return len;
}

static ssize_t calibartion_ibat_show_attrs(struct device *dev,
					      struct device_attribute *attr,
					      char *buf)
{
	int value, len = 0;
	//struct pm860x_battery_info *info = dev_get_drvdata(dev);
	//measure_current(info, &value);
	//len = sprintf(buf, "%d\n", value);
	return len;
}

#ifdef VBAT_CALIB_OTHER
static ssize_t battery_show_vbat_calib_other(struct device *dev,
				struct device_attribute *attr, char *buf)
{
		pr_info( \
		"\battery_show_vbat_calib_other vbatCalibOther[%ld]\n", vbatCalibOther);
		return sprintf(buf, "%ld\n", vbatCalibOther);
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

static ssize_t battery_show_vbat_avg(struct device *dev,
				struct device_attribute *attr, char *buf)
{
    int ret, i, vbat = 0;
    u32 sum = 0;
	struct d1980 *d1980 = dev_get_drvdata(dev);	
	adc_read_type temp_buf = adc_mode;

    d1980_set_adc_mode(D1980_ADC_READ_MANUAL);

#ifdef CONFIG_BATTERY_D1980_OTHER_PSY
    charger_ctrl_cb(0);
#endif

    // ++ 02/Sep/2011. Additory calibration
    if (vbatNumAvg > 0) {
        if(adc_mode == D1980_ADC_READ_AUTO) {
            d1980_set_bits(d1980, D1980_ADCCONT_REG, D1980_ADCCONT_AUTOADCEN); 
            d1980_set_bits(d1980, D1980_ADCCONT_REG, D1980_ADCCONT_AUTOVBATEN); 
            msleep(1);

    	for (i = 0; i < vbatNumAvg; i++) {
        		ret = d1980_battery_calibration_auto(d1980);
        		if (ret)
    			dev_err(d1980->dev, \
            			"battery_show_vbat_avg. auto. :ret val wrong");

    		vbat = BattVolt;
    		pr_info( \
    		"battery_show_vbat_avg vbat[%d]\n", vbat);
    		sum += vbat;
    	}
        } else {
    	    for (i = 0; i < vbatNumAvg; i++) {
        		ret = d1980_battery_calibration_manual(d1980);
        		if (ret < 0)
    			dev_err(d1980->dev, \
            			"battery_show_vbat_avg. manual. :ret val wrong");

    		    vbat = BattVolt;
    		    pr_info("battery_show_vbat_avg vbat[%d]\n", vbat);
    		    sum += vbat;
    	    }
        }
    	vbat = sum/vbatNumAvg;
    }
    // -- 02/Sep/2011. Additory calibration

#ifdef CONFIG_BATTERY_D1980_OTHER_PSY
    charger_ctrl_cb(1);
#endif

    d1980_set_adc_mode(temp_buf);

    pr_info("\nbattery_show_vbat_avg vbatAvg[%d]from[%ld]measurments\n",\
                 vbat, vbatNumAvg);

    return sprintf(buf, "%d\n", vbat);
}

static ssize_t battery_show_vbat_gain(struct device *dev,
				struct device_attribute *attr, char *buf)
{
#ifdef D1982_DEBUG
    printk(KERN_INFO "\nbattery_show_vbat_gain vbatGain[%ld]\n", vbatGain);
#endif
    return sprintf(buf, "%ld\n", vbatGain);
}

static ssize_t battery_set_vbat_gain(struct device *dev,
struct device_attribute *attr, const char *buf, size_t count)
{
    strict_strtol(buf, 10, &vbatGain);
    /* if set gain then other vbat calib,will happen on init*/
    vbatCalibOther = 1;
#ifdef D1982_DEBUG
    printk(KERN_INFO "\nbattery_set_vbat_gain vbatGain[%ld]\n", vbatGain);
#endif
    return count;
}

static ssize_t battery_show_vbat_offset(struct device *dev,
		struct device_attribute *attr, char *buf)
{
#ifdef D1982_DEBUG
    printk(KERN_INFO "\nbattery_show_vbat_offset vbatOffset[%ld]\n", vbatOffset);
#endif
    return sprintf(buf, "%ld\n", vbatOffset);
}

static ssize_t battery_set_vbat_offset(struct device *dev,
struct device_attribute *attr, const char *buf, size_t count)
{
    strict_strtol(buf, 10, &vbatOffset);
#ifdef D1982_DEBUG
    printk(KERN_INFO "\nbattery_set_vbat_offset vbatOffset[%ld]\n", vbatOffset);
#endif
    return count;
}

//++ Charger control 04/Aug/2011
static DEVICE_ATTR(control, S_IWUSR, 
                    NULL, control_charging);
//-- Charger control 04/Aug/2011
static DEVICE_ATTR(vbatcalibother, S_IRUSR|S_IWUSR, \
					battery_show_vbat_calib_other, \
					NULL);
static DEVICE_ATTR(numavg, S_IRUSR|S_IWUSR, \
					battery_show_vbat_num_avg, \
					battery_set_vbat_num_avg);
static DEVICE_ATTR(vbatavg, S_IRUSR|S_IWUSR, \
					battery_show_vbat_avg, \
					NULL);
static DEVICE_ATTR(vbatgain, S_IRUSR|S_IWUSR, \
					battery_show_vbat_gain, \
					battery_set_vbat_gain);
static DEVICE_ATTR(vbatoffset, S_IRUSR|S_IWUSR, \
					battery_show_vbat_offset, \
					battery_set_vbat_offset);
#endif/* VBAT_CALIB_OTHER */

static DEVICE_ATTR(charger_state, S_IRUSR|S_IWUSR, \
                    charger_state_show, \
                    NULL);
static DEVICE_ATTR(calibration, S_IRUSR|S_IWUSR, \
                    calibration_show, \
                    calibration_store);
static DEVICE_ATTR(calibration_vbat, S_IRUSR, \
                    calibartion_vbat_show_attrs, \
                    NULL);
static DEVICE_ATTR(calibration_ibat, S_IRUSR, \
                    calibartion_ibat_show_attrs, \
                    NULL);

static struct attribute *battery_attributes[] = {
    &dev_attr_charger_state.attr,
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
#endif /*CONFIG_SYSFS */

static irqreturn_t d1980_charger_handler(int irq, void *data)
{
	struct d1980 *d1980 = data;
	struct d1980_power *power = &d1980->power;
	struct d1980_bat_status *bat_status = &power->bat_status;
    struct d1980_bat_threshold *threshold = &power->threshold;

/* + 20110627 SAMSUNG YJ.Choi : PLM Issue# P110624-3138*/
	struct power_supply *psy;

	psy = power_supply_get_by_name("battery");

	if (!psy) {
    		dev_info(d1980->dev, "\n\n\nNo Battery module\n");
		return IRQ_HANDLED;
	}
	/* - 20110627 SAMSUNG YJ.Choi : PLM Issue# P110624-3138*/


	switch (irq) {
    	case D1980_IRQ_EVBATLOW:
            dev_warn(d1980->dev,"########################################## \n" );
            dev_warn(d1980->dev, "-Battery LOW \n");
            dev_warn(d1980->dev,"########################################## \n" );
#ifndef CONFIG_BATTERY_D1980_OTHER_PSY
    		power_supply_changed(&power->battery);
#endif
            spa_voltThresholdEvent(1);
    		break;

    	case D1980_IRQ_ETBAT:
            dev_warn(d1980->dev,"########################################## \n" );
    		dev_warn(d1980->dev, "-Battery Temperature\n");
            dev_warn(d1980->dev,"########################################## \n" );
    		//power_supply_changed(&power->battery);
            spa_tempThresholdEvent(1);
    		break;

        case D1980_IRQ_ETA:
        {
            u8  value;

            dev_warn(d1980->dev,"########################################## \n" );
            dev_warn(d1980->dev, "-Travel Adapter Interrupt");
            dev_warn(d1980->dev,"########################################## \n" );
            value = d1980_reg_read(d1980, D1980_STATUSC_REG);
            printk(KERN_INFO "Value = 0x%x\n", value);
            d1980_set_TAdetect((value & D1980_STATUSC_TA_LEVEL) ? TRUE : FALSE);
            break;
        }
    		
    /*
    	case D1980_IRQ_EGPI1:
    		//dev_warn(d1980->dev, "\n\n\nCharger Event\n");
    		//power_supply_changed(&power->usb);
    		if (d1980_reg_read(d1980, D1980_GPIO0001_REG) & D1980_GPIO01_GPIO1TYPE)	{
    			bat_status->charger_type = D1980_USB_CHARGER;
    			bat_status->status = D1980_CHARGING;
    			d1980_clear_bits(d1980, D1980_GPIO0001_REG, D1980_GPIO01_GPIO1TYPE);	
    		} else {
    			bat_status->charger_type = D1980_NOCHARGER;
    			bat_status->status = D1980_DISCHARGING_WITHOUT_CHARGER;
    			d1980_set_bits(d1980, D1980_GPIO0001_REG, D1980_GPIO01_GPIO1TYPE);
    		}
    		break;
    */
    	default:
    		dev_err(d1980->dev, "Unknown interrupt %d\n", irq);
	}

	return IRQ_HANDLED;
}

/*********************************************************************
 *		Battery properties
 *********************************************************************/

static u8 d1980_bat_check_health(struct d1980 *d1980)
{
	struct d1980_power *power = &d1980->power;
	struct d1980_bat_status *bat_status = &power->bat_status;

	dev_warn(d1980->dev,"#[Power-Battery] d1980_bat_check_health : \n" );
	if (d1980_read_battery_uvolts(d1980) < 2850)
		return POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
	
	// Read ADC
	if (d1980_get_bat_level(d1980) != 0)
		return POWER_SUPPLY_HEALTH_UNKNOWN;
		
	if (bat_status->health != POWER_SUPPLY_HEALTH_OVERHEAT) {
		if (bat_status->illegalbattery)
			bat_status->health = POWER_SUPPLY_HEALTH_UNKNOWN;
		else if (bat_status->cal_capacity <
						BAT_CAPACITY_LIMIT_LOW)
			bat_status->health = POWER_SUPPLY_HEALTH_DEAD;
		else
			bat_status->health = POWER_SUPPLY_HEALTH_GOOD;
	}
	return bat_status->health;	
}


static int d1980_bat_get_property(struct power_supply *psy,
				   enum power_supply_property psp,
				   union power_supply_propval *val)
{
	struct d1980 *d1980 = dev_get_drvdata(psy->dev->parent);
	struct d1980_power *power = &d1980->power;
	struct d1980_bat_status *bat_status = &power->bat_status;
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = d1980_batt_status(d1980);
		if(val->intval < 0)
			return val->intval;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		if (bat_status->illegalbattery)
			val->intval = 0;
		else
			val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = BAT_TYPE;
    	break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = bat_status->bat_voltage;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = d1980_bat_check_health(d1980);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		// Read ADC
		if (d1980_get_bat_level(d1980) == 0)
			val->intval = bat_status->cal_capacity;
		else
			val->intval = 0;	
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = bat_status->bat_temp;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static enum power_supply_property d1980_bat_props[] = {
#ifdef CONFIG_BATTERY_D1980_OTHER_PSY
    //POWER_SUPPLY_PROP_STATUS,
    //POWER_SUPPLY_PROP_PRESENT,
    //POWER_SUPPLY_PROP_TECHNOLOGY,
    POWER_SUPPLY_PROP_VOLTAGE_NOW,
    POWER_SUPPLY_PROP_TEMP,
#else
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
#endif
};


/*********************************************************************
 *		Work Schedule
 *********************************************************************/
static void d1980_battery_work(struct work_struct *work)
{
	struct d1980 *d1980 = container_of(work , struct d1980, monitor_work.work);
	struct d1980_power *power = &d1980->power;
	
	dev_warn(d1980->dev,"########################## d1980_battery_work : \n" );
	// Read ADC
	d1980_get_bat_level(d1980);
	
#ifndef CONFIG_BATTERY_D1980_OTHER_PSY
	power_supply_changed(&power->battery);
#endif
	schedule_delayed_work(&d1980->monitor_work, HZ * 60);
}

/*********************************************************************
 *		Initialisation
 *********************************************************************/

static void d1980_init_charger(struct d1980 *d1980)
{
	/* register our interest in charger events */
	/* todo need to decide which events should be register 
	   this is at the moment just implemented to complete the driver 
	*/
	//dev_warn(d1980->dev, "Battery LOW VOLTAGE IRQ Register \n");
	d1980_register_irq(d1980, D1980_IRQ_EVBATLOW,
			    d1980_charger_handler, 0, "LOW Battery VOLTAGE", d1980);

#ifdef USE_TEMP_THRESHOLD
	//dev_warn(d1980->dev, "Battery temperature IRQ Register \n");
	d1980_register_irq(d1980, D1980_IRQ_ETBAT,
			    d1980_charger_handler, 0, "Battery temperature", d1980);
#endif

    d1980_register_irq(d1980, D1980_IRQ_ETA,
                d1980_charger_handler, 0, "Travel Adapter interrupt", d1980);

	//dev_warn(d1980->dev, "Charger Event IRQ Register \n");
	//d1980_register_irq(d1980, D1980_IRQ_EGPI1,
	//		    d1980_charger_handler, 0, "Charger Event", d1980);

}

static void free_charger_irq(struct d1980 *d1980)
{
	d1980_free_irq(d1980, D1980_IRQ_EVBATLOW);
#ifdef USE_TEMP_THRESHOLD
	d1980_free_irq(d1980, D1980_IRQ_ETBAT);
#endif
	//d1980_free_irq(d1980, D1980_IRQ_EGPI1);
}

static u16 cnt = 0;

static int d1980_monitor_thread(void *d1980_monitor_tcb)
{
    struct d1980 *d1980 = (struct d1980 *)d1980_monitor_tcb;

    //set_freezable();
#ifdef D1982_DEBUG
    printk(KERN_INFO "d1980_monitor_thread start  !!!!!! \n");
#endif
    while(!kthread_should_stop())
    {
        set_current_state(TASK_INTERRUPTIBLE);
		// Read ADC
        //if(cnt % 2)
        //    d1980_get_bat_level(d1980);
        //else
            d1980_get_bat_level(d1980);
        //cnt++;
        schedule_timeout(threadPeriod);
    }
    
    return 0;
}


int d1980_set_threshold_vbat(u16 lowBatThr, u16 vBatThr)
{
    int    ret = 0;
    struct d1980 *d1980 = NULL;
	struct d1980_power *power = NULL;
	struct d1980_bat_threshold *threshold = NULL;

    if(d1980_g_client)
    {
	    d1980 = d1980_g_client;
    	power = &d1980->power;
    	threshold = &power->threshold;

    	threshold->vbatlow_mon_thr = lowBatThr;
    	
    	d1980_reg_write(d1980, D1980_VBAT_THR_REG, vbat_thr_mV_to_reg(threshold->vbatlow_mon_thr));
    }
    else
    {
        printk(KERN_ERR " %s >> Set threshold failure !\n", __func__);
        ret = -EIO;
    }
    return ret;
}
EXPORT_SYMBOL_GPL(d1980_set_threshold_vbat);


int d1980_set_threshold_temp(u16 tempBatMaxThr, u16 tempBatMinThr)
{
    int    ret = 0;
    struct d1980 *d1980 = NULL;
	struct d1980_power *power = NULL;
	struct d1980_bat_threshold *threshold = NULL;

    if(d1980_g_client)
    {
	    d1980 = d1980_g_client;
    	power = &d1980->power;
    	threshold = &power->threshold;

    	threshold->tbat_thr_max = tempBatMaxThr;
    	threshold->tbat_thr_min = tempBatMinThr;
    	
    	d1980_reg_write(d1980, D1980_TEMPHIGHTH_REG, threshold->tbat_thr_max);
    	d1980_reg_write(d1980, D1980_TEMPLOWTH_REG, threshold->tbat_thr_min);
    }
    else
    {
        printk(KERN_ERR " %s >> Set threshold failure !\n", __func__);
        ret = -EIO;
    }
    return ret;
}
EXPORT_SYMBOL_GPL(d1980_set_threshold_temp);

// Read ADC ++
void d1980_set_adc_mode(adc_read_type adc_type)
{
    if(adc_mode == adc_type)
        return;

    if(adc_type <= D1980_ADC_READ_MANUAL)
        adc_mode = adc_type;

    return;
}
EXPORT_SYMBOL_GPL(d1980_set_adc_mode);
// Read ADC --

#ifdef CONFIG_BATTERY_D1980_OTHER_PSY
int d1980_register_charger_ctrl_cb_func(charging_ctrl_cb_func callback)
{
	if (callback) {
		charger_ctrl_cb = callback;
		pr_info("d1980_register_charger_ctrl_cb_func[%x]\n", (int)charger_ctrl_cb);
		return 0;
	} else {
		pr_info( \
		"d1980_register_charger_ctrl_cb_func fail\n");
		return 1;
	}
}
#endif

/*********************************************************************
 *		Initialisation
 *********************************************************************/

static __devinit int d1980_power_probe(struct platform_device *pdev)
{
	struct d1980 *d1980 = platform_get_drvdata(pdev);
	struct d1980_power *power = &d1980->power;
	struct power_supply *battery = &power->battery;
	struct d1980_bat_status *bat_status = &power->bat_status;
	struct d1980_bat_threshold *threshold = &power->threshold;
	int ret = 0, retries = D1980_MANUAL_READ_RETRIES, value = 0;

	power->max_capacity = 1350;
	power->resistor = 300;
    power->nvram_data.ibat_offset = 0;
    power->nvram_data.vbat_offset = 0;
    power->nvram_data.vbat_slope_high = 1000;
    power->nvram_data.vbat_slope_low = 1000;

#ifdef CONFIG_BATTERY_D1980_OTHER_PSY
    if(d1980)
    {
        d1980_g_client = d1980;
        printk(KERN_ERR "d1980_power_probe start  !!!!!! \n");
    }
    else
        return -EIO;

    d1980_register_charger_ctrl_cb_func(spa_Charger_Ctrl);

#endif

	// Read ADC ++
    d1980_set_adc_mode(D1980_ADC_READ_AUTO);
	// Read ADC --

	battery->name = "d1980-battery";
	battery->type = POWER_SUPPLY_TYPE_BATTERY;
	battery->properties = d1980_bat_props;
	battery->num_properties = ARRAY_SIZE(d1980_bat_props);
	battery->get_property = d1980_bat_get_property;
	battery->use_for_apm = 1;

	ret = power_supply_register(&pdev->dev, battery);
	if (ret)
		goto battery_failed;

	mutex_init(&d1980->battery_lock);
    //++ Charger control 04/Aug/2011
	mutex_init(&d1980->charger_lock);
    //-- Charger control 04/Aug/2011

	threshold->vbatlow_mon_thr  = D1980_VBATLOW_DEFAULT_LIMIT;
	d1980_reg_write(d1980, D1980_VBAT_THR_REG, 0x00);
#ifdef USE_TEMP_THRESHOLD
	threshold->tbat_thr_max     = D1980_TBATMAX_DEFAULT_LIMIT;
	threshold->tbat_thr_min     = D1980_TBATMIN_DEFAULT_LIMIT;
	
	//d1980_reg_write(d1980, D1980_TEMPHIGHTH_REG, bit8_mV_to_reg(threshold->tbat_thr_max));
	//d1980_reg_write(d1980, D1980_TEMPLOWTH_REG, bit8_mV_to_reg(threshold->tbat_thr_min));
	d1980_reg_write(d1980, D1980_TEMPHIGHTH_REG, threshold->tbat_thr_max);
	d1980_reg_write(d1980, D1980_TEMPLOWTH_REG, threshold->tbat_thr_min);
#endif

	d1980_init_charger(d1980);

	bat_status->charger_type = D1980_NOCHARGER;
	bat_status->status = D1980_DISCHARGING_WITHOUT_CHARGER;

	d1980_set_bits(d1980, D1980_ADCCONT_REG, D1980_ADCCONT_AUTOADCEN | D1980_ADCCONT_AUTOVBATEN); 

	msleep(1);

	bat_status->illegalbattery = 0;
	
	/* todo need to define the Threshold values? but seen as not needed */

#ifdef CONFIG_SYSFS
	ret=sysfs_create_group(&pdev->dev.kobj, &battery_attr_group);
	if (ret < 0) {
		goto battery_failed;
	}
	//++ Charger control 04/Aug/2011
    ret = device_create_file(&pdev->dev, &dev_attr_control);
	if (ret < 0) {
		goto chg_ctl_failed;
	}
    //-- Charger control 04/Aug/2011
#endif

	device_init_wakeup(&pdev->dev, 1);
	return ret;

usb_failed:
    printk(KERN_ERR "[D1980] Power supply device register failed... !\n");
#ifndef CONFIG_BATTERY_D1980_OTHER_PSY
	power_supply_unregister(battery);
#endif
//++ Charger control 04/Aug/2011
chg_ctl_failed:
    sysfs_remove_group(&pdev->dev.kobj, &battery_attr_group);
//-- Charger control 04/Aug/2011
battery_failed:
	dev_warn(d1980->dev, "failed to register d1980-power driver: %d\n", ret);
	return ret;
}

static __devexit int d1980_power_remove(struct platform_device *pdev)
{
	struct d1980 *d1980 = platform_get_drvdata(pdev);
	struct d1980_power *power = &d1980->power;

	free_charger_irq(d1980);
#ifdef CONFIG_SYSFS
    //++ Charger control 04/Aug/2011
    device_remove_file(&pdev->dev, &dev_attr_control);
    //-- Charger control 04/Aug/2011
	sysfs_remove_group(&pdev->dev.kobj, &battery_attr_group);
#endif /* CONFIG_SYSFS */
#ifndef CONFIG_BATTERY_D1980_OTHER_PSY
	power_supply_unregister(&power->battery);
#endif
	return 0;
}

#ifdef CONFIG_PM
static int d1980_power_suspend(struct device *pdev)
{
	struct d1980 *d1980 = dev_get_drvdata(pdev);
	struct d1980_power *power = &d1980->power;
	struct d1980_bat_threshold *threshold = &power->threshold;

	if(d1980 == NULL) {
	    printk(KERN_ERR "[POWER] %s. device data is NULL \n", pdev->driver->name);
        return -ENODATA;
    }

    d1980_reg_write(d1980, D1980_VBAT_THR_REG, vbat_thr_mV_to_reg(threshold->vbatlow_mon_thr));
   	d1980_set_bits(d1980, D1980_ADCCONT_REG, D1980_ADCCONT_AUTOADCEN|D1980_ADCCONT_AUTOVBATEN); 
    
    if(device_may_wakeup(pdev)) {
       // TODO:enable_irq_wake(D1980_IRQ_EVBATLOW);
       //enable_irq_wake(d1980->chip_irq);
    }

    return 0;
}

static int d1980_power_resume(struct device *pdev)
{
	struct d1980 *d1980 = dev_get_drvdata(pdev);
	struct d1980_power *power = &d1980->power;
	struct d1980_bat_threshold *threshold = &power->threshold;

	if(d1980 == NULL) {
	    printk(KERN_ERR "[POWER] %s. device data is NULL \n", pdev->driver->name);
        return -ENODATA;
    }

    if(device_may_wakeup(pdev)) {
        // TODO:disable_irq_wake(D1980_IRQ_EVBATLOW);
        //disable_irq_wake(d1980->chip_irq);
    }

    //d1980_reg_write(d1980, D1980_VBAT_THR_REG, 0x00);
    //printk(KERN_INFO "%s, d1980_power_resume. ADC_CONTROL_REG = 0x%X\n", "Power(Battery)", d1980_reg_read(d1980, D1980_ADCCONT_REG));

    return 0;
}

static struct dev_pm_ops d1980_power_pm_ops = {
	.suspend	= d1980_power_suspend,
	.resume		= d1980_power_resume,
};
#endif /* CONFIG_PM */

static struct platform_driver d1980_power_driver = {
	.probe = d1980_power_probe,
	.remove = __devexit_p(d1980_power_remove),
	.driver = {
		.name  = "d1980-power",
		.owner = THIS_MODULE,
#ifdef CONFIG_PM
        .pm    = &d1980_power_pm_ops,
#endif
	},
};

static int __init d1980_power_init(void)
{
	printk(banner);
	return platform_driver_register(&d1980_power_driver);
}
module_init(d1980_power_init);

static void __exit d1980_power_exit(void)
{
	platform_driver_unregister(&d1980_power_driver);
}
module_exit(d1980_power_exit);

MODULE_AUTHOR("Dialog Semiconductor Ltd <divyang.patel@diasemi.com>");
MODULE_DESCRIPTION("Battery driver for the Dialog D1980 PMIC");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform: d1980-power");
