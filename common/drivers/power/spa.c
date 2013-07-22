#ifndef __SPA_C__
#define __SPA_C__

/*
 * Samsung Power & Accessory
 *
 * linux/i2c/drivers/chips/spa.c
 *
 *  driver for pxa955 - spa.c
 *
 *  Copyright (C) 2010, Samsung Electronics
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
  
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/power_supply.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/i2c.h>
#include <linux/proc_fs.h>
#include <linux/mutex.h>
#include <linux/delay.h>

#include <linux/time.h>
#include <linux/rtc.h>

#include <asm/irq.h>
#include <mach/gpio.h>
#include <asm/uaccess.h>
#include <mach/AL25.h>
#include <mach/spa.h>
#include <mach/hardware.h>
#include <linux/mfd/88pm860x.h>

#if defined(CONFIG_PMIC_D1980)
#include <linux/d1982/bat.h>
#include <linux/d1982/batUtil.h>
#endif

/* FuelGauge Feature */
#ifdef FEAT_EN_FUELGAUGE
#ifdef CONFIG_FUELGAUGE_STC3105
#include <mach/stc3105_battery.h>
#endif
#endif

/* Declare mutex */
DEFINE_MUTEX(spa_charger_mutex);


/* Declare Global variable */
static struct i2c_client 				*g_i2c_client;
static struct AL25_platform_data 		*g_pdata;

static struct wake_lock 			gv_spa_wakelock;
static struct wake_lock 			gv_thr_wakelock;
static struct wake_lock 			gv_JIG_wakelock;
static struct workqueue_struct		*spa_workqueue;



/****************************************************************
  Level 1 : Accessory Insert/Remove
  Level 2 : Start/Stop Charge
  Level 3 : Battery_Work
  TODO level define
  ****************************************************************/
static unsigned int 					gv_DebugLevel 	= 0;

static unsigned int					gv_IsSuspendState = FALSE;
static unsigned int					gv_needOldInfoUpdate = FALSE;


static unsigned char 				gv_NextLevelAlarmValues = 0;
static unsigned char 				gv_IsThresholdState = FALSE;

static signed long 					gv_AverageVbat = 0;
signed long 					gv_AverageTbat = 0;
EXPORT_SYMBOL(gv_AverageTbat);

#ifdef FEAT_EN_FG_PLUSONE
static unsigned int					gv_FgPollingCnt = 0;
static unsigned char				gv_FgOneUpdate = FALSE;
#endif

#define SPA_POWEROFF_BUF_SIZE	1024
static char a_rcvBuf[SPA_POWEROFF_BUF_SIZE] = {0,};


static u16 isTestMode = FALSE; 

static struct spa_status	g_spa_data = {
	.ac_status			= DISCONNECT,
	.usb_status			= DISCONNECT,
	.Charge_Status		= POWER_SUPPLY_STATUS_DISCHARGING,
	.Charge_Health		= POWER_SUPPLY_HEALTH_GOOD,
	.ChargeTimerStatus	= CHARGE_TIMER_INIT,
	.isEndOfCharge		= FALSE,
	.isTimerExpire			= FALSE,
	.isRecharge			= FALSE,
	.rechargeCount		=0,
	.VF_Status			= TRUE,
};

#define SPA_DBG_MSG(n, args...) 			if (gv_DebugLevel>(n)) printk(KERN_ALERT "[Battery] "args)

#define SPA_GET_STATUS(property) 		spa_Status(TRUE, property, 0)
#define SPA_SET_STATUS(property, value) 	spa_Status(FALSE, property, value)


static int spa_Status(u8 isGet, spa_Status_t property, s8 value)
{
	if(isGet) {
		SPA_DBG_MSG(9,  "%s : GET Property <%d>\n", __func__, property); 
		switch(property) {
			case STATUS_AC :
				return g_spa_data.ac_status;
			case STATUS_USB :
				return g_spa_data.usb_status;
			case STATUS_CHARGE :
				return g_spa_data.Charge_Status;
			case STATUS_HEALTH :
				return g_spa_data.Charge_Health;
			case STATUS_EOC :
				return g_spa_data.isEndOfCharge;
			case STATUS_TIMER_EXPIRE :
				return g_spa_data.isTimerExpire;
			case STATUS_RECHG :
				return g_spa_data.isRecharge;
			case STATUS_RECHG_COUNT:
				return g_spa_data.rechargeCount;
			case STATUS_IS_RECHG_COND :
				if(SPA_GET_STATUS(STATUS_FULL) && (gv_AverageVbat < BATT_RECHAR_VOLT) && !SPA_GET_STATUS(STATUS_RECHG))
					return TRUE;
				else
					return FALSE;				
			case STATUS_CHG_TIMER :
				return g_spa_data.ChargeTimerStatus;
			case STATUS_FULL :
				return (g_spa_data.Charge_Status == POWER_SUPPLY_STATUS_FULL);
			case STATUS_IS_CHARGING :
				return (g_spa_data.Charge_Status == POWER_SUPPLY_STATUS_CHARGING);
			case STATUS_IS_GOOD :
				return (g_spa_data.Charge_Health == POWER_SUPPLY_HEALTH_GOOD);
			case STATUS_IS_COLD :
				return (g_spa_data.Charge_Health == POWER_SUPPLY_HEALTH_COLD);
			case STATUS_IS_OVERHEAT :			
				return (g_spa_data.Charge_Health == POWER_SUPPLY_HEALTH_OVERHEAT);
			case STATUS_IS_OVERVOLT :
				return (g_spa_data.Charge_Health == POWER_SUPPLY_HEALTH_OVERVOLTAGE);
			case STATUS_VF:
				return g_spa_data.VF_Status;
			default :
				break;
		}
	}
	else {
		SPA_DBG_MSG(9,  "%s : SET Property <%d>, Value = <%d>\n", __func__, property, value); 
		switch(property) {
			case STATUS_AC :
				g_spa_data.ac_status 		= value;
				break;
			case STATUS_USB :
				g_spa_data.usb_status 	= value;
				break;
			case STATUS_CHARGE :
				g_spa_data.Charge_Status	= value;
				break;
			case STATUS_HEALTH :
				g_spa_data.Charge_Health	= value;
				break;
			case STATUS_EOC :
				g_spa_data.isEndOfCharge	= value;
				break;
			case STATUS_TIMER_EXPIRE :
				g_spa_data.isTimerExpire 	= value;
				break;
			case STATUS_RECHG :
				g_spa_data.isRecharge 	= value;
				break;
			case STATUS_RECHG_COUNT:
				if(value == FALSE)
					g_spa_data.rechargeCount	= 0;
				else 
					g_spa_data.rechargeCount++;
				break;
			case STATUS_CHG_TIMER :
				g_spa_data.ChargeTimerStatus = value;
				break;
			case STATUS_VF:
				g_spa_data.VF_Status = value;
				break;
			default :
				return -EINVAL;
		}
		return 0;
	}

	return -EINVAL;

}


int spa_Lfs_Access(int rw_op, spa_poweroff_t value)
{
	int ret=0, f_flags=0, size=0;
	struct file *filp;
	mm_segment_t mm_seg_fs;

	char a_temp[50] = {0,};
	char a_cause[50] = {0,};

	struct timeval tv;
	struct rtc_time tm;

	char* spa_PowerOffCause[POWEROFF_NB_END] = {	
		"Init", 
		"LowBattery",
		"VF",
		"OverVolt",
		"Hot",
		"Cold",
		"Good again",
		"Full Charge",
		"Recharge",
		"EOC Interrupt",
		"Charge Timeout",
		"Charge Enable",
		"Charge Disable",
		"Discharge",
		"Control_EN",
		"Control_DIS",
		"Wrong EOC",
		"PXA95X_PM"
	};

	if (rw_op == DEBUG_READ)
	        f_flags = (int)(O_RDONLY | O_CREAT);
	else
	        f_flags = (int)(O_RDWR|O_SYNC | O_CREAT);

	filp = filp_open(POWEROFF_CAUSE, f_flags, 0);

	if (IS_ERR(filp)) {
	        //pr_err("%s: filp_open failed. (%ld)\n", __FUNCTION__, PTR_ERR(filp));
	        return -1;
	}

	ret = filp->f_op->llseek(filp, 0, SEEK_SET);

	if(rw_op == DEBUG_WRITE) {
		do_gettimeofday(&tv);
		rtc_time_to_tm(tv.tv_sec, &tm);

		sprintf(a_temp, "%d/%d %d:%d:%d, %s\n", 
			tm.tm_mon+1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec,  spa_PowerOffCause[value]);
		strncpy(a_cause, a_temp, strlen(a_temp));
	}

	mm_seg_fs = get_fs();
	set_fs( get_ds() );

	size = filp->f_op->read(filp,  a_rcvBuf, SPA_POWEROFF_BUF_SIZE-1, &filp->f_pos);
	printk(KERN_INFO "Size = %d\n", size);
	printk(KERN_ALERT "%s\n", a_rcvBuf);

	set_fs(mm_seg_fs);

	if(rw_op == DEBUG_WRITE) {
		ret = filp->f_op->llseek(filp, 8, SEEK_SET);

		mm_seg_fs = get_fs();
		set_fs( get_ds() );

		ret = filp->f_op->write(filp,  a_cause, strlen(a_cause), &filp->f_pos);
		ret = filp->f_op->write(filp,  a_rcvBuf, size, &filp->f_pos);

		set_fs(mm_seg_fs);
	}

	filp_close(filp, NULL);

	return ret;
}


#ifndef FEAT_EN_FUELGAUGE
unsigned int calculate_batt_level(int batt_volt)
{
	int scaled_level = 0;

	if(batt_volt >= BATT_RECHAR_VOLT) {		//100%
		scaled_level = 100;
	}
	else if(batt_volt >=  BATT_LEVEL5_VOLT) {	//99% ~ 80%
			scaled_level = ((batt_volt -BATT_LEVEL5_VOLT+1)*19)/(BATT_RECHAR_VOLT-BATT_LEVEL5_VOLT);
 		scaled_level = scaled_level+80;
	}
	else if(batt_volt >= BATT_LEVEL4_VOLT) { 	//79% ~ 65%
		scaled_level = ((batt_volt -BATT_LEVEL4_VOLT)*15)/(BATT_LEVEL5_VOLT-BATT_LEVEL4_VOLT);
 		scaled_level = scaled_level+65;
	}
	else if(batt_volt >= BATT_LEVEL3_VOLT) { 	//64% ~ 50%
		scaled_level = ((batt_volt -BATT_LEVEL3_VOLT)*15)/(BATT_LEVEL4_VOLT-BATT_LEVEL3_VOLT);
 		scaled_level = scaled_level+50;
	}
	else if(batt_volt >= BATT_LEVEL2_VOLT) {	//49% ~ 35%
		scaled_level = ((batt_volt -BATT_LEVEL2_VOLT)*15)/(BATT_LEVEL3_VOLT-BATT_LEVEL2_VOLT);
		scaled_level = scaled_level+35;
	}
	else if(batt_volt >= BATT_LEVEL1_VOLT) {	//34% ~ 20%
		scaled_level = ((batt_volt -BATT_LEVEL1_VOLT)*15)/(BATT_LEVEL2_VOLT-BATT_LEVEL1_VOLT);
 		scaled_level = scaled_level+20;
	}
	else if(batt_volt >= BATT_LEVEL0_VOLT) {	//19% ~ 5%
		scaled_level = ((batt_volt -BATT_LEVEL0_VOLT)*15)/(BATT_LEVEL1_VOLT-BATT_LEVEL0_VOLT);
 		scaled_level = scaled_level+5;
	}
		else if(batt_volt > BATT_LOW_VOLT) {		// 4% ~ 1%
		scaled_level = ((batt_volt -BATT_LOW_VOLT)*4)/(BATT_LEVEL0_VOLT-BATT_LOW_VOLT);
 		scaled_level = scaled_level+1;
	}
	else {
		scaled_level = 0; 
	}
	
	SPA_DBG_MSG(2, "%s : batt_volt %d, scaled_level %d\n", __func__, batt_volt, scaled_level);

	return scaled_level;
}
#endif	//!FEAT_EN_FUELGAUGE


static signed int spa_TemperatureTable(unsigned short vl_GainData)
{
	u8 	i;
	s32 vl_A1, vl_A2, vl_A3;
	signed int v_ConvTemp;

#ifdef CONFIG_MACH_ALKON
	signed short a_TempAdcTable[] = {
		1607,
		1467,
		1413,
		1332,
		1186,
		1044,
		925,
		809,
		695,
		592,
		513,
		437,
		386,
		367,
		321,
		274,
		239,
		225,
	}; //a_TempAdcTable
	
	signed char a_TempDegreeTable[] ={
		-10,
		-5,
		-3,
		0,
		5,
		10,
		15,
		20,
		25,
		30,
		35,
		40,
		43,
		45,
		50,
		55,
		60,
		65,
	}; //a_TempDegreeTable
#else //else CONFIG_MACH_ALKON
	signed short a_TempAdcTable[] = {
		1015,
		1001,
		986,
		911,
		835,
		705,
		570,
		443,
		343,
		280,
		231,
		195,
		164,
		137,
		117,
		99,
		86,
		74,
		62,
		54,
	}; //a_TempAdcTable
	
	signed char a_TempDegreeTable[] ={
		-30,
		-25,
		-20,
		-15,
		-10,
		-5,
		0,
		5,
		10,
		15,
		20,
		25,
		30,
		35,
		40,
		45,
		50,
		55,
		60,
		65,
	}; //a_TempDegreeTable
#endif //endif CONFIG_MACH_ALKON

	for(i = 0; i < SPA_NB_TEMP_TABLE; i++) {
		if(vl_GainData >= a_TempAdcTable[i])
			break;
	}


	if( i == 0 ){
		i = 1;
	}
	else if( i == SPA_NB_TEMP_TABLE ) {
		i = (SPA_NB_TEMP_TABLE-1);			
	} 

	vl_A1 = vl_GainData * (a_TempDegreeTable[i-1] - a_TempDegreeTable[i]);
	vl_A2 = a_TempAdcTable[i-1] * (a_TempDegreeTable[i-1] - a_TempDegreeTable[i]) \
			- a_TempDegreeTable[i-1] * (a_TempAdcTable[i-1] - a_TempAdcTable[i]);
	vl_A3 = (a_TempAdcTable[i-1] - a_TempAdcTable[i]);

	v_ConvTemp = ((vl_A1-vl_A2)*10)/vl_A3;
	
	SPA_DBG_MSG(3, "%s : vl_GainData %d, v_ConvTemp %d\n", __func__, vl_GainData, v_ConvTemp);
	
	return v_ConvTemp;
}


static void spa_ShiftTableAndSetLastElt(s32 *ap_Table, u8 vp_EltNumber, s32 vp_LastElt)
{
	u8	i;
	
	for ( i = 1; i < vp_EltNumber; i++) {
		ap_Table[i-1] = ap_Table[i];
	}

	ap_Table[vp_EltNumber-1] = vp_LastElt;
}


static s32 spa_GetAverage(s32 *ap_Table , u8 vp_EltNumber)
{
	u8	i;
	s32	vl_Sum = 0;

	/* For each element in the table */
	for( i = 0; i < vp_EltNumber; i++) {
		vl_Sum += ap_Table[i];
	}

	return( vl_Sum/vp_EltNumber );
} 


static int spa_Create_Attrs(struct device * dev, struct device_attribute *power_attr,  int array_size)
{
	int i; 
	int rc = EINVAL;

	for (i = 0; i < array_size; i++) {
		rc = device_create_file(dev, power_attr+i);
		if (rc) {
			device_remove_file(dev, power_attr+i);
		}
	}
	return rc;
}


/* ++++++ START : I2C Control function for Battery */
int spa_I2C_Write( unsigned char devAddr, unsigned char devReg, unsigned short  numBytesToWrite, unsigned char *pData )
{              
	int ret;

	if (g_i2c_client == NULL) 
		return -ENODEV;

	ret = i2c_smbus_write_i2c_block_data(g_i2c_client, devReg,numBytesToWrite, pData);
	SPA_DBG_MSG(8, "\n%s : ret = %d\n", __func__, ret);
	
	return ret;
}

int spa_I2C_Read( unsigned char devAddr, unsigned char devReg, unsigned char numBytesToRead, unsigned char *pData )
{          
	int ret;

	if (g_i2c_client == NULL)   
		return -ENODEV;

	ret = i2c_smbus_read_i2c_block_data(g_i2c_client, devReg, numBytesToRead, pData);
	SPA_DBG_MSG(8, "\n%s : ret = %d\n", __func__, ret);
	
	return ret;
}
/* ----- FINISH : I2C Control function for Battery */



/*  + Charge Bit Control */
static void spa_Enable_Charge_Bit(void)
{
	int ret;

#ifdef FEAT_EN_USE_MAX14577
	SPA_DBG_MSG(1, "%s : Current Accessory = %d\n", __func__, MG_AL25_GetAccessory()); 
   
	gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL2 ] =
	(
		MD_AL25_REG_CHGCTRL2_VCHGR_RC_ENABLE|
		MD_AL25_REG_CHGCTRL2_MBHOSTEN_ENABLE
	);
	
	ret= spa_I2C_Write( MD_AL25_IIC_ADDRESS, 			//I2C Address
		MD_AL25_REG_CHGCTRL2,							//Register Address
		1, 												//Size
		&( gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL2 ] )	//Value
	);

	MD_AL25_AddMetric(MAX14577_ChargingEnabled);
#else
	SPA_DBG_MSG(1, "%s\n", __func__);
#endif

}

static void spa_Disable_Charge_Bit(void)
{
	int ret;

#ifdef FEAT_EN_USE_MAX14577
	SPA_DBG_MSG(1, "%s : Current Accessory = %d\n", __func__, MG_AL25_GetAccessory()); 
    

	gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL2 ] =
	(
		MD_AL25_REG_CHGCTRL2_VCHGR_RC_DISABLE   |
		MD_AL25_REG_CHGCTRL2_MBHOSTEN_DISABLE
	);
		
	ret = spa_I2C_Write( MD_AL25_IIC_ADDRESS, 
		MD_AL25_REG_CHGCTRL2,
		1, 
		&( gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL2 ] )
	);

	MD_AL25_AddMetric(MAX14577_ChargingDisabled);
#else
	SPA_DBG_MSG(1, "%s\n", __func__);
#endif
}
/*  - Charge Bit Control */


static void spa_SetVbatThresholds(u16 lower, u16 upper)
{

	SPA_DBG_MSG(1, "%s Lower : %d, Upper %d\n", __func__, lower, upper);

#if 1

	#if defined(CONFIG_PMIC_D1980)
		d1980_set_threshold_vbat(lower, upper);
	#elif defined(CONFIG_ALTERNATE_CHARGER)
		pm860x_set_vbatt_threshold(lower, upper);
	#endif

#else

	#if defined(CONFIG_PMIC_D1980)
		d1980_set_threshold_vbat(3400, upper);
	#elif defined(CONFIG_ALTERNATE_CHARGER)
		pm860x_set_vbatt_threshold(3400, upper);
	#endif

#endif
}



/*  + Get Temperature value */
static unsigned int spa_Get_Temp_Adc(void)
{
	unsigned int v_tbat_adc;

#if defined(CONFIG_PMIC_D1980)
	v_tbat_adc = (unsigned int)d1980_get_Temperature();
#elif defined(CONFIG_ALTERNATE_CHARGER)
	pm860x_read_tbat_adc(PM8607_GPADC_TBAT, &v_tbat_adc);
#else
	v_tbat_adc = 0;
#endif

	return v_tbat_adc;
}

static void spa_Bat_AverageTemp(unsigned char v_update)
{
	static unsigned int a_VbatTempValues[BAT_TEMPERATURE_SAMPLE];
	unsigned int v_tbat_adc;

	u16 i;
	s32 v_ConvTemp = 0;

	if(v_update == TRUE) {
		for(i = 0; i <  BAT_TEMPERATURE_SAMPLE; i++) {
			v_tbat_adc= spa_Get_Temp_Adc();
	   		a_VbatTempValues[i]= spa_TemperatureTable(v_tbat_adc);
		}
	}

	v_tbat_adc= spa_Get_Temp_Adc();
   	v_ConvTemp = spa_TemperatureTable(v_tbat_adc);

	spa_ShiftTableAndSetLastElt(a_VbatTempValues, BAT_TEMPERATURE_SAMPLE, v_ConvTemp);
	gv_AverageTbat = spa_GetAverage(a_VbatTempValues, BAT_TEMPERATURE_SAMPLE);	
	SPA_DBG_MSG(2, "%s :  AverageTemperature = <%ld> \n", __func__, gv_AverageTbat);
}
/*  - Get Temperature value */



/*  + Get Voltage value */
static unsigned int spa_Get_Volt(void)
{
	unsigned int v_vbat_adc;

#ifdef FEAT_EN_FUELGAUGE

	#ifdef CONFIG_FUELGAUGE_STC3105
		v_vbat_adc = fg_read_vcell();
	#else
		return 3800; 	
	#endif

#else

	#if defined(CONFIG_PMIC_D1980)
		v_vbat_adc = (unsigned int)d1980_get_Voltage();
	#elif defined(CONFIG_ALTERNATE_CHARGER)
		struct power_supply *psy;
		psy = power_supply_get_by_name("88pm860x_battery");
		if (psy)
			pm860x_read_vbat(&v_vbat_adc);
		else
			v_vbat_adc = 0;
	#else
		v_vbat_adc = 3800;
	#endif

#endif

	SPA_DBG_MSG(3, "%s Voltage = %d\n", __func__, v_vbat_adc);

	return v_vbat_adc;
}

static unsigned int spa_Get_Volt_Compensation(void)
{
	unsigned int v_vbat_adc;

	v_vbat_adc = spa_Get_Volt();

	#ifndef FEAT_EN_FUELGAUGE
		if(SPA_GET_STATUS(STATUS_IS_CHARGING))
			v_vbat_adc += COMPENSATION_VOLTAGE;	

		#ifdef CONFIG_MACH_JETTA
		//In Resume status voltage values are correct
		//In Suspend status voltage values are increase upto 0.04V in Jetta
		//Condition 1. Battery Full status
		if(SPA_GET_STATUS(STATUS_FULL))
			if(!IsLCDon())
				v_vbat_adc = v_vbat_adc - 40;
		#endif
	#endif

	SPA_DBG_MSG(3, "%s Voltage = %d\n", __func__, v_vbat_adc);

	return v_vbat_adc;
}

/*
	spa_Bat_AverageVolt functin can be called from 
	1. spa_Get_Capacity
	2. spa_bat_work
	3. spa_voltThresholdEvent
	4. spa_Probe : To having initial values. 
*/
static void spa_Bat_AverageVolt(unsigned char v_update)
{
	u16 i;
	unsigned int v_vbat_adc;
	static unsigned int a_VbatVoltValues[BAT_VOLTAGE_SAMPLE];

	if(v_update == TRUE) {
		for(i = 0; i < BAT_VOLTAGE_SAMPLE; i++) {
			v_vbat_adc = spa_Get_Volt_Compensation();			
			a_VbatVoltValues[i] = v_vbat_adc;
		}
	}

	v_vbat_adc = spa_Get_Volt_Compensation();

	spa_ShiftTableAndSetLastElt(a_VbatVoltValues, BAT_VOLTAGE_SAMPLE, v_vbat_adc);
	gv_AverageVbat = spa_GetAverage(a_VbatVoltValues, BAT_VOLTAGE_SAMPLE);	
	SPA_DBG_MSG(2, "%s :  AverageVolt = <%ld> \n", __func__, gv_AverageVbat);
}
/*  - Get Voltage value */

/*
	spa_Get_Capacity function can be called from 
	1. spa_Bat_Get_Property
	2. spa_bat_work
	3. spa_CallBack_Capacity
	4. spa_Test_Show
	5. spa_Suspend
*/

static unsigned int spa_Get_Capacity(void) 
{
	unsigned int v_capacity;
	static char v_lfs_zero_done = FALSE;
#ifdef FEAT_EN_CONTROL_CHARGING_LV_SUDDEN_CHANGE
	static int v_prevCapVal = 0;
#endif 

#ifdef FEAT_EN_FUELGAUGE

	#ifdef CONFIG_FUELGAUGE_STC3105
		v_capacity = fg_read_soc();
	#else //CONFIG_FUELGAUGE_STC3105
		v_capacity = 50;
	#endif

#else //FEAT_EN_FUELGAUGE

	/* Periodic voltage update done in battery_work function */
	/* In suspend status battery voltages are not updated */
	/* In case of at+batgetlevel? at command while sleep status, 
            we need to get new average voltage value */
	if(gv_IsSuspendState)
		spa_Bat_AverageVolt(TRUE);

	//Abnormal case spa_Get_Capacity function calling before probe end.
	if(gv_AverageVbat == 0)
		return 0;

	v_capacity = calculate_batt_level(gv_AverageVbat);
	SPA_DBG_MSG(2,  "%s : Curr Capacity = <%d>\n", __func__, v_capacity); 
	SPA_DBG_MSG(2,  "%s : Prev Capacity = <%d>\n", __func__, v_prevCapVal); 

	#ifdef FEAT_EN_CONTROL_CHARGING_LV_SUDDEN_CHANGE
		if(!v_prevCapVal)
			v_prevCapVal = v_capacity;

		//Add charging/discharging buffer to avoid sudden level up/down
		if(SPA_GET_STATUS(STATUS_IS_CHARGING)) {
			 if(v_prevCapVal < v_capacity)  {
				#ifdef FEAT_EN_FG_PLUSONE
				if(gv_FgOneUpdate) {
					v_capacity = v_prevCapVal+1;
					v_prevCapVal = v_capacity;
					gv_FgOneUpdate = FALSE;
				}
				else
					v_capacity = v_prevCapVal;
				#endif
			}
			else
				v_capacity = v_prevCapVal;
		}
		else {
			if(v_capacity  < v_prevCapVal) {
				#ifdef FEAT_EN_FG_PLUSONE
				/*	gv_needOldInfoUpdate : 
					Sleep->Wakeup & Calculated cap less than Prev cap. 
					It can make capacity change like 50% -> 30%. 
				*/
				if(gv_IsSuspendState)
					v_prevCapVal = v_capacity;
				else if (gv_needOldInfoUpdate) {
					v_prevCapVal = v_capacity;
					gv_needOldInfoUpdate = FALSE;
					SPA_DBG_MSG(4, "Discharge : gv_BatOldValues\n");
				}
				else if(gv_FgOneUpdate) {
					v_capacity = v_prevCapVal -1;
					v_prevCapVal = v_capacity;
					gv_FgOneUpdate = FALSE;
					SPA_DBG_MSG(4, "Discharge : gv_FgOneUpdate\n");
				}
				else {
					SPA_DBG_MSG(4, "Discharge : Nothing\n");
					v_capacity = v_prevCapVal;			
				}
				#endif
			}
			else {
				SPA_DBG_MSG(4,  "Discharge : v_prevCapValp[%d]  < v_capacity[%d]\n", v_prevCapVal,v_capacity);
				v_capacity = v_prevCapVal;		
			}
		}
	#endif //FEAT_EN_CONTROL_CHARGING_LV_SUDDEN_CHANGE

#endif //FEAT_EN_FUELGAUGE

	SPA_DBG_MSG(2,  "%s : Android Sending Capacity = <%d>\n", __func__, v_capacity); 

/*+++++ Exception Case */
	/* To avoid PowerOff while charger connect */
	if ((v_capacity == 0) &&(SPA_GET_STATUS(STATUS_IS_CHARGING)))
		v_capacity = 1; 

	if(gv_IsThresholdState == TRUE) {
		v_capacity = gv_NextLevelAlarmValues;
		//After report Alarm capacity calculated capacity sharphly drop due to voltage drop.
		//To prevent such situation i'll set previous capacityy value as sending capacity.
		v_prevCapVal = v_capacity;
		gv_IsThresholdState = FALSE;
	}
	else  if((v_capacity == 0) && ( v_lfs_zero_done == FALSE)) {
		//Capacity 0 called several time, v_lfs_zero_done will prevent unnecessary record.
		v_lfs_zero_done = TRUE;
		//To avoid access lfs while interrupt context
		spa_Lfs_Access(DEBUG_WRITE, POWEROFF_LOWBAT);
	}

	if(SPA_GET_STATUS(STATUS_FULL)) 				/* Return 100% */
		v_capacity = 100;

	 if(SPA_GET_STATUS(STATUS_VF) == FALSE)	 {	/* Reture 0 : Online -1, Battery health dead. */
		v_capacity= 0;
	}
/*------- Exception Case */

	return v_capacity;
}

static u8 spa_VfCheck(void)
{
	int v_vf_adc;

#if defined(CONFIG_PMIC_D1980)    
    v_vf_adc = (int)d1980_get_VFdetection();
#elif defined(CONFIG_ALTERNATE_CHARGER)
	pm860x_read_tbat_adc(PM8607_GPADC3_MEAS1, &v_vf_adc);
#else
	TRUE;
#endif

	SPA_DBG_MSG(1, "%s : VF ADC values =  <%d>\n", __func__, v_vf_adc);

	if(v_vf_adc < 0x70)
		return TRUE;
	else
		return FALSE;
}




/* +++++++++++++++++++++++++++  Property for AC and USB  +++++++++++++++++++++++++++ */
static int spa_Power_Get_Property(struct power_supply *psy, enum power_supply_property psp, union power_supply_propval *val)
{
	switch (psp) {
		case POWER_SUPPLY_PROP_ONLINE:
			if (psy->type == POWER_SUPPLY_TYPE_MAINS) {
				val->intval = SPA_GET_STATUS(STATUS_AC);
				SPA_DBG_MSG(4, "%s POWER_SUPPLY_PROP_ONLINE, POWER_SUPPLY_TYPE_MAINS, Value = <%d>\n", __func__, val->intval);
			}
			else {
				val->intval = SPA_GET_STATUS(STATUS_USB);
				SPA_DBG_MSG(4, "%s POWER_SUPPLY_PROP_ONLINE, POWER_SUPPLY_TYPE_USB, Value = <%d>\n", __func__, val->intval);
			}
			break;
		default:
			return -EINVAL;
	}
	return 0;
}

static enum power_supply_property spa_power_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static char *spa_power_supplied_to[] = {
	"battery",
};

static struct power_supply spa_psy_ac = {
	.name 			= "ac",
	.type 			= POWER_SUPPLY_TYPE_MAINS,
	.supplied_to 		= spa_power_supplied_to,
	.num_supplicants 	= ARRAY_SIZE(spa_power_supplied_to),
	.properties 		= spa_power_props,
	.num_properties 	= ARRAY_SIZE(spa_power_props),
	.get_property 		= spa_Power_Get_Property,
};

static struct power_supply spa_psy_usb = {
	.name 			= "usb",
	.type 			= POWER_SUPPLY_TYPE_USB,
	.supplied_to 		= spa_power_supplied_to,
	.num_supplicants 	= ARRAY_SIZE(spa_power_supplied_to),
	.properties 		= spa_power_props,
	.num_properties 	= ARRAY_SIZE(spa_power_props),
	.get_property 		= spa_Power_Get_Property,
};
/* -----------------------------------  Property for AC and USB  ----------------------------------- */




/* +++++++++++++++++++++++++++  Property for Battery  +++++++++++++++++++++++++++ */
static int spa_Bat_Get_Property(struct power_supply *bat_ps, enum power_supply_property psp, union power_supply_propval *val)
{
	SPA_DBG_MSG(4, "%s Battery Property = <%d>\n", __func__, psp);

	switch (psp) {
		case POWER_SUPPLY_PROP_STATUS:
			val->intval = SPA_GET_STATUS(STATUS_CHARGE);
			break;
		case POWER_SUPPLY_PROP_HEALTH:
			val->intval = SPA_GET_STATUS(STATUS_HEALTH);
			break;
		case POWER_SUPPLY_PROP_PRESENT:
			val->intval = 1;
			break;
		case POWER_SUPPLY_PROP_CAPACITY:
			val->intval = spa_Get_Capacity();
			break;
		case POWER_SUPPLY_PROP_VOLTAGE_NOW :
			val->intval = spa_Get_Volt();
			break;
		case POWER_SUPPLY_PROP_VOLTAGE_AVG :
			val->intval = gv_AverageVbat;
			break;
		case POWER_SUPPLY_PROP_TEMP :
			val->intval = gv_AverageTbat; 
			break;
		case POWER_SUPPLY_PROP_TECHNOLOGY:
			val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
			break;
		default:
			return -EINVAL;
	}
	return 0;
}

static enum power_supply_property spa_bat_main_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_AVG,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TECHNOLOGY,
};
/* 5th */

static void spa_Bat_External_Power_Changed(struct power_supply *bat_ps)
{
	SPA_DBG_MSG(2, "%s\n", __func__);
}

struct power_supply spa_bat_psy = {
	.name					= "battery",
	.type					= POWER_SUPPLY_TYPE_BATTERY,
	.properties				= spa_bat_main_props,
	.num_properties			= ARRAY_SIZE(spa_bat_main_props),
	.get_property				= spa_Bat_Get_Property,
	.external_power_changed 	= spa_Bat_External_Power_Changed,
	.use_for_apm				= 1,
};
/* -----------------------------------  Property for Battery  ----------------------------------- */



/******	+++++ START : Function for Battery Timer.********/
/*
	typedef enum {
		CHARGE_TIMER_90MIN,
		CHARGE_TIMER_5HOUR,
		CHARGE_TIMER_6HOUR
	} chargeTimer_t;
*/
static void spa_Start_Charge_Timer(chargeTimer_t duration, void *data)
{	
	struct AL25_platform_data *pdata = (struct AL25_platform_data *)data;
	unsigned char isStartTimer = MCS_TRUE;
	
	if (timer_pending(&pdata->Charge_timer))
	{
		SPA_DBG_MSG(1, "spa_Start_Charge_Timer : timer_pending exist FAIL\n"); 
		return;
	}
	
	switch(duration)
	{
		case CHARGE_TIMER_90MIN:
		//60sec * 90min : Recharge Timer after force charging timer expire. 
		pdata->Charge_timer.expires = jiffies + 90*MINUTE*HZ; 		
		SPA_DBG_MSG(1, "spa_Start_Charge_Timer : 90Min\n");
			break;
			
		case CHARGE_TIMER_5HOUR:
		//60sec * 60min * 5hour : Since we are using under 1500mAh battery.
		pdata->Charge_timer.expires = jiffies + 5*HOUR*HZ; 
		SPA_DBG_MSG(1, "spa_Start_Charge_Timer : 5Hour\n");
			break;
			
		case CHARGE_TIMER_6HOUR:
		//60sec * 60min * 6hour : Since we are using 1500mAh battery.
		pdata->Charge_timer.expires = jiffies + 6*HOUR*HZ; 
		SPA_DBG_MSG(1, "spa_Start_Charge_Timer : 6Hour\n");
			break;
		
		default:
			SPA_DBG_MSG(1, "spa_Start_Charge_Timer : Fault!!!\n");
			isStartTimer = MCS_FALSE;
		break;
	}
	
	if(isStartTimer == MCS_TRUE)
	{
		add_timer(&pdata->Charge_timer);
		SPA_SET_STATUS(STATUS_CHG_TIMER, CHARGE_TIMER_STARTED);
	}
}

static void spa_Stop_Charge_Timer(spa_chargeStatus_t endType, void *data)
{
	struct AL25_platform_data *pdata = (struct AL25_platform_data *)data;

	SPA_DBG_MSG(1, "spa_Stop_Charge_Timer : EndOfCharge Type is %d\n", endType); 
	
	if(timer_pending(&pdata->Charge_timer))
	{
		SPA_DBG_MSG(1, "spa_Stop_Charge_Timer : PendingTimer exist, Delete Timer\n"); 
		del_timer(&pdata->Charge_timer);
	}
	SPA_SET_STATUS(STATUS_CHG_TIMER, endType);
}

//Expire function when 6Hour, 5Hour and 90Min timer timeout 
static void spa_Expire_Charge_Timer(unsigned long data)
{
	struct AL25_platform_data *pdata = (struct AL25_platform_data *)data;

	/* ++++ Start 30second RechargeCheck timer *//* Expire function : spa_Expire_ReChargeCheck_Timer */
	pdata->reCharge_start_timer.expires = jiffies + 30*HZ; 				
	
	if (timer_pending(&pdata->reCharge_start_timer)){
		SPA_DBG_MSG(1, "spa_Expire_Charge_Timer : reCharge_start_timer already exist FAIL!!!\n"); 
		return;
	}
	SPA_DBG_MSG(1, "spa_Expire_Charge_Timer : Start reCharge_start_timer 30s\n"); 
	add_timer(&pdata->reCharge_start_timer); 
	/* ---- Start 30second RechargeCheck timer *//*  Expire function : spa_Expire_ReChargeCheck_Timer */

	SPA_SET_STATUS(STATUS_TIMER_EXPIRE, TRUE);
	SPA_SET_STATUS(STATUS_CHARGE, POWER_SUPPLY_STATUS_FULL);

	queue_work(spa_workqueue,&pdata->Charge_work);
}

//Expire 30s Timer
static void spa_Expire_ReChargeCheck_Timer(unsigned long data)
{
	struct AL25_platform_data *pdata = (struct AL25_platform_data *)data;

	if(SPA_GET_STATUS(STATUS_IS_RECHG_COND))
	{
		SPA_DBG_MSG(1, "spa_Expire_ReChargeCheck_Timer : Battery Voltage is under ReChargeVoltage, ReCharge Necessary Charge Bit enable again\n");
		SPA_SET_STATUS(STATUS_RECHG, TRUE);

		queue_work(spa_workqueue,&pdata->Charge_work);
	}
	else
	{
		SPA_DBG_MSG(1, "spa_Expire_ReChargeCheck_Timer : Battery Voltage is over ReChargeVoltage, ReCharge Unnecessaary, Charge Bit Already Disabled\n");
	}
}
/******	------ FINISH : Function for Battery Timer.********/




/* Generic Function */
/* Call register control function following charge status. */
/* Since it use I2C transfer function it will be call in workQueue */
static void spa_Charge_Worker(struct work_struct *work)
{
	struct AL25_platform_data *pdata = container_of(work, struct AL25_platform_data, Charge_work);

	u16 v_Curr_Health;

	static u16 v_Prev_AC_Status 	= DISCONNECT;
	static u16 v_Prev_USB_Status 	= DISCONNECT;

	SPA_DBG_MSG(1, "\nspa_Charge_Worker\n");

	mutex_lock(&spa_charger_mutex);	

	v_Curr_Health = SPA_GET_STATUS(STATUS_HEALTH);

	switch(SPA_GET_STATUS(STATUS_CHARGE))
	{
		case POWER_SUPPLY_STATUS_CHARGING:

			/* For debugging screen *//* reset recharge count value *//* TA/USB plug-in, Being health good. */
			SPA_SET_STATUS(STATUS_RECHG_COUNT, FALSE);

			if (v_Curr_Health == POWER_SUPPLY_HEALTH_GOOD) 
			{
				SPA_DBG_MSG(2, "POWER_SUPPLY_HEALTH_GOOD\n");

				//Check VF ADC values 				
				if (spa_VfCheck() == TRUE)
				{
					SPA_DBG_MSG(2, " VF check PASS!!! Enable Charge Bit & Set Threshold & Start Charge Timer\n");

					spa_Enable_Charge_Bit();
					spa_Lfs_Access(DEBUG_WRITE, POWEROFF_CHARGE_ENABLE);
					spa_Start_Charge_Timer(SPA_FORCE_CHG_TIME, (void *)pdata);

					SPA_SET_STATUS(STATUS_VF, TRUE);
				}
				else
				{
					SPA_DBG_MSG(2, " VF check FAIL!!! Status : NOT CHG, Health : DAED, AC : DISCON, USB : DISCON\n");

					SPA_SET_STATUS(STATUS_CHARGE, POWER_SUPPLY_STATUS_NOT_CHARGING);
					SPA_SET_STATUS(STATUS_HEALTH, POWER_SUPPLY_HEALTH_DEAD);
					SPA_SET_STATUS(STATUS_AC, DISCONNECT);
					SPA_SET_STATUS(STATUS_USB, DISCONNECT);
					SPA_SET_STATUS(STATUS_VF, FALSE);

					spa_Disable_Charge_Bit();

					spa_Lfs_Access(DEBUG_WRITE, POWEROFF_VF);
					spa_Lfs_Access(DEBUG_WRITE, POWEROFF_CHARGE_DISABLE);
				}
			}
			else if( SPA_GET_STATUS(STATUS_IS_OVERHEAT)	\
				|| SPA_GET_STATUS(STATUS_IS_COLD)			\
				|| SPA_GET_STATUS(STATUS_IS_OVERVOLT) 	\
			)
			{
				SPA_DBG_MSG(2, "POWER_SUPPLY_HEALTH_OVERHEAT, COLD, OVERVOLT!!! Status : NOT CHG\n");
				SPA_DBG_MSG(2, "Disable charge bit, Stop charge timer\n");
				SPA_SET_STATUS(STATUS_CHARGE, POWER_SUPPLY_STATUS_NOT_CHARGING);

				if(SPA_GET_STATUS(STATUS_IS_OVERVOLT))
					spa_Lfs_Access(DEBUG_WRITE, POWEROFF_OVERVOLT);
				else if(SPA_GET_STATUS(STATUS_IS_OVERHEAT))
					spa_Lfs_Access(DEBUG_WRITE, POWEROFF_HOT);
				else
					spa_Lfs_Access(DEBUG_WRITE, POWEROFF_COLD);
					

				spa_Lfs_Access(DEBUG_WRITE, POWEROFF_CHARGE_DISABLE);

				//MAXIM request us not disable charge bit but in case of Notcharging we do.
				spa_Disable_Charge_Bit();
				spa_Stop_Charge_Timer(CHARGE_TIMER_END, (void *)pdata);

				SPA_DBG_MSG(2, "CHARGE_TIMER_END_DUE_TO_TEMPERATURE or CHARGE_TIMER_END_DUE_TO_OVERVOLTAGE\n");
			}
			break;	//POWER_SUPPLY_STATUS_CHARGING

		case POWER_SUPPLY_STATUS_FULL:
			SPA_DBG_MSG(2, "POWER_SUPPLY_STATUS_FULL\n");

			spa_Lfs_Access(DEBUG_WRITE, POWEROFF_FULL);

			if(SPA_GET_STATUS(STATUS_EOC)) 
			{
				unsigned int v_vbat_adc;
				v_vbat_adc = spa_Get_Volt();

				SPA_DBG_MSG(2, "End Of Charge Interrupt occured : Disable Charge Bit & Set Threshold & Stop Charge Timer\n");

				//To avoid wrong Full charge interrupt add voltage condition 
				//Voltage > 3978 and Full charge interrupt will send FULL charge indication to Android platform
				//PLM : P111020-4536
				if(v_vbat_adc >= BATT_LEVEL5_VOLT) {
				//Becuase of MAXIM request. 
				// TODO: spa_Disable_Charge_Bit();
				spa_Stop_Charge_Timer(CHARGE_TIMER_END_BY_INTERRUPT, (void *)pdata);
				spa_Lfs_Access(DEBUG_WRITE, POWEROFF_EOC);
				}
				else {
					#ifdef FEAT_EN_USE_MAX14577
					MD_AL25_AccCfg_DedChgr();
					#endif
					spa_Lfs_Access(DEBUG_WRITE, POWEROFF_WRONG_EOC);
					SPA_SET_STATUS(STATUS_CHARGE, POWER_SUPPLY_STATUS_CHARGING);
				}

				SPA_SET_STATUS(STATUS_EOC, FALSE);
				SPA_SET_STATUS(STATUS_RECHG, FALSE);
			}
			else if (SPA_GET_STATUS(STATUS_TIMER_EXPIRE))
			{
				SPA_DBG_MSG(2, "6Hour or 90Min timer expired : Disable Charge Bit & Set Threshold & Stop Charge Timer\n");
				SPA_DBG_MSG(2, "Recharge Necessity will be checked after 30s\n");

				//MAXIM request us not disable charge bit but in case of Timer Expire we do.
				spa_Disable_Charge_Bit();
				spa_Stop_Charge_Timer(CHARGE_TIMER_END_BY_EXPIRE, (void *)pdata);

				spa_Lfs_Access(DEBUG_WRITE, POWEROFF_CHARGE_TIMEOUT);
				spa_Lfs_Access(DEBUG_WRITE, POWEROFF_CHARGE_DISABLE);

				SPA_SET_STATUS(STATUS_TIMER_EXPIRE, FALSE);
				SPA_SET_STATUS(STATUS_RECHG, FALSE);

			}
			else if(SPA_GET_STATUS(STATUS_RECHG))
			{
				spa_Lfs_Access(DEBUG_WRITE, POWEROFF_RECHG);
					#if 0
					 if( SPA_GET_STATUS(STATUS_IS_OVERHEAT)	\
						|| SPA_GET_STATUS(STATUS_IS_COLD)			\
						|| SPA_GET_STATUS(STATUS_IS_OVERVOLT) \
					{
					SPA_SET_STATUS(STATUS_CHARGE, POWER_SUPPLY_STATUS_DISCHARGING);
						schedule_work(&g_pdata->Charge_work);
					}
					else 
					#endif


				if(SPA_GET_STATUS(STATUS_CHG_TIMER) == CHARGE_TIMER_STARTED)
				{
					SPA_DBG_MSG(2, "Recharge already started\n");
				}
				else if(SPA_GET_STATUS(STATUS_CHG_TIMER) == CHARGE_TIMER_END_BY_EXPIRE)
				{
					SPA_DBG_MSG(2, "Recharge necessary : Enable Charge Bit again & Start 90Minute Timer\n");

					// Increase recharge count for debugging purpose
					SPA_SET_STATUS(STATUS_RECHG_COUNT, TRUE);

					spa_Disable_Charge_Bit();
					spa_Enable_Charge_Bit();
					spa_Lfs_Access(DEBUG_WRITE, POWEROFF_CHARGE_ENABLE);
					spa_Start_Charge_Timer(CHARGE_TIMER_90MIN, (void *)pdata);
				}
				else if(SPA_GET_STATUS(STATUS_CHG_TIMER) == CHARGE_TIMER_END_BY_INTERRUPT)
				{
					SPA_DBG_MSG(2, "Recharge necessary : Enable Charge Bit again & Start 6Hour Timer\n");

					// Increase recharge count for debugging purpose
					SPA_SET_STATUS(STATUS_RECHG_COUNT, TRUE);

					spa_Disable_Charge_Bit();
					spa_Enable_Charge_Bit();
					spa_Lfs_Access(DEBUG_WRITE, POWEROFF_CHARGE_ENABLE);
					spa_Start_Charge_Timer(SPA_FORCE_CHG_TIME, (void *)pdata);
				}
				else
				{
					//Blocking Defense
					SPA_DBG_MSG(2, "Rechargement necessary Error !!!!!!\n");
				}
			}
			break;	//POWER_SUPPLY_STATUS_FULL

		case POWER_SUPPLY_STATUS_NOT_CHARGING:
			if(v_Curr_Health == POWER_SUPPLY_HEALTH_GOOD) 
			{
				SPA_DBG_MSG(2, "Battery Health be Good again\n");
				SPA_SET_STATUS(STATUS_CHARGE, POWER_SUPPLY_STATUS_CHARGING);

				spa_Enable_Charge_Bit();
				spa_Lfs_Access(DEBUG_WRITE, POWEROFF_CHARGE_ENABLE);
				spa_Start_Charge_Timer(SPA_FORCE_CHG_TIME, (void *)pdata);		
			}
			else
			{
				SPA_DBG_MSG(2, "POWER_SUPPLY_STATUS_NOT_CHARGING\n");
				//MAXIM request us not disable charge bit but in case of Notcharging we do.
				spa_Disable_Charge_Bit();
				spa_Lfs_Access(DEBUG_WRITE, POWEROFF_CHARGE_DISABLE);
				spa_Stop_Charge_Timer(CHARGE_TIMER_END, (void *)pdata);

				SPA_DBG_MSG(2, "CHARGE_TIMER_END_DUE_TO_TEMPERATURE or CHARGE_TIMER_END_DUE_TO_OVERVOLTAGE\n");
			}
			break;	//POWER_SUPPLY_STATUS_NOT_CHARGING

		case POWER_SUPPLY_STATUS_DISCHARGING:
			if ((SPA_GET_STATUS(STATUS_AC) == DISCONNECT) \
				 && (SPA_GET_STATUS(STATUS_USB) == DISCONNECT) ) {
				SPA_DBG_MSG(2, "POWER_SUPPLY_STATUS_DISCHARGING : No Plugged\n");
				//MAXIM request us not disable charge bit even removing TA or USB from the device. 
				// TODO: spa_Disable_Charge_Bit();
	
				spa_Lfs_Access(DEBUG_WRITE, POWEROFF_DISCHARGE);

				spa_Stop_Charge_Timer(CHARGE_TIMER_END, (void *)pdata);	//Stop charge timer

				wake_lock_timeout(&gv_spa_wakelock, 2*HZ);
			}
			else {
				SPA_DBG_MSG(2, "POWER_SUPPLY_STATUS_DISCHARGING : Currently there is only one case entering Not Charging status. \n");

				//spa_SetProperty_CHG_Status(POWER_SUPPLY_STATUS_CHARGING);
				//schedule_work(&g_pdata->Charge_work);
			}	
			break;	//POWER_SUPPLY_STATUS_DISCHARGING
		default :
			break; 	
	}

	if(v_Prev_AC_Status != SPA_GET_STATUS(STATUS_AC)) {
		v_Prev_AC_Status = SPA_GET_STATUS(STATUS_AC);
		power_supply_changed(&spa_psy_ac);
	}
	else if (	v_Prev_USB_Status != SPA_GET_STATUS(STATUS_USB)) {
		v_Prev_USB_Status = SPA_GET_STATUS(STATUS_USB);
		power_supply_changed(&spa_psy_usb);
	}
	else {
		SPA_DBG_MSG(2, "%s : Error Call SPA_PSY_CHANGED(spa_bat_psy) \n", __func__);
		power_supply_changed(&spa_bat_psy);
	}

	//In case of powerOff charging if we remove battery device still alive. 
	//To turn off the device after removing battery we check VF every times and that's why i comment out below statement and reschedule battery work immediately.
	cancel_delayed_work(&pdata->battery_work);
	schedule_delayed_work(&pdata->battery_work, 0);

	mutex_unlock(&spa_charger_mutex);	
}


/* 
	spa_bat_work function can be called from 
	1. spa_Charge_Worker
	2. spa_Resume
	3. spa_Probe
 */
static void spa_bat_work(struct work_struct *work)
{
	struct AL25_platform_data *pdata = container_of(work, struct AL25_platform_data, battery_work.work);

	struct power_supply *psy;
	union power_supply_propval psy_data;

	static int ret_lfs = -1;
	signed long v_AverageTbat = 0;

	schedule_delayed_work(&pdata->battery_work, HZ*BAT_WORK_INTERVAL);

	spa_Bat_AverageVolt(FALSE);
	spa_Bat_AverageTemp(FALSE);

	SPA_DBG_MSG(1, "%s, Average Vbat = %ld\n",__func__, gv_AverageVbat);
	SPA_DBG_MSG(1, "%s, Capacity = %d\n",__func__, spa_Get_Capacity());

	if(SPA_GET_STATUS(STATUS_IS_RECHG_COND)) {
		SPA_DBG_MSG(1, "%s, STATUS_IS_RECHG_COND\n",__func__);
		if (timer_pending(&pdata->reCharge_start_timer)) {
			SPA_DBG_MSG(1, "reCharge_Check_Timer Exist\n"); 
		}
		else {
			SPA_SET_STATUS(STATUS_RECHG, TRUE);
			queue_work(spa_workqueue,&pdata->Charge_work);
		}
	}

	#ifdef FEAT_EN_FG_PLUSONE
	if(gv_FgOneUpdate == FALSE) {
		#if 0
		unsigned int v_Fg_Interval = 0;
		if(SPA_GET_STATUS(STATUS_IS_CHARGING))
			v_Fg_Interval = 11;
		else
			v_Fg_Interval = 16;
		#else
		unsigned int v_Fg_Interval = 11;		
		#endif

		if(gv_FgPollingCnt >= v_Fg_Interval) {
			gv_FgOneUpdate = TRUE;
			gv_FgPollingCnt = 0;
		}
		gv_FgPollingCnt++;
	}
	#endif

	if(ret_lfs)
		ret_lfs = spa_Lfs_Access(DEBUG_READ, POWEROFF_INIT);

	//In case of powerOff charging if we remove battery device still alive. 
	//To turn off the device after removing battery we check VF every times and that's why i comment out below statement and reschedule battery work immediately.
	if( (SPA_GET_STATUS(STATUS_IS_CHARGING))&&(spa_VfCheck() == FALSE))
	{
		SPA_DBG_MSG(1, " VF check FAIL!!! Status : NOT CHG, Health : DAED, AC : DISCON, USB : DISCON\n");

		SPA_SET_STATUS(STATUS_CHARGE, POWER_SUPPLY_STATUS_NOT_CHARGING);
		SPA_SET_STATUS(STATUS_HEALTH, POWER_SUPPLY_HEALTH_DEAD);
		SPA_SET_STATUS(STATUS_AC, DISCONNECT);
		SPA_SET_STATUS(STATUS_USB, DISCONNECT);
		SPA_SET_STATUS(STATUS_VF, FALSE);

		spa_Lfs_Access(DEBUG_WRITE, POWEROFF_VF);

		//Without below statement we can not turn-off the device while suspend status. 
		power_supply_changed(&spa_psy_ac);
	}


	psy = power_supply_get_by_name("battery");
	if ((psy) && (!(psy->get_property(psy,POWER_SUPPLY_PROP_TEMP , &psy_data))))
		v_AverageTbat = psy_data.intval; 

	
	if(SPA_GET_STATUS(STATUS_IS_GOOD) && SPA_GET_STATUS(STATUS_IS_CHARGING)) {
		if(v_AverageTbat <= LOW_SUSPEND_TEMP) {
			SPA_SET_STATUS(STATUS_HEALTH, POWER_SUPPLY_HEALTH_COLD);
			queue_work(spa_workqueue,&pdata->Charge_work);

			SPA_DBG_MSG(1, "spa_bat_work called! Charge Suspend, COLD, Average Temp = %ld\n", v_AverageTbat);
		}
		else if (v_AverageTbat >= HIGH_SUSPEND_TEMP) {
			SPA_SET_STATUS(STATUS_HEALTH, POWER_SUPPLY_HEALTH_OVERHEAT);
			queue_work(spa_workqueue,&pdata->Charge_work);

			SPA_DBG_MSG(1, "spa_bat_work called! Charge Suspend, OVERHEAT, Average Temp = %ld\n", v_AverageTbat);
		}
	}
	else if(SPA_GET_STATUS(STATUS_IS_COLD)) {
		if(v_AverageTbat >= LOW_RECOVER_TEMP) {
			SPA_SET_STATUS(STATUS_HEALTH, POWER_SUPPLY_HEALTH_GOOD);
			SPA_DBG_MSG(1, "spa_bat_work called! Charge Resume, Cold => Good, Average Temp = %ld\n",v_AverageTbat);

			queue_work(spa_workqueue,&pdata->Charge_work);
		}
	}
	else if(SPA_GET_STATUS(STATUS_IS_OVERHEAT)) {
		if(v_AverageTbat <= HIGH_RECOVER_TEMP) {
			SPA_SET_STATUS(STATUS_HEALTH, POWER_SUPPLY_HEALTH_GOOD);
			SPA_DBG_MSG(1, "spa_bat_work called! Charge Resume, Overheat => Good, Average Temp = %ld\n",v_AverageTbat);

			queue_work(spa_workqueue,&pdata->Charge_work);
		}
	}

	if(psy)
		power_supply_changed(psy);
}


void spa_OVP_Interrupt(unsigned char status)
{
	SPA_DBG_MSG(1, "%s, Status = %d\n",__func__, status);

	if(status) {
		SPA_SET_STATUS(STATUS_HEALTH, POWER_SUPPLY_HEALTH_OVERVOLTAGE);
		queue_work(spa_workqueue,&g_pdata->Charge_work);
	}
	else if(SPA_GET_STATUS(STATUS_HEALTH) != POWER_SUPPLY_HEALTH_GOOD) {
		SPA_SET_STATUS(STATUS_HEALTH, POWER_SUPPLY_HEALTH_GOOD);
		queue_work(spa_workqueue,&g_pdata->Charge_work);
	}

}EXPORT_SYMBOL(spa_OVP_Interrupt);

void spa_EOC_Interrupt()
{
	SPA_DBG_MSG(1, "%s\n",__func__);

	SPA_SET_STATUS(STATUS_EOC, TRUE);
	SPA_SET_STATUS(STATUS_CHARGE, POWER_SUPPLY_STATUS_FULL);

	queue_work(spa_workqueue,&g_pdata->Charge_work);
} EXPORT_SYMBOL(spa_EOC_Interrupt);


void spa_TA_Attached()
{
	SPA_DBG_MSG(1, "%s\n", __func__);

	wake_lock(&gv_spa_wakelock);

	mutex_lock(&spa_charger_mutex);	
	SPA_SET_STATUS(STATUS_AC, CONNECT);
	SPA_SET_STATUS(STATUS_CHARGE, POWER_SUPPLY_STATUS_CHARGING);
	mutex_unlock(&spa_charger_mutex);

	power_supply_changed(&spa_psy_ac);	
	
	queue_work(spa_workqueue,&g_pdata->Charge_work);
} EXPORT_SYMBOL(spa_TA_Attached);


void spa_TA_Detached()
{
	SPA_DBG_MSG(1, "%s\n", __func__);

	mutex_lock(&spa_charger_mutex);	
	SPA_SET_STATUS(STATUS_AC, DISCONNECT);
	SPA_SET_STATUS(STATUS_CHARGE, POWER_SUPPLY_STATUS_DISCHARGING);
	SPA_SET_STATUS(STATUS_HEALTH, POWER_SUPPLY_HEALTH_GOOD);
	mutex_unlock(&spa_charger_mutex);	

	power_supply_changed(&spa_psy_ac);

	queue_work(spa_workqueue,&g_pdata->Charge_work);
} EXPORT_SYMBOL(spa_TA_Detached);


void spa_USB_Attached()
{
	SPA_DBG_MSG(1, "%s\n", __func__);

	wake_lock(&gv_spa_wakelock);

	mutex_lock(&spa_charger_mutex);	
	SPA_SET_STATUS(STATUS_USB, CONNECT);
	SPA_SET_STATUS(STATUS_CHARGE, POWER_SUPPLY_STATUS_CHARGING);
	mutex_unlock(&spa_charger_mutex);

	power_supply_changed(&spa_psy_usb);	

	queue_work(spa_workqueue,&g_pdata->Charge_work);
} EXPORT_SYMBOL(spa_USB_Attached);


void spa_USB_Detached()
{
	SPA_DBG_MSG(1, "%s\n", __func__);

	mutex_lock(&spa_charger_mutex);	
	SPA_SET_STATUS(STATUS_USB, DISCONNECT);
	SPA_SET_STATUS(STATUS_CHARGE, POWER_SUPPLY_STATUS_DISCHARGING);
	SPA_SET_STATUS(STATUS_HEALTH, POWER_SUPPLY_HEALTH_GOOD);
	mutex_unlock(&spa_charger_mutex);	

	power_supply_changed(&spa_psy_usb);

	queue_work(spa_workqueue,&g_pdata->Charge_work);
} EXPORT_SYMBOL(spa_USB_Detached);

void spa_JIG_Attached()
{
	SPA_DBG_MSG(1, "%s\n", __func__);
	wake_lock(&gv_JIG_wakelock);
} EXPORT_SYMBOL(spa_JIG_Attached);

void spa_JIG_Detached()
{
	SPA_DBG_MSG(1, "%s\n", __func__);
	wake_unlock(&gv_JIG_wakelock);
}EXPORT_SYMBOL(spa_JIG_Detached);

/* For AT+syssleep Mode: force the system enter to suspend  */
void JIG_froce_unlock(void)
{
	if (wake_lock_active(&gv_JIG_wakelock)) {
		printk(KERN_WARNING "%s %s gv_JIG_wakelock is active\n", __FILE__, __FUNCTION__);
		wake_unlock(&gv_JIG_wakelock);
	}
}EXPORT_SYMBOL(JIG_froce_unlock);


int spa_Charger_Ctrl(int data)
{
	if (data) { /* start the charger */
		spa_Enable_Charge_Bit();
		spa_Lfs_Access(DEBUG_WRITE, POWEROFF_CONTROL_EN);
		#ifdef FEAT_EN_USE_MAX14577
		MD_AL25_ServiceStateMachine();
		#endif
	}
	else { /* disable the charger */
		spa_Lfs_Access(DEBUG_WRITE, POWEROFF_CONTROL_DIS);
		spa_Disable_Charge_Bit();
	}

	printk(KERN_ALERT "AL25: charger manual control: %s\n", (data)? "start" : "stop");
	return 0;
}  EXPORT_SYMBOL(spa_Charger_Ctrl);


int spa_CallBack_Status(void)
{
	return SPA_GET_STATUS(STATUS_CHARGE);
} EXPORT_SYMBOL(spa_CallBack_Status);


int spa_CallBack_Health(void)
{
	return SPA_GET_STATUS(STATUS_HEALTH);
} EXPORT_SYMBOL(spa_CallBack_Health);


int spa_CallBack_Capacity(void)
{
	return spa_Get_Capacity();
} EXPORT_SYMBOL(spa_CallBack_Capacity);


int spa_CallBack_Temp(void)
{
	int v_tbat_adc = 0;

	v_tbat_adc= spa_Get_Temp_Adc();   
	return spa_TemperatureTable(v_tbat_adc);
} EXPORT_SYMBOL(spa_CallBack_Temp);


signed int spa_is_usb_connect(void)
{
	SPA_DBG_MSG(2, "spa_is_usb_connect = <%d>\n",SPA_GET_STATUS(STATUS_USB));	
	return SPA_GET_STATUS(STATUS_USB);
} EXPORT_SYMBOL(spa_is_usb_connect);




/*============== ADD battery atribute for power of charging ========*/
#ifdef FEAT_EN_LPM_MODE
static unsigned int charging_mode_booting;  //pps-a

static void charging_mode_set(unsigned int val)
{
	charging_mode_booting=val;
}
static unsigned int charging_mode_get(void)
{
	return charging_mode_booting;
}

int get_boot_charger_info(void)
{
	int temp;
#if defined(CONFIG_PMIC_D1980)
    temp = (int)d1980_get_chargerDetect();
#elif defined(CONFIG_ALTERNATE_CHARGER)
	pm860x_get_general_user(&temp);
#endif
	return temp;
}

/*SAMSUNG_PPS: added to write value for power of charging */
int set_boot_charger_info(unsigned char data)
{
      int ret=0;
	
#if defined(CONFIG_PMIC_D1980)
        ret = d1980_set_chargerDetect(data);
#elif defined(CONFIG_ALTERNATE_CHARGER)
	ret = pm860x_set_general_user(data);
#endif

	return ret;
}
EXPORT_SYMBOL(set_boot_charger_info);

void lpm_mode_check(void)
{
	if(get_boot_charger_info())
	{	
		charging_mode_set(1);

	}
	else
	{
		charging_mode_set(0);
	}
	printk(KERN_ALERT "lpm_mode_check:charging_mode_set= %d \n",charging_mode_booting );
}

#define SEC_LPM_ATTR(_name)								\
{											\
        .attr = { .name = #_name, .mode = S_IRUGO, .owner = THIS_MODULE },	\
        .show = spa_Lpm_Show_Property,							\
        .store = spa_Lpm_Store,								\
}

static struct device_attribute AL25_battery_attrs[] = {
	SEC_LPM_ATTR(charging_source),
	SEC_LPM_ATTR(fg_soc),
	SEC_LPM_ATTR(charging_mode_booting),
	SEC_LPM_ATTR(batt_temp_check),
	SEC_LPM_ATTR(batt_full_check),
};

enum {
	BATT_CHARGING_SOURCE=0,
	BATT_FG_SOC,
	BATT_CHARGING_MODE_BOOTING,
	BATT_TEMP_CHECK,
	BATT_FULL_CHECK,
};

static ssize_t spa_Lpm_Show_Property(struct device *dev, struct device_attribute *attr, char *buf)
{
        int i = 0,battery_level=0, v_Charge_Status= 0;
        char chargerDetected=0, vbDetected=0;
        const ptrdiff_t off = attr - AL25_battery_attrs;
//ssong_power_off + 
	struct power_supply *psy;
	union power_supply_propval psy_data;
//ssong_power_off  - 

        switch (off)
	{
		case BATT_CHARGING_SOURCE: 
			#if 1
			MG_AL25_HW_I2CRead( MD_AL25_IIC_ADDRESS,MD_AL25_REG_INTSTAT2,1,&(chargerDetected));//pps-a
			//Adding vbDetect to check 5V input voltage is exist or not. 
			vbDetected = chargerDetected & 0x40;
			chargerDetected &= 0x7;
			//printk(KERN_ERR "\nBATT_CHARGING_SOURCE: chargerDetected = %d, vbDetected = %d\n",chargerDetected, vbDetected);
			if((chargerDetected >= 1) && (chargerDetected <= 5) && vbDetected)
			#else
			if ((SPA_GET_STATUS(STATUS_AC) == CONNECT) || (SPA_GET_STATUS(STATUS_USB) == CONNECT) )
			#endif
				i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", true);
			else
				i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", false);
			break;

		case BATT_FG_SOC:
			psy = power_supply_get_by_name("battery");
			if ((psy) && !(psy->get_property(psy,POWER_SUPPLY_PROP_CAPACITY , &psy_data)))
				battery_level = psy_data.intval; 
			
			printk(KERN_ERR "\nBATT_FG_SOC:  gv_AverageVbat = %ld battery = %d \n",gv_AverageVbat, battery_level);
			i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", battery_level);
			break;
		case BATT_CHARGING_MODE_BOOTING:
			i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", charging_mode_get());
			break;		
		case BATT_TEMP_CHECK:
			i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", SPA_GET_STATUS(STATUS_HEALTH));
			printk(KERN_ERR "\nBATT_TEMP_CHECK: v_Charge_Health = %d \n",SPA_GET_STATUS(STATUS_HEALTH));
			break;		
		case BATT_FULL_CHECK:
			psy = power_supply_get_by_name("battery");
			if ((psy) && !(psy->get_property(psy,POWER_SUPPLY_PROP_STATUS , &psy_data)))
				v_Charge_Status = psy_data.intval; 
                     if(v_Charge_Status == POWER_SUPPLY_STATUS_FULL)
			//if(gv_AverageVbat >= 4080)  //pps-a : adjust this voltage value properly
				i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", true);
			else
				i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", false);
			break;			

		default:
			i = -EINVAL;
        }       
        
	return i;
}



static ssize_t spa_Lpm_Store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int x = 0;
	int ret = 0;
	const ptrdiff_t off = attr - AL25_battery_attrs;

	printk(KERN_ERR " ------------->    AL25_bat_store is called  %d\n",off);

	switch (off) {
		case BATT_CHARGING_MODE_BOOTING:
			if (sscanf(buf, "%d\n", &x) == 1) {
				charging_mode_set(x);
				ret = count;
			}
			break;		
		default:
			ret = -EINVAL;
        }       

	return ret;
}


#endif	//FEAT_EN_LPM_MODE



/* +++++++++++++++++++++++++++++++++ T E S T mode ++++++++++++++++++++++++++++++++++++ */
#ifdef FEAT_EN_TEST_MODE
#define SPA_TEST_ATTR(_name)													\
{																				\
        .attr = { .name = #_name, .mode = S_IRUGO , .owner = THIS_MODULE },	\
        .show = spa_Test_Show,														\
        .store = spa_Test_Store,														\
}


enum {
	BATT_DEBUG_LEVEL =0,
	BATT_HEALTH,
	BATT_EOC,
	BATT_SET_CHG_TIMER,
	BATT_START_RECHARGE,
	BATT_AVG_VOLT,
	BATT_AVG_TEMP,
	BATT_CUR_VOLT,
	BATT_CUR_TEMP,
	BATT_VOLT_ADC,
	BATT_TEMP_ADC,
	BATT_STATUS,
	BATT_RECHG_COUNT,
	BATT_GET_TIMER_STATUS,
	BATT_LEFT_CHG_TIME,
	BATT_JIG_WAKE_UNLOCK,
#ifdef FEAT_EN_FUELGAUGE
	BATT_FG_VOLT,
	BATT_FG_LEVEL,
	BATT_FG_CUR,
#endif
};

static struct device_attribute spa_Test_Attrs[] = {
	SPA_TEST_ATTR(debug_level),
	SPA_TEST_ATTR(health),
	SPA_TEST_ATTR(EOC),
	SPA_TEST_ATTR(timerExpire),
	SPA_TEST_ATTR(reCharge),
	SPA_TEST_ATTR(average_volt),
	SPA_TEST_ATTR(average_temp),
	SPA_TEST_ATTR(current_volt),
	SPA_TEST_ATTR(current_temp),
	SPA_TEST_ATTR(volt_adc),
	SPA_TEST_ATTR(temp_adc),
	SPA_TEST_ATTR(status),
	SPA_TEST_ATTR(rechargeCount),
	SPA_TEST_ATTR(get_timer_status),
	SPA_TEST_ATTR(left_chg_time),
	SPA_TEST_ATTR(jig_wakeunlock),
#ifdef FEAT_EN_FUELGAUGE
	SPA_TEST_ATTR(fg_volt),
	SPA_TEST_ATTR(fg_level),
	SPA_TEST_ATTR(fg_curr),
#endif

};

static ssize_t spa_Test_Show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct AL25_platform_data *pdata = dev_get_drvdata(dev->parent);

	ssize_t i = 0;
	const ptrdiff_t off = attr - spa_Test_Attrs;

	switch (off)
	{
		case BATT_DEBUG_LEVEL:
			i += scnprintf(buf + i, PAGE_SIZE - i, "Debug Level = <%d>\n", gv_DebugLevel);
			break;
		case BATT_HEALTH:
			i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", SPA_GET_STATUS(STATUS_HEALTH));
			break;
		case BATT_EOC:
			i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", SPA_GET_STATUS(STATUS_EOC));
			break;
		case BATT_START_RECHARGE:
			i += scnprintf(buf + i, PAGE_SIZE -i, "%d\n", SPA_GET_STATUS(STATUS_RECHG));
			break;
		case BATT_AVG_VOLT:
			i += scnprintf(buf + i, PAGE_SIZE - i, "%ld\n", gv_AverageVbat);
			break;
		case BATT_AVG_TEMP:
			i += scnprintf(buf + i, PAGE_SIZE - i, "%ld\n", gv_AverageTbat);
			break;
		case BATT_CUR_VOLT:
			i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", spa_Get_Volt());
			break;
		case BATT_CUR_TEMP:
			i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", 	spa_TemperatureTable(spa_Get_Temp_Adc()));
			break;
		case BATT_VOLT_ADC :
			i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", spa_Get_Volt());
			break;
		case BATT_TEMP_ADC :
			i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", spa_Get_Temp_Adc());
			break;
		case BATT_STATUS:
			i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", SPA_GET_STATUS(STATUS_CHARGE));
			break;
		case BATT_RECHG_COUNT:
			i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", SPA_GET_STATUS(STATUS_RECHG_COUNT));
			break;
		case BATT_GET_TIMER_STATUS :
			i += scnprintf(buf + i, PAGE_SIZE -i, "%d\n", SPA_GET_STATUS(STATUS_CHG_TIMER));
			break;
		case BATT_LEFT_CHG_TIME :
			if(SPA_GET_STATUS(STATUS_CHG_TIMER) == CHARGE_TIMER_STARTED)
				i += scnprintf(buf + i, PAGE_SIZE -i, "%d\n", jiffies_to_msecs(pdata->Charge_timer.expires - jiffies)/1000);
			else
				i += scnprintf(buf + i, PAGE_SIZE -i, "%d\n", 0);
			break;
#ifdef FEAT_EN_FUELGAUGE
		case BATT_FG_VOLT :
			i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", spa_Get_Volt());
			break;
		case BATT_FG_LEVEL:
			i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", spa_Get_Capacity());
			break;
		#ifdef CONFIG_FUELGAUGE_STC3105
		case BATT_FG_CUR :
			i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", fg_read_current());
			break;
#endif
#endif
		default :
			break;
	}
	
	return i;
}

static ssize_t spa_Test_Store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct AL25_platform_data *pdata = dev_get_drvdata(dev->parent);

	ssize_t ret = 0;
	u16 v_value;
	const ptrdiff_t off = attr - spa_Test_Attrs;

	switch (off)
	{
		case BATT_DEBUG_LEVEL:
		        sscanf(buf, "%u", &gv_DebugLevel);
		        printk(KERN_ALERT "Debug Level = %d\n", gv_DebugLevel);

		        ret = count;
			break;
		case BATT_HEALTH:
			sscanf(buf, "%hd", &v_value);
		       printk(KERN_ALERT "Charge health = %d\n", v_value);

			SPA_SET_STATUS(STATUS_HEALTH, v_value);

			queue_work(spa_workqueue,&pdata->Charge_work);

			ret = count;
			break;
		case BATT_EOC:
			sscanf(buf, "%hd", &v_value);
		       printk(KERN_ALERT "EndOfCharge = <%d>\n", v_value);

			SPA_SET_STATUS(STATUS_EOC, TRUE);

			spa_EOC_Interrupt();

			ret=count;
			break;
		case BATT_SET_CHG_TIMER:
		{
			u16 v_Remain_Time;

			sscanf(buf, "%hd", &v_Remain_Time);
		       printk(KERN_ALERT "Timer exipre after <%d>s\n", v_Remain_Time);

			if(timer_pending(&pdata->Charge_timer))
			{
				SPA_DBG_MSG(1, "spa_Stop_Charge_Timer : PendingTimer exist, Delete Timer\n"); 
				del_timer(&pdata->Charge_timer);
			}
			
			pdata->Charge_timer.expires = jiffies + v_Remain_Time*HZ; 

			add_timer(&pdata->Charge_timer);
			SPA_SET_STATUS(STATUS_CHG_TIMER, CHARGE_TIMER_STARTED);
		}
			ret = count;
			break;
		case BATT_START_RECHARGE:
			sscanf(buf, "%hd", &v_value);
		       printk(KERN_ALERT "isRecharge = %d\n", v_value);

			SPA_SET_STATUS(STATUS_RECHG, TRUE);	
				
			queue_work(spa_workqueue,&pdata->Charge_work);

			ret = count;
			break;
		case BATT_AVG_VOLT:
			sscanf(buf, "%ld", &gv_AverageVbat);
		       printk(KERN_ALERT "Battery Voltage stored as = %ld\n", gv_AverageVbat);

			if(gv_AverageVbat == -100)
				isTestMode = FALSE;
			else
			isTestMode = TRUE;

			ret = count;
			break;
		case BATT_AVG_TEMP:
			sscanf(buf, "%ld", &gv_AverageTbat);
		       printk(KERN_ALERT "Battery Temperature stored as = %ld\n", gv_AverageTbat);

			if(gv_AverageTbat == -100)
				isTestMode = FALSE;
			else
				isTestMode = TRUE;

			ret = count;
			break;
		case BATT_JIG_WAKE_UNLOCK:
			sscanf(buf, "%hd", &v_value);
		       printk(KERN_ALERT "BATT_JIG_WAKE_UNLOCK\n");

			JIG_froce_unlock();

			ret=count;
			break;
		default :
			break;
	}

	return ret;
}
#endif	//FEAT_EN_TEST_MODE
/*------------------------------------------- T E S T mode ----------------------------------------------*/


#ifdef CONFIG_MACH_ALKON
static void spa_threshold_wq_fnc(struct work_struct *work)
{
	unsigned int v_capacity;
	/* While not-charging it will stop reading voltage and temperature in suspend status*/
#ifndef FEAT_EN_POLLING_MODE
	if ((SPA_GET_STATUS(STATUS_AC) == DISCONNECT) \
				 && (SPA_GET_STATUS(STATUS_USB) == DISCONNECT) ) {

		//Getting current Capacity information 
		v_capacity = spa_Get_Capacity();

		if(v_capacity > BATT_FG_15PCTG) {
			spa_SetVbatThresholds(BATT_FG_15P_V, 0);
			gv_NextLevelAlarmValues = BATT_FG_15PCTG;
		}
		else if(v_capacity > BATT_FG_05PCTG) {
			spa_SetVbatThresholds(BATT_FG_05P_V, 0);
			gv_NextLevelAlarmValues = BATT_FG_05PCTG;
		}
		else {
			spa_SetVbatThresholds(BATT_LOW_VOLT, 0);
			gv_NextLevelAlarmValues = BATT_FG_00PCTG;	
		}
	}
#endif
	return 0;
}

static DECLARE_DELAYED_WORK(spa_threshold_wq, spa_threshold_wq_fnc);

#endif
/* spa_voltThresholdEvent triggered by oher module to notify battery voltage reach certain value. */
void spa_voltThresholdEvent(int event_status, void *data)
{
	pm860x_charger_print_voltage(data);
	
	#ifndef FEAT_EN_POLLING_MODE



	//Threshold condition is discharge phase
	if ((SPA_GET_STATUS(STATUS_AC) == DISCONNECT) \
		 && (SPA_GET_STATUS(STATUS_USB) == DISCONNECT) ) {

		//Check if interrupt correctly generated. 
		spa_Bat_AverageVolt(TRUE);
		//gv_NextLevelAlarmValues + 1 : Threshold interrupt has 1% tolerance.
		if(gv_NextLevelAlarmValues + 1 >= calculate_batt_level(gv_AverageVbat)) {
		
			//Event Occured. release previous threshold setting. 
			spa_SetVbatThresholds(0, 0);

			//To send gv_NextLevelAlarmValues values to Android platform. 
			gv_IsThresholdState = TRUE;

			//Capacity values are greater than 1%(15%, 5%), Device need to enter sleep status again 
			if(spa_Get_Capacity() >= 1) {
				if (!wake_lock_active(&gv_thr_wakelock)) 
					wake_lock_timeout(&gv_thr_wakelock, 1 * HZ);
			}
			//Capacity values 0%, Device turn-off.
			else {
				if (!wake_lock_active(&gv_thr_wakelock)) 
					wake_lock(&gv_thr_wakelock);
			}
			printk(KERN_INFO "%s : report %d%% capacity to Android platform\n", __func__, gv_NextLevelAlarmValues);
		}
	}
	#endif

#ifdef CONFIG_MACH_ALKON
		/* We will get this interrupt in HZ*BAT_WORK_INTERVAL */
		spa_SetVbatThresholds(0, 0); /*removing the interrupt*/
		schedule_delayed_work(&spa_threshold_wq, HZ*BAT_WORK_INTERVAL);
#endif
}

irqreturn_t m_spa_voltThresholdEvent(int irq, void *data)
{
	spa_voltThresholdEvent(TRUE, data);
	return IRQ_HANDLED;
}



void spa_tempThresholdEvent(int event_status)
{
}

irqreturn_t m_spa_tempThresholdEvent(int irq, void *data)
{
	spa_tempThresholdEvent(TRUE);
	return IRQ_HANDLED;
}



//++++++++++ Request from JooSung Jin provide charger insertion info. 
static ssize_t spa_Show_Charger_Status(struct device *dev,
				struct device_attribute *attr, char *buf)
{
        int i = 0;

	if ((SPA_GET_STATUS(STATUS_AC) == CONNECT) \
				 || (SPA_GET_STATUS(STATUS_USB) == CONNECT) ) 
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", true);
	else
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", false);

	return i;
}

static DEVICE_ATTR(is_charger_inserted, S_IRUSR|S_IWUSR,
					spa_Show_Charger_Status, NULL);
//------------- Request from JooSung Jin provide charger insertion info. 






/*============== PMIC interrupt handling workaround ========
* All PMIC events are muxed into one physical INT signal.
* The High level is inactive, low is active.
* While any event is active the INT level is low.
* The level goes up when all events are cleared.
* There are 3 EVENT registers (1/2/3) on AL25.
* The clear is done when write 1 into appropriated EVENT-bit.
*
* The TAVOR IRQ works on FALLING-EDGE
*
* TS Pen-Detect event occured it is present in the 3 register.
* USB event is in the reg 2.
*
* SCENARIO:
* The Pen-Detect event causes the INT High-to-Low and IRQ occured ans handler called.
* The handler reads reg 2, then 3
*              At this point just before reading the reg-3
*             new USB-event occured and written into reg-2
* Since the SW has already read the 2 !!_before_event_!! the 2 stays with active event inside.
* The INT-level stays active low, no falling edge, no new IRQ generated and never would be generated.
*
* The solution is:
*   Read the IRQ/GPIO level after "worker".
*   Normally it should be High-Inactive,
*   but if there is no new IRQ found and level is still Low-Active
*   repeat the "worker".
*/


static void spa_Worker_Init(unsigned int irq)
{
	int 				err;
	static unsigned int 	AL25_int_pin;
	
	AL25_int_pin = IRQ_TO_GPIO(irq);
	
	err = gpio_request(AL25_int_pin, "AL25_IRQ_GPIO");
	if (err) {
		gpio_free(AL25_int_pin);
		//printk(KERN_ERR "Request GPIO_%d failed with %d\n", AL25_int_pin, err);
		return;
	}
	gpio_direction_input(AL25_int_pin);
}
/*============== End PMIC interrupt handling workaround ========*/


static void spa_Worker(struct work_struct *work)
{
	SPA_DBG_MSG(1, "\n%s\n", __func__);
#ifdef FEAT_EN_USE_MAX14577
	MD_AL25_ServiceStateMachine();
#endif
}
		

/*
 * Levante interrupt service routine.
 * In the ISR we need to check the Status bits in Levante and according
 * to those bits to check which kind of IRQ had happened.
 */
static irqreturn_t spa_Irq_Handler(int irq, void *dev_id)
{
	struct i2c_client *client = dev_id;
	struct AL25_platform_data *pdata = client->dev.platform_data;

	printk(KERN_ALERT "spa_Irq_Handler\n");

	/* clear the irq */
	pdata->ack_irq();
	queue_work(spa_workqueue,&pdata->work);

	return IRQ_HANDLED;
}


static int spa_Remove(struct i2c_client *client)
{

	printk(KERN_INFO "--->>> spa_Remove!!!\n");

#ifdef FEAT_EN_USE_MAX14577
	MD_AL25_ModuleClose();
#endif

	free_irq(client->irq, NULL);
	return 0;
}

static int spa_Suspend(struct i2c_client *client, pm_message_t state)
{
	struct AL25_platform_data *pdata = client->dev.platform_data;
	unsigned int v_capacity;

	printk(KERN_INFO "--->>> spa_Suspend!!!\n");

	/* While not-charging it will stop reading voltage and temperature in suspend status*/
#ifndef FEAT_EN_POLLING_MODE
	if ((SPA_GET_STATUS(STATUS_AC) == DISCONNECT) \
				 && (SPA_GET_STATUS(STATUS_USB) == DISCONNECT) ) {

		//Getting current Capacity information 
		v_capacity = spa_Get_Capacity();

		if(v_capacity > BATT_FG_15PCTG) {
			spa_SetVbatThresholds(BATT_FG_15P_V, 0);
			gv_NextLevelAlarmValues = BATT_FG_15PCTG;
		}
		else if(v_capacity > BATT_FG_05PCTG) {
			spa_SetVbatThresholds(BATT_FG_05P_V, 0);
			gv_NextLevelAlarmValues = BATT_FG_05PCTG;
		}
		else {
			spa_SetVbatThresholds(BATT_LOW_VOLT, 0);
			gv_NextLevelAlarmValues = BATT_FG_00PCTG;	
		}

		cancel_delayed_work(&pdata->battery_work);
	}
#endif

	gv_IsSuspendState = TRUE;

	return 0;
}

static int spa_Resume(struct i2c_client *client)
{
	struct AL25_platform_data *pdata = client->dev.platform_data;

	printk(KERN_INFO "--->>> spa_Resume!!!\n");

#ifndef FEAT_EN_POLLING_MODE
	//In Resuume stauts we don't need Threshold interrupt
	spa_SetVbatThresholds(0, 0);

	/*	
		In case of "Sleep -> Resume", huge voltage changing can be exist.
		If gv_needOldInfoUpdate is TRUE spa_Get_Capacity 
		function will update capacity immediately 	
	*/
	gv_needOldInfoUpdate = TRUE;

	// Update Average volt and temp value
	if(isTestMode == FALSE) {
		spa_Bat_AverageVolt(TRUE);
		spa_Bat_AverageTemp(TRUE);
	}

	//Scheduling bat_work to check voltage & temp while resume status. 
	cancel_delayed_work(&pdata->battery_work);
	schedule_delayed_work(&pdata->battery_work, 0);
#endif
	
	gv_IsSuspendState = FALSE;

	return 0;
}

static int __devinit spa_Probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct AL25_platform_data 	*pdata = client->dev.platform_data;

	int ret;
	int rc = EINVAL;

	printk(KERN_INFO "--->>> spa_probe!!!\n");

	g_i2c_client 	= client;	 
	g_pdata 	= pdata;

	i2c_set_clientdata(client, (void *) pdata);

	//AL25 can detect AC or USB plugged in or out thus we register ac and usb power supply to power_supply class
	ret = power_supply_register(&client->dev, &spa_psy_ac);
	if (ret) {
		printk(KERN_ALERT "power_supply_register failed\n");
		power_supply_unregister(&spa_psy_ac);	
	} 
	
	ret = power_supply_register(&client->dev, &spa_psy_usb);
	if (ret) {
		printk(KERN_ALERT "power_supply_register failed\n");
		power_supply_unregister(&spa_psy_usb);	
	} 
	
	ret = power_supply_register(&client->dev, &spa_bat_psy);
	if (ret) {
		printk(KERN_ALERT "power_supply_register failed\n");
		power_supply_unregister(&spa_bat_psy);	
	} 
	
	//Timer Definition for restart timer after 6Hour or 90Min charge timer expire. 
	init_timer(&pdata->reCharge_start_timer);
	pdata->reCharge_start_timer.function	= spa_Expire_ReChargeCheck_Timer;
	pdata->reCharge_start_timer.data	= (u_long)pdata;
	
	//Timer Definition for charge force timer. It can be 6Hour or 90Min
	init_timer(&pdata->Charge_timer);
	pdata->Charge_timer.function	= spa_Expire_Charge_Timer;
	pdata->Charge_timer.data		= (u_long)pdata;
	pdata->Charge_timer.expires 	= 0; /* For debuging screen. */

	spa_workqueue = create_freezeable_workqueue("wq_bat_spa");
		if (!spa_workqueue) {
	//		dev_err(&pdev->dev, "%s: fail to create workqueue\n", __func__);
	//		goto err_supply_unreg_ac;
		}

	/* init workqueue */
	INIT_WORK(&pdata->work, spa_Worker);
	INIT_WORK(&pdata->Charge_work, spa_Charge_Worker);

	//Update AverageVoltage and Temperature. 
	spa_Bat_AverageVolt(TRUE);
	spa_Bat_AverageTemp(TRUE);

	INIT_DELAYED_WORK(&pdata->battery_work, spa_bat_work);
	schedule_delayed_work(&pdata->battery_work, 0);
	
	
	/* init irq */
	/* irq = IRQ_GPIO(mfp_to_gpio(MFP_PIN_GPIO92)) : tavor_evb3.c, evb3_i2c_board_info_AL25 */
	pdata->init_irq();
	
	/* PinMux_table_BENI_V1PXA955_C0.h */
	/* Current  mode 0 - USB_SW:USB_SW_IRQ {0xa8b0, AF_0, Pull Dn, SLOW, Apps, Input, BOTH */ 
	/* Previous mode 0 - USB_SW:USB_SW_IRQ {0x8b0,  AF_0, ByAF Slp:No pull, SLOW, Apps, Input, BOTH we can change both to FALLING*/
	/* GPIO92_SAMSUNG_PV2_B0 */
	ret = request_irq(client->irq, spa_Irq_Handler, IRQF_TRIGGER_FALLING, "AL25", client);
	if (ret) {
		printk(KERN_ALERT "Request IRQ for AL25 failed, return:%d\n", ret);
		return ret;
	}

#ifdef FEAT_EN_LPM_MODE 		/* PowerOff Charging */
	spa_Create_Attrs(spa_psy_usb.dev, AL25_battery_attrs, ARRAY_SIZE(AL25_battery_attrs));
#endif	//FEAT_EN_LPM_MODE
#ifdef FEAT_EN_TEST_MODE	/* Debugging */
	spa_Create_Attrs(spa_psy_ac.dev, spa_Test_Attrs, ARRAY_SIZE(spa_Test_Attrs));
#endif	//FEAT_EN_TEST_MODE

	//Following Request from JooSung Jin to know charger connection status. 
	rc = device_create_file(spa_bat_psy.dev, &dev_attr_is_charger_inserted);

	spa_Worker_Init(client->irq);


#ifdef FEAT_EN_LPM_MODE
	lpm_mode_check();
#endif	//FEAT_EN_LPM_MODE

#ifdef FEAT_EN_USE_MAX14577
	MG_AL25_ModuleInit();
#endif	//FEAT_EN_USE_MAX14577

	return 0;
}


static const struct i2c_device_id AL25_id[] = {
	{ "AL25", 0 },
	{ }
};


MODULE_DEVICE_TABLE(i2c, AL25_id); 

/* Probe & Remove function */
/* Set Driver Naem as AL25 */
static struct i2c_driver spa_driver = {
	.driver = {
		.name	= "AL25",
	},
	.probe		= spa_Probe,
	.remove		= spa_Remove,
	.suspend		= spa_Suspend,
	.resume		= spa_Resume, 
	.id_table	= AL25_id,
};

/* spa Init Function */
static int __init spa_Init(void) {
	int retVal = -EINVAL;

	printk(KERN_INFO "--->>> spa_init\n");
	
	wake_lock_init(&gv_spa_wakelock, WAKE_LOCK_SUSPEND, "spa_charge");	
	wake_lock_init(&gv_thr_wakelock, WAKE_LOCK_SUSPEND, "spa_threshold");
	wake_lock_init(&gv_JIG_wakelock, WAKE_LOCK_SUSPEND, "spa_JIG");
	retVal=i2c_add_driver(&spa_driver);

	return (retVal);
}

/* spa Exit Function */
static void __exit spa_Exit(void) {
	SPA_DBG_MSG(1, "--->>> spa_exit\n");

	flush_scheduled_work();
	wake_lock_destroy(&gv_spa_wakelock);
	wake_lock_destroy(&gv_thr_wakelock);
	wake_lock_destroy(&gv_JIG_wakelock);
	i2c_del_driver(&spa_driver);
}


module_init(spa_Init);
module_exit(spa_Exit);

MODULE_AUTHOR("YJ.Choi<yj4toe.choi@samsung.com, Minseon.Zee <minseon.zee@samsung.com>");
MODULE_DESCRIPTION("Linux Driver for MAX14577");
MODULE_LICENSE("GPL");

#endif //__SPA_C__

