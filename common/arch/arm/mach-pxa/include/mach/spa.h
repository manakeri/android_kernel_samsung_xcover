/*
 * Copyright (C) 2010, SAMSUNG Corporation.
 * Author: MinSeon Zee  <minseon.zee@samsung.com> 
 * 		   YoungJun Choi  <yj4toe.choi@samsung.com> 
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */



#ifndef __MACH_SPA_H
#define __MACH_SPA_H

#include <linux/interrupt.h>

/**-----------------------------------------------------------------------------
 *
 * Added for power off charging
 *
 *------------------------------------------------------------------------------
 */

#define FEAT_EN_LPM_MODE	1
#define FEAT_EN_TEST_MODE	1

#define FEAT_EN_CONTROL_CHARGING_LV_SUDDEN_CHANGE	1

#if defined(CONFIG_PMIC_D1980)
#define FEAT_EN_BATTERY_PSY 1
//#define FEAT_EN_POLLING_MODE 1
#endif

#if defined(CONFIG_FUELGAUGE_STC3105)
#define FEAT_EN_FUELGAUGE 1
#endif
//#define FEAT_EN_POLLING_MODE 1

/* Option for AL25 */
//#define FEAT_EN_SAMSUNG_API 1 				//To Enable SAMSUNG API
//#define MCFG_DEBUG 			1
#define FEAT_EN_USE_MAX14577
/*
#define FEAT_EN_TRC_MAIN_STATE_MACHINE 1	//To Enable Main Trace
#define FEAT_EN_TRC_I2C_TRAFFIC 1			//To Enable I2C Traffic
#define FEAT_EN_TRC_ACCESSORY 1				//To Enable Accessroy Trace
#define FEAT_EN_TRC_GLUE 1					//To Enable GLUE Trace
*/
#define AL25_REG_NUM		256 				/*0..0xff*/


/* Declare Define statement */
#ifdef CONFIG_MACH_ALKON
#define SPA_NB_TEMP_TABLE 		18

#define SPA_FORCE_CHG_TIME 	CHARGE_TIMER_6HOUR

#define BATT_FULL_VOLT			4200
#define BATT_RECHAR_VOLT		4120
#define BATT_LEVEL5_VOLT		3980
#define BATT_LEVEL4_VOLT		3860
#define BATT_LEVEL3_VOLT		3780
#define BATT_LEVEL2_VOLT		3730
#define BATT_LEVEL1_VOLT		3690
#define BATT_LEVEL0_VOLT		3580
#define BATT_LOW_VOLT			3400

#define FEAT_EN_FG_PLUSONE	1

#define BATT_FG_15PCTG			15
#define BATT_FG_05PCTG			5
#define BATT_FG_00PCTG			0
#define BATT_FG_15P_V			3660
#define BATT_FG_05P_V			3587
#else
#define SPA_NB_TEMP_TABLE 		20

#define SPA_FORCE_CHG_TIME 	CHARGE_TIMER_5HOUR

#define BATT_FULL_VOLT			4200
#define BATT_RECHAR_VOLT		4140
#define BATT_LEVEL5_VOLT		3978
#define BATT_LEVEL4_VOLT		3863
#define BATT_LEVEL3_VOLT		3778
#define BATT_LEVEL2_VOLT		3731
#define BATT_LEVEL1_VOLT		3687
#define BATT_LEVEL0_VOLT		3575
#define BATT_LOW_VOLT			3400

#define FEAT_EN_FG_PLUSONE	1

#define BATT_FG_15PCTG			15
#define BATT_FG_05PCTG			5
#define BATT_FG_15P_V			3660
#define BATT_FG_05P_V			3587
#endif


#define BAT_WORK_INTERVAL 			10

#define BAT_VOLTAGE_SAMPLE 		8
#define BAT_TEMPERATURE_SAMPLE 	4

#define TRUE		1
#define FALSE	0

#define DEBUG_READ 			0
#define DEBUG_WRITE 		1
//#define POWEROFF_CAUSE	"/sys/samsung_debug/poweroff_cause_file"
#define POWEROFF_CAUSE 	"/mnt/.lfs/poweroff"

#define DISCONNECT	-1
#define CONNECT		1

#define HIGH_SUSPEND_TEMP	650
#define LOW_SUSPEND_TEMP	-30
#define HIGH_RECOVER_TEMP	450
#define LOW_RECOVER_TEMP	0

#define SECOND	1
#define MINUTE	60*SECOND
#define HOUR		60*MINUTE

/* Optional */
#ifndef FEAT_EN_FUELGAUGE
	#ifdef FEAT_EN_FG_PLUSONE
		#define COMPENSATION_VOLTAGE 0
	#else
		#define COMPENSATION_VOLTAGE -100
	#endif
#endif


struct AL25_read_cache {
	u8  data;
	u8  hit;
	u8  cachable;
	u8  spare;
};


typedef enum {
	POWEROFF_INIT,
	POWEROFF_LOWBAT,
	POWEROFF_VF,
	POWEROFF_OVERVOLT,
	POWEROFF_HOT,
	POWEROFF_COLD,
	POWEROFF_GOOD,
	POWEROFF_FULL,
	POWEROFF_RECHG,
	POWEROFF_EOC,
	POWEROFF_CHARGE_TIMEOUT,
	POWEROFF_CHARGE_ENABLE,
	POWEROFF_CHARGE_DISABLE,
	POWEROFF_DISCHARGE,
	POWEROFF_CONTROL_EN,
	POWEROFF_CONTROL_DIS,
	POWEROFF_WRONG_EOC,
	POWEROFF_PXA95X_PM,

	POWEROFF_NB_END
} spa_poweroff_t;


typedef enum {
	CHARGE_TIMER_90MIN,
	CHARGE_TIMER_5HOUR,
	CHARGE_TIMER_6HOUR
} chargeTimer_t;

typedef enum {
	CHARGE_TIMER_INIT,
	CHARGE_TIMER_STARTED,
	CHARGE_TIMER_END_BY_INTERRUPT,
	CHARGE_TIMER_END_BY_EXPIRE,
	CHARGE_TIMER_END,
} spa_chargeTimerStatus_t;

typedef enum {
	CHARGE_STATUS_INIT,
	CHARGE_STARTED_DUE_TO_PLUG_IN,
	CHARGE_RESTART_DUE_TO_HEALTH,
	CHARGE_RESTART_DUE_TO_RECHARGE,
	CHARGE_END_BY_INTERRUPT,
	CHARGE_END_BY_TIMER_EXPIRE,
	CHARGE_DUE_TO_TEMPERATURE,
	CHARGE_END_DUE_TO_PLUG_OUT,
	CHARGE_END_DUE_TO_OVERVOLTAGE,
} spa_chargeStatus_t;


typedef enum {
	STATUS_AC,
	STATUS_USB,
	STATUS_CHARGE,
	STATUS_HEALTH,
	STATUS_EOC,
	STATUS_TIMER_EXPIRE,
	STATUS_RECHG,
	STATUS_RECHG_COUNT,
	STATUS_IS_RECHG_COND,
	STATUS_CHG_TIMER,
	STATUS_FULL,
	STATUS_IS_CHARGING,
	STATUS_IS_GOOD,
	STATUS_IS_COLD,
	STATUS_IS_OVERHEAT,
	STATUS_IS_OVERVOLT,
	STATUS_VF,
} spa_Status_t;


struct AL25_platform_data {
	int     			(*init_irq)(void);
	int     			(*ack_irq)(void);
	spinlock_t  		lock;
	struct mutex		m_lock;
	struct work_struct  work;
	struct work_struct  Charge_work;
	struct power_chip   *power_chips;
	struct timer_list 	Charge_timer;
	struct timer_list 	reCharge_start_timer;
	struct delayed_work monitor_VOLT_work; /* monitor battery */
	struct delayed_work monitor_TEMP_work; /* monitor battery */
	struct delayed_work battery_work; /* monitor battery */
	};


struct spa_status
{
	int 	ac_status;
	int 	usb_status;
	u16	Charge_Status;
	u16	Charge_Health;
	u16 	isEndOfCharge;
	u16 	isTimerExpire;
	u16 	isRecharge;
	u16 	rechargeCount;
	spa_chargeTimerStatus_t ChargeTimerStatus;
	u16	VF_Status;
};


int spa_Charger_Ctrl(int data);
int spa_CallBack_Status(void);
int spa_CallBack_Health(void);
int spa_CallBack_Capacity(void);
int spa_CallBack_Temp(void);


void spa_voltThresholdEvent(int event_status, void *data);
void spa_tempThresholdEvent(int event_status);


//////////////////////////////////////////////////////////////////////////////////

void spa_OVP_Interrupt(unsigned char status);
void spa_EOC_Interrupt(void);
void spa_TA_Attached(void);
void spa_TA_Detached(void);
void spa_USB_Attached(void);
void spa_USB_Detached(void);
void spa_JIG_Attached(void);
void spa_JIG_Detached(void);

signed int spa_is_usb_connect(void); // +SAMSUNG_YOUNGJUN : for ACAT connection

int spa_Lfs_Access(int rw_op, spa_poweroff_t value);

int spa_I2C_Write( unsigned char devAddr,
                         unsigned char devReg,        
                         unsigned short  numBytesToWrite,
                         unsigned char *pData );
                         
int spa_I2C_Read( unsigned char devAddr,
                        unsigned char devReg,
                        unsigned char numBytesToRead,
                        unsigned char *pData );

int spa_Charger_Ctrl(int data);

#ifdef CONFIG_MACH_JETTA
extern unsigned char IsLCDon(void);
#endif


//////////////////////////////////////////////////////////////////////////////////
#ifdef FEAT_EN_LPM_MODE
void lpm_mode_check(void); //pps-a
/*SAMSUNG_PPS: added to write value for power of charging */
int set_boot_charger_info(unsigned char data);
static ssize_t spa_Lpm_Show_Property(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t spa_Lpm_Store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
#endif

static ssize_t spa_Test_Show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t spa_Test_Store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);

irqreturn_t m_spa_voltThresholdEvent(int irq, void *data);
void pm860x_charger_print_voltage(void *data);

#endif /* __MACH_SPA_H */
