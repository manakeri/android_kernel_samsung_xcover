/*
 * core.h  --  Core Driver for Dialog Semiconductor D1980 PMIC
 *
 * Copyright 2010 Dialog Semiconductor Ltd
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#ifndef __LINUX_D1980_CORE_H_
#define __LINUX_D1980_CORE_H_

#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/switch.h>
#include <linux/d1982/pmic.h>
#include <linux/d1982/rtc.h>
#include <linux/d1982/bat.h>

#include "../../../drivers/staging/android/timed_output.h"

/*
 * Register values.
 */  

#define I2C						        2

#define D1980_I2C					    "d1980"

#define D1980_IRQ					    S3C_EINT(9)

/* Module specific error codes */
#define INVALID_REGISTER				2
#define INVALID_READ					3
#define INVALID_PAGE					4

/* Total number of registers in D1980 */
#define D1980_MAX_REGISTER_CNT		    D1980_PAGE1_REG_END


#define D1980_I2C_DEVICE_NAME			"d1980_i2c"
#define D1980_I2C_ADDR					(0x90 >> 1)
//#define D1980_I2C_ADDR				(0x92>>1)	/* Panther can use both the I2C but make sure I2C 1 is configured as Standard mode */


/*
 * DA1980 Number of Interrupts
 */

#define D1980_IRQ_EVBATLOW			    0	//4 
#define D1980_IRQ_EALRAM			    1	//5
#define D1980_IRQ_ESEQRDY			    2	//6
#define D1980_IRQ_ETICK				    3	//7
                                                                                                 
#define D1980_IRQ_ENONKEY			    4	//8
#define D1980_IRQ_ETBAT				    5	//12
#define D1980_IRQ_EADCEOM			    6	//13

#define D1980_IRQ_EGPI0		        	7	//16
#define D1980_IRQ_EGPI1				    8	//17
#define D1980_IRQ_EAUDIO			    9	//18
#define D1980_IRQ_ETA                   10

#define D1980_IRQ_EGPI2				    11	//24
#define D1980_IRQ_EGPI3		    		12	//27
#define D1980_IRQ_EGPI4				    13	//28
#define D1980_IRQ_EGPI5				    14	//29

#define D1980_NUM_IRQ				    15

/* Channel Definations. The reason for it being random 
   number is to support the actual bit in the Register
*/
#define D1980_ADC_VBAT					0
#define D1980_ADC_TEMP					2
#define D1980_ADC_VF					4
#define D1980_ADC_ADCIN					5
#define D1980_ADC_TJUNC					8
#define D1980_ADC_VBBAT					9
#define D1980_ADC_NUMBER_OF_CHANNELS	10
                

#undef D1982_DEBUG
                                   
struct d1980;

struct d1980_irq {
	irq_handler_t handler;
	void *data;
};

struct d1980_hwmon {
	struct platform_device *pdev;
	struct device *classdev;
};

struct d1980_levante {
	struct platform_device *pdev;
};

struct d1980_adc {
	u16 adc_buf;
	u8  adc_flag;
};


struct d1980_onkey {
	struct platform_device *pdev;
	struct input_dev *input;
	//spinlock_t onkey_lock;
	struct mutex onkey_mutex;
	struct timer_list onkey_timer;
	struct work_struct onkey_work;
};

struct d1980_hs {
	struct platform_device *pdev;	
	struct switch_dev sdev;
	struct device *hs_dev;
	unsigned long event;
	int hsdetect_status;
	int pre_hsdetect_status;
	int hookswitch_status;
	int hsdetect_timer_count;
	int hookswitch_token_count;
	int mic_exist;
	struct timer_list hsdetect_timer;
	struct timer_list hookswitch_timer;
	struct work_struct hsdetect_work;
	struct work_struct hookswitch_work;
	struct input_dev *input;
	int pressed; 
	int pressed_code; 
	struct input_device_id ids[2];
};

struct d1980_vibrator {
    struct platform_device  *pdev;
	struct timed_output_dev vibrator_timed_dev;
    struct timer_list       vibrator_timer;
    struct work_struct      vibrator_work;
};

struct d1980 {
	struct device *dev;

	struct i2c_client *i2c_client;
	
	int (*read_dev)(struct d1980 *d1980, char reg, int size, void *dest);
	int (*write_dev)(struct d1980 *d1980, char reg, int size, void *src);
	u8 *reg_cache;

	/* Interrupt handling */
    struct work_struct      irq_work;
    struct task_struct      *irq_task;
	struct mutex            irq_lock; /* IRQ table mutex */
	struct mutex            battery_lock;
    //++ Charger control 04/Aug/2011
	struct mutex            charger_lock;
    //-- Charger control 04/Aug/2011
	struct d1980_irq        irq[D1980_NUM_IRQ];
	int                     chip_irq;
	
	struct mutex adc_mutex;
	struct delayed_work monitor_work;
	struct task_struct  *monitor_task;
	
	/* Client devices */
	struct d1980_adc adc_res[D1980_ADC_NUMBER_OF_CHANNELS];

	struct d1980_hwmon      hwmon;
	struct d1980_pmic       pmic;
	struct d1980_rtc        rtc; 
	struct d1980_onkey      onkey;
	struct d1980_power      power;
	struct d1980_hs         hsdetect;
	struct d1980_vibrator   vibrator;
};

struct d1980_bat_platform_data {
	u16 sw_temp_control_en;
	u16 sw_bat_temp_threshold;
	u16 sw_junc_temp_threshold;
	u16 bat_with_no_resistor;
	u16 bat_capacity_limit_low;
	u16 bat_capacity_full;
	u16 bat_capacity_limit_high;
	u16 bat_volt_cutoff;
};


/**
 * Data to be supplied by the platform to initialise the D1980.
 *
 * @init: Function called during driver initialisation.  Should be
 *        used by the platform to configure GPIO functions and similar.
 * @irq_high: Set if D1980 IRQ is active high.
 * @irq_base: Base IRQ for genirq (not currently used).
 */
struct d1980_platform_data {
	int (*init)(struct d1980 *d1980);
	int (*irq_init)(void);
	int irq_high;
	int irq_base;	
};


/*
 * d1980 device initialisation and exit.
 */
int d1980_device_init(struct d1980 *d1980, int irq,
		       struct d1980_platform_data *pdata);
void d1980_device_exit(struct d1980 *d1980);



/*
 * d1980 device IO
 */
int d1980_clear_bits(struct d1980 * const d1980, u8 const reg, u8 const mask);
int d1980_set_bits(struct d1980* const d1980, u8 const reg, u8 const mask);
u8 d1980_reg_read(struct d1980 * const d1980, u8 const reg);
int d1980_reg_write(struct d1980 * const d1980, u8 const reg, u8 const val);
int d1980_block_read(struct d1980 * const d1980, u8 const start_reg, u8 const regs, u8 * const dest);
int d1980_block_write(struct d1980 * const d1980, u8 const start_reg, u8 const regs, u8 * const src);


int d1980_adc_manual_conversion(struct d1980 * const d1980, const u8 chnl);


/*
 * d1980 internal interrupts
 */
int d1980_register_irq(struct d1980 *d1980, int irq, irq_handler_t handler, 
			unsigned long flags, const char *name, void *data);
int d1980_free_irq(struct d1980 *d1980, int irq);
int d1980_mask_irq(struct d1980 *d1980, int irq);
int d1980_unmask_irq(struct d1980 *d1980, int irq);
int d1980_irq_init(struct d1980 *d1980, int irq,
		    	struct d1980_platform_data *pdata);
int d1980_irq_exit(struct d1980 *d1980);

#endif
