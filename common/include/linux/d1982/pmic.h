/*
 * pmic.h  --  Power Managment Driver for Dialog D1980 PMIC
 *
 * Copyright 2011 Dialog Semiconductor Ltd.
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#ifndef __LINUX_D1980_PMIC_H
#define __LINUX_D1980_PMIC_H

#include <linux/platform_device.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/driver.h>

/*
 * Register values.
 */

/*
 * DCDC's
 */
#define D1980_BUCK_1				        0
#define D1980_BUCK_2				        1
#define D1980_BUCK_3				        2
#define D1980_BUCK_4				        3

/*
 * LDOs
 */
#define D1980_LDO_1				            4
#define D1980_LDO_2				            5
#define D1980_LDO_3				            6
#define D1980_LDO_4				            7
#define D1980_LDO_5				            8
#define D1980_LDO_6				            9
#define D1980_LDO_7				            10
#define D1980_LDO_8				            11
#define D1980_LDO_9				            12
#define D1980_LDO_10				        13

#define D1980_LDO_12				        14
#define D1980_LDO_13				        15
#define D1980_LDO_14				        16
#define D1980_LDO_16				        17
#define D1980_LDO_17				        18
#define D1980_LDO_18				        19
#define D1980_LDO_19				        20

#define D1980_LDO_AUD1						21
#define D1980_LDO_AUD2						22

#define D1980_LDO_CLASSD                    23

#define D1980_NUMBER_OF_REGULATORS			24


/* CONF and EN is same for all */
#define D1980_BUCK_LDO_CONF          		(1<<7)
#define D1980_BUCK_LDO_EN            		(1<<6)


/* Maximum value possible for VSEL */
#define D1980_MAX_VSEL               		0x3F  

/* Buck Config Validation Macros */
#define D1980_BUCK12_VOLT_UPPER			    2075
#define D1980_BUCK12_VOLT_LOWER			    500
#define D1980_BUCK3_VOLT_UPPER        		2500
#define D1980_BUCK3_VOLT_LOWER        		925
#define D1980_BUCK4_VOLT_UPPER        		2800
#define D1980_BUCK4_VOLT_LOWER        		1225
#define D1980_BUCK_VOLT_STEP         		25

#define D1980_LDO_VOLT_UPPER            	3300
#define D1980_LDO_VOLT_LOWER            	1200
#define D1980_LDO_VOLT_STEP             	50


/* DCDC's & LDO's mapping information */
#define REGULATOR_VCC                       "D1980_BUCK1"
#define REGULATOR_RFT                       "D1980_BUCK4"

#define REGULATOR_LCD                       "D1980_LDO9"
#define REGULATOR_GPS                       "D1980_LDO1"
#define REGULATOR_USB                       "D1980_LDO5"
#define REGULATOR_TOUCHKEY_LED              "D1980_LDO6"
#define REGULATOR_TCXO                      "D1980_LDO7"
#define REGULATOR_TOUCH                     "D1980_LDO8"
#define REGULATOR_TFLASH                    "D1980_LDO13"   // "D1980_LDO9"
#define REGULATOR_WIFI                      "D1980_LDO10"
#define REGULATOR_RTC                       "D1980_LDO12"
#define REGULATOR_VIBRATOR                  "D1980_LDO17"
#define REGULATOR_SIM                       "D1980_LDO4"
#define REGULATOR_CAM_AVDD                  "D1980_LDO16"
#define REGULATOR_CAM_IO                    "D1980_LDO2"
#define REGULATOR_CAM_AF                    "D1980_LDO18"
#define REGULATOR_PROXMITY                  "D1980_LDO14"
#define REGULATOR_CAM_S_CORE                "D1980_LDO19"

#define REGULATOR_AUDIO_A                   "D1980_AUD1"
#define REGULATOR_AUDIO_B                   "D1980_AUD2"
#define REGULATOR_CLASSD                    "D1980_CLASSD"

#define OUTPUT_1_2V                         (1200000)
#define OUTPUT_1_8V                         (1800000)
#define OUTPUT_2_5V                         (2500000)
#define OUTPUT_2_8V                         (2800000)
#define OUTPUT_2_9V                         (2900000)
#define OUTPUT_3_0V                         (3000000)
#define OUTPUT_3_3V                         (3300000)

// change WORKINIT -> KTHREAD
#undef D1980_INT_USE_THREAD

struct d1980;
struct platform_device;
struct regulator_init_data;


struct d1980_pmic {
	/* Number of regulators of each type on this device */
	int max_dcdc;

	/* regulator devices */
	struct platform_device *pdev[D1980_NUMBER_OF_REGULATORS];
};


int d1980_platform_regulator_init(struct d1980 *d1980);

int d1980_register_regulator(struct d1980 *d1980, int reg,
			      struct regulator_init_data *initdata);


/*
* Additional support via regulator API
*/
int d1980_regulator_set_voltage(struct regulator_dev *rdev, int min_uV, int max_uV);
			  
int d1980_regulator_get_voltage(struct regulator_dev *rdev);

int d1980_regulator_enable(struct regulator_dev *rdev);

int d1980_regulator_disable(struct regulator_dev *rdev);

unsigned int d1980_regulator_get_mode(struct regulator_dev *rdev);

int d1980_regulator_is_enabled(struct regulator_dev *rdev);

int d1980_control_vibrator(unsigned char value);

int d1980_control_gps(int value);


#endif  /* __LINUX_D1980_PMIC_H */

