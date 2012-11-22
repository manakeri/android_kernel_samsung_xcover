/*
 * d1980-regulator.c: Regulator driver for Dialog D1980
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

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/bitops.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/d1982/core.h>
#include <linux/d1982/pmic.h>
#include <linux/d1982/d1980_reg.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>


#define DRIVER_NAME	"d1980-regulator"
#define CONSTANT_1000	(1000)
#define HW_REGISTER_ADJUST (3)


// TODO: register regulator_consumer_supply
static struct regulator_consumer_supply regulator_supply[] = {
	[D1980_BUCK_1]	    = REGULATOR_SUPPLY("D1980_BUCK1",   NULL),
	[D1980_BUCK_2]	    = REGULATOR_SUPPLY("D1980_BUCK2",   NULL),
	[D1980_BUCK_3]	    = REGULATOR_SUPPLY("D1980_BUCK3",   NULL),
	[D1980_BUCK_4]	    = REGULATOR_SUPPLY("D1980_BUCK4",   NULL),
	[D1980_LDO_1]	    = REGULATOR_SUPPLY("D1980_LDO1",    NULL),
	[D1980_LDO_2]	    = REGULATOR_SUPPLY("D1980_LDO2",    NULL),
	[D1980_LDO_3]	    = REGULATOR_SUPPLY("D1980_LDO3",    NULL),
	[D1980_LDO_4]	    = REGULATOR_SUPPLY("D1980_LDO4",    NULL),
	[D1980_LDO_5]	    = REGULATOR_SUPPLY("D1980_LDO5",    NULL),
	[D1980_LDO_6]	    = REGULATOR_SUPPLY("D1980_LDO6",    NULL),
	[D1980_LDO_7]	    = REGULATOR_SUPPLY("D1980_LDO7",    NULL),
	[D1980_LDO_8]	    = REGULATOR_SUPPLY("D1980_LDO8",    NULL),
	[D1980_LDO_9]	    = REGULATOR_SUPPLY("D1980_LDO9",    NULL),
	[D1980_LDO_10]	    = REGULATOR_SUPPLY("D1980_LDO10",   NULL),
	[D1980_LDO_12]	    = REGULATOR_SUPPLY("D1980_LDO12",   NULL),
	[D1980_LDO_13]	    = REGULATOR_SUPPLY("D1980_LDO13",   NULL),
	[D1980_LDO_14]	    = REGULATOR_SUPPLY("D1980_LDO14",   NULL),
	[D1980_LDO_16]	    = REGULATOR_SUPPLY("D1980_LDO16",   NULL),
	[D1980_LDO_17]	    = REGULATOR_SUPPLY("D1980_LDO17",   NULL),
	[D1980_LDO_18]	    = REGULATOR_SUPPLY("D1980_LDO18",   NULL),
	[D1980_LDO_19]	    = REGULATOR_SUPPLY("D1980_LDO19",   NULL),
	[D1980_LDO_AUD1]	= REGULATOR_SUPPLY("D1980_AUD1",    NULL),
	[D1980_LDO_AUD2]	= REGULATOR_SUPPLY("D1980_AUD2",    NULL),
	[D1980_LDO_CLASSD]	= REGULATOR_SUPPLY("D1980_CLASSD",  NULL),
};


static struct regulator_consumer_supply platform_vddarm_consumers[] = {
	{
		.supply = "vcc",
	}
};
				
static struct regulator_init_data buck1_data = {
	.constraints = {
        .name = "D1980_BUCK1",
        .min_uV = D1980_BUCK12_VOLT_LOWER * CONSTANT_1000,
        .max_uV = D1980_BUCK12_VOLT_UPPER * CONSTANT_1000,
        .valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_MODE | REGULATOR_CHANGE_STATUS,
        .valid_modes_mask = REGULATOR_MODE_NORMAL,
        .always_on = 1,
    },
	//.num_consumer_supplies = ARRAY_SIZE(platform_vddarm_consumers),
	//.consumer_supplies = platform_vddarm_consumers,
	.num_consumer_supplies = 1,
	.consumer_supplies = &regulator_supply[D1980_BUCK_1],
};


static struct regulator_init_data buck2_data = {
	.constraints = {
		.name = "D1980_BUCK2",
		.min_uV = D1980_BUCK12_VOLT_LOWER * CONSTANT_1000,
		.max_uV = D1980_BUCK12_VOLT_UPPER * CONSTANT_1000,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_MODE| REGULATOR_CHANGE_STATUS,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
		.always_on = 1,
	},
	.num_consumer_supplies = 1,
	.consumer_supplies = &regulator_supply[D1980_BUCK_2],
};


static struct regulator_init_data buck3_data = {
	.constraints = {
		.name = "D1980_BUCK3",
		.min_uV = D1980_BUCK3_VOLT_LOWER * CONSTANT_1000,
		.max_uV = D1980_BUCK3_VOLT_UPPER * CONSTANT_1000,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_MODE | REGULATOR_CHANGE_STATUS,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
		.boot_on = 1,
	},
	.num_consumer_supplies = 1,
	.consumer_supplies = &regulator_supply[D1980_BUCK_3],
};

static struct regulator_init_data buck4_data = {
	.constraints = {
		.name = "D1980_BUCK4",
		.min_uV = D1980_BUCK4_VOLT_LOWER * CONSTANT_1000,
		.max_uV = D1980_BUCK4_VOLT_UPPER * CONSTANT_1000,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_MODE | REGULATOR_CHANGE_STATUS,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
		.boot_on = 1,
	},
	.num_consumer_supplies = 1,
	.consumer_supplies = &regulator_supply[D1980_BUCK_4],
};

static struct regulator_init_data ldo1_data = {
	.constraints = {
		.name = "D1980_LDO1",
		.min_uV = D1980_LDO_VOLT_LOWER * CONSTANT_1000,
		.max_uV = D1980_LDO_VOLT_UPPER * CONSTANT_1000,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
		.boot_on = 1,
	},
	.num_consumer_supplies = 1,
	.consumer_supplies = &regulator_supply[D1980_LDO_1],
};

static struct regulator_init_data ldo2_data = {
	.constraints = {
		.name = "D1980_LDO2",
		.min_uV = D1980_LDO_VOLT_LOWER * CONSTANT_1000,
		.max_uV = D1980_LDO_VOLT_UPPER * CONSTANT_1000,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
	},
	.num_consumer_supplies = 1,
	.consumer_supplies = &regulator_supply[D1980_LDO_2],
};

static struct regulator_init_data ldo3_data = {
	.constraints = {
		.name = "D1980_LDO3",
		.min_uV = D1980_LDO_VOLT_LOWER * CONSTANT_1000,
		.max_uV = D1980_LDO_VOLT_UPPER * CONSTANT_1000,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
		.boot_on = 1,
	},
	.num_consumer_supplies = 1,
	.consumer_supplies = &regulator_supply[D1980_LDO_3],
};

static struct regulator_init_data ldo4_data = {
	.constraints = {
		.name = "D1980_LDO4",
		.min_uV = D1980_LDO_VOLT_LOWER * CONSTANT_1000,
		.max_uV = D1980_LDO_VOLT_UPPER * CONSTANT_1000,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
		// ++ 2011/Sep/07. Turn off SIM power. .boot_on = 1,
	},
	.num_consumer_supplies = 1,
	.consumer_supplies = &regulator_supply[D1980_LDO_4],
};

static struct regulator_init_data ldo5_data = {
	.constraints = {
		.name = "D1980_LDO5",
		.min_uV = D1980_LDO_VOLT_LOWER * CONSTANT_1000,
		.max_uV = D1980_LDO_VOLT_UPPER * CONSTANT_1000,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
		.boot_on = 1,
	},
	.num_consumer_supplies = 1,
	.consumer_supplies = &regulator_supply[D1980_LDO_5],
};

static struct regulator_init_data ldo6_data = {
	.constraints = {
		.name = "D1980_LDO6",
		.min_uV = D1980_LDO_VOLT_LOWER * CONSTANT_1000,
		.max_uV = D1980_LDO_VOLT_UPPER * CONSTANT_1000,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
		//.boot_on = 1,
	},
	.num_consumer_supplies = 1,
	.consumer_supplies = &regulator_supply[D1980_LDO_6],
};

static struct regulator_init_data ldo7_data = {
	.constraints = {
		.name = "D1980_LDO7",
		.min_uV = D1980_LDO_VOLT_LOWER * CONSTANT_1000,
		.max_uV = D1980_LDO_VOLT_UPPER * CONSTANT_1000,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
		.boot_on = 1,
	},
	.num_consumer_supplies = 1,
	.consumer_supplies = &regulator_supply[D1980_LDO_7],
};

static struct regulator_init_data ldo8_data = {
	.constraints = {
		.name = "D1980_LDO8",
		.min_uV = D1980_LDO_VOLT_LOWER * CONSTANT_1000,
		.max_uV = D1980_LDO_VOLT_UPPER * CONSTANT_1000,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
		// 08/Aug/2011. Turned off in OBM side. .boot_on = 1,
	},
	.num_consumer_supplies = 1,
	.consumer_supplies = &regulator_supply[D1980_LDO_8],
};

static struct regulator_init_data ldo9_data = {
	.constraints = {
		.name = "D1980_LDO9",
		.min_uV = D1980_LDO_VOLT_LOWER * CONSTANT_1000,
		.max_uV = D1980_LDO_VOLT_UPPER * CONSTANT_1000,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
		.boot_on = 1,
	},
	.num_consumer_supplies = 1,
	.consumer_supplies = &regulator_supply[D1980_LDO_9],
};

static struct regulator_init_data ldo10_data = {
	.constraints = {
		.name = "D1980_LDO10",
		.min_uV = D1980_LDO_VOLT_LOWER * CONSTANT_1000,
		.max_uV = D1980_LDO_VOLT_UPPER * CONSTANT_1000,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
		.boot_on = 1,
	 },
	.num_consumer_supplies = 1,
	.consumer_supplies = &regulator_supply[D1980_LDO_10],
};

static struct regulator_init_data ldo12_data = {
	.constraints = {
		.name = "D1980_LDO12",
		.min_uV = D1980_LDO_VOLT_LOWER * CONSTANT_1000,
		.max_uV = D1980_LDO_VOLT_UPPER * CONSTANT_1000,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
		.boot_on = 1,
	 },
	.num_consumer_supplies = 1,
	.consumer_supplies = &regulator_supply[D1980_LDO_12],
};

static struct regulator_init_data ldo13_data = {
	.constraints = {
		.name = "D1980_LDO13",
		.min_uV = D1980_LDO_VOLT_LOWER * CONSTANT_1000,
		.max_uV = D1980_LDO_VOLT_UPPER * CONSTANT_1000,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
	 },
	.num_consumer_supplies = 1,
	.consumer_supplies = &regulator_supply[D1980_LDO_13],
};

static struct regulator_init_data ldo14_data = {
	.constraints = {
		.name = "D1980_LDO14",
		.min_uV = D1980_LDO_VOLT_LOWER * CONSTANT_1000,
		.max_uV = D1980_LDO_VOLT_UPPER * CONSTANT_1000,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
	 },
	.num_consumer_supplies = 1,
	.consumer_supplies = &regulator_supply[D1980_LDO_14],
};

static struct regulator_init_data ldo16_data = {
	.constraints = {
		.name = "D1980_LDO16",
		.min_uV = D1980_LDO_VOLT_LOWER * CONSTANT_1000,
		.max_uV = D1980_LDO_VOLT_UPPER * CONSTANT_1000,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
	 },
	.num_consumer_supplies = 1,
	.consumer_supplies = &regulator_supply[D1980_LDO_16],
};

static struct regulator_init_data ldo17_data = {
	.constraints = {
		.name = "D1980_LDO17",
		.min_uV = D1980_LDO_VOLT_LOWER * CONSTANT_1000,
		.max_uV = D1980_LDO_VOLT_UPPER * CONSTANT_1000,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
	 },
	.num_consumer_supplies = 1,
	.consumer_supplies = &regulator_supply[D1980_LDO_17],
};

static struct regulator_init_data ldo18_data = {
	.constraints = {
		.name = "D1980_LDO18",
		.min_uV = D1980_LDO_VOLT_LOWER * CONSTANT_1000,
		.max_uV = D1980_LDO_VOLT_UPPER * CONSTANT_1000,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
	 },
	.num_consumer_supplies = 1,
	.consumer_supplies = &regulator_supply[D1980_LDO_18],
};

static struct regulator_init_data ldo19_data = {
	.constraints = {
		.name = "D1980_LDO19",
		.min_uV = D1980_LDO_VOLT_LOWER * CONSTANT_1000,
		.max_uV = D1980_LDO_VOLT_UPPER * CONSTANT_1000,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
	 },
	.num_consumer_supplies = 1,
	.consumer_supplies = &regulator_supply[D1980_LDO_19],
};

static struct regulator_init_data ldoAUD1_data = {
	.constraints = {
		.name = "D1980_AUD1",
		.min_uV = D1980_LDO_VOLT_LOWER * CONSTANT_1000,
		.max_uV = D1980_LDO_VOLT_UPPER * CONSTANT_1000,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
	 },
	.num_consumer_supplies = 1,
	.consumer_supplies = &regulator_supply[D1980_LDO_AUD1],
};

static struct regulator_init_data ldoAUD2_data = {
	.constraints = {
		.name = "D1980_AUD2",
		.min_uV = D1980_LDO_VOLT_LOWER * CONSTANT_1000,
		.max_uV = D1980_LDO_VOLT_UPPER * CONSTANT_1000,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
	 },
	.num_consumer_supplies = 1,
	.consumer_supplies = &regulator_supply[D1980_LDO_AUD2],
};

static struct regulator_init_data ldoClassD_data = {
	.constraints = {
		.name = "D1980_CLASSD",
		.min_uV = D1980_LDO_VOLT_LOWER * CONSTANT_1000,
		.max_uV = D1980_LDO_VOLT_UPPER * CONSTANT_1000,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
	 },
	.num_consumer_supplies = 1,
	.consumer_supplies = &regulator_supply[D1980_LDO_CLASSD],
};

int __init d1980_platform_regulator_init(struct d1980 *d1980)
{
	d1980_register_regulator(d1980, D1980_BUCK_1,       &buck1_data);
	d1980_register_regulator(d1980, D1980_BUCK_2,       &buck2_data);
	d1980_register_regulator(d1980, D1980_BUCK_3,       &buck3_data);
	d1980_register_regulator(d1980, D1980_BUCK_4,       &buck4_data);
	d1980_register_regulator(d1980, D1980_LDO_1,        &ldo1_data);
	d1980_register_regulator(d1980, D1980_LDO_2,        &ldo2_data);
	d1980_register_regulator(d1980, D1980_LDO_3,        &ldo3_data);
	d1980_register_regulator(d1980, D1980_LDO_4,        &ldo4_data);
	d1980_register_regulator(d1980, D1980_LDO_5,        &ldo5_data);
	d1980_register_regulator(d1980, D1980_LDO_6,        &ldo6_data);
	d1980_register_regulator(d1980, D1980_LDO_7,        &ldo7_data);
	d1980_register_regulator(d1980, D1980_LDO_8,        &ldo8_data);
	d1980_register_regulator(d1980, D1980_LDO_9,        &ldo9_data);
	d1980_register_regulator(d1980, D1980_LDO_10,       &ldo10_data);

	d1980_register_regulator(d1980, D1980_LDO_12,       &ldo12_data);
	d1980_register_regulator(d1980, D1980_LDO_13,       &ldo13_data);
	d1980_register_regulator(d1980, D1980_LDO_14,       &ldo14_data);
	d1980_register_regulator(d1980, D1980_LDO_16,       &ldo16_data);
	d1980_register_regulator(d1980, D1980_LDO_17,       &ldo17_data);
	d1980_register_regulator(d1980, D1980_LDO_18,       &ldo18_data);
	d1980_register_regulator(d1980, D1980_LDO_19,       &ldo19_data);

	d1980_register_regulator(d1980, D1980_LDO_AUD1,     &ldoAUD1_data);
	d1980_register_regulator(d1980, D1980_LDO_AUD2,     &ldoAUD2_data);
	d1980_register_regulator(d1980, D1980_LDO_CLASSD,   &ldoClassD_data);

	return 0;
}


static unsigned int d1980_regulator_val_to_mvolts(unsigned int val, 
                    unsigned int buck_ldo, struct regulator_dev *rdev)
{
	struct regulation_constraints *constraints;
	int min_mV;
	constraints =  rdev->constraints;
	min_mV = constraints->min_uV / CONSTANT_1000;
  	
	if ( (buck_ldo == D1980_BUCK_1) || (buck_ldo == D1980_BUCK_2) || 
	     (buck_ldo == D1980_BUCK_3) || (buck_ldo == D1980_BUCK_4)  )
    	return ((val * D1980_BUCK_VOLT_STEP) + min_mV );
	return ((val * D1980_LDO_VOLT_STEP) + min_mV );     
}

static unsigned int d1980_regulator_mvolts_to_val(unsigned int mV, 
                    unsigned int buck_ldo, struct regulator_dev *rdev)
{
	struct regulation_constraints *constraints =  rdev->constraints;
	int min_mV = constraints->min_uV / CONSTANT_1000;

	if ( (buck_ldo == D1980_BUCK_1) || (buck_ldo == D1980_BUCK_2) || 
	     (buck_ldo == D1980_BUCK_3) || (buck_ldo == D1980_BUCK_4)  )	     
    	return ((mV - min_mV) / D1980_BUCK_VOLT_STEP);
	return ((mV - min_mV) / D1980_LDO_VOLT_STEP);   
}  


int d1980_regulator_set_voltage(struct regulator_dev *rdev, int min_uV, int max_uV)
{
	struct d1980 *d1980 = rdev_get_drvdata(rdev);
	int mV_val, min_mV, max_mV, tmp, ret = 0;
	int tmp_max_mV, tmp_min_mV;
	unsigned int reg_num, buck_ldo = rdev_get_id(rdev);
	struct regulation_constraints *constraints =  rdev->constraints;
  
	u8 val;

	max_mV = max_uV / CONSTANT_1000;
	min_mV = min_uV / CONSTANT_1000;
	tmp_max_mV = constraints->max_uV / CONSTANT_1000;
	tmp_min_mV = constraints->min_uV / CONSTANT_1000;
	
	if (0 == max_mV)
		max_mV = tmp_max_mV;
  
	if (min_mV > max_mV)
		return -EINVAL;

	if (min_mV < tmp_min_mV || min_mV > tmp_max_mV)
		return -EINVAL;
	if (max_mV < tmp_min_mV || max_mV > tmp_max_mV)
		return -EINVAL;
	
	/* before we do anything check the lock bit */
	if (d1980_reg_read(d1980, D1980_SUPPLY_REG) & D1980_SUPPLY_VLOCK)	
		d1980_clear_bits(d1980, D1980_SUPPLY_REG, D1980_SUPPLY_VLOCK);
		
	mV_val =  d1980_regulator_mvolts_to_val(min_mV, buck_ldo, rdev);

	/* Sanity check for maximum value */   		
	tmp = d1980_regulator_val_to_mvolts(mV_val, buck_ldo, rdev); 
	if (!tmp || (tmp > max_mV)) 
		return -EINVAL;
     
	/* Register address as per Datasheet. 
	*/
	if (buck_ldo <= D1980_LDO_13) {	   
		reg_num = D1980_BUCK1_REG + buck_ldo;
	} else {
		reg_num = D1980_BUCK1_REG + buck_ldo + HW_REGISTER_ADJUST;
	}	
  
	/* group it according to ramped regulator */
	switch(buck_ldo) {
		case D1980_BUCK_1:
			val = d1980_reg_read(d1980, reg_num) & ~D1980_MAX_VSEL;
			d1980_reg_write(d1980, reg_num, (val | mV_val));
			/* Now enable the ramp */
			d1980_set_bits(d1980, D1980_SUPPLY_REG, D1980_SUPPLY_VBUCK1GO); 
			break;
		case D1980_BUCK_2:
			val = d1980_reg_read(d1980, reg_num) & ~D1980_MAX_VSEL;
			d1980_reg_write(d1980, reg_num, (val | mV_val));
			/* Now enable the ramp */
			d1980_set_bits(d1980, D1980_SUPPLY_REG, D1980_SUPPLY_VBUCK2GO); 
			break;
		case D1980_BUCK_3:
			val = d1980_reg_read(d1980, reg_num) & ~D1980_MAX_VSEL;
			d1980_reg_write(d1980, reg_num, (val | mV_val));
			/* Now enable the ramp */
			d1980_set_bits(d1980, D1980_SUPPLY_REG, D1980_SUPPLY_VBUCK3GO); 
			break;
		case D1980_BUCK_4:
			val = d1980_reg_read(d1980, reg_num) & ~D1980_MAX_VSEL;
			d1980_reg_write(d1980, reg_num, (val | mV_val));
			/* Now enable the ramp */
			d1980_set_bits(d1980, D1980_SUPPLY_REG, D1980_SUPPLY_VBUCK4GO); 
			break;
	  
		case D1980_LDO_1:  
		case D1980_LDO_2:
		case D1980_LDO_3:
		case D1980_LDO_4:
		case D1980_LDO_5:
		case D1980_LDO_6:
		case D1980_LDO_7:
		case D1980_LDO_8:
		case D1980_LDO_9:
		case D1980_LDO_10:
		case D1980_LDO_12:
		case D1980_LDO_13:
		case D1980_LDO_14:
		case D1980_LDO_16:
		case D1980_LDO_17:
		case D1980_LDO_18:
		case D1980_LDO_19:
			val = d1980_reg_read(d1980, reg_num) & ~D1980_MAX_VSEL;
			d1980_reg_write(d1980, reg_num, (val | mV_val));
			break;
		default : 
			ret = -EINVAL;
			break;
	}

	return ret;
}
EXPORT_SYMBOL_GPL(d1980_regulator_set_voltage);

int d1980_regulator_get_voltage(struct regulator_dev *rdev)
{
	struct d1980 *d1980 = rdev_get_drvdata(rdev);
	unsigned int reg_num, buck_ldo = rdev_get_id(rdev);
	int ret;
	u8 val;
	
	/* Register address as per Datasheet. 
	*/
	if (buck_ldo <= D1980_LDO_13) {	   
		reg_num = D1980_BUCK1_REG + buck_ldo;
	} else {
		reg_num = D1980_BUCK1_REG + buck_ldo + HW_REGISTER_ADJUST;
	}
	switch(buck_ldo) {
		case D1980_BUCK_1:
		case D1980_BUCK_2:
		case D1980_BUCK_3:
		case D1980_BUCK_4:
		case D1980_LDO_1:     
		case D1980_LDO_2: 
		case D1980_LDO_3:     
		case D1980_LDO_4: 
		case D1980_LDO_5:
		case D1980_LDO_6:
		case D1980_LDO_7:
		case D1980_LDO_8:
		case D1980_LDO_9:
		case D1980_LDO_10:
		case D1980_LDO_12:  
		case D1980_LDO_13:
		case D1980_LDO_14:
		case D1980_LDO_16:
		case D1980_LDO_17:
		case D1980_LDO_18:
		case D1980_LDO_19:		
			val = d1980_reg_read(d1980, reg_num) & D1980_MAX_VSEL;
			ret = d1980_regulator_val_to_mvolts(val, buck_ldo, rdev) * CONSTANT_1000;
			break;
		default : 
			ret = -EINVAL;
			break;
	} 
  
	return ret;
}
EXPORT_SYMBOL_GPL(d1980_regulator_get_voltage);

int d1980_regulator_enable(struct regulator_dev *rdev)
{
	struct d1980 *d1980 = rdev_get_drvdata(rdev);
	unsigned int reg_num, buck_ldo = rdev_get_id(rdev);

	if (buck_ldo < D1980_BUCK_1 || buck_ldo >= D1980_NUMBER_OF_REGULATORS)
		return -EINVAL;
	/* Register address as per Datasheet. 
	*/
	if (buck_ldo == D1980_LDO_AUD1) {
		d1980_set_bits(d1980, D1980_SUPPLY_REG, D1980_SUPPLY_LDO15AEN);
	} else if (buck_ldo == D1980_LDO_AUD2) {
		d1980_set_bits(d1980, D1980_SUPPLY_REG, D1980_SUPPLY_LDO15BEN);
    } else if (buck_ldo == D1980_LDO_CLASSD) {
		d1980_set_bits(d1980, D1980_AUDIO_CONF2_REG_B, D1980_AUDIOCONF_CLASSDEN);
	} else if (buck_ldo <= D1980_LDO_13) {	   
		reg_num = D1980_BUCK1_REG + buck_ldo;
		d1980_set_bits(d1980, reg_num, D1980_BUCK_LDO_EN);
	} else {
		reg_num = D1980_BUCK1_REG + buck_ldo + HW_REGISTER_ADJUST;
		d1980_set_bits(d1980, reg_num, D1980_BUCK_LDO_EN);
	}
#ifdef D1982_DEBUG
	printk(KERN_INFO "[REGULATOR] : [%s] >> %s(%d) : Enable\n", __func__, rdev->desc->name, buck_ldo);
#endif

	return 0;
}
EXPORT_SYMBOL_GPL(d1980_regulator_enable);

int d1980_regulator_disable(struct regulator_dev *rdev)
{
	struct d1980 *d1980 = rdev_get_drvdata(rdev);
	unsigned int reg_num, buck_ldo = rdev_get_id(rdev);

	if (buck_ldo < D1980_BUCK_1 || buck_ldo >= D1980_NUMBER_OF_REGULATORS)
		return -EINVAL;

	/* Register address as per Datasheet. 
	 */
	if (buck_ldo == D1980_LDO_AUD1) {
		d1980_clear_bits(d1980, D1980_SUPPLY_REG, D1980_SUPPLY_LDO15AEN);
	} else if (buck_ldo == D1980_LDO_AUD2) {
		d1980_clear_bits(d1980, D1980_SUPPLY_REG, D1980_SUPPLY_LDO15BEN);
	} else if (buck_ldo == D1980_LDO_CLASSD) {
	    d1980_clear_bits(d1980, D1980_AUDIO_CONF2_REG_B, D1980_AUDIOCONF_CLASSDEN);
	} else if (buck_ldo <= D1980_LDO_13) {	   
		reg_num = D1980_BUCK1_REG + buck_ldo;
		d1980_clear_bits(d1980, reg_num, D1980_BUCK_LDO_EN);
	} else {
		reg_num = D1980_BUCK1_REG + buck_ldo + HW_REGISTER_ADJUST;
		d1980_clear_bits(d1980, reg_num, D1980_BUCK_LDO_EN);
	}
#ifdef D1982_DEBUG	
	printk(KERN_INFO "[REGULATOR] : [%s] >> %s(%d) : Disable \n", __func__, rdev->desc->name, buck_ldo);

#endif
	return 0;
}
EXPORT_SYMBOL_GPL(d1980_regulator_disable);

unsigned int d1980_regulator_get_mode(struct regulator_dev *rdev)
{
	return REGULATOR_MODE_NORMAL;
}
EXPORT_SYMBOL_GPL(d1980_regulator_get_mode);

int d1980_regulator_is_enabled(struct regulator_dev *rdev)
{
	struct d1980 *d1980 = rdev_get_drvdata(rdev);
	unsigned int reg_num, buck_ldo = rdev_get_id(rdev);
	int ret = -EINVAL;

	if (buck_ldo < D1980_BUCK_1 || buck_ldo >= D1980_NUMBER_OF_REGULATORS)
		return -EINVAL;
	/* Register address as per Datasheet. 
	*/
	if (buck_ldo == D1980_LDO_AUD1) {
		ret = d1980_reg_read(d1980, D1980_SUPPLY_REG) & D1980_SUPPLY_LDO15AEN;
	} else if (buck_ldo == D1980_LDO_AUD2) {
		ret = d1980_reg_read(d1980, D1980_SUPPLY_REG) & D1980_SUPPLY_LDO15BEN;
    } else if (buck_ldo == D1980_LDO_CLASSD ) {
        ret = d1980_reg_read(d1980, D1980_AUDIO_CONF2_REG_B) & D1980_AUDIOCONF_CLASSDEN;
	} else if (buck_ldo <= D1980_LDO_13) {	   
		reg_num = D1980_BUCK1_REG + buck_ldo;
		ret = d1980_reg_read(d1980, reg_num) & D1980_BUCK_LDO_EN;
	} else {
		reg_num = D1980_BUCK1_REG + buck_ldo + HW_REGISTER_ADJUST;
		ret = d1980_reg_read(d1980, reg_num) & D1980_BUCK_LDO_EN;
	}

#ifdef D1982_DEBUG
    printk(KERN_INFO "[%s] %s >> enabled : [%d]\n", rdev->desc->name, __func__, ret);
#endif
	return ret;
}
EXPORT_SYMBOL_GPL(d1980_regulator_is_enabled);


static struct regulator_ops d1980_ldo_ops = {
	.set_voltage = d1980_regulator_set_voltage,
	.get_voltage = d1980_regulator_get_voltage,
	.enable = d1980_regulator_enable,
	.disable = d1980_regulator_disable,
	.is_enabled = d1980_regulator_is_enabled,
	.get_mode = d1980_regulator_get_mode,
};


static struct regulator_desc d1980_reg[D1980_NUMBER_OF_REGULATORS] = {
	{
		.name = "D1980_BUCK1",
		.id = D1980_BUCK_1,
		.ops = &d1980_ldo_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = D1980_MAX_VSEL + 1,
		.owner = THIS_MODULE,
	},
	{
		.name = "D1980_BUCK2",
		.id = D1980_BUCK_2,
		.ops = &d1980_ldo_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = D1980_MAX_VSEL + 1,
		.owner = THIS_MODULE,
	},
	{
		.name = "D1980_BUCK3",
		.id = D1980_BUCK_3,
		.ops = &d1980_ldo_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = D1980_MAX_VSEL + 1,
		.owner = THIS_MODULE,
	},
	{
		.name = "D1980_BUCK4",
		.id = D1980_BUCK_4,
		.ops = &d1980_ldo_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = D1980_MAX_VSEL + 1,
		.owner = THIS_MODULE,
	},
	{
		.name = "D1980_LDO1",
		.id = D1980_LDO_1,
		.ops = &d1980_ldo_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = D1980_MAX_VSEL + 1,
		.owner = THIS_MODULE,
	},
	{
		.name = "D1980_LDO2",
		.id = D1980_LDO_2,
		.ops = &d1980_ldo_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = D1980_MAX_VSEL + 1,
		.owner = THIS_MODULE,
	},
	{
		.name = "D1980_LDO3",
		.id = D1980_LDO_3,
		.ops = &d1980_ldo_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = D1980_MAX_VSEL + 1,
		.owner = THIS_MODULE,
	},
	{
		.name = "D1980_LDO4",
		.id = D1980_LDO_4,
		.ops = &d1980_ldo_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = D1980_MAX_VSEL + 1,
		.owner = THIS_MODULE,
	},
	{
		.name = "D1980_LDO5",
		.id = D1980_LDO_5,
		.ops = &d1980_ldo_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = D1980_MAX_VSEL + 1,
		.owner = THIS_MODULE,
	},
	{
		.name = "D1980_LDO6",
		.id = D1980_LDO_6,
		.ops = &d1980_ldo_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = D1980_MAX_VSEL + 1,
		.owner = THIS_MODULE,
	},
	{
		.name = "D1980_LDO7",
		.id = D1980_LDO_7,
		.ops = &d1980_ldo_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = D1980_MAX_VSEL + 1,
		.owner = THIS_MODULE,
	},
	{
		.name = "D1980_LDO8",
		.id = D1980_LDO_8,
		.ops = &d1980_ldo_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = D1980_MAX_VSEL + 1,
		.owner = THIS_MODULE,
	},
	{
		.name = "D1980_LDO9",
		.id = D1980_LDO_9,
		.ops = &d1980_ldo_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = D1980_MAX_VSEL + 1,
		.owner = THIS_MODULE,
	},
	{
		.name = "D1980_LDO10",
		.id = D1980_LDO_10,
		.ops = &d1980_ldo_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = D1980_MAX_VSEL + 1,
		.owner = THIS_MODULE,
	},
	{
		.name = "D1980_LDO12",
		.id = D1980_LDO_12,
		.ops = &d1980_ldo_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = D1980_MAX_VSEL + 1,
		.owner = THIS_MODULE,
	},
	{
		.name = "D1980_LDO13",
		.id = D1980_LDO_13,
		.ops = &d1980_ldo_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = D1980_MAX_VSEL + 1,
		.owner = THIS_MODULE,
	},
	{
		.name = "D1980_LDO14",
		.id = D1980_LDO_14,
		.ops = &d1980_ldo_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = D1980_MAX_VSEL + 1,
		.owner = THIS_MODULE,
	},
	{
		.name = "D1980_LDO16",
		.id = D1980_LDO_16,
		.ops = &d1980_ldo_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = D1980_MAX_VSEL + 1,
		.owner = THIS_MODULE,
	},
	{
		.name = "D1980_LDO17",
		.id = D1980_LDO_17,
		.ops = &d1980_ldo_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = D1980_MAX_VSEL + 1,
		.owner = THIS_MODULE,
	},
	{
		.name = "D1980_LDO18",
		.id = D1980_LDO_18,
		.ops = &d1980_ldo_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = D1980_MAX_VSEL + 1,
		.owner = THIS_MODULE,
	},
	{
		.name = "D1980_LDO19",
		.id = D1980_LDO_19,
		.ops = &d1980_ldo_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = D1980_MAX_VSEL + 1,
		.owner = THIS_MODULE,
	},
	{
		.name = "D1980_AUD1",
		.id = D1980_LDO_AUD1,
		.ops = &d1980_ldo_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = D1980_MAX_VSEL + 1,
		.owner = THIS_MODULE,
	},
	{
		.name = "D1980_AUD2",
		.id = D1980_LDO_AUD2,
		.ops = &d1980_ldo_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = D1980_MAX_VSEL + 1,
		.owner = THIS_MODULE,
	},
	{
	    .name = "D1980_CLASSD",
	    .id = D1980_LDO_CLASSD,
	    .ops = &d1980_ldo_ops,
	    .type = REGULATOR_VOLTAGE,
	    .n_voltages = D1980_MAX_VSEL + 1,
	    .owner = THIS_MODULE,
	}
};


static int d1980_regulator_probe(struct platform_device *pdev)
{
	struct regulator_dev *rdev;

	if (pdev->id < D1980_BUCK_1 || pdev->id >= D1980_NUMBER_OF_REGULATORS)
		return -ENODEV;

	/* register regulator */
	rdev = regulator_register(&d1980_reg[pdev->id], &pdev->dev,
				  pdev->dev.platform_data,
				  dev_get_drvdata(&pdev->dev));
	if (IS_ERR(rdev)) {
		dev_err(&pdev->dev, "failed to register %s\n",
			d1980_reg[pdev->id].name);
		return PTR_ERR(rdev);
	}

	return 0;
}

static int d1980_regulator_remove(struct platform_device *pdev)
{
	struct regulator_dev *rdev = platform_get_drvdata(pdev);

	regulator_unregister(rdev);

	return 0;
}

int d1980_register_regulator(struct d1980 *d1980, int reg,
			      struct regulator_init_data *initdata)
{
	struct platform_device *pdev;
	int ret;
	if (reg < D1980_BUCK_1 || reg >= D1980_NUMBER_OF_REGULATORS)
		return -EINVAL;

	if (d1980->pmic.pdev[reg])
		return -EBUSY;

	pdev = platform_device_alloc(DRIVER_NAME, reg);
	if (!pdev)
		return -ENOMEM;

	d1980->pmic.pdev[reg] = pdev;

	initdata->driver_data = d1980;

	pdev->dev.platform_data = initdata;
	pdev->dev.parent = d1980->dev;
	platform_set_drvdata(pdev, d1980);

	ret = platform_device_add(pdev);

	if (ret != 0) {
		dev_err(d1980->dev, "Failed to register regulator %d: %d\n",
			reg, ret);
		platform_device_del(pdev);
		d1980->pmic.pdev[reg] = NULL;
	}

	return ret;
}

EXPORT_SYMBOL_GPL(d1980_register_regulator);


static struct platform_driver d1980_regulator_driver = {
	.probe = d1980_regulator_probe,
	.remove = d1980_regulator_remove,
	.driver		= {
		.name	= DRIVER_NAME,
	},
};

static int __init d1980_regulator_init(void)
{
	return platform_driver_register(&d1980_regulator_driver);
}
subsys_initcall(d1980_regulator_init);

static void __exit d1980_regulator_exit(void)
{
	platform_driver_unregister(&d1980_regulator_driver);
}
module_exit(d1980_regulator_exit);

/* Module information */
MODULE_AUTHOR("Dialog Semiconductor Ltd <divyang.patel@diasemi.com>");
MODULE_DESCRIPTION("D1980 voltage and current regulator driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRIVER_NAME);
