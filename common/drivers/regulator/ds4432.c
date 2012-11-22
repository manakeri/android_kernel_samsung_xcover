/*
 * Regulators driver for ds4432
 *
 * Copyright (C) 2011 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/ds4432.h>
#include <linux/slab.h>
#include <linux/delay.h>

#define DS4432_PATH_CNT		(2)
#define DS4432_MAX_STEP		(127)
#define DS4432_AS_SOURCE	(0x80)

struct ds4432_regulator_info {
	int total;
	struct regulator_dev *regulator;
	struct i2c_client *i2c;
	struct regulator_desc desc;

	struct ds4432_dac_data *config;
};
static inline unsigned char ds4432_path_to_reg(unsigned path_idx)
{
	return (path_idx == 0) ? (0xf8) : (0xf9);
}

/* I2C operations */
static int ds4432_read(struct i2c_client *i2c, unsigned char reg,
		       unsigned char *val)
{
	int ret;

	ret = i2c_smbus_read_byte_data(i2c, reg);
	if (ret >= 0)
		*val = (unsigned char)ret;
	return (ret >= 0) ? 0 : (ret);
}

static int ds4432_write(struct i2c_client *i2c, unsigned char reg,
			unsigned char val)
{
	unsigned char buf[2];
	int ret;

	/* send the DAC reg address and the value */
	buf[0] = reg;
	buf[1] = val;
	ret = i2c_master_send(i2c, (const char *)&buf, sizeof(buf));

	return (ret >= 0) ? 0 : (ret);
}

/* DS4432 DAC control register 8 bits
   [7]		0: to sink; 1: to source
   [6:0]	steps to sink/source

   bit[7] looks like a sign bit, but the value of the register is
   not a complemental code considering the bit[6:0] is a absolute
   distance from the zero point.
*/
static int ds4432_set_current(struct ds4432_regulator_info *pdata, int c10nA)
{
	unsigned char reg, val;
	int idx, step;

	reg = ds4432_path_to_reg(pdata->config->dac_path);
	val = 0;
	step = pdata->config->cstep_10nA;

	/* Use min_uA to get step marker, base is 0 according DS4432 feature */
	if (c10nA < 0) {
		/* source */
		idx = (0 - c10nA) / step;
		if (idx > DS4432_MAX_STEP)
			return -EINVAL;
		val = DS4432_AS_SOURCE | ((unsigned char)idx);
	} else if (c10nA > 0) {
		/* sink */
		idx = (c10nA + step - 1) / step;
		if (val > DS4432_MAX_STEP)
			return -EINVAL;
		val = (unsigned char)idx;
	} else {
		idx = 0;
		val = 0;
	}

	/* val should be in range of unsigned char here */
	return ds4432_write(pdata->i2c, reg, val);
}

static int ds4432_get_current(struct ds4432_regulator_info *pdata, int *c10nA)
{
	unsigned char reg, val;
	int idx;

	reg = ds4432_path_to_reg(pdata->config->dac_path);
	if (ds4432_read(pdata->i2c, reg, &val) < 0)
		return -EINVAL;
	idx = (int)(val & ~DS4432_AS_SOURCE);	/* index */
	idx = (idx * pdata->config->cstep_10nA);	/* 10 nA */
	*c10nA = (val & DS4432_AS_SOURCE) ? (-idx) : idx;
	return 0;
}

/* Common required interfaces for DS4432 regulators */
static int ds4432_enable(struct regulator_dev *rdev)
{
	return 0;
}

/* the following functions is for DS4432 current regulation mode */
static int ds4432_set_current_limit(struct regulator_dev *rdev,
				    int min_uA, int max_uA)
{
	struct ds4432_regulator_info *pdata = rdev_get_drvdata(rdev);
	return ds4432_set_current(pdata, min_uA * 100);
}

static int ds4432_get_current_limit(struct regulator_dev *rdev)
{
	int c10nA;
	struct ds4432_regulator_info *pdata = rdev_get_drvdata(rdev);
	ds4432_get_current(pdata, &c10nA);
	return (int)(c10nA / 100);
}

/* the following functions is for DS4432+DCDC voltage regulation mode */
static int ds4432_get_voltage(struct regulator_dev *rdev)
{
	int c10nA, uV;
	struct ds4432_regulator_info *pdata = rdev_get_drvdata(rdev);

	if (ds4432_get_current(pdata, &c10nA) < 0)
		return -EINVAL;
	if (pdata->config->param_convert(pdata->config->dac_path,
					 DS4432_DCDC_CURRENT_TO_VOLTAGE,
					 c10nA, &uV) < 0)
		return -EINVAL;
	return uV;
}

static int ds4432_set_voltage(struct regulator_dev *rdev,
			      int min_uV, int max_uV)
{
	int c10nA;
	struct ds4432_regulator_info *pdata = rdev_get_drvdata(rdev);

	if (pdata->config->param_convert(pdata->config->dac_path,
					 DS4432_DCDC_VOLTAGE_TO_CURRENT,
					 min_uV, &c10nA) < 0)
		return -EINVAL;

	return ds4432_set_current(pdata, c10nA);
}

static struct regulator_ops ds4432_ops = {
	.enable = ds4432_enable,
	.set_current_limit = ds4432_set_current_limit,
	.get_current_limit = ds4432_get_current_limit,
};

static struct regulator_ops ds4432_dcdc_ops = {
	.enable = ds4432_enable,
	.set_voltage = ds4432_set_voltage,
	.get_voltage = ds4432_get_voltage,
};

static int __devinit ds4432_regulator_probe(struct i2c_client *client,
					    const struct i2c_device_id *id)
{
	struct ds4432_platform_data *pdata = client->dev.platform_data;
	struct ds4432_regulator_info *info;
	int ret, i;
	unsigned char val;

	if (!pdata || !pdata->regulators) {
		dev_err(&client->dev, "No platform configuration data\n");
		return -EINVAL;
	}

	if ((pdata->regulator_count > DS4432_PATH_CNT)
	    || (pdata->regulator_count == 0)) {
		dev_err(&client->dev, "Invalid configuration data\n");
		return -EINVAL;
	}

	/* test read */
	ret = ds4432_read(client, ds4432_path_to_reg(0), &val);
	if (ret < 0) {
		dev_err(&client->dev, "Failed to detect:%d\n", ret);
		return -EINVAL;
	}

	info = kzalloc(pdata->regulator_count * sizeof(*info), GFP_KERNEL);
	if (!info) {
		dev_err(&client->dev, "No enough memory\n");
		return -ENOMEM;
	}

	i2c_set_clientdata(client, info);
	for (i = 0; i < pdata->regulator_count; i++) {
		info[i].total = pdata->regulator_count;
		info[i].i2c = client;
		info[i].config = &pdata->regulators[i];
		info[i].desc.name = info[i].config->name;
		info[i].desc.id = info[i].config->dac_path;
		info[i].desc.type = info[i].config->type;
		info[i].desc.owner = THIS_MODULE;
		if (info[i].config->type == REGULATOR_VOLTAGE)
			info[i].desc.ops = &ds4432_dcdc_ops;
		else
			info[i].desc.ops = &ds4432_ops;

		info[i].regulator = regulator_register(&info[i].desc,
						       &client->dev,
						       &info[i].config->initdat,
						       &info[i]);
		if (IS_ERR(info[i].regulator)) {
			dev_err(&client->dev, "failed to register %s\n",
				info[i].desc.name);
			ret = PTR_ERR(info[i].regulator);
			info[i].regulator = NULL;
			goto out;
		}

		if (info[i].config->type == REGULATOR_VOLTAGE) {
			dev_info(&client->dev, "%s online: voltage is %d uV\n",
				 info[i].desc.name,
				 ds4432_get_voltage(info[i].regulator));
		} else {
			dev_info(&client->dev, "%s online: current is %d uA\n",
				 info[i].desc.name,
				 ds4432_get_current_limit(info[i].regulator));
		}
	}

	return 0;

out:
	for (i = 0; i < pdata->regulator_count; i++) {
		if (info[i].regulator)
			regulator_unregister(info[i].regulator);
	}
	kfree(info);
	return ret;
}

static int __devexit ds4432_regulator_remove(struct i2c_client *client)
{
	int i;
	struct ds4432_regulator_info *info = i2c_get_clientdata(client);

	if (info) {
		for (i = 0; i < info[0].total; i++) {
			if (info[i].regulator)
				regulator_unregister(info[i].regulator);
		}
		kfree(info);
	}
	return 0;
}

static const struct i2c_device_id ds4432_id[] = {
	{"ds4432", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, ds4432_id);

static struct i2c_driver ds4432_driver = {
	.probe = ds4432_regulator_probe,
	.remove = __devexit_p(ds4432_regulator_remove),
	.driver = {
		   .name = "ds4432",
		   },
	.id_table = ds4432_id,
};

static int __init ds4432_init(void)
{
	return i2c_add_driver(&ds4432_driver);
}

subsys_initcall(ds4432_init);

static void __exit ds4432_exit(void)
{
	i2c_del_driver(&ds4432_driver);
}

module_exit(ds4432_exit);

/* Module information */
MODULE_DESCRIPTION("MAXIM DS4432 voltage regulator driver");
MODULE_LICENSE("GPL");
