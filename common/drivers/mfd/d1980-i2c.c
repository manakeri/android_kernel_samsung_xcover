/*
 * d1980-i2c.c: I2C SSC (Synchronous Serial Communication) driver for D1980
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
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <linux/d1982/core.h>
#include <linux/d1982/d1980_reg.h> 

// Modified. 25/Jul/2011.
#define  D1980_I2C_USE_SMBUS   

static int d1980_i2c_read_device(struct d1980 *d1980, char reg,
				  int bytes, void *dest)
{
	int ret;

#ifdef D1980_I2C_USE_SMBUS
	if (bytes > 1)
		ret = i2c_smbus_read_i2c_block_data(d1980->i2c_client, reg, bytes, dest);
	else {
		ret = i2c_smbus_read_byte_data(d1980->i2c_client, reg);
		if (ret < 0)
			return ret;
		*(unsigned char *)dest = (unsigned char)ret;
	}
	return 0;
#else
	ret = i2c_master_send(d1980->i2c_client, &reg, 1);
	if (ret < 0) {
        printk("Err in i2c_master_send(0x%x)\n", reg);
		return ret;
    }
	ret = i2c_master_recv(d1980->i2c_client, dest, bytes);
	if (ret < 0) {
        printk("Err in i2c_master_recv(0x%x)\n", ret);
		return ret;
    }
	if (ret != bytes)
		return -EIO;

	return 0;
#endif /* D1980_I2C_USE_SMBUS */
}

static int d1980_i2c_write_device(struct d1980 *d1980, char reg,
				   int bytes, void *src)
{
#ifdef D1980_I2C_USE_SMBUS
	/* we add 1 byte for device register */
	u8 msg[bytes + 1];
	int ret;

	msg[0] = (unsigned char)reg;
	memcpy(&msg[1], src, bytes);

	ret = i2c_master_send(d1980->i2c_client, msg, bytes + 1);
	if (ret < 0)
		return ret;

    return 0;
#else
	/* we add 1 byte for device register */
	u8 msg[D1980_MAX_REGISTER_CNT + 1];
	int ret;

#if 0   //duplicate
	if ((reg + bytes) > D1980_MAX_REGISTER_CNT) {
        printk("Bat input to d1980_i2c_write_device(0x%x, %d)\n", reg, bytes);
		return -EINVAL;
    }
#endif
	
	msg[0] = reg;
	memcpy(&msg[1], src, bytes);
	ret = i2c_master_send(d1980->i2c_client, msg, bytes + 1);
	if (ret < 0)
		return ret;
	if (ret != bytes + 1)
		return -EIO;

	return 0;
#endif   /* D1980_I2C_USE_SMBUS */
}

static int d1980_i2c_probe(struct i2c_client *i2c,
			    const struct i2c_device_id *id)
{
	struct d1980 *d1980;
	int ret = 0;

	d1980 = kzalloc(sizeof(struct d1980), GFP_KERNEL);
	if (d1980 == NULL) {
		kfree(i2c);
		return -ENOMEM;
	}

	i2c_set_clientdata(i2c, d1980);
	d1980->dev = &i2c->dev;
	d1980->i2c_client = i2c;
	d1980->read_dev = d1980_i2c_read_device;
	d1980->write_dev = d1980_i2c_write_device;

	ret = d1980_device_init(d1980, i2c->irq, i2c->dev.platform_data);
	if (ret < 0)
		goto err;

#if defined(D1980_PMIC_OPS)
	pmic_set_ops(&d1980_pmic_ops);
#endif

	return ret;

err:
	kfree(d1980);
	return ret;
}

static int d1980_i2c_remove(struct i2c_client *i2c)
{
	struct d1980 *d1980 = i2c_get_clientdata(i2c);

	d1980_device_exit(d1980);
	kfree(d1980);

	return 0;
}

static const struct i2c_device_id d1980_i2c_id[] = {
       { D1980_I2C, 0 },
       { }
};
MODULE_DEVICE_TABLE(i2c, d1980_i2c_id);


static struct i2c_driver d1980_i2c_driver = {
	.driver = {
		   .name = D1980_I2C,
		   .owner = THIS_MODULE,
	},
	.probe = d1980_i2c_probe,
	.remove = d1980_i2c_remove,
	.id_table = d1980_i2c_id,
};

static int __init d1980_i2c_init(void)
{
	return i2c_add_driver(&d1980_i2c_driver);
}

/* Initialised very early during bootup (in parallel with Subsystem init) */
subsys_initcall(d1980_i2c_init);
//module_init(d1980_i2c_init);

static void __exit d1980_i2c_exit(void)
{
	i2c_del_driver(&d1980_i2c_driver);
}
module_exit(d1980_i2c_exit);

MODULE_AUTHOR("Dialog Semiconductor Ltd <divyang.patel@diasemi.com>");
MODULE_DESCRIPTION("I2C MFD driver for Dialog D1980 PMIC plus Audio");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" D1980_I2C);
