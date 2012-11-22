/*
 * d1981-i2c.c: I2C SSC (Synchronous Serial Communication) driver for D1981
 *   
 * Copyright(c) 2010 Dialog Semiconductor Ltd.
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

//#include <linux/d1982/core.h>
//#include <linux/d1982/reg.h>

#define D1981_I2C_USE_SMBUS

u8 d1981_reg_read(u8 const reg);
int d1981_reg_write(u8 const reg, u8 const val);
int d1981_block_write(u8 const num_regs, u8 *src);

static int d1981_i2c_write_device( int bytes, void *src);
static int d1981_i2c_read_device(char reg, int bytes, void *dest);

static DEFINE_MUTEX(io_mutex);


struct d1981 {
	struct device *dev;

	struct i2c_client *i2c_client;
	
	int (*read_dev)(char reg, int size, void *dest);
	int (*write_dev)( int size, void *src);
	u8 *reg_cache;
};

/* Unique ID allocation */
static struct i2c_client *g_client;


static int d1981_read(u8 const reg, int const num_regs, u8 * const dest)
{
	int bytes = num_regs;

	if ((bytes) > 256) {
		return -EINVAL;
	}


  /* Actually write it out */
  return d1981_i2c_read_device(reg, bytes, (char *)dest);
}

static int d1981_write(int const num_bytes, u8 * const src)
{

	if ((num_bytes) > 256) {
		return -EINVAL;
	}

	/* Actually write it out */
	return d1981_i2c_write_device(num_bytes, (char *)src);
}


u8 d1981_reg_read(u8 const reg)
{
	u8 data;
	int err;

	mutex_lock(&io_mutex);
	
	d1981_read(reg, 1, &data);
	mutex_unlock(&io_mutex);
	return data;
}
EXPORT_SYMBOL_GPL(d1981_reg_read);


int d1981_reg_write(u8 const reg, u8 const val)
{
	int ret;
	u8 msg[2];

	msg[0] = reg;
	msg[1] = val;

	mutex_lock(&io_mutex);
	ret = d1981_write( 2, msg);
//	if (ret)
//		printk(KERN_ALERT "Write to reg R%x failed ret=%x\n", reg, ret);
	mutex_unlock(&io_mutex);
	return ret;
}
EXPORT_SYMBOL_GPL(d1981_reg_write);


int d1981_block_write(u8 const num_regs, u8 *src)
{
	int ret = 0;

	mutex_lock(&io_mutex);
	ret = d1981_write(num_regs, src);
	//if (ret)
		//dev_err(d1981->dev, "block write starting at R%d failed\n",start_reg);
	mutex_unlock(&io_mutex);
	return ret;
}
EXPORT_SYMBOL_GPL(d1981_block_write);




static int d1981_i2c_read_device(char reg, int bytes, void *dest)
{
	int ret;

	if (g_client == NULL)	/* No global client pointer? */
		return -1;

#ifdef D1981_I2C_USE_SMBUS
	if (bytes > 1)
		ret = i2c_smbus_read_i2c_block_data(g_client, reg, bytes, dest);
	else {
		ret = i2c_smbus_read_byte_data(g_client, reg);
		if (ret < 0)
			return ret;
		*(unsigned char *)dest = (unsigned char)ret;
	}
	return 0;
#else
//DLG TODO: Single call to i2c_transfer with two msg is safer than calling 
//DLG TODO: twice with single msg in "i2c_master_send" and "i2c_master_recv"!
//DLG TODO: in the light of atomic operation request
	ret = i2c_master_send(g_client, &reg, 1);
	if (ret < 0) {
        printk("Err in i2c_master_send(0x%x)\n", reg);
		return ret;
    }
	ret = i2c_master_recv(g_client, dest, bytes);
	if (ret < 0) {
        printk("Err in i2c_master_recv\n");
		return ret;
    }
	if (ret != bytes)
		return -EIO;
	return 0;
#endif
}

static int d1981_i2c_write_device( int bytes, void *src)
{
	int ret;

	if (g_client == NULL)	/* No global client pointer? */
		return -42;

#if 0   // 25/Jul/2011 duplicate
	if ((bytes) > 256) {
        printk("Bad input to d1981_i2c_write_device( %d)\n",  bytes);
		return -EINVAL;
	}
#endif

	ret = i2c_master_send(g_client,(char *) src, bytes );
	if (ret < 0)
		return ret;

	return 0;
}

static int d1981_i2c_probe(struct i2c_client *i2c,
			    const struct i2c_device_id *id)
{
	struct d1981 *d1981;
	int ret = 0;

	d1981 = kzalloc(sizeof(struct d1981), GFP_KERNEL);
	if (d1981 == NULL) {
		kfree(i2c);
		return -ENOMEM;
	}

	i2c_set_clientdata(i2c, d1981);
	d1981->dev = &i2c->dev;
	d1981->i2c_client = i2c;
	d1981->read_dev = d1981_i2c_read_device;
	d1981->write_dev = d1981_i2c_write_device;

	g_client = d1981->i2c_client;

	printk(KERN_ALERT "**** D1981 I2C Driver Loaded **** I2C Addr 0x%x\n", g_client->addr);

	return ret;

err:
	kfree(d1981);
	return ret;
}

static int d1981_i2c_remove(struct i2c_client *i2c)
{
	struct d1981 *d1981 = i2c_get_clientdata(i2c);

	kfree(d1981);

	return 0;
}

static const struct i2c_device_id d1981_i2c_id[] = {
       { "d1981", 0 },
       { }
};
MODULE_DEVICE_TABLE(i2c, d1981_i2c_id);


static struct i2c_driver d1981_i2c_driver = {
	.driver = {
		   .name = "d1981",
		   .owner = THIS_MODULE,
	},
	.probe = d1981_i2c_probe,
	.remove = d1981_i2c_remove,
	.id_table = d1981_i2c_id,
};

static int __init d1981_i2c_init(void)
{
	int ret;
	ret =i2c_add_driver(&d1981_i2c_driver);
	printk(KERN_ALERT "******** d1981_i2c_init (%d)***********\n", ret);
	return (ret);
}
/* init early so consumer devices can complete system boot */
subsys_initcall(d1981_i2c_init);

static void __exit d1981_i2c_exit(void)
{
	i2c_del_driver(&d1981_i2c_driver);
}
module_exit(d1981_i2c_exit);

MODULE_AUTHOR("Dialog Semiconductor Ltd <alan.austin@diasemi.com>");
MODULE_DESCRIPTION("I2C SSC driver for Dialog D1981  Audio");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" D1981_SSC);



