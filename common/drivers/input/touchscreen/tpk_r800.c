/*
 *  linux/drivers/input/touchscreen/tpk_r800.c
 *
 *  touch screen driver for tpk_r800 capacitive touch controller
 *
 *  Copyright (C) 2010, Marvell Corporation
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/freezer.h>
#include <linux/proc_fs.h>
#include <linux/clk.h>
#include <linux/i2c.h>
#include <linux/earlysuspend.h>
#include <mach/gpio.h>
#include <linux/i2c/tpk_r800.h>

#include <linux/sysctl.h>
#include <asm/system.h>

#define TS_NFINGER	2
#define NFINGER_MASK	3

#define TS_INFO_REG	0x00
#define TS_X0_MSB_REG	0x01
#define TS_X0_LSB_REG	0x02
#define TS_Y0_MSB_REG	0x03
#define TS_Y0_LSB_REG	0x04
#define TS_P0_REG	0x05
#define TS_T0_REG	0x06
#define TS_X1_MSB_REG	0x07
#define TS_X1_LSB_REG	0x08
#define TS_Y1_MSB_REG	0x09
#define TS_Y1_LSB_REG	0x0a
#define TS_P1_REG	0x0b
#define TS_T1_REG	0x0c
#define TS_VERSION	0x0f
#define TS_RESET_REG	0x10
#define TS_BUSY_REG	0x11

#define TS_RESET_BIT	(1 << 7)

#define TS_DATA_START	TS_INFO_REG
#define TS_DATA_END	TS_T1_REG
#define TS_DATA_LEN	(TS_T0_REG - TS_DATA_START + 1)
#define TS_DATA_OFF(x)	((x) - TS_DATA_START)

static int x_min;
static int y_min;
static int x_max = 1280;
static int y_max = 720;
static int invert_xy = true;
static int update_minmax;

extern int ts_linear_scale(int *x, int *y, int swap_xy);
static int tpk_r800_reset(struct i2c_client *client);

static struct ctl_table_header *sysctl_header;

struct tpk_r800 {
	struct input_dev *dev;
	struct work_struct work;
	struct i2c_client *client;
	int reported_finger_count;
	struct early_suspend early_suspend;
};

/* update abs params when min and max coordinate values are set */
int tpk_r800_proc_minmax(struct ctl_table *table, int write,
			 void __user *buffer, size_t *lenp, loff_t *ppos)
{
	int ret = proc_dointvec(table, write, buffer, lenp, ppos);

	if (!ret)
		update_minmax = true;

	return ret;
}

static ctl_table tpk_r800_proc_table[] = {
	{
	 .procname = "x-max",
	 .data = &x_max,
	 .maxlen = sizeof(int),
	 .mode = 0666,
	 .proc_handler = &tpk_r800_proc_minmax,
	 },
	{
	 .procname = "y-max",
	 .data = &y_max,
	 .maxlen = sizeof(int),
	 .mode = 0666,
	 .proc_handler = &tpk_r800_proc_minmax,
	 },
	{
	 .procname = "x-min",
	 .data = &x_min,
	 .maxlen = sizeof(int),
	 .mode = 0666,
	 .proc_handler = &tpk_r800_proc_minmax,
	 },
	{
	 .procname = "y-min",
	 .data = &y_min,
	 .maxlen = sizeof(int),
	 .mode = 0666,
	 .proc_handler = &tpk_r800_proc_minmax,
	 },
	{
	 .procname = "invert_xy",
	 .data = &invert_xy,
	 .maxlen = sizeof(int),
	 .mode = 0666,
	 .proc_handler = &proc_dointvec,
	 },
	{0}
};

static ctl_table tpk_r800_proc_root[] = {
	{
	 .procname = "ts_device",
	 .mode = 0555,
	 .child = tpk_r800_proc_table,
	 },
	{0}
};

static ctl_table tpk_r800_proc_dev_root[] = {
	{
	 .procname = "dev",
	 .mode = 0555,
	 .child = tpk_r800_proc_root,
	 },
	{0}
};

static int __init init_sysctl(void)
{
	sysctl_header = register_sysctl_table(tpk_r800_proc_dev_root);
	return 0;
}

static void __exit cleanup_sysctl(void)
{
	unregister_sysctl_table(sysctl_header);
}

static ssize_t user_irq_handshake_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct tpk_r800 *ts = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n",
		       i2c_smbus_read_byte_data(ts->client, TS_BUSY_REG));
}

static ssize_t user_irq_handshake_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct tpk_r800 *ts = dev_get_drvdata(dev);
	unsigned long val;
	int error;

	error = strict_strtoul(buf, 10, &val);
	if (error)
		return error;

	if (val) {
		i2c_smbus_write_byte_data(ts->client, TS_BUSY_REG, 0x00);
		dev_info(dev, "Clean tpk800 busy bit\n");
	}

	return count;
}

static ssize_t user_reset_tpk800(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct tpk_r800 *ts = dev_get_drvdata(dev);
	unsigned long val;
	int error;

	error = strict_strtoul(buf, 10, &val);
	if (error)
		return error;

	if (val) {
		tpk_r800_reset(ts->client);
		i2c_smbus_write_byte_data(ts->client, TS_BUSY_REG, 0x00);
		dev_info(dev, "Reset tpk800 done\n");
	}

	return count;
}

static DEVICE_ATTR(reset, S_IWUGO, NULL, user_reset_tpk800);
static DEVICE_ATTR(irq, S_IRUGO | S_IWUGO,
		   user_irq_handshake_show, user_irq_handshake_store);

static struct attribute *tpk800_attrs[] = {
	&dev_attr_reset.attr,
	&dev_attr_irq.attr,
	NULL
};

static struct attribute_group tpk800_attr_grp = {
	.attrs = tpk800_attrs,
};

static int tpk_r800_reset(struct i2c_client *client)
{
	int ret = 0;
	int retry = 3;

	pr_debug("tpk_r800_reset: reset touch\n");

	while (retry-- > 0) {
		ret = i2c_smbus_write_byte_data(client,
						TS_RESET_REG, TS_RESET_BIT);
		if (ret < 0) {
			pr_err("tpk_r800_reset: failed to reset\n");
			msleep(250);
			continue;
		}
		msleep(250);
		ret = i2c_smbus_read_byte_data(client, TS_VERSION);
		if (ret < 0) {
			pr_err("tpk_r800_reset: "
				"read product version failed after reset\n");
			msleep(250);
			continue;
		}
		break;
	}

	return ret;
}

static void out_range_coordinate_filter(int *x, int *y)
{
	if (*x < x_min)
		*x = x_min;
	else if (*x > x_max)
		*x = x_max;

	if (*y < y_min)
		*y = y_min;
	else if (*y > y_max)
		*y = y_max;
}

static void tpk_r800_irq_work(struct work_struct *work)
{
	int ret;
	int finger;
	int touch = 0;
	static int X[2];
	static int Y[2];
	static int Z[2];
	static int TID[2];
	static int TIP[2];
	unsigned char data[TS_DATA_LEN];
	struct tpk_r800 *ts = container_of(work, struct tpk_r800, work);
	struct i2c_client *client = ts->client;
	struct input_dev *input_dev = ts->dev;

	if (update_minmax) {
		input_set_abs_params(input_dev, ABS_X, x_min, x_max, 0, 0);
		input_set_abs_params(input_dev, ABS_Y, y_min, y_max, 0, 0);
		update_minmax = false;
	}

	ret = i2c_smbus_read_i2c_block_data(client,
					    TS_DATA_START, TS_DATA_LEN, data);
	if (ret < 0) {
		pr_err("tpk_r800_read_sensor: touch data read failed\n");
		goto err;
	}

	finger = data[0] & 0x3;

	if (finger == 1) {
		ret = i2c_smbus_write_byte_data(client, TS_BUSY_REG, 0x00);
		if (ret < 0) {
			pr_err("tpk_r800_irq_work: "
				"failed to clear the touchscreen interrupt\n");
			goto err;
		}
	}

	X[0] = (data[1] << 8) | data[2];
	Y[0] = (data[3] << 8) | data[4];
	Z[0] = data[5];
	TID[0] = (data[6] & 0xf0) >> 4;
	TIP[0] = data[6] & 0x0f;

	X[1] = (data[7] << 8) | data[8];
	Y[1] = (data[9] << 8) | data[10];
	Z[1] = data[11];
	TID[1] = (data[12] & 0xf0) >> 4;
	TIP[1] = data[12] & 0x0f;

	if (TID[0] == 1) {
		if (TIP[0] == 1) {	/* first finger down */
			ts_linear_scale(&X[0], &Y[0], invert_xy);
			out_range_coordinate_filter(&X[0], &Y[0]);
			input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, Z[0]);
			input_report_abs(input_dev, ABS_MT_WIDTH_MAJOR, 1);
			input_report_abs(input_dev, ABS_MT_POSITION_X, X[0]);
			input_report_abs(input_dev, ABS_MT_POSITION_Y, Y[0]);
			input_mt_sync(input_dev);
			touch++;
			input_report_abs(input_dev, ABS_X, X[0]);
			input_report_abs(input_dev, ABS_Y, Y[0]);
			input_report_abs(input_dev, ABS_TOOL_WIDTH, 1);
			input_report_abs(input_dev, ABS_PRESSURE, Z[0]);
		} else if (TIP[0] == 0) {	/* first finger up */
			input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, 0);
			input_report_abs(input_dev, ABS_MT_WIDTH_MAJOR, 0);
			input_mt_sync(input_dev);

			input_report_abs(input_dev, ABS_PRESSURE, 0);
			input_report_abs(input_dev, ABS_TOOL_WIDTH, 0);
		}
	} else if (TID[0] == 2) {
		if (TIP[0] == 1) {	/* second finger down */
			ts_linear_scale(&X[0], &Y[0], invert_xy);
			out_range_coordinate_filter(&X[0], &Y[0]);
			input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, Z[0]);
			input_report_abs(input_dev, ABS_MT_WIDTH_MAJOR, 1);
			input_report_abs(input_dev, ABS_MT_POSITION_X, X[0]);
			input_report_abs(input_dev, ABS_MT_POSITION_Y, Y[0]);
			input_mt_sync(input_dev);
		} else if (TIP[0] == 0) {	/* second finger up */
			input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, 0);
			input_report_abs(input_dev, ABS_MT_WIDTH_MAJOR, 0);
			input_mt_sync(input_dev);
		}
	}

	if (finger == 2) {
		ret =
		    i2c_smbus_read_i2c_block_data(client, TS_X1_MSB_REG,
						  TS_DATA_LEN - 1, data);
		if (ret < 0) {
			pr_err
			    ("tpk_r800_read_sensor: touch data read failed\n");
			goto err;
		}

		ret = i2c_smbus_write_byte_data(client, TS_BUSY_REG, 0x00);
		if (ret < 0) {
			pr_err("tpk_r800_irq_work: "
				"failed to clear the touchscreen interrupt\n");
			goto err;
		}

		X[1] = (data[0] << 8) | data[1];
		Y[1] = (data[2] << 8) | data[3];
		Z[1] = data[4];
		TID[1] = (data[5] & 0xf0) >> 4;
		TIP[1] = data[5] & 0x0f;

		if (TID[1] == 1) {	/* error case */
			pr_err("tpk_r800_irq_work: error case\n");
			if (TIP[1] == 1) {	/* first finger down */
				ts_linear_scale(&X[1], &Y[1], invert_xy);
				out_range_coordinate_filter(&X[1], &Y[1]);
				input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR,
						 Z[1]);
				input_report_abs(input_dev, ABS_MT_WIDTH_MAJOR,
						 1);
				input_report_abs(input_dev, ABS_MT_POSITION_X,
						 X[1]);
				input_report_abs(input_dev, ABS_MT_POSITION_Y,
						 Y[1]);
				input_mt_sync(input_dev);
				touch++;
				input_report_abs(input_dev, ABS_X, X[1]);
				input_report_abs(input_dev, ABS_Y, Y[1]);
				input_report_abs(input_dev, ABS_TOOL_WIDTH, 1);
				input_report_abs(input_dev, ABS_PRESSURE, Z[1]);
			} else if (TIP[1] == 0) {	/* first finger up */
				input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR,
						 0);
				input_report_abs(input_dev, ABS_MT_WIDTH_MAJOR,
						 0);
				input_mt_sync(input_dev);

				input_report_abs(input_dev, ABS_PRESSURE, 0);
				input_report_abs(input_dev, ABS_TOOL_WIDTH, 0);
			}
		} else if (TID[1] == 2) {
			if (TIP[1] == 1) {	/* second finger down */
				ts_linear_scale(&X[1], &Y[1], invert_xy);
				out_range_coordinate_filter(&X[1], &Y[1]);
				input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR,
						 Z[1]);
				input_report_abs(input_dev, ABS_MT_WIDTH_MAJOR,
						 1);
				input_report_abs(input_dev, ABS_MT_POSITION_X,
						 X[1]);
				input_report_abs(input_dev, ABS_MT_POSITION_Y,
						 Y[1]);
				input_mt_sync(input_dev);
			} else if (TIP[1] == 0) {	/* second finger up */
				input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR,
						 0);
				input_report_abs(input_dev, ABS_MT_WIDTH_MAJOR,
						 0);
				input_mt_sync(input_dev);
			}
		}
	}

	input_report_key(input_dev, BTN_TOUCH, touch);
	input_sync(input_dev);

	return;
err:
	tpk_r800_reset(client);
	i2c_smbus_write_byte_data(ts->client, TS_BUSY_REG, 0x00);
	return;
}

static irqreturn_t tpk_r800_interrupt(int irq, void *dev_id)
{
	struct tpk_r800 *ts = (struct tpk_r800 *)dev_id;
	schedule_work(&ts->work);

	return IRQ_HANDLED;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void tpk_r800_early_suspend(struct early_suspend *h)
{
	struct tpk_r800 *ts = container_of(h, struct tpk_r800, early_suspend);
	disable_irq(ts->client->irq);
	flush_work(&ts->work);
	if (((struct touchscreen_platform_data *)
	     (ts->client->dev.platform_data))->set_power)
		((struct touchscreen_platform_data *)
		 (ts->client->dev.platform_data))->set_power(0);
	return;
}

static void tpk_r800_late_resume(struct early_suspend *h)
{
	struct tpk_r800 *ts = container_of(h, struct tpk_r800, early_suspend);
	if (((struct touchscreen_platform_data *)
	     (ts->client->dev.platform_data))->set_power)
		((struct touchscreen_platform_data *)
		 (ts->client->dev.platform_data))->set_power(1);
	tpk_r800_reset(ts->client);
	enable_irq(ts->client->irq);
	i2c_smbus_write_byte_data(ts->client, TS_BUSY_REG, 0x00);
	return;
}
#endif

static int __devinit tpk_r800_probe(struct i2c_client *client,
				    const struct i2c_device_id *id)
{
	struct tpk_r800 *ts;
	struct input_dev *input_dev;
	int ret = 0;

	if (((struct touchscreen_platform_data *)
	     (client->dev.platform_data))->set_power)
		((struct touchscreen_platform_data *)
		 (client->dev.platform_data))->set_power(1);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("tpk_r800_probe: need I2C_FUNC_I2C\n");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}

	ret = tpk_r800_reset(client);

	if (ret < 0) {
		pr_err("tpk_r800_probe: not detected\n");
		goto err_detect_failed;
	}

	ts = kzalloc(sizeof(struct tpk_r800), GFP_KERNEL);
	input_dev = input_allocate_device();

	if (!ts || !input_dev) {
		ret = -ENOMEM;
		goto alloc_fail;
	}

	i2c_set_clientdata(client, ts);

	ts->dev = input_dev;

	input_dev->name = "tpk_r800";

	input_dev->dev.parent = &client->dev;

	__set_bit(EV_SYN, input_dev->evbit);
	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);
	__set_bit(EV_ABS, input_dev->evbit);

	input_set_abs_params(input_dev, ABS_X, x_min, x_max, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, y_min, y_max, 0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(input_dev, ABS_TOOL_WIDTH, 0, 15, 0, 0);
	/*Android MT */
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, x_min, x_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, y_min, y_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR, 0, 15, 0, 0);

	ret = input_register_device(ts->dev);
	if (ret) {
		pr_err("tpk_r800_probe: Failed to allocate input device\n");
		goto input_dev_fail;
	}

	ts->client = client;

	INIT_WORK(&ts->work, tpk_r800_irq_work);

	ret = request_irq(client->irq, tpk_r800_interrupt,
			  IRQF_DISABLED | IRQF_TRIGGER_RISING,
			  "tpk_r800 irq", ts);

	if (ret) {
		pr_err("tpk_r800_probe: request irq failed\n");
		goto irq_fail;
	}

	ret = i2c_smbus_write_byte_data(ts->client, TS_BUSY_REG, 0x00);
	if (ret) {
		pr_err("tpk_r800_probe: clear irq failed\n");
		goto irq_fail;
	}
#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_STOP_DRAWING;
	ts->early_suspend.suspend = tpk_r800_early_suspend;
	ts->early_suspend.resume = tpk_r800_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif

	dev_set_drvdata(&input_dev->dev, ts);

	ret = sysfs_create_group(&input_dev->dev.kobj, &tpk800_attr_grp);
	if (ret)
		goto irq_fail;

	return 0;

irq_fail:
	free_irq(client->irq, client);

input_dev_fail:
	i2c_set_clientdata(client, NULL);
	input_free_device(input_dev);

alloc_fail:
	kfree(ts);
err_detect_failed:
err_check_functionality_failed:
	return ret;
}

static int __devexit tpk_r800_remove(struct i2c_client *client)
{
	struct tpk_r800 *ts = i2c_get_clientdata(client);

	if (client->irq)
		free_irq(client->irq, client);

	sysfs_remove_group(&ts->dev->dev.kobj, &tpk800_attr_grp);
	i2c_set_clientdata(client, NULL);
	input_unregister_device(ts->dev);
	kfree(ts);

	return 0;
}

static struct i2c_device_id tpk_r800_idtable[] = {
	{"tpk_r800", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, tpk_r800_idtable);

static struct i2c_driver tpk_r800_driver = {
	.driver = {
		   .name = "tpk_r800",
		   },
	.id_table = tpk_r800_idtable,
	.probe = tpk_r800_probe,
	.remove = __devexit_p(tpk_r800_remove),
};

static int __init tpk_r800_ts_init(void)
{
	init_sysctl();
	return i2c_add_driver(&tpk_r800_driver);
}

static void __exit tpk_r800_ts_exit(void)
{
	cleanup_sysctl();
	i2c_del_driver(&tpk_r800_driver);
}

module_init(tpk_r800_ts_init);
module_exit(tpk_r800_ts_exit);

MODULE_DESCRIPTION("tpk_r800 touch screen driver");
MODULE_LICENSE("GPL");
