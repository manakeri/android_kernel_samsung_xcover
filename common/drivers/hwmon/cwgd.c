/* cwgd.c - cwgd gyroscope driver
 *
 * Copyright (C) 2010 CyWee Group Ltd.
 * Author: Joe Wei <joewei@cywee.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/input.h>
#include <linux/sysfs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/cwgd.h>
#include <linux/slab.h>
#include <asm/div64.h>
#include <linux/interrupt.h>
#include <linux/earlysuspend.h>
#include <mach/gpio.h>
#include <plat/mfp.h>

#define DEV_NAME		"CyWee CWGD 3-axis digital Gyroscope"

/* Use the old-fashion ioctl to control device */
#define USE_MISC_DEVICE		1

/* First-In First-Out buffer settings */
#define USE_FIFO			0

#define DPS_MAX			(1 << (16 - 1))

/* cwgd gyroscope registers */
#define WHO_AM_I        0x0F

#define CTRL_REG1       0x20	/* power control reg */
#define CTRL_REG2       0x21	/* power control reg */
#define CTRL_REG3       0x22	/* power control reg */
#define CTRL_REG4       0x23	/* interrupt control reg */
#define CTRL_REG5       0x24	/* interrupt control reg */
#define OUT_TEMP_REG	0x26	/* Temperature register */
#define STATUS_REG		0x27	/* Status register */
#define AXISDATA_REG    0x28
#define OUT_Y_L			0x2A
#define FIFO_CTRL_REG	0x2E
#define FIFO_SRC_REG	0x2F

#define BLOCK_DATA_UPDATE		0x80
#define READ_MULTIPLE_BYTES		0x80
#define ENABLE_LPF2				0x02

#define CWGD_PM_OFF				0x00
#define CWGD_PM_NORMAL			0x08
#define CWGD_ENABLE_ALL_AXES	0x07

#define STATUS_REG_ZYX_DATA_AVAILABLE		0x08
#define ZYX_DATA_OVERRUN					0x80

/* Interrupt sources */
#define DATA_READY_INT			0x08
#define FIFO_WATERMARK_INT		0x04

#define ODR100			0x00	/* ODR = 100Hz */
#define ODR200			0x40	/* ODR = 200Hz */
#define ODR400			0x80	/* ODR = 400Hz */
#define ODR800			0xC0	/* ODR = 800Hz */

#define ODR100_BW25	0x10	/* ODR = 100Hz, Bandwidth = 25Hz */
#define ODR400_BW110	0xB0	/* ODR = 400Hz, Bandwidth = 110Hz */
#define ODR800_BW110	0xF0	/* ODR = 800Hz, Bandwidth = 100Hz */

/** Registers Contents */
#define WHOAMI_CWGD		0x00D3	/*      Expectd content for WAI */

#define MIN_INTERVAL_MS		2
#define CWGD_ENABLED		1
#define CWGD_DISABLED		0

#define FIFO_LEVEL_MASK		0x1F
#define FIFO_EMPTY			0x20
#define FIFO_ENABLE			0x40

#define FIFO_MODE_BYPASS					(0 << 5)
#define FIFO_MODE_FIFO						(1 << 5)
#define FIFO_MODE_STREAM					(2 << 5)
#define FIFO_MODE_STREAM_TO_FIFO			(3 << 5)
#define FIFO_MODE_BYPASS_TO_STREAM			(4 << 5)

enum data_status {
	DATA_STATUS_READY = 0x0,
	DATA_STATUS_NO_DATA = 0x1,
	DATA_STATUS_OVERRUN = 0x2,
};

static const char default_ctrl_regs[] = {
	ODR100_BW25 | CWGD_ENABLE_ALL_AXES | CWGD_PM_NORMAL,
	0x00,
	0x00,
#if USE_FIFO
	BLOCK_DATA_UPDATE | CWGD_FS_2000DPS,
	FIFO_ENABLE
#else
	CWGD_FS_2000DPS,
	ENABLE_LPF2
#endif
};

static struct {
	u32 delay_ns;
	unsigned int mask;
} odr_table[] = {
	{
	1250000LL, ODR800_BW110}, {
	2500000LL, ODR400_BW110}, {
	5000000LL, ODR200}, {
10000000LL, ODR100_BW25},};

struct i2c_cwgd_sensor {
	struct i2c_client *client;
	struct input_dev *input;
	unsigned int sample_interval;
	bool use_interrupt;
	struct mutex lock;

#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif

	int entries;
	bool discard_next_event;

	struct hrtimer timer;
	ktime_t polling_delay;

	/* time needed to read one entry */
	u32 time_to_read;

	u8 ctrl_regs[5];

	struct workqueue_struct *cwgd_wq;
	struct work_struct work;

	atomic_t enabled;
	int data[3];
	int bias[3];
};

static struct i2c_cwgd_sensor *gyro;

static int cwgd_i2c_write(struct i2c_cwgd_sensor *sensor,
			  u8 reg_addr, u8 *data, u8 len)
{
	int dummy;
	int i;

	for (i = 0; i < len; i++) {
		dummy = i2c_smbus_write_byte_data(sensor->client,
						  reg_addr++, data[i]);
		if (dummy) {
			pr_err("i2c write error\n");
			return dummy;
		}
	}
	return 0;
}

/* Returns the number of read bytes on success */
static int cwgd_i2c_read(struct i2c_cwgd_sensor *sensor,
			 u8 reg_addr, u8 *data, u8 len)
{
	if (len > 1)
		reg_addr |= READ_MULTIPLE_BYTES;

	return i2c_smbus_read_i2c_block_data(sensor->client, reg_addr, len,
					     data);
}

static int cwgd_read_register(struct i2c_cwgd_sensor *sensor, u8 reg)
{
	return i2c_smbus_read_byte_data(sensor->client, reg);
}

#if USE_FIFO
/**
 * Return the number of data in the fifo
 */
static int cwgd_read_fifo_status(struct i2c_cwgd_sensor *sensor)
{
	int status = i2c_smbus_read_byte_data(sensor->client, FIFO_SRC_REG);
	if (status < 0) {
		pr_err("%s(): failed to read fifo source register\n", __func__);
		return status;
	}

	return (status & FIFO_LEVEL_MASK) + !(status & FIFO_EMPTY);
}

static int cwgd_restart_fifo(struct i2c_cwgd_sensor *sensor)
{
	u8 mode = FIFO_MODE_BYPASS;
	int res = cwgd_i2c_write(sensor, FIFO_CTRL_REG, &mode, 1);
	if (res < 0) {
		pr_err("%s(): failed to turn off fifo\n", __func__);
		return res;
	}

	mode = FIFO_MODE_FIFO | (sensor->entries - 1);
	res = cwgd_i2c_write(sensor, FIFO_CTRL_REG, &mode, 1);
	if (res < 0)
		pr_err("%s(): failed to turn on fifo\n", __func__);

	return res;
}

static int cwgd_is_data_ready(struct i2c_cwgd_sensor *sensor)
{
	return DATA_STATUS_READY;
}
#else
static int cwgd_read_fifo_status(struct i2c_cwgd_sensor *sensor)
{
	return 0;
}

static int cwgd_restart_fifo(struct i2c_cwgd_sensor *sensor)
{
	return 0;
}

static enum data_status cwgd_is_data_ready(struct i2c_cwgd_sensor *sensor)
{
	int status = cwgd_read_register(sensor, STATUS_REG);

	if (!(status & STATUS_REG_ZYX_DATA_AVAILABLE))
		return DATA_STATUS_NO_DATA;

	if (status & ZYX_DATA_OVERRUN)
		return DATA_STATUS_OVERRUN;

	return DATA_STATUS_READY;
}
#endif

/**
 * Make sure the chip is real CWGD one
 *
 * @return 0 if the chip is cwgd, != 0 otherwise
 */
static int cwgd_check_chip(struct i2c_cwgd_sensor *sensor)
{
	int wmi = cwgd_read_register(sensor, WHO_AM_I);
	return !(wmi == WHOAMI_CWGD);
}

static int cwgd_set_range(u8 range)
{
	int err = -1;
	int res = -1;
	u8 data = 0;

	res = cwgd_i2c_read(gyro, CTRL_REG4, &data, 1);

	if (res >= 0)
		data = data & 0x00CF;
	else
		return res;

	data = range | data;
	err = cwgd_i2c_write(gyro, CTRL_REG4, &data, 1);
	return err;
}

static int cwgd_set_delay(struct i2c_cwgd_sensor *sensor, int delay)
{
	int i = 0;
	u8 value = 0;
	u64 delay_ns = delay * NSEC_PER_MSEC;
	int err = 0;

	if (atomic_read(&sensor->enabled)) {
		if (sensor->use_interrupt)
			disable_irq(sensor->client->irq);
		else
			hrtimer_cancel(&sensor->timer);
	}

	if (delay_ns <= odr_table[0].delay_ns) {
		delay_ns = odr_table[0].delay_ns;
		sensor->time_to_read = odr_table[0].delay_ns;
		sensor->entries = 1;
	} else {
		for (i = ARRAY_SIZE(odr_table) - 1; i >= 0; i--)
			if (delay_ns >= odr_table[i].delay_ns) {
				delay_ns = odr_table[i].delay_ns;
				sensor->time_to_read = delay_ns;
				sensor->entries = 1;
				break;
			}
	}

	value = odr_table[i].mask;
	value |= (CWGD_ENABLE_ALL_AXES + CWGD_PM_NORMAL);
	sensor->ctrl_regs[0] = value;
	err = cwgd_i2c_write(sensor, CTRL_REG1, &value, 1);
	if (err < 0)
		return err;

	/* Ignore the noise data */
	msleep((u32) delay_ns * 2 / NSEC_PER_MSEC);

	cwgd_restart_fifo(sensor);

	if (atomic_read(&sensor->enabled)) {
		if (sensor->use_interrupt) {
			enable_irq(sensor->client->irq);

			/* Restart interrupt generator */
			value = 0x00;
			if (cwgd_i2c_write(sensor, CTRL_REG3, &value, 1))
				return -1;
			if (cwgd_i2c_write
			    (sensor, CTRL_REG3, &sensor->ctrl_regs[2], 1))
				return -1;
		} else {
			delay_ns = sensor->entries * sensor->time_to_read;
			sensor->polling_delay = ns_to_ktime(delay_ns);
			hrtimer_start(&sensor->timer,
				      sensor->polling_delay, HRTIMER_MODE_REL);
		}
	}

	return 0;
}

static int cwgd_align_axis(struct i2c_cwgd_sensor *sensor, int *aligned_data)
{
	int i = 0;
	int j = 0;
	int sum = 0;
	int *axes = NULL;

	if (!(struct cwgd_platform_data *)sensor->client->dev.platform_data) {
		for (i = 0; i < 3; i++)
			aligned_data[i] = sensor->data[i];

		return 0;
	}

	axes =
	    ((struct cwgd_platform_data *)sensor->client->dev.
	     platform_data)->axes;
	for (i = 0; i < 3; i++) {
		sum = 0;
		for (j = 0; j < 3; j++)
			sum += sensor->data[j] * axes[i * 3 + j];
		aligned_data[i] = sum;
	}

	return 0;
}

/* gyroscope data readout */
static int cwgd_read_raw(struct i2c_cwgd_sensor *sensor, int num)
{
	int err = -1;
	u8 gyro_out[6 * (num ? (num - 1) : 1)];
#if USE_FIFO
	u8 reg_buf;
	err = cwgd_i2c_read(sensor, AXISDATA_REG, &reg_buf, 1);
	if (err != 1)
		return (err < 0) ? err : -EIO;
#endif
	err = cwgd_i2c_read(sensor, OUT_Y_L, gyro_out, 6);
	if (err != 6)
		return (err < 0) ? err : -EIO;

	sensor->data[1] =
	    (s16) (((gyro_out[1]) << 8) | gyro_out[0]) - sensor->bias[1];
	sensor->data[2] =
	    (s16) (((gyro_out[3]) << 8) | gyro_out[2]) - sensor->bias[2];
	sensor->data[0] =
	    (s16) (((gyro_out[5]) << 8) | gyro_out[4]) - sensor->bias[0];

	return 0;
}

/**
 * Returns the number of data in the fifo on success, negative value on failure.
 */
static int cwgd_report_values(struct i2c_cwgd_sensor *sensor)
{
	int aligned_data[3];
	int res;
	int status = 0;

	while ((status = cwgd_is_data_ready(sensor)) != DATA_STATUS_READY)
		if (status == DATA_STATUS_OVERRUN)
			cwgd_read_raw(sensor, sensor->entries);

	res =
	    cwgd_read_raw(sensor, sensor->entries + sensor->discard_next_event);
	if (res < 0) {
		pr_err("%s(): get_gyroscope_data failed\n", __func__);
		return res;
	}

	res = cwgd_read_fifo_status(sensor);
	sensor->discard_next_event = !res;

	if (res >= 31 - sensor->entries)
		return cwgd_restart_fifo(sensor);

	if (cwgd_align_axis(sensor, aligned_data)) {
		pr_err("%s(): axis alignment failed.\n", __func__);
		return -1;
	}

	input_report_abs(sensor->input, ABS_X, aligned_data[0]);
	input_report_abs(sensor->input, ABS_Y, aligned_data[1]);
	input_report_abs(sensor->input, ABS_Z, aligned_data[2]);
	input_sync(sensor->input);

	return res;
}

static void cwgd_device_power_off(struct i2c_cwgd_sensor *sensor)
{
	int err;
	u8 val = CWGD_PM_OFF;
	err = cwgd_i2c_write(sensor, CTRL_REG1, &val, 1);
	if (err < 0)
		pr_err("soft power off failed\n");

	/* Turn off interrupt */
	val = 0x00;
	err = cwgd_i2c_write(sensor, CTRL_REG3, &val, 1);
	if (err < 0)
		pr_err("soft power off failed\n");
}

static int cwgd_device_power_on(struct i2c_cwgd_sensor *sensor)
{
	return cwgd_i2c_write(sensor, CTRL_REG1, sensor->ctrl_regs,
			      ARRAY_SIZE(sensor->ctrl_regs));
}

static int cwgd_enable(struct i2c_cwgd_sensor *sensor)
{
	int err;

	if (!atomic_cmpxchg(&sensor->enabled, 0, 1)) {
		err = cwgd_device_power_on(sensor);
		if (err < 0) {
			atomic_set(&sensor->enabled, 0);
			return err;
		}

		err = cwgd_restart_fifo(sensor);
		if (err < 0)
			goto exit_turn_off;

		if (sensor->use_interrupt)
			enable_irq(sensor->client->irq);
		else
			hrtimer_start(&sensor->timer,
				      sensor->polling_delay, HRTIMER_MODE_REL);
	}

	return 0;

exit_turn_off:
	cwgd_device_power_off(sensor);
	return err;
}

static int cwgd_disable(struct i2c_cwgd_sensor *sensor)
{
	if (atomic_cmpxchg(&sensor->enabled, 1, 0)) {
		if (sensor->use_interrupt)
			disable_irq(sensor->client->irq);
		else {
			hrtimer_cancel(&sensor->timer);
			cancel_work_sync(&sensor->work);
		}
		cwgd_device_power_off(sensor);
	}

	return 0;
}

static int cwgd_self_test(struct i2c_cwgd_sensor *sensor, u8 mode)
{
	int err = -1;
	int res = -1;
	u8 data = 0;

	res = cwgd_i2c_read(gyro, CTRL_REG4, &data, 1);
	if (res >= 0)
		data = data & 0xF9;
	else
		return res;

	data = mode | data;
	err = cwgd_i2c_write(gyro, CTRL_REG4, &data, 1);

	return err;
}

static int active_set(struct device *dev,
		      struct device_attribute *attr,
		      const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	struct i2c_cwgd_sensor *sensor =
	    (struct i2c_cwgd_sensor *)i2c_get_clientdata(client);
	int enabled = (strcmp(buf, "1\n") == 0) ? 1 : 0;

	if (enabled)
		cwgd_enable(sensor);
	else
		cwgd_disable(sensor);

	return count;
}

static int active_show(struct device *dev,
		       struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	struct i2c_cwgd_sensor *sensor =
	    (struct i2c_cwgd_sensor *)i2c_get_clientdata(client);

	if (atomic_read(&sensor->enabled))
		return sprintf(buf, "1\n");
	else
		return sprintf(buf, "0\n");
}

static int interval_show(struct device *dev,
			 struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	struct i2c_cwgd_sensor *sensor =
	    (struct i2c_cwgd_sensor *)i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", sensor->sample_interval);
}

static int interval_set(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	struct i2c_cwgd_sensor *sensor =
	    (struct i2c_cwgd_sensor *)i2c_get_clientdata(client);
	long val = 0;
	int res;

	res = strict_strtol(buf, 10, &val);
	if (res < 0)
		return res;

	if (val < MIN_INTERVAL_MS)
		val = MIN_INTERVAL_MS;
	sensor->sample_interval = val;
	if (cwgd_set_delay(sensor, sensor->sample_interval) < 0)
		pr_err("error in set bandwidth function\n");

	return count;
}

static ssize_t wake_set(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	return count;
}

static int data_show(struct device *dev,
		     struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	struct i2c_cwgd_sensor *sensor =
	    (struct i2c_cwgd_sensor *)i2c_get_clientdata(client);

	if (cwgd_read_raw(sensor, 1))
		pr_err("Cannot read raw data\n");

	return sprintf(buf, "%d %d %d\n", sensor->data[0], sensor->data[1],
		       sensor->data[2]);
}

static int status_show(struct device *dev,
		       struct device_attribute *attr, char *buf)
{
	return 0;
}

static DEVICE_ATTR(active, S_IRUGO | S_IWUGO, active_show, active_set);
static DEVICE_ATTR(interval, S_IRUGO | S_IWUGO, interval_show, interval_set);
static DEVICE_ATTR(data, S_IRUGO, data_show, NULL);
static DEVICE_ATTR(wake, S_IRUGO | S_IWUGO, NULL, wake_set);
static DEVICE_ATTR(status, S_IRUGO | S_IWUGO, status_show, NULL);

static struct attribute *sysfs_attributes[] = {
	&dev_attr_status.attr,
	&dev_attr_interval.attr,
	&dev_attr_data.attr,
	&dev_attr_active.attr,
	&dev_attr_wake.attr,
	NULL
};

static struct attribute_group sysfs_attribute_group = {
	.attrs = sysfs_attributes
};

#if USE_MISC_DEVICE
static int cwgd_misc_open(struct inode *inode, struct file *file)
{
	int err;
	err = nonseekable_open(inode, file);
	if (err < 0)
		return err;

	file->private_data = gyro;

	return 0;
}

static int cwgd_ioctl(struct inode *inode, struct file *file,
		      unsigned int cmd, unsigned long arg)
{
	int err = 0;
	unsigned char data[6];
	int bias[3];
	int interval;
	int enabled = 0;
	int mode = 0;
	u8 temperature = 0;
	struct i2c_cwgd_sensor *sensor =
	    (struct i2c_cwgd_sensor *)file->private_data;

	if (!sensor || sensor->client == NULL) {
		pr_err("CWGD I2C driver not installed\n");
		return -EFAULT;
	}

	switch (cmd) {
	case CWGD_SET_RANGE:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0)
			return -EFAULT;
		err = cwgd_set_range(*data);
		if (err < 0)
			pr_err("error in set range function\n");
		return err;

	case CWGD_SET_ENABLE:
		if (copy_from_user(data, (unsigned char *)arg, 1))
			return -EFAULT;

		if ((data[0] != CWGD_ENABLED) && (data[0] != CWGD_DISABLED))
			return -EINVAL;
		if (data[0] == CWGD_ENABLED)
			err = cwgd_enable(gyro);
		else
			err = cwgd_disable(gyro);
		return err;
		break;

	case CWGD_GET_ENABLE:
		enabled = atomic_read(&sensor->enabled);
		if (copy_to_user((int *)arg, &enabled, sizeof(enabled)) != 0) {
			pr_err("copy_to error\n");
			return -EFAULT;
		}
		return err;
		break;

	case CWGD_SET_DELAY:
		if (copy_from_user(&interval, (unsigned char *)arg,
				   sizeof(interval)) != 0) {
			pr_err("copy_from_user error\n");
			return -EFAULT;
		}

		if (interval < 0)
			return -EINVAL;

		err = cwgd_set_delay(gyro, interval);
		if (err < 0)
			pr_err("error in set bandwidth function\n");
		return err;

	case CWGD_READ_GYRO_VALUES:
		err = cwgd_read_raw(gyro, 1);
		if (copy_to_user((int *)arg,
				 (int *)gyro->data, sizeof(gyro->data)) != 0) {
			pr_err("copy_to error\n");
			return -EFAULT;
		}
		return err;
		break;

	case CWGD_SELF_TEST:
		if (copy_from_user(&mode, (unsigned char *)arg,
				   sizeof(mode)) != 0) {
			pr_err("copy_from_user error\n");
			return -EFAULT;
		}
		return cwgd_self_test(gyro, mode);

	case CWGD_GET_TEMPERATURE:
		temperature = (u8) cwgd_read_register(gyro, OUT_TEMP_REG);
		if (copy_to_user((u8 *) arg,
				 &temperature, sizeof(temperature)) != 0) {
			pr_err("copy_to error\n");
			return -EFAULT;
		}
		return err;

	case CWGD_SET_BIAS:
		if (copy_from_user(bias, (int *)arg, sizeof(bias)) != 0) {
			pr_err("copy_from_user error\n");
			return -EFAULT;
		}
		memcpy(gyro->bias, bias, sizeof(gyro->bias));
		return err;

	default:
		return 0;
	}
}

static const struct file_operations cwgd_misc_fops = {
	.owner = THIS_MODULE,
	.open = cwgd_misc_open,
	.ioctl = cwgd_ioctl,
};

static struct miscdevice cwgd_misc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "cwgd",
	.fops = &cwgd_misc_fops,
};
#endif

static enum hrtimer_restart cwgd_timer_func(struct hrtimer *timer)
{
	struct i2c_cwgd_sensor *sensor =
	    container_of(timer, struct i2c_cwgd_sensor, timer);
	queue_work(sensor->cwgd_wq, &sensor->work);
	return HRTIMER_NORESTART;
}

static void cwgd_work_func(struct work_struct *work)
{
	struct i2c_cwgd_sensor *sensor =
	    container_of(work, struct i2c_cwgd_sensor, work);
	int res;
	s64 delay_ns;

#if USE_FIFO
	/**
	 * Read out all the data in the FIFO
	 */
	do {
		res = cwgd_read_fifo_status(sensor);
		if (res < 0)
			return;

		if (res < sensor->entries) {
			pr_warning("%s: fifo entries %d < entries wanted %d\n",
				   __func__, res, sensor->entries);
		} else {
			res = cwgd_report_values(sensor);
			if (res < 0)
				return;
		}

		delay_ns = sensor->entries + 1 - res;
		if (delay_ns < 0)
			delay_ns = 0;
		else
			delay_ns *= sensor->time_to_read;
		sensor->polling_delay = ns_to_ktime(delay_ns);
	} while (!delay_ns);
	hrtimer_start(&sensor->timer, sensor->polling_delay, HRTIMER_MODE_REL);
#else
	delay_ns = ktime_to_ns(ktime_get());
	res = cwgd_report_values(sensor);
	if (res < 0)
		return;
	delay_ns = ktime_to_ns(ktime_get()) - delay_ns;
	delay_ns = ktime_to_ns(sensor->polling_delay) - delay_ns;
	if (delay_ns <= 0)
		delay_ns = 1;

	hrtimer_start(&sensor->timer, ns_to_ktime(delay_ns), HRTIMER_MODE_REL);
#endif
}

static irqreturn_t cwgd_interrupt_thread(int irq, void *data)
{
	int res;
	struct i2c_cwgd_sensor *sensor = data;
	pr_debug("cwgd irq\n");
	res = cwgd_report_values(sensor);
	if (res < 0)
		pr_err("%s: failed to report gyro values\n", __func__);

	return IRQ_HANDLED;
}

static int cwgd_input_init(struct i2c_cwgd_sensor *sensor)
{
	int err = -1;

	sensor->input = input_allocate_device();
	if (!sensor->input) {
		err = -ENOMEM;
		pr_err("input device allocate failed\n");
		return err;
	}

	sensor->input->name = DEV_NAME;
	sensor->input->id.bustype = BUS_I2C;
	sensor->input->dev.parent = &sensor->client->dev;

	set_bit(EV_ABS, sensor->input->evbit);
	input_set_abs_params(sensor->input, ABS_X, -DPS_MAX, DPS_MAX, 0, 0);
	input_set_abs_params(sensor->input, ABS_Y, -DPS_MAX, DPS_MAX, 0, 0);
	input_set_abs_params(sensor->input, ABS_Z, -DPS_MAX, DPS_MAX, 0, 0);

	err = input_register_device(sensor->input);
	if (err) {
		pr_err("unable to register input polled device %s\n",
		       sensor->input->name);
		input_free_device(sensor->input);
		sensor->input = NULL;
		return err;
	}

	return 0;
}

static int cwgd_timer_init(struct i2c_cwgd_sensor *sensor)
{
	u64 delay_ns;
	hrtimer_init(&sensor->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	sensor->polling_delay = ns_to_ktime(200 * NSEC_PER_MSEC);
	sensor->time_to_read = 10000000LL;
	delay_ns = ktime_to_ns(sensor->polling_delay);
	do_div(delay_ns, sensor->time_to_read);
	sensor->entries = delay_ns;
	sensor->timer.function = cwgd_timer_func;

	return 0;
}

static int cwgd_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct i2c_cwgd_sensor *sensor = i2c_get_clientdata(client);
	if (atomic_read(&sensor->enabled)) {
		if (sensor->use_interrupt)
			disable_irq(sensor->client->irq);
		else {
			hrtimer_cancel(&sensor->timer);
			cancel_work_sync(&sensor->work);
		}
		cwgd_device_power_off(sensor);
	}
	return 0;
}

static int cwgd_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct i2c_cwgd_sensor *sensor = i2c_get_clientdata(client);
	if (atomic_read(&sensor->enabled)) {
		int err = cwgd_device_power_on(sensor);
		if (err < 0) {
			atomic_set(&sensor->enabled, 0);
			return err;
		}

		cwgd_restart_fifo(sensor);

		if (sensor->use_interrupt)
			enable_irq(sensor->client->irq);
		else
			hrtimer_start(&sensor->timer,
				      sensor->polling_delay, HRTIMER_MODE_REL);
	}
	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void cwgd_early_suspend(struct early_suspend *h)
{
	struct i2c_cwgd_sensor *sensor =
	    container_of(h, struct i2c_cwgd_sensor, early_suspend);
	cwgd_suspend(&sensor->client->dev);

}

static void cwgd_late_resume(struct early_suspend *h)
{
	struct i2c_cwgd_sensor *sensor =
	    container_of(h, struct i2c_cwgd_sensor, early_suspend);
	cwgd_resume(&sensor->client->dev);
}
#endif

static int cwgd_probe(struct i2c_client *client,
		      const struct i2c_device_id *devid)
{
	struct i2c_cwgd_sensor *sensor;
	int err = 0;
	int tempvalue;

	pr_info("%s: probe start.\n", DEV_NAME);
	if (client->dev.platform_data)
		if (((struct cwgd_platform_data *)(client->dev.
						   platform_data))->set_power)
			((struct cwgd_platform_data *)(client->
						       dev.platform_data))->set_power
			    (1);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("client not i2c capable:1\n");
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	sensor = kzalloc(sizeof(struct i2c_cwgd_sensor), GFP_KERNEL);
	if (sensor == NULL) {
		pr_err("failed to allocate memory for module data\n");
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}

	if (i2c_smbus_read_byte(client) < 0) {
		pr_err("i2c_smbus_read_byte error!!\n");
		goto exit_kfree;
	}

	sensor->sample_interval = MIN_INTERVAL_MS;
	sensor->entries = 1;
	sensor->client = client;
	mutex_init(&sensor->lock);
	atomic_set(&sensor->enabled, 0);
	i2c_set_clientdata(client, sensor);
	gyro = sensor;

	if (cwgd_check_chip(sensor)) {
		pr_err("chip id is incorrect.\n");
		goto exit_destroy_mutex;
	}

	pr_info("i2c Device detected!\n");

	/* read chip id */
	tempvalue = i2c_smbus_read_word_data(client, WHO_AM_I);
	if ((tempvalue & 0x00FF) == WHOAMI_CWGD) {
		pr_debug("I2C driver registered!\n");
	} else {
		sensor->client = NULL;
		pr_err("I2C driver not registered! Device unknown\n");
		goto exit_destroy_mutex;
	}

	memcpy(sensor->ctrl_regs, default_ctrl_regs, sizeof(default_ctrl_regs));
	if (sensor->client->irq > 0) {
		sensor->use_interrupt = true;

		/* Disable all interrupts */
		if (cwgd_i2c_write(sensor, CTRL_REG3, &sensor->ctrl_regs[2], 1))
			goto exit_destroy_mutex;
		sensor->ctrl_regs[2] = DATA_READY_INT;

		err = request_threaded_irq(sensor->client->irq, NULL,
					   cwgd_interrupt_thread,
					   IRQF_TRIGGER_RISING | IRQF_ONESHOT,
					   "cwgd", sensor);
		if (err < 0) {
			pr_err("request irq %d failed\n", sensor->client->irq);
			goto exit_destroy_mutex;
		}
		disable_irq(sensor->client->irq);
	} else {
		err = cwgd_timer_init(sensor);
		if (err < 0) {
			pr_err("%s(): Failed to create the timer\n", __func__);
			goto exit_destroy_mutex;
		}

		sensor->cwgd_wq = create_singlethread_workqueue("cwgd_wq");
		if (!sensor->cwgd_wq) {
			err = -ENOMEM;
			pr_err("%s: could not create workqueue\n", __func__);
			goto exit_destroy_mutex;
		}
		INIT_WORK(&sensor->work, cwgd_work_func);
	}

	err = cwgd_input_init(sensor);
	if (err < 0)
		goto exit_destroy_workqueue;

	err =
	    sysfs_create_group(&sensor->input->dev.kobj,
			       &sysfs_attribute_group);
	if (err)
		goto exit_free_input;

	atomic_set(&sensor->enabled, 0);

#if USE_MISC_DEVICE
	/* register a misc dev */
	err = misc_register(&cwgd_misc_device);
	if (err < 0) {
		pr_err("%s device register failed\n", DEV_NAME);
		goto out_unreg_miscdev;
	}
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
	sensor->early_suspend.level = EARLY_SUSPEND_LEVEL_STOP_DRAWING;
	sensor->early_suspend.suspend = cwgd_early_suspend;
	sensor->early_suspend.resume = cwgd_late_resume;
	register_early_suspend(&sensor->early_suspend);
#endif

	printk
	    ("%s probed: device created successfully: interruptible: %d irq=%d\n",
	     DEV_NAME, sensor->use_interrupt, sensor->client->irq);

	return 0;

#if USE_MISC_DEVICE
out_unreg_miscdev:
	misc_deregister(&cwgd_misc_device);
#endif
exit_free_input:
	input_free_device(sensor->input);
exit_destroy_workqueue:
	if (sensor->use_interrupt) {
		enable_irq(sensor->client->irq);
		free_irq(sensor->client->irq, sensor);
	} else {
		destroy_workqueue(sensor->cwgd_wq);
	}
exit_destroy_mutex:
	mutex_destroy(&sensor->lock);
exit_kfree:
	kfree(sensor);
exit_alloc_data_failed:
exit_check_functionality_failed:
	pr_err("%s: Driver Initialization failed\n", DEV_NAME);
	return err;
}

static int cwgd_remove(struct i2c_client *client)
{
	int err = 0;
	struct i2c_cwgd_sensor *sensor = i2c_get_clientdata(client);

#if USE_MISC_DEVICE
	misc_deregister(&cwgd_misc_device);
#endif

	if (!atomic_read(&sensor->enabled)) {
		sensor->ctrl_regs[0] = 0x00;
		err =
		    cwgd_i2c_write(sensor, CTRL_REG1, &sensor->ctrl_regs[1], 1);
	}

	if (sensor->use_interrupt) {
		if (!atomic_read(&sensor->enabled))
			enable_irq(sensor->client->irq);
		free_irq(sensor->client->irq, sensor);
	} else {
		hrtimer_cancel(&sensor->timer);
		cancel_work_sync(&sensor->work);
		destroy_workqueue(sensor->cwgd_wq);
	}

	input_unregister_device(sensor->input);
	input_free_device(sensor->input);
	mutex_destroy(&sensor->lock);

	kfree(sensor);
	return 0;
}

static const struct dev_pm_ops cwgd_pm_ops = {
	.suspend = cwgd_suspend,
	.resume = cwgd_resume
};

static const struct i2c_device_id cwgd_id[] = {
	{CWGD_I2C_NAME, 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, cwgd_id);

static struct i2c_driver cwgd_driver = {
	.class = I2C_CLASS_HWMON,
	.probe = cwgd_probe,
	.remove = __devexit_p(cwgd_remove),
	.id_table = cwgd_id,
	.driver = {
		   .owner = THIS_MODULE,
		   .name = DEV_NAME,
		   .pm = &cwgd_pm_ops},
};

static int __init cwgd_init(void)
{
	return i2c_add_driver(&cwgd_driver);
}

static void __exit cwgd_exit(void)
{
	i2c_del_driver(&cwgd_driver);
	return;
}

module_init(cwgd_init);
module_exit(cwgd_exit);

MODULE_DESCRIPTION("cwgd digital gyroscope misc driver");
MODULE_AUTHOR("CyWee Group Ltd.");
MODULE_LICENSE("GPL");
