/* cwmi.c - Implementation for CyWee 6-axis motion sensor
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
#include <linux/list.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/cwmi.h>
#include <linux/slab.h>

#define USE_VIRTUAL_ORIENTATION_SENSOR		1

#define ORI_NAME	"cwmi_orientation"
#define DEV_NAME	"cwmi"

#define DEFAULT_SAMPLE_INTERVAL		10
#define ACC_MAX						2000
#define MAG_MAX						4096

#define SENSOR_TYPE_ACC				1
#define SENSOR_TYPE_MAG				2
#define SENSOR_TYPE_ORI				3

#define WAIT_DATA_READY				0

#define MAX_RETRY					15

/*=============================================================================
 * Accelerometer Registers
 *=============================================================================*/
#define CTRL_REG1_A				0x20
#define CTRL_REG3_A				0x22
#define CTRL_REG4_A				0x23
#define CTRL_REG5_A				0x24
#define STATUS_REG_A			0x27
#define ACC_DATA_X_L			0x28
#define INT1_CFG_REG_A			0x30

#define CTRL_REG1_A_DEFAULT		0x07

/* Output Data Rate */
#define ODR_1_ACC					0x10
#define ODR_10_ACC					0x20
#define ODR_25_ACC					0x30
#define ODR_50_ACC					0x40
#define ODR_100_ACC					0x50
#define ODR_200_ACC					0x60
#define ODR_400_ACC					0x70

#define ODR_MASK_ACC				0xF0

/* Range */
#define ACC_RANGE_2G				0x00
#define ACC_RANGE_4G				0x10
#define ACC_RANGE_8G				0x20
#define ACC_RANGE_16G				0x30

/* Interrupt settings */
#define ACC_DATA_READY1_INT_ON_INT1	0x10
#define ACC_DATA_READY2_INT_ON_INT1		0x08

#define ENABLE_ALL_AXES_ACC			0x07
#define NORMAL_MODE_ACC				0x08
#define ENABLE_HIGH_RESOLUTION		0x08
#define XYZ_DATA_AVAILABLE_ACC		0x08

#define GSENSOR_1G_COUNT_DIFF		1000

#define READ_MULTIPLE_BYTES			0x80

/*=============================================================================
 * Magnetometer Registers
 *============================================================================= */
#define CRA_REG_M				0x00
#define CRB_REG_M				0x01
#define MR_REG_M				0x02
#define MAG_DATA_X_H			0x03
#define SR_REG_M				0x09
#define IRA_REG_M				0x0A
#define IRB_REG_M				0x0B
#define IRC_REG_M				0x0C

/* Output Data Rate */
#define ODR_0_75_MAG				0x00
#define ODR_1_5_MAG					0x04
#define ODR_3_0_MAG					0x08
#define ODR_7_5_MAG					0x0C
#define ODR_15_MAG					0x10
#define ODR_30_MAG					0x14
#define ODR_75_MAG					0x18
#define ODR_220_MAG					0x1C

#define ODR_MASK_MAG				0x1C

/* Range */
#define ODR_1_3_GAUSS				0x20
#define ODR_1_9_GAUSS				0x40
#define ODR_2_5_GAUSS				0x30
#define ODR_4_0_GAUSS				0x80
#define ODR_4_7_GAUSS				0xA0
#define ODR_5_6_GAUSS				0xC0
#define ODR_8_1_GAUSS				0xE0

#define MAG_ID_A				0x48
#define MAG_ID_B				0x34
#define MAG_ID_C				0x33

#define MAG_DATA_READY			0x01

#define MAG_1GAUSS_COUNT_DIFF_X		1100
#define MAG_1GAUSS_COUNT_DIFF_Y		1100
#define MAG_1GAUSS_COUNT_DIFF_Z		980

static const char acc_default_ctrl_regs[] = {
	ODR_100_ACC | ENABLE_ALL_AXES_ACC | NORMAL_MODE_ACC,
	0x00,
	0x00,
	ACC_RANGE_2G | ENABLE_HIGH_RESOLUTION,
	0x00,
	0x00
};

static const char mag_default_ctrl_regs[] = {
	ODR_75_MAG,
	ODR_1_3_GAUSS,
};

struct i2c_cwmi_sensor {
	struct i2c_client *client;
	struct input_dev *input;
	struct delayed_work input_work;
	struct list_head list;
	bool is_virtual;

	bool use_interrupt;
	u8 ctrl_regs[6];

	int type;

	unsigned int sample_interval;

	atomic_t enabled;
	struct mutex lock;
	int zero_offset[3];
	int sensitivity[3];
	int data[3];
};

static struct odr_struct {
	u32 delay_ns;
	unsigned int mask;
} acc_odr_table[] = {
	{
	2500000LL, ODR_400_ACC}, {
	5000000LL, ODR_200_ACC}, {
	10000000LL, ODR_100_ACC}, {
	20000000LL, ODR_50_ACC}, {
	40000000LL, ODR_25_ACC}, {
	100000000LL, ODR_10_ACC}, {
1000000000LL, ODR_1_ACC},}, mag_odr_table[] = {

	{
	4545455LL, ODR_220_MAG}, {
	13333333LL, ODR_75_MAG}, {
	33333333LL, ODR_30_MAG}, {
	66666667LL, ODR_15_MAG}, {
	133333333LL, ODR_7_5_MAG}, {
	333333333LL, ODR_3_0_MAG}, {
	666666667LL, ODR_1_5_MAG}, {
1333333333LL, ODR_0_75_MAG},};

LIST_HEAD(sensors);

static struct i2c_cwmi_sensor *cwmi_find_sensor(const char *name)
{
	struct i2c_cwmi_sensor *sensor;
	list_for_each_entry(sensor, &sensors, list)
	    if (strcmp(sensor->input->name, name) == 0)
		return sensor;

	return NULL;
}

static int cwmi_i2c_write(struct i2c_cwmi_sensor *sensor,
			  u8 reg_addr, u8 *data, u8 len)
{
	int dummy;
	int i;

	if (sensor->client == NULL)
		return -1;
	for (i = 0; i < len; i++) {
		dummy = i2c_smbus_write_byte_data(sensor->client,
						  reg_addr++, data[i]);
		if (dummy) {
			pr_err("%s(): i2c write error: %d\n", __func__, dummy);
			return dummy;
		}
	}
	return 0;
}

static int cwmi_i2c_read(struct i2c_cwmi_sensor *sensor,
			 u8 reg_addr, u8 *data, u8 len)
{
	if (sensor->client == NULL)
		return -1;

	return i2c_smbus_read_i2c_block_data(sensor->client, reg_addr, len,
					     data);
}

/**
 * Find the delay which is less than or equals to delay_ns, then return the
 * corresponding output data rate mask.
 */
static u8 cwmi_find_delay(struct i2c_cwmi_sensor *sensor, u64 * delay_ns)
{
	struct odr_struct *odr_table =
	    (sensor->type == SENSOR_TYPE_ACC) ? acc_odr_table : mag_odr_table;
	int size =
	    (sensor->type ==
	     SENSOR_TYPE_ACC) ? ARRAY_SIZE(acc_odr_table) :
	    ARRAY_SIZE(mag_odr_table);
	int i = size - 1;

	if (*delay_ns < odr_table[i].delay_ns)
		for (i = size - 1; i >= 0; i--)
			if (*delay_ns >= odr_table[i].delay_ns) {
				*delay_ns = odr_table[i].delay_ns;
				break;
			}
	if (i < 0)
		i = 0;

	pr_info("odr mask=0x%02X\n", odr_table[i].mask);

	return odr_table[i].mask;
}

static int cwmi_set_delay(struct i2c_cwmi_sensor *sensor, int delay)
{
	bool is_acc = (sensor->type == SENSOR_TYPE_ACC) ? true : false;
	u8 odr_mask = is_acc ? ODR_MASK_ACC : ODR_MASK_MAG;
	u64 delay_ns = delay * NSEC_PER_MSEC;
	u8 value = 0;
	u8 ctrl_reg = is_acc ? CTRL_REG1_A : CRA_REG_M;
	int res;

	mutex_lock(&sensor->lock);

	if (atomic_read(&sensor->enabled)) {
		if (sensor->use_interrupt)
			disable_irq(sensor->client->irq);
		else
			cancel_delayed_work_sync(&sensor->input_work);
	}

	value = (sensor->ctrl_regs[0] & ~odr_mask);
	value |= cwmi_find_delay(sensor, &delay_ns);
	sensor->ctrl_regs[0] = value;
	res = cwmi_i2c_write(sensor, ctrl_reg, &sensor->ctrl_regs[0], 1);

	if (atomic_read(&sensor->enabled) && !res) {

		if (sensor->use_interrupt) {
			enable_irq(sensor->client->irq);

			if (sensor->type == SENSOR_TYPE_ACC) {
				/* Restart interrupt generator */
				value = 0x00;
				if (cwmi_i2c_write
				    (sensor, CTRL_REG3_A, &value, 1))
					return -1;
				if (cwmi_i2c_write
				    (sensor, CTRL_REG3_A, &sensor->ctrl_regs[2],
				     1))
					return -1;
			}
		} else {
			sensor->sample_interval =
			    ((u32) delay_ns / NSEC_PER_MSEC);
			schedule_delayed_work(&sensor->input_work,
					      msecs_to_jiffies
					      (sensor->sample_interval));
		}
	}

	mutex_unlock(&sensor->lock);

	return res;
}

static u8 cwmi_read_register(struct i2c_cwmi_sensor *sensor, uint8_t reg)
{
	u8 value = 0;

	if (cwmi_i2c_read(sensor, reg, &value, 1) < 0) {
		pr_err("Read 0x%02X failed\n", reg);
		return 0;
	}

	return value;
}

#if USE_VIRTUAL_ORIENTATION_SENSOR
static int cwmi_is_sensor_enabled(const char *name)
{
	struct i2c_cwmi_sensor *sensor = cwmi_find_sensor(name);

	if (sensor)
		return atomic_read(&sensor->enabled);

	pr_err("%s doesn't exist\n", name);
	return 0;
}

static int cwmi_ori_report_control(struct i2c_cwmi_sensor *sensor)
{
	int value = (cwmi_is_sensor_enabled(ORI_NAME) << 16) |
	    (cwmi_is_sensor_enabled(I2C_MAG_NAME) << 17) |
	    sensor->sample_interval;
	struct i2c_cwmi_sensor *ori_sensor = cwmi_find_sensor(ORI_NAME);

	pr_debug("%s(): value = 0x%08X\n", __func__, value);

	if (ori_sensor) {
		input_report_abs(ori_sensor->input, ABS_THROTTLE, value);
		input_sync(ori_sensor->input);
	} else {
		pr_err("Cannot find orientation sensor\n");
		return -ENODEV;
	}
	return 0;
}

static int cwmi_ori_enable(struct i2c_cwmi_sensor *sensor, int enabled)
{
	if (enabled) {
		if (!atomic_cmpxchg(&sensor->enabled, 0, 1)) {
			if (cwmi_ori_report_control(sensor)) {
				atomic_cmpxchg(&sensor->enabled, 1, 0);
				pr_err
				    ("CWMI: orientation sensor power on failed.\n");
				return -1;
			}
		}
	} else {
		if (atomic_cmpxchg(&sensor->enabled, 1, 0))
			cwmi_ori_report_control(sensor);
	}

	pr_debug("cwmi orientation %s\n", enabled ? "enabled" : "disabled");

	return 0;
}
#else /* USE_VIRTUAL_ORIENTATION_SENSOR */
static int cwmi_ori_report_control(struct i2c_cwmi_sensor *sensor)
{
	return 0;
}

static int cwmi_ori_enable(struct i2c_cwmi_sensor *sensor, int enabled)
{
	return 0;
}
#endif /* USE_VIRTUAL_ORIENTATION_SENSOR */

static int cwmi_mag_set_op_mode(struct i2c_cwmi_sensor *sensor, u8 mode)
{
	sensor->ctrl_regs[2] = mode;
	if (cwmi_i2c_write(sensor, MR_REG_M, &sensor->ctrl_regs[2], 1)) {
		pr_err
		    ("Failed to set magnetometer operating mode: mode=0x%02X\n",
		     mode);
		return -1;
	}

	return 0;
}

static int cwmi_mag_data_ready(struct i2c_cwmi_sensor *sensor)
{
	return cwmi_read_register(sensor, SR_REG_M) & MAG_DATA_READY;
}

static int cwmi_mag_read(struct i2c_cwmi_sensor *sensor)
{
	u8 buf[6];
	int hw_d[3] = { 0 };

	if (cwmi_i2c_read(sensor, MAG_DATA_X_H, buf, sizeof(buf)) < 0)
		return -1;

	hw_d[0] = (int)(buf[0] << 8 | buf[1]);
	hw_d[1] = (int)(buf[4] << 8 | buf[5]);
	hw_d[2] = (int)(buf[2] << 8 | buf[3]);

	hw_d[0] = (hw_d[0] & 0x8000) ? (hw_d[0] | 0xFFFF0000) : (hw_d[0]);
	hw_d[1] = (hw_d[1] & 0x8000) ? (hw_d[1] | 0xFFFF0000) : (hw_d[1]);
	hw_d[2] = (hw_d[2] & 0x8000) ? (hw_d[2] | 0xFFFF0000) : (hw_d[2]);

	/* Unit: mGauss */
	sensor->data[0] = hw_d[0] * 1000 / MAG_1GAUSS_COUNT_DIFF_X;
	sensor->data[1] = hw_d[1] * 1000 / MAG_1GAUSS_COUNT_DIFF_Y;
	sensor->data[2] = hw_d[2] * 1000 / MAG_1GAUSS_COUNT_DIFF_Z;

	return 0;
}

static int cwmi_mag_power_off(struct i2c_cwmi_sensor *sensor)
{
	if (cwmi_mag_set_op_mode(sensor, MAG_MODE_SLEEP))
		return -1;

	return 0;
}

static int cwmi_mag_power_on(struct i2c_cwmi_sensor *sensor)
{
	return cwmi_i2c_write(sensor, CRA_REG_M, sensor->ctrl_regs, 3);
}

static int cwmi_mag_enable(struct i2c_cwmi_sensor *sensor, int enabled)
{
	if (enabled) {
		if (!atomic_cmpxchg(&sensor->enabled, 0, 1)) {
			if (cwmi_mag_power_on(sensor)) {
				atomic_cmpxchg(&sensor->enabled, 1, 0);
				pr_err("CWMI: magnetometer power on failed.\n");
				return -1;
			}

			if (cwmi_ori_report_control(sensor)) {
				atomic_cmpxchg(&sensor->enabled, 1, 0);
				pr_err("%s(): Cannot send enabled report\n",
				       __func__);
				return -1;
			}

			if (sensor->use_interrupt)
				enable_irq(sensor->client->irq);
			else
				schedule_delayed_work(&sensor->input_work,
						      msecs_to_jiffies
						      (sensor->sample_interval));

			pr_debug("cwmi magnetometer enabled\n");
		}

	} else {
		if (atomic_cmpxchg(&sensor->enabled, 1, 0)) {
			if (sensor->use_interrupt)
				disable_irq(sensor->client->irq);
			else
				cancel_delayed_work_sync(&sensor->input_work);
			cwmi_ori_report_control(sensor);
			cwmi_mag_power_off(sensor);
			pr_debug("cwmi magnetometer disabled\n");
		}
	}

	return 0;
}

#if WAIT_DATA_READY
static int cwmi_acc_data_ready(struct i2c_cwmi_sensor *sensor)
{
	u8 status = cwmi_read_register(sensor, STATUS_REG_A);
	return status & XYZ_DATA_AVAILABLE_ACC;
}
#else
static int cwmi_acc_data_ready(struct i2c_cwmi_sensor *sensor)
{
	return 1;
}
#endif

static int cwmi_acc_read(struct i2c_cwmi_sensor *sensor)
{
	u8 buf[6];

	if (cwmi_i2c_read
	    (sensor, ACC_DATA_X_L | READ_MULTIPLE_BYTES, buf, sizeof(buf)) < 0)
		return -1;

	sensor->data[0] = ((short)(buf[1] << 8 | buf[0])) >> 4;
	sensor->data[1] = ((short)(buf[3] << 8 | buf[2])) >> 4;
	sensor->data[2] = ((short)(buf[5] << 8 | buf[4])) >> 4;

	return 0;
}

static int cwmi_acc_power_off(struct i2c_cwmi_sensor *sensor)
{
	u8 value;

	/* Turn off interrupts */
	value = 0x00;
	if (cwmi_i2c_write(sensor, CTRL_REG3_A, &value, 1))
		return -1;

	/* Power down */
	value = CTRL_REG1_A_DEFAULT;
	if (cwmi_i2c_write(sensor, CTRL_REG1_A, &value, 1))
		return -1;

	return 0;
}

static int cwmi_acc_power_on(struct i2c_cwmi_sensor *sensor)
{
	return cwmi_i2c_write(sensor, CTRL_REG1_A, sensor->ctrl_regs,
			      ARRAY_SIZE(sensor->ctrl_regs));
}

static int cwmi_acc_enable(struct i2c_cwmi_sensor *sensor, int enabled)
{
	if (enabled) {
		if (!atomic_cmpxchg(&sensor->enabled, 0, 1)) {
			if (cwmi_acc_power_on(sensor)) {
				atomic_cmpxchg(&sensor->enabled, 1, 0);
				pr_err
				    ("CWMI: accelerometer power on failed.\n");
				return -1;
			}

			if (sensor->use_interrupt)
				enable_irq(sensor->client->irq);
			else
				schedule_delayed_work(&sensor->input_work,
						      msecs_to_jiffies
						      (sensor->sample_interval));

			pr_debug("cwmi accelerometer enabled\n");
		}

	} else {
		if (atomic_cmpxchg(&sensor->enabled, 1, 0)) {
			if (sensor->use_interrupt)
				disable_irq(sensor->client->irq);
			else
				cancel_delayed_work_sync(&sensor->input_work);
			cwmi_acc_power_off(sensor);

			pr_debug("cwmi accelerometer disabled\n");
		}
	}

	return 0;
}

static int active_set(struct device *dev,
		      struct device_attribute *attr,
		      const char *buf, size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	struct i2c_cwmi_sensor *sensor =
	    (struct i2c_cwmi_sensor *)input_get_drvdata(input);
	int enabled = (strcmp(buf, "1\n") == 0) ? 1 : 0;

	pr_debug("%s(): %s\n", __func__, input->name);

	if (strcmp(input->name, I2C_ACC_NAME) == 0)
		cwmi_acc_enable(sensor, enabled);
	else if (strcmp(input->name, ORI_NAME) == 0)
		cwmi_ori_enable(sensor, enabled);
	else
		cwmi_mag_enable(sensor, enabled);

	return count;
}

static int active_show(struct device *dev,
		       struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct i2c_cwmi_sensor *sensor =
	    (struct i2c_cwmi_sensor *)input_get_drvdata(input);

	if (atomic_read(&sensor->enabled))
		return sprintf(buf, "1\n");
	else
		return sprintf(buf, "0\n");
}

static int interval_show(struct device *dev,
			 struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct i2c_cwmi_sensor *sensor =
	    (struct i2c_cwmi_sensor *)input_get_drvdata(input);

	return sprintf(buf, "%d\n", sensor->sample_interval);
}

static int interval_set(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	struct i2c_cwmi_sensor *sensor =
	    (struct i2c_cwmi_sensor *)input_get_drvdata(input);
	unsigned long val = 0;
	int res = 0;

	res = strict_strtoul(buf, 10, &val);
	if (res < 0)
		return res;

	pr_info("Sensor [%s] interval is %ld\n", input->name, val);

	if (!sensor->is_virtual)
		cwmi_set_delay(sensor, (int)val);

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
	struct input_dev *input = to_input_dev(dev);
	struct i2c_cwmi_sensor *sensor =
	    (struct i2c_cwmi_sensor *)input_get_drvdata(input);

	if (strcmp(input->name, I2C_ACC_NAME) == 0)
		cwmi_acc_read(sensor);
	else
		cwmi_mag_read(sensor);

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

static int cwmi_align_axis(struct i2c_cwmi_sensor *sensor, int *aligned_data)
{
	int i = 0;
	int j = 0;
	int sum = 0;
	int *axes = NULL;

	if (!(struct cwmi_platform_data *)sensor->client->dev.platform_data) {
		aligned_data[0] = sensor->data[0];
		aligned_data[1] = sensor->data[1];
		aligned_data[2] = sensor->data[2];
		return 0;
	}

	axes =
	    ((struct cwmi_platform_data *)sensor->client->dev.
	     platform_data)->axes;
	for (i = 0; i < 3; i++) {
		sum = 0;
		for (j = 0; j < 3; j++)
			sum += sensor->data[j] * axes[i * 3 + j];
		aligned_data[i] = sum;
	}

	return 0;
}

static void cwmi_mag_report_values(struct i2c_cwmi_sensor *sensor)
{
	int aligned_data[3];
	if (cwmi_align_axis(sensor, aligned_data))
		pr_err("%s(): axis alignment failed.\n", __func__);

	input_report_abs(sensor->input, ABS_HAT0X, aligned_data[0]);
	input_report_abs(sensor->input, ABS_HAT0Y, aligned_data[1]);
	input_report_abs(sensor->input, ABS_HAT1X, aligned_data[2]);
	input_sync(sensor->input);

	pr_debug("mx=%04d, my=%04d, mz=%04d\n",
		 aligned_data[0], aligned_data[1], aligned_data[2]);
}

static void cwmi_acc_report_values(struct i2c_cwmi_sensor *sensor)
{
	int aligned_data[3];
	if (cwmi_align_axis(sensor, aligned_data))
		pr_err("%s(): axis alignment failed.\n", __func__);

	input_report_abs(sensor->input, ABS_X, aligned_data[0]);
	input_report_abs(sensor->input, ABS_Y, aligned_data[1]);
	input_report_abs(sensor->input, ABS_Z, aligned_data[2]);
	input_sync(sensor->input);

	pr_debug("ax=%04d, ay=%04d, az=%04d\n",
		 aligned_data[0], aligned_data[1], aligned_data[2]);
}

static void cwmi_report_values(struct i2c_cwmi_sensor *sensor)
{
	if (sensor->type == SENSOR_TYPE_ACC) {
		if (!sensor->use_interrupt) {
			int retry = 0;

			while (!cwmi_acc_data_ready(sensor)
			       && retry++ <= MAX_RETRY)
				udelay(sensor->sample_interval * 1000);

			if (retry >= MAX_RETRY) {
				pr_err("exhausted retries\n");
				return;
			}
		}
		if (!cwmi_acc_read(sensor))
			cwmi_acc_report_values(sensor);
		else
			pr_err("Read acc value failed\n");
	} else {
		if (!sensor->use_interrupt) {
			int retry = MAX_RETRY;

			if (cwmi_mag_set_op_mode(sensor, MAG_MODE_SINGLE))
				return;

			while (!cwmi_mag_data_ready(sensor) && retry-- > 0)
				udelay(sensor->sample_interval * 1000);

			if (retry <= 0) {
				pr_err("exhausted retries\n");
				return;
			}
		}

		if (!cwmi_mag_read(sensor))
			cwmi_mag_report_values(sensor);
		else
			pr_err("Read mag values failed\n");
	}
}

static void cwmi_work_func(struct work_struct *work)
{
	struct i2c_cwmi_sensor *sensor =
	    container_of((struct delayed_work *)work,
			 struct i2c_cwmi_sensor,
			 input_work);

	cwmi_report_values(sensor);

	schedule_delayed_work(&sensor->input_work,
			      msecs_to_jiffies(sensor->sample_interval));
}

static irqreturn_t cwmi_interrupt_thread(int irq, void *data)
{
	struct i2c_cwmi_sensor *sensor = data;
	pr_debug("interrupt cwmi: %d\n", sensor->type);
	cwmi_report_values(sensor);

	return IRQ_HANDLED;
}

static int cwmi_acc_input_init(struct i2c_cwmi_sensor *sensor)
{
	int err = 0;

	sensor->input = input_allocate_device();
	if (!sensor->input) {
		err = -ENOMEM;
		dev_err(&sensor->client->dev,
			"Failed to allocate input device\n");
		goto failed;
	}

	sensor->input->name = I2C_ACC_NAME;
	sensor->input->id.bustype = BUS_I2C;
	input_set_drvdata(sensor->input, sensor);

	set_bit(EV_ABS, sensor->input->evbit);
	input_set_abs_params(sensor->input, ABS_X, -ACC_MAX, ACC_MAX, 0, 0);
	input_set_abs_params(sensor->input, ABS_Y, -ACC_MAX, ACC_MAX, 0, 0);
	input_set_abs_params(sensor->input, ABS_Z, -ACC_MAX, ACC_MAX, 0, 0);

	err = input_register_device(sensor->input);
	if (err) {
		pr_err("Failed to register input device\n");
		input_free_device(sensor->input);
		sensor->input = NULL;
		goto failed;
	}

	return 0;

failed:
	return err;
}

static int cwmi_mag_input_init(struct i2c_cwmi_sensor *sensor)
{
	int err = 0;

	sensor->input = input_allocate_device();
	if (!sensor->input) {
		err = -ENOMEM;
		dev_err(&sensor->client->dev,
			"Failed to allocate input device\n");
		goto failed;
	}

	sensor->input->name = I2C_MAG_NAME;
	sensor->input->id.bustype = BUS_I2C;
	input_set_drvdata(sensor->input, sensor);

	set_bit(EV_ABS, sensor->input->evbit);
	input_set_abs_params(sensor->input, ABS_HAT0X, -MAG_MAX, MAG_MAX, 0, 0);
	input_set_abs_params(sensor->input, ABS_HAT0Y, -MAG_MAX, MAG_MAX, 0, 0);
	input_set_abs_params(sensor->input, ABS_HAT1X, -MAG_MAX, MAG_MAX, 0, 0);
	input_set_abs_params(sensor->input, ABS_X, -MAG_MAX, MAG_MAX, 0, 0);
	input_set_abs_params(sensor->input, ABS_Y, -MAG_MAX, MAG_MAX, 0, 0);
	input_set_abs_params(sensor->input, ABS_Z, -MAG_MAX, MAG_MAX, 0, 0);
	input_set_abs_params(sensor->input, ABS_BRAKE, 0, 5, 0, 0);
	input_set_abs_params(sensor->input, ABS_MISC, 0, 5, 0, 0);

	err = input_register_device(sensor->input);
	if (err) {
		dev_err(&sensor->input->dev,
			"Failed to register input device\n");
		input_free_device(sensor->input);
		sensor->input = NULL;
		goto failed;
	}

	return 0;

failed:
	return err;
}

#if USE_VIRTUAL_ORIENTATION_SENSOR
static int cwmi_ori_input_init(struct i2c_cwmi_sensor *sensor)
{
	int err = 0;

	sensor->input = input_allocate_device();
	if (!sensor->input) {
		err = -ENOMEM;
		pr_err("Failed to allocate input device\n");
		goto failed;
	}

	sensor->input->name = ORI_NAME;
	sensor->input->id.bustype = BUS_VIRTUAL;
	input_set_drvdata(sensor->input, sensor);

	set_bit(EV_ABS, sensor->input->evbit);
	input_set_abs_params(sensor->input, ABS_X, -MAG_MAX, MAG_MAX, 0, 0);
	input_set_abs_params(sensor->input, ABS_Y, -MAG_MAX, MAG_MAX, 0, 0);
	input_set_abs_params(sensor->input, ABS_Z, -MAG_MAX, MAG_MAX, 0, 0);
	input_set_abs_params(sensor->input, ABS_THROTTLE, 0, 0x1ffff, 0, 0);

	err = input_register_device(sensor->input);
	if (err) {
		dev_err(&sensor->input->dev,
			"Failed to register input device for orientation sensor\n");
		input_free_device(sensor->input);
		sensor->input = NULL;
		goto failed;
	}

	return 0;

failed:
	return err;
}

static int cwmi_ori_sensor_init(void)
{
	int err = 0;
	struct i2c_cwmi_sensor *sensor;

	sensor = kzalloc(sizeof(struct i2c_cwmi_sensor), GFP_KERNEL);
	if (sensor == NULL) {
		pr_err("Failed to allocate memory\n");
		err = -ENOMEM;
		goto failed_alloc;
	}

	sensor->sample_interval = DEFAULT_SAMPLE_INTERVAL;
	atomic_set(&sensor->enabled, 0);
	mutex_init(&sensor->lock);
	sensor->is_virtual = true;
	sensor->type = SENSOR_TYPE_ORI;

	err = cwmi_ori_input_init(sensor);
	if (err)
		goto exit_free;

	/* Creates attributes */
	err =
	    sysfs_create_group(&sensor->input->dev.kobj,
			       &sysfs_attribute_group);
	if (err)
		goto exit_free_input;

	list_add(&sensor->list, &sensors);

	return 0;

exit_free_input:
	input_free_device(sensor->input);
exit_free:
	kfree(sensor);
failed_alloc:
	pr_err("Orientation sensor initialization failed\n");
	return err;
}
#else
static int cwmi_ori_sensor_init(void)
{
	return 0;
}

static int cwmi_ori_input_init(struct i2c_cwmi_sensor *sensor)
{
	return 0;
}
#endif

static int cwmi_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct i2c_cwmi_sensor *sensor;
	int err = 0;

	sensor = kzalloc(sizeof(struct i2c_cwmi_sensor), GFP_KERNEL);
	if (sensor == NULL) {
		pr_err("Failed to allocate memory\n");
		err = -ENOMEM;
		goto failed_alloc;
	}

	i2c_set_clientdata(client, sensor);
	sensor->client = client;
	sensor->sample_interval = DEFAULT_SAMPLE_INTERVAL;
	atomic_set(&sensor->enabled, 0);
	mutex_init(&sensor->lock);
	sensor->type = strcmp(client->name, I2C_ACC_NAME) ?
	    SENSOR_TYPE_MAG : SENSOR_TYPE_ACC;

	err = (sensor->type == SENSOR_TYPE_ACC) ? cwmi_acc_input_init(sensor) :
	    cwmi_mag_input_init(sensor);
	if (err)
		goto exit_free;

	if (sensor->type == SENSOR_TYPE_ACC)
		memcpy(sensor->ctrl_regs, acc_default_ctrl_regs,
		       sizeof(acc_default_ctrl_regs));
	else
		memcpy(sensor->ctrl_regs, mag_default_ctrl_regs,
		       sizeof(mag_default_ctrl_regs));

	if (sensor->client->irq > 0) {
		sensor->use_interrupt = true;
		if (sensor->type == SENSOR_TYPE_ACC) {
			/* Disable all interrupts */
			if (cwmi_i2c_write
			    (sensor, CTRL_REG3_A, &sensor->ctrl_regs[2], 1))
				goto exit_free_input;
			sensor->ctrl_regs[2] |= ACC_DATA_READY1_INT_ON_INT1;
		} else if (sensor->type == SENSOR_TYPE_MAG) {
			cwmi_mag_set_op_mode(sensor, MAG_MODE_CONTINUOUS);
		}
		err = request_threaded_irq(sensor->client->irq, NULL,
					   cwmi_interrupt_thread,
					   IRQF_TRIGGER_RISING | IRQF_ONESHOT,
					   client->name, sensor);
		if (err < 0) {
			pr_err("request irq %d failed\n", sensor->client->irq);
			goto exit_free_input;
		}
		disable_irq(sensor->client->irq);
	} else
		INIT_DELAYED_WORK(&sensor->input_work, cwmi_work_func);

	/* Creates attributes */
	err =
	    sysfs_create_group(&sensor->input->dev.kobj,
			       &sysfs_attribute_group);
	if (err)
		goto exit_free_irq;

	list_add(&sensor->list, &sensors);

	if (!cwmi_find_sensor(ORI_NAME))
		cwmi_ori_sensor_init();

	pr_info("%s i2c device created (Interruptible=%d).\n", client->name,
		sensor->use_interrupt);

	return 0;

exit_free_irq:
	if (sensor->use_interrupt) {
		enable_irq(sensor->client->irq);
		free_irq(sensor->client->irq, sensor);
	}
exit_free_input:
	input_free_device(sensor->input);
exit_free:
	mutex_destroy(&sensor->lock);
	kfree(sensor);
failed_alloc:
	pr_err("%s: Initialized driver failed\n", client->name);
	return err;
}

static int cwmi_remove(struct i2c_client *client)
{
	return 0;
}

static int cwmi_resume(struct i2c_client *client)
{
	return 0;
}

static int cwmi_suspend(struct i2c_client *client, pm_message_t mesg)
{
	return 0;
}

static const struct i2c_device_id cwmi_id[] = {
	{I2C_ACC_NAME, 0},
	{I2C_MAG_NAME, 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, cwmi_id);

static struct i2c_driver cwmi_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = DEV_NAME,
		   },
	.class = I2C_CLASS_HWMON,
	.probe = cwmi_probe,
	.remove = __devexit_p(cwmi_remove),
	.id_table = cwmi_id,
#ifdef CONFIG_PM
	.suspend = cwmi_suspend,
	.resume = cwmi_resume,
#endif

};

static int __init cwmi_init(void)
{
	return i2c_add_driver(&cwmi_driver);
}

static void __exit cwmi_exit(void)
{
	i2c_del_driver(&cwmi_driver);
	return;
}

module_init(cwmi_init);
module_exit(cwmi_exit);

MODULE_DESCRIPTION("CyWee 6-axis motion sensor");
MODULE_AUTHOR("Joe Wei <joewei@cywee.com>");
MODULE_LICENSE("GPL");
