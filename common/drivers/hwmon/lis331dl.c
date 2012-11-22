/*
 * drivers/hwmon/lis331dl.c
 *
 * Copyright 2010 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/workqueue.h>
#include <linux/input.h>
#include <linux/slab.h>

#define LIS331DLH_VERSION "1.0.0"

#define LIS331DLM_NAME "lis331dlm"
#define LIS331DLH_NAME "lis331dlh"
#define LIS331DL_NAME "lis331dl"

/* for debugging */
#define DEBUG 0
#define TRACE_FUNC() pr_debug(LIS331DLH_NAME ": <trace> %s()\n", __FUNCTION__)

/*
 * Default parameters
 */
#define LIS331DLH_DEFAULT_DELAY         100
#define LIS331DLH_MAX_DELAY             2000

/*
 * Registers
 */
#define LIS331DLH_WHO_AM_I              0x0f
#define LIS331DLH_CTRL_REG1             0x20
#define LIS331DLH_CTRL_REG2             0x21
#define LIS331DLH_CTRL_REG3             0x22
#define LIS331DLH_CTRL_REG4             0x23
#define LIS331DLH_CTRL_REG5             0x24
#define LIS331DLH_HP_FILTER_RESET       0x25
#define LIS331DLH_REFERENCE             0x26
#define LIS331DLH_STATUS_REG            0x27
#define LIS331DLH_OUT_X_L               0x28
#define LIS331DLH_OUT_X                 0x29
#define LIS331DLH_OUT_Y_L               0x2a
#define LIS331DLH_OUT_Y                 0x2b
#define LIS331DLH_OUT_Z_L               0x2c
#define LIS331DLH_OUT_Z                 0x2d

#define LIS331DLM_ID                    0x12
#define LIS331DLH_ID                    0x32
#define LIS331DL_ID                    0x3B

#define LIS331DL_CTRL_REG1			0x20
#define LIS331DL_CTRL_REG2			0x21
#define LIS331DL_FF_WU_CFG			0x30

#define LIS331DL_CR1_DR_400		0x80
#define LIS331DL_CR1_PM_NORMAL	0x40
#define LIS331DL_CR1_Z_ENABLE          0x04
#define LIS331DL_CR1_Y_ENABLE          0x02
#define LIS331DL_CR1_X_ENABLE          0x01
#define LIS331DL_CR1_XYZ_ENABLE        0x07
#define LIS331DL_CR2_BDU		0x40
#define LIS331DL_CR2_IEN		0x08

#define LIS331DLH_CR1_PM_NORMAL         0x20
#define LIS331DLH_CR1_DR_50             0x00
#define LIS331DLH_CR1_DR_100            0x08
#define LIS331DLH_CR1_DR_400            0x10
#define LIS331DLH_CR1_DR_1000           0x18
#define LIS331DLH_CR1_Z_ENABLE          0x04
#define LIS331DLH_CR1_Y_ENABLE          0x02
#define LIS331DLH_CR1_X_ENABLE          0x01
#define LIS331DLH_CR1_XYZ_ENABLE        0x07

#define LIS331DLH_CR4_BDU_ON            0x80
#define LIS331DLH_CR4_BDU_OFF           0x00
#define LIS331DLH_CR4_BLE_BE            0x40
#define LIS331DLH_CR4_BLE_LE            0x00
#define LIS331DLH_CR4_FS_2G             0x00
#define LIS331DLH_CR4_FS_4G             0x10
#define LIS331DLH_CR4_FS_8G             0x30

/*
 * Acceleration measurement
 */
#define LIS331DLM_RESOLUTION            64
#define LIS331DLH_RESOLUTION            16384

/* ABS axes parameter range [um/s^2] (for input event) */
#define GRAVITY_EARTH                   9806550
#define ABSMIN_2G                       (-GRAVITY_EARTH * 2)
#define ABSMAX_2G                       (GRAVITY_EARTH * 2)

struct acceleration {
	int x;
	int y;
	int z;
};

/*
 * Output data rate
 */
/* DR bits of CTRL_REG1 in low power mode */
#define DR_LP_LPF_37HZ                  (0<<3)
#define DR_LP_LPF_74HZ                  (1<<3)
#define DR_LP_LPF_292HZ                 (2<<3)
#define DR_LP_LPF_780HZ                 (3<<3)

struct lis331dlh_odr {
	unsigned long delay;	/* min delay (msec) in the range of ODR */
	u8 odr;			/* register value of ODR */
};

static const struct lis331dlh_odr lis331dlh_odr_table[] = {
	{1, 0x38},		/* ODR = 1000 (Hz) */
	{3, 0x30},		/*        400      */
	{10, 0x28},		/*        100      */
	{20, 0x20},		/*         50      */
	{100, 0xc0 | DR_LP_LPF_780HZ},	/*         10      */
	{200, 0xa0 | DR_LP_LPF_780HZ},	/*          5      */
	{500, 0x80 | DR_LP_LPF_780HZ},	/*          2      */
	{1000, 0x60 | DR_LP_LPF_780HZ},	/*          1      */
	{2000, 0x40 | DR_LP_LPF_780HZ},	/*          0.5    */
};

static const struct lis331dlh_odr lis331dlm_odr_table[] = {
	{3, 0x30},		/* ODR =  400 (Hz) */
	{10, 0x28},		/*        100      */
	{20, 0x20},		/*         50      */
	{100, 0xc0 | DR_LP_LPF_74HZ},	/*         10      */
	{200, 0xa0 | DR_LP_LPF_74HZ},	/*          5      */
	{500, 0x80 | DR_LP_LPF_74HZ},	/*          2      */
	{1000, 0x60 | DR_LP_LPF_74HZ},	/*          1      */
	{2000, 0x40 | DR_LP_LPF_74HZ},	/*          0.5    */
};

/*
 * Transformation matrix for chip mounting position
 */
static const int lis331dlh_position_map[][3][3] = {
	{{-1, 0, 0}, {0, -1, 0}, {0, 0, 1}},	/* top/upper-left */
	{{0, -1, 0}, {1, 0, 0}, {0, 0, 1}},	/* top/upper-right */
	{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}},	/* top/lower-right */
	{{0, 1, 0}, {-1, 0, 0}, {0, 0, 1}},	/* top/lower-left */
	{{1, 0, 0}, {0, -1, 0}, {0, 0, -1}},	/* bottom/upper-left */
	{{0, 1, 0}, {1, 0, 0}, {0, 0, -1}},	/* bottom/upper-right */
	{{-1, 0, 0}, {0, 1, 0}, {0, 0, -1}},	/* bottom/lower-right */
	{{0, -1, 0}, {-1, 0, 0}, {0, 0, -1}}, /* bottom/lower-right */
};

/*
 * driver private data
 */
struct lis331dlh_data {
	int id;			/* value of WHO_AM_I */
	atomic_t enable;	/* attribute value */
	atomic_t delay;		/* attribute value */
	atomic_t position;	/* attribute value */
	u8 odr;			/* output data rate register value */
	unsigned long min_delay;	/* min interval for delay */
	struct acceleration last;	/* last measured data */
	struct mutex enable_mutex;
	struct mutex data_mutex;
	struct i2c_client *client;
	struct input_dev *input;
	struct delayed_work work;
#if DEBUG
	int suspend;
#endif
};

#define is_lis331dl(p) ((p)->id == LIS331DL_ID)
#define is_lis331dlm(p) ((p)->id == LIS331DLM_ID)

#define delay_to_jiffies(d) ((d)?msecs_to_jiffies(d):1)
#define actual_delay(d)     (jiffies_to_msecs(delay_to_jiffies(d)))

/* register access functions */
#define lis331dlh_read(c,a,d,l)  lis331dlh_i2c_transfer((c),(a),(d),(l),I2C_M_RD)
#define lis331dlh_write(c,a,d,l) lis331dlh_i2c_transfer((c),(a),(d),(l),0)
static int lis331dlh_i2c_transfer(struct i2c_client *client,
				  u8 addr, u8 * data, int len, int flag);

static int lis331dl_read(struct i2c_client *client, u8 reg, u8 * pval)
{
	int ret;
	int status;

	if (client == NULL)	/* No global client pointer? */
		return -1;
	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret >= 0) {
		*pval = ret;
		status = 0;
	} else {
		status = -EIO;
	}

	return status;
}

static int lis331dl_write(struct i2c_client *client, u8 reg, u8 val)
{
	int ret;
	int status;

	if (client == NULL)	/* No global client pointer? */
		return -1;
	ret = i2c_smbus_write_byte_data(client, reg, val);
	if (ret == 0)
		status = 0;
	else
		status = -EIO;

	return status;
}

static int lis331dl_read_16(struct i2c_client *client, int reg, int *pval)
{
	unsigned char dest[2];
	int ret, status;
	/*g-sensor spec indicats if use i2c block read, the MSb of SUB must be 1 */
	ret = i2c_smbus_read_i2c_block_data(client, reg | 0x80, 2, dest);
	if (ret >= 0) {
		status = 0;
		/* In "12 bit right justified" mode, bit 6, bit 7, bit 8 = bit 5 */
		*pval = (s16) ((dest[1] << 8) | dest[0]);
	} else
		status = -EIO;
	return status;
}

/*
 * Device dependant operations
 */
static int lis331dlh_power_up(struct lis331dlh_data *lis331dlh)
{
	struct i2c_client *client = lis331dlh->client;
	u8 data;

	if (lis331dlh->id == LIS331DL_ID) {
		lis331dl_write(client, LIS331DL_CTRL_REG1, LIS331DL_CR1_DR_400 |
			       LIS331DL_CR1_PM_NORMAL |
			       LIS331DL_CR1_XYZ_ENABLE);

		lis331dl_write(client, LIS331DL_FF_WU_CFG, 0);

		lis331dl_read(client, LIS331DL_CTRL_REG2, &data);
		data |= LIS331DL_CR2_BDU | LIS331DL_CR2_IEN;
		lis331dl_write(client, LIS331DL_CTRL_REG2, data);
	} else {
		data = lis331dlh->odr | LIS331DLH_CR1_XYZ_ENABLE;
		lis331dlh_write(client, LIS331DLH_CTRL_REG1, &data, 1);
	}
	return 0;
}

static int lis331dlh_power_down(struct lis331dlh_data *lis331dlh)
{
	struct i2c_client *client = lis331dlh->client;
	u8 data;

	if (lis331dlh->id == LIS331DL_ID) {
		lis331dl_write(client, LIS331DLH_CTRL_REG1, 0);
	} else {
		data = LIS331DLH_CR1_XYZ_ENABLE;
		lis331dlh_write(client, LIS331DLH_CTRL_REG1, &data, 1);
	}
	return 0;
}

static int lis331dlh_hw_init(struct lis331dlh_data *lis331dlh)
{
	struct i2c_client *client = lis331dlh->client;
	u8 data;

	if (lis331dlh->id == LIS331DL_ID) {
		lis331dl_write(client, LIS331DL_CTRL_REG1, LIS331DL_CR1_DR_400 |
			       LIS331DL_CR1_PM_NORMAL |
			       LIS331DL_CR1_XYZ_ENABLE);

		lis331dl_write(client, LIS331DL_FF_WU_CFG, 0);

		lis331dl_read(client, LIS331DL_CTRL_REG2, &data);
		data |= LIS331DL_CR2_BDU | LIS331DL_CR2_IEN;
		lis331dl_write(client, LIS331DL_CTRL_REG2, data);
	} else {
		data = LIS331DLH_CR1_XYZ_ENABLE;
		lis331dlh_write(client, LIS331DLH_CTRL_REG1, &data, 1);

		data = 0x00;
		lis331dlh_write(client, LIS331DLH_CTRL_REG2, &data, 1);

		data = 0x00;
		lis331dlh_write(client, LIS331DLH_CTRL_REG3, &data, 1);

		data = LIS331DLH_CR4_FS_2G;
		if (lis331dlh->id == LIS331DLH_ID)
			data |= LIS331DLH_CR4_BDU_ON | LIS331DLH_CR4_BLE_LE;
		lis331dlh_write(client, LIS331DLH_CTRL_REG4, &data, 1);

		data = 0x00;
		lis331dlh_write(client, LIS331DLH_CTRL_REG5, &data, 1);
	}
	return 0;
}

static int lis331dlh_get_enable(struct device *dev)
{
	struct input_dev *input = to_input_dev(dev);
	struct lis331dlh_data *lis331dlh = input_get_drvdata(input);

	return atomic_read(&lis331dlh->enable);
}

static void lis331dlh_set_enable(struct device *dev, unsigned long enable)
{
	struct input_dev *input = to_input_dev(dev);
	struct lis331dlh_data *lis331dlh = input_get_drvdata(input);
	int delay = 0;

	delay = atomic_read(&lis331dlh->delay);
	mutex_lock(&lis331dlh->enable_mutex);

	if (enable) {		/* enable if state will be changed */
		if (!atomic_cmpxchg(&lis331dlh->enable, 0, 1)) {
			lis331dlh_power_up(lis331dlh);
			schedule_delayed_work(&lis331dlh->work,
					      delay_to_jiffies(delay) + 1);
		}
	} else {		/* disable if state will be changed */
		if (atomic_cmpxchg(&lis331dlh->enable, 1, 0)) {
			cancel_delayed_work_sync(&lis331dlh->work);
			lis331dlh_power_down(lis331dlh);
		}
	}
	atomic_set(&lis331dlh->enable, enable);

	mutex_unlock(&lis331dlh->enable_mutex);
}

static int lis331dlh_get_delay(struct device *dev)
{
	struct input_dev *input = to_input_dev(dev);
	struct lis331dlh_data *lis331dlh = input_get_drvdata(input);

	return atomic_read(&lis331dlh->delay);
}

static void lis331dlh_set_delay(struct device *dev, unsigned long delay)
{
	struct input_dev *input = to_input_dev(dev);
	struct lis331dlh_data *lis331dlh = input_get_drvdata(input);
	struct i2c_client *client = lis331dlh->client;
	const struct lis331dlh_odr *odr_table = is_lis331dlm(lis331dlh) ?
	    lis331dlm_odr_table : lis331dlh_odr_table;
	const int size = is_lis331dlm(lis331dlh) ?
	    ARRAY_SIZE(lis331dlm_odr_table) : ARRAY_SIZE(lis331dlh_odr_table);
	int i;
	u8 data;

	if (delay < lis331dlh->min_delay) {
		dev_dbg(dev,
			"delay value %ld ms is too small, set to min delay %ld ms\n",
			delay, lis331dlh->min_delay);
		delay = lis331dlh->min_delay;
	}
	/* determine optimum ODR */
	for (i = 1; (i < size) && (actual_delay(delay) >= odr_table[i].delay);
	     i++) ;
	lis331dlh->odr = odr_table[i - 1].odr;
	atomic_set(&lis331dlh->delay, delay);

	mutex_lock(&lis331dlh->enable_mutex);

	/* update CTRL register and reschedule work_queue if enable=1 */
	if (lis331dlh_get_enable(dev)) {
		cancel_delayed_work_sync(&lis331dlh->work);
		if (lis331dlh->id == LIS331DL_ID) {
			lis331dl_write(client, LIS331DL_CTRL_REG1,
				       LIS331DL_CR1_DR_400 |
				       LIS331DL_CR1_PM_NORMAL |
				       LIS331DL_CR1_XYZ_ENABLE);

			lis331dl_write(client, LIS331DL_FF_WU_CFG, 0);

			lis331dl_read(client, LIS331DL_CTRL_REG2, &data);
			data |= LIS331DL_CR2_BDU | LIS331DL_CR2_IEN;
			lis331dl_write(client, LIS331DL_CTRL_REG2, data);
		} else {
			data = lis331dlh->odr | LIS331DLH_CR1_XYZ_ENABLE;
			lis331dlh_write(client, LIS331DLH_CTRL_REG1, &data, 1);
		}
		schedule_delayed_work(&lis331dlh->work,
				      delay_to_jiffies(delay) + 1);
	}

	mutex_unlock(&lis331dlh->enable_mutex);
}

static int lis331dlh_get_position(struct device *dev)
{
	struct input_dev *input = to_input_dev(dev);
	struct lis331dlh_data *lis331dlh = input_get_drvdata(input);

	return atomic_read(&lis331dlh->position);
}

static void lis331dlh_set_position(struct device *dev, unsigned long position)
{
	struct input_dev *input = to_input_dev(dev);
	struct lis331dlh_data *lis331dlh = input_get_drvdata(input);

	atomic_set(&lis331dlh->position, position);
}

static int lis331dlh_measure(struct lis331dlh_data *lis331dlh,
			     struct acceleration *accel)
{
	struct i2c_client *client = lis331dlh->client;
	u8 buf[6];
	int raw[3], data[3];
	int pos = atomic_read(&lis331dlh->position);
	int i, j, ret;

	if (is_lis331dl(lis331dlh)) {
		ret = lis331dl_read_16(client, LIS331DLH_OUT_X_L, &raw[0]);
		if (ret < 0)
			return ret;
		ret = lis331dl_read_16(client, LIS331DLH_OUT_Y_L, &raw[1]);
		if (ret < 0)
			return ret;
		ret = lis331dl_read_16(client, LIS331DLH_OUT_Z_L, &raw[2]);
		if (ret < 0)
			return ret;
	} else {
		/* read acceleration data */
		if (lis331dlh_read(client, LIS331DLH_OUT_X_L, buf, 6) < 0) {
			dev_err(&client->dev,
				"I2C block read error: addr=0x%02x, len=%d\n",
				LIS331DLH_OUT_X_L, 6);
			for (i = 0; i < 3; i++) {
				raw[i] = 0;
			}
		} else {
			for (i = 0; i < 3; i++) {
				raw[i] = *(s16 *) & buf[i * 2];
			}
		}
	}

	/* for X, Y, Z axis */
	for (i = 0; i < 3; i++) {
		/* coordinate transformation */
		data[i] = 0;
		for (j = 0; j < 3; j++) {
			data[i] += raw[j] * lis331dlh_position_map[pos][i][j];
		}
		/* normalization */
		if (is_lis331dlm(lis331dlh) || is_lis331dl(lis331dlh)) {
			data[i] =
			    (data[i] >> 8) * GRAVITY_EARTH /
			    LIS331DLM_RESOLUTION;
		} else {
			long long g;
			g = (long long)data[i] * GRAVITY_EARTH /
			    LIS331DLH_RESOLUTION;
			data[i] = g;
		}
	}

	dev_dbg(&client->dev, "raw(%6d,%6d,%6d) => norm(%8d,%8d,%8d)\n",
		raw[0], raw[1], raw[2], data[0], data[1], data[2]);

	accel->x = data[0];
	accel->y = data[1];
	accel->z = data[2];

	return 0;
}

static void lis331dlh_work_func(struct work_struct *work)
{
	struct lis331dlh_data *lis331dlh =
	    container_of((struct delayed_work *)work,
			 struct lis331dlh_data, work);
	unsigned long delay = delay_to_jiffies(atomic_read(&lis331dlh->delay));
	struct acceleration accel;
	accel.x = accel.y = accel.z = 0;
	lis331dlh_measure(lis331dlh, &accel);

	input_report_abs(lis331dlh->input, ABS_X, accel.x);
	input_report_abs(lis331dlh->input, ABS_Y, accel.y);
	input_report_abs(lis331dlh->input, ABS_Z, accel.z);
	input_sync(lis331dlh->input);

	mutex_lock(&lis331dlh->data_mutex);
	lis331dlh->last = accel;
	mutex_unlock(&lis331dlh->data_mutex);

	schedule_delayed_work(&lis331dlh->work, delay);
}

/*
 * Input device interface
 */

static void lis331dlh_input_fini(struct lis331dlh_data *lis331dlh)
{
	struct input_dev *dev = lis331dlh->input;

	input_unregister_device(dev);
	input_free_device(dev);
}

/*
 * sysfs device attributes
 */
static ssize_t lis331dlh_enable_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", lis331dlh_get_enable(dev));
}

static ssize_t lis331dlh_enable_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	unsigned long enable = simple_strtoul(buf, NULL, 10);
	lis331dlh_set_enable(dev, enable);
	return count;
}

static ssize_t lis331dlh_delay_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", lis331dlh_get_delay(dev));
}

static ssize_t lis331dlh_delay_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	unsigned long delay = simple_strtoul(buf, NULL, 10);

	if (delay > LIS331DLH_MAX_DELAY) {
		delay = LIS331DLH_MAX_DELAY;
	}

	lis331dlh_set_delay(dev, delay);

	return count;
}

static ssize_t lis331dlh_position_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", lis331dlh_get_position(dev));
}

static ssize_t lis331dlh_position_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	unsigned long position;

	position = simple_strtoul(buf, NULL, 10);
	if ((position >= 0) && (position <= 7)) {
		lis331dlh_set_position(dev, position);
	}

	return count;
}

static ssize_t lis331dlh_wake_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	static atomic_t serial = ATOMIC_INIT(0);

	input_report_abs(input, ABS_MISC, atomic_inc_return(&serial));

	return count;
}

static ssize_t lis331dlh_data_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct lis331dlh_data *lis331dlh = input_get_drvdata(input);
	struct acceleration accel;

	mutex_lock(&lis331dlh->data_mutex);
	accel = lis331dlh->last;
	mutex_unlock(&lis331dlh->data_mutex);

	return sprintf(buf, "%d %d %d\n", accel.x, accel.y, accel.z);
}

#if DEBUG
static ssize_t lis331dlh_debug_reg_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct lis331dlh_data *lis331dlh = input_get_drvdata(input);
	struct i2c_client *client = lis331dlh->client;
	u8 reg[5];

	lis331dlh_read(client, LIS331DLH_CTRL_REG1, reg, 5);
	return sprintf(buf, "%02x %02x %02x %02x %02x\n",
		       reg[0], reg[1], reg[2], reg[3], reg[4]);
}

static int lis331dlh_suspend(struct i2c_client *client, pm_message_t mesg);
static int lis331dlh_resume(struct i2c_client *client);

static ssize_t lis331dlh_debug_suspend_show(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct lis331dlh_data *lis331dlh = input_get_drvdata(input);

	return sprintf(buf, "%d\n", lis331dlh->suspend);
}

static ssize_t lis331dlh_debug_suspend_store(struct device *dev,
					     struct device_attribute *attr,
					     const char *buf, size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	struct lis331dlh_data *lis331dlh = input_get_drvdata(input);
	struct i2c_client *client = lis331dlh->client;
	unsigned long suspend = simple_strtoul(buf, NULL, 10);

	if (suspend) {
		pm_message_t msg;
		lis331dlh_suspend(client, msg);
	} else {
		lis331dlh_resume(client);
	}

	return count;
}
#endif /* DEBUG */

static DEVICE_ATTR(active, S_IRUGO | S_IWUGO,
		   lis331dlh_enable_show, lis331dlh_enable_store);
static DEVICE_ATTR(interval, S_IRUGO | S_IWUGO,
		   lis331dlh_delay_show, lis331dlh_delay_store);
static DEVICE_ATTR(position, S_IRUGO | S_IWUGO,
		   lis331dlh_position_show, lis331dlh_position_store);
static DEVICE_ATTR(wake, S_IWUGO, NULL, lis331dlh_wake_store);
static DEVICE_ATTR(data, S_IRUGO, lis331dlh_data_show, NULL);

#if DEBUG
static DEVICE_ATTR(debug_reg, S_IRUGO, lis331dlh_debug_reg_show, NULL);
static DEVICE_ATTR(debug_suspend, S_IRUGO | S_IWUGO,
		   lis331dlh_debug_suspend_show, lis331dlh_debug_suspend_store);
#endif /* DEBUG */

static struct attribute *lis331dlh_attributes[] = {
	&dev_attr_active.attr,
	&dev_attr_interval.attr,
	&dev_attr_position.attr,
	&dev_attr_wake.attr,
	&dev_attr_data.attr,
#if DEBUG
	&dev_attr_debug_reg.attr,
	&dev_attr_debug_suspend.attr,
#endif /* DEBUG */
	NULL
};

static struct attribute_group lis331dlh_attribute_group = {
	.attrs = lis331dlh_attributes
};

/*
 * I2C client
 */
static int lis331dlh_i2c_transfer(struct i2c_client *client,
				  u8 addr, u8 * data, int len, int flag)
{
	struct i2c_msg msg[2];
	u8 reg;
	int err;

	/* set msb of register address when multi-byte access */
	reg = addr | (len > 1 ? 0x80 : 0);
	msg[0].addr = client->addr;
	msg[0].flags = client->flags & I2C_M_TEN;
	msg[0].len = 1;
	msg[0].buf = &reg;
	msg[1].addr = client->addr;
	msg[1].flags = (client->flags & I2C_M_TEN) | flag;
	msg[1].len = len;
	msg[1].buf = data;

	err = i2c_transfer(client->adapter, msg, 2);
	if (err != 2) {
		dev_err(&client->dev,
			"I2C %s error: slave_addr=%d, addr=0x%02x, err=%d\n",
			(flag & I2C_M_RD) ? "read" : "write", client->addr, reg,
			err);
		return err;
	}

	return 0;
}

static int lis331dlh_detect(struct i2c_client *client,
			    struct i2c_board_info *info)
{
	struct lis331dlh_data *lis331dlh = i2c_get_clientdata(client);
	s32 id;

	id = i2c_smbus_read_byte_data(client, LIS331DLH_WHO_AM_I);
	if (!
	    ((id == LIS331DL_ID) || (id == LIS331DLH_ID)
	     || (id == LIS331DLM_ID))) {
		return -ENODEV;
	}
	lis331dlh->id = id;

	return 0;
}

static int lis331dlh_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct lis331dlh_data *lis331dlh;
	struct input_dev *input_data = NULL;
	int err;
	/* setup private data */
	lis331dlh = kzalloc(sizeof(struct lis331dlh_data), GFP_KERNEL);
	if (!lis331dlh) {
		err = -ENOMEM;
		goto error_0;
	}
	/*platform data should be min delay value */
	if (client->dev.platform_data)
		lis331dlh->min_delay =
		    *((unsigned long *)client->dev.platform_data);
	mutex_init(&lis331dlh->enable_mutex);
	mutex_init(&lis331dlh->data_mutex);
	/* setup i2c client */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto error_1;
	}
	i2c_set_clientdata(client, lis331dlh);
	lis331dlh->client = client;
	/* detect and init hardware */
	if ((err = lis331dlh_detect(client, NULL) < 0)) {
		goto error_1;
	}
	dev_info(&client->dev, "device id=%02x: %s found\n",
		 lis331dlh->id, id->name);
	lis331dlh_hw_init(lis331dlh);
	lis331dlh_set_delay(&client->dev, LIS331DLH_DEFAULT_DELAY);
	lis331dlh_set_position(&client->dev, CONFIG_SENSORS_LIS331DL_POSITION);

	/* setup driver interfaces */
	INIT_DELAYED_WORK(&lis331dlh->work, lis331dlh_work_func);

	input_data = input_allocate_device();
	if (!input_data) {
		err = -ENOMEM;
		printk(KERN_ERR
		       "lis331dlh_probe: Failed to allocate input_data device\n");
		goto error_1;
	}
	input_data->name = "accelerometer";
	input_data->id.bustype = BUS_I2C;
	input_set_capability(input_data, EV_ABS, ABS_MISC);
	input_set_abs_params(input_data, ABS_X, ABSMIN_2G, ABSMAX_2G, 0, 0);
	input_set_abs_params(input_data, ABS_Y, ABSMIN_2G, ABSMAX_2G, 0, 0);
	input_set_abs_params(input_data, ABS_Z, ABSMIN_2G, ABSMAX_2G, 0, 0);
	input_set_drvdata(input_data, lis331dlh);

	err = input_register_device(input_data);
	if (err < 0) {
		input_free_device(input_data);
		goto error_1;
	}
	lis331dlh->input = input_data;

	err =
	    sysfs_create_group(&input_data->dev.kobj,
			       &lis331dlh_attribute_group);
	if (err < 0) {
		goto error_2;
	}

	return 0;

error_2:
	lis331dlh_input_fini(lis331dlh);
error_1:
	kfree(lis331dlh);
error_0:
	return err;
}

static int lis331dlh_remove(struct i2c_client *client)
{
	struct lis331dlh_data *lis331dlh = i2c_get_clientdata(client);

	lis331dlh_set_enable(&client->dev, 0);

	sysfs_remove_group(&lis331dlh->input->dev.kobj,
			   &lis331dlh_attribute_group);
	lis331dlh_input_fini(lis331dlh);
	kfree(lis331dlh);

	return 0;
}

static int lis331dlh_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct lis331dlh_data *lis331dlh = i2c_get_clientdata(client);
	TRACE_FUNC();

	mutex_lock(&lis331dlh->enable_mutex);

	if (lis331dlh_get_enable(&client->dev)) {
		cancel_delayed_work_sync(&lis331dlh->work);
		lis331dlh_power_down(lis331dlh);
	}
#if DEBUG
	lis331dlh->suspend = 1;
#endif

	mutex_unlock(&lis331dlh->enable_mutex);
	return 0;
}

static int lis331dlh_resume(struct i2c_client *client)
{
	struct lis331dlh_data *lis331dlh = i2c_get_clientdata(client);
	int delay = atomic_read(&lis331dlh->delay);

	TRACE_FUNC();

	lis331dlh_hw_init(lis331dlh);

	mutex_lock(&lis331dlh->enable_mutex);

	if (lis331dlh_get_enable(&client->dev)) {
		lis331dlh_power_up(lis331dlh);
		schedule_delayed_work(&lis331dlh->work,
				      delay_to_jiffies(delay) + 1);
	}
#if DEBUG
	lis331dlh->suspend = 0;
#endif

	mutex_unlock(&lis331dlh->enable_mutex);

	return 0;
}

static const struct i2c_device_id lis331dlh_id[] = {
	{LIS331DL_NAME, 0},
	{LIS331DLH_NAME, 0},
	{LIS331DLM_NAME, 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, lis331dlh_id);

static struct i2c_driver lis331dlh_driver = {
	.driver = {
		   .name = "lis331dl",
		   .owner = THIS_MODULE,
		   },
	.probe = lis331dlh_probe,
	.remove = lis331dlh_remove,
	.suspend = lis331dlh_suspend,
	.resume = lis331dlh_resume,
	.id_table = lis331dlh_id,
};

/*
 * Module init and exit
 */
static int __init lis331dlh_init(void)
{
	return i2c_add_driver(&lis331dlh_driver);
}

module_init(lis331dlh_init);

static void __exit lis331dlh_exit(void)
{
	i2c_del_driver(&lis331dlh_driver);
}

module_exit(lis331dlh_exit);

MODULE_AUTHOR("JC Liao <jcliao@marvell.com>");
MODULE_DESCRIPTION("LIS331DL/LIS331DLH/LIS331DLM accelerometer driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(LIS331DLH_VERSION);
