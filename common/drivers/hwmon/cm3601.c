/*
 * drivers/hwmon/cm3601.c
 *
 * Copyright (2008-2010) Marvell International Ltd.
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

#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/mfd/88pm860x.h>
#include <linux/slab.h>
#include <plat/mfp.h>

#define SENSOR_NAME_LIGHT "cm3601_as"
#define SENSOR_NAME_PROXIMITY "cm3601_ps"
#define CM3601_DEFAULT_DELAY         200
#define CM3601_MAX_DELAY             2000
#define delay_to_jiffies(d) ((d)?msecs_to_jiffies(d):1)
#define DEVICE_ATTR2(_name, _mode, _show, _store) \
struct device_attribute dev_attr2_##_name = __ATTR(_name, _mode, _show, _store)

struct input_dev *this_input_dev_as = NULL;
struct input_dev *this_input_dev_ps = NULL;

struct pm860x_cm3601_info {
	struct pm860x_chip	*chip;
	struct i2c_client	*i2c;
	int gpio_en; /*GPIO NUM*/
	int gpio_out; /*GPIO NUM */
	int (*request_source)(unsigned char gpio_num, char *name);
	void (*release_source)(unsigned char gpio_num);
};
static struct pm860x_cm3601_info *info;

struct cm3601_data {
	atomic_t enable;                /* attribute value */
	atomic_t delay;                 /* attribute value */
	int last;       /* last measured data */
	struct mutex enable_mutex;
	struct mutex data_mutex;
	struct input_dev *input;
	struct delayed_work work;
};

static int get_cm3601_enable(struct input_dev * this_input_dev)
{
	struct cm3601_data *cm3601 = NULL;
	int ret;

	cm3601 = input_get_drvdata(this_input_dev);
	ret = atomic_read(&cm3601->enable);

	return ret;
}

static void cm3601_as_power_onoff(int power)
{
	if(power == 1) {
		if(!get_cm3601_enable(this_input_dev_ps))	{
#if defined(CONFIG_PMIC_D1980)
            // TODO: 12Apr2011. DLG Check...
			gpio_direction_output(mfp_to_gpio(MFP_PIN_GPIO51),0);
#else
			gpio_direction_output(mfp_to_gpio(info->gpio_en),0);
			pm860x_set_bits(info->i2c, PM8607_GPADC_MISC2, PM8607_GPADC3_GP_BIAS_A3, 0x00);	
#endif
        }
	}
	else {
		if(!get_cm3601_enable(this_input_dev_ps)){
			/* disable sensor */
#if defined(CONFIG_PMIC_D1980)
            // TODO: 12Apr2011. DLG Check...
			gpio_direction_output(mfp_to_gpio(MFP_PIN_GPIO51),1);
#else
			gpio_direction_output(mfp_to_gpio(info->gpio_en),1);
			pm860x_set_bits(info->i2c, PM8607_GPADC_MISC2, PM8607_GPADC3_GP_BIAS_A3, PM8607_GPADC3_GP_BIAS_A3);
#endif
		}
	}
}

static void cm3601_ps_power_onoff(int power)
{
#if defined(CONFIG_PMIC_D1980)
	if(power == 1) {
		gpio_direction_output(mfp_to_gpio(MFP_PIN_GPIO51),0);
	}
	else {
		if(!get_cm3601_enable(this_input_dev_as))
			gpio_direction_output(mfp_to_gpio(MFP_PIN_GPIO51),1);
	}
#else
	if(power == 1) {
		gpio_direction_output(mfp_to_gpio(info->gpio_en),0);
	}
	else {
		if(!get_cm3601_enable(this_input_dev_as))
			gpio_direction_output(mfp_to_gpio(info->gpio_en),1);
	}
#endif
}

static u16 pm860xGPADC3MeasurmentValue(u8 measRegister)
{
	u16 mesurementValue;
	u8 regValue[2];

#if defined(CONFIG_PMIC_D1980)
    // TODO: 12Apr2011. DLG Check...
#else
	if (pm860x_bulk_read(info->i2c, measRegister, 2, regValue) == 2)
	{
		/* Read two registers, the alignment will be done as follows:
		     Register 1 - bits 7:0 => 8 MSB bits <11:4> of measurement value
		     Register 2 - bits 3:0 => 4 LSB bits <3:0> of measurement value */
		mesurementValue = ((regValue[0] << 4) | (regValue[1] & 0x0F));
		pr_debug("sensor value: 0x%x\n",mesurementValue);
	}
	else
#endif
	{
		return 0;
	}
	return (mesurementValue);
}

void cm3601_getADC(int * measmentRead)
{
	*measmentRead = pm860xGPADC3MeasurmentValue(PM8607_GPADC3_MEAS1);
}
EXPORT_SYMBOL(cm3601_getADC);


static int cm3601_get_enable(struct device *dev)
{
	struct input_dev *input = to_input_dev(dev);
	struct cm3601_data *cm3601 = input_get_drvdata(input);

	return atomic_read(&cm3601->enable);
}

static void cm3601_set_delay(struct device *dev, unsigned long delay)
{
	struct input_dev *input = to_input_dev(dev);
	struct cm3601_data *cm3601 = input_get_drvdata(input);

	atomic_set(&cm3601->delay, delay);

	mutex_lock(&cm3601->enable_mutex);

	if (cm3601_get_enable(dev)) {
		cancel_delayed_work_sync(&cm3601->work);
		schedule_delayed_work(&cm3601->work,
				      delay_to_jiffies(delay) + 1);
	}

	mutex_unlock(&cm3601->enable_mutex);
}

static void cm3601_input_fini(struct cm3601_data *cm3601)
{
	struct input_dev *dev = cm3601->input;

	input_unregister_device(dev);
	input_free_device(dev);
}

static void cm3601_as_set_enable(struct device *dev, unsigned long enable)
{
	struct input_dev *input = to_input_dev(dev);
	struct cm3601_data *cm3601 = input_get_drvdata(input);
	int delay = atomic_read(&cm3601->delay);

	mutex_lock(&cm3601->enable_mutex);

  if (enable) {                   /* enable if state will be changed */
		if (!atomic_cmpxchg(&cm3601->enable, 0, 1)) {
			cm3601_as_power_onoff(1);
			schedule_delayed_work(&cm3601->work,
					      delay_to_jiffies(delay) + 1);
		}
	} else {                        /* disable if state will be changed */
		if (atomic_cmpxchg(&cm3601->enable, 1, 0)) {
			cancel_delayed_work_sync(&cm3601->work);
			cm3601_as_power_onoff(0);
		}
	}
	atomic_set(&cm3601->enable, enable);

	mutex_unlock(&cm3601->enable_mutex);
}

static void cm3601_ps_set_enable(struct device *dev, unsigned long enable)
{
	struct input_dev *input = to_input_dev(dev);
	struct cm3601_data *cm3601 = input_get_drvdata(input);
	int delay = atomic_read(&cm3601->delay);

	mutex_lock(&cm3601->enable_mutex);

  if (enable) {                   /* enable if state will be changed */
		if (!atomic_cmpxchg(&cm3601->enable, 0, 1)) {
			cm3601_ps_power_onoff(1);
			schedule_delayed_work(&cm3601->work,
					      delay_to_jiffies(delay) + 1);
		}
	} else {                        /* disable if state will be changed */
		if (atomic_cmpxchg(&cm3601->enable, 1, 0)) {
			cancel_delayed_work_sync(&cm3601->work);
			cm3601_ps_power_onoff(0);
		}
	}
	atomic_set(&cm3601->enable, enable);

	mutex_unlock(&cm3601->enable_mutex);
}

static ssize_t store_as_active(struct device *dev, struct device_attribute *attr, const char *buf,
		size_t count)
{
	unsigned long enable = simple_strtoul(buf, NULL, 10);

	if ((enable == 0) || (enable == 1)) {
		cm3601_as_set_enable(dev, enable);
	}

	return count;
}

static ssize_t store_ps_active(struct device *dev, struct device_attribute *attr, const char *buf,
		size_t count)
{
	unsigned long enable = simple_strtoul(buf, NULL, 10);

	if ((enable == 0) || (enable == 1)) {
		cm3601_ps_set_enable(dev, enable);
	}

	return count;
}

static ssize_t show_active(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct cm3601_data *cm3601 = input_get_drvdata(input);
	int ret;

	ret = atomic_read(&cm3601->enable);

	return sprintf(buf, "%d\n", ret);
}

/* interval-- rw*/
static ssize_t store_interval(struct device *dev, struct device_attribute *attr, const char *buf,
		size_t count)
{
	unsigned long delay = simple_strtoul(buf, NULL, 10);

	if (delay > CM3601_MAX_DELAY) {
		delay = CM3601_MAX_DELAY;
	}

	cm3601_set_delay(dev, delay);

	return count;
}
static ssize_t show_interval(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret;
	struct input_dev *input = to_input_dev(dev);
	struct cm3601_data *cm3601 = input_get_drvdata(input);

	ret = atomic_read(&cm3601->delay);

	return sprintf(buf, "%d\n", ret);
}

/* wake-- wo*/
static ssize_t store_wake(struct device *dev, struct device_attribute *attr, const char *buf,
		size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	static atomic_t serial = ATOMIC_INIT(0);

	input_report_abs(input, ABS_MISC, atomic_inc_return(&serial));

	return count;
}

/* data-- ro*/
static ssize_t show_data(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct cm3601_data *cm3601 = input_get_drvdata(input);
	int val;

	mutex_lock(&cm3601->data_mutex);
	val = cm3601->last;
	mutex_unlock(&cm3601->data_mutex);

	return sprintf(buf, "%d\n", val);
}

/* status-- ro*/
static ssize_t show_status(struct device *dev, struct device_attribute *attr, char *buf)
{
	/* TODO*/
	return 0;
}

static DEVICE_ATTR(active, S_IRUGO|S_IWUGO, show_active, store_as_active);
static DEVICE_ATTR(interval, S_IRUGO|S_IWUGO, show_interval, store_interval);
static DEVICE_ATTR(wake, S_IWUGO, NULL, store_wake);
static DEVICE_ATTR(data, S_IRUGO, show_data, NULL);
static DEVICE_ATTR(status, S_IRUGO, show_status, NULL);

static DEVICE_ATTR2(active, S_IRUGO|S_IWUGO, show_active, store_ps_active);
static DEVICE_ATTR2(interval, S_IRUGO|S_IWUGO, show_interval, store_interval);
static DEVICE_ATTR2(wake, S_IWUGO, NULL, store_wake);
static DEVICE_ATTR2(data, S_IRUGO, show_data, NULL);
static DEVICE_ATTR2(status, S_IRUGO, show_status, NULL);

static struct attribute *cm3601_as_attributes[] = {
	&dev_attr_active.attr,
	&dev_attr_interval.attr,
	&dev_attr_wake.attr,
	&dev_attr_data.attr,
	&dev_attr_status.attr,
	NULL
};

static struct attribute *cm3601_ps_attributes[] = {
	&dev_attr2_active.attr,
	&dev_attr2_interval.attr,
	&dev_attr2_wake.attr,
	&dev_attr2_data.attr,
	&dev_attr2_status.attr,
	NULL
};

static struct attribute_group cm3601_ps_attribute_group = {
	.attrs = cm3601_ps_attributes
};

static struct attribute_group cm3601_as_attribute_group = {
	.attrs = cm3601_as_attributes
};

static void cm3601_as_work_func(struct work_struct *work)
{
	struct cm3601_data *cm3601 = container_of((struct delayed_work *)work, struct cm3601_data, work);
	unsigned long delay = delay_to_jiffies(atomic_read(&cm3601->delay));
	int val = 0;

	cm3601_getADC(&val);
	input_report_abs(cm3601->input, ABS_PRESSURE, val);
	input_sync(cm3601->input);

	mutex_lock(&cm3601->data_mutex);
	cm3601->last = val;
	mutex_unlock(&cm3601->data_mutex);

	schedule_delayed_work(&cm3601->work, delay);
}

static void cm3601_ps_work_func(struct work_struct *work)
{
	struct cm3601_data *cm3601 = container_of((struct delayed_work *)work, struct cm3601_data, work);
	unsigned long delay = delay_to_jiffies(atomic_read(&cm3601->delay));
	int val = 0;

#if defined(CONFIG_PMIC_D1980)
	if(__gpio_get_value(mfp_to_gpio(MFP_PIN_GPIO50))) {
#else
	if(__gpio_get_value(mfp_to_gpio(info->gpio_out))) {
#endif
		/* report leaving the face value*/
		input_report_abs(cm3601->input, ABS_DISTANCE, 1);
		val = 1;
	} else {
		/* report closing to the face value */
		input_report_abs(cm3601->input, ABS_DISTANCE, 0);
		val = 0;
	}
	input_sync(cm3601->input);

	mutex_lock(&cm3601->data_mutex);
	cm3601->last = val;
	mutex_unlock(&cm3601->data_mutex);

	schedule_delayed_work(&cm3601->work, delay);
}

static int cm3601_probe(struct platform_device *pdev)
{
	int ret;

#if !defined(CONFIG_PMIC_D1980)
	struct pm860x_platform_data *pm860x_pdata = NULL;
	struct pm860x_cm3601_pdata *plat_info = NULL;
	struct pm860x_chip *chip = dev_get_drvdata(pdev->dev.parent);
#endif

	struct cm3601_data *cm3601_as_data = NULL;
	struct cm3601_data *cm3601_ps_data = NULL;
	struct input_dev *input_dev_as = NULL;
	struct input_dev *input_dev_ps = NULL;

#if !defined(CONFIG_PMIC_D1980)
	if (pdev->dev.parent->platform_data) {
		pm860x_pdata = pdev->dev.parent->platform_data;
		plat_info = pm860x_pdata->cm3601;
	}
	if (plat_info == NULL) {
		dev_err(&pdev->dev, "platform data isn't assigned to cm3601 sensor !\n");
		return -EINVAL;
	}
#endif

	cm3601_as_data = kzalloc(sizeof(struct cm3601_data), GFP_KERNEL);
	if (!cm3601_as_data) {
		ret = -ENOMEM;
		goto error_0;
	}

	cm3601_ps_data = kzalloc(sizeof(struct cm3601_data), GFP_KERNEL);
	if (!cm3601_ps_data) {
		kfree(cm3601_as_data);
		ret = -ENOMEM;
		goto error_0;
	}

#if defined(CONFIG_PMIC_D1980)
    // TODO: 12Apr2011. DLG Check...
	ret = gpio_request(mfp_to_gpio(MFP_PIN_GPIO51), "CM3601_EN");
	if (ret) {
		printk(KERN_ERR "%s: can't request GPIO51.\n", __func__);
		goto error_1;
	}
	/* Enagle CM3601 */
	gpio_direction_output(mfp_to_gpio(MFP_PIN_GPIO51),0);

	/* GPIO50 output   0:leaving  1:closing */
	ret = gpio_request(mfp_to_gpio(MFP_PIN_GPIO50), "CM3601_PS_OUT");
	if (ret) {
		printk(KERN_ERR "%s: can't request GPIO50.\n", __func__);
		goto error_1;
	}
	gpio_direction_input(mfp_to_gpio(MFP_PIN_GPIO50));
#else
	info = kzalloc(sizeof(struct pm860x_cm3601_info), GFP_KERNEL);
	if (!info) {
		kfree(cm3601_as_data);
		kfree(cm3601_ps_data);
		return -ENOMEM;
	}
	info->chip = chip;
	info->i2c = (chip->id == CHIP_PM8607) ? chip->client : chip->companion;
	info->gpio_en = plat_info->gpio_en;
	info->gpio_out = plat_info->gpio_out;
	info->request_source = plat_info->request_source;
	info->release_source = plat_info->release_source;
	ret = info->request_source(mfp_to_gpio(info->gpio_en),"CM3601_EN");
	if (ret) {
		goto error_1;
	}
	/* Enagle CM3601 */
	gpio_direction_output(mfp_to_gpio(info->gpio_en),0);

	/* GPIO50 output   0:leaving  1:closing */
	ret = info->request_source(mfp_to_gpio(info->gpio_out),"CM3601_PS_OUT");
	if (ret) {
		goto error_1;
	}
	gpio_direction_input(mfp_to_gpio(info->gpio_out));
#endif

	mutex_init(&cm3601_as_data->enable_mutex);
	mutex_init(&cm3601_as_data->data_mutex);
	mutex_init(&cm3601_ps_data->enable_mutex);
	mutex_init(&cm3601_ps_data->data_mutex);

	atomic_set(&cm3601_as_data->delay, CM3601_DEFAULT_DELAY);
	atomic_set(&cm3601_ps_data->delay, CM3601_DEFAULT_DELAY);

	INIT_DELAYED_WORK(&cm3601_as_data->work, cm3601_as_work_func);
	INIT_DELAYED_WORK(&cm3601_ps_data->work, cm3601_ps_work_func);

	input_dev_as = input_allocate_device();
	input_dev_ps = input_allocate_device();
	if (!input_dev_as || !input_dev_ps) {
		ret = -ENOMEM;
		printk(KERN_ERR "cm3601_probe: Failed to allocate input_data device\n");
		goto error_1;
	}

	input_dev_as->name = SENSOR_NAME_LIGHT;
	input_dev_as->id.bustype = BUS_I2C;
	input_set_capability(input_dev_as, EV_ABS, ABS_MISC);
	__set_bit(EV_ABS, input_dev_as->evbit);
	__set_bit(ABS_PRESSURE, input_dev_as->absbit);
	input_set_abs_params(input_dev_as, ABS_PRESSURE, -100000, 100000, 0, 0);
	input_set_drvdata(input_dev_as, cm3601_as_data);

	input_dev_ps->name = SENSOR_NAME_PROXIMITY;
	input_dev_ps->id.bustype = BUS_I2C;
	input_set_capability(input_dev_ps, EV_ABS, ABS_MISC);
	__set_bit(EV_ABS, input_dev_ps->evbit);
	__set_bit(ABS_DISTANCE, input_dev_ps->absbit);
	input_set_abs_params(input_dev_ps, ABS_DISTANCE, 0, 1, 0, 0);
	input_set_drvdata(input_dev_ps, cm3601_ps_data);

	ret = input_register_device(input_dev_as);
	if (ret < 0) {
		input_free_device(input_dev_as);
		goto error_1;
	}
	cm3601_as_data->input = input_dev_as;

	ret = input_register_device(input_dev_ps);
	if (ret < 0) {
		input_free_device(input_dev_ps);
		goto error_1;
	}
	cm3601_ps_data->input = input_dev_ps;

	ret = sysfs_create_group(&input_dev_as->dev.kobj,&cm3601_as_attribute_group);
	if (ret < 0) {
		goto error_2;
	}

	ret = sysfs_create_group(&input_dev_ps->dev.kobj,&cm3601_ps_attribute_group);
	if (ret < 0) {
		goto error_2;
	}

	this_input_dev_as = input_dev_as;
	this_input_dev_ps = input_dev_ps;

#if defined(CONFIG_PMIC_D1980)
    // TODO: 12Apr2011. DLG Check...
#else
	/*light sensor will need GPADC3 enabled to get data*/
	ret = pm860x_reg_read(info->i2c,PM8607_MEAS_EN1);
	if( ret < 0)
		goto error_1;
	if(! (ret & PM8607_MEAS_EN1_GPADC3))
	{
		pm860x_set_bits(info->i2c, PM8607_MEAS_EN1,PM8607_MEAS_EN1_GPADC3, PM8607_MEAS_EN1_GPADC3);
	}
#endif
	printk(KERN_INFO "cm3601 init successfully.\n");
	return 0;

error_2:
	cm3601_input_fini(cm3601_as_data);
	cm3601_input_fini(cm3601_ps_data);
error_1:
	if (cm3601_as_data != NULL)
		kfree(cm3601_as_data);
	if (cm3601_ps_data != NULL)
		kfree(cm3601_ps_data);
error_0:
	printk(KERN_INFO "cm3601 init error.\n");
	return ret;
}

static int cm3601_remove(struct platform_device *pdev)
{
	struct cm3601_data *data;

	if (this_input_dev_as != NULL) {
		data = input_get_drvdata(this_input_dev_as);
		sysfs_remove_group(&this_input_dev_as->dev.kobj, &cm3601_as_attribute_group);
		input_unregister_device(this_input_dev_as);
		if (data != NULL) {
			kfree(data);
		}
	}

	if (this_input_dev_ps != NULL) {
		data = input_get_drvdata(this_input_dev_ps);
		sysfs_remove_group(&this_input_dev_ps->dev.kobj, &cm3601_ps_attribute_group);
		input_unregister_device(this_input_dev_ps);
		if (data != NULL) {
			kfree(data);
		}
	}

	info->release_source(info->gpio_en);
	info->release_source(info->gpio_out);

	return 0;
}

static int cm3601_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct cm3601_data *cm3601 = NULL;

	cm3601 = input_get_drvdata(this_input_dev_as);
	mutex_lock(&cm3601->enable_mutex);

	if (atomic_read(&cm3601->enable)) {
		cancel_delayed_work_sync(&cm3601->work);
		cm3601_as_power_onoff(0);
	}

	mutex_unlock(&cm3601->enable_mutex);

	cm3601 = input_get_drvdata(this_input_dev_ps);
	mutex_lock(&cm3601->enable_mutex);

	if (atomic_read(&cm3601->enable)) {
		cancel_delayed_work_sync(&cm3601->work);
		cm3601_ps_power_onoff(0);
	}

	mutex_unlock(&cm3601->enable_mutex);

	return 0;
}

static int cm3601_resume(struct platform_device *pdev)
{
	struct cm3601_data *cm3601 = NULL;
	int delay;

	cm3601 = input_get_drvdata(this_input_dev_as);
	delay = atomic_read(&cm3601->delay);
	mutex_lock(&cm3601->enable_mutex);

	if (atomic_read(&cm3601->enable)) {
		cm3601_as_power_onoff(1);
		schedule_delayed_work(&cm3601->work, delay_to_jiffies(delay) + 1);
	}

	mutex_unlock(&cm3601->enable_mutex);

	cm3601 = input_get_drvdata(this_input_dev_ps);
	delay = atomic_read(&cm3601->delay);
	mutex_lock(&cm3601->enable_mutex);

	if (atomic_read(&cm3601->enable)) {
		cm3601_ps_power_onoff(1);
		schedule_delayed_work(&cm3601->work, delay_to_jiffies(delay) + 1);
	}

	mutex_unlock(&cm3601->enable_mutex);

	return 0;
}

static struct platform_driver cm3601_driver = {
	.probe          = cm3601_probe,
	.remove         = cm3601_remove,
	.suspend	= cm3601_suspend,
	.resume		= cm3601_resume,
	.driver		= {
		.name	= "cm3601",
		.owner	= THIS_MODULE,
	},
};

static int __init cm3601_init(void)
{
	return platform_driver_register(&cm3601_driver);
}

static void __exit cm3601_exit(void)
{
	platform_driver_unregister(&cm3601_driver);
}

MODULE_DESCRIPTION("Capella CM3601 ambient light sensor and proximity sensor");
MODULE_AUTHOR("JC Liao <jcliao@marvell.com>");
MODULE_LICENSE("GPL");

module_init(cm3601_init);
module_exit(cm3601_exit);
