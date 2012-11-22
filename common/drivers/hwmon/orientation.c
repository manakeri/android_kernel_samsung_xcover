/*
 * drivers/hwmon/orientation.c
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
#include <linux/init.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/slab.h>

/* for debugging */
#define DEBUG 0

#define ORIENTATION_NAME "orientation"
#define ORIENTATION_DEFAULT_DELAY            (200)	/* 200 ms */
#define ORIENTATION_MAX_DELAY                (2000)	/* 2000 ms */
#define ABS_STATUS                      (ABS_BRAKE)
#define ABS_WAKE                        (ABS_MISC)
#define ABS_CONTROL_REPORT              (ABS_THROTTLE)

struct orientation_data {
	struct mutex mutex;
	int enabled;
	int delay;
#if DEBUG
	int suspend;
#endif
};

static struct input_dev *this_data;

/* Sysfs interface */
static ssize_t
orientation_delay_show(struct device *dev,
		  struct device_attribute *attr, char *buf)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct orientation_data *data = input_get_drvdata(input_data);
	int delay;

	mutex_lock(&data->mutex);

	delay = data->delay;

	mutex_unlock(&data->mutex);

	return sprintf(buf, "%d\n", delay);
}

static ssize_t
orientation_delay_store(struct device *dev,
		   struct device_attribute *attr,
		   const char *buf, size_t count)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct orientation_data *data = input_get_drvdata(input_data);
	int value = simple_strtoul(buf, NULL, 10);

	if (value < 0)
		return count;

	if (ORIENTATION_MAX_DELAY < value)
		value = ORIENTATION_MAX_DELAY;

	mutex_lock(&data->mutex);

	data->delay = value;

	input_report_abs(input_data, ABS_CONTROL_REPORT,
			 (data->enabled << 16) | value);

	mutex_unlock(&data->mutex);

	return count;
}

static ssize_t
orientation_enable_show(struct device *dev,
		   struct device_attribute *attr, char *buf)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct orientation_data *data = input_get_drvdata(input_data);
	int enabled;

	mutex_lock(&data->mutex);

	enabled = data->enabled;

	mutex_unlock(&data->mutex);

	return sprintf(buf, "%d\n", enabled);
}

static ssize_t
orientation_enable_store(struct device *dev,
		    struct device_attribute *attr,
		    const char *buf, size_t count)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct orientation_data *data = input_get_drvdata(input_data);
	int value = simple_strtoul(buf, NULL, 10);

	if (value != 0 && value != 1)
		return count;

	mutex_lock(&data->mutex);

	data->enabled = value;

	input_report_abs(input_data, ABS_CONTROL_REPORT,
			 (value << 16) | data->delay);

	mutex_unlock(&data->mutex);

	return count;
}

static ssize_t
orientation_wake_store(struct device *dev,
		  struct device_attribute *attr,
		  const char *buf, size_t count)
{
	struct input_dev *input_data = to_input_dev(dev);
	static int cnt = 1;

	input_report_abs(input_data, ABS_WAKE, cnt++);

	return count;
}

static ssize_t
orientation_data_show(struct device *dev,
		 struct device_attribute *attr, char *buf)
{
	struct input_dev *input_data = to_input_dev(dev);
	unsigned long flags;
	int x, y, z;

	spin_lock_irqsave(&input_data->event_lock, flags);

	x = input_data->abs[ABS_X];
	y = input_data->abs[ABS_Y];
	z = input_data->abs[ABS_Z];

	spin_unlock_irqrestore(&input_data->event_lock, flags);

	return sprintf(buf, "%d %d %d\n", x, y, z);
}

static ssize_t
orientation_status_show(struct device *dev,
		   struct device_attribute *attr, char *buf)
{
	struct input_dev *input_data = to_input_dev(dev);
	unsigned long flags;
	int status;

	spin_lock_irqsave(&input_data->event_lock, flags);

	status = input_data->abs[ABS_STATUS];

	spin_unlock_irqrestore(&input_data->event_lock, flags);

	return sprintf(buf, "%d\n", status);
}

#if DEBUG

static ssize_t orientation_debug_suspend_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct orientation_data *data = input_get_drvdata(input_data);

	return sprintf(buf, "%d\n", data->suspend);
}

static ssize_t orientation_debug_suspend_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
	unsigned long value = simple_strtoul(buf, NULL, 10);

	return count;
}
#endif				/* DEBUG */

static DEVICE_ATTR(interval, S_IRUGO | S_IWUGO,
		   orientation_delay_show, orientation_delay_store);
static DEVICE_ATTR(active, S_IRUGO | S_IWUGO,
		   orientation_enable_show, orientation_enable_store);
static DEVICE_ATTR(wake, S_IWUGO, NULL, orientation_wake_store);
static DEVICE_ATTR(data, S_IRUGO, orientation_data_show, NULL);
static DEVICE_ATTR(status, S_IRUGO, orientation_status_show, NULL);

#if DEBUG
static DEVICE_ATTR(debug_suspend, S_IRUGO | S_IWUGO,
		   orientation_debug_suspend_show, orientation_debug_suspend_store);
#endif				/* DEBUG */

static struct attribute *orientation_attributes[] = {
	&dev_attr_interval.attr,
	&dev_attr_active.attr,
	&dev_attr_wake.attr,
	&dev_attr_data.attr,
	&dev_attr_status.attr,
#if DEBUG
	&dev_attr_debug_suspend.attr,
#endif				/* DEBUG */
	NULL
};

static struct attribute_group orientation_attribute_group = {
	.attrs = orientation_attributes
};

static int orientation_probe(struct platform_device *pdev)
{
	struct orientation_data *data = NULL;
	struct input_dev *input_data = NULL;
	int input_registered = 0, sysfs_created = 0;
	int rt;

	data = kzalloc(sizeof(struct orientation_data), GFP_KERNEL);
	if (!data) {
		rt = -ENOMEM;
		goto err;
	}

	data->enabled = 0;
	data->delay = ORIENTATION_DEFAULT_DELAY;

	input_data = input_allocate_device();
	if (!input_data) {
		rt = -ENOMEM;
		printk(KERN_ERR
		       "orientation_probe: Failed to allocate input_data device\n");
		goto err;
	}

	set_bit(EV_ABS, input_data->evbit);
	input_set_capability(input_data, EV_ABS, ABS_X);
	input_set_capability(input_data, EV_ABS, ABS_Y);
	input_set_capability(input_data, EV_ABS, ABS_Z);
	input_set_capability(input_data, EV_ABS, ABS_STATUS);	/* status */
	input_set_capability(input_data, EV_ABS, ABS_WAKE);	/* wake */
	input_set_capability(input_data, EV_ABS, ABS_CONTROL_REPORT);	/* enabled/delay */
	input_data->name = ORIENTATION_NAME;

	rt = input_register_device(input_data);
	if (rt) {
		printk(KERN_ERR
		       "orientation_probe: Unable to register input_data device: %s\n",
		       input_data->name);
		goto err;
	}
	input_set_drvdata(input_data, data);
	input_registered = 1;

	rt = sysfs_create_group(&input_data->dev.kobj,
				&orientation_attribute_group);
	if (rt) {
		printk(KERN_ERR
		       "orientation_probe: sysfs_create_group failed[%s]\n",
		       input_data->name);
		goto err;
	}
	sysfs_created = 1;
	mutex_init(&data->mutex);
	this_data = input_data;

	return 0;

err:
	if (data != NULL) {
		if (input_data != NULL) {
			if (sysfs_created)
				sysfs_remove_group(&input_data->dev.kobj,
						   &orientation_attribute_group);
			if (input_registered)
				input_unregister_device(input_data);
			else
				input_free_device(input_data);
			input_data = NULL;
		}
		kfree(data);
	}

	return rt;
}

static int orientation_remove(struct platform_device *pdev)
{
	struct orientation_data *data;

	if (this_data != NULL) {
		data = input_get_drvdata(this_data);
		sysfs_remove_group(&this_data->dev.kobj,
				   &orientation_attribute_group);
		input_unregister_device(this_data);
		if (data != NULL)
			kfree(data);
	}

	return 0;
}

/*
 * Module init and exit
 */
static struct platform_driver orientation_driver = {
	.probe = orientation_probe,
	.remove = orientation_remove,
	.suspend = NULL,
	.resume = NULL,
	.driver = {
		   .name = ORIENTATION_NAME,
		   .owner = THIS_MODULE,
		   },
};

static int __init orientation_init(void)
{
	return platform_driver_register(&orientation_driver);
}

module_init(orientation_init);

static void __exit orientation_exit(void)
{
	platform_driver_unregister(&orientation_driver);
}

module_exit(orientation_exit);

MODULE_AUTHOR("JC Liao <jcliao@marvell.com>");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0.0");
