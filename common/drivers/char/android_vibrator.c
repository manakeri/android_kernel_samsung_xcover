/* Copyright (C) 2010 Marvell */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/mfd/88pm860x.h>
#include <linux/slab.h>
#include <mach/mfp.h>
#include <mach/gpio.h>
#include "../staging/android/timed_output.h"

struct vibrator_info {
	struct pm860x_chip	*chip;
	struct i2c_client	*i2c;
	struct timed_output_dev vibrator_timed_dev;
	struct timer_list	vibrate_timer;
	struct work_struct	vibrator_off_work;
	unsigned char vibratorA;/*PM8606_VIBRATORA register offset*/
	unsigned char vibratorB;/*PM8606_VIBRATORB register offset*/
};

#define VIBRA_OFF_VALUE	0
#define VIBRA_ON_VALUE	1

int motor_gpio=  mfp_to_gpio(MFP_PIN_GPIO126);

int pm860x_control_vibrator(struct pm860x_chip *chip, struct i2c_client *i2c,
		unsigned char value)
{

	if (value == VIBRA_OFF_VALUE) 
	{
			printk("vibrator off \n");
			gpio_direction_output(motor_gpio,VIBRA_OFF_VALUE);
 
	} 
	else if (value == VIBRA_ON_VALUE)
	{
			printk("vibrator on \n");
			gpio_direction_output(motor_gpio,VIBRA_ON_VALUE);
	}

	return 0;
}

static void vibrator_off_worker(struct work_struct *work)
{
	struct vibrator_info *info;
	info = container_of(work, struct vibrator_info, vibrator_off_work);
	printk(" vibrator_off_worker \n");
	gpio_set_value(motor_gpio,0);
}

static void on_vibrate_timer_expired(unsigned long x)
{
	struct vibrator_info *info;
	info = (struct vibrator_info*)x;
	printk(" timer expired \n");
	schedule_work(&info->vibrator_off_work);
}

static void vibrator_enable_set_timeout(struct timed_output_dev *sdev,
	int timeout)
{
	struct vibrator_info *info;
	info = container_of(sdev, struct vibrator_info, vibrator_timed_dev);
	printk(KERN_NOTICE "Vibrator: Set duration: %dms\n", timeout);
	if(timeout==0)
	{	
		gpio_set_value(motor_gpio,0);
		return;
	}
	else{
	if(timeout < 100)
		timeout = timeout*2;
	gpio_direction_output(motor_gpio,1);
	gpio_set_value(motor_gpio,1);
//	pm860x_control_vibrator(info->chip, info->i2c, 1);
	mod_timer(&info->vibrate_timer, jiffies + msecs_to_jiffies(timeout));
	}
	return;
}

static int vibrator_get_remaining_time(struct timed_output_dev *sdev)
{
	struct vibrator_info *info;
	int retTime;
	info = container_of(sdev, struct vibrator_info, vibrator_timed_dev);
	retTime = jiffies_to_msecs(jiffies-info->vibrate_timer.expires);
	printk(KERN_NOTICE "Vibrator: Current duration: %dms\n", retTime);
	return retTime;
}

static int vibrator_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct resource *res;
	struct pm860x_chip *chip = dev_get_drvdata(pdev->dev.parent);
	struct vibrator_info *info = kzalloc(sizeof(struct vibrator_info), GFP_KERNEL);
	if (!info)
			return -ENOMEM;
	res = platform_get_resource(pdev, IORESOURCE_IO, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "No I/O resource!\n");
		kfree(info);
		return -EINVAL;
	}
	info->vibratorA = res->start;
	info->vibratorB = res->end;
	info->chip = chip;
#if defined(CONFIG_MACH_GFORCE) || defined(CONFIG_MACH_ALKON) || defined(CONFIG_MACH_JETTA)
	// +SAMSUNG_SSENI : The below code is not for GFORCE board. This should be checked later.
	//info->i2c = (chip->id == CHIP_PM8606) ? chip->client : chip->companion;
	info->i2c = chip->client;
	// -SAMSUNG_SSENI : The below code is not for GFORCE board. This should be checked later.
#else
	info->i2c = (chip->id == CHIP_PM8606) ? chip->client : chip->companion;
#endif

	/* Setup timed_output obj */
	info->vibrator_timed_dev.name = "vibrator";
	info->vibrator_timed_dev.enable = vibrator_enable_set_timeout;
	info->vibrator_timed_dev.get_time = vibrator_get_remaining_time;
	/* Vibrator dev register in /sys/class/timed_output/ */
	ret = timed_output_dev_register(&info->vibrator_timed_dev);
	if (ret < 0) {
		printk(KERN_ERR "Vibrator: timed_output dev registration failure\n");
		timed_output_dev_unregister(&info->vibrator_timed_dev);
	}

	INIT_WORK(&info->vibrator_off_work, vibrator_off_worker);

	init_timer(&info->vibrate_timer);
	info->vibrate_timer.function = on_vibrate_timer_expired;
	info->vibrate_timer.data = (unsigned long)info;

	platform_set_drvdata(pdev, info);

	if (gpio_request(motor_gpio, "motor enable pin"))
	printk(KERN_ERR "Request GPIO_%d failed!\n", motor_gpio);

	return 0;
}

static int __devexit vibrator_remove(struct platform_device *pdev)
{
	struct vibrator_info *info;
	info = platform_get_drvdata(pdev);

	gpio_free(motor_gpio);
	timed_output_dev_unregister(&info->vibrator_timed_dev);
	return 0;
}

static struct platform_driver vibrator_driver = {
	.probe		= vibrator_probe,
	.remove		= __devexit_p(vibrator_remove),
	.driver		= {
		.name	= "android-vibrator",
		.owner	= THIS_MODULE,
	},
};

static int __init vibrator_init(void)
{
	return platform_driver_register(&vibrator_driver);
}

static void __exit vibrator_exit(void)
{
	platform_driver_unregister(&vibrator_driver);
}

module_init(vibrator_init);
module_exit(vibrator_exit);

MODULE_DESCRIPTION("Android Vibrator driver");
MODULE_LICENSE("GPL");
