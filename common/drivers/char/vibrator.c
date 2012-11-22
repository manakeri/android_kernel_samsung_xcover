/* Copyright (C) 2010 Marvell */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/d1982/pmic.h>


#include "../staging/android/timed_output.h"


#include <linux/delay.h>
#include <linux/d1982/core.h>
#include <linux/d1982/d1980_reg.h>

#define VIBRA_OFF_VALUE	0
#define VIBRA_ON_VALUE	1

static void vibrator_off_worker(struct work_struct *work)
{
    struct d1980_vibrator *vibrator = NULL;
    struct d1980 *d1980 = NULL;

	printk(KERN_INFO "[VIBRATOR] vibrator off worker\n");
    if(work == NULL)
    {
        printk(KERN_ERR "[VIBRATOR] work struct data is NULL\n");
        return;
    }
    vibrator = container_of(work, struct d1980_vibrator, vibrator_work);
    d1980 = container_of(vibrator, struct d1980, vibrator);
 
    d1980_reg_write(d1980, D1980_LDO17_REG, 0x2A);

    return;
}

static void on_vibrate_timer_expired(unsigned long x)
{
	struct d1980 *d1980 = NULL;
	struct d1980_vibrator *vibrator = NULL;

	printk(KERN_INFO "[VIBRATOR] on vibrate timer expired \n");
    if(!x)
    {
        printk(KERN_ERR "[VIBRATOR] Timer data is NULL\n");
        return ;
    }
    d1980 = (struct d1980 *)x;
    vibrator = &(d1980->vibrator);
	schedule_work(&(vibrator->vibrator_work));

    return ;
}


static void vibrator_enable_set_timeout(struct timed_output_dev *sdev, 	int timeout)
{
	struct d1980_vibrator *vibrator = NULL;
	struct d1980 *d1980 = NULL;
	int retTime;

    if(sdev == NULL)
    {
        printk(KERN_ERR "[VIBRATOR] timed output device is NULL\n");
        return ;
    }
    vibrator = container_of(sdev, struct d1980_vibrator, vibrator_timed_dev);
    d1980 = container_of(vibrator, struct d1980, vibrator);
    
	printk(KERN_INFO "[VIBRATOR] Set duration: %dms\n", timeout);
	if(timeout == 0 )
	{	
#if 0   
       schedule_work(&vibrator->vibrator_work);
#else/* If timeout equals 0 we just stop the vibrator */
       d1980_reg_write(d1980, D1980_LDO17_REG, 0x2A); 
#endif /* #if 0 */
		return;

	}
	else{	
		if(timeout < 100)
			timeout= timeout*2;
	d1980_reg_write(d1980, D1980_LDO17_REG, 0x6A);
        
	mod_timer(&vibrator->vibrator_timer, jiffies + msecs_to_jiffies(timeout));
	}
	return;
}


static int vibrator_get_remaining_time(struct timed_output_dev *sdev)
{
	struct d1980_vibrator *vibrator = NULL;
	struct d1980 *d1980 = NULL;
	int retTime = 0;

    if(sdev == NULL)
    {
        printk(KERN_ERR "timed output device data is NULL\n");
        return retTime;
    }
    vibrator = container_of(sdev, struct d1980_vibrator, vibrator_timed_dev);
    d1980 = container_of(vibrator, struct d1980, vibrator);
    retTime = jiffies_to_msecs(jiffies- vibrator->vibrator_timer.expires);
    
	printk(KERN_INFO "Vibrator: ramaining time : %dms\n", retTime);

	return retTime;
}

static int vibrator_probe(struct platform_device *pdev)
{
	struct d1980 *d1980 = NULL;;
	struct d1980_vibrator *vibrator = NULL;;
	int ret = 0;

    printk(KERN_INFO "VIBRATOR probe. Starting...\n");
    if(pdev == NULL)
    {
        printk(KERN_ERR "[VIBRATOR] platform device data is NULL\n");
        goto pdev_err;
    }
    d1980 = platform_get_drvdata(pdev);
    vibrator = &d1980->vibrator;

    vibrator->vibrator_timed_dev.name = "vibrator";
    vibrator->vibrator_timed_dev.enable = vibrator_enable_set_timeout;
    vibrator->vibrator_timed_dev.get_time = vibrator_get_remaining_time;

    ret = timed_output_dev_register(&vibrator->vibrator_timed_dev);
	if (ret < 0) {
		printk(KERN_ERR "[VIBRATOR] timed_output device registration failure\n");
		goto timed_reg_error;
	}

	init_timer(&vibrator->vibrator_timer);
	vibrator->vibrator_timer.function = on_vibrate_timer_expired;
	vibrator->vibrator_timer.data = (unsigned long)d1980;
	INIT_WORK(&vibrator->vibrator_work, vibrator_off_worker);
    printk(KERN_INFO "VIBRATOR probe. Ending...\n");

	return 0;

pdev_err:
timed_reg_error:
    return ret;
}

static int __devexit vibrator_remove(struct platform_device *pdev)
{
    struct d1980 *d1980 = platform_get_drvdata(pdev);
    struct d1980_vibrator *vibrator = &d1980->vibrator;
    
    timed_output_dev_unregister(&vibrator->vibrator_timed_dev);

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
