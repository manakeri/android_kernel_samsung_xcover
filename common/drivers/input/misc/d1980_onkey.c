/*
 * d1980_onkey.c: ON Key support for Dialog D1980
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
#include <linux/init.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
 
#include <linux/d1982/d1980_reg.h> 
#include <linux/d1982/core.h>

#define DRIVER_NAME         "d1980-onkey"
#define ONKEY_CHECK_TIME    100


static u8 gOnKeyState = 0;   // o : released, 1 : pressed

u8 get_Onkey_State(void)
{
    return gOnKeyState;
}
EXPORT_SYMBOL(get_Onkey_State);


static void onkey_check_worker(struct work_struct *work)
{
	u8  onKeyStatus;
	struct d1980_onkey *dlg_onkey = container_of(work, struct d1980_onkey, onkey_work);
	struct d1980 *d1980 = container_of(dlg_onkey, struct d1980, onkey);

	mutex_lock(&dlg_onkey->onkey_mutex);
	onKeyStatus = (d1980_reg_read(d1980, D1980_STATUSA_REG) & D1980_STATUSA_NONKEY);
	onKeyStatus = onKeyStatus ? 0 : 1;
	gOnKeyState = onKeyStatus;
	mutex_unlock(&dlg_onkey->onkey_mutex);
    //dev_info(d1980->dev, "OnKey Interrupt event generated. onkey status = [%d]\n", onKeyStatus);
	input_report_key(dlg_onkey->input, KEY_POWER, onKeyStatus);
	input_sync(dlg_onkey->input);

	if(onKeyStatus)
        mod_timer(&dlg_onkey->onkey_timer, jiffies + msecs_to_jiffies(ONKEY_CHECK_TIME));
	else
	    del_timer(&dlg_onkey->onkey_timer);

}


static void onkey_timer_expired(unsigned long data)
{
	u8  onKeyStatus;
    struct d1980 *d1980 = (struct d1980 *)data;
    struct d1980_onkey *onkey = &d1980->onkey;

	schedule_work(&onkey->onkey_work);
}


static irqreturn_t d1980_onkey_event_handler(int irq, void *data)
{
    struct d1980 *d1980 = data;
    struct d1980_onkey *dlg_onkey = &d1980->onkey;
    u8  onKeyStatus;
    int i;

#if 1
    mutex_lock(&dlg_onkey->onkey_mutex);
    onKeyStatus = (d1980_reg_read(d1980, D1980_STATUSA_REG) & D1980_STATUSA_NONKEY);
    mutex_unlock(&dlg_onkey->onkey_mutex);
	onKeyStatus = onKeyStatus ? 0 : 1;
    //dev_info(d1980->dev, "OnKey Interrupt event generated. onkey status = [%d]\n", onKeyStatus);
    input_report_key(dlg_onkey->input, KEY_POWER, onKeyStatus);
    input_sync(dlg_onkey->input);

    mod_timer(&dlg_onkey->onkey_timer, jiffies + msecs_to_jiffies(ONKEY_CHECK_TIME));
#else
    dev_info(d1980->dev, "Onkey Interrupt Event generated\n");

	input_report_key(dlg_onkey->input, KEY_POWER, 1);
	input_sync(dlg_onkey->input);
	mutex_lock(&dlg_onkey->onkey_mutex);
	for(i = 0 ; i < 24 ; i++) {
		mdelay(50);
		if(1 == (d1980_reg_read(d1980, D1980_STATUSA_REG) & D1980_STATUSA_NONKEY))
			break;
		dev_info(d1980->dev, "The Onkey loop with the state of i = [%d]\n", i);
	}
	mutex_unlock(&dlg_onkey->onkey_mutex);
	msleep(2); 
	input_report_key(dlg_onkey->input, KEY_POWER, 0);
	input_sync(dlg_onkey->input);
#endif

	return IRQ_HANDLED;
} 

static int __devinit d1980_onkey_probe(struct platform_device *pdev)
{
	struct d1980 *d1980 = platform_get_drvdata(pdev);
	struct d1980_onkey *dlg_onkey = &d1980->onkey;
	int ret = 0;
	

    dev_info(d1980->dev, "Starting Onkey Driver\n");

    dlg_onkey->input = input_allocate_device();
    if (!dlg_onkey->input) {
    	dev_err(&pdev->dev, "failed to allocate data device\n");
    	return -ENOMEM;
	}
        
	dlg_onkey->input->evbit[0] = BIT_MASK(EV_KEY);
	dlg_onkey->input->keybit[BIT_WORD(KEY_POWER)] = BIT_MASK(KEY_POWER);
	dlg_onkey->input->name = DRIVER_NAME;
	dlg_onkey->input->phys = "d1980-onkey/input0";
	dlg_onkey->input->dev.parent = &pdev->dev;

	ret = input_register_device(dlg_onkey->input);
	if (ret) {
		dev_err(&pdev->dev, "Unable to register input\
				device,error: %d\n", ret);
		input_free_device(dlg_onkey->input);
		return ret;
	}

    d1980_register_irq(d1980, D1980_IRQ_ENONKEY, d1980_onkey_event_handler, 
                            0, DRIVER_NAME, d1980);
	//spin_lock_init(&dlg_onkey->onkey_mutex);
	mutex_init(&dlg_onkey->onkey_mutex);

	init_timer(&dlg_onkey->onkey_timer);
	dlg_onkey->onkey_timer.function = onkey_timer_expired;
	dlg_onkey->onkey_timer.data = (unsigned long)d1980;
	INIT_WORK(&dlg_onkey->onkey_work, onkey_check_worker);

	dev_info(d1980->dev, "Onkey Driver registered\n");
	return 0;

}


#ifdef CONFIG_PM
static int d1980_onkey_suspend(struct platform_device *pdev)
{
	struct d1980 *d1980 = platform_get_drvdata(pdev);
	struct d1980_onkey *dlg_onkey = &d1980->onkey;

	if(device_may_wakeup(pdev)) {
	    enable_irq_wake(d1980->chip_irq);
	}
	return 0;
}

static int d1980_onkey_resume(struct platform_device *pdev)
{
	struct d1980 *d1980 = platform_get_drvdata(pdev);
	struct d1980_onkey *dlg_onkey = &d1980->onkey;

    if(device_may_wakeup(pdev)) {
        disable_irq_wake(d1980->chip_irq);
    }

	return 0;
}

static struct dev_pm_ops d1980_onkey_pm_ops = {
	.suspend	= d1980_onkey_suspend,
	.resume		= d1980_onkey_resume,
};
#endif


static int __devexit d1980_onkey_remove(struct platform_device *pdev)
{
	struct d1980 *d1980 = platform_get_drvdata(pdev);
	struct d1980_onkey *dlg_onkey = &d1980->onkey;

	d1980_free_irq(d1980, D1980_IRQ_ENONKEY);
	input_unregister_device(dlg_onkey->input);
	return 0;
}   

static struct platform_driver d1980_onkey_driver = {
	.probe		= d1980_onkey_probe,
	.remove		= __devexit_p(d1980_onkey_remove),
	.driver		= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
#ifdef CONFIG_PM
		.pm	= &d1980_onkey_pm_ops,
#endif
	}
};

static int __init d1980_onkey_init(void)
{
	return platform_driver_register(&d1980_onkey_driver);
}

static void __exit d1980_onkey_exit(void)
{
	platform_driver_unregister(&d1980_onkey_driver);
}

module_init(d1980_onkey_init);
module_exit(d1980_onkey_exit);   

MODULE_AUTHOR("Dialog Semiconductor Ltd <divyang.patel@diasemi.com>");
MODULE_DESCRIPTION("Onkey driver for the Dialog D1980 PMIC");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRIVER_NAME);
