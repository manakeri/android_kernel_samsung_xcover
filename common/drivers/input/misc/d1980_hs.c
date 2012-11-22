/*
 * d1980_hs.c: Hs Detect support for Dialog D1980
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
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/kdev_t.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/uio.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/miscdevice.h>
#include <linux/kthread.h>
#include <linux/freezer.h>
#include <mach/hardware.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/mman.h>
#include <asm/uaccess.h>
#include <linux/switch.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <mach/gpio.h>
#include <mach/mfp-pxa9xx.h>
#include <linux/d1982/d1980_reg.h> 
#include <linux/d1982/core.h>
#include <mach/d1980_hs.h>
#include <mach/d1980_hs.h>

#define HS_DETECT_REV0203_TEMP

#define D1981_DEBUG 1
#if D1981_DEBUG
#define dbg(format, arg...) printk(KERN_INFO "d1981: " format "\n", ## arg)
#else
#define dbg(format, arg...)
#endif

#define DRIVER_NAME "d1980-hsdetect"
#define HS_HOOK_CHECK_TIME 60
#define HS_CHECK_TIME 100
#define HS_HSDETET_COUNT 2
#define HS_HOOKSWITCH_TOKEN_COUNT 4

#define ADC_4POLE_MAX 860 //840
#define ADC_4POLE_MIN 620 //540 
#define ADC_3POLE_MAX 875 //841
#define ADC_3POLE_MIN 539

extern int d1981_mic_bias(unsigned char on);
extern int d1980_adc_in_read_auto(void);
extern u8 d1980_set_adcIn_enable(u8 on);
extern u8 d1980_set_nSleep_enable(u8 on);
extern int d1981LowPowerHookDetection(u8 on);
extern int d1981_hs_ready(void);

static struct d1980_hs *g_dlg_hsdetect;
unsigned char g_dlg_4pole_detect=0;
extern char g_d1981_power_state;
struct sec_jack_buttons_zone {
	unsigned int code;
	unsigned int adc_low;
	unsigned int adc_high;
};

/* To support 3-buttons earjack */
static struct sec_jack_buttons_zone sec_jack_buttons_zones[] = {
	{
		.code		= KEY_MEDIA,
		.adc_low		= 1000,//995,
		.adc_high	= 1030,
	},
	{
		.code		= KEY_VOLUMEUP,
		.adc_low		= 971,//950,
		.adc_high	= 999,//996,
	},
	{
		.code		= KEY_VOLUMEDOWN,
		.adc_low		= 900,
		.adc_high	= 970,//949,
	},
};

void d1980_hookswitch_polling_start(void)
{
	   
    del_timer(&g_dlg_hsdetect->hookswitch_timer);
    mod_timer(&g_dlg_hsdetect->hookswitch_timer, jiffies + msecs_to_jiffies(HS_CHECK_TIME));
}

void d1980_hookswitch_polling_stop(void)
{

    del_timer(&g_dlg_hsdetect->hookswitch_timer);
}


static void d1980_hookswitch_polling_worker(struct work_struct *work)
{
   struct d1980_hs *dlg_hsdetect = container_of(work, struct d1980_hs, hookswitch_work);
   struct d1980 *d1980 = container_of(dlg_hsdetect, struct d1980, hsdetect); 
    u8 val;

    int value=0;
	int v_vf_adc=0,v_vf_adc2=0;
	int i; 

    if(g_d1981_power_state==0) //using interrupt in case of low power mode
    {

         if(dlg_hsdetect->hookswitch_status==D1980_HOOKSWITCH_PRESSED)
         {
             val=d1981_get_hook_state();   
             
             if(val==1)
             {        
                
                input_report_key(dlg_hsdetect->input, dlg_hsdetect->pressed_code, 0);
        		input_sync(dlg_hsdetect->input);
        		dlg_hsdetect->hookswitch_status=D1980_HOOKSWITCH_RELEASED;
        		switch_set_state(&dlg_hsdetect->sdev, 0);

            }   
            else if(val==0)
            {           
                mod_timer(&dlg_hsdetect->hookswitch_timer, jiffies + msecs_to_jiffies(HS_HOOK_CHECK_TIME));
                return;
            }
        }
    }
    else
    {
     
         val=d1981_get_hook_state();
    
		if(val==1)
		{
			dlg_hsdetect->hookswitch_token_count=0;
		}
         if((val==1) && (dlg_hsdetect->hookswitch_status==D1980_HOOKSWITCH_PRESSED)) //released
         {        
            input_report_key(dlg_hsdetect->input, dlg_hsdetect->pressed_code, 0);
    		input_sync(dlg_hsdetect->input);

            dlg_hsdetect->hookswitch_status=D1980_HOOKSWITCH_RELEASED; 
            switch_set_state(&dlg_hsdetect->sdev, 0);   
            dbg( "------->d1980_hk released  !!!!!! \n");

        }   
        else if((val==0) && (dlg_hsdetect->hookswitch_status==D1980_HOOKSWITCH_RELEASED)) //pressed
        {

            if(g_dlg_4pole_detect==1)
            {
				if(val==0 && dlg_hsdetect->hookswitch_token_count < HS_HOOKSWITCH_TOKEN_COUNT)
				{
					dlg_hsdetect->hookswitch_token_count++;
					mod_timer(&dlg_hsdetect->hookswitch_timer, jiffies + msecs_to_jiffies(HS_HOOK_CHECK_TIME));
					return;
				}
               d1980_set_adcIn_enable(1);
               msleep(2);
               v_vf_adc=d1980_adc_in_read_auto();

                v_vf_adc2=dlg_hsdetect->pre_hsdetect_status - v_vf_adc;

                if(v_vf_adc2 < -2 || v_vf_adc2 > 2)
                {
                    dlg_hsdetect->pre_hsdetect_status=v_vf_adc;

                    dbg( "ADC read again !!!!  v_vf_adc = %d \n",v_vf_adc);
                    mod_timer(&dlg_hsdetect->hookswitch_timer, jiffies + msecs_to_jiffies(5));
                    return;
                }


                dbg( "ADC VALUE !!!!  v_vf_adc = %d \n",v_vf_adc);

                printk("SSONG HOOK SWITCH ADC %d \n",v_vf_adc);
                dlg_hsdetect->hookswitch_status=D1980_HOOKSWITCH_PRESSED; //PRESSED

    		    for (i = 0; i < 3; i++)
    		    {
        			if (v_vf_adc >= sec_jack_buttons_zones[i].adc_low && v_vf_adc <= sec_jack_buttons_zones[i].adc_high) 
        			{
        				dlg_hsdetect->pressed_code = sec_jack_buttons_zones[i].code;
        				input_report_key(dlg_hsdetect->input, sec_jack_buttons_zones[i].code, 1);
        				input_sync(dlg_hsdetect->input);
        				pr_info("%s: keycode=%d, is pressed\n", __func__,sec_jack_buttons_zones[i].code);
        			}	
                }
                switch_set_state(&dlg_hsdetect->sdev, 1); 
                dbg( "------->d1980_hk pressedd  !!!!!! \n"); 
                d1980_set_adcIn_enable(0);
                dlg_hsdetect->pre_hsdetect_status=0;
           }  
                  

        }
        mod_timer(&dlg_hsdetect->hookswitch_timer, jiffies + msecs_to_jiffies(HS_HOOK_CHECK_TIME));//test

    }
}
static void d1980_hookswitch_timer(unsigned long data)
{
    struct d1980 *d1980 = (struct d1980 *)data;
    struct d1980_hs *dlg_hsdetect = &d1980->hsdetect;

    if(g_dlg_4pole_detect==1)
    schedule_work(&dlg_hsdetect->hookswitch_work);
}


static irqreturn_t d1980_hookswitch_interrupt_handler(int irq, void *data)
{   
        struct d1980 *d1980 = data;
        struct d1980_hs *dlg_hsdetect = &d1980->hsdetect;
        u8 val = 0;
	int v_vf_adc=0;
	int i;

	dbg("------->d1980_hookswitch_interrupt_handler, g_d1981_power_state[%d] \n", g_d1981_power_state);

        if(g_d1981_power_state ==1) //using polling in case of normal mode
            return IRQ_HANDLED;
            
        del_timer(&dlg_hsdetect->hookswitch_timer);
        
	if(dlg_hsdetect->hookswitch_status==D1980_HOOKSWITCH_PRESSED) // was pressed -> now released
        {
            input_report_key(dlg_hsdetect->input, dlg_hsdetect->pressed_code, 0);
    		input_sync(dlg_hsdetect->input);

            dlg_hsdetect->hookswitch_status=D1980_HOOKSWITCH_RELEASED; 
            switch_set_state(&dlg_hsdetect->sdev, 0);
            dbg( "------->d1980_hk released  !!!!!! \n");  
        }   
	else // was released -> now pressed
	{
		if(g_dlg_4pole_detect==1)
		{
			dbg("g_d1981_power_state [%d]\n", g_d1981_power_state);		// check LYNX power-state

			// nSleep disable
			d1980_set_nSleep_enable(0);
			msleep(10);

			//LYNX power on (if already power on, return)
			d1981LowPowerHookDetection(1);
			msleep(2);

			// ADC enable
			d1980_set_adcIn_enable(1);
			msleep(2);

			// ADC read
			v_vf_adc=d1980_adc_in_read_auto();
			msleep(10);

			printk(">>>>>>ADC value(in INT method) [%d] \n", v_vf_adc);

			for (i = 0; i < 3; i++)
			{
				if (v_vf_adc >= sec_jack_buttons_zones[i].adc_low && v_vf_adc <= sec_jack_buttons_zones[i].adc_high)
				{
					dlg_hsdetect->hookswitch_status=D1980_HOOKSWITCH_PRESSED;
					dlg_hsdetect->pressed_code = sec_jack_buttons_zones[i].code;
					input_report_key(dlg_hsdetect->input, sec_jack_buttons_zones[i].code, 1);
					input_sync(dlg_hsdetect->input);
					pr_info("%s: keycode=%d, is pressed\n", __func__,sec_jack_buttons_zones[i].code);
       
					switch_set_state(&dlg_hsdetect->sdev, 1);
					dbg( "------->d1980_hk pressedd  !!!!!! \n");
        			}
			}
#if 0
			// ADC disable
			d1980_set_adcIn_enable(0);
			msleep(2);

			// LYNX -> Low power mode
			d1981LowPowerHookDetection(0);
			msleep(2);

			// nSleep enable
			d1980_set_nSleep_enable(1);
#endif			
           	}
		else
		{
        dlg_hsdetect->pressed_code = KEY_MEDIA;
		input_report_key(dlg_hsdetect->input, dlg_hsdetect->pressed_code, 1);
		input_sync(dlg_hsdetect->input);

        dlg_hsdetect->hookswitch_status=D1980_HOOKSWITCH_PRESSED; //PRESSED
        switch_set_state(&dlg_hsdetect->sdev, 1);              
        dbg( "------->d1980_hk pressedd  !!!!!! \n");  
		}
	}

    if (!timer_pending(&dlg_hsdetect->hookswitch_timer)) 
    {
        mod_timer(&dlg_hsdetect->hookswitch_timer, jiffies + msecs_to_jiffies(HS_CHECK_TIME));
    }
	
	dbg( "d1980_hookswitch_interrupt_handler ---> END\n");
    return IRQ_HANDLED;
} 

static void d1980_hs_gpio_worker(struct work_struct *work)
{
   struct d1980_hs *dlg_hsdetect = container_of(work, struct d1980_hs, hsdetect_work);
   struct d1980 *d1980 = container_of(dlg_hsdetect, struct d1980, hsdetect); 
   int ready=0;
    int status;
#if defined(HS_DETECT_REV0203_TEMP)
    int status2;
	int status3;
#endif
    u8 val;

//    dbg( "d1980_hs_gpio_worker d1980 addr=0x%x dlg_hsdetect=0x%x \n",d1980,dlg_hsdetect);   

#if defined(HS_DETECT_REV0203_TEMP)
    status3 = gpio_get_value(MFP_PIN_GPIO79);
    status3 &= (1 << (MFP_PIN_GPIO79%32));
    status3 = !!status3;
    status2 = gpio_get_value(MFP_PIN_GPIO133);
    status2 &= (1 << (MFP_PIN_GPIO133%32));
    status2 = !!status2;

	if(!(status2&status3))
	{
		status=status3^status2;
	}
	else
	{
		// wrong status, just return 
		printk("%s : wring signal\n", __func__);
		return;
	}
	printk("%s : status=%d, status2=%d, status3=%d\n", __func__, status, status2, status3);
#else
    status = gpio_get_value(MFP_PIN_GPIO79);
    status &= (1 << (MFP_PIN_GPIO79%32));
    status = !!status;
#endif

     if(status==1) //insert
     {   
           int v_vf_adc=0,v_vf_adc2=0;
            

            ready=d1981_hs_ready();
            if(ready==-1)
            {
                mod_timer(&dlg_hsdetect->hsdetect_timer, jiffies + msecs_to_jiffies(HS_CHECK_TIME));
                return ;
            }
            d1981_mic_bias(1);
            d1980_set_adcIn_enable(1); //adc on
            
            v_vf_adc=d1980_adc_in_read_auto();

            v_vf_adc2=dlg_hsdetect->pre_hsdetect_status - v_vf_adc;
            
            if(v_vf_adc2 < -2 || v_vf_adc2 > 2)
            {
                dlg_hsdetect->pre_hsdetect_status=v_vf_adc;
                
                dbg( "Not complete insert !!!!  v_vf_adc = %d \n",v_vf_adc);
                mod_timer(&dlg_hsdetect->hsdetect_timer, jiffies + msecs_to_jiffies(20));
                return;
            }
         

            dbg( "ADC VALUE !!!!  v_vf_adc = %d \n",v_vf_adc);
            
            if( v_vf_adc < ADC_4POLE_MAX && ADC_4POLE_MIN< v_vf_adc) // 540 < 4 pole < 840
            {
                g_dlg_4pole_detect=1;
                
                d1980_hookswitch_polling_start();
                 
                dbg( "------->4Pole Detected ~~~~~~~~~~~~~~~ \n");
                dlg_hsdetect->pre_hsdetect_status=0;
            }
            else if( v_vf_adc < ADC_3POLE_MIN || ADC_3POLE_MAX < v_vf_adc) // 3 pole <539  841 < 3pole
            {
                g_dlg_4pole_detect=0;
                d1981_mic_bias(0);
                dlg_hsdetect->pre_hsdetect_status=0;
            }
            else // invalid value
            {
                 mod_timer(&dlg_hsdetect->hsdetect_timer, jiffies + msecs_to_jiffies(HS_CHECK_TIME));
                 return;

            }
            
            ready=d1981_hs_handler(&dlg_hsdetect->hs_dev->kobj, D1980_HEADSET_ADD); //to do

            d1980_set_adcIn_enable(0);
            dbg( "------->d1980_hs_gpio_worker headset Added  !!!!!! \n");


        }
        else
        {   
            //d1980_reg_write(d1980,D1980_GPIO0001_REG,0xe8); //active low for insert     
   
            del_timer(&dlg_hsdetect->hookswitch_timer);
            
            if(dlg_hsdetect->hookswitch_status==D1980_HOOKSWITCH_PRESSED)
            {
                input_report_key(dlg_hsdetect->input, dlg_hsdetect->pressed_code, 0);
        		input_sync(dlg_hsdetect->input);
        		dlg_hsdetect->hookswitch_status=D1980_HOOKSWITCH_RELEASED;
        		switch_set_state(&dlg_hsdetect->sdev, 0);
            }
            
            d1981_hs_handler(&dlg_hsdetect->hs_dev->kobj, D1980_HEADSET_REMOVE);  
            g_dlg_4pole_detect=0;
            d1980_set_adcIn_enable(0);
            d1981_mic_bias(0);    
            dbg( "->d1980_hs_gpio_worker un-plug period \n");


         }

}

static void d1980_hs_timer(unsigned long data)
{
    struct d1980 *d1980 = (struct d1980 *)data;
    struct d1980_hs *dlg_hsdetect = &d1980->hsdetect;
    
    u8 val;

    schedule_work(&dlg_hsdetect->hsdetect_work);
}
static irqreturn_t d1980_hs_interrupt_gpio_handler(int irq, void *data)
{
	struct d1980 *d1980 = data;
	struct d1980_hs *dlg_hsdetect = &d1980->hsdetect;
       int mic_exist=0;
        int status;

#if defined(HS_DETECT_REV0203_TEMP)
    int status2;
    int status3;
#endif       
            
   d1981_mic_bias(0);
    
       del_timer(&dlg_hsdetect->hookswitch_timer);
       del_timer(&dlg_hsdetect->hsdetect_timer);
        
#if defined(HS_DETECT_REV0203_TEMP)
    status3 = gpio_get_value(MFP_PIN_GPIO79);
    status3 &= (1 << (MFP_PIN_GPIO79%32));
    status3 = !!status3;
    status2 = gpio_get_value(MFP_PIN_GPIO133);
    status2 &= (1 << (MFP_PIN_GPIO133%32));
    status2 = !!status2;
	if(!(status2&status3))
	{
		status=status3^status2;
	}
	else
	{
		// wrong status, just return 
		printk("%s : wring signal\n", __func__);
		return;
	}
	printk("%s : status=%d, status2=%d, status3=%d\n", __func__, status, status2, status3);
#else
    	status = gpio_get_value(MFP_PIN_GPIO79);
    	status &= (1 << (MFP_PIN_GPIO79%32));
    	status = !!status;
#endif	
	
	   dlg_hsdetect->pre_hsdetect_status=0;
	    
       if(status==1) //insert
       {
            dlg_hsdetect->hsdetect_status=1;
       }
       else
       {
            dlg_hsdetect->hsdetect_status=0;
       }

       mod_timer(&dlg_hsdetect->hsdetect_timer, jiffies + msecs_to_jiffies(HS_CHECK_TIME));

	return IRQ_HANDLED;
} 

static ssize_t print_button_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%s\n", "h3w");
}

static ssize_t print_button_state(struct switch_dev *sdev, char *buf)
{
       struct d1980_hs	*dlg_hsdetect =
		container_of(sdev, struct d1980_hs, sdev);
		
	return sprintf(buf, "%s\n", (dlg_hsdetect->hookswitch_status ? "1" : "0"));
}

static int __devinit d1980_hs_probe(struct platform_device *pdev)
{
	struct d1980 *d1980 = platform_get_drvdata(pdev);
	struct d1980_hs *dlg_hsdetect = &d1980->hsdetect;
	int ret = 0;
	u8 val;
	
	
       dbg( "Starting hsdetect Driver\n");
       dlg_hsdetect->hookswitch_status=0;

       dlg_hsdetect->sdev.name            = "h3w";
	dlg_hsdetect->sdev.print_name      = print_button_name;
	dlg_hsdetect->sdev.print_state     = print_button_state;

	ret = switch_dev_register(&dlg_hsdetect->sdev); 
	if (ret < 0)
		return ret;
		
        dlg_hsdetect->hs_dev = &pdev->dev;      
			 
       init_timer(&dlg_hsdetect->hsdetect_timer);
	dlg_hsdetect->hsdetect_timer.function = d1980_hs_timer;
	dlg_hsdetect->hsdetect_timer.data = (unsigned long) d1980;

	init_timer(&dlg_hsdetect->hookswitch_timer);
	dlg_hsdetect->hookswitch_timer.function = d1980_hookswitch_timer;
	dlg_hsdetect->hookswitch_timer.data = (unsigned long) d1980;

    dlg_hsdetect->input = input_allocate_device();
	if (dlg_hsdetect->input == NULL) {
		ret = -ENOMEM;
		pr_err("%s : Failed to allocate input device.\n", __func__);
		goto err_register_input_dev;
	}
	dlg_hsdetect->input->name = "sec_jack";
	dlg_hsdetect->input->phys = "headset/input0";
	

      set_bit(EV_KEY, dlg_hsdetect->input->evbit);


	/* define what keys are available */
	input_set_capability(dlg_hsdetect->input, EV_KEY, KEY_MEDIA );
	input_set_capability(dlg_hsdetect->input, EV_KEY, KEY_VOLUMEUP);
	input_set_capability(dlg_hsdetect->input, EV_KEY, KEY_VOLUMEDOWN);

	ret = input_register_device(dlg_hsdetect->input);
	if (ret) {
		pr_err("%s : Failed to register driver\n", __func__);
		goto err_register_input_dev;
	}
 
	set_bit(EV_KEY,dlg_hsdetect->ids[0].evbit);
	dlg_hsdetect->ids[0].flags = INPUT_DEVICE_ID_MATCH_EVBIT;

    g_dlg_hsdetect=dlg_hsdetect;
    
	if(gpio_request(mfp_to_gpio(MFP_PIN_GPIO79), "D1980_HS_DETECT")) {
		return -1;
	}
	gpio_direction_input(mfp_to_gpio(MFP_PIN_GPIO79));

    ret = request_threaded_irq(gpio_to_irq(mfp_to_gpio(MFP_PIN_GPIO79)), NULL,
                    d1980_hs_interrupt_gpio_handler, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
				   "d1980_hs_detect", d1980);
    d1980_register_irq(d1980, D1980_IRQ_EAUDIO, d1980_hookswitch_interrupt_handler, 
                       0, DRIVER_NAME, d1980);   				   
#if defined(HS_DETECT_REV0203_TEMP)
	if(gpio_request(mfp_to_gpio(MFP_PIN_GPIO133), "D1980_HS_DETECT")) {
		return -1;
	}
	gpio_direction_input(mfp_to_gpio(MFP_PIN_GPIO133));
    ret = request_threaded_irq(gpio_to_irq(mfp_to_gpio(MFP_PIN_GPIO133)), NULL,
                    d1980_hs_interrupt_gpio_handler, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
				   "d1980_hs_detect2", d1980);
#endif
    dlg_hsdetect->hsdetect_status=0;
    dlg_hsdetect->hsdetect_timer_count=0;
    dlg_hsdetect->hookswitch_token_count=0;
    dlg_hsdetect->pre_hsdetect_status=0;
    dlg_hsdetect->mic_exist=0;


    INIT_WORK(&dlg_hsdetect->hsdetect_work,
    d1980_hs_gpio_worker);
    INIT_WORK(&dlg_hsdetect->hookswitch_work,
    d1980_hookswitch_polling_worker);			 
    schedule_work(&dlg_hsdetect->hsdetect_work);
    dbg( "hsdetect Driver registered\n");
	return 0;

    err_register_input_dev:
	input_free_device(dlg_hsdetect->input);  
	return ret;

}

static int __devexit d1980_hs_remove(struct platform_device *pdev)
{
	struct d1980 *d1980 = platform_get_drvdata(pdev);
	struct d1980_hs *dlg_hsdetect = &d1980->hsdetect;

	d1980_free_irq(d1980, D1980_IRQ_EGPI0);
	//d1980_free_irq(d1980, D1980_IRQ_EAUDIO);

       del_timer_sync(&dlg_hsdetect->hsdetect_timer);     
	del_timer_sync(&dlg_hsdetect->hookswitch_timer);

	return 0;
}   

static struct platform_driver d1980_hs_driver = {
	.probe		= d1980_hs_probe,
	.remove		= __devexit_p(d1980_hs_remove),
	.driver		= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
	}
};

static int __init d1980_hs_init(void)
{
	return platform_driver_register(&d1980_hs_driver);
}

static void __exit d1980_hs_exit(void)
{
	platform_driver_unregister(&d1980_hs_driver);
}

module_init(d1980_hs_init);
module_exit(d1980_hs_exit);   

MODULE_AUTHOR("Dialog Semiconductor Ltd <divyang.patel@diasemi.com>");
MODULE_DESCRIPTION("hsdetect driver for the Dialog D1980 PMIC");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRIVER_NAME);
