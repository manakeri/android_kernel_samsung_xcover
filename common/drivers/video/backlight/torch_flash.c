/* drivers/video/backlight/torch_flash.c
 *
 * Author:	
 * Created:	
 * Copyright:	
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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/backlight.h>
#include <linux/fb.h>
#include <mach/mfp-pxa9xx.h>
#include <mach/gpio.h>
#include <linux/delay.h>
#include <linux/spinlock.h>

#if defined(CONFIG_PMIC_D1980)
#include <linux/d1982/pmic.h>
#else
//#include <mach/levante.h>
//#include <mach/levante_hw.h>
#endif

#include <linux/regulator/consumer.h>
#include <linux/regulator/machine.h>


#include <mach/hardware.h>

#define TORCH_FLASH_MAX	   1801
#define TORCH_FLASH_MIN	   0

#define TORCH_FLASH_DEBUG    1
#define TORCH_TIME_EXPIRE      1

#define FLASH_CONTROL_NUM    14

static int torch_flash_suspended;
static int current_intensity = 0;
int torch_flash_enable_gpio = 0; 
int torch_flash_mode_gpio = 0; 


#if TORCH_TIME_EXPIRE
struct work_struct  work_timer;
struct hrtimer torch_timer;
static struct workqueue_struct *time_expire_wq;
#endif

#if	TORCH_FLASH_DEBUG
#define	torch_debug_msg(fmt, args...)	printk(KERN_INFO "[TORCH: %-18s:%5d]" fmt, __FUNCTION__, __LINE__, ## args)
#else
#define	torch_debug_msg(fmt, args...)	do{}while(0)
#endif

/* sys fs  */
struct class *torch_class;
EXPORT_SYMBOL(torch_class);
struct device *torch_dev;
EXPORT_SYMBOL(torch_dev);
 
static void Torch_Flash_On(struct device *dev, struct device_attribute *attr, char *buf);
static DEVICE_ATTR(torch_on , S_IRUGO /*0444*/, Torch_Flash_On, NULL);

static void Torch_Flash_Off(struct device *dev, struct device_attribute *attr, char *buf);
static DEVICE_ATTR(torch_off , S_IRUGO /*0444*/, Torch_Flash_Off, NULL);


static void torch_control_current(int address, int data)
{
    int index = 0;

    torch_debug_msg("torch_control_current address = %d , data = %d\n", address, data);
    
    for(index = 0; index < address ; index ++)
    {
       gpio_direction_output(torch_flash_mode_gpio,0);  
       udelay(1);
       gpio_direction_output(torch_flash_mode_gpio,1);
       udelay(1);
    }

    mdelay(1);
    
    index = 0;

    for(index = 0; index < data ; index ++)
    {
       gpio_direction_output(torch_flash_mode_gpio,0);  
       udelay(1);
       gpio_direction_output(torch_flash_mode_gpio,1);
       udelay(1);
    }

    mdelay(1);


}

static void Torch_Flash_On(struct device *dev, struct device_attribute *attr, char *buf)
{
	printk(KERN_INFO "Torch_Flash_On\n");

 	torch_control_current(17,9);  /* 40% of max */
}

static void Torch_Flash_Off(struct device *dev, struct device_attribute *attr, char *buf)
{
	printk(KERN_INFO "Torch_Flash_Off\n");

	gpio_direction_output(torch_flash_mode_gpio,0);	
}
/* sys fs */


#if TORCH_TIME_EXPIRE
static void time_expire_work_func(struct work_struct *work)
{
       int ret = 0;
       int *mfpr_value = 0;

       mfpr_value = io_p2v(0x40e102dc);

       *(mfpr_value) = 0xa840;
       
       torch_debug_msg("off\n");
	
       gpio_direction_output(torch_flash_mode_gpio,0);

       return HRTIMER_NORESTART;
}

static enum hrtimer_restart torch_flash_timer_func(struct hrtimer *timer)
{
	//queue_work(time_expire_wq, &work_timer);
	//hrtimer_start(&torch_timer, ktime_set(2, 0), HRTIMER_MODE_REL);

	return HRTIMER_NORESTART;
}
#endif


/* input: intensity in percentage 0% - 100% */
static int torch_flash_send_intensity(struct backlight_device *bd)
{
	int user_intensity = bd->props.brightness;
	int plat_intensity = 0;
	u8 wled2b = 0;
      int ret = 0;

      int *mfpr_value = 0;


      torch_debug_msg("intensity = %d\n", user_intensity);
      
      /* for production mode */
      if(user_intensity >= TORCH_FLASH_MAX)
          return 0;
      
      mfpr_value = io_p2v(0x40e102dc);
      
      if(user_intensity){

          *(mfpr_value) = 0xa940;

           
           /* + on110824 to control the current of flash */
           
           /* address : 0 (17 pulse), data : 6 (6 pulse)  for 280 mA */
           //torch_control_current(17,6);  /* 56% of max */

           /* address : 0 (17 pulse), data : 8 (8 pulse)  for 227 mA */
           //torch_control_current(17,8);  /* 45% of max */

           /* address : 0 (17 pulse), data : 9 (9 pulse)  for 202 mA */
           //torch_control_current(17,9);  /* 40% of max */

            /* address : 0 (17 pulse), data : 14 (14 pulse)  for 111.4 mA */
           torch_control_current(17,14);  /* 22% of max */

           /* address : 0 (17 pulse), data : 10 (10 pulse)  for 182 mA */
           //torch_control_current(17,10);  /* 36% of max */
           
           /* address : 3 (20 pulse), data : 1 (1 pulse)  for MAX 500 mA */
           torch_control_current(20,1);  /* max 500mA */

           /* - on110824 to control the current of flash */

           #if TORCH_TIME_EXPIRE
           hrtimer_start(&torch_timer, ktime_set(user_intensity, 0), HRTIMER_MODE_REL);

           torch_debug_msg("timer set to %d\n", user_intensity);
           #endif
           
      }else{
      
           *(mfpr_value) = 0xa840;
      
           hrtimer_cancel(&torch_timer);
           
           gpio_direction_output(torch_flash_mode_gpio,0);	
      }

	return 0;
}

#ifdef CONFIG_PM
static int torch_flash_suspend(struct platform_device *pdev, pm_message_t state)
{

	torch_flash_suspended = 1;

         #if TSP_ESD_CHECK
         hrtimer_cancel(&torch_timer);
         #endif

      torch_debug_msg("torch_flash_suspended = %d\n", torch_flash_suspended);
	return 0;
}

static int torch_flash_resume(struct platform_device *pdev)
{

	torch_flash_suspended = 0;
	torch_debug_msg("torch_flash_suspended = %d\n", torch_flash_suspended);
	return 0;
}
#else
#define torch_flash_suspend	NULL
#define torch_flash_resume	NULL
#endif

static int torch_flash_set_intensity(struct backlight_device *bd)
{
	torch_flash_send_intensity(bd);
	return 0;
}

static int torch_flash_get_intensity(struct backlight_device *bd)
{
	return current_intensity;
}

static struct backlight_ops torch_flash_ops = {
	.get_brightness = torch_flash_get_intensity,
	.update_status  = torch_flash_set_intensity,
};

static int __init torch_flash_probe(struct platform_device *pdev)
{
	struct backlight_device *bd;
       struct backlight_properties props;
      
      memset(&props, 0, sizeof(struct backlight_properties));
      props.brightness = TORCH_FLASH_MAX;
      props.max_brightness = TORCH_FLASH_MAX;
      //props.power = FB_BLANK_UNBLANK;

	printk(KERN_INFO "------->torch_flash_probe\n");

	bd = backlight_device_register("torch", &pdev->dev, NULL,
		    &torch_flash_ops, &props);
	if (IS_ERR(bd))
		return PTR_ERR(bd);


      torch_flash_enable_gpio = mfp_to_gpio(MFP_PIN_GPIO0);
      torch_flash_mode_gpio = mfp_to_gpio(MFP_PIN_GPIO1);


	if (gpio_request(torch_flash_enable_gpio, "torch_enable"))
	    printk(KERN_ERR "torch_flash_enable_gpio Request GPIO_%d failed!\n", torch_flash_enable_gpio);

      if (gpio_request(torch_flash_mode_gpio, "torch_mode"))
	    printk(KERN_ERR "torch_flash_mode_gpio Request GPIO_%d failed!\n", torch_flash_mode_gpio);

	platform_set_drvdata(pdev, bd);

       #if TORCH_TIME_EXPIRE
       hrtimer_init(&torch_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
       torch_timer.function = time_expire_work_func;
       #endif

	bd->props.power = FB_BLANK_UNBLANK;
	bd->props.brightness = TORCH_FLASH_MAX;
	bd->props.max_brightness = TORCH_FLASH_MAX;

	return 0;
}

static int torch_flash_remove(struct platform_device *pdev)
{
	struct backlight_device *bd = platform_get_drvdata(pdev);

	bd->props.brightness = 0;
	bd->props.power = 0;
	torch_flash_send_intensity(bd);

	backlight_device_unregister(bd);

	return 0;
}

static struct platform_driver torch_flash_driver = {
	.probe		= torch_flash_probe,
	.remove		= torch_flash_remove,
#ifdef	CONFIG_PM
	.suspend	= torch_flash_suspend,
	.resume		= torch_flash_resume,
#endif
	.driver		= {
		.name	= "torch-flash",
	},
};

static int __init torch_flash_init(void)
{

      #if TORCH_TIME_EXPIRE
      //time_expire_wq = create_singlethread_workqueue("time_expire_wq");	
      #endif
      
    /* sys fs */
	torch_class = class_create(THIS_MODULE, "torch");
	if (IS_ERR(torch_class))
		pr_err("Failed to create class(torch)!\n");

	torch_dev = device_create(torch_class, NULL, 0, NULL, "torch");
	if (IS_ERR(torch_dev))
		pr_err("Failed to create device(torch)!\n");

	if (device_create_file(torch_dev, &dev_attr_torch_on) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_torch_on.attr.name); 

	if (device_create_file(torch_dev, &dev_attr_torch_off) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_torch_off.attr.name); 

		
	/* sys fs */

      
	return platform_driver_register(&torch_flash_driver);
}

static void __exit torch_flash_exit(void)
{
	platform_driver_unregister(&torch_flash_driver);

	class_destroy(torch_class);

}

late_initcall(torch_flash_init);
module_exit(torch_flash_exit);
MODULE_LICENSE("GPL");
