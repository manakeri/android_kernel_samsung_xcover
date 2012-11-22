 /* drivers/video/backlight/aat1402iuq.c
 *
 * Author:	jonggab.park
 * Created:	January 25, 2011
 * Copyright:	SAMSUNG. All Rights Reserved

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
#include <linux/spinlock.h>
#include <linux/backlight.h>
#include <linux/fb.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/spinlock.h>

#ifdef CONFIG_PXA3xx_DVFM
#include <mach/dvfm.h>
#include <mach/pxa3xx_dvfm.h>
#endif

#include <plat/mfp.h>

#include <linux/earlysuspend.h>



/* + for LDI Brightness Control */
#define LCD_COMMAND 0x0 << 9
#define LCD_PARAMETERS 0x1 << 8

#define CMD(x) (LCD_COMMAND | x)
#define PARA(x) (LCD_PARAMETERS | x)
#define ARRAY_AND_SIZE(x)	(x), ARRAY_SIZE(x)

static void aat1402iuq_early_suspend(struct early_suspend *h);

static void aat1402iuq_late_resume(struct early_suspend *h);


static struct early_suspend backlight_early_suspend_desc = {
	.level = 40,
	.suspend = aat1402iuq_early_suspend,
	.resume = aat1402iuq_late_resume,
};



//////////////////////////////////TSP TRACE by PSJ///////////////////////////////////////
typedef	struct	
{
	u16 time;
	u16	x;
	u16	y;
	u16 up_down; //down : 0, up : 1, sub_up : 2
}_ts_trace_coord;

_ts_trace_coord touch_trace[500];

static int touch_trace_count =0;

void push_touch_trace(int coord_x, int coord_y, int result)
{
	if(touch_trace_count >= 500)
	{
		touch_trace_count = 0;
	}
	memset(&touch_trace[touch_trace_count], 0x0, sizeof(_ts_trace_coord));

	touch_trace[touch_trace_count].x = coord_x;
	touch_trace[touch_trace_count].y = coord_y;
	touch_trace[touch_trace_count].up_down = result;
	touch_trace[touch_trace_count].time = jiffies;

	//printk("[TSP] time : %d, x : %d, y : %d, result : %d. \n",touch_trace[touch_trace_count].time, touch_trace[touch_trace_count].x, touch_trace[touch_trace_count].y, touch_trace[touch_trace_count].up_down);

	touch_trace_count++;
	
}
EXPORT_SYMBOL(push_touch_trace);
//////////////////////////////////TSP TRACE by PSJ///////////////////////////////////////

extern void ssp_send_cmd_para(u16 *cmd,int num);
/* - for LDI Brightness Control */

static int current_tuned_level;
extern int wakeup_tuned_level;
static int aat1402iuq_suspended;

extern int backlight_status;
static int backlight_pin = 0;

static struct backlight_device *p_backlight_device;

static DEFINE_SPINLOCK(bl_ctrl_lock);

//#define __PWM_CONTROL__
#define __BACKLIGHT_DEBUG__

#define	LCD_LED_MAX	255
#define	LCD_LED_MIN	30

#define	LCD_LED_CURRENT	141

#define MAX_BRIGHTNESS_IN_BLU	32

#if defined (__PWM_CONTROL__)
#define DIMMING_VALUE		7
#define MIN_BRIGHTNESS_VALUE	30
#else
#define DIMMING_VALUE		31
#define MIN_BRIGHTNESS_VALUE	16
#endif

#define MAX_BRIGHTNESS_VALUE	255



struct brt_value{
	int level;				// Platform setting values
	int tune_level;			// Chip Setting values
};

#if defined (__PWM_CONTROL__)
struct brt_value brt_table_ktd[] = {
	  { 0,  5 }, // Off
	  { 20, DIMMING_VALUE }, // Dimming pulse 26(33-7) by HW
	  { 30,  6 }, // Min pulse 27(33-6) by HW
	  { 41,  7 }, 
	  { 53,  8 },  
	  { 64,  9 }, 
	  { 76,  10 }, 
	  { 87,  11 }, 
	  { 99,  12 }, 
	  { 110,  13 },
	  { 122,  14 },
	  { 133,  15 },
	  { 145,  16 }, // default pulse 17(33-16) by HW
	  { 156,  17 },
	  { 167,  18 },
	  { 178,  19 }, 
	  { 189,  20 }, 
	  { 200,  21 }, 
	  { 211,  22 }, 
	  { 222,  23 }, 
	  { 233,  24  },
	  { 244,  25  },
	  { MAX_BRIGHTNESS_VALUE,  26 }, // Max pulse 7(33-26) by HW
};


static u16 DisplayBrightness[] = {
	CMD(0xB5),
	PARA(0x80), //0x80
};

#else

#if defined (CONFIG_MACH_JETTA)
struct brt_value brt_table_ktd[] = {
  	  { MIN_BRIGHTNESS_VALUE, 30  }, // 1
	  { 32, 30  }, // 2
	  { 48, 30  }, // 3
	  { 64, 29  }, // 4
	  { 80, 29  }, // 5
	  { 96, 29  }, // 6
	  { 112, 25 }, // 7
	  { 128, 22  }, // 8
	  { 144, 20  }, // 9
	  { 160, 19  }, // 10
	  { 176, 17  }, // 11
	  { 192, 15  }, // 12
	  { 208, 13  }, // 13
	  { 224, 11  }, // 14	  
	  { 240, 9  }, // 15
	  { MAX_BRIGHTNESS_VALUE, 7  }, // 16
};


#else

struct brt_value brt_table_ktd[] = {
  	  { 30 /*MIN_BRIGHTNESS_VALUE*/, 30  }, // 1
	  { 35, 29  }, // 2
	  { 45, 28  }, // 3
	  { 60, 26  }, // 4
	  { 75, 24  }, // 5
	  { 90, 23  }, // 6
	  { 105, 22  }, // 7
	  { 120, 21  }, // 8
	  { 135, 20  }, // 9
	  { 150, 19  }, // 10
	  { 165, 18  }, // 11
	  { 180, 17  }, // 12
	  { 195, 16  }, // 13
	  { 210, 15  }, // 14
	  { 225, 14  }, // 15	 
	  { 230, 13  }, // 16
	  { MAX_BRIGHTNESS_VALUE,6 }, // 17
};

/*
struct brt_value brt_table_ktd[] = {
  	  { MIN_BRIGHTNESS_VALUE, 30  }, // 1
	  { 32, 29  }, // 2
	  { 48, 28  }, // 3
	  { 64, 26  }, // 4
	  { 80, 24  }, // 5
	  { 96, 23  }, // 6
	  { 112, 22  }, // 7
	  { 128, 21  }, // 8
	  { 144, 20  }, // 9
	  { 160, 19  }, // 10
	  { 176, 17  }, // 11
	  { 192, 16  }, // 12
	  { 208, 15  }, // 13
	  { 224, 14  }, // 14
	  { 240, 13  }, // 15	 
	  { MAX_BRIGHTNESS_VALUE,6 }, // 16
};
*/

#endif

#endif


#define MAX_BRT_STAGE (int)(sizeof(brt_table_ktd)/sizeof(struct brt_value))

#ifdef CONFIG_PXA3xx_DVFM
static int dvfm_dev_idx;
static void set_dvfm_constraint(void)
{
	/* Disable Lowpower mode */
	dvfm_disable_op_name("D0CS", dvfm_dev_idx);
	dvfm_disable_op_name("D1", dvfm_dev_idx);
	dvfm_disable_op_name("D2", dvfm_dev_idx);
	dvfm_disable_op_name("CG", dvfm_dev_idx);
}

static void unset_dvfm_constraint(void)
{
	/* Enable Lowpower mode */
	dvfm_enable_op_name("D0CS", dvfm_dev_idx);
	dvfm_enable_op_name("D1", dvfm_dev_idx);
	dvfm_enable_op_name("D2", dvfm_dev_idx);
	dvfm_enable_op_name("CG", dvfm_dev_idx);
}


#else
static void set_dvfm_constraint(void) {}
static void unset_dvfm_constraint(void) {}
#endif

void lcd_backlight_power_on(unsigned char intensity)
{

    #if defined(__BACKLIGHT_DEBUG__)
        printk("[BACKLIGHT] lcd_backlight_power_on ===> intensity : %d\n",intensity);
    #endif

}

void lcd_backlight_power_off(void)
{
    #if defined(__BACKLIGHT_DEBUG__)
	printk("[BACKLIGHT] lcd_backlight_power_off\n");
    #endif

}


static int lcd_backlight_control(int num)
{
    int index = 0;
    int limit;
    
    
    limit = num+1;
    set_dvfm_constraint();
    spin_lock(&bl_ctrl_lock);
    
	if(backlight_status==0)
	{    
	    gpio_set_value(backlight_pin, 1);
	    backlight_status = 1;
	    mdelay(1);
        #if defined(__BACKLIGHT_DEBUG__)
	    printk("[BACKLIGHT] Backlight On \n");
	    #endif
 	 
	    
   	}

    
    for(index = 0; index < limit; index ++)
    {
       gpio_direction_output(backlight_pin,0);  
       udelay(1);
       gpio_direction_output(backlight_pin,1);
       udelay(1);
    }
    
    spin_unlock(&bl_ctrl_lock);
    unset_dvfm_constraint();

    return 0;
}

#if defined (__PWM_CONTROL__)
static void aat1402iuq_BacklightCtrlbyPWM(int tune_level)
{
	DisplayBrightness[1] = PARA(tune_level);

	ssp_send_cmd_para(ARRAY_AND_SIZE(DisplayBrightness));
    #if defined(__BACKLIGHT_DEBUG__)
	printk("[BACKLIGHT] tuned_level : %d\n",tune_level);
	#endif
	
}

#else
static void aat1402iuq_BacklightCtrlbySingleWire(int tune_level)
{

	int pulse;

	if(tune_level <= 0) 
	{
        gpio_direction_output(backlight_pin,0);
        
	    mdelay(1);
        
        #if defined(__BACKLIGHT_DEBUG__)
        printk("[BACKLIGHT] Backlight Off \n");
        #endif

	}
	else 
	{
         lcd_backlight_control(tune_level); 
	}

	
    udelay(550); // over 60us
	
	
}
#endif
	
/* input: intensity in percentage 0% - 100% */
static int aat1402iuq_send_intensity(int tune_level)
{

	if(current_tuned_level != tune_level)
	{
  #if defined(__PWM_CONTROL__)
	aat1402iuq_BacklightCtrlbyPWM(tune_level);
  #else
	aat1402iuq_BacklightCtrlbySingleWire(tune_level);
  #endif
	}

	current_tuned_level = tune_level;	
      
	return 0;
}

#ifdef CONFIG_PM
static int aat1402iuq_suspend(struct platform_device *pdev, pm_message_t state)
{

    #if defined(__BACKLIGHT_DEBUG__)
	printk("[BACKLIGHT] aat1402iuq_suspend\n");
    #endif

	return 0;
}

static int aat1402iuq_resume(struct platform_device *pdev)
{

    #if defined(__BACKLIGHT_DEBUG__)
	printk("[BACKLIGHT] aat1402iuq_resume\n");
    #endif

	return 0;
}
#else
#define aat1402iuq_suspend	NULL
#define aat1402iuq_resume	NULL
#endif

static int aat1402iuq_set_intensity(struct backlight_device *bd)
{
    
	int user_intensity = bd->props.brightness;
    int tune_level = 0;
    int i;   


      if(user_intensity > 0) 
      {
			if(user_intensity < MIN_BRIGHTNESS_VALUE) 
			{
				tune_level = DIMMING_VALUE; //DIMMING
			} 
			else if (user_intensity == MAX_BRIGHTNESS_VALUE) 
			{
				tune_level = brt_table_ktd[MAX_BRT_STAGE-1].tune_level;
			} 
			else 
			{
				for(i = 0; i < MAX_BRT_STAGE; i++)
				{
					if(user_intensity <= brt_table_ktd[i].level ) 
					{
						tune_level = brt_table_ktd[i].tune_level;
						break;
					}
				}
			}
     }

   	if(aat1402iuq_suspended == 1)
	{
	  #if defined(__BACKLIGHT_DEBUG__)
	      printk("[BACKLIGHT] skip backlight level \n");
      #endif
	}
	else
	{
      #if defined(__BACKLIGHT_DEBUG__)
	      printk("[BACKLIGHT] user_intensity, tuned_level : %d\n",user_intensity,tune_level);
      #endif

		aat1402iuq_send_intensity(tune_level);
	}

	return 0;
}


static void aat1402iuq_early_suspend(struct early_suspend *h)
	{

	  #if defined(__BACKLIGHT_DEBUG__)
	printk("[BACKLIGHT] aat1402iuq_early_suspend\n");
      #endif
	aat1402iuq_suspended = 1;
	aat1402iuq_set_intensity(p_backlight_device);


	}

static void aat1402iuq_late_resume(struct early_suspend *h)
{

    #if defined(__BACKLIGHT_DEBUG__)
	printk("[BACKLIGHT] aat1402iuq_late_resume\n");
    #endif

	aat1402iuq_suspended = 0;
	aat1402iuq_set_intensity(p_backlight_device);

}


static int aat1402iuq_get_intensity(struct backlight_device *bd)
{
    #if defined(__BACKLIGHT_DEBUG__)
	printk("[BACKLIGHT] aat1402iuq_get_intensity\n");
    #endif

	return current_tuned_level;
}

static struct backlight_ops aat1402iuq_ops = {
	.get_brightness = aat1402iuq_get_intensity,
	.update_status  = aat1402iuq_set_intensity,
};

static int aat1402iuq_probe(struct platform_device *pdev)
{
	struct backlight_device *bd;
      struct backlight_properties props;

      int ret = 0;
      
      memset(&props, 0, sizeof(struct backlight_properties));
      props.brightness = LCD_LED_CURRENT;
      props.max_brightness = LCD_LED_MAX;
      props.power = FB_BLANK_UNBLANK;

      #if defined(__BACKLIGHT_DEBUG__)
	printk("[BACKLIGHT] aat1402iuq_probe\n");
      #endif

      backlight_pin = mfp_to_gpio(MFP_PIN_GPIO43);

       #if defined(__BACKLIGHT_DEBUG__)
	printk("[BACKLIGHT] pwm_enable ret = %d\n", ret);
      #endif

#if 0      
	if (gpio_request(backlight_pin, "backlight"))
	    printk(KERN_ERR "Backlight Request GPIO_%d failed!\n", backlight_pin);    
#endif
    
	bd = backlight_device_register("backlight-0", &pdev->dev, NULL,
		    &aat1402iuq_ops, &props);

	p_backlight_device = bd;
    
	if (IS_ERR(bd))
	{
             #if defined(__BACKLIGHT_DEBUG__)
	      printk("[BACKLIGHT] backlight_device_register Fail \n");
             #endif

             return PTR_ERR(bd);
	}

      #if defined(__BACKLIGHT_DEBUG__)
	printk("[BACKLIGHT] backlight_device_register success\n");
      #endif

	platform_set_drvdata(pdev, bd);

	bd->props.power = FB_BLANK_UNBLANK;
	bd->props.brightness = LCD_LED_CURRENT;
	bd->props.max_brightness = LCD_LED_MAX;

	register_early_suspend(&backlight_early_suspend_desc);

      aat1402iuq_set_intensity(bd);
      
	return 0;
}

static int aat1402iuq_remove(struct platform_device *pdev)
{
	struct backlight_device *bd = platform_get_drvdata(pdev);

    #if defined(__BACKLIGHT_DEBUG__)
	printk("[BACKLIGHT] aat1402iuq_remove\n");
    #endif

	bd->props.brightness = 0;
	bd->props.power = 0;
     aat1402iuq_set_intensity(bd);
     
	unregister_early_suspend(&backlight_early_suspend_desc);

	backlight_device_unregister(bd);

	return 0;
}

static struct platform_driver aat1402iuq_driver = {
	.probe		= aat1402iuq_probe,
	.remove		= aat1402iuq_remove,
	.driver		= {
		.name	= "backlight-0",
	},
};

static int __devinit backlight_init(void)
{

    #if defined(__BACKLIGHT_DEBUG__)
	printk("[BACKLIGHT] backlight_init\n");
    #endif
#ifdef CONFIG_PXA3xx_DVFM
	dvfm_register("aat1402iuq", &dvfm_dev_idx);
#endif
	return platform_driver_register(&aat1402iuq_driver);
}

static void __exit backlight_exit(void)
{
	platform_driver_unregister(&aat1402iuq_driver);
}

module_init(backlight_init);
module_exit(backlight_exit);
MODULE_LICENSE("GPL");

