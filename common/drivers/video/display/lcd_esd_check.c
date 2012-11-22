 /* drivers/video/display/lcd_esd_check.c
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
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <mach/gpio.h>
#include <linux/device.h>
//#include <mach/levante_hw.h>

#ifdef CONFIG_PXA3xx_DVFM
#include <mach/dvfm.h>
#include <mach/pxa3xx_dvfm.h>
#endif

#define LCD_ESD_DEBUG
#define CONFIG_HAS_EARLYSUSPEND

struct lcd_esd_check_data {
	int use_irq;
	struct work_struct  work;
	struct early_suspend early_suspend;
};

static struct workqueue_struct *lcd_esd_check_wq;
struct lcd_esd_check_data lcd_esd_check_global;
int  lcd_esd_irq;

#ifdef CONFIG_HAS_EARLYSUSPEND
static void lcd_esd_check_early_suspend(struct early_suspend *h);
static void lcd_esd_check_late_resume(struct early_suspend *h);



static struct early_suspend lcdesd_early_suspend_desc = {
	.level = 40,
	.suspend = lcd_esd_check_early_suspend,
	.resume = lcd_esd_check_late_resume,
};

#endif

extern int lcd_esd_reset(void);

static void lcd_esd_check_work_func(struct work_struct *work)
{    
     struct lcd_esd_check_data *ts = container_of(work, struct lcd_esd_check_data, work);

     #ifdef LCD_ESD_DEBUG 
     printk("[LCD_ESD] lcd_esd_check_work_func\n");
     #endif

	disable_irq(lcd_esd_irq);
     /* LCD RESTART */
     lcd_esd_reset();
	enable_irq(lcd_esd_irq);     
     msleep(250);


}



static irqreturn_t lcd_esd_check_irq_handler(int irq, void *dev_id)
{
	struct lcd_esd_check_data *ts = dev_id;

       #ifdef LCD_ESD_DEBUG 
       printk("[LCD_ESD] lcd_esd_check_irq_handler\n");
       #endif

	queue_work(lcd_esd_check_wq, &(lcd_esd_check_global.work));
       
	return IRQ_HANDLED;
}


static int lcd_esd_check_probe(struct platform_device *pdev)
{
       int ret = 0;

      #ifdef LCD_ESD_DEBUG
      printk("[LCD_ESD] lcd_esd_check_probe START\n");
      #endif
      
       lcd_esd_irq = platform_get_irq(pdev, 0);

       #ifdef LCD_ESD_DEBUG
       printk("[LCD_ESD] lcd_esd_irq == %d\n",lcd_esd_irq);
       #endif

    
	INIT_WORK(&(lcd_esd_check_global.work), lcd_esd_check_work_func);

      #ifdef LCD_ESD_DEBUG
       printk("[LCD_ESD] %s, after INIT_WORK\n", __func__ );
      #endif

      
	if (lcd_esd_irq) {
		ret = request_irq(lcd_esd_irq, lcd_esd_check_irq_handler, IRQF_TRIGGER_RISING , "lcd_esd", &lcd_esd_check_global);
		if (ret == 0){
                   #ifdef LCD_ESD_DEBUG
                   printk("[LCD_ESD] request_irq Success\n");
                   #endif
			lcd_esd_check_global.use_irq = 1;
            
		}else{
		      #ifdef LCD_ESD_DEBUG
                   printk("[LCD_ESD] request_irq Fail\n");
                   #endif
			lcd_esd_check_global.use_irq = 0;
		}
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	register_early_suspend(&lcdesd_early_suspend_desc);
#endif
	return 0;


err_input_dev_alloc_failed:
//	kfree(ts);
err_alloc_data_failed:
err_check_functionality_failed:
	return ret;
}

static int lcd_esd_check_remove(struct platform_device *pdev)
{

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&lcdesd_early_suspend_desc);
#endif


	unregister_early_suspend(&lcd_esd_check_global.early_suspend);
	if (lcd_esd_check_global.use_irq)
		free_irq(lcd_esd_irq,&lcd_esd_check_global);
	return 0;
}

static int lcd_esd_check_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	int ret;
	struct lcd_esd_check_data *ts = platform_get_drvdata(pdev);


	disable_irq(lcd_esd_irq);
       
	return 0;
}

static int lcd_esd_check_resume (struct platform_device *pdev)
{
     	int ret, key;
	struct lcd_esd_check_data *ts = platform_get_drvdata(pdev);

	enable_irq(lcd_esd_irq);
      
      return 0;
}


#ifdef CONFIG_HAS_EARLYSUSPEND
static void lcd_esd_check_early_suspend(struct early_suspend *h)
{
	struct lcd_esd_check_data *ts;
	ts = container_of(h, struct lcd_esd_check_data, early_suspend);
	lcd_esd_check_suspend(ts, PMSG_SUSPEND);
}

static void lcd_esd_check_late_resume(struct early_suspend *h)
{
	struct lcd_esd_check_data *ts;
	ts = container_of(h, struct lcd_esd_check_data, early_suspend);
	lcd_esd_check_resume(ts);
}
#endif


static const struct i2c_device_id lcd_esd_check_id[] = {
	{ "lcd_esd", 1 },
	{ }
};


static struct platform_driver  lcd_esd_check_driver = {
	.probe		= lcd_esd_check_probe,
	.remove		= lcd_esd_check_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= lcd_esd_check_suspend,
	.resume		= lcd_esd_check_resume,
#endif
	.id_table	= lcd_esd_check_id,
	.driver = {
             .name	= "lcd_esd",
	},
};


static int  __devinit lcd_esd_check_init(void)
{
      #ifdef LCD_ESD_DEBUG
	printk("[LCD_ESD] %s\n", __func__ );
      #endif
    
	lcd_esd_check_wq = create_singlethread_workqueue("lcd_esd_wq");
      
	if (!lcd_esd_check_wq)
		return -ENOMEM;

	return platform_driver_register(&lcd_esd_check_driver);

}

static void __exit lcd_esd_check_exit(void)
{
	if (lcd_esd_check_wq)
		destroy_workqueue(lcd_esd_check_wq);
}


module_init(lcd_esd_check_init);
module_exit(lcd_esd_check_exit);

MODULE_DESCRIPTION("LCD ESD CHECK Driver");
MODULE_LICENSE("GPL");
