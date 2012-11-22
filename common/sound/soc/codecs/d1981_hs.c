#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/serio.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <sound/soc-dapm.h>
#include <mach/d1980_hs.h>

#include "d1981_reg.h"

//#define D1981_DEBUG 1
#if D1981_DEBUG
#define dbg(format, arg...) printk(KERN_INFO "d1981: " format "\n", ## arg)
#else
#define dbg(format, arg...)
#endif

struct d1981_hs_data *g_hs_data=NULL;
extern unsigned char g_dlg_4pole_detect;
extern u8 d1981_reg_read(u8 const reg);
extern int d1981_reg_write(u8 const reg, u8 const val);
extern int d1981LowPowerHookDetection(u8 on);

int d1981_mic_bias(unsigned char on)
{
    u8 val;
    
    if(on==1)
    {
        val=d1981_reg_write(D1981_REG_MICBIAS1,0x82); //micbias1 enable
        val=d1981_reg_write(D1981_REG_MICDET,0xc8);//0x80);//mic detect enable
    }
    else
    {
        val=d1981_reg_write(D1981_REG_MICDET,0x00);//0x80);//mic detect enable
        val=d1981_reg_write(D1981_REG_MICBIAS1,0x02); //micbias1 disable
    }

    return val;

}
EXPORT_SYMBOL(d1981_mic_bias);

int d1981_hs_ready(void)
{
    u8 val;
    int ret=0;

    if(g_hs_data==NULL)
        return -1;
    else
        return 0;
}    
int d1981_hs_handler(struct kobject *kobj, int event)
{
    u8 val;
    int ret=0;

    if(g_hs_data==NULL)
    {
        //printk(KERN_ERR  "------->d1981_hs_handler g_hs_data is NULL !!!!!!!! \n");  
        return -1;
    }
    if(D1980_HEADSET_ADD ==event)
    {
#if 0
         val=d1981_reg_write(D1981_REG_MICBIAS1,0x82); //micbias1 enable
         //msleep(1);
         val=d1981_reg_write(D1981_REG_MICDET,0xc8); //0x80);//mic detect enable
         //msleep(1);
         val= (0x4 & d1981_reg_read(D1981_REG_STATUS_EXT));

         dbg("------->d1981_hs_handler MIC11 Register =0x%x \n",val);         
      
         if(val ==0) 
         {
             dbg( "------->d1981_hs_handler no MIC \n");
             val=d1981_reg_write(D1981_REG_MICBIAS1,0x02); //micbias1 disable
             //msleep(1);
             val=d1981_reg_write(D1981_REG_MICDET,0x00);//mic detect disable
             //msleep(1);
             g_hs_data->mic_status=D1980_HS_MIC_REMOVE;
             ret=0;
             
         }
         else 
         {
             dbg( "------->d1981_hs_handler MIC detected g_hs_data=0x%x \n",g_hs_data);
              g_hs_data->mic_status=D1980_HS_MIC_ADD;
              ret=1;
         }
#endif         
		if(g_dlg_4pole_detect==1)
		{
			g_hs_data->mic_status=D1980_HS_MIC_ADD;            
		}
		else
		{
			g_hs_data->mic_status=D1980_HS_MIC_REMOVE;
            
		}
		dbg("------->d1981_hs_handler ADD event \n");
		g_hs_data->hs_status=D1980_HEADSET_ADD;
		kobject_uevent(kobj, KOBJ_ADD);
		switch_set_state(&g_hs_data->sdev, 1);   

    }
    else
    {        
        g_hs_data->hs_status=D1980_HEADSET_REMOVE;
        g_hs_data->mic_status=D1980_HS_MIC_REMOVE;
        d1981LowPowerHookDetection(0);
        kobject_uevent(kobj, KOBJ_REMOVE);
        switch_set_state(&g_hs_data->sdev, 0);   
        dbg( "------->d1981_hs_handler REMOVE event \n");
        ret=0;
    }
    return ret;
}
EXPORT_SYMBOL_GPL(d1981_hs_handler);

int d1981_get_hook_state(void)
{
    u8 val;

    val=d1981_reg_read(D1981_REG_STATUS_EXT);
       
    val=(0x08 &val );

    //dbg("d1981_get_hook_state REG=0x0 val=%d \n",val);

    if(val==0)
    {
        return 1; //hook released
    }
    else
        return 0; //hook pressed

  
}

EXPORT_SYMBOL_GPL(d1981_get_hook_state);

static ssize_t print_switch_name(struct switch_dev *sdev, char *buf)
{
    dbg(  "------->print_switch_name \n");
	return sprintf(buf, "%s\n", "h2w");
}

static ssize_t print_switch_state(struct switch_dev *sdev, char *buf)
{
/*
pasted from Java code
int BIT_HEADSET = (1 << 0);
int BIT_HEADSET_NO_MIC = (1 << 1);
int SUPPORTED_HEADSETS = (BIT_HEADSET|BIT_HEADSET_NO_MIC);
int HEADSETS_WITH_MIC = BIT_HEADSET;
*/
//	static struct D1980_HS_IOCTL hsData;
//	getHeadSetStatus(&hsData);

/* 4 poles plug, Headset, with Mic */
struct d1981_hs_data	*hs_data =
		container_of(sdev, struct d1981_hs_data, sdev);
		
   // dbg( "------->print_switch_state hs_data->hs_status=%d g_dlg_4pole_detect=%d\n",hs_data->hs_status,g_dlg_4pole_detect);
    if(hs_data->hs_status == D1980_HEADSET_ADD)
    {
       // if (hs_data->mic_status== D1980_HS_MIC_ADD)
       if(g_dlg_4pole_detect==1)
        	return sprintf(buf, "%s\n", "1");
        else
        	return sprintf(buf, "%s\n", "2");
    }
    else
    {
        return sprintf(buf, "%s\n", "0");
    }
}

/****************************************************************************
 * File System I/O operations
 ***************************************************************************/

static int d1981_hsdetect_open(struct inode *inode, struct file *file)
{
	return 0;
}

static ssize_t d1981_hsdetect_read(struct file *file, char __user *buf, size_t count,
			    loff_t *ppos)
{
	return 0;
}

/* We assume that the user buf will not larger than kbuf size */
static ssize_t d1981_hsdetect_write(struct file *file, const char __user *buf,
			      size_t count, loff_t *ppos)
{
	return count;
}

static int d1981_hsdetect_release(struct inode *inode, struct file *file)
{
	return 0;
}

static int d1981_hsdetect_ioctl(struct inode *inode, struct file *file,unsigned int cmd, unsigned long arg)
{
	struct D1981_HS_IOCTL hs_ioctl;
		
	if (copy_from_user(&hs_ioctl, (void *)arg, sizeof(struct D1981_HS_IOCTL) ))
		return -EFAULT;

	switch(cmd)
	{
		case D1980_HSDETECT_STATUS:
			hs_ioctl.hsdetect_status = g_hs_data->hs_status;
			hs_ioctl.hookswitch_status = d1981_get_hook_state();
			#if defined(ENABLE_HS_DETECT_POLES)
			hs_ioctl.hsmic_status = g_hs_data->mic_status;
			#endif
			dbg( "------->d1980_hsdetect_ioctl d1980_HSDETECT_STATUS \n");
			break;

		case D1980_HOOKSWITCH_STATUS:
			hs_ioctl.hookswitch_status = d1981_get_hook_state();
			hs_ioctl.hsdetect_status = g_hs_data->hs_status;
			#if defined(ENABLE_HS_DETECT_POLES)
			hs_ioctl.hsmic_status = g_hs_data->mic_status;
			#endif
			dbg("------->d1980_hsdetect_ioctl d1980_HOOKSWITCH_STATUS \n");
			break;

		default:
			 return -ENOTTY;

	}
	return copy_to_user((void *)arg,&hs_ioctl, sizeof(struct D1981_HS_IOCTL));
}

static struct file_operations d1981_hsdetect_fops = {
	.owner		= THIS_MODULE,
	.open		= d1981_hsdetect_open,
	.release	= d1981_hsdetect_release,
	.ioctl		= d1981_hsdetect_ioctl,
	.write		= d1981_hsdetect_write,
	.read		= d1981_hsdetect_read,
};

#if 0
static struct miscdevice d1981_hsdetect_miscdev = {
	.minor		= MISC_DYNAMIC_MINOR,
	.name		= "levante_hsdetect",
	.fops		= &d1981_hsdetect_fops,
};
#endif

static int d1981_hs_probe(struct platform_device *pdev)
{
	int ret = -1;
	struct d1981_hs_data *hs_data;
	u8 val;

     dbg( "d1981_hs_probe !!!!!!!!!!!! \n");

       hs_data = kzalloc(sizeof(struct d1981_hs_data), GFP_KERNEL);
	if (!hs_data)
	{
		ret=-ENOMEM;
		goto err;
       }
       g_hs_data=hs_data;
       
       hs_data->hs_status=D1980_HEADSET_REMOVE;
       hs_data->mic_status=D1980_HS_MIC_REMOVE;
       
	hs_data->sdev.name		= "h2w";
	hs_data->sdev.print_name	= print_switch_name;
	hs_data->sdev.print_state	= print_switch_state;

	ret = switch_dev_register(&hs_data->sdev);
	if (ret < 0)
		goto err;

       hs_data->miscdev.minor=MISC_DYNAMIC_MINOR;
       hs_data->miscdev.name="pm860x_hsdetect"; //"levante_hsdetect";
       hs_data->miscdev.fops	= &d1981_hsdetect_fops;
	ret = misc_register(&hs_data->miscdev);
        
err:
	printk(KERN_DEBUG "Headset driver for android register ret=%d \n", ret);
	return ret;
}

static int __devexit d1981_hs_remove(struct platform_device *pdev)
{
       struct d1981_hs_data *hs_data = platform_get_drvdata(pdev);

       switch_dev_unregister(&hs_data->sdev);
       misc_deregister(&hs_data->miscdev);
       kfree(hs_data);
	return 0;
}

static int d1981_hs_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

static int d1981_hs_resume(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver d1981_hs_driver = {
	.probe		= d1981_hs_probe,
	.remove		= __devexit_p(d1981_hs_remove),
	.suspend	= d1981_hs_suspend,
	.resume		=  d1981_hs_resume,
	.driver		= {
		.name	= "android-headset",
		.owner	= THIS_MODULE,
	},
};

static int __init d1981_hs_init(void)
{
	return platform_driver_register(&d1981_hs_driver);
}

static void __exit d1981_hs_exit(void)
{
	platform_driver_unregister(&d1981_hs_driver);
}

module_init(d1981_hs_init);
module_exit(d1981_hs_exit);

MODULE_DESCRIPTION("d1981_hs emulation driver");
MODULE_LICENSE("GPL");

