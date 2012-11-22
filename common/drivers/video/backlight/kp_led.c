/* drivers/video/backlight/levante_kp_bl.c
 *
 * Author:	yhanin
 * Created:	May 25, 2009
 * Copyright:	Marvell International Ltd. All Rights Reserved
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

#if defined(CONFIG_PMIC_D1980)
#include <linux/d1982/pmic.h>
#else
//#include <mach/levante.h>
//#include <mach/levante_hw.h>
#endif

#include <linux/regulator/consumer.h>
#include <linux/regulator/machine.h>

#define FORCE_REF_GP_AND_OSC_VOTING

#define KEYPAD_LED_MAX	100
#define	KEYPAD_LED_MIN	0
#define LDO6_EN1	(1 << 0)
#define KEYPAD_BL_DEBUG      0

static int levante_kp_bl_suspended;
static int current_intensity = 0;
struct regulator *kp_bl_regulator; 


void kp_bl_power_on(unsigned char intensity)
{
	
	int ret;
#if defined(CONFIG_PMIC_D1980)
	printk(KERN_INFO "[TOUCH_LED] kp_bl_power_on. Enable [%d] \n", regulator_is_enabled(kp_bl_regulator));
    if(regulator_is_enabled(kp_bl_regulator) == 0)
        regulator_enable(kp_bl_regulator);
#else
      #if KEYPAD_BL_DEBUG
	printk(KERN_INFO "------->kp_bl_power_on\n");
      #endif
       ret = regulator_is_enabled(kp_bl_regulator);

      #if KEYPAD_BL_DEBUG
	printk(KERN_INFO "------->regulator_is_enabled ret = %d \n", ret);
      #endif

      ret =  regulator_get_voltage(kp_bl_regulator);

      #if KEYPAD_BL_DEBUG
	printk(KERN_INFO "------->regulator_get_voltage ret = %d \n", ret);
      #endif
      
       ret = regulator_enable(kp_bl_regulator);
      
      #if KEYPAD_BL_DEBUG
	printk(KERN_INFO "------->regulator_enable ret = %d \n", ret);
      #endif
#endif	

}

void kp_bl_power_off(void)
{
	unsigned long flags;
	int ret;	

#if defined(CONFIG_PMIC_D1980)
	printk(KERN_INFO "[TOUCH_LED] kp_bl_power_off. Enable [%d] \n", regulator_is_enabled(kp_bl_regulator));
    if(regulator_is_enabled(kp_bl_regulator))
        ret = regulator_disable(kp_bl_regulator);
#else
      #if KEYPAD_BL_DEBUG
	printk(KERN_INFO "------->kp_bl_power_off \n");
      #endif

      ret = regulator_is_enabled(kp_bl_regulator);

      #if KEYPAD_BL_DEBUG
	printk(KERN_INFO "------->regulator_is_enabled ret = %d \n", ret);
      #endif
      
      ret =  regulator_get_voltage(kp_bl_regulator);

      #if KEYPAD_BL_DEBUG
	printk(KERN_INFO "------->regulator_get_voltage ret = %d \n", ret);
      #endif
      
      ret = regulator_disable(kp_bl_regulator);

      #if KEYPAD_BL_DEBUG
	printk(KERN_INFO "------->regulator_force_disable ret = %d \n", ret);
      #endif
#endif	
}

/* input: intensity in percentage 0% - 100% */
static int levante_kp_bl_send_intensity(struct backlight_device *bd)
{
	int user_intensity = bd->props.brightness;
	int plat_intensity = 0;
	u8 wled2b = 0;

       #if KEYPAD_BL_DEBUG
	printk(KERN_INFO "------->levante_kp_bl_send_intensity = %d current_intensity = %d\n", user_intensity, current_intensity);
      #endif

	if (bd->props.power != FB_BLANK_UNBLANK)
		user_intensity = 0;
	if (bd->props.fb_blank != FB_BLANK_UNBLANK)
		user_intensity = 0;
	if (levante_kp_bl_suspended)
		user_intensity = 0;

	if(user_intensity >= 100)
		plat_intensity = 0xFF;
	else
		plat_intensity = (user_intensity*256/100); // convert precentage to WLED_DUTY

	/*
	*If current brightness >= 100, the PORTOFINO_WLED_100DUTY bit must has already been
	*set, if the user intensity is < 100, PORTOFINO_WLED_100DUTY bit must be cleared.
	*If the current brightness is < 100, the PORTOFINO_WLED_100DUTY must has already
	*been cleared, if user intensity >= 100, PORTOFINO_WLED_100DUTY bit must be set.
	
	if(current_intensity >= 100 && user_intensity < 100){
		portofino_rw_lock();
		portofino_read(PORTOFINO_WLED2B, &wled2b);
		portofino_write(PORTOFINO_WLED2B, wled2b & ~PORTOFINO_WLED_100DUTY);
		portofino_rw_unlock();
	}else if(current_intensity < 100 && user_intensity >= 100){
		portofino_rw_lock();
		portofino_read(PORTOFINO_WLED2B, &wled2b);
		portofino_write(PORTOFINO_WLED2B, wled2b | PORTOFINO_WLED_100DUTY);
		portofino_rw_unlock();
	}

	if (plat_intensity != 0x0) {
		levante_ref_gp_and_osc_get(WLED2_DUTY);
		portofino_write(PORTOFINO_WLED2A, plat_intensity);
	} else {
		portofino_write(PORTOFINO_WLED2A, plat_intensity);
		levante_ref_gp_and_osc_release(WLED2_DUTY);
	}
     */
	if (user_intensity && current_intensity == 0) {
	    kp_bl_power_on(plat_intensity);
	} else if (user_intensity == 0 && current_intensity != 0) {
		kp_bl_power_off();
	}

	current_intensity = user_intensity;

	return 0;
}

#ifdef CONFIG_PM
static int levante_kp_bl_suspend(struct platform_device *pdev, pm_message_t state)
{
	//struct backlight_device *bd = platform_get_drvdata(pdev);

	levante_kp_bl_suspended = 1;
	//kp_bl_power_off();
	return 0;
}

static int levante_kp_bl_resume(struct platform_device *pdev)
{
	//struct backlight_device *bd = platform_get_drvdata(pdev);

	levante_kp_bl_suspended = 0;
	//kp_bl_power_on(bd);
	return 0;
}
#else
#define levante_bl_suspend	NULL
#define levante_bl_resume	NULL
#endif

static int levante_bl_set_intensity(struct backlight_device *bd)
{
	levante_kp_bl_send_intensity(bd);
	return 0;
}

static int levante_bl_get_intensity(struct backlight_device *bd)
{
    #if KEYPAD_BL_DEBUG
    printk(KERN_INFO "------->levante_kp_bl(levante_bl_get_intensity)= %d\n",current_intensity);
    #endif
    
	return current_intensity;
}

static struct backlight_ops levante_bl_ops = {
	.get_brightness = levante_bl_get_intensity,
	.update_status  = levante_bl_set_intensity,
};

static int __init levante_kp_bl_probe(struct platform_device *pdev)
{
	struct backlight_device *bd;
       struct backlight_properties props;
      
      memset(&props, 0, sizeof(struct backlight_properties));
      props.brightness = KEYPAD_LED_MAX;
      props.max_brightness = KEYPAD_LED_MAX;
      //props.power = FB_BLANK_UNBLANK;

	printk(KERN_INFO "------->levante_kp_bl_probe\n");

	bd = backlight_device_register("backlight-1", &pdev->dev, NULL,
		    &levante_bl_ops, &props);
	if (IS_ERR(bd))
		return PTR_ERR(bd);

	platform_set_drvdata(pdev, bd);

       /* + to get touch regulator */
#if defined(CONFIG_PMIC_D1980)
    kp_bl_regulator = regulator_get(NULL, REGULATOR_TOUCHKEY_LED);
#else
       kp_bl_regulator = regulator_get(NULL, "v_kpled");
#endif
       if (IS_ERR(kp_bl_regulator)) {	
		kp_bl_regulator = NULL;
		printk("[KP_BL] failed to get regulator\n");
	}
      /* - to get touch regulator */

#if defined(CONFIG_PMIC_D1980)
    regulator_set_voltage(kp_bl_regulator, OUTPUT_3_3V, OUTPUT_3_3V);
#else
#if !defined(FORCE_REF_GP_AND_OSC_VOTING)
{
	unsigned char val= 0x00;
	/* turn on BOOST_EN to enable lcd backlight */
	/* allow WLED strings you need to allow reference voltage */
	portofino_rw_lock();
 	portofino_read(PORTOFINO_VSYS, &val);
	if(!(val & PORTOFINO_VSYS_ENABLE)){
		val |= PORTOFINO_VSYS_ENABLE;
		portofino_write(PORTOFINO_VSYS, val);
	}
	portofino_rw_unlock();

	/* allow WLED strings you need to allow reference frequency*/
	portofino_rw_lock();
 	portofino_read(PORTOFINO_USB_MISC, &val);
	if(!(val & PRTOFINIO_USB_MISC_OSC_EN_BIT)){
		val |= PRTOFINIO_USB_MISC_OSC_EN_BIT;
		portofino_write(PORTOFINO_USB_MISC, val);
	}
	portofino_rw_unlock();
}
	printk(KERN_INFO "------->levante_kp_bl_probe:PORTOFINO_VSYS+PORTOFINO_USB_MISC\n");
#else
	printk(KERN_INFO "------->levante_kp_bl_probe: Using voting for PORTOFINO_VSYS+PORTOFINO_USB_MISC\n");
#endif /* FORCE_REF_GP_AND_OSC_VOTING */
#endif /* !CONFIG_PMIC_D1980 */

	bd->props.power = FB_BLANK_UNBLANK;
	bd->props.brightness = KEYPAD_LED_MAX;
	bd->props.max_brightness = KEYPAD_LED_MAX;
	//levante_kp_bl_send_intensity(bd);

	return 0;
}

static int levante_kp_bl_remove(struct platform_device *pdev)
{
	struct backlight_device *bd = platform_get_drvdata(pdev);

	bd->props.brightness = 0;
	bd->props.power = 0;
	levante_kp_bl_send_intensity(bd);

	backlight_device_unregister(bd);

	return 0;
}

static struct platform_driver levante_bl_driver = {
	.probe		= levante_kp_bl_probe,
	.remove		= levante_kp_bl_remove,
#ifdef	CONFIG_PM
	.suspend	= levante_kp_bl_suspend,
	.resume		= levante_kp_bl_resume,
#endif
	.driver		= {
		.name	= "keypad-led",
	},
};

static int __init levante_backlight_init(void)
{
	return platform_driver_register(&levante_bl_driver);
}

static void __exit levante_backlight_exit(void)
{
	platform_driver_unregister(&levante_bl_driver);
}

late_initcall(levante_backlight_init);
module_exit(levante_backlight_exit);
MODULE_LICENSE("GPL");
