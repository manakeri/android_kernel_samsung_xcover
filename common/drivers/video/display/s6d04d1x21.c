 /* drivers/video/display/s6d04d1x21.c
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

#include <linux/platform_device.h>
#include <mach/gpio.h>
#include <mach/s6d04d1x21.h>
#include <linux/delay.h>
#include <mach/pxa95xfb.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/machine.h>

#include <mach/mfp-pxa9xx.h>
#include <plat/ssp.h>
#if defined(CONFIG_PMIC_D1980)
#include <linux/d1982/pmic.h>
#endif

#define ARRAY_AND_SIZE(x)       (x), ARRAY_SIZE(x)
#define __LCD_DEBUG__   1

extern struct ssp_device * ssp_lcd_init(void);
extern int ssp_lcd_send_cmd_para(struct ssp_device *ssp, u16 *cmd, int num);
void tavorpv2_saarb_dsi_reset(void) {};

int backlight_pin = 0; 
unsigned char lcdPanelOnOff_status = 0; //pps-a: To maintain lcd panel previous status
struct regulator *vfb; /* TODO: This should be stored in camera structure */
struct ssp_device *ssp;

void ssp_send_cmd_para(u16 *cmd, int num)
{
	int ret;

	if (ssp) {
		ret = ssp_lcd_send_cmd_para(ssp, cmd, num);
		WARN(ret, "ssp_lcd_send_cmd_para returned error (%d)\n", ret);
	}
	else
		WARN(1, "%s: called before *ssp initialized\n", __func__);
}
EXPORT_SYMBOL(ssp_send_cmd_para);

static int lcd_power_control(int enable)
{
	if (enable) {
#if defined(CONFIG_PMIC_D1980)
		if(regulator_is_enabled(vfb) == 0)
#endif
		regulator_enable(vfb);
		// turn on backlight
		gpio_set_value(backlight_pin, 1);
	} else {
		regulator_disable(vfb);
		// turn off backlight
		gpio_set_value(backlight_pin, 0);
	}
	return 0;
}

void setgpio20(int value)
{
	

#define GPLR0         __REG(0x40E00000) //levlel
#define GPDR0         __REG(0x40E0000C) //direction 
#define GSDR0         __REG(0x40E00018) //set
#define GCDR0         __REG(0x40E00024) //clear
	GPDR0 |= 1<<20;
		
	if(value){
		GSDR0 |= 1<<20;
	
	}else
	{
		 GCDR0 |= 1<<20;
	}

}

void s6d04d1_smd_lcd_panel_reset(void)
{
    #if defined(__LCD_DEBUG__)
	   printk("[LCD] s6d04d1_smd_lcd_panel_reset\n");
    #endif
	msleep(10);                 
	GPDR0 &= (~(1<<16));                 
	msleep(10);
	setgpio20(0);                  
	msleep(10);
	mdelay(10);
	setgpio20(1);
	msleep(10);
	mdelay(10);                 
}

void s6d04d1_smd_lcd_power(int on)
{
    #if defined(__LCD_DEBUG__)
	   printk("[LCD] s6d04d1_smd_lcd_power\n");
    #endif
	
	if (!ssp)
		ssp = ssp_lcd_init();

//if(on != lcdPanelOnOff_status) //pps-a
{
	lcdPanelOnOff_status = on;
	if (on) {
        
            #if defined(__LCD_DEBUG__)
                printk("[LCD] LCD ON\n");
            #endif
            	lcd_power_control(1);
		s6d04d1_smd_lcd_panel_reset();
		mdelay(10);	
		ssp_send_cmd_para(ARRAY_AND_SIZE(smd_passwd1));
		ssp_send_cmd_para(ARRAY_AND_SIZE(smd_passwd2));
    		ssp_send_cmd_para(ARRAY_AND_SIZE(smd_disctl));
    		ssp_send_cmd_para(ARRAY_AND_SIZE(smd_pwrctl));
		ssp_send_cmd_para(ARRAY_AND_SIZE(smd_vcmctll));
		ssp_send_cmd_para(ARRAY_AND_SIZE(smd_pixel_format));
		ssp_send_cmd_para(ARRAY_AND_SIZE(smd_tearing_lineon));
		ssp_send_cmd_para(ARRAY_AND_SIZE(smd_memory_data));
		ssp_send_cmd_para(ARRAY_AND_SIZE(smd_column_addset));
		ssp_send_cmd_para(ARRAY_AND_SIZE(smd_page_addset));
		ssp_send_cmd_para(ARRAY_AND_SIZE(smd_srcctl));
		ssp_send_cmd_para(ARRAY_AND_SIZE(smd_ifctl));
		ssp_send_cmd_para(ARRAY_AND_SIZE(smd_panelctl));
		ssp_send_cmd_para(ARRAY_AND_SIZE(smd_gammasel1));
  		ssp_send_cmd_para(ARRAY_AND_SIZE(smd_pgammactl1)); 		
  		ssp_send_cmd_para(ARRAY_AND_SIZE(smd_gammasel2));
		ssp_send_cmd_para(ARRAY_AND_SIZE(smd_pgammactl2));
		ssp_send_cmd_para(ARRAY_AND_SIZE(smd_gammasel3));
		ssp_send_cmd_para(ARRAY_AND_SIZE(smd_pgammactl3));
		ssp_send_cmd_para(ARRAY_AND_SIZE(smd_slpout));
		mdelay(120);
		ssp_send_cmd_para(ARRAY_AND_SIZE(smd_dison));
		ssp_send_cmd_para(ARRAY_AND_SIZE(smd_wrctrld_on));
		ssp_send_cmd_para(ARRAY_AND_SIZE(brightness01));
		mdelay(40);

	} else {
            #if defined(__LCD_DEBUG__)
                 printk("[LCD] LCD OFF\n");
            #endif

		ssp_send_cmd_para(ARRAY_AND_SIZE(brightness_off));
		ssp_send_cmd_para(ARRAY_AND_SIZE(smd_disoff));
		mdelay(40);
		ssp_send_cmd_para(ARRAY_AND_SIZE(smd_slpin));
    		mdelay(120);        
    		setgpio20(0);
		mdelay(10);

            	lcd_power_control(0);
	}
}
}
EXPORT_SYMBOL(s6d04d1_smd_lcd_power);

void fb_get_regulator(void *data)
{
	int ret = 0;
	
	struct platform_device *pdev = data;
#if defined(CONFIG_PMIC_D1980)
	vfb = regulator_get(&pdev->dev, REGULATOR_LCD);
#else
	vfb = regulator_get(&pdev->dev, "v_lcd");
#endif
	if (IS_ERR(vfb)) {	
		vfb = NULL;
		WARN(1, "REGULATOR VFB IS_ERR");
	}
}
#if defined(CONFIG_PXA950_HDMI) 
static struct fb_videomode video_modes[] = {
	[0] = {
		.pixclock       = 41701,
		.refresh        = 60,
		.xres           = 640,
		.yres           = 480,
		.hsync_len      = 76,
		.left_margin    = 5,
		.right_margin   = 97,
		.vsync_len      = 1,
		.upper_margin   = 0,
		.lower_margin   = 42,
		.sync           = 0,
	},
};
#else
static struct fb_videomode video_modes[] = {
	[0] = {
		.pixclock       = 178600,
		.refresh        = 80,
		.xres           = 320,
		.yres           = 480,
		.hsync_len      = 2,
		.left_margin    = 16,
		.right_margin   = 16,
		.vsync_len      = 4,
		.upper_margin   = 8,
		.lower_margin   = 8,
		.sync           = 0,
	},
};
#endif

static struct pxa95xfb_mach_info tavor_evb3_lcd_info __initdata = {
	.id                     = "Base",
	.modes                  = video_modes,
	.num_modes              = ARRAY_SIZE(video_modes),
	.pix_fmt_in             = PIX_FMTIN_RGB_24,
	.pix_fmt_out            = PIX_FMTOUT_24_RGB888,
	.panel_type             = LCD_Controller_Active,
	.window                 = 0,
	.mixer_id               = 0,
	.zorder                 = 1,
	.converter              = LCD_M2PARALELL_CONVERTER,
	.output                 = OUTPUT_PANEL,
	.active                 = 1,
	.panel_power		= s6d04d1_smd_lcd_power,
	.reset			= tavorpv2_saarb_dsi_reset,
	.regulator_initialize	= fb_get_regulator,
	.invert_pixclock        = 1,
};

static struct pxa95xfb_mach_info tavor_evb3_lcd_ovly_info __initdata = {
	.id                     = "Ovly",
	.modes                  = video_modes,
	.num_modes              = ARRAY_SIZE(video_modes),
	.pix_fmt_in             = PIX_FMTIN_RGB_16,
	.pix_fmt_out            = PIX_FMTOUT_24_RGB888,
	.panel_type             = LCD_Controller_Active,
	.window                 = 4,
	.mixer_id               = 0,
	.zorder                 = 0,
	.converter              = LCD_M2PARALELL_CONVERTER,
	.output                 = OUTPUT_PANEL,
	.active                 = 1,
	.panel_power		= s6d04d1_smd_lcd_power,
	.reset			= tavorpv2_saarb_dsi_reset,
	.invert_pixclock        = 1,
};

 void  gforce_lcd_init(void)
{
      #if defined(__LCD_DEBUG__)
            printk("[LCD] lcd_init\n");
      #endif
      
	set_pxa95x_fb_info(&tavor_evb3_lcd_info);
	set_pxa95x_fb_ovly_info(&tavor_evb3_lcd_ovly_info, 0);
    
    #if defined(CONFIG_MACH_JETTA)
    backlight_pin = mfp_to_gpio(MFP_PIN_GPIO84);
    #else
    backlight_pin = mfp_to_gpio(MFP_PIN_GPIO43);
    #endif
    
	if (gpio_request(backlight_pin, "backlight"))
	    printk(KERN_ERR "Backlight Request GPIO_%d failed!\n", backlight_pin);
	gpio_direction_output(backlight_pin,1);	
}
EXPORT_SYMBOL(gforce_lcd_init);





