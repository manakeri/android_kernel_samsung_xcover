 /* drivers/video/display/db7431.c
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

#include "db7431.h"

void db7431_smd_lcd_panel_reset(void)
{
    if (0 == first_reset_flag)
    {
 	first_reset_flag = 1;
	return;
    }
    #if defined(__LCD_DEBUG__)
	   printk("[LCD] db7431_smd_lcd_panel_reset\n");
    #endif
        
	GPDR0 &= (~(1<<16));                 
	SetLCDGpio(0,20);                  
	mdelay(2);
	SetLCDGpio(1,20);
	msleep(120);
}

void db7431_smd_lcd_power(int on)
{
    #if defined(__LCD_DEBUG__)
	   printk("[LCD]  db7431_smd_lcd_power\n");
    #endif
	
    if(0 == first_booting_flag){
       first_booting_flag = 1;
       return;
    }
	
	if (!ssp){     

             #if defined(__LCD_DEBUG__)
             printk("[LCD] set SSP for LCD \n");
             #endif
             
		msleep(20); // added for LCD CS timing problem.
             
		ssp = ssp_lcd_init();

	}else{
              #if defined(__LCD_DEBUG__)
              printk("[LCD] ALready SSP for LCD \n");
              #endif
	}


	if (on) {
        
            #if defined(__LCD_DEBUG__)
            printk("[LCD] poweron setting start \n");
            #endif


            /* + PNL Contro */
		ssp_send_cmd_para(ARRAY_AND_SIZE(smd_PNL_Manufacturer));
		ssp_send_cmd_para(ARRAY_AND_SIZE(smd_PNL_SET_POR_Mode));
             ssp_send_cmd_para(ARRAY_AND_SIZE(smd_PNL_Enter_Invert_Mode));
             ssp_send_cmd_para(ARRAY_AND_SIZE(smd_PNL_SET_Address_Mode));
             ssp_send_cmd_para(ARRAY_AND_SIZE(smd_PNL_Panel_Driving));
             ssp_send_cmd_para(ARRAY_AND_SIZE(smd_PNL_Source_Control));
             ssp_send_cmd_para(ARRAY_AND_SIZE(smd_PNL_Gate_Interface));
             ssp_send_cmd_para(ARRAY_AND_SIZE(smd_PNL_Display_H_Timming));
             ssp_send_cmd_para(ARRAY_AND_SIZE(smd_PNL_RGB_Sync_Option));
             ssp_send_cmd_para(ARRAY_AND_SIZE(smd_PNL_Logic_Test_Option));
            /* - PNL Contro */

            /* + Power Control */
		ssp_send_cmd_para(ARRAY_AND_SIZE(smd_Power_Bias_Current));
		ssp_send_cmd_para(ARRAY_AND_SIZE(smd_Power_DDV));
             ssp_send_cmd_para(ARRAY_AND_SIZE(smd_Power_Gamma_Control_Ref));
             ssp_send_cmd_para(ARRAY_AND_SIZE(smd_Power_DCDC));
             ssp_send_cmd_para(ARRAY_AND_SIZE(smd_Power_VCL));
            /* - Power Control */

            /* - Gamma Control */
             ssp_send_cmd_para(ARRAY_AND_SIZE(smd_gammaseq_RED));
             ssp_send_cmd_para(ARRAY_AND_SIZE(smd_gammaseq_GREEN));
             ssp_send_cmd_para(ARRAY_AND_SIZE(smd_gammaseq_BLUE));
            /* - Gamma Control */

            /* + Sleep Out */
            ssp_send_cmd_para(ARRAY_AND_SIZE(smd_sleepout));
            /* - Sleep Out */

            msleep(20);

		/* + Initializing Sequence Vcom Voltage Set */
            ssp_send_cmd_para(ARRAY_AND_SIZE(smd_Vcom_Voltage_Set));
            /* - Initializing Sequence Vcom Voltage Set */

            
            /* + Display On */
            ssp_send_cmd_para(ARRAY_AND_SIZE(smd_displayon));
            /* - Display On */
       	    
            msleep(130);
            ssp_send_cmd_para(ARRAY_AND_SIZE(smd_CABC_Turn_on3));
			
            #if defined(__LCD_DEBUG__)
            printk("[LCD] poweron setting end \n");
            #endif


           	lcdPanelOnOff_status = on;

	} else {
            #if defined(__LCD_DEBUG__)
             printk("[LCD] LCD OFF\n");
            #endif

             gpio_set_value(backlight_pin, 0);
    	backlight_status = 0;


		ssp_send_cmd_para(ARRAY_AND_SIZE(smd_displayoff));
	
		msleep(40);//mdelay(40);
        
		ssp_send_cmd_para(ARRAY_AND_SIZE(smd_sleepin));
        
    		msleep(120);       

		ssp_lcd_deinit(ssp);
		ssp=NULL;
            	
		lcdPanelOnOff_status = on;            	
	}

}
EXPORT_SYMBOL(db7431_smd_lcd_power);


static struct fb_videomode video_modes[] = {
	[0] = {
		.pixclock       = 10800000,
		.refresh        = 61,
		.xres           = 480,
		.yres           = 320,
		.hsync_len      = 4,
		.left_margin    = 10,
		.right_margin   = 44,
		.vsync_len      = 1,
		.upper_margin   = 7,
		.lower_margin   = 6,
		.sync           = 0,
	},
};


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
	.panel_power		= db7431_smd_lcd_power,
	.reset			= db7431_smd_lcd_panel_reset,//tavorpv2_saarb_dsi_reset,
	.regulator_initialize	= fb_get_regulator,
	.invert_pixclock        = 1,
};

static struct pxa95xfb_mach_info tavor_evb3_lcd_ovly_info __initdata = {
	.id                     = "Ovly",
	.modes                  = video_modes,
	.num_modes              = ARRAY_SIZE(video_modes),
	.pix_fmt_in             = PIX_FMTIN_RGB_16,
	.pix_fmt_out            = PIX_FMTOUT_18_RGB666,
	.panel_type             = LCD_Controller_Active,
	.window                 = 4,
	.mixer_id               = 0,
	.zorder                 = 0,
	.converter              = LCD_M2PARALELL_CONVERTER,
	.output                 = OUTPUT_PANEL,
	.active                 = 1,
	.panel_power		= db7431_smd_lcd_power,
	.reset			= tavorpv2_saarb_dsi_reset,
	.invert_pixclock        = 1,
};

 void  jetta_lcd_init(void)
{
      #if defined(__LCD_DEBUG__)
            printk("[LCD] jetta_lcd_init\n");
      #endif
      
	set_pxa95x_fb_info(&tavor_evb3_lcd_info);
	set_pxa95x_fb_ovly_info(&tavor_evb3_lcd_ovly_info, 0);
      #if defined(CONFIG_MACH_JETTA_EMUL)
    backlight_pin = mfp_to_gpio(MFP_PIN_GPIO84);
      #else
    backlight_pin = mfp_to_gpio(MFP_PIN_GPIO43);
      #endif	  
	if (gpio_request(backlight_pin, "backlight"))
	    printk(KERN_ERR "Backlight Request GPIO_%d failed!\n", backlight_pin);
	gpio_direction_output(backlight_pin,1);	
	
}
EXPORT_SYMBOL(jetta_lcd_init);





