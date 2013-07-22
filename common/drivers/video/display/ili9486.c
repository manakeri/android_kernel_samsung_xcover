 /* drivers/video/display/ili9486.c
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

#include "ili9486.h"
#include "ili9486_DTC.h"

extern long get_lcd_id(void);

void ili9486_smd_lcd_panel_reset(void)
{
    if (0 == first_reset_flag)
    {
 	first_reset_flag = 1;
	return;
    }
    #if defined(__LCD_DEBUG__)
	   printk("[LCD] ili9486_smd_lcd_panel_reset\n");
    #endif
    
	GPDR0 &= (~(1<<16));                 
	msleep(10);
	SetLCDGpio(0,20);             
	mdelay(1);
	SetLCDGpio(1,20);
	msleep(120);
}

unsigned char  gv_LCD_Id_kernel[3];
#if 0 //def READ_LCD_ID

void Lcd_ID_Read(struct ssp_device *ssp)
{
	unsigned int LCD_data;

	pxa_init_ssp_for_read(ssp);

	pxa_ssp_flush(ssp);	

	pxa_ssp_write_word(ssp,((0xDA << 8) & 0x1ff00));
 	mdelay(1);
	pxa_ssp_read_word(ssp,&LCD_data);

 	gv_LCD_Id_kernel[0] = LCD_data;  

	pxa_ssp_write_word(ssp,((0xDB << 8) & 0x1ff00));
 	mdelay(1);
	pxa_ssp_read_word(ssp,&LCD_data);

 	gv_LCD_Id_kernel[1] = LCD_data;   

 	pxa_ssp_write_word(ssp,((0xDC << 8) & 0x1ff00));
 	mdelay(1);
 	pxa_ssp_read_word(ssp,&LCD_data);

 	gv_LCD_Id_kernel[2] = LCD_data;

	mdelay(5);

	//printk("[LCD] Lcd_ID_Read - gv_LCD_Id_kernel[0] : %x \n",gv_LCD_Id_kernel[0]);
	//printk("[LCD] Lcd_ID_Read - gv_LCD_Id_kernel[1] : %x \n",gv_LCD_Id_kernel[1]);
	//printk("[LCD] Lcd_ID_Read - gv_LCD_Id_kernel[2] : %x \n",gv_LCD_Id_kernel[2]);
    
}
#endif

void ili9486_smd_lcd_power(int on)
{
	int id_read_count = 10;

    #if defined(__LCD_DEBUG__)
	   printk("[LCD]  ili9486_smd_lcd_power\n");
    #endif
	
    if(0 == first_booting_flag){
       first_booting_flag = 1;
       return;
    }
    
	if (!ssp){     

             #if defined(__LCD_DEBUG__)
             printk("[LCD] set SSP for LCD \n");
             #endif
             
		ssp = ssp_lcd_init();

	}else{
              #if defined(__LCD_DEBUG__)
              printk("[LCD] ALready SSP for LCD \n");
              #endif
	}

#if 0
	if((gv_LCD_Id_kernel[1] != 0xc0)&&(gv_LCD_Id_kernel[1] != 0xac))
	{
		ili9486_smd_lcd_panel_reset();
		writel(0xB0C3,io_p2v(0x40e102c8));
		ili9486_smd_lcd_panel_reset();

		do{
			Lcd_ID_Read(ssp);
			//printk("[LCD] Lcd_ID_Read count : %d !! \n",id_read_count);
			
			if(gv_LCD_Id_kernel[1])
				break;

			if(id_read_count == 0)
				break;

			id_read_count--;
		}while(1);

		pxa_init_ssp_for_write(ssp);
	}
#endif
	
	if (on) {
        
            #if defined(__LCD_DEBUG__)
            printk("[LCD] poweron setting start \n");
            #endif


            set_dvfm_constraint();
            
	 
       	msleep(10);

		if(gv_LCD_Id_kernel[1] == 0xc0)
		{
			//printk("[LCD] ili9486_smd_lcd_power, sequence 2 !! \n");
			/* + Power Setting Sequence */
			ssp_send_cmd_para(ARRAY_AND_SIZE(smd_powersetseq1_DTC));
			ssp_send_cmd_para(ARRAY_AND_SIZE(smd_powersetseq2_DTC));
             		ssp_send_cmd_para(ARRAY_AND_SIZE(smd_powersetseq3_DTC));
             		ssp_send_cmd_para(ARRAY_AND_SIZE(smd_powersetseq4_DTC));
            		/* - Power Setting Sequence */

            		/* + Initializing Sequence */
             		ssp_send_cmd_para(ARRAY_AND_SIZE(smd_initseq1_DTC));
             		ssp_send_cmd_para(ARRAY_AND_SIZE(smd_initseq2_DTC));
             		ssp_send_cmd_para(ARRAY_AND_SIZE(smd_initseq3_DTC));
             		ssp_send_cmd_para(ARRAY_AND_SIZE(smd_initseq4_DTC));
             		ssp_send_cmd_para(ARRAY_AND_SIZE(smd_initseq5_DTC));
             		ssp_send_cmd_para(ARRAY_AND_SIZE(smd_initseq6_DTC));
             		ssp_send_cmd_para(ARRAY_AND_SIZE(smd_initseq7_DTC));
             		ssp_send_cmd_para(ARRAY_AND_SIZE(smd_initseq8_DTC));
             		ssp_send_cmd_para(ARRAY_AND_SIZE(smd_initseq9_DTC));
             		ssp_send_cmd_para(ARRAY_AND_SIZE(smd_initseq10_DTC));
             		ssp_send_cmd_para(ARRAY_AND_SIZE(smd_initseq11_DTC));             
            		/* - Initializing Sequence */

            
            		/* + Gamma Setting Sequence */
             		ssp_send_cmd_para(ARRAY_AND_SIZE(smd_gammaseq1_DTC));
             		ssp_send_cmd_para(ARRAY_AND_SIZE(smd_gammaseq2_DTC));
            		/* - Gamma Setting Sequence */


            		/* - Display Setting Sequence */
             		ssp_send_cmd_para(ARRAY_AND_SIZE(smd_dispseq1_DTC));
			
             		ssp_send_cmd_para(ARRAY_AND_SIZE(smd_digitalgammacrtl1_DTC));
             		ssp_send_cmd_para(ARRAY_AND_SIZE(smd_digitalgammacrtl2_DTC));
			 
             		ssp_send_cmd_para(ARRAY_AND_SIZE(smd_dispseq5_DTC));
             		//ssp_send_cmd_para(ARRAY_AND_SIZE(smd_dispseq3));
             		ssp_send_cmd_para(ARRAY_AND_SIZE(smd_dispseq4_DTC));
             		//ssp_send_cmd_para(ARRAY_AND_SIZE(smd_dispseq6));	 
            		/* - Display Setting Sequence */
			ssp_send_cmd_para(ARRAY_AND_SIZE(smd_dispseq6_DTC));

             
            		/* + Sleep Out */
            		//ssp_send_cmd_para(ARRAY_AND_SIZE(smd_normaldispon));
            		/* - Sleep Out */

            		/* + Sleep Out */
            		ssp_send_cmd_para(ARRAY_AND_SIZE(smd_sleepout_DTC));
            		/* - Sleep Out */

            		msleep(120);
            
            		/* + Display On */
            		ssp_send_cmd_para(ARRAY_AND_SIZE(smd_displayon_DTC));
            		/* - Display On */

            		msleep(10);
            		ssp_send_cmd_para(ARRAY_AND_SIZE(smd_CABC1_DTC));
            		ssp_send_cmd_para(ARRAY_AND_SIZE(smd_CABC3_DTC));
		}
		else //if(gv_LCD_Id_kernel[1] == 0xac)
		{
			//printk("[LCD] ili9486_smd_lcd_power, sequence 1 !! \n");
            /* + Power Setting Sequence */
		ssp_send_cmd_para(ARRAY_AND_SIZE(smd_powersetseq1));
		ssp_send_cmd_para(ARRAY_AND_SIZE(smd_powersetseq2));
             ssp_send_cmd_para(ARRAY_AND_SIZE(smd_powersetseq3));
             ssp_send_cmd_para(ARRAY_AND_SIZE(smd_powersetseq4));
            /* - Power Setting Sequence */

            /* + Initializing Sequence */
             ssp_send_cmd_para(ARRAY_AND_SIZE(smd_initseq1));
             ssp_send_cmd_para(ARRAY_AND_SIZE(smd_initseq2));
             ssp_send_cmd_para(ARRAY_AND_SIZE(smd_initseq3));
             ssp_send_cmd_para(ARRAY_AND_SIZE(smd_initseq4));
             ssp_send_cmd_para(ARRAY_AND_SIZE(smd_initseq5));
             ssp_send_cmd_para(ARRAY_AND_SIZE(smd_initseq6));
             ssp_send_cmd_para(ARRAY_AND_SIZE(smd_initseq7));
             ssp_send_cmd_para(ARRAY_AND_SIZE(smd_initseq8));
             ssp_send_cmd_para(ARRAY_AND_SIZE(smd_initseq9));
             ssp_send_cmd_para(ARRAY_AND_SIZE(smd_initseq10));
             ssp_send_cmd_para(ARRAY_AND_SIZE(smd_initseq11));             
            /* - Initializing Sequence */

            
            /* + Gamma Setting Sequence */
             ssp_send_cmd_para(ARRAY_AND_SIZE(smd_gammaseq1));
             ssp_send_cmd_para(ARRAY_AND_SIZE(smd_gammaseq2));
            /* - Gamma Setting Sequence */


            /* - Display Setting Sequence */
             ssp_send_cmd_para(ARRAY_AND_SIZE(smd_dispseq1));
			
             ssp_send_cmd_para(ARRAY_AND_SIZE(smd_digitalgammacrtl1));
             ssp_send_cmd_para(ARRAY_AND_SIZE(smd_digitalgammacrtl2));
			 
             ssp_send_cmd_para(ARRAY_AND_SIZE(smd_dispseq5));
             //ssp_send_cmd_para(ARRAY_AND_SIZE(smd_dispseq3));
             ssp_send_cmd_para(ARRAY_AND_SIZE(smd_dispseq4));
             //ssp_send_cmd_para(ARRAY_AND_SIZE(smd_dispseq6));	 
            /* - Display Setting Sequence */

             
            /* + Sleep Out */
            //ssp_send_cmd_para(ARRAY_AND_SIZE(smd_normaldispon));
            /* - Sleep Out */

            /* + Sleep Out */
            ssp_send_cmd_para(ARRAY_AND_SIZE(smd_sleepout));
            /* - Sleep Out */

            msleep(120);
            
            /* + Display On */
            ssp_send_cmd_para(ARRAY_AND_SIZE(smd_displayon));
            /* - Display On */

            msleep(10);
            ssp_send_cmd_para(ARRAY_AND_SIZE(smd_CABC1));
            ssp_send_cmd_para(ARRAY_AND_SIZE(smd_CABC3));
		}
		

            msleep(20);/*this delay is added because the lcd require time to get init*/

            lcdPanelOnOff_status = on;

            #if defined(__LCD_DEBUG__)
            printk("[LCD] poweron setting end \n");
            #endif

            unset_dvfm_constraint();

	} 
	else 
	{
            #if defined(__LCD_DEBUG__)
             printk("[LCD] LCD OFF\n");
            #endif

             gpio_set_value(backlight_pin, 0);
    	     backlight_status = 0;
            
		if(gv_LCD_Id_kernel[1] == 0xc0)
		{
			ssp_send_cmd_para(ARRAY_AND_SIZE(smd_displayoff_DTC));
	
			msleep(10);
        
			ssp_send_cmd_para(ARRAY_AND_SIZE(smd_sleepin_DTC));
		}
		else //if(gv_LCD_Id_kernel[1] == 0xac)
		{
		ssp_send_cmd_para(ARRAY_AND_SIZE(smd_displayoff));
	
		msleep(10);
        
		ssp_send_cmd_para(ARRAY_AND_SIZE(smd_sleepin));
		}
		
        
    	msleep(120);        
            
		ssp_lcd_deinit(ssp);

		ssp=NULL;

		lcdPanelOnOff_status = on;
}

}
EXPORT_SYMBOL(ili9486_smd_lcd_power);


static struct fb_videomode video_modes[] = {
	[0] = {
		.pixclock       = 10000000,
		.refresh        = 64,
		.xres           = 320,
		.yres           = 480,
		.hsync_len      = 4,
		.left_margin    = 52,//16,
		.right_margin   = 50,//20,
		.vsync_len      = 2,
		.upper_margin   = 12,//6,
		.lower_margin   = 8,//8,
		.sync           = 0,
	},
};

static struct fb_videomode video_modes_DTC[] = {
	[0] = {
		.pixclock       = 10000000,
		.refresh        = 62,
		.xres           = 320,
		.yres           = 480,
		.hsync_len      = 4,
		.left_margin    = 30,//16,
		.right_margin   = 26,//20,
		.vsync_len      = 2,
		.upper_margin   = 12,//6,
		.lower_margin   = 8,//8,
		.sync           = 0,
	},
};


static struct pxa95xfb_mach_info tavor_evb3_lcd_info __initdata = {
	.id                     = "Base",
	.modes                  = video_modes,
	.num_modes              = ARRAY_SIZE(video_modes),
	.pix_fmt_in             = PIX_FMTIN_RGB_24,
#if defined(CONFIG_LCD_FOR_ALKON_REV02)	
	.pix_fmt_out            = PIX_FMTOUT_24_RGB888,
#else
	.pix_fmt_out            = PIX_FMTOUT_18_RGB666,
#endif
	.panel_type             = LCD_Controller_Active,
	.window                 = 0,
	.mixer_id               = 0,
	.zorder                 = 1,
	.converter              = LCD_M2PARALELL_CONVERTER,
	.output                 = OUTPUT_PANEL,
	.active                 = 1,
	.panel_power		= ili9486_smd_lcd_power,
	.reset			= ili9486_smd_lcd_panel_reset,//tavorpv2_saarb_dsi_reset,
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
	.panel_power		= ili9486_smd_lcd_power,
	.reset			= tavorpv2_saarb_dsi_reset,
	.invert_pixclock        = 1,
};

static struct pxa95xfb_mach_info tavor_evb3_lcd_info_DTC __initdata = {
	.id                     = "Base",
	.modes                  = video_modes_DTC,
	.num_modes              = ARRAY_SIZE(video_modes_DTC),
	.pix_fmt_in             = PIX_FMTIN_RGB_24,
#if defined(CONFIG_LCD_FOR_ALKON_REV02)	
	.pix_fmt_out            = PIX_FMTOUT_24_RGB888,
#else
	.pix_fmt_out            = PIX_FMTOUT_18_RGB666,
#endif
	.panel_type             = LCD_Controller_Active,
	.window                 = 0,
	.mixer_id               = 0,
	.zorder                 = 1,
	.converter              = LCD_M2PARALELL_CONVERTER,
	.output                 = OUTPUT_PANEL,
	.active                 = 1,
	.panel_power		= ili9486_smd_lcd_power,
	.reset			= ili9486_smd_lcd_panel_reset,//tavorpv2_saarb_dsi_reset,
	.regulator_initialize	= fb_get_regulator,
	.invert_pixclock        = 1,
};

static struct pxa95xfb_mach_info tavor_evb3_lcd_ovly_info_DTC __initdata = {
	.id                     = "Ovly",
	.modes                  = video_modes_DTC,
	.num_modes              = ARRAY_SIZE(video_modes_DTC),
	.pix_fmt_in             = PIX_FMTIN_RGB_16,
	.pix_fmt_out            = PIX_FMTOUT_18_RGB666,
	.panel_type             = LCD_Controller_Active,
	.window                 = 4,
	.mixer_id               = 0,
	.zorder                 = 0,
	.converter              = LCD_M2PARALELL_CONVERTER,
	.output                 = OUTPUT_PANEL,
	.active                 = 1,
	.panel_power		= ili9486_smd_lcd_power,
	.reset			= tavorpv2_saarb_dsi_reset,
	.invert_pixclock        = 1,
};

 void  alkon_lcd_init(void)
{
	int id_read_count = 10;
	ssp=NULL;
	
      #if defined(__LCD_DEBUG__)
            printk("[LCD] lcd_init\n");
      #endif
      
    gv_LCD_Id_kernel[1] = get_lcd_id();
    printk("[LCD] alkon_lcd_init, g_lcd_id = %x \n",gv_LCD_Id_kernel[1]);

	if(gv_LCD_Id_kernel[1] == 0xc0)
	{
		//printk("[LCD] alkon_lcd_init, sequence 2 !! \n");
		set_pxa95x_fb_info(&tavor_evb3_lcd_info_DTC);
		set_pxa95x_fb_ovly_info(&tavor_evb3_lcd_ovly_info_DTC, 0);
	}
	else //if(gv_LCD_Id_kernel[1] == 0xac)
	{
		//printk("[LCD] alkon_lcd_init, sequence 1 !! \n");
	set_pxa95x_fb_info(&tavor_evb3_lcd_info);
	set_pxa95x_fb_ovly_info(&tavor_evb3_lcd_ovly_info, 0);
	}
    
    backlight_pin = mfp_to_gpio(MFP_PIN_GPIO43);
	if (gpio_request(backlight_pin, "backlight"))
	    printk(KERN_ERR "Backlight Request GPIO_%d failed!\n", backlight_pin);
	gpio_direction_output(backlight_pin,1);	
}
EXPORT_SYMBOL(alkon_lcd_init);





