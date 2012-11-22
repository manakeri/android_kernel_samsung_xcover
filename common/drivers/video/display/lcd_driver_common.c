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

#include "lcd_driver_common.h"


#if defined (CONFIG_LCD_DB7431)
#include "db7431.c"
#else
#include "ili9486.c"
#endif

void ssp_send_cmd_para(u16 *cmd, int num)
{
	int ret;

	if (ssp) 
	{
		ret = ssp_lcd_send_cmd_para(ssp, cmd, num);

		if(ret != 0)
		{
			printk("ssp_lcd_send_cmd_para returned error (%d)\n", ret);
		}
	}
	else
		printk("%s: called before *ssp initialized\n", __func__);
}


unsigned char IsLCDon(void)
{
	return lcdPanelOnOff_status;
}



void SetLCDGpio(int value, int gpionum)
{
	

#define GPLR0         __REG(0x40E00000) //levlel
#define GPDR0         __REG(0x40E0000C) //direction 
#define GSDR0         __REG(0x40E00018) //set
#define GCDR0         __REG(0x40E00024) //clear
	GPDR0 |= 1<<gpionum;
		
	if(value){
		GSDR0 |= 1<<gpionum;
	
	}else
	{
		 GCDR0 |= 1<<gpionum;
	}

}

#ifdef CONFIG_PXA95x_DVFM

void set_dvfm_constraint(void)
{
	/* Disable Lowpower mode */
	dvfm_disable_op_name("D0CS", dvfm_dev_idx);
	dvfm_disable_op_name("D1", dvfm_dev_idx);
	dvfm_disable_op_name("D2", dvfm_dev_idx);
	dvfm_disable_op_name("CG", dvfm_dev_idx);
	/* Disable below frequency since they only support DSI clock up to 156Mhz */
	dvfm_disable_op_name("156M", dvfm_dev_idx);
	dvfm_disable_op_name("208M", dvfm_dev_idx);
	dvfm_disable_op_name("156M_HF", dvfm_dev_idx);

}

void unset_dvfm_constraint(void)
{
	/* Enable Lowpower mode */
	dvfm_enable_op_name("D0CS", dvfm_dev_idx);
	dvfm_enable_op_name("D1", dvfm_dev_idx);
	dvfm_enable_op_name("D2", dvfm_dev_idx);
	dvfm_enable_op_name("CG", dvfm_dev_idx);
	/* Enable frequency */
	dvfm_enable_op_name("156M", dvfm_dev_idx);
	dvfm_enable_op_name("208M", dvfm_dev_idx);
	dvfm_enable_op_name("156M_HF", dvfm_dev_idx);
}

#else
static void set_dvfm_constraint(void) {}
static void unset_dvfm_constraint(void) {}
#endif

#if defined (CONFIG_LCD_ESD_CHECK)
void lcd_esd_reset ()
{
#if defined(__LCD_DEBUG__)
	printk("[LCD]  %s = Reinitialising lcd for ESD \n",__func__);
#endif

#if defined (CONFIG_LCD_DB7431)
		db7431_smd_lcd_panel_reset();
		db7431_smd_lcd_power(1);
#endif
}
#endif

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


