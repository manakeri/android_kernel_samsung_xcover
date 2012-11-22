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

#include <linux/platform_device.h>
#include <mach/gpio.h>
#include <linux/delay.h>
#include <mach/pxa95xfb.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/machine.h>

#include <mach/mfp-pxa9xx.h>
#include <plat/ssp.h>
#if defined(CONFIG_PMIC_D1980)
#include <linux/d1982/pmic.h>
#endif

#ifdef CONFIG_PXA95x_DVFM
#include <mach/dvfm.h>
#include <mach/pxa95x_dvfm.h>
#endif

#define ARRAY_AND_SIZE(x)       (x), ARRAY_SIZE(x)
#define __LCD_DEBUG__

extern struct ssp_device * ssp_lcd_init(void);
extern void ssp_lcd_deinit(struct ssp_device *ssp);
extern int ssp_lcd_send_cmd_para(struct ssp_device *ssp, u16 *cmd, int num);

static int wakeup_tuned_level = 0x80;
EXPORT_SYMBOL(wakeup_tuned_level);

void tavorpv2_saarb_dsi_reset(void) {};

int backlight_pin = 0; 
int backlight_status = 1;
unsigned char lcdPanelOnOff_status = 1; //pps-a: To maintain lcd panel previous status
int first_booting_flag = 0;
int first_reset_flag = 0;
int dvfm_dev_idx;
EXPORT_SYMBOL(backlight_status);

EXPORT_SYMBOL(lcdPanelOnOff_status);

static DEFINE_SPINLOCK(LCD_lock);

struct regulator *vfb; /* TODO: This should be stored in camera structure */
struct ssp_device *ssp;

void ssp_send_cmd_para(u16 *cmd, int num);
EXPORT_SYMBOL(ssp_send_cmd_para);


void fb_get_regulator(void *data);

unsigned char IsLCDon(void);
EXPORT_SYMBOL(IsLCDon);


void set_dvfm_constraint(void);
void unset_dvfm_constraint(void);

void SetLCDGpio(int value, int gpionum);

#if defined (CONFIG_LCD_ESD_CHECK)
void lcd_esd_reset (void);
EXPORT_SYMBOL(lcd_esd_reset);
#endif

