/*
 * d1982 Battery/Power module declarations.
  *
 * Copyright(c) 2011 Dialog Semiconductor Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 *
 */

#ifndef __LINUX_D1982_BAT_UTIL_H
#define __LINUX_D1982_BAT_UTIL_H

#include <linux/power_supply.h>
#include <linux/mutex.h>
#include <linux/d1982/core.h>


typedef enum {
    D1980_ADC_READ_AUTO = 0,
    D1980_ADC_READ_MANUAL
} adc_read_type;


int     d1980_set_threshold(u16 lowBatThr, u16 tempBatMaxThr, u16 tempBatMinThr);

int     d1980_set_threshold_vbat(u16 lowBat, u16 upperBat);

int     d1980_set_threshold_temp(u16 lowTemp, u16 upperTemp);

int d1980_set_chargerDetect(u8 data);

u8      d1980_get_TAdetect(void);

u16     d1980_get_Voltage(void);

u16     d1980_get_Temperature(void);

u16     d1980_get_VFdetection(void);

u8      d1980_get_chargerDetect(void);

void    d1980_set_adc_mode(adc_read_type adc_type);


#endif /* __LINUX_D1982_BAT_UTIL_H */
