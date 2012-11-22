/* cwmi.h - header file for CyWee digital 6-axis motion sensor
 *
 * Copyright (C) 2010 CyWee Group Ltd.
 * Author: Joe Wei <joewei@cywee.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#ifndef __CWMI_H__
#define __CWMI_H__

#include <linux/ioctl.h>
#define I2C_ACC_NAME	"cwmi_acc"
#define I2C_MAG_NAME	"cwmi_mag"

#define CWMI_IOCTL_BASE 'w'

#define CWMI_SET_ENABLE		_IOW(CWMI_IOCTL_BASE, 1, int)
#define CWMI_GET_ENABLE		_IOW(CWMI_IOCTL_BASE, 2, int)
#define CWMI_SET_RANGE		_IOW(CWMI_IOCTL_BASE, 3, int)
#define CWMI_GET_RANGE		_IOW(CWMI_IOCTL_BASE, 4, int)
#define CWMI_SET_DELAY		_IOW(CWMI_IOCTL_BASE, 5, int)
#define CWMI_GET_DELAY		_IOW(CWMI_IOCTL_BASE, 6, int)
#define CWMI_READ_ACC_VALUES	_IOW(CWMI_IOCTL_BASE, 7, int)

/***********************************************************
 * Magnetometer Defines
 ***********************************************************/

/* Output data rate */
#define ODR_30_MAG				0x14
#define ODR_75_MAG				0x18
#define ODR_220_MAG				0x1C

/* Operating mode */
#define MAG_MODE_CONTINUOUS		0x00
#define MAG_MODE_SINGLE			0x01
#define MAG_MODE_SLEEP			0x02

#ifdef __KERNEL__
struct cwmi_platform_data {
	int (*set_power) (int);
	int axes[9];
};
#endif /* __CWMI_H__ */

#endif /* __CWMI_H__ */
