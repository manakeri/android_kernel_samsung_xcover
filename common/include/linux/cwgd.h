/* cwgd.h - header file for CyWee digital 3-axis gyroscope
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
#ifndef __CWGD_H__
#define __CWGD_H__

#include <linux/ioctl.h>

#define CWGD_IOCTL_BASE 'w'

#define CWGD_I2C_NAME "cwgd"

#define CWGD_SET_ENABLE			_IOW(CWGD_IOCTL_BASE, 1, int)
#define CWGD_GET_ENABLE			_IOR(CWGD_IOCTL_BASE, 2, int)
#define CWGD_SET_RANGE			_IOW(CWGD_IOCTL_BASE, 3, int)
#define CWGD_GET_RANGE			_IOW(CWGD_IOCTL_BASE, 4, int)
#define CWGD_SET_DELAY			_IOW(CWGD_IOCTL_BASE, 5, int)
#define CWGD_GET_DELAY			_IOW(CWGD_IOCTL_BASE, 6, int)
#define CWGD_READ_GYRO_VALUES	_IOR(CWGD_IOCTL_BASE, 7, int[3])
#define CWGD_SELF_TEST			_IOW(CWGD_IOCTL_BASE, 8, int)
#define CWGD_GET_TEMPERATURE	_IOR(CWGD_IOCTL_BASE, 9, char)
#define CWGD_SET_BIAS			_IOW(CWGD_IOCTL_BASE, 10, int[3])

#define CWGD_FS_250DPS	0x00
#define CWGD_FS_500DPS	0x10
#define CWGD_FS_2000DPS	0x20

#define CWGD_ST_NORMAL		0x00
#define CWGD_ST_POSITIVE	0x02
#define CWGD_ST_NEGATIVE	0x06

#ifdef __KERNEL__
struct cwgd_platform_data {
	int (*set_power) (int);
	int axes[9];
};
#endif /* __KERNEL */

#endif /* __CWGD_H__ */
