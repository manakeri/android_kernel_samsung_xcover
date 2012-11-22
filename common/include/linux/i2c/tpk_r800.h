/*
 * linux/i2c/tpk_r800.h
 *
 *  Copyright (C) 2009 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef __LINUX_I2C_TPK_R800_H
#define __LINUX_I2C_TPK_R800_H

struct touchscreen_platform_data {
	int (*set_power) (int);
};

#endif
