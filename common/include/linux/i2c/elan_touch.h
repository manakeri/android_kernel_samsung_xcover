#ifndef __LINUX_I2C_ELAN_TOUCH_H
#define __LINUX_I2C_ELAN_TOUCH_H

/* linux/i2c/elan_touch.h */

struct elan_touch_platform_data {
	int(*power)(int);
};

#endif
