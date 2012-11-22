/*
 * drivers/hwmon/yas529.h
 *
 * Copyright 2010 Marvell International Ltd.
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

#ifndef _YAS529_H_
#define _YAS529_H_

#define YAS529_DEFAULT_THRESHOLD            (1)
#define YAS529_DEFAULT_DISTORTION           (15)
#define YAS529_DEFAULT_SHAPE                (0)

#ifndef FALSE
#define FALSE (0)
#endif

#ifndef TRUE
#define TRUE (!(0))
#endif

#ifndef NELEMS
#define NELEMS(a) (ARRAY_SIZE(a))
#endif

#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/kernel.h>
#include <linux/i2c.h>

struct utimeval {
	int32_t tv_sec;
	int32_t tv_msec;
};

struct utimer {
	struct utimeval prev_time;
	struct utimeval total_time;
	struct utimeval delay_ms;
};

static int utimeval_init(struct utimeval *val);
static int utimeval_is_initial(struct utimeval *val);
static int utimeval_is_overflow(struct utimeval *val);
static struct utimeval utimeval_plus(struct utimeval *first,
				     struct utimeval *second);
static struct utimeval utimeval_minus(struct utimeval *first,
				      struct utimeval *second);
static int utimeval_greater_than(struct utimeval *first,
				 struct utimeval *second);
static int utimeval_greater_or_equal(struct utimeval *first,
				     struct utimeval *second);
static int utimeval_greater_than_zero(struct utimeval *val);
static int utimeval_less_than_zero(struct utimeval *val);
static struct utimeval *msec_to_utimeval(struct utimeval *result,
					 uint32_t msec);
static uint32_t utimeval_to_msec(struct utimeval *val);

static struct utimeval utimer_calc_next_time(struct utimer *ut,
					     struct utimeval *cur);
static struct utimeval utimer_current_time(void);
static int utimer_is_timeout(struct utimer *ut);
static int utimer_clear_timeout(struct utimer *ut);
static uint32_t utimer_get_delay(struct utimer *ut);
static int utimer_set_delay(struct utimer *ut, uint32_t delay_ms);
static int utimer_update(struct utimer *ut);
static int utimer_update_with_curtime(struct utimer *ut,
				      struct utimeval *cur);
static uint32_t utimer_sleep_time_with_curtime(struct utimer *ut,
					       struct utimeval *cur);
static int utimer_init(struct utimer *ut, uint32_t delay_ms);
static int utimer_clear(struct utimer *ut);
static void utimer_lib_init(void (*func) (int *sec, int *msec));

# define YAS529_CDRV_CENTER_X  512
# define YAS529_CDRV_CENTER_Y1 512
# define YAS529_CDRV_CENTER_Y2 512
# define YAS529_CDRV_CENTER_T  256
# define YAS529_CDRV_CENTER_I1 512
# define YAS529_CDRV_CENTER_I2 512
# define YAS529_CDRV_CENTER_I3 512

#define YAS529_CDRV_ROUGHOFFSET_MEASURE_OF_VALUE 33
#define YAS529_CDRV_ROUGHOFFSET_MEASURE_UF_VALUE  0
#define YAS529_CDRV_NORMAL_MEASURE_OF_VALUE 1024
#define YAS529_CDRV_NORMAL_MEASURE_UF_VALUE    1

#define MS3CDRV_CMD_MEASURE_ROUGHOFFSET 0x1
#define MS3CDRV_CMD_MEASURE_XY1Y2T      0x2

#define MS3CDRV_RDSEL_MEASURE     0xc0
#define MS3CDRV_RDSEL_CALREGISTER 0xc8

#define MS3CDRV_WAIT_MEASURE_ROUGHOFFSET  2	/*  1.5[ms] */
#define MS3CDRV_WAIT_MEASURE_XY1Y2T      13	/* 12.3[ms] */

#define MS3CDRV_I2C_SLAVE_ADDRESS 0x2e
#define MS3CDRV_GSENSOR_INITIALIZED     (0x01)
#define MS3CDRV_MSENSOR_INITIALIZED     (0x02)

#define YAS529_CDRV_NO_ERROR 0
#define YAS529_CDRV_ERR_ARG (-1)
#define YAS529_CDRV_ERR_NOT_INITIALIZED (-3)
#define YAS529_CDRV_ERR_BUSY (-4)
#define YAS529_CDRV_ERR_I2CCTRL (-5)
#define YAS529_CDRV_ERR_ROUGHOFFSET_NOT_WRITTEN (-126)
#define YAS529_CDRV_ERROR (-127)

#define YAS529_CDRV_MEASURE_X_OFUF  0x1
#define YAS529_CDRV_MEASURE_Y1_OFUF 0x2
#define YAS529_CDRV_MEASURE_Y2_OFUF 0x4

struct yas529_machdep_func {
	int (*i2c_open) (void);
	int (*i2c_close) (void);
	void (*msleep) (int ms);
};

static int yas529_cdrv_actuate_initcoil(void);
static int yas529_cdrv_set_rough_offset(const uint8_t *rough_offset);
static int yas529_cdrv_recalc_fine_offset(int32_t *prev_fine_offset,
					  int32_t *new_fine_offset,
					  uint8_t *prev_rough_offset,
					  uint8_t *new_rough_offset);
static int yas529_cdrv_set_transformatiom_matrix(const int8_t *transform);
static int yas529_cdrv_measure_rough_offset(uint8_t *rough_offset);
static int yas529_cdrv_measure(int32_t *msens, int32_t *raw,
			       int16_t *t);
static int yas529_cdrv_init(const int8_t *transform,
			    struct yas529_machdep_func *func);
static int yas529_cdrv_term(void);

#define YAS529_NO_ERROR                 (0)
#define YAS529_ERROR_ARG                (YAS529_CDRV_ERR_ARG)
#define YAS529_ERROR_NOT_INITIALIZED    (YAS529_CDRV_ERR_NOT_INITIALIZED)
#define YAS529_ERROR_BUSY               (YAS529_CDRV_ERR_BUSY)
#define YAS529_ERROR_I2C                (YAS529_CDRV_ERR_I2CCTRL)
#define YAS529_ERROR_NOT_ACTIVE         (-124)
#define YAS529_ERROR_ROUGHOFFSET_NOT_WRITTEN (YAS529_CDRV_ERR_ROUGHOFFSET_NOT_WRITTEN)
#define YAS529_ERROR_ERROR              (YAS529_CDRV_ERROR)
#define YAS529_ERROR_RESTARTSYS         (-512)

#define YAS529_IOC_GET_DRIVER_STATE     (1)
#define YAS529_IOC_SET_DRIVER_STATE     (2)

#define YAS529_REPORT_DATA                  (0x01)
#define YAS529_REPORT_CALIB                 (0x02)
#define YAS529_REPORT_OVERFLOW_OCCURED      (0x04)
#define YAS529_REPORT_ROUGH_OFFSET_CHANGED  (0x08)
#define YAS529_REPORT_FINE_OFFSET_CHANGED   (0x10)

struct yas529_driver_state {
	int32_t fine_offset[3];
	uint8_t rough_offset[3];
	int accuracy;
};

struct geomagnetic_hwdep_driver {
	int (*init) (void);
	int (*term) (void);
	int (*get_enable) (void);
	int (*set_enable) (int enable);
	int (*get_filter_enable) (void);
	int (*set_filter_enable) (int enable);
	int (*get_filter_len) (void);
	int (*set_filter_len) (int len);
	int (*get_delay) (void);
	int (*set_delay) (int delay);
	int (*get_position) (void);
	int (*set_position) (int accuracy);
	int (*measure) (int32_t *magnetic, int32_t *raw,
			int32_t *accuracy, uint32_t *time_delay_ms);
	int (*ioctl) (unsigned int cmd, unsigned long args);

	struct geomagnetic_hwdep_callback {
		int (*lock) (void);
		int (*unlock) (void);
		int (*i2c_open) (void);
		int (*i2c_close) (void);
		void (*msleep) (int ms);
		void (*current_time) (int32_t *sec, int32_t *msec);
	} callback;
};

static int geomagnetic_driver_init(struct geomagnetic_hwdep_driver
				   *hwdep_driver);

#define GEOMAGNETIC_I2C_DEVICE_NAME     "yas529"
#define GEOMAGNETIC_DEVICE_NAME         "yas529"
#define GEOMAGNETIC_INPUT_NAME          "geomagnetic"
#define GEOMAGNETIC_INPUT_RAW_NAME      "geomagnetic_raw"

#endif				/* _YAS529_H_ */


