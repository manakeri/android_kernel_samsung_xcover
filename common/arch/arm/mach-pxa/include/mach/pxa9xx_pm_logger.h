/*
 *  PM Logger
 *
 *  Support for Power management related event logger over PXAxxx
 *
 *  Author:	Shay Pathov
 *  Created:	Dec 15, 2010
 *  Copyright:	(C) Copyright 2010 Marvell International Ltd.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  publishhed by the Free Software Foundation.
 */


#ifndef PM_LOGGER_H
#define PM_LOGGER_H

/* total buffer size (in cells) = *MUST* be an exponent of 2 */
#define PM_LOGGER_BUFFER_SZ_DEFAULT ((1024*32) / sizeof(unsigned int)) /*32K*/
#define PM_LOGGER_BUFFER_SZ_MIN ((1024*8) / sizeof(unsigned int)) /*8K*/
#define PM_LOGGER_BUFFER_SZ_MAX ((1024*128) / sizeof(unsigned int)) /*128K*/
#define PM_LOG_SYNC_PATTERN 0x7E7E7E00
#define MAX_DATA_NUM 10
#define PM_LOG_EVENT_MASK 0xFF
#define PM_LOG_SYNC_MASK 0xFFFFFF00

/* pm logger descriptor */
struct pm_logger_descriptor {
	unsigned int *buffer;
	unsigned int current_entry;
	unsigned int buffSize; /* num of cells */
	unsigned int enabled;
	unsigned int mode; /* buffer mode: regular/one-shot */
	unsigned int debug_length_in_msec; /* for long wakeups */
};

/* event */
#define PM_D1_ENTRY 0
#define PM_D2_ENTRY 1
#define PM_CGM_ENTRY 2
#define PM_C1_ENTRY 3
#define PM_D1_EXIT 4
#define PM_D2_EXIT 5
#define PM_CGM_EXIT 6
#define PM_C1_EXIT 7
#define PM_OP_REQ 8
#define PM_OP_EN 9
#define PM_OP_DIS 10
#define PM_OP_EN_NAME 11
#define	PM_OP_DIS_NAME 12
#define	PM_OP_EN_NO_CHANGE 13
#define	PM_OP_DIS_NO_CHANGE 14
#define PM_SET_OP 15

/* start/stop */
#define PM_LOGGER_STOP 0
#define PM_LOGGER_START 1

/* modes */
#define PM_LOGGER_REG_MODE 0
#define PM_LOGGER_ONESHOT_MODE 1






/* functions */
extern void pm_logger_app_add_trace_short(int event,
							unsigned int timeStamp,
							unsigned int arg1,
							unsigned int arg2);
extern void pm_logger_app_add_trace(unsigned int num_args,
					int event,
					unsigned int timeStamp,
					...);
extern void pm_logger_app_clear(void);
extern void pm_parser_display_log(int subsystem);
int pm_logger_app_alloc_buffer(void);
extern int pm_logger_app_change_buffSize(unsigned int new_buffSize);
void pm_logger_app_start(void);
void pm_logger_app_stop(void);
void set_pm_logger_app_mode(int mode);
int get_pm_logger_app_mode(void);
u32 *get_pm_logger_app_buffer(void);
int get_pm_logger_app_buffSize(void);
int get_pm_logger_app_current_entry(void);
u8 ***get_pm_logger_app_db(void);
int get_pm_logger_app_status(void);
int is_valid_size(unsigned int size);
void pm_logger_app_set_debug_length_in_msec(unsigned int msec);
void turn_on_pm_logger_print(void);
void debug_check_active_time(unsigned int active_time_stop,
						unsigned int active_time_start);

#endif

