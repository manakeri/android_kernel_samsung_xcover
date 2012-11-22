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

#include <linux/version.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/vmalloc.h>
#include <linux/mman.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/device.h>
#include <linux/moduleparam.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <asm/pgtable.h>
#include <mach/pmu.h>
#include <linux/semaphore.h>
#include <mach/pxa9xx_pm_logger.h>

#define DRIVER_VERSION "v1.0"
#define DRIVER_AUTHOR "MARVELL"
#define DRIVER_DESC "Power management event Logger"

/* module information */
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");

/* global logger descriptor */
struct pm_logger_descriptor pm_logger_app;

/* database for parser print */
/* pay attention: when adding new string to the database, make sure to
 *  update MAX_DATA_NUM so we'll not exceed it */
u8 *pm_logger_app_db[][MAX_DATA_NUM] = {
	{"D1 ENTRY", "PWRMODE", "CKENA", "CKENB", "CKENC",
		"MIPI", "CPUPWR"},
	{"D2 ENTRY", "PWRMODE", "CKENA", "CKENB", "CKENC",
		"OSCC", "MIPI", "CPUPWR"},
	{"CGM ENTRY", "PWRMODE", "CKENA", "CKENB", "CKENC",
		"MIPI", "CPUPWR"},
	{"C1 ENTRY", "CPUPWR"},
	{"D1 EXIT", "AD1D0SR"},
	{"D2 EXIT", "AD2D0SR"},
	{"CGM EXIT", "ACGD0SR"},
	{"C1 EXIT"},
	{"OP REQ", "OP"},
	{"OP EN", "OP", "DEV"},
	{"OP DIS", "OP", "DEV"},
	{"OP EN NAME", "OP", "DEV"},
	{"OP DIS NAME", "OP", "DEV"},
	{"OP EN NO CHANGE", "OP", "DEV"},
	{"OP DIS NO CHANGE", "OP", "DEV"},
	{"OP SET", "FREQ", "ACCR", "ACSR"},
	{"C2 ENTRY"},
	{"C2 EXIT", "ICHP"}
};

/* var to avoid a print because of previous print */
static unsigned int pm_logger_print = 1;

static void pm_logger_wq_fnc(struct work_struct *work);

static DECLARE_DELAYED_WORK(pm_logger_wq, pm_logger_wq_fnc);

static void pm_logger_wq_fnc(struct work_struct *work)
{
	printk(KERN_INFO "Long active time, printing pm logger\n");
	pm_parser_display_log(1);
	pm_logger_print = 0;
}


/******************************************************************************
* Function: pm_logger_app_add_trace_short
*******************************************************************************
* Description: add logger trace with 2 parameters
*
* Parameters: event number, time stamp and 2 arguments
*
* Return value: none
*
* Notes:
******************************************************************************/
void pm_logger_app_add_trace_short(int event,
			unsigned int timeStamp,
			unsigned int arg1,
			unsigned int arg2)
{
	register unsigned int entry = pm_logger_app.current_entry;

	if (pm_logger_app.enabled == PM_LOGGER_STOP) /* do noting */
		return;

	/* if end of buffer and ONESHOT mode, stop */
	if ((pm_logger_app.mode == PM_LOGGER_ONESHOT_MODE) &&
		(entry+4 > pm_logger_app.buffSize-1)) {
		printk(KERN_INFO "PM LOGGER APP: buffer is full\n");
		pm_logger_app_stop();
		return;
	}

	/* add 24 bits of sync pattern and 8 bit of event num */
	pm_logger_app.buffer[entry++] =
	(event & PM_LOG_EVENT_MASK) | PM_LOG_SYNC_PATTERN;
	entry = entry & (pm_logger_app.buffSize - 1); /* like modulo */

	/* add time stamp */
	pm_logger_app.buffer[entry++] = timeStamp;
	entry = entry & (pm_logger_app.buffSize - 1);

	/* add arguements */
	pm_logger_app.buffer[entry++] = arg1;
	entry = entry & (pm_logger_app.buffSize - 1);

	pm_logger_app.buffer[entry++] = arg2;
	entry = entry & (pm_logger_app.buffSize - 1);

	/* update current entry */
	pm_logger_app.current_entry = entry;
}
EXPORT_SYMBOL(pm_logger_app_add_trace_short);

/******************************************************************************
* Function: pm_logger_app_add_trace
*******************************************************************************
* Description: add logger trace with unknown number of parameters
*
* Parameters: event number, time stamp and the arguments
*
* Return value: none
*
* Notes:
******************************************************************************/
void pm_logger_app_add_trace(unsigned int num_args,
				int event,
				unsigned int timeStamp,
				...)
{
	va_list ap;
	int i = 0;
	unsigned int entry = pm_logger_app.current_entry;

	if (pm_logger_app.enabled == PM_LOGGER_STOP) /* do noting */
		return;

	/* if end of buffer and ONESHOT mode, stop */
	if ((pm_logger_app.mode == PM_LOGGER_ONESHOT_MODE) &&
		(entry+2+num_args > pm_logger_app.buffSize-1)) {
		printk(KERN_INFO "PM LOGGER APP: buffer is full\n");
		pm_logger_app_stop();
		return;
	}

	/* add 24 bits of sync pattern and 8 bit of event num */
	pm_logger_app.buffer[entry++] =
	(event & PM_LOG_EVENT_MASK) | PM_LOG_SYNC_PATTERN;
	entry = entry & (pm_logger_app.buffSize - 1);

	/* add time stamp */
	pm_logger_app.buffer[entry++] = timeStamp;
	entry = entry & (pm_logger_app.buffSize - 1);

	/* add arguements */
	va_start(ap, timeStamp);
	while (i < num_args) {
		pm_logger_app.buffer[entry++] = va_arg(ap, unsigned int);
		entry = entry & (pm_logger_app.buffSize - 1);
		i++;
	}
	va_end(ap);

	/* update current entry */
	pm_logger_app.current_entry = entry;
}
EXPORT_SYMBOL(pm_logger_app_add_trace);

/******************************************************************************
* Function: pm_logger_app_clear
*******************************************************************************
* Description: clear buffer
*
* Parameters: none
*
* Return value: none
*
* Notes:
******************************************************************************/
void pm_logger_app_clear(void)
{
	/* clear buffer */
	if (pm_logger_app.buffer)
		memset(pm_logger_app.buffer, 0x00,
			pm_logger_app.buffSize * sizeof(unsigned int));

	/* update current entry */
	pm_logger_app.current_entry = 0;
}

/******************************************************************************
* Function: pm_logger_app_alloc_buffer
*******************************************************************************
* Description: allocate buffer
*
* Parameters: none
*
* Return value: 0 on success, -1 otherwise
*
* Notes:
******************************************************************************/
int pm_logger_app_alloc_buffer(void)
{
	pm_logger_app.buffer =
		kzalloc(pm_logger_app.buffSize * sizeof(unsigned int),
		GFP_KERNEL);
	if (pm_logger_app.buffer == NULL) {
		printk(KERN_ERR "PM Logger: failed to allocate buffer\n");
		return -1;
	}

	return 0;
}

/******************************************************************************
* Function: pm_logger_app_change_buffSize
*******************************************************************************
* Description: change buffer size
*
* Parameters: buffer size (in cells)
*
* Return value: 0 on success, -1 if buffer size exceeded it's limits
*
* Notes:
******************************************************************************/
int pm_logger_app_change_buffSize(unsigned int new_buffSize)
{
	int ret;
	unsigned long flags;

	if (new_buffSize == pm_logger_app.buffSize)
		return 0;

	/* verify that buffer size is legal */
	if ((new_buffSize < PM_LOGGER_BUFFER_SZ_MIN) ||
		(new_buffSize > PM_LOGGER_BUFFER_SZ_MAX)) {
		printk(KERN_ERR "PM Logger: invalid buffer size\n");
		return -1;
	}

	/* verify the buffer size is a power of 2 */
	if (is_valid_size(new_buffSize) == false) {
		printk(KERN_ERR "PM Logger: buffer size must be power of 2\n");
		return -1;
	}

    /* disable interrupts, so no write to buffer will occure */
	local_fiq_disable();
	local_irq_save(flags);
	/* free old buffer */
	kfree(pm_logger_app.buffer);

	/* update buffer size var */
	pm_logger_app.buffSize = new_buffSize;

	/* allocate new buffer with new size and update current entry 0 */
	ret = pm_logger_app_alloc_buffer();
	if (ret == -1)
		return -1;
	pm_logger_app.current_entry = 0;

	/* enable interrupts */
	local_irq_restore(flags);
	local_fiq_enable();

	return 0;
}

/******************************************************************************
* Function: pm_logger_app_start
*******************************************************************************
* Description: Enable logger tracing
*
* Parameters: none
*
* Return value: none
*
* Notes:
******************************************************************************/
void pm_logger_app_start(void)
{
	pm_logger_app.enabled = PM_LOGGER_START;
}

/******************************************************************************
* Function: pm_logger_app_stop
*******************************************************************************
* Description: Disable logger tracing
*
* Parameters: none
*
* Return value: none
*
* Notes:
******************************************************************************/
void pm_logger_app_stop(void)
{
	pm_logger_app.enabled = PM_LOGGER_STOP;
}

/******************************************************************************
* Function: set_pm_logger_app_mode
*******************************************************************************
* Description: set logger mode
*
* Parameters:	PM_LOGGER_REG_MODE		- regular mode,
*				PM_LOGGER_ONESHOT_MODE	- one shot mode
*
* Return value: none
*
* Notes:
******************************************************************************/
void set_pm_logger_app_mode(int mode)
{
	pm_logger_app_stop();
	pm_logger_app_clear();
	pm_logger_app.mode = mode;
	pm_logger_app_start();
}

/******************************************************************************
* Function: get_pm_logger_app_mode
*******************************************************************************
* Description: get logger mode
*
* Parameters: none
*
* Return value:	PM_LOGGER_REG_MODE	- regular mode,
*				PM_LOGGER_ONESHOT_MODE	- one shot mode
*
* Notes:
******************************************************************************/
int get_pm_logger_app_mode(void)
{
	return pm_logger_app.mode;
}

/******************************************************************************
* Function: get_pm_logger_app_buffer
*******************************************************************************
* Description: get pm logger buffer
*
* Parameters: none
*
* Return value: pm logger buffer
*
* Notes:
******************************************************************************/
u32 *get_pm_logger_app_buffer(void)
{
	return pm_logger_app.buffer;
}

/******************************************************************************
* Function: get_pm_logger_app_buffSize
*******************************************************************************
* Description: get pm logger buffer size
*
* Parameters: none
*
* Return value: pm logger buffer size
*
* Notes:
******************************************************************************/
int get_pm_logger_app_buffSize(void)
{
	return pm_logger_app.buffSize;
}

/******************************************************************************
* Function: get_pm_logger_app_current_entry
*******************************************************************************
* Description: get pm logger current entry
*
* Parameters: none
*
* Return value: pm logger current enrty
*
* Notes:
******************************************************************************/
int get_pm_logger_app_current_entry(void)
{
	return pm_logger_app.current_entry;
}

/******************************************************************************
* Function: get_pm_logger_app_db
*******************************************************************************
* Description: get pm logger database
*
* Parameters: none
*
* Return value: pm logger database
*
* Notes:
******************************************************************************/
u8 ***get_pm_logger_app_db(void)
{
	return (u8 ***)pm_logger_app_db;
}

/******************************************************************************
* Function: get_pm_logger_app_status
*******************************************************************************
* Description: get pm logger status
*
* Parameters: none
*
* Return value: PM_LOGGER_START if enabled, PM_LOGGER_STOP otherwise
*
* Notes:
******************************************************************************/
int get_pm_logger_app_status(void)
{
	return pm_logger_app.enabled;
}

/******************************************************************************
* Function: is_valid_size
*******************************************************************************
* Description:	check if size is a power of 2
*
* Parameters:	buffer size
*
* Return value:	true if valid, false - otherwise
*
******************************************************************************/
int is_valid_size(unsigned int size)
{
	return ((size != 0) && !(size & (size - 1)));
}

/******************************************************************************
* Function: pm_logger_init
*******************************************************************************
* Description: logger init
*
* Parameters: none
*
* Return value: 0 on success, -1 otherwise
*
* Notes:
******************************************************************************/
static int __init pm_logger_app_init(void)
{
	int ret;

	/* set buffer size to default */
	pm_logger_app.buffSize = PM_LOGGER_BUFFER_SZ_DEFAULT;

	/* allocate buffer */
	ret = pm_logger_app_alloc_buffer();
	if (ret == -1)
		return -1;

	/* pm logger diabled by default */
	pm_logger_app_stop();

	/* Set debug feature for D2 active time to 0 (OFF) */
	pm_logger_app.debug_length_in_msec = 0;

	/* regular mode by default */
	 set_pm_logger_app_mode(PM_LOGGER_REG_MODE);

    return 0;
}

/******************************************************************************
* Function: pm_logger_exit
*******************************************************************************
* Description: logger exit
*
* Parameters: none
*
* Return value: none
*
* Notes:
******************************************************************************/
static void __exit pm_logger_app_exit(void)
{
	kfree(pm_logger_app.buffer);
	pm_logger_app.buffer = NULL;
}

/******************************************************************************
* Function: pm_logger_app_set_debug_length_in_msec
*******************************************************************************
* Description: set debug_length_in_msec value
*
* Parameters: time in msec
*
* Return value: none
*
* Notes:
******************************************************************************/
void pm_logger_app_set_debug_length_in_msec(unsigned int msec)
{
	pm_logger_app.debug_length_in_msec = msec;
}

/******************************************************************************
* Function: debug_check_active_time
*******************************************************************************
* Description: check if active time length was valid
*
* Parameters: stop time, start time and print (to avoid a print after a
* previous one)
*
* Return value: none
*
* Notes:
******************************************************************************/
void debug_check_active_time(unsigned int active_time_stop,
	unsigned int active_time_start)
{
	if (pm_logger_app.debug_length_in_msec &&
		((active_time_stop - active_time_start) * 1000 / 32768) >
			pm_logger_app.debug_length_in_msec &&
			pm_logger_print)
		schedule_delayed_work(&pm_logger_wq, msecs_to_jiffies(1000));
}

/******************************************************************************
* Function: turn_on_pm_logger_print
*******************************************************************************
* Description: set pm_logger_print var to be 1
*
* Parameters: none
*
* Return value: none
*
* Notes:
******************************************************************************/
void turn_on_pm_logger_print(void)
{
	pm_logger_print = 1;
}

subsys_initcall(pm_logger_app_init);
