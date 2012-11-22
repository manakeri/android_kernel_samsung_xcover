/* pxa95x_freeze.h
 *
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html

 * (C) Copyright 2007 Marvell International Ltd.
 * All Rights Reserved
 */

#ifndef PXA95X_FREEZE_H
#define PXA95X_FREEZE_H

#ifdef CONFIG_PXA95x_SUSPEND
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/sysdev.h>
#include <linux/spinlock.h>
#include <linux/notifier.h>
#include <linux/string.h>
#include <linux/kobject.h>
#include <linux/list.h>
#include <linux/sched.h>
#include <linux/ioctl.h>
#include <linux/uaccess.h>
#include <linux/slab.h>


/* this is the entry to list of unfreezable tasks. It containts the task name and
link to the next entry */
struct unfreezable_task_entry {
struct list_head	link;
char		name[TASK_COMM_LEN];
};

extern int android_freezer_debug;
extern void FRZ_process_show(int freeze_only);
extern int FRZ_set_nonfreezeable_procees(char *task_name);
extern int FRZ_process_unfreezable_list_check(void);
#endif

#define PROCESS_NON_FREEZE 1
#define MAX_LINE_LENGTH 256
#define FRZ_DEBUG_DISPLAY_TASK 1
#define FRZ_DEBUG_CHECK_FREEZE_TASKS (1 << 1)

typedef struct {
    unsigned long data_arg1;
    unsigned long data_arg2;
} data_struct;

#endif

