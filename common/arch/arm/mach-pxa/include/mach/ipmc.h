/*
 * Copyright (C) 2003-2004 Intel Corporation.
 *
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html
 *
 *
 * (C) Copyright 2006 Marvell International Ltd.
 * All Rights Reserved
 */

#ifndef __IPMC_H
#define __IPMC_H

#ifdef __KERNEL__
#include <linux/ioctl.h>
#include <linux/miscdevice.h>
#endif

#include <mach/pxa95x_dvfm.h>
#include <mach/pxa95x_pm.h>

struct ipmc_ioctl_setfvop {
	unsigned int op;
	unsigned int mode;
};

struct ipmc_ioctl_getfvopinfo {
	unsigned int op;
	struct pxa95x_fv_info info;
};

/* Use 'K' as magic number */
#define IPMC_IOC_MAGIC  'K'
#define IPMC_IOCTL_SET_DEEPIDLE		_IOW(IPMC_IOC_MAGIC, 2, unsigned int)
#define IPMC_IOCTL_GET_FV_OP		_IOR(IPMC_IOC_MAGIC, 3, unsigned int)
#define IPMC_IOCTL_SET_FV_OP		_IOW(IPMC_IOC_MAGIC, 4, struct ipmc_ioctl_setfvop)
#define IPMC_IOCTL_GET_FV_OP_COUNT	_IOR(IPMC_IOC_MAGIC, 5, unsigned int)
#define IPMC_IOCTL_GET_FV_OP_INFO	_IOR(IPMC_IOC_MAGIC, 6, struct ipmc_ioctl_getfvopinfo)
#define IPMC_IOCTL_SET_CPU_MODE		_IOW(IPMC_IOC_MAGIC, 7, unsigned int)
#define IPMC_IOCTL_GET_SLEEPTIME	_IOR(IPMC_IOC_MAGIC, 8, unsigned int)
#define IPMC_IOCTL_SET_SLEEPTIME	_IOW(IPMC_IOC_MAGIC, 9, unsigned int)
#define IPMC_IOCTL_RESET_UI_TIMER	_IOW(IPMC_IOC_MAGIC, 15, unsigned int)
#define IPMC_IOCTL_GET_WAKETIME		_IOR(IPMC_IOC_MAGIC, 16, unsigned int)
#define IPMC_IOCTL_SET_WAKETIME		_IOW(IPMC_IOC_MAGIC, 17, unsigned int)
#define IPMC_IOCTL_GET_UITIME		_IOR(IPMC_IOC_MAGIC, 18, unsigned int)
#define IPMC_IOCTL_SET_UITIME		_IOW(IPMC_IOC_MAGIC, 19, unsigned int)
#define IPMC_IOCTL_STARTPMU		_IOW(IPMC_IOC_MAGIC, 20, int)
#define IPMC_IOCTL_GET_WAKEUPSRC	_IOR(IPMC_IOC_MAGIC, 21, pm_wakeup_src_t)
#define IPMC_IOCTL_SET_WAKEUPSRC	_IOW(IPMC_IOC_MAGIC, 22, pm_wakeup_src_t)
#define IPMC_IOCTL_STOPPMU		_IOW(IPMC_IOC_MAGIC, 23, int)
#define IPMC_IOCTL_CLEAR_EVENTLIST	_IOW(IPMC_IOC_MAGIC, 24, int)
#define IPMC_IOCTL_GET_BKLIGHT		_IOR(IPMC_IOC_MAGIC, 25, int)
#define IPMC_IOCTL_SET_BKLIGHT		_IOW(IPMC_IOC_MAGIC, 26, int)
#define IPMC_IOCTL_SET_WINDOWSIZE	_IOW(IPMC_IOC_MAGIC, 30, int)

/* 20 IPM event max */
#define MAX_IPME_NUM	20
#define INFO_SIZE	128

#ifdef CONFIG_FB_PXA
extern int pxafb_get_backlight(void);
extern void pxafb_set_backlight(int);
#endif

struct ipm_event {
	int type;		/* What type of IPM events. */
	int kind;		/* What kind, or sub-type of events. */
	unsigned char info[INFO_SIZE];	/* events specific data. */
};

#ifdef __KERNEL__
/* IPM events queue */
struct ipme_queue {
	int head;
	int tail;
	int len;
	struct ipm_event ipmes[MAX_IPME_NUM];
	wait_queue_head_t waitq;
};

extern int enable_deepidle;
#endif

/* IPM event types. */
#define IPM_EVENT_PMU			0x1	/* PMU events, may not need. */
#define IPM_EVENT_UI			0x2	/* Device event. */
#define IPM_EVENT_POWER			0x3	/* Power fail/revocer event. */
#define IPM_EVENT_UITIMER		0x4	/* Device Timer timeout event. */
#define IPM_EVENT_CPU			0x5	/* CPU utilization change event. */
#define IPM_EVENT_DEVICE		0x6
#define IPM_EVENT_PROFILER		0x7	/* Profiler events. */
/* #define IPM_EVENT_SYSTEM_WAKEUP       0x8*/
#define IPM_EVENT_SUSPEND_WAKEUP	0x9
#define IPM_EVENT_STANDBY_WAKEUP	0xa

/* IPM event kinds. */
#define IPM_EVENT_POWER_LOW		0x1
#define IPM_EVENT_POWER_FAULT		0x2
#define IPM_EVENT_POWER_OK		0x3
#define IPM_EVENT_IDLE_PROFILER		0x1
#define IPM_EVENT_PERF_PROFILER		0x2
#define IPM_EVENT_DEVICE_KEYPAD		0x0
#define IPM_EVENT_DEVICE_TSI		0x1
#define IPM_EVENT_DEVICE_TIMEOUT	0x2
#define IPM_EVENT_DEVICE_BUSY		0x3
#define IPM_EVENT_DEVICE_OUTD0CS	0x10
#define IPM_EVENT_DEVICE_OUT104		0x20
#define IPM_EVENT_DEVICE_OUT208		0x30
#define IPM_EVENT_DEVICE_OUT416		0x40
#define IPM_EVENT_WAKEUPTIMEOUT		0x0	/* Wakeup Timeout (invalid) */
#define IPM_EVENT_WAKEUPBYRTC		0x1
#define IPM_EVENT_WAKEUPBYUI		0x2	/* User Interface: Touch Screen, Keypad */

#define IPM_WAKEUPTIMER			0x1
#define IPM_UITIMER			0x2

/* IPM event infos, not defined yet. */
#define IPM_EVENT_NULLINFO		0x0

/* IPM functions */
#ifdef __KERNEL__
extern int ipm_event_notify(int type, int kind, void *info,
			    unsigned int info_len);
#endif

#endif
