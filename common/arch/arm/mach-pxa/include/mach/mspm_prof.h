/*
 * PXA Performance profiler and Idle profiler Routines
 *
 * Copyright (c) 2003 Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * (C) Copyright 2006 Marvell International Ltd.
 * All Rights Reserved
 */

#ifndef MSPM_PROF_H
#define MSPM_PROF_H

#include <mach/pmu.h>
#include <mach/xscale-pmu.h>

#define IPM_IDLE_PROFILER	1
#define IPM_PMU_PROFILER	2

struct ipm_profiler_result {
	struct pmu_results pmu;
	unsigned int busy_ratio;	/* CPU busy ratio */
	unsigned int window_size;
};

struct ipm_profiler_arg {
	unsigned int size;	/* size of ipm_profiler_arg */
	unsigned int flags;
	unsigned int window_size;	/* in microseconds */
	unsigned int pmn0;
	unsigned int pmn1;
	unsigned int pmn2;
	unsigned int pmn3;
};

#ifdef __KERNEL__
extern volatile int hlt_counter;

#undef MAX_OP_NUM
#define MAX_OP_NUM		20

/* The minimum sample window is 20ms, the default window is 100ms */
#define MIN_SAMPLE_WINDOW	20
#define DEF_SAMPLE_WINDOW	100

#define DEF_HIGH_THRESHOLD	80
#define DEF_LOW_THRESHOLD	20

extern int mspm_add_event(int op, int cpu_idle);
extern int mspm_prof_init(void);
extern void mspm_prof_exit(void);

extern int ipm_event_notify(int type, int kind, void *info,
			    unsigned int info_len);
extern int (*pipm_start_pmu) (void *);
extern int (*pipm_stop_pmu) (void);

extern void mspm_disable(void);
extern void mspm_enable(void);
extern int mspm_state(void);

#endif

#endif
