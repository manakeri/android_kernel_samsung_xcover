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

#ifdef __KERNEL__

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
#endif

#endif
