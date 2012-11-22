/*
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *
 * (C) Copyright 2006 Marvell International Ltd.
 * All Rights Reserved
 */

#ifndef _XSCALE_PMU_H_
#define _XSCALE_PMU_H_

#include <linux/types.h>

/*
 * Different types of events that can be counted by the XScale PMU
 */
#define EVT_ICACHE_MISS			0x00
#define EVT_ICACHE_NO_DELIVER		0x01
#define EVT_DATA_STALL			0x02
#define EVT_ITLB_MISS			0x03
#define EVT_DTLB_MISS			0x04
#define EVT_BRANCH			0x05
#define EVT_BRANCH_MISS			0x06
#define EVT_INSTRUCTION			0x07
#define EVT_DCACHE_FULL_STALL		0x08
#define EVT_DCACHE_FULL_STALL_CONTIG	0x09
#define EVT_DCACHE_ACCESS		0x0A
#define EVT_DCACHE_MISS			0x0B
#define EVT_DCACE_WRITE_BACK		0x0C
#define EVT_PC_CHANGED			0x0D
#define EVT_BCU_REQUEST			0x10
#define EVT_BCU_FULL			0x11
#define EVT_BCU_DRAIN			0x12
#define EVT_BCU_ECC_NO_ELOG		0x14
#define EVT_BCU_1_BIT_ERR		0x15
#define EVT_RMW				0x16

struct pmu_results {
	u32 ccnt_of;
	u32 ccnt;		/* Clock Counter Register */
	u32 pmn0_of;
	u32 pmn0;		/* Performance Counter Register 0 */
	u32 pmn1_of;
	u32 pmn1;		/* Performance Counter Register 1 */
	u32 pmn2_of;
	u32 pmn2;		/* Performance Counter Register 2 */
	u32 pmn3_of;
	u32 pmn3;		/* Performance Counter Register 3 */
};

#ifdef __KERNEL__

extern struct pmu_results results;

int pmu_claim(void);		/* Claim PMU for usage */
int pmu_start(u32, u32, u32, u32);	/* Start PMU execution */
int pmu_stop(struct pmu_results *);	/* Stop perfmon unit */
int pmu_release(int);		/* Release PMU */
#endif

#endif				/* _XSCALE_PMU_H_ */
