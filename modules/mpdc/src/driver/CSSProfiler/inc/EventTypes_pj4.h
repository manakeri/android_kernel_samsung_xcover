/*
** (C) Copyright 2009 Marvell International Ltd.
**  		All Rights Reserved

** This software file (the "File") is distributed by Marvell International Ltd. 
** under the terms of the GNU General Public License Version 2, June 1991 (the "License"). 
** You may use, redistribute and/or modify this File in accordance with the terms and 
** conditions of the License, a copy of which is available along with the File in the 
** license.txt file or by writing to the Free Software Foundation, Inc., 59 Temple Place, 
** Suite 330, Boston, MA 02111-1307 or on the worldwide web at http://www.gnu.org/licenses/gpl.txt.
** THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE IMPLIED WARRANTIES 
** OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY DISCLAIMED.  
** The License provides additional details about this warranty disclaimer.
*/

/* (C) Copyright 2009 Marvell International Ltd. All Rights Reserved */

#ifndef __PX_EVENT_TYPES_PJ4_H__
#define __PX_EVENT_TYPES_PJ4_H__


/* OS Timer event */
#define PJ4_OS_TIMER_DEFAULT_EVENT_TYPE 0xFFFF

/* CCNT event */
#define PJ4_PMU_CCNT_CORE_CLOCK_TICK 0xFF
    
/* CCNT 64 cycle event */
#define PJ4_PMU_CCNT_CORE_CLOCK_TICK_64 0xFE
    
/* Software increment */
#define PJ4_PMU_SOFTWARE_INC  0x0

/* Instruction fetch that causes a refill at the lowest level of instruction or unified cache */
#define PJ4_PMU_INST_FETECH_CAUSE_CACHE_REFILL  0x1

/* Instruction fetch that causes a TLB refill at the lowest level of TLB */
#define PJ4_PMU_INST_FETECH_CAUSE_TLB_REFILL    0x2

/* Data read or write operation that causes a refill at the lowest level of data or unified cache */
#define PJ4_PMU_DATA_OPERATION_CAUSE_CACHE_REFILL 0x3

/* Data read or write operation that causes a cache access at the lowest level of data or unified cache */
#define PJ4_PMU_DATA_OPERATION_CAUSE_CACHE_ACCESS 0x4

/* Data read or write operation that causes a TLB refill at the lowest level of TLB */
#define PJ4_PMU_DATA_OPERATION_CAUSE_TLB_REFILL 0x5

/* Data read architecturally executed */
#define PJ4_PMU_DATA_READ 0x6

/* Data write architecturally executed */
#define PJ4_PMU_DATA_WRITE 0x7

/* Instruction architecturally executed */
#define PJ4_PMU_INST_EXECUTED   0x8

/* Exception taken */
#define PJ4_PMU_EXCEPTION_TAKEN 0x9

/* Exception return architecturally executed */
#define PJ4_PMU_EXCEPTION_RETURN_EXECUTED 0xa

/* Instruction that writes to the Context ID Register architecturally executed */
#define PJ4_PMU_INST_WRITE_CONTEXT_ID_REG_EXECUTED 0xb

/* Software change of PC, except by an exception, architecturally executed */
#define PJ4_PMU_SOFTWARE_CHANGE_OF_PC_EXECUTED 0xc

/* Immediate branch architecturally executed, taken or not taken */
#define PJ4_PMU_IMMEDIATE_BRANCH_EXECUTED 0xd

/* Procedure return, other than exception returns, architecturally executed */
#define PJ4_PMU_PROCEDURE_RETURN_EXECUTED 0xe

/* Unaligned access architecturally executed */
#define PJ4_PMU_UNALIGNED_ACCESS_EXECUTED 0xf


/* Branch mispredicted or not predicted */
#define PJ4_PMU_BRANCH_MISPREDICTED_OR_NOT_PREDICTED 0x10

/* Cycle Count */
#define PJ4_PMU_CYCLE_COUNT 0x11

/* Branches or other change in the program flow that could have been predicted by the branch prediction resources of the processor */
#define PJ4_PMU_BRANCH_COULD_HAVE_BEEN_PREDICTED 0x12

/* D-Cache Read Hit */
#define PJ4_PMU_DCACHE_READ_HIT 0x40

/* D-Cache Read Miss */
#define PJ4_PMU_DCACHE_READ_MISS 0x41

/* D-Cache Write Hit */
#define PJ4_PMU_DCACHE_WRITE_HIT 0x42

/* D-Cache Write Miss */
#define PJ4_PMU_DCACHE_WRITE_MISS 0x43

/* MMU Bus Request */
#define PJ4_PMU_MMU_BUS_REQUEST 0x44

/* I-Cache Bus Request */
#define PJ4_PMU_ICACHE_BUS_REQUEST 0x45

/* WB write latency */
#define PJ4_PMU_WB_WRITE_LATENCY 0x46

/* Hold LDM/STM */
#define PJ4_PMU_HOLD_LDM_STM 0x47

/* No Dual cflag */
#define PJ4_PMU_NO_DUAL_CFLAG 0x48

/* No Dual Register Plus */
#define PJ4_PMU_NO_DUAL_REGISTER_PLUS 0x49

/* LDST ROB0 on Hold */
#define PJ4_PMU_LDST_ROB0_ON_HOLD 0x4a

/* LDST ROB1 on Hold */
#define PJ4_PMU_LDST_ROB1_ON_HOLD 0x4b

/* Data Write Access Count */
#define PJ4_PMU_DATA_WRITE_ACCESS_COUNT 0x4c

/* Data Read Access Count */
#define PJ4_PMU_DATA_READ_ACCESS_COUNT 0x4d

/* A2 Stall */
#define PJ4_PMU_A2_STALL 0x4e

/* L2 Cache Write Hit */
#define PJ4_PMU_L2_CACHE_WRITE_HIT 0x4f

/* L2 Cache Write Miss */
#define PJ4_PMU_L2_CACHE_WRITE_MISS 0x50

/* L2 Cache Read Count */
#define PJ4_PMU_L2_CACHE_READ_COUNT 0x51

/* I-Cache Read Miss */
#define PJ4_PMU_ICACHE_READ_MISS 0x60

/* ITLB Miss */
#define PJ4_PMU_ITLB_MISS 0x61

/* Single Issue */
#define PJ4_PMU_SINGLE_ISSUE 0x62

/* Branch Retired */
#define PJ4_PMU_BRANCH_RETIRED 0x63

/* ROB Full */
#define PJ4_PMU_ROB_FULL 0x64

/* MMU Read Beat */
#define PJ4_MMU_READ_BEAT 0x65

/* WB Write Beat */
#define PJ4_PMU_WB_WRITE_BEAT 0x66

/* Dual Issue */
#define PJ4_PMU_DUAL_ISSUE 0x67

/* No Dual raw */
#define PJ4_PMU_NO_DUAL_RAW 0x68

/* Hold IS */
#define PJ4_PMU_HOLD_IS 0x69

/* L2 Cache Latency */
#define PJ4_PMU_L2_CACHE_LATENCY 0x6a

/* D-Cache Access */
#define PJ4_PMU_DCACHE_ACCESS 0x70

/* DTLB Miss */
#define PJ4_PMU_DTLB_MISS 0x71

/* Branch Prediction Miss */
#define PJ4_PMU_BRANCH_PREDICTION_MISS 0x72

/* A1 Stall */
#define PJ4_PMU_A1_STALL 0x74

/* D-Cache Read Latency */
#define PJ4_PMU_DCACHE_READ_LATENCY 0x75

/* D-Cache Write Latency */
#define PJ4_PMU_DCACHE_WRITE_LATENCY 0x76

/* No Dual Register File */
#define PJ4_PMU_NO_DUAL_REGISTER_FILE 0x77

/* BIU Simultaneous Access */
#define PJ4_PMU_BIU_SIMULTANEOUS_ACCESS 0x78

/* L2 Cache Read Hit */
#define PJ4_PMU_L2_CACHE_READ_HIT 0x79

/* L2 Cache Read Miss */
#define PJ4_PMU_L2_CACHE_READ_MISS 0x7a


/* L2 Cache Eviction */
#define PJ4_PMU_L2_CACHE_EVICTION 0x7b

/* TLB Miss */
#define PJ4_PMU_TLB_MISS 0x80

/* Branches Taken */
#define PJ4_PMU_BRANCH_TAKEN 0x81

/* WB Full */
#define PJ4_PMU_WB_FULL 0x82

/* D-Cache Read Beat */
#define PJ4_PMU_DCACHE_READ_BEAT 0x83

/* D-Cache Write Beat */
#define PJ4_PMU_DCACHE_WRITE_BEAT 0x84

/* No Dual HW */
#define PJ4_PMU_NO_DUAL_HW 0x85

/* No Dual Multiple */
#define PJ4_PMU_NO_DUAL_MULTIPLE 0x86

/* BIU Any Access */
#define PJ4_PMU_BIU_ANY_ACCESS 0x87

/* Main TLB refill caused by I-Cache */
#define PJ4_PMU_MAIN_TLB_REFILL_BY_ICACHE 0x88

/* Main TLB refill caused by D-Cache */
#define PJ4_PMU_MAIN_TLB_REFILL_BY_DCACHE 0x89

/* I-Cache read beat */
#define PJ4_PMU_ICACHE_READ_BEAT 0x8a

/* Counts any event from external input source PMUEXTIN[0] */
#define PJ4_PMU_COUNT_ANY_EVENT_FROM_EXTERNAL_INPUT_SOURCE_PMUEXTIN0 0x90

/* Counts any event from external input source PMUEXTIN[1] */
#define PJ4_PMU_COUNT_ANY_EVENT_FROM_EXTERNAL_INPUT_SOURCE_PMUEXTIN1 0x91


/* Counts any event from both external input source PMUEXTIN[0] and PMUEXTIN[1]*/
#define PJ4_PMU_COUNT_ANY_EVENT_FROM_EXTERNAL_INPUT_SOURCE_PMUEXTIN0_PMUEXTIN1 0x92


/* WMMX2 store FIFO full */
#define PJ4_PMU_WMMX2_STORE_FIFO_FULL 0xc0

/* WMMX2 finish FIFO full */
#define PJ4_PMU_WMMX2_FINISH_FIFO_FULL 0xc1

/* WMMX2 instruction FIFO full */
#define PJ4_PMU_WMMX2_INST_FIFO_FULL 0xc2

/* WMMX2 instruction retired */
#define PJ4_PMU_WMMX2_INST_RETIRED 0xc3

/* WMMX2 Busy */
#define PJ4_PMU_WMMX2_BUSY 0xc4

/* WMMX2 Hold MI */
#define PJ4_PMU_WMMX2_HOLD_MI 0xc5

/* WMMX2 Hold MW */
#define PJ4_PMU_WMMX2_HOLD_MW 0xc6

/* L0IC line fill */
#define PJ4_PMU_L0IC_LINE_FILL 0xf0

/* L0IC hit prefetch buffer */
#define PJ4_PMU_L0IC_HIT_PREFETECH_BUFFER 0xf1

#endif /* __PX_EVENT_TYPES_PJ4_H__ */

