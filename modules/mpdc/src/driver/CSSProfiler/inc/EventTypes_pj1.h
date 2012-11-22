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

#ifndef __PX_EVENT_TYPES_PJ1_H__
#define __PX_EVENT_TYPES_PJ1_H__

/* OS Timer event */
#define PJ1_OS_TIMER_DEFAULT_EVENT_TYPE 0xFFFF

/* Counts the number of cycles ALU A1 is stalled */
/* c2 */
#define PJ1_PMU_EVENT_A1_STALL             0x401

/* Counts the number of cycles the BIU is accessed by any unit */
/* c3 */
#define PJ1_PMU_EVENT_BIU_ANY_ACCESS       0x10001

/* Counts the number of cycles the bus is requested by more
 * than one master.*/
/* c2 */
#define PJ1_PMU_EVENT_BIU_SIMULTANEOUS_ACCESS	0x10001

/* Counts the number of times branch prediction causes the
 * wrong branch to be prefetched.*/
/* c2 */
#define PJ1_PMU_EVENT_BRANCH_PREDICT_MISS           0x101

/* Counts the number of times one branch retires */
/* c1 */
#define PJ1_PMU_EVENT_BRANCH_RETIED                 0x101

/* Counts the number of times branch prediction correctly
 * prefetches the required branch */
/* c3 */
#define PJ1_PMU_EVENT_BRANCH_TAKEN                  0x101

/* Counts the number of clock cycles. Every clock cycle
 * increments the counter */
/* c0, c1, c2, c3 */
#define PJ1_PMU_EVENT_CYCLE_COUNT                   0x3

/* Counts all data reads */
/* c1 */
#define PJ1_PMU_EVENT_DATA_READ_ACCESS_COUNT_COUNTER_1        0x10001

/* Counts all data reads */
/* c0 */
#define PJ1_PMU_EVENT_DATA_READ_ACCESS_COUNT_COUNTER_0         0x80001

/* Counts all data writes */
/* c0 */
#define PJ1_PMU_EVENT_DATA_WRITE_ACCESS_COUNT_COUNTER_0        0x40001

/* Counts all data writes */
/* c3 */
#define PJ1_PMU_EVENT_DATA_WRITE_ACCESS_COUNT_COUNTER_3        0x400001

/* Counts the number of data-cache accesses (read hits /
 * misses and write hits / misses) */
/* c2 */
#define PJ1_PMU_EVENT_DCACHE_ACCESS                 0x9

/* Counts the number of times the bus returns data to the data
 * cache during read requests */
/* c3 */
#define PJ1_PMU_EVENT_DCACHE_READ_BEAT              0x801

/* Counts the number of data-cache read hits */
/* c0 */
#define PJ1_PMU_EVENT_DCACHE_READ_HIT               0x5

/* Counts the number of cycles the data cache requests the
 * bus for a read */
/* c2 */
#define PJ1_PMU_EVENT_DCACHE_READ_LATENCY           0x801

/* Counts the number of data-cache read misses (including
 * non-cacheable and non-bufferable cache accesses) */
/* c0 */
#define PJ1_PMU_EVENT_DCACHE_READ_MISS_ON_COUNTER_0   0x9

/* Counts the number of data-cache read misses (including
 * non-cacheable and non-bufferable cache accesses) */
/* c1 */
#define PJ1_PMU_EVENT_DCACHE_READ_MISS_ON_COUNTER_1   0x9

/* Counts the number of data-cache read misses (including
 * non-cacheable and non-bufferable cache accesses) */
/* c3*/
#define PJ1_PMU_EVENT_DCACHE_READ_MISS_ON_COUNTER_3   0x5

/* Counts the number of times the bus returns ready to the
 * data cache during write requests */
/* c3 */
#define PJ1_PMU_EVENT_DCACHE_WRITE_BEAT               0x1001

/* Counts the number of data-cache write hits */
/* c0 */
#define PJ1_PMU_EVENT_DCACHE_WRITE_HIT                0x11

/* Counts the number of cycles the data cache requests the
 * bus for a write */
/* c2 */
#define PJ1_PMU_EVENT_DCACHE_WRITE_LATENCY            0x1001

/* Counts the number of data-cache write misses (including
 * non-cacheable and non-bufferable misses) */
/* c0 */
#define PJ1_PMU_EVENT_DCACHE_WRITE_MISS_ON_COUNTER_0  0x21

/* Counts the number of data-cache write misses (including
 * non-cacheable and non-bufferable misses) */
/* c1 */
#define PJ1_PMU_EVENT_DCACHE_WRITE_MISS_ON_COUNTER_1  0x11

/* Counts the number of data-cache write misses (including
 * non-cacheable and non-bufferable misses) */
/* c3 */
#define PJ1_PMU_EVENT_DCACHE_WRITE_MISS_ON_COUNTER_3  0x9

/* Counts the number of TLB misses for data entries */
/* c2 */
#define PJ1_PMU_EVENT_DTLB_MISS                       0x11

/* Counts the number of cycles the instruction issue is stalled */
/* c1 */
#define PJ1_PMU_EVENT_HOLD_IS                         0x8001

/* Counts the number of cycles the pipeline is blocked
 * because of multiple load/store operations */
/* c0 */
#define PJ1_PMU_EVENT_HOLD_LDM_STM                    0x2001

/* Counts the number of cycles the Instruction cache requests
 * the bus until the data returns */
/* c0 */
#define PJ1_PMU_EVENT_ICACHE_BUS_REQUEST              0x801

/* Counts the number of times the bus returns RDY to the
 * instruction cache, useful to determine the cache¡¯s average
 * read latency (also known as ¡°read miss¡± or ¡°read count¡±) */
/* c1 */
#define PJ1_PMU_EVENT_ICACHE_READ_BEAT                0x801

/* Counts the number of instruction-cache read misses */
/* c1 */
#define PJ1_PMU_EVENT_ICACHE_READ_MISS                0x5

/* Counts the number of Instruction TLB misses */
/* c1 */
#define PJ1_PMU_EVENT_ITLB_MISS                       0x21

/* Counts the number of cycles to complete a request via the
 * MMU bus. This request can derive from multiple masters */
/* c0 */
#define PJ1_PMU_EVENT_MMU_BUS_REQUEST                 0x401

/* Counts the number of times the bus returns RDY to the
 * MMU, useful when determining bus efficiency. A user can
 * use the signal that the MMU is requesting the bus and how
 * long it takes on average for the data to return.
 * (mmu_bus_req / mmu_read_count) */
/* c1 */
#define PJ1_PMU_EVENT_MMU_READ_BEAT                   0x401

/* Counts the number of times a branch is predicted
 * successfully */
/* c0 */
#define PJ1_PMU_EVENT_PREDICTED_BRANCH_COUNT          0x2000001

/* Counts every time an instruction is retired */
/* c0 */
#define PJ1_PMU_EVENT_RETIRED_INSTRUTION              0x41

/* Counts the number of cycles the ROB is full */
/* c1 */
#define PJ1_PMU_EVENT_ROB_FULL                        0x201

/* Counts the number of SIMD cycles */
/* c0 */
#define PJ1_PMU_EVENT_SIMD_CYCLE_COUNT                0x1000001

/* Counts the number of cycles the SIMD coprocessor
 * instruction buffer is full */
/* c2*/
#define PJ1_PMU_EVENT_SIMD_INST_BUFFER_FULL           0x2000001

/* Counts the number of cycles the SIMD coprocessor holds
 * in its Issue (IS) stage */
/* c2 */
#define PJ1_PMU_EVENT_SIMD_HOLD_IS                    0x1000001

/* Counts the number of cycles the SIMD coprocessor holds
 * in its writeback stage */
/* c3 */
#define PJ1_PMU_EVENT_SIMD_HOLD_WRITEBACK_STAGE       0x1000001

/* Counts the number of cycles the SIMD coprocessor retire
 * FIFO is full */
/* c3 */
#define PJ1_PMU_EVENT_SIMD_RETIRE_FIFO_FULL           0x2000001

/* Counts the number of cycles the SIMD coprocessor store
 * FIFO is full */
/* c1 */
#define PJ1_PMU_EVENT_SIMD_STORE_FIFO_FULL            0x2000001

/* Counts the number of retired SIMD instructions */
/* c1 */
#define PJ1_PMU_EVENT_SIMD_RETIED_INST                0x1000001

/* Counts the number of cycles the processor single-issues
 * instructions */
/* c1 */
#define PJ1_PMU_EVENT_SINGLE_ISSUE                    0x41

/* Counts the number of instruction and data TLB misses */
/* c3 */
#define PJ1_PMU_EVENT_TLB_MISS                        0x11

/* Counts the number of cycles the write-back buffer requests
 * the bus until the data is written to the bus */
/* c0 */
#define PJ1_PMU_EVENT_WB_BUS_REQUEST                  0x1001

/* Counts the number of cycles WB is full */
/* c3 */
#define PJ1_PMU_EVENT_WB_FULL                         0x201

/* Counts the number times the bus returns RDY to the write
 * buffer, useful to determine the write buffer¡¯s average write
 * latency
 * (WB Write Latency/ WB Write Beat) */
/* c1 */
#define PJ1_PMU_EVENT_WB_WRITE_BEAT_ON_COUNTER_1      0x1001

/* Counts the number times the bus returns RDY to the write
 * buffer, useful to determine the write buffer¡¯s average write
 * latency
 * (WB Write Latency/ WB Write Beat) */
/* c2 */
#define PJ1_PMU_EVENT_WB_WRITE_BEAT_ON_COUNTER_2      0x201

/* The number of write accesses to addresses already in the
 * L2C */
/* c0, c1 */
#define PJ1_PMU_EVENT_L2C_WRITE_HIT                   0x20000001

/* The number of read accesses served from the L2C */
/* c2, c3 */
#define PJ1_PMU_EVENT_L2C_READ_HIT                    0x20000001

/* The number of write accesses to addresses not in the L2C */
/* c0, c1 */
#define PJ1_PMU_EVENT_L2C_WRITE_MISS                  0x40000001

/* The number of L2C read accesses that resulted in an
 * external read request */
/* c2, c3 */
#define PJ1_PMU_EVENT_L2C_READ_MISS                   0x40000001

/* The number of L2C cache-to-bus external read requests */
/* c0 */
#define PJ1_PMU_EVENT_L2C_READ_COUNT                  0x80000001

/* The latency for the most recent L2C read from the external
 * bus */
/* c1 */
#define PJ1_PMU_EVENT_L2C_LATENCY                     0x80000001

#endif /* __PX_EVENT_TYPES_PJ1_PMU_H__ */
