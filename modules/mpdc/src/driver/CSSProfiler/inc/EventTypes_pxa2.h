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

#ifndef __PX_EVENT_TYPES_PXA2_H__
#define __PX_EVENT_TYPES_PXA2_H__

/**
  Events for Counter Monitor
*/
typedef enum _PXA2_Event_Type_e
{
	INVALIDATE			= 0xFFFFFFFFu,
	
	/* L1 Instruction cache miss requires fetch from external memory */
	PXA2_PMU_L1_INSTRUCTION_MISS	= 0x0u,
	
	/* L1 Instruction cache cannot deliver an instruction. this  indicate
	an instruction cache or TLB miss. This event will occur eveyr cycle in 
	which the condition is present */
	PXA2_PMU_L1_INSTRUCTION_NOT_DELIVER = 0x01,	
	
	/* Stall due to a data dependency. This event will occur every cycle in
	which the condition is present */
	PXA2_PMU_STALL_DATA_DEPENDENCY = 0x02,		
	
	/* Instruction TLB miss*/
	PXA2_PMU_INSTRUCTION_TLB_MISS = 0x03,
	
	/* Data TLB miss*/
	PXA2_PMU_DATA_TLB_MISS = 0x04,

	/* Branch instruction retired, branch may or many not have changed program
	 flow. (Counts only B and BL instruction, in both ARM and Thumb mode)*/
	PXA2_PMU_BRANCH_RETIRED = 0x05,
	
	/* Branch mis predicted. Counts only B and BL instructions, in both ARM 
	and Thumb mode*/
	PXA2_PMU_BRANCH_MISPREDICTED = 0x06,
	
	/* Instruction retired. This event will occur every cycle in which the 
	condition is present*/
	PXA2_PMU_INSTRUCTION_RETIRED = 0x07,			

	/* L1 Data cache buffer full stall. This event will occur every cycle in 
	   which the condition is present. */
	PXA2_PMU_L1_DATA_STALL = 0x08,

	/* L1 Data cache buffer full stall. This event occur for each contiguous 
	   sequence of this type of stall*/
	PXA2_PMU_L1_DATA_STALL_C = 0x09,

	/* L1 Data cache access, not including Cache Operations. All data accesses
	   are treated as cacheable access and are counted here even if the cache is 
	   not enabled*/
	PXA2_PMU_L1_DATA_ACCESS = 0x0A,

	/* L1 Data cache miss, not including Cache Operations. All data accesses 
	   are treated as cachedable accesses and are counted as misses if the data 
	   cache is not enable*/
	PXA2_PMU_L1_DATA_MISS = 0x0B,

	/* L1 data cache write-back. This event occurs once for each line that is 
	   written back from the cache*/	
	PXA2_PMU_L1_DATA_WRITE_BACK = 0x0C,

	/* Software changed the PC(b bx bl blx and eor sub rsb add adc sbc rsc orr
	   mov bic mvn ldm pop) will be counted. The count does not increment when an 
	   exception occurs and the PC changed to the exception address
	   (e.g.. IRQ, FIR, SWI,...)*/
	PXA2_PMU_SOFTWARE_CHANGED_PC = 0x0D,

	/* Branch instruction retired, branch may or may not have changed program
	   flow.(Count	ALL branch instructions, indirect as well as direct*/
	PXA2_PMU_BRANCH_RETIRED_ALL = 0x0E,

	/* Instruction issue cycle of retired instruction. This event is a count 
	   of the number of core cycle each instruction requires to issue*/	
	PXA2_PMU_INSTRUCTION_CYCLE_RETIRED = 0x0F,

	/* coprocessor stalled the instruction pipeline. This event will occur every 
	   cycle in which the condition is present */
	PXA2_PMU_COPROCESSOR_STALL_PIPE_NEW = 0x17,

	/* All change to the PC. (includes software changes and exceptions*/
	PXA2_PMU_ALL_CHANGED_PC = 0x18,

	/* Pipe line flush due to branch mis predict or exception*/
	PXA2_PMU_PIPE_FLUSH_BRANCH = 0x19,

	/* The core could not issue an instruction due to a backed stall. This 
	   event will occur every cycle in which the condition is present*/
	PXA2_PMU_BACKEND_STALL = 0x1A,				

	/* Multiplier in use. This event will occur every cycle in which the 
	   multiplier is active*/
	PXA2_PMU_MULTIPLIER = 0x1B,

	/* Multiplier stalled the instruction pipeline due to resource stall. 
	   This event will occur every cycle in which the condition is present. */

	PXA2_PMU_MULTIPLIER_STALL_PIPE = 0x1C,

	/* Co-processor stalled the instruction pipeline. This event will occur 
	   every cycle in which the condition is present. Event #0x1D is valid for 
	   1213A0, but moved to Event #0x17 for 8x5 1213B0 as per EAS ECO #3382*/
	PXA2_PMU_COPROCESSOR_STALL_PIPE = 0x1D,

	/* Data cache stalled the instruction pipeline. This event will occur 
	   every cycle in which the condition is present*/
	PXA2_PMU_DATA_CACHE_STALL_PIPE = 0x1E,

	/* Snoop request stalled the instruction pipeline. This event will occur 
	   every cycle in which the condition is present */
	PXA2_PMU_SNOOP_REQUEST_STALL_PIPE = 0x1F,

	/* Unified L2 Cache request, not including cache operations. This event 
	   includes table walks, data and instruction requests*/
	PXA2_PMU_L2_CACHE_REQUEST = 0x20,

	/* Unified L2 data request */
	PXA2_PMU_L2_DATA_REQUEST = 0x21,

	/* Unified L2 instruction request */
	PXA2_PMU_L2_INSTRUCTION_REQUEST = 0x22,

	/* Unified L2 cache miss, not including cache operations*/
	PXA2_PMU_L2_CACHE_MISS=0x23,

	/* Unified L2 cache data miss. (L2 data access that misses the cache) */
	PXA2_PMU_L2_CACHE_DATA_MISS = 0x24,

	/* Unified L2 cache instruction fetch miss. */
	PXA2_PMU_L2_CACHE_INSTRUTION_FETCH_MISS = 0x25,

	/* Unified L2 cache data read miss. */
	PXA2_PMU_L2_CACHE_DATA_READ_MISS = 0x26,

	/* Unified L2 cache data read request. */
	PXA2_PMU_L2_CACHE_DATA_READ_REQUEST = 0x27,

	/* Unified L2 cache data write miss. */
	PXA2_PMU_L2_CACHE_DATA_WRITE_MISS = 0x28,

	/* Unified L2 cache data write request. */
	PXA2_PMU_L2_CACHE_DATA_WRITE_REQUEST = 0x29,

	/* Unified L2 cache line writeback. */
	PXA2_PMU_L2_CACHE_LINE_WRITE_BACK = 0x2A,

	/* Unified L2 snoop or snoop confirm access. */
	PXA2_PMU_L2_CACHE_SNOOP_OR_SNOOP_CONFIRM_ACCESS = 0x2B,

	/* Unified L2 snoop miss. */
	PXA2_PMU_L2_CACHE_SNOOP_MISS = 0x2C,

	/* Unified L2 cache active. 
	 * This event occurs once for each L2 cycle
	 */
	PXA2_PMU_L2_CACHE_ACTIVE = 0x2D,

	/* Unified L2 push access. */
	PXA2_PMU_L2_CACHE_PUSH_ACCESS = 0x2E,

	/* Unified L2 cache access. 
	 * not including Cache Operations
	 */
	PXA2_PMU_L2_CACHE_ACCESS = 0x2F,

	/* Address bus transaction*/
	PXA2_PMU_ADDRESS_BUS = 0x40,

	/* Self initiated(Core Generated) address bus transaction*/
	PXA2_PMU_SELF_INITIATED_ADDRRES = 0x41,

	/* Bus grant delay of self initiated address bus transaction. This event 
	   will occur every bus cycle in which the condition is present. (Bus 
	   cycle count from request to grant) */
	PXA2_PMU_BUS_GRANT_DELAY_ADDRESS = 0x42,

	/* Bus clock. This event occurs once for each bus cycle*/
	PXA2_PMU_BUS_CLOCK = 0x43,				

	/* Bus initiated (core generated) data bus transaction. */
	PXA2_PMU_SELF_INITIATED_DATA = 0x44,

	/* Bus grant delay of self initiated data bus transaction.
	   This event will occur every bus cycle in which the condition is present. 
	   (Bus cycle count from request to grant) */
	PXA2_PMU_BUS_GRANT_DELAY_DATA = 0x45,

	/* Data bus transaction. This event occurs once for each data bus cycle*/
	PXA2_PMU_SELF_INITIATED_DATA_COUNT = 0x47,

	/* Data bus transaction. This event occures once for each data bus cycle*/
	PXA2_PMU_DATA_BUS_TRANSACTION = 0x48,

	/* Retired bus transaction. */
	PXA2_PMU_RETIRED_BUS_TRANSACTION = 0x49,

	/* Unified L2 cache line allocation. */
	PXA2_PMU_L2_CACHE_LINE_ALLOCATION = 0x50,

	/* Unified L2 cache line update. */
	PXA2_PMU_L2_CACHE_LINE_UPDATE = 0x51,

	/* Unified L2 recirculated operation */
	PXA2_PMU_L2_RECIRCULATED_OPERATION = 0x52,

	/* Unified L2 snoop request */
	PXA2_PMU_L2_SNOOP_REQUEST = 0x53,

	/* Unified L2 snoop confirm */
	PXA2_PMU_L2_SNOOP_CONFIRM = 0x54,

	/* Unified L2 push request */
	PXA2_PMU_L2_PUSH_REQUEST = 0x55,

	/* Unified L2 push update */
	PXA2_PMU_L2_PUSH_UPDATE = 0x56,

	/* Unified L2 push allocation */
	PXA2_PMU_L2_PUSH_ALLOCATION = 0x57,

	/* Unified L2 special operation */
	PXA2_PMU_L2_SPECIAL_OPERATION = 0x58,

	/* Unified L2 snoop hit on clean cache line */
	PXA2_PMU_L2_SNOOP_HIT_ON_CLEAN_CACHE_LINE = 0x59,

	/* Unified L2 snoop hit on dirty cache line */
	PXA2_PMU_L2_SNOOP_HIT_ON_DIRTY_CACHE_LINE = 0x5A,

	/* Address transaction retry */
	PXA2_PMU_ADDRESS_TRANSACTION_RETRY = 0x60,

	/* Snoop transaction retry */
	PXA2_PMU_SNOOP_TRANSACTION_RETRY = 0x61,

	PXA2_PMU_EVENT_ASSP_0=0x80,
	PXA2_PMU_EVENT_ASSP_1,
	PXA2_PMU_EVENT_ASSP_2,
	PXA2_PMU_EVENT_ASSP_3,
	PXA2_PMU_EVENT_ASSP_4,
	PXA2_PMU_EVENT_ASSP_5,
	PXA2_PMU_EVENT_ASSP_6,
	PXA2_PMU_EVENT_ASSP_7,

	/* Power Saving event. This event deactivates the corresponding PMU event
	   counter*/
	PXA2_PMU_PMU_EVNET_POWER_SAVING=0xFF,

	PXA2_PMU_MONAHANS_EVENT_MASK=0x10000000,

	/* Core is performing a new instruction fetch. e.g. an L2 cache miss.*/
	PXA2_PMU_MONAHANS_EVENT_CORE_INSTRUCTION_FETCH=PXA2_PMU_MONAHANS_EVENT_MASK,  
	
	/* Core is performing a new data fetch*/
	PXA2_PMU_MONAHANS_EVENT_CORE_DATA_FETCH,				
	
	/* Core read request count*/
	PXA2_PMU_MONAHANS_EVENT_CORE_READ,					
	
	/* LCD read request cout*/
	PXA2_PMU_MONAHANS_EVENT_LCD_READ,					
	
	/* DMA read request count*/
	PXA2_PMU_MONAHANS_EVENT_DMA_READ,					
	
	/* Camera interface read request cout*/
	PXA2_PMU_MONAHANS_EVENT_CAMERA_READ,
	
	/* USB 2.0 read request count*/
	PXA2_PMU_MONAHANS_EVENT_USB20_READ,
	
	/* 2D graphic read request count*/
	PXA2_PMU_MONAHANS_EVENT_2D_READ,
	
	/* USB1.1 host read reqeust count*/
	PXA2_PMU_MONAHANS_EVENT_USB11_READ,
	
	/* PX1 bus utilization. the number of cycles during which the PX1 bus is
	occupied*/
	PXA2_PMU_MONAHANS_EVENT_PX1_UNITIZATION,
	
	/* PX2(sidecar) bus utilization. the number of cycles during which the PX2
	bus is occupied*/
	PXA2_PMU_MONAHANS_EVENT_PX2_UNITIZATION,
	
	/* Dynamic memory queue for Mandris occupied. the number of cycles when 
	the DMC queue is not empty*/
	PXA2_PMU_MONAHANS_EVENT_DMC_NOT_EMPTY=PXA2_PMU_MONAHANS_EVENT_MASK|14,			
	
	/* Dynamic memory queue for Mandris occupied by more than 1 request. the 
	number of cycles when the DMC queue has 2 or more requests*/
	PXA2_PMU_MONAHANS_EVENT_DMC_2,

	/* Dynamic memory queue for Mandris occupied by more than 2 request. the 
	number of cycles when the DMC queue has 3 or more requests*/
	PXA2_PMU_MONAHANS_EVENT_DMC_3,						

	/* Dynamic memory queue for Mandris occupied by more than 3 request. the 
	number of cycles when the DMC queue is full*/
	PXA2_PMU_MONAHANS_EVENT_DMC_FULL,					

	/* Static memory queue for Mandris occupied. the number of cycles when the
	SMC queue is not empty*/
	PXA2_PMU_MONAHANS_EVENT_SMC_NOT_EMPTY,

	/* Static memory queue for Mandris occupied by more than 1 request. the 
	number of cycles when the SMC queue has 2 or more requests*/
	PXA2_PMU_MONAHANS_EVENT_SMC_2,

	/* Static memory queue for Mandris occupied by more than 2 request. the 
	number of cycles when the SMC queue has 3 or more requests*/
	PXA2_PMU_MONAHANS_EVENT_SMC_3,

	/* Static memory queue for Mandris occupied by more than 3 request. the 
	number of cycles when the SMC queue is full*/				
	PXA2_PMU_MONAHANS_EVENT_SMC_FULL,					

	/* Internal SRAM memory queue occupied
	Number of cycles when the internal memory controller queue is not empty */
	PXA2_PMU_SRAM_MEM_QUEUE_OCCUPIED_0 = PXA2_PMU_MONAHANS_EVENT_MASK|26,
	
	/* Internal SRAM memory queue occupied by more than one request
	Number of cycles when the internal memory controller queue has two or more requests */
	PXA2_PMU_SRAM_MEM_QUEUE_OCCUPIED_1,
	
	/* Internal SRAM memory queue occupied by more than two requests
	Number of cycles when the internal memory controller queue three or more requests */
	PXA2_PMU_SRAM_MEM_QUEUE_OCCUPIED_2,
	
	/*Internal SRAM memory queue occupied by more than three requests
	Number of cycles when the internal memory controller queue is full */
	PXA2_PMU_SRAM_MEM_QUEUE_OCCUPIED_3,
	
	/* Number of cycles when External Memory controller bus is occupied */
	PXA2_PMU_EXTERNAL_MEM_CONTROLLER_BUS_OCCUPIED,
	
	/* Number of cycles when External Data Flash bus is occupied */
	PXA2_PMU_EXTERNAL_DATA_FLUSH_BUS_OCCUPIED,
	
	/* Core write request count*/
	PXA2_PMU_MONAHANS_EVENT_CORE_WRITE=PXA2_PMU_MONAHANS_EVENT_MASK|36,
	
	/* DMA write request count*/
	PXA2_PMU_MONAHANS_EVENT_DMA_WRITE,
	
	/* Camera interface write request cout*/
	PXA2_PMU_MONAHANS_EVENT_CAMERA_WRITE,
	
	/* USB 2.0 write request count*/
	PXA2_PMU_MONAHANS_EVENT_USB20_WRITE,
	
	/* 2D graphic write request count*/
	PXA2_PMU_MONAHANS_EVENT_2D_WRITE,
	
	/* USB1.1 host write request count*/
	PXA2_PMU_MONAHANS_EVENT_USB11_WRITE,

	/* PX1 bus request. length of time that at least one bus request is 
	asserted on PX bus 1*/
	PXA2_PMU_MONAHANS_EVENT_PX1_REQUEST,
	
	/* PX2 bus request.  length of time that at least one bus request is 
	asserted on PX bus 2*/
	PXA2_PMU_MONAHANS_EVENT_PX2_REQUEST,

	/* PX1 bus retries. number of retries on PX bus 1*/
	PXA2_PMU_MONAHANS_EVENT_PX1_RETRIES,
	
	/* PX2 bus retries. number of retries on PX bus 2*/
	PXA2_PMU_MONAHANS_EVENT_PX2_RETRIES,

	/* Temperature level 1. time the part has spent in temperature range 1*/
	PXA2_PMU_MONAHANS_EVENT_TEMPERATURE_1,
	
	/* Temperature level 2. time the part has spent in temperature range 2*/
	PXA2_PMU_MONAHANS_EVENT_TEMPERATURE_2,
	
	/* Temperature level 3. time the part has spent in temperature range 3*/
	PXA2_PMU_MONAHANS_EVENT_TEMPERATURE_3,
	
	/* Temperature level 4. time the part has spent in temperature range 4*/
	PXA2_PMU_MONAHANS_EVENT_TEMPERATURE_4,

	/* Core read/write latency measurement. amount of time when core have more 
	than 1 read/write request outstanding*/
	PXA2_PMU_MONAHANS_EVENT_CORE_LATENCY_1,				

	/* Core read/write latency measurement. amount of time when core have more
	than 2 read/write request outstanding*/
	PXA2_PMU_MONAHANS_EVENT_CORE_LATENCY_2,
	
	/* Core read/write latency measurement. amount of time when core have more
	than 3 read/write request outstanding*/
	PXA2_PMU_MONAHANS_EVENT_CORE_LATENCY_3,
	
	/* Core read/write latency measurement. amount of time when core have more
	than 4 read/write request outstanding*/
	PXA2_PMU_MONAHANS_EVENT_CORE_LATENCY_4,

	/*  PX1 to IM read/write latency measurement. Amount of time when PX1 to 
	IM has more than 1 read/write requests outstanding.*/
	PXA2_PMU_MONAHANS_EVENT_PX1_IM_1,
	
	/*  PX1 to IM read/write latency measurement. Amount of time when PX1 to 
	IM has more than 2 read/write requests outstanding.*/
	PXA2_PMU_MONAHANS_EVENT_PX1_IM_2,

	/*  PX1 to IM read/write latency measurement. Amount of time when PX1 to 
	IM has more than 3 read/write requests outstanding.*/
	PXA2_PMU_MONAHANS_EVENT_PX1_IM_3,

	/*  PX1 to IM read/write latency measurement. Amount of time when PX1 to 
	IM has more than 4 read/write requests outstanding.*/
	PXA2_PMU_MONAHANS_EVENT_PX1_IM_4,

	/*  PX1 to DMEM/SMEM read/write latency measurement. Amount of time 
	when PX1 to DMEM/SMEM has more than 1 read/write requests outstanding.*/
	PXA2_PMU_MONAHANS_EVENT_PX1_MEM_1,
	
	/*  PX1 to DMEM/SMEM read/write latency measurement. Amount of time 
	when PX1 to DMEM/SMEM has more than 2 read/write requests outstanding.*/
	PXA2_PMU_MONAHANS_EVENT_PX1_MEM_2,
	
	/*  PX1 to DMEM/SMEM read/write latency measurement. Amount of time 
	when PX1 to DMEM/SMEM has more than 3 read/write requests outstanding.*/
	PXA2_PMU_MONAHANS_EVENT_PX1_MEM_3,

	/*  PX1 to DMEM/SMEM read/write latency measurement. Amount of time 
	when PX1 to DMEM/SMEM has more than 4 read/write requests outstanding.*/
	PXA2_PMU_MONAHANS_EVENT_PX1_MEM_4,

	/*  PX2 to IM read/write latency measurement. Amount of time when PX2 to 
	IM has more than 1 read/write requests outstanding.*/
	PXA2_PMU_MONAHANS_EVENT_PX2_IM_1,
	
	/*  PX2 to IM read/write latency measurement. Amount of time when PX2 to 
	IM has more than 2 read/write requests outstanding.*/
	PXA2_PMU_MONAHANS_EVENT_PX2_IM_2,

	/*  PX2 to IM read/write latency measurement. Amount of time when PX2 to 
	IM has more than 3 read/write requests outstanding.*/
	PXA2_PMU_MONAHANS_EVENT_PX2_IM_3,

	/*  PX2 to IM read/write latency measurement. Amount of time when PX2 to 
	IM has more than 4 read/write requests outstanding.*/
	PXA2_PMU_MONAHANS_EVENT_PX2_IM_4,

	/*  PX2 to DMEM/SMEM read/write latency measurement. Amount of time 
	when PX2 to DMEM/SMEM has more than 1 read/write requests outstanding.*/
	PXA2_PMU_MONAHANS_EVENT_PX2_MEM_1,
	
	/*  PX2 to DMEM/SMEM read/write latency measurement. Amount of time 
	when PX2 to DMEM/SMEM has more than 2 read/write requests outstanding.*/
	PXA2_PMU_MONAHANS_EVENT_PX2_MEM_2,
	
	/*  PX2 to DMEM/SMEM read/write latency measurement. Amount of time 
	when PX2 to DMEM/SMEM has more than 3 read/write requests outstanding.*/
	PXA2_PMU_MONAHANS_EVENT_PX2_MEM_3,

	/*  PX2 to DMEM/SMEM read/write latency measurement. Amount of time 
	when PX2 to DMEM/SMEM has more than 4 read/write requests outstanding.*/
	PXA2_PMU_MONAHANS_EVENT_PX2_MEM_4,
	
	/* CCNT event */
	PXA2_PMU_CCNT_CORE_CLOCK_TICK = 0xFF,
	
	/* CCNT 64 cycle event */
	PXA2_PMU_CCNT_CORE_CLOCK_TICK_64 = 0xFE,
	
	/* OS Timer event */
	PXA2_OS_TIMER_DEFAULT_EVENT_TYPE = 0xFFFF

} PXA2_Event_Type;

#endif /* __PX_EVENT_TYPES_PXA2_H__ */
