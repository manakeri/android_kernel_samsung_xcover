/*
 *  linux/include/mipsram.h
 *
 *  Support for the Marvell MIPSRAM over PXAxxx
 *
 *  Author:	Shuki Zanyovka
 *  Created:	Feb 9, 2009
 *  Copyright:	(C) Copyright 2009 Marvell International Ltd.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  publishhed by the Free Software Foundation.
 */

#ifndef MIPS_RAM_H
#define MIPS_RAM_H

/* total buffer size = 128KB - *MUST* be an exponent of 2 */
#define MIPS_RAM_BUFFER_SZ        ((128*1024) / sizeof(unsigned int))
#define MIPS_RAM_BUFFER_SZ_BYTES  (MIPS_RAM_BUFFER_SZ * sizeof(unsigned int))

struct mipsram_descriptor {
	unsigned int *buffer;
	unsigned int buffer_phys_ptr;
	unsigned int *bufferVirt;
	unsigned int current_entry;
	unsigned int compensation_ticks;
	unsigned int current_pp_msk;
};

extern struct mipsram_descriptor mipsram_desc;

struct mipsram_descriptor *MIPSRAM_get_descriptor(void);
void MIPSRAM_Trace(unsigned long id);
void MIPSRAM_clear(void);
void mipsram_reinit_counter(void);
void mipsram_disable_counter(void);


/* Please note, that the lower bits in the case of MIPSRAM IRQ will hold the
 * IRQ number e.g. for IRQ#1 - 0x10001 */
#define MIPSRAM_IRQ_EVENTS_RANGE_START   0x10000
#define MIPSRAM_IRQ_EVENTS_RANGE_LENGTH  0x10000
#define MIPSRAM_IRQ_LEAVE_ISR_MSK		 0x08000
#define MIPSRAM_PM_EVENTS_RANGE_START (MIPSRAM_IRQ_EVENTS_RANGE_START +\
					MIPSRAM_IRQ_EVENTS_RANGE_LENGTH)
#define MIPSRAM_PM_EVENTS_RANGE_LENGTH   0x200
#define MIPSRAM_EVENT_TS_32K_CLK (MIPSRAM_PM_EVENTS_RANGE_START +\
				MIPSRAM_PM_EVENTS_RANGE_LENGTH)

#define MIPSRAM_LOG_END_MARK_EVENT		 0xFFFFFFFF
#define MIPSRAM_EVENT_RAW_DATA			 0x50000

#define MIPSRAM_PP_MSK_BIT			29
#define MIPSRAM_PP_FIELD_NOT_MSK	0x1FFFFFFF

#define MIPS_RAM_ADD_TIMESTAMP_COMPENSATION(comp_ticks) {\
		mipsram_desc.compensation_ticks += comp_ticks; }

/* This is the main TRACE add marco of MIPS_RAM
 * ASSUMPTION: This macro is called when interrupts are disabled
 * in _kernel mode_ (not supported in user-space) */
#ifndef CONFIG_CPU_PJ4
#define MIPS_RAM_ADD_TRACE(process_id) {\
	register unsigned int tIMEsTAMP;\
	register unsigned int eNTRY = mipsram_desc.current_entry;\
	if (likely(mipsram_desc.buffer)) {\
		unsigned int *temp_buffer = &(mipsram_desc.buffer[eNTRY]);\
	/* Read CCNT timestamp from CP14 */\
	 __asm__ __volatile__("mrc p14, 0, %0, c1, c1, 0" : "=r" (tIMEsTAMP));\
	 tIMEsTAMP += mipsram_desc.compensation_ticks;\
	 tIMEsTAMP &= MIPSRAM_PP_FIELD_NOT_MSK;  \
	 tIMEsTAMP |= mipsram_desc.current_pp_msk; \
     *temp_buffer++ = process_id;\
	 *temp_buffer = tIMEsTAMP;\
	 mipsram_desc.current_entry = ((eNTRY+2) & (MIPS_RAM_BUFFER_SZ-1));\
	} \
}
#else
#define MIPS_RAM_ADD_TRACE(process_id) {\
	register unsigned int tIMEsTAMP;\
	register unsigned int eNTRY = mipsram_desc.current_entry;\
	if (likely(mipsram_desc.buffer)) {\
		unsigned int *temp_buffer = &(mipsram_desc.buffer[eNTRY]);\
	/* Read CCNT timestamp from CP15 */\
	 __asm__ __volatile__("mrc p15, 0, %0, c9, c13, 0" : "=r" (tIMEsTAMP));\
	 tIMEsTAMP += mipsram_desc.compensation_ticks;\
	tIMEsTAMP &= MIPSRAM_PP_FIELD_NOT_MSK;  \
	tIMEsTAMP |= mipsram_desc.current_pp_msk; \
	*temp_buffer++ = process_id;\
	*temp_buffer = tIMEsTAMP;\
	mipsram_desc.current_entry = ((eNTRY+2) & (MIPS_RAM_BUFFER_SZ-1));\
	} \
}
#endif

#define MIPS_RAM_ADD_32K_TIME_STAMP(tIMEsTAMP_32K) {\
	register unsigned int eNTRY = mipsram_desc.current_entry;\
	if (likely(mipsram_desc.buffer)) {\
		mipsram_desc.buffer[eNTRY++] = MIPSRAM_EVENT_TS_32K_CLK;\
		mipsram_desc.buffer[eNTRY++] = tIMEsTAMP_32K;\
		mipsram_desc.current_entry = (eNTRY & (MIPS_RAM_BUFFER_SZ-1));\
	} \
}

#define MIPS_RAM_ADD_IRQ_TRACE(irq_id) \
		MIPS_RAM_ADD_TRACE(MIPSRAM_IRQ_EVENTS_RANGE_START + irq_id);

#define MIPS_RAM_ADD_IRQ_LEAVE_TRACE(irq_id) \
		MIPS_RAM_ADD_TRACE(MIPSRAM_IRQ_EVENTS_RANGE_START +\
		(irq_id | MIPSRAM_IRQ_LEAVE_ISR_MSK));

#define MIPS_RAM_ADD_PM_TRACE(pm_event_id) \
		MIPS_RAM_ADD_TRACE(MIPSRAM_PM_EVENTS_RANGE_START + pm_event_id);

#define MIPS_RAM_ADD_PP_CHANGE(new_pp) \
		mipsram_desc.current_pp_msk = (new_pp<<MIPSRAM_PP_MSK_BIT);\
		MIPS_RAM_ADD_TRACE(MIPSRAM_PM_EVENTS_RANGE_START + new_pp);


/* Mark END_OF_LOG, we assume that MIPSRAM is not stopped when this
 * MACRO is used */
#define MIPS_RAM_MARK_END_OF_LOG(mipsram_desc_ptr, c_time) {\
     mipsram_desc_ptr->bufferVirt[mipsram_desc_ptr->current_entry++] =\
		MIPSRAM_LOG_END_MARK_EVENT; \
     mipsram_desc_ptr->bufferVirt[mipsram_desc_ptr->current_entry++] =\
		c_time; \
     mipsram_desc_ptr->current_entry &= (MIPS_RAM_BUFFER_SZ-1);\
	 }

#endif/* MIPS_RAM_H*/
