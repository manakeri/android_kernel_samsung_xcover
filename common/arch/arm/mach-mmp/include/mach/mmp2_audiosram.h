/*
 *  linux/arch/arm/mach-mmp/include/mach/pxa688_audiosram.h
 *
 *  PXA688 Audio SRAM Memory Management (based on IMM)
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.

 *(C) Copyright 2008 Marvell International Ltd.
 * All Rights Reserved
 */

#ifndef _ARCH_PXA_AUDIOSRAM_H
#define _ARCH_PXA_AUDIOSRAM_H

#include <linux/list.h>

/* -------- audio sram global data --------------- */

/* error list */
enum error_cond {
	AUDIO_SRAM_ERROR_NONE = 0,
	AUDIO_SRAM_ERROR_KMALLOC,
	AUDIO_SRAM_ERROR_NOSPACE,
	AUDIO_SRAM_ERROR_SIZETOOLOW,
	AUDIO_SRAM_ERROR_ENTRY_NOTFOUND,
	AUDIO_SRAM_ERROR_PERMISSION_DENIED,
	AUDIO_SRAM_ERROR_UNSUPPORTED,
	AUDIO_SRAM_ERROR_RANGEOVERLAP,
	AUDIO_SRAM_ERROR_UNINIT,
	AUDIO_SRAM_ERROR_UNKNOWN
};

/* defines and macros */
#define AUDIO_SRAM_FIRST_KERNEL_ID		0xc0000000
#define AUDIO_SRAM_DEFAULT_DRIVER_NAME		"unnamed"
#define SRAMID_OVERLAP(id1, id2)	((id1) == (id2))

#define AUDIO_SRAM_MALLOC_L2CACHE	0x08	/* target L2 Cache */
#define AUDIO_SRAM_MALLOC_SRAM		0x01	/* target SRAM */
#define AUDIO_SRAM_MALLOC_DRAM		0x02	/* target DRAM */
/* force contiguous, immovable physical pages */
#define AUDIO_SRAM_MALLOC_HARDWARE	0x04
#define AUDIO_SRAM_CLIENT_NAME_SIZE				16

/* -------- audio sram client data --------------- */
/* entries for virtual memory sram malloc list */
struct audio_sram_virt_t {
	u32 sramid;
	/* pointer to physical start of high speed memory block */
	u32 start;
	/* pointer to first physical address after the high 
	 * speed memory block */
	u32 end;
	struct audio_sram_virt_t *prev;
	struct audio_sram_virt_t *next;
};

/* entries for the global audio sram list */
struct audio_sram_info_t {
	/* general info for the audio sram instance */
	u32 sramid;		/* audio sram ID of the caller */
	int last_error;		/* last registered error */
	/* name of the client process/driver */
	char name[AUDIO_SRAM_CLIENT_NAME_SIZE];
	struct mm_struct *mm;	/* memory struct used for remapping */
	/* pointer to virtual allocation list for this space */
	struct audio_sram_virt_t *malloc_list;
	u32 used_space;
	/* list pointers */
	struct audio_sram_info_t *prev;
	struct audio_sram_info_t *next;
};

struct audio_sram_info_t *audio_sram_info_getentry(u32);

/* imm internal flags */
#define AUDIO_SRAM_INFO_USED			0x10
#define AUDIO_SRAM_INFO_RESERVED		0x20
#define AUDIO_SRAM_INFO_DMA		AUDIO_SRAM_MALLOC_HARDWARE
#define AUDIO_SRAM_INFO_SRAM	AUDIO_SRAM_MALLOC_SRAM

/* defines and macros */
#define PAGE_IS_MAPPED(x)	((x) & AUDIO_SRAM_INFO_USED)
#define PAGE_IS_DMA(x)		((x) & AUDIO_SRAM_INFO_DMA)

/* entries for the global physical memory list */
struct audio_sram_page_t {
	u8 flag;		/* flags */
	u32 sramid;		/* each page is associated with a process id */
	/* virtual address for this page (into the sramid's space) */
	u32 virt_addr;
	/* fixed physical address associated with this page */
	u32 phys_addr;
	s16 index;		/* for debug purpose */
};

extern void *audio_sram_malloc(u32 size, u32 flags, u32 immid);
extern int audio_sram_free(void *address, u32 immid);
extern u32 audio_sram_get_physical(void *address, u32 immid);
extern u32 audio_sram_get_virtual(u32 address, struct task_struct *);
extern u32 audio_sram_get_freespace(u32 flags, u32 immid);
extern u32 audio_sram_register_kernel(char *name);

#endif
