/*
 *  linux/arch/arm/mach-pxa/ramdump.h
 *
 *  Support for the Marvell PXA RAMDUMP error handling capability.
 *
 *  Author:     Anton Eidelman (anton.eidelman@marvell.com)
 *  Created:    May 20, 2010
 *  Copyright:  (C) Copyright 2006 Marvell International Ltd.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  publishhed by the Free Software Foundation.
 */

#ifndef ARCH_ARM_MACH_PXA_RAMDUMP_H
#define ARCH_ARM_MACH_PXA_RAMDUMP_H

#include <asm/ptrace.h> /*pt_regs*/

void ramdump_save_dynamic_context(const char *str, int err,
			struct thread_info *thread, struct pt_regs *regs);
#define RAMDUMP_ERR_STR_LEN 100
#endif
