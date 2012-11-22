/* arch/arm/mach-mmp/acpuclock.h
 *
 * MMP architecture clock driver header
 *
 * Copyright (C) 2010 Marvell International Ltd.
 * Author: Raul Xiong <xjian@marvell.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __ARCH_ARM_MACH_MMP_ACPUCLOCK_H
#define __ARCH_ARM_MACH_MMP_ACPUCLOCK_H

#define MHZ_TO_KHZ	1000

int pxa910_get_freq(void);
int pxa910_set_freq(int index);
void gc_aclk_fc(void);
extern u32 get_pll2_freq(void);

extern void freq_change(unsigned char __iomem *sram_last_page,
		unsigned int value, unsigned int base_addr);
extern void freq_sram_start(void);
extern void freq_sram_end(void);

extern struct sysdev_class cpu_sysdev_class;

#ifdef CONFIG_PXA910_ACPU_STATS
int acpu_add_timeslot(int run_time);

#else
static inline int acpu_add_timeslot(int run_time) { return 0; }
#endif

#endif
