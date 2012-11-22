/*
 * include/asm-arm/hardware/cache-l2x0.h
 *
 * Copyright (C) 2007 ARM Limited
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#ifndef __ASM_ARM_HARDWARE_TAUROS3_H
#define __ASM_ARM_HARDWARE_TAUROS3_H

#define TAUROS3_CACHE_ID			0x000
#define TAUROS3_CACHE_TYPE			0x004
#define TAUROS3_CTRL			0x100
#define TAUROS3_AUX_CTRL			0x104
#define TAUROS3_EVENT_CNT_CTRL		0x200
#define TAUROS3_EVENT_CNT1_CFG		0x204
#define TAUROS3_EVENT_CNT0_CFG		0x208
#define TAUROS3_EVENT_CNT1_VAL		0x20C
#define TAUROS3_EVENT_CNT0_VAL		0x210
#define TAUROS3_INTR_MASK			0x214
#define TAUROS3_MASKED_INTR_STAT		0x218
#define TAUROS3_RAW_INTR_STAT		0x21C
#define TAUROS3_INTR_CLEAR			0x220
#define TAUROS3_EVENT_CNT2_CFG		0x224
#define TAUROS3_EVENT_CNT2_VAL		0x228
#define TAUROS3_CACHE_SYNC			0x730
#define TAUROS3_INV_LINE_PA		0x770
#define TAUROS3_INV_WAY			0x77C
#define TAUROS3_INV_ALL			0x780
#define TAUROS3_CLEAN_ALL			0x784
#define TAUROS3_CLEAN_LINE_PA		0x7B0
#define TAUROS3_CLEAN_LINE_IDX		0x7B8
#define TAUROS3_CLEAN_WAY			0x7BC
#define TAUROS3_CLEAN_INV_LINE_PA		0x7F0
#define TAUROS3_CLEAN_INV_LINE_IDX		0x7F8
#define TAUROS3_CLEAN_INV_WAY		0x7FC
#define TAUROS3_HIDDEN_DEBUG_REG    0x820
#define TAUROS3_LOCKDOWN_WAY_D		0x900
#define TAUROS3_LOCKDOWN_WAY_I		0x904
#define TAUROS3_TEST_OPERATION		0xF00
#define TAUROS3_LINE_DATA			0xF10
#define TAUROS3_LINE_TAG			0xF30
#define TAUROS3_DEBUG_CTRL			0xF40


#define TAUROS3_ASSOCIATIVITY		0x0		/*  0=8-way, 1=16-way */
#define TAUROS3_ASSOCIATIVITY_SHIFT	13		/*  [16:13] */
#define TAUROS3_WAY_SIZE		    0x1		/* 001b=16kb, 010b=32kb, 011b=64kb */
#define TAUROS3_WAY_SIZE_SHIFT		17		/* [19:17] */
#define TAUROS3_LATENCY_1_CYCLE		0x0
#define TAUROS3_LATENCY_2_CYCLE		0x1
#define TAUROS3_LATENCY_3_CYCLE		0x2
#define TAUROS3_LATENCY_4_CYCLE		0x3
#define TAUROS3_LATENCY_5_CYCLE		0x4
#define TAUROS3_LATENCY_6_CYCLE		0x5
#define TAUROS3_LATENCY_7_CYCLE		0x6
#define TAUROS3_LATENCY_8_CYCLE		0x7
#define TAUROS3_LATENCY_TGRAM		TAUROS3_LATENCY_8_CYCLE
#define TAUROS3_LATENCY_TGRAM_SHIFT	6
#define TAUROS3_LATENCY_WTRAM		TAUROS3_LATENCY_8_CYCLE
#define TAUROS3_LATENCY_WTRAM_SHIFT	3
#define TAUROS3_LATENCY_RDRAM		TAUROS3_LATENCY_8_CYCLE
#define TAUROS3_LATENCY_RDRAM_SHIFT	0
#define TAUROS3_LATENCY_RAM_MASK	0x1ff
#define TAUROS3_LATENCY_RAM		((TAUROS3_LATENCY_TGRAM << TAUROS3_LATENCY_TGRAM_SHIFT) | \
					             (TAUROS3_LATENCY_WTRAM << TAUROS3_LATENCY_WTRAM_SHIFT) | \
					             (TAUROS3_LATENCY_RDRAM << TAUROS3_LATENCY_RDRAM_SHIFT))

#ifndef __ASSEMBLY__
extern int __init tauros3_init(void __iomem *base, __u32 aux_val, __u32 aux_mask);
void tauros3_clean_all(void);
void tauros3_clean_by_mva(unsigned long addr);
void tauros3_clean_range_pa(unsigned long start, unsigned long end);
#endif
#endif
