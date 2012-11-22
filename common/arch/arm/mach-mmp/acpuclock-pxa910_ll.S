/*
 * Low-level PXA910 ddr frequency change support
 *
 * Copyright (C) 2010, Marvell Corporation.
 *
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html
 */

#include <linux/linkage.h>
#include <asm/assembler.h>

	.global freq_change, freq_sram_start, freq_sram_end

freq_change:

	mov	pc, r0

freq_sram_start:
	b	1f
	.align  5
1:
	@ frequency change sequence
	str	r1, [r2, #0x4]	@ APMU_CCR
wait_fc_done:
	ldr	r1, [r2, #0xa0] @ APMU_ISR
	ands	r1, #(1<<1)
	beq	wait_fc_done
	mov	r1, #0
	str	r1, [r2, #0xa0]

	mov	pc, lr

	.ltorg
freq_sram_end:
