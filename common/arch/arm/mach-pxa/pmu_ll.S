@ "This software program is available to you under a choice of one of two
@ licenses.  You may choose to be licensed under either the GNU General Public
@ License (GPL) Version 2, June 1991, available at
@ http://www.fsf.org/copyleft/gpl.html, or the BSD License, the text of
@ which follows:
@
@ Copyright (c) 1996-2005, Intel Corporation. All rights reserved.
@
@ Redistribution and use in source and binary forms, with or without
@ modification, are permitted provided that the following conditions are met:
@
@ Redistributions of source code must retain the above copyright notice, this
@ list of conditions and the following disclaimer.
@
@ Redistributions in binary form must reproduce the above copyright notice, this
@ list of conditions and the following disclaimer in the documentation and/or
@ other materials provided with the distribution.
@
@ Neither the name of the Intel Corporation ("Intel") nor the names of its
@ contributors may be used to endorse or promote products derived from this
@ software without specific prior written permission.
@
@ THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
@ AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
@ IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
@ DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
@ FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
@ DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
@ SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
@ CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
@ OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
@ OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."
@
@  FILENAME:       pmu_ll.S
@
@  PURPOSE:        Provides low level PMU primitive functions written
@		   specifically for the Bulverde/Mainstone processor/platform.
@		   Specially design to fit into Intel VTUNE Architecture
@
@
@  LAST MODIFIED:  10/31/02
@******************************************************************************
@
@
@ List of primitive functions in this source code include:
@
	.global pmu_read_reg
	.global pmu_write_reg

	.text

@
@ pmu_read_reg - Read the PMU Register
@
@ Description:
@	This routine reads the designated PMU register via CoProcesser 14.
@
@ Input Parameters:
@	r0 - arg1, PMU register number to read.  Number between 0 to 8
@	if r0 contains:
@		0 -> PMNC,	PMU Control Register
@		1 -> CCNT,	PMU Clock Counter
@		2 -> PMN0,	PMU Count Register 0
@		3 -> PMN1,	PMU Count Register 1
@		4 -> PMN2,	PMU Count Register 2
@		5 -> PMN3,	PMU Count Register 3
@		6 -> INTEN,	PMU Interupt Enable Register
@		7 -> FLAG,	PMU Overflow Flag Status Register
@		8 -> EVTSEL	PMU Event Select Register
@
@ Returns:
@	r0 - 32-bit value read from CoProcessor
@
@ Registers Modified:
@	CoProcessor Register Modified: None
@	General Purpose Registers Modified: r0
@
@ NOTE:
@	Currently not support THUMB mode
@	Error checking not included

pmu_read_reg:
	@test whether cp15 or cp14
	mrc	p15, 0, r1, c0, c0
	and r1,#0x000f0000 @ARMv7 architecture
	cmp r1,#0x000f0000
	beq pmu_read_reg_cp15

	cmp	r0, #8
	addls	pc, pc, r0, lsl #2
	b	RRet
	b	RdPMNC
	b	RdCCNT
	b	RdPMN0
	b	RdPMN1
	b	RdPMN2
	b	RdPMN3
	b	RdINTEN
	b	RdFLAG
	b	RdEVTSEL

RdPMNC:
	mrc	p14, 0, r0, c0, c1, 0	@ Read PMNC
	b	RRet
RdCCNT:
	mrc	p14, 0, r0, c1, c1, 0	@ Read CCNT
	b	RRet
RdPMN0:
	mrc	p14, 0, r0, c0, c2, 0	@ Read PMN0
	b	RRet
RdPMN1:
	mrc	p14, 0, r0, c1, c2, 0	@ Read PMN1
	b	RRet
RdPMN2:
	mrc	p14, 0, r0, c2, c2, 0	@ Read PMN2
	b	RRet
RdPMN3:
	mrc	p14, 0, r0, c3, c2, 0	@ Read PMN3
	b	RRet
RdINTEN:
	mrc	p14, 0, r0, c4, c1, 0	@ Read INTEN
	b	RRet
RdFLAG:
	mrc	p14, 0, r0, c5, c1, 0	@ Read FLAG
	b	RRet
RdEVTSEL:
	mrc	p14, 0, r0, c8, c1, 0	@ Read EVTSEL

RRet:
	mov	pc, lr			@ return

pmu_read_reg_cp15:
	@this code suits PXA955
	cmp	r0, #8
	addls	pc, pc, r0, lsl #2
	b	RRet_cp15
	b	RdPMNC_cp15
	b	RdCCNT_cp15
	b	RdPMN0_cp15
	b	RdPMN1_cp15
	b	RdPMN2_cp15
	b	RdPMN3_cp15
	b	RdINTEN_cp15
	b	RdFLAG_cp15
	b	RdEVTSEL_cp15

RdPMNC_cp15:
	mrc  p15, 0, r0, c9, c12, 0
	b	RRet
RdCCNT_cp15:
	mrc  p15, 0, r0, c9, c13, 0
	b	RRet
RdPMN0_cp15:
	mov r0, #0x0
	mcr p15, 0, r0, c9, c12, 5   @select PMN0
	mrc p15, 0, r0, c9, c13, 2
	b	RRet
RdPMN1_cp15:
	mov r0, #0x1
	mcr p15, 0, r0, c9, c12, 5   @select PMN1
	mrc p15, 0, r0, c9, c13, 2
	b	RRet
RdPMN2_cp15:
	mov r0, #0x2
	mcr p15, 0, r0, c9, c12, 5   @select PMN2
	mrc p15, 0, r0, c9, c13, 2
	b	RRet
RdPMN3_cp15:
	mov r0, #0x3
	mcr p15, 0, r0, c9, c12, 5   @select PMN3
	mrc p15, 0, r0, c9, c13, 2
	b	RRet
RdINTEN_cp15:
	mrc p15, 0, r0, c9, c14, 1
	b	RRet
RdFLAG_cp15:
	mrc p15, 0, r0, c9, c12, 3
	b	RRet
RdEVTSEL_cp15:
	mrc p15, 0, r0, c9, c13, 1

RRet_cp15:
	mov	pc, lr			@ return

@
@ pmu_write_reg - Writes to the PMU Register
@
@ Description:
@	This routine writes to the designated PMU register via CoProcesser 14.
@
@ Input Parameters:
@	r0 - arg1 - PMU register number to write
@	r1 - arg2 - Value to write to PMU register
@
@	if r0 contains:
@		0 -> PMNC,	PMU Control Register
@		1 -> CCNT,	PMU Clock Counter
@		2 -> PMN0,	PMU Count Register 0
@		3 -> PMN1,	PMU Count Register 1
@		4 -> PMN2,	PMU Count Register 2
@		5 -> PMN3,	PMU Count Register 3
@		6 -> INTEN,	PMU Interupt Enable Register
@		7 -> FLAG,	PMU Overflow Flag Status Register
@		8 -> EVTSEL	PMU Event Select Register
@
@ Returns:
@	None
@
@ Registers Modified:
@	CoProcessor Register Modified: PMU Register
@	General Purpose Registers Modified: None
@
@NOTE:
@	Currently not support THUMB mode
@	Error checking not included

pmu_write_reg:
	@test whether cp15 or cp14
	mrc	p15, 0, r2, c0, c0
	and r2,#0x000f0000 @ARMv7 architecture
	cmp r2,#0x000f0000
	beq pmu_write_reg_cp15

	cmp	r0, #8
	addls	pc, pc, r0, lsl #2
	b	WRet
	b	WrPMNC
	b	WrCCNT
	b	WrPMN0
	b	WrPMN1
	b	WrPMN2
	b	WrPMN3
	b	WrINTEN
	b	WrFLAG
	b	WrEVTSEL

WrPMNC:
	mcr	p14, 0, r1, c0, c1, 0	@ Write PMNC
	b	WRet
WrCCNT:
	mcr	p14, 0, r1, c1, c1, 0	@ Write CCNT
	b	WRet
WrPMN0:
	mcr	p14, 0, r1, c0, c2, 0	@ Write PMN0
	b	WRet
WrPMN1:
	mcr	p14, 0, r1, c1, c2, 0	@ Write PMN1
	b	WRet
WrPMN2:
	mcr	p14, 0, r1, c2, c2, 0	@ Write PMN2
	b	WRet
WrPMN3:
	mcr	p14, 0, r1, c3, c2, 0	@ Write PMN3
	b	WRet
WrINTEN:
	mcr	p14, 0, r1, c4, c1, 0	@ Write INTEN
	b	WRet
WrFLAG:
	mcr	p14, 0, r1, c5, c1, 0	@ Write FLAG
	b	WRet
WrEVTSEL:
	mcr	p14, 0, r1, c8, c1, 0	@ Write EVTSEL

WRet:
	mov	pc, lr			@ return

pmu_write_reg_cp15:
	@this code suits PXA955
	cmp	r0, #8
	addls	pc, pc, r0, lsl #2
	b	WRet_cp15
	b	WrPMNC_cp15
	b	WrCCNT_cp15
	b	WrPMN0_cp15
	b	WrPMN1_cp15
	b	WrPMN2_cp15
	b	WrPMN3_cp15
	b	WrINTEN_cp15
	b	WrFLAG_cp15
	b	WrEVTSEL_cp15

WrPMNC_cp15:
	mcr  p15, 0, r0, c9, c12, 0
	b	WRet
WrCCNT_cp15:
	mcr  p15, 0, r0, c9, c13, 0
	b	WRet
WrPMN0_cp15:
	mov r0, #0x0
	mcr p15, 0, r0, c9, c12, 5   @select PMN0
	mcr p15, 0, r1, c9, c13, 2
	b	WRet
WrPMN1_cp15:
	mov r0, #0x1
	mcr p15, 0, r0, c9, c12, 5   @select PMN1
	mcr p15, 0, r1, c9, c13, 2
	b	WRet
WrPMN2_cp15:
	mov r0, #0x2
	mcr p15, 0, r0, c9, c12, 5   @select PMN2
	mcr p15, 0, r1, c9, c13, 2
	b	WRet
WrPMN3_cp15:
	mov r0, #0x3
	mcr p15, 0, r0, c9, c12, 5   @select PMN3
	mrc p15, 0, r1, c9, c13, 2
	b	WRet
WrINTEN_cp15:
	mcr p15, 0, r0, c9, c14, 1
	b	WRet
WrFLAG_cp15:
	mcr p15, 0, r0, c9, c12, 3
	b	WRet
WrEVTSEL_cp15:
	mcr p15, 0, r0, c9, c13, 1

WRet_cp15:
	mov	pc, lr			@ return
