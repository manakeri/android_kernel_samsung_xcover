@
@ This program is free software; you can redistribute it and/or modify
@ it under the terms of the GNU General Public License as published by
@ the Free Software Foundation; either version 2 of the License, or
@ (at your option) any later version.
@
@ This program is distributed in the hope that it will be useful,
@ but WITHOUT ANY WARRANTY; without even the implied warranty of
@ MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
@ GNU General Public License for more details.
@
@ You should have received a copy of the GNU General Public License
@ along with this program; if not, write to the Free Software
@ Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
@
@
@  FILENAME:       pxa95x_dvfm_ll.S
@
@  PURPOSE:        Provides low level DVFM primitive functions written
@		   specifically for the Monahans/Zylonite processor/platform.
@
@******************************************************************************


@
@ List of primitive functions in this module:
@
    .global enter_d0cs_a
    .global exit_d0cs_a
    .global pxa_clkcfg_read
    .global pxa_clkcfg_write

.equ     CLKMGR_ACCR_OFFSET,0x0000
.equ     CLKMGR_ACSR_OFFSET,0x0004
.equ     CLKMGR_CCLKCFG_OFFSET,0x0040

.equ     DMEMC_MDCNFG_OFFSET, 0x0000
.equ     DMEMC_DDRHCAL_OFFSET,0x0060
.equ     BPMU_PWRMODE_OFFSET,0x0080

    .text

@
@
@ UINT32  enter_d0cs_a
@
@
@ Description:
@ 	put system into D0CS mode.
@
@ Input Parameters:
@       r0 - arg1, the address of Clock Manager Controller
@	r1 - arg2, the address of Dynamic Memory controller
@	r2 - arg3, the address of BPMU base address
@ Returns:
@	r0 - success (0) or failure(1)
@
@ Registers Modified:
@	ACCR, MDCNFG, DDR_HCAL
@	General Purpose Registers Modified: r3, r4, r5
@
@ NOTE:
@

enter_d0cs_a:
	stmfd	sp!, {r3, r4, r5, lr}
	@
	@ return directly if current mode is D0CS already
	@
	ldr     r3, [r0, #CLKMGR_ACSR_OFFSET]   @ load ACSR
	tst     r3, #0x04000000
	movne   r0, #0
	bne     6f

@no need to do 0,1 this is done in prepare_dmc
@0:
@	@
@	@ set DMEMC.MDCFG[29]
@	@
@	ldr     r3, [r1, #DMEMC_MDCNFG_OFFSET]	@ get MDCNFG
@	orr     r3, r3, #0x20000000		@ Set DMEMC.MDCNFG[29].
@	str     r3, [r1, #DMEMC_MDCNFG_OFFSET]	@ load MDCNFG
@1:
@	ldr     r3, [r1, #DMEMC_MDCNFG_OFFSET]	@ ensure DMEMC.MDCNFG[29] bit is written
@	tst     r3, #0x20000000
@	beq     1b
@
@	@
@	@ clear DMEMC.DDR_HCAL[31]
@	@
@	ldr     r3, [r1, #DMEMC_DDRHCAL_OFFSET]	@ get DDR_HCAL
@	bic     r3, r3, #0x80000000 		@ Insure DDR_HCAL[31] is clear
@	str     r3, [r1, #DMEMC_DDRHCAL_OFFSET]	@ load DDR_HCAL
@2:
@	ldr     r3, [r1, #DMEMC_DDRHCAL_OFFSET]	@ Insure DDR_HCAL[31] is clear
@	tst     r3, #0x80000000
@	bne     2b

	@
	@ set ACCR[D0CS] bit
	@
	ldr     r3, [r0, #CLKMGR_ACCR_OFFSET]	@ get ACCR
	orr     r3, r3, #0x04000000		@ set D0CS bit in ACCR
	str	r3, [r0, #CLKMGR_ACCR_OFFSET]   @ load ACCR
3:
	ldr     r3, [r0, #CLKMGR_ACCR_OFFSET]   @ ensure D0CS bit is written
	tst     r3, #0x04000000
	beq     3b

	@
	@ enter D0CS mode
	@
	mov     r4, #5              		@ r4: power mode
	b       enterd0cs           		@ skip the garbage before .align 5
	.align 5
enterd0cs:
	@test whether it's PXA955
	mrc	p15, 0, r5, c0, c0
	and r5,#0x000f0000 @ARMv7 architecture
	cmp r5,#0x000f0000
	beq pxa955enterd0cs

	mcr     p14, 0, r4, c7, c0, 0		@ enter D0CS mode Not PXA955
	b 4f

pxa955enterd0cs:
	mov     r4, #0x25              		   @ r4: power mode
	ldr     r3, [r2, #BPMU_PWRMODE_OFFSET] @ load PWRMODE register
	orr     r3, r3, r4					   @ set D0CS mode in PWRMODE
	str		r3, [r2, #BPMU_PWRMODE_OFFSET]      @ load PWRMODE
pwrmode_polling_entry:
	ldr     r3, [r2, #BPMU_PWRMODE_OFFSET]
	cmp		r3, r4							@r4 contains PWRMODE
	bne pwrmode_polling_entry
#ifdef CONFIG_CPU_PJ4
	dsb
	wfi
#endif
4:				 		@ wait for system to enter D0CS really
	ldr     r3, [r0, #CLKMGR_ACSR_OFFSET]   @ load ACSR
	tst     r3, #0x04000000
	beq     4b

@no need to wait here since we will wait at polling_dmc
@5:				 		@ wait for DMEMC.MDCNFG[29] clear
@	ldr     r3, [r1, #DMEMC_MDCNFG_OFFSET]
@	tst     r3, #0x20000000
@	bne     5b

6:
	@
	@ return
	@
	mov     r0, #0
	ldmfd   sp!, {r3, r4, r5, pc}		@ return

@
@
@ UINT32  exit_d0cs_a
@
@
@ Description:
@ 	let system exit D0CS mode.
@
@       r0 - arg1, the address of Clock Manager Controller
@	r1 - arg2, the address of Dynamic Memory controller
@	r2 - arg3, the address of BPMU base address
@ Returns:
@	r0 - success (0) or failure(1)
@
@ Registers Modified:
@	ACCR, MDCNFG, DDR_HCAL
@	General Purpose Registers Modified: r3, r4
@
@ NOTE:
@

exit_d0cs_a:
	stmfd	sp!, {r3,r4,r5,lr}
	@
	@ return directly if current mode is not D0CS
	@
	ldr     r3, [r0, #CLKMGR_ACSR_OFFSET]   @ load ACSR
	tst     r3, #0x04000000
	beq     6f

@no need to do 0,1 this is done in prepare_dmc
@0:
@	@
@	@ set DMEMC.MDCFG[29]
@	@
@	ldr     r3, [r1, #DMEMC_MDCNFG_OFFSET]	@ get MDCNFG
@	orr     r3, r3, #0x20000000 		@ Set DMEMC.MDCNFG[29].
@	str     r3, [r1, #DMEMC_MDCNFG_OFFSET]	@ load MDCNFG
@1:
@	ldr     r3, [r1, #DMEMC_MDCNFG_OFFSET]	@ ensure DMEMC.MDCNFG[29] bit is written
@	tst     r3, #0x20000000
@	beq     1b
@
@	@
@	@ set DMEMC.DDR_HCAL[31]
@	@
@	ldr     r3, [r1, #DMEMC_DDRHCAL_OFFSET]	@ get DDR_HCAL
@	orr     r3, r3, #0x80000000 		@ Insure DDR_HCAL[31] is set
@	str     r3, [r1, #DMEMC_DDRHCAL_OFFSET]	@ load DDR_HCAL
@2:
@	ldr     r3, [r1, #DMEMC_DDRHCAL_OFFSET]	@ Insure DDR_HCAL[31] is set
@	tst     r3, #0x80000000
@	beq     2b

	@
	@ clear ACCR[D0CS] bit
	@
	ldr     r3, [r0, #CLKMGR_ACCR_OFFSET]	@ get ACCR
	bic     r3, r3, #0x04000000		@ clear D0CS bit in ACCR
	str	r3, [r0, #CLKMGR_ACCR_OFFSET]   @ load ACCR
3:
	ldr     r3, [r0, #CLKMGR_ACCR_OFFSET]   @ ensure D0CS bit is clear
	tst     r3, #0x04000000
	bne     3b

	@
	@ exit D0CS mode
	@
	mov     r4, #5              		@ r4: power mode
	b       exitd0cs           		@ skip the garbage before .align 5
	.align 5
exitd0cs:
	@test whether it's PXA955
	mrc	p15, 0, r5, c0, c0
	and r5,#0x000f0000 @ARMv7 architecture
	cmp r5,#0x000f0000
	beq pxa955exitd0cs
	mcr     p14, 0, r4, c7, c0, 0		@ exit D0CS mode
	b 4f

pxa955exitd0cs:
	mov     r4, #0x25              		@ r4: power mode
	ldr     r3, [r2, #BPMU_PWRMODE_OFFSET]  @ load PWRMODE register
	orr     r3, r3, r4			@ set D0CS mode in PWRMODE
	str	r3, [r2, #BPMU_PWRMODE_OFFSET]  @ load PWRMODE
pwrmode_polling_exit:
	ldr     r3, [r2, #BPMU_PWRMODE_OFFSET]
	cmp	r3, r4				@r4 contains PWRMODE
	bne pwrmode_polling_exit
#ifdef CONFIG_CPU_PJ4
	dsb
	wfi
#endif
4:
	@ wait for system to exit D0CS really
	ldr     r3, [r0, #CLKMGR_ACSR_OFFSET]   @ load ACSR
	tst     r3, #0x04000000
	bne     4b

@no need to wait here since we will wait at polling_dmc
@5:				 		@ wait for DMEMC.MDCNFG[29] clear
@	ldr     r3, [r1, #DMEMC_MDCNFG_OFFSET]
@	tst     r3, #0x20000000
@	bne     5b
6:
	@
	@ return
	@
	mov     r0, #0
	ldmfd   sp!, {r3,r4,r5,pc}		@ return

@
@ UINT32 pxa_clkcfg_read
@
@ Description:
@ 	This routine reads the designated PMU register via CoProcesser 14.
@
@ Input Parameters:
@
@ Returns:
@	r0 - clkcfg value
@
@ Registers Modified:
@	CoProcessor Register Modified: None
@	General Purpose Registers Modified: None
@
@

pxa_clkcfg_read:
	mrc  p14, 0, r0, c6, c0, 0		@ Read clkcfg
	bx lr          @ return



@
@ void pxa_clkcfg_write
@
@ Description:
@ 	This routine writes to the designated ClkCFG register via
@	CoProcesser 14.
@
@ Input Parameters:
@       r0 - arg1 - Value to write to ClkCFG register
@	r1 - arg2 - Clk manager base address for PXA955 implementation

@ Returns:
@	None
@
@ Registers Modified:
@	CoProcessor Register Modified: ClkCFG Register
@	General Purpose Registers Modified: None
@
@ NOTE
@	Error checking not included
@

pxa_clkcfg_write:
	@test whether it's PXA955
	mrc	p15, 0, r2, c0, c0
	and r2,#0x000f0000 @ARMv7 architecture
	cmp r2,#0x000f0000
	beq pxa_clkcfg_write_pxa955

	mcr	 p14, 0, r0, c6, c0, 0	@ Write ClkCFG CO-processor
	b pxa_clkcfg_write_end

pxa_clkcfg_write_pxa955:
	str     r0, [r1, #CLKMGR_CCLKCFG_OFFSET] @ Store to CCLKCFG register
verify_write:
	ldr     r2, [r1, #CLKMGR_CCLKCFG_OFFSET] @ load CCLKCFG register
	cmp	r0, r2
	bne 	verify_write			 @ verify that the write succedded

pxa_clkcfg_write_end:
	bx	lr				 @ return

