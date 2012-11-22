/*
** (C) Copyright 2007 Marvell International Ltd.
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

/*++

Module Name:  $Workfile: pmu_pxa2.c $

Abstract:  
 This file is for pxa2 PMU access function

Notes:

--*/
#include <linux/kernel.h>

#include "pmu_pxa2.h"

// Read the PMU Register
u32 ReadPMUReg(int reg)
{
	u32 v = 0;
	switch(reg)
	{
	case PXA2_PMU_PMNC:
		__asm__ __volatile__ ("mrc  p14, 0, %0, c0, c1, 0" : "=r"(v)); break;
	case PXA2_PMU_CCNT:
		__asm__ __volatile__ ("mrc  p14, 0, %0, c1, c1, 0" : "=r"(v)); break;
	case PXA2_PMU_PMN0:
		__asm__ __volatile__ ("mrc  p14, 0, %0, c0, c2, 0" : "=r"(v)); break;
	case PXA2_PMU_PMN1:
		__asm__ __volatile__ ("mrc  p14, 0, %0, c1, c2, 0" : "=r"(v)); break;
	case PXA2_PMU_PMN2:
		__asm__ __volatile__ ("mrc  p14, 0, %0, c2, c2, 0" : "=r"(v)); break;
	case PXA2_PMU_PMN3:
		__asm__ __volatile__ ("mrc  p14, 0, %0, c3, c2, 0" : "=r"(v)); break;
	case PXA2_PMU_INTEN:
		__asm__ __volatile__ ("mrc  p14, 0, %0, c4, c1, 0" : "=r"(v)); break;
	case PXA2_PMU_FLAG:
		__asm__ __volatile__ ("mrc  p14, 0, %0, c5, c1, 0" : "=r"(v)); break;
	case PXA2_PMU_EVTSEL:
		__asm__ __volatile__ ("mrc  p14, 0, %0, c8, c1, 0" : "=r"(v)); break;
	default:
		break;
	}
	return v;
}


// Write the PMU Register
void WritePMUReg(int reg, u32 value)
{
	switch(reg)
	{
	case PXA2_PMU_PMNC:
		__asm__ __volatile__ ("mcr  p14, 0, %0, c0, c1, 0" : :"r"(value)); break;
	case PXA2_PMU_CCNT:
		__asm__ __volatile__ ("mcr  p14, 0, %0, c1, c1, 0" : :"r"(value)); break;
	case PXA2_PMU_PMN0:
		__asm__ __volatile__ ("mcr  p14, 0, %0, c0, c2, 0" : :"r"(value)); break;
	case PXA2_PMU_PMN1:
		__asm__ __volatile__ ("mcr  p14, 0, %0, c1, c2, 0" : :"r"(value)); break;
	case PXA2_PMU_PMN2:
		__asm__ __volatile__ ("mcr  p14, 0, %0, c2, c2, 0" : :"r"(value)); break;
	case PXA2_PMU_PMN3:
		__asm__ __volatile__ ("mcr  p14, 0, %0, c3, c2, 0" : :"r"(value)); break;
	case PXA2_PMU_INTEN:
		__asm__ __volatile__ ("mcr  p14, 0, %0, c4, c1, 0" : :"r"(value)); break;
	case PXA2_PMU_FLAG:
		__asm__ __volatile__ ("mcr  p14, 0, %0, c5, c1, 0" : :"r"(value)); break;
	case PXA2_PMU_EVTSEL:
		__asm__ __volatile__ ("mcr  p14, 0, %0, c8, c1, 0" : :"r"(value)); break;
	default:
		break;
	}
}
