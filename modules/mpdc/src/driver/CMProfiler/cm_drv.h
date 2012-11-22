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

#ifndef __PX_COUNTER_MONITOR_DRV_H__
#define __PX_COUNTER_MONITOR_DRV_H__

#include <linux/types.h>

#include "ring_buffer.h"
#include "CMProfilerDef.h"

struct cm_ctr_op_mach
{
	int (*start)(void);
	int (*stop)(void);
	int (*pause)(void);
	int (*resume)(void);
	bool (*read_counter)(int counter_id, unsigned long long *p_value);
	void (*pmu_isr)(void);
};

extern struct cm_ctr_op_mach cm_op_pxa2;
extern struct cm_ctr_op_mach cm_op_pj1;
extern struct cm_ctr_op_mach cm_op_pj4;

#endif /* __PX_COUNTER_MONITOR_DRV_H__ */
