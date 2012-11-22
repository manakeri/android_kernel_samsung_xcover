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

/* (C) Copyright 2009 Marvell International Ltd. All Rights Reserved */

#ifndef __CM_EVENT_TYPE_H__
#define __CM_EVENT_TYPE_H__

#include "EventTypes_pxa2.h"
#include "EventTypes_pj1.h"
#include "EventTypes_pj4.h"

/* The events for memory controller performance counters, like MC0, MC1*/
#define PJ1_EVENT_MC_CLOCK                              0x0
/* MC pipeline empty */
#define PJ1_EVENT_MC_IDLE_CYCLES                        0x1
#define PJ1_EVENT_MC_NON_IDLE_WAIT_tRFC                 0x2
#define PJ1_EVENT_MC_NON_IDLE_WAIT_tWTR                 0x3
#define PJ1_EVENT_MC_NON_IDLE_NO_DATA_BUS_UTILIZATION   0x4
#define PJ1_EVENT_MC_PRECHARGE_CMD_ALL                  0xC
#define PJ1_EVENT_MC_PRECHARGE_CMD_DATA                 0xD
#define PJ1_EVENT_MC_PRECHARGE_CMD_NON_DATA             0xE
#define PJ1_EVENT_MC_ACTIVE_CMD                         0x10
#define PJ1_EVENT_MC_READ_WRITE_CMD                     0x14
#define PJ1_EVENT_MC_READ_CMD                           0x15
#define PJ1_EVENT_MC_WRITE_CMD                          0x16
#define PJ1_EVENT_MC_ALL_DATA_REQ                       0x18
#define PJ1_EVENT_MC_READ_REQ                           0x19
#define PJ1_EVENT_MC_WRITE_REQ                          0x1A
#define PJ1_EVENT_MC_AUTO_REFRESH_ALL                   0x1C
#define PJ1_EVENT_MC_AUTO_REFRESH_NON_IDLE              0x1D

/* The events for memory controller performance counters, like MC0, MC1*/
#define PJ4_MC_CLOCK                              0x0
/* MC pipeline empty */
#define PJ4_MC_IDLE_CYCLES                        0x1
#define PJ4_MC_NON_IDLE_WAIT_tRFC                 0x2
#define PJ4_MC_NON_IDLE_WAIT_tWTR                 0x3
#define PJ4_MC_NON_IDLE_NO_DATA_BUS_UTILIZATION   0x4
#define PJ4_MC_PRECHARGE_CMD_ALL                  0xC
#define PJ4_MC_PRECHARGE_CMD_DATA                 0xD
#define PJ4_MC_PRECHARGE_CMD_NON_DATA             0xE
#define PJ4_MC_ACTIVE_CMD                         0x10
#define PJ4_MC_READ_WRITE_CMD                     0x14
#define PJ4_MC_READ_CMD                           0x15
#define PJ4_MC_WRITE_CMD                          0x16
#define PJ4_MC_ALL_DATA_REQ                       0x18
#define PJ4_MC_READ_REQ                           0x19
#define PJ4_MC_WRITE_REQ                          0x1A
#define PJ4_MC_AUTO_REFRESH_ALL                   0x1C
#define PJ4_MC_AUTO_REFRESH_NON_IDLE              0x1D

#endif // __CM_EVENT_TYPE_H__
