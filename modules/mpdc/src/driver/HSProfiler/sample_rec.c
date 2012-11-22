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

#include "hotspot_dsa.h"
#include "PXD_hs.h"
#include "ring_buffer.h"

#include <linux/interrupt.h>
#include <linux/sched.h>

extern struct px_hotspot_dsa *g_dsa;
extern struct RingBufferInfo g_sample_buffer;
extern wait_queue_head_t pxhs_kd_wait;

static void write_sample_record(PXD32_Hotspot_Sample *sample_rec, bool *need_flush, bool *buffer_full)
{
	g_dsa->sample_count++;

	write_ring_buffer(&g_sample_buffer.buffer, sample_rec, sizeof(PXD32_Hotspot_Sample), buffer_full, need_flush);
}

bool write_sample(PXD32_Hotspot_Sample * sample_rec)
{
	bool need_flush;
	bool buffer_full;
	
	write_sample_record(sample_rec, &need_flush, &buffer_full);

	if (need_flush && !g_sample_buffer.is_full_event_set)
	{
		g_sample_buffer.is_full_event_set = true;

		if (waitqueue_active(&pxhs_kd_wait))
			wake_up_interruptible(&pxhs_kd_wait);
	}

	return buffer_full;
}
