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

#include <linux/pagemap.h>
#include <linux/miscdevice.h>
#include <linux/version.h>

#include "cm_drv.h"
#include "CMProfilerDef.h"
#include "ring_buffer.h"

static unsigned int g_client_count = 0;

extern struct RingBufferInfo g_buffer_info[CM_BUFFER_NUMBER];

static int pxcm_thrd_switch_buf_read(struct file *filp, char *buf, size_t count, loff_t *ppos)
{
	unsigned int count_read;

	count_read = read_ring_buffer(&g_buffer_info[CM_BUFFER_ID_THREAD_SWITCH].buffer, buf);

	return count_read;
}

static int pxcm_thrd_switch_buf_open(struct inode *inode, struct file *fp)
{
	if (g_client_count > 0)
		return -EBUSY;

	g_client_count++;

	return 0;
}

static int pxcm_thrd_switch_buf_release(struct inode *inode, struct file *fp)
{
	g_client_count--;

	return 0;
}

static struct file_operations pxcm_thrd_switch_buf_d_fops = {
	.owner   = THIS_MODULE,
	.open    = pxcm_thrd_switch_buf_open,
	.release = pxcm_thrd_switch_buf_release,
	.read    = pxcm_thrd_switch_buf_read,
};

struct miscdevice pxcm_thrd_switch_buf_d = {
	MISC_DYNAMIC_MINOR,
	"pxcm_ts_d",
	&pxcm_thrd_switch_buf_d_fops,
};

