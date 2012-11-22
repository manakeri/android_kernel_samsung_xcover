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

#include <linux/version.h>

#include "hotspot_drv.h"	
#include "pxpmu.h"

#if defined(LINUX_VERSION_CODE) && (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 21))
#include <asm/irq_regs.h>
#endif

#ifdef PRM_SUPPORT
#if defined(LINUX_VERSION_CODE) && (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 28))
#include <mach/prm.h>
#else
#include <asm/arch/prm.h>
#endif


#define PRM_ALLOC_RES_COUNT	6
static char* prm_client_name = "Marvell Performance Data Collector";
int prm_client_id = -1;
static prm_resource_id prm_resource[PRM_ALLOC_RES_COUNT] = {PRM_CCNT, PRM_PMN0, PRM_PMN1,
	PRM_PMN2, PRM_PMN3, PRM_COP};
#else

#include <linux/interrupt.h>
#include <linux/errno.h>
static void* dev_id = 0; 
#endif

int g_pmu_irq_num = PX_IRQ_PMU;

extern irqreturn_t px_hotspot_isr(int irq, void * dev);

#if 0
static irqreturn_t hotspot_pmu_isr(int irq, void *dev_id)
{
	struct pt_regs *regs = get_irq_regs();
	pmu_isr(regs->ARM_pc, current->tgid, current->pid, 0);

	return IRQ_HANDLED;
}
#endif

/*
 * Free PMU resources and unhook the PMU ISR
 */
void free_pmu(void)
{
#ifdef PRM_SUPPORT
	if (prm_client_id != -1)
	{
		pmu_unregister_isr(prm_client_id);
		prm_free_resources(prm_client_id, 0);
		prm_close_session(prm_client_id);
		prm_client_id = -1;
		return;
	}
#else
	free_irq(g_pmu_irq_num, dev_id);

#endif	
}

/* 
 * Allocate PMU resources and hook the PMU ISR
 */
int allocate_pmu(void)
{
	int ret;
	
#ifdef PRM_SUPPORT
	int i;

	ret = prm_open_session(PRI_VTUNE, prm_client_name, NULL, NULL);
	
	if (ret < 0)
	{
		printk(KERN_ERR "failed to open prm open session\n");
		return ret;
	}

	prm_client_id = ret;

	for (i=0; i<PRM_ALLOC_RES_COUNT; i++)
	{
		ret = prm_allocate_resource(prm_client_id, prm_resource[i], 0);

		if (ret != 0)
		{
			printk(KERN_ERR "failed to allocate prm resource %d\n", 
								 prm_resource[i]);
								 
			printk(KERN_ERR "ret = %d\n", ret);
			goto alloc_pmu_err;
		}
	}

	if ((ret = pmu_register_isr(prm_client_id, px_hotspot_isr, 0)) != 0)
	{
		printk(KERN_ERR "failed to register ISR\n");
		goto alloc_pmu_err;
	}

	if ((ret = prm_commit_resources(prm_client_id, 0)) != 0)
	{
		printk(KERN_ERR "failed to commit prm resource\n");
		goto alloc_pmu_err;
	}
	
	return 0;

alloc_pmu_err:
	free_pmu();

	return ret;

#else // PRM_SUPPORT

	if ((ret = request_irq(g_pmu_irq_num/*IRQ_PMU*/, px_hotspot_isr,
#if defined(LINUX_VERSION_CODE) && (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 21))
			0//IRQF_DISABLED
#else
			SA_INTERRUPT
#endif
			, "CPA PMU", dev_id)) != 0)
	{
		printk(KERN_ERR "request_irq(%d) fails: ret = %d\n", g_pmu_irq_num, ret);
		return -EACCES;
	}

	return 0;
#endif // PRM_SUPPORT	
}

