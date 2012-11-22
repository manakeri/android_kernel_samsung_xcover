/*
 * ** (C) Copyright 2009 Marvell International Ltd.
 * **  		All Rights Reserved
 *
 * ** This software file (the "File") is distributed by Marvell International Ltd. 
 * ** under the terms of the GNU General Public License Version 2, June 1991 (the "License"). 
 * ** You may use, redistribute and/or modify this File in accordance with the terms and 
 * ** conditions of the License, a copy of which is available along with the File in the 
 * ** license.txt file or by writing to the Free Software Foundation, Inc., 59 Temple Place, 
 * ** Suite 330, Boston, MA 02111-1307 or on the worldwide web at http:www.gnu.org/licenses/gpl.txt.
 * ** THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE IMPLIED WARRANTIES 
 * ** OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY DISCLAIMED.  
 * ** The License provides additional details about this warranty disclaimer.
 * */

#include <linux/version.h>

#include "cm_drv.h"
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
static prm_resource_id prm_resource[PRM_ALLOC_RES_COUNT] = 
{
	PRM_CCNT,
	PRM_PMN0,
	PRM_PMN1,
	PRM_PMN2,
	PRM_PMN3,
	PRM_COP
};
#else

#include <linux/interrupt.h>
#include <linux/errno.h>
static void* dev_id = NULL; 
#endif

int g_pmu_irq_num = PX_IRQ_PMU;

extern struct cm_ctr_op_mach *cm_ctr_op;

#if defined(LINUX_VERSION_CODE) && (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 21))
	static irqreturn_t cm_pmu_isr(int irq, void *dev_id)
#elif defined(LINUX_VERSION_CODE) && (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 0))
	static irqreturn_t cm_pmu_isr(int irq, void *dev_id, struct pt_regs *regs)
#else
	static void cm_pmu_isr(int irq, void *dev_id, struct pt_regs *regs)
#endif

{
	cm_ctr_op->pmu_isr();
	
#if defined(LINUX_VERSION_CODE) && (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 0))
	return IRQ_HANDLED;
#endif

}
/*
 * Free PMU resources and unhook the PMU ISR
 */
void free_pmu(void)
{
#ifdef PX_CPU_PJ1
	return;
#else
#ifdef PRM_SUPPORT
	if (prm_client_id != -1)
	{
		prm_free_resources(prm_client_id, 0);
		prm_close_session(prm_client_id);
		prm_client_id = -1;
		return;
	}
#else
//	free_irq(g_pmu_irq_num/*IRQ_PMU*/, dev_id);

#endif
#endif // PX_CPU_PJ1
}

/* 
 * Allocate PMU resources and hook the PMU ISR
 */
int allocate_pmu(void)
{
#ifdef PX_CPU_PJ1
	return 0;
#else
#ifdef PRM_SUPPORT
	int ret;
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

/*	if ((ret = pmu_register_isr(prm_client_id, cm_pmu_isr, 0)) != 0)
	{
		printk(KERN_ERR "failed to register ISR\n");
		goto alloc_pmu_err;
	}
*/
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
#if 0
	if ((ret = request_irq(g_pmu_irq_num/*IRQ_PMU*/, cm_pmu_isr,
#if defined(LINUX_VERSION_CODE) && (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 21))
					IRQF_DISABLED
#else
					SA_INTERRUPT
#endif
					, "CPA PMU", dev_id)) != 0)
	{
		printk(KERN_ERR "request_irq fails: ret = %d", ret);
		return -EACCES;
	}
#endif
	return 0;
#endif // PRM_SUPPORT
#endif // PX_CPU_PJ1
}

int register_pmu_isr(void)
{
	int ret;
#ifdef PRM_SUPPORT

	if ((ret = pmu_register_isr(prm_client_id, cm_pmu_isr, 0)) != 0) {
		return ret;
	} 

#else // PRM_SUPPORT
	/* directly install ISR for PMU */
	if ((ret = request_irq(g_pmu_irq_num/*IRQ_PMU*/, cm_pmu_isr, 
#if defined(LINUX_VERSION_CODE) && (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 21))
					IRQF_DISABLED
#else
					SA_INTERRUPT
#endif
					, "CPA PMU", dev_id)) != 0) {
		return ret;
	}

#endif // PRM_SUPPORT

	//g_flgHookPMUIRQ = true;
	return 0;

}

int unregister_pmu_isr(void)
{
	int ret = 0;

#ifdef PRM_SUPPORT
	ret = pmu_unregister_isr(prm_client_id);
#else    
	free_irq(g_pmu_irq_num/*IRQ_PMU*/, dev_id);
#endif

	return ret;    
}

