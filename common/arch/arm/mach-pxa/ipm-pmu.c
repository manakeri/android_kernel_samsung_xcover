/*
 * This function provides the implementation of the access functions to
 * the Performance Monitoring Unit on all CPUs based on the XScale core.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License.
 *
 * Copyright (c) 2003 Intel Corporation.
 *
 * (C) Copyright 2006 Marvell International Ltd.
 * All Rights Reserved
 */

/*
#undef DEBUG
#define DEBUG
*/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <asm/atomic.h>
#include <mach/xscale-pmu.h>
#include <mach/pmu.h>
#include <mach/prm.h>

/* PRM related definitiion */
#define IPM_CLI_NAME	"IPM profiler"	/* IPM name as a client of PMU */
/* IPM profiler's priority as a client of PMU */
#define IPM_CLI_PRI	PRI_PROFILER
/* resource group: PMU related resources */
#define GROUP_PMU	0
/* resource group: Core Operating Point related resources */
#define GROUP_COP	1
/* resource group: Idle informaiton related resources */
#define GROUP_IDLE	2

static struct task_struct *ipm_thread;
static unsigned int ipm_client_id;	/* IPM's client ID as a client of PMU */
static unsigned int rsrcs_avail;	/* resources available now */
static wait_queue_head_t pmu_wait_q;
static void pmu_event_handler(prm_event, unsigned int, void *);

static atomic_t usage = ATOMIC_INIT(0);
static unsigned long id;
struct pmu_results results;

static u32 pmu_started;

#define PMU_RESET	(PMU_CLOCK_RESET | PMU_COUNTERS_RESET)
#define CLK_DIV		0x008	/* Clock divide enbable */

#define PMN3_OVERFLOW	0x10	/* Perfromance counter 3 overflow */
#define PMN2_OVERFLOW	0x08	/* Performance counter 2 overflow */
#define PMN1_OVERFLOW	0x04	/* Performance counter 1 overflow */
#define PMN0_OVERFLOW	0x02	/* Performance counter 0 overflow */
#define CCNT_OVERFLOW	0x01	/* Clock counter overflow */

static irqreturn_t pmu_irq_handler(int, void *);

#ifdef DEBUG
static void print_pmu_reg(struct pmu_results *presult)
{
	pr_debug("ccnf_of: 0x%08x  ", presult->ccnt_of);
	pr_debug("ccnt: 0x%08x\n", presult->ccnt);

	pr_debug("pmn0_of: 0x%08x  ", presult->pmn0_of);
	pr_debug("pmn0: 0x%08x\n", presult->pmn0);

	pr_debug("pmn1_of: 0x%08x  ", presult->pmn1_of);
	pr_debug("pmn1: 0x%08x\n", presult->pmn1);

	pr_debug("pmn2_of: 0x%08x  ", presult->pmn2_of);
	pr_debug("pmn2: 0x%08x\n", presult->pmn2);

	pr_debug("pmn3_of: 0x%08x  ", presult->pmn3_of);
	pr_debug("pmn3: 0x%08x\n", presult->pmn3);
}
#else
static void print_pmu_reg(struct pmu_results *presult)
{
}
#endif

/* this function should be called by IPM thread! */
int pmu_claim(void)
{
	int ret;

	if (atomic_read(&usage))
		return -EBUSY;

	/* get reference to ipm thread */
	ipm_thread = current;
	init_waitqueue_head(&pmu_wait_q);

	/* open session on behalf of IPM */
	ret = prm_open_session(IPM_CLI_PRI, IPM_CLI_NAME, pmu_event_handler,
			     (void *) ipm_thread);
	if (ret < 0) {
		printk(KERN_ERR "unable to open session of PMU access: %d\n",
				ret);
		return ret;
	}
	ipm_client_id = ret;

	/* allocate necessary PMU resources */
	if (prm_allocate_resource(ipm_client_id, PRM_CCNT, GROUP_PMU)) {
		pr_debug("prm_allocate_resource:PRM_CCNT failed\n");
		goto exit;
	}
	if (prm_allocate_resource(ipm_client_id, PRM_PMN0, GROUP_PMU)) {
		pr_debug("prm_allocate_resource:PRM_PMN0 failed\n");
		goto exit;
	}
	if (prm_allocate_resource(ipm_client_id, PRM_PMN1, GROUP_PMU)) {
		pr_debug("prm_allocate_resource:PRM_PMN1 failed\n");
		goto exit;
	}
	if (prm_allocate_resource(ipm_client_id, PRM_PMN2, GROUP_PMU)) {
		pr_debug("prm_allocate_resource:PRM_PMN2 failed\n");
		goto exit;
	}
	if (prm_allocate_resource(ipm_client_id, PRM_PMN3, GROUP_PMU)) {
		pr_debug("prm_allocate_resource:PRM_PMN3 failed\n");
		goto exit;
	}

	/* register ISR */
	if (pmu_register_isr(ipm_client_id, pmu_irq_handler,
			     (void *) &results)) {
		prm_close_session(ipm_client_id);
		goto exit;
	}

	atomic_inc(&usage);

	pmu_started = 0;

	pr_debug("PMU claimed: claim_id<%lu>\n", id + 1);

	/* commit resources */
	if (prm_commit_resources(ipm_client_id, GROUP_PMU)) {
		/* if PRM resources can't be committed, let IPM thread
		 * enter sleep
		 */
		rsrcs_avail = 0;
		pr_debug("Failed to commit PRM resources, enter sleep...\n");
		wait_event_interruptible(pmu_wait_q, (rsrcs_avail != 0));
		pr_debug("PRM resources are available again, wake up\n");
	}

	rsrcs_avail = 1;

	return ++id;

exit:
	/* free resources in case of failure */
	prm_free_resources(ipm_client_id, GROUP_PMU);

	/* close session */
	prm_close_session(ipm_client_id);

	return 0;
}
EXPORT_SYMBOL(pmu_claim);

int pmu_release(int claim_id)
{
	if (!atomic_read(&usage))
		return 0;

	if (claim_id != id)
		return -EPERM;

	/* unregister ISR */
	pmu_unregister_isr(ipm_client_id);

	/* free resources */
	prm_free_resources(ipm_client_id, GROUP_PMU);

	/* close the session */
	prm_close_session(ipm_client_id);

	atomic_dec(&usage);

	pr_debug("PMU released: claim_id<%d>\n", claim_id);

	return 0;
}
EXPORT_SYMBOL(pmu_release);

int pmu_start(u32 pmn0, u32 pmn1, u32 pmn2, u32 pmn3)
{
	int pre_type;

	memset(&results, 0, sizeof(struct pmu_results));

	print_pmu_reg(&results);

	/* disable counters */
	if (pmu_disable_event_counting(ipm_client_id)) {
		pr_debug("pmu_disable_event_counting:failed\n");
		goto exit;
	}

	/* set events */
	if (pmu_set_event(ipm_client_id, 0, &pre_type, (int) pmn0)) {
		pr_debug("pmu_set_event:pmn0 failed\n");
		goto exit;
	}
	if (pmu_set_event(ipm_client_id, 1, &pre_type, (int) pmn1)) {
		pr_debug("pmu_set_event:pmn1 failed\n");
		goto exit;
	}
	if (pmu_set_event(ipm_client_id, 2, &pre_type, (int) pmn2)) {
		pr_debug("pmu_set_event:pmn2 failed\n");
		goto exit;
	}
	if (pmu_set_event(ipm_client_id, 3, &pre_type, (int) pmn3)) {
		pr_debug("pmu_set_event:pmn3 failed\n");
		goto exit;
	}

	/* All interrupt are turned on */
	if (pmu_enable_event_interrupt(ipm_client_id, PMU_CCNT)) {
		pr_debug("pmu_enable_event_interrupt:PMU_CCNT failed\n");
		goto exit;
	}
	if (pmu_enable_event_interrupt(ipm_client_id, PMU_PMN0)) {
		pr_debug("pmu_enable_event_interrupt:PMU_PMN0 failed\n");
		goto exit;
	}
	if (pmu_enable_event_interrupt(ipm_client_id, PMU_PMN1)) {
		pr_debug("pmu_enable_event_interrupt:PMU_PMN1 failed\n");
		goto exit;
	}
	if (pmu_enable_event_interrupt(ipm_client_id, PMU_PMN2)) {
		pr_debug("pmu_enable_event_interrupt:PMU_PMN2 failed\n");
		goto exit;
	}
	if (pmu_enable_event_interrupt(ipm_client_id, PMU_PMN3)) {
		pr_debug("pmu_enable_event_interrupt:PMU_PMN3 failed\n");
		goto exit;
	}

	/* reset PMU counters */
	if (pmu_write_register(ipm_client_id, PMU_PMNC, PMU_RESET)) {
		pr_debug("pmu_write_register:PMU_PMNC failed\n");
		goto exit;
	}

	/* enable counting */
	if (pmu_enable_event_counting(ipm_client_id)) {
		pr_debug("pmu_enable_event_counting failed\n");
		goto exit;
	}

	pmu_started = 1;

	return 0;

exit:
	/* if PRM resources are appropriated, let IPM thread enter sleep */
	if (rsrcs_avail == 0) {
		pr_debug("PRM resources are appropriated, enter sleep...\n");
		wait_event_interruptible(pmu_wait_q, (rsrcs_avail != 0));
		pr_debug("PRM resources are available again, wake up\n");
	}

	return -EBUSY;
}
EXPORT_SYMBOL(pmu_start);

int pmu_stop(struct pmu_results *pmu_results)
{
	u32 ccnt;
	u32 pmn0;
	u32 pmn1;
	u32 pmn2;
	u32 pmn3;
	int ret = 0;

	if (!pmu_started) {
		pr_debug("PMU not started!\n");
		return -ENOSYS;
	}

	if (pmu_results == NULL) {
		pr_debug("pmu results is invalid pointer\n");
		goto exit;
	}

	/* read results */
	ret = pmu_read_register(ipm_client_id, PMU_CCNT, &ccnt);
	if (ret) {
		pr_debug("pmu_read_register:PMU_CCNT failed\n");
		goto exit;
	}
	ret = pmu_read_register(ipm_client_id, PMU_PMN0, &pmn0);
	if (ret) {
		pr_debug("pmu_read_register:PMU_PMN0 failed\n");
		goto exit;
	}
	ret = pmu_read_register(ipm_client_id, PMU_PMN1, &pmn1);
	if (ret) {
		pr_debug("pmu_read_register:PMU_PMN1 failed\n");
		goto exit;
	}
	ret = pmu_read_register(ipm_client_id, PMU_PMN2, &pmn2);
	if (ret) {
		pr_debug("pmu_read_register:PMU_PMN2 failed\n");
		goto exit;
	}
	ret = pmu_read_register(ipm_client_id, PMU_PMN3, &pmn3);
	if (ret) {
		pr_debug("pmu_read_register:PMU_PMN3 failed\n");
		goto exit;
	}

	/* disable counting */
	ret = pmu_disable_event_counting(ipm_client_id);
	if (ret) {
		pr_debug("pmu_disable_event_counting failed\n");
		goto exit;
	}

	/* return results */
	results.ccnt = ccnt;
	results.pmn0 = pmn0;
	results.pmn1 = pmn1;
	results.pmn2 = pmn2;
	results.pmn3 = pmn3;

	print_pmu_reg(&results);

	memcpy(pmu_results, &results, sizeof(struct pmu_results));

	pmu_started = 0;

	return 0;

exit:
	pmu_started = 0;
	/* detect the resources are appropriated by other client(s) */
	rsrcs_avail = 0;

	return ret;
}
EXPORT_SYMBOL(pmu_stop);

static irqreturn_t pmu_irq_handler(int irq, void *dev_id)
{
	struct pmu_results *pmu_results = (struct pmu_results *) dev_id;
	unsigned int flag;

	pr_debug("pmu_irq_handler: interrupt generated!\n");

	/* read the status */
	flag = pmu_read_reg(PMU_FLAG);

	/* count the overflow event */
	if (flag & PMN0_OVERFLOW) {
		pmu_results->pmn0_of++;
	}

	if (flag & PMN1_OVERFLOW) {
		pmu_results->pmn1_of++;
	}

	if (flag & PMN2_OVERFLOW) {
		pmu_results->pmn2_of++;
	}

	if (flag & PMN3_OVERFLOW) {
		pmu_results->pmn3_of++;
	}

	if (flag & CCNT_OVERFLOW) {
		pmu_results->ccnt_of++;
	}

	/* clear event */
	pmu_write_reg(PMU_FLAG, flag);

	return IRQ_HANDLED;
}

static void pmu_event_handler(prm_event event, unsigned int group_id,
			      void *data)
{
	struct task_struct *tsk = (struct task_struct *) data;

	(void) tsk;

	if (event == PRM_RES_READY) {
		pr_debug("event PRM_RES_READY received!\n");
		/* commite resource again */
		if (prm_commit_resources(ipm_client_id, group_id)) {
			pr_debug("Failed to commit PRM resources again\n");
			return;
		}

		/* register ISR again in case it was removed by
		 * higher client
		 */
		pmu_register_isr(ipm_client_id, pmu_irq_handler,
				 (void *) &results);

		/* if the PMU resources are available again, wake up the ipm
		 * profiler thread
		 */
		rsrcs_avail = 1;
		wake_up_interruptible(&pmu_wait_q);
	} else if (event == PRM_RES_APPROPRIATED) {
		/* if the PMU resources are appropriated, notify IPM profiler
		 * thread to enter sleep
		 */
		pr_debug("event PRM_RES_APPROPRIATED received!\n");
		rsrcs_avail = 0;
	}

	return;
}
