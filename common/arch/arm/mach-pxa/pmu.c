/*
 * "This software program is available to you under a choice of one of two
 * licenses.  You may choose to be licensed under either the GNU General Public
 * License (GPL) Version 2, June 1991, available at
 * http://www.fsf.org/copyleft/gpl.html, or the BSD License, the text of
 * which follows:
 *
 * Copyright (c) 1996-2005, Intel Corporation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * Neither the name of the Intel Corporation ("Intel") nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE."
 */

/*
 *  FILENAME: pmu.c
 *
 *  CORE STEPPING:
 *
 *  PURPOSE: contains all PMU C function.
 *
 *  (C) Copyright 2006 Marvell International Ltd.
 *  All Rights Reserved
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <mach/pmu.h>
#include <asm/types.h>
#include <mach/hardware.h>
#include <asm/io.h>

static struct pxa95x_pmu_info *pmu_info;

/*
 * Select one event including PMU and PML envent for PMU counter
 *
 * @par
 *	This function selects one event including Manzano and Monahans event.
 *	When type is Monahans PML Event, it is Monahans PML Event Number OR
 *	PXA3xx_EVENT_MASK. Other words, when type is Manzano event, bit31 is
 *	zero. When type is Monahans PML Event, bit31 is one.
 * @par
 *	We only use Monahans PML first four event selectors because manzano
 *	has only 4 counters and every selector can choose all PML events.
 *	We use 1:1 map from PMU counter to PML selector. So counter 0 use
 *	PML_SEL0, counter1 use PML_SEL1 and so on.
 * @param
 *	counter  PMU Counter Number. It must be between 0 and 3
 *	type     PMU And PML type
 * @return
 *	old event type before call this function.
 * @remarks
 *	required kernel/supervisor mode
 */
int pmu_select_event(int counter, int type)
{
	u32 oldevent, value, pmuevent, shift;

	if (counter < 0 || counter > 3) {
		return PMU_EVENT_INVALIDATE;
	}
	shift = counter * 8;

	value = pmu_read_reg((u32) PMU_EVTSEL);
	pmuevent = (value >> shift) & 0xFF;

	if (pmuevent >= PMU_EVENT_ASSP_0 && pmuevent <= PMU_EVENT_ASSP_3) {
		oldevent = PXA3xx_EVENT_MASK |
		    (*(pmu_info->pmu_base + (counter << 2)));
	} else {
		oldevent = pmuevent;
	}

	if (type & PXA3xx_EVENT_MASK) {
		/* PML Event */
		value &= ~(0xFF << shift);
		value |= (PMU_EVENT_ASSP_0 + counter) << shift;
		*(pmu_info->pmu_base + (counter << 2)) = type &
			(~PXA3xx_EVENT_MASK);
	} else {
		/* PMU Event */
		value &= ~(0xFF << shift);
		value |= (type & 0xFF) << shift;
	}
	pmu_write_reg((u32) PMU_EVTSEL, value);

	return oldevent;
}

#ifdef CONFIG_PM
static int pxa95x_pmu_suspend(struct platform_device *pdev,
			      pm_message_t state)
{
	return 0;
}

static int pxa95x_pmu_resume(struct platform_device *pdev)
{
	return 0;
}
#else
#define pxa95x_pmu_suspend    NULL
#define pxa95x_pmu_resume     NULL
#endif

static int pxa95x_pmu_probe(struct platform_device *pdev)
{
	struct resource *res;

	pmu_info = kzalloc(sizeof(struct pxa95x_pmu_info), GFP_KERNEL);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "pmu_regs");
	if (!res)
		goto err;
	pmu_info->pmu_base = ioremap(res->start, res->end - res->start + 1);
	return 0;
err:
	printk(KERN_ERR "pxa95x PMU init failed\n");
	return -EIO;
}

static int pxa95x_pmu_remove(struct platform_device *pdev)
{
	struct resource *res;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "pmu_regs");
	if (!res)
		goto err;
	iounmap(pmu_info->pmu_base);
	kfree(pmu_info);
	return 0;
err:
	printk(KERN_ERR "pxa95x PMU remove failed\n");
	return -EIO;
}

static struct platform_driver pxa95x_pmu_driver = {
	.driver = {
		.name = "pxa95x-pmu",
	},
	.probe = pxa95x_pmu_probe,
	.remove = pxa95x_pmu_remove,
#ifdef CONFIG_PM
	.suspend = pxa95x_pmu_suspend,
	.resume = pxa95x_pmu_resume,
#endif
};

static int __init pxa95x_pmu_init(void)
{
	return platform_driver_register(&pxa95x_pmu_driver);
}

static void __exit pxa95x_pmu_exit(void)
{
	platform_driver_unregister(&pxa95x_pmu_driver);
}

module_init(pxa95x_pmu_init);
module_exit(pxa95x_pmu_exit);
