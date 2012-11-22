/*
 *  linux/arch/arm/mach-pxa/pxa910_squ.c
 *
 *  PXA910 SQU registration and IRQ dispatching
 *
 *  Author:	Nicolas Pitre
 *  Created:	Nov 15, 2001
 *  Copyright:	MontaVista Software Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/debugfs.h>
#include <asm/system.h>
#include <asm/irq.h>
#include <mach/hardware.h>
#include <mach/pxa910-squ.h>
#include <linux/spinlock.h>

struct squ_channel {
	char *name;
	pxa910_squ_prio prio;
	void (*irq_handler)(int, void *);
	void *data;
};

static struct squ_channel *squ_channels;
static int num_squ_channels;

static DEFINE_SPINLOCK(squ_lock);

int pxa910_request_squ (char *name, pxa910_squ_prio prio,
			 void (*irq_handler)(int, void *),
			 void *data)
{
	unsigned long flags;
	int i, found = 0;

	/* basic sanity checks */
	if (!name || !irq_handler)
		return -EINVAL;

	spin_lock_irqsave(&squ_lock, flags);

	do {
		/* try grabbing a SQU channel with the requested priority */
		for (i = 0; i < num_squ_channels; i++) {
			if ((squ_channels[i].prio == prio) &&
			    !squ_channels[i].name) {
				found = 1;
				break;
			}
		}
		/* if requested prio group is full, try a hier priority */
	} while (!found && prio--);

	if (found) {
		SDCR(i) = 0;
		squ_channels[i].name = name;
		squ_channels[i].irq_handler = irq_handler;
		squ_channels[i].data = data;
	} else {
		pr_debug("No more available SQU channels for %s\n", name);
		i = -ENODEV;
	}

	spin_unlock_irqrestore(&squ_lock, flags);
	return i;
}

void pxa910_free_squ (int squ_ch)
{
	unsigned long flags;

	if (!squ_channels[squ_ch].name) {
		pr_debug(
			"%s: trying to free channel %d which is already freed\n",
			__func__, squ_ch);
		return;
	}

	spin_lock_irqsave(&squ_lock, flags);
	SDCR(squ_ch) = 0;
	squ_channels[squ_ch].name = NULL;
	spin_unlock_irqrestore(&squ_lock, flags);
}

static irqreturn_t squ_irq_handler(int irq, void *dev_id)
{
	int i, dint;

	for (i = 0; i < num_squ_channels; i++) {
		dint = SDISR(i);
		if (dint) {
			struct squ_channel *channel = &squ_channels[i];
			if (channel->name && channel->irq_handler) {
				channel->irq_handler(i, channel->data);
				SDISR(i)=0;
			} else {
				/*
				 * IRQ for an unregistered SQU channel:
				 * let's clear the interrupts and disable it.
				 */
				pr_debug("spurious IRQ for SQU channel %d\n", i);
				SDCR(i) = 0;
			}
		}
	}
	return IRQ_HANDLED;
}

int __init pxa910_init_squ(int num_ch)
{
	int i, ret;

	squ_channels = kzalloc(sizeof(struct squ_channel) * num_ch, GFP_KERNEL);
	if (squ_channels == NULL)
		return -ENOMEM;

	ret = request_irq(IRQ_PXA168_HIFI_DMA, squ_irq_handler, IRQF_DISABLED, "SQU", NULL);
	if (ret) {
		pr_debug("Wow!  Can't register IRQ for SQU\n");
		kfree(squ_channels);
		return ret;
	}

	for (i = 0; i < num_ch; i++)
		squ_channels[i].prio = min((i & 0xf) >> 2, SQU_PRIO_LOW);

	num_squ_channels = num_ch;
	return 0;
}

EXPORT_SYMBOL(pxa910_request_squ);
EXPORT_SYMBOL(pxa910_free_squ);
