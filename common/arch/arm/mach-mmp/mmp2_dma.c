/*
 *  linux/arch/arm/mach-pxa/pxa688_dma.c
 *
 *  PXA688 DMA registration and IRQ dispatching
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
#include <linux/io.h>
#include <linux/slab.h>

#include <asm/system.h>
#include <asm/irq.h>

#include <mach/hardware.h>
#include <mach/regs-icu.h>
#include <mach/mmp2_dma.h>

struct pxa688_dma_channel {
	char *name;
	void (*irq_handler) (int, void *);
	void *data;
};

static struct pxa688_dma_channel *pxa688_dma_channels;

u32 pxa688_find_dma_register_base(int dma_ch)
{
	u32 base_register = 0;

	switch (dma_ch) {
	case MDMA_CH_0:
		base_register = MDMA_CH0_BASE;
		break;
	case MDMA_CH_1:
		base_register = MDMA_CH1_BASE;
		break;
	case ADMA1_CH_0:
		base_register = ADMA1_CH0_BASE;
		break;
	case ADMA1_CH_1:
		base_register = ADMA1_CH1_BASE;
		break;
	case ADMA2_CH_0:
		base_register = ADMA2_CH0_BASE;
		break;
	case ADMA2_CH_1:
		base_register = ADMA2_CH1_BASE;
		break;
	case VDMA_CH_0:
		base_register = VDMA_CH0_BASE;
		break;
	case VDMA_CH_1:
		base_register = VDMA_CH1_BASE;
		break;
	}

	return base_register;
}

int pxa688_request_dma(char *name, pxa688_dma_channel_mapping dma_ch,
		       void (*irq_handler) (int, void *), void *data)
{
	unsigned long flags;
	int ret = 0;

	/* basic sanity checks */
	if (!name || !irq_handler || (dma_ch >= DMA_CH_NUM))
		return -EINVAL;

	local_irq_save(flags);

	if (!pxa688_dma_channels[dma_ch].name) {
		u32 base_register, msk;

		base_register = pxa688_find_dma_register_base(dma_ch);

		if (dma_ch < ADMA1_CH_0)
			PXA688_DCR(base_register) = 0;
		else if (dma_ch < VDMA_CH_0)
			PXA688_ADCR(base_register) = 0;
		else
			PXA688_VDCR(base_register) = 0;

		msk = __raw_readl(ICU_DMAIRQ_MASK);
		msk &= ~(1 << (16 + dma_ch));
		__raw_writel(msk, ICU_DMAIRQ_MASK);

		pxa688_dma_channels[dma_ch].name = name;
		pxa688_dma_channels[dma_ch].irq_handler = irq_handler;
		pxa688_dma_channels[dma_ch].data = data;
		ret = dma_ch;
	} else {
		printk(KERN_WARNING "No more available PXA688 DMA "
		       "channels for %s\n", name);
		ret = -ENODEV;
	}

	local_irq_restore(flags);

	return ret;
}

EXPORT_SYMBOL(pxa688_request_dma);

void pxa688_free_dma(int dma_ch)
{
	unsigned long flags;
	u32 base_register, msk;

	if (dma_ch >= DMA_CH_NUM) {
		printk(KERN_CRIT
		       "%s: trying to free an invalid channel %d\n",
		       __func__, dma_ch);
		return;
	}

	if (!pxa688_dma_channels[dma_ch].name) {
		printk(KERN_CRIT "%s: trying to free channel %d "
		       "which is already freed\n", __func__, dma_ch);
		return;
	}

	local_irq_save(flags);

	base_register = pxa688_find_dma_register_base(dma_ch);

	if (dma_ch < ADMA1_CH_0)
		PXA688_DCR(base_register) = 0;
	else if (dma_ch < VDMA_CH_0)
		PXA688_ADCR(base_register) = 0;
	else
		PXA688_VDCR(base_register) = 0;

	msk = __raw_readl(ICU_DMAIRQ_MASK);
	msk |= (1 << (16 + dma_ch));
	__raw_writel(msk, ICU_DMAIRQ_MASK);

	pxa688_dma_channels[dma_ch].name = NULL;
	local_irq_restore(flags);
}

EXPORT_SYMBOL(pxa688_free_dma);

static irqreturn_t pxa688_dma_irq_handler(int irq, void *dev_id)
{
	int i;
	u32 base_register;
	int dint = __raw_readl(ICU_DMAIRQ_STATUS);

	if ((dint & 0xff0000) == 0)
		return IRQ_NONE;

	for (i = 0; i < DMA_CH_NUM; i++) {
		if (dint & (1 << (i + 16))) {
			struct pxa688_dma_channel *channel =
			    &pxa688_dma_channels[i];
			base_register = pxa688_find_dma_register_base(i);
			if (channel->name && channel->irq_handler) {
				channel->irq_handler(i, channel->data);
				/*note: clear irq status in the handler */
			} else {
				/*
				 * IRQ for an unregistered DMA channel:
				 * let's clear the interrupts and disable it.
				 */
				printk(KERN_WARNING "spurious IRQ for DMA "
				       "channel %d\n", i);
				if (i < ADMA1_CH_0)
					PXA688_DCR(base_register) = 0;
				else if (i < VDMA_CH_0)
					PXA688_ADCR(base_register) = 0;
				else
					PXA688_VDCR(base_register) = 0;
			}
		}
	}

	return IRQ_HANDLED;
}

int __init pxa688_init_dma(void)
{
	int ret;
	pxa688_dma_channels = kzalloc(sizeof(struct pxa688_dma_channel)
				      * DMA_CH_NUM, GFP_KERNEL);
	if (pxa688_dma_channels == NULL)
		return -ENOMEM;
	ret = request_irq(IRQ_MMP2_DMA_RIQ, pxa688_dma_irq_handler,
			  IRQF_DISABLED | IRQF_SHARED, "DMA", "PXA688_DMA");
	if (ret) {
		printk(KERN_CRIT "Wow!  Can't register IRQ for PXA688 DMA\n");
		kfree(pxa688_dma_channels);
		return ret;
	}
	return 0;
}
