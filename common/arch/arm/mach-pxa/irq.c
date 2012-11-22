/*
 *  linux/arch/arm/mach-pxa/irq.c
 *
 *  Generic PXA IRQ handling
 *
 *  Author:	Nicolas Pitre
 *  Created:	Jun 15, 2001
 *  Copyright:	MontaVista Software Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/sysdev.h>
#include <linux/io.h>
#include <linux/irq.h>

#include <mach/hardware.h>
#include <mach/irqs.h>
#include <mach/gpio.h>

#include "generic.h"

#define IRQ_BASE		(void __iomem *)io_p2v(0x40d00000)

#define ICIP			(0x000)
#define ICMR			(0x004)
#define ICLR			(0x008)
#define ICFR			(0x00c)
#define ICPR			(0x010)
#define ICCR			(0x014)
#define ICHP			(0x018)
#define IPR(i)			(((i) < 32) ? (0x01c + ((i) << 2)) :		\
				((i) < 64) ? (0x0b0 + (((i) - 32) << 2)) :	\
				      (0x144 + (((i) - 64) << 2)))
#define IPR_VALID		(1 << 31)
#define IRQ_BIT(n)		(((n) - PXA_IRQ(0)) & 0x1f)

#define MAX_INTERNAL_IRQS	128

/*
 * This is for peripheral IRQs internal to the PXA chip.
 */

static int pxa_internal_irq_nr;

static inline int cpu_has_ipr(void)
{
	return !cpu_is_pxa25x();
}

static void pxa_mask_irq(unsigned int irq)
{
	void __iomem *base = get_irq_chip_data(irq);
	uint32_t icmr = __raw_readl(base + ICMR);

	icmr &= ~(1 << IRQ_BIT(irq));
	__raw_writel(icmr, base + ICMR);
}

static void pxa_unmask_irq(unsigned int irq)
{
	void __iomem *base = get_irq_chip_data(irq);
	uint32_t icmr = __raw_readl(base + ICMR);

	icmr |= 1 << IRQ_BIT(irq);
	__raw_writel(icmr, base + ICMR);
}

#define IRQ_REG_IDX(irq)	((irq)>>5)
#define IRQ_REG_BIT(irq)	((irq)&31)
#define IRQ_REG_MASK(irq)	(1<<IRQ_REG_BIT(irq))
#define IRQ_PRI_VALID		0x80000000
static inline void pxa_set_irq_ipr(unsigned int irq, unsigned int pri)
{
	__raw_writel(IRQ_PRI_VALID | irq, IRQ_BASE + IPR(pri));
}

/* pxa_set_irq_pri_table
 * Public service to set up the non-default IRQ priority table
 * Inputs:
 *	high_prio[], high_nr: array of IRQ ids to be assigned the highest prio,
 *		high_prio[0] is the highest.
 *		Pass NULL, 0, if not needed.
 *	low_prio[], low_nr: array of IRQ ids to be assigned the lowest prio,
 *		low_prio[0] is the lowest.
 *		Pass NULL, 0, if not needed.
 *	All other IRQ's are assigned priorities in a backward-compatible way,
 *	i.e. lower IRQ ids get lower priorities.
 */
void pxa_set_irq_pri_table(const unsigned int *high_prio, int high_nr,
			const unsigned int *low_prio, int low_nr)
{
	unsigned int pri_set_db[] = {0, 0, 0};
	unsigned int i = 0, irq;
	unsigned int max_irq = pxa_internal_irq_nr - 1; /* max valid IRQ */

	if (!cpu_has_ipr())
		return;
	BUG_ON(IRQ_REG_IDX(max_irq) >= ARRAY_SIZE(pri_set_db));

	/* We don't set pri_set_db unused bits even though nr of IRQ's is
	 * not a multiple of 32. The code below disregards these bits.
	 */

	/* Set the IRQ's listed in high_prio to the top of the table (higest prio) */
	if (high_prio)
		for (i = 0; i < high_nr; i++) {
			irq = high_prio[i];
			BUG_ON(irq > max_irq);
			pxa_set_irq_ipr(irq, i);
			/* Mark this irq as prio assigned */
			pri_set_db[IRQ_REG_IDX(irq)] |= IRQ_REG_MASK(irq);
		}

	/* Set the IRQ's listed in low_prio to the top of the table(lowest prio) */
	if (low_prio)
		for (i = 0; i < low_nr; i++) {
			irq = low_prio[i];
			BUG_ON(irq > max_irq);
			pxa_set_irq_ipr(low_prio[i], max_irq - i);
			/* Mark this irq as prio assigned */
			pri_set_db[IRQ_REG_IDX(irq)] |= IRQ_REG_MASK(irq);
		}
	low_nr = max_irq - low_nr;
	/* Fill pri entries [high_nr..low_nr] inclusive with all the IRQ's
	 *left unassigned, lower IRQ's get lower prio to keep
	 * backwards compatibility with the old IRQ handling code
	 */
	for (irq = 0; irq <= max_irq; i >>= 1, irq++) {
		if (IRQ_REG_BIT(irq) == 0)
			i = ~pri_set_db[IRQ_REG_IDX(irq)];
		if (i&1)
			/*low_nr points to the next unused priority entry */
			pxa_set_irq_ipr(irq, low_nr--);
	}
	BUG_ON(high_nr != (low_nr+1));
}
EXPORT_SYMBOL(pxa_set_irq_pri_table);

static struct irq_chip pxa_internal_irq_chip = {
	.name		= "SC",
	.ack		= pxa_mask_irq,
	.mask		= pxa_mask_irq,
	.unmask		= pxa_unmask_irq,
};

/*
 * GPIO IRQs for GPIO 0 and 1
 */
static int pxa_set_low_gpio_type(unsigned int irq, unsigned int type)
{
	int gpio = irq - IRQ_GPIO0;

	if (__gpio_is_occupied(gpio)) {
		pr_err("%s failed: GPIO is configured\n", __func__);
		return -EINVAL;
	}

	if (type & IRQ_TYPE_EDGE_RISING)
		GRER0 |= GPIO_bit(gpio);
	else
		GRER0 &= ~GPIO_bit(gpio);

	if (type & IRQ_TYPE_EDGE_FALLING)
		GFER0 |= GPIO_bit(gpio);
	else
		GFER0 &= ~GPIO_bit(gpio);

	return 0;
}

static void pxa_ack_low_gpio(unsigned int irq)
{
	GEDR0 = (1 << (irq - IRQ_GPIO0));
}

static void pxa_mask_low_gpio(unsigned int irq)
{
	struct irq_desc *desc = irq_to_desc(irq);

	desc->chip->mask(irq);
}

static void pxa_unmask_low_gpio(unsigned int irq)
{
	struct irq_desc *desc = irq_to_desc(irq);

	desc->chip->unmask(irq);
}

static struct irq_chip pxa_low_gpio_chip = {
	.name		= "GPIO-l",
	.ack		= pxa_ack_low_gpio,
	.mask		= pxa_mask_low_gpio,
	.unmask		= pxa_unmask_low_gpio,
	.set_type	= pxa_set_low_gpio_type,
};

static void __init pxa_init_low_gpio_irq(set_wake_t fn)
{
	int irq;

	/* clear edge detection on GPIO 0 and 1 */
	GFER0 &= ~0x3;
	GRER0 &= ~0x3;
	GEDR0 = 0x3;

	for (irq = IRQ_GPIO0; irq <= IRQ_GPIO1; irq++) {
		set_irq_chip(irq, &pxa_low_gpio_chip);
		set_irq_handler(irq, handle_edge_irq);
		set_irq_flags(irq, IRQF_VALID);
	}

	pxa_low_gpio_chip.set_wake = fn;
}

static inline void __iomem *irq_base(int i)
{
	static unsigned long phys_base[] = {
		0x40d00000,
		0x40d0009c,
		0x40d00130,
	};

	return (void __iomem *)io_p2v(phys_base[i >> 5]);
}

static unsigned int pxa_high_pri_irqs[] = {IRQ_LCDPARALLEL};

void __init pxa_init_irq(int irq_nr, set_wake_t fn)
{
	int irq, i, n;

	BUG_ON(irq_nr > MAX_INTERNAL_IRQS);

	pxa_internal_irq_nr = irq_nr;

	for (n = 0; n < irq_nr; n += 32) {
		void __iomem *base = irq_base(n);

		__raw_writel(0, base + ICMR);	/* disable all IRQs */
		__raw_writel(0, base + ICLR);	/* all IRQs are IRQ, not FIQ */
		for (i = n; (i < (n + 32)) && (i < irq_nr); i++) {
			/* initialize interrupt priority */
			if (cpu_has_ipr())
				__raw_writel(i | IPR_VALID, IRQ_BASE + IPR(i));

			irq = PXA_IRQ(i);
			set_irq_chip(irq, &pxa_internal_irq_chip);
			set_irq_chip_data(irq, base);
			set_irq_handler(irq, handle_level_irq);
			set_irq_flags(irq, IRQF_VALID);
		}
	}

	/* only unmasked interrupts kick us out of idle */
	__raw_writel(1, irq_base(0) + ICCR);

        pxa_set_irq_pri_table(pxa_high_pri_irqs, sizeof(pxa_high_pri_irqs)/sizeof(unsigned int), 0, 0);

	pxa_internal_irq_chip.set_wake = fn;
	pxa_init_low_gpio_irq(fn);
}

#ifdef CONFIG_PM
static unsigned long saved_icmr[MAX_INTERNAL_IRQS/32];
static unsigned long saved_ipr[MAX_INTERNAL_IRQS];

static int pxa_irq_suspend(struct sys_device *dev, pm_message_t state)
{
	int i;

	for (i = 0; i < pxa_internal_irq_nr; i += 32) {
		void __iomem *base = irq_base(i);

		saved_icmr[i >> 5] = __raw_readl(base + ICMR);
		__raw_writel(0, base + ICMR);
	}

	if (cpu_has_ipr()) {
		for (i = 0; i < pxa_internal_irq_nr; i++)
			saved_ipr[i] = __raw_readl(IRQ_BASE + IPR(i));
	}

	return 0;
}

static int pxa_irq_resume(struct sys_device *dev)
{
	int i;

	for (i = 0; i < pxa_internal_irq_nr; i += 32) {
		void __iomem *base = irq_base(i);

		__raw_writel(saved_icmr[i >> 5], base + ICMR);
		__raw_writel(0, base + ICLR);
	}

	if (!cpu_is_pxa25x())
		for (i = 0; i < pxa_internal_irq_nr; i++)
			__raw_writel(saved_ipr[i], IRQ_BASE + IPR(i));

	__raw_writel(1, IRQ_BASE + ICCR);
	return 0;
}
#else
#define pxa_irq_suspend		NULL
#define pxa_irq_resume		NULL
#endif

struct sysdev_class pxa_irq_sysclass = {
	.name		= "irq",
	.suspend	= pxa_irq_suspend,
	.resume		= pxa_irq_resume,
};

static int __init pxa_irq_init(void)
{
	return sysdev_class_register(&pxa_irq_sysclass);
}

core_initcall(pxa_irq_init);
