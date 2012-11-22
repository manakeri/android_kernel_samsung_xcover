/*
 *  linux/arch/arm/mach-mmp/irq-mmp2.c
 *
 *  Generic IRQ handling, GPIO IRQ demultiplexing, etc.
 *
 *  Author:	Haojian Zhuang <haojian.zhuang@marvell.com>
 *  Copyright:	Marvell International Ltd.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/irq.h>
#include <linux/io.h>

#include <mach/regs-icu.h>

#include "common.h"

static void icu_mask_irq(unsigned int irq)
{
	unsigned int _irq = irq - IRQ_MMP_START;
	uint32_t r = __raw_readl(ICU_INT_CONF(_irq));

	r &= ~ICU_INT_ROUTE_PJ4_IRQ;
	__raw_writel(r, ICU_INT_CONF(_irq));
}

static void icu_unmask_irq(unsigned int irq)
{
	unsigned int _irq = irq - IRQ_MMP_START;
	uint32_t r = __raw_readl(ICU_INT_CONF(_irq));

	r |= ICU_INT_ROUTE_PJ4_IRQ;
	__raw_writel(r, ICU_INT_CONF(_irq));
}

static struct irq_chip icu_irq_chip = {
	.name		= "icu_irq",
	.mask		= icu_mask_irq,
	.mask_ack	= icu_mask_irq,
	.unmask		= icu_unmask_irq,
	.disable	= icu_mask_irq,
};

#define SECOND_IRQ_MASK(_name_, irq_base, prefix)			\
static void _name_##_mask_irq(unsigned int irq)				\
{									\
	uint32_t r;							\
	r = __raw_readl(prefix##_MASK) | (1 << (irq - irq_base));	\
	__raw_writel(r, prefix##_MASK);					\
}

#define SECOND_IRQ_UNMASK(_name_, irq_base, prefix)			\
static void _name_##_unmask_irq(unsigned int irq)			\
{									\
	uint32_t r;							\
	r = __raw_readl(prefix##_MASK) & ~(1 << (irq - irq_base));	\
	__raw_writel(r, prefix##_MASK);					\
}

#define SECOND_IRQ_DEMUX(_name_, irq_base, prefix)			\
static void _name_##_irq_demux(unsigned int irq, struct irq_desc *desc)	\
{									\
	unsigned long status, mask, n;					\
	mask = __raw_readl(prefix##_MASK);				\
	while (1) {							\
		status = __raw_readl(prefix##_STATUS) & ~mask;		\
		if (status == 0)					\
			break;						\
		n = find_first_bit(&status, BITS_PER_LONG);		\
		while (n < BITS_PER_LONG) {				\
			generic_handle_irq(irq_base + n);		\
			n = find_next_bit(&status, BITS_PER_LONG, n+1);	\
		}							\
	}								\
}

#define SECOND_IRQ_CHIP(_name_, irq_base, prefix)			\
SECOND_IRQ_MASK(_name_, irq_base, prefix)				\
SECOND_IRQ_UNMASK(_name_, irq_base, prefix)				\
SECOND_IRQ_DEMUX(_name_, irq_base, prefix)				\
static struct irq_chip _name_##_irq_chip = {				\
	.name		= #_name_,					\
	.mask		= _name_##_mask_irq,				\
	.unmask		= _name_##_unmask_irq,				\
	.disable	= _name_##_mask_irq,				\
}

SECOND_IRQ_CHIP(twsi,   IRQ_MMP2_TWSI_BASE,   MMP2_ICU_INT17);
SECOND_IRQ_CHIP(rtc,    IRQ_MMP2_RTC_BASE,    MMP2_ICU_INT5);
SECOND_IRQ_CHIP(keypad, IRQ_MMP2_KEYPAD_BASE, MMP2_ICU_INT9);
SECOND_IRQ_CHIP(misc,   IRQ_MMP2_MISC_BASE,   MMP2_ICU_INT35);
SECOND_IRQ_CHIP(hsi1,   IRQ_MMP2_HSI1_BASE,   MMP2_ICU_INT51);
SECOND_IRQ_CHIP(hsi0,   IRQ_MMP2_HSI0_BASE,   MMP2_ICU_INT55);

static void init_mux_irq(struct irq_chip *chip, int start, int num)
{
	int irq;

	for (irq = start; num > 0; irq++, num--) {
		/* mask and clear the IRQ */
		chip->mask(irq);
		if (chip->ack)
			chip->ack(irq);

		set_irq_chip(irq, chip);
		set_irq_flags(irq, IRQF_VALID);
		set_irq_handler(irq, handle_level_irq);
	}
}

static void icu_handle_irq(unsigned int irq, struct irq_desc *desc)
{
	struct irq_chip *chip = get_irq_chip(irq);
	uint32_t r = __raw_readl(MMP2_ICU_PJ4_IRQ_SEL);

	if (r & (1<<6))
		generic_handle_irq((r & 0x3f) + IRQ_MMP_START);

	/* primary controller ack'ing */
	chip->ack(irq);
	/* primary controller unmasking */
	chip->unmask(irq);
}

void __init mmp3_init_icu(void)
{
	struct irq_chip *chip;
	int irq;

	for (irq = IRQ_MMP_START; irq < IRQ_MMP2_MUX_BASE; irq++) {
		icu_mask_irq(irq);
		set_irq_chip(irq, &icu_irq_chip);
		set_irq_flags(irq, IRQF_VALID);

		switch (irq) {
		case IRQ_MMP2_PMIC_MUX:
		case IRQ_MMP2_RTC_MUX:
		case IRQ_MMP2_KEYPAD_MUX:
		case IRQ_MMP2_TWSI_MUX:
		case IRQ_MMP2_MISC_MUX:
		case IRQ_MMP2_MIPI_HSI1_MUX:
		case IRQ_MMP2_MIPI_HSI0_MUX:
			break;
		default:
			set_irq_handler(irq, handle_level_irq);
			break;
		}
	}

	init_mux_irq(&twsi_irq_chip, IRQ_MMP2_TWSI_BASE, 5);
	init_mux_irq(&rtc_irq_chip, IRQ_MMP2_RTC_BASE, 2);
	init_mux_irq(&keypad_irq_chip, IRQ_MMP2_KEYPAD_BASE, 3);
	init_mux_irq(&misc_irq_chip, IRQ_MMP2_MISC_BASE, 15);
	init_mux_irq(&hsi1_irq_chip, IRQ_MMP2_HSI1_BASE, 2);
	init_mux_irq(&hsi0_irq_chip, IRQ_MMP2_HSI0_BASE, 2);

	set_irq_chained_handler(IRQ_MMP2_TWSI_MUX, twsi_irq_demux);
	set_irq_chained_handler(IRQ_MMP2_RTC_MUX, rtc_irq_demux);
	set_irq_chained_handler(IRQ_MMP2_KEYPAD_MUX, keypad_irq_demux);
	set_irq_chained_handler(IRQ_MMP2_MISC_MUX, misc_irq_demux);
	set_irq_chained_handler(IRQ_MMP2_MIPI_HSI1_MUX, hsi1_irq_demux);
	set_irq_chained_handler(IRQ_MMP2_MIPI_HSI0_MUX, hsi0_irq_demux);

	set_irq_chained_handler(IRQ_LEGACYIRQ, icu_handle_irq);
	chip = get_irq_chip(IRQ_LEGACYIRQ);
	chip->unmask(IRQ_LEGACYIRQ);
}
