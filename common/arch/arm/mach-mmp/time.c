/*
 * linux/arch/arm/mach-mmp/time.c
 *
 *   Support for clocksource and clockevents
 *
 * Copyright (C) 2008 Marvell International Ltd.
 * All rights reserved.
 *
 *   2008-04-11: Jason Chagas <Jason.chagas@marvell.com>
 *   2008-10-08: Bin Yang <bin.yang@marvell.com>
 *
 * The timers module actually includes three timers, each timer with upto
 * three match comparators. Timer #0 is used here in free-running mode as
 * the clock source, and match comparator #1 used as clock event device.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/clockchips.h>

#include <linux/io.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/cnt32_to_63.h>

#include <mach/addr-map.h>
#include <mach/regs-timers.h>
#include <mach/regs-apbc.h>
#include <mach/irqs.h>
#include <mach/cputype.h>
#include <asm/mach/time.h>

#ifdef CONFIG_LOCAL_TIMERS
#include <asm/localtimer.h>
#endif

#include "clock.h"

#define TIMERS_VIRT_BASE	TIMERS1_VIRT_BASE

#define MAX_DELTA		(0xfffffffe)
#define MIN_DELTA		(32)

#define TCR2NS_SCALE_FACTOR	10

static inline cycle_t timer_read(int counter);
static unsigned long tcr2ns_scale;

static void __init set_tcr2ns_scale(unsigned long tcr_rate)
{
	unsigned long long v = 1000000000ULL << TCR2NS_SCALE_FACTOR;
	do_div(v, tcr_rate);
	tcr2ns_scale = v;
	/*
	 * We want an even value to automatically clear the top bit
	 * returned by cnt32_to_63() without an additional run time
	 * instruction. So if the LSB is 1 then round it up.
	 */
	if (tcr2ns_scale & 1)
		tcr2ns_scale++;
}

unsigned long long sched_clock(void)
{
	unsigned long long v;
#ifdef CONFIG_PXA_32KTIMER
	v = cnt32_to_63(timer_read(1));
#else
	v = cnt32_to_63(timer_read(0));
#endif
	return (v * tcr2ns_scale) >> TCR2NS_SCALE_FACTOR;
}

unsigned long read_timer(void)
{
	/* return 32kHz ticks */
	return timer_read(1);
}
#if 0
void read_persistent_clock(struct timespec *ts)
{
	unsigned int ticks;
	ticks = read_timer();
	ts->tv_sec = (unsigned long)ticks >> 15;
	ts->tv_nsec = ((unsigned long)ticks & (0xffffffff <<15))>>6;
}
#endif

static int timer_set_next_event(unsigned long delta,
				struct clock_event_device *dev)
{
	unsigned long next;

	if (delta < MIN_DELTA)
		delta = MIN_DELTA;

#ifdef CONFIG_PXA_32KTIMER
	/* clear pending interrupt status and enable */
	__raw_writel(0x01, TIMERS_VIRT_BASE + TMR_ICR(1));
	__raw_writel(0x01, TIMERS_VIRT_BASE + TMR_IER(1));

	next = timer_read(1) + delta;
	__raw_writel(next, TIMERS_VIRT_BASE + TMR_TN_MM(1, 0));
#else
	/* clear pending interrupt status and enable */
	__raw_writel(0x01, TIMERS_VIRT_BASE + TMR_ICR(0));
	__raw_writel(0x01, TIMERS_VIRT_BASE + TMR_IER(0));

	next = timer_read(0) + delta;
	__raw_writel(next, TIMERS_VIRT_BASE + TMR_TN_MM(0, 0));
#endif

	return 0;
}

static void timer_set_mode(enum clock_event_mode mode,
			   struct clock_event_device *dev)
{
	switch (mode) {
	case CLOCK_EVT_MODE_ONESHOT:
	case CLOCK_EVT_MODE_UNUSED:
	case CLOCK_EVT_MODE_SHUTDOWN:
		/* disable the matching interrupt */
#ifdef CONFIG_PXA_32KTIMER
		__raw_writel(0x00, TIMERS_VIRT_BASE + TMR_IER(1));
#else
		__raw_writel(0x00, TIMERS_VIRT_BASE + TMR_IER(0));
#endif
		break;
	case CLOCK_EVT_MODE_RESUME:
	case CLOCK_EVT_MODE_PERIODIC:
		break;
	}
}

static struct clock_event_device ckevt = {
	.name		= "clockevent",
	.features	= CLOCK_EVT_FEAT_ONESHOT,
	.shift		= 32,
	.rating		= 200,
	.set_next_event	= timer_set_next_event,
	.set_mode	= timer_set_mode,
};

static irqreturn_t timer_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *c = dev_id;

	/* disable and clear pending interrupt status */
	__raw_writel(0x0, TIMERS_VIRT_BASE + TMR_IER(0));
	__raw_writel(0x1, TIMERS_VIRT_BASE + TMR_ICR(0));
	c->event_handler(c);
	return IRQ_HANDLED;
}

static struct irqaction timer_fast_irq = {
	.name		= "fast timer",
	.flags		= IRQF_DISABLED | IRQF_TIMER | IRQF_IRQPOLL,
	.handler	= timer_interrupt,
	.dev_id		= &ckevt,
};

static irqreturn_t timer_32k_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *c = dev_id;

	/* disable and clear pending interrupt status */
	__raw_writel(0x0, TIMERS_VIRT_BASE + TMR_IER(1));
	__raw_writel(0x1, TIMERS_VIRT_BASE + TMR_ICR(1));
	c->event_handler(c);
	return IRQ_HANDLED;
}

static struct irqaction timer_32k_irq = {
	.name		= "32k timer",
	.flags		= IRQF_DISABLED | IRQF_TIMER | IRQF_IRQPOLL,
	.handler	= timer_32k_interrupt,
	.dev_id		= &ckevt,
};

/*
 * FIXME: the timer needs some delay to stablize the counter capture
 */
static uint32_t timer_hw_inited;

static inline cycle_t timer_read(int counter)
{
	volatile int delay __maybe_unused = 2;
	unsigned long flags;
	volatile uint32_t val = 0, val2 = 0;

	if (timer_hw_inited == 0)
		return 0;

	local_irq_save(flags);

	if (counter) {
		/* 32KHz timer */
		do {
			val = __raw_readl(TIMERS_VIRT_BASE + TMR_CR(1));
			val2 = __raw_readl(TIMERS_VIRT_BASE + TMR_CR(1));
		} while (val2 != val);
	} else {
		__raw_writel(1, TIMERS_VIRT_BASE + TMR_CVWR(0));

		while (delay--) {
			val = __raw_readl(TIMERS_VIRT_BASE + TMR_CVWR(0));
		}
		val = __raw_readl(TIMERS_VIRT_BASE + TMR_CVWR(0));
	}
	local_irq_restore(flags);

	return val;
}

static cycle_t read_fast_timer(struct clocksource *cs)
{
	return timer_read(0);
}

static struct clocksource cksrc_fast = {
	.name		= "fast",
	.shift		= 20,
	.rating		= 200,
	.read		= read_fast_timer,
	.mask		= CLOCKSOURCE_MASK(32),
	.flags		= CLOCK_SOURCE_IS_CONTINUOUS,
};

static cycle_t read_32k_timer(struct clocksource *cs)
{
	return timer_read(1);
}

static struct clocksource cksrc_32k = {
	.name		= "32k",
	.shift		= 17,
	.rating		= 150,
	.read		= read_32k_timer,
	.mask		= CLOCKSOURCE_MASK(32),
	.flags		= CLOCK_SOURCE_IS_CONTINUOUS,
};

static void __init timer_config(void)
{
	unsigned long ccr = __raw_readl(TIMERS_VIRT_BASE + TMR_CCR);
	unsigned long cer = __raw_readl(TIMERS_VIRT_BASE + TMR_CER);
	unsigned long cmr = __raw_readl(TIMERS_VIRT_BASE + TMR_CMR);

	/* disable Timer 0 & 1 */
	__raw_writel(cer & ~0x3, TIMERS_VIRT_BASE + TMR_CER);

	/* clock frequency from clock/reset control register for Timer 0 */
	ccr &= ~0x1f;			/* Timer 0 (2-bit), Timer 1 (3-bit) */
	ccr |= TMR_CCR_CS_1(1);		/* Timer 1 -- 32KHz */
	__raw_writel(ccr, TIMERS_VIRT_BASE + TMR_CCR);

	/* free-running mode for Timer 0 & 1 */
	__raw_writel(cmr | 0x03, TIMERS_VIRT_BASE + TMR_CMR);

	/* Timer 0 */
	__raw_writel(0x0, TIMERS_VIRT_BASE + TMR_PLCR(0)); /* free-running */
	__raw_writel(0x7, TIMERS_VIRT_BASE + TMR_ICR(0));  /* clear status */
	__raw_writel(0x0, TIMERS_VIRT_BASE + TMR_IER(0));  /* disable int */
	/* Timer 1 */
	__raw_writel(0x0, TIMERS_VIRT_BASE + TMR_PLCR(1)); /* free-running */
	__raw_writel(0x7, TIMERS_VIRT_BASE + TMR_ICR(1));  /* clear status */
	__raw_writel(0x0, TIMERS_VIRT_BASE + TMR_IER(1));  /* disable int */

	/* enable Timer 0 & 1 */
	__raw_writel(cer | 0x03, TIMERS_VIRT_BASE + TMR_CER);
	timer_hw_inited = 1;
}

void __init timer_init(int irq0, int irq1)
{
	timer_config();

#ifdef CONFIG_PXA_32KTIMER
	set_tcr2ns_scale(32768);
#else
	set_tcr2ns_scale(CLOCK_TICK_RATE);
#endif

	cksrc_32k.mult = clocksource_hz2mult(32768, cksrc_32k.shift);
	setup_irq(irq1, &timer_32k_irq);
	cksrc_fast.mult = clocksource_hz2mult(CLOCK_TICK_RATE, cksrc_fast.shift);
	setup_irq(irq0, &timer_fast_irq);

#ifdef CONFIG_PXA_32KTIMER
	clocksource_register(&cksrc_32k);
	ckevt.mult = div_sc(32768, NSEC_PER_SEC, ckevt.shift);
#else
	clocksource_register(&cksrc_fast);
	ckevt.mult = div_sc(CLOCK_TICK_RATE, NSEC_PER_SEC, ckevt.shift);
#endif

	ckevt.max_delta_ns = clockevent_delta2ns(MAX_DELTA, &ckevt);
	ckevt.min_delta_ns = clockevent_delta2ns(MIN_DELTA, &ckevt);
	ckevt.cpumask = cpumask_of(0);
	clockevents_register_device(&ckevt);
}

