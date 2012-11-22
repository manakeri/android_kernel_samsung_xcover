/*
 * arch/arm/mach-pxa/time.c
 *
 * PXA clocksource, clockevents, and OST interrupt handlers.
 * Copyright (c) 2007 by Bill Gatliff <bgat@billgatliff.com>.
 *
 * Derived from Nicolas Pitre's PXA timer handler Copyright (c) 2001
 * by MontaVista Software, Inc.  (Nico, your code rocks!)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/clockchips.h>
#include <linux/sched.h>
#include <linux/cnt32_to_63.h>

#include <asm/div64.h>
#include <asm/mach/irq.h>
#include <asm/mach/time.h>
#include <mach/regs-ost.h>

/*
 * This is PXA's sched_clock implementation. This has a resolution
 * of at least 308 ns and a maximum value of 208 days.
 *
 * The return value is guaranteed to be monotonic in that range as
 * long as there is always less than 582 seconds between successive
 * calls to sched_clock() which should always be the case in practice.
 */
static cycle_t pxa_timer_read(struct clocksource *cs)
{
	return (cycle_t)OSCR4;
}

static unsigned long long pxa_32k_ticks_to_nsec(cycle_t cycles)
{
	return (unsigned long long)cycles * 1000 *5*5*5*5*5*5 >> 9;
}


unsigned long long sched_clock(void)
{
	return pxa_32k_ticks_to_nsec(pxa_timer_read(NULL));
}

#define MIN_OSCR_DELTA 16

static irqreturn_t
pxa_timer_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *c = dev_id;

	if (!(OSSR & OST_C4))
		return IRQ_NONE;

	OIER &= ~OST_C4;
	OSSR = OST_C4;
	c->event_handler(c);

	return IRQ_HANDLED;
}

static int
pxa_timer_set_next_event(unsigned long delta, struct clock_event_device *dev)
{
	unsigned long flags, next, oscr;

	raw_local_irq_save(flags);
	OIER |= OST_C4;
	next = OSCR4 + delta;
	OSMR4 = next;
	oscr = OSCR4;
	raw_local_irq_restore(flags);
	return (signed)(next - oscr) <= MIN_OSCR_DELTA ? -ETIME : 0;
}

static void
pxa_timer_set_mode(enum clock_event_mode mode, struct clock_event_device *dev)
{
	unsigned long irqflags;

	switch (mode) {
	case CLOCK_EVT_MODE_ONESHOT:
		raw_local_irq_save(irqflags);
		OIER &= ~OST_C4;
		OSSR = OST_C4;
		raw_local_irq_restore(irqflags);
		break;

	case CLOCK_EVT_MODE_UNUSED:
	case CLOCK_EVT_MODE_SHUTDOWN:
		/* initializing, released, or preparing for suspend */
		raw_local_irq_save(irqflags);
		OIER &= ~OST_C4;
		OSSR = OST_C4;
		raw_local_irq_restore(irqflags);
		break;

	case CLOCK_EVT_MODE_RESUME:
	case CLOCK_EVT_MODE_PERIODIC:
		break;
	}
}

static struct clock_event_device ckevt_32ktimer = {
	.name		= "32k timer",
	.features	= CLOCK_EVT_FEAT_ONESHOT,
	.rating		= 100,
	.set_next_event	= pxa_timer_set_next_event,
	.set_mode	= pxa_timer_set_mode,
};

static struct clocksource cksrc_32ktimer = {
	.name		= "oscr4",
	.rating		= 100,
	.read		= pxa_timer_read,
	.mask		= CLOCKSOURCE_MASK(32),
	.flags		= CLOCK_SOURCE_IS_CONTINUOUS,
};

static struct irqaction pxa_timer_irq = {
	.name		= "os timer",
	.flags		= IRQF_DISABLED | IRQF_TIMER | IRQF_IRQPOLL | IRQF_SHARED,
	.handler	= pxa_timer_interrupt,
};

extern void rtc_calib_init(void);

static void __init pxa_timer_init(void)
{
	OIER = 0;
	OSSR = OST_C0 | OST_C1 | OST_C2 | OST_C3 | OST_C4 | OST_C10 | OST_C11;

	OMCR4 = 0xC1;		/* 32.768KHz */

	pxa_timer_irq.dev_id = &ckevt_32ktimer;

	OSMR4 = 0;
	OSCR4 = 1;		/* enable channel 4 interrupt */

	clocksource_calc_mult_shift(&cksrc_32ktimer, CLOCK_TICK_RATE, 4);
	clockevents_calc_mult_shift(&ckevt_32ktimer, CLOCK_TICK_RATE, 4);
	ckevt_32ktimer.max_delta_ns =
		clockevent_delta2ns(0x7fffffff, &ckevt_32ktimer);
	ckevt_32ktimer.min_delta_ns =
		clockevent_delta2ns(MIN_OSCR_DELTA, &ckevt_32ktimer);
	ckevt_32ktimer.cpumask = cpumask_of(0);


	clocksource_register(&cksrc_32ktimer);
	clockevents_register_device(&ckevt_32ktimer);
	setup_irq(IRQ_OST_4_11, &pxa_timer_irq);

	rtc_calib_init();
}

#ifdef CONFIG_PM
static unsigned long osmr[4], oier, oscr;

static void pxa_timer_suspend(void)
{
	osmr[0] = OSMR0;
	osmr[1] = OSMR1;
	osmr[2] = OSMR2;
	osmr[3] = OSMR3;
	oier = OIER;
	oscr = OSCR;
}

static void pxa_timer_resume(void)
{
	/*
	 * Ensure that we have at least MIN_OSCR_DELTA between match
	 * register 0 and the OSCR, to guarantee that we will receive
	 * the one-shot timer interrupt.  We adjust OSMR0 in preference
	 * to OSCR to guarantee that OSCR is monotonically incrementing.
	 */
	if (osmr[0] - oscr < MIN_OSCR_DELTA)
		osmr[0] += MIN_OSCR_DELTA;

	OSMR0 = osmr[0];
	OSMR1 = osmr[1];
	OSMR2 = osmr[2];
	OSMR3 = osmr[3];
	OIER = oier;
	OSCR = oscr;
}
#else
#define pxa_timer_suspend NULL
#define pxa_timer_resume NULL
#endif

struct sys_timer pxa_timer = {
	.init		= pxa_timer_init,
	.suspend	= pxa_timer_suspend,
	.resume		= pxa_timer_resume,
};
