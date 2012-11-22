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

#define OSCR2NS_SCALE_FACTOR 10

static unsigned long oscr2ns_scale;

static void __init set_oscr2ns_scale(unsigned long oscr_rate)
{
	unsigned long long v = 1000000000ULL << OSCR2NS_SCALE_FACTOR;
	do_div(v, oscr_rate);
	oscr2ns_scale = v;
	/*
	 * We want an even value to automatically clear the top bit
	 * returned by cnt32_to_63() without an additional run time
	 * instruction. So if the LSB is 1 then round it up.
	 */
	if (oscr2ns_scale & 1)
		oscr2ns_scale++;
}

unsigned long long sched_clock(void)
{
	unsigned long long v = cnt32_to_63(OSCR);
	return (v * oscr2ns_scale) >> OSCR2NS_SCALE_FACTOR;
}


#define MIN_OSCR_DELTA 16

static irqreturn_t
pxa_ost0_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *c = dev_id;
	if (!(OSSR & OST_C0))
		return IRQ_NONE;

	/* Disarm the compare/match, signal the event. */
	OIER &= ~OIER_E0;
	OSSR = OSSR_M0;
	c->event_handler(c);

	return IRQ_HANDLED;
}

static int
pxa_osmr0_set_next_event(unsigned long delta, struct clock_event_device *dev)
{
	unsigned long next, oscr, irqflags;
	raw_local_irq_save(irqflags);
	OIER |= OIER_E0;
	next = OSCR + delta;
	OSMR0 = next;
	oscr = OSCR;
	raw_local_irq_restore(irqflags);

	return (signed)(next - oscr) <= MIN_OSCR_DELTA ? -ETIME : 0;
}

static void
pxa_osmr0_set_mode(enum clock_event_mode mode, struct clock_event_device *dev)
{
	unsigned long irqflags;

	switch (mode) {
	case CLOCK_EVT_MODE_ONESHOT:
		raw_local_irq_save(irqflags);
		OIER &= ~OIER_E0;
		OSSR = OSSR_M0;
		raw_local_irq_restore(irqflags);
		break;

	case CLOCK_EVT_MODE_UNUSED:
	case CLOCK_EVT_MODE_SHUTDOWN:
		/* initializing, released, or preparing for suspend */
		raw_local_irq_save(irqflags);
		OIER &= ~OIER_E0;
		OSSR = OSSR_M0;
		raw_local_irq_restore(irqflags);
		break;

	case CLOCK_EVT_MODE_RESUME:
	case CLOCK_EVT_MODE_PERIODIC:
		break;
	}
}

static struct clock_event_device ckevt_pxa_osmr0 = {
	.name		= "osmr0",
	.features	= CLOCK_EVT_FEAT_ONESHOT,
	.rating		= 200,
	.set_next_event	= pxa_osmr0_set_next_event,
	.set_mode	= pxa_osmr0_set_mode,
};

static cycle_t pxa_read_oscr(struct clocksource *cs)
{
	return OSCR;
}

static struct clocksource cksrc_pxa_oscr0 = {
	.name           = "oscr0",
	.rating		= 200,
	.read           = pxa_read_oscr,
	.mask           = CLOCKSOURCE_MASK(32),
	.flags		= CLOCK_SOURCE_IS_CONTINUOUS,
};

static struct irqaction pxa_ost0_irq = {
	.name		= "ost0",
	.flags		= IRQF_DISABLED | IRQF_TIMER | IRQF_IRQPOLL,
	.handler	= pxa_ost0_interrupt,
	.dev_id		= &ckevt_pxa_osmr0,
};

extern void rtc_calib_init(void);

static void __init pxa_timer_init(void)
{
	unsigned long clock_tick_rate = get_clock_tick_rate();

	OIER = 0;
	OSSR = OSSR_M0 | OSSR_M1 | OSSR_M2 | OSSR_M3;

	set_oscr2ns_scale(clock_tick_rate);

	clocksource_calc_mult_shift(&cksrc_pxa_oscr0, CLOCK_TICK_RATE, 4);
	clockevents_calc_mult_shift(&ckevt_pxa_osmr0, CLOCK_TICK_RATE, 4);
	ckevt_pxa_osmr0.max_delta_ns =
		clockevent_delta2ns(0x7fffffff, &ckevt_pxa_osmr0);
	ckevt_pxa_osmr0.min_delta_ns =
		clockevent_delta2ns(MIN_OSCR_DELTA * 2, &ckevt_pxa_osmr0) + 1;
	ckevt_pxa_osmr0.cpumask = cpumask_of(0);


	clocksource_register(&cksrc_pxa_oscr0);
	clockevents_register_device(&ckevt_pxa_osmr0);
	setup_irq(IRQ_OST0, &pxa_ost0_irq);

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
