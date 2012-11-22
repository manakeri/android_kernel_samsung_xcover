/*
 *  linux/arch/arm/mach-pxa/rtc-calibrate.c
 *
 *  Support rtc calibrate for the Marvell PXA platforms
 *
 *  Copyright (C) 2007-2010 Marvell International Ltd.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  publishhed by the Free Software Foundation.
 */


#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>

#include <mach/regs-ost.h>
#include <mach/regs-rtc.h>


/* Use OST channel 11 and 10 to calibrate RTC:
 * RTC will use 32K clock. 32K clock is provided by VCTCXO, which
 * will become inaccurate with age. In order to ensure RTC accurate,
 * 1MHZ clock(fast clock) is connected to OST channel 10 and 32K clock
 * (slow clock) is connected to the OST channel 11. OST channel 10
 * and channel 11 will be started at same time. When OST channel 11
 * stops at 1s match event occurs, the read the counts of channel 10
 * and channel 11 at same time. By comparing the count of channel 11
 * with its theoretic value, the deviation of 32K clock can be calculated,
 * so the RTC can be calibrated by adjusting its divisor.
 */
#define FAST_TIMER_1SEC_VALUE	1000000
#define RTTR_DEFAULT_DIV_VALUE	0x00007FFF
#define FAST_TIMER_START_VALUE  1000000
#define ONE_SEC_TIME_INTERVAL	32769

static struct completion rtc_calib_complete;
int rtc_calib(void)
{
	int ret;

	/* disable OS timers IRQ (10 & 11) */
	OIER &= ~OST_C10;
	OIER &= ~OST_C11;

	/* run 32 Khz timer */
	OMCR11 = (1 << 7) | (1 << 0) | (1 << 9);

	/* run 1 Mhz timer */
	OMCR10 = (1 << 7) | (4 << 0);

	/* configure 32Khz timer */
	OSCR11 = 1;
	OSMR11 = ONE_SEC_TIME_INTERVAL;

	/* configure OS timer to 1 Mhz with 10 sec interval */
	OSCR10 = FAST_TIMER_START_VALUE;

	/* Enable OSCR11 match interrupt for slow timer */
	OIER |= OST_C11;

	/* wait for RTC calibration done */
	ret = wait_for_completion_timeout(&rtc_calib_complete, 2*HZ);

	if (!ret) {
		INIT_COMPLETION(rtc_calib_complete);
		printk(KERN_ERR "%s: rtc calibration failed\n", __func__);
		return -EIO;
	} else {
		return 0;
	}
}
EXPORT_SYMBOL(rtc_calib);

static void set_rtc_rttr(int count)
{
	int divisor = 0;

	/* clear the lock bit */
	RTTR |= 0x7FFFFFFF;

	/* calculate delta time */
	divisor = FAST_TIMER_1SEC_VALUE - count;
	pr_debug("- %s:delta count:%d\n", __func__, divisor);

	/* convert fast clock ticks to slow ticks */
	divisor = divisor*32*1024/count;
	pr_debug("- %s:delta divisor:%d\n", __func__, divisor);

	/* update CLK_DIV */
	divisor = RTTR_DEFAULT_DIV_VALUE + divisor;

	RTTR &= ~0x0000FFFF;
	RTTR |= divisor;
}

static irqreturn_t
rtc_calib_interrupt(int irq, void *dev_id)
{
	unsigned int oscr11;

	if (!(OSSR & OST_C11))
		return IRQ_NONE;

	/* clear interrupt status */
	OSSR = OST_C11;

	/* OSC11 will be read out and OSCR10 will read into OSNR simutaneously */
	oscr11 = OSCR11;

	/* Disable OST channel 10 (fast clock) */
	OIER &= ~OST_C10;
	OMCR10 = 0;

	/* stop channel 11 (slow clock) */
	OIER &= ~OST_C11;
	OMCR11 = 0;

	/* calibrate the RTC */
	set_rtc_rttr(OSNR - FAST_TIMER_START_VALUE);

	/* notify rtc calibration is done */
	complete(&rtc_calib_complete);

	return IRQ_HANDLED;
}

void rtc_calib_init(void)
{
	int ret;

	init_completion(&rtc_calib_complete);

	ret = request_irq(IRQ_OST_4_11, rtc_calib_interrupt, IRQF_SHARED | IRQF_TIMER,
			  "rtc calibrate", &rtc_calib_complete);
	if (ret)
		printk(KERN_ERR "rtc calibrate: couldn't get irq\n");
}
