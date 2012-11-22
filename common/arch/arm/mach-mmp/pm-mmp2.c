/*
 * MMP2 Power Management Driver
 *
 * Copyright (c) 2010 Marvell International Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License.
 */

#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/suspend.h>
#include <linux/errno.h>
#include <mach/irqs.h>
#include <mach/regs-pmu.h>

enum {
	MMP2_PM_MODE_ACTIVE = 0,
	MMP2_PM_MODE_CORE_IDLE,
	MMP2_PM_MODE_APPS_IDLE,
	MMP2_PM_MODE_APPS_SLEEP,
	MMP2_PM_MODE_SYS_SLEEP,
	MMP2_PM_MODE_INVALID,
};

int mmp2_set_wake(unsigned int irq, unsigned int on)
{
	struct irq_desc *desc = irq_to_desc(irq);
	unsigned long flags, data = 0;

	if (unlikely(irq >= nr_irqs)) {
		pr_err("IRQ nubmers are out of boundary!\n");
		return -EINVAL;
	}

	raw_spin_lock_irqsave(&desc->lock, flags);
	if (on) {
		if (desc->action)
			desc->action->flags |= IRQF_NO_SUSPEND;
	} else {
		if (desc->action)
			desc->action->flags &= ~IRQF_NO_SUSPEND;
	}
	raw_spin_unlock_irqrestore(&desc->lock, flags);

	/* enable wakeup sources */
	switch (irq) {
	case IRQ_MMP2_TBALL:
		data = MMP2_WAKEUP3 | MMP2_WAKEUP_TBALL;
		break;
	case IRQ_MMP2_KPC:
		data = MMP2_WAKEUP3 | MMP2_WAKEUP_KPC;
		break;
	case IRQ_MMP2_ROTARY:
		data = MMP2_WAKEUP3 | MMP2_WAKEUP_ROTARY;
		break;
	case IRQ_MMP2_TIMER1:
		data = MMP2_WAKEUP4 | MMP2_WAKEUP_TIMER1G1;
		break;
	case IRQ_MMP2_TIMER2:
		data = MMP2_WAKEUP4 | MMP2_WAKEUP_TIMER1G2;
		break;
	case IRQ_MMP2_TIMER3:
		data = MMP2_WAKEUP4 | MMP2_WAKEUP_TIMER1G3;
		break;
	case IRQ_MMP2_PMU_TIMER1:
		data = MMP2_WAKEUP4 | MMP2_WAKEUP_TIMER2G1;
		break;
	case IRQ_MMP2_PMU_TIMER2:
		data = MMP2_WAKEUP4 | MMP2_WAKEUP_TIMER2G2;
		break;
	case IRQ_MMP2_PMU_TIMER3:
		data = MMP2_WAKEUP4 | MMP2_WAKEUP_TIMER2G3;
		break;
	case IRQ_MMP2_WDT1:
		data = MMP2_WAKEUP4 | MMP2_WAKEUP_WDT1;
		break;
	case IRQ_MMP2_WDT2:
		data = MMP2_WAKEUP4 | MMP2_WAKEUP_WDT2;
		break;
	case IRQ_MMP2_RTC:
		data = MMP2_WAKEUP4 | MMP2_WAKEUP_RTC;
		break;
	case IRQ_MMP2_MMC:
		data = MMP2_WAKEUP6 | MMP2_WAKEUP_SDH1;
		break;
	case IRQ_MMP2_MMC3:
		data = MMP2_WAKEUP6 | MMP2_WAKEUP_SDH3;
		break;
	case IRQ_MMP2_MSP:
		data = MMP2_WAKEUP6 | MMP2_WAKEUP_MSP;
		break;
	case IRQ_MMP2_PMIC:
		data = MMP2_WAKEUP7;
		break;
	}
	if (on) {
		if (data) {
			data |= __raw_readl(MMP2_PMUM_WUCRM_PJ);
			__raw_writel(data, MMP2_PMUM_WUCRM_PJ);
		}
	} else {
		if (data) {
			data = ~data & __raw_readl(MMP2_PMUM_WUCRM_PJ);
			__raw_writel(data, MMP2_PMUM_WUCRM_PJ);
		}
	}
	return 0;
}

static int mmp2_pm_valid(suspend_state_t state)
{
	int ret = 0;

	switch (state) {
	case PM_SUSPEND_STANDBY:
	case PM_SUSPEND_MEM:
		ret = 1;
	}
	return ret;
}

static int mmp2_pm_enter(suspend_state_t state)
{
	unsigned int data, idle, pcr_sp, pcr_pj;
	int mode = MMP2_PM_MODE_SYS_SLEEP;

	pcr_sp = __raw_readl(MMP2_PMUM_PCR_SP);
	pcr_pj = __raw_readl(MMP2_PMUM_PCR_PJ);
	/* configure power control register of Secure Processor */
	data = ~0x1ff;
	switch (mode) {
	case MMP2_PM_MODE_SYS_SLEEP:
		data &= ~MMP2_PCR_SLPWPALL;
		__raw_writel(data, MMP2_PMUM_PCR_SP);
		break;
	case MMP2_PM_MODE_APPS_SLEEP:
		data &= ~(MMP2_PCR_SLPEN | MMP2_PCR_VCXOSD);
		__raw_writel(data, MMP2_PMUM_PCR_SP);
		break;
	}
	/* configure power control register of PJ4 */
	data = __raw_readl(MMP2_PMUM_PCR_PJ);
	idle = __raw_readl(MMP2_PMUA_PJ_IDLE_CFG);
	switch (mode) {
	case MMP2_PM_MODE_SYS_SLEEP:
		data |= MMP2_PCR_SLPEN | MMP2_PCR_VCXOSD;
		data &= ~MMP2_PCR_SLPWPALL;
	case MMP2_PM_MODE_APPS_SLEEP:
		data |= MMP2_PCR_APBSD;
	case MMP2_PM_MODE_APPS_IDLE:
		data |= MMP2_PCR_AXISD | MMP2_PCR_DDRCORSD;
		__raw_writel(data, MMP2_PMUM_PCR_PJ);
		break;
	case MMP2_PM_MODE_CORE_IDLE:
		idle |= MMP2_IDLE_PJ_IDLE;
		idle &= ~MMP2_IDLE_PJ_ISO;
		idle |= (0xa << 16);
		break;
	}
	idle |= MMP2_IDLE_PJ_MC_WAKE | MMP2_IDLE_PJ_NO_MCREQ;
	idle |= MMP2_IDLE_PJ_DWN;
	__raw_writel(idle, MMP2_PMUA_PJ_IDLE_CFG);
	/* set memory controller hardware sleep as self-refresh power down */
	__raw_writel(MMP2_MC_HWSLP_SELFREF, MMP2_PMUA_MC_HW_SLP);

	/* enter suspend/standby */
	pm_idle();

	/* finish */
	__raw_writel(pcr_sp, MMP2_PMUM_PCR_SP);
	__raw_writel(pcr_pj, MMP2_PMUM_PCR_PJ);
	/* enable VCXO */
	__raw_writel(0x1, MMP2_PMUM_VRCR);
	return 0;
}

static struct platform_suspend_ops mmp2_pm_ops = {
	.valid		= mmp2_pm_valid,
	.enter		= mmp2_pm_enter,
};

static int __init mmp2_pm_init(void)
{
	unsigned int data;

	suspend_set_ops(&mmp2_pm_ops);

	/* set default low power control bit, only VCXO shutdown is allowed */
	data = ~0x1ff;
	data &= ~(MMP2_PCR_SLPWP7 | MMP2_PCR_SLPWP6 | MMP2_PCR_SLPWP5
		| MMP2_PCR_SLPWP4 | MMP2_PCR_SLPWP3 | MMP2_PCR_SLPWP2
		| MMP2_PCR_SLPWP1 | MMP2_PCR_SLPWP0 | MMP2_PCR_SLPEN
		| MMP2_PCR_DDRCORSD | MMP2_PCR_APBSD | MMP2_PCR_AXISD);
	__raw_writel(data, MMP2_PMUM_PCR_PJ);

	/* use external 32KHz cock (bit 1), reserved bits should be written 1 */
	__raw_writel(~0x0, MMP2_PMUM_SCCR);
	return 0;
}
late_initcall(mmp2_pm_init);
