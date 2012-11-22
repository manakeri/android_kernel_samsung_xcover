/*
 * PXA IPM IDLE
 *
 * Copyright (c) 2003 Intel Corporation.
 *
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html
 *
 * (C) Copyright 2006 Marvell International Ltd.
 * All Rights Reserved
 */

/*
#undef DEBUG
#define DEBUG
*/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/spinlock.h>

#include <mach/hardware.h>
#include <asm/proc-fns.h>
#include <asm/mach/time.h>
#include <mach/pxa95x-regs.h>
#include <mach/regs-intc.h>
#include <mach/regs-ost.h>
#include <mach/ipmc.h>
#include <mach/mspm_prof.h>
#ifdef CONFIG_ISPT
#include <mach/pxa_ispt.h>
#endif
#include <mach/pxa95x_pm.h>
#ifdef CONFIG_PXA_MIPSRAM
#include <mach/pxa_mips_ram.h>
#endif
#ifdef CONFIG_PXA95x_DVFM
#include <mach/dvfm.h>
#endif

#define MAX_OSCR0		0xFFFFFFFF

/* D0CS idle interval is 1ms */
#define D0CS_IDLE_INTERVAL      1
/* Low power mode idle interval is 2ms */
#define LOWPOWER_IDLE_INTERVAL  2

#define IDLE_STATS_NUM  1000


struct idle_stats {
	unsigned int index;
	unsigned int msec;
	unsigned int hr_data;	/* OSCR0, used in D0 or D0CS */
	unsigned int lr_data;	/* OSCR4, used in D0, D0CS, D1 or D2 */
	unsigned int icip;
	unsigned int icip2;
};

struct mspm_idle_stats {
	struct idle_stats stats[IDLE_STATS_NUM];
	unsigned int stats_index;
	spinlock_t lock;
};

static struct mspm_idle_stats mspm_stats = {
	.stats_index = 0,
	.lock = __SPIN_LOCK_UNLOCKED(mspm_stats.lock),
};

struct mspm_idle_prof {
	unsigned int start_oscr;
	unsigned int end_oscr;
	unsigned int oscr_idle_tick;
	unsigned int start_oscr4;
	unsigned int end_oscr4;
	unsigned int oscr4_idle_tick;
	unsigned int op;
	unsigned int jiffies;
};

static struct mspm_idle_prof prev_prof, cur_prof;


static int d0csidle;
static int idle_flaw;	/* silicon issue on IDLE */

static void (*orig_idle) (void);
static unsigned int cpuid;
static int d0csidx = -1, d1idx = -1, d2idx = -1, cgidx = -1;

#ifdef CONFIG_ISPT
#define ispt_power_state_c1() ispt_power_msg(CT_P_PWR_STATE_ENTRY_C1);
#define ispt_power_state_c0() ispt_power_msg(CT_P_PWR_STATE_ENTRY_C0);
#else
static int ispt_power_state_c1(void)
{
	return 0;
}

static int ispt_power_state_c0(void)
{
	return 0;
}
#endif


/* check whether current operating point is idle flaw operating point */
static int is_idle_flaw_op(void)
{
	if (idle_flaw) {
		if (ACSR & ACCR_D0CS_MASK)
			return 1;
		if (((ACSR & ACCR_XL_MASK) == 8)
		    && ((ACSR & ACCR_XN_MASK) == (1 << ACCR_XN_OFFSET))) {
			/* 104MHz */
			return 1;
		}
	}
	return 0;
}

extern int is_wkr_mg1_1274(void);
/*This parameter is used in order to analyze C2 crash.
This parameter measure C2 length see JIRA 1495
This parameter will be extraced through RAMPDUMP*/
unsigned int g_lastC2time;

static void pxa95x_cpu_idle(void)
{
	unsigned int icpr, icpr2, icmr, icmr2, iccr;
	unsigned int c1_enter_time, c1_exit_time, pollreg;
	struct op_info *info = NULL;
	int op;
	DVFMLPMGlobalCount.D0C1_Enter_count++;
	op = dvfm_get_op(&info);

#ifdef CONFIG_PXA95x_DVFM_STATS
	if (op < 0) {
		printk(KERN_ERR "Fatal: current OP is not in OP table!\n");
		return;
	}
	dvfm_add_event(op, CPU_STATE_RUN, op, CPU_STATE_IDLE);
	dvfm_add_timeslot(op, CPU_STATE_RUN);
#endif
	mspm_add_event(op, CPU_STATE_RUN);
	if (is_idle_flaw_op()) {
		/* Loop and query interrupt.
		 * At here, only IRQ is awared. FIQ is ignored.
		 */
		iccr = ICCR;
		while (1) {
			__asm__ __volatile__("\n\
				mrc p6, 0, %0, c4, c0, 0   @ Read out ICPR\n\
				mrc p6, 0, %1, c10, c0, 0  @ Read out ICPR2\n\
				mrc p6, 0, %2, c1, c0, 0   @ Read out ICMR\n\
				mrc p6, 0, %3, c7, c0, 0   @ Read out ICMR2\n"
				: "=&r"(icpr), "=&r"(icpr2), "=&r"(icmr), "=&r"(icmr2)
				:
				: "memory", "cc");
			if (iccr & 0x1) {
				if (((icpr & icmr) != 0) ||
						((icpr2 & icmr2) != 0))
					break;
			} else {
				if ((icpr != 0) || (icpr2 != 0))
					break;
			}
		}
	} else {
		ispt_power_state_c1();
#ifdef CONFIG_PXA_MIPSRAM
		c1_enter_time = OSCR4;
		MIPS_RAM_ADD_32K_TIME_STAMP(c1_enter_time);
		MIPS_RAM_ADD_PM_TRACE(ENTER_IDLE_MIPS_RAM);
		mipsram_disable_counter();
#endif
		if (cpu_is_pxa95x() && !(is_wkr_mg1_1274())) {
			PWRMODE = (PXA95x_PM_S0D0C1 | PXA95x_PM_I_Q_BIT);
			do {
				pollreg = PWRMODE;
			} while (pollreg !=
					(PXA95x_PM_S0D0C1 | PXA95x_PM_I_Q_BIT));
			/*D0CS will wait at C2 exit as well
			This is OK as D0CS is defeatured soon*/
			g_lastC2time = pm_enter_deepidle(CPU_PDWN_3_25M_CYCLES);
		} else {
		cpu_do_idle();
		}
#ifdef CONFIG_PXA_MIPSRAM
		mipsram_reinit_counter();
		MIPS_RAM_ADD_PM_TRACE(EXIT_IDLE_MIPS_RAM);
		c1_exit_time = OSCR4;
		MIPS_RAM_ADD_32K_TIME_STAMP(c1_exit_time);
#endif
		ispt_power_state_c0();
	}
#ifdef CONFIG_PXA95x_DVFM_STATS
	dvfm_add_event(op, CPU_STATE_IDLE, op, CPU_STATE_RUN);
	dvfm_add_timeslot(op, CPU_STATE_IDLE);
#endif
	mspm_add_event(op, CPU_STATE_IDLE);
}

static void mspm_d0csidle(void)
{
	struct dvfm_freqs freqs;
	struct dvfm_md_opt *op;
	struct op_info *info = NULL;
	int ret, prev_op;

	ret = dvfm_get_op(&info);
	if (info == NULL)
		return;
	op = (struct dvfm_md_opt *) info->op;
	if (op == NULL)
		return;
	pm_core_pwdn(CPU_PDWN_D0CS_ENTRY);
	if ((ret >= 0) && (op->power_mode == POWER_MODE_D0)) {
		memset(&freqs, 0, sizeof(struct dvfm_freqs));
		prev_op = ret;
		freqs.old = ret;
		if (d0csidle) {
			freqs.new = d0csidx;
			/* If RELATION_LOW is used, DVFM will choose the valid
			 * lowest operating point.
			 * If RELATION_STICK is used, DVFM will only try the
			 * specified operating point.
			 */
			ret = dvfm_set_op(&freqs, freqs.new, RELATION_STICK);
		}
		pxa95x_cpu_idle();
		if (!ret) {
			memset(&freqs, 0, sizeof(struct dvfm_freqs));
			if (d0csidle)
				freqs.old = d0csidx;
			freqs.new = prev_op;
			ret = dvfm_set_op(&freqs, freqs.new, RELATION_LOW);
		} else {
			pr_debug("%s, ret=%d, current op:%d\n",
				 __func__, ret, prev_op);
		}
	} else
		pxa95x_cpu_idle();
	pm_core_pwdn(CPU_PDWN_D0CS_EXIT);
}

static int beyond_d0cs_tick(unsigned int msec)
{
	if (d0csidx == -1)
		return 0;

	if (msec >= D0CS_IDLE_INTERVAL)
		return 1;

	return 0;
}

extern int suspend_forbidden;
static int lpidle_is_valid(int enable, unsigned int msec,
			   struct dvfm_freqs *freqs, int lp_idle)
{
	struct op_info *info = NULL;
	struct dvfm_md_opt *op;
	int prev_op;
	int ret;


	if ((freqs == NULL) || (lp_idle == IDLE_D2 && d2idx == -1)
	    || (lp_idle == IDLE_D1 && d1idx == -1)
	    || (lp_idle == IDLE_CG && cgidx == -1))
		return 0;

	if (enable & lp_idle) {
		/* Check dynamic tick flag && idle interval 
		 * suspend_forbidden is 1 when using sysleep */
		if ((msec < LOWPOWER_IDLE_INTERVAL) && (suspend_forbidden == 0))
			return 0;
		/* Check whether the specified low power mode is valid */
		ret = dvfm_get_op(&info);
		if (info == NULL)
			return 0;
		op = (struct dvfm_md_opt *) info->op;
		if (op == NULL)
			return 0;
		if ((ret >= 0) && ((op->power_mode == POWER_MODE_D0)
				   || (op->power_mode == POWER_MODE_D0CS))) {
			prev_op = ret;
			freqs->old = ret;

			freqs->new = lp_idle == IDLE_D2 ? d2idx :
			    lp_idle == IDLE_D1 ? d1idx :
			    lp_idle == IDLE_CG ? cgidx : 0xFFFFFFFF;
			if (freqs->new == 0xFFFFFFFF)
				return 0;
			ret = dvfm_get_opinfo(freqs->new, &info);
			if ((ret >= 0) && (info->device == 0)) {
				return 1;
			}
		}
	}
	return 0;
}

/* Collect statistic information before entering idle */
static void record_idle_stats(unsigned int msec)
{
	int i;

	spin_lock(&mspm_stats.lock);
	if (++mspm_stats.stats_index == IDLE_STATS_NUM)
		mspm_stats.stats_index = 0;
	i = mspm_stats.stats_index;
	memset(&mspm_stats.stats[i], 0, sizeof(struct idle_stats));
	mspm_stats.stats[i].msec = msec;
	mspm_stats.stats[i].index = i;
	/* Record current OSCR */
	mspm_stats.stats[i].hr_data = OSCR;
	mspm_stats.stats[i].lr_data = OSCR4;
	spin_unlock(&mspm_stats.lock);
}

/* Collect statistic information after exiting idle.
 * Return OSCR interval
 */
static int update_idle_stats(void)
{
	unsigned int oscr, tmp;
	struct op_info *info = NULL;
	int i, op;

	op = dvfm_get_op(&info);
	spin_lock(&mspm_stats.lock);
	i = mspm_stats.stats_index;

	if (op != cur_prof.op) {
		/* If it's a new OP, refresh cur_prof */
		memcpy(&prev_prof, &cur_prof,
		       sizeof(struct mspm_idle_prof));
		memset(&cur_prof, 0, sizeof(struct mspm_idle_prof));
		cur_prof.op = op;
		cur_prof.start_oscr = mspm_stats.stats[i].hr_data;
		cur_prof.start_oscr4 = mspm_stats.stats[i].lr_data;
		cur_prof.jiffies = jiffies;
	}

	tmp = mspm_stats.stats[i].hr_data;
	oscr = OSCR;
	if (oscr > tmp)
		tmp = oscr - tmp;
	else
		tmp = MAX_OSCR0 - oscr + tmp;
	/* update the OSCR0 interval into hr_data field */
	mspm_stats.stats[i].hr_data = tmp;

	tmp = mspm_stats.stats[i].lr_data;
	oscr = OSCR4;
	if (oscr > tmp)
		tmp = oscr - tmp;
	else
		tmp = MAX_OSCR0 - oscr + tmp;
	/* update the OSCR4 interval into lr_data field */
	mspm_stats.stats[i].lr_data = tmp;

	mspm_stats.stats[i].icip = ICIP;
	mspm_stats.stats[i].icip2 = ICIP2;

	cur_prof.oscr_idle_tick += mspm_stats.stats[i].hr_data;
	cur_prof.oscr4_idle_tick += mspm_stats.stats[i].lr_data;
	cur_prof.end_oscr = OSCR;
	cur_prof.end_oscr4 = OSCR4;

	spin_unlock(&mspm_stats.lock);

	return mspm_stats.stats[i].lr_data;
}

static unsigned int pxa_ticks_to_msec(unsigned int ticks)
{
#ifdef CONFIG_PXA_32KTIMER
	return (ticks * 5 * 5 * 5) >> 12;
#else
	return ticks / 3250;
#endif
}

/*
 * IDLE Thread
 */
void mspm_do_idle(void)
{
	struct dvfm_freqs freqs;
	int ret = -EINVAL;
	unsigned int msec, ticks;
	int delta;
	if (ForceC0) {
		return;
	}
	local_irq_disable();

	/* value of current in this context? */
	if (!need_resched() && !hlt_counter) {
#ifdef CONFIG_NO_IDLE_HZ
		timer_dyn_reprogram();
#endif
#ifdef CONFIG_PXA_32KTIMER
		ticks = (OSMR4 > OSCR4) ? OSMR4 - OSCR4
		    : (0xFFFFFFFF - OSCR4 + OSMR4);
#else
		ticks = (OSMR0 > OSCR) ? OSMR0 - OSCR
		    : (0xFFFFFFFF - OSCR + OSMR0);
#endif
		msec = pxa_ticks_to_msec(ticks);
		record_idle_stats(msec);

		if (lpidle_is_valid(enable_deepidle, msec, &freqs, IDLE_D2)) {
			delta = dvfm_is_comm_wakep_near();
			/* checking if comm wakeup is to close and forbid D2
			 * if so.
			 */
			if (delta) {
				/* going to c1 instead... */
				pxa95x_cpu_idle();
				goto out;
			} else {
				/* this means that we are far
				 * enough and can enter D2.
				 * start D2 entry sequence.
				 */
				ret = dvfm_set_op(&freqs, freqs.new,
						  RELATION_STICK);
				if (ret == 0)
					goto out;
			}
		}
		if (lpidle_is_valid(enable_deepidle, msec, &freqs, IDLE_D1)) {
			delta = dvfm_is_comm_wakep_near();
			/* checking whether comm wakeup is too close
			 * and forbid D1 if so.
			 */
			if (delta) {
				/* going to c1 instead... */
				pxa95x_cpu_idle();
				goto out;
			} else {
				/* this means that we are far
				 * enough and can enter D2.
				 * start D1 entry sequence.
				 */
				ret = dvfm_set_op(&freqs, freqs.new,
						  RELATION_STICK);
				if (ret == 0)
					goto out;
			}
		}

		if (lpidle_is_valid(enable_deepidle, msec, &freqs, IDLE_CG)) {
			ret = dvfm_set_op(&freqs, freqs.new, RELATION_STICK);
			if (ret == 0)
				goto out;
		}

		if ((enable_deepidle & IDLE_D0CS)
		    && beyond_d0cs_tick(msec))
			mspm_d0csidle();
		else
			pxa95x_cpu_idle();
out:
		update_idle_stats();
	}

	local_irq_enable();
}

static struct proc_dir_entry *entry_dir;
static struct proc_dir_entry *entry_stats;

static int stats_show(struct seq_file *s, void *v)
{
	struct idle_stats *p = NULL;
	int i, ui;
	unsigned long flags;

	spin_lock_irqsave(&mspm_stats.lock, flags);

	ui = mspm_stats.stats_index;
	for (i = 0; i < IDLE_STATS_NUM; i++) {
		p = &mspm_stats.stats[ui++];
		seq_printf(s, "INDEX:%d MSECS:%u usec:%u "
			   "OSCR4DATA:%u ICIP:0x%x ICIP2:0x%x\n", p->index,
			   p->msec, p->hr_data * 4 / 13, p->lr_data,
			   p->icip, p->icip2);
		switch (p->icip) {
		case 0x4000000:
			seq_printf(s, ",OST0\n");
			break;
		case 0x400000:
			seq_printf(s, ",UART1\n");
			break;
		case 0x40000:
			seq_printf(s, ",I2C\n");
			break;
		case 0x400:
			seq_printf(s, ",GPIO_x\n");
			break;
		case 0x80:
			seq_printf(s, ",OST4\n");
			break;
		}
		if (ui == IDLE_STATS_NUM)
			ui = 0;
	}

	spin_unlock_irqrestore(&mspm_stats.lock, flags);

	seq_printf(s, "ICMR:0x%x, ICMR:0x%x\n", ICMR, ICMR2);
	/* set interrupt mask */
	return 0;
}

static int stats_seq_open(struct inode *inode, struct file *file)
{
	return single_open(file, &stats_show, NULL);
}

static const struct file_operations stats_seq_ops = {
	.owner = THIS_MODULE,
	.open = stats_seq_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int mspm_proc_init(void)
{
	entry_dir = proc_mkdir("driver/mspm", NULL);
	if (entry_dir == NULL) {
		return -ENOMEM;
	}

	entry_stats = create_proc_entry("stats", 0, entry_dir);
	if (entry_stats)
		entry_stats->proc_fops = &stats_seq_ops;
	return 0;
}

static void mspm_proc_cleanup(void)
{
	remove_proc_entry("stats", entry_dir);
	remove_proc_entry("driver/mspm", NULL);
}

int mspm_idle_load(void)
{
	orig_idle = pm_idle;
	pm_idle = mspm_do_idle;
	return 0;
}

void mspm_idle_clean(void)
{
	if (orig_idle != NULL)
		pm_idle = orig_idle;
	else
		/* in case mspm_idle_clean is called before mspm_idle_load */
		orig_idle = pm_idle;
}

static void query_idle_flaw(void)
{
	idle_flaw = 0;
	if ((cpuid >= 0x6880) && (cpuid <= 0x6881))
		idle_flaw = 1;	/* PXA300 A0/A1 */
	else if ((cpuid >= 0x6890) && (cpuid <= 0x6892))
		idle_flaw = 1;	/* PXA310 A0/A1/A2 */
}

static void query_d0csidle(void)
{
#ifdef CONFIG_FB_PXA_LCD_VGA
	d0csidle = 0;
#else
	d0csidle = 1;
#endif
	/* PXA310 A2 */
	if (cpuid == 0x6892)
		d0csidle = 1;
}

void set_idle_op(int idx, int mode)
{
	switch (mode) {
	case POWER_MODE_D0CS:
		d0csidx = idx;
		break;
	case POWER_MODE_D1:
		d1idx = idx;
		break;
	case POWER_MODE_D2:
		d2idx = idx;
		break;
	case POWER_MODE_CG:
		cgidx = idx;
		break;
	}
}

static int __init mspm_init(void)
{
	/* check idle flaw */
	cpuid = read_cpuid(0) & 0xFFFF;
	query_idle_flaw();
	query_d0csidle();

	/* Create file in procfs */
	if (mspm_proc_init()) {
		mspm_idle_clean();
		return -EFAULT;
	}

	mspm_prof_init();

	/* clear data in prev_prof & cur_prof */
	memset(&prev_prof, 0, sizeof(struct mspm_idle_prof));
	memset(&cur_prof, 0, sizeof(struct mspm_idle_prof));
	prev_prof.op = -1;
	cur_prof.op = -1;

	pr_info("Initialize IPM.\n");

	return 0;
}

static void __exit mspm_exit(void)
{
	/* Remove procfs */
	mspm_proc_cleanup();

	mspm_prof_exit();

	pr_info("Quit IPM\n");
}

module_init(mspm_init);
module_exit(mspm_exit);

MODULE_DESCRIPTION("IPM");
MODULE_LICENSE("GPL");
