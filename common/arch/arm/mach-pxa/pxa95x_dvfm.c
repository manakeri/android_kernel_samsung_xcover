/*
 * arch/arm/mach-pxa/pxa95x_dvfm.c
 *
 * PXA95x DVFM Driver
 *
 * Copyright (C) 2010 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#undef DEBUG

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/sysdev.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/list.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/string.h>
#include <asm/uaccess.h>
#include <mach/regs-intc.h>
#include <mach/regs-ost.h>
#include <mach/pxa95x-regs.h>
#include <mach/hardware.h>
#include <mach/dvfm.h>
#include <mach/pxa95x_pm.h>
#include <mach/pxa95x_dvfm.h>
#include <mach/debug_pm.h>
#include <asm/io.h>
#include <linux/pm.h>
#ifdef CONFIG_ISPT
#include <mach/pxa_ispt.h>
#endif
#include <mach/mspm_prof.h>
#include <linux/regulator/consumer.h>
#if defined(CONFIG_PXA9XX_ACIPC)
#include <plat/pxa9xx_acipc.h>
#endif
#ifdef CONFIG_PXA_MIPSRAM
#include <mach/pxa_mips_ram.h>
#endif
#ifdef CONFIG_PMIC_D1980
#include <linux/d1982/pmic.h>
#endif
#include <plat/pxa3xx_onenand.h>
#include "devices.h"
#include "generic.h"
#include <linux/mfd/88pm860x.h>

#include <mach/pxa9xx_pm_logger.h> /* for pm debug tracing */

#ifdef CONFIG_PMIC_D1980
#define AVCR         				 (volatile unsigned long *)(0x40f50094)
#define AVCR_REG                    __REG(0x40f50094)
#endif

#ifdef CONFIG_PXA95x_SUSPEND
extern int suspend_forbidden;
#include <linux/wakelock.h>
#endif


/* Enter D2 before exiting D0CS */
#define DVFM_LP_SAFE
int ForceOP, ForcedOPIndex, ForceC0, ForceVCTCXO_EN, EnableD2VoltageChange = 1;
static struct dvfm_md_opt *lowest_freq_op;
static int lowest_freq_index;
/* setting the default voltage level to 1.05V */
unsigned int D2voltageLevelValue = 0x0D;
extern struct info_head dvfm_trace_list;
#ifdef DVFM_LP_SAFE
static struct work_struct postpone_work;
#endif
extern int enable_deepidle;
extern void wifi_set_power(unsigned int on);
extern int suspend_state(unsigned int * state);
extern void mmc_froce_unlock(void);
extern void JIG_froce_unlock(void);
#ifdef CONFIG_PMIC_D1980
extern int d1980_read_dvfm_profile(void);
#endif
/* Counter Structure for Debugging ENTER/EXIT D2/CGM/D0CS */
pxa95x_DVFM_LPM_Global_Count DVFMLPMGlobalCount = { 0, 0, 0, 0 };

/*Offset for "zero" the data */
pxa95x_DVFM_LPM_Global_Count DVFMLPMGlobalCountOffset = { 0, 0, 0, 0 };

struct pxa95x_dvfm_info {
	/* flags */
	uint32_t flags;

	/* CPU ID */
	uint32_t cpuid;
#if 0
	/* LCD clock */
	struct clk *lcd_clk;

	/* HSIO2 clock */
	struct clk *hsio2_clk;
#endif
	/* clock manager register base */
	unsigned char __iomem *clkmgr_base;

	/* service power management unit */
	unsigned char __iomem *spmu_base;

	/* slave power management unit */
	unsigned char __iomem *bpmu_base;

	/* dynamic memory controller register base */
	unsigned char __iomem *dmc_base;

	/* static memory controller register base */
	unsigned char __iomem *smc_base;
};

#define MIN_SAFE_FREQUENCY	624

struct info_head pxa95x_dvfm_op_list = {
	.list = LIST_HEAD_INIT(pxa95x_dvfm_op_list.list),
	.lock = __RW_LOCK_UNLOCKED(pxa95x_dvfm_op_list.lock),
};

#ifdef CONFIG_PXA95x_DVFM_STATS

static unsigned int switch_lowpower_before, switch_lowpower_after;

static int pxa95x_stats_notifier_freq(struct notifier_block *nb,
				      unsigned long val, void *data);
static struct notifier_block notifier_freq_block = {
	.notifier_call = pxa95x_stats_notifier_freq,
};
#endif
static int disabe_high_pp_on_low_voltage_board(void);

/* the operating point preferred by policy maker or user */
static int preferred_op;

extern unsigned int cur_op;	/* current operating point */
extern unsigned int def_op;	/* default operating point */

#ifdef CONFIG_PXA_MIPSRAM
int update_op_mips_ram(u32 old_pp, u32 new_pp);
#endif
static unsigned int pxa95x_ticks_to_msec(unsigned int ticks);
extern int enter_d0cs_a(unsigned char __iomem *, unsigned char __iomem *,
		unsigned char __iomem *);
extern int exit_d0cs_a(unsigned char __iomem *, unsigned char __iomem *,
		unsigned char __iomem *);
extern int md2fvinfo(struct pxa95x_fv_info *, struct dvfm_md_opt *);
extern void set_idle_op(int, int);
extern u32 set_DDR_avail_flag(void);
extern u32 clear_DDR_avail_flag(void);
extern void acipc_set_constraint_no_op_change(void);
extern void acipc_disable_d0cs_no_change(void);
extern void acipc_disable_d0cs(void);
extern u32 get_ddr_avail_state(void);
extern int is_wkr_mg1_1274(void);

/*1GHz support - voltage for 1GHz Frequency PP*/
static unsigned long alvl3HighVoltage, alvl3LowVoltage;
static int dvfm_dev_id;
static int d0cs_lpj = 293888 * 100 / HZ;
static int comm_wakeup_time, timer_expiry_time;
extern unsigned int AppsSyncEnabled;

enum pxa9xx_force_lpm ForceLPM = PXA9xx_Force_None;
enum pxa9xx_force_lpm LastForceLPM = PXA9xx_Force_None;
unsigned int ForceLPMWakeup;
#define EARLY_D0CS_WAKE
#define D0CS_TIMER_THRESHOLD 40

/* define the operating point of S0D0 and S0D0CS mode */
static struct dvfm_md_opt pxa955_op_array[] = {
	/* 60MHz -- ring oscillator */
	{
#ifdef CONFIG_PMIC_D1980
		.vcc_core = 1150,//1250,alvl0
#else
		.vcc_core = 1250,
#endif
		.xl = 0,
		.xn = 0,
		.smcfs = 15,
		.sflfs = 60,
		.hss = 60,
		.axifs = 60,
		.dmcfs = 30,
		.df_clk = 15,
		.empi_clk = 15,
		.dsi = 30,
		.gcfs = 60,
		.vmfc = 120,
		.power_mode = POWER_MODE_D0CS,
		.flag = OP_FLAG_FACTORY,
		.lpj = 293888 * 100 / HZ,
		.name = "D0CS",
	},
	/* 156MHz -- single PLL mode */
	{
#ifdef CONFIG_PMIC_D1980
		.vcc_core = 1150,//1250,alvl0
#else
		.vcc_core = 1250,
#endif
		.xl = 12,
		.xn = 1,
		.smcfs = 156,
		.sflfs = 156,
		.hss = 104,
		.axifs = 78,
		.dmcfs = 208,
		.df_clk = 78,
		.empi_clk = 52,
		.dsi = 156,
		.gcfs = 156,
		.vmfc = 156,
		.power_mode = POWER_MODE_D0,
		.flag = OP_FLAG_FACTORY,
		.lpj = 778128 * 100 / HZ,
		.name = "156M",
	},
	/* 208MHz canceled pxa955 */
	/*{
		.vcc_core = 1250,
		.xl = 16,
		.xn = 1,
		.smcfs = 156,
		.sflfs = 156,
		.hss = 104,
		.axifs = 78,
		.dmcfs = 208,
		.df_clk = 78,
		.empi_clk = 52,
		.dsi = 156,
		.power_mode = POWER_MODE_D0,
		.flag = OP_FLAG_FACTORY,
		.lpj = 1036288*100/HZ,
		.name = "208M",
	  }, */
	/* 156MHz -- with HF DDR */
	{
#ifdef CONFIG_PMIC_D1980
		.vcc_core = 1175,//1250,alvl1
#else
		.vcc_core = 1250,
#endif
		.xl = 12,
		.xn = 1,
		.smcfs = 156,
		.sflfs = 156,
		.hss = 104,
		.axifs = 78,
		.dmcfs = 312,
		.df_clk = 78,
		.empi_clk = 52,
		.dsi = 156,
		.gcfs = 156,
		.vmfc = 156,
		.power_mode = POWER_MODE_D0,
		.flag = OP_FLAG_FACTORY,
		.lpj = 778128 * 100 / HZ,
		.name = "156M_HF",
	},
	/* 208MHz -- with HF DDR - canceled pxa955 */
	{
#ifdef CONFIG_PMIC_D1980
		.vcc_core = 1175,//alvl1
#else
		.vcc_core = 1250,
#endif
		.xl = 16,
		.xn = 1,
		.smcfs = 156,
		.sflfs = 156,
		.hss = 104,
		.axifs = 78,
		.dmcfs = 312,
		.df_clk = 78,
		.empi_clk = 52,
		.dsi = 312,
		.gcfs = 156,
		.vmfc = 156,
		.power_mode = POWER_MODE_D0,
		.flag = OP_FLAG_FACTORY,
		.lpj = 1036288*100/HZ,
		.name = "208M_HF",
	  },

	/* 416MHz, added for VGA scenario */
	{
#ifdef CONFIG_PMIC_D1980
		.vcc_core = 1175,//1250,alvl1
#else
		.vcc_core = 1250,
#endif
		.xl = 16,
		.xn = 2,
		.smcfs = 156,
		.sflfs = 156,
		.hss = 104,
		.axifs = 78,
		.dmcfs = 312,
		.df_clk = 78,
		.empi_clk = 52,
		.dsi = 312,
		.gcfs = 156,
		.vmfc = 156,
		.power_mode = POWER_MODE_D0,
		.flag = OP_FLAG_FACTORY,
		.lpj = 2076672 * 100 / HZ,
		.name = "416M_VGA",
	},

	/* 416MHz */
	{
#ifdef CONFIG_PMIC_D1980
                .vcc_core = 1175,//1250,alvl1
#else
                .vcc_core = 1250,
#endif
		.xl = 16,
		.xn = 2,
		.smcfs = 156,
		.sflfs = 208,
		.hss = 156,
		.axifs = 156,
		.dmcfs = 312,
		.df_clk = 78,
		.empi_clk = 52,
		.dsi = 312,
		.gcfs = 312,
		.vmfc = 312,
		.power_mode = POWER_MODE_D0,
		.flag = OP_FLAG_FACTORY,
		.lpj = 2076672 * 100 / HZ,
		.name = "416M",
	},
	/* 624MHz */
	{
#ifdef CONFIG_PMIC_D1980
		.vcc_core = 1175, //1250,alvl2
#else
		.vcc_core = 1250,
#endif
		.xl = 24,
		.xn = 2,
		.smcfs = 156,
		.sflfs = 312,
		.hss = 208,
		.axifs = 156,
		.dmcfs = 400,
		.df_clk = 78,
		.empi_clk = 52,
		.dsi = 312,
		.gcfs = 312,
		.vmfc = 312,
		.power_mode = POWER_MODE_D0,
		.flag = OP_FLAG_FACTORY,
		.lpj = 3112960 * 100 / HZ,
		.name = "624M",
	},
	/* 728MHz */
	/*{
		.vcc_core = 1250,
		.xl = 28,
		.xn = 2,
		.smcfs = 156,
		.sflfs = 312,
		.hss = 208,
		.axifs = 156,
		.dmcfs = 400,
		.df_clk = 78,
		.empi_clk = 52,
		.dsi = 312,
		.gcfs = 312,
		.vmfc = 312,
		.power_mode = POWER_MODE_D0,
		.flag = OP_FLAG_FACTORY,
		.lpj = 3607102*100/HZ,
		.name = "728M",
	  }, */
	/* 806MHz */
	{
#ifdef CONFIG_PMIC_D1980
		.vcc_core = 1275,//1250,//alvl3
#else
		.vcc_core = 1250,
#endif
		.xl = 31,
		.xn = 2,
		.smcfs = 156,
		.sflfs = 312,
		.hss = 208,
		.axifs = 156,
		.dmcfs = 400,
		.df_clk = 78,
		.empi_clk = 52,
		.dsi = 312,
		.gcfs = 312,
		.vmfc = 312,
		.power_mode = POWER_MODE_D0,
		.flag = OP_FLAG_FACTORY,
		.lpj = 4020906 * 100 / HZ,
		.name = "806M",
	},
#if defined(CONFIG_PXA95x_1GHZ_SUPPORT)
	/* 988MHz */
	{
#ifdef CONFIG_PMIC_D1980
		.vcc_core = 1300,//1250,alvl3
#else
		.vcc_core = 1250,
#endif
		.xl = 38,
		.xn = 2,
		.smcfs = 156,
		.sflfs = 312,
		.hss = 208,
		.axifs = 78,
		.dmcfs = 400,
		.df_clk = 78,
		.empi_clk = 52,
		.dsi = 312,
		.gcfs = 156,
		.vmfc = 156,
		.power_mode = POWER_MODE_D0,
		.flag = OP_FLAG_FACTORY,
		.lpj = 4929132*100/HZ,
		.name = "988M",
	},
#endif
	/* D1 mode */
	{
#ifdef CONFIG_PMIC_D1980
		.vcc_core = 1150,
#else
		.vcc_core = 1250,
#endif
		.power_mode = POWER_MODE_D1,
		.flag = OP_FLAG_FACTORY,
		.name = "D1",
	},
	/* D2 mode */
	{
#ifdef CONFIG_PMIC_D1980
		.vcc_core = 1150,
#else
		.vcc_core = 1250,
#endif
		.power_mode = POWER_MODE_D2,
		.flag = OP_FLAG_FACTORY,
		.name = "D2",
	},
	/* CG (clock gated) mode */
	{
#ifdef CONFIG_PMIC_D1980
		.vcc_core = 1150,
#else
		.vcc_core = 1250,
#endif
		.power_mode = POWER_MODE_CG,
		.flag = OP_FLAG_FACTORY,
		.name = "CG",
	},

};

static struct dvfm_md_opt pxa970_op_array[] = {
	/* 60MHz -- ring oscillator */
	{
		.vcc_core = 1250,
		.xl = 0,
		.xn = 0,
		.smcfs = 15,
		.sflfs = 60,
		.hss = 60,
		.axifs = 60,
		.dmcfs = 30,
		.df_clk = 15,
		.empi_clk = 15,
		.dsi = 30,
		.gcfs = 60,
		.vmfc = 120,
		.power_mode = POWER_MODE_D0CS,
		.flag = OP_FLAG_FACTORY,
		.lpj = 293888 * 100 / HZ,
		.name = "D0CS",
	},
	/* 156MHz -- single PLL mode */
	{
		.vcc_core = 1250,
		.xl = 12,
		.xn = 1,
		.smcfs = 104,
		.sflfs = 156,
		.hss = 104,
		.axifs = 78,
		.dmcfs = 208,
		.df_clk = 52,
		.empi_clk = 52,
		.dsi = 156,
		.gcfs = 156,
		.vmfc = 156,
		.power_mode = POWER_MODE_D0,
		.flag = OP_FLAG_FACTORY,
		.lpj = 778128 * 100 / HZ,
		.name = "156M",
	},
	/* 208MHz canceled pxa970 */
	/*{
		.vcc_core = 1250,
		.xl = 16,
		.xn = 1,
		.smcfs = 104,
		.sflfs = 156,
		.hss = 104,
		.axifs = 78,
		.dmcfs = 208,
		.df_clk = 52,
		.empi_clk = 52,
		.dsi = 156,
		.power_mode = POWER_MODE_D0,
		.flag = OP_FLAG_FACTORY,
		.lpj = 1036288*100/HZ,
		.name = "208M",
	  }, */
	/* 156MHz -- with HF DDR */
	{
		.vcc_core = 1250,
		.xl = 12,
		.xn = 1,
		.smcfs = 104,
		.sflfs = 156,
		.hss = 104,
		.axifs = 78,
		.dmcfs = 400,
		.df_clk = 52,
		.empi_clk = 52,
		.dsi = 156,
		.gcfs = 156,
		.vmfc = 156,
		.power_mode = POWER_MODE_D0,
		.flag = OP_FLAG_FACTORY,
		.lpj = 778128 * 100 / HZ,
		.name = "156M_HF",
	},
	/* 208MHz -- with HF DDR - canceled pxa970 */
	/*{
		.vcc_core = 1250,
		.xl = 16,
		.xn = 1,
		.smcfs = 104,
		.sflfs = 156,
		.hss = 208,
		.axifs = 156,
		.dmcfs = 400,
		.df_clk = 52,
		.empi_clk = 52,
		.dsi = 312,
		.power_mode = POWER_MODE_D0,
		.flag = OP_FLAG_FACTORY,
		.lpj = 1036288*100/HZ,
		.name = "208M_HF",
	  }, */

	/* 416MHz */
	{
		.vcc_core = 1250,
		.xl = 16,
		.xn = 2,
		.smcfs = 156,
		.sflfs = 208,
		.hss = 156,
		.axifs = 156,
		.dmcfs = 400,
		.df_clk = 78,
		.empi_clk = 52,
		.dsi = 312,
		.gcfs = 312,
		.vmfc = 312,
		.power_mode = POWER_MODE_D0,
		.flag = OP_FLAG_FACTORY,
		.lpj = 2076672 * 100 / HZ,
		.name = "416M",
	},
	/* 624MHz */
	{
		.vcc_core = 1250,
		.xl = 24,
		.xn = 2,
		.smcfs = 156,
		.sflfs = 312,
		.hss = 208,
		.axifs = 156,
		.dmcfs = 400,
		.df_clk = 78,
		.empi_clk = 52,
		.dsi = 312,
		.gcfs = 312,
		.vmfc = 312,
		.power_mode = POWER_MODE_D0,
		.flag = OP_FLAG_FACTORY,
		.lpj = 3112960 * 100 / HZ,
		.name = "624M",
	},
	/* 728MHz */
	/*{
		.vcc_core = 1250,
		.xl = 28,
		.xn = 2,
		.smcfs = 156,
		.sflfs = 312,
		.hss = 208,
		.axifs = 156,
		.dmcfs = 400,
		.df_clk = 78,
		.empi_clk = 52,
		.dsi = 312,
		.gcfs = 312,
		.vmfc = 312,
		.power_mode = POWER_MODE_D0,
		.flag = OP_FLAG_FACTORY,
		.lpj = 3607102*100/HZ,
		.name = "728M",
	  }, */
	/* 806MHz */
	{
		.vcc_core = 1250,
		.xl = 31,
		.xn = 2,
		.smcfs = 156,
		.sflfs = 312,
		.hss = 208,
		.axifs = 156,
		.dmcfs = 400,
		.df_clk = 78,
		.empi_clk = 52,
		.dsi = 312,
		.gcfs = 312,
		.vmfc = 312,
		.power_mode = POWER_MODE_D0,
		.flag = OP_FLAG_FACTORY,
		.lpj = 4020906 * 100 / HZ,
		.name = "806M",
	},
	/* D1 mode */
	{
		.vcc_core = 1250,
		.power_mode = POWER_MODE_D1,
		.flag = OP_FLAG_FACTORY,
		.name = "D1",
	},
	/* D2 mode */
	{
		.vcc_core = 1250,
		.power_mode = POWER_MODE_D2,
		.flag = OP_FLAG_FACTORY,
		.name = "D2",
	},
	/* CG (clock gated) mode */
	{
		.vcc_core = 1250,
		.power_mode = POWER_MODE_CG,
		.flag = OP_FLAG_FACTORY,
		.name = "CG",
	},

};

/* workaround for bug JIRA MG2-462 */
int is_wkr_mg2_462(void)
{
	return cpu_is_pxa968_C0();
}

struct proc_op_array {
	unsigned int cpuid;
	char *cpu_name;
	struct dvfm_md_opt *op_array;
	unsigned int nr_op;
};

static struct proc_op_array proc_op_arrays[] = {
	{0x5810, "pxa955", ARRAY_AND_SIZE(pxa955_op_array)},
	{0x5815, "pxa970", ARRAY_AND_SIZE(pxa970_op_array)},
};

extern void pxa_clkcfg_write(unsigned int, unsigned char __iomem *);

static int prepare_dmc(void *driver_data, int flag);
static int polling_dmc(void *driver_data);

#ifdef CONFIG_ISPT
#define ispt_dvfm_op ispt_dvfm_msg

static int ispt_block_dvfm(int enable, int dev_id)
{
	int ret;
	if (enable)
		ret = ispt_driver_msg(CT_P_DVFM_BLOCK_REQ, dev_id);
	else
		ret = ispt_driver_msg(CT_P_DVFM_BLOCK_REL, dev_id);
	return ret;
}
#else
static int ispt_dvfm_op(int old, int new)
{
	return 0;
}

static int ispt_block_dvfm(int enable, int dev_id)
{
	return 0;
}
#endif

static struct regulator *v_buck1;

static unsigned int pxa95x_dvfm_get_core_voltage(void)
{
	int vcc_volt;

	if (IS_ERR(v_buck1)) {
		printk(KERN_ERR "%s: invalid v_buck1\n", __func__);
		return -EIO;
	}

	vcc_volt = regulator_get_voltage(v_buck1);
	return vcc_volt / 1000;
}

static int pxa95x_dvfm_set_core_voltage(unsigned int mV)
{
	int vcc_volt;

	if (IS_ERR(v_buck1)) {
		printk(KERN_ERR "%s: invalid v_buck1\n", __func__);
		return -EIO;
	}

	if (regulator_set_voltage(v_buck1, mV * 1000, mV * 1000))
		return -EIO;
	vcc_volt = regulator_get_voltage(v_buck1);
	if (vcc_volt != mV * 1000)
		return -EIO;
	return 0;
}

/* #####################Debug Function######################## */
static int dump_op(void *driver_data, struct op_info *p, char *buf)
{
	int len, count, x;
	struct dvfm_md_opt *md = (struct dvfm_md_opt *) p->op;

	if (md == NULL)
		len = sprintf(buf, "Can't dump the op info\n");
	else {
		/* calculate how much bits is set in device word */
		x = p->device;
		for (count = 0; x; x = x & (x - 1), count++)
			;
		len = sprintf(buf, "OP:%d name:%s [%s, %d]\n",
				p->index, md->name, (count) ? "Disabled"
				: "Enabled", count);
		len += sprintf(buf + len, "vcore:%d vsram:%d xl:%d xn:%d "
				"smcfs:%d sflfs:%d hss:%d dmcfs:%d ",
				md->vcc_core, md->vcc_sram, md->xl, md->xn,
				md->smcfs, md->sflfs, md->hss, md->dmcfs);
		len += sprintf(buf + len, "axifs:%d ", md->axifs);
		len += sprintf(buf + len, "gcfs:%d vmfc:%d ", md->gcfs,
				md->vmfc);
		len += sprintf(buf + len, "df_clk:%d power_mode:%d flag:%d\n",
				md->df_clk, md->power_mode, md->flag);
	}
	return len;
}

static int dump_op_list(void *driver_data, struct info_head *op_table,
			int flag)
{
	struct op_info *p = NULL;
	struct dvfm_md_opt *q = NULL;
	struct list_head *list = NULL;
	struct pxa95x_dvfm_info *info = driver_data;
	char buf[256];

	if (!op_table || list_empty(&op_table->list)) {
		printk(KERN_WARNING "op list is null\n");
		return -EINVAL;
	}
	memset(buf, 0, 256);
	list_for_each(list, &op_table->list) {
		p = list_entry(list, struct op_info, list);
		q = (struct dvfm_md_opt *) p->op;
		if (q->flag <= flag) {
			dump_op(info, p, buf);
			pr_debug("%s", buf);
		}
	}
	return 0;
}

/* ########################################################## */
static int freq2reg(struct pxa95x_fv_info *fv_info,
		    struct dvfm_md_opt *orig)
{
	int res = -EFAULT, tmp;

	if (orig && fv_info) {
		fv_info->vcc_core = orig->vcc_core;
		fv_info->vcc_sram = orig->vcc_sram;
		if (orig->power_mode == POWER_MODE_D0) {
			res = 0;
			fv_info->xl = orig->xl;
			fv_info->xn = orig->xn;
			fv_info->d0cs = 0;
			if (orig->smcfs == 78)
				fv_info->smcfs = 0;
			else if (orig->smcfs == 104)
				fv_info->smcfs = 2;
			else if (orig->smcfs == 156)
				fv_info->smcfs = 4;
			else if (orig->smcfs == 208)
				fv_info->smcfs = 5;
			else
				res = -EINVAL;
			if (orig->sflfs == 104)
				fv_info->sflfs = 0;
			else if (orig->sflfs == 156)
				fv_info->sflfs = 1;
			else if (orig->sflfs == 208)
				fv_info->sflfs = 2;
			else if (orig->sflfs == 312)
				fv_info->sflfs = 3;
			else
				res = -EINVAL;
			if (orig->hss == 104)
				fv_info->hss = 0;
			else if (orig->hss == 156)
				fv_info->hss = 1;
			else if (orig->hss == 208)
				fv_info->hss = 2;
			else
				res = -EINVAL;
			if (orig->axifs == 78)
				fv_info->axifs = 3;
			else if (orig->axifs == 104)
				fv_info->axifs = 0;
			else if (orig->axifs == 156)
				fv_info->axifs = 1;
			else if (orig->axifs == 208)
				fv_info->axifs = 2;
			else
				res = -EINVAL;
			if (orig->dmcfs == 26)
				fv_info->dmcfs = 0;
			else if (orig->dmcfs == 208)
				fv_info->dmcfs = 2;
			else if (orig->dmcfs == 400 || orig->dmcfs == 312)
				fv_info->dmcfs = 3;
			else
				res = -EINVAL;
			if (orig->gcfs == 400)
				fv_info->gcfs = 3;
			else if (orig->gcfs == 312)
				fv_info->gcfs = 2;
			else if (orig->gcfs == 208)
				fv_info->gcfs = 0;
			else if (orig->gcfs == 156)
				fv_info->gcfs = 1;
			else
				res = -EINVAL;

			if (orig->vmfc == 312)
				fv_info->vmfc = 1;
			else if (orig->vmfc == 156)
				fv_info->vmfc = 0;
			else
				res = -EINVAL;
			tmp = orig->smcfs / orig->df_clk;
			if (tmp == 2)
				fv_info->df_clk = 2;
			else if (tmp == 4)
				fv_info->df_clk = 3;
			fv_info->empi_clk = fv_info->df_clk;
		} else if (orig->power_mode == POWER_MODE_D0CS) {
			fv_info->d0cs = 1;
			res = 0;
		}
	}
	return res;
}

int md2fvinfo(struct pxa95x_fv_info *fv_info, struct dvfm_md_opt *orig)
{
	return freq2reg(fv_info, orig);
}

static int reg2freq(void *driver_data, struct dvfm_md_opt *fv_info)
{
	struct pxa95x_dvfm_info *info = driver_data;
	int res = -EFAULT, tmp;
	uint32_t accr;

	if (fv_info) {
		res = 0;
		if (fv_info->power_mode == POWER_MODE_D0CS) {
			/* set S0D0CS operating pointer */
			fv_info->power_mode = POWER_MODE_D0CS;
			fv_info->xl = 0;
			fv_info->xn = 0;
			fv_info->smcfs = 15;
			fv_info->sflfs = 60;
			fv_info->hss = 60;
			/* PXA310 A2 or PXA935/PXA940 */
			accr = __raw_readl(info->clkmgr_base + ACCR_OFF);
			if (accr & 0x80)
				fv_info->dmcfs = 60;
			else
				fv_info->dmcfs = 30;
			fv_info->df_clk = 15;
			fv_info->empi_clk = 15;
			fv_info->axifs = 60;
		} else {
			/* set S0D0 operating pointer */
			fv_info->power_mode = POWER_MODE_D0;
			tmp = fv_info->smcfs;
			if (tmp == 0)
				fv_info->smcfs = 78;
			else if (tmp == 2)
				fv_info->smcfs = 104;
			else if (tmp == 4)
				fv_info->smcfs = 156;
			else if (tmp == 5)
				fv_info->smcfs = 208;
			else
				res = -EINVAL;
			tmp = fv_info->sflfs;
			if (tmp == 0)
				fv_info->sflfs = 104;
			else if (tmp == 1)
				fv_info->sflfs = 156;
			else if (tmp == 2)
				fv_info->sflfs = 208;
			else if (tmp == 3)
				fv_info->sflfs = 312;
			tmp = fv_info->hss;
			if (tmp == 0)
				fv_info->hss = 104;
			else if (tmp == 1)
				fv_info->hss = 156;
			else if (tmp == 2)
				fv_info->hss = 208;
			else
				res = -EINVAL;
			tmp = fv_info->dmcfs;
			if (tmp == 0)
				fv_info->dmcfs = 26;
			else if (tmp == 2)
				fv_info->dmcfs = 208;
			else if (tmp == 3) {
				accr = __raw_readl(info->clkmgr_base + ACCR_OFF);
				if (accr & ACCR_DMCFS_312_MASK)
					fv_info->dmcfs = 312;
				else
					fv_info->dmcfs = 400;
			}
			else
				res = -EINVAL;
			tmp = fv_info->df_clk;
			if (tmp == 1)
				fv_info->df_clk = fv_info->smcfs;
			else if (tmp == 2)
				fv_info->df_clk = fv_info->smcfs / 2;
			else if (tmp == 3)
				fv_info->df_clk = fv_info->smcfs / 4;
			fv_info->empi_clk = fv_info->df_clk;

			tmp = fv_info->axifs;
			if (tmp == 0)
				fv_info->axifs = 104;
			else if (tmp == 1)
				fv_info->axifs = 156;
			else if (tmp == 3)
				fv_info->axifs = 78;
			else if (tmp == 2)
				fv_info->axifs = 208;
			else
				res = -EINVAL;
			tmp = fv_info->gcfs;
			if (tmp == 0)
				fv_info->gcfs = 208;
			else if (tmp == 1)
				fv_info->gcfs = 156;
			else if (tmp == 2)
				fv_info->gcfs = 312;
			else if (tmp == 3)
				fv_info->gcfs = 400;
			tmp = fv_info->vmfc;
			if (tmp == 0)
				fv_info->vmfc = 156;
			else if (tmp == 1)
				fv_info->vmfc = 312;
		}
	}
	return res;
}

/* Get current setting, and record it in fv_info structure
 */
static int capture_op_info(void *driver_data, struct dvfm_md_opt *fv_info)
{
	struct pxa95x_dvfm_info *info = driver_data;
	int res = -EFAULT;
	uint32_t acsr, memclkcfg, accr1;

	if (fv_info) {
		memset(fv_info, 0, sizeof(struct dvfm_md_opt));
		acsr = __raw_readl(info->clkmgr_base + ACSR_OFF);
		fv_info->xl = (acsr >> ACCR_XL_OFFSET) & 0x3F;
		fv_info->xn = (acsr >> ACCR_XN_OFFSET) & 0x07;
		fv_info->smcfs = (acsr >> ACCR_SMCFS_OFFSET) & 0x07;
		fv_info->sflfs = (acsr >> ACCR_SFLFS_OFFSET) & 0x03;
		fv_info->hss = (acsr >> ACCR_HSS_OFFSET) & 0x03;
		fv_info->dmcfs = (acsr >> ACCR_DMCFS_OFFSET) & 0x03;
		fv_info->power_mode = (acsr >> ACCR_D0CS_OFFSET) & 0x01;
		memclkcfg = __raw_readl(info->smc_base + MEMCLKCFG_OFF);
		fv_info->df_clk = (memclkcfg >> MEMCLKCFG_DF_OFFSET) & 0x07;
		fv_info->empi_clk = (memclkcfg >> MEMCLKCFG_EMPI_OFFSET) & 0x07;
		fv_info->axifs = (acsr >> ACSR_AXIFS_OFFSET) & 0x03;
		accr1 = __raw_readl(info->clkmgr_base + ACCR1_OFF);
		fv_info->gcfs = (acsr >> ACSR_GCFS_OFFSET) & 0x03;
		fv_info->vmfc = (accr1 >> ACCR1_VMFC_OFFSET) & 0x01;
		/* Convert bits into frequency */
		res = reg2freq(info, fv_info);
		/* disable useless get voltage action */
		/* fv_info->vcc_core = pxa95x_dvfm_get_core_voltage(); */
		/* TODO: mix up the usage of struct dvfm_md_opt and
		 * struct pxa95x_fv_info. Better to define
		 * reg2freq(struct dvfm_md_opt *md_info,
		 * struct pxa95x_fv_info *fv_info)
		 */
	}
	return res;
}

/* return all op including user defined op, and boot op */
static int get_op_num(void *driver_data, struct info_head *op_table)
{
	struct list_head *entry = NULL;
	int num = 0;

	if (!op_table)
		goto out;
	read_lock(&op_table->lock);
	if (list_empty(&op_table->list)) {
		read_unlock(&op_table->lock);
		goto out;
	}
	list_for_each(entry, &op_table->list) {
		num++;
	}
	read_unlock(&op_table->lock);
out:
	return num;
}

/* return op name. */
static char *get_op_name(void *driver_data, struct op_info *p)
{
	struct dvfm_md_opt *q = NULL;
	if (p == NULL)
		return NULL;
	q = (struct dvfm_md_opt *) p->op;
	return q->name;
}

#ifdef CONFIG_PMIC_D1980
extern void d1980_pi2c_write2bulk1(int value); 
#endif
static int update_voltage(void *driver_data, struct dvfm_md_opt *old,
			  struct dvfm_md_opt *new)
{
	struct pxa95x_dvfm_info *info = driver_data;
	/* we will disable power for mg1, for pv2 pm is enabled
	   only if the PowerDisabled flag is set (using sys) */
	if (DvfmDisabled)
		return 0;

#ifdef CONFIG_PMIC_D1980
#if 1
        d1980_pi2c_write2bulk1(new->vcc_core);
#else
        info->current_voltage=new->vcc_core;
        schedule_work(&info->voltage_work);
#endif        
#else
	if (!(info->flags & PXA95x_USE_POWER_I2C))
		pxa95x_dvfm_set_core_voltage(new->vcc_core);
#endif
	return 0;
}

static struct timer_list d0cs_prevent_timer;
static void pxa95x_d0cs_prevent_cbk(unsigned long data)
{
#if defined(CONFIG_PXA9XX_ACIPC)
	acipc_disable_d0cs();
#endif
#ifdef CONFIG_PXA_MIPSRAM
	MIPS_RAM_ADD_PM_TRACE(EARLY_DDR_PREVENTING_D0CS);
#endif
	del_timer_sync(&d0cs_prevent_timer);
}

static void pxa95x_d0cs_clear_aicsr(void)
{
	uint32_t aicsr;
	/*
	 * Clearing AICSR D0CS transition wakeup
	 */
	aicsr = AICSR;
	/* Clear all status bit so we don't miss any status bit
	 * (write to clear) */
	aicsr &= ~AICSR_STATUS_BITS;
	aicsr |= AICSR_WSD0CS;	/*Set D0CS status bit */
	AICSR = aicsr;
}



static void pxa95x_enter_d0cs(void *driver_data)
{
	struct pxa95x_dvfm_info *info = driver_data;
	unsigned int icmr[3] = { 0, 0, 0 };
	unsigned int reg, spll = 0;
	uint32_t accr, acsr, mdrefr;

	acsr = __raw_readl(info->clkmgr_base + ACSR_OFF);
	/* making sure we are not in D0CS before we try to enter. */
	if (!(acsr & (1 << ACCR_D0CS_OFFSET))) {
		DVFMLPMGlobalCount.D0CS_Enter_count++;
#if defined EARLY_D0CS_WAKE
		/* we will call the timer only if apps comm sync is enabled */
		if (AppsSyncEnabled) {
			timer_expiry_time = comm_wakeup_time -
				D0CS_TIMER_THRESHOLD;
			/* setting the timer expiry time to be 30 mse before
			 * next comm wakeup
			 */
			timer_expiry_time = msecs_to_jiffies(timer_expiry_time);
			/* we will set a timer so we will exit D0CS TH msec
			 * before the next comm expected wakeup.
			 */
			mod_timer(&d0cs_prevent_timer,
				  jiffies + timer_expiry_time);
		}
#endif

		reg = (12 << ACCR_XL_OFFSET) | (1 << ACCR_XN_OFFSET);
		accr = __raw_readl(info->clkmgr_base + ACCR_OFF);
		if (reg == (accr & (ACCR_XN_MASK | ACCR_XL_MASK))) {
			spll = 1;
		}

		if (is_wkr_mg1_1274()) {
			/* Mask all interrupts except R2B20 BCCU */
			icmr[0] = ICMR;
			icmr[1] = ICMR2;
			icmr[2] = ICMR3;
			ICMR = 0x00000000;
			ICMR2 = 0x00100000;
			ICMR3 = 0x00000000;
		}
		prepare_dmc(info, DMEMC_D0CS_ENTER);
#if defined(CONFIG_PXA9XX_ACIPC)
		/*should not happen */
		if (get_ddr_avail_state())
			printk(KERN_ERR "****EDDR - WARNING - GOING TO D0CS"
					"WITH DDR INDICATION TO COM SET -"
					"*****");
#endif
		enter_d0cs_a(info->clkmgr_base, info->dmc_base,
				info->bpmu_base);

		polling_dmc(info);

		/* update to D0CS LPJ, it must be updated before udelay() */
		loops_per_jiffy = d0cs_lpj;
		udelay(200);

		/* disable PLL */

		accr = __raw_readl(info->clkmgr_base + ACCR_OFF);
		if (spll) {
			/* core pll is already disabled.
			 * Only disable System PLL */
			accr |= (1 << ACCR_SPDIS_OFFSET);
		} else {
			/* Disable both System PLL and Core PLL */
			accr |= (1 << ACCR_XPDIS_OFFSET) |
				(1 << ACCR_SPDIS_OFFSET);
		}
		__raw_writel(accr, info->clkmgr_base + ACCR_OFF);


		mdrefr = __raw_readl(info->dmc_base + MDREFR_OFF);
		__raw_writel(mdrefr, info->dmc_base + MDREFR_OFF);
	} else {
		printk(KERN_WARNING "Attempting to enter D0CS when we are"
				"already in D0CS\n");
	}

	/* Clearing D0CS wakeup status bit in order to clean ACCU interrupt */
	pxa95x_d0cs_clear_aicsr();

	if (is_wkr_mg1_1274()) {
		ICMR = icmr[0];
		ICMR2 = icmr[1];
		ICMR3 = icmr[2];
	}

}

static void pxa95x_exit_d0cs(void *driver_data)
{
	struct pxa95x_dvfm_info *info = driver_data;
	unsigned int spll = 0;
	uint32_t reg, accr, acsr, mdrefr;
	unsigned int icmr[3] = { 0, 0, 0 };

	acsr = __raw_readl(info->clkmgr_base + ACSR_OFF);
	/* making sure we are actually in D0CS before we try to exit,*/
	if (acsr & (1 << ACCR_D0CS_OFFSET)) {
		DVFMLPMGlobalCount.D0CS_Exit_count++;
		reg = (12 << ACCR_XL_OFFSET) | (1 << ACCR_XN_OFFSET);
		accr = __raw_readl(info->clkmgr_base + ACCR_OFF);
		if (reg == (accr & (ACCR_XN_MASK | ACCR_XL_MASK))) {
			spll = 1;
		}
		/* enable PLL */

		if (spll) {
			/* core PLL shound not be enabled.
			 * only enable System PLL */
			accr = __raw_readl(info->clkmgr_base + ACCR_OFF);
			accr &= ~(1 << ACCR_SPDIS_OFFSET);
			__raw_writel(accr, info->clkmgr_base + ACCR_OFF);
			do {
				acsr = __raw_readl(info->clkmgr_base +
						ACSR_OFF);
			} while ((acsr & ACSR_SPLCK) == 0);
		} else {
			/* enable both System PLL and Core PLL */
			accr = __raw_readl(info->clkmgr_base + ACCR_OFF);
			accr &= ~((1 << ACCR_XPDIS_OFFSET) |
				  (1 << ACCR_SPDIS_OFFSET));
			__raw_writel(accr, info->clkmgr_base + ACCR_OFF);
			do {
				acsr = __raw_readl(info->clkmgr_base +
						ACSR_OFF);
			} while (((acsr & ACSR_XPLCK) == 0) ||
					((acsr & ACSR_SPLCK) == 0));
		}


		prepare_dmc(info, DMEMC_D0CS_EXIT);
		if (is_wkr_mg1_1274()) {
			/*Mask all interrupts except R2B20 BCCU */
			icmr[0] = ICMR;
			icmr[1] = ICMR2;
			icmr[2] = ICMR3;
			ICMR = 0x00000000;
			ICMR2 = 0x00100000;
			ICMR3 = 0x00000000;
		}
		exit_d0cs_a(info->clkmgr_base, info->dmc_base, info->bpmu_base);
		polling_dmc(info);

		mdrefr = __raw_readl(info->dmc_base + MDREFR_OFF);
		__raw_writel(mdrefr, info->dmc_base + MDREFR_OFF);
#if defined(CONFIG_PXA9XX_ACIPC)
		set_DDR_avail_flag();
#endif
		/* Clearing D0CS wakeup status bit in order to clean ACCU
		 * interrupt */
		pxa95x_d0cs_clear_aicsr();

		if (is_wkr_mg1_1274()) {
			ICMR = icmr[0];
			ICMR2 = icmr[1];
			ICMR3 = icmr[2];
		}
#if defined EARLY_D0CS_WAKE
		if (AppsSyncEnabled) {
			/* we are checking the proximity to the next xomm
			 * wakeup to detrmine if this is a ap-cp-sync timer
			 */
			comm_wakeup_time = pxa95x_ticks_to_msec
				(dvfm_get_next_comm_wakeup_time());

			/* we should not disable the timer if we are not sure
			 * if this wake is because of the timer...
			 */
			/* if we are far enough from the comm wakeup we can
			 * disable the timer (it will be retriggered if we
			 * enter D0CS again
			 */
			if (comm_wakeup_time > D0CS_TIMER_THRESHOLD) {
				del_timer_sync(&d0cs_prevent_timer);
			}
		}
#endif
	} else {
		printk(KERN_WARNING "Attempting to exit D0CS when we are not"
				"in D0CS in the first place\n");
	}
}

/* Return 1 if Grayback PLL is on. */
static int check_grayback_pll(void *driver_data)
{
	struct pxa95x_dvfm_info *info = driver_data;

	return __raw_readl(info->clkmgr_base + OSCC_OFF) & (1 << 17);
}

static int set_grayback_pll(void *driver_data, int enable)
{
	struct pxa95x_dvfm_info *info = driver_data;
	uint32_t oscc, agenp;

	if (enable) {
		/*
		 * unmask enable/disable GBPLL commands initiated
		 * by AGENP[GBPLL_CTRL]
		 */
		oscc = __raw_readl(info->clkmgr_base + OSCC_OFF);
		oscc &= ~OSCC_GPRM;
		__raw_writel(oscc, info->clkmgr_base + OSCC_OFF);
		do {
			oscc = __raw_readl(info->clkmgr_base + OSCC_OFF);
		} while (oscc & OSCC_GPRM);

		/* turn on GB PLL by AGENP[GBPLL_CTRL] and AGENP[GBPLL_DATA] */
		agenp = __raw_readl(info->bpmu_base + AGENP_OFF);
		agenp |= (AGENP_GBPLL_CTRL | AGENP_GBPLL_DATA);
		__raw_writel(agenp, info->bpmu_base + AGENP_OFF);

		/* wait until GB PLL is ready for use as DMC clock source */
		do {
			oscc = __raw_readl(info->clkmgr_base + OSCC_OFF);
		} while (!(oscc & OSCC_GPLS));
	} else {
		/* turn off Grayback PLL, only set AGENP[GBPLL_CTRL] */
		agenp = __raw_readl(info->bpmu_base + AGENP_OFF);
		agenp &= ~(AGENP_GBPLL_CTRL | AGENP_GBPLL_DATA);
		agenp |= AGENP_GBPLL_CTRL;
		__raw_writel(agenp, info->bpmu_base + AGENP_OFF);

		/* wait until GB PLL isn't ready to use */
		do {
			oscc = __raw_readl(info->clkmgr_base + OSCC_OFF);
		} while (oscc & OSCC_GPLS);

		/*
		 * mask enable/disable GBPLL commands initiated
		 * by AGENP[GBPLL_CTRL]
		 */
		oscc = __raw_readl(info->clkmgr_base + OSCC_OFF);
		oscc |= OSCC_GPRM;
		__raw_writel(oscc, info->clkmgr_base + OSCC_OFF);
		do {
			oscc = __raw_readl(info->clkmgr_base + OSCC_OFF);
		} while (!(oscc & OSCC_GPRM));
	}
	return 0;
}

/*
 * Return 2 if MTS should be changed to 2.
 * Return 1 if MTS should be changed to 1.
 * Return 0 if MTS won't be changed.
 * In this function, the maxium MTS is 2.
 */
static int check_mts(struct dvfm_md_opt *old, struct dvfm_md_opt *new)
{
	int ret = 0;
	if ((old->xn == 1) && (new->xn == 2))
		ret = 2;
	if ((old->xn == 2) && (new->xn == 1))
		ret = 1;
	return ret;
}

static int set_mts(void *driver_data, int mts)
{
	struct pxa95x_dvfm_info *info = driver_data;
	unsigned int ascr;

	ascr = __raw_readl(info->bpmu_base + ASCR_OFF);
	ascr &= ~(3 << ASCR_MTS_OFFSET);
	ascr |= (mts << ASCR_MTS_OFFSET);
	__raw_writel(ascr, info->bpmu_base + ASCR_OFF);

	/* wait MTS is set */
	do {
		ascr = __raw_readl(info->bpmu_base + ASCR_OFF);
	} while (((ascr >> ASCR_MTS_OFFSET) & 0x3)
		 != ((ascr >> ASCR_MTS_S_OFFSET) & 0x3));

	return 0;
}

static int prepare_dmc(void *driver_data, int flag)
{
	struct pxa95x_dvfm_info *info = driver_data;
	int data;
	uint32_t mdcnfg;

	if (flag == DMEMC_D0CS_ENTER || flag == DMEMC_FREQ_LOW) {
		/* Set MDCNFG_HWFREQ & MDCNFG_HWNOPHD */
		mdcnfg = __raw_readl(info->dmc_base + MDCNFG_OFF);
		data = (1 << MDCNFG_HWFREQ_OFFSET) |
			(1 << MDCNFG_HWNOPHD_OFFSET);
		mdcnfg |= data;
		__raw_writel(mdcnfg, info->dmc_base + MDCNFG_OFF);
		do {
			mdcnfg = __raw_readl(info->dmc_base + MDCNFG_OFF);
		} while ((mdcnfg & data) != data);


		return 0;
	}
	if (flag == DMEMC_D0CS_EXIT || flag == DMEMC_FREQ_HIGH) {
		/* Set MDCNFG_HWFREQ */
		mdcnfg = __raw_readl(info->dmc_base + MDCNFG_OFF);
		mdcnfg |= (1 << MDCNFG_HWFREQ_OFFSET);
		__raw_writel(mdcnfg, info->dmc_base + MDCNFG_OFF);
		do {
			mdcnfg = __raw_readl(info->dmc_base + MDCNFG_OFF);
		} while ((mdcnfg & (1 << MDCNFG_HWFREQ_OFFSET)) == 0);

		return 0;
	}

	return -EINVAL;
}

static int polling_dmc(void *driver_data)
{
	struct pxa95x_dvfm_info *info = driver_data;
	unsigned int mdcnfg;
	/* polling MDCNFG_HWFREQ cleared */
	do {
		mdcnfg = __raw_readl(info->dmc_base + MDCNFG_OFF);
	} while (mdcnfg & (1 << MDCNFG_HWFREQ_OFFSET));
	return 0;
}

static int set_dmc60(void *driver_data, int flag)
{
	struct pxa95x_dvfm_info *info = driver_data;
	uint32_t accr, reg;

	accr = __raw_readl(info->clkmgr_base + ACCR_OFF);
	if (flag)
		accr |= 0x80;
	else
		accr &= ~0x80;
	__raw_writel(accr, info->clkmgr_base + ACCR_OFF);
	/* polling ACCR */
	do {
		reg = __raw_readl(info->clkmgr_base + ACCR_OFF);
	} while ((accr & 0x80) != (reg & 0x80));

	return 0;
}

/* FIXME: it's a workaround for MG1: hss change would make lcd hang,
 * so delay hss change to lcd end of frame
 */
extern int display_enabled;
static int new_hss = -1, cur_hss;

void update_hss(void)
{
	uint32_t accr, acsr, data = 0, mask = 0;
	if (new_hss >= 0 && cur_hss != new_hss) {
		accr = ACCR;
		data |= (new_hss << ACCR_HSS_OFFSET);
		mask |= ACCR_HSS_MASK;
		accr &= ~mask;
		accr |= data;
		ACCR = accr;
		/* wait until ACSR is changed */
		do {
			accr = ACCR;
			acsr = ACSR;
		} while ((accr & mask) != data || (acsr & mask) != data);
		cur_hss = new_hss;
	}
}

static int set_ddr_pll(struct pxa95x_dvfm_info *info)
{
	volatile unsigned int accr;
	accr = __raw_readl(info->clkmgr_base + ACCR_OFF);

	__raw_writel((accr & ~ACCR_DMCFS_312_MASK), info->clkmgr_base + ACCR_OFF);
	while (accr & ACCR_DMCFS_312_MASK)
		accr = __raw_readl(info->clkmgr_base + ACCR_OFF);
	return 0;
}

static int set_system_pll(struct pxa95x_dvfm_info *info)
{
	volatile unsigned int accr;

	accr = __raw_readl(info->clkmgr_base + ACCR_OFF);
	__raw_writel(accr | (1 << ACCR_DMCFS_312_OFFSET), info->clkmgr_base + ACCR_OFF);
	while (!(accr & (1 << ACCR_DMCFS_312_OFFSET)))
		accr = __raw_readl(info->clkmgr_base + ACCR_OFF);
	return 0;
}

/*
	Set the source PLL for the DDR must be before setting the high DDR frequency.
	Since for high frequency the hardware must knows the source PLL  BEFORE start the high frequency.
	Therefor it done while stay at low DDR frequency - when this bit is no care.
	Therefore changing DDR frequency between 312MHz to 400MHz change first to 208MHz
	and from 208MHz to the new DDR frequency.
*/
static int set_ddr_208Mhz(struct pxa95x_dvfm_info *info, struct dvfm_md_opt *old, struct dvfm_md_opt *new)
{
	volatile unsigned int accr, acsr;
	int rc = 0;

	prepare_dmc(info, DMEMC_FREQ_LOW);
	accr = __raw_readl(info->clkmgr_base + ACCR_OFF);
	accr = (accr & ~ACCR_DMCFS_MASK) | (2 << ACCR_DMCFS_OFFSET); /* set 208 MHz */
	__raw_writel(accr , info->clkmgr_base + ACCR_OFF);

	/* verify that DDR change to 208MHz */
	do {
		acsr = __raw_readl(info->clkmgr_base + ACSR_OFF);
	} while ((acsr & ACCR_DMCFS_MASK) != (2 << ACCR_DMCFS_OFFSET));

	polling_dmc(info);
	/* the frequency is 208 now we can change  the clock source PLL */
	return rc;
}

/* TODO: sugguest to differentiate the operating point definition from
 * register info.And we can remove *reg_new here, and convert dvfm_md_opt to
 * it in the routine. That will make it much more clear.
 */
static int update_bus_freq(void *driver_data, struct dvfm_md_opt *old,
			   struct dvfm_md_opt *new)
{
	struct pxa95x_dvfm_info *info = driver_data;
	struct pxa95x_fv_info fv_info;
	uint32_t accr, acsr, accr1 = 0, mask = 0, mask2 = 0;
	unsigned int data = 0, data2 = 0;

	freq2reg(&fv_info, new);
	/* moving from High DDR to other High DDR frequency */
	if ((400 == old->dmcfs && 312 == new->dmcfs) || (312 == old->dmcfs && 400 == new->dmcfs)) {
		if (set_ddr_208Mhz(info, old, new)) {
			printk(KERN_ERR "ERROR %s set_ddr_208Mhz failed\n", __func__);
			return -1;
		}
		old->dmcfs = 208;
	}
	/*
	set the source PLL for the DDR must be before setting the high DDR frequency.
	Therefor it done while stay at low DDR frequency - when this bit is no care.
	*/
	if (new->dmcfs == 312 && old->dmcfs != 312)
		set_system_pll(info);
	else if (new->dmcfs == 400 && old->dmcfs != 400)
		set_ddr_pll(info);

	if (old->dmcfs < new->dmcfs)
		prepare_dmc(info, DMEMC_FREQ_HIGH);
	else if (old->dmcfs > new->dmcfs)
		prepare_dmc(info, DMEMC_FREQ_LOW);

	accr = __raw_readl(info->clkmgr_base + ACCR_OFF);
	accr1 = __raw_readl(info->clkmgr_base + ACCR1_OFF);
	if (old->gcfs != new->gcfs) {
		data2 |= (fv_info.gcfs << ACCR_GCFS_OFFSET);
		mask2 |= ACCR_GCFS_MASK;
	}
	if (old->vmfc != new->vmfc) {
		accr1 &= ~(ACCR1_VMFC_MASK);
		accr1 |= (fv_info.vmfc << ACCR1_VMFC_OFFSET);
	}
	if (old->smcfs != new->smcfs) {
		data |= (fv_info.smcfs << ACCR_SMCFS_OFFSET);
		mask |= ACCR_SMCFS_MASK;
	}
	if (old->sflfs != new->sflfs) {
		data |= (fv_info.sflfs << ACCR_SFLFS_OFFSET);
		mask |= ACCR_SFLFS_MASK;
	}
	if (old->dmcfs != new->dmcfs) {
		data |= (fv_info.dmcfs << ACCR_DMCFS_OFFSET);
		mask |= ACCR_DMCFS_MASK;
	}
	if (old->hss != new->hss) {
		if (!display_enabled) {
			data |= (fv_info.hss << ACCR_HSS_OFFSET);
			mask |= ACCR_HSS_MASK;
			cur_hss = fv_info.hss;
		}
		new_hss = fv_info.hss;
	}
	if (old->axifs != new->axifs) {
		data2 |= (fv_info.axifs << ACCR_AXIFS_OFFSET);
		mask2 |= ACCR_AXIFS_MASK;
	}
	accr &= ~(mask | mask2);
	accr |= data | data2;
	__raw_writel(accr, info->clkmgr_base + ACCR_OFF);
	__raw_writel(accr1, info->clkmgr_base + ACCR1_OFF);

	/* wait until ACSR is changed */
	do {
		accr = __raw_readl(info->clkmgr_base + ACCR_OFF);
		acsr = __raw_readl(info->clkmgr_base + ACSR_OFF);
	} while ((((accr & mask) != data) || ((acsr & mask) != data)) ||
			((accr & ACCR_AXIFS_MASK) >> ACCR_AXIFS_OFFSET !=
			 (acsr & ACSR_AXIFS_MASK) >> ACSR_AXIFS_OFFSET));
	do {
		accr = __raw_readl(info->clkmgr_base + ACCR_OFF);
		acsr = __raw_readl(info->clkmgr_base + ACSR_OFF);
	} while (((accr & ACCR_GCFS_MASK) >> ACCR_GCFS_OFFSET
			!= ((acsr & ACSR_GCFS_MASK) >> ACSR_GCFS_OFFSET)));
	if (old->dmcfs != new->dmcfs)
		polling_dmc(info);

	return 0;
}

void __iomem *addr_trim_value_wa;
/* This function is relevant for MG1 for handling trim values when changing core frequency */

/*
MG1:
	PP		PJ4_CPU_L2C_SRAM_SPD[9:0]	PJ4_CPU_SRAM_SPD[31:0]		pj4_cpu_sram_spd2[31:0]
	156-208		0				0				0
	416-624		0x2CA				0x0000_0BF5			0
	806		0x2CA				0x0000_0BF5			0x55555555
	988		0x2CA				0x0000_0BF5			0x66666666
*/
static void set_trim_values_mg1(struct dvfm_md_opt *old, struct dvfm_md_opt *new)
{
	unsigned int pj4_cpu_sram_spd, pj4_cpu_sram_spd2, pj4_cpu_l2c_sram_spd;
	int set_val = 0;

	pj4_cpu_l2c_sram_spd = __raw_readl(addr_trim_value_wa + 0x18);

	if (new->core <= 208) {
		/* core frequency 156-208 */
		if (old->core > 208) {
			set_val = 1;
			pj4_cpu_sram_spd = 0x0;
			pj4_cpu_sram_spd2 = 0x0;
			pj4_cpu_l2c_sram_spd &= 0xFFFFFC00;
		}
	} else if (new->core <= 624) {
		/* core frequency 416 - 624 */
		if ((old->core < 416) || (old->core > 624)) {
			set_val = 1;
			pj4_cpu_sram_spd = 0x00000BF5;
			pj4_cpu_sram_spd2 = 0x0;
			pj4_cpu_l2c_sram_spd &= 0xFFFFFC00;
			pj4_cpu_l2c_sram_spd |= 0x2CA;
		}
	} else if (new->core == 806) {
		/* core frequency 806 */
		set_val = 1;
		pj4_cpu_sram_spd = 0x00000BF5;
		pj4_cpu_sram_spd2 = 0x55555555;
		pj4_cpu_l2c_sram_spd &= 0xFFFFFC00;
		pj4_cpu_l2c_sram_spd |= 0x2CA;
	} else if (new->core == 988) {
		/* core frequency 988 */
		set_val = 1;
		pj4_cpu_sram_spd = 0x00000BF5;
		pj4_cpu_sram_spd2 = 0x66666666;
		pj4_cpu_l2c_sram_spd &= 0xFFFFFC00;
		pj4_cpu_l2c_sram_spd |= 0x2CA;
	} else
		BUG_ON(1);
	if (set_val) {
		__raw_writel(pj4_cpu_sram_spd, addr_trim_value_wa + 0x10);
		__raw_writel(pj4_cpu_sram_spd2, addr_trim_value_wa + 0x28);
		__raw_writel(pj4_cpu_l2c_sram_spd, addr_trim_value_wa + 0x18);
	}
}

/*
MG2:
	PP		PJ4_CPU_SRAM_SPD [17:0]		PJ4_TRIM_BITS_HMIPS [31:0]
	156-208		0				0
	416-624		0x19AF9				0
	806		0x19AF9				0x55555555
	988		0x19AF9				0x66666666
*/

#define BITS_0_17 0x3FFFF
static void set_trim_values_mg2(struct dvfm_md_opt *old, struct dvfm_md_opt *new)
{
	unsigned int cpu_sram_spd, trim_bits_hmips;
	int set_val_flag = 0;

	/* use "if else" instead of "switch case" - due to performance issue.*/
	if (new->core <= 208) {
		/* core frequency 156-208 */
		if (old->core > 208) {
			set_val_flag = 1;
			cpu_sram_spd = 0;
			trim_bits_hmips = 0;
		}
	} else if (new->core <= 624) {
		/* core frequency 416 - 624 */
		if ((old->core < 416) || (old->core > 624)) {
			set_val_flag = 1;
			cpu_sram_spd = 0x19AF9;
			trim_bits_hmips = 0;
		}
	} else if (new->core == 806) {
		/* core frequency 806 */
		set_val_flag = 1;
		cpu_sram_spd = 0x19AF9;
		trim_bits_hmips = 0x55555555;
	} else if (new->core == 988) {
		/* core frequency 988 */
		set_val_flag = 1;
		cpu_sram_spd = 0x19AF9;
		trim_bits_hmips = 0x66666666;
	} else
		BUG_ON(1);

	if (set_val_flag) {
		unsigned int sram_value_reg = __raw_readl(addr_trim_value_wa + 0x10);
		sram_value_reg = (sram_value_reg & (~BITS_0_17)) | cpu_sram_spd;
		__raw_writel(sram_value_reg, addr_trim_value_wa + 0x10);
		__raw_writel(trim_bits_hmips, addr_trim_value_wa + 0x28);
	}
}

static void set_trim_values(struct dvfm_md_opt *old, struct dvfm_md_opt *new)
{
	if (cpu_is_pxa968())
		set_trim_values_mg2(old, new);
	else if (cpu_is_pxa955())
		set_trim_values_mg1(old, new);
}

int temperture_sensor_int_high_freq_pp_callback(int highTempDetected)
{
	int rc, user_index;
	rc = dvfm_find_index("User", &user_index);
#if defined(CONFIG_PXA95x_1GHZ_SUPPORT)
	if (!rc) {
		if (highTempDetected == CORE_OVERHEATING_DETECTED)
			rc = dvfm_disable_op_name("988M", user_index);
		else
			rc = dvfm_enable_op_name("988M", user_index);
	}
#endif
	//pm_logger_app_add_trace(3, PM_SET_OP, OSCR4, new->core, ACCR, ACSR); 

	return rc;
}

unsigned long uboot_DefaultAvcrValue;

static int update_freq_for_voltage_change(void *driver_data,
			struct dvfm_md_opt *old, struct dvfm_md_opt *new)
{
	struct pxa95x_dvfm_info *info = driver_data;
	uint32_t avcr, avcr_temp, accr, acsr;

	/* Moving to/from PP8 (1GHz) requires a AVLV3 change
	 * We need to set always to the default value from OBM
	 * because all higher fields of each ALVL are write
	 * access only.*/
		avcr = uboot_DefaultAvcrValue;
		avcr &= ~(AVCR_ALVL3_MASK); /*clean the AVLV3*/
		/*We should not encounter fast transitions
		  from low PP to 1Ghz PP*/
		BUG_ON((new->xl == 38) && (old->xl < 31));

		if (new->xl == 38) /*moving to PP8 1GHz*/
			avcr |=	(alvl3HighVoltage << AVCR_ALVL3_OFFSET);
		else /*moving from PP8 1GHz*/
			avcr |=	(alvl3LowVoltage << AVCR_ALVL3_OFFSET);

		__raw_writel(avcr, info->spmu_base + AVCR_OFF);

		/* Verify the AVCR write.*/
		do {
			avcr_temp = __raw_readl(info->spmu_base + AVCR_OFF);
		} while ((avcr_temp & AVCR_ALVL3_MASK_5bit) !=
				(avcr & AVCR_ALVL3_MASK_5bit));
		/*We need to get through 624MHz if moving from 1GHz to 806MHz
		and backwards*/
		if (((old->xl == 31) && (new->xl == 38))
		|| ((old->xl == 38) && (new->xl == 31))) {
			accr = __raw_readl(info->clkmgr_base + ACCR_OFF);
			accr &= ~(ACCR_XL_MASK|ACCR_XN_MASK|ACCR_XSPCLK_MASK);
			accr |= ((24 << ACCR_XL_OFFSET) | (2 << ACCR_XN_OFFSET)
				| (3 << ACCR_XSPCLK_OFFSET));
			__raw_writel(accr, info->clkmgr_base + ACCR_OFF);
			/* delay 2 cycles of 13MHz clock */
			udelay(1);
			pxa_clkcfg_write(2, (unsigned char *)info->clkmgr_base);
			do {
				accr = __raw_readl(info->clkmgr_base + ACCR_OFF);
				acsr = __raw_readl(info->clkmgr_base + ACSR_OFF);
			} while ((accr & (ACCR_XL_MASK | ACCR_XN_MASK))
				!= (acsr & (ACCR_XL_MASK | ACCR_XN_MASK)));
		}
	return 0;
}

static int set_freq(void *driver_data, struct dvfm_md_opt *old,
		    struct dvfm_md_opt *new)
{
	struct pxa95x_dvfm_info *info = driver_data;
	int low_ddr;
	volatile uint32_t accr, acsr;

	/* check whether new OP is single PLL mode */
	if (new->dmcfs == 208 || new->dmcfs == 312)
		low_ddr = 1;
	else
		low_ddr = 0;

	set_trim_values(old, new);

	/* turn on Grayback PLL */
	if (!low_ddr && !check_grayback_pll(info))
		set_grayback_pll(info, 1);
	if (check_mts(old, new) == 2)
		set_mts(info, 2);

#if defined(CONFIG_PXA95x_1GHZ_SUPPORT)
	/*If the alvl3HighVoltage does not equal to zero
	then this mechanism is active*/
	if (((new->xl == 38) || (old->xl == 38)) && alvl3HighVoltage)
		update_freq_for_voltage_change(info, old, new);
#endif

	accr = __raw_readl(info->clkmgr_base + ACCR_OFF);
	accr &= ~(ACCR_XL_MASK | ACCR_XN_MASK | ACCR_XSPCLK_MASK);
	accr |= ((new->xl << ACCR_XL_OFFSET) | (new->xn << ACCR_XN_OFFSET)
		 | (3 << ACCR_XSPCLK_OFFSET));
	__raw_writel(accr, info->clkmgr_base + ACCR_OFF);
	/* delay 2 cycles of 13MHz clock */
	udelay(1);

	if (check_mts(old, new) == 1)
		set_mts(info, 1);

	if ((new->xl != old->xl) && (new->xn != old->xn))
		/* set F and T bit */
		pxa_clkcfg_write(3, info->clkmgr_base);
	else if ((new->xl == old->xl) && (new->xn != old->xn))
		/* Only set T-Bit */
		pxa_clkcfg_write(1, info->clkmgr_base);
	else
		/* set F bit */
		pxa_clkcfg_write(2, info->clkmgr_base);

	do {
		accr = __raw_readl(info->clkmgr_base + ACCR_OFF);
		acsr = __raw_readl(info->clkmgr_base + ACSR_OFF);
	} while ((accr & (ACCR_XL_MASK | ACCR_XN_MASK))
		 != (acsr & (ACCR_XL_MASK | ACCR_XN_MASK)));

	udelay(1);
	update_bus_freq(info, old, new);

	/* turn off Grayback PLL wen no need it*/
	if (low_ddr)
		set_grayback_pll(info, 0);
	pm_logger_app_add_trace(3, PM_SET_OP, OSCR4, new->core, ACCR, ACSR); 
	return 0;
}

static int update_freq(void *driver_data, struct dvfm_freqs *freqs)
{
	struct pxa95x_dvfm_info *info = driver_data;
	static struct dvfm_md_opt before_d0cs, pp806;
	struct dvfm_md_opt old, new;
	struct dvfm_md_opt *tmp;
	struct op_info *p = NULL;
	unsigned long flags;
	int found = 0, new_op = cur_op;
	static int found_pp_806;

	/* we will disable power for mg1 and pv2. pm is enabled
	 * only if the PowerDisabled flag is cleared (using sys)
	 */
	if (DvfmDisabled)
		return 0;

	memset(&old, 0, sizeof(struct dvfm_md_opt));
	memset(&new, 0, sizeof(struct dvfm_md_opt));
	write_lock_irqsave(&pxa95x_dvfm_op_list.lock, flags);
	if (!list_empty(&pxa95x_dvfm_op_list.list)) {
		list_for_each_entry(p, &pxa95x_dvfm_op_list.list, list) {
			if (!found_pp_806) {
				tmp = (struct dvfm_md_opt *)(p->op);
				if (!(strcmp(tmp->name , "806M"))) {
					found_pp_806 = 1;
					memcpy(&pp806, (struct dvfm_md_opt *) p->op,
						sizeof(struct dvfm_md_opt));
				}
			}
			if (p->index == freqs->old) {
				found++;
				memcpy(&old, (struct dvfm_md_opt *) p->op,
				       sizeof(struct dvfm_md_opt));
			}
			if (p->index == freqs->new) {
				found++;
				memcpy(&new, (struct dvfm_md_opt *) p->op,
				       sizeof(struct dvfm_md_opt));
				new_op = p->index;
			}
			if (found == 2)
				break;
		}
	}
	write_unlock_irqrestore(&pxa95x_dvfm_op_list.lock, flags);
	if (found != 2)
		return -EINVAL;

	if ((old.power_mode == POWER_MODE_D0)
	    && (new.power_mode == POWER_MODE_D0CS)) {
		memcpy(&before_d0cs, &old, sizeof(struct dvfm_md_opt));

		pxa95x_enter_d0cs(info);
		update_voltage(info, &old, &new);
		cur_op = new_op;
		loops_per_jiffy = new.lpj;
		return 0;
	} else if ((old.power_mode == POWER_MODE_D0CS)
		   && (new.power_mode == POWER_MODE_D0)) {
		if (memcmp(&before_d0cs, &new, sizeof(struct dvfm_md_opt))) {
			/* exit d0cs and set new operating point */
			if ((before_d0cs.vcc_core < new.vcc_core) ||
			    (before_d0cs.vcc_sram < new.vcc_sram)) {
				update_voltage(info, &old, &new);
			} else {
				update_voltage(info, &old, &before_d0cs);
			}
			pxa95x_exit_d0cs(info);
			set_freq(info, &before_d0cs, &new);

			if ((before_d0cs.vcc_core > new.vcc_core) ||
			    (before_d0cs.vcc_sram > new.vcc_sram))
				update_voltage(info, &before_d0cs, &new);
		} else {
			update_voltage(info, &old, &new);
			/* exit d0cs */
			pxa95x_exit_d0cs(info);
		}
		cur_op = new_op;
		loops_per_jiffy = new.lpj;
		return 0;
	} else if ((old.power_mode == POWER_MODE_D0CS) &&
			(new.power_mode == POWER_MODE_D0CS)) {
		cur_op = new_op;
		return 0;
	}

	if (old.core < new.core) {
		update_voltage(info, &old, &new);
	}
	if (((new.xl == 38) && (old.xl < 31))
		|| ((old.xl == 38) && (new.xl < 31))) {
		set_freq(info, &old, &pp806);
		set_freq(info, &pp806, &new);
	} else
	set_freq(info, &old, &new);

	if (old.core > new.core) {
		update_voltage(info, &old, &new);
	}
	cur_op = new_op;
	if ((new.power_mode == POWER_MODE_D0) ||
			(new.power_mode == POWER_MODE_D0CS))
		loops_per_jiffy = new.lpj;
	return 0;
}

/* function of entering low power mode */
extern void enter_lowpower_mode(int state);

static void do_freq_notify(void *driver_data, struct dvfm_freqs *freqs)
{
	struct pxa95x_dvfm_info *info = driver_data;

	dvfm_notifier_frequency(freqs, DVFM_FREQ_PRECHANGE);
	update_freq(info, freqs);
	dvfm_notifier_frequency(freqs, DVFM_FREQ_POSTCHANGE);
#ifdef CONFIG_PXA_MIPSRAM
	update_op_mips_ram(freqs->old, freqs->new);
#endif
	ispt_dvfm_op(freqs->old, freqs->new);
}

void do_lowpower_notify(void *driver_data, struct dvfm_freqs *freqs,
			       unsigned int state)
{
	dvfm_notifier_frequency(freqs, DVFM_FREQ_PRECHANGE);

	/* we will disable power for mg1 and for pv2. pm is enabled
	 * only if the PowerDisabled flag is cleared (using sys)
	 * this is possible only for PV2
	 */
	if (!PowerDisabled)
		enter_lowpower_mode(state);
	dvfm_notifier_frequency(freqs, DVFM_FREQ_POSTCHANGE);
}

static int check_op(void *driver_data, struct dvfm_freqs *freqs,
		    unsigned int new, unsigned int relation)
{
	struct op_info *p = NULL;
	struct dvfm_md_opt *q = NULL;
	int core, tmp_core = -1, found = 0;
	int first_op = -1;

	freqs->new = -1;
	if (!dvfm_find_op(new, &p)) {
		q = (struct dvfm_md_opt *) p->op;
		core = q->core;
	} else
		return -EINVAL;
	/*
	   pr_debug("%s, old:%d, new:%d, core:%d\n", __FUNCTION__, freqs->old,
	   new, core);
	 */
	read_lock(&pxa95x_dvfm_op_list.lock);
	if (relation == RELATION_LOW) {
		/* Set the lowest frequency that is higher than specifed one */
		list_for_each_entry(p, &pxa95x_dvfm_op_list.list, list) {
			q = (struct dvfm_md_opt *) p->op;
			if (core == 0) {
				/* Lowpower mode */
				if ((q->power_mode == POWER_MODE_D1)
				    || (q->power_mode == POWER_MODE_D2)
				    || (q->power_mode == POWER_MODE_CG)) {
					if (!p->device && (new == p->index)) {
						freqs->new = p->index;
						/*
						   pr_debug("%s, found op%d\n",
						   __FUNCTION__, p->index);
						 */
						break;
					}
				}
				continue;
			}

			if (!p->device && (q->core >= core)) {
				if (tmp_core == -1 || (tmp_core >= q->core)) {
					/*
					   pr_debug("%s, found op%d, core:%d\n",
					   __FUNCTION__, p->index,
					   q->core);
					 */
					if (first_op == -1)
						first_op = p->index;
					freqs->new = p->index;
					tmp_core = q->core;
					found = 1;
				}
				if (found && (new == p->index)) {
					first_op = -1;
					break;
				}
			}
		}
		if (found && (first_op != -1))
			freqs->new = first_op;
	} else if (relation == RELATION_HIGH) {
		/* Set the highest frequency that is lower than specified one */
		list_for_each_entry(p, &pxa95x_dvfm_op_list.list, list) {
			q = (struct dvfm_md_opt *) p->op;
			if (!p->device && (q->core <= core)) {
				if (tmp_core == -1 || tmp_core < q->core) {
					freqs->new = p->index;
					tmp_core = q->core;
				}
			}
		}
	} else if (relation == RELATION_STICK) {
		/* Set the specified frequency */
		list_for_each_entry(p, &pxa95x_dvfm_op_list.list, list) {
			if (!p->device && (p->index == new)) {
				freqs->new = p->index;
				break;
			}
		}
	}
	read_unlock(&pxa95x_dvfm_op_list.lock);
	if (freqs->new == -1) {
		/*
		   pr_debug("%s, Can't find op\n", __FUNCTION__);
		   pr_debug("%s, old:%d, new:%d, core:%d\n", __FUNCTION__,
		   freqs->old, new, core);
		 */
		return -EINVAL;
	}
	return 0;
}
unsigned int set_d2_from_high_pp_8787;
struct dvfm_freqs freqs_tmp;
static int pxa95x_set_op(void *driver_data, struct dvfm_freqs *freqs,
			 unsigned int new, unsigned int relation)
{
	struct pxa95x_dvfm_info *info = driver_data;
	struct dvfm_md_opt *md = NULL, *old_md = NULL;
	struct op_info *p = NULL;
	unsigned long flags;
	int ret = 1;
	unsigned int ckena = 0;

	local_fiq_disable();
	local_irq_save(flags);

/**************/
/* the following code will check if the op will like to go, is D0CS. if this is
 * the case we verify that the PM will actually allow D0CS, then we try to
 * clear the shared flag. in case the clear of the shared flag is not allowed
 * we have to prevent D0CS. we do so by calling to
 * acipc_set_constraint_no_op_change which will set the "comm awake" constraint
 * and with no op change */

	if (dvfm_find_op(freqs->old, &p))
		goto out;

	memcpy(&freqs->old_info, p, sizeof(struct op_info));

	if (dvfm_find_op(new, &p))
		goto out;

	/* verify that the requested PP is D0CS */
	if (POWER_MODE_D0CS == ((struct dvfm_md_opt *) (p->op))->power_mode) {
		memcpy(&freqs_tmp, freqs, sizeof(struct dvfm_freqs));
		if (dvfm_find_op(freqs_tmp.old, &p))
			goto out;

		memcpy(&freqs_tmp.old_info, p, sizeof(struct op_info));

		/* check if the PM will actually allow to go to D0CS */
		if (!check_op(info, &freqs_tmp, new, relation)) {
			if (!dvfm_find_op(freqs_tmp.new, &p)) {
				memcpy(&(freqs_tmp.new_info), p,
				       sizeof(struct op_info));

/* this check is to prevent an attempt to clear DDR_avail_flag if we already are
 * in D0CS because if this attempt fails (meaning there is a pending DDR_REQ
 * from the comm) we will set this flag although we are still in D0CS! this
 * situation is no allowed since it means comm can from DDR although apps is in
 * D0CS */
				if (freqs->old_info.index ==
						freqs_tmp.new_info.index)
					goto out;

				md = (struct dvfm_md_opt *) (freqs_tmp.
						new_info.op);
				if (md->power_mode == POWER_MODE_D0CS) {
#if defined(CONFIG_PXA9XX_ACIPC)
/* if we are going to D0CS we need to clear the shared flag indicating that DDR
 * available for Comm. if a DDR request is pending while clear_DDR_avail_flag is
 * trying to clear the share flag, it will not change the shared flag state and
 * return it. if the shared flag state was on we have to disable D0CS, so the
 * decision made later (in check_op) will not decide to go to D0CS this should
 * happen only if ddr_request arrive between disable interrupt in the beginning
 * of pxa95x_set_op, and the call to clear_DDR_avail_flag so it is fairy rare
 * case. */
					if (clear_DDR_avail_flag() == 1) {
						acipc_set_constraint_no_op_change();
					}
#endif
#if defined EARLY_D0CS_WAKE

#if defined(CONFIG_PXA9XX_ACIPC)
					else
#endif
					{
/* this is additional code to protect against D0CS entry near COMM wakeups. This
 * situation can cuase a delay in APPS DDR ack time if another interupt service
 * routine is running Therfore, D0CS is not allowed within a defined thrrshold
 * time of comm D2 wakeup */
						if (AppsSyncEnabled) {
							comm_wakeup_time = pxa95x_ticks_to_msec(dvfm_get_next_comm_wakeup_time());

#if defined(CONFIG_PXA9XX_ACIPC)
							/* we should not be enetering d0cs. */
							if (comm_wakeup_time < D0CS_TIMER_THRESHOLD) {
								/* we will prevent D0CS entry because comm is soon to wake up. */
								acipc_disable_d0cs_no_change();
								/* if we are not going to D0CS we must set back the ddr_avail flag to 1. */
								set_DDR_avail_flag();
							}
#endif
						}
					}
#endif
				}
				md = NULL;
				p = NULL;
			} else
				goto out;
		} else
			goto out;
	}
	ret = check_op(info, freqs, new, relation);
	if (ret)
		goto out;

	if (!dvfm_find_op(freqs->new, &p)) {
		memcpy(&(freqs->new_info), p, sizeof(struct op_info));
		/* If find old op and new op is same, skip it.
		 * At here, ret should be zero.
		 */
		if (freqs->old_info.index == freqs->new_info.index)
			goto out;
#ifdef DVFM_LP_SAFE
		md = (struct dvfm_md_opt *) (freqs->new_info.op);
		old_md = (struct dvfm_md_opt *) (freqs->old_info.op);

		/*System cannot enter D2 from D0CS */
		/*TODO: System can enter D1 from D0CS, so this limitation
		 * should be removed */
		if ((old_md->power_mode == POWER_MODE_D0CS) &&
				((md->power_mode == POWER_MODE_D1) ||
				 (md->power_mode == POWER_MODE_D2))) {
			dvfm_disable_op_name("D0CS", dvfm_dev_id);
		}

		/* System should enter D0CS or D2 from the lowest op */
		if ((md->power_mode == POWER_MODE_D0CS ||
				md->power_mode == POWER_MODE_D2) &&
				old_md->power_mode == POWER_MODE_D0 &&
				old_md->core > lowest_freq_op->core &&
				!(set_d2_from_high_pp_8787)) {
			struct dvfm_freqs temp_freqs;
			struct op_info *lowest_op_info;
			if (!dvfm_find_op(lowest_freq_index, &lowest_op_info)) {
				memcpy(&(temp_freqs.new_info),
				       lowest_op_info,
				       sizeof(struct op_info));
				memcpy(&(temp_freqs.old_info),
				       &(freqs->old_info),
				       sizeof(struct op_info));
				temp_freqs.new = lowest_freq_index;
				temp_freqs.old = freqs->old;

				/*Enter lowest op */
				do_freq_notify(info, &temp_freqs);

				/*Update the freqs info */
				memcpy(&(freqs->old_info), lowest_op_info,
				       sizeof(struct op_info));
				freqs->old = lowest_freq_index;
			}
		}

		md = (struct dvfm_md_opt *) p->op;
		ckena = CKENA;
		CKENA |= (1 << CKEN_SMC) | (1 << CKEN_NAND);
		switch (md->power_mode) {
		case POWER_MODE_D0:
		case POWER_MODE_D0CS:
			/* this means that op is forced by user for debug */
			if (ForceOP) {
				freqs->new = ForcedOPIndex;
				if (!dvfm_find_op(freqs->new, &p))
					memcpy(&(freqs->new_info), p,
					       sizeof(struct op_info));
			}
			do_freq_notify(info, freqs);
			break;
		case POWER_MODE_D1:
		case POWER_MODE_D2:
		case POWER_MODE_CG:
			do_lowpower_notify(info, freqs, md->power_mode);
			break;
		}
		/* enabling back DVFN but with no real change taking place */
		dvfm_enable_op_name_no_change("D0CS", dvfm_dev_id);
#else
		md = (struct dvfm_md_opt *) p->op;
		ckena = CKENA;
		CKENA |= (1 << CKEN_SMC) | (1 << CKEN_NAND);
		switch (md->power_mode) {
		case POWER_MODE_D0:
		case POWER_MODE_D0CS:
			/* this means that op is forced by user for debug */
			if (ForceOP) {
				freqs->new = ForcedOPIndex;
				if (!dvfm_find_op(freqs->new, &p))
					memcpy(&(freqs->new_info), p,
					       sizeof(struct op_info));
			}
			do_freq_notify(info, freqs);
			break;
		case POWER_MODE_D1:
		case POWER_MODE_D2:
		case POWER_MODE_CG:
			do_lowpower_notify(info, freqs, md->power_mode);
			break;
		}
#endif
	}

	CKENA = ckena;
	local_irq_restore(flags);
	local_fiq_enable();
	return 0;
out:
	local_irq_restore(flags);
	local_fiq_enable();
	return ret;
}

/* Put the operation of switching back to D0CS in workqueue. */
static void d0cs_worker(struct work_struct *work)
{
	dvfm_enable_op_name("D0CS", dvfm_dev_id);
}

static int pxa95x_request_op(void *driver_data, int index)
{
	struct dvfm_freqs freqs;
	struct op_info *info = NULL;
	struct dvfm_md_opt *md = NULL;
	int relation, ret;
	ret = dvfm_find_op(index, &info);
	if (ret)
		goto out;
	freqs.old = cur_op;
	freqs.new = index;
	md = (struct dvfm_md_opt *) (info->op);
	switch (md->power_mode) {
	case POWER_MODE_D1:
	case POWER_MODE_D2:
	case POWER_MODE_CG:
		relation = RELATION_STICK;
		ret = pxa95x_set_op(driver_data, &freqs, index, relation);
		break;
	default:
		relation = RELATION_LOW;
		/* only use non-low power mode as preferred op */
		ret = pxa95x_set_op(driver_data, &freqs, index, relation);
		if (!ret)
			preferred_op = index;
		break;
	}
out:
	return ret;
}

static int is_d0cs(void *driver_data)
{
	struct pxa95x_dvfm_info *info = driver_data;
	unsigned int acsr;
	/* read ACSR */
	acsr = __raw_readl(info->clkmgr_base + ACSR_OFF);
	/* Check ring oscillator status */
	if (acsr & (1 << 26))
		return 1;
	return 0;
}

/* Produce a operating point table */
static int op_init(void *driver_data, struct info_head *op_table)
{
	struct pxa95x_dvfm_info *info = driver_data;
	unsigned long flags;
	int i, index;
	struct op_info *p = NULL, *q = NULL;
	struct dvfm_md_opt *md = NULL, *smd = NULL;
	struct proc_op_array *proc = NULL;
	unsigned int lowest_freq;

	write_lock_irqsave(&op_table->lock, flags);
	for (i = 0; i < ARRAY_SIZE(proc_op_arrays); i++) {
		if (proc_op_arrays[i].cpuid == (info->cpuid & 0xfff0)) {
			if (0x6930 == proc_op_arrays[i].cpuid) {
				/* PXA935 is after PXA935H in table */
				proc = &proc_op_arrays[i + 1];
				break;
			}
			proc = &proc_op_arrays[i];
			break;
		}
	}
	if (proc == NULL) {
		printk(KERN_ERR
		       "Failed to find op tables for cpu_id 0x%08x",
		       info->cpuid);
		write_unlock_irqrestore(&op_table->lock, flags);
		return -EIO;
	} else {
		printk("initializing op table for %s\n", proc->cpu_name);
	}
	for (i = 0, index = 0; i < proc->nr_op; i++) {
		/* PXA310 A2 or PXA935/PXA940, dmcfs 60MHz in S0D0CS mode */
		if ((proc->op_array[i].power_mode == POWER_MODE_D0CS) &&
				(info->cpuid == 0x6892 || (info->cpuid & 0xFFF0)
				 == 0x6930)) {
			set_dmc60(info, 1);
			proc->op_array[i].dmcfs = 60;
		}

		/* Set index of operating point used in idle */
		if (proc->op_array[i].power_mode != POWER_MODE_D0) {
#ifdef CONFIG_IPM
			set_idle_op(index, proc->op_array[i].power_mode);
#endif
		}

		md = kzalloc(sizeof(struct dvfm_md_opt), GFP_KERNEL);
		p = kzalloc(sizeof(struct op_info), GFP_KERNEL);
		p->op = (void *) md;
		memcpy(p->op, &proc->op_array[i], sizeof(struct dvfm_md_opt));
		md->core = 13 * md->xl * md->xn;
		if (md->power_mode == POWER_MODE_D0CS)
			md->core = 60;
		p->index = index++;
		list_add_tail(&(p->list), &(op_table->list));
	}

	lowest_freq = (unsigned int) -1;
	list_for_each_entry(p, &op_table->list, list) {
		md = (struct dvfm_md_opt *) p->op;
		if (md->power_mode == POWER_MODE_D0 &&
				md->core < lowest_freq) {
			lowest_freq_op = md;
			lowest_freq_index = p->index;
			lowest_freq = md->core;
		}
	}


	md = kzalloc(sizeof(struct dvfm_md_opt), GFP_KERNEL);
	p = kzalloc(sizeof(struct op_info), GFP_KERNEL);
	p->op = (void *) md;
	if (capture_op_info(info, md)) {
		printk(KERN_WARNING "Failed to get current op setting\n");
	} else {
		def_op = 0x5a5a;	/* magic number */
		list_for_each_entry(q, &(op_table->list), list) {
			smd = (struct dvfm_md_opt *) q->op;
			md->flag = smd->flag;
			md->lpj = smd->lpj;
			md->core = smd->core;
			if (memcmp(md, smd, sizeof(struct dvfm_md_opt)) == 0) {
				def_op = q->index;
				break;
			}
		}
	}
	if (is_d0cs(driver_data))
		md->core = 60;
	else
		md->core = 13 * md->xl * md->xn;
	md->lpj = loops_per_jiffy;
	md->flag = OP_FLAG_BOOT;
	sprintf(md->name, "BOOT OP");

	smd = kzalloc(sizeof(struct dvfm_md_opt), GFP_KERNEL);
	q = kzalloc(sizeof(struct op_info), GFP_KERNEL);
	memcpy(q, p, sizeof(struct op_info));
	memcpy(smd, md, sizeof(struct dvfm_md_opt));
	smd->core = md->core;
	smd->lpj = md->lpj;
	smd->flag = OP_FLAG_USER_DEFINED;
	sprintf(smd->name, "CUSTOM OP");
	q->op = (void *) smd;
	/* Add CUSTOM OP into op list */
	q->index = index++;
	list_add_tail(&q->list, &op_table->list);
	/* Add BOOT OP into op list */
	p->index = index++;
	preferred_op = p->index;
	list_add_tail(&p->list, &op_table->list);
	/* BOOT op */
	if (def_op == 0x5a5a) {
		cur_op = p->index;
		def_op = p->index;
	} else
		cur_op = def_op;
	pr_debug("%s, def_op:%d, cur_op:%d\n", __func__, def_op, cur_op);

	/* set the operating point number */
	op_nums = proc->nr_op + 2;

	pr_debug("Current Operating Point is %d\n", cur_op);
	dump_op_list(info, op_table, OP_FLAG_ALL);
	write_unlock_irqrestore(&op_table->lock, flags);

	return 0;
}

/*
 * The machine operation of dvfm_enable
 */
static int pxa95x_enable_dvfm(void *driver_data, int dev_id)
{
	struct pxa95x_dvfm_info *info = driver_data;
	struct dvfm_md_opt *md = NULL;
	struct op_info *p = NULL;
	int i, num;
	num = get_op_num(info, &pxa95x_dvfm_op_list);
	for (i = 0; i < num; i++) {
		if (!dvfm_find_op(i, &p)) {
			md = (struct dvfm_md_opt *) p->op;
			if (md->core < MIN_SAFE_FREQUENCY)
				dvfm_enable_op(i, dev_id);
		}
	}
	ispt_block_dvfm(0, dev_id);
	return 0;
}

/*
 * The mach operation of dvfm_disable
 */
static int pxa95x_disable_dvfm(void *driver_data, int dev_id)
{
	struct pxa95x_dvfm_info *info = driver_data;
	struct dvfm_md_opt *md = NULL;
	struct op_info *p = NULL;
	int i, num;
	num = get_op_num(info, &pxa95x_dvfm_op_list);
	for (i = 0; i < num; i++) {
		if (!dvfm_find_op(i, &p)) {
			md = (struct dvfm_md_opt *) p->op;
			if (md->core < MIN_SAFE_FREQUENCY)
				dvfm_disable_op(i, dev_id);
		}
	}
	ispt_block_dvfm(1, dev_id);
	return 0;
}

static int pxa95x_enable_op(void *driver_data, int index, int relation)
{
	/*
	 * Restore preferred_op. Because this op is sugguested by policy maker
	 * or user.
	 */
	return pxa95x_request_op(driver_data, preferred_op);
}

static int pxa95x_disable_op(void *driver_data, int index, int relation)
{
	struct dvfm_freqs freqs;
	int ret = 0, the_other_relation;
	if (RELATION_HIGH == relation)
		the_other_relation = RELATION_LOW;
	else if (RELATION_LOW == relation)
		the_other_relation = RELATION_HIGH;
	else {
		pr_err("%s: only support RELATION_LOW and RELATION_HIGH"
			       "relation when disable op.\n", __func__);
		return -EINVAL;
	}

	// for syssleep production (disable set LPM constarint while sys sleep test)
	if(suspend_forbidden) {
	  printk(KERN_ERR "%s try request %d constarint but suspend_forbidden\n", __FUNCTION__, index);
	  return 0;
	}

	if (cur_op == index) {
		freqs.old = index;
		freqs.new = -1;
		/* using the relation base on parameter, if it failed, try
		 * the other relation.
		 */
		ret = dvfm_set_op(&freqs, freqs.old, relation);
		if (ret) {
			ret = dvfm_set_op(&freqs, freqs.old,
					the_other_relation);
		}

		if (index == cur_op) {
			/*Which means that all ops are unavailable!*/
			printk(KERN_ERR "pxa3xx_disable_op::Unable to disable op %d\n", index);
			WARN_ON(!DvfmDisabled);
		}
	}
	return ret;
}

static int pxa95x_volt_show(void *driver_data, char *buf)
{
	struct dvfm_md_opt new;
	int len = 0;
	struct pxa95x_dvfm_info *info = driver_data;

	if (info->flags & PXA95x_USE_POWER_I2C) {
		printk(KERN_ERR "volt show is not supported when power i2c used!\n");
		return 0;
	}

	memset(&new, 0, sizeof(struct dvfm_md_opt));
	new.vcc_core = pxa95x_dvfm_get_core_voltage();
	len = sprintf(buf, "core voltage:%dmv\n", new.vcc_core);
	return len;
}

static unsigned int pxa95x_ticks_to_msec(unsigned int ticks)
{
	return (ticks * 5 * 5 * 5) >> 12;
}

#ifdef CONFIG_PXA95x_DVFM_STATS
/* Convert ticks from 32K timer to microseconds */
static unsigned int pxa95x_ticks_to_usec(unsigned int ticks)
{
	return (ticks * 5 * 5 * 5 * 5 * 5 * 5) >> 9;
}

static unsigned int pxa95x_ticks_to_sec(unsigned int ticks)
{
	return ticks >> 15;
}

static unsigned int pxa95x_read_time(void)
{
	return OSCR4;
}

/* It's invoked by PM functions.
 * PM functions can store the accurate time of entering/exiting low power
 * mode.
 */
int calc_switchtime(unsigned int end, unsigned int start)
{
	switch_lowpower_before = end;
	switch_lowpower_after = start;
	return 0;
}

static int pxa95x_stats_notifier_freq(struct notifier_block *nb,
				      unsigned long val, void *data)
{
	struct dvfm_freqs *freqs = (struct dvfm_freqs *) data;
	struct op_info *info = &(freqs->new_info);
	struct dvfm_md_opt *md = NULL;
	unsigned int ticks;

	ticks = pxa95x_read_time();
	md = (struct dvfm_md_opt *) (info->op);
	if (md->power_mode == POWER_MODE_D0 ||
	    md->power_mode == POWER_MODE_D0CS) {
		switch (val) {
		case DVFM_FREQ_PRECHANGE:
			calc_switchtime_start(freqs->old, freqs->new,
					      ticks);
			break;
		case DVFM_FREQ_POSTCHANGE:
			/* Calculate the costed time on switching frequency */
			calc_switchtime_end(freqs->old, freqs->new, ticks);
			dvfm_add_event(freqs->old, CPU_STATE_RUN,
				       freqs->new, CPU_STATE_RUN);
			dvfm_add_timeslot(freqs->old, CPU_STATE_RUN);
#ifdef CONFIG_IPM
			mspm_add_event(freqs->old, CPU_STATE_RUN);
#endif
			break;
		}
	} else if (md->power_mode == POWER_MODE_D1 ||
		   md->power_mode == POWER_MODE_D2 ||
		   md->power_mode == POWER_MODE_CG) {
		switch (val) {
		case DVFM_FREQ_PRECHANGE:
			calc_switchtime_start(freqs->old, freqs->new,
					      ticks);
			/* Consider lowpower mode as idle mode */
			dvfm_add_event(freqs->old, CPU_STATE_RUN,
				       freqs->new, CPU_STATE_IDLE);
			dvfm_add_timeslot(freqs->old, CPU_STATE_RUN);
#ifdef CONFIG_IPM
			mspm_add_event(freqs->old, CPU_STATE_RUN);
#endif
			break;
		case DVFM_FREQ_POSTCHANGE:
			/* switch_lowpower_start before switch_lowpower_after
			 * is updated in calc_switchtime().
			 * It's invoked in pm function.
			 */
			calc_switchtime_end(freqs->old, freqs->new,
					    switch_lowpower_before);
			calc_switchtime_start(freqs->new, freqs->old,
					      switch_lowpower_after);
			calc_switchtime_end(freqs->new, freqs->old, ticks);
			dvfm_add_event(freqs->new, CPU_STATE_IDLE,
				       freqs->old, CPU_STATE_RUN);
			dvfm_add_timeslot(freqs->new, CPU_STATE_IDLE);
#ifdef CONFIG_IPM
			mspm_add_event(freqs->new, CPU_STATE_IDLE);
#endif
			break;
		}
	}
	return 0;
}
#else
#define pxa95x_ticks_to_usec	NULL
#define pxa95x_ticks_to_sec	NULL
#define pxa95x_read_time	NULL
#endif

static struct dvfm_driver pxa95x_driver = {
	.count = get_op_num,
	.set = pxa95x_set_op,
	.dump = dump_op,
	.name = get_op_name,
	.request_set = pxa95x_request_op,
	.enable_dvfm = pxa95x_enable_dvfm,
	.disable_dvfm = pxa95x_disable_dvfm,
	.enable_op = pxa95x_enable_op,
	.disable_op = pxa95x_disable_op,
	.volt_show = pxa95x_volt_show,
	.ticks_to_usec = pxa95x_ticks_to_usec,
	.ticks_to_sec = pxa95x_ticks_to_sec,
	.read_time = pxa95x_read_time,
};

#ifdef CONFIG_PM
static int pxa95x_freq_suspend(struct platform_device *pdev,
			       pm_message_t state)
{
	return 0;
}

static int pxa95x_freq_resume(struct platform_device *pdev)
{
	return 0;
}
#else
#define pxa95x_freq_suspend    NULL
#define pxa95x_freq_resume     NULL
#endif

#ifdef CONFIG_PXA_MIPSRAM
int update_op_mips_ram(u32 old_pp, u32 new_pp)
{
	u32 data, d2_constraints, d0cs_constraints;
	struct op_info *p = NULL;

	if (old_pp != new_pp) {
		switch (new_pp) {
		case 0:
			data = PP_D0CS_MIPS_RAM;
			break;
		case 1:
			data = PP_PP1_MIPS_RAM;
			break;
		case 2:
			data = PP_PP2_MIPS_RAM;
			break;
		case 3:
			data = PP_PP3_MIPS_RAM;
			break;
		case 4:
			data = PP_PP4_MIPS_RAM;
			break;
		case 5:
			data = PP_PP5_MIPS_RAM;
			break;
		case 9:
			data = PP_PP_CUSTOM_MIPS_RAM;
			break;
		case 10:
			data = PP_PP_BOOT_MIPS_RAM;
			break;
		default:
			data = INVALID_PP_NUMBER_MIPS_RAM;
			break;
		}
		MIPS_RAM_ADD_PP_CHANGE(data);
		dvfm_get_opinfo(0, &p);
		d0cs_constraints = p->device & 0x0000FFFF;
		MIPS_RAM_ADD_TRACE(MIPSRAM_EVENT_RAW_DATA |
				   d0cs_constraints);
		d0cs_constraints = (p->device & 0xFFFF0000) >> 16;
		MIPS_RAM_ADD_TRACE(MIPSRAM_EVENT_RAW_DATA |
				   d0cs_constraints);
		dvfm_get_opinfo(7, &p);
		d2_constraints = p->device & 0x0000FFFF;
		MIPS_RAM_ADD_TRACE(MIPSRAM_EVENT_RAW_DATA |
				   d2_constraints);
		d2_constraints = (p->device & 0xFFFF0000) >> 16;
		MIPS_RAM_ADD_TRACE(MIPSRAM_EVENT_RAW_DATA |
				   d2_constraints);

	}
	return 0;

}

#endif
int dvfm_toggle_gpio(int OnOff)
{
	if (OnOff) {
		/* set gpio high */
		__raw_writel(0x20000000, (void *) &(__REG(0x40e0001c)));
	} else {
		/* setting gpio low */
		__raw_writel(0x20000000, (void *) &(__REG(0x40e00028)));
	}
	return 0;
}

typedef struct {
	unsigned long data_arg1;
	unsigned long data_arg2;
} data_struct;
static struct timer_list cpu_load_simulate_timer;
unsigned int load_cycle_time;
unsigned int first_call = 1;

static void cpu_simulate_load_cbk(unsigned long data)
{
	unsigned int timestamp1, timestamp2;
	unsigned int load_time = data;

	mod_timer(&cpu_load_simulate_timer, jiffies + load_cycle_time);
	timestamp1 = __raw_readl((void *) &(__REG(0x40A00040)));
	timestamp2 = timestamp1;

	while ((timestamp2 - timestamp1) < load_time) {
		timestamp2 = __raw_readl((void *) &(__REG(0x40A00040)));
	}

}

int power_debug_dev_idx;
int gpio_state;
int dvfm_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
		unsigned long arg)
{
	unsigned long val;
	int disable_idx = -1, rc, dev_count;
	struct op_info *p = NULL;
	struct dvfm_trace_info *ptr = NULL;

	struct dvfm_md_opt new;

	data_struct reg_data;

	switch (cmd) {
		int err;
	case DVFM_FORCE_PP:
		err = copy_from_user(&reg_data, (data_struct *) arg,
				sizeof(data_struct));
		if (reg_data.data_arg1 == UNDO_FORCE_PP) {
			ForceOP = 0;
			printk(KERN_WARNING "PP is unforced\n");
		} else {
			ForceOP = 1;
			ForcedOPIndex = reg_data.data_arg1;
			printk(KERN_WARNING "PP is forced to %ld\n",
			       reg_data.data_arg1);

		}
		break;

	case FORCE_C0:
		err = copy_from_user(&reg_data, (data_struct *) arg,
				sizeof(data_struct));
		if (reg_data.data_arg1 == 0) {
			ForceC0 = 0;
			printk(KERN_WARNING "ForceC0 is unforced\n");
		} else {
			ForceC0 = 1;
			printk(KERN_WARNING "ForceC0 is forced\n");
		}
		break;

	case VCTCXO_FORCE_ON:
		err = copy_from_user(&reg_data, (data_struct *) arg,
				sizeof(data_struct));
		if (reg_data.data_arg1) {
			ForceVCTCXO_EN = 1;
		} else {
			ForceVCTCXO_EN = 0;
		}
		break;

	case ENABLE_D2_VOLTAGE_CHANGE:
		err = copy_from_user(&reg_data, (data_struct *) arg,
				sizeof(data_struct));
		if (reg_data.data_arg1) {
			EnableD2VoltageChange = 1;
			D2voltageLevelValue = reg_data.data_arg1;

		} else {
			EnableD2VoltageChange = 0;
		}

		break;

	case DVFM_FORCE_D2:
		printk(KERN_WARNING "********!!Seting endless D2!!********\n");
		ForceLPM = PXA9xx_Force_D2;
		LastForceLPM = PXA9xx_Force_None;
		ForceLPMWakeup = 0;
		break;

	case DVFM_FORCE_D2_WAKEUP_SELECT:
		reg_data.data_arg1 = 0;
		reg_data.data_arg2 = 0;
		err = copy_from_user(&reg_data, (data_struct *) arg,
				sizeof(data_struct));
		if (!err) {
			ForceLPM = PXA9xx_Force_D2;
			LastForceLPM = PXA9xx_Force_None;
			ForceLPMWakeup = (unsigned int) reg_data.data_arg1;
			//enable_deepidle|= (IDLE_D2|IDLE_D1|IDLE_CG);
			wifi_set_power(0);

			/* close codec */
			pm860x_codec_reg_write(0xe1,0x0);
			pm860x_codec_reg_write(0x42,0x4);
			pm860x_codec_reg_write(0xdc,0x0);
			
						
			rc = dvfm_find_index("ACIPC", &disable_idx);
			if (rc != 0) {
				printk(KERN_ERR "%s (%d) failed get ACIPC index from table rc = %d\n", __func__, __LINE__, rc);
				BUG_ON(1);
				break;
			}			
	                for(dev_count = 0 ; dev_count < 32;dev_count++)
			{
				if (dev_count != disable_idx) {
					dvfm_enable_op_name("D2", dev_count);
				}
			}


#ifdef CONFIG_DEBUG_FS
			printk(KERN_WARNING "Next LPM is %s,\t next LPM source"
					"wakeups is 0x%x\n",
					pxa9xx_force_lpm_names__[(unsigned long)
					ForceLPM],
					ForceLPMWakeup);
#endif
		} else
			printk(KERN_WARNING "Wrong paremter size\n");
		break;

	case GET_SUSPEND_STATE:
#ifdef CONFIG_PXA95x_SUSPEND
		get_full_suspend_state((unsigned int *)&reg_data.data_arg1);
		err = copy_to_user((data_struct *) arg, &reg_data,
				sizeof(data_struct));
#endif
		break;
	case FORCE_SUSPEND_STATE:
#ifdef CONFIG_PXA95x_SUSPEND
		/* remove all active wake locks before request suspend*/
		force_remove_active_locks();
#endif
		break;
	case GET_CURRENT_PP:
		printk(KERN_WARNING "current PP is %d\n", cur_op);
		val = __raw_readl((void *) &ACSR);
		printk(KERN_WARNING "ACSR value is 0x%x\n",
		       (unsigned int) val);
		val = __raw_readl((void *) &(__REG(0x4134000C)));
		printk(KERN_WARNING "D0CKEN_A value is 0x%x\n",
		       (unsigned int) val);
		val = __raw_readl((void *) &(__REG(0x41340010)));
		printk(KERN_WARNING "D0CKEN_B value is 0x%x\n",
		       (unsigned int) val);
		val = __raw_readl((void *) &(__REG(0x41340024)));
		printk(KERN_WARNING "D0CKEN_C value is 0x%x\n",
		       (unsigned int) val);
		list_for_each_entry(p, &pxa95x_dvfm_op_list.list, list) {
			memcpy(&new, (struct dvfm_md_opt *) p->op,
					sizeof(struct dvfm_md_opt));
			printk(KERN_WARNING "PP index number %d is %s and it"
					"constrain status is %x\n",
					p->index, new.name, p->device);

		}
		list_for_each_entry(ptr, &dvfm_trace_list.list, list) {
			printk(KERN_WARNING "device index %d is %s\n",
					ptr->index, ptr->name);
		}

		break;
	case TOGGLE_GPIO:
		/* setting MFPR configuration to alternate function 0 */
		val = __raw_readl((void *) &(__REG(0x40e102b0)));
		val &= 0xfffffff8;
		__raw_writel(val, (void *) &(__REG(0x40e102b0)));

		/* setting GSDR3 to configure gpio 61 as output */
		val = __raw_readl((void *) &(__REG(0x40e0010C)));
		__raw_writel(0x20000000, (void *) &(__REG(0x40e00404)));
		val = __raw_readl((void *) &(__REG(0x40e0010C)));
		printk(KERN_WARNING "MFPR value is 0x%x\n", (unsigned int) val);

		/* setting GPSR3,GPCR3 to set the data of the pin to 1<->0 */
		if (gpio_state) {
			/* set gpio high */
			dvfm_toggle_gpio(1);
			gpio_state = 0;
		} else {
			/* set gpio low */
			dvfm_toggle_gpio(0);
			gpio_state = 1;
		}
		break;

	case RESET_COMM:
		val = __raw_readl((void *) &(__REG(0x40f5001c)));
		/* clearing bit 0 to reset the comm. */
		val &= 0xfffffffe;
		__raw_writel(val, (void *) &(__REG(0x40f5001c)));
		rc = dvfm_find_index("ACIPC", &disable_idx);
		if (rc != 0) {
			printk(KERN_ERR "%s (%d) failed get ACIPC index from"
					"table rc = %d\n", __func__, __LINE__,
					rc);
			BUG_ON(1);
			break;
		}
		dvfm_enable_op_name("D2", disable_idx);
		rc = dvfm_find_index("SHMEM", &disable_idx);
		if (rc != 0) {
			printk(KERN_ERR "%s (%d) failed get SHMEM index from"
					"table rc = %d\n", __func__, __LINE__,
					rc);
			BUG_ON(1);
			break;
		}
		dvfm_enable_op_name("D2", disable_idx);
		break;

	case POWER_DISABLE:
		err = copy_from_user(&reg_data, (data_struct *) arg,
				sizeof(data_struct));
		if (reg_data.data_arg1 == 1)
			PowerDisabled = 1;
		else
			PowerDisabled = 0;
		if (reg_data.data_arg2 == 1)
			DvfmDisabled = 1;
		else
			DvfmDisabled = 0;
		break;

	case DEBUG_DRIVER_REGISTER:
		dvfm_register("POWER_DEBUG", &power_debug_dev_idx);
		break;
	case DEBUG_DRIVER_UNREGISTER:
		dvfm_unregister("POWER_DEBUG", &power_debug_dev_idx);
		break;

	case DEBUG_REMOVE_PP1_REQ:
		dvfm_enable_op(1, power_debug_dev_idx);
		break;

	case DEBUG_REMOVE_PP3_REQ:
		dvfm_enable_op(2, power_debug_dev_idx);
		dvfm_enable_op(1, power_debug_dev_idx);
		dvfm_enable_op(0, power_debug_dev_idx);
		break;
	case DEBUG_REMOVE_PP2_REQ:
		dvfm_enable_op(1, power_debug_dev_idx);
		dvfm_enable_op(0, power_debug_dev_idx);
		break;
	case STRESS_TEST:
		err = copy_from_user(&reg_data, (data_struct *) arg,
				sizeof(data_struct));
		dvfm_disable_op(reg_data.data_arg1, power_debug_dev_idx);
		dvfm_enable_op(reg_data.data_arg1, power_debug_dev_idx);
		break;

	case DEBUG_DRIVER_DISABLE_PP:
		err = copy_from_user(&reg_data, (data_struct *) arg,
				sizeof(data_struct));
		dvfm_disable_op(reg_data.data_arg1, power_debug_dev_idx);
		break;

	case DEBUG_DRIVER_ENABLE_PP:
		err = copy_from_user(&reg_data, (data_struct *) arg,
				sizeof(data_struct));
		dvfm_enable_op(reg_data.data_arg1, power_debug_dev_idx);
		break;

	case DEVICE_DRIVER_DISABLE_PP:
		err = copy_from_user(&reg_data, (data_struct *) arg,
				sizeof(data_struct));
		dvfm_disable_op(reg_data.data_arg1, reg_data.data_arg2);
		break;

	case DEVICE_DRIVER_ENABLE_PP:
		err = copy_from_user(&reg_data, (data_struct *) arg,
				sizeof(data_struct));
		dvfm_enable_op(reg_data.data_arg1, reg_data.data_arg2);
		break;

	case GET_REG_VALUE:
		err = copy_from_user(&reg_data, (data_struct *) arg,
				sizeof(data_struct));
		reg_data.data_arg2 = __raw_readl(
				(void *) &(__REG(reg_data.data_arg1)));
		err = copy_to_user((data_struct *) arg, &reg_data,
				sizeof(data_struct));
		break;
	case SET_REG_VALUE:
		err = copy_from_user(&reg_data, (data_struct *) arg,
				sizeof(data_struct));
		__raw_writel(reg_data.data_arg2,
				(void *) &(__REG(reg_data.data_arg1)));
		err = copy_to_user((data_struct *) arg, &reg_data,
				sizeof(data_struct));
		break;
	case SET_CPU_LOAD:
		err = copy_from_user(&reg_data, (data_struct *) arg,
				sizeof(data_struct));
		if (!first_call) {
			del_timer_sync(&cpu_load_simulate_timer);
		}
		first_call = 0;
		init_timer(&cpu_load_simulate_timer);
		cpu_load_simulate_timer.function = cpu_simulate_load_cbk;
		/* calculating the load time in 32Khz clocks. */
		cpu_load_simulate_timer.data = (reg_data.data_arg1 *
				reg_data.data_arg2) / 100;
		/* turning 32Khz ticks into jiffies */
		load_cycle_time = pxa95x_ticks_to_msec(msecs_to_jiffies(
					reg_data.data_arg2));
		printk(KERN_WARNING "setting cpu simulation. percentege is %ld,"
			       "cycle time is %d\n",
			       cpu_load_simulate_timer.data, load_cycle_time);
		mod_timer(&cpu_load_simulate_timer, jiffies + load_cycle_time);
		break;

		/* Zero all the counters for debugging LPM Task. Set the offset
		 * to the latest value */
	case DEBUG_MSPM_START_LPM_DEBUG_COUNT:
		memcpy(&DVFMLPMGlobalCountOffset, &DVFMLPMGlobalCount,
				sizeof(DVFMLPMGlobalCountOffset));
		printk("DVFM Counters Reset: D2=%lu CGM=%lu D0CS_Enter=%lu"
			       "D0CS_exit= %lu\n",
			       DVFMLPMGlobalCountOffset.D2_Enter_Exit_count,
			       DVFMLPMGlobalCountOffset.CGM_Enter_Exit_count,
			       DVFMLPMGlobalCountOffset.D0CS_Enter_count,
			       DVFMLPMGlobalCountOffset.D0CS_Exit_count);
		break;
		/*Print the Counters */
	case DEBUG_MSPM_GET_LPM_DEBUG_COUNT:
		printk("DVFM Counters: D2=%lu CGM=%lu D0CS_Enter=%lu"
				"D0CS_exit=%lu D0C1_Enter_count=%lu\n",
				DVFMLPMGlobalCount.D2_Enter_Exit_count -
				DVFMLPMGlobalCountOffset.D2_Enter_Exit_count,
				DVFMLPMGlobalCount.CGM_Enter_Exit_count -
				DVFMLPMGlobalCountOffset.CGM_Enter_Exit_count,
				DVFMLPMGlobalCount.D0CS_Enter_count -
				DVFMLPMGlobalCountOffset.D0CS_Enter_count,
				DVFMLPMGlobalCount.D0CS_Exit_count -
				DVFMLPMGlobalCountOffset.D0CS_Exit_count,
				DVFMLPMGlobalCount.D0C1_Enter_count);
		break;
	case FORCE_LPM:
		err = copy_from_user(&reg_data, (data_struct *) arg,
				sizeof(data_struct));
		if (!err) {
			if (reg_data.data_arg1 <
			    ((unsigned long) PXA9xx_Force_count)) {
				ForceLPM = (enum pxa9xx_force_lpm)
					reg_data.data_arg1;
				LastForceLPM = PXA9xx_Force_None;
				ForceLPMWakeup = reg_data.data_arg2;
#ifdef CONFIG_DEBUG_FS
				printk(KERN_WARNING "Next LPM is %s,\t next LPM"
						"source wakeups is 0x%x\n",
						pxa9xx_force_lpm_names__[
						(unsigned int) ForceLPM],
						ForceLPMWakeup);
#endif
			} else {
				printk(KERN_ERR "[0x%x,0x%x]Unrecognize LPM\n",
						(unsigned int) reg_data.data_arg1,
						(unsigned int) reg_data.data_arg2);
			}
		} else
			printk(KERN_ERR "Wrong paremeter size\n");
		break;
	}
	return 0;
}

static const struct file_operations dvfm_fops = {
      .owner = THIS_MODULE,
      .ioctl = dvfm_ioctl,
};


static struct miscdevice dvfm_misc_device = {
      .name = "dvfm",
      .fops = &dvfm_fops,
      .minor = 201,
};

static unsigned long uboot_AvcrValue;
static int SetAvcrFlag;

static void pxa95x_poweri2c_init(struct pxa95x_dvfm_info *info)
{
	uint32_t pcfr, pvcr, sdcr;

	if (info->flags & PXA95x_USE_POWER_I2C) {
		/* set AVCR for PXA935/PXA940:
		 *      level 0: 1250mv, 0x15
		 *      level 1: 1250mv, 0x15
		 *      level 2: 1250mv, 0x15
		 *      level 3: 1250mv, 0x15
		 */
		/* Do not modify AVCR on PV2: OBM sets it so the chip works */
		/*
		   avcr = __raw_readl(info->spmu_base + AVCR_OFF);
		   avcr &= 0xE0E0E0E0;
		   avcr |= (0x17 << 24) | (0x15 << 16) | (0x15 << 8) | 0x0f;
		   __raw_writel(avcr, info->spmu_base + AVCR_OFF);
		   avcr = __raw_readl(info->spmu_base + AVCR_OFF);
		 */
		/* set delay */
		pcfr = __raw_readl(info->spmu_base + PCFR_OFF);
		pcfr &= 0xF00FFFFF;
		pcfr |= 0x00100000;
		/* Disable pullup/pulldown in PWR_SCL and PWR_SDA */
		pcfr |= 0x04;
		pcfr |= 0x100;

		/* this will enable the writes by setting WRM bits. */
		pcfr |= 0x00028000;

		__raw_writel(pcfr, info->spmu_base + PCFR_OFF);
		pcfr = __raw_readl(info->spmu_base + PCFR_OFF);



		/* enable FVE, PVE bit */
		pvcr = __raw_readl(info->spmu_base + PVCR_OFF);
		__raw_writel(pvcr | (0x3<<30), info->spmu_base + PVCR_OFF);

		/* sdcr = __raw_readl(info->spmu_base + SDCR_OFF); */

		/* turning on bit 7 (EN bit) and turing off bit 6 (VREN)
		 * and AVL. */
		sdcr = 0x00000080;
		__raw_writel(sdcr, info->spmu_base + SDCR_OFF);
	} else {
		/* disable FVE,PVE,TVE,FVC bit */
		pvcr = __raw_readl(info->spmu_base + PVCR_OFF);
		pvcr &= 0x0fffffff;
		__raw_writel(pvcr, info->spmu_base + PVCR_OFF);
	}
	/* if AVCR voltage level are configured by uboot parameter they will
	 * be set here */
	if (SetAvcrFlag)
		__raw_writel(uboot_AvcrValue, info->spmu_base + AVCR_OFF);
}

static void pxa95x_df_init(struct pxa95x_dvfm_info *info)
{
	unsigned int memclkcfg;
	/* unsigned int accr; */

	/* initialize the DF-DIC to 2 -
	 * this is a constant value for TAVOR PV2 */
	memclkcfg = __raw_readl(info->smc_base + MEMCLKCFG_OFF);
	memclkcfg |= (2 << MEMCLKCFG_DF_OFFSET);
	__raw_writel(memclkcfg, info->smc_base + MEMCLKCFG_OFF);

	/* accr = __raw_readl(info->smc_base + ACCR_OFF);
	   accr &= ~(ACCR_DMCFS_312_OFFSET);
	   __raw_writel(accr, info->clkmgr_base + ACCR_OFF); */
}

static void pxa95x_d0cs_wakeup_init(struct pxa95x_dvfm_info *info)
{
	unsigned int aicsr = 0;
	aicsr = __raw_readl(info->clkmgr_base + AICSR_OFF);
	aicsr &= ~AICSR_STATUS_BITS;
	/*enabling WED0CS bit */
	aicsr |= AICSR_WED0CS;
	/* Enabling AICSR wakeup from D0CS */
	__raw_writel(aicsr, info->clkmgr_base + AICSR_OFF);
}

#ifdef CONFIG_PMIC_D1980
extern void d1980_pi2c_reset(void);
#endif

static int pxa95x_freq_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct pxa95x_freq_mach_info *pdata;
	struct pxa95x_dvfm_info *info;
	int rc, user_index = -1;

	/* initialize the information necessary to frequency/voltage change
	 * operation */
	pdata = pdev->dev.platform_data;
	pxa95x_driver.priv = kzalloc(sizeof(struct pxa95x_dvfm_info),
			GFP_KERNEL);
	info = pxa95x_driver.priv;
	info->flags = pdata->flags;
	info->cpuid = read_cpuid(0) & 0xFFFF;

	if (!(pdata->flags & PXA95x_USE_POWER_I2C)) {
#ifdef CONFIG_PMIC_D1980
        d1980_pi2c_reset();
	    v_buck1 = regulator_get(NULL, "D1980_BUCK1");
#else
		v_buck1 = regulator_get(&pdev->dev, "v_buck1");
#endif
		if (IS_ERR(v_buck1))
			goto err;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
					 "clkmgr_regs");
	if (!res)
		goto err;
	info->clkmgr_base = ioremap(res->start, res->end - res->start + 1);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
					 "spmu_regs");
	if (!res)
		goto err;
	info->spmu_base = ioremap(res->start, res->end - res->start + 1);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
					 "bpmu_regs");
	if (!res)
		goto err;
	info->bpmu_base = ioremap(res->start, res->end - res->start + 1);

	if (!cpu_is_pxa970()) {
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dmc_regs");
	if (!res)
		goto err;
	info->dmc_base = ioremap(res->start, res->end - res->start + 1);
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "smc_regs");
	if (!res)
		goto err;
	info->smc_base = ioremap(res->start, res->end - res->start + 1);

	pxa95x_df_init(info);
	pxa95x_d0cs_wakeup_init(info);
	addr_trim_value_wa = ioremap(0x58110000, 0x30);

	pxa95x_poweri2c_init(info);
	op_init(info, &pxa95x_dvfm_op_list);

	init_timer(&d0cs_prevent_timer);
	d0cs_prevent_timer.function = pxa95x_d0cs_prevent_cbk;
	d0cs_prevent_timer.data = 0;

	rc = dvfm_register_driver(&pxa95x_driver, &pxa95x_dvfm_op_list);
	if (disabe_high_pp_on_low_voltage_board() != 0)
		goto err;

	CKENA &= ~(1 << CKEN_SMC | 1 << CKEN_NAND);
	rc = dvfm_find_index("User", &user_index);
	if (!rc) {
		rc |= dvfm_disable_op_name("BOOT OP", user_index);
		rc |= dvfm_disable_op_name("CUSTOM OP", user_index);
		rc |= dvfm_disable_op_name("D0CS", user_index);
		rc |= dvfm_disable_op_name("208M_HF", user_index);
		rc |= dvfm_disable_op_name("416M_VGA", user_index);
		/* Blocking 806M until overheating intterupt is enable at later init stage*/
		rc |= dvfm_disable_op_name("806M", user_index); 
		if (cpu_is_pxa970()) {
			rc |= dvfm_disable_op_name("156M", user_index);
			rc |= dvfm_disable_op_name("D1", user_index);
			rc |= dvfm_disable_op_name("D2", user_index);
			rc |= dvfm_disable_op_name("CG", user_index);
		}
		if (rc)
			printk(KERN_ERR "Error disable op\n");
#if defined(CONFIG_PXA95x_1GHZ_SUPPORT)
		/* This OP should be enabled for MG1-D0 and MG2-B0.*/
		if (cpu_is_pxa955_Cx() || cpu_is_pxa968_Ax() ||
			(!alvl3HighVoltage))
				rc |= dvfm_disable_op_name("988M", user_index);
		/* We do not read the AVCR reg and rely on OBM value
		 * because higher value of each ALVL is write only access*/
		if (uboot_DefaultAvcrValue)
			alvl3LowVoltage = uboot_DefaultAvcrValue;
		else /*backward compatibility*/
			alvl3LowVoltage = uboot_DefaultAvcrValue =
				__raw_readl(info->spmu_base + AVCR_OFF);
		alvl3LowVoltage = ((alvl3LowVoltage & AVCR_ALVL3_MASK) >> AVCR_ALVL3_OFFSET);
		if (rc) {
			printk(KERN_ERR "Error initiliaze 988Mhz pp\n");
			goto err;
		}
#endif
	} else {
		printk(KERN_ERR "pxa9xx_freq_probe:Unable to find 'User' driver\n");
		BUG_ON(1);
	}
	return rc;
err:
	printk(KERN_ERR "pxa95x_dvfm init failed\n");
	CKENA &= ~(1 << CKEN_SMC | 1 << CKEN_NAND);
	kzfree(pxa95x_driver.priv);
	return -EIO;
}

static int pxa95x_freq_remove(struct platform_device *pdev)
{
	if (!IS_ERR(v_buck1))
		regulator_put(v_buck1);

	kfree(pxa95x_driver.priv);
	return dvfm_unregister_driver(&pxa95x_driver);
}

static struct platform_driver pxa95x_freq_driver = {
	.driver = {
		   .name = "pxa95x-freq",
		   },
	.probe = pxa95x_freq_probe,
	.remove = pxa95x_freq_remove,
#ifdef CONFIG_PM
	.suspend = pxa95x_freq_suspend,
	.resume = pxa95x_freq_resume,
#endif
};

int d2_led_toggle_flag;
static int d2_led_toggle_proc(char *page, char **start, off_t off,
			      int count, int *eof, void *data)
{
	d2_led_toggle_flag = 1;
	return 0;
}

/* the next functions are handling uboot parmetres */
static int temp_PowerDisabled;
static int temp_DvfmDisabled;
static int uboot_disablePower(char *s)
{
	unsigned long data;

	if (strict_strtoul(s, 10, &data))
		pr_err("Wrong value assigned to disable power!\n");

	/* on seeing "tavorcfg_nolpm", disable power */
	if (data == 0)
		temp_PowerDisabled = 0;
	else
		temp_PowerDisabled = 1;
	return 1;
}

static int uboot_disableDvfm(char *s)
{
	unsigned long data;
	if (strict_strtoul(s, 10, &data))
		pr_err("Wrong value assigned to disable DVFM!\n");
	/* on seeing "tavorcfg_nodvfm", disable dvfm */
	if (data == 0)
		temp_DvfmDisabled = 0;
	else
		temp_DvfmDisabled = 1;
	return 1;
}

static int uboot_defaultAvcr(char *s)
{
	/* This value is AVCR which is
	 * set at OBM stage. we need to keep this
	 * value as shadow register because higher
	 * bits of every alvl field [5:7] are write only
	 * If uboot_setAvcr  function was called already
	 * we can skip this function as u-boot forced
	 * other value
	 * , get the input value (hexa) */
	if (!SetAvcrFlag) {
		if (strict_strtoul(s, 16, &uboot_DefaultAvcrValue))
			pr_err("Wrong value assigned to set AVCR!\n");
	}
	printk(KERN_ERR "OBM AVCR is 0x%x\n", uboot_DefaultAvcrValue);
	/* indicating that AVCR value should be updated */
	return 1;
}

static int uboot_setAvcr(char *s)
{
	/* on seeing "tavorcfg_avcr", get the input value (hexa) */
	if (strict_strtoul(s, 16, &uboot_AvcrValue))
		pr_err("Wrong value assigned to set AVCR!\n");
	/* indicating that AVCR value should be updated
	 * If this function is called it will overide the
	 * default value which is delivered by OBM at the
	 * uboot_defaultAvcr function*/
	uboot_DefaultAvcrValue = uboot_AvcrValue;
	printk(KERN_ERR "AVCR is forced by u-boot to 0x%x\n", uboot_AvcrValue);
	SetAvcrFlag = 1;
	return 1;
}

static int uboot_setAlvl3Value(char *s)
{
	/* on seeing "DROV", get the input value (hexa) */
	if (strict_strtoul(s, 16, &alvl3HighVoltage))
		pr_err("Wrong value assigned to set voltage for 988MHz!\n");
	printk(KERN_INFO "1GHz PP voltage is set to 0x%lx\n", alvl3HighVoltage);
	return 1;
}

/* both tavorcfg_nodvfm and androidboot.bsp=bsp will clear PowerDisabled */
__setup("tavorcfg_nolpm=", uboot_disablePower);
__setup("tavorcfg_nodvfm=", uboot_disableDvfm);
__setup("tavorcfg_avcr=", uboot_setAvcr);
__setup("AVCR=", uboot_defaultAvcr);
__setup("DROV=", uboot_setAlvl3Value);
/* all power related uboot handling functions should be placed above this
 * comment in this section */
static int __init pxa95x_freq_init(void)
{
	int ret;
	struct proc_dir_entry *entry;
/* +SAMSUNG_SSENI : Workaround code, disable lpm & DVFM feature when JTAG is connected. */	
	int IsJtagConnected = 0;
/* -SAMSUNG_SSENI : Workaround code, disable lpm & DVFM feature when JTAG is connected. */	

#ifdef CONFIG_PMIC_D1980
///////////////////////////////////////////
//Powerup 104 ALVL0
//D0CS    60  ALVL0
//PP1    156  ALVL0
//PP2    208  ALVL0
//PP3    156  ALVL1
//PP4    208  ALVL1
//PP5    416  ALVL1
//PP6    624  ALVL2
//PP7-8  806  ALVL3
////////////////////////////////////////
    uint32_t profile,voltage;
    
    profile=AVCR_REG;//d1980_read_dvfm_profile();

    pr_err("DVFM AVCR VALUE=0x%x ~~~~~~~~~\n",profile);

    voltage=( profile&0xFF) * 25 + 725;
//pr_err("BSWBSW voltage VALUE=%d ~~~~~~~~~\n",voltage);
     pxa955_op_array[0].vcc_core =voltage; //60Mhz
     pxa955_op_array[1].vcc_core =voltage; //156Mhz
     
     
    voltage=( (profile>>8)&0xFF) * 25 + 725;
   // pr_err("BSWBSW voltage VALUE=%d ~~~~~~~~~\n",voltage);
     pxa955_op_array[2].vcc_core =voltage; //156Mhz
     pxa955_op_array[3].vcc_core =voltage; //208Mhz
     pxa955_op_array[4].vcc_core =voltage; //416
     pxa955_op_array[5].vcc_core =voltage; //416
     
    voltage=( (profile>>16)&0xFF) * 25 + 725;
   // pr_err("BSWBSW voltage VALUE=%d ~~~~~~~~~\n",voltage);
    pxa955_op_array[6].vcc_core =voltage; //624 

    voltage=( (profile>>24)&0xFF) * 25 + 725;
  //  pr_err("BSWBSW voltage VALUE=%d ~~~~~~~~~\n",voltage);
    pxa955_op_array[7].vcc_core =voltage; // 824
     pxa955_op_array[8].vcc_core =voltage; //998


    voltage=( profile&0xFF) * 25 + 725;
  //  pr_err("BSWBSW voltage VALUE=%d ~~~~~~~~~\n",voltage);
     pxa955_op_array[9].vcc_core =voltage; //D1
     pxa955_op_array[10].vcc_core =voltage; //D2
     pxa955_op_array[11].vcc_core =voltage; //CG
#endif
	/* power is detemined by uboot */
	/* Set before registration below otherwise pxa95x_freq_probe attempts
	to set op and call DVFM code */
	if (cpu_is_pxa970()) {
		temp_PowerDisabled = 1;
		temp_DvfmDisabled = 1;
	}
/* +SAMSUNG_SSENI : Workaround code, disable lpm & DVFM feature when JTAG is connected. */	
	{
		__asm__ __volatile__ (
				"mrc   p14, 0, %0, c0, c1, 0\n\t"
				"mov   %0, %0, lsl #17\n\t"
				"mov   %0, %0, lsr #31\n\t"
				:"=r" (IsJtagConnected));
				
		if(IsJtagConnected)
		{
			printk("JTAG is connected. Disable to enter lpm mode & DVFM feature [%d]\n", IsJtagConnected);
			temp_PowerDisabled = 1;
			temp_DvfmDisabled = 1;
		}
	}
/* -SAMSUNG_SSENI : Workaround code, disable lpm & DVFM feature when JTAG is connected. */		
	PowerDisabled = temp_PowerDisabled;
	DvfmDisabled = temp_DvfmDisabled;
	printk("Initial power state: PowerDisabled = %d, DvfmDisabled = %d\n", PowerDisabled, DvfmDisabled);
	ret = misc_register(&dvfm_misc_device);
	if (ret != 0) {
		printk(KERN_ERR "Could not register device ipmc, res = %d.\n ",
		       ret);
		return -EBUSY;
	}
	ret = platform_driver_register(&pxa95x_freq_driver);
	if (ret)
		goto out;
#ifdef CONFIG_PXA95x_DVFM_STATS
	ret = dvfm_register_notifier(&notifier_freq_block,
				     DVFM_FREQUENCY_NOTIFIER);
#endif
	ret = dvfm_register("DVFM", &dvfm_dev_id);
#ifdef DVFM_LP_SAFE
	INIT_WORK(&postpone_work, d0cs_worker);
#endif

	entry = create_proc_entry("d2_led_toggle", 0444, NULL);
	if (entry)
		entry->read_proc = d2_led_toggle_proc;

out:
	return ret;
}

static void __exit pxa95x_freq_exit(void)
{
#ifdef CONFIG_PXA95x_DVFM_STATS
	dvfm_unregister_notifier(&notifier_freq_block, DVFM_FREQUENCY_NOTIFIER);
#endif
	dvfm_unregister("DVFM", &dvfm_dev_id);
	platform_driver_unregister(&pxa95x_freq_driver);
}

/* disable high PP on board with low voltage support */
static int disabe_high_pp_on_low_voltage_board(void)
{
	long board_id = get_board_id();
	int rc;

	if (board_id == -1) {
		printk(KERN_ERR "%s board_id = %d (did not init)\n",
				__func__, (int) board_id);
		return -1;
	} else {
		int user_index = -1;
		rc = dvfm_find_index("User", &user_index);
		if (rc != 0) {
			printk(KERN_ERR "%s (%d) failed get user index from"
					" table rc = %d\n", __func__, __LINE__,
					rc);
			return rc;
		}
		if ((board_id == OBM_SAAR_B_PV2_B0_V1_BOARD)
		    && cpu_is_pxa955_Cx()) {
			printk(KERN_WARNING "%s (%d) board_id %d cpuid is"
					" cpu_is_pxa955_Cx - board is low"
					" voltage\n, need disable the high"
					" PPs\n", __func__, __LINE__,
					(int) board_id);
			rc = dvfm_disable_op_name("624M", user_index);
			if (rc != 0) {
				printk(KERN_ERR "%s (%d) failed set user"
						" constrain on high PP rc"
						" = %d\n", __func__, __LINE__,
						rc);
				return rc;
			}
			rc = dvfm_disable_op_name("806M", user_index);
			if (rc != 0) {
				printk(KERN_ERR "%s (%d) failed set user"
						" constrain on high PP rc"
						" = %d\n", __func__, __LINE__,
						rc);
				return rc;
			}
		}
		if (board_id == OBM_SAAR_B_MG2_A0_V13_BOARD) {
			printk(KERN_WARNING "%s (%d) board_id %d - limit PP to"
				       " 624M Temporary until installing SB2M\n",
				       __func__, __LINE__, (int) board_id);
			rc = dvfm_disable_op_name("806M", user_index);
			if (rc != 0) {
				printk(KERN_ERR "%s (%d) failed set user"
						" constrain on high PP rc"
						" = %d\n", __func__, __LINE__,
						rc);
				return rc;
			}
		}
	}
	return 0;
}

module_init(pxa95x_freq_init);
module_exit(pxa95x_freq_exit);
