/*
 * Copyright (C) 2010 Marvell Corporation
 *
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html
 *
 * (C) Copyright 2010 Marvell International Ltd.
 * All Rights Reserved
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/sysdev.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/list.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/cpufreq.h>
#include <linux/pm_qos_params.h>
#include <asm/io.h>
#include <mach/cputype.h>
#include <mach/hardware.h>
#include <mach/pxa168_pm.h>
#include <mach/regs-mpmu.h>
#include <mach/regs-apmu.h>
#include <mach/regs-ciu.h>
#include "acpuclock.h"

#define OP_NAME_LEN		16
#define CLK_SRC_VCTCXO		(1u<<0)
#define CLK_SRC_PLL1_312	(1u<<1)
#define CLK_SRC_PLL1_624	(1u<<2)
#define CLK_SRC_PLL2		(1u<<3)

struct pxa910_md_opt {
	int power_mode;
	int vcc_core;		/* core voltage, in uV */
	int pclk;		/* core clock */
	int pdclk;		/* DDR interface clock */
	int baclk;		/* bus interface clock */
	int xpclk;		/* L2 cache interface clock */
	int dclk;		/* DDR clock */
	int aclk;		/* bus clock */
	int cp_pclk;
	int cp_pdclk;
	int cp_baclk;
	int cp_xpclk;
	int cp_clk_src;
	int ap_clk_src;
	int ddr_clk_src;
	int axi_clk_src;
	int gc_clk_src;
	int pll2freq;
	int lpj;
	char name[OP_NAME_LEN];
	unsigned int run_time;
	unsigned int idle_time;
};

union pmum_fccr {
	struct {
		unsigned int pll1fbd:9;
		unsigned int pll1refd:5;
		unsigned int pll1cen:1;
		unsigned int mfc:1;
		unsigned int reserved0:1;
		unsigned int gcaclksel:2;
		unsigned int axiclksel0:1;
		unsigned int reserved1:3;
		unsigned int ddrclksel:2;
		unsigned int axiclksel1:1;
		unsigned int seaclksel:3;
		unsigned int mohclksel:3;
	} b;
	unsigned int v;
};

union pmum_pll2cr {
	struct {
		unsigned int reserved0:6;
		unsigned int reserved1:2;
		unsigned int en:1;
		unsigned int ctrl:1;
		unsigned int pll2fbd:9;
		unsigned int pll2refd:5;
		unsigned int reserved2:8;
	} b;
	unsigned int v;
};

union pmua_pllsel {
	struct {
		unsigned int cpclksel:2;
		unsigned int apclksel:2;
		unsigned int ddrclksel:2;
		unsigned int axiclksel:2;
		unsigned int gcaclksel:2;
		unsigned int reserved0:22;
	} b;
	unsigned int v;
};

union pmua_cc {
	struct {
		unsigned int core_clk_div:3;
		unsigned int bus_mc_clk_div:3;
		unsigned int biu_clk_div:3;
		unsigned int xp_clk_div:3;
		unsigned int ddr_clk_div:3;
		unsigned int bus_clk_div:3;
		unsigned int async1:1;
		unsigned int async2:1;
		unsigned int async3:1;
		unsigned int async3_1:1;
		unsigned int async4:1;
		unsigned int async5:1;
		unsigned int core_freq_chg_req:1;
		unsigned int ddr_freq_chg_req:1;
		unsigned int bus_freq_chg_req:1;
		unsigned int core_allow_spd_chg:1;
		unsigned int core_dyn_fc:1;
		unsigned int dclk_dyn_fc:1;
		unsigned int aclk_dyn_fc:1;
		unsigned int core_rd_st_clear:1;
	} b;
	unsigned int v;
};

union pmua_dm_cc {
	struct {
		unsigned int core_clk_div:3;
		unsigned int bus_mc_clk_div:3;
		unsigned int biu_clk_div:3;
		unsigned int xp_clk_div:3;
		unsigned int ddr_clk_div:3;
		unsigned int bus_clk_div:3;
		unsigned int async1:1;
		unsigned int async2:1;
		unsigned int async3:1;
		unsigned int async3_1:1;
		unsigned int async4:1;
		unsigned int async5:1;
		unsigned int sea_rd_status:1;
		unsigned int moh_rd_status:1;
		unsigned int cp_fc_done:1;
		unsigned int ap_fc_done:1;
		unsigned int dclk_fc_done:1;
		unsigned int aclk_fc_done:1;
		unsigned int reserved:2;
	} b;
	unsigned int v;
};

/*mutex lock protecting frequency change */
static DEFINE_MUTEX(freqs_mutex);

/* current operating point */
static struct pxa910_md_opt *cur_md;
static struct pxa910_md_opt pxa920_op_array[] = {
	/* core 156MHz ddr 104MHz bus 104MHz */
	{
	 .power_mode = POWER_MODE_ACTIVE,
	 .vcc_core = 1225000,
	 .pclk = 156,
	 .pdclk = 78,
	 .baclk = 78,
	 .xpclk = 156,
	 .dclk = 104,
	 .aclk = 104,
	 .lpj = 156 * 500000 / HZ,
	 .name = "156MHz",
	 },
	/* core 312MHz ddr 156MHz bus 156MHz */
	{
	 .power_mode = POWER_MODE_ACTIVE,
	 .vcc_core = 1225000,
	 .pclk = 312,
	 .pdclk = 156,
	 .baclk = 156,
	 .xpclk = 312,
	 .dclk = 156,
	 .aclk = 156,
	 .lpj = 312 * 500000 / HZ,
	 .name = "312MHz",
	 },
	/* core 624MHz ddr 156MHz bus 156MHz */
	{
	 .power_mode = POWER_MODE_ACTIVE,
	 .vcc_core = 1225000,
	 .pclk = 624,
	 .pdclk = 156,
	 .baclk = 156,
	 .xpclk = 312,
	 .dclk = 156,
	 .aclk = 156,
	 .lpj = 624 * 500000 / HZ,
	 .name = "624MHz",
	 },
	/* core 797MHz ddr 199MHz bus 208MHz */
	/*{
	   .power_mode = POWER_MODE_ACTIVE,
	   .vcc_core = 1300,
	   .pclk = 797,
	   .pdclk = 199,
	   .baclk = 199,
	   .xpclk = 398,
	   .dclk = 199,
	   .aclk = 208,
	   .lpj = 797*500000/HZ,
	   .name = "797MHz",
	   }, */
	/* core 801MHz ddr 200MHz bus 200MHz */
	/*{
	   .power_mode = POWER_MODE_ACTIVE,
	   .vcc_core = 1300,
	   .pclk = 801,
	   .pdclk = 200,
	   .baclk = 200,
	   .xpclk = 400,
	   .dclk = 200,
	   .aclk = 208,
	   .lpj = 801*500000/HZ,
	   .name = "801MHz",
	   }, */
	/* core 806MHz ddr 201MHz bus 208MHz */
	{
	 .power_mode = POWER_MODE_ACTIVE,
	 .vcc_core = 1300000,
	 .pclk = 806,
	 .pdclk = 201,
	 .baclk = 201,
	 .xpclk = 403,
	 .dclk = 201,
	 .aclk = 208,
	 .lpj = 806 * 500000 / HZ,
	 .name = "806MHz",
	 },
};

static struct pxa910_md_opt pxa918_op_array[] = {
	/* core 156MHz ddr 104MHz bus 104MHz */
	{
		.power_mode = POWER_MODE_ACTIVE,
		.vcc_core = 1225000,
		.pclk = 156,
		.pdclk = 78,
		.baclk = 78,
		.xpclk = 156,
		.dclk = 104,
		.aclk = 104,
		.lpj = 156*500000/HZ,
		.name = "156MHz",
	},
	/* core 312MHz ddr 156MHz bus 156MHz */
	{
		.power_mode = POWER_MODE_ACTIVE,
		.vcc_core = 1225000,
		.pclk = 312,
		.pdclk = 156,
		.baclk = 156,
		.xpclk = 312,
		.dclk = 156,
		.aclk = 156,
		.lpj = 312*500000/HZ,
		.name = "312MHz",
	},
	/* core 624MHz ddr 156MHz bus 156MHz */
	{
		.power_mode = POWER_MODE_ACTIVE,
		.vcc_core = 1225000,
		.pclk = 624,
		.pdclk = 156,
		.baclk = 156,
		.xpclk = 312,
		.dclk = 156,
		.aclk = 156,
		.lpj = 624*500000/HZ,
		.name = "624MHz",
	},
};

static struct pxa910_md_opt pxa921_op_array[] = {
	/* core 156MHz ddr 156MHz bus 156MHz */
	{
		.power_mode = POWER_MODE_ACTIVE,
		.vcc_core = 1225000,
		.pclk = 156,
		.pdclk = 156,
		.baclk = 156,
		.xpclk = 156,
		.dclk = 156,
		.aclk = 156,
		.lpj = 156*500000/HZ,
		.name = "156MHz",
	},
	/* core 312MHz ddr 156MHz bus 156MHz */
	{
		.power_mode = POWER_MODE_ACTIVE,
		.vcc_core = 1225000,
		.pclk = 312,
		.pdclk = 156,
		.baclk = 156,
		.xpclk = 312,
		.dclk = 156,
		.aclk = 156,
		.lpj = 312*500000/HZ,
		.name = "312MHz",
	},
	/* core 500.5MHz ddr 250MHz bus 250MHz */
	{
		.power_mode = POWER_MODE_ACTIVE,
		.vcc_core = 1225000,
		.pclk = 500,
		.pdclk = 250,
		.baclk = 250,
		.xpclk = 250,
		.dclk = 250,
		.aclk = 156,
		.lpj = 500*500000/HZ,
		.name = "500MHz",
	},
	/* core 1001MHz ddr 250MHz bus 208MHz */
	{
		.power_mode = POWER_MODE_ACTIVE,
		.vcc_core = 1350000,
		.pclk = 1001,
		.pdclk = 250,
		.baclk = 250,
		.xpclk = 500,
		.dclk = 250,
		.aclk = 208,
		.lpj = 1001*500000/HZ,
		.name = "1001MHz",
	},
};

static struct pxa910_md_opt pxa910_op_array[] = {
	/* core 208MHz ddr 104MHz bus 104MHz */
	{
	 .power_mode = POWER_MODE_ACTIVE,
	 .vcc_core = 1225000,
	 .pclk = 208,
	 .pdclk = 104,
	 .baclk = 104,
	 .xpclk = 104,
	 .dclk = 104,
	 .aclk = 104,
	 .lpj = 208 * 500000 / HZ,
	 .name = "208MHz",
	 },
	/* core 312MHz ddr 156MHz bus 156MHz */
	{
	 .power_mode = POWER_MODE_ACTIVE,
	 .vcc_core = 1225000,
	 .pclk = 312,
	 .pdclk = 156,
	 .baclk = 156,
	 .xpclk = 156,
	 .dclk = 156,
	 .aclk = 156,
	 .lpj = 312 * 500000 / HZ,
	 .name = "312MHz",
	 },
	/* core 624MHz ddr 156MHz bus 156MHz */
	{
	 .power_mode = POWER_MODE_ACTIVE,
	 .vcc_core = 1225000,
	 .pclk = 624,
	 .pdclk = 156,
	 .baclk = 156,
	 .xpclk = 312,
	 .dclk = 156,
	 .aclk = 156,
	 .lpj = 624 * 500000 / HZ,
	 .name = "624MHz",
	 },
	/* core 806MHz ddr 201MHz bus 201MHz */
	{
	 .power_mode = POWER_MODE_ACTIVE,
	 .vcc_core = 1300000,
	 .pclk = 806,
	 .pdclk = 201,
	 .baclk = 201,
	 .xpclk = 403,
	 .dclk = 201,
	 .aclk = 208,
	 .lpj = 806 * 500000 / HZ,
	 .name = "806MHz",
	 },
};

struct proc_op_array {
	unsigned int cpuid;
	unsigned int chip_id;
	char *cpu_name;
	struct pxa910_md_opt *op_array;
	int op_array_size;
};

static struct proc_op_array proc_op_arrays[] = {
	{0x8000, 0xc921, "PXA921", pxa921_op_array, ARRAY_SIZE(pxa921_op_array)},
    {0x8000, 0xc920, "PXA920", pxa920_op_array, ARRAY_SIZE(pxa920_op_array)},
    {0x8000, 0xc910, "PXA910", pxa910_op_array, ARRAY_SIZE(pxa910_op_array)},
    {0x8000, 0xc918, "PXA918", pxa918_op_array, ARRAY_SIZE(pxa918_op_array)},
};

static struct pxa910_md_opt *op_array;
static int op_array_size;

static int constraint_min_freq;	/*in MHz */
static int constraint_cpufreq_disable;

#ifdef CONFIG_CPU_FREQ_TABLE
static struct cpufreq_frequency_table *freq_table;

static void __init acpuclk_init_cpufreq_table(void)
{
	int i;
	int n = op_array_size;

	freq_table = kmalloc(sizeof(struct cpufreq_frequency_table) * (n + 1),
			     GFP_KERNEL);
	if (!freq_table)
		return;

	for (i = 0; i < n; i++) {
		freq_table[i].index = i;
		freq_table[i].frequency = op_array[i].pclk * MHZ_TO_KHZ;
	}

	freq_table[i].index = i;
	freq_table[i].frequency = CPUFREQ_TABLE_END;

	cpufreq_frequency_table_get_attr(freq_table, smp_processor_id());
}
#else
#define acpuclk_init_cpufreq_table() do {} while (0);
#endif

static int fc_lock_ref_cnt;

static void get_fc_lock(void)
{
	union pmua_dm_cc dm_cc_ap;

	fc_lock_ref_cnt++;

	if (fc_lock_ref_cnt == 1) {
		int timeout = 100000;

		/* AP-CP FC mutual exclusion */
		dm_cc_ap.v = __raw_readl(APMU_CCSR);
		while (dm_cc_ap.b.sea_rd_status && timeout) {
			dm_cc_ap.v = __raw_readl(APMU_CCSR);
			timeout--;
		}
		if (timeout <= 0) {
			pr_err("cp does not release its fc lock\n");
			BUG();
		}
	}
}

static void put_fc_lock(void)
{
	union pmua_cc cc_ap;

	fc_lock_ref_cnt--;

	if (fc_lock_ref_cnt < 0)
		pr_err("unmatched put_fc_lock\n");

	if (fc_lock_ref_cnt == 0) {
		/* write 1 to MOH_RD_ST_CLEAR to clear MOH_RD_STATUS */
		cc_ap.v = __raw_readl(APMU_CCR);
		cc_ap.b.core_rd_st_clear = 1;
		__raw_writel(cc_ap.v, APMU_CCR);
		cc_ap.b.core_rd_st_clear = 0;
		__raw_writel(cc_ap.v, APMU_CCR);
	}
}

static unsigned int to_refd_div(unsigned int refd)
{
	unsigned int div;
	switch (refd) {
	case 0x00:
		div = 1;
		break;
	case 0x10:
		div = 2;
		break;
	case 0x01:
		div = 3;
		break;
	default:
		div = refd + 2;
		break;
	}
	return div;
}

static unsigned int to_fbd_div(unsigned int fbd)
{
	return fbd + 2;
}

u32 get_pll2_freq(void)
{
	union pmum_pll2cr pll2cr;

	pll2cr.v = __raw_readl(MPMU_PLL2CR);
	if ((pll2cr.b.ctrl == 1) && (pll2cr.b.en == 0))
		return 0;
	return 26 * to_fbd_div(pll2cr.b.pll2fbd)
	    / to_refd_div(pll2cr.b.pll2refd);
}
EXPORT_SYMBOL(get_pll2_freq);

unsigned char __iomem *sram_last_page;

static unsigned int to_clk_src(unsigned int sel, u32 pll2freq)
{
	unsigned int clk = 0;

	switch (sel) {
	case 0:
		clk = 312;
		break;
	case 1:
		clk = 624;
		break;
	case 2:
		clk = pll2freq;
		break;
	case 3:
		clk = 26;
		break;
	default:
		pr_err("Wrong clock source\n");
		break;
	}
	return clk;
}

static void get_current_op(struct pxa910_md_opt *cop)
{
	union pmua_pllsel pllsel;
	union pmua_dm_cc dm_cc_cp, dm_cc_ap;
	union pmua_cc cc_cp;

	get_fc_lock();
	dm_cc_ap.v = __raw_readl(APMU_CCSR);
	dm_cc_cp.v = __raw_readl(APMU_CP_CCSR);
	cc_cp.v = __raw_readl(APMU_CP_CCR);
	cc_cp.b.core_rd_st_clear = 1;
	__raw_writel(cc_cp.v, APMU_CP_CCR);
	cc_cp.b.core_rd_st_clear = 0;
	__raw_writel(cc_cp.v, APMU_CP_CCR);
	pllsel.v = __raw_readl(APMU_PLL_SEL_STATUS);

	cop->pll2freq = get_pll2_freq();
	cop->ap_clk_src = to_clk_src(pllsel.b.apclksel, cop->pll2freq);
	cop->cp_clk_src = to_clk_src(pllsel.b.cpclksel, cop->pll2freq);
	cop->axi_clk_src = to_clk_src(pllsel.b.axiclksel, cop->pll2freq);
	cop->ddr_clk_src = to_clk_src(pllsel.b.ddrclksel, cop->pll2freq);
	cop->gc_clk_src = to_clk_src(pllsel.b.gcaclksel, cop->pll2freq);
	cop->pclk = cop->ap_clk_src / (dm_cc_ap.b.core_clk_div + 1);
	cop->pdclk = cop->ap_clk_src / (dm_cc_ap.b.bus_mc_clk_div + 1);
	cop->baclk = cop->ap_clk_src / (dm_cc_ap.b.biu_clk_div + 1);
	cop->xpclk = cop->ap_clk_src / (dm_cc_ap.b.xp_clk_div + 1);
	cop->cp_pclk = cop->cp_clk_src / (dm_cc_cp.b.core_clk_div + 1);
	cop->cp_pdclk = cop->cp_clk_src / (dm_cc_cp.b.bus_mc_clk_div + 1);
	cop->cp_baclk = cop->cp_clk_src / (dm_cc_cp.b.biu_clk_div + 1);
	cop->cp_xpclk = cop->cp_clk_src / (dm_cc_cp.b.xp_clk_div + 1);
	cop->dclk = cop->ddr_clk_src / (dm_cc_ap.b.ddr_clk_div + 1) / 2;
	cop->aclk = cop->axi_clk_src / (dm_cc_ap.b.bus_clk_div + 1);

	if ((cop->cp_clk_src != 312))
		pr_err("Wrong cp clock source\n");
	if (dm_cc_ap.b.biu_clk_div)
		pr_err("biu_clk_div should be 0 for Ax stepping\n");
	put_fc_lock();
}

static u32 choose_clk_src(u32 clk)
{
	u32 choice = 0;

	if ((26 % clk) == 0) {
		choice |= CLK_SRC_VCTCXO;
	} else {
		if ((312 / clk <= 8) && ((312 % clk) == 0))
			choice |= CLK_SRC_PLL1_312;
		if ((624 / clk <= 8) && ((624 % clk) == 0))
			choice |= CLK_SRC_PLL1_624;
		/* we assume the clk can always be generated from PLL2 */
		choice |= CLK_SRC_PLL2;
	}
	return choice;
}

/*
 * Clock source selection principle:
 *   1. do not use PLL2 if possible
 *   2. use the same clock source for core and ddr if possible
 *   3. try to use the lowest PLL source for core/ddr/axi
 *
 * FIXME: the code does not follow the principle right now
 */
static void select_clk_src(struct pxa910_md_opt *top)
{
	u32 dclk2x = top->dclk * 2;
	u32 ap_clk_choice, axi_clk_choice, ddr_clk_choice;

	ap_clk_choice = choose_clk_src(top->pclk);
	axi_clk_choice = choose_clk_src(top->aclk);
	ddr_clk_choice = choose_clk_src(dclk2x);

	if ((ap_clk_choice == 0) || (axi_clk_choice == 0)
	    || (ddr_clk_choice == 0))
		pr_err("Wrong clock combination\n");

	top->pll2freq = top->ap_clk_src =
	    top->axi_clk_src = top->ddr_clk_src = 0;

	if (ap_clk_choice & axi_clk_choice & ddr_clk_choice &
		   CLK_SRC_PLL1_624) {
		top->ap_clk_src = top->axi_clk_src = top->ddr_clk_src = 624;
	} else if (ap_clk_choice & axi_clk_choice & ddr_clk_choice &
		   CLK_SRC_PLL1_312) {
		top->ap_clk_src = top->axi_clk_src = top->ddr_clk_src = 312;
	} else if ((ap_clk_choice & CLK_SRC_PLL1_312)
		   && (axi_clk_choice & ddr_clk_choice & CLK_SRC_PLL1_624)) {
		top->ap_clk_src = 312;
		top->axi_clk_src = top->ddr_clk_src = 624;
	} else if ((ap_clk_choice & CLK_SRC_PLL1_624) &&
		   (axi_clk_choice & ddr_clk_choice & CLK_SRC_PLL1_312)) {
		top->ap_clk_src = 624;
		top->axi_clk_src = top->ddr_clk_src = 312;
	} else if ((axi_clk_choice & CLK_SRC_PLL1_312) &&
		   (ap_clk_choice & ddr_clk_choice & CLK_SRC_PLL1_624)) {
		top->axi_clk_src = 312;
		top->ap_clk_src = top->ddr_clk_src = 624;
	} else if ((axi_clk_choice & CLK_SRC_PLL1_624) &&
		   (ap_clk_choice & ddr_clk_choice & CLK_SRC_PLL1_312)) {
		top->axi_clk_src = 624;
		top->ap_clk_src = top->ddr_clk_src = 312;
	} else if ((ddr_clk_choice & CLK_SRC_PLL1_312) &&
		   (ap_clk_choice & axi_clk_choice & CLK_SRC_PLL1_624)) {
		top->ddr_clk_src = 312;
		top->ap_clk_src = top->axi_clk_src = 624;
	} else if ((ddr_clk_choice & CLK_SRC_PLL1_624) &&
		   (ap_clk_choice & axi_clk_choice & CLK_SRC_PLL1_312)) {
		top->ddr_clk_src = 624;
		top->ap_clk_src = top->axi_clk_src = 312;
	} else if (ap_clk_choice & CLK_SRC_PLL2) {
		if (cpu_is_pxa921()) {
			if (top->pclk == 500)
				top->pll2freq = 1001;
			else
				top->pll2freq = top->pclk;
		} else
			top->pll2freq = top->pclk;

		top->ap_clk_src = top->pll2freq;
		if ((top->pll2freq / dclk2x <= 8)
		    && (top->pll2freq % dclk2x) == 0)
			top->ddr_clk_src = top->pll2freq;
		else if (ddr_clk_choice & CLK_SRC_PLL1_312)
			top->ddr_clk_src = 312;
		else if (ddr_clk_choice & CLK_SRC_PLL1_624)
			top->ddr_clk_src = 624;
		if (top->ddr_clk_src == 0)
			top->ddr_clk_src = top->pll2freq;
		/* try to use lowest PLL source for axi bus */
		if (axi_clk_choice & CLK_SRC_PLL1_312)
			top->axi_clk_src = 312;
		else if (axi_clk_choice & CLK_SRC_PLL1_624)
			top->axi_clk_src = 624;
		else if ((top->pll2freq / top->aclk <= 8)
			 && (top->pll2freq % top->aclk) == 0)
			top->axi_clk_src = top->pll2freq;
		if (top->axi_clk_src == 0)
			top->axi_clk_src = top->pll2freq;
	} else if (axi_clk_choice & CLK_SRC_PLL2) {
		top->pll2freq = top->aclk;
		top->axi_clk_src = top->pll2freq;
		if (ap_clk_choice & CLK_SRC_PLL1_312)
			top->ap_clk_src = 312;
		else if (ap_clk_choice & CLK_SRC_PLL1_624)
			top->ap_clk_src = 624;
		if ((top->pll2freq / dclk2x <= 8) &&
		    (top->pll2freq % dclk2x) == 0)
			top->ddr_clk_src = top->pll2freq;
		else if (ddr_clk_choice & CLK_SRC_PLL1_312)
			top->ddr_clk_src = 312;
		else if (ddr_clk_choice & CLK_SRC_PLL1_624)
			top->ddr_clk_src = 624;
	} else if (ddr_clk_choice & CLK_SRC_PLL2) {
		top->pll2freq = dclk2x;
		top->ddr_clk_src = top->pll2freq;
		if (ap_clk_choice & CLK_SRC_PLL1_312)
			top->ap_clk_src = 312;
		else if (ap_clk_choice & CLK_SRC_PLL1_624)
			top->ap_clk_src = 624;
		if (axi_clk_choice & CLK_SRC_PLL1_312)
			top->axi_clk_src = 312;
		else if (axi_clk_choice & CLK_SRC_PLL1_624)
			top->axi_clk_src = 624;
	} else if (ap_clk_choice & axi_clk_choice &
		   ddr_clk_choice & CLK_SRC_VCTCXO) {
		top->ap_clk_src = top->axi_clk_src = top->ddr_clk_src = 26;
	}

	if ((top->ap_clk_src == 0) || (top->axi_clk_src == 0) ||
	    (top->ddr_clk_src == 0))
		pr_err("Wrong clock combination\n");
}

static void set_ap_clk_sel(struct pxa910_md_opt *top)
{
	union pmum_fccr fccr;

	fccr.v = __raw_readl(MPMU_FCCR);
	if (top->ap_clk_src == 312)
		fccr.b.mohclksel = 0;
	else if (top->ap_clk_src == 624)
		fccr.b.mohclksel = 1;
	else if (top->ap_clk_src == top->pll2freq)
		fccr.b.mohclksel = 2;
	else if (top->ap_clk_src == 26)
		fccr.b.mohclksel = 3;
	__raw_writel(fccr.v, MPMU_FCCR);
}

static void set_axi_clk_sel(struct pxa910_md_opt *top)
{
	union pmum_fccr fccr;

	fccr.v = __raw_readl(MPMU_FCCR);
	if (top->axi_clk_src == 312) {
		fccr.b.axiclksel1 = 0;
		fccr.b.axiclksel0 = 0;
	} else if (top->axi_clk_src == 624) {
		fccr.b.axiclksel1 = 0;
		fccr.b.axiclksel0 = 1;
	} else if (top->axi_clk_src == top->pll2freq) {
		fccr.b.axiclksel1 = 1;
		fccr.b.axiclksel0 = 0;
	} else if (top->axi_clk_src == 26) {
		fccr.b.axiclksel1 = 1;
		fccr.b.axiclksel0 = 1;
	}
	__raw_writel(fccr.v, MPMU_FCCR);
}

static void set_ddr_clk_sel(struct pxa910_md_opt *top)
{
	union pmum_fccr fccr;

	fccr.v = __raw_readl(MPMU_FCCR);
	if (top->ddr_clk_src == 312)
		fccr.b.ddrclksel = 0;
	else if (top->ddr_clk_src == 624)
		fccr.b.ddrclksel = 1;
	else if (top->ddr_clk_src == top->pll2freq)
		fccr.b.ddrclksel = 2;
	else if (top->ddr_clk_src == 26)
		fccr.b.ddrclksel = 3;
	__raw_writel(fccr.v, MPMU_FCCR);
}

static void set_clk_sel(struct pxa910_md_opt *top)
{
	union pmum_fccr fccr;

	fccr.v = __raw_readl(MPMU_FCCR);

	if (top->ap_clk_src == 312)
		fccr.b.mohclksel = 0;
	else if (top->ap_clk_src == 624)
		fccr.b.mohclksel = 1;
	else if (top->ap_clk_src == top->pll2freq)
		fccr.b.mohclksel = 2;
	else if (top->ap_clk_src == 26)
		fccr.b.mohclksel = 3;

	if (top->axi_clk_src == 312) {
		fccr.b.axiclksel1 = 0;
		fccr.b.axiclksel0 = 0;
	} else if (top->axi_clk_src == 624) {
		fccr.b.axiclksel1 = 0;
		fccr.b.axiclksel0 = 1;
	} else if (top->axi_clk_src == top->pll2freq) {
		fccr.b.axiclksel1 = 1;
		fccr.b.axiclksel0 = 0;
	} else if (top->axi_clk_src == 26) {
		fccr.b.axiclksel1 = 1;
		fccr.b.axiclksel0 = 1;
	}

	if (top->ddr_clk_src == 312)
		fccr.b.ddrclksel = 0;
	else if (top->ddr_clk_src == 624)
		fccr.b.ddrclksel = 1;
	else if (top->ddr_clk_src == top->pll2freq)
		fccr.b.ddrclksel = 2;
	else if (top->ddr_clk_src == 26)
		fccr.b.ddrclksel = 3;

	__raw_writel(fccr.v, MPMU_FCCR);
}

static void wait_for_fc_done(void)
{
	int timeout = 1000000;
	while (!((1 << 1) & __raw_readl(APMU_ISR)) && timeout)
		timeout--;
	if (timeout <= 0)
		panic("AP frequency change timeout!\n");
	__raw_writel(0x0, APMU_ISR);
}

static struct regulator *v_buck1;

static int pxa910_dvfm_get_core_voltage(void)
{
	int vcc_volt;

	vcc_volt = regulator_get_voltage(v_buck1);
	return vcc_volt;
}

static int pxa910_dvfm_set_core_voltage(int uV)
{
	int vcc_volt;

	if (regulator_set_voltage(v_buck1, uV, uV))
		return -EIO;
	vcc_volt = regulator_get_voltage(v_buck1);
	if (vcc_volt != uV)
		return -EIO;
	return 0;
}

static void enable_fc_intr(void)
{
	u32 fc_int_msk;

	fc_int_msk = __raw_readl(APMU_IMR);
	/*
	 * enable AP FC done interrupt for one step,
	 * while not use three interrupts by three steps
	 */
	fc_int_msk |= (1<<1);
	__raw_writel(fc_int_msk, APMU_IMR);
}

#define VOL_UP_FAIL	-1
#define VOL_DOWN_FAIL	-2


static void PMUcore2_fc_seq(struct pxa910_md_opt *cop,
			    struct pxa910_md_opt *top,
			    struct pxa910_md_opt *old)
{
	union pmua_cc cc_ap, cc_cp;
	u32 dclk2x = top->dclk * 2;
	int two_step = 0;

	cc_cp.v = __raw_readl(APMU_CP_CCR);
	cc_cp.b.core_allow_spd_chg = 1;
	__raw_writel(cc_cp.v, APMU_CP_CCR);

	cc_ap.v = __raw_readl(APMU_CCR);
	cc_ap.b.core_allow_spd_chg = 1;

	/* If PLL2 is involved, two-step frequency change is needed */
	if (old->pll2freq || top->pll2freq)
		two_step = 1;

	/* async5 async4 async3_1 async3 are always 1 */
	cc_ap.b.async5 = 1;
	cc_ap.b.async4 = 1;
	cc_ap.b.async3_1 = 1;
	cc_ap.b.async3 = 1;

	/* use async mode when doing core/axi frequency change */
	cc_ap.b.async2 = 1;
	cc_ap.b.async1 = 1;

	if ((cop->ap_clk_src != top->ap_clk_src)
	    || (cop->pclk != top->pclk)
	    || (cop->pdclk != top->pdclk)
	    || (cop->baclk != top->baclk)) {
		if (two_step)
			set_ap_clk_sel(top);
		cc_ap.b.core_clk_div = top->ap_clk_src / top->pclk - 1;
		cc_ap.b.bus_mc_clk_div = top->ap_clk_src / top->pdclk - 1;
		cc_ap.b.biu_clk_div = 0;
		cc_ap.b.xp_clk_div = top->ap_clk_src / top->xpclk - 1;
		cc_ap.b.core_freq_chg_req = 1;
		if (two_step) {
			__raw_writel(cc_ap.v, APMU_CCR);
			wait_for_fc_done();
			cc_ap.b.core_freq_chg_req = 0;
		}
	}

	if ((cop->axi_clk_src != top->axi_clk_src)
	    || (cop->aclk != top->aclk)) {
		if (two_step)
			set_axi_clk_sel(top);
		cc_ap.b.bus_clk_div = top->axi_clk_src / top->aclk - 1;
		cc_ap.b.bus_freq_chg_req = 1;
	}

	/* set sync mode if possible when doing ddr frequency change */
	if ((top->ap_clk_src == top->ddr_clk_src)
	    && (top->pdclk == top->dclk))
		cc_ap.b.async2 = 0;
	else
		cc_ap.b.async2 = 1;
	/* keep ASYNC mode for CP ddr access for all operating points */
	/* if ((cop->cp_clk_src == top->ddr_clk_src)
	    && (cop->cp_pdclk == top->dclk))
		cc_ap.b.async1 = 0;
	else
		cc_ap.b.async1 = 1; */

	if ((cop->ddr_clk_src != top->ddr_clk_src)
	    || (cop->dclk != top->dclk)
	    || (cc_ap.b.async2 == 0) || (cc_ap.b.async1 == 0)) {
		if (two_step)
			set_ddr_clk_sel(top);
		cc_ap.b.ddr_clk_div = top->ddr_clk_src / dclk2x - 1;
		cc_ap.b.ddr_freq_chg_req = 1;
		if (two_step) {
			freq_change(sram_last_page, cc_ap.v, APMU_CP_CCR);
			cc_ap.b.ddr_freq_chg_req = 0;
		}
	}

	/*
	 * set clk sources for pclk, aclk and ddrclk,
	 * and update FCCR at the same time
	 */
	if (!two_step) {
		set_clk_sel(top);
		freq_change(sram_last_page, cc_ap.v, APMU_CP_CCR);
	}

	cc_ap.v = __raw_readl(APMU_CCR);
	cc_ap.b.core_allow_spd_chg = 0;
	__raw_writel(cc_ap.v, APMU_CCR);
	cc_cp.v = __raw_readl(APMU_CP_CCR);
	cc_cp.b.core_allow_spd_chg = 0;
	__raw_writel(cc_cp.v, APMU_CP_CCR);

	__raw_writel(0x0, APMU_ISR);
}

static void set_freq(struct pxa910_md_opt *old, struct pxa910_md_opt *new)
{
	u32 temp = 0;
	struct pxa910_md_opt cop;

	/*
	 * Check LCD reset is released so that AXI2MC reset
	 * is de-asserted and ddr frequency change can be done.
	 * If not, issue warning message and return.
	 */
	if ((__raw_readl(APMU_LCD_CLK_RES_CTRL) & 0x1) != 0x1) {
		pr_info("LCD reset is not released,"
			"set_freq will do nothing\n");
		return;
	}

	/* Check CP reset is released */
	if (__raw_readl(MPMU_APRR) & 0x1) {
		temp = __raw_readl(APMU_DEBUG);
		__raw_writel(temp | (1 << 0) | (1 << 3), APMU_DEBUG);
	}

	get_fc_lock();

	memcpy(&cop, old, sizeof(struct pxa910_md_opt));
	get_current_op(&cop);
	if ((cop.pclk != old->pclk) || (cop.pdclk != old->pdclk) ||
	    (cop.xpclk != old->xpclk) ||
	    (cop.aclk != old->aclk) || (cop.dclk != old->dclk) ||
	    (cop.ap_clk_src != old->ap_clk_src) ||
	    (cop.axi_clk_src != old->axi_clk_src) ||
	    (cop.ddr_clk_src != old->ddr_clk_src)) {
		pr_err("%d %d %d %d %d %d %d %d %d\n", old->pclk,
		       old->pdclk, old->xpclk, old->baclk, old->aclk,
		       old->dclk, old->ap_clk_src, old->axi_clk_src,
		       old->ddr_clk_src);
		pr_err("%d %d %d %d %d %d %d %d %d\n", cop.pclk, cop.pdclk,
		       cop.xpclk, cop.baclk, cop.aclk, cop.dclk,
		       cop.ap_clk_src, cop.axi_clk_src, cop.ddr_clk_src);
		pr_err("%d %d %d %d %d %d %d %d %d\n", new->pclk,
		       new->pdclk, new->xpclk, new->baclk, new->aclk,
		       new->dclk, new->ap_clk_src, new->axi_clk_src,
		       new->ddr_clk_src);
		dump_stack();
	}

	PMUcore2_fc_seq(&cop, new, old);

	get_current_op(&cop);
	if ((cop.pclk != new->pclk) || (cop.pdclk != new->pdclk) ||
	    (cop.xpclk != new->xpclk) ||
	    (cop.aclk != new->aclk) || (cop.dclk != new->dclk) ||
	    (cop.ap_clk_src != new->ap_clk_src) ||
	    (cop.axi_clk_src != new->axi_clk_src) ||
	    (cop.ddr_clk_src != new->ddr_clk_src))
		pr_err("unsuccessful frequency change!\n");

	put_fc_lock();

	/* Check CP reset is released */
	if (__raw_readl(MPMU_APRR) & 0x1)
		__raw_writel(temp, APMU_DEBUG);
}

int pxa910_set_freq(int index)
{
	struct pxa910_md_opt *md_new, *md_old;
	int ret = 0;
	mutex_lock(&freqs_mutex);

	if (constraint_cpufreq_disable != 0) {
		ret = -EAGAIN;
		goto out;
	}

	md_new = &op_array[index];
	if (md_new->pclk < constraint_min_freq || md_new == cur_md)
		goto out;
	md_old = cur_md;

	/*if new op's voltage higher than cur op's,
	   increase the voltage before changing op */
	if (md_new->vcc_core > md_old->vcc_core) {
		if (pxa910_dvfm_set_core_voltage(md_new->vcc_core) < 0) {
			ret = VOL_UP_FAIL;
			pr_err("voltage up failed, still at %d\n",
			       md_old->vcc_core);
			goto out;
		}
	}

	set_freq(md_old, md_new);
	cur_md = md_new;
	loops_per_jiffy = md_new->lpj;

	/*if new op's voltage lower than cur op's,
	   decrease the voltage after changing op */
	if (md_new->vcc_core < md_old->vcc_core) {
		if (pxa910_dvfm_set_core_voltage(md_new->vcc_core) < 0) {
			ret = VOL_DOWN_FAIL;
			pr_err("voltage down failed, still at %d\n",
			       md_old->vcc_core);
			goto out;
		}
	}
out:
	mutex_unlock(&freqs_mutex);
	return ret;
}

/* Get current setting, and record it in fv_info structure
 */
static int capture_op_info(struct pxa910_md_opt *fv_info)
{
	if (!fv_info)
		return -EFAULT;
	memset(fv_info, 0, sizeof(struct pxa910_md_opt));
	get_current_op(fv_info);
	fv_info->vcc_core = pxa910_dvfm_get_core_voltage();
	return 0;
}

int pxa910_get_freq(void)
{
	if (cur_md)
		return cur_md->pclk;
	else
		pr_err("pxa910_get_freq: cur_md NULL\n");

	return 0;
}

/* Note that we fix gc aclk to PLL2/4 so make sure PLL2 is enabled previously */
void gc_aclk_fc(void)
{
	union pmum_fccr fccr;
	unsigned int temp;

	/* set GC ACLK clock source to PLL2 */
	fccr.v = __raw_readl(MPMU_FCCR);
	fccr.b.gcaclksel = 0x2;
	__raw_writel(fccr.v, MPMU_FCCR);

	/* set GC ACLK clock divider to 4 */
	temp = __raw_readl(APMU_GC_CLK_RES_CTRL);
	temp |= (3 << 12);
	__raw_writel(temp, APMU_GC_CLK_RES_CTRL);

	/* GC ACLK dynamic clock change */
	temp |= (1 << 16);
	__raw_writel(temp, APMU_GC_CLK_RES_CTRL);
}

#define SRAM_ADDR_END 0xd1020000

static int cpufreq_min_notify(struct notifier_block *b,
			      unsigned long min, void *v)
{
	int i;
	mutex_lock(&freqs_mutex);

	if (min > 806)
		min = 806;

	constraint_min_freq = min;

	for (i = 0; i < op_array_size; i++) {
		if (op_array[i].pclk >= min) {
			mutex_unlock(&freqs_mutex);
			pxa910_set_freq(i);
			return NOTIFY_OK;
		}
	}
	mutex_unlock(&freqs_mutex);
	return NOTIFY_OK;
}

static struct notifier_block cpufreq_min_notifier = {
	.notifier_call = cpufreq_min_notify,
};

static int cpufreq_disable_notify(struct notifier_block *b,
			      unsigned long min, void *v)
{
	mutex_lock(&freqs_mutex);
	constraint_cpufreq_disable = min;
	mutex_unlock(&freqs_mutex);

	return NOTIFY_OK;
}

static struct notifier_block cpufreq_disable_notifier = {
	.notifier_call = cpufreq_disable_notify,
};

static struct pm_qos_request_list *user_qos_req_min;


/* #####################Debug Function######################## */
 static int dump_op(char *buf, struct pxa910_md_opt *q)
{
	int len;

	if (q == NULL)
		len = sprintf(buf, "Can't dump the op info\n");
	else {
		len = sprintf(buf, "OP name:%s\n", q->name);
		len += sprintf(buf + len, "pclk:%d pdclk:%d baclk:%d xpclk:%d "
				"dclk:%d aclk:%d vcc_core:%d\n",
				q->pclk, q->pdclk, q->baclk, q->xpclk,
				q->dclk, q->aclk, q->vcc_core);
	}
	return len;
}

/* Display current operating point */
static ssize_t cur_op_show(struct sys_device *sys_dev,
			struct sysdev_attribute *attr, char *buf)
{
	return dump_op(buf, cur_md);
}

static SYSDEV_ATTR(cur_op, 0444, cur_op_show, NULL);

/* Dump all operating point */
static ssize_t ops_show(struct sys_device *sys_dev,
			struct sysdev_attribute *attr, char *buf)
{
	int i;
	int len = 0;

	for (i = 0; i < op_array_size; i++) {
		len += dump_op(buf + len, op_array + i);
		len += sprintf(buf + len, "\n");
	}

	return len;
}
SYSDEV_ATTR(ops, 0444, ops_show, NULL);

static ssize_t min_freq_show(struct sys_device *sys_dev,
			struct sysdev_attribute *attr, char *buf)
{
	int len;
	int min_freq = pm_qos_request(PM_QOS_CPUFREQ_MIN);

	if (min_freq > 806)
		min_freq = 806;

	len = sprintf(buf, "%dMHz\n", min_freq);
	return len;
}

static ssize_t min_freq_store(struct sys_device *sys_dev,
			struct sysdev_attribute *attr,
			const char *buf, size_t len)
{
	int new_min_freq;
	if (sscanf(buf, "%u", &new_min_freq) != 1) {
		pr_info("min_freq_store fail! Please retry!\n");
		return len;
	}

	pm_qos_update_request(user_qos_req_min, new_min_freq);
	return len;
}
SYSDEV_ATTR(min_freq, 0644, min_freq_show, min_freq_store);

static ssize_t disable_cpufreq_show(struct sys_device *sys_dev,
			struct sysdev_attribute *attr, char *buf)
{
	int len;

	len = sprintf(buf, "%d\n", constraint_cpufreq_disable);
	return len;
}
SYSDEV_ATTR(disable_cpufreq, 0444, disable_cpufreq_show, NULL);

#ifdef CONFIG_PXA910_ACPU_STATS

static int start_statistic;
static unsigned int prev_timestamp;

/*
 * Add this information into timeslot table.
 * This table records the time cost on different cpu state and different
 * operating points.
 */
int acpu_add_timeslot(int run_time)
{
	unsigned int timestamp, time;

	if (start_statistic == 0) {
		prev_timestamp = 0;
		return 0;
	}

	timestamp = read_timer();
	if (prev_timestamp == 0)
		time = 0;
	else
		time = (timestamp >= prev_timestamp) ? timestamp - prev_timestamp
			: 0xFFFFFFFF - prev_timestamp + timestamp;
	prev_timestamp = timestamp;
	if (run_time == 1)
		cur_md->run_time += time;
	else
		cur_md->idle_time += time;
	return 0;
}


/* Display duty cycles on all operating points */
static ssize_t duty_cycle_show(struct sys_device *sys_dev,
				struct sysdev_attribute *attr, char *buf)
{
	int len, i;
	unsigned int total_ticks;

	total_ticks = 0;
	for (i = 0; i < op_array_size; i++)
		total_ticks += op_array[i].run_time
				+ op_array[i].idle_time;

	if (total_ticks == 0) {
		len = sprintf(buf, "no duty cycle info\n");
		return len;
	}
	len = sprintf(buf, "Duty cycle of operating point list:\n");
	for (i = 0; i < op_array_size; i++) {
		len += sprintf(buf + len, "op%d %s run:%u%% idle:%u%%\n",
			i, op_array[i].name,
			op_array[i].run_time * 100 / total_ticks,
			op_array[i].idle_time * 100 / total_ticks);
	}
	return len;
}

static ssize_t duty_cycle_store(struct sys_device *sys_dev,
			struct sysdev_attribute *attr,
			const char *buf, size_t len)
{
	int start, i;
	if (sscanf(buf, "%u", &start) != 1) {
		pr_info("duty_cycle_store fail! Please retry!\n");
		return len;
	}

	if (start == 0)
		start_statistic = 0;
	else if (start_statistic == 0) {
		start_statistic = 1;
		for (i = 0; i < op_array_size; i++)
				op_array[i].run_time = op_array[i].idle_time = 0;
	}

	return len;
}

SYSDEV_ATTR(duty_cycle, 0644, duty_cycle_show, duty_cycle_store);
#endif

static struct attribute *acpu_attr[] = {
	&attr_cur_op.attr,
	&attr_ops.attr,
	&attr_min_freq.attr,
	&attr_disable_cpufreq.attr,
#ifdef CONFIG_PXA910_ACPU_STATS
	&attr_duty_cycle.attr,
#endif
};

static int acpu_add(struct sys_device *sys_dev)
{
	int i, n;
	int ret;

	n = ARRAY_SIZE(acpu_attr);
	for (i = 0; i < n; i++) {
		ret = sysfs_create_file(&(sys_dev->kobj), acpu_attr[i]);
		if (ret)
			return -EIO;
	}
	return 0;
}

static int acpu_rm(struct sys_device *sys_dev)
{
	int i, n;
	n = ARRAY_SIZE(acpu_attr);
	for (i = 0; i < n; i++)
		sysfs_remove_file(&(sys_dev->kobj), acpu_attr[i]);
	return 0;
}

static int acpu_suspend(struct sys_device *sysdev, pm_message_t pmsg)
{
	return 0;
}

static int acpu_resume(struct sys_device *sysdev)
{
	return 0;
}

static struct sysdev_driver acpu_sysdev_driver = {
	.add		= acpu_add,
	.remove		= acpu_rm,
	.suspend	= acpu_suspend,
	.resume		= acpu_resume,
};


static int __init pxa910_freq_init(void)
{
	struct pxa910_md_opt md;
	unsigned int chip_id;
	struct proc_op_array *proc = NULL;
	int i;
	int ret;

	if (!cpu_is_pxa910())
		return -EIO;

	chip_id = __raw_readl(CIU_CHIP_ID) & 0xffff;
	if (cpu_is_pxa921())
		chip_id = 0xc921;
	else if (cpu_is_pxa918())
		chip_id = 0xc918;

	for (i = 0; i < ARRAY_SIZE(proc_op_arrays); i++) {
		proc = proc_op_arrays + i;
		if (proc->chip_id == chip_id)
			break;
	}
	if (i >= ARRAY_SIZE(proc_op_arrays))
		return -EINVAL;

	op_array = proc->op_array;
	op_array_size = proc->op_array_size;

	pr_info("Initializing cpufreq table for %s\n", proc->cpu_name);

	acpuclk_init_cpufreq_table();

	for (i = 0; i < op_array_size; i++)
		select_clk_src(op_array + i);

	v_buck1 = regulator_get(NULL, "v_buck1");
	if (IS_ERR(v_buck1))
		goto err;

	/* map the last page of SRAM for ddr frequency change */
	sram_last_page = ioremap(SRAM_ADDR_END - PAGE_SIZE, PAGE_SIZE);
	if ((freq_sram_end - freq_sram_start) < PAGE_SIZE)
		memcpy(sram_last_page, freq_sram_start,
		       freq_sram_end - freq_sram_start);
	else
		goto err;

	if (capture_op_info(&md))
		pr_err("Failed to get current op setting\n");

	for (i = 0; i < op_array_size; i++)
		if (md.pclk == op_array[i].pclk)
			cur_md = &op_array[i];

	enable_fc_intr();
	/*
	 * pll2 should be initialized by boot loader and if it is not, we just
	 * report a serious bug here.
	 */
	if (!cpu_is_pxa918() && get_pll2_freq() == 0) {
		printk(KERN_ERR "pll2 is off, please turn it on first\n");
		BUG();
	}

	pm_qos_add_notifier(PM_QOS_CPUFREQ_MIN, &cpufreq_min_notifier);
	pm_qos_add_notifier(PM_QOS_CPUFREQ_DISABLE, &cpufreq_disable_notifier);

	pr_info("cpufreq table init, current frequency is %d MHz\n",
		cur_md->pclk);

	user_qos_req_min = pm_qos_add_request(PM_QOS_CPUFREQ_MIN,
			PM_QOS_DEFAULT_VALUE);

	ret = sysdev_driver_register(&cpu_sysdev_class, &acpu_sysdev_driver);

	return ret;

err:
	pr_err("pxa910_freq_init failed\n");
	return -EIO;
}

subsys_initcall_sync(pxa910_freq_init);
