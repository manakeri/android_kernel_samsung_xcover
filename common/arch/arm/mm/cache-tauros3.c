/*
 * arch/arm/mm/cache-tauros3.c - Tauros3 L2 cache controller support
 *
 * Copyright (C) 2008 Marvell Semiconductor
 * Original Author: Lennert Buytenhek
 * Modified by Peter Liao <pliao@marvell.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 *
 * References:
 * - PJ1 CPU Core Datasheet,
 *   Document ID MV-S104837-01, Rev 0.7, January 24 2008.
 * - PJ4 CPU Core Datasheet,
 *   Document ID MV-S105190-00, Rev 1.1, March 14 2008.
 */
#include <linux/module.h>
#include <linux/compiler.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <asm/cacheflush.h>
#include <asm/hardware/cache-tauros3.h>
#include <asm/io.h>

#define CACHE_LINE_SIZE		32
static void __iomem *tauros3_base = NULL;
static bool tauros3_enable = 0;
static u32 dgb_control_default = 0;
static u32 hidden_dgb_control_default = 0;

static DEFINE_SPINLOCK(tauros3_lock);
static bool tauros3_disabled;

#ifdef CONFIG_PROC_FS
static unsigned char *associativity[] ={"8-way", "16-way"};
static unsigned char *wsize[]={   "reserved(16KB)",
	"16KB",
	"32KB",
	"64KB",
	"128KB",
	"256KB",
	"512KB",
	"reserved(512KB)"
};

static int proc_tauros3_info_read(char *page, char **start, off_t off, int count, int *eof,
		void *data)
{
	char *p = page;
	int len;
	__u32 aux;

	p += sprintf(p, "Tauros3 Information:\n");

	aux = readl(tauros3_base + TAUROS3_CACHE_ID);
	p += sprintf(p, "Implementer : %#02x\n", (aux >> 24) & 0xff);
	p += sprintf(p, "Cache ID    : %#02x\n", (aux >> 10) & 0x3f);
	p += sprintf(p, "Part Number : %#02x\n", (aux >> 6) & 0xf);
	p += sprintf(p, "RTL release : %#02x\n", (aux >> 0) & 0x3f);

	aux = readl(tauros3_base + TAUROS3_CACHE_TYPE);
	p += sprintf(p, "System L2 Cache Associativity : %s\n", associativity[(aux >> 6) & 0x1]);
	p += sprintf(p, "System L2 Cache Way size      : %s\n", wsize[(aux >> 8) & 0x7]);

	aux = readl(tauros3_base + TAUROS3_AUX_CTRL);
	p += sprintf(p, "TAUROS3_AUX_CTRL : %#08x\n", aux);

	aux = readl(tauros3_base + TAUROS3_DEBUG_CTRL);
	p += sprintf(p, "TAUROS3_DEBUG_CTRL : %#08x\n", aux);

	aux = readl(tauros3_base + TAUROS3_HIDDEN_DEBUG_REG);
	p += sprintf(p, "TAUROS3_HIDDEN_DEBUG_REG : %#08x\n", aux);

	len = (p - page) - off;
	if (len < 0)
		len = 0;

	*eof = (len <= count) ? 1 : 0;
	*start = page + off;

	return len;
}

static int proc_tauros3_info_write(struct file *file, const char __user *buffer,
		unsigned long count, void *data)
{
	u8 param[2][32];
	u32 configs[2] = {0};
	u8 *buffer_tmp = kmalloc(count+16, GFP_KERNEL);
	int i;

	memset(buffer_tmp, 0x0, sizeof(buffer_tmp));

	if(copy_from_user(buffer_tmp, buffer, count))
		return -EFAULT;

	sscanf(buffer_tmp, "%s %s\n",  param[0], param[1]);

	for (i=0; i<2; i++)
		configs[i] = simple_strtoul(param[i], NULL, 0);

	if (strcmp(param[0], "reset") == 0)
		writel(dgb_control_default, tauros3_base + TAUROS3_DEBUG_CTRL);
	else
		writel(configs[0], tauros3_base + TAUROS3_DEBUG_CTRL);

	if (strcmp(param[1], "reset") == 0)
		writel(hidden_dgb_control_default, tauros3_base + TAUROS3_HIDDEN_DEBUG_REG);
	else
		writel(configs[1], tauros3_base + TAUROS3_HIDDEN_DEBUG_REG);


	if (buffer_tmp)
		kfree(buffer_tmp);

	return count;
}

#ifdef CONFIG_CACHE_TAUROS3_EVENT_MONITOR_ENABLE
static unsigned int last_counter[3] = {0,0,0};

static unsigned char *event_name[]= {
	"Counter Disabled",
	"CO",
	"DRHIT",
	"DRREQ",
	"DWHIT",
	"DWREQ",
	"DWTREQ",
	"IRHIT",
	"IRREQ",
	"WA",
	"PF",
	"MRHIT",
	"MRREQ",
	"Counter Disabled",
	"Counter Disabled",
	"Counter Disabled"
};
static int proc_tauros3_counter_read(char *page, char **start, off_t off, int count, int *eof,
		void *data)
{
	char *p = page;
	int len, i, cfg0, cfg1, cfg2;
	unsigned int counter[3], delta_counter[3];

	cfg0 = readl(tauros3_base + TAUROS3_EVENT_CNT0_CFG);
	cfg1 = readl(tauros3_base + TAUROS3_EVENT_CNT1_CFG);
	cfg2 = readl(tauros3_base + TAUROS3_EVENT_CNT2_CFG);

	p += sprintf(p, "Tauros3 Event Counter Information:\n\n");
	p += sprintf(p, "TAUROS3_EVENT_CNT_CTRL : %#08x\n", readl(tauros3_base + TAUROS3_EVENT_CNT_CTRL));
	p += sprintf(p, "TAUROS3_EVENT_CNT0_CFG : %#08x[%s]\n", cfg0, event_name[(cfg0 >> 2) & 0xF]);
	p += sprintf(p, "TAUROS3_EVENT_CNT1_CFG : %#08x[%s]\n", cfg1, event_name[(cfg1 >> 2) & 0xF]);
	p += sprintf(p, "TAUROS3_EVENT_CNT2_CFG : %#08x[%s]\n", cfg2, event_name[(cfg2 >> 2) & 0xF]);

	counter[0] = readl(tauros3_base + TAUROS3_EVENT_CNT0_VAL);
	counter[1] = readl(tauros3_base + TAUROS3_EVENT_CNT1_VAL);
	counter[2] = readl(tauros3_base + TAUROS3_EVENT_CNT2_VAL);

	for (i=0; i<3; i++)
		delta_counter[i] = counter[i] - last_counter[i];

	p += sprintf(p, "\n=========================================================================\n");
	p += sprintf(p, "currnet counter 0 1 2 : %12u     %12u     %12u\n",  counter[0],  counter[1],  counter[2]);
	p += sprintf(p, "delta   counter 0 1 2 : %12u     %12u     %12u\n",  delta_counter[0],  delta_counter[1],  delta_counter[2]);

	len = (p - page) - off;
	if (len < 0)
		len = 0;

	*eof = (len <= count) ? 1 : 0;
	*start = page + off;

	memcpy((unsigned char *)last_counter, (unsigned char *)counter, sizeof(last_counter));

	return len;
}


static int proc_tauros3_counter_write(struct file *file, const char __user *buffer,
		unsigned long count, void *data)
{
	u8 param[4][32];
	u32 configs[4] = {0};
	u8 *buffer_tmp = kmalloc(count+16, GFP_KERNEL);
	int i;

	memset(buffer_tmp, 0x0, sizeof(buffer_tmp));

	if(copy_from_user(buffer_tmp, buffer, count))
	{
		if (buffer_tmp)
			kfree(buffer_tmp);
		return -EFAULT;
	}

	sscanf(buffer_tmp, "%s %s %s %s\n",  param[0], param[1], param[2], param[3]);

	if (strcmp(param[0], "reset") == 0)
	{
		writel(0, tauros3_base + TAUROS3_EVENT_CNT_CTRL);
		writel(0, tauros3_base + TAUROS3_EVENT_CNT0_CFG);
		writel(0, tauros3_base + TAUROS3_EVENT_CNT1_CFG);
		writel(0, tauros3_base + TAUROS3_EVENT_CNT2_CFG);
		writel(0, tauros3_base + TAUROS3_EVENT_CNT0_VAL);
		writel(0, tauros3_base + TAUROS3_EVENT_CNT1_VAL);
		writel(0, tauros3_base + TAUROS3_EVENT_CNT2_VAL);

		memset((unsigned char *)last_counter, 0, sizeof(last_counter));

		goto out;
	}

	for (i=0; i<4; i++)
		configs[i] = simple_strtoul(param[i], NULL, 0);

	writel(configs[1], tauros3_base + TAUROS3_EVENT_CNT0_CFG);
	writel(configs[2], tauros3_base + TAUROS3_EVENT_CNT1_CFG);
	writel(configs[3], tauros3_base + TAUROS3_EVENT_CNT2_CFG);
	writel(configs[0], tauros3_base + TAUROS3_EVENT_CNT_CTRL);

out:

	if (buffer_tmp)
		kfree(buffer_tmp);

	return count;
}
#endif /* CONFIG_CACHE_TAUROS3_EVENT_MONITOR_ENABLE */
#endif /* CONFIG_PROC_FS */


/*
 */
static inline void sync_writel(unsigned long val, unsigned long reg,
		unsigned long complete_mask)
{
	unsigned long flags;

	spin_lock_irqsave(&tauros3_lock, flags);

	writel(val, tauros3_base + reg);

	/* wait for the operation to complete */
	while (readl(tauros3_base + reg) & complete_mask)
		;
	spin_unlock_irqrestore(&tauros3_lock, flags);
}

static inline void cache_sync(void)
{
	sync_writel(0, TAUROS3_CACHE_SYNC, 1);
}

static inline void tauros3_inv_all(void)
{
	/* invalidate all ways */
	sync_writel(0x1, TAUROS3_INV_ALL, 0x1);
	cache_sync();
}

void tauros3_clean_all(void)
{
	if (!tauros3_enable)
		return;

	/* clean all */
	sync_writel(0x1, TAUROS3_CLEAN_ALL, 0x1);
	cache_sync();
}
EXPORT_SYMBOL(tauros3_clean_all);


void tauros3_clean_by_mva(unsigned long addr)
{
	if (!tauros3_enable)
		return;

	/* clean by pa */
	sync_writel(__pa(addr), TAUROS3_CLEAN_LINE_PA, 1);
}

EXPORT_SYMBOL(tauros3_clean_by_mva);

void tauros3_clean_range_pa(unsigned long start, unsigned long end)
{
	unsigned long addr;

	if (!tauros3_enable)
		return;

	start &= ~(CACHE_LINE_SIZE - 1);
	for (addr = start; addr < end; addr += CACHE_LINE_SIZE)
		sync_writel(addr, TAUROS3_CLEAN_LINE_PA, 1);
	cache_sync();
}

EXPORT_SYMBOL(tauros3_clean_range_pa);

/*
 * Linux primitives.
 *
 * Note that the end addresses passed to Linux primitives are
 * noninclusive.
 */
static void tauros3_inv_range(unsigned long start, unsigned long end)
{
	unsigned long addr;

	//printk("%s\n\r", __FUNCTION__);
	if (start & (CACHE_LINE_SIZE - 1)) {
		start &= ~(CACHE_LINE_SIZE - 1);
		sync_writel(start, TAUROS3_CLEAN_INV_LINE_PA, 1);
		start += CACHE_LINE_SIZE;
	}

	if (end & (CACHE_LINE_SIZE - 1)) {
		end &= ~(CACHE_LINE_SIZE - 1);
		sync_writel(end, TAUROS3_CLEAN_INV_LINE_PA, 1);
	}

	for (addr = start; addr < end; addr += CACHE_LINE_SIZE)
		sync_writel(addr, TAUROS3_INV_LINE_PA, 1);
	cache_sync();
}

static void tauros3_clean_range(unsigned long start, unsigned long end)
{
	unsigned long addr;

	//printk("%s\n\r", __FUNCTION__);

	start &= ~(CACHE_LINE_SIZE - 1);
	for (addr = start; addr < end; addr += CACHE_LINE_SIZE)
		sync_writel(addr, TAUROS3_CLEAN_LINE_PA, 1);
	cache_sync();
}

static void tauros3_flush_range(unsigned long start, unsigned long end)
{
	unsigned long addr;

	//printk("%s\n\r", __FUNCTION__);

	start &= ~(CACHE_LINE_SIZE - 1);
	for (addr = start; addr < end; addr += CACHE_LINE_SIZE)
		sync_writel(addr, TAUROS3_CLEAN_INV_LINE_PA, 1);
	cache_sync();
}

int __init tauros3_init(void __iomem *base, __u32 aux_val, __u32 aux_mask)
{
	__u32 aux;

#ifdef CONFIG_PROC_FS
	struct proc_dir_entry *res;
	struct proc_dir_entry *res_file;
#endif

	if (tauros3_disabled) {
		printk(KERN_INFO "TAUROS3 cache controller disabled\n");
		return 0;
	}

#ifdef CONFIG_PROC_FS

	res = proc_mkdir("tauros3", NULL);
	if (!res)
		return -ENOMEM;

	/* Create information proc file
	 */
	res_file = create_proc_entry("info", S_IWUSR | S_IRUGO, res);
	if (!res)
		return -ENOMEM;

	res_file->read_proc = proc_tauros3_info_read;
	res_file->write_proc = proc_tauros3_info_write;
#ifdef CONFIG_CACHE_TAUROS3_EVENT_MONITOR_ENABLE
	/* Create counter proc file
	 */
	res_file = create_proc_entry("counter", S_IWUSR | S_IRUGO, res);
	if (!res)
		return -ENOMEM;

	res_file->read_proc = proc_tauros3_counter_read;
	res_file->write_proc = proc_tauros3_counter_write;
#endif /* CONFIG_CACHE_TAUROS3_EVENT_MONITOR_ENABLE */
#endif

	tauros3_base = base;

	/* 1. Write to TAUROS3 Auxiliary Control Register, 0x104
	 *    Setting up Associativity, Way Size, and Latencies
	 */
	aux = readl(tauros3_base + TAUROS3_AUX_CTRL);

	aux &= ~aux_mask;
	aux |= aux_val;

#ifdef CONFIG_CACHE_TAUROS3_FORCE_WRITE_ALLOCATE
	aux &= ~(0x3 << 23);
	aux |= (0x2 <<23);
#endif

#ifdef CONFIG_CACHE_TAUROS3_PREFETCH_ENABLE
	aux &= ~(0x1 << 28);
	aux |= (0x1 <<28);
#endif

#ifdef CONFIG_CACHE_TAUROS3_L2_ECC_ENABLE
	aux &= ~(0x1 << 21);
	aux |= (0x1 <<21);
#endif

#ifdef CONFIG_CACHE_TAUROS3_EVENT_MONITOR_ENABLE
	aux &= ~(0x1 << 20);
	aux |= (0x1 <<20);
#endif

	writel(aux, tauros3_base + TAUROS3_AUX_CTRL);

	/* AUX2
	 */
	aux = readl(tauros3_base + TAUROS3_HIDDEN_DEBUG_REG);

#ifdef CONFIG_CACHE_TAUROS3_CHIP_Z0
	aux &= ~(0x1 << 27);
	aux |= (0x1 <<27);
#endif

#ifdef CONFIG_CACHE_TAUROS3_L2LINEFILL_BURST8_ENABLE
	aux &= ~(0x1 << 2);
	aux |= (0x1 <<2);
#endif

	writel(aux, tauros3_base + TAUROS3_HIDDEN_DEBUG_REG);

	/* 2. Secure write to TAUROS3 Invalidate by Way, 0x77c
	 *    Setting up Associativity, Way Size, and Latencies
	 */

	tauros3_inv_all();

	/* 3. Write to the Lockdown D and Lockdown I Register 9 if required
	 */
#if 0
	aux = readl(tauros3_base + TAUROS3_DEBUG_CTRL);

	aux |= 0x3;

	writel(aux, tauros3_base + TAUROS3_DEBUG_CTRL);

#endif
	/* 4. Write to interrupt clear register, 0x220, to clear any residual raw interrupt set.
	 */
	writel(0x1FF, tauros3_base + TAUROS3_INTR_CLEAR);

	/* 5. Write to Control Register 1, bit-0 to enable the cache
	 */
	writel(1, tauros3_base + TAUROS3_CTRL);

	tauros3_enable = 1;

	dgb_control_default = readl(tauros3_base + TAUROS3_DEBUG_CTRL);
	hidden_dgb_control_default = readl(tauros3_base + TAUROS3_HIDDEN_DEBUG_REG);

	outer_cache.inv_range = tauros3_inv_range;
	outer_cache.clean_range = tauros3_clean_range;
	outer_cache.flush_range = tauros3_flush_range;

	printk(KERN_INFO "Tauros3: System L2 cache support initialised\n");

	return 0;
}

static int __init tauros3_disable(char *unused)
{
	tauros3_disabled = 1;
	return 0;
}
early_param("notauros3", tauros3_disable);
