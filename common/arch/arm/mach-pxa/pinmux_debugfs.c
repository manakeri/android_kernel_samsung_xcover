/*
 * PXA9xx Pinmux print feature in debugfs
 *
 * Copyright (C) 2007 Marvell Corporation
 * Shay Pathov <shayp@marvell.com>
 *
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html

 * (C) Copyright 2007 Marvell International Ltd.
 * All Rights Reserved
 */


#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <asm/system.h>
#include <linux/types.h>

/*
 * Debug fs
 */
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/seq_file.h>
#include <mach/hardware.h>
#include <asm/io.h>
#include <linux/slab.h>

/*
*	For each platform must be selected appropriated pinmux table
*/
#ifdef CONFIG_CPU_PXA955
#include "pinmux_table_pxa955.h"
#endif /* CONFIG_CPU_PXA955 */

#define PXA9XX__PINMUX_NAME "PINMUX"
#define USER_BUF_SIZE 50
#define BASE_ADDRESS 0x40E10000
#define BUFF_SIZE 4096

static ssize_t PXA9xx_PINMUX_read(struct file *file, char __user *userbuf, size_t count, loff_t *ppos);

	/*	The exec names of the enum defined in debug_pm.h*/

static struct dentry *dbgfs_root, *Pinmux_file;

static const struct file_operations PXA9xx_file_op = { \
	.owner		= THIS_MODULE, \
	.read		= PXA9xx_PINMUX_read \
};

static ssize_t PXA9xx_PINMUX_read(struct file *file, char __user *userbuf, size_t count, loff_t *ppos)
{
	char *buf = NULL;
	int sum = 0, i, max_gpio;
	ssize_t ret;
	uint32_t address, value;
	int *Pinmux_offsets = NULL;

#ifdef CONFIG_CPU_PXA955
	if (cpu_is_pxa955() || cpu_is_pxa968()) {
		Pinmux_offsets = Pinmux_offsets_pxa955;
		max_gpio = MAX_GPIO_PXA955;
	}
#endif

	if (Pinmux_offsets == NULL)	{
		printk(KERN_ERR "ERROR: This feature doesnt support this platform\n");
		return 0;
	}
	buf = kmalloc(BUFF_SIZE, GFP_THISNODE);
	if (buf == NULL) {
		printk(KERN_ERR "%s Failed to malloc\n", __func__);
		return -1;
	}
	ret = snprintf(buf, BUFF_SIZE - 1, "*** KERNEL MFPR ***\n");
	sum += ret;

	for (i = 0; i < max_gpio ; i++) {
		address = BASE_ADDRESS + Pinmux_offsets[i];
		value = __raw_readl((void *)&(__REG(address)));
		ret = snprintf(buf + sum, BUFF_SIZE - 1 - sum, "0x%x: 0x%x\n", address, value);
		if (-1 == ret) {
			printk(KERN_ERR "%s Failed to write. Buffer too small\n", __func__);
			kfree(buf);
			return ret;
		}
		sum += ret;
	}
	ret = simple_read_from_buffer(userbuf, count, ppos, buf, sum);
	kfree(buf);
	return ret;

}

int __init pxa9xx_pinmux_init_debugfs(void)
{
	dbgfs_root = debugfs_create_dir(PXA9XX__PINMUX_NAME, NULL);
	if (!(IS_ERR(dbgfs_root) || !dbgfs_root)) {
		Pinmux_file = debugfs_create_file("Pinmux", 0600, dbgfs_root, NULL,  &PXA9xx_file_op);
		if (Pinmux_file)	{
			printk(KERN_WARNING "%s success\n", __func__);
		} else {
			debugfs_remove_recursive(dbgfs_root);
			printk(KERN_ERR "%s Failed\n", __func__);
		}
	} else
		pr_err("pinmux error: debugfs is not available\n");

	return 0;
}

void __exit pxa9xx_pinmux_cleanup_debugfs(void)
{
	debugfs_remove_recursive(dbgfs_root);
}

module_init(pxa9xx_pinmux_init_debugfs);
module_exit(pxa9xx_pinmux_cleanup_debugfs);
