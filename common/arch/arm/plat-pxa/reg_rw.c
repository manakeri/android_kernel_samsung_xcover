/*
 *  linux/arch/arm/plat-pxa/include/plat/reg_rw.h
 *
 *  Support for write-only register access on PXA platforms
 *
 *  Copyright (C) 2011 Marvell International Ltd.
 *  Raymond Wu <xywu@marvell.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  publishhed by the Free Software Foundation.
 */

#include <linux/io.h>
#include <linux/spinlock.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <mach/hardware.h>

#include <plat/reg_rw.h>


static LIST_HEAD(reg_list); /* registers list */
typedef struct cache_reg{
	struct list_head list;
	/* physical address of the register */
	u32 pa;
	/* kernel mapped address of the register */
	u32 kva;
	/* If it's an write only register, set the corresponding bit as 1 */
	u32 wo_mask;
	/* If it has been written, set the corresponding bit as 0 */
	u32 written;
	/* register value cache for write-only register */
	u32 val;
} cache_reg_t;

static spinlock_t reg_lock;

int pxa_reg_add(u32 pa, u32 mask)
{
	cache_reg_t *r;
	unsigned long flags;

	spin_lock_irqsave(&reg_lock, flags);
	list_for_each_entry(r, &reg_list, list) {
		if (r->pa == pa) {
			spin_unlock_irqrestore(&reg_lock, flags);
			return PXA_REG_ERR_ALREADY_ADD;
		}
	}

	r = (cache_reg_t *) kzalloc(sizeof(cache_reg_t), GFP_KERNEL);
	if (r == NULL) {
		printk(KERN_ERR"%s OOM \n", __FUNCTION__);
		spin_unlock_irqrestore(&reg_lock, flags);
		return PXA_REG_ERR_FAIL;
	}

	r->pa = pa;
	r->wo_mask = mask;
	r->written = 0xffffffff & r->wo_mask;
	r->kva = ioremap(pa,sizeof(u32));
	r->val = readl(r->kva);

	list_add(&r->list, &reg_list);
	spin_unlock_irqrestore(&reg_lock, flags);
	return PXA_REG_ERR_OK;
}

int pxa_reg_remove(u32 addr)
{
	return PXA_REG_ERR_OK;
}

inline int pxa_reg_read(u32 pa, u32 *val, u32 mask)
{
	cache_reg_t *r;
	u32 t;
	unsigned long flags;

	spin_lock_irqsave(&reg_lock, flags);

	list_for_each_entry(r, &reg_list, list) {
		if (r->pa == pa)
			break;
		if (list_is_last(&r->list, &reg_list)) {
			spin_unlock_irqrestore(&reg_lock, flags);
			printk(KERN_ERR"%s cannot find pa=0x%x in list\n",
				   __FUNCTION__, pa);
			return PXA_REG_ERR_FAIL;
		}
	}

	if (r->written & r->wo_mask & mask) {
		printk(KERN_WARNING"%s some bit cannot be read: "\
			   "pa(0x%x) written(0x%x) wo_mask(0x%x) mask(0x%x)\n",\
			   __FUNCTION__, r->pa, r->written, r->wo_mask, mask);
		spin_unlock_irqrestore(&reg_lock, flags);
		return PXA_REG_ERR_FAIL;
	}

	t = readl(r->kva);
	*val = (t & ~(r->wo_mask)) | (r->val & r->wo_mask);
	spin_unlock_irqrestore(&reg_lock, flags);
	return PXA_REG_ERR_OK;
}

inline int pxa_reg_write(u32 pa, u32 val, u32 mask)
{
	cache_reg_t *r;
	u32 t;
	unsigned long flags;

	spin_lock_irqsave(&reg_lock, flags);

	list_for_each_entry(r, &reg_list, list) {
		if (r->pa == pa)
			break;
		if (list_is_last(&r->list, &reg_list)) {
			spin_unlock_irqrestore(&reg_lock, flags);
			printk(KERN_ERR"%s cannot find pa=0x%x in list\n",
				   __FUNCTION__, pa);
			return PXA_REG_ERR_FAIL;
		}
	}

	t = readl(r->kva);
	t = (t & (~(r->wo_mask))) | (r->val & r->wo_mask);
	t = (t & (~mask)) | (val & mask);
	writel(t, r->kva);
	r->val = t;
	r->written &= ~mask;

	spin_unlock_irqrestore(&reg_lock, flags);
	return PXA_REG_ERR_OK;
}

static int __init reg_pxa_init(void)
{
	spin_lock_init(&reg_lock);
	return 0;
}

static void __exit reg_pxa_exit(void)
{
	return;
}

module_init(reg_pxa_init);
module_exit(reg_pxa_exit);

MODULE_DESCRIPTION("Library for write-only registers access");
MODULE_AUTHOR("Raymond Wu <xywu@marvell.com>");
MODULE_LICENSE("GPL v2");
