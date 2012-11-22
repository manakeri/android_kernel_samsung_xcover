/*
 * Marvell VBus abstraction layer
 *
 * Copyright (C) 2010 Marvell International Ltd.
 * 	Haojian Zhuang <haojian.zhuang@marvell.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <asm/io.h>
#include <plat/vbus.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/slab.h>

static struct pxa_vbus_info *vbus_info = NULL;

static int prepare_vbus(int enable)
{
	return 0;
}

static int set_vbus(int enable, int srp)
{
	return 0;
}

/* return vbus is high or low */
static int query_vbus(void)
{
	unsigned int ret;
	int status = 0;

	clk_enable(vbus_info->clk);
	ret = __raw_readl(vbus_info->base + USB_OTGSC);
	clk_disable(vbus_info->clk);

	if (ret & OTGSC_INTS_MASK)
		status |= VBUS_EVENT;

	if (ret & OTGSC_ASV)
		status |= VBUS_HIGH;
	else
		status |= VBUS_LOW;
	return status;
}

extern unsigned int mv_usb_otgsc;
static int query_usbid(void)
{
	unsigned long flags;
	unsigned int ret;
	int event = 0;

	local_irq_save(flags);
	/* events are cleared in interrupt */
	ret = mv_usb_otgsc;
	local_irq_restore(flags);

	if (ret & OTGSC_IDIS) {
		if (ret & OTGSC_ID)
			event |= OTG_INT_IDR;
		else
			event |= OTG_INT_IDF;
	}

	return event;
}

static int query_session(void)
{
	unsigned long flags;
	unsigned int ret;
	int event = 0;

	local_irq_save(flags);
	/* events are cleared in interrupt */
	ret = mv_usb_otgsc;
	local_irq_restore(flags);

	if (ret & OTGSC_AVVIS) {
		if (ret & OTGSC_AVV)
			event |= OTG_INT_RVV;
		else
			event |= OTG_INT_FVV;
	}
	if (ret & OTGSC_ASVIS) {
		if (ret & OTGSC_ASV)
			event |= OTG_INT_RSV;
		else
			event |= OTG_INT_FSV;
	}
	if (ret & OTGSC_BSVIS) {
		if (ret & OTGSC_BSV)
			event |= OTG_INT_B_RSV;
		else
			event |= OTG_INT_B_FSV;
	}
	if (ret & OTGSC_BSEIS) {
		if (ret & OTGSC_BSE)
			event |= OTG_INT_B_RSE;
		else
			event |= OTG_INT_B_FSE;
	}

	return event;
}

static int mask_vbus(int flags)
{
	unsigned int data = 0;

	if (flags & VBUS_A_VALID)
		data |= OTGSC_AVVIE;
	if (flags & VBUS_A_SESSION_VALID)
		data |= OTGSC_ASVIE;
	if (flags & VBUS_B_SESSION_VALID)
		data |= OTGSC_BSVIE;
	if (flags & VBUS_B_SESSION_END)
		data |= OTGSC_BSEIE;
	if (flags & VBUS_ID)
		data |= OTGSC_IDIE;
	if (flags & VBUS_DPS)
		data |= OTGSC_DPIE;
	if (flags & VBUS_1MS)
		data |= OTGSC_1MSE;

	/* update OTGSC */
	if (data) {
		data = ~data;
		clk_enable(vbus_info->clk);
		data &= __raw_readl(vbus_info->base + USB_OTGSC);
		__raw_writel(data, vbus_info->base + USB_OTGSC);
		clk_disable(vbus_info->clk);
	}
	return 0;
}

static int unmask_vbus(int flags)
{
	unsigned int data = 0;

	if (flags & VBUS_A_VALID)
		data |= OTGSC_AVVIE;
	if (flags & VBUS_A_SESSION_VALID)
		data |= OTGSC_ASVIE;
	if (flags & VBUS_B_SESSION_VALID)
		data |= OTGSC_BSVIE;
	if (flags & VBUS_B_SESSION_END)
		data |= OTGSC_BSEIE;
	if (flags & VBUS_ID)
		data |= OTGSC_IDIE;
	if (flags & VBUS_DPS)
		data |= OTGSC_DPIE;
	if (flags & VBUS_1MS)
		data |= OTGSC_1MSE;

	/* update OTGSC */
	if (data) {
		clk_enable(vbus_info->clk);
		data |= __raw_readl(vbus_info->base + USB_OTGSC);
		__raw_writel(data, vbus_info->base + USB_OTGSC);
		clk_disable(vbus_info->clk);
	}
	return 0;
}

int pxa_set_usbdev(int usbdev)
{
	if (vbus_info)
		vbus_info->usbdev = usbdev;
	else
		pr_err("vbus_info memory is not allocated.\n");
	return 0;
}
EXPORT_SYMBOL(pxa_set_usbdev);

int pxa_query_usbdev(void)
{
	int ret = -EINVAL;

	if (vbus_info)
		ret = vbus_info->usbdev;
	return ret;
}
EXPORT_SYMBOL(pxa_query_usbdev);

int pxa_prepare_vbus(int enable)
{
	int ret = -EINVAL;

	if (vbus_info && vbus_info->prepare_vbus)
		ret = vbus_info->prepare_vbus(enable);
	return ret;
}
EXPORT_SYMBOL(pxa_prepare_vbus);

int pxa_set_vbus(int enable, int srp)
{
	int ret = -EINVAL;

	if (vbus_info && vbus_info->set_vbus)

		ret = vbus_info->set_vbus(enable, srp);
	return ret;
}
EXPORT_SYMBOL(pxa_set_vbus);

int pxa_query_vbus(void)
{
	int ret = -EINVAL;

	if (vbus_info && vbus_info->query_vbus)
		ret = vbus_info->query_vbus();
	return ret;
}
EXPORT_SYMBOL(pxa_query_vbus);

int pxa_query_usbid(void)
{
	int ret = -EINVAL;

	if (vbus_info && vbus_info->query_usbid)
		ret = vbus_info->query_usbid();
	return ret;
}
EXPORT_SYMBOL(pxa_query_usbid);

int pxa_query_session(void)
{
	int ret = -EINVAL;

	if (vbus_info && vbus_info->query_session)
		ret = vbus_info->query_session();
	return ret;
}
EXPORT_SYMBOL(pxa_query_session);

int pxa_mask_vbus(int flags)
{
	int ret = -EINVAL;

	if (vbus_info && vbus_info->mask_vbus)
		ret = vbus_info->mask_vbus(flags);
	return ret;
}
EXPORT_SYMBOL(pxa_mask_vbus);

int pxa_unmask_vbus(int flags)
{
	int ret = -EINVAL;
	if (vbus_info && vbus_info->unmask_vbus)
		ret = vbus_info->unmask_vbus(flags);
	return ret;
}
EXPORT_SYMBOL(pxa_unmask_vbus);

void pxa_vbus_handler(int status)
{
	if (vbus_info && vbus_info->func)
		vbus_info->func(status);
}
EXPORT_SYMBOL(pxa_vbus_handler);

int pxa_register_vbus_event(void (*func)(int))
{
	int ret = -EINVAL;

	if (vbus_info) {
		if (vbus_info->func)
			dev_warn(vbus_info->dev,
				"callback existed and replaced\n");
		vbus_info->func = func;
		ret = 0;
	}
	return ret;
}
EXPORT_SYMBOL(pxa_register_vbus_event);

int pxa_unregister_vbus_event(void (*func)(int))
{
	int ret = -EINVAL;

	if (vbus_info) {
		vbus_info->func = NULL;
		ret = 0;
	}
	return ret;
}
EXPORT_SYMBOL(pxa_unregister_vbus_event);

extern int pxa_usb_phy_init(unsigned base);
int __devinit pxa_vbus_init(struct pxa_vbus_info *info)
{
	int ret, flags;

	if (!info || !info->dev) {
		pr_err("device isn't assigned\n");
		return -EINVAL;
	}

	if (!info || !info->res) {
		dev_err(info->dev, "Wrong parameter is specified\n");
		return -EINVAL;
	}

	if (vbus_info) {
		dev_err(info->dev, "vbus_info is already assigned\n");
		return -EINVAL;
	}

	vbus_info = kzalloc(sizeof(struct pxa_vbus_info), GFP_KERNEL);
	if (!vbus_info) {
		ret = -ENOMEM;
		goto out_mem;
	}

	vbus_info->clk = clk_get(NULL, "U2OCLK");
	if (IS_ERR(vbus_info->clk)) {
		dev_err(info->dev, "clk_get fails.\n");
		ret = -ENODEV;
		goto out_clk;
	}

	vbus_info->dev = info->dev;
	vbus_info->base = (unsigned int)ioremap(info->res->start,
				resource_size(info->res));
	if (!vbus_info->base) {
		ret = -ENOMEM;
		goto out_remap;
	}

	vbus_info->prepare_vbus = prepare_vbus;
	vbus_info->set_vbus = set_vbus;
	vbus_info->query_vbus = query_vbus;
	vbus_info->query_usbid = query_usbid;
	vbus_info->query_session = query_session;
	vbus_info->mask_vbus = mask_vbus;
	vbus_info->unmask_vbus = unmask_vbus;
	vbus_info->vbus_init = NULL;

	if (info->prepare_vbus)
		vbus_info->prepare_vbus = info->prepare_vbus;

	if (info->set_vbus)
		vbus_info->set_vbus = info->set_vbus;
	if (info->query_vbus)
		vbus_info->query_vbus = info->query_vbus;
	if (info->query_usbid)
		vbus_info->query_usbid = info->query_usbid;
	if (info->query_session)
		vbus_info->query_session = info->query_session;
	if (info->mask_vbus)
		vbus_info->mask_vbus = info->mask_vbus;
	if (info->unmask_vbus)
		vbus_info->unmask_vbus = info->unmask_vbus;
	if (info->vbus_init)
		vbus_info->vbus_init = info->vbus_init;

	clk_enable(vbus_info->clk);

	/* disable USB interrupt since VBus and USB is sharing one int */
	flags = __raw_readl(vbus_info->base + USB_INTR);
	__raw_writel(flags & ~INTR_UE, vbus_info->base + USB_INTR);
	flags = __raw_readl(vbus_info->base + USB_STS);
	__raw_writel(flags, vbus_info->base + USB_STS);

	flags = VBUS_A_VALID | VBUS_A_SESSION_VALID | VBUS_B_SESSION_VALID
		| VBUS_B_SESSION_END;
	unmask_vbus(flags);

	flags = __raw_readl(vbus_info->base + USB_OTGSC);
	__raw_writel(flags | OTGSC_IDPU, vbus_info->base + USB_OTGSC);

	clk_disable(vbus_info->clk);

	return 0;

out_remap:
	clk_put(vbus_info->clk);
out_clk:
	kfree(vbus_info);
	vbus_info = NULL;
out_mem:
	return ret;
}
EXPORT_SYMBOL(pxa_vbus_init);

void __devexit pxa_vbus_deinit(void)
{
	if (vbus_info) {
		clk_put(vbus_info->clk);
		iounmap((void *)vbus_info->base);
		kfree(vbus_info);
		vbus_info = NULL;
	}
}
EXPORT_SYMBOL(pxa_vbus_deinit);

