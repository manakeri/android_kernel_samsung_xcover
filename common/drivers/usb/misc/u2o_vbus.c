/*
 * VBus driver for Marvell USB
 *
 * Copyright (C) 2010 Marvell International Ltd.
 * 	Haojian Zhuang <haojian.zhuang@marvell.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/slab.h>

#include <asm/io.h>
#include <mach/regs-apmu.h>

#include <plat/vbus.h>

struct u2o_vbus_info {
	struct resource		res;
	struct device		*dev;
	int			irq;
	int			gpio_en;
};

static struct u2o_vbus_info *vbus_data = NULL;

static irqreturn_t u2o_vbus_handler(int irq, void *data)
{
	int status;

	status = pxa_query_vbus();

// VBUS #ifdef CONFIG_PXA_VBUS
	/* notify usb driver if vbus event */
	if (status & VBUS_EVENT)
		pxa_vbus_handler(status);
//#endif
	return IRQ_HANDLED;
}

static int u2o_set_vbus(int enable, int srp)
{
	int ret = 0;

	ret = gpio_request(vbus_data->gpio_en, "VBus En GPIO");
	if (ret) {
		dev_err(vbus_data->dev, "failed to get gpio pin\n");
		goto out;
	}
	gpio_direction_output(vbus_data->gpio_en, enable);
	mdelay(1);
	gpio_free(vbus_data->gpio_en);
out:
	return ret;
}

static int __devinit u2o_vbus_probe(struct platform_device *pdev)
{
	struct u2o_vbus_pdata *pdata = pdev->dev.platform_data;
	struct u2o_vbus_info *vbus;
	struct pxa_vbus_info info;
	int ret = 0;

	if (!pdata) {
		dev_err(&pdev->dev, "platform data is missing\n");
		ret = -EINVAL;
		goto out;
	}

	vbus = kzalloc(sizeof(struct u2o_vbus_info), GFP_KERNEL);
	if (!vbus) {
		ret = -ENOMEM;
		goto out;
	}
	vbus_data = vbus;

	if (!pdata->reg_base || !pdata->reg_end) {
		dev_err(&pdev->dev, "USB register address is missed\n");
		ret = -EINVAL;
		goto out_mem;
	}
	vbus->res.start = pdata->reg_base;
	vbus->res.end = pdata->reg_end;
	vbus->dev = &pdev->dev;

	memset(&info, 0, sizeof(struct pxa_vbus_info));
	info.set_vbus = u2o_set_vbus;
	info.dev = &pdev->dev;
	info.res = &vbus->res;
	pxa_vbus_init(&info);

	/* VBus is controller by GPIO */
	if (pdata->vbus_en) {
		vbus->gpio_en = pdata->vbus_en;
		u2o_set_vbus(VBUS_LOW, 0);
	}

	if (!pdata->vbus_irq) {
		dev_err(&pdev->dev, "IRQ is missed\n");
		goto out_mount;
	}
	vbus->irq = pdata->vbus_irq;
	ret = request_irq(vbus->irq, u2o_vbus_handler, IRQF_SHARED,
			  "u2o_vbus", vbus);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to request IRQ on vbus\n");
		ret = -EINVAL;
		goto out_mount;
	}

	platform_set_drvdata(pdev, vbus);

	return ret;

out_mount:
	pxa_vbus_deinit();
out_mem:
	kfree(vbus);
out:
	vbus_data = NULL;
	return ret;
}

static int __devexit u2o_vbus_remove(struct platform_device *pdev)
{
	struct u2o_vbus_info *vbus = platform_get_drvdata(pdev);
	unsigned int data;

	if (vbus) {
		data = VBUS_A_VALID | VBUS_A_SESSION_VALID
			| VBUS_B_SESSION_VALID
			| VBUS_B_SESSION_END;
		pxa_mask_vbus(data);
		if (vbus->gpio_en) {
			gpio_direction_input(vbus->gpio_en);
			gpio_free(vbus->gpio_en);
		}

		free_irq(vbus->irq, vbus);
		pxa_vbus_deinit();
		platform_set_drvdata(pdev, NULL);
		kfree(vbus);
		vbus_data = NULL;
	}
	return 0;
}

static struct platform_driver u2o_vbus_driver = {
	.driver		= {
		.name	= "u2o_vbus",
		.owner	= THIS_MODULE,
	},
	.probe		= u2o_vbus_probe,
	.remove		= __devexit_p(u2o_vbus_remove),
};

static int __init u2o_vbus_init(void)
{
	return platform_driver_register(&u2o_vbus_driver);
}
module_init(u2o_vbus_init);

static void __exit u2o_vbus_exit(void)
{
	platform_driver_unregister(&u2o_vbus_driver);
}
module_exit(u2o_vbus_exit);

MODULE_DESCRIPTION("Marvell VBUS driver");
MODULE_AUTHOR("Haojian Zhuang <haojian.zhuang@marvell.com>");
MODULE_LICENSE("GPL");

