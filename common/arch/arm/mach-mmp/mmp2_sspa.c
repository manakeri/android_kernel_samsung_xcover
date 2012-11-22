/*
 *  linux/arch/arm/mach-mmp/pxa688_sspa.c
 *
 *  based on linux/arch/arm/mach-sa1100/ssp.c by Russell King
 *
 *  Copyright (C) 2003 Russell King.
 *  Copyright (C) 2003 Wolfson Microelectronics PLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *  PXA2xx SSP driver.  This provides the generic core for simple
 *  IO-based SSP applications and allows easy port setup for DMA access.
 *
 *  Author: Liam Girdwood <liam.girdwood@wolfsonmicro.com>
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/mutex.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/io.h>

#include <asm/irq.h>
#include <mach/hardware.h>
#include <plat/ssp.h>

static DEFINE_MUTEX(sspa_lock);
static LIST_HEAD(sspa_list);

struct ssp_device *sspa_request(int port, const char *label)
{
	struct ssp_device *ssp = NULL;

	mutex_lock(&sspa_lock);

	list_for_each_entry(ssp, &sspa_list, node) {
		if (ssp->port_id == port && ssp->use_count == 0) {
			ssp->use_count++;
			ssp->label = label;
			break;
		}
	}

	mutex_unlock(&sspa_lock);

	if (&ssp->node == &sspa_list)
		return NULL;

	return ssp;
}

EXPORT_SYMBOL(sspa_request);

void sspa_free(struct ssp_device *ssp)
{
	mutex_lock(&sspa_lock);
	if (ssp->use_count) {
		ssp->use_count--;
		ssp->label = NULL;
	} else
		dev_err(&ssp->pdev->dev, "device already free\n");
	mutex_unlock(&sspa_lock);
}

EXPORT_SYMBOL(sspa_free);

static int __devinit pxa688_sspa_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct ssp_device *ssp;
	int ret = 0;

	ssp = kzalloc(sizeof(struct ssp_device), GFP_KERNEL);
	if (ssp == NULL) {
		dev_err(&pdev->dev, "failed to allocate memory");
		return -ENOMEM;
	}
	ssp->pdev = pdev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "no memory resource defined\n");
		ret = -ENODEV;
		goto err_free;
	}

	res = request_mem_region(res->start, res->end - res->start + 1,
				 pdev->name);
	if (res == NULL) {
		dev_err(&pdev->dev, "failed to request memory resource\n");
		ret = -EBUSY;
		goto err_free;
	}

	ssp->phys_base = res->start;

	ssp->mmio_base = ioremap(res->start, res->end - res->start + 1);
	if (ssp->mmio_base == NULL) {
		dev_err(&pdev->dev, "failed to ioremap() registers\n");
		ret = -ENODEV;
		goto err_free_mem;
	}

	ssp->irq = platform_get_irq(pdev, 0);
	if (ssp->irq < 0) {
		dev_err(&pdev->dev, "no IRQ resource defined\n");
		ret = -ENODEV;
		goto err_free_io;
	}

	res = platform_get_resource(pdev, IORESOURCE_DMA, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "no SSP RX DRCMR defined\n");
		ret = -ENODEV;
		goto err_free_io;
	}
	ssp->drcmr_rx = res->start;

	res = platform_get_resource(pdev, IORESOURCE_DMA, 1);
	if (res == NULL) {
		dev_err(&pdev->dev, "no SSP TX DRCMR defined\n");
		ret = -ENODEV;
		goto err_free_io;
	}
	ssp->drcmr_tx = res->start;

	/* PXA688 SSP ports starts from 1 and the internal pdev->id
	 * starts from 0, do a translation here
	 */
	ssp->port_id = pdev->id + 1;
	ssp->use_count = 0;

	mutex_lock(&sspa_lock);
	list_add(&ssp->node, &sspa_list);
	mutex_unlock(&sspa_lock);

	platform_set_drvdata(pdev, ssp);
	printk(KERN_INFO "mmp2: sspa driverwas loaded\n");
	return 0;

err_free_io:
	iounmap(ssp->mmio_base);
err_free_mem:
	release_mem_region(res->start, res->end - res->start + 1);
err_free:
	kfree(ssp);
	return ret;
}

static int __devexit pxa688_sspa_remove(struct platform_device *pdev)
{
	struct resource *res;
	struct ssp_device *ssp;

	ssp = platform_get_drvdata(pdev);
	if (ssp == NULL)
		return -ENODEV;

	iounmap(ssp->mmio_base);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(res->start, res->end - res->start + 1);

	mutex_lock(&sspa_lock);
	list_del(&ssp->node);
	mutex_unlock(&sspa_lock);

	kfree(ssp);
	return 0;
}

static struct platform_driver pxa688_sspa_driver = {
	.driver = {
		   .name = "mmp2-sspa",
		   },
	.probe = pxa688_sspa_probe,
	.remove = __devexit_p(pxa688_sspa_remove),
};

static int __init pxa_sspa_init(void)
{
	int ret = 0;

	ret = platform_driver_register(&pxa688_sspa_driver);
	if (ret) {
		printk(KERN_ERR "failed to register pxa688_sspa_driver");
		return ret;
	}

	return ret;
}

static void __exit pxa_sspa_exit(void)
{
	platform_driver_unregister(&pxa688_sspa_driver);
}

arch_initcall(pxa_sspa_init);
module_exit(pxa_sspa_exit);

MODULE_DESCRIPTION("PXA688 SSPA driver");
MODULE_LICENSE("GPL");
