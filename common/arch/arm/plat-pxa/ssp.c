/*
 *  linux/arch/arm/mach-pxa/ssp.c
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

static const int SSP_TIMEOUT = 100000;

/**
 * ssp_write_word - write a word to the SSP port
 * @data: 32-bit, MSB justified data to write.
 *
 * Wait for a free entry in the SSP transmit FIFO, and write a data
 * word to the SSP port.
 *
 * The caller is expected to perform the necessary locking.
 *
 * Returns:
 *   %-ETIMEDOUT	timeout occurred
 *   0			success
 */
int pxa_ssp_write_word(struct ssp_device *ssp, u32 data)
{
	int timeout = SSP_TIMEOUT;

	while (!(pxa_ssp_read_reg(ssp, SSSR) & SSSR_TNF)) {
		if (!--timeout)
			return -ETIMEDOUT;
		cpu_relax();
	}

	pxa_ssp_write_reg(ssp, SSDR, data);

	return 0;
}
EXPORT_SYMBOL(pxa_ssp_write_word);

/**
 * ssp_read_word - read a word from the SSP port
 *
 * Wait for a data word in the SSP receive FIFO, and return the
 * received data.  Data is LSB justified.
 *
 * Note: Currently, if data is not expected to be received, this
 * function will wait for ever.
 *
 * The caller is expected to perform the necessary locking.
 *
 * Returns:
 *   %-ETIMEDOUT	timeout occurred
 *   32-bit data	success
 */
int pxa_ssp_read_word(struct ssp_device *ssp, u32 *data)
{
	int timeout = SSP_TIMEOUT;

	while (!(pxa_ssp_read_reg(ssp, SSSR) & SSSR_RNE)) {
		if (!--timeout)
			return -ETIMEDOUT;
		cpu_relax();
	}

	*data = pxa_ssp_read_reg(ssp, SSDR);
	return 0;
}
EXPORT_SYMBOL(pxa_ssp_read_word);


/**
 * ssp_flush - flush the transmit and receive FIFOs
 *
 * Wait for the SSP to idle, and ensure that the receive FIFO
 * is empty.
 *
 * The caller is expected to perform the necessary locking.
 */
int pxa_ssp_flush(struct ssp_device *ssp)
{
	int timeout = SSP_TIMEOUT * 2;

#ifdef CONFIG_PXA3xx
	/* ensure TX FIFO is empty instead of not full */
	if (cpu_is_pxa3xx()) {
		while (pxa_ssp_read_reg(ssp, SSSR) & 0xf00) {
			if (!--timeout)
				return -ETIMEDOUT;
			cpu_relax();
		}
		timeout = SSP_TIMEOUT * 2;
	}
#endif

	do {
		while (pxa_ssp_read_reg(ssp, SSSR) & SSSR_RNE) {
			if (!--timeout)
				return -ETIMEDOUT;
			(void)pxa_ssp_read_reg(ssp, SSDR);
		}
		if (!--timeout)
			return -ETIMEDOUT;
	} while (pxa_ssp_read_reg(ssp, SSSR) & SSSR_BSY);

	return 0;
}
EXPORT_SYMBOL(pxa_ssp_flush);

/**
 * ssp_enable - enable the SSP port
 *
 * Turn on the SSP port.
 */
void pxa_ssp_enable(struct ssp_device *ssp)
{
	uint32_t sscr0;

	sscr0 = pxa_ssp_read_reg(ssp, + SSCR0);
	sscr0 |= SSCR0_SSE;
	pxa_ssp_write_reg(ssp, SSCR0, sscr0);
}
EXPORT_SYMBOL(pxa_ssp_enable);


/**
 * ssp_disable - shut down the SSP port
 *
 * Turn off the SSP port, optionally powering it down.
 */
void pxa_ssp_disable(struct ssp_device *ssp)
{
	uint32_t sscr0;

	sscr0 = pxa_ssp_read_reg(ssp, SSCR0);
	sscr0 &= ~SSCR0_SSE;
	pxa_ssp_write_reg(ssp, SSCR0, sscr0);
}
EXPORT_SYMBOL(pxa_ssp_disable);

static DEFINE_MUTEX(ssp_lock);
static LIST_HEAD(ssp_list);

struct ssp_device *pxa_ssp_request(int port, const char *label)
{
	struct ssp_device *ssp = NULL;

	mutex_lock(&ssp_lock);

	list_for_each_entry(ssp, &ssp_list, node) {
		if (ssp->port_id == port && ssp->use_count == 0) {
			ssp->use_count++;
			ssp->label = label;
			break;
		}
	}

	mutex_unlock(&ssp_lock);

	if (&ssp->node == &ssp_list)
		return NULL;

	return ssp;
}
EXPORT_SYMBOL(pxa_ssp_request);

void pxa_ssp_free(struct ssp_device *ssp)
{
	mutex_lock(&ssp_lock);
	if (ssp->use_count) {
		ssp->use_count--;
		ssp->label = NULL;
	} else
		dev_err(&ssp->pdev->dev, "device already free\n");
	mutex_unlock(&ssp_lock);
}
EXPORT_SYMBOL(pxa_ssp_free);

static int __devinit pxa_ssp_probe(struct platform_device *pdev)
{
	const struct platform_device_id *id = platform_get_device_id(pdev);
	struct resource *res;
	struct ssp_device *ssp;
	int ret = 0;

	ssp = kzalloc(sizeof(struct ssp_device), GFP_KERNEL);
	if (ssp == NULL) {
		dev_err(&pdev->dev, "failed to allocate memory");
		return -ENOMEM;
	}
	ssp->pdev = pdev;

	ssp->clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(ssp->clk)) {
		ret = PTR_ERR(ssp->clk);
		goto err_free;
	}

	res = platform_get_resource(pdev, IORESOURCE_DMA, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "no SSP RX DRCMR defined\n");
		ret = -ENODEV;
		goto err_free_clk;
	}
	ssp->drcmr_rx = res->start;

	res = platform_get_resource(pdev, IORESOURCE_DMA, 1);
	if (res == NULL) {
		dev_err(&pdev->dev, "no SSP TX DRCMR defined\n");
		ret = -ENODEV;
		goto err_free_clk;
	}
	ssp->drcmr_tx = res->start;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "no memory resource defined\n");
		ret = -ENODEV;
		goto err_free_clk;
	}

	res = request_mem_region(res->start, resource_size(res),
			pdev->name);
	if (res == NULL) {
		dev_err(&pdev->dev, "failed to request memory resource\n");
		ret = -EBUSY;
		goto err_free_clk;
	}

	ssp->phys_base = res->start;

	ssp->mmio_base = ioremap(res->start, resource_size(res));
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

	/* PXA2xx/3xx SSP ports starts from 1 and the internal pdev->id
	 * starts from 0, do a translation here
	 */
	ssp->port_id = pdev->id + 1;
	ssp->use_count = 0;
	ssp->type = (int)id->driver_data;

	mutex_lock(&ssp_lock);
	list_add(&ssp->node, &ssp_list);
	mutex_unlock(&ssp_lock);

	platform_set_drvdata(pdev, ssp);
	return 0;

err_free_io:
	iounmap(ssp->mmio_base);
err_free_mem:
	release_mem_region(res->start, resource_size(res));
err_free_clk:
	clk_put(ssp->clk);
err_free:
	kfree(ssp);
	return ret;
}

static int __devexit pxa_ssp_remove(struct platform_device *pdev)
{
	struct resource *res;
	struct ssp_device *ssp;

	ssp = platform_get_drvdata(pdev);
	if (ssp == NULL)
		return -ENODEV;

	iounmap(ssp->mmio_base);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(res->start, resource_size(res));

	clk_put(ssp->clk);

	mutex_lock(&ssp_lock);
	list_del(&ssp->node);
	mutex_unlock(&ssp_lock);

	kfree(ssp);
	return 0;
}

static const struct platform_device_id ssp_id_table[] = {
	{ "pxa25x-ssp",		PXA25x_SSP },
	{ "pxa25x-nssp",	PXA25x_NSSP },
	{ "pxa27x-ssp",		PXA27x_SSP },
	{ "pxa168-ssp",		PXA168_SSP },
	{ "pxa910-ssp",		PXA910_SSP },
	{ },
};

static struct platform_driver pxa_ssp_driver = {
	.probe		= pxa_ssp_probe,
	.remove		= __devexit_p(pxa_ssp_remove),
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "pxa2xx-ssp",
	},
	.id_table	= ssp_id_table,
};

static int __init pxa_ssp_init(void)
{
	return platform_driver_register(&pxa_ssp_driver);
}

static void __exit pxa_ssp_exit(void)
{
	platform_driver_unregister(&pxa_ssp_driver);
}

arch_initcall(pxa_ssp_init);
module_exit(pxa_ssp_exit);

MODULE_DESCRIPTION("PXA SSP driver");
MODULE_AUTHOR("Liam Girdwood");
MODULE_LICENSE("GPL");
