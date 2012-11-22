/* linux/drivers/mmc/host/sdhci-pxa.c
 *
 * Copyright 2010 Marvell
 *      Zhangfei Gao <zhangfei.gao@marvell.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/* Supports:
 * SDHCI support for MMP2/PXA910/PXA168
 *
 * Based on sdhci-platfm.c
 */

#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/mmc/host.h>
#include <linux/mmc/sdhci.h>
#include <linux/mmc/card.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/slab.h>

#include <plat/sdhci.h>

#include "sdhci.h"

#define DRIVER_NAME	"sdhci-pxa"


struct platform_device *pdev_sd_mmc;
EXPORT_SYMBOL(pdev_sd_mmc);

/*****************************************************************************\
 *                                                                           *
 * SDHCI core callbacks                                                      *
 *                                                                           *
\*****************************************************************************/
static void set_clock(struct sdhci_host *host, unsigned int clock)
{
	struct sdhci_pxa *pxa = sdhci_priv(host);
	struct sdhci_pxa_platdata *pdata = pxa->pdata;

	if (clock) {
		if (pdata && pdata->soc_set_timing)
			pdata->soc_set_timing(host, pdata);
	}
}

static u32 get_max_clock(struct sdhci_host *host)
{
	struct sdhci_pxa *pxa = sdhci_priv(host);

	return clk_get_rate(pxa->clk);
}

void sdhci_pxa_set_con_clock(struct sdhci_host *host, int clock)
{
	struct sdhci_pxa *pxa = sdhci_priv(host);

	if (clock == 0) {
		if (pxa->clk_enable) {
			clk_disable(pxa->clk);
			pxa->clk_enable = 0;
		}
	} else {
		if(!pxa->clk_enable) {
			clk_enable(pxa->clk);
			pxa->clk_enable = 1;
		}
	}
}

void sdhci_pxa_notify_change(struct platform_device *pdev, int state)
{
	struct sdhci_host *host = platform_get_drvdata(pdev);
	struct sdhci_pxa *pxa = sdhci_priv(host);
	unsigned long flags;

	if (host) {
		spin_lock_irqsave(&host->lock, flags);
		if (state) {
			dev_dbg(&pdev->dev, "card inserted.\n");
			host->flags &= ~SDHCI_DEVICE_DEAD;
			spin_unlock_irqrestore(&host->lock, flags);
			pxa->pdata->setpower(&pdev->dev, 1);
		} else {
			dev_dbg(&pdev->dev, "card removed.\n");
			host->flags |= SDHCI_DEVICE_DEAD;
			spin_unlock_irqrestore(&host->lock, flags);
			pxa->pdata->setpower(&pdev->dev, 0);
		}
		tasklet_schedule(&host->card_tasklet);
	}
}
EXPORT_SYMBOL(sdhci_pxa_notify_change);


void sdhci_pxa_quality_calib(struct sdhci_host *host)
{
	extern u32 sdhci_pxa_mfpr_drive(struct sdhci_pxa_platdata *pdata,
				u32 fast0_medium1_slow2);

	u32 *vaddr, hs, delay_val, mfpr;
	struct sdhci_pxa *pxa = sdhci_priv(host);
	struct sdhci_pxa_platdata *pdata = pxa->pdata;

	if (pxa->MM4_RETCLK_DEL_calib)
		return; /* already done once */

	if (!pdata->MM4_RETCLK_DEL_paddr)
		return; /* Currently set for SD only */

	if (!(host->mmc) || !(host->mmc->card))
		return;
	if (!(host->mmc->card->scr.bus_widths))
		return; /* Tricky, just to know the capabilities are known */

	if (in_interrupt())
		return; /* ioremap is not permitted */

	/* Hit!
	* Read RETCLK_DEL calibration applied by OBM
	* Set differently for Slow/HigSpeed
	* The best values should be
	*               DelayReg   MFPR-drive
	*  Value  Slow:   0x1001   MEDIUM
	*  Value  Fast:   0x3124   FAST
	*/
	vaddr = ioremap(pdata->MM4_RETCLK_DEL_paddr, sizeof(u32));
	if (!vaddr)
		return;

	pxa->MM4_RETCLK_DEL_calib = (u32)vaddr;
	delay_val = readl(vaddr);

	printk(KERN_INFO "%s: clk delay calib=%x from OBM\n",
		mmc_hostname(host->mmc), delay_val);

	hs = mmc_card_highspeed(host->mmc->card);
	if (hs) {
		if (delay_val == 0x20002000) {
			delay_val = 0x20003124;
			writel(delay_val, vaddr);
		}
		mfpr = sdhci_pxa_mfpr_drive(pxa->pdata, 0xFFFFFFF0);/*read*/
	} else {
#if 0 /* =1 is TEST DEBUG code. Should be =0  */
printk("\n !!!!!!!!!  Bad Slow configuration for testing !!!!!!!!!\n\n");
		mfpr = sdhci_pxa_mfpr_drive(pxa->pdata, 0xFFFFFFF0);
#else
		delay_val = 0x20001001; /* default 0x20002000 */
		writel(delay_val, vaddr);
		mfpr = sdhci_pxa_mfpr_drive(pxa->pdata, 1/*set medium*/);
#endif
	}

	printk(KERN_INFO "%s: clk delay calib=%08x used. MFPR=%08x\n",
		mmc_hostname(host->mmc), delay_val, mfpr);

	printk(KERN_INFO "%s: [%s] clock freq %d, HSpeed=%d\n",
		mmc_hostname(host->mmc), dev_name(mmc_dev(host->mmc)),
		mmc_host_clk_rate(host->mmc), (hs != 0));
}


/*****************************************************************************\
 *                                                                           *
 * Device probing/removal                                                    *
 *                                                                           *
\*****************************************************************************/

static int __devinit sdhci_pxa_probe(struct platform_device *pdev)
{
	struct sdhci_pxa_platdata *pdata = pdev->dev.platform_data;
	struct device *dev = &pdev->dev;
	struct sdhci_host *host = NULL;
	struct resource *iomem = NULL;
	struct sdhci_pxa *pxa = NULL;
	int ret, irq;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(dev, "no irq specified\n");
		return irq;
	}

	iomem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!iomem) {
		dev_err(dev, "no memory specified\n");
		return -ENOENT;
	}

	host = sdhci_alloc_host(&pdev->dev, sizeof(struct sdhci_pxa));
	if (IS_ERR(host)) {
		dev_err(dev, "failed to alloc host\n");
		ret = PTR_ERR(host);
		goto out;
	}

	pxa = sdhci_priv(host);
	pxa->host = host;
	pxa->pdata = pdata;
	pxa->clk_enable = 0;

	pxa->ops = kzalloc(sizeof(struct sdhci_ops), GFP_KERNEL);
	if (!pxa->ops) {
		ret = -ENOMEM;
		goto out;
	}

	pxa->clk = clk_get(dev, "PXA-SDHCLK");
	if (IS_ERR(pxa->clk)) {
		dev_err(dev, "failed to get io clock\n");
		ret = PTR_ERR(pxa->clk);
		goto out;
	}

	pxa->res = request_mem_region(iomem->start, resource_size(iomem),
		mmc_hostname(host->mmc));
	if (!pxa->res) {
		dev_err(&pdev->dev, "cannot request region\n");
		ret = -EBUSY;
		goto out;
	}

	host->ioaddr = ioremap(iomem->start, resource_size(iomem));
	if (!host->ioaddr) {
		dev_err(&pdev->dev, "failed to remap registers\n");
		ret = -ENOMEM;
		goto out;
	}

	host->hw_name = "MMC";

	host->irq = irq;
	host->quirks = SDHCI_QUIRK_BROKEN_TIMEOUT_VAL;

	if (pdata && pdata->flags & PXA_FLAG_CARD_PERMANENT) {
		/* on-chip device */
		host->quirks |= SDHCI_QUIRK_BROKEN_CARD_DETECTION;
		host->mmc->caps |= MMC_CAP_NONREMOVABLE;
	}

	if (pdata && pdata->flags & PXA_FLAG_DISABLE_CLOCK_GATING)
		host->mmc->caps |= MMC_CAP_DISABLE_BUS_CLK_GATING;

	// start marvell johnryu
	if (pdata && pdata->flags & PXA_FLAG_DISABLE_PROBE_CDDET)
		host->mmc->caps |= MMC_CAP_DISABLE_PROBE_CDDET;
	// end marvell johnryu

	if (pdata && pdata->quirks)
		host->quirks |= pdata->quirks;

#ifdef MMC_AUTO_RESUME
	/* CARD level: queue always active, thread never down() */
	/*BUG_ON(!pdata);*/
	if (pdata->ext_cd_init)
		host->mmc->pm_flags |= MMC_PM_CARD_ALWAYS_ACTIVE;
	if (pdata && pdata->flags & PXA_FLAG_ACITVE_IN_SUSPEND)
		host->mmc->pm_flags |= MMC_PM_VLDO_ALWAYS_ACTIVE;
#else
#error Configuration obsolet and should be reconsidered
	if (pdata && pdata->flags & PXA_FLAG_ACITVE_IN_SUSPEND)
		host->mmc->pm_flags |= MMC_PM_VLDO_ALWAYS_ACTIVE | MMC_PM_CARD_ALWAYS_ACTIVE;
#endif
	if (pdata && pdata->soc_set_ops)
		pdata->soc_set_ops(pxa);

	pxa->ops->set_clock = set_clock;
	pxa->ops->get_max_clock = get_max_clock;
	host->ops = pxa->ops;

	if (host->ops->set_con_clock)
		host->ops->set_con_clock(host,1);
	else
		clk_enable(pxa->clk);

	if (pdata && pdata->flags & PXA_FLAG_SDIO_RESUME) {
		host->mmc->pm_caps |= MMC_PM_KEEP_POWER |
					MMC_PM_SKIP_RESUME_PROBE;
	}

	if (pdata && pdata->flags & PXA_FLAG_SDIO_IRQ_ALWAYS_ON)
		host->mmc->pm_flags |= MMC_PM_IRQ_ALWAYS_ON;

	/* If slot design supports 8 bit data, indicate this to MMC. */
	if (pdata->flags & PXA_FLAG_SD_8_BIT_CAPABLE_SLOT)
		host->mmc->caps |= MMC_CAP_8_BIT_DATA;

	host->mmc->caps |= MMC_CAP_ERASE;

	ret = sdhci_add_host(host);
	if (ret) {
		dev_err(&pdev->dev, "failed to add host\n");
		goto out;
	}

	if (pxa->pdata->max_speed) {
		host->mmc->f_max = pxa->pdata->max_speed;
		if (!(host->mmc->f_max > 25000000))
			host->mmc->caps &= ~(MMC_CAP_SD_HIGHSPEED | MMC_CAP_MMC_HIGHSPEED);
	}

	platform_set_drvdata(pdev, host);

	if (pdata && pdata->ext_cd_init) {
		pdata->ext_cd_init(&sdhci_pxa_notify_change, pdev);
		pdev_sd_mmc = pdev;
	}

	device_init_wakeup(&pdev->dev, 0);

#ifdef CONFIG_SD8XXX_RFKILL
	if (pxa->pdata->pmmc)
		*pxa->pdata->pmmc = host->mmc;
#endif

	if (host->ops->set_con_clock)
		host->ops->set_con_clock(host,0);

	return 0;
out:
	if (host) {
		if (host->ops->set_con_clock)
			host->ops->set_con_clock(host, 0);
		else
			clk_disable(pxa->clk);

		clk_put(pxa->clk);
		if (pxa->ops)
			kfree(pxa->ops);
		if (host->ioaddr)
			iounmap(host->ioaddr);
		if (pxa->res)
			release_mem_region(pxa->res->start,
					resource_size(pxa->res));
		sdhci_free_host(host);
	}

	return ret;
}

static int __devexit sdhci_pxa_remove(struct platform_device *pdev)
{
	struct sdhci_pxa_platdata *pdata = pdev->dev.platform_data;
	struct sdhci_host *host = platform_get_drvdata(pdev);
	struct sdhci_pxa *pxa = sdhci_priv(host);
	int dead = 0;
	u32 scratch;

	if (host) {
		if (host->ops->set_con_clock)
			host->ops->set_con_clock(host, 1);

		if (pdata && pdata->ext_cd_cleanup)
			pdata->ext_cd_cleanup(&sdhci_pxa_notify_change, pdev);

		scratch = readl(host->ioaddr + SDHCI_INT_STATUS);
		if (scratch == (u32)-1)
			dead = 1;

		sdhci_remove_host(host, dead);

		if (pxa->ops)
			kfree(pxa->ops);
		if (host->ioaddr)
			iounmap(host->ioaddr);
		if (pxa->res)
			release_mem_region(pxa->res->start,
					resource_size(pxa->res));

		if (host->ops->set_con_clock)
			host->ops->set_con_clock(host, 0);
		else
		    clk_disable(pxa->clk);
		clk_put(pxa->clk);

		sdhci_free_host(host);
		platform_set_drvdata(pdev, NULL);
	}

	return 0;
}

#ifdef CONFIG_PM
static int sdhci_pxa_suspend(struct platform_device *dev, pm_message_t state)
{
	struct sdhci_host *host = platform_get_drvdata(dev);
	struct sdhci_pxa *pxa = sdhci_priv(host);
	int ret = 0;

	/* Skip the suspend process if the controller is to be accessed during suspend */
	if (!host || !host->mmc || !(host->mmc->card))
		return ret;
	if (host->mmc->pm_flags & MMC_PM_VLDO_ALWAYS_ACTIVE)
		return ret;

#if defined(CONFIG_DEBUG_FS)
	/* Forced to Skip the suspend process by debugfs*/
	if (host->mmc->skip_suspend)
		return ret;
#endif
	if (host->mmc->suspended)
		return ret; /* already suspended */

	if (device_may_wakeup(&dev->dev))
		enable_irq_wake(host->irq);

	ret = sdhci_suspend_host(host, state);
	if (ret) {
		dev_err(&dev->dev, "Cannot sdhci_pxa_suspend since BUSY\n");
		return ret;
	}

	spin_lock(&host->lock);
	host->mmc->suspended = 1;
	spin_unlock(&host->lock);
	if (pxa->pdata->lp_switch) {
		ret = pxa->pdata->lp_switch(1, (int)host->mmc->card);
		if (ret)
			ret = pxa->pdata->lp_switch(1, (int)host->mmc->card);
	}
	if (ret)
		dev_err(&dev->dev, "Fail to switch MMC LDO off\n");
	
	return 0;
}

static int sdhci_pxa_resume(struct platform_device *dev)
{
	struct sdhci_host *host = platform_get_drvdata(dev);
	struct sdhci_pxa *pxa = sdhci_priv(host);
	int ret = 0;

	/* Skip the resume process if the controller is to be accessed during suspend */
	if (!host || !host->mmc || !(host->mmc->card))
		return ret;
	if (host->mmc->pm_flags & MMC_PM_VLDO_ALWAYS_ACTIVE)
		return ret;

	if (!host->mmc->suspended)
		return ret; /* already resumed */

	if (pxa->pdata->lp_switch) {
		ret = pxa->pdata->lp_switch(0, (int)host->mmc->card);
		if (ret)
			ret = pxa->pdata->lp_switch(0, (int)host->mmc->card);
	}
	spin_lock(&host->lock);
	host->mmc->suspended = 0;
	spin_unlock(&host->lock);
	ret = sdhci_resume_host(host);

	if (device_may_wakeup(&dev->dev))
		disable_irq_wake(host->irq);

	return 0;
}
#else
#define sdhci_pxa_suspend	NULL
#define sdhci_pxa_resume	NULL
#endif

static struct platform_driver sdhci_pxa_driver = {
	.probe		= sdhci_pxa_probe,
	.remove		= sdhci_pxa_remove,
	.suspend	= sdhci_pxa_suspend,
	.resume		= sdhci_pxa_resume,
	.driver		= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
	},
};

/*****************************************************************************\
 *                                                                           *
 * Driver init/exit                                                          *
 *                                                                           *
\*****************************************************************************/

static int __init sdhci_pxa_init(void)
{
	return platform_driver_register(&sdhci_pxa_driver);
}

static void __exit sdhci_pxa_exit(void)
{
	platform_driver_unregister(&sdhci_pxa_driver);
}

module_init(sdhci_pxa_init);
module_exit(sdhci_pxa_exit);

MODULE_DESCRIPTION("SDH controller driver for PXA168/PXA910/MMP2");
MODULE_AUTHOR("Zhangfei Gao <zhangfei.gao@marvell.com>");
MODULE_LICENSE("GPL v2");
