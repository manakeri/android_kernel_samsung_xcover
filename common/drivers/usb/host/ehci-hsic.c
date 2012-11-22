/*
 * (C) Marvell Inc. 2008 (kvedere@marvell.com)
 * Code Based on ehci-fsl.c & ehci-pxa9xx.c
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <plat/pxa_u2o.h>

static struct pxa_usb_plat_info *info;
static int mmp3_hsic_run (struct usb_hcd *hcd)
{
	u32 hsic_int;
	u32 status;
	int count;

	ehci_run(hcd);
	hsic_int = __raw_readl(info->phybase + HSIC_INT);
	status = __raw_readl(info->regbase + U2xPORTSC);
	/*disable connect irq*/
	hsic_int &= ~HSIC_INT_CONNECT_INT_EN;
	__raw_writel(hsic_int, info->phybase + HSIC_INT);

	/* enable port power and reserved bit 25 */
	status = __raw_readl(info->regbase + U2xPORTSC);
	status |= (0x00001000) | (1<<25);
	/* Clear bits 30:31 for HSIC to be enabled */
	status &= ~(0x3 << 30);
	__raw_writel(status, info->regbase + U2xPORTSC);

	/* test mode: force enable hs */
	status = __raw_readl(info->regbase + U2xPORTSC);
	status &= ~(0xf<<16);
	status |= (0x5<<16);
	__raw_writel(status, info->regbase + U2xPORTSC);

	/* disable test mode */
	status = __raw_readl(info->regbase + U2xPORTSC);
	status &= ~(0xf<<16);
	__raw_writel(status, info->regbase + U2xPORTSC);

	/* check HS ready */
	count = 0x10000;
	do {
		hsic_int = __raw_readl(info->phybase + HSIC_INT);
		status = __raw_readl(info->regbase + U2xPORTSC);
		count--;
	} while ((count >= 0) && !(hsic_int & HSIC_INT_HS_READY) && !(hsic_int & HSIC_INT_CONNECT));
	if (count <= 0) {
		printk(KERN_INFO "HSIC_INT_HS_READY not set: hsic_int 0x%x\n", hsic_int);
		return -EAGAIN;
	}

	/* issue port reset */
	status = __raw_readl(info->regbase + U2xPORTSC);
	status |= (1<<8);
	__raw_writel(status, info->regbase + U2xPORTSC);

	/* check reset done */
	count = 0x10000;
	do {
		status = __raw_readl(info->regbase + U2xPORTSC);
		count--;
	} while ((count >= 0) && !(status & (1<<8)));
	if (count <= 0) {
		printk(KERN_INFO "port reset not done: portsc 0x%x\n", status);
		return -EAGAIN;
	}
	return 0;
}
/* called during probe() after chip reset completes */
static int mmp3_hsic_setup(struct usb_hcd *hcd)
{
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);
	int retval;

	/* EHCI registers start at offset 0x100 */
	ehci->caps = hcd->regs + U2x_CAPREGS_OFFSET;
	ehci->regs = hcd->regs + U2x_CAPREGS_OFFSET +
		HC_LENGTH(ehci_readl(ehci, &ehci->caps->hc_capbase));
	dbg_hcs_params(ehci, "reset");
	dbg_hcc_params(ehci, "reset");

	/* cache this readonly data; minimize chip reads */
	ehci->hcs_params = ehci_readl(ehci, &ehci->caps->hcs_params);

	retval = ehci_halt(ehci);
	if (retval)
		return retval;

	/* data structure init */
	retval = ehci_init(hcd);
	if (retval)
		return retval;

	hcd->has_tt = 1;
	ehci->sbrn = 0x20;

	ehci_reset(ehci);
	return retval;
}

static const struct hc_driver mmp3_hsic_hc_driver = {
	.description = hcd_name,
	.product_desc = "Marvell PXA SOC EHCI Host Controller",
	.hcd_priv_size = sizeof(struct ehci_hcd),

	/*
	 * generic hardware linkage
	 */
	.irq = ehci_irq,
	.flags = HCD_USB2,

	/*
	 * basic lifecycle operations
	 */
	.reset = mmp3_hsic_setup,
	.start = mmp3_hsic_run,
#ifdef	CONFIG_PM
	.bus_suspend = ehci_bus_suspend,
	.bus_resume = ehci_bus_resume,
#endif
	.stop = ehci_stop,
	.shutdown = ehci_shutdown,

	/*
	 * managing i/o requests and associated device resources
	 */
	.urb_enqueue = ehci_urb_enqueue,
	.urb_dequeue = ehci_urb_dequeue,
	.endpoint_disable = ehci_endpoint_disable,

	/*
	 * scheduling support
	 */
	.get_frame_number = ehci_get_frame,

	/*
	 * root hub support
	 */
	.hub_status_data = ehci_hub_status_data,
	.hub_control = ehci_hub_control,
	.bus_suspend = ehci_bus_suspend,
	.bus_resume = ehci_bus_resume,

	.relinquish_port = ehci_relinquish_port,
	.port_handed_over = ehci_port_handed_over,
};

static int mmp3_hsic_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
        struct resource	    *res;
	struct clk *clk = NULL;
	struct usb_hcd *hcd;
	int irq, retval, tmp;

	dev_dbg(&pdev->dev,"Initializing PXA EHCI-SOC USB Controller(U2H)\n");
	info = dev->platform_data;

	/* Enable CKEN_USB */
	clk = clk_get(NULL, "U2OCLK");
	if (IS_ERR(clk)) {
	        printk(KERN_ERR "Cannot get USB OTG clk\n");
		return PTR_ERR(clk);
	}
	clk_enable(clk);


	clk = clk_get(NULL, "HSIC1CLK");
	if (IS_ERR(clk)) {
	        printk(KERN_ERR "Cannot get USB clk\n");
		return PTR_ERR(clk);
	}
	clk_enable(clk);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "u2h");
	info->regbase = (unsigned)ioremap_nocache(res->start, res_size(res));
	if (!info->regbase) {
	        printk(KERN_ERR "Cannot get regbase 0x%x\n", info->regbase);
	        return -ENOMEM;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "u2hphy");
	info->phybase = (unsigned)ioremap_nocache(res->start, res_size(res));
	if (!info->phybase) {
	        printk(KERN_ERR "Cannot get phybase 0x%x\n", info->phybase);
	        retval = -ENODEV;
		goto err1;
	}

	irq = platform_get_irq(pdev, 0);
	if (!irq) {
	        printk( KERN_ERR "Cannot get irq %x\n", irq);
	        retval = -ENODEV;
		goto err1;
	}

	printk("u2h regbase 0x%x phybase 0x%x irq %d\n", info->regbase,
			info->phybase, irq);

	if (!info->phy_init || info->phy_init(info->phybase)) {
		printk(KERN_ERR "unable to init pxa usb 2.0 host controller"
				" phy_init 0x%p\n", info->phy_init);
		retval = -EBUSY;
		goto err1;
	}

	if (!info->vbus_set || info->vbus_set(1)) {
		printk(KERN_ERR "Unable to power USB Host Controller.\n");
		retval = -EBUSY;
		goto err1;
	}

	hcd = usb_create_hcd(&mmp3_hsic_hc_driver, &pdev->dev,
			"MMP3 HSIC");
	if (!hcd) {
		retval = -ENOMEM;
		goto err1;
	}

	hcd->rsrc_start = virt_to_phys((void *)info->regbase);
	hcd->rsrc_len = 0x1ff;
	hcd->regs = (void __iomem*)info->regbase;
	hcd->irq = (unsigned int)irq;

	/* @USBCMD, Reset USB core */
	tmp = readl(hcd->regs + U2xUSBCMD);
	tmp |= U2xUSBCMD_RST;
	writel(tmp,hcd->regs + U2xUSBCMD);
	udelay(1000);

	retval = usb_add_hcd(hcd, hcd->irq, IRQF_DISABLED | IRQF_SHARED);
	if (retval != 0) {
		goto err2;
	}
	platform_set_drvdata(pdev, hcd);
	return retval;

err2:
	usb_put_hcd(hcd);
err1:
	dev_err(&pdev->dev, "init MMP3 HSIC fail, %d\n", retval);
	return retval;
}

static int mmp3_hsic_remove(struct platform_device *pdev)
{
	struct usb_hcd  *hcd = platform_get_drvdata(pdev);

	if (HC_IS_RUNNING(hcd->state))
		hcd->state = HC_STATE_QUIESCING;

	usb_disconnect(&hcd->self.root_hub);
	hcd->driver->stop(hcd);

	usb_remove_hcd(hcd);
	iounmap(hcd->regs);
	iounmap((void *)info->phybase);
	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
	usb_put_hcd(hcd);
	return 0;
}

#ifdef CONFIG_PM
static int mmp3_hsic_suspend (struct platform_device *pdev, pm_message_t message)
{
	struct usb_hcd  *hcd = platform_get_drvdata(pdev);
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);
	struct clk *clk;
	unsigned long flags;
	int    rc = 0;

	if (time_before(jiffies, ehci->next_statechange))
		msleep(10);

	/* Root hub was already suspended. Disable irq emission and
	 * mark HW unaccessible, bail out if RH has been resumed. Use
	 * the spinlock to properly synchronize with possible pending
	 * RH suspend or resume activity.
	 *
	 * This is still racy as hcd->state is manipulated outside of
	 * any locks =P But that will be a different fix.
	 */
	spin_lock_irqsave(&ehci->lock, flags);
	if (hcd->state != HC_STATE_SUSPENDED) {
		rc = -EINVAL;
		goto bail;
	}
	ehci_writel(ehci, 0, &ehci->regs->intr_enable);
	(void)ehci_readl(ehci, &ehci->regs->intr_enable);

	/* make sure snapshot being resumed re-enumerates everything */
	if (message.event == PM_EVENT_PRETHAW) {
		ehci_halt(ehci);
		ehci_reset(ehci);
	}

	clear_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);

bail:
	spin_unlock_irqrestore(&ehci->lock, flags);

	// could save FLADJ in case of Vaux power loss
	// ... we'd only use it to handle clock skew

	clk = clk_get(NULL, "HSIC1CLK");
	if (IS_ERR(clk)) {
		printk(KERN_ERR "Cannot get USB clk\n");
		return PTR_ERR(clk);
	}
	clk_disable(clk);

	if (info->vbus_set && info->vbus_set(0)) {
		printk(KERN_ERR "%s set_power failed\n", __func__);
		return -EIO;
	}
	return rc;
}

static int mmp3_hsic_resume (struct platform_device *pdev)
{
	struct usb_hcd  *hcd = platform_get_drvdata(pdev);
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);
	struct clk	*clk;

	if (info->set_power && info->set_power(1)) {
		printk(KERN_ERR "%s set_power failed\n", __func__);
		return -EBUSY;
	}

	clk = clk_get(NULL, "HSCI1CLK");
	if (IS_ERR(clk)) {
	        printk(KERN_ERR "Cannot get USB clk\n");
		if (info->set_power)
			info->set_power(0);
		return PTR_ERR(clk);
	}
	clk_enable(clk);

	if (!info->phy_init || info->phy_init(info->phybase)) {
		printk("%s phy_init failed\n", __func__);
		clk_disable(clk);
		clk_put(clk);
		if (info->set_power)
			info->set_power(0);
		return 0;
	}

	if (time_before(jiffies, ehci->next_statechange))
		udelay(100);

	/* Mark hardware accessible again */
	set_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);

#if 0
	/* If CF is still set, we maintained PCI Vaux power.
	 * Just undo the effect of ehci_pci_suspend().
	 */
	if (ehci_readl(ehci, &ehci->regs->configured_flag) == FLAG_CF) {
		int	mask = INTR_MASK;

		if (!hcd->self.root_hub->do_remote_wakeup)
			mask &= ~STS_PCD;
		ehci_writel(ehci, mask, &ehci->regs->intr_enable);
		ehci_readl(ehci, &ehci->regs->intr_enable);
		printk("%s end--------------------------------\n", __func__);
		return 0;
	}
#endif

	usb_root_hub_lost_power(hcd->self.root_hub);
	ehci_halt(ehci);
	ehci_reset(ehci);

	/* emptying the schedule aborts any urbs */
	spin_lock_irq(&ehci->lock);
	if (ehci->reclaim)
		end_unlink_async(ehci);
	ehci_work(ehci);
	spin_unlock_irq(&ehci->lock);

	ehci_writel(ehci, ehci->command, &ehci->regs->command);
	ehci_writel(ehci, FLAG_CF, &ehci->regs->configured_flag);
	ehci_readl(ehci, &ehci->regs->command);	/* unblock posted writes */

	/* here we "know" root ports should always stay powered */
	ehci_port_power(ehci, 1);

	hcd->state = HC_STATE_SUSPENDED;

	return 0;
}
#endif

MODULE_ALIAS("mmp3-hsic");

static struct platform_driver mmp3_hsic_driver = {
	.probe		= mmp3_hsic_probe,
	.remove		= mmp3_hsic_remove,
	.shutdown	= usb_hcd_platform_shutdown,
	.driver = {
		   .name = "mmp3-hsic",
		   .bus = &platform_bus_type
		   },
#ifdef CONFIG_PM
	.suspend	= mmp3_hsic_suspend,
	.resume		= mmp3_hsic_resume,
#endif
};
