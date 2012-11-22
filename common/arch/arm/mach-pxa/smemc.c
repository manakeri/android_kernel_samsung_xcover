/*
 * Static Memory Controller
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/sysdev.h>

#include <mach/hardware.h>

#define SMEMC_PHYS_BASE	(0x4A000000)
#define SMEMC_PHYS_SIZE	(0x90)

#define MSC0		(0x08)	/* Static Memory Controller Register 0 */
#define MSC1		(0x0C)	/* Static Memory Controller Register 1 */
#define SXCNFG		(0x1C)	/* Synchronous Static Memory Control Register */
#define MEMCLKCFG	(0x68)	/* Clock Configuration */
#define CSADRCFG0	(0x80)	/* Address Configuration Register for CS0 */
#define CSADRCFG1	(0x84)	/* Address Configuration Register for CS1 */
#define CSADRCFG2	(0x88)	/* Address Configuration Register for CS2 */
#define CSADRCFG3	(0x8C)	/* Address Configuration Register for CS3 */
#define CLK_RET_DEL	(0xB0)
#define ADV_RET_DEL	(0xB4)
#define DFCLK_CTL(x)	(x<<24)
#define DFCLK_CTL_EN	DFCLK_CTL(1)
#define DFCLK_CTLDIV(x)	(x<<16)
#define DFCLK_CTL_FRE	DFCLK_CTLDIV(2)

DEFINE_MUTEX(smc_lock);

#if defined(CONFIG_PM) && !defined(CONFIG_PXA95x)
static void __iomem *smemc_mmio_base;

static unsigned long msc[2];
static unsigned long sxcnfg, memclkcfg;
static unsigned long csadrcfg[4];

static int pxa3xx_smemc_suspend(struct sys_device *dev, pm_message_t state)
{
	mutex_lock(&smc_lock);
	msc[0] = __raw_readl(smemc_mmio_base + MSC0);
	msc[1] = __raw_readl(smemc_mmio_base + MSC1);
	sxcnfg = __raw_readl(smemc_mmio_base + SXCNFG);
	memclkcfg = __raw_readl(smemc_mmio_base + MEMCLKCFG);
	csadrcfg[0] = __raw_readl(smemc_mmio_base + CSADRCFG0);
	csadrcfg[1] = __raw_readl(smemc_mmio_base + CSADRCFG1);
	csadrcfg[2] = __raw_readl(smemc_mmio_base + CSADRCFG2);
	csadrcfg[3] = __raw_readl(smemc_mmio_base + CSADRCFG3);
	mutex_unlock(&smc_lock);
	return 0;
}

static int pxa3xx_smemc_resume(struct sys_device *dev)
{
	mutex_lock(&smc_lock);
	__raw_writel(msc[0], smemc_mmio_base + MSC0);
	__raw_writel(msc[1], smemc_mmio_base + MSC1);
	__raw_writel(sxcnfg, smemc_mmio_base + SXCNFG);
	__raw_writel(memclkcfg, smemc_mmio_base + MEMCLKCFG);
	__raw_writel(csadrcfg[0], smemc_mmio_base + CSADRCFG0);
	__raw_writel(csadrcfg[1], smemc_mmio_base + CSADRCFG1);
	__raw_writel(csadrcfg[2], smemc_mmio_base + CSADRCFG2);
	__raw_writel(csadrcfg[3], smemc_mmio_base + CSADRCFG3);
	mutex_unlock(&smc_lock);
	return 0;
}

static struct sysdev_class smemc_sysclass = {
	.name		= "smemc",
	.suspend	= pxa3xx_smemc_suspend,
	.resume		= pxa3xx_smemc_resume,
};

static struct sys_device smemc_sysdev = {
	.id		= 0,
	.cls		= &smemc_sysclass,
};

static int __init smemc_init(void)
{
	int ret = 0;

	smemc_mmio_base = ioremap(SMEMC_PHYS_BASE, SMEMC_PHYS_SIZE);
	if (smemc_mmio_base == NULL)
		return -ENODEV;

	ret = sysdev_class_register(&smemc_sysclass);
	if (ret)
		return ret;

	ret = sysdev_register(&smemc_sysdev);

	return ret;
}
subsys_initcall(smemc_init);
#endif

#if (defined(CONFIG_MTD_ONENAND) || defined(CONFIG_MTD_ONENAND_MODULE))
static unsigned char __iomem *smc_mbase = NULL;

void onenand_mmcontrol_smc_cfg(void)
{
	unsigned int csadrcfg2, msc1, sxcnfg;
	if(smc_mbase == NULL)
		smc_mbase = ioremap(SMEMC_PHYS_BASE, SMEMC_PHYS_SIZE);
	mutex_lock(&smc_lock);
	csadrcfg2 = readl(smc_mbase + CSADRCFG2);
	msc1 = readl(smc_mbase + MSC1);
	sxcnfg = readl(smc_mbase + SXCNFG);
	__raw_writel(0x0032091d, smc_mbase + CSADRCFG2);
	__raw_writel(((msc1 & 0xffff0000) | 0x7f18), smc_mbase + MSC1);
	__raw_writel(((sxcnfg & 0xffff) | 0x30110000), smc_mbase + SXCNFG);
	__raw_writel(0x04, smc_mbase + CLK_RET_DEL);
	__raw_writel(0x00, smc_mbase + ADV_RET_DEL);
	mutex_unlock(&smc_lock);
}

void onenand_sync_clk_cfg(void)
{
	u32 temp;
	if(smc_mbase == NULL)
		smc_mbase = ioremap(SMEMC_PHYS_BASE, SMEMC_PHYS_SIZE);
	mutex_lock(&smc_lock);
	temp = __raw_readl(smc_mbase + MEMCLKCFG);
	/*
	 * bit18~bit16
	 */
	temp &=(~(0x07<<16));
	__raw_writel(temp | DFCLK_CTL_EN | DFCLK_CTL_FRE, smc_mbase + MEMCLKCFG);  /*106Mhz divided by 2*/
	mutex_unlock(&smc_lock);
}
EXPORT_SYMBOL(onenand_mmcontrol_smc_cfg);
EXPORT_SYMBOL(onenand_sync_clk_cfg);
#endif
