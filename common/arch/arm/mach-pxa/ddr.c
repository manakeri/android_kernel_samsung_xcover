/*
 * ddr calibration Driver
 *
 * Copyright (C) 2007 Marvell Corporation
 * Idan Bartura <ibartura@marvell.com>
 *
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html

 * (C) Copyright 2007 Marvell International Ltd.
 * All Rights Reserved
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <mach/hardware.h>
#include <mach/pxa95x_pm.h>
#include <asm/io.h>

#define RCI_BIT			0x80000000

#define RCOMP_UPDATE_BIT	0x40000000

#define RCOMP_PCODE_MASK	0x1FC00000	/* Rcomp PCODE */
#define RCOMP_NCODE_MASK	0x003F8000	/* Rcomp NCODE */
#define XCODE_MASK		0x0000000F	/* XCODE from DMCISR2 register */



static unsigned int dmc_base;
static unsigned int is_pxa930;

static irqreturn_t ddr_calibration_handler(int irq, void *dev_id)
{
	unsigned int dmcier, dmcisr, rcomp;
	unsigned int ncode, pcode, pad_vals;

	dmcisr = __raw_readl(dmc_base + DMCISR_OFF);

	if (dmcisr & RCI_BIT) {	/* this is a RCOMP interrupt */

		/* disabling the rcomp interrupt */
		dmcier = __raw_readl(dmc_base + DMCIER_OFF);
		dmcier &= ~RCI_BIT;
		__raw_writel(dmcier, dmc_base + DMCIER_OFF);

		/* calculating and updating pads */
		ncode = __raw_readl(dmc_base + DMCISR_OFF);
		ncode &= RCOMP_NCODE_MASK;
		ncode = ncode >> 15;	/* shift to rightmost 7 bits */
		pcode = __raw_readl(dmc_base + DMCISR_OFF);
		pcode &= RCOMP_PCODE_MASK;
		pcode = pcode >> 22;	/* shift to rightmost 7 bits */

		if (!is_pxa930) {
			unsigned int xcode, sr = 0x1;

			xcode = __raw_readl(dmc_base + DMCISR2_OFF);
			if ((xcode & XCODE_MASK) == 0)
				xcode = 0x6;

			/* NCODE and PCODE are inputs from SV */
			if (pcode == 0x0)
				pcode = 0x10;

			ncode = pcode;

			/* move values to correct position */
			pcode = pcode << 24;
			ncode = ncode << 16;
			xcode = xcode << 8;

			/* put values in one 32bit result */
			pad_vals = pcode | ncode | xcode | sr;

			/* send result to pad registers */
			__raw_writel(pad_vals, dmc_base + PAD_MA_OFF);
			__raw_writel(pad_vals, dmc_base + PAD_MDLSB_OFF);
			__raw_writel(pad_vals, dmc_base + PAD_SDCLK_OFF);
			/*check with ASI!!!!!!!!!!!!!!!!!! if ( CalCond.isDDR_MDPAD_WA ) */
			__raw_writel(pad_vals, dmc_base + PAD_MDMSB_OFF);

		} else {	/* TAVOR P */

			unsigned int nslew, pslew;

			/*The calculation that is done (new formula) here is:
			 *PSLEW= 1.9473 -  0.051*Pcode + 0.1914*Ncode
			 *NSLEW=-2.0786+ 0.2739*Pcode+ 0.0279*Ncode
			 *To gain the desired accuracy, the constant values above are multiplied
			 *by 10,000 (so 0.0435 will become 435) and the math is done as integer math
			 *rather than floating point. When the calculations are done, the result
			 *is divided by 10,000 and rounded up/down to the closest integer value
			 *for PSLEW and NSLEW.
			 */

			pslew = 19473 - 510 * pcode + 1914 * ncode;
			pslew = (unsigned int) (pslew / 10000);
			nslew = 2739 * pcode + 279 * ncode - 20786;
			nslew = (unsigned int) (nslew / 10000);

			/* clear irrelevant bits */
			pslew = pslew & 0x0F;
			nslew = nslew & 0x0F;

			/* move values to correct position */
			pcode = pcode << 24;
			ncode = ncode << 16;
			pslew = pslew << 8;

			/* put values in one 32bit result */
			pad_vals = pcode | ncode | pslew | nslew;

			/* send result to pad registers */
			__raw_writel(pad_vals, dmc_base + PAD_MA_OFF);
			__raw_writel(pad_vals, dmc_base + PAD_MDMSB_OFF);
			__raw_writel(pad_vals, dmc_base + PAD_MDLSB_OFF);
			__raw_writel(pad_vals, dmc_base + PAD_SDRAM_OFF);
			__raw_writel(pad_vals, dmc_base + PAD_SDCLK_OFF);
			/*check with ASI!!!!!!!!!!!!!!!!!! if ( CalCond.isDDR_MDPAD_WA ) */
			__raw_writel(pad_vals, dmc_base + PAD_SDCS_OFF);
			__raw_writel(pad_vals, dmc_base + PAD_SMEM_OFF);
			__raw_writel(pad_vals, dmc_base + PAD_SCLK_OFF);
		}

		/* enabling the rcomp interrupt. */
		dmcier = __raw_readl(dmc_base + DMCIER_OFF);
		dmcier |= RCI_BIT;
		__raw_writel(dmcier, dmc_base + DMCIER_OFF);

		/* set the RCOMP update bit (30) so DMEMC initiates EMPI NOP cycle and
		   programs EMPI pads with values in PAD_XX regs. */
		rcomp = __raw_readl(dmc_base + RCOMP_OFF);
		rcomp |= RCOMP_UPDATE_BIT;
		__raw_writel(rcomp, dmc_base + RCOMP_OFF);

		/* clear the interrupt status */
		dmcisr = __raw_readl(dmc_base + DMCISR_OFF);
		dmcisr |= RCI_BIT;
		__raw_writel(dmcisr, dmc_base + DMCISR_OFF);

		pr_debug("DDR Rcomp calibration\n");

	}

	return IRQ_HANDLED;

}

static int __init ddr_init(void)
{
	unsigned int dmcier, dmcisr, temp;

	if ((cpu_is_pxa95x() && !cpu_is_pxa970()) || cpu_is_pxa935())
		is_pxa930 = 0;
	else if (cpu_is_pxa930())
		is_pxa930 = 1;
	else {
		pr_err("DDR calibration is only for pxa93x, pxa955 and pxa968\n");
		return 0;
	}

	/* signing up to the interupt controler */
	temp = request_irq(IRQ_DMEMC, ddr_calibration_handler,
			   IRQF_DISABLED, "pxa9xx-dmemc", NULL);

	dmc_base =
	    (unsigned int) ioremap(DMC_START, DMC_END - DMC_START + 1);

	/* clearing both RCOMP interupt from the status register. */
	dmcisr = __raw_readl(dmc_base + DMCISR_OFF);
	dmcisr |= RCI_BIT;
	__raw_writel(dmcisr, dmc_base + DMCISR_OFF);

	/* enabling RCOMP interupt from dmemc. */
	dmcier = __raw_readl(dmc_base + DMCIER_OFF);
	dmcier |= RCI_BIT;
	__raw_writel(dmcier, dmc_base + DMCIER_OFF);

	return 0;
}

module_init(ddr_init);
