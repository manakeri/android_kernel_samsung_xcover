/*
 * linux/include/linux/mmc/pm.h
 *
 * Author:	Nicolas Pitre
 * Copyright:	(C) 2009 Marvell Technology Group Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef LINUX_MMC_PM_H
#define LINUX_MMC_PM_H

/*
 * These flags are used to describe power management features that
 * some cards (typically SDIO cards) might wish to benefit from when
 * the host system is being suspended.  There are several layers of
 * abstractions involved, from the host controller driver, to the MMC core
 * code, to the SDIO core code, to finally get to the actual SDIO function
 * driver.  This file is therefore used for common definitions shared across
 * all those layers.
 */

typedef unsigned int mmc_pm_flag_t;

#define MMC_PM_KEEP_POWER		(1 << 0)	/* preserve card power during suspend */
#define MMC_PM_WAKE_SDIO_IRQ		(1 << 1)	/* wake up host system on SDIO IRQ assertion */
#define MMC_PM_IGNORE_PM_NOTIFY		(1 << 2)	/* ignore mmc pm notify */
#define MMC_PM_SKIP_RESUME_PROBE	(1 << 3)	/* skip the attempt to reidentify the card in powered resume */
#define MMC_PM_CARD_ALWAYS_ACTIVE	(1 << 4)	/* keep host Low-Level functional during suspend */
#define MMC_PM_IRQ_ALWAYS_ON (1<<5) /* keep host controller's irq enabled during suspend */
#define MMC_PM_FUNC_SUSPENDED		(1 << 6)	/* function is suspended */
#define MMC_PM_VLDO_ALWAYS_ACTIVE	(1 << 7)	/* keep host functional during suspend but switch LDO-off */

#endif
