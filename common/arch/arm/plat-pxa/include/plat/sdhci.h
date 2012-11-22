/* linux/arch/arm/plat-pxa/include/plat/sdhci.h
 *
 * Copyright 2010 Marvell
 *	Zhangfei Gao <zhangfei.gao@marvell.com>
 *
 * PXA Platform - SDHCI platform data definitions
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef __PLAT_PXA_SDHCI_H
#define __PLAT_PXA_SDHCI_H

#include <linux/mmc/sdhci.h>
#include <linux/mmc/host.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>

struct mmc_host;
struct device;

/* pxa specific flag */
/* require clock free running */
#define PXA_FLAG_DISABLE_CLOCK_GATING (1<<0)
/* card alwayes wired to host, like on-chip emmc */
#define PXA_FLAG_CARD_PERMANENT	(1<<1)
/* Board design supports 8-bit data on SD/SDIO BUS */
#define PXA_FLAG_SD_8_BIT_CAPABLE_SLOT (1<<2)
/* mmc v3 controller could support clock gating */
#define PXA_FLAG_CONTROL_CLK_GATE (1<<3)
/* controller always active during suspend */
#define PXA_FLAG_ACITVE_IN_SUSPEND (1<<4)
/* keep power always on and skip resume probe for Marvell 8686/8688 wifi */
#define PXA_FLAG_SDIO_RESUME (1<<5)
/* keep sdio interrupt enabled during suspend, for 8787 host sleep support on MG1/MG2 */
#define PXA_FLAG_SDIO_IRQ_ALWAYS_ON (1<<6)
// start marvell johnryu
/* no need for card detection in probe procedure, eg, for 8787 wifi/bt/fm sdio card */
#define PXA_FLAG_DISABLE_PROBE_CDDET (1<<7)
// end marvell johnryu
enum sdhci_pin_type {
	SDHCI_PIN_CMD,
	SDHCI_PIN_CLK,
	SDHCI_PIN_DAT0,
	SDHCI_PIN_DAT1,
	SDHCI_PIN_DAT2,
	SDHCI_PIN_DAT3,
	SDHCI_PIN_MAX,
};

struct sdhci_gpio {
	enum sdhci_pin_type	pin_type;
	unsigned short		gpio_num;
	char				*description;
};

struct sdhci_pins_data {
	struct sdhci_gpio   *sdhci_gpios;
	unsigned short      number_of_pins;
};

/**
 * struct pxa_sdhci_platdata() - Platform device data for PXA SDHCI
 * @max_speed: The maximum speed supported.
 * @quirks: quirks of specific device
 * @flags: flags for platform requirement
 * @clk_delay_cycles:
 * 	mmp2: 	each step is roughly 100ps, 5-bit width
 * 	pxa910:	each step is 1ns, 4-bit width
 * @clk_delay_enable: enable clk_delay or not, used on pxa910 & pxa168
 * @clk_delay_sel: select clk_delay, used on pxa910 & pxa168
 * 	0: choose feedback clk
 * 	1: choose feedback clk + delay value
 * 	2: choose internal clk
 * @ext_cd_gpio: gpio pin used for external CD line
 * @ext_cd_gpio_invert: invert values for external CD gpio line
 * @soc_set_timing(): set timing for specific soc
 * @ext_cd_init(): for external card detect initialization
 * @ext_cd_cleanup: for external card detect clean
 * @soc_set_ops: overwrite host ops with soc specific ops
 * @setpower(): used if we choose external power supply for SD
 * @lp_switch(): slot operations needed while going in/out low-power mode
 * @pmmc: for sdio-wifi8688-rfkill device
*/
struct sdhci_pxa;
struct sdhci_pxa_platdata {
	unsigned int	max_speed;
	unsigned int	quirks;
	unsigned int    flags;
	unsigned int    clk_delay_cycles;
	unsigned int	clk_delay_sel;
	unsigned int	ext_cd_gpio;
	int		cd_force_status; /*-1/0/+1 = foceRemoved/noForce/forceInserted */
	int		ext_cd_level;
	bool		ext_cd_gpio_invert;
	bool		clk_delay_enable;
	struct sdhci_pins_data *pins_data;

	void		(*cd_wakelock)(unsigned int); /* general wakelock */
	void            (*get_sdio_wakelock)(unsigned int); /* get wakelock if host has suspend for sdio slot only, for MG1/MG2 only */
	void		(*soc_set_timing)(struct sdhci_host *host,
						struct sdhci_pxa_platdata *pdata);
	int		(*ext_cd_init)(void (*notify_func)(struct platform_device *dev,
						int state), void *data);
	int		(*ext_cd_cleanup)(void (*notify_func)(struct platform_device *dev,
						int state), void *data);
	void		(*soc_set_ops)(struct sdhci_pxa *pxa);

	void		(*setpower)(struct device *, unsigned int);
	/* slot operations needed while going in/out low-power mode */
	int		(*lp_switch)(unsigned int on, int with_card);

#ifdef CONFIG_SD8XXX_RFKILL
	/*for sd8688-rfkill device*/
	struct mmc_host **pmmc;
#endif
	u32		MM4_RETCLK_DEL_paddr;
};

struct sdhci_pxa {
	struct sdhci_host		*host;
	struct sdhci_pxa_platdata	*pdata;
	struct clk			*clk;
	struct resource			*res;
	struct sdhci_ops		*ops;

	u8 clk_enable;
	u8 power_mode;
	u32	MM4_RETCLK_DEL_calib;
};



extern void __init pxa95x_set_mci_info(int id, void *info);

#ifdef CONFIG_ARCH_MMP
int mmc1_idle_switch(u32 on);
#endif

#endif /* __PLAT_PXA_SDHCI_H */
