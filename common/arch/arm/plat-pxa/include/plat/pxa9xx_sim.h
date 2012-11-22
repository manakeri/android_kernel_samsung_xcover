/*
 * SIM control support
 *
 * Copyright (C) 2011 Marvell Internation Ltd.
 *
 * Michael Zaidman <zmichael@marvell.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#ifndef SIM_H_
#define SIM_H_

#define SIM_DEVNAME			"pxa_sim"
#define SIM_MAJOR			'J'
#define SIM_IOC_MAGIC		SIM_MAJOR
/*
 * The SIM_DB_INIT should be called first
 * after SIM device opening.
 */
#define SIM_SELECT			_IOW(SIM_IOC_MAGIC, 1, int)
#define SIM_D2_LVL_SET		_IOW(SIM_IOC_MAGIC, 2, int)
#define SIM_D2_LVL_GET		_IOW(SIM_IOC_MAGIC, 3, int)
#define SIM_DISCONNECT		_IOW(SIM_IOC_MAGIC, 4, int)
#define SIM_CONNECT			_IOW(SIM_IOC_MAGIC, 5, int)
#define SIM_IOC_MAXNR		5

union sim_ioctl_args {
	unsigned long ul32;
	struct {
		unsigned char	sim_num;
		unsigned char	data_sleep_level;
		unsigned short	reserved;
	} __attribute__((packed)) a;
};
/*
 * SIM numbers definitions for sim_num field.
 */
#define SIM_1				0
#define SIM_2				1

/*
 * DATA SLEEP MFPR bit voltage level definitions
 * for data_sleep_level field.
 */
#define LPM_UCLK_LO			2
#define LPM_UCLK_HI			3


#ifdef __KERNEL__
enum mfprs {
	MFPR_SIM1,
	MFPR_GSIM_UCLK = MFPR_SIM1,
	MFPR_GSIM_UIO,
	MFPR_GSIM_URST,
	MFPR_SIM2,
	MFPR_RF_IF_17_UCLK = MFPR_SIM2,
	MFPR_RF_IF_18_UIO,
	MFPR_RF_IF_19_URST,
	MFPR_MAX_NUM,
};

struct sim_mfp_reg {
	unsigned short reg_num;
	unsigned short obm_val;
	unsigned short new_val;
	unsigned short reserved;
} __attribute__((packed));

struct sim_platform_data {
	struct sim_mfp_reg *sim_mfp_cfg;
	char sim_cards;
};
#endif

#endif /* SIM_H_ */
