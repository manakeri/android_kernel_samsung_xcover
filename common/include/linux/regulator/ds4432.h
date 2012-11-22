/*
 * Interface of DS4432 regulator
 *
 * Copyright (C) 2011 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __LINUX_REGULATOR_DS4432_H
#define	__LINUX_REGULATOR_DS4432_H

/* There are two DAC path in DS4432. Basic control is to adjust the current
   for each path in steps, either in source or sink mode.
   Usually DS4432 may work with a DC convertor to implement a voltage regulator.
   This driver supports current and voltage mode on each path based on init
   data passed from platform/board. When the path is initialized as current
   mode, it works on its basic functions; when initialized as voltage mode, it
   works with a DC-DC converter to do voltage regulation assuming using the
   application model in the DS4432 document
 */
enum {
	DS4432_DCDC_VOLTAGE_TO_CURRENT,
	DS4432_DCDC_CURRENT_TO_VOLTAGE,
};
struct ds4432_dac_data {
	struct regulator_init_data initdat;
	char *name;
	int type;
	int dac_path;		/* DAC path index */
	int cstep_10nA;		/* current step for 1 in regulation control
				   unit is 10 nA
				 */

	/* The following required for DC convertor voltage regulation mode */
	/* Convert voltage (uV) between control current (10nA) on DAC path
	   if >0: sink current
	   if <0: source current
	   On success, returns 0
	 */
	int (*param_convert) (int path, int mode, int iparam, int *oparam);
};
struct ds4432_platform_data {
	unsigned regulator_count;
	struct ds4432_dac_data *regulators;
};

#endif /* __LINUX_REGULATOR_DS4432_H */
