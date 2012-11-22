/*
 *  PM Parser
 *
 *  Support for Power management related event parser over PXAxxx
 *
 *  Author:	Moran Raviv
 *  Created:	Dec 24, 2010
 *  Copyright:	(C) Copyright 2010 Marvell International Ltd.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  publishhed by the Free Software Foundation.
 */

#define COMM_SS 0
#define APP_SS 1

extern struct info_head dvfm_trace_list;
extern struct info_head pxa95x_dvfm_op_list;

/* functions */
int get_pm_parser_args_num(int tsIndex);
void pm_parser_display_log(int subsystem);
