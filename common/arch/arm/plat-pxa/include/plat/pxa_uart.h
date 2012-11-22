/*
 * Header file to define the uart platform info structure
 *
 * Copyright (C) 2009 Marvell International Ltd.
 *     Xiaofan Tian <tianxf@marvell.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef PXA_UART_H
#define PXA_UART_H

struct pxa_uart_mach_info {
	int stay_awake_in_suspend;
};

#endif
