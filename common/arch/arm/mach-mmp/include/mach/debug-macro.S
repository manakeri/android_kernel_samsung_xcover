/* arch/arm/mach-mmp/include/mach/debug-macro.S
 *
 * Debugging macro include header
 *
 *  Copied from arch/arm/mach-pxa/include/mach/debug.S
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <mach/addr-map.h>

		.macro	addruart, rx, tmp
		mrc	p15, 0, \rx, c1, c0
		tst	\rx, #1				@ MMU enabled?
		ldreq	\rx, =APB_PHYS_BASE		@ physical
		ldrne	\rx, =APB_VIRT_BASE		@ virtual
#if defined(CONFIG_MACH_BROWNSTONE) || defined(CONFIG_MACH_MARVELL_JASPER) \
	|| defined(CONFIG_MACH_AVENGERS_LITE)
		orr	\rx, \rx, #0x00018000
#elif defined(CONFIG_MACH_ABILENE)
 		orr	\rx, \rx, #0x00030000
#else
		orr	\rx, \rx, #0x00017000
#endif
		.endm

#define UART_SHIFT	2
#include <asm/hardware/debug-8250.S>
