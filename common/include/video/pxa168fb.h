/*
 * linux/arch/arm/mach-mmp/include/mach/pxa168fb.h
 *
 *  Copyright (C) 2009 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_MACH_PXA168FB_H
#define __ASM_MACH_PXA168FB_H

#include <linux/fb.h>
#include <linux/interrupt.h>

/* Dumb interface */
#define PIN_MODE_DUMB_24		0
#define PIN_MODE_DUMB_18_SPI		1
#define PIN_MODE_DUMB_18_GPIO		2
#define PIN_MODE_DUMB_16_SPI		3
#define PIN_MODE_DUMB_16_GPIO		4
#define PIN_MODE_DUMB_12_SPI_GPIO	5
#define PIN_MODE_SMART_18_SPI		6
#define PIN_MODE_SMART_16_SPI		7
#define PIN_MODE_SMART_8_SPI_GPIO	8

/* Dumb interface pin allocation */
#define DUMB_MODE_RGB565		0
#define DUMB_MODE_RGB565_UPPER		1
#define DUMB_MODE_RGB666		2
#define DUMB_MODE_RGB666_UPPER		3
#define DUMB_MODE_RGB444		4
#define DUMB_MODE_RGB444_UPPER		5
#define DUMB_MODE_RGB888		6

/* default fb buffer size WVGA-32bits */
#define DEFAULT_FB_SIZE	(800 * 480 * 4)

/*
 * Buffer pixel format
 * bit0 is for rb swap.
 * bit12 is for Y UorV swap
 */
#define PIX_FMT_RGB565		0
#define PIX_FMT_BGR565		1
#define PIX_FMT_RGB1555		2
#define PIX_FMT_BGR1555		3
#define PIX_FMT_RGB888PACK	4
#define PIX_FMT_BGR888PACK	5
#define PIX_FMT_RGB888UNPACK	6
#define PIX_FMT_BGR888UNPACK	7
#define PIX_FMT_RGBA888		8
#define PIX_FMT_BGRA888		9
#define PIX_FMT_YUV422PACK	10
#define PIX_FMT_YVU422PACK	11
#define PIX_FMT_YUV422PLANAR	12
#define PIX_FMT_YVU422PLANAR	13
#define PIX_FMT_YUV420PLANAR	14
#define PIX_FMT_YVU420PLANAR	15
#define PIX_FMT_PSEUDOCOLOR	20
#define PIX_FMT_UYVY422PACK	(0x1000|PIX_FMT_YUV422PACK)

#endif /* __ASM_MACH_PXA168FB_H */
