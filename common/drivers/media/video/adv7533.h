/*******************************************************************
 *	Copyright 2009 Marvell Corporation
 *	Copyright (c) 2010, Marvell International Ltd (qingx@marvell.com)
 *
 *    DESCRIPTION:
 *
 *    AUTHOR:  O. Baron
 *
 *    Date Created:  17/1/2011 5:28PM
 *
 *    FILENAME: adi7533.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *******************************************************************/

#ifndef ADV7533_H
#define ADV7533_H

#define	ADV7533_PACKET	0
#define	ADV7533_MAIN	1
#define	ADV7533_CEC_DSI	2
#define	ADV7533_EDID	3
#define	ADV7533_MAX		3

#define ADV7533_I2C_MAX_NUM_OF_BYTES 0xFF
#define ADV7533_I2C_ADDR_CEC_DSI	0x58

#define ADI7533_MAIN_CEC_DSI_ADDR	0xE1

#define ADV7533_MAIN_HPD_REG		0x42
#define ADV7533_MAIN_HPD_BIT		(0x1u<<6)
#define ADV7533_MAIN_POWER_DOWN_REG	0x41
#define ADV7533_MAIN_POWER_DOWN_BIT	(0x1u<<6)
#define ADV7533_MAIN_SYNC_REG		0x17
#define ADV7533_MAIN_SYNC_H			(0x1u<<5)
#define ADV7533_MAIN_SYNC_V			(0x1u<<6)

#define ADV7533_CEC_DSI_INTERNAL_TIMING		0x27
#define ADV7533_CEC_DSI_TOTAL_WIDTH_H_REG	0x28
#define ADV7533_CEC_DSI_TOTAL_WIDTH_L_REG	0x29
#define ADV7533_CEC_DSI_HSYNC_H_REG			0x2A
#define ADV7533_CEC_DSI_HSYNC_L_REG			0x2B
#define ADV7533_CEC_DSI_HFP_H_REG			0x2C
#define ADV7533_CEC_DSI_HFP_L_REG			0x2D
#define ADV7533_CEC_DSI_HBP_H_REG			0x2E
#define ADV7533_CEC_DSI_HBP_L_REG			0x2F
#define ADV7533_CEC_DSI_TOTAL_HEIGHT_H_REG	0x30
#define ADV7533_CEC_DSI_TOTAL_HEIGHT_L_REG	0x31
#define ADV7533_CEC_DSI_VSYNC_H_REG			0x32
#define ADV7533_CEC_DSI_VSYNC_L_REG			0x33
#define ADV7533_CEC_DSI_VFP_H_REG			0x34
#define ADV7533_CEC_DSI_VFP_L_REG			0x35
#define ADV7533_CEC_DSI_VBP_H_REG			0x36
#define ADV7533_CEC_DSI_VBP_L_REG			0x37

#define ADV7533_CEC_DSI_TIMING_H_MASK		(0xFF0)
#define ADV7533_CEC_DSI_TIMING_L_MASK		(0x00F)
#define ADV7533_CEC_DSI_TIMING_OFFSET		(4)
#define ADV7533_CEC_DSI_INTERNAL_TIMING_EN	(0x1u<<7)
#endif


