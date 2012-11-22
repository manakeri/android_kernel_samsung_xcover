/*
    camera.h - PXA camera driver header file

    Copyright (C) 2003, Intel Corporation
    Copyright (C) 2008, Guennadi Liakhovetski <kernel@pengutronix.de>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
*/

#ifndef __ASM_ARCH_CAMERA_H_
#define __ASM_ARCH_CAMERA_H_

#define PXA_CAMERA_MASTER	1
#define PXA_CAMERA_DATAWIDTH_4	2
#define PXA_CAMERA_DATAWIDTH_5	4
#define PXA_CAMERA_DATAWIDTH_8	8
#define PXA_CAMERA_DATAWIDTH_9	0x10
#define PXA_CAMERA_DATAWIDTH_10	0x20
#define PXA_CAMERA_PCLK_EN	0x40
#define PXA_CAMERA_MCLK_EN	0x80
#define PXA_CAMERA_PCP		0x100
#define PXA_CAMERA_HSP		0x200
#define PXA_CAMERA_VSP		0x400

/* for Marvell pxa955 camera driver */

#define CCIC_0 0    // for 2 camera interface
#define CCIC_1 1
#define CCIC_NUM 1	// we temp enable only one ccic controller

#define SENSOR_LOW	0	/* Low resolution sensor, ov7690 ...etc.*/
#define SENSOR_HIGH	1	/* High resolution sensor, ov5642 ...etc.*/
#define SENSOR_NONE	2	/* None sensor seleted */
#define SENSOR_CLOSE	0	/* Sensor clock disable */
#define SENSOR_OPEN	1	/* Sensor clock enable */

struct pxa95x_csi_dev {
	u32 irq_num;
	u32 reg_start;
	void __iomem *regs;
	spinlock_t dev_lock;
	struct clk *axi_clk;
	struct clk *csi_tx_esc;
};

struct pxa95x_cam_pdata {
	unsigned long mclk_mhz;
	struct pxa95x_csi_dev *csidev;
};

struct cam_platform_data {
	unsigned int vsync_gpio;
	int (*init)(void);
	void (*deinit)(void);
	void (*suspend)(void);
	void (*resume)(void);
	void (*sync_to_gpio)(void);
	void (*sync_from_gpio)(void);
	int (*power_set)(int flag);
};

struct sensor_platform_data {
	int id;
	int (*power_on)(int);
	int (*power_off)(int);
	int (*power_set)(int res, int flag);
};

struct pxacamera_platform_data {
	unsigned long flags;
	unsigned long mclk_10khz;
};

extern void pxa_set_camera_info(struct pxacamera_platform_data *);

#endif /* __ASM_ARCH_CAMERA_H_ */
