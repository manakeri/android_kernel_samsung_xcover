/*
 * linux/sound/soc/pxa/pxa910-squ.h
 *
 * Base on linux/sound/soc/pxa/pxa2xx-pcm.h
 *
 * Copyright (C) 2007 Marvell International Ltd.
 * Author: Bin Yang <bin.yang@marvell.com>
 *			 Yael Sheli Chemla<yael.s.shemla@marvell.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
#ifndef _PXA910_SQU_H
#define _PXA910_SQU_H

#if 0
struct pxa910_squ_dma_params {
	char *name;			/* stream identifier */
	u32 dcmd;			/* DMA descriptor dcmd field */
	u32 dev_addr;			/* device physical address for DMA */
};
#endif

/* platform data */
extern struct snd_soc_platform pxa910_soc_platform;

#endif
