/*
 * linux/sound/soc/pxa/pxa3xx-pcm.h
 *
 * Base on linux/sound/soc/pxa/pxa2xx-pcm.h
 *
 * Copyright (C) 2007 Marvell International Ltd.
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
#ifndef _PXA3XX_PCM_H
#define _PXA3XX_PCM_H

struct pxa3xx_pcm_dma_params {
	char *name;			/* stream identifier */
	u32 dcmd;			/* DMA descriptor dcmd field */
	volatile u32 *drcmr;		/* the DMA request channel to use */
	u32 dev_addr;			/* device physical address for DMA */
};

struct pxa3xx_gpio {
	u32 sys;
	u32	rx;
	u32 tx;
	u32 clk;
	u32 frm;
};

/* platform data */
extern struct snd_soc_platform pxa3xx_soc_platform;

#endif
