/*
 * linux/sound/soc/pxa/pxa3xx-ssp.h
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
#ifndef _PXA3XX_SSP_H
#define _PXA3XX_SSP_H

/* pxa2xx DAI SSP ID's */
#define PXA3XX_DAI_SSP1			0
#define PXA3XX_DAI_SSP2			1
#define PXA3XX_DAI_SSP3			2
#define PXA3XX_DAI_SSP4			3

/* SSP clock sources */
#define PXA3XX_SSP_CLK_PLL	0
#define PXA3XX_SSP_CLK_EXT	1
#define PXA3XX_SSP_CLK_NET	2
#define PXA3XX_SSP_CLK_AUDIO	3

/* SSP audio dividers */
#define PXA3XX_SSP_AUDIO_DIV_ACDS		0
#define PXA3XX_SSP_AUDIO_DIV_SCDB		1
#define PXA3XX_SSP_DIV_SCR			2
#define PXA3XX_SSP_AUDIO_DIV_ACPS		3

/* SSP ACDS audio dividers values */
#define PXA3XX_SSP_CLK_AUDIO_DIV_1		0
#define PXA3XX_SSP_CLK_AUDIO_DIV_2		1
#define PXA3XX_SSP_CLK_AUDIO_DIV_4		2
#define PXA3XX_SSP_CLK_AUDIO_DIV_8		3
#define PXA3XX_SSP_CLK_AUDIO_DIV_16	4
#define PXA3XX_SSP_CLK_AUDIO_DIV_32	5

/* SSP divider bypass */
#define PXA3XX_SSP_CLK_SCDB_1		0
#define PXA3XX_SSP_CLK_SCDB_4		1
#define PXA3XX_SSP_CLK_SCDB_8		2

/*
 * SSP audio private data
 */
struct ssp_priv {
	struct ssp_device *ssp;
	unsigned int sysclk;
	int dai_fmt;
#ifdef CONFIG_PM
	uint32_t	cr0;
	uint32_t	cr1;
	uint32_t	to;
	uint32_t	psp;
#endif
};

extern struct snd_soc_dai pxa3xx_ssp_dai[5];

#endif
