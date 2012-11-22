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
#ifndef _PXA9XX_PCM_ABU_H
#define _PXA9XX_PCM_ABU_H

#include <mach/dma.h>
#include "pxa95x-pcm-ssp.h"
#include <plat/ssp.h>
#include <mach/regs-abu.h>
#include <mach/regs-ost.h>
#include "pxa95x-abu.h"

#include <mach/dvfm.h>

/* max bytes for one DMA descriptor */
#define BYTES_PER_DMA_DESC_MAX 8000

/* max buffer bytes for DMA transer, must be page aligned */
#define ABU_DMA_MAX_BUFFER_BYTES   128*1024

/* max ABU dma descripor nums */
#define ABU_DMA_DESC_MAX_NUMS	 (PAGE_SIZE/sizeof(pxa_dma_desc))

/* ABU DMA channel macro define */
#define ABU_DMA_CHANNEL_NUMS	  2

/* the enum define for ABU logical working status */
typedef enum
{
	ABU_PLAYBACK_ONLY_STATUS		= 0,
	ABU_RECORD_ONLY_STATUS			= 1,
	ABU_PLAYBACK_RECORD_STATUS		= 2,
	ABU_IDLE_STATUTS			= 3

}ABU_WORKING_STATUS_ENUM;

struct abu_stream_hw_params
{
	u32  period_bytes;
	u32  period_nums;
	u32  total_bytes;
	u32  total_desc_nums;
	u32  trans_period_nums;
	u32  descs_per_period;
};

/* the structu define for ABU DMA device */
struct abu_dma_device
{
	int dma_ch[ABU_DMA_CHANNEL_NUMS];

	struct pxa95x_pcm_dma_params abu_dma_data[ABU_DMA_CHANNEL_NUMS];

	pxa_dma_desc *p_dma_desc_vaddr[ABU_DMA_CHANNEL_NUMS];

	dma_addr_t dma_desc_paddr[ABU_DMA_CHANNEL_NUMS];

	struct snd_pcm_substream *sub_stream[ABU_DMA_CHANNEL_NUMS];

	bool b_stream_active[ABU_DMA_CHANNEL_NUMS];

	struct abu_stream_hw_params stream_hw_params[ABU_DMA_CHANNEL_NUMS];

	u32 dma_dcsr[ABU_DMA_CHANNEL_NUMS];

	int dma_dvfm_idx[ABU_DMA_CHANNEL_NUMS];

	unsigned char *p_dummy_buf_vaddr;
	dma_addr_t dummy_buf_paddr;
};

/* the structu define for ABU SSP device */
struct abu_ssp_device
{
	/* ABU SSI (SSP) structure, device, status */
	struct ssp_device *ssp;
};

/*  the structure define for ABU whole device ctx (includ SSP and DMA part)  */
struct abu_device_ctx
{
	/* abu reference count for open/close */
	u32 abu_ref_cnt;

	/* the lock for abu device access */
	spinlock_t abu_lock;

	/* current ABU working status */
	ABU_WORKING_STATUS_ENUM abu_status;

	/* abu DVFM device id */
	int abu_ssp_dvfm_idx;

	/* abu function clock */
	struct clk *abu_clk;

	/* ABU DMA part */
	struct abu_dma_device  abu_dma_dev;

	/* ABU SSP part */
	struct abu_ssp_device  abu_ssp_dev;

	/* ABU time for debug usage */
	u32  start_time;
	u32  end_time;

	/* Enable ABU Error Interrupt. Disable by default */
	bool b_err_int_flag;
};

/* inline function */
static inline void pxa95x_abu_reg_dump(void __iomem *aub_reg_base)
{
	pr_debug("[ABU] *******************************************************************\r\n");
	pr_debug("[ABU] pxa95x_abu_reg_dump dump start \r\n");

	/* dump ABU register contents */
	pr_debug("[ABU] ABUCR0 = 0x%x\r\n",
			__raw_readl(aub_reg_base + ABUCR0));

	pr_debug("[ABU] ABUSR = 0x%x\r\n",
			__raw_readl(aub_reg_base + ABUSR));

	pr_debug("[ABU] ABURWR = 0x%x\r\n",
			__raw_readl(aub_reg_base + ABURWR));

	pr_debug("[ABU] ABUCR1 = 0x%x\r\n",
			__raw_readl(aub_reg_base + ABUCR1));

	pr_debug("[ABU] ABUSPR = 0x%x\r\n",
			__raw_readl(aub_reg_base + ABUSPR));

	pr_debug("[ABU] ABUCPR = 0x%x\r\n",
			__raw_readl(aub_reg_base + ABUCPR));

	pr_debug("[ABU] ABUADSR = 0x%x\r\n",
			__raw_readl(aub_reg_base + ABUADSR));

	pr_debug("[ABU] ABUDR = 0x%x\r\n",
			__raw_readl(aub_reg_base + ABUDR));

	/* dump ABU SSI register contents */
	pr_debug("[ABU] SSCR0 = 0x%x\r\n",
			__raw_readl(aub_reg_base + SSCR0));

	pr_debug("[ABU] SSCR1 = 0x%x\r\n",
			__raw_readl(aub_reg_base + SSCR1));

	pr_debug("[ABU] SSSR = 0x%x\r\n",
			__raw_readl(aub_reg_base + SSSR));

	pr_debug("[ABU] SSPSP = 0x%x\r\n",
			__raw_readl(aub_reg_base + SSPSP));

	pr_debug("[ABU] SSTSA = 0x%x\r\n",
			__raw_readl(aub_reg_base + SSTSA));

	pr_debug("[ABU] SSTSS = 0x%x\r\n",
			__raw_readl(aub_reg_base + SSTSS));

	pr_debug("[ABU] SSICR = 0x%x\r\n",
			__raw_readl(aub_reg_base + SSICR));

	pr_debug("[ABU] pxa95x_abu_reg_dump dump stop \r\n");
	pr_debug("[ABU] *******************************************************************\r\n");

	return ;
};




#endif
