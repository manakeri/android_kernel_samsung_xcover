/*
 * linux/sound/soc/pxa/pxa95x-pcm-abu.c
 *
 * Base on linux/sound/soc/pxa/pxa95x-pcm.c
 *
 * Copyright (C) 2010 Marvell International Ltd.
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



#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>


#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include <mach/dma.h>
#include <mach/io.h>
#include <mach/irqs.h>
#include <mach/hardware.h>
#include <mach/audio.h>

#include "pxa95x-pcm-abu.h"


/* The PCM ABU parameters setting, the same for both playback and record */
static const struct snd_pcm_hardware pxa95x_pcm_abu_hardware_params =
{
	.info	      = SNDRV_PCM_INFO_MMAP |
			SNDRV_PCM_INFO_MMAP_VALID |
			SNDRV_PCM_INFO_INTERLEAVED |
			SNDRV_PCM_INFO_JOINT_DUPLEX |
			SNDRV_PCM_INFO_RESUME |
			SNDRV_PCM_INFO_PAUSE,
	.formats      = SNDRV_PCM_FMTBIT_S16_LE |
		        SNDRV_PCM_FMTBIT_S24_LE |
		        SNDRV_PCM_FMTBIT_S32_LE,
	.period_bytes_min	= 32,
	.period_bytes_max	= 40 * 1024,
	.periods_min		= 1,
	.periods_max		= PAGE_SIZE/sizeof(pxa_dma_desc),
	.buffer_bytes_max   = ABU_DMA_MAX_BUFFER_BYTES,
	.fifo_size	    = 32,
};


static char *pxa95x_pcm_abu_channel_names[1][2] = {
	{"ABU SSI PCM out", "ABU SSI PCM in"}
};

static inline void pxa95x_pcm_abu_start_calc_time(struct abu_device_ctx *p_ctx)
{
#ifdef ABU_DEBUG
	/* 32K timer */
	p_ctx->start_time = __raw_readl((void *)&OSCR4);
#endif
}

static inline void pxa95x_pcm_abu_stop_calc_time(struct abu_device_ctx *p_ctx)
{
#ifdef ABU_DEBUG
	u32 time;

	p_ctx->end_time = __raw_readl((void *)&OSCR4);
	time = p_ctx->end_time - p_ctx->start_time;
	pr_debug("[ABU] time gone %d (32K timer)\r\n", time);
#endif
}

/* ABU DMA power requirement */
static void pxa95x_pcm_abu_set_dvfm_constraint(int abu_dev_idx, struct abu_device_ctx *p_ctx)
{
	pr_debug("[ABU] pxa95x_pcm_abu_set_dvfm_constraint (id %d) disable D1 \r\n", abu_dev_idx);
	pxa95x_pcm_abu_stop_calc_time(p_ctx);

	/* Disable Lowpower mode */
	dvfm_disable_op_name("D1", abu_dev_idx);
}

static void pxa95x_pcm_abu_unset_dvfm_constraint(int abu_dev_idx, struct abu_device_ctx *p_ctx)
{
	pr_debug("[ABU] pxa95x_pcm_abu_unset_dvfm_constraint (id %d) enable D1 \r\n", abu_dev_idx);
	pxa95x_pcm_abu_start_calc_time(p_ctx);

	/* Enable Lowpower mode */
	dvfm_enable_op_name("D1", abu_dev_idx);
}

static inline struct abu_device_ctx * pxa95x_pcm_abu_get_ctx(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = NULL;
	struct abu_device_ctx *p_abu_dev = NULL;

	if (substream){
		rtd = substream->private_data;
		p_abu_dev = rtd->dai->cpu_dai->private_data;
	}

	if (!p_abu_dev){
		pr_err("[ABU]  pxa95x_pcm_abu_get_runtime_ctx can't get correct abu_device_ctx\r\n");
		return NULL;
	}

	return p_abu_dev;

};

static u32  pxa95x_pcm_abu_clear_dma_status(int dma_ch)
{
	int dcsr = DCSR(dma_ch);

	/* clear status firstly */
	DCSR(dma_ch) = dcsr;

	/* In order to clear RASINTR, it need to clear RASIRQEN  */
	if (dcsr & DCSR_RASINTR)
		DCSR(dma_ch) &= (~DCSR_RASIRQEN);

	if ((dcsr & DCSR_RASIRQEN) && (dcsr & DCSR_RUN))
		DCSR(dma_ch) |= DCSR_RASIRQEN;

	/* In order to clear STOPSTATE, it need to clear STOPIRQEN  */
	if (dcsr & DCSR_STOPSTATE)
		DCSR(dma_ch) &= (~DCSR_STOPIRQEN);

	if ((dcsr & DCSR_STOPIRQEN) && (dcsr & DCSR_RUN))
		DCSR(dma_ch) |= DCSR_STOPIRQEN;

	pr_debug("[ABU] dma_ch (%d) , before clean. dcsr = 0x%x, after clean. dcsr = 0x%x\r\n", dma_ch, dcsr, DCSR(dma_ch));

	/* return the previous status */
	return dcsr;
}

static void pxa95x_pcm_abu_dma_irq(int dma_ch, void *p_abu_rtx)
{
	struct abu_device_ctx *p_abu_dev_rtx = (struct abu_device_ctx *)p_abu_rtx;
	struct abu_dma_device *p_abu_dma_dev = &(p_abu_dev_rtx->abu_dma_dev);
	struct ssp_device *p_ssp = p_abu_dev_rtx->abu_ssp_dev.ssp;
	void __iomem * p_abu_mmio_base = p_ssp->mmio_base;
	u32 reg_value = 0;
	int dcsr;
	int i = 0;
	unsigned long flags = 0;

	dcsr = pxa95x_pcm_abu_clear_dma_status(dma_ch);
	pxa95x_abu_reg_dump(p_abu_mmio_base);

	spin_lock_irqsave(&p_abu_dev_rtx->abu_lock, flags);

	for (i = 0; i < ABU_DMA_CHANNEL_NUMS; i++) {

		if ((dma_ch == p_abu_dma_dev->dma_ch[i]) && (p_abu_dma_dev->b_stream_active[i])) {

			/* check if there was ENDINTR happened */
			if (dcsr & DCSR_ENDINTR) {

				pr_debug("[ABU] DMA (%d) ENDINTR  happened \r\n", dma_ch);

				p_abu_dma_dev->stream_hw_params[i].trans_period_nums =
					(p_abu_dma_dev->stream_hw_params[i].trans_period_nums + 1)
					% p_abu_dma_dev->stream_hw_params[i].period_nums;

				snd_pcm_period_elapsed(p_abu_dma_dev->sub_stream[i]);

				/* if the DMA channel was in stop, it can be ready for D1 mode */
				if (dcsr & DCSR_STOPSTATE) {
					DCSR(p_abu_dma_dev->dma_ch[i]) |=  p_abu_dma_dev->dma_dcsr[i];
					pxa95x_pcm_abu_unset_dvfm_constraint(p_abu_dma_dev->dma_dvfm_idx[i], p_abu_dev_rtx);
				}
			}

			/* check if there was RASINTR happened */
			if (dcsr & DCSR_RASINTR) {

				pr_debug("[ABU] DMA (%d) RASINTR happened\r\n", dma_ch);

				/* read ABU statusregister contents */
				reg_value = __raw_readl(p_abu_mmio_base + ABUSR);
				pr_debug("[ABU] ABUSR = 0x%x\r\n", reg_value);

				/* check if it need to clear ABU error */
				if ((reg_value & ABUSR_FIFO_OVERRUN) || (reg_value & ABUSR_FIFO_UNDERRUN) || (reg_value & ABUSR_PLAYBACKTIMEOUT)) {

					__raw_writel(reg_value & 0x0F, p_abu_mmio_base + ABUSR);
					pr_debug("[ABU] after  clean ABUSR = 0x%x\r\n", __raw_readl(p_abu_mmio_base + ABUSR));

					/* reset and reinit SSP of ABU */
					abu_reg_soft_reset(p_abu_mmio_base);
					abu_reg_init_ssp(p_abu_mmio_base);
					abu_reg_enable_disable_ssp(p_abu_mmio_base, true);
				}

				if (dcsr & DCSR_STOPSTATE) {

					/* start DMA transfer now, it need to disable D1 mode */
					pxa95x_pcm_abu_set_dvfm_constraint(p_abu_dma_dev->dma_dvfm_idx[i], p_abu_dev_rtx);

					DDADR(p_abu_dma_dev->dma_ch[i]) = p_abu_dma_dev->dma_desc_paddr[i]
						+ sizeof(pxa_dma_desc) * p_abu_dma_dev->stream_hw_params[i].trans_period_nums
						* p_abu_dma_dev->stream_hw_params[i].descs_per_period;

					pr_debug("[ABU]  DDADR = 0x%x \r\n", DDADR(p_abu_dma_dev->dma_ch[i]));

					DCSR(p_abu_dma_dev->dma_ch[i]) = DCSR_RUN | p_abu_dma_dev->dma_dcsr[i];
				}
			}
		}
	}

	spin_unlock_irqrestore(&p_abu_dev_rtx->abu_lock, flags);

	return;
}

static void pxa95x_pcm_abu_cfg_dma_desc(struct abu_device_ctx * p_abu_dev_ctx,
					u32 period_bytes, u32 period_nums, u32 direction, bool b_active)
{
	struct abu_dma_device *p_abu_dma_dev = &p_abu_dev_ctx->abu_dma_dev;
	u32 bytes_per_desc = 0;
	u32 descs_per_period = 0;
	u32 trailing_bytes = 0;
	pxa_dma_desc *p_dma_desc = NULL;
	dma_addr_t dma_buff_phys;
	dma_addr_t next_desc_phys;
	u32 tmp_dcmd;
	u32 i = 0, j = 0;
	u32 dcmd_width = 0;
	u32 dcmd_burst = 0;

	pr_debug("[ABU] pxa95x_pcm_abu_cfg_dma_desc ++++++++++++++++++ \r\n");

	if (period_bytes <= BYTES_PER_DMA_DESC_MAX) {

		bytes_per_desc = period_bytes;
		descs_per_period = 1;

	} else {

		bytes_per_desc = BYTES_PER_DMA_DESC_MAX;
		descs_per_period = period_bytes/BYTES_PER_DMA_DESC_MAX;
		trailing_bytes = period_bytes % BYTES_PER_DMA_DESC_MAX;
		if (trailing_bytes)
			descs_per_period++;
	}

	p_abu_dma_dev->stream_hw_params[direction].total_bytes = period_bytes*period_nums;
	p_abu_dma_dev->stream_hw_params[direction].period_bytes = period_bytes;
	p_abu_dma_dev->stream_hw_params[direction].period_nums = period_nums;
	p_abu_dma_dev->stream_hw_params[direction].total_desc_nums = descs_per_period*period_nums;
	p_abu_dma_dev->stream_hw_params[direction].trans_period_nums = 0;
	p_abu_dma_dev->stream_hw_params[direction].descs_per_period = descs_per_period;


	pr_debug("[ABU] active %d, dir %d, p_bytes = %d, p_nums = %d, bytes_pdesc = %d, descs = %d \r\n",
		(u32)b_active, direction, period_bytes, period_nums, bytes_per_desc, descs_per_period);

	/* dma cmd width. For ABU DMA, it only support (16 bits, 16 bytes burst )  */
	dcmd_width = DCMD_WIDTH2;
	dcmd_burst = DCMD_BURST16;

	/* DMA drcmr/dcmd setting */
	if (direction == SNDRV_PCM_STREAM_PLAYBACK) { /* for playback */

		tmp_dcmd = DCMD_FLOWTRG | dcmd_burst | dcmd_width;

		if (b_active)
			tmp_dcmd |= DCMD_INCSRCADDR;

	} else { /* for recording */

		tmp_dcmd = DCMD_FLOWSRC | dcmd_burst | dcmd_width;

		if (b_active)
			tmp_dcmd |= DCMD_INCTRGADDR;
	}

	/* enable RAS IRQ if needed */
	if (b_active)
		p_abu_dma_dev->dma_dcsr[direction] = DCSR_RASIRQEN;
	else
		p_abu_dma_dev->dma_dcsr[direction] = 0;


	p_dma_desc = p_abu_dma_dev->p_dma_desc_vaddr[direction];
	next_desc_phys = p_abu_dma_dev->dma_desc_paddr[direction];
	if(p_abu_dma_dev->sub_stream[direction])
		dma_buff_phys = p_abu_dma_dev->sub_stream[direction]->dma_buffer.addr;
	else
		dma_buff_phys = p_abu_dma_dev->dummy_buf_paddr;

	for (i = 0; i < period_nums; i++) {
		for (j = 0; j < descs_per_period; j++) {
			u32 transfer_size;

			/* the last descriptor will pointer the head of descriptor chain (circle buffer) */
			if ((i == period_nums - 1) && (j == descs_per_period -1 ))
				next_desc_phys = p_abu_dma_dev->dma_desc_paddr[direction];
			else
				next_desc_phys += sizeof(pxa_dma_desc);

			p_dma_desc->ddadr = next_desc_phys;

			if (direction == SNDRV_PCM_STREAM_PLAYBACK) {
				p_dma_desc->dsadr = dma_buff_phys;
				p_dma_desc->dtadr = p_abu_dma_dev->abu_dma_data[direction].dev_addr;
			} else {
				p_dma_desc->dsadr = p_abu_dma_dev->abu_dma_data[direction].dev_addr;
				p_dma_desc->dtadr = dma_buff_phys;
			}

			/* default dcmd will not enable any interrrupt per descriptors */
			/* for the beginning desc of active stream, it need to enable start interrupt */
			/* for the end desc of active stream, it need to enable end interrupt */
			if ((j == descs_per_period - 1) && trailing_bytes)
				transfer_size = trailing_bytes;
			else
				transfer_size = bytes_per_desc;

			p_dma_desc->dcmd = tmp_dcmd | transfer_size;

			if ((b_active) && (j == (descs_per_period - 1))) {
				p_dma_desc->ddadr |= DDADR_STOP;
				p_dma_desc->dcmd |= DCMD_ENDIRQEN;
			}

			p_dma_desc++;
			dma_buff_phys += transfer_size;
		}
	}

	return;
}



static int pxa95x_pcm_abu_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params)
{
	struct abu_device_ctx * p_abu_dev_ctx = pxa95x_pcm_abu_get_ctx(substream);
	struct abu_dma_device *p_abu_dma_dev = &p_abu_dev_ctx->abu_dma_dev;
	u32 total_bytes = params_buffer_bytes(params);
	u32 period_bytes = params_period_bytes(params);
	u32 period_nums = params_periods(params);

	pr_debug("[ABU] pxa95x_pcm_abu_hw_params (%s stream) +++\r\n", substream->stream?"input":"output");

	pr_debug("[ABU] stream & Runtime addr 0x%x, 0x%x,  prt 0x%x, 0x%x, bytes %d \r\n",
			(u32)substream, (u32)substream->runtime, (u32)substream->runtime->dma_buffer_p,
			(u32)substream->runtime->dma_addr, (u32)substream->runtime->dma_bytes);

	pr_debug("[ABU] total_bytes %d, period_bytes %d, period_nums %d\r\n",
			total_bytes, period_bytes, period_nums);

	/* store the period bytes and period numbers. */
	p_abu_dma_dev->stream_hw_params[substream->stream].period_bytes = period_bytes;
	p_abu_dma_dev->stream_hw_params[substream->stream].period_nums = period_nums;

	return 0;
}

static int pxa95x_pcm_abu_hw_free(struct snd_pcm_substream *substream)
{
	int ret = 0;

	pr_debug("[ABU] pxa95x_pcm_abu_hw_free (%s stream) +++\r\n", substream->stream?"input":"output");

	/* reset runtime buffer of current stream */
	snd_pcm_set_runtime_buffer(substream, NULL);
	return ret;
}

static int pxa95x_pcm_abu_prepare(struct snd_pcm_substream *substream)
{
	pr_debug("[ABU] pxa95x_pcm_abu_prepare (%s stream) +++\r\n", substream->stream?"input":"output");
	return 0;
}

static int pxa95x_pcm_abu_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct abu_device_ctx * p_abu_dev_ctx = pxa95x_pcm_abu_get_ctx(substream);
	struct abu_dma_device *p_abu_dma_dev = &p_abu_dev_ctx->abu_dma_dev;
	struct ssp_device *p_ssp = p_abu_dev_ctx->abu_ssp_dev.ssp;

	u32 period_bytes = 0;
	u32 period_nums = 0;
	int ret = 0;
	int i;
	bool b_run = false;
	unsigned long flags = 0;

	pr_debug("[ABU] pxa95x_pcm_abu_trigger (%s stream. cmd %d) +++\r\n", substream->stream?"input":"output", cmd);
	pr_debug("[ABU] output %d, input %d\r\n", p_abu_dma_dev->b_stream_active[SNDRV_PCM_STREAM_PLAYBACK], p_abu_dma_dev->b_stream_active[SNDRV_PCM_STREAM_CAPTURE]);

	switch (cmd) {

	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:

		pxa95x_pcm_abu_set_dvfm_constraint(p_abu_dma_dev->dma_dvfm_idx[substream->stream], p_abu_dev_ctx);

		spin_lock_irqsave(&p_abu_dev_ctx->abu_lock, flags);

		if ((!p_abu_dma_dev->b_stream_active[SNDRV_PCM_STREAM_PLAYBACK])
			&& (!p_abu_dma_dev->b_stream_active[SNDRV_PCM_STREAM_CAPTURE]))
			b_run = true;
		else
			b_run = false;

		p_abu_dma_dev->b_stream_active[substream->stream] = true;


		/* before runnin, need to check if it need to configure non-active DMA channel */
		/* playback stream is not active, need to use record stream parameter */
		if (!p_abu_dma_dev->b_stream_active[SNDRV_PCM_STREAM_PLAYBACK]) {

			period_bytes = p_abu_dma_dev->stream_hw_params[SNDRV_PCM_STREAM_CAPTURE].period_bytes;
			period_nums = p_abu_dma_dev->stream_hw_params[SNDRV_PCM_STREAM_CAPTURE].period_nums;

			pxa95x_pcm_abu_cfg_dma_desc(p_abu_dev_ctx, period_bytes, period_nums,
							 SNDRV_PCM_STREAM_PLAYBACK, false);
		}

		/* recording stream is not active, need to use palyback stream parameter */
		if (!p_abu_dma_dev->b_stream_active[SNDRV_PCM_STREAM_CAPTURE]) {
			period_bytes = p_abu_dma_dev->stream_hw_params[SNDRV_PCM_STREAM_PLAYBACK].period_bytes;
			period_nums = p_abu_dma_dev->stream_hw_params[SNDRV_PCM_STREAM_PLAYBACK].period_nums;

			pxa95x_pcm_abu_cfg_dma_desc(p_abu_dev_ctx, period_bytes, period_nums,
							SNDRV_PCM_STREAM_CAPTURE, false);
		}

		/* configure DMA */
		period_bytes = p_abu_dma_dev->stream_hw_params[substream->stream].period_bytes;
		period_nums = p_abu_dma_dev->stream_hw_params[substream->stream].period_nums;
		pxa95x_pcm_abu_cfg_dma_desc(p_abu_dev_ctx, period_bytes, period_nums, substream->stream, true);

		if (b_run) {
			/*  start the DMA (for any playback/recording) */
			/*  the two DMA need to be enabled even there is only one mode )*/
			for (i = 0; i < ABU_DMA_CHANNEL_NUMS; i++) {
				DDADR(p_abu_dma_dev->dma_ch[i]) = p_abu_dma_dev->dma_desc_paddr[i];
				DCSR(p_abu_dma_dev->dma_ch[i]) = DCSR_RUN | p_abu_dma_dev->dma_dcsr[i];
			}

			pr_debug("[ABU] pxa95x_pcm_abu_trigger star two DMA now \r\n");

			if (p_abu_dev_ctx->b_err_int_flag)
				enable_irq(p_ssp->irq);
		}

		spin_unlock_irqrestore(&p_abu_dev_ctx->abu_lock, flags);

		break;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:

		spin_lock_irqsave(&p_abu_dev_ctx->abu_lock, flags);

		p_abu_dma_dev->b_stream_active[substream->stream] = false;

		if ((!p_abu_dma_dev->b_stream_active[SNDRV_PCM_STREAM_PLAYBACK])
			 && (!p_abu_dma_dev->b_stream_active[SNDRV_PCM_STREAM_CAPTURE])) {

			if (p_abu_dev_ctx->b_err_int_flag)
				disable_irq(p_ssp->irq);

			for (i = 0; i < ABU_DMA_CHANNEL_NUMS; i++) {
				DCSR(p_abu_dma_dev->dma_ch[i]) &= (~DCSR_RUN) & (~DCSR_RASIRQEN);
			}

			pr_debug("[ABU] pxa95x_pcm_abu_trigger stop two DMA now \r\n");

		} else {

			if (p_abu_dma_dev->b_stream_active[SNDRV_PCM_STREAM_PLAYBACK]) {

				period_bytes = p_abu_dma_dev->stream_hw_params[SNDRV_PCM_STREAM_PLAYBACK].period_bytes;
				period_nums = p_abu_dma_dev->stream_hw_params[SNDRV_PCM_STREAM_PLAYBACK].period_nums;

				pxa95x_pcm_abu_cfg_dma_desc(p_abu_dev_ctx, period_bytes, period_nums,
								 SNDRV_PCM_STREAM_CAPTURE, false);
			}

			if (p_abu_dma_dev->b_stream_active[SNDRV_PCM_STREAM_CAPTURE]) {

				period_bytes = p_abu_dma_dev->stream_hw_params[SNDRV_PCM_STREAM_CAPTURE].period_bytes;
				period_nums = p_abu_dma_dev->stream_hw_params[SNDRV_PCM_STREAM_CAPTURE].period_nums;

				pxa95x_pcm_abu_cfg_dma_desc(p_abu_dev_ctx, period_bytes, period_nums,
								 SNDRV_PCM_STREAM_PLAYBACK, false);
			}

			DDADR(p_abu_dma_dev->dma_ch[substream->stream]) = p_abu_dma_dev->dma_desc_paddr[substream->stream];
			DCSR(p_abu_dma_dev->dma_ch[substream->stream]) = DCSR_RUN;

			pr_debug("[ABU] still need to keep the DMA(%d) running (DCSR 0x%x)\r\n",
				p_abu_dma_dev->dma_ch[substream->stream], DCSR(p_abu_dma_dev->dma_ch[substream->stream]));

		}

		spin_unlock_irqrestore(&p_abu_dev_ctx->abu_lock, flags);

		pxa95x_pcm_abu_unset_dvfm_constraint(p_abu_dma_dev->dma_dvfm_idx[substream->stream], p_abu_dev_ctx);

		break;

	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static snd_pcm_uframes_t pxa95x_pcm_abu_pointer(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct abu_device_ctx * p_abu_dev_ctx = pxa95x_pcm_abu_get_ctx(substream);
	struct abu_dma_device *p_abu_dma_dev = &p_abu_dev_ctx->abu_dma_dev;
	struct abu_stream_hw_params *p_params = &(p_abu_dma_dev->stream_hw_params[substream->stream]);
	snd_pcm_uframes_t trans_frame;
	u32  trans_bytes = 0;

	trans_bytes = p_params->trans_period_nums * p_params->period_bytes;
	trans_frame = bytes_to_frames(runtime, trans_bytes);
	pr_debug("[ABU] pxa95x_pcm_abu_pointer[%s] transfer %d bytes \r\n", substream->stream?"input":"output", trans_bytes);

	if (trans_frame == runtime->buffer_size)
		trans_frame = 0;

	return trans_frame;
}

static int pxa95x_pcm_abu_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct abu_device_ctx * p_abu_dev_ctx = pxa95x_pcm_abu_get_ctx(substream);
	struct abu_dma_device *p_abu_dma_dev = &p_abu_dev_ctx->abu_dma_dev;
	struct ssp_device *p_ssp = p_abu_dev_ctx->abu_ssp_dev.ssp;
	int ret = 0;
	int i = 0;
	int dma_ret = 0;
	unsigned long flags = 0;

	pr_debug("[ABU] pxa95x_pcm_abu_open (%s stream) +++\r\n", substream->stream?"input":"output");

	/* set paramseter for runtime stream */
	snd_soc_set_runtime_hwparams(substream, &pxa95x_pcm_abu_hardware_params);
	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);

	/*
	 * playback samples are lost if the DMA count is not a multiple
	 * of the DMA burst size.  Let's add a rule to enforce that.
	 */
	ret = snd_pcm_hw_constraint_step(runtime, 0, SNDRV_PCM_HW_PARAM_PERIOD_BYTES, 160);
	if (ret)
		pr_err("[ABU] failed in snd_pcm_hw_constraint_step ret = %d \r\n", ret);

	ret = snd_pcm_hw_constraint_step(runtime, 0, SNDRV_PCM_HW_PARAM_BUFFER_BYTES, 160);
	if (ret)
		pr_err("[ABU] failed in snd_pcm_hw_constraint_step ret = %d \r\n", ret);

	ret = snd_pcm_hw_constraint_integer(runtime, SNDRV_PCM_HW_PARAM_PERIODS);
	if (ret < 0)
		pr_err("[ABU] failed in snd_pcm_hw_constraint_integer ret = %d \r\n", ret);

	ret = 0;

	spin_lock_irqsave(&p_abu_dev_ctx->abu_lock, flags);

	/* check if it need to request DMA channel */
	for (i = 0; i < ABU_DMA_CHANNEL_NUMS; i++)
	{
		pr_debug("[ABU] pxa95x_pcm_abu_open: %d: substream %p runtime %p \r\n",
					i, p_abu_dma_dev->sub_stream[i], p_abu_dma_dev->sub_stream[i]->runtime);

		/* request DMA channel if not exist */
		if (p_abu_dma_dev->dma_ch[i] == -1) {
			p_abu_dma_dev->abu_dma_data[i].name = pxa95x_pcm_abu_channel_names[0][i];
			dma_ret = pxa_request_dma(p_abu_dma_dev->abu_dma_data[i].name, DMA_PRIO_LOW,
							pxa95x_pcm_abu_dma_irq, p_abu_dev_ctx);

			pr_debug("[ABU] pxa95x_pcm_abu_open: pxa_request_dma, %d: ret is %d \r\n", i, dma_ret);

			if (dma_ret < 0) {
				pr_err("[ABU] failed in pxa_request_dma (%d) ret %d \r\n", i,  dma_ret);
				ret = -1;
				goto func_exit;
			}

			p_abu_dma_dev->dma_ch[i] = dma_ret;

			/* update the DMA DEV addr, DRCMR  */
			p_abu_dma_dev->abu_dma_data[i].dev_addr = p_ssp->phys_base + ABUDR;

			if (i == SNDRV_PCM_STREAM_PLAYBACK)
				p_abu_dma_dev->abu_dma_data[i].drcmr = &DRCMR(p_ssp->drcmr_tx);
			else
				p_abu_dma_dev->abu_dma_data[i].drcmr = &DRCMR(p_ssp->drcmr_rx);

			*(p_abu_dma_dev->abu_dma_data[i].drcmr) = p_abu_dma_dev->dma_ch[i] | DRCMR_MAPVLD;

			/* clear DMA status firstly */
			DCSR(p_abu_dma_dev->dma_ch[i]) &= ~DCSR_RUN;
			DCSR(p_abu_dma_dev->dma_ch[i]) = 0;
			DCMD(p_abu_dma_dev->dma_ch[i]) = 0;
		}
	}

func_exit:

	spin_unlock_irqrestore(&p_abu_dev_ctx->abu_lock, flags);
	return ret;
}

static int pxa95x_pcm_abu_close(struct snd_pcm_substream *substream)
{
	struct abu_device_ctx * p_abu_dev_ctx = pxa95x_pcm_abu_get_ctx(substream);
	struct abu_dma_device *p_abu_dma_dev = &p_abu_dev_ctx->abu_dma_dev;
	int i = 0;
	unsigned long flags = 0;


	pr_debug("[ABU] pxa95x_pcm_abu_close (%s stream) +++\r\n", substream->stream?"input":"output");

	if (p_abu_dev_ctx->abu_ref_cnt) {
		pr_debug("[ABU] pxa95x_pcm_abu_close (there is still stream running) +++\r\n");
		return 0;
	}

	spin_lock_irqsave(&p_abu_dev_ctx->abu_lock, flags);

	/* it can free the DMA resource now because there is no active stream now */
	for (i = 0; i < ABU_DMA_CHANNEL_NUMS; i++) {

		if (p_abu_dma_dev->abu_dma_data[i].drcmr) {
			*(p_abu_dma_dev->abu_dma_data[i].drcmr) = 0;
			p_abu_dma_dev->abu_dma_data[i].drcmr = NULL;
		}

		if (p_abu_dma_dev->dma_ch[i] != -1)  {

			pxa_free_dma(p_abu_dma_dev->dma_ch[i]);
			p_abu_dma_dev->dma_ch[i] = -1;

			pr_debug("[ABU] pxa95x_pcm_abu_close: pxa_free_dma, %d. \r\n", i);
		}
	}

	spin_unlock_irqrestore(&p_abu_dev_ctx->abu_lock, flags);

	return 0;
}

static int pxa95x_pcm_abu_mmap(struct snd_pcm_substream *substream,
							   struct vm_area_struct *vma)
{
	struct snd_pcm_runtime *runtime = substream->runtime;

	pr_debug("[ABU] pxa95x_pcm_abu_mmap ++++++++++++++++++ \r\n");
	return dma_mmap_writecombine(substream->pcm->card->dev, vma,
					 runtime->dma_area,
					 runtime->dma_addr,
					 runtime->dma_bytes);
}

static int pxa95x_pcm_abu_alloc_dma_buf(struct snd_pcm *pcm, int stream)
{
	struct snd_pcm_substream *substream = pcm->streams[stream].substream;
	struct snd_dma_buffer *buf = &substream->dma_buffer;
	size_t size;

	pr_debug("[ABU] pxa95x_pcm_abu_alloc_dma_buf ++++++++++++++++++ \r\n");

	size = ABU_DMA_MAX_BUFFER_BYTES;
	buf->dev.type = SNDRV_DMA_TYPE_DEV;
	buf->dev.dev = pcm->card->dev;
	buf->private_data = NULL;
	buf->area = dma_alloc_writecombine(pcm->card->dev, size, &buf->addr, GFP_KERNEL);
	if (!buf->area)
		return -ENOMEM;

	buf->bytes = size;

	pr_debug("[ABU] alloc DMA buffer size = %d, virtual addr = 0x%x, physical addr = 0x%x \r\n",
				size, (u32)buf->area, (u32)buf->addr);
	return 0;
}

static void pxa95x_pcm_abu_free(struct snd_pcm *pcm)
{
	struct snd_pcm_substream *substream;
	struct snd_dma_buffer *buf;
	struct abu_dma_device *p_abu_dma_dev = NULL;
	struct snd_soc_pcm_runtime *rtd = pcm->private_data;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;   /* This is cpu dai, the input dai is codec dai */
	struct abu_device_ctx *p_abu_rtctx = cpu_dai->private_data;
	int i = 0;

	pr_debug("[ABU] pxa95x_pcm_abu_free ++++++++++++++++++ \r\n");

	/* free DMA buffer firstly */
	for (i = 0; i < ABU_DMA_CHANNEL_NUMS; i++) {

		substream = pcm->streams[i].substream;

		if (!substream)
			continue;

		buf = &substream->dma_buffer;
		if (!buf->area)
			continue;

		dma_free_writecombine(pcm->card->dev, buf->bytes, buf->area, buf->addr);
		buf->area = NULL;
	}

	if (p_abu_rtctx) {
		dvfm_unregister("abu-tx-dma", &p_abu_rtctx->abu_dma_dev.dma_dvfm_idx[0]);
		dvfm_unregister("abu-rx-dma", &p_abu_rtctx->abu_dma_dev.dma_dvfm_idx[1]);
		pr_info("[ABU] dvfm unregister \r\n");

		p_abu_dma_dev = &(p_abu_rtctx->abu_dma_dev);

		for (i = 0; i < ABU_DMA_CHANNEL_NUMS; i++) {

			if (p_abu_dma_dev->p_dma_desc_vaddr[i]) {

				dma_free_writecombine(pcm->card->dev, PAGE_SIZE,
						p_abu_dma_dev->p_dma_desc_vaddr[i],
						p_abu_dma_dev->dma_desc_paddr[i]);

				p_abu_dma_dev->p_dma_desc_vaddr[i] = NULL;
			}

			if (p_abu_dma_dev->sub_stream[i]) {

				p_abu_dma_dev->sub_stream[i] = NULL;
				p_abu_dma_dev->b_stream_active[i] = false;
				p_abu_dma_dev->dma_ch[i] = -1;
			}
		}

		if (p_abu_dma_dev->p_dummy_buf_vaddr) {
			dma_free_writecombine(pcm->card->dev, ABU_DMA_MAX_BUFFER_BYTES,
					p_abu_dma_dev->p_dummy_buf_vaddr,
					p_abu_dma_dev->dummy_buf_paddr);

			p_abu_dma_dev->p_dummy_buf_vaddr = NULL;
		}
	}

}


int pxa95x_pcm_abu_new(struct snd_card *card, struct snd_soc_dai *dai, struct snd_pcm *pcm)
{
	static u64 pxa95x_pcm_abu_dmamask = DMA_BIT_MASK(32);
	struct snd_soc_pcm_runtime *rtd = pcm->private_data;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;   /* This is cpu dai, the input dai is codec dai */
	struct abu_dma_device *p_abu_dma_dev = NULL;
	struct abu_device_ctx *p_abu_rtctx = NULL;
	int ret = 0;
	int i = 0;

	pr_debug("[ABU] pxa95x_pcm_abu_new ++++++++++++++++++ \r\n");
	pr_info("[ABU] [audio]-->pxa95x_pcm_abu_new: start. \r\n");

	if (!card->dev->dma_mask)
		card->dev->dma_mask = &pxa95x_pcm_abu_dmamask;
	if (!card->dev->coherent_dma_mask)
		card->dev->coherent_dma_mask = DMA_BIT_MASK(32);

	/* pass the abu deivce context pointer to local */
	p_abu_rtctx = cpu_dai->private_data;

	/* set abu DMA pointer */
	p_abu_dma_dev = &(p_abu_rtctx->abu_dma_dev);

	/* alloca the resource of ABU DMA part */
	for (i = 0; i < ABU_DMA_CHANNEL_NUMS; i++) {

		p_abu_dma_dev->p_dma_desc_vaddr[i] = dma_alloc_writecombine(pcm->card->dev,
									ABU_DMA_DESC_MAX_NUMS*sizeof(pxa_dma_desc),
									&(p_abu_dma_dev->dma_desc_paddr[i]),
									GFP_KERNEL);

		if (!p_abu_dma_dev->p_dma_desc_vaddr[i]) {
			pr_err("[ABU] pxa95x_pcm_abu_new failed in alloc p_dma_desc_vaddr\r\n");
			ret = -ENOMEM;
			goto error_out;
		}

		p_abu_dma_dev->sub_stream[i] =  pcm->streams[i].substream;

		p_abu_dma_dev->b_stream_active[i] = false;

		p_abu_dma_dev->dma_ch[i] = -1;

		pr_debug("[ABU] stream(%d) desc_vaddr = 0x%x, paddr = 0x%x, sub stream addr = 0x%x  \r\n",
			i, (u32)p_abu_dma_dev->p_dma_desc_vaddr[i],
			(u32)p_abu_dma_dev->dma_desc_paddr[i], (u32)p_abu_dma_dev->sub_stream[i]);
	}

	/* alloca DMA buffer for playback & Recording */
	if (dai->playback.channels_min) {

		pr_debug("[ABU] alloc DMA buffer for playback \r\n");
		ret = pxa95x_pcm_abu_alloc_dma_buf(pcm, SNDRV_PCM_STREAM_PLAYBACK);
		if (ret) {
			pr_err("[ABU] failed in pxa95x_pcm_abu_alloc_dma_buf \r\n");
			ret = -ENOMEM;
			goto error_out;
		}
	}

	if (dai->capture.channels_min) {

		pr_debug("[ABU] alloc DMA buffer for recording \r\n");
		ret = pxa95x_pcm_abu_alloc_dma_buf(pcm, SNDRV_PCM_STREAM_CAPTURE);
		if (ret) {
			pr_err("[ABU] failed in pxa95x_pcm_abu_alloc_dma_buf \r\n");
			ret = -ENOMEM;
			goto error_out;
		}
	}

	if (!dai->playback.channels_min || !dai->capture.channels_min) {
		p_abu_dma_dev->p_dummy_buf_vaddr = dma_alloc_writecombine(pcm->card->dev,
				ABU_DMA_MAX_BUFFER_BYTES,
				&(p_abu_dma_dev->dummy_buf_paddr),
				GFP_KERNEL);
		if (!p_abu_dma_dev->p_dummy_buf_vaddr) {
			pr_err("[ABU] failed in allocate dummy buffer\n");
			ret = -ENOMEM;
			goto error_out;
		}
	}

	/* DVFM register */
	dvfm_register("abu-tx-dma", &(p_abu_rtctx->abu_dma_dev.dma_dvfm_idx[0]));
	dvfm_register("abu-rx-dma", &(p_abu_rtctx->abu_dma_dev.dma_dvfm_idx[1]));

	pr_info("[ABU] dvfm register (%d) \r\n", p_abu_rtctx->abu_dma_dev.dma_dvfm_idx[0]);
	pr_info("[ABU] dvfm register (%d) \r\n", p_abu_rtctx->abu_dma_dev.dma_dvfm_idx[1]);

	pr_info("[ABU] pxa95x pcm abu complete successfully +++++++++++++++++++++\r\n");
	return ret;

error_out:
	pxa95x_pcm_abu_free(pcm);
	return ret;
}

struct snd_pcm_ops pxa95x_pcm_abu_ops = {
	.open	    = pxa95x_pcm_abu_open,
	.close	    = pxa95x_pcm_abu_close,
	.ioctl	    = snd_pcm_lib_ioctl,
	.hw_params  = pxa95x_pcm_abu_hw_params,
	.hw_free    = pxa95x_pcm_abu_hw_free,
	.prepare    = pxa95x_pcm_abu_prepare,
	.trigger    = pxa95x_pcm_abu_trigger,
	.pointer    = pxa95x_pcm_abu_pointer,
	.mmap	    = pxa95x_pcm_abu_mmap,
};


struct snd_soc_platform pxa95x_abu_soc_platform = {
	.name		= "pxa95x-abu-platform",
	.pcm_ops	= &pxa95x_pcm_abu_ops,
	.pcm_new	= pxa95x_pcm_abu_new,
	.pcm_free       = pxa95x_pcm_abu_free,
};

EXPORT_SYMBOL_GPL(pxa95x_abu_soc_platform);

static int __init pxa95x_pcm_abu_modinit(void)
{
	pr_info("[ABU] [audio]-->pxa95x_pcm_abu_modinit: register abu platform driver to asoc core.\r\n");
	return snd_soc_register_platform(&pxa95x_abu_soc_platform);
}

module_init(pxa95x_pcm_abu_modinit);

static void __exit pxa95x_pcm_abu_modexit(void)
{
	pr_info("[ABU] pxa95x_pcm_abu_modinit unregister \r\n");
	snd_soc_unregister_platform(&pxa95x_abu_soc_platform);
}

module_exit(pxa95x_pcm_abu_modexit);

MODULE_AUTHOR("jtang11@marvell.com"); /* jin tang */
MODULE_DESCRIPTION("PXA95x PCM DMA module specially for ABU");
MODULE_LICENSE("GPL");
