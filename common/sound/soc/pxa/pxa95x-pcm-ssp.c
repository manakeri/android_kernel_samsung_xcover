/*
 * linux/sound/soc/pxa/pxa95x-pcm.c
 *
 * Base on linux/sound/soc/pxa/pxa2xx-pcm.c
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


#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include <mach/dma.h>
#include <mach/hardware.h>
#include <mach/regs-ost.h>
#include <mach/audio.h>
#include <mach/dvfm.h>

#include "pxa95x-pcm-ssp.h"

static int pxa95x_ssp_dvfm_idx;

static const struct snd_pcm_hardware pxa95x_pcm_hardware = {
	.info			= SNDRV_PCM_INFO_MMAP |
				  SNDRV_PCM_INFO_MMAP_VALID |
				  SNDRV_PCM_INFO_INTERLEAVED |
				  SNDRV_PCM_INFO_PAUSE |
				  SNDRV_PCM_INFO_RESUME,
	.formats		= SNDRV_PCM_FMTBIT_S16_LE |
					SNDRV_PCM_FMTBIT_S24_LE |
					SNDRV_PCM_FMTBIT_S32_LE,
	.period_bytes_min	= 32,
	.period_bytes_max	= 8192 - 32,
	.periods_min		= 1,
	.periods_max		= PAGE_SIZE/sizeof(pxa_dma_desc),
	.buffer_bytes_max	= 128 * 1024,
	.fifo_size		= 32,
};

struct pxa95x_runtime_data {
	int dma_ch;
	struct pxa95x_pcm_dma_params *params;
	pxa_dma_desc *dma_desc_array;
	dma_addr_t dma_desc_array_phys;
};

static void pxa95x_pcm_dma_irq(int dma_ch, void *dev_id)
{
	struct snd_pcm_substream *substream = dev_id;
	struct pxa95x_runtime_data *prtd = substream->runtime->private_data;
	int dcsr;

	dcsr = DCSR(dma_ch);
	DCSR(dma_ch) = dcsr & ~DCSR_STOPIRQEN;

	if (dcsr & DCSR_ENDINTR) {
		snd_pcm_period_elapsed(substream);
	} else {
		printk(KERN_ERR "%s: DMA error on channel %d (DCSR=%#x)\n",
			prtd->params->name, dma_ch, dcsr);
	}
}

static int pxa95x_pcm_ssp_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct pxa95x_runtime_data *prtd = runtime->private_data;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct pxa95x_pcm_dma_params *dma;
	size_t totsize = params_buffer_bytes(params);
	size_t period = params_period_bytes(params);
	pxa_dma_desc *dma_desc;
	dma_addr_t dma_buff_phys, next_desc_phys;
	int ret;

	dma = substream->stream == SNDRV_PCM_STREAM_PLAYBACK ?
		rtd->dai->cpu_dai->playback.dma_data : rtd->dai->cpu_dai->capture.dma_data;
	/* return if this is a bufferless transfer e.g.
	 * codec <--> BT codec or GSM modem -- lg FIXME */
	 if (!dma)
		return 0;

	/* this may get called several times by oss emulation
	 * with different params */
	if (prtd->params == NULL) {
		prtd->params = dma;
		ret = pxa_request_dma(prtd->params->name, DMA_PRIO_LOW,
			      pxa95x_pcm_dma_irq, substream);
		if (ret < 0)
			return ret;
		prtd->dma_ch = ret;
	} else if (prtd->params != dma) {
		pxa_free_dma(prtd->dma_ch);
		prtd->params = dma;
		ret = pxa_request_dma(prtd->params->name, DMA_PRIO_LOW,
			      pxa95x_pcm_dma_irq, substream);
		if (ret < 0)
			return ret;
		prtd->dma_ch = ret;
	}

	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);
	runtime->dma_bytes = totsize;

	dma_desc = prtd->dma_desc_array;
	next_desc_phys = prtd->dma_desc_array_phys;
	dma_buff_phys = runtime->dma_addr;
	do {
		next_desc_phys += sizeof(pxa_dma_desc);
		dma_desc->ddadr = next_desc_phys;
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			dma_desc->dsadr = dma_buff_phys;
			dma_desc->dtadr = prtd->params->dev_addr;
		} else {
			dma_desc->dsadr = prtd->params->dev_addr;
			dma_desc->dtadr = dma_buff_phys;
		}
		if (period > totsize)
			period = totsize;
		dma_desc->dcmd = prtd->params->dcmd | period | DCMD_ENDIRQEN;
		dma_desc++;
		dma_buff_phys += period;
	} while (totsize -= period);
	dma_desc[-1].ddadr = prtd->dma_desc_array_phys;

	return 0;
}

static int pxa95x_pcm_ssp_hw_free(struct snd_pcm_substream *substream)
{
	struct pxa95x_runtime_data *prtd = substream->runtime->private_data;

	if (prtd && prtd->params)
		*prtd->params->drcmr = 0;

	if (prtd && prtd->dma_ch) {
		snd_pcm_set_runtime_buffer(substream, NULL);
		pxa_free_dma(prtd->dma_ch);
		prtd->dma_ch = 0;
	}

	return 0;
}

static int pxa95x_pcm_ssp_prepare(struct snd_pcm_substream *substream)
{
	struct pxa95x_runtime_data *prtd = substream->runtime->private_data;

	DCSR(prtd->dma_ch) &= ~DCSR_RUN;
	DCSR(prtd->dma_ch) = 0;
	DCMD(prtd->dma_ch) = 0;
	*prtd->params->drcmr = prtd->dma_ch | DRCMR_MAPVLD;

	return 0;
}

static int pxa95x_pcm_ssp_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct pxa95x_runtime_data *prtd = substream->runtime->private_data;
	int ret = 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		dvfm_disable_lowpower(pxa95x_ssp_dvfm_idx);

		DDADR(prtd->dma_ch) = prtd->dma_desc_array_phys;
		DCSR(prtd->dma_ch) |= DCSR_RUN;
		break;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		DCSR(prtd->dma_ch) &= ~DCSR_RUN;

		dvfm_enable_lowpower(pxa95x_ssp_dvfm_idx);

		break;

	default:
		ret = -EINVAL;
	}

	return ret;
}

static snd_pcm_uframes_t
pxa95x_pcm_ssp_pointer(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct pxa95x_runtime_data *prtd = runtime->private_data;
	dma_addr_t ptr;
	snd_pcm_uframes_t x;

	ptr = (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) ?
			 DSADR(prtd->dma_ch) : DTADR(prtd->dma_ch);
	x = bytes_to_frames(runtime, ptr - runtime->dma_addr);

	if (x == runtime->buffer_size)
		x = 0;
	return x;
}

static int pxa95x_pcm_ssp_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct pxa95x_runtime_data *prtd;
	int ret;

	snd_soc_set_runtime_hwparams(substream, &pxa95x_pcm_hardware);

	/*
	 * For mysterious reasons (and despite what the manual says)
	 * playback samples are lost if the DMA count is not a multiple
	 * of the DMA burst size.  Let's add a rule to enforce that.
	 */
	ret = snd_pcm_hw_constraint_step(runtime, 0,
		SNDRV_PCM_HW_PARAM_PERIOD_BYTES, 32);
	if (ret)
		goto out;

	ret = snd_pcm_hw_constraint_step(runtime, 0,
		SNDRV_PCM_HW_PARAM_BUFFER_BYTES, 32);
	if (ret)
		goto out;

	ret = snd_pcm_hw_constraint_integer(runtime,
					SNDRV_PCM_HW_PARAM_PERIODS);
	if (ret < 0)
		goto out;

	prtd = kzalloc(sizeof(struct pxa95x_runtime_data), GFP_KERNEL);
	if (prtd == NULL) {
		ret = -ENOMEM;
		goto out;
	}

	prtd->dma_desc_array =
		dma_alloc_writecombine(substream->pcm->card->dev, PAGE_SIZE,
				       &prtd->dma_desc_array_phys, GFP_KERNEL);
	if (!prtd->dma_desc_array) {
		ret = -ENOMEM;
		goto err1;
	}

	runtime->private_data = prtd;
	return 0;

 err1:
	kfree(prtd);
 out:
	return ret;
}

static int pxa95x_pcm_ssp_close(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct pxa95x_runtime_data *prtd = runtime->private_data;

	dma_free_writecombine(substream->pcm->card->dev, PAGE_SIZE,
			      prtd->dma_desc_array, prtd->dma_desc_array_phys);
	kfree(prtd->params);
	kfree(prtd);
	return 0;
}

static int pxa95x_pcm_ssp_mmap(struct snd_pcm_substream *substream,
	struct vm_area_struct *vma)
{
	struct snd_pcm_runtime *runtime = substream->runtime;

	return dma_mmap_writecombine(substream->pcm->card->dev, vma,
						runtime->dma_area,
						runtime->dma_addr,
						runtime->dma_bytes);
}

struct snd_pcm_ops pxa95x_pcm_ssp_ops = {
	.open		= pxa95x_pcm_ssp_open,
	.close		= pxa95x_pcm_ssp_close,
	.ioctl		= snd_pcm_lib_ioctl,
	.hw_params	= pxa95x_pcm_ssp_hw_params,
	.hw_free	= pxa95x_pcm_ssp_hw_free,
	.prepare	= pxa95x_pcm_ssp_prepare,
	.trigger	= pxa95x_pcm_ssp_trigger,
	.pointer	= pxa95x_pcm_ssp_pointer,
	.mmap		= pxa95x_pcm_ssp_mmap,
};

static int pxa95x_pcm_preallocate_dma_buffer(struct snd_pcm *pcm, int stream)
{
	struct snd_pcm_substream *substream = pcm->streams[stream].substream;
	struct snd_dma_buffer *buf = &substream->dma_buffer;

	size_t size = pxa95x_pcm_hardware.buffer_bytes_max;
	buf->dev.type = SNDRV_DMA_TYPE_DEV;
	buf->dev.dev = pcm->card->dev;
	buf->private_data = NULL;
	buf->area = dma_alloc_writecombine(pcm->card->dev, size,
				   &buf->addr, GFP_KERNEL);
	if (!buf->area)
		return -ENOMEM;
	buf->bytes = size;
	return 0;
}

static void pxa95x_pcm_ssp_free_dma_buffers(struct snd_pcm *pcm)
{
	struct snd_pcm_substream *substream;
	struct snd_dma_buffer *buf;
	int stream;

	for (stream = 0; stream < 2; stream++) {
		substream = pcm->streams[stream].substream;
		if (!substream)
			continue;

		buf = &substream->dma_buffer;
		if (!buf->area)
			continue;

		dma_free_writecombine(pcm->card->dev, buf->bytes,
				      buf->area, buf->addr);
		buf->area = NULL;
	}
}

static u64 pxa95x_pcm_dmamask = DMA_BIT_MASK(32);

int pxa95x_pcm_ssp_new(struct snd_card *card, struct snd_soc_dai *dai,
	struct snd_pcm *pcm)
{
	int ret = 0;

	pr_debug("[audio]-->pxa95x_pcm_new: start.\n");

	if (!card->dev->dma_mask)
		card->dev->dma_mask = &pxa95x_pcm_dmamask;
	if (!card->dev->coherent_dma_mask)
		card->dev->coherent_dma_mask = DMA_BIT_MASK(32);

	if (dai->playback.channels_min) {
		ret = pxa95x_pcm_preallocate_dma_buffer(pcm,
			SNDRV_PCM_STREAM_PLAYBACK);
		if (ret)
			goto out;
	}

	if (dai->capture.channels_min) {
		ret = pxa95x_pcm_preallocate_dma_buffer(pcm,
			SNDRV_PCM_STREAM_CAPTURE);
		if (ret)
			goto out;
	}
 out:
	return ret;
}

struct snd_soc_platform pxa95x_ssp_soc_platform = {
	.name		= "pxa95x-audio",
	.pcm_ops 	= &pxa95x_pcm_ssp_ops,
	.pcm_new	= pxa95x_pcm_ssp_new,
	.pcm_free	= pxa95x_pcm_ssp_free_dma_buffers,
};
EXPORT_SYMBOL_GPL(pxa95x_ssp_soc_platform);

static int __init pxa95x_pcm_ssp_modinit(void)
{
	int ret = 0;

	dvfm_register("audio-ssp", &pxa95x_ssp_dvfm_idx);

	pr_debug("[audio]-->pxa95x_pcm_modinit: register ssp platform driver to asoc core.\n");
	ret = snd_soc_register_platform(&pxa95x_ssp_soc_platform);
	if(ret)
		dvfm_unregister("audio-ssp", &pxa95x_ssp_dvfm_idx);

	return ret;
}
module_init(pxa95x_pcm_ssp_modinit);

static void __exit pxa95x_pcm_ssp_modexit(void)
{
	snd_soc_unregister_platform(&pxa95x_ssp_soc_platform);
	dvfm_unregister("audio-ssp", &pxa95x_ssp_dvfm_idx);
}
module_exit(pxa95x_pcm_ssp_modexit);


MODULE_AUTHOR("bin.yang@marvell.com");
MODULE_DESCRIPTION("PXA95x PCM DMA module");
MODULE_LICENSE("GPL");
