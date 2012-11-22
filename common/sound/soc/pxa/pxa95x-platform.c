/*
 * linux/sound/soc/pxa/pxa95x-platform.c
 *
 * Copyright (C) 2011 Marvell International Ltd.
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

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include <mach/hardware.h>
#include <mach/audio.h>

#include "pxa95x-pcm-ssp.h"
#include "pxa95x-pcm-abu.h"

extern struct snd_soc_platform pxa95x_abu_soc_platform;
extern struct snd_soc_platform pxa95x_ssp_soc_platform;

struct snd_soc_platform *pxa95x_mix_platforms[] = {
	[SNDRV_PCM_STREAM_PLAYBACK] = &pxa95x_abu_soc_platform,
	[SNDRV_PCM_STREAM_CAPTURE] = &pxa95x_ssp_soc_platform,
};

static int pxa95x_pcm_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	return pxa95x_mix_platforms[substream->stream]->pcm_ops->hw_params(substream, params);
}

static int pxa95x_pcm_hw_free(struct snd_pcm_substream *substream)
{
	return pxa95x_mix_platforms[substream->stream]->pcm_ops->hw_free(substream);
}

static int pxa95x_pcm_prepare(struct snd_pcm_substream *substream)
{
	return pxa95x_mix_platforms[substream->stream]->pcm_ops->prepare(substream);
}

static int pxa95x_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	return pxa95x_mix_platforms[substream->stream]->pcm_ops->trigger(substream, cmd);
}

static snd_pcm_uframes_t
pxa95x_pcm_pointer(struct snd_pcm_substream *substream)
{
	return pxa95x_mix_platforms[substream->stream]->pcm_ops->pointer(substream);
}

static int pxa95x_pcm_open(struct snd_pcm_substream *substream)
{
	return pxa95x_mix_platforms[substream->stream]->pcm_ops->open(substream);
}

static int pxa95x_pcm_close(struct snd_pcm_substream *substream)
{
	return pxa95x_mix_platforms[substream->stream]->pcm_ops->close(substream);
}

static int pxa95x_pcm_mmap(struct snd_pcm_substream *substream,
	struct vm_area_struct *vma)
{
	return pxa95x_mix_platforms[substream->stream]->pcm_ops->mmap(substream, vma);
}

struct snd_pcm_ops pxa95x_pcm_ops = {
	.open		= pxa95x_pcm_open,
	.close		= pxa95x_pcm_close,
	.ioctl		= snd_pcm_lib_ioctl,
	.hw_params	= pxa95x_pcm_hw_params,
	.hw_free	= pxa95x_pcm_hw_free,
	.prepare	= pxa95x_pcm_prepare,
	.trigger	= pxa95x_pcm_trigger,
	.pointer	= pxa95x_pcm_pointer,
	.mmap		= pxa95x_pcm_mmap,
};

int pxa95x_pcm_new(struct snd_card *card, struct snd_soc_dai *dai,
	struct snd_pcm *pcm)
{
	if (pcm->streams[SNDRV_PCM_STREAM_PLAYBACK].substream)
		pxa95x_mix_platforms[SNDRV_PCM_STREAM_PLAYBACK]->pcm_new(card, dai, pcm);

	if (pcm->streams[SNDRV_PCM_STREAM_CAPTURE].substream)
		pxa95x_mix_platforms[SNDRV_PCM_STREAM_CAPTURE]->pcm_new(card, dai, pcm);

	return 0;
}

void pxa95x_pcm_free(struct snd_pcm *pcm)
{
	if (pcm->streams[SNDRV_PCM_STREAM_PLAYBACK].substream)
		pxa95x_mix_platforms[SNDRV_PCM_STREAM_PLAYBACK]->pcm_free(pcm);

	if (pcm->streams[SNDRV_PCM_STREAM_CAPTURE].substream)
		pxa95x_mix_platforms[SNDRV_PCM_STREAM_CAPTURE]->pcm_free(pcm);

	return;
}

struct snd_soc_platform pxa95x_soc_platform = {
	.name		= "pxa95x-audio",
	.pcm_ops 	= &pxa95x_pcm_ops,
	.pcm_new	= pxa95x_pcm_new,
	.pcm_free	= pxa95x_pcm_free,
};
EXPORT_SYMBOL_GPL(pxa95x_soc_platform);

static int __init pxa95x_pcm_modinit(void)
{
	int ret = 0;

	pr_debug("[audio]-->pxa95x_pcm_modinit: register ssp platform driver to asoc core.\n");
	ret = snd_soc_register_platform(&pxa95x_soc_platform);

	return ret;
}
module_init(pxa95x_pcm_modinit);

static void __exit pxa95x_pcm_modexit(void)
{
	snd_soc_unregister_platform(&pxa95x_soc_platform);
}
module_exit(pxa95x_pcm_modexit);


MODULE_AUTHOR("tianxf@marvell.com");
MODULE_DESCRIPTION("PXA95x PCM DMA module");
MODULE_LICENSE("GPL");
