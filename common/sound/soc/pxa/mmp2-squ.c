/*
 * linux/sound/soc/pxa/mmp2-squ.c
 *
 * Base on linux/sound/soc/pxa/pxa2xx-pcm.c
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
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include <asm/dma.h>

#include <mach/pxa910-squ.h>
#include <mach/mmp2_dma.h>
#include <mach/mmp2_audiosram.h>
#include <mach/cputype.h>
#include "pxa3xx-pcm.h"
#include "mmp2-squ.h"

#include <linux/delay.h>
static const struct snd_pcm_hardware mmp2_pcm_hardware = {
	.info = SNDRV_PCM_INFO_MMAP |
	    SNDRV_PCM_INFO_MMAP_VALID |
	    SNDRV_PCM_INFO_INTERLEAVED |
	    SNDRV_PCM_INFO_PAUSE | SNDRV_PCM_INFO_RESUME,
	.formats = SNDRV_PCM_FMTBIT_S16_LE |
	    SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE,
	.period_bytes_min = 32,
	.period_bytes_max = 4096 - 32,
	.periods_min = 1,
	.periods_max = PAGE_SIZE / sizeof(pxa910_squ_desc),
	.buffer_bytes_max = 12 * 1024,
	.fifo_size = 32,
};

#define AUDIO_SRAM_START		(0xe0000000)
static const struct snd_pcm_hardware mmp2_a0_pcm_hardware = {
	.info = SNDRV_PCM_INFO_MMAP |
	    SNDRV_PCM_INFO_MMAP_VALID |
	    SNDRV_PCM_INFO_INTERLEAVED |
	    SNDRV_PCM_INFO_PAUSE | SNDRV_PCM_INFO_RESUME,
	.formats = SNDRV_PCM_FMTBIT_S16_LE |
	    SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE,
	.period_bytes_min = 32,
	.period_bytes_max = 2048,
	.periods_min = 1,
	.periods_max = PAGE_SIZE / sizeof(pxa910_squ_desc),
	.buffer_bytes_max = 4 * 1024,
	.fifo_size = 32,
};

struct pxa688_adma_registers {
	u32 byte_counter;
	u32 src_addr;
	u32 dest_addr;
	u32 next_desc_ptr;
	u32 ctrl;
	u32 chan_pri;		/* Only used in channel 0 */
	u32 curr_desc_ptr;
	u32 intr_mask;
	u32 intr_status;
};

struct mmp2_runtime_data {
	int dma_ch;
	struct pxa3xx_pcm_dma_params *params;
	void *squ_desc_array;
	dma_addr_t squ_desc_array_phys;

	struct pxa688_adma_registers pxa688_adma_saved[4];
	void *squ_desc_array_saved;
};

static unsigned int immid = -1;
static unsigned int count = 0;
static unsigned int wm8994_immid = -1;
static unsigned int hdmi_immid = -1;
static unsigned int temp = 0;

static DECLARE_WAIT_QUEUE_HEAD(dma_wq);

static void *mmp2_sram_alloc(int size, u32 * p)
{
	void *v;

	if (immid == -1 && count == 0)
		immid = audio_sram_register_kernel("pxa688 audio");
	else if (immid == -1 && count != 0)
		immid = audio_sram_register_kernel("pxa688 audio1");
	v = audio_sram_malloc(size, AUDIO_SRAM_MALLOC_HARDWARE
			      | AUDIO_SRAM_MALLOC_SRAM, immid);
	if (v != NULL) {
		*p = audio_sram_get_physical(v, immid);
		if (count > 1)
			hdmi_immid = immid;
		else {
			count++;
			wm8994_immid = immid;
		}
	}
	temp++;
	if (temp == 2)
		immid = -1;
	return v;
}

static void pxa688_sram_free(void *v, unsigned int immid)
{
	audio_sram_free(v, immid);
}

static int mmp2_sram_mmap_noncached(struct vm_area_struct *vma,
				    void *cpu_addr, dma_addr_t dma_addr,
				    size_t size)
{
	unsigned long user_size;
	unsigned long off = vma->vm_pgoff;
	u32 ret;

	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	user_size = (vma->vm_end - vma->vm_start) >> PAGE_SHIFT;
	ret = remap_pfn_range(vma, vma->vm_start,
			      __phys_to_pfn(dma_addr) + off,
			      user_size << PAGE_SHIFT, vma->vm_page_prot);

	return ret;
}

static void mmp2_squ_dma_irq(int dma_ch, void *dev_id)
{
	struct snd_pcm_substream *substream = dev_id;
	struct mmp2_runtime_data *prtd = substream->runtime->private_data;

	u32 base_register = pxa688_find_dma_register_base(dma_ch);
	if (base_register) {
		if (PXA688_DISR(base_register) & 0x1)
			snd_pcm_period_elapsed(substream);
		else {
			printk(KERN_ERR "%s: SQU error on channel %d\n",
			       prtd->params->name, dma_ch);
		}
		PXA688_DISR(base_register) = 0;
	}
}

static int mmp2_pcm_hw_params(struct snd_pcm_substream *substream,
			      struct snd_pcm_hw_params *params)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct mmp2_runtime_data *prtd = runtime->private_data;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct pxa3xx_pcm_dma_params *dma;
	size_t totsize = params_buffer_bytes(params);
	size_t period = params_period_bytes(params);
	pxa910_squ_desc *squ_desc;
	dma_addr_t dma_buff_phys, next_desc_phys;
	int ret;

	dma = substream->stream == SNDRV_PCM_STREAM_PLAYBACK ?
	    rtd->dai->cpu_dai->playback.dma_data :
	    rtd->dai->cpu_dai->capture.dma_data;
	/* return if this is a bufferless transfer e.g.
	 * codec <--> BT codec or GSM modem -- lg FIXME */
	if (!dma)
		return 0;

	/* this may get called several times by oss emulation
	 * with different params */
	if (prtd->params == NULL) {
		prtd->params = dma;
		ret = pxa688_request_dma(prtd->params->name, *(dma->drcmr),
					 mmp2_squ_dma_irq, substream);
		if (ret < 0)
			return ret;

		prtd->dma_ch = ret;
	} else if (prtd->params != dma) {
		pxa688_free_dma(prtd->dma_ch);
		prtd->params = dma;
		ret = pxa688_request_dma(prtd->params->name, *(dma->drcmr),
					 mmp2_squ_dma_irq, substream);
		if (ret < 0)
			return ret;

		prtd->dma_ch = ret;
	}

	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);
	runtime->dma_bytes = totsize;

	next_desc_phys = prtd->squ_desc_array_phys;
	dma_buff_phys = runtime->dma_addr;

	squ_desc = prtd->squ_desc_array;
	do {
		next_desc_phys += sizeof(pxa910_squ_desc);

		squ_desc->nxt_desc = next_desc_phys;
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			squ_desc->src_addr = dma_buff_phys;
			squ_desc->dst_addr = prtd->params->dev_addr;
		} else {
			squ_desc->src_addr = prtd->params->dev_addr;
			squ_desc->dst_addr = dma_buff_phys;
		}
		if (period > totsize)
			period = totsize;
		squ_desc->byte_cnt = period;
		squ_desc++;
		dma_buff_phys += period;

	} while (totsize -= period);
	squ_desc[-1].nxt_desc = prtd->squ_desc_array_phys;

	return 0;
}

static int mmp2_pcm_hw_free(struct snd_pcm_substream *substream)
{
	struct mmp2_runtime_data *prtd = substream->runtime->private_data;

	if (prtd->dma_ch != -1) {
		snd_pcm_set_runtime_buffer(substream, NULL);
		pxa688_free_dma(prtd->dma_ch);
		prtd->dma_ch = -1;
	}

	return 0;
}

static int mmp2_pcm_prepare(struct snd_pcm_substream *substream)
{
	int ret = 0;
	struct mmp2_runtime_data *prtd = substream->runtime->private_data;

	u32 base_register = pxa688_find_dma_register_base(prtd->dma_ch);
	if (base_register) {
		PXA688_DCR(base_register) = (prtd->params->dcmd)
		    & (~SDCR_CHANEN);
		PXA688_DIMR(base_register) = SDIMR_COMP;
	} else
		ret = -EINVAL;

	return ret;
}

static int mmp2_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct mmp2_runtime_data *prtd = substream->runtime->private_data;
	int ret = 0;
	u32 base_register;

	base_register = pxa688_find_dma_register_base(prtd->dma_ch);
	if (!base_register)
		return -EINVAL;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		PXA688_DNDPR(base_register) = prtd->squ_desc_array_phys;
		PXA688_DCR(base_register) = prtd->params->dcmd | SDCR_CHANEN;

		break;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		PXA688_DCR(base_register) = prtd->params->dcmd;
		wake_up(&dma_wq);
		break;

	case SNDRV_PCM_TRIGGER_RESUME:
		PXA688_DCR(base_register) = prtd->params->dcmd | SDCR_CHANEN;
		break;

	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		PXA688_DNDPR(base_register) = prtd->squ_desc_array_phys;
		PXA688_DCR(base_register) = prtd->params->dcmd | SDCR_CHANEN;
		break;

	default:
		ret = -EINVAL;
	}

	return ret;
}

static snd_pcm_uframes_t mmp2_pcm_pointer(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct mmp2_runtime_data *prtd = runtime->private_data;
	dma_addr_t ptr;
	snd_pcm_uframes_t x;

	u32 base_register = pxa688_find_dma_register_base(prtd->dma_ch);
	ptr = (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) ?
	    PXA688_DSAR(base_register) : PXA688_DDAR(base_register);

	x = bytes_to_frames(runtime, ptr - runtime->dma_addr);

	if (x == runtime->buffer_size)
		x = 0;
	return x;
}

static int mmp2_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct mmp2_runtime_data *prtd;
	int ret;

	if (cpu_is_mmp2() && !cpu_is_mmp2_z0() && !cpu_is_mmp2_z1())
		snd_soc_set_runtime_hwparams(substream, &mmp2_a0_pcm_hardware);
	else
		snd_soc_set_runtime_hwparams(substream, &mmp2_pcm_hardware);

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

	prtd = kzalloc(sizeof(struct mmp2_runtime_data), GFP_KERNEL);
	if (prtd == NULL) {
		ret = -ENOMEM;
		goto out;
	}

	prtd->dma_ch = -1;
	if (cpu_is_mmp2() && !cpu_is_mmp2_z0() && !cpu_is_mmp2_z1()) {
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			prtd->squ_desc_array_phys = AUDIO_SRAM_START +
			    mmp2_a0_pcm_hardware.buffer_bytes_max * 3;
		else
			prtd->squ_desc_array_phys = AUDIO_SRAM_START +
			    mmp2_a0_pcm_hardware.buffer_bytes_max * 3 + 0x300;
		prtd->squ_desc_array = ioremap(prtd->squ_desc_array_phys,
					       0x300);
	} else
		prtd->squ_desc_array =
		    prtd->squ_desc_array =
		    mmp2_sram_alloc(PAGE_SIZE, &prtd->squ_desc_array_phys);

	if (!prtd->squ_desc_array) {
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

static int mmp2_pcm_close(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai_link *machine = rtd->dai;

	struct snd_pcm_runtime *runtime = substream->runtime;
	struct mmp2_runtime_data *prtd = runtime->private_data;
	int immid;
	if (strcmp("I2S", machine->name) == 0)
		immid = hdmi_immid;
	else
		immid = wm8994_immid;

	if (cpu_is_mmp2() && !cpu_is_mmp2_z0() && !cpu_is_mmp2_z1())
		iounmap(prtd->squ_desc_array);
	else {
		pxa688_sram_free(prtd->squ_desc_array, immid);
		if (immid == wm8994_immid)
			pxa688_sram_free(prtd->squ_desc_array, hdmi_immid);
	}

	kfree(prtd);
	return 0;
}

static int mmp2_pcm_mmap(struct snd_pcm_substream *substream,
			 struct vm_area_struct *vma)
{
	struct snd_pcm_runtime *runtime = substream->runtime;

	return mmp2_sram_mmap_noncached(vma,
					runtime->dma_area,
					runtime->dma_addr, runtime->dma_bytes);
}

struct snd_pcm_ops mmp2_pcm_ops = {
	.open = mmp2_pcm_open,
	.close = mmp2_pcm_close,
	.ioctl = snd_pcm_lib_ioctl,
	.hw_params = mmp2_pcm_hw_params,
	.hw_free = mmp2_pcm_hw_free,
	.prepare = mmp2_pcm_prepare,
	.trigger = mmp2_pcm_trigger,
	.pointer = mmp2_pcm_pointer,
	.mmap = mmp2_pcm_mmap,
};

void mmp2_save_adma_regs(struct snd_pcm_substream *substream)
{
	u32 base_register;
	int ch;

	struct snd_pcm_runtime *runtime;
	struct mmp2_runtime_data *prtd;

	if (substream == NULL)
		return;

	runtime = substream->runtime;
	if (runtime == NULL)
		return;		/* No open PCM, return directly */

	prtd = runtime->private_data;
	if (prtd == NULL)
		return;

	ch = prtd->dma_ch;
	base_register = pxa688_find_dma_register_base(ch);

	/* MMP2 only uses chain mode */
	prtd->pxa688_adma_saved[ch - 2].src_addr = PXA688_DSAR(base_register);
	prtd->pxa688_adma_saved[ch - 2].dest_addr = PXA688_DDAR(base_register);
	prtd->pxa688_adma_saved[ch - 2].next_desc_ptr =
	    PXA688_DNDPR(base_register);
	prtd->pxa688_adma_saved[ch - 2].ctrl = PXA688_DCR(base_register);
	if (ch == 2 || ch == 4) {
		prtd->pxa688_adma_saved[ch - 2].chan_pri =
		    PXA688_DCP(base_register);
	}
	prtd->pxa688_adma_saved[ch - 2].curr_desc_ptr =
	    PXA688_DCDPR(base_register);
	prtd->pxa688_adma_saved[ch - 2].intr_mask = PXA688_DIMR(base_register);
	prtd->pxa688_adma_saved[ch - 2].intr_status =
	    PXA688_DISR(base_register);

	return;
}

EXPORT_SYMBOL_GPL(mmp2_save_adma_regs);

void mmp2_restore_adma_regs(struct snd_pcm_substream *substream)
{
	u32 base_register;
	int ch;

	struct snd_pcm_runtime *runtime;
	struct mmp2_runtime_data *prtd;

	if (substream == NULL)
		return;

	runtime = substream->runtime;
	if (runtime == NULL)
		return;		/* No open PCM, return directly */

	prtd = runtime->private_data;
	if (prtd == NULL)
		return;

	ch = prtd->dma_ch;
	base_register = pxa688_find_dma_register_base(ch);

	/* MMP2 only uses chain mode */
	PXA688_DNDPR(base_register) =
	    prtd->pxa688_adma_saved[ch - 2].next_desc_ptr;
	if (ch == 2 || ch == 4) {
		PXA688_DCP(base_register) =
		    prtd->pxa688_adma_saved[ch - 2].chan_pri;
	}
	PXA688_DIMR(base_register) = prtd->pxa688_adma_saved[ch - 2].intr_mask;
	PXA688_DCR(base_register) = prtd->pxa688_adma_saved[ch - 2].ctrl;

	return;
}

EXPORT_SYMBOL_GPL(mmp2_restore_adma_regs);

int mmp2_save_desc(struct snd_pcm_substream *substream)
{
	dma_addr_t squ_desc_array_phys;
	struct snd_pcm_runtime *runtime;
	struct mmp2_runtime_data *prtd;
	pxa910_squ_desc *squ_desc_save, *squ_desc;

	if (substream == NULL)
		return 0;

	runtime = substream->runtime;
	if (runtime == NULL)
		return 0;	/* No open PCM, return directly */

	prtd = runtime->private_data;
	if (prtd == NULL)
		return 0;

	if (prtd->squ_desc_array_saved)
		/* already saved */
		return 0;

	prtd->squ_desc_array_saved =
	    kmalloc(sizeof(struct pxa910_squ_desc) * 64, GFP_KERNEL);
	if (!prtd->squ_desc_array_saved)
		return -ENOMEM;

	squ_desc_array_phys = prtd->squ_desc_array_phys;
	squ_desc = prtd->squ_desc_array;
	squ_desc_save = prtd->squ_desc_array_saved;

	while (squ_desc && (squ_desc->nxt_desc != squ_desc_array_phys)) {
		squ_desc_save->byte_cnt = squ_desc->byte_cnt;
		squ_desc_save->src_addr = squ_desc->src_addr;
		squ_desc_save->dst_addr = squ_desc->dst_addr;
		squ_desc_save->nxt_desc = squ_desc->nxt_desc;
		squ_desc++;
		squ_desc_save++;
	}
	squ_desc_save->byte_cnt = squ_desc->byte_cnt;
	squ_desc_save->src_addr = squ_desc->src_addr;
	squ_desc_save->dst_addr = squ_desc->dst_addr;
	squ_desc_save->nxt_desc = squ_desc->nxt_desc;
	return 0;
}

EXPORT_SYMBOL_GPL(mmp2_save_desc);

void mmp2_restore_desc(struct snd_pcm_substream *substream)
{
	dma_addr_t squ_desc_array_phys;
	struct snd_pcm_runtime *runtime;
	struct mmp2_runtime_data *prtd;
	pxa910_squ_desc *squ_desc_save, *squ_desc;
	struct snd_dma_buffer *buf;
	void *vmem;
	size_t size;

	if (substream == NULL)
		return;

	runtime = substream->runtime;
	if (runtime == NULL)
		return;		/* No open PCM, return directly */

	prtd = runtime->private_data;
	if (prtd == NULL)
		return;

	if (!prtd->squ_desc_array_saved)
		/* not saved or already restored? */
		return;

	buf = &substream->dma_buffer;
	vmem = buf->area;
	size = buf->bytes;
	memset(vmem, 0, size);

	squ_desc_array_phys = prtd->squ_desc_array_phys;
	squ_desc = prtd->squ_desc_array;
	squ_desc_save = prtd->squ_desc_array_saved;

	while (squ_desc_save &&
	       (squ_desc_save->nxt_desc != squ_desc_array_phys)) {
		squ_desc->byte_cnt = squ_desc_save->byte_cnt;
		squ_desc->src_addr = squ_desc_save->src_addr;
		squ_desc->dst_addr = squ_desc_save->dst_addr;
		squ_desc->nxt_desc = squ_desc_save->nxt_desc;
		squ_desc++;
		squ_desc_save++;
	}
	squ_desc->byte_cnt = squ_desc_save->byte_cnt;
	squ_desc->src_addr = squ_desc_save->src_addr;
	squ_desc->dst_addr = squ_desc_save->dst_addr;
	squ_desc->nxt_desc = squ_desc_save->nxt_desc;
	kfree(prtd->squ_desc_array_saved);
	prtd->squ_desc_array_saved = NULL;
	return;
}

EXPORT_SYMBOL_GPL(mmp2_restore_desc);

static int mmp2_pcm_preallocate_dma_buffer(struct snd_pcm *pcm, int stream)
{
	struct snd_pcm_substream *substream = pcm->streams[stream].substream;
	struct snd_dma_buffer *buf = &substream->dma_buffer;

	size_t size = mmp2_pcm_hardware.buffer_bytes_max;

	if (cpu_is_mmp2() && !cpu_is_mmp2_z0() && !cpu_is_mmp2_z1())
		size = mmp2_a0_pcm_hardware.buffer_bytes_max;

	buf->dev.type = SNDRV_DMA_TYPE_DEV;
	buf->dev.dev = pcm->card->dev;
	buf->private_data = NULL;
	buf->area = mmp2_sram_alloc(size, &buf->addr);
	if (!buf->area)
		return -ENOMEM;
	buf->bytes = size;
	return 0;
}

static void mmp2_pcm_free_dma_buffers(struct snd_pcm *pcm)
{
	struct snd_pcm_substream *substream;
	struct snd_dma_buffer *buf;
	int stream;
	int immid;
	for (stream = 0; stream < 2; stream++) {
		struct snd_soc_pcm_runtime *rtd;
		struct snd_soc_dai_link *machine;

		substream = pcm->streams[stream].substream;
		if (!substream)
			continue;

		rtd = substream->private_data;
		machine = rtd->dai;

		buf = &substream->dma_buffer;
		if (!buf->area)
			continue;
		if (strcmp("I2S", machine->name) == 0)
			immid = hdmi_immid;
		else
			immid = wm8994_immid;

		pxa688_sram_free(buf->area, immid);
		if (immid == wm8994_immid)
			pxa688_sram_free(buf->area, hdmi_immid);

		buf->area = NULL;
	}
}

static u64 mmp2_pcm_dmamask = DMA_32BIT_MASK;

int mmp2_pcm_new(struct snd_card *card, struct snd_soc_dai *dai,
		 struct snd_pcm *pcm)
{
	int ret = 0;

	if (!card->dev->dma_mask)
		card->dev->dma_mask = &mmp2_pcm_dmamask;

	if (!card->dev->coherent_dma_mask)
		card->dev->coherent_dma_mask = DMA_32BIT_MASK;

	if (dai->playback.channels_min) {
		ret = mmp2_pcm_preallocate_dma_buffer(pcm,
						      SNDRV_PCM_STREAM_PLAYBACK);
		if (ret)
			goto out;
	}

	if (dai->capture.channels_min) {
		ret = mmp2_pcm_preallocate_dma_buffer(pcm,
						      SNDRV_PCM_STREAM_CAPTURE);
		if (ret)
			goto out;
	}
out:
	return ret;
}

struct snd_soc_platform mmp2_soc_platform = {
	.name = "mmp2-audio",
	.pcm_ops = &mmp2_pcm_ops,
	.pcm_new = mmp2_pcm_new,
	.pcm_free = mmp2_pcm_free_dma_buffers,
};

EXPORT_SYMBOL_GPL(mmp2_soc_platform);

static int __init mmp2_pcm_modinit(void)
{
	return snd_soc_register_platform(&mmp2_soc_platform);
}

module_init(mmp2_pcm_modinit);

static void __exit mmp2_pcm_modexit(void)
{
	snd_soc_unregister_platform(&mmp2_soc_platform);
}

module_exit(mmp2_pcm_modexit);

MODULE_AUTHOR("bin.yang@marvell.com");
MODULE_DESCRIPTION("MMP2 SQU DMA module");
MODULE_LICENSE("GPL");
