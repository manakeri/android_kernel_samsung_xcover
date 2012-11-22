/*
 * linux/sound/soc/pxa/pxa95x-abu.c
 * Base on pxa3xx-ssp.c
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



#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/pm_qos_params.h>
#include <linux/spinlock.h>
#include <linux/slab.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/initval.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include <mach/hardware.h>
#include <mach/io.h>
#include <mach/audio.h>
#include <plat/ssp.h>
#include <asm/mach-types.h>

#include "pxa95x-pcm-abu.h"


/* golobal ctx */
static struct abu_device_ctx *gp_abu_runtime_ctx = NULL;


/* currently , ABU IRQ will be only use to debug error  */
static irqreturn_t pxa95x_abu_irq(int irq, void *dev_id)
{
	struct abu_device_ctx *p_abu_runtime_ctx = (struct abu_device_ctx *)dev_id;
	void __iomem  *abu_mmio_base = NULL;
	u32 reg_value = 0;

	abu_mmio_base = p_abu_runtime_ctx->abu_ssp_dev.ssp->mmio_base;
	pxa95x_abu_reg_dump(abu_mmio_base);

	reg_value = __raw_readl(abu_mmio_base + ABUSR);
	pr_debug("[ABU] ABUSR = 0x%x\r\n", reg_value);

	__raw_writel(reg_value, abu_mmio_base + ABUSR);

	if ((reg_value & ABUSR_FIFO_OVERRUN) || (reg_value & ABUSR_FIFO_UNDERRUN)
		|| (reg_value & ABUSR_PLAYBACKTIMEOUT)) {
		pr_err("[ABU] Error happened. ABUSR = 0x%x\r\n", reg_value);
	}

	if ((reg_value & ABUSR_DMA_TX_REQ) || (reg_value & ABUSR_DMA_RX_REQ)) {
		pr_debug("[ABU] RX or TX request INTR happened. ABUSR = 0x%x\r\n", reg_value);
	}

	return IRQ_HANDLED;
}

/* ABU power requirement */
static void pxa95x_abu_set_dvfm_constraint(int abu_dev_idx)
{
	dvfm_disable_op_name("D2", abu_dev_idx);
	dvfm_disable_op_name("CG", abu_dev_idx);
	pr_debug("[ABU] pxa95x_abu_set_dvfm_constraint (set constrain) disable D2/CG \r\n");
}

static void pxa95x_abu_unset_dvfm_constraint(int abu_dev_idx)
{
	dvfm_enable_op_name("CG", abu_dev_idx);
	dvfm_enable_op_name("D2", abu_dev_idx);
	pr_debug("[ABU] pxa95x_abu_unset_dvfm_constraint (release constrain) enable D2/CG \r\n");
}


static void pxa95x_abu_forward_status(ABU_WORKING_STATUS_ENUM *p_status, int stream_direction)
{
	switch (*p_status) {
	case ABU_IDLE_STATUTS:

		if (stream_direction == SNDRV_PCM_STREAM_PLAYBACK)
			*p_status = ABU_PLAYBACK_ONLY_STATUS;
		else
			*p_status = ABU_RECORD_ONLY_STATUS;
		break;

	case ABU_PLAYBACK_ONLY_STATUS:

		if (stream_direction == SNDRV_PCM_STREAM_CAPTURE)
			*p_status = ABU_PLAYBACK_RECORD_STATUS;
		break;

	case ABU_RECORD_ONLY_STATUS:

		if (stream_direction == SNDRV_PCM_STREAM_PLAYBACK)
			*p_status = ABU_PLAYBACK_RECORD_STATUS;
		break;

	default:
		break;
	}

	return ;
}

static void pxa95x_abu_backward_status(ABU_WORKING_STATUS_ENUM *p_status, int stream_direction)
{
	switch (*p_status) {
	case ABU_IDLE_STATUTS:

		pr_err("[ABU] pxa95x_abu_backward_status : invalid (status = %d) (direction = %d) \r\n ",
					*p_status, stream_direction);
		break;

	case ABU_PLAYBACK_ONLY_STATUS:

		if (stream_direction == SNDRV_PCM_STREAM_PLAYBACK)
			*p_status = ABU_IDLE_STATUTS;
		else
			pr_err("[ABU]  invalid (status = %d) (direction = %d) \r\n ",
						*p_status, stream_direction);
		break;

	case ABU_RECORD_ONLY_STATUS:

		if (stream_direction == SNDRV_PCM_STREAM_CAPTURE)
			*p_status = ABU_IDLE_STATUTS;
		else
			pr_err("[ABU] invalid (*p_status = %d) (direction = %d) \r\n ",
						*p_status, stream_direction);

		break;

	case ABU_PLAYBACK_RECORD_STATUS:

		if (stream_direction == SNDRV_PCM_STREAM_PLAYBACK)
			*p_status = ABU_RECORD_ONLY_STATUS;
		else
			*p_status = ABU_PLAYBACK_ONLY_STATUS;
		break;
	}

	return;
}

/* should enable ABU and SSI function clock before any ABU/SSI operation,
	including register access */
static void pxa95x_abu_ssi_clock_enable(struct abu_device_ctx
							*p_abu_runtime_ctx)
{
	struct abu_ssp_device *p_abu_ssp_dev =
					&(p_abu_runtime_ctx->abu_ssp_dev);
	struct ssp_device *ssp =  p_abu_ssp_dev->ssp;

	pr_debug("[ABU] pxa95x_abu_ssi_clock_enable: enable ABU/SSI function clock\r\n");

	clk_enable(p_abu_runtime_ctx->abu_clk);
	clk_enable(ssp->clk);
}

/* disable ABU and SSI function clock once possible */
static void pxa95x_abu_ssi_clock_disable(struct abu_device_ctx
							*p_abu_runtime_ctx)
{
	struct abu_ssp_device *p_abu_ssp_dev =
					&(p_abu_runtime_ctx->abu_ssp_dev);
	struct ssp_device *ssp =  p_abu_ssp_dev->ssp;

	pr_debug("[ABU] pxa95x_abu_ssi_clock_disable: disable ABU/SSI function clock\r\n");

	clk_disable(p_abu_runtime_ctx->abu_clk);
	clk_disable(ssp->clk);
}

/* ABU playback/record watermark and auto-dma size setting */
static void pxa95x_abu_cfg_wmark_size(struct abu_device_ctx *p_abu_dev_ctx,
					u32 period_dma_bytes, u32 direction)
{
	struct ssp_device *p_ssp = p_abu_dev_ctx->abu_ssp_dev.ssp;
	void __iomem *p_abu_mmio_base = p_ssp->mmio_base;

	pr_debug("[ABU] pxa95x_pcm_abu_cfg_wmark_size (%d) +++ \r\n",
					direction);

	/* ABU water mark and auto-dma size setting */
	if (direction == SNDRV_PCM_STREAM_PLAYBACK) { /* for playback */
		abu_reg_set_playback_dma_size(p_abu_mmio_base,
					period_dma_bytes);
		abu_reg_set_playback_watermark(p_abu_mmio_base,
					ABU_WMARk_SIZE_PLAY);
		pr_debug("[ABU]  Playback: Auto DMA Size = 0x%x bytes, Watermark Size = 0x%x bytes.\r\n",
			period_dma_bytes, ABU_WMARk_SIZE_PLAY);
	} else { /* for recording */
		abu_reg_set_record_dma_size(p_abu_mmio_base, period_dma_bytes);
		abu_reg_set_record_watermark(p_abu_mmio_base, period_dma_bytes);
		pr_debug("[ABU]  Record: Auto DMA Size = 0x%x bytes, Watermark Size = 0x%x bytes.\r\n",
			period_dma_bytes, period_dma_bytes);
	}
}

static int pxa95x_abu_startup(struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	struct abu_device_ctx *p_abu_runtime_ctx = cpu_dai->private_data;
	struct abu_ssp_device *p_abu_ssp_dev = &p_abu_runtime_ctx->abu_ssp_dev;
	struct ssp_device *ssp = p_abu_ssp_dev->ssp;
	int ret = 0;

	pr_debug("[ABU] pxa95x_abu_startup (%s stream) +++\r\n", substream->stream?"input":"output");

	if (!p_abu_runtime_ctx) {
		pr_err("[ABU] p_abu_runtime_ctx is NULL \r\n");
		return -1;
	}

	if ((!cpu_dai->active) && (!p_abu_runtime_ctx->abu_ref_cnt)) {

		/* enable abu function & SSI function clock
			before any ABU/SSI operation */
		pxa95x_abu_ssi_clock_enable(p_abu_runtime_ctx);

		/* ABU init, do a software reset */
		abu_reg_soft_reset(ssp->mmio_base);

		/* Configures ABU transfer mode to auto DMA mode (by default) */
		abu_reg_set_interrupt_dma_mode(ssp->mmio_base, ABU_AUTO_DMA_MODE);

		/* ABU buffer size setting */
		abu_reg_set_buffer_size(ssp->mmio_base,  ABU_BUFFER_SIZE);
		pr_debug("[ABU]  ABU buffer size = 0x%x bytes.\r\n",
						ABU_BUFFER_SIZE);

		/* enable playback timeout interrupt, error interrupt for error detecting handling. */
		/* turn on the codec data mask in case of error. */
		abu_reg_set_error_interrupt(ssp->mmio_base, true);
		abu_reg_set_timeout_interrupt(ssp->mmio_base, true);
		abu_reg_set_playback_timeout(ssp->mmio_base, 0x0F);
		abu_reg_set_codec_mask(ssp->mmio_base, true);

		/* init SSP register of ABU */
		abu_reg_init_ssp(ssp->mmio_base);

		/* register dump */
		pxa95x_abu_reg_dump(ssp->mmio_base);

		/* disable ABU/SSI clock
			when no ABU/SSI operation any more */
		pxa95x_abu_ssi_clock_disable(p_abu_runtime_ctx);

		p_abu_runtime_ctx->abu_status = ABU_IDLE_STATUTS;

		pr_info("[ABU] [audio]-->pxa95x_abu_startup: Init ABU/SSI successfully.\r\n");
	}

	p_abu_runtime_ctx->abu_ref_cnt++;

	return ret;
}

static void pxa95x_abu_shutdown(struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	struct abu_device_ctx *p_abu_runtime_ctx = cpu_dai->private_data;
	struct abu_ssp_device *p_abu_ssp_dev = &(p_abu_runtime_ctx->abu_ssp_dev);
	struct ssp_device *ssp = p_abu_ssp_dev->ssp;

	pr_debug("[ABU] pxa95x_abu_shutdown (%s stream) +++\r\n", substream->stream?"input":"output");

	if ((!p_abu_runtime_ctx) || (!p_abu_runtime_ctx->abu_ref_cnt)) {
		pr_err("[ABU] pxa95x_abu_shutdown invalid input parameter \r\n");
		return;
	}

	if ((!cpu_dai->active) && (1 == p_abu_runtime_ctx->abu_ref_cnt)) {

		/* enable abu function & SSI function clock
			before any ABU/SSI operation */
		pxa95x_abu_ssi_clock_enable(p_abu_runtime_ctx);

		/* a workaround for fixing the issue of "sometimes, no audio
			when restart playback after stop".
			By reseting back to playback/record mode, the next time
			soft reset of ABU will reset both ABUCPR and ABUSPR to 0x0,
			or ABUSPR=0x80000 if ABU is playback mode */
		abu_reg_set_playback_mode(ssp->mmio_base, false);

		/* register dump */
		pxa95x_abu_reg_dump(ssp->mmio_base);

		/* disable ABU/SSI clock
			when no ABU/SSI operation any more */
		pxa95x_abu_ssi_clock_disable(p_abu_runtime_ctx);

		p_abu_runtime_ctx->abu_status = ABU_IDLE_STATUTS;
		pr_info("[ABU] [audio]-->pxa95x_abu_shutdown: De-Init ABU/SSI now.\r\n");
	}

	p_abu_runtime_ctx->abu_ref_cnt--;

	return;
}

static void pxa95x_abu_remove(struct platform_device *pdev, struct snd_soc_dai *cpu_dai)
{
	struct abu_device_ctx *p_abu_runtime_ctx = cpu_dai->private_data;
	struct ssp_device *ssp;

	pr_debug("[ABU] pxa95x_abu_remove  ++++++++++++++++++ \r\n");

	if (p_abu_runtime_ctx) {
		dvfm_unregister("pxa95x-abu", &p_abu_runtime_ctx->abu_ssp_dvfm_idx);
		pr_info("[ABU] dvfm unregister \r\n");
		ssp = p_abu_runtime_ctx->abu_ssp_dev.ssp;
		/* reset reference count */
		p_abu_runtime_ctx->abu_ref_cnt = 0;
		p_abu_runtime_ctx->start_time = 0;
		p_abu_runtime_ctx->end_time = 0;

		/* free ABU IRQ firstly */
		free_irq(ssp->irq, NULL);

		/* free ssp */
		pxa_ssp_free(ssp);
		ssp = NULL;

		kfree(p_abu_runtime_ctx);
		p_abu_runtime_ctx = NULL;
	}

	return;
}

static int pxa95x_abu_probe(struct platform_device *pdev, struct snd_soc_dai *cpu_dai)
{
	int err = 0;
	struct ssp_device *ssp;
	pr_debug("[ABU] [audio]-->pxa95x_abu_probe: start.\r\n");

	if (gp_abu_runtime_ctx) {
		pr_err("[ABU] pxa95x_abu_probe abu runtime context had already been initialized \r\n");
		err = -ENOMEM;
		goto error_out;
	}

	/* alloc runtime context for ABU device context */
	gp_abu_runtime_ctx = kzalloc(sizeof(struct abu_device_ctx), GFP_KERNEL);

	if (!gp_abu_runtime_ctx) {
		pr_err("[ABU] pxa95x_abu_probe failed in alloc gp_abu_runtime_ctx\r\n");
		err = -ENOMEM;
		goto error_out;
	}

	pr_debug("[ABU] gp_abu_runtime_ctx addr = 0x%x \r\n", (u32)gp_abu_runtime_ctx);

	/* abu clock register. Disabled by default */
	gp_abu_runtime_ctx->abu_clk = NULL;
	gp_abu_runtime_ctx->abu_clk = clk_get(NULL, "PXA95X_ABUCLK");
	if (IS_ERR(gp_abu_runtime_ctx->abu_clk)) {
		pr_err("[ABU] pxa95x_abu_probe can't get the clock of ABU \r\n");
		err = -ENOMEM;
		goto error_out;
	}

	/* requeset ssp: ssp4 for abu ssi */
	ssp = pxa_ssp_request(cpu_dai->id + 1, "ABU_SSI");
	if(!ssp)
	{
		pr_err("[ABU] pxa95x_abu_probe can' request ssp \n");
		err = -ENODEV;
		goto error_out;
	}

	gp_abu_runtime_ctx->abu_ssp_dev.ssp = ssp;

	/* don't disable clock, assume the clock managemer had already disable clock */
	/* clk_disable(gp_abu_runtime_ctx->abu_ssp_dev.abu_ssi_dev->clk); */
	pr_debug("[ABU] ssp port (%d) was initialized \r\n", cpu_dai->id + 1);

	/* require the ABU IRQ (it will be used to replace the SSP4 irq handle) */
	gp_abu_runtime_ctx->b_err_int_flag = false;
	err = request_irq(ssp->irq, pxa95x_abu_irq, IRQF_SHARED, "ABU", gp_abu_runtime_ctx);

	if (err < 0) {
		pr_err("[ABU] pxa95x_abu_probe can't get irq of ABU \r\n");
		err = -ENOMEM;
		goto error_out;
	}

	/*  disable by default */
	disable_irq(ssp->irq);
	pr_debug("[ABU] ABU request_irq completed. disabled by default. \r\n");

	/* reference count to 0 */
	gp_abu_runtime_ctx->abu_ref_cnt = 0;
	gp_abu_runtime_ctx->start_time = 0;
	gp_abu_runtime_ctx->end_time = 0;


	/* DVFM register */
	dvfm_register("abu-ssp", &gp_abu_runtime_ctx->abu_ssp_dvfm_idx);
	pr_info("[ABU] dvfm register (%d) \r\n", gp_abu_runtime_ctx->abu_ssp_dvfm_idx);

	/* pass the abu deivce context pointer to cpu dai private data */
	cpu_dai->private_data = (void *)gp_abu_runtime_ctx;

	/* init the spin lock of abu */
	spin_lock_init(&gp_abu_runtime_ctx->abu_lock);

	pr_info("[ABU] pxa95x_abu_probe  completed. \r\n");
	return 0;

error_out:
	pxa95x_abu_remove(pdev, cpu_dai);
	return err;
}

/*
 * Set the SSP audio DMA parameters and sample size.
 * Can be called multiple times by oss emulation.
 */
static int pxa95x_abu_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	struct abu_device_ctx *p_abu_runtime_ctx = cpu_dai->private_data;
	struct abu_ssp_device *p_abu_ssp_dev = &(p_abu_runtime_ctx->abu_ssp_dev);
	struct ssp_device *ssp = p_abu_ssp_dev->ssp;
	u32 sscr0;
	int ret = 0;

	pr_debug("[ABU] pxa95x_abu_hw_params  ++++++++++++++++++ \r\n");
	pr_debug("[ABU] p_abu_device = 0x%x, ssp = 0x%x\r\n", (u32)p_abu_ssp_dev, (u32)ssp);

	/* enable abu function & SSI function clock
		before any ABU/SSI operation */
	pxa95x_abu_ssi_clock_enable(p_abu_runtime_ctx);

	/* we can only change the settings if the port is not in use */
	sscr0 = __raw_readl(ssp->mmio_base + SSCR0);
	if (sscr0 & SSCR0_SSE) {
		pr_debug("[ABU] pxa95x_abu_hw_params: SSP is still running. Can't change SSP setting \r\n");
		return 0;
	}

	/* bit size */
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		sscr0 = __raw_readl(ssp->mmio_base + SSCR0);
		sscr0 |= SSCR0_DataSize(16);
		__raw_writel(sscr0, ssp->mmio_base + SSCR0);
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		sscr0 = __raw_readl(ssp->mmio_base + SSCR0);
		sscr0 |= SSCR0_EDSS | SSCR0_DataSize(16);
		__raw_writel(sscr0, ssp->mmio_base + SSCR0);
		break;
	default:
		pr_err("[ABU]  pxa95x_abu_hw_params: not support SNDRV PCM Format\r\n");
		ret =  -EINVAL;
		break;
	}

	/* disable ABU/SSI clock
		when no ABU/SSI operation any more */
	pxa95x_abu_ssi_clock_disable(p_abu_runtime_ctx);

	pr_debug("[ABU] pxa95x_abu_hw_params completed \r\n");
	return ret;
}

static int pxa95x_abu_trigger(struct snd_pcm_substream *substream, int cmd, struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	struct abu_device_ctx *p_abu_runtime_ctx = cpu_dai->private_data;
	struct abu_dma_device *p_abu_dma_dev =
					&(p_abu_runtime_ctx->abu_dma_dev);
	struct abu_ssp_device *p_abu_ssp_dev = &(p_abu_runtime_ctx->abu_ssp_dev);
	struct ssp_device *ssp = p_abu_ssp_dev->ssp;
	u32 period_bytes = 0;
	int ret = 0;
	unsigned long flags = 0;

	pr_debug("[ABU] pxa95x_abu_trigger (%s stream cmd %d) +++\r\n", substream->stream?"input":"output", cmd);

	switch (cmd) {

	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:

		spin_lock_irqsave(&p_abu_runtime_ctx->abu_lock, flags);

		/* upgrade the status of ABU by stream direction */
		pxa95x_abu_forward_status(&p_abu_runtime_ctx->abu_status, substream->stream);
		pr_debug("[ABU] pxa95x_abu_trigger (start)  %d  \r\n",
				p_abu_runtime_ctx->abu_status);

		switch (p_abu_runtime_ctx->abu_status) {
		case ABU_PLAYBACK_ONLY_STATUS:
		case ABU_RECORD_ONLY_STATUS:

			/* disable D2/CG and then enable abu function & SSI
				function clock before any ABU/SSI operation */
			pxa95x_abu_set_dvfm_constraint(p_abu_runtime_ctx->abu_ssp_dvfm_idx);
			pxa95x_abu_ssi_clock_enable(p_abu_runtime_ctx);

			/* configure ABU parameter */
			period_bytes = p_abu_dma_dev->stream_hw_params[substream->stream].period_bytes;
			pxa95x_abu_cfg_wmark_size(p_abu_runtime_ctx,
					period_bytes, substream->stream);

			abu_reg_set_playback_mode(ssp->mmio_base,
				ABU_PLAYBACK_ONLY_STATUS == p_abu_runtime_ctx->abu_status);

			/* enable SSP port now */
			abu_reg_enable_disable_ssp(ssp->mmio_base, true);

			pr_debug("[ABU] pxa95x_abu_trigger enable ABU/SSI now \r\n");
			break;

		case ABU_PLAYBACK_RECORD_STATUS:

			/* configure ABU parameter */
			period_bytes = p_abu_dma_dev->stream_hw_params[substream->stream].period_bytes;
			pxa95x_abu_cfg_wmark_size(p_abu_runtime_ctx, period_bytes, substream->stream);

			/* abu configure. set playback&record mode */
			abu_reg_set_playback_mode(ssp->mmio_base, false);
			break;

		case ABU_IDLE_STATUTS:
			/* do nothing  */
			pr_err("[ABU] invalid abu status in SNDRV_PCM_TRIGGER_START case \r\n");
			break;
		}

		spin_unlock_irqrestore(&p_abu_runtime_ctx->abu_lock, flags);

		break;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:

		spin_lock_irqsave(&p_abu_runtime_ctx->abu_lock, flags);

		/* upgrade the status of ABU by stream direction */
		pxa95x_abu_backward_status(&p_abu_runtime_ctx->abu_status, substream->stream);
		pr_debug("[ABU] pxa95x_abu_trigger (stop) %d \r\n",
				p_abu_runtime_ctx->abu_status);

		switch (p_abu_runtime_ctx->abu_status) {
		case ABU_IDLE_STATUTS:

			/* disable SSP port now */
			abu_reg_enable_disable_ssp(ssp->mmio_base, false);

			/* disable ABU/SSI clock and then enable D2/CG
				when no ABU/SSI operation any more */
			pxa95x_abu_ssi_clock_disable(p_abu_runtime_ctx);
			pxa95x_abu_unset_dvfm_constraint(p_abu_runtime_ctx->abu_ssp_dvfm_idx);

			pr_debug("[ABU] pxa95x_abu_trigger disable ABU/SSI now \r\n");
			break;

		case ABU_PLAYBACK_ONLY_STATUS:

			/* abu configure. set playback only mode */
			abu_reg_set_playback_mode(ssp->mmio_base, true);
			break;

		case ABU_RECORD_ONLY_STATUS:

			/* the prevoious status of ABU should support the record mode */
			break;

		case ABU_PLAYBACK_RECORD_STATUS:
			/* do nothing  */
			pr_err("[ABU] invalid abu status in SNDRV_PCM_TRIGGER_STOP case \r\n");
			break;
		}

		spin_unlock_irqrestore(&p_abu_runtime_ctx->abu_lock, flags);

		break;

	default:
		pr_err("[ABU] invalid cmd \r\n");
		ret = -EINVAL;
	}

	return ret;
}

static struct snd_soc_dai_ops pxa_abu_dai_ops = {
	.startup = pxa95x_abu_startup,
	.shutdown = pxa95x_abu_shutdown,
	.trigger = pxa95x_abu_trigger,
	.hw_params = pxa95x_abu_hw_params,
};

struct snd_soc_dai pxa95x_abu_dai[] =
{
	{
		.name = "pxa95x-abu-dai",
		.id = 3, /* 3 = SSP4 == ABU */
		.probe = pxa95x_abu_probe,
		.remove = pxa95x_abu_remove,
		.suspend = NULL,
		.resume = NULL,
		.playback = {
			.channels_min = 1,
			.channels_max = 2,
#if defined(CONFIG_SND_SOC_D1981)
			.rates = SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000,

#else			
			.rates = SNDRV_PCM_RATE_44100,
#endif			
			.formats = SNDRV_PCM_FMTBIT_S16_LE,},
		.capture = {
			.channels_min = 1,
			.channels_max = 2,
#if defined(CONFIG_SND_SOC_D1981)
			.rates = SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000,

#else			
			.rates = SNDRV_PCM_RATE_44100,
#endif			
			.formats = SNDRV_PCM_FMTBIT_S16_LE,},
		.ops = &pxa_abu_dai_ops,
	}
};

EXPORT_SYMBOL_GPL(pxa95x_abu_dai);

static int __init pxa95x_abu_modinit(void)
{
	pr_info("[ABU] [audio]-->pxa95x_abu_modinit: register abu dai to asoc core.\r\n");
	return snd_soc_register_dais(pxa95x_abu_dai, ARRAY_SIZE(pxa95x_abu_dai));
}
module_init(pxa95x_abu_modinit);

static void __exit pxa95x_abu_exit(void)
{
	pr_info("[ABU] pxa95x_abu_exit unregister \r\n");
	snd_soc_unregister_dais(pxa95x_abu_dai, ARRAY_SIZE(pxa95x_abu_dai));
}
module_exit(pxa95x_abu_exit);


/* Module information */
MODULE_AUTHOR("jtang11@marvell.com"); /* jin tang */
MODULE_DESCRIPTION("pxa95x ABU SoC Interface");
MODULE_LICENSE("GPL");
