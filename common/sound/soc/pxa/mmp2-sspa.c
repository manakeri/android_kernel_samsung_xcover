/*
 * linux/sound/soc/pxa/mmp2-sspa.c
 * Base on pxa2xx-ssp.c
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
#include <linux/slab.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/initval.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include <linux/io.h>
#include <mach/regs-sspa.h>
#include <mach/pxa910-squ.h>
#include <plat/dma.h>
#include <plat/ssp.h>

#include "pxa3xx-pcm.h"
#include "pxa3xx-ssp.h"
#include "mmp2-sspa.h"

#define ARRAY_AND_SIZE(x)	x, ARRAY_SIZE(x)

static char *sspa_names[2][2] = {
	{"SSPA1 PCM out", "SSPA1 PCM in",},
	{"SSPA2 PCM out", "SSPA2 PCM in",},
};

void mmp2_save_sspa_regs(struct ssp_device *sspa)
{
	if (sspa == NULL)
		return;

	sspa->mmp2_sspa_saved.rx_data = __raw_readl(sspa->mmio_base + SSPA_RXD);
	sspa->mmp2_sspa_saved.rx_id = __raw_readl(sspa->mmio_base + SSPA_RXID);
	sspa->mmp2_sspa_saved.rx_ctrl =
	    __raw_readl(sspa->mmio_base + SSPA_RXCTL);
	sspa->mmp2_sspa_saved.rx_sp_ctrl =
	    __raw_readl(sspa->mmio_base + SSPA_RXSP);
	sspa->mmp2_sspa_saved.rx_fifo_ul =
	    __raw_readl(sspa->mmio_base + SSPA_RXFIFO_UL);
	sspa->mmp2_sspa_saved.rx_int_msk =
	    __raw_readl(sspa->mmio_base + SSPA_RXINT_MASK);
	sspa->mmp2_sspa_saved.rx_core = __raw_readl(sspa->mmio_base + SSPA_RXC);
	sspa->mmp2_sspa_saved.rx_fifo_nofs =
	    __raw_readl(sspa->mmio_base + SSPA_RXFIFO_NOFS);
	sspa->mmp2_sspa_saved.rx_fifo_size =
	    __raw_readl(sspa->mmio_base + SSPA_RXFIFO_SIZE);
	sspa->mmp2_sspa_saved.tx_data = __raw_readl(sspa->mmio_base + SSPA_TXD);
	sspa->mmp2_sspa_saved.tx_id = __raw_readl(sspa->mmio_base + SSPA_TXID);
	sspa->mmp2_sspa_saved.tx_ctrl =
	    __raw_readl(sspa->mmio_base + SSPA_TXCTL);
	sspa->mmp2_sspa_saved.tx_sp_ctrl =
	    __raw_readl(sspa->mmio_base + SSPA_TXSP);
	sspa->mmp2_sspa_saved.tx_fifo_ll =
	    __raw_readl(sspa->mmio_base + SSPA_TXFIFO_LL);
	sspa->mmp2_sspa_saved.tx_int_msk =
	    __raw_readl(sspa->mmio_base + SSPA_TXINT_MASK);
	sspa->mmp2_sspa_saved.tx_core = __raw_readl(sspa->mmio_base + SSPA_TXC);
	sspa->mmp2_sspa_saved.tx_fifo_nofs =
	    __raw_readl(sspa->mmio_base + SSPA_TXFIFO_NOFS);
	sspa->mmp2_sspa_saved.tx_fifo_size =
	    __raw_readl(sspa->mmio_base + SSPA_TXFIFO_SIZE);
	sspa->mmp2_sspa_saved.aud_ctrl0 =
	    __raw_readl(sspa->mmio_base + SSPA_AUD_CTRL);
	sspa->mmp2_sspa_saved.aud_pll_ctrl0 =
	    __raw_readl(sspa->mmio_base + SSPA_AUD_PLL_CTRL0);
	sspa->mmp2_sspa_saved.aud_pll_ctrl1 =
	    __raw_readl(sspa->mmio_base + SSPA_AUD_PLL_CTRL1);

	return;
}

EXPORT_SYMBOL_GPL(mmp2_save_sspa_regs);

void mmp2_restore_sspa_regs(struct ssp_device *sspa)
{
	if (sspa == NULL)
		return;

	__raw_writel(sspa->mmp2_sspa_saved.rx_data, sspa->mmio_base + SSPA_RXD);
	__raw_writel(sspa->mmp2_sspa_saved.rx_id, sspa->mmio_base + SSPA_RXID);
	__raw_writel(sspa->mmp2_sspa_saved.rx_ctrl,
		     sspa->mmio_base + SSPA_RXCTL);
	__raw_writel(sspa->mmp2_sspa_saved.rx_sp_ctrl | 0x80000000,
		     sspa->mmio_base + SSPA_RXSP);
	__raw_writel(sspa->mmp2_sspa_saved.rx_fifo_ul,
		     sspa->mmio_base + SSPA_RXFIFO_UL);
	__raw_writel(sspa->mmp2_sspa_saved.rx_int_msk,
		     sspa->mmio_base + SSPA_RXINT_MASK);
	__raw_writel(sspa->mmp2_sspa_saved.rx_core, sspa->mmio_base + SSPA_RXC);
	__raw_writel(sspa->mmp2_sspa_saved.rx_fifo_nofs,
		     sspa->mmio_base + SSPA_RXFIFO_NOFS);
	__raw_writel(sspa->mmp2_sspa_saved.rx_fifo_size,
		     sspa->mmio_base + SSPA_RXFIFO_SIZE);
	__raw_writel(sspa->mmp2_sspa_saved.tx_data, sspa->mmio_base + SSPA_TXD);
	__raw_writel(sspa->mmp2_sspa_saved.tx_id, sspa->mmio_base + SSPA_TXID);
	__raw_writel(sspa->mmp2_sspa_saved.tx_ctrl,
		     sspa->mmio_base + SSPA_TXCTL);
	__raw_writel(sspa->mmp2_sspa_saved.tx_sp_ctrl | 0x80000000,
		     sspa->mmio_base + SSPA_TXSP);
	__raw_writel(sspa->mmp2_sspa_saved.tx_fifo_ll,
		     sspa->mmio_base + SSPA_TXFIFO_LL);
	__raw_writel(sspa->mmp2_sspa_saved.tx_int_msk,
		     sspa->mmio_base + SSPA_TXINT_MASK);
	__raw_writel(sspa->mmp2_sspa_saved.tx_core, sspa->mmio_base + SSPA_TXC);
	__raw_writel(sspa->mmp2_sspa_saved.tx_fifo_nofs,
		     sspa->mmio_base + SSPA_TXFIFO_NOFS);
	__raw_writel(sspa->mmp2_sspa_saved.tx_fifo_size,
		     sspa->mmio_base + SSPA_TXFIFO_SIZE);
	__raw_writel(sspa->mmp2_sspa_saved.aud_ctrl0,
		     sspa->mmio_base + SSPA_AUD_CTRL);
	__raw_writel(sspa->mmp2_sspa_saved.aud_pll_ctrl0,
		     sspa->mmio_base + SSPA_AUD_PLL_CTRL0);
	__raw_writel(sspa->mmp2_sspa_saved.aud_pll_ctrl1,
		     sspa->mmio_base + SSPA_AUD_PLL_CTRL1);

	return;
}

EXPORT_SYMBOL_GPL(mmp2_restore_sspa_regs);

static int pxa688_ssp_startup(struct snd_pcm_substream *substream,
			      struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	struct ssp_priv *priv = cpu_dai->private_data;
	struct ssp_device *sspa = priv->ssp;
	int ret = 0;
	unsigned int sspasp;

	if (!cpu_dai->active) {
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			sspasp = __raw_readl(sspa->mmio_base + SSPA_TXSP);
			sspasp &= ~SSPA_SP_S_EN;
			__raw_writel(sspasp, sspa->mmio_base + SSPA_TXSP);
		} else {
			sspasp = __raw_readl(sspa->mmio_base + SSPA_RXSP);
			sspasp &= ~SSPA_SP_S_EN;
			__raw_writel(sspasp, sspa->mmio_base + SSPA_RXSP);
		}
	}

	return ret;
}

static void pxa688_ssp_shutdown(struct snd_pcm_substream *substream,
				struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	struct ssp_priv *priv = cpu_dai->private_data;
	struct ssp_device *sspa = priv->ssp;
	unsigned int sspasp;

	if (!cpu_dai->active) {
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			sspasp = __raw_readl(sspa->mmio_base + SSPA_TXSP);
			sspasp &= ~SSPA_SP_S_EN;
			__raw_writel(sspasp, sspa->mmio_base + SSPA_TXSP);
			/*
			 * mmp2 uses RX's clk as output clk,
			 * so disable RX
			 */
			sspasp = __raw_readl(sspa->mmio_base + SSPA_RXSP);
			sspasp &= ~SSPA_SP_S_EN;
			__raw_writel(sspasp, sspa->mmio_base + SSPA_RXSP);

		} else {
			sspasp = __raw_readl(sspa->mmio_base + SSPA_RXSP);
			sspasp &= ~SSPA_SP_S_EN;
			__raw_writel(sspasp, sspa->mmio_base + SSPA_RXSP);
		}
	}
}

#ifdef CONFIG_PM

static int pxa688_sspa_suspend(struct snd_soc_dai *cpu_dai)
{
	return 0;
}

static int pxa688_sspa_resume(struct snd_soc_dai *cpu_dai)
{
	return 0;
}

#else
#define pxa688_sspa_suspend	NULL
#define pxa688_sspa_resume	NULL
#endif

/*
 * Set the SSP ports SYSCLK.
 */
static int pxa688_ssp_set_dai_sysclk(struct snd_soc_dai *cpu_dai,
				     int clk_id, unsigned int freq, int dir)
{
	/*FIX ME: SSPA sys clock */
	return 0;
}

/*
 * Set the SSPA audio DMA parameters and sample size.
 * Can be called multiple times by oss emulation.
 */
static int pxa688_ssp_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	struct ssp_priv *priv = cpu_dai->private_data;
	struct ssp_device *sspa = priv->ssp;
	struct pxa3xx_pcm_dma_params *pcm_o, *pcm_i;
	u32 sspactl, sspactl_addr;
#if 0
	u32 sspa_sp;
#endif

	pcm_o = kmalloc(sizeof(struct pxa3xx_pcm_dma_params), GFP_KERNEL);
	pcm_i = kmalloc(sizeof(struct pxa3xx_pcm_dma_params), GFP_KERNEL);

	pcm_o->name = sspa_names[cpu_dai->id][0];
	pcm_i->name = sspa_names[cpu_dai->id][1];
	/* bit size */
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		sspactl_addr = SSPA_TXCTL;
		sspactl = __raw_readl(sspa->mmio_base + SSPA_TXCTL);
	} else {
		sspactl_addr = SSPA_RXCTL;
		sspactl = __raw_readl(sspa->mmio_base + SSPA_RXCTL);
	}
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S8:
		sspactl &= ~0x7;
		break;
	case SNDRV_PCM_FORMAT_S16_LE:
		sspactl &= ~0x7;
		sspactl |= 0x2;
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		sspactl &= ~0x7;
		sspactl |= 0x3;
		break;
	case SNDRV_PCM_FORMAT_S24_3LE:
		sspactl &= ~0x7;
		sspactl |= 0x4;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		sspactl &= ~0x7;
		sspactl |= 0x5;
		break;
	default:
		return -EINVAL;
	}
	__raw_writel(sspactl, sspa->mmio_base + sspactl_addr);

	pcm_o->dcmd = SDCR_DST_ADDR_HOLD | SDCR_SRC_ADDR_INC |
	    SDCR_PACKMOD | SDCR_DMA_BURST_4B | SDCR_FETCHND |
	    ((sspactl & 0x7) << 22);
	pcm_i->dcmd = SDCR_SRC_ADDR_HOLD | SDCR_DST_ADDR_INC |
	    SDCR_PACKMOD | SDCR_DMA_BURST_4B | SDCR_FETCHND |
	    ((sspactl & 0x7) << 22);
	pcm_o->drcmr = &(sspa->drcmr_tx);
	pcm_i->drcmr = &(sspa->drcmr_rx);
	pcm_o->dev_addr = sspa->phys_base + SSPA_TXD;
	pcm_i->dev_addr = sspa->phys_base + SSPA_RXD;

	/* select correct DMA params */
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		cpu_dai->playback.dma_data = pcm_o;
		kfree(pcm_i);
	} else {
		cpu_dai->capture.dma_data = pcm_i;
		kfree(pcm_o);
	}

#if 0
	if (sspa_clk[cpu_dai->id].dai_fmt == SND_SOC_DAIFMT_I2S) {
		int sfrmwidth =
		    snd_pcm_format_physical_width(params_format(params));
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			sspa_sp = __raw_readl(sspa->mmio_base + SSPA_TXSP);
			sspa_sp |= (sfrmwidth << 20);
			__raw_writel(sspa_sp, sspa->mmio_base + SSPA_TXSP);
		} else {
			sspa_sp = __raw_readl(sspa->mmio_base + SSPA_RXSP);
			sspa_sp |= (sfrmwidth << 20);
			__raw_writel(sspa_sp, sspa->mmio_base + SSPA_RXSP);
		}
	}
#endif

	return 0;
}

static int pxa688_ssp_trigger(struct snd_pcm_substream *substream,
			      int cmd, struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	struct ssp_priv *priv = cpu_dai->private_data;
	struct ssp_device *sspa = priv->ssp;
	int ret = 0;
	unsigned int sspasp, sspasp_addr;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		sspasp_addr = SSPA_TXSP;
	else
		sspasp_addr = SSPA_RXSP;

	switch (cmd) {
#if 0				/*temp solution: sspa register can't be read out correctly every time. */
	case SNDRV_PCM_TRIGGER_RESUME:
		sspasp = __raw_readl(sspa->mmio_base + sspasp_addr);
		sspasp |= SSPA_SP_S_EN;
		__raw_writel(sspasp | SSPA_SP_WEN, sspa->mmio_base
			     + sspasp_addr);
		break;
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		sspasp = __raw_readl(sspa->mmio_base + sspasp_addr);
		sspasp &= ~(SSPA_SP_FFLUSH | SSPA_SP_S_RST);
		__raw_writel(sspasp | SSPA_SP_WEN, sspa->mmio_base
			     + sspasp_addr);
		break;
	case SNDRV_PCM_TRIGGER_START:
		sspasp = __raw_readl(sspa->mmio_base + sspasp_addr);
		sspasp &= ~(SSPA_SP_FFLUSH | SSPA_SP_S_RST);
		sspasp |= SSPA_SP_S_EN;
		__raw_writel(sspasp | SSPA_SP_WEN, sspa->mmio_base
			     + sspasp_addr);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		sspasp = __raw_readl(sspa->mmio_base + sspasp_addr);
		sspasp |= (SSPA_SP_FFLUSH | SSPA_SP_S_RST);
		__raw_writel(sspasp | SSPA_SP_WEN, sspa->mmio_base
			     + sspasp_addr);
		break;
	case SNDRV_PCM_TRIGGER_SUSPEND:
		sspasp = __raw_readl(sspa->mmio_base + sspasp_addr);
		sspasp &= ~SSPA_SP_S_EN;
		__raw_writel(sspasp | SSPA_SP_WEN, sspa->mmio_base
			     + sspasp_addr);
		break;
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		sspasp = __raw_readl(sspa->mmio_base + sspasp_addr);
		sspasp |= (SSPA_SP_FFLUSH | SSPA_SP_S_RST);
		__raw_writel(sspasp | SSPA_SP_WEN, sspa->mmio_base
			     + sspasp_addr);
		break;
#else
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			/*
			 * mmp2 uses RX's clk as output clk,
			 * so enable RX before TX
			 */
			sspasp = __raw_readl(sspa->mmio_base + SSPA_RXSP);
			sspasp |= SSPA_SP_S_EN;
			__raw_writel(sspasp | SSPA_SP_WEN,
				     sspa->mmio_base + SSPA_RXSP);
			sspasp = __raw_readl(sspa->mmio_base + SSPA_TXSP);
		} else
			sspasp = __raw_readl(sspa->mmio_base + SSPA_RXSP);
		sspasp |= SSPA_SP_S_EN;
		__raw_writel(sspasp | SSPA_SP_WEN, sspa->mmio_base
			     + sspasp_addr);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			sspasp = __raw_readl(sspa->mmio_base + SSPA_TXSP);
			sspasp |= SSPA_SP_FFLUSH | SSPA_SP_S_RST;
			__raw_writel(sspasp | SSPA_SP_WEN,
				     sspa->mmio_base + sspasp_addr);
		} else if (codec_dai->playback.active == 0) {
			sspasp = __raw_readl(sspa->mmio_base + SSPA_RXSP);
			sspasp |= SSPA_SP_FFLUSH | SSPA_SP_S_RST;
			__raw_writel(sspasp | SSPA_SP_WEN,
				     sspa->mmio_base + sspasp_addr);
		}
		break;
#endif
	default:
		ret = -EINVAL;
	}

	return ret;
}

#define PXA688_SSPA_RATES 0xffffffff
#define PXA688_SSPA_FORMATS 0xffffffff

static struct snd_soc_dai_ops pxa688_sspa_dai_ops = {
	.startup = pxa688_ssp_startup,
	.shutdown = pxa688_ssp_shutdown,
	.trigger = pxa688_ssp_trigger,
	.hw_params = pxa688_ssp_hw_params,
	.set_sysclk = pxa688_ssp_set_dai_sysclk,
};

static int pxa688_ssp_probe(struct platform_device *pdev,
			    struct snd_soc_dai *dai)
{
	struct ssp_priv *priv;
	int ret;

	priv = kzalloc(sizeof(struct ssp_priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->ssp = sspa_request(dai->id + 1, "SSPA");
	if (priv->ssp == NULL) {
		ret = -ENODEV;
		goto err_priv;
	}

	priv->dai_fmt = (unsigned int)-1;
	dai->private_data = priv;

	return 0;

err_priv:
	kfree(priv);
	return ret;
}

static void pxa688_ssp_remove(struct platform_device *pdev,
			      struct snd_soc_dai *dai)
{
	struct ssp_priv *priv = dai->private_data;
	sspa_free(priv->ssp);
}

struct snd_soc_dai pxa688_sspa_dai[] = {
	{.name = "pxa688-sspa1",
	 .id = 0,
	 .probe = pxa688_ssp_probe,
	 .remove = pxa688_ssp_remove,
	 .suspend = pxa688_sspa_suspend,
	 .resume = pxa688_sspa_resume,
	 .playback = {
		      .channels_min = 1,
		      .channels_max = 128,
		      .rates = PXA688_SSPA_RATES,
		      .formats = PXA688_SSPA_FORMATS,},
	 .capture = {
		     .channels_min = 1,
		     .channels_max = 2,
		     .rates = PXA688_SSPA_RATES,
		     .formats = PXA688_SSPA_FORMATS,},
	 .ops = &pxa688_sspa_dai_ops,
	 },
	{.name = "pxa688-sspa2",
	 .id = 1,
	 .probe = pxa688_ssp_probe,
	 .remove = pxa688_ssp_remove,
	 .suspend = pxa688_sspa_suspend,
	 .resume = pxa688_sspa_resume,
	 .playback = {
		      .channels_min = 1,
		      .channels_max = 128,
		      .rates = PXA688_SSPA_RATES,
		      .formats = PXA688_SSPA_FORMATS,},
	 .capture = {
		     .channels_min = 1,
		     .channels_max = 2,
		     .rates = PXA688_SSPA_RATES,
		     .formats = PXA688_SSPA_FORMATS,},
	 .ops = &pxa688_sspa_dai_ops,
	 },
};

EXPORT_SYMBOL_GPL(pxa688_sspa_dai);

static int __init pxa688_sspa_modinit(void)
{
	return snd_soc_register_dais(ARRAY_AND_SIZE(pxa688_sspa_dai));
}

module_init(pxa688_sspa_modinit);

static void __exit pxa688_sspa_exit(void)
{
	snd_soc_unregister_dais(ARRAY_AND_SIZE(pxa688_sspa_dai));
}

module_exit(pxa688_sspa_exit);

/* Module information */
MODULE_AUTHOR("mingliang.hu@marvell.com");
MODULE_DESCRIPTION("MMP2 SSPA SoC Interface");
MODULE_LICENSE("GPL");
