/*
 * linux/sound/soc/pxa/pxa3xx-ssp.c
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

#include <asm/io.h>
#include <asm/mach-types.h>

#include <plat/ssp.h>
#include <mach/dma.h>

#include "pxa3xx-pcm.h"
#include "pxa3xx-ssp.h"

#define ARRAY_AND_SIZE(x)	x, ARRAY_SIZE(x)


static struct ssp_priv ssp_clk[4];

static char *ssp_names[4][2] = {
	{"SSP1 PCM out", "SSP1 PCM in",},
	{"SSP2 PCM out", "SSP2 PCM in",},
	{"SSP3 PCM out", "SSP3 PCM in",},
	{"SSP4 PCM out", "SSP4 PCM in",},
};

static int pxa3xx_ssp_startup(struct snd_pcm_substream *substream,
	struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	struct ssp_priv *priv = cpu_dai->private_data;
	struct ssp_device *ssp = priv->ssp;
	int ret = 0;
	u32 sscr0;

	if (!cpu_dai->active) {
		clk_enable(ssp->clk);
		sscr0 = __raw_readl(ssp->mmio_base + SSCR0);
		sscr0 &= ~SSCR0_SSE;
		__raw_writel(sscr0, ssp->mmio_base + SSCR0);
	}

	return ret;
}

static void pxa3xx_ssp_shutdown(struct snd_pcm_substream *substream,
	struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	struct ssp_priv *priv = cpu_dai->private_data;
	struct ssp_device *ssp = priv->ssp;
	u32 sscr0;

	if (!cpu_dai->active) {
		sscr0 = __raw_readl(ssp->mmio_base + SSCR0);
		sscr0 &= ~SSCR0_SSE;
		__raw_writel(sscr0, ssp->mmio_base + SSCR0);
		clk_disable(ssp->clk);
	}
}

#ifdef CONFIG_PM
static int pxa3xx_ssp_suspend(struct snd_soc_dai *cpu_dai)
{
	struct ssp_priv *priv = cpu_dai->private_data;
	struct ssp_device *ssp = priv->ssp;

	if (!cpu_dai->active)
		return 0;

	priv->cr0 = __raw_readl(ssp->mmio_base + SSCR0);
	priv->cr1 = __raw_readl(ssp->mmio_base + SSCR1);
	priv->to  = __raw_readl(ssp->mmio_base + SSTO);
	priv->psp = __raw_readl(ssp->mmio_base + SSPSP);

	__raw_writel(priv->cr0 & ~SSCR0_SSE, ssp->mmio_base + SSCR0);
	clk_disable(ssp->clk);
	return 0;
}

static int pxa3xx_ssp_resume(struct snd_soc_dai *cpu_dai)
{
	struct ssp_priv *priv = cpu_dai->private_data;
	struct ssp_device *ssp = priv->ssp;
	uint32_t sssr = SSSR_ROR | SSSR_TUR | SSSR_BCE;
	u32 sscr0;

	if (!cpu_dai->active)
		return 0;

	clk_enable(ssp->clk);

	__raw_writel(sssr, ssp->mmio_base + SSSR);
	__raw_writel(priv->cr0 & ~SSCR0_SSE, ssp->mmio_base + SSCR0);
	__raw_writel(priv->cr1, ssp->mmio_base + SSCR1);
	__raw_writel(priv->to,  ssp->mmio_base + SSTO);
	__raw_writel(priv->psp, ssp->mmio_base + SSPSP);

	sscr0 = __raw_readl(ssp->mmio_base + SSCR0);
	sscr0 |= SSCR0_SSE;
	__raw_writel(sscr0, ssp->mmio_base + SSCR0);

	return 0;
}

#else
#define pxa3xx_ssp_suspend	NULL
#define pxa3xx_ssp_resume	NULL
#endif

/*
 * Set the SSP ports SYSCLK.
 */
static int pxa3xx_ssp_set_dai_sysclk(struct snd_soc_dai *cpu_dai,
	int clk_id, unsigned int freq, int dir)
{
	struct ssp_priv *priv = cpu_dai->private_data;
	struct ssp_device *ssp = priv->ssp;
	u32 temp, sscr0;

	sscr0 = __raw_readl(ssp->mmio_base + SSCR0);
	sscr0 &= ~(SSCR0_ECS |  SSCR0_NCS | SSCR0_MOD | SSCR0_ACS);

	switch (clk_id) {
	case PXA3XX_SSP_CLK_PLL:
		ssp_clk[cpu_dai->id].sysclk = 13000000;
		break;
	case PXA3XX_SSP_CLK_EXT:
		ssp_clk[cpu_dai->id].sysclk = freq;
		sscr0 |= SSCR0_ECS;
		break;
	case PXA3XX_SSP_CLK_NET:
		ssp_clk[cpu_dai->id].sysclk = freq;
		sscr0 |= (SSCR0_NCS | SSCR0_MOD);
		break;
	case PXA3XX_SSP_CLK_AUDIO:
		ssp_clk[cpu_dai->id].sysclk = 0;
		temp = __raw_readl(ssp->mmio_base + SSCR0);
		temp |= 0;
		__raw_writel(temp, ssp->mmio_base + SSCR0);
		sscr0 |= SSCR0_ACS;
		break;
	default:
		return -ENODEV;
	}

	/* the SSP CKEN clock must be disabled when changing SSP clock mode */
	clk_disable(ssp->clk);
	sscr0 |= __raw_readl(ssp->mmio_base + SSCR0);
	__raw_writel(sscr0, ssp->mmio_base + SSCR0);
	clk_enable(ssp->clk);

	return 0;
}

/*
 * Set the SSP clock dividers.
 */
static int pxa3xx_ssp_set_dai_clkdiv(struct snd_soc_dai *cpu_dai,
	int div_id, int div)
{
	struct ssp_priv *priv = cpu_dai->private_data;
	struct ssp_device *ssp = priv->ssp;
	u32 ssacd, sscr0;

	switch (div_id) {
	case PXA3XX_SSP_AUDIO_DIV_ACDS:
		ssacd = __raw_readl(ssp->mmio_base + SSACD);
		ssacd &= ~0x7;
		__raw_writel(ssacd, ssp->mmio_base + SSACD);
		ssacd |= SSACD_ACDS(div);
		__raw_writel(ssacd, ssp->mmio_base + SSACD);
		break;
	case PXA3XX_SSP_AUDIO_DIV_ACPS:
		ssacd = __raw_readl(ssp->mmio_base + SSACD);
		ssacd &= ~0x70;
		__raw_writel(ssacd, ssp->mmio_base + SSACD);
		ssacd |= SSACD_ACPS(div);
		__raw_writel(ssacd, ssp->mmio_base + SSACD);
		break;
	case PXA3XX_SSP_AUDIO_DIV_SCDB:
		ssacd = __raw_readl(ssp->mmio_base + SSACD);
		ssacd &= ~(SSACD_SCDB | SSACD_SCDX8);
		__raw_writel(ssacd, ssp->mmio_base + SSACD);
		switch (div) {
		case PXA3XX_SSP_CLK_SCDB_1:
			ssacd |= SSACD_SCDB;
			__raw_writel(ssacd, ssp->mmio_base + SSACD);
			break;
		case PXA3XX_SSP_CLK_SCDB_4:
			break;
		case PXA3XX_SSP_CLK_SCDB_8:
			ssacd |= SSACD_SCDX8;
			__raw_writel(ssacd, ssp->mmio_base + SSACD);
			break;
		default:
			return -EINVAL;
		}
		break;
	case PXA3XX_SSP_DIV_SCR:
		sscr0 = __raw_readl(ssp->mmio_base + SSCR0);
		sscr0 &= ~0x000fff00;
		__raw_writel(sscr0, ssp->mmio_base + SSCR0);
		sscr0 |= (div - 1) << 8;
		__raw_writel(sscr0, ssp->mmio_base + SSCR0);
		break;
	default:
		return -ENODEV;
	}

	return 0;
}

/*
 * Configure the PLL frequency
 */
static int pxa3xx_ssp_set_dai_pll(struct snd_soc_dai *cpu_dai,
	int pll_id, int source, unsigned int freq_in, unsigned int freq_out)
{
	struct ssp_priv *priv = cpu_dai->private_data;
	struct ssp_device *ssp = priv->ssp;
	u32 ssacd;

	ssacd = __raw_readl(ssp->mmio_base + SSACD);
	ssacd &= ~0x70;
	__raw_writel(ssacd, ssp->mmio_base + SSACD);
	__raw_writel(0, ssp->mmio_base + SSACDD);
	switch (freq_out) {
	case 5622000:
		break;
	case 11345000:
		ssacd |= (0x1 << 4);
		__raw_writel(ssacd, ssp->mmio_base + SSACD);
		break;
	case 12235000:
		ssacd |= (0x2 << 4);
		__raw_writel(ssacd, ssp->mmio_base + SSACD);
		break;
	case 14857000:
		ssacd |= (0x3 << 4);
		__raw_writel(ssacd, ssp->mmio_base + SSACD);
		break;
	case 32842000:
		ssacd |= (0x4 << 4);
		__raw_writel(ssacd, ssp->mmio_base + SSACD);
		break;
	case 48000000:
		ssacd |= (0x5 << 4);
		__raw_writel(ssacd, ssp->mmio_base + SSACD);
		break;
	case 12288000:
		ssacd |= (0x6 << 4);
		__raw_writel(ssacd, ssp->mmio_base + SSACD);
		__raw_writel((1625 << 16) | 64, ssp->mmio_base + SSACDD);
		break;
	case 11289600:
		ssacd |= (0x6 << 4);
		__raw_writel(ssacd, ssp->mmio_base + SSACD);
		__raw_writel((1769 << 16) | 64, ssp->mmio_base + SSACDD);
		break;
	case 4096000:
		ssacd |= (0x6 << 4);
		__raw_writel(ssacd, ssp->mmio_base + SSACD);
		__raw_writel((4875 << 16) | 64, ssp->mmio_base + SSACDD);
		break;
	}
	return 0;
}

/*
 * Tristate the SSP DAI lines
 */
static int pxa3xx_ssp_set_dai_tristate(struct snd_soc_dai *cpu_dai,
	int tristate)
{
	struct ssp_priv *priv = cpu_dai->private_data;
	struct ssp_device *ssp = priv->ssp;
	u32 sscr1;

	if (tristate) {
		sscr1 = __raw_readl(ssp->mmio_base + SSCR1);
		sscr1 |= SSCR1_TTE;
		__raw_writel(sscr1, ssp->mmio_base + SSCR1);
	} else {
		sscr1 = __raw_readl(ssp->mmio_base + SSCR1);
		sscr1 &= ~SSCR1_TTE;
		__raw_writel(sscr1, ssp->mmio_base + SSCR1);
	}

	return 0;
}

/*
 * Set up the SSP DAI format.
 * The SSP Port must be inactive before calling this function as the
 * physical interface format is changed.
 */
static int pxa3xx_ssp_set_dai_fmt(struct snd_soc_dai *cpu_dai,
		unsigned int fmt)
{
	struct ssp_priv *priv = cpu_dai->private_data;
	struct ssp_device *ssp = priv->ssp;
	int dai_fmt = fmt & SND_SOC_DAIFMT_FORMAT_MASK;
	u32 sscr0, sscr1, sspsp;

	sscr0 = __raw_readl(ssp->mmio_base + SSCR0);
	if (sscr0 & SSCR0_SSE)
		return 0;

	ssp_clk[cpu_dai->id].dai_fmt = dai_fmt;

	/*
	 * reset port settings
	 * PXA3xx docs say to use RxThresh = 8 and TxThresh = 7 with
	 * DMA bursts of 32
	 */
	__raw_writel(0, ssp->mmio_base + SSCR0);
	sscr1 = SSCR1_RxTresh(8) | SSCR1_TxTresh(7);
	__raw_writel(sscr1, ssp->mmio_base + SSCR1);
	__raw_writel(0, ssp->mmio_base + SSPSP);

	switch (dai_fmt) {
	case SND_SOC_DAIFMT_I2S:
		sscr0 = SSCR0_MOD | SSCR0_PSP;
		__raw_writel(sscr0, ssp->mmio_base + SSCR0);
		sscr1 = __raw_readl(ssp->mmio_base + SSCR1);
		sscr1 |= SSCR1_RWOT | SSCR1_TRAIL;
		__raw_writel(sscr1, ssp->mmio_base + SSCR1);

		switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
		case SND_SOC_DAIFMT_NB_NF:
			sspsp = __raw_readl(ssp->mmio_base + SSPSP);
			sspsp |= SSPSP_FSRT;
			__raw_writel(sspsp, ssp->mmio_base + SSPSP);
			break;
		case SND_SOC_DAIFMT_NB_IF:
			sspsp = __raw_readl(ssp->mmio_base + SSPSP);
			sspsp |= SSPSP_SFRMP | SSPSP_FSRT;
			__raw_writel(sspsp, ssp->mmio_base + SSPSP);
			break;
		case SND_SOC_DAIFMT_IB_IF:
			sspsp = __raw_readl(ssp->mmio_base + SSPSP);
			sspsp |= SSPSP_SFRMP;
			__raw_writel(sspsp, ssp->mmio_base + SSPSP);
			break;
		default:
			return -EINVAL;
		}
		break;
	case SND_SOC_DAIFMT_DSP_A:
	case SND_SOC_DAIFMT_DSP_B:
		sscr0 = SSCR0_MOD | SSCR0_PSP;
		__raw_writel(sscr0, ssp->mmio_base + SSCR0);
		sscr1 = __raw_readl(ssp->mmio_base + SSCR1);
		sscr1 |= SSCR1_TRAIL | SSCR1_RWOT;
		__raw_writel(sscr1, ssp->mmio_base + SSCR1);
		if (dai_fmt == SND_SOC_DAIFMT_DSP_A)
			__raw_writel(SSPSP_FSRT, ssp->mmio_base + SSPSP);

		switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
		case SND_SOC_DAIFMT_NB_NF:
			sspsp = __raw_readl(ssp->mmio_base + SSPSP);
			sspsp |= SSPSP_SFRMP;
			__raw_writel(sspsp, ssp->mmio_base + SSPSP);
			break;
		case SND_SOC_DAIFMT_IB_IF:
			break;
		default:
			return -EINVAL;
		}

		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		sscr1 = __raw_readl(ssp->mmio_base + SSCR1);
		sscr1 |= SSCR1_SCLKDIR | SSCR1_SFRMDIR;
		__raw_writel(sscr1, ssp->mmio_base + SSCR1);
		break;
	case SND_SOC_DAIFMT_CBM_CFS:
		sscr1 = __raw_readl(ssp->mmio_base + SSCR1);
		sscr1 |= SSCR1_SCLKDIR;
		__raw_writel(sscr1, ssp->mmio_base + SSCR1);
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/*
 * Set the SSP audio DMA parameters and sample size.
 * Can be called multiple times by oss emulation.
 */
static int pxa3xx_ssp_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params, struct snd_soc_dai * dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	struct ssp_priv *priv = cpu_dai->private_data;
	struct ssp_device *ssp = priv->ssp;
	struct pxa3xx_pcm_dma_params *pcm_o, *pcm_i;
	u32 sscr0, sspsp;
	u32 width;

	printk("pxa3xx_ssp_hw_params; rate= %d, channels= %d\n", params_rate(params), params_channels(params));

	pcm_o = kmalloc(sizeof(struct pxa3xx_pcm_dma_params), GFP_KERNEL);
	pcm_i = kmalloc(sizeof(struct pxa3xx_pcm_dma_params), GFP_KERNEL);

	pcm_o->name = ssp_names[cpu_dai->id][0];
	pcm_i->name = ssp_names[cpu_dai->id][1];

	/* bit size */
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		sscr0 = __raw_readl(ssp->mmio_base + SSCR0);
		if (params_channels(params) == 1)
			sscr0 &= ~SSCR0_EDSS;
		else
			sscr0 |= SSCR0_EDSS;
		sscr0 |= SSCR0_DataSize(16);
		__raw_writel(sscr0, ssp->mmio_base + SSCR0);

		if ((sscr0 & SSCR0_FPCKE ) || (sscr0 & SSCR0_EDSS))
			width = DCMD_WIDTH4;
		else
			width = DCMD_WIDTH2;

		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		sscr0 = __raw_readl(ssp->mmio_base + SSCR0);
		sscr0 |= SSCR0_EDSS | SSCR0_DataSize(16);
		__raw_writel(sscr0, ssp->mmio_base + SSCR0);

		width = DCMD_WIDTH4;

		break;
	default:
		kfree(pcm_i);
		kfree(pcm_o);
		return -EINVAL;
	}

	switch (params_rate(params)) {
	case 48000:
		snd_soc_dai_set_clkdiv(cpu_dai, PXA3XX_SSP_AUDIO_DIV_ACPS, 0x6);
		snd_soc_dai_set_clkdiv(cpu_dai, PXA3XX_SSP_AUDIO_DIV_SCDB, PXA3XX_SSP_CLK_SCDB_1);
		snd_soc_dai_set_clkdiv(cpu_dai, PXA3XX_SSP_AUDIO_DIV_ACDS, 0x3);
		snd_soc_dai_set_pll(cpu_dai, 0, 0, 0, 12288000);
		break;
	case 44100:
		snd_soc_dai_set_clkdiv(cpu_dai, PXA3XX_SSP_AUDIO_DIV_ACPS, 0x1);
		snd_soc_dai_set_clkdiv(cpu_dai, PXA3XX_SSP_AUDIO_DIV_SCDB, PXA3XX_SSP_CLK_SCDB_4);
		snd_soc_dai_set_clkdiv(cpu_dai, PXA3XX_SSP_AUDIO_DIV_ACDS, 0x1);
		break;
	case 32000:
		snd_soc_dai_set_clkdiv(cpu_dai, PXA3XX_SSP_AUDIO_DIV_ACPS, 0x6);
		snd_soc_dai_set_clkdiv(cpu_dai, PXA3XX_SSP_AUDIO_DIV_SCDB, PXA3XX_SSP_CLK_SCDB_1);
		snd_soc_dai_set_clkdiv(cpu_dai, PXA3XX_SSP_AUDIO_DIV_ACDS, 0x2);
		snd_soc_dai_set_pll(cpu_dai, 0, 0, 0, 4096000);
		break;
	case 16000:
		snd_soc_dai_set_clkdiv(cpu_dai, PXA3XX_SSP_AUDIO_DIV_ACPS, 0x6);
		snd_soc_dai_set_clkdiv(cpu_dai, PXA3XX_SSP_AUDIO_DIV_SCDB, PXA3XX_SSP_CLK_SCDB_1);
		snd_soc_dai_set_clkdiv(cpu_dai, PXA3XX_SSP_AUDIO_DIV_ACDS, 0x3);
		snd_soc_dai_set_pll(cpu_dai, 0, 0, 0, 4096000);
		break;
	case 8000:
		snd_soc_dai_set_clkdiv(cpu_dai, PXA3XX_SSP_AUDIO_DIV_ACPS, 0x6);
		snd_soc_dai_set_clkdiv(cpu_dai, PXA3XX_SSP_AUDIO_DIV_SCDB, PXA3XX_SSP_CLK_SCDB_1);
		snd_soc_dai_set_clkdiv(cpu_dai, PXA3XX_SSP_AUDIO_DIV_ACDS, 0x4);
		snd_soc_dai_set_pll(cpu_dai, 0, 0, 0, 4096000);
		break;
	default:
		break;
	}

	pcm_o->dcmd = DCMD_INCSRCADDR | DCMD_FLOWTRG | DCMD_BURST16 | width;
	pcm_i->dcmd = DCMD_INCTRGADDR | DCMD_FLOWSRC | DCMD_BURST16 | width;
	pcm_o->drcmr = &DRCMR(ssp->drcmr_tx);
	pcm_i->drcmr = &DRCMR(ssp->drcmr_rx);

	pcm_o->dev_addr	= ssp->phys_base + SSDR;
	pcm_i->dev_addr	= ssp->phys_base + SSDR;

	/* select correct DMA params */
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		cpu_dai->playback.dma_data = pcm_o;
		kfree(pcm_i);
	}
	else {
		cpu_dai->capture.dma_data = pcm_i;
		kfree(pcm_o);
	}

	if (ssp_clk[cpu_dai->id].dai_fmt == SND_SOC_DAIFMT_I2S) {
		int sfrmwidth =
			snd_pcm_format_physical_width(params_format(params));
		sspsp = __raw_readl(ssp->mmio_base + SSPSP);
		sspsp |= SSPSP_SFRMWDTH(sfrmwidth);
		__raw_writel(sspsp, ssp->mmio_base + SSPSP);
	}

	return 0;
}

static int pxa3xx_ssp_trigger(struct snd_pcm_substream *substream, int cmd, struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	struct ssp_priv *priv = cpu_dai->private_data;
	struct ssp_device *ssp = priv->ssp;
	int ret = 0;
	u32 sscr0, sscr1, sssr;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_RESUME:
		sscr0 = __raw_readl(ssp->mmio_base + SSCR0);
		sscr0 |= SSCR0_SSE;
		__raw_writel(sscr0, ssp->mmio_base + SSCR0);
		break;
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			sscr1 = __raw_readl(ssp->mmio_base + SSCR1);
			sscr1 |= SSCR1_TSRE;
			__raw_writel(sscr1, ssp->mmio_base + SSCR1);
		} else {
			sscr1 = __raw_readl(ssp->mmio_base + SSCR1);
			sscr1 |= SSCR1_RSRE;
			__raw_writel(sscr1, ssp->mmio_base + SSCR1);
		}
		sssr = __raw_readl(ssp->mmio_base + SSSR);
		__raw_writel(sssr, ssp->mmio_base + SSSR);
		break;
	case SNDRV_PCM_TRIGGER_START:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			sscr1 = __raw_readl(ssp->mmio_base + SSCR1);
			sscr1 |= SSCR1_TSRE;
			__raw_writel(sscr1, ssp->mmio_base + SSCR1);
		} else {
			sscr1 = __raw_readl(ssp->mmio_base + SSCR1);
			sscr1 |= SSCR1_RWOT;
			sscr1 |= SSCR1_RSRE;
			__raw_writel(sscr1, ssp->mmio_base + SSCR1);
		}
		sscr0 = __raw_readl(ssp->mmio_base + SSCR0);
		sscr0 |= SSCR0_SSE;
		__raw_writel(sscr0, ssp->mmio_base + SSCR0);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			sscr1 = __raw_readl(ssp->mmio_base + SSCR1);
			sscr1 &= ~SSCR1_TSRE;
			__raw_writel(sscr1, ssp->mmio_base + SSCR1);
		} else {
			sscr1 = __raw_readl(ssp->mmio_base + SSCR1);
			sscr1 &= ~SSCR1_RSRE;
			__raw_writel(sscr1, ssp->mmio_base + SSCR1);
		}
		break;
	case SNDRV_PCM_TRIGGER_SUSPEND:
		sscr0 = __raw_readl(ssp->mmio_base + SSCR0);
		sscr0 &= ~SSCR0_SSE;
		__raw_writel(sscr0, ssp->mmio_base + SSCR0);
		break;
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			sscr1 = __raw_readl(ssp->mmio_base + SSCR1);
			sscr1 &= ~SSCR1_TSRE;
			__raw_writel(sscr1, ssp->mmio_base + SSCR1);
		} else {
			sscr1 = __raw_readl(ssp->mmio_base + SSCR1);
			sscr1 &= ~SSCR1_RSRE;
			__raw_writel(sscr1, ssp->mmio_base + SSCR1);
		}
		break;

	default:
		ret = -EINVAL;
	}

	return ret;
}

static int pxa3xx_ssp_probe(struct platform_device *pdev,
			    struct snd_soc_dai *dai)
{
	struct ssp_priv *priv;
	int ret;

	priv = kzalloc(sizeof(struct ssp_priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->ssp = pxa_ssp_request(dai->id + 1, "SoC audio");
	if (priv->ssp == NULL) {
		ret = -ENODEV;
		goto err_priv;
	}

	priv->dai_fmt = (unsigned int) -1;
	dai->private_data = priv;

	return 0;

err_priv:
	kfree(priv);
	return ret;
}

static void pxa3xx_ssp_remove(struct platform_device *pdev,
			      struct snd_soc_dai *dai)
{
	struct ssp_priv *priv = dai->private_data;
	pxa_ssp_free(priv->ssp);
}


#define PXA3XX_SSP_RATES 0xffffffff
#define PXA3XX_SSP_FORMATS 0xffffffff

static struct snd_soc_dai_ops pxa3xx_ssp_dai_ops = {
		.startup	= pxa3xx_ssp_startup,
		.shutdown	= pxa3xx_ssp_shutdown,
		.trigger    = pxa3xx_ssp_trigger,
		.hw_params	= pxa3xx_ssp_hw_params,
		.set_fmt	= pxa3xx_ssp_set_dai_fmt,
		.set_pll	= pxa3xx_ssp_set_dai_pll,
		.set_tristate	= pxa3xx_ssp_set_dai_tristate,
		.set_sysclk	= pxa3xx_ssp_set_dai_sysclk,
		.set_clkdiv = pxa3xx_ssp_set_dai_clkdiv,
};

struct snd_soc_dai pxa3xx_ssp_dai[] = {
	{	.name = "pxa3xx-ssp1",
		.id = 0,
		.probe = pxa3xx_ssp_probe,
		.remove = pxa3xx_ssp_remove,
		.suspend = pxa3xx_ssp_suspend,
		.resume = pxa3xx_ssp_resume,
		.playback = {
			.channels_min = 1,
			.channels_max = 2,
			.rates = PXA3XX_SSP_RATES,
			.formats = PXA3XX_SSP_FORMATS,},
		.capture = {
			.channels_min = 1,
			.channels_max = 2,
			.rates = PXA3XX_SSP_RATES,
			.formats = PXA3XX_SSP_FORMATS,},
		.ops = &pxa3xx_ssp_dai_ops,
	},
	{	.name = "pxa3xx-ssp2",
		.id = 1,
		.probe = pxa3xx_ssp_probe,
		.remove = pxa3xx_ssp_remove,
		.suspend = pxa3xx_ssp_suspend,
		.resume = pxa3xx_ssp_resume,
		.playback = {
			.channels_min = 1,
			.channels_max = 2,
			.rates = PXA3XX_SSP_RATES,
			.formats = PXA3XX_SSP_FORMATS,},
		.capture = {
			.channels_min = 1,
			.channels_max = 2,
			.rates = PXA3XX_SSP_RATES,
			.formats = PXA3XX_SSP_FORMATS,},
			.ops = &pxa3xx_ssp_dai_ops,
	},
	{	.name = "pxa3xx-ssp3",
		.id = 2,
		.probe = pxa3xx_ssp_probe,
		.remove = pxa3xx_ssp_remove,
		.suspend = pxa3xx_ssp_suspend,
		.resume = pxa3xx_ssp_resume,
		.playback = {
			.channels_min = 1,
			.channels_max = 2,
			.rates = PXA3XX_SSP_RATES,
			.formats = PXA3XX_SSP_FORMATS,},
		.capture = {
			.channels_min = 1,
			.channels_max = 2,
			.rates = PXA3XX_SSP_RATES,
			.formats = PXA3XX_SSP_FORMATS,},
			.ops = &pxa3xx_ssp_dai_ops,
	},
	{	.name = "pxa3xx-ssp4",
		.id = 3,
		.probe = pxa3xx_ssp_probe,
		.remove = pxa3xx_ssp_remove,
		.suspend = pxa3xx_ssp_suspend,
		.resume = pxa3xx_ssp_resume,
		.playback = {
			.channels_min = 1,
			.channels_max = 2,
			.rates = PXA3XX_SSP_RATES,
			.formats = PXA3XX_SSP_FORMATS,},
		.capture = {
			.channels_min = 1,
			.channels_max = 2,
			.rates = PXA3XX_SSP_RATES|SNDRV_PCM_RATE_32000,
			.formats = PXA3XX_SSP_FORMATS},
			.ops = &pxa3xx_ssp_dai_ops,
	},
	{	.name = "Grayback-ssp",
		.id = 4,
		.playback = {
			.channels_min = 1,
			.channels_max = 1,
			.rates = PXA3XX_SSP_RATES,
			.formats = PXA3XX_SSP_FORMATS,},
		.capture = {
			.channels_min = 1,
			.channels_max = 1,
			.rates = PXA3XX_SSP_RATES,
			.formats = PXA3XX_SSP_FORMATS,},

	},
};
EXPORT_SYMBOL_GPL(pxa3xx_ssp_dai);

static int __init pxa3xx_ssp_modinit(void)
{
	return snd_soc_register_dais(ARRAY_AND_SIZE(pxa3xx_ssp_dai));
}
module_init(pxa3xx_ssp_modinit);

static void __exit pxa3xx_ssp_exit(void)
{
	snd_soc_unregister_dais(ARRAY_AND_SIZE(pxa3xx_ssp_dai));
}
module_exit(pxa3xx_ssp_exit);

/* Module information */
MODULE_AUTHOR("xjian@marvell.com");
MODULE_DESCRIPTION("pxa3xx SSP/PCM SoC Interface");
MODULE_LICENSE("GPL");
