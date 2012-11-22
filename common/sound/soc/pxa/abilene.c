/*
 * linux/sound/soc/pxa/abilene.c
 *
 * Copyright (C) 2009 Marvell International Ltd.
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
#include <linux/moduleparam.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <linux/i2c.h>

#include <asm/mach-types.h>
#include <linux/io.h>

#include <linux/uaccess.h>
#include <plat/ssp.h>
#include <mach/addr-map.h>
#include <mach/regs-sspa.h>
#include <mach/regs-mpmu.h>
#include <mach/regs-apmu.h>

#include <linux/mfd/wm8994/registers.h>
#include "../codecs/wm8994.h"
#include "pxa3xx-ssp.h"
#include "mmp2-squ.h"
#include "mmp2-sspa.h"
#include <linux/delay.h>

#define	MPMU_SPCGR			MPMU_REG(0x0024)
#define MPMU_ISCCR1			MPMU_REG(0x0040)
#define MPMU_ISCCR2			MPMU_REG(0x0044)
#define APMU_AUDIO_CLK_RES_CTRL		APMU_REG(0x010c)
#define SSPA1_VIRT_BASE			(AXI_VIRT_BASE + 0xa0c00)
#define SSPA2_VIRT_BASE			(AXI_VIRT_BASE + 0xa0d00)

#define ABILENE_SAMPLE_RATES (SNDRV_PCM_RATE_48000)

extern struct snd_soc_dai hdmi_audio_dai[1];
extern struct snd_soc_dai wm8994_dai[1];
extern struct snd_soc_codec_device soc_codec_dev_hdmi_audio;
extern struct snd_soc_codec_device soc_codec_dev_wm8994;

extern void mmp2_save_adma_regs(struct snd_pcm_substream *substream);
extern void mmp2_restore_adma_regs(struct snd_pcm_substream *substream);
extern int mmp2_save_desc(struct snd_pcm_substream *substream);
extern void mmp2_restore_desc(struct snd_pcm_substream *substream);
extern void mmp2_save_sspa_regs(struct ssp_device *sspa);
extern void mmp2_restore_sspa_regs(struct ssp_device *sspa);

static int sspa2_rate;
static int sspa1_suspend_post, sspa2_suspend_post;

#ifdef CONFIG_SWITCH_HEADSET
#define WM8994_IRQPIN 1
struct snd_soc_codec *abilene_wm8994_codec;
int wm8994_headset_detect(void)
{
	struct snd_soc_codec *codec;
	u32 temp;
	int i, ret = 0;

	codec = abilene_wm8994_codec;
	if (codec == NULL)
		return 0;

	/* disable the irq */
	codec->write(codec, WM8994_INTERRUPT_STATUS_2_MASK, 0xffff);

	udelay(500);
	/* clear the interrupt status */
	i = WM8994_INTERRUPT_STATUS_2;
	temp = codec->read(codec, i);
	codec->write(codec, i, (temp & 0x18));

	i = WM8994_INTERRUPT_RAW_STATUS_2;
	temp = codec->read(codec, i);
	switch (temp & 0x18) {
	case (0x0):
		ret = 0x0;
		break;
	case (0x08):
		ret = 0x01;
		break;
	case (0x18):
		ret = 0x02;
		break;
	default:
		ret = 0x0;
		break;
	}

	/* re-enable the irq */
	codec->write(codec, WM8994_INTERRUPT_STATUS_2_MASK, 0xffe7);
	return ret;
}
#endif

static int codec_hdmi_init(struct snd_soc_codec *codec)
{
	return 0;
}

static int codec_wm8994_init(struct snd_soc_codec *codec)
{
#ifdef CONFIG_SND_WM8994_SLAVE_MODE
	codec->write(codec, 0x01, 0x3333);
	codec->write(codec, 0x02, 0x6310);
	codec->write(codec, 0x03, 0x03F0);
	codec->write(codec, 0x04, 0x330f);
	codec->write(codec, 0x05, 0x3303);
	codec->write(codec, 0x06, 0x000a);
	codec->write(codec, 0x1A, 0x0112);
	codec->write(codec, 0x28, 0x0003);
	codec->write(codec, 0x2A, 0x0020);

	codec->write(codec, 0x2D, 0x0100);
	codec->write(codec, 0x2E, 0x0100);
	codec->write(codec, 0x4C, 0x9F25);
	codec->write(codec, 0x60, 0x00EE);

	codec->write(codec, 0x22, 0x0000);
	codec->write(codec, 0x23, 0x0000);
	codec->write(codec, 0x36, 0x0003);

	codec->write(codec, 0x200, 0x0001);
	codec->write(codec, 0x204, 0x0001);
	codec->write(codec, 0x208, 0x0007);
	codec->write(codec, 0x208, 0x000E);
	codec->write(codec, 0x210, 0x0073);
	codec->write(codec, 0x211, 0x0073);
	codec->write(codec, 0x300, 0x4010);
	codec->write(codec, 0x310, 0xC010);
	codec->write(codec, 0x311, 0x4000);

	codec->write(codec, 0x400, 0x01C0);
	codec->write(codec, 0x401, 0x01C0);
	codec->write(codec, 0x410, 0x1800);
	codec->write(codec, 0x420, 0x0000);

	codec->write(codec, 0x500, 0x01C0);
	codec->write(codec, 0x501, 0x01C0);
	codec->write(codec, 0x502, 0x01C0);
	codec->write(codec, 0x503, 0x01C0);
	codec->write(codec, 0x520, 0x0030);

	codec->write(codec, 0x601, 0x0005);
	codec->write(codec, 0x602, 0x0005);
	codec->write(codec, 0x604, 0x0020);
	codec->write(codec, 0x605, 0x0020);
	codec->write(codec, 0x606, 0x0002);
	codec->write(codec, 0x607, 0x0002);
	codec->write(codec, 0x610, 0x01C0);
	codec->write(codec, 0x611, 0x01C0);
	codec->write(codec, 0x612, 0x01C0);
	codec->write(codec, 0x613, 0x01C0);

	codec->write(codec, 0x700, 0xA101);
	codec->write(codec, 0x702, 0xA100);
	codec->write(codec, 0x703, 0xA100);
	codec->write(codec, 0x704, 0xA100);
	codec->write(codec, 0x706, 0x2100);
	codec->write(codec, 0x707, 0xa100);
	codec->write(codec, 0x708, 0x2100);
	codec->write(codec, 0x709, 0x2100);
	codec->write(codec, 0x70a, 0x2100);
	msleep(50);
#endif
#ifdef CONFIG_SND_WM8994_MASTER_MODE
	codec->write(codec, 0x01, 0x3333);
	codec->write(codec, 0x05, 0x0303);
	codec->write(codec, 0x208, 0x000A);
	codec->write(codec, 0x210, 0x0073);	/* 44.1K */
	codec->write(codec, 0x224, 0x0c80);	/* FLL1_CLK_REF_DIV by 2 */
	codec->write(codec, 0x221, 0x0700);	/* FLL1_OUTDIV by 8 */
	codec->write(codec, 0x222, 0x3f9d);	/* FLL1_K */
	codec->write(codec, 0x223, 0x00e0);	/* FLL1_N */

	codec->write(codec, 0x220, 0x0005);	/* FLL1 Enable */
	msleep(5);
	codec->write(codec, 0x200, 0x0011);	/* select MCLK1 */

	codec->write(codec, 0x420, 0x0000);
	codec->write(codec, 0x601, 0x0001);
	codec->write(codec, 0x602, 0x0001);
	codec->write(codec, 0x610, 0x01C0);
	codec->write(codec, 0x611, 0x01C0);

	codec->write(codec, 0x700, 0x2103);
	codec->write(codec, 0x300, 0x4070);	/* codec master mode */
	codec->write(codec, 0x304, 0x0840);	/* codec master mode */
	codec->write(codec, 0x400, 0x01C0);
	codec->write(codec, 0x401, 0x01C0);
	codec->write(codec, 0x606, 0x0002);
	codec->write(codec, 0x607, 0x0002);
	msleep(1);
	codec->write(codec, 0x700, 0xA101);

	codec->write(codec, 0x302, 0x7000);	/* codec master mode */
	msleep(5);
	codec->write(codec, 0x300, 0x4070);
	codec->write(codec, 0x303, 0x0040);
	codec->write(codec, 0x304, 0x0840);
	codec->write(codec, 0x305, 0x0840);
#endif
#ifdef CONFIG_SWITCH_HEADSET
	/* 1. setup the interrupt registers */
	/* 1.1 unmask MICBIAS interrupt */
	codec->write(codec, WM8994_INTERRUPT_STATUS_2_MASK, 0xffe7);

	/* 1.2 unmask interrupt mask */
	codec->write(codec, WM8994_INTERRUPT_CONTROL, 0x0);

	/* 1.3 enable debounce */
	codec->write(codec, WM8994_IRQ_DEBOUNCE, 0x18);

	/* 2. setup the toclk */

	ret = codec->read(codec, WM8994_CLOCKING_2);
	ret &= 0x0f;
	ret = ret | 2 << 4 | 4 << 8;
	codec->write(codec, WM8994_CLOCKING_2, ret);

	ret = codec->read(codec, WM8994_CLOCKING_1);
	ret = ret | 1 << 4;
	codec->write(codec, WM8994_CLOCKING_1, ret);

	/* 3. GPIO setup */

	/* 3.1 setup the GPIOn as IRQ output */

	codec->write(codec, WM8994_GPIO_1 - 1 + WM8994_IRQPIN, 0x0003);

	/* codec->write(codec, WM8994_GPIO_1 - 1 + 7, 0x0003); *//* test pin */

	/*
	 * 3.2 setup the current threshold and short threshold
	 *     setup the MICD enable
	 *     setup the MICBIAS lvl
	 */
	codec->write(codec, WM8994_MICBIAS, 0x44);

	/* 3.3 setup the MICBIAS debounce */
	codec->write(codec, WM8994_IRQ_DEBOUNCE, 0x1e);

	/* 3.4 setup the MICBIAS enable */
	ret = codec->read(codec, WM8994_POWER_MANAGEMENT_1);
	ret = ret | 0x3;
	codec->write(codec, WM8994_POWER_MANAGEMENT_1, ret);

	abilene_wm8994_codec = codec;
#endif
	return 0;
}

static ssize_t
sspa2_clk_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", sspa2_rate);
}

static ssize_t sspa2_clk_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t len)
{
	int val;
	struct platform_device *pdev;
	struct snd_soc_device *devdata;
	struct snd_soc_dai *codec_dai = &wm8994_dai[1];
	struct snd_soc_codec *codec;
	int DIV_FBCCLK = 0, FRACT = 0, DIV_OCLK_MODULO = 0, DIV_MCLK = 0;
	int DIV_OCLK_PATTERN = 0;
	int SSPA1_CLK_DIV2_FREE = 0x8, SSPA1_CLK_DIV2_EN = 1, SSPA1_CLK_SEL = 1;
	int SSPA1_CLK_DIV1_FREE = 2, SSPA1_CLK_DIV1_EN = 1;
	u32 reg_val, sspasp;
	int rate = 0;

	pdev = container_of(dev, struct platform_device, dev);
	devdata = platform_get_drvdata(pdev);
	codec = devdata->card->codec;
	codec_dai->codec = codec;

	sspasp = 1 << 31 | 31 << 20 | 1 << 18 | 1 << 16 | 63 << 4 | 6;
	__raw_writel(sspasp, SSPA2_VIRT_BASE + SSPA_RXSP);
	sspasp &= ~(0x6);
	__raw_writel(sspasp | (1 << 31), SSPA2_VIRT_BASE + SSPA_RXSP);

	sscanf(buf, "%u", &val);

	/* As SSPA2 will use SSPA1's register,
	 * here will use SSPAx_VIRT_BASE for convenience
	 */
	if (val) {
		switch (val) {
		case 48000:
			DIV_FBCCLK = 1;
			FRACT = 0x00da1;
			DIV_OCLK_MODULO = 0x1;
			DIV_OCLK_PATTERN = 0x1;
			SSPA1_CLK_DIV2_FREE = 0x8;
			rate = 12288000;
			sspa2_rate = 48000;
			break;
#if 0
		case 8000:
			DIV_FBCCLK = 1;
			FRACT = 0x00da1;
			DIV_OCLK_MODULO = 0x4;
			DIV_OCLK_PATTERN = 0x2;
			SSPA1_CLK_DIV1_FREE = 0X1;
			SSPA1_CLK_DIV2_FREE = 0x8;
			rate = 4096000;
			break;
		case 16000:
			DIV_FBCCLK = 1;
			FRACT = 0x00da1;
			DIV_OCLK_MODULO = 0x2;
			DIV_OCLK_PATTERN = 0x2;
			SSPA1_CLK_DIV2_FREE = 0x8;
			rate = 4096000;
			break;
		case 32000:
			DIV_FBCCLK = 1;
			FRACT = 0x00da1;
			DIV_OCLK_MODULO = 0x1;
			DIV_OCLK_PATTERN = 0x2;
			SSPA1_CLK_DIV2_FREE = 0x8;
			rate = 8192000;
			break;
		case 96000:
			DIV_FBCCLK = 1;
			FRACT = 0x00da1;
			DIV_OCLK_MODULO = 0x1;
			DIV_OCLK_PATTERN = 0x1;
			SSPA1_CLK_DIV2_FREE = 0x4;
			rate = 12288000;
			break;
		case 192000:
			/* Not support */
			break;
		case 11025:
			DIV_FBCCLK = 0;
			FRACT = 0x08a18;
			DIV_OCLK_MODULO = 0x4;
			DIV_OCLK_PATTERN = 0x1;
			SSPA1_CLK_DIV1_FREE = 0X1;
			SSPA1_CLK_DIV2_FREE = 0x8;
			rate = 5644800;
			break;
		case 22050:
			DIV_FBCCLK = 0;
			FRACT = 0x08a18;
			DIV_OCLK_MODULO = 0x2;
			DIV_OCLK_PATTERN = 0x1;
			SSPA1_CLK_DIV2_FREE = 0x8;
			rate = 5644800;
			break;
		case 44100:
			DIV_FBCCLK = 0;
			FRACT = 0x08a18;
			DIV_OCLK_MODULO = 0x1;
			DIV_OCLK_PATTERN = 0x1;
			SSPA1_CLK_DIV2_FREE = 0x8;
			rate = 11289600;
			break;
#endif
		default:
			break;
		}
		codec_dai->ops->set_sysclk(codec_dai, WM8994_SYSCLK_MCLK1,
					   rate, 0);
		reg_val = (1 << 16) | (1 << 11) | DIV_OCLK_PATTERN;
		__raw_writel(reg_val, SSPA1_VIRT_BASE + SSPA_AUD_PLL_CTRL1);
		reg_val = (DIV_OCLK_MODULO << 28) | (FRACT << 8) | 1 << 7 |
		    (DIV_FBCCLK << 3) | DIV_MCLK << 2 | 1;
		__raw_writel(reg_val, SSPA1_VIRT_BASE + SSPA_AUD_PLL_CTRL0);
		reg_val = SSPA1_CLK_DIV2_FREE << 9 | (SSPA1_CLK_DIV2_EN << 8) |
		    (SSPA1_CLK_SEL << 7) | (SSPA1_CLK_DIV1_FREE << 1) |
		    SSPA1_CLK_DIV1_EN | (SSPA1_CLK_SEL << 23) | (8 << 17) |
		    (SSPA1_CLK_DIV1_EN << 16);
		__raw_writel(reg_val, SSPA1_VIRT_BASE + SSPA_AUD_CTRL);
		/* trigger the clock */
		sspasp = __raw_readl(SSPA2_VIRT_BASE + SSPA_RXSP);
		sspasp |= SSPA_SP_S_EN;
		__raw_writel(sspasp | SSPA_SP_WEN, SSPA2_VIRT_BASE + SSPA_RXSP);
		snd_soc_dapm_stream_event(codec, "AIF2 Playback",
					  SND_SOC_DAPM_STREAM_START);
		snd_soc_dapm_stream_event(codec, "AIF3 Playback",
					  SND_SOC_DAPM_STREAM_START);
	} else {
		sspa2_rate = 0;
		sspasp = __raw_readl(SSPA2_VIRT_BASE + SSPA_RXSP);
		sspasp &= ~SSPA_SP_S_EN;
		__raw_writel(sspasp | SSPA_SP_WEN, SSPA2_VIRT_BASE + SSPA_RXSP);
		snd_soc_dapm_stream_event(codec, "AIF2 Playback",
					  SND_SOC_DAPM_STREAM_STOP);
		snd_soc_dapm_stream_event(codec, "AIF3 Playback",
					  SND_SOC_DAPM_STREAM_STOP);

	}

	return len;
}

static DEVICE_ATTR(sspa2_clk, S_IRUGO | S_IWUSR,
		   sspa2_clk_show, sspa2_clk_store);

static struct platform_device *abilene_snd_device[3];

#ifdef CONFIG_PM
static int sspa1_pwr_stats, sspa2_pwr_stats;

static int sspa_clk_enable(void)
{
	__raw_writel(0x0600, APMU_AUDIO_CLK_RES_CTRL);
	udelay(10);
	__raw_writel(0x0610, APMU_AUDIO_CLK_RES_CTRL);
	udelay(10);
	__raw_writel(0x0710, APMU_AUDIO_CLK_RES_CTRL);
	udelay(10);
	__raw_writel(0x0712, APMU_AUDIO_CLK_RES_CTRL);
	udelay(10);

	return 0;
}

static int sspa_clk_disable(void)
{
	int audio_clk_res;

	/* Power off the Audio controller and disable the clock */
	audio_clk_res = __raw_readl(APMU_AUDIO_CLK_RES_CTRL);
	audio_clk_res &= ~(1 << 1);
	__raw_writel(audio_clk_res, APMU_AUDIO_CLK_RES_CTRL);
	udelay(10);
	audio_clk_res &= ~(1 << 8);
	__raw_writel(audio_clk_res, APMU_AUDIO_CLK_RES_CTRL);
	udelay(10);
	audio_clk_res &= ~(1 << 4);
	__raw_writel(audio_clk_res, APMU_AUDIO_CLK_RES_CTRL);
	udelay(10);
	audio_clk_res &= ~(3 << 9);
	__raw_writel(audio_clk_res, APMU_AUDIO_CLK_RES_CTRL);
	udelay(10);

	return 0;
}

int sspa_pll_enable(void)
{
	/* select audio pll : 34M */
	__raw_writel(0x10800, SSPA1_VIRT_BASE + SSPA_AUD_PLL_CTRL1);
	/* div : 12, 44.1K */
	__raw_writel(0x00001185, SSPA1_VIRT_BASE + SSPA_AUD_CTRL);

	return 0;
}

static ssize_t enable_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	if (dev == &abilene_snd_device[0]->dev)
		return sprintf(buf, "%u\n", sspa1_pwr_stats);
	if (dev == &abilene_snd_device[1]->dev)
		return sprintf(buf, "%u\n", sspa2_pwr_stats);
	return 0;
}

static ssize_t enable_store(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t len)
{
	int val, sspa_sel;
	unsigned long reg;
	struct platform_device *pdev = container_of
	    (dev, struct platform_device, dev);
	struct snd_soc_device *devdata = platform_get_drvdata(pdev);
	struct snd_soc_card *card = devdata->card;
	struct snd_soc_dai_link *dai_link = card->dai_link;
	struct ssp_priv *priv = dai_link->cpu_dai->private_data;
	struct ssp_device *sspa = priv->ssp;
	struct snd_pcm_substream *substream;
	int stream;

	sscanf(buf, "%u", &val);
	sspa_sel = 0;
	if (&abilene_snd_device[0]->dev == dev)
		sspa_sel = 1;
	else if (&abilene_snd_device[1]->dev == dev)
		sspa_sel = 2;

	if (val) {
		if ((sspa1_pwr_stats && sspa_sel == 1) ||
		    (sspa2_pwr_stats && sspa_sel == 2))
			return len;

		if ((!sspa2_pwr_stats && sspa_sel == 1) ||
		    (!sspa1_pwr_stats && sspa_sel == 2))
			sspa_clk_enable();

		if (sspa_sel == 1) {
			/* SSPA1 SYSCLK source: APB CLK = 24.576 * 2MHz */
			__raw_writel(0xd3721e95, MPMU_ISCCR1);
			reg = __raw_readl(MPMU_SPCGR);
			reg |= 0x20;
			__raw_writel(reg, MPMU_SPCGR);
			sspa1_pwr_stats = 1;
		} else if (sspa_sel == 2) {
			/* SSPA2 BITCLK : 12M audio pll bitclk input from PMU */
			__raw_writel(0xd0040040, MPMU_ISCCR2);
			reg = __raw_readl(MPMU_SPCGR);
			reg |= 0x100000;
			__raw_writel(reg, MPMU_SPCGR);
			sspa2_pwr_stats = 1;
		}

		if ((!sspa2_pwr_stats && sspa_sel == 1) ||
		    (!sspa1_pwr_stats && sspa_sel == 2))
			sspa_pll_enable();

		for (stream = 0; stream < 2; stream++) {
			substream = dai_link->pcm->streams[stream].substream;
			mmp2_restore_desc(substream);
			mmp2_restore_adma_regs(substream);
		}
		mmp2_restore_sspa_regs(sspa);
	} else {
		if ((!sspa1_pwr_stats && sspa_sel == 1) ||
		    (!sspa2_pwr_stats && sspa_sel == 2))
			return len;

		if (attr == NULL) {
			if (sspa_sel == 1)
				sspa1_suspend_post = 1;
			else if (sspa_sel == 2)
				sspa2_suspend_post = 1;
		}

		for (stream = 0; stream < 2; stream++) {
			substream = dai_link->pcm->streams[stream].substream;
			mmp2_save_desc(substream);
			mmp2_save_adma_regs(substream);
		}
		mmp2_save_sspa_regs(sspa);

		if (sspa_sel == 1) {
			/* disable I2S clock output of the main PMU to SSPA1 */
			reg = __raw_readl(MPMU_SPCGR);
			reg &= ~0x20;
			__raw_writel(reg, MPMU_SPCGR);
			/* disable the I2S clock input to SYSCLKx */
			reg = __raw_readl(MPMU_ISCCR1);
			reg &= ~(1 << 31);
			__raw_writel(reg, MPMU_ISCCR1);
			sspa1_pwr_stats = 0;
		} else if (sspa_sel == 2) {
			/* disable I2S clock output of the main PMU to SSPA2 */
			reg = __raw_readl(MPMU_SPCGR);
			reg &= ~0x100000;
			__raw_writel(reg, MPMU_SPCGR);
			/* disable the I2S clock input to SYSCLKx */
			reg = __raw_readl(MPMU_ISCCR2);
			reg &= ~(1 << 31);
			__raw_writel(reg, MPMU_ISCCR2);
			sspa2_pwr_stats = 0;
		}

		if ((!sspa2_pwr_stats && sspa_sel == 1) ||
		    (!sspa1_pwr_stats && sspa_sel == 2))
			sspa_clk_disable();
	}

	return len;
}

static DEVICE_ATTR(enable, S_IRUGO | S_IWUSR, enable_show, enable_store);
#endif

static int abilene_probe(struct platform_device *pdev)
{
	/* CPU PMU audio clock, enable SRAM bank 3 */
	unsigned long reg;
	int ret;
	struct snd_soc_device *devdata;

	__raw_writel(0x0600, APMU_AUDIO_CLK_RES_CTRL);
	udelay(10);
	__raw_writel(0x0610, APMU_AUDIO_CLK_RES_CTRL);
	udelay(10);
	__raw_writel(0x0710, APMU_AUDIO_CLK_RES_CTRL);
	udelay(10);
	__raw_writel(0x0712, APMU_AUDIO_CLK_RES_CTRL);
	udelay(10);

	/* SSPA1 SYSCLK source: APB CLK = 24.576 * 2MHz */
	__raw_writel(0xd3721e95, MPMU_ISCCR1);
	/* SSPA2 SYSCLK : 12M */
	__raw_writel(0xd0040040, MPMU_ISCCR2);
	reg = __raw_readl(MPMU_SPCGR);
	reg |= 0x20;
	__raw_writel(reg, MPMU_SPCGR);

	/* select audio pll : 34M */
	__raw_writel(0x10800, SSPA1_VIRT_BASE + SSPA_AUD_PLL_CTRL1);
	/* div : 12, 44.1K */
	/*(* ((volatile unsigned long *) (0xfe2a0c34))) = 0x211921; */
	__raw_writel(0x00001185, SSPA1_VIRT_BASE + SSPA_AUD_CTRL);

#ifdef CONFIG_PM
	if (device_create_file(&pdev->dev, &dev_attr_enable))
		printk(KERN_ERR "sspa: creating proc file faild");
#endif
	devdata = platform_get_drvdata(pdev);
	if (strcmp(devdata->card->name, "wm8994") == 0)
		ret = device_create_file(&pdev->dev, &dev_attr_sspa2_clk);

	return 0;
}

static int abilene_hifi_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai_link *machine = rtd->dai;
	struct snd_soc_dai *cpu_dai = machine->cpu_dai;

	cpu_dai->playback.channels_min = 2;
	cpu_dai->playback.channels_max = 2;

	cpu_dai->playback.formats = SNDRV_PCM_FMTBIT_S16_LE;
	cpu_dai->playback.rates = ABILENE_SAMPLE_RATES;

	return 0;
}

static int abilene_wm8994_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai_link *machine = rtd->dai;
	struct snd_soc_dai *cpu_dai = machine->cpu_dai;
	struct snd_soc_dai *codec_dai = machine->codec_dai;

	int format;

	cpu_dai->playback.channels_min = 2;
	cpu_dai->playback.channels_max = 2;
	cpu_dai->capture.channels_min = 2;
	cpu_dai->capture.channels_max = 2;

	cpu_dai->playback.formats = SNDRV_PCM_FMTBIT_S16_LE;
	cpu_dai->capture.formats = SNDRV_PCM_FMTBIT_S16_LE;
	cpu_dai->playback.rates = ABILENE_SAMPLE_RATES;
	cpu_dai->capture.rates = ABILENE_SAMPLE_RATES;

	format = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_CBS_CFS
	    | SND_SOC_DAIFMT_NB_NF;
	codec_dai->ops->set_fmt(codec_dai, format);
	codec_dai->ops->set_sysclk(codec_dai, WM8994_SYSCLK_MCLK1, 11289600, 0);

	return 0;
}

#if 0
extern void hdmi_audio_cfg(u32 sr, u32 i2s, u32 wsp, u32 fsp, u32 clkp,
			   u32 xdatdly, u32 xwdlen1, u32 xfrlen1);
#endif

static int abilene_hifi_prepare(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai_link *machine = rtd->dai;
	struct snd_soc_dai *cpu_dai = machine->cpu_dai;
	struct ssp_priv *priv = cpu_dai->private_data;
	struct ssp_device *sspa = priv->ssp;
	struct snd_pcm_runtime *runtime = substream->runtime;
	unsigned long rate = runtime->rate;
	unsigned long channels = runtime->channels;
	unsigned int value_ctl, value_sp;
	unsigned int fper, fwid;
	unsigned int xfrlen1, xwdlen1, xdatdly, xssz1, jst;

	xfrlen1 = channels - 1;
	fper = 63;		/* 64 bits of frame period */
	fwid = 31;		/* 31 bits of frame sync pulse */
	xwdlen1 = 5;		/* 32 bits per channel word */
	xssz1 = 2;		/* 16 bits sample word length  */
	xdatdly = 1;		/* 1 bits  data bit delay */
	jst = 0;		/* i2s with 1 bit delay */

	/*enable RX before enabling TX */
	value_sp = 1 << 31 | fwid << 20 | 1 << 18 | fper << 4 | 6;
	__raw_writel(value_sp, sspa->mmio_base + SSPA_RXSP);
	value_ctl = xdatdly << 19 | xfrlen1 << 8 | xwdlen1 << 5 | xssz1;
	__raw_writel(value_ctl, sspa->mmio_base + SSPA_RXCTL);
	value_sp &= ~(0x6);
	/*msl = 1, master mode, start RX */
	__raw_writel(value_sp | (1 << 31), sspa->mmio_base + SSPA_RXSP);

	/* tx slave mode */
	value_sp = 1 << 31 | fwid << 20 | fper << 4 | 6;
	/*msl = 0, slave mode */
	__raw_writel(value_sp, sspa->mmio_base + SSPA_TXSP);
	value_ctl = xdatdly << 19 | xwdlen1 << 5 | xfrlen1 << 8 | xssz1;
	__raw_writel(value_ctl, sspa->mmio_base + SSPA_TXCTL);
	__raw_writel(1, sspa->mmio_base + SSPA_TXID);

	__raw_writel(1, sspa->mmio_base + SSPA_TXFIFO_LL);	/*need tune */
	/*__raw_writel(0x1, sspa->mmio_base + SSPA_TXINT_MASK);*//*enable int */

	value_sp &= ~(0x6);

	__raw_writel(value_sp | (1 << 31), sspa->mmio_base + SSPA_TXSP);

	/* remove hdmi for quick enabling */
#if 0
	hdmi_audio_cfg(rate, 1, 0, 0, 0, xdatdly, xwdlen1, xfrlen1);
#endif
	return 0;
}

static int abilene_wm8994_prepare(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai_link *machine = rtd->dai;
	struct snd_soc_dai *cpu_dai = machine->cpu_dai;
	struct snd_soc_dai *codec_dai = machine->codec_dai;
	struct ssp_priv *priv = cpu_dai->private_data;
	struct ssp_device *sspa = priv->ssp;
	struct snd_pcm_runtime *runtime = substream->runtime;
	unsigned long channels = runtime->channels;
	unsigned int value_ctl, value_sp;
	unsigned int fper, fwid;
	unsigned int xfrlen1, xwdlen1, xdatdly, xssz1, jst;

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE &&
	    codec_dai->playback.active)
		return 0;

	xfrlen1 = channels - 1;
	fper = 63;		/* 64 bits of frame period */
	fwid = 31;		/* 31 bits of frame sync pulse */
	xwdlen1 = 5;		/* 32 bits per channel word */
	xssz1 = 2;		/* 16 bits sample word length  */
	xdatdly = 1;		/* 1 bits  data bit delay */
	jst = 0;		/* i2s with 1 bit delay */

#ifdef CONFIG_SND_WM8994_MASTER_MODE
	value_sp = 1 << 31 | fwid << 20 | 1 << 16 | fper << 4 | 6;
#endif
#ifdef CONFIG_SND_WM8994_SLAVE_MODE
	value_sp = 1 << 31 | fwid << 20 | 1 << 18 | 1 << 16 | fper << 4 | 6;
#endif
	__raw_writel(value_sp, sspa->mmio_base + SSPA_RXSP);
	value_ctl = xdatdly << 19 | xfrlen1 << 8 | xwdlen1 << 5 | xssz1;
	__raw_writel(value_ctl, sspa->mmio_base + SSPA_RXCTL);
	__raw_writel(0x0, sspa->mmio_base + SSPA_RXFIFO_UL);
	value_sp &= ~(0x6);
	__raw_writel(value_sp | (1 << 31), sspa->mmio_base + SSPA_RXSP);
	/*msl = 1, master mode, start RX */
	/* tx slave mode */
	value_sp = 1 << 31 | fwid << 20 | 1 << 16 | fper << 4 | 6;
	__raw_writel(value_sp, sspa->mmio_base + SSPA_TXSP);
	/*msl = 0, slave mode */
	value_ctl = xdatdly << 19 | xwdlen1 << 5 | xfrlen1 << 8 | xssz1;
	__raw_writel(value_ctl, sspa->mmio_base + SSPA_TXCTL);

	__raw_writel(1, sspa->mmio_base + SSPA_TXFIFO_LL);
	/*need tune */
	/*__raw_writel(0x1, sspa->mmio_base + SSPA_TXINT_MASK);*//*enable int */

	value_sp &= ~(0x6);
	__raw_writel(value_sp | (1 << 31), sspa->mmio_base + SSPA_TXSP);
	/*msl = 0, slave mode */
	return 0;
}

static int abilene_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	struct ssp_priv *priv = cpu_dai->private_data;
	struct ssp_device *sspa = priv->ssp;
	int DIV_FBCCLK = 0, FRACT = 0, DIV_OCLK_MODULO = 0, DIV_MCLK = 0;
	int DIV_OCLK_PATTERN = 0;
	int SSPA1_CLK_DIV2_FREE = 0x8, SSPA1_CLK_DIV2_EN = 1, SSPA1_CLK_SEL = 1;
	int SSPA1_CLK_DIV1_FREE = 2, SSPA1_CLK_DIV1_EN = 1;
	u32 reg_val;
	int rate = 0;

	switch (params_rate(params)) {
	case 8000:
		DIV_FBCCLK = 1;
		FRACT = 0x00da1;
		DIV_OCLK_MODULO = 0x4;
		DIV_OCLK_PATTERN = 0x2;
		SSPA1_CLK_DIV1_FREE = 0X1;
		SSPA1_CLK_DIV2_FREE = 0x8;
		rate = 4096000;
		break;
	case 16000:
		DIV_FBCCLK = 1;
		FRACT = 0x00da1;
		DIV_OCLK_MODULO = 0x2;
		DIV_OCLK_PATTERN = 0x2;
		SSPA1_CLK_DIV2_FREE = 0x8;
		rate = 4096000;
		break;
	case 32000:
		DIV_FBCCLK = 1;
		FRACT = 0x00da1;
		DIV_OCLK_MODULO = 0x1;
		DIV_OCLK_PATTERN = 0x2;
		SSPA1_CLK_DIV2_FREE = 0x8;
		rate = 8192000;
		break;
	case 48000:
		DIV_FBCCLK = 1;
		FRACT = 0x00da1;
		DIV_OCLK_MODULO = 0x1;
		DIV_OCLK_PATTERN = 0x1;
		SSPA1_CLK_DIV2_FREE = 0x8;
		rate = 12288000;
		break;
	case 96000:
		DIV_FBCCLK = 1;
		FRACT = 0x00da1;
		DIV_OCLK_MODULO = 0x1;
		DIV_OCLK_PATTERN = 0x1;
		SSPA1_CLK_DIV2_FREE = 0x4;
		rate = 12288000;
		break;
	case 192000:
		/* Not support */
		break;
	case 11025:
		DIV_FBCCLK = 0;
		FRACT = 0x08a18;
		DIV_OCLK_MODULO = 0x4;
		DIV_OCLK_PATTERN = 0x1;
		SSPA1_CLK_DIV1_FREE = 0X1;
		SSPA1_CLK_DIV2_FREE = 0x8;
		rate = 5644800;
		break;
	case 22050:
		DIV_FBCCLK = 0;
		FRACT = 0x08a18;
		DIV_OCLK_MODULO = 0x2;
		DIV_OCLK_PATTERN = 0x1;
		SSPA1_CLK_DIV2_FREE = 0x8;
		rate = 5644800;
		break;
	case 44100:
		DIV_FBCCLK = 0;
		FRACT = 0x08a18;
		DIV_OCLK_MODULO = 0x1;
		DIV_OCLK_PATTERN = 0x1;
		SSPA1_CLK_DIV2_FREE = 0x8;
		rate = 11289600;
		break;
	default:
		break;
	}

	/* WM8994 need set the sysclk */
	if (strcmp(codec_dai->name, "WM8994 AIF1") == 0 ||
	    strcmp(codec_dai->name, "WM8994 AIF2") == 0)
		codec_dai->ops->set_sysclk(codec_dai,
					   WM8994_SYSCLK_MCLK1, rate, 0);

	/* setup the SSPA clock related registers */
	reg_val = (1 << 16) | (1 << 11) | DIV_OCLK_PATTERN;
	__raw_writel(reg_val, SSPA1_VIRT_BASE + SSPA_AUD_PLL_CTRL1);
	reg_val = (DIV_OCLK_MODULO << 28) | (FRACT << 8) | 1 << 7 |
	    (DIV_FBCCLK << 3) | DIV_MCLK << 2 | 1;
	__raw_writel(reg_val, SSPA1_VIRT_BASE + SSPA_AUD_PLL_CTRL0);
	reg_val = SSPA1_CLK_DIV2_FREE << 9 | (SSPA1_CLK_DIV2_EN << 8) |
	    (SSPA1_CLK_SEL << 7) | (SSPA1_CLK_DIV1_FREE << 1) |
	    SSPA1_CLK_DIV1_EN | (SSPA1_CLK_SEL << 23) | (8 << 17) |
	    (SSPA1_CLK_DIV1_EN << 16);
	__raw_writel(reg_val, SSPA1_VIRT_BASE + SSPA_AUD_CTRL);
	return 0;
}

static void abilene_hifi_shutdown(struct snd_pcm_substream *substream)
{
}

#ifdef CONFIG_PM
static int abilene_suspend_post(struct platform_device *pdev,
				pm_message_t state)
{
	struct snd_soc_device *devdata;
	struct snd_soc_card *card;

	devdata = platform_get_drvdata(pdev);
	card = devdata->card;
	enable_store(&pdev->dev, NULL, "0", 1);

	return 0;
}

static int abilene_resume_pre(struct platform_device *pdev)
{
	struct snd_soc_device *devdata;
	struct snd_soc_card *card;

	devdata = platform_get_drvdata(pdev);
	card = devdata->card;
	if (strcmp(card->name, "hdmi") == 0) {
		if (sspa1_suspend_post == 1) {
			enable_store(&pdev->dev, NULL, "1", 1);
			sspa1_suspend_post = 0;
		}
	} else if (strcmp(card->name, "wm8994") == 0) {
		if (sspa2_suspend_post == 1) {
			enable_store(&pdev->dev, NULL, "1", 1);
			sspa2_suspend_post = 0;
		}
	}
	return 0;
}

#endif

/* machine stream operations */
static struct snd_soc_ops abilene_machine_ops[] = {
	{
	 .startup = abilene_hifi_startup,
	 .prepare = abilene_hifi_prepare,
	 .hw_params = abilene_hw_params,
	 .shutdown = abilene_hifi_shutdown,
	 },
	{
	 .startup = abilene_wm8994_startup,
	 .prepare = abilene_wm8994_prepare,
	 .hw_params = abilene_hw_params,
	 .shutdown = abilene_hifi_shutdown,
	 },
};

/* digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link abilene_dai[] = {
#if 0
	{
	 .name = "I2S",
	 .stream_name = "I2S HiFi",
	 .cpu_dai = &pxa688_sspa_dai[0],
	 .codec_dai = &hdmi_audio_dai[0],
	 .ops = &abilene_machine_ops[0],
	 .init = codec_hdmi_init,
	 },
#endif
	{
	 .name = "WM8994 Voice",
	 .stream_name = "Wm8994 Voice",
	 .cpu_dai = &pxa688_sspa_dai[0],
	 .codec_dai = &wm8994_dai[0],
	 .ops = &abilene_machine_ops[1],
	 .init = codec_wm8994_init,
	 },
};

/* audio machine driver */
static struct snd_soc_card snd_soc_machine_abilene[] = {
#if 0
	{.name = "hdmi",
	 .platform = &mmp2_soc_platform,
	 .dai_link = &abilene_dai[0],
	 .num_links = 1,
	 .probe = abilene_probe,
#ifdef CONFIG_PM
	 .suspend_post = abilene_suspend_post,
	 .resume_pre = abilene_resume_pre,
#endif
	 },
#endif
	{.name = "wm8994",
	 .platform = &mmp2_soc_platform,
	 .dai_link = &abilene_dai[0],
	 .num_links = 1,
	 .probe = abilene_probe,
#ifdef CONFIG_PM
	 .suspend_post = abilene_suspend_post,
	 .resume_pre = abilene_resume_pre,
#endif
	 },
};

/* audio subsystem */
static struct snd_soc_device abilene_snd_devdata[] = {
#if 0
	{
	 .card = &snd_soc_machine_abilene[0],
	 .codec_dev = &soc_codec_dev_hdmi_audio,
	 },
#endif
	{
	 .card = &snd_soc_machine_abilene[0],
	 .codec_dev = &soc_codec_dev_wm8994,
	 },
};

static int __init abilene_init(void)
{
	int ret[3];

	if (!machine_is_abilene())
		return -ENODEV;
#if 0
	abilene_snd_device[1] = platform_device_alloc("soc-audio", 1);
	if (!abilene_snd_device[1])
		return -ENOMEM;
	platform_set_drvdata(abilene_snd_device[1], &abilene_snd_devdata[1]);
	abilene_snd_devdata[1].dev = &abilene_snd_device[1]->dev;
	ret[1] = platform_device_add(abilene_snd_device[1]);

	if (ret[1])
		platform_device_put(abilene_snd_device[1]);
#endif
	abilene_snd_device[0] = platform_device_alloc("soc-audio", 0);
	if (!abilene_snd_device[0])
		return -ENOMEM;
	platform_set_drvdata(abilene_snd_device[0], &abilene_snd_devdata[0]);
	abilene_snd_devdata[0].dev = &abilene_snd_device[0]->dev;
	ret[0] = platform_device_add(abilene_snd_device[0]);

	if (ret[0])
		platform_device_put(abilene_snd_device[0]);

#ifdef CONFIG_PM
	sspa1_pwr_stats = 1;
	sspa2_pwr_stats = 1;
#endif

	return ret[0] | ret[1];
}

static void __exit abilene_exit(void)
{
	platform_device_unregister(abilene_snd_device[0]);
	platform_device_unregister(abilene_snd_device[1]);
}

module_init(abilene_init);
module_exit(abilene_exit);

/* Module information */
MODULE_DESCRIPTION("ALSA SoC ABILENE");
MODULE_LICENSE("GPL");
