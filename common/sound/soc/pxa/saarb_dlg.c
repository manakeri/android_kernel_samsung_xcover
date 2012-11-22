/*
 * linux/sound/soc/pxa/saarb.c
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

#include <asm/mach-types.h>
#include <linux/io.h>

#include <mach/regs-ost.h>
#include <mach/hardware.h>
#include <mach/audio.h>
#include <plat/ssp.h>
#include <mach/gpio.h>
#include <mach/pxa3xx-regs.h>

#include "pxa95x-pcm-ssp.h"
#include "pxa3xx-ssp.h"
#include <mach/audio.h>


extern void pxa95x_abu_mfp_init(bool abu);

#define SAARB_SND_CARD_NUM 2	/* ASoC sound card abstraction number */
#define I2S_SSP_PORT	3	  /* used as ap hifi port */

#define ARRAY_AND_SIZE(x)       (x), ARRAY_SIZE(x)

#if defined(CONFIG_SND_SOC_D1981)
extern struct snd_soc_dai d1981_audio_dai[2];
extern struct snd_soc_codec_device soc_codec_dev_d1981_audio;
#else
extern struct snd_soc_dai pm860x_audio_dai[2];
extern struct snd_soc_codec_device soc_codec_dev_pm860x_audio;
#endif /* CONFIG_SND_SOC_D1981 */

extern struct snd_soc_dai hdmi_audio_dai[1];
extern struct snd_soc_codec_device soc_codec_dev_hdmi_audio;

extern struct snd_soc_dai pxa95x_abu_dai[1];
extern struct snd_soc_platform pxa95x_abu_soc_platform;

extern struct snd_soc_dai pxa3xx_ssp_dai[5];
extern struct snd_soc_platform pxa95x_ssp_soc_platform;

#if defined(CONFIG_SND_SOC_D1981)
static int saarb_d1981_init(struct snd_soc_codec *codec)
{
	pr_debug("[audio]-->saarb_d1981_init: start.\n");
	return 0;
}
#else
static int saarb_pm860x_init(struct snd_soc_codec *codec)
{
	pr_debug("[audio]-->saarb_pm860x_init: start.\n");
	return 0;
}
#endif

static int saarb_abu_pm860x_hifi_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai_link *machine = rtd->dai;
	struct snd_soc_dai *cpu_dai = machine->cpu_dai;

	cpu_dai->playback.channels_min = 2;
	cpu_dai->playback.channels_max = 2;
	cpu_dai->capture.channels_min = 2;
	cpu_dai->capture.channels_max = 2;

	/* The rate is set according to the combination below:
	 * 1. codec:    sound/soc/codecs/pm860x-audio.c
	 * 2. ssp/abu:  sound/soc/pxa/pxa3xx-ssp.c / pxa95x-abu.c
	 * 3. dma:      sound/soc/pxa/pxa95x-pcm-ssp.c
	 * 4. platform: here!
	 * Allowing here 44.1 KHz rate will enforce working with this rate only!
	 */
	cpu_dai->playback.formats = SNDRV_PCM_FMTBIT_S16_LE;
	cpu_dai->capture.formats = SNDRV_PCM_FMTBIT_S16_LE;

	cpu_dai->playback.rates = SNDRV_PCM_RATE_44100;
	cpu_dai->capture.rates = SNDRV_PCM_RATE_44100;

	/* configure GPIOs for ABU/SSI */
	pr_debug("[audio]-->saarb_abu_pm860x_hifi_startup: configure GPIOs for ABU/SSI.\n");

	pxa95x_abu_mfp_init(true);
	
/*	enable_oscc_pout();*/
	return 0;
}

static int saarb_abu_pm860x_hifi_prepare(struct snd_pcm_substream *substream)
{
	pr_debug("[audio]-->saarb_abu_pm860x_hifi_prepare: start.\n");

	return 0;
}

static void saarb_abu_pm860x_hifi_shutdown(struct snd_pcm_substream *substream)
{
	pr_debug("[audio]-->saarb_abu_pm860x_hifi_shutdown: start.\n");

/*	disable_oscc_pout();*/
}


static int saarb_ssp_hdmi_hifi_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai_link *machine = rtd->dai;
	struct snd_soc_dai *cpu_dai = machine->cpu_dai;

	struct ssp_priv *ssp_private = cpu_dai->private_data;
	struct ssp_device *ssp = ssp_private->ssp;
	u32 sscr0, sscr1;

	cpu_dai->playback.channels_min = 2;
	cpu_dai->playback.channels_max = 2;
	cpu_dai->capture.channels_min = 2;
	cpu_dai->capture.channels_max = 2;
	/* The rate is set according to the combination below:
	 * 1. codec:    sound/soc/codecs/levante.c
	 * 2. ssp/abu:  sound/soc/pxa/pxa-3xx-ssp.c / pxa95x-abu.c
	 * 3. dma:      sound/soc/pxa/pxa-3xx-pcm.c
	 * 4. platform: here!
	 * Allowing here 44.1 KHz rate will enforce working with this rate only!
	 */
	cpu_dai->playback.formats = SNDRV_PCM_FMTBIT_S16_LE;
	cpu_dai->capture.formats = SNDRV_PCM_FMTBIT_S16_LE;

	cpu_dai->playback.rates = SNDRV_PCM_RATE_44100;
	cpu_dai->capture.rates = SNDRV_PCM_RATE_44100;

	/* need to add ssp initialization, otherwise when audio(ssp) first
	 * works, it sounds bad as if ssp is un-initialized */
	sscr0 = __raw_readl(ssp->mmio_base + SSCR0);
	sscr1 = __raw_readl(ssp->mmio_base + SSCR1);
	sscr0 |= 0x41D0003F;
	sscr1 |= 0x00B01DC0;
	sscr1 &= ~0x03000000;	/* SSP2 as master for generating hdmi clocks */
	__raw_writel(sscr0, ssp->mmio_base + SSCR0);
	__raw_writel(sscr1, ssp->mmio_base + SSCR1);
	__raw_writel(0x02100004, ssp->mmio_base + SSPSP);

	/* configure GPIO for BSSP2 */
	pr_debug("[audio]-->saarb_ssp_hdmi_hifi_startup: configure GPIOs for BSSP2 ***\n");

		pxa95x_abu_mfp_init(false);

	return 0;
}

static int saarb_ssp_hdmi_hifi_prepare(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai_link *machine = rtd->dai;
	struct snd_soc_dai *cpu_dai = machine->cpu_dai;
	struct ssp_priv *ssp_private = cpu_dai->private_data;
	struct ssp_device *ssp = ssp_private->ssp;
	unsigned long sscr0, sscr1, sspsp, sstsa, ssrsa;

	pr_debug("[audio]-->saarb_ssp_hdmi_hifi_prepare: start.\n");

	/* Because the internal 13M clock will be 10M in D0CS,
	 * we route SSP_CLK to GPIO126(EXT_CLK) and let SSP select
	 * NETWORK CLK as CLK source.
	 * This workaround need an ECO on Littleton mainboard.
	 */

	sscr0 = __raw_readl(ssp->mmio_base + SSCR0);
	sscr1 = __raw_readl(ssp->mmio_base + SSCR1);
	sscr0 |= 0x41D0003F;
	sscr1 |= 0x00B01DC0;
	sscr1 &= ~0x03000000;	/* SSP2 as master for generating hdmi clocks */
	sspsp = 0x02100004;
	sstsa = 0x00000003;
	ssrsa = 0x00000003;

	__raw_writel(sscr0, ssp->mmio_base + SSCR0);
	__raw_writel(sscr1, ssp->mmio_base + SSCR1);
	__raw_writel(sspsp, ssp->mmio_base + SSPSP);
	__raw_writel(sstsa, ssp->mmio_base + SSTSA);
	__raw_writel(ssrsa, ssp->mmio_base + SSRSA);

	return 0;
}

static void saarb_ssp_hdmi_hifi_shutdown(struct snd_pcm_substream *substream)
{
	pr_debug("[audio]-->saarb_ssp_hdmi_hifi_shutdown: start.\n");
}

#if defined(CONFIG_SND_SOC_D1981)
static int saarb_d1981_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai_link *machine = rtd->dai;
	struct snd_soc_dai *cpu_dai = machine->cpu_dai;

	cpu_dai->playback.channels_min = 2;
	cpu_dai->playback.channels_max = 2;
	cpu_dai->capture.channels_min = 2;
	cpu_dai->capture.channels_max = 2;

	/* The rate is set according to the combination below:
	 * 1. codec:    sound/soc/codecs/pm860x-audio.c
	 * 2. ssp/abu:  sound/soc/pxa/pxa3xx-ssp.c / pxa95x-abu.c
	 * 3. dma:      sound/soc/pxa/pxa95x-pcm-ssp.c
	 * 4. platform: here!
	 * Allowing here 44.1 KHz rate will enforce working with this rate only!
	 */
	cpu_dai->playback.formats = SNDRV_PCM_FMTBIT_S16_LE;
	cpu_dai->capture.formats = SNDRV_PCM_FMTBIT_S16_LE;

	cpu_dai->playback.rates = SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000; //bsw
	cpu_dai->capture.rates = SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000; //bsw

	/* configure GPIOs for ABU/SSI */
	pr_debug("[audio]-->saarb_abu_pm860x_hifi_startup: configure GPIOs for ABU/SSI.\n");

		pxa95x_abu_mfp_init(true);
/*	enable_oscc_pout();*/
	return 0;
}

extern void d1981PathExternalMute(unsigned char mute);
static int saarb_d1981_prepare(struct snd_pcm_substream *substream)
{
	printk("[audio]-->saarb_d1981_prepare: start. \n");

	return 0;
}

static void saarb_d1981_shutdown(struct snd_pcm_substream *substream)
{
	printk("[audio]-->saarb_d1981_shutdown: start. \n");

}
#endif /* CONFIG_SND_SOC_D1981 */


/* machine stream operations */
static struct snd_soc_ops saarb_machine_ops[] = {
#if defined(CONFIG_SND_SOC_D1981)
    {
	    .startup = saarb_d1981_startup,
	    .prepare = saarb_d1981_prepare,
	    .shutdown = saarb_d1981_shutdown,
    },
#else
{
	.startup = saarb_abu_pm860x_hifi_startup,
	.prepare = saarb_abu_pm860x_hifi_prepare,
	.shutdown = saarb_abu_pm860x_hifi_shutdown,
},
#endif /* CONFIG_SND_SOC_D1981 */
{
	.startup = saarb_ssp_hdmi_hifi_startup,
	.prepare = saarb_ssp_hdmi_hifi_prepare,
	.shutdown = saarb_ssp_hdmi_hifi_shutdown,
},
};

/* saarb digital audio interface glue - connects codec <--> CPU
 ABU/SSI-I2S: ABU/SSI @MG1 <--> I2S@Sanremo codec
 */
#if defined(CONFIG_SND_SOC_D1981)
static struct snd_soc_dai_link saarb_d1981_dai_links[] = {
    {
        .name = "ABU/SSI-I2S",
	    .stream_name = "ABU/SSI and D1981: I2S HiFi",
	    .cpu_dai = &pxa95x_abu_dai[0],
	    .codec_dai = &d1981_audio_dai[0],
	    .ops = &saarb_machine_ops[0],
	    .init = saarb_d1981_init,
    },
};
#else
static struct snd_soc_dai_link saarb_abu_dai_links[] = {
{
	.name = "ABU/SSI-I2S",
	.stream_name = "ABU/SSI and Sanremo: I2S HiFi",
	.cpu_dai = &pxa95x_abu_dai[0],
	.codec_dai = &pm860x_audio_dai[0],
	.ops = &saarb_machine_ops[0],
	.init = saarb_pm860x_init,
},
};
#endif /* CONFIG_SND_SOC_D1981 */

/* saarb digital audio interface glue - connects codec <--> CPU
 BSSP2-HDMI: BSSP2 @MG1 <--> I2S@HDMI
 */
#if defined(CONFIG_SND_SOC_D1981)
static struct snd_soc_dai_link saarb_ssp_dai_links[] = {
{
	.name = "BSSP2-I2S",
	.stream_name = "BSSP2 and HDMI: I2S HiFi",
	.cpu_dai = &pxa3xx_ssp_dai[I2S_SSP_PORT -1],
	.codec_dai = &hdmi_audio_dai[0],
	.ops = &saarb_machine_ops[1],
	.init = saarb_d1981_init,
},
};
#else
static struct snd_soc_dai_link saarb_ssp_dai_links[] = {
{
	.name = "BSSP2-I2S",
	.stream_name = "BSSP2 and HDMI: I2S HiFi",
	.cpu_dai = &pxa3xx_ssp_dai[I2S_SSP_PORT -1],
	.codec_dai = &hdmi_audio_dai[0],
	.ops = &saarb_machine_ops[1],
	.init = saarb_pm860x_init,
},
};
#endif

/* saarb audio machine driver */
static struct snd_soc_card saarb_snd_soc_machine[] = {

#if defined(CONFIG_SND_SOC_D1981)
    {
	    .name = "saarb-d1981",
	    .platform = &pxa95x_abu_soc_platform,
	    .dai_link = saarb_d1981_dai_links,
	    .num_links = ARRAY_SIZE(saarb_d1981_dai_links),
    },
#else
{
	.name = "saarb-abu-pm860x",
	.platform = &pxa95x_abu_soc_platform,
	.dai_link = saarb_abu_dai_links,
	.num_links = ARRAY_SIZE(saarb_abu_dai_links),
},
#endif
{
	.name = "saarb-bssp2-hdmi",
	.platform = &pxa95x_soc_platform,
	.dai_link = saarb_ssp_dai_links,
	.num_links = ARRAY_SIZE(saarb_ssp_dai_links),
},
};

/* saarb audio subsystem */
static struct snd_soc_device saarb_snd_devdata[] = {
#if defined(CONFIG_SND_SOC_D1981)
{
	.card = &saarb_snd_soc_machine[0],
	.codec_dev = &soc_codec_dev_d1981_audio,
},
#else
{
	.card = &saarb_snd_soc_machine[0],
	.codec_dev = &soc_codec_dev_pm860x_audio,
},
#endif
{
	.card = &saarb_snd_soc_machine[1],
	.codec_dev = &soc_codec_dev_hdmi_audio,
},
};

static struct platform_device * saarb_snd_device[SAARB_SND_CARD_NUM];


static int __init saarb_init(void)
{
	int ret, i;

	pr_info("[audio]-->saarb_init: machine_arch_type= %d (MACH_TYPE_SAARB= %d).\n", machine_arch_type, MACH_TYPE_SAARB_MG1);

	if ((machine_arch_type != MACH_TYPE_TAVOREVB3) && (machine_arch_type != MACH_TYPE_SAARB_MG1) &&
			(machine_arch_type != MACH_TYPE_NEVOEVB3) && (machine_arch_type != MACH_TYPE_GFORCE_MG2) &&
			(machine_arch_type != MACH_TYPE_ALKON_MG2) && (machine_arch_type != MACH_TYPE_JETTA_MG2))
	{
		pr_err("[audio]-->saarb_init: machine is NOT 'MACH_TYPE_TAVOREVB3'or 'MACH_TYPE_SAARB' or 'MACH_TYPE_GFORCE_MG2' ==> ABORT!\n");
		return -ENODEV;
	}

	/* add two cards as platform devices */
	for (i = 0; i < SAARB_SND_CARD_NUM; i++)
	{
		saarb_snd_device[i] = platform_device_alloc("soc-audio", i);
		if (!saarb_snd_device[i])
		{
			pr_err("[audio]-->saarb_init: ERROR in platform device alloc!");
			return -ENOMEM;
		}

		platform_set_drvdata(saarb_snd_device[i], &saarb_snd_devdata[i]);
		saarb_snd_devdata[i].dev = &saarb_snd_device[i]->dev;
		ret = platform_device_add(saarb_snd_device[i]);

		if(ret)
		{
			pr_err("[audio]-->saarb_init: ERROR in platform device add!");
			platform_device_put(saarb_snd_device[i]);
		}
	}

	return ret;
}

static void __exit saarb_exit(void)
{
	int i;

	for (i = 0; i < SAARB_SND_CARD_NUM; i++)
	{
		platform_device_unregister(saarb_snd_device[i]);
	}
}

module_init(saarb_init);
module_exit(saarb_exit);

/* Module information */
MODULE_AUTHOR("mwwang@marvell.com");
MODULE_DESCRIPTION("ALSA SoC LEVANTE SAARB");
MODULE_LICENSE("GPL");
