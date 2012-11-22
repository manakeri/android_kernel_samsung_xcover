/*
 * linux/sound/soc/codecs/hdmi-audio.c
 * Base on linux/sound/soc/codecs/wm8753.c
 *
 * Copyright (C) 2007 Marvell International Ltd.
 * 			 Yael Sheli Chemla<yael.s.shemla@marvell.com>
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
#include <linux/device.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/initval.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#define ARRAY_AND_SIZE(x)	x, ARRAY_SIZE(x)


static struct snd_soc_codec *hdmi_codec;

/*
 * read hdmi audio register cache
 */
static unsigned int hdmi_audio_read(struct snd_soc_codec *codec, unsigned int reg)
{
	return 0;
}

/*
 * write to the hdmi audio register space
 */
static int hdmi_audio_write(struct snd_soc_codec *codec, unsigned int reg,
	unsigned int value)
{
	return 0;
}


static int hdmi_audio_hifi_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	return 0;
}

static int hdmi_audio_mute(struct snd_soc_dai *dai, int mute)
{
	return 0;
}


#define HDMI_AUDIO_HIFI_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |\
		                SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 |\
				SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000)

#define HDMI_AUDIO_HIFI_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE)


static struct snd_soc_dai_ops hdmi_dai_ops = {
	.hw_params = hdmi_audio_hifi_hw_params,
	.digital_mute = hdmi_audio_mute,
	.set_fmt = NULL,
	.set_clkdiv = NULL,
	.set_pll = NULL,
	.set_sysclk = NULL,
};

/*
 * HIFI DAI
 */
struct snd_soc_dai hdmi_audio_dai[]={
{
	.name = "hdmi audio HiFi",
	.id = 1,
	.playback = {
		.stream_name = "HiFi Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = HDMI_AUDIO_HIFI_RATES,
		.formats = HDMI_AUDIO_HIFI_FORMATS,
	},
	.ops = &hdmi_dai_ops,
},
};

static int hdmi_audio_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}


static int hdmi_audio_resume(struct platform_device *pdev)
{
	return 0;
}


static int hdmi_audio_init(struct snd_soc_device *socdev)
{
	struct snd_soc_codec *codec = socdev->card->codec;
	int ret = 0;

	pr_err("hdmi_audio_init :power & pll init\n" );

	codec->bias_level = SND_SOC_BIAS_STANDBY;

	/* register pcms */
	ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (ret < 0) {
		pr_err("hdmi audio: failed to create pcms\n");
		goto pcm_err;
	}

pcm_err:
	return ret;
}


static int hdmi_audio_probe(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);

	socdev->card->codec = hdmi_codec;

	hdmi_audio_init(socdev);

	return 0;
}

/* power down chip */
static int hdmi_audio_remove(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;

	kfree(codec);
	return 0;
}

struct snd_soc_codec_device soc_codec_dev_hdmi_audio = {
	.probe = 	hdmi_audio_probe,
	.remove = 	hdmi_audio_remove,
	.suspend = 	hdmi_audio_suspend,
	.resume =	hdmi_audio_resume,
};

EXPORT_SYMBOL_GPL(soc_codec_dev_hdmi_audio);

/* In normal case, here we should register a i2c client driver to controll codec through i2c
   and do these registrations in probe function. But hdmi has its own driver, so here just
   do some asoc registrations in moduele init function*/
static int __init hdmi_audio_modinit(void)
{
	struct snd_soc_codec *codec;
	int ret;

	codec = kzalloc(sizeof(struct snd_soc_codec), GFP_KERNEL);
	if (codec == NULL)
		return -ENOMEM;

	mutex_init(&codec->mutex);
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);

	codec->name = "hdmi audio";
	codec->owner = THIS_MODULE;
	codec->read = hdmi_audio_read;
	codec->write = hdmi_audio_write;
	codec->dai = hdmi_audio_dai;
	codec->num_dai = ARRAY_SIZE(hdmi_audio_dai);

	mutex_init(&codec->mutex);
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);

	hdmi_codec = codec;

	ret = snd_soc_register_codec(codec);
	if (ret != 0) {
		dev_err(codec->dev, "Failed to register codec: %d\n", ret);
		goto err;
	}

	ret = snd_soc_register_dais(ARRAY_AND_SIZE(hdmi_audio_dai));
	if (ret != 0) {
		dev_err(codec->dev, "Failed to register DAIs: %d\n", ret);
		goto err_codec;
	}

	return 0;

err_codec:
	snd_soc_unregister_codec(codec);
err:
	kfree(codec);
	return ret;
}
module_init(hdmi_audio_modinit);

static void __exit hdmi_audio_exit(void)
{
	snd_soc_unregister_dais(ARRAY_AND_SIZE(hdmi_audio_dai));
	snd_soc_unregister_codec(hdmi_codec);
}
module_exit(hdmi_audio_exit);

MODULE_DESCRIPTION("virtual ASoC hdmi audio driver");
MODULE_LICENSE("GPL");
