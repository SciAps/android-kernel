/*
 * pcm049.c  --  SoC audio for Phytec phyCORE-OMAP44xx
 *
 * Copyright (C) 2008 Phytec
 *
 * Contact: Steve Schefter <steve@scheftec.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include <linux/clk.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/platform_data/omap-abe-tlv320aic3x.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include <asm/mach-types.h>
#include <mach/hardware.h>
#include <linux/gpio.h>
#include <plat/mcbsp.h>

#include "mcbsp.h"
#include "omap-mcbsp.h"
#include "omap-pcm.h"
#include "omap-abe-priv.h"
#include "../codecs/tlv320aic3x.h"

enum {
	JACK_DISABLED,
	JACK_HP,
	JACK_HS,
	JACK_MIC,
};

#define CLK_NAME	"auxclk5_ck"

static int pcm049_spk_func = 1;
static int pcm049_jack_func = JACK_HP;

static void pcm049_ext_control(struct snd_soc_codec *codec)
{
	int hp = 0, line1l = 0;

	switch (pcm049_jack_func) {
	case JACK_HS:
		line1l = 1;
	case JACK_HP:
		hp = 1;
		break;
	case JACK_MIC:
		line1l = 1;
		break;
	}

	if (pcm049_spk_func)
		snd_soc_dapm_enable_pin(&codec->dapm, "Ext Spk");
	else
		snd_soc_dapm_disable_pin(&codec->dapm, "Ext Spk");

	if (hp)
		snd_soc_dapm_enable_pin(&codec->dapm, "Headphone Jack");
	else
		snd_soc_dapm_disable_pin(&codec->dapm, "Headphone Jack");

	snd_soc_dapm_sync(&codec->dapm);
}

static int pcm049_startup(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_card *card = rtd->card;
	struct omap_abe_tlv320aic3x_data *pdata = dev_get_platdata(card->dev);


	snd_pcm_hw_constraint_minmax(runtime,
				     SNDRV_PCM_HW_PARAM_CHANNELS, 2, 2);

	snd_soc_dapm_enable_pin(&codec->dapm, "Mic Jack");
	pcm049_ext_control(codec);
	return clk_enable(pdata->clk);
}

static void pcm049_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_card *card = rtd->card;
	struct omap_abe_tlv320aic3x_data *pdata = dev_get_platdata(card->dev);

	snd_soc_dapm_disable_pin(&codec->dapm, "Mic Jack");
	clk_disable(pdata->clk);
}

static int pcm049_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *card = rtd->card;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct omap_mcbsp *mcbsp = snd_soc_dai_get_drvdata(cpu_dai);
	unsigned long fclk_rate;
	struct clk *fclk;
	unsigned int channels;
	int err;

	/* Set codec DAI configuration */
	err = snd_soc_dai_set_fmt(codec_dai,
					 SND_SOC_DAIFMT_I2S |
					 SND_SOC_DAIFMT_NB_NF |
					 SND_SOC_DAIFMT_CBM_CFM);
	if (err < 0)
		return err;

	/* Set cpu DAI configuration */
	err = snd_soc_dai_set_fmt(cpu_dai,
				       SND_SOC_DAIFMT_I2S |
				       SND_SOC_DAIFMT_NB_NF |
				       SND_SOC_DAIFMT_CBM_CFM);
	if (err < 0)
		return err;

	fclk = clk_get(mcbsp->dev, CLK_NAME);
	if (IS_ERR(fclk)) {
		dev_err(card->dev, "can't get clk %s\n", CLK_NAME);
		return PTR_ERR(fclk);
	}
	fclk_rate = clk_get_rate(fclk);
	clk_put(fclk);

	if (params != NULL) {
		/* Configure McBSP internal buffer usage */
		/* this need to be done for playback and/or record */
		channels = params_channels(params);
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			omap_mcbsp_set_tx_threshold(mcbsp, channels);
		else
			omap_mcbsp_set_rx_threshold(mcbsp, channels);
	}

	/* Set the codec system clock for DAC and ADC */
	err = snd_soc_dai_set_sysclk(codec_dai, OMAP_MCBSP_SYSCLK_CLKS_FCLK,
					fclk_rate, SND_SOC_CLOCK_IN);

	return err;
}

static struct snd_soc_ops pcm049_ops = {
	.startup = pcm049_startup,
	.hw_params = pcm049_hw_params,
	.shutdown = pcm049_shutdown,
};

static int mcbsp_be_hw_params_fixup(struct snd_soc_pcm_runtime *rtd,
			struct snd_pcm_hw_params *params)
{
	struct snd_interval *channels = hw_param_interval(params,
					SNDRV_PCM_HW_PARAM_CHANNELS);

	channels->min = 2;
	snd_mask_set(&params->masks[SNDRV_PCM_HW_PARAM_FORMAT -
				    SNDRV_PCM_HW_PARAM_FIRST_MASK],
				    SNDRV_PCM_FORMAT_S16_LE);
        return 0;
}

static int pcm049_get_spk(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = pcm049_spk_func;

	return 0;
}

static int pcm049_set_spk(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec =  snd_kcontrol_chip(kcontrol);

	if (pcm049_spk_func == ucontrol->value.integer.value[0])
		return 0;

	pcm049_spk_func = ucontrol->value.integer.value[0];
	pcm049_ext_control(codec);

	return 1;
}

static int pcm049_get_jack(struct snd_kcontrol *kcontrol,
			 struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = pcm049_jack_func;

	return 0;
}

static int pcm049_set_jack(struct snd_kcontrol *kcontrol,
			 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec =  snd_kcontrol_chip(kcontrol);

	if (pcm049_jack_func == ucontrol->value.integer.value[0])
		return 0;

	pcm049_jack_func = ucontrol->value.integer.value[0];
	pcm049_ext_control(codec);

	return 1;
}

static int pcm049_spk_event(struct snd_soc_dapm_widget *w,
			  struct snd_kcontrol *k, int event)
{
	return 0;
}

static int pcm049_jack_event(struct snd_soc_dapm_widget *w,
			   struct snd_kcontrol *k, int event)
{
	return 0;
}

static const struct snd_soc_dapm_widget aic33_dapm_widgets[] = {
	SND_SOC_DAPM_SPK("Ext Spk", pcm049_spk_event),
	SND_SOC_DAPM_HP("Headphone Jack", pcm049_jack_event),
	SND_SOC_DAPM_MIC("Mic Jack", NULL),
};

static const struct snd_soc_dapm_route audio_map[] = {
	{"Headphone Jack", NULL, "HPLOUT"},
	{"Headphone Jack", NULL, "HPROUT"},

	{"Ext Spk", NULL, "LLOUT"},
	{"Ext Spk", NULL, "RLOUT"},

	{"MIC3L", NULL, "Mic Bias 2V"},
	{"MIC3R", NULL, "Mic Bias 2V"},
	{"Mic Bias 2V", NULL, "Mic Jack"},
};

static const char *spk_function[] = {"Off", "On"};
static const char *jack_function[] = {"Off", "Headphone", "Headset", "Mic"};
static const struct soc_enum pcm049_enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(spk_function), spk_function),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(jack_function), jack_function),
};

static const struct snd_kcontrol_new aic33_pcm049_controls[] = {
	SOC_ENUM_EXT("Speaker Function", pcm049_enum[0],
		     pcm049_get_spk, pcm049_set_spk),
	SOC_ENUM_EXT("Jack Function", pcm049_enum[1],
		     pcm049_get_jack, pcm049_set_jack),
};

static int pcm049_aic33_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	struct snd_soc_card *card = codec->card;
	int err;

	/* Not connected */
	snd_soc_dapm_nc_pin(dapm, "HPLCOM");
	snd_soc_dapm_nc_pin(dapm, "HPRCOM");
	snd_soc_dapm_nc_pin(dapm, "LINE2L");
	snd_soc_dapm_nc_pin(dapm, "LINE2R");

	/* allow audio paths from the audio modem to run during suspend */
	snd_soc_dapm_ignore_suspend(&card->dapm, "Ext Spk");
	snd_soc_dapm_ignore_suspend(&card->dapm, "Mic Jack");
	snd_soc_dapm_ignore_suspend(&card->dapm, "Headphone Jack");

	/* Add board specific controls */
	err = snd_soc_add_codec_controls(codec, aic33_pcm049_controls,
				ARRAY_SIZE(aic33_pcm049_controls));
	if (err < 0)
		return err;

	return 0;
}

/* Digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link pcm049_dai[] = {
	/* Frontend */
	{
		.name = "TLV320AIC33",
		.stream_name = "Multimedia",
		.cpu_dai_name = "omap-mcbsp.3",
		.platform_name = "omap-pcm-audio",
		.codec_name = "tlv320aic3x-codec.4-0018",
		.codec_dai_name = "tlv320aic3x-hifi",
		.init = pcm049_aic33_init,
		.ops = &pcm049_ops,
	},
	/* Backend */
	{
		.name = OMAP_ABE_BE_MM_EXT0_DL,
		.stream_name = "AIC33",
		.cpu_dai_name = "omap-mcbsp.3",
		.platform_name = "aess",
		.codec_name = "tlv320aic3x-codec.4-0018",
		.codec_dai_name = "tlv320aic3x-hifi",
		.no_pcm = 1,
		.be_hw_params_fixup = mcbsp_be_hw_params_fixup,
		.ops = &pcm049_ops,
		.be_id = OMAP_ABE_DAI_MM_FM,
	}
};

/* Audio machine driver */
static struct snd_soc_card omap_abe_card = {
	.owner = THIS_MODULE,
	.dapm_widgets = aic33_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(aic33_dapm_widgets),
	.dapm_routes = audio_map,
	.num_dapm_routes = ARRAY_SIZE(audio_map),
};

static __devinit int omap_abe_probe(struct platform_device *pdev)
{
	struct omap_abe_tlv320aic3x_data *pdata = dev_get_platdata(&pdev->dev);
	struct snd_soc_card *card = &omap_abe_card;
	int ret;

	card->dev = &pdev->dev;

	if (!pdata) {
		dev_err(&pdev->dev, "Missing pdata\n");
		return -ENODEV;
	}

	if (pdata->card_name) {
		card->name = pdata->card_name;
	} else {
		dev_err(&pdev->dev, "Card name is not provided\n");
		return -ENODEV;
	}

	if (!pdata->mclk_freq) {
		dev_err(&pdev->dev, "MCLK frequency missing\n");
		return -ENODEV;
	}

	pdata->clk = clk_get(&pdev->dev, CLK_NAME);
	if (IS_ERR(pdata->clk)) {
		dev_err(&pdev->dev, "Could not get clock %s\n", CLK_NAME);
		return -ENODEV;
	}
	clk_set_rate(pdata->clk, pdata->mclk_freq);

	card->dai_link = pcm049_dai;
	card->num_links = ARRAY_SIZE(pcm049_dai);

	ret = snd_soc_register_card(card);
	if (ret)
		dev_err(&pdev->dev, "snd_soc_register_card() failed: %d\n", ret);

	return ret;
}

static int __devexit omap_abe_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	struct omap_abe_tlv320aic3x_data *pdata = dev_get_platdata(&pdev->dev);

	clk_put(pdata->clk);
	snd_soc_unregister_card(card);
	return 0;
}

static void omap_abe_shutdown(struct platform_device *pdev)
{
	snd_soc_poweroff(&pdev->dev);
}

static struct platform_driver omap_abe_driver = {
	.driver = {
		.name = "omap-abe-tlv320aic3x",
		.owner = THIS_MODULE,
		.pm = &snd_soc_pm_ops,
	},
	.probe = omap_abe_probe,
	.remove = __devexit_p(omap_abe_remove),
	.shutdown = omap_abe_shutdown,
};

module_platform_driver(omap_abe_driver);

MODULE_AUTHOR("Steve Schefter <steve@scheftech.com>");
MODULE_DESCRIPTION("ALSA SoC phyCORE-OMAP44xx");
MODULE_LICENSE("GPL");
