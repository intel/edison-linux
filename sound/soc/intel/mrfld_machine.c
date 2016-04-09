/*
 *  mrfld_machine.c - ASoc Machine driver for Intel Merrifield MID platform
 *
 *  Copyright (C) 2010 Intel Corp
 *  Author: Vinod Koul <vinod.koul@intel.com>
 *  Author: Harsha Priya <priya.harsha@intel.com>
 *  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/init.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/module.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>


static const struct snd_soc_dapm_route mrfld_map[] = {
	{ "Dummy Playback", NULL, "codec_out0"  },
	{ "Dummy Playback", NULL, "codec_out1"  },
	{ "codec_in0", NULL, "Dummy Capture" },
	{ "codec_in1", NULL, "Dummy Capture" },
};

static unsigned int rates_48000[] = {
	48000,
};

static struct snd_pcm_hw_constraint_list constraints_48000 = {
	.count = ARRAY_SIZE(rates_48000),
	.list  = rates_48000,
};

static int mrfld_dummy_startup(struct snd_pcm_substream *substream)
{
	return snd_pcm_hw_constraint_list(substream->runtime, 0,
			SNDRV_PCM_HW_PARAM_RATE,
			&constraints_48000);
}

static struct snd_soc_ops mrfld_dummy_ops = {
		.startup = mrfld_dummy_startup,
};

static int mrfld_codec_fixup(struct snd_soc_pcm_runtime *rtd,
		struct snd_pcm_hw_params *params)
{
	struct snd_interval *rate = hw_param_interval(params,
			SNDRV_PCM_HW_PARAM_RATE);
	struct snd_interval *channels = hw_param_interval(params,
			SNDRV_PCM_HW_PARAM_CHANNELS);

	pr_debug("Invoked %s for dailink %s\n", __func__, rtd->dai_link->name);

	/* The DSP will convert the FE rate to 48k, stereo, 24bits */
	rate->min = rate->max = 48000;
	channels->min = channels->max = 2;

	/* set SSP2 to 24-bit */
	snd_mask_set(&params->masks[SNDRV_PCM_HW_PARAM_FORMAT -
				SNDRV_PCM_HW_PARAM_FIRST_MASK],
				SNDRV_PCM_FORMAT_S24_LE);
	return 0;
}

static struct snd_soc_dai_link mrfld_msic_dailink[] = {
	{
		.name = "Media Audio Port",
		.stream_name = "Edison Audio",
		.cpu_dai_name = "media-cpu-dai",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "sst-mrfld-platform",
		.ignore_suspend = 1,
		.dynamic = 1,
		.ops = &mrfld_dummy_ops,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
	},
	/* back ends */
	{
		.name = "SSP2-Codec",
		.be_id = 1,
		.cpu_dai_name = "ssp2-port",
		.platform_name = "sst-mrfld-platform",
		.no_pcm = 1,
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.be_hw_params_fixup = mrfld_codec_fixup,
		.ignore_suspend = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		},
	{
		.name = "SSP1-BT",
		.be_id = 2,
		.cpu_dai_name = "snd-soc-dummy-dai",
		.platform_name = "sst-mrfld-platform",
		.no_pcm = 1,
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.ignore_suspend = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
	},
};

/* SoC card */
static struct snd_soc_card snd_soc_card_mrfld = {
	.name = "dummy-audio",
	.owner = THIS_MODULE,
	.dai_link = mrfld_msic_dailink,
	.num_links = ARRAY_SIZE(mrfld_msic_dailink),
	.dapm_routes = mrfld_map,
	.num_dapm_routes = ARRAY_SIZE(mrfld_map),
};

static int snd_mrfld_mc_probe(struct platform_device *pdev)
{
	int ret_val = 0;
	struct mrfld_mc_private *mc_drv_ctx;

	pr_debug("snd_mrfld_mc_probe called\n");
	/* register the soc card */
	snd_soc_card_mrfld.dev = &pdev->dev;
	ret_val = devm_snd_soc_register_card(&pdev->dev, &snd_soc_card_mrfld);
	if (ret_val) {
		pr_debug("snd_soc_register_card failed %d\n", ret_val);
		return ret_val;
	}
	platform_set_drvdata(pdev, mc_drv_ctx);
	pr_debug("successfully exited probe\n");
	return 0;
}

static struct platform_driver snd_mrfld_mc_driver = {
	.driver = {
		.name = "mrfld_mc_dummy",
		.pm = &snd_soc_pm_ops,
	},
	.probe = snd_mrfld_mc_probe,
};

module_platform_driver(snd_mrfld_mc_driver);

MODULE_DESCRIPTION("ASoC Intel(R) MID Merrifield Machine driver");
MODULE_AUTHOR("Vinod Koul <vinod.koul@intel.com>");
MODULE_AUTHOR("Harsha Priya <priya.harsha@intel.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:mrfld_mc_dummy");
