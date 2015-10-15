/*
 * platform_mrfld_audio.c: MRFLD audio platform data initilization file
 *
 * (C) Copyright 2012 Intel Corporation
 * Author: Dharageswari R <dharageswari.r@intel.com>
 *	Vinod Koul <vinod.koul@intel.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/interrupt.h>
#include <linux/scatterlist.h>
#include <linux/init.h>
#include <linux/sfi.h>
#include <linux/platform_device.h>
#include <asm/intel-mid.h>
#include <linux/platform_data/intel_mid_remoteproc.h>
#include <asm/platform_sst_audio.h>
#include "platform_mrfld_audio.h"
#include "platform_msic.h"
#include "platform_wm8994.h"

static char* audio_codec = "dummy";
module_param(audio_codec, charp, S_IRUSR);
MODULE_PARM_DESC(audio_codec, "Hardware codec's name in use");

static struct mrfld_audio_platform_data mrfld_audio_pdata;

void *merfld_audio_platform_data(void *info)
{
	struct platform_device *pdev;
	int ret;

	pr_debug("in %s\n", __func__);

	ret = add_sst_platform_device();
	if (ret < 0) {
		pr_err("%s failed to sst_platform device\n", __func__);
		return NULL;
	}

	pdev = platform_device_alloc("hdmi-audio", -1);
	if (!pdev) {
		pr_err("failed to allocate hdmi-audio platform device\n");
		return NULL;
	}

	ret = platform_device_add(pdev);
	if (ret) {
		pr_err("failed to add hdmi-audio platform device\n");
		platform_device_put(pdev);
		return NULL;
	}

	/* request the gpios for audio */
	mrfld_audio_pdata.codec_gpio = get_gpio_by_name("audiocodec_int");
	mrfld_audio_pdata.codec_rst = get_gpio_by_name("audiocodec_rst");

	pdev = platform_device_alloc("mrfld_lm49453", -1);
	if (!pdev) {
		pr_err("failed to allocate mrfld_lm49453 platform device\n");
		return NULL;
	}

	ret = platform_device_add(pdev);
	if (ret) {
		pr_err("failed to add mrfld_lm49453 platform device\n");
		platform_device_put(pdev);
		return NULL;
	}
	if (platform_device_add_data(pdev, &mrfld_audio_pdata,
				     sizeof(mrfld_audio_pdata))) {
		pr_err("failed to add mrfld_lm49453 platform data\n");
		platform_device_put(pdev);
		return NULL;
	}

	register_rpmsg_service("rpmsg_msic_mrfld_audio", RPROC_SCU,
				RP_MSIC_MRFLD_AUDIO);

	return NULL;
}

void *mrfld_sst_audio_platform_data(void *info)
{
	struct platform_device *pdev;
	int ret;

	ret = add_sst_platform_device();
	if (ret < 0) {
		pr_err("%s failed to sst_platform device\n", __func__);
		return NULL;
	}

	if(!audio_codec || !strcmp(audio_codec, "dummy")) {
		pdev = platform_device_register_simple("merr_dpcm_dummy",
					0, NULL, 0);
		if (!pdev) {
			pr_err("failed to register merr_dpcm_dummy platform device\n");
			return NULL;
		}
	} else if (!strcmp(audio_codec, "wm8958")) {
		/* Register i2c audio codec wm8958 */
		wm8958_platform_data(NULL);

		pdev = platform_device_alloc("mrfld_wm8958", -1);
		if (!pdev) {
			pr_err("failed to allocate mrfld_wm8958 platform device\n");
			return NULL;
		}

		ret = platform_device_add(pdev);
		if (ret) {
			pr_err("failed to add mrfld_wm8958 platform device\n");
			platform_device_put(pdev);
			return NULL;
		}
		if (platform_device_add_data(pdev, &mrfld_audio_pdata,
						 sizeof(mrfld_audio_pdata))) {
			pr_err("failed to add mrfld_wm8958 platform data\n");
			platform_device_put(pdev);
			return NULL;
		}

		register_rpmsg_service("rpmsg_mrfld_wm8958_audio", RPROC_SCU,
					RP_MSIC_MRFLD_AUDIO);
	}
	/*
	 * To add a new codec, add a "else if" statement with
	 * its name and its specific implementation.
	 */
	else {
		pr_info("Codec %s is not implemented."
				"Dummy codec selected...\n", audio_codec);

		pdev = platform_device_register_simple("merr_dpcm_dummy",
							0, NULL, 0);
		if (!pdev) {
			pr_err("failed to register merr_dpcm_dummy platform device\n");
			return NULL;
		}
	}

	return NULL;
}
