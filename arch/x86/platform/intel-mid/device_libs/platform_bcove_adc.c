/*
 * platform_bcove_adc.c: Platform data for Merrifield Basincove GPADC driver
 *
 * (C) Copyright 2012 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/sfi.h>
#include <linux/iio/machine.h>
#include <asm/intel-mid.h>
#include <asm/intel_basincove_gpadc.h>
#include <asm/intel_mid_remoteproc.h>

#include "platform_bcove_adc.h"

/* SRAM address where the GPADC interrupt register is cached */
#define GPADC_SRAM_INTR_ADDR	0xfffff615

#define MSIC_ADC_MAP(_adc_channel_label,			\
		     _consumer_dev_name,                        \
		     _consumer_channel)                         \
	{                                                       \
		.adc_channel_label = _adc_channel_label,        \
		.consumer_dev_name = _consumer_dev_name,        \
		.consumer_channel = _consumer_channel,          \
	}

struct iio_map iio_maps[] = {
	MSIC_ADC_MAP("CH0", "VIBAT", "VBAT"),
	MSIC_ADC_MAP("CH1", "BATID", "BATID"),
	MSIC_ADC_MAP("CH2", "VIBAT", "IBAT"),
	MSIC_ADC_MAP("CH3", "PMICTEMP", "PMICTEMP"),
	MSIC_ADC_MAP("CH4", "BATTEMP", "BATTEMP0"),
	MSIC_ADC_MAP("CH5", "BATTEMP", "BATTEMP1"),
	MSIC_ADC_MAP("CH6", "SYSTEMP", "SYSTEMP0"),
	MSIC_ADC_MAP("CH7", "SYSTEMP", "SYSTEMP1"),
	MSIC_ADC_MAP("CH8", "SYSTEMP", "SYSTEMP2"),
	MSIC_ADC_MAP("CH6", "bcove_thrm", "SYSTEMP0"),
	MSIC_ADC_MAP("CH7", "bcove_thrm", "SYSTEMP1"),
	MSIC_ADC_MAP("CH8", "bcove_thrm", "SYSTEMP2"),
	MSIC_ADC_MAP("CH3", "bcove_thrm", "PMICTEMP"),
	{ },
};

void __init *bcove_adc_platform_data(void *info)
{
	struct platform_device *pdev = NULL;
	struct sfi_device_table_entry *entry = info;
	static struct intel_basincove_gpadc_platform_data bcove_adc_pdata;
	int ret;

	pdev = platform_device_alloc(BCOVE_ADC_DEV_NAME, -1);

	if (!pdev) {
		pr_err("out of memory for SFI platform dev %s\n",
					BCOVE_ADC_DEV_NAME);
		goto out;
	}

	bcove_adc_pdata.intr = GPADC_SRAM_INTR_ADDR;
	bcove_adc_pdata.iio_maps = iio_maps;

	pdev->dev.platform_data = &bcove_adc_pdata;

	ret = platform_device_add(pdev);
	if (ret) {
		pr_err("failed to add bcove adc platform device\n");
		platform_device_put(pdev);
		goto out;
	}

	install_irq_resource(pdev, entry->irq);

	register_rpmsg_service("rpmsg_bcove_adc", RPROC_SCU,
				RP_BCOVE_ADC);
out:
	return &bcove_adc_pdata;
}
