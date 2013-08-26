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
#include <asm/intel-mid.h>
#include <asm/intel_basincove_gpadc.h>
#include <asm/intel_mid_remoteproc.h>

#include "platform_bcove_adc.h"

/* SRAM address where the GPADC interrupt register is cached */
#define GPADC_SRAM_INTR_ADDR	0xfffff615

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
