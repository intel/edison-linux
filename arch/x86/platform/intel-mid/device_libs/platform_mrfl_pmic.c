/*
 * platform_mrfl_pmic.c: Platform data for Merrifield PMIC driver
 *
 * (C) Copyright 2012 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/scatterlist.h>
#include <linux/init.h>
#include <linux/sfi.h>
#include <asm/intel-mid.h>
#include <asm/pmic_pdata.h>
#include <asm/intel_mid_remoteproc.h>
#include <linux/power/bq24261_charger.h>

#include "platform_ipc.h"
#include "platform_mrfl_pmic.h"

void __init *mrfl_pmic_ccsm_platform_data(void *info)
{
	struct sfi_device_table_entry *entry = info;
	static struct pmic_platform_data pmic_pdata;
	struct platform_device *pdev = NULL;
	int ret;

	pdev = platform_device_alloc(entry->name, -1);
	if (!pdev) {
		pr_err("Out of memory for SFI platform dev %s\n", entry->name);
		goto out;
	}
	pdev->dev.platform_data = &pmic_pdata;
	ret = platform_device_add(pdev);
	if (ret) {
		pr_err("Failed to add adc platform device\n");
		platform_device_put(pdev);
		goto out;
	}
	install_irq_resource(pdev, entry->irq);
#ifdef CONFIG_BQ24261_CHARGER
	pmic_pdata.cc_to_reg = bq24261_cc_to_reg;
	pmic_pdata.cv_to_reg = bq24261_cv_to_reg;
#endif
	register_rpmsg_service("rpmsg_pmic_ccsm", RPROC_SCU,
				RP_PMIC_CCSM);
out:
	return &pmic_pdata;
}

