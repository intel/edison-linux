/*
 * platform_mrfl_thermal.c: Platform data initilization file for
 *			Intel Merrifield Platform thermal driver
 *
 * (C) Copyright 2013 Intel Corporation
 * Author: Durgadoss R <durgadoss.r@intel.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/init.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/mfd/intel_msic.h>
#include <linux/platform_device.h>
#include <asm/intel_mid_thermal.h>
#include <asm/intel-mid.h>
#include <asm/intel_mid_remoteproc.h>
#include "platform_mrfl_thermal.h"

/* 'enum' of Thermal ADC channels */
enum thermal_adc_channels { SYS0, SYS1, SYS2, PMIC_DIE };

static int linear_temp_correlation(void *info, long temp, long *res)
{
	struct intel_mid_thermal_sensor *sensor = info;

	*res = ((temp * sensor->slope) / 1000) + sensor->intercept;

	return 0;
}

/*
 * Naming convention:
 * skin0 -> front skin,
 * skin1--> back skin
 */

static struct intel_mid_thermal_sensor mrfl_sensors[] = {
	{
		.name = SKIN0_NAME,
		.index = SYS2,
		.slope = 969,
		.intercept = -3741,
		.temp_correlation = linear_temp_correlation,
		.direct = false,
	},
	{
		.name = SKIN1_NAME,
		.index = SYS0,
		.slope = 966,
		.intercept = -2052,
		.temp_correlation = linear_temp_correlation,
		.direct = false,
	},
	{
		.name = MSIC_DIE_NAME,
		.index = PMIC_DIE,
		.slope = 1000,
		.intercept = 0,
		.temp_correlation = linear_temp_correlation,
		.direct = true,
	},
};

/* Bodegabay - PRh thermal sensor list */
static struct intel_mid_thermal_sensor bdgb_sensors[] = {
	{
		.name = SKIN0_NAME,
		.index = SYS0,
		.slope = 410,
		.intercept = 16808,
		.temp_correlation = linear_temp_correlation,
		.direct = false,
	},
	{
		.name = SKIN1_NAME,
		.index = SYS0,
		.slope = 665,
		.intercept = 8375,
		.temp_correlation = linear_temp_correlation,
		.direct = false,
	},
	{
		.name = MSIC_DIE_NAME,
		.index = PMIC_DIE,
		.slope = 1000,
		.intercept = 0,
		.temp_correlation = linear_temp_correlation,
		.direct = true,
	},
};

static struct intel_mid_thermal_platform_data pdata[] = {
	[mrfl_thermal] = {
		.num_sensors = 3,
		.sensors = mrfl_sensors,
	},
	[bdgb_thermal] = {
		.num_sensors = 3,
		.sensors = bdgb_sensors,
	},
};

void __init *mrfl_thermal_platform_data(void *info)
{
	struct platform_device *pdev;
	struct sfi_device_table_entry *entry = info;

	pdev = platform_device_alloc(MRFL_THERM_DEV_NAME, -1);
	if (!pdev) {
		pr_err("out of memory for SFI platform dev %s\n",
			MRFL_THERM_DEV_NAME);
		return NULL;
	}

	if (platform_device_add(pdev)) {
		pr_err("failed to add thermal platform device\n");
		platform_device_put(pdev);
		return NULL;
	}

	pdev->dev.platform_data = &pdata[mrfl_thermal];

	install_irq_resource(pdev, entry->irq);
	register_rpmsg_service("rpmsg_mrfl_thermal", RPROC_SCU,
				RP_BCOVE_THERMAL);

	return 0;
}
