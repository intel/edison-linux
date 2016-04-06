/*
 * platform_pmic_charger.c: pmic charger platform data initilization file
 *
 * (C) Copyright 2013 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <asm/intel-mid.h>

static const struct devs_id pmic_charger_dev_id __initconst = {
	.name = "pmic_charger",
	.type = SFI_DEV_TYPE_IPC,
	.delay = 1,
};

sfi_device(pmic_charger_dev_id);
