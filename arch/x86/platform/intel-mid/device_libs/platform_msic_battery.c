/*
 * platform_msic_battery.c: MSIC battery platform data initilization file
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
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
#include <linux/mfd/intel_msic.h>
#include <asm/intel-mid.h>
#include "platform_msic.h"
#include "platform_msic_battery.h"

void __init *msic_battery_platform_data(void *info)
{
	return msic_generic_platform_data(info, INTEL_MSIC_BLOCK_BATTERY);
}
