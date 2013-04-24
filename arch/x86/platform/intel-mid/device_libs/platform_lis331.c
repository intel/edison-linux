/*
 * platform_lis331.c:  lis331 platform data initilization file
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/i2c.h>
#include <linux/gpio.h>
#include <asm/intel-mid.h>
#include "platform_lis331.h"

void __init *lis331dl_platform_data(void *info)
{
	static short intr2nd_pdata;
	struct i2c_board_info *i2c_info = info;
	int intr = get_gpio_by_name("accel_int");
	int intr2nd = get_gpio_by_name("accel_2");

	if (intr == -1 || intr2nd == -1)
		return NULL;

	i2c_info->irq = intr + INTEL_MID_IRQ_OFFSET;
	intr2nd_pdata = intr2nd + INTEL_MID_IRQ_OFFSET;

	return &intr2nd_pdata;
}
