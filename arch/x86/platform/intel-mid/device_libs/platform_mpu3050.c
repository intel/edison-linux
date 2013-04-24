/*
 * platform_mpu3050.c: mpu3050 platform data initilization file
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/gpio.h>
#include <linux/i2c.h>
#include <asm/intel-mid.h>
#include "platform_mpu3050.h"

void *mpu3050_platform_data(void *info)
{
	struct i2c_board_info *i2c_info = info;
	int intr = get_gpio_by_name("mpu3050_int");

	if (intr == -1)
		return NULL;

	i2c_info->irq = intr + INTEL_MID_IRQ_OFFSET;
	return NULL;
}
