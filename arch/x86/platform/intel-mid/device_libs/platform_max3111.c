/*
 * platform_max3111.c: max3111 platform data initilization file
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
#include <linux/spi/spi.h>
#include <asm/intel-mid.h>
#include "platform_max3111.h"

void __init *max3111_platform_data(void *info)
{
	struct spi_board_info *spi_info = info;
	int intr = get_gpio_by_name("max3111_int");

	spi_info->mode = SPI_MODE_0;
	if (intr == -1)
		return NULL;
	spi_info->irq = intr + INTEL_MID_IRQ_OFFSET;
	return NULL;
}
