/*
 * platform_spidev.c: spidev platform data initilization file
 *
 * (C) Copyright 2014 Intel Corporation
 * Author: Dan O'Donovan <dan@emutex.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/spi/spi.h>
#include <linux/spi/intel_mid_ssp_spi.h>
#include <asm/intel-mid.h>
#include "platform_spidev.h"

static struct intel_mid_ssp_spi_chip chip = {
	.burst_size = DFLT_FIFO_BURST_SIZE,
	.timeout = DFLT_TIMEOUT_VAL,
	/* SPI DMA is currently not usable on Tangier */
	.dma_enabled = false,
};

void __init *spidev_platform_data(void *info)
{
	struct spi_board_info *spi_info = info;

	if (!spi_info) {
		pr_err("%s: invalid info pointer\n", __func__);
		return NULL;
	}

	spi_info->mode = SPI_MODE_0;

	spi_info->controller_data = &chip;
	spi_info->bus_num = FORCE_SPI_BUS_NUM;

	return NULL;
}
