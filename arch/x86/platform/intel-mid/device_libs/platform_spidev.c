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
#include <linux/gpio.h>
#include <linux/lnw_gpio.h>
#include "platform_spidev.h"

static void tng_ssp_spi_cs_control(u32 command);
static void tng_ssp_spi_platform_pinmux(void);

static int tng_ssp_spi2_FS_gpio = 111;

static struct intel_mid_ssp_spi_chip chip = {
	.burst_size = DFLT_FIFO_BURST_SIZE,
	.timeout = DFLT_TIMEOUT_VAL,
	/* SPI DMA is currently usable on Tangier */
	.dma_enabled = true,
	.cs_control = tng_ssp_spi_cs_control,
	.platform_pinmux = tng_ssp_spi_platform_pinmux,
};

static void tng_ssp_spi_cs_control(u32 command)
{
	gpio_set_value(tng_ssp_spi2_FS_gpio, (command != 0) ? 1 : 0);
}

static void tng_ssp_spi_platform_pinmux(void)
{
	int err;
	int saved_muxing;
	/* Request Chip Select gpios */
	saved_muxing = gpio_get_alt(tng_ssp_spi2_FS_gpio);

	lnw_gpio_set_alt(tng_ssp_spi2_FS_gpio, LNW_GPIO);
	err = gpio_request_one(tng_ssp_spi2_FS_gpio,
			GPIOF_DIR_OUT|GPIOF_INIT_HIGH, "Arduino Shield SS");
	if (err) {
		pr_err("%s: unable to get Chip Select GPIO,\
				fallback to legacy CS mode \n", __func__);
		lnw_gpio_set_alt(tng_ssp_spi2_FS_gpio, saved_muxing);
		chip.cs_control = NULL;
		chip.platform_pinmux = NULL;
	}
}

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
