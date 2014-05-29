/*
 * platform_spidev.h: spidev platform data header file
 *
 * (C) Copyright 2014 Intel Corporation
 * Author: Dan O'Donovan <dan@emutex.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */
#ifndef _PLATFORM_SPIDEV_H_
#define _PLATFORM_SPIDEV_H_

/* REVERT ME workaround[MRFL] for invalid bus number in IAFW .25 */
#define FORCE_SPI_BUS_NUM	5
#define FORCE_CHIP_SELECT	1

extern void *spidev_platform_data(void *info) __attribute__((weak));
#endif
