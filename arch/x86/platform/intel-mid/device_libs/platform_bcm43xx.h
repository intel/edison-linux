/*
 * platform_bcm43xx.h: bcm43xx platform data header file
 *
 * (C) Copyright 2011 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */
#ifndef _PLATFORM_BCM43XX_H_
#define _PLATFORM_BCM43XX_H_

#define BCM43XX_SFI_GPIO_IRQ_NAME "WLAN-interrupt"
#define BCM43XX_SFI_GPIO_ENABLE_NAME "WLAN-enable"
#define ICDK_BOARD_REF_CLK 26000000
#define NCDK_BOARD_REF_CLK 38400000

extern void __init *bcm43xx_platform_data(void *info);
#endif
