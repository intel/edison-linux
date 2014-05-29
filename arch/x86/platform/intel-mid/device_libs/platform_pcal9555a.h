/*
 * platform_pcal9555a.h: pcal9555a platform data header file
 *
 * (C) Copyright 2014 Intel Corporation
 * Author: Dan O'Donovan <dan@emutex.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */
#ifndef _PLATFORM_PCAL9555A_H_
#define _PLATFORM_PCAL9555A_H_

/* we have multiple pcal9555a on the board ... */
#define PCAL9555A_NUM 4

extern void __init *pcal9555a_platform_data(void *info) __attribute__((weak));
#endif
