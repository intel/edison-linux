/*
 * ADS7955 SPI ADC driver
 *
 * (C) Copyright 2014 Intel Corporation
 * Author: Dave Hunt <dave.hunt@emutex.com>
 *
 * Licensed under the GPL-2.
 */

#ifndef __LINUX_PLATFORM_DATA_TI_ADS7955_H__
#define __LINUX_PLATFORM_DATA_TI_ADS7955_H__

/**
 * struct ads7955_platform_data - Platform data for the ads7955 ADC driver
 * @ext_ref: Whether to use an external reference voltage.
 **/
struct ads7955_platform_data {
	bool ext_ref;
};

#endif /* __LINUX_PLATFORM_DATA_TI_ADS7955_H__ */
