/*
 * platform_hsu.h: hsu platform data header file
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */
#ifndef _PLATFORM_HSU_H_
#define _PLATFORM_HSU_H_

#define HSU_BT_PORT "hsu_bt_port"
#define HSU_MODEM_PORT "hsu_modem_port"
#define HSU_GPS_PORT "hsu_gps_port"
#define HSU_DEBUG_PORT "hsu_debug_port"

enum hsu_core {
	hsu_pnw,
	hsu_clv,
	hsu_tgr,
};

struct hsu_port_pin_cfg {
	char *name;
	int id;
	int wake_gpio;
	int rx_gpio;
	int rx_alt;
	int tx_gpio;
	int tx_alt;
	int cts_gpio;
	int cts_alt;
	int rts_gpio;
	int rts_alt;
	struct device *dev;
	irq_handler_t wake_isr;
};

extern void *hsu_dev_platform_data(void *data) __attribute__((weak));
#endif
