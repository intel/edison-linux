/*
 * platform_hsu.c: hsu platform data initilization file
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/pm_runtime.h>
#include <linux/lnw_gpio.h>
#include <linux/gpio.h>
#include <asm/setup.h>
#include <asm/intel_mid_hsu.h>
#include <asm/intel-mid.h>
#include "platform_hsu.h"

static struct hsu_port_pin_cfg hsu_port_pin_cfgs[][hsu_port_max] = {
	[hsu_pnw] = {
		[hsu_port0] = {
			.id = 0,
			.name = HSU_BT_PORT,
			.wake_gpio = 13,
			.rx_gpio = 96+26,
			.rx_alt = 1,
			.tx_gpio = 96+27,
			.tx_alt = 1,
			.cts_gpio = 96+28,
			.cts_alt = 1,
			.rts_gpio = 96+29,
			.rts_alt = 1,
		},
		[hsu_port1] = {
			.id = 1,
			.name = HSU_MODEM_PORT,
			.wake_gpio = 64,
			.rx_gpio = 64,
			.rx_alt = 1,
			.tx_gpio = 65,
			.tx_alt = 1,
			.cts_gpio = 68,
			.cts_alt = 1,
			.rts_gpio = 66,
			.rts_alt = 2,
		},
		[hsu_port2] = {
			.id = 2,
			.name = HSU_GPS_PORT,
		},
		[hsu_port_share] = {
			.id = 1,
			.name = HSU_DEBUG_PORT,
			.wake_gpio = 96+30,
			.rx_gpio = 96+30,
			.rx_alt = 1,
			.tx_gpio = 96+31,
			.tx_alt = 1,
		},
	},
	[hsu_clv] = {
		[hsu_port0] = {
			.id = 0,
			.name = HSU_BT_PORT,
			.wake_gpio = 42,
			.rx_gpio = 96+26,
			.rx_alt = 1,
			.tx_gpio = 96+27,
			.tx_alt = 1,
			.cts_gpio = 96+28,
			.cts_alt = 1,
			.rts_gpio = 96+29,
			.rts_alt = 1,
		},
		[hsu_port1] = {
			.id = 1,
			.name = HSU_MODEM_PORT,
			.wake_gpio = 64,
			.rx_gpio = 64,
			.rx_alt = 1,
			.tx_gpio = 65,
			.tx_alt = 1,
			.cts_gpio = 68,
			.cts_alt = 1,
			.rts_gpio = 66,
			.rts_alt = 2,
		},
		[hsu_port2] = {
			.id = 2,
			.name = HSU_DEBUG_PORT,
			.wake_gpio = 67,
			.rx_gpio = 67,
			.rx_alt = 1,
		},
		[hsu_port_share] = {
			.id = 1,
			.name = HSU_GPS_PORT,
			.wake_gpio = 96+30,
			.rx_gpio = 96+30,
			.rx_alt = 1,
			.tx_gpio = 96+31,
			.tx_alt = 1,
			.cts_gpio = 96+33,
			.cts_alt = 1,
			.rts_gpio = 96+32,
			.rts_alt = 2,
		},
	},
};

static struct hsu_port_cfg hsu_port_cfgs[][hsu_port_max] = {
	[hsu_pnw] = {
		[hsu_port0] = {
			.type = bt_port,
			.index = 0,
			.name = HSU_BT_PORT,
			.idle = 20,
			.hw_init = intel_mid_hsu_init,
			.hw_set_alt = intel_mid_hsu_switch,
			.hw_set_rts = intel_mid_hsu_rts,
			.hw_suspend = intel_mid_hsu_suspend,
			.hw_resume = intel_mid_hsu_resume,
		},
		[hsu_port1] = {
			.type = modem_port,
			.index = 1,
			.name = HSU_MODEM_PORT,
			.idle = 100,
			.hw_init = intel_mid_hsu_init,
			.hw_set_alt = intel_mid_hsu_switch,
			.hw_set_rts = intel_mid_hsu_rts,
			.hw_suspend = intel_mid_hsu_suspend,
			.hw_resume = intel_mid_hsu_resume,
			.has_alt = 1,
			.alt = hsu_port_share,
			.force_suspend = 0,
		},
		[hsu_port2] = {
			.type = gps_port,
			.index = 2,
			.name = HSU_GPS_PORT,
			.idle = 100,
			.hw_init = intel_mid_hsu_init,
			.hw_set_alt = intel_mid_hsu_switch,
			.hw_set_rts = intel_mid_hsu_rts,
			.hw_suspend = intel_mid_hsu_suspend,
			.hw_resume = intel_mid_hsu_resume,
		},
		[hsu_port_share] = {
			.type = debug_port,
			.index = 3,
			.name = HSU_DEBUG_PORT,
			.idle = 2000,
			.hw_init = intel_mid_hsu_init,
			.hw_set_alt = intel_mid_hsu_switch,
			.hw_set_rts = intel_mid_hsu_rts,
			.hw_suspend = intel_mid_hsu_suspend,
			.hw_resume = intel_mid_hsu_resume,
			.has_alt = 1,
			.alt = hsu_port1,
			.force_suspend = 1,
		},
	},
	[hsu_clv] = {
		[hsu_port0] = {
			.type = bt_port,
			.index = 0,
			.name = HSU_BT_PORT,
			.idle = 20,
			.hw_init = intel_mid_hsu_init,
			.hw_set_alt = intel_mid_hsu_switch,
			.hw_set_rts = intel_mid_hsu_rts,
			.hw_suspend = intel_mid_hsu_suspend,
			.hw_resume = intel_mid_hsu_resume,
		},
		[hsu_port1] = {
			.type = modem_port,
			.index = 1,
			.name = HSU_MODEM_PORT,
			.idle = 100,
			.hw_init = intel_mid_hsu_init,
			.hw_set_alt = intel_mid_hsu_switch,
			.hw_set_rts = intel_mid_hsu_rts,
			.hw_suspend = intel_mid_hsu_suspend,
			.hw_resume = intel_mid_hsu_resume,
			.has_alt = 1,
			.alt = hsu_port_share,
			.force_suspend = 0,
		},
		[hsu_port2] = {
			.type = debug_port,
			.index = 2,
			.name = HSU_DEBUG_PORT,
			.idle = 2000,
			.hw_init = intel_mid_hsu_init,
			.hw_set_alt = intel_mid_hsu_switch,
			.hw_set_rts = intel_mid_hsu_rts,
			.hw_suspend = intel_mid_hsu_suspend,
			.hw_resume = intel_mid_hsu_resume,
		},
		[hsu_port_share] = {
			.type = gps_port,
			.index = 3,
			.name = HSU_GPS_PORT,
			.idle = 100,
			.hw_init = intel_mid_hsu_init,
			.hw_set_alt = intel_mid_hsu_switch,
			.hw_set_rts = intel_mid_hsu_rts,
			.hw_suspend = intel_mid_hsu_suspend,
			.hw_resume = intel_mid_hsu_resume,
			.has_alt = 1,
			.alt = hsu_port1,
			.force_suspend = 1,
		},
	},
};

struct hsu_port_pin_cfg *hsu_port_gpio_mux;

static void hsu_port_enable(int port)
{
	struct hsu_port_pin_cfg *info = hsu_port_gpio_mux + port;

	if (info->rx_gpio) {
		lnw_gpio_set_alt(info->rx_gpio, info->rx_alt);
		gpio_direction_input(info->rx_gpio);
	}
	if (info->tx_gpio) {
		gpio_direction_output(info->tx_gpio, 0);
		lnw_gpio_set_alt(info->tx_gpio, info->tx_alt);

	}
	if (info->cts_gpio) {
		lnw_gpio_set_alt(info->cts_gpio, info->cts_alt);
		gpio_direction_input(info->cts_gpio);
	}
	if (info->rts_gpio) {
		gpio_direction_output(info->rts_gpio, 0);
		lnw_gpio_set_alt(info->rts_gpio, info->rts_alt);
	}
}

static void hsu_port_disable(int port)
{
	struct hsu_port_pin_cfg *info = hsu_port_gpio_mux + port;

	if (info->rx_gpio) {
		lnw_gpio_set_alt(info->rx_gpio, LNW_GPIO);
		gpio_direction_input(info->rx_gpio);
	}
	if (info->tx_gpio) {
		lnw_gpio_set_alt(info->tx_gpio, LNW_GPIO);
		gpio_direction_input(info->tx_gpio);
	}
	if (info->cts_gpio) {
		lnw_gpio_set_alt(info->cts_gpio, LNW_GPIO);
		gpio_direction_input(info->cts_gpio);
	}
	if (info->rts_gpio) {
		lnw_gpio_set_alt(info->rts_gpio, LNW_GPIO);
		gpio_direction_input(info->rts_gpio);
	}
}

void intel_mid_hsu_suspend(int port, struct device *dev, irq_handler_t wake_isr)
{
	int ret;
	struct hsu_port_pin_cfg *info = hsu_port_gpio_mux + port;

	info->dev = dev;
	info->wake_isr = wake_isr;
	if (info->rts_gpio) {
		gpio_direction_output(info->rts_gpio, 1);
		lnw_gpio_set_alt(info->rts_gpio, LNW_GPIO);
	}
	if (info->wake_gpio) {
		lnw_gpio_set_alt(info->wake_gpio, LNW_GPIO);
		gpio_direction_input(info->wake_gpio);
		udelay(100);
		ret = request_irq(gpio_to_irq(info->wake_gpio), info->wake_isr,
				IRQ_TYPE_EDGE_FALLING | IRQ_TYPE_EDGE_RISING,
				info->name, info->dev);
		if (ret)
			dev_err(info->dev, "failed to register wakeup irq\n");
	}
}

void intel_mid_hsu_resume(int port, struct device *dev)
{
	struct hsu_port_pin_cfg *info = hsu_port_gpio_mux + port;

	if (info->wake_gpio)
		free_irq(gpio_to_irq(info->wake_gpio), info->dev);

	hsu_port_enable(port);
}

void intel_mid_hsu_switch(int port)
{
	int i;
	struct hsu_port_pin_cfg *tmp;
	struct hsu_port_pin_cfg *info = hsu_port_gpio_mux + port;

	for (i = 0; i < hsu_port_max; i++) {
		tmp = hsu_port_gpio_mux + i;
		if (tmp != info && tmp->id == info->id)
			hsu_port_disable(i);
	}
	hsu_port_enable(port);
}

void intel_mid_hsu_rts(int port, int value)
{
	struct hsu_port_pin_cfg *info = hsu_port_gpio_mux + port;

	if (value) {
		if (info->rts_gpio) {
			gpio_direction_output(info->rts_gpio, 1);
			lnw_gpio_set_alt(info->rts_gpio, LNW_GPIO);
		}
	} else {
		if (info->rts_gpio)
			lnw_gpio_set_alt(info->rts_gpio, info->rts_alt);
	}
}

int intel_mid_hsu_init(int port)
{
	struct hsu_port_pin_cfg *info;
	switch (intel_mid_identify_cpu()) {
	case INTEL_MID_CPU_CHIP_CLOVERVIEW:
		hsu_port_gpio_mux = &hsu_port_pin_cfgs[hsu_clv][0];
		break;

	case INTEL_MID_CPU_CHIP_PENWELL:
	default:
		hsu_port_gpio_mux = &hsu_port_pin_cfgs[hsu_pnw][0];
		break;
	}

	if (hsu_port_gpio_mux == NULL)
		return -ENODEV;

	if (port >= hsu_port_max)
		return -ENODEV;

	info = hsu_port_gpio_mux + port;
	if (info->wake_gpio)
		gpio_request(info->wake_gpio, "hsu");
	if (info->rx_gpio)
		gpio_request(info->rx_gpio, "hsu");
	if (info->tx_gpio)
		gpio_request(info->tx_gpio, "hsu");
	if (info->cts_gpio)
		gpio_request(info->cts_gpio, "hsu");
	if (info->rts_gpio)
		gpio_request(info->rts_gpio, "hsu");

	return 1;
}

void *hsu_dev_platform_data(void *data)
{
	switch (intel_mid_identify_cpu()) {
	case INTEL_MID_CPU_CHIP_CLOVERVIEW:
		hsu_register_board_info(&hsu_port_cfgs[hsu_clv][0]);
		break;

	case INTEL_MID_CPU_CHIP_PENWELL:
	default:
		hsu_register_board_info(&hsu_port_cfgs[hsu_pnw][0]);
		break;
	}
}
arch_initcall(hsu_dev_platform_data);
