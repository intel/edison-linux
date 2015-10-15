/*
 * platform_dw_i2c.c: I2C platform data initilization file
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
#include <linux/lnw_gpio.h>
#include <linux/gpio.h>
#include <asm/intel-mid.h>

struct i2c_pin_cfg {
	int scl_gpio;
	int scl_alt;
	int sda_gpio;
	int sda_alt;
};

enum {
	BOARD_NONE = 0,
	BOARD_VTB,
	BOARD_SALTBAY,
};

static struct i2c_pin_cfg dw_i2c_pin_cfgs[][10] = {
	[BOARD_NONE] =  {},
	[BOARD_VTB] =  {
		[1] = {27, 1, 26, 1},
	},
	[BOARD_SALTBAY] =  {
		[1] = {19, 1, 20, 1},
	},
};

int intel_mid_dw_i2c_abort(int busnum)
{
	int i;
	int ret = -EBUSY;
	struct i2c_pin_cfg *pins = &dw_i2c_pin_cfgs[BOARD_NONE][busnum];

	switch (intel_mid_identify_cpu()) {
	case INTEL_MID_CPU_CHIP_CLOVERVIEW:
		pins = &dw_i2c_pin_cfgs[BOARD_VTB][busnum];
		break;
	case INTEL_MID_CPU_CHIP_TANGIER:
		pins = &dw_i2c_pin_cfgs[BOARD_SALTBAY][busnum];
		break;
	default:
		break;
	}

	if (!pins->scl_gpio || !pins->sda_gpio) {
		pr_err("i2c-%d: recovery ignore\n", busnum);
		return 0;
	}
	pr_err("i2c-%d: try to abort xfer, scl_gpio %d, sda_gpio %d\n",
			busnum, pins->scl_gpio, pins->sda_gpio);
	gpio_request(pins->scl_gpio, "scl");
	gpio_request(pins->sda_gpio, "sda");
	lnw_gpio_set_alt(pins->scl_gpio, LNW_GPIO);
	lnw_gpio_set_alt(pins->sda_gpio, LNW_GPIO);
	gpio_direction_input(pins->scl_gpio);
	gpio_direction_input(pins->sda_gpio);
	usleep_range(10, 10);
	pr_err("i2c-%d: scl_gpio val %d, sda_gpio val %d\n",
			busnum,
			gpio_get_value(pins->scl_gpio) ? 1 : 0,
			gpio_get_value(pins->sda_gpio) ? 1 : 0);
	gpio_direction_output(pins->scl_gpio, 1);
	pr_err("i2c-%d: toggle begin\n", busnum);
	for (i = 0; i < 9; i++) {
		if (gpio_get_value(pins->sda_gpio)) {
			if (gpio_get_value(pins->scl_gpio)) {
				pr_err("i2c-%d: recovery success\n", busnum);
				break;
			} else {
				gpio_direction_output(pins->scl_gpio, 0);
				pr_err("i2c-%d: scl_gpio val 0, sda_gpio val 1\n",
					busnum);
			}
		}
		gpio_set_value(pins->scl_gpio, 0);
		usleep_range(10, 20);
		gpio_set_value(pins->scl_gpio, 1);
		usleep_range(10, 20);
		pr_err("i2c-%d: toggle SCL loop %d\n", busnum, i);
	}
	pr_err("i2c-%d: toggle end\n", busnum);
	gpio_direction_output(pins->scl_gpio, 1);
	gpio_direction_output(pins->sda_gpio, 0);
	gpio_set_value(pins->scl_gpio, 0);
	usleep_range(10, 20);
	gpio_set_value(pins->scl_gpio, 1);
	usleep_range(10, 20);
	gpio_set_value(pins->sda_gpio, 0);
	lnw_gpio_set_alt(pins->scl_gpio, pins->scl_alt);
	lnw_gpio_set_alt(pins->sda_gpio, pins->sda_alt);
	usleep_range(10, 10);
	gpio_free(pins->scl_gpio);
	gpio_free(pins->sda_gpio);

	return ret;
}
EXPORT_SYMBOL(intel_mid_dw_i2c_abort);
