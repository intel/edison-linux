/*
 * platform_gpio_keys.c: gpio_keys platform data initilization file
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/input.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/platform_device.h>
#include <asm/intel-mid.h>
#include "platform_gpio_keys.h"

/*
 * we will search these buttons in SFI GPIO table (by name)
 * and register them dynamically. Please add all possible
 * buttons here, we will shrink them if no GPIO found.
 */
static struct gpio_keys_button gpio_button[] = {
        {
                .code = KEY_POWER,
                .gpio = -1, /* GPIO number */
                .active_low = 1,
                .desc = "power_btn",/*Button description*/
                .type = EV_KEY,
                .wakeup = 0,
                .debounce_interval = 3000,
        },
        {
		.code = KEY_PROG1,
		.gpio = 61,
		.active_low = 1,
		.desc = "SW1UI4",
		.type = EV_KEY,
		.wakeup = 0,
		.debounce_interval = 50,
        },
};

static struct gpio_keys_platform_data gpio_keys = {
	.buttons	= gpio_button,
	.rep		= 1,
	.nbuttons	= -1, /* will fill it after search */
};

static struct platform_device pb_device = {
	.name		= DEVICE_NAME,
	.id		= -1,
	.dev		= {
		.platform_data	= &gpio_keys,
	},
};

/*
 * Shrink the non-existent buttons, register the gpio button
 * device if there is some
 */
static int __init pb_keys_init(void)
{
	struct gpio_keys_button *gb = gpio_button;
	int i, num, good = 0;

	num = sizeof(gpio_button) / sizeof(struct gpio_keys_button);
	for (i = 0; i < num; i++) {
		pr_info("info[%2d]: name = %s, gpio = %d\n",
			 i, gb[i].desc, gb[i].gpio);
		if (gb[i].gpio == -1)
			continue;

		if (i != good)
			gb[good] = gb[i];
		good++;
	}

	if (good) {
		gpio_keys.nbuttons = good;
		return platform_device_register(&pb_device);
	}
	return 0;
}
late_initcall(pb_keys_init);
