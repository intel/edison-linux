/*
 * platform_pcal9555a.c: pcal9555a platform data initilization file
 *
 * (C) Copyright 2014 Intel Corporation
 * Author: Dan O'Donovan <dan@emutex.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/i2c/pca953x.h>
#include <asm/intel-mid.h>
#include "platform_pcal9555a.h"


void __init *pcal9555a_platform_data(void *info)
{
	static struct pca953x_platform_data pcal9555a_pdata[PCAL9555A_NUM];
	static int nr;
	struct pca953x_platform_data *pcal9555a;
	struct i2c_board_info *i2c_info = info;
	int gpio_base, intr;
	char base_pin_name[SFI_NAME_LEN + 1];
	char intr_pin_name[SFI_NAME_LEN + 1];

	if (!info) {
		pr_err("%s: invalid info pointer\n", __func__);
		return NULL;
	}

	if (nr >= PCAL9555A_NUM) {
		pr_err("%s: too many pcal9555a, we only support %d\n",
			__func__, PCAL9555A_NUM);
		return NULL;
	}
	pcal9555a = &pcal9555a_pdata[nr++];

	/* we have several pcal9555a on the board, we only need load several
	 * instances of the same pca953x driver to cover them
	 */

	snprintf(base_pin_name, sizeof(base_pin_name),
		 "%s_base", i2c_info->type);
	snprintf(intr_pin_name, sizeof(intr_pin_name),
		 "%s_int", i2c_info->type);

	strcpy(i2c_info->type, "pcal9555a");

	gpio_base = get_gpio_by_name(base_pin_name);
	intr = get_gpio_by_name(intr_pin_name);

	if (gpio_base == -1)
		return NULL;
	pcal9555a->gpio_base = gpio_base;
	if (intr != -1) {
		i2c_info->irq = intr + INTEL_MID_IRQ_OFFSET;
		pcal9555a->irq_base = gpio_base + INTEL_MID_IRQ_OFFSET;
	} else {
		i2c_info->irq = -1;
		pcal9555a->irq_base = -1;
	}
	return pcal9555a;
}
