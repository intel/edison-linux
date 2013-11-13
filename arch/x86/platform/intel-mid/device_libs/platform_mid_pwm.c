/*
 * platform_mid_pwm.c: mid_pwm platform data initilization file
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
#include <linux/platform_device.h>
#include <linux/lnw_gpio.h>

#include <asm/intel-mid.h>
#include <asm/intel_mid_pwm.h>
#include <asm/intel_mid_remoteproc.h>

#include "platform_mid_pwm.h"

static struct intel_mid_pwm_device_data mfld_pwms[] = {
	[PWM_LED] = {
		.reg_clkdiv0 = 0x62,
		.reg_clkdiv1 = 0x61,
		.reg_dutycyc = 0x67,
		.val_clkdiv1 = 0x00,
		.val_clkdiv0 = 0x03,
	},
	[PWM_VIBRATOR] = {
		.reg_clkdiv0 = 0x64,
		.reg_clkdiv1 = 0x63,
		.reg_dutycyc = 0x68,
		.val_clkdiv1 = 0x00,
		.val_clkdiv0 = 0x03,
	},
	[PWM_LCD_BACKLIGHT] = {
		.reg_clkdiv0 = 0x66,
		.reg_clkdiv1 = 0x65,
		.reg_dutycyc = 0x69,
		.val_clkdiv1 = 0x00,
		.val_clkdiv0 = 0x03,
	},
};

static struct intel_mid_pwm_device_data ctp_pwms[] = {
	[PWM_LED] = {
		.reg_clkdiv0 = 0x62,
		.reg_clkdiv1 = 0x61,
		.reg_dutycyc = 0x67,
		.val_clkdiv1 = 0x00,
		.val_clkdiv0 = 0x00,
	},
	[PWM_VIBRATOR] = {
		.reg_clkdiv0 = 0x64,
		.reg_clkdiv1 = 0x63,
		.reg_dutycyc = 0x68,
		.val_clkdiv1 = 0x00,
		.val_clkdiv0 = 0x03,
	},
	[PWM_LCD_BACKLIGHT] = {
		.reg_clkdiv0 = 0x66,
		.reg_clkdiv1 = 0x65,
		.reg_dutycyc = 0x69,
		.val_clkdiv1 = 0x00,
		.val_clkdiv0 = 0x03,
	},
};

static struct intel_mid_pwm_platform_data pdata[] = {
	[mfld_pwm] = {
		.pwm_num = PWM_NUM,
		.ddata = mfld_pwms,
		.reg_clksel = 0x38F,
		.val_clksel = 0x01,
	},
	[ctp_pwm] = {
		.pwm_num = PWM_NUM,
		.ddata = ctp_pwms,
		.reg_clksel = 0x38F,
		.val_clksel = 0x00,
	},
};

static void *get_pwm_platform_data(void)
{
	if (INTEL_MID_BOARD(1, PHONE, CLVTP) ||
		(INTEL_MID_BOARD(1, TABLET, CLVT))) {
		pr_info("%s, CLV board detected\n", __func__);
		return &pdata[ctp_pwm];
	} else {
		pr_info("%s, MFLD board detected\n", __func__);
		return &pdata[mfld_pwm];
	}
}

static int __init intel_mid_pwm_init(void)
{
	struct platform_device *pdev = NULL;
	int ret = 0;

	pdev = platform_device_alloc(DEVICE_NAME, -1);

	if (!pdev) {
		pr_err("out of memory for platform dev %s\n",
					DEVICE_NAME);
		return -1;
	}

	pdev->dev.platform_data = get_pwm_platform_data();

	ret = platform_device_add(pdev);
	if (ret) {
		pr_err("failed to add platform device %s\n",
					DEVICE_NAME);
		platform_device_put(pdev);
		return -1;
	}

	return 0;
}

fs_initcall(intel_mid_pwm_init);
