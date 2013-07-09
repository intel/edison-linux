/*
 * platform_wl12xx.c: wl12xx platform data initilization file
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/gpio.h>
#include <linux/lnw_gpio.h>
#include <linux/wl12xx.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <asm/intel-mid.h>
#include "platform_wl12xx.h"

static int wl12xx_platform_init(struct wl12xx_platform_data *platform_data);
static void wl12xx_platform_deinit(struct wl12xx_platform_data *platform_data);

static struct wl12xx_platform_data mid_wifi_control = {
	.board_ref_clock = 1,
	.irq = 2,
	.gpio = -EINVAL,
	.board_tcxo_clock = 1,
	.platform_quirks = WL12XX_PLATFORM_QUIRK_EDGE_IRQ,
	.hw_init = wl12xx_platform_init,
	.hw_deinit = wl12xx_platform_deinit,
};

static struct regulator_consumer_supply wl12xx_vmmc3_supply = {
	.supply		= "vmmc",
	.dev_name	= "0000:00:00.0", /*default value*/
};

static struct regulator_init_data wl12xx_vmmc3 = {
	.constraints = {
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies = &wl12xx_vmmc3_supply,
};

static struct fixed_voltage_config wl12xx_vwlan = {
	.supply_name		= "vwl1271",
	.microvolts		= 1800000,
	.gpio			= 75,
	.startup_delay		= 70000,
	.enable_high		= 1,
	.enabled_at_boot	= 0,
	.init_data		= &wl12xx_vmmc3,
};

static struct platform_device wl12xx_vwlan_device = {
	.name		= "reg-fixed-voltage",
	.id		= 1,
	.dev = {
		.platform_data	= &wl12xx_vwlan,
	},
};

void __init wl12xx_platform_data_init(void *info)
{
	struct sd_board_info *sd_info = info;
	int err;

	/*Get GPIO numbers from the SFI table*/
	mid_wifi_control.gpio = get_gpio_by_name(WL12XX_SFI_GPIO_IRQ_NAME);
	if (mid_wifi_control.gpio == -1) {
		pr_err("%s: Unable to find WLAN-interrupt GPIO in the SFI table\n",
				__func__);
		return;
	}

	/* Set our board_ref_clock from SFI SD board info */
	if (sd_info->board_ref_clock == ICDK_BOARD_REF_CLK)
		/*iCDK board*/
		/*26Mhz TCXO clock ref*/
		mid_wifi_control.board_ref_clock = 1;
	else if (sd_info->board_ref_clock == NCDK_BOARD_REF_CLK)
		/*nCDK board*/
		/*38,4Mhz TCXO clock ref*/
		mid_wifi_control.board_ref_clock = 2;
	err = wl12xx_set_platform_data(&mid_wifi_control);
	if (err < 0)
		pr_err("error setting wl12xx data\n");

	/* this is the fake regulator that mmc stack use to power of the
	   wifi sdio card via runtime_pm apis */
	wl12xx_vwlan.gpio = get_gpio_by_name(WL12XX_SFI_GPIO_ENABLE_NAME);
	if (wl12xx_vwlan.gpio == -1) {
		pr_err("%s: Unable to find WLAN-enable GPIO in the SFI table\n",
		       __func__);
		return;
	}
	/* format vmmc reg address from sfi table */
	sprintf((char *)wl12xx_vmmc3_supply.dev_name, "0000:00:%02x.%01x",
		(sd_info->addr)>>8, sd_info->addr&0xFF);

	err = platform_device_register(&wl12xx_vwlan_device);
	if (err < 0)
		pr_err("error platform_device_register\n");

}

void __init *wl12xx_platform_data(void *info)
{
	wl12xx_platform_data_init(info);

	return &mid_wifi_control;
}

static int wl12xx_platform_init(struct wl12xx_platform_data *platform_data)
{
	int err = 0;

	if (IS_ERR(platform_data)) {
		err = PTR_ERR(platform_data);
		pr_err("%s: missing wlan platform data: %d\n", __func__, err);
		goto out;
	}

	/* gpio must be set to -EINVAL by platform code if
	   gpio based irq is not used*/

	if (gpio_is_valid(platform_data->gpio)) {
		if (!platform_data->gpio)
			pr_warn("using GPIO %d for wl12xx\n",
						platform_data->gpio);

		/* Request gpio */
		err = gpio_request(platform_data->gpio, "wl12xx");
		if (err < 0) {
			pr_err("%s: Unable to request GPIO:%d, err:%d\n",
					__func__, platform_data->gpio, err);
			goto out;
		}

		/* set gpio direction */
		err = gpio_direction_input(platform_data->gpio);
		if (err < 0) {
			pr_err("%s: Unable to set GPIO:%d direction, err:%d\n",
			 __func__, platform_data->gpio, err);
			goto out;
		}

		/* convert gpio to irq */
		platform_data->irq = gpio_to_irq(platform_data->gpio);
		if (platform_data->irq < 0) {
			pr_err("%s: Error gpio_to_irq:%d->%d\n", __func__,
					platform_data->gpio,
					platform_data->irq);
			goto out;
		}
	}

	sdhci_pci_request_regulators();

	pr_info("%s done\n", __func__);
out:
	return err;
}

static void wl12xx_platform_deinit(struct wl12xx_platform_data *pdata)
{
	/* get platform data and free the gpio */
	if (IS_ERR(pdata)) {
		pr_err("%s: missing wlan platform data\n", __func__);
		goto out;
	}

	if (gpio_is_valid(pdata->gpio)) {
		if (!pdata->gpio)
			pr_warn("using GPIO %d for wl12xx\n", pdata->gpio);
		gpio_free(pdata->gpio);
	}
out:
	return ;
}
