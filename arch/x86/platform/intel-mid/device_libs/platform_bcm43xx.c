/*
 * platform_bcm43xx.c: bcm43xx platform data initilization file
 *
 * (C) Copyright 2011 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/gpio.h>
#include <linux/lnw_gpio.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <asm/intel-mid.h>
#include <linux/wlan_plat.h>
#include <linux/interrupt.h>
#include "platform_bcm43xx.h"
#include <linux/mmc/sdhci.h>
#include <linux/delay.h>
#include <linux/pm_runtime.h>

#include "pci/platform_sdhci_pci.h"

/* Delay copied from broadcom reference design */
#define DELAY_ONOFF 250

static int gpio_enable;
static void (*g_virtual_cd)(void *dev_id, int card_present);
void *g_host;
static struct platform_device bcm43xx_vwlan_device;
static struct fixed_voltage_config bcm43xx_vwlan;
static char nvram_id[30];

static int bcmdhd_set_power(int on)
{
	struct sdhci_host *host = (struct sdhci_host *)g_host;

	if (on)
		mmc_power_restore_host(host->mmc);
	else
		mmc_power_save_host(host->mmc);

	return 0;
}

static int bcmdhd_set_card_detect(int detect)
{
	if (!g_virtual_cd)
		return -1;

	if (g_host)
		g_virtual_cd(g_host, detect);

	return 0;
}

static struct wifi_platform_data bcmdhd_data = {
	.set_power = bcmdhd_set_power,
	.set_carddetect = bcmdhd_set_card_detect,
	.nvram_id = (char *)&nvram_id,
};

static struct resource bcmdhd_res[] = {
	{
	.name = "bcmdhd_wlan_irq",
	.start = 1,
	.end = 1,
	.flags = IORESOURCE_IRQ | IRQF_TRIGGER_FALLING ,
	},
	{
	.name = "bcmdhd_wlan_en",
	.start = 1,
	.end = 1,
	.flags = IORESOURCE_IRQ ,
	}

};

static struct platform_device bcmdhd_device = {
	.name = "bcmdhd_wlan",
	.dev = {
		.platform_data = &bcmdhd_data,
		},
	.num_resources = ARRAY_SIZE(bcmdhd_res),
	.resource = bcmdhd_res,
};

static struct regulator_consumer_supply bcm43xx_vmmc3_supply = {
	.supply		= "vmmc",
};

static struct regulator_init_data bcm43xx_vmmc3 = {
	.constraints = {
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies = &bcm43xx_vmmc3_supply,
};

static struct fixed_voltage_config bcm43xx_vwlan = {
	.supply_name		= "vbcm43xx",
	.microvolts		= 1800000,
	.gpio			= -EINVAL,
	.startup_delay		= 1000 * DELAY_ONOFF,
	.enable_high		= 1,
	.enabled_at_boot	= 0,
	.init_data		= &bcm43xx_vmmc3,
};

static void bcm43xx_vwlan_device_release(struct device *dev) {}

static struct platform_device bcm43xx_vwlan_device = {
	.name		= "reg-fixed-voltage",
	.id		= PLATFORM_DEVID_AUTO,
	.dev = {
		.platform_data	= &bcm43xx_vwlan,
		.release = bcm43xx_vwlan_device_release,
	},
};

static void generate_nvram_id(void)
{
	if (INTEL_MID_BOARD(2, PHONE, CLVTP, VB, PRO))
		strncpy(nvram_id, "victoriabay_prx", sizeof(nvram_id));
	else if (INTEL_MID_BOARD(2, PHONE, MRFL, BB, PRO))
		strncpy(nvram_id, "bodegabay_pr1", sizeof(nvram_id));
	else
		strncpy(nvram_id, "aob", sizeof(nvram_id));

	nvram_id[sizeof(nvram_id) - 1] = '\0';
}

void bcmdhd_register_embedded_control(void *dev_id,
			void (*virtual_cd)(void *dev_id, int card_present))
{
	struct sdhci_host *host = (struct sdhci_host *)dev_id;
	int err;

	g_virtual_cd = virtual_cd;
	g_host = dev_id;

	bcm43xx_vmmc3_supply.dev_name = kstrdup(dev_name(mmc_dev(host->mmc)),
						GFP_KERNEL);

	bcm43xx_vwlan.gpio = gpio_enable;

	/* add a fake sdhci controler regulator routed to bcm enable gpio */
	err = platform_device_register(&bcm43xx_vwlan_device);
	if (err < 0)
		pr_err("%s: failed to register platform device\n", __func__);

	sdhci_pci_request_regulators();
}

void __init wifi_platform_data_init_sfi(void)
{
	int wifi_irq_gpio;
	int err;

	pr_debug("%s\n", __func__);

	wifi_irq_gpio = get_gpio_by_name(BCM43XX_SFI_GPIO_IRQ_NAME);
	if (wifi_irq_gpio < 0) {
		pr_err("%s: Unable to find WLAN-interrupt in the SFI table\n",
				__func__);
		return;
	}

	bcmdhd_res[0].start = wifi_irq_gpio;
	bcmdhd_res[0].end = bcmdhd_res[0].start;

	gpio_enable = get_gpio_by_name(BCM43XX_SFI_GPIO_ENABLE_NAME);
	if (gpio_enable < 0) {
		pr_err("%s: Unable to find WLAN-enable GPIO in the SFI table\n",
		       __func__);
		return;
	}


	bcmdhd_res[1].start = gpio_enable;
	bcmdhd_res[1].end = bcmdhd_res[1].start;

	err = platform_device_register(&bcmdhd_device);
	if (err < 0)
		pr_err("platform_device_register failed for bcmdhd_device\n");
}

void __init *bcm43xx_platform_data(void *info)
{
	struct sd_board_info *sd_info = NULL;
	unsigned int sdhci_quirk = SDHCI_QUIRK2_ADVERTISE_2V0_FORCE_1V8
			| SDHCI_QUIRK2_ENABLE_MMC_PM_IGNORE_PM_NOTIFY;

	pr_debug("%s\n", __func__);

	sdhci_pdata_set_quirks(sdhci_quirk);
	sdhci_pdata_set_embedded_control(&bcmdhd_register_embedded_control);

	generate_nvram_id();

#ifndef CONFIG_ACPI
	sd_info = kmemdup(info, sizeof(*sd_info), GFP_KERNEL);

	if (!sd_info) {
		pr_err("%s: kmemdup error\n", __func__);
		return NULL;
	}
	wifi_platform_data_init_sfi();
#endif

	return &bcmdhd_device;
}
