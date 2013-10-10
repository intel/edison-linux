/*
 * board-blackbay.c: Intel Medfield based board (Blackbay)
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
#include <linux/scatterlist.h>
#include <linux/sfi.h>
#include <linux/intel_pmic_gpio.h>
#include <linux/spi/spi.h>
#include <linux/i2c.h>
#include <linux/i2c/pca953x.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/mfd/intel_msic.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/i2c-gpio.h>

#include <asm/setup.h>
#include <asm/mpspec_def.h>
#include <asm/hw_irq.h>
#include <asm/apic.h>
#include <asm/io_apic.h>
#include <asm/intel-mid.h>
#include <asm/intel_mid_vrtc.h>
#include <asm/io.h>
#include <asm/i8259.h>
#include <asm/intel_scu_ipc.h>
#include <asm/apb_timer.h>
#include <asm/reboot.h>

/*
 * IPC devices
 */
#include "device_libs/platform_ipc.h"
#include "device_libs/platform_pmic_gpio.h"
#include "device_libs/platform_msic.h"
#include "device_libs/platform_msic_battery.h"
#include "device_libs/platform_msic_gpio.h"
#include "device_libs/platform_msic_audio.h"
#include "device_libs/platform_msic_power_btn.h"
#include "device_libs/platform_msic_ocd.h"
#include "device_libs/platform_msic_vdd.h"
#include "device_libs/platform_mrfl_ocd.h"
#include "device_libs/platform_msic_thermal.h"
#include "device_libs/platform_soc_thermal.h"
#include "device_libs/platform_msic_adc.h"
#include "device_libs/platform_bcove_adc.h"
#include <asm/platform_mrfld_audio.h>
#include <asm/platform_ctp_audio.h>
#include "device_libs/platform_mrfl_thermal.h"

/*
 * I2C devices
 */
#include "device_libs/platform_max7315.h"
#include "device_libs/platform_tca6416.h"
#include "device_libs/platform_mpu3050.h"
#include "device_libs/platform_emc1403.h"
#include "device_libs/platform_lis331.h"
#include "device_libs/platform_mpu3050.h"
#include "device_libs/platform_tc35876x.h"

/*
 * SPI devices
 */
#include "device_libs/platform_max3111.h"

/* WIFI devices */
#include "device_libs/platform_wl12xx.h"
#include "device_libs/platform_wifi.h"

static void __init *no_platform_data(void *info)
{
	return NULL;
}

struct devs_id __initconst device_ids[] = {
	/* SD devices */
	{"wl12xx_clk_vmmc", SFI_DEV_TYPE_SD, 0, &wl12xx_platform_data, NULL},
	{"bcm43xx_clk_vmmc", SFI_DEV_TYPE_SD, 0, &wifi_platform_data, NULL},
	{"bcm43xx_vmmc", SFI_DEV_TYPE_SD, 0, &wifi_platform_data, NULL},
	{"iwlwifi_clk_vmmc", SFI_DEV_TYPE_SD, 0, &wifi_platform_data, NULL},
	{"WLAN_FAST_IRQ", SFI_DEV_TYPE_SD, 0, &no_platform_data,
	 &wifi_platform_data_fastirq},

	/* SPI devices */
	{"bma023", SFI_DEV_TYPE_I2C, 1, &no_platform_data, NULL},
	{"pmic_gpio", SFI_DEV_TYPE_SPI, 1, &pmic_gpio_platform_data, NULL},
	{"pmic_gpio", SFI_DEV_TYPE_IPC, 1, &pmic_gpio_platform_data,
					&ipc_device_handler},
	{"spi_max3111", SFI_DEV_TYPE_SPI, 0, &max3111_platform_data, NULL},
	{"i2c_max7315", SFI_DEV_TYPE_I2C, 1, &max7315_platform_data, NULL},
	{"i2c_max7315_2", SFI_DEV_TYPE_I2C, 1, &max7315_platform_data, NULL},
	{"tca6416", SFI_DEV_TYPE_I2C, 1, &tca6416_platform_data, NULL},
	{"emc1403", SFI_DEV_TYPE_I2C, 1, &emc1403_platform_data, NULL},
	{"i2c_accel", SFI_DEV_TYPE_I2C, 0, &lis331dl_platform_data, NULL},
	{"pmic_audio", SFI_DEV_TYPE_IPC, 1, &no_platform_data,
					&ipc_device_handler},
	{"mpu3050", SFI_DEV_TYPE_I2C, 1, &mpu3050_platform_data, NULL},
	{"i2c_disp_brig", SFI_DEV_TYPE_I2C, 0, &tc35876x_platform_data, NULL},

	/* MSIC subdevices */
	{"msic_adc", SFI_DEV_TYPE_IPC, 1, &msic_adc_platform_data,
						&ipc_device_handler},
	{"bcove_power_btn", SFI_DEV_TYPE_IPC, 1, &msic_power_btn_platform_data,
					&ipc_device_handler},
	{"msic_battery", SFI_DEV_TYPE_IPC, 1, &msic_battery_platform_data,
					&ipc_device_handler},
	{"msic_gpio", SFI_DEV_TYPE_IPC, 1, &msic_gpio_platform_data,
					&ipc_device_handler},
	{"msic_audio", SFI_DEV_TYPE_IPC, 1, &msic_audio_platform_data,
					&ipc_device_handler},
	{"msic_power_btn", SFI_DEV_TYPE_IPC, 1, &msic_power_btn_platform_data,
					&ipc_device_handler},
	{"msic_ocd", SFI_DEV_TYPE_IPC, 1, &msic_ocd_platform_data,
					&ipc_device_handler},
	{"bcove_bcu", SFI_DEV_TYPE_IPC, 1, &mrfl_ocd_platform_data,
					&ipc_device_handler},
	{"msic_thermal", SFI_DEV_TYPE_IPC, 1, &msic_thermal_platform_data,
					&ipc_device_handler},
	{"bcove_adc", SFI_DEV_TYPE_IPC, 1, &bcove_adc_platform_data,
					&ipc_device_handler},
	{"bcove_thrm", SFI_DEV_TYPE_IPC, 1, &mrfl_thermal_platform_data,
					&ipc_device_handler},

	/* I2C devices */
	{"bq24192", SFI_DEV_TYPE_I2C, 1, &bq24192_platform_data},
	/* Panel */
	{"PANEL_CMI_CMD", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},
	{"PANEL_JDI_VID", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},
	{"ctp_lt_wm8994", SFI_DEV_TYPE_IPC, 1, &ctp_audio_platform_data,
						&ipc_device_handler},

	/* I2C devices for camera image subsystem */
	{"lm3554", SFI_DEV_TYPE_I2C, 0, &lm3554_platform_data_func,
					&intel_register_i2c_camera_device},
	{"mt9e013", SFI_DEV_TYPE_I2C, 0, &mt9e013_platform_data,
					&intel_register_i2c_camera_device},
	{"mt9d113", SFI_DEV_TYPE_I2C, 0, &mt9d113_platform_data,
					&intel_register_i2c_camera_device},
	{"mt9m114", SFI_DEV_TYPE_I2C, 0, &mt9m114_platform_data,
					&intel_register_i2c_camera_device},
	{"mt9v113", SFI_DEV_TYPE_I2C, 0, &mt9v113_platform_data,
					&intel_register_i2c_camera_device},
	{"ov8830", SFI_DEV_TYPE_I2C, 0, &ov8830_platform_data,
					&intel_register_i2c_camera_device},
	{"ov5640", SFI_DEV_TYPE_I2C, 0, &ov5640_platform_data,
					&intel_register_i2c_camera_device},
	{"imx175", SFI_DEV_TYPE_I2C, 0, &imx175_platform_data,
					&intel_register_i2c_camera_device},
	{"imx135", SFI_DEV_TYPE_I2C, 0, &imx135_platform_data,
					&intel_register_i2c_camera_device},
	{"imx134", SFI_DEV_TYPE_I2C, 0, &imx134_platform_data,
					&intel_register_i2c_camera_device},
	{"imx132", SFI_DEV_TYPE_I2C, 0, &imx132_platform_data,
					&intel_register_i2c_camera_device},
	{"s5k8aay", SFI_DEV_TYPE_I2C, 0, &s5k8aay_platform_data,
					&intel_register_i2c_camera_device},
	{"ov9724", SFI_DEV_TYPE_I2C, 0, &ov9724_platform_data,
					&intel_register_i2c_camera_device},
	{"ov2722", SFI_DEV_TYPE_I2C, 0, &ov2722_platform_data,
					&intel_register_i2c_camera_device},
	{"lm3559", SFI_DEV_TYPE_I2C, 0, &lm3559_platform_data_func,
					&intel_register_i2c_camera_device},
	{"audience_es305", SFI_DEV_TYPE_I2C, 0, &audience_platform_data,
						NULL},
	{"wm8994", SFI_DEV_TYPE_I2C, 0, &wm8994_platform_data, NULL},
	{"PANEL_JDI_CMD", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},

	/* IPC devices */
	{"pmic_charger", SFI_DEV_TYPE_IPC, 1, &no_platform_data, NULL},
	{"pmic_ccsm", SFI_DEV_TYPE_IPC, 1, &mrfl_pmic_ccsm_platform_data,
						&ipc_device_handler},
	{"i2c_pmic_adap", SFI_DEV_TYPE_IPC, 1, &mrfl_pmic_i2c_platform_data,
						&ipc_device_handler},

	/* IPC devices */
	{"ctp_rhb_cs42l73", SFI_DEV_TYPE_IPC, 1, &ctp_audio_platform_data,
						&ipc_device_handler},
	{"ctp_vb_cs42l73", SFI_DEV_TYPE_IPC, 1, &ctp_audio_platform_data,
						&ipc_device_handler},
	{"ctp_ht_wm5102", SFI_DEV_TYPE_IPC, 1, &ctp_audio_platform_data,
						&ipc_device_handler},
	{"mrfld_lm49453", SFI_DEV_TYPE_IPC, 1, &merfld_audio_platform_data,
						&ipc_device_handler},
	{"mrfld_wm8958", SFI_DEV_TYPE_IPC, 1, &merfld_wm8958_audio_platform_data,
						&ipc_device_handler},
	{"soc_thrm", SFI_DEV_TYPE_IPC, 1, &no_platform_data,
						&soc_thrm_device_handler},
	{"wm8958", SFI_DEV_TYPE_I2C, 0, &wm8994_platform_data, NULL},
	{"lm49453_codec", SFI_DEV_TYPE_I2C, 1, &no_platform_data, NULL},
	{"cs42l73", SFI_DEV_TYPE_I2C, 1, &cs42l73_platform_data, NULL},
#ifndef CONFIG_HSI_NO_MODEM
	{"hsi_ifx_modem", SFI_DEV_TYPE_HSI, 0, &hsi_modem_platform_data, NULL},
	{"hsi_ffl_modem", SFI_DEV_TYPE_HSI, 0, &ffl_modem_platform_data, NULL},
	{"hsi_edlp_modem", SFI_DEV_TYPE_HSI, 0, &edlp_modem_platform_data,
						NULL},
	{"hsi_edlp_fast", SFI_DEV_TYPE_HSI, 0, &edlp_fast_platform_data, NULL},
#endif
	{"XMM_6260", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM_6268", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM_6360", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM_7160_REV1", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM_7160_REV3", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM_7160_REV3_5", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM_7160_REV4", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM_7260_REV1", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"RMC_CYGNUS", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"CYGNUS_FFRD_EU", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"CYGNUS_FFRD_NA", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"RMC_CYGNUS_PCI", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"RMC_PEGASUS", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{},
};
