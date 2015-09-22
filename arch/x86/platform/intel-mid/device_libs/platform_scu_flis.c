/*
 * platform_scu_flis.c: scu_flis platform data initilization file
 *
 * (C) Copyright 2013 Intel Corporation
 * Author: Ning Li <ning.li@intel.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/input.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <asm/intel-mid.h>
#include <asm/intel_mid_remoteproc.h>
#include <asm/intel_scu_flis.h>
#include "platform_scu_flis.h"

static struct pin_mmio_flis_t tng_pin_mmio_flis_table[TNG_PIN_NUM] = {
	[tng_usb_ulpi_0_clk] = { writable, 0x0500 },
	[tng_usb_ulpi_0_data_0] = { writable, 0x0504 },
	[tng_usb_ulpi_0_data_1] = { writable, 0x0508 },
	[tng_usb_ulpi_0_data_2] = { writable, 0x050C },
	[tng_usb_ulpi_0_data_3] = { writable, 0x0510 },
	[tng_usb_ulpi_0_data_4] = { writable, 0x0514 },
	[tng_usb_ulpi_0_data_5] = { writable, 0x0518 },
	[tng_usb_ulpi_0_data_6] = { writable, 0x051C },
	[tng_usb_ulpi_0_data_7] = { writable, 0x0520 },
	[tng_usb_ulpi_0_dir] = { writable, 0x0524 },
	[tng_usb_ulpi_0_nxt] = { writable, 0x0528 },
	[tng_usb_ulpi_0_refclk] = { writable, 0x052C },
	[tng_usb_ulpi_0_stp] = { writable, 0x0530 },
	[tng_emmc_0_clk] = { writable, 0x0900 },
	[tng_emmc_0_cmd] = { writable, 0x0904 },
	[tng_emmc_0_d_0] = { writable, 0x0908 },
	[tng_emmc_0_d_1] = { writable, 0x090C },
	[tng_emmc_0_d_2] = { writable, 0x0910 },
	[tng_emmc_0_d_3] = { writable, 0x0914 },
	[tng_emmc_0_d_4] = { writable, 0x0918 },
	[tng_emmc_0_d_5] = { writable, 0x091C },
	[tng_emmc_0_d_6] = { writable, 0x0920 },
	[tng_emmc_0_d_7] = { writable, 0x0924 },
	[tng_emmc_0_rst_b] = { writable, 0x0928 },
	[tng_gp_emmc_1_clk] = { writable, 0x092C },
	[tng_gp_emmc_1_cmd] = { writable, 0x0930 },
	[tng_gp_emmc_1_d_0] = { writable, 0x0934 },
	[tng_gp_emmc_1_d_1] = { writable, 0x0938 },
	[tng_gp_emmc_1_d_2] = { writable, 0x093C },
	[tng_gp_emmc_1_d_3] = { writable, 0x0940 },
	[tng_gp_emmc_1_d_4] = { writable, 0x0944 },
	[tng_gp_emmc_1_d_5] = { writable, 0x0948 },
	[tng_gp_emmc_1_d_6] = { writable, 0x094C },
	[tng_gp_emmc_1_d_7] = { writable, 0x0950 },
	[tng_gp_emmc_1_rst_b] = { writable, 0x0954 },
	[tng_gp_28] = { writable, 0x0958 },
	[tng_gp_29] = { writable, 0x095C },
	[tng_gp_sdio_0_cd_b] = { writable, 0x0D00 },
	[tng_gp_sdio_0_clk] = { writable, 0x0D04 },
	[tng_gp_sdio_0_cmd] = { writable, 0x0D08 },
	[tng_gp_sdio_0_dat_0] = { writable, 0x0D0C },
	[tng_gp_sdio_0_dat_1] = { writable, 0x0D10 },
	[tng_gp_sdio_0_dat_2] = { writable, 0x0D14 },
	[tng_gp_sdio_0_dat_3] = { writable, 0x0D18 },
	[tng_gp_sdio_0_lvl_clk_fb] = { writable, 0x0D1C },
	[tng_gp_sdio_0_lvl_cmd_dir] = { writable, 0x0D20 },
	[tng_gp_sdio_0_lvl_dat_dir] = { writable, 0x0D24 },
	[tng_gp_sdio_0_lvl_sel] = { writable, 0x0D28 },
	[tng_gp_sdio_0_powerdown_b] = { writable, 0x0D2C },
	[tng_gp_sdio_0_wp] = { writable, 0x0D30 },
	[tng_gp_sdio_1_clk] = { writable, 0x0D34 },
	[tng_gp_sdio_1_cmd] = { writable, 0x0D38 },
	[tng_gp_sdio_1_dat_0] = { writable, 0x0D3C },
	[tng_gp_sdio_1_dat_1] = { writable, 0x0D40 },
	[tng_gp_sdio_1_dat_2] = { writable, 0x0D44 },
	[tng_gp_sdio_1_dat_3] = { writable, 0x0D48 },
	[tng_gp_sdio_1_powerdown_b] = { writable, 0x0D4C },
	[tng_mhsi_acdata] = { writable, 0x1100 },
	[tng_mhsi_acflag] = { writable, 0x1104 },
	[tng_mhsi_acready] = { writable, 0x1108 },
	[tng_mhsi_acwake] = { writable, 0x110C },
	[tng_mhsi_cadata] = { writable, 0x1110 },
	[tng_mhsi_caflag] = { writable, 0x1114 },
	[tng_mhsi_caready] = { writable, 0x1118 },
	[tng_mhsi_cawake] = { writable, 0x111C },
	[tng_gp_mslim_0_bclk] = { writable, 0x1500 },
	[tng_gp_mslim_0_bdat] = { writable, 0x1504 },
	[tng_gp_ssp_0_clk] = { writable, 0x1508 },
	[tng_gp_ssp_0_fs] = { writable, 0x150C },
	[tng_gp_ssp_0_rxd] = { writable, 0x1510 },
	[tng_gp_ssp_0_txd] = { writable, 0x1514 },
	[tng_gp_ssp_1_clk] = { writable, 0x1518 },
	[tng_gp_ssp_1_fs] = { writable, 0x151C },
	[tng_gp_ssp_1_rxd] = { writable, 0x1520 },
	[tng_gp_ssp_1_txd] = { writable, 0x1524 },
	[tng_gp_ssp_2_clk] = { writable, 0x1528 },
	[tng_gp_ssp_2_fs] = { writable, 0x152C },
	[tng_gp_ssp_2_rxd] = { writable, 0x1530 },
	[tng_gp_ssp_2_txd] = { writable, 0x1534 },
	[tng_gp_ssp_3_clk] = { writable, 0x1900 },
	[tng_gp_ssp_3_fs] = { writable, 0x1904 },
	[tng_gp_ssp_3_rxd] = { writable, 0x1908 },
	[tng_gp_ssp_3_txd] = { writable, 0x190C },
	[tng_gp_ssp_4_clk] = { writable, 0x1910 },
	[tng_gp_ssp_4_fs_0] = { writable, 0x1914 },
	[tng_gp_ssp_4_fs_1] = { writable, 0x1918 },
	[tng_gp_ssp_4_fs_2] = { writable, 0x191C },
	[tng_gp_ssp_4_fs_3] = { writable, 0x1920 },
	[tng_gp_ssp_4_rxd] = { writable, 0x1924 },
	[tng_gp_ssp_4_txd] = { writable, 0x1928 },
	[tng_gp_ssp_5_clk] = { writable, 0x192C },
	[tng_gp_ssp_5_fs_0] = { writable, 0x1930 },
	[tng_gp_ssp_5_fs_1] = { writable, 0x1934 },
	[tng_gp_ssp_5_fs_2] = { writable, 0x1938 },
	[tng_gp_ssp_5_fs_3] = { writable, 0x193C },
	[tng_gp_ssp_5_rxd] = { writable, 0x1940 },
	[tng_gp_ssp_5_txd] = { writable, 0x1944 },
	[tng_gp_ssp_6_clk] = { writable, 0x1948 },
	[tng_gp_ssp_6_fs] = { writable, 0x194C },
	[tng_gp_ssp_6_rxd] = { writable, 0x1950 },
	[tng_gp_ssp_6_txd] = { writable, 0x1954 },
	[tng_gp_i2c_1_scl] = { writable, 0x1D00 },
	[tng_gp_i2c_1_sda] = { writable, 0x1D04 },
	[tng_gp_i2c_2_scl] = { writable, 0x1D08 },
	[tng_gp_i2c_2_sda] = { writable, 0x1D0C },
	[tng_gp_i2c_3_scl] = { writable, 0x1D10 },
	[tng_gp_i2c_3_sda] = { writable, 0x1D14 },
	[tng_gp_i2c_4_scl] = { writable, 0x1D18 },
	[tng_gp_i2c_4_sda] = { writable, 0x1D1C },
	[tng_gp_i2c_5_scl] = { writable, 0x1D20 },
	[tng_gp_i2c_5_sda] = { writable, 0x1D24 },
	[tng_gp_i2c_6_scl] = { writable, 0x1D28 },
	[tng_gp_i2c_6_sda] = { writable, 0x1D2C },
	[tng_gp_i2c_7_scl] = { writable, 0x1D30 },
	[tng_gp_i2c_7_sda] = { writable, 0x1D34 },
	[tng_gp_uart_0_cts] = { writable, 0x2100 },
	[tng_gp_uart_0_rts] = { writable, 0x2104 },
	[tng_gp_uart_0_rx] = { writable, 0x2108 },
	[tng_gp_uart_0_tx] = { writable, 0x210C },
	[tng_gp_uart_1_cts] = { writable, 0x2110 },
	[tng_gp_uart_1_rts] = { writable, 0x2114 },
	[tng_gp_uart_1_rx] = { writable, 0x2118 },
	[tng_gp_uart_1_tx] = { writable, 0x211C },
	[tng_gp_uart_2_cts] = { writable, 0x2120 },
	[tng_gp_uart_2_rts] = { writable, 0x2124 },
	[tng_gp_uart_2_rx] = { writable, 0x2128 },
	[tng_gp_uart_2_tx] = { writable, 0x212C },
	[tng_gp_13] = { writable, 0x2500 },
	[tng_gp_14] = { writable, 0x2504 },
	[tng_gp_15] = { writable, 0x2508 },
	[tng_gp_16] = { writable, 0x250C },
	[tng_gp_17] = { writable, 0x2510 },
	[tng_gp_18] = { writable, 0x2514 },
	[tng_gp_19] = { writable, 0x2518 },
	[tng_gp_20] = { writable, 0x251C },
	[tng_gp_21] = { writable, 0x2520 },
	[tng_gp_22] = { writable, 0x2524 },
	[tng_gp_23] = { writable, 0x2528 },
	[tng_gp_24] = { writable, 0x252C },
	[tng_gp_25] = { writable, 0x2530 },
	[tng_gp_fast_int_0] = { writable, 0x2534 },
	[tng_gp_fast_int_1] = { writable, 0x2538 },
	[tng_gp_fast_int_2] = { writable, 0x253C },
	[tng_gp_fast_int_3] = { writable, 0x2540 },
	[tng_gp_pwm_0] = { writable, 0x2544 },
	[tng_gp_pwm_1] = { writable, 0x2548 },
	[tng_gp_camerasb_0] = { writable, 0x2900 },
	[tng_gp_camerasb_1] = { writable, 0x2904 },
	[tng_gp_camerasb_2] = { writable, 0x2908 },
	[tng_gp_camerasb_3] = { writable, 0x290C },
	[tng_gp_camerasb_4] = { writable, 0x2910 },
	[tng_gp_camerasb_5] = { writable, 0x2914 },
	[tng_gp_camerasb_6] = { writable, 0x2918 },
	[tng_gp_camerasb_7] = { writable, 0x291C },
	[tng_gp_camerasb_8] = { writable, 0x2920 },
	[tng_gp_camerasb_9] = { writable, 0x2924 },
	[tng_gp_camerasb_10] = { writable, 0x2928 },
	[tng_gp_camerasb_11] = { writable, 0x292C },
	[tng_gp_clkph_0] = { writable, 0x2D00 },
	[tng_gp_clkph_1] = { writable, 0x2D04 },
	[tng_gp_clkph_2] = { writable, 0x2D08 },
	[tng_gp_clkph_3] = { writable, 0x2D0C },
	[tng_gp_clkph_4] = { writable, 0x2D10 },
	[tng_gp_clkph_5] = { writable, 0x2D14 },
	[tng_gp_hdmi_hpd] = { writable, 0x2D18 },
	[tng_gp_intd_dsi_te1] = { writable, 0x2D1C },
	[tng_gp_intd_dsi_te2] = { writable, 0x2D20 },
	[tng_osc_clk_ctrl_0] = { writable, 0x2D24 },
	[tng_osc_clk_ctrl_1] = { writable, 0x2D28 },
	[tng_osc_clk_out_0] = { writable, 0x2D2C },
	[tng_osc_clk_out_1] = { writable, 0x2D30 },
	[tng_osc_clk_out_2] = { writable, 0x2D34 },
	[tng_osc_clk_out_3] = { writable, 0x2D38 },
	[tng_osc_clk_out_4] = { writable, 0x2D3C },
	[tng_resetout_b] = { writable, 0x2D40 },
	[tng_xxpmode] = { writable, 0x2D44 },
	[tng_xxprdy] = { writable, 0x2D48 },
	[tng_xxpreq_b] = { writable, 0x2D4C },
	[tng_gp_26] = { writable, 0x2D50 },
	[tng_gp_27] = { writable, 0x2D54 },
	[tng_i2c_0_scl] = { writable, 0x3100 },
	[tng_i2c_0_sda] = { writable, 0x3104 },
	[tng_ierr_b] = { writable, 0x3108 },
	[tng_jtag_tckc] = { writable, 0x310C },
	[tng_jtag_tdic] = { writable, 0x3110 },
	[tng_jtag_tdoc] = { writable, 0x3114 },
	[tng_jtag_tmsc] = { writable, 0x3118 },
	[tng_jtag_trst_b] = { writable, 0x311C },
	[tng_prochot_b] = { writable, 0x3120 },
	[tng_rtc_clk] = { writable, 0x3124 },
	[tng_svid_vclk] = { writable, 0x3128 },
	[tng_svid_vdio] = { writable, 0x3130 },
	[tng_thermtrip_b] = { writable, 0x3134 },
	[tng_standby] = { writable, 0x3138 },
	[tng_gp_kbd_dkin_0] = { writable, 0x3500 },
	[tng_gp_kbd_dkin_1] = { writable, 0x3504 },
	[tng_gp_kbd_dkin_2] = { writable, 0x3508 },
	[tng_gp_kbd_dkin_3] = { writable, 0x350C },
	[tng_gp_kbd_mkin_0] = { writable, 0x3510 },
	[tng_gp_kbd_mkin_1] = { writable, 0x3514 },
	[tng_gp_kbd_mkin_2] = { writable, 0x3518 },
	[tng_gp_kbd_mkin_3] = { writable, 0x351C },
	[tng_gp_kbd_mkin_4] = { writable, 0x3520 },
	[tng_gp_kbd_mkin_5] = { writable, 0x3524 },
	[tng_gp_kbd_mkin_6] = { writable, 0x3528 },
	[tng_gp_kbd_mkin_7] = { writable, 0x352C },
	[tng_gp_kbd_mkout_0] = { writable, 0x3530 },
	[tng_gp_kbd_mkout_1] = { writable, 0x3534 },
	[tng_gp_kbd_mkout_2] = { writable, 0x3538 },
	[tng_gp_kbd_mkout_3] = { writable, 0x353C },
	[tng_gp_kbd_mkout_4] = { writable, 0x3540 },
	[tng_gp_kbd_mkout_5] = { writable, 0x3544 },
	[tng_gp_kbd_mkout_6] = { writable, 0x3548 },
	[tng_gp_kbd_mkout_7] = { writable, 0x354C },
	[tng_gp_0] = { writable, 0x3900 },
	[tng_gp_1] = { writable, 0x3904 },
	[tng_gp_2] = { writable, 0x3908 },
	[tng_gp_3] = { writable, 0x390C },
	[tng_gp_4] = { writable, 0x3910 },
	[tng_gp_5] = { writable, 0x3914 },
	[tng_gp_6] = { writable, 0x3918 },
	[tng_gp_7] = { writable, 0x391C },
	[tng_gp_8] = { writable, 0x3920 },
	[tng_gp_9] = { writable, 0x3924 },
	[tng_gp_10] = { writable, 0x3928 },
	[tng_gp_11] = { writable, 0x392C },
	[tng_gp_12] = { writable, 0x3930 },
	[tng_gp_mpti_clk] = { writable, 0x3D00 },
	[tng_gp_mpti_data_0] = { writable, 0x3D04 },
	[tng_gp_mpti_data_1] = { writable, 0x3D08 },
	[tng_gp_mpti_data_2] = { writable, 0x3D0C },
	[tng_gp_mpti_data_3] = { writable, 0x3D10 },
};

static int __init intel_scu_flis_init(void)
{
	int ret;
	struct platform_device *pdev = NULL;
	static struct intel_scu_flis_platform_data flis_pdata;

	flis_pdata.pin_t = NULL;
	flis_pdata.pin_num = TNG_PIN_NUM;
	flis_pdata.flis_base = 0xFF0C0000;
	flis_pdata.flis_len = 0x8000;
	flis_pdata.mmio_flis_t = tng_pin_mmio_flis_table;

	pdev = platform_device_alloc(FLIS_DEVICE_NAME, -1);
	if (!pdev) {
		pr_err("out of memory for platform dev %s\n", FLIS_DEVICE_NAME);
		ret = -EINVAL;
		goto out;
	}

	pdev->dev.platform_data = &flis_pdata;

	ret = platform_device_add(pdev);
	if (ret) {
		pr_err("failed to add flis platform device\n");
		platform_device_put(pdev);
		goto out;
	}

	pr_info("intel_scu_flis platform device created\n");
out:
	return ret;
}
fs_initcall(intel_scu_flis_init);

