/*
 * platform_otg_pci.c: USB OTG platform data initilization file
 *
 * (C) Copyright 2013 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/pci.h>
#include <asm/intel-mid.h>
#include <asm/intel_scu_ipc.h>
#include <linux/dma-mapping.h>

#ifdef CONFIG_USB_DWC3_OTG
#include <linux/usb/dwc3-intel-mrfl.h>
static struct intel_dwc_otg_pdata dwc_otg_pdata;
static struct intel_dwc_otg_pdata *get_otg_platform_data(struct pci_dev *pdev)
{
	switch (pdev->device) {
	case PCI_DEVICE_ID_INTEL_MRFL_DWC3_OTG:
		if (intel_mid_identify_sim() == INTEL_MID_CPU_SIMULATION_HVP)
			dwc_otg_pdata.is_hvp = 1;
		return &dwc_otg_pdata;
	default:
		break;
	}

	return NULL;
}
#endif

#ifdef CONFIG_USB_PENWELL_OTG
#include <linux/usb/penwell_otg.h>
static struct intel_mid_otg_pdata otg_pdata = {
	.gpio_vbus = 0,
	.gpio_cs = 0,
	.gpio_reset = 0,
	.charging_compliance = 0,
	.hnp_poll_support = 0,
	.power_budget = 500
};

static struct intel_mid_otg_pdata *get_otg_platform_data(struct pci_dev *pdev)
{
	struct intel_mid_otg_pdata *pdata = &otg_pdata;

	switch (pdev->device) {
	case PCI_DEVICE_ID_INTEL_MFD_OTG:
		if (INTEL_MID_BOARD(2, TABLET, MFLD, SLP, PRO) ||
			INTEL_MID_BOARD(2, TABLET, MFLD, SLP, ENG))
			pdata->gpio_vbus = 54;

		if (!INTEL_MID_BOARD(2, TABLET, MFLD, RR, PRO) &&
			!INTEL_MID_BOARD(2, TABLET, MFLD, RR, ENG))
			pdata->power_budget = 200;
		break;

	case PCI_DEVICE_ID_INTEL_CLV_OTG:
		pdata->gpio_cs = get_gpio_by_name("usb_otg_phy_cs");
		if (pdata->gpio_cs == -1) {
			pr_err("%s: No gpio pin usb_otg_phy_cs\n", __func__);
			return NULL;
		}
		pdata->gpio_reset = get_gpio_by_name("usb_otg_phy_rst");
		if (pdata->gpio_reset == -1) {
			pr_err("%s: No gpio pin usb_otg_phy_rst\n", __func__);
			return NULL;
		}
		break;

	default:
		break;
	}

	return pdata;
}
#endif

static void otg_pci_early_quirks(struct pci_dev *pci_dev)
{
	pci_dev->dev.platform_data = get_otg_platform_data(pci_dev);
}

DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_INTEL_MFD_OTG,
			otg_pci_early_quirks);
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_INTEL_CLV_OTG,
			otg_pci_early_quirks);
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_INTEL_MRFL_DWC3_OTG,
			otg_pci_early_quirks);
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_INTEL_BYT_OTG,
			otg_pci_early_quirks);
