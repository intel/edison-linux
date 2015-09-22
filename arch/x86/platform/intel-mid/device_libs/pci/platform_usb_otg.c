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
#include <linux/usb/dwc3-intel-mid.h>
static struct intel_dwc_otg_pdata dwc_otg_pdata;

static bool dwc_otg_get_usbspecoverride(void)
{
	void __iomem *usb_comp_iomap;
	bool usb_spec_override;

	/* Read MISCFLAGS byte from offset 0x717 */
	usb_comp_iomap = ioremap_nocache(0xFFFCE717, 4);
	/* MISCFLAGS.BIT[6] indicates USB spec override */
	usb_spec_override = ioread8(usb_comp_iomap) & 0x40;
	iounmap(usb_comp_iomap);

	return usb_spec_override;
}

static struct intel_dwc_otg_pdata *get_otg_platform_data(struct pci_dev *pdev)
{
	switch (pdev->device) {
	case PCI_DEVICE_ID_INTEL_MRFL_DWC3_OTG:
		dwc_otg_pdata.pmic_type = BASIN_COVE;
		dwc_otg_pdata.charger_detect_enable = 1;

		dwc_otg_pdata.charging_compliance =
			dwc_otg_get_usbspecoverride();

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
	case PCI_DEVICE_ID_INTEL_MRFL_DWC3_OTG:
		dwc_otg_pdata.pmic_type = BASIN_COVE;
		dwc_otg_pdata.charger_detect_enable = 1;

		dwc_otg_pdata.charging_compliance =
			dwc_otg_get_usbspecoverride();
		return &dwc_otg_pdata;

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

DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_INTEL_MRFL_DWC3_OTG,
			otg_pci_early_quirks);
