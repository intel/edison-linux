/**
 * Copyright (C) 2012 Intel Corp.
 * Author: Jiebing Li
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/kernel.h>

#include <linux/usb/dwc3-intel-mid.h>
#include <linux/usb/phy.h>

#include "core.h"
#include "gadget.h"
#include "io.h"
#include "otg.h"

#include "debug.h"

#include "core.c"
#include "ep0.c"
#include "gadget.c"

int dwc3_start_peripheral(struct usb_gadget *g)
{
	struct dwc3		*dwc = gadget_to_dwc(g);
	unsigned long		flags;
	int			irq;
	int			ret = 0;

	pm_runtime_get_sync(dwc->dev);

	irq = platform_get_irq(to_platform_device(dwc->dev), 0);
	ret = request_threaded_irq(irq, dwc3_interrupt, dwc3_thread_interrupt,
			IRQF_SHARED, "dwc3", dwc);
	if (ret) {
		dev_err(dwc->dev, "failed to request irq #%d --> %d\n",
				irq, ret);
		return ret;
	}

	spin_lock_irqsave(&dwc->lock, flags);

	if (dwc->gadget_driver) {
		dwc3_core_init(dwc);
		dwc3_event_buffers_setup(dwc);
		ret = dwc3_init_for_enumeration(dwc);
		if (ret)
			goto err0;

		if (dwc->soft_connected)
			dwc3_gadget_run_stop(dwc, 1);
	}

	spin_unlock_irqrestore(&dwc->lock, flags);
	return 0;

err0:
	spin_unlock_irqrestore(&dwc->lock, flags);

	free_irq(irq, dwc);

	return ret;
}

int dwc3_stop_peripheral(struct usb_gadget *g)
{
	struct dwc3		*dwc = gadget_to_dwc(g);
	unsigned long		flags;
	u8			epnum;
	int			irq;

	spin_lock_irqsave(&dwc->lock, flags);

	dwc3_stop_active_transfers(dwc);

	if (dwc->gadget.speed != USB_SPEED_UNKNOWN) {
		dwc3_disconnect_gadget(dwc);

		dwc->gadget.speed = USB_SPEED_UNKNOWN;
	}

	dwc->start_config_issued = false;

	/* Clear Run/Stop bit */
	dwc3_gadget_run_stop(dwc, 0);

	dwc->pm_state = PM_DISCONNECTED;

	for (epnum = 0; epnum < 2; epnum++) {
		struct dwc3_ep  *dep;

		dep = dwc->eps[epnum];

		if (dep->flags & DWC3_EP_ENABLED)
			__dwc3_gadget_ep_disable(dep);
	}

	dwc3_gadget_disable_irq(dwc);

	dwc3_event_buffers_cleanup(dwc);

	spin_unlock_irqrestore(&dwc->lock, flags);

	irq = platform_get_irq(to_platform_device(dwc->dev), 0);
	free_irq(irq, dwc);

	pm_runtime_put(dwc->dev);

	return 0;
}

int dwc3_vbus_draw(struct usb_gadget *g, unsigned ma)
{
	struct dwc3         *dwc = gadget_to_dwc(g);
	struct usb_phy      *usb_phy;
	int             ret;

	dev_dbg(dwc->dev, "otg_set_power: %d ma\n", ma);

	usb_phy = usb_get_phy(USB_PHY_TYPE_USB2);
	if (!usb_phy) {
		dev_err(dwc->dev, "OTG driver not available\n");
		return -ENODEV;
	}

	ret = usb_phy_set_power(usb_phy, ma);
	usb_put_phy(usb_phy);

	return ret;
}

static int dwc3_device_intel_probe(struct platform_device *pdev)
{
	struct device_node	*node = pdev->dev.of_node;
	struct dwc3		*dwc;
	struct device		*dev = &pdev->dev;
	int			ret = -ENOMEM;
	void			*mem;

	struct dwc_device_par	*pdata;
	struct usb_phy		*usb_phy;
	struct dwc_otg2		*otg;

	mem = devm_kzalloc(dev, sizeof(*dwc) + DWC3_ALIGN_MASK, GFP_KERNEL);
	if (!mem) {
		dev_err(dev, "not enough memory\n");
		return -ENOMEM;
	}
	dwc = PTR_ALIGN(mem, DWC3_ALIGN_MASK + 1);
	dwc->mem = mem;

	pdata = (struct dwc_device_par *)pdev->dev.platform_data;
	if (!pdata) {
		dev_err(&pdev->dev, "No platform data for %s.\n",
				dev_name(&pdev->dev));
		return -ENODEV;
	}

	if (node) {
		dwc->usb2_phy = devm_usb_get_phy_by_phandle(dev, "usb-phy", 0);
		dwc->usb3_phy = devm_usb_get_phy_by_phandle(dev, "usb-phy", 1);
	} else {
		dwc->usb2_phy = devm_usb_get_phy(dev, USB_PHY_TYPE_USB2);
		dwc->usb3_phy = devm_usb_get_phy(dev, USB_PHY_TYPE_USB3);
	}

	if (IS_ERR(dwc->usb2_phy)) {
		ret = PTR_ERR(dwc->usb2_phy);

		/*
		 * if -ENXIO is returned, it means PHY layer wasn't
		 * enabled, so it makes no sense to return -EPROBE_DEFER
		 * in that case, since no PHY driver will ever probe.
		 */
		if (ret == -ENXIO)
			return ret;

		dev_err(dev, "no usb2 phy configured\n");
		return -EPROBE_DEFER;
	}

	if (IS_ERR(dwc->usb3_phy)) {
		ret = PTR_ERR(dwc->usb2_phy);

		/*
		 * if -ENXIO is returned, it means PHY layer wasn't
		 * enabled, so it makes no sense to return -EPROBE_DEFER
		 * in that case, since no PHY driver will ever probe.
		 */
		if (ret == -ENXIO)
			return ret;

		dev_err(dev, "no usb3 phy configured\n");
		return -EPROBE_DEFER;
	}

	spin_lock_init(&dwc->lock);
	platform_set_drvdata(pdev, dwc);

	dwc->regs   = pdata->io_addr + DWC3_GLOBALS_REGS_START;
	dwc->regs_size  = pdata->len - DWC3_GLOBALS_REGS_START;
	dwc->dev	= dev;

	dev->dma_mask	= dev->parent->dma_mask;
	dev->dma_parms	= dev->parent->dma_parms;
	dma_set_coherent_mask(dev, dev->parent->coherent_dma_mask);

	if (!strncmp("super", maximum_speed, 5))
		dwc->maximum_speed = DWC3_DCFG_SUPERSPEED;
	else if (!strncmp("high", maximum_speed, 4))
		dwc->maximum_speed = DWC3_DCFG_HIGHSPEED;
	else if (!strncmp("full", maximum_speed, 4))
		dwc->maximum_speed = DWC3_DCFG_FULLSPEED1;
	else if (!strncmp("low", maximum_speed, 3))
		dwc->maximum_speed = DWC3_DCFG_LOWSPEED;
	else
		dwc->maximum_speed = DWC3_DCFG_SUPERSPEED;

	dwc->needs_fifo_resize = of_property_read_bool(node, "tx-fifo-resize");

	pm_runtime_enable(dev);
	pm_runtime_get_sync(dev);
	pm_runtime_forbid(dev);

	dwc3_cache_hwparams(dwc);
	dwc3_core_num_eps(dwc);

	ret = dwc3_alloc_event_buffers(dwc, DWC3_EVENT_BUFFERS_SIZE);
	if (ret) {
		dev_err(dwc->dev, "failed to allocate event buffers\n");
		ret = -ENOMEM;
		goto err0;
	}

	usb_phy = usb_get_phy(USB_PHY_TYPE_USB2);
	otg = container_of(usb_phy, struct dwc_otg2, usb2_phy);
	otg->start_device = dwc3_start_peripheral;
	otg->stop_device = dwc3_stop_peripheral;
	otg->vbus_draw = dwc3_vbus_draw;
	usb_put_phy(usb_phy);
	dwc->is_otg = 1;


	dwc3_set_mode(dwc, DWC3_GCTL_PRTCAP_DEVICE);
	ret = dwc3_gadget_init(dwc);
	if (ret) {
		dev_err(dev, "failed to initialize gadget\n");
		goto err0;
	}
	dwc->mode = DWC3_MODE_DEVICE;

	ret = dwc3_debugfs_init(dwc);
	if (ret) {
		dev_err(dev, "failed to initialize debugfs\n");
		goto err1;
	}

	pm_runtime_allow(dev);

	return 0;

err1:
	dwc3_gadget_exit(dwc);

err0:
	dwc3_free_event_buffers(dwc);

	return ret;
}

static struct platform_driver dwc3_device_intel_driver = {
	.probe		= dwc3_device_intel_probe,
	.remove		= dwc3_remove,
	.driver		= {
		.name	= "dwc3-device",
		.of_match_table	= of_match_ptr(of_dwc3_match),
		.pm	= DWC3_PM_OPS,
	},
};

module_platform_driver(dwc3_device_intel_driver);

MODULE_ALIAS("platform:dwc3");
MODULE_AUTHOR("Felipe Balbi <balbi@ti.com>");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_DESCRIPTION("DesignWare USB3 DRD Controller Driver");
