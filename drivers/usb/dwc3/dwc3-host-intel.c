/*
 * Copyright (C) 2012 Intel Corp.
 * Author: Yu Wang
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

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/usb/otg.h>
#include <linux/platform_device.h>
#include <linux/usb/dwc3-intel-mid.h>
#include "../host/xhci.h"
#include "core.h"
#include "otg.h"

#define WAIT_DISC_EVENT_COMPLETE_TIMEOUT 5 /* 100ms */

static int otg_irqnum;

static int dwc3_start_host(struct usb_hcd *hcd);
static int dwc3_stop_host(struct usb_hcd *hcd);
static struct platform_driver dwc3_xhci_driver;

static void xhci_dwc3_quirks(struct device *dev, struct xhci_hcd *xhci)
{
	/*
	 * As of now platform drivers don't provide MSI support so we ensure
	 * here that the generic code does not try to make a pci_dev from our
	 * dev struct in order to setup MSI
	 *
	 * Synopsys DWC3 controller will generate PLC when link transfer to
	 * compliance/loopback mode.
	 */
	xhci->quirks |= XHCI_PLAT;
}

/* called during probe() after chip reset completes */
static int xhci_dwc3_setup(struct usb_hcd *hcd)
{
	return xhci_gen_setup(hcd, xhci_dwc3_quirks);
}

static int xhci_dwc_bus_resume(struct usb_hcd *hcd)
{
	int ret;

	/* before resume bus, delay 1ms to waiting core stable */
	mdelay(1);

	ret = xhci_bus_resume(hcd);
	return ret;
}

static const struct hc_driver xhci_dwc_hc_driver = {
	.description =		"dwc-xhci",
	.product_desc =		"xHCI Host Controller",
	.hcd_priv_size =	sizeof(struct xhci_hcd *),

	/*
	 * generic hardware linkage
	 */
	.irq =			xhci_irq,
	.flags =		HCD_MEMORY | HCD_USB3 | HCD_SHARED,

	/*
	 * basic lifecycle operations
	 */
	.reset =		xhci_dwc3_setup,
	.start =		xhci_run,
	.stop =			xhci_stop,
	.shutdown =		xhci_shutdown,

	/*
	 * managing i/o requests and associated device resources
	 */
	.urb_enqueue =		xhci_urb_enqueue,
	.urb_dequeue =		xhci_urb_dequeue,
	.alloc_dev =		xhci_alloc_dev,
	.free_dev =		xhci_free_dev,
	.alloc_streams =	xhci_alloc_streams,
	.free_streams =		xhci_free_streams,
	.add_endpoint =		xhci_add_endpoint,
	.drop_endpoint =	xhci_drop_endpoint,
	.endpoint_reset =	xhci_endpoint_reset,
	.check_bandwidth =	xhci_check_bandwidth,
	.reset_bandwidth =	xhci_reset_bandwidth,
	.address_device =	xhci_address_device,
	.update_hub_device =	xhci_update_hub_device,
	.reset_device =		xhci_discover_or_reset_device,

	/*
	 * scheduling support
	 */
	.get_frame_number =	xhci_get_frame,

	/* Root hub support */
	.hub_control =		xhci_hub_control,
	.hub_status_data =	xhci_hub_status_data,
	.bus_suspend =		xhci_bus_suspend,
	.bus_resume =		xhci_dwc_bus_resume,
};

static int if_usb_devices_connected(struct xhci_hcd *xhci)
{
	struct usb_device		*usb_dev;
	int i, connected_devices = 0;

	if (!xhci)
		return -EINVAL;

	usb_dev = xhci->main_hcd->self.root_hub;
	for (i = 1; i <= usb_dev->maxchild; ++i) {
		if (usb_hub_find_child(usb_dev, i))
			connected_devices++;
	}

	usb_dev = xhci->shared_hcd->self.root_hub;
	for (i = 1; i <= usb_dev->maxchild; ++i) {
		if (usb_hub_find_child(usb_dev, i))
			connected_devices++;
	}

	if (connected_devices)
		return 1;

	return 0;
}

static void dwc_xhci_enable_phy_auto_resume(struct usb_hcd *hcd, bool enable)
{
	u32 val;

	val = readl(hcd->regs + GUSB2PHYCFG0);
	val |= GUSB2PHYCFG_ULPI_EXT_VBUS_DRV;
	if (enable)
		val |= GUSB2PHYCFG_ULPI_AUTO_RESUME;
	else
		val &= ~GUSB2PHYCFG_ULPI_AUTO_RESUME;
	writel(val, hcd->regs + GUSB2PHYCFG0);
}

static void dwc_xhci_enable_phy_suspend(struct usb_hcd *hcd, bool enable)
{
	u32 val;

	val = readl(hcd->regs + GUSB3PIPECTL0);
	if (enable)
		val |= GUSB3PIPECTL_SUS_EN;
	else
		val &= ~GUSB3PIPECTL_SUS_EN;
	writel(val, hcd->regs + GUSB3PIPECTL0);

	val = readl(hcd->regs + GUSB2PHYCFG0);
	if (enable)
		val |= GUSB2PHYCFG_SUS_PHY;
	else
		val &= ~GUSB2PHYCFG_SUS_PHY;
	writel(val, hcd->regs + GUSB2PHYCFG0);
}

static void dwc_silicon_wa(struct usb_hcd *hcd)
{
	void __iomem *addr;
	u32 val;

	/* Clear GUCTL bit 15 as workaround of DWC controller Bugs
	 * This Bug cause the xHCI driver does not see any
	 * transfer complete events for certain EP after exit
	 * from hibernation mode.*/
	val = readl(hcd->regs + GUCTL);
	val &= ~GUCTL_CMDEVADDR;
	writel(val, hcd->regs + GUCTL);

	/* Disable OTG3-EXI interface by default. It is one
	 * workaround for silicon BUG. It will cause transfer
	 * failed on EP#8 of any USB device.
	 */
	addr = ioremap_nocache(APBFC_EXIOTG3_MISC0_REG, 4);
	val = readl(addr);
	val |= (1 << 3);
	writel(val, addr);
	iounmap(addr);
}

static void dwc_core_reset(struct usb_hcd *hcd)
{
	u32 val;

	val = readl(hcd->regs + GCTL);
	val |= GCTL_CORESOFTRESET;
	writel(val, hcd->regs + GCTL);

	val = readl(hcd->regs + GUSB3PIPECTL0);
	val |= GUSB3PIPECTL_PHYSOFTRST;
	writel(val, hcd->regs + GUSB3PIPECTL0);

	val = readl(hcd->regs + GUSB2PHYCFG0);
	val |= GUSB2PHYCFG_PHYSOFTRST;
	writel(val, hcd->regs + GUSB2PHYCFG0);

	msleep(100);

	val = readl(hcd->regs + GUSB3PIPECTL0);
	val &= ~GUSB3PIPECTL_PHYSOFTRST;
	writel(val, hcd->regs + GUSB3PIPECTL0);

	val = readl(hcd->regs + GUSB2PHYCFG0);
	val &= ~GUSB2PHYCFG_PHYSOFTRST;
	writel(val, hcd->regs + GUSB2PHYCFG0);

	msleep(20);

	val = readl(hcd->regs + GCTL);
	val &= ~GCTL_CORESOFTRESET;
	writel(val, hcd->regs + GCTL);
}

/*
 * On MERR platform, the suspend clock is 19.2MHz.
 * Hence PwrDnScale = 19200 / 16 = 1200 (= 0x4B0).
 * To account for possible jitter of suspend clock and to have margin,
 * So recommend it to be set to 1250 (= 0x4E2).
 * */
static void dwc_set_ssphy_p3_clockrate(struct usb_hcd *hcd)
{
	u32 gctl;

	gctl = readl(hcd->regs + GCTL);
	gctl &= ~GCTL_PWRDNSCALE_MASK;
	gctl |= GCTL_PWRDNSCALE(0x4E2);
	writel(gctl, hcd->regs + GCTL);
}

static ssize_t
show_pm_get(struct device *_dev, struct device_attribute *attr, char *buf)
{
	struct platform_device		*pdev = to_platform_device(_dev);
	struct usb_hcd		*hcd = platform_get_drvdata(pdev);

	pm_runtime_put(hcd->self.controller);
	return 0;

}
static ssize_t store_pm_get(struct device *_dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct platform_device		*pdev = to_platform_device(_dev);
	struct usb_hcd		*hcd = platform_get_drvdata(pdev);

	pm_runtime_get(hcd->self.controller);
	return count;

}
static DEVICE_ATTR(pm_get, S_IRUGO|S_IWUSR|S_IWGRP,
			show_pm_get, store_pm_get);

static void dwc_set_host_mode(struct usb_hcd *hcd)
{
	writel(0x45801000, hcd->regs + GCTL);

	msleep(20);
}

static int dwc3_start_host(struct usb_hcd *hcd)
{
	int ret = -EINVAL;
	struct xhci_hcd *xhci;
	struct usb_hcd *xhci_shared_hcd;

	if (!hcd)
		return ret;

	if (hcd->rh_registered) {
		dev_dbg(hcd->self.controller,
				"%s() - Already registered", __func__);
		return 0;
	}

	pm_runtime_get_sync(hcd->self.controller);

	dwc_core_reset(hcd);
	dwc_silicon_wa(hcd);
	dwc_set_host_mode(hcd);
	dwc_set_ssphy_p3_clockrate(hcd);

	/* Clear the hcd->flags.
	 * To prevent incorrect flags set during last time. */
	hcd->flags = 0;

	ret = usb_add_hcd(hcd, otg_irqnum, IRQF_SHARED);
	if (ret)
		return -EINVAL;

	xhci = hcd_to_xhci(hcd);
	xhci->shared_hcd = usb_create_shared_hcd(&xhci_dwc_hc_driver,
		   hcd->self.controller, dev_name(hcd->self.controller), hcd);
	if (!xhci->shared_hcd) {
		ret = -ENOMEM;
		goto dealloc_usb2_hcd;
	}

	xhci->quirks |= XHCI_PLAT;

	/* Set the xHCI pointer before xhci_pci_setup() (aka hcd_driver.reset)
	 * is called by usb_add_hcd().
	 */
	*((struct xhci_hcd **) xhci->shared_hcd->hcd_priv) = xhci;

	xhci->shared_hcd->regs = hcd->regs;

	xhci->shared_hcd->rsrc_start = hcd->rsrc_start;
	xhci->shared_hcd->rsrc_len = hcd->rsrc_len;

	ret = usb_add_hcd(xhci->shared_hcd, otg_irqnum, IRQF_SHARED);
	if (ret)
		goto put_usb3_hcd;

	pm_runtime_put(hcd->self.controller);

	ret = device_create_file(hcd->self.controller, &dev_attr_pm_get);
	if (ret < 0)
		dev_err(hcd->self.controller,
			"Can't register sysfs attribute: %d\n", ret);

	dwc3_xhci_driver.shutdown = usb_hcd_platform_shutdown;

	return ret;

put_usb3_hcd:
	if (xhci->shared_hcd) {
		xhci_shared_hcd = xhci->shared_hcd;
		usb_remove_hcd(xhci_shared_hcd);
		usb_put_hcd(xhci_shared_hcd);
	}

dealloc_usb2_hcd:
	local_irq_disable();
	usb_hcd_irq(0, hcd);
	local_irq_enable();
	usb_remove_hcd(hcd);

	kfree(xhci);
	*((struct xhci_hcd **) hcd->hcd_priv) = NULL;

	pm_runtime_put(hcd->self.controller);
	return ret;
}

static int dwc3_stop_host(struct usb_hcd *hcd)
{
	int count = 0;
	struct xhci_hcd *xhci;
	struct usb_hcd *xhci_shared_hcd;

	if (!hcd)
		return -EINVAL;

	xhci = hcd_to_xhci(hcd);

	pm_runtime_get_sync(hcd->self.controller);

	/* When plug out micro A cable, there will be two flows be executed.
	 * The first one is xHCI controller get disconnect event. The
	 * second one is PMIC get ID change event. During these events
	 * handling, they both try to call usb_disconnect. Then met some
	 * conflicts and cause kernel panic.
	 * So treat disconnect event as first priority, handle the ID change
	 * event until disconnect event handled done.*/
	while (if_usb_devices_connected(xhci)) {
		msleep(20);
		if (count++ > WAIT_DISC_EVENT_COMPLETE_TIMEOUT)
			break;
	};
	dwc3_xhci_driver.shutdown = NULL;

	if (xhci->shared_hcd) {
		xhci_shared_hcd = xhci->shared_hcd;
		usb_remove_hcd(xhci_shared_hcd);
		usb_put_hcd(xhci_shared_hcd);
	}

	usb_remove_hcd(hcd);

	kfree(xhci);
	*((struct xhci_hcd **) hcd->hcd_priv) = NULL;

	dwc_xhci_enable_phy_suspend(hcd, false);

	pm_runtime_put(hcd->self.controller);
	device_remove_file(hcd->self.controller, &dev_attr_pm_get);
	return 0;
}
static int xhci_dwc_drv_probe(struct platform_device *pdev)
{
	struct dwc_otg2 *otg;
	struct usb_phy *usb_phy;
	struct dwc_device_par *pdata;
	struct usb_hcd *hcd;
	struct resource *res;
	int retval = 0;

	if (usb_disabled())
		return -ENODEV;

	pr_debug("initializing FSL-SOC USB Controller\n");

	/* Need platform data for setup */
	pdata = (struct dwc_device_par *)pdev->dev.platform_data;
	if (!pdata) {
		dev_err(&pdev->dev,
			"No platform data for %s.\n", dev_name(&pdev->dev));
		return -ENODEV;
	}

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		dev_err(&pdev->dev,
			"Found HC with no IRQ. Check %s setup!\n",
			dev_name(&pdev->dev));
		return -ENODEV;
	}
	otg_irqnum = res->start;

	hcd = usb_create_hcd(&xhci_dwc_hc_driver,
			&pdev->dev, dev_name(&pdev->dev));
	if (!hcd) {
		retval = -ENOMEM;
		return retval;
	}

	hcd->regs = pdata->io_addr;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev,
			"Found HC with no IRQ. Check %s setup!\n",
			dev_name(&pdev->dev));
		return -ENODEV;
	}
	hcd->rsrc_start = res->start;
	hcd->rsrc_len = res->end - res->start;

	usb_phy = usb_get_phy(USB_PHY_TYPE_USB2);
	if (usb_phy)
		otg_set_host(usb_phy->otg, &hcd->self);

	otg = container_of(usb_phy->otg, struct dwc_otg2, otg);
	if (otg) {
		otg->start_host = dwc3_start_host;
		otg->stop_host = dwc3_stop_host;
	}


	usb_put_phy(usb_phy);

	/* Enable wakeup irq */
	hcd->has_wakeup_irq = 1;

	platform_set_drvdata(pdev, hcd);
	pm_runtime_enable(hcd->self.controller);

	return retval;
}

static int xhci_dwc_drv_remove(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);
	struct usb_phy *usb_phy;
	struct xhci_hcd *xhci = hcd_to_xhci(hcd);

	usb_phy = usb_get_phy(USB_PHY_TYPE_USB2);
	otg_set_host(usb_phy->otg, NULL);
	usb_put_phy(usb_phy);

	if (xhci)
		dwc3_stop_host(hcd);
	usb_put_hcd(hcd);

	pm_runtime_disable(hcd->self.controller);
	pm_runtime_set_suspended(hcd->self.controller);
	return 0;
}


#ifdef CONFIG_PM

#ifdef CONFIG_PM_RUNTIME
/*
 * Do nothing in runtime pm callback.
 * On HVP platform, if make controller go to hibernation mode.
 * controller will not send IRQ until restore status which
 * implement in pm runtime resume callback. So there is no
 * any one can trigger pm_runtime_get to resume USB3 device.
 * This issue need to continue investigate. So just implement SW logic at here.
 */
static int dwc_hcd_runtime_idle(struct device *dev)
{
	return 0;
}

/* dwc_hcd_suspend_common and dwc_hcd_resume_common are refer to
 * suspend_common and resume_common in usb core.
 * Because the usb core function just support PCI device.
 * So re-write them in here to support platform devices.
 */
static int dwc_hcd_suspend_common(struct device *dev)
{
	struct platform_device		*pdev = to_platform_device(dev);
	struct usb_hcd		*hcd = platform_get_drvdata(pdev);
	struct xhci_hcd		*xhci = hcd_to_xhci(hcd);
	int			retval = 0;
	u32 data = 0;

	if (!xhci) {
		dev_dbg(dev, "%s: host already stop!\n", __func__);
		return 0;
	}

	/* Root hub suspend should have stopped all downstream traffic,
	 * and all bus master traffic.  And done so for both the interface
	 * and the stub usb_device (which we check here).  But maybe it
	 * didn't; writing sysfs power/state files ignores such rules...
	 */
	if (HCD_RH_RUNNING(hcd)) {
		dev_warn(dev, "Root hub is not suspended\n");
		return -EBUSY;
	}
	if (hcd->shared_hcd) {
		hcd = hcd->shared_hcd;
		if (HCD_RH_RUNNING(hcd)) {
			dev_warn(dev, "Secondary root hub is not suspended\n");
			return -EBUSY;
		}
	}

	if (!HCD_DEAD(hcd)) {
		/* Optimization: Don't suspend if a root-hub wakeup is
		 * pending and it would cause the HCD to wake up anyway.
		 */
		if (HCD_WAKEUP_PENDING(hcd))
			return -EBUSY;
		if (hcd->shared_hcd &&
				HCD_WAKEUP_PENDING(hcd->shared_hcd))
			return -EBUSY;
		if (hcd->state != HC_STATE_SUSPENDED ||
				xhci->shared_hcd->state != HC_STATE_SUSPENDED)
			retval = -EINVAL;

		if (!retval) {
			/* The auto-resume is diabled by default. Need enable it
			 * if there have valid connection. To ensure that when
			 * device resumes, host does resume reflect within
			 * 900 usec as in USB spec.
			 */
			if (if_usb_devices_connected(xhci) == 1)
				dwc_xhci_enable_phy_auto_resume(
						xhci->main_hcd, true);

			/* Ensure that suspend enable are set for
			 * USB2 and USB3 PHY
			 */
			dwc_xhci_enable_phy_suspend(hcd, true);

			data = readl(hcd->regs + GCTL);
			data |= GCTL_GBL_HIBERNATION_EN;
			writel(data, hcd->regs + GCTL);
			dev_dbg(hcd->self.controller, "set xhci hibernation enable!\n");
			retval = xhci_suspend(xhci);
		}

		/* Check again in case wakeup raced with pci_suspend */
		if ((retval == 0 && HCD_WAKEUP_PENDING(hcd)) ||
				(retval == 0 && hcd->shared_hcd &&
				 HCD_WAKEUP_PENDING(hcd->shared_hcd))) {
			xhci_resume(xhci, false);
			retval = -EBUSY;
		}
		if (retval)
			return retval;
	}

	synchronize_irq(otg_irqnum);

	return retval;

}

static int dwc_hcd_resume_common(struct device *dev)
{
	struct platform_device		*pdev = to_platform_device(dev);
	struct usb_hcd		*hcd = platform_get_drvdata(pdev);
	struct xhci_hcd		*xhci = hcd_to_xhci(hcd);
	int			retval = 0;

	if (!xhci)
		return 0;

	if (HCD_RH_RUNNING(hcd) ||
			(hcd->shared_hcd &&
			 HCD_RH_RUNNING(hcd->shared_hcd))) {
		dev_dbg(dev, "can't resume, not suspended!\n");
		return 0;
	}

	if (!HCD_DEAD(hcd)) {
		retval = xhci_resume(xhci, false);
		if (retval) {
			dev_err(dev, "PCI post-resume error %d!\n", retval);
			if (hcd->shared_hcd)
				usb_hc_died(hcd->shared_hcd);
			usb_hc_died(hcd);
		}
	}

	dev_dbg(dev, "hcd_pci_runtime_resume: %d\n", retval);

	return retval;
}

static int dwc_hcd_runtime_suspend(struct device *dev)
{
	int retval;
	struct platform_device      *pdev = to_platform_device(dev);
	struct usb_hcd      *hcd = platform_get_drvdata(pdev);

	retval = dwc_hcd_suspend_common(dev);

	if (retval)
		dwc_xhci_enable_phy_auto_resume(
			hcd, false);

	dev_dbg(dev, "hcd_pci_runtime_suspend: %d\n", retval);
	return retval;
}

static int dwc_hcd_runtime_resume(struct device *dev)
{
	int retval;
	struct platform_device      *pdev = to_platform_device(dev);
	struct usb_hcd      *hcd = platform_get_drvdata(pdev);

	dwc_xhci_enable_phy_auto_resume(
			hcd, false);

	retval = dwc_hcd_resume_common(dev);
	dev_dbg(dev, "hcd_pci_runtime_resume: %d\n", retval);

	return retval;
}
#else
#define dwc_hcd_runtime_idle NULL
#define dwc_hcd_runtime_suspend NULL
#define dwc_hcd_runtime_resume NULL
#endif


static int dwc_hcd_suspend(struct device *dev)
{
	int retval;
	struct platform_device      *pdev = to_platform_device(dev);
	struct usb_hcd      *hcd = platform_get_drvdata(pdev);

	retval = dwc_hcd_suspend_common(dev);

	if (retval)
		dwc_xhci_enable_phy_auto_resume(
			hcd, false);

	dev_dbg(dev, "hcd_pci_runtime_suspend: %d\n", retval);
	return retval;
}

static int dwc_hcd_resume(struct device *dev)
{
	int retval;
	struct platform_device      *pdev = to_platform_device(dev);
	struct usb_hcd      *hcd = platform_get_drvdata(pdev);

	dwc_xhci_enable_phy_auto_resume(
			hcd, false);

	retval = dwc_hcd_resume_common(dev);
	dev_dbg(dev, "hcd_pci_runtime_resume: %d\n", retval);

	return retval;
}

static const struct dev_pm_ops dwc_usb_hcd_pm_ops = {
	.runtime_suspend = dwc_hcd_runtime_suspend,
	.runtime_resume	= dwc_hcd_runtime_resume,
	.runtime_idle	= dwc_hcd_runtime_idle,
	.suspend	=	dwc_hcd_suspend,
	.resume		=	dwc_hcd_resume,
};
#endif

static struct platform_driver dwc3_xhci_driver = {
	.probe = xhci_dwc_drv_probe,
	.remove = xhci_dwc_drv_remove,
	.driver = {
		.name = "dwc3-host",
#ifdef CONFIG_PM
		.pm = &dwc_usb_hcd_pm_ops,
#endif
	},
};
