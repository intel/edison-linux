
/* intel_mid_dma_acpi.c - Intel MID DMA driver init file for ACPI enumaration.
 *
 * Copyright (c) 2013, Intel Corporation.
 *
 *  Authors:	Ramesh Babu K V <Ramesh.Babu@intel.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/intel_mid_dma.h>
#include <linux/pm_runtime.h>
#include <acpi/acpi_bus.h>

#include "intel_mid_dma_regs.h"

#define HID_MAX_SIZE 8

struct list_head dma_dev_list;

LIST_HEAD(dma_dev_list);

struct acpi_dma_dev_list {
	struct list_head dmadev_list;
	char dma_hid[HID_MAX_SIZE];
	struct device *acpi_dma_dev;
};

struct device *intel_mid_get_acpi_dma(const char *hid)
{
	struct acpi_dma_dev_list *listnode;
	if (list_empty(&dma_dev_list))
		return NULL;

	list_for_each_entry(listnode, &dma_dev_list, dmadev_list) {
		if (!(strncmp(listnode->dma_hid, hid, HID_MAX_SIZE)))
			return listnode->acpi_dma_dev;
	}
	return NULL;
}
EXPORT_SYMBOL_GPL(intel_mid_get_acpi_dma);

#if IS_ENABLED(CONFIG_ACPI)
static int mid_get_and_map_rsrc(void **dest, struct platform_device *pdev,
				unsigned int num)
{
	struct resource *rsrc;
	rsrc = platform_get_resource(pdev, IORESOURCE_MEM, num);
	if (!rsrc) {
		pr_err("%s: Invalid resource - %d", __func__, num);
		return -EIO;
	}
	pr_debug("rsrc #%d = %#x", num, rsrc->start);
	*dest = devm_ioremap_nocache(&pdev->dev, rsrc->start, resource_size(rsrc));
	if (!*dest) {
		pr_err("%s: unable to map resource: %#x", __func__, rsrc->start);
		return -EIO;
	}
	return 0;
}

static int mid_platform_get_resources(struct middma_device *mid_device,
				      struct platform_device *pdev)
{
	int ret;
	pr_debug("%s", __func__);

	/* All ACPI resource request here */
	/* Get DDR addr from platform resource table */
	ret = mid_get_and_map_rsrc(&mid_device->dma_base, pdev, 0);
	if (ret)
		return ret;
	pr_debug("dma_base:%p", mid_device->dma_base);

	ret = mid_get_and_map_rsrc(&mid_device->mask_reg, pdev, 1);
	if (ret)
		return ret;
	/* mask_reg should point to ISRX register */
	mid_device->mask_reg += 0x18;
	pr_debug("pimr_base:%p", mid_device->mask_reg);

	mid_device->irq = platform_get_irq(pdev, 0);
	if (mid_device->irq < 0) {
		pr_err("invalid irq:%d", mid_device->irq);
		return mid_device->irq;
	}
	pr_debug("irq from pdev is:%d", mid_device->irq);

	return 0;
}

int dma_acpi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	acpi_handle handle = ACPI_HANDLE(dev);
	struct acpi_device *device;
	struct middma_device *mid_device;
	struct intel_mid_dma_probe_info *info;
	const char *hid;
	int ret;

	ret = acpi_bus_get_device(handle, &device);
	if (ret) {
		pr_err("%s: could not get acpi device - %d\n", __func__, ret);
		return -ENODEV;
	}

	if (acpi_bus_get_status(device) || !device->status.present) {
		pr_err("%s: device has invalid status", __func__);
		return -ENODEV;
	}

	hid = acpi_device_hid(device);
	pr_info("%s for %s", __func__, hid);

	/* Apply default dma_mask if needed */
	if (!pdev->dev.dma_mask) {
		pdev->dev.dma_mask = &pdev->dev.coherent_dma_mask;
		pdev->dev.coherent_dma_mask = DMA_BIT_MASK(32);
	}

	ret = dma_set_mask(&pdev->dev, DMA_BIT_MASK(32));
	if (ret) {
		pr_err("dma_set_mask failed with err:%d", ret);
		return ret;
	}

	ret = dma_set_coherent_mask(&pdev->dev, DMA_BIT_MASK(32));
	if (ret) {
		pr_err("_coherent_mask failed with err:%d", ret);
		return ret;
	}
	info = mid_get_acpi_driver_data(hid);
	if (!info) {
		pr_err("acpi driver data is null");
		goto err_dma;
	}

	mid_device = mid_dma_setup_context(&pdev->dev, info);
	if (!mid_device)
		goto err_dma;

	ret = mid_platform_get_resources(mid_device, pdev);
	if (ret) {
		pr_err("Error while get resources:%d", ret);
		goto err_dma;
	}
	platform_set_drvdata(pdev, mid_device);
	ret = mid_setup_dma(&pdev->dev);
	if (ret)
		goto err_dma;
	pm_runtime_enable(&pdev->dev);
	acpi_dma_dev = &pdev->dev;
	pr_debug("%s:completed", __func__);
	return 0;
err_dma:
	pr_err("ERR_MDMA:Probe failed %d\n", ret);
	return ret;
}
#else
int dma_acpi_probe(struct platform_device *pdev)
{
	return -EIO;
}
#endif

int dma_acpi_remove(struct platform_device *pdev)
{
	pm_runtime_forbid(&pdev->dev);
	middma_shutdown(&pdev->dev);
	platform_set_drvdata(pdev, NULL);
	return 0;
}
