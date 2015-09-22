/*
 * Synopsys DesignWare I2C adapter driver (master only).
 *
 * Based on the TI DAVINCI I2C adapter driver.
 *
 * Copyright (C) 2006 Texas Instruments.
 * Copyright (C) 2007 MontaVista Software Inc.
 * Copyright (C) 2009 Provigent Ltd.
 *
 * ----------------------------------------------------------------------------
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 * ----------------------------------------------------------------------------
 *
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/clk.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/acpi.h>
#include <linux/pci.h>
#include <linux/pm_runtime.h>
#include "i2c-designware-core.h"

#ifdef CONFIG_ACPI
static const struct acpi_device_id dw_i2c_acpi_ids[] = {
	{ "80860F41", valleyview_0 },
	{ "808622C1", cherryview_0 },
	{ }
};
MODULE_DEVICE_TABLE(acpi, dw_i2c_acpi_ids);
#endif

static int dw_i2c_plat_suspend(struct device *dev)
{
	struct platform_device *pdev =
		container_of(dev, struct platform_device, dev);
	struct dw_i2c_dev *i2c = platform_get_drvdata(pdev);

	dev_dbg(dev, "suspend called\n");
	return i2c_dw_suspend(i2c, false);
}

static int dw_i2c_plat_runtime_suspend(struct device *dev)
{
	struct platform_device *pdev =
		container_of(dev, struct platform_device, dev);
	struct dw_i2c_dev *i2c = platform_get_drvdata(pdev);

	dev_dbg(dev, "runtime suspend called\n");
	i2c_dw_suspend(i2c, true);

	return 0;
}

static int dw_i2c_plat_resume(struct device *dev)
{
	struct platform_device *pdev =
		container_of(dev, struct platform_device, dev);
	struct dw_i2c_dev *i2c = platform_get_drvdata(pdev);

	dev_dbg(dev, "resume called\n");
	return i2c_dw_resume(i2c, false);
}

static int dw_i2c_plat_runtime_resume(struct device *dev)
{
	struct platform_device *pdev =
		container_of(dev, struct platform_device, dev);
	struct dw_i2c_dev *i2c = platform_get_drvdata(pdev);

	dev_dbg(dev, "runtime resume called\n");
	i2c_dw_resume(i2c, true);

	return 0;
}

static const struct dev_pm_ops dw_i2c_plat_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(dw_i2c_plat_suspend,
				dw_i2c_plat_resume)
	SET_RUNTIME_PM_OPS(dw_i2c_plat_runtime_suspend,
			   dw_i2c_plat_runtime_resume,
			   NULL)
};

static int __init dw_i2c_probe(struct platform_device *pdev)
{
	struct dw_i2c_dev *dev;
	struct resource *mem, *ioarea;
	const struct acpi_device_id *id;
	unsigned long start, len;
	int bus_idx = 0;
	static int bus_num;
	int irq;

#ifdef CONFIG_ACPI
	for (id = dw_i2c_acpi_ids; id->id[0]; id++)
		if (!strncmp(id->id, dev_name(&pdev->dev), strlen(id->id))) {
			bus_idx = id->driver_data + bus_num;
			bus_num++;
		}
#else
	bus_idx = pdev->id;
#endif

	/* NOTE: driver uses the static register mapping */
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem) {
		dev_err(&pdev->dev, "no mem resource?\n");
		return -EINVAL;
	}
	start = mem->start;
	len = resource_size(mem);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "no irq resource?\n");
		return irq; /* -ENXIO */
	}

	ioarea = request_mem_region(mem->start, resource_size(mem),
			pdev->name);
	if (!ioarea) {
		dev_err(&pdev->dev, "I2C region already claimed\n");
		return -EBUSY;
	}

	dev = i2c_dw_setup(&pdev->dev, bus_idx, start, len, irq);
	if (IS_ERR(dev)) {
		release_mem_region(mem->start, resource_size(mem));
		dev_err(&pdev->dev, "failed to setup i2c\n");
		return -EINVAL;
	}

	platform_set_drvdata(pdev, dev);

	acpi_i2c_register_devices(&dev->adapter);

	pm_runtime_set_active(&pdev->dev);
	pm_runtime_enable(&pdev->dev);
	pm_runtime_allow(&pdev->dev);
	pm_runtime_use_autosuspend(&pdev->dev);
	pm_runtime_set_autosuspend_delay(&pdev->dev, 50);

	return 0;
}

static int __exit dw_i2c_remove(struct platform_device *pdev)
{
	struct dw_i2c_dev *dev = platform_get_drvdata(pdev);
	struct resource *mem;

	pm_runtime_forbid(&pdev->dev);
	i2c_dw_free(&pdev->dev, dev);
	platform_set_drvdata(pdev, NULL);
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (mem)
		release_mem_region(mem->start, resource_size(mem));
	return 0;
}

/* work with hotplug and coldplug */
MODULE_ALIAS("platform:i2c_designware");

static struct platform_driver dw_i2c_driver = {
	.remove		= dw_i2c_remove,
	.driver		= {
		.name	= "i2c_designware",
		.owner	= THIS_MODULE,
		.pm     = &dw_i2c_plat_pm_ops,
#ifdef CONFIG_ACPI
		.acpi_match_table = ACPI_PTR(dw_i2c_acpi_ids),
#endif
	},
};

static int __init dw_i2c_init_driver(void)
{
	struct pci_dev *dw_pci;

	/*
	 * Try to get pci device, if exist, then exit ACPI platform
	 * register, On BYT FDK, include two enum mode: PCI, ACPI,
	 * ignore ACPI enum mode.
	 */
	dw_pci = pci_get_device(PCI_VENDOR_ID_INTEL, 0x0F41, NULL);
	if (dw_pci) {
		pr_info("DW I2C: Find I2C controller in PCI device, "
			"exit ACPI platform register!\n");
		return 0;
	}

	return platform_driver_probe(&dw_i2c_driver, dw_i2c_probe);
}
module_init(dw_i2c_init_driver);

static void __exit dw_i2c_exit_driver(void)
{
	platform_driver_unregister(&dw_i2c_driver);
}
module_exit(dw_i2c_exit_driver);

MODULE_AUTHOR("Baruch Siach <baruch@tkos.co.il>");
MODULE_DESCRIPTION("Synopsys DesignWare I2C bus adapter");
MODULE_LICENSE("GPL");
