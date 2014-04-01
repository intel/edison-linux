/*
 * Synopsys DesignWare I2C adapter driver (master only).
 *
 * Based on the TI DAVINCI I2C adapter driver.
 *
 * Copyright (C) 2006 Texas Instruments.
 * Copyright (C) 2007 MontaVista Software Inc.
 * Copyright (C) 2009 Provigent Ltd.
 * Copyright (C) 2011 Intel corporation.
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
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/pci.h>
#include <linux/pm_runtime.h>
#include <linux/semaphore.h>
#include <linux/fs.h>
#include <linux/acpi.h>
#include "i2c-designware-core.h"

#define DRIVER_NAME "i2c-designware-pci"
#define DW_I2C_STATIC_BUS_NUM	10

static int i2c_dw_pci_suspend(struct device *dev)
{
	struct pci_dev *pdev = container_of(dev, struct pci_dev, dev);
	struct dw_i2c_dev *i2c = pci_get_drvdata(pdev);

	dev_dbg(dev, "suspend called\n");

	return i2c_dw_suspend(i2c, false);
}

static int i2c_dw_pci_runtime_suspend(struct device *dev)
{
	struct pci_dev *pdev = container_of(dev, struct pci_dev, dev);
	struct dw_i2c_dev *i2c = pci_get_drvdata(pdev);
	int err;

	dev_dbg(dev, "runtime suspend called\n");
	i2c_dw_suspend(i2c, true);

	err = pci_save_state(pdev);
	if (err) {
		dev_err(&pdev->dev, "pci_save_state failed\n");
		return err;
	}

	err = pci_set_power_state(pdev, PCI_D3hot);
	if (err) {
		dev_err(&pdev->dev, "pci_set_power_state failed\n");
		return err;
	}

	return 0;
}

static int i2c_dw_pci_resume(struct device *dev)
{
	struct pci_dev *pdev = container_of(dev, struct pci_dev, dev);
	struct dw_i2c_dev *i2c = pci_get_drvdata(pdev);

	dev_dbg(dev, "resume called\n");
	return i2c_dw_resume(i2c, false);
}

static int i2c_dw_pci_runtime_resume(struct device *dev)
{
	struct pci_dev *pdev = container_of(dev, struct pci_dev, dev);
	struct dw_i2c_dev *i2c = pci_get_drvdata(pdev);
	int err;

	dev_dbg(dev, "runtime resume called\n");
	err = pci_set_power_state(pdev, PCI_D0);
	if (err) {
		dev_err(&pdev->dev, "pci_set_power_state() failed\n");
		return err;
	}
	pci_restore_state(pdev);
	i2c_dw_resume(i2c, true);

	return 0;
}

static const struct dev_pm_ops i2c_dw_pm_ops = {
	.suspend_late = i2c_dw_pci_suspend,
	.resume_early = i2c_dw_pci_resume,
	SET_RUNTIME_PM_OPS(i2c_dw_pci_runtime_suspend,
			   i2c_dw_pci_runtime_resume,
			   NULL)
};

static int i2c_dw_pci_probe(struct pci_dev *pdev,
const struct pci_device_id *id)
{
	struct dw_i2c_dev *dev;
	unsigned long start, len;
	int r;
	int bus_idx;
	static int bus_num;

	bus_idx = id->driver_data + bus_num;
	bus_num++;

	r = pci_enable_device(pdev);
	if (r) {
		dev_err(&pdev->dev, "Failed to enable I2C PCI device (%d)\n",
			r);
		return r;
	}

	/* Determine the address of the I2C area */
	start = pci_resource_start(pdev, 0);
	len = pci_resource_len(pdev, 0);
	if (!start || len == 0) {
		dev_err(&pdev->dev, "base address not set\n");
		return -ENODEV;
	}

	r = pci_request_region(pdev, 0, DRIVER_NAME);
	if (r) {
		dev_err(&pdev->dev, "failed to request I2C region "
			"0x%lx-0x%lx\n", start,
			(unsigned long)pci_resource_end(pdev, 0));
		return r;
	}

	dev = i2c_dw_setup(&pdev->dev, bus_idx, start, len, pdev->irq);
	if (IS_ERR(dev)) {
		pci_release_region(pdev, 0);
		dev_err(&pdev->dev, "failed to setup i2c\n");
		return -EINVAL;
 	}

	pci_set_drvdata(pdev, dev);

	i2c_acpi_devices_setup(&pdev->dev, dev);

	pm_runtime_set_autosuspend_delay(&pdev->dev, 50);
	pm_runtime_use_autosuspend(&pdev->dev);
	pm_runtime_put_noidle(&pdev->dev);
	pm_runtime_allow(&pdev->dev);

	return 0;
}

static void i2c_dw_pci_remove(struct pci_dev *pdev)
{
	struct dw_i2c_dev *dev = pci_get_drvdata(pdev);

	pm_runtime_forbid(&pdev->dev);
	i2c_dw_free(&pdev->dev, dev);
	pci_set_drvdata(pdev, NULL);
	pci_release_region(pdev, 0);
}

/* work with hotplug and coldplug */
MODULE_ALIAS("i2c_designware-pci");

DEFINE_PCI_DEVICE_TABLE(i2c_designware_pci_ids) = {
	/* Moorestown */
	{ PCI_VDEVICE(INTEL, 0x0802), moorestown_0 },
	{ PCI_VDEVICE(INTEL, 0x0803), moorestown_0 },
	{ PCI_VDEVICE(INTEL, 0x0804), moorestown_0 },
	/* Medfield */
	{ PCI_VDEVICE(INTEL, 0x0817), medfield_0 },
	{ PCI_VDEVICE(INTEL, 0x0818), medfield_0 },
	{ PCI_VDEVICE(INTEL, 0x0819), medfield_0 },
	{ PCI_VDEVICE(INTEL, 0x082C), medfield_0 },
	{ PCI_VDEVICE(INTEL, 0x082D), medfield_0 },
	{ PCI_VDEVICE(INTEL, 0x082E), medfield_0 },
	/* Cloverview */
	{ PCI_VDEVICE(INTEL, 0x08E2), cloverview_0 },
	{ PCI_VDEVICE(INTEL, 0x08E3), cloverview_0 },
	{ PCI_VDEVICE(INTEL, 0x08E4), cloverview_0 },
	{ PCI_VDEVICE(INTEL, 0x08F4), cloverview_0 },
	{ PCI_VDEVICE(INTEL, 0x08F5), cloverview_0 },
	{ PCI_VDEVICE(INTEL, 0x08F6), cloverview_0 },
	/* Merrifield */
	{ PCI_VDEVICE(INTEL, 0x1195), merrifield_0 },
	{ PCI_VDEVICE(INTEL, 0x1196), merrifield_0 },
	/* Valleyview 2 */
	{ PCI_VDEVICE(INTEL, 0x0F41), valleyview_0 },
	{ PCI_VDEVICE(INTEL, 0x0F42), valleyview_0 },
	{ PCI_VDEVICE(INTEL, 0x0F43), valleyview_0 },
	{ PCI_VDEVICE(INTEL, 0x0F44), valleyview_0 },
	{ PCI_VDEVICE(INTEL, 0x0F45), valleyview_0 },
	{ PCI_VDEVICE(INTEL, 0x0F46), valleyview_0 },
	{ PCI_VDEVICE(INTEL, 0x0F47), valleyview_0 },
	/* Cherryview */
	{ PCI_VDEVICE(INTEL, 0x22C1), cherryview_0 },
	{ PCI_VDEVICE(INTEL, 0x22C2), cherryview_0 },
	{ PCI_VDEVICE(INTEL, 0x22C3), cherryview_0 },
	{ PCI_VDEVICE(INTEL, 0x22C4), cherryview_0 },
	{ PCI_VDEVICE(INTEL, 0x22C5), cherryview_0 },
	{ PCI_VDEVICE(INTEL, 0x22C6), cherryview_0 },
	{ PCI_VDEVICE(INTEL, 0x22C7), cherryview_0 },
	{ 0,}
};
MODULE_DEVICE_TABLE(pci, i2c_designware_pci_ids);

static struct pci_driver dw_i2c_driver = {
	.name		= DRIVER_NAME,
	.id_table	= i2c_designware_pci_ids,
	.probe		= i2c_dw_pci_probe,
	.remove		= i2c_dw_pci_remove,
	.driver         = {
		.pm     = &i2c_dw_pm_ops,
	},
};

static int __init dw_i2c_init_driver(void)
{
	return  pci_register_driver(&dw_i2c_driver);
}
module_init(dw_i2c_init_driver);

static void __exit dw_i2c_exit_driver(void)
{
	pci_unregister_driver(&dw_i2c_driver);
}
module_exit(dw_i2c_exit_driver);

#ifndef MODULE
static int __init dw_i2c_reserve_static_bus(void)
{
	struct i2c_board_info dummy = {
		I2C_BOARD_INFO("dummy", 0xff),
	};

	i2c_register_board_info(DW_I2C_STATIC_BUS_NUM, &dummy, 1);
	return 0;
}
subsys_initcall(dw_i2c_reserve_static_bus);

static void dw_i2c_pci_final_quirks(struct pci_dev *pdev)
{
	pdev->pm_cap = 0x80;
}

DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_INTEL, 0x0F44,
				dw_i2c_pci_final_quirks);
#endif

MODULE_AUTHOR("Baruch Siach <baruch@tkos.co.il>");
MODULE_DESCRIPTION("Synopsys DesignWare PCI I2C bus adapter");
MODULE_LICENSE("GPL");
