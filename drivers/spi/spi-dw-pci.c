/*
 * PCI interface driver for DW SPI Core
 *
 * Copyright (c) 2009, 2014 Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <linux/interrupt.h>
#include <linux/pci.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>
#include <linux/spi/spi.h>
#include <linux/module.h>

#include "spi-dw.h"

#define DRIVER_NAME "dw_spi_pci"

struct dw_spi_pci {
	struct pci_dev	*pdev;
	struct dw_spi	dws;
};

struct spi_pci_desc {
	int	(*setup)(struct dw_spi *);
};

static struct spi_pci_desc spi_pci_mid_desc = {
	.setup = dw_spi_mid_init,
};

static int spi_pci_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
	struct dw_spi_pci *dwpci;
	struct dw_spi *dws;
	struct spi_pci_desc *desc = (struct spi_pci_desc *)ent->driver_data;
	int pci_bar = 0;
	int ret;

	ret = pcim_enable_device(pdev);
	if (ret)
		return ret;

	dwpci = devm_kzalloc(&pdev->dev, sizeof(struct dw_spi_pci),
			GFP_KERNEL);
	if (!dwpci)
		return -ENOMEM;

	dwpci->pdev = pdev;
	dws = &dwpci->dws;

	/* Get basic io resource and map it */
	dws->paddr = pci_resource_start(pdev, pci_bar);

	ret = pcim_iomap_regions(pdev, 1 << pci_bar, pci_name(pdev));
	if (ret)
		return ret;

	dws->regs = pcim_iomap_table(pdev)[pci_bar];

	dws->bus_num = ent->driver_data;
	dws->num_cs = 4;
	dws->irq = pdev->irq;

	/*
	 * Specific handling for paltforms, like dma setup,
	 * clock rate, FIFO depth.
	 */
	if (desc && desc->setup) {
		ret = desc->setup(dws);
		if (ret)
			return ret;
	}

	ret = dw_spi_add_host(&pdev->dev, dws);
	if (ret)
		return ret;

	/* PCI hook and SPI hook use the same drv data */
	pci_set_drvdata(pdev, dwpci);

	pm_suspend_ignore_children(&pdev->dev, true);
	pm_runtime_put_noidle(&pdev->dev);
	pm_runtime_allow(&pdev->dev);

	dev_info(&pdev->dev, "found PCI SPI controller(ID: %04x:%04x)\n",
		pdev->vendor, pdev->device);

	return 0;
}

static void spi_pci_remove(struct pci_dev *pdev)
{
	struct dw_spi_pci *dwpci = pci_get_drvdata(pdev);

	dw_spi_remove_host(&dwpci->dws);
	pm_runtime_forbid(&pdev->dev);
	pm_runtime_get_noresume(&pdev->dev);
}

#ifdef CONFIG_PM_SLEEP
static int spi_suspend(struct device *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct dw_spi_pci *dwpci = pci_get_drvdata(pdev);

	return dw_spi_suspend_host(&dwpci->dws);
}

static int spi_resume(struct device *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct dw_spi_pci *dwpci = pci_get_drvdata(pdev);

	return dw_spi_resume_host(&dwpci->dws);
}

static int spi_dw_pci_runtime_suspend(struct device *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct dw_spi_pci *dwpci = pci_get_drvdata(pdev);
 
	dev_dbg(dev, "PCI runtime suspend called\n");
	return dw_spi_suspend_host(&dwpci->dws);
}

static int spi_dw_pci_runtime_resume(struct device *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct dw_spi_pci *dwpci = pci_get_drvdata(pdev);
 
	dev_dbg(dev, "pci_runtime_resume called\n");
	return dw_spi_resume_host(&dwpci->dws);
}

static int spi_dw_pci_runtime_idle(struct device *dev)
{
	int err;
 
	dev_dbg(dev, "pci_runtime_idle called\n");
	if (system_state == SYSTEM_BOOTING)
		/* if SPI UART is set as default console and earlyprintk
		 * is enabled, it cannot shutdown SPI controller during booting.
		 */
		err = pm_schedule_suspend(dev, 30000);
	else
		err = pm_schedule_suspend(dev, 500);
 
	if (err != 0)
		return 0;
 
	return -EBUSY;
}

#endif

static const struct pci_device_id pci_ids[] = {
	/* Intel MID platform SPI controller 0 */
	{ PCI_VDEVICE(INTEL, 0x0800), (kernel_ulong_t)&spi_pci_mid_desc},
	/* Intel Medfield platform SPI controller 1 */
	{ PCI_DEVICE(PCI_VENDOR_ID_INTEL, 0x0800), .driver_data = 0 },
	/* Intel Cloverview platform SPI controller 1 */
	{ PCI_DEVICE(PCI_VENDOR_ID_INTEL, 0x08E1), .driver_data = 0 },
	{ PCI_DEVICE(PCI_VENDOR_ID_INTEL, 0x08EE), .driver_data = 1 },
	/* Intel EVx platform SPI controller 1 */
	{ PCI_DEVICE(PCI_VENDOR_ID_INTEL, 0x0812), .driver_data = 2 },
	{},
};

static const struct dev_pm_ops dw_spi_pm_ops = {
	.suspend = spi_suspend,
	.resume = spi_resume,
	.runtime_suspend = spi_dw_pci_runtime_suspend,
	.runtime_resume = spi_dw_pci_runtime_resume,
	.runtime_idle = spi_dw_pci_runtime_idle,
};

static struct pci_driver dw_spi_driver = {
	.name =		DRIVER_NAME,
	.id_table =	pci_ids,
	.probe =	spi_pci_probe,
	.remove =	spi_pci_remove,
	.driver         = {
		.pm     = &dw_spi_pm_ops,
	},
};

static int __init mrst_spi_init(void)
{
	return pci_register_driver(&dw_spi_driver);
}

static void __exit mrst_spi_exit(void)
{
	pci_unregister_driver(&dw_spi_driver);
}

module_init(mrst_spi_init);
module_exit(mrst_spi_exit);

MODULE_AUTHOR("Feng Tang <feng.tang@intel.com>");
MODULE_DESCRIPTION("PCI interface driver for DW SPI Core");
MODULE_LICENSE("GPL v2");
