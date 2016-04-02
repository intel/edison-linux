/*
 * platform_ipc.c: IPC platform library file
 *
 * (C) Copyright 2013 Intel Corporation
 * Author: Sathyanarayanan Kuppuswamy <sathyanarayanan.kuppuswamy@intel.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/sfi.h>
#include <linux/gpio.h>
#include <asm/intel-mid.h>
#include "platform_ipc.h"

void __init ipc_device_handler(struct sfi_device_table_entry *pentry,
				struct devs_id *dev)
{
	void *pdata = NULL;

	pr_debug("IPC bus, name = %16.16s, irq = 0x%2x\n",
		pentry->name, pentry->irq);

	/*
	 * We need to call platform init of IPC devices to fill misc_pdata
	 * structure. It will be used in msic_init for initialization.
	 */
	if (dev != NULL)
		pdata = dev->get_platform_data(pentry);
}

static const struct devs_id pmic_audio_dev_id __initconst = {
	.name = "pmic_audio",
	.type = SFI_DEV_TYPE_IPC,
	.delay = 1,
	.device_handler = &ipc_device_handler,
};

sfi_device(pmic_audio_dev_id);
