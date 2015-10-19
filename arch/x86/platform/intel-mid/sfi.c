/*
 * intel_mid_sfi.c: Intel MID SFI initialization code
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
#include <linux/scatterlist.h>
#include <linux/sfi.h>
#include <linux/intel_pmic_gpio.h>
#include <linux/spi/spi.h>
#include <linux/i2c.h>
#include <linux/skbuff.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/mmc/core.h>
#include <linux/mmc/card.h>
#include <linux/blkdev.h>

#include <asm/setup.h>
#include <asm/mpspec_def.h>
#include <asm/hw_irq.h>
#include <asm/apic.h>
#include <asm/io_apic.h>
#include <asm/intel-mid.h>
#include <asm/intel_mid_vrtc.h>
#include <asm/io.h>
#include <asm/i8259.h>
#include <asm/intel_scu_ipc.h>
#include <asm/apb_timer.h>
#include <asm/reboot.h>

#define	SFI_SIG_OEM0	"OEM0"
#define MAX_IPCDEVS	24
#define MAX_SCU_SPI	24
#define MAX_SCU_I2C	24

static struct platform_device *ipc_devs[MAX_IPCDEVS];
static struct spi_board_info *spi_devs[MAX_SCU_SPI];
static struct i2c_board_info *i2c_devs[MAX_SCU_I2C];
static struct sfi_gpio_table_entry *gpio_table;
static struct sfi_timer_table_entry sfi_mtimer_array[SFI_MTMR_MAX_NUM];
static int ipc_next_dev;
static int spi_next_dev;
static int i2c_next_dev;
static int i2c_bus[MAX_SCU_I2C];
static int gpio_num_entry;
static u32 sfi_mtimer_usage[SFI_MTMR_MAX_NUM];

#define intel_mid_sfi_get_pdata(dev, priv)	\
	((dev)->get_platform_data ? (dev)->get_platform_data(priv) : NULL)

/*
 * Parsing GPIO table first, since the DEVS table will need this table
 * to map the pin name to the actual pin.
 */
static int __init sfi_parse_gpio(struct sfi_table_header *table)
{
	struct sfi_table_simple *sb;
	struct sfi_gpio_table_entry *pentry;
	int num, i;

	if (gpio_table)
		return 0;
	sb = (struct sfi_table_simple *)table;
	num = SFI_GET_NUM_ENTRIES(sb, struct sfi_gpio_table_entry);
	pentry = (struct sfi_gpio_table_entry *)sb->pentry;

	gpio_table = kmalloc(num * sizeof(*pentry), GFP_KERNEL);
	if (!gpio_table)
		return -1;
	memcpy(gpio_table, pentry, num * sizeof(*pentry));
	gpio_num_entry = num;

	pr_debug("GPIO pin info:\n");
	for (i = 0; i < num; i++, pentry++)
		pr_debug("info[%2d]: controller = %16.16s, pin_name = %16.16s,"
		" pin = %d\n", i,
			pentry->controller_name,
			pentry->pin_name,
			pentry->pin_no);
	return 0;
}

static void __init intel_scu_i2c_device_register(int bus,
						struct i2c_board_info *idev)
{
	struct i2c_board_info *new_dev;

	if (i2c_next_dev == MAX_SCU_I2C) {
		pr_err("too many SCU I2C devices");
		return;
	}

	new_dev = kzalloc(sizeof(*idev), GFP_KERNEL);
	if (!new_dev) {
		pr_err("failed to alloc mem for delayed i2c dev %s\n",
			idev->type);
		return;
	}
	*new_dev = *idev;

	i2c_bus[i2c_next_dev] = bus;
	i2c_devs[i2c_next_dev++] = new_dev;
}

static void __init sfi_handle_ipc_dev(struct sfi_device_table_entry *pentry,
					struct devs_id *dev)
{
	struct platform_device *pdev;
	void *pdata = NULL;

	pr_debug("IPC bus, name = %16.16s, irq = 0x%2x\n",
		pentry->name, pentry->irq);
	pdata = intel_mid_sfi_get_pdata(dev, pentry);
	if (IS_ERR(pdata))
		return;

	pdev = platform_device_alloc(pentry->name, 0);
	if (pdev == NULL) {
		pr_err("out of memory for SFI platform device '%s'.\n",
			pentry->name);
		return;
	}
	install_irq_resource(pdev, pentry->irq);

	pdev->dev.platform_data = pdata;
	platform_device_add(pdev);
}

static void __init sfi_handle_i2c_dev(struct sfi_device_table_entry *pentry,
					struct devs_id *dev)
{
	struct i2c_board_info i2c_info;
	void *pdata = NULL;

	memset(&i2c_info, 0, sizeof(i2c_info));
	strncpy(i2c_info.type, pentry->name, SFI_NAME_LEN);
	i2c_info.irq = ((pentry->irq == (u8)0xff) ? 0 : pentry->irq);
	i2c_info.addr = pentry->addr;
	pr_debug("I2C bus = %d, name = %16.16s, irq = 0x%2x, addr = 0x%x\n",
		pentry->host_num,
		i2c_info.type,
		i2c_info.irq,
		i2c_info.addr);
	pdata = intel_mid_sfi_get_pdata(dev, &i2c_info);
	i2c_info.platform_data = pdata;
	if (IS_ERR(pdata))
		return;

	if (dev->delay)
		intel_scu_i2c_device_register(pentry->host_num, &i2c_info);
	else
		i2c_register_board_info(pentry->host_num, &i2c_info, 1);
}

extern struct devs_id *const __x86_intel_mid_dev_start[],
		      *const __x86_intel_mid_dev_end[];
