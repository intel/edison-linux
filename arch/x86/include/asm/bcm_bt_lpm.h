/*
 * Copyright (C) 2009 Google, Inc.
*
* This software is licensed under the terms of the GNU General Public
* License version 2, as published by the Free Software Foundation, and
* may be copied, distributed, and modified under those terms.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*/

#ifndef BCM_BT_LMP_H
#define BCM_BT_LMP_H

#include <linux/serial_core.h>
#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci_core.h>

/* Uart driver must call this every time it beings TX, to ensure
* this driver keeps WAKE asserted during TX. Called with uart
* spinlock held. */
extern void bcm_bt_lpm_exit_lpm_locked(struct device *dev,
							struct hci_dev *hdev);

struct bcm_bt_lpm_platform_data {
	int gpio_wake;		/* CPU -> BCM wakeup gpio */
	int gpio_host_wake;	/* BCM -> CPU wakeup gpio */
	int int_host_wake;	/* BCM -> CPU wakeup irq */
	int gpio_enable;	/* GPIO enable/disable BT/FM */

	int port;			/* UART port to use with BT/FM */
	/*
	 * Callback to request the uart driver to clock off.
	 * Called with uart spinlock held.
	 */
	void (*uart_disable)(struct device *tty);
	/*
	 * Callback to request the uart driver to clock on.
	 * Called with uart spinlock held.
	 */
	void (*uart_enable)(struct device *tty);
};

#endif
