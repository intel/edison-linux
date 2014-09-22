/**
 * intel_mcu_common.c - Intel MCU common interface file
 *
 * Copyright (C) 2014 Intel Inc. - http://www.intel.com
 *
 * Authors: Lei Wen <lei.wen@intel.com>,
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions, and the following disclaimer,
 *    without modification.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The names of the above-listed copyright holders may not be used
 *    to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") version 2, as published by the Free
 * Software Foundation.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>
#include <asm/byteorder.h>
#include <asm/unaligned.h>
#include <linux/list.h>
#include <linux/pci.h>
#include <linux/completion.h>
#include <linux/firmware.h>
#include <asm/intel_psh_ipc.h>
#include "intel_mcu_common.h"
#include <linux/circ_buf.h>

#define APP_IMR_SIZE (1024 * 126)
#define DRIVER_AUTHOR "Lei Wen <lei.wen@intel.com>"
#define DRIVER_DESC "Intel mcu common control interface"
#define INTEL_MCU_TTY_MAJOR		168
#define INTEL_MCU_TTY_MINORS		3

#define LOAD_APP		"load mcu app"
#define GET_VERSION		"get mcu app version"
struct tty_driver *intel_mcu_tty_driver;

#define VER_LEN		1024
struct mcu {
	char ver[VER_LEN];
	uintptr_t ddr_phy[2];
	void *ddr[2];
	int load_in_progress;
};

struct mcu_data {
	struct device *dev;
	struct tty_port port;
	struct mcu *mcu;
	struct completion cmp;
	struct loop_buffer lbuf;
	int index;
};

static struct mcu_data *mcu_table[INTEL_MCU_TTY_MINORS];
static int log_level = 1;
static char *debug_msg[] = {
	"fatal",
	"error",
	"warning",
	"info",
	"debug",
};

static int send_cmd(struct mcu_data *data,
		struct psh_msg *in, int ch, int wait)
{
	int ret;
	ret = intel_ia2psh_command(in, NULL, ch, 1000000);
	if (ret)
		return ret;

	if (wait) {
		ret = wait_for_completion_timeout(&data->cmp, 3 * HZ);
		if (ret == 0)
			return -ETIME;
	}

	return 0;
}

static void lbuf_read_reset(struct loop_buffer *lbuf)
{
	if (lbuf) {
		lbuf->off_head = lbuf->off_tail = 0;
		lbuf->in_reading = 0;
	}
}

static int lbuf_read_next(struct loop_buffer *lbuf, u8 **buf, u16 *size)
{
	struct frame_head *fhead =
		(struct frame_head *)(lbuf->addr + lbuf->off_head);
	*buf = NULL;
	*size = 0;

	if (lbuf->in_reading) {
		lbuf->in_reading = 0;

		/* go over previous frame has been read */
		lbuf->off_head += frame_size(fhead->length);
		lbuf->off_tail = lbuf->off_head;
		fhead = (struct frame_head *)(lbuf->addr + lbuf->off_head);
	}

	if (fhead->sign == LBUF_DISCARD_SIGN) {
		fhead = (struct frame_head *)lbuf->addr;
		lbuf->off_head = lbuf->off_tail = 0;
	}

	if (fhead->sign == LBUF_CELL_SIGN) {

		*buf = lbuf->addr + lbuf->off_head + sizeof(*fhead);
		*size = fhead->length;
		lbuf->in_reading = 1;
	}

	return !lbuf->in_reading;
}

static int intel_mcu_mcudbg_level(struct mcu_data *data, int level)
{
	struct psh_msg in;
	struct cmd_debug_param *param;

	in.param = 0;
	in.msg = CMD_MCU_APP_DEBUG;
	param = (struct cmd_debug_param *) (&(in.param));
	if (level > 0) {
		param->level = level;
		param->sub_cmd = CMD_DEBUG_SET_MASK;
	} else
		param->sub_cmd = CMD_DEBUG_GET_MASK;

	return send_cmd(data, &in, PSH2IA_CHANNEL2, 1);
}

static void push_char_into_port(struct tty_port *port, const char *buf, int len)
{
	int count;

	if (len <= 0)
		return;

	do {
		count = tty_insert_flip_string(port, buf, len);
		len -= count;
		buf += count;
	} while (len > 0);

	tty_flip_buffer_push(port);
}

static int intel_mcu_tty_open(struct tty_struct *tty, struct file *filp)
{
	dev_dbg(tty->dev, "%s\n", __func__);
	tty->driver_data = mcu_table[tty->index];
	/*
	 * For we may get data cached while we don't open this tty,
	 * so we need to flush out buffer, then we could
	 * get full content without disappoint user
	 */
	if (tty->port)
		tty_flip_buffer_push(tty->port);

	return 0;
}

static void intel_mcu_tty_close(struct tty_struct *tty, struct file *filp)
{
	dev_dbg(tty->dev, "%s\n", __func__);
	tty->driver_data = NULL;
}

static int do_get_ver(struct mcu_data *data)
{
	struct psh_msg in;

	in.param = 0;
	in.msg = CMD_MCU_APP_GET_VERSION;
	return send_cmd(data, &in, PSH2IA_CHANNEL2, 1);
}

static int do_setup_ddr(struct mcu_data *data)
{
	struct mcu *mcu = data->mcu;
	const struct firmware *fw_entry;
	static int fw_load_done;
	char fname[20];
	struct psh_msg in;

	if (fw_load_done)
		return 0;

	snprintf(fname, 20, "intel_mcu.bin");
	if (!request_firmware(&fw_entry, fname, data->dev)) {
		if (!fw_entry)
			return -ENOMEM;

		pr_debug("psh fw size %d virt:0x%p\n",
				(int)fw_entry->size, fw_entry->data);
		if (fw_entry->size > APP_IMR_SIZE) {
			pr_err("psh fw size too big\n");
		} else {
			memcpy(mcu->ddr[0], fw_entry->data,
					fw_entry->size);
			in.msg = CMD_MCU_LOAD_APP;
			in.param = mcu->ddr_phy[0];
			mcu->load_in_progress = 1;
			if (send_cmd(data, &in, PSH2IA_CHANNEL3, 1))
				return -1;
			fw_load_done = 1;
		}
		release_firmware(fw_entry);
	} else {
		pr_err("cannot find psh firmware(%s)\n", fname);
		return -ENODEV;
	}
	in.msg = CMD_MCU_SETUP_DDR;
	in.param = mcu->ddr_phy[1];
	return send_cmd(data, &in, PSH2IA_CHANNEL2, 1);
}

static ssize_t load_app_store(struct device *device,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int len = strlen(LOAD_APP);

	if (count >= len && strncmp(buf, LOAD_APP, len) == 0) {
		do_setup_ddr(mcu_table[2]);
		return count;
	}

	pr_err("Please provide right string as [%s]!\n", LOAD_APP);
	return -1;
}

static ssize_t get_ver_show(struct device *device,
		struct device_attribute *attr, char *buf)
{
	struct mcu_data *data = mcu_table[2];
	struct mcu *mcu = data->mcu;

	if (do_get_ver(data))
		return -1;

	return scnprintf(buf, VER_LEN, "%s", mcu->ver);
}

static ssize_t mdbg_control_show(struct device *device,
		struct device_attribute *attr, char *buf)
{
	if (intel_mcu_mcudbg_level(mcu_table[2], -1) < 0)
		goto err;

	if (log_level > 0 && log_level < 6)
		return scnprintf(buf, 8, "%s\n", debug_msg[log_level - 1]);

err:
	pr_info("get log level err\n");
	return -1;
}
/*
 *set msg level:echo log_level=fatal|info|warning|error|debug| >control
*/
#define LOG_LEVEL	"fatal|error|warning|info|debug"
static ssize_t mdbg_control_store(struct device *device,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct mcu_data *data = mcu_table[2];
	int level = 0;
	long ltmp = 0;

	if (!buf)
		return -1;
	if (!strncmp(buf, "fatal", strlen("fatal")))
		level = 1;
	else if (!strncmp(buf, "error", strlen("error")))
		level = 2;
	else if (!strncmp(buf, "warning", strlen("warning")))
		level = 3;
	else if (!strncmp(buf, "info", strlen("info")))
		level = 4;
	else if (!strncmp(buf, "debug", strlen("debug")))
		level = 5;
	else {
		int err;
		err = kstrtol(buf, 10, &ltmp);
		if (!err && (ltmp > 0) && (ltmp < 6))
			level = ltmp;
		else {
			pr_err("Please input words as [%s]\n", LOG_LEVEL);
			return -1;
		}
	}
	pr_info("set level:%d\n", level);
	if (intel_mcu_mcudbg_level(data, level) < 0)
		return -1;
	return count;
}

static DEVICE_ATTR(control, 0200, NULL, load_app_store);
static DEVICE_ATTR(fw_version, 0400, get_ver_show, NULL);
static DEVICE_ATTR(log_level, 0600, mdbg_control_show, mdbg_control_store);

static struct attribute *control_sysfs_attrs[] = {
	&dev_attr_control.attr,
	&dev_attr_fw_version.attr,
	&dev_attr_log_level.attr,
	NULL,

};

static struct attribute_group intel_mcu_tty_attribute_group = {
	.name = NULL,
	.attrs = control_sysfs_attrs,

};

static void raw_output(struct mcu_data *data, int ch,
		const unsigned char *buf, int count)
{
	struct psh_msg in;
	int i, left;

	for (i = 0; i < count; i += 4) {
		left = count - i;
		if (left > 4) {
			left = 4;
			in.msg = PSH_IPC_CONTINUE;
		} else
			in.msg = 0;

		memcpy(&in.param, buf, left);
		buf += left;
		send_cmd(data, &in, ch, 0);
	}
}

#define TTY_WRITE_ROOM		512
static int intel_mcu_tty_write(struct tty_struct *tty,
		const unsigned char *buf, int count)
{
	struct mcu_data *data = tty->driver_data;

	switch (tty->index) {
	default:
		pr_err("TTY index %d not supported!\n", tty->index);
	case 1:
		return -1;
	case 0:
		if (count > TTY_WRITE_ROOM) {
			pr_err("Port 0's input size is limited by %d!\n",
					TTY_WRITE_ROOM);
			return -1;
		}
		raw_output(data, tty->index, buf, count);
		break;
	}
	return count;
}

static int intel_mcu_tty_write_room(struct tty_struct *tty)
{
	return TTY_WRITE_ROOM;
}

static const struct tty_operations intel_mcu_ops = {
	.open =			intel_mcu_tty_open,
	.close =		intel_mcu_tty_close,
	.write =		intel_mcu_tty_write,
	.write_room =		intel_mcu_tty_write_room,
};

static int mem_alloc(struct pci_dev *pdev, uintptr_t *phy_addr,
		void **virt_addr, int bar)
{
	void __iomem *mem;
	int ret = 0;
	unsigned long start = 0, len;

	/* dedicate isolated memory region */
	start = pci_resource_start(pdev, bar);
	len = pci_resource_len(pdev, bar);
	if (!start || !len) {
		dev_err(&pdev->dev, "bar %d address not set\n", bar);
		ret = -EINVAL;
		goto err;
	}

	ret = pci_request_region(pdev, bar, "intel_mcu");
	if (ret) {
		dev_err(&pdev->dev,
				"failed to request psh region 0x%lx-0x%lx\n",
				start,
				(unsigned long)pci_resource_end(pdev, bar));
		goto err;
	}

	mem = ioremap_nocache(start, len);
	if (!mem) {
		dev_err(&pdev->dev, "can not ioremap app imr address\n");
		ret = -EINVAL;
		goto err_ioremap;
	}

	*phy_addr = start;
	*virt_addr = (void *)mem;
	return 0;

err_ioremap:
	pci_release_region(pdev, bar);
err:
	return ret;
}

static void cmd_handler(u32 msg, u32 param, void *_data)
{
	struct mcu_data *data = (struct mcu_data *)_data;
	struct mcu *mcu = data->mcu;
	struct cmd_resp *resp;
	const struct version_resp *version;
	struct debug_resp *debug_resp;
	u8 *dbuf = NULL;
	u16 size = 0;

	if (mcu->load_in_progress) {
		mcu->load_in_progress = 0;
		goto done;
	}

	while (!lbuf_read_next(&data->lbuf, &dbuf, &size)) {
		resp = (struct cmd_resp *)dbuf;

		if (!resp->len)
			continue;

		switch (resp->cmd_id) {
		case CMD_MCU_APP_GET_VERSION:
			version = (struct version_resp *)resp->param;
			if (version->total_length)
				snprintf(mcu->ver, VER_LEN, version->buf,
						version->total_length);
			break;
		case CMD_MCU_APP_DEBUG:
			debug_resp = (struct debug_resp *)resp->param;
			log_level = debug_resp->level;
		default:
			break;
		}
	}
done:
	complete(&data->cmp);
}

static void raw_data_handler(u32 msg, u32 param, void *_data)
{
	struct mcu_data *data = (struct mcu_data *)_data;
	struct cmd_resp *resp;
	u8 *dbuf = NULL;
	u16 size = 0;

	while (!lbuf_read_next(&data->lbuf, &dbuf, &size)) {
		resp = (struct cmd_resp *)dbuf;
		push_char_into_port(&data->port, resp->param, resp->len);
	}
	complete(&data->cmp);
}

static int mcu_platform_probe(struct platform_device *pdev)
{
	int ret, i;
	struct mcu_data *data;
	struct mcu *mcu;
	u8 *base;

	mcu = platform_get_drvdata(pdev);
	intel_mcu_tty_driver = alloc_tty_driver(INTEL_MCU_TTY_MINORS);
	if (!intel_mcu_tty_driver) {
		dev_err(&pdev->dev, "fail to alloc tty driver\n");
		return -ENODEV;
	}

	intel_mcu_tty_driver->name = "ttymcu";
	intel_mcu_tty_driver->major = INTEL_MCU_TTY_MAJOR;
	intel_mcu_tty_driver->minor_start = 0;
	intel_mcu_tty_driver->type = TTY_DRIVER_TYPE_SERIAL;
	intel_mcu_tty_driver->subtype = SERIAL_TYPE_NORMAL;
	intel_mcu_tty_driver->flags = TTY_DRIVER_REAL_RAW
		| TTY_DRIVER_DYNAMIC_DEV;
	intel_mcu_tty_driver->init_termios = tty_std_termios;
	intel_mcu_tty_driver->init_termios.c_cflag = B9600 | CS8 | CREAD |
		HUPCL | CLOCAL;
	intel_mcu_tty_driver->init_termios.c_ispeed = 38400;
	intel_mcu_tty_driver->init_termios.c_ospeed = 38400;
	intel_mcu_tty_driver->init_termios.c_iflag = 0;
	intel_mcu_tty_driver->init_termios.c_oflag = 0;
	intel_mcu_tty_driver->init_termios.c_lflag = 0;
	tty_set_operations(intel_mcu_tty_driver, &intel_mcu_ops);

	ret = tty_register_driver(intel_mcu_tty_driver);
	if (ret) {
		dev_err(&pdev->dev, "fail to register tty driver\n");
		goto tty_reg_fail;
	}

	base = (u8 *)mcu->ddr[1];
	for (i = INTEL_MCU_TTY_MINORS - 1; i >= 0; i--) {
		data = kzalloc(sizeof(struct mcu_data), GFP_KERNEL);
		if (data == NULL) {
			dev_err(&pdev->dev, "fail to alloc mcu data\n");
			goto data_alloc_fail;
		}

		data->index = i;
		tty_port_init(&data->port);
		data->dev = tty_port_register_device(&data->port,
				intel_mcu_tty_driver, i, &pdev->dev);
		mcu_table[i] = data;
		data->mcu = mcu;
		init_completion(&data->cmp);
		data->lbuf.addr = base;
		data->lbuf.length = BUF_IA_DDR_SIZE;
		lbuf_read_reset(&data->lbuf);
		base += BUF_IA_DDR_SIZE;
	}
	ret = sysfs_create_group(&pdev->dev.kobj,
			&intel_mcu_tty_attribute_group);
	if (ret) {
		pr_err("failed to create the mdbg sysfs attributes\n");
		sysfs_remove_group(&pdev->dev.kobj,
				&intel_mcu_tty_attribute_group);
		goto data_alloc_fail;
	}

	intel_psh_ipc_bind(PSH_RECV_CH0, raw_data_handler, mcu_table[0]);
	intel_psh_ipc_bind(PSH_RECV_CH1, raw_data_handler, mcu_table[1]);
	intel_psh_ipc_bind(PSH_RECV_CH2, cmd_handler, mcu_table[2]);

	pr_info("MCU detected and ready to used!\n");

	return 0;

data_alloc_fail:
	for (i = 0; i < INTEL_MCU_TTY_MINORS; i++)
		kfree(mcu_table[i]);
tty_reg_fail:
	put_tty_driver(intel_mcu_tty_driver);
	return ret;
}

static int mcu_platform_remove(struct platform_device *pdev)
{
	struct mcu *mcu;
	int i;

	mcu = platform_get_drvdata(pdev);
	sysfs_remove_group(&pdev->dev.kobj,
			&intel_mcu_tty_attribute_group);

	for (i = 0; i < INTEL_MCU_TTY_MINORS; i++)
		kfree(mcu_table[i]);
	put_tty_driver(intel_mcu_tty_driver);
	kfree(mcu);

	return 0;
}

static struct platform_driver intel_mcu_platform = {
	.driver = {
		.name	= "intel_mcu",
	},
	.probe		= mcu_platform_probe,
	.remove		= mcu_platform_remove,
};
module_platform_driver(intel_mcu_platform);

static int intel_mcu_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	struct platform_device *dev;
	struct mcu *mcu;
	int ret;

	ret = pci_enable_device(pdev);
	if (ret) {
		dev_err(&pdev->dev, "fail to enable psh pci device\n");
		return -ENODEV;
	}

	mcu = kzalloc(sizeof(struct mcu), GFP_KERNEL);
	if (!mcu) {
		dev_err(&pdev->dev, "cannot allocate memory for mcu\n");
		ret = -ENOMEM;
		goto mcu_err;
	}

	ret = mem_alloc(pdev, &mcu->ddr_phy[0], &mcu->ddr[0], 0);
	if (ret)
		goto plat_alloc_fail;

	ret = mem_alloc(pdev, &mcu->ddr_phy[1], &mcu->ddr[1], 1);
	if (ret)
		goto plat_alloc_fail;

	dev = platform_device_alloc("intel_mcu", -1);
	if (!dev) {
		ret = -ENODEV;
		goto plat_alloc_fail;
	}

	dev->dev.dma_mask = &dev->dev.coherent_dma_mask;
	platform_set_drvdata(dev, mcu);
	dev_set_drvdata(&pdev->dev, mcu);

	ret = platform_device_add(dev);
	return ret;

plat_alloc_fail:
	kfree(mcu);
mcu_err:
	pci_dev_put(pdev);
	return ret;
}

static void intel_mcu_remove(struct pci_dev *pdev)
{
	struct mcu *mcu;

	mcu = dev_get_drvdata(&pdev->dev);
	iounmap((void __iomem *)mcu->ddr[0]);
	iounmap((void __iomem *)mcu->ddr[1]);

	pci_release_region(pdev, 0);
	pci_release_region(pdev, 1);
	pci_dev_put(pdev);
}

static DEFINE_PCI_DEVICE_TABLE(pci_ids) = {
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL, 0x11a4)},
	{ 0,}
};

MODULE_DEVICE_TABLE(pci, pci_ids);
static struct pci_driver intel_mcu_driver = {
	.name = "intel_mcu",
	.id_table = pci_ids,
	.probe	= intel_mcu_probe,
	.remove	= intel_mcu_remove,
};

static int __init intel_mcu_init(void)
{
	return pci_register_driver(&intel_mcu_driver);
}

static void __exit intel_mcu_exit(void)
{
	pci_unregister_driver(&intel_mcu_driver);
}

module_init(intel_mcu_init);
module_exit(intel_mcu_exit);
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
MODULE_ALIAS_CHARDEV_MAJOR(INTEL_MCU_TTY_MAJOR);
