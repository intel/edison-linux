/*
 * iio_basincove_gpadc.c - Intel Merrifield Basin Cove GPADC Driver
 *
 * Copyright (C) 2012 Intel Corporation
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Author: Bin Yang <bin.yang@intel.com>
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/sched.h>
#include <linux/intel_mid_pm.h>
#include <linux/rpmsg.h>

#include <asm/intel_mid_rpmsg.h>
#include <asm/intel_mid_remoteproc.h>
#include <asm/intel_scu_pmic.h>
#include <asm/intel_basincove_gpadc.h>

#include <linux/iio/iio.h>
#include <linux/iio/machine.h>
#include <linux/iio/buffer.h>
#include <linux/iio/driver.h>
#include <linux/iio/types.h>
#include <linux/iio/consumer.h>

#define GPADCREQ	0xDC
#define GPADCREQ_IRQEN	(1 << 1)
#define GPADCREQ_BUSY	(1 << 0)
#define MIRQLVL1_ADC	(1 << 4)
#define MIRQLVL1	0x0C
#define ADC1CNTL	0xDD
#define ADCIRQ		0x06
#define MADCIRQ		0x11

static struct gpadc_regmap_t {
	char *name;
	int cntl;	/* GPADC Conversion Control Bit indicator */
	int rslth;	/* GPADC Conversion Result Register Addr High */
	int rsltl;	/* GPADC Conversion Result Register Addr Low */
} gpadc_regmaps[GPADC_CH_NUM] = {
	{"VBAT",	5, 0xE9, 0xEA, },
	{"BATID",	4, 0xEB, 0xEC, },
	{"IBAT",	5, 0xED, 0xEE, },
	{"PMICTEMP",	3, 0xCC, 0xCD, },
	{"BATTEMP0",	2, 0xC8, 0xC9, },
	{"BATTEMP1",	2, 0xCA, 0xCB, },
	{"SYSTEMP0",	3, 0xC2, 0xC3, },
	{"SYSTEMP1",	3, 0xC4, 0xC5, },
	{"SYSTEMP2",	3, 0xC6, 0xC7, },
};

struct gpadc_info {
	int initialized;
	/* This mutex protects gpadc sample/config from concurrent conflict.
	   Any function, which does the sample or config, needs to
	   hold this lock.
	   If it is locked, it also means the gpadc is in active mode.
	*/
	struct mutex lock;
	struct device *dev;
	int irq;
	u8 irq_status;
	wait_queue_head_t wait;
	int sample_done;
	void __iomem *intr;
};

static inline int gpadc_clear_bits(u16 addr, u8 mask)
{
	return intel_scu_ipc_update_register(addr, 0, mask);
}

static inline int gpadc_set_bits(u16 addr, u8 mask)
{
	return intel_scu_ipc_update_register(addr, 0xff, mask);
}

static inline int gpadc_write(u16 addr, u8 data)
{
	return intel_scu_ipc_iowrite8(addr, data);
}

static inline int gpadc_read(u16 addr, u8 *data)
{
	return intel_scu_ipc_ioread8(addr, data);
}

static int gpadc_busy_wait(void)
{
	u8 tmp;
	int timeout = 0;

	gpadc_read(GPADCREQ, &tmp);
	while (tmp & GPADCREQ_BUSY && timeout < 500) {
		gpadc_read(GPADCREQ, &tmp);
		usleep_range(1800, 2000);
		timeout++;
	}

	if (tmp & GPADCREQ_BUSY)
		return -EBUSY;
	else
		return 0;
}

static void gpadc_dump(struct gpadc_info *info)
{
	u8 tmp;

	dev_err(info->dev, "GPADC registers dump:\n");
	gpadc_read(ADCIRQ, &tmp);
	dev_err(info->dev, "ADCIRQ: 0x%x\n", tmp);
	gpadc_read(MADCIRQ, &tmp);
	dev_err(info->dev, "MADCIRQ: 0x%x\n", tmp);
	gpadc_read(GPADCREQ, &tmp);
	dev_err(info->dev, "GPADCREQ: 0x%x\n", tmp);
	gpadc_read(ADC1CNTL, &tmp);
	dev_err(info->dev, "ADC1CNTL: 0x%x\n", tmp);
}

static irqreturn_t gpadc_isr(int irq, void *data)
{
	struct gpadc_info *info = iio_priv(data);

	info->irq_status = ioread8(info->intr);
	info->sample_done = 1;
	wake_up(&info->wait);
	return IRQ_WAKE_THREAD;
}

static irqreturn_t gpadc_threaded_isr(int irq, void *data)
{
	/* Clear IRQLVL1MASK */
	gpadc_clear_bits(MIRQLVL1, MIRQLVL1_ADC);

	return IRQ_HANDLED;
}


/**
 * iio_basincove_gpadc_sample - do gpadc sample.
 * @indio_dev: industrial IO GPADC device handle
 * @ch: gpadc bit set of channels to sample, for example, set ch = (1<<0)|(1<<2)
 *	means you are going to sample both channel 0 and 2 at the same time.
 * @res:gpadc sampling result
 *
 * Returns 0 on success or an error code.
 *
 * This function may sleep.
 */

int iio_basincove_gpadc_sample(struct iio_dev *indio_dev,
				int ch, struct gpadc_result *res)
{
	struct gpadc_info *info = iio_priv(indio_dev);
	int i, ret;
	u8 tmp, th, tl;
	u8 mask;

	if (!info->initialized)
		return -ENODEV;

	mutex_lock(&info->lock);

	mask = MBATTEMP | MSYSTEMP | MBATT | MVIBATT | MCCTICK;
	gpadc_clear_bits(MADCIRQ, mask);
	gpadc_clear_bits(MIRQLVL1, MIRQLVL1_ADC);

	tmp = GPADCREQ_IRQEN;

	for (i = 0; i < GPADC_CH_NUM; i++) {
		if (ch & (1 << i))
			tmp |= (1 << gpadc_regmaps[i].cntl);
	}

	info->sample_done = 0;

	ret = gpadc_busy_wait();
	if (ret) {
		dev_err(info->dev, "GPADC is busy\n");
		goto done;
	}

	gpadc_write(GPADCREQ, tmp);

	ret = wait_event_timeout(info->wait, info->sample_done, HZ);
	if (ret == 0) {
		gpadc_dump(info);
		ret = -ETIMEDOUT;
		dev_err(info->dev, "sample timeout, return %d\n", ret);
		goto done;
	} else {
		ret = 0;
	}

	for (i = 0; i < GPADC_CH_NUM; i++) {
		if (ch & (1 << i)) {
			gpadc_read(gpadc_regmaps[i].rslth, &th);
			gpadc_read(gpadc_regmaps[i].rsltl, &tl);
			res->data[i] = ((th & 0x3) << 8) + tl;
		}
	}

done:
	gpadc_set_bits(MIRQLVL1, MIRQLVL1_ADC);
	gpadc_set_bits(MADCIRQ, mask);
	mutex_unlock(&info->lock);
	return ret;
}
EXPORT_SYMBOL(iio_basincove_gpadc_sample);

static struct gpadc_result sample_result;
static int chs;

static ssize_t intel_basincove_gpadc_store_channel(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t size)
{
	if (sscanf(buf, "%x", &chs) != 1) {
		dev_err(dev, "one channel argument is needed\n");
		return -EINVAL;
	}

	if (chs < (1 << 0) || chs >= (1 << GPADC_CH_NUM)) {
		dev_err(dev, "invalid channel, should be in [0x1 - 0x1FF]\n");
		return -EINVAL;
	}

	return size;
}

static ssize_t intel_basincove_gpadc_show_channel(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "0x%x\n", chs);
}

static ssize_t intel_basincove_gpadc_store_sample(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t size)
{
	int value, ret;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);

	memset(sample_result.data, 0, sizeof(sample_result.data));

	if (sscanf(buf, "%d", &value) != 1) {
		dev_err(dev, "one argument is needed\n");
		return -EINVAL;
	}

	if (value == 1) {
		ret = iio_basincove_gpadc_sample(indio_dev, chs,
						&sample_result);
		if (ret) {
			dev_err(dev, "sample failed\n");
			return ret;
		}
	} else {
		dev_err(dev, "input '1' to sample\n");
		return -EINVAL;
	}

	return size;
}

static ssize_t intel_basincove_gpadc_show_result(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int i;
	int used = 0;

	for (i = 0; i < GPADC_CH_NUM; i++) {
		used += snprintf(buf + used, PAGE_SIZE - used,
			 "sample_result[%d] = %d\n", i, sample_result.data[i]);
	}

	return used;
}


static DEVICE_ATTR(channel, S_IWUSR | S_IRUGO,
		intel_basincove_gpadc_show_channel,
		intel_basincove_gpadc_store_channel);
static DEVICE_ATTR(sample, S_IWUSR, NULL, intel_basincove_gpadc_store_sample);
static DEVICE_ATTR(result, S_IRUGO, intel_basincove_gpadc_show_result, NULL);

static struct attribute *intel_basincove_gpadc_attrs[] = {
	&dev_attr_channel.attr,
	&dev_attr_sample.attr,
	&dev_attr_result.attr,
	NULL,
};
static struct attribute_group intel_basincove_gpadc_attr_group = {
	.name = "basincove_gpadc",
	.attrs = intel_basincove_gpadc_attrs,
};

#define MSIC_ADC_CHANNEL(_type, _channel, _datasheet_name) \
	{				\
		.indexed = 1,		\
		.type = _type,		\
		.channel = _channel,	\
		.datasheet_name = _datasheet_name,	\
	}

static const struct iio_chan_spec const basincove_adc_channels[] = {
	MSIC_ADC_CHANNEL(IIO_VOLTAGE, 0, "CH0"),
	MSIC_ADC_CHANNEL(IIO_RESISTANCE, 1, "CH1"),
	MSIC_ADC_CHANNEL(IIO_CURRENT, 2, "CH2"),
	MSIC_ADC_CHANNEL(IIO_TEMP, 3, "CH3"),
	MSIC_ADC_CHANNEL(IIO_TEMP, 4, "CH4"),
	MSIC_ADC_CHANNEL(IIO_TEMP, 5, "CH5"),
	MSIC_ADC_CHANNEL(IIO_TEMP, 6, "CH6"),
	MSIC_ADC_CHANNEL(IIO_TEMP, 7, "CH7"),
	MSIC_ADC_CHANNEL(IIO_TEMP, 8, "CH8"),
};

static int basincove_adc_read_raw(struct iio_dev *indio_dev,
			struct iio_chan_spec const *chan,
			int *val, int *val2, long m)
{
	int ret;
	int ch = chan->channel;
	struct gpadc_info *info = iio_priv(indio_dev);
	struct gpadc_result res;

	ret = iio_basincove_gpadc_sample(indio_dev, (1 << ch), &res);
	if (ret) {
		dev_err(info->dev, "sample failed\n");
		return -EINVAL;
	}

	*val = res.data[ch];

	return ret;
}

static int basincove_adc_read_all_raw(struct iio_channel *chan,
					int *val)
{
	int ret;
	int i, num = 0;
	int ch = 0;
	int *channels;
	struct gpadc_info *info = iio_priv(chan->indio_dev);
	struct gpadc_result res;

	while (chan[num].indio_dev)
		num++;

	channels = kzalloc(sizeof(int) * num, GFP_KERNEL);
	if (channels == NULL)
		return -ENOMEM;

	for (i = 0; i < num; i++) {
		channels[i] = chan[i].channel->channel;
		ch |= (1 << channels[i]);
	}

	ret = iio_basincove_gpadc_sample(chan->indio_dev, ch, &res);
	if (ret) {
		dev_err(info->dev, "sample failed\n");
		ret = -EINVAL;
		goto end;
	}

	for (i = 0; i < num; i++)
		val[i] = res.data[channels[i]];

end:
	kfree(channels);
	return ret;
}

static const struct iio_info basincove_adc_info = {
	.read_raw = &basincove_adc_read_raw,
	.read_all_raw = &basincove_adc_read_all_raw,
	.driver_module = THIS_MODULE,
};

static int bcove_gpadc_probe(struct platform_device *pdev)
{
	int err;
	struct gpadc_info *info;
	struct iio_dev *indio_dev;
	struct intel_basincove_gpadc_platform_data *pdata =
			pdev->dev.platform_data;

	if (!pdata) {
		dev_err(&pdev->dev, "no platform data supplied\n");
		err = -EINVAL;
		goto out;
	}

	indio_dev = iio_device_alloc(sizeof(struct gpadc_info));
	if (indio_dev == NULL) {
		dev_err(&pdev->dev, "allocating iio device failed\n");
		err = -ENOMEM;
		goto out;
	}

	info = iio_priv(indio_dev);

	mutex_init(&info->lock);
	init_waitqueue_head(&info->wait);
	info->dev = &pdev->dev;
	info->irq = platform_get_irq(pdev, 0);
	info->intr = ioremap_nocache(pdata->intr, 1);
	if (!info->intr) {
		dev_err(&pdev->dev, "ioremap of ADCIRQ failed\n");
		err = -ENOMEM;
		goto err_free;
	}

	err = request_threaded_irq(info->irq, gpadc_isr, gpadc_threaded_isr,
			IRQF_ONESHOT, "adc", indio_dev);
	if (err) {
		gpadc_dump(info);
		dev_err(&pdev->dev, "unable to register irq %d\n", info->irq);
		goto err_iounmap;
	}

	platform_set_drvdata(pdev, indio_dev);

	indio_dev->dev.parent = &pdev->dev;
	indio_dev->name = pdev->name;

	indio_dev->channels = basincove_adc_channels;
	indio_dev->num_channels = ARRAY_SIZE(basincove_adc_channels);
	indio_dev->info = &basincove_adc_info;
	indio_dev->modes = INDIO_DIRECT_MODE;

	err = iio_map_array_register(indio_dev, pdata->iio_maps);
	if (err)
		goto err_release_irq;

	err = iio_device_register(indio_dev);
	if (err < 0)
		goto err_array_unregister;

	err = sysfs_create_group(&pdev->dev.kobj,
			&intel_basincove_gpadc_attr_group);
	if (err) {
		dev_err(&pdev->dev, "Unable to export sysfs interface, error: %d\n",
			err);
		goto err_iio_device_unregister;
	}

	info->initialized = 1;

	dev_info(&pdev->dev, "bcove adc probed\n");

	return 0;

err_iio_device_unregister:
	iio_device_unregister(indio_dev);
err_array_unregister:
	iio_map_array_unregister(indio_dev);
err_release_irq:
	free_irq(info->irq, info);
err_iounmap:
	iounmap(info->intr);
err_free:
	iio_device_free(indio_dev);
out:
	return err;
}

static int bcove_gpadc_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);
	struct gpadc_info *info = iio_priv(indio_dev);

	sysfs_remove_group(&pdev->dev.kobj,
			&intel_basincove_gpadc_attr_group);

	iio_device_unregister(indio_dev);
	iio_map_array_unregister(indio_dev);
	free_irq(info->irq, info);
	iounmap(info->intr);
	iio_device_free(indio_dev);

	return 0;
}

#ifdef CONFIG_PM
static int bcove_gpadc_suspend(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct gpadc_info *info = iio_priv(indio_dev);

	if (!mutex_trylock(&info->lock))
		return -EBUSY;

	return 0;
}

static int bcove_gpadc_resume(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct gpadc_info *info = iio_priv(indio_dev);

	mutex_unlock(&info->lock);
	return 0;
}
#else
#define bcove_gpadc_suspend		NULL
#define bcove_gpadc_resume		NULL
#endif

static const struct dev_pm_ops bcove_gpadc_driver_pm_ops = {
	.suspend	= bcove_gpadc_suspend,
	.resume		= bcove_gpadc_resume,
};

static struct platform_driver bcove_gpadc_driver = {
	.driver = {
		   .name = "bcove_adc",
		   .owner = THIS_MODULE,
		   .pm = &bcove_gpadc_driver_pm_ops,
		   },
	.probe = bcove_gpadc_probe,
	.remove = bcove_gpadc_remove,
};

static int bcove_gpadc_module_init(void)
{
	return platform_driver_register(&bcove_gpadc_driver);
}

static void bcove_gpadc_module_exit(void)
{
	platform_driver_unregister(&bcove_gpadc_driver);
}

static int bcove_adc_rpmsg_probe(struct rpmsg_channel *rpdev)
{
	int ret = 0;

	if (rpdev == NULL) {
		pr_err("rpmsg channel not created\n");
		ret = -ENODEV;
		goto out;
	}

	dev_info(&rpdev->dev, "Probed bcove_gpadc rpmsg device\n");

	ret = bcove_gpadc_module_init();

out:
	return ret;
}

static void bcove_adc_rpmsg_remove(struct rpmsg_channel *rpdev)
{
	bcove_gpadc_module_exit();
	dev_info(&rpdev->dev, "Removed bcove_gpadc rpmsg device\n");
}

static void bcove_adc_rpmsg_cb(struct rpmsg_channel *rpdev, void *data,
					int len, void *priv, u32 src)
{
	dev_warn(&rpdev->dev, "unexpected, message\n");

	print_hex_dump(KERN_DEBUG, __func__, DUMP_PREFIX_NONE, 16, 1,
		       data, len,  true);
}

static struct rpmsg_device_id bcove_adc_rpmsg_id_table[] = {
	{ .name	= "rpmsg_bcove_adc" },
	{ },
};
MODULE_DEVICE_TABLE(rpmsg, bcove_adc_rpmsg_id_table);

static struct rpmsg_driver bcove_adc_rpmsg = {
	.drv.name	= KBUILD_MODNAME,
	.drv.owner	= THIS_MODULE,
	.id_table	= bcove_adc_rpmsg_id_table,
	.probe		= bcove_adc_rpmsg_probe,
	.callback	= bcove_adc_rpmsg_cb,
	.remove		= bcove_adc_rpmsg_remove,
};

static int __init bcove_adc_rpmsg_init(void)
{
	return register_rpmsg_driver(&bcove_adc_rpmsg);
}

#ifdef MODULE
module_init(bcove_adc_rpmsg_init);
#else
rootfs_initcall(bcove_adc_rpmsg_init);
#endif

static void __exit bcove_adc_rpmsg_exit(void)
{
	return unregister_rpmsg_driver(&bcove_adc_rpmsg);
}
module_exit(bcove_adc_rpmsg_exit);

MODULE_AUTHOR("Yang Bin<bin.yang@intel.com>");
MODULE_DESCRIPTION("Intel Merrifield Basin Cove GPADC Driver");
MODULE_LICENSE("GPL");
