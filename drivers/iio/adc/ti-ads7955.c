/*
 * ADS7955 SPI ADC driver
 *
 * (C) Copyright 2014 Intel Corporation
 * Author: Dave Hunt <dave.hunt@emutex.com>
 *
 * Licensed under the GPL-2.
 */

#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/spi/spi.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/interrupt.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>

#include <linux/platform_data/ti-ads7955.h>

#define ADS7955_EXTREF	2.5

#define ADS7955_MANUAL_MODE	(1 << 12)  /* Selects Manual Mode */
#define ADS7955_CONFIG		(1 << 11)  /* Enable programming bits DI06-00*/
#define ADS7955_CHANNEL(x)	((x) << 7) /* Channel select (DI10-07) */

#define ADS7955_RANGE_1		0	/* Selects 2.5V input range (Range 1)*/
#define ADS7955_RANGE_2		(1 << 6)/* Selects 5.0V input range (Range 2)*/

#define ADS7955_POWER_NORMAL	0	/* No Powerdown */
#define ADS7955_POWER_DOWN (1 << 5)	/* Powerdown on 16th fall-edge of SCLK*/

#define ADS7955_GET_CONVERSION	0	/* Powerdown on 16th fall-edge of SCLK*/
#define ADS7955_GET_GPIO	(1 << 4)/* Powerdown on 16th fall-edge of SCLK*/

#define ADS7955_SET_READ	(ADS7955_MANUAL_MODE | ADS7955_CONFIG | \
				ADS7955_RANGE_2 | ADS7955_POWER_NORMAL | \
				ADS7955_GET_CONVERSION)

#define ADS7955_MAX_CHAN	8
#define ADS7955_BITS		10
#define ADS7955_STORAGE_BITS	12
#define ADS7955_INTREF_mV	3300


#define RES_MASK(bits)	((1 << (bits)) - 1)

struct ads7955_state {
	struct spi_device		*spi;
	struct regulator		*reg;
	unsigned			ext_ref;
	struct spi_transfer		scan_single_xfer[3];
	struct spi_message		scan_single_msg;
	/*
	 * DMA (thus cache coherency maintenance) requires the
	 * transfer buffers to live in their own cache lines.
	 */
	__be16				rx_buf[1] ____cacheline_aligned;
	__be16				tx_buf[1];
};

#define ADS7955_V_CHAN(index)						\
	{								\
		.type = IIO_VOLTAGE,					\
		.indexed = 1,						\
		.channel = index,					\
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		\
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),	\
		.address = index,					\
		.scan_index = index,					\
		.scan_type = {						\
			.sign = 'u',					\
			.realbits = 10,					\
			.storagebits = 16,				\
			.endianness = IIO_BE,				\
		},							\
	}

static const struct iio_chan_spec ads7955_channels[] = {
	ADS7955_V_CHAN(0),
	ADS7955_V_CHAN(1),
	ADS7955_V_CHAN(2),
	ADS7955_V_CHAN(3),
	ADS7955_V_CHAN(4),
	ADS7955_V_CHAN(5),
	ADS7955_V_CHAN(6),
	ADS7955_V_CHAN(7),
	IIO_CHAN_SOFT_TIMESTAMP(8),
};

static int ads7955_scan_direct(struct ads7955_state *st, unsigned ch)
{
	int ret, count = 3;

	st->tx_buf[0] = (ADS7955_SET_READ | ADS7955_CHANNEL(ch));
	while (count--) {
		ret = spi_sync(st->spi, &st->scan_single_msg);
		if (ret)
			return ret;
	}

	return st->rx_buf[0];
}


static int ads7955_get_ref_voltage(struct ads7955_state *st)
{
	int vref;

	if (st->ext_ref) {
		vref = regulator_get_voltage(st->reg);
		if (vref < 0)
			return vref;

		return vref / 1000;
	} else {
		return ADS7955_INTREF_mV;
	}
}

static int ads7955_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val,
			   int *val2,
			   long m)
{
	int ret;
	struct ads7955_state *st = iio_priv(indio_dev);

	switch (m) {
	case IIO_CHAN_INFO_RAW:
		mutex_lock(&indio_dev->mlock);
		if (indio_dev->currentmode == INDIO_BUFFER_TRIGGERED)
			ret = -EBUSY;
		else
			ret = ads7955_scan_direct(st, chan->address);
		mutex_unlock(&indio_dev->mlock);

		if (ret < 0)
			return ret;

		*val = ret & RES_MASK(ADS7955_BITS);

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		switch (chan->type) {
		case IIO_VOLTAGE:
			*val = ads7955_get_ref_voltage(st);
			*val2 = chan->scan_type.realbits;
			return IIO_VAL_FRACTIONAL_LOG2;
		default:
			return -EINVAL;
		}
	case IIO_CHAN_INFO_OFFSET:
		*val = 1093 - 2732500 / ads7955_get_ref_voltage(st);
		return IIO_VAL_INT;
	}
	return -EINVAL;
}

static const struct iio_info ads7955_info = {
	.read_raw = &ads7955_read_raw,
	.driver_module = THIS_MODULE,
};

static int ads7955_probe(struct spi_device *spi)
{
	struct ads7955_platform_data *pdata = spi->dev.platform_data;
	struct ads7955_state *st;
	struct iio_dev *indio_dev = iio_device_alloc(sizeof(*st));
	int ret;

	if (indio_dev == NULL)
		return -ENOMEM;

	st = iio_priv(indio_dev);

	if (pdata && pdata->ext_ref)
		st->ext_ref = ADS7955_EXTREF;

	if (st->ext_ref) {
		st->reg = regulator_get(&spi->dev, "vref");
		if (IS_ERR(st->reg)) {
			ret = PTR_ERR(st->reg);
			goto error_free;
		}
		ret = regulator_enable(st->reg);
		if (ret)
			goto error_put_reg;
	}

	spi_set_drvdata(spi, indio_dev);

	st->spi = spi;

	indio_dev->name = spi_get_device_id(spi)->name;
	indio_dev->dev.parent = &spi->dev;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = ads7955_channels;
	indio_dev->num_channels = ARRAY_SIZE(ads7955_channels);
	indio_dev->info = &ads7955_info;

	/* Setup default SPI comms parameters */
	spi->bits_per_word = 16;
	spi->max_speed_hz = 20000000;
	spi_setup(spi);

	st->scan_single_xfer[0].tx_buf = &st->tx_buf[0];
	st->scan_single_xfer[0].rx_buf = &st->rx_buf[0];
	st->scan_single_xfer[0].len = 2;

	spi_message_init(&st->scan_single_msg);
	spi_message_add_tail(&st->scan_single_xfer[0], &st->scan_single_msg);

	ret = iio_device_register(indio_dev);
	if (ret)
		goto error_disable_reg;

	return 0;

error_disable_reg:
	if (st->ext_ref)
		regulator_disable(st->reg);
error_put_reg:
	if (st->ext_ref)
		regulator_put(st->reg);
error_free:
	iio_device_free(indio_dev);

	return ret;
}

static int ads7955_remove(struct spi_device *spi)
{
	struct iio_dev *indio_dev = spi_get_drvdata(spi);
	struct ads7955_state *st = iio_priv(indio_dev);

	iio_device_unregister(indio_dev);
	if (st->ext_ref) {
		regulator_disable(st->reg);
		regulator_put(st->reg);
	}
	iio_device_free(indio_dev);

	return 0;
}

static const struct spi_device_id ads7955_id[] = {
	{"ads7955", 0},
	{}
};
MODULE_DEVICE_TABLE(spi, ads7955_id);

static struct spi_driver ads7955_driver = {
	.driver = {
		.name	= "ads7955",
		.owner	= THIS_MODULE,
	},
	.probe		= ads7955_probe,
	.remove		= ads7955_remove,
	.id_table	= ads7955_id,
};
module_spi_driver(ads7955_driver);

MODULE_AUTHOR("Dave Hunt <dave.hunt@emutex.com>");
MODULE_DESCRIPTION("Texas Instruments ADS7955 ADC");
MODULE_LICENSE("GPL v2");
