/*
 * ADS7955 SPI ADC driver
 *
 * (C) Copyright 2014 Intel Corporation
 * Author: Dave Hunt <dave.hunt@emutex.com>
 *
 * Licensed under the GPL-2.
 */

/*
 * [FIXME]
 * Notes: This version of the ti-ads7955 driver is written with a couple of
 * workarounds for the functionality of the SPI driver on Edison at the time
 * of writing.
 * Issue 1: The CS is pushed low between every frame
 * Issue 2: spi_message_add_tail() can only be called once in the driver.
 *		Subsequent messages are ignored.
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
#include <linux/bitops.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>

#include <linux/platform_data/ti-ads7955.h>

#define ADS7955_EXTREF		true
#define SPI_MAX_SPEED_HZ	20000000
#define SPI_BITS_PER_WORD	16

#define ADS7955_MANUAL_MODE	(0x1 << 12)	/* Selects Manual Mode */
#define ADS7955_AUTO1_MODE	(0x2 << 12)	/* Selects Auto Mode 1 */
#define ADS7955_AUTO2_MODE	(0x3 << 12)	/* Selects Auto Mode 2 */
#define ADS7955_AUTO1_PROGRAM	(0x8 << 12)	/* Programming Auto Mode 1 */
#define ADS7955_AUTO2_PROGRAM	(0x9 << 12)	/* Programming Auto Mode 2 */

#define ADS7955_CONFIG		BIT(11)		/* program bits DI06-00 */
#define ADS7955_AUTO1_RESET	BIT(10)		/* Reset to first channel  */
#define ADS7955_CHANNEL(x)	((x & 0xf) << 7)/* Channel select (DI10-07) */

#define ADS7955_RANGE_1		0		/* Selects 2.5V input range */
#define ADS7955_RANGE_2		BIT(6)		/* Selects 5.0V input range */

#define ADS7955_POWER_NORMAL	0		/* No Powerdown */
#define ADS7955_POWER_DOWN	BIT(5)		/* Powerdown on last edge */

#define ADS7955_GET_CONVERSION	0		/* High bits have ch index*/
#define ADS7955_GET_GPIO	BIT(4)		/* High bits have GPIO bits */

#define ADS7955_SET_READ	(ADS7955_MANUAL_MODE | ADS7955_CONFIG | \
				ADS7955_RANGE_2 | ADS7955_POWER_NORMAL | \
				ADS7955_GET_CONVERSION)

#define ADS7955_READ_AUTO1	(ADS7955_AUTO1_MODE | ADS7955_CONFIG | \
				ADS7955_RANGE_2 | ADS7955_POWER_NORMAL | \
				ADS7955_GET_CONVERSION)

#define ADS7955_MAX_CHAN	8
#define ADS7955_BITS		12
#define ADS7955_STORAGE_BITS	16
/*
 * Define the Reference Voltage for the board on which this ADC is used.
 * May change depending on jumper settings or wiring configuration.
 */
#define ADS7955_INTREF_mV	5000
#define SPI_MSG_MAX_LEN		20		/* 8 channels plus timestamp */

#define RES_MASK(bits)	((1 << (bits)) - 1)

struct ads7955_state {
	struct spi_device	*spi;
	struct regulator	*reg;
	unsigned		ext_ref;
	struct spi_transfer	ring_xfer[10];
	struct spi_transfer	scan_single_xfer[3];
	struct spi_message	ring_msg;
	struct spi_message	scan_single_msg;
	/*
	 * DMA (thus cache coherency maintenance) requires the
	 * transfer buffers to live in their own cache lines.
	 */
	__u16			rx_buf[SPI_MSG_MAX_LEN] ____cacheline_aligned;
	__u16			tx_buf[SPI_MSG_MAX_LEN];
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
			.realbits = ADS7955_BITS,			\
			.storagebits = ADS7955_STORAGE_BITS,		\
			.endianness = IIO_CPU,				\
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

/**
 * ads7955_update_scan_mode() setup the spi transfer buffer for the scan mask
 **/
static int ads7955_update_scan_mode(struct iio_dev *indio_dev,
	const unsigned long *active_scan_mask)
{
	struct ads7955_state *st = iio_priv(indio_dev);
	int i, ret;
	unsigned short channel_count;

	/*
	 * For programming the auto1 mode, we need to send two words, one to
	 * specify program mode, and the other to give a bitmask of channels
	 * to be read when reading the auto sequence.
	 */
	/*
	 * [FIXME]
	 * Workaround: Build up a custom SPI message containing all required
	 * frames (including space for expected responses), and send as one
	 * SPI messge. This is to get around the issue that the current SPI
	 * driver only supports the first 'spi_message_add_tail' call.
	 */
	st->tx_buf[0] = ADS7955_AUTO1_PROGRAM;
	st->tx_buf[1] = (unsigned short)*active_scan_mask;
	st->tx_buf[2] = (ADS7955_SET_READ | ADS7955_POWER_DOWN);

	ret = spi_sync(st->spi, &st->scan_single_msg);
	if (ret)
		return ret;

	/*
	* So now we've told the hardware about the channels we want to sample,
	* now we set up the message sequence for when we're triggered.
	*/
	/*
	 * [FIXME]
	 * Workaround: Build up a custom SPI message containing all required
	 * frames (including space for expected responses), and send as one
	 * SPI messge. This is to get around the issue that the current SPI
	 * driver only supports the first 'spi_message_add_tail' call.
	 */
	channel_count = 0;
	for (i = 0; i < ADS7955_MAX_CHAN; i++) {
		if (test_bit(i, active_scan_mask)) {
			if (channel_count == 0)
				st->tx_buf[channel_count] = (ADS7955_READ_AUTO1
							| ADS7955_AUTO1_RESET);
			else
				st->tx_buf[channel_count] =
							(ADS7955_READ_AUTO1);
			channel_count++;
		}
	}

	/* Put in some extra tx frames to allow us to get the
		rx frames (behind tx by two frames) */
	st->tx_buf[channel_count++] = (ADS7955_READ_AUTO1);
	st->tx_buf[channel_count++] = (ADS7955_READ_AUTO1 |
					ADS7955_POWER_DOWN);

	st->ring_xfer[0].tx_buf = &st->tx_buf[0];
	st->ring_xfer[0].rx_buf = &st->rx_buf[0];
	st->ring_xfer[0].len = channel_count * 2;
	spi_message_init(&st->ring_msg);
	spi_message_add_tail(&st->ring_xfer[0], &st->ring_msg);
	return 0;
}

/**
 * ads7955_trigger_handler() bh of trigger launched polling to ring buffer
 **/
static irqreturn_t ads7955_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct ads7955_state *st = iio_priv(indio_dev);
	u8 *return_data = (u8 *)&(st->rx_buf[2]);
	s64 time_ns = 0;
	int ret;

	ret = spi_sync(st->spi, &st->ring_msg);
	if (ret)
		return IRQ_HANDLED;

	if (indio_dev->scan_timestamp) {
		time_ns = iio_get_time_ns();
		memcpy(return_data +
			indio_dev->scan_bytes - sizeof(s64),
			&time_ns, sizeof(time_ns));
	}

	iio_push_to_buffers(indio_dev, return_data);

	iio_trigger_notify_done(indio_dev->trig);

	return IRQ_HANDLED;
}

static int ads7955_scan_direct(struct ads7955_state *st, unsigned ch)
{
	int ret;

	/*
	 * [FIXME]
	 * Workaround: Build up a custom SPI message containing all required
	 * frames (including space for expected responses), and send as one
	 * SPI messge. This is to get around the issue that the current SPI
	 * driver only supports the first 'spi_message_add_tail' call.
	 */
	st->tx_buf[0] = (ADS7955_SET_READ | ADS7955_CHANNEL(ch));
	st->tx_buf[1] = (ADS7955_SET_READ | ADS7955_CHANNEL(ch));
	st->tx_buf[2] = (ADS7955_SET_READ | ADS7955_CHANNEL(ch) |
						ADS7955_POWER_DOWN);

	ret = spi_sync(st->spi, &st->scan_single_msg);
	if (ret)
		return ret;
	return st->rx_buf[2];
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
	}
	return -EINVAL;
}

static const struct iio_info ads7955_info = {
	.read_raw = &ads7955_read_raw,
	.update_scan_mode = ads7955_update_scan_mode,
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

	/*
	 * Setup default message
	 * [FIXME]
	 * Workaround:  Send each frame as 16 bits to get over the fact that
	 * the current SPI hardware pulls CS low between every frame.
	 */
	spi->bits_per_word = SPI_BITS_PER_WORD;
	spi->max_speed_hz = SPI_MAX_SPEED_HZ;
	spi_setup(spi);

	st->scan_single_xfer[0].tx_buf = &st->tx_buf[0];
	st->scan_single_xfer[0].rx_buf = &st->rx_buf[0];
	st->scan_single_xfer[0].len = 6;

	spi_message_init(&st->scan_single_msg);
	spi_message_add_tail(&st->scan_single_xfer[0], &st->scan_single_msg);

	ret = iio_triggered_buffer_setup(indio_dev, NULL,
			&ads7955_trigger_handler, NULL);
	if (ret) {
		dev_warn(&indio_dev->dev,
			"Failed to set up iio_triggered_buffer_setup\n");
		goto error_disable_reg;
	}

	ret = iio_device_register(indio_dev);
	if (ret)
		goto error_cleanup_ring;

	return 0;

error_cleanup_ring:
	iio_triggered_buffer_cleanup(indio_dev);
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
	iio_triggered_buffer_cleanup(indio_dev);
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
