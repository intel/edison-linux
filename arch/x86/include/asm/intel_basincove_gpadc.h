#ifndef __INTEL_BASINCOVE_GPADC_H__
#define __INTEL_BASINCOVE_GPADC_H__

#define GPADC_VBAT	(1 << 0)
#define GPADC_BATID	(1 << 1)
#define GPADC_IBAT	(1 << 2)
#define GPADC_PMICTEMP	(1 << 3)
#define GPADC_BATTEMP0	(1 << 4)
#define GPADC_BATTEMP1	(1 << 5)
#define GPADC_SYSTEMP0	(1 << 6)
#define GPADC_SYSTEMP1	(1 << 7)
#define GPADC_SYSTEMP2	(1 << 8)
#define GPADC_CH_NUM	9

#define MBATTEMP	(1 << 2)
#define MSYSTEMP	(1 << 3)
#define MBATT		(1 << 4)
#define MVIBATT		(1 << 5)
#define MCCTICK		(1 << 7)

#define GPADC_RSL(channel, res) (res->data[ffs(channel)-1])

struct gpadc_regmap_t {
	char *name;
	int cntl;       /* GPADC Conversion Control Bit indicator */
	int rslth;      /* GPADC Conversion Result Register Addr High */
	int rsltl;      /* GPADC Conversion Result Register Addr Low */
};

struct gpadc_regs_t {
	u16 gpadcreq;
	u16 gpadcreq_irqen;
	u16 gpadcreq_busy;
	u16 mirqlvl1;
	u16 mirqlvl1_adc;
	u16 adc1cntl;
	u16 adcirq;
	u16 madcirq;
};

struct iio_dev;

struct intel_basincove_gpadc_platform_data {
	int channel_num;
	unsigned long intr;
	struct iio_map *gpadc_iio_maps;
	struct gpadc_regmap_t *gpadc_regmaps;
	struct gpadc_regs_t *gpadc_regs;
	struct iio_chan_spec *gpadc_channels;
};

struct gpadc_result {
	int data[GPADC_CH_NUM];
};

int iio_basincove_gpadc_sample(struct iio_dev *indio_dev,
				int ch, struct gpadc_result *res);

int intel_basincove_gpadc_sample(int ch, struct gpadc_result *res);
#endif
