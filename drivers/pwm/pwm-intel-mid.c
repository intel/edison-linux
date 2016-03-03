/*
 * pwm-intel-mid.c: Driver for PWM on Intel MID platform
 *
 * (C) Copyright 2014 Intel Corporation
 * Author: Nicolas Pernas Maradei <nicolas.pernas.maradei@emutex.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/pwm.h>
#include <linux/pci.h>
#include <linux/pm_runtime.h>

#define PWM_INTEL_MID_DRIVER_NAME     "pwm-intel-mid"
#define PCI_DEVICE_ID_INTEL_MID_MRFLD 0x11a5
#define CLOCK_RATE                    19200000
#define BASE_10                       10
#define NSECS_PER_SEC                 1000000000UL
#define PWM_PERIOD_NS_MAX             218453000 /* about 4.6 Hz */
#define PWM_PERIOD_NS_MIN             104 /* about 9.6 MHz */
#define PWM_ON_TIME_DIVISOR_BITS      8
#define PWM_BASE_UNIT_FRAC_BITS       14
#define PWM_BASE_UNIT_INT_BITS        8
#define PWM_COMPARE_UNIT_SIZE         256UL
#define PWM_DYNAMYC_RANGE_DEFAULT     100UL
#define PWM_DYNAMYC_RANGE_THRESHOLD   13333
#define PWM_DEFAULT_PERIOD            4950495 /* about 200 Hz */
#define PWM_CONTROL_REGISTER_SIZE     0x400

#define get_control_register(pwm, pwm_id) \
	((u8 *)pwm->regs + (pwm_id * PWM_CONTROL_REGISTER_SIZE));

union pwmctrl_reg {
	struct {
		u32 on_time_divisor:PWM_ON_TIME_DIVISOR_BITS;
		u32 base_unit_frac:PWM_BASE_UNIT_FRAC_BITS;
		u32 base_unit_int:PWM_BASE_UNIT_INT_BITS;
		u32 sw_update:1;
		u32 enable:1;
	} part;
	u32 full;
};

struct intel_mid_pwm_chip {
	struct pwm_chip chip;
	void __iomem *regs;
	union pwmctrl_reg *pwmctrls;
	int num_of_pwms;
};

static inline struct intel_mid_pwm_chip *
to_pwm(struct pwm_chip *chip)
{
	return container_of(chip, struct intel_mid_pwm_chip, chip);
}

static inline void
pwm_set_enable_bit(void __iomem *reg, union pwmctrl_reg *pwmctrl, u8 value)
{
	pwmctrl->full = readl(reg);
	pwmctrl->part.enable = value;
	writel(pwmctrl->full, reg);
}

static void
intel_mid_pwm_on_time_divisor(void __iomem *reg, union pwmctrl_reg *pwmctrl,
	const u32 period, const u32 duty_cycle)
{
	u64 on_time_divisor;

	/* Calculate and set on time divisor */
	on_time_divisor = duty_cycle * (u64)(PWM_COMPARE_UNIT_SIZE - 1UL);
	do_div(on_time_divisor, period);
	on_time_divisor = PWM_COMPARE_UNIT_SIZE - on_time_divisor - 1UL;

	pwmctrl->full = readl(reg);
	pwmctrl->part.on_time_divisor = on_time_divisor;
	writel(pwmctrl->full, reg);
}

static int
intel_mid_pwm_base_unit(void __iomem *reg, union pwmctrl_reg *pwmctrl,
	const u32 period, const u32 clock_rate)
{
	u32 dynamic_range = PWM_DYNAMYC_RANGE_DEFAULT;
	u64 rest;
	u64 tmp;
	u64 fraction = 0;
	u64 numerator = 1;
	u64 base_unit_integer;
	u64 frequency = NSECS_PER_SEC;
	u32 base_unit_fraction = 0;
	int i;

	/* The dynamic range multiplier is used to get more accurate
	calculations when the frequency is small (less than 75 KHz) in the
	fraction part of base unit. In some way when requesting low frequencies
	all calculations are done using the period in nano-secs e-2. For high
	frecuencies we use nano-secs only. */
	if (period < PWM_DYNAMYC_RANGE_THRESHOLD)
		dynamic_range = 1UL;
	frequency *= dynamic_range;

	/* calculate frequency: f (hz) = 1e9 (ns/ps) / p (ns/ps). Result is in
	Hz depending on dynamic_range being 1. */
	do_div(frequency, period);

	/* base_unit is a 22 bits register composed of a fractional part (first
	14 bits) and an integer part (next 8 bits). The integer part
	calculation is trivial. Done in place by do_div() below. */
	base_unit_integer = frequency * PWM_COMPARE_UNIT_SIZE;

	rest = do_div(base_unit_integer, clock_rate * dynamic_range);

	/* The fractional part of base_unit needs to be calculated and converted
	to binary fixed point notation. Two steps will be needed to do the
	calculation. First to calculate the fraction part in decimal and
	secondly to convert it to binary fixed point. Due to lack of float
	support in the kernel we'll use the rest of (frequency *
	PWM_COMPARE_UNIT_SIZE / clock_speed) division to calculate it and then
	following the standard division algorithm the rest will be multiplied
	by 10 and divided by clock_rate several times until the desired level
	of precision is reached. At the end it will look like this:
	base_unit_fraction = fraction / numerator where numerator is a power
	of 10.

	E.g.:	base_unit = 1.123, base_unit_fraction = 0.123,
			fraction = 123, numerator = 1000 */
	for (i = 0; i < PWM_BASE_UNIT_FRAC_BITS; i++) {
		tmp = rest * BASE_10;
		rest = do_div(tmp, clock_rate * dynamic_range);
		fraction += tmp;
		fraction *= BASE_10;
		numerator *= BASE_10;
	}
	do_div(fraction, BASE_10);

	/* At this point we've got the fraction and numerator done following
	the above description. The binary fixed point conversion is done by
	repeated multiplications of the fraction (but using fractions (fra/num)
	instead of floats). When the fraction is multipled by 2 and gets greater
	or equal than 1 (or in our case frac/num >= 1 -> frac >= num) then we
	know the next digit in the binary fixed point number will be a '1'.
	Also this excess needs to be removed. In the original algorithm the
	overflow digit is substracted. In our case we can substract the
	numerator. */
	for (i = 0; i < PWM_BASE_UNIT_FRAC_BITS; i++) {
		/* Multiply fraction by 2 */
		fraction <<= 1;
		base_unit_fraction <<= 1;

		/* frac / num >= 1 -> set next bit to '1' and remove "overflow
		digit" */
		if (fraction >= numerator) {
			base_unit_fraction |= 1;
			fraction -= numerator;
		}
	}
	/* If both the values are 0, the output will be somehow not correct.
	 * So if it happens, change the fraction to 1.
	 */
	if ((0 == base_unit_fraction) && (0 == base_unit_integer))
		base_unit_fraction = 1UL;

	pwmctrl->full = readl(reg);
	pwmctrl->part.base_unit_int = (u32)base_unit_integer;
	pwmctrl->part.base_unit_frac = base_unit_fraction;
	writel(pwmctrl->full, reg);

	return 0;
}

static int
intel_mid_pwm_setup(void __iomem *reg, union pwmctrl_reg *pwmctrl,
	int duty_ns, int period_ns)
{
	int ret;

	/* Calculate and set base_unit */
	ret = intel_mid_pwm_base_unit(reg, pwmctrl, period_ns, CLOCK_RATE);
	if (ret)
		return ret;

	/* Calculate and set on time divisor */
	intel_mid_pwm_on_time_divisor(reg, pwmctrl, period_ns, duty_ns);

	/* Set software update bit */
	pwmctrl->part.sw_update = 1UL;
	writel(pwmctrl->full, reg);

	return 0;
}

static int
intel_mid_pwm_config(struct pwm_chip *chip, struct pwm_device *pwm_dev,
	int duty_ns, int period_ns)
{
	struct intel_mid_pwm_chip *pwm = to_pwm(chip);
	union pwmctrl_reg *pwmctrl = &pwm->pwmctrls[pwm_dev->hwpwm];
	void __iomem *reg = get_control_register(pwm, pwm_dev->hwpwm);

	dev_dbg(chip->dev, "%s: period_ns %d, duty_ns %d\n", __func__,
		period_ns, duty_ns);

	/* Check the period is valid within HW capabilities */
	if (period_ns < PWM_PERIOD_NS_MIN || period_ns > PWM_PERIOD_NS_MAX) {
		dev_err(chip->dev, "Period (ns) must be in range %u:%u\n",
			PWM_PERIOD_NS_MIN, PWM_PERIOD_NS_MAX);
		return -EINVAL;
	}

	return intel_mid_pwm_setup(reg, pwmctrl, duty_ns, period_ns);
}

static int
intel_mid_pwm_enable(struct pwm_chip *chip, struct pwm_device *pwm_dev)
{
	struct intel_mid_pwm_chip *pwm = to_pwm(chip);
	union pwmctrl_reg *pwmctrl = &pwm->pwmctrls[pwm_dev->hwpwm];
	int ret;
	void __iomem *reg = get_control_register(pwm, pwm_dev->hwpwm);

	pm_runtime_get_sync(chip->dev);

	ret = intel_mid_pwm_setup(reg, pwmctrl, pwm_dev->duty_cycle,
		pwm_dev->period);
	if (ret)
		return ret;

	pwm_set_enable_bit(reg, pwmctrl, 1U);

	dev_dbg(chip->dev, "%s: pwmctrl %#x\n", __func__, pwmctrl->full);

	return 0;
}

static void
intel_mid_pwm_disable(struct pwm_chip *chip, struct pwm_device *pwm_dev)
{
	struct intel_mid_pwm_chip *pwm = to_pwm(chip);
	union pwmctrl_reg *pwmctrl = &pwm->pwmctrls[pwm_dev->hwpwm];
	void __iomem *reg = get_control_register(pwm, pwm_dev->hwpwm);

	pwm_set_enable_bit(reg, pwmctrl, 0);
	pm_runtime_put(chip->dev);
	dev_dbg(chip->dev, "%s: pwmctrl %#x\n", __func__, pwmctrl->full);
}

static const struct pwm_ops intel_mid_pwm_ops = {
	.config = intel_mid_pwm_config,
	.enable = intel_mid_pwm_enable,
	.disable = intel_mid_pwm_disable,
	.owner = THIS_MODULE,
};

static int
intel_mid_pwm_probe(struct pci_dev *pci, const struct pci_device_id *pci_id)
{
	struct intel_mid_pwm_chip *pwm;
	int ret, i;
	resource_size_t resource_len;

	pwm = devm_kzalloc(&pci->dev, sizeof(*pwm), GFP_KERNEL);
	if (!pwm) {
		dev_err(&pci->dev, "Can't allocate memory for pwm\n");
		return -ENOMEM;
	}

	/* Init the device */
	ret = pci_enable_device(pci);
	if (ret) {
		dev_err(&pci->dev, "Can't enable pci device\n");
		return ret;
	}

	ret = pci_request_regions(pci, PWM_INTEL_MID_DRIVER_NAME);
	if (ret) {
		dev_err(&pci->dev, "Can't request regions\n");
		goto do_disable_device;
	}
	pci_dev_get(pci);

	pwm->regs = pci_ioremap_bar(pci, 0);
	if (!pwm->regs) {
		dev_err(&pci->dev, "ioremap failed\n");
		ret = -EIO;
		goto do_disable_device;
	}

	/* Calculate number of available pwm modules */
	resource_len = pci_resource_len(pci, 0);
	do_div(resource_len, PWM_CONTROL_REGISTER_SIZE);
	pwm->num_of_pwms = resource_len;

	/* allocate memory for PWM control register images */
	pwm->pwmctrls = devm_kzalloc(&pci->dev,
		sizeof(*pwm->pwmctrls) * pwm->num_of_pwms, GFP_KERNEL);
	if (!pwm->pwmctrls) {
		dev_err(&pci->dev, "Can't allocate memory for pwm pwmctrls\n");
		ret = -ENOMEM;
		goto do_unmap_regs;
	}

	/* register the driver with PWM framework */
	pwm->chip.dev = &pci->dev;
	pwm->chip.ops = &intel_mid_pwm_ops;
	pwm->chip.base = -1;
	pwm->chip.npwm = pwm->num_of_pwms;

	ret = pwmchip_add(&pwm->chip);
	if (ret) {
		dev_err(&pci->dev, "Failed to add PWM chip: %d\n", ret);
		goto do_unmap_regs;
	}

	/* Set default frequency/period (about 200Hz) on all pwm modules. */
	for (i = 0; i < pwm->num_of_pwms; i++)
		pwm->chip.pwms[i].period = PWM_DEFAULT_PERIOD;

	pci_set_drvdata(pci, pwm);
	pm_runtime_allow(&pci->dev);
	pm_runtime_put_noidle(&pci->dev);

	return ret;

do_unmap_regs:
	iounmap(pwm->regs);
	pci_release_regions(pci);
do_disable_device:
	pci_disable_device(pci);

	return ret;
}

static void
intel_mid_pwm_remove(struct pci_dev *pci)
{
	struct intel_mid_pwm_chip *pwm = pci_get_drvdata(pci);

	pm_runtime_get_noresume(&pci->dev);
	pm_runtime_forbid(&pci->dev);
	pwmchip_remove(&pwm->chip);
	iounmap(pwm->regs);
	pci_release_regions(pci);
	pci_disable_device(pci);
	pci_set_drvdata(pci, NULL);
}

#if CONFIG_PM
static int
intel_mid_pwm_runtime_suspend(struct device *dev)
{
	return 0;
}

static int
intel_mid_pwm_runtime_resume(struct device *dev)
{
	return 0;
}

static void
intel_mid_pwm_runtime_complete(struct device *dev) {
}

static const struct dev_pm_ops intel_mid_pm_ops = {
	.prepare = intel_mid_pwm_runtime_suspend,
	.complete = intel_mid_pwm_runtime_complete,
	.runtime_suspend = intel_mid_pwm_runtime_suspend,
	.runtime_resume = intel_mid_pwm_runtime_resume,
};
#endif

/* PCI Routines */
static DEFINE_PCI_DEVICE_TABLE(intel_mid_pwm_pci_ids) = {
	{ PCI_VDEVICE(INTEL, PCI_DEVICE_ID_INTEL_MID_MRFLD), 0},
	{ 0, }
};
MODULE_DEVICE_TABLE(pci, intel_mid_pwm_pci_ids);

static struct pci_driver intel_mid_pci_driver = {
	.name = PWM_INTEL_MID_DRIVER_NAME,
	.id_table = intel_mid_pwm_pci_ids,
	.probe = intel_mid_pwm_probe,
	.remove = intel_mid_pwm_remove,
#ifdef CONFIG_PM
	.driver = {
		.pm = &intel_mid_pm_ops,
	},
#endif
};

module_pci_driver(intel_mid_pci_driver);

MODULE_ALIAS("pci:" PWM_INTEL_MID_DRIVER_NAME);
MODULE_DESCRIPTION("Intel(R) MID PWM driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Nicolas Pernas Maradei <nicolas.pernas.maradei@emutex.com>");

