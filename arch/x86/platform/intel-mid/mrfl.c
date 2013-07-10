/*
 * mrfl.c: Intel Merrifield platform specific setup code
 *
 * (C) Copyright 2012 Intel Corporation
 * Author: Mark F. Brown <mark.f.brown@intel.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <asm/setup.h>
#include <asm/intel-mid.h>
#include <asm/processor.h>

#define APIC_DIVISOR 16

enum intel_mid_sim_type __intel_mid_sim_platform;
EXPORT_SYMBOL_GPL(__intel_mid_sim_platform);

static void (*intel_mid_timer_init)(void);

static void tangier_arch_setup(void);

/* tangier arch ops */
static struct intel_mid_ops tangier_ops = {
	.arch_setup = tangier_arch_setup,
};

static unsigned long __init tangier_calibrate_tsc(void)
{
	/* [REVERT ME] fast timer calibration method to be defined */
	if ((intel_mid_identify_sim() == INTEL_MID_CPU_SIMULATION_VP) ||
	    (intel_mid_identify_sim() == INTEL_MID_CPU_SIMULATION_HVP)) {
		lapic_timer_frequency = 50000;
		return 1000000;
	}

	if ((intel_mid_identify_sim() == INTEL_MID_CPU_SIMULATION_SLE) ||
	    (intel_mid_identify_sim() == INTEL_MID_CPU_SIMULATION_NONE)) {

		unsigned long fast_calibrate;
		u32 lo, hi, ratio, fsb, bus_freq;

		/* *********************** */
		/* Compute TSC:Ratio * FSB */
		/* *********************** */

		/* Compute Ratio */
		rdmsr(MSR_PLATFORM_INFO, lo, hi);
		pr_debug("IA32 PLATFORM_INFO is 0x%x : %x\n", hi, lo);

		ratio = (lo >> 8) & 0xFF;
		pr_debug("ratio is %d\n", ratio);
		if (!ratio) {
			pr_err("Read a zero ratio, force tsc ratio to 4 ...\n");
			ratio = 4;
		}

		/* Compute FSB */
		rdmsr(MSR_FSB_FREQ, lo, hi);
		pr_debug("Actual FSB frequency detected by SOC 0x%x : %x\n",
			hi, lo);

		bus_freq = lo & 0x7;
		pr_debug("bus_freq = 0x%x\n", bus_freq);

		if (bus_freq == 0)
			fsb = FSB_FREQ_100SKU;
		else if (bus_freq == 1)
			fsb = FSB_FREQ_100SKU;
		else if (bus_freq == 2)
			fsb = FSB_FREQ_133SKU;
		else if (bus_freq == 3)
			fsb = FSB_FREQ_167SKU;
		else if (bus_freq == 4)
			fsb = FSB_FREQ_83SKU;
		else if (bus_freq == 5)
			fsb = FSB_FREQ_400SKU;
		else if (bus_freq == 6)
			fsb = FSB_FREQ_267SKU;
		else if (bus_freq == 7)
			fsb = FSB_FREQ_333SKU;
		else {
			BUG();
			pr_err("Invalid bus_freq! Setting to minimal value!\n");
			fsb = FSB_FREQ_100SKU;
		}

		/* TSC = FSB Freq * Resolved HFM Ratio */
		fast_calibrate = ratio * fsb;
		pr_debug("calculate tangier tsc %lu KHz\n", fast_calibrate);

		/* ************************************ */
		/* Calculate Local APIC Timer Frequency */
		/* ************************************ */
		lapic_timer_frequency = (fsb * 1000) / HZ;

		pr_debug("Setting lapic_timer_frequency = %d\n",
			lapic_timer_frequency);

		/* mark tsc clocksource as reliable */
		set_cpu_cap(&boot_cpu_data, X86_FEATURE_TSC_RELIABLE);

		if (fast_calibrate)
			return fast_calibrate;
	}
	return 0;
}

/* Allow user to enable simulator quirks settings for kernel */
static int __init set_simulation_platform(char *str)
{
	int platform;

	__intel_mid_sim_platform = INTEL_MID_CPU_SIMULATION_NONE;
	if (get_option(&str, &platform)) {
		__intel_mid_sim_platform = platform;
		pr_info("simulator mode %d enabled.\n",
			__intel_mid_sim_platform);
		return 0;
	}

	return -EINVAL;
}
early_param("mrfld_simulation", set_simulation_platform);

static void __init tangier_time_init(void)
{
	/* [REVERT ME] ARAT capability not set in VP. Force setting */
	if (intel_mid_identify_sim() == INTEL_MID_CPU_SIMULATION_VP ||
	    intel_mid_identify_sim() == INTEL_MID_CPU_SIMULATION_HVP)
		set_cpu_cap(&boot_cpu_data, X86_FEATURE_ARAT);

	if (intel_mid_timer_init)
		intel_mid_timer_init();
}

static void __init tangier_arch_setup(void)
{
	x86_platform.calibrate_tsc = tangier_calibrate_tsc;
	intel_mid_timer_init = x86_init.timers.timer_init;
	x86_init.timers.timer_init = tangier_time_init;
}

void *get_tangier_ops()
{
	return &tangier_ops;
}

/* piggy back on anniedale ops right now */
void *get_anniedale_ops()
{
	return &tangier_ops;
}
