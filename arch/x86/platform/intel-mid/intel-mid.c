/*
 * intel-mid.c: Intel MID platform setup code
 *
 * (C) Copyright 2012 Intel Corporation
 * Author: Jacob Pan (jacob.jun.pan@intel.com)
 * Author: Sathyanarayanan KN(sathyanarayanan.kuppuswamy@intel.com)
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */
#define	SFI_SIG_OEM0	"OEM0"
#define pr_fmt(fmt) "intel_mid: " fmt

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/scatterlist.h>
#include <linux/sfi.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/spinlock.h>

#include <asm/setup.h>
#include <asm/mpspec_def.h>
#include <asm/hw_irq.h>
#include <asm/apic.h>
#include <asm/io_apic.h>
#include <asm/intel-mid.h>
#include <asm/io.h>
#include <asm/i8259.h>
#include <asm/intel_scu_ipc.h>
#include <asm/apb_timer.h>
#include <asm/reboot.h>
#include "intel_mid_weak_decls.h"

/*
 * the clockevent devices on Moorestown/Medfield can be APBT or LAPIC clock,
 * cmdline option x86_intel_mid_timer can be used to override the configuration
 * to prefer one or the other.
 * at runtime, there are basically three timer configurations:
 * 1. per cpu apbt clock only
 * 2. per cpu always-on lapic clocks only, this is Penwell/Medfield only
 * 3. per cpu lapic clock (C3STOP) and one apbt clock, with broadcast.
 *
 * by default (without cmdline option), platform code first detects cpu type
 * to see if we are on lincroft or penwell, then set up both lapic or apbt
 * clocks accordingly.
 * i.e. by default, medfield uses configuration #2, moorestown uses #1.
 * config #3 is supported but not recommended on medfield.
 *
 * rating and feature summary:
 * lapic (with C3STOP) --------- 100
 * apbt (always-on) ------------ 110
 * lapic (always-on,ARAT) ------ 150
 */
__cpuinitdata enum intel_mid_timer_options intel_mid_timer_options;

enum intel_mid_cpu_type __intel_mid_cpu_chip;
EXPORT_SYMBOL_GPL(__intel_mid_cpu_chip);

static int force_cold_boot;
module_param(force_cold_boot, int, 0644);
MODULE_PARM_DESC(force_cold_boot,
		 "Set to Y to force a COLD BOOT instead of a COLD RESET "
		 "on the next reboot system call.");

static void intel_mid_reboot(void)
{
	if (force_cold_boot)
		intel_scu_ipc_simple_command(IPCMSG_COLD_BOOT, 0);
	else
		intel_scu_ipc_simple_command(IPCMSG_COLD_RESET, 0);
}

static unsigned long __init intel_mid_calibrate_tsc(void)
{
	return 0;
}
/* Unified message bus read/write operation */
DEFINE_SPINLOCK(msgbus_lock);

u32 intel_mid_msgbus_read32_raw(u32 cmd)
{
	struct pci_dev *pci_root;
	unsigned long irq_flags;
	u32 data;

	pci_root = pci_get_bus_and_slot(0, PCI_DEVFN(0, 0));

	spin_lock_irqsave(&msgbus_lock, irq_flags);
	pci_write_config_dword(pci_root, PCI_ROOT_MSGBUS_CTRL_REG, cmd);
	pci_read_config_dword(pci_root, PCI_ROOT_MSGBUS_DATA_REG, &data);
	spin_unlock_irqrestore(&msgbus_lock, irq_flags);

	pci_dev_put(pci_root);

	return data;
}
EXPORT_SYMBOL(intel_mid_msgbus_read32_raw);

void intel_mid_msgbus_write32_raw(u32 cmd, u32 data)
{
	struct pci_dev *pci_root;
	unsigned long irq_flags;

	pci_root = pci_get_bus_and_slot(0, PCI_DEVFN(0, 0));

	spin_lock_irqsave(&msgbus_lock, irq_flags);
	pci_write_config_dword(pci_root, PCI_ROOT_MSGBUS_DATA_REG, data);
	pci_write_config_dword(pci_root, PCI_ROOT_MSGBUS_CTRL_REG, cmd);
	spin_unlock_irqrestore(&msgbus_lock, irq_flags);

	pci_dev_put(pci_root);
}
EXPORT_SYMBOL(intel_mid_msgbus_write32_raw);

u32 intel_mid_msgbus_read32(u8 port, u8 addr)
{
	u32 cmd = (PCI_ROOT_MSGBUS_READ << 24) | (port << 16) |
		  (addr << 8) | PCI_ROOT_MSGBUS_DWORD_ENABLE;

	return intel_mid_msgbus_read32_raw(cmd);
}
EXPORT_SYMBOL(intel_mid_msgbus_read32);

void intel_mid_msgbus_write32(u8 port, u8 addr, u32 data)
{
	u32 cmd = (PCI_ROOT_MSGBUS_WRITE << 24) | (port << 16) |
		  (addr << 8) | PCI_ROOT_MSGBUS_DWORD_ENABLE;

	intel_mid_msgbus_write32_raw(cmd, data);
}
EXPORT_SYMBOL(intel_mid_msgbus_write32);

static void __init intel_mid_time_init(void)
{

#ifdef CONFIG_SFI
	sfi_table_parse(SFI_SIG_MTMR, NULL, NULL, sfi_parse_mtmr);
#endif
	switch (intel_mid_timer_options) {
	case INTEL_MID_TIMER_APBT_ONLY:
		break;
	case INTEL_MID_TIMER_LAPIC_APBT:
		x86_init.timers.setup_percpu_clockev = setup_boot_APIC_clock;
		x86_cpuinit.setup_percpu_clockev = setup_secondary_APIC_clock;
		break;
	default:
		if (!boot_cpu_has(X86_FEATURE_ARAT))
			break;
		x86_init.timers.setup_percpu_clockev = setup_boot_APIC_clock;
		x86_cpuinit.setup_percpu_clockev = setup_secondary_APIC_clock;
		return;
	}
	/* we need at least one APB timer */
	pre_init_apic_IRQ0();
	apbt_time_init();
}

static void __cpuinit intel_mid_arch_setup(void)
{
	if (boot_cpu_data.x86 == 6 && boot_cpu_data.x86_model == 0x27)
		__intel_mid_cpu_chip = INTEL_MID_CPU_CHIP_PENWELL;
	else if (boot_cpu_data.x86 == 6 && boot_cpu_data.x86_model == 0x35)
		__intel_mid_cpu_chip = INTEL_MID_CPU_CHIP_CLOVERVIEW;
	else {
		pr_err("Unknown Intel MID CPU (%d:%d), default to Penwell\n",
			boot_cpu_data.x86, boot_cpu_data.x86_model);
		__intel_mid_cpu_chip = INTEL_MID_CPU_CHIP_PENWELL;
	}
}

/* MID systems don't have i8042 controller */
static int intel_mid_i8042_detect(void)
{
	return 0;
}

/*
 * Moorestown does not have external NMI source nor port 0x61 to report
 * NMI status. The possible NMI sources are from pmu as a result of NMI
 * watchdog or lock debug. Reading io port 0x61 results in 0xff which
 * misled NMI handler.
 */
static unsigned char intel_mid_get_nmi_reason(void)
{
	return 0;
}

/*
 * Moorestown specific x86_init function overrides and early setup
 * calls.
 */
void __init x86_intel_mid_early_setup(void)
{
	x86_init.resources.probe_roms = x86_init_noop;
	x86_init.resources.reserve_resources = x86_init_noop;

	x86_init.timers.timer_init = intel_mid_time_init;
	x86_init.timers.setup_percpu_clockev = x86_init_noop;

	x86_init.irqs.pre_vector_init = x86_init_noop;

	x86_init.oem.arch_setup = intel_mid_arch_setup;

	x86_cpuinit.setup_percpu_clockev = apbt_setup_secondary_clock;

	x86_platform.calibrate_tsc = intel_mid_calibrate_tsc;
	x86_platform.i8042_detect = intel_mid_i8042_detect;
	x86_init.timers.wallclock_init = intel_mid_rtc_init;
	x86_platform.get_nmi_reason = intel_mid_get_nmi_reason;

	x86_init.pci.init = intel_mid_pci_init;
	x86_init.pci.fixup_irqs = x86_init_noop;

	legacy_pic = &null_legacy_pic;

	pm_power_off = intel_mid_power_off;
	machine_ops.emergency_restart  = intel_mid_reboot;

	/* Avoid searching for BIOS MP tables */
	x86_init.mpparse.find_smp_config = x86_init_noop;
	x86_init.mpparse.get_smp_config = x86_init_uint_noop;
	set_bit(MP_BUS_ISA, mp_bus_not_pci);
}

/*
 * if user does not want to use per CPU apb timer, just give it a lower rating
 * than local apic timer and skip the late per cpu timer init.
 */
static inline int __init setup_x86_intel_mid_timer(char *arg)
{
	if (!arg)
		return -EINVAL;

	if (strcmp("apbt_only", arg) == 0)
		intel_mid_timer_options = INTEL_MID_TIMER_APBT_ONLY;
	else if (strcmp("lapic_and_apbt", arg) == 0)
		intel_mid_timer_options = INTEL_MID_TIMER_LAPIC_APBT;
	else {
		pr_warn("X86 INTEL_MID timer option %s not recognised"
			   " use x86_intel_mid_timer=apbt_only or lapic_and_apbt\n",
			   arg);
		return -EINVAL;
	}
	return 0;
}
__setup("x86_intel_mid_timer=", setup_x86_intel_mid_timer);
