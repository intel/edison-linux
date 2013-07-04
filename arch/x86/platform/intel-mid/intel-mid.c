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

/*
 * Parsing GPIO table first, since the DEVS table will need this table
 * to map the pin name to the actual pin.
 */
static struct sfi_gpio_table_entry *gpio_table;
static int gpio_num_entry;

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

static int get_gpio_by_name(const char *name)
{
	struct sfi_gpio_table_entry *pentry = gpio_table;
	int i;

	if (!pentry)
		return -1;
	for (i = 0; i < gpio_num_entry; i++, pentry++) {
		if (!strncmp(name, pentry->pin_name, SFI_NAME_LEN))
			return pentry->pin_no;
	}
	return -1;
}

/*
 * Here defines the array of devices platform data that IAFW would export
 * through SFI "DEVS" table, we use name and type to match the device and
 * its platform data.
 */
struct devs_id {
	char name[SFI_NAME_LEN + 1];
	u8 type;
	u8 delay;
	void *(*get_platform_data)(void *info);
};

/* the offset for the mapping of global gpio pin to irq */
#define INTEL_MID_IRQ_OFFSET 0x100

static void __init *pmic_gpio_platform_data(void *info)
{
	static struct intel_pmic_gpio_platform_data pmic_gpio_pdata;
	int gpio_base = get_gpio_by_name("pmic_gpio_base");

	if (gpio_base == -1)
		gpio_base = 64;
	pmic_gpio_pdata.gpio_base = gpio_base;
	pmic_gpio_pdata.irq_base = gpio_base + INTEL_MID_IRQ_OFFSET;
	pmic_gpio_pdata.gpiointr = 0xffffeff8;

	return &pmic_gpio_pdata;
}

static void __init *max3111_platform_data(void *info)
{
	struct spi_board_info *spi_info = info;
	int intr = get_gpio_by_name("max3111_int");

	spi_info->mode = SPI_MODE_0;
	if (intr == -1)
		return NULL;
	spi_info->irq = intr + INTEL_MID_IRQ_OFFSET;
	return NULL;
}

/* we have multiple max7315 on the board ... */
#define MAX7315_NUM 2
static void __init *max7315_platform_data(void *info)
{
	static struct pca953x_platform_data max7315_pdata[MAX7315_NUM];
	static int nr;
	struct pca953x_platform_data *max7315 = &max7315_pdata[nr];
	struct i2c_board_info *i2c_info = info;
	int gpio_base, intr;
	char base_pin_name[SFI_NAME_LEN + 1];
	char intr_pin_name[SFI_NAME_LEN + 1];

	if (nr == MAX7315_NUM) {
		pr_err("too many max7315s, we only support %d\n",
				MAX7315_NUM);
		return NULL;
	}
	/* we have several max7315 on the board, we only need load several
	 * instances of the same pca953x driver to cover them
	 */
	strcpy(i2c_info->type, "max7315");
	if (nr++) {
		sprintf(base_pin_name, "max7315_%d_base", nr);
		sprintf(intr_pin_name, "max7315_%d_int", nr);
	} else {
		strcpy(base_pin_name, "max7315_base");
		strcpy(intr_pin_name, "max7315_int");
	}

	gpio_base = get_gpio_by_name(base_pin_name);
	intr = get_gpio_by_name(intr_pin_name);

	if (gpio_base == -1)
		return NULL;
	max7315->gpio_base = gpio_base;
	if (intr != -1) {
		i2c_info->irq = intr + INTEL_MID_IRQ_OFFSET;
		max7315->irq_base = gpio_base + INTEL_MID_IRQ_OFFSET;
	} else {
		i2c_info->irq = -1;
		max7315->irq_base = -1;
	}
	return max7315;
}

static void *tca6416_platform_data(void *info)
{
	static struct pca953x_platform_data tca6416;
	struct i2c_board_info *i2c_info = info;
	int gpio_base, intr;
	char base_pin_name[SFI_NAME_LEN + 1];
	char intr_pin_name[SFI_NAME_LEN + 1];

	strcpy(i2c_info->type, "tca6416");
	strcpy(base_pin_name, "tca6416_base");
	strcpy(intr_pin_name, "tca6416_int");

	gpio_base = get_gpio_by_name(base_pin_name);
	intr = get_gpio_by_name(intr_pin_name);

	if (gpio_base == -1)
		return NULL;
	tca6416.gpio_base = gpio_base;
	if (intr != -1) {
		i2c_info->irq = intr + INTEL_MID_IRQ_OFFSET;
		tca6416.irq_base = gpio_base + INTEL_MID_IRQ_OFFSET;
	} else {
		i2c_info->irq = -1;
		tca6416.irq_base = -1;
	}
	return &tca6416;
}

static void *mpu3050_platform_data(void *info)
{
	struct i2c_board_info *i2c_info = info;
	int intr = get_gpio_by_name("mpu3050_int");

	if (intr == -1)
		return NULL;

	i2c_info->irq = intr + INTEL_MID_IRQ_OFFSET;
	return NULL;
}

static void __init *emc1403_platform_data(void *info)
{
	static short intr2nd_pdata;
	struct i2c_board_info *i2c_info = info;
	int intr = get_gpio_by_name("thermal_int");
	int intr2nd = get_gpio_by_name("thermal_alert");

	if (intr == -1 || intr2nd == -1)
		return NULL;

	i2c_info->irq = intr + INTEL_MID_IRQ_OFFSET;
	intr2nd_pdata = intr2nd + INTEL_MID_IRQ_OFFSET;

	return &intr2nd_pdata;
}

static void __init *lis331dl_platform_data(void *info)
{
	static short intr2nd_pdata;
	struct i2c_board_info *i2c_info = info;
	int intr = get_gpio_by_name("accel_int");
	int intr2nd = get_gpio_by_name("accel_2");

	if (intr == -1 || intr2nd == -1)
		return NULL;

	i2c_info->irq = intr + INTEL_MID_IRQ_OFFSET;
	intr2nd_pdata = intr2nd + INTEL_MID_IRQ_OFFSET;

	return &intr2nd_pdata;
}

static void __init *no_platform_data(void *info)
{
	return NULL;
}

static struct resource msic_resources[] = {
	{
		.start	= INTEL_MSIC_IRQ_PHYS_BASE,
		.end	= INTEL_MSIC_IRQ_PHYS_BASE + 64 - 1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct intel_msic_platform_data msic_pdata;

static struct platform_device msic_device = {
	.name		= "intel_msic",
	.id		= -1,
	.dev		= {
		.platform_data	= &msic_pdata,
	},
	.num_resources	= ARRAY_SIZE(msic_resources),
	.resource	= msic_resources,
};

static inline bool intel_mid_has_msic(void)
{
	return intel_mid_identify_cpu() == INTEL_MID_CPU_CHIP_PENWELL;
}

static int msic_scu_status_change(struct notifier_block *nb,
				  unsigned long code, void *data)
{
	if (code == SCU_DOWN) {
		platform_device_unregister(&msic_device);
		return 0;
	}

	return platform_device_register(&msic_device);
}

static int __init msic_init(void)
{
	static struct notifier_block msic_scu_notifier = {
		.notifier_call	= msic_scu_status_change,
	};

	/*
	 * We need to be sure that the SCU IPC is ready before MSIC device
	 * can be registered.
	 */
	if (intel_mid_has_msic())
		intel_scu_notifier_add(&msic_scu_notifier);

	return 0;
}
arch_initcall(msic_init);

/*
 * msic_generic_platform_data - sets generic platform data for the block
 * @info: pointer to the SFI device table entry for this block
 * @block: MSIC block
 *
 * Function sets IRQ number from the SFI table entry for given device to
 * the MSIC platform data.
 */
static void *msic_generic_platform_data(void *info, enum intel_msic_block block)
{
	struct sfi_device_table_entry *entry = info;

	BUG_ON(block < 0 || block >= INTEL_MSIC_BLOCK_LAST);
	msic_pdata.irq[block] = entry->irq;

	return no_platform_data(info);
}

static void *msic_battery_platform_data(void *info)
{
	return msic_generic_platform_data(info, INTEL_MSIC_BLOCK_BATTERY);
}

static void *msic_gpio_platform_data(void *info)
{
	static struct intel_msic_gpio_pdata pdata;
	int gpio = get_gpio_by_name("msic_gpio_base");

	if (gpio < 0)
		return NULL;

	pdata.gpio_base = gpio;
	msic_pdata.gpio = &pdata;

	return msic_generic_platform_data(info, INTEL_MSIC_BLOCK_GPIO);
}

static void *msic_audio_platform_data(void *info)
{
	struct platform_device *pdev;

	pdev = platform_device_register_simple("sst-platform", -1, NULL, 0);
	if (IS_ERR(pdev)) {
		pr_err("failed to create audio platform device\n");
		return NULL;
	}

	return msic_generic_platform_data(info, INTEL_MSIC_BLOCK_AUDIO);
}

static void *msic_power_btn_platform_data(void *info)
{
	return msic_generic_platform_data(info, INTEL_MSIC_BLOCK_POWER_BTN);
}

static void *msic_ocd_platform_data(void *info)
{
	static struct intel_msic_ocd_pdata pdata;
	int gpio = get_gpio_by_name("ocd_gpio");

	if (gpio < 0)
		return NULL;

	pdata.gpio = gpio;
	msic_pdata.ocd = &pdata;

	return msic_generic_platform_data(info, INTEL_MSIC_BLOCK_OCD);
}

static void *msic_thermal_platform_data(void *info)
{
	return msic_generic_platform_data(info, INTEL_MSIC_BLOCK_THERMAL);
}

/* tc35876x DSI-LVDS bridge chip and panel platform data */
static void *tc35876x_platform_data(void *data)
{
	static struct tc35876x_platform_data pdata;

	/* gpio pins set to -1 will not be used by the driver */
	pdata.gpio_bridge_reset = get_gpio_by_name("LCMB_RXEN");
	pdata.gpio_panel_bl_en = get_gpio_by_name("6S6P_BL_EN");
	pdata.gpio_panel_vadd = get_gpio_by_name("EN_VREG_LCD_V3P3");

	return &pdata;
}

static const struct devs_id __initconst device_ids[] = {
	{"bma023", SFI_DEV_TYPE_I2C, 1, &no_platform_data},
	{"pmic_gpio", SFI_DEV_TYPE_SPI, 1, &pmic_gpio_platform_data},
	{"pmic_gpio", SFI_DEV_TYPE_IPC, 1, &pmic_gpio_platform_data},
	{"spi_max3111", SFI_DEV_TYPE_SPI, 0, &max3111_platform_data},
	{"i2c_max7315", SFI_DEV_TYPE_I2C, 1, &max7315_platform_data},
	{"i2c_max7315_2", SFI_DEV_TYPE_I2C, 1, &max7315_platform_data},
	{"tca6416", SFI_DEV_TYPE_I2C, 1, &tca6416_platform_data},
	{"emc1403", SFI_DEV_TYPE_I2C, 1, &emc1403_platform_data},
	{"i2c_accel", SFI_DEV_TYPE_I2C, 0, &lis331dl_platform_data},
	{"pmic_audio", SFI_DEV_TYPE_IPC, 1, &no_platform_data},
	{"mpu3050", SFI_DEV_TYPE_I2C, 1, &mpu3050_platform_data},
	{"i2c_disp_brig", SFI_DEV_TYPE_I2C, 0, &tc35876x_platform_data},

	/* MSIC subdevices */
	{"msic_battery", SFI_DEV_TYPE_IPC, 1, &msic_battery_platform_data},
	{"msic_gpio", SFI_DEV_TYPE_IPC, 1, &msic_gpio_platform_data},
	{"msic_audio", SFI_DEV_TYPE_IPC, 1, &msic_audio_platform_data},
	{"msic_power_btn", SFI_DEV_TYPE_IPC, 1, &msic_power_btn_platform_data},
	{"msic_ocd", SFI_DEV_TYPE_IPC, 1, &msic_ocd_platform_data},
	{"msic_thermal", SFI_DEV_TYPE_IPC, 1, &msic_thermal_platform_data},

	{},
};

#define MAX_IPCDEVS	24
static struct platform_device *ipc_devs[MAX_IPCDEVS];
static int ipc_next_dev;

#define MAX_SCU_SPI	24
static struct spi_board_info *spi_devs[MAX_SCU_SPI];
static int spi_next_dev;

#define MAX_SCU_I2C	24
static struct i2c_board_info *i2c_devs[MAX_SCU_I2C];
static int i2c_bus[MAX_SCU_I2C];
static int i2c_next_dev;

static void __init intel_scu_device_register(struct platform_device *pdev)
{
	if (ipc_next_dev == MAX_IPCDEVS)
		pr_err("too many SCU IPC devices");
	else
		ipc_devs[ipc_next_dev++] = pdev;
}

static void __init intel_scu_spi_device_register(struct spi_board_info *sdev)
{
	struct spi_board_info *new_dev;

	if (spi_next_dev == MAX_SCU_SPI) {
		pr_err("too many SCU SPI devices");
		return;
	}

	new_dev = kzalloc(sizeof(*sdev), GFP_KERNEL);
	if (!new_dev) {
		pr_err("failed to alloc mem for delayed spi dev %s\n",
			sdev->modalias);
		return;
	}
	memcpy(new_dev, sdev, sizeof(*sdev));

	spi_devs[spi_next_dev++] = new_dev;
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
	memcpy(new_dev, idev, sizeof(*idev));

	i2c_bus[i2c_next_dev] = bus;
	i2c_devs[i2c_next_dev++] = new_dev;
}

BLOCKING_NOTIFIER_HEAD(intel_scu_notifier);
EXPORT_SYMBOL_GPL(intel_scu_notifier);

/* Called by IPC driver */
void intel_scu_devices_create(void)
{
	int i;

	for (i = 0; i < ipc_next_dev; i++)
		platform_device_add(ipc_devs[i]);

	for (i = 0; i < spi_next_dev; i++)
		spi_register_board_info(spi_devs[i], 1);

	for (i = 0; i < i2c_next_dev; i++) {
		struct i2c_adapter *adapter;
		struct i2c_client *client;

		adapter = i2c_get_adapter(i2c_bus[i]);
		if (adapter) {
			client = i2c_new_device(adapter, i2c_devs[i]);
			if (!client)
				pr_err("can't create i2c device %s\n",
					i2c_devs[i]->type);
		} else
			i2c_register_board_info(i2c_bus[i], i2c_devs[i], 1);
	}
	intel_scu_notifier_post(SCU_AVAILABLE, NULL);
}
EXPORT_SYMBOL_GPL(intel_scu_devices_create);

/* Called by IPC driver */
void intel_scu_devices_destroy(void)
{
	int i;

	intel_scu_notifier_post(SCU_DOWN, NULL);

	for (i = 0; i < ipc_next_dev; i++)
		platform_device_del(ipc_devs[i]);
}
EXPORT_SYMBOL_GPL(intel_scu_devices_destroy);

static void __init install_irq_resource(struct platform_device *pdev, int irq)
{
	/* Single threaded */
	static struct resource __initdata res = {
		.name = "IRQ",
		.flags = IORESOURCE_IRQ,
	};
	res.start = irq;
	platform_device_add_resources(pdev, &res, 1);
}

static void __init sfi_handle_ipc_dev(struct sfi_device_table_entry *entry)
{
	const struct devs_id *dev = device_ids;
	struct platform_device *pdev;
	void *pdata = NULL;

	while (dev->name[0]) {
		if (dev->type == SFI_DEV_TYPE_IPC &&
			!strncmp(dev->name, entry->name, SFI_NAME_LEN)) {
			pdata = dev->get_platform_data(entry);
			break;
		}
		dev++;
	}

	/*
	 * On Medfield the platform device creation is handled by the MSIC
	 * MFD driver so we don't need to do it here.
	 */
	if (intel_mid_has_msic())
		return;

	pdev = platform_device_alloc(entry->name, 0);
	if (pdev == NULL) {
		pr_err("out of memory for SFI platform device '%s'.\n",
			entry->name);
		return;
	}
	install_irq_resource(pdev, entry->irq);

	pdev->dev.platform_data = pdata;
	intel_scu_device_register(pdev);
}

static void __init sfi_handle_spi_dev(struct spi_board_info *spi_info)
{
	const struct devs_id *dev = device_ids;
	void *pdata = NULL;

	while (dev->name[0]) {
		if (dev->type == SFI_DEV_TYPE_SPI &&
			!strncmp(dev->name, spi_info->modalias,
						SFI_NAME_LEN)) {
			pdata = dev->get_platform_data(spi_info);
			break;
		}
		dev++;
	}
	spi_info->platform_data = pdata;
	if (dev->delay)
		intel_scu_spi_device_register(spi_info);
	else
		spi_register_board_info(spi_info, 1);
}

static void __init sfi_handle_i2c_dev(int bus, struct i2c_board_info *i2c_info)
{
	const struct devs_id *dev = device_ids;
	void *pdata = NULL;

	while (dev->name[0]) {
		if (dev->type == SFI_DEV_TYPE_I2C &&
			!strncmp(dev->name, i2c_info->type, SFI_NAME_LEN)) {
			pdata = dev->get_platform_data(i2c_info);
			break;
		}
		dev++;
	}
	i2c_info->platform_data = pdata;

	if (dev->delay)
		intel_scu_i2c_device_register(bus, i2c_info);
	else
		i2c_register_board_info(bus, i2c_info, 1);
}


static int __init sfi_parse_devs(struct sfi_table_header *table)
{
	struct sfi_table_simple *sb;
	struct sfi_device_table_entry *pentry;
	struct spi_board_info spi_info;
	struct i2c_board_info i2c_info;
	int num, i, bus;
	int ioapic;
	struct io_apic_irq_attr irq_attr;

	sb = (struct sfi_table_simple *)table;
	num = SFI_GET_NUM_ENTRIES(sb, struct sfi_device_table_entry);
	pentry = (struct sfi_device_table_entry *)sb->pentry;

	for (i = 0; i < num; i++, pentry++) {
		int irq = pentry->irq;

		if (irq != (u8)0xff) { /* native RTE case */
			/* these SPI2 devices are not exposed to system as PCI
			 * devices, but they have separate RTE entry in IOAPIC
			 * so we have to enable them one by one here
			 */
			ioapic = mp_find_ioapic(irq);
			irq_attr.ioapic = ioapic;
			irq_attr.ioapic_pin = irq;
			irq_attr.trigger = 1;
			irq_attr.polarity = 1;
			io_apic_set_pci_routing(NULL, irq, &irq_attr);
		} else
			irq = 0; /* No irq */

		switch (pentry->type) {
		case SFI_DEV_TYPE_IPC:
			pr_debug("info[%2d]: IPC bus, name = %16.16s, "
				"irq = 0x%2x\n", i, pentry->name, pentry->irq);
			sfi_handle_ipc_dev(pentry);
			break;
		case SFI_DEV_TYPE_SPI:
			memset(&spi_info, 0, sizeof(spi_info));
			strncpy(spi_info.modalias, pentry->name, SFI_NAME_LEN);
			spi_info.irq = irq;
			spi_info.bus_num = pentry->host_num;
			spi_info.chip_select = pentry->addr;
			spi_info.max_speed_hz = pentry->max_freq;
			pr_debug("info[%2d]: SPI bus = %d, name = %16.16s, "
				"irq = 0x%2x, max_freq = %d, cs = %d\n", i,
				spi_info.bus_num,
				spi_info.modalias,
				spi_info.irq,
				spi_info.max_speed_hz,
				spi_info.chip_select);
			sfi_handle_spi_dev(&spi_info);
			break;
		case SFI_DEV_TYPE_I2C:
			memset(&i2c_info, 0, sizeof(i2c_info));
			bus = pentry->host_num;
			strncpy(i2c_info.type, pentry->name, SFI_NAME_LEN);
			i2c_info.irq = irq;
			i2c_info.addr = pentry->addr;
			pr_debug("info[%2d]: I2C bus = %d, name = %16.16s, "
				"irq = 0x%2x, addr = 0x%x\n", i, bus,
				i2c_info.type,
				i2c_info.irq,
				i2c_info.addr);
			sfi_handle_i2c_dev(bus, &i2c_info);
			break;
		case SFI_DEV_TYPE_UART:
		case SFI_DEV_TYPE_HSI:
		default:
			break;
		}
	}
	return 0;
}

static int __init intel_mid_platform_init(void)
{
	/* Get MFD Validation SFI OEMB Layout */
	sfi_table_parse(SFI_SIG_GPIO, NULL, NULL, sfi_parse_gpio);
	sfi_table_parse(SFI_SIG_DEVS, NULL, NULL, sfi_parse_devs);
	return 0;
}
arch_initcall(intel_mid_platform_init);

/*
 * we will search these buttons in SFI GPIO table (by name)
 * and register them dynamically. Please add all possible
 * buttons here, we will shrink them if no GPIO found.
 */
static struct gpio_keys_button gpio_button[] = {
	{KEY_POWER,		-1, 1, "power_btn",	EV_KEY, 0, 3000},
	{KEY_PROG1,		-1, 1, "prog_btn1",	EV_KEY, 0, 20},
	{KEY_PROG2,		-1, 1, "prog_btn2",	EV_KEY, 0, 20},
	{SW_LID,		-1, 1, "lid_switch",	EV_SW,  0, 20},
	{KEY_VOLUMEUP,		-1, 1, "vol_up",	EV_KEY, 0, 20},
	{KEY_VOLUMEDOWN,	-1, 1, "vol_down",	EV_KEY, 0, 20},
	{KEY_CAMERA,		-1, 1, "camera_full",	EV_KEY, 0, 20},
	{KEY_CAMERA_FOCUS,	-1, 1, "camera_half",	EV_KEY, 0, 20},
	{SW_KEYPAD_SLIDE,	-1, 1, "MagSw1",	EV_SW,  0, 20},
	{SW_KEYPAD_SLIDE,	-1, 1, "MagSw2",	EV_SW,  0, 20},
};

static struct gpio_keys_platform_data intel_mid_gpio_keys = {
	.buttons	= gpio_button,
	.rep		= 1,
	.nbuttons	= -1, /* will fill it after search */
};

static struct platform_device pb_device = {
	.name		= "gpio-keys",
	.id		= -1,
	.dev		= {
		.platform_data	= &intel_mid_gpio_keys,
	},
};

/*
 * Shrink the non-existent buttons, register the gpio button
 * device if there is some
 */
static int __init pb_keys_init(void)
{
	struct gpio_keys_button *gb = gpio_button;
	int i, num, good = 0;

	num = sizeof(gpio_button) / sizeof(struct gpio_keys_button);
	for (i = 0; i < num; i++) {
		gb[i].gpio = get_gpio_by_name(gb[i].desc);
		pr_debug("info[%2d]: name = %s, gpio = %d\n", i, gb[i].desc,
							gb[i].gpio);
		if (gb[i].gpio == -1)
			continue;

		if (i != good)
			gb[good] = gb[i];
		good++;
	}

	if (good) {
		intel_mid_gpio_keys.nbuttons = good;
		return platform_device_register(&pb_device);
	}
	return 0;
}
late_initcall(pb_keys_init);
