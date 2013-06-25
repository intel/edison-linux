/*
 * intel-mid.h: Intel MID specific setup code
 *
 * (C) Copyright 2009 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */
#ifndef _ASM_X86_INTEL_MID_H
#define _ASM_X86_INTEL_MID_H

#include <linux/sfi.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <asm/spid.h>

#define INTEL_MID_SSN_SIZE	32

extern struct soft_platform_id spid;
extern char intel_mid_ssn[INTEL_MID_SSN_SIZE + 1];

#ifdef CONFIG_SFI
extern int get_gpio_by_name(const char *name);
#else
static inline int get_gpio_by_name(const char *name) { return -1; }
#endif

extern int intel_mid_pci_init(void);
extern void intel_delayed_device_register(void *dev,
			void (*delayed_callback)(void *dev_desc));
extern void intel_scu_device_register(struct platform_device *pdev);
extern struct devs_id *get_device_id(u8 type, char *name);
extern int __init sfi_parse_mrtc(struct sfi_table_header *table);
extern int __init sfi_parse_mtmr(struct sfi_table_header *table);
extern int sfi_mrtc_num;
extern struct sfi_rtc_table_entry sfi_mrtc_array[];

/* OEMB table */
struct sfi_table_oemb {
	struct sfi_table_header header;
	u32 board_id;
	u32 board_fab;
	u8 iafw_major_version;
	u8 iafw_main_version;
	u8 val_hooks_major_version;
	u8 val_hooks_minor_version;
	u8 ia_suppfw_major_version;
	u8 ia_suppfw_minor_version;
	u8 scu_runtime_major_version;
	u8 scu_runtime_minor_version;
	u8 ifwi_major_version;
	u8 ifwi_minor_version;
	struct soft_platform_id spid;
	u8 ssn[INTEL_MID_SSN_SIZE];
} __packed;

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
	void (*device_handler)(struct sfi_device_table_entry *pentry,
				struct devs_id *dev);
	/* Custom handler for devices */
	u8 trash_itp;/* true if this driver uses pin muxed with XDB connector */
};

#define SD_NAME_SIZE 16
/**
 * struct sd_board_info - template for device creation
 * @name: Initializes sdio_device.name; identifies the driver.
 * @bus_num: board-specific identifier for a given SDIO controller.
 * @board_ref_clock: Initializes sd_device.board_ref_clock;
 * @platform_data: Initializes sd_device.platform_data; the particular
 *      data stored there is driver-specific.
 *
 */
struct sd_board_info {
	char            name[SD_NAME_SIZE];
	int             bus_num;
	unsigned short  addr;
	u32             board_ref_clock;
	void            *platform_data;
};


/*
 * Medfield is the follow-up of Moorestown, it combines two chip solution into
 * one. Other than that it also added always-on and constant tsc and lapic
 * timers. Medfield is the platform name, and the chip name is called Penwell
 * we treat Medfield/Penwell as a variant of Moorestown. Penwell can be
 * identified via MSRs.
 */
enum intel_mid_cpu_type {
	/* 1 was Moorestown */
	INTEL_MID_CPU_CHIP_PENWELL = 2,
	INTEL_MID_CPU_CHIP_CLOVERVIEW,
};

extern enum intel_mid_cpu_type __intel_mid_cpu_chip;

#ifdef CONFIG_X86_INTEL_MID

static inline enum intel_mid_cpu_type intel_mid_identify_cpu(void)
{
	return __intel_mid_cpu_chip;
}

#else /* !CONFIG_X86_INTEL_MID */

#define intel_mid_identify_cpu()    (0)

#endif /* !CONFIG_X86_INTEL_MID */

enum intel_mid_timer_options {
	INTEL_MID_TIMER_DEFAULT,
	INTEL_MID_TIMER_APBT_ONLY,
	INTEL_MID_TIMER_LAPIC_APBT,
};

extern enum intel_mid_timer_options intel_mid_timer_options;

#define spid_attr(_name) \
static struct kobj_attribute _name##_attr = { \
	.attr = {                             \
		.name = __stringify(_name),   \
		.mode = 0444,                 \
	},                                    \
	.show   = _name##_show,               \
}

/*
 * Penwell uses spread spectrum clock, so the freq number is not exactly
 * the same as reported by MSR based on SDM.
 */
#define PENWELL_FSB_FREQ_83SKU         83200
#define PENWELL_FSB_FREQ_100SKU        99840

#define SFI_MTMR_MAX_NUM 8
#define SFI_MRTC_MAX	8

extern struct console early_mrst_console;
extern void mrst_early_console_init(void);

extern struct console early_mrfld_console;
extern void mrfld_early_console_init(void);

extern struct console early_hsu_console;
extern void hsu_early_console_init(const char *);

extern void intel_scu_devices_create(void);
extern void intel_scu_devices_destroy(void);

/* VRTC timer */
#define MRST_VRTC_MAP_SZ	(1024)
/*#define MRST_VRTC_PGOFFSET	(0xc00) */

extern void intel_mid_rtc_init(void);

#define INTEL_MID_IRQ_OFFSET 0x100
#endif /* _ASM_X86_INTEL_MID_H */
