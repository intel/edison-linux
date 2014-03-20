obj-$(CONFIG_X86_INTEL_MID) += intel-mid.o
obj-$(CONFIG_X86_INTEL_MID)	+= intel_mid_vrtc.o
obj-$(CONFIG_EARLY_PRINTK_INTEL_MID)	+= early_printk_intel_mid.o

# SFI specific code
obj-$(CONFIG_SFI) += intel_mid_sfi.o

# platform configuration for board devices
obj-y += device_libs/

# SoC specific files
obj-$(CONFIG_X86_INTEL_MID) += mfld.o mrfl.o
obj-$(CONFIG_X86_WANT_INTEL_MID) += intel_mid_pcihelpers.o
obj-$(CONFIG_X86_INTEL_MID) += intel_mid_scu.o

# BOARD files
obj-$(CONFIG_X86_INTEL_MID) += board.o

# PMU driver
obj-$(CONFIG_ATOM_SOC_POWER) += intel_soc_pmu.o intel_soc_pm_debug.o intel_soc_dump.o
obj-$(CONFIG_REMOVEME_INTEL_ATOM_MDFLD_POWER) += intel_soc_mdfld.o intel_soc_mdfld_clv_common.o
obj-$(CONFIG_REMOVEME_INTEL_ATOM_CLV_POWER) += intel_soc_clv.o intel_soc_mdfld_clv_common.o
obj-$(CONFIG_REMOVEME_INTEL_ATOM_MRFLD_POWER) += intel_soc_mrfld.o
obj-$(CONFIG_REMOVEME_INTEL_ATOM_MRFLD_POWER) += pmu_tng.o

# Debug features driver
obj-$(CONFIG_INTEL_DEBUG_FEATURE) += intel_soc_debug.o
