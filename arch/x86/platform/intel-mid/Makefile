obj-$(CONFIG_X86_INTEL_MID) += intel-mid.o
obj-$(CONFIG_X86_INTEL_MID)	+= intel_mid_vrtc.o
obj-$(CONFIG_EARLY_PRINTK_INTEL_MID)	+= early_printk_intel_mid.o

# SFI specific code
obj-$(CONFIG_SFI) += intel_mid_sfi.o

# platform configuration for board devices
obj-y += device_libs/

# SoC specific files
obj-$(CONFIG_X86_INTEL_MID) += mfld.o mrfl.o
obj-$(CONFIG_X86_WANT_INTEL_MID) += vlv2.o
obj-$(CONFIG_X86_INTEL_MID) += intel_mid_scu.o

>>>>>>> 6a9f567... intel_mid: Added intel mid scu remoteproc init code.
# BOARD files
obj-$(CONFIG_X86_INTEL_MID) += board.o
