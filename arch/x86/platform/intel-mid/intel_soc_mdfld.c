/*
 * intel_soc_mdfld.c - This driver provides utility api's for medfield
 * platform
 * Copyright (c) 2012, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *
 */

#include "intel_soc_pmu.h"

/* To CLEAR C6 offload Bit(LSB) in MSR 120 */
static inline void clear_c6offload_bit(void)
{
	u32 msr_low, msr_high;

	rdmsr(MSR_C6OFFLOAD_CTL_REG, msr_low, msr_high);
	msr_low = msr_low & ~MSR_C6OFFLOAD_SET_LOW;
	msr_high = msr_high & ~MSR_C6OFFLOAD_SET_HIGH;
	wrmsr(MSR_C6OFFLOAD_CTL_REG, msr_low, msr_high);
}

/* To SET C6 offload Bit(LSB) in MSR 120 */
static inline void set_c6offload_bit(void)
{
	u32 msr_low, msr_high;

	rdmsr(MSR_C6OFFLOAD_CTL_REG, msr_low, msr_high);
	msr_low = msr_low | MSR_C6OFFLOAD_SET_LOW;
	msr_high = msr_high | MSR_C6OFFLOAD_SET_HIGH;
	wrmsr(MSR_C6OFFLOAD_CTL_REG, msr_low, msr_high);
}

static bool mfld_pmu_enter(int s0ix_state)
{
	u32 s0ix_value;
	u32 ssw_val;
	int num_retry = PMU_MISC_SET_TIMEOUT;

	s0ix_value = get_s0ix_val_set_pm_ssc(s0ix_state);

	clear_c6offload_bit();

	/* issue a command to SCU */
	writel(s0ix_value, &mid_pmu_cxt->pmu_reg->pm_cmd);

	pmu_log_command(s0ix_value, NULL);

	do {
		if (readl(&mid_pmu_cxt->pmu_reg->pm_msic))
			break;
		udelay(1);
	} while (--num_retry);

	if (!num_retry && !readl(&mid_pmu_cxt->pmu_reg->pm_msic))
		WARN(1, "%s: pm_msic not set.\n", __func__);

	num_retry = PMU_C6OFFLOAD_ACCESS_TIMEOUT;

	/* At this point we have committed an S0ix command
	 * will have to wait for the SCU s0ix complete
	 * intertupt to proceed further.
	 */
	mid_pmu_cxt->s0ix_entered = s0ix_state;

	if (s0ix_value == S0I3_VALUE) {
		do {
			ssw_val = readl(mid_pmu_cxt->base_addr.offload_reg);
			if ((ssw_val & C6OFFLOAD_BIT_MASK) ==  C6OFFLOAD_BIT) {
				set_c6offload_bit();
				break;
			}

			udelay(1);
		} while (--num_retry);

		if (unlikely(!num_retry)) {
			WARN(1, "mid_pmu: error cpu offload bit not set.\n");
			pmu_stat_clear();
			return false;
		}
	}

	return true;
}

static void mfld_pmu_wakeup(void)
{

	/* Wakeup allother CPU's */
	if (mid_pmu_cxt->s0ix_entered)
		apic->send_IPI_allbutself(RESCHEDULE_VECTOR);

	clear_c6offload_bit();
}

static void mfld_pmu_remove(void)
{
	/* Freeing up memory allocated for PMU1 & PMU2 */
	iounmap(mid_pmu_cxt->base_addr.offload_reg);
	mid_pmu_cxt->base_addr.offload_reg = NULL;

}

static pci_power_t mfld_pmu_choose_state(int device_lss)
{
	pci_power_t state;

	switch (device_lss) {
	case PMU_SECURITY_LSS_04:
		state = PCI_D2;
		break;

	case PMU_USB_OTG_LSS_06:
	case PMU_USB_HSIC_LSS_07:
	case PMU_UART2_LSS_41:
		state = PCI_D1;
		break;

	default:
		state = PCI_D3hot;
		break;
	}

	return state;
}

static int mfld_pmu_init(void)
{
	int ret = PMU_SUCCESS;

	/* Map the memory of offload_reg */
	mid_pmu_cxt->base_addr.offload_reg =
				ioremap_nocache(C6_OFFLOAD_REG_ADDR, 4);
	if (mid_pmu_cxt->base_addr.offload_reg == NULL) {
		dev_dbg(&mid_pmu_cxt->pmu_dev->dev,
		"Unable to map the offload_reg address space\n");
		ret = PMU_FAILED;
		goto out_err;
	}

	mid_pmu_cxt->s3_hint = C6_HINT;

out_err:
	return ret;
}

/**
 *      platform_set_pmu_ops - Set the global pmu method table.
 *      @ops:   Pointer to ops structure.
 */
void platform_set_pmu_ops(void)
{
	pmu_ops = &mfld_pmu_ops;
}

struct platform_pmu_ops mfld_pmu_ops = {
	.init	 = mfld_pmu_init,
	.enter	 = mfld_pmu_enter,
	.wakeup = mfld_pmu_wakeup,
	.remove = mfld_pmu_remove,
	.pci_choose_state = mfld_pmu_choose_state,
	.set_power_state_ops = pmu_set_s0ix_possible,
	.set_s0ix_complete = s0ix_complete,
	.nc_set_power_state = mdfld_clv_nc_set_power_state,
};
