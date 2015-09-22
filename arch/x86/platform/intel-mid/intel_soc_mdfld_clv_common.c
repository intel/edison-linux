/*
 * intel_soc_mdfld_clv_common.c - This driver provides utility api's common for
 * mdfld and clv platforms
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

static int extended_cstate_mode = MID_S0IX_STATE;
int set_extended_cstate_mode(const char *val, struct kernel_param *kp)
{
	char valcp[5];
	int cstate_mode;

	memcpy(valcp, val, 5);
	valcp[4] = '\0';

	if (strcmp(valcp, "s0i1") == 0)
		cstate_mode = MID_S0I1_STATE;
	else if (strcmp(valcp, "lmp3") == 0)
		cstate_mode = MID_LPMP3_STATE;
	else if (strcmp(valcp, "s0i3") == 0)
		cstate_mode = MID_S0I3_STATE;
	else if (strcmp(valcp, "i1i3") == 0)
		cstate_mode = MID_I1I3_STATE;
	else if (strcmp(valcp, "lpi1") == 0)
		cstate_mode = MID_LPI1_STATE;
	else if (strcmp(valcp, "lpi3") == 0)
		cstate_mode = MID_LPI3_STATE;
	else if (strcmp(valcp, "s0ix") == 0)
		cstate_mode = MID_S0IX_STATE;
	else {
		cstate_mode = 0;
		strncpy(valcp, "none", 5);
	}
	memcpy(s0ix, valcp, 5);

	down(&mid_pmu_cxt->scu_ready_sem);
	extended_cstate_mode = cstate_mode;
	up(&mid_pmu_cxt->scu_ready_sem);

	return 0;
}

int get_extended_cstate_mode(char *buffer, struct kernel_param *kp)
{
	strcpy(buffer, s0ix);
	return 4;
}

/*
 *Decide which state the platfrom can go to based on user and
 *platfrom inputs
*/
static int get_final_state(unsigned long *eax)
{
	int ret = 0;
	int possible = mid_pmu_cxt->s0ix_possible;

	switch (extended_cstate_mode) {
	case MID_S0I1_STATE:
	case MID_S0I3_STATE:
	case MID_I1I3_STATE:
		/* user asks s0i1/s0i3 then only
		 * do s0i1/s0i3, dont do lpmp3
		 */
		if (possible == MID_S0IX_STATE)
			ret = extended_cstate_mode & possible;
		break;

	case MID_LPMP3_STATE:
		/* user asks lpmp3 then only
		 * do lpmp3
		 */
		if (possible == MID_LPMP3_STATE)
			ret = MID_LPMP3_STATE;
		break;

	case MID_LPI1_STATE:
	case MID_LPI3_STATE:
		/* user asks lpmp3/i1/i3 then only
		 * do lpmp3/i1/i3
		 */
		if (possible == MID_LPMP3_STATE)
			ret = MID_LPMP3_STATE;
		else if (possible == MID_S0IX_STATE)
			ret = extended_cstate_mode >> REMOVE_LP_FROM_LPIX;
		break;

	case MID_S0IX_STATE:
		ret = possible;
		break;
	}

	if ((ret == MID_S0IX_STATE) &&
			(*eax == MID_LPMP3_STATE))
		ret = MID_S0I1_STATE;
	else if ((ret <= *eax ||
			(ret == MID_S0IX_STATE)))
		ret = ret & *eax;
	else
		ret = 0;

	return ret;
}

static bool check_s0ix_possible(struct pmu_ss_states *pmsss)
{
	if (((pmsss->pmu2_states[0] & S0IX_TARGET_SSS0_MASK) ==
					S0IX_TARGET_SSS0) &&
		((pmsss->pmu2_states[1] & S0IX_TARGET_SSS1_MASK) ==
					S0IX_TARGET_SSS1) &&
		((pmsss->pmu2_states[2] & S0IX_TARGET_SSS2_MASK) ==
					S0IX_TARGET_SSS2) &&
		((pmsss->pmu2_states[3] & S0IX_TARGET_SSS3_MASK) ==
					S0IX_TARGET_SSS3))
		return true;

	return false;
}

static bool check_lpmp3_possible(struct pmu_ss_states *pmsss)
{
	if (((pmsss->pmu2_states[0] & LPMP3_TARGET_SSS0_MASK) ==
					LPMP3_TARGET_SSS0) &&
		((pmsss->pmu2_states[1] & LPMP3_TARGET_SSS1_MASK) ==
					LPMP3_TARGET_SSS1) &&
		((pmsss->pmu2_states[2] & LPMP3_TARGET_SSS2_MASK) ==
					LPMP3_TARGET_SSS2) &&
		((pmsss->pmu2_states[3] & LPMP3_TARGET_SSS3_MASK) ==
					LPMP3_TARGET_SSS3))
		return true;

	return false;
}

void pmu_set_s0ix_possible(int state)
{
	/* assume S0ix not possible */
	mid_pmu_cxt->s0ix_possible = 0;

	if (state != PCI_D0) {
		struct pmu_ss_states cur_pmsss;

		pmu_read_sss(&cur_pmsss);

		if (likely(check_s0ix_possible(&cur_pmsss)))
			mid_pmu_cxt->s0ix_possible = MID_S0IX_STATE;
		else if (check_lpmp3_possible(&cur_pmsss))
			mid_pmu_cxt->s0ix_possible = MID_LPMP3_STATE;
	}
}

int get_target_platform_state(unsigned long *eax)
{
	int ret = 0;

	if (unlikely(!pmu_initialized))
		goto ret;

	/* dont do s0ix if suspend in progress */
	if (unlikely(mid_pmu_cxt->suspend_started))
		goto ret;

	/* dont do s0ix if shutdown in progress */
	if (unlikely(mid_pmu_cxt->shutdown_started))
		goto ret;

	if (nc_device_state())
		goto ret;

	ret = get_final_state(eax);

ret:
	*eax = C6_HINT;
	return ret;
}
EXPORT_SYMBOL(get_target_platform_state);

u32 get_s0ix_val_set_pm_ssc(int s0ix_state)
{
	u32 s0ix_value = 0;

	switch (s0ix_state) {
	case MID_S0I1_STATE:
		writel(S0I1_SSS0, &mid_pmu_cxt->pmu_reg->pm_ssc[0]);
		writel(S0I1_SSS1, &mid_pmu_cxt->pmu_reg->pm_ssc[1]);
		writel(S0I1_SSS2, &mid_pmu_cxt->pmu_reg->pm_ssc[2]);
		writel(S0I1_SSS3, &mid_pmu_cxt->pmu_reg->pm_ssc[3]);
		pmu_stat_start(SYS_STATE_S0I1);
		s0ix_value = S0I1_VALUE;
		break;
	case MID_LPMP3_STATE:
		writel(LPMP3_SSS0, &mid_pmu_cxt->pmu_reg->pm_ssc[0]);
		writel(LPMP3_SSS1, &mid_pmu_cxt->pmu_reg->pm_ssc[1]);
		writel(LPMP3_SSS2, &mid_pmu_cxt->pmu_reg->pm_ssc[2]);
		writel(LPMP3_SSS3, &mid_pmu_cxt->pmu_reg->pm_ssc[3]);
		pmu_stat_start(SYS_STATE_S0I2);
		s0ix_value = LPMP3_VALUE;
		break;
	case MID_S0I3_STATE:
		writel(S0I3_SSS0, &mid_pmu_cxt->pmu_reg->pm_ssc[0]);
		writel(S0I3_SSS1, &mid_pmu_cxt->pmu_reg->pm_ssc[1]);
		writel(S0I3_SSS2, &mid_pmu_cxt->pmu_reg->pm_ssc[2]);
		writel(S0I3_SSS3, &mid_pmu_cxt->pmu_reg->pm_ssc[3]);
		pmu_stat_start(SYS_STATE_S0I3);
		s0ix_value = S0I3_VALUE;
		break;
	case MID_S3_STATE:
		writel(S0I3_SSS0, &mid_pmu_cxt->pmu_reg->pm_ssc[0]);
		writel(S0I3_SSS1, &mid_pmu_cxt->pmu_reg->pm_ssc[1]);
		writel(S0I3_SSS2, &mid_pmu_cxt->pmu_reg->pm_ssc[2]);
		writel(S0I3_SSS3, &mid_pmu_cxt->pmu_reg->pm_ssc[3]);
		pmu_stat_start(SYS_STATE_S3);
		s0ix_value = S0I3_VALUE;
		break;
	case MID_FAST_ON_OFF_STATE:
		writel(S0I3_SSS0, &mid_pmu_cxt->pmu_reg->pm_ssc[0]);
		writel(S0I3_SSS1, &mid_pmu_cxt->pmu_reg->pm_ssc[1]);
		writel(S0I3_SSS2, &mid_pmu_cxt->pmu_reg->pm_ssc[2]);
		writel(S0I3_SSS3, &mid_pmu_cxt->pmu_reg->pm_ssc[3]);
		pmu_stat_start(SYS_STATE_S3);
		s0ix_value = FAST_ON_OFF_VALUE;
		break;
	default:
		pmu_dump_logs();
		BUG_ON(1);
	}
	return s0ix_value;
}

void platform_update_all_lss_states(struct pmu_ss_states *pmu_config,
					int *PCIALLDEV_CFG)
{
	/* We shutdown devices that are in the target config, and that are
	   not in the pci table, some devices are indeed not advertised in pci
	   table for certain firmwares. This is the case for HSI firmwares,
	   SPI3 device is not advertised, and would then prevent s0i3. */
	/* Also take IGNORE_CFG in account (for e.g. GPIO1)*/
	pmu_config->pmu2_states[0] |= S0IX_TARGET_SSS0_MASK & ~PCIALLDEV_CFG[0];
	pmu_config->pmu2_states[0] &= ~IGNORE_SSS0;
	pmu_config->pmu2_states[1] |= S0IX_TARGET_SSS1_MASK & ~PCIALLDEV_CFG[1];
	pmu_config->pmu2_states[1] &= ~IGNORE_SSS1;
	pmu_config->pmu2_states[2] |= S0IX_TARGET_SSS2_MASK & ~PCIALLDEV_CFG[2];
	pmu_config->pmu2_states[2] &= ~IGNORE_SSS2;
	pmu_config->pmu2_states[3] |= S0IX_TARGET_SSS3_MASK & ~PCIALLDEV_CFG[3];
	pmu_config->pmu2_states[3] &= ~IGNORE_SSS3;
}

void s0ix_complete(void)
{
	if (unlikely(mid_pmu_cxt->s0ix_entered))
		writel(0, &mid_pmu_cxt->pmu_reg->pm_msic);
}

/*
 * Valid wake source: lss_number 0 to 63
 * Returns true if 'lss_number' is wake source
 * else false
 */
bool mid_pmu_is_wake_source(u32 lss_number)
{
	u32 wake = 0;
	bool ret = false;

	if (lss_number > PMU_MAX_LSS)
		return ret;

	if (lss_number < PMU_LSS_IN_FIRST_DWORD) {
		wake = readl(&mid_pmu_cxt->pmu_reg->pm_wks[0]);
		wake &= (1 << lss_number);
	} else {
		wake = readl(&mid_pmu_cxt->pmu_reg->pm_wks[1]);
		wake &= (1 << (lss_number - PMU_LSS_IN_FIRST_DWORD));
	}

	if (wake)
		ret = true;

	return ret;
}

static void log_wakeup_source(int source)
{
	enum sys_state type = mid_pmu_cxt->pmu_current_state;

	mid_pmu_cxt->num_wakes[source][type]++;

	trace_printk("wake_from_lss%d\n",
		     source - mid_pmu_cxt->pmu1_max_devs);

	if ((mid_pmu_cxt->pmu_current_state != SYS_STATE_S3)
	    || !mid_pmu_cxt->suspend_started)
		return;

	switch (source - mid_pmu_cxt->pmu1_max_devs) {
	case PMU_USB_OTG_LSS_06:
		pr_info("wakeup from USB.\n");
		break;
	case PMU_GPIO0_LSS_39:
		pr_info("wakeup from GPIO.\n");
		break;
	case PMU_HSI_LSS_03:
		pr_info("wakeup from HSI.\n");
		break;
	default:
		pr_info("wakeup from LSS%02d.\n",
			source - mid_pmu_cxt->pmu1_max_devs);
		break;
	}
}

/* return the last wake source id, and make statistics about wake sources */
int pmu_get_wake_source(void)
{
	u32 wake0, wake1;
	int i;
	int source = INVALID_WAKE_SRC;

	wake0 = readl(&mid_pmu_cxt->pmu_reg->pm_wks[0]);
	wake1 = readl(&mid_pmu_cxt->pmu_reg->pm_wks[1]);

	if (!wake0 && !wake1) {
		log_wakeup_irq();
		goto out;
	}

	while (wake0) {
		i = fls(wake0) - 1;
		source = i + mid_pmu_cxt->pmu1_max_devs;
		log_wakeup_source(source);
		wake0 &= ~(1<<i);
	}

	while (wake1) {
		i = fls(wake1) - 1;
		source = i + 32 + mid_pmu_cxt->pmu1_max_devs;
		log_wakeup_source(source);
		wake1 &= ~(1<<i);
	}
out:
	return source;
}

static int wait_for_nc_pmcmd_complete(int verify_mask, int state_type
					, int reg_type)
{
	int pwr_sts;
	int count = 0;
	u32 addr;

	switch (reg_type) {
	case APM_REG_TYPE:
		addr = mid_pmu_cxt->apm_base + APM_STS;
		break;
	case OSPM_REG_TYPE:
		addr = mid_pmu_cxt->ospm_base + OSPM_PM_SSS;
		break;
	default:
		return -EINVAL;
	}

	while (true) {
		pwr_sts = inl(addr);
		if (state_type == OSPM_ISLAND_DOWN) {
			if ((pwr_sts & verify_mask) == verify_mask)
				break;
			else
				udelay(10);
		} else if (state_type == OSPM_ISLAND_UP) {
			if (pwr_sts  == verify_mask)
				break;
			else
				udelay(10);
		}
		count++;
		if (WARN_ONCE(count > 500000, "Timed out waiting for P-Unit"))
			return -EBUSY;
	}
	return 0;
}

int mdfld_clv_nc_set_power_state(int islands, int state_type,
					int reg_type, int *change)
{
	u32 pwr_cnt = 0;
	u32 pwr_mask = 0;
	int i, lss, mask;
	int ret = 0;

	*change = 0;

	switch (reg_type) {
	case APM_REG_TYPE:
		pwr_cnt = inl(mid_pmu_cxt->apm_base + APM_STS);
		break;
	case OSPM_REG_TYPE:
		pwr_cnt = inl(mid_pmu_cxt->ospm_base + OSPM_PM_SSS);
		break;
	default:
		ret = -EINVAL;
		goto unlock;
	}

	pwr_mask = pwr_cnt;
	for (i = 0; i < OSPM_MAX_POWER_ISLANDS; i++) {
		lss = islands & (0x1 << i);
		if (lss) {
			mask = D0I3_MASK << (BITS_PER_LSS * i);
			if (state_type == OSPM_ISLAND_DOWN)
				pwr_mask |= mask;
			else if (state_type == OSPM_ISLAND_UP)
				pwr_mask &= ~mask;
		}
	}

	if (pwr_mask != pwr_cnt) {
		switch (reg_type) {
		case APM_REG_TYPE:
			outl(pwr_mask, mid_pmu_cxt->apm_base + APM_CMD);
			break;
		case OSPM_REG_TYPE:
			outl(pwr_mask, mid_pmu_cxt->ospm_base + OSPM_PM_SSC);
			break;
		}

		ret =
		wait_for_nc_pmcmd_complete(pwr_mask, state_type, reg_type);
		if (!ret)
			*change = 1;
		if (nc_report_power_state)
			nc_report_power_state(pwr_mask, reg_type);
	}

unlock:
	return ret;
}
