#include <linux/module.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/usb/otg.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/freezer.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/version.h>

#include <linux/usb.h>
#include <linux/usb/hcd.h>
#include <linux/usb/gadget.h>
#include <linux/usb/dwc3-intel-mid.h>
#include <asm/intel_scu_pmic.h>
#include "otg.h"

#define VERSION "2.10a"

static int otg_id = -1;
static int enable_usb_phy(struct dwc_otg2 *otg, bool on_off);
static int dwc3_intel_notify_charger_type(struct dwc_otg2 *otg,
		enum usb_charger_state state);

static int is_hybridvp(struct dwc_otg2 *otg)
{
	struct intel_dwc_otg_pdata *data;
	if (!otg || !otg->otg_data)
		return -EINVAL;

	data = (struct intel_dwc_otg_pdata *)otg->otg_data;

	return data->is_hvp;
}

static enum usb_charger_type aca_check(struct dwc_otg2 *otg)
{
	u8 rarbrc;
	enum usb_charger_type type = CHRG_UNKNOWN;
	int ret;

	/* Wait >66.1ms (for TCHGD_SERX_DEB) */
	msleep(66);

	/* Read decoded RID value */
	ret = intel_scu_ipc_ioread8(PMIC_USBIDSTS, &rarbrc);
	if (ret)
		otg_err(otg, "Fail to read decoded RID value\n");
	rarbrc &= USBIDSTS_ID_RARBRC_STS(3);

	/* If ID_RARBRC_STS==01: ACA-Dock detected
	 * If ID_RARBRC_STS==00: MHL detected
	 */
	if (rarbrc == 1) {
		/* ACA-Dock */
		type = CHRG_ACA_DOCK;
	} else if (!rarbrc) {
		/* MHL */
		type = CHRG_MHL;
	}

	return type;
}

static ssize_t store_otg_id(struct device *_dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long flags;
	struct dwc_otg2 *otg = dwc3_get_otg();

	if (!otg)
		return 0;
	if (count != 2) {
		otg_err(otg, "return EINVAL\n");
		return -EINVAL;
	}

	if (count > 0 && buf[count-1] == '\n')
		((char *) buf)[count-1] = 0;

	switch (buf[0]) {
	case 'a':
	case 'A':
		otg_dbg(otg, "Change ID to A\n");
		otg->user_events |= USER_ID_A_CHANGE_EVENT;
		spin_lock_irqsave(&otg->lock, flags);
		dwc3_wakeup_otg_thread(otg);
		otg_id = 0;
		spin_unlock_irqrestore(&otg->lock, flags);
		return count;
	case 'b':
	case 'B':
		otg_dbg(otg, "Change ID to B\n");
		otg->user_events |= USER_ID_B_CHANGE_EVENT;
		spin_lock_irqsave(&otg->lock, flags);
		dwc3_wakeup_otg_thread(otg);
		otg_id = 1;
		spin_unlock_irqrestore(&otg->lock, flags);
		return count;
	default:
		otg_err(otg, "Just support change ID to A!\n");
		return -EINVAL;
	}

	return count;
}

static ssize_t
show_otg_id(struct device *_dev, struct device_attribute *attr, char *buf)
{
	char				*next;
	unsigned			size, t;

	next = buf;
	size = PAGE_SIZE;

	t = scnprintf(next, size,
		"USB OTG ID: %s\n",
		(otg_id ? "B" : "A")
		);
	size -= t;
	next += t;

	return PAGE_SIZE - size;
}

static DEVICE_ATTR(otg_id, S_IRUGO|S_IWUSR|S_IWGRP,
			show_otg_id, store_otg_id);

static void set_sus_phy(struct dwc_otg2 *otg, int bit)
{
	u32 data = 0;

	data = otg_read(otg, GUSB2PHYCFG0);
	if (bit)
		data |= GUSB2PHYCFG_SUS_PHY;
	else
		data &= ~GUSB2PHYCFG_SUS_PHY;

	otg_write(otg, GUSB2PHYCFG0, data);

	data = otg_read(otg, GUSB3PIPECTL0);
	if (bit)
		data |= GUSB3PIPECTL_SUS_EN;
	else
		data &= ~GUSB3PIPECTL_SUS_EN;
	otg_write(otg, GUSB3PIPECTL0, data);
}

int dwc3_intel_platform_init(struct dwc_otg2 *otg)
{
	u32 gctl;
	int retval;

	otg_info(otg, "De-assert USBRST# to enable PHY\n");
	retval = intel_scu_ipc_iowrite8(PMIC_USBPHYCTRL,
			PMIC_USBPHYCTRL_D0);
	if (retval)
		otg_err(otg, "Fail to de-assert USBRST#\n");

	/* Don't let phy go to suspend mode, which
	 * will cause FS/LS devices enum failed in host mode.
	 */
	set_sus_phy(otg, 0);

	retval = device_create_file(otg->dev, &dev_attr_otg_id);
	if (retval < 0) {
		otg_dbg(otg,
			"Can't register sysfs attribute: %d\n", retval);
		return -ENOMEM;
	}

	otg_dbg(otg, "\n");
	otg_write(otg, OEVTEN, 0);
	otg_write(otg, OCTL, 0);
	gctl = otg_read(otg, GCTL);
	gctl |= GCTL_PRT_CAP_DIR_OTG << GCTL_PRT_CAP_DIR_SHIFT;
	otg_write(otg, GCTL, gctl);

	return 0;
}

/* This function will control VUSBPHY to power gate/ungate USBPHY */
static int enable_usb_phy(struct dwc_otg2 *otg, bool on_off)
{
	int ret;

	if (on_off) {
		ret = intel_scu_ipc_update_register(PMIC_VLDOCNT,
				0xff, PMIC_VLDOCNT_VUSBPHYEN);
		if (ret)
			otg_err(otg, "Fail to enable VBUSPHY\n");

		msleep(20);
	} else {
		ret = intel_scu_ipc_update_register(PMIC_VLDOCNT,
				0x00, PMIC_VLDOCNT_VUSBPHYEN);
		if (ret)
			otg_err(otg, "Fail to disable VBUSPHY\n");
	}

	return 0;
}

int dwc3_intel_get_id(struct dwc_otg2 *otg)
{
	int ret, id = RID_UNKNOWN;
	u8 idsts;

	ret = intel_scu_ipc_update_register(PMIC_USBIDCTRL,
			USBIDCTRL_ACA_DETEN_D1 | PMIC_USBPHYCTRL_D0,
			USBIDCTRL_ACA_DETEN_D1 | PMIC_USBPHYCTRL_D0);
	if (ret)
		otg_err(otg, "Fail to enable ACA&ID detection logic\n");

	mdelay(50);

	ret = intel_scu_ipc_ioread8(PMIC_USBIDSTS, &idsts);
	if (ret) {
		otg_err(otg, "Fail to read id\n");
		return id;
	}

	if (idsts & USBIDSTS_ID_FLOAT_STS)
		id = RID_FLOAT;
	else if (idsts & USBIDSTS_ID_GND)
		id = RID_GND;
	else if (idsts & USBIDSTS_ID_RARBRC_STS(1))
		id = RID_A;
	else if (idsts & USBIDSTS_ID_RARBRC_STS(2))
		id = RID_B;
	else if (idsts & USBIDSTS_ID_RARBRC_STS(3))
		id = RID_C;

	return id;
}

int dwc3_intel_b_idle(struct dwc_otg2 *otg)
{
	u32 gctl, tmp;

	if (!is_hybridvp(otg))
		enable_usb_phy(otg, false);

	/* Disable hibernation mode by default */
	gctl = otg_read(otg, GCTL);
	gctl &= ~GCTL_GBL_HIBERNATION_EN;
	otg_write(otg, GCTL, gctl);

	/* Reset ADP related registers */
	otg_write(otg, ADPCFG, 0);
	otg_write(otg, ADPCTL, 0);
	otg_write(otg, ADPEVTEN, 0);
	tmp = otg_read(otg, ADPEVT);
	otg_write(otg, ADPEVT, tmp);

	otg_write(otg, OCFG, 0);
	otg_write(otg, OEVTEN, 0);
	tmp = otg_read(otg, OEVT);
	otg_write(otg, OEVT, tmp);
	otg_write(otg, OCTL, OCTL_PERI_MODE);

	/* Force config to device mode as default */
	gctl = otg_read(otg, GCTL);
	gctl &= ~GCTL_PRT_CAP_DIR;
	gctl |= GCTL_PRT_CAP_DIR_DEV << GCTL_PRT_CAP_DIR_SHIFT;
	otg_write(otg, GCTL, gctl);

	enable_usb_phy(otg, true);
	mdelay(100);

	return 0;
}

static int dwc3_intel_set_power(struct usb_phy *_otg,
		unsigned ma)
{
	unsigned long flags;
	struct dwc_otg2 *otg = dwc3_get_otg();

	if (!otg)
		return 0;

	/* force 896ma to 900ma
	 * Beucause the power just can be set as an
	 * integer multiple of 8 in usb configuration descriptor
	 */
	if (ma == 896)
		ma = 900;

	if ((ma != 100) &&
		(ma != 150) &&
		(ma != 500) &&
		(ma != 900)) {
		otg_err(otg, "Device driver set invalid SDP current value!\n");
		return -EINVAL;
	}

	if (otg->charging_cap.chrg_type == CHRG_CDP)
		return 0;
	else if (otg->charging_cap.chrg_type != CHRG_SDP) {
		otg_err(otg, "%s: currently, chrg type is not SDP!\n",
				__func__);
		return -EINVAL;
	}

	spin_lock_irqsave(&otg->lock, flags);
	otg->charging_cap.ma = ma;
	spin_unlock_irqrestore(&otg->lock, flags);

	dwc3_intel_notify_charger_type(otg,
			OTG_CHR_STATE_CONNECTED);

	return 0;
}

int dwc3_intel_enable_vbus(struct dwc_otg2 *otg, int enable)
{
	atomic_notifier_call_chain(&otg->usb2_phy.notifier,
			USB_EVENT_DRIVE_VBUS, &cap);

	return 0;
}

static int dwc3_intel_notify_charger_type(struct dwc_otg2 *otg,
		enum usb_charger_state state)
{
	struct otg_bc_cap cap;
	int ret = 0;
	unsigned long flags;

	if (state > OTG_CHR_STATE_HOST) {
		otg_err(otg, "%s: Invalid usb_charger_state!\n", __func__);
		return -EINVAL;
	}

	if ((otg->charging_cap.chrg_type ==  CHRG_SDP) &&
			((otg->charging_cap.ma != 100) &&
			 (otg->charging_cap.ma != 150) &&
			 (otg->charging_cap.ma != 500) &&
			 (otg->charging_cap.ma != 900))) {
		otg_err(otg, "%s: invalid SDP current!\n", __func__);
		return -EINVAL;
	}

	spin_lock_irqsave(&otg->lock, flags);
	cap.chrg_type = otg->charging_cap.chrg_type;
	cap.ma = otg->charging_cap.ma;
	cap.chrg_state = state;
	spin_unlock_irqrestore(&otg->lock, flags);

	atomic_notifier_call_chain(&otg->usb2_phy.notifier, USB_EVENT_CHARGER,
			&cap);

	return ret;
}

enum usb_charger_type dwc3_intel_get_charger_type(struct dwc_otg2 *otg)
{
	u8 val, vdat_det, chgd_serx_dm;
	unsigned long timeout, interval;
	int ret;
	enum usb_charger_type type = CHRG_UNKNOWN;
	struct usb_phy *phy;

	phy = usb_get_phy(USB_PHY_TYPE_USB2);
	if (!phy) {
		otg_err(otg, "Get USB2 PHY failed\n");
		return CHRG_UNKNOWN;
	}

	/* PHY Enable:
	 * Power on PHY
	 */
	enable_usb_phy(otg, true);

	/* Wait 10ms (~5ms before PHY de-asserts DIR,
	 * XXus for initial Link reg sync-up).*/
	msleep(20);

	/* Enable ACA:
	 * Enable ACA & ID detection logic.
	 */
	ret = intel_scu_ipc_update_register(PMIC_USBIDCTRL,
			USBIDCTRL_ACA_DETEN_D1 | PMIC_USBPHYCTRL_D0,
			USBIDCTRL_ACA_DETEN_D1 | PMIC_USBPHYCTRL_D0);
	if (ret)
		otg_err(otg, "Fail to enable ACA&ID detection logic\n");

	/* DCD Enable: Change OPMODE to 01 (Non-driving),
	 * TermSel to 0, &
	 * XcvrSel to 01 (enable FS xcvr)
	 */
	usb_phy_io_write(phy, FUNCCTRL_OPMODE(1) | FUNCCTRL_XCVRSELECT(1),
					TUSB1211_FUNC_CTRL_SET);

	usb_phy_io_write(phy, FUNCCTRL_OPMODE(2) | FUNCCTRL_XCVRSELECT(2)
					| FUNCCTRL_TERMSELECT,
					TUSB1211_FUNC_CTRL_CLR);

	/*Enable SW control*/
	usb_phy_io_write(phy, PWCTRL_SW_CONTROL, TUSB1211_POWER_CONTROL_SET);

	/* Enable IDPSRC */
	usb_phy_io_write(phy, VS3_CHGD_IDP_SRC_EN,
			TUSB1211_VENDOR_SPECIFIC3_SET);

	/* Check DCD result, use same polling parameter */
	timeout = jiffies + msecs_to_jiffies(DATACON_TIMEOUT);
	interval = DATACON_INTERVAL * 1000; /* us */

	/* DCD Check:
	 * Delay 66.5 ms. (Note:
	 * TIDP_SRC_ON + TCHGD_SERX_DEB =
	 * 347.8us + 66.1ms).
	 */
	usleep_range(66500, 67000);

	while (!time_after(jiffies, timeout)) {
		/* Read DP logic level. */
		val = usb_phy_io_read(phy, TUSB1211_VENDOR_SPECIFIC4);
		if (val < 0) {
			otg_err(otg, "ULPI read error! try again\n");
			continue;
		}

		if (!(val & VS4_CHGD_SERX_DP)) {
			otg_info(otg, "Data contact detected!\n");
			break;
		}

		/* Polling interval */
		usleep_range(interval, interval + 2000);
	}

	/* Disable DP pullup (Idp_src) */
	usb_phy_io_write(phy, VS3_CHGD_IDP_SRC_EN,
			TUSB1211_VENDOR_SPECIFIC3_CLR);

	/* ID Check:
	 * Check ID pin state.
	 */
	val = dwc3_intel_get_id(otg);
	if (val)
		otg_err(otg, "Fail to enable ACA&ID detection logic\n");

	val &= USBIDSTS_ID_FLOAT_STS;
	if (!val) {
		type = aca_check(otg);
		goto cleanup;
	}

	/* SE1 Det Enable:
	 * Read DP/DM logic level. Note: use DEBUG
	 * because VS4 isn’t enabled in this situation.
     */
	val = usb_phy_io_read(phy, TUSB1211_DEBUG);
	if (val < 0)
		otg_err(otg, "ULPI read error!\n");

	val &= DEBUG_LINESTATE;

	/* If '11': SE1 detected; goto 'Cleanup'.
	 * Else: goto 'Pri Det Enable'.
	 */
	if (val == 3) {
		type = CHRG_SE1;
		goto cleanup;
	}

	/* Pri Det Enable:
	 * Enable VDPSRC.
	 */
	usb_phy_io_write(phy, PWCTRL_DP_VSRC_EN, TUSB1211_POWER_CONTROL_SET);

	/* Wait >106.1ms (40ms for BC
	 * Tvdpsrc_on, 66.1ms for TI CHGD_SERX_DEB).
	 */
	msleep(107);

	/* Pri Det Check:
	 * Check if DM > VDATREF.
	 */
	vdat_det = usb_phy_io_read(phy, TUSB1211_POWER_CONTROL);
	if (vdat_det < 0)
		otg_err(otg, "ULPI read error!\n");

	vdat_det &= PWCTRL_VDAT_DET;

	/* Check if DM<VLGC */
	chgd_serx_dm = usb_phy_io_read(phy, TUSB1211_VENDOR_SPECIFIC4);
	if (chgd_serx_dm < 0)
		otg_err(otg, "ULPI read error!\n");

	chgd_serx_dm &= VS4_CHGD_SERX_DM;

	/* If VDAT_DET==0 || CHGD_SERX_DM==1: SDP detected
	 * If VDAT_DET==1 && CHGD_SERX_DM==0: CDP/DCP
	 */
	if (vdat_det == 0 || chgd_serx_dm == 1)
		type = CHRG_SDP;

	/* Disable VDPSRC. */
	usb_phy_io_write(phy, PWCTRL_DP_VSRC_EN, TUSB1211_POWER_CONTROL_CLR);

	/* If SDP, goto “Cleanup”.
	 * Else, goto “Sec Det Enable”
	 */
	if (type == CHRG_SDP)
		goto cleanup;

	/* Sec Det Enable:
	 * delay 1ms.
	 */
	usleep_range(1000, 1500);

	/* Swap DP & DM */
	usb_phy_io_write(phy, VS1_DATAPOLARITY, TUSB1211_VENDOR_SPECIFIC1_CLR);

	/* Enable 'VDMSRC'. */
	usb_phy_io_write(phy, PWCTRL_DP_VSRC_EN, TUSB1211_POWER_CONTROL_SET);

	/* Wait >73ms (40ms for BC Tvdmsrc_on, 33ms for TI TVDPSRC_DEB) */
	msleep(80);

	/* Sec Det Check:
	 * Check if DP>VDATREF.
	 */
	val = usb_phy_io_read(phy, TUSB1211_POWER_CONTROL);
	if (val < 0)
		otg_err(otg, "ULPI read error!\n");

	val &= PWCTRL_VDAT_DET;

	/* If VDAT_DET==0: CDP detected.
	 * If VDAT_DET==1: DCP detected.
	 */
	if (!val)
		type = CHRG_CDP;
	else
		type = CHRG_DCP;

	/* Disable VDMSRC. */
	usb_phy_io_write(phy, PWCTRL_DP_VSRC_EN, TUSB1211_POWER_CONTROL_CLR);

	/* Swap DP & DM. */
	usb_phy_io_write(phy, VS1_DATAPOLARITY, TUSB1211_VENDOR_SPECIFIC1_SET);

cleanup:

	/* If DCP detected, assert VDPSRC. */
	if (type == CHRG_DCP)
		usb_phy_io_write(phy, PWCTRL_SW_CONTROL | PWCTRL_DP_VSRC_EN,
				TUSB1211_POWER_CONTROL_SET);

	usb_put_phy(phy);

	return type;

}

static int dwc3_intel_handle_notification(struct notifier_block *nb,
		unsigned long event, void *data)
{
	struct dwc_otg2 *otg = dwc3_get_otg();
	int state, val;
	unsigned long flags;

	if (!otg)
		return NOTIFY_BAD;

	val = *(int *)data;

	spin_lock_irqsave(&otg->lock, flags);
	switch (event) {
	case USB_EVENT_ID:
		otg->otg_events |= OEVT_CONN_ID_STS_CHNG_EVNT;
		state = NOTIFY_OK;
		break;
	case USB_EVENT_VBUS:
		if (val)
			otg->otg_events |= OEVT_B_DEV_SES_VLD_DET_EVNT;
		else
			otg->otg_events |= OEVT_A_DEV_SESS_END_DET_EVNT;
		state = NOTIFY_OK;
		break;
	default:
		otg_dbg(otg, "DWC OTG Notify unknow notify message\n");
		state = NOTIFY_DONE;
	}
	dwc3_wakeup_otg_thread(otg);
	spin_unlock_irqrestore(&otg->lock, flags);

	return state;

}

int dwc3_intel_prepare_start_host(struct dwc_otg2 *otg)
{
	if (!is_hybridvp(otg))
		enable_usb_phy(otg, true);
	return 0;
}

int dwc3_intel_prepare_start_peripheral(struct dwc_otg2 *otg)
{
	if (!is_hybridvp(otg))
		enable_usb_phy(otg, true);

	return 0;
}

struct dwc3_otg_hw_ops dwc3_intel_otg_pdata = {
	.mode = DWC3_DRD,
	.bus = DWC3_PCI,
	.get_id = dwc3_intel_get_id,
	.b_idle = dwc3_intel_b_idle,
	.set_power = dwc3_intel_set_power,
	.enable_vbus = dwc3_intel_enable_vbus,
	.platform_init = dwc3_intel_platform_init,
	.get_charger_type = dwc3_intel_get_charger_type,
	.otg_notifier_handler = dwc3_intel_handle_notification,
	.prepare_start_peripheral = dwc3_intel_prepare_start_peripheral,
	.prepare_start_host = dwc3_intel_prepare_start_host,
};

static int __init dwc3_intel_init(void)
{
	return dwc3_otg_register(&dwc3_intel_otg_pdata);
}
module_init(dwc3_intel_init);

static void __exit dwc3_intel_exit(void)
{
	dwc3_otg_unregister(&dwc3_intel_otg_pdata);
}
module_exit(dwc3_intel_exit);

MODULE_AUTHOR("Wang Yu <yu.y.wang@intel.com>");
MODULE_DESCRIPTION("DWC3 Intel OTG Driver");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_VERSION(VERSION);
