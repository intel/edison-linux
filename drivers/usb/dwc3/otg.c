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

#include "otg.h"

#define VERSION "2.10a"

struct dwc3_otg_hw_ops *dwc3_otg_pdata;
struct dwc_device_par *platform_par;

static struct mutex lock;
static const char driver_name[] = "dwc3_otg";
static struct dwc_otg2 *the_transceiver;
static void dwc_otg_remove(struct pci_dev *pdev);


static inline struct dwc_otg2 *xceiv_to_dwc_otg2(struct usb_otg *x)
{
	return container_of(x, struct dwc_otg2, otg);
}

struct dwc_otg2 *dwc3_get_otg(void)
{
	return the_transceiver;
}
EXPORT_SYMBOL_GPL(dwc3_get_otg);

/* Caller must hold otg->lock */
void dwc3_wakeup_otg_thread(struct dwc_otg2 *otg)
{
	if (!otg->main_thread)
		return;

	otg_dbg(otg, "\n");
	/* Tell the main thread that something has happened */
	otg->main_wakeup_needed = 1;
	wake_up_interruptible(&otg->main_wq);
}
EXPORT_SYMBOL_GPL(dwc3_wakeup_otg_thread);

static int sleep_main_thread_timeout(struct dwc_otg2 *otg, int msecs)
{
	signed long jiffies;
	int rc = msecs;

	if (otg->state == DWC_STATE_EXIT) {
		otg_dbg(otg, "Main thread exiting\n");
		rc = -EINTR;
		goto done;
	}

	if (signal_pending(current)) {
		otg_dbg(otg, "Main thread signal pending\n");
		rc = -EINTR;
		goto done;
	}
	if (otg->main_wakeup_needed) {
		otg_dbg(otg, "Main thread wakeup needed\n");
		rc = msecs;
		goto done;
	}

	jiffies = msecs_to_jiffies(msecs);
	rc = wait_event_freezable_timeout(otg->main_wq,
					otg->main_wakeup_needed,
					jiffies);

	if (otg->state == DWC_STATE_EXIT) {
		otg_dbg(otg, "Main thread exiting\n");
		rc = -EINTR;
		goto done;
	}

	if (rc > 0)
		rc = jiffies_to_msecs(rc);

done:
	otg->main_wakeup_needed = 0;
	return rc;
}

static int sleep_main_thread(struct dwc_otg2 *otg)
{
	int rc = 0;

	do {
		rc = sleep_main_thread_timeout(otg, 5000);
	} while (rc == 0);

	return rc;
}

static void get_events(struct dwc_otg2 *otg,
		u32 *otg_events,
		u32 *user_events)
{
	unsigned long flags;

	spin_lock_irqsave(&otg->lock, flags);

	if (otg_events)
		*otg_events = otg->otg_events;

	if (user_events)
		*user_events = otg->user_events;
	spin_unlock_irqrestore(&otg->lock, flags);
}

static void get_and_clear_events(struct dwc_otg2 *otg,
				u32 *otg_events,
				u32 *user_events)
{
	unsigned long flags;

	spin_lock_irqsave(&otg->lock, flags);

	if (otg_events)
		*otg_events = otg->otg_events;

	if (user_events)
		*user_events = otg->user_events;

	otg->otg_events = 0;
	otg->user_events = 0;

	spin_unlock_irqrestore(&otg->lock, flags);
}

static int check_event(struct dwc_otg2 *otg,
		u32 otg_mask,
		u32 user_mask)
{
	u32 otg_events = 0;
	u32 user_events = 0;

	get_events(otg, &otg_events, &user_events);
	if ((otg_events & otg_mask) ||
			(user_events & user_mask)) {
		otg_dbg(otg, "Event occurred:");
		otg_dbg(otg, "otg_events=%x, otg_mask=%x,",
				otg_events, otg_mask);
		otg_dbg(otg, "user_events=%x, user_mask=%x",
				user_events, user_mask);
		return 1;
	}

	return 0;
}

static int sleep_until_event(struct dwc_otg2 *otg,
			u32 otg_mask, u32 user_mask,
			u32 *otg_events, u32 *user_events,
			int timeout)
{
	int rc = 0;

	/* Wait until it occurs, or timeout, or interrupt. */
	if (timeout) {
		otg_dbg(otg, "Waiting for event (timeout=%d)...\n", timeout);
		rc = sleep_main_thread_until_condition_timeout(otg,
				check_event(otg, otg_mask,
				user_mask), timeout);
	} else {
		otg_dbg(otg, "Waiting for event (no timeout)...\n");
		rc = sleep_main_thread_until_condition(otg,
				check_event(otg, otg_mask,
					user_mask));
	}

	/* Disable the events */
	otg_write(otg, OEVTEN, 0);
	otg_write(otg, ADPEVTEN, 0);

	otg_dbg(otg, "Woke up rc=%d\n", rc);
	if (rc < 0)
		goto done;
	else
		get_and_clear_events(otg, otg_events, user_events);

done:
	return rc;
}


static int start_host(struct dwc_otg2 *otg)
{
	int ret = 0;
	struct usb_hcd *hcd = NULL;

	otg_dbg(otg, "\n");

	if (!otg->otg.host) {
		otg_err(otg, "Haven't set host yet!\n");
		return -ENODEV;
	}

	if (dwc3_otg_pdata->prepare_start_host)
		ret = dwc3_otg_pdata->prepare_start_host(otg);

	/* Start host driver */
	hcd = container_of(otg->otg.host, struct usb_hcd, self);
	ret = otg->start_host(hcd);

	return ret;
}

static int stop_host(struct dwc_otg2 *otg)
{
	int ret = -1;
	struct usb_hcd *hcd = NULL;

	otg_dbg(otg, "\n");

	hcd = container_of(otg->otg.host, struct usb_hcd, self);
	if (otg->otg.host)
		ret = otg->stop_host(hcd);

	if (dwc3_otg_pdata->after_stop_host)
		ret = dwc3_otg_pdata->after_stop_host(otg);

	return ret;
}

static void start_peripheral(struct dwc_otg2 *otg)
{
	struct usb_gadget *gadget;
	int ret;

	if (dwc3_otg_pdata->prepare_start_peripheral)
		ret = dwc3_otg_pdata->prepare_start_peripheral(otg);

	gadget = otg->otg.gadget;
	if (!gadget) {
		otg_err(otg, "Haven't set gadget yet!\n");
		return;
	}

	otg->start_device(gadget);
}

static void stop_peripheral(struct dwc_otg2 *otg)
{
	struct usb_gadget *gadget = otg->otg.gadget;
	int ret;

	if (!gadget)
		return;

	otg->stop_device(gadget);

	if (dwc3_otg_pdata->after_stop_peripheral)
		ret = dwc3_otg_pdata->after_stop_peripheral(otg);
}

static int get_id(struct dwc_otg2 *otg)
{
	if (dwc3_otg_pdata->get_id)
		return dwc3_otg_pdata->get_id(otg);
	return RID_UNKNOWN;
}

static int dwc_otg_notify_charger_type(struct dwc_otg2 *otg,
		enum usb_charger_state state)
{
	if (dwc3_otg_pdata->notify_charger_type)
		return dwc3_otg_pdata->notify_charger_type(otg, state);

	return 0;
}

static int dwc_otg_get_chrg_status(struct usb_phy *x, void *data)
{
	unsigned long flags;
	struct otg_bc_cap *cap = (struct otg_bc_cap *)data;
	struct dwc_otg2 *otg = the_transceiver;

	if (!x)
		return -ENODEV;

	if (!data)
		return -EINVAL;

	spin_lock_irqsave(&otg->lock, flags);
	cap->chrg_type = otg->charging_cap.chrg_type;
	cap->chrg_state = otg->charging_cap.chrg_state;
	cap->ma = otg->charging_cap.ma;
	spin_unlock_irqrestore(&otg->lock, flags);

	return 0;
}

static int dwc_otg_enable_vbus(struct dwc_otg2 *otg, int enable)
{
	if (dwc3_otg_pdata->enable_vbus)
		return dwc3_otg_pdata->enable_vbus(otg, enable);

	return -EINVAL;
}

static int is_self_powered_b_device(struct dwc_otg2 *otg)
{
	return get_id(otg) == RID_GND;
}

static enum dwc_otg_state do_wait_vbus_raise(struct dwc_otg2 *otg)
{
	int ret;
	unsigned long flags;
	u32 otg_events = 0;
	u32 user_events = 0;
	u32 otg_mask = 0;
	u32 user_mask = 0;

	otg_mask = OEVT_B_DEV_SES_VLD_DET_EVNT |
				OEVT_CONN_ID_STS_CHNG_EVNT;

	ret = sleep_until_event(otg, otg_mask,
			user_mask, &otg_events,
			&user_events, VBUS_TIMEOUT);
	if (ret < 0)
		return DWC_STATE_EXIT;

	if (otg_events & OEVT_B_DEV_SES_VLD_DET_EVNT) {
		otg_dbg(otg, "OEVT_B_SES_VLD_EVT\n");
		return DWC_STATE_CHARGER_DETECTION;
	}

	if (otg_events & OEVT_CONN_ID_STS_CHNG_EVNT) {
		otg_dbg(otg, "OEVT_CONN_ID_STS_CHNG_EVNT\n");
		return DWC_STATE_B_IDLE;
	}

	/* timeout*/
	if (!ret) {
		if (is_self_powered_b_device(otg)) {
			spin_lock_irqsave(&otg->lock, flags);
			otg->charging_cap.chrg_type = B_DEVICE;
			spin_unlock_irqrestore(&otg->lock, flags);

			return DWC_STATE_A_HOST;
		}
	}

	return DWC_STATE_INVALID;
}

static enum dwc_otg_state do_wait_vbus_fall(struct dwc_otg2 *otg)
{
	int ret;

	u32 otg_events = 0;
	u32 user_events = 0;
	u32 otg_mask = 0;
	u32 user_mask = 0;

	otg_mask = OEVT_A_DEV_SESS_END_DET_EVNT;

	ret = sleep_until_event(otg, otg_mask,
			user_mask, &otg_events,
			&user_events, VBUS_TIMEOUT);
	if (ret < 0)
		return DWC_STATE_EXIT;

	if (otg_events & OEVT_A_DEV_SESS_END_DET_EVNT) {
		otg_dbg(otg, "OEVT_A_DEV_SESS_END_DET_EVNT\n");
		if (otg->charging_cap.chrg_type == CHRG_ACA_DOCK)
			dwc_otg_notify_charger_type(otg,
				OTG_CHR_STATE_DISCONNECTED);
		return DWC_STATE_B_IDLE;
	}

	/* timeout*/
	if (!ret) {
		otg_err(otg, "Haven't get VBus drop event! Maybe something wrong\n");
		return DWC_STATE_B_IDLE;
	}

	return DWC_STATE_INVALID;
}

static enum dwc_otg_state do_charging(struct dwc_otg2 *otg)
{
	int ret;
	u32 otg_events = 0;
	u32 user_events = 0;
	u32 otg_mask = 0;
	u32 user_mask = 0;

	otg_mask = OEVT_A_DEV_SESS_END_DET_EVNT;

	if (dwc3_otg_pdata->do_charging)
		dwc3_otg_pdata->do_charging(otg);

	ret = sleep_until_event(otg, otg_mask,
			user_mask, &otg_events,
			&user_events, 0);
	if (ret < 0)
		return DWC_STATE_EXIT;

	if (otg_events & OEVT_A_DEV_SESS_END_DET_EVNT) {
		otg_dbg(otg, "OEVT_A_DEV_SESS_END_DET_EVNT\n");
		dwc_otg_notify_charger_type(otg,
				OTG_CHR_STATE_DISCONNECTED);
		return DWC_STATE_B_IDLE;
	}

	return DWC_STATE_INVALID;
}

static enum usb_charger_type get_charger_type(struct dwc_otg2 *otg)
{
	if (dwc3_otg_pdata->get_charger_type)
		return dwc3_otg_pdata->get_charger_type(otg);

	return CHRG_UNKNOWN;
}

static enum dwc_otg_state do_charger_detection(struct dwc_otg2 *otg)
{
	enum dwc_otg_state state = DWC_STATE_INVALID;
	enum usb_charger_type charger = CHRG_UNKNOWN;
	unsigned long flags, ma = 0;

	charger = get_charger_type(otg);
	switch (charger) {
	case CHRG_ACA_C:
	case CHRG_ACA_A:
	case CHRG_ACA_B:
		otg_err(otg, "Ignore micro ACA charger.\n");
		charger = CHRG_UNKNOWN;
		break;
	case CHRG_SDP:
	case CHRG_CDP:
		state = DWC_STATE_B_PERIPHERAL;
		break;
	case CHRG_ACA_DOCK:
		state = DWC_STATE_A_HOST;
		break;
	case CHRG_DCP:
	case CHRG_SE1:
		state = DWC_STATE_CHARGING;
		break;
	case CHRG_UNKNOWN:
	default:
		if (is_self_powered_b_device(otg)) {
			state = DWC_STATE_A_HOST;
			charger = B_DEVICE;
			break;
		}
	};

	switch (charger) {
	case CHRG_ACA_DOCK:
	case CHRG_ACA_A:
	case CHRG_ACA_B:
	case CHRG_ACA_C:
	case CHRG_DCP:
	case CHRG_CDP:
	case CHRG_SE1:
		ma = 1500;
		break;
	case CHRG_SDP:
		/* Notify SDP current is 100ma before enumeration. */
		ma = 100;
		break;
	default:
		otg_err(otg, "Charger type is not valid to notify battery\n");
		return -EINVAL;
	}

	spin_lock_irqsave(&otg->lock, flags);
	otg->charging_cap.chrg_type = charger;
	otg->charging_cap.ma = ma;
	spin_unlock_irqrestore(&otg->lock, flags);

	switch (charger) {
	case CHRG_ACA_DOCK:
	case CHRG_DCP:
	case CHRG_CDP:
	case CHRG_SDP:
	case CHRG_SE1:
		if (dwc_otg_notify_charger_type(otg,
					OTG_CHR_STATE_CONNECTED) < 0)
			otg_err(otg, "Notify battery type failed!\n");
		break;
	default:
		break;
	}

	return state;
}

static enum dwc_otg_state do_connector_id_status(struct dwc_otg2 *otg)
{
	int ret;
	unsigned long flags;
	u32 events = 0, user_events = 0;
	u32 otg_mask = 0, user_mask = 0;
	enum dwc_otg_state state = DWC_STATE_INVALID;

	otg_dbg(otg, "\n");
	spin_lock_irqsave(&otg->lock, flags);
	otg->charging_cap.chrg_type = CHRG_UNKNOWN;
	otg->charging_cap.ma = 0;
	otg->charging_cap.chrg_state = OTG_CHR_STATE_DISCONNECTED;
	spin_unlock_irqrestore(&otg->lock, flags);

	otg_mask = OEVT_CONN_ID_STS_CHNG_EVNT |
			OEVT_B_DEV_SES_VLD_DET_EVNT;

	user_mask = USER_ID_B_CHANGE_EVENT |
				USER_ID_A_CHANGE_EVENT;

	if (dwc3_otg_pdata->b_idle)
		dwc3_otg_pdata->b_idle(otg);

	ret = sleep_until_event(otg, otg_mask,
			user_mask, &events,
			&user_events, 0);
	if (ret < 0)
		return DWC_STATE_EXIT;

	if (events & OEVT_B_DEV_SES_VLD_DET_EVNT) {
		otg_dbg(otg, "OEVT_B_DEV_SES_VLD_DET_EVNT\n");
		state = DWC_STATE_CHARGER_DETECTION;
	}

	if (events & OEVT_CONN_ID_STS_CHNG_EVNT) {
		otg_dbg(otg, "OEVT_CONN_ID_STS_CHNG_EVNT\n");
		state = DWC_STATE_WAIT_VBUS_RAISE;
	}

	if (user_events & USER_ID_A_CHANGE_EVENT) {
		otg_dbg(otg, "events is user id A change\n");
		state = DWC_STATE_A_HOST;
	}

	if (user_events & USER_ID_B_CHANGE_EVENT) {
		otg_dbg(otg, "events is user id B change\n");
		state = DWC_STATE_B_PERIPHERAL;
	}

	/** TODO: This is a workaround for latest hibernation-enabled bitfiles
     ** which have problems before initializing SRP.*/
	mdelay(50);

	return state;
}

static enum dwc_otg_state do_a_host(struct dwc_otg2 *otg)
{
	int rc = 0;
	u32 otg_events, user_events, otg_mask, user_mask;
	int id = RID_UNKNOWN;
	unsigned long flags;

	if (otg->charging_cap.chrg_type != CHRG_ACA_DOCK) {
		dwc_otg_enable_vbus(otg, 1);

		/* meant receive vbus valid event*/
		if (do_wait_vbus_raise(otg) !=
				DWC_STATE_CHARGER_DETECTION) {
			otg_err(otg, "Drive VBUS maybe fail!\n");
		}
	}

	rc = start_host(otg);
	if (rc < 0) {
		stop_host(otg);
		otg_err(otg, "start_host failed!");
		return DWC_STATE_INVALID;
	}

	otg_events = 0;
	user_events = 0;
	otg_mask = 0;
	user_mask = 0;

	otg_mask = OEVT_CONN_ID_STS_CHNG_EVNT |
			OEVT_A_DEV_SESS_END_DET_EVNT;
	user_mask =	USER_ID_B_CHANGE_EVENT;

	rc = sleep_until_event(otg,
			otg_mask, user_mask,
			&otg_events, &user_events, 0);
	if (rc < 0) {
		stop_host(otg);
		return DWC_STATE_EXIT;
	}

	/* Higher priority first */
	if (otg_events & OEVT_A_DEV_SESS_END_DET_EVNT) {
		otg_dbg(otg, "OEVT_A_DEV_SESS_END_DET_EVNT\n");

		/* ACA-Dock plug out */
		if (otg->charging_cap.chrg_type == CHRG_ACA_DOCK)
			dwc_otg_notify_charger_type(otg,
					OTG_CHR_STATE_DISCONNECTED);
		else
			dwc_otg_enable_vbus(otg, 0);

		stop_host(otg);
		return DWC_STATE_B_IDLE;
	}

	if (otg_events & OEVT_CONN_ID_STS_CHNG_EVNT) {
		otg_dbg(otg, "OEVT_CONN_ID_STS_CHNG_EVNT\n");
		id = get_id(otg);

		/* Plug out ACA_DOCK/USB device */
		if (id == RID_FLOAT) {
			if (otg->charging_cap.chrg_type == CHRG_ACA_DOCK) {
				/* ACA_DOCK plug out, receive
				 * id change prior to vBus change
				 */
				stop_host(otg);
			} else {
				/* Normal USB device plug out */
				spin_lock_irqsave(&otg->lock, flags);
				otg->charging_cap.chrg_type = CHRG_UNKNOWN;
				spin_unlock_irqrestore(&otg->lock, flags);

				stop_host(otg);
				dwc_otg_enable_vbus(otg, 0);
			}
		} else {
			otg_err(otg, "Meet invalid charger cases!");
			spin_lock_irqsave(&otg->lock, flags);
			otg->charging_cap.chrg_type = CHRG_UNKNOWN;
			spin_unlock_irqrestore(&otg->lock, flags);

			stop_host(otg);
		}
		return DWC_STATE_WAIT_VBUS_FALL;
	}

	/* Higher priority first */
	if (user_events & USER_ID_B_CHANGE_EVENT) {
		otg_dbg(otg, "USER_ID_B_CHANGE_EVENT\n");
		stop_host(otg);
		otg->user_events |= USER_ID_B_CHANGE_EVENT;
		return DWC_STATE_B_IDLE;
	}

	/* Invalid state */
	return DWC_STATE_INVALID;
}

static int do_b_peripheral(struct dwc_otg2 *otg)
{
	int rc = 0;
	u32 otg_mask, user_mask, otg_events, user_events;

	otg_mask = 0;
	user_mask = 0;
	otg_events = 0;
	user_events = 0;

	otg_mask = OEVT_A_DEV_SESS_END_DET_EVNT;
	user_mask = USER_ID_A_CHANGE_EVENT;

	rc = sleep_until_event(otg,
			otg_mask, user_mask,
			&otg_events, &user_events, 0);
	if (rc < 0)
		return DWC_STATE_EXIT;

	if (otg_events & OEVT_A_DEV_SESS_END_DET_EVNT) {
		otg_dbg(otg, "OEVT_A_DEV_SESS_END_DET_EVNT\n");
		dwc_otg_notify_charger_type(otg,
				OTG_CHR_STATE_DISCONNECTED);
		return DWC_STATE_B_IDLE;
	}

	if (user_events & USER_ID_A_CHANGE_EVENT) {
		otg_dbg(otg, "USER_ID_A_CHANGE_EVENT\n");
		otg->user_events |= USER_ID_A_CHANGE_EVENT;
		return DWC_STATE_B_IDLE;
	}

	return DWC_STATE_INVALID;
}

/* Charger driver may send ID change and VBus change event to OTG driver.
 * This is like IRQ handler, just the event source is from charger driver.
 * Because on Merrifield platform, the ID line and VBus line are connect to
 * PMic which can make USB controller and PHY power off to save power.
 */
static int dwc_otg_handle_notification(struct notifier_block *nb,
		unsigned long event, void *data)
{
	if (dwc3_otg_pdata->otg_notifier_handler)
		return dwc3_otg_pdata->otg_notifier_handler(nb, event, data);

	return NOTIFY_DONE;
}

int otg_main_thread(void *data)
{
	struct dwc_otg2 *otg = (struct dwc_otg2 *)data;

	/* Allow the thread to be killed by a signal, but set the signal mask
	 * to block everything but INT, TERM, KILL, and USR1. */
	allow_signal(SIGINT);
	allow_signal(SIGTERM);
	allow_signal(SIGKILL);
	allow_signal(SIGUSR1);

	/* Allow the thread to be frozen */
	set_freezable();

	/* Set device mode as default for EM driver WA */
	otg->user_events |= USER_ID_B_CHANGE_EVENT;

	otg_dbg(otg, "Thread running\n");
	while (otg->state != DWC_STATE_TERMINATED) {
		int next = DWC_STATE_B_IDLE;
		otg_dbg(otg, "\n\n\nMain thread entering state\n");

		switch (otg->state) {
		case DWC_STATE_B_IDLE:
			otg_dbg(otg, "DWC_STATE_B_IDLE\n");
			next = do_connector_id_status(otg);
			break;
		case DWC_STATE_CHARGER_DETECTION:
			otg_dbg(otg, "DWC_STATE_CHARGER_DETECTION\n");
			next = do_charger_detection(otg);
			break;
		case DWC_STATE_WAIT_VBUS_RAISE:
			otg_dbg(otg, "DWC_STATE_WAIT_VBUS_RAISE\n");
			next = do_wait_vbus_raise(otg);
			break;
		case DWC_STATE_WAIT_VBUS_FALL:
			otg_dbg(otg, "DWC_STATE_WAIT_VBUS_FALL\n");
			next = do_wait_vbus_fall(otg);
			break;
		case DWC_STATE_CHARGING:
			otg_dbg(otg, "DWC_STATE_CHARGING\n");
			next = do_charging(otg);
			break;
		case DWC_STATE_A_HOST:
			otg_dbg(otg, "DWC_STATE_A_HOST\n");
			next = do_a_host(otg);
			break;
		case DWC_STATE_B_PERIPHERAL:
			otg_dbg(otg, "DWC_STATE_B_PERIPHERAL\n");
			start_peripheral(otg);
			next = do_b_peripheral(otg);

			stop_peripheral(otg);
			break;
		case DWC_STATE_EXIT:
			otg_dbg(otg, "DWC_STATE_EXIT\n");
			next = DWC_STATE_TERMINATED;
			break;
		case DWC_STATE_INVALID:
			otg_dbg(otg, "DWC_STATE_INVALID!!!\n");
		default:
			otg_dbg(otg, "Unknown State %d, sleeping...\n",
					otg->state);
			sleep_main_thread(otg);
			break;
		}

		otg->prev = otg->state;
		otg->state = next;
	}

	otg->main_thread = NULL;
	otg_dbg(otg, "OTG main thread exiting....\n");

	return 0;
}

static void start_main_thread(struct dwc_otg2 *otg)
{
	enum dwc3_otg_mode mode = dwc3_otg_pdata->mode;
	bool children_ready = false;

	mutex_lock(&lock);

	if ((mode == DWC3_DEVICE_ONLY) &&
			otg->otg.gadget)
		children_ready = true;

	if ((mode == DWC3_HOST_ONLY) &&
			otg->otg.host)
		children_ready = true;

	if ((mode == DWC3_DRD) &&
			otg->otg.host && otg->otg.gadget)
		children_ready = true;

	if (!otg->main_thread && children_ready) {
		otg_dbg(otg, "Starting OTG main thread\n");
		otg->main_thread = kthread_create(otg_main_thread, otg, "otg");
		wake_up_process(otg->main_thread);
	}
	mutex_unlock(&lock);
}

static void stop_main_thread(struct dwc_otg2 *otg)
{
	mutex_lock(&lock);
	if (otg->main_thread) {
		otg_dbg(otg, "Stopping OTG main thread\n");
		otg->state = DWC_STATE_EXIT;
		dwc3_wakeup_otg_thread(otg);
	}
	mutex_unlock(&lock);
}

static int dwc_otg2_set_peripheral(struct usb_otg *x,
		struct usb_gadget *gadget)
{
	struct dwc_otg2 *otg;

	if (!x) {
		otg_err(otg, "otg is NULL!\n");
		return -ENODEV;
	}

	otg = xceiv_to_dwc_otg2(x);
	otg_dbg(otg, "\n");

	if (!gadget) {
		otg->otg.gadget = NULL;
		stop_main_thread(otg);
		return -ENODEV;
	}

	otg->otg.gadget = gadget;
	otg->usb2_phy.state = OTG_STATE_B_IDLE;
	start_main_thread(otg);
	return 0;
}

static int dwc_otg2_set_host(struct usb_otg *x, struct usb_bus *host)
{
	struct dwc_otg2 *otg;

	if (!x) {
		otg_dbg(otg, "otg is NULL!\n");
		return -ENODEV;
	}

	otg = xceiv_to_dwc_otg2(x);
	otg_dbg(otg, "\n");

	if (!host) {
		otg->otg.host = NULL;
		stop_main_thread(otg);
		return -ENODEV;
	}

	otg->otg.host = host;
	start_main_thread(otg);
	return 0;
}

static int ulpi_read(struct usb_phy *phy, u32 reg)
{
	struct dwc_otg2 *otg = container_of(phy, struct dwc_otg2, usb2_phy);
	u32 val32 = 0, count = 200;
	u8 val;

	reg &= 0xFF;

	while (count) {
		if (otg_read(otg, GUSB2PHYACC0) & GUSB2PHYACC0_VSTSBSY)
			udelay(5);
		else
			break;

		count--;
	}

	if (!count) {
		otg_err(otg, "USB2 PHY always busy!!\n");
		return -EBUSY;
	}

	count = 200;
	/* Determine if use extend registers access */
	if (reg & EXTEND_ULPI_REGISTER_ACCESS_MASK) {
		otg_dbg(otg, "Access extend registers 0x%x\n", reg);
		val32 = GUSB2PHYACC0_NEWREGREQ
			| GUSB2PHYACC0_REGADDR(ULPI_ACCESS_EXTENDED)
			| GUSB2PHYACC0_VCTRL(reg);
	} else {
		otg_dbg(otg, "Access normal registers 0x%x\n", reg);
		val32 = GUSB2PHYACC0_NEWREGREQ | GUSB2PHYACC0_REGADDR(reg)
			| GUSB2PHYACC0_VCTRL(0x00);
	}
	otg_write(otg, GUSB2PHYACC0, val32);

	while (count) {
		if (otg_read(otg, GUSB2PHYACC0) & GUSB2PHYACC0_VSTSDONE) {
			val = otg_read(otg, GUSB2PHYACC0) &
				  GUSB2PHYACC0_REGDATA_MASK;
			otg_dbg(otg, "%s - reg 0x%x data 0x%x\n",
					__func__, reg, val);
			return val;
		}

		count--;
	}

	otg_err(otg, "%s read PHY data failed.\n", __func__);

	return -ETIMEDOUT;
}

static int ulpi_write(struct usb_phy *phy, u32 val, u32 reg)
{
	struct dwc_otg2 *otg = container_of(phy, struct dwc_otg2, usb2_phy);
	u32 val32 = 0, count = 200;

	val &= 0xFF;
	reg &= 0xFF;

	while (count) {
		if (otg_read(otg, GUSB2PHYACC0) & GUSB2PHYACC0_VSTSBSY)
			udelay(5);
		else
			break;

		count--;
	}

	if (!count) {
		otg_err(otg, "USB2 PHY always busy!!\n");
		return -EBUSY;
	}

	count = 200;

	if (reg & EXTEND_ULPI_REGISTER_ACCESS_MASK) {
		otg_dbg(otg, "Access extend registers 0x%x\n", reg);
		val32 = GUSB2PHYACC0_NEWREGREQ
			| GUSB2PHYACC0_REGADDR(ULPI_ACCESS_EXTENDED)
			| GUSB2PHYACC0_VCTRL(reg)
			| GUSB2PHYACC0_REGWR | GUSB2PHYACC0_REGDATA(val);
	} else {
		otg_dbg(otg, "Access normal registers 0x%x\n", reg);
		val32 = GUSB2PHYACC0_NEWREGREQ
			| GUSB2PHYACC0_REGADDR(reg)
			| GUSB2PHYACC0_REGWR
			| GUSB2PHYACC0_REGDATA(val);
	}
	otg_write(otg, GUSB2PHYACC0, val32);

	while (count) {
		if (otg_read(otg, GUSB2PHYACC0) & GUSB2PHYACC0_VSTSDONE) {
			otg_dbg(otg, "%s - reg 0x%x data 0x%x write done\n",
					__func__, reg, val);
			return 0;
		}

		count--;
	}

	otg_err(otg, "%s read PHY data failed.\n", __func__);

	return -ETIMEDOUT;
}

static struct usb_phy_io_ops dwc_otg_io_ops = {
	.read = ulpi_read,
	.write = ulpi_write,
};

static struct dwc_otg2 *dwc3_otg_alloc(struct device *dev)
{
	struct dwc_otg2 *otg = NULL;
	struct usb_phy *usb_phy;
	int retval;

	otg = kzalloc(sizeof(*otg), GFP_KERNEL);
	if (!otg) {
		otg_err(otg, "Alloc otg failed\n");
		return NULL;
	}

	the_transceiver = otg;
	otg->otg_data = dev->platform_data;

	usb_phy = &otg->usb2_phy;
	otg->otg.phy = usb_phy;
	otg->usb2_phy.otg = &otg->otg;

	otg->dev		= dev;
	otg->usb3_phy.dev		= otg->dev;
	otg->usb3_phy.label		= "dwc-usb3-phy";
	otg->usb3_phy.state		= OTG_STATE_UNDEFINED;
	otg->usb3_phy.otg	= &otg->otg;
	otg->usb2_phy.dev		= otg->dev;
	otg->usb2_phy.label		= "dwc-usb2-phy";
	otg->usb2_phy.state		= OTG_STATE_UNDEFINED;
	otg->usb2_phy.set_power	= dwc3_otg_pdata->set_power;
	otg->usb2_phy.get_chrg_status	= dwc_otg_get_chrg_status;
	otg->usb2_phy.io_ops = &dwc_otg_io_ops;
	otg->usb2_phy.otg	= &otg->otg;
	otg->otg.set_host	= dwc_otg2_set_host;
	otg->otg.set_peripheral	= dwc_otg2_set_peripheral;
	ATOMIC_INIT_NOTIFIER_HEAD(&otg->usb2_phy.notifier);
	ATOMIC_INIT_NOTIFIER_HEAD(&otg->usb3_phy.notifier);

	otg->state = DWC_STATE_B_IDLE;
	spin_lock_init(&otg->lock);
	init_waitqueue_head(&otg->main_wq);

	/* Register otg notifier to monitor ID and VBus change events */
	otg->nb.notifier_call = dwc_otg_handle_notification;
	usb_register_notifier(&otg->usb2_phy, &otg->nb);

	otg_dbg(otg, "Version: %s\n", VERSION);
	retval = usb_add_phy(&otg->usb2_phy, USB_PHY_TYPE_USB2);
	if (retval) {
		otg_err(otg, "can't register transceiver, err: %d\n",
			retval);
		goto err1;
	}

	retval = usb_add_phy(&otg->usb3_phy, USB_PHY_TYPE_USB3);
	if (retval) {
		otg_err(otg, "can't register transceiver, err: %d\n",
			retval);
		goto err2;
	}

	return otg;

err2:
	usb_remove_phy(&otg->usb2_phy);

err1:
	kfree(otg);
	otg = NULL;

	return otg;
}

static int dwc3_otg_create_children(struct dwc_otg2 *otg,
		struct resource *res, int num)
{
	struct platform_device *dwc_host, *dwc_gadget;
	enum dwc3_otg_mode mode = dwc3_otg_pdata->mode;
	int retval = 0, i;

	if (!otg || !res)
		return -EINVAL;

	if (num != 2)
		return -EINVAL;

	dwc_host = dwc_gadget = NULL;

	for (i = 0; i < 2; i++) {
		if (res[i].flags == IORESOURCE_MEM) {
			otg->usb2_phy.io_priv = ioremap_nocache(
				res[i].start, res[i].end - res[i].start);
			if (!otg->usb2_phy.io_priv) {
				otg_err(otg, "dwc3 otg ioremap failed\n");
				return -ENOMEM;
			}
			break;
		}
	}

	/* resource have no mem io resource */
	if (!otg->usb2_phy.io_priv)
		return -EINVAL;

	platform_par = kzalloc(sizeof(*platform_par), GFP_KERNEL);
	if (!platform_par) {
		otg_err(otg, "alloc dwc_device_par failed\n");
		goto err1;
	}

	platform_par->io_addr = otg->usb2_phy.io_priv;
	platform_par->len = res[i].end - res[i].start;

	if (mode == DWC3_DEVICE_ONLY)
		goto device_only;

	dwc_host = platform_device_alloc(DWC3_HOST_NAME,
			HOST_DEVID);
	if (!dwc_host) {
		otg_err(otg, "couldn't allocate dwc3 host device\n");
		goto err2;
	}

	retval = platform_device_add_resources(dwc_host, res, num);
	if (retval) {
		otg_err(otg, "couldn't add resources to dwc3 device\n");
		goto err3;
	}

	platform_device_add_data(dwc_host, platform_par,
			sizeof(struct dwc_device_par));

	dwc_host->dev.dma_mask = otg->dev->dma_mask;
	dwc_host->dev.dma_parms = otg->dev->dma_parms;
	dwc_host->dev.parent = otg->dev;

	retval = platform_device_add(dwc_host);
	if (retval)	{
		otg_err(otg, "failed to register dwc3 host\n");
		goto err1;
	}

	otg->host = dwc_host;

	if (mode != DWC3_DRD)
		return 0;

device_only:
	dwc_gadget = platform_device_alloc(DWC3_DEVICE_NAME,
			GADGET_DEVID);
	if (!dwc_gadget) {
		otg_err(otg, "couldn't allocate dwc3 device\n");
		goto err3;
	}

	retval = platform_device_add_resources(dwc_gadget,
				res, num);
	if (retval) {
		otg_err(otg, "couldn't add resources to dwc3 device\n");
		goto err3;
	}

	dwc_gadget->dev.dma_mask = otg->dev->dma_mask;
	dwc_gadget->dev.dma_parms = otg->dev->dma_parms;
	dwc_gadget->dev.parent = otg->dev;

	platform_device_add_data(dwc_gadget, platform_par,
			sizeof(struct dwc_device_par));
	retval = platform_device_add(dwc_gadget);
	if (retval) {
		otg_err(otg, "failed to register dwc3 gadget\n");
		goto err3;
	}
	otg->gadget = dwc_gadget;

	return 0;

err3:
	if (mode == DWC3_DRD)
		platform_device_unregister(dwc_host);

err2:
	kfree(platform_par);

err1:
	iounmap(otg->usb2_phy.io_priv);

	return retval;
}

#ifdef CONFIG_PCI

static int dwc_otg_probe(struct pci_dev *pdev,
			const struct pci_device_id *id)
{
	int retval = 0;
	struct resource		res[2];
	struct dwc_otg2 *otg = NULL;
	unsigned long resource, len;

	if (!dwc3_otg_pdata)
		return -ENODEV;

	if (pci_enable_device(pdev) < 0) {
		dev_err(&pdev->dev, "pci device enable failed\n");
		return -ENODEV;
	}

	pci_set_power_state(pdev, PCI_D0);
	pci_set_master(pdev);

	otg = dwc3_otg_alloc(&pdev->dev);
	if (!otg) {
		otg_err(otg, "dwc3 otg init failed\n");
		goto err;
	}

	/* control register: BAR 0 */
	resource = pci_resource_start(pdev, 0);
	len = pci_resource_len(pdev, 0);
	if (!request_mem_region(resource, len, driver_name)) {
		otg_err(otg, "Request memory region failed\n");
		retval = -EBUSY;
		goto err;
	}

	otg_dbg(otg, "dwc otg pci resouce: 0x%lu, len: 0x%lu\n",
			resource, len);
	otg_dbg(otg, "vendor: 0x%x, device: 0x%x\n",
			pdev->vendor, pdev->device);

	memset(res, 0x00, sizeof(struct resource) * ARRAY_SIZE(res));

	res[0].start	= pci_resource_start(pdev, 0);
	res[0].end	= pci_resource_end(pdev, 0);
	res[0].name	= "dwc_usb3_io";
	res[0].flags	= IORESOURCE_MEM;

	res[1].start	= pdev->irq;
	res[1].name	= "dwc_usb3_irq";
	res[1].flags	= IORESOURCE_IRQ;

	retval = dwc3_otg_create_children(otg, res, ARRAY_SIZE(res));
	if (retval) {
		otg_err(otg, "dwc3 otg create alloc children failed\n");
		goto err;
	}

	otg->irqnum = pdev->irq;

	if (dwc3_otg_pdata->platform_init) {
		retval = dwc3_otg_pdata->platform_init(otg);
		if (retval)
			goto err;
	}

	return 0;

err:
	if (the_transceiver)
		dwc_otg_remove(pdev);

	return retval;
}

static void dwc_otg_remove(struct pci_dev *pdev)
{
	struct dwc_otg2 *otg = the_transceiver;
	int resource, len;

	if (otg->gadget)
		platform_device_unregister(otg->gadget);
	if (otg->host)
		platform_device_unregister(otg->host);

	kfree(platform_par);
	iounmap(otg->usb2_phy.io_priv);

	usb_remove_phy(&otg->usb2_phy);
	usb_remove_phy(&otg->usb3_phy);
	kfree(otg);
	otg = NULL;

	resource = pci_resource_start(pdev, 0);
	len = pci_resource_len(pdev, 0);
	release_mem_region(resource, len);

	pci_disable_device(pdev);

	the_transceiver = NULL;
}

static DEFINE_PCI_DEVICE_TABLE(pci_ids) = {
	{ PCI_DEVICE_CLASS(((PCI_CLASS_SERIAL_USB << 8) | 0x20), ~0),
		.vendor = PCI_VENDOR_ID_INTEL,
		.device = PCI_DEVICE_ID_DWC,
	},
	{ /* end: all zeroes */ }
};

static struct pci_driver dwc_otg_pci_driver = {
	.name =		(char *) driver_name,
	.id_table =	pci_ids,
	.probe =	dwc_otg_probe,
	.remove =	dwc_otg_remove,
	.driver = {
		.name = (char *) driver_name,
		.owner = THIS_MODULE,
	},
};
#endif

int dwc3_otg_register(struct dwc3_otg_hw_ops *pdata)
{
	int retval;

	if (!pdata)
		return -EINVAL;

	if (dwc3_otg_pdata)
		return -EBUSY;

	dwc3_otg_pdata = pdata;

#ifdef CONFIG_PCI
	retval = pci_register_driver(&dwc_otg_pci_driver);
#endif
	mutex_init(&lock);

	return retval;
}
EXPORT_SYMBOL_GPL(dwc3_otg_register);

int dwc3_otg_unregister(struct dwc3_otg_hw_ops *pdata)
{
	if (!pdata)
		return -EINVAL;

	if (dwc3_otg_pdata != pdata)
		return -EINVAL;

	dwc3_otg_pdata = NULL;

#ifdef CONFIG_PCI
	pci_unregister_driver(&dwc_otg_pci_driver);
#endif
	mutex_destroy(&lock);

	return 0;
}
EXPORT_SYMBOL_GPL(dwc3_otg_unregister);

static int __init dwc_otg_init(void)
{
	return 0;
}
module_init(dwc_otg_init);

static void __exit dwc_otg_exit(void)
{
}
module_exit(dwc_otg_exit);

MODULE_AUTHOR("Wang Yu <yu.y.wang@intel.com>");
MODULE_DESCRIPTION("DWC3 OTG-PCI Driver");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_VERSION("1.0");
