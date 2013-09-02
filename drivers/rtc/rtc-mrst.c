/*
 * rtc-mrst.c: Driver for Moorestown virtual RTC
 *
 * (C) Copyright 2009 Intel Corporation
 * Author: Jacob Pan (jacob.jun.pan@intel.com)
 *	   Feng Tang (feng.tang@intel.com)
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 *
 * Note:
 * VRTC is emulated by system controller firmware, the real HW
 * RTC is located in the PMIC device. SCU FW shadows PMIC RTC
 * in a memory mapped IO space that is visible to the host IA
 * processor.
 *
 * This driver is based upon drivers/rtc/rtc-cmos.c
 */

/*
 * Note:
 *  * vRTC only supports binary mode and 24H mode
 *  * vRTC only support PIE and AIE, no UIE, and its PIE only happens
 *    at 23:59:59pm everyday, no support for adjustable frequency
 *  * Alarm function is also limited to hr/min/sec.
 */

#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/sfi.h>
#include <linux/io.h>

#include <asm-generic/rtc.h>
#include <asm/intel_scu_ipc.h>
#include <asm/intel-mid.h>
#include <asm/mrst-vrtc.h>
#include <linux/rpmsg.h>
#include <asm/intel_mid_rpmsg.h>

struct mrst_rtc {
	struct rtc_device	*rtc;
	struct device		*dev;
	int			irq;
	struct resource		*iomem;

	u8			enabled_wake;
	u8			suspend_ctrl;
};

/* both platform and pnp busses use negative numbers for invalid irqs */
#define is_valid_irq(n)		((n) >= 0)

static const char driver_name[] = "rtc_mrst";

#define	RTC_IRQMASK	(RTC_PF | RTC_AF)

#define OSHOB_ALARM_OFFSET 0x68
#define OSHOB_DAYW_OFFSET  0x00
#define OSHOB_DAYM_OFFSET  0x01
#define OSHOB_MON_OFFSET   0x02
#define OSHOB_YEAR_OFFSET  0x03

static u32 oshob_base;
static void __iomem *oshob_addr;

static struct rpmsg_instance *vrtc_mrst_instance;

static inline int is_intr(u8 rtc_intr)
{
	if (!(rtc_intr & RTC_IRQF))
		return 0;
	return rtc_intr & RTC_IRQMASK;
}

static inline unsigned char vrtc_is_updating(void)
{
	unsigned char uip;
	unsigned long flags;

	spin_lock_irqsave(&rtc_lock, flags);
	uip = (vrtc_cmos_read(RTC_FREQ_SELECT) & RTC_UIP);
	spin_unlock_irqrestore(&rtc_lock, flags);
	return uip;
}

/* If the interrupt is of alarm-type-RTC_AF, then check if it's for
 * the correct day. With the support for alarms more than 24-hours,
 * alarm-date is compared with date-fields in OSHOB, as the vRTC
 * doesn't have date-fields for alarm
 */
static int is_valid_af(u8 rtc_intr)
{
	char *p;
	unsigned long vrtc_date, oshob_date;

	if ((__intel_mid_cpu_chip == INTEL_MID_CPU_CHIP_PENWELL) ||
	    (__intel_mid_cpu_chip == INTEL_MID_CPU_CHIP_CLOVERVIEW)) {
		if (rtc_intr & RTC_AF) {
			p = (char *) &vrtc_date;
			*(p+1) = vrtc_cmos_read(RTC_DAY_OF_MONTH);
			*(p+2) = vrtc_cmos_read(RTC_MONTH);
			*(p+3) = vrtc_cmos_read(RTC_YEAR);

			oshob_date = readl(oshob_addr);
			if ((oshob_date & 0xFFFFFF00)
					!= (vrtc_date & 0xFFFFFF00))
				return false;
		}
	}

	return true;
}

/*
 * rtc_time's year contains the increment over 1900, but vRTC's YEAR
 * register can't be programmed to value larger than 0x64, so vRTC
 * driver chose to use 1972 (1970 is UNIX time start point) as the base,
 * and does the translation at read/write time.
 *
 * Why not just use 1970 as the offset? it's because using 1972 will
 * make it consistent in leap year setting for both vrtc and low-level
 * physical rtc devices. Then why not use 1960 as the offset? If we use
 * 1960, for a device's first use, its YEAR register is 0 and the system
 * year will be parsed as 1960 which is not a valid UNIX time and will
 * cause many applications to fail mysteriously.
 */
static int mrst_read_time(struct device *dev, struct rtc_time *time)
{
	unsigned long flags;

	if (vrtc_is_updating())
		mdelay(20);

	spin_lock_irqsave(&rtc_lock, flags);
	time->tm_sec = vrtc_cmos_read(RTC_SECONDS);
	time->tm_min = vrtc_cmos_read(RTC_MINUTES);
	time->tm_hour = vrtc_cmos_read(RTC_HOURS);
	time->tm_mday = vrtc_cmos_read(RTC_DAY_OF_MONTH);
	time->tm_mon = vrtc_cmos_read(RTC_MONTH);
	time->tm_year = vrtc_cmos_read(RTC_YEAR);
	spin_unlock_irqrestore(&rtc_lock, flags);

	/* Adjust for the 1972/1900 */
	time->tm_year += 72;
	time->tm_mon--;
	return rtc_valid_tm(time);
}

static int mrst_set_time(struct device *dev, struct rtc_time *time)
{
	int ret;
	unsigned long flags;
	unsigned char mon, day, hrs, min, sec;
	unsigned int yrs;

	yrs = time->tm_year;
	mon = time->tm_mon + 1;   /* tm_mon starts at zero */
	day = time->tm_mday;
	hrs = time->tm_hour;
	min = time->tm_min;
	sec = time->tm_sec;

	if (yrs < 72 || yrs > 138)
		return -EINVAL;
	yrs -= 72;

	spin_lock_irqsave(&rtc_lock, flags);

	vrtc_cmos_write(yrs, RTC_YEAR);
	vrtc_cmos_write(mon, RTC_MONTH);
	vrtc_cmos_write(day, RTC_DAY_OF_MONTH);
	vrtc_cmos_write(hrs, RTC_HOURS);
	vrtc_cmos_write(min, RTC_MINUTES);
	vrtc_cmos_write(sec, RTC_SECONDS);

	spin_unlock_irqrestore(&rtc_lock, flags);

	ret = rpmsg_send_simple_command(vrtc_mrst_instance,
				IPCMSG_VRTC, IPC_CMD_VRTC_SETTIME);
	return ret;
}

static int mrst_read_alarm(struct device *dev, struct rtc_wkalrm *t)
{
	struct mrst_rtc	*mrst = dev_get_drvdata(dev);
	unsigned char rtc_control;

	if (!is_valid_irq(mrst->irq))
		return -EIO;

	/* Basic alarms only support hour, minute, and seconds fields.
	 * Some also support day and month, for alarms up to a year in
	 * the future.
	 */
	t->time.tm_mday = -1;
	t->time.tm_mon = -1;
	t->time.tm_year = -1;

	/* vRTC only supports binary mode */
	spin_lock_irq(&rtc_lock);
	t->time.tm_sec = vrtc_cmos_read(RTC_SECONDS_ALARM);
	t->time.tm_min = vrtc_cmos_read(RTC_MINUTES_ALARM);
	t->time.tm_hour = vrtc_cmos_read(RTC_HOURS_ALARM);

	rtc_control = vrtc_cmos_read(RTC_CONTROL);
	spin_unlock_irq(&rtc_lock);

	t->enabled = !!(rtc_control & RTC_AIE);
	t->pending = 0;

	return 0;
}

static void mrst_checkintr(struct mrst_rtc *mrst, unsigned char rtc_control)
{
	unsigned char	rtc_intr;

	/*
	 * NOTE after changing RTC_xIE bits we always read INTR_FLAGS;
	 * allegedly some older rtcs need that to handle irqs properly
	 */
	rtc_intr = vrtc_cmos_read(RTC_INTR_FLAGS);
	rtc_intr &= (rtc_control & RTC_IRQMASK) | RTC_IRQF;
	if (is_intr(rtc_intr) && is_valid_af(rtc_intr))
		rtc_update_irq(mrst->rtc, 1, rtc_intr);
}

static void mrst_irq_enable(struct mrst_rtc *mrst, unsigned char mask)
{
	unsigned char	rtc_control;

	/*
	 * Flush any pending IRQ status, notably for update irqs,
	 * before we enable new IRQs
	 */
	rtc_control = vrtc_cmos_read(RTC_CONTROL);
	mrst_checkintr(mrst, rtc_control);

	rtc_control |= mask;
	vrtc_cmos_write(rtc_control, RTC_CONTROL);

	mrst_checkintr(mrst, rtc_control);
}

static void mrst_irq_disable(struct mrst_rtc *mrst, unsigned char mask)
{
	unsigned char	rtc_control;

	rtc_control = vrtc_cmos_read(RTC_CONTROL);
	rtc_control &= ~mask;
	vrtc_cmos_write(rtc_control, RTC_CONTROL);
	mrst_checkintr(mrst, rtc_control);
}

static int mrst_set_alarm(struct device *dev, struct rtc_wkalrm *t)
{
	struct mrst_rtc	*mrst = dev_get_drvdata(dev);
	unsigned char hrs, min, sec;
	unsigned char wday, mday, mon, year;
	int ret = 0;

	if (!is_valid_irq(mrst->irq))
		return -EIO;

	hrs = t->time.tm_hour;
	min = t->time.tm_min;
	sec = t->time.tm_sec;

	wday = t->time.tm_wday;
	mday = t->time.tm_mday;
	mon = t->time.tm_mon;
	year = t->time.tm_year;

	spin_lock_irq(&rtc_lock);
	/* Next rtc irq must not be from previous alarm setting */
	mrst_irq_disable(mrst, RTC_AIE);

	/* Update alarm */
	vrtc_cmos_write(hrs, RTC_HOURS_ALARM);
	vrtc_cmos_write(min, RTC_MINUTES_ALARM);
	vrtc_cmos_write(sec, RTC_SECONDS_ALARM);

	if ((__intel_mid_cpu_chip == INTEL_MID_CPU_CHIP_PENWELL) ||
	    (__intel_mid_cpu_chip == INTEL_MID_CPU_CHIP_CLOVERVIEW)) {
		/* Support for date-field in Alarm using OSHOB
		 * Since, vRTC doesn't have Alarm-registers for date-fields,
		 * write date-fields into OSHOB for SCU to sync to MSIC-RTC */
		writeb(wday, oshob_addr+OSHOB_DAYW_OFFSET);
		writeb(mday, oshob_addr+OSHOB_DAYM_OFFSET);
		writeb(mon+1, oshob_addr+OSHOB_MON_OFFSET);
		/* Adjust for the 1972/1900 */
		writeb(year-72, oshob_addr+OSHOB_YEAR_OFFSET);
	}

	if (t->enabled)
		mrst_irq_enable(mrst, RTC_AIE);

	spin_unlock_irq(&rtc_lock);

	return 0;
}

#if defined(CONFIG_RTC_INTF_DEV) || defined(CONFIG_RTC_INTF_DEV_MODULE)

/* Currently, the vRTC doesn't support UIE ON/OFF */
static int
mrst_rtc_ioctl(struct device *dev, unsigned int cmd, unsigned long arg)
{
	struct mrst_rtc	*mrst = dev_get_drvdata(dev);
	unsigned long	flags;

	switch (cmd) {
	case RTC_AIE_OFF:
	case RTC_AIE_ON:
		if (!is_valid_irq(mrst->irq))
			return -EINVAL;
		break;
	default:
		/* PIE ON/OFF is handled by mrst_irq_set_state() */
		return -ENOIOCTLCMD;
	}

	spin_lock_irqsave(&rtc_lock, flags);
	switch (cmd) {
	case RTC_AIE_OFF:	/* alarm off */
		mrst_irq_disable(mrst, RTC_AIE);
		break;
	case RTC_AIE_ON:	/* alarm on */
		mrst_irq_enable(mrst, RTC_AIE);
		break;
	}
	spin_unlock_irqrestore(&rtc_lock, flags);
	return 0;
}

#else
#define	mrst_rtc_ioctl	NULL
#endif

#if defined(CONFIG_RTC_INTF_PROC) || defined(CONFIG_RTC_INTF_PROC_MODULE)

static int mrst_procfs(struct device *dev, struct seq_file *seq)
{
	unsigned char	rtc_control, valid;

	spin_lock_irq(&rtc_lock);
	rtc_control = vrtc_cmos_read(RTC_CONTROL);
	valid = vrtc_cmos_read(RTC_VALID);
	spin_unlock_irq(&rtc_lock);

	return seq_printf(seq,
			"periodic_IRQ\t: %s\n"
			"alarm\t\t: %s\n"
			"BCD\t\t: no\n"
			"periodic_freq\t: daily (not adjustable)\n",
			(rtc_control & RTC_PIE) ? "on" : "off",
			(rtc_control & RTC_AIE) ? "on" : "off");
}

#else
#define	mrst_procfs	NULL
#endif

static int mrst_alarm_irq_enable(struct device *dev, unsigned int enabled)
{
	struct mrst_rtc *mrst = dev_get_drvdata(dev);

	if (enabled)
		mrst_irq_enable(mrst, RTC_AIE);
	else
		mrst_irq_disable(mrst, RTC_AIE);

	return 0;
}

static const struct rtc_class_ops mrst_rtc_ops = {
	.ioctl		  = mrst_rtc_ioctl,
	.read_time	  = mrst_read_time,
	.set_time	  = mrst_set_time,
	.read_alarm	  = mrst_read_alarm,
	.set_alarm	  = mrst_set_alarm,
	.proc		  = mrst_procfs,
	.alarm_irq_enable = mrst_alarm_irq_enable,
};

static struct mrst_rtc	mrst_rtc;

/*
 * When vRTC IRQ is captured by SCU FW, FW will clear the AIE bit in
 * Reg B, so no need for this driver to clear it
 */
static irqreturn_t mrst_rtc_irq(int irq, void *p)
{
	u8 irqstat;
	int ret = 0;

	spin_lock(&rtc_lock);
	/* This read will clear all IRQ flags inside Reg C */
	irqstat = vrtc_cmos_read(RTC_INTR_FLAGS);
	irqstat &= RTC_IRQMASK | RTC_IRQF;
	ret = is_valid_af(irqstat);
	spin_unlock(&rtc_lock);

	if (is_intr(irqstat)) {
		/* If it's an alarm-interrupt, update RTC-IRQ only if it's
		 * for current day. Alarms beyond 24-hours will result in
		 * interrupts at given time, everyday till actual alarm-date.
		 * From hardware perspective, it's still a valid interrupt,
		 * hence need to return IRQ_HANDLED. */
		if (ret)
			rtc_update_irq(p, 1, irqstat);

		return IRQ_HANDLED;
	} else {
		pr_err("vRTC: error in IRQ handler\n");
		return IRQ_NONE;
	}
}

static int
vrtc_mrst_do_probe(struct device *dev, struct resource *iomem, int rtc_irq)
{
	int retval = 0;
	unsigned char rtc_control;

	/* There can be only one ... */
	if (mrst_rtc.dev)
		return -EBUSY;

	if (!iomem)
		return -ENODEV;

	iomem = request_mem_region(iomem->start,
			iomem->end + 1 - iomem->start,
			driver_name);
	if (!iomem) {
		dev_dbg(dev, "i/o mem already in use.\n");
		return -EBUSY;
	}

	mrst_rtc.irq = rtc_irq;
	mrst_rtc.iomem = iomem;
	mrst_rtc.dev = dev;
	dev_set_drvdata(dev, &mrst_rtc);

	mrst_rtc.rtc = rtc_device_register(driver_name, dev,
				&mrst_rtc_ops, THIS_MODULE);
	if (IS_ERR(mrst_rtc.rtc)) {
		retval = PTR_ERR(mrst_rtc.rtc);
		goto cleanup0;
	}

	rename_region(iomem, dev_name(&mrst_rtc.rtc->dev));

	spin_lock_irq(&rtc_lock);
	mrst_irq_disable(&mrst_rtc, RTC_PIE | RTC_AIE);
	rtc_control = vrtc_cmos_read(RTC_CONTROL);
	spin_unlock_irq(&rtc_lock);

	if (!(rtc_control & RTC_24H) || (rtc_control & (RTC_DM_BINARY)))
		dev_dbg(dev, "TODO: support more than 24-hr BCD mode\n");

	if (is_valid_irq(rtc_irq)) {
		retval = request_irq(rtc_irq, mrst_rtc_irq,
				IRQF_NO_SUSPEND, dev_name(&mrst_rtc.rtc->dev),
				mrst_rtc.rtc);
		if (retval < 0) {
			dev_dbg(dev, "IRQ %d is already in use, err %d\n",
				rtc_irq, retval);
			goto cleanup1;
		}
	}

	/* make RTC device wake capable from sleep */
	device_init_wakeup(dev, true);

	if ((__intel_mid_cpu_chip == INTEL_MID_CPU_CHIP_PENWELL) ||
	    (__intel_mid_cpu_chip == INTEL_MID_CPU_CHIP_CLOVERVIEW)) {
		retval = rpmsg_send_command(vrtc_mrst_instance,
				IPCMSG_GET_HOBADDR, 0, NULL, &oshob_base, 0, 1);
		if (retval < 0) {
			dev_dbg(dev,
				"Unable to get OSHOB base address, err %d\n",
				retval);
			goto cleanup1;
		}

		oshob_addr = ioremap_nocache(oshob_base+OSHOB_ALARM_OFFSET, 4);
		if (!oshob_addr) {
			dev_dbg(dev, "Unable to do ioremap for OSHOB\n");
			retval = -ENOMEM;
			goto cleanup1;
		}
	}

	dev_info(dev, "vRTC driver initialised\n");
	return 0;

cleanup1:
	rtc_device_unregister(mrst_rtc.rtc);
cleanup0:
	dev_set_drvdata(dev, NULL);
	mrst_rtc.dev = NULL;
	release_mem_region(iomem->start, resource_size(iomem));
	dev_err(dev, "rtc-mrst: unable to initialise\n");
	return retval;
}

static void rtc_mrst_do_shutdown(void)
{
	spin_lock_irq(&rtc_lock);
	mrst_irq_disable(&mrst_rtc, RTC_IRQMASK);
	spin_unlock_irq(&rtc_lock);
}

static void rtc_mrst_do_remove(struct device *dev)
{
	struct mrst_rtc	*mrst = dev_get_drvdata(dev);
	struct resource *iomem;

	rtc_mrst_do_shutdown();

	if (is_valid_irq(mrst->irq))
		free_irq(mrst->irq, mrst->rtc);

	if ((__intel_mid_cpu_chip == INTEL_MID_CPU_CHIP_PENWELL) ||
	    (__intel_mid_cpu_chip == INTEL_MID_CPU_CHIP_CLOVERVIEW)) {
		if (oshob_addr != NULL)
			iounmap(oshob_addr);
	}

	rtc_device_unregister(mrst->rtc);
	mrst->rtc = NULL;

	iomem = mrst->iomem;
	release_mem_region(iomem->start, resource_size(iomem));
	mrst->iomem = NULL;

	mrst->dev = NULL;
	dev_set_drvdata(dev, NULL);
}

#ifdef	CONFIG_PM
static int mrst_suspend(struct device *dev)
{
	struct mrst_rtc	*mrst = dev_get_drvdata(dev);
	unsigned char	tmp;

	/* Only the alarm might be a wakeup event source */
	spin_lock_irq(&rtc_lock);
	mrst->suspend_ctrl = tmp = vrtc_cmos_read(RTC_CONTROL);
	if (tmp & (RTC_PIE | RTC_AIE)) {
		unsigned char	mask;

		if (device_may_wakeup(dev))
			mask = RTC_IRQMASK & ~RTC_AIE;
		else
			mask = RTC_IRQMASK;
		tmp &= ~mask;
		vrtc_cmos_write(tmp, RTC_CONTROL);

		mrst_checkintr(mrst, tmp);
	}
	spin_unlock_irq(&rtc_lock);

	if (tmp & RTC_AIE) {
		mrst->enabled_wake = 1;
		enable_irq_wake(mrst->irq);
	}

	dev_dbg(&mrst_rtc.rtc->dev, "suspend%s, ctrl %02x\n",
			(tmp & RTC_AIE) ? ", alarm may wake" : "",
			tmp);

	return 0;
}

/*
 * We want RTC alarms to wake us from the deep power saving state
 */
static inline int mrst_poweroff(struct device *dev)
{
	return mrst_suspend(dev);
}

static int mrst_resume(struct device *dev)
{
	struct mrst_rtc	*mrst = dev_get_drvdata(dev);
	unsigned char tmp = mrst->suspend_ctrl;

	/* Re-enable any irqs previously active */
	if (tmp & RTC_IRQMASK) {
		unsigned char	mask;

		if (mrst->enabled_wake) {
			disable_irq_wake(mrst->irq);
			mrst->enabled_wake = 0;
		}

		spin_lock_irq(&rtc_lock);
		do {
			vrtc_cmos_write(tmp, RTC_CONTROL);

			mask = vrtc_cmos_read(RTC_INTR_FLAGS);
			mask &= (tmp & RTC_IRQMASK) | RTC_IRQF;
			if (!(is_intr(mask) && is_valid_af(mask)))
				break;

			rtc_update_irq(mrst->rtc, 1, mask);
			tmp &= ~RTC_AIE;
		} while (mask & RTC_AIE);
		spin_unlock_irq(&rtc_lock);
	}

	dev_dbg(&mrst_rtc.rtc->dev, "resume, ctrl %02x\n", tmp);

	return 0;
}

#else
#define	mrst_suspend	NULL
#define	mrst_resume	NULL

static inline int mrst_poweroff(struct device *dev)
{
	return -ENOSYS;
}

#endif

static int vrtc_mrst_platform_probe(struct platform_device *pdev)
{
	return vrtc_mrst_do_probe(&pdev->dev,
			platform_get_resource(pdev, IORESOURCE_MEM, 0),
			platform_get_irq(pdev, 0));
}

static int vrtc_mrst_platform_remove(struct platform_device *pdev)
{
	rtc_mrst_do_remove(&pdev->dev);
	return 0;
}

static void vrtc_mrst_platform_shutdown(struct platform_device *pdev)
{
	if (system_state == SYSTEM_POWER_OFF && !mrst_poweroff(&pdev->dev))
		return;

	rtc_mrst_do_shutdown();
}

MODULE_ALIAS("platform:vrtc_mrst");

static const struct dev_pm_ops vrtc_mrst_platform_driver_pm_ops = {
	.suspend	= mrst_suspend,
	.resume		= mrst_resume,
};

static struct platform_driver vrtc_mrst_platform_driver = {
	.probe		= vrtc_mrst_platform_probe,
	.remove		= vrtc_mrst_platform_remove,
	.shutdown	= vrtc_mrst_platform_shutdown,
	.driver.name	= (char *) driver_name,
	.driver.pm	= &vrtc_mrst_platform_driver_pm_ops,
};

static int vrtc_mrst_init(void)
{
	return platform_driver_register(&vrtc_mrst_platform_driver);
}

static void vrtc_mrst_exit(void)
{
	platform_driver_unregister(&vrtc_mrst_platform_driver);
}

static int vrtc_mrst_rpmsg_probe(struct rpmsg_channel *rpdev)
{
	int ret;

	if (rpdev == NULL) {
		pr_err("vrtc_mrst rpmsg channel not created\n");
		ret = -ENODEV;
		goto out;
	}

	dev_info(&rpdev->dev, "Probed vrtc_mrst rpmsg device\n");

	/* Allocate rpmsg instance for fw_update*/
	ret = alloc_rpmsg_instance(rpdev, &vrtc_mrst_instance);
	if (!vrtc_mrst_instance) {
		dev_err(&rpdev->dev, "kzalloc vrtc_mrst instance failed\n");
		goto out;
	}

	/* Initialize rpmsg instance */
	init_rpmsg_instance(vrtc_mrst_instance);

	ret = vrtc_mrst_init();
	if (ret)
		free_rpmsg_instance(rpdev, &vrtc_mrst_instance);

out:
	return ret;
}

static void vrtc_mrst_rpmsg_remove(struct rpmsg_channel *rpdev)
{
	vrtc_mrst_exit();
	free_rpmsg_instance(rpdev, &vrtc_mrst_instance);
	dev_info(&rpdev->dev, "Removed vrtc_mrst rpmsg device\n");
}

static void vrtc_mrst_rpmsg_cb(struct rpmsg_channel *rpdev, void *data,
					int len, void *priv, u32 src)
{
	dev_warn(&rpdev->dev, "unexpected, message\n");

	print_hex_dump(KERN_DEBUG, __func__, DUMP_PREFIX_NONE, 16, 1,
		       data, len,  true);
}

static struct rpmsg_device_id vrtc_mrst_rpmsg_id_table[] = {
	{ .name	= "rpmsg_vrtc" },
	{ },
};
MODULE_DEVICE_TABLE(rpmsg, vrtc_mrst_rpmsg_id_table);

static struct rpmsg_driver vrtc_mrst_rpmsg = {
	.drv.name	= KBUILD_MODNAME,
	.drv.owner	= THIS_MODULE,
	.id_table	= vrtc_mrst_rpmsg_id_table,
	.probe		= vrtc_mrst_rpmsg_probe,
	.callback	= vrtc_mrst_rpmsg_cb,
	.remove		= vrtc_mrst_rpmsg_remove,
};

static int __init vrtc_mrst_rpmsg_init(void)
{
	return register_rpmsg_driver(&vrtc_mrst_rpmsg);
}

static void __exit vrtc_mrst_rpmsg_exit(void)
{
	return unregister_rpmsg_driver(&vrtc_mrst_rpmsg);
}

module_init(vrtc_mrst_rpmsg_init);
module_exit(vrtc_mrst_rpmsg_exit);

MODULE_AUTHOR("Jacob Pan; Feng Tang");
MODULE_DESCRIPTION("Driver for Moorestown virtual RTC");
MODULE_LICENSE("GPL");
