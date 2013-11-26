/*
 * Intel Penwell USB OTG transceiver driver
 * Copyright (C) 2009 - 2010, Intel Corporation.
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

#ifndef __PENWELL_OTG_H__
#define __PENWELL_OTG_H__

#include <linux/usb/intel_mid_otg.h>
#include <linux/power_supply.h>
#include <linux/wakelock.h>

#define PMU_OTG_WAKE_SOURCE     6
#define CI_USBCMD		0x30
#	define USBCMD_RST		BIT(1)
#	define USBCMD_RS		BIT(0)
#define CI_USBSTS		0x34
#	define USBSTS_SLI		BIT(8)
#	define USBSTS_URI		BIT(6)
#	define USBSTS_PCI		BIT(2)
#define CI_USBINTR		0x38
#	define USBINTR_PCE		BIT(2)
#define CI_ULPIVP		0x60
#	define ULPI_WU			BIT(31)
#	define ULPI_RUN			BIT(30)
#	define ULPI_RW			BIT(29)
#	define ULPI_SS			BIT(27)
#	define ULPI_PORT		(BIT(26) | BIT(25) | BIT(24))
#	define ULPI_ADDR		(0xff << 16)
#	define ULPI_DATRD		(0xff << 8)
#	define ULPI_DATWR		(0xff << 0)
#define CI_PORTSC1		0x74
#	define PORTSC_PP		BIT(12)
#	define PORTSC_LS		(BIT(11) | BIT(10))
#	define PORTSC_SUSP		BIT(7)
#	define PORTSC_CCS		BIT(0)
#define CI_HOSTPC1		0xb4
#	define HOSTPC1_PHCD		BIT(22)
#define CI_OTGSC		0xf4
#	define OTGSC_DPIE		BIT(30)
#	define OTGSC_1MSE		BIT(29)
#	define OTGSC_BSEIE		BIT(28)
#	define OTGSC_BSVIE		BIT(27)
#	define OTGSC_ASVIE		BIT(26)
#	define OTGSC_AVVIE		BIT(25)
#	define OTGSC_IDIE		BIT(24)
#	define OTGSC_DPIS		BIT(22)
#	define OTGSC_1MSS		BIT(21)
#	define OTGSC_BSEIS		BIT(20)
#	define OTGSC_BSVIS		BIT(19)
#	define OTGSC_ASVIS		BIT(18)
#	define OTGSC_AVVIS		BIT(17)
#	define OTGSC_IDIS		BIT(16)
#	define OTGSC_DPS		BIT(14)
#	define OTGSC_1MST		BIT(13)
#	define OTGSC_BSE		BIT(12)
#	define OTGSC_BSV		BIT(11)
#	define OTGSC_ASV		BIT(10)
#	define OTGSC_AVV		BIT(9)
#	define OTGSC_ID			BIT(8)
#	define OTGSC_HABA		BIT(7)
#	define OTGSC_HADP		BIT(6)
#	define OTGSC_IDPU		BIT(5)
#	define OTGSC_DP			BIT(4)
#	define OTGSC_OT			BIT(3)
#	define OTGSC_HAAR		BIT(2)
#	define OTGSC_VC			BIT(1)
#	define OTGSC_VD			BIT(0)
#define CI_USBMODE		0xf8
#	define USBMODE_CM		(BIT(1) | BIT(0))
#	define USBMODE_IDLE		0
#	define USBMODE_DEVICE		0x2
#	define USBMODE_HOST		0x3
#define USBCFG_ADDR			0xff10801c
#define USBCFG_LEN			4
#	define USBCFG_VBUSVAL		BIT(14)
#	define USBCFG_AVALID		BIT(13)
#	define USBCFG_BVALID		BIT(12)
#	define USBCFG_SESEND		BIT(11)

#define OTGSC_INTEN_MASK \
	(OTGSC_DPIE | OTGSC_BSEIE | OTGSC_BSVIE \
	| OTGSC_ASVIE | OTGSC_AVVIE | OTGSC_IDIE)

#define OTGSC_INTSTS_MASK \
	(OTGSC_DPIS | OTGSC_BSEIS | OTGSC_BSVIS \
	| OTGSC_ASVIS | OTGSC_AVVIS | OTGSC_IDIS)

#define INTR_DUMMY_MASK (USBSTS_SLI | USBSTS_URI | USBSTS_PCI)

#define HOST_REQUEST_FLAG		BIT(0)

/* MSIC register for vbus power control */
#define MSIC_ID			0x00
#	define ID0_VENDID0		(BIT(7) | BIT(6))
#define MSIC_ID1		0x01
#	define ID1_VENDID1		(BIT(7) | BIT(6))
#define MSIC_VUSB330CNT		0xd4
#define MSIC_VOTGCNT		0xdf
#	define VOTGEN			BIT(7)
#	define VOTGRAMP			BIT(4)
#define MSIC_SPWRSRINT1		0x193
#	define SUSBCHPDET		BIT(6)
#	define SUSBDCDET		BIT(2)
#	define MSIC_SPWRSRINT1_MASK	(BIT(6) | BIT(2))
#	define SPWRSRINT1_CDP		BIT(6)
#	define SPWRSRINT1_SDP		0
#	define SPWRSRINT1_DCP		BIT(2)
#define MSIC_USB_MISC		0x2c8	/* Intel Specific */
#	define MISC_CHGDSERXDPINV	BIT(5)
#define MSIC_OTGCTRL		0x39c
#define MSIC_OTGCTRLSET		0x340
#define MSIC_OTGCTRLCLR		0x341
#define	ULPI_OTGCTRL		0x0a
#define	ULPI_OTGCTRLSET		0x0b
#define	ULPI_OTGCTRLCLR		0x0c
#	define DRVVBUS_EXTERNAL		BIT(6)
#	define DRVVBUS			BIT(5)
#	define DMPULLDOWN		BIT(2)
#	define DPPULLDOWN		BIT(1)
#define MSIC_USBINTEN_RISE	0x39d
#define MSIC_USBINTEN_RISESET	0x39e
#define MSIC_USBINTEN_RISECLR	0x39f
#define MSIC_USBINTEN_FALL	0x3a0
#define MSIC_USBINTEN_FALLSET	0x3a1
#define MSIC_USBINTEN_FALLCLR	0x3a2

/*
 * For Clovertrail, due to change of USB PHY from MSIC to external standalone
 * chip, USB Interrupt Enable Rising/Falling registers can be accessed only
 * from ULPI interface.
 */
#define ULPI_USBINTEN_RISING		0xd
#define ULPI_USBINTEN_RISINGSET		0xe
#define ULPI_USBINTEN_RISINGCLR		0xf
#define ULPI_USBINTEN_FALLING		0x10
#define ULPI_USBINTEN_FALLINGSET	0x11
#define ULPI_USBINTEN_FALLINGCLR	0x12

#	define IDGND			BIT(4)
#	define SESSEND			BIT(3)
#	define SESSVLD			BIT(2)
#	define VBUSVLD			BIT(1)
#	define HOSTDISCON		BIT(0)
#define MSIC_PWRCTRL		0x3b5
#define MSIC_PWRCTRLSET		0x342
#define MSIC_PWRCTRLCLR		0x343
#define ULPI_PWRCTRL		0x3d
#define ULPI_PWRCTRLSET		0x3e
#define ULPI_PWRCTRLCLR		0x3f
#	define HWDET			BIT(7)
#	define DPVSRCEN			BIT(6)
#	define VDATDET			BIT(5)
#	define DPWKPUEN			BIT(4)
#	define SWCNTRL			BIT(0)
#define MSIC_FUNCTRL		0x398
#define MSIC_FUNCTRLSET		0x344
#define MSIC_FUNCTRLCLR		0x345
#define ULPI_FUNCTRL		0x04
#define ULPI_FUNCTRLSET		0x05
#define ULPI_FUNCTRLCLR		0x06
#	define PHYRESET			BIT(5)
#	define OPMODE1			BIT(4)
#	define OPMODE0			BIT(3)
#	define TERMSELECT		BIT(2)
#	define XCVRSELECT1		BIT(1)
#	define XCVRSELECT0		BIT(0)
#define MSIC_DEBUG		0x3a5
#define ULPI_DEBUG		0x15
#	define LINESTATE_MSK		(BIT(0) | BIT(1))
#	define LINESTATE_SE1		(BIT(0) | BIT(1))
#	define LINESTATE_SE0		(0)
#	define LINESTATE_FSJ		BIT(0)
#	define LINESTATE_FSK		BIT(1)
#define MSIC_VS1		0x3b6
#define MSIC_VS1SET		0x3a9
#define MSIC_VS1CLR		0x3aa
#define ULPI_VS1		0x80
#define ULPI_VS1SET		0x81
#define ULPI_VS1CLR		0x82
#	define DATAPOLARITY		BIT(6)
#define ULPI_VS2STS		0x83
#define ULPI_VS2LATCH		0x84
#	define VBUS_MNTR_STS		BIT(7)
#	define REG3V3_MNTR_STS		BIT(6)
#	define SVLDCONWKB_WDOG_STS	BIT(5)
#	define IDFLOAT_STS		BIT(4)
#	define IDRARBRC_STS(d)		(((d)>>2)&3)
#	define IDRARBRC_STS1		BIT(3)
#	define IDRARBRC_STS2		BIT(2)
#	define IDRARBRC_MSK		(BIT(2) | BIT(3))
#	define IDRARBRC_A		1
#	define IDRARBRC_B		2
#	define IDRARBRC_C		3
#	define BVALID_STS		BIT(0)
#define MSIC_VS3		0x3b9
#define MSIC_VS3SET		0x346	/* Vendor Specific */
#define MSIC_VS3CLR		0x347
#	define SWUSBDET			BIT(4)
#	define DATACONEN		BIT(3)
#define ULPI_VS3		0x85
#define ULPI_VS3SET		0x86
#define ULPI_VS3CLR		0x87
#	define CHGD_IDP_SRC		BIT(6)
#	define IDPULLUP_WK		BIT(5)
#	define SWUSBDET			BIT(4)
#	define DATACONEN		BIT(3)
#define MSIC_VS4		0x3ba
#define MSIC_VS4SET		0x3ab
#define MSIC_VS4CLR		0x3ac
#define ULPI_VS4		0x88
#define ULPI_VS4SET		0x89
#define ULPI_VS4CLR		0x8a
#	define ACADET			BIT(6)
#	define RABUSIN			BIT(5)
#	define R1KERIES			BIT(4)
#	define CHRG_SERX_DP		BIT(1)
#	define CHRG_SERX_DM		BIT(0)
#define ULPI_VS5		0x8b
#define ULPI_VS5SET		0x8c
#define ULPI_VS5CLR		0x8d
#	define AUTORESUME_WDOG		BIT(6)
#	define IDFLOAT_EN		BIT(5)
#	define IDRES_EN			BIT(4)
#	define SVLDCONWKB_WDOG		BIT(3)
#	define VBUS_MNTR_RISEEN		BIT(2)
#	define VBUS_MNTR_FALLEN		BIT(1)
#	define REG3V3IN_MNTR_EN		BIT(0)
#define ULPI_VS6		0x8e
#define ULPI_VS6SET		0x8f
#define ULPI_VS6CLR		0x90
#	define ACA_RID_B_CFG		BIT(7)
#	define ACA_RID_A_CFG		BIT(6)
#	define SOF_EN			BIT(5)
#define MSIC_ULPIACCESSMODE	0x348
#	define SPIMODE			BIT(0)
#define MSIC_INT_EN_RISE	0x39D
#define MSIC_INT_EN_RISE_SET	0x39E
#define MSIC_INT_EN_RISE_CLR	0x39F
#define MSIC_INT_EN_FALL	0x3A0
#define MSIC_INT_EN_FALL_SET	0x3A1
#define MSIC_INT_EN_FALL_CLR	0x3A2

/* MSIC TI implementation for ADP/ACA */
#define SPI_TI_VS2		0x3B7
#define SPI_TI_VS2_LATCH	0x3B8
#define SPI_TI_VS4		0x3BA
#define SPI_TI_VS5		0x3BB
#define ULPI_TI_USB_INT_STS	0x13
#define ULPI_TI_USB_INT_LAT	0x14
#	define USB_INT_IDGND		BIT(4)
#	define USB_INT_SESSEND		BIT(3)
#	define USB_INT_SESSVLD		BIT(2)
#	define USB_INT_VBUSVLD		BIT(1)
#define ULPI_TI_VS2		0x83
#	define TI_ID_FLOAT_STS		BIT(4)
#	define TI_ID_RARBRC_STS(d)	(((d)>>2)&3)
#	define TI_ID_RARBRC_STS_MASK	(BIT(3) | BIT(2))
#	define TI_ID_RARBRC_NONE	0
#	define TI_ID_RARBRC_A		1
#	define TI_ID_RARBRC_B		2
#	define TI_ID_RARBRC_C		3
#	define TI_ADP_INT_STS		BIT(1)
#define ULPI_TI_VS4		0x88
#	define TI_ACA_DET_EN		BIT(6)
#define ULPI_TI_VS5		0x8b
#	define TI_ADP_INT_EN		BIT(7)
#	define TI_ID_FLOAT_EN		BIT(5)
#	define TI_ID_RES_EN		BIT(4)
#define ULPI_TI_VS6		0x8e
#	define TI_HS_TXPREN		BIT(4)
#	define TI_ADP_MODE(d)		(((d)>>2)&3)
#	define TI_ADP_MODE_MASK		(BIT(3) | BIT(2))
#	define TI_ADP_MODE_DISABLE	0
#	define TI_ADP_MODE_SENSE	1
#	define TI_ADP_MODE_PRB_A	2
#	define TI_ADP_MODE_PRB_B	3
#	define TI_VBUS_IADP_SRC		BIT(1)
#	define TI_VBUS_IADP_SINK	BIT(0)
#define ULPI_TI_VS7		0x91
#	define TI_T_ADP_HIGH		(0xff)
#define ULPI_TI_VS8		0x94
#	define TI_T_ADP_LOW		(0xff)
#define ULPI_TI_VS9		0x97
#	define TI_T_ADP_RISE		(0xff)

#define TI_PRB_DELTA			0x08

/* MSIC FreeScale Implementation for ADP */
#define ULPI_FS_ADPCL		0x28
#	define ADPCL_PRBDSCHG		(BIT(5) | BIT(6))
#	define ADPCL_PRBDSCHG_4		0
#	define ADPCL_PRBDSCHG_8		1
#	define ADPCL_PRBDSCHG_16	2
#	define ADPCL_PRBDSCHG_32	3
#	define ADPCL_PRBPRD		(BIT(3) | BIT(4))
#	define ADPCL_PRBPRD_A_HALF	0
#	define ADPCL_PRBPRD_B_HALF	1
#	define ADPCL_PRBPRD_A		2
#	define ADPCL_PRBPRD_B		3
#	define ADPCL_SNSEN		BIT(2)
#	define ADPCL_PRBEN		BIT(1)
#	define ADPCL_ADPEN		BIT(0)
#define ULPI_FS_ADPCH		0x29
#	define ADPCH_PRBDELTA		(0x1f << 0)
#define ULPI_FS_ADPIE		0x2a
#	define ADPIE_ADPRAMPIE		BIT(2)
#	define ADPIE_SNSMISSIE		BIT(1)
#	define ADPIE_PRBTRGIE		BIT(0)
#define ULPI_FS_ADPIS		0x2b
#	define ADPIS_ADPRAMPS		BIT(5)
#	define ADPIS_SNSMISSS		BIT(4)
#	define ADPIS_PRBTRGS		BIT(3)
#	define ADPIS_ADPRAMPI		BIT(2)
#	define ADPIS_SNSMISSI		BIT(1)
#	define ADPIS_PRBTRGI		BIT(0)
#define ULPI_FS_ADPRL		0x2c
#	define ADPRL_ADPRAMP		(0xff << 0)
#define ULPI_FS_ADPRH		0x2d
#	define ADPRH_ADPRAMP		(0x7 << 0)

#define FS_ADPI_MASK	(ADPIS_ADPRAMPI | ADPIS_SNSMISSI | ADPIS_PRBTRGI)

/* define Data connect checking timeout and polling interval */
#define DATACON_TIMEOUT		750
#define DATACON_INTERVAL	20

enum penwell_otg_timer_type {
	TA_WAIT_VRISE_TMR,
	TA_WAIT_BCON_TMR,
	TA_AIDL_BDIS_TMR,
	TA_BIDL_ADIS_TMR,
	TA_WAIT_VFALL_TMR,
	TB_ASE0_BRST_TMR,
	TB_SE0_SRP_TMR,
	TB_SRP_FAIL_TMR, /* wait for response of SRP */
	TB_BUS_SUSPEND_TMR,
	TTST_MAINT_TMR,
	TTST_NOADP_TMR,
};

#define TA_WAIT_VRISE		100
#define TA_WAIT_BCON		50000
#define TA_AIDL_BDIS		1500
#define TA_BIDL_ADIS		300
#define TA_WAIT_VFALL		950
#define TB_ASE0_BRST		300
#define TB_SE0_SRP		1200
#define TB_SSEND_SRP		1800
#	define SRP_MON_INVAL	300	/* TODO: interval needs more tuning */
#define TB_SRP_FAIL		5500
#define TB_BUS_SUSPEND		500
#define THOS_REQ_POL		1500
/* Test mode */
#define	TTST_MAINT		9900
#define	TTST_NOADP		5000

/* MSIC vendor information */
enum msic_vendor {
	MSIC_VD_FS,
	MSIC_VD_TI,
	MSIC_VD_UNKNOWN
};

/* charger defined in BC 1.2 */
enum usb_charger_type {
	CHRG_UNKNOWN,
	CHRG_SDP,	/* Standard Downstream Port */
	CHRG_CDP,	/* Charging Downstream Port */
	CHRG_SDP_INVAL,	/* Invaild Standard Downstream Port */
	CHRG_DCP,	/* Dedicated Charging Port */
	CHRG_ACA,	/* Accessory Charger Adapter */
	CHRG_ACA_DOCK,	/* Accessory Charger Adapter - Dock */
	CHRG_ACA_A,	/* Accessory Charger Adapter - RID_A */
	CHRG_ACA_B,	/* Accessory Charger Adapter - RID_B */
	CHRG_ACA_C,	/* Accessory Charger Adapter - RID_C */
	CHRG_SE1,	/* SE1 (Apple)*/
	CHRG_MHL	/* Moblie High-Definition Link */
};

struct adp_status {
	struct completion	adp_comp;
	u8			t_adp_rise;
};

/* Invalid SDP checking timeout */
#define INVALID_SDP_TIMEOUT	(HZ * 15)

/* OTG Battery Charging capability is used in charger capability detection */
struct otg_bc_cap {
	enum usb_charger_type	chrg_type;
	unsigned int		ma;
#define CHRG_CURR_UNKNOWN	0
#define CHRG_CURR_DISCONN	0
#define CHRG_CURR_SDP_SUSP	2
#define CHRG_CURR_SDP_UNCONFIG	100
#define CHRG_CURR_SDP_LOW	100
#define CHRG_CURR_SDP_HIGH	500
#define CHRG_CURR_SDP_INVAL	500
#define CHRG_CURR_CDP		1500
#define CHRG_CURR_DCP	1500
#define CHRG_CURR_SE1	1500
#define CHRG_CURR_ACA	1500
	unsigned int            current_event;
};

struct otg_bc_event {
	struct list_head		node;
	struct power_supply_cable_props	cap;
};

/* Bus monitor action for b_ssend_srp/b_se0_srp */
#define BUS_MON_STOP		0
#define BUS_MON_START		1
#define BUS_MON_CONTINUE	2

/* define event ids to notify battery driver */
#define USBCHRG_EVENT_CONNECT	1
#define USBCHRG_EVENT_DISCONN	2
#define USBCHRG_EVENT_SUSPEND	3
#define USBCHRG_EVENT_RESUME	4
#define USBCHRG_EVENT_UPDATE	5

struct intel_mid_otg_pdata {
	int gpio_vbus;
	int gpio_cs;
	int gpio_reset;
	int charging_compliance;
	int hnp_poll_support;
	unsigned power_budget;
};

struct penwell_otg {
	struct intel_mid_otg_xceiv	iotg;
	struct device			*dev;

	unsigned			region;
	unsigned			cfg_region;

	struct work_struct		work;
	struct work_struct		hnp_poll_work;
	struct work_struct		psc_notify;
	struct work_struct		uevent_work;
	struct delayed_work		ulpi_poll_work;
	struct delayed_work		ulpi_check_work;
	struct delayed_work		sdp_check_work;
	struct workqueue_struct		*qwork;
	struct workqueue_struct		*chrg_qwork;


	struct timer_list		hsm_timer;
	struct timer_list		hnp_poll_timer;
	struct timer_list		bus_mon_timer;

	unsigned long			b_se0_srp_time;
	unsigned long			b_ssend_srp_time;

	struct mutex			msic_mutex;
	enum msic_vendor		msic;

	struct notifier_block		iotg_notifier;
	int				queue_stop;

	struct adp_status		adp;

	spinlock_t			charger_lock;
	struct list_head		chrg_evt_queue;
	struct otg_bc_cap		charging_cap;
	spinlock_t			cap_lock;
	struct power_supply_cable_props psc_cap;
	int (*bc_callback)(void *arg, int event, struct otg_bc_cap *cap);
	void				*bc_arg;

	unsigned			rt_resuming;

	unsigned			rt_quiesce;
	struct intel_mid_otg_pdata	*otg_pdata;

	struct wake_lock		wake_lock;
	spinlock_t			lock;

	int				phy_power_state;
};

static inline
struct penwell_otg *iotg_to_penwell(struct intel_mid_otg_xceiv *iotg)
{
	return container_of(iotg, struct penwell_otg, iotg);
}

extern int penwell_otg_query_charging_cap(struct otg_bc_cap *cap);
extern int penwell_otg_query_power_supply_cap(
			struct power_supply_cable_props *cap);
extern void *penwell_otg_register_bc_callback(
	int (*cb)(void *, int, struct otg_bc_cap *), void *arg);
extern int penwell_otg_unregister_bc_callback(void *handler);

extern int pnw_otg_ulpi_write(u8 reg, u8 val);
extern int is_clovertrail(struct pci_dev *pdev);

#endif /* __PENWELL_OTG_H__ */
