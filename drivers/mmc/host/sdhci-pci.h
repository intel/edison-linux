#ifndef __SDHCI_PCI_H
#define __SDHCI_PCI_H

/*
 * PCI device IDs
 */


/*
 * PCI registers
 */

#define PCI_SDHCI_IFPIO			0x00
#define PCI_SDHCI_IFDMA			0x01
#define PCI_SDHCI_IFVENDOR		0x02

#define PCI_SLOT_INFO			0x40	/* 8 bits */
#define  PCI_SLOT_INFO_SLOTS(x)		((x >> 4) & 7)
#define  PCI_SLOT_INFO_FIRST_BAR_MASK	0x07

#define MAX_SLOTS			8

#define IPC_EMMC_MUTEX_CMD             0xEE

/* CLV SD card power resource */

#define VCCSDIO_ADDR       0xd5
#define VCCSDIO_OFF        0x4
#define VCCSDIO_NORMAL     0x7
#define ENCTRL0_ISOLATE        0x55555557
#define ENCTRL1_ISOLATE        0x5555
#define STORAGESTIO_FLISNUM    0x8
#define ENCTRL0_OFF        0x10
#define ENCTRL1_OFF        0x11

struct sdhci_pci_chip;
struct sdhci_pci_slot;

struct sdhci_pci_fixes {
	unsigned int		quirks;
	unsigned int		quirks2;
	bool			allow_runtime_pm;
	bool			own_cd_for_runtime_pm;

	int			(*probe) (struct sdhci_pci_chip *);

	int			(*probe_slot) (struct sdhci_pci_slot *);
	void			(*remove_slot) (struct sdhci_pci_slot *, int);

	int			(*suspend) (struct sdhci_pci_chip *);
	int			(*resume) (struct sdhci_pci_chip *);
};

struct sdhci_pci_slot {
	struct sdhci_pci_chip	*chip;
	struct sdhci_host	*host;
	struct sdhci_pci_data	*data;

	int			pci_bar;
	int			rst_n_gpio;
	int			cd_gpio;
	int			cd_irq;

	char			*cd_con_id;
	int			cd_idx;
	bool			cd_override_level;

	void (*hw_reset)(struct sdhci_host *host);
	bool		dev_power;
	struct mutex	power_lock;
	bool		dma_enabled;
};

struct sdhci_pci_chip {
	struct pci_dev		*pdev;

	unsigned int		quirks;
	unsigned int		quirks2;
	bool			allow_runtime_pm;
	unsigned int		autosuspend_delay;
	const struct sdhci_pci_fixes *fixes;

	int			num_slots;	/* Slots on controller */
	struct sdhci_pci_slot	*slots[MAX_SLOTS]; /* Pointers to host slots */
	unsigned int		enctrl0_orig;
	unsigned int		enctrl1_orig;
};

#endif /* __SDHCI_PCI_H */
