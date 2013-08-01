#ifndef __INTEL_MID_HSU_H__
#define __INTEL_MID_HSU_H__

#define hsu_port_func_max 4

enum {
	hsu_port0,
	hsu_port1,
	hsu_port2,
	hsu_port_share,
	hsu_port_max,
	hsu_dma,
};

enum {
	bt_port,
	modem_port,
	gps_port,
	debug_port,
};

struct hsu_port_cfg {
	int type;
	int index;
	char *name;
	int idle;
	int has_alt;
	int alt;
	int force_suspend;
	int preamble;
	int hw_context_save;
	int hw_ctrl_cts;
	struct device *dev;
	int (*hw_init)(struct device *dev, int port);
	void(*hw_set_alt)(int port);
	void(*hw_set_rts)(int port, int value);
	void(*hw_suspend)(int port, struct device *dev, irq_handler_t wake_isr);
	void(*hw_suspend_post)(int port);
	void(*hw_resume)(int port, struct device *dev);
	unsigned int (*hw_get_clk)(void);
	void (*wake_peer)(struct device *tty);
};


void intel_mid_hsu_suspend(int port, struct device *dev,
				irq_handler_t wake_isr);
void intel_mid_hsu_resume(int port, struct device *dev);
void intel_mid_hsu_rts(int port, int value);
void intel_mid_hsu_switch(int port);
int intel_mid_hsu_init(struct device *dev, int port);
int intel_mid_hsu_func_to_port(unsigned int func);
unsigned int intel_mid_hsu_get_clk(void);
int hsu_register_board_info(void *inf);
void intel_mid_hsu_suspend_post(int port);
struct device *intel_mid_hsu_set_wake_peer(int port,
	void (*wake_peer)(struct device *));
#endif
