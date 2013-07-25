#ifndef _ASM_X86_INTEL_SCU_FLIS_H_
#define _ASM_X86_INTEL_SCU_FLIS_H_

enum flis_param_t {
	PULL,
	MUX,
	OPEN_DRAIN,
};

/*
 * Config value for PULL
 */
#define NONE		(0 << 0)
#define DOWN_20K	(1 << 0)
#define DOWN_2K		(1 << 1)
/* DOWN_75K is reserved */
#define UP_20K		(1 << 3)
#define UP_2K		(1 << 4)
/*910 Weak Pull-Up*/
#define UP_910		(1 << 5)

/*
 * Config value for OPEN_DRAIN
 */
#define OD_DISABLE	(1 << 0)
#define OD_ENABLE	(0 << 0)

/*
 * Config value for MUX
 */
/*Bit0: Mux Enable for Input Enable*/
#define MUX_EN_INPUT_EN		(1<<0)
/*Bit1: Input Enable for*/
#define INPUT_EN		(1<<1)
/*Bit2: Mux Enable for Output Enable*/
#define MUX_EN_OUTPUT_EN	(1<<2)
/*Bit3: Output Enable*/
#define OUTPUT_EN		(1<<3)

#define PULL_MASK		0x3F
#define MUX_MASK		0xF
#define OPEN_DRAIN_MASK		0x1

enum pinname_t {
	i2s_2_clk = 0,
	i2s_2_fs = 1,
	i2s_2_rxd = 2,
	i2s_2_txd = 3,
	msic_reset_b = 4,
	spi_0_clk = 5,
	spi_0_sdi = 6,
	spi_0_sdo = 7,
	spi_0_ss = 8,
	svid_clkout = 9,
	svid_clksynch = 10,
	svid_din = 11,
	svid_dout = 12,
	usb_ulpi_clk = 13,
	usb_ulpi_data0 = 14,
	usb_ulpi_data1 = 15,
	usb_ulpi_data2 = 16,
	usb_ulpi_data3 = 17,
	usb_ulpi_data4 = 18,
	usb_ulpi_data5 = 19,
	usb_ulpi_data6 = 20,
	usb_ulpi_data7 = 21,
	usb_ulpi_dir = 22,
	usb_ulpi_nxt = 23,
	usb_ulpi_refclk = 24,
	usb_ulpi_stp = 25,
	ulpi1lpc_gpe_b = 26,
	ulpi1lpc_lpc_ad0 = 27,
	ulpi1lpc_lpc_ad1 = 28,
	ulpi1lpc_lpc_ad2 = 29,
	ulpi1lpc_lpc_ad3 = 30,
	ulpi1lpc_lpc_clkout = 31,
	ulpi1lpc_lpc_clkrun = 32,
	ulpi1lpc_lpc_frame_b = 33,
	ulpi1lpc_lpc_reset_b = 34,
	ulpi1lpc_lpc_serirq = 35,
	ulpi1lpc_lpc_smi_b = 36,
	ulpi1lpc_usb_ulpi_1_clk = 37,
	ulpi1lpc_usb_ulpi_1_data0 = 38,
	ulpi1lpc_usb_ulpi_1_data1 = 39,
	ulpi1lpc_usb_ulpi_1_data2 = 40,
	ulpi1lpc_usb_ulpi_1_data3 = 41,
	ulpi1lpc_usb_ulpi_1_data4 = 42,
	ulpi1lpc_usb_ulpi_1_data5 = 43,
	ulpi1lpc_usb_ulpi_1_data6 = 44,
	ulpi1lpc_usb_ulpi_1_data7 = 45,
	ulpi1lpc_usb_ulpi_1_dir = 46,
	ulpi1lpc_usb_ulpi_1_nxt = 47,
	ulpi1lpc_usb_ulpi_1_refclk = 48,
	ulpi1lpc_usb_ulpi_1_stp = 49,
	kbd_dkin0 = 50,
	kbd_dkin1 = 51,
	kbd_dkin2 = 52,
	kbd_dkin3 = 53,
	kbd_mkin0 = 54,
	kbd_mkin1 = 55,
	kbd_mkin2 = 56,
	kbd_mkin3 = 57,
	kbd_mkin4 = 58,
	kbd_mkin5 = 59,
	kbd_mkin6 = 60,
	kbd_mkin7 = 61,
	kbd_mkout0 = 62,
	kbd_mkout1 = 63,
	kbd_mkout2 = 64,
	kbd_mkout3 = 65,
	kbd_mkout4 = 66,
	kbd_mkout5 = 67,
	kbd_mkout6 = 68,
	kbd_mkout7 = 69,
	camerasb10 = 70,
	camerasb4 = 71,
	camerasb5 = 72,
	camerasb6 = 73,
	camerasb7 = 74,
	camerasb8 = 75,
	camerasb9 = 76,
	i2c_4_scl = 77,
	i2c_4_sda = 78,
	i2c_5_scl = 79,
	i2c_5_sda = 80,
	intd_dsi_te1 = 81,
	intd_dsi_te2 = 82,
	stio_0_cd_b = 83,
	stio_0_clk = 84,
	stio_0_cmd = 85,
	stio_0_dat0 = 86,
	stio_0_dat1 = 87,
	stio_0_dat2 = 88,
	stio_0_dat3 = 89,
	stio_0_dat4 = 90,
	stio_0_dat5 = 91,
	stio_0_dat6 = 92,
	stio_0_dat7 = 93,
	stio_0_wp_b = 94,
	camerasb0 = 95,
	camerasb1 = 96,
	camerasb2 = 97,
	camerasb3 = 98,
	ded_gpio10 = 99,
	ded_gpio11 = 100,
	ded_gpio12 = 101,
	ded_gpio13 = 102,
	ded_gpio14 = 103,
	ded_gpio15 = 104,
	ded_gpio16 = 105,
	ded_gpio17 = 106,
	ded_gpio18 = 107,
	ded_gpio19 = 108,
	ded_gpio20 = 109,
	ded_gpio21 = 110,
	ded_gpio22 = 111,
	ded_gpio23 = 112,
	ded_gpio24 = 113,
	ded_gpio25 = 114,
	ded_gpio26 = 115,
	ded_gpio27 = 116,
	ded_gpio28 = 117,
	ded_gpio29 = 118,
	ded_gpio30 = 119,
	ded_gpio8 = 120,
	ded_gpio9 = 121,
	mpti_nidnt_clk = 122,
	mpti_nidnt_data0 = 123,
	mpti_nidnt_data1 = 124,
	mpti_nidnt_data2 = 125,
	mpti_nidnt_data3 = 126,
	stio_1_clk = 127,
	stio_1_cmd = 128,
	stio_1_dat0 = 129,
	stio_1_dat1 = 130,
	stio_1_dat2 = 131,
	stio_1_dat3 = 132,
	stio_2_clk = 133,
	stio_2_cmd = 134,
	stio_2_dat0 = 135,
	stio_2_dat1 = 136,
	stio_2_dat2 = 137,
	stio_2_dat3 = 138,
	coms_int0 = 139,
	coms_int1 = 140,
	coms_int2 = 141,
	coms_int3 = 142,
	ded_gpio4 = 143,
	ded_gpio5 = 144,
	ded_gpio6 = 145,
	ded_gpio7 = 146,
	i2s_0_clk = 147,
	i2s_0_fs = 148,
	i2s_0_rxd = 149,
	i2s_0_txd = 150,
	i2s_1_clk = 151,
	i2s_1_fs = 152,
	i2s_1_rxd = 153,
	i2s_1_txd = 154,
	mslim_1_bclk = 155,
	mslim_1_bdat = 156,
	resetout_b = 157,
	spi_2_clk = 158,
	spi_2_sdi = 159,
	spi_2_sdo = 160,
	spi_2_ss0 = 161,
	spi_2_ss1 = 162,
	spi_3_clk = 163,
	spi_3_sdi = 164,
	spi_3_sdo = 165,
	spi_3_ss0 = 166,
	spi_3_ss1 = 167,
	uart_0_cts = 168,
	uart_0_rts = 169,
	uart_0_rx = 170,
	uart_0_tx = 171,
	uart_1_rx = 172,
	uart_1_sd = 173,
	uart_1_tx = 174,
	uart_2_rx = 175,
	uart_2_tx = 176,
	aclkph = 177,
	dclkph = 178,
	dsiclkph = 179,
	ierr = 180,
	jtag_tckc = 181,
	jtag_tdic = 182,
	jtag_tdoc = 183,
	jtag_tmsc = 184,
	jtag_trst_b = 185,
	lclkph = 186,
	lfhclkph = 187,
	osc_clk_ctrl0 = 188,
	osc_clk_ctrl1 = 189,
	osc_clk_out0 = 190,
	osc_clk_out1 = 191,
	osc_clk_out2 = 192,
	osc_clk_out3 = 193,
	prochot_b = 194,
	thermtrip_b = 195,
	uclkph = 196,
	ded_gpio31 = 197,
	ded_gpio32 = 198,
	ded_gpio33 = 199,
	hdmi_cec = 200,
	i2c_3_scl_hdmi_ddc = 201,
	i2c_3_sda_hdmi_ddc = 202,
	i2c_0_scl = 203,
	i2c_0_sda = 204,
	i2c_1_scl = 205,
	i2c_1_sda = 206,
	i2c_2_scl = 207,
	i2c_2_sda = 208,
	spi_1_clk = 209,
	spi_1_sdi = 210,
	spi_1_sdo = 211,
	spi_1_ss0 = 212,
	spi_1_ss1 = 213,
	spi_1_ss2 = 214,
	spi_1_ss3 = 215,
	spi_1_ss4 = 216,
};

struct pinstruct_t {
	bool valid;	/* the pin is allowed to be configured or not */
	u8 bus_address;
	u8 pullup_offset;
	u8 pullup_lsb_pos;
	u8 direction_offset;
	u8 direction_lsb_pos;
	u8 open_drain_offset;
	u8 open_drain_bit;
};

struct intel_scu_flis_platform_data {
	struct pinstruct_t *pin_t;
	int pin_num;
};

#define OPS_STR_LEN 10

enum {
	DBG_SHIM_FLIS_ADDR,
	DBG_SHIM_OFFSET,
	DBG_SHIM_DATA,

	DBG_PARAM_VAL,
	DBG_PARAM_TYPE,
	DBG_PIN_NAME,
};

int intel_scu_ipc_write_shim(u32 data, u32 flis_addr, u32 offset);
int intel_scu_ipc_read_shim(u32 *data, u32 flis_addr, u32 offset);
int intel_scu_ipc_update_shim(u32 data, u32 mask, u32 flis_addr, u32 offset);
int config_pin_flis(enum pinname_t name, enum flis_param_t param, u8 val);
int get_pin_flis(enum pinname_t name, enum flis_param_t param, u8 *val);

extern struct pinstruct_t ctp_pin_table[];
#endif
