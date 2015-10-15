#ifndef _ASM_X86_INTEL_SCU_FLIS_H_
#define _ASM_X86_INTEL_SCU_FLIS_H_

enum flis_param_t {
	PULL,
	MUX,
	OPEN_DRAIN,
};

/* For MERR */

#define PULL_MASK	((7 << 4) | (3 << 8))
#define MUX_MASK	(0xF << 12)
#define OPEN_DRAIN_MASK	BIT(21)

#define PULL_UP		(1 << 8)
#define PULL_DOWN	(2 << 8)
#define R2Kohms		(0 << 4)
#define R20Kohms	(1 << 4)
#define R50Kohms	(2 << 4)
#define R910ohms	(3 << 4)

#define UP_2K		(PULL_UP | R2Kohms)
#define UP_20K		(PULL_UP | R20Kohms)
#define UP_50K		(PULL_UP | R50Kohms)
#define UP_910		(PULL_UP | R910ohms)
#define DOWN_2K		(PULL_DOWN | R2Kohms)
#define DOWN_20K	(PULL_DOWN | R20Kohms)
#define DOWN_50K	(PULL_DOWN | R50Kohms)
#define DOWN_910	(PULL_DOWN | R910ohms)

#define OD_DISABLE	(0 << 21)
#define OD_ENABLE	(1 << 21)

#define MUX_EN_INPUT_EN		(2 << 12)
#define INPUT_EN		(1 << 12)
#define MUX_EN_OUTPUT_EN	(8 << 12)
#define OUTPUT_EN		(4 << 12)

/* Add prefix "tng_" to avoid name duplication with ctp pins */
enum tng_pinname_t {
	tng_usb_ulpi_0_clk = 0,
	tng_usb_ulpi_0_data_0 = 1,
	tng_usb_ulpi_0_data_1 = 2,
	tng_usb_ulpi_0_data_2 = 3,
	tng_usb_ulpi_0_data_3 = 4,
	tng_usb_ulpi_0_data_4 = 5,
	tng_usb_ulpi_0_data_5 = 6,
	tng_usb_ulpi_0_data_6 = 7,
	tng_usb_ulpi_0_data_7 = 8,
	tng_usb_ulpi_0_dir = 9,
	tng_usb_ulpi_0_nxt = 10,
	tng_usb_ulpi_0_refclk = 11,
	tng_usb_ulpi_0_stp = 12,
	tng_emmc_0_clk = 13,
	tng_emmc_0_cmd = 14,
	tng_emmc_0_d_0 = 15,
	tng_emmc_0_d_1 = 16,
	tng_emmc_0_d_2 = 17,
	tng_emmc_0_d_3 = 18,
	tng_emmc_0_d_4 = 19,
	tng_emmc_0_d_5 = 20,
	tng_emmc_0_d_6 = 21,
	tng_emmc_0_d_7 = 22,
	tng_emmc_0_rst_b = 23,
	tng_gp_emmc_1_clk = 24,
	tng_gp_emmc_1_cmd = 25,
	tng_gp_emmc_1_d_0 = 26,
	tng_gp_emmc_1_d_1 = 27,
	tng_gp_emmc_1_d_2 = 28,
	tng_gp_emmc_1_d_3 = 29,
	tng_gp_emmc_1_d_4 = 30,
	tng_gp_emmc_1_d_5 = 31,
	tng_gp_emmc_1_d_6 = 32,
	tng_gp_emmc_1_d_7 = 33,
	tng_gp_emmc_1_rst_b = 34,
	tng_gp_28 = 35,
	tng_gp_29 = 36,
	tng_gp_sdio_0_cd_b = 37,
	tng_gp_sdio_0_clk = 38,
	tng_gp_sdio_0_cmd = 39,
	tng_gp_sdio_0_dat_0 = 40,
	tng_gp_sdio_0_dat_1 = 41,
	tng_gp_sdio_0_dat_2 = 42,
	tng_gp_sdio_0_dat_3 = 43,
	tng_gp_sdio_0_lvl_clk_fb = 44,
	tng_gp_sdio_0_lvl_cmd_dir = 45,
	tng_gp_sdio_0_lvl_dat_dir = 46,
	tng_gp_sdio_0_lvl_sel = 47,
	tng_gp_sdio_0_powerdown_b = 48,
	tng_gp_sdio_0_wp = 49,
	tng_gp_sdio_1_clk = 50,
	tng_gp_sdio_1_cmd = 51,
	tng_gp_sdio_1_dat_0 = 52,
	tng_gp_sdio_1_dat_1 = 53,
	tng_gp_sdio_1_dat_2 = 54,
	tng_gp_sdio_1_dat_3 = 55,
	tng_gp_sdio_1_powerdown_b = 56,
	tng_mhsi_acdata = 57,
	tng_mhsi_acflag = 58,
	tng_mhsi_acready = 59,
	tng_mhsi_acwake = 60,
	tng_mhsi_cadata = 61,
	tng_mhsi_caflag = 62,
	tng_mhsi_caready = 63,
	tng_mhsi_cawake = 64,
	tng_gp_mslim_0_bclk = 65,
	tng_gp_mslim_0_bdat = 66,
	tng_gp_ssp_0_clk = 67,
	tng_gp_ssp_0_fs = 68,
	tng_gp_ssp_0_rxd = 69,
	tng_gp_ssp_0_txd = 70,
	tng_gp_ssp_1_clk = 71,
	tng_gp_ssp_1_fs = 72,
	tng_gp_ssp_1_rxd = 73,
	tng_gp_ssp_1_txd = 74,
	tng_gp_ssp_2_clk = 75,
	tng_gp_ssp_2_fs = 76,
	tng_gp_ssp_2_rxd = 77,
	tng_gp_ssp_2_txd = 78,
	tng_gp_ssp_3_clk = 79,
	tng_gp_ssp_3_fs = 80,
	tng_gp_ssp_3_rxd = 81,
	tng_gp_ssp_3_txd = 82,
	tng_gp_ssp_4_clk = 83,
	tng_gp_ssp_4_fs_0 = 84,
	tng_gp_ssp_4_fs_1 = 85,
	tng_gp_ssp_4_fs_2 = 86,
	tng_gp_ssp_4_fs_3 = 87,
	tng_gp_ssp_4_rxd = 88,
	tng_gp_ssp_4_txd = 89,
	tng_gp_ssp_5_clk = 90,
	tng_gp_ssp_5_fs_0 = 91,
	tng_gp_ssp_5_fs_1 = 92,
	tng_gp_ssp_5_fs_2 = 93,
	tng_gp_ssp_5_fs_3 = 94,
	tng_gp_ssp_5_rxd = 95,
	tng_gp_ssp_5_txd = 96,
	tng_gp_ssp_6_clk = 97,
	tng_gp_ssp_6_fs = 98,
	tng_gp_ssp_6_rxd = 99,
	tng_gp_ssp_6_txd = 100,
	tng_gp_i2c_1_scl = 101,
	tng_gp_i2c_1_sda = 102,
	tng_gp_i2c_2_scl = 103,
	tng_gp_i2c_2_sda = 104,
	tng_gp_i2c_3_scl = 105,
	tng_gp_i2c_3_sda = 106,
	tng_gp_i2c_4_scl = 107,
	tng_gp_i2c_4_sda = 108,
	tng_gp_i2c_5_scl = 109,
	tng_gp_i2c_5_sda = 110,
	tng_gp_i2c_6_scl = 111,
	tng_gp_i2c_6_sda = 112,
	tng_gp_i2c_7_scl = 113,
	tng_gp_i2c_7_sda = 114,
	tng_gp_uart_0_cts = 115,
	tng_gp_uart_0_rts = 116,
	tng_gp_uart_0_rx = 117,
	tng_gp_uart_0_tx = 118,
	tng_gp_uart_1_cts = 119,
	tng_gp_uart_1_rts = 120,
	tng_gp_uart_1_rx = 121,
	tng_gp_uart_1_tx = 122,
	tng_gp_uart_2_cts = 123,
	tng_gp_uart_2_rts = 124,
	tng_gp_uart_2_rx = 125,
	tng_gp_uart_2_tx = 126,
	tng_gp_13 = 127,
	tng_gp_14 = 128,
	tng_gp_15 = 129,
	tng_gp_16 = 130,
	tng_gp_17 = 131,
	tng_gp_18 = 132,
	tng_gp_19 = 133,
	tng_gp_20 = 134,
	tng_gp_21 = 135,
	tng_gp_22 = 136,
	tng_gp_23 = 137,
	tng_gp_24 = 138,
	tng_gp_25 = 139,
	tng_gp_fast_int_0 = 140,
	tng_gp_fast_int_1 = 141,
	tng_gp_fast_int_2 = 142,
	tng_gp_fast_int_3 = 143,
	tng_gp_pwm_0 = 144,
	tng_gp_pwm_1 = 145,
	tng_gp_camerasb_0 = 146,
	tng_gp_camerasb_1 = 147,
	tng_gp_camerasb_2 = 148,
	tng_gp_camerasb_3 = 149,
	tng_gp_camerasb_4 = 150,
	tng_gp_camerasb_5 = 151,
	tng_gp_camerasb_6 = 152,
	tng_gp_camerasb_7 = 153,
	tng_gp_camerasb_8 = 154,
	tng_gp_camerasb_9 = 155,
	tng_gp_camerasb_10 = 156,
	tng_gp_camerasb_11 = 157,
	tng_gp_clkph_0 = 158,
	tng_gp_clkph_1 = 159,
	tng_gp_clkph_2 = 160,
	tng_gp_clkph_3 = 161,
	tng_gp_clkph_4 = 162,
	tng_gp_clkph_5 = 163,
	tng_gp_hdmi_hpd = 164,
	tng_gp_intd_dsi_te1 = 165,
	tng_gp_intd_dsi_te2 = 166,
	tng_osc_clk_ctrl_0 = 167,
	tng_osc_clk_ctrl_1 = 168,
	tng_osc_clk_out_0 = 169,
	tng_osc_clk_out_1 = 170,
	tng_osc_clk_out_2 = 171,
	tng_osc_clk_out_3 = 172,
	tng_osc_clk_out_4 = 173,
	tng_resetout_b = 174,
	tng_xxpmode = 175,
	tng_xxprdy = 176,
	tng_xxpreq_b = 177,
	tng_gp_26 = 178,
	tng_gp_27 = 179,
	tng_i2c_0_scl = 180,
	tng_i2c_0_sda = 181,
	tng_ierr_b = 182,
	tng_jtag_tckc = 183,
	tng_jtag_tdic = 184,
	tng_jtag_tdoc = 185,
	tng_jtag_tmsc = 186,
	tng_jtag_trst_b = 187,
	tng_prochot_b = 188,
	tng_rtc_clk = 189,
	tng_svid_vclk = 190,
	tng_svid_vdio = 191,
	tng_thermtrip_b = 192,
	tng_standby = 193,
	tng_gp_kbd_dkin_0 = 194,
	tng_gp_kbd_dkin_1 = 195,
	tng_gp_kbd_dkin_2 = 196,
	tng_gp_kbd_dkin_3 = 197,
	tng_gp_kbd_mkin_0 = 198,
	tng_gp_kbd_mkin_1 = 199,
	tng_gp_kbd_mkin_2 = 200,
	tng_gp_kbd_mkin_3 = 201,
	tng_gp_kbd_mkin_4 = 202,
	tng_gp_kbd_mkin_5 = 203,
	tng_gp_kbd_mkin_6 = 204,
	tng_gp_kbd_mkin_7 = 205,
	tng_gp_kbd_mkout_0 = 206,
	tng_gp_kbd_mkout_1 = 207,
	tng_gp_kbd_mkout_2 = 208,
	tng_gp_kbd_mkout_3 = 209,
	tng_gp_kbd_mkout_4 = 210,
	tng_gp_kbd_mkout_5 = 211,
	tng_gp_kbd_mkout_6 = 212,
	tng_gp_kbd_mkout_7 = 213,
	tng_gp_0 = 214,
	tng_gp_1 = 215,
	tng_gp_2 = 216,
	tng_gp_3 = 217,
	tng_gp_4 = 218,
	tng_gp_5 = 219,
	tng_gp_6 = 220,
	tng_gp_7 = 221,
	tng_gp_8 = 222,
	tng_gp_9 = 223,
	tng_gp_10 = 224,
	tng_gp_11 = 225,
	tng_gp_12 = 226,
	tng_gp_mpti_clk = 227,
	tng_gp_mpti_data_0 = 228,
	tng_gp_mpti_data_1 = 229,
	tng_gp_mpti_data_2 = 230,
	tng_gp_mpti_data_3 = 231,
	TNG_PIN_NUM,
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

enum ACCESS_CTRL {
	readonly = (1 << 0),
	writable = (1 << 1),
};

struct pin_mmio_flis_t {
	u8 access_ctrl; /* mmio flis access control */
	u32 offset;	/* pin offset from flis base address */
};

struct intel_scu_flis_platform_data {
	struct pinstruct_t *pin_t;
	int pin_num;
	u32 flis_base;
	u32 flis_len;
	struct pin_mmio_flis_t *mmio_flis_t;
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
int config_pin_flis(unsigned int name, enum flis_param_t param, u32 val);
int get_pin_flis(unsigned int name, enum flis_param_t param, u32 *val);
u32 get_flis_value(u32 offset);
void set_flis_value(u32 value, u32 offset);

#endif
