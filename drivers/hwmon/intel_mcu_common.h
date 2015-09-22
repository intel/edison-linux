#ifndef _EDISON_COMMON_H_
#define _EDISON_COMMON_H_

#define PSH2IA_CHANNEL0	0
#define PSH2IA_CHANNEL1	1
#define PSH2IA_CHANNEL2	2
#define PSH2IA_CHANNEL3	3

enum cmd_id {
	CMD_MCU_LOAD_APP = 0,
	CMD_MCU_SETUP_DDR,
	CMD_MCU_APP_DEBUG,
	CMD_MCU_APP_GET_VERSION,
};

#define CIRC_SIZE (1024 * 64)
struct ddr_param {
	u32 ddr;
	u32 ddr1;
} __packed;

#define CMD_DEBUG_SET_MASK	((u8)0x1)
#define CMD_DEBUG_GET_MASK	((u8)0x2)
#define MCU_DBG_ALL		((u16)-1)
#define MCU_DBG_FATAL		1
#define MCU_DBG_ERR		2
#define MCU_DBG_WARN	3
#define MCU_DBG_INFO	4
#define MCU_DBG_DBG		5

struct cmd_debug_param {
	u8 sub_cmd;
	u16 level;
	char tag[30];
} __packed;

#define RESP_PARAM_MAX_SIZE	56
struct cmd_resp {
	u8 cmd_id;
	u8 len;
	int ret;
	char param[RESP_PARAM_MAX_SIZE];
} __packed;

struct debug_resp {
	u16 level;
} __packed;

struct version_resp {
	u8 total_length;
	u8 segment_length;
	u8 sequence_number;
	char buf[0];
} __packed;

#define LBUF_CELL_SIGN ((u16)0x4853)
#define LBUF_EMPTY_SIGN ((u16)0x0000)
#define LBUF_DISCARD_SIGN ((u16)0x4944)
#define size_align(size) ((size % 4) ? (size + 4 - (size % 4)) : size)
#define frame_size(size) (size_align(size) + \
		sizeof(struct frame_head))

struct frame_head {
	u16 sign;
	u16 length;
	u8 buf[0];
} __packed;

#define BUF_IA_DDR_SIZE 8192
struct loop_buffer {
	int in_reading;
	u8 *addr;
	u16 length;

	u16 off_head;
	u16 off_tail;
};

#endif
