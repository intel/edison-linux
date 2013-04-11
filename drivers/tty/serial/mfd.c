/*
 * mfd.c: driver for High Speed UART device of Intel Medfield platform
 *
 * Refer pxa.c, 8250.c and some other drivers in drivers/serial/
 *
 * (C) Copyright 2010 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

/* Notes:
 * 1. DMA channel allocation: 0/1 channel are assigned to port 0,
 *    2/3 chan to port 1, 4/5 chan to port 3. Even number chans
 *    are used for RX, odd chans for TX
 *
 * 2. The RI/DSR/DCD/DTR are not pinned out, DCD & DSR are always
 *    asserted, only when the HW is reset the DDCD and DDSR will
 *    be triggered
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/sysrq.h>
#include <linux/slab.h>
#include <linux/serial_reg.h>
#include <linux/circ_buf.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial_core.h>
#include <linux/serial_mfd.h>
#include <linux/dma-mapping.h>
#include <linux/pci.h>
#include <linux/nmi.h>
#include <linux/io.h>
#include <linux/debugfs.h>
#include <linux/pm_runtime.h>
#include <linux/pm_qos.h>
#include <asm/intel_mid_hsu.h>

#define HSU_PORT_MAX		8
#define HSU_DMA_BUF_SIZE	2048
#define HSU_Q_MAX		1024
#define HSU_CL_BUF_LEN		(1 << CONFIG_LOG_BUF_SHIFT)

#define chan_readl(chan, offset)	readl(chan->reg + offset)
#define chan_writel(chan, offset, val)	writel(val, chan->reg + offset)

#define mfd_readl(obj, offset)		readl(obj->reg + offset)
#define mfd_writel(obj, offset, val)	writel(val, obj->reg + offset)

static int hsu_dma_enable = 0x7;
module_param(hsu_dma_enable, int, 0);
MODULE_PARM_DESC(hsu_dma_enable,
		 "It is a bitmap to set working mode, if bit[x] is 1, then port[x] will work in DMA mode, otherwise in PIO mode.");

enum {
	flag_init = 1,
	flag_startup,
	flag_console,
	flag_suspend,
	flag_wakeup,
	flag_set_alt,
};

enum {
	qcmd_overflow = 1,
	qcmd_set_mcr,
	qcmd_set_ier,
	qcmd_set_lcr,
	qcmd_set_speed,
	qcmd_stop_rx,
	qcmd_start_tx,
	qcmd_stop_tx,
	qcmd_startup,
	qcmd_shutdown,
	qcmd_cl,
	qcmd_set_alt,
	qcmd_clear_alt,
	qcmd_reg_port,
};

struct hsu_dma_buffer {
	u8		*buf;
	dma_addr_t	dma_addr;
	u32		dma_size;
	u32		ofs;
};

struct hsu_dma_chan {
	u32	id;
	enum dma_data_direction	dirt;
	struct uart_hsu_port	*uport;
	void __iomem		*reg;
};

struct uart_hsu_port {
	struct uart_port        port;

	struct workqueue_struct *workqueue;
	struct work_struct	work;
	struct circ_buf		qcirc;
	int			qbuf[HSU_Q_MAX];
	struct circ_buf		cl_circ;
	spinlock_t		cl_lock;

	unsigned char           lsr;
	unsigned char           msr;
	unsigned char           dll;
	unsigned char           dlm;
	unsigned char		fcr;
	unsigned char           ier;
	unsigned char           lcr;
	unsigned char           mcr;

	unsigned int		mul;
	unsigned int		ps;

	unsigned int            lsr_break_flag;
	char			name[12];
	int			index;
	struct device		*dev;

	struct hsu_dma_chan	*txc;
	struct hsu_dma_chan	*rxc;
	struct hsu_dma_buffer	txbuf;
	struct hsu_dma_buffer	rxbuf;
	unsigned char		rxc_chcr_save;
	int			use_dma;	/* flag for DMA/PIO */
	int			running;
	int			dma_tx_on;
	unsigned long		flags;
	struct pm_qos_request	qos;
};

/* Top level data structure of HSU */
struct hsu_port {
	int dma_irq;
	int init_completion;
	struct hsu_port_cfg	*configs[HSU_PORT_MAX];
	void __iomem	*reg;
	struct uart_hsu_port	port[HSU_PORT_MAX];
	struct hsu_dma_chan	chans[HSU_PORT_MAX * 2];
	struct dentry *debugfs;
};

static struct hsu_port hsu;
static struct hsu_port *phsu = &hsu;
static struct uart_driver serial_hsu_reg;
static struct hsu_port_cfg *hsu_port_func_cfg;

int hsu_register_board_info(void *inf)
{
	hsu_port_func_cfg = inf;
	return 0;
}

static inline void insert_qcmd(struct uart_hsu_port *up, char cmd)
{
	struct circ_buf *circ = &up->qcirc;
	char *buf;

	buf = circ->buf + circ->head;
	if (CIRC_SPACE(circ->head, circ->tail, HSU_Q_MAX) < 1)
		*buf = qcmd_overflow;
	else {
		*buf = cmd;
		circ->head++;
		if (circ->head == HSU_Q_MAX)
			circ->head = 0;
	}
}

static inline int get_qcmd(struct uart_hsu_port *up, char *cmd)
{
	struct circ_buf *circ = &up->qcirc;
	char *buf;

	if (!CIRC_CNT(circ->head, circ->tail, HSU_Q_MAX))
		return 0;
	buf = circ->buf + circ->tail;
	*cmd = *buf;
	circ->tail++;
	if (circ->tail == HSU_Q_MAX)
		circ->tail = 0;
	return 1;
}

static inline void cl_put_char(struct uart_hsu_port *up, char c)
{
	struct circ_buf *circ = &up->cl_circ;
	char *buf;
	unsigned long flags;

	spin_lock_irqsave(&up->cl_lock, flags);
	buf = circ->buf + circ->head;
	if (CIRC_SPACE(circ->head, circ->tail, HSU_CL_BUF_LEN) > 1) {
		*buf = c;
		circ->head++;
		if (circ->head == HSU_CL_BUF_LEN)
			circ->head = 0;
	}
	spin_unlock_irqrestore(&up->cl_lock, flags);
}

static inline int cl_get_char(struct uart_hsu_port *up, char *c)
{
	struct circ_buf *circ = &up->cl_circ;
	char *buf;
	unsigned long flags;

	spin_lock_irqsave(&up->cl_lock, flags);
	if (!CIRC_CNT(circ->head, circ->tail, HSU_CL_BUF_LEN)) {
		spin_unlock_irqrestore(&up->cl_lock, flags);
		return 0;
	}
	buf = circ->buf + circ->tail;
	*c = *buf;
	circ->tail++;
	if (circ->tail == HSU_CL_BUF_LEN)
		circ->tail = 0;
	spin_unlock_irqrestore(&up->cl_lock, flags);
	return 1;
}


static inline unsigned int serial_in(struct uart_hsu_port *up, int offset)
{
	unsigned int val;

	if (offset > UART_MSR) {
		offset <<= 2;
		val = readl(up->port.membase + offset);
	} else
		val = (unsigned int)readb(up->port.membase + offset);

	return val;
}

static inline void serial_out(struct uart_hsu_port *up, int offset, int value)
{
	if (offset > UART_MSR) {
		offset <<= 2;
		writel(value, up->port.membase + offset);
	} else {
		unsigned char val = value & 0xff;
		writeb(val, up->port.membase + offset);
	}
}

static void serial_set_alt(int index)
{
	struct uart_hsu_port *up = phsu->port + index;
	struct hsu_dma_chan *txc = up->txc;
	struct hsu_dma_chan *rxc = up->rxc;
	struct hsu_port_cfg *cfg = phsu->configs[index];
	struct pci_dev *pdev = container_of(up->dev, struct pci_dev, dev);

	if (test_bit(flag_set_alt, &up->flags))
		return;

	txc->uport = up;
	rxc->uport = up;
	pci_set_drvdata(pdev, up);
	if (cfg->hw_set_alt)
		cfg->hw_set_alt(index);
	if (cfg->hw_set_rts)
		cfg->hw_set_rts(up->index, 0);
	set_bit(flag_set_alt, &up->flags);
}

static void serial_clear_alt(int index)
{
	struct uart_hsu_port *up = phsu->port + index;
	struct hsu_port_cfg *cfg = phsu->configs[index];

	if (!test_bit(flag_set_alt, &up->flags))
		return;

	if (cfg->hw_set_rts)
		cfg->hw_set_rts(up->index, 1);
	clear_bit(flag_set_alt, &up->flags);
}

#ifdef CONFIG_DEBUG_FS

#define HSU_REGS_BUFSIZE	1024

static int hsu_show_regs_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t port_show_regs(struct file *file, char __user *user_buf,
				size_t count, loff_t *ppos)
{
	struct uart_hsu_port *up = file->private_data;
	char *buf;
	u32 len = 0;
	ssize_t ret;

	buf = kzalloc(HSU_REGS_BUFSIZE, GFP_KERNEL);
	if (!buf)
		return 0;

	len += snprintf(buf + len, HSU_REGS_BUFSIZE - len,
			"MFD HSU port[%d] regs:\n", up->index);

	len += snprintf(buf + len, HSU_REGS_BUFSIZE - len,
			"=================================\n");
	len += snprintf(buf + len, HSU_REGS_BUFSIZE - len,
			"IER: \t\t0x%08x\n", serial_in(up, UART_IER));
	len += snprintf(buf + len, HSU_REGS_BUFSIZE - len,
			"IIR: \t\t0x%08x\n", serial_in(up, UART_IIR));
	len += snprintf(buf + len, HSU_REGS_BUFSIZE - len,
			"LCR: \t\t0x%08x\n", serial_in(up, UART_LCR));
	len += snprintf(buf + len, HSU_REGS_BUFSIZE - len,
			"MCR: \t\t0x%08x\n", serial_in(up, UART_MCR));
	len += snprintf(buf + len, HSU_REGS_BUFSIZE - len,
			"LSR: \t\t0x%08x\n", serial_in(up, UART_LSR));
	len += snprintf(buf + len, HSU_REGS_BUFSIZE - len,
			"MSR: \t\t0x%08x\n", serial_in(up, UART_MSR));
	len += snprintf(buf + len, HSU_REGS_BUFSIZE - len,
			"FOR: \t\t0x%08x\n", serial_in(up, UART_FOR));
	len += snprintf(buf + len, HSU_REGS_BUFSIZE - len,
			"PS: \t\t0x%08x\n", serial_in(up, UART_PS));
	len += snprintf(buf + len, HSU_REGS_BUFSIZE - len,
			"MUL: \t\t0x%08x\n", serial_in(up, UART_MUL));
	len += snprintf(buf + len, HSU_REGS_BUFSIZE - len,
			"DIV: \t\t0x%08x\n", serial_in(up, UART_DIV));

	if (len > HSU_REGS_BUFSIZE)
		len = HSU_REGS_BUFSIZE;

	ret =  simple_read_from_buffer(user_buf, count, ppos, buf, len);
	kfree(buf);
	return ret;
}

static ssize_t dma_show_regs(struct file *file, char __user *user_buf,
				size_t count, loff_t *ppos)
{
	struct hsu_dma_chan *chan = file->private_data;
	char *buf;
	u32 len = 0;
	ssize_t ret;

	buf = kzalloc(HSU_REGS_BUFSIZE, GFP_KERNEL);
	if (!buf)
		return 0;

	len += snprintf(buf + len, HSU_REGS_BUFSIZE - len,
			"MFD HSU DMA channel [%d] regs:\n", chan->id);

	len += snprintf(buf + len, HSU_REGS_BUFSIZE - len,
			"=================================\n");
	len += snprintf(buf + len, HSU_REGS_BUFSIZE - len,
			"CR: \t\t0x%08x\n", chan_readl(chan, HSU_CH_CR));
	len += snprintf(buf + len, HSU_REGS_BUFSIZE - len,
			"DCR: \t\t0x%08x\n", chan_readl(chan, HSU_CH_DCR));
	len += snprintf(buf + len, HSU_REGS_BUFSIZE - len,
			"BSR: \t\t0x%08x\n", chan_readl(chan, HSU_CH_BSR));
	len += snprintf(buf + len, HSU_REGS_BUFSIZE - len,
			"MOTSR: \t\t0x%08x\n", chan_readl(chan, HSU_CH_MOTSR));
	len += snprintf(buf + len, HSU_REGS_BUFSIZE - len,
			"D0SAR: \t\t0x%08x\n", chan_readl(chan, HSU_CH_D0SAR));
	len += snprintf(buf + len, HSU_REGS_BUFSIZE - len,
			"D0TSR: \t\t0x%08x\n", chan_readl(chan, HSU_CH_D0TSR));
	len += snprintf(buf + len, HSU_REGS_BUFSIZE - len,
			"D0SAR: \t\t0x%08x\n", chan_readl(chan, HSU_CH_D1SAR));
	len += snprintf(buf + len, HSU_REGS_BUFSIZE - len,
			"D0TSR: \t\t0x%08x\n", chan_readl(chan, HSU_CH_D1TSR));
	len += snprintf(buf + len, HSU_REGS_BUFSIZE - len,
			"D0SAR: \t\t0x%08x\n", chan_readl(chan, HSU_CH_D2SAR));
	len += snprintf(buf + len, HSU_REGS_BUFSIZE - len,
			"D0TSR: \t\t0x%08x\n", chan_readl(chan, HSU_CH_D2TSR));
	len += snprintf(buf + len, HSU_REGS_BUFSIZE - len,
			"D0SAR: \t\t0x%08x\n", chan_readl(chan, HSU_CH_D3SAR));
	len += snprintf(buf + len, HSU_REGS_BUFSIZE - len,
			"D0TSR: \t\t0x%08x\n", chan_readl(chan, HSU_CH_D3TSR));

	if (len > HSU_REGS_BUFSIZE)
		len = HSU_REGS_BUFSIZE;

	ret =  simple_read_from_buffer(user_buf, count, ppos, buf, len);
	kfree(buf);
	return ret;
}

static const struct file_operations port_regs_ops = {
	.owner		= THIS_MODULE,
	.open		= hsu_show_regs_open,
	.read		= port_show_regs,
	.llseek		= default_llseek,
};

static const struct file_operations dma_regs_ops = {
	.owner		= THIS_MODULE,
	.open		= hsu_show_regs_open,
	.read		= dma_show_regs,
	.llseek		= default_llseek,
};

static int hsu_debugfs_init(struct hsu_port *hsu)
{
	int i;
	char name[32];

	hsu->debugfs = debugfs_create_dir("hsu", NULL);
	if (!hsu->debugfs)
		return -ENOMEM;

	for (i = 0; i < 3; i++) {
		snprintf(name, sizeof(name), "port_%d_regs", i);
		debugfs_create_file(name, S_IFREG | S_IRUGO,
			hsu->debugfs, (void *)(&hsu->port[i]), &port_regs_ops);
	}

	for (i = 0; i < 6; i++) {
		snprintf(name, sizeof(name), "dma_chan_%d_regs", i);
		debugfs_create_file(name, S_IFREG | S_IRUGO,
			hsu->debugfs, (void *)&hsu->chans[i], &dma_regs_ops);
	}

	return 0;
}

static void hsu_debugfs_remove(struct hsu_port *hsu)
{
	if (hsu->debugfs)
		debugfs_remove_recursive(hsu->debugfs);
}

#else
static inline int hsu_debugfs_init(struct hsu_port *hsu)
{
	return 0;
}

static inline void hsu_debugfs_remove(struct hsu_port *hsu)
{
}
#endif /* CONFIG_DEBUG_FS */

static void serial_hsu_enable_ms(struct uart_port *port)
{
	struct uart_hsu_port *up =
		container_of(port, struct uart_hsu_port, port);

	up->ier |= UART_IER_MSI;
	insert_qcmd(up, qcmd_set_ier);
	queue_work(up->workqueue, &up->work);
}

void hsu_dma_tx(struct uart_hsu_port *up)
{
	struct circ_buf *xmit = &up->port.state->xmit;
	struct hsu_dma_buffer *dbuf = &up->txbuf;
	int count;

	/* test_and_set_bit may be better, but anyway it's in lock protected mode */
	if (up->dma_tx_on)
		return;

	/* Update the circ buf info */
	xmit->tail += dbuf->ofs;
	xmit->tail &= UART_XMIT_SIZE - 1;

	up->port.icount.tx += dbuf->ofs;
	dbuf->ofs = 0;

	/* Disable the channel */
	chan_writel(up->txc, HSU_CH_CR, 0x0);

	if (!uart_circ_empty(xmit) && !uart_tx_stopped(&up->port)) {
		dma_sync_single_for_device(up->port.dev,
					   dbuf->dma_addr,
					   dbuf->dma_size,
					   DMA_TO_DEVICE);

		count = CIRC_CNT_TO_END(xmit->head, xmit->tail, UART_XMIT_SIZE);
		dbuf->ofs = count;

		/* Reprogram the channel */
		chan_writel(up->txc, HSU_CH_D0SAR, dbuf->dma_addr + xmit->tail);
		chan_writel(up->txc, HSU_CH_D0TSR, count);

		/* Reenable the channel */
		chan_writel(up->txc, HSU_CH_DCR, 0x1
						 | (0x1 << 8)
						 | (0x1 << 16));
		up->dma_tx_on = 1;
		chan_writel(up->txc, HSU_CH_CR, 0x1);
	}

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&up->port);
}

/* The buffer is already cache coherent */
void hsu_dma_start_rx_chan(struct hsu_dma_chan *rxc,
			struct hsu_dma_buffer *dbuf)
{
	dbuf->ofs = 0;

	chan_writel(rxc, HSU_CH_BSR, 32);
	chan_writel(rxc, HSU_CH_MOTSR, 4);

	chan_writel(rxc, HSU_CH_D0SAR, dbuf->dma_addr);
	chan_writel(rxc, HSU_CH_D0TSR, dbuf->dma_size);
	chan_writel(rxc, HSU_CH_DCR, 0x1 | (0x1 << 8)
					 | (0x1 << 16)
					 | (0x1 << 24)	/* timeout bit, see HSU Errata 1 */
					 );
	chan_writel(rxc, HSU_CH_CR, 0x3);
}

/* Protected by spin_lock_irqsave(port->lock) */
static void serial_hsu_start_tx(struct uart_port *port)
{
	struct uart_hsu_port *up =
		container_of(port, struct uart_hsu_port, port);

	insert_qcmd(up, qcmd_start_tx);
	queue_work(up->workqueue, &up->work);
}

static void serial_hsu_stop_tx(struct uart_port *port)
{
	struct uart_hsu_port *up =
		container_of(port, struct uart_hsu_port, port);

	insert_qcmd(up, qcmd_stop_tx);
	queue_work(up->workqueue, &up->work);
}

/* This is always called in spinlock protected mode, so
 * modify timeout timer is safe here */
void hsu_dma_rx(struct uart_hsu_port *up, u32 int_sts)
{
	struct hsu_dma_buffer *dbuf = &up->rxbuf;
	struct hsu_dma_chan *chan = up->rxc;
	struct uart_port *port = &up->port;
	struct tty_port *tport = &port->state->port;
	int count;

	/*
	 * First need to know how many is already transferred,
	 * then check if its a timeout DMA irq, and return
	 * the trail bytes out, push them up and reenable the
	 * channel
	 */

	/* Timeout IRQ, need wait some time, see Errata 2 */
	if (int_sts & 0xf00)
		udelay(2);

	/* Stop the channel */
	chan_writel(chan, HSU_CH_CR, 0x0);

	count = chan_readl(chan, HSU_CH_D0SAR) - dbuf->dma_addr;
	if (!count) {
		/* Restart the channel before we leave */
		chan_writel(chan, HSU_CH_CR, 0x3);
		return;
	}

	dma_sync_single_for_cpu(port->dev, dbuf->dma_addr,
			dbuf->dma_size, DMA_FROM_DEVICE);

	/*
	 * Head will only wrap around when we recycle
	 * the DMA buffer, and when that happens, we
	 * explicitly set tail to 0. So head will
	 * always be greater than tail.
	 */
	tty_insert_flip_string(tport, dbuf->buf, count);
	port->icount.rx += count;

	dma_sync_single_for_device(up->port.dev, dbuf->dma_addr,
			dbuf->dma_size, DMA_FROM_DEVICE);

	/* Reprogram the channel */
	chan_writel(chan, HSU_CH_D0SAR, dbuf->dma_addr);
	chan_writel(chan, HSU_CH_D0TSR, dbuf->dma_size);
	chan_writel(chan, HSU_CH_DCR, 0x1
					 | (0x1 << 8)
					 | (0x1 << 16)
					 | (0x1 << 24)	/* timeout bit, see HSU Errata 1 */
					 );
	tty_flip_buffer_push(tport);

	chan_writel(chan, HSU_CH_CR, 0x3);

}

static void serial_hsu_stop_rx(struct uart_port *port)
{
	struct uart_hsu_port *up =
		container_of(port, struct uart_hsu_port, port);

	insert_qcmd(up, qcmd_stop_rx);
	queue_work(up->workqueue, &up->work);
}

static inline void receive_chars(struct uart_hsu_port *up, int *status)
{
	unsigned int ch, flag;
	unsigned int max_count = 256;

	do {
		ch = serial_in(up, UART_RX);
		flag = TTY_NORMAL;
		up->port.icount.rx++;

		if (unlikely(*status & (UART_LSR_BI | UART_LSR_PE |
				       UART_LSR_FE | UART_LSR_OE))) {

			dev_warn(up->dev, "We really rush into ERR/BI case"
				"status = 0x%02x", *status);
			/* For statistics only */
			if (*status & UART_LSR_BI) {
				*status &= ~(UART_LSR_FE | UART_LSR_PE);
				up->port.icount.brk++;
				/*
				 * We do the SysRQ and SAK checking
				 * here because otherwise the break
				 * may get masked by ignore_status_mask
				 * or read_status_mask.
				 */
				if (uart_handle_break(&up->port))
					goto ignore_char;
			} else if (*status & UART_LSR_PE)
				up->port.icount.parity++;
			else if (*status & UART_LSR_FE)
				up->port.icount.frame++;
			if (*status & UART_LSR_OE)
				up->port.icount.overrun++;

			/* Mask off conditions which should be ignored. */
			*status &= up->port.read_status_mask;

#ifdef CONFIG_SERIAL_MFD_HSU_CONSOLE
			if (up->port.cons &&
				up->port.cons->index == up->port.line) {
				/* Recover the break flag from console xmit */
				*status |= up->lsr_break_flag;
				up->lsr_break_flag = 0;
			}
#endif
			if (*status & UART_LSR_BI) {
				flag = TTY_BREAK;
			} else if (*status & UART_LSR_PE)
				flag = TTY_PARITY;
			else if (*status & UART_LSR_FE)
				flag = TTY_FRAME;
		}

		if (uart_handle_sysrq_char(&up->port, ch))
			goto ignore_char;

		uart_insert_char(&up->port, *status, UART_LSR_OE, ch, flag);
	ignore_char:
		*status = serial_in(up, UART_LSR);
	} while ((*status & UART_LSR_DR) && max_count--);
	tty_flip_buffer_push(&up->port.state->port);
}

static void transmit_chars(struct uart_hsu_port *up)
{
	struct circ_buf *xmit = &up->port.state->xmit;
	int count;

	if (up->port.x_char) {
		serial_out(up, UART_TX, up->port.x_char);
		up->port.icount.tx++;
		up->port.x_char = 0;
		return;
	}
	if (uart_circ_empty(xmit) || uart_tx_stopped(&up->port)) {
		serial_hsu_stop_tx(&up->port);
		return;
	}

	/* The IRQ is for TX FIFO half-empty */
	count = up->port.fifosize / 2;

	do {
		serial_out(up, UART_TX, xmit->buf[xmit->tail]);
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);

		up->port.icount.tx++;
		if (uart_circ_empty(xmit))
			break;
	} while (--count > 0);

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&up->port);

	if (uart_circ_empty(xmit))
		serial_hsu_stop_tx(&up->port);
}

static inline void check_modem_status(struct uart_hsu_port *up)
{
	int status;

	status = serial_in(up, UART_MSR);

	if ((status & UART_MSR_ANY_DELTA) == 0)
		return;

	if (status & UART_MSR_TERI)
		up->port.icount.rng++;
	if (status & UART_MSR_DDSR)
		up->port.icount.dsr++;
	/* We may only get DDCD when HW init and reset */
	if (status & UART_MSR_DDCD)
		uart_handle_dcd_change(&up->port, status & UART_MSR_DCD);
	/* Will start/stop_tx accordingly */
	if (status & UART_MSR_DCTS)
		uart_handle_cts_change(&up->port, status & UART_MSR_CTS);

	wake_up_interruptible(&up->port.state->port.delta_msr_wait);
}

/*
 * This handles the interrupt from one port.
 */
static irqreturn_t port_irq(int irq, void *dev_id)
{
	struct uart_hsu_port *up = dev_id;
	unsigned int iir, lsr;
	unsigned long flags;

	if (unlikely(!test_bit(flag_set_alt, &up->flags)))
		return IRQ_NONE;

	if (unlikely(!up->running))
		return IRQ_NONE;

	spin_lock_irqsave(&up->port.lock, flags);
	if (up->use_dma) {
		lsr = serial_in(up, UART_LSR);
		if (unlikely(lsr & (UART_LSR_BI | UART_LSR_PE |
				       UART_LSR_FE | UART_LSR_OE)))
			dev_warn(up->dev,
				"Got lsr irq while using DMA, lsr = 0x%2x\n",
				lsr);
		check_modem_status(up);
		spin_unlock_irqrestore(&up->port.lock, flags);
		return IRQ_HANDLED;
	}

	iir = serial_in(up, UART_IIR);
	if (iir & UART_IIR_NO_INT) {
		spin_unlock_irqrestore(&up->port.lock, flags);
		return IRQ_NONE;
	}

	lsr = serial_in(up, UART_LSR);
	if (lsr & UART_LSR_DR)
		receive_chars(up, &lsr);
	check_modem_status(up);

	/* lsr will be renewed during the receive_chars */
	if (lsr & UART_LSR_THRE)
		transmit_chars(up);

	spin_unlock_irqrestore(&up->port.lock, flags);
	return IRQ_HANDLED;
}

static inline void dma_chan_irq(struct hsu_dma_chan *chan)
{
	struct uart_hsu_port *up = chan->uport;
	unsigned long flags;
	u32 int_sts;

	spin_lock_irqsave(&up->port.lock, flags);

	if (!up->use_dma || !up->running)
		goto exit;

	/*
	 * No matter what situation, need read clear the IRQ status
	 * There is a bug, see Errata 5, HSD 2900918
	 */
	int_sts = chan_readl(chan, HSU_CH_SR);

	/* Rx channel */
	if (chan->dirt == DMA_FROM_DEVICE)
		hsu_dma_rx(up, int_sts);

	/* Tx channel */
	if (chan->dirt == DMA_TO_DEVICE) {
		chan_writel(chan, HSU_CH_CR, 0x0);
		up->dma_tx_on = 0;
		hsu_dma_tx(up);
	}

exit:
	spin_unlock_irqrestore(&up->port.lock, flags);
	return;
}

static irqreturn_t dma_irq(int irq, void *dev_id)
{
	struct hsu_port *hsu = dev_id;
	u32 int_sts, i;

	int_sts = mfd_readl(hsu, HSU_GBL_DMAISR);

	/* Currently we only have 6 channels may be used */
	for (i = 0; i < 6; i++) {
		if (int_sts & 0x1)
			dma_chan_irq(&hsu->chans[i]);
		int_sts >>= 1;
	}

	return IRQ_HANDLED;
}

static unsigned int serial_hsu_tx_empty(struct uart_port *port)
{
	struct uart_hsu_port *up =
		container_of(port, struct uart_hsu_port, port);

	return  up->lsr & UART_LSR_TEMT ? TIOCSER_TEMT : 0;
}

static unsigned int serial_hsu_get_mctrl(struct uart_port *port)
{
	struct uart_hsu_port *up =
		container_of(port, struct uart_hsu_port, port);
	unsigned char status = up->msr;
	unsigned int ret = 0;

	if (status & UART_MSR_DCD)
		ret |= TIOCM_CAR;
	if (status & UART_MSR_RI)
		ret |= TIOCM_RNG;
	if (status & UART_MSR_DSR)
		ret |= TIOCM_DSR;
	if (status & UART_MSR_CTS)
		ret |= TIOCM_CTS;
	return ret;
}

static void serial_hsu_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	struct uart_hsu_port *up =
		container_of(port, struct uart_hsu_port, port);

	if (mctrl & TIOCM_RTS)
		up->mcr |= UART_MCR_RTS;
	if (mctrl & TIOCM_DTR)
		up->mcr |= UART_MCR_DTR;
	if (mctrl & TIOCM_OUT1)
		up->mcr |= UART_MCR_OUT1;
	if (mctrl & TIOCM_OUT2)
		up->mcr |= UART_MCR_OUT2;
	if (mctrl & TIOCM_LOOP)
		up->mcr |= UART_MCR_LOOP;
	insert_qcmd(up, qcmd_set_mcr);
	queue_work(up->workqueue, &up->work);
}

static void serial_hsu_break_ctl(struct uart_port *port, int break_state)
{
	struct uart_hsu_port *up =
		container_of(port, struct uart_hsu_port, port);
	unsigned long flags;

	spin_lock_irqsave(&up->port.lock, flags);
	if (break_state == -1)
		up->lcr |= UART_LCR_SBC;
	else
		up->lcr &= ~UART_LCR_SBC;
	insert_qcmd(up, qcmd_set_lcr);
	queue_work(up->workqueue, &up->work);
	spin_unlock_irqrestore(&up->port.lock, flags);
}

/*
 * What special to do:
 * 1. chose the 64B fifo mode
 * 2. start dma or pio depends on configuration
 * 3. we only allocate dma memory when needed
 */
static int serial_hsu_startup(struct uart_port *port)
{
	int run_cnt = 0;
	static DEFINE_MUTEX(lock);
	struct uart_hsu_port *up =
		container_of(port, struct uart_hsu_port, port);
	struct hsu_port_cfg *cfg = phsu->configs[up->index];

	mutex_lock(&lock);
	if (cfg->has_alt) {
		struct hsu_port_cfg *alt_cfg = hsu_port_func_cfg + cfg->alt;
		struct uart_hsu_port *alt_up = phsu->port + alt_cfg->index;

		if (alt_up->running && alt_cfg->force_suspend) {
			uart_suspend_port(&serial_hsu_reg, &alt_up->port);
		} else {
			while (alt_up->running) {
				msleep(20);
				if (++run_cnt >= 50) {
					dev_warn(up->dev, "waite alt port %d 1S\n",
						 alt_up->index);
					run_cnt = 0;
				}
			}
		}
		insert_qcmd(alt_up, qcmd_clear_alt);
		queue_work(alt_up->workqueue, &alt_up->work);
		flush_workqueue(alt_up->workqueue);
	}
	insert_qcmd(up, qcmd_set_alt);
	queue_work(up->workqueue, &up->work);
	up->running = 1;
	mutex_unlock(&lock);

	insert_qcmd(up, qcmd_startup);
	queue_work(up->workqueue, &up->work);
	flush_workqueue(up->workqueue);
	return 0;
}

static void serial_hsu_shutdown(struct uart_port *port)
{
	static DEFINE_MUTEX(lock);
	struct uart_hsu_port *up =
		container_of(port, struct uart_hsu_port, port);
	struct hsu_port_cfg *cfg = phsu->configs[up->index];

	insert_qcmd(up, qcmd_shutdown);
	insert_qcmd(up, qcmd_clear_alt);
	queue_work(up->workqueue, &up->work);
	flush_workqueue(up->workqueue);

	mutex_lock(&lock);
	if (cfg->has_alt) {
		struct hsu_port_cfg *alt_cfg = hsu_port_func_cfg + cfg->alt;
		struct uart_hsu_port *alt_up = phsu->port + alt_cfg->index;

		if (alt_up->running) {
			insert_qcmd(alt_up, qcmd_set_alt);
			queue_work(alt_up->workqueue, &alt_up->work);
			uart_resume_port(&serial_hsu_reg, &alt_up->port);
		}
	}
	if (!test_bit(flag_console, &up->flags))
		up->running = 0;
	mutex_unlock(&lock);
}

static void
serial_hsu_set_termios(struct uart_port *port, struct ktermios *termios,
		       struct ktermios *old)
{
	struct uart_hsu_port *up =
			container_of(port, struct uart_hsu_port, port);
	unsigned char cval, fcr = 0;
	unsigned long flags;
	unsigned int baud, quot;
	u32 ps, mul;

	switch (termios->c_cflag & CSIZE) {
	case CS5:
		cval = UART_LCR_WLEN5;
		break;
	case CS6:
		cval = UART_LCR_WLEN6;
		break;
	case CS7:
		cval = UART_LCR_WLEN7;
		break;
	default:
	case CS8:
		cval = UART_LCR_WLEN8;
		break;
	}

	/* CMSPAR isn't supported by this driver */
	termios->c_cflag &= ~CMSPAR;

	if (termios->c_cflag & CSTOPB)
		cval |= UART_LCR_STOP;
	if (termios->c_cflag & PARENB)
		cval |= UART_LCR_PARITY;
	if (!(termios->c_cflag & PARODD))
		cval |= UART_LCR_EPAR;

	/*
	 * The base clk is 50Mhz, and the baud rate come from:
	 *	baud = 50M * MUL / (DIV * PS * DLAB)
	 *
	 * For those basic low baud rate we can get the direct
	 * scalar from 2746800, like 115200 = 2746800/24. For those
	 * higher baud rate, we handle them case by case, mainly by
	 * adjusting the MUL/PS registers, and DIV register is kept
	 * as default value 0x3d09 to make things simple
	 */
	baud = uart_get_baud_rate(port, termios, old, 0, 4000000);

	quot = 1;
	ps = 0x10;
	mul = 0x3600;
	switch (baud) {
	case 3500000:
		mul = 0x3345;
		ps = 0xC;
		break;
	case 1843200:
		mul = 0x2400;
		break;
	case 3000000:
	case 2500000:
	case 2000000:
	case 1500000:
	case 1000000:
	case 500000:
		/* mul/ps/quot = 0x9C4/0x10/0x1 will make a 500000 bps */
		mul = baud / 500000 * 0x9C4;
		break;
	default:
		/* Use uart_get_divisor to get quot for other baud rates */
		quot = 0;
	}

	if (!quot)
		quot = uart_get_divisor(port, baud);

	if ((up->port.uartclk / quot) < (2400 * 16))
		fcr = UART_FCR_ENABLE_FIFO | UART_FCR_HSU_64_1B;
	else if ((up->port.uartclk / quot) < (230400 * 16))
		fcr = UART_FCR_ENABLE_FIFO | UART_FCR_HSU_64_16B;
	else
		fcr = UART_FCR_ENABLE_FIFO | UART_FCR_HSU_64_32B;

	fcr |= UART_FCR_HSU_64B_FIFO;

	/*
	 * Ok, we're now changing the port state.  Do it with
	 * interrupts disabled.
	 */
	spin_lock_irqsave(&up->port.lock, flags);

	/* Update the per-port timeout */
	uart_update_timeout(port, termios->c_cflag, baud);

	up->port.read_status_mask = UART_LSR_OE | UART_LSR_THRE | UART_LSR_DR;
	if (termios->c_iflag & INPCK)
		up->port.read_status_mask |= UART_LSR_FE | UART_LSR_PE;
	if (termios->c_iflag & (BRKINT | PARMRK))
		up->port.read_status_mask |= UART_LSR_BI;

	/* Characters to ignore */
	up->port.ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR)
		up->port.ignore_status_mask |= UART_LSR_PE | UART_LSR_FE;
	if (termios->c_iflag & IGNBRK) {
		up->port.ignore_status_mask |= UART_LSR_BI;
		/*
		 * If we're ignoring parity and break indicators,
		 * ignore overruns too (for real raw support).
		 */
		if (termios->c_iflag & IGNPAR)
			up->port.ignore_status_mask |= UART_LSR_OE;
	}

	/* Ignore all characters if CREAD is not set */
	if ((termios->c_cflag & CREAD) == 0)
		up->port.ignore_status_mask |= UART_LSR_DR;

	/*
	 * CTS flow control flag and modem status interrupts, disable
	 * MSI by default
	 */
	up->ier &= ~UART_IER_MSI;
	if (UART_ENABLE_MS(&up->port, termios->c_cflag))
		up->ier |= UART_IER_MSI;
	insert_qcmd(up, qcmd_set_ier);

	if (termios->c_cflag & CRTSCTS)
		up->mcr |= UART_MCR_AFE | UART_MCR_RTS;
	else
		up->mcr &= ~UART_MCR_AFE;
	up->lcr = cval;
	up->dll = quot & 0xff;
	up->dlm = quot >> 8;
	up->mul = mul;
	up->ps = ps;
	up->fcr = fcr;
	insert_qcmd(up, qcmd_set_speed);
	queue_work(up->workqueue, &up->work);
	serial_hsu_set_mctrl(&up->port, up->port.mctrl);
	spin_unlock_irqrestore(&up->port.lock, flags);
}

static void
serial_hsu_pm(struct uart_port *port, unsigned int state,
	      unsigned int oldstate)
{
}

static void serial_hsu_release_port(struct uart_port *port)
{
}

static int serial_hsu_request_port(struct uart_port *port)
{
	return 0;
}

static void serial_hsu_config_port(struct uart_port *port, int flags)
{
	struct uart_hsu_port *up =
		container_of(port, struct uart_hsu_port, port);
	up->port.type = PORT_MFD;
}

static int
serial_hsu_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	/* We don't want the core code to modify any port params */
	return -EINVAL;
}

static const char *
serial_hsu_type(struct uart_port *port)
{
	struct uart_hsu_port *up =
		container_of(port, struct uart_hsu_port, port);
	return up->name;
}

#ifdef CONFIG_SERIAL_MFD_HSU_CONSOLE

#define BOTH_EMPTY (UART_LSR_TEMT | UART_LSR_THRE)

/* Wait for transmitter & holding register to empty */
static inline void wait_for_xmitr(struct uart_hsu_port *up)
{
	unsigned int status, tmout = 1000;

	/* Wait up to 1ms for the character to be sent. */
	do {
		status = serial_in(up, UART_LSR);

		if (status & UART_LSR_BI)
			up->lsr_break_flag = UART_LSR_BI;

		if (--tmout == 0)
			break;
		udelay(1);
	} while (!(status & BOTH_EMPTY));

	/* Wait up to 1s for flow control if necessary */
	if (up->port.flags & UPF_CONS_FLOW) {
		tmout = 1000000;
		while (--tmout &&
		       ((serial_in(up, UART_MSR) & UART_MSR_CTS) == 0))
			udelay(1);
	}
}

static void serial_hsu_console_putchar(struct uart_port *port, int ch)
{
	struct uart_hsu_port *up =
		container_of(port, struct uart_hsu_port, port);
	cl_put_char(up, ch);
}

/*
 * Print a string to the serial port trying not to disturb
 * any possible real use of the port...
 *
 *	The console_lock must be held when we get here.
 */
static void
serial_hsu_console_write(struct console *co, const char *s, unsigned int count)
{
	struct uart_hsu_port *up = phsu->port + co->index;
	unsigned long flags;

	uart_console_write(&up->port, s, count, serial_hsu_console_putchar);
	spin_lock_irqsave(&up->cl_lock, flags);
	insert_qcmd(up, qcmd_cl);
	spin_unlock_irqrestore(&up->cl_lock, flags);
	queue_work(up->workqueue, &up->work);
}

static struct console serial_hsu_console;

static int __init
serial_hsu_console_setup(struct console *co, char *options)
{
	struct uart_hsu_port *up = phsu->port + co->index;
	int baud = 115200;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';

	if (co->index < 0 || co->index >= hsu_port_max)
		return -ENODEV;

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);

	set_bit(flag_console, &up->flags);
	up->running = 1;
	insert_qcmd(up, qcmd_set_alt);
	queue_work(up->workqueue, &up->work);
	up->cl_circ.buf = kzalloc(HSU_CL_BUF_LEN, GFP_KERNEL);
	if (up->cl_circ.buf == NULL)
		return -ENOMEM;
	return uart_set_options(&up->port, co, baud, parity, bits, flow);
}

static struct console serial_hsu_console = {
	.name		= "ttyMFD",
	.write		= serial_hsu_console_write,
	.device		= uart_console_device,
	.setup		= serial_hsu_console_setup,
	.flags		= CON_PRINTBUFFER,
	.index		= -1,
	.data		= &serial_hsu_reg,
};

#define SERIAL_HSU_CONSOLE	(&serial_hsu_console)
#else
#define SERIAL_HSU_CONSOLE	NULL
#endif

struct uart_ops serial_hsu_pops = {
	.tx_empty	= serial_hsu_tx_empty,
	.set_mctrl	= serial_hsu_set_mctrl,
	.get_mctrl	= serial_hsu_get_mctrl,
	.stop_tx	= serial_hsu_stop_tx,
	.start_tx	= serial_hsu_start_tx,
	.stop_rx	= serial_hsu_stop_rx,
	.enable_ms	= serial_hsu_enable_ms,
	.break_ctl	= serial_hsu_break_ctl,
	.startup	= serial_hsu_startup,
	.shutdown	= serial_hsu_shutdown,
	.set_termios	= serial_hsu_set_termios,
	.pm		= serial_hsu_pm,
	.type		= serial_hsu_type,
	.release_port	= serial_hsu_release_port,
	.request_port	= serial_hsu_request_port,
	.config_port	= serial_hsu_config_port,
	.verify_port	= serial_hsu_verify_port,
};

static struct uart_driver serial_hsu_reg = {
	.owner		= THIS_MODULE,
	.driver_name	= "MFD serial",
	.dev_name	= "ttyMFD",
	.major		= TTY_MAJOR,
	.minor		= 128,
	.nr		= HSU_PORT_MAX,
};

static irqreturn_t wakeup_irq(int irq, void *dev)
{
	struct pci_dev *pdev = container_of(dev, struct pci_dev, dev);
	struct uart_hsu_port *up = pci_get_drvdata(pdev);

	set_bit(flag_wakeup, &up->flags);
	pm_runtime_get(dev);
	pm_runtime_put(dev);
	return IRQ_HANDLED;
}

#if defined(CONFIG_PM) || defined(CONFIG_PM_RUNTIME)
static int serial_hsu_do_suspend(struct pci_dev *pdev)
{
	struct uart_hsu_port *up = pci_get_drvdata(pdev);
	struct hsu_port_cfg *cfg = phsu->configs[up->index];

	if (unlikely(!phsu->init_completion))
		return -EBUSY;

	if (cfg->hw_set_rts)
		cfg->hw_set_rts(up->index, 1);
	disable_irq(up->port.irq);
	up->rxc_chcr_save = chan_readl(up->rxc, HSU_CH_CR);
	chan_writel(up->rxc, HSU_CH_CR, 0x2);
	udelay(10);
	synchronize_irq(phsu->dma_irq);
	if (cfg->hw_suspend)
		cfg->hw_suspend(up->index, &pdev->dev, wakeup_irq);
	return 0;
}

static int serial_hsu_do_resume(struct pci_dev *pdev)
{
	struct uart_hsu_port *up = pci_get_drvdata(pdev);
	struct hsu_port_cfg *cfg = phsu->configs[up->index];

	if (cfg->hw_resume)
		cfg->hw_resume(up->index, &pdev->dev);
	if (cfg->hw_set_rts)
		cfg->hw_set_rts(up->index, 0);
	enable_irq(up->port.irq);
	chan_writel(up->rxc, HSU_CH_CR, up->rxc_chcr_save);
	return 0;
}
#endif

#ifdef CONFIG_PM
static int serial_hsu_suspend(struct pci_dev *pdev, pm_message_t state)
{
	return serial_hsu_do_suspend(pdev);
}

static int serial_hsu_resume(struct pci_dev *pdev)
{
	return serial_hsu_do_resume(pdev);
}
#else
#define serial_hsu_suspend	NULL
#define serial_hsu_resume	NULL
#endif

#ifdef CONFIG_PM_RUNTIME
static int serial_hsu_runtime_idle(struct device *dev)
{
	struct pci_dev *pdev = container_of(dev, struct pci_dev, dev);
	struct uart_hsu_port *up = pci_get_drvdata(pdev);
	struct hsu_port_cfg *cfg = phsu->configs[up->index];

	if (test_bit(flag_console, &up->flags) &&
		test_bit(flag_wakeup, &up->flags)) {
		pm_schedule_suspend(dev, 2000);
		clear_bit(flag_wakeup, &up->flags);
	} else
		pm_schedule_suspend(dev, cfg->idle);
	return -EBUSY;
}

static int serial_hsu_runtime_suspend(struct device *dev)
{
	struct pci_dev *pdev = container_of(dev, struct pci_dev, dev);

	return serial_hsu_do_suspend(pdev);
}

static int serial_hsu_runtime_resume(struct device *dev)
{
	struct pci_dev *pdev = container_of(dev, struct pci_dev, dev);

	return serial_hsu_do_resume(pdev);
}
#else
#define serial_hsu_runtime_idle		NULL
#define serial_hsu_runtime_suspend	NULL
#define serial_hsu_runtime_resume	NULL
#endif

static void serial_hsu_work(struct work_struct *work)
{
	struct uart_hsu_port *up =
		container_of(work, struct uart_hsu_port, work);
	char cmd, c;
	unsigned long flags;

	while (unlikely(!phsu->init_completion))
		msleep(100);
	pm_qos_add_request(&up->qos,
			PM_QOS_CPU_DMA_LATENCY,	1040-1);
	pm_runtime_get_sync(up->dev);
	spin_lock_irqsave(&up->port.lock, flags);
	while (get_qcmd(up, &cmd)) {
		spin_unlock_irqrestore(&up->port.lock, flags);
		switch (cmd) {
		case qcmd_overflow:
			dev_err(up->dev, "queue overflow!!\n");
			break;
		case qcmd_set_mcr:
			serial_out(up, UART_MCR, up->mcr);
			break;
		case qcmd_set_ier:
			serial_out(up, UART_IER, up->ier);
			break;
		case qcmd_set_lcr:
			serial_out(up, UART_LCR, up->lcr);
			break;
		case qcmd_set_speed:
			serial_out(up, UART_LCR, up->lcr | UART_LCR_DLAB);
			serial_out(up, UART_DLL, up->dll);
			serial_out(up, UART_DLM, up->dlm);
			serial_out(up, UART_LCR, up->lcr);
			serial_out(up, UART_MUL, up->mul);
			serial_out(up, UART_PS, up->ps);
			serial_out(up, UART_FCR, up->fcr);
			break;
		case qcmd_stop_rx:
			if (up->use_dma) {
				chan_writel(up->rxc, HSU_CH_CR, 0x2);
			} else {
				up->ier &= ~UART_IER_RLSI;
				up->port.read_status_mask &= ~UART_LSR_DR;
				serial_out(up, UART_IER, up->ier);
			}
			break;
		case qcmd_start_tx:
			if (up->use_dma) {
				hsu_dma_tx(up);
			} else if (!(up->ier & UART_IER_THRI)) {
				up->ier |= UART_IER_THRI;
				serial_out(up, UART_IER, up->ier);
			}
			break;
		case qcmd_stop_tx:
			if (up->use_dma)
				chan_writel(up->txc, HSU_CH_CR, 0x0);
			else if (up->ier & UART_IER_THRI) {
				up->ier &= ~UART_IER_THRI;
				serial_out(up, UART_IER, up->ier);
			}
			break;
		case qcmd_startup:
			/*
			 * Clear the FIFO buffers and disable them.
			 * (they will be reenabled in set_termios())
			 */
			serial_out(up, UART_FCR, UART_FCR_ENABLE_FIFO);
			serial_out(up, UART_FCR, UART_FCR_ENABLE_FIFO |
					UART_FCR_CLEAR_RCVR |
					UART_FCR_CLEAR_XMIT);
			serial_out(up, UART_FCR, 0);
			up->fcr = 0;

			/* Clear the interrupt registers. */
			(void) serial_in(up, UART_LSR);
			(void) serial_in(up, UART_RX);
			(void) serial_in(up, UART_IIR);
			(void) serial_in(up, UART_MSR);

			/* Now, initialize the UART, default is 8n1 */
			serial_out(up, UART_LCR, UART_LCR_WLEN8);
			up->lcr = UART_LCR_WLEN8;

			up->port.mctrl |= TIOCM_OUT2;
			serial_hsu_set_mctrl(&up->port, up->port.mctrl);

			/*
			 * Finally, enable interrupts.  Note: Modem status
			 * interrupts are set via set_termios(), which will
			 *  be occurring imminently
			 * anyway, so we don't enable them here.
			 */
			if (!up->use_dma)
				up->ier = UART_IER_RLSI | UART_IER_RDI |
						UART_IER_RTOIE;
			else
				up->ier = 0;
			serial_out(up, UART_IER, up->ier);

			/* DMA init */
			if (up->use_dma) {
				struct hsu_dma_buffer *dbuf;
				struct circ_buf *xmit = &up->port.state->xmit;

				up->dma_tx_on = 0;

				/* First allocate the RX buffer */
				dbuf = &up->rxbuf;
				dbuf->buf = kzalloc(HSU_DMA_BUF_SIZE,
							GFP_KERNEL);
				if (!dbuf->buf) {
					up->use_dma = 0;
					dev_err(up->dev, "allocate DMA buffer failed!!\n");
					break;
				}
				dbuf->dma_addr = dma_map_single(up->dev,
						dbuf->buf,
						HSU_DMA_BUF_SIZE,
						DMA_FROM_DEVICE);
				dbuf->dma_size = HSU_DMA_BUF_SIZE;

				/* Start the RX channel right now */
				hsu_dma_start_rx_chan(up->rxc, dbuf);

				/* Next init the TX DMA */
				dbuf = &up->txbuf;
				dbuf->buf = xmit->buf;
				dbuf->dma_addr = dma_map_single(up->dev,
						dbuf->buf,
						UART_XMIT_SIZE,
						DMA_TO_DEVICE);
				dbuf->dma_size = UART_XMIT_SIZE;

				/* This should not be changed all around */
				chan_writel(up->txc, HSU_CH_BSR, 32);
				chan_writel(up->txc, HSU_CH_MOTSR, 4);
				dbuf->ofs = 0;
			}

			/* And clear the interrupt registers again for luck. */
			(void) serial_in(up, UART_LSR);
			(void) serial_in(up, UART_RX);
			(void) serial_in(up, UART_IIR);
			(void) serial_in(up, UART_MSR);

			break;
		case qcmd_shutdown:
			/* Disable interrupts from this port */
			up->ier = 0;
			serial_out(up, UART_IER, 0);

			up->port.mctrl &= ~TIOCM_OUT2;
			serial_hsu_set_mctrl(&up->port, up->port.mctrl);

			/* Disable break condition and FIFOs */
			serial_out(up, UART_LCR,
				serial_in(up, UART_LCR) & ~UART_LCR_SBC);
			serial_out(up, UART_FCR, UART_FCR_ENABLE_FIFO |
					UART_FCR_CLEAR_RCVR |
					UART_FCR_CLEAR_XMIT);
			serial_out(up, UART_FCR, 0);
			up->fcr = 0;
			break;
		case qcmd_cl:
			serial_out(up, UART_IER, 0);
			while (cl_get_char(up, &c)) {
				wait_for_xmitr(up);
				serial_out(up, UART_TX, c);
			}
			serial_out(up, UART_IER, up->ier);
			break;
		case qcmd_set_alt:
			serial_set_alt(up->index);
			break;
		case qcmd_clear_alt:
			serial_clear_alt(up->index);
			break;
		default:
			dev_err(up->dev, "invalid command!!\n");
			break;
		}
		spin_lock_irqsave(&up->port.lock, flags);
	}
	up->lsr = serial_in(up, UART_LSR);
	spin_unlock_irqrestore(&up->port.lock, flags);
	pm_runtime_put(up->dev);
	pm_qos_remove_request(&up->qos);
}

static const struct dev_pm_ops serial_hsu_pm_ops = {
	.runtime_suspend = serial_hsu_runtime_suspend,
	.runtime_resume = serial_hsu_runtime_resume,
	.runtime_idle = serial_hsu_runtime_idle,
};

static int serial_hsu_probe(struct pci_dev *pdev,
				const struct pci_device_id *ent)
{
	struct uart_hsu_port *uport;
	struct hsu_dma_chan *dchan;
	int i, index, ret;

	printk(KERN_INFO "HSU: found PCI Serial controller(ID: %04x:%04x)\n",
		pdev->vendor, pdev->device);

	ret = pci_enable_device(pdev);
	if (ret)
		return ret;

	if (ent->driver_data == hsu_dma) {
		ret = pci_request_region(pdev, 0, "hsu dma");
		if (ret)
			goto err_disable;

		phsu->reg = ioremap_nocache(pci_resource_start(pdev, 0),
						pci_resource_len(pdev, 0));
		dchan = phsu->chans;
		for (i = 0; i < 6; i++) {
			dchan->id = i;
			dchan->dirt = (i & 0x1) ? DMA_FROM_DEVICE :
						DMA_TO_DEVICE;
			dchan->uport = &phsu->port[i/2];
			dchan->reg = phsu->reg + HSU_DMA_CHANS_REG_OFFSET +
					i * HSU_DMA_CHANS_REG_LENGTH;

			dchan++;
		}
		phsu->dma_irq = pdev->irq;
		ret = request_irq(pdev->irq, dma_irq, 0, "hsu dma", phsu);
		if (ret) {
			dev_err(&pdev->dev, "can not get IRQ\n");
			goto err_disable;
		}
		pci_set_drvdata(pdev, phsu);
		phsu->init_completion = 1;
	} else {
		struct hsu_port_cfg *cfg = hsu_port_func_cfg + ent->driver_data;
		index = cfg->index;
		phsu->configs[index] = cfg;
		uport = phsu->port + index;

		ret = pci_request_region(pdev, 0, cfg->name);
		if (ret)
			goto err_disable;

		if (cfg->hw_init)
			cfg->hw_init(index);

		uport->dev = &pdev->dev;
		uport->port.type = PORT_MFD;
		uport->port.iotype = UPIO_MEM;
		uport->port.mapbase = pci_resource_start(pdev, 0);
		uport->port.membase = ioremap_nocache(uport->port.mapbase,
						pci_resource_len(pdev, 0));
		strcpy(uport->name, cfg->name);
		uport->port.fifosize = 64;
		uport->port.ops = &serial_hsu_pops;
		uport->port.line = index;
		uport->port.flags = UPF_IOREMAP;
		/* set the scalable maxim support rate to 2746800 bps */
		uport->port.uartclk = 115200 * 24 * 16;
		uport->port.irq = pdev->irq;
		uport->port.dev = &pdev->dev;

		uport->running = 0;
		uport->txc = &phsu->chans[index * 2];
		uport->rxc = &phsu->chans[index * 2 + 1];

		uport->index = index;

		if (hsu_dma_enable & (1<<index))
			uport->use_dma = 1;
		else
			uport->use_dma = 0;

		uport->workqueue = create_singlethread_workqueue(uport->name);
		INIT_WORK(&uport->work, serial_hsu_work);
		uport->qcirc.buf = (char *)uport->qbuf;
		spin_lock_init(&uport->cl_lock);
		ret = request_irq(pdev->irq, port_irq, IRQF_SHARED,
					uport->name, uport);
		if (ret) {
			dev_err(&pdev->dev, "can not get IRQ\n");
			goto err_disable;
		}
		if (cfg->type == debug_port) {
			serial_hsu_reg.cons = SERIAL_HSU_CONSOLE;
			serial_hsu_reg.cons->index = index;
		} else
			serial_hsu_reg.cons = NULL;
		uart_add_one_port(&serial_hsu_reg, &uport->port);
		pci_set_drvdata(pdev, uport);
		pm_runtime_put_noidle(&pdev->dev);
		pm_runtime_allow(&pdev->dev);

		if (cfg->has_alt) {
			struct uart_hsu_port *alt_uport;
			struct hsu_port_cfg *alt_cfg =
				hsu_port_func_cfg + cfg->alt;
			int alt_index = alt_cfg->index;

			phsu->configs[alt_index] = alt_cfg;
			alt_uport = phsu->port + alt_index;
			memcpy(alt_uport, uport, sizeof(*uport));
			alt_uport->port.line = alt_index;
			strcpy(alt_uport->name, alt_cfg->name);
			alt_uport->index = alt_index;
			if (hsu_dma_enable & (1<<alt_index))
				alt_uport->use_dma = 1;
			else
				alt_uport->use_dma = 0;
			if (alt_cfg->hw_init)
				alt_cfg->hw_init(alt_index);
			alt_uport->workqueue =
				create_singlethread_workqueue(alt_uport->name);
			INIT_WORK(&alt_uport->work, serial_hsu_work);
			alt_uport->qcirc.buf = (char *)alt_uport->qbuf;
			spin_lock_init(&alt_uport->cl_lock);
			ret = request_irq(pdev->irq, port_irq, IRQF_SHARED,
						alt_uport->name, alt_uport);
			if (ret) {
				dev_err(&pdev->dev, "can not get IRQ\n");
				goto err_disable;
			}
			if (alt_cfg->type == debug_port) {
				serial_hsu_reg.cons = SERIAL_HSU_CONSOLE;
				serial_hsu_reg.cons->index = alt_index;
			} else
				serial_hsu_reg.cons = NULL;
			uart_add_one_port(&serial_hsu_reg, &alt_uport->port);
		}
	}

	return 0;
err_disable:
	pci_disable_device(pdev);
	return ret;
}

static void serial_hsu_remove(struct pci_dev *pdev)
{
	void *priv = pci_get_drvdata(pdev);
	struct uart_hsu_port *up;

	if (!priv)
		return;

	pm_runtime_forbid(&pdev->dev);
	pm_runtime_get_noresume(&pdev->dev);

	/* For port 0/1/2, priv is the address of uart_hsu_port */
	if ((pdev->device != 0x081E) && (pdev->device != 0x08FF)) {
		up = priv;
		uart_remove_one_port(&serial_hsu_reg, &up->port);
	}

	pci_set_drvdata(pdev, NULL);
	free_irq(pdev->irq, priv);
	pci_disable_device(pdev);
}

DEFINE_PCI_DEVICE_TABLE(hsu_pci_ids) = {
	{ PCI_VDEVICE(INTEL, 0x081B), hsu_port0 },
	{ PCI_VDEVICE(INTEL, 0x081C), hsu_port1 },
	{ PCI_VDEVICE(INTEL, 0x081D), hsu_port2 },
	{ PCI_VDEVICE(INTEL, 0x081E), hsu_dma },

	/* Cloverview support */
	{ PCI_VDEVICE(INTEL, 0x08FC), hsu_port0 },
	{ PCI_VDEVICE(INTEL, 0x08FD), hsu_port1 },
	{ PCI_VDEVICE(INTEL, 0x08FE), hsu_port2 },
	{ PCI_VDEVICE(INTEL, 0x08FF), hsu_dma },
	{},
};

static struct pci_driver hsu_pci_driver = {
	.name =		"HSU serial",
	.id_table =	hsu_pci_ids,
	.probe =	serial_hsu_probe,
	.remove =	serial_hsu_remove,
	.suspend =	serial_hsu_suspend,
	.resume	=	serial_hsu_resume,
	.driver = {
		.pm = &serial_hsu_pm_ops,
	},
};

static int __init hsu_pci_init(void)
{
	int ret;

	ret = uart_register_driver(&serial_hsu_reg);
	if (ret)
		return ret;

	hsu_debugfs_init(phsu);
	return pci_register_driver(&hsu_pci_driver);
}

static void __exit hsu_pci_exit(void)
{
	pci_unregister_driver(&hsu_pci_driver);
	uart_unregister_driver(&serial_hsu_reg);
	hsu_debugfs_remove(phsu);
}

module_init(hsu_pci_init);
module_exit(hsu_pci_exit);

MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:medfield-hsu");
