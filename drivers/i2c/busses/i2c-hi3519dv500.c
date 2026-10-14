// SPDX-License-Identifier: GPL-2.0-only
/*
 * I2C bus driver for the HiSilicon Hi3519DV500 SoC
 *
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023.
 * Copyright (c) 2026 Jun Yan.
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/property.h>
#include <linux/reset.h>
#include <linux/scatterlist.h>
#include <linux/spinlock.h>

/*
 * I2C register offsets
 */
#define HI3519DV500_I2C_GLB			0x0000
#define HI3519DV500_I2C_SCL_H			0x0004
#define HI3519DV500_I2C_SCL_L			0x0008
#define HI3519DV500_I2C_DATA1			0x0010
#define HI3519DV500_I2C_TXF			0x0020
#define HI3519DV500_I2C_RXF			0x0024
#define HI3519DV500_I2C_CMD_BASE		0x0030
#define HI3519DV500_I2C_LOOP1			0x00b0
#define HI3519DV500_I2C_DST1			0x00b4
#define HI3519DV500_I2C_TX_WATER		0x00c8
#define HI3519DV500_I2C_RX_WATER		0x00cc
#define HI3519DV500_I2C_CTRL1			0x00d0
#define HI3519DV500_I2C_CTRL2			0x00d4
#define HI3519DV500_I2C_STAT			0x00d8
#define HI3519DV500_I2C_INTR_RAW		0x00e0
#define HI3519DV500_I2C_INTR_EN			0x00e4
#define HI3519DV500_I2C_INTR_STAT		0x00e8

/* Global config register */
#define HI3519DV500_I2C_GLB_EN			BIT(0)
#define HI3519DV500_I2C_GLB_SDA_HOLD_MASK	GENMASK(23, 8)
#define HI3519DV500_I2C_GLB_SDA_HOLD_SHIFT	8
#define HI3519DV500_I2C_SDA_HOLD_DEFAULT	0xa

/* Command sequence registers, offset n = 0..31 */
#define HI3519DV500_I2C_CMD_EXIT		0x0
#define HI3519DV500_I2C_CMD_TX_S			0x1
#define HI3519DV500_I2C_CMD_TX_D1_2		0x4
#define HI3519DV500_I2C_CMD_TX_D1_1		0x5
#define HI3519DV500_I2C_CMD_TX_FIFO		0x9
#define HI3519DV500_I2C_CMD_RX_FIFO		0x12
#define HI3519DV500_I2C_CMD_RX_ACK		0x13
#define HI3519DV500_I2C_CMD_IGN_ACK		0x15
#define HI3519DV500_I2C_CMD_TX_ACK		0x16
#define HI3519DV500_I2C_CMD_TX_NACK		0x17
#define HI3519DV500_I2C_CMD_JMP1			0x18
#define HI3519DV500_I2C_CMD_UP_TXF		0x1d
#define HI3519DV500_I2C_CMD_TX_RS		0x1e
#define HI3519DV500_I2C_CMD_TX_P			0x1f

/* Control register 1 */
#define HI3519DV500_I2C_CTRL1_CMD_START		BIT(0)
#define HI3519DV500_I2C_CTRL1_DMA_OP_MASK	GENMASK(9, 8)
#define HI3519DV500_I2C_CTRL1_DMA_W		BIT(9)
#define HI3519DV500_I2C_CTRL1_DMA_R		GENMASK(9, 8)

/* Control register 2 (bus rescue) */
#define HI3519DV500_I2C_CTRL2_GPIO_MODE		BIT(8)
#define HI3519DV500_I2C_CTRL2_FORCE_SCL_OEN	BIT(4)
#define HI3519DV500_I2C_CTRL2_FORCE_SDA_OEN	BIT(0)
#define HI3519DV500_I2C_CTRL2_CHECK_SDA_IN	BIT(16)

/* Status register */
#define HI3519DV500_I2C_STAT_RXF_NOE		BIT(16)
#define HI3519DV500_I2C_STAT_TXF_NOF		BIT(19)

/* Interrupt registers */
#define HI3519DV500_I2C_INTR_ABORT		(BIT(0) | BIT(11))
#define HI3519DV500_I2C_INTR_RX			BIT(2)
#define HI3519DV500_I2C_INTR_TX			BIT(4)
#define HI3519DV500_I2C_INTR_CMD_DONE		BIT(12)
#define HI3519DV500_I2C_INTR_USE		(HI3519DV500_I2C_INTR_ABORT | \
					 HI3519DV500_I2C_INTR_RX | \
					 HI3519DV500_I2C_INTR_TX | \
					 HI3519DV500_I2C_INTR_CMD_DONE)
#define HI3519DV500_I2C_INTR_ALL		GENMASK(31, 0)

#define HI3519DV500_I2C_DEFAULT_FREQ		100000
#define HI3519DV500_I2C_TXF_DEPTH		64
#define HI3519DV500_I2C_RXF_DEPTH		64
#define HI3519DV500_I2C_TXF_WATER		32
#define HI3519DV500_I2C_RXF_WATER		32
#define HI3519DV500_I2C_TIMEOUT_US		(10 * USEC_PER_MSEC)
#define HI3519DV500_I2C_SLEEP_US		50

/*
 * Message length range served by DMA.  The DMA handshake moves one
 * byte at a time, so short transactions are faster through the FIFO
 * interrupts while longer ones offload the CPU.
 */
#define HI3519DV500_I2C_DMA_MIN_LEN		5
#define HI3519DV500_I2C_DMA_MAX_LEN		0xffc

/**
 * struct hi3519dv500_i2c_dev - private data of the Hi3519DV500 I2C controller
 *
 * @dev: device reference
 * @base: virtual base address of the controller
 * @clk: functional clock of the controller
 * @irq: interrupt number, negative value means polling mode
 * @freq: desired bus frequency in Hz
 * @adap: I2C adapter owned by this controller
 * @msg: current message being transferred
 * @msg_num: total number of messages in the transfer
 * @msg_idx: index of the current message
 * @msg_buf_ptr: position inside the current message buffer
 * @msg_complete: transfer completion
 * @lock: lock serializing transfers and protecting the registers
 * @status: result of the last transfer
 * @phys_addr: physical base address of the controller
 * @chan_rx: optional receive DMA channel, NULL means FIFO mode
 * @chan_tx: optional transmit DMA channel, NULL means FIFO mode
 * @dma_complete: completion signaled by the DMA callback
 */
struct hi3519dv500_i2c_dev {
	struct device *dev;
	void __iomem *base;
	struct clk *clk;
	int irq;

	unsigned int freq;
	struct i2c_adapter adap;
	struct i2c_msg *msg;
	unsigned int msg_num;
	unsigned int msg_idx;
	unsigned int msg_buf_ptr;
	struct completion msg_complete;
	spinlock_t lock;
	int status;
	phys_addr_t phys_addr;
	struct dma_chan *chan_rx;
	struct dma_chan *chan_tx;
	struct completion dma_complete;
};

static inline void hi3519dv500_i2c_disable(const struct hi3519dv500_i2c_dev *i2c)
{
	u32 val;

	val = readl(i2c->base + HI3519DV500_I2C_GLB);
	val &= ~HI3519DV500_I2C_GLB_EN;
	writel(val, i2c->base + HI3519DV500_I2C_GLB);
}

static inline void hi3519dv500_i2c_enable(const struct hi3519dv500_i2c_dev *i2c)
{
	u32 val;

	val = readl(i2c->base + HI3519DV500_I2C_GLB);
	val |= HI3519DV500_I2C_GLB_EN;
	writel(val, i2c->base + HI3519DV500_I2C_GLB);
}

static inline void hi3519dv500_i2c_cfg_irq(const struct hi3519dv500_i2c_dev *i2c,
					    u32 mask)
{
	writel(mask, i2c->base + HI3519DV500_I2C_INTR_EN);
}

static void hi3519dv500_i2c_disable_irq(const struct hi3519dv500_i2c_dev *i2c,
					 u32 mask)
{
	u32 val;

	val = readl(i2c->base + HI3519DV500_I2C_INTR_EN);
	val &= ~mask;
	writel(val, i2c->base + HI3519DV500_I2C_INTR_EN);
}

static u32 hi3519dv500_i2c_clr_irq(const struct hi3519dv500_i2c_dev *i2c)
{
	u32 val;

	val = readl(i2c->base + HI3519DV500_I2C_INTR_STAT);
	writel(HI3519DV500_I2C_INTR_ALL, i2c->base + HI3519DV500_I2C_INTR_RAW);

	return val;
}

/*
 * Drive SDA/SCL low and high for up to nine clock cycles to release a
 * slave device that stretches the bus.
 */
static void hi3519dv500_i2c_rescue(const struct hi3519dv500_i2c_dev *i2c)
{
	u32 val;
	unsigned int time_cnt;
	int index;

	hi3519dv500_i2c_disable(i2c);
	hi3519dv500_i2c_cfg_irq(i2c, 0);
	hi3519dv500_i2c_clr_irq(i2c);

	val = HI3519DV500_I2C_CTRL2_GPIO_MODE |
	      HI3519DV500_I2C_CTRL2_FORCE_SCL_OEN |
	      HI3519DV500_I2C_CTRL2_FORCE_SDA_OEN;
	writel(val, i2c->base + HI3519DV500_I2C_CTRL2);

	time_cnt = 0;
	do {
		for (index = 0; index < 9; index++) {
			/* SCL low, SDA released */
			writel(HI3519DV500_I2C_CTRL2_GPIO_MODE | 1,
			       i2c->base + HI3519DV500_I2C_CTRL2);
			udelay(5);

			/* SCL high, SDA released */
			writel(HI3519DV500_I2C_CTRL2_GPIO_MODE |
			       HI3519DV500_I2C_CTRL2_FORCE_SCL_OEN |
			       HI3519DV500_I2C_CTRL2_FORCE_SDA_OEN,
			       i2c->base + HI3519DV500_I2C_CTRL2);
			udelay(5);
		}

		time_cnt++;
		if (time_cnt > HI3519DV500_I2C_TIMEOUT_US / HI3519DV500_I2C_SLEEP_US) {
			dev_err(i2c->dev, "bus rescue timeout\n");
			goto disable_rescue;
		}

		val = readl(i2c->base + HI3519DV500_I2C_CTRL2);
	} while (!(val & HI3519DV500_I2C_CTRL2_CHECK_SDA_IN));

	writel(HI3519DV500_I2C_CTRL2_GPIO_MODE | 1,
	       i2c->base + HI3519DV500_I2C_CTRL2);

	/* STOP condition */
	writel(HI3519DV500_I2C_CTRL2_GPIO_MODE |
	       HI3519DV500_I2C_CTRL2_FORCE_SCL_OEN,
	       i2c->base + HI3519DV500_I2C_CTRL2);
	udelay(10);

	writel(HI3519DV500_I2C_CTRL2_GPIO_MODE |
	       HI3519DV500_I2C_CTRL2_FORCE_SCL_OEN |
	       HI3519DV500_I2C_CTRL2_FORCE_SDA_OEN,
	       i2c->base + HI3519DV500_I2C_CTRL2);

disable_rescue:
	writel(HI3519DV500_I2C_CTRL2_FORCE_SCL_OEN | 1,
	       i2c->base + HI3519DV500_I2C_CTRL2);
}

static int hi3519dv500_i2c_wait_rx_noempty(const struct hi3519dv500_i2c_dev *i2c)
{
	u32 val;
	int ret;

	ret = readl_poll_timeout(i2c->base + HI3519DV500_I2C_STAT, val,
				 val & HI3519DV500_I2C_STAT_RXF_NOE,
				 HI3519DV500_I2C_SLEEP_US,
				 HI3519DV500_I2C_TIMEOUT_US);
	if (ret) {
		dev_err(i2c->dev, "rx FIFO empty, RIS: 0x%x\n",
			readl(i2c->base + HI3519DV500_I2C_INTR_RAW));
		hi3519dv500_i2c_rescue(i2c);
	}

	return ret;
}

static int hi3519dv500_i2c_wait_tx_nofull(const struct hi3519dv500_i2c_dev *i2c)
{
	u32 val;
	int ret;

	ret = readl_poll_timeout(i2c->base + HI3519DV500_I2C_STAT, val,
				 val & HI3519DV500_I2C_STAT_TXF_NOF,
				 HI3519DV500_I2C_SLEEP_US,
				 HI3519DV500_I2C_TIMEOUT_US);
	if (ret) {
		dev_err(i2c->dev, "tx FIFO full, RIS: 0x%x\n",
			readl(i2c->base + HI3519DV500_I2C_INTR_RAW));
		hi3519dv500_i2c_rescue(i2c);
	}

	return ret;
}

static int hi3519dv500_i2c_wait_idle(const struct hi3519dv500_i2c_dev *i2c)
{
	u32 val;
	int ret;

	ret = readl_poll_timeout(i2c->base + HI3519DV500_I2C_INTR_RAW, val,
				 val & HI3519DV500_I2C_INTR_CMD_DONE,
				 HI3519DV500_I2C_SLEEP_US,
				 HI3519DV500_I2C_TIMEOUT_US);
	if (ret == 0 && (val & HI3519DV500_I2C_INTR_ABORT)) {
		dev_err(i2c->dev, "transfer aborted, RIS: 0x%x\n", val);
		return -EIO;
	}
	if (ret) {
		dev_err(i2c->dev, "wait idle timeout, RIS: 0x%x, SR: 0x%x\n",
			val, readl(i2c->base + HI3519DV500_I2C_STAT));
		hi3519dv500_i2c_rescue(i2c);
	}

	return ret;
}

static void hi3519dv500_i2c_set_freq(struct hi3519dv500_i2c_dev *i2c)
{
	unsigned int freq = i2c->freq;
	unsigned int clk_rate = clk_get_rate(i2c->clk);
	unsigned int max_freq = clk_rate >> 1;
	u32 val;

	if (freq > max_freq)
		freq = max_freq;

	if (!freq) {
		dev_err(i2c->dev, "clock rate is zero\n");
		return;
	}

	if (freq <= I2C_MAX_STANDARD_MODE_FREQ) {
		/* Standard mode, duty cycle 50% */
		val = clk_rate / (freq * 2);
		writel(val, i2c->base + HI3519DV500_I2C_SCL_H);
		writel(val, i2c->base + HI3519DV500_I2C_SCL_L);
	} else {
		/* Fast mode, duty cycle 36%/64% */
		val = ((clk_rate / 100) * 36) / freq;
		writel(val, i2c->base + HI3519DV500_I2C_SCL_H);
		val = ((clk_rate / 100) * 64) / freq;
		writel(val, i2c->base + HI3519DV500_I2C_SCL_L);
	}

	val = readl(i2c->base + HI3519DV500_I2C_GLB);
	val &= ~HI3519DV500_I2C_GLB_SDA_HOLD_MASK;
	val |= HI3519DV500_I2C_SDA_HOLD_DEFAULT
	       << HI3519DV500_I2C_GLB_SDA_HOLD_SHIFT;
	writel(val, i2c->base + HI3519DV500_I2C_GLB);
}

static inline void hi3519dv500_i2c_set_water(const struct hi3519dv500_i2c_dev *i2c)
{
	writel(HI3519DV500_I2C_TXF_WATER, i2c->base + HI3519DV500_I2C_TX_WATER);
	writel(HI3519DV500_I2C_RXF_WATER, i2c->base + HI3519DV500_I2C_RX_WATER);
}

/* Program the bus frequency and FIFO watermarks, keep the controller off. */
static void hi3519dv500_i2c_hw_init(struct hi3519dv500_i2c_dev *i2c)
{
	hi3519dv500_i2c_disable(i2c);
	hi3519dv500_i2c_disable_irq(i2c, HI3519DV500_I2C_INTR_ALL);
	hi3519dv500_i2c_set_freq(i2c);
	hi3519dv500_i2c_set_water(i2c);
}

/*
 * hi3519dv500_i2c_set_addr - program the slave address into DATA1
 *
 * 7-bit address: DATA1 = addr << 1 | rw
 * 10-bit address: first byte 11110xx0, second byte is the remaining
 * 8 bits, the read flag is carried by a repeated start.
 */
static void hi3519dv500_i2c_set_addr(const struct hi3519dv500_i2c_dev *i2c)
{
	struct i2c_msg *msg = i2c->msg;
	u16 addr;

	if (msg->flags & I2C_M_TEN) {
		addr = ((msg->addr & 0x300) << 1) | 0xf000;
		if (msg->flags & I2C_M_RD)
			addr |= BIT(8);
		addr |= msg->addr & 0xff;
	} else {
		addr = (msg->addr & 0x7f) << 1;
		if (msg->flags & I2C_M_RD)
			addr |= 1;
	}

	writel(addr, i2c->base + HI3519DV500_I2C_DATA1);
}

static inline void hi3519dv500_i2c_start_cmd(const struct hi3519dv500_i2c_dev *i2c)
{
	u32 val;

	val = readl(i2c->base + HI3519DV500_I2C_CTRL1);
	val |= HI3519DV500_I2C_CTRL1_CMD_START;
	writel(val, i2c->base + HI3519DV500_I2C_CTRL1);
}

static void hi3519dv500_i2c_cmdreg_set(const struct hi3519dv500_i2c_dev *i2c,
					u32 cmd, unsigned int *offset)
{
	writel(cmd, i2c->base + HI3519DV500_I2C_CMD_BASE + *offset * 4);
	(*offset)++;
}

/*
 * hi3519dv500_i2c_cfg_cmd - build the hardware command sequence
 *
 * The controller executes the command sequence autonomously once
 * started; the CPU only feeds or drains the FIFOs afterwards.
 *
 * Read transfers loop back to DST1/LOOP1 to clock the first len - 2
 * bytes out with an ACK, then the last byte is NACKed. For a
 * zero-length read (SMBus quick) the loop is skipped and only the
 * NACK read remains.
 */
static void hi3519dv500_i2c_cfg_cmd(struct hi3519dv500_i2c_dev *i2c)
{
	struct i2c_msg *msg = i2c->msg;
	unsigned int offset = 0;

	if (i2c->msg_idx == 0)
		hi3519dv500_i2c_cmdreg_set(i2c, HI3519DV500_I2C_CMD_TX_S, &offset);
	else
		hi3519dv500_i2c_cmdreg_set(i2c, HI3519DV500_I2C_CMD_TX_RS, &offset);

	if (msg->flags & I2C_M_TEN) {
		if (i2c->msg_idx == 0)
			hi3519dv500_i2c_cmdreg_set(i2c, HI3519DV500_I2C_CMD_TX_D1_2,
						   &offset);
		hi3519dv500_i2c_cmdreg_set(i2c, HI3519DV500_I2C_CMD_TX_D1_1, &offset);
	} else {
		hi3519dv500_i2c_cmdreg_set(i2c, HI3519DV500_I2C_CMD_TX_D1_1, &offset);
	}

	if (msg->flags & I2C_M_IGNORE_NAK)
		hi3519dv500_i2c_cmdreg_set(i2c, HI3519DV500_I2C_CMD_IGN_ACK, &offset);
	else
		hi3519dv500_i2c_cmdreg_set(i2c, HI3519DV500_I2C_CMD_RX_ACK, &offset);

	if (msg->flags & I2C_M_RD) {
		if (msg->len >= 2) {
			writel(offset, i2c->base + HI3519DV500_I2C_DST1);
			writel(msg->len - 2, i2c->base + HI3519DV500_I2C_LOOP1);
			hi3519dv500_i2c_cmdreg_set(i2c, HI3519DV500_I2C_CMD_RX_FIFO, &offset);
			hi3519dv500_i2c_cmdreg_set(i2c, HI3519DV500_I2C_CMD_TX_ACK, &offset);
			hi3519dv500_i2c_cmdreg_set(i2c, HI3519DV500_I2C_CMD_JMP1, &offset);
		}
		hi3519dv500_i2c_cmdreg_set(i2c, HI3519DV500_I2C_CMD_RX_FIFO, &offset);
		hi3519dv500_i2c_cmdreg_set(i2c, HI3519DV500_I2C_CMD_TX_NACK, &offset);
	} else {
		writel(offset, i2c->base + HI3519DV500_I2C_DST1);
		writel(msg->len ? msg->len - 1 : 0, i2c->base + HI3519DV500_I2C_LOOP1);
		hi3519dv500_i2c_cmdreg_set(i2c, HI3519DV500_I2C_CMD_UP_TXF, &offset);
		hi3519dv500_i2c_cmdreg_set(i2c, HI3519DV500_I2C_CMD_TX_FIFO, &offset);

		if (msg->flags & I2C_M_IGNORE_NAK)
			hi3519dv500_i2c_cmdreg_set(i2c, HI3519DV500_I2C_CMD_IGN_ACK, &offset);
		else
			hi3519dv500_i2c_cmdreg_set(i2c, HI3519DV500_I2C_CMD_RX_ACK, &offset);

		hi3519dv500_i2c_cmdreg_set(i2c, HI3519DV500_I2C_CMD_JMP1, &offset);
	}

	if (i2c->msg_idx == (i2c->msg_num - 1) || (msg->flags & I2C_M_STOP))
		hi3519dv500_i2c_cmdreg_set(i2c, HI3519DV500_I2C_CMD_TX_P, &offset);

	hi3519dv500_i2c_cmdreg_set(i2c, HI3519DV500_I2C_CMD_EXIT, &offset);
}

/* Make sure a previous transfer finished, then restart from a known state. */
static void hi3519dv500_i2c_prepare_xfer(struct hi3519dv500_i2c_dev *i2c)
{
	u32 val;

	val = readl(i2c->base + HI3519DV500_I2C_GLB);
	if (val & HI3519DV500_I2C_GLB_EN) {
		hi3519dv500_i2c_wait_idle(i2c);
		hi3519dv500_i2c_disable(i2c);
	}

	hi3519dv500_i2c_set_freq(i2c);
}

/*
 * Transfer one message in polling mode. Used when no interrupt is
 * wired, for zero-length messages and for SMBus quick detection.
 */
static int hi3519dv500_i2c_polling_xfer_one_msg(struct hi3519dv500_i2c_dev *i2c)
{
	struct i2c_msg *msg = i2c->msg;
	int ret;

	hi3519dv500_i2c_prepare_xfer(i2c);
	hi3519dv500_i2c_enable(i2c);
	hi3519dv500_i2c_clr_irq(i2c);
	hi3519dv500_i2c_set_addr(i2c);
	hi3519dv500_i2c_cfg_cmd(i2c);
	hi3519dv500_i2c_start_cmd(i2c);

	i2c->msg_buf_ptr = 0;

	if (msg->flags & I2C_M_RD) {
		while (i2c->msg_buf_ptr < msg->len) {
			ret = hi3519dv500_i2c_wait_rx_noempty(i2c);
			if (ret)
				goto end;

			msg->buf[i2c->msg_buf_ptr] =
				readl(i2c->base + HI3519DV500_I2C_RXF);
			i2c->msg_buf_ptr++;
		}
	} else {
		while (i2c->msg_buf_ptr < msg->len) {
			ret = hi3519dv500_i2c_wait_tx_nofull(i2c);
			if (ret)
				goto end;

			writel(msg->buf[i2c->msg_buf_ptr],
			       i2c->base + HI3519DV500_I2C_TXF);
			i2c->msg_buf_ptr++;
		}
	}

	ret = hi3519dv500_i2c_wait_idle(i2c);
end:
	hi3519dv500_i2c_disable(i2c);

	return ret;
}

static irqreturn_t hi3519dv500_i2c_isr(int irq, void *dev_id)
{
	struct hi3519dv500_i2c_dev *i2c = dev_id;
	unsigned int irq_status;
	struct i2c_msg *msg = i2c->msg;

	spin_lock(&i2c->lock);

	irq_status = hi3519dv500_i2c_clr_irq(i2c);

	if (!irq_status)
		goto end;

	if (irq_status & HI3519DV500_I2C_INTR_ABORT) {
		dev_err(i2c->dev, "irq handle abort, RIS: 0x%x\n", irq_status);
		i2c->status = -EIO;
		hi3519dv500_i2c_disable_irq(i2c, HI3519DV500_I2C_INTR_ALL);
		complete(&i2c->msg_complete);
		goto end;
	}

	if (msg->flags & I2C_M_RD) {
		while ((readl(i2c->base + HI3519DV500_I2C_STAT) &
			HI3519DV500_I2C_STAT_RXF_NOE) &&
		       i2c->msg_buf_ptr < msg->len) {
			msg->buf[i2c->msg_buf_ptr] =
				readl(i2c->base + HI3519DV500_I2C_RXF);
			i2c->msg_buf_ptr++;
		}
	} else {
		while ((readl(i2c->base + HI3519DV500_I2C_STAT) &
			HI3519DV500_I2C_STAT_TXF_NOF) &&
		       i2c->msg_buf_ptr < msg->len) {
			writel(msg->buf[i2c->msg_buf_ptr],
			       i2c->base + HI3519DV500_I2C_TXF);
			i2c->msg_buf_ptr++;
		}
	}

	if (i2c->msg_buf_ptr >= msg->len)
		hi3519dv500_i2c_disable_irq(i2c, HI3519DV500_I2C_INTR_TX |
						    HI3519DV500_I2C_INTR_RX);

	if (irq_status & HI3519DV500_I2C_INTR_CMD_DONE) {
		i2c->status = 0;
		hi3519dv500_i2c_disable_irq(i2c, HI3519DV500_I2C_INTR_ALL);
		complete(&i2c->msg_complete);
	}

end:
	spin_unlock(&i2c->lock);

	return IRQ_HANDLED;
}

static int hi3519dv500_i2c_interrupt_xfer_one_msg(struct hi3519dv500_i2c_dev *i2c)
{
	struct i2c_msg *msg = i2c->msg;
	unsigned long timeout;
	unsigned long flags;
	int ret;

	reinit_completion(&i2c->msg_complete);
	i2c->msg_buf_ptr = 0;
	i2c->status = -EIO;

	spin_lock_irqsave(&i2c->lock, flags);
	hi3519dv500_i2c_prepare_xfer(i2c);
	hi3519dv500_i2c_enable(i2c);
	hi3519dv500_i2c_clr_irq(i2c);
	if (msg->flags & I2C_M_RD)
		hi3519dv500_i2c_cfg_irq(i2c, HI3519DV500_I2C_INTR_USE &
					     ~HI3519DV500_I2C_INTR_TX);
	else
		hi3519dv500_i2c_cfg_irq(i2c, HI3519DV500_I2C_INTR_USE &
					     ~HI3519DV500_I2C_INTR_RX);

	hi3519dv500_i2c_set_addr(i2c);
	hi3519dv500_i2c_cfg_cmd(i2c);
	hi3519dv500_i2c_start_cmd(i2c);
	spin_unlock_irqrestore(&i2c->lock, flags);

	timeout = wait_for_completion_timeout(&i2c->msg_complete,
					       i2c->adap.timeout);

	spin_lock_irqsave(&i2c->lock, flags);
	if (timeout == 0) {
		hi3519dv500_i2c_disable_irq(i2c, HI3519DV500_I2C_INTR_ALL);
		ret = -ETIMEDOUT;
		dev_err(i2c->dev, "%s timeout\n",
			msg->flags & I2C_M_RD ? "rx" : "tx");
	} else {
		ret = i2c->status;
	}

	hi3519dv500_i2c_disable(i2c);
	spin_unlock_irqrestore(&i2c->lock, flags);

	return ret;
}

static void hi3519dv500_i2c_dma_callback(void *param)
{
	struct hi3519dv500_i2c_dev *i2c = param;

	complete(&i2c->dma_complete);
}

/*
 * Transfer one message through the DMA engine.
 *
 * The hardware command sequence drives the bus protocol exactly as in
 * FIFO mode, while the DMA handshake drains or feeds the FIFOs.  The
 * completion fires when the DMA side has moved the whole buffer, the
 * command engine still needs to reach the EXIT command afterwards.
 */
static int hi3519dv500_i2c_dma_xfer_one_msg(struct hi3519dv500_i2c_dev *i2c)
{
	struct i2c_msg *msg = i2c->msg;
	bool rd = msg->flags & I2C_M_RD;
	struct dma_chan *chan = rd ? i2c->chan_rx : i2c->chan_tx;
	enum dma_data_direction dir = rd ? DMA_FROM_DEVICE : DMA_TO_DEVICE;
	struct dma_async_tx_descriptor *desc;
	struct scatterlist sg;
	dma_addr_t dma_addr;
	unsigned long timeout;
	u32 val;
	int ret;

	hi3519dv500_i2c_prepare_xfer(i2c);

	/* The DMA handshake moves one byte per request. */
	if (rd)
		writel(0, i2c->base + HI3519DV500_I2C_RX_WATER);
	else
		writel(1, i2c->base + HI3519DV500_I2C_TX_WATER);

	hi3519dv500_i2c_enable(i2c);
	hi3519dv500_i2c_clr_irq(i2c);
	hi3519dv500_i2c_set_addr(i2c);
	hi3519dv500_i2c_cfg_cmd(i2c);

	dma_addr = dma_map_single(i2c->dev, msg->buf, msg->len, dir);
	if (dma_mapping_error(i2c->dev, dma_addr)) {
		ret = -ENOMEM;
		goto out_disable;
	}

	sg_init_table(&sg, 1);
	sg_dma_address(&sg) = dma_addr;
	sg_dma_len(&sg) = msg->len;

	desc = dmaengine_prep_slave_sg(chan, &sg, 1,
				       rd ? DMA_DEV_TO_MEM : DMA_MEM_TO_DEV,
				       DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!desc) {
		ret = -ENOMEM;
		goto out_unmap;
	}

	reinit_completion(&i2c->dma_complete);
	desc->callback = hi3519dv500_i2c_dma_callback;
	desc->callback_param = i2c;
	dmaengine_submit(desc);
	dma_async_issue_pending(chan);

	/* Start the command engine with the DMA handshake enabled. */
	val = readl(i2c->base + HI3519DV500_I2C_CTRL1);
	val &= ~HI3519DV500_I2C_CTRL1_DMA_OP_MASK;
	val |= (rd ? HI3519DV500_I2C_CTRL1_DMA_R : HI3519DV500_I2C_CTRL1_DMA_W) |
	       HI3519DV500_I2C_CTRL1_CMD_START;
	writel(val, i2c->base + HI3519DV500_I2C_CTRL1);

	timeout = wait_for_completion_timeout(&i2c->dma_complete,
					      i2c->adap.timeout);
	if (timeout == 0) {
		dmaengine_terminate_sync(chan);
		dev_err(i2c->dev, "DMA %s timeout\n", rd ? "rx" : "tx");
		ret = -ETIMEDOUT;
	} else {
		ret = hi3519dv500_i2c_wait_idle(i2c);
	}

out_unmap:
	dma_unmap_single(i2c->dev, dma_addr, msg->len, dir);
out_disable:
	hi3519dv500_i2c_disable(i2c);

	return ret;
}

static int hi3519dv500_i2c_xfer(struct i2c_adapter *adap,
				 struct i2c_msg *msgs, int num)
{
	struct hi3519dv500_i2c_dev *i2c = i2c_get_adapdata(adap);
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&i2c->lock, flags);
	i2c->msg = msgs;
	i2c->msg_num = num;
	i2c->msg_idx = 0;
	spin_unlock_irqrestore(&i2c->lock, flags);

	/*
	 * Zero-length messages are used for SMBus quick detection. The
	 * interrupt engine cannot complete them (no data phase happens),
	 * so always poll them.
	 */
	while (i2c->msg_idx < i2c->msg_num) {
		struct i2c_msg *msg = &msgs[i2c->msg_idx];
		struct dma_chan *chan = (msg->flags & I2C_M_RD) ?
					i2c->chan_rx : i2c->chan_tx;

		if (chan && msg->len >= HI3519DV500_I2C_DMA_MIN_LEN &&
		    msg->len <= HI3519DV500_I2C_DMA_MAX_LEN)
			ret = hi3519dv500_i2c_dma_xfer_one_msg(i2c);
		else if (i2c->irq >= 0 && msg->len)
			ret = hi3519dv500_i2c_interrupt_xfer_one_msg(i2c);
		else
			ret = hi3519dv500_i2c_polling_xfer_one_msg(i2c);

		if (ret)
			break;

		i2c->msg++;
		i2c->msg_idx++;
	}

	spin_lock_irqsave(&i2c->lock, flags);
	if (!ret || i2c->msg_idx > 0)
		ret = i2c->msg_idx;
	spin_unlock_irqrestore(&i2c->lock, flags);

	return ret;
}

static u32 hi3519dv500_i2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_10BIT_ADDR | I2C_FUNC_PROTOCOL_MANGLING |
	       I2C_FUNC_SMBUS_WORD_DATA | I2C_FUNC_SMBUS_BYTE_DATA |
	       I2C_FUNC_SMBUS_BYTE | I2C_FUNC_SMBUS_I2C_BLOCK;
}

static const struct i2c_algorithm hi3519dv500_i2c_algo = {
	.master_xfer = hi3519dv500_i2c_xfer,
	.functionality = hi3519dv500_i2c_func,
};

static void hi3519dv500_i2c_dma_release(void *data)
{
	struct hi3519dv500_i2c_dev *i2c = data;

	if (i2c->chan_tx)
		dma_release_channel(i2c->chan_tx);
	if (i2c->chan_rx)
		dma_release_channel(i2c->chan_rx);
}

static void hi3519dv500_i2c_dma_drop(struct hi3519dv500_i2c_dev *i2c)
{
	if (i2c->chan_tx) {
		dma_release_channel(i2c->chan_tx);
		i2c->chan_tx = NULL;
	}
	if (i2c->chan_rx) {
		dma_release_channel(i2c->chan_rx);
		i2c->chan_rx = NULL;
	}
}

/*
 * Acquire the optional receive and transmit DMA channels.  Controllers
 * without a dmas property keep both channels NULL and the transfers
 * stay in FIFO mode.
 */
static int hi3519dv500_i2c_dma_setup(struct hi3519dv500_i2c_dev *i2c)
{
	struct dma_slave_config cfg = { };
	int ret;

	i2c->chan_rx = dma_request_chan(i2c->dev, "rx");
	if (IS_ERR(i2c->chan_rx)) {
		ret = PTR_ERR(i2c->chan_rx);
		i2c->chan_rx = NULL;
		return ret == -EPROBE_DEFER ? ret : 0;
	}

	i2c->chan_tx = dma_request_chan(i2c->dev, "tx");
	if (IS_ERR(i2c->chan_tx)) {
		ret = PTR_ERR(i2c->chan_tx);
		i2c->chan_tx = NULL;
		goto err_no_txchan;
	}

	cfg.direction = DMA_DEV_TO_MEM;
	cfg.src_addr = i2c->phys_addr + HI3519DV500_I2C_RXF;
	cfg.src_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	cfg.src_maxburst = 1;

	ret = dmaengine_slave_config(i2c->chan_rx, &cfg);
	if (ret)
		goto err_config;

	memset(&cfg, 0, sizeof(cfg));
	cfg.direction = DMA_MEM_TO_DEV;
	cfg.dst_addr = i2c->phys_addr + HI3519DV500_I2C_TXF;
	cfg.dst_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	cfg.dst_maxburst = 1;

	ret = dmaengine_slave_config(i2c->chan_tx, &cfg);
	if (ret)
		goto err_config;

	ret = devm_add_action_or_reset(i2c->dev,
				       hi3519dv500_i2c_dma_release, i2c);

	return ret;

err_config:
	hi3519dv500_i2c_dma_drop(i2c);

	return 0;

err_no_txchan:
	hi3519dv500_i2c_dma_drop(i2c);

	return ret == -EPROBE_DEFER ? ret : 0;
}

static int hi3519dv500_i2c_probe(struct platform_device *pdev)
{
	struct hi3519dv500_i2c_dev *i2c;
	struct reset_control *rst;
	struct resource *res;
	u32 freq;
	int irq, ret;

	i2c = devm_kzalloc(&pdev->dev, sizeof(*i2c), GFP_KERNEL);
	if (!i2c)
		return -ENOMEM;

	i2c->dev = &pdev->dev;
	i2c->irq = -1;
	spin_lock_init(&i2c->lock);
	init_completion(&i2c->msg_complete);
	init_completion(&i2c->dma_complete);
	platform_set_drvdata(pdev, i2c);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	i2c->phys_addr = res->start;
	i2c->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(i2c->base))
		return PTR_ERR(i2c->base);

	i2c->clk = devm_clk_get_enabled(&pdev->dev, NULL);
	if (IS_ERR(i2c->clk))
		return dev_err_probe(&pdev->dev, PTR_ERR(i2c->clk),
				     "cannot enable clock\n");

	/* Deassert the controller reset if one is wired up. */
	rst = devm_reset_control_get_optional_exclusive(&pdev->dev,
							"i2c_reset");
	if (IS_ERR(rst))
		return dev_err_probe(&pdev->dev, PTR_ERR(rst),
				     "cannot get reset\n");

	ret = reset_control_deassert(rst);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "cannot deassert reset\n");

	if (device_property_read_u32(&pdev->dev, "clock-frequency", &freq))
		freq = HI3519DV500_I2C_DEFAULT_FREQ;

	if (freq > I2C_MAX_FAST_MODE_FREQ) {
		dev_warn(&pdev->dev, "clock-frequency %u too high, capped to %u\n",
			 freq, I2C_MAX_FAST_MODE_FREQ);
		freq = I2C_MAX_FAST_MODE_FREQ;
	}
	i2c->freq = freq;

	irq = platform_get_irq_optional(pdev, 0);
	if (irq < 0 && irq != -ENXIO)
		return irq;

	if (irq > 0) {
		ret = devm_request_irq(&pdev->dev, irq, hi3519dv500_i2c_isr, 0,
				       dev_name(&pdev->dev), i2c);
		if (ret)
			return dev_err_probe(&pdev->dev, ret,
					     "cannot request IRQ %d\n", irq);
		i2c->irq = irq;
	}

	ret = hi3519dv500_i2c_dma_setup(i2c);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "cannot setup DMA channels\n");

	hi3519dv500_i2c_hw_init(i2c);

	i2c->adap.owner = THIS_MODULE;
	strscpy(i2c->adap.name, "hi3519dv500-i2c", sizeof(i2c->adap.name));
	i2c->adap.algo = &hi3519dv500_i2c_algo;
	i2c->adap.retries = 3;
	i2c->adap.timeout = msecs_to_jiffies(1000);
	i2c->adap.dev.parent = &pdev->dev;
	i2c->adap.dev.of_node = pdev->dev.of_node;
	i2c_set_adapdata(&i2c->adap, i2c);

	ret = devm_i2c_add_adapter(&pdev->dev, &i2c->adap);

	return ret;
}

static int hi3519dv500_i2c_suspend(struct device *dev)
{
	struct hi3519dv500_i2c_dev *i2c = dev_get_drvdata(dev);

	i2c_lock_bus(&i2c->adap, I2C_LOCK_ROOT_ADAPTER);
	clk_disable_unprepare(i2c->clk);
	i2c_unlock_bus(&i2c->adap, I2C_LOCK_ROOT_ADAPTER);

	return 0;
}

static int hi3519dv500_i2c_resume(struct device *dev)
{
	struct hi3519dv500_i2c_dev *i2c = dev_get_drvdata(dev);

	i2c_lock_bus(&i2c->adap, I2C_LOCK_ROOT_ADAPTER);
	clk_prepare_enable(i2c->clk);
	hi3519dv500_i2c_hw_init(i2c);
	i2c_unlock_bus(&i2c->adap, I2C_LOCK_ROOT_ADAPTER);

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(hi3519dv500_i2c_pm,
				hi3519dv500_i2c_suspend,
				hi3519dv500_i2c_resume);

static const struct of_device_id hi3519dv500_i2c_match[] = {
	{ .compatible = "hisilicon,hi3519dv500-i2c" },
	{ }
};
MODULE_DEVICE_TABLE(of, hi3519dv500_i2c_match);

static struct platform_driver hi3519dv500_i2c_driver = {
	.probe = hi3519dv500_i2c_probe,
	.driver = {
		.name = "hi3519dv500-i2c",
		.of_match_table = hi3519dv500_i2c_match,
		.pm = pm_sleep_ptr(&hi3519dv500_i2c_pm),
	},
};
module_platform_driver(hi3519dv500_i2c_driver);

MODULE_AUTHOR("Jun Yan <jerrysteve1101@gmail.com>");
MODULE_DESCRIPTION("HiSilicon Hi3519DV500 I2C controller driver");
MODULE_LICENSE("GPL");
