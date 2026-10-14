// SPDX-License-Identifier: GPL-2.0-only
/*
 * HiSilicon EDMAC v310 DMA driver
 *
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023.
 * Copyright (c) 2026 Jun Yan <jerrysteve1101@gmail.com>
 *
 * Ported from the HiSilicon vendor SDK to the mainline kernel.
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/dmapool.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/of_dma.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/reset.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

#include "virt-dma.h"

#define DRIVER_NAME		"hisi-edmacv310"

/* Interrupt registers */
#define EDMAC_INT_STAT		0x000
#define EDMAC_INT_TC1		0x004
#define EDMAC_INT_TC2		0x008
#define EDMAC_INT_ERR1		0x00c
#define EDMAC_INT_ERR2		0x010
#define EDMAC_INT_ERR3		0x014

#define EDMAC_INT_TC1_MASK	0x018
#define EDMAC_INT_TC2_MASK	0x01c
#define EDMAC_INT_ERR1_MASK	0x020
#define EDMAC_INT_ERR2_MASK	0x024
#define EDMAC_INT_ERR3_MASK	0x028

#define EDMAC_INT_TC1_RAW	0x600
#define EDMAC_INT_TC2_RAW	0x608
#define EDMAC_INT_ERR1_RAW	0x610
#define EDMAC_INT_ERR2_RAW	0x618
#define EDMAC_INT_ERR3_RAW	0x620

#define EDMAC_CH_STAT		0x690

/* Per channel registers */
#define EDMAC_CX_LLI_L(cn)	(0x800 + (cn) * 0x40)
#define EDMAC_CX_LLI_H(cn)	(0x804 + (cn) * 0x40)
#define EDMAC_CX_CNT0(cn)	(0x81c + (cn) * 0x40)
#define EDMAC_CX_SRC_L(cn)	(0x820 + (cn) * 0x40)
#define EDMAC_CX_SRC_H(cn)	(0x824 + (cn) * 0x40)
#define EDMAC_CX_DST_L(cn)	(0x828 + (cn) * 0x40)
#define EDMAC_CX_DST_H(cn)	(0x82c + (cn) * 0x40)
#define EDMAC_CX_CONFIG(cn)	(0x830 + (cn) * 0x40)
#define EDMAC_CX_CURR_CNT0(cn)	(0x404 + (cn) * 0x20)

#define EDMAC_ALL_CHAN_CLR	0xff
#define EDMAC_INT_ENABLE_ALL	0xff

/* Channel config register fields */
#define EDMAC_CONFIG_SRC_INC		BIT(31)
#define EDMAC_CONFIG_DST_INC		BIT(30)
#define EDMAC_CONFIG_SRC_BURST_SHIFT	24
#define EDMAC_CONFIG_DST_BURST_SHIFT	20
#define EDMAC_CONFIG_SRC_WIDTH_SHIFT	16
#define EDMAC_CONFIG_DST_WIDTH_SHIFT	12

#define EDMAC_CXCONFIG_TSF_TYPE_SHIFT	2
#define EDMAC_CXCONFIG_SIGNAL_SHIFT	4
#define EDMAC_CXCONFIG_LLI_START	BIT(0)
#define EDMAC_CXCONFIG_CX_EN		BIT(0)
#define EDMAC_CXCONFIG_MEM_TYPE		0x0
#define EDMAC_CXCONFIG_DEV_MEM_TYPE	0x1

#define EDMAC_CONTROL_SRC_WIDTH_MASK	GENMASK(18, 16)
#define EDMAC_CONTROL_DST_WIDTH_MASK	GENMASK(14, 12)

/* Width encoding of the config register */
#define EDMAC_WIDTH_8BIT		0
#define EDMAC_WIDTH_16BIT		1
#define EDMAC_WIDTH_32BIT		2
#define EDMAC_WIDTH_64BIT		3
#define EDMAC_MEM_BIT_WIDTH		EDMAC_WIDTH_64BIT

#define EDMAC_MAX_BURST_WIDTH		16
#define EDMAC_MIN_BURST_WIDTH		1

/* The LLI is a 64 byte descriptor, which mirrors the per channel
 * register layout and carries the full 64 bit AXI addresses.
 */
#define EDMAC_LLI_ALIGN			0x40
#define EDMAC_LLI_ENABLE		0x2

#define MAX_TRANSFER_BYTES		0xffff
#define LLI_BLOCK_SIZE			(4 * PAGE_SIZE)
#define MAX_TSFR_LLIS			(LLI_BLOCK_SIZE / EDMAC_LLI_ALIGN)
#define BITS_PER_HALF_WORD		32

struct edmac_lli {
	u64 next_lli;
	u32 reserved[5];
	u32 count;
	u64 src_addr;
	u64 dest_addr;
	u32 config;
	u32 pad[3];
};

struct edmac_sg {
	dma_addr_t src_addr;
	dma_addr_t dst_addr;
	size_t len;
	struct list_head node;
};

struct edmac_txd {
	struct virt_dma_desc vd;
	dma_addr_t llis_busaddr;
	u64 *llis_vaddr;
	u32 ccfg;
	size_t size;
	bool cyclic;
};

enum edmac_chan_state {
	EDMAC_CHAN_IDLE,
	EDMAC_CHAN_RUNNING,
	EDMAC_CHAN_PAUSED,
	EDMAC_CHAN_WAITING,
};

struct edmacv310_chan {
	bool slave;
	int signal;
	int id;
	struct virt_dma_chan vc;
	struct edmacv310_phy_chan *phychan;
	struct dma_slave_config cfg;
	struct edmac_txd *at;
	struct edmacv310_dmadev *host;
	enum edmac_chan_state state;
};

struct edmacv310_phy_chan {
	unsigned int id;
	spinlock_t lock;	/* protects serving channel reassignment */
	struct edmacv310_chan *serving;
};

struct edmacv310_dmadev {
	struct dma_device slave;
	struct dma_device memcpy;
	void __iomem *base;
	struct edmacv310_phy_chan *phy_chans;
	struct dma_pool *pool;
	int irq;
	struct clk *clk;
	struct clk *axi_clk;
	struct reset_control *rstc;
	unsigned int channels;
	unsigned int slave_requests;
};

static inline struct edmacv310_chan *to_edmac_chan(struct dma_chan *chan)
{
	return container_of(chan, struct edmacv310_chan, vc.chan);
}

static inline struct edmac_txd *to_edmac_txd(struct dma_async_tx_descriptor *tx)
{
	return container_of(tx, struct edmac_txd, vd.tx);
}

static struct dma_chan *edmacv310_find_chan(struct edmacv310_dmadev *edmac,
					    int request_num)
{
	struct edmacv310_chan *chan;

	list_for_each_entry(chan, &edmac->slave.channels, vc.chan.device_node) {
		if (chan->id == request_num)
			return &chan->vc.chan;
	}

	return NULL;
}

static struct dma_chan *edmacv310_of_xlate(struct of_phandle_args *dma_spec,
					   struct of_dma *ofdma)
{
	struct edmacv310_dmadev *edmac = ofdma->of_dma_data;
	struct edmacv310_chan *chan;
	struct dma_chan *dma_chan;

	if (dma_spec->args_count != 2)
		return NULL;

	/*
	 * The first cell is the request line number, which selects the
	 * virtual slave channel and is written to the request selector
	 * field of the channel config register.  The second cell is the
	 * request multiplexer value of the requesting peripheral, which
	 * is ignored on this SoC.
	 */
	dma_chan = edmacv310_find_chan(edmac, dma_spec->args[0]);
	if (!dma_chan)
		return NULL;

	chan = to_edmac_chan(dma_chan);
	chan->signal = dma_spec->args[0];

	return dma_get_slave_channel(dma_chan);
}

static size_t edmacv310_read_residue(struct edmacv310_chan *chan,
				     const struct edmac_txd *txd)
{
	struct edmacv310_dmadev *edmac = chan->host;
	struct edmacv310_phy_chan *phychan = chan->phychan;
	struct edmac_lli *plli;
	unsigned int i, index;
	size_t bytes;
	u64 next_lli;

	next_lli = readl(edmac->base + EDMAC_CX_LLI_L(phychan->id)) &
		   ~(EDMAC_LLI_ALIGN - 1);
	next_lli |= (u64)readl(edmac->base + EDMAC_CX_LLI_H(phychan->id))
		    << BITS_PER_HALF_WORD;
	bytes = readl(edmac->base + EDMAC_CX_CURR_CNT0(phychan->id));

	if (next_lli != 0) {
		/* The channel runs in LLI mode. */
		bytes += txd->size;
		index = (next_lli - txd->llis_busaddr) / sizeof(*plli);
		plli = (struct edmac_lli *)txd->llis_vaddr;

		if (index > MAX_TSFR_LLIS)
			return 0;

		for (i = 0; i < index; i++)
			bytes -= plli[i].count;
	}

	return bytes;
}

static enum dma_status edmacv310_tx_status(struct dma_chan *chan,
					   dma_cookie_t cookie,
					   struct dma_tx_state *txstate)
{
	struct edmacv310_chan *edmac_chan = to_edmac_chan(chan);
	struct edmac_txd *txd = NULL;
	struct virt_dma_desc *vd;
	unsigned long flags;
	enum dma_status ret;
	size_t bytes;

	ret = dma_cookie_status(chan, cookie, txstate);
	if (ret == DMA_COMPLETE)
		return ret;

	spin_lock_irqsave(&edmac_chan->vc.lock, flags);
	vd = vchan_find_desc(&edmac_chan->vc, cookie);
	if (vd) {
		/* The transfer has not been started yet. */
		txd = to_edmac_txd(&vd->tx);
		bytes = txd->size;
	} else {
		/* The transfer is running. */
		txd = edmac_chan->at;

		if (!edmac_chan->phychan || !txd) {
			spin_unlock_irqrestore(&edmac_chan->vc.lock, flags);
			return ret;
		}

		bytes = edmacv310_read_residue(edmac_chan, txd);
	}
	spin_unlock_irqrestore(&edmac_chan->vc.lock, flags);
	dma_set_residue(txstate, bytes);

	if (edmac_chan->state == EDMAC_CHAN_PAUSED && ret == DMA_IN_PROGRESS)
		ret = DMA_PAUSED;

	return ret;
}

static struct edmacv310_phy_chan *
edmacv310_get_phy_chan(struct edmacv310_dmadev *edmac,
		       struct edmacv310_chan *chan)
{
	struct edmacv310_phy_chan *ch = NULL;
	unsigned long flags;
	int i;

	for (i = 0; i < edmac->channels; i++) {
		ch = &edmac->phy_chans[i];

		spin_lock_irqsave(&ch->lock, flags);
		if (!ch->serving) {
			ch->serving = chan;
			spin_unlock_irqrestore(&ch->lock, flags);
			break;
		}
		spin_unlock_irqrestore(&ch->lock, flags);
	}

	if (i == edmac->channels)
		return NULL;

	return ch;
}

static void edmacv310_write_first_lli(struct edmacv310_dmadev *edmac,
				      struct edmacv310_phy_chan *phychan,
				      const struct edmac_txd *txd)
{
	struct edmac_lli *plli = (struct edmac_lli *)txd->llis_vaddr;
	void __iomem *base = edmac->base;
	unsigned int id = phychan->id;

	writel(lower_32_bits(plli->next_lli) |
	       (plli->next_lli ? EDMAC_LLI_ENABLE : 0),
	       base + EDMAC_CX_LLI_L(id));
	writel(upper_32_bits(plli->next_lli), base + EDMAC_CX_LLI_H(id));
	writel(plli->count, base + EDMAC_CX_CNT0(id));
	writel(lower_32_bits(plli->src_addr), base + EDMAC_CX_SRC_L(id));
	writel(upper_32_bits(plli->src_addr), base + EDMAC_CX_SRC_H(id));
	writel(lower_32_bits(plli->dest_addr), base + EDMAC_CX_DST_L(id));
	writel(upper_32_bits(plli->dest_addr), base + EDMAC_CX_DST_H(id));
	writel(plli->config, base + EDMAC_CX_CONFIG(id));
}

static void edmacv310_start_next_txd(struct edmacv310_chan *chan)
{
	struct edmacv310_dmadev *edmac = chan->host;
	struct edmacv310_phy_chan *phychan = chan->phychan;
	struct virt_dma_desc *vd;
	struct edmac_txd *txd;
	u32 val;

	vd = vchan_next_desc(&chan->vc);
	if (!vd)
		return;

	txd = to_edmac_txd(&vd->tx);
	list_del(&txd->vd.node);

	chan->at = txd;
	edmacv310_write_first_lli(edmac, phychan, txd);

	val = readl(edmac->base + EDMAC_CX_CONFIG(phychan->id));
	writel(val | EDMAC_CXCONFIG_LLI_START,
	       edmac->base + EDMAC_CX_CONFIG(phychan->id));
}

static void edmacv310_start(struct edmacv310_chan *chan)
{
	struct edmacv310_phy_chan *phychan;

	phychan = edmacv310_get_phy_chan(chan->host, chan);
	if (!phychan) {
		chan->state = EDMAC_CHAN_WAITING;
		return;
	}

	chan->phychan = phychan;
	chan->state = EDMAC_CHAN_RUNNING;
	edmacv310_start_next_txd(chan);
}

static void edmacv310_issue_pending(struct dma_chan *chan)
{
	struct edmacv310_chan *edmac_chan = to_edmac_chan(chan);
	unsigned long flags;

	spin_lock_irqsave(&edmac_chan->vc.lock, flags);
	if (vchan_issue_pending(&edmac_chan->vc)) {
		if (!edmac_chan->phychan &&
		    edmac_chan->state != EDMAC_CHAN_WAITING)
			edmacv310_start(edmac_chan);
	}
	spin_unlock_irqrestore(&edmac_chan->vc.lock, flags);
}

static int edmacv310_config(struct dma_chan *chan,
			    struct dma_slave_config *config)
{
	struct edmacv310_chan *edmac_chan = to_edmac_chan(chan);

	if (!edmac_chan->slave)
		return -EINVAL;

	edmac_chan->cfg = *config;

	return 0;
}

static void edmacv310_pause_phy_chan(struct edmacv310_chan *chan)
{
	struct edmacv310_dmadev *edmac = chan->host;
	struct edmacv310_phy_chan *phychan = chan->phychan;
	int timeout;
	u32 val;

	val = readl(edmac->base + EDMAC_CX_CONFIG(phychan->id));
	val &= ~EDMAC_CXCONFIG_CX_EN;
	writel(val, edmac->base + EDMAC_CX_CONFIG(phychan->id));

	/* Wait for the channel to become inactive. */
	for (timeout = 2000; timeout > 0; timeout--) {
		if (!(readl(edmac->base + EDMAC_CH_STAT) & BIT(phychan->id)))
			break;

		writel(val, edmac->base + EDMAC_CX_CONFIG(phychan->id));
		udelay(1);
	}

	if (timeout == 0)
		dev_err(chan->vc.chan.device->dev,
			"channel%u timeout waiting for pause\n", phychan->id);
}

static int edmacv310_pause(struct dma_chan *chan)
{
	struct edmacv310_chan *edmac_chan = to_edmac_chan(chan);
	unsigned long flags;

	spin_lock_irqsave(&edmac_chan->vc.lock, flags);
	if (edmac_chan->phychan) {
		edmacv310_pause_phy_chan(edmac_chan);
		edmac_chan->state = EDMAC_CHAN_PAUSED;
	}
	spin_unlock_irqrestore(&edmac_chan->vc.lock, flags);

	return 0;
}

static int edmacv310_resume(struct dma_chan *chan)
{
	struct edmacv310_chan *edmac_chan = to_edmac_chan(chan);
	struct edmacv310_dmadev *edmac;
	struct edmacv310_phy_chan *phychan;
	unsigned long flags;
	u32 val;

	spin_lock_irqsave(&edmac_chan->vc.lock, flags);
	if (edmac_chan->phychan) {
		edmac = edmac_chan->host;
		phychan = edmac_chan->phychan;

		val = readl(edmac->base + EDMAC_CX_CONFIG(phychan->id));
		writel(val | EDMAC_CXCONFIG_CX_EN,
		       edmac->base + EDMAC_CX_CONFIG(phychan->id));

		edmac_chan->state = EDMAC_CHAN_RUNNING;
	}
	spin_unlock_irqrestore(&edmac_chan->vc.lock, flags);

	return 0;
}

static void edmacv310_desc_free(struct virt_dma_desc *vd);
static void edmacv310_phy_free(struct edmacv310_chan *chan);

static void edmacv310_free_txd_list(struct edmacv310_chan *chan)
{
	LIST_HEAD(head);

	vchan_get_all_descriptors(&chan->vc, &head);
	vchan_dma_desc_free_list(&chan->vc, &head);
}

static int edmacv310_terminate_all(struct dma_chan *chan)
{
	struct edmacv310_chan *edmac_chan = to_edmac_chan(chan);
	unsigned long flags;

	spin_lock_irqsave(&edmac_chan->vc.lock, flags);
	edmac_chan->state = EDMAC_CHAN_IDLE;

	if (edmac_chan->phychan)
		edmacv310_phy_free(edmac_chan);

	if (edmac_chan->at) {
		edmacv310_desc_free(&edmac_chan->at->vd);
		edmac_chan->at = NULL;
	}

	edmacv310_free_txd_list(edmac_chan);
	spin_unlock_irqrestore(&edmac_chan->vc.lock, flags);

	return 0;
}

static void edmacv310_synchronize(struct dma_chan *chan)
{
	vchan_synchronize(to_virt_chan(chan));
}

static int edmacv310_get_width(enum dma_slave_buswidth width)
{
	switch (width) {
	case DMA_SLAVE_BUSWIDTH_1_BYTE:
		return EDMAC_WIDTH_8BIT;
	case DMA_SLAVE_BUSWIDTH_2_BYTES:
		return EDMAC_WIDTH_16BIT;
	case DMA_SLAVE_BUSWIDTH_4_BYTES:
		return EDMAC_WIDTH_32BIT;
	case DMA_SLAVE_BUSWIDTH_8_BYTES:
		return EDMAC_WIDTH_64BIT;
	default:
		return -EINVAL;
	}
}

static int edmacv310_set_config_value(enum dma_transfer_direction direction,
				      unsigned int addr_width,
				      unsigned int burst,
				      unsigned int signal, u32 *config)
{
	int width;

	if (direction == DMA_MEM_TO_DEV)
		*config = EDMAC_CONFIG_SRC_INC;
	else
		*config = EDMAC_CONFIG_DST_INC;

	width = edmacv310_get_width(addr_width);
	if (width < 0)
		return width;

	*config |= width << EDMAC_CONFIG_SRC_WIDTH_SHIFT;
	*config |= width << EDMAC_CONFIG_DST_WIDTH_SHIFT;
	*config |= burst << EDMAC_CONFIG_SRC_BURST_SHIFT;
	*config |= burst << EDMAC_CONFIG_DST_BURST_SHIFT;
	*config |= signal << EDMAC_CXCONFIG_SIGNAL_SHIFT;
	*config |= EDMAC_CXCONFIG_DEV_MEM_TYPE << EDMAC_CXCONFIG_TSF_TYPE_SHIFT;

	return 0;
}

static struct edmac_txd *
edmacv310_init_txd(struct dma_chan *chan,
		   enum dma_transfer_direction direction,
		   dma_addr_t *slave_addr)
{
	struct edmacv310_chan *edmac_chan = to_edmac_chan(chan);
	struct edmac_txd *txd;
	unsigned int addr_width = 0;
	unsigned int maxburst = 0;
	unsigned int burst;

	txd = kzalloc_obj(*txd, GFP_NOWAIT);
	if (!txd)
		return NULL;

	if (direction == DMA_MEM_TO_DEV) {
		*slave_addr = edmac_chan->cfg.dst_addr;
		addr_width = edmac_chan->cfg.dst_addr_width;
		maxburst = edmac_chan->cfg.dst_maxburst;
	} else if (direction == DMA_DEV_TO_MEM) {
		*slave_addr = edmac_chan->cfg.src_addr;
		addr_width = edmac_chan->cfg.src_addr_width;
		maxburst = edmac_chan->cfg.src_maxburst;
	} else {
		dev_err(chan->device->dev, "direction unsupported!\n");
		kfree(txd);
		return NULL;
	}

	if (maxburst > EDMAC_MAX_BURST_WIDTH)
		burst = EDMAC_MAX_BURST_WIDTH - 1;
	else if (maxburst == 0)
		burst = EDMAC_MIN_BURST_WIDTH;
	else
		burst = maxburst - 1;

	if (edmacv310_set_config_value(direction, addr_width, burst,
				       edmac_chan->signal, &txd->ccfg)) {
		dev_err(chan->device->dev, "invalid bus width!\n");
		kfree(txd);
		return NULL;
	}

	return txd;
}

static int edmacv310_fill_desc(const struct edmac_sg *dsg,
			       struct edmac_txd *txd,
			       unsigned int length, unsigned int num)
{
	struct edmac_lli *plli;

	if (num >= MAX_TSFR_LLIS) {
		dev_err(txd->vd.tx.chan->device->dev, "lli out of range\n");
		return -ENOMEM;
	}

	plli = (struct edmac_lli *)txd->llis_vaddr;
	memset(&plli[num], 0, sizeof(*plli));

	plli[num].src_addr = dsg->src_addr;
	plli[num].dest_addr = dsg->dst_addr;
	plli[num].config = txd->ccfg;
	plli[num].count = length;
	txd->size += length;

	if (num > 0) {
		plli[num - 1].next_lli = txd->llis_busaddr +
					 num * sizeof(*plli);
		plli[num - 1].next_lli |= EDMAC_LLI_ENABLE;
	}

	return 0;
}

static void edmacv310_free_dsg(struct list_head *dsg_head)
{
	struct edmac_sg *dsg, *_dsg;

	list_for_each_entry_safe(dsg, _dsg, dsg_head, node) {
		list_del(&dsg->node);
		kfree(dsg);
	}
}

static int edmacv310_add_sg(struct list_head *sg_head, dma_addr_t dst,
			    dma_addr_t src, size_t len)
{
	struct edmac_sg *dsg;

	if (len == 0) {
		edmacv310_free_dsg(sg_head);
		return -EINVAL;
	}

	dsg = kzalloc(sizeof(*dsg), GFP_NOWAIT);
	if (!dsg) {
		edmacv310_free_dsg(sg_head);
		return -ENOMEM;
	}

	list_add_tail(&dsg->node, sg_head);
	dsg->src_addr = src;
	dsg->dst_addr = dst;
	dsg->len = len;

	return 0;
}

static int edmacv310_add_sg_slave(struct list_head *sg_head,
				  dma_addr_t slave_addr, dma_addr_t addr,
				  size_t length,
				  enum dma_transfer_direction direction)
{
	dma_addr_t src = 0;
	dma_addr_t dst = 0;

	if (direction == DMA_MEM_TO_DEV) {
		src = addr;
		dst = slave_addr;
	} else if (direction == DMA_DEV_TO_MEM) {
		src = slave_addr;
		dst = addr;
	} else {
		return -EINVAL;
	}

	return edmacv310_add_sg(sg_head, dst, src, length);
}

static int edmacv310_fill_sg_for_slave(struct list_head *sg_head,
				       dma_addr_t slave_addr,
				       struct scatterlist *sgl,
				       unsigned int sg_len,
				       enum dma_transfer_direction direction)
{
	struct scatterlist *sg;
	int tmp, ret = 0;
	size_t length;
	dma_addr_t addr;

	for_each_sg(sgl, sg, sg_len, tmp) {
		addr = sg_dma_address(sg);
		length = sg_dma_len(sg);
		ret = edmacv310_add_sg_slave(sg_head, slave_addr, addr,
					     length, direction);
		if (ret)
			break;
	}

	return ret;
}

struct edmac_cyclic_args {
	dma_addr_t slave_addr;
	dma_addr_t buf_addr;
	size_t buf_len;
	size_t period_len;
	enum dma_transfer_direction direction;
};

static int edmacv310_fill_sg_for_cyclic(struct list_head *sg_head,
					struct edmac_cyclic_args args)
{
	size_t count_in_sg = 0;
	size_t trans_bytes;
	int ret;

	while (count_in_sg < args.buf_len) {
		trans_bytes = min(args.period_len, args.buf_len - count_in_sg);
		count_in_sg += trans_bytes;
		ret = edmacv310_add_sg_slave(sg_head, args.slave_addr,
					     args.buf_addr + count_in_sg,
					     count_in_sg, args.direction);
		if (ret)
			return ret;
	}

	return 0;
}

static unsigned short edmacv310_get_max_width(u32 ccfg)
{
	unsigned short src_width = (ccfg & EDMAC_CONTROL_SRC_WIDTH_MASK) >>
				   EDMAC_CONFIG_SRC_WIDTH_SHIFT;
	unsigned short dst_width = (ccfg & EDMAC_CONTROL_DST_WIDTH_MASK) >>
				   EDMAC_CONFIG_DST_WIDTH_SHIFT;

	return 1 << max(src_width, dst_width); /* in bytes */
}

static int edmacv310_fill_sg_lli_for_desc(struct edmac_sg *dsg,
					  struct edmac_txd *txd,
					  unsigned int *lli_count)
{
	unsigned short width = edmacv310_get_max_width(txd->ccfg);
	int ret;

	while (dsg->len != 0) {
		size_t lli_len = MAX_TRANSFER_BYTES;

		lli_len = (lli_len / width) * width; /* bus width aligned */
		lli_len = min(lli_len, dsg->len);
		ret = edmacv310_fill_desc(dsg, txd, lli_len, *lli_count);
		if (ret)
			return ret;

		if (txd->ccfg & EDMAC_CONFIG_SRC_INC)
			dsg->src_addr += lli_len;
		if (txd->ccfg & EDMAC_CONFIG_DST_INC)
			dsg->dst_addr += lli_len;
		dsg->len -= lli_len;
		(*lli_count)++;
	}

	return 0;
}

static int edmacv310_fill_lli_for_desc(struct list_head *sg_head,
				       struct edmac_txd *txd)
{
	struct edmac_lli *last_plli;
	struct edmac_sg *dsg;
	unsigned int lli_count = 0;
	int ret;

	list_for_each_entry(dsg, sg_head, node) {
		ret = edmacv310_fill_sg_lli_for_desc(dsg, txd, &lli_count);
		if (ret)
			return ret;
	}

	last_plli = (struct edmac_lli *)((uintptr_t)txd->llis_vaddr +
					  (lli_count - 1) * sizeof(*last_plli));
	if (txd->cyclic)
		last_plli->next_lli = txd->llis_busaddr | EDMAC_LLI_ENABLE;
	else
		last_plli->next_lli = 0;

	return 0;
}

static struct dma_async_tx_descriptor *
edmacv310_prep_slave_sg(struct dma_chan *chan, struct scatterlist *sgl,
			unsigned int sg_len,
			enum dma_transfer_direction direction,
			unsigned long flags, void *context)
{
	struct edmacv310_chan *edmac_chan = to_edmac_chan(chan);
	struct edmacv310_dmadev *edmac = edmac_chan->host;
	struct dma_async_tx_descriptor *tx;
	struct edmac_txd *txd;
	dma_addr_t slave_addr = 0;
	LIST_HEAD(sg_head);
	int ret;

	txd = edmacv310_init_txd(chan, direction, &slave_addr);
	if (!txd)
		return NULL;

	txd->llis_vaddr = dma_pool_alloc(edmac->pool, GFP_NOWAIT,
					 &txd->llis_busaddr);
	if (!txd->llis_vaddr)
		goto err_alloc_lli;

	ret = edmacv310_fill_sg_for_slave(&sg_head, slave_addr, sgl, sg_len,
					  direction);
	if (ret)
		goto err_fill_sg;

	ret = edmacv310_fill_lli_for_desc(&sg_head, txd);
	edmacv310_free_dsg(&sg_head);
	if (ret)
		goto err_fill_sg;

	tx = vchan_tx_prep(&edmac_chan->vc, &txd->vd, flags);
	if (!tx)
		goto err_fill_sg;

	return tx;

err_fill_sg:
	dma_pool_free(edmac->pool, txd->llis_vaddr, txd->llis_busaddr);
err_alloc_lli:
	kfree(txd);
	return NULL;
}

static struct dma_async_tx_descriptor *
edmacv310_prep_dma_memcpy(struct dma_chan *chan, dma_addr_t dst,
			  dma_addr_t src, size_t len, unsigned long flags)
{
	struct edmacv310_chan *edmac_chan = to_edmac_chan(chan);
	struct edmacv310_dmadev *edmac = edmac_chan->host;
	struct dma_async_tx_descriptor *tx;
	struct edmac_txd *txd;
	LIST_HEAD(sg_head);
	u32 config = 0;
	int ret;

	if (!len)
		return NULL;

	txd = kzalloc_obj(*txd, GFP_NOWAIT);
	if (!txd)
		return NULL;

	txd->llis_vaddr = dma_pool_alloc(edmac->pool, GFP_NOWAIT,
					 &txd->llis_busaddr);
	if (!txd->llis_vaddr)
		goto err_alloc_lli;

	config |= EDMAC_CONFIG_SRC_INC | EDMAC_CONFIG_DST_INC;
	config |= EDMAC_CXCONFIG_MEM_TYPE << EDMAC_CXCONFIG_TSF_TYPE_SHIFT;
	/* The max burst width is 16, and the register encodes it as 0xf. */
	config |= (EDMAC_MAX_BURST_WIDTH - 1) << EDMAC_CONFIG_SRC_BURST_SHIFT;
	config |= (EDMAC_MAX_BURST_WIDTH - 1) << EDMAC_CONFIG_DST_BURST_SHIFT;
	config |= EDMAC_MEM_BIT_WIDTH << EDMAC_CONFIG_SRC_WIDTH_SHIFT;
	config |= EDMAC_MEM_BIT_WIDTH << EDMAC_CONFIG_DST_WIDTH_SHIFT;
	txd->ccfg = config;

	ret = edmacv310_add_sg(&sg_head, dst, src, len);
	if (ret)
		goto err_fill_sg;

	ret = edmacv310_fill_lli_for_desc(&sg_head, txd);
	edmacv310_free_dsg(&sg_head);
	if (ret)
		goto err_fill_sg;

	tx = vchan_tx_prep(&edmac_chan->vc, &txd->vd, flags);
	if (!tx)
		goto err_fill_sg;

	return tx;

err_fill_sg:
	dma_pool_free(edmac->pool, txd->llis_vaddr, txd->llis_busaddr);
err_alloc_lli:
	kfree(txd);
	return NULL;
}

static struct dma_async_tx_descriptor *
edmacv310_prep_dma_cyclic(struct dma_chan *chan, dma_addr_t buf_addr,
			  size_t buf_len, size_t period_len,
			  enum dma_transfer_direction direction,
			  unsigned long flags)
{
	struct edmacv310_chan *edmac_chan = to_edmac_chan(chan);
	struct edmacv310_dmadev *edmac = edmac_chan->host;
	struct dma_async_tx_descriptor *tx;
	struct edmac_cyclic_args args = {
		.buf_addr = buf_addr,
		.buf_len = buf_len,
		.period_len = period_len,
		.direction = direction,
	};
	struct edmac_txd *txd;
	LIST_HEAD(sg_head);
	int ret;

	txd = edmacv310_init_txd(chan, direction, &args.slave_addr);
	if (!txd)
		return NULL;

	txd->llis_vaddr = dma_pool_alloc(edmac->pool, GFP_NOWAIT,
					 &txd->llis_busaddr);
	if (!txd->llis_vaddr)
		goto err_alloc_lli;

	txd->cyclic = true;

	ret = edmacv310_fill_sg_for_cyclic(&sg_head, args);
	if (ret)
		goto err_fill_sg;

	ret = edmacv310_fill_lli_for_desc(&sg_head, txd);
	edmacv310_free_dsg(&sg_head);
	if (ret)
		goto err_fill_sg;

	tx = vchan_tx_prep(&edmac_chan->vc, &txd->vd, flags);
	if (!tx)
		goto err_fill_sg;

	return tx;

err_fill_sg:
	dma_pool_free(edmac->pool, txd->llis_vaddr, txd->llis_busaddr);
err_alloc_lli:
	kfree(txd);
	return NULL;
}

static void edmacv310_phy_reassign(struct edmacv310_phy_chan *phychan,
				   struct edmacv310_chan *chan)
{
	phychan->serving = chan;
	chan->phychan = phychan;
	chan->state = EDMAC_CHAN_RUNNING;

	edmacv310_start_next_txd(chan);
}

static void edmacv310_terminate_phy_chan(struct edmacv310_chan *chan)
{
	struct edmacv310_dmadev *edmac = chan->host;
	struct edmacv310_phy_chan *phychan = chan->phychan;
	u32 val;

	edmacv310_pause_phy_chan(chan);

	val = BIT(phychan->id);
	writel(val, edmac->base + EDMAC_INT_TC1_RAW);
	writel(val, edmac->base + EDMAC_INT_ERR1_RAW);
	writel(val, edmac->base + EDMAC_INT_ERR2_RAW);
}

static void edmacv310_phy_free(struct edmacv310_chan *chan)
{
	struct edmacv310_dmadev *edmac = chan->host;
	struct edmacv310_chan *p, *next = NULL;

	/* Give the freed physical channel to a waiting memory to memory
	 * channel first, then to a waiting slave channel.
	 */
	list_for_each_entry(p, &edmac->memcpy.channels, vc.chan.device_node) {
		if (p->state == EDMAC_CHAN_WAITING) {
			next = p;
			break;
		}
	}

	if (!next) {
		list_for_each_entry(p, &edmac->slave.channels,
				    vc.chan.device_node) {
			if (p->state == EDMAC_CHAN_WAITING) {
				next = p;
				break;
			}
		}
	}

	edmacv310_terminate_phy_chan(chan);

	if (next) {
		spin_lock(&next->vc.lock);
		edmacv310_phy_reassign(chan->phychan, next);
		spin_unlock(&next->vc.lock);
	} else {
		chan->phychan->serving = NULL;
	}

	chan->phychan = NULL;
	chan->state = EDMAC_CHAN_IDLE;
}

static bool edmacv310_handle_irq(struct edmacv310_dmadev *edmac, int chan_id)
{
	struct edmacv310_phy_chan *phychan = &edmac->phy_chans[chan_id];
	struct edmacv310_chan *chan = phychan->serving;
	struct edmac_txd *txd;
	u32 tc_status, err_status;
	u32 chan_bit = BIT(chan_id);

	if (!chan) {
		dev_err(edmac->memcpy.dev,
			"error interrupt on chan: %d!\n", chan_id);
		return false;
	}

	txd = chan->at;

	tc_status = readl(edmac->base + EDMAC_INT_TC1_RAW);
	if (tc_status & chan_bit)
		writel(chan_bit, edmac->base + EDMAC_INT_TC1_RAW);

	tc_status = readl(edmac->base + EDMAC_INT_TC2);
	if (tc_status & chan_bit)
		writel(chan_bit, edmac->base + EDMAC_INT_TC2_RAW);

	err_status = readl(edmac->base + EDMAC_INT_ERR1) |
		     readl(edmac->base + EDMAC_INT_ERR2) |
		     readl(edmac->base + EDMAC_INT_ERR3);
	if (err_status & chan_bit) {
		dev_err(edmac->memcpy.dev,
			"error in edmac %d!, ERR1 = 0x%x, ERR2 = 0x%x, ERR3 = 0x%x\n",
			chan_id, readl(edmac->base + EDMAC_INT_ERR1),
			readl(edmac->base + EDMAC_INT_ERR2),
			readl(edmac->base + EDMAC_INT_ERR3));
		writel(chan_bit, edmac->base + EDMAC_INT_ERR1_RAW);
		writel(chan_bit, edmac->base + EDMAC_INT_ERR2_RAW);
		writel(chan_bit, edmac->base + EDMAC_INT_ERR3_RAW);
	}

	spin_lock(&chan->vc.lock);

	if (txd->cyclic) {
		vchan_cyclic_callback(&txd->vd);
		spin_unlock(&chan->vc.lock);
		return true;
	}

	chan->at = NULL;
	vchan_cookie_complete(&txd->vd);

	if (vchan_next_desc(&chan->vc))
		edmacv310_start_next_txd(chan);
	else
		edmacv310_phy_free(chan);

	spin_unlock(&chan->vc.lock);

	return true;
}

static irqreturn_t edmacv310_irq(int irq, void *dev_id)
{
	struct edmacv310_dmadev *edmac = dev_id;
	u32 channel_status, mask = 0;
	unsigned int i;

	channel_status = readl(edmac->base + EDMAC_INT_STAT);
	if (!channel_status)
		return IRQ_NONE;

	for (i = 0; i < edmac->channels; i++) {
		if (channel_status & BIT(i))
			mask |= edmacv310_handle_irq(edmac, i) << i;
	}

	return mask ? IRQ_HANDLED : IRQ_NONE;
}

static void edmacv310_desc_free(struct virt_dma_desc *vd)
{
	struct edmac_txd *txd = to_edmac_txd(&vd->tx);
	struct edmacv310_chan *chan = to_edmac_chan(vd->tx.chan);

	dma_descriptor_unmap(&vd->tx);
	dma_pool_free(chan->host->pool, txd->llis_vaddr, txd->llis_busaddr);
	kfree(txd);
}

static void edmacv310_free_chan_resources(struct dma_chan *chan)
{
	vchan_free_chan_resources(to_virt_chan(chan));
}

static int edmacv310_init_virt_channels(struct edmacv310_dmadev *edmac,
					struct dma_device *dmadev,
					unsigned int channels, bool slave)
{
	struct edmacv310_chan *chan;
	int i;

	INIT_LIST_HEAD(&dmadev->channels);

	for (i = 0; i < channels; i++) {
		chan = kzalloc_obj(*chan, GFP_KERNEL);
		if (!chan)
			return -ENOMEM;

		chan->host = edmac;
		chan->state = EDMAC_CHAN_IDLE;
		chan->signal = -1;

		if (slave) {
			chan->id = i;
			chan->slave = true;
		}

		chan->vc.desc_free = edmacv310_desc_free;
		vchan_init(&chan->vc, dmadev);
	}

	return 0;
}

static void edmacv310_free_virt_channels(struct dma_device *dmadev)
{
	struct edmacv310_chan *chan, *next;

	list_for_each_entry_safe(chan, next, &dmadev->channels,
				 vc.chan.device_node) {
		list_del(&chan->vc.chan.device_node);
		tasklet_kill(&chan->vc.task);
		kfree(chan);
	}
}

static int edmacv310_init_channels(struct edmacv310_dmadev *edmac)
{
	int i, ret;

	edmac->phy_chans = kcalloc(edmac->channels,
				   sizeof(*edmac->phy_chans), GFP_KERNEL);
	if (!edmac->phy_chans)
		return -ENOMEM;

	for (i = 0; i < edmac->channels; i++) {
		edmac->phy_chans[i].id = i;
		spin_lock_init(&edmac->phy_chans[i].lock);
	}

	ret = edmacv310_init_virt_channels(edmac, &edmac->memcpy,
					   edmac->channels, false);
	if (ret)
		goto free_phychans;

	ret = edmacv310_init_virt_channels(edmac, &edmac->slave,
					   edmac->slave_requests, true);
	if (ret)
		goto free_memcpy_virt_channels;

	return 0;

free_memcpy_virt_channels:
	edmacv310_free_virt_channels(&edmac->memcpy);
free_phychans:
	kfree(edmac->phy_chans);

	return -ENOMEM;
}

static void edmacv310_free_channels(struct edmacv310_dmadev *edmac)
{
	edmacv310_free_virt_channels(&edmac->slave);
	edmacv310_free_virt_channels(&edmac->memcpy);
	kfree(edmac->phy_chans);
}

static void edmacv310_prep_dma_devices(struct edmacv310_dmadev *edmac)
{
	struct dma_device *memcpy = &edmac->memcpy;
	struct dma_device *slave = &edmac->slave;

	dma_cap_set(DMA_MEMCPY, memcpy->cap_mask);
	memcpy->device_free_chan_resources = edmacv310_free_chan_resources;
	memcpy->device_prep_dma_memcpy = edmacv310_prep_dma_memcpy;
	memcpy->device_tx_status = edmacv310_tx_status;
	memcpy->device_issue_pending = edmacv310_issue_pending;
	memcpy->device_config = edmacv310_config;
	memcpy->device_pause = edmacv310_pause;
	memcpy->device_resume = edmacv310_resume;
	memcpy->device_terminate_all = edmacv310_terminate_all;
	memcpy->device_synchronize = edmacv310_synchronize;
	memcpy->directions = BIT(DMA_MEM_TO_MEM);
	memcpy->residue_granularity = DMA_RESIDUE_GRANULARITY_SEGMENT;

	dma_cap_set(DMA_SLAVE, slave->cap_mask);
	dma_cap_set(DMA_CYCLIC, slave->cap_mask);
	slave->device_free_chan_resources = edmacv310_free_chan_resources;
	slave->device_tx_status = edmacv310_tx_status;
	slave->device_issue_pending = edmacv310_issue_pending;
	slave->device_prep_slave_sg = edmacv310_prep_slave_sg;
	slave->device_prep_dma_cyclic = edmacv310_prep_dma_cyclic;
	slave->device_config = edmacv310_config;
	slave->device_pause = edmacv310_pause;
	slave->device_resume = edmacv310_resume;
	slave->device_terminate_all = edmacv310_terminate_all;
	slave->device_synchronize = edmacv310_synchronize;
	slave->directions = BIT(DMA_DEV_TO_MEM) | BIT(DMA_MEM_TO_DEV);
	slave->residue_granularity = DMA_RESIDUE_GRANULARITY_SEGMENT;
}

static void edmacv310_enable_hw(struct edmacv310_dmadev *edmac)
{
	void __iomem *base = edmac->base;

	/* Clear and unmask the interrupts of all channels. */
	writel(EDMAC_ALL_CHAN_CLR, base + EDMAC_INT_TC1_RAW);
	writel(EDMAC_ALL_CHAN_CLR, base + EDMAC_INT_TC2_RAW);
	writel(EDMAC_ALL_CHAN_CLR, base + EDMAC_INT_ERR1_RAW);
	writel(EDMAC_ALL_CHAN_CLR, base + EDMAC_INT_ERR2_RAW);
	writel(EDMAC_ALL_CHAN_CLR, base + EDMAC_INT_ERR3_RAW);

	writel(EDMAC_INT_ENABLE_ALL, base + EDMAC_INT_TC1_MASK);
	writel(EDMAC_INT_ENABLE_ALL, base + EDMAC_INT_TC2_MASK);
	writel(EDMAC_INT_ENABLE_ALL, base + EDMAC_INT_ERR1_MASK);
	writel(EDMAC_INT_ENABLE_ALL, base + EDMAC_INT_ERR2_MASK);
	writel(EDMAC_INT_ENABLE_ALL, base + EDMAC_INT_ERR3_MASK);
}

static int edmacv310_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct edmacv310_dmadev *edmac;
	int ret;

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(64));
	if (ret)
		return ret;

	edmac = devm_kzalloc(dev, sizeof(*edmac), GFP_KERNEL);
	if (!edmac)
		return -ENOMEM;

	ret = device_property_read_u32(dev, "dma-channels", &edmac->channels);
	if (ret)
		return ret;

	ret = device_property_read_u32(dev, "dma-requests",
				       &edmac->slave_requests);
	if (ret)
		return ret;

	edmac->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(edmac->base))
		return PTR_ERR(edmac->base);

	edmac->irq = platform_get_irq(pdev, 0);
	if (edmac->irq < 0)
		return edmac->irq;

	edmac->clk = devm_clk_get_enabled(dev, "apb_pclk");
	if (IS_ERR(edmac->clk))
		return PTR_ERR(edmac->clk);

	edmac->axi_clk = devm_clk_get_enabled(dev, "axi_aclk");
	if (IS_ERR(edmac->axi_clk))
		return PTR_ERR(edmac->axi_clk);

	edmac->rstc = devm_reset_control_get_exclusive(dev, "dma-reset");
	if (IS_ERR(edmac->rstc))
		return PTR_ERR(edmac->rstc);

	reset_control_deassert(edmac->rstc);

	edmac->memcpy.dev = dev;
	edmac->slave.dev = dev;
	edmacv310_prep_dma_devices(edmac);

	edmac->pool = dmam_pool_create(DRIVER_NAME, dev, LLI_BLOCK_SIZE,
				       EDMAC_LLI_ALIGN, 0);
	if (!edmac->pool)
		return -ENOMEM;

	ret = edmacv310_init_channels(edmac);
	if (ret)
		return ret;

	ret = devm_request_irq(dev, edmac->irq, edmacv310_irq, 0,
			       DRIVER_NAME, edmac);
	if (ret)
		goto free_channels;

	edmacv310_enable_hw(edmac);

	ret = dma_async_device_register(&edmac->memcpy);
	if (ret)
		goto free_channels;

	ret = dma_async_device_register(&edmac->slave);
	if (ret)
		goto unregister_memcpy;

	ret = devm_of_dma_controller_register(dev, pdev->dev.of_node,
					      edmacv310_of_xlate, edmac);
	if (ret)
		goto unregister_slave;

	platform_set_drvdata(pdev, edmac);

	return 0;

unregister_slave:
	dma_async_device_unregister(&edmac->slave);
unregister_memcpy:
	dma_async_device_unregister(&edmac->memcpy);
free_channels:
	edmacv310_free_channels(edmac);

	return ret;
}

static void edmacv310_remove(struct platform_device *pdev)
{
	struct edmacv310_dmadev *edmac = platform_get_drvdata(pdev);

	dma_async_device_unregister(&edmac->slave);
	dma_async_device_unregister(&edmac->memcpy);
	edmacv310_free_channels(edmac);
}

static int __maybe_unused edmacv310_pm_suspend(struct device *dev)
{
	struct edmacv310_dmadev *edmac = dev_get_drvdata(dev);
	void __iomem *base = edmac->base;
	u32 stat;

	stat = readl(base + EDMAC_CH_STAT);
	if (stat) {
		dev_err(dev, "channels still active: 0x%x\n", stat);
		return -EBUSY;
	}

	writel(0, base + EDMAC_INT_TC1_MASK);
	writel(0, base + EDMAC_INT_TC2_MASK);
	writel(0, base + EDMAC_INT_ERR1_MASK);
	writel(0, base + EDMAC_INT_ERR2_MASK);
	writel(0, base + EDMAC_INT_ERR3_MASK);

	clk_disable_unprepare(edmac->clk);
	clk_disable_unprepare(edmac->axi_clk);

	return 0;
}

static int __maybe_unused edmacv310_pm_resume(struct device *dev)
{
	struct edmacv310_dmadev *edmac = dev_get_drvdata(dev);
	int ret;

	ret = clk_prepare_enable(edmac->clk);
	if (ret)
		return ret;

	ret = clk_prepare_enable(edmac->axi_clk);
	if (ret) {
		clk_disable_unprepare(edmac->clk);
		return ret;
	}

	edmacv310_enable_hw(edmac);

	return 0;
}

static const struct dev_pm_ops edmacv310_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(edmacv310_pm_suspend, edmacv310_pm_resume)
};

static const struct of_device_id edmacv310_match[] = {
	{ .compatible = "hisilicon,hi3519dv500-edmacv310" },
	{ }
};
MODULE_DEVICE_TABLE(of, edmacv310_match);

static struct platform_driver edmacv310_driver = {
	.probe = edmacv310_probe,
	.remove = edmacv310_remove,
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = edmacv310_match,
		.pm = pm_ptr(&edmacv310_pm_ops),
	},
};

module_platform_driver(edmacv310_driver);

MODULE_AUTHOR("Jun Yan <jerrysteve1101@gmail.com>");
MODULE_DESCRIPTION("HiSilicon EDMAC v310 DMA driver");
MODULE_LICENSE("GPL");
