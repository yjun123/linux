// SPDX-License-Identifier: GPL-2.0-only
/*
 * HiSilicon GEMAC v5 Ethernet MAC driver
 *
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023.
 * Copyright (c) 2026 Jun Yan <jerrysteve1101@gmail.com>
 *
 * Ported from the HiSilicon vendor SDK to the mainline kernel.
 */

#include <linux/clk.h>
#include <linux/circ_buf.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/etherdevice.h>
#include <linux/if_vlan.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/ip.h>
#include <linux/mdio.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/of_mdio.h>
#include <linux/of_net.h>
#include <linux/phy.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/sched/clock.h>
#include <linux/skbuff.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <net/gso.h>
#include <net/ipv6.h>

/* MAC registers */
#define STATION_ADDR_LOW		0x0000
#define STATION_ADDR_HIGH		0x0004
#define MAC_DUPLEX_HALF_CTRL		0x0008

#define PORT_MODE			0x0040
#define GMAC_SPEED_1000			0x05
#define GMAC_SPEED_100			0x01
#define GMAC_SPEED_10			0x00

#define PORT_EN				0x0044
#define BITS_TX_EN			BIT(2)
#define BITS_RX_EN			BIT(1)

#define FC_TX_TIMER			0x001c
#define PAUSE_THR			0x0038
#define PAUSE_EN			0x0048
#define BIT_RX_FDFC			BIT(0)
#define BIT_TX_FDFC			BIT(1)

#define REC_FILT_CONTROL		0x0064
#define BIT_CRC_ERR_PASS		BIT(5)
#define BIT_PAUSE_FRM_PASS		BIT(4)
#define BIT_VLAN_DROP_EN		BIT(3)
#define BIT_BC_DROP_EN			BIT(2)
#define BIT_MC_MATCH_EN			BIT(1)
#define BIT_UC_MATCH_EN			BIT(0)

#define PORT_MC_ADDR_LOW		0x0068
#define PORT_MC_ADDR_HIGH		0x006c
#define MAC_CLEAR			0x0070
#define BIT_TX_SOFT_RESET		BIT(0)

#define MODE_CHANGE_EN			0x01b4
#define BIT_MODE_CHANGE_EN		BIT(0)

#define COL_SLOT_TIME			0x01c0

#define CRF_MIN_PACKET			0x0210
#define BIT_OFFSET_TX_MIN_LEN		8
#define BIT_MASK_TX_MIN_LEN		GENMASK(13, 8)
#define BITS_CRF_MIN_PACKET_TSO_VER	GENMASK(31, 28)
#define BIT_CRF_MIN_PACKET_RXHASH	BIT(30)
#define BIT_CRF_MIN_PACKET_RSS		BIT(31)

#define CONTROL_WORD			0x0214
#define CONTROL_WORD_CONFIG		0x640

#define TSO_COE_CTRL			0x02e8
#define BIT_COE_IPHDR_DROP		BIT(4)
#define BIT_COE_PAYLOAD_DROP		BIT(5)
#define BIT_COE_IPV6_UDP_ZERO_DROP	BIT(6)
#define COE_ERR_DROP			(BIT_COE_IPHDR_DROP | \
					 BIT_COE_PAYLOAD_DROP | \
					 BIT_COE_IPV6_UDP_ZERO_DROP)

#define RX_PAUSE_EN			0x02a4
#define BIT_RX_FQ_PAUSE_EN		BIT(0)
#define BIT_RX_BQ_PAUSE_EN		BIT(1)

#define CRF_TX_PAUSE			0x0340
#define BITS_Q_PAUSE_TH_OFFSET		16
#define BITS_Q_PAUSE_TH_MASK		0xffff

/* RX frame queue */
#define RX_FQ_START_ADDR		0x0500
#define RX_FQ_DEPTH			0x0504
#define RX_FQ_WR_ADDR			0x0508
#define RX_FQ_RD_ADDR			0x050c
#define RX_FQ_ALEMPTY_TH		0x0514
#define RX_FQ_ALFULL_TH			0x051c

/* RX buffer queue */
#define RX_BQ_START_ADDR		0x0520
#define RX_BQ_DEPTH			0x0524
#define RX_BQ_WR_ADDR			0x0528
#define RX_BQ_RD_ADDR			0x052c
#define RX_BQ_ALEMPTY_TH		0x0534
#define RX_BQ_ALFULL_TH			0x053c

/* TX buffer queue */
#define TX_BQ_START_ADDR		0x0580
#define TX_BQ_DEPTH			0x0584
#define TX_BQ_WR_ADDR			0x0588
#define TX_BQ_RD_ADDR			0x058c
#define TX_BQ_ALEMPTY_TH		0x0594
#define TX_BQ_ALFULL_TH			0x059c

/* TX return queue */
#define TX_RQ_START_ADDR		0x05a0
#define TX_RQ_DEPTH			0x05a4
#define TX_RQ_WR_ADDR			0x05a8
#define TX_RQ_RD_ADDR			0x05ac
#define TX_RQ_ALEMPTY_TH		0x05b4
#define TX_RQ_ALFULL_TH			0x05bc

#define QUEUE_REG_EN			0x0518
#define BITS_Q_START_ADDR_EN		BIT(2)
#define BITS_Q_DEPTH_EN			BIT(1)
#define RX_FQ_REG_EN			0x0518
#define RX_BQ_REG_EN			0x0538
#define TX_BQ_REG_EN			0x0598
#define TX_RQ_REG_EN			0x05b8

#define RAW_PMU_INT			0x05c0
#define ENA_PMU_INT			0x05c4
#define DESC_WR_RD_ENA			0x05cc

#define IN_QUEUE_TH			0x05d8
#define BITS_OFFSET_TX_RQ_IN_TH		16

#define RX_BQ_IN_TIMEOUT_TH		0x05e0
#define TX_RQ_IN_TIMEOUT_TH		0x05e4

#define STOP_CMD			0x05e8
#define BITS_TX_STOP_EN			BIT(1)
#define BITS_RX_STOP_EN			BIT(0)
#define STOP_RX_TX			(BITS_TX_STOP_EN | BITS_RX_STOP_EN)

#define RX_BQ_IN_INT			BIT(17)
#define TX_RQ_IN_INT			BIT(19)
#define RX_BQ_IN_TIMEOUT_INT		BIT(28)
#define TX_RQ_IN_TIMEOUT_INT		BIT(29)
#define DEF_INT_MASK			(RX_BQ_IN_INT | RX_BQ_IN_TIMEOUT_INT | \
					 TX_RQ_IN_INT | TX_RQ_IN_TIMEOUT_INT)

/* RSS registers */
#define HW_CAP_EN			0x0c00
#define BIT_RSS_CAP			BIT(0)
#define BIT_RXHASH_CAP			BIT(1)
#define RSS_HASH_KEY			0x0c04
#define RSS_HASH_CONFIG			0x0c08
#define RSS_IND_TBL			0x0c0c
#define BIT_IND_TBL_READY		BIT(13)
#define BIT_IND_TLB_WR			BIT(12)
#define RSS_RAW_PMU_INT			0x0c10
#define RSS_QUEUE1_START_ADDR		0x0c20
#define RSS_QUEUE1_DEPTH		0x0c24
#define RX_BQ_WR_ADDR_QUEUE1		0x0c28
#define RX_BQ_RD_ADDR_QUEUE1		0x0c2c
#define RSS_QUEUE1_ENA_INT		0x0c90

#define TCPV4_L3_HASH_EN		BIT(0)
#define TCPV4_L4_HASH_EN		BIT(1)
#define TCPV4_VLAN_HASH_EN		BIT(2)
#define UDPV4_L3_HASH_EN		BIT(4)
#define UDPV4_L4_HASH_EN		BIT(5)
#define UDPV4_VLAN_HASH_EN		BIT(6)
#define IPV4_L3_HASH_EN			BIT(8)
#define IPV4_VLAN_HASH_EN		BIT(9)
#define TCPV6_L3_HASH_EN		BIT(12)
#define TCPV6_L4_HASH_EN		BIT(13)
#define TCPV6_VLAN_HASH_EN		BIT(14)
#define UDPV6_L3_HASH_EN		BIT(16)
#define UDPV6_L4_HASH_EN		BIT(17)
#define UDPV6_VLAN_HASH_EN		BIT(18)
#define IPV6_L3_HASH_EN			BIT(20)
#define IPV6_VLAN_HASH_EN		BIT(21)
#define DEF_HASH_CFG			0x377377

/* MACIF interface mode values */
#define RGMII_SPEED_1000		0x2c
#define RGMII_SPEED_100			0x2f
#define RGMII_SPEED_10			0x2d
#define MII_SPEED_100			0x0f
#define MII_SPEED_10			0x0d
#define RMII_SPEED_100			0x8f
#define RMII_SPEED_10			0x8d
#define GMAC_FULL_DUPLEX		BIT(4)

/* TSO descriptor flags */
#define SG_FLAG				BIT(30)
#define COE_FLAG			BIT(29)
#define TSO_FLAG			BIT(28)
#define VLAN_FLAG			BIT(10)
#define IPV6_FLAG			BIT(9)
#define UDP_FLAG			BIT(8)

#define PKT_IPV6_HDR_LEN		10
#define PKT_UDP_HDR_LEN			2
#define WORD_TO_BYTE			4
#define IPV4_HEAD_LENGTH		0x5

enum {
	PKT_NORMAL,
	PKT_SG,
};

enum {
	PKT_IPV4,
	PKT_IPV6,
};

enum {
	PKT_TCP,
	PKT_UDP,
};

enum gmac_tx_err {
	ERR_NONE = 0,
	ERR_DESC_CFG = BIT(0),
	ERR_DATA_LEN = BIT(1),
	ERR_DESC_NFRAG_NUM = BIT(2),
	ERR_DESC_IP_HDR_LEN = BIT(3),
	ERR_DESC_PROT_HDR_LEN = BIT(4),
	ERR_DESC_MTU = BIT(5),
	ERR_LINK_SGPKT_LEN = BIT(8),
	ERR_LINK_TSOPKT_LINEAR = BIT(9),
	ERR_LINK_NFRAG_LEN = BIT(10),
	ERR_LINK_TOTAL_LEN = BIT(11),
	ERR_HDR_TCP_BCMC = BIT(12),
	ERR_HDR_UDP_BC = BIT(13),
	ERR_HDR_VLAN_IP_TYPE = BIT(14),
	ERR_HDR_IP_TYPE = BIT(15),
	ERR_HDR_IP_VERSION = BIT(16),
	ERR_HDR_IP_HDR_LEN = BIT(17),
	ERR_HDR_IP_TOTAL_LEN = BIT(18),
	ERR_HDR_IPV6_TTL_PROT = BIT(19),
	ERR_HDR_IPV4_OFFSET = BIT(20),
	ERR_HDR_IPV4_TTL_PROT = BIT(21),
	ERR_HDR_UDP_LEN = BIT(22),
	ERR_HDR_TCP_LEN = BIT(23),
	ERR_DESC = (ERR_DESC_CFG | ERR_DATA_LEN |
		    ERR_DESC_NFRAG_NUM | ERR_DESC_IP_HDR_LEN |
		    ERR_DESC_PROT_HDR_LEN | ERR_DESC_MTU),
	ERR_LINK = (ERR_LINK_SGPKT_LEN | ERR_LINK_TSOPKT_LINEAR |
		    ERR_LINK_NFRAG_LEN | ERR_LINK_TOTAL_LEN),
	ERR_HDR = (ERR_HDR_TCP_BCMC | ERR_HDR_UDP_BC |
		   ERR_HDR_VLAN_IP_TYPE | ERR_HDR_IP_TYPE |
		   ERR_HDR_IP_VERSION | ERR_HDR_IP_HDR_LEN |
		   ERR_HDR_IP_TOTAL_LEN | ERR_HDR_IPV6_TTL_PROT |
		   ERR_HDR_IPV4_OFFSET | ERR_HDR_IPV4_TTL_PROT |
		   ERR_HDR_UDP_LEN | ERR_HDR_TCP_LEN),
	ERR_ALL = (ERR_DESC | ERR_LINK | ERR_HDR),
};

#define GMAC_DRIVER_NAME		"hisi-gmac-v5"
#define GMAC_MAC_CLK_NAME		"gmac_clk"
#define GMAC_MACIF_CLK_NAME		"macif_clk"
#define GMAC_PORT_RST_NAME		"port_reset"
#define GMAC_MACIF_RST_NAME		"macif_reset"
#define GMAC_PHY_RST_NAME		"phy_reset"

#define VENDOR_PHY_ID_FESTAV200		0x20669823

/* DMA address high 8 bits, carried in queue depth and descriptor fields */
#define REG_BIT_WIDTH			32
#define Q_ADDR_HI8_OFFSET		24
#define Q_ADDR_HI8_MASK			(BIT(Q_ADDR_HI8_OFFSET) - 1)
#define TX_DESC_HI8_MASK		0xff
#define SG_DESC_HI8_OFFSET		8

/* write or read descriptor needs memory barrier */
#define gmac_sync_barrier()		do { isb(); smp_mb(); } while (0)

enum {
	MEM_GMAC_IOBASE,
	MEM_MACIF_IOBASE,
};

/* Flow control */
#define FLOW_OFF			0
#define FLOW_RX				1
#define FLOW_TX				2
#define FLOW_AUTO			(FLOW_TX | FLOW_RX)

#define FC_ACTIVE_DEFAULT		16
#define FC_DEACTIVE_DEFAULT		32
#define FC_PAUSE_TIME_DEFAULT		0xffff
#define FC_PAUSE_INTERVAL_DEFAULT	0xffff

#define RX_BQ_INT_THRESHOLD		0x40
#define TX_RQ_INT_THRESHOLD		0x20

#define GMAC_MONITOR_TIMER		(msecs_to_jiffies(200))

#define ETH_MAX_FRAME_SIZE		(1600 + 128)

#define DESC_VLD_FREE			0
#define DESC_VLD_BUSY			1

#define DESC_FL_FIRST			2
#define DESC_FL_MID			0
#define DESC_FL_LAST			1
#define DESC_FL_FULL			3

/*
 * The GEMAC v5 uses 4-word descriptors, carries the receive checksum
 * status inside them and addresses the full 64 bit DDR space through
 * the high 8 bits of the queue depth registers and descriptor fields.
 */
#define DESC_WORD_SHIFT			2
#define DESC_BYTE_SHIFT			(DESC_WORD_SHIFT + 2)
#define DESC_WORD_CNT			BIT(DESC_WORD_SHIFT)
#define DESC_SIZE			BIT(DESC_BYTE_SHIFT)

#define RX_DESC_NUM			1024
#define TX_DESC_NUM			1024

/* DMA descriptor ring helpers */
#define dma_ring_incr(n, s)		(((n) + 1) & ((s) - 1))
#define dma_cnt(n)			((n) >> DESC_BYTE_SHIFT)
#define dma_byte(n)			((n) << DESC_BYTE_SHIFT)

#define RSS_HASH_KEY_SIZE		4
#define RSS_INDIRECTION_TABLE_SIZE	128
#define RSS_NUM_RXQS			4

#define QUEUE_NUMS			4
#define BASE_QUEUE_NUMS			3

struct frags_info {
	/* Word(2*i+2) */
	u32 addr;
	/* Word(2*i+3) */
	u32 size : 16;
	u32 reserved : 16;
};

struct sg_desc {
	/* Word0 */
	u32 total_len : 17;
	u32 reserv : 15;
	/* Word1 */
	u32 ipv6_id;
	/* Word2 */
	u32 linear_addr;
	/* Word3 */
	u32 linear_len : 16;
	u32 reserv3 : 16;
	/* MAX_SKB_FRAGS is 18 */
	struct frags_info frags[18];
};

struct gmac_desc {
	u32 data_buff_addr;

	u32 buffer_len : 11;
	u32 reserve2 : 1;
	u32 payload_csum_err : 1;
	u32 header_csum_err : 1;
	u32 payload_csum_done : 1;
	u32 header_csum_done : 1;
	u32 data_len : 11;
	u32 reserve1 : 2;
	u32 fl : 2;
	u32 descvid : 1;

	u32 rxhash;
	u32 reserve3 : 8;
	u32 l3_hash : 1;
	u32 has_hash : 1;
	u32 skb_id : 14;
	u32 reserve31 : 8;
};

struct gmac_tso_desc {
	u32 data_buff_addr;
	union {
		struct {
			u32 prot_hdr_len : 4;
			u32 ip_hdr_len : 4;
			u32 prot_type : 1;
			u32 ip_ver : 1;
			u32 vlan_flag : 1;
			u32 nfrags_num : 5;
			u32 data_len : 11;
			u32 reservel : 1;
			u32 tso_flag : 1;
			u32 coe_flag : 1;
			u32 sg_flag : 1;
			u32 hw_own : 1;
		} tx;
		u32 val;
	} desc1;
	u32 reserve_desc2;
	u32 tx_err;
};

#define SKB_MAGIC	((struct sk_buff *)0x5a)

struct gmac_napi {
	struct napi_struct napi;
	struct gmac_netdev_local *ndev_priv;
	int rxq_id;
};

struct gmac_rss_info {
	u32 hash_cfg;
	u32 ind_tbl_size;
	u8 ind_tbl[RSS_INDIRECTION_TABLE_SIZE];
	u8 key[RSS_HASH_KEY_SIZE];
};

struct gmac_netdev_local {
#define GMAC_SG_DESC_ADD		64U
	struct sg_desc *dma_sg_desc ____cacheline_aligned;
	dma_addr_t dma_sg_phy;
	unsigned int sg_head;
	unsigned int sg_tail;
	unsigned int sg_count;

	void __iomem *gmac_iobase;
	void __iomem *macif_base;

	u32 hw_cap;
	bool tso_supported;
	bool has_rxhash_cap;
	bool has_rss_cap;
	int num_rxqs;
	struct gmac_napi q_napi[RSS_NUM_RXQS];
	int irq[RSS_NUM_RXQS];
	struct gmac_rss_info rss_info;

	struct reset_control *port_rst;
	struct reset_control *macif_rst;
	struct reset_control *phy_rst;

	struct {
		struct gmac_desc *desc;
		dma_addr_t phys_addr;
		int *sg_desc_offset;
		/* how many desc in the desc pool */
		unsigned int count;
		struct sk_buff **skb;
		unsigned int size;
	} pool[QUEUE_NUMS + RSS_NUM_RXQS - 1];
#define RX_FQ		pool[0]
#define RX_BQ		pool[1]
#define TX_BQ		pool[2]
#define TX_RQ		pool[3]

	struct sk_buff **tx_skb;
	struct sk_buff **rx_skb;

	struct device *dev;
	struct net_device *netdev;
	struct clk *clk;
	struct clk *macif_clk;

	struct timer_list monitor;

	struct phy_device *phy;
	struct device_node *phy_node;
	phy_interface_t phy_mode;

	unsigned int flow_ctrl;
	unsigned int pause;
	unsigned int pause_interval;
	unsigned int flow_ctrl_active_threshold;
	unsigned int flow_ctrl_deactive_threshold;

	int old_link;
	int old_speed;
	int old_duplex;

	/* receive packet lock */
	spinlock_t rxlock;
	/* transmit packet lock */
	spinlock_t txlock;

	u32 msg_enable;
};

enum tso_version {
	VER_NO_TSO = 0x0,
	VER_BYTE_SPLICE = 0x1,
	VER_SG_COE = 0x2,
	VER_TSO = 0x3,
};

struct cyclic_queue_info {
	u32 start;
	u32 end;
	u32 num;
	u32 pos;
};

#define has_tso_cap(hw_cap)		((((hw_cap) >> 28) & 0x3) == VER_TSO)
#define has_rxhash_cap(hw_cap)		((hw_cap) & BIT(30))
#define has_rss_cap(hw_cap)		((hw_cap) & BIT(31))

#define DEFAULT_MSG_ENABLE	(NETIF_MSG_DRV | NETIF_MSG_PROBE | \
				 NETIF_MSG_LINK)
static int debug = -1;
module_param(debug, int, 0);
MODULE_PARM_DESC(debug, "Debug level (0=none,...,16=all)");

static void gmac_irq_enable(struct gmac_netdev_local *ld)
{
	writel(RX_BQ_IN_INT | RX_BQ_IN_TIMEOUT_INT | TX_RQ_IN_INT |
	       TX_RQ_IN_TIMEOUT_INT, ld->gmac_iobase + ENA_PMU_INT);
}

static void gmac_irq_enable_queue(struct gmac_netdev_local *ld,
				  unsigned int rxq_id)
{
	if (rxq_id) {
		const unsigned int reg = RSS_QUEUE1_ENA_INT + (rxq_id - 1) * 4;

		writel(~0, ld->gmac_iobase + reg);
	} else {
		gmac_irq_enable(ld);
	}
}

static void gmac_irq_enable_all_queue(struct gmac_netdev_local *ld)
{
	unsigned int i;

	for (i = 0; i < ld->num_rxqs; i++)
		gmac_irq_enable_queue(ld, i);
}

static void gmac_irq_disable(struct gmac_netdev_local *ld)
{
	writel(0, ld->gmac_iobase + ENA_PMU_INT);
}

static void gmac_irq_disable_queue(struct gmac_netdev_local *ld, int rxq_id)
{
	if (rxq_id) {
		const unsigned int reg = RSS_QUEUE1_ENA_INT + (rxq_id - 1) * 4;

		writel(0, ld->gmac_iobase + reg);
	} else {
		gmac_irq_disable(ld);
	}
}

static void gmac_irq_disable_all_queue(struct gmac_netdev_local *ld)
{
	int i;

	for (i = 0; i < ld->num_rxqs; i++)
		gmac_irq_disable_queue(ld, i);
}

static bool gmac_queue_irq_disabled(struct gmac_netdev_local *ld, int rxq_id)
{
	u32 reg;

	if (rxq_id)
		reg = RSS_QUEUE1_ENA_INT + (rxq_id - 1) * 4;
	else
		reg = ENA_PMU_INT;

	return !readl(ld->gmac_iobase + reg);
}

static void gmac_hw_desc_enable(struct gmac_netdev_local *ld)
{
	writel(0xf, ld->gmac_iobase + DESC_WR_RD_ENA);
}

static void gmac_hw_desc_disable(struct gmac_netdev_local *ld)
{
	writel(0, ld->gmac_iobase + DESC_WR_RD_ENA);
}

static void gmac_port_enable(struct gmac_netdev_local *ld)
{
	writel(BITS_TX_EN | BITS_RX_EN, ld->gmac_iobase + PORT_EN);
}

/* RSS queue helpers */
static u32 rx_bq_start_addr_queue(int i)
{
	return RSS_QUEUE1_START_ADDR + (i - 1) * 0x10;
}

static u32 rx_bq_depth_queue(int i)
{
	return RSS_QUEUE1_DEPTH + (i - 1) * 0x10;
}

static u32 rx_bq_wr_addr_queue(int i)
{
	return i ? RX_BQ_WR_ADDR_QUEUE1 + (i - 1) * 0x10 : RX_BQ_WR_ADDR;
}

static u32 rx_bq_rd_addr_queue(int i)
{
	return i ? RX_BQ_RD_ADDR_QUEUE1 + (i - 1) * 0x10 : RX_BQ_RD_ADDR;
}

static u32 def_int_mask_queue(int i)
{
	return 0x3 << (2 * (i - 1));
}

static void gmac_set_queue_depth(struct gmac_netdev_local *ld, u32 reg_en,
				 u32 depth_reg, u32 depth)
{
	u32 val;

	writel(BITS_Q_DEPTH_EN, ld->gmac_iobase + reg_en);
	val = readl(ld->gmac_iobase + depth_reg);
	val &= ~Q_ADDR_HI8_MASK;
	val |= depth << DESC_WORD_SHIFT;
	writel(val, ld->gmac_iobase + depth_reg);
	writel(0, ld->gmac_iobase + reg_en);
}

static void gmac_set_queue_addr(struct gmac_netdev_local *ld, u32 reg_en,
				u32 depth_reg, u32 start_reg, u32 addr_en,
				dma_addr_t phys_addr)
{
	u32 val;

	writel(addr_en, ld->gmac_iobase + reg_en);
	val = readl(ld->gmac_iobase + depth_reg);
	val &= Q_ADDR_HI8_MASK;
	val |= (phys_addr >> REG_BIT_WIDTH) << Q_ADDR_HI8_OFFSET;
	writel(val, ld->gmac_iobase + depth_reg);
	writel((u32)phys_addr, ld->gmac_iobase + start_reg);
	writel(0, ld->gmac_iobase + reg_en);
}

static void gmac_set_desc_depth(struct gmac_netdev_local *ld, u32 rx, u32 tx)
{
	int i;

	gmac_set_queue_depth(ld, RX_FQ_REG_EN, RX_FQ_DEPTH, rx);
	gmac_set_queue_depth(ld, RX_BQ_REG_EN, RX_BQ_DEPTH, rx);
	for (i = 1; i < ld->num_rxqs; i++)
		gmac_set_queue_depth(ld, RX_BQ_REG_EN, rx_bq_depth_queue(i), rx);
	gmac_set_queue_depth(ld, TX_BQ_REG_EN, TX_BQ_DEPTH, tx);
	gmac_set_queue_depth(ld, TX_RQ_REG_EN, TX_RQ_DEPTH, tx);
}

static void gmac_set_rx_fq(struct gmac_netdev_local *ld, dma_addr_t phys_addr)
{
	gmac_set_queue_addr(ld, RX_FQ_REG_EN, RX_FQ_DEPTH, RX_FQ_START_ADDR,
			    BITS_Q_START_ADDR_EN, phys_addr);
}

static void gmac_set_rx_bq(struct gmac_netdev_local *ld, dma_addr_t phys_addr)
{
	gmac_set_queue_addr(ld, RX_BQ_REG_EN, RX_BQ_DEPTH, RX_BQ_START_ADDR,
			    BITS_Q_START_ADDR_EN, phys_addr);
}

static void gmac_set_tx_bq(struct gmac_netdev_local *ld, dma_addr_t phys_addr)
{
	gmac_set_queue_addr(ld, TX_BQ_REG_EN, TX_BQ_DEPTH, TX_BQ_START_ADDR,
			    BITS_Q_START_ADDR_EN, phys_addr);
}

static void gmac_set_tx_rq(struct gmac_netdev_local *ld, dma_addr_t phys_addr)
{
	gmac_set_queue_addr(ld, TX_RQ_REG_EN, TX_RQ_DEPTH, TX_RQ_START_ADDR,
			    BITS_Q_START_ADDR_EN, phys_addr);
}

static void gmac_hw_set_desc_addr(struct gmac_netdev_local *ld)
{
	u32 val;
	int i;

	gmac_set_rx_fq(ld, ld->RX_FQ.phys_addr);
	gmac_set_rx_bq(ld, ld->RX_BQ.phys_addr);
	gmac_set_tx_rq(ld, ld->TX_RQ.phys_addr);
	gmac_set_tx_bq(ld, ld->TX_BQ.phys_addr);

	for (i = 1; i < ld->num_rxqs; i++) {
		dma_addr_t phys = ld->pool[BASE_QUEUE_NUMS + i].phys_addr;

		writel(BITS_Q_START_ADDR_EN | BITS_Q_DEPTH_EN,
		       ld->gmac_iobase + RX_BQ_REG_EN);
		/*
		 * The RX_BQ depth register doubles as the carrier of the
		 * queue start address high 8 bits, so the depth enable is
		 * required to write them.
		 */
		val = readl(ld->gmac_iobase + rx_bq_depth_queue(i));
		val &= Q_ADDR_HI8_MASK;
		val |= (phys >> REG_BIT_WIDTH) << Q_ADDR_HI8_OFFSET;
		writel(val, ld->gmac_iobase + rx_bq_depth_queue(i));
		writel((u32)phys, ld->gmac_iobase + rx_bq_start_addr_queue(i));
		writel(0, ld->gmac_iobase + RX_BQ_REG_EN);
	}
}

static void gmac_set_rss_cap(struct gmac_netdev_local *ld)
{
	u32 val = 0;

	if (ld->has_rxhash_cap)
		val |= BIT_RXHASH_CAP;
	if (ld->has_rss_cap)
		val |= BIT_RSS_CAP;
	writel(val, ld->gmac_iobase + HW_CAP_EN);
}

static void gmac_hw_init(struct gmac_netdev_local *ld)
{
	u32 val;
	int i;

	/* disable and clear all interrupts */
	writel(0, ld->gmac_iobase + ENA_PMU_INT);
	writel(~0, ld->gmac_iobase + RAW_PMU_INT);
	for (i = 1; i < ld->num_rxqs; i++)
		writel(0, ld->gmac_iobase + RSS_QUEUE1_ENA_INT + (i - 1) * 4);
	writel(~0, ld->gmac_iobase + RSS_RAW_PMU_INT);

	/* enable CRC erro packets filter */
	val = readl(ld->gmac_iobase + REC_FILT_CONTROL);
	val |= BIT_CRC_ERR_PASS;
	writel(val, ld->gmac_iobase + REC_FILT_CONTROL);

	/* set tx min packet length */
	val = readl(ld->gmac_iobase + CRF_MIN_PACKET);
	val &= ~BIT_MASK_TX_MIN_LEN;
	val |= ETH_HLEN << BIT_OFFSET_TX_MIN_LEN;
	writel(val, ld->gmac_iobase + CRF_MIN_PACKET);

	/* fix bug for udp and ip error check */
	writel(CONTROL_WORD_CONFIG, ld->gmac_iobase + CONTROL_WORD);

	writel(0, ld->gmac_iobase + COL_SLOT_TIME);
	writel(DUPLEX_HALF, ld->gmac_iobase + MAC_DUPLEX_HALF_CTRL);

	/* interrupt when rcv packets >= RX_BQ_INT_THRESHOLD */
	val = RX_BQ_INT_THRESHOLD |
	      (TX_RQ_INT_THRESHOLD << BITS_OFFSET_TX_RQ_IN_TH);
	writel(val, ld->gmac_iobase + IN_QUEUE_TH);

	/* RX_BQ/TX_RQ in timeout threshold */
	writel(0x10000, ld->gmac_iobase + RX_BQ_IN_TIMEOUT_TH);
	writel(0x18000, ld->gmac_iobase + TX_RQ_IN_TIMEOUT_TH);

	gmac_set_desc_depth(ld, RX_DESC_NUM, TX_DESC_NUM);
}

/*
 * Stop the hw descriptors and reclaim the software skb resources.
 * Before reusing the gmac, the gmac must be reset.
 */
static void gmac_reclaim_rx_tx_resource(struct gmac_netdev_local *ld)
{
	unsigned long rxflags, txflags;
	int rd_offset, wr_offset;
	int i;

	gmac_irq_disable_all_queue(ld);
	gmac_hw_desc_disable(ld);
	writel(STOP_RX_TX, ld->gmac_iobase + STOP_CMD);

	spin_lock_irqsave(&ld->rxlock, rxflags);
	/* RX_BQ: logic write pointer */
	wr_offset = readl(ld->gmac_iobase + RX_BQ_WR_ADDR);
	/* prevent to reclaim skb in rx bottom half */
	writel(wr_offset, ld->gmac_iobase + RX_BQ_RD_ADDR);

	for (i = 1; i < ld->num_rxqs; i++) {
		wr_offset = readl(ld->gmac_iobase + rx_bq_wr_addr_queue(i));
		writel(wr_offset, ld->gmac_iobase + rx_bq_rd_addr_queue(i));
	}

	/* RX_FQ: logic read pointer */
	rd_offset = readl(ld->gmac_iobase + RX_FQ_RD_ADDR);
	if (rd_offset == 0)
		rd_offset = (RX_DESC_NUM - 1) << DESC_BYTE_SHIFT;
	else
		rd_offset -= DESC_SIZE;
	/* stop to feed hw desc */
	writel(rd_offset, ld->gmac_iobase + RX_FQ_WR_ADDR);

	for (i = 0; i < ld->RX_FQ.count; i++) {
		if (!ld->RX_FQ.skb[i])
			ld->RX_FQ.skb[i] = SKB_MAGIC;
	}
	spin_unlock_irqrestore(&ld->rxlock, rxflags);

	/*
	 * No need to wait for pkts in TX_RQ to finish to free all skb,
	 * because gmac_xmit_reclaim is under the tx lock.
	 */
	spin_lock_irqsave(&ld->txlock, txflags);
	/* TX_RQ: logic write */
	wr_offset = readl(ld->gmac_iobase + TX_RQ_WR_ADDR);
	/* stop to reclaim tx skb */
	writel(wr_offset, ld->gmac_iobase + TX_RQ_RD_ADDR);

	/* TX_BQ: logic read */
	rd_offset = readl(ld->gmac_iobase + TX_BQ_RD_ADDR);
	if (rd_offset == 0)
		rd_offset = (TX_DESC_NUM - 1) << DESC_BYTE_SHIFT;
	else
		rd_offset -= DESC_SIZE;
	/* stop software tx skb */
	writel(rd_offset, ld->gmac_iobase + TX_BQ_WR_ADDR);

	for (i = 0; i < ld->TX_BQ.count; i++) {
		if (!ld->TX_BQ.skb[i])
			ld->TX_BQ.skb[i] = SKB_MAGIC;
	}
	spin_unlock_irqrestore(&ld->txlock, txflags);
}

static void gmac_hw_set_mac_addr(struct net_device *dev)
{
	struct gmac_netdev_local *ld = netdev_priv(dev);
	unsigned char *mac = (unsigned char *)dev->dev_addr;
	u32 val;

	val = mac[1] | (mac[0] << 8); /* mac[1]->(7, 0) mac[0]->(15, 8) */
	writel(val, ld->gmac_iobase + STATION_ADDR_HIGH);
	val = mac[5] | (mac[4] << 8) | /* mac[5]->(7, 0) mac[4]->(8, 15) */
	      (mac[3] << 16) | (mac[2] << 24); /* mac[3]->(23, 16) mac[2]->(31, 24) */
	writel(val, ld->gmac_iobase + STATION_ADDR_LOW);
}

static void gmac_free_rx_skb(struct gmac_netdev_local *ld)
{
	struct sk_buff *skb;
	int i;

	for (i = 0; i < ld->RX_FQ.count; i++) {
		skb = ld->RX_FQ.skb[i];
		if (skb) {
			ld->rx_skb[i] = NULL;
			ld->RX_FQ.skb[i] = NULL;
			if (skb == SKB_MAGIC)
				continue;
			dev_kfree_skb_any(skb);
		}
	}
}

static void gmac_free_tx_skb(struct gmac_netdev_local *ld)
{
	struct sk_buff *skb;
	int i;

	for (i = 0; i < ld->TX_BQ.count; i++) {
		skb = ld->TX_BQ.skb[i];
		if (skb) {
			ld->tx_skb[i] = NULL;
			ld->TX_BQ.skb[i] = NULL;
			if (skb == SKB_MAGIC)
				continue;
			netdev_completed_queue(ld->netdev, 1, skb->len);
			dev_kfree_skb_any(skb);
		}
	}
}

static void gmac_mac_core_reset(struct gmac_netdev_local *ld)
{
	/* undo reset */
	reset_control_deassert(ld->port_rst);
	usleep_range(50, 60);

	/* soft reset mac port */
	reset_control_assert(ld->port_rst);
	usleep_range(50, 60);
	/* undo reset */
	reset_control_deassert(ld->port_rst);
}

#define GMAC_MS_TO_NS (1000000ULL)
#define GMAC_FLUSH_WAIT_TIME (100 * GMAC_MS_TO_NS)

static void gmac_linkup_flush(struct gmac_netdev_local *ld)
{
	int tx_bq_wr_offset, tx_bq_rd_offset;
	unsigned long long time_limit, time_now;

	time_now = sched_clock();
	time_limit = time_now + GMAC_FLUSH_WAIT_TIME;

	do {
		tx_bq_wr_offset = readl(ld->gmac_iobase + TX_BQ_WR_ADDR);
		tx_bq_rd_offset = readl(ld->gmac_iobase + TX_BQ_RD_ADDR);

		time_now = sched_clock();
		if (unlikely(((long long)time_now - (long long)time_limit) >= 0))
			break;
	} while (tx_bq_rd_offset != tx_bq_wr_offset);

	mdelay(1);
}

static void gmac_mac_tx_state_engine_reset(struct gmac_netdev_local *ld)
{
	u32 val;

	val = readl(ld->gmac_iobase + MAC_CLEAR);
	val |= BIT_TX_SOFT_RESET;
	writel(val, ld->gmac_iobase + MAC_CLEAR);

	mdelay(5); /* wait 5ms */

	val = readl(ld->gmac_iobase + MAC_CLEAR);
	val &= ~BIT_TX_SOFT_RESET;
	writel(val, ld->gmac_iobase + MAC_CLEAR);
}

static void gmac_config_port(struct net_device *dev, u32 speed, u32 duplex)
{
	struct gmac_netdev_local *ld = netdev_priv(dev);
	u32 val;

	switch (ld->phy_mode) {
	case PHY_INTERFACE_MODE_RGMII:
	case PHY_INTERFACE_MODE_RGMII_ID:
	case PHY_INTERFACE_MODE_RGMII_RXID:
	case PHY_INTERFACE_MODE_RGMII_TXID:
		if (speed == SPEED_1000)
			val = RGMII_SPEED_1000;
		else if (speed == SPEED_100)
			val = RGMII_SPEED_100;
		else
			val = RGMII_SPEED_10;
		break;
	case PHY_INTERFACE_MODE_MII:
		if (speed == SPEED_100)
			val = MII_SPEED_100;
		else
			val = MII_SPEED_10;
		break;
	case PHY_INTERFACE_MODE_RMII:
		if (speed == SPEED_100)
			val = RMII_SPEED_100;
		else
			val = RMII_SPEED_10;
		break;
	default:
		netdev_warn(dev, "not supported mode\n");
		val = MII_SPEED_10;
		break;
	}

	if (duplex)
		val |= GMAC_FULL_DUPLEX;

	reset_control_assert(ld->macif_rst);
	writel_relaxed(val, ld->macif_base);
	reset_control_deassert(ld->macif_rst);

	writel_relaxed(BIT_MODE_CHANGE_EN, ld->gmac_iobase + MODE_CHANGE_EN);
	if (speed == SPEED_1000)
		val = GMAC_SPEED_1000;
	else if (speed == SPEED_100)
		val = GMAC_SPEED_100;
	else
		val = GMAC_SPEED_10;
	writel_relaxed(val, ld->gmac_iobase + PORT_MODE);
	writel_relaxed(0, ld->gmac_iobase + MODE_CHANGE_EN);
	writel_relaxed(duplex, ld->gmac_iobase + MAC_DUPLEX_HALF_CTRL);
}

static void gmac_set_flow_ctrl_params(struct gmac_netdev_local *ld)
{
	u32 rx_fq_empty_th, rx_fq_full_th;
	u32 rx_bq_empty_th, rx_bq_full_th;
	u32 rec_filter;

	writel(ld->pause, ld->gmac_iobase + FC_TX_TIMER);
	writel(ld->pause_interval, ld->gmac_iobase + PAUSE_THR);

	rx_fq_empty_th = readl(ld->gmac_iobase + RX_FQ_ALEMPTY_TH);
	rx_fq_empty_th &= ~(BITS_Q_PAUSE_TH_MASK << BITS_Q_PAUSE_TH_OFFSET);
	rx_fq_empty_th |= ld->flow_ctrl_active_threshold << BITS_Q_PAUSE_TH_OFFSET;
	writel(rx_fq_empty_th, ld->gmac_iobase + RX_FQ_ALEMPTY_TH);

	rx_fq_full_th = readl(ld->gmac_iobase + RX_FQ_ALFULL_TH);
	rx_fq_full_th &= ~(BITS_Q_PAUSE_TH_MASK << BITS_Q_PAUSE_TH_OFFSET);
	rx_fq_full_th |= ld->flow_ctrl_deactive_threshold << BITS_Q_PAUSE_TH_OFFSET;
	writel(rx_fq_full_th, ld->gmac_iobase + RX_FQ_ALFULL_TH);

	rx_bq_empty_th = readl(ld->gmac_iobase + RX_BQ_ALEMPTY_TH);
	rx_bq_empty_th &= ~(BITS_Q_PAUSE_TH_MASK << BITS_Q_PAUSE_TH_OFFSET);
	rx_bq_empty_th |= ld->flow_ctrl_active_threshold << BITS_Q_PAUSE_TH_OFFSET;
	writel(rx_bq_empty_th, ld->gmac_iobase + RX_BQ_ALEMPTY_TH);

	rx_bq_full_th = readl(ld->gmac_iobase + RX_BQ_ALFULL_TH);
	rx_bq_full_th &= ~(BITS_Q_PAUSE_TH_MASK << BITS_Q_PAUSE_TH_OFFSET);
	rx_bq_full_th |= ld->flow_ctrl_deactive_threshold << BITS_Q_PAUSE_TH_OFFSET;
	writel(rx_bq_full_th, ld->gmac_iobase + RX_BQ_ALFULL_TH);

	writel(0, ld->gmac_iobase + CRF_TX_PAUSE);

	rec_filter = readl(ld->gmac_iobase + REC_FILT_CONTROL);
	rec_filter |= BIT_PAUSE_FRM_PASS;
	writel(rec_filter, ld->gmac_iobase + REC_FILT_CONTROL);
}

static void gmac_set_flow_ctrl_state(struct gmac_netdev_local *ld, int pause)
{
	u32 flow_rx_q_en, flow;

	flow_rx_q_en = readl(ld->gmac_iobase + RX_PAUSE_EN);
	flow_rx_q_en &= ~(BIT_RX_FQ_PAUSE_EN | BIT_RX_BQ_PAUSE_EN);
	if (pause && (ld->flow_ctrl & FLOW_TX))
		flow_rx_q_en |= BIT_RX_FQ_PAUSE_EN | BIT_RX_BQ_PAUSE_EN;
	writel(flow_rx_q_en, ld->gmac_iobase + RX_PAUSE_EN);

	flow = readl(ld->gmac_iobase + PAUSE_EN);
	flow &= ~(BIT_RX_FDFC | BIT_TX_FDFC);
	if (pause) {
		if (ld->flow_ctrl & FLOW_RX)
			flow |= BIT_RX_FDFC;
		if (ld->flow_ctrl & FLOW_TX)
			flow |= BIT_TX_FDFC;
	}
	writel(flow, ld->gmac_iobase + PAUSE_EN);
}

static void gmac_adjust_link(struct net_device *dev)
{
	struct gmac_netdev_local *ld = netdev_priv(dev);
	struct phy_device *phy = ld->phy;
	bool link_status_changed = false;

	if (phy->link) {
		if (ld->old_speed != phy->speed ||
		    ld->old_duplex != phy->duplex) {
			unsigned long txflags;

			spin_lock_irqsave(&ld->txlock, txflags);

			gmac_linkup_flush(ld);
			gmac_config_port(dev, phy->speed, phy->duplex);
			gmac_mac_tx_state_engine_reset(ld);

			spin_unlock_irqrestore(&ld->txlock, txflags);

			gmac_set_flow_ctrl_state(ld, phy->pause);

			link_status_changed = true;
			ld->old_link = 1;
			ld->old_speed = phy->speed;
			ld->old_duplex = phy->duplex;
		}
	} else if (ld->old_link) {
		link_status_changed = true;
		ld->old_link = 0;
		netif_carrier_off(dev);
		ld->old_speed = SPEED_UNKNOWN;
		ld->old_duplex = DUPLEX_UNKNOWN;
	}

	if (link_status_changed && netif_msg_link(ld))
		phy_print_status(phy);
}

static int gmac_init_sg_desc_queue(struct gmac_netdev_local *ld)
{
	ld->sg_count = ld->TX_BQ.count + GMAC_SG_DESC_ADD;
	ld->dma_sg_desc = dma_alloc_coherent(ld->dev,
					     ld->sg_count * sizeof(struct sg_desc),
					     &ld->dma_sg_phy, GFP_KERNEL);
	if (!ld->dma_sg_desc) {
		netdev_err(ld->netdev, "alloc sg desc dma error!\n");
		return -ENOMEM;
	}

	ld->sg_head = 0;
	ld->sg_tail = 0;

	return 0;
}

static void gmac_destroy_sg_desc_queue(struct gmac_netdev_local *ld)
{
	if (ld->dma_sg_desc) {
		dma_free_coherent(ld->dev,
				  ld->sg_count * sizeof(struct sg_desc),
				  ld->dma_sg_desc, ld->dma_sg_phy);
		ld->dma_sg_desc = NULL;
	}
}

static bool gmac_rx_fq_empty(struct gmac_netdev_local *ld)
{
	u32 start = readl(ld->gmac_iobase + RX_FQ_WR_ADDR);
	u32 end = readl(ld->gmac_iobase + RX_FQ_RD_ADDR);

	return start == end;
}

static bool gmac_rxq_has_packets(struct gmac_netdev_local *ld, int rxq_id)
{
	u32 start = readl(ld->gmac_iobase + rx_bq_rd_addr_queue(rxq_id));
	u32 end = readl(ld->gmac_iobase + rx_bq_wr_addr_queue(rxq_id));

	return start != end;
}

static u32 gmac_rx_refill(struct gmac_netdev_local *ld);

static void gmac_monitor_func(struct timer_list *t)
{
	struct gmac_netdev_local *ld = timer_container_of(ld, t, monitor);
	u32 refill_cnt;

	if (!netif_running(ld->netdev))
		return;

	spin_lock(&ld->rxlock);
	refill_cnt = gmac_rx_refill(ld);
	if (!refill_cnt && gmac_rx_fq_empty(ld)) {
		int rxq_id;

		for (rxq_id = 0; rxq_id < ld->num_rxqs; rxq_id++) {
			if (gmac_rxq_has_packets(ld, rxq_id))
				napi_schedule(&ld->q_napi[rxq_id].napi);
		}
	}
	spin_unlock(&ld->rxlock);

	mod_timer(&ld->monitor, jiffies + GMAC_MONITOR_TIMER);
}

static u32 gmac_rx_refill(struct gmac_netdev_local *ld)
{
	struct gmac_desc *desc;
	struct sk_buff *skb;
	struct cyclic_queue_info dma_info;
	u32 len = ETH_MAX_FRAME_SIZE;
	dma_addr_t addr;
	u32 refill_cnt = 0;
	u32 i;

	/* software write pointer */
	dma_info.start = dma_cnt(readl(ld->gmac_iobase + RX_FQ_WR_ADDR));
	/* logic read pointer */
	dma_info.end = dma_cnt(readl(ld->gmac_iobase + RX_FQ_RD_ADDR));
	dma_info.num = CIRC_SPACE(dma_info.start, dma_info.end, RX_DESC_NUM);

	for (i = 0, dma_info.pos = dma_info.start; i < dma_info.num; i++) {
		if (ld->RX_FQ.skb[dma_info.pos] || ld->rx_skb[dma_info.pos])
			break;

		skb = netdev_alloc_skb_ip_align(ld->netdev, len);
		if (unlikely(!skb))
			break;

		addr = dma_map_single(ld->dev, skb->data, len, DMA_FROM_DEVICE);
		if (dma_mapping_error(ld->dev, addr)) {
			dev_kfree_skb_any(skb);
			break;
		}

		desc = ld->RX_FQ.desc + dma_info.pos;
		desc->data_buff_addr = (u32)addr;
		desc->reserve31 = addr >> REG_BIT_WIDTH;
		ld->RX_FQ.skb[dma_info.pos] = skb;
		ld->rx_skb[dma_info.pos] = skb;

		desc->buffer_len = len - 1;
		desc->data_len = 0;
		desc->fl = 0;
		desc->descvid = DESC_VLD_FREE;
		desc->skb_id = dma_info.pos;

		refill_cnt++;
		dma_info.pos = dma_ring_incr(dma_info.pos, RX_DESC_NUM);
	}

	/*
	 * This barrier is important here.  It is required to ensure
	 * the ARM CPU flushes its DMA write buffers before proceeding
	 * to the next instruction, to ensure that GMAC will see
	 * our descriptor changes in memory.
	 */
	gmac_sync_barrier();

	if (dma_info.pos != dma_info.start)
		writel(dma_byte(dma_info.pos), ld->gmac_iobase + RX_FQ_WR_ADDR);

	return refill_cnt;
}

static int gmac_rx_checksum(struct net_device *dev, struct sk_buff *skb,
			    const struct gmac_desc *desc)
{
	int hdr_csum_done, payload_csum_done;
	int hdr_csum_err, payload_csum_err;

	if (dev->features & NETIF_F_RXCSUM) {
		hdr_csum_done = desc->header_csum_done;
		payload_csum_done = desc->payload_csum_done;
		hdr_csum_err = desc->header_csum_err;
		payload_csum_err = desc->payload_csum_err;

		if (hdr_csum_done && payload_csum_done) {
			if (unlikely(hdr_csum_err || payload_csum_err)) {
				dev->stats.rx_errors++;
				dev->stats.rx_crc_errors++;
				dev_kfree_skb_any(skb);
				return -1;
			}
			skb->ip_summed = CHECKSUM_UNNECESSARY;
		}
	}
	return 0;
}

static void gmac_rx_skbput(struct net_device *dev, struct sk_buff *skb,
			   const struct gmac_desc *desc, int rxq_id)
{
	struct gmac_netdev_local *ld = netdev_priv(dev);
	dma_addr_t addr;
	u32 len;

	len = desc->data_len;

	addr = desc->data_buff_addr;
	addr |= (dma_addr_t)desc->reserve31 << REG_BIT_WIDTH;
	dma_unmap_single(ld->dev, addr, ETH_MAX_FRAME_SIZE, DMA_FROM_DEVICE);

	if ((addr & NET_IP_ALIGN) == 0)
		skb_reserve(skb, 2); /* 2:NET_IP_ALIGN */

	skb_put(skb, len);
	if (skb->len > ETH_MAX_FRAME_SIZE) {
		netdev_err(dev, "rcv len err, len = %d\n", skb->len);
		dev->stats.rx_errors++;
		dev->stats.rx_length_errors++;
		dev_kfree_skb_any(skb);
		return;
	}

	skb->protocol = eth_type_trans(skb, dev);
	skb->ip_summed = CHECKSUM_NONE;

	if (gmac_rx_checksum(dev, skb, desc))
		return;

	if ((dev->features & NETIF_F_RXHASH) && desc->has_hash)
		skb_set_hash(skb, desc->rxhash, desc->l3_hash ?
			     PKT_HASH_TYPE_L3 : PKT_HASH_TYPE_L4);

	skb_record_rx_queue(skb, rxq_id);

	napi_gro_receive(&ld->q_napi[rxq_id].napi, skb);
	dev->stats.rx_packets++;
	dev->stats.rx_bytes += len;
}

static int gmac_rx_skb(struct net_device *dev, struct gmac_desc *desc,
		       u16 skb_id, int rxq_id)
{
	struct gmac_netdev_local *ld = netdev_priv(dev);
	struct sk_buff *skb;

	spin_lock(&ld->rxlock);
	skb = ld->rx_skb[skb_id];
	if (unlikely(!skb)) {
		spin_unlock(&ld->rxlock);
		netdev_err(dev, "inconsistent rx_skb\n");
		return -1;
	}

	/* data consistent check */
	if (unlikely(skb != ld->RX_FQ.skb[skb_id])) {
		netdev_err(dev, "desc->skb(%p),RX_FQ.skb[%d](%p)\n",
			   skb, skb_id, ld->RX_FQ.skb[skb_id]);
		if (ld->RX_FQ.skb[skb_id] == SKB_MAGIC) {
			spin_unlock(&ld->rxlock);
			return 0;
		}
		WARN_ON(1);
	} else {
		ld->RX_FQ.skb[skb_id] = NULL;
	}
	spin_unlock(&ld->rxlock);

	gmac_rx_skbput(dev, skb, desc, rxq_id);
	return 0;
}

static int gmac_rx(struct net_device *dev, int limit, int rxq_id)
{
	struct gmac_netdev_local *ld = netdev_priv(dev);
	struct gmac_desc *desc;
	struct cyclic_queue_info dma_info;
	u32 rx_bq_rd_reg, rx_bq_wr_reg;
	u16 skb_id;
	u32 i;

	rx_bq_rd_reg = rx_bq_rd_addr_queue(rxq_id);
	rx_bq_wr_reg = rx_bq_wr_addr_queue(rxq_id);

	/* software read pointer */
	dma_info.start = dma_cnt(readl(ld->gmac_iobase + rx_bq_rd_reg));
	/* logic write pointer */
	dma_info.end = dma_cnt(readl(ld->gmac_iobase + rx_bq_wr_reg));
	dma_info.num = CIRC_CNT(dma_info.end, dma_info.start, RX_DESC_NUM);
	if (dma_info.num > limit)
		dma_info.num = limit;

	/* ensure get updated desc */
	rmb();
	for (i = 0, dma_info.pos = dma_info.start; i < dma_info.num; i++) {
		if (rxq_id)
			desc = ld->pool[BASE_QUEUE_NUMS + rxq_id].desc + dma_info.pos;
		else
			desc = ld->RX_BQ.desc + dma_info.pos;
		skb_id = desc->skb_id;

		if (unlikely(gmac_rx_skb(dev, desc, skb_id, rxq_id)))
			break;

		spin_lock(&ld->rxlock);
		ld->rx_skb[skb_id] = NULL;
		spin_unlock(&ld->rxlock);
		dma_info.pos = dma_ring_incr(dma_info.pos, RX_DESC_NUM);
	}

	if (dma_info.pos != dma_info.start)
		writel(dma_byte(dma_info.pos), ld->gmac_iobase + rx_bq_rd_reg);

	spin_lock(&ld->rxlock);
	gmac_rx_refill(ld);
	spin_unlock(&ld->rxlock);

	return dma_info.num;
}

static int gmac_check_tx_err(struct gmac_netdev_local *ld,
			     const struct gmac_tso_desc *tx_bq_desc,
			     unsigned int desc_pos)
{
	unsigned int tx_err = tx_bq_desc->tx_err;

	if (unlikely(tx_err & ERR_ALL)) {
		const struct sg_desc *desc_cur =
			ld->dma_sg_desc + ld->TX_BQ.sg_desc_offset[desc_pos];
		const u32 *sg_word = (const u32 *)desc_cur;
		int i;

		WARN(tx_err & ERR_ALL,
		     "TX ERR: desc1=0x%x, desc2=0x%x, desc5=0x%x\n",
		     tx_bq_desc->data_buff_addr,
		     tx_bq_desc->desc1.val, tx_bq_desc->tx_err);

		for (i = 0; i < sizeof(struct sg_desc) / sizeof(u32); i++)
			pr_err("%s: sg_desc word[%d]=0x%x\n",
			       __func__, i, sg_word[i]);

		return -1;
	}

	return 0;
}

static void gmac_xmit_release_gso_sg(struct gmac_netdev_local *ld,
				     const struct gmac_tso_desc *tx_rq_desc,
				     unsigned int desc_pos)
{
	struct sg_desc *desc_cur;
	int nfrags = tx_rq_desc->desc1.tx.nfrags_num;
	unsigned int desc_offset;
	dma_addr_t addr;
	size_t len;
	int i;

	desc_offset = ld->TX_BQ.sg_desc_offset[desc_pos];
	WARN_ON(desc_offset != ld->sg_tail);
	desc_cur = ld->dma_sg_desc + desc_offset;

	addr = desc_cur->linear_addr;
	addr |= (dma_addr_t)(desc_cur->reserv3 >> SG_DESC_HI8_OFFSET) << REG_BIT_WIDTH;
	len = desc_cur->linear_len;
	dma_unmap_single(ld->dev, addr, len, DMA_TO_DEVICE);
	for (i = 0; i < nfrags; i++) {
		addr = desc_cur->frags[i].addr;
		addr |= (dma_addr_t)(desc_cur->frags[i].reserved >> SG_DESC_HI8_OFFSET)
			<< REG_BIT_WIDTH;
		len = desc_cur->frags[i].size;
		dma_unmap_page(ld->dev, addr, len, DMA_TO_DEVICE);
	}
}

static int gmac_xmit_release_gso(struct gmac_netdev_local *ld,
				 struct gmac_tso_desc *tx_rq_desc,
				 unsigned int desc_pos)
{
	unsigned int nfrags = tx_rq_desc->desc1.tx.nfrags_num;
	dma_addr_t addr;
	size_t len;

	if (unlikely(gmac_check_tx_err(ld, tx_rq_desc, desc_pos) < 0)) {
		/* dev_close */
		gmac_irq_disable_all_queue(ld);
		gmac_hw_desc_disable(ld);

		netif_carrier_off(ld->netdev);
		netif_stop_queue(ld->netdev);

		phy_stop(ld->phy);
		timer_delete_sync(&ld->monitor);
		return -1;
	}

	if (tx_rq_desc->desc1.tx.tso_flag || nfrags) {
		gmac_xmit_release_gso_sg(ld, tx_rq_desc, desc_pos);
		ld->sg_tail = (ld->sg_tail + 1) % ld->sg_count;
	} else {
		addr = tx_rq_desc->data_buff_addr;
		addr |= (dma_addr_t)(tx_rq_desc->reserve_desc2 & TX_DESC_HI8_MASK)
			<< REG_BIT_WIDTH;
		len = tx_rq_desc->desc1.tx.data_len;
		dma_unmap_single(ld->dev, addr, len, DMA_TO_DEVICE);
	}

	return 0;
}

static int gmac_xmit_reclaim_release(struct net_device *dev,
				     struct sk_buff *skb,
				     struct gmac_desc *desc, u32 pos)
{
	struct gmac_netdev_local *ld = netdev_priv(dev);
	struct gmac_tso_desc *tso_desc;
	dma_addr_t addr;

	if (ld->tso_supported) {
		tso_desc = (struct gmac_tso_desc *)desc;
		return gmac_xmit_release_gso(ld, tso_desc, pos);
	}

	addr = desc->data_buff_addr;
	addr |= (dma_addr_t)(desc->rxhash & TX_DESC_HI8_MASK) << REG_BIT_WIDTH;
	dma_unmap_single(ld->dev, addr, skb->len, DMA_TO_DEVICE);

	return 0;
}

static void gmac_xmit_reclaim(struct net_device *dev)
{
	struct gmac_netdev_local *ld = netdev_priv(dev);
	struct sk_buff *skb;
	struct gmac_desc *desc;
	unsigned int bytes_compl = 0;
	unsigned int pkts_compl = 0;
	struct cyclic_queue_info dma_info;
	u32 i;

	spin_lock(&ld->txlock);

	/* software read */
	dma_info.start = dma_cnt(readl(ld->gmac_iobase + TX_RQ_RD_ADDR));
	/* logic write */
	dma_info.end = dma_cnt(readl(ld->gmac_iobase + TX_RQ_WR_ADDR));
	dma_info.num = CIRC_CNT(dma_info.end, dma_info.start, TX_DESC_NUM);

	for (i = 0, dma_info.pos = dma_info.start; i < dma_info.num; i++) {
		skb = ld->tx_skb[dma_info.pos];
		if (unlikely(!skb)) {
			netdev_err(dev, "inconsistent tx_skb\n");
			break;
		}

		if (skb != ld->TX_BQ.skb[dma_info.pos]) {
			netdev_err(dev, "tx skb[%d](%p) != skb(%p)\n",
				   dma_info.pos, ld->TX_BQ.skb[dma_info.pos], skb);
			if (ld->TX_BQ.skb[dma_info.pos] == SKB_MAGIC)
				goto next;
		}

		pkts_compl++;
		bytes_compl += skb->len;
		desc = ld->TX_RQ.desc + dma_info.pos;
		if (gmac_xmit_reclaim_release(dev, skb, desc, dma_info.pos) < 0)
			break;

		ld->TX_BQ.skb[dma_info.pos] = NULL;
next:
		ld->tx_skb[dma_info.pos] = NULL;
		dev_consume_skb_any(skb);
		dma_info.pos = dma_ring_incr(dma_info.pos, TX_DESC_NUM);
	}

	if (dma_info.pos != dma_info.start)
		writel(dma_byte(dma_info.pos), ld->gmac_iobase + TX_RQ_RD_ADDR);

	if (pkts_compl || bytes_compl)
		netdev_completed_queue(dev, pkts_compl, bytes_compl);

	if (unlikely(netif_queue_stopped(ld->netdev)) && pkts_compl)
		netif_wake_queue(ld->netdev);

	spin_unlock(&ld->txlock);
}

static int gmac_poll(struct napi_struct *napi, int budget)
{
	struct gmac_napi *q_napi = container_of(napi, struct gmac_napi, napi);
	struct gmac_netdev_local *ld = q_napi->ndev_priv;
	struct net_device *dev = ld->netdev;
	int work_done = 0;
	int task = budget;
	u32 ints, num;
	u32 raw_int_reg, raw_int_mask;

	if (q_napi->rxq_id) {
		raw_int_reg = RSS_RAW_PMU_INT;
		raw_int_mask = def_int_mask_queue(q_napi->rxq_id);
	} else {
		raw_int_reg = RAW_PMU_INT;
		raw_int_mask = DEF_INT_MASK;
	}

	do {
		if (!q_napi->rxq_id)
			gmac_xmit_reclaim(dev);
		num = gmac_rx(dev, task, q_napi->rxq_id);
		work_done += num;
		task -= num;
		if (work_done >= budget)
			break;

		ints = readl(ld->gmac_iobase + raw_int_reg);
		ints &= raw_int_mask;
		writel(ints, ld->gmac_iobase + raw_int_reg);
	} while (ints || gmac_rxq_has_packets(ld, q_napi->rxq_id));

	if (work_done < budget) {
		napi_complete(napi);
		gmac_irq_enable_queue(ld, q_napi->rxq_id);
	}

	return work_done;
}

static irqreturn_t gmac_interrupt(int irq, void *dev_id)
{
	struct gmac_napi *q_napi = dev_id;
	struct gmac_netdev_local *ld = q_napi->ndev_priv;
	u32 ints;
	u32 raw_int_reg, raw_int_mask;

	if (gmac_queue_irq_disabled(ld, q_napi->rxq_id))
		return IRQ_NONE;

	if (q_napi->rxq_id) {
		raw_int_reg = RSS_RAW_PMU_INT;
		raw_int_mask = def_int_mask_queue(q_napi->rxq_id);
	} else {
		raw_int_reg = RAW_PMU_INT;
		raw_int_mask = DEF_INT_MASK;
	}

	ints = readl(ld->gmac_iobase + raw_int_reg);
	ints &= raw_int_mask;
	writel(ints, ld->gmac_iobase + raw_int_reg);

	if (likely(ints || gmac_rxq_has_packets(ld, q_napi->rxq_id))) {
		gmac_irq_disable_queue(ld, q_napi->rxq_id);
		napi_schedule(&q_napi->napi);
	}

	return IRQ_HANDLED;
}

static void gmac_enable_napi(struct gmac_netdev_local *ld)
{
	int i;

	for (i = 0; i < ld->num_rxqs; i++)
		napi_enable(&ld->q_napi[i].napi);
}

static void gmac_disable_napi(struct gmac_netdev_local *ld)
{
	int i;

	for (i = 0; i < ld->num_rxqs; i++)
		napi_disable(&ld->q_napi[i].napi);
}

static void gmac_enable_rxcsum_drop(struct gmac_netdev_local *ld, bool drop)
{
	u32 v = readl(ld->gmac_iobase + TSO_COE_CTRL);

	if (drop)
		v |= COE_ERR_DROP;
	else
		v &= ~COE_ERR_DROP;
	writel(v, ld->gmac_iobase + TSO_COE_CTRL);
}

static void gmac_get_rss_key(struct gmac_netdev_local *ld)
{
	struct gmac_rss_info *rss = &ld->rss_info;

	*((u32 *)rss->key) = readl(ld->gmac_iobase + RSS_HASH_KEY);
}

static void gmac_set_rss_key(struct gmac_netdev_local *ld)
{
	struct gmac_rss_info *rss = &ld->rss_info;

	writel(*((u32 *)rss->key), ld->gmac_iobase + RSS_HASH_KEY);
}

static int gmac_wait_rss_ready(struct gmac_netdev_local *ld)
{
	void __iomem *base = ld->gmac_iobase;
	int i;
	const int timeout = 10000;

	for (i = 0; !(readl(base + RSS_IND_TBL) & BIT_IND_TBL_READY); i++) {
		if (i == timeout) {
			netdev_err(ld->netdev, "wait rss ready timeout!\n");
			return -ETIMEDOUT;
		}
		usleep_range(10, 20);
	}

	return 0;
}

static void gmac_config_rss(struct gmac_netdev_local *ld)
{
	struct gmac_rss_info *rss = &ld->rss_info;
	unsigned int i;

	for (i = 0; i < rss->ind_tbl_size; i++) {
		if (gmac_wait_rss_ready(ld))
			break;
		writel(BIT_IND_TLB_WR | (rss->ind_tbl[i] << 8) | i,
		       ld->gmac_iobase + RSS_IND_TBL);
	}
}

static void gmac_get_rss(struct gmac_netdev_local *ld)
{
	struct gmac_rss_info *rss = &ld->rss_info;
	u32 rss_val;
	int i;

	for (i = 0; i < rss->ind_tbl_size; i++) {
		if (gmac_wait_rss_ready(ld))
			break;
		writel(i, ld->gmac_iobase + RSS_IND_TBL);
		if (gmac_wait_rss_ready(ld))
			break;
		rss_val = readl(ld->gmac_iobase + RSS_IND_TBL);
		rss->ind_tbl[i] = (rss_val >> 10) & 0x3;
	}
}

static void gmac_config_hash_policy(struct gmac_netdev_local *ld)
{
	writel(ld->rss_info.hash_cfg, ld->gmac_iobase + RSS_HASH_CONFIG);
}

/* ethtool operations */

static void gmac_get_drvinfo(struct net_device *net_dev,
			     struct ethtool_drvinfo *info)
{
	strscpy(info->driver, GMAC_DRIVER_NAME, sizeof(info->driver));
	strscpy(info->bus_info, "platform", sizeof(info->bus_info));
}

static u32 gmac_get_link(struct net_device *net_dev)
{
	struct gmac_netdev_local *ld = netdev_priv(net_dev);

	return ld->phy->link ? 1 : 0;
}

static void gmac_get_pauseparam(struct net_device *net_dev,
				struct ethtool_pauseparam *pause)
{
	struct gmac_netdev_local *ld = netdev_priv(net_dev);

	pause->rx_pause = 0;
	pause->tx_pause = 0;
	pause->autoneg = ld->phy->autoneg;

	if (ld->phy->pause && (ld->flow_ctrl & FLOW_RX))
		pause->rx_pause = 1;
	if (ld->phy->pause && (ld->flow_ctrl & FLOW_TX))
		pause->tx_pause = 1;
}

static int gmac_set_pauseparam(struct net_device *net_dev,
			       struct ethtool_pauseparam *pause)
{
	struct gmac_netdev_local *ld = netdev_priv(net_dev);
	struct phy_device *phy = ld->phy;
	unsigned int new_pause = FLOW_OFF;

	if (pause->rx_pause)
		new_pause |= FLOW_RX;
	if (pause->tx_pause)
		new_pause |= FLOW_TX;

	if (new_pause != ld->flow_ctrl)
		ld->flow_ctrl = new_pause;

	gmac_set_flow_ctrl_state(ld, phy->pause);
	linkmode_clear_bit(ETHTOOL_LINK_MODE_Pause_BIT, phy->advertising);
	if (ld->flow_ctrl)
		linkmode_set_bit(ETHTOOL_LINK_MODE_Pause_BIT, phy->advertising);

	if (phy->autoneg) {
		if (netif_running(net_dev))
			return phy_start_aneg(phy);
	}

	return 0;
}

static u32 gmac_ethtool_getmsglevel(struct net_device *ndev)
{
	struct gmac_netdev_local *ld = netdev_priv(ndev);

	return ld->msg_enable;
}

static void gmac_ethtool_setmsglevel(struct net_device *ndev, u32 level)
{
	struct gmac_netdev_local *ld = netdev_priv(ndev);

	ld->msg_enable = level;
}

static u32 gmac_get_rxfh_key_size(struct net_device *ndev)
{
	return RSS_HASH_KEY_SIZE;
}

static u32 gmac_get_rxfh_indir_size(struct net_device *ndev)
{
	struct gmac_netdev_local *ld = netdev_priv(ndev);

	return ld->rss_info.ind_tbl_size;
}

static int gmac_get_rxfh(struct net_device *ndev,
			 struct ethtool_rxfh_param *rxfh)
{
	struct gmac_netdev_local *ld = netdev_priv(ndev);
	struct gmac_rss_info *rss = &ld->rss_info;
	int i;

	rxfh->hfunc = ETH_RSS_HASH_TOP;

	if (rxfh->key)
		memcpy(rxfh->key, rss->key, RSS_HASH_KEY_SIZE);

	if (rxfh->indir)
		for (i = 0; i < rss->ind_tbl_size; i++)
			rxfh->indir[i] = rss->ind_tbl[i];

	return 0;
}

static int gmac_set_rxfh(struct net_device *ndev,
			 struct ethtool_rxfh_param *rxfh,
			 struct netlink_ext_ack *extack)
{
	struct gmac_netdev_local *ld = netdev_priv(ndev);
	struct gmac_rss_info *rss = &ld->rss_info;
	int i;

	if (rxfh->hfunc != ETH_RSS_HASH_NO_CHANGE &&
	    rxfh->hfunc != ETH_RSS_HASH_TOP)
		return -EOPNOTSUPP;

	if (rxfh->indir)
		for (i = 0; i < rss->ind_tbl_size; i++)
			rss->ind_tbl[i] = rxfh->indir[i];

	if (rxfh->key) {
		memcpy(rss->key, rxfh->key, RSS_HASH_KEY_SIZE);
		gmac_set_rss_key(ld);
	}

	gmac_config_rss(ld);

	return 0;
}

static void gmac_get_rss_hash(struct ethtool_rxnfc *info, u32 hash_cfg,
			      u32 l3_hash_en, u32 l4_hash_en, u32 vlan_hash_en)
{
	if (hash_cfg & l3_hash_en)
		info->data |= RXH_IP_SRC | RXH_IP_DST;
	if (hash_cfg & l4_hash_en)
		info->data |= RXH_L4_B_0_1 | RXH_L4_B_2_3;
	if (hash_cfg & vlan_hash_en)
		info->data |= RXH_VLAN;
}

static int gmac_get_rss_hash_opts(struct gmac_netdev_local *ld,
				  struct ethtool_rxnfc *info)
{
	u32 hash_cfg = ld->rss_info.hash_cfg;

	info->data = 0;

	switch (info->flow_type) {
	case TCP_V4_FLOW:
		gmac_get_rss_hash(info, hash_cfg, TCPV4_L3_HASH_EN,
				  TCPV4_L4_HASH_EN, TCPV4_VLAN_HASH_EN);
		break;
	case TCP_V6_FLOW:
		gmac_get_rss_hash(info, hash_cfg, TCPV6_L3_HASH_EN,
				  TCPV6_L4_HASH_EN, TCPV6_VLAN_HASH_EN);
		break;
	case UDP_V4_FLOW:
		gmac_get_rss_hash(info, hash_cfg, UDPV4_L3_HASH_EN,
				  UDPV4_L4_HASH_EN, UDPV4_VLAN_HASH_EN);
		break;
	case UDP_V6_FLOW:
		gmac_get_rss_hash(info, hash_cfg, UDPV6_L3_HASH_EN,
				  UDPV6_L4_HASH_EN, UDPV6_VLAN_HASH_EN);
		break;
	case IPV4_FLOW:
		gmac_get_rss_hash(info, hash_cfg, IPV4_L3_HASH_EN, 0,
				  IPV4_VLAN_HASH_EN);
		break;
	case IPV6_FLOW:
		gmac_get_rss_hash(info, hash_cfg, IPV6_L3_HASH_EN, 0,
				  IPV6_VLAN_HASH_EN);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int gmac_get_rxnfc(struct net_device *ndev,
			  struct ethtool_rxnfc *info, u32 *rules)
{
	struct gmac_netdev_local *ld = netdev_priv(ndev);

	switch (info->cmd) {
	case ETHTOOL_GRXRINGS:
		info->data = ld->num_rxqs;
		return 0;
	case ETHTOOL_GRXFH:
		return gmac_get_rss_hash_opts(ld, info);
	default:
		break;
	}
	return -EOPNOTSUPP;
}

static int gmac_set_tcp_udp_hash_cfg(const struct ethtool_rxnfc *info,
				     u32 *hash_cfg, u32 l4_mask, u32 vlan_mask)
{
	switch (info->data & (RXH_L4_B_0_1 | RXH_L4_B_2_3)) {
	case 0: /* all bits are 0 */
		*hash_cfg &= ~l4_mask;
		break;
	case (RXH_L4_B_0_1 | RXH_L4_B_2_3):
		*hash_cfg |= l4_mask;
		break;
	default:
		return -EINVAL;
	}
	if (info->data & RXH_VLAN)
		*hash_cfg |= vlan_mask;
	else
		*hash_cfg &= ~vlan_mask;
	return 0;
}

static int gmac_ip_hash_cfg(const struct ethtool_rxnfc *info, u32 *hash_cfg,
			    u32 vlan_mask)
{
	if (info->data & (RXH_L4_B_0_1 | RXH_L4_B_2_3))
		return -EINVAL;
	if (info->data & RXH_VLAN)
		*hash_cfg |= vlan_mask;
	else
		*hash_cfg &= ~vlan_mask;
	return 0;
}

static int gmac_set_rss_hash_opts(struct gmac_netdev_local *ld,
				  const struct ethtool_rxnfc *info)
{
	u32 hash_cfg = ld->rss_info.hash_cfg;
	int ret;

	if (!(info->data & RXH_IP_SRC) || !(info->data & RXH_IP_DST))
		return -EINVAL;

	switch (info->flow_type) {
	case TCP_V4_FLOW:
		ret = gmac_set_tcp_udp_hash_cfg(info, &hash_cfg,
						TCPV4_L4_HASH_EN,
						TCPV4_VLAN_HASH_EN);
		break;
	case TCP_V6_FLOW:
		ret = gmac_set_tcp_udp_hash_cfg(info, &hash_cfg,
						TCPV6_L4_HASH_EN,
						TCPV6_VLAN_HASH_EN);
		break;
	case UDP_V4_FLOW:
		ret = gmac_set_tcp_udp_hash_cfg(info, &hash_cfg,
						UDPV4_L4_HASH_EN,
						UDPV4_VLAN_HASH_EN);
		break;
	case UDP_V6_FLOW:
		ret = gmac_set_tcp_udp_hash_cfg(info, &hash_cfg,
						UDPV6_L4_HASH_EN,
						UDPV6_VLAN_HASH_EN);
		break;
	case IPV4_FLOW:
		ret = gmac_ip_hash_cfg(info, &hash_cfg, IPV4_VLAN_HASH_EN);
		break;
	case IPV6_FLOW:
		ret = gmac_ip_hash_cfg(info, &hash_cfg, IPV6_VLAN_HASH_EN);
		break;
	default:
		return -EINVAL;
	}
	if (ret)
		return ret;

	ld->rss_info.hash_cfg = hash_cfg;
	gmac_config_hash_policy(ld);

	return 0;
}

static int gmac_set_rxnfc(struct net_device *ndev, struct ethtool_rxnfc *info)
{
	struct gmac_netdev_local *ld = netdev_priv(ndev);

	switch (info->cmd) {
	case ETHTOOL_SRXFH:
		return gmac_set_rss_hash_opts(ld, info);
	default:
		break;
	}
	return -EOPNOTSUPP;
}

static const struct ethtool_ops gmac_ethtool_ops = {
	.get_drvinfo = gmac_get_drvinfo,
	.get_link = gmac_get_link,
	.get_pauseparam = gmac_get_pauseparam,
	.set_pauseparam = gmac_set_pauseparam,
	.get_msglevel = gmac_ethtool_getmsglevel,
	.set_msglevel = gmac_ethtool_setmsglevel,
	.get_rxfh_key_size = gmac_get_rxfh_key_size,
	.get_rxfh_indir_size = gmac_get_rxfh_indir_size,
	.get_rxfh = gmac_get_rxfh,
	.set_rxfh = gmac_set_rxfh,
	.get_rxnfc = gmac_get_rxnfc,
	.set_rxnfc = gmac_set_rxnfc,
	.get_link_ksettings = phy_ethtool_get_link_ksettings,
	.set_link_ksettings = phy_ethtool_set_link_ksettings,
};

/* netdevice operations */

static int gmac_net_open(struct net_device *dev)
{
	struct gmac_netdev_local *ld = netdev_priv(dev);
	unsigned long flags;

	clk_prepare_enable(ld->macif_clk);
	clk_prepare_enable(ld->clk);

	/*
	 * If we configure the mac address while the device is down, the
	 * mac core clock is disabled and the address has not been written
	 * to the mac core registers, so set it again here.
	 */
	gmac_hw_set_mac_addr(dev);

	/*
	 * The default carrier state should be off, and this call should
	 * come before phy_start().
	 */
	netif_carrier_off(dev);
	gmac_enable_napi(ld);
	phy_start(ld->phy);

	gmac_hw_desc_enable(ld);
	gmac_port_enable(ld);
	gmac_irq_enable_all_queue(ld);

	spin_lock_irqsave(&ld->rxlock, flags);
	gmac_rx_refill(ld);
	spin_unlock_irqrestore(&ld->rxlock, flags);

	mod_timer(&ld->monitor, jiffies + GMAC_MONITOR_TIMER);

	netif_start_queue(dev);

	return 0;
}

static int gmac_net_close(struct net_device *dev)
{
	struct gmac_netdev_local *ld = netdev_priv(dev);

	gmac_irq_disable_all_queue(ld);
	gmac_hw_desc_disable(ld);

	gmac_disable_napi(ld);

	netif_carrier_off(dev);
	netif_stop_queue(dev);

	phy_stop(ld->phy);
	timer_delete_sync(&ld->monitor);

	clk_disable_unprepare(ld->clk);
	clk_disable_unprepare(ld->macif_clk);

	return 0;
}

static int gmac_check_skb_len(struct sk_buff *skb, struct net_device *dev)
{
	if (skb->len < ETH_HLEN) {
		dev_kfree_skb_any(skb);
		dev->stats.tx_errors++;
		dev->stats.tx_dropped++;
		return -1;
	}
	return 0;
}

static int gmac_net_xmit_normal(struct sk_buff *skb, struct net_device *dev,
				struct gmac_desc *desc, u32 pos)
{
	struct gmac_netdev_local *ld = netdev_priv(dev);
	dma_addr_t addr;

	addr = dma_map_single(ld->dev, skb->data, skb->len, DMA_TO_DEVICE);
	if (unlikely(dma_mapping_error(ld->dev, addr))) {
		dev_kfree_skb_any(skb);
		dev->stats.tx_dropped++;
		ld->tx_skb[pos] = NULL;
		ld->TX_BQ.skb[pos] = NULL;
		return -1;
	}
	desc->data_buff_addr = (u32)addr;
	desc->rxhash = (addr >> REG_BIT_WIDTH) & TX_DESC_HI8_MASK;
	desc->buffer_len = ETH_MAX_FRAME_SIZE - 1;
	desc->data_len = skb->len;
	desc->fl = DESC_FL_FULL;
	desc->descvid = DESC_VLD_BUSY;

	return 0;
}

static int gmac_tx_avail(struct gmac_netdev_local *ld)
{
	unsigned int tx_bq_wr_offset, tx_bq_rd_offset;

	tx_bq_wr_offset = readl(ld->gmac_iobase + TX_BQ_WR_ADDR);
	tx_bq_rd_offset = readl(ld->gmac_iobase + TX_BQ_RD_ADDR);

	return (tx_bq_rd_offset >> DESC_BYTE_SHIFT) + TX_DESC_NUM -
	       (tx_bq_wr_offset >> DESC_BYTE_SHIFT) - 1;
}

static void gmac_do_udp_checksum(struct sk_buff *skb)
{
	int offset;
	__wsum csum;
	__sum16 udp_csum;

	offset = skb_checksum_start_offset(skb);
	WARN_ON(offset >= skb_headlen(skb));
	csum = skb_checksum(skb, offset, skb->len - offset, 0);

	offset += skb->csum_offset;
	WARN_ON(offset + sizeof(__sum16) > skb_headlen(skb));
	udp_csum = csum_fold(csum);
	if (udp_csum == 0)
		udp_csum = CSUM_MANGLED_0;

	*(__sum16 *)(skb->data + offset) = udp_csum;

	skb->ip_summed = CHECKSUM_NONE;
}

static int gmac_get_pkt_info_l3l4(struct gmac_tso_desc *tx_bq_desc,
				  struct sk_buff *skb, unsigned int *l4_proto,
				  unsigned int *max_mss, unsigned char *coe_enable)
{
	__be16 l3_proto; /* level 3 protocol */
	int max_data_len = skb->len - ETH_HLEN;

	l3_proto = skb->protocol;
	if (skb->protocol == htons(ETH_P_8021Q)) {
		l3_proto = vlan_get_protocol(skb);
		tx_bq_desc->desc1.tx.vlan_flag = 1;
		max_data_len -= VLAN_HLEN;
	}

	if (l3_proto == htons(ETH_P_IP)) {
		struct iphdr *iph = ip_hdr(skb);

		tx_bq_desc->desc1.tx.ip_ver = PKT_IPV4;
		tx_bq_desc->desc1.tx.ip_hdr_len = iph->ihl;

		if (max_data_len >= GSO_MAX_SIZE &&
		    ntohs(iph->tot_len) <= (iph->ihl << 2))
			iph->tot_len = htons(GSO_MAX_SIZE - 1);

		*max_mss -= iph->ihl * WORD_TO_BYTE;
		*l4_proto = iph->protocol;
	} else if (l3_proto == htons(ETH_P_IPV6)) {
		tx_bq_desc->desc1.tx.ip_ver = PKT_IPV6;
		tx_bq_desc->desc1.tx.ip_hdr_len = PKT_IPV6_HDR_LEN;
		*max_mss -= PKT_IPV6_HDR_LEN * WORD_TO_BYTE;
		*l4_proto = ipv6_hdr(skb)->nexthdr;
	} else {
		*coe_enable = 0;
	}

	if (*l4_proto == IPPROTO_TCP) {
		tx_bq_desc->desc1.tx.prot_type = PKT_TCP;
		if (tcp_hdr(skb)->doff < sizeof(struct tcphdr) / WORD_TO_BYTE)
			return -EFAULT;
		tx_bq_desc->desc1.tx.prot_hdr_len = tcp_hdr(skb)->doff;
		*max_mss -= tcp_hdr(skb)->doff * WORD_TO_BYTE;
	} else if (*l4_proto == IPPROTO_UDP) {
		tx_bq_desc->desc1.tx.prot_type = PKT_UDP;
		tx_bq_desc->desc1.tx.prot_hdr_len = PKT_UDP_HDR_LEN;
		if (l3_proto == htons(ETH_P_IPV6))
			*max_mss -= sizeof(struct frag_hdr);
	} else {
		*coe_enable = 0;
	}

	return 0;
}

static int gmac_get_pkt_info(struct sk_buff *skb,
			     struct gmac_tso_desc *tx_bq_desc)
{
	int nfrags;
	unsigned int l4_proto = IPPROTO_MAX;
	unsigned int max_mss = ETH_DATA_LEN;
	unsigned char coe_enable = 0;
	int ret;

	nfrags = skb_shinfo(skb)->nr_frags;
	if (likely(skb->ip_summed == CHECKSUM_PARTIAL))
		coe_enable = 1;

	tx_bq_desc->desc1.val = 0;

	if (skb_is_gso(skb)) {
		tx_bq_desc->desc1.tx.tso_flag = 1;
		tx_bq_desc->desc1.tx.sg_flag = 1;
	} else if (nfrags) {
		tx_bq_desc->desc1.tx.sg_flag = 1;
	}

	ret = gmac_get_pkt_info_l3l4(tx_bq_desc, skb, &l4_proto,
				     &max_mss, &coe_enable);
	if (ret < 0)
		return ret;

	if (skb_is_gso(skb))
		tx_bq_desc->desc1.tx.data_len =
			(skb_shinfo(skb)->gso_size > max_mss) ? max_mss :
				skb_shinfo(skb)->gso_size;
	else
		tx_bq_desc->desc1.tx.data_len = skb->len;

	if (coe_enable && skb_is_gso(skb) && l4_proto == IPPROTO_UDP)
		gmac_do_udp_checksum(skb);

	if (coe_enable)
		tx_bq_desc->desc1.tx.coe_flag = 1;

	tx_bq_desc->desc1.tx.nfrags_num = nfrags;

	tx_bq_desc->desc1.tx.hw_own = DESC_VLD_BUSY;
	return 0;
}

static int gmac_check_hw_capability_for_udp(struct sk_buff *skb)
{
	struct ethhdr *eth = (struct ethhdr *)skb->data;

	/* hardware can't deal with UFO broadcast packet */
	if (skb_is_gso(skb) && is_broadcast_ether_addr(eth->h_dest))
		return -ENOTSUPP;

	return 0;
}

static int gmac_check_hw_capability_for_ipv6(struct sk_buff *skb)
{
	unsigned int l4_proto;

	l4_proto = ipv6_hdr(skb)->nexthdr;
	if (l4_proto != IPPROTO_TCP && l4_proto != IPPROTO_UDP) {
		/*
		 * When the IPv6 next header is not tcp or udp, it means
		 * that the IPv6 next header is an extension header.
		 * Hardware can't deal with this case, so do checksumming
		 * by software or do GSO by software.
		 */
		if (skb_is_gso(skb))
			return -ENOTSUPP;

		if (skb->ip_summed == CHECKSUM_PARTIAL &&
		    skb_checksum_help(skb))
			return -EFAULT;
	}

	return 0;
}

static __be16 gmac_get_l3_proto(struct sk_buff *skb)
{
	__be16 l3_proto = skb->protocol;

	if (skb->protocol == htons(ETH_P_8021Q))
		l3_proto = vlan_get_protocol(skb);

	return l3_proto;
}

static unsigned int gmac_get_l4_proto(struct sk_buff *skb)
{
	__be16 l3_proto = gmac_get_l3_proto(skb);
	unsigned int l4_proto = IPPROTO_MAX;

	if (l3_proto == htons(ETH_P_IP))
		l4_proto = ip_hdr(skb)->protocol;
	else if (l3_proto == htons(ETH_P_IPV6))
		l4_proto = ipv6_hdr(skb)->nexthdr;

	return l4_proto;
}

static inline bool gmac_skb_is_ipv6(struct sk_buff *skb)
{
	return gmac_get_l3_proto(skb) == htons(ETH_P_IPV6);
}

static inline bool gmac_skb_is_udp(struct sk_buff *skb)
{
	return gmac_get_l4_proto(skb) == IPPROTO_UDP;
}

static inline bool gmac_skb_is_ipv4_with_options(struct sk_buff *skb)
{
	return (gmac_get_l3_proto(skb) == htons(ETH_P_IP)) &&
	       (ip_hdr(skb)->ihl > IPV4_HEAD_LENGTH);
}

static int gmac_check_hw_capability(struct sk_buff *skb)
{
	int ret;

	/*
	 * If tcp_mtu_probe() uses (2 * tp->mss_cache) as probe_size,
	 * the linear data length will be larger than 2048,
	 * the MAC can't handle it, so let the software do it.
	 */
	if (skb_is_gso(skb) && (skb_headlen(skb) > 2048))
		return -ENOTSUPP;

	if (gmac_skb_is_ipv6(skb)) {
		ret = gmac_check_hw_capability_for_ipv6(skb);
		if (ret)
			return ret;
	}

	if (gmac_skb_is_udp(skb)) {
		ret = gmac_check_hw_capability_for_udp(skb);
		if (ret)
			return ret;
	}

	if ((skb->ip_summed == CHECKSUM_PARTIAL || skb_is_gso(skb)) &&
	    gmac_skb_is_ipv4_with_options(skb))
		return -ENOTSUPP;

	return 0;
}

static int gmac_xmit_gso_sg_frag(struct gmac_netdev_local *ld,
				 struct sk_buff *skb, struct sg_desc *desc_cur,
				 struct gmac_tso_desc *tx_bq_desc,
				 unsigned int desc_pos)
{
	int nfrags = skb_shinfo(skb)->nr_frags;
	dma_addr_t addr, dma_addr;
	int i, len;

	for (i = 0; i < nfrags; i++) {
		skb_frag_t *frag = &skb_shinfo(skb)->frags[i];

		len = skb_frag_size(frag);
		dma_addr = skb_frag_dma_map(ld->dev, frag, 0, len,
					    DMA_TO_DEVICE);
		if (dma_mapping_error(ld->dev, dma_addr)) {
			netdev_err(ld->netdev, "skb frag DMA mapping fail\n");
			return -EFAULT;
		}
		desc_cur->frags[i].addr = (u32)dma_addr;
		desc_cur->frags[i].reserved =
			(dma_addr >> REG_BIT_WIDTH) << SG_DESC_HI8_OFFSET;
		desc_cur->frags[i].size = len;
	}

	addr = ld->dma_sg_phy + ld->sg_head * sizeof(struct sg_desc);
	tx_bq_desc->data_buff_addr = (u32)addr;
	tx_bq_desc->reserve_desc2 =
		(addr >> REG_BIT_WIDTH) & TX_DESC_HI8_MASK;
	ld->TX_BQ.sg_desc_offset[desc_pos] = ld->sg_head;

	ld->sg_head = (ld->sg_head + 1) % ld->sg_count;

	return 0;
}

static int gmac_xmit_gso_sg(struct gmac_netdev_local *ld,
			    struct sk_buff *skb,
			    struct gmac_tso_desc *tx_bq_desc,
			    unsigned int desc_pos)
{
	struct sg_desc *desc_cur;
	dma_addr_t dma_addr;
	int ret;

	if (unlikely(((ld->sg_head + 1) % ld->sg_count) == ld->sg_tail)) {
		/* SG pkt, but all sg descs are used */
		netdev_err(ld->netdev, "sg desc all used\n");
		return -EBUSY;
	}

	desc_cur = ld->dma_sg_desc + ld->sg_head;

	desc_cur->total_len = skb->len;
	desc_cur->linear_len = skb_headlen(skb);
	dma_addr = dma_map_single(ld->dev, skb->data, desc_cur->linear_len,
				  DMA_TO_DEVICE);
	if (dma_mapping_error(ld->dev, dma_addr)) {
		netdev_err(ld->netdev, "DMA mapping fail\n");
		return -EFAULT;
	}
	desc_cur->linear_addr = (u32)dma_addr;
	desc_cur->reserv3 =
		(dma_addr >> REG_BIT_WIDTH) << SG_DESC_HI8_OFFSET;

	ret = gmac_xmit_gso_sg_frag(ld, skb, desc_cur, tx_bq_desc, desc_pos);
	if (unlikely(ret))
		return ret;

	return 0;
}

static int gmac_xmit_gso(struct gmac_netdev_local *ld, struct sk_buff *skb,
			 struct gmac_tso_desc *tx_bq_desc, unsigned int desc_pos)
{
	int pkt_type = PKT_NORMAL;
	int nfrags = skb_shinfo(skb)->nr_frags;
	dma_addr_t addr;
	int ret;

	if (skb_is_gso(skb) || nfrags)
		pkt_type = PKT_SG; /* TSO pkt or SG pkt */

	ret = gmac_check_hw_capability(skb);
	if (unlikely(ret))
		return ret;

	ret = gmac_get_pkt_info(skb, tx_bq_desc);
	if (unlikely(ret))
		return ret;

	if (pkt_type == PKT_NORMAL) {
		addr = dma_map_single(ld->dev, skb->data, skb->len,
				      DMA_TO_DEVICE);
		if (dma_mapping_error(ld->dev, addr)) {
			netdev_err(ld->netdev, "Normal packet DMA mapping fail\n");
			return -EFAULT;
		}
		tx_bq_desc->data_buff_addr = (u32)addr;
		tx_bq_desc->reserve_desc2 =
			(addr >> REG_BIT_WIDTH) & TX_DESC_HI8_MASK;
	} else {
		ret = gmac_xmit_gso_sg(ld, skb, tx_bq_desc, desc_pos);
		if (unlikely(ret))
			return ret;
	}

	return 0;
}

static netdev_tx_t gmac_net_xmit(struct sk_buff *skb, struct net_device *dev);

static netdev_tx_t gmac_sw_gso(struct gmac_netdev_local *ld,
			       struct sk_buff *skb)
{
	struct sk_buff *segs, *curr_skb;
	int ret;
	int gso_segs = skb_shinfo(skb)->gso_segs;

	if (gso_segs == 0 && skb_shinfo(skb)->gso_size != 0)
		gso_segs = DIV_ROUND_UP(skb->len, skb_shinfo(skb)->gso_size);

	/* Estimate the number of fragments in the worst case */
	if (unlikely(gmac_tx_avail(ld) < gso_segs)) {
		netif_stop_queue(ld->netdev);
		if (gmac_tx_avail(ld) < gso_segs) {
			ld->netdev->stats.tx_dropped++;
			ld->netdev->stats.tx_fifo_errors++;
			return NETDEV_TX_BUSY;
		}
		netif_wake_queue(ld->netdev);
	}

	segs = skb_gso_segment(skb, ld->netdev->features &
			       ~(NETIF_F_CSUM_MASK | NETIF_F_SG |
				 NETIF_F_GSO_SOFTWARE));
	if (IS_ERR_OR_NULL(segs))
		goto drop;

	do {
		curr_skb = segs;
		segs = segs->next;
		curr_skb->next = NULL;
		ret = gmac_net_xmit(curr_skb, ld->netdev);
		if (unlikely(ret != NETDEV_TX_OK))
			netdev_err_once(ld->netdev,
					"gmac_net_xmit error ret=%d\n", ret);
	} while (segs);

	dev_kfree_skb_any(skb);
	return NETDEV_TX_OK;

drop:
	dev_kfree_skb_any(skb);
	ld->netdev->stats.tx_dropped++;
	return NETDEV_TX_OK;
}

static netdev_tx_t gmac_net_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct gmac_netdev_local *ld = netdev_priv(dev);
	struct gmac_desc *desc;
	unsigned long txflags;
	int ret;
	u32 pos;

	if (unlikely(gmac_check_skb_len(skb, dev) < 0))
		return NETDEV_TX_OK;

	/* software write pointer */
	pos = dma_cnt(readl(ld->gmac_iobase + TX_BQ_WR_ADDR));

	spin_lock_irqsave(&ld->txlock, txflags);

	if (unlikely(ld->tx_skb[pos] || ld->TX_BQ.skb[pos])) {
		dev->stats.tx_dropped++;
		dev->stats.tx_fifo_errors++;
		netif_stop_queue(dev);
		spin_unlock_irqrestore(&ld->txlock, txflags);

		return NETDEV_TX_BUSY;
	}

	ld->TX_BQ.skb[pos] = skb;
	ld->tx_skb[pos] = skb;

	desc = ld->TX_BQ.desc + pos;

	if (ld->tso_supported) {
		ret = gmac_xmit_gso(ld, skb, (struct gmac_tso_desc *)desc, pos);
		if (unlikely(ret < 0)) {
			ld->tx_skb[pos] = NULL;
			ld->TX_BQ.skb[pos] = NULL;
			spin_unlock_irqrestore(&ld->txlock, txflags);

			if (ret == -ENOTSUPP)
				return gmac_sw_gso(ld, skb);

			dev_kfree_skb_any(skb);
			dev->stats.tx_dropped++;
			return NETDEV_TX_OK;
		}
	} else {
		ret = gmac_net_xmit_normal(skb, dev, desc, pos);
		if (unlikely(ret < 0)) {
			spin_unlock_irqrestore(&ld->txlock, txflags);
			return NETDEV_TX_OK;
		}
	}

	/*
	 * This barrier is important here.  It is required to ensure
	 * the ARM CPU flushes its DMA write buffers before proceeding
	 * to the next instruction, to ensure that GMAC will see
	 * our descriptor changes in memory.
	 */
	gmac_sync_barrier();
	pos = dma_ring_incr(pos, TX_DESC_NUM);
	writel(dma_byte(pos), ld->gmac_iobase + TX_BQ_WR_ADDR);

	netif_trans_update(dev);
	dev->stats.tx_packets++;
	dev->stats.tx_bytes += skb->len;
	netdev_sent_queue(dev, skb->len);

	spin_unlock_irqrestore(&ld->txlock, txflags);

	return NETDEV_TX_OK;
}

/* set gmac's multicast list, here we setup gmac's mc filter */
static void gmac_set_multicast_list(struct net_device *dev)
{
	struct gmac_netdev_local *ld = netdev_priv(dev);
	struct netdev_hw_addr *ha;
	unsigned int d;
	unsigned int rec_filter;

	rec_filter = readl(ld->gmac_iobase + REC_FILT_CONTROL);
	/*
	 * when set gmac in promisc mode
	 * a. dev in IFF_PROMISC mode
	 */
	if (dev->flags & IFF_PROMISC) {
		/* promisc mode, receive all pkgs */
		rec_filter &= ~(BIT_BC_DROP_EN | BIT_MC_MATCH_EN |
				BIT_UC_MATCH_EN);
	} else {
		/* drop uc pkgs with field 'DA' not match ours */
		rec_filter |= BIT_UC_MATCH_EN;

		if (dev->flags & IFF_BROADCAST) /* no broadcast */
			rec_filter &= ~BIT_BC_DROP_EN;
		else
			rec_filter |= BIT_BC_DROP_EN;

		if (netdev_mc_empty(dev) || !(dev->flags & IFF_MULTICAST)) {
			/* haven't joined any mc group */
			writel(0, ld->gmac_iobase + PORT_MC_ADDR_LOW);
			writel(0, ld->gmac_iobase + PORT_MC_ADDR_HIGH);
			rec_filter |= BIT_MC_MATCH_EN;
		} else if ((netdev_mc_count(dev) == 1) &&
			   (dev->flags & IFF_MULTICAST)) {
			netdev_for_each_mc_addr(ha, dev) {
				/* mac[0]->(15, 8) mac[1]->(7, 0) */
				d = (ha->addr[0] << 8) | ha->addr[1];
				writel(d, ld->gmac_iobase + PORT_MC_ADDR_HIGH);

				/* mac[2]->(31, 24) mac[3]->(23, 16)
				 * mac[4]->(15, 8) mac[5]->(7, 0)
				 */
				d = (ha->addr[2] << 24) | (ha->addr[3] << 16) |
				    (ha->addr[4] << 8) | ha->addr[5];
				writel(d, ld->gmac_iobase + PORT_MC_ADDR_LOW);
			}
			rec_filter |= BIT_MC_MATCH_EN;
		} else {
			rec_filter &= ~BIT_MC_MATCH_EN;
		}
	}
	writel(rec_filter, ld->gmac_iobase + REC_FILT_CONTROL);
}

static int gmac_set_features(struct net_device *dev, netdev_features_t features)
{
	struct gmac_netdev_local *ld = netdev_priv(dev);
	netdev_features_t changed = dev->features ^ features;

	if (changed & NETIF_F_RXCSUM)
		gmac_enable_rxcsum_drop(ld, !!(features & NETIF_F_RXCSUM));

	return 0;
}

static int gmac_net_set_mac_address(struct net_device *dev, void *p)
{
	int ret;

	ret = eth_mac_addr(dev, p);
	if (!ret)
		gmac_hw_set_mac_addr(dev);

	return ret;
}

static int gmac_net_ioctl(struct net_device *dev, struct ifreq *ifr, int cmd)
{
	struct gmac_netdev_local *ld = netdev_priv(dev);

	if (!netif_running(dev) || !ld->phy)
		return -EINVAL;

	return phy_mii_ioctl(ld->phy, ifr, cmd);
}

static const struct net_device_ops gmac_netdev_ops = {
	.ndo_open = gmac_net_open,
	.ndo_stop = gmac_net_close,
	.ndo_start_xmit = gmac_net_xmit,
	.ndo_set_rx_mode = gmac_set_multicast_list,
	.ndo_set_features = gmac_set_features,
	.ndo_eth_ioctl = gmac_net_ioctl,
	.ndo_set_mac_address = gmac_net_set_mac_address,
	.ndo_validate_addr = eth_validate_addr,
};

/* probe and platform driver */

static void gmac_destroy_hw_desc_queue(struct gmac_netdev_local *ld)
{
	int i;

	for (i = 0; i < QUEUE_NUMS + RSS_NUM_RXQS - 1; i++) {
		if (ld->pool[i].desc) {
			dma_free_coherent(ld->dev, ld->pool[i].size,
					  ld->pool[i].desc, ld->pool[i].phys_addr);
			ld->pool[i].desc = NULL;
		}
	}

	kfree(ld->RX_FQ.skb);
	kfree(ld->TX_BQ.skb);
	ld->RX_FQ.skb = NULL;
	ld->TX_BQ.skb = NULL;

	if (ld->tso_supported) {
		kfree(ld->TX_BQ.sg_desc_offset);
		ld->TX_BQ.sg_desc_offset = NULL;
	}

	kfree(ld->tx_skb);
	ld->tx_skb = NULL;

	kfree(ld->rx_skb);
	ld->rx_skb = NULL;
}

static int gmac_init_desc_queue_mem(struct gmac_netdev_local *ld)
{
	ld->RX_FQ.skb = kcalloc(ld->RX_FQ.count, sizeof(struct sk_buff *),
				GFP_KERNEL);
	if (!ld->RX_FQ.skb)
		return -ENOMEM;

	ld->rx_skb = kcalloc(ld->RX_FQ.count, sizeof(struct sk_buff *),
			     GFP_KERNEL);
	if (!ld->rx_skb)
		return -ENOMEM;

	ld->TX_BQ.skb = kcalloc(ld->TX_BQ.count, sizeof(struct sk_buff *),
				GFP_KERNEL);
	if (!ld->TX_BQ.skb)
		return -ENOMEM;

	ld->tx_skb = kcalloc(ld->TX_BQ.count, sizeof(struct sk_buff *),
			     GFP_KERNEL);
	if (!ld->tx_skb)
		return -ENOMEM;

	if (ld->tso_supported) {
		ld->TX_BQ.sg_desc_offset =
			kcalloc(ld->TX_BQ.count, sizeof(int), GFP_KERNEL);
		if (!ld->TX_BQ.sg_desc_offset)
			return -ENOMEM;
	}

	return 0;
}

static int gmac_init_hw_desc_queue(struct gmac_netdev_local *ld)
{
	struct device *dev = ld->dev;
	struct gmac_desc *virt_addr;
	dma_addr_t phys_addr;
	int size, i;

	ld->RX_FQ.count = RX_DESC_NUM;
	ld->RX_BQ.count = RX_DESC_NUM;
	ld->TX_BQ.count = TX_DESC_NUM;
	ld->TX_RQ.count = TX_DESC_NUM;

	for (i = 1; i < RSS_NUM_RXQS; i++)
		ld->pool[BASE_QUEUE_NUMS + i].count = RX_DESC_NUM;

	for (i = 0; i < (QUEUE_NUMS + RSS_NUM_RXQS - 1); i++) {
		size = ld->pool[i].count * sizeof(struct gmac_desc);
		virt_addr = dma_alloc_coherent(dev, size, &phys_addr,
					       GFP_KERNEL | __GFP_ZERO);
		if (!virt_addr)
			goto error_free_pool;

		ld->pool[i].size = size;
		ld->pool[i].desc = virt_addr;
		ld->pool[i].phys_addr = phys_addr;
	}

	if (gmac_init_desc_queue_mem(ld) == -ENOMEM)
		goto error_free_pool;

	gmac_hw_set_desc_addr(ld);

	return 0;

error_free_pool:
	gmac_destroy_hw_desc_queue(ld);

	return -ENOMEM;
}

static void gmac_init_napi(struct gmac_netdev_local *ld)
{
	int i;

	for (i = 0; i < ld->num_rxqs; i++) {
		ld->q_napi[i].rxq_id = i;
		ld->q_napi[i].ndev_priv = ld;
		netif_napi_add(ld->netdev, &ld->q_napi[i].napi, gmac_poll);
	}
}

static void gmac_destroy_napi(struct gmac_netdev_local *ld)
{
	int i;

	for (i = 0; i < ld->num_rxqs; i++)
		netif_napi_del(&ld->q_napi[i].napi);
}

static int gmac_request_irqs(struct platform_device *pdev,
			     struct gmac_netdev_local *ld)
{
	struct device *dev = ld->dev;
	int ret;
	int i;

	for (i = 0; i < ld->num_rxqs; i++) {
		ret = platform_get_irq(pdev, i);
		if (ret < 0) {
			dev_err(dev, "No irq[%d] resource, ret=%d\n", i, ret);
			return ret;
		}
		ld->irq[i] = ret;

		ret = devm_request_irq(dev, ld->irq[i], gmac_interrupt,
				       IRQF_SHARED, pdev->name,
				       &ld->q_napi[i]);
		if (ret) {
			dev_err(dev, "devm_request_irq failed, ret=%d\n", ret);
			return ret;
		}
	}

	return 0;
}

static int gmac_dev_probe_res(struct platform_device *pdev,
			      struct gmac_netdev_local *ld)
{
	struct device *dev = ld->dev;

	ld->gmac_iobase = devm_platform_ioremap_resource(pdev, MEM_GMAC_IOBASE);
	if (IS_ERR(ld->gmac_iobase))
		return PTR_ERR(ld->gmac_iobase);

	ld->macif_base = devm_platform_ioremap_resource(pdev, MEM_MACIF_IOBASE);
	if (IS_ERR(ld->macif_base))
		return PTR_ERR(ld->macif_base);

	ld->port_rst = devm_reset_control_get_exclusive(dev, GMAC_PORT_RST_NAME);
	if (IS_ERR(ld->port_rst))
		return PTR_ERR(ld->port_rst);

	ld->macif_rst = devm_reset_control_get_exclusive(dev,
							 GMAC_MACIF_RST_NAME);
	if (IS_ERR(ld->macif_rst))
		return PTR_ERR(ld->macif_rst);

	ld->phy_rst = devm_reset_control_get_optional_exclusive(dev,
								GMAC_PHY_RST_NAME);
	if (IS_ERR(ld->phy_rst))
		return PTR_ERR(ld->phy_rst);

	ld->clk = devm_clk_get_enabled(dev, GMAC_MAC_CLK_NAME);
	if (IS_ERR(ld->clk)) {
		dev_err(dev, "failed to get and enable gmac clk\n");
		return PTR_ERR(ld->clk);
	}

	ld->macif_clk = devm_clk_get_optional_enabled(dev, GMAC_MACIF_CLK_NAME);
	if (IS_ERR(ld->macif_clk)) {
		dev_err(dev, "failed to get and enable macif clk\n");
		return PTR_ERR(ld->macif_clk);
	}

	return 0;
}

static void gmac_set_hw_cap(struct gmac_netdev_local *ld)
{
	unsigned int hw_cap;

	hw_cap = readl(ld->gmac_iobase + CRF_MIN_PACKET);
	ld->tso_supported = has_tso_cap(hw_cap);
	ld->has_rxhash_cap = has_rxhash_cap(hw_cap);
	ld->has_rss_cap = has_rss_cap(hw_cap);

	gmac_set_rss_cap(ld);
	gmac_get_rss_key(ld);
	if (ld->has_rss_cap) {
		ld->rss_info.ind_tbl_size = RSS_INDIRECTION_TABLE_SIZE;
		gmac_get_rss(ld);
	}

	if (ld->has_rxhash_cap) {
		ld->rss_info.hash_cfg = DEF_HASH_CFG;
		gmac_config_hash_policy(ld);
	}
}

static int gmac_set_mac_addr(struct net_device *ndev, struct device_node *node)
{
	int ret;

	ret = of_get_ethdev_address(node, ndev);
	if (ret)
		eth_hw_addr_random(ndev);

	gmac_hw_set_mac_addr(ndev);

	return 0;
}

static void gmac_hw_external_phy_reset(struct gmac_netdev_local *ld)
{
	if (!ld->phy_rst)
		return;

	/* write 0 to cancel reset */
	reset_control_deassert(ld->phy_rst);
	msleep(50); /* wait 50ms */

	/* RST_BIT, write 1 to reset phy, write 0 to cancel reset */
	reset_control_assert(ld->phy_rst);
	/*
	 * Delay some time to ensure reset ok,
	 * this depends on the PHY hardware feature.
	 */
	msleep(50); /* wait 50ms */

	/* write 0 to cancel reset */
	reset_control_deassert(ld->phy_rst);
	/* delay some time to ensure later MDIO access */
	msleep(50); /* wait 50ms */
}

static int gmac_phy_init(struct device *dev, struct net_device *ndev,
			 struct gmac_netdev_local *ld,
			 struct device_node *node, bool *fixed_link)
{
	int ret;

	/*
	 * phy reset should come before of_phy_connect, because
	 * the PHY registers are read over MDIO during probe.
	 */
	gmac_hw_external_phy_reset(ld);

	ret = of_get_phy_mode(node, &ld->phy_mode);
	if (ret < 0) {
		netdev_err(ndev, "not find phy-mode\n");
		return ret;
	}

	ld->phy_node = of_parse_phandle(node, "phy-handle", 0);
	if (!ld->phy_node) {
		/* check if a fixed-link is defined in device-tree */
		if (of_phy_is_fixed_link(node)) {
			ret = of_phy_register_fixed_link(node);
			if (ret < 0) {
				dev_err(dev, "cannot register fixed PHY %d\n", ret);
				return ret;
			}

			/*
			 * In the case of a fixed PHY, the DT node associated
			 * to the PHY is the Ethernet MAC DT node.
			 */
			ld->phy_node = of_node_get(node);
			*fixed_link = true;
		} else {
			netdev_err(ndev, "not find phy-handle\n");
			return -EINVAL;
		}
	}
	return 0;
}

static int gmac_dev_probe_phy(struct platform_device *pdev,
			      struct gmac_netdev_local *ld,
			      struct net_device *ndev, bool fixed_link)
{
	int ret;

	ld->phy = of_phy_connect(ndev, ld->phy_node, &gmac_adjust_link, 0,
				 ld->phy_mode);
	if (!ld->phy || !ld->phy->drv) {
		ret = -ENODEV;
		return ret;
	}

	/* If the phy_id is all zero and not fixed link, there is no device there */
	if (ld->phy->phy_id == 0 && !fixed_link) {
		netdev_info(ndev, "phy %d not found\n", ld->phy->mdio.addr);
		ret = -ENODEV;
		return ret;
	}

	netdev_info(ndev, "attached PHY %d to driver %s, PHY_ID=0x%x\n",
		    ld->phy->mdio.addr, ld->phy->drv->name, ld->phy->phy_id);

	/* Stop advertising 1000BASE capability if interface is not RGMII */
	if (ld->phy_mode == PHY_INTERFACE_MODE_MII ||
	    ld->phy_mode == PHY_INTERFACE_MODE_RMII) {
		linkmode_clear_bit(ETHTOOL_LINK_MODE_1000baseT_Full_BIT,
				   ld->phy->advertising);
		linkmode_clear_bit(ETHTOOL_LINK_MODE_1000baseT_Half_BIT,
				   ld->phy->advertising);
		/*
		 * The internal FE phy's reg BMSR bit8 is wrong, making
		 * the kernel believe it has the 1000base capability,
		 * so fix it here.
		 */
		if (ld->phy->phy_id == VENDOR_PHY_ID_FESTAV200) {
			linkmode_clear_bit(ETHTOOL_LINK_MODE_1000baseT_Full_BIT,
					   ld->phy->supported);
			linkmode_clear_bit(ETHTOOL_LINK_MODE_1000baseT_Half_BIT,
					   ld->phy->supported);
		}
	}

	ld->flow_ctrl = FLOW_AUTO;
	ld->pause = FC_PAUSE_TIME_DEFAULT;
	ld->pause_interval = FC_PAUSE_INTERVAL_DEFAULT;
	ld->flow_ctrl_active_threshold = FC_ACTIVE_DEFAULT;
	ld->flow_ctrl_deactive_threshold = FC_DEACTIVE_DEFAULT;
	gmac_set_flow_ctrl_params(ld);
	linkmode_set_bit(ETHTOOL_LINK_MODE_Pause_BIT, ld->phy->supported);
	if (ld->flow_ctrl)
		linkmode_set_bit(ETHTOOL_LINK_MODE_Pause_BIT,
				 ld->phy->advertising);

	return gmac_request_irqs(pdev, ld);
}

static int gmac_dev_probe_init(struct platform_device *pdev,
			       struct gmac_netdev_local *ld,
			       struct net_device *ndev)
{
	int ret;

	gmac_init_napi(ld);
	spin_lock_init(&ld->rxlock);
	spin_lock_init(&ld->txlock);

	/* init netdevice */
	ndev->irq = ld->irq[0];
	ndev->watchdog_timeo = 3 * HZ;
	ndev->netdev_ops = &gmac_netdev_ops;
	ndev->ethtool_ops = &gmac_ethtool_ops;

	if (ld->has_rxhash_cap)
		ndev->hw_features |= NETIF_F_RXHASH;
	if (ld->has_rss_cap)
		ndev->hw_features |= NETIF_F_NTUPLE;
	if (ld->tso_supported)
		ndev->hw_features |= NETIF_F_SG |
				     NETIF_F_IP_CSUM | NETIF_F_IPV6_CSUM |
				     NETIF_F_TSO | NETIF_F_TSO6;

	ndev->hw_features |= NETIF_F_RXCSUM;

	ndev->features |= ndev->hw_features;
	ndev->features |= NETIF_F_HIGHDMA | NETIF_F_GSO;
	ndev->vlan_features |= ndev->features;

	timer_setup(&ld->monitor, gmac_monitor_func, 0);

	ld->msg_enable = netif_msg_init(debug, DEFAULT_MSG_ENABLE);

	ret = dma_set_mask_and_coherent(ld->dev, DMA_BIT_MASK(64));
	if (ret) {
		dev_err(ld->dev, "dma set mask 64 failed! ret=%d\n", ret);
		return ret;
	}

	/* init hw desc queue */
	ret = gmac_init_hw_desc_queue(ld);
	if (ret)
		return ret;

	return 0;
}

static int gmac_dev_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	struct net_device *ndev;
	struct gmac_netdev_local *ld;
	int ret;
	bool fixed_link = false;

	ndev = alloc_etherdev_mqs(sizeof(struct gmac_netdev_local), 1,
				  RSS_NUM_RXQS);
	if (!ndev)
		return -ENOMEM;

	platform_set_drvdata(pdev, ndev);
	SET_NETDEV_DEV(ndev, dev);

	ld = netdev_priv(ndev);
	ld->dev = dev;
	ld->netdev = ndev;
	ld->num_rxqs = RSS_NUM_RXQS;

	ret = gmac_dev_probe_res(pdev, ld);
	if (ret)
		goto out_free_netdev;

	gmac_mac_core_reset(ld);

	ret = gmac_phy_init(dev, ndev, ld, node, &fixed_link);
	if (ret)
		goto out_free_netdev;

	gmac_set_mac_addr(ndev, node);
	gmac_set_hw_cap(ld);

	/* init hw controller */
	gmac_hw_init(ld);

	ret = gmac_dev_probe_phy(pdev, ld, ndev, fixed_link);
	if (ret) {
		if (!ld->phy)
			goto out_phy_node;
		else
			goto out_phy_disconnect;
	}

	ret = gmac_dev_probe_init(pdev, ld, ndev);
	if (ret)
		goto out_destroy_hw_desc_queue;

	if (ld->tso_supported) {
		ret = gmac_init_sg_desc_queue(ld);
		if (ret)
			goto out_destroy_hw_desc_queue;
	}

	/* register netdevice */
	ret = register_netdev(ld->netdev);
	if (ret) {
		netdev_err(ndev, "register_netdev failed!\n");
		goto out_destroy_sg_desc_queue;
	}

	/*
	 * Reset the queue here so that BQL is only reset once.
	 * If netdev_reset_queue() were called in gmac_net_open(),
	 * BQL would be reset when bringing eth0 down and up again,
	 * while the tx ring is not cleared before that, and the
	 * NAPI poll would call netdev_completed_queue() with a
	 * stale BQL state.
	 */
	netdev_reset_queue(ndev);

	/* config PHY power down to save power */
	phy_suspend(ld->phy);

	netdev_info(ndev, "ETH: %s, phy_addr=%d\n",
		    phy_modes(ld->phy_mode), ld->phy->mdio.addr);

	return 0;

out_destroy_sg_desc_queue:
	if (ld->tso_supported)
		gmac_destroy_sg_desc_queue(ld);
out_destroy_hw_desc_queue:
	gmac_destroy_hw_desc_queue(ld);
	gmac_destroy_napi(ld);
out_phy_disconnect:
	phy_disconnect(ld->phy);
out_phy_node:
	of_node_put(ld->phy_node);
out_free_netdev:
	free_netdev(ndev);

	return ret;
}

static void gmac_dev_remove(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct gmac_netdev_local *ld = netdev_priv(ndev);

	/* stop the gmac and free all resources */
	timer_delete_sync(&ld->monitor);
	gmac_destroy_napi(ld);

	unregister_netdev(ndev);

	gmac_reclaim_rx_tx_resource(ld);
	gmac_free_rx_skb(ld);
	gmac_free_tx_skb(ld);

	if (ld->tso_supported)
		gmac_destroy_sg_desc_queue(ld);
	gmac_destroy_hw_desc_queue(ld);

	phy_disconnect(ld->phy);
	of_node_put(ld->phy_node);

	free_netdev(ndev);
}

static const struct of_device_id gmac_of_match[] = {
	{ .compatible = "hisilicon,gmac-v5" },
	{ }
};
MODULE_DEVICE_TABLE(of, gmac_of_match);

static struct platform_driver gmac_driver = {
	.probe = gmac_dev_probe,
	.remove = gmac_dev_remove,
	.driver = {
		.name = GMAC_DRIVER_NAME,
		.of_match_table = gmac_of_match,
	},
};

module_platform_driver(gmac_driver);

MODULE_AUTHOR("Jun Yan <jerrysteve1101@gmail.com>");
MODULE_DESCRIPTION("HiSilicon GEMAC v5 Ethernet MAC driver");
MODULE_LICENSE("GPL");
