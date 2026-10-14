// SPDX-License-Identifier: GPL-2.0-only
/*
 * Hi3519DV500 Clock Driver
 *
 * Copyright (c) 2020-2023 HiSilicon Technologies Co., Ltd.
 */

#include <dt-bindings/clock/hi3519dv500-clock.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include "clk.h"
#include "reset.h"

static const struct hisi_fixed_rate_clock hi3519dv500_fixed_rate_clks[] = {
	{ HI3519DV500_FIXED_2400M, "2400m", NULL, 0, 2400000000, },
	{ HI3519DV500_FIXED_1200M, "1200m", NULL, 0, 1200000000, },
	{ HI3519DV500_FIXED_1188M, "1188m", NULL, 0, 1188000000, },
	{ HI3519DV500_FIXED_896M, "896m", NULL, 0, 896000000, },
	{ HI3519DV500_FIXED_800M, "800m", NULL, 0, 800000000, },
	{ HI3519DV500_FIXED_792M, "792m", NULL, 0, 792000000, },
	{ HI3519DV500_FIXED_786M, "786m", NULL, 0, 786000000, },
	{ HI3519DV500_FIXED_750M, "750m", NULL, 0, 750000000, },
	{ HI3519DV500_FIXED_700M, "700m", NULL, 0, 700000000, },
	{ HI3519DV500_FIXED_672M, "672m", NULL, 0, 672000000, },
	{ HI3519DV500_FIXED_600M, "600m", NULL, 0, 600000000, },
	{ HI3519DV500_FIXED_594M, "594m", NULL, 0, 594000000, },
	{ HI3519DV500_FIXED_560M, "560m", NULL, 0, 560000000, },
	{ HI3519DV500_FIXED_500M, "500m", NULL, 0, 500000000, },
	{ HI3519DV500_FIXED_475M, "475m", NULL, 0, 475000000, },
	{ HI3519DV500_FIXED_396M, "396m", NULL, 0, 396000000, },
	{ HI3519DV500_FIXED_300M, "300m", NULL, 0, 300000000, },
	{ HI3519DV500_FIXED_297M, "297m", NULL, 0, 297000000, },
	{ HI3519DV500_FIXED_257M, "257m", NULL, 0, 257000000, },
	{ HI3519DV500_FIXED_250M, "250m", NULL, 0, 250000000, },
	{ HI3519DV500_FIXED_200M, "200m", NULL, 0, 200000000, },
	{ HI3519DV500_FIXED_198M, "198m", NULL, 0, 198000000, },
	{ HI3519DV500_FIXED_187P_5M, "187p5m", NULL, 0, 187500000, },
	{ HI3519DV500_FIXED_150M, "150m", NULL, 0, 150000000, },
	{ HI3519DV500_FIXED_148P_5M, "148p5m", NULL, 0, 148500000, },
	{ HI3519DV500_FIXED_134M, "134m", NULL, 0, 134000000, },
	{ HI3519DV500_FIXED_108M, "108m", NULL, 0, 108000000, },
	{ HI3519DV500_FIXED_100M, "100m", NULL, 0, 100000000, },
	{ HI3519DV500_FIXED_99M, "99m", NULL, 0, 99000000, },
	{ HI3519DV500_FIXED_74P_25M, "74p25m", NULL, 0, 74250000, },
	{ HI3519DV500_FIXED_72M, "72m", NULL, 0, 72000000, },
	{ HI3519DV500_FIXED_64M, "64m", NULL, 0, 64000000, },
	{ HI3519DV500_FIXED_60M, "60m", NULL, 0, 60000000, },
	{ HI3519DV500_FIXED_54M, "54m", NULL, 0, 54000000, },
	{ HI3519DV500_FIXED_50M, "50m", NULL, 0, 50000000, },
	{ HI3519DV500_FIXED_49P_5M, "49p5m", NULL, 0, 49500000, },
	{ HI3519DV500_FIXED_37P_125M, "37p125m", NULL, 0, 37125000, },
	{ HI3519DV500_FIXED_36M, "36m", NULL, 0, 36000000, },
	{ HI3519DV500_FIXED_27M, "27m", NULL, 0, 27000000, },
	{ HI3519DV500_FIXED_25M, "25m", NULL, 0, 25000000, },
	{ HI3519DV500_FIXED_24M, "24m", NULL, 0, 24000000, },
	{ HI3519DV500_FIXED_12M, "12m", NULL, 0, 12000000, },
	{ HI3519DV500_FIXED_12P_288M, "12p288m", NULL, 0, 12288000, },
	{ HI3519DV500_FIXED_6M, "6m", NULL, 0, 6000000, },
	{ HI3519DV500_FIXED_3M, "3m", NULL, 0, 3000000, },
	{ HI3519DV500_FIXED_1P_6M, "1p6m", NULL, 0, 1600000, },
	{ HI3519DV500_FIXED_400K, "400k", NULL, 0, 400000, },
	{ HI3519DV500_FIXED_100K, "100k", NULL, 0, 100000, },
};

static const char *const fmc_mux_p[] = {
	"24m", "100m", "150m", "198m", "237m", "300m", "396m"
};
static u32 fmc_mux_table[] = {0, 1, 2, 3, 4, 5, 6};

static const char *const mmc_mux_p[] = {
	"400k", "25m", "50m", "100m", "150m", "187p5m", "200m"
};
static u32 mmc_mux_table[] = {0, 1, 2, 3, 4, 5, 6, 7};

static const char *const uart_mux_p[] = { "100m", "50m", "24m", "3m" };
static u32 uart_mux_table[] = {0, 1, 2, 3};

static const char *const i2c_mux_p[] = { "50m", "100m" };
static u32 i2c_mux_table[] = {0, 1};

static const char *const pwm_mux_p[] = { "198m" };
static u32 pwm_mux_table[] = {0};

static const struct hisi_mux_clock hi3519dv500_mux_clks[] = {
	{ HI3519DV500_FMC_MUX, "fmc_mux", fmc_mux_p, ARRAY_SIZE(fmc_mux_p),
		CLK_SET_RATE_PARENT, 0x3f40, 12, 3, 0, fmc_mux_table, },
	{ HI3519DV500_MMC0_MUX, "mmc0_mux", mmc_mux_p, ARRAY_SIZE(mmc_mux_p),
		CLK_SET_RATE_PARENT, 0x34c0, 24, 3, 0, mmc_mux_table, },
	{ HI3519DV500_MMC1_MUX, "mmc1_mux", mmc_mux_p, ARRAY_SIZE(mmc_mux_p),
		CLK_SET_RATE_PARENT, 0x35c0, 24, 3, 0, mmc_mux_table, },
	{ HI3519DV500_MMC2_MUX, "mmc2_mux", mmc_mux_p, ARRAY_SIZE(mmc_mux_p),
		CLK_SET_RATE_PARENT, 0x36c0, 24, 3, 0, mmc_mux_table, },
	{ HI3519DV500_UART0_MUX, "uart0_mux", uart_mux_p, ARRAY_SIZE(uart_mux_p),
		CLK_SET_RATE_PARENT, 0x4180, 12, 2, 0, uart_mux_table, },
	{ HI3519DV500_UART1_MUX, "uart1_mux", uart_mux_p, ARRAY_SIZE(uart_mux_p),
		CLK_SET_RATE_PARENT, 0x4188, 12, 2, 0, uart_mux_table, },
	{ HI3519DV500_UART2_MUX, "uart2_mux", uart_mux_p, ARRAY_SIZE(uart_mux_p),
		CLK_SET_RATE_PARENT, 0x4190, 12, 2, 0, uart_mux_table, },
	{ HI3519DV500_UART3_MUX, "uart3_mux", uart_mux_p, ARRAY_SIZE(uart_mux_p),
		CLK_SET_RATE_PARENT, 0x4198, 12, 2, 0, uart_mux_table, },
	{ HI3519DV500_UART4_MUX, "uart4_mux", uart_mux_p, ARRAY_SIZE(uart_mux_p),
		CLK_SET_RATE_PARENT, 0x41a0, 12, 2, 0, uart_mux_table, },
	{ HI3519DV500_UART5_MUX, "uart5_mux", uart_mux_p, ARRAY_SIZE(uart_mux_p),
		CLK_SET_RATE_PARENT, 0x41a8, 12, 2, 0, uart_mux_table, },
	{ HI3519DV500_I2C0_MUX, "i2c0_mux", i2c_mux_p, ARRAY_SIZE(i2c_mux_p),
		CLK_SET_RATE_PARENT, 0x4280, 12, 1, 0, i2c_mux_table, },
	{ HI3519DV500_I2C1_MUX, "i2c1_mux", i2c_mux_p, ARRAY_SIZE(i2c_mux_p),
		CLK_SET_RATE_PARENT, 0x4288, 12, 1, 0, i2c_mux_table, },
	{ HI3519DV500_I2C2_MUX, "i2c2_mux", i2c_mux_p, ARRAY_SIZE(i2c_mux_p),
		CLK_SET_RATE_PARENT, 0x4290, 12, 1, 0, i2c_mux_table, },
	{ HI3519DV500_I2C3_MUX, "i2c3_mux", i2c_mux_p, ARRAY_SIZE(i2c_mux_p),
		CLK_SET_RATE_PARENT, 0x4298, 12, 1, 0, i2c_mux_table, },
	{ HI3519DV500_I2C4_MUX, "i2c4_mux", i2c_mux_p, ARRAY_SIZE(i2c_mux_p),
		CLK_SET_RATE_PARENT, 0x42a0, 12, 1, 0, i2c_mux_table, },
	{ HI3519DV500_I2C5_MUX, "i2c5_mux", i2c_mux_p, ARRAY_SIZE(i2c_mux_p),
		CLK_SET_RATE_PARENT, 0x42a8, 12, 1, 0, i2c_mux_table, },
	{ HI3519DV500_I2C6_MUX, "i2c6_mux", i2c_mux_p, ARRAY_SIZE(i2c_mux_p),
		CLK_SET_RATE_PARENT, 0x42b0, 12, 1, 0, i2c_mux_table, },
	{ HI3519DV500_I2C7_MUX, "i2c7_mux", i2c_mux_p, ARRAY_SIZE(i2c_mux_p),
		CLK_SET_RATE_PARENT, 0x42b8, 12, 1, 0, i2c_mux_table, },
	{ HI3519DV500_PWM0_MUX, "pwm0_mux", pwm_mux_p, ARRAY_SIZE(pwm_mux_p),
		CLK_SET_RATE_PARENT, 0x4588, 12, 2, 0, pwm_mux_table, },
	{ HI3519DV500_PWM1_MUX, "pwm1_mux", pwm_mux_p, ARRAY_SIZE(pwm_mux_p),
		CLK_SET_RATE_PARENT, 0x4590, 12, 2, 0, pwm_mux_table, },
	{ HI3519DV500_PWM2_MUX, "pwm2_mux", pwm_mux_p, ARRAY_SIZE(pwm_mux_p),
		CLK_SET_RATE_PARENT, 0x4598, 12, 2, 0, pwm_mux_table, },
};

static const struct hisi_gate_clock hi3519dv500_gate_clks[] = {
	/* uart */
	{ HI3519DV500_UART0_CLK, "clk_uart0", "uart0_mux",
		CLK_SET_RATE_PARENT, 0x4180, 4, 0, },
	{ HI3519DV500_UART1_CLK, "clk_uart1", "uart1_mux",
		CLK_SET_RATE_PARENT, 0x4188, 4, 0, },
	{ HI3519DV500_UART2_CLK, "clk_uart2", "uart2_mux",
		CLK_SET_RATE_PARENT, 0x4190, 4, 0, },
	{ HI3519DV500_UART3_CLK, "clk_uart3", "uart3_mux",
		CLK_SET_RATE_PARENT, 0x4198, 4, 0, },
	{ HI3519DV500_UART4_CLK, "clk_uart4", "uart4_mux",
		CLK_SET_RATE_PARENT, 0x41a0, 4, 0, },
	{ HI3519DV500_UART5_CLK, "clk_uart5", "uart5_mux",
		CLK_SET_RATE_PARENT, 0x41a8, 4, 0, },
	/* i2c */
	{ HI3519DV500_I2C0_CLK, "clk_i2c0", "i2c0_mux",
		CLK_SET_RATE_PARENT, 0x4280, 4, 0, },
	{ HI3519DV500_I2C1_CLK, "clk_i2c1", "i2c1_mux",
		CLK_SET_RATE_PARENT, 0x4288, 4, 0, },
	{ HI3519DV500_I2C2_CLK, "clk_i2c2", "i2c2_mux",
		CLK_SET_RATE_PARENT, 0x4290, 4, 0, },
	{ HI3519DV500_I2C3_CLK, "clk_i2c3", "i2c3_mux",
		CLK_SET_RATE_PARENT, 0x4298, 4, 0, },
	{ HI3519DV500_I2C4_CLK, "clk_i2c4", "i2c4_mux",
		CLK_SET_RATE_PARENT, 0x42a0, 4, 0, },
	{ HI3519DV500_I2C5_CLK, "clk_i2c5", "i2c5_mux",
		CLK_SET_RATE_PARENT, 0x42a8, 4, 0, },
	{ HI3519DV500_I2C6_CLK, "clk_i2c6", "i2c6_mux",
		CLK_SET_RATE_PARENT, 0x42b0, 4, 0, },
	{ HI3519DV500_I2C7_CLK, "clk_i2c7", "i2c7_mux",
		CLK_SET_RATE_PARENT, 0x42b8, 4, 0, },
	/* spi */
	{ HI3519DV500_SPI0_CLK, "clk_spi0", "100m",
		CLK_SET_RATE_PARENT, 0x4480, 4, 0, },
	{ HI3519DV500_SPI1_CLK, "clk_spi1", "100m",
		CLK_SET_RATE_PARENT, 0x4488, 4, 0, },
	{ HI3519DV500_SPI2_CLK, "clk_spi2", "100m",
		CLK_SET_RATE_PARENT, 0x4490, 4, 0, },
	{ HI3519DV500_SPI3_CLK, "clk_spi3", "100m",
		CLK_SET_RATE_PARENT, 0x4498, 4, 0, },
	/* fmc */
	{ HI3519DV500_FMC_CLK, "clk_fmc", "fmc_mux",
		CLK_SET_RATE_PARENT, 0x3f40, 4, 0, },
	/* mmc */
	{ HI3519DV500_MMC0_CLK, "clk_mmc0", "mmc0_mux",
		CLK_SET_RATE_PARENT, 0x34c0, 0, 0, },
	{ HI3519DV500_MMC0_HCLK, "hclk_mmc0", NULL,
		CLK_SET_RATE_PARENT, 0x34c0, 1, 0, },
	{ HI3519DV500_MMC1_CLK, "clk_mmc1", "mmc1_mux",
		CLK_SET_RATE_PARENT, 0x35c0, 0, 0, },
	{ HI3519DV500_MMC1_HCLK, "hclk_mmc1", NULL,
		CLK_SET_RATE_PARENT, 0x35c0, 1, 0, },
	{ HI3519DV500_MMC2_CLK, "clk_mmc2", "mmc2_mux",
		CLK_SET_RATE_PARENT, 0x36c0, 0, 0, },
	{ HI3519DV500_MMC2_HCLK, "hclk_mmc2", NULL,
		CLK_SET_RATE_PARENT, 0x36c0, 1, 0, },
	/* ethernet mac */
	{ HI3519DV500_ETH_CLK, "clk_eth", NULL,
		CLK_SET_RATE_PARENT, 0x37c4, 4, 0, },
	{ HI3519DV500_ETH_MACIF_CLK, "clk_eth_macif", NULL,
		CLK_SET_RATE_PARENT, 0x37c0, 4, 0, },
	{ HI3519DV500_ETH1_CLK, "clk_eth1", NULL,
		CLK_SET_RATE_PARENT, 0x3804, 4, 0, },
	{ HI3519DV500_ETH1_MACIF_CLK, "clk_eth1_macif", NULL,
		CLK_SET_RATE_PARENT, 0x3800, 4, 0, },
	/* edmac */
	{ HI3519DV500_EDMAC_CLK, "clk_edmac", NULL,
		CLK_SET_RATE_PARENT, 0x2a80, 4, 0, },
	{ HI3519DV500_EDMAC_AXICLK, "axi_clk_edmac", NULL,
		CLK_SET_RATE_PARENT, 0x2a80, 5, 0, },
	/* lsadc */
	{ HI3519DV500_LSADC_CLK, "clk_lsadc", NULL,
		CLK_SET_RATE_PARENT, 0x46c0, 4, 0, },
	/* pwm */
	{ HI3519DV500_PWM0_CLK, "clk_pwm0", "pwm0_mux",
		CLK_SET_RATE_PARENT, 0x4588, 4, 0, },
	{ HI3519DV500_PWM1_CLK, "clk_pwm1", "pwm1_mux",
		CLK_SET_RATE_PARENT, 0x4590, 4, 0, },
	{ HI3519DV500_PWM2_CLK, "clk_pwm2", "pwm2_mux",
		CLK_SET_RATE_PARENT, 0x4598, 4, 0, },
};

struct hi3519dv500_crg_data {
	struct hisi_clock_data *clk_data;
	struct hisi_reset_controller *rstc;
};

/*
 * The USB clocks are composite CRG sequences: a default value is
 * written as a whole first, then a set of gate and reset bits is
 * managed across the prepare and enable stages, so that the PHY
 * drivers can program their analog trim words while the reset line
 * is still asserted.
 */
#define USB2_PHY_CRG_OFFSET		0x38c0
#define USB2_PHY_CRG_DEFAULT		0x107
#define USB2_PHY_APB_CKEN		BIT(6)
#define USB2_PHY_XTAL_CKEN		BIT(4)
#define USB2_PHY_APB_SREQ		BIT(2)
#define USB2_PHY_TREQ			BIT(1)
#define USB2_PHY_REQ			BIT(0)

#define USB30_CTRL_CRG_OFFSET		0x3940
#define USB30_CTRL_CRG_DEFAULT		0x10003
#define USB30_CTRL_FREECLK_SEL		BIT(16)
#define USB30_CTRL_PIPE_CKEN		BIT(12)
#define USB30_CTRL_UTMI_CKEN		BIT(8)
#define USB30_CTRL_SUSPEND_CKEN		BIT(6)
#define USB30_CTRL_REF_CKEN		BIT(5)
#define USB30_CTRL_BUS_CKEN		BIT(4)
#define USB30_CTRL_SRST_REQ		BIT(0)

#define USB3_PHY_CRG_OFFSET		0x3944
#define USB3_PHY_CRG_DEFAULT		0x3
#define USB3_PHY_REF_CKEN		BIT(4)
#define USB3_PHY_APB_SREQ		BIT(2)
#define USB3_PHY_SRST_REQ		BIT(1)
#define USB3_PHY_REQ			BIT(0)

struct hi3519dv500_usb_clk {
	struct clk_hw hw;
	void __iomem *base;
	const struct hi3519dv500_usb_clk_info *info;
};

struct hi3519dv500_usb_clk_info {
	u32 offset;
	u32 default_val;
	u32 prepare_set;
	u32 prepare_clr;
	u32 enable_set;
	u32 enable_clr;
	const char *name;
};

#define to_hi3519dv500_usb_clk(_hw) \
	container_of(_hw, struct hi3519dv500_usb_clk, hw)

static int hi3519dv500_usb_clk_prepare(struct clk_hw *hw)
{
	struct hi3519dv500_usb_clk *clk = to_hi3519dv500_usb_clk(hw);
	const struct hi3519dv500_usb_clk_info *info = clk->info;
	u32 val;

	writel(info->default_val, clk->base + info->offset);
	udelay(200); /* delay 200 us to wait the CRG stable */

	val = readl(clk->base + info->offset);
	val |= info->prepare_set;
	val &= ~info->prepare_clr;
	writel(val, clk->base + info->offset);

	return 0;
}

static void hi3519dv500_usb_clk_unprepare(struct clk_hw *hw)
{
	struct hi3519dv500_usb_clk *clk = to_hi3519dv500_usb_clk(hw);

	writel(clk->info->default_val, clk->base + clk->info->offset);
}

static int hi3519dv500_usb_clk_enable(struct clk_hw *hw)
{
	struct hi3519dv500_usb_clk *clk = to_hi3519dv500_usb_clk(hw);
	u32 val;

	val = readl(clk->base + clk->info->offset);
	val |= clk->info->enable_set;
	val &= ~clk->info->enable_clr;
	writel(val, clk->base + clk->info->offset);

	return 0;
}

static void hi3519dv500_usb_clk_disable(struct clk_hw *hw)
{
	struct hi3519dv500_usb_clk *clk = to_hi3519dv500_usb_clk(hw);
	u32 val;

	val = readl(clk->base + clk->info->offset);
	val |= clk->info->enable_clr;
	writel(val, clk->base + clk->info->offset);
}

static const struct clk_ops hi3519dv500_usb_clk_ops = {
	.prepare = hi3519dv500_usb_clk_prepare,
	.unprepare = hi3519dv500_usb_clk_unprepare,
	.enable = hi3519dv500_usb_clk_enable,
	.disable = hi3519dv500_usb_clk_disable,
};

/*
 * The USB2 PHY keeps the TREQ reset asserted across prepare, so the
 * trim words can be programmed before the reset is released by enable.
 * The USB30 controller sequence leaves the pinout and SuperSpeed
 * enable handling to the DWC3 glue driver.
 */
static const struct hi3519dv500_usb_clk_info hi3519dv500_usb_clks[] = {
	{
		.offset = USB2_PHY_CRG_OFFSET,
		.default_val = USB2_PHY_CRG_DEFAULT,
		.prepare_set = USB2_PHY_APB_CKEN | USB2_PHY_XTAL_CKEN,
		.prepare_clr = USB2_PHY_APB_SREQ | USB2_PHY_REQ,
		.enable_clr = USB2_PHY_TREQ,
		.name = "clk_usb2_phy0",
	},
	{
		.offset = USB30_CTRL_CRG_OFFSET,
		.default_val = USB30_CTRL_CRG_DEFAULT,
		.enable_set = USB30_CTRL_FREECLK_SEL | USB30_CTRL_PIPE_CKEN |
			      USB30_CTRL_UTMI_CKEN | USB30_CTRL_SUSPEND_CKEN |
			      USB30_CTRL_REF_CKEN | USB30_CTRL_BUS_CKEN,
		.enable_clr = USB30_CTRL_SRST_REQ,
		.name = "clk_usb30_ctrl0",
	},
	{
		.offset = USB3_PHY_CRG_OFFSET,
		.default_val = USB3_PHY_CRG_DEFAULT,
		.prepare_set = USB3_PHY_REF_CKEN | USB3_PHY_APB_SREQ,
		.enable_clr = USB3_PHY_SRST_REQ | USB3_PHY_REQ,
		.name = "clk_usb3_phy0",
	},
};

static const int hi3519dv500_usb_clk_ids[] = {
	HI3519DV500_USB2_PHY0_CLK,
	HI3519DV500_USB30_CTRL0_CLK,
	HI3519DV500_USB3_PHY0_CLK,
};

static int hi3519dv500_clk_register_usb(struct platform_device *pdev,
					struct hisi_clock_data *clk_data)
{
	struct clk_init_data init = {};
	int ret;
	int i;

	for (i = 0; i < ARRAY_SIZE(hi3519dv500_usb_clks); i++) {
		const struct hi3519dv500_usb_clk_info *info =
			&hi3519dv500_usb_clks[i];
		struct hi3519dv500_usb_clk *usb_clk;
		struct clk_hw *hw;

		usb_clk = devm_kzalloc(&pdev->dev, sizeof(*usb_clk),
				       GFP_KERNEL);
		if (!usb_clk)
			return -ENOMEM;

		usb_clk->base = clk_data->base;
		usb_clk->info = info;

		init.name = info->name;
		init.ops = &hi3519dv500_usb_clk_ops;
		init.flags = 0;
		init.parent_names = NULL;
		init.num_parents = 0;
		usb_clk->hw.init = &init;

		hw = &usb_clk->hw;
		ret = clk_hw_register(&pdev->dev, hw);
		if (ret)
			return ret;

		clk_data->clk_data.clks[hi3519dv500_usb_clk_ids[i]] =
			hw->clk;
	}

	return 0;
}

static void hi3519dv500_clk_unregister_usb(struct platform_device *pdev,
					   struct hisi_clock_data *clk_data)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(hi3519dv500_usb_clk_ids); i++) {
		struct clk *clk;

		clk = clk_data->clk_data.clks[hi3519dv500_usb_clk_ids[i]];
		if (!clk)
			continue;

		clk_unregister(clk);
		clk_data->clk_data.clks[hi3519dv500_usb_clk_ids[i]] = NULL;
	}
}

static struct hisi_clock_data *hi3519dv500_clk_register(struct platform_device *pdev)
{
	struct hisi_clock_data *clk_data;
	int ret;

	clk_data = hisi_clk_alloc(pdev, HI3519DV500_CRG_NR_CLKS);
	if (!clk_data)
		return ERR_PTR(-ENOMEM);

	ret = hisi_clk_register_fixed_rate(hi3519dv500_fixed_rate_clks,
			ARRAY_SIZE(hi3519dv500_fixed_rate_clks), clk_data);
	if (ret)
		return ERR_PTR(ret);

	ret = hisi_clk_register_mux(hi3519dv500_mux_clks,
			ARRAY_SIZE(hi3519dv500_mux_clks), clk_data);
	if (ret)
		goto unregister_fixed_rate;

	ret = hisi_clk_register_gate(hi3519dv500_gate_clks,
			ARRAY_SIZE(hi3519dv500_gate_clks), clk_data);
	if (ret)
		goto unregister_mux;

	ret = hi3519dv500_clk_register_usb(pdev, clk_data);
	if (ret)
		goto unregister_gate;

	ret = of_clk_add_provider(pdev->dev.of_node,
			of_clk_src_onecell_get, &clk_data->clk_data);
	if (ret)
		goto unregister_usb;

	return clk_data;

unregister_usb:
	hi3519dv500_clk_unregister_usb(pdev, clk_data);
unregister_gate:
	hisi_clk_unregister_gate(hi3519dv500_gate_clks,
			ARRAY_SIZE(hi3519dv500_gate_clks), clk_data);
unregister_mux:
	hisi_clk_unregister_mux(hi3519dv500_mux_clks,
			ARRAY_SIZE(hi3519dv500_mux_clks), clk_data);
unregister_fixed_rate:
	hisi_clk_unregister_fixed_rate(hi3519dv500_fixed_rate_clks,
			ARRAY_SIZE(hi3519dv500_fixed_rate_clks), clk_data);
	return ERR_PTR(ret);
}

static void hi3519dv500_clk_unregister(struct platform_device *pdev)
{
	struct hi3519dv500_crg_data *crg = platform_get_drvdata(pdev);

	of_clk_del_provider(pdev->dev.of_node);

	hi3519dv500_clk_unregister_usb(pdev, crg->clk_data);
	hisi_clk_unregister_gate(hi3519dv500_gate_clks,
			ARRAY_SIZE(hi3519dv500_gate_clks), crg->clk_data);
	hisi_clk_unregister_mux(hi3519dv500_mux_clks,
			ARRAY_SIZE(hi3519dv500_mux_clks), crg->clk_data);
	hisi_clk_unregister_fixed_rate(hi3519dv500_fixed_rate_clks,
			ARRAY_SIZE(hi3519dv500_fixed_rate_clks), crg->clk_data);
}

static int hi3519dv500_clk_probe(struct platform_device *pdev)
{
	struct hi3519dv500_crg_data *crg;

	crg = devm_kmalloc(&pdev->dev, sizeof(*crg), GFP_KERNEL);
	if (!crg)
		return -ENOMEM;

	crg->rstc = hisi_reset_init(pdev);
	if (!crg->rstc)
		return -ENOMEM;

	crg->clk_data = hi3519dv500_clk_register(pdev);
	if (IS_ERR(crg->clk_data)) {
		hisi_reset_exit(crg->rstc);
		return PTR_ERR(crg->clk_data);
	}

	platform_set_drvdata(pdev, crg);
	return 0;
}

static void hi3519dv500_clk_remove(struct platform_device *pdev)
{
	struct hi3519dv500_crg_data *crg = platform_get_drvdata(pdev);

	hisi_reset_exit(crg->rstc);
	hi3519dv500_clk_unregister(pdev);
}

static const struct of_device_id hi3519dv500_clk_match_table[] = {
	{ .compatible = "hisilicon,hi3519dv500-clock" },
	{ }
};
MODULE_DEVICE_TABLE(of, hi3519dv500_clk_match_table);

static struct platform_driver hi3519dv500_clk_driver = {
	.probe		= hi3519dv500_clk_probe,
	.remove		= hi3519dv500_clk_remove,
	.driver		= {
		.name	= "hi3519dv500-clock",
		.of_match_table = hi3519dv500_clk_match_table,
	},
};

static int __init hi3519dv500_clk_init(void)
{
	return platform_driver_register(&hi3519dv500_clk_driver);
}
core_initcall(hi3519dv500_clk_init);

static void __exit hi3519dv500_clk_exit(void)
{
	platform_driver_unregister(&hi3519dv500_clk_driver);
}
module_exit(hi3519dv500_clk_exit);

MODULE_DESCRIPTION("HiSilicon Hi3519DV500 CRG Driver");
MODULE_LICENSE("GPL");
