// SPDX-License-Identifier: GPL-2.0-only
/*
 * HiSilicon Hi3519DV500 USB PHY driver
 *
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023.
 * Copyright (c) 2026 Jun Yan <jerrysteve1101@gmail.com>
 *
 * Ported from the HiSilicon vendor SDK to the mainline kernel.
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/property.h>

/*
 * USB2 PHY analog registers.  The trim words are written while the
 * TREQ reset line of the PHY clock is still asserted, so the analog
 * parameters are stable before the reset is released by clk_enable.
 */
#define U2PHY_ANA_CFG0_OFFSET		0x0000
#define U2PHY_ANA_CFG0_DFT_VALUE	0x0a33cc2b
#define U2PHY_ANA_CFG2_OFFSET		0x0008
#define U2PHY_ANA_CFG2_DFT_VALUE	0x00260f0f
#define U2PHY_ANA_CFG5_OFFSET		0x0014
#define PLL_EN_MASK			GENMASK(1, 0)

#define PHY_RT_TRIM_MASK		0x001f
#define RG_RT_TRIM_MASK			0x1f00
#define RG_RT_TRIM_OFFSET		8
#define RT_TRIM_MIN_VALUE		0x0009
#define RT_TRIM_MAX_VALUE		0x001d

/* USB3 combo PHY test and analog registers */
#define USB3PHY_TEST_REG_8		0x20
#define CDR_CPF_TRIM_MASK		0xf
#define CDR_CPF_TRIM_VAL		0xf

#define USB3PHY_TEST_REG_15		0x3c
#define TX_PLL_TRIM_USB3_SATA_MASK	0x1f
#define TX_PLL_TRIM_USB3_SATA_VAL	0x3
#define TX_PLL_TRIM_PCIE_MASK		(0x1f << 5)
#define TX_PLL_TRIM_PCIE_VAL		(0x3 << 5)

#define USB3PHY_TEST_REG_16		0x40
#define PI_CURRENT_TRIM_MASK		0x3
#define PI_CURRENT_TRIM_VAL		0x2

#define USB3PHY_TEST_REG_18		0x48
#define RX_TERM_USB3_MASK		0xf
#define RX_TERM_USB3_VAL		0xa
#define RX_TERM_MASK			(0xf << 4)
#define RX_TERM_VAL			(0x7 << 4)

#define USB3PHY_TEST_REG_22		0x58
#define REDUCE_RX_DET_TH_MASK		BIT(1)
#define REDUCE_RX_DET_TH_VAL		BIT(1)
#define INTERPOLATOR_JUMP_LATER_MASK	BIT(2)
#define INTERPOLATOR_JUMP_LATER_VAL	BIT(2)

#define USB3PHY_ANALOG_REG0		0x80
#define CDR_CTRL_MASK			(0x7f << 16)
#define CDR_CTRL_VAL			(0x48 << 16)

#define USB3PHY_ANALOG_REG1		0x84
#define CSEL_MASK			(0x3 << 10)
#define CSEL_VAL			(0x1 << 10)
#define RESEL11_MASK			(0x3 << 12)
#define RESEL11_VAL			(0x1 << 12)
#define RESEL12_MASK			(0x3 << 14)
#define RESEL12_VAL			(0x0 << 14)

#define USB3PHY_AUTO_EQ_REG0		0xb8
#define CFG_AUTO_DESKEW_EN_MASK		BIT(20)
#define CFG_AUTO_EQ_EN_MASK		BIT(21)

#define USB3PHY_AUTO_EQ_REG5		0xcc
#define CFG_EQ_OW_MASK			0x1f
#define CFG_EQ_OW_VAL			0xc
#define CFG_EQ_OW_EN_MASK		BIT(5)
#define CFG_EQ_OW_EN_VAL		BIT(5)
#define CFG_DESKEW_OW_MASK		(0x1f << 6)
#define CFG_DESKEW_OW_VAL		(0x10 << 6)
#define CFG_DESKEW_OW_EN_MASK		BIT(11)
#define CFG_DESKEW_OW_EN_VAL		BIT(11)

struct hi3519dv500_usb_phy {
	struct device *dev;
	void __iomem *base;
	void __iomem *trim_otp;
	struct clk *clk;
	u32 u2phy_trim[2];
};

static void hi3519dv500_usb2_phy_config(struct hi3519dv500_usb_phy *priv)
{
	u32 rt_trim;
	u32 val;

	val = readl(priv->base + U2PHY_ANA_CFG5_OFFSET);
	val |= PLL_EN_MASK;
	writel(val, priv->base + U2PHY_ANA_CFG5_OFFSET);

	/*
	 * The ANA_CFG2 eye diagram configuration must be written before
	 * the trim adjustment, since the eye diagram write replaces the
	 * whole register while the trim write only touches a few bits.
	 */
	writel(priv->u2phy_trim[0], priv->base + U2PHY_ANA_CFG0_OFFSET);
	writel(priv->u2phy_trim[1], priv->base + U2PHY_ANA_CFG2_OFFSET);

	if (!priv->trim_otp)
		return;

	/* adjust the trim with the value from the OTP, when valid */
	rt_trim = readl(priv->trim_otp) & PHY_RT_TRIM_MASK;
	if (rt_trim >= RT_TRIM_MIN_VALUE && rt_trim <= RT_TRIM_MAX_VALUE) {
		val = readl(priv->base + U2PHY_ANA_CFG2_OFFSET);
		val &= ~RG_RT_TRIM_MASK;
		val |= rt_trim << RG_RT_TRIM_OFFSET;
		writel(val, priv->base + U2PHY_ANA_CFG2_OFFSET);
	}
}

static int hi3519dv500_usb2_phy_power_on(struct phy *phy)
{
	struct hi3519dv500_usb_phy *priv = phy_get_drvdata(phy);
	int ret;

	ret = clk_prepare(priv->clk);
	if (ret)
		return ret;

	/*
	 * The clock keeps the PHY reset asserted across prepare, so the
	 * analog trim words can be programmed before the reset is
	 * released by the enable stage below.
	 */
	hi3519dv500_usb2_phy_config(priv);

	ret = clk_enable(priv->clk);
	if (ret)
		clk_unprepare(priv->clk);

	return ret;
}

static int hi3519dv500_usb2_phy_power_off(struct phy *phy)
{
	struct hi3519dv500_usb_phy *priv = phy_get_drvdata(phy);

	clk_disable_unprepare(priv->clk);

	return 0;
}

static const struct phy_ops hi3519dv500_usb2_phy_ops = {
	.power_on = hi3519dv500_usb2_phy_power_on,
	.power_off = hi3519dv500_usb2_phy_power_off,
};

static void hi3519dv500_usb3_phy_init(struct hi3519dv500_usb_phy *priv)
{
	u32 val;

	val = readl(priv->base + USB3PHY_TEST_REG_8);
	val &= ~CDR_CPF_TRIM_MASK;
	val |= CDR_CPF_TRIM_VAL;
	writel(val, priv->base + USB3PHY_TEST_REG_8);

	val = readl(priv->base + USB3PHY_TEST_REG_15);
	val &= ~(TX_PLL_TRIM_USB3_SATA_MASK | TX_PLL_TRIM_PCIE_MASK);
	val |= TX_PLL_TRIM_USB3_SATA_VAL | TX_PLL_TRIM_PCIE_VAL;
	writel(val, priv->base + USB3PHY_TEST_REG_15);

	val = readl(priv->base + USB3PHY_TEST_REG_16);
	val &= ~PI_CURRENT_TRIM_MASK;
	val |= PI_CURRENT_TRIM_VAL;
	writel(val, priv->base + USB3PHY_TEST_REG_16);

	val = readl(priv->base + USB3PHY_TEST_REG_18);
	val &= ~(RX_TERM_USB3_MASK | RX_TERM_MASK);
	val |= RX_TERM_USB3_VAL | RX_TERM_VAL;
	writel(val, priv->base + USB3PHY_TEST_REG_18);

	val = readl(priv->base + USB3PHY_TEST_REG_22);
	val &= ~(REDUCE_RX_DET_TH_MASK | INTERPOLATOR_JUMP_LATER_MASK);
	val |= REDUCE_RX_DET_TH_VAL | INTERPOLATOR_JUMP_LATER_VAL;
	writel(val, priv->base + USB3PHY_TEST_REG_22);

	val = readl(priv->base + USB3PHY_ANALOG_REG0);
	val &= ~CDR_CTRL_MASK;
	val |= CDR_CTRL_VAL;
	writel(val, priv->base + USB3PHY_ANALOG_REG0);
}

static void hi3519dv500_usb3_phy_eye(struct hi3519dv500_usb_phy *priv)
{
	u32 val;

	val = readl(priv->base + USB3PHY_ANALOG_REG1);
	val &= ~(CSEL_MASK | RESEL11_MASK | RESEL12_MASK);
	val |= CSEL_VAL | RESEL11_VAL | RESEL12_VAL;
	writel(val, priv->base + USB3PHY_ANALOG_REG1);

	val = readl(priv->base + USB3PHY_AUTO_EQ_REG0);
	val &= ~(CFG_AUTO_DESKEW_EN_MASK | CFG_AUTO_EQ_EN_MASK);
	writel(val, priv->base + USB3PHY_AUTO_EQ_REG0);

	val = readl(priv->base + USB3PHY_AUTO_EQ_REG5);
	val &= ~(CFG_EQ_OW_MASK | CFG_EQ_OW_EN_MASK |
		 CFG_DESKEW_OW_MASK | CFG_DESKEW_OW_EN_MASK);
	val |= CFG_EQ_OW_VAL | CFG_EQ_OW_EN_VAL |
	       CFG_DESKEW_OW_VAL | CFG_DESKEW_OW_EN_VAL;
	writel(val, priv->base + USB3PHY_AUTO_EQ_REG5);
}

static int hi3519dv500_usb3_phy_power_on(struct phy *phy)
{
	struct hi3519dv500_usb_phy *priv = phy_get_drvdata(phy);
	int ret;

	ret = clk_prepare_enable(priv->clk);
	if (ret)
		return ret;

	hi3519dv500_usb3_phy_init(priv);
	hi3519dv500_usb3_phy_eye(priv);

	return 0;
}

static int hi3519dv500_usb3_phy_power_off(struct phy *phy)
{
	struct hi3519dv500_usb_phy *priv = phy_get_drvdata(phy);

	clk_disable_unprepare(priv->clk);

	return 0;
}

static const struct phy_ops hi3519dv500_usb3_phy_ops = {
	.power_on = hi3519dv500_usb3_phy_power_on,
	.power_off = hi3519dv500_usb3_phy_power_off,
};

static int hi3519dv500_usb_phy_probe(struct platform_device *pdev)
{
	const struct phy_ops *phy_ops;
	struct device *dev = &pdev->dev;
	struct hi3519dv500_usb_phy *priv;
	struct phy_provider *provider;
	struct phy *phy;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = dev;

	priv->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	priv->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(priv->clk))
		return PTR_ERR(priv->clk);

	phy_ops = device_get_match_data(dev);
	if (!phy_ops)
		return -EINVAL;

	if (phy_ops == &hi3519dv500_usb2_phy_ops) {
		priv->trim_otp = devm_platform_ioremap_resource(pdev, 1);
		if (IS_ERR(priv->trim_otp))
			priv->trim_otp = NULL;

		priv->u2phy_trim[0] = U2PHY_ANA_CFG0_DFT_VALUE;
		priv->u2phy_trim[1] = U2PHY_ANA_CFG2_DFT_VALUE;
		of_property_read_u32_array(dev->of_node,
					   "hisilicon,u2phy-trim",
					   priv->u2phy_trim, 2);
	}

	phy = devm_phy_create(dev, NULL, phy_ops);
	if (IS_ERR(phy))
		return PTR_ERR(phy);

	phy_set_drvdata(phy, priv);

	provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);
	if (IS_ERR(provider))
		return PTR_ERR(provider);

	platform_set_drvdata(pdev, priv);

	return 0;
}

static const struct of_device_id hi3519dv500_usb_phy_of_match[] = {
	{
		.compatible = "hisilicon,hi3519dv500-usb2-phy",
		.data = &hi3519dv500_usb2_phy_ops,
	}, {
		.compatible = "hisilicon,hi3519dv500-usb3-phy",
		.data = &hi3519dv500_usb3_phy_ops,
	},
	{ }
};
MODULE_DEVICE_TABLE(of, hi3519dv500_usb_phy_of_match);

static struct platform_driver hi3519dv500_usb_phy_driver = {
	.probe = hi3519dv500_usb_phy_probe,
	.driver = {
		.name = "phy-hi3519dv500-usb",
		.of_match_table = hi3519dv500_usb_phy_of_match,
	},
};

module_platform_driver(hi3519dv500_usb_phy_driver);

MODULE_AUTHOR("Jun Yan <jerrysteve1101@gmail.com>");
MODULE_DESCRIPTION("HiSilicon Hi3519DV500 USB PHY driver");
MODULE_LICENSE("GPL");
