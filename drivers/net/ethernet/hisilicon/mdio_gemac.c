// SPDX-License-Identifier: GPL-2.0-only
/*
 * HiSilicon GEMAC MDIO bus controller driver
 *
 * Copyright (c) 2012-2021 Hisilicon Technologies Co., Ltd.
 * Copyright (c) 2026 Jun Yan <jerrysteve1101@gmail.com>
 *
 * Ported from the HiSilicon vendor SDK to the mainline kernel.
 */

#include <linux/clk.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/of_mdio.h>
#include <linux/phy.h>
#include <linux/platform_device.h>
#include <linux/reset.h>

#define MDIO_SINGLE_CMD		0x00
#define MDIO_SINGLE_DATA	0x04
#define MDIO_RDATA_STATUS	0x10
#define BIT_PHY_ADDR_OFFSET	8
#define MDIO_WRITE		BIT(16)
#define MDIO_READ		BIT(17)
#define MDIO_START		BIT(20)
#define MDIO_START_READ		(MDIO_START | MDIO_READ)
#define MDIO_START_WRITE	(MDIO_START | MDIO_WRITE)

struct gemac_mdio_data {
	struct reset_control *phy_rst;
	void __iomem *membase;
};

static int gemac_mdio_wait_ready(struct gemac_mdio_data *data)
{
	u32 val;

	return readl_poll_timeout(data->membase + MDIO_SINGLE_CMD,
				  val, !(val & MDIO_START), 20, 10000);
}

static int gemac_mdio_read(struct mii_bus *bus, int mii_id, int regnum)
{
	struct gemac_mdio_data *data = bus->priv;
	int ret;

	ret = gemac_mdio_wait_ready(data);
	if (ret)
		return ret;

	writel(MDIO_START_READ | (mii_id << BIT_PHY_ADDR_OFFSET) | regnum,
	       data->membase + MDIO_SINGLE_CMD);

	ret = gemac_mdio_wait_ready(data);
	if (ret)
		return ret;

	/*
	 * If the read data is invalid, return 0 instead of -EAGAIN.
	 * This makes the MDIO bus scan robust when probing PHY addresses
	 * with no device attached.
	 */
	if (readl(data->membase + MDIO_RDATA_STATUS))
		return 0;

	return readl(data->membase + MDIO_SINGLE_DATA) >> 16;
}

static int gemac_mdio_write(struct mii_bus *bus, int mii_id, int regnum,
			    u16 value)
{
	struct gemac_mdio_data *data = bus->priv;
	int ret;

	ret = gemac_mdio_wait_ready(data);
	if (ret)
		return ret;

	writel(value, data->membase + MDIO_SINGLE_DATA);
	writel(MDIO_START_WRITE | (mii_id << BIT_PHY_ADDR_OFFSET) | regnum,
	       data->membase + MDIO_SINGLE_CMD);

	return gemac_mdio_wait_ready(data);
}

static void gemac_external_phy_reset(struct gemac_mdio_data *data)
{
	if (!data->phy_rst)
		return;

	/* write 0 to cancel reset */
	reset_control_deassert(data->phy_rst);
	msleep(50);

	/* write 1 to reset phy, write 0 to cancel reset */
	reset_control_assert(data->phy_rst);

	/*
	 * Delay some time to ensure reset ok,
	 * this depends on the PHY hardware feature.
	 */
	msleep(20);

	/* write 0 to cancel reset */
	reset_control_deassert(data->phy_rst);
	/* delay some time to ensure later MDIO access */
	msleep(80);
}

static int gemac_mdio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct gemac_mdio_data *data;
	struct mii_bus *bus;
	struct clk *clk;
	int ret;

	bus = devm_mdiobus_alloc_size(dev, sizeof(*data));
	if (!bus)
		return -ENOMEM;

	bus->name = "gemac_mii_bus";
	bus->read = &gemac_mdio_read;
	bus->write = &gemac_mdio_write;
	snprintf(bus->id, MII_BUS_ID_SIZE, "%s", pdev->name);
	bus->parent = dev;

	data = bus->priv;
	data->membase = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(data->membase))
		return PTR_ERR(data->membase);

	clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(clk))
		return PTR_ERR(clk);

	data->phy_rst = devm_reset_control_get_optional_exclusive(dev,
								  "phy_reset");
	if (IS_ERR(data->phy_rst))
		return PTR_ERR(data->phy_rst);

	gemac_external_phy_reset(data);

	ret = of_mdiobus_register(bus, dev->of_node);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, bus);

	return 0;
}

static void gemac_mdio_remove(struct platform_device *pdev)
{
	struct mii_bus *bus = platform_get_drvdata(pdev);

	mdiobus_unregister(bus);
}

static const struct of_device_id gemac_mdio_dt_ids[] = {
	{ .compatible = "hisilicon,gemac-mdio" },
	{ }
};
MODULE_DEVICE_TABLE(of, gemac_mdio_dt_ids);

static struct platform_driver gemac_mdio_driver = {
	.probe = gemac_mdio_probe,
	.remove = gemac_mdio_remove,
	.driver = {
		.name = "hisi-gemac-mdio",
		.of_match_table = gemac_mdio_dt_ids,
	},
};

module_platform_driver(gemac_mdio_driver);

MODULE_AUTHOR("Jun Yan <jerrysteve1101@gmail.com>");
MODULE_DESCRIPTION("HiSilicon GEMAC MDIO bus controller driver");
MODULE_LICENSE("GPL");
