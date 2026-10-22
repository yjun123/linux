// SPDX-License-Identifier: GPL-2.0
/*
 * BQ257XX Battery Charger Driver
 * Copyright (C) 2025 Chris Morgan <macromorgan@hotmail.com>
 */

#include <linux/bitfield.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/mfd/bq257xx.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>

struct bq257xx_reg_data {
	struct regulator_dev *bq257xx_reg;
	struct gpio_desc *otg_en_gpio;
	struct regulator_desc desc;
};

static int bq25703_vbus_get_cur_limit(struct regulator_dev *rdev)
{
	struct regmap *regmap = rdev_get_regmap(rdev);
	int ret;
	unsigned int reg;

	ret = regmap_read(regmap, BQ25703_OTG_CURRENT, &reg);
	if (ret)
		return ret;
	return FIELD_GET(BQ25703_OTG_CUR_MASK, reg) * BQ25703_OTG_CUR_STEP_UA;
}

/*
 * Check if the minimum current and maximum current requested are
 * sane values, then set the register accordingly.
 */
static int bq25703_vbus_set_cur_limit(struct regulator_dev *rdev,
				      int min_uA, int max_uA)
{
	struct regmap *regmap = rdev_get_regmap(rdev);
	unsigned int reg;

	if ((min_uA > BQ25703_OTG_CUR_MAX_UA) || (max_uA < 0))
		return -EINVAL;

	reg = (max_uA / BQ25703_OTG_CUR_STEP_UA);

	/* Catch rounding errors since our step is 50000uA. */
	if ((reg * BQ25703_OTG_CUR_STEP_UA) < min_uA)
		return -EINVAL;

	return regmap_write(regmap, BQ25703_OTG_CURRENT,
			    FIELD_PREP(BQ25703_OTG_CUR_MASK, reg));
}

static int bq25703_vbus_enable(struct regulator_dev *rdev)
{
	struct bq257xx_reg_data *pdata = rdev_get_drvdata(rdev);

	if (pdata->otg_en_gpio)
		gpiod_set_value_cansleep(pdata->otg_en_gpio, 1);
	return regulator_enable_regmap(rdev);
}

static int bq25703_vbus_disable(struct regulator_dev *rdev)
{
	struct bq257xx_reg_data *pdata = rdev_get_drvdata(rdev);

	if (pdata->otg_en_gpio)
		gpiod_set_value_cansleep(pdata->otg_en_gpio, 0);
	return regulator_disable_regmap(rdev);
}

static const struct regulator_ops bq25703_vbus_ops = {
	.enable = bq25703_vbus_enable,
	.disable = bq25703_vbus_disable,
	.is_enabled = regulator_is_enabled_regmap,
	.list_voltage = regulator_list_voltage_linear,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
	.get_current_limit = bq25703_vbus_get_cur_limit,
	.set_current_limit = bq25703_vbus_set_cur_limit,
};

static const struct regulator_desc bq25703_vbus_desc = {
	.name = "vbus",
	.of_match = of_match_ptr("vbus"),
	.regulators_node = of_match_ptr("regulators"),
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
	.ops = &bq25703_vbus_ops,
	.min_uV = BQ25703_OTG_VOLT_MIN_UV,
	.uV_step = BQ25703_OTG_VOLT_STEP_UV,
	.n_voltages = BQ25703_OTG_VOLT_NUM_VOLT,
	.enable_mask = BQ25703_EN_OTG_MASK,
	.enable_reg = BQ25703_CHARGE_OPTION_3,
	.enable_val = BQ25703_EN_OTG_MASK,
	.disable_val = 0,
	.vsel_reg = BQ25703_OTG_VOLT,
	.vsel_mask = BQ25703_OTG_VOLT_MASK,
};

/* The SC8886S OTG voltage DAC uses a 12-bit field at 8 mV per LSB in bits
 * [13:2], plus a DC offset selected by the OTG_RANGE_LOW bit (ChargeOption3
 * [2]). With OTG_RANGE_LOW set the offset is 0 V and the output ranges from
 * 3.00 V to 22.72 V; with it cleared the offset is 1.28 V and the output
 * ranges from 4.28 V to 24.00 V. Voltages up to 22.72 V use the low range,
 * higher voltages use the high range.
 */
static int sc8886s_vbus_get_voltage_sel(struct regulator_dev *rdev)
{
	struct regmap *regmap = rdev_get_regmap(rdev);
	unsigned int field, range_low;
	int ret;

	ret = regmap_read(regmap, BQ25703_OTG_VOLT, &field);
	if (ret)
		return ret;
	field = FIELD_GET(SC8886S_OTG_VOLT_MASK, field);

	ret = regmap_read(regmap, BQ25703_CHARGE_OPTION_3, &range_low);
	if (ret)
		return ret;
	range_low = !!(range_low & SC8886S_OTG_RANGE_LOW);

	/* sel = (V - min) / step; low-range field = V / step, high-range
	 * field = (V - offset) / step.
	 */
	if (range_low)
		return field - SC8886S_OTG_VOLT_MIN_UV / SC8886S_OTG_VOLT_STEP_UV;
	return field - SC8886S_OTG_VOLT_MIN_UV / SC8886S_OTG_VOLT_STEP_UV
		    + SC8886S_OTG_VOLT_OFFSET_UV / SC8886S_OTG_VOLT_STEP_UV;
}

static int sc8886s_vbus_set_voltage_sel(struct regulator_dev *rdev, unsigned int sel)
{
	struct regmap *regmap = rdev_get_regmap(rdev);
	unsigned int vout_mv, range_low, field;
	int ret;

	vout_mv = SC8886S_OTG_VOLT_MIN_UV / 1000 +
		  sel * (SC8886S_OTG_VOLT_STEP_UV / 1000);

	/* Up to 22.72 V use the low range (0 V offset), otherwise the high
	 * range (1.28 V offset).
	 */
	range_low = vout_mv <= SC8886S_OTG_VOLT_LOW_MAX_UV / 1000;

	ret = regmap_update_bits(regmap, BQ25703_CHARGE_OPTION_3,
				 SC8886S_OTG_RANGE_LOW,
				 range_low ? SC8886S_OTG_RANGE_LOW : 0);
	if (ret)
		return ret;

	if (range_low)
		field = vout_mv / (SC8886S_OTG_VOLT_STEP_UV / 1000);
	else
		field = (vout_mv - SC8886S_OTG_VOLT_OFFSET_UV / 1000) /
			(SC8886S_OTG_VOLT_STEP_UV / 1000);

	return regmap_update_bits(regmap, BQ25703_OTG_VOLT,
				  SC8886S_OTG_VOLT_MASK,
				  FIELD_PREP(SC8886S_OTG_VOLT_MASK, field));
}

static const struct regulator_ops sc8886s_vbus_ops = {
	.enable = bq25703_vbus_enable,
	.disable = bq25703_vbus_disable,
	.is_enabled = regulator_is_enabled_regmap,
	.list_voltage = regulator_list_voltage_linear,
	.get_voltage_sel = sc8886s_vbus_get_voltage_sel,
	.set_voltage_sel = sc8886s_vbus_set_voltage_sel,
	.get_current_limit = bq25703_vbus_get_cur_limit,
	.set_current_limit = bq25703_vbus_set_cur_limit,
};

static const struct regulator_desc sc8886s_vbus_desc = {
	.name = "vbus",
	.of_match = of_match_ptr("vbus"),
	.regulators_node = of_match_ptr("regulators"),
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
	.ops = &sc8886s_vbus_ops,
	.min_uV = SC8886S_OTG_VOLT_MIN_UV,
	.uV_step = SC8886S_OTG_VOLT_STEP_UV,
	.n_voltages = SC8886S_OTG_VOLT_NUM_VOLT,
	.enable_mask = BQ25703_EN_OTG_MASK,
	.enable_reg = BQ25703_CHARGE_OPTION_3,
	.enable_val = BQ25703_EN_OTG_MASK,
	.disable_val = 0,
	.vsel_reg = BQ25703_OTG_VOLT,
	.vsel_mask = SC8886S_OTG_VOLT_MASK,
};

/* Get optional GPIO for OTG regulator enable. */
static void bq257xx_reg_dt_parse_gpio(struct platform_device *pdev)
{
	struct device_node *child, *subchild;
	struct bq257xx_reg_data *pdata = platform_get_drvdata(pdev);

	child = of_get_child_by_name(pdev->dev.of_node,
				     pdata->desc.regulators_node);
	if (!child)
		return;

	subchild = of_get_child_by_name(child, pdata->desc.of_match);
	of_node_put(child);
	if (!subchild)
		return;

	pdata->otg_en_gpio = devm_fwnode_gpiod_get_index(&pdev->dev,
							 of_fwnode_handle(subchild),
							 "enable", 0,
							 GPIOD_OUT_LOW,
							 pdata->desc.of_match);

	of_node_put(subchild);

	if (IS_ERR(pdata->otg_en_gpio)) {
		if (PTR_ERR(pdata->otg_en_gpio) == -ENOENT) {
			/* No GPIO, will only use register writes for OTG */
			pdata->otg_en_gpio = NULL;
			return;
		}

		dev_err(&pdev->dev, "Error getting enable gpio: %ld\n",
			PTR_ERR(pdata->otg_en_gpio));
		return;
	}
}

static int bq257xx_regulator_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct bq257xx_reg_data *pdata;
	struct regulator_config cfg = {};
	struct bq257xx_device *bq = dev_get_drvdata(pdev->dev.parent);

	device_set_of_node_from_dev(&pdev->dev, pdev->dev.parent);

	pdata = devm_kzalloc(&pdev->dev, sizeof(struct bq257xx_reg_data), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	if (bq->type == SC8886S)
		pdata->desc = sc8886s_vbus_desc;
	else
		pdata->desc = bq25703_vbus_desc;

	platform_set_drvdata(pdev, pdata);
	bq257xx_reg_dt_parse_gpio(pdev);

	cfg.dev = &pdev->dev;
	cfg.driver_data = pdata;
	cfg.regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!cfg.regmap)
		return -ENODEV;

	pdata->bq257xx_reg = devm_regulator_register(dev, &pdata->desc, &cfg);
	if (IS_ERR(pdata->bq257xx_reg)) {
		return dev_err_probe(&pdev->dev, PTR_ERR(pdata->bq257xx_reg),
				     "error registering bq257xx regulator");
	}

	return 0;
}

static const struct platform_device_id bq257xx_reg_id[] = {
	{ "bq257xx-regulator", 0 },
	{}
};
MODULE_DEVICE_TABLE(platform, bq257xx_reg_id);

static struct platform_driver bq257xx_reg_driver = {
	.driver = {
		.name = "bq257xx-regulator",
	},
	.probe = bq257xx_regulator_probe,
	.id_table = bq257xx_reg_id,
};

module_platform_driver(bq257xx_reg_driver);

MODULE_DESCRIPTION("bq257xx regulator driver");
MODULE_AUTHOR("Chris Morgan <macromorgan@hotmail.com>");
MODULE_LICENSE("GPL");
