// SPDX-License-Identifier: GPL-2.0
/*
 * BQ257XX Fuel Gauge Driver
 * Copyright (C) 2026 Jun Yan <jerrysteve1101@gmail.com>
 */

#include <linux/bitfield.h>
#include <linux/mfd/bq257xx.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/regmap.h>

#include "adc-battery-helper.h"

struct bq257xx_fg {
	/* Must be the first member, see adc-battery-helper.h */
	struct adc_battery_helper helper;
	struct regmap *regmap;
	struct power_supply *psy;
};

static int bq257xx_fg_get_voltage_and_current_now(struct power_supply *psy,
						  int *volt_uv, int *curr_ua)
{
	struct bq257xx_fg *fg = power_supply_get_drvdata(psy);
	unsigned int reg, status;
	int ret;

	ret = regmap_read(fg->regmap, BQ25703_ADCVSYSVBAT, &reg);
	if (ret)
		return ret;

	*volt_uv = FIELD_GET(BQ25703_ADCVBAT_MASK, reg) *
		   BQ25703_ADCVSYSVBAT_STEP + BQ25703_ADCVSYSVBAT_OFFSET_UV;

	ret = regmap_read(fg->regmap, BQ25703_CHARGER_STATUS, &status);
	if (ret)
		return ret;

	ret = regmap_read(fg->regmap, BQ25703_ADCIBAT_CHG, &reg);
	if (ret)
		return ret;

	/*
	 * Report the battery current with the charge direction sign: positive
	 * while charging (adapter present) and negative while discharging, as
	 * expected by the adc-battery-helper.
	 */
	if (status & BQ25703_STS_AC_STAT)
		*curr_ua = FIELD_GET(BQ25703_ADCIBAT_CHG_MASK, reg) *
			   BQ25703_ADCIBAT_CHG_STEP_UA;
	else
		*curr_ua = -(FIELD_GET(BQ25703_ADCIBAT_DISCHG_MASK, reg) *
			     BQ25703_ADCIBAT_DIS_STEP_UA);

	return 0;
}

static const struct power_supply_desc bq257xx_fg_desc = {
	.name = "bq257xx-fgu",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.get_property = adc_battery_helper_get_property,
	.external_power_changed = adc_battery_helper_external_power_changed,
	.properties = adc_battery_helper_properties,
	.num_properties = ADC_HELPER_NUM_PROPERTIES,
};

static int bq257xx_fg_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct bq257xx_fg *fg;
	struct power_supply_config cfg = {};

	device_set_of_node_from_dev(dev, dev->parent);

	fg = devm_kzalloc(dev, sizeof(*fg), GFP_KERNEL);
	if (!fg)
		return -ENOMEM;

	fg->regmap = dev_get_regmap(dev->parent, NULL);
	if (!fg->regmap)
		return dev_err_probe(dev, -ENODEV, "Failed to get regmap\n");

	platform_set_drvdata(pdev, fg);

	cfg.drv_data = fg;
	cfg.fwnode = dev_fwnode(dev);
	fg->psy = devm_power_supply_register(dev, &bq257xx_fg_desc, &cfg);
	if (IS_ERR(fg->psy))
		return dev_err_probe(dev, PTR_ERR(fg->psy),
				     "Failed to register power supply\n");

	return adc_battery_helper_init(&fg->helper, fg->psy,
				       bq257xx_fg_get_voltage_and_current_now,
				       NULL);
}

static struct platform_driver bq257xx_fg_driver = {
	.driver = {
		.name = "bq257xx-fgu",
	},
	.probe = bq257xx_fg_probe,
};
module_platform_driver(bq257xx_fg_driver);

MODULE_DESCRIPTION("bq257xx fuel gauge driver");
MODULE_AUTHOR("Jun Yan <jerrysteve1101@gmail.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:bq257xx-fgu");
