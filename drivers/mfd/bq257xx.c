// SPDX-License-Identifier: GPL-2.0
/*
 * BQ257XX Core Driver
 * Copyright (C) 2025 Chris Morgan <macromorgan@hotmail.com>
 */

#include <linux/bitfield.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/mfd/bq257xx.h>
#include <linux/mfd/core.h>
#include <linux/regmap.h>

static const struct regmap_range bq25703_readonly_reg_ranges[] = {
	regmap_reg_range(BQ25703_CHARGER_STATUS, BQ25703_MANUFACT_DEV_ID),
};

static const struct regmap_access_table bq25703_writeable_regs = {
	.no_ranges = bq25703_readonly_reg_ranges,
	.n_no_ranges = ARRAY_SIZE(bq25703_readonly_reg_ranges),
};

static const struct regmap_range bq25703_volatile_reg_ranges[] = {
	regmap_reg_range(BQ25703_CHARGE_OPTION_0, BQ25703_IIN_HOST),
	regmap_reg_range(BQ25703_CHARGER_STATUS, BQ25703_ADC_OPTION),
};

static const struct regmap_access_table bq25703_volatile_regs = {
	.yes_ranges = bq25703_volatile_reg_ranges,
	.n_yes_ranges = ARRAY_SIZE(bq25703_volatile_reg_ranges),
};

static const struct regmap_config bq25703_regmap_config = {
	.reg_bits = 8,
	.val_bits = 16,
	.max_register = BQ25703_ADC_OPTION,
	.cache_type = REGCACHE_MAPLE,
	.wr_table = &bq25703_writeable_regs,
	.volatile_table = &bq25703_volatile_regs,
	.val_format_endian = REGMAP_ENDIAN_LITTLE,
};

static const struct regmap_range bq25792_writeable_reg_ranges[] = {
	regmap_reg_range(BQ25792_REG00_MIN_SYS_VOLTAGE,
			 BQ25792_REG18_NTC_CONTROL_1),
	regmap_reg_range(BQ25792_REG28_CHARGER_MASK_0,
			 BQ25792_REG30_ADC_FUNCTION_DISABLE_1),
};

static const struct regmap_access_table bq25792_writeable_regs = {
	.yes_ranges = bq25792_writeable_reg_ranges,
	.n_yes_ranges = ARRAY_SIZE(bq25792_writeable_reg_ranges),
};

static const struct regmap_range bq25792_volatile_reg_ranges[] = {
	regmap_reg_range(BQ25792_REG05_INPUT_VOLTAGE_LIMIT,
			 BQ25792_REG06_INPUT_CURRENT_LIMIT + 1),
	regmap_reg_range(BQ25792_REG0F_CHARGER_CONTROL_0,
			 BQ25792_REG0F_CHARGER_CONTROL_0),
	regmap_reg_range(BQ25792_REG11_CHARGER_CONTROL_2,
			 BQ25792_REG11_CHARGER_CONTROL_2),
	regmap_reg_range(BQ25792_REG13_CHARGER_CONTROL_4,
			 BQ25792_REG13_CHARGER_CONTROL_4),
	regmap_reg_range(BQ25792_REG19_ICO_CURRENT_LIMIT,
			 BQ25792_REG27_FAULT_FLAG_1),
	regmap_reg_range(BQ25792_REG2E_ADC_CONTROL,
			 BQ25792_REG2E_ADC_CONTROL),
	regmap_reg_range(BQ25792_REG31_IBUS_ADC,
			 BQ25792_REG47_DPDM_DRIVER),
};

static const struct regmap_access_table bq25792_volatile_regs = {
	.yes_ranges = bq25792_volatile_reg_ranges,
	.n_yes_ranges = ARRAY_SIZE(bq25792_volatile_reg_ranges),
};

static const struct regmap_config bq25792_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = BQ25792_REG48_PART_INFORMATION,
	.cache_type = REGCACHE_MAPLE,
	.wr_table = &bq25792_writeable_regs,
	.volatile_table = &bq25792_volatile_regs,
};

static const struct mfd_cell bq25703_cells[] = {
	MFD_CELL_NAME("bq257xx-regulator"),
	MFD_CELL_NAME("bq257xx-charger"),
	MFD_CELL_NAME("bq257xx-fgu"),
};

static const struct mfd_cell bq25792_cells[] = {
	MFD_CELL_NAME("bq257xx-regulator"),
	MFD_CELL_NAME("bq257xx-charger"),
};

/*
 * Verify that the device on the bus reports the ID expected for the type
 * selected by the compatible string. Only the SC8886S is checked here; the
 * BQ25703A and BQ25792 have historically been probed without an ID check.
 */
static int bq257xx_check_dev_id(struct bq257xx_device *ddata)
{
	unsigned int id;
	int ret;

	if (ddata->type != SC8886S)
		return 0;

	ret = regmap_read(ddata->regmap, BQ25703_MANUFACT_DEV_ID, &id);
	if (ret)
		return ret;

	if (FIELD_GET(SC8886S_MANUFACT_ID_MASK, id) != SC8886S_MANUFACT_ID) {
		dev_err(&ddata->client->dev,
			"Unexpected manufacturer ID 0x%02x (expected 0x%02x)\n",
			FIELD_GET(SC8886S_MANUFACT_ID_MASK, id), SC8886S_MANUFACT_ID);
		return -ENODEV;
	}

	if (FIELD_GET(SC8886S_DEV_ID_MASK, id) != SC8886S_DEV_ID) {
		dev_err(&ddata->client->dev,
			"Unexpected device ID 0x%02x (expected 0x%02x)\n",
			FIELD_GET(SC8886S_DEV_ID_MASK, id), SC8886S_DEV_ID);
		return -ENODEV;
	}

	return 0;
}

static int bq257xx_probe(struct i2c_client *client)
{
	const struct regmap_config *rcfg;
	const struct mfd_cell *cells;
	struct bq257xx_device *ddata;
	int n_cells, ret;

	ddata = devm_kzalloc(&client->dev, sizeof(*ddata), GFP_KERNEL);
	if (!ddata)
		return -ENOMEM;

	ddata->type = (uintptr_t)i2c_get_match_data(client);
	ddata->client = client;

	switch (ddata->type) {
	case BQ25703A:
	case SC8886S:
		rcfg = &bq25703_regmap_config;
		break;
	case BQ25792:
		rcfg = &bq25792_regmap_config;
		break;
	default:
		return dev_err_probe(&client->dev, -ENODEV, "Unsupported device type\n");
	}

	ddata->regmap = devm_regmap_init_i2c(client, rcfg);
	if (IS_ERR(ddata->regmap)) {
		return dev_err_probe(&client->dev, PTR_ERR(ddata->regmap),
				     "Failed to allocate register map\n");
	}

	i2c_set_clientdata(client, ddata);
	ret = bq257xx_check_dev_id(ddata);
	if (ret)
		return dev_err_probe(&client->dev, ret, "Failed to verify device ID\n");

	if (ddata->type == BQ25792) {
		cells = bq25792_cells;
		n_cells = ARRAY_SIZE(bq25792_cells);
	} else {
		cells = bq25703_cells;
		n_cells = ARRAY_SIZE(bq25703_cells);
	}

	ret = devm_mfd_add_devices(&client->dev, PLATFORM_DEVID_AUTO,
				   cells, n_cells, NULL, 0, NULL);
	if (ret)
		return dev_err_probe(&client->dev, ret,
				     "Failed to register child devices\n");

	return ret;
}

static const struct i2c_device_id bq257xx_i2c_ids[] = {
	{ "bq25703a", BQ25703A },
	{ "bq25792", BQ25792 },
	{ "sc8886s", SC8886S },
	{}
};
MODULE_DEVICE_TABLE(i2c, bq257xx_i2c_ids);

static const struct of_device_id bq257xx_of_match[] = {
	{ .compatible = "southchip,sc8886s", .data = (void *)SC8886S },
	{ .compatible = "ti,bq25703a", .data = (void *)BQ25703A },
	{ .compatible = "ti,bq25792", .data = (void *)BQ25792 },
	{}
};
MODULE_DEVICE_TABLE(of, bq257xx_of_match);

static struct i2c_driver bq257xx_driver = {
	.driver = {
		.name = "bq257xx",
		.of_match_table = bq257xx_of_match,
	},
	.probe = bq257xx_probe,
	.id_table = bq257xx_i2c_ids,
};
module_i2c_driver(bq257xx_driver);

MODULE_DESCRIPTION("bq257xx buck/boost/charger driver");
MODULE_AUTHOR("Chris Morgan <macromorgan@hotmail.com>");
MODULE_LICENSE("GPL");
