// SPDX-License-Identifier: GPL-2.0-only
/*
 * es7202.c -- Everest Semi ES7202 PDM ADC ALSA SoC driver
 *
 * Copyright (C) 2020 Everest Semiconductor Co., Ltd.
 *
 * Author: David Yang <yangxiaohua@everest-semi.com>
 *
 * Based on the Rockchip BSP driver for ES7202.
 */

#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/tlv.h>

/* ES7202 register space */
#define ES7202_RESET		0x00
#define ES7202_SOFT_MODE	0x01
#define ES7202_CLK_DIV		0x02
#define ES7202_CLK_EN		0x03
#define ES7202_T1_VMID		0x04
#define ES7202_T2_VMID		0x05
#define ES7202_CHIP_STA		0x06
#define ES7202_PDM_INF_CTL	0x07
#define ES7202_MISC_CTL		0x08
#define ES7202_ANALOG_EN	0x10
#define ES7202_BIAS_VMID	0x11
#define ES7202_PGA1_BIAS	0x12
#define ES7202_PGA2_BIAS	0x13
#define ES7202_MOD1_BIAS	0x14
#define ES7202_MOD2_BIAS	0x15
#define ES7202_VREFP_BIAS	0x16
#define ES7202_VMMOD_BIAS	0x17
#define ES7202_MODS_BIAS	0x18
#define ES7202_ANALOG_LP1	0x19
#define ES7202_ANALOG_LP2	0x1a
#define ES7202_ANALOG_MISC1	0x1b
#define ES7202_ANALOG_MISC2	0x1c
#define ES7202_PGA1		0x1d
#define ES7202_PGA2		0x1e

/* PDM interface control: bit[1:0] mute/enable the PDM output */
#define ES7202_PDM_INF_CTL_MUTE	GENMASK(1, 0)

/*
 * The main supply voltage selects the analog front-end configuration:
 * below this threshold the device is configured for 1.8 V, otherwise 3.3 V.
 */
#define ES7202_SUPPLY_1V8_MAX_UV	1980000
#define ES7202_SUPPLY_3V3_DEFAULT_UV	3300000

struct es7202_priv {
	struct regmap *regmap;
	struct regulator *supply;
	struct gpio_desc *reset_gpio;
	unsigned int supply_uv;
};

static bool es7202_volatile_reg(struct device *dev, unsigned int reg)
{
	/* chip status is read-only and may change at any time */
	return reg == ES7202_CHIP_STA;
}

static const struct regmap_config es7202_regmap = {
	.reg_bits	= 8,
	.val_bits	= 8,
	.max_register	= ES7202_PGA2,
	.volatile_reg	= es7202_volatile_reg,
	.cache_type	= REGCACHE_MAPLE,
};

static const SNDRV_CTL_TLVD_DECLARE_DB_SCALE(mic_boost_tlv, 0, 300, 0);

static const struct snd_kcontrol_new es7202_controls[] = {
	SOC_SINGLE_TLV("PGA1 Capture Volume", ES7202_PGA1, 0, 0x0c, 0,
		       mic_boost_tlv),
	SOC_SINGLE_TLV("PGA2 Capture Volume", ES7202_PGA2, 0, 0x0c, 0,
		       mic_boost_tlv),
};

static const struct snd_soc_dapm_widget es7202_dapm_widgets[] = {
	SND_SOC_DAPM_INPUT("MIC1"),
	SND_SOC_DAPM_INPUT("MIC2"),
	SND_SOC_DAPM_ADC("ADC", "Capture", SND_SOC_NOPM, 0, 0),
};

static const struct snd_soc_dapm_route es7202_dapm_routes[] = {
	{ "ADC", NULL, "MIC1" },
	{ "ADC", NULL, "MIC2" },
};

static int es7202_hw_init(struct es7202_priv *es7202)
{
	struct regmap *regmap = es7202->regmap;
	bool is_1v8 = es7202->supply_uv > 0 &&
		      es7202->supply_uv <= ES7202_SUPPLY_1V8_MAX_UV;
	int ret;

	ret = regmap_write(regmap, ES7202_SOFT_MODE, 0x01);
	if (ret)
		return ret;

	regmap_write(regmap, ES7202_ANALOG_MISC1, is_1v8 ? 0x40 : 0x50);
	regmap_write(regmap, ES7202_PGA1, 0x1b);
	regmap_write(regmap, ES7202_PGA2, 0x1b);
	regmap_write(regmap, ES7202_ANALOG_EN, 0x7f);
	regmap_write(regmap, ES7202_BIAS_VMID, 0x2f);
	regmap_write(regmap, ES7202_ANALOG_EN, is_1v8 ? 0x3f : 0x0f);
	regmap_write(regmap, ES7202_ANALOG_EN, 0x00);

	regmap_write(regmap, ES7202_MOD1_BIAS, 0x58);
	regmap_write(regmap, ES7202_CLK_DIV, 0x01);
	regmap_write(regmap, ES7202_T2_VMID, 0x01);
	regmap_write(regmap, ES7202_MISC_CTL, 0x02);
	regmap_write(regmap, ES7202_RESET, 0x01);
	regmap_write(regmap, ES7202_CLK_EN, 0x03);
	regmap_write(regmap, ES7202_BIAS_VMID, 0x2e);

	/* unmute the PDM output */
	return regmap_update_bits(regmap, ES7202_PDM_INF_CTL,
				  ES7202_PDM_INF_CTL_MUTE, 0x00);
}

static int es7202_mute_stream(struct snd_soc_dai *dai, int mute, int stream)
{
	struct snd_soc_component *component = dai->component;
	struct es7202_priv *es7202 = snd_soc_component_get_drvdata(component);

	if (stream == SNDRV_PCM_STREAM_PLAYBACK)
		return 0;

	regmap_update_bits(es7202->regmap, ES7202_PDM_INF_CTL,
			   ES7202_PDM_INF_CTL_MUTE,
			   mute ? ES7202_PDM_INF_CTL_MUTE : 0x00);

	return 0;
}

static const struct snd_soc_dai_ops es7202_dai_ops = {
	.mute_stream = es7202_mute_stream,
};

static struct snd_soc_dai_driver es7202_dai = {
	.name = "es7202-hifi",
	.capture = {
		.stream_name	= "Capture",
		.channels_min	= 1,
		.channels_max	= 2,
		.rates		= SNDRV_PCM_RATE_8000_96000,
		.formats	= SNDRV_PCM_FMTBIT_S16_LE,
	},
	.ops = &es7202_dai_ops,
};

static int es7202_suspend(struct snd_soc_component *component)
{
	struct es7202_priv *es7202 = snd_soc_component_get_drvdata(component);

	regmap_update_bits(es7202->regmap, ES7202_PDM_INF_CTL,
			   ES7202_PDM_INF_CTL_MUTE, ES7202_PDM_INF_CTL_MUTE);

	return 0;
}

static int es7202_resume(struct snd_soc_component *component)
{
	struct es7202_priv *es7202 = snd_soc_component_get_drvdata(component);

	regmap_update_bits(es7202->regmap, ES7202_PDM_INF_CTL,
			   ES7202_PDM_INF_CTL_MUTE, 0x00);

	return 0;
}

static const struct snd_soc_component_driver es7202_component_driver = {
	.controls		= es7202_controls,
	.num_controls		= ARRAY_SIZE(es7202_controls),
	.dapm_widgets		= es7202_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(es7202_dapm_widgets),
	.dapm_routes		= es7202_dapm_routes,
	.num_dapm_routes	= ARRAY_SIZE(es7202_dapm_routes),
	.suspend		= es7202_suspend,
	.resume			= es7202_resume,
	.idle_bias_on		= 1,
	.endianness		= 1,
};

static int es7202_i2c_probe(struct i2c_client *i2c)
{
	struct device *dev = &i2c->dev;
	struct es7202_priv *es7202;
	int ret;

	es7202 = devm_kzalloc(dev, sizeof(*es7202), GFP_KERNEL);
	if (!es7202)
		return -ENOMEM;

	i2c_set_clientdata(i2c, es7202);

	es7202->regmap = devm_regmap_init_i2c(i2c, &es7202_regmap);
	if (IS_ERR(es7202->regmap))
		return dev_err_probe(dev, PTR_ERR(es7202->regmap),
				     "failed to init regmap\n");

	es7202->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						     GPIOD_OUT_HIGH);
	if (IS_ERR(es7202->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(es7202->reset_gpio),
				     "failed to get reset gpio\n");

	es7202->supply = devm_regulator_get_optional(dev, "power");
	if (IS_ERR(es7202->supply)) {
		if (PTR_ERR(es7202->supply) != -ENODEV)
			return dev_err_probe(dev, PTR_ERR(es7202->supply),
					     "failed to get supply\n");

		es7202->supply = NULL;
		es7202->supply_uv = ES7202_SUPPLY_3V3_DEFAULT_UV;
	} else {
		ret = regulator_enable(es7202->supply);
		if (ret)
			return dev_err_probe(dev, ret,
					     "failed to enable supply\n");

		es7202->supply_uv = regulator_get_voltage(es7202->supply);
		if (es7202->supply_uv < 0)
			es7202->supply_uv = ES7202_SUPPLY_3V3_DEFAULT_UV;
	}

	/* take the device out of reset */
	gpiod_set_value_cansleep(es7202->reset_gpio, 0);
	usleep_range(1000, 2000);

	ret = es7202_hw_init(es7202);
	if (ret)
		return dev_err_probe(dev, ret, "failed to init device\n");

	return devm_snd_soc_register_component(dev, &es7202_component_driver,
					       &es7202_dai, 1);
}

static const struct of_device_id es7202_of_match[] = {
	{ .compatible = "everest,es7202" },
	{ }
};
MODULE_DEVICE_TABLE(of, es7202_of_match);

static const struct i2c_device_id es7202_i2c_id[] = {
	{ "es7202" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, es7202_i2c_id);

static struct i2c_driver es7202_i2c_driver = {
	.driver = {
		.name		= "es7202",
		.of_match_table	= of_match_ptr(es7202_of_match),
	},
	.probe		= es7202_i2c_probe,
	.id_table	= es7202_i2c_id,
};
module_i2c_driver(es7202_i2c_driver);

MODULE_DESCRIPTION("Everest Semi ES7202 PDM ADC ALSA SoC Codec Driver");
MODULE_AUTHOR("David Yang <yangxiaohua@everest-semi.com>");
MODULE_LICENSE("GPL v2");
