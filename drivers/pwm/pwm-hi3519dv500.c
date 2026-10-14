// SPDX-License-Identifier: GPL-2.0-only
/*
 * PWM driver for the HiSilicon Hi3519DV500 SoC
 *
 * The SoC has three independent PWM controllers with six, six and five
 * channels.  Every channel has 32 bit period and duty cycle registers
 * at a 0x100 stride and a control register with enable, polarity and
 * clock prescaler fields.  The PWM_KEEP bit keeps the output of a
 * channel stable while its configuration registers are updated.
 *
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023.
 * Copyright (c) 2026 Jun Yan.
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/reset.h>

/*
 * Register offset of the xth channel
 */
#define HI3519DV500_PWM_PERIOD_CFG(x)	(0x0000 + (0x100 * (x)))
#define HI3519DV500_PWM_DUTY_CFG(x)	(0x0004 + (0x100 * (x)))
#define HI3519DV500_PWM_CTRL(x)		(0x0014 + (0x100 * (x)))

#define HI3519DV500_PWM_ENABLE		BIT(0)
#define HI3519DV500_PWM_POLARITY	BIT(1)
#define HI3519DV500_PWM_KEEP		BIT(2)

#define HI3519DV500_PWM_PRE_DIV_SEL	GENMASK(11, 8)
#define HI3519DV500_PWM_PRE_DIV_1	0

#define HI3519DV500_PWM_PERIOD_MASK	GENMASK(31, 0)
#define HI3519DV500_PWM_DUTY_MASK	GENMASK(31, 0)

struct hi3519dv500_pwm_chip {
	struct clk *clk;
	void __iomem *base;
	struct reset_control *rstc;
};

static inline struct hi3519dv500_pwm_chip *to_hi3519dv500_pwm_chip(struct pwm_chip *chip)
{
	return pwmchip_get_drvdata(chip);
}

static void hi3519dv500_pwm_set_bits(struct hi3519dv500_pwm_chip *priv,
				     u32 offset, u32 mask, u32 data)
{
	void __iomem *address = priv->base + offset;
	u32 value;

	value = readl(address);
	value &= ~mask;
	value |= (data & mask);
	writel(value, address);
}

static int hi3519dv500_pwm_apply(struct pwm_chip *chip, struct pwm_device *pwm,
				const struct pwm_state *state)
{
	struct hi3519dv500_pwm_chip *priv = to_hi3519dv500_pwm_chip(chip);
	u64 rate, period, duty;

	if (state->polarity != pwm->state.polarity)
		hi3519dv500_pwm_set_bits(priv,
				HI3519DV500_PWM_CTRL(pwm->hwpwm),
				HI3519DV500_PWM_POLARITY,
				state->polarity == PWM_POLARITY_INVERSED ?
				HI3519DV500_PWM_POLARITY : 0);

	if (state->period != pwm->state.period ||
	    state->duty_cycle != pwm->state.duty_cycle) {
		rate = div_u64(clk_get_rate(priv->clk), 1000000);

		period = div64_u64(state->period * rate, 1000);
		if (period > HI3519DV500_PWM_PERIOD_MASK)
			return -EINVAL;

		duty = div64_u64(state->duty_cycle * rate, 1000);

		hi3519dv500_pwm_set_bits(priv,
				HI3519DV500_PWM_PERIOD_CFG(pwm->hwpwm),
				HI3519DV500_PWM_PERIOD_MASK, period);
		hi3519dv500_pwm_set_bits(priv,
				HI3519DV500_PWM_DUTY_CFG(pwm->hwpwm),
				HI3519DV500_PWM_DUTY_MASK, duty);
	}

	if (state->enabled != pwm->state.enabled)
		hi3519dv500_pwm_set_bits(priv,
				HI3519DV500_PWM_CTRL(pwm->hwpwm),
				HI3519DV500_PWM_ENABLE,
				state->enabled ? HI3519DV500_PWM_ENABLE : 0);

	return 0;
}

static int hi3519dv500_pwm_get_state(struct pwm_chip *chip,
				     struct pwm_device *pwm,
				     struct pwm_state *state)
{
	struct hi3519dv500_pwm_chip *priv = to_hi3519dv500_pwm_chip(chip);
	u64 rate, value;

	rate = div_u64(clk_get_rate(priv->clk), 1000000);

	value = readl(priv->base + HI3519DV500_PWM_PERIOD_CFG(pwm->hwpwm));
	state->period = div64_u64(value * 1000, rate);

	value = readl(priv->base + HI3519DV500_PWM_DUTY_CFG(pwm->hwpwm));
	state->duty_cycle = div64_u64(value * 1000, rate);

	value = readl(priv->base + HI3519DV500_PWM_CTRL(pwm->hwpwm));
	state->enabled = value & HI3519DV500_PWM_ENABLE;
	state->polarity = (value & HI3519DV500_PWM_POLARITY) ?
			  PWM_POLARITY_INVERSED : PWM_POLARITY_NORMAL;

	return 0;
}

static const struct pwm_ops hi3519dv500_pwm_ops = {
	.apply = hi3519dv500_pwm_apply,
	.get_state = hi3519dv500_pwm_get_state,
};

static int hi3519dv500_pwm_probe(struct platform_device *pdev)
{
	struct hi3519dv500_pwm_chip *priv;
	struct pwm_chip *chip;
	int ret, i;

	chip = devm_pwmchip_alloc(&pdev->dev,
			(u64)(uintptr_t)of_device_get_match_data(&pdev->dev),
			sizeof(*priv));
	if (IS_ERR(chip))
		return PTR_ERR(chip);
	priv = to_hi3519dv500_pwm_chip(chip);

	priv->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	priv->clk = devm_clk_get_enabled(&pdev->dev, NULL);
	if (IS_ERR(priv->clk))
		return dev_err_probe(&pdev->dev, PTR_ERR(priv->clk),
				      "failed to get clock\n");

	priv->rstc = devm_reset_control_get_exclusive(&pdev->dev, NULL);
	if (IS_ERR(priv->rstc))
		return dev_err_probe(&pdev->dev, PTR_ERR(priv->rstc),
				      "failed to get reset\n");

	ret = reset_control_reset(priv->rstc);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				      "failed to reset\n");

	chip->ops = &hi3519dv500_pwm_ops;

	ret = devm_pwmchip_add(&pdev->dev, chip);
	if (ret)
		return ret;

	/*
	 * Keep the output of all channels stable while their
	 * configuration registers are updated.
	 */
	for (i = 0; i < chip->npwm; i++)
		hi3519dv500_pwm_set_bits(priv, HI3519DV500_PWM_CTRL(i),
				HI3519DV500_PWM_KEEP, HI3519DV500_PWM_KEEP);

	return 0;
}

static const struct of_device_id hi3519dv500_pwm_of_match[] = {
	{ .compatible = "hisilicon,hi3519dv500-pwm0", .data = (void *)6 },
	{ .compatible = "hisilicon,hi3519dv500-pwm1", .data = (void *)6 },
	{ .compatible = "hisilicon,hi3519dv500-pwm2", .data = (void *)5 },
	{ }
};
MODULE_DEVICE_TABLE(of, hi3519dv500_pwm_of_match);

static struct platform_driver hi3519dv500_pwm_driver = {
	.driver = {
		.name = "pwm-hi3519dv500",
		.of_match_table = hi3519dv500_pwm_of_match,
	},
	.probe = hi3519dv500_pwm_probe,
};
module_platform_driver(hi3519dv500_pwm_driver);

MODULE_DESCRIPTION("PWM driver for the HiSilicon Hi3519DV500 SoC");
MODULE_AUTHOR("Jun Yan <jerrysteve1101@gmail.com>");
MODULE_LICENSE("GPL");
