// SPDX-License-Identifier: GPL-2.0-only
/*
 * Thermal sensor driver for the HiSilicon Hi3519DV500 SoC
 *
 * The sensor reports one channel and raises an interrupt when the
 * temperature crosses the configured upper limit.
 *
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023.
 * Copyright (c) 2026 Jun Yan.
 */

#include <linux/bitops.h>
#include <linux/bits.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/thermal.h>

#define HI3519DV500_TSENSOR_CTRL0		(0x00)
#define  HI3519DV500_TSENSOR_EN			BIT(31)
#define  HI3519DV500_TSENSOR_MONITOR_EN		BIT(30)
#define  HI3519DV500_TSENSOR_MONITOR_PERIOD(x)	((GENMASK(7, 0) & (x)) << 20)
#define  HI3519DV500_TSENSOR_UPLIMIT(x)		((GENMASK(9, 0) & (x)) << 10)
#define  HI3519DV500_TSENSOR_TEMP_CT_SEL(x)	((GENMASK(1, 0) & (x)) << 2)
#define HI3519DV500_TSENSOR_CTRL2		(0x08)
#define  HI3519DV500_TSENSOR_RESULT0		GENMASK(9, 0)
#define HI3519DV500_TSENSOR_INT_MASK		(0x20)
#define  HI3519DV500_TSENSOR_UP_WARN_INT_MASK	BIT(0)
#define HI3519DV500_TSENSOR_INT_CLR		(0x24)
#define  HI3519DV500_TSENSOR_UP_WARN_INT_CLR	BIT(0)

#define HI3519DV500_TEMP_BASE			(-40000)
#define HI3519DV500_TEMP_STEP			(205)
#define HI3519DV500_TEMP_RESULT_MIN		(132)
#define HI3519DV500_CONVERT_TIME		(0)	/* 0.512 ms */
#define HI3519DV500_MONITOR_PERIOD		(5)	/* 5 * 2 = 10 ms */

struct hi3519dv500_thermal {
	struct device *dev;
	void __iomem *regs;
	struct thermal_zone_device *tzd;
	int thres_temp;
};

/*
 * The register value is 10 bit wide with a 132 step offset:
 *
 *	Temp = -40000 + 205 * (steps - 132)
 */
static int hi3519dv500_thermal_step_to_temp(int step)
{
	return HI3519DV500_TEMP_BASE +
	       HI3519DV500_TEMP_STEP * (step - HI3519DV500_TEMP_RESULT_MIN);
}

static int hi3519dv500_thermal_temp_to_step(int temp)
{
	return DIV_ROUND_UP(temp - HI3519DV500_TEMP_BASE, HI3519DV500_TEMP_STEP) +
	       HI3519DV500_TEMP_RESULT_MIN;
}

static void hi3519dv500_thermal_ctrl0_set(struct hi3519dv500_thermal *priv,
					  u32 value, u32 mask)
{
	writel((readl(priv->regs + HI3519DV500_TSENSOR_CTRL0) & ~mask) | value,
	       priv->regs + HI3519DV500_TSENSOR_CTRL0);
}

static void hi3519dv500_thermal_enable(struct hi3519dv500_thermal *priv,
				       bool enable)
{
	hi3519dv500_thermal_ctrl0_set(priv,
				      enable ? HI3519DV500_TSENSOR_EN : 0,
				      HI3519DV500_TSENSOR_EN);
}

static void hi3519dv500_thermal_alarm_enable(struct hi3519dv500_thermal *priv,
					     bool enable)
{
	u32 reg = readl(priv->regs + HI3519DV500_TSENSOR_INT_MASK);

	if (enable)
		reg |= HI3519DV500_TSENSOR_UP_WARN_INT_MASK;
	else
		reg &= ~HI3519DV500_TSENSOR_UP_WARN_INT_MASK;

	writel(reg, priv->regs + HI3519DV500_TSENSOR_INT_MASK);
}

static int hi3519dv500_thermal_get_temp(struct thermal_zone_device *tz, int *temp)
{
	struct hi3519dv500_thermal *priv = thermal_zone_device_priv(tz);
	u32 raw = readl(priv->regs + HI3519DV500_TSENSOR_CTRL2);

	*temp = hi3519dv500_thermal_step_to_temp(raw & HI3519DV500_TSENSOR_RESULT0);

	return 0;
}

static const struct thermal_zone_device_ops hi3519dv500_thermal_ops = {
	.get_temp = hi3519dv500_thermal_get_temp,
};

static irqreturn_t hi3519dv500_thermal_irq(int irq, void *dev_id)
{
	struct hi3519dv500_thermal *priv = dev_id;
	int temp;

	writel(HI3519DV500_TSENSOR_UP_WARN_INT_CLR,
	       priv->regs + HI3519DV500_TSENSOR_INT_CLR);

	hi3519dv500_thermal_get_temp(priv->tzd, &temp);

	if (temp >= priv->thres_temp)
		dev_crit(priv->dev, "THERMAL ALARM: %d > %d\n",
			 temp, priv->thres_temp);
	else
		dev_crit(priv->dev, "THERMAL ALARM stopped: %d < %d\n",
			 temp, priv->thres_temp);

	thermal_zone_device_update(priv->tzd, THERMAL_EVENT_UNSPECIFIED);

	return IRQ_HANDLED;
}

static int hi3519dv500_thermal_trip_cb(struct thermal_trip *trip, void *arg)
{
	struct hi3519dv500_thermal *priv = arg;

	if (trip->type != THERMAL_TRIP_PASSIVE)
		return 0;

	priv->thres_temp = trip->temperature;

	return 1;
}

static int hi3519dv500_thermal_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct hi3519dv500_thermal *priv;
	u32 value, period = HI3519DV500_MONITOR_PERIOD;
	int irq, ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = dev;

	priv->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(priv->regs))
		return PTR_ERR(priv->regs);

	priv->tzd = devm_thermal_of_zone_register(dev, 0, priv,
						  &hi3519dv500_thermal_ops);
	if (IS_ERR(priv->tzd))
		return dev_err_probe(dev, PTR_ERR(priv->tzd),
				     "failed to register thermal zone\n");

	thermal_zone_for_each_trip(priv->tzd, hi3519dv500_thermal_trip_cb, priv);

	/* disable interrupt and the sensor before configuring it */
	hi3519dv500_thermal_alarm_enable(priv, false);
	hi3519dv500_thermal_enable(priv, false);

	/* conversion time and monitor period */
	hi3519dv500_thermal_ctrl0_set(priv,
				      HI3519DV500_TSENSOR_TEMP_CT_SEL(HI3519DV500_CONVERT_TIME),
				      GENMASK(3, 2));
	hi3519dv500_thermal_ctrl0_set(priv,
				      HI3519DV500_TSENSOR_MONITOR_PERIOD(period),
				      GENMASK(27, 20));
	hi3519dv500_thermal_ctrl0_set(priv, HI3519DV500_TSENSOR_MONITOR_EN,
				      HI3519DV500_TSENSOR_MONITOR_EN);

	/* upper limit from the passive trip point */
	value = hi3519dv500_thermal_temp_to_step(priv->thres_temp);
	hi3519dv500_thermal_ctrl0_set(priv, HI3519DV500_TSENSOR_UPLIMIT(value),
				      GENMASK(19, 10));

	hi3519dv500_thermal_enable(priv, true);

	writel(HI3519DV500_TSENSOR_UP_WARN_INT_CLR,
	       priv->regs + HI3519DV500_TSENSOR_INT_CLR);
	hi3519dv500_thermal_alarm_enable(priv, true);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	ret = devm_request_threaded_irq(dev, irq, NULL, hi3519dv500_thermal_irq,
					IRQF_ONESHOT, "tsensor", priv);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request irq\n");

	platform_set_drvdata(pdev, priv);

	return 0;
}

static const struct of_device_id hi3519dv500_thermal_of_match[] = {
	{ .compatible = "hisilicon,hi3519dv500-tsensor" },
	{ /* end */ }
};
MODULE_DEVICE_TABLE(of, hi3519dv500_thermal_of_match);

static struct platform_driver hi3519dv500_thermal_driver = {
	.probe = hi3519dv500_thermal_probe,
	.driver = {
		.name = "hi3519dv500-thermal",
		.of_match_table = hi3519dv500_thermal_of_match,
	},
};
module_platform_driver(hi3519dv500_thermal_driver);

MODULE_AUTHOR("Jun Yan <jerrysteve1101@gmail.com>");
MODULE_DESCRIPTION("HiSilicon Hi3519DV500 thermal sensor driver");
MODULE_LICENSE("GPL");
