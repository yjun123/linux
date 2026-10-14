// SPDX-License-Identifier: GPL-2.0-only
/*
 * HiSilicon Hi3519DV500 LSADC driver
 *
 * The Hi3519DV500 Low Speed ADC provides four 10 bit voltage input
 * channels.  A conversion is started on demand and the converted
 * value is reported with an interrupt.
 *
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023.
 * Copyright (c) 2026 Jun Yan.
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/reset.h>

#include <linux/iio/iio.h>

#define HI3519DV500_LSADC_CONFIG			0x00
#define  HI3519DV500_CONFIG_DEGLITCH			BIT(17)
#define  HI3519DV500_CONFIG_RESET				BIT(15)
#define  HI3519DV500_CONFIG_MODE				BIT(13)
#define  HI3519DV500_CONFIG_EQU_MODE				BIT(12)
#define  HI3519DV500_CONFIG_CHN3				BIT(11)
#define  HI3519DV500_CONFIG_CHN2				BIT(10)
#define  HI3519DV500_CONFIG_CHN1				BIT(9)
#define  HI3519DV500_CONFIG_CHN0				BIT(8)
#define  HI3519DV500_CONFIG_EQU_CHN3				BIT(3)
#define  HI3519DV500_CONFIG_EQU_CHN2				BIT(2)
#define  HI3519DV500_CONFIG_EQU_CHN1				BIT(1)
#define  HI3519DV500_CONFIG_EQU_CHN0				BIT(0)

#define HI3519DV500_LSADC_TIMESCAN				0x08
#define HI3519DV500_LSADC_INTEN				0x10
#define HI3519DV500_LSADC_INTSTATUS				0x14
#define HI3519DV500_LSADC_INTCLR				0x18
#define HI3519DV500_LSADC_START				0x1c
#define HI3519DV500_LSADC_STOP				0x20
#define HI3519DV500_LSADC_ACTBIT				0x24
#define HI3519DV500_LSADC_CHNDATA				0x2c

#define HI3519DV500_LSADC_NUM_BITS				10
#define HI3519DV500_LSADC_CHN_MASK				0xf

/*
 * The ADC is clocked with a fixed 3 MHz clock.  Give a conversion
 * 10 ms to finish before bailing out.
 */
#define HI3519DV500_LSADC_TIMEOUT_MS				10

struct hi3519dv500_lsadc {
	void __iomem		*base;
	struct clk		*clk;
	struct reset_control	*rstc;
	struct regulator	*vref;
	struct mutex		lock;
	struct completion	completion;
	u32			value;
};

#define HI3519DV500_ADC_CHANNEL(_index) {			\
	.type = IIO_VOLTAGE,					\
	.indexed = 1,						\
	.channel = (_index),					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),	\
	.datasheet_name = "adc" #_index,			\
}

static const struct iio_chan_spec hi3519dv500_lsadc_channels[] = {
	HI3519DV500_ADC_CHANNEL(0),
	HI3519DV500_ADC_CHANNEL(1),
	HI3519DV500_ADC_CHANNEL(2),
	HI3519DV500_ADC_CHANNEL(3),
};

static void hi3519dv500_lsadc_start_conv(struct hi3519dv500_lsadc *adc,
					 unsigned int channel)
{
	u32 con;

	/* Set the number of valid bits. */
	writel(GENMASK(HI3519DV500_LSADC_NUM_BITS - 1, 0),
	       adc->base + HI3519DV500_LSADC_ACTBIT);

	con = readl(adc->base + HI3519DV500_LSADC_CONFIG);
	con &= ~HI3519DV500_CONFIG_RESET;
	con &= ~HI3519DV500_CONFIG_EQU_MODE;
	con |= HI3519DV500_CONFIG_DEGLITCH | HI3519DV500_CONFIG_MODE;
	con &= ~(HI3519DV500_CONFIG_CHN0 | HI3519DV500_CONFIG_CHN1 |
		 HI3519DV500_CONFIG_CHN2 | HI3519DV500_CONFIG_CHN3 |
		 HI3519DV500_CONFIG_EQU_CHN0 | HI3519DV500_CONFIG_EQU_CHN1 |
		 HI3519DV500_CONFIG_EQU_CHN2 | HI3519DV500_CONFIG_EQU_CHN3);
	con |= HI3519DV500_CONFIG_CHN0 << channel;
	con |= HI3519DV500_CONFIG_EQU_CHN0 << channel;
	writel(con, adc->base + HI3519DV500_LSADC_CONFIG);

	/* Clear pending interrupts. */
	writel(HI3519DV500_LSADC_CHN_MASK,
	       adc->base + HI3519DV500_LSADC_INTCLR);

	/* Enable the channel interrupt. */
	writel(1, adc->base + HI3519DV500_LSADC_INTEN);

	/* Start the scan. */
	writel(1, adc->base + HI3519DV500_LSADC_START);
}

static void hi3519dv500_lsadc_stop_conv(struct hi3519dv500_lsadc *adc)
{
	writel(0, adc->base + HI3519DV500_LSADC_INTEN);
	writel(1, adc->base + HI3519DV500_LSADC_STOP);
}

static int hi3519dv500_lsadc_read_raw(struct iio_dev *indio_dev,
				      struct iio_chan_spec const *chan,
				      int *val, int *val2, long mask)
{
	struct hi3519dv500_lsadc *adc = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		mutex_lock(&adc->lock);

		reinit_completion(&adc->completion);

		hi3519dv500_lsadc_start_conv(adc, chan->channel);

		if (!wait_for_completion_timeout(&adc->completion,
				msecs_to_jiffies(HI3519DV500_LSADC_TIMEOUT_MS))) {
			hi3519dv500_lsadc_stop_conv(adc);
			mutex_unlock(&adc->lock);
			return -ETIMEDOUT;
		}

		*val = adc->value;
		mutex_unlock(&adc->lock);
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		*val = regulator_get_voltage(adc->vref) / 1000;
		*val2 = HI3519DV500_LSADC_NUM_BITS;
		return IIO_VAL_FRACTIONAL_LOG2;
	default:
		return -EINVAL;
	}
}

static irqreturn_t hi3519dv500_lsadc_isr(int irq, void *dev_id)
{
	struct hi3519dv500_lsadc *adc = dev_id;
	u32 status;

	status = readl(adc->base + HI3519DV500_LSADC_INTSTATUS);
	if ((status & HI3519DV500_LSADC_CHN_MASK) == 0)
		return IRQ_NONE;

	/* Clear the interrupt. */
	writel(status & HI3519DV500_LSADC_CHN_MASK,
	       adc->base + HI3519DV500_LSADC_INTCLR);

	/* Read the converted value. */
	adc->value = readl(adc->base + HI3519DV500_LSADC_CHNDATA);
	adc->value &= GENMASK(HI3519DV500_LSADC_NUM_BITS - 1, 0);

	hi3519dv500_lsadc_stop_conv(adc);

	complete(&adc->completion);

	return IRQ_HANDLED;
}

static const struct iio_info hi3519dv500_lsadc_iio_info = {
	.read_raw = hi3519dv500_lsadc_read_raw,
};

static int hi3519dv500_lsadc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct hi3519dv500_lsadc *adc;
	struct iio_dev *indio_dev;
	int irq, ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*adc));
	if (!indio_dev)
		return -ENOMEM;
	adc = iio_priv(indio_dev);

	adc->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(adc->base))
		return PTR_ERR(adc->base);

	adc->clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(adc->clk))
		return dev_err_probe(dev, PTR_ERR(adc->clk),
				      "failed to get clock\n");

	adc->rstc = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(adc->rstc))
		return dev_err_probe(dev, PTR_ERR(adc->rstc),
				      "failed to get reset\n");

	ret = reset_control_reset(adc->rstc);
	if (ret)
		return dev_err_probe(dev, ret, "failed to reset\n");

	adc->vref = devm_regulator_get(dev, "vref");
	if (IS_ERR(adc->vref))
		return dev_err_probe(dev, PTR_ERR(adc->vref),
				      "failed to get regulator\n");

	ret = regulator_enable(adc->vref);
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable regulator\n");

	init_completion(&adc->completion);
	mutex_init(&adc->lock);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	ret = devm_request_irq(dev, irq, hi3519dv500_lsadc_isr, 0,
				dev_name(dev), adc);
	if (ret < 0)
		return dev_err_probe(dev, ret, "failed to request irq\n");

	indio_dev->name = dev_name(dev);
	indio_dev->info = &hi3519dv500_lsadc_iio_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = hi3519dv500_lsadc_channels;
	indio_dev->num_channels = ARRAY_SIZE(hi3519dv500_lsadc_channels);

	return devm_iio_device_register(dev, indio_dev);
}

static const struct of_device_id hi3519dv500_lsadc_of_match[] = {
	{ .compatible = "hisilicon,hi3519dv500-lsadc" },
	{ }
};
MODULE_DEVICE_TABLE(of, hi3519dv500_lsadc_of_match);

static struct platform_driver hi3519dv500_lsadc_driver = {
	.driver = {
		.name = "hi3519dv500-lsadc",
		.of_match_table = hi3519dv500_lsadc_of_match,
	},
	.probe = hi3519dv500_lsadc_probe,
};
module_platform_driver(hi3519dv500_lsadc_driver);

MODULE_DESCRIPTION("HiSilicon Hi3519DV500 LSADC driver");
MODULE_AUTHOR("Jun Yan <jerrysteve1101@gmail.com>");
MODULE_LICENSE("GPL");

