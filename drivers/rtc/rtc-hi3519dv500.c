// SPDX-License-Identifier: GPL-2.0-only
/*
 * RTC driver for the HiSilicon Hi3519DV500 SoC
 *
 * The block keeps binary second, minute, hour and day counters, and
 * compares them against a programmable match value to raise the alarm
 * interrupt.  All registers are byte wide and located at a 4 byte
 * stride; the counters are latched through the LORD register before
 * they are read.
 *
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2016-2024.
 * Copyright (c) 2026 Jun Yan.
 */

#include <linux/bitops.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/math.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/rtc.h>

/*
 * Running time counters
 */
#define HI3519DV500_RTC_10MS_CTR	0x000
#define HI3519DV500_RTC_SEC_CTR	0x004
#define HI3519DV500_RTC_MIN_CTR	0x008
#define HI3519DV500_RTC_HOUR_CTR	0x00c
#define HI3519DV500_RTC_DAYL_CTR	0x010
#define HI3519DV500_RTC_DAYH_CTR	0x014

/*
 * Alarm match registers
 */
#define HI3519DV500_RTC_10MS_MRT	0x018
#define HI3519DV500_RTC_SEC_MRT	0x01c
#define HI3519DV500_RTC_MIN_MRT	0x020
#define HI3519DV500_RTC_HOUR_MRT	0x024
#define HI3519DV500_RTC_DAYL_MRT	0x028
#define HI3519DV500_RTC_DAYH_MRT	0x02c

/*
 * Registers written to load the time counters
 */
#define HI3519DV500_RTC_10MS_LDR	0x030
#define HI3519DV500_RTC_SEC_LDR	0x034
#define HI3519DV500_RTC_MIN_LDR	0x038
#define HI3519DV500_RTC_HOUR_LDR	0x03c
#define HI3519DV500_RTC_DAYL_LDR	0x040
#define HI3519DV500_RTC_DAYH_LDR	0x044

/*
 * Load and lock the time counters
 */
#define HI3519DV500_RTC_LORD	0x048
#define HI3519DV500_RTC_LOAD	BIT(0)		/* self-clearing */
#define HI3519DV500_RTC_LOCK	BIT(1)		/* self-clearing */
#define HI3519DV500_RTC_LOCK_BYPASS	BIT(2)

#define HI3519DV500_RTC_IMSC	0x04c
#define HI3519DV500_RTC_INT_CLR	0x050
#define HI3519DV500_RTC_INT	0x054
#define HI3519DV500_RTC_INT_RAW	0x058
#define HI3519DV500_RTC_ALARM	BIT(0)
#define HI3519DV500_RTC_LOWV	BIT(1)		/* low voltage detected */

#define HI3519DV500_RTC_SAR_CTRL	0x068
#define HI3519DV500_RTC_CLK_CFG	0x06c
#define HI3519DV500_RTC_FREQ_H	0x144
#define HI3519DV500_RTC_FREQ_L	0x148

/*
 * Registers holding the write-protection key; they must be programmed
 * in this order to unlock the configurable registers above.
 */
#define HI3519DV500_RTC_REG_LOCK4	0x19c
#define HI3519DV500_RTC_REG_LOCK3	0x198
#define HI3519DV500_RTC_REG_LOCK2	0x194
#define HI3519DV500_RTC_REG_LOCK1	0x190

/* Vendor BSP power-up defaults */
#define HI3519DV500_RTC_IMSC_DEFAULT	0x04
#define HI3519DV500_RTC_SAR_DEFAULT	0x20
#define HI3519DV500_RTC_CLK_CFG_DEFAULT	0x02
#define HI3519DV500_RTC_FREQ_H_DEFAULT	0x08
#define HI3519DV500_RTC_FREQ_L_DEFAULT	0x1b
#define HI3519DV500_RTC_UNLOCK4	0x5a
#define HI3519DV500_RTC_UNLOCK3	0x5a
#define HI3519DV500_RTC_UNLOCK2	0xab
#define HI3519DV500_RTC_UNLOCK1	0xcd

/*
 * The interrupt status bits are cleared by writing to INT_CLR; the
 * vendor BSP always writes 1.
 */
#define HI3519DV500_RTC_INT_CLR_ALL	BIT(0)

/*
 * Day counters are 16 bit wide, the time wraps after day 0xffff.
 */
#define HI3519DV500_RTC_MAX_DAYS	0xffff
#define HI3519DV500_RTC_MAX_SECS	((u64)(HI3519DV500_RTC_MAX_DAYS + 1) * 86400 - 1)

#define HI3519DV500_RTC_POLL_US	20000
#define HI3519DV500_RTC_TIMEOUT_US	100000

struct hi3519dv500_rtc {
	void __iomem *base;
	struct rtc_device *rtc_dev;
};

static int hi3519dv500_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct hi3519dv500_rtc *priv = dev_get_drvdata(dev);
	u8 val, sec, min, hour, dayl, dayh;
	u64 seconds;
	int ret;

	val = readb(priv->base + HI3519DV500_RTC_INT_RAW);
	if (val & HI3519DV500_RTC_LOWV) {
		/*
		 * The supply voltage dropped low enough that the
		 * counters may have been corrupted; acknowledge the
		 * event and let rtc_valid_tm() judge the result.
		 */
		writeb(HI3519DV500_RTC_INT_CLR_ALL,
		       priv->base + HI3519DV500_RTC_INT_CLR);
	}

	val = readb(priv->base + HI3519DV500_RTC_LORD);
	if (val & HI3519DV500_RTC_LOCK_BYPASS)
		writeb(val & ~HI3519DV500_RTC_LOCK_BYPASS,
		       priv->base + HI3519DV500_RTC_LORD);

	/* Latch the counters for a consistent read */
	val = readb(priv->base + HI3519DV500_RTC_LORD);
	writeb(val | HI3519DV500_RTC_LOCK, priv->base + HI3519DV500_RTC_LORD);

	ret = readb_poll_timeout(priv->base + HI3519DV500_RTC_LORD, val,
				 !(val & HI3519DV500_RTC_LOCK),
				 HI3519DV500_RTC_POLL_US,
				 HI3519DV500_RTC_TIMEOUT_US);
	if (ret)
		return ret;

	sec = readb(priv->base + HI3519DV500_RTC_SEC_CTR);
	min = readb(priv->base + HI3519DV500_RTC_MIN_CTR);
	hour = readb(priv->base + HI3519DV500_RTC_HOUR_CTR);
	dayl = readb(priv->base + HI3519DV500_RTC_DAYL_CTR);
	dayh = readb(priv->base + HI3519DV500_RTC_DAYH_CTR);

	seconds = ((u64)dayl | ((u64)dayh << 8)) * 86400 +
		  hour * 3600 + min * 60 + sec;

	rtc_time64_to_tm(seconds, tm);

	return rtc_valid_tm(tm);
}

static int hi3519dv500_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct hi3519dv500_rtc *priv = dev_get_drvdata(dev);
	time64_t seconds;
	u32 days;
	u8 val;

	seconds = rtc_tm_to_time64(tm);
	days = div_u64(seconds, 86400);

	writeb(0, priv->base + HI3519DV500_RTC_10MS_LDR);
	writeb(tm->tm_sec, priv->base + HI3519DV500_RTC_SEC_LDR);
	writeb(tm->tm_min, priv->base + HI3519DV500_RTC_MIN_LDR);
	writeb(tm->tm_hour, priv->base + HI3519DV500_RTC_HOUR_LDR);
	writeb(days & 0xff, priv->base + HI3519DV500_RTC_DAYL_LDR);
	writeb((days >> 8) & 0xff, priv->base + HI3519DV500_RTC_DAYH_LDR);

	writeb(HI3519DV500_RTC_LOAD, priv->base + HI3519DV500_RTC_LORD);

	return readb_poll_timeout(priv->base + HI3519DV500_RTC_LORD, val,
				  !(val & HI3519DV500_RTC_LOAD),
				  HI3519DV500_RTC_POLL_US,
				  HI3519DV500_RTC_TIMEOUT_US);
}

static int hi3519dv500_rtc_read_alarm(struct device *dev,
				      struct rtc_wkalrm *alrm)
{
	struct hi3519dv500_rtc *priv = dev_get_drvdata(dev);
	u8 sec, min, hour, dayl, dayh;
	u64 seconds;

	sec = readb(priv->base + HI3519DV500_RTC_SEC_MRT);
	min = readb(priv->base + HI3519DV500_RTC_MIN_MRT);
	hour = readb(priv->base + HI3519DV500_RTC_HOUR_MRT);
	dayl = readb(priv->base + HI3519DV500_RTC_DAYL_MRT);
	dayh = readb(priv->base + HI3519DV500_RTC_DAYH_MRT);

	seconds = ((u64)dayl | ((u64)dayh << 8)) * 86400 +
		  hour * 3600 + min * 60 + sec;

	rtc_time64_to_tm(seconds, &alrm->time);

	alrm->enabled = !!(readb(priv->base + HI3519DV500_RTC_IMSC) &
			   HI3519DV500_RTC_ALARM);
	alrm->pending = !!(readb(priv->base + HI3519DV500_RTC_INT) &
			   HI3519DV500_RTC_ALARM);

	return 0;
}

static int hi3519dv500_rtc_set_alarm(struct device *dev,
				     struct rtc_wkalrm *alrm)
{
	struct hi3519dv500_rtc *priv = dev_get_drvdata(dev);
	time64_t seconds;
	u32 days;

	seconds = rtc_tm_to_time64(&alrm->time);
	days = div_u64(seconds, 86400);

	writeb(0, priv->base + HI3519DV500_RTC_10MS_MRT);
	writeb(alrm->time.tm_sec, priv->base + HI3519DV500_RTC_SEC_MRT);
	writeb(alrm->time.tm_min, priv->base + HI3519DV500_RTC_MIN_MRT);
	writeb(alrm->time.tm_hour, priv->base + HI3519DV500_RTC_HOUR_MRT);
	writeb(days & 0xff, priv->base + HI3519DV500_RTC_DAYL_MRT);
	writeb((days >> 8) & 0xff, priv->base + HI3519DV500_RTC_DAYH_MRT);

	return 0;
}

static int hi3519dv500_rtc_alarm_irq_enable(struct device *dev,
					    unsigned int enabled)
{
	struct hi3519dv500_rtc *priv = dev_get_drvdata(dev);
	u8 val;

	val = readb(priv->base + HI3519DV500_RTC_IMSC);
	if (enabled)
		val |= HI3519DV500_RTC_ALARM;
	else
		val &= ~HI3519DV500_RTC_ALARM;
	writeb(val, priv->base + HI3519DV500_RTC_IMSC);

	return 0;
}

static irqreturn_t hi3519dv500_rtc_isr(int irq, void *dev_id)
{
	struct hi3519dv500_rtc *priv = dev_id;
	u8 val;

	val = readb(priv->base + HI3519DV500_RTC_INT);
	if (!(val & HI3519DV500_RTC_ALARM))
		return IRQ_NONE;

	writeb(HI3519DV500_RTC_INT_CLR_ALL,
	       priv->base + HI3519DV500_RTC_INT_CLR);

	rtc_update_irq(priv->rtc_dev, 1, RTC_IRQF | RTC_AF);

	return IRQ_HANDLED;
}

static void hi3519dv500_rtc_hw_init(struct hi3519dv500_rtc *priv)
{
	/* Mask the alarm and low voltage interrupts */
	writeb(HI3519DV500_RTC_IMSC_DEFAULT,
	       priv->base + HI3519DV500_RTC_IMSC);
	writeb(HI3519DV500_RTC_SAR_DEFAULT,
	       priv->base + HI3519DV500_RTC_SAR_CTRL);

	/* Unlock the configurable registers */
	writeb(HI3519DV500_RTC_UNLOCK4,
	       priv->base + HI3519DV500_RTC_REG_LOCK4);
	writeb(HI3519DV500_RTC_UNLOCK3,
	       priv->base + HI3519DV500_RTC_REG_LOCK3);
	writeb(HI3519DV500_RTC_UNLOCK2,
	       priv->base + HI3519DV500_RTC_REG_LOCK2);
	writeb(HI3519DV500_RTC_UNLOCK1,
	       priv->base + HI3519DV500_RTC_REG_LOCK1);

	writeb(HI3519DV500_RTC_CLK_CFG_DEFAULT,
	       priv->base + HI3519DV500_RTC_CLK_CFG);

	/* Default oscillator frequency calibration coefficients */
	writeb(HI3519DV500_RTC_FREQ_H_DEFAULT,
	       priv->base + HI3519DV500_RTC_FREQ_H);
	writeb(HI3519DV500_RTC_FREQ_L_DEFAULT,
	       priv->base + HI3519DV500_RTC_FREQ_L);

	/* Acknowledge a low voltage event left over from a power cycle */
	if (readb(priv->base + HI3519DV500_RTC_INT_RAW) &
	    HI3519DV500_RTC_LOWV)
		writeb(HI3519DV500_RTC_INT_CLR_ALL,
		       priv->base + HI3519DV500_RTC_INT_CLR);
}

static const struct rtc_class_ops hi3519dv500_rtc_ops = {
	.read_time = hi3519dv500_rtc_read_time,
	.set_time = hi3519dv500_rtc_set_time,
	.read_alarm = hi3519dv500_rtc_read_alarm,
	.set_alarm = hi3519dv500_rtc_set_alarm,
	.alarm_irq_enable = hi3519dv500_rtc_alarm_irq_enable,
};

static int hi3519dv500_rtc_probe(struct platform_device *pdev)
{
	struct hi3519dv500_rtc *priv;
	int irq, ret;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	platform_set_drvdata(pdev, priv);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	hi3519dv500_rtc_hw_init(priv);

	priv->rtc_dev = devm_rtc_allocate_device(&pdev->dev);
	if (IS_ERR(priv->rtc_dev))
		return PTR_ERR(priv->rtc_dev);

	priv->rtc_dev->ops = &hi3519dv500_rtc_ops;
	priv->rtc_dev->range_min = 0;
	priv->rtc_dev->range_max = HI3519DV500_RTC_MAX_SECS;
	/* The hardware has no update-ended interrupt */
	clear_bit(RTC_FEATURE_UPDATE_INTERRUPT, priv->rtc_dev->features);

	ret = devm_request_irq(&pdev->dev, irq, hi3519dv500_rtc_isr, 0,
			       dev_name(&pdev->dev), priv);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "cannot request irq\n");

	return devm_rtc_register_device(priv->rtc_dev);
}

static const struct of_device_id hi3519dv500_rtc_of_match[] = {
	{ .compatible = "hisilicon,hi3519dv500-rtc" },
	{ }
};
MODULE_DEVICE_TABLE(of, hi3519dv500_rtc_of_match);

static struct platform_driver hi3519dv500_rtc_driver = {
	.probe = hi3519dv500_rtc_probe,
	.driver = {
		.name = "hi3519dv500-rtc",
		.of_match_table = hi3519dv500_rtc_of_match,
	},
};

module_platform_driver(hi3519dv500_rtc_driver);

MODULE_DESCRIPTION("HiSilicon Hi3519DV500 RTC driver");
MODULE_AUTHOR("Jun Yan <jerrysteve1101@gmail.com>");
MODULE_LICENSE("GPL");
