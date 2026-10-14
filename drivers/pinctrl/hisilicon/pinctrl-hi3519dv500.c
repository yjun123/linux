// SPDX-License-Identifier: GPL-2.0
/*
 * Pin controller driver for HiSilicon Hi3519DV500 SoC
 *
 * Copyright (c) 2026 HiSilicon Co., Ltd.
 */

#include <linux/bits.h>
#include <linux/bsearch.h>
#include <linux/cleanup.h>
#include <linux/io.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

#include <linux/pinctrl/pinconf-generic.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>

#include "../core.h"
#include "../pinconf.h"
#include "../pinmux.h"
#include "../pinctrl-utils.h"

/*
 * IO config register field layout (32 bit):
 *
 *   bit  3:0   function select
 *   bit  7:4   drive strength level
 *   bit  9:8   pull up/down (encoding depends on the IO domain)
 *   bit    10  slew rate (0 = fast, 1 = slow)
 *   bit 14:11  reserved, must be preserved on write
 */
#define HI3519DV500_FUNC_MASK		GENMASK(3, 0)
#define HI3519DV500_DRV_MASK		GENMASK(7, 4)
#define HI3519DV500_PULL_MASK		GENMASK(9, 8)
#define HI3519DV500_SLEW_SLOW		BIT(10)

/* pull field values, encoding depends on the IO domain */
#define HI3519DV500_PULL_COMBO_DOWN	0x1
#define HI3519DV500_PULL_COMBO_UP	0x3
#define HI3519DV500_PULL_SPLIT_DOWN	0x2
#define HI3519DV500_PULL_SPLIT_UP	0x1

/*
 * The "pinmux" property of each group node carries one 32-bit value per
 * pin, as encoded by HI3519DV500_PINMUX() from
 * <dt-bindings/pinctrl/pinctrl-hi3519dv500.h>: the IO config register
 * byte offset in bits 15:0 and the function number in bits 19:16.
 */
#define HI3519DV500_PINMUX_OFFSET(x)	((x) & 0xffff)
#define HI3519DV500_PINMUX_FUNC(x)	(((x) >> 16) & 0xf)

/* IO domains of the SoC */
enum hi3519dv500_io_type {
	HI3519DV500_IO_EMMC,
	HI3519DV500_IO_2,
	HI3519DV500_IO_6,
	HI3519DV500_IO_7_1,
	HI3519DV500_IO_7_2,
};

struct hi3519dv500_pin {
	const char *name;
	u16 offset;
	u8 io_type;
};

struct hi3519dv500_pinctrl_data {
	const struct hi3519dv500_pin *pins;
	unsigned int npins;
};

/* Per-domain pinconf capabilities, indexed by hi3519dv500_io_type. */
struct hi3519dv500_io_cap {
	bool has_pull;
	bool combo_pull;	/* bit 9:8 combined vs. independent up/down */
	bool has_slew;
	u8 drv_max;
};

static const struct hi3519dv500_io_cap hi3519dv500_io_caps[] = {
	[HI3519DV500_IO_EMMC]	= { true,  true,  true,  15 },
	[HI3519DV500_IO_2]	= { true,  false, true,   3 },
	[HI3519DV500_IO_6]	= { true,  true,  true,   7 },
	[HI3519DV500_IO_7_1]	= { false, false, false, 3 },
	[HI3519DV500_IO_7_2]	= { false, false, false, 1 },
};

static const struct hi3519dv500_pin hi3519dv500_pins_ioconfig0[] = {
	{ "GPIO0_0",		0x000,	HI3519DV500_IO_2 },
	{ "SENSOR1_CLK",	0x004,	HI3519DV500_IO_6 },
	{ "SENSOR1_RSTN",	0x008,	HI3519DV500_IO_2 },
	{ "SENSOR1_HS",	0x00c,	HI3519DV500_IO_6 },
	{ "SENSOR1_VS",	0x010,	HI3519DV500_IO_6 },
	{ "SVB_PWM2",		0x014,	HI3519DV500_IO_2 },
	{ "USB_OVRCUR",	0x018,	HI3519DV500_IO_2 },
	{ "TEST_MODE",		0x01c,	HI3519DV500_IO_2 },
	{ "SVB_PWM0",		0x020,	HI3519DV500_IO_2 },
	{ "SPI1_SCLK",		0x024,	HI3519DV500_IO_EMMC },
	{ "SPI1_SDO",		0x028,	HI3519DV500_IO_EMMC },
	{ "SPI1_SDI",		0x02c,	HI3519DV500_IO_EMMC },
	{ "SPI1_CSN",		0x030,	HI3519DV500_IO_EMMC },
	{ "EMMC_CLK",		0x034,	HI3519DV500_IO_EMMC },
	{ "EMMC_CMD",		0x038,	HI3519DV500_IO_EMMC },
	{ "EMMC_DATA0",	0x03c,	HI3519DV500_IO_EMMC },
	{ "EMMC_DATA1",	0x040,	HI3519DV500_IO_EMMC },
	{ "EMMC_DATA2",	0x044,	HI3519DV500_IO_EMMC },
	{ "EMMC_DATA3",	0x048,	HI3519DV500_IO_EMMC },
	{ "EMMC_RST_N",	0x04c,	HI3519DV500_IO_EMMC },
	{ "EMMC_DS",		0x050,	HI3519DV500_IO_EMMC },
	{ "UART0_RXD",		0x054,	HI3519DV500_IO_EMMC },
	{ "UART0_TXD",		0x058,	HI3519DV500_IO_2 },
};

static const struct hi3519dv500_pin hi3519dv500_pins_ioconfig1[] = {
	{ "UART2_RXD",		0x000,	HI3519DV500_IO_EMMC },
	{ "UART2_TXD",		0x004,	HI3519DV500_IO_2 },
	{ "UART2_RTSN",	0x008,	HI3519DV500_IO_2 },
	{ "UART2_CTSN",	0x00c,	HI3519DV500_IO_2 },
	{ "EPHY_CLK",		0x010,	HI3519DV500_IO_2 },
	{ "EPHY_RSTN",		0x014,	HI3519DV500_IO_2 },
	{ "POR_ENABLE",	0x018,	HI3519DV500_IO_2 },
	{ "MDCK",		0x01c,	HI3519DV500_IO_2 },
	{ "MDIO",		0x020,	HI3519DV500_IO_2 },
	{ "RSTN",		0x024,	HI3519DV500_IO_2 },
	{ "RGMII_RXD3",	0x028,	HI3519DV500_IO_6 },
	{ "RGMII_RXD2",	0x02c,	HI3519DV500_IO_6 },
	{ "RGMII_RXD1",	0x030,	HI3519DV500_IO_6 },
	{ "RGMII_RXD0",	0x034,	HI3519DV500_IO_6 },
	{ "RGMII_RXDV",	0x038,	HI3519DV500_IO_6 },
	{ "RGMII_RXCK",	0x03c,	HI3519DV500_IO_6 },
	{ "RGMII_TXD3",	0x040,	HI3519DV500_IO_6 },
	{ "RGMII_TXD2",	0x044,	HI3519DV500_IO_6 },
	{ "RGMII_TXD1",	0x048,	HI3519DV500_IO_6 },
	{ "RGMII_TXD0",	0x04c,	HI3519DV500_IO_6 },
	{ "RGMII_TXEN",	0x050,	HI3519DV500_IO_6 },
	{ "RGMII_TXCKOUT",	0x054,	HI3519DV500_IO_6 },
	{ "JTAG_TRSTN",	0x058,	HI3519DV500_IO_2 },
	{ "JTAG_TCK",		0x05c,	HI3519DV500_IO_2 },
	{ "JTAG_TMS",		0x060,	HI3519DV500_IO_EMMC },
	{ "JTAG_TDO",		0x064,	HI3519DV500_IO_2 },
	{ "JTAG_TDI",		0x068,	HI3519DV500_IO_EMMC },
	{ "I2C0_SDA",		0x06c,	HI3519DV500_IO_6 },
	{ "I2C0_SCL",		0x070,	HI3519DV500_IO_EMMC },
	{ "SVB_PWM1",		0x074,	HI3519DV500_IO_2 },
	{ "DSI_D3N",		0x078,	HI3519DV500_IO_7_1 },
	{ "DSI_D3P",		0x07c,	HI3519DV500_IO_7_1 },
	{ "DSI_D1N",		0x080,	HI3519DV500_IO_7_1 },
	{ "DSI_D1P",		0x084,	HI3519DV500_IO_7_1 },
	{ "DSI_CKN",		0x088,	HI3519DV500_IO_7_1 },
	{ "DSI_CKP",		0x08c,	HI3519DV500_IO_7_1 },
	{ "DSI_D0N",		0x090,	HI3519DV500_IO_7_1 },
	{ "DSI_D0P",		0x094,	HI3519DV500_IO_7_1 },
	{ "DSI_D2N",		0x098,	HI3519DV500_IO_7_1 },
	{ "DSI_D2P",		0x09c,	HI3519DV500_IO_7_1 },
	{ "SDIO1_CDATA0",	0x0a0,	HI3519DV500_IO_EMMC },
	{ "SDIO1_CDATA1",	0x0a4,	HI3519DV500_IO_EMMC },
	{ "SDIO1_CDATA2",	0x0a8,	HI3519DV500_IO_EMMC },
	{ "SDIO1_CDATA3",	0x0ac,	HI3519DV500_IO_EMMC },
	{ "SDIO1_CCLK_OUT",	0x0b0,	HI3519DV500_IO_EMMC },
	{ "SDIO1_CCMD",	0x0b4,	HI3519DV500_IO_EMMC },
	{ "SDIO0_CCMD",	0x0b8,	HI3519DV500_IO_EMMC },
	{ "SDIO0_CDATA0",	0x0bc,	HI3519DV500_IO_EMMC },
	{ "SDIO0_CDATA1",	0x0c0,	HI3519DV500_IO_EMMC },
	{ "SDIO0_CDATA2",	0x0c4,	HI3519DV500_IO_EMMC },
	{ "SDIO0_CDATA3",	0x0c8,	HI3519DV500_IO_EMMC },
	{ "SDIO0_CCLK_OUT",	0x0cc,	HI3519DV500_IO_EMMC },
	{ "SDIO0_CARD_DETECT",	0x0d0,	HI3519DV500_IO_2 },
	{ "SDIO0_CARD_POWER_EN_N", 0x0d4,	HI3519DV500_IO_2 },
	{ "UART1_RXD",		0x0d8,	HI3519DV500_IO_EMMC },
	{ "UART1_TXD",		0x0dc,	HI3519DV500_IO_2 },
	{ "UART1_RTSN",	0x0e0,	HI3519DV500_IO_2 },
	{ "UART1_CTSN",	0x0e4,	HI3519DV500_IO_2 },
};

static const struct hi3519dv500_pin hi3519dv500_pins_ioconfig2[] = {
	{ "LSADC_CH0",		0x000,	HI3519DV500_IO_2 },
	{ "LSADC_CH1",		0x004,	HI3519DV500_IO_2 },
	{ "LSADC_CH2",		0x008,	HI3519DV500_IO_2 },
	{ "LSADC_CH3",		0x00c,	HI3519DV500_IO_2 },
	{ "USB_VBUS",		0x010,	HI3519DV500_IO_2 },
	{ "USB_PWREN",		0x014,	HI3519DV500_IO_2 },
	{ "SYS_RSTN_OUT",	0x018,	HI3519DV500_IO_2 },
	{ "PWM_OUT0_0_P",	0x01c,	HI3519DV500_IO_6 },
	{ "PWM_OUT1_0_P",	0x020,	HI3519DV500_IO_2 },
	{ "SPI0_CSN1",		0x024,	HI3519DV500_IO_6 },
	{ "SPI0_CSN0",		0x028,	HI3519DV500_IO_6 },
	{ "SPI0_SDI",		0x02c,	HI3519DV500_IO_6 },
	{ "SPI0_SDO",		0x030,	HI3519DV500_IO_6 },
	{ "SPI0_SCLK",		0x034,	HI3519DV500_IO_EMMC },
	{ "SENSOR0_HS",	0x038,	HI3519DV500_IO_6 },
	{ "SENSOR0_VS",	0x03c,	HI3519DV500_IO_6 },
	{ "SENSOR0_RSTN",	0x040,	HI3519DV500_IO_6 },
	{ "SENSOR0_CLK",	0x044,	HI3519DV500_IO_EMMC },
	{ "MIPI_RX0_D0N",	0x048,	HI3519DV500_IO_7_2 },
	{ "MIPI_RX0_D0P",	0x04c,	HI3519DV500_IO_7_2 },
	{ "MIPI_RX0_D2N",	0x050,	HI3519DV500_IO_7_2 },
	{ "MIPI_RX0_D2P",	0x054,	HI3519DV500_IO_7_2 },
	{ "MIPI_RX0_CK0N",	0x058,	HI3519DV500_IO_7_2 },
	{ "MIPI_RX0_CK0P",	0x05c,	HI3519DV500_IO_7_2 },
	{ "MIPI_RX0_CK1N",	0x060,	HI3519DV500_IO_7_2 },
	{ "MIPI_RX0_CK1P",	0x064,	HI3519DV500_IO_7_2 },
	{ "MIPI_RX0_D1N",	0x068,	HI3519DV500_IO_7_2 },
	{ "MIPI_RX0_D1P",	0x06c,	HI3519DV500_IO_7_2 },
	{ "MIPI_RX0_D3N",	0x070,	HI3519DV500_IO_7_2 },
	{ "MIPI_RX0_D3P",	0x074,	HI3519DV500_IO_7_2 },
	{ "MIPI_RX1_D0N",	0x078,	HI3519DV500_IO_7_2 },
	{ "MIPI_RX1_D0P",	0x07c,	HI3519DV500_IO_7_2 },
	{ "MIPI_RX1_D2N",	0x080,	HI3519DV500_IO_7_2 },
	{ "MIPI_RX1_D2P",	0x084,	HI3519DV500_IO_7_2 },
	{ "MIPI_RX1_CK0N",	0x088,	HI3519DV500_IO_7_2 },
	{ "MIPI_RX1_CK0P",	0x08c,	HI3519DV500_IO_7_2 },
	{ "MIPI_RX1_CK1N",	0x090,	HI3519DV500_IO_7_2 },
	{ "MIPI_RX1_CK1P",	0x094,	HI3519DV500_IO_7_2 },
	{ "MIPI_RX1_D1N",	0x098,	HI3519DV500_IO_7_2 },
	{ "MIPI_RX1_D1P",	0x09c,	HI3519DV500_IO_7_2 },
	{ "MIPI_RX1_D3N",	0x0a0,	HI3519DV500_IO_7_2 },
	{ "MIPI_RX1_D3P",	0x0a4,	HI3519DV500_IO_7_2 },
};

static const struct hi3519dv500_pinctrl_data hi3519dv500_ioconfig0_data = {
	.pins = hi3519dv500_pins_ioconfig0,
	.npins = ARRAY_SIZE(hi3519dv500_pins_ioconfig0),
};

static const struct hi3519dv500_pinctrl_data hi3519dv500_ioconfig1_data = {
	.pins = hi3519dv500_pins_ioconfig1,
	.npins = ARRAY_SIZE(hi3519dv500_pins_ioconfig1),
};

static const struct hi3519dv500_pinctrl_data hi3519dv500_ioconfig2_data = {
	.pins = hi3519dv500_pins_ioconfig2,
	.npins = ARRAY_SIZE(hi3519dv500_pins_ioconfig2),
};

struct hi3519dv500_mux {
	unsigned int pin;
	u32 func;
};

struct hi3519dv500_pinctrl {
	struct device *dev;
	const struct hi3519dv500_pinctrl_data *data;
	void __iomem *base;
	raw_spinlock_t lock;	/* protects the IO config registers */
	struct mutex mutex;	/* serializes group/function registration */
	struct pinctrl_desc pdesc;
};

static int hi3519dv500_pin_cmp(const void *key, const void *pivot)
{
	const struct hi3519dv500_pin *pin = pivot;
	unsigned long offset = (unsigned long)key;

	return offset - pin->offset;
}

static const struct hi3519dv500_pin *
hi3519dv500_find_pin(const struct hi3519dv500_pinctrl_data *data,
		     unsigned int offset)
{
	/* the pin tables are sorted by register offset */
	return bsearch((void *)(uintptr_t)offset, data->pins, data->npins,
		       sizeof(*data->pins), hi3519dv500_pin_cmp);
}

static int hi3519dv500_dt_node_to_map(struct pinctrl_dev *pctldev,
				      struct device_node *np,
				      struct pinctrl_map **maps,
				      unsigned int *num_maps)
{
	struct hi3519dv500_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctldev);
	struct device *dev = pctrl->dev;
	const struct hi3519dv500_pin *pin;
	struct hi3519dv500_mux *muxes;
	struct pinctrl_map *map;
	const char *grpname;
	const char **grpnames;
	unsigned int *pins;
	unsigned int nmaps = 0;
	int npins, i, ret;

	/*
	 * In the board device tree each group node is a direct child of
	 * the ioconfig node and is referenced by phandle from a
	 * pinctrl-{0,1,...} property, so the group node itself describes
	 * a single pin group with one function of the same name.
	 */
	npins = of_property_count_u32_elems(np, "pinmux");
	if (npins < 1) {
		dev_err(dev, "invalid pinctrl group %pOFn\n", np);
		return -EINVAL;
	}

	pins = devm_kcalloc(dev, npins, sizeof(*pins), GFP_KERNEL);
	if (!pins)
		return -ENOMEM;

	muxes = devm_kcalloc(dev, npins, sizeof(*muxes), GFP_KERNEL);
	if (!muxes)
		return -ENOMEM;

	grpname = devm_kstrdup(dev, np->name, GFP_KERNEL);
	if (!grpname)
		return -ENOMEM;

	/* the groups array is not copied by pinmux_generic_add_function() */
	grpnames = devm_kcalloc(dev, 1, sizeof(*grpnames), GFP_KERNEL);
	if (!grpnames)
		return -ENOMEM;

	map = kzalloc_objs(*map, 2);
	if (!map)
		return -ENOMEM;

	for (i = 0; i < npins; i++) {
		u32 val;
		u16 offset;

		ret = of_property_read_u32_index(np, "pinmux", i, &val);
		if (ret)
			goto dt_failed;

		offset = HI3519DV500_PINMUX_OFFSET(val);
		pin = hi3519dv500_find_pin(pctrl->data, offset);
		if (!pin) {
			dev_err(dev, "%pOFn: no pin at offset 0x%04x\n",
				np, offset);
			ret = -EINVAL;
			goto dt_failed;
		}

		pins[i] = pin - pctrl->data->pins;
		muxes[i].pin = pins[i];
		muxes[i].func = HI3519DV500_PINMUX_FUNC(val);
	}

	map[nmaps].type = PIN_MAP_TYPE_MUX_GROUP;
	map[nmaps].data.mux.function = grpname;
	map[nmaps].data.mux.group = grpname;
	nmaps++;

	ret = pinconf_generic_parse_dt_config(np, pctldev,
					      &map[nmaps].data.configs.configs,
					      &map[nmaps].data.configs.num_configs);
	if (ret) {
		dev_err(dev, "failed to parse pin config of group %pOFn: %d\n",
			np, ret);
		goto dt_failed;
	}

	/* set the type first so the error path frees the parsed configs */
	if (map[nmaps].data.configs.num_configs) {
		map[nmaps].type = PIN_MAP_TYPE_CONFIGS_GROUP;
		map[nmaps].data.configs.group_or_pin = grpname;
		nmaps++;
	}

	guard(mutex)(&pctrl->mutex);

	ret = pinctrl_generic_add_group(pctldev, grpname, pins, npins, muxes);
	if (ret < 0) {
		dev_err(dev, "failed to add group %s: %d\n", grpname, ret);
		goto dt_failed;
	}

	grpnames[0] = grpname;

	ret = pinmux_generic_add_function(pctldev, grpname, grpnames, 1, NULL);
	if (ret < 0) {
		dev_err(dev, "failed to add function %s: %d\n", grpname, ret);
		goto dt_failed;
	}

	*maps = map;
	*num_maps = nmaps;

	return 0;

dt_failed:
	pinctrl_utils_free_map(pctldev, map, nmaps);
	return ret;
}

static const struct pinctrl_ops hi3519dv500_pctl_ops = {
	.get_groups_count = pinctrl_generic_get_group_count,
	.get_group_name = pinctrl_generic_get_group_name,
	.get_group_pins = pinctrl_generic_get_group_pins,
	.dt_node_to_map = hi3519dv500_dt_node_to_map,
	.dt_free_map = pinctrl_utils_free_map,
};

static int hi3519dv500_pmx_set_mux(struct pinctrl_dev *pctldev,
				   unsigned int fsel, unsigned int gsel)
{
	struct hi3519dv500_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctldev);
	const struct hi3519dv500_mux *muxes;
	const struct group_desc *group;
	unsigned int i;

	group = pinctrl_generic_get_group(pctldev, gsel);
	if (!group)
		return -EINVAL;

	muxes = group->data;

	guard(raw_spinlock_irqsave)(&pctrl->lock);

	for (i = 0; i < group->grp.npins; i++) {
		unsigned int offset = pctrl->data->pins[muxes[i].pin].offset;
		u32 val = readl(pctrl->base + offset);

		val = (val & ~HI3519DV500_FUNC_MASK) |
		      FIELD_PREP(HI3519DV500_FUNC_MASK, muxes[i].func);
		writel(val, pctrl->base + offset);
	}

	return 0;
}

static const struct pinmux_ops hi3519dv500_pmx_ops = {
	.get_functions_count = pinmux_generic_get_function_count,
	.get_function_name = pinmux_generic_get_function_name,
	.get_function_groups = pinmux_generic_get_function_groups,
	.set_mux = hi3519dv500_pmx_set_mux,
};

static int hi3519dv500_pinconf_apply(struct pinctrl_dev *pctldev,
				     unsigned int pin_id, u32 value, u32 mask)
{
	struct hi3519dv500_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctldev);
	const struct hi3519dv500_pin *pin = &pctrl->data->pins[pin_id];
	u32 val;

	guard(raw_spinlock_irqsave)(&pctrl->lock);

	val = readl(pctrl->base + pin->offset);
	writel((val & ~mask) | (value & mask), pctrl->base + pin->offset);

	return 0;
}

static int hi3519dv500_pinconf_set(struct pinctrl_dev *pctldev,
				   unsigned int pin_id,
				   unsigned long *configs,
				   unsigned int num_configs)
{
	struct hi3519dv500_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctldev);
	const struct hi3519dv500_pin *pin = &pctrl->data->pins[pin_id];
	u32 value = 0, mask = 0;
	unsigned int i;

	for (i = 0; i < num_configs; i++) {
		enum pin_config_param param = pinconf_to_config_param(configs[i]);
		u32 arg = pinconf_to_config_argument(configs[i]);
		const struct hi3519dv500_io_cap *cap =
			&hi3519dv500_io_caps[pin->io_type];

		switch (param) {
		case PIN_CONFIG_BIAS_DISABLE:
			if (!cap->has_pull)
				return -ENOTSUPP;
			mask |= HI3519DV500_PULL_MASK;
			break;
		case PIN_CONFIG_BIAS_PULL_UP:
			if (!cap->has_pull)
				return -ENOTSUPP;
			mask |= HI3519DV500_PULL_MASK;
			value |= FIELD_PREP(HI3519DV500_PULL_MASK,
					    cap->combo_pull ?
					    HI3519DV500_PULL_COMBO_UP :
					    HI3519DV500_PULL_SPLIT_UP);
			break;
		case PIN_CONFIG_BIAS_PULL_DOWN:
			if (!cap->has_pull)
				return -ENOTSUPP;
			mask |= HI3519DV500_PULL_MASK;
			value |= FIELD_PREP(HI3519DV500_PULL_MASK,
					    cap->combo_pull ?
					    HI3519DV500_PULL_COMBO_DOWN :
					    HI3519DV500_PULL_SPLIT_DOWN);
			break;
		case PIN_CONFIG_DRIVE_STRENGTH:
			if (arg > cap->drv_max)
				return -EINVAL;
			mask |= HI3519DV500_DRV_MASK;
			value |= FIELD_PREP(HI3519DV500_DRV_MASK, arg);
			break;
		case PIN_CONFIG_SLEW_RATE:
			if (!cap->has_slew || arg > 1)
				return -ENOTSUPP;
			mask |= HI3519DV500_SLEW_SLOW;
			value |= FIELD_PREP(HI3519DV500_SLEW_SLOW, arg);
			break;
		default:
			return -ENOTSUPP;
		}
	}

	return hi3519dv500_pinconf_apply(pctldev, pin_id, value, mask);
}

static int hi3519dv500_pinconf_group_set(struct pinctrl_dev *pctldev,
					 unsigned int gsel,
					 unsigned long *configs,
					 unsigned int num_configs)
{
	const struct group_desc *group;
	unsigned int i;
	int ret;

	group = pinctrl_generic_get_group(pctldev, gsel);
	if (!group)
		return -EINVAL;

	/*
	 * Configs have to be computed per pin: a group may contain pins
	 * from different IO domains with different pull encodings and
	 * drive strength ranges.
	 */
	for (i = 0; i < group->grp.npins; i++) {
		ret = hi3519dv500_pinconf_set(pctldev, group->grp.pins[i],
					      configs, num_configs);
		if (ret)
			return ret;
	}

	return 0;
}

static const struct pinconf_ops hi3519dv500_pconf_ops = {
	.pin_config_set = hi3519dv500_pinconf_set,
	.pin_config_group_set = hi3519dv500_pinconf_group_set,
};

static int hi3519dv500_pinctrl_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct hi3519dv500_pinctrl *pctrl;
	const struct hi3519dv500_pinctrl_data *data;
	struct pinctrl_pin_desc *pin_desc;
	struct pinctrl_dev *pctl;
	unsigned int i;
	int ret;

	data = device_get_match_data(dev);
	if (!data)
		return -ENODEV;

	pctrl = devm_kzalloc(dev, sizeof(*pctrl), GFP_KERNEL);
	if (!pctrl)
		return -ENOMEM;

	pctrl->dev = dev;
	pctrl->data = data;
	pctrl->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(pctrl->base))
		return PTR_ERR(pctrl->base);

	raw_spin_lock_init(&pctrl->lock);
	mutex_init(&pctrl->mutex);

	pin_desc = devm_kcalloc(dev, data->npins, sizeof(*pin_desc), GFP_KERNEL);
	if (!pin_desc)
		return -ENOMEM;

	for (i = 0; i < data->npins; i++) {
		pin_desc[i].number = i;
		pin_desc[i].name = data->pins[i].name;
	}

	pctrl->pdesc.name = dev_name(dev);
	pctrl->pdesc.pins = pin_desc;
	pctrl->pdesc.npins = data->npins;
	pctrl->pdesc.pctlops = &hi3519dv500_pctl_ops;
	pctrl->pdesc.pmxops = &hi3519dv500_pmx_ops;
	pctrl->pdesc.confops = &hi3519dv500_pconf_ops;
	pctrl->pdesc.owner = THIS_MODULE;

	ret = devm_pinctrl_register_and_init(dev, &pctrl->pdesc, pctrl, &pctl);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to register pin controller\n");

	return pinctrl_enable(pctl);
}

static const struct of_device_id hi3519dv500_pinctrl_of_match[] = {
	{
		.compatible = "hisilicon,hi3519dv500-ioconfig0",
		.data = &hi3519dv500_ioconfig0_data,
	}, {
		.compatible = "hisilicon,hi3519dv500-ioconfig1",
		.data = &hi3519dv500_ioconfig1_data,
	}, {
		.compatible = "hisilicon,hi3519dv500-ioconfig2",
		.data = &hi3519dv500_ioconfig2_data,
	}, { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, hi3519dv500_pinctrl_of_match);

static struct platform_driver hi3519dv500_pinctrl_driver = {
	.probe = hi3519dv500_pinctrl_probe,
	.driver = {
		.name = "pinctrl-hi3519dv500",
		.of_match_table = hi3519dv500_pinctrl_of_match,
	},
};
module_platform_driver(hi3519dv500_pinctrl_driver);

MODULE_AUTHOR("Jun Yan <jerrysteve1101@gmail.com>");
MODULE_DESCRIPTION("Pin controller driver for HiSilicon Hi3519DV500 SoC");
MODULE_LICENSE("GPL");
