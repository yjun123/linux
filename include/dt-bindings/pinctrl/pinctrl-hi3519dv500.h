/* SPDX-License-Identifier: (GPL-2.0-only OR BSD-2-Clause) */
/*
 * HiSilicon Hi3519DV500 pinctrl definitions
 *
 * Copyright (c) 2026 HiSilicon Co., Ltd.
 */

#ifndef _DT_BINDINGS_PINCTRL_HI3519DV500_H
#define _DT_BINDINGS_PINCTRL_HI3519DV500_H

/*
 * The "pinmux" property of a pin control group takes one 32-bit value per
 * pin, encoding the pin's IO config register offset within its ioconfig
 * block and the function number to select:
 *
 *   bits 15:0  - byte offset of the pin's IO config register
 *   bits 19:16 - function number (register func field, bit 3:0)
 */
#define HI3519DV500_PINMUX(offset, func)	\
	(((offset) & 0xffff) | (((func) & 0xf) << 16))

#endif /* _DT_BINDINGS_PINCTRL_HI3519DV500_H */
