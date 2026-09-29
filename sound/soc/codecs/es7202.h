/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2020 Everest Semiconductor Co., Ltd.
 *
 * Author: David Yang <yangxiaohua@everest-semi.com>
 */

#ifndef _ES7202_H
#define _ES7202_H

#include <linux/bits.h>

/*
 * ES7202 register space
 */
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

/*
 * Field definitions
 */

/* ES7202_PDM_INF_CTL: bit[1:0] mute/enable the PDM output */
#define ES7202_PDM_INF_CTL_MUTE	GENMASK(1, 0)

#endif
