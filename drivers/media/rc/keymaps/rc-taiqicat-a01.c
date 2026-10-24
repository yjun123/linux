// SPDX-License-Identifier: GPL-2.0+
// Copyright (c) 2026 Jun Yan <jerrysteve1101@gmail.com>

#include <media/rc-map.h>
#include <linux/module.h>

/*
 * Keymap for the TaiqiCat (TQC) A01 remote control.
 *
 * The remote uses extended NEC with address 0x9f and command codes as
 * found in the vendor keylayout shipped in the stock firmware.
 */

static struct rc_map_table taiqicat_a01_table[] = {
	{ 0x9f0057, KEY_POWER },
	{ 0x9f004f, KEY_BACK },
	{ 0x9f0047, KEY_HOME },
	{ 0x9f0016, KEY_MENU },
	{ 0x9f000d, KEY_SEARCH },

	{ 0x9f0043, KEY_UP },
	{ 0x9f000a, KEY_DOWN },
	{ 0x9f0006, KEY_LEFT },
	{ 0x9f000e, KEY_RIGHT },
	{ 0x9f0002, KEY_OK },

	{ 0x9f0000, KEY_VOLUMEUP },
	{ 0x9f005d, KEY_VOLUMEDOWN },
	{ 0x9f005c, KEY_MUTE },

	{ 0x9f001a, KEY_1 },
	{ 0x9f0001, KEY_2 },
	{ 0x9f0045, KEY_3 },
	{ 0x9f0059, KEY_4 },
	{ 0x9f004d, KEY_5 },
	{ 0x9f0040, KEY_6 },
	{ 0x9f0052, KEY_7 },
	{ 0x9f0053, KEY_8 },
	{ 0x9f0041, KEY_9 },
	{ 0x9f005a, KEY_0 },

	{ 0x9f0050, KEY_PLAYPAUSE },
	{ 0x9f0013, KEY_STOP },
	{ 0x9f000f, KEY_PREVIOUS },
	{ 0x9f000b, KEY_NEXT },
	{ 0x9f0007, KEY_REWIND },
	{ 0x9f0003, KEY_FASTFORWARD },

	{ 0x9f0046, KEY_SUBTITLE },
	{ 0x9f0044, KEY_AUDIO },
	{ 0x9f0004, KEY_TV },
	{ 0x9f0009, KEY_SETUP },
	{ 0x9f0010, KEY_INFO },
	{ 0x9f0054, KEY_VIDEO },
	{ 0x9f005b, KEY_APPSELECT },
	{ 0x9f0017, KEY_WWW },

	{ 0x9f0055, KEY_RED },
	{ 0x9f004e, KEY_GREEN },
	{ 0x9f0049, KEY_YELLOW },
	{ 0x9f0048, KEY_BLUE },

	{ 0x9f0015, KEY_FAVORITES },
	{ 0x9f0005, KEY_HELP },
	{ 0x9f0008, KEY_MEDIA_REPEAT },
	{ 0x9f0014, KEY_FULL_SCREEN },
	{ 0x9f000c, KEY_ZOOMIN },
	{ 0x9f0058, KEY_ZOOMOUT },
	{ 0x9f0011, KEY_CONTEXT_MENU },
};

static struct rc_map_list taiqicat_a01_map = {
	.map = {
		.scan     = taiqicat_a01_table,
		.size     = ARRAY_SIZE(taiqicat_a01_table),
		.rc_proto = RC_PROTO_NECX,
		.name     = RC_MAP_TAIQICAT_A01,
	}
};

static int __init init_rc_map_taiqicat_a01(void)
{
	return rc_map_register(&taiqicat_a01_map);
}

static void __exit exit_rc_map_taiqicat_a01(void)
{
	rc_map_unregister(&taiqicat_a01_map);
}

module_init(init_rc_map_taiqicat_a01)
module_exit(exit_rc_map_taiqicat_a01)

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jun Yan <jerrysteve1101@gmail.com>");
MODULE_DESCRIPTION("TaiqiCat (TQC) A01 remote controller keytable");
