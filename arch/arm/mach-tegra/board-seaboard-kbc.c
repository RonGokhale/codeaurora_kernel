/*
 * arch/arm/mach-tegra/board-seaboard-kbc.c
 *
 * Copyright (c) 2010, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */
#if CONFIG_KEYBOARD_TEGRA

#include <linux/platform_device.h>
#include <linux/input.h>
#include <mach/kbc.h>
#include <mach/iomap.h>
#include <mach/irqs.h>

/* The total number of soc scan codes will be (first - last) */
#define NORMAL_SCAN_CODE_TABLE_FIRST	KBC_NORMAL_KEY_CODE_BASE
#define NORMAL_SCAN_CODE_TABLE_LAST	(KBC_NORMAL_KEY_CODE_BASE + 0x7F)

#define FUNCTION_SCAN_CODE_TABLE_FIRST	KBC_FUNCTION_KEY_CODE_BASE
#define FUNCTION_SCAN_CODE_TABLE_LAST	(KBC_FUNCTION_KEY_CODE_BASE + 0x7F)

/*
 * @brief This is the actual Scan-code-to-VKey mapping table. For new layouts
 *        this is the only structure which needs to be modified to return the
 *        proper vkey depending on the scan code.
 */

static int keymap_normal[] = {
	/*
	 * Row 0 Unused, Unused, 'W', 'S', 'A', 'Z', Unused, Function,
	 * Row 1 Unused, Unused, Unused, Unused, Unused, Unused, Unused, Menu
	 * Row 2 Unused, Unused, Unused, Unused, Unused, Unused, Alt, Alt2
	 * Row 3 '5', '4', 'R', 'E', 'F', 'D', 'X', Unused,
	 * Row 4 '7', '6', 'T', 'H', 'G', 'V', 'C', SPACEBAR,
	 * Row 5 '9', '8', 'U', 'Y', 'J', 'N', 'B', '|\',
	 * Row 6 Minus, '0', 'O', 'I', 'L', 'K', '<', M,
	 * Row 7 Unused, '+', '}]', '#', Unused, Unused, Unused, Menu,
	 * Row 8 Unused, Unused, Unused, Unused, SHIFT, SHIFT, Unused, Unused,
	 * Row 9 Unused, Unused, Unused, Unused, Unused, Ctrl, Unused, Ctrl,
	 * Row A Unused, Unused, Unused, Unused, Unused, Unused, Unused, Unused,
	 * Row B '{[', 'P', '"', ':;', '/?, '>', Unused, Unused,
	 * Row C 'F10', 'F9', 'BckSpc', '3', '2', Up, Prntscr, Pause
	 * Row D INS, DEL, Unused, Pgup, PgDn, right, Down, Left,
	 * Row E F11, F12, F8, 'Q', F4, F3, '1', F7,
	 * Row F ESC, '~', F5, TAB, F1, F2, CAPLOCK, F6,
	 */
	KEY_RESERVED, KEY_RESERVED, KEY_W, KEY_S,
		KEY_A, KEY_Z, KEY_RESERVED, KEY_FN,
	KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
		KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_MENU,
	KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
		KEY_RESERVED, KEY_RESERVED, KEY_LEFTALT, KEY_RIGHTALT,
	KEY_5, KEY_4, KEY_R, KEY_E,
		KEY_F, KEY_D, KEY_X, KEY_RESERVED,
	KEY_7, KEY_6, KEY_T, KEY_H,
		KEY_G, KEY_V, KEY_C, KEY_SPACE,
	KEY_9, KEY_8, KEY_U, KEY_Y,
		KEY_J, KEY_N, KEY_B, KEY_BACKSLASH,
	KEY_MINUS, KEY_0, KEY_O, KEY_I,
		KEY_L, KEY_K, KEY_COMMA, KEY_M,
	KEY_RESERVED, KEY_EQUAL, KEY_RIGHTBRACE, KEY_ENTER,
		KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_MENU,
	KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
		KEY_LEFTSHIFT, KEY_RIGHTSHIFT, KEY_RESERVED, KEY_RESERVED,
	KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
		KEY_RESERVED, KEY_LEFTCTRL, KEY_RESERVED, KEY_RIGHTCTRL,
	KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
		KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
	KEY_LEFTBRACE, KEY_P, KEY_APOSTROPHE, KEY_SEMICOLON,
		KEY_SLASH, KEY_DOT, KEY_RESERVED, KEY_RESERVED,
	KEY_F10, KEY_F9, KEY_BACKSPACE, KEY_3,
		KEY_2, KEY_UP, KEY_PRINT, KEY_PAUSE,
	KEY_INSERT, KEY_DELETE, KEY_RESERVED, KEY_PAGEUP,
		KEY_PAGEDOWN, KEY_RIGHT, KEY_DOWN, KEY_LEFT,
	KEY_F11, KEY_F12, KEY_F8, KEY_Q,
		KEY_F4, KEY_F3, KEY_1, KEY_F7,
	KEY_ESC, KEY_GRAVE, KEY_F5, KEY_TAB,
		KEY_F1, KEY_F2, KEY_CAPSLOCK, KEY_F6
};

static int keymap_function[] = {
	/*
	 * Row 0 Unused, Unused, 'W', 'S', 'A', 'Z', Unused, Function,
	 * Row 1 Special, Unused, Unused, Unused, Unused, Unused, Unused, Menu
	 * Row 2 Unused, Unused, Unused, Unused, Unused, Unused, Alt, Alt2
	 * Row 3 '5', '4', 'R', 'E', 'F', 'D', 'X', Unused,
	 * Row 4 '7', '6', 'T', 'H', 'G', 'V', 'C', SPACEBAR,
	 * Row 5 '9', '8', 'U', 'Y', 'J', 'N', 'B', '|\',
	 * Row 6 Minus, '0', 'O', 'I', 'L', 'K', '<', M,
	 * Row 7 Unused, '+', '}]', '#', Unused, Unused, Unused, Menu,
	 * Row 8 Unused, Unused, Unused, Unused, SHIFT, SHIFT, Unused, Unused,
	 * Row 9 Unused, Unused, Unused, Unused, Unused, Ctrl, Unused, Control,
	 * Row A Unused, Unused, Unused, Unused, Unused, Unused, Unused, Unused,
	 * Row B '{[', 'P', '"', ':;', '/?, '>', Unused, Unused,
	 * Row C 'F10', 'F9', 'BckSpc', '3', '2', 'Up, Prntscr, Pause
	 * Row D INS, DEL, Unused, Pgup, PgDn, right, Down, Left,
	 * Row E F11, F12, F8, 'Q', F4, F3, '1', F7,
	 * Row F ESC, '~', F5, TAB, F1, F2, CAPLOCK, F6,
	 */
	KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
		KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
	KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
		KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
	KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
		KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
	KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
		KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
	KEY_7, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
		KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
	KEY_9, KEY_8, KEY_4, KEY_RESERVED,
		KEY_1, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
	KEY_RESERVED, KEY_SLASH, KEY_6, KEY_5,
		KEY_3, KEY_2, KEY_RESERVED, KEY_0,
	KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
		KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
	KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
		KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
	KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
		KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
	KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
		KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
	KEY_RESERVED, KEY_KPASTERISK, KEY_RESERVED, KEY_KPMINUS,
		KEY_KPPLUS, KEY_DOT, KEY_RESERVED, KEY_RESERVED,
	KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
		KEY_RESERVED, KEY_VOLUMEUP, KEY_RESERVED, KEY_RESERVED,
	KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_HOME,
		KEY_END, KEY_BRIGHTNESSUP, KEY_VOLUMEDOWN, KEY_BRIGHTNESSDOWN,
	KEY_NUMLOCK, KEY_SCROLLLOCK, KEY_MUTE, KEY_RESERVED,
		KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
	KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
		KEY_QUESTION, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED
};

struct kbc_virt_table {
	int start_code;
	int end_code;
	int base;
	int *table;
};

static struct kbc_virt_table kbc_normal = {
	NORMAL_SCAN_CODE_TABLE_FIRST,	/* scan code start */
	NORMAL_SCAN_CODE_TABLE_LAST,	/* scan code end */
	KBC_NORMAL_KEY_CODE_BASE,
	keymap_normal			/* normal qwerty keyboard */
};

static struct kbc_virt_table kbc_function = {
	FUNCTION_SCAN_CODE_TABLE_FIRST,	/* scan code start */
	FUNCTION_SCAN_CODE_TABLE_LAST,	/* scan code end */
	KBC_FUNCTION_KEY_CODE_BASE,
	keymap_function			/* Function Qwerty keyboard */
};

static const struct kbc_virt_table *kbc_tables[] = {&kbc_normal, &kbc_function};

static struct tegra_kbc_wake_key seaboard_wake_cfg[] = {
	[0] = {
		.row = 1,
		.col = 7,
	},
	[1] = {
		.row = 15,
		.col = 0,
	},
};

static struct resource tegra_kbc_resources[] = {
	[0] = {
		.start = TEGRA_KBC_BASE,
		.end   = TEGRA_KBC_BASE + TEGRA_KBC_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = INT_KBC,
		.end   = INT_KBC,
		.flags = IORESOURCE_IRQ,
	},
};

static struct tegra_kbc_platform_data tegra_kbc_platform_data0 = {
	.debounce_cnt = 2,
	.repeat_cnt = 5 * 32,
	.wake_cnt = 2,
	.wake_cfg = &seaboard_wake_cfg[0],
};

static struct platform_device tegra_kbc_device = {
	.name = "tegra-kbc",
	.id = -1,
	.resource = tegra_kbc_resources,
	.num_resources = ARRAY_SIZE(tegra_kbc_resources),
	.dev = {
		.platform_data = &tegra_kbc_platform_data0,
	},

};

static int kbc_get_keycode(int base, int row, int col, int row_cnt, int col_cnt)
{
	int code_data;

	code_data = base + ((row * col_cnt) + col);

	return code_data;
}

void seaboard_kbc_init()
{
	struct tegra_kbc_platform_data *data = &tegra_kbc_platform_data0;
	int i, j, k;
	int rows = KBC_MAX_ROW;
	int cols = KBC_MAX_COL;

	BUG_ON((rows + cols) > KBC_MAX_GPIO);
	/*
	 * Setup the pin configuration information.
	 */
	for (i = 0; i < rows; i++) {
		data->pin_cfg[i].num = i;
		data->pin_cfg[i].is_row = true;
		data->pin_cfg[i].is_col = false;
	}

	for (j = 0; j < cols; j++) {
		data->pin_cfg[rows + j].num = j;
		data->pin_cfg[rows + j].is_row = false;
		data->pin_cfg[rows + j].is_col = true;
	}

	for (k = 0; k < ARRAY_SIZE(kbc_tables); k++) {
		for (i = 0; i < KBC_MAX_KEY; i++)
			data->keymap[k][i] = -1;
	}

	for (i = 0; i < rows; i++) {
		for (j = 0; j < cols; j++) {
			for (k = 0; k < ARRAY_SIZE(kbc_tables); k++) {
				int sc = kbc_get_keycode(kbc_tables[k]->base,
							 i, j, rows, cols);
				if (sc >= kbc_tables[k]->start_code &&
				    sc <= kbc_tables[k]->end_code) {
					sc -= kbc_tables[k]->start_code;
					sc = kbc_tables[k]->table[sc];
					if (!sc)
						continue;
					data->keymap[k][kbc_indexof(i, j)] = sc;
				}
			}
		}
	}

	platform_device_register(&tegra_kbc_device);
}
#endif
