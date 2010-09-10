/*
 * arch/arm/mach-tegra/board-harmony.h
 *
 * Copyright (C) 2010 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _MACH_TEGRA_BOARD_HARMONY_H
#define _MACH_TEGRA_BOARD_HARMONY_H

#define TEGRA_GPIO_BACKLIGHT		TEGRA_GPIO_PB5
#define TEGRA_GPIO_LVDS_SHUTDOWN	TEGRA_GPIO_PB2
#define TEGRA_GPIO_BACKLIGHT_PWM	TEGRA_GPIO_PB4
#define TEGRA_GPIO_BACKLIGHT_VDD	TEGRA_GPIO_PW0
#define TEGRA_GPIO_EN_VDD_PNL		TEGRA_GPIO_PC6

void harmony_pinmux_init(void);
int harmony_panel_init(void);
int harmony_sdhci_init(void);

#endif
