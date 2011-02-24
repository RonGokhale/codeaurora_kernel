/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/io.h>
#include <linux/module.h>
#include <mach/msm_iomap.h>
#include "tlmm-msm8660.h"
#include "gpiomux.h"

struct msm_gpiomux_config msm_gpiomux_configs[GPIOMUX_NGPIOS] = {};

void __msm_gpiomux_write(unsigned gpio, gpiomux_config_t val)
{
	writel(val & ~GPIOMUX_CTL_MASK, GPIO_CONFIG(gpio));
}
