/*
 * Early serial output macro for Hisilicon SoC
 *
 * Copyright (C) 2012-2013 Hisilicon Technologies Co., Ltd.
 * Copyright (C) 2012-2013 Linaro Ltd.
 *
 * Author: Haojian Zhuang <haojian.zhuang@linaro.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#if defined(CONFIG_DEBUG_HI3620_UART)
#define HS_UART_PHYS_BASE	0xfcb00000
#elif defined(CONFIG_DEBUG_HI3716_UART)
#define HS_UART_PHYS_BASE	0xf8b00000
#endif
#define HS_UART_VIRT_BASE	0xfeb00000

#if defined(CONFIG_DEBUG_HI3620_UART) || defined(CONFIG_DEBUG_HI3716_UART)
		.macro	addruart,rp,rv,tmp
		ldr	\rp, =HS_UART_PHYS_BASE
		ldr	\rv, =HS_UART_VIRT_BASE
		.endm

#include <asm/hardware/debug-pl01x.S>

#endif
