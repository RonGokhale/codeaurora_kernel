/*
 * Copyright (c) 2012, Code Aurora Forum. All rights reserved.
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
#ifndef __MACH_RTB__
#define __MACH_RTB__

enum uncached_log_type {
	NONE = 0,
	READL,
	WRITEL,
	OTHER,
};

#if defined(CONFIG_MSM_RTB)
void __log_uncached_data(enum uncached_log_type log_type, void *caller,
				void *data);

void log_uncached_data(enum uncached_log_type log_type, void *data);

#else
static inline void __log_uncached_data(enum uncached_log_type log_type,
					void *caller,
					void *data) { return; }
static inline void log_uncached_data(enum uncached_log_type log_type,
					void *unused) { return; }
#endif
#endif
