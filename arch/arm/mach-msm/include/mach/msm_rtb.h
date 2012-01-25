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
	LOGBUF,
	CTXID,
	OTHER,
};

#if defined(CONFIG_MSM_RTB)
void __log_uncached_data(enum uncached_log_type log_type, void *caller,
				void *data);

void log_uncached_data(enum uncached_log_type log_type, void *data);

#define ETB_EXTRA_BRANCHES  do {BRANCH_TO_NEXT_ISTR; \
				nop(); \
				BRANCH_TO_NEXT_ISTR; \
				nop(); } while (0)
#define BRANCH_TO_NEXT_ISTR  asm volatile("b .+4\n" : : : "memory")
/*
 * both the mb and the isb are needed to ensure enough waypoints for
 * etb tracing
 */
#define LOG_BARRIER	do { mb(); isb(); } while (0)
#else
static inline void __log_uncached_data(enum uncached_log_type log_type,
					void *caller,
					void *data) { return; }
static inline void log_uncached_data(enum uncached_log_type log_type,
					void *unused) { return; }
#define ETB_EXTRA_BRANCHES
#define BRANCH_TO_NEXT_ISTR
/*
 * Due to a GCC bug, we need to have a nop here in order to prevent an extra
 * read from being generated after the write.
 */
#define LOG_BARRIER		nop()
#endif
#endif
