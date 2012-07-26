/*
	All files except if stated otherwise in the begining of the file are under the ISC license:
	-----------------------------------------------------------------------------------

	Copyright (c) 2010-2012 Design Art Networks Ltd.

	Permission to use, copy, modify, and/or distribute this software for any
	purpose with or without fee is hereby granted, provided that the above
	copyright notice and this permission notice appear in all copies.

	THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
	WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
	MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
	ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
	WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
	ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
	OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
*/


#ifndef __DANIPC_TYPES_H__
#define __DANIPC_TYPES_H__

/* Types visible in user-space. */

#ifdef __KERNEL__
#include <linux/kernel.h>
#else
#include <stdint.h>
#endif /* KERNEL */

#define DANIPC_MAX_BUF		1500		/* FIXME: calculate */


typedef uint8_t 		danipc_addr_t;


/* FIXME: move to kernel-only header. */
#ifdef __KERNEL__
/* Connection information. */
typedef struct
{
	unsigned		prio;
	danipc_addr_t		dst;
	danipc_addr_t		src;
} danipc_pair_t;
#endif


#endif /* __DANIPC_TYPES_H__ */
