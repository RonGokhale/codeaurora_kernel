/*
 *  Copyright (C) 2001 Andrea Arcangeli <andrea@suse.de> SuSE
 *  Copyright 2003 Andi Kleen, SuSE Labs.
 *
 *  Modified for x86 32 bit arch by Stefani Seibold <stefani@seibold.net>
 *
 *  Thanks to hpa@transmeta.com for some useful hint.
 *  Special thanks to Ingo Molnar for his early experience with
 *  a different vsyscall implementation for Linux/IA32 and for the name.
 *
 */

#include <asm/vsyscall.h>
#include <asm/pgtable.h>
#include <asm/fixmap.h>
#include <asm/elf.h>

void __init map_vsyscall(void)
{
	if (vdso_enabled != VDSO_COMPAT)
		return;

	__set_fixmap(VVAR_PAGE, __pa_symbol(&__vvar_page), PAGE_KERNEL_VVAR);
}
