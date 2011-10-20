#ifndef _ASM_X86_TYPES_H
#define _ASM_X86_TYPES_H

#if defined __x86_64__ && !defined __LP64__
#define __KERNEL_NATIVE_LONG_TYPE long long
#else
#define __KERNEL_NATIVE_LONG_TYPE long
#endif

#include <asm-generic/types.h>

#endif /* _ASM_X86_TYPES_H */
