#include <linux/export.h>
#include <linux/types.h>

#include <asm/checksum.h>
#include <asm/delay.h>
#include <asm/div64.h>
#include <asm/ftrace.h>
#include <asm/page.h>
#include <asm/string.h>
#include <asm/tbx.h>
#include <asm/uaccess.h>

EXPORT_SYMBOL(clear_page);
EXPORT_SYMBOL(copy_page);

#ifdef CONFIG_FLATMEM
/* needed for the pfn_valid macro */
EXPORT_SYMBOL(max_pfn);
EXPORT_SYMBOL(min_low_pfn);
#endif

/* Network checksum functions */
EXPORT_SYMBOL(ip_fast_csum);
EXPORT_SYMBOL(csum_partial);
EXPORT_SYMBOL(ip_compute_csum);
EXPORT_SYMBOL(csum_partial_copy_from_user);
EXPORT_SYMBOL(csum_partial_copy);

/* Delay functions */
EXPORT_SYMBOL(__delay);
EXPORT_SYMBOL(__const_udelay);
EXPORT_SYMBOL(__udelay);
EXPORT_SYMBOL(__ndelay);

/* Userspace copying functions */
EXPORT_SYMBOL(__copy_user);
EXPORT_SYMBOL(__copy_user_zeroing);
EXPORT_SYMBOL(__do_clear_user);
EXPORT_SYMBOL(__get_user_asm_b);
EXPORT_SYMBOL(__get_user_asm_w);
EXPORT_SYMBOL(__get_user_asm_d);
EXPORT_SYMBOL(__put_user_asm_b);
EXPORT_SYMBOL(__put_user_asm_w);
EXPORT_SYMBOL(__put_user_asm_d);
EXPORT_SYMBOL(__put_user_asm_l);
EXPORT_SYMBOL(strnlen_user);
EXPORT_SYMBOL(__strncpy_from_user);

/* TBI symbols */
EXPORT_SYMBOL(__TBI);
EXPORT_SYMBOL(__TBIFindSeg);
EXPORT_SYMBOL(__TBIPoll);
EXPORT_SYMBOL(__TBITimeStamp);
EXPORT_SYMBOL(__TBITransStr);

#define DECLARE_EXPORT(name) extern void name(void); EXPORT_SYMBOL(name)

/* libgcc functions */
DECLARE_EXPORT(__ashldi3);
DECLARE_EXPORT(__ashrdi3);
DECLARE_EXPORT(__lshrdi3);
DECLARE_EXPORT(__udivsi3);
DECLARE_EXPORT(__divsi3);
DECLARE_EXPORT(__umodsi3);
DECLARE_EXPORT(__modsi3);
DECLARE_EXPORT(__muldi3);
DECLARE_EXPORT(__cmpdi2);
DECLARE_EXPORT(__ucmpdi2);

/* Maths functions */
EXPORT_SYMBOL(div_u64);
EXPORT_SYMBOL(div_s64);

/* String functions */
EXPORT_SYMBOL(memcpy);
EXPORT_SYMBOL(memset);
EXPORT_SYMBOL(memmove);

#ifdef CONFIG_FUNCTION_TRACER
EXPORT_SYMBOL(mcount_wrapper);
#endif
