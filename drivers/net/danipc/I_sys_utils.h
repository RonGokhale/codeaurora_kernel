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


#ifndef _I_SYS_UTILS_H
#define _I_SYS_UTILS_H

/*
 * -----------------------------------------------------------
 * Include section
 * -----------------------------------------------------------
 */
#if !defined(WIN32)
//#include "I_sys_defs.h"
#endif

/*
 * -----------------------------------------------------------
 * MACRO (define) section
 * -----------------------------------------------------------
 */

#define ALIGN_128BIT(n)				((n) & ~0x0000000f)
#define INC_128BIT(n)				((n) + 0x00000010)
#define CHECK_ALIGN_128BIT(n)		(!((n) & 0x0000000c))

#define ALIGN_32BIT(n)              ((n) & ~0x00000003)
#define INC_32BIT(n)                ((n) + 0x00000004)
#define CHECK_ALIGN_32BIT(n)		(!((n) & 0x00000003))

#define ALIGN_4SAMPLES(n)			((n) & ~0x00000003)
#define CHECK_ALIGN_4SAMPLES(n)		(!((n) & 0x00000003))

#define BITS_TO_BYTES(n)            ((((n) + 7)) >> 3)
#define ALIGN_TO_4(n)				(((n) + 3) & ~3)


#define W32_TO_BYTES(n)				((n) << 2)
#define BYTES_TO_W32(n)				((n) >> 2)

#define PHY_SAMPLE_TO_BYTES(n)		W32_TO_BYTES(n)
#define BYTES_TO_PHY_SAMPLE(n)		BYTES_TO_W32(n)

#define	TRUE	1
#define	FALSE	0
#ifndef NULL
#define	NULL		((void *)0)
#endif
#define	PTR(x)		((void *)(x))
#define	UNUSED(x)	((void)(x))


extern int printk(const char * fmt, ...)
        __attribute__ ((format (printf, 1, 2))) __attribute__((__cold__));


#define ASSERT(flag)														\
	if (!(UINT32)(flag))													\
	{																		\
		printk("%s:%d: *** ASSERT ***\n", __FILE__, __LINE__);/*OS_assert((UINT32)(flag), __FILE__, __LINE__);*/						\
	}

#define NO_ASSERT(flag)														\
	if ((flag))																\
	{																		\
		OS_assert((UINT32)(flag), __FILE__, __LINE__);						\
	}

#define	LIMIT(x,l,h)		XT_MIN(XT_MAX(x,l),h)

#define	POW2(n)				(0x1<<(n))
#define	POW2_F(n)			(1.0 * (float)POW2(n))

#define	F_POW2_INT(n)	((n) >= 0 ? (float)POW2(n) : 1.0/(float)POW2(-n))


//#define	INC_MOD(a,b,m)		(((a)+(b))<(m) ? ((a)+(b)) : ((a)+(b)-(m)))
#define	INC_MOD(a,b,m)		(((a)+(b)) %(m))
#define	MASK_BIT(b)			(POW2(b))
#define	MASK_N_BITS(n)		(POW2(n)-1)

#ifndef MIN
#if !defined(UNIX) && !defined(WIN32)
#define MIN(x,y)            XT_MINU (x, y)
#else
#define MIN(x,y)			((x)<(y) ? (x):(y))
#endif
#endif

#ifndef MAX
#if !defined(UNIX) && !defined(WIN32)
#define MAX(x,y)            XT_MAXU (x, y)
#else
#define MAX(x,y)			((x)>(y) ? (x):(y))
#endif
#endif

#define ABS(x)          (((x)>=0) ? (x) : -(x))


#define BUILD_WORD(halfword1,halfword2)		(((halfword2) << 16) | halfword1)
#define HALFWORD_LOW(word)					(word & 0x0000FFFF)
#define HALFWORD_HIGH(word)					(word >> 16)

// Get an array size
#define TABLESIZE(table) 	(sizeof(table)/sizeof((table)[0]))

#define	ADDR_MOD_4K(n)		((n) & MASK_N_BITS(12))
#define	ADDR_4K				(POW2(12))

#define	SIZE_OF_4_REs		(16)	// 16 bytes

#define	IQ_FIXP_MANTISSA	(16)

#define	SQRT_2				(1.4142135623730950488016887242097)
#define	SQRT_0_DOT_5		(0.70710678118654752440084436210485)

#define PI					(3.1415926535897932384626433832795)

#define	F_TO_FIXP(x,n)			((INT16)((x)*POW2_F((n)-1)))
#define	F_TO_FIXP16(x)			F_TO_FIXP(x,IQ_FIXP_MANTISSA)

#define	ImRe_PACK(i,r)			(((((INT16)(i)) & MASK_N_BITS(IQ_FIXP_MANTISSA)) << IQ_FIXP_MANTISSA)	\
									| (((INT16)(r)) & MASK_N_BITS(IQ_FIXP_MANTISSA))	)

#define	ImRe_FIXP_PACK(i,r,n)		ImRe_PACK(F_TO_FIXP(i,n),F_TO_FIXP(r,n))
#define	ImRe_FIXP16_PACK(i,r)		ImRe_FIXP_PACK(i,r,IQ_FIXP_MANTISSA)


#define	EXTRACT_IMAG(n)	(((n) >> (IQ_FIXP_MANTISSA)) & MASK_N_BITS(IQ_FIXP_MANTISSA))
#define	EXTRACT_REAL(n)	((n) & MASK_N_BITS(IQ_FIXP_MANTISSA))


/* Used as termination value for initialization list of HW registers */
#define HW_REG_OFFSET_LAST			(0xFFFFFFFF)

#define HW_REG_CONF_TABLE_PTR(name) &HW_reg_conf_##name[0]
// Create a configuration registers table for specific HW_REG entity
#define HW_START_REG_CONF_TABLE(name)		\
		const HW_reg_conf_array_t HW_reg_conf_##name[] = {
// Configuration registers table entity
#define HW_REG_CONF_ENTRY(_offset,_val)	{_offset,_val},
// End of configuration registers table
#define HW_END_REG_CONF_TABLE {HW_REG_OFFSET_LAST,0}};

/* INTERNAL: Read/Write access to APB memory mapped registers */
#define HW_SET_REG(ba,x,y) (*((VUINT32 *)((UINT32)ba+(UINT32)(x))) =(UINT32)y)
#define HW_GET_REG(ba,x)	 (*((VUINT32 *)((UINT32)ba+(UINT32)(x))))

#define	ROUND_DIV(x,y)			(((x) + (y) / 2) / (y))
#define	ROUND_UP_DIV(x,y)		(((x) + (y) - 1) / (y))
#define	ROUND_UP_POW_2(x,y)		(((x) + ((y) - 1)) & (~((y) - 1)))
#define	ROUND_DOWN_POW_2(x,y)	((x) & (~((y) - 1)))
#define	BITS_IN_WORD			(sizeof(UINT32)*8)

#define INLINE  __attribute__((always_inline))
#define NOINLINE  __attribute__((noinline))

#define SECTION(_sec)  __attribute__ ((section(_sec)))

#define    outb(x,y)    (*((char *)(x)) = y)
#define    outw(x,y)    (*((short *)(x)) = y)
#define    outl(x,y)    (*((long *)(x)) = y)

#define    inb(x)       (*((char *)(x)))
#define    inw(x)       (*((short *)(x)))
#define    inl(x)       (*((long *)(x)))

#define    outvb(x,y)    (*((volatile char *)(x)) = y)
#define    outvw(x,y)    (*((volatile short *)(x)) = y)
#define    outvl(x,y)    (*((volatile long *)(x)) = y)

#define    invb(x)       (*((volatile char *)(x)))
#define    invw(x)       (*((volatile short *)(x)))
#define    invl(x)       (*((volatile long *)(x)))
//#define    invl(x)       readl(x)

/* Embed a bit field (LEN) bits wide from a source byte (SRC)
*  where the field is in the (LEN) least significant bits, into
*  a destination byte (DST) starting at bit position (POS) and
*  return the modified version of (DST).
*/
#define PACK_W32(SRC,DST,POS,LEN) \
((((~(0xFFFFFFFF<<(LEN)))&(SRC))<<(POS))|((DST)&(~((~(0xFFFFFFFF<<(LEN)))<<(POS)))))

/* Extract a bit field (LEN) bits wide, starting at bit (POS) from
*  within a source byte (SRC) and return it as the least significant
*  bits of an otherwise zero byte.
*
*/
#define SUB_W32(SRC,POS,LEN) (((SRC)>>(POS))&(~(0xFFFFFFFF<<(LEN))))

#ifndef DEBUG_OUT
#ifdef SIMULATION
	#define	DEBUG_OUT(x)	WRITE_OUTQ0_32(x)
#endif
#endif

#define	ADDR_W128_TO_ADDR_W32(n)	((n)<<2)
#define	ADDR_W32_TO_ADDR_W128(n)	((n)>>2)


#if !defined(WIN32)

//BIT reversal Macros (Temp until implemented in the 'silicon'
#ifndef UINT8_BIT_REVERSAL
//bitReversalLu is defined in DEBUG.c
extern const UINT8 bitReversalLu[];
static inline UINT16 uint16_bit_reversal (UINT16 val)
{
	UINT16  vals = val;
	UINT16  rev_val;
	UINT8   *valByteArray = (UINT8 *)&vals;
	UINT8   *revValPtr = (UINT8 *)&rev_val;

	revValPtr[0] = bitReversalLu[valByteArray[1]];
	revValPtr[1] = bitReversalLu[valByteArray[0]];

	return rev_val;
}

static inline UINT32 uint32_bit_reversal (UINT32 val)
{
	UINT32  vall = val;
	UINT32  rev_val;
	UINT8  *valByteArray = (UINT8 *)&vall;
	UINT8  *revValPtr = (UINT8 *)&rev_val;

	revValPtr[0] = bitReversalLu[valByteArray[3]];
	revValPtr[1] = bitReversalLu[valByteArray[2]];
	revValPtr[2] = bitReversalLu[valByteArray[1]];
	revValPtr[3] = bitReversalLu[valByteArray[0]];

	return rev_val;
}

#define UINT8_BIT_REVERSAL(x)  (bitReversalLu[(UINT8)x])
#define UINT16_BIT_REVERSAL(x) (uint16_bit_reversal((UINT16)x) )
#define UINT32_BIT_REVERSAL(x)  (uint32_bit_reversal ((UINT32)x) )
#endif

// fast euclid algoroithm
static inline UINT32 calc_gcd(UINT32 a,UINT32 b)
{
	UINT32 temp;

	while(b>0) {
		temp = b;
		b = a % b;		// b=a mod b
		a = temp;
	}
	return(a);
}


/*
 * -----------------------------------------------------------
 * Type definition section
 * -----------------------------------------------------------
 */
/* TBD: remove prototypes after proper ASSERT implementation  */
extern void OS_assert(unsigned cause, char *file, unsigned line);

#ifdef RTOS
#include "tx_api.h"
#include "OSA_api.h"
#endif /* RTOS */

#endif /* WIN32 */


/*
 * -----------------------------------------------------------
 * MACRO to set memory allocation attributes for code & data
 * -----------------------------------------------------------
 */
#if !defined(UNIX) && !defined(WIN32)
#define DATA_FASTEST	__attribute__ ((section(MODULENAME".fastest.data")))
#define CODE_FASTEST	__attribute__ ((section(MODULENAME".fastest.text")))
#define DATA_FAST		__attribute__ ((section(MODULENAME".fast.data")))
#define CODE_FAST		__attribute__ ((section(MODULENAME".fast.text")))
#define DATA_MEDIUM		__attribute__ ((section(MODULENAME".medium.data")))
#define CODE_MEDIUM		__attribute__ ((section(MODULENAME".medium.text")))
#define DATA_SLOW		__attribute__ ((section(MODULENAME".slow.data")))
#define CODE_SLOW		__attribute__ ((section(MODULENAME".slow.text")))
#define DATA_SLOWEST	__attribute__ ((section(MODULENAME".slowest.data")))
#define CODE_SLOWEST	__attribute__ ((section(MODULENAME".slowest.text")))
#define DATA_DEFAULT	__attribute__ ((section(MODULENAME".default.data")))
#define CODE_DEFAULT	__attribute__ ((section(MODULENAME".default.text")))

#define DATA_FASTEST_ALIGN(x)	__attribute__ ((section(MODULENAME".fastest.data"),aligned(x)))
#define CODE_FASTEST_ALIGN(x)	__attribute__ ((section(MODULENAME".fastest.text"),aligned(x)))
#define DATA_FAST_ALIGN(x)		__attribute__ ((section(MODULENAME".fast.data"),aligned(x)))
#define CODE_FAST_ALIGN(x)		__attribute__ ((section(MODULENAME".fast.text"),aligned(x)))
#define DATA_MEDIUM_ALIGN(x)	__attribute__ ((section(MODULENAME".medium.data"),aligned(x)))
#define CODE_MEDIUM_ALIGN(x)	__attribute__ ((section(MODULENAME".medium.text"),aligned(x)))
#define DATA_SLOW_ALIGN(x)		__attribute__ ((section(MODULENAME".slow.data"),aligned(x)))
#define CODE_SLOW_ALIGN(x)		__attribute__ ((section(MODULENAME".slow.text"),aligned(x)))
#define DATA_SLOWEST_ALIGN(x)	__attribute__ ((section(MODULENAME".slowest.data"),aligned(x)))
#define CODE_SLOWEST_ALIGN(x)	__attribute__ ((section(MODULENAME".slowest.text"),aligned(x)))
#define DATA_DEFAULT_ALIGN(x)	__attribute__ ((section(MODULENAME".default.data"),aligned(x)))
#define CODE_DEFAULT_ALIGN(x)	__attribute__ ((section(MODULENAME".default.text"),aligned(x)))

#else
#define DATA_FASTEST
#define CODE_FASTEST
#define DATA_FAST
#define CODE_FAST
#define DATA_MEDIUM
#define CODE_MEDIUM
#define DATA_SLOW
#define CODE_SLOW
#define DATA_SLOWEST
#define CODE_SLOWEST
#define DATA_DEFAULT
#define CODE_DEFAULT

#define DATA_FASTEST_ALIGN(x)
#define CODE_FASTEST_ALIGN(x)
#define DATA_FAST_ALIGN(x)
#define CODE_FAST_ALIGN(x)
#define DATA_MEDIUM_ALIGN(x)
#define CODE_MEDIUM_ALIGN(x)
#define DATA_SLOW_ALIGN(x)
#define CODE_SLOW_ALIGN(x)
#define DATA_SLOWEST_ALIGN(x)
#define CODE_SLOWEST_ALIGN(x)
#define DATA_DEFAULT_ALIGN(x)
#define CODE_DEFAULT_ALIGN(x)
#endif // #if !defined(UNIX) && !defined(WIN32)

#ifndef BIG_ENDIAN
#define	BIG_ENDIAN			0
#define	LITTLE_ENDIAN			1
#else
#error "Supported only little endian"
#endif

#define	ENDIANESS				LITTLE_ENDIAN

#if ENDIANESS == BIG_ENDIAN
     //  #warning: "System is BIG_ENDIAN"
  #define htons(A) (A)
  #define htonl(A) (A)
  #define ntohs(A) (A)
  #define ntohl(A) (A)
#elif ENDIANESS == LITTLE_ENDIAN
     //  #warning: "System is LITTLE_ENDIAN"
  #define htons(A) ((((unsigned short)(A) & 0xff00) >> 8) | \
                         (((unsigned short)(A) & 0x00ff) << 8))
  #define htonl(A) ((((unsigned long)(A) & 0xff000000) >> 24) | \
                         (((unsigned long)(A) & 0x00ff0000) >> 8)  | \
                         (((unsigned long)(A) & 0x0000ff00) << 8)  | \
                         (((unsigned long)(A) & 0x000000ff) << 24))

  #define ntohs  htons
  #define ntohl  htonl
#else
  #error: "Must define one of BIG_ENDIAN or LITTLE_ENDIAN"
#endif

/*
 * Represent number (16.16 bits) as 32 bit
 */

#define	FIX_16DOT16_M				(16-1)
#define	FIX_16DOT16_N  				(16+16)


#define FIX_16DOT16_CONST(x)	    ((INT32)((x)*(int)POW2(16)))
#define INT_TO_FIX_16DOT16(x)	    ((INT32)(x) << 16)
#define FIX_16DOT16_TO_INT(x)	    ((x) >> 16)
#define FIX_16DOT16_TO_FRAC(x)		((INT32)((UINT32)(x) << 16))

#define ROUND_0_5    FIX_16DOT16_CONST(0.5)

// shifts
#define	SHIFT(x,n)		((n) >= 0 ? ((x) << (n)) : ((x) >> (-(n))))

// general fixpoint quantization
#define	LSB_LOG2_FIX_MqN(m,n)					((m)-(n)+1)
#define	LSB_FIX_MqN_2_FLOAT(m,n)				POW2_FLOAT(LSB_LOG2_FIX_MqN(m,n))

#define	M_FIX_M1qN1_X_FIX_M2qN2(m1,n1,m2,n2)	((m1)+(m2)+1)
#define	N_FIX_M1qN1_X_FIX_M2qN2(m1,n1,m2,n2)	((n1)+(n2))

#define	M_INT(n)								((n)-1)
#define	N_INT(n)								(n)

#define	FLOAT_2_FIX_MqN(x,m,n)					((INT32)((x)/LSB_FIX_MqN_2_FLOAT(m,n)))
#define	FLOAT_2_FIX_MqN_RND(x,m,n)				((INT32)RND((x)/LSB_FIX_MqN_2_FLOAT(m,n)))

#define	FIX_M1qN1_2_FIX_M2qN2(x,m1,n1,m2,n2)	SHIFT(x,(LSB_LOG2_FIX_MqN(m1,n1) - LSB_LOG2_FIX_MqN(m2,n2)))

#define	FIX_MqN_2_INT(x,m,n)					FIX_M1qN1_2_FIX_M2qN2(x,m,n,n-1,n)
#define	FIX_MqN_2_UFRAC(x,m,n)					FIX_M1qN1_2_FIX_M2qN2(((unsigned)x),m,n,-1,n)

#define ConvToUINT128(Data)						*((UINT128*)(&(Data)))
#define ConvToUINT32(Data)						*((UINT32*)(&(Data)))

#endif

/*
 * -----------------------------------------------------------
 * End of file
 * -----------------------------------------------------------
 */
