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


#ifndef _I_SYS_TYPES_H_
#define _I_SYS_TYPES_H_


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


/*
 * -----------------------------------------------------------
 * Type definition section
 * -----------------------------------------------------------
 */
#ifndef UINT8
typedef unsigned char UINT8;
#endif

#ifndef VUINT8
typedef volatile unsigned char VUINT8;
#endif

#ifndef UINT16
typedef unsigned short UINT16;
#endif

#ifndef VUINT16
typedef volatile unsigned short VUINT16;
#endif

#ifndef UINT32
typedef unsigned int UINT32;
#endif

#ifndef VUINT32
typedef volatile unsigned int VUINT32;
#endif

#ifndef UINT64
typedef unsigned long long UINT64;
#endif

#ifndef long64
typedef long long long64;
#endif


#ifndef INT8
typedef signed char INT8;
#endif

#ifndef INT16
typedef signed short INT16;
#endif

#ifndef INT32
typedef signed int INT32;
#endif

#ifndef INT64
typedef signed long long INT64;
#endif

#ifndef RTOS
#ifndef CHAR
typedef char CHAR;
#endif

#ifndef INT
typedef int INT;
#endif

#ifndef UINT
typedef int UINT;
#endif
#endif /* RTOS */

#ifndef VCHAR
typedef volatile char VCHAR;
#endif

#ifndef VOID
typedef void VOID;
#endif

#ifndef BOOLEAN
typedef unsigned long BOOLEAN;
#endif

#ifndef BOOL
typedef unsigned long BOOL;
#endif

#ifndef TBOOL
typedef unsigned char TBOOL;
#endif

#ifndef db16dot16
typedef INT32 db16dot16;
#endif

#ifndef fix16dot16
typedef INT32 fix16dot16;
#endif

#ifndef fix48dot16
typedef INT64 fix48dot16;
#endif


/*
 * puts_fn_t - Type for pointer to the function puts()
 */
#ifndef puts_fn_t
typedef void (*puts_fn_t)(const char *);
#endif

/* Initialization pairs (address offset + desired value)
 *  for HW registers configuration	*/
typedef struct HW_reg_conf_array_s
{
	UINT32 reg_offset;	/* HW register address offset from base address  	*/
	UINT32 reg_value;	/* initialization value */
} HW_reg_conf_array_t;

#ifdef DSP
#define ALIGN(x)  __attribute__((aligned((x))))

typedef struct
{
	short int real ;
	short int imag ;
} complexT ;

typedef struct
{
    complexT lane0;
    complexT lane1;
    complexT lane2;
    complexT lane3;
} c_cm ALIGN(16);

typedef struct
{
    long real0;
    long real1;
    long real2;
    long real3;
} c_cm4l ALIGN(16);

typedef struct
{
    short real0;
    short real1;
    short real2;
    short real3;
    short real4;
    short real5;
    short real6;
    short real7;
} c_cm8s ALIGN(16);

typedef struct
{
    signed char real0;
    signed char real1;
    signed char real2;
    signed char real3;
    signed char real4;
    signed char real5;
    signed char real6;
    signed char real7;
    signed char real8;
    signed char real9;
    signed char real10;
    signed char real11;
    signed char real12;
    signed char real13;
    signed char real14;
    signed char real15;
} c_cm16c ALIGN(16);



typedef union
{
//	complexTx4		cval ;
	complexT		cmp_ar[4]		ALIGN(16);
	signed  char	chars[16]		ALIGN(16);
	short int		short_ar[8]		ALIGN(16);
	long			long_ar[4]		ALIGN(16);
	c_cm			c_cm_var		ALIGN(16);
	c_cm4l			cm4l			ALIGN(16);
	c_cm8s			cm8s			ALIGN(16);
	c_cm16c			cm16c			ALIGN(16);
	cm				cm_var			ALIGN(16);
} cVecT ALIGN(16) ;


//--- PQ -----------------------------------------------------------


typedef struct
{
	c_cm p;
	c_cm q;
} c_pq;

typedef struct
{
	c_cm16c p;
	c_cm16c q;
} c_pq32c;


typedef union
{
	struct
	{
		cVecT p;
		cVecT q;
    }						ALIGN(16);
    c_pq		cpq			ALIGN(16);
	c_pq32c		cpq32c		ALIGN(16);
} complexPQ ;



//---- ACC ----------------------------------------------------------

typedef struct
{
	long64 real0;
	long64 imag0;
	long64 real1;
	long64 imag1;
	long64 real2;
	long64 imag2;
	long64 real3;
	long64 imag3;
} c_acc ;

typedef struct
{
	short short0;
	short short1;
	long  unusedA;
	short short2;
	short short3;
	long  unusedB;
	short short4;
	short short5;
	long  unusedC;
	short short6;
	short short7;
	long  unusedD;
	short short8;
	short short9;
	long  unusedE;
	short short10;
	short short11;
	long  unusedF;
	short short12;
	short short13;
	long  unusedG;
	short short14;
	short short16;
	long  unusedH;
} c_acc16s ;


typedef struct
{
    signed char real0;
    signed char real1;
    signed char real2;
    signed char real3;
	long unusedA;
    signed char real4;
    signed char real5;
    signed char real6;
    signed char real7;
	long unusedB;
    signed char real8;
    signed char real9;
    signed char real10;
    signed char real11;
	long unusedC;
    signed char real12;
    signed char real13;
    signed char real14;
    signed char real15;
	long unusedD;
    signed char real16;
    signed char real17;
    signed char real18;
    signed char real19;
	long unusedE;
    signed char real20;
    signed char real21;
    signed char real22;
    signed char real23;
	long unusedF;
    signed char real24;
    signed char real25;
    signed char real26;
    signed char real27;
	long unusedG;
    signed char real28;
    signed char real29;
    signed char real30;
    signed char real31;
	long unusedH;
} c_acc32c ;


typedef struct
{
	long64	real ;
	long64	imag ;
} acc_complex40;


typedef union
{
	c_acc			cacc		ALIGN(16);
	c_acc16s		cacc16s		ALIGN(16);
	c_acc32c		cacc32c		ALIGN(16);
	signed char		chars[32]	ALIGN(16);
	long64			longs[8]	ALIGN(16);
	acc_complex40	cmplx[4]	ALIGN(16);
} UACC;

//------ c_acc_acc -------------------------------------------
typedef struct
{
	c_acc	accX ;
	c_acc	accY ;
} c_acc_acc;

//------ c_cm8s_ar -------------------------------------------
typedef struct
{
	c_cm8s	cm8s;
	int		ar;
} c_cm8s_ar;

//------ c_cm_acc -------------------------------------------
typedef struct
{
	c_cm	c_cm_var;
	c_acc	acc;
} c_cm_acc;

//------ c_cm4l_acc -------------------------------------------
typedef struct
{
	c_cm4l	cm4l;
	c_acc	acc;
} c_cm4l_acc;

//------ c_pq_acc -------------------------------------------
typedef struct
{
	c_pq	pq;
	c_acc	acc;
} c_pq_acc;

//------ c_cm_pq -------------------------------------------
typedef struct
{
	c_cm	c_cm_var;
	c_pq	pq;
} c_cm_pq;

//------ c_cm_int -------------------------------------------
typedef struct
{
	c_cm	c_cm_var;
	int		ar;
} c_cm_int;

//------ c_acc16_acc16 -------------------------------------------
typedef struct
{
	c_acc16s	accZ1;
	c_acc16s	accZ2;
} c_acc16_acc16;
#endif /* DSP */


#endif // _I_SYS_TYPES_H_
/*
 * -----------------------------------------------------------
 * End of file
 * -----------------------------------------------------------
 */
