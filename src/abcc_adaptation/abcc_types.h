/*******************************************************************************
********************************************************************************
**                                                                            **
** ABCC Starter Kit version 1640ed3 (2025-03-05)                              **
**                                                                            **
** Delivered with:                                                            **
**    ABP            c799efc (2024-05-14)                                     **
**    ABCC Driver    576777a (2025-03-06)                                     **
**                                                                            */
/*******************************************************************************
** Copyright 2025-present HMS Industrial Networks AB.
** Licensed under the MIT License.
********************************************************************************
** File Description:
** Definition of ABCC types
********************************************************************************
*/

#ifndef ABCC_TD_H
#define ABCC_TD_H

#include <stdint.h>

/* #define ABCC_SYS_BIG_ENDIAN  */
/* #define ABCC_SYS_16_BIT_CHAR */

#define ABCC_CFG_PAR_EXT_BUS_ENDIAN_DIFF 0

/*---------------------------------------------------------------------------
**
** BOOL
** BOOL8
**
** INT8
** INT16
** INT32
**
** UINT8
** UINT16
** UINT32
**
** FLOAT32
**
** The standard boolean data type, 8-bit.
** The standard signed 8, 16, and 32 bit data types (respectively).
** The standard unsigned 8, 16, and 32 bit data types (respectively).
**---------------------------------------------------------------------------
*/
typedef int             BOOL;
typedef unsigned char   BOOL8;
typedef uint8_t         UINT8;
typedef int8_t          INT8;
typedef uint16_t        UINT16;
typedef int16_t         INT16;
typedef uint32_t        UINT32;
typedef int32_t         INT32;
typedef float           FLOAT32;

/*---------------------------------------------------------------------------
**
** FLOAT64
**
** INT64
** UINT64
**
** 64-bit floating point type according to IEC 60559 / IEEE 754.
** 64-bit signed and unsigned integer types.
**
** Those types are optional and support for them must also be enabled in
** "abcc_driver_config.h" if they are to be used. See the comments in
** "abcc_config.h" for more information.
**---------------------------------------------------------------------------
*/
typedef double          FLOAT64;
typedef int64_t         INT64;
typedef uint64_t        UINT64;

/*------------------------------------------------------------------------------
** LeINT16
** LeINT32
** LeINT64
** LeUINT16
** LeUINT32
** LeUINT64
**
** Little endian data types for words and longwords.
**------------------------------------------------------------------------------
*/
typedef INT16     LeINT16;
typedef INT32     LeINT32;
typedef INT64     LeINT64;
typedef UINT16    LeUINT16;
typedef UINT32    LeUINT32;
typedef UINT64    LeUINT64;

/*------------------------------------------------------------------------------
** BeINT16
** BeINT32
** BeINT64
** BeUINT16
** BeUINT32
** BeUINT64
**
** Big endian data types for words and longwords.
**------------------------------------------------------------------------------
*/
typedef INT16     BeINT16;
typedef INT32     BeINT32;
typedef INT64     BeINT64;
typedef UINT16    BeUINT16;
typedef UINT32    BeUINT32;
typedef UINT64    BeUINT64;

/*---------------------------------------------------------------------------
**
** FALSE
** TRUE
**
** These are the symbolic constants for true and false used in boolean
** data type comparisons.
**
**---------------------------------------------------------------------------
*/
#ifndef FALSE
   #define FALSE     0
#endif

#ifndef TRUE
   #define TRUE      ( !FALSE )
#endif

/*---------------------------------------------------------------------------
**
** NULL
**
** Default value for invalid pointers.
**
**---------------------------------------------------------------------------
*/
#ifndef NULL
#define NULL 0
#endif

#ifdef __cplusplus
   #define CPLUSPLUS
#endif

#ifdef CPLUSPLUS
   #define EXTFUNC extern "C"
   #define EXTVAR extern "C"
#else
   #define EXTFUNC extern
   #define EXTVAR extern
#endif

/*---------------------------------------------------------------------------
** Compiler/platform-specific packing macros.
**
** The macros ABCC_SYS_PACK_ON, ABCC_SYS_PACK_OFF and PACKED_STRUCT must be
** defined if the compiler/platform in use does not pack (byte-align) data by
** default. This is relevant for the message-related typedef:s and struct:s
** in "abp.h", but may also be used elsewere in the driver and example code.
**---------------------------------------------------------------------------
*/
#if defined( __GNUC__ )
   /*
   ** https://gcc.gnu.org/onlinedocs/cpp/Common-Predefined-Macros.html
   ** https://gcc.gnu.org/onlinedocs/gcc-10.2.0/gcc/Common-Type-Attributes.html
   */
   #define ABCC_SYS_PACK_ON
   #define ABCC_SYS_PACK_OFF
   #define PACKED_STRUCT       __attribute__((packed))
#else
   #error No packing macros defined!
#endif /* Packing macros */

#endif  /* inclusion lock */
