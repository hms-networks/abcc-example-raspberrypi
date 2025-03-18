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
********************************************************************************
** COPYRIGHT NOTIFICATION (c) 2022 HMS Industrial Networks AB                 **
**                                                                            **
** This code is the property of HMS Industrial Networks AB.                   **
** The source code may not be reproduced, distributed, or used without        **
** permission. When used together with a product from HMS, permission is      **
** granted to modify, reproduce and distribute the code in binary form        **
** without any restrictions.                                                  **
**                                                                            **
** THE CODE IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND. HMS DOES NOT    **
** WARRANT THAT THE FUNCTIONS OF THE CODE WILL MEET YOUR REQUIREMENTS, OR     **
** THAT THE OPERATION OF THE CODE WILL BE UNINTERRUPTED OR ERROR-FREE, OR     **
** THAT DEFECTS IN IT CAN BE CORRECTED.                                       **
********************************************************************************
** Source file for LogPrint
********************************************************************************
*/

#if defined( __linux__ ) && defined( __GNUC__ )
#include <errno.h>
#include <time.h>
#endif

#if defined( _WIN32 ) && defined( _MSC_VER )
#define VC_EXTRALEAN
#define WIN32_LEAN_AND_MEAN
#define WIN32_EXTRA_LEAN
#include <windows.h>
#endif

#include <stdio.h>
#include <stdarg.h>

#include "../../abcc_adaptation/abcc_types.h"
#include "logprint.h"

#if defined( __linux__ ) && defined( __GNUC__ )
/*
** 20 bytes are needed for "YYYY-MM-DD HH:MM:SS", and '\0'.
*/
#define TIME_STAMP_STRING_SIZE                     ( 20 )
#endif

/*
** Streams / file pointers for the console and log file.
*/
static FILE* pxConsoleStream = NULL;
static FILE* pcFileStream = NULL;

#if defined( __linux__ ) && defined( __GNUC__ )
/*
** Format specifier for strftime(), shall give a "YYYY-MM-DD HH:MM:SS" output.
*/
static const char acTimeFormat[] = "%Y-%m-%d %H:%M:%S";
#endif

/*------------------------------------------------------------------------------
** See declaration in "logprint.h" for more information.
**------------------------------------------------------------------------------
*/
void LOGPRINT_SetConsoleStream( FILE* pxStream )
{
   pxConsoleStream = pxStream;
   return;
}

/*------------------------------------------------------------------------------
** See declaration in "logprint.h" for more information.
**------------------------------------------------------------------------------
*/
void LOGPRINT_SetLogFileStream( FILE* pxStream )
{
   pcFileStream = pxStream;
   return;
}

/*------------------------------------------------------------------------------
** See declaration in "logprint.h" for more information.
**------------------------------------------------------------------------------
*/
int LOGPRINT_Printf( const char *pacFormat, ... )
{
   va_list  xArgs;
   int      xNumOfChars;

   xNumOfChars = 0;

   va_start( xArgs, pacFormat );
   if( pxConsoleStream != NULL )
   {
      xNumOfChars = vfprintf( pxConsoleStream, pacFormat, xArgs );
   }
   va_end( xArgs );

   va_start( xArgs, pacFormat );
   if( pcFileStream != NULL )
   {
      xNumOfChars = vfprintf( pcFileStream, pacFormat, xArgs );
   }
   va_end( xArgs );

   return( xNumOfChars );
}

/*------------------------------------------------------------------------------
** See declaration in "logprint.h" for more information.
**------------------------------------------------------------------------------
*/
#if defined( __linux__ ) && defined( __GNUC__ )
BOOL LOGPRINT_TimeStamp( void )
{
   struct timespec   sTimespecNow;
   struct tm         sTmNow;
   int               xErrnoCopy;
   char              acString[ TIME_STAMP_STRING_SIZE ];

   if( clock_gettime( CLOCK_REALTIME, &sTimespecNow ) < 0 )
   {
      xErrnoCopy = errno;
      fprintf( stderr, "LOGPRINT_TimeStamp() failed, errno = %d!\n", xErrnoCopy );
      return( FALSE );
   }

   /*
   ** NOTE: localtime_r() may require _GNU_SOURCE to be defined in the
   ** makefile or similar.
   */
   localtime_r( &sTimespecNow.tv_sec, &sTmNow );
   strftime( acString, TIME_STAMP_STRING_SIZE, acTimeFormat, &sTmNow );
   LOGPRINT_Printf( "%s.%09lu",acString, sTimespecNow.tv_nsec );

   return( TRUE );
}
#endif
#if defined( _WIN32 ) && defined( _MSC_VER )
BOOL LOGPRINT_TimeStamp( void )
{
   SYSTEMTIME  sTimeNow;

   GetLocalTime( &sTimeNow );
   LOGPRINT_Printf( "%u-%02u-%02u %02u:%02u:%02u.%03u",
      sTimeNow.wYear, sTimeNow.wMonth, sTimeNow.wDay,
      sTimeNow.wHour, sTimeNow.wMinute,  sTimeNow.wSecond, sTimeNow.wMilliseconds );

   return( TRUE );
}
#endif

