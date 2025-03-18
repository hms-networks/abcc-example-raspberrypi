/*******************************************************************************
** Copyright 2025-present HMS Industrial Networks AB.
** Licensed under the MIT License.
********************************************************************************
** File Description:
** Source file containing the main loop, a basic user interface,
** as well as basic timer and serial port handling.
********************************************************************************
*/

#include "abcc_types.h"
#include "abcc.h"
#include "abcc_hardware_abstraction.h"
#include "abcc_adaptation/abcc_hardware_abstraction_aux.h"
#include "abcc_api_select_firmware.h"
#include "abcc_api.h"

#include "example_application/Logprint/logprint.h"

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>

#define CONSOLE_INPUT_BUFFER_SIZE     16

static bool vSetRawConsoleMode( bool fGetSet )
{
   static struct termios   sOriginalTermIOSet;
   struct termios          sRawTermIOSet;
   int                     xErrnoCopy;

   if( fGetSet )
   {
      if( tcgetattr( STDIN_FILENO, &sOriginalTermIOSet ) < 0 )
      {
         xErrnoCopy = errno;
         fprintf( stderr, "tcgetattr() failed when reading 'sOriginalTermIOSet' - %s\n", ABCC_HAL_GetErrMsg( xErrnoCopy ) );
         return( false );
      }

      if( tcgetattr( STDIN_FILENO, &sRawTermIOSet ) < 0 )
      {
         xErrnoCopy = errno;
         fprintf( stderr, "tcgetattr() failed when reading 'sRawTermIOSet' - %s\n", ABCC_HAL_GetErrMsg( xErrnoCopy ) );
         return( false );
      }

      sRawTermIOSet.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
      sRawTermIOSet.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
      sRawTermIOSet.c_cflag &= ~(CSIZE | PARENB);
      sRawTermIOSet.c_cflag |= CS8;

      sRawTermIOSet.c_cc[ VMIN ] = 0;
      sRawTermIOSet.c_cc[ VTIME ] = 0;

      if( tcsetattr( STDIN_FILENO, TCSANOW, &sRawTermIOSet ) < 0 )
      {
         xErrnoCopy = errno;
         fprintf( stderr, "tcsetattr() failed when writing 'sRawTermIOSet' - %s\n", ABCC_HAL_GetErrMsg( xErrnoCopy ) );
         return( false );
      }
   }
   else
   {
      if( tcsetattr( STDIN_FILENO, TCSANOW, &sOriginalTermIOSet ) < 0 )
      {
         xErrnoCopy = errno;
         fprintf( stderr, "tcsetattr() failed when writing 'sOriginalTermIOSet' - %s\n", ABCC_HAL_GetErrMsg( xErrnoCopy ) );
         return( false );
      }
   }

   return( true );
}

static int Init( FILE** const ppxDebugFile )
{
   LOGPRINT_SetConsoleStream( stderr );
   *ppxDebugFile = fopen( "log_file.txt", "a" );
   if( *ppxDebugFile == NULL )
   {
      fprintf( stderr, "failed to open log_file.txt\n" );
      exit( EXIT_FAILURE );
   }
   LOGPRINT_SetLogFileStream( *ppxDebugFile );

   LOGPRINT_Printf( "-------------------------------------------------\n" );
   LOGPRINT_Printf( "Program started at: " );
   LOGPRINT_TimeStamp();
   LOGPRINT_Printf( "\n" );
   LOGPRINT_Printf( "-------------------------------------------------\n" );
   LOGPRINT_Printf( "HMS Networks\n" );
   LOGPRINT_Printf( "Anybus CompactCom Driver API\n" );
   LOGPRINT_Printf( "Example Application: Raspberry Pi\n" );
   LOGPRINT_Printf( "\n" );

   if( !vSetRawConsoleMode( true ) )
   {
      fprintf( stderr, "vSetRawConsoleMode( true ) failed\n" );
      exit( EXIT_FAILURE );
   }

   /*
   ** Function to initialize CompactCom-related systems.
   ** Note: This function in not required to call unless
   ** ABCC_HAL_HwInit() contain anything.
   */
   if( ABCC_API_Init() != ABCC_EC_NO_ERROR )
   {
      // We need to do cleanup, so do not exit directly.
      return( EXIT_FAILURE );
   }

   return( EXIT_SUCCESS );
}

static bool HandleInput( int* const pxReturnVal )
{
   int   xErrnoCopy;
   /*
   ** Check if someone has pressed a key.
   */
   static uint8_t abConsoleInput[ CONSOLE_INPUT_BUFFER_SIZE ];
   const ssize_t xCount = read( STDIN_FILENO, abConsoleInput, CONSOLE_INPUT_BUFFER_SIZE );

   if( xCount > 0 )
   {
      int i;
      for( i = 0; i < xCount; i++ )
      {
         if( abConsoleInput[ i ] == 'q' || abConsoleInput[ i ] == 'Q' )
         {
            return( false );
         }
         else if(abConsoleInput[ i ] == 't' || abConsoleInput[ i ] == 'T')
         {ABCC_API_SelectFirmware(
   ABCC_API_NW_TYPE_PROFINET,
   NULL );}
      }
   }
   else if( xCount == 0 )
   {
      /*
      ** Empty
      */
   }
   else // if( xCount < 0 )
   {
      xErrnoCopy = errno;
      fprintf( stderr, "read( STDIN ) failed - %s\n", ABCC_HAL_GetErrMsg( xErrnoCopy ) );
      *pxReturnVal = EXIT_FAILURE;
      return( false );
   }

   return( true );
}

/*------------------------------------------------------------------------------
** ABCC_API_CbfUserInit()
** Place to take action based on ABCC module network type and firmware version.
** Calls ABCC_API_UserInitComplete() to indicate to the abcc-api to continue.
**------------------------------------------------------------------------------
*/
void ABCC_API_CbfUserInit( ABCC_API_NetworkType iNetworkType, ABCC_API_FwVersionType iFirmwareVersion )
{
   LOGPRINT_Printf( "ABCC_API_CbfUserInit() entered.\n" );
   LOGPRINT_Printf( " - Network type:     0x%X\n", iNetworkType );
   LOGPRINT_Printf( " - Firmware version: %u.%u.%u\n", iFirmwareVersion.bMajor, iFirmwareVersion.bMinor, iFirmwareVersion.bBuild );
   LOGPRINT_Printf( "Now calling ABCC_API_UserInitComplete() to progress from SETUP state to NW_INIT.\n" );
   ABCC_API_UserInitComplete();
   return;
}

int main()
{
   FILE*    pxDebugFile = NULL;
   int      xReturnVal  = Init( &pxDebugFile );
   bool     fRun        = xReturnVal == EXIT_SUCCESS;

   while( fRun )
   {
      /*
      ** Primary function start and drive the abcc-api.
      */
      const ABCC_ErrorCodeType eErrorCode = ABCC_API_Run();

      /*
      ** Handle potential error codes returned from the abcc-api here.
      */
      if( eErrorCode != ABCC_EC_NO_ERROR )
      {
         fprintf( stderr, "ABCC_API_Run() returned error code %d\n", eErrorCode );
         xReturnVal = EXIT_FAILURE;
         fRun = false;
         continue;
      }

      fRun = HandleInput( &xReturnVal );
   }
 
   /*
   ** Shut down the abcc-api and the CompactCom.
   */
   ABCC_API_Shutdown();

   if( !ABCC_HAL_HwShutdown() )
   {
      fprintf( stderr, "One or more shutdown operations failed\n" );
      xReturnVal = EXIT_FAILURE;
   }

   if( !vSetRawConsoleMode( false ) )
   {
      fprintf( stderr, "vSetRawConsoleMode( false ) failed\n" );
      xReturnVal = EXIT_FAILURE;
   }

   LOGPRINT_Printf( "-------------------------------------------------\n" );
   LOGPRINT_Printf( "Program ended at: " );
   LOGPRINT_TimeStamp();
   LOGPRINT_Printf( "\n" );
   LOGPRINT_Printf( "-------------------------------------------------\n" );

   fclose( pxDebugFile );

   return( xReturnVal );
}

