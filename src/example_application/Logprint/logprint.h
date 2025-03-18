/*******************************************************************************
** Copyright 2015-present HMS Industrial Networks AB.
** Licensed under the MIT License.
********************************************************************************
** File Description:
** Header file for LogPrint
********************************************************************************
*/

#ifndef LOGPRINT_H_
#define LOGPRINT_H_

/*------------------------------------------------------------------------------
** Sets the stream to use for the console, normally 'stdout' or 'stderr'.
** Setting the stream to NULL will disable printouts.
**------------------------------------------------------------------------------
** Arguments:
**    pxStream - Stream to use for the console.
**
** Returns:
**    None.
**------------------------------------------------------------------------------
*/
EXTFUNC void LOGPRINT_SetConsoleStream( FILE* pxStream );

/*------------------------------------------------------------------------------
** Sets the stream to use for the log file, the caller is responsible for
** opening and closing the the stream. Setting the stream to NULL will disable
** printouts.
**------------------------------------------------------------------------------
** Arguments:
**    pxStream - Stream to use for the logfile.
**
** Returns:
**    None.
**------------------------------------------------------------------------------
*/
EXTFUNC void LOGPRINT_SetLogFileStream( FILE* pxStream );

/*------------------------------------------------------------------------------
** printf()-equivalent that sends the output to one or more streams.
**------------------------------------------------------------------------------
** Arguments:
**    printf()-style arguments.
**
** Returns:
**    Number of printed characters.
**------------------------------------------------------------------------------
*/
EXTFUNC int LOGPRINT_Printf( const char *pacFormat, ... );

/*------------------------------------------------------------------------------
** Produces a platform-dependent timestamp on the output streams.
**------------------------------------------------------------------------------
** Arguments:
**    None.
**
** Returns:
**    TRUE on success, FALSE otherwise.
**------------------------------------------------------------------------------
*/
#if ( defined( __linux__ ) && defined( __GNUC__ ) ) || \
    ( defined( _WIN32 )    && defined( _MSC_VER ) )
EXTFUNC BOOL LOGPRINT_TimeStamp( void );
#endif

#endif /* LOGPRINT_H */
