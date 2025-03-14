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
