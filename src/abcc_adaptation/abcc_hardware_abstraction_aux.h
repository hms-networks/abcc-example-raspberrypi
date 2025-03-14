/*******************************************************************************
** Copyright 2025-present HMS Industrial Networks AB.
** Licensed under the MIT License.
********************************************************************************
** File Description:
** Header file for port-specific SYS_ADAPT data structures and functions.
********************************************************************************
*/
#ifndef ABCC_HAL_AUX_H
#define ABCC_HAL_AUX_H

/*------------------------------------------------------------------------------
** Thread-safe wrapper/variant of 'perror()'.
**------------------------------------------------------------------------------
** Inputs:
**    xError - Error code to get description string for.
** Outputs:
**    -
** Returns:
**    Pointer to string with error description.
**------------------------------------------------------------------------------
*/
EXTFUNC const char *ABCC_HAL_GetErrMsg( int xError );

/*------------------------------------------------------------------------------
** Enter/exit critical section functions for the driver timer.
**------------------------------------------------------------------------------
** Inputs:
**    -
** Outputs:
**    -
** Returns:
**    -
**------------------------------------------------------------------------------
*/
EXTFUNC void ABCC_HAL_TimerLock( void );
EXTFUNC void ABCC_HAL_TimerUnlock( void );

/*------------------------------------------------------------------------------
** Releases system resources (GPIO, SPI, UART, etc.) that was allocated during
** startup by ABCC_HAL_HwInit().
**------------------------------------------------------------------------------
** Inputs:
**    -
** Outputs:
**    -
** Returns:
**    TRUE on success, FALSE if one or more unallocation steps failed.
**------------------------------------------------------------------------------
*/
EXTFUNC BOOL ABCC_HAL_HwShutdown( void );

#endif  /* inclusion lock */
