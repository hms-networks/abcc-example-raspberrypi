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
** Platform dependent macros and functions required by the ABCC driver and
** Anybus objects implementation to be platform independent.
** The description of the macros are found in abcc_port.h. Abcc_port.h is found
** in the public ABCC40 driver interface.
********************************************************************************
*/

#ifndef ABCC_SW_PORT_H_
#define ABCC_SW_PORT_H_

#include <stdarg.h>
#include <stdio.h>
#include "abcc_types.h"
#include "abcc_config.h"
#include "logprint.h"
#include "abcc_hardware_abstraction_aux.h"

#define ABCC_PORT_printf( ... )          LOGPRINT_Printf( __VA_ARGS__ )
#define ABCC_PORT_vprintf( ... )         vprintf( __VA_ARGS__ )

#define ABCC_PORT_UseCritical()
#define ABCC_PORT_EnterCritical()
#define ABCC_PORT_ExitCritical()

#define ABCC_PORT_TIMER_UseCritical()
#define ABCC_PORT_TIMER_EnterCritical()       ABCC_HAL_TimerLock()
#define ABCC_PORT_TIMER_ExitCritical()        ABCC_HAL_TimerUnlock()

#endif  /* inclusion lock */
