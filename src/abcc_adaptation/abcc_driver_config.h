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
** User configuration of the ABCC driver. The configuration parameters are
** documented in the driver's public interface abcc_config.h.
**
** Configuration options with default values defined in abcc_config.h will be
** used unless overriden here.
********************************************************************************
*/

#ifndef ABCC_DRV_CFG_H_
#define ABCC_DRV_CFG_H_

#include "abcc_types.h"
#include "abp.h"

/*------------------------------------------------------------------------------
** SPI and Serial operating modes supported
**------------------------------------------------------------------------------
*/
#define ABCC_CFG_DRV_SPI_ENABLED                   1
#define ABCC_CFG_DRV_SERIAL_ENABLED                1

/*------------------------------------------------------------------------------
** The software can set the selected operating mode to the ABCC host connector.
**------------------------------------------------------------------------------
*/
#define ABCC_CFG_OP_MODE_SETTABLE                  1
#define ABCC_CFG_ABCC_OP_MODE                      ABP_OP_MODE_SPI

/*------------------------------------------------------------------------------
** Module identification configuration
**------------------------------------------------------------------------------
*/
#define ABCC_CFG_MODULE_ID_PINS_CONN               1

/*------------------------------------------------------------------------------
** Module detection configuration
**------------------------------------------------------------------------------
*/
#define ABCC_CFG_MOD_DETECT_PINS_CONN              1

/*------------------------------------------------------------------------------
** Configuration of message handling
**------------------------------------------------------------------------------
*/
#define ABCC_CFG_MAX_NUM_APPL_CMDS                 ( 4 )
#define ABCC_CFG_MAX_NUM_ABCC_CMDS                 ( 4 )
#define ABCC_CFG_MAX_PROCESS_DATA_SIZE             ( 4096 )

/*------------------------------------------------------------------------------
** Interrupt configuration excluding sync
**------------------------------------------------------------------------------
*/
#define ABCC_CFG_POLL_ABCC_IRQ_PIN_ENABLED         1

/*------------------------------------------------------------------------------
** Assume firmware update on initial communication timeout.
**------------------------------------------------------------------------------
*/
#define ABCC_CFG_DRV_ASSUME_FW_UPDATE_ENABLED      1

/*------------------------------------------------------------------------------
** Host application objects to support.
**------------------------------------------------------------------------------
*/
#define EIP_OBJ_ENABLE 1
#define PIR_OBJ_ENABLE 1
#define MOD_OBJ_ENABLE 1
#define ANB_FSI_OBJ_ENABLE 1

/*------------------------------------------------------------------------------
** Command response list
**
** Contains macros describing what command shall be responded to and how.
** See abcc_api_command_handler_lookup.h to find available command response
** macros and associated callback function definitions.
**
** Note: the associated object must be enabled via its _OBJ_ENABLE macro for a
** command response entry to take effect.
**------------------------------------------------------------------------------
*/
#define ABCC_API_COMMAND_RESPONSE_LIST \
   ABCC_APPLICATION_OBJ_SERIAL_NR_GET_CBFUNC, \
   ABCC_APPLICATION_OBJ_PRODUCT_NAME_GET_CBFUNC, \
   ABCC_APPLICATION_OBJ_VENDOR_NAME_GET_VALUE("My Company Name"), \
   ABCC_ETHERNETIP_OBJ_VENDOR_ID_GET_VALUE(0xBEEF), \
   ABCC_ETHERNETIP_OBJ_DEVICE_TYPE_GET_CBFUNC, \
   ABCC_ETHERNETIP_OBJ_PRODUCT_CODE_GET_CBFUNC, \
   ABCC_ETHERNETIP_OBJ_REVISION_GET_VALUE( "\x44\x43" ), \
   ABCC_PROFINET_OBJ_VENDOR_ID_GET_VALUE(0xDEAD), \
   ABCC_PROFINET_OBJ_DEVICE_ID_GET_CBFUNC, \
   ABCC_PROFINET_OBJ_ORDER_ID_GET_CBFUNC, \
   ABCC_APPLICATION_OBJ_FW_AVAILABLE_GET_CBFUNC, \
   ABCC_APPLICATION_OBJ_FW_AVAILABLE_SET_CBFUNC,

/*------------------------------------------------------------------------------
** Debug level configuration
**
** Configures the level of debug logging.
**
** One of:
** ABCC_LOG_SEVERITY_DISABLED
** ABCC_LOG_SEVERITY_FATAL_ENABLED
** ABCC_LOG_SEVERITY_ERROR_ENABLED
** ABCC_LOG_SEVERITY_WARNING_ENABLED
** ABCC_LOG_SEVERITY_INFO_ENABLED
** ABCC_LOG_SEVERITY_DEBUG_ENABLED
**------------------------------------------------------------------------------
*/
#define ABCC_CFG_LOG_SEVERITY         ABCC_LOG_SEVERITY_ERROR_ENABLED
#define ABCC_CFG_LOG_COLORS_ENABLED   1

#endif  /* inclusion lock */
