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
********************************************************************************
*/

#include "abcc.h"
#include "abcc_hardware_abstraction.h"
#include "abcc_hardware_abstraction_spi.h"
#include "abcc_hardware_abstraction_parallel.h"
#include "abcc_hardware_abstraction_serial.h"
#include "abcc_log.h"
#include "abcc_api.h"

#include <errno.h>
#include <gpiod.h>
#include <pthread.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <termios.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>

#define HAL_LINUX_CLOCK_TYPE                 CLOCK_MONOTONIC_RAW
#define HAL_TIMER_INTERVAL_MS                50U

#define HAL_GPIOD_GPIO_NUM_OF                10
#define HAL_GPIOD_CONSUMER_NAME              "abcc"

#define HAL_SPI_BITS_PER_SECOND              10000000UL
#define HAL_SPI_BITS_PER_UNIT                8U

#define HAL_INVALID_MODULE_ID                255U
#define HAL_INVALID_OPMODE                   255U

#define HAL_PLATFORM_TYPE_FILE_NAME          "/sys/firmware/devicetree/base/model"
#define HAL_PLATFORM_TYPE_STRING_MAX_LEN     64

#define HAL_GPIO_RESET 17
#define HAL_GPIO_OM0   5
#define HAL_GPIO_OM1   6
#define HAL_GPIO_OM2   13
#define HAL_GPIO_OM3   19
#define HAL_GPIO_MD0   27
#define HAL_GPIO_MD1   22
#define HAL_GPIO_MI0   23
#define HAL_GPIO_MI1   24
#define HAL_GPIO_IRQ   26

/*
** PORTING ALERT!
**
** This list has to be expanded if support for other Linux SBCs are added.
*/

typedef enum hal_PlatformTag
{
   PLATFORM_TYPE_UNKNOWN = 0,
   PLATFORM_TYPE_RASPI_ZW,
   PLATFORM_TYPE_RASPI_Z2W,
   PLATFORM_TYPE_RASPI_3BP,
   PLATFORM_TYPE_RASPI_4B,
   PLATFORM_TYPE_RASPI_CM4,
   PLATFORM_TYPE_RASPI_5B,
   PLATFORM_TYPE_NUM_OF
} hal_PlatformType;

/*------------------------------------------------------------------------------
** List type for GPIO setup, used by hal_GpioInit() and
** hal_GpioShutdown().
**------------------------------------------------------------------------------
*/

typedef struct hal_GpioCfgEntryTag
{
   const unsigned int xLineNum;
   const int xRequestType;
   const int xFlags;
   const int xDefaultState;
} hal_GpioCfgEntryType;

/*------------------------------------------------------------------------------
** List that holds the device names and GPIO settings for a specific platform.
**------------------------------------------------------------------------------
*/

/*
** PORTING ALERT!
**
** This list may have to be expanded if support for other Linux SBCs are added.
*/

typedef struct hal_PlatformCfgEntryTag
{
   const char* const acGpioDeviceName;
   const char* const acSpiDeviceName;
   const char* const acUartDeviceName;
   const hal_GpioCfgEntryType* const psGpioCfgList;
} hal_PlatformCfgEntryType;

/*------------------------------------------------------------------------------
** GPIO-related references.
**------------------------------------------------------------------------------
*/

static struct gpiod_chip*           hal_psGpioChip = NULL;
static struct gpiod_request_config* hal_psGpioReqConf = NULL;
static struct gpiod_line_config*    hal_psGpioLineConf = NULL;
static struct gpiod_line_settings*  hal_psGpioLineSettings = NULL;
#define hal_psGpioLineSettgs hal_psGpioLineSettings
static struct gpiod_line_request*   hal_psGpioReqLines = NULL;

/*------------------------------------------------------------------------------
** Storage variables for the application interface parts.
**------------------------------------------------------------------------------
*/

static UINT8 hal_bPresentOpmode = HAL_INVALID_OPMODE;

static int hal_xSpiFd = -1;
struct spi_ioc_transfer hal_sSpiTransfer;
static ABCC_HAL_SpiDataReceivedCbfType hal_pnSpiDataReadyCbf = NULL;

static int hal_xUartFd = -1;
static ABCC_HAL_SerDataReceivedCbfType hal_pnSerDataReadyCbf = NULL;

/*------------------------------------------------------------------------------
** Status flags and locks for the threaded parts.
**------------------------------------------------------------------------------
*/

static pthread_mutex_t hal_xThreadFlagsLock;
static BOOL hal_fFlagsLockInitialised = FALSE;

static BOOL hal_fRunThreads = FALSE;

static pthread_mutex_t hal_xTimerLock;
static BOOL hal_fTimerLockInitialised = FALSE;

static pthread_t hal_xTimerThread;
static BOOL hal_fTimerThreadStarted = FALSE;

/*------------------------------------------------------------------------------
** Variables and lists for platform detection, and platform-specific settings
** for the GPIO mapping and device names.
**------------------------------------------------------------------------------
*/

static const hal_PlatformCfgEntryType* hal_psPlatformCfg = NULL;

/*
** PORTING ALERT!
**
** The present tables are OK for Raspberry Pi Zero W, Zero 2 W, 4B,
** Compute Module 4, and 5B. New strings, entries, and tables has to be added in
** order to support other Linux SBCs.
*/

static const char* const hal_aacMatchStrings[ PLATFORM_TYPE_NUM_OF ] =
{
   /*
   ** NOTE: The first entry should be blank, it corresponds to the
   ** PLATFORM_TYPE_UNKNOWN enum value and is not used during lookup.
   */
   "",
   "Raspberry Pi Zero W Rev 1.1",
   "Raspberry Pi Zero 2 W Rev 1.0",
   "Raspberry Pi 3 Model B Plus",
   "Raspberry Pi 4 Model B",
   "Raspberry Pi Compute Module 4",
   "Raspberry Pi 5 Model B"
};

static hal_GpioCfgEntryType hal_asGpioCfgRasPi[ HAL_GPIOD_GPIO_NUM_OF ] =
{
/*
**  xLineNum       , xRequestType               , xFlags                  , xDefaultState
*/
   { HAL_GPIO_RESET, GPIOD_LINE_DIRECTION_OUTPUT, GPIOD_LINE_BIAS_DISABLED, GPIOD_LINE_VALUE_INACTIVE },
   { HAL_GPIO_OM0,   GPIOD_LINE_DIRECTION_OUTPUT, GPIOD_LINE_BIAS_DISABLED, GPIOD_LINE_VALUE_ACTIVE   },
   { HAL_GPIO_OM1,   GPIOD_LINE_DIRECTION_OUTPUT, GPIOD_LINE_BIAS_DISABLED, GPIOD_LINE_VALUE_ACTIVE   },
   { HAL_GPIO_OM2,   GPIOD_LINE_DIRECTION_OUTPUT, GPIOD_LINE_BIAS_DISABLED, GPIOD_LINE_VALUE_ACTIVE   },
   { HAL_GPIO_OM3,   GPIOD_LINE_DIRECTION_OUTPUT, GPIOD_LINE_BIAS_DISABLED, GPIOD_LINE_VALUE_ACTIVE   },
   { HAL_GPIO_MD0,   GPIOD_LINE_DIRECTION_INPUT,  GPIOD_LINE_BIAS_PULL_UP,  GPIOD_LINE_VALUE_INACTIVE },
   { HAL_GPIO_MD1,   GPIOD_LINE_DIRECTION_INPUT,  GPIOD_LINE_BIAS_PULL_UP,  GPIOD_LINE_VALUE_INACTIVE },
   { HAL_GPIO_MI0,   GPIOD_LINE_DIRECTION_INPUT,  GPIOD_LINE_BIAS_DISABLED, GPIOD_LINE_VALUE_INACTIVE },
   { HAL_GPIO_MI1,   GPIOD_LINE_DIRECTION_INPUT,  GPIOD_LINE_BIAS_DISABLED, GPIOD_LINE_VALUE_INACTIVE },
   { HAL_GPIO_IRQ,   GPIOD_LINE_DIRECTION_INPUT,  GPIOD_LINE_BIAS_PULL_UP,  GPIOD_LINE_VALUE_INACTIVE }
};

static hal_PlatformCfgEntryType hal_sRasPi34Cfg =
{
   "/dev/gpiochip0",
   "/dev/spidev0.0",
   "/dev/ttyS0",
   &hal_asGpioCfgRasPi[ 0 ]
};

static hal_PlatformCfgEntryType hal_sRasPi5Cfg =
{
   "/dev/gpiochip4",
   "/dev/spidev0.0",
   "/dev/ttyAMA0",
   &hal_asGpioCfgRasPi[ 0 ]
};

/*------------------------------------------------------------------------------
** Determine which platform we run on, required for the automatic setup of
** GPIO and SPI/UART devices.
**------------------------------------------------------------------------------
** Inputs:
**    -
** Outputs:
**    pePlatform - pointer to enum to store result in.
** Returns:
**    TRUE if the lookup could complete, FALSE otherwise.
**------------------------------------------------------------------------------
*/
static BOOL hal_CheckPlatform( hal_PlatformType* const pePlatform )
{
   char  acPlatformString[ HAL_PLATFORM_TYPE_STRING_MAX_LEN ];
   FILE* pxFile;
   int   xIndex;
   int   xErrnoCopy;

   if( pePlatform == NULL )
   {
      return( FALSE );
   }

   memset( acPlatformString, 0, HAL_PLATFORM_TYPE_STRING_MAX_LEN );

   /*
   ** PORTING ALERT!
   **
   ** Presently there is only one detection mechanism implemented here,
   ** checking the contents of the "/sys/firmware/devicetree/base/model"
   ** file. This shall work with Rasperry Pi:s, but other Linux SBCs may
   ** require other files to be checked, or entirely different platform
   ** detection mechanisms.
   */

   pxFile = fopen( HAL_PLATFORM_TYPE_FILE_NAME, "r" );
   if( pxFile == NULL )
   {
      xErrnoCopy = errno;
      fprintf( stderr, "HAL: fopen(platform) failed - %s\n", ABCC_HAL_GetErrMsg( xErrnoCopy ) );
      return( FALSE );
   }

   if( fgets( acPlatformString, HAL_PLATFORM_TYPE_STRING_MAX_LEN, pxFile ) == NULL )
   {
      xErrnoCopy = errno;
      fprintf( stderr, "HAL: fgets(platform) failed - %s\n", ABCC_HAL_GetErrMsg( xErrnoCopy ) );
      fclose( pxFile );
      return( FALSE );
   }

   if( fclose( pxFile ) < 0 )
   {
      xErrnoCopy = errno;
      fprintf( stderr, "HAL: fclose(platform) failed - %s\n", ABCC_HAL_GetErrMsg( xErrnoCopy ) );
      return( FALSE );
   }

   /*
   ** Start strncmp() at index 1, index 0 is reseved for
   ** 'PLATFORM_TYPE_UNKNOWN'.
   */
   for( xIndex = 1; xIndex < PLATFORM_TYPE_NUM_OF; xIndex++ )
   {
      if( strncmp( hal_aacMatchStrings[ xIndex ], acPlatformString, strlen( hal_aacMatchStrings[ xIndex ] ) ) == 0 )
      {
         *pePlatform = xIndex;
         return( TRUE );
      }
   }

   *pePlatform = PLATFORM_TYPE_UNKNOWN;

   return( TRUE );
}

/*------------------------------------------------------------------------------
** Open the GPIO device, and allocate and initialise GPIO lines according to
** the list given by private global "hal_psPlatformCfg".
** Requires libgpiod3 (V2.x.x)
**------------------------------------------------------------------------------
** Inputs:
**    -
** Outputs:
**    -
** Returns:
**    TRUE on success, FALSE on failure.
**------------------------------------------------------------------------------
*/
static BOOL hal_GpioInit( void )
{
   int   xIndex;
   int   xErrnoCopy;

   /*
   ** Get a reference for the GPIO chip we should use.
   */
   hal_psGpioChip = gpiod_chip_open( hal_psPlatformCfg->acGpioDeviceName );
   if ( hal_psGpioChip == NULL )
   {
       xErrnoCopy = errno;
       fprintf( stderr, "HAL: gpiod_chip_open() failed - %s\n", ABCC_HAL_GetErrMsg( xErrnoCopy ) );
       return( FALSE );
   }

   /*
   ** Get a reference for the GPIO request configuration and initialize the consumer name.
   */
   hal_psGpioReqConf = gpiod_request_config_new();
   if ( hal_psGpioReqConf == NULL )
   {
      xErrnoCopy = errno;
      fprintf( stderr, "HAL: gpiod_request_config_new() failed - %s\n", ABCC_HAL_GetErrMsg( xErrnoCopy ) );
      return( FALSE );
   }
   gpiod_request_config_set_consumer( hal_psGpioReqConf, HAL_GPIOD_CONSUMER_NAME );

   /*
   ** Get a reference for the GPIO line configuration.
   */
   hal_psGpioLineConf = gpiod_line_config_new();
   if ( hal_psGpioLineConf == NULL )
   {
      xErrnoCopy = errno;
      fprintf( stderr, "HAL: gpiod_line_config_new() failed - %s\n", ABCC_HAL_GetErrMsg( xErrnoCopy ) );
      return( FALSE );
   }

   /*
   ** Get a reference for the GPIO line settings.
   */
   hal_psGpioLineSettgs = gpiod_line_settings_new();
   if ( hal_psGpioLineSettgs == NULL )
   {
      xErrnoCopy = errno;
      fprintf( stderr, "HAL: gpiod_line_settings_new() failed - %s\n", ABCC_HAL_GetErrMsg( xErrnoCopy ) );
      return( FALSE );
   }

   for( xIndex = 0; xIndex < HAL_GPIOD_GPIO_NUM_OF; xIndex++ )
   {
      /*
      ** Populate GPIO line setting with direction, bias flags, default state.
      */
      if ( gpiod_line_settings_set_direction( hal_psGpioLineSettgs, hal_psPlatformCfg->psGpioCfgList[xIndex].xRequestType ) < 0 )
      {
         xErrnoCopy = errno;
         fprintf( stderr, "HAL: gpiod_line_settings_set_direction() failed - %s\n", ABCC_HAL_GetErrMsg( xErrnoCopy ) );
         return( FALSE );
      }

      if ( gpiod_line_settings_set_bias( hal_psGpioLineSettgs, hal_psPlatformCfg->psGpioCfgList[xIndex].xFlags ) < 0 )
      {
         xErrnoCopy = errno;
         fprintf( stderr, "HAL: gpiod_line_settings_set_bias() failed - %s\n", ABCC_HAL_GetErrMsg( xErrnoCopy ) );
         return( FALSE );
      }

      if ( gpiod_line_settings_set_output_value( hal_psGpioLineSettgs, hal_psPlatformCfg->psGpioCfgList[xIndex].xDefaultState ) < 0 )
      {
         xErrnoCopy = errno;
         fprintf( stderr, "HAL: gpiod_line_settings_set_output_value() failed - %s\n", ABCC_HAL_GetErrMsg( xErrnoCopy ) );
         return( FALSE );
      }

     /*
      ** Populate GPIO line configuration with the line number and its settings.
      ** Parameter "1" tells to treat one line at a time.
      */
      if ( gpiod_line_config_add_line_settings( hal_psGpioLineConf, &hal_psPlatformCfg->psGpioCfgList[xIndex].xLineNum, 1, hal_psGpioLineSettgs ) < 0 )
      {
         xErrnoCopy = errno;
         fprintf( stderr, "HAL: gpiod_line_config_add_line_settings() failed - %s\n", ABCC_HAL_GetErrMsg( xErrnoCopy ) );
         return( FALSE );
      }

      /*
      ** Set GPIO line settings to default values.
      */
      gpiod_line_settings_reset( hal_psGpioLineSettgs );
   }

   /*
   ** GPIO line settings reference no longer needed.
   */
   if ( hal_psGpioLineSettgs != NULL )
   {
      gpiod_line_settings_free( hal_psGpioLineSettgs );
      hal_psGpioLineSettgs = NULL;
   }

   /*
   ** Request a set of lines for exclusive usage.
   */
   hal_psGpioReqLines = gpiod_chip_request_lines( hal_psGpioChip, hal_psGpioReqConf, hal_psGpioLineConf );
   if ( hal_psGpioReqLines == NULL )
   {
      xErrnoCopy = errno;
      fprintf( stderr, "HAL: gpiod_chip_request_lines() failed - %s\n", ABCC_HAL_GetErrMsg( xErrnoCopy ) );
      return( FALSE );
   }

  /*
   ** GPIO request configuration reference no longer needed.
   */
   if ( hal_psGpioReqConf != NULL )
   {
      gpiod_request_config_free( hal_psGpioReqConf );
      hal_psGpioReqConf = NULL;
   }

   /*
   ** GPIO line configuration reference no longer needed.
   */
   if ( hal_psGpioLineConf != NULL )
   {
      gpiod_line_config_free( hal_psGpioLineConf );
      hal_psGpioLineConf = NULL;
   }

  /*
   ** GPIO chip configuration no longer needed,
   ** from here on, we will only need the hal_psGpioReqLines reference.
   */
   if ( hal_psGpioChip != NULL )
   {
      gpiod_chip_close( hal_psGpioChip );
      hal_psGpioChip = NULL;
   }

   return( TRUE );
}

/*------------------------------------------------------------------------------
** Release allocated GPIO lines and close the GPIO device.
**------------------------------------------------------------------------------
** Inputs:
**    -
** Outputs:
**    -
** Returns:
**    TRUE on success, FALSE on failure.
**------------------------------------------------------------------------------
*/
static BOOL hal_GpioShutdown( void )
{
   if ( hal_psGpioReqLines != NULL )
   {
      gpiod_line_request_release( hal_psGpioReqLines );
      hal_psGpioReqLines = NULL;
   }

   return( TRUE );
}

/*------------------------------------------------------------------------------
** Wrapper for gpiod_line_request_get_value(), prints error messages on failure.
**------------------------------------------------------------------------------
** Inputs:
**    iLine        - GPIO line reference
**    pacLineLabel - Text label describing the GPIO line for error printouts
** Outputs:
**    pxValue      - Pointer to int that should hold the GPIO value
** Returns:
**    TRUE on success, FALSE on failure.
**------------------------------------------------------------------------------
*/
static BOOL hal_GpiodGetWrapper( unsigned int iLine, int* pxValue, const char* pacLineLabel )
{
   int xStatus;
   int xErrnoCopy;

   if( ( pxValue == NULL ) || ( pacLineLabel == NULL ) )
   {
      return( FALSE );
   }

   xStatus = gpiod_line_request_get_value( hal_psGpioReqLines, iLine );
   if( xStatus < 0 )
   {
      xErrnoCopy = errno;
      fprintf( stderr, "HAL: gpiod_line_request_get_value(%s) failed - %s\n", pacLineLabel, ABCC_HAL_GetErrMsg( xErrnoCopy ) );
      return( FALSE );
   }

   *pxValue = xStatus;

   return( TRUE );
}

/*------------------------------------------------------------------------------
** Wrapper for gpiod_line_request_set_value(), prints error messages on failure.
**------------------------------------------------------------------------------
** Inputs:
**    iLine        - GPIO line reference
**    xValue       - Output value
**    pacLineLabel - Text label describing the GPIO line for error printouts
** Outputs:
**    -
** Returns:
**    TRUE on success, FALSE on failure.
**------------------------------------------------------------------------------
*/
static BOOL hal_GpiodSetWrapper( unsigned int iLine, int xValue, const char* pacLineLabel )
{
   int xStatus;
   int xErrnoCopy;

   if( pacLineLabel == NULL )
   {
      return( FALSE );
   }

   xStatus = gpiod_line_request_set_value( hal_psGpioReqLines, iLine, xValue );
   if( xStatus < 0 )
   {
      xErrnoCopy = errno;
      fprintf( stderr, "HAL: gpiod_line_request_set_value(%s) failed - %s\n", pacLineLabel, ABCC_HAL_GetErrMsg( xErrnoCopy ) );
      return( FALSE );
   }

   return( TRUE );
}

/*------------------------------------------------------------------------------
** Open and configure the SPI device given by private global
** "hal_psPlatformCfg".
**------------------------------------------------------------------------------
** Inputs:
**    -
** Outputs:
**    -
** Returns:
**    TRUE on success, FALSE on failure.
**------------------------------------------------------------------------------
*/
static BOOL hal_SpiInit( void )
{
   uint8_t  bTemp;
   int      xErrnoCopy;

   if( hal_psPlatformCfg == NULL )
   {
      return( FALSE );
   }

   if( hal_bPresentOpmode != ABP_OP_MODE_SPI )
   {
      return( FALSE );
   }

   hal_xSpiFd = open( hal_psPlatformCfg->acSpiDeviceName, O_RDWR );
   if( hal_xSpiFd < 0 )
   {
      xErrnoCopy = errno;
      fprintf( stderr, "HAL: open(spi) failed - %s\n", ABCC_HAL_GetErrMsg( xErrnoCopy ) );
      return( FALSE );
   }

   bTemp = SPI_MODE_0;

   if( ioctl( hal_xSpiFd, SPI_IOC_RD_MODE, &bTemp ) < 0 ) {
      xErrnoCopy = errno;
      fprintf( stderr, "HAL: ioctl(spi,SPI_IOC_RD_MODE) failed - %s\n", ABCC_HAL_GetErrMsg( xErrnoCopy ) );
      return( FALSE );
   }
   if( ioctl( hal_xSpiFd, SPI_IOC_WR_MODE, &bTemp ) < 0 ) {
      xErrnoCopy = errno;
      fprintf( stderr, "HAL: ioctl(spi,SPI_IOC_WR_MODE) failed - %s\n", ABCC_HAL_GetErrMsg( xErrnoCopy ) );
      return( FALSE );
   }

   memset( &hal_sSpiTransfer, 0, sizeof( hal_sSpiTransfer ) );

   hal_sSpiTransfer.speed_hz = HAL_SPI_BITS_PER_SECOND;
   hal_sSpiTransfer.bits_per_word = HAL_SPI_BITS_PER_UNIT;

   return( TRUE );
}

/*------------------------------------------------------------------------------
** Close the SPI device.
**------------------------------------------------------------------------------
** Inputs:
**    -
** Outputs:
**    -
** Returns:
**    TRUE on success, FALSE on failure.
**------------------------------------------------------------------------------
*/
static BOOL hal_SpiShutdown( void )
{
   int   xErrnoCopy;

   if( hal_bPresentOpmode != ABP_OP_MODE_SPI )
   {
      return( FALSE );
   }

   if( hal_xSpiFd < 0 )
   {
      /*
      ** Nothing to do, device was never opened.
      */
      return( TRUE );
   }

   if( close( hal_xSpiFd ) < 0 )
   {
      xErrnoCopy = errno;
      fprintf( stderr, "HAL: close(spi) failed - %s\n", ABCC_HAL_GetErrMsg( xErrnoCopy ) );
      return( FALSE );
   }
   hal_xSpiFd = -1;

   return( TRUE );
}

/*------------------------------------------------------------------------------
** Open and configure the UART device given by private global
** "hal_psPlatformCfg".
**------------------------------------------------------------------------------
** Inputs:
**    -
** Outputs:
**    -
** Returns:
**    TRUE on success, FALSE on failure.
**------------------------------------------------------------------------------
*/
static BOOL hal_UartInit( void )
{
   struct termios sTermOptions;
   int            xErrnoCopy;

   if( hal_psPlatformCfg == NULL )
   {
      return( FALSE );
   }

   if( ( hal_bPresentOpmode != ABP_OP_MODE_SERIAL_19_2 ) &&
       ( hal_bPresentOpmode != ABP_OP_MODE_SERIAL_57_6 ) &&
       ( hal_bPresentOpmode != ABP_OP_MODE_SERIAL_115_2 ) )
   {
      return( FALSE );
   }

   hal_xUartFd = open( hal_psPlatformCfg->acUartDeviceName, O_RDWR | O_NOCTTY );
   if( hal_xUartFd == -1 )
   {
      xErrnoCopy = errno;
      fprintf( stderr, "HAL: open(uart) failed - %s\n", ABCC_HAL_GetErrMsg( xErrnoCopy ) );
      return( FALSE );
   }

   memset( &sTermOptions, 0, sizeof( sTermOptions ) );

   sTermOptions.c_cflag = CS8 | CLOCAL | CREAD;
   sTermOptions.c_iflag = IGNPAR | IGNBRK;
   sTermOptions.c_oflag = 0;
   sTermOptions.c_lflag = 0;
   sTermOptions.c_cc[ VMIN ] = 0;

   if( tcflush( hal_xUartFd, TCIOFLUSH ) < 0 )
   {
      xErrnoCopy = errno;
      fprintf( stderr, "HAL: tcflush(uart) failed - %s\n", ABCC_HAL_GetErrMsg( xErrnoCopy ) );
      return( FALSE );
   }

   switch( hal_bPresentOpmode )
   {
      case ABP_OP_MODE_SERIAL_19_2:
         sTermOptions.c_cflag |= B19200;
         sTermOptions.c_cc[ VTIME ] = 4;
         break;

      case ABP_OP_MODE_SERIAL_57_6:
         sTermOptions.c_cflag |= B57600;
         sTermOptions.c_cc[ VTIME ] = 2;
         break;

      case ABP_OP_MODE_SERIAL_115_2:
         sTermOptions.c_cflag |= B115200;
         sTermOptions.c_cc[ VTIME ] = 1;
         break;

      case ABP_OP_MODE_SPI:
      case ABP_OP_MODE_SHIFT_REGISTER:
      case ABP_OP_MODE_16_BIT_PARALLEL:
      case ABP_OP_MODE_8_BIT_PARALLEL:
      case ABP_OP_MODE_SERIAL_625:
      default:
         fprintf( stderr, "HAL: invalid opmode - %u\n", hal_bPresentOpmode );
         return( FALSE );
         break;
   }

   if( tcsetattr( hal_xUartFd, TCSANOW, &sTermOptions ) < 0 )
   {
      xErrnoCopy = errno;
      fprintf( stderr, "HAL: tcsetattr(uart) failed - %s\n", ABCC_HAL_GetErrMsg( xErrnoCopy ) );
      return( FALSE );
   }

   return( TRUE );
}

/*------------------------------------------------------------------------------
** Close the UART device.
**------------------------------------------------------------------------------
** Inputs:
**    -
** Outputs:
**    -
** Returns:
**    TRUE on success, FALSE on failure.
**------------------------------------------------------------------------------
*/
static BOOL hal_UartShutdown( void )
{
   int   xErrnoCopy;

   if( ( hal_bPresentOpmode != ABP_OP_MODE_SERIAL_19_2 ) &&
       ( hal_bPresentOpmode != ABP_OP_MODE_SERIAL_57_6 ) &&
       ( hal_bPresentOpmode != ABP_OP_MODE_SERIAL_115_2 ) )
   {
      return( FALSE );
   }

   if( hal_xUartFd < 0 )
   {
      /*
      ** Nothing to do, device was never opened.
      */
      return( TRUE );
   }

   if( close( hal_xUartFd ) < 0 )
   {
      xErrnoCopy = errno;
      fprintf( stderr, "HAL: close(uart) failed - %s\n", ABCC_HAL_GetErrMsg( xErrnoCopy ) );
      return( FALSE );
   }
   hal_xUartFd = -1;

   return( TRUE );
}

/*------------------------------------------------------------------------------
** Fetches the value of the 'run threads' flag.
**------------------------------------------------------------------------------
** Inputs:
**    pfDest - Pointer to BOOL to store value in.
** Outputs:
**    -
** Returns:
**    TRUE if all steps succeeded, FALSE otherwise.
**------------------------------------------------------------------------------
*/
static BOOL hal_GetRunThreadsFlag( BOOL* pfDest )
{
   int   xStatus;

   if( !hal_fFlagsLockInitialised )
   {
      return( FALSE );
   }

   xStatus = pthread_mutex_lock( &hal_xThreadFlagsLock );
   if( xStatus != 0 )
   {
      fprintf( stderr, "HAL: pthread_mutex_lock(flags) failed - %s\n", ABCC_HAL_GetErrMsg( xStatus ) );
      return( FALSE );
   }

   *pfDest = hal_fRunThreads;

   xStatus = pthread_mutex_unlock( &hal_xThreadFlagsLock );
   if( xStatus != 0 )
   {
      fprintf( stderr, "HAL: pthread_mutex_unlock(flags) failed - %s\n", ABCC_HAL_GetErrMsg( xStatus ) );
      return( FALSE );
   }

   return( TRUE );
}

/*------------------------------------------------------------------------------
** Sets the value of the 'run threads' flag.
**------------------------------------------------------------------------------
** Inputs:
**    fValue - New value.
** Outputs:
**    -
** Returns:
**    TRUE if all steps succeeded, FALSE otherwise.
**------------------------------------------------------------------------------
*/
static BOOL hal_SetRunThreadsFlag( BOOL fValue )
{
   int   xStatus;

   if( !hal_fFlagsLockInitialised )
   {
      return( FALSE );
   }

   xStatus = pthread_mutex_lock( &hal_xThreadFlagsLock );
   if( xStatus != 0 )
   {
      fprintf( stderr, "HAL: pthread_mutex_lock(flags) failed - %s\n", ABCC_HAL_GetErrMsg( xStatus ) );
      return( FALSE );
   }

   hal_fRunThreads = fValue;

   xStatus = pthread_mutex_unlock( &hal_xThreadFlagsLock );
   if( xStatus != 0 )
   {
      fprintf( stderr, "HAL: pthread_mutex_unlock(flags) failed - %s\n", ABCC_HAL_GetErrMsg( xStatus ) );
      return( FALSE );
   }

   return( TRUE );
}

/*------------------------------------------------------------------------------
** Timer thread, simulates a periodic interrupt from a HW timer.
**------------------------------------------------------------------------------
** Inputs:
**    -
** Outputs:
**    -
** Returns:
**    -
**------------------------------------------------------------------------------
*/
static void* hal_TimerThread( void* pvPtr )
{
   const struct timespec   sSleepTime = { 0, HAL_TIMER_INTERVAL_MS * 1000000UL };

   uint64_t          llThen, llNow, llDiff;
   struct timespec   sTimeValue;

   BOOL  fRun;
   int   xErrnoCopy;

   (void)pvPtr;

   if( clock_gettime( HAL_LINUX_CLOCK_TYPE, &sTimeValue ) < 0 )
   {
      xErrnoCopy = errno;
      fprintf( stderr, "HAL: clock_gettime(init) failed - %s\n", ABCC_HAL_GetErrMsg( xErrnoCopy ) );
      return( NULL );
   }

   llThen = (uint64_t)sTimeValue.tv_sec * 1000000000UL;
   llThen += (uint64_t)sTimeValue.tv_nsec;

   do
   {
      if( nanosleep( &sSleepTime, NULL ) < 0 )
      {
         xErrnoCopy = errno;
         if( xErrnoCopy != EINTR )
         {
            fprintf( stderr, "HAL: nanosleep() failed - %s\n", ABCC_HAL_GetErrMsg( xErrnoCopy ) );
            return( NULL );
         }
      }

      if( !hal_GetRunThreadsFlag( &fRun ) )
      {
         return( NULL );
      }
      if( !fRun )
      {
         break;
      }

      if( clock_gettime( HAL_LINUX_CLOCK_TYPE, &sTimeValue ) < 0 )
      {
         xErrnoCopy = errno;
         fprintf( stderr, "HAL: clock_gettime(poll) failed - %s\n", ABCC_HAL_GetErrMsg( xErrnoCopy ) );
         return( NULL );
      }

      llNow = (uint64_t)sTimeValue.tv_sec * 1000000000UL;
      llNow += (uint64_t)sTimeValue.tv_nsec;

      llDiff = llNow - llThen;
      if( llDiff > 1000000UL )
      {
         llThen = llNow;
         llDiff /= 1000000UL;

         /*
         ** Make sure that *all* elapsed time is accounted for.
         */
         while( llDiff > 0 )
         {
            if( llDiff > ABP_SINT16_MAX )
            {
               ABCC_API_RunTimerSystem( ABP_SINT16_MAX );
               llDiff -= ABP_SINT16_MAX;
            }
            else
            {
               ABCC_API_RunTimerSystem( (INT16)llDiff );
               llDiff = 0;
            }
         }
      }
   }
   while( TRUE );

   return( NULL );
}

/*------------------------------------------------------------------------------
** See function definition in "abcc_hardware_abstraction_aux.h"
**------------------------------------------------------------------------------
*/
const char* ABCC_HAL_GetErrMsg( int xError )
{
   static const char acUndefined[] = "(no description exists)";
   const char* pcMessage;

   pcMessage = strerrordesc_np( xError );

   if( pcMessage == NULL )
   {
      return( acUndefined );
   }

   return( pcMessage );
}

/*------------------------------------------------------------------------------
** See function definition in "abcc_hardware_abstraction.h"
**------------------------------------------------------------------------------
*/
BOOL ABCC_HAL_HwInit( void )
{
   UINT8                   bModuleId;
   hal_PlatformType        ePlatform;
   int                     xStatus;

   /*
   ** Check platform type and select the correct I/O configuration
   ** structure.
   */

   if( !hal_CheckPlatform( &ePlatform ) )
   {
      fprintf( stderr, "HAL: could not detect platform\n" );
      return( FALSE );
   }

   /*
   ** PORTING ALERT!
   **
   ** This switch/case has to be expanded if support for other Linux SBCs
   ** are added.
   */

   switch( ePlatform )
   {
      case PLATFORM_TYPE_UNKNOWN:
         fprintf( stderr, "HAL: unknown platform\n" );
         return( FALSE );
         break;

      case PLATFORM_TYPE_RASPI_ZW:
      case PLATFORM_TYPE_RASPI_Z2W:
      case PLATFORM_TYPE_RASPI_3BP:
      case PLATFORM_TYPE_RASPI_4B:
      case PLATFORM_TYPE_RASPI_CM4:
         hal_psPlatformCfg = &hal_sRasPi34Cfg;
         break;

      case PLATFORM_TYPE_RASPI_5B:
         hal_psPlatformCfg = &hal_sRasPi5Cfg;
         break;

      case PLATFORM_TYPE_NUM_OF:
      default:
         fprintf( stderr, "HAL: invalid platform - %d\n", ePlatform );
         return( FALSE );
   }

   /*
   ** Initalise our GPIO lines.
   */

   if( !hal_GpioInit() )
   {
      return( FALSE );
   }

   /*
   ** Check that the ABCC is one that we can talk to.
   */

   bModuleId = ABCC_ReadModuleId();
   if( bModuleId == ABP_MODULE_ID_ACTIVE_ABCC40 )
   {
      hal_bPresentOpmode = ABCC_CFG_ABCC_OP_MODE;
   }
   else
   {
      hal_bPresentOpmode = HAL_INVALID_OPMODE;
      return( FALSE );
   }

   /*
   ** Initialise the relevant interface device depending on the opmode.
   */

   if( hal_bPresentOpmode == ABP_OP_MODE_SPI )
   {
      if( !hal_SpiInit() )
      {
         return( FALSE );
      }
   }
   else if( ( hal_bPresentOpmode == ABP_OP_MODE_SERIAL_19_2 ) ||
            ( hal_bPresentOpmode == ABP_OP_MODE_SERIAL_57_6 ) ||
            ( hal_bPresentOpmode == ABP_OP_MODE_SERIAL_115_2 ) )
   {
      if( !hal_UartInit() )
      {
         return( FALSE );
      }
   }
   else
   {
      fprintf( stderr, "HAL: invalid opmode - %u\n", hal_bPresentOpmode );
      return( FALSE );
   }

   /*
   ** Initialise the locks and start the timer thread.
   */

   xStatus = pthread_mutex_init( &hal_xThreadFlagsLock, PTHREAD_PROCESS_PRIVATE );
   if( xStatus != 0 )
   {
      fprintf( stderr, "HAL: pthread_mutex_init(flags) failed - %s\n", ABCC_HAL_GetErrMsg( xStatus ) );
      return( FALSE );
   }
   hal_fFlagsLockInitialised = TRUE;

   xStatus = pthread_mutex_init( &hal_xTimerLock, NULL );
   if( xStatus != 0 )
   {
      fprintf( stderr, "HAL: pthread_mutex_init(timer) failed - %s\n", ABCC_HAL_GetErrMsg( xStatus ) );
      return( FALSE );
   }
   hal_fTimerLockInitialised = TRUE;

   if( !hal_SetRunThreadsFlag( TRUE ) )
   {
      return( FALSE );
   }

   xStatus = pthread_create( &hal_xTimerThread, NULL, hal_TimerThread, NULL );
   if( xStatus != 0 )
   {
      fprintf( stderr, "HAL: pthread_create(timer) failed - %s\n", ABCC_HAL_GetErrMsg( xStatus ) );
      return( FALSE );
   }
   hal_fTimerThreadStarted = TRUE;

   return( TRUE );
}

/*------------------------------------------------------------------------------
** See function definition in "abcc_hardware_abstraction_aux.h"
**------------------------------------------------------------------------------
*/
BOOL ABCC_HAL_HwShutdown( void )
{
   BOOL  fReturnValue;
   int   xStatus;

   fReturnValue = TRUE;

   ABCC_HAL_HWReset();

   /*
   ** Shut down the timer thread and destroy the locks.
   */

   if( hal_fFlagsLockInitialised )
   {
      if( !hal_SetRunThreadsFlag( FALSE ) )
      {
         fReturnValue = FALSE;
         hal_fRunThreads = FALSE;
      }
   }

   if( hal_fTimerThreadStarted )
   {
      xStatus = pthread_join( hal_xTimerThread, NULL );
      if( xStatus != 0 )
      {
         fprintf( stderr, "HAL: pthread_join(timer) failed - %s\n", ABCC_HAL_GetErrMsg( xStatus ) );
         fReturnValue = FALSE;
      }
   }
   hal_fTimerThreadStarted = FALSE;

   if( hal_fTimerLockInitialised )
   {
      xStatus = pthread_mutex_destroy( &hal_xTimerLock );
      if( xStatus != 0 )
      {
         fprintf( stderr, "HAL: pthread_mutex_destroy(timer) failed - %s\n", ABCC_HAL_GetErrMsg( xStatus ) );
         fReturnValue = FALSE;
      }
   }
   hal_fTimerLockInitialised = FALSE;

   if( hal_fFlagsLockInitialised )
   {
      xStatus = pthread_mutex_destroy( &hal_xThreadFlagsLock );
      if( xStatus != 0 )
      {
         fprintf( stderr, "HAL: pthread_mutex_destroy(flags) failed - %s\n", ABCC_HAL_GetErrMsg( xStatus ) );
         fReturnValue = FALSE;
      }
   }
   hal_fFlagsLockInitialised = FALSE;

   /*
   ** Shut down the communication interface.
   */

   if( hal_bPresentOpmode == ABP_OP_MODE_SPI )
   {
      if( !hal_SpiShutdown() )
      {
         fReturnValue = FALSE;
      }
      hal_pnSpiDataReadyCbf = NULL;
   }
   else if( ( hal_bPresentOpmode == ABP_OP_MODE_SERIAL_19_2 ) ||
            ( hal_bPresentOpmode == ABP_OP_MODE_SERIAL_57_6 ) ||
            ( hal_bPresentOpmode == ABP_OP_MODE_SERIAL_115_2 ) )
   {
      if( !hal_UartShutdown() )
      {
         fReturnValue = FALSE;
      }
      hal_pnSerDataReadyCbf = NULL;
   }
   else
   {
      fprintf( stderr, "HAL: invalid opmode - %u\n", hal_bPresentOpmode );
      fReturnValue = FALSE;
   }

   /*
   ** Release the GPIOs.
   */

   if( !hal_GpioShutdown() )
   {
      fReturnValue = FALSE;
   }

   hal_bPresentOpmode = HAL_INVALID_OPMODE;
   hal_psPlatformCfg = NULL;

   return( fReturnValue );
}

/*------------------------------------------------------------------------------
** See function definition in "abcc_hardware_abstraction.h"
**------------------------------------------------------------------------------
*/
BOOL ABCC_HAL_Init( void )
{
   return( TRUE );
}

/*------------------------------------------------------------------------------
** See function definition in "abcc_hardware_abstraction.h"
**------------------------------------------------------------------------------
*/
void ABCC_HAL_Close( void )
{
   return;
}

#if( ABCC_CFG_OP_MODE_SETTABLE )
/*------------------------------------------------------------------------------
** See function definition in "abcc_hardware_abstraction.h"
**------------------------------------------------------------------------------
*/
void ABCC_HAL_SetOpmode( UINT8 bOpmode )
{
   int   xOM0Value;
   int   xOM1Value;
   int   xOM2Value;
   int   xOM3Value;
   
   xOM0Value = ( bOpmode & 0x1 ) != 0;
   xOM1Value = ( bOpmode & 0x2 ) != 0;
   xOM2Value = ( bOpmode & 0x4 ) != 0;
   xOM3Value = ( bOpmode & 0x8 ) != 0;

   hal_GpiodSetWrapper( HAL_GPIO_OM0, xOM0Value, "OM0" );
   hal_GpiodSetWrapper( HAL_GPIO_OM1, xOM1Value, "OM1" );
   hal_GpiodSetWrapper( HAL_GPIO_OM2, xOM2Value, "OM2" );
   hal_GpiodSetWrapper( HAL_GPIO_OM3, xOM3Value, "OM3" );

   return;
}
#endif

#if( ABCC_CFG_OP_MODE_GETTABLE )
/*------------------------------------------------------------------------------
** See function definition in "abcc_hardware_abstraction.h"
**------------------------------------------------------------------------------
*/
UINT8 ABCC_HAL_GetOpmode( void )
{
   /*
   ** Implement according to abcc_hardware_abstraction.h
   */
}
#endif

/*------------------------------------------------------------------------------
** See function definition in "abcc_hardware_abstraction.h"
**------------------------------------------------------------------------------
*/
void ABCC_HAL_HWReset( void )
{
   hal_GpiodSetWrapper( HAL_GPIO_RESET, 0, "RESET" );

   return;
}

/*------------------------------------------------------------------------------
** See function definition in "abcc_hardware_abstraction.h"
**------------------------------------------------------------------------------
*/
void ABCC_HAL_HWReleaseReset( void )
{
   hal_GpiodSetWrapper( HAL_GPIO_RESET, 1, "RESET" );

   return;
}

#if ABCC_CFG_MODULE_ID_PINS_CONN
/*------------------------------------------------------------------------------
** See function definition in "abcc_hardware_abstraction.h"
**------------------------------------------------------------------------------
*/
UINT8 ABCC_HAL_ReadModuleId( void )
{
   int   xTemp;
   UINT8 bModuleId;

   bModuleId = 0;

   if( !hal_GpiodGetWrapper( HAL_GPIO_MI0, &xTemp, "MI0" ) )
   {
      return( HAL_INVALID_MODULE_ID );
   }
   if( xTemp > 0 )
   {
      bModuleId |= 1;
   }

   if( !hal_GpiodGetWrapper( HAL_GPIO_MI1, &xTemp, "MI1" ) )
   {
      return( HAL_INVALID_MODULE_ID );
   }
   if( xTemp > 0 )
   {
      bModuleId |= 2;
   }

   return( bModuleId );
}
#endif

#if( ABCC_CFG_MOD_DETECT_PINS_CONN )
/*------------------------------------------------------------------------------
** See function definition in "abcc_hardware_abstraction.h"
**------------------------------------------------------------------------------
*/
BOOL ABCC_HAL_ModuleDetect( void )
{
   int   xTemp;

   if( !hal_GpiodGetWrapper( HAL_GPIO_MD0, &xTemp, "MD0" ) )
   {
      return( FALSE );
   }
   if( xTemp > 0 )
   {
      return( FALSE );
   }

   if( !hal_GpiodGetWrapper( HAL_GPIO_MD1, &xTemp, "MD1" ) )
   {
      return( FALSE );
   }
   if( xTemp > 0 )
   {
      return( FALSE );
   }

   return( TRUE );
}
#endif

#if( ABCC_CFG_SYNC_ENABLED && ABCC_CFG_USE_ABCC_SYNC_SIGNAL_ENABLED )
/*------------------------------------------------------------------------------
** See function definition in "abcc_hardware_abstraction.h"
**------------------------------------------------------------------------------
*/
void ABCC_HAL_SyncInterruptEnable( void )
{
   /*
   ** Implement according to abcc_hardware_abstraction.h
   */
}
#endif

#if( ABCC_CFG_SYNC_ENABLED && ABCC_CFG_USE_ABCC_SYNC_SIGNAL_ENABLED )
/*------------------------------------------------------------------------------
** See function definition in "abcc_hardware_abstraction.h"
**------------------------------------------------------------------------------
*/
void ABCC_HAL_SyncInterruptDisable( void )
{
   /*
   ** Implement according to abcc_hardware_abstraction.h
   */
}
#endif

#if( ABCC_CFG_INT_ENABLED )
/*------------------------------------------------------------------------------
** See function definition in "abcc_hardware_abstraction.h"
**------------------------------------------------------------------------------
*/
void ABCC_HAL_AbccInterruptEnable( void )
{
   /*
   ** Implement according to abcc_hardware_abstraction.h
   */
}

/*------------------------------------------------------------------------------
** See function definition in "abcc_hardware_abstraction.h"
**------------------------------------------------------------------------------
*/
void ABCC_HAL_AbccInterruptDisable( void )
{
   /*
   ** Implement according to abcc_hardware_abstraction.h
   */
}
#endif

#if( ABCC_CFG_POLL_ABCC_IRQ_PIN_ENABLED )
/*------------------------------------------------------------------------------
** See function definition in "abcc_hardware_abstraction.h"
**------------------------------------------------------------------------------
*/
BOOL ABCC_HAL_IsAbccInterruptActive( void )
{
   int   xTemp;

   if( !hal_GpiodGetWrapper( HAL_GPIO_IRQ, &xTemp, "IRQ" ) )
   {
      return( FALSE );
   }
   if( xTemp > 0 )
   {
      return( FALSE );
   }

   return( TRUE );
}
#endif

#if( ABCC_CFG_DRV_SPI_ENABLED )
/*------------------------------------------------------------------------------
** See function definition in "abcc_hardware_abstraction_spi.h"
**------------------------------------------------------------------------------
*/
void ABCC_HAL_SpiRegDataReceived( ABCC_HAL_SpiDataReceivedCbfType pnDataReceived  )
{
   hal_pnSpiDataReadyCbf = pnDataReceived;

   return;
}

/*------------------------------------------------------------------------------
** See function definition in "abcc_hardware_abstraction_spi.h"
**------------------------------------------------------------------------------
*/
void ABCC_HAL_SpiSendReceive( void* pxSendDataBuffer, void* pxReceiveDataBuffer, UINT16 iLength )
{
   int   xStatus;
   int   xErrnoCopy;

   if( ( pxSendDataBuffer == NULL ) || ( pxReceiveDataBuffer == NULL ) )
   {
      return;
   }
   if( iLength == 0 )
   {
      return;
   }

   hal_sSpiTransfer.tx_buf = (uint64_t)pxSendDataBuffer;
   hal_sSpiTransfer.rx_buf = (uint64_t)pxReceiveDataBuffer;
   hal_sSpiTransfer.len = iLength;

   xStatus = ioctl( hal_xSpiFd, SPI_IOC_MESSAGE(1), &hal_sSpiTransfer );
   if( xStatus < 0 )
   {
      xErrnoCopy = errno;
      fprintf( stderr, "HAL: ioctl(spi) failed - %s\n", ABCC_HAL_GetErrMsg( xErrnoCopy ) );
      return;
   }

   if( hal_pnSpiDataReadyCbf == NULL )
   {
      return;
   }

   hal_pnSpiDataReadyCbf();

   return;
}
#endif

#if ( ( ABCC_CFG_DRV_PARALLEL_ENABLED ) && !ABCC_CFG_MEMORY_MAPPED_ACCESS_ENABLED )
/*------------------------------------------------------------------------------
** See function definition in "abcc_hardware_abstraction_parallel.h"
**------------------------------------------------------------------------------
*/
void ABCC_HAL_ParallelRead( UINT16 iMemOffset, void* pxData, UINT16 iLength )
{
   /*
   ** Implement according to abcc_hardware_abstraction_parallel.h
   */
}

#if( ABCC_CFG_DRV_PARALLEL_ENABLED )
/*------------------------------------------------------------------------------
** See function definition in "abcc_hardware_abstraction_parallel.h"
**------------------------------------------------------------------------------
*/
UINT16 ABCC_HAL_ParallelRead16( UINT16 iMemOffset )
{
   /*
   ** Implement according to abcc_hardware_abstraction_parallel.h
   */
}
#endif

/*------------------------------------------------------------------------------
** See function definition in "abcc_hardware_abstraction_parallel.h"
**------------------------------------------------------------------------------
*/
void ABCC_HAL_ParallelWrite( UINT16 iMemOffset, void* pxData, UINT16 iLength )
{
   /*
   ** Implement according to abcc_hardware_abstraction_parallel.h
   */
}

#if( ABCC_CFG_DRV_PARALLEL_ENABLED )
/*------------------------------------------------------------------------------
** See function definition in "abcc_hardware_abstraction_parallel.h"
**------------------------------------------------------------------------------
*/
void ABCC_HAL_ParallelWrite16( UINT16 iMemOffset, UINT16 piData )
{
   /*
   ** Implement according to abcc_hardware_abstraction_parallel.h
   */
}
#endif

/*------------------------------------------------------------------------------
** See function definition in "abcc_hardware_abstraction_parallel.h"
**------------------------------------------------------------------------------
*/
void* ABCC_HAL_ParallelGetRdPdBuffer( void )
{
   /*
   ** Implement according to abcc_hardware_abstraction_parallel.h
   */
}

/*------------------------------------------------------------------------------
** See function definition in "abcc_hardware_abstraction_parallel.h"
**------------------------------------------------------------------------------
*/
void* ABCC_HAL_ParallelGetWrPdBuffer( void )
{
   /*
   ** Implement according to abcc_hardware_abstraction_parallel.h
   */
}
#endif

#if( ABCC_CFG_DRV_SERIAL_ENABLED )
/*------------------------------------------------------------------------------
** See function definition in "abcc_hardware_abstraction_serial.h"
**------------------------------------------------------------------------------
*/
void ABCC_HAL_SerRegDataReceived( ABCC_HAL_SerDataReceivedCbfType pnDataReceived  )
{
   hal_pnSerDataReadyCbf = pnDataReceived;

   return;
}

/*------------------------------------------------------------------------------
** See function definition in "abcc_hardware_abstraction_serial.h"
**------------------------------------------------------------------------------
*/
void ABCC_HAL_SerSendReceive( void* pxTxDataBuffer, void* pxRxDataBuffer, UINT16 iTxSize, UINT16 iRxSize )
{
   UINT8*   pbDataPtr;
   ssize_t  xCount;
   int      xErrnoCopy;

   if( hal_xUartFd == -1 )
   {
      return;
   }

   /*
   ** Transmission.
   */
   pbDataPtr = (UINT8*)pxTxDataBuffer;
   while( iTxSize > 0 )
   {
      xCount = write( hal_xUartFd, pbDataPtr, iTxSize );
      if( xCount < 0 )
      {
         xErrnoCopy = errno;
         fprintf( stderr, "HAL: write(uart) failed - %s\n", ABCC_HAL_GetErrMsg( xErrnoCopy ) );
         return;
      }
      pbDataPtr += xCount;
      iTxSize -= (UINT16)xCount;
   }

   /*
   ** Reception.
   */
   pbDataPtr = (UINT8*)pxRxDataBuffer;
   while( iRxSize > 0 )
   {
      xCount = read( hal_xUartFd, pbDataPtr, iRxSize );
      if( xCount < 0 )
      {
         xErrnoCopy = errno;
         fprintf( stderr, "HAL: read(uart) failed - %s\n", ABCC_HAL_GetErrMsg( xErrnoCopy ) );
         return;
      }
      else if( xCount == 0 )
      {
         /*
         ** No data available.
         */
         break;
      }
      else
      {
         pbDataPtr += xCount;
         iRxSize -= (UINT16)xCount;
      }
   }

   if( iRxSize == 0 )
   {
      if( hal_pnSerDataReadyCbf == NULL )
      {
         return;
      }

      hal_pnSerDataReadyCbf();
   }

   return;
}

/*------------------------------------------------------------------------------
** See function definition in "abcc_hardware_abstraction_ser.h"
**------------------------------------------------------------------------------
*/
void ABCC_HAL_SerRestart( void )
{
   int   xErrnoCopy;

   if( hal_xUartFd == -1 )
   {
      return;
   }

   if( tcflush( hal_xUartFd, TCIOFLUSH ) < 0 )
   {
      xErrnoCopy = errno;
      fprintf( stderr, "HAL: tcflush(uart) failed - %s\n", ABCC_HAL_GetErrMsg( xErrnoCopy ) );
   }

   return;
}
#endif

/*------------------------------------------------------------------------------
** See function definition in "abcc_hardware_abstraction_aux.h"
**------------------------------------------------------------------------------
*/
void ABCC_HAL_TimerLock( void )
{
   int   xStatus;

   xStatus = pthread_mutex_lock( &hal_xTimerLock );
   if( xStatus != 0 )
   {
      fprintf( stderr, "HAL: pthread_mutex_lock(timer) failed - %s\n", ABCC_HAL_GetErrMsg( xStatus ) );
   }

   return;
}

/*------------------------------------------------------------------------------
** See function definition in "abcc_hardware_abstraction_aux.h"
**------------------------------------------------------------------------------
*/
void ABCC_HAL_TimerUnlock( void )
{
   int   xStatus;

   xStatus = pthread_mutex_unlock( &hal_xTimerLock );
   if( xStatus != 0 )
   {
      fprintf( stderr, "HAL: pthread_mutex_unlock(timer) failed - %s\n", ABCC_HAL_GetErrMsg( xStatus ) );
   }

   return;
}
