//*****************************************************************************
//
//! @file am_devices_mspi_atxp032.c
//!
//! @brief ATXP032 Multibit SPI Flash driver.
//!
//! @addtogroup atxp032 ATXP032 MSPI Driver
//! @ingroup devices
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
// Copyright (c) 2023, Ambiq Micro, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
//
// Third party software included in this distribution is subject to the
// additional license terms as defined in the /docs/licenses directory.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is part of revision release_sdk_4_4_1-7498c7b770 of the AmbiqSuite Development Package.
//
//*****************************************************************************

#include <string.h>
#include "am_mcu_apollo.h"
#include "am_devices_mspi_atxp032.h"
#include "am_util_stdio.h"
#include "am_bsp.h"
#include "am_util.h"

//*****************************************************************************
//
// Global variables.
//
//*****************************************************************************
//#define ATXP032_QUAD_CLKON4_MODE_EN

//#define USE_NON_DQS_MODE

#define AM_DEVICES_MSPI_ATXP032_TIMEOUT                     1000000
#define AM_DEVICES_MSPI_ATXP032_ERASE_TIMEOUT               1000000
#define AM_DEVICES_MSPI_ATXP032_SECTOR_FOR_TIMING_CHECK     30       // max 31

#define ATXP032_TIMING_SCAN_MIN_ACCEPTANCE_LENGTH           (8)     // there should be at least
                                                                    // this amount of consecutive
                                                                    // passing settings to be accepted.
#define FLASH_CHECK_DATA_SIZE_BYTES                         AM_DEVICES_MSPI_ATXP032_PAGE_SIZE   // Data trunk size
#define FLASH_TIMING_SCAN_SIZE_BYTES                        AM_DEVICES_MSPI_ATXP032_SECTOR_SIZE // Total scan size
#define FLASH_TEST_PATTERN_NUMBER                           5       // 5 patterns

typedef struct
{
    uint32_t                    ui32Module;
    void                        *pMspiHandle;
    am_hal_mspi_dev_config_t    stSetting;
    bool                        bOccupied;
} am_devices_mspi_atxp032_t;

#if defined(AM_PART_APOLLO4_API)
am_hal_mspi_xip_config_t gXipConfig[] =
{
  {
    .ui32APBaseAddr       = MSPI0_APERTURE_START_ADDR,
    .eAPMode              = AM_HAL_MSPI_AP_READ_WRITE,
    .eAPSize              = AM_HAL_MSPI_AP_SIZE64M,
    .scramblingStartAddr  = 0,
    .scramblingEndAddr    = 0,
  },
  {
    .ui32APBaseAddr       = MSPI1_APERTURE_START_ADDR,
    .eAPMode              = AM_HAL_MSPI_AP_READ_WRITE,
    .eAPSize              = AM_HAL_MSPI_AP_SIZE64M,
    .scramblingStartAddr  = 0,
    .scramblingEndAddr    = 0,
  },
  {
    .ui32APBaseAddr       = MSPI2_APERTURE_START_ADDR,
    .eAPMode              = AM_HAL_MSPI_AP_READ_WRITE,
    .eAPSize              = AM_HAL_MSPI_AP_SIZE64M,
    .scramblingStartAddr  = 0,
    .scramblingEndAddr    = 0,
  },
};

am_hal_mspi_config_t gMspiCfg =
{
  .ui32TCBSize          = 0,
  .pTCB                 = NULL,
#if defined(ATXP032_QUAD_CLKON4_MODE_EN)
  .bClkonD4             = 1
#else
  .bClkonD4             = 0
#endif
};
#endif

am_devices_mspi_atxp032_t gAmAtxp032[AM_DEVICES_MSPI_ATXP032_MAX_DEVICE_NUM];

am_hal_mspi_dev_config_t MSPI_ATXP032_Serial_CE0_MSPIConfig =
{
    .eSpiMode             = AM_HAL_MSPI_SPI_MODE_0,
    .eClockFreq           = AM_HAL_MSPI_CLK_16MHZ,
    .ui8TurnAround        = 8,
    .eAddrCfg             = AM_HAL_MSPI_ADDR_4_BYTE,
    .eInstrCfg            = AM_HAL_MSPI_INSTR_1_BYTE,
    .eDeviceConfig        = AM_HAL_MSPI_FLASH_SERIAL_CE0,
    .bSendInstr           = true,
    .bSendAddr            = true,
    .bTurnaround          = true,
#if defined(AM_PART_APOLLO4_API)
  .ui16ReadInstr         = AM_DEVICES_MSPI_ATXP032_FAST_READ,
  .ui16WriteInstr        = AM_DEVICES_MSPI_ATXP032_PAGE_PROGRAM,
#else
  .ui8ReadInstr         = AM_DEVICES_MSPI_ATXP032_FAST_READ,
  .ui8WriteInstr        = AM_DEVICES_MSPI_ATXP032_PAGE_PROGRAM,
#endif
#if defined(AM_PART_APOLLO3P)
    .ui8WriteLatency      = 0,
    .bEnWriteLatency      = false,
    .bEmulateDDR          = false,
    .ui16DMATimeLimit     = 0,
    .eDMABoundary         = AM_HAL_MSPI_BOUNDARY_NONE,
#elif defined(AM_PART_APOLLO4_API)
    .ui8WriteLatency      = 0,
    .bEnWriteLatency      = false,
    .bEmulateDDR          = false,
    .ui16DMATimeLimit     = 0,
    .eDMABoundary         = AM_HAL_MSPI_BOUNDARY_NONE,
#if defined(AM_PART_APOLLO4)
    .eDeviceNum           = AM_HAL_MSPI_DEVICE0,
#endif
#else
    .ui32TCBSize          = 0,
    .pTCB                 = NULL,
    .scramblingStartAddr  = 0,
    .scramblingEndAddr    = 0,
#endif
};

am_hal_mspi_dev_config_t MSPI_ATXP032_Serial_CE1_MSPIConfig =
{
    .eSpiMode             = AM_HAL_MSPI_SPI_MODE_0,
    .eClockFreq           = AM_HAL_MSPI_CLK_24MHZ,
    .ui8TurnAround        = 8,
    .eAddrCfg             = AM_HAL_MSPI_ADDR_4_BYTE,
    .eInstrCfg            = AM_HAL_MSPI_INSTR_1_BYTE,
    .eDeviceConfig        = AM_HAL_MSPI_FLASH_SERIAL_CE1,
    .bSendInstr           = true,
    .bSendAddr            = true,
    .bTurnaround          = true,
#if defined(AM_PART_APOLLO4_API)
  .ui16ReadInstr         = AM_DEVICES_MSPI_ATXP032_FAST_READ,
  .ui16WriteInstr        = AM_DEVICES_MSPI_ATXP032_PAGE_PROGRAM,
#else
  .ui8ReadInstr         = AM_DEVICES_MSPI_ATXP032_FAST_READ,
  .ui8WriteInstr        = AM_DEVICES_MSPI_ATXP032_PAGE_PROGRAM,
#endif
#if defined(AM_PART_APOLLO3P)
    .ui8WriteLatency      = 0,
    .bEnWriteLatency      = false,
    .bEmulateDDR          = false,
    .ui16DMATimeLimit     = 0,
    .eDMABoundary         = AM_HAL_MSPI_BOUNDARY_NONE,
#elif defined(AM_PART_APOLLO4_API)
    .ui8WriteLatency      = 0,
    .bEnWriteLatency      = false,
    .bEmulateDDR          = false,
    .ui16DMATimeLimit     = 0,
    .eDMABoundary         = AM_HAL_MSPI_BOUNDARY_NONE,
#if defined(AM_PART_APOLLO4)
    .eDeviceNum           = AM_HAL_MSPI_DEVICE0,
#endif
#else
    .ui32TCBSize          = 0,
    .pTCB                 = NULL,
    .scramblingStartAddr  = 0,
    .scramblingEndAddr    = 0,
#endif
};

am_hal_mspi_dev_config_t MSPI_ATXP032_Quad_CE0_MSPIConfig =
{
  .eSpiMode             = AM_HAL_MSPI_SPI_MODE_0,
  .eClockFreq           = AM_HAL_MSPI_CLK_24MHZ,
  .ui8TurnAround        = 8,
  .eAddrCfg             = AM_HAL_MSPI_ADDR_4_BYTE,
  .eInstrCfg            = AM_HAL_MSPI_INSTR_1_BYTE,
  .eDeviceConfig        = AM_HAL_MSPI_FLASH_QUAD_CE0,
  .bSendInstr           = true,
  .bSendAddr            = true,
  .bTurnaround          = true,
#if defined(AM_PART_APOLLO4_API)
  .ui16ReadInstr         = AM_DEVICES_MSPI_ATXP032_FAST_READ,
  .ui16WriteInstr        = AM_DEVICES_MSPI_ATXP032_PAGE_PROGRAM,
#else
  .ui8ReadInstr         = AM_DEVICES_MSPI_ATXP032_FAST_READ,
  .ui8WriteInstr        = AM_DEVICES_MSPI_ATXP032_PAGE_PROGRAM,
#endif
#if defined(AM_PART_APOLLO3P)
    .ui8WriteLatency      = 0,
    .bEnWriteLatency      = false,
    .bEmulateDDR          = false,
    .ui16DMATimeLimit     = 0,
    .eDMABoundary         = AM_HAL_MSPI_BOUNDARY_NONE,
#elif defined(AM_PART_APOLLO4_API)
    .ui8WriteLatency      = 0,
    .bEnWriteLatency      = false,
    .bEmulateDDR          = false,
    .ui16DMATimeLimit     = 0,
    .eDMABoundary         = AM_HAL_MSPI_BOUNDARY_NONE,
#else
    .ui32TCBSize          = 0,
    .pTCB                 = NULL,
    .scramblingStartAddr  = 0,
    .scramblingEndAddr    = 0,
#endif
};

am_hal_mspi_dev_config_t MSPI_ATXP032_Quad_CE1_MSPIConfig =
{
    .eSpiMode             = AM_HAL_MSPI_SPI_MODE_0,
    .eClockFreq           = AM_HAL_MSPI_CLK_24MHZ,
    .ui8TurnAround        = 8,
    .eAddrCfg             = AM_HAL_MSPI_ADDR_4_BYTE,
    .eInstrCfg            = AM_HAL_MSPI_INSTR_1_BYTE,
    .eDeviceConfig        = AM_HAL_MSPI_FLASH_QUAD_CE1,
    .bSendInstr           = true,
    .bSendAddr            = true,
    .bTurnaround          = true,
#if defined(AM_PART_APOLLO4_API)
  .ui16ReadInstr         = AM_DEVICES_MSPI_ATXP032_FAST_READ,
  .ui16WriteInstr        = AM_DEVICES_MSPI_ATXP032_PAGE_PROGRAM,
#else
  .ui8ReadInstr         = AM_DEVICES_MSPI_ATXP032_FAST_READ,
  .ui8WriteInstr        = AM_DEVICES_MSPI_ATXP032_PAGE_PROGRAM,
#endif
#if defined(AM_PART_APOLLO3P)
    .ui8WriteLatency      = 0,
    .bEnWriteLatency      = false,
    .bEmulateDDR          = false,
    .ui16DMATimeLimit     = 0,
    .eDMABoundary         = AM_HAL_MSPI_BOUNDARY_NONE,
#elif defined(AM_PART_APOLLO4_API)
    .ui8WriteLatency      = 0,
    .bEnWriteLatency      = false,
    .bEmulateDDR          = false,
    .ui16DMATimeLimit     = 0,
    .eDMABoundary         = AM_HAL_MSPI_BOUNDARY_NONE,
#if defined(AM_PART_APOLLO4)
    .eDeviceNum           = AM_HAL_MSPI_DEVICE0,
#endif
#else
    .ui32TCBSize          = 0,
    .pTCB                 = NULL,
    .scramblingStartAddr  = 0,
    .scramblingEndAddr    = 0,
#endif
};

am_hal_mspi_dev_config_t MSPI_ATXP032_Octal_CE0_MSPIConfig =
{
  .eSpiMode             = AM_HAL_MSPI_SPI_MODE_0,
  .eClockFreq           = AM_HAL_MSPI_CLK_24MHZ,
  .ui8TurnAround        = 8,
  .eAddrCfg             = AM_HAL_MSPI_ADDR_4_BYTE,
  .eInstrCfg            = AM_HAL_MSPI_INSTR_1_BYTE,
  .eDeviceConfig        = AM_HAL_MSPI_FLASH_OCTAL_CE0,
  .bSendInstr           = true,
  .bSendAddr            = true,
  .bTurnaround          = true,
#if defined(AM_PART_APOLLO4_API)
  .ui16ReadInstr         = AM_DEVICES_MSPI_ATXP032_FAST_READ,
  .ui16WriteInstr        = AM_DEVICES_MSPI_ATXP032_PAGE_PROGRAM,
#else
  .ui8ReadInstr         = AM_DEVICES_MSPI_ATXP032_FAST_READ,
  .ui8WriteInstr        = AM_DEVICES_MSPI_ATXP032_PAGE_PROGRAM,
#endif
#if defined(AM_PART_APOLLO3P)
    .ui8WriteLatency      = 0,
    .bEnWriteLatency      = false,
    .bEmulateDDR          = false,
    .ui16DMATimeLimit     = 0,
    .eDMABoundary         = AM_HAL_MSPI_BOUNDARY_NONE,
#elif defined(AM_PART_APOLLO4_API)
    .ui8WriteLatency      = 0,
    .bEnWriteLatency      = false,
    .bEmulateDDR          = false,
    .ui16DMATimeLimit     = 0,
    .eDMABoundary         = AM_HAL_MSPI_BOUNDARY_NONE,
#if defined(AM_PART_APOLLO4)
    .eDeviceNum           = AM_HAL_MSPI_DEVICE0,
#endif
#else
    .ui32TCBSize          = 0,
    .pTCB                 = NULL,
    .scramblingStartAddr  = 0,
    .scramblingEndAddr    = 0,
#endif
};

am_hal_mspi_dev_config_t MSPI_ATXP032_Octal_CE1_MSPIConfig =
{
  .eSpiMode             = AM_HAL_MSPI_SPI_MODE_0,
  .eClockFreq           = AM_HAL_MSPI_CLK_24MHZ,
  .ui8TurnAround        = 8,
  .eAddrCfg             = AM_HAL_MSPI_ADDR_4_BYTE,
  .eInstrCfg            = AM_HAL_MSPI_INSTR_1_BYTE,
  .eDeviceConfig        = AM_HAL_MSPI_FLASH_OCTAL_CE1,
  .bSendInstr           = true,
  .bSendAddr            = true,
  .bTurnaround          = true,
#if defined(AM_PART_APOLLO4_API)
  .ui16ReadInstr         = AM_DEVICES_MSPI_ATXP032_FAST_READ,
  .ui16WriteInstr        = AM_DEVICES_MSPI_ATXP032_PAGE_PROGRAM,
#else
  .ui8ReadInstr         = AM_DEVICES_MSPI_ATXP032_FAST_READ,
  .ui8WriteInstr        = AM_DEVICES_MSPI_ATXP032_PAGE_PROGRAM,
#endif
#if defined(AM_PART_APOLLO3P)
    .ui8WriteLatency      = 0,
    .bEnWriteLatency      = false,
    .bEmulateDDR          = false,
    .ui16DMATimeLimit     = 0,
    .eDMABoundary         = AM_HAL_MSPI_BOUNDARY_NONE,
#elif defined(AM_PART_APOLLO4_API)
    .ui8WriteLatency      = 0,
    .bEnWriteLatency      = false,
    .bEmulateDDR          = false,
    .ui16DMATimeLimit     = 0,
    .eDMABoundary         = AM_HAL_MSPI_BOUNDARY_NONE,
#if defined(AM_PART_APOLLO4)
    .eDeviceNum           = AM_HAL_MSPI_DEVICE0,
#endif
#else
    .ui32TCBSize          = 0,
    .pTCB                 = NULL,
    .scramblingStartAddr  = 0,
    .scramblingEndAddr    = 0,
#endif
};

struct
{
    am_hal_mspi_device_e eHalDeviceEnum;
    am_hal_mspi_dev_config_t *psDevConfig;
}g_ATXP032_DevConfig[] =
{
    {AM_HAL_MSPI_FLASH_SERIAL_CE0,                  &MSPI_ATXP032_Serial_CE0_MSPIConfig},
    {AM_HAL_MSPI_FLASH_SERIAL_CE1,                  &MSPI_ATXP032_Serial_CE1_MSPIConfig},
    {AM_HAL_MSPI_FLASH_QUAD_CE0,                    &MSPI_ATXP032_Quad_CE0_MSPIConfig},
    {AM_HAL_MSPI_FLASH_QUAD_CE1,                    &MSPI_ATXP032_Quad_CE1_MSPIConfig},
    {AM_HAL_MSPI_FLASH_OCTAL_CE0,                   &MSPI_ATXP032_Octal_CE0_MSPIConfig},
    {AM_HAL_MSPI_FLASH_OCTAL_CE1,                   &MSPI_ATXP032_Octal_CE1_MSPIConfig},
};

#if defined(AM_PART_APOLLO4) || defined(AM_PART_APOLLO4B)
//
// SDR timing default setting
//
am_devices_mspi_atxp032_sdr_timing_config_t SDRTimingConfigDefault =
{
    .ui32Turnaround     = 9,
    .ui32Rxneg          = 0,
    .ui32Rxdqsdelay     = 15
};
//
// SDR timing stored setting
//
static bool bSDRTimingConfigSaved = false;
static am_devices_mspi_atxp032_sdr_timing_config_t SDRTimingConfigStored;

#elif defined(AM_PART_APOLLO4P) || defined(AM_PART_APOLLO4L)
//
// SDR timing default setting
//
am_devices_mspi_atxp032_sdr_timing_config_t SDRTimingConfigDefault =
{
    .bTxNeg            = 1,
    .bRxNeg            = 0,
    .bRxCap            = 0,
    .ui8TxDQSDelay     = 4,
    .ui8RxDQSDelay     = 16,
    .ui8Turnaround     = 8,
};
//
// SDR timing stored setting
//
static bool bSDRTimingConfigSaved = false;
static am_devices_mspi_atxp032_sdr_timing_config_t SDRTimingConfigStored;
#endif

#if defined(AM_PART_APOLLO4P) || defined(AM_PART_APOLLO4L)
am_hal_mspi_rxcfg_t gAtxp032MspiRxCfg =
{
    .ui8DQSturn         = 2,
    .bRxHI              = 0,
    .bTaForth           = 0,
    .bHyperIO           = 0,
    .ui8RxSmp           = 1,
    .bRBX               = 0,
    .bWBX               = 0,
    .bSCLKRxHalt        = 0,
    .bRxCapEXT          = 0,
    .ui8Sfturn          = 0,
};

am_hal_mspi_dqs_t gAtxp032DqsCfg[] =
{
  {
#ifdef USE_NON_DQS_MODE
    .bDQSEnable             = 0,
#else
    .bDQSEnable             = 1,
#endif
    .bDQSSyncNeg            = 0,
    .bEnableFineDelay       = 0,
    .ui8TxDQSDelay          = 0,
    .ui8RxDQSDelay          = 16,
    .ui8RxDQSDelayNeg       = 0,
    .bRxDQSDelayNegEN       = 0,
    .ui8RxDQSDelayHi        = 0,
    .ui8RxDQSDelayNegHi     = 0,
    .bRxDQSDelayHiEN        = 0,
  },
  {
#ifdef USE_NON_DQS_MODE
    .bDQSEnable             = 0,
#else
    .bDQSEnable             = 1,
#endif
    .bDQSSyncNeg            = 0,
    .bEnableFineDelay       = 0,
    .ui8TxDQSDelay          = 0,
    .ui8RxDQSDelay          = 16,
    .ui8RxDQSDelayNeg       = 0,
    .bRxDQSDelayNegEN       = 0,
    .ui8RxDQSDelayHi        = 0,
    .ui8RxDQSDelayNegHi     = 0,
    .bRxDQSDelayHiEN        = 0,
  },
  {
#ifdef USE_NON_DQS_MODE
    .bDQSEnable             = 0,
#else
    .bDQSEnable             = 1,
#endif
    .bDQSSyncNeg            = 0,
    .bEnableFineDelay       = 0,
    .ui8TxDQSDelay          = 4,
    .ui8RxDQSDelay          = 16,
    .ui8RxDQSDelayNeg       = 0,
    .bRxDQSDelayNegEN       = 0,
    .ui8RxDQSDelayHi        = 0,
    .ui8RxDQSDelayNegHi     = 0,
    .bRxDQSDelayHiEN        = 0,
  },
};
#endif

#if defined(AM_PART_APOLLO4_API)
//! MSPI interrupts.
static const IRQn_Type mspi_interrupts[] =
{
    MSPI0_IRQn,
    MSPI1_IRQn,
    MSPI2_IRQn,
};

static const uint32_t ui32MspiXipBaseAddress[] =
{
    0x14000000, // mspi0
    0x18000000, // mspi1
    0x1C000000, // mspi2
};
#endif

//
// Forward declarations.
//
static uint32_t am_devices_mspi_atxp032_command_write(void *pHandle,
                                                      uint8_t ui8Instr,
                                                      bool bSendAddr,
                                                      uint32_t ui32Addr,
                                                      uint32_t *pData,
                                                      uint32_t ui32NumBytes);
static uint32_t am_devices_mspi_atxp032_command_read(void *pHandle,
                                                     uint8_t ui8Instr,
                                                     bool bSendAddr,
                                                     uint32_t ui32Addr,
                                                     uint32_t *pData,
                                                     uint32_t ui32NumBytes);


//*****************************************************************************
//
// Adesto ATXP032 Support
//
//*****************************************************************************
//
// Device specific initialization function.
//
static uint32_t
am_device_init_flash(void *pHandle)
{
    uint32_t    ui32PIOBuffer[32] = {0};
    am_devices_mspi_atxp032_t *pFlash = (am_devices_mspi_atxp032_t *)pHandle;

    //
    // Set the Dummy Cycles in Status/Control register 3 to 8 for clock <96MHz and 10 for clock = 96MHz
    //
    am_devices_mspi_atxp032_command_write(pHandle, AM_DEVICES_MSPI_ATXP032_WRITE_ENABLE, false, 0, ui32PIOBuffer, 0);

#if defined(AM_PART_APOLLO4_API)
    if ( pFlash->stSetting.eClockFreq == AM_HAL_MSPI_CLK_96MHZ )
    {
        ui32PIOBuffer[0] = 0x00000103;
    }
    else
#endif

    {
        ui32PIOBuffer[0] = 0x00000003;
    }
    am_devices_mspi_atxp032_command_write(pHandle, AM_DEVICES_ATXP032_WRITE_STATUS_CTRL, false, 0, ui32PIOBuffer, 2);

    //
    // Configure the ATXP032 mode based on the MSPI configuration.
    //
    am_devices_mspi_atxp032_command_write(pHandle, AM_DEVICES_MSPI_ATXP032_WRITE_ENABLE, false, 0, ui32PIOBuffer, 0);

    switch ( pFlash->stSetting.eDeviceConfig )
    {
        case AM_HAL_MSPI_FLASH_SERIAL_CE0:
        case AM_HAL_MSPI_FLASH_SERIAL_CE1:
            am_devices_mspi_atxp032_command_write(pHandle, AM_DEVICES_ATXP032_RETURN_TO_SPI_MODE, false, 0, ui32PIOBuffer, 0);
            break;
        case AM_HAL_MSPI_FLASH_QUAD_CE0:
        case AM_HAL_MSPI_FLASH_QUAD_CE1:
            am_devices_mspi_atxp032_command_write(pHandle, AM_DEVICES_ATXP032_ENTER_QUAD_MODE, false, 0, ui32PIOBuffer, 0);
            break;
        case AM_HAL_MSPI_FLASH_OCTAL_CE0:
        case AM_HAL_MSPI_FLASH_OCTAL_CE1:
            am_devices_mspi_atxp032_command_write(pHandle, AM_DEVICES_ATXP032_ENTER_OCTAL_MODE, false, 0, ui32PIOBuffer, 0);
            break;
        default:
            return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
    }

    return AM_DEVICES_MSPI_ATXP032_STATUS_SUCCESS;
}

//
// Device specific de-initialization function.
//
static uint32_t
am_device_deinit_flash(void *pHandle)
{
    uint32_t      ui32Status;
    am_devices_mspi_atxp032_t *pFlash = (am_devices_mspi_atxp032_t *)pHandle;
    uint32_t      ui32PIOBuffer[32] = {0};

    ui32Status = am_devices_mspi_atxp032_command_write(pHandle, AM_DEVICES_MSPI_ATXP032_WRITE_ENABLE, false, 0, ui32PIOBuffer, 0);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
       return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
    }

    //
    // Configure the Adesto ATXP032 Device mode.
    //
    switch (pFlash->stSetting.eDeviceConfig)
    {
        case AM_HAL_MSPI_FLASH_SERIAL_CE0:
        case AM_HAL_MSPI_FLASH_SERIAL_CE1:
            // Nothing to do.  Device defaults to SPI mode.
            break;
        case AM_HAL_MSPI_FLASH_QUAD_CE0:
        case AM_HAL_MSPI_FLASH_QUAD_CE1:
        case AM_HAL_MSPI_FLASH_OCTAL_CE0:
        case AM_HAL_MSPI_FLASH_OCTAL_CE1:
            ui32Status = am_devices_mspi_atxp032_command_write(pHandle, AM_DEVICES_ATXP032_RETURN_TO_SPI_MODE, false, 0, ui32PIOBuffer, 0);
            if (AM_HAL_STATUS_SUCCESS != ui32Status)
            {
                return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
            }
            break;
        default:
            return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
            //break;
    }

    ui32Status = am_devices_mspi_atxp032_command_write(pHandle, AM_DEVICES_MSPI_ATXP032_WRITE_DISABLE, false, 0, ui32PIOBuffer, 0);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
       return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
    }

    return AM_DEVICES_MSPI_ATXP032_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Generic Command Write function.
//
//*****************************************************************************
static uint32_t
am_devices_mspi_atxp032_command_write(void *pHandle, uint8_t ui8Instr, bool bSendAddr,
                                      uint32_t ui32Addr, uint32_t *pData,
                                      uint32_t ui32NumBytes)
{
    uint32_t ui32Status;
    am_devices_mspi_atxp032_t *pFlash = (am_devices_mspi_atxp032_t *)pHandle;
    am_hal_mspi_pio_transfer_t  stMSPIFlashPIOTransaction = {0};
    am_hal_mspi_dqs_t dqsCfg;
#if defined(AM_PART_APOLLO4P) || defined(AM_PART_APOLLO4L)
    am_hal_mspi_timing_scan_t timingCfg;
#endif

#if defined(AM_PART_APOLLO4_API)
    am_hal_mspi_clock_e clkCfg;

    // Check if we are trying to send the command at 96MHz.
    if (AM_HAL_MSPI_CLK_96MHZ == pFlash->stSetting.eClockFreq)
    {
      clkCfg = AM_HAL_MSPI_CLK_48MHZ;  // Set the clock to 48MHz for commmands.
      ui32Status = am_hal_mspi_control(pFlash->pMspiHandle, AM_HAL_MSPI_REQ_CLOCK_CONFIG, &clkCfg);
      if (AM_HAL_STATUS_SUCCESS != ui32Status)
      {
        return ui32Status;
      }
    }
#endif
    // Create the individual write transaction.
    stMSPIFlashPIOTransaction.ui32NumBytes       = ui32NumBytes;
    stMSPIFlashPIOTransaction.eDirection         = AM_HAL_MSPI_TX;
    stMSPIFlashPIOTransaction.bSendAddr          = bSendAddr;
    stMSPIFlashPIOTransaction.ui32DeviceAddr     = ui32Addr;
    stMSPIFlashPIOTransaction.bSendInstr         = true;
    stMSPIFlashPIOTransaction.ui16DeviceInstr    = ui8Instr;
    stMSPIFlashPIOTransaction.bTurnaround        = false;
#if 0 // Deprecate MSPI CONT for Apollo3
    stMSPIFlashPIOTransaction.bContinue          = false;
#endif
    stMSPIFlashPIOTransaction.pui32Buffer        = pData;

#if defined(AM_PART_APOLLO4_API)
#if defined(AM_PART_APOLLO4)
    stMSPIFlashPIOTransaction.eDeviceNum         = AM_HAL_MSPI_DEVICE0;
#endif
    stMSPIFlashPIOTransaction.bDCX               = false;
    stMSPIFlashPIOTransaction.bEnWRLatency       = false;
   stMSPIFlashPIOTransaction.bContinue           = false;   // MSPI CONT is deprecated for Apollo4
#endif // AM_PART_APOLLO4_API

#if defined(AM_PART_APOLLO4_API)
    if ( ui8Instr == AM_DEVICES_ATXP032_WRITE_STATUS_CTRL )
    {
        // Write status/control register command uses 1 byte address
        am_hal_mspi_instr_addr_t sInstAddrCfg;
        sInstAddrCfg.eAddrCfg = AM_HAL_MSPI_ADDR_1_BYTE;
        sInstAddrCfg.eInstrCfg = pFlash->stSetting.eInstrCfg;   // keep instruction setting the same
        am_hal_mspi_control(pFlash->pMspiHandle, AM_HAL_MSPI_REQ_SET_INSTR_ADDR_LEN, &sInstAddrCfg);
    }

#if defined(AM_PART_APOLLO4) || defined(AM_PART_APOLLO4B)
    // do not use enable fine delay for command read
    dqsCfg.bDQSEnable           = false;
    dqsCfg.bDQSSyncNeg          = false;
    dqsCfg.bEnableFineDelay     = false;
    dqsCfg.bOverrideRXDQSDelay  = false;
    dqsCfg.bOverrideTXDQSDelay  = false;
    dqsCfg.bRxNeg               = 0;
    dqsCfg.ui8DQSDelay          = 0;        // not used
    dqsCfg.ui8PioTurnaround     = 4;        // Read command 0x77 and 0x5A requrest 8 dummy bytes (not supported) others 4
    dqsCfg.ui8XipTurnaround     = 4;        // Read command 0x77 and 0x5A requrest 8 dummy bytes (not supported) others 4
    dqsCfg.ui8RxDQSDelay        = 15;       // not used
    dqsCfg.ui8TxDQSDelay        = 0;
    am_hal_mspi_control(pFlash->pMspiHandle, AM_HAL_MSPI_REQ_DQS, &dqsCfg);
#elif defined(AM_PART_APOLLO4P) || defined(AM_PART_APOLLO4L)
    // do not use enable fine delay for command read
    dqsCfg.bDQSEnable           = false;
    dqsCfg.bDQSSyncNeg          = false;
    dqsCfg.bEnableFineDelay     = false;
    dqsCfg.ui8RxDQSDelayNeg     = 0;
    dqsCfg.bRxDQSDelayNegEN     = false;
    dqsCfg.bRxDQSDelayHiEN      = false;
    dqsCfg.ui8RxDQSDelay        = 16;
    dqsCfg.ui8TxDQSDelay        = 0;
    am_hal_mspi_control(pFlash->pMspiHandle, AM_HAL_MSPI_REQ_DQS, &dqsCfg);

    timingCfg.bTxNeg            = false;
    timingCfg.bRxNeg            = false;
    timingCfg.bRxCap            = false;
    timingCfg.ui8TxDQSDelay     = 0;
    timingCfg.ui8RxDQSDelay     = 16;
    timingCfg.ui8Turnaround     = 4;
    am_hal_mspi_control(pFlash->pMspiHandle, AM_HAL_MSPI_REQ_TIMING_SCAN, &timingCfg);
#endif
#endif


    // Execute the transction over MSPI.
    ui32Status = am_hal_mspi_blocking_transfer(pFlash->pMspiHandle, &stMSPIFlashPIOTransaction,
                                         AM_DEVICES_MSPI_ATXP032_TIMEOUT);

#if defined(AM_PART_APOLLO4_API)
    // restore enable fine delay timing settings
    if ( bSDRTimingConfigSaved == true )
    {
        am_devices_mspi_atxp032_apply_sdr_timing(pFlash, &SDRTimingConfigStored);
    }

    // restore the address length setting
    if ( ui8Instr == AM_DEVICES_ATXP032_WRITE_STATUS_CTRL )
    {
        am_hal_mspi_instr_addr_t sInstAddrCfg;
        sInstAddrCfg.eAddrCfg = pFlash->stSetting.eAddrCfg;
        sInstAddrCfg.eInstrCfg = pFlash->stSetting.eInstrCfg;
        am_hal_mspi_control(pFlash->pMspiHandle, AM_HAL_MSPI_REQ_SET_INSTR_ADDR_LEN, &sInstAddrCfg);
    }
#endif

    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
      return ui32Status;
    }

#if defined(AM_PART_APOLLO4_API)
    // Check if we had to step down the command to 48MHz.
    if (AM_HAL_MSPI_CLK_96MHZ == pFlash->stSetting.eClockFreq)
    {
      clkCfg = AM_HAL_MSPI_CLK_96MHZ;  // Reset the clock to 96MHz.
      ui32Status = am_hal_mspi_control(pFlash->pMspiHandle, AM_HAL_MSPI_REQ_CLOCK_CONFIG, &clkCfg);
    }
#endif
    return ui32Status;
}

//*****************************************************************************
//
// Generic Command Read function.
//
//*****************************************************************************
static uint32_t
am_devices_mspi_atxp032_command_read(void *pHandle, uint8_t ui8Instr, bool bSendAddr,
                                     uint32_t ui32Addr, uint32_t *pData,
                                     uint32_t ui32NumBytes)
{
    uint32_t ui32Status;
    am_devices_mspi_atxp032_t *pFlash = (am_devices_mspi_atxp032_t *)pHandle;
    am_hal_mspi_pio_transfer_t      stMSPIFlashPIOTransaction = {0};

    am_hal_mspi_dqs_t dqsCfg;
#if defined(AM_PART_APOLLO4P) || defined(AM_PART_APOLLO4L)
    am_hal_mspi_timing_scan_t timingCfg;
#endif

#if defined(AM_PART_APOLLO4_API)
    am_hal_mspi_clock_e clkCfg;

    // Check if we are trying to send the command at 96MHz.
    if ((AM_HAL_MSPI_CLK_96MHZ == pFlash->stSetting.eClockFreq)
        && (ui8Instr != AM_DEVICES_ATXP032_ECHO_WITH_INVSERSION))
    {
      clkCfg = AM_HAL_MSPI_CLK_48MHZ;  // Set the clock to 48MHz for commmands.
      ui32Status = am_hal_mspi_control(pFlash->pMspiHandle, AM_HAL_MSPI_REQ_CLOCK_CONFIG, &clkCfg);
      if (AM_HAL_STATUS_SUCCESS != ui32Status)
      {
        return ui32Status;
      }
    }
#endif
    // Create the individual write transaction.
    stMSPIFlashPIOTransaction.eDirection         = AM_HAL_MSPI_RX;
    stMSPIFlashPIOTransaction.bSendAddr          = bSendAddr;
    stMSPIFlashPIOTransaction.ui32DeviceAddr     = ui32Addr;
    stMSPIFlashPIOTransaction.bSendInstr         = true;
    stMSPIFlashPIOTransaction.ui16DeviceInstr    = ui8Instr;

#if defined(AM_PART_APOLLO4_API)
    if ( ui8Instr == AM_DEVICES_MSPI_ATXP032_READ_STATUS )
    {
        // Read status/control register command uses 1 byte address
        am_hal_mspi_instr_addr_t sInstAddrCfg;
        sInstAddrCfg.eAddrCfg = AM_HAL_MSPI_ADDR_1_BYTE;
        sInstAddrCfg.eInstrCfg = pFlash->stSetting.eInstrCfg;   // keep instruction setting the same
        am_hal_mspi_control(pFlash->pMspiHandle, AM_HAL_MSPI_REQ_SET_INSTR_ADDR_LEN, &sInstAddrCfg);
    }
#endif

    if ( (pFlash->stSetting.eDeviceConfig == AM_HAL_MSPI_FLASH_OCTAL_CE0) ||
         (pFlash->stSetting.eDeviceConfig == AM_HAL_MSPI_FLASH_OCTAL_CE1) )
    {
        stMSPIFlashPIOTransaction.bTurnaround    = true;
    }
    else
    {
        stMSPIFlashPIOTransaction.bTurnaround    = false;
    }

#if defined(AM_PART_APOLLO4) || defined(AM_PART_APOLLO4B)
    // do not use enable fine delay for command read
    dqsCfg.bDQSEnable           = false;
    dqsCfg.bDQSSyncNeg          = false;
    dqsCfg.bEnableFineDelay     = false;
    dqsCfg.bOverrideRXDQSDelay  = false;
    dqsCfg.bOverrideTXDQSDelay  = false;
    dqsCfg.bRxNeg               = 0;
    dqsCfg.ui8DQSDelay          = 0;        // not used
    dqsCfg.ui8PioTurnaround     = 4;        // Read command 0x77 and 0x5A requrest 8 dummy bytes (not supported) others 4
    dqsCfg.ui8XipTurnaround     = 4;        // Read command 0x77 and 0x5A requrest 8 dummy bytes (not supported) others 4
    dqsCfg.ui8RxDQSDelay        = 15;       // not used
    dqsCfg.ui8TxDQSDelay        = 0;
    am_hal_mspi_control(pFlash->pMspiHandle, AM_HAL_MSPI_REQ_DQS, &dqsCfg);
#elif defined(AM_PART_APOLLO4P) || defined(AM_PART_APOLLO4L)
    // do not use enable fine delay for command read
    dqsCfg.bDQSEnable           = false;
    dqsCfg.bDQSSyncNeg          = false;
    dqsCfg.bEnableFineDelay     = false;
    dqsCfg.ui8RxDQSDelayNeg     = 0;
    dqsCfg.bRxDQSDelayNegEN     = false;
    dqsCfg.bRxDQSDelayHiEN      = false;
    dqsCfg.ui8RxDQSDelay        = 16;
    dqsCfg.ui8TxDQSDelay        = 0;
    am_hal_mspi_control(pFlash->pMspiHandle, AM_HAL_MSPI_REQ_DQS, &dqsCfg);

    timingCfg.bTxNeg            = false;
    timingCfg.bRxNeg            = false;
    timingCfg.bRxCap            = false;
    timingCfg.ui8TxDQSDelay     = 0;
    timingCfg.ui8RxDQSDelay     = 16;
    timingCfg.ui8Turnaround     = 4;
    am_hal_mspi_control(pFlash->pMspiHandle, AM_HAL_MSPI_REQ_TIMING_SCAN, &timingCfg);
#endif

#if 0 // Deprecate MSPI CONT
    stMSPIFlashPIOTransaction.bContinue          = false;   // MSPI CONT is deprecated for Apollo4
#endif //
    stMSPIFlashPIOTransaction.ui32NumBytes       = ui32NumBytes;
    stMSPIFlashPIOTransaction.pui32Buffer        = pData;

#if defined(AM_PART_APOLLO4_API)
#if defined(AM_PART_APOLLO4)
    stMSPIFlashPIOTransaction.eDeviceNum         = AM_HAL_MSPI_DEVICE0;
#endif
    stMSPIFlashPIOTransaction.bDCX               = false;
    stMSPIFlashPIOTransaction.bEnWRLatency       = false;
   stMSPIFlashPIOTransaction.bContinue           = false;   // MSPI CONT is deprecated for Apollo4
#endif

    // Execute the transction over MSPI.
    ui32Status = am_hal_mspi_blocking_transfer(pFlash->pMspiHandle, &stMSPIFlashPIOTransaction,
                                         AM_DEVICES_MSPI_ATXP032_TIMEOUT);

#if defined(AM_PART_APOLLO4_API)

    // restore the address length setting
    if ( ui8Instr == AM_DEVICES_MSPI_ATXP032_READ_STATUS )
    {
        am_hal_mspi_instr_addr_t sInstAddrCfg;
        sInstAddrCfg.eAddrCfg = pFlash->stSetting.eAddrCfg;
        sInstAddrCfg.eInstrCfg = pFlash->stSetting.eInstrCfg;
        am_hal_mspi_control(pFlash->pMspiHandle, AM_HAL_MSPI_REQ_SET_INSTR_ADDR_LEN, &sInstAddrCfg);
    }

    // restore enable fine delay timing settings
    if ( bSDRTimingConfigSaved == true )
    {
        am_devices_mspi_atxp032_apply_sdr_timing(pFlash, &SDRTimingConfigStored);
    }
#endif


    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
      return ui32Status;
    }

#if defined(AM_PART_APOLLO4_API)
    // Check if we had to step down the command to 48MHz.
    if ((AM_HAL_MSPI_CLK_96MHZ == pFlash->stSetting.eClockFreq)
        && (ui8Instr != AM_DEVICES_ATXP032_ECHO_WITH_INVSERSION))
    {
      clkCfg = AM_HAL_MSPI_CLK_96MHZ;  // Reset the clock to 96MHz.
      ui32Status = am_hal_mspi_control(pFlash->pMspiHandle, AM_HAL_MSPI_REQ_CLOCK_CONFIG, &clkCfg);
    }
#endif
    return ui32Status;
}

static void
pfnMSPI_ATXP032_Callback(void *pCallbackCtxt, uint32_t status)
{
    // Set the DMA complete flag.
    *(volatile uint32_t *)pCallbackCtxt = status;
}

//*****************************************************************************
//
//  Initialize the mspi_flash driver.
//
//*****************************************************************************
uint32_t
am_devices_mspi_atxp032_init(uint32_t ui32Module, am_devices_mspi_atxp032_config_t *psMSPISettings, void **ppHandle, void **ppMspiHandle)
{
    uint32_t      ui32Status;
    am_hal_mspi_dev_config_t *psConfig = g_ATXP032_DevConfig[0].psDevConfig;
    am_hal_mspi_dev_config_t    tempDevCfg;
    void                     *pMspiHandle;
    uint32_t      ui32Index = 0;

    if ((ui32Module > AM_REG_MSPI_NUM_MODULES) || (psMSPISettings == NULL))
    {
        return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
    }

    // Allocate a vacant device handle
    for ( ui32Index = 0; ui32Index < AM_DEVICES_MSPI_ATXP032_MAX_DEVICE_NUM; ui32Index++ )
    {
        if ( gAmAtxp032[ui32Index].bOccupied == false )
        {
            break;
        }
    }
    if ( ui32Index == AM_DEVICES_MSPI_ATXP032_MAX_DEVICE_NUM )
    {
        return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
    }

    bool bFound = false;
    for ( uint32_t i = 0; i < (sizeof(g_ATXP032_DevConfig) / sizeof(g_ATXP032_DevConfig[0])); i++ )
    {
        if ( psMSPISettings->eDeviceConfig == g_ATXP032_DevConfig[i].eHalDeviceEnum )
        {
            psConfig = g_ATXP032_DevConfig[i].psDevConfig;
            psConfig->eClockFreq = psMSPISettings->eClockFreq;
#if !defined(AM_PART_APOLLO4_API)
            psConfig->pTCB = psMSPISettings->pNBTxnBuf;
            psConfig->ui32TCBSize = psMSPISettings->ui32NBTxnBufLength;
            psConfig->scramblingStartAddr = psMSPISettings->ui32ScramblingStartAddr;
            psConfig->scramblingEndAddr = psMSPISettings->ui32ScramblingEndAddr;
#endif
            bFound = true;
            break;
        }
    }

    if ( !bFound )
    {
        am_util_debug_printf("Error - Incorrect eDeviceConfig.\n");
        return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
    }

    //
    // Enable fault detection.
    //
#if defined(AM_PART_APOLLO4_API)
    am_hal_fault_capture_enable();
#elif AM_APOLLO3_MCUCTRL
    am_hal_mcuctrl_control(AM_HAL_MCUCTRL_CONTROL_FAULT_CAPTURE_ENABLE, 0);
#else
    am_hal_mcuctrl_fault_capture_enable();
#endif

    if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_initialize(ui32Module, &pMspiHandle))
    {
        am_util_debug_printf("Error - Failed to initialize MSPI.\n");
        return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
    }
    if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_power_control(pMspiHandle, AM_HAL_SYSCTRL_WAKE, false))
    {
        am_util_debug_printf("Error - Failed to power on MSPI.\n");
        return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
    }

#if defined(AM_PART_APOLLO4_API)
    {
        am_hal_mspi_config_t    mspiCfg = gMspiCfg;
        mspiCfg.ui32TCBSize = psMSPISettings->ui32NBTxnBufLength;
        mspiCfg.pTCB = psMSPISettings->pNBTxnBuf;
        if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_configure(pMspiHandle, &mspiCfg))
        {
            am_util_debug_printf("Error - Failed to configure MSPI device.\n");
            return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
        }
    }
#endif

    //
    // Configure the MSPI for Serial or Quad-Paired Serial operation during initialization.
    //
    switch (psConfig->eDeviceConfig)
    {
        case AM_HAL_MSPI_FLASH_SERIAL_CE0:
        case AM_HAL_MSPI_FLASH_QUAD_CE0:
        case AM_HAL_MSPI_FLASH_OCTAL_CE0:
            gAmAtxp032[ui32Index].stSetting = MSPI_ATXP032_Serial_CE0_MSPIConfig;
            tempDevCfg = MSPI_ATXP032_Serial_CE0_MSPIConfig;

#if defined(AM_PART_APOLLO4_API)
            // Adjust turnaround for the highest clock speed as part of MSPI SW Workaround.
            if (AM_HAL_MSPI_CLK_96MHZ == tempDevCfg.eClockFreq)
            {
                tempDevCfg.ui8TurnAround++;
            }
#endif

            break;

        case AM_HAL_MSPI_FLASH_SERIAL_CE1:
        case AM_HAL_MSPI_FLASH_QUAD_CE1:
        case AM_HAL_MSPI_FLASH_OCTAL_CE1:
            gAmAtxp032[ui32Index].stSetting = MSPI_ATXP032_Serial_CE1_MSPIConfig;
            tempDevCfg = MSPI_ATXP032_Serial_CE1_MSPIConfig;

#if defined(AM_PART_APOLLO4_API)
            // Adjust turnaround for the highest clock speed as part of MSPI SW Workaround.
            if (AM_HAL_MSPI_CLK_96MHZ == tempDevCfg.eClockFreq)
            {
              tempDevCfg.ui8TurnAround++;
            }
#endif
        default:
            return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
            //break;
    }

    if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_device_configure(pMspiHandle, &tempDevCfg))
    {
        am_util_debug_printf("Error - Failed to configure MSPI.\n");
        return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
    }

    if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_enable(pMspiHandle))
    {
        am_util_debug_printf("Error - Failed to enable MSPI.\n");
        return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
    }

#if defined(ATXP032_QUAD_CLKON4_MODE_EN)
    am_bsp_mspi_clkond4_pins_enable(ui32Module, tempDevCfg.eDeviceConfig);
#else
    am_bsp_mspi_pins_enable(ui32Module, tempDevCfg.eDeviceConfig);
#endif

    gAmAtxp032[ui32Index].pMspiHandle = pMspiHandle;
    gAmAtxp032[ui32Index].ui32Module = ui32Module;

    if (AM_HAL_STATUS_SUCCESS != am_devices_mspi_atxp032_reset((void*)&gAmAtxp032[ui32Index]))
    {
        return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
    }
    gAmAtxp032[ui32Index].stSetting = *psConfig;

    //
    // Device specific MSPI Flash initialization.
    //
    ui32Status = am_device_init_flash((void*)&gAmAtxp032[ui32Index]);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
    }
    am_devices_mspi_atxp032_enable_xip((void*)&gAmAtxp032[ui32Index]);
    // Disable MSPI defore re-configuring it
    ui32Status = am_hal_mspi_disable(pMspiHandle);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
    }
    // Adjust turnaround for the highest clock speed as part of MSPI SW Workaround.
    am_hal_mspi_dev_config_t psTempConfig = *psConfig;  // We cannot change the static global config.

#if defined(AM_PART_APOLLO4_API)
    if (AM_HAL_MSPI_CLK_96MHZ == psTempConfig.eClockFreq)
    {
      psTempConfig.ui8TurnAround++;
    }
#endif

    //
    // Re-Configure the MSPI for the requested operation mode.
    //
    ui32Status = am_hal_mspi_device_configure(pMspiHandle, &psTempConfig);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
    }

#if defined(AM_PART_APOLLO4P) || defined(AM_PART_APOLLO4L)
    am_hal_mspi_dqs_t dqsCfg = gAtxp032DqsCfg[ui32Module];
    ui32Status = am_hal_mspi_control(pMspiHandle, AM_HAL_MSPI_REQ_DQS, &dqsCfg);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
      return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
    }

    am_hal_mspi_rxcfg_t RxCfg = gAtxp032MspiRxCfg;
    ui32Status = am_hal_mspi_control(pMspiHandle, AM_HAL_MSPI_REQ_RXCFG, &RxCfg);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
      return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
    }
#endif

    // Re-Enable MSPI
    ui32Status = am_hal_mspi_enable(pMspiHandle);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
    }

    //
    // Configure the MSPI pins.
    //

#if defined(ATXP032_QUAD_CLKON4_MODE_EN)
    am_bsp_mspi_clkond4_pins_enable(ui32Module, psConfig->eDeviceConfig);
#else
    am_bsp_mspi_pins_enable(ui32Module, psConfig->eDeviceConfig);
#endif

    //
    // Enable MSPI interrupts.
    //

    ui32Status = am_hal_mspi_interrupt_clear(pMspiHandle, AM_HAL_MSPI_INT_CQUPD | AM_HAL_MSPI_INT_ERR );
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
    }

//  ui32Status = am_hal_mspi_interrupt_enable(pMspiHandle, AM_HAL_MSPI_INT_CQUPD | AM_HAL_MSPI_INT_ERR | AM_HAL_MSPI_INT_RX_FIFO_FULL);
    ui32Status = am_hal_mspi_interrupt_enable(pMspiHandle, AM_HAL_MSPI_INT_CQUPD | AM_HAL_MSPI_INT_ERR );
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
    }

    //
    // Return the handle.
    //
    gAmAtxp032[ui32Index].bOccupied = true;
    *ppMspiHandle = pMspiHandle;
    *ppHandle = (void *)&gAmAtxp032[ui32Index];

    //
    // Return the status.
    //
    return AM_DEVICES_MSPI_ATXP032_STATUS_SUCCESS;
}

//*****************************************************************************
//
//  De-Initialization the mspi_flash driver.
//
//*****************************************************************************
uint32_t
am_devices_mspi_atxp032_deinit(void *pHandle)
{
    uint32_t      ui32Status;
    am_devices_mspi_atxp032_t *pFlash = (am_devices_mspi_atxp032_t *)pHandle;

    //
    // Device specific MSPI Flash de-initialization.
    //
    ui32Status = am_device_deinit_flash(pHandle);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
    }

    if (AM_HAL_STATUS_SUCCESS != am_devices_mspi_atxp032_reset(pHandle))
    {
        return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
    }

    //
    // Disable and clear the interrupts to start with.
    //
    ui32Status = am_hal_mspi_interrupt_disable(pFlash->pMspiHandle, 0xFFFFFFFF);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
      return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
    }
    ui32Status = am_hal_mspi_interrupt_clear(pFlash->pMspiHandle, 0xFFFFFFFF);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
      return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
    }

    //
    // Disable the MSPI instance.
    //
    ui32Status = am_hal_mspi_disable(pFlash->pMspiHandle);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
    }

    if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_power_control(pFlash->pMspiHandle, AM_HAL_SYSCTRL_DEEPSLEEP, false))
    {
        am_util_debug_printf("Error - Failed to power on MSPI.\n");
        return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
    }

    //
    // Deinitialize the MSPI instance.
    //
    ui32Status = am_hal_mspi_deinitialize(pFlash->pMspiHandle);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
    }

    // Free this device handle
    pFlash->bOccupied = false;

    //
    // Clear the Flash Caching.
    //
#if !defined(AM_PART_APOLLO4_API)
#if AM_CMSIS_REGS
    CACHECTRL->CACHECFG = 0;
#else // AM_CMSIS_REGS
    AM_REG(CACHECTRL, CACHECFG) = 0;
#endif // AM_CMSIS_REGS
#endif // !AM_PART_APOLLO4
    //
    // Return the status.
    //
    return AM_DEVICES_MSPI_ATXP032_STATUS_SUCCESS;
}

//*****************************************************************************
//
//  Reads the current status of the external flash
//
//*****************************************************************************
uint32_t
am_devices_mspi_atxp032_reset(void *pHandle)
{
    uint32_t      ui32Status;
    uint32_t      ui32PIOBuffer[32] = {0};


    //
    // Enable write.
    //
    ui32Status = am_devices_mspi_atxp032_command_write(pHandle, AM_DEVICES_MSPI_ATXP032_WRITE_ENABLE, false, 0, ui32PIOBuffer, 0);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
       return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
    }

    //
    // Return the device to SPI mode.
    //
    ui32Status = am_devices_mspi_atxp032_command_write(pHandle, AM_DEVICES_ATXP032_RETURN_TO_SPI_MODE, false, 0, ui32PIOBuffer, 0);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
    }

    //
    // Disable write.
    //
    ui32Status = am_devices_mspi_atxp032_command_write(pHandle, AM_DEVICES_MSPI_ATXP032_WRITE_DISABLE, false, 0, ui32PIOBuffer, 0);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
       return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
    }

    return AM_DEVICES_MSPI_ATXP032_STATUS_SUCCESS;
}

#if defined(AM_PART_APOLLO4) || defined(AM_PART_APOLLO4B)
//*****************************************************************************
//
//!  Test signal integrity and delays on IO lines using echo with inversion command .
//!
//! @param pHandle - handle to the mspi instance.
//! @param pattern - data pattern to be used for the test.
//! @param length - data pattern to be used for the test.
//! @param pui8RxBuffer - handle to the mspi instance.
//!
//! This function reads the data of the echo with inversion command used for signal
//! integrity and delay on IO lines test.
//!
//! @return 32-bit status
//
//*****************************************************************************
uint32_t
am_devices_mspi_atxp032_echo_with_inversion(void *pHandle,
                                            uint8_t pattern,
                                            uint32_t length,
                                            uint8_t *pui8RxBuffer)
{
    am_devices_mspi_atxp032_t *pFlash = (am_devices_mspi_atxp032_t *)pHandle;

    // Read status/control register command uses 1 byte address
    am_hal_mspi_instr_addr_t sInstAddrCfg;
    sInstAddrCfg.eAddrCfg = AM_HAL_MSPI_ADDR_1_BYTE;
    sInstAddrCfg.eInstrCfg = pFlash->stSetting.eInstrCfg;   // keep instruction setting the same
    am_hal_mspi_control(pFlash->pMspiHandle, AM_HAL_MSPI_REQ_SET_INSTR_ADDR_LEN, &sInstAddrCfg);

    //
    // Send and read back the echo with inversion using the given patten
    //
    uint32_t ui32Status = am_devices_mspi_atxp032_command_read(pHandle, AM_DEVICES_ATXP032_ECHO_WITH_INVSERSION, true, pattern, (uint32_t *)pui8RxBuffer, length);

    sInstAddrCfg.eAddrCfg = pFlash->stSetting.eAddrCfg;
    sInstAddrCfg.eInstrCfg = pFlash->stSetting.eInstrCfg;
    am_hal_mspi_control(pFlash->pMspiHandle, AM_HAL_MSPI_REQ_SET_INSTR_ADDR_LEN, &sInstAddrCfg);

    if ( AM_HAL_STATUS_SUCCESS == ui32Status )
    {
        return AM_DEVICES_MSPI_ATXP032_STATUS_SUCCESS;
    }
    else
    {
        return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
    }
}
#endif

//*****************************************************************************
//
//  Reads the ID of the external flash and returns the value.
//
//*****************************************************************************
uint32_t
am_devices_mspi_atxp032_id(void *pHandle)
{
    uint32_t      ui32Status;
    uint32_t      ui32DeviceID;

    //
    // Send the command sequence to read the Device ID and return status.
    //
    uint8_t       ui8Response[11];
    ui32Status = am_devices_mspi_atxp032_command_read(pHandle, AM_DEVICES_MSPI_ATXP032_READ_ID, false, 0, (uint32_t *)&ui8Response[0], 11);
    ui32DeviceID = (ui8Response[7] << 16) | (ui8Response[8] << 8) | ui8Response[9];
    if ( ((ui32DeviceID & AM_DEVICES_MSPI_ATXP032_ID_MASK) == AM_DEVICES_MSPI_ATXP032_ID) &&
       (AM_HAL_STATUS_SUCCESS == ui32Status) )
    {
        return AM_DEVICES_MSPI_ATXP032_STATUS_SUCCESS;
    }
    else
    {
        return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
    }
}

//*****************************************************************************
//
//  Reads the current status of the external flash
//
//*****************************************************************************
uint32_t
am_devices_mspi_atxp032_status(void *pHandle, uint32_t *pStatus)
{
    uint32_t      ui32Status;

    //
    // Send the command sequence to read the device status.
    //
    ui32Status = am_devices_mspi_atxp032_command_read(pHandle, AM_DEVICES_MSPI_ATXP032_READ_STATUS_BYTE1, false, 0, pStatus, 1);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
    }

    return AM_DEVICES_MSPI_ATXP032_STATUS_SUCCESS;
}

//*****************************************************************************
//
//
//
//*****************************************************************************
uint32_t
am_devices_mspi_atxp032_read_adv(void *pHandle, uint8_t *pui8RxBuffer,
                                 uint32_t ui32ReadAddress,
                                 uint32_t ui32NumBytes,
                                 uint32_t ui32PauseCondition,
                                 uint32_t ui32StatusSetClr,
                                 am_hal_mspi_callback_t pfnCallback,
                                 void *pCallbackCtxt)
{
    am_hal_mspi_dma_transfer_t    Transaction;
    uint32_t                      ui32Status;
    am_devices_mspi_atxp032_t *pFlash = (am_devices_mspi_atxp032_t *)pHandle;

    // Set the DMA priority
    Transaction.ui8Priority = 1;

    // Set the transfer direction to RX (Read)
    Transaction.eDirection = AM_HAL_MSPI_RX;

    // Set the transfer count in bytes.
    Transaction.ui32TransferCount = ui32NumBytes;

    // Set the address to read data from.
    Transaction.ui32DeviceAddress = ui32ReadAddress;

    // Set the target SRAM buffer address.
    Transaction.ui32SRAMAddress = (uint32_t)pui8RxBuffer;

    // Clear the CQ stimulus.
    Transaction.ui32PauseCondition = ui32PauseCondition;
    // Clear the post-processing
    Transaction.ui32StatusSetClr = ui32StatusSetClr;

#if defined(AM_PART_APOLLO4)
    Transaction.eDeviceNum         = AM_HAL_MSPI_DEVICE0;
#endif

    // Check the transaction status.
    ui32Status = am_hal_mspi_nonblocking_transfer(pFlash->pMspiHandle, &Transaction,
                                                  AM_HAL_MSPI_TRANS_DMA, pfnCallback, pCallbackCtxt);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
    }

    //
    // Return the status.
    //
    return AM_DEVICES_MSPI_ATXP032_STATUS_SUCCESS;
}


//*****************************************************************************
//
//  Reads the contents of the external flash into a buffer.
//
//*****************************************************************************
uint32_t
am_devices_mspi_atxp032_read(void *pHandle, uint8_t *pui8RxBuffer,
                             uint32_t ui32ReadAddress,
                             uint32_t ui32NumBytes,
                             bool bWaitForCompletion)
{
    am_hal_mspi_dma_transfer_t    Transaction;
    uint32_t                      ui32Status;
    am_devices_mspi_atxp032_t *pFlash = (am_devices_mspi_atxp032_t *)pHandle;

    // Set the DMA priority
    Transaction.ui8Priority = 1;

    // Set the transfer direction to RX (Read)
    Transaction.eDirection = AM_HAL_MSPI_RX;

    // Set the transfer count in bytes.
    Transaction.ui32TransferCount = ui32NumBytes;

    // Set the address to read data from.
    Transaction.ui32DeviceAddress = ui32ReadAddress;

    // Set the target SRAM buffer address.
    Transaction.ui32SRAMAddress = (uint32_t)pui8RxBuffer;

    // Clear the CQ stimulus.
    Transaction.ui32PauseCondition = 0;
    // Clear the post-processing
    Transaction.ui32StatusSetClr = 0;

#if defined(AM_PART_APOLLO4)
    Transaction.eDeviceNum         = AM_HAL_MSPI_DEVICE0;
#endif

    if (bWaitForCompletion)
    {
        // Start the transaction.
        volatile uint32_t ui32DMAStatus = 0xFFFFFFFF;
        ui32Status = am_hal_mspi_nonblocking_transfer(pFlash->pMspiHandle, &Transaction, AM_HAL_MSPI_TRANS_DMA, pfnMSPI_ATXP032_Callback, (void *)&ui32DMAStatus);

        // Check the transaction status.
        if (AM_HAL_STATUS_SUCCESS != ui32Status)
        {
            return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
        }

        // Wait for DMA Complete or Timeout
        for (uint32_t i = 0; i < AM_DEVICES_MSPI_ATXP032_TIMEOUT; i++)
        {
#if defined(AM_PART_APOLLO4_API)
            if ( (AM_HAL_STATUS_SUCCESS == ui32DMAStatus) || (AM_HAL_MSPI_FIFO_FULL_CONDITION == ui32DMAStatus) )
            {
                break;
            }
#else
            if (AM_HAL_STATUS_SUCCESS == ui32DMAStatus)
            {
                break;
            }
#endif

            //
            // Call the BOOTROM cycle function to delay for about 1 microsecond.
            //
            am_util_delay_us(1);
        }

#if defined(AM_PART_APOLLO4_API)
        if (AM_HAL_MSPI_FIFO_FULL_CONDITION == ui32DMAStatus)
        {
            am_hal_gpio_output_toggle(22);
            return AM_HAL_MSPI_FIFO_FULL_CONDITION;
        }
        else if (AM_HAL_STATUS_SUCCESS == ui32DMAStatus)
        {
            return AM_DEVICES_MSPI_ATXP032_STATUS_SUCCESS;
        }
#else
        if (AM_HAL_STATUS_SUCCESS == ui32DMAStatus)
        {
            return AM_DEVICES_MSPI_ATXP032_STATUS_SUCCESS;
        }
#endif
        else
        {
            return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
        }
    }
    else
    {
        // Check the transaction status.
        ui32Status = am_hal_mspi_nonblocking_transfer(pFlash->pMspiHandle, &Transaction,
                                                      AM_HAL_MSPI_TRANS_DMA, NULL, NULL);
        if (AM_HAL_STATUS_SUCCESS != ui32Status)
        {
            return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
        }
    }

    //
    // Return the status.
    //
    return AM_DEVICES_MSPI_ATXP032_STATUS_SUCCESS;
}

#if defined(AM_PART_APOLLO4_API)
static uint32_t
mspi_atxp032_dma_read(void *pHandle, uint8_t *pui8RxBuffer,
                      uint32_t ui32ReadAddress,
                      uint32_t ui32NumBytes)
{
    am_hal_mspi_dma_transfer_t    Transaction;
    uint32_t                      ui32Status;
    am_devices_mspi_atxp032_t *pFlash = (am_devices_mspi_atxp032_t *)pHandle;

    // Set the DMA priority
    Transaction.ui8Priority = 1;

    // Set the transfer direction to RX (Read)
    Transaction.eDirection = AM_HAL_MSPI_RX;

    // Set the transfer count in bytes.
    Transaction.ui32TransferCount = ui32NumBytes;

    // Set the address to read data from.
    Transaction.ui32DeviceAddress = ui32ReadAddress;

    // Set the target SRAM buffer address.
    Transaction.ui32SRAMAddress = (uint32_t)pui8RxBuffer;

    // Clear the CQ stimulus.
    Transaction.ui32PauseCondition = 0;
    // Clear the post-processing
    Transaction.ui32StatusSetClr = 0;

#if defined(AM_PART_APOLLO4)
    Transaction.eDeviceNum         = AM_HAL_MSPI_DEVICE0;
#endif

    // Start the transaction.
    volatile uint32_t ui32DMAStatus = 0xFFFFFFFF;
    ui32Status = am_hal_mspi_nonblocking_transfer(pFlash->pMspiHandle, &Transaction, AM_HAL_MSPI_TRANS_DMA, pfnMSPI_ATXP032_Callback, (void *)&ui32DMAStatus);

    // Check the transaction status.
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
    }

    // Wait for DMA Complete or Timeout
    for (uint32_t i = 0; i < AM_DEVICES_MSPI_ATXP032_TIMEOUT; i++)
    {

        // check DMA status without using ISR
        am_hal_mspi_interrupt_status_get(pFlash->pMspiHandle, &ui32Status, false);
        am_hal_mspi_interrupt_clear(pFlash->pMspiHandle, ui32Status);
        am_hal_mspi_interrupt_service(pFlash->pMspiHandle, ui32Status);

#if defined(AM_PART_APOLLO4) || defined(AM_PART_APOLLO4B)
        if ( (AM_HAL_STATUS_SUCCESS == ui32DMAStatus) || (AM_HAL_MSPI_FIFO_FULL_CONDITION == ui32DMAStatus) )
        {
            break;
        }
#else
        if (AM_HAL_STATUS_SUCCESS == ui32DMAStatus)
        {
            break;
        }
#endif

        //
        // Call the BOOTROM cycle function to delay for about 1 microsecond.
        //
        am_util_delay_us(1);
    }

#if defined(AM_PART_APOLLO4) || defined(AM_PART_APOLLO4B)
    if (AM_HAL_MSPI_FIFO_FULL_CONDITION == ui32DMAStatus)
    {
        return AM_HAL_MSPI_FIFO_FULL_CONDITION;
    }
    else if (AM_HAL_STATUS_SUCCESS == ui32DMAStatus)
    {
        return AM_DEVICES_MSPI_ATXP032_STATUS_SUCCESS;
    }
#else
    if (AM_HAL_STATUS_SUCCESS == ui32DMAStatus)
    {
        return AM_DEVICES_MSPI_ATXP032_STATUS_SUCCESS;
    }
#endif
    else
    {
        return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
    }
}
#endif // defined(AM_PART_APOLLO4_API)


//*****************************************************************************
//
//  Reads the contents of the external flash into a buffer.
//
//*****************************************************************************
uint32_t
am_devices_mspi_atxp032_read_hiprio(void *pHandle, uint8_t *pui8RxBuffer,
                                    uint32_t ui32ReadAddress,
                                    uint32_t ui32NumBytes,
                                    bool bWaitForCompletion)
{
    am_hal_mspi_dma_transfer_t    Transaction;
    uint32_t                      ui32Status;
    am_devices_mspi_atxp032_t *pFlash = (am_devices_mspi_atxp032_t *)pHandle;

    // Set the DMA priority
    Transaction.ui8Priority = 1;

    // Set the transfer direction to RX (Read)
    Transaction.eDirection = AM_HAL_MSPI_RX;

    // Set the transfer count in bytes.
    Transaction.ui32TransferCount = ui32NumBytes;

    // Set the address to read data from.
    Transaction.ui32DeviceAddress = ui32ReadAddress;

    // Set the target SRAM buffer address.
    Transaction.ui32SRAMAddress = (uint32_t)pui8RxBuffer;

    // Clear the CQ stimulus.
    Transaction.ui32PauseCondition = 0;
    // Clear the post-processing
    Transaction.ui32StatusSetClr = 0;

#if defined(AM_PART_APOLLO4)
    Transaction.eDeviceNum         = AM_HAL_MSPI_DEVICE0;
#endif

    if (bWaitForCompletion)
    {
        // Start the transaction.
        volatile bool bDMAComplete = false;
        ui32Status = am_hal_mspi_highprio_transfer(pFlash->pMspiHandle, &Transaction, AM_HAL_MSPI_TRANS_DMA, pfnMSPI_ATXP032_Callback, (void*)&bDMAComplete);

        // Check the transaction status.
        if (AM_HAL_STATUS_SUCCESS != ui32Status)
        {
            return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
        }

        // Wait for DMA Complete or Timeout
        for (uint32_t i = 0; i < AM_DEVICES_MSPI_ATXP032_TIMEOUT; i++)
        {
            if (bDMAComplete)
            {
                break;
            }
            //
            // Call the BOOTROM cycle function to delay for about 1 microsecond.
            //
            am_util_delay_us(1);
        }

        // Check the status.
        if (!bDMAComplete)
        {
            return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
        }
    }
    else
    {
        // Check the transaction status.
        ui32Status = am_hal_mspi_highprio_transfer(pFlash->pMspiHandle, &Transaction,
                                                      AM_HAL_MSPI_TRANS_DMA, NULL, NULL);
        if (AM_HAL_STATUS_SUCCESS != ui32Status)
        {
            return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
        }
    }

    //
    // Return the status.
    //
    return AM_DEVICES_MSPI_ATXP032_STATUS_SUCCESS;
}



//*****************************************************************************
//
//  Programs the given range of flash addresses.
//
//*****************************************************************************
uint32_t
am_devices_mspi_atxp032_write(void *pHandle, uint8_t *pui8TxBuffer,
                              uint32_t ui32WriteAddress,
                              uint32_t ui32NumBytes,
                              bool bWaitForCompletion)
{
    am_hal_mspi_dma_transfer_t    Transaction;
    bool                          bWriteComplete = false;
    uint32_t                      ui32BytesLeft = ui32NumBytes;
    uint32_t                      ui32PageAddress = ui32WriteAddress;
    uint32_t                      ui32BufferAddress = (uint32_t)pui8TxBuffer;
    uint32_t                      ui32Status;
    am_devices_mspi_atxp032_t     *pFlash = (am_devices_mspi_atxp032_t *)pHandle;
    uint32_t                      ui32PIOBuffer[32] = {0};

    while (ui32BytesLeft > 0)
    {
        //
        // Send the command sequence to enable writing.
        //
        ui32Status = am_devices_mspi_atxp032_command_write(pHandle, AM_DEVICES_MSPI_ATXP032_WRITE_ENABLE, false, 0, ui32PIOBuffer, 0);
        if (AM_HAL_STATUS_SUCCESS != ui32Status)
        {
            return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
        }

        // Set the DMA priority
        Transaction.ui8Priority = 1;

        // Set the transfer direction to TX (Write)
        Transaction.eDirection = AM_HAL_MSPI_TX;

        if (ui32BytesLeft > AM_DEVICES_MSPI_ATXP032_PAGE_SIZE)
        {
            // Set the transfer count in bytes.
            Transaction.ui32TransferCount = AM_DEVICES_MSPI_ATXP032_PAGE_SIZE;
            ui32BytesLeft -= AM_DEVICES_MSPI_ATXP032_PAGE_SIZE;
        }
        else
        {
            // Set the transfer count in bytes.
            Transaction.ui32TransferCount = ui32BytesLeft;
            ui32BytesLeft = 0;
        }

        // Set the address to read data to.
        Transaction.ui32DeviceAddress = ui32PageAddress;
        ui32PageAddress += AM_DEVICES_MSPI_ATXP032_PAGE_SIZE;

        // Set the source SRAM buffer address.
        Transaction.ui32SRAMAddress = ui32BufferAddress;
        ui32BufferAddress += AM_DEVICES_MSPI_ATXP032_PAGE_SIZE;

        // Clear the CQ stimulus.
        Transaction.ui32PauseCondition = 0;
        // Clear the post-processing
        Transaction.ui32StatusSetClr = 0;

#if defined(AM_PART_APOLLO4)
    Transaction.eDeviceNum         = AM_HAL_MSPI_DEVICE0;
#endif

        // Start the transaction.
        volatile uint32_t ui32DMAStatus = 0xFFFFFFFF;
        ui32Status = am_hal_mspi_nonblocking_transfer(pFlash->pMspiHandle, &Transaction, AM_HAL_MSPI_TRANS_DMA, pfnMSPI_ATXP032_Callback, (void*)&ui32DMAStatus);

        // Check the transaction status.
        if (AM_HAL_STATUS_SUCCESS != ui32Status)
        {
            return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
        }

        // Wait for DMA Complete or Timeout
        for (uint32_t i = 0; i < AM_DEVICES_MSPI_ATXP032_TIMEOUT; i++)
        {
            if (AM_HAL_STATUS_SUCCESS == ui32DMAStatus)
            {
                break;
            }
            //
            // Call the BOOTROM cycle function to delay for about 1 microsecond.
            //
            am_util_delay_us(1);
        }

        // Check the status.
        if (AM_HAL_STATUS_SUCCESS != ui32DMAStatus)
        {
            return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
        }

        //
        // Wait for the Write In Progress to indicate the erase is complete.
        //
        for (uint32_t i = 0; i < AM_DEVICES_MSPI_ATXP032_TIMEOUT; i++)
        {
            // ATXP032 has different number of bytes for each speed of status read.
            switch ( pFlash->stSetting.eDeviceConfig )
            {
                case AM_HAL_MSPI_FLASH_SERIAL_CE0:
                case AM_HAL_MSPI_FLASH_SERIAL_CE1:
                    ui32Status = am_devices_mspi_atxp032_command_read(pHandle, AM_DEVICES_MSPI_ATXP032_READ_STATUS_BYTE1, false, 0, ui32PIOBuffer, 2);
                    if (AM_HAL_STATUS_SUCCESS != ui32Status)
                    {
                        return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
                    }
                    bWriteComplete = (0 == (ui32PIOBuffer[0] & AM_DEVICES_ATXP032_WIP));
                    break;
                case AM_HAL_MSPI_FLASH_QUAD_CE0:
                case AM_HAL_MSPI_FLASH_QUAD_CE1:
                    ui32Status = am_devices_mspi_atxp032_command_read(pHandle, AM_DEVICES_MSPI_ATXP032_READ_STATUS_BYTE1, false, 0, ui32PIOBuffer, 4);
                    if (AM_HAL_STATUS_SUCCESS != ui32Status)
                    {
                        return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
                    }
                    bWriteComplete = (0 == ((ui32PIOBuffer[0] >> 16) & AM_DEVICES_ATXP032_WIP));
                    break;
                case AM_HAL_MSPI_FLASH_OCTAL_CE0:
                case AM_HAL_MSPI_FLASH_OCTAL_CE1:
                    ui32Status = am_devices_mspi_atxp032_command_read(pHandle, AM_DEVICES_MSPI_ATXP032_READ_STATUS_BYTE1, false, 0, ui32PIOBuffer, 6);
                    if (AM_HAL_STATUS_SUCCESS != ui32Status)
                    {
                        return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
                    }
                    bWriteComplete = (0 == (ui32PIOBuffer[1] & AM_DEVICES_ATXP032_WIP));
                    break;
                default:
                    return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
            }

            am_util_delay_us(100);
            if (bWriteComplete)
            {
                break;
            }
        }

        //
        // Send the command sequence to disable writing.
        //
        ui32Status = am_devices_mspi_atxp032_command_write(pHandle, AM_DEVICES_MSPI_ATXP032_WRITE_DISABLE, false, 0, ui32PIOBuffer, 0);
        if (AM_HAL_STATUS_SUCCESS != ui32Status)
        {
            return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
        }
    }

  //
  // Return the status.
  //
  return AM_DEVICES_MSPI_ATXP032_STATUS_SUCCESS;
}

#if defined(AM_PART_APOLLO4_API)
static uint32_t
mspi_atxp032_dma_write(void *pHandle, uint8_t *pui8TxBuffer,
                       uint32_t ui32WriteAddress,
                       uint32_t ui32NumBytes)
{
    am_hal_mspi_dma_transfer_t    Transaction;
    bool                          bWriteComplete = false;
    uint32_t                      ui32BytesLeft = ui32NumBytes;
    uint32_t                      ui32PageAddress = ui32WriteAddress;
    uint32_t                      ui32BufferAddress = (uint32_t)pui8TxBuffer;
    uint32_t                      ui32Status;
    am_devices_mspi_atxp032_t *pFlash = (am_devices_mspi_atxp032_t *)pHandle;
    uint32_t      ui32PIOBuffer[32] = {0};


    while (ui32BytesLeft > 0)
    {
        //
        // Send the command sequence to enable writing.
        //
        ui32Status = am_devices_mspi_atxp032_command_write(pHandle, AM_DEVICES_MSPI_ATXP032_WRITE_ENABLE, false, 0, ui32PIOBuffer, 0);
        if (AM_HAL_STATUS_SUCCESS != ui32Status)
        {
            return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
        }

        // Set the DMA priority
        Transaction.ui8Priority = 1;

        // Set the transfer direction to TX (Write)
        Transaction.eDirection = AM_HAL_MSPI_TX;

        if (ui32BytesLeft > AM_DEVICES_MSPI_ATXP032_PAGE_SIZE)
        {
            // Set the transfer count in bytes.
            Transaction.ui32TransferCount = AM_DEVICES_MSPI_ATXP032_PAGE_SIZE;
            ui32BytesLeft -= AM_DEVICES_MSPI_ATXP032_PAGE_SIZE;
        }
        else
        {
            // Set the transfer count in bytes.
            Transaction.ui32TransferCount = ui32BytesLeft;
            ui32BytesLeft = 0;
        }

        // Set the address to read data to.
        Transaction.ui32DeviceAddress = ui32PageAddress;
        ui32PageAddress += AM_DEVICES_MSPI_ATXP032_PAGE_SIZE;

        // Set the source SRAM buffer address.
        Transaction.ui32SRAMAddress = ui32BufferAddress;
        ui32BufferAddress += AM_DEVICES_MSPI_ATXP032_PAGE_SIZE;

        // Clear the CQ stimulus.
        Transaction.ui32PauseCondition = 0;
        // Clear the post-processing
        Transaction.ui32StatusSetClr = 0;

#if defined(AM_PART_APOLLO4)
    Transaction.eDeviceNum         = AM_HAL_MSPI_DEVICE0;
#endif

        // Start the transaction.
        volatile uint32_t ui32DMAStatus = 0xFFFFFFFF;
        ui32Status = am_hal_mspi_nonblocking_transfer(pFlash->pMspiHandle, &Transaction, AM_HAL_MSPI_TRANS_DMA, pfnMSPI_ATXP032_Callback, (void*)&ui32DMAStatus);

        // Check the transaction status.
        if (AM_HAL_STATUS_SUCCESS != ui32Status)
        {
            return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
        }

        // Wait for DMA Complete or Timeout
        for (uint32_t i = 0; i < AM_DEVICES_MSPI_ATXP032_TIMEOUT; i++)
        {
            // check DMA status without using ISR
            am_hal_mspi_interrupt_status_get(pFlash->pMspiHandle, &ui32Status, false);
            am_hal_mspi_interrupt_clear(pFlash->pMspiHandle, ui32Status);
            am_hal_mspi_interrupt_service(pFlash->pMspiHandle, ui32Status);

            if (AM_HAL_STATUS_SUCCESS == ui32DMAStatus)
            {
                break;
            }
            //
            // Call the BOOTROM cycle function to delay for about 1 microsecond.
            //
            am_util_delay_us(1);
        }

        // Check the status.
        if (AM_HAL_STATUS_SUCCESS != ui32DMAStatus)
        {
            return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
        }

        //
        // Wait for the Write In Progress to indicate the erase is complete.
        //
        for (uint32_t i = 0; i < AM_DEVICES_MSPI_ATXP032_TIMEOUT; i++)
        {
            // ATXP032 has different number of bytes for each speed of status read.
            switch ( pFlash->stSetting.eDeviceConfig )
            {
                case AM_HAL_MSPI_FLASH_SERIAL_CE0:
                case AM_HAL_MSPI_FLASH_SERIAL_CE1:
                    ui32Status = am_devices_mspi_atxp032_command_read(pHandle, AM_DEVICES_MSPI_ATXP032_READ_STATUS_BYTE1, false, 0, ui32PIOBuffer, 2);
                    if (AM_HAL_STATUS_SUCCESS != ui32Status)
                    {
                        return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
                    }
                    bWriteComplete = (0 == (ui32PIOBuffer[0] & AM_DEVICES_ATXP032_WIP));
                    break;
                case AM_HAL_MSPI_FLASH_QUAD_CE0:
                case AM_HAL_MSPI_FLASH_QUAD_CE1:
                    ui32Status = am_devices_mspi_atxp032_command_read(pHandle, AM_DEVICES_MSPI_ATXP032_READ_STATUS_BYTE1, false, 0, ui32PIOBuffer, 4);
                    if (AM_HAL_STATUS_SUCCESS != ui32Status)
                    {
                        return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
                    }
                    bWriteComplete = (0 == ((ui32PIOBuffer[0] >> 16) & AM_DEVICES_ATXP032_WIP));
                    break;
                case AM_HAL_MSPI_FLASH_OCTAL_CE0:
                case AM_HAL_MSPI_FLASH_OCTAL_CE1:
                    ui32Status = am_devices_mspi_atxp032_command_read(pHandle, AM_DEVICES_MSPI_ATXP032_READ_STATUS_BYTE1, false, 0, ui32PIOBuffer, 6);
                    if (AM_HAL_STATUS_SUCCESS != ui32Status)
                    {
                        return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
                    }
                    bWriteComplete = (0 == (ui32PIOBuffer[1] & AM_DEVICES_ATXP032_WIP));
                    break;
                default:
                    return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
            }

            am_util_delay_us(100);
            if (bWriteComplete)
            {
                break;
            }
        }

        //
        // Send the command sequence to disable writing.
        //
        ui32Status = am_devices_mspi_atxp032_command_write(pHandle, AM_DEVICES_MSPI_ATXP032_WRITE_DISABLE, false, 0, ui32PIOBuffer, 0);
        if (AM_HAL_STATUS_SUCCESS != ui32Status)
        {
            return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
        }
    }

  //
  // Return the status.
  //
  return AM_DEVICES_MSPI_ATXP032_STATUS_SUCCESS;
}
#endif // defined(AM_PART_APOLLO4_API)

//*****************************************************************************
//
//  Erases the entire contents of the external flash
//
//*****************************************************************************
uint32_t
am_devices_mspi_atxp032_mass_erase(void *pHandle)
{
    bool          bEraseComplete = false;
    uint32_t      ui32Status;
    //am_devices_mspi_atxp032_t *pFlash = (am_devices_mspi_atxp032_t *)pHandle;
    uint32_t      ui32PIOBuffer[32] = {0};

    //
    // Send the command sequence to enable writing.
    //
    ui32Status = am_devices_mspi_atxp032_command_write(pHandle, AM_DEVICES_MSPI_ATXP032_WRITE_ENABLE, false, 0, ui32PIOBuffer, 0);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
    }

    //
    // Send the command sequence to do the mass erase.
    //
    ui32Status = am_devices_mspi_atxp032_command_write(pHandle, AM_DEVICES_MSPI_ATXP032_BULK_ERASE, false, 0, ui32PIOBuffer, 0);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
    }
    //
    // Wait for the Write In Progress to indicate the erase is complete.
    //
    for (uint32_t i = 0; i < AM_DEVICES_MSPI_ATXP032_ERASE_TIMEOUT; i++)
    {
        ui32PIOBuffer[0] = 0;
        am_devices_mspi_atxp032_command_read(pHandle, AM_DEVICES_MSPI_ATXP032_READ_STATUS_BYTE1, false, 0, ui32PIOBuffer, 1);
        bEraseComplete = (0 == (ui32PIOBuffer[0] & AM_DEVICES_MSPI_ATXP032_WIP));
        if (bEraseComplete)
        {
            break;
        }
        am_util_delay_ms(10);
    }

    //
    // Check the status.
    //
    if (!bEraseComplete)
    {
        return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
    }

    //
    // Send the command sequence to disable writing.
    //
    ui32Status = am_devices_mspi_atxp032_command_write(pHandle, AM_DEVICES_MSPI_ATXP032_WRITE_DISABLE, false, 0, ui32PIOBuffer, 0);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
    }

    //
    // Return the status.
    //
    return AM_DEVICES_MSPI_ATXP032_STATUS_SUCCESS;
}

//*****************************************************************************
//
//  Erases the contents of a single sector of flash
//
//*****************************************************************************
uint32_t
am_devices_mspi_atxp032_sector_erase(void *pHandle, uint32_t ui32SectorAddress)
{
    bool          bEraseComplete = false;
    uint32_t      ui32Status;
    am_devices_mspi_atxp032_t *pFlash = (am_devices_mspi_atxp032_t *)pHandle;
    uint32_t      ui32PIOBuffer[32] = {0};

    //
    // Send the command sequence to enable writing.
    //
    ui32Status = am_devices_mspi_atxp032_command_write(pHandle, AM_DEVICES_MSPI_ATXP032_WRITE_ENABLE, false, 0, ui32PIOBuffer, 0);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
    }

    //
    // Send the command to remove protection from the sector.
    //
    ui32Status = am_devices_mspi_atxp032_command_write(pHandle, AM_DEVICES_ATXP032_UNPROTECT_SECTOR, true, ui32SectorAddress, ui32PIOBuffer, 0);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
      return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
    }

    //
    // Send the command sequence to enable writing.
    //
    ui32Status = am_devices_mspi_atxp032_command_write(pHandle, AM_DEVICES_MSPI_ATXP032_WRITE_ENABLE, false, 0, ui32PIOBuffer, 0);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
    }

    //
    // Send the command sequence to do the sector erase.
    //
    ui32Status = am_devices_mspi_atxp032_command_write(pHandle, AM_DEVICES_MSPI_ATXP032_SECTOR_ERASE, true, ui32SectorAddress, ui32PIOBuffer, 0);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
    }

    //
    // Wait for the Write In Progress to indicate the erase is complete.
    //
    for (uint32_t i = 0; i < AM_DEVICES_MSPI_ATXP032_ERASE_TIMEOUT; i++)
    {
        // ATXP032 has different number of bytes for each speed of status read.
        switch ( pFlash->stSetting.eDeviceConfig )
        {
            case AM_HAL_MSPI_FLASH_SERIAL_CE0:
            case AM_HAL_MSPI_FLASH_SERIAL_CE1:
                ui32Status = am_devices_mspi_atxp032_command_read(pHandle, AM_DEVICES_MSPI_ATXP032_READ_STATUS_BYTE1, false, 0, ui32PIOBuffer, 2);
                if (AM_HAL_STATUS_SUCCESS != ui32Status)
                {
                    return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
                }
                bEraseComplete = (0 == (ui32PIOBuffer[0] & AM_DEVICES_ATXP032_WIP));
                break;
            case AM_HAL_MSPI_FLASH_QUAD_CE0:
            case AM_HAL_MSPI_FLASH_QUAD_CE1:
                ui32Status = am_devices_mspi_atxp032_command_read(pHandle, AM_DEVICES_MSPI_ATXP032_READ_STATUS_BYTE1, false, 0, ui32PIOBuffer, 4);
                if (AM_HAL_STATUS_SUCCESS != ui32Status)
                {
                    return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
                }
                bEraseComplete = (0 == ((ui32PIOBuffer[0] >> 16) & AM_DEVICES_ATXP032_WIP));
                break;
            case AM_HAL_MSPI_FLASH_OCTAL_CE0:
            case AM_HAL_MSPI_FLASH_OCTAL_CE1:
                ui32Status = am_devices_mspi_atxp032_command_read(pHandle, AM_DEVICES_MSPI_ATXP032_READ_STATUS_BYTE1, false, 0, ui32PIOBuffer, 6);
                if (AM_HAL_STATUS_SUCCESS != ui32Status)
                {
                    return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
                }
                bEraseComplete = (0 == (ui32PIOBuffer[1] & AM_DEVICES_ATXP032_WIP));
                break;
            default:
                return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
        }

        if (bEraseComplete)
        {
            break;
        }
        am_util_delay_ms(10);
    }

    //
    // Check the status.
    //
    if (!bEraseComplete)
    {
        return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
    }

    //
    // Send the command sequence to disable writing.
    //

    ui32Status = am_devices_mspi_atxp032_command_write(pHandle, AM_DEVICES_MSPI_ATXP032_WRITE_DISABLE, false, 0, ui32PIOBuffer, 0);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
    }
    //
    // Return the status.
    //
    return AM_DEVICES_MSPI_ATXP032_STATUS_SUCCESS;
}

//*****************************************************************************
//
//  Sets up the MSPI and external FLASH into XIP mode.
//
//*****************************************************************************
uint32_t
am_devices_mspi_atxp032_enable_xip(void *pHandle)
{
    uint32_t ui32Status;
    am_devices_mspi_atxp032_t *pFlash = (am_devices_mspi_atxp032_t *)pHandle;

#if defined(AM_PART_APOLLO4_API)
    //
    // Set Aperture XIP range
    //
    ui32Status = am_hal_mspi_control(pFlash->pMspiHandle, AM_HAL_MSPI_REQ_XIP_CONFIG, &gXipConfig[pFlash->ui32Module]);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
    }
#endif

    //
    // Enable XIP on the MSPI.
    //
#if defined(AM_PART_APOLLO4_API)
    ui32Status = am_hal_mspi_control(pFlash->pMspiHandle, AM_HAL_MSPI_REQ_XIP_EN, &gXipConfig[pFlash->ui32Module]);
#else
    ui32Status = am_hal_mspi_control(pFlash->pMspiHandle, AM_HAL_MSPI_REQ_XIP_EN, NULL);
#endif
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
    }

#if !MSPI_USE_CQ
    // Disable the DMA interrupts.
    ui32Status = am_hal_mspi_interrupt_disable(pFlash->pMspiHandle,
                                               AM_HAL_MSPI_INT_DMAERR |
                                               AM_HAL_MSPI_INT_DMACMP );
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
    }
#endif

    return AM_DEVICES_MSPI_ATXP032_STATUS_SUCCESS;
}

//*****************************************************************************
//
//  Removes the MSPI and external FLASH from XIP mode.
//
//*****************************************************************************
uint32_t
am_devices_mspi_atxp032_disable_xip(void *pHandle)
{
    uint32_t ui32Status;
    am_devices_mspi_atxp032_t *pFlash = (am_devices_mspi_atxp032_t *)pHandle;
    uint32_t      ui32PIOBuffer[32] = {0};

    //
    // Send the command to enable writing.
    //
    ui32Status = am_devices_mspi_atxp032_command_write(pHandle, AM_DEVICES_MSPI_ATXP032_WRITE_ENABLE, false, 0, ui32PIOBuffer, 0);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
    }

    //
    // Disable XIP on the MSPI.
    //
#if defined(AM_PART_APOLLO4_API)
    ui32Status = am_hal_mspi_control(pFlash->pMspiHandle, AM_HAL_MSPI_REQ_XIP_DIS, &gXipConfig[pFlash->ui32Module]);
#else
    ui32Status = am_hal_mspi_control(pFlash->pMspiHandle, AM_HAL_MSPI_REQ_XIP_DIS, NULL);
#endif
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
    }

    return AM_DEVICES_MSPI_ATXP032_STATUS_SUCCESS;
}

//*****************************************************************************
//
//  Sets up the MSPI and external FLASH into scrambling mode.
//
//*****************************************************************************
uint32_t
am_devices_mspi_atxp032_enable_scrambling(void *pHandle)
{
    uint32_t ui32Status;
    am_devices_mspi_atxp032_t *pFlash = (am_devices_mspi_atxp032_t *)pHandle;

    //
    // Enable scrambling on the MSPI.
    //
#if defined(AM_PART_APOLLO4_API)
    ui32Status = am_hal_mspi_control(pFlash->pMspiHandle, AM_HAL_MSPI_REQ_SCRAMB_EN, &gXipConfig[pFlash->ui32Module]);
#else
    ui32Status = am_hal_mspi_control(pFlash->pMspiHandle, AM_HAL_MSPI_REQ_SCRAMB_EN, NULL);
#endif
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
    }
    return AM_DEVICES_MSPI_ATXP032_STATUS_SUCCESS;
}

//*****************************************************************************
//
//  Removes the MSPI and external FLASH from scrambling mode.
//
//*****************************************************************************
uint32_t
am_devices_mspi_atxp032_disable_scrambling(void *pHandle)
{
    uint32_t ui32Status;
    am_devices_mspi_atxp032_t *pFlash = (am_devices_mspi_atxp032_t *)pHandle;
    uint32_t      ui32PIOBuffer[32] = {0};

    //
    // Send the command to enable writing.
    //
    ui32Status = am_devices_mspi_atxp032_command_write(pHandle, AM_DEVICES_MSPI_ATXP032_WRITE_ENABLE, false, 0, ui32PIOBuffer, 0);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
    }

    //
    // Disable Scrambling on the MSPI.
    //
#if defined(AM_PART_APOLLO4_API)
    ui32Status = am_hal_mspi_control(pFlash->pMspiHandle, AM_HAL_MSPI_REQ_SCRAMB_DIS, &gXipConfig[pFlash->ui32Module]);
#else
    ui32Status = am_hal_mspi_control(pFlash->pMspiHandle, AM_HAL_MSPI_REQ_SCRAMB_DIS, NULL);
#endif
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
    }

    return AM_DEVICES_MSPI_ATXP032_STATUS_SUCCESS;
}

#if defined(AM_PART_APOLLO4_API)


static int prepare_test_pattern(uint32_t pattern_index, uint8_t* buff, uint32_t len)
{
    uint32_t *pui32TxPtr = (uint32_t*)buff;
    uint8_t  *pui8TxPtr  = (uint8_t*)buff;

    // length has to be multiple of 4 bytes
    if ( len % 4 )
    {
        return -1;
    }

    switch ( pattern_index )
    {
        case 0:
            // 0x5555AAAA
            for (uint32_t i = 0; i < len / 4; i++)
            {
               pui32TxPtr[i] = (0x5555AAAA);
            }
            break;
        case 1:
            // 0xFFFF0000
            for (uint32_t i = 0; i < len / 4; i++)
            {
               pui32TxPtr[i] = (0xFFFF0000);
            }
            break;
        case 2:
            // walking
            for (uint32_t i = 0; i < len; i++)
            {
               pui8TxPtr[i] = 0x01 << (i % 8);
            }
            break;
        case 3:
            // incremental from 1
            for (uint32_t i = 0; i < len; i++)
            {
               pui8TxPtr[i] = ((i + 1) & 0xFF);
            }
            break;
        case 4:
            // decremental from 0xff
            for ( uint32_t i = 0; i < len; i++ )
            {
                // decrement starting from 0xff
                pui8TxPtr[i] = (0xff - i) & 0xFF;
            }
            break;
        default:
            // incremental from 1
            for (uint32_t i = 0; i < len; i++)
            {
               pui8TxPtr[i] = ((i + 1) & 0xFF);
            }
            break;

    }

    return 0;
}

bool
flash_write(void* flashHandle, uint32_t length)
{
    // Try to use as less ram as possible in stack
    uint32_t ui32NumberOfBytesLeft = length;
    uint32_t ui32TestBytes = 0;
    uint32_t ui32AddressOffset = 0;
    uint8_t  ui8PatternCounter = 0;
    uint8_t  ui8TxBuffer[FLASH_CHECK_DATA_SIZE_BYTES];
    uint32_t ui32Status = AM_DEVICES_MSPI_ATXP032_STATUS_SUCCESS;

    while ( ui32NumberOfBytesLeft )
    {
        if ( ui32NumberOfBytesLeft > FLASH_CHECK_DATA_SIZE_BYTES )
        {
            ui32TestBytes = FLASH_CHECK_DATA_SIZE_BYTES;
            ui32NumberOfBytesLeft -= FLASH_CHECK_DATA_SIZE_BYTES;
        }
        else
        {
            ui32TestBytes = ui32NumberOfBytesLeft;
            ui32NumberOfBytesLeft = 0;
        }

        //
        // Write to target address with test pattern with given length
        // Use 5 patterns: 0x5555AAAA, 0xFFFF0000, Walking, incremental and decremental
        //

        prepare_test_pattern((ui8PatternCounter) % FLASH_TEST_PATTERN_NUMBER, ui8TxBuffer, ui32TestBytes);
        ui8PatternCounter++;

        // write to target address
        ui32Status = mspi_atxp032_dma_write(flashHandle, ui8TxBuffer,
                                            (AM_DEVICES_MSPI_ATXP032_SECTOR_FOR_TIMING_CHECK << 16) + ui32AddressOffset,
                                            ui32TestBytes);
        if ( ui32Status ==  AM_DEVICES_MSPI_ATXP032_STATUS_ERROR)
        {
            return true;
        }

        ui32AddressOffset += ui32TestBytes;
    }

    return false;
}

#if defined(AM_PART_APOLLO4P) || defined(AM_PART_APOLLO4L)
#else
static uint32_t
mspi_atxp032_disable_xip(void *pHandle)
{
    uint32_t ui32Status;
    am_devices_mspi_atxp032_t *pFlash = (am_devices_mspi_atxp032_t *)pHandle;

    //
    // Disable XIP on the MSPI.
    //
#if defined(AM_PART_APOLLO4_API)
    ui32Status = am_hal_mspi_control(pFlash->pMspiHandle, AM_HAL_MSPI_REQ_XIP_DIS, &gXipConfig[pFlash->ui32Module]);
#else
    ui32Status = am_hal_mspi_control(pFlash->pMspiHandle, AM_HAL_MSPI_REQ_XIP_DIS, NULL);
#endif
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
    }

    return AM_DEVICES_MSPI_ATXP032_STATUS_SUCCESS;
}
#endif

bool
flash_check(void* flashHandle, uint32_t length)
{
    // Try to use as less ram as possible in stack
    uint32_t ui32NumberOfBytesLeft = length;
    uint32_t ui32TestBytes = 0;
    uint32_t ui32AddressOffset = 0;
    uint8_t  ui8PatternCounter = 0;
    uint8_t  ui8TxBuffer[FLASH_CHECK_DATA_SIZE_BYTES];
    uint8_t  ui8RxBuffer[FLASH_CHECK_DATA_SIZE_BYTES];
    uint32_t ui32Status = AM_DEVICES_MSPI_ATXP032_STATUS_SUCCESS;

    while ( ui32NumberOfBytesLeft )
    {
        if ( ui32NumberOfBytesLeft > FLASH_CHECK_DATA_SIZE_BYTES )
        {
            ui32TestBytes = FLASH_CHECK_DATA_SIZE_BYTES;
            ui32NumberOfBytesLeft -= FLASH_CHECK_DATA_SIZE_BYTES;
        }
        else
        {
            ui32TestBytes = ui32NumberOfBytesLeft;
            ui32NumberOfBytesLeft = 0;
        }

        //
        // Write to target address with test pattern with given length
        // Use 5 patterns: 0x5555AAAA, 0xFFFF0000, Walking, incremental and decremental
        //
        prepare_test_pattern((ui8PatternCounter) % FLASH_TEST_PATTERN_NUMBER, ui8TxBuffer, ui32TestBytes);
        ui8PatternCounter++;

        //
        // Read back data
        //
#if defined(AM_PART_APOLLO4P) || defined(AM_PART_APOLLO4L)
        ui32Status = mspi_atxp032_dma_read(flashHandle, ui8RxBuffer,
                                    (AM_DEVICES_MSPI_ATXP032_SECTOR_FOR_TIMING_CHECK << 16) + ui32AddressOffset,
                                    ui32TestBytes);
#else
        am_devices_mspi_atxp032_t *pFlash = (am_devices_mspi_atxp032_t *)flashHandle;
        if ( pFlash->stSetting.eClockFreq == AM_HAL_MSPI_CLK_96MHZ )
        {
            //
            // Read the data back into the RX buffer using XIP
            //
            ui32Status = am_devices_mspi_atxp032_enable_xip(flashHandle);
            if (AM_DEVICES_MSPI_ATXP032_STATUS_SUCCESS != ui32Status)
            {
                am_util_debug_printf("Failed to put the MSPI into XIP mode!\n");
            }
            am_hal_sysctrl_bus_write_flush();
            uint8_t * xipPointer = (uint8_t *)(ui32MspiXipBaseAddress[pFlash->ui32Module] + (AM_DEVICES_MSPI_ATXP032_SECTOR_FOR_TIMING_CHECK << 16) + ui32AddressOffset);
            memcpy((uint8_t*)ui8RxBuffer, xipPointer, ui32TestBytes);

            //
            // Quit XIP mode
            //
            ui32Status = mspi_atxp032_disable_xip(flashHandle);
            if (AM_DEVICES_MSPI_ATXP032_STATUS_SUCCESS != ui32Status)
            {
                am_util_debug_printf("Failed to disable XIP mode in the MSPI!\n");
            }
        }
        else
        {
            ui32Status = mspi_atxp032_dma_read(flashHandle, ui8RxBuffer,
                                                (AM_DEVICES_MSPI_ATXP032_SECTOR_FOR_TIMING_CHECK << 16) + ui32AddressOffset,
                                                ui32TestBytes);
        }
#endif

        if ( ui32Status ==  AM_DEVICES_MSPI_ATXP032_STATUS_ERROR)
        {
            return true;
        }

        //
        // Verify the result
        //
        if ( memcmp(ui8RxBuffer, ui8TxBuffer, ui32TestBytes) )
        {
            //am_util_debug_printf("    Failed to verify at offset 0x%08x!\n", ui32AddressOffset);
            // verify failed, return directly
            return true;
        }

        ui32AddressOffset += ui32TestBytes;
    }

    return false;
}

//
// Static helper function:
//  Count the longest consecutive 1s in a 32bit word
//
static uint32_t
count_consecutive_ones(uint32_t* pVal)
{
    uint32_t count = 0;
    uint32_t data = *pVal;

    while ( data )
    {
        data = (data & (data << 1));
        count++;
    }
    return count;
}

//
// Static helper function:
//  Find and return the mid point of the longest continuous 1s in a 32bit word
//
static uint32_t
find_mid_point(uint32_t* pVal)
{
    uint32_t pattern_len = 0;
    uint32_t max_len = 0;
    uint32_t pick_point = 0;
    bool pattern_start = false;
    uint32_t val = *pVal;
    uint8_t remainder = 0;
    bool pick_point_flag = false;

    for ( uint32_t i = 0; i < 32; i++ )
    {
        if ( val & (0x01 << i) )
        {
            pattern_start = true;
            pattern_len++;
        }
        else
        {
            if ( pattern_start == true )
            {
                pattern_start = false;
                pick_point_flag = true;
            }
        }
        if ( (i == 31) && ( pattern_start == true ) )
        {
            pick_point_flag = true;
        }

        if (pick_point_flag == true)
        {
            if ( pattern_len > max_len )
            {
                max_len = pattern_len;
                pick_point = i - 1 - pattern_len / 2;
                remainder = pattern_len % 2;
            }
            pattern_len = 0;
            pick_point_flag = false;
        }
    }

    //
    // check the passing window side
    //

    if ( (pick_point < 16) && (val & 0x00000002) )
    {
        // window is likely on low side
        pick_point = pick_point - remainder;    // minus only when pattern length is odd
    }
    else if ( (pick_point > 15) && (val & 0x40000000) )
    {
        // window is likely on high side
        pick_point = pick_point + 1;
    }
    else
    {
        // window is in the middle, no action
    }

    return pick_point;
}
#endif


#if defined(AM_PART_APOLLO4) || defined(AM_PART_APOLLO4B)

//
//! @note If you uncomment out lines, the test will automatically include them.
//! No need to change any code below.
//
const am_devices_mspi_atxp032_sdr_timing_config_t atxp032_sConfigArray[] =
{
    {6 , 0, 1}, // Turnaround=6 , RXNEG=0, RXDQSDELAY=Dummy
    {6 , 1, 1}, // Turnaround=6 , RXNEG=1, RXDQSDELAY=Dummy
    {7 , 0, 1}, // Turnaround=7 , RXNEG=0, RXDQSDELAY=Dummy
    {7 , 1, 1}, // Turnaround=7 , RXNEG=1, RXDQSDELAY=Dummy
    {8 , 0, 1}, // Turnaround=8 , RXNEG=0, RXDQSDELAY=Dummy
    {8 , 1, 1}, // Turnaround=8 , RXNEG=1, RXDQSDELAY=Dummy
    {9 , 0, 1}, // Turnaround=9 , RXNEG=0, RXDQSDELAY=Dummy
    {9 , 1, 1}, // Turnaround=9 , RXNEG=1, RXDQSDELAY=Dummy
    {10, 0, 1}, // Turnaround=10, RXNEG=0, RXDQSDELAY=Dummy
    {10, 1, 1}, // Turnaround=10, RXNEG=1, RXDQSDELAY=Dummy
    {11, 0, 1}, // Turnaround=11, RXNEG=0, RXDQSDELAY=Dummy
    {11, 1, 1}, // Turnaround=11, RXNEG=1, RXDQSDELAY=Dummy
    {12, 0, 1}, // Turnaround=12, RXNEG=0, RXDQSDELAY=Dummy
    {12, 1, 1}, // Turnaround=12, RXNEG=1, RXDQSDELAY=Dummy
};

//*****************************************************************************
//
//  Checks PSRAM timing and determine a delay setting.
//
//*****************************************************************************
uint32_t
am_devices_mspi_atxp032_sdr_init_timing_check(uint32_t module,
                                              am_devices_mspi_atxp032_config_t *pDevCfg,
                                              am_devices_mspi_atxp032_sdr_timing_config_t *pDevSdrCfg)
{
    uint32_t ui32Status;
    void *pDevHandle;
    void *pHandle;

    uint32_t ui32ResultArray[sizeof(atxp032_sConfigArray) / sizeof(am_devices_mspi_atxp032_sdr_timing_config_t)] = {0};
    const uint32_t ui32TestSize = sizeof(atxp032_sConfigArray) / sizeof(am_devices_mspi_atxp032_sdr_timing_config_t);

    am_hal_mspi_dqs_t scanCfg =
    {
        .bDQSEnable             = 0,
        .bEnableFineDelay       = 1,
        .bOverrideRXDQSDelay    = 1,
        .ui8RxDQSDelay          = 15,
        .bOverrideTXDQSDelay    = 0,
        .ui8TxDQSDelay          = 0,
        .bDQSSyncNeg            = 0,
        .ui8DQSDelay            = 0,
        .ui8PioTurnaround       = 8,
        .ui8XipTurnaround       = 8,
        .bRxNeg                 = 0,
    };

    // clear previous saved config, rescan
    if ( bSDRTimingConfigSaved == true )
    {
        bSDRTimingConfigSaved                   = false;
        SDRTimingConfigStored.ui32Rxdqsdelay    = SDRTimingConfigDefault.ui32Rxdqsdelay;
        SDRTimingConfigStored.ui32Rxneg         = SDRTimingConfigDefault.ui32Rxneg;
        SDRTimingConfigStored.ui32Turnaround    = SDRTimingConfigDefault.ui32Turnaround;
    }

    //
    // initialize interface
    //
    ui32Status = am_devices_mspi_atxp032_init(module, pDevCfg, &pDevHandle, &pHandle);
    if (AM_DEVICES_MSPI_ATXP032_STATUS_SUCCESS != ui32Status)
    {
        am_util_debug_printf("    Failed to configure the MSPI and Flash Device correctly!\n");
        return ui32Status;
    }

    //
    // erase target sector first (each "sector is 64Kbyte block")
    //
    if ( FLASH_TIMING_SCAN_SIZE_BYTES % AM_DEVICES_MSPI_ATXP032_SECTOR_SIZE )
    {
        // scan size shall be at block boundary
        am_util_debug_printf("ERROR: Timing scan data size shall be at sector boundary!\n");
        return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
    }
    for ( uint8_t i = 0; i < (FLASH_TIMING_SCAN_SIZE_BYTES / AM_DEVICES_MSPI_ATXP032_SECTOR_SIZE); i++ )
    {
        ui32Status = am_devices_mspi_atxp032_sector_erase(pDevHandle,
                                                        (AM_DEVICES_MSPI_ATXP032_SECTOR_FOR_TIMING_CHECK << 16) + i*AM_DEVICES_MSPI_ATXP032_SECTOR_SIZE);

        if (AM_DEVICES_MSPI_ATXP032_STATUS_SUCCESS != ui32Status)
        {
            am_util_debug_printf("Failed to erase Flash Device sector!\n");
            return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
        }
    }

    // write test pattern into target sector
    if ( flash_write(pDevHandle, FLASH_TIMING_SCAN_SIZE_BYTES) )
    {
        return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
    }

    //
    // Start scan loop
    //
    for ( uint8_t i = 0; i < ui32TestSize; i++ )
    {
        // set Turnaround and RXNEG
        scanCfg.ui8PioTurnaround    = scanCfg.ui8XipTurnaround = atxp032_sConfigArray[i].ui32Turnaround;
        scanCfg.bRxNeg              = atxp032_sConfigArray[i].ui32Rxneg;
        for ( uint8_t RxDqs_Index = 1; RxDqs_Index < 31; RxDqs_Index++ )
        {
            // set RXDQSDELAY0 value
            scanCfg.ui8RxDQSDelay   = RxDqs_Index;
            // apply settings
            ui32Status = am_hal_mspi_control(pHandle, AM_HAL_MSPI_REQ_DQS, &scanCfg);
            if (AM_HAL_STATUS_SUCCESS != ui32Status)
            {
                return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
            }

            // run data check
            if ( false == flash_check(pDevHandle, FLASH_TIMING_SCAN_SIZE_BYTES) )
            {
                // data check pass
                ui32ResultArray[i] |= 0x01 << RxDqs_Index;
            }
            else
            {
                // data check failed
            }
        }
    }

    //
    // Check result
    //
    uint32_t ui32MaxOnesIndex = 0;
    uint32_t ui32MaxOnes = 0;
    uint32_t ui32Result = 0;
    for ( uint32_t i = 0; i < ui32TestSize; i++ )
    {
        ui32Result = count_consecutive_ones(&ui32ResultArray[i]);
        if ( ui32Result > ui32MaxOnes )
        {
            ui32MaxOnes = ui32Result;
            ui32MaxOnesIndex = i;
        }

        //
        // print result for test
        //
        am_util_debug_printf(" Turnaround %d - RXNEG %d = 0x%08X\n", atxp032_sConfigArray[i].ui32Turnaround, atxp032_sConfigArray[i].ui32Rxneg, ui32ResultArray[i]);
    }

    am_util_debug_printf("Timing Scan found a window %d fine steps wide.\n", ui32MaxOnes);

    //
    // Find RXDQSDELAY Value
    //
    uint32_t dqsdelay = find_mid_point(&ui32ResultArray[ui32MaxOnesIndex]);

    //
    // Deinitialize the MSPI interface
    //
    am_devices_mspi_atxp032_deinit(pDevHandle);
    NVIC_ClearPendingIRQ(mspi_interrupts[module]);

    //
    // Check consecutive passing settings
    //
    if ( ui32MaxOnes < ATXP032_TIMING_SCAN_MIN_ACCEPTANCE_LENGTH )
    {
        // too short is the passing settings, use default setting
        pDevSdrCfg->ui32Rxdqsdelay = SDRTimingConfigDefault.ui32Rxdqsdelay;
        pDevSdrCfg->ui32Rxneg = SDRTimingConfigDefault.ui32Rxneg;
        pDevSdrCfg->ui32Turnaround = SDRTimingConfigDefault.ui32Turnaround;
        return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
    }
    else
    {
        //
        // Set output values
        //
        pDevSdrCfg->ui32Rxdqsdelay = dqsdelay;
        pDevSdrCfg->ui32Rxneg = atxp032_sConfigArray[ui32MaxOnesIndex].ui32Rxneg;
        pDevSdrCfg->ui32Turnaround = atxp032_sConfigArray[ui32MaxOnesIndex].ui32Turnaround;

        return AM_DEVICES_MSPI_ATXP032_STATUS_SUCCESS;
    }
}

//*****************************************************************************
//
//  Apply given SDR timing settings to target MSPI instance.
//
//*****************************************************************************
uint32_t
am_devices_mspi_atxp032_apply_sdr_timing(void *pHandle,
                                         am_devices_mspi_atxp032_sdr_timing_config_t *pDevSdrCfg)
{
    am_devices_mspi_atxp032_t *pFlash = (am_devices_mspi_atxp032_t *)pHandle;
    am_hal_mspi_dqs_t applyCfg =
    {
        .bDQSEnable             = 0,
        .bEnableFineDelay       = 1,
        .bOverrideRXDQSDelay    = 1,
        .bOverrideTXDQSDelay    = 0,
        .ui8TxDQSDelay          = 0,
        .bDQSSyncNeg            = 0,
        .ui8DQSDelay            = 0,
    };

    // apply timing settings: Turnaround, RXNEG and RXDQSDELAY
    applyCfg.ui8RxDQSDelay      = pDevSdrCfg->ui32Rxdqsdelay;
    applyCfg.ui8PioTurnaround   = pDevSdrCfg->ui32Turnaround;
    applyCfg.ui8XipTurnaround   = pDevSdrCfg->ui32Turnaround;
    applyCfg.bRxNeg             = pDevSdrCfg->ui32Rxneg;

    // save a local copy of the timing settings
    if ( bSDRTimingConfigSaved == false )
    {
        bSDRTimingConfigSaved                   = true;
        SDRTimingConfigStored.ui32Rxdqsdelay    = pDevSdrCfg->ui32Rxdqsdelay;
        SDRTimingConfigStored.ui32Rxneg         = pDevSdrCfg->ui32Rxneg;
        SDRTimingConfigStored.ui32Turnaround    = pDevSdrCfg->ui32Turnaround;
    }

    return am_hal_mspi_control(pFlash->pMspiHandle, AM_HAL_MSPI_REQ_DQS, &applyCfg);

}
#endif

#if defined(AM_PART_APOLLO4P) || defined(AM_PART_APOLLO4L)

const am_devices_mspi_atxp032_sdr_timing_config_t atxp032_sConfigArray[] =
{
//TXNEG RXNEG RXCAP TXDLY RXDLY TURNAROUND
    {1 ,  0,   0,    1,    1,      6},
    {1 ,  0,   0,    2,    1,      6},
    {1 ,  0,   0,    3,    1,      6},
    {1 ,  0,   0,    4,    1,      6},
    {1 ,  0,   0,    5,    1,      6},
    {1 ,  0,   0,    6,    1,      6},
    {1 ,  0,   0,    7,    1,      6},
    {1 ,  0,   0,    8,    1,      6},
    {1 ,  0,   0,    9,    1,      6},
    {1 ,  0,   0,    1,    1,      7},
    {1 ,  0,   0,    2,    1,      7},
    {1 ,  0,   0,    3,    1,      7},
    {1 ,  0,   0,    4,    1,      7},
    {1 ,  0,   0,    5,    1,      7},
    {1 ,  0,   0,    6,    1,      7},
    {1 ,  0,   0,    7,    1,      7},
    {1 ,  0,   0,    8,    1,      7},
    {1 ,  0,   0,    9,    1,      7},
    {1 ,  0,   0,    1,    1,      8},
    {1 ,  0,   0,    2,    1,      8},
    {1 ,  0,   0,    3,    1,      8},
    {1 ,  0,   0,    4,    1,      8},
    {1 ,  0,   0,    5,    1,      8},
    {1 ,  0,   0,    6,    1,      8},
    {1 ,  0,   0,    7,    1,      8},
    {1 ,  0,   0,    8,    1,      8},
    {1 ,  0,   0,    9,    1,      8},
    {1 ,  0,   0,    1,    1,      9},
    {1 ,  0,   0,    2,    1,      9},
    {1 ,  0,   0,    3,    1,      9},
    {1 ,  0,   0,    4,    1,      9},
    {1 ,  0,   0,    5,    1,      9},
    {1 ,  0,   0,    6,    1,      9},
    {1 ,  0,   0,    7,    1,      9},
    {1 ,  0,   0,    8,    1,      9},
    {1 ,  0,   0,    9,    1,      9},
    {1 ,  0,   0,    1,    1,     10},
    {1 ,  0,   0,    2,    1,     10},
    {1 ,  0,   0,    3,    1,     10},
    {1 ,  0,   0,    4,    1,     10},
    {1 ,  0,   0,    5,    1,     10},
    {1 ,  0,   0,    6,    1,     10},
    {1 ,  0,   0,    7,    1,     10},
    {1 ,  0,   0,    8,    1,     10},
    {1 ,  0,   0,    9,    1,     10},
    {1 ,  0,   0,    1,    1,     11},
    {1 ,  0,   0,    2,    1,     11},
    {1 ,  0,   0,    3,    1,     11},
    {1 ,  0,   0,    4,    1,     11},
    {1 ,  0,   0,    5,    1,     11},
    {1 ,  0,   0,    6,    1,     11},
    {1 ,  0,   0,    7,    1,     11},
    {1 ,  0,   0,    8,    1,     11},
    {1 ,  0,   0,    9,    1,     11},
    {1 ,  0,   1,    1,    1,      6},
    {1 ,  0,   1,    2,    1,      6},
    {1 ,  0,   1,    3,    1,      6},
    {1 ,  0,   1,    4,    1,      6},
    {1 ,  0,   1,    5,    1,      6},
    {1 ,  0,   1,    6,    1,      6},
    {1 ,  0,   1,    7,    1,      6},
    {1 ,  0,   1,    8,    1,      6},
    {1 ,  0,   1,    9,    1,      6},
    {1 ,  0,   1,    1,    1,      7},
    {1 ,  0,   1,    2,    1,      7},
    {1 ,  0,   1,    3,    1,      7},
    {1 ,  0,   1,    4,    1,      7},
    {1 ,  0,   1,    5,    1,      7},
    {1 ,  0,   1,    6,    1,      7},
    {1 ,  0,   1,    7,    1,      7},
    {1 ,  0,   1,    8,    1,      7},
    {1 ,  0,   1,    9,    1,      7},
    {1 ,  0,   1,    1,    1,      8},
    {1 ,  0,   1,    2,    1,      8},
    {1 ,  0,   1,    3,    1,      8},
    {1 ,  0,   1,    4,    1,      8},
    {1 ,  0,   1,    5,    1,      8},
    {1 ,  0,   1,    6,    1,      8},
    {1 ,  0,   1,    7,    1,      8},
    {1 ,  0,   1,    8,    1,      8},
    {1 ,  0,   1,    9,    1,      8},
    {1 ,  0,   1,    1,    1,      9},
    {1 ,  0,   1,    2,    1,      9},
    {1 ,  0,   1,    3,    1,      9},
    {1 ,  0,   1,    4,    1,      9},
    {1 ,  0,   1,    5,    1,      9},
    {1 ,  0,   1,    6,    1,      9},
    {1 ,  0,   1,    7,    1,      9},
    {1 ,  0,   1,    8,    1,      9},
    {1 ,  0,   1,    9,    1,      9},
    {1 ,  0,   1,    1,    1,     10},
    {1 ,  0,   1,    2,    1,     10},
    {1 ,  0,   1,    3,    1,     10},
    {1 ,  0,   1,    4,    1,     10},
    {1 ,  0,   1,    5,    1,     10},
    {1 ,  0,   1,    6,    1,     10},
    {1 ,  0,   1,    7,    1,     10},
    {1 ,  0,   1,    8,    1,     10},
    {1 ,  0,   1,    9,    1,     10},
    {1 ,  0,   1,    1,    1,     11},
    {1 ,  0,   1,    2,    1,     11},
    {1 ,  0,   1,    3,    1,     11},
    {1 ,  0,   1,    4,    1,     11},
    {1 ,  0,   1,    5,    1,     11},
    {1 ,  0,   1,    6,    1,     11},
    {1 ,  0,   1,    7,    1,     11},
    {1 ,  0,   1,    8,    1,     11},
    {1 ,  0,   1,    9,    1,     11},
    {1 ,  1,   0,    1,    1,      6},
    {1 ,  1,   0,    2,    1,      6},
    {1 ,  1,   0,    3,    1,      6},
    {1 ,  1,   0,    4,    1,      6},
    {1 ,  1,   0,    5,    1,      6},
    {1 ,  1,   0,    6,    1,      6},
    {1 ,  1,   0,    1,    1,      7},
    {1 ,  1,   0,    2,    1,      7},
    {1 ,  1,   0,    3,    1,      7},
    {1 ,  1,   0,    4,    1,      7},
    {1 ,  1,   0,    5,    1,      7},
    {1 ,  1,   0,    6,    1,      7},
    {1 ,  1,   0,    1,    1,      8},
    {1 ,  1,   0,    2,    1,      8},
    {1 ,  1,   0,    3,    1,      8},
    {1 ,  1,   0,    4,    1,      8},
    {1 ,  1,   0,    5,    1,      8},
    {1 ,  1,   0,    6,    1,      8},
    {1 ,  1,   0,    1,    1,      9},
    {1 ,  1,   0,    2,    1,      9},
    {1 ,  1,   0,    3,    1,      9},
    {1 ,  1,   0,    4,    1,      9},
    {1 ,  1,   0,    5,    1,      9},
    {1 ,  1,   0,    6,    1,      9},
    {1 ,  1,   0,    1,    1,     10},
    {1 ,  1,   0,    2,    1,     10},
    {1 ,  1,   0,    3,    1,     10},
    {1 ,  1,   0,    4,    1,     10},
    {1 ,  1,   0,    5,    1,     10},
    {1 ,  1,   0,    6,    1,     10},
    {1 ,  1,   0,    1,    1,     11},
    {1 ,  1,   0,    2,    1,     11},
    {1 ,  1,   0,    3,    1,     11},
    {1 ,  1,   0,    4,    1,     11},
    {1 ,  1,   0,    5,    1,     11},
    {1 ,  1,   0,    6,    1,     11},
    {1 ,  1,   1,    1,    1,      6},
    {1 ,  1,   1,    2,    1,      6},
    {1 ,  1,   1,    3,    1,      6},
    {1 ,  1,   1,    4,    1,      6},
    {1 ,  1,   1,    5,    1,      6},
    {1 ,  1,   1,    6,    1,      6},
    {1 ,  1,   1,    1,    1,      7},
    {1 ,  1,   1,    2,    1,      7},
    {1 ,  1,   1,    3,    1,      7},
    {1 ,  1,   1,    4,    1,      7},
    {1 ,  1,   1,    5,    1,      7},
    {1 ,  1,   1,    6,    1,      7},
    {1 ,  1,   1,    1,    1,      8},
    {1 ,  1,   1,    2,    1,      8},
    {1 ,  1,   1,    3,    1,      8},
    {1 ,  1,   1,    4,    1,      8},
    {1 ,  1,   1,    5,    1,      8},
    {1 ,  1,   1,    6,    1,      8},
    {1 ,  1,   1,    1,    1,      9},
    {1 ,  1,   1,    2,    1,      9},
    {1 ,  1,   1,    3,    1,      9},
    {1 ,  1,   1,    4,    1,      9},
    {1 ,  1,   1,    5,    1,      9},
    {1 ,  1,   1,    6,    1,      9},
    {1 ,  1,   1,    7,    1,      9},
    {1 ,  1,   1,    8,    1,      9},
    {1 ,  1,   1,    9,    1,      9},
    {1 ,  1,   1,    1,    1,     10},
    {1 ,  1,   1,    2,    1,     10},
    {1 ,  1,   1,    3,    1,     10},
    {1 ,  1,   1,    4,    1,     10},
    {1 ,  1,   1,    5,    1,     10},
    {1 ,  1,   1,    6,    1,     10},
    {1 ,  1,   1,    7,    1,     10},
    {1 ,  1,   1,    8,    1,     10},
    {1 ,  1,   1,    9,    1,     10},
    {1 ,  1,   1,    1,    1,     11},
    {1 ,  1,   1,    2,    1,     11},
    {1 ,  1,   1,    3,    1,     11},
    {1 ,  1,   1,    4,    1,     11},
    {1 ,  1,   1,    5,    1,     11},
    {1 ,  1,   1,    6,    1,     11},
    {1 ,  1,   1,    7,    1,     11},
    {1 ,  1,   1,    8,    1,     11},
    {1 ,  1,   1,    9,    1,     11},
};

//*****************************************************************************
//
//  Checks PSRAM timing and determine a delay setting.
//
//*****************************************************************************
uint32_t
am_devices_mspi_atxp032_sdr_init_timing_check(uint32_t module,
                                              am_devices_mspi_atxp032_config_t *pDevCfg,
                                              am_devices_mspi_atxp032_sdr_timing_config_t *pDevSdrCfg)
{
    uint32_t ui32Status;
    void *pDevHandle;
    void *pHandle;

    uint32_t ui32ResultArray[sizeof(atxp032_sConfigArray) / sizeof(am_devices_mspi_atxp032_sdr_timing_config_t)] = {0};
    const uint32_t ui32TestSize = sizeof(atxp032_sConfigArray) / sizeof(am_devices_mspi_atxp032_sdr_timing_config_t);

    am_hal_mspi_timing_scan_t scanCfg =
    {
        .bTxNeg            = 1,
        .bRxNeg            = 0,
        .bRxCap            = 0,
        .ui8TxDQSDelay     = 4,
        .ui8RxDQSDelay     = 16,
        .ui8Turnaround     = 8,
    };

    am_hal_mspi_dqs_t dqsCfg;

    // clear previous saved config, rescan
    if ( bSDRTimingConfigSaved == true )
    {
        bSDRTimingConfigSaved                   = false;
        SDRTimingConfigStored.bTxNeg            = SDRTimingConfigDefault.bTxNeg;
        SDRTimingConfigStored.bRxNeg            = SDRTimingConfigDefault.bRxNeg;
        SDRTimingConfigStored.bRxCap            = SDRTimingConfigDefault.bRxCap;
        SDRTimingConfigStored.ui8TxDQSDelay     = SDRTimingConfigDefault.ui8TxDQSDelay;
        SDRTimingConfigStored.ui8RxDQSDelay     = SDRTimingConfigDefault.ui8RxDQSDelay;
        SDRTimingConfigStored.ui8Turnaround     = SDRTimingConfigDefault.ui8Turnaround;
    }

    //
    // initialize interface
    //
    ui32Status = am_devices_mspi_atxp032_init(module, pDevCfg, &pDevHandle, &pHandle);
    if (AM_DEVICES_MSPI_ATXP032_STATUS_SUCCESS != ui32Status)
    {
        am_util_debug_printf("    Failed to configure the MSPI and Flash Device correctly!\n");
        return ui32Status;
    }

    // do not use enable fine delay for command read
    dqsCfg.bDQSEnable           = false;
    dqsCfg.bDQSSyncNeg          = false;
    dqsCfg.bEnableFineDelay     = false;
    dqsCfg.ui8RxDQSDelayNeg     = 0;
    dqsCfg.bRxDQSDelayNegEN     = false;
    dqsCfg.bRxDQSDelayHiEN      = false;
    dqsCfg.ui8RxDQSDelay        = 16;
    dqsCfg.ui8TxDQSDelay        = 0;
    am_hal_mspi_control(pHandle, AM_HAL_MSPI_REQ_DQS, &dqsCfg);

    //
    // erase target sector first (each "sector is 64Kbyte block")
    //
    if ( FLASH_TIMING_SCAN_SIZE_BYTES % AM_DEVICES_MSPI_ATXP032_SECTOR_SIZE )
    {
        // scan size shall be at block boundary
        am_util_debug_printf("ERROR: Timing scan data size shall be at sector boundary!\n");
        return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
    }

    for ( uint8_t i = 0; i < (FLASH_TIMING_SCAN_SIZE_BYTES / AM_DEVICES_MSPI_ATXP032_SECTOR_SIZE); i++ )
    {
        ui32Status = am_devices_mspi_atxp032_sector_erase(pDevHandle,
                                                        (AM_DEVICES_MSPI_ATXP032_SECTOR_FOR_TIMING_CHECK << 16) + i*AM_DEVICES_MSPI_ATXP032_SECTOR_SIZE);

        if (AM_DEVICES_MSPI_ATXP032_STATUS_SUCCESS != ui32Status)
        {
            am_util_debug_printf("Failed to erase Flash Device sector!\n");
            return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
        }
    }

    // write test pattern into target sector
    if ( flash_write(pDevHandle, FLASH_TIMING_SCAN_SIZE_BYTES) )
    {
        return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
    }

    am_util_debug_printf("\nStart MSPI Timing Scan!\n");

    //
    // Start scan loop
    //
    for ( uint8_t i = 0; i < ui32TestSize; i++ )
    {
        // set Timing Scan Parameters
        scanCfg.bTxNeg                = atxp032_sConfigArray[i].bTxNeg;
        scanCfg.bRxNeg                = atxp032_sConfigArray[i].bRxNeg;
        scanCfg.bRxCap                = atxp032_sConfigArray[i].bRxCap;
        scanCfg.ui8TxDQSDelay         = atxp032_sConfigArray[i].ui8TxDQSDelay;
        scanCfg.ui8Turnaround         = atxp032_sConfigArray[i].ui8Turnaround;

        for ( uint8_t RxDqs_Index = 0; RxDqs_Index <= 31; RxDqs_Index++ )
        {
            // set RXDQSDELAY0 value
            scanCfg.ui8RxDQSDelay   = RxDqs_Index;
            // apply settings
            ui32Status = am_hal_mspi_control(pHandle, AM_HAL_MSPI_REQ_TIMING_SCAN, &scanCfg);
            if (AM_HAL_STATUS_SUCCESS != ui32Status)
            {
                return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
            }

            // run data check
            if ( false == flash_check(pDevHandle, FLASH_TIMING_SCAN_SIZE_BYTES) )
            {
                // data check pass
                ui32ResultArray[i] |= 0x01 << RxDqs_Index;
            }
            else
            {
                // data check failed
            }
        }
    }

    //
    // Check result
    //
    uint32_t ui32MaxOnesIndex = 0;
    uint32_t ui32MaxOnes = 0;
    uint32_t ui32Result = 0;
    for ( uint32_t i = 0; i < ui32TestSize; i++ )
    {
        ui32Result = count_consecutive_ones(&ui32ResultArray[i]);
        if ( ui32Result > ui32MaxOnes )
        {
            ui32MaxOnes = ui32Result;
            ui32MaxOnesIndex = i;
        }

        //
        // print result for test
        //
        am_util_debug_printf(" TxNeg %d, RxNeg %d, RxCap %d, TxDQSDelay %d, Turnaround %d == 0x%08X\n",
                             atxp032_sConfigArray[i].bTxNeg, atxp032_sConfigArray[i].bRxNeg, atxp032_sConfigArray[i].bRxCap,
                             atxp032_sConfigArray[i].ui8TxDQSDelay,
                             atxp032_sConfigArray[i].ui8Turnaround, ui32ResultArray[i]);
    }

    am_util_debug_printf("\nTiming Scan found a window %d fine steps wide in setting %d.\n", ui32MaxOnes, ui32MaxOnesIndex);
    am_util_debug_printf(" TxNeg %d, RxNeg %d, RxCap %d, TxDQSDelay %d, Turnaround %d == 0x%08X\n",
                             atxp032_sConfigArray[ui32MaxOnesIndex].bTxNeg, atxp032_sConfigArray[ui32MaxOnesIndex].bRxNeg, atxp032_sConfigArray[ui32MaxOnesIndex].bRxCap,
                             atxp032_sConfigArray[ui32MaxOnesIndex].ui8TxDQSDelay,
                             atxp032_sConfigArray[ui32MaxOnesIndex].ui8Turnaround, ui32ResultArray[ui32MaxOnesIndex]);

    //
    // Find RXDQSDELAY Value
    //
    uint32_t dqsdelay = find_mid_point(&ui32ResultArray[ui32MaxOnesIndex]);
    am_util_debug_printf("RxDQSDelay is set to %d.\n\n", dqsdelay);

    //
    // Deinitialize the MSPI interface
    //
    am_devices_mspi_atxp032_deinit(pDevHandle);
    NVIC_ClearPendingIRQ(mspi_interrupts[module]);

    //
    // Check consecutive passing settings
    //
    if ( ui32MaxOnes < ATXP032_TIMING_SCAN_MIN_ACCEPTANCE_LENGTH )
    {
        // too short is the passing settings, use default setting
        pDevSdrCfg->bTxNeg                = SDRTimingConfigDefault.bTxNeg;
        pDevSdrCfg->bRxNeg                = SDRTimingConfigDefault.bRxNeg;
        pDevSdrCfg->bRxCap                = SDRTimingConfigDefault.bRxCap;
        pDevSdrCfg->ui8TxDQSDelay         = SDRTimingConfigDefault.ui8TxDQSDelay;
        pDevSdrCfg->ui8RxDQSDelay         = SDRTimingConfigDefault.ui8RxDQSDelay;
        pDevSdrCfg->ui8Turnaround         = SDRTimingConfigDefault.ui8Turnaround;
        return AM_DEVICES_MSPI_ATXP032_STATUS_ERROR;
    }
    else
    {
        //
        // Set output values
        //
        pDevSdrCfg->ui8RxDQSDelay         = dqsdelay;
        pDevSdrCfg->bTxNeg                = atxp032_sConfigArray[ui32MaxOnesIndex].bTxNeg;
        pDevSdrCfg->bRxNeg                = atxp032_sConfigArray[ui32MaxOnesIndex].bRxNeg;
        pDevSdrCfg->bRxCap                = atxp032_sConfigArray[ui32MaxOnesIndex].bRxCap;
        pDevSdrCfg->ui8TxDQSDelay         = atxp032_sConfigArray[ui32MaxOnesIndex].ui8TxDQSDelay;
        pDevSdrCfg->ui8Turnaround         = atxp032_sConfigArray[ui32MaxOnesIndex].ui8Turnaround;
        return AM_DEVICES_MSPI_ATXP032_STATUS_SUCCESS;
    }
}

//*****************************************************************************
//
//  Apply given SDR timing settings to target MSPI instance.
//
//*****************************************************************************
uint32_t
am_devices_mspi_atxp032_apply_sdr_timing(void *pHandle,
                                         am_devices_mspi_atxp032_sdr_timing_config_t *pDevSdrCfg)
{
    am_devices_mspi_atxp032_t *pFlash = (am_devices_mspi_atxp032_t *)pHandle;
    am_hal_mspi_timing_scan_t applyCfg =
    {
        .bTxNeg            = 1,
        .bRxNeg            = 0,
        .bRxCap            = 0,
        .ui8TxDQSDelay     = 4,
        .ui8RxDQSDelay     = 16,
        .ui8Turnaround     = 8,
    };

    // apply timing setting
    applyCfg.bTxNeg                = pDevSdrCfg->bTxNeg;
    applyCfg.bRxNeg                = pDevSdrCfg->bRxNeg;
    applyCfg.bRxCap                = pDevSdrCfg->bRxCap;
    applyCfg.ui8TxDQSDelay         = pDevSdrCfg->ui8TxDQSDelay;
    applyCfg.ui8RxDQSDelay         = pDevSdrCfg->ui8RxDQSDelay;
    applyCfg.ui8Turnaround         = pDevSdrCfg->ui8Turnaround;

    // save a local copy of the timing settings
    if ( bSDRTimingConfigSaved == false )
    {
        bSDRTimingConfigSaved                   = true;
        SDRTimingConfigStored.bTxNeg            = pDevSdrCfg->bTxNeg;
        SDRTimingConfigStored.bRxNeg            = pDevSdrCfg->bRxNeg;
        SDRTimingConfigStored.bRxCap            = pDevSdrCfg->bRxCap;
        SDRTimingConfigStored.ui8TxDQSDelay     = pDevSdrCfg->ui8TxDQSDelay;
        SDRTimingConfigStored.ui8RxDQSDelay     = pDevSdrCfg->ui8RxDQSDelay;
        SDRTimingConfigStored.ui8Turnaround     = pDevSdrCfg->ui8Turnaround;
    }

    return am_hal_mspi_control(pFlash->pMspiHandle, AM_HAL_MSPI_REQ_TIMING_SCAN, &applyCfg);

}
#endif

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************

