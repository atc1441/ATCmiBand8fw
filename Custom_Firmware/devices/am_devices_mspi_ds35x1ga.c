//*****************************************************************************
//
//! @file am_devices_mspi_ds35x1ga.c
//!
//! @brief Multibit SPI ds35x1ga NAND flash driver.
//!
//! @addtogroup mspi_ds35x1ga DX35X1GA MSPI Driver
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
#include "am_devices_mspi_ds35x1ga.h"
#include "am_util_stdio.h"
#include "am_bsp.h"
#include "am_util_delay.h"
#include "am_util.h"


//*****************************************************************************
//
// Global variables.
//
//*****************************************************************************
#if defined (apollo4l_eb)
#define DS35X1GA_QUAD_CLKON4_MODE_EN
#endif

#define AM_DEVICES_MSPI_DS35X1GA_TIMEOUT       1000000
#define AM_DEVICES_MSPI_DS35X1GA_ERASE_TIMEOUT 1000000


#define NANDFLASH_TIMING_SCAN_MIN_ACCEPTANCE_LENGTH           (8)     // there should be at least
                                                                    // this amount of consecutive
                                                                    // passing settings to be accepted.
#define FLASH_CHECK_DATA_SIZE_BYTES                         2048    // Data trunk size
#define FLASH_TIMING_SCAN_PAGE_NUM                          16      // Total scan page
#define FLASH_TEST_START_PAGE                               0x000   //Flash check start page
#define FLASH_TEST_END_PAGE                                 FLASH_TEST_START_PAGE + FLASH_TIMING_SCAN_PAGE_NUM   //Flash check end page number

//
//!
//
typedef struct
{
    uint32_t ui32Module;
    void *pMspiHandle;
    am_hal_mspi_dev_config_t sCurrentSetting;       //This holds current mspi setting
    am_hal_mspi_dev_config_t sSerialSetting;        //This holds serial cmd setting
    bool bOccupied;
} am_devices_mspi_ds35x1ga_t;

//
//!
//
#if defined(AM_PART_APOLLO4_API)
static am_hal_mspi_config_t gMspiCfg =
{
  .ui32TCBSize          = 0,
  .pTCB                 = NULL,
#if defined(DS35X1GA_QUAD_CLKON4_MODE_EN)
  .bClkonD4             = 1
#else
  .bClkonD4             = 0
#endif
};
#endif

//
//!
//
static am_devices_mspi_ds35x1ga_t gAmDs35x1ga[AM_DEVICES_MSPI_DS35X1GA_MAX_DEVICE_NUM];

//
//!
//
static am_hal_mspi_dev_config_t MSPI_DS35X1GA_Serial_CE0_MSPIConfig =
{
    .eSpiMode = AM_HAL_MSPI_SPI_MODE_0,
    .eClockFreq = AM_HAL_MSPI_CLK_4MHZ,
    .ui8TurnAround = 8,
    .eAddrCfg = AM_HAL_MSPI_ADDR_2_BYTE,
    .eInstrCfg = AM_HAL_MSPI_INSTR_1_BYTE,
    .eDeviceConfig = AM_HAL_MSPI_FLASH_SERIAL_CE0,
    .bSendInstr = true,
    .bSendAddr = true,
    .bTurnaround = true,
#if defined(AM_PART_APOLLO4_API)
    .ui16ReadInstr         = AM_DEVICES_MSPI_DS35X1GA_READ_BUFFER_X1,
    .ui16WriteInstr        = AM_DEVICES_MSPI_DS35X1GA_PROGRAM_LOAD_X1,
#else
    .ui8ReadInstr = AM_DEVICES_MSPI_DS35X1GA_READ_BUFFER_X1,
    .ui8WriteInstr = AM_DEVICES_MSPI_DS35X1GA_PROGRAM_LOAD_X1,
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

//
//!
//
static am_hal_mspi_dev_config_t MSPI_DS35X1GA_Serial_CE1_MSPIConfig =
{
    .eSpiMode = AM_HAL_MSPI_SPI_MODE_0,
    .eClockFreq = AM_HAL_MSPI_CLK_4MHZ,
    .ui8TurnAround = 8,
    .eAddrCfg = AM_HAL_MSPI_ADDR_2_BYTE,
    .eInstrCfg = AM_HAL_MSPI_INSTR_1_BYTE,
    .eDeviceConfig = AM_HAL_MSPI_FLASH_SERIAL_CE1,
    .bSendInstr = true,
    .bSendAddr = true,
    .bTurnaround = true,
#if defined(AM_PART_APOLLO4_API)
    .ui16ReadInstr         = AM_DEVICES_MSPI_DS35X1GA_READ_BUFFER_X1,
    .ui16WriteInstr        = AM_DEVICES_MSPI_DS35X1GA_PROGRAM_LOAD_X1,
#else
    .ui8ReadInstr = AM_DEVICES_MSPI_DS35X1GA_READ_BUFFER_X1,
    .ui8WriteInstr = AM_DEVICES_MSPI_DS35X1GA_PROGRAM_LOAD_X1,
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


//
//!
//
static am_hal_mspi_dev_config_t MSPI_DS35X1GA_Quad_CE0_MSPIConfig =
{
    .eSpiMode = AM_HAL_MSPI_SPI_MODE_0,
    .eClockFreq = AM_HAL_MSPI_CLK_4MHZ,
    .ui8TurnAround = 8,
    .eAddrCfg = AM_HAL_MSPI_ADDR_2_BYTE,
    .eInstrCfg = AM_HAL_MSPI_INSTR_1_BYTE,
    .eDeviceConfig = AM_HAL_MSPI_FLASH_QUAD_CE0_1_1_4,
    .bSendInstr = true,
    .bSendAddr = true,
    .bTurnaround = true,
#if defined(AM_PART_APOLLO4_API)
    .ui16ReadInstr         = AM_DEVICES_MSPI_DS35X1GA_READ_BUFFER_X4,
    .ui16WriteInstr        = AM_DEVICES_MSPI_DS35X1GA_PROGRAM_LOAD_X4,
#else
    .ui8ReadInstr = AM_DEVICES_MSPI_DS35X1GA_READ_BUFFER_X4,
    .ui8WriteInstr = AM_DEVICES_MSPI_DS35X1GA_PROGRAM_LOAD_X4,
#endif

#if defined(AM_PART_APOLLO3P)
    .ui8WriteLatency      = 6,
    .bEnWriteLatency      = true,
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


//
//!
//
static am_hal_mspi_dev_config_t MSPI_DS35X1GA_Quad_CE1_MSPIConfig =
{
    .eSpiMode = AM_HAL_MSPI_SPI_MODE_0,
    .eClockFreq = AM_HAL_MSPI_CLK_4MHZ,
    .ui8TurnAround = 8,
    .eAddrCfg = AM_HAL_MSPI_ADDR_2_BYTE,
    .eInstrCfg = AM_HAL_MSPI_INSTR_1_BYTE,
    .eDeviceConfig = AM_HAL_MSPI_FLASH_QUAD_CE1_1_1_4,
    .bSendInstr = true,
    .bSendAddr = true,
    .bTurnaround = true,
#if defined(AM_PART_APOLLO4_API)
    .ui16ReadInstr         = AM_DEVICES_MSPI_DS35X1GA_READ_BUFFER_X4,
    .ui16WriteInstr        = AM_DEVICES_MSPI_DS35X1GA_PROGRAM_LOAD_X4,
#else
    .ui8ReadInstr = AM_DEVICES_MSPI_DS35X1GA_READ_BUFFER_X4,
    .ui8WriteInstr = AM_DEVICES_MSPI_DS35X1GA_PROGRAM_LOAD_X4,
#endif

#if defined(AM_PART_APOLLO3P)
    .ui8WriteLatency      = 6,
    .bEnWriteLatency      = true,
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


//
//!
//
static struct
{
    am_hal_mspi_device_e eHalDeviceEnum;
    am_hal_mspi_dev_config_t *psDevConfig;
} g_DS35X1GA_DevConfig[] =
    {
        { AM_HAL_MSPI_FLASH_SERIAL_CE0, &MSPI_DS35X1GA_Serial_CE0_MSPIConfig },
        { AM_HAL_MSPI_FLASH_SERIAL_CE1, &MSPI_DS35X1GA_Serial_CE1_MSPIConfig },
        { AM_HAL_MSPI_FLASH_QUAD_CE0_1_1_4,   &MSPI_DS35X1GA_Quad_CE0_MSPIConfig   },
        { AM_HAL_MSPI_FLASH_QUAD_CE1_1_1_4,   &MSPI_DS35X1GA_Quad_CE1_MSPIConfig   },
    };

#if defined(AM_PART_APOLLO4_API)
//! MSPI interrupts.
static const IRQn_Type mspi_interrupts[] =
{
    MSPI0_IRQn,
    MSPI1_IRQn,
    MSPI2_IRQn,
};

//
// SDR timing default setting
//
#if defined(AM_PART_APOLLO4) || defined(AM_PART_APOLLO4B)
static am_devices_mspi_ds35x1ga_sdr_timing_config_t SDRTimingConfigDefault =
{
    .ui32Turnaround     = 9,
    .ui32Rxneg          = 0,
    .ui32Rxdqsdelay     = 15
};
#elif defined(AM_PART_APOLLO4P) || defined(AM_PART_APOLLO4L)
static am_devices_mspi_ds35x1ga_sdr_timing_config_t SDRTimingConfigDefault =
{
    .bTxNeg            = 1,
    .bRxNeg            = 0,
    .bRxCap            = 0,
    .ui8TxDQSDelay     = 4,
    .ui8RxDQSDelay     = 16,
    .ui8Turnaround     = 8,
};
#endif

//
// SDR timing stored setting
//
static bool bSDRTimingConfigSaved = false;
static am_devices_mspi_ds35x1ga_sdr_timing_config_t SDRTimingConfigStored;
#endif // defined(AM_PART_APOLLO4_API)

//*****************************************************************************
//
// static function forward declarations.
//
//*****************************************************************************
static uint32_t am_devices_mspi_ds35x1ga_command_write(void *pHandle,
                                                       uint8_t ui8Instr,
                                                       bool bSendAddr,
                                                       uint32_t ui32Addr,
                                                       uint32_t *pData,
                                                       uint32_t ui32NumBytes);
static uint32_t am_devices_mspi_ds35x1ga_command_read(void *pHandle,
                                                      uint8_t ui8Instr,
                                                      bool bSendAddr,
                                                      uint32_t ui32Addr,
                                                      uint32_t *pData,
                                                      uint32_t ui32NumBytes);

//*****************************************************************************
//
// DS35X1GA Support
//
//*****************************************************************************

#if defined(AM_PART_APOLLO3P)
//*****************************************************************************
//
//! @brief
//! @param cmd
//! @return
//
//*****************************************************************************
static uint8_t
ds35x1ga_cmd_addr_len(uint8_t cmd)
{
    switch ( cmd )
    {
        //
        // zero byte length of the command
        //
        case AM_DEVICES_MSPI_DS35X1GA_WRITE_DISABLE:
        case AM_DEVICES_MSPI_DS35X1GA_WRITE_ENABLE:
        case AM_DEVICES_MSPI_DS35X1GA_RESET:
        case AM_DEVICES_MSPI_DS35X1GA_READ_ID:
        return 0;

        //
        // one byte length of the command
        //
        case AM_DEVICES_MSPI_DS35X1GA_SET_FEATURE:
        case AM_DEVICES_MSPI_DS35X1GA_GET_FEATURE:
        return 1;

        //
        // two bytes length of the command
        //
        case AM_DEVICES_MSPI_DS35X1GA_READ_BUFFER_X1:
        case AM_DEVICES_MSPI_DS35X1GA_READ_BUFFER_X2:
        case AM_DEVICES_MSPI_DS35X1GA_READ_BUFFER_X4:
        case AM_DEVICES_MSPI_DS35X1GA_PROGRAM_LOAD_X1:
        case AM_DEVICES_MSPI_DS35X1GA_PROGRAM_LOAD_X4:
        return 2;

        //
        // three bytes length of the command
        //
        case AM_DEVICES_MSPI_DS35X1GA_PROGRAM_EXECUTE:
        case AM_DEVICES_MSPI_DS35X1GA_BLOCK_ERASE:
        case AM_DEVICES_MSPI_DS35X1GA_READ_CELL_ARRAY:
        return 3;
    }
}
#endif

//*****************************************************************************
//
//! @brief
//! @param pHandle
//! @param ui8Addr
//! @param pu8iData
//! @return
//
//*****************************************************************************
static uint32_t
am_devices_mspi_ds35x1ga_read_status(void *pHandle, uint8_t ui8Addr, uint8_t *pu8iData)
{
    uint8_t status = AM_DEVICES_DS35X1GA_OIP;

    for (uint32_t i = 0; i < AM_DEVICES_MSPI_DS35X1GA_TIMEOUT; i++)
    {
        am_devices_mspi_ds35x1ga_command_read(pHandle,
            AM_DEVICES_MSPI_DS35X1GA_GET_FEATURE, true, (uint32_t)ui8Addr, (uint32_t *)(&status), 1);
        if ((status & AM_DEVICES_DS35X1GA_OIP) != AM_DEVICES_DS35X1GA_OIP)
        {
            *pu8iData = status;
            return AM_DEVICES_MSPI_DS35X1GA_STATUS_SUCCESS;
        }
        am_util_delay_us(10);
    }
    *pu8iData = status;
    return AM_DEVICES_MSPI_DS35X1GA_STATUS_ERROR;
}

//*****************************************************************************
//
//! @brief
//! @param pHandle
//! @param ui8Data
//! @param ui8Addr
//! @param pui8Data
//! @return
//
//*****************************************************************************
static uint32_t
am_devices_mspi_ds35x1ga_write_status(void *pHandle, uint8_t ui8Data, uint8_t ui8Addr, uint8_t *pui8Data)
{
    uint8_t status = 0;

    if (AM_HAL_STATUS_SUCCESS != am_devices_mspi_ds35x1ga_command_write(pHandle,
        AM_DEVICES_MSPI_DS35X1GA_SET_FEATURE, true, (uint32_t)ui8Addr, (uint32_t *)(&ui8Data), 1))
    {
        return AM_DEVICES_MSPI_DS35X1GA_STATUS_ERROR;
    }
    if (AM_HAL_STATUS_SUCCESS != am_devices_mspi_ds35x1ga_command_read(pHandle,
        AM_DEVICES_MSPI_DS35X1GA_GET_FEATURE, true, (uint32_t)ui8Addr, (uint32_t *)(&status), 1))
    {
        return AM_DEVICES_MSPI_DS35X1GA_STATUS_ERROR;
    }
    *pui8Data = status;

    return AM_DEVICES_MSPI_DS35X1GA_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief
//! @param pHandle
//! @return
//
//*****************************************************************************
static uint32_t
am_devices_mspi_ds35x1ga_reset(void *pHandle)
{
    uint8_t ui8PIOBuffer = 0x0;

    if (AM_HAL_STATUS_SUCCESS != am_devices_mspi_ds35x1ga_command_write(pHandle, AM_DEVICES_MSPI_DS35X1GA_RESET, false, 0, NULL, 0))
    {
        return AM_DEVICES_MSPI_DS35X1GA_STATUS_ERROR;
    }

    if (AM_DEVICES_MSPI_DS35X1GA_STATUS_SUCCESS != am_devices_mspi_ds35x1ga_read_status(pHandle, AM_DEVICES_MSPI_DS35X1GA_FEATURE_STATUS, &ui8PIOBuffer))
    {
        return AM_DEVICES_MSPI_DS35X1GA_STATUS_ERROR;
    }

    return AM_DEVICES_MSPI_DS35X1GA_STATUS_SUCCESS;
}
//*****************************************************************************
//
//! @brief Device specific initialization function.
//! @param pHandle
//! @return
//
//*****************************************************************************
static uint32_t
am_device_init_flash(void *pHandle)
{
    uint8_t ui8FeatureData = 0x0;
    uint8_t ui8FeatureStatus = 0x0;


    //
    // The block lock feature provides the ability to protect the entire device, or ranges of blocks,
    // from the PROGRAM and ERASE operations. After power-up, the device is in the "locked"
    // state, i.e., bits 1, 2, 3, 4, and 5 of the block lock register are set to 1.
    // To unlock all the blocks, or a range of blocks, the SET FEATURES command must be issued with
    // the A0h feature address.
    //
    if (AM_DEVICES_MSPI_DS35X1GA_STATUS_SUCCESS != am_devices_mspi_ds35x1ga_write_status(pHandle,
        ui8FeatureData, AM_DEVICES_MSPI_DS35X1GA_FEATURE_A0, &ui8FeatureStatus))
    {
        return AM_DEVICES_MSPI_DS35X1GA_STATUS_ERROR;
    }

    uint32_t ui32Id;
    if (AM_DEVICES_MSPI_DS35X1GA_STATUS_SUCCESS != am_devices_mspi_ds35x1ga_id(pHandle, &ui32Id))
    {
        am_util_stdio_printf("NAND flash ID is mismatch \r\n");
    }

    //
    // the SET FEATURES command must be issued to enable the Nand flash ECC and QE (Quad 1-1-4 mode for high speed
    // reading and writing) setting with sending B0 feature address.
    //
    ui8FeatureData = AM_DEVICES_DS35X1GA_OTP_QC_EN | AM_DEVICES_DS35X1GA_OTP_ECC_EN;
    if (AM_DEVICES_MSPI_DS35X1GA_STATUS_SUCCESS != am_devices_mspi_ds35x1ga_write_status(pHandle,
        ui8FeatureData, AM_DEVICES_MSPI_DS35X1GA_FEATURE_B0, &ui8FeatureStatus))
    {
        return AM_DEVICES_MSPI_DS35X1GA_STATUS_ERROR;
    }

    return AM_DEVICES_MSPI_DS35X1GA_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Device specific de-initialization function.
//! @param pHandle
//! @return
//
//*****************************************************************************
static uint32_t
am_device_deinit_flash(void *pHandle)
{
    // To be add if needed
    return AM_DEVICES_MSPI_DS35X1GA_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Generic Command Write function.
//
//*****************************************************************************

#if defined(AM_PART_APOLLO3P)
//*****************************************************************************
//! @brief
//! @param pHandle
//! @param ui8NewLen
//! @return
//*****************************************************************************
static uint8_t
am_devices_mspi_ds35x1ga_asize_reconfig(void * pHandle, uint8_t ui8NewLen)
{
    am_devices_mspi_ds35x1ga_t *pFlash = (am_devices_mspi_ds35x1ga_t *)pHandle;

    am_hal_mspi_control(pFlash->pMspiHandle, AM_HAL_MSPI_REQ_ASIZE_SET, &ui8NewLen);

    return ui8NewLen;
}
#endif

//*****************************************************************************
//
//! @brief
//!
//! @param pMspiHandle
//! @param pConfig
//!
//! @return
//
//*****************************************************************************
static uint32_t
am_devices_mspi_device_reconfigure(void * pHandle, am_hal_mspi_dev_config_t *pConfig)
{
    am_devices_mspi_ds35x1ga_t *pFlash = (am_devices_mspi_ds35x1ga_t *)pHandle;

    // Disable MSPI defore re-configuring it
    uint32_t ui32Status = am_hal_mspi_disable(pFlash->pMspiHandle);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        am_util_stdio_printf("Error - Failed to disble mspi.\n");
        return AM_DEVICES_MSPI_DS35X1GA_STATUS_ERROR;
    }
    //
    // Re-Configure the MSPI for the requested operation mode.
    //
    ui32Status = am_hal_mspi_device_configure(pFlash->pMspiHandle, pConfig);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        am_util_stdio_printf("Error - Failed to configure mspi.\n");
        return AM_DEVICES_MSPI_DS35X1GA_STATUS_ERROR;
    }
    // Re-Enable MSPI
    ui32Status = am_hal_mspi_enable(pFlash->pMspiHandle);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        am_util_stdio_printf("Error - Failed to configure mspi.\n");
        return AM_DEVICES_MSPI_DS35X1GA_STATUS_ERROR;
    }

    //Restore GPIO configuration
#if defined(DS35X1GA_QUAD_CLKON4_MODE_EN)
    am_bsp_mspi_clkond4_pins_enable(pFlash->ui32Module, pConfig->eDeviceConfig);
#else
    am_bsp_mspi_pins_enable(pFlash->ui32Module, pConfig->eDeviceConfig);
#endif

    return AM_DEVICES_MSPI_DS35X1GA_STATUS_SUCCESS;
}


static inline uint32_t
am_devices_mspi_ds35x1ga_enter_command_mode(void *pHandle)
{
    am_devices_mspi_ds35x1ga_t *pFlash = (am_devices_mspi_ds35x1ga_t *)pHandle;
    if (pFlash->sCurrentSetting.eDeviceConfig != pFlash->sSerialSetting.eDeviceConfig || pFlash->sCurrentSetting.eClockFreq != pFlash->sSerialSetting.eClockFreq)
    {
        return am_devices_mspi_device_reconfigure(pFlash, &pFlash->sSerialSetting);
    }
    return AM_DEVICES_MSPI_DS35X1GA_STATUS_SUCCESS;
}

static inline uint32_t
am_devices_mspi_ds35x1ga_exit_command_mode(void *pHandle)
{
    am_devices_mspi_ds35x1ga_t *pFlash = (am_devices_mspi_ds35x1ga_t *)pHandle;
    if (pFlash->sCurrentSetting.eDeviceConfig != pFlash->sSerialSetting.eDeviceConfig || pFlash->sCurrentSetting.eClockFreq != pFlash->sSerialSetting.eClockFreq)
    {
        return am_devices_mspi_device_reconfigure(pFlash, &pFlash->sCurrentSetting);
    }
    return AM_DEVICES_MSPI_DS35X1GA_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief
//!
//! @param pHandle
//! @param ui8Instr
//! @param bSendAddr
//! @param ui32Addr
//! @param pData
//! @param ui32NumBytes
//!
//! @return
//
//*****************************************************************************
static uint32_t
am_devices_mspi_ds35x1ga_command_write(void *pHandle, uint8_t ui8Instr, bool bSendAddr,
                                       uint32_t ui32Addr, uint32_t *pData,
                                       uint32_t ui32NumBytes)
{
    uint32_t ui32Status;
    am_devices_mspi_ds35x1ga_t *pFlash = (am_devices_mspi_ds35x1ga_t *)pHandle;
    am_hal_mspi_pio_transfer_t stMSPIFlashPIOTransaction = {0};

    // Create the individual write transaction.
    stMSPIFlashPIOTransaction.ui32NumBytes       = ui32NumBytes;
    stMSPIFlashPIOTransaction.eDirection         = AM_HAL_MSPI_TX;
    stMSPIFlashPIOTransaction.bSendAddr          = bSendAddr;
    stMSPIFlashPIOTransaction.ui32DeviceAddr     = ui32Addr;
    stMSPIFlashPIOTransaction.bSendInstr         = true;
    stMSPIFlashPIOTransaction.ui16DeviceInstr    = ui8Instr;
    stMSPIFlashPIOTransaction.bTurnaround        = false;
    stMSPIFlashPIOTransaction.pui32Buffer        = pData;

#if defined(AM_PART_APOLLO3P)
    stMSPIFlashPIOTransaction.bQuadCmd        = false;
#endif

#if defined(AM_PART_APOLLO4_API)
#if defined(AM_PART_APOLLO4)
    stMSPIFlashPIOTransaction.eDeviceNum         = AM_HAL_MSPI_DEVICE0;
#endif
    stMSPIFlashPIOTransaction.bDCX               = false;
    stMSPIFlashPIOTransaction.bEnWRLatency       = false;
    stMSPIFlashPIOTransaction.bContinue          = false;   // MSPI CONT is deprecated for Apollo4
#endif

#if defined(AM_PART_APOLLO4_API)
    if ( (ui8Instr == AM_DEVICES_MSPI_DS35X1GA_PROGRAM_EXECUTE) || (ui8Instr == AM_DEVICES_MSPI_DS35X1GA_READ_CELL_ARRAY) || (ui8Instr == AM_DEVICES_MSPI_DS35X1GA_BLOCK_ERASE) || (ui8Instr == AM_DEVICES_MSPI_DS35X1GA_SET_FEATURE) )
    {
        // Write status/control register command byte address
        am_hal_mspi_instr_addr_t sInstAddrCfg;

        if (ui8Instr == AM_DEVICES_MSPI_DS35X1GA_SET_FEATURE)
        {
            sInstAddrCfg.eAddrCfg = AM_HAL_MSPI_ADDR_1_BYTE;
        }
        else
        {
            sInstAddrCfg.eAddrCfg = AM_HAL_MSPI_ADDR_3_BYTE;
        }

        sInstAddrCfg.eInstrCfg = pFlash->sSerialSetting.eInstrCfg;   // keep instruction setting the same
        am_hal_mspi_control(pFlash->pMspiHandle, AM_HAL_MSPI_REQ_SET_INSTR_ADDR_LEN, &sInstAddrCfg);
    }
#else
    uint8_t nlen = 0, olen = 0;
    if (bSendAddr)
    {
        nlen = ds35x1ga_cmd_addr_len(ui8Instr);
        olen = am_devices_mspi_ds35x1ga_asize_reconfig(pFlash, nlen);
    }
#endif

#ifdef DEBUG_PRINTF
    am_util_stdio_printf("cmd write instr 0x%x, addr len: %d\n", ui8Instr, nlen);
#endif

    // Execute the transction over MSPI.
    ui32Status = am_hal_mspi_blocking_transfer(pFlash->pMspiHandle, &stMSPIFlashPIOTransaction,
                                               AM_DEVICES_MSPI_DS35X1GA_TIMEOUT);

#if defined(AM_PART_APOLLO4_API)
    // restore the address length setting
    if ( (ui8Instr == AM_DEVICES_MSPI_DS35X1GA_PROGRAM_EXECUTE) || (ui8Instr == AM_DEVICES_MSPI_DS35X1GA_READ_CELL_ARRAY) || (ui8Instr == AM_DEVICES_MSPI_DS35X1GA_BLOCK_ERASE) || (ui8Instr == AM_DEVICES_MSPI_DS35X1GA_SET_FEATURE) )
    {
        am_hal_mspi_instr_addr_t sInstAddrCfg;
        sInstAddrCfg.eAddrCfg = pFlash->sSerialSetting.eAddrCfg;
        sInstAddrCfg.eInstrCfg = pFlash->sSerialSetting.eInstrCfg;
        am_hal_mspi_control(pFlash->pMspiHandle, AM_HAL_MSPI_REQ_SET_INSTR_ADDR_LEN, &sInstAddrCfg);
    }
#else
    if (bSendAddr && nlen != olen)
    {
        am_devices_mspi_ds35x1ga_asize_reconfig(pFlash, olen);
    }
#endif

    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
      return ui32Status;
    }

    return ui32Status;
}

//*****************************************************************************
//
//! @brief  Generic Command Read function.
//!
//! @param pHandle
//! @param ui8Instr
//! @param bSendAddr
//! @param ui32Addr
//! @param pData
//! @param ui32NumBytes
//!
//! @return
//
//*****************************************************************************
static uint32_t
am_devices_mspi_ds35x1ga_command_read(void *pHandle, uint8_t ui8Instr, bool bSendAddr,
                                      uint32_t ui32Addr, uint32_t *pData,
                                      uint32_t ui32NumBytes)
{
    uint32_t ui32Status;
    am_devices_mspi_ds35x1ga_t *pFlash = (am_devices_mspi_ds35x1ga_t *)pHandle;
    am_hal_mspi_pio_transfer_t stMSPIFlashPIOTransaction = {0};

    // Create the individual write transaction.
    stMSPIFlashPIOTransaction.eDirection         = AM_HAL_MSPI_RX;
    stMSPIFlashPIOTransaction.bSendAddr          = bSendAddr;
    stMSPIFlashPIOTransaction.ui32DeviceAddr     = ui32Addr;
    stMSPIFlashPIOTransaction.bSendInstr         = true;
    stMSPIFlashPIOTransaction.ui16DeviceInstr    = ui8Instr;

#if defined(AM_PART_APOLLO3P)
    stMSPIFlashPIOTransaction.bQuadCmd        = false;
    stMSPIFlashPIOTransaction.bTurnaround     = false;
#endif

#if defined(AM_PART_APOLLO4_API)
    if ( ui8Instr == AM_DEVICES_MSPI_DS35X1GA_GET_FEATURE )
    {
        // Read status/control register command uses 1 byte address
        am_hal_mspi_instr_addr_t sInstAddrCfg;


        sInstAddrCfg.eAddrCfg = AM_HAL_MSPI_ADDR_1_BYTE;
        sInstAddrCfg.eInstrCfg = pFlash->sSerialSetting.eInstrCfg;   // keep instruction setting the same
        am_hal_mspi_control(pFlash->pMspiHandle, AM_HAL_MSPI_REQ_SET_INSTR_ADDR_LEN, &sInstAddrCfg);
    }
#else
    uint8_t nlen = 0, olen = 0;
    if (bSendAddr)
    {
        nlen = ds35x1ga_cmd_addr_len(ui8Instr);
        olen = am_devices_mspi_ds35x1ga_asize_reconfig(pFlash, nlen);
    }
#endif

    stMSPIFlashPIOTransaction.ui32NumBytes       = ui32NumBytes;
    stMSPIFlashPIOTransaction.pui32Buffer        = pData;

#if defined(AM_PART_APOLLO4_API)
#if defined(AM_PART_APOLLO4)
    stMSPIFlashPIOTransaction.eDeviceNum         = AM_HAL_MSPI_DEVICE0;
#endif
    stMSPIFlashPIOTransaction.bDCX               = false;
    stMSPIFlashPIOTransaction.bEnWRLatency       = false;
    stMSPIFlashPIOTransaction.bContinue          = false;   // MSPI CONT is deprecated for Apollo4
#endif

#ifdef DEBUG_PRINTF
    am_util_stdio_printf("cmd read instr 0x%x, addr len, : %d bSendAddr : %d\n,", ui8Instr, nlen, bSendAddr);
#endif

    // Execute the transction over MSPI.
    ui32Status = am_hal_mspi_blocking_transfer(pFlash->pMspiHandle, &stMSPIFlashPIOTransaction,
                                               AM_DEVICES_MSPI_DS35X1GA_TIMEOUT);

#if defined(AM_PART_APOLLO4_API)
    // restore the address length setting
    if ( ui8Instr == AM_DEVICES_MSPI_DS35X1GA_GET_FEATURE )
    {
        am_hal_mspi_instr_addr_t sInstAddrCfg;
        sInstAddrCfg.eAddrCfg = pFlash->sSerialSetting.eAddrCfg;
        sInstAddrCfg.eInstrCfg = pFlash->sSerialSetting.eInstrCfg;
        am_hal_mspi_control(pFlash->pMspiHandle, AM_HAL_MSPI_REQ_SET_INSTR_ADDR_LEN, &sInstAddrCfg);
    }
#else
    if ( bSendAddr && nlen != olen )
    {
        am_devices_mspi_ds35x1ga_asize_reconfig(pFlash, olen);
    }
#endif

    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
      return ui32Status;
    }

    return ui32Status;
}

// ****************************************************************************
//
//! @brief
//!
//! @param pCallbackCtxt
//!
//! @param status
//
// ****************************************************************************
static void
pfnMSPI_DS35X1GA_Callback(void *pCallbackCtxt, uint32_t status)
{
    // Set the DMA complete flag.
    *(volatile bool *)pCallbackCtxt = true;
}

// ****************************************************************************
//
//! @brief
//!
//! @param pHandle
//!
//! @return
//
// ****************************************************************************
static uint32_t
am_devices_mspi_ds35x1ga_tx_enter_ad4_mode(void *pHandle)
{
    uint32_t ui32Status;
    am_devices_mspi_ds35x1ga_t *pFlash = (am_devices_mspi_ds35x1ga_t *)pHandle;

    if ( pFlash->sCurrentSetting.eDeviceConfig == pFlash->sSerialSetting.eDeviceConfig )
    {
        return AM_DEVICES_MSPI_DS35X1GA_STATUS_SUCCESS;
    }

    //
    // Set writelatency after command mode in write mode in nand flash.
    //
    ui32Status = am_hal_mspi_control(pFlash->pMspiHandle, AM_HAL_MSPI_REQ_NAND_FLASH_SET_WLAT, &pFlash->sCurrentSetting);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
      return AM_DEVICES_MSPI_DS35X1GA_STATUS_ERROR;
    }

    return AM_DEVICES_MSPI_DS35X1GA_STATUS_SUCCESS;
}


// ****************************************************************************
//
//
//
// ****************************************************************************
uint32_t
am_devices_mspi_ds35x1ga_init(uint32_t ui32Module, const am_devices_mspi_ds35x1ga_config_t *psMSPISettings, void **ppHandle, void **ppMspiHandle)
{
    uint32_t ui32Status;
    am_hal_mspi_dev_config_t *psConfig;

    void *pMspiHandle;
    uint32_t ui32Index = 0;
    am_hal_mspi_config_t mspiCfg;

    if ((ui32Module >= AM_REG_MSPI_NUM_MODULES) || (psMSPISettings == NULL))
    {
        return AM_DEVICES_MSPI_DS35X1GA_STATUS_ERROR;
    }

    // Allocate a vacant device handle
    for (ui32Index = 0; ui32Index < AM_DEVICES_MSPI_DS35X1GA_MAX_DEVICE_NUM; ui32Index++)
    {
        if (gAmDs35x1ga[ui32Index].bOccupied == false)
        {
            psConfig = &gAmDs35x1ga[ui32Index].sCurrentSetting;
            break;
        }
    }
    if (ui32Index == AM_DEVICES_MSPI_DS35X1GA_MAX_DEVICE_NUM)
    {
        return AM_DEVICES_MSPI_DS35X1GA_STATUS_ERROR;
    }

    for (uint32_t i = 0; i < (sizeof(g_DS35X1GA_DevConfig) / sizeof(g_DS35X1GA_DevConfig[0])); i++)
    {
        if (psMSPISettings->eDeviceConfig == g_DS35X1GA_DevConfig[i].eHalDeviceEnum)
        {
            *psConfig = *g_DS35X1GA_DevConfig[i].psDevConfig;
            psConfig->eClockFreq = psMSPISettings->eClockFreq;
#if !defined(AM_PART_APOLLO4_API)
            psConfig->pTCB = psMSPISettings->pNBTxnBuf;
            psConfig->ui32TCBSize = psMSPISettings->ui32NBTxnBufLength;
            psConfig->scramblingStartAddr = psMSPISettings->ui32ScramblingStartAddr;
            psConfig->scramblingEndAddr = psMSPISettings->ui32ScramblingEndAddr;
#endif
            break;
        }
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

    //
    // Configure the MSPI for Serial or Quad-Paired Serial operation during initialization.
    //
    switch (psConfig->eDeviceConfig)
    {
        case AM_HAL_MSPI_FLASH_SERIAL_CE0:
        case AM_HAL_MSPI_FLASH_QUAD_CE0_1_1_4:
            gAmDs35x1ga[ui32Index].sSerialSetting = MSPI_DS35X1GA_Serial_CE0_MSPIConfig;
            if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_initialize(ui32Module, &pMspiHandle))
            {
                am_util_stdio_printf("Error - Failed to initialize MSPI.\n");
                return AM_DEVICES_MSPI_DS35X1GA_STATUS_ERROR;
            }

            if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_power_control(pMspiHandle, AM_HAL_SYSCTRL_WAKE, false))
            {
                am_util_stdio_printf("Error - Failed to power on MSPI.\n");
                return AM_DEVICES_MSPI_DS35X1GA_STATUS_ERROR;
            }

#if defined(AM_PART_APOLLO4_API)
            mspiCfg = gMspiCfg;
            mspiCfg.ui32TCBSize = psMSPISettings->ui32NBTxnBufLength;
            mspiCfg.pTCB = psMSPISettings->pNBTxnBuf;
            if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_configure(pMspiHandle, &mspiCfg))
            {
                am_util_stdio_printf("Error - Failed to configure MSPI device.\n");
                return AM_DEVICES_MSPI_DS35X1GA_STATUS_ERROR;
            }
#endif

            if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_device_configure(pMspiHandle, &MSPI_DS35X1GA_Serial_CE0_MSPIConfig))
            {
                am_util_stdio_printf("Error - Failed to configure MSPI.\n");
                return AM_DEVICES_MSPI_DS35X1GA_STATUS_ERROR;
            }

            if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_enable(pMspiHandle))
            {
                am_util_stdio_printf("Error - Failed to enable MSPI.\n");
                return AM_DEVICES_MSPI_DS35X1GA_STATUS_ERROR;
            }

#if defined(DS35X1GA_QUAD_CLKON4_MODE_EN)
            am_bsp_mspi_clkond4_pins_enable(ui32Module, MSPI_DS35X1GA_Serial_CE0_MSPIConfig.eDeviceConfig);
#else
            am_bsp_mspi_pins_enable(ui32Module, MSPI_DS35X1GA_Serial_CE0_MSPIConfig.eDeviceConfig);
#endif
            break;

        case AM_HAL_MSPI_FLASH_SERIAL_CE1:
        case AM_HAL_MSPI_FLASH_QUAD_CE1_1_1_4:
            gAmDs35x1ga[ui32Index].sSerialSetting = MSPI_DS35X1GA_Serial_CE1_MSPIConfig;
            if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_initialize(ui32Module, &pMspiHandle))
            {
                am_util_stdio_printf("Error - Failed to initialize MSPI.\n");
                return AM_DEVICES_MSPI_DS35X1GA_STATUS_ERROR;
            }

            if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_power_control(pMspiHandle, AM_HAL_SYSCTRL_WAKE, false))
            {
                am_util_stdio_printf("Error - Failed to power on MSPI.\n");
                return AM_DEVICES_MSPI_DS35X1GA_STATUS_ERROR;
            }

#if defined(AM_PART_APOLLO4_API)
            mspiCfg = gMspiCfg;
            mspiCfg.ui32TCBSize = psMSPISettings->ui32NBTxnBufLength;
            mspiCfg.pTCB = psMSPISettings->pNBTxnBuf;
            if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_configure(pMspiHandle, &mspiCfg))
            {
                am_util_stdio_printf("Error - Failed to configure MSPI device.\n");
                return AM_DEVICES_MSPI_DS35X1GA_STATUS_ERROR;
            }
#endif

            if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_device_configure(pMspiHandle, &MSPI_DS35X1GA_Serial_CE1_MSPIConfig))
            {
                am_util_stdio_printf("Error - Failed to configure MSPI.\n");
                return AM_DEVICES_MSPI_DS35X1GA_STATUS_ERROR;
            }

            if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_enable(pMspiHandle))
            {
                am_util_stdio_printf("Error - Failed to enable MSPI.\n");
                return AM_DEVICES_MSPI_DS35X1GA_STATUS_ERROR;
            }

#if defined(DS35X1GA_QUAD_CLKON4_MODE_EN)
            am_bsp_mspi_clkond4_pins_enable(ui32Module, MSPI_DS35X1GA_Serial_CE1_MSPIConfig.eDeviceConfig);
#else
            am_bsp_mspi_pins_enable(ui32Module, MSPI_DS35X1GA_Serial_CE1_MSPIConfig.eDeviceConfig);
#endif
            break;

        default:
            return AM_DEVICES_MSPI_DS35X1GA_STATUS_ERROR;
    }

    gAmDs35x1ga[ui32Index].pMspiHandle = pMspiHandle;
    gAmDs35x1ga[ui32Index].ui32Module = ui32Module;

    if (AM_HAL_STATUS_SUCCESS != am_devices_mspi_ds35x1ga_reset((void *)&gAmDs35x1ga[ui32Index]))
    {
        am_util_stdio_printf("Error - Failed to reset ds34x1ga.\n");
        return AM_DEVICES_MSPI_DS35X1GA_STATUS_ERROR;
    }

    //
    // Device specific MSPI Flash initialization.
    //
    ui32Status = am_device_init_flash((void *)&gAmDs35x1ga[ui32Index]);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        am_util_stdio_printf("Error - Failed to initialize nand flash ds34x1ga.\n");
        return AM_DEVICES_MSPI_DS35X1GA_STATUS_ERROR;
    }

    // Disable MSPI before re-configuring it
    ui32Status = am_hal_mspi_disable(pMspiHandle);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        am_util_stdio_printf("Error - Failed to disble mspi.\n");
        return AM_DEVICES_MSPI_DS35X1GA_STATUS_ERROR;
    }

    //
    // Re-Configure the MSPI for the requested operation mode.
    //
    ui32Status = am_hal_mspi_device_configure(pMspiHandle, psConfig);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        am_util_stdio_printf("Error - Failed to configure mspi.\n");
        return AM_DEVICES_MSPI_DS35X1GA_STATUS_ERROR;
    }
    // Re-Enable MSPI
    ui32Status = am_hal_mspi_enable(pMspiHandle);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        am_util_stdio_printf("Error - Failed to configure mspi.\n");
        return AM_DEVICES_MSPI_DS35X1GA_STATUS_ERROR;
    }

    //
    // Configure the MSPI pins.
    //
#if defined(DS35X1GA_QUAD_CLKON4_MODE_EN)
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
        return AM_DEVICES_MSPI_DS35X1GA_STATUS_ERROR;
    }

    ui32Status = am_hal_mspi_interrupt_enable(pMspiHandle, AM_HAL_MSPI_INT_CQUPD | AM_HAL_MSPI_INT_ERR );
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_DS35X1GA_STATUS_ERROR;
    }

    //
    // Return the handle.
    //
    gAmDs35x1ga[ui32Index].bOccupied = true;
    *ppMspiHandle = pMspiHandle;
    *ppHandle = (void *)&gAmDs35x1ga[ui32Index];

    //
    // Return the status.
    //
    return AM_DEVICES_MSPI_DS35X1GA_STATUS_SUCCESS;
}

// ****************************************************************************
//
//
//
// ****************************************************************************
uint32_t
am_devices_mspi_ds35x1ga_deinit(void *pHandle)
{
    uint32_t ui32Status;
    am_devices_mspi_ds35x1ga_t *pFlash = (am_devices_mspi_ds35x1ga_t *)pHandle;

    //
    // Device specific MSPI Flash de-initialization.
    //
    ui32Status = am_device_deinit_flash(pHandle);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_DS35X1GA_STATUS_ERROR;
    }

    if (AM_HAL_STATUS_SUCCESS != am_devices_mspi_ds35x1ga_reset(pHandle))
    {
        return AM_DEVICES_MSPI_DS35X1GA_STATUS_ERROR;
    }

    //
    // Disable and clear the interrupts to start with.
    //
    ui32Status = am_hal_mspi_interrupt_disable(pFlash->pMspiHandle, 0xFFFFFFFF);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_DS35X1GA_STATUS_ERROR;
    }
    ui32Status = am_hal_mspi_interrupt_clear(pFlash->pMspiHandle, 0xFFFFFFFF);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_DS35X1GA_STATUS_ERROR;
    }

    //
    // Disable the MSPI instance.
    //
    ui32Status = am_hal_mspi_disable(pFlash->pMspiHandle);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_DS35X1GA_STATUS_ERROR;
    }

    if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_power_control(pFlash->pMspiHandle, AM_HAL_SYSCTRL_DEEPSLEEP, false))
    {
        am_util_stdio_printf("Error - Failed to power on MSPI.\n");
        return AM_DEVICES_MSPI_DS35X1GA_STATUS_ERROR;
    }

    //
    // Deinitialize the MSPI instance.
    //
    ui32Status = am_hal_mspi_deinitialize(pFlash->pMspiHandle);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_DS35X1GA_STATUS_ERROR;
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
    return AM_DEVICES_MSPI_DS35X1GA_STATUS_SUCCESS;
}

//*****************************************************************************
//
//  Read the ID of the NAND flash.
//
//*****************************************************************************
uint32_t
am_devices_mspi_ds35x1ga_id(void *pHandle, uint32_t *pui32DeviceID)
{
    uint32_t ui32Status;

    am_devices_mspi_ds35x1ga_enter_command_mode(pHandle);

    //
    // Send the command sequence to read the Device ID and return status.
    //
    uint8_t ui8Response[3];
    ui32Status = am_devices_mspi_ds35x1ga_command_read(pHandle, AM_DEVICES_MSPI_DS35X1GA_READ_ID, false, 0, (uint32_t *)&ui8Response[0], 3);
    *pui32DeviceID = (ui8Response[1] << 8) | ui8Response[2];
    am_util_debug_printf("Device ID: %04x\n", *pui32DeviceID);
    if (((*pui32DeviceID & AM_DEVICES_MSPI_DS35X1GA_ID_MASK) == AM_DEVICES_MSPI_DS35X1GA_ID) &&
        (AM_HAL_STATUS_SUCCESS == ui32Status))
    {
        am_devices_mspi_ds35x1ga_exit_command_mode(pHandle);
        return AM_DEVICES_MSPI_DS35X1GA_STATUS_SUCCESS;
    }
    else
    {
        am_devices_mspi_ds35x1ga_exit_command_mode(pHandle);
        return AM_DEVICES_MSPI_DS35X1GA_STATUS_ERROR;
    }
}

// ****************************************************************************
//
//
// ****************************************************************************
#if defined (AM_PART_APOLLO3P)
static uint8_t PageBuffer[AM_DEVICES_MSPI_DS35X1GA_PAGE_FULL_SIZE];
#elif defined (AM_PART_APOLLO4) || (AM_PART_APOLLO4B)
AM_SHARED_RW static uint8_t PageBuffer[AM_DEVICES_MSPI_DS35X1GA_PAGE_FULL_SIZE + 8];
#else
AM_SHARED_RW static uint8_t PageBuffer[AM_DEVICES_MSPI_DS35X1GA_PAGE_FULL_SIZE];
#endif

uint32_t
am_devices_mspi_ds35x1ga_read(void *pHandle, uint32_t ui32PageNum,
                              uint8_t *pui8DataBuffer,
                              uint32_t ui32DataLen,
                              uint8_t *pui8OobBuffer,
                              uint32_t ui32OobLen,
                              uint8_t *pui32EccResult)
{
    am_hal_mspi_dma_transfer_t Transaction;
    uint32_t ui32Status;

    am_devices_mspi_ds35x1ga_t *pFlash = (am_devices_mspi_ds35x1ga_t *)pHandle;


    if (ui32PageNum >= AM_DEVICES_MSPI_DS35X1GA_MAX_PAGES)
    {
        return AM_DEVICES_MSPI_DS35X1GA_STATUS_ERROR;
    }

    am_devices_mspi_ds35x1ga_enter_command_mode(pHandle);

    ui32Status = am_devices_mspi_ds35x1ga_command_write(pHandle, AM_DEVICES_MSPI_DS35X1GA_READ_CELL_ARRAY, true, ui32PageNum, NULL, 0);

    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_DS35X1GA_STATUS_ERROR;
    }

    if (AM_DEVICES_MSPI_DS35X1GA_STATUS_SUCCESS != am_devices_mspi_ds35x1ga_read_status(pHandle, AM_DEVICES_MSPI_DS35X1GA_FEATURE_STATUS, (uint8_t *)&ui32Status))
    {
        return AM_DEVICES_MSPI_DS35X1GA_STATUS_ERROR;
    }

    switch (ui32Status & AM_DEVICES_DS35X1GA_ECCS)
    {
        case AM_DEVICES_DS35X1GA_ECCS_NO_BIT_FLIPS:
            *pui32EccResult = AM_DEVICES_MSPI_DS35X1GA_ECC_STATUS_NO_BIT_FLIPS;
            break;
        case AM_DEVICES_DS35X1GA_ECCS_BIT_FLIPS_CORRECTED:
        case AM_DEVICES_DS35X1GA_ECCS_BIT_FLIPS_CORRECTED_THR:
            *pui32EccResult = AM_DEVICES_MSPI_DS35X1GA_ECC_STATUS_BIT_FLIPS_CORRECTED;
            break;
        case AM_DEVICES_DS35X1GA_ECCS_BIT_FLIPS_NOT_CORRECTED:
            *pui32EccResult = AM_DEVICES_MSPI_DS35X1GA_ECC_STATUS_BIT_FLIPS_NOT_CORRECTED;
            break;
        default:
            *pui32EccResult = 0xFF; // invalid ECC status
            break;
    }

    am_devices_mspi_ds35x1ga_exit_command_mode(pHandle);

#if defined(AM_PART_APOLLO4_API)
    if ( bSDRTimingConfigSaved == true)
    {
#if defined(AM_PART_APOLLO4) || defined(AM_PART_APOLLO4B)
        ui32Status = am_devices_mspi_ds35x1ga_apply_sdr_timing(pFlash, &SDRTimingConfigStored);
#else
        ui32Status = am_hal_mspi_control(pFlash->pMspiHandle, AM_HAL_MSPI_REQ_TIMING_SCAN, &SDRTimingConfigStored);
#endif
        if ( AM_HAL_STATUS_SUCCESS != ui32Status )
        {
            return AM_DEVICES_MSPI_DS35X1GA_STATUS_ERROR;
        }
    }
#endif

    // Set the DMA priority
    Transaction.ui8Priority = 1;
    // Set the transfer direction to RX (Read)
    Transaction.eDirection = AM_HAL_MSPI_RX;
    // Set the transfer count in bytes.
    Transaction.ui32TransferCount = AM_DEVICES_MSPI_DS35X1GA_PAGE_FULL_SIZE;
    // Read the whole page.
    Transaction.ui32DeviceAddress = 0x0;
    // Set the target SRAM buffer address.
    Transaction.ui32SRAMAddress = (uint32_t)PageBuffer;
    // Clear the CQ stimulus.
    Transaction.ui32PauseCondition = 0;
    // Clear the post-processing
    Transaction.ui32StatusSetClr = 0;

 #if defined(AM_PART_APOLLO4)
    Transaction.eDeviceNum         = AM_HAL_MSPI_DEVICE0;
#endif

    // Start the transaction.
    volatile bool bDMAComplete = false;
    ui32Status = am_hal_mspi_nonblocking_transfer(pFlash->pMspiHandle, &Transaction, AM_HAL_MSPI_TRANS_DMA, pfnMSPI_DS35X1GA_Callback, (void *)&bDMAComplete);

    // Check the transaction status.
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_DS35X1GA_STATUS_ERROR;
    }

    // Wait for DMA Complete or Timeout
    for (uint32_t i = 0; i < AM_DEVICES_MSPI_DS35X1GA_TIMEOUT; i++)
    {
        if (bDMAComplete)
        {
            break;
        }
        //
        // Call the BOOTROM cycle function to delay for about 1 microsecond.
        //
#if defined(AM_PART_APOLLO4_API)
        am_util_delay_us(1);
#else
        am_hal_flash_delay(FLASH_CYCLES_US(1));
#endif
    }

    // Check the status.
    if (!bDMAComplete)
    {
        return AM_DEVICES_MSPI_DS35X1GA_STATUS_ERROR;
    }

    memcpy(pui8DataBuffer, PageBuffer, ui32DataLen);
    memcpy(pui8OobBuffer, (&PageBuffer[AM_DEVICES_MSPI_DS35X1GA_PAGE_DATA_SIZE]), ui32OobLen);

    // Return the status.
    return AM_DEVICES_MSPI_DS35X1GA_STATUS_SUCCESS;
}


// ****************************************************************************
//
//
// ****************************************************************************
uint32_t
am_devices_mspi_ds35x1ga_write(void *pHandle, uint32_t ui32PageNum,
                               uint8_t *pui8DataBuffer,
                               uint32_t ui32DataLen,
                               uint8_t *pui8OobBuffer,
                               uint32_t ui32OobLen)
{
    am_hal_mspi_dma_transfer_t Transaction;
    uint32_t ui32Status;

    am_devices_mspi_ds35x1ga_t *pFlash = (am_devices_mspi_ds35x1ga_t *)pHandle;

    if (ui32PageNum >= AM_DEVICES_MSPI_DS35X1GA_MAX_PAGES)
    {
        return AM_DEVICES_MSPI_DS35X1GA_STATUS_ERROR;
    }

    am_devices_mspi_ds35x1ga_enter_command_mode(pHandle);

    //
    // Send the command sequence to enable writing.
    //
    ui32Status = am_devices_mspi_ds35x1ga_command_write(pHandle, AM_DEVICES_MSPI_DS35X1GA_WRITE_ENABLE, false, 0, NULL, 0);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_DS35X1GA_STATUS_ERROR;
    }

    am_devices_mspi_ds35x1ga_exit_command_mode(pHandle);

#if defined(AM_PART_APOLLO3P)
    ui32Status = am_devices_mspi_ds35x1ga_tx_enter_ad4_mode(pHandle);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_DS35X1GA_STATUS_ERROR;
    }
#endif

#if defined (AM_PART_APOLLO4) || (AM_PART_APOLLO4B)
    uint8_t i;
    uint16_t ui16PageOffset = 0;
    if (pFlash->sCurrentSetting.eDeviceConfig == AM_HAL_MSPI_FLASH_QUAD_CE0_1_1_4 ||
        pFlash->sCurrentSetting.eDeviceConfig == AM_HAL_MSPI_FLASH_QUAD_CE1_1_1_4)
    {
        ui16PageOffset >>= 8;
        PageBuffer[7] = 0xEE;
        PageBuffer[6] = 0xEE;
        PageBuffer[5] = 0xEE;
        PageBuffer[4] = 0xEE;
        for ( i = 4; i > 0; i-- )
        {
            PageBuffer[i-1] = (ui16PageOffset & 0x01) | 0x0E;
            ui16PageOffset >>= 1;
            PageBuffer[i-1] |= ((ui16PageOffset & 0x01) | 0x0E) << 4;
            ui16PageOffset >>= 1;
        }

        memcpy(&PageBuffer[8], pui8DataBuffer, ui32DataLen);
        memcpy(&PageBuffer[AM_DEVICES_MSPI_DS35X1GA_PAGE_DATA_SIZE + 8], pui8OobBuffer, ui32OobLen);

        //
        // Disable sending row address temporarily.
        //
        ui32Status = am_hal_mspi_control(pFlash->pMspiHandle, AM_HAL_MSPI_REQ_NAND_FLASH_SENDADDR_DIS, NULL);
        if (AM_HAL_STATUS_SUCCESS != ui32Status)
        {
            return AM_DEVICES_MSPI_DS35X1GA_STATUS_ERROR;
        }
        //
        // Set the transfer count in bytes.
        //
        Transaction.ui32TransferCount = AM_DEVICES_MSPI_DS35X1GA_PAGE_FULL_SIZE + 8;
    }
    else
    {
        memcpy(PageBuffer, pui8DataBuffer, ui32DataLen);
        memcpy(&PageBuffer[AM_DEVICES_MSPI_DS35X1GA_PAGE_DATA_SIZE], pui8OobBuffer, ui32OobLen);
        //
        // Set the transfer count in bytes.
        //
        Transaction.ui32TransferCount = AM_DEVICES_MSPI_DS35X1GA_PAGE_FULL_SIZE;
    }
#else
    memcpy(PageBuffer, pui8DataBuffer, ui32DataLen);
    memcpy(&PageBuffer[AM_DEVICES_MSPI_DS35X1GA_PAGE_DATA_SIZE], pui8OobBuffer, ui32OobLen);
    //
    // Set the transfer count in bytes.
    //
    Transaction.ui32TransferCount = AM_DEVICES_MSPI_DS35X1GA_PAGE_FULL_SIZE;
#endif

    // Set the DMA priority
    Transaction.ui8Priority = 1;
    // Set the transfer direction to TX (Write)
    Transaction.eDirection = AM_HAL_MSPI_TX;

    // Program the whole page.
    Transaction.ui32DeviceAddress = 0x0;

    // Set the source SRAM buffer address.
    Transaction.ui32SRAMAddress = (uint32_t)PageBuffer;

    // Clear the CQ stimulus.
    Transaction.ui32PauseCondition = 0;
    // Clear the post-processing
    Transaction.ui32StatusSetClr = 0;

    // Start the transaction.
    volatile bool bDMAComplete = false;
    ui32Status = am_hal_mspi_nonblocking_transfer(pFlash->pMspiHandle, &Transaction, AM_HAL_MSPI_TRANS_DMA, pfnMSPI_DS35X1GA_Callback, (void *)&bDMAComplete);

    // Check the transaction status.
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_DS35X1GA_STATUS_ERROR;
    }

    // Wait for DMA Complete or Timeout
    for (uint32_t i = 0; i < AM_DEVICES_MSPI_DS35X1GA_TIMEOUT; i++)
    {
        if (bDMAComplete)
        {
            // program execute here.
            am_devices_mspi_ds35x1ga_enter_command_mode(pHandle);

            ui32Status = am_devices_mspi_ds35x1ga_command_write(pHandle, AM_DEVICES_MSPI_DS35X1GA_PROGRAM_EXECUTE, true, ui32PageNum, NULL, 0);

            if (AM_HAL_STATUS_SUCCESS != ui32Status)
            {
                am_devices_mspi_ds35x1ga_exit_command_mode(pHandle);
                return AM_DEVICES_MSPI_DS35X1GA_STATUS_ERROR;
            }

            if (AM_DEVICES_MSPI_DS35X1GA_STATUS_SUCCESS == am_devices_mspi_ds35x1ga_read_status(pHandle, AM_DEVICES_MSPI_DS35X1GA_FEATURE_STATUS, (uint8_t *)&ui32Status))
            {
                if (((ui32Status & AM_DEVICES_DS35X1GA_PRG_F) != AM_DEVICES_DS35X1GA_PRG_F) &&
                    ((ui32Status & AM_DEVICES_DS35X1GA_OIP) != AM_DEVICES_DS35X1GA_OIP))
                {
                    //
                    // Send the command sequence to disable writing.
                    //
                    ui32Status = am_devices_mspi_ds35x1ga_command_write(pHandle, AM_DEVICES_MSPI_DS35X1GA_WRITE_DISABLE, false, 0, NULL, 0);
                    if (AM_HAL_STATUS_SUCCESS != ui32Status)
                    {
                        am_devices_mspi_ds35x1ga_exit_command_mode(pHandle);
                        return AM_DEVICES_MSPI_DS35X1GA_STATUS_ERROR;
                    }
                    break;
                }
            }
            am_devices_mspi_ds35x1ga_exit_command_mode(pHandle);
        }
        //
        // Call the BOOTROM cycle function to delay for about 1 microsecond.
        //
#if defined(AM_PART_APOLLO4_API)
        am_util_delay_us(1);
#else
        am_hal_flash_delay(FLASH_CYCLES_US(1));
#endif
    }

    // Check the status.
    if (!bDMAComplete)
    {
        return AM_DEVICES_MSPI_DS35X1GA_STATUS_ERROR;
    }

    //
    // Return the status.
    //
    return AM_DEVICES_MSPI_DS35X1GA_STATUS_SUCCESS;
}

// ****************************************************************************
//
//  Erase a certain page of the NAND flash.
//
// ****************************************************************************
uint32_t
am_devices_mspi_ds35x1ga_block_erase(void *pHandle, uint32_t ui32BlockNum)
{
    uint32_t ui32Status;


    if (ui32BlockNum >= AM_DEVICES_MSPI_DS35X1GA_MAX_BLOCKS)
    {
        return AM_DEVICES_MSPI_DS35X1GA_STATUS_ERROR;
    }

    am_devices_mspi_ds35x1ga_enter_command_mode(pHandle);

    //
    // Send the command sequence to enable writing.
    //
    ui32Status = am_devices_mspi_ds35x1ga_command_write(pHandle, AM_DEVICES_MSPI_DS35X1GA_WRITE_ENABLE, false, 0, NULL, 0);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_DS35X1GA_STATUS_ERROR;
    }

    //
    // Send the command to remove protection from the sector.
    //
    ui32Status = am_devices_mspi_ds35x1ga_command_write(pHandle, AM_DEVICES_MSPI_DS35X1GA_BLOCK_ERASE, true, (ui32BlockNum << 6), NULL, 0);

    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_DS35X1GA_STATUS_ERROR;
    }

    for (uint32_t i = 0; i < AM_DEVICES_MSPI_DS35X1GA_TIMEOUT; i++)
    {
        if (AM_DEVICES_MSPI_DS35X1GA_STATUS_SUCCESS == am_devices_mspi_ds35x1ga_read_status(pHandle, AM_DEVICES_MSPI_DS35X1GA_FEATURE_STATUS, (uint8_t *)&ui32Status))
        {
            // check for ERS_F, OIP bit
            if (((ui32Status & AM_DEVICES_DS35X1GA_ERS_F) != AM_DEVICES_DS35X1GA_ERS_F) &&
                ((ui32Status & AM_DEVICES_DS35X1GA_OIP) != AM_DEVICES_DS35X1GA_OIP))
            {
                //
                // Send the command sequence to disable writing.
                //
                ui32Status = am_devices_mspi_ds35x1ga_command_write(pHandle, AM_DEVICES_MSPI_DS35X1GA_WRITE_DISABLE, false, 0, NULL, 0);
                if (AM_HAL_STATUS_SUCCESS != ui32Status)
                {
                    am_devices_mspi_ds35x1ga_exit_command_mode(pHandle);
                    return AM_DEVICES_MSPI_DS35X1GA_STATUS_ERROR;
                }
                break;
            }
        }
        //
        // Call the BOOTROM cycle function to delay for about 1 microsecond.
        //
#if defined(AM_PART_APOLLO4_API)
        am_util_delay_us(1);
#else
        am_hal_flash_delay(FLASH_CYCLES_US(1));
#endif
    }

    am_devices_mspi_ds35x1ga_exit_command_mode(pHandle);

    //
    // Return the status.
    //
    return AM_DEVICES_MSPI_DS35X1GA_STATUS_SUCCESS;
}

#if defined(AM_PART_APOLLO4_API)
static uint8_t write_oob[AM_DEVICES_MSPI_DS35X1GA_PAGE_OOB_SIZE];
static uint8_t read_oob[AM_DEVICES_MSPI_DS35X1GA_PAGE_OOB_SIZE];

static uint8_t  ui8TxBuffer[FLASH_CHECK_DATA_SIZE_BYTES];
static uint8_t  ui8RxBuffer[FLASH_CHECK_DATA_SIZE_BYTES];

#if defined(AM_PART_APOLLO4) || defined(AM_PART_APOLLO4B)
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
#else
static am_hal_mspi_timing_scan_t scanCfg =
{
    .bTxNeg            = 1,
    .bRxNeg            = 0,
    .bRxCap            = 0,
    .ui8TxDQSDelay     = 4,
    .ui8RxDQSDelay     = 16,
    .ui8Turnaround     = 8,
};
#endif
#if defined(AM_PART_APOLLO4) || defined(AM_PART_APOLLO4B)
//*****************************************************************************
//
//! @brief
//!
//! @param pHandle
//! @param ui32PageNum
//! @param pui8DataBuffer
//! @param ui32DataLen
//! @param pui8OobBuffer
//! @param ui32OobLen
//! @param pDevTimgScanCfg
//!
//! @return
//
//*****************************************************************************
static uint32_t
am_devices_mspi_ds35x1ga_timing_check_write(void *pHandle, uint32_t ui32PageNum,
                                            uint8_t *pui8DataBuffer,
                                            uint32_t ui32DataLen,
                                            uint8_t *pui8OobBuffer,
                                            uint32_t ui32OobLen, am_hal_mspi_dqs_t* pDevTimgScanCfg)
#else
//*****************************************************************************
//
//! @brief
//!
//! @param pHandle
//! @param ui32PageNum
//! @param pui8DataBuffer
//! @param ui32DataLen
//! @param pui8OobBuffer
//! @param ui32OobLen
//! @param pDevTimgScanCfg
//!
//! @return
//
//*****************************************************************************
static uint32_t
am_devices_mspi_ds35x1ga_timing_check_write(void *pHandle, uint32_t ui32PageNum,
                                        uint8_t *pui8DataBuffer,
                                        uint32_t ui32DataLen,
                                        uint8_t *pui8OobBuffer,
                                        uint32_t ui32OobLen, am_hal_mspi_timing_scan_t* pDevTimgScanCfg)
#endif
{
    am_hal_mspi_dma_transfer_t Transaction;
    uint32_t ui32Status;

    am_devices_mspi_ds35x1ga_t *pFlash = (am_devices_mspi_ds35x1ga_t *)pHandle;

    if (ui32PageNum >= AM_DEVICES_MSPI_DS35X1GA_MAX_PAGES)
    {
        return AM_DEVICES_MSPI_DS35X1GA_STATUS_ERROR;
    }

    am_devices_mspi_ds35x1ga_enter_command_mode(pHandle);

    //
    // Send the command sequence to enable writing.
    //
    ui32Status = am_devices_mspi_ds35x1ga_command_write(pHandle, AM_DEVICES_MSPI_DS35X1GA_WRITE_ENABLE, false, 0, NULL, 0);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_DS35X1GA_STATUS_ERROR;
    }

    am_devices_mspi_ds35x1ga_exit_command_mode(pHandle);

    ui32Status = am_devices_mspi_ds35x1ga_tx_enter_ad4_mode(pHandle);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_DS35X1GA_STATUS_ERROR;
    }
#if defined(AM_PART_APOLLO4) || defined(AM_PART_APOLLO4B)
    am_hal_mspi_control(pFlash->pMspiHandle, AM_HAL_MSPI_REQ_DQS, pDevTimgScanCfg);
#else
    am_hal_mspi_control(pFlash->pMspiHandle, AM_HAL_MSPI_REQ_TIMING_SCAN, pDevTimgScanCfg);
#endif

#if defined (AM_PART_APOLLO4) || (AM_PART_APOLLO4B)
    am_hal_mspi_control(pFlash->pMspiHandle, AM_HAL_MSPI_REQ_DQS, pDevTimgScanCfg);

    // Prepare the source data buffer.
    uint16_t ui16PageOffset = 0;
    if (pFlash->sCurrentSetting.eDeviceConfig == AM_HAL_MSPI_FLASH_QUAD_CE0_1_1_4 ||
        pFlash->sCurrentSetting.eDeviceConfig == AM_HAL_MSPI_FLASH_QUAD_CE1_1_1_4)
    {
        ui16PageOffset >>= 8;
        PageBuffer[7] = 0xEE;
        PageBuffer[6] = 0xEE;
        PageBuffer[5] = 0xEE;
        PageBuffer[4] = 0xEE;
        for (uint8_t i = 4; i > 0; i-- )
        {
            PageBuffer[i-1] = (ui16PageOffset & 0x01) | 0x0E;
            ui16PageOffset >>= 1;
            PageBuffer[i-1] |= ((ui16PageOffset & 0x01) | 0x0E) << 4;
            ui16PageOffset >>= 1;
        }

        memcpy(&PageBuffer[8], pui8DataBuffer, ui32DataLen);
        memcpy(&PageBuffer[AM_DEVICES_MSPI_DS35X1GA_PAGE_DATA_SIZE + 8], pui8OobBuffer, ui32OobLen);

        //
        // Disable sending row address temporarily.
        //
        ui32Status = am_hal_mspi_control(pFlash->pMspiHandle, AM_HAL_MSPI_REQ_NAND_FLASH_SENDADDR_DIS, NULL);
        if (AM_HAL_STATUS_SUCCESS != ui32Status)
        {
            return AM_DEVICES_MSPI_DS35X1GA_STATUS_ERROR;
        }
        //
        // Set the transfer count in bytes.
        //
        Transaction.ui32TransferCount = AM_DEVICES_MSPI_DS35X1GA_PAGE_FULL_SIZE + 8;
    }
    else
    {
        // Prepare the source data buffer.
        memcpy(PageBuffer, pui8DataBuffer, ui32DataLen);
        memcpy(&PageBuffer[AM_DEVICES_MSPI_DS35X1GA_PAGE_DATA_SIZE], pui8OobBuffer, ui32OobLen);
        //
        // Set the transfer count in bytes.
        //
        Transaction.ui32TransferCount = AM_DEVICES_MSPI_DS35X1GA_PAGE_FULL_SIZE;
    }
#else

    am_hal_mspi_control(pFlash->pMspiHandle, AM_HAL_MSPI_REQ_TIMING_SCAN, pDevTimgScanCfg);
    // Prepare the source data buffer.
    memcpy(PageBuffer, pui8DataBuffer, ui32DataLen);
    memcpy(&PageBuffer[AM_DEVICES_MSPI_DS35X1GA_PAGE_DATA_SIZE], pui8OobBuffer, ui32OobLen);
    //
    // Set the transfer count in bytes.
    //
    Transaction.ui32TransferCount = AM_DEVICES_MSPI_DS35X1GA_PAGE_FULL_SIZE;
#endif

    // Set the DMA priority
    Transaction.ui8Priority = 1;
    // Set the transfer direction to TX (Write)
    Transaction.eDirection = AM_HAL_MSPI_TX;

    // Program the whole page.

    Transaction.ui32DeviceAddress = 0x0;

    // Set the source SRAM buffer address.
    Transaction.ui32SRAMAddress = (uint32_t)PageBuffer;

    // Clear the CQ stimulus.
    Transaction.ui32PauseCondition = 0;
    // Clear the post-processing
    Transaction.ui32StatusSetClr = 0;

    // Start the transaction.
    volatile bool bDMAComplete = false;
    ui32Status = am_hal_mspi_nonblocking_transfer(pFlash->pMspiHandle, &Transaction, AM_HAL_MSPI_TRANS_DMA, pfnMSPI_DS35X1GA_Callback, (void *)&bDMAComplete);

    // Check the transaction status.
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_DS35X1GA_STATUS_ERROR;
    }

    // Wait for DMA Complete or Timeout
    for (uint32_t i = 0; i < AM_DEVICES_MSPI_DS35X1GA_TIMEOUT; i++)
    {

        // check DMA status without using ISR
        am_hal_mspi_interrupt_status_get(pFlash->pMspiHandle, &ui32Status, false);
        am_hal_mspi_interrupt_clear(pFlash->pMspiHandle, ui32Status);
        am_hal_mspi_interrupt_service(pFlash->pMspiHandle, ui32Status);

        if (bDMAComplete)
        {
            // program execute here.
            am_devices_mspi_ds35x1ga_enter_command_mode(pHandle);

            ui32Status = am_devices_mspi_ds35x1ga_command_write(pHandle, AM_DEVICES_MSPI_DS35X1GA_PROGRAM_EXECUTE, true, ui32PageNum, NULL, 0);

            if (AM_HAL_STATUS_SUCCESS != ui32Status)
            {
                am_devices_mspi_ds35x1ga_exit_command_mode(pHandle);
                return AM_DEVICES_MSPI_DS35X1GA_STATUS_ERROR;
            }

            if (AM_DEVICES_MSPI_DS35X1GA_STATUS_SUCCESS == am_devices_mspi_ds35x1ga_read_status(pHandle, AM_DEVICES_MSPI_DS35X1GA_FEATURE_STATUS, (uint8_t *)&ui32Status))
            {
                if (((ui32Status & AM_DEVICES_DS35X1GA_PRG_F) != AM_DEVICES_DS35X1GA_PRG_F) &&
                    ((ui32Status & AM_DEVICES_DS35X1GA_OIP) != AM_DEVICES_DS35X1GA_OIP))
                {
                    //
                    // Send the command sequence to disable writing.
                    //
                    ui32Status = am_devices_mspi_ds35x1ga_command_write(pHandle, AM_DEVICES_MSPI_DS35X1GA_WRITE_DISABLE, false, 0, NULL, 0);
                    if (AM_HAL_STATUS_SUCCESS != ui32Status)
                    {
                        am_devices_mspi_ds35x1ga_exit_command_mode(pHandle);
                        return AM_DEVICES_MSPI_DS35X1GA_STATUS_ERROR;
                    }
                    break;
                }
            }
            am_devices_mspi_ds35x1ga_exit_command_mode(pHandle);
        }

        //
        // Call the BOOTROM cycle function to delay for about 1 microsecond.
        //
#if defined(AM_PART_APOLLO4_API)
        am_util_delay_us(1);
#else
        am_hal_flash_delay(FLASH_CYCLES_US(1));
#endif
    }

    // Check the status.
    if (!bDMAComplete)
    {
        return AM_DEVICES_MSPI_DS35X1GA_STATUS_ERROR;
    }

    //
    // Return the status.
    //
    return AM_DEVICES_MSPI_DS35X1GA_STATUS_SUCCESS;
}

#if defined(AM_PART_APOLLO4) || defined(AM_PART_APOLLO4B)
//*****************************************************************************
//
//! @brief
//!
//! @param pHandle
//! @param ui32PageNum
//! @param pui8DataBuffer
//! @param ui32DataLen
//! @param pui8OobBuffer
//! @param ui32OobLen
//! @param pui32EccResult
//! @param pDevTimgScanCfg
//!
//! @return
//
//*****************************************************************************
static uint32_t
am_devices_mspi_ds35x1ga_timing_check_read(void *pHandle, uint32_t ui32PageNum,
                                           uint8_t *pui8DataBuffer,
                                           uint32_t ui32DataLen,
                                           uint8_t *pui8OobBuffer,
                                           uint32_t ui32OobLen,
                                           uint8_t *pui32EccResult, am_hal_mspi_dqs_t* pDevTimgScanCfg)
#else
//*****************************************************************************
//
//! @brief
//!
//! @param pHandle
//! @param ui32PageNum
//! @param pui8DataBuffer
//! @param ui32DataLen
//! @param pui8OobBuffer
//! @param ui32OobLen
//! @param pui32EccResult
//! @param pDevTimgScanCfg
//!
//! @return
//
//*****************************************************************************
static uint32_t
am_devices_mspi_ds35x1ga_timing_check_read(void *pHandle, uint32_t ui32PageNum,
                                       uint8_t *pui8DataBuffer,
                                       uint32_t ui32DataLen,
                                       uint8_t *pui8OobBuffer,
                                       uint32_t ui32OobLen,
                                       uint8_t *pui32EccResult, am_hal_mspi_timing_scan_t* pDevTimgScanCfg)
#endif
{
    am_hal_mspi_dma_transfer_t Transaction;
    uint32_t ui32Status;

    am_devices_mspi_ds35x1ga_t *pFlash = (am_devices_mspi_ds35x1ga_t *)pHandle;


    if (ui32PageNum >= AM_DEVICES_MSPI_DS35X1GA_MAX_PAGES)
    {
        return AM_DEVICES_MSPI_DS35X1GA_STATUS_ERROR;
    }

    am_devices_mspi_ds35x1ga_enter_command_mode(pHandle);

    ui32Status = am_devices_mspi_ds35x1ga_command_write(pHandle, AM_DEVICES_MSPI_DS35X1GA_READ_CELL_ARRAY, true, ui32PageNum, NULL, 0);

    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_DS35X1GA_STATUS_ERROR;
    }

    if (AM_DEVICES_MSPI_DS35X1GA_STATUS_SUCCESS != am_devices_mspi_ds35x1ga_read_status(pHandle, AM_DEVICES_MSPI_DS35X1GA_FEATURE_STATUS, (uint8_t *)&ui32Status))
    {
        return AM_DEVICES_MSPI_DS35X1GA_STATUS_ERROR;
    }

    switch (ui32Status & AM_DEVICES_DS35X1GA_ECCS)
    {
        case AM_DEVICES_DS35X1GA_ECCS_NO_BIT_FLIPS:
            *pui32EccResult = AM_DEVICES_MSPI_DS35X1GA_ECC_STATUS_NO_BIT_FLIPS;
            break;
        case AM_DEVICES_DS35X1GA_ECCS_BIT_FLIPS_CORRECTED:
        case AM_DEVICES_DS35X1GA_ECCS_BIT_FLIPS_CORRECTED_THR:
            *pui32EccResult = AM_DEVICES_MSPI_DS35X1GA_ECC_STATUS_BIT_FLIPS_CORRECTED;
            break;
        case AM_DEVICES_DS35X1GA_ECCS_BIT_FLIPS_NOT_CORRECTED:
            *pui32EccResult = AM_DEVICES_MSPI_DS35X1GA_ECC_STATUS_BIT_FLIPS_NOT_CORRECTED;
            break;
        default:
            *pui32EccResult = 0xFF; // invalid ECC status
            break;
    }

    am_devices_mspi_ds35x1ga_exit_command_mode(pHandle);

#if defined(AM_PART_APOLLO4) || defined(AM_PART_APOLLO4B)
    am_hal_mspi_control(pFlash->pMspiHandle, AM_HAL_MSPI_REQ_DQS, pDevTimgScanCfg);
#else
    am_hal_mspi_control(pFlash->pMspiHandle, AM_HAL_MSPI_REQ_TIMING_SCAN, pDevTimgScanCfg);
#endif
    // Set the DMA priority
    Transaction.ui8Priority = 1;
    // Set the transfer direction to RX (Read)
    Transaction.eDirection = AM_HAL_MSPI_RX;
    // Set the transfer count in bytes.
    Transaction.ui32TransferCount = AM_DEVICES_MSPI_DS35X1GA_PAGE_FULL_SIZE;
    // Read the whole page.
    Transaction.ui32DeviceAddress = 0x0;
    // Set the target SRAM buffer address.
    Transaction.ui32SRAMAddress = (uint32_t)PageBuffer;
    // Clear the CQ stimulus.
    Transaction.ui32PauseCondition = 0;
    // Clear the post-processing
    Transaction.ui32StatusSetClr = 0;

 #if defined(AM_PART_APOLLO4)
    Transaction.eDeviceNum         = AM_HAL_MSPI_DEVICE0;
#endif

    // Start the transaction.
    volatile bool bDMAComplete = false;
    ui32Status = am_hal_mspi_nonblocking_transfer(pFlash->pMspiHandle, &Transaction, AM_HAL_MSPI_TRANS_DMA, pfnMSPI_DS35X1GA_Callback, (void *)&bDMAComplete);

    // Check the transaction status.
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_DS35X1GA_STATUS_ERROR;
    }

    // Wait for DMA Complete or Timeout
    for (uint32_t i = 0; i < AM_DEVICES_MSPI_DS35X1GA_TIMEOUT; i++)
    {

        // check DMA status without using ISR
        am_hal_mspi_interrupt_status_get(pFlash->pMspiHandle, &ui32Status, false);
        am_hal_mspi_interrupt_clear(pFlash->pMspiHandle, ui32Status);
        am_hal_mspi_interrupt_service(pFlash->pMspiHandle, ui32Status);

        if (bDMAComplete)
        {
            break;
        }

        //
        // Call the BOOTROM cycle function to delay for about 1 microsecond.
        //
#if defined(AM_PART_APOLLO4_API)
        am_util_delay_us(1);
#else
        am_hal_flash_delay(FLASH_CYCLES_US(1));
#endif
    }

    // Check the status.
    if (!bDMAComplete)
    {
        return AM_DEVICES_MSPI_DS35X1GA_STATUS_ERROR;
    }

    memcpy(pui8DataBuffer, PageBuffer, ui32DataLen);
    memcpy(pui8OobBuffer, (&PageBuffer[AM_DEVICES_MSPI_DS35X1GA_PAGE_DATA_SIZE]), ui32OobLen);

    // Return the status.
    return AM_DEVICES_MSPI_DS35X1GA_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief
//! @param pHandle
//! @param cacheadd
//! @param dat
//! @param num
//! @return
//
//*****************************************************************************
static uint32_t
read_from_cache(void *pHandle, uint32_t cacheadd, uint32_t *dat, uint32_t num)
{
    uint32_t ui32Status;
    uint8_t ui8EccStatus;

    ui32Status = am_devices_mspi_ds35x1ga_timing_check_read(pHandle, cacheadd, (uint8_t *)dat, num, &read_oob[0], 16, &ui8EccStatus, &scanCfg);

    return ui32Status;
}

//*****************************************************************************
//
//! @brief
//! @param pHandle
//! @param cacheadd
//! @param dat
//! @param num
//! @return
//
//*****************************************************************************
static uint32_t
program_load(void *pHandle, uint32_t cacheadd, uint32_t *dat, uint32_t num)
{
    uint32_t ui32Status = 0;
    memset(write_oob, 0x0, sizeof(write_oob));
    ui32Status = am_devices_mspi_ds35x1ga_timing_check_write(pHandle, cacheadd, (uint8_t *)dat, num, &write_oob[0], AM_DEVICES_MSPI_DS35X1GA_PAGE_OOB_SIZE, &scanCfg);

    return ui32Status;
}

//*****************************************************************************
//
//! @brief
//! @param pattern_index
//! @param buff
//! @param len
//! @return
//
//*****************************************************************************
static int
prepare_test_pattern(uint32_t pattern_index, uint8_t* buff, uint32_t len)
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
               pui8TxPtr[i] = ((i ) & 0xFF);
            }
            break;

    }

    return 0;
}
//*****************************************************************************
//
//! @brief
//! @param flashHandle
//! @param length
//! @return
//
//*****************************************************************************
static bool
flash_check(void* flashHandle, uint32_t length)
{
    //
    // Write to target address with test pattern with given length
    // Use 5 patterns: 0x5555AAAA, 0xFFFF0000, Walking, incremental and decremental
    //
    uint8_t counter;
    for ( counter = 0; counter < FLASH_TIMING_SCAN_PAGE_NUM; counter++ )
    {
        prepare_test_pattern((counter % 5), ui8TxBuffer, FLASH_CHECK_DATA_SIZE_BYTES);

        //
        // Verify the result
        //
        if ( program_load(flashHandle, (FLASH_TEST_START_PAGE + counter), (uint32_t *)ui8TxBuffer, FLASH_CHECK_DATA_SIZE_BYTES) )
        {
            break;
        }
        if ( read_from_cache(flashHandle, (FLASH_TEST_START_PAGE + counter), (uint32_t *)ui8RxBuffer, FLASH_CHECK_DATA_SIZE_BYTES) )
        {
            break;
        }
        if ( memcmp(ui8RxBuffer, ui8TxBuffer, FLASH_CHECK_DATA_SIZE_BYTES) !=0 )
        {
            // verify failed, return directly
            break;
        }
    }
    if ( counter >= FLASH_TIMING_SCAN_PAGE_NUM )
    {
        return true;
    }

    return false;
}

//*****************************************************************************
//
//! @brief Count the longest consecutive 1s in a 32bit word
//!
//! @param pVal
//! @return
//
//*****************************************************************************

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

//*****************************************************************************
//
//! @brief Find and return the mid point of the longest continuous 1s in a 32bit word
//!
//! @param pVal
//! @return
//
//*****************************************************************************
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

        if ( pick_point_flag == true )
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

#if defined(AM_PART_APOLLO4) || defined(AM_PART_APOLLO4B)
const am_devices_mspi_ds35x1ga_sdr_timing_config_t ds35x1ga_sConfigArray[] =
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
#else
const am_devices_mspi_ds35x1ga_sdr_timing_config_t ds35x1ga_sConfigArray[] =
{
//TXNEG RXNEG RXCAP TXDLY RXDLY TURNAROUND
    {0 ,  0,   1,    0,    1,     8},
    {0 ,  0,   1,    1,    1,     8},
    {0 ,  0,   1,    2,    1,     8},
    {0 ,  0,   1,    3,    1,     8},
    {0 ,  0,   1,    4,    1,     8},
    {0 ,  0,   1,    5,    1,     8},
    {0 ,  0,   1,    6,    1,     8},
    {0 ,  0,   1,    7,    1,     8},
    {0 ,  0,   1,    8,    1,     8},
    {0 ,  0,   1,    9,    1,     8},
    {0 ,  0,   1,    10,   1,     8},
    {0 ,  0,   1,    11,   1,     8},
    {0 ,  1,   1,    0,    1,     8},
    {0 ,  1,   1,    1,    1,     8},
    {0 ,  1,   1,    2,    1,     8},
    {0 ,  1,   1,    3,    1,     8},
    {0 ,  1,   1,    4,    1,     8},
    {0 ,  1,   1,    5,    1,     8},
    {0 ,  1,   1,    6,    1,     8},
    {0 ,  1,   1,    7,    1,     8},
    {0 ,  1,   1,    8,    1,     8},
    {0 ,  1,   1,    9,    1,     8},
    {0 ,  1,   1,    10,   1,     8},
    {0 ,  1,   1,    11,   1,     8},
    {1 ,  0,   0,    0,    1,     8},
    {1 ,  0,   0,    1,    1,     8},
    {1 ,  0,   0,    2,    1,     8},
    {1 ,  0,   0,    3,    1,     8},
    {1 ,  0,   0,    4,    1,     8},
    {1 ,  0,   0,    5,    1,     8},
    {1 ,  0,   0,    6,    1,     8},
    {1 ,  0,   1,    0,    1,     8},
    {1 ,  0,   1,    1,    1,     8},
    {1 ,  0,   1,    2,    1,     8},
    {1 ,  0,   1,    3,    1,     8},
    {1 ,  0,   1,    4,    1,     8},
    {1 ,  0,   1,    5,    1,     8},
    {1 ,  0,   1,    6,    1,     8},
    {1 ,  1,   0,    0,    1,     8},
    {1 ,  1,   0,    1,    1,     8},
    {1 ,  1,   0,    2,    1,     8},
    {1 ,  1,   0,    3,    1,     8},
    {1 ,  1,   0,    4,    1,     8},
    {1 ,  1,   0,    5,    1,     8},
    {1 ,  1,   0,    6,    1,     8},
    {1 ,  1,   0,    7,    1,     8},
    {1 ,  1,   0,    8,    1,     8},
    {1 ,  1,   0,    9,    1,     8},
    {1 ,  1,   0,    10,    1,    8},
    {1 ,  1,   1,    0,    1,     8},
    {1 ,  1,   1,    1,    1,     8},
    {1 ,  1,   1,    2,    1,     8},
    {1 ,  1,   1,    3,    1,     8},
    {1 ,  1,   1,    4,    1,     8},
    {1 ,  1,   1,    5,    1,     8},
    {1 ,  1,   1,    6,    1,     8},
    {1 ,  1,   1,    7,    1,     8},
    {1 ,  1,   1,    8,    1,     8},
    {1 ,  1,   1,    9,    1,     8},
    {1 ,  1,   1,    10,   1,     8},
    {1 ,  1,   1,    4,    1,     9},
    {1 ,  1,   1,    5,    1,     9},
    {1 ,  1,   1,    6,    1,     9},
    {1 ,  1,   1,    7,    1,     9},
    {1 ,  1,   1,    8,    1,     9},
    {1 ,  1,   1,    9,    1,     9},
    {1 ,  1,   1,    10,   1,     9},
};
#endif
//*****************************************************************************
//
// Checks DS35X1GA timing and determine a delay setting.
//
//*****************************************************************************
uint32_t
am_devices_mspi_ds35x1ga_sdr_init_timing_check(uint32_t module,
                                               const am_devices_mspi_ds35x1ga_config_t *pDevCfg,
                                               am_devices_mspi_ds35x1ga_sdr_timing_config_t *pDevSdrCfg)
{
    uint32_t ui32Status;
    void *pDevHandle;
    void *pHandle;

    uint32_t ui32TestSize = sizeof(ds35x1ga_sConfigArray) / sizeof(am_devices_mspi_ds35x1ga_sdr_timing_config_t);
    uint32_t ui32ResultArray[2*(sizeof(ds35x1ga_sConfigArray) / sizeof(am_devices_mspi_ds35x1ga_sdr_timing_config_t))] = {0};

    bool  bTimingConfigSaved = bSDRTimingConfigSaved;

    // clear previous saved config, rescan
    if ( bTimingConfigSaved == true )
    {
        bSDRTimingConfigSaved = false;
    }
    //
    // initialize interface
    //
    ui32Status = am_devices_mspi_ds35x1ga_init(module, pDevCfg, &pDevHandle, &pHandle);
    if (AM_DEVICES_MSPI_DS35X1GA_STATUS_SUCCESS != ui32Status)
    {
        am_util_debug_printf("    Failed to configure the MSPI and Flash Device correctly!\n");
        return ui32Status;
    }

#if defined(FAST_TIMING_SCAN)
    if ( bTimingConfigSaved )
    {
#if defined(AM_PART_APOLLO4) || defined(AM_PART_APOLLO4B)
        scanCfg.ui8PioTurnaround    = scanCfg.ui8XipTurnaround = SDRTimingConfigStored.ui32Turnaround;
        scanCfg.bRxNeg              = SDRTimingConfigStored.ui32Rxneg;
        scanCfg.ui8RxDQSDelay       = SDRTimingConfigStored.ui32Rxdqsdelay;
#else
        scanCfg.bTxNeg              = SDRTimingConfigStored.bTxNeg;
        scanCfg.bRxNeg              = SDRTimingConfigStored.bRxNeg;
        scanCfg.bRxCap              = SDRTimingConfigStored.bRxCap;
        scanCfg.ui8TxDQSDelay       = SDRTimingConfigStored.ui8TxDQSDelay;
        scanCfg.ui8RxDQSDelay       = SDRTimingConfigStored.ui8RxDQSDelay;
        scanCfg.ui8Turnaround       = SDRTimingConfigStored.ui8Turnaround;
#endif

        // erase flash before flash check
        ui32Status = am_devices_mspi_ds35x1ga_block_erase(pDevHandle, FLASH_TEST_START_PAGE >> 6);
        if (AM_HAL_STATUS_SUCCESS != ui32Status)
        {
            return AM_DEVICES_MSPI_DS35X1GA_STATUS_ERROR;
        }
#if defined(AM_PART_APOLLO4) || defined(AM_PART_APOLLO4B)
        ui32Status = am_hal_mspi_control(pHandle, AM_HAL_MSPI_REQ_DQS, &scanCfg);
#else
        ui32Status = am_hal_mspi_control(pHandle, AM_HAL_MSPI_REQ_TIMING_SCAN, &scanCfg);
#endif
        if ( AM_HAL_STATUS_SUCCESS != ui32Status )
        {
            return AM_DEVICES_MSPI_DS35X1GA_STATUS_ERROR;
        }

        // run data check
        if ( true == flash_check(pDevHandle, FLASH_TIMING_SCAN_PAGE_NUM) )
        {
            //
            // data check pass, deinitialize the MSPI interface
            //
            am_devices_mspi_ds35x1ga_deinit(pDevHandle);
            NVIC_ClearPendingIRQ(mspi_interrupts[module]);
            am_util_debug_printf("  Skipping Timing Scan!!\n");
            return AM_DEVICES_MSPI_DS35X1GA_STATUS_SUCCESS;
        }
    }
#endif

    //
    // Start scan loop
    //
    am_util_debug_printf("timing scan running ");
    for ( uint8_t i = 0; i < ui32TestSize; i++ )
    {
         am_util_debug_printf(".");
        // set Timing Scan Parameters
#if defined(AM_PART_APOLLO4) || defined(AM_PART_APOLLO4B)
        scanCfg.ui8PioTurnaround    = scanCfg.ui8XipTurnaround = ds35x1ga_sConfigArray[i].ui32Turnaround;
        scanCfg.bRxNeg              = ds35x1ga_sConfigArray[i].ui32Rxneg;
#else
        scanCfg.bTxNeg                = ds35x1ga_sConfigArray[i].bTxNeg;
        scanCfg.bRxNeg                = ds35x1ga_sConfigArray[i].bRxNeg;
        scanCfg.bRxCap                = ds35x1ga_sConfigArray[i].bRxCap;
        scanCfg.ui8TxDQSDelay         = ds35x1ga_sConfigArray[i].ui8TxDQSDelay;
        scanCfg.ui8Turnaround         = ds35x1ga_sConfigArray[i].ui8Turnaround;
#endif
        // erase flash before flash check
        ui32Status = am_devices_mspi_ds35x1ga_block_erase(pDevHandle, FLASH_TEST_START_PAGE >> 6);
        if (AM_HAL_STATUS_SUCCESS != ui32Status)
        {
            return AM_DEVICES_MSPI_DS35X1GA_STATUS_ERROR;
        }
#if defined(AM_PART_APOLLO4) || defined(AM_PART_APOLLO4B)
        for ( uint8_t RxDqs_Index = 1; RxDqs_Index < 31; RxDqs_Index++ )
#else
        for ( uint8_t RxDqs_Index = 0; RxDqs_Index < 32; RxDqs_Index++ )
#endif
        {
            // set RXDQSDELAY0 value
            scanCfg.ui8RxDQSDelay   = RxDqs_Index;
            // apply settings
#if defined(AM_PART_APOLLO4) || defined(AM_PART_APOLLO4B)
            ui32Status = am_hal_mspi_control(pHandle, AM_HAL_MSPI_REQ_DQS, &scanCfg);
#else
            ui32Status = am_hal_mspi_control(pHandle, AM_HAL_MSPI_REQ_TIMING_SCAN, &scanCfg);
#endif
            if ( AM_HAL_STATUS_SUCCESS != ui32Status )
            {
                return AM_DEVICES_MSPI_DS35X1GA_STATUS_ERROR;
            }

            // run data check
            if ( true == flash_check(pDevHandle, FLASH_TIMING_SCAN_PAGE_NUM) )
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
    }
#if defined(AM_PART_APOLLO4) || defined(AM_PART_APOLLO4B)
    am_util_debug_printf("Timing Scan found a window %d fine steps wide.\n", ui32MaxOnes);
#else
    am_util_debug_printf("\nTiming Scan found a window %d fine steps wide in setting %d.\n", ui32MaxOnes, ui32MaxOnesIndex);
    am_util_debug_printf(" TxNeg %d, RxNeg %d, RxCap %d, TxDQSDelay %d, Turnaround %d == 0x%08X\n",
                             ds35x1ga_sConfigArray[ui32MaxOnesIndex].bTxNeg, ds35x1ga_sConfigArray[ui32MaxOnesIndex].bRxNeg, ds35x1ga_sConfigArray[ui32MaxOnesIndex].bRxCap,
                             ds35x1ga_sConfigArray[ui32MaxOnesIndex].ui8TxDQSDelay,
                             ds35x1ga_sConfigArray[ui32MaxOnesIndex].ui8Turnaround, ui32ResultArray[ui32MaxOnesIndex]);
#endif
    //
    // Find RXDQSDELAY Value
    //
    uint32_t dqsdelay = find_mid_point(&ui32ResultArray[ui32MaxOnesIndex]);
    am_util_debug_printf("RxDQSDelay is set to %d.\n\n", dqsdelay);

    //
    // Deinitialize the MSPI interface
    //
    am_devices_mspi_ds35x1ga_deinit(pDevHandle);
    NVIC_ClearPendingIRQ(mspi_interrupts[module]);

    //
    // Check consecutive passing settings
    //
    if ( ui32MaxOnes < NANDFLASH_TIMING_SCAN_MIN_ACCEPTANCE_LENGTH )
    {
#if defined(AM_PART_APOLLO4) || defined(AM_PART_APOLLO4B)
        // too short is the passing settings, use default setting
        pDevSdrCfg->ui32Rxdqsdelay = SDRTimingConfigDefault.ui32Rxdqsdelay;
        pDevSdrCfg->ui32Rxneg = SDRTimingConfigDefault.ui32Rxneg;
        pDevSdrCfg->ui32Turnaround = SDRTimingConfigDefault.ui32Turnaround;
#else
        pDevSdrCfg->bTxNeg                = SDRTimingConfigDefault.bTxNeg;
        pDevSdrCfg->bRxNeg                = SDRTimingConfigDefault.bRxNeg;
        pDevSdrCfg->bRxCap                = SDRTimingConfigDefault.bRxCap;
        pDevSdrCfg->ui8TxDQSDelay         = SDRTimingConfigDefault.ui8TxDQSDelay;
        pDevSdrCfg->ui8RxDQSDelay         = SDRTimingConfigDefault.ui8RxDQSDelay;
        pDevSdrCfg->ui8Turnaround         = SDRTimingConfigDefault.ui8Turnaround;
#endif
        return AM_DEVICES_MSPI_DS35X1GA_STATUS_ERROR;
    }
    else
    {
        //
        // Set output values
        //
#if defined(AM_PART_APOLLO4) || defined(AM_PART_APOLLO4B)
        pDevSdrCfg->ui32Rxdqsdelay = dqsdelay;
        pDevSdrCfg->ui32Rxneg = ds35x1ga_sConfigArray[ui32MaxOnesIndex].ui32Rxneg;
        pDevSdrCfg->ui32Turnaround = ds35x1ga_sConfigArray[ui32MaxOnesIndex].ui32Turnaround;
#else
        pDevSdrCfg->ui8RxDQSDelay         = dqsdelay;
        pDevSdrCfg->bTxNeg                = ds35x1ga_sConfigArray[ui32MaxOnesIndex].bTxNeg;
        pDevSdrCfg->bRxNeg                = ds35x1ga_sConfigArray[ui32MaxOnesIndex].bRxNeg;
        pDevSdrCfg->bRxCap                = ds35x1ga_sConfigArray[ui32MaxOnesIndex].bRxCap;
        pDevSdrCfg->ui8TxDQSDelay         = ds35x1ga_sConfigArray[ui32MaxOnesIndex].ui8TxDQSDelay;
        pDevSdrCfg->ui8Turnaround         = ds35x1ga_sConfigArray[ui32MaxOnesIndex].ui8Turnaround;
#endif
        return AM_DEVICES_MSPI_DS35X1GA_STATUS_SUCCESS;
    }
}


//*****************************************************************************
//
// Apply given SDR timing settings to target MSPI instance.
//
//*****************************************************************************
uint32_t
am_devices_mspi_ds35x1ga_apply_sdr_timing(void *pHandle,
                                          am_devices_mspi_ds35x1ga_sdr_timing_config_t *pDevSdrCfg)
{
    am_devices_mspi_ds35x1ga_t *pFlash = (am_devices_mspi_ds35x1ga_t *)pHandle;

    // apply timing settings: Turnaround, RXNEG and RXDQSDELAY
#if defined(AM_PART_APOLLO4) || defined(AM_PART_APOLLO4B)
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

    applyCfg.ui8RxDQSDelay      = pDevSdrCfg->ui32Rxdqsdelay;
    applyCfg.ui8PioTurnaround   = pDevSdrCfg->ui32Turnaround;
    applyCfg.ui8XipTurnaround   = pDevSdrCfg->ui32Turnaround;
    applyCfg.bRxNeg             = pDevSdrCfg->ui32Rxneg;
#else
    am_hal_mspi_timing_scan_t applyCfg =
    {
        .bTxNeg            = 1,
        .bRxNeg            = 0,
        .bRxCap            = 0,
        .ui8TxDQSDelay     = 4,
        .ui8RxDQSDelay     = 16,
        .ui8Turnaround     = 8,
    };

    applyCfg.bTxNeg                = pDevSdrCfg->bTxNeg;
    applyCfg.bRxNeg                = pDevSdrCfg->bRxNeg;
    applyCfg.bRxCap                = pDevSdrCfg->bRxCap;
    applyCfg.ui8TxDQSDelay         = pDevSdrCfg->ui8TxDQSDelay;
    applyCfg.ui8RxDQSDelay         = pDevSdrCfg->ui8RxDQSDelay;
    applyCfg.ui8Turnaround         = pDevSdrCfg->ui8Turnaround;
#endif

    // save a local copy of the timing settings
    if ( bSDRTimingConfigSaved == false )
    {
        bSDRTimingConfigSaved                   = true;
#if defined(AM_PART_APOLLO4) || defined(AM_PART_APOLLO4B)
        SDRTimingConfigStored.ui32Rxdqsdelay    = pDevSdrCfg->ui32Rxdqsdelay;
        SDRTimingConfigStored.ui32Rxneg         = pDevSdrCfg->ui32Rxneg;
        SDRTimingConfigStored.ui32Turnaround    = pDevSdrCfg->ui32Turnaround;
#elif defined(AM_PART_APOLLO4P) || defined(AM_PART_APOLLO4L)
        SDRTimingConfigStored.bTxNeg            = pDevSdrCfg->bTxNeg;
        SDRTimingConfigStored.bRxNeg            = pDevSdrCfg->bRxNeg;
        SDRTimingConfigStored.bRxCap            = pDevSdrCfg->bRxCap;
        SDRTimingConfigStored.ui8TxDQSDelay     = pDevSdrCfg->ui8TxDQSDelay;
        SDRTimingConfigStored.ui8RxDQSDelay     = pDevSdrCfg->ui8RxDQSDelay;
        SDRTimingConfigStored.ui8Turnaround     = pDevSdrCfg->ui8Turnaround;
#endif
    }
#if defined(AM_PART_APOLLO4) || defined(AM_PART_APOLLO4B)
    return am_hal_mspi_control(pFlash->pMspiHandle, AM_HAL_MSPI_REQ_DQS, &applyCfg);
#elif defined(AM_PART_APOLLO4P) || defined(AM_PART_APOLLO4L)
    return am_hal_mspi_control(pFlash->pMspiHandle, AM_HAL_MSPI_REQ_TIMING_SCAN, &applyCfg);
#endif
}

#endif

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
