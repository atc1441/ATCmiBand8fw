//*****************************************************************************
//
//! @file am_devices_mspi_is25wx064.c
//!
//! @brief General Multibit SPI Flash driver.
//!
//! @addtogroup mspi_serial_flash IS25WX064 MSPI FLASH Driver
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


#if defined(AM_PART_APOLLO4P) || defined(AM_PART_APOLLO4L)

#include <string.h>
#include "am_mcu_apollo.h"
#include "am_devices_mspi_is25wx064.h"
#include "am_util_stdio.h"
#include "am_bsp.h"
#include "am_util.h"

//*****************************************************************************
//
//! Global variables.
//
//*****************************************************************************

//#define IS25WX064_APOLLO4P_NON_DQS_ENABLE

#define AM_DEVICES_MSPI_IS25WX064_TIMEOUT                     1000000
#define AM_DEVICES_MSPI_IS25WX064_ERASE_TIMEOUT               1000000
#define AM_DEVICES_MSPI_IS25WX064_SECTOR_FOR_TIMING_CHECK     63                // max 63

#define IS25WX064_TIMING_SCAN_MIN_ACCEPTANCE_LENGTH           (8)               // there should be at least
                                                                                // this amount of consecutive
                                                                                // passing settings to be accepted.
#define FLASH_CHECK_DATA_SIZE_BYTES    AM_DEVICES_MSPI_IS25WX064_PAGE_SIZE      // Data trunk size
#define FLASH_TIMING_SCAN_SIZE_BYTES   AM_DEVICES_MSPI_IS25WX064_SECTOR_SIZE    // Total scan size
#define FLASH_TEST_PATTERN_NUMBER      5                                        // 5 patterns

//
//! MSPI device struct
//
typedef struct
{
    uint32_t                    ui32Module;
    void                        *pMspiHandle;
    am_hal_mspi_dev_config_t    stSetting;
    am_hal_mspi_dev_config_t    stCmdSetting;
    bool                        bOccupied;
} am_devices_mspi_is25wx064_t;

//
//! XIP default setting
//

static am_hal_mspi_xip_config_t gXipConfig[] =
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

//
//!
//
static am_hal_mspi_config_t gMspiCfg =
{
    .ui32TCBSize          = 0,
    .pTCB                 = NULL,
    .bClkonD4             = 0
};


//
//!
//
static am_devices_mspi_is25wx064_t gAmIS25WX064[AM_DEVICES_MSPI_IS25WX064_MAX_DEVICE_NUM];

//
//! Serial_CE0 default setting
//
static am_hal_mspi_dev_config_t
MSPI_IS25WX064_Serial_CE0_MSPIConfig =
{
    .ui8TurnAround        = 8,
    .eSpiMode             = AM_HAL_MSPI_SPI_MODE_0,
    .eClockFreq           = AM_HAL_MSPI_CLK_24MHZ,
    .eAddrCfg             = AM_HAL_MSPI_ADDR_4_BYTE,
    .eInstrCfg            = AM_HAL_MSPI_INSTR_1_BYTE,
    .eDeviceConfig        = AM_HAL_MSPI_FLASH_SERIAL_CE0,
    .bSendInstr           = true,
    .bSendAddr            = true,
    .bTurnaround          = true,
    .ui16ReadInstr         = AM_DEVICES_MSPI_IS25WX064_FAST_READ_4B,
    .ui16WriteInstr        = AM_DEVICES_MSPI_IS25WX064_PAGE_PROGRAM_4B,
    .ui8WriteLatency      = 0,
    .bEnWriteLatency      = false,
    .bEmulateDDR          = false,
    .ui16DMATimeLimit     = 0,
    .eDMABoundary         = AM_HAL_MSPI_BOUNDARY_NONE
};

//
//! Serial_CE1 default setting
//
static am_hal_mspi_dev_config_t
MSPI_IS25WX064_Serial_CE1_MSPIConfig =
{
    .ui8TurnAround        = 8,
    .eSpiMode             = AM_HAL_MSPI_SPI_MODE_0,
    .eClockFreq           = AM_HAL_MSPI_CLK_24MHZ,
    .eAddrCfg             = AM_HAL_MSPI_ADDR_4_BYTE,
    .eInstrCfg            = AM_HAL_MSPI_INSTR_1_BYTE,
    .eDeviceConfig        = AM_HAL_MSPI_FLASH_SERIAL_CE0,
    .bSendInstr           = true,
    .bSendAddr            = true,
    .bTurnaround          = true,
    .ui16ReadInstr         = AM_DEVICES_MSPI_IS25WX064_FAST_READ_4B,
    .ui16WriteInstr        = AM_DEVICES_MSPI_IS25WX064_PAGE_PROGRAM_4B,
    .ui8WriteLatency      = 0,
    .bEnWriteLatency      = false,
    .bEmulateDDR          = false,
    .ui16DMATimeLimit     = 0,
    .eDMABoundary         = AM_HAL_MSPI_BOUNDARY_NONE
};

//
//! OCTAL_DDR_CE0 default setting
//
static am_hal_mspi_dev_config_t
DDROctalCE0MSPIConfig =
{
    .ui8TurnAround        = 31,
    .eAddrCfg             = AM_HAL_MSPI_ADDR_4_BYTE,
    .eInstrCfg            = AM_HAL_MSPI_INSTR_2_BYTE,
    .ui16ReadInstr        = AM_DEVICES_MSPI_IS25WX064_OCTA_READ_DTR_CMD,
    .ui16WriteInstr       = AM_DEVICES_MSPI_IS25WX064_OCTA_PAGE_PROG_CMD,
    .eDeviceConfig        = AM_HAL_MSPI_FLASH_OCTAL_DDR_CE0,
    .eSpiMode             = AM_HAL_MSPI_SPI_MODE_0,
    .eClockFreq           = AM_HAL_MSPI_CLK_96MHZ,
    .bSendAddr            = true,
    .bSendInstr           = true,
    .bTurnaround          = true,
    .ui8WriteLatency      = 0,
    .bEnWriteLatency      = true,
    .bEmulateDDR          = true,
    .ui16DMATimeLimit     = 0,
    .eDMABoundary         = AM_HAL_MSPI_BOUNDARY_NONE
};

//
//! OCTAL_DDR_CE1 default setting
//
static am_hal_mspi_dev_config_t
DDROctalCE1MSPIConfig =
{
    .ui8TurnAround        = 31,
    .eAddrCfg             = AM_HAL_MSPI_ADDR_4_BYTE,
    .eInstrCfg            = AM_HAL_MSPI_INSTR_2_BYTE,
    .ui16ReadInstr        = AM_DEVICES_MSPI_IS25WX064_OCTA_READ_DTR_CMD,
    .ui16WriteInstr       = AM_DEVICES_MSPI_IS25WX064_OCTA_PAGE_PROG_CMD,
    .eDeviceConfig        = AM_HAL_MSPI_FLASH_OCTAL_DDR_CE1,
    .eSpiMode             = AM_HAL_MSPI_SPI_MODE_0,
    .eClockFreq           = AM_HAL_MSPI_CLK_96MHZ,
    .bSendAddr            = true,
    .bSendInstr           = true,
    .bTurnaround          = true,
    .ui8WriteLatency      = 0,
    .bEnWriteLatency      = true,
    .bEmulateDDR          = true,
    .ui16DMATimeLimit     = 0,
    .eDMABoundary         = AM_HAL_MSPI_BOUNDARY_NONE
};

//
//! OCTAL_1-8-8_CE0 default setting
//
static am_hal_mspi_dev_config_t
OctalCE0_1_8_8_MSPIConfig =
{
    .ui8TurnAround        = 16,
    .eAddrCfg             = AM_HAL_MSPI_ADDR_4_BYTE,
    .eInstrCfg            = AM_HAL_MSPI_INSTR_1_BYTE,
    .ui16ReadInstr        = AM_DEVICES_MSPI_IS25WX064_OCTAL_1_8_8_FAST_READ_4B,
    .ui16WriteInstr       = AM_DEVICES_MSPI_IS25WX064_OCTAL_1_8_8_FAST_PGM_4B,
    .eDeviceConfig        = AM_HAL_MSPI_FLASH_OCTAL_CE0_1_8_8,
    .eSpiMode             = AM_HAL_MSPI_SPI_MODE_0,
    .eClockFreq           = AM_HAL_MSPI_CLK_96MHZ,
    .bSendAddr            = true,
    .bSendInstr           = true,
    .bTurnaround          = true,
    .ui8WriteLatency      = 0,
    .bEnWriteLatency      = false,
    .bEmulateDDR          = false,
    .ui16DMATimeLimit     = 0,
    .eDMABoundary         = AM_HAL_MSPI_BOUNDARY_NONE
};

//
//! OCTAL_1-8-8_CE1 default setting
//
static am_hal_mspi_dev_config_t
OctalCE1_1_8_8_MSPIConfig =
{
    .ui8TurnAround        = 16,
    .eAddrCfg             = AM_HAL_MSPI_ADDR_4_BYTE,
    .eInstrCfg            = AM_HAL_MSPI_INSTR_1_BYTE,
    .ui16ReadInstr        = AM_DEVICES_MSPI_IS25WX064_OCTAL_1_8_8_FAST_READ_4B,
    .ui16WriteInstr       = AM_DEVICES_MSPI_IS25WX064_OCTAL_1_8_8_FAST_PGM_4B,
    .eDeviceConfig        = AM_HAL_MSPI_FLASH_OCTAL_CE1_1_8_8,
    .eSpiMode             = AM_HAL_MSPI_SPI_MODE_0,
    .eClockFreq           = AM_HAL_MSPI_CLK_96MHZ,
    .bSendAddr            = true,
    .bSendInstr           = true,
    .bTurnaround          = true,
    .ui8WriteLatency      = 0,
    .bEnWriteLatency      = false,
    .bEmulateDDR          = false,
    .ui16DMATimeLimit     = 0,
    .eDMABoundary         = AM_HAL_MSPI_BOUNDARY_NONE
};

//
//! All the MSPI config setting
//
struct
{
    am_hal_mspi_device_e eHalDeviceEnum;
    am_hal_mspi_dev_config_t *psDevConfig;
}g_IS25WX064_DevConfig[] =
{
    {AM_HAL_MSPI_FLASH_SERIAL_CE0,                  &MSPI_IS25WX064_Serial_CE0_MSPIConfig},
    {AM_HAL_MSPI_FLASH_SERIAL_CE1,                  &MSPI_IS25WX064_Serial_CE1_MSPIConfig},
    {AM_HAL_MSPI_FLASH_OCTAL_DDR_CE0,               &DDROctalCE0MSPIConfig},
    {AM_HAL_MSPI_FLASH_OCTAL_DDR_CE1,               &DDROctalCE1MSPIConfig},
    {AM_HAL_MSPI_FLASH_OCTAL_CE0_1_8_8,             &OctalCE0_1_8_8_MSPIConfig},
    {AM_HAL_MSPI_FLASH_OCTAL_CE1_1_8_8,             &OctalCE1_1_8_8_MSPIConfig},
};

//
//! timing default setting
//
am_devices_mspi_is25wx064_timing_config_t sTimingConfigDefault =
{
    .bTxNeg        = 1,
    .bRxNeg        = 0,
    .bRxCap        = 1,
    .ui8Turnaround = 31,
    .ui8TxDQSDelay = 1,
    .ui8RxDQSDelay = 14,
};

am_hal_mspi_timing_scan_t sTimingCfgDefault_OCTAL_DDR =
{
    .bTxNeg        = 1,
    .bRxNeg        = 0,
    .bRxCap        = 1,
    .ui8Turnaround = 31,
    .ui8TxDQSDelay = 1,
    .ui8RxDQSDelay = 14,
};

am_hal_mspi_timing_scan_t sTimingCfgDefault_OCTAL_1_1_8 =
{
    .bTxNeg        = 1,
    .bRxNeg        = 0,
    .bRxCap        = 0,
    .ui8Turnaround = 8,
    .ui8TxDQSDelay = 1,
    .ui8RxDQSDelay = 14,
};

am_hal_mspi_timing_scan_t sTimingCfgDefault_OCTAL_1_8_8 =
{
    .bTxNeg        = 1,
    .bRxNeg        = 0,
    .bRxCap        = 0,
    .ui8Turnaround = 16,
    .ui8TxDQSDelay = 1,
    .ui8RxDQSDelay = 14,
};

//
//! @{
//! timing stored setting
//
static bool bTimingConfigSaved = false;
static am_devices_mspi_is25wx064_timing_config_t sTimingConfigStored;

//! @}

//
//! MSPI RX config
//
am_hal_mspi_rxcfg_t gIS25WX064MspiRxCfg =
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

//
//! MSPI dqs config
//
am_hal_mspi_dqs_t gIS25WX064DqsCfg[] =
{
  {
#ifdef IS25WX064_APOLLO4P_NON_DQS_ENABLE
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
#ifdef IS25WX064_APOLLO4P_NON_DQS_ENABLE
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
#ifdef IS25WX064_APOLLO4P_NON_DQS_ENABLE
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


//
//! MSPI interrupts.
//
static const IRQn_Type mspi_interrupts[] =
{
    MSPI0_IRQn,
    MSPI1_IRQn,
    MSPI2_IRQn,
};


//
// Static declarations.
//
static uint32_t
command_write_serial(void *pHandle,
                     uint8_t ui8Instr,
                     bool bSendAddr,
                     uint32_t ui32Addr,
                     uint32_t *pData,
                     uint32_t ui32NumBytes);
static uint32_t
command_read_serial(void *pHandle,
                    uint8_t ui8Instr,
                    bool bSendAddr,
                    uint32_t ui32Addr,
                    uint32_t *pData,
                    uint32_t ui32NumBytes);
static uint32_t
command_write_octal(void *pHandle,
                    uint16_t ui16Instr,
                    bool bSendAddr,
                    uint32_t ui32Addr,
                    uint32_t *pData,
                    uint32_t ui32NumBytes);

static uint32_t
command_read_octal(void *pMspiHandle,
                   uint16_t ui16Instr,
                   bool bSendAddr,
                   uint32_t ui32Addr,
                   uint32_t *pData,
                   uint32_t ui32NumBytes);

static uint32_t
command_write_combo(void *pHandle,
                    uint16_t ui16Instr,
                    bool bSendAddr,
                    uint32_t ui32Addr,
                    uint32_t *pData,
                    uint32_t ui32NumBytes);

static uint32_t
command_read_combo(void *pHandle,
                   uint16_t ui16Instr,
                   bool bSendAddr,
                   uint32_t ui32Addr,
                   uint32_t *pData,
                   uint32_t ui32NumBytes);

static uint32_t
is25wx064_pre_config(void *pHandle, am_hal_mspi_dev_config_t *dev_config);


static am_devices_mspi_is25wx064_status_t
is25wx064_busy_wait_until(void *pHandle, uint32_t timeout_ms);


static void
is25wx064_power_reset(void);


//*****************************************************************************
//
//! @brief device busy check with timeout
//!
//! @param pHandle
//! @param timeout_ms
//!
//! @return
//
static am_devices_mspi_is25wx064_status_t
is25wx064_busy_wait_until(void *pHandle, uint32_t timeout_ms)
{
    uint32_t ui8Response    = 0;
    uint32_t* pui8Response  = &ui8Response;
    uint32_t ms             = 0;

    while(ms < timeout_ms)
    {
        command_read_combo(pHandle, AM_DEVICES_MSPI_IS25WX064_OCTA_READ_STATUS_REG_CMD, false, 0, pui8Response, 1);
        if ((*pui8Response & 0x01) != 0)
        {
            am_util_delay_ms(1);
            ms++;
        }
        else
        {
            return AM_DEVICES_MSPI_IS25WX064_STATUS_READY;
        }
    }

    return AM_DEVICES_MSPI_IS25WX064_STATUS_BUSY;
}
//*****************************************************************************
//
//! @brief config the device for setting mode
//!
//! @param pHandle
//! @param dev_config
//!
//! @return
//
//*****************************************************************************
static uint32_t
is25wx064_pre_config(void *pHandle, am_hal_mspi_dev_config_t *dev_config)
{
    uint8_t ui8Send;
    uint8_t ui8Response;

    command_write_serial(pHandle, AM_DEVICES_MSPI_IS25WX064_WRITE_ENABLE, false, 0, (uint32_t*)(NULL), 0);
    is25wx064_busy_wait_until(pHandle, 1000);
    command_write_serial(pHandle, AM_DEVICES_MSPI_IS25WX064_ENTER_4BYTE_ADDRESS_MODE, false, 0, (uint32_t*)(NULL), 0);
    is25wx064_busy_wait_until(pHandle, 1000);

    command_read_serial(pHandle, AM_DEVICES_MSPI_IS25WX064_READ_NONVOLATILE_CR, true, AM_DEVICES_MSPI_IS25WX064_CR_REG_ADDR_06, (uint32_t *)&ui8Response, 1);
    am_util_stdio_printf("Serial mode read NCR[6] = 0x%02x\n", ui8Response);
    if (ui8Response != AM_DEVICES_MSPI_IS25WX064_CR_XIP_ENABLE)
    {
        ui8Send = AM_DEVICES_MSPI_IS25WX064_CR_XIP_ENABLE;
        command_write_serial(pHandle, AM_DEVICES_MSPI_IS25WX064_WRITE_NONVOLATILE_CR, true, AM_DEVICES_MSPI_IS25WX064_CR_REG_ADDR_06, (uint32_t *)&ui8Send, 1);
        am_util_delay_ms(1);
        is25wx064_power_reset();
    }

    if (dev_config->eDeviceConfig == AM_HAL_MSPI_FLASH_OCTAL_DDR_CE0 ||
        dev_config->eDeviceConfig == AM_HAL_MSPI_FLASH_OCTAL_DDR_CE1)
    {
        ui8Send = AM_DEVICES_MSPI_IS25WX064_CR_IO_MODE_OCTAL_DDR;
        command_write_serial(pHandle, AM_DEVICES_MSPI_IS25WX064_WRITE_VOLATILE_CR, true, 0x00, (uint32_t *)&ui8Send, 1);
        is25wx064_busy_wait_until(pHandle, 1000);
        command_write_octal(pHandle, AM_DEVICES_MSPI_IS25WX064_OCTA_WRITE_DISABLE_CMD, false, 0, (uint32_t*)(NULL), 0);
    }
    else
    {
        command_write_serial(pHandle, AM_DEVICES_MSPI_IS25WX064_WRITE_DISABLE, false, 0, (uint32_t*)(NULL), 0);
    }

    return AM_DEVICES_MSPI_IS25WX064_STATUS_SUCCESS;
}


//*****************************************************************************
//
//! @brief Device specific de-initialization function.
//!
//! @param pHandle
//!
//! @return
//
//*****************************************************************************
static uint32_t
am_device_deinit_flash(void *pHandle)
{
    uint32_t      ui32Status;
    am_devices_mspi_is25wx064_t *pFlash = (am_devices_mspi_is25wx064_t *)pHandle;

    ui32Status = am_devices_mspi_is25wx064_reset(pHandle, &pFlash->stSetting);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
       return AM_DEVICES_MSPI_IS25WX064_STATUS_ERROR;
    }

    return AM_DEVICES_MSPI_IS25WX064_STATUS_SUCCESS;
}

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
    am_devices_mspi_is25wx064_t *pFlash = (am_devices_mspi_is25wx064_t *)pHandle;
    //
    // Disable MSPI defore re-configuring it
    //
    uint32_t ui32Status = am_hal_mspi_disable(pFlash->pMspiHandle);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        am_util_stdio_printf("Error - Failed to disble mspi.\n");
        return AM_DEVICES_MSPI_IS25WX064_STATUS_ERROR;
    }
    //
    // Re-Configure the MSPI for the requested operation mode.
    //
    ui32Status = am_hal_mspi_device_configure(pFlash->pMspiHandle, pConfig);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        am_util_stdio_printf("Error - Failed to configure mspi.\n");
        return AM_DEVICES_MSPI_IS25WX064_STATUS_ERROR;
    }
    //
    // Re-Enable MSPI
    //
    ui32Status = am_hal_mspi_enable(pFlash->pMspiHandle);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        am_util_stdio_printf("Error - Failed to configure mspi.\n");
        return AM_DEVICES_MSPI_IS25WX064_STATUS_ERROR;
    }

    am_bsp_mspi_pins_enable(pFlash->ui32Module, pConfig->eDeviceConfig);

    return AM_DEVICES_MSPI_IS25WX064_STATUS_SUCCESS;
}

static inline uint32_t
enter_command_mode(void *pHandle)
{
    am_devices_mspi_is25wx064_t *pFlash = (am_devices_mspi_is25wx064_t *)pHandle;
    if (pFlash->stSetting.eDeviceConfig != pFlash->stCmdSetting.eDeviceConfig || pFlash->stSetting.eClockFreq != AM_HAL_MSPI_CLK_24MHZ)
    {
        return am_devices_mspi_device_reconfigure(pFlash, &pFlash->stCmdSetting);
    }
    return AM_DEVICES_MSPI_IS25WX064_STATUS_SUCCESS;
}

static inline uint32_t
exit_command_mode(void *pHandle)
{
    am_devices_mspi_is25wx064_t *pFlash = (am_devices_mspi_is25wx064_t *)pHandle;
    if (pFlash->stSetting.eDeviceConfig != pFlash->stCmdSetting.eDeviceConfig || pFlash->stSetting.eClockFreq != AM_HAL_MSPI_CLK_24MHZ)
    {
        return am_devices_mspi_device_reconfigure(pFlash, &pFlash->stSetting);
    }
    return AM_DEVICES_MSPI_IS25WX064_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief  Generic Command Write function.
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
command_write_serial(void *pHandle, uint8_t ui8Instr, bool bSendAddr,
                     uint32_t ui32Addr, uint32_t *pData,
                     uint32_t ui32NumBytes)
{
    uint32_t ui32Status;
    am_devices_mspi_is25wx064_t *pFlash = (am_devices_mspi_is25wx064_t *)pHandle;
    am_hal_mspi_pio_transfer_t  stMSPIFlashPIOTransaction = {0};
    am_hal_mspi_dqs_t dqsCfg;
    am_hal_mspi_timing_scan_t timingCfg;
    //
    // Create the individual write transaction.
    //
    stMSPIFlashPIOTransaction.ui32NumBytes       = ui32NumBytes;
    stMSPIFlashPIOTransaction.eDirection         = AM_HAL_MSPI_TX;
    stMSPIFlashPIOTransaction.bSendAddr          = bSendAddr;
    stMSPIFlashPIOTransaction.ui32DeviceAddr     = ui32Addr;
    stMSPIFlashPIOTransaction.bSendInstr         = true;
    stMSPIFlashPIOTransaction.ui16DeviceInstr    = ui8Instr;
    stMSPIFlashPIOTransaction.bTurnaround        = false;
    stMSPIFlashPIOTransaction.pui32Buffer        = pData;
    stMSPIFlashPIOTransaction.bDCX               = false;
    stMSPIFlashPIOTransaction.bEnWRLatency       = false;

    enter_command_mode(pHandle);
    //
    // do not use enable fine delay for command read
    //
    dqsCfg.bDQSEnable           = false;
    dqsCfg.bDQSSyncNeg          = false;
    dqsCfg.bEnableFineDelay     = false;
    dqsCfg.ui8RxDQSDelayNeg     = 0;
    dqsCfg.bRxDQSDelayNegEN     = false;
    dqsCfg.bRxDQSDelayHiEN      = false;
    dqsCfg.ui8RxDQSDelay        = 16;
    dqsCfg.ui8TxDQSDelay        = 0;
    am_hal_mspi_control(pFlash->pMspiHandle, AM_HAL_MSPI_REQ_DQS, &dqsCfg);

    timingCfg.bRxNeg            = false;
    timingCfg.bRxCap            = false;
    timingCfg.ui8TxDQSDelay     = 0;
    timingCfg.ui8RxDQSDelay     = 16;
    if (ui8Instr == AM_DEVICES_MSPI_IS25WX064_WRITE_VOLATILE_CR ||
        ui8Instr == AM_DEVICES_MSPI_IS25WX064_WRITE_NONVOLATILE_CR)
    {
        timingCfg.ui8Turnaround = 0;
    }
    else
    {
        timingCfg.ui8Turnaround = 8;
    }
    am_hal_mspi_control(pFlash->pMspiHandle, AM_HAL_MSPI_REQ_TIMING_SCAN, &timingCfg);
    //
    // Execute the transction over MSPI.
    //
    ui32Status = am_hal_mspi_blocking_transfer(pFlash->pMspiHandle, &stMSPIFlashPIOTransaction,
                                         AM_DEVICES_MSPI_IS25WX064_TIMEOUT);

    exit_command_mode(pHandle);
    //
    // restore enable fine delay timing settings
    //
    if ( bTimingConfigSaved == true )
    {
        am_devices_mspi_is25wx064_apply_ddr_timing(pFlash, &sTimingConfigStored);
    }

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
//! @return uint32_t
//
//*****************************************************************************
static uint32_t
command_read_serial(void *pHandle, uint8_t ui8Instr, bool bSendAddr,
                    uint32_t ui32Addr, uint32_t *pData,
                    uint32_t ui32NumBytes)
{
    uint32_t ui32Status;
    am_devices_mspi_is25wx064_t *pFlash = (am_devices_mspi_is25wx064_t *)pHandle;
    am_hal_mspi_pio_transfer_t      stMSPIFlashPIOTransaction = {0};

    am_hal_mspi_dqs_t dqsCfg;
    am_hal_mspi_timing_scan_t timingCfg;
    //
    // Create the individual write transaction.
    //
    stMSPIFlashPIOTransaction.eDirection         = AM_HAL_MSPI_RX;
    stMSPIFlashPIOTransaction.bSendAddr          = bSendAddr;
    stMSPIFlashPIOTransaction.ui32DeviceAddr     = ui32Addr;
    stMSPIFlashPIOTransaction.bSendInstr         = true;
    stMSPIFlashPIOTransaction.ui16DeviceInstr    = ui8Instr;

    enter_command_mode(pHandle);

    if ( ui8Instr == AM_DEVICES_MSPI_IS25WX064_READ_STATUS )
    {
        //
        // Read status/control register command uses 1 byte address
        //
        am_hal_mspi_instr_addr_t sInstAddrCfg;
        sInstAddrCfg.eAddrCfg = AM_HAL_MSPI_ADDR_4_BYTE;
        sInstAddrCfg.eInstrCfg = pFlash->stSetting.eInstrCfg;   // keep instruction setting the same
        am_hal_mspi_control(pFlash->pMspiHandle, AM_HAL_MSPI_REQ_SET_INSTR_ADDR_LEN, &sInstAddrCfg);
    }


    stMSPIFlashPIOTransaction.bTurnaround    = true;
    //
    // do not use enable fine delay for command read
    //
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
    if (ui8Instr == AM_DEVICES_MSPI_IS25WX064_READ_VOLATILE_CR ||
        ui8Instr == AM_DEVICES_MSPI_IS25WX064_READ_NONVOLATILE_CR)
    {
        timingCfg.ui8Turnaround = 8;
    }
    else
    {
        timingCfg.ui8Turnaround = 0;
    }
    am_hal_mspi_control(pFlash->pMspiHandle, AM_HAL_MSPI_REQ_TIMING_SCAN, &timingCfg);

    stMSPIFlashPIOTransaction.ui32NumBytes       = ui32NumBytes;
    stMSPIFlashPIOTransaction.pui32Buffer        = pData;

    stMSPIFlashPIOTransaction.bDCX               = false;
    stMSPIFlashPIOTransaction.bEnWRLatency       = false;
    //
    // Execute the transction over MSPI.
    //
    ui32Status = am_hal_mspi_blocking_transfer(pFlash->pMspiHandle, &stMSPIFlashPIOTransaction,
                                         AM_DEVICES_MSPI_IS25WX064_TIMEOUT);

    exit_command_mode(pHandle);

    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
      return ui32Status;
    }

    return ui32Status;
}

//*****************************************************************************
//
//! @brief Octal Command Write function.
//!
//! @param pHandle
//! @param ui16Instr
//! @param bSendAddr
//! @param ui32Addr
//! @param pData
//! @param ui32NumBytes
//!
//! @return uint32_t
//
//*****************************************************************************
static uint32_t
command_write_octal(void *pHandle, uint16_t ui16Instr, bool bSendAddr,
                    uint32_t ui32Addr, uint32_t *pData,
                    uint32_t ui32NumBytes)
{
    uint32_t ui32Status;
    am_devices_mspi_is25wx064_t *pFlash = (am_devices_mspi_is25wx064_t *)pHandle;
    am_hal_mspi_pio_transfer_t  stMSPIFlashPIOTransaction = {0};
    am_hal_mspi_dqs_t dqsCfg;
    am_hal_mspi_timing_scan_t timingCfg;
    //
    // Create the individual write transaction.
    //
    stMSPIFlashPIOTransaction.ui32NumBytes       = ui32NumBytes;
    stMSPIFlashPIOTransaction.eDirection         = AM_HAL_MSPI_TX;
    stMSPIFlashPIOTransaction.bSendAddr          = bSendAddr;
    stMSPIFlashPIOTransaction.ui32DeviceAddr     = ui32Addr;
    stMSPIFlashPIOTransaction.bSendInstr         = true;
    stMSPIFlashPIOTransaction.ui16DeviceInstr    = ui16Instr;
    stMSPIFlashPIOTransaction.bTurnaround        = false;
    stMSPIFlashPIOTransaction.pui32Buffer        = pData;
    stMSPIFlashPIOTransaction.bDCX               = false;
    stMSPIFlashPIOTransaction.bEnWRLatency       = false;
    //
    // do not use enable fine delay for command read
    //
    dqsCfg.bDQSEnable           = false;
    dqsCfg.bDQSSyncNeg          = false;
    dqsCfg.bEnableFineDelay     = false;
    dqsCfg.ui8RxDQSDelayNeg     = 0;
    dqsCfg.bRxDQSDelayNegEN     = false;
    dqsCfg.bRxDQSDelayHiEN      = false;
    dqsCfg.ui8RxDQSDelay        = 16;
    dqsCfg.ui8TxDQSDelay        = 0;
    am_hal_mspi_control(pFlash->pMspiHandle, AM_HAL_MSPI_REQ_DQS, &dqsCfg);
    if (AM_HAL_MSPI_CLK_96MHZ == pFlash->stSetting.eClockFreq)
    {
        timingCfg.bTxNeg            = true;
        timingCfg.bRxNeg            = true;
        timingCfg.bRxCap            = false;
        timingCfg.ui8TxDQSDelay     = 0;
        timingCfg.ui8RxDQSDelay     = 16;
        timingCfg.ui8Turnaround     = 0;
    }
    else
    {
        timingCfg.bTxNeg            = false;
        timingCfg.bRxNeg            = false;
        timingCfg.bRxCap            = false;
        timingCfg.ui8TxDQSDelay     = 0;
        timingCfg.ui8RxDQSDelay     = 16;
        timingCfg.ui8Turnaround     = 0;
    }
    am_hal_mspi_control(pFlash->pMspiHandle, AM_HAL_MSPI_REQ_TIMING_SCAN, &timingCfg);
    //
    // Execute the transction over MSPI.
    //
    ui32Status = am_hal_mspi_blocking_transfer(pFlash->pMspiHandle, &stMSPIFlashPIOTransaction,
                                         AM_DEVICES_MSPI_IS25WX064_TIMEOUT);

    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
      return ui32Status;
    }

    return ui32Status;
}

//*****************************************************************************
//
//! @brief  Octal Command Read function.
//!
//! @param pHandle
//! @param ui16Instr
//! @param bSendAddr
//! @param ui32Addr
//! @param pData
//! @param ui32NumBytes
//!
//! @return uint32_t
//
//*****************************************************************************
static uint32_t
command_read_octal(void *pHandle, uint16_t ui16Instr, bool bSendAddr,
                   uint32_t ui32Addr, uint32_t *pData,
                   uint32_t ui32NumBytes)
{
    uint32_t ui32Status;
    am_devices_mspi_is25wx064_t *pFlash = (am_devices_mspi_is25wx064_t *)pHandle;
    am_hal_mspi_pio_transfer_t      stMSPIFlashPIOTransaction = {0};

    am_hal_mspi_dqs_t dqsCfg;
    am_hal_mspi_timing_scan_t timingCfg;
    //
    // Create the individual write transaction.
    //
    stMSPIFlashPIOTransaction.eDirection         = AM_HAL_MSPI_RX;
    stMSPIFlashPIOTransaction.bSendAddr          = bSendAddr;
    stMSPIFlashPIOTransaction.ui32DeviceAddr     = ui32Addr;
    stMSPIFlashPIOTransaction.bSendInstr         = true;
    stMSPIFlashPIOTransaction.ui16DeviceInstr    = ui16Instr;
    stMSPIFlashPIOTransaction.bTurnaround        = true;
    //
    // do not use enable fine delay for command read
    //
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

    if (ui16Instr == AM_DEVICES_MSPI_IS25WX064_OCTA_READ_ID_CMD)
    {
        if (AM_HAL_MSPI_CLK_96MHZ == pFlash->stSetting.eClockFreq)
        {
            timingCfg.ui8Turnaround = 16;
        }
        else
        {
            timingCfg.ui8Turnaround = 15;
        }
    }
    else
    {
        timingCfg.ui8Turnaround = 16;
    }

    am_hal_mspi_control(pFlash->pMspiHandle, AM_HAL_MSPI_REQ_TIMING_SCAN, &timingCfg);

    stMSPIFlashPIOTransaction.ui32NumBytes       = ui32NumBytes;
    stMSPIFlashPIOTransaction.pui32Buffer        = pData;
    stMSPIFlashPIOTransaction.bDCX               = false;
    stMSPIFlashPIOTransaction.bEnWRLatency       = false;
    //
    // Execute the transction over MSPI.
    //
    ui32Status = am_hal_mspi_blocking_transfer(pFlash->pMspiHandle, &stMSPIFlashPIOTransaction,
                                       AM_DEVICES_MSPI_IS25WX064_TIMEOUT);

    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return ui32Status;
    }

    return ui32Status;

}

//*****************************************************************************
//
//! @brief Combo Command Write function.
//!
//! @param pHandle
//! @param ui16Instr
//! @param bSendAddr
//! @param ui32Addr
//! @param pData
//! @param ui32NumBytes
//!
//! @return uint32_t
//
//*****************************************************************************
static uint32_t
command_write_combo(void *pHandle,
                    uint16_t ui16Instr,
                    bool bSendAddr,
                    uint32_t ui32Addr,
                    uint32_t *pData,
                    uint32_t ui32NumBytes)
{
    uint32_t ui32Status;
    am_devices_mspi_is25wx064_t *pFlash = (am_devices_mspi_is25wx064_t *)pHandle;

    if (pFlash->stSetting.eDeviceConfig == AM_HAL_MSPI_FLASH_SERIAL_CE0 ||
        pFlash->stSetting.eDeviceConfig == AM_HAL_MSPI_FLASH_SERIAL_CE1 ||
        pFlash->stSetting.eDeviceConfig == AM_HAL_MSPI_FLASH_OCTAL_CE0_1_1_8 ||
        pFlash->stSetting.eDeviceConfig == AM_HAL_MSPI_FLASH_OCTAL_CE1_1_1_8 ||
        pFlash->stSetting.eDeviceConfig == AM_HAL_MSPI_FLASH_OCTAL_CE0_1_8_8 ||
        pFlash->stSetting.eDeviceConfig == AM_HAL_MSPI_FLASH_OCTAL_CE1_1_8_8)
    {
        ui32Status = command_write_serial(pHandle, (uint8_t)ui16Instr, bSendAddr, ui32Addr, pData, ui32NumBytes);
    }
    else if (pFlash->stSetting.eDeviceConfig == AM_HAL_MSPI_FLASH_OCTAL_DDR_CE0 ||
        pFlash->stSetting.eDeviceConfig == AM_HAL_MSPI_FLASH_OCTAL_DDR_CE1)
    {
        ui32Status = command_write_octal(pHandle, ui16Instr, bSendAddr, ui32Addr, pData, ui32NumBytes);
    }
    else
    {
        ui32Status = AM_HAL_STATUS_FAIL;
    }

    return ui32Status;
}

//*****************************************************************************
//
//! @brief
//!
//! @param pHandle
//! @param ui16Instr
//! @param bSendAddr
//! @param ui32Addr
//! @param pData
//! @param ui32NumBytes
//!
//! @return uint32_t
//
//*****************************************************************************
static uint32_t
command_read_combo(void *pHandle,
                   uint16_t ui16Instr,
                   bool bSendAddr,
                   uint32_t ui32Addr,
                   uint32_t *pData,
                   uint32_t ui32NumBytes)
{
    uint32_t ui32Status;
    am_devices_mspi_is25wx064_t *pFlash = (am_devices_mspi_is25wx064_t *)pHandle;

    if (pFlash->stSetting.eDeviceConfig == AM_HAL_MSPI_FLASH_SERIAL_CE0 ||
        pFlash->stSetting.eDeviceConfig == AM_HAL_MSPI_FLASH_SERIAL_CE1 ||
        pFlash->stSetting.eDeviceConfig == AM_HAL_MSPI_FLASH_OCTAL_CE0_1_1_8 ||
        pFlash->stSetting.eDeviceConfig == AM_HAL_MSPI_FLASH_OCTAL_CE1_1_1_8 ||
        pFlash->stSetting.eDeviceConfig == AM_HAL_MSPI_FLASH_OCTAL_CE0_1_8_8 ||
        pFlash->stSetting.eDeviceConfig == AM_HAL_MSPI_FLASH_OCTAL_CE1_1_8_8)
    {
        ui32Status = command_read_serial(pHandle, (uint8_t)ui16Instr, bSendAddr, ui32Addr, pData, ui32NumBytes);
    }
    else if (pFlash->stSetting.eDeviceConfig == AM_HAL_MSPI_FLASH_OCTAL_DDR_CE0 ||
        pFlash->stSetting.eDeviceConfig == AM_HAL_MSPI_FLASH_OCTAL_DDR_CE1)
    {
        ui32Status = command_read_octal(pHandle, ui16Instr, bSendAddr, ui32Addr, pData, ui32NumBytes);
    }
    else
    {
        ui32Status = AM_HAL_STATUS_FAIL;
    }

    return ui32Status;

}

//*****************************************************************************
//
//! @brief Callback function.
//!
//! @param pCallbackCtxt
//! @param status
//
//*****************************************************************************
static void
pfnMSPI_IS25WX064_Callback(void *pCallbackCtxt, uint32_t status)
{
    //
    // Set the DMA complete flag.
    //
    *(volatile uint32_t *)pCallbackCtxt = status;
}

//*****************************************************************************
//
//! Device hardware reset
//
//*****************************************************************************
static void
is25wx064_power_reset(void)
{
#ifdef AM_BSP_GPIO_NOR_RST
    am_hal_gpio_pinconfig(AM_BSP_GPIO_NOR_RST, am_hal_gpio_pincfg_output);
    am_hal_gpio_output_clear(AM_BSP_GPIO_NOR_RST);
    am_util_delay_ms(10);
    am_hal_gpio_output_set(AM_BSP_GPIO_NOR_RST);
#endif
}

//*****************************************************************************
//
// Initialize the mspi_flash driver.
//
//*****************************************************************************
uint32_t
am_devices_mspi_is25wx064_init(uint32_t ui32Module,
                               am_devices_mspi_is25wx064_config_t *psMSPISettings,
                               void **ppHandle,
                               void **ppMspiHandle)
{
    uint32_t      ui32Status;
    am_hal_mspi_dev_config_t *psConfig = g_IS25WX064_DevConfig[0].psDevConfig;
    void                     *pMspiHandle;
    uint32_t      ui32Index = 0;

    is25wx064_power_reset();

    if ((ui32Module > AM_REG_MSPI_NUM_MODULES) || (psMSPISettings == NULL))
    {
        return AM_DEVICES_MSPI_IS25WX064_STATUS_ERROR;
    }
    //
    // Allocate a vacant device handle
    //
    for ( ui32Index = 0; ui32Index < AM_DEVICES_MSPI_IS25WX064_MAX_DEVICE_NUM; ui32Index++ )
    {
        if ( gAmIS25WX064[ui32Index].bOccupied == false )
        {
            break;
        }
    }
    if ( ui32Index == AM_DEVICES_MSPI_IS25WX064_MAX_DEVICE_NUM )
    {
        return AM_DEVICES_MSPI_IS25WX064_STATUS_ERROR;
    }

    for ( uint32_t i = 0; i < (sizeof(g_IS25WX064_DevConfig) / sizeof(g_IS25WX064_DevConfig[0])); i++ )
    {
        if ( psMSPISettings->eDeviceConfig == g_IS25WX064_DevConfig[i].eHalDeviceEnum )
        {
            psConfig = g_IS25WX064_DevConfig[i].psDevConfig;
#if !defined(AM_PART_APOLLO4) && !defined(AM_PART_APOLLO4B) && !defined(AM_PART_APOLLO4P) && !defined(AM_PART_APOLLO4L)
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
        case AM_HAL_MSPI_FLASH_OCTAL_CE0_1_1_8:
        case AM_HAL_MSPI_FLASH_OCTAL_CE0_1_8_8:
        case AM_HAL_MSPI_FLASH_OCTAL_DDR_CE0:
            gAmIS25WX064[ui32Index].stSetting = MSPI_IS25WX064_Serial_CE0_MSPIConfig;
            gAmIS25WX064[ui32Index].stCmdSetting = MSPI_IS25WX064_Serial_CE0_MSPIConfig;
            if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_initialize(ui32Module, &pMspiHandle))
            {
                am_util_debug_printf("Error - Failed to initialize MSPI.\n");
                return AM_DEVICES_MSPI_IS25WX064_STATUS_ERROR;
            }
            if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_power_control(pMspiHandle, AM_HAL_SYSCTRL_WAKE, false))
            {
                am_util_debug_printf("Error - Failed to power on MSPI.\n");
                return AM_DEVICES_MSPI_IS25WX064_STATUS_ERROR;
            }
            gAmIS25WX064[ui32Index].stCmdSetting.eClockFreq = AM_HAL_MSPI_CLK_24MHZ;
            {
              am_hal_mspi_config_t    mspiCfg = gMspiCfg;
              mspiCfg.ui32TCBSize = psMSPISettings->ui32NBTxnBufLength;
              mspiCfg.pTCB = psMSPISettings->pNBTxnBuf;
              if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_configure(pMspiHandle, &mspiCfg))
              {
                am_util_debug_printf("Error - Failed to configure MSPI device.\n");
                return AM_DEVICES_MSPI_IS25WX064_STATUS_ERROR;
              }
            }
            if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_device_configure(pMspiHandle, &gAmIS25WX064[ui32Index].stCmdSetting))
            {
                am_util_debug_printf("Error - Failed to configure MSPI.\n");
                return AM_DEVICES_MSPI_IS25WX064_STATUS_ERROR;
            }


            if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_enable(pMspiHandle))
            {
                am_util_debug_printf("Error - Failed to enable MSPI.\n");
                return AM_DEVICES_MSPI_IS25WX064_STATUS_ERROR;
            }

            am_bsp_mspi_pins_enable(ui32Module, MSPI_IS25WX064_Serial_CE0_MSPIConfig.eDeviceConfig);
            break;
        case AM_HAL_MSPI_FLASH_SERIAL_CE1:
        case AM_HAL_MSPI_FLASH_OCTAL_CE1_1_1_8:
        case AM_HAL_MSPI_FLASH_OCTAL_CE1_1_8_8:
        case AM_HAL_MSPI_FLASH_OCTAL_DDR_CE1:
            gAmIS25WX064[ui32Index].stSetting = MSPI_IS25WX064_Serial_CE1_MSPIConfig;
            gAmIS25WX064[ui32Index].stCmdSetting = MSPI_IS25WX064_Serial_CE1_MSPIConfig;
            if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_initialize(ui32Module, &pMspiHandle))
            {
                am_util_debug_printf("Error - Failed to initialize MSPI.\n");
                return AM_DEVICES_MSPI_IS25WX064_STATUS_ERROR;
            }

            if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_power_control(pMspiHandle, AM_HAL_SYSCTRL_WAKE, false))
            {
                am_util_debug_printf("Error - Failed to power on MSPI.\n");
                return AM_DEVICES_MSPI_IS25WX064_STATUS_ERROR;
            }
            gAmIS25WX064[ui32Index].stCmdSetting.eClockFreq = AM_HAL_MSPI_CLK_24MHZ;
            {
                am_hal_mspi_config_t    mspiCfg = gMspiCfg;
                mspiCfg.ui32TCBSize = psMSPISettings->ui32NBTxnBufLength;
                mspiCfg.pTCB = psMSPISettings->pNBTxnBuf;
                if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_configure(pMspiHandle, &mspiCfg))
                {
                    am_util_debug_printf("Error - Failed to configure MSPI device.\n");
                    return AM_DEVICES_MSPI_IS25WX064_STATUS_ERROR;
                }
            }
            if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_device_configure(pMspiHandle, &gAmIS25WX064[ui32Index].stCmdSetting))
            {
                am_util_debug_printf("Error - Failed to configure MSPI.\n");
                return AM_DEVICES_MSPI_IS25WX064_STATUS_ERROR;
            }
            if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_enable(pMspiHandle))
            {
                am_util_debug_printf("Error - Failed to enable MSPI.\n");
                return AM_DEVICES_MSPI_IS25WX064_STATUS_ERROR;
            }

            am_bsp_mspi_pins_enable(ui32Module, MSPI_IS25WX064_Serial_CE1_MSPIConfig.eDeviceConfig);

            break;
        default:
            return AM_DEVICES_MSPI_IS25WX064_STATUS_ERROR;
            //break;
    }

    gAmIS25WX064[ui32Index].pMspiHandle = pMspiHandle;
    gAmIS25WX064[ui32Index].ui32Module = ui32Module;

    if (AM_HAL_STATUS_SUCCESS != am_devices_mspi_is25wx064_reset((void*)&gAmIS25WX064[ui32Index], &gAmIS25WX064[ui32Index].stSetting))
    {
        return AM_DEVICES_MSPI_IS25WX064_STATUS_ERROR;
    }


    am_devices_mspi_is25wx064_id((void *)&gAmIS25WX064[ui32Index]);

    is25wx064_pre_config((void *)&gAmIS25WX064[ui32Index], psConfig);

    gAmIS25WX064[ui32Index].stSetting = *psConfig;
    gAmIS25WX064[ui32Index].stSetting.eClockFreq = psMSPISettings->eClockFreq;
    am_devices_mspi_is25wx064_enable_xip((void*)&gAmIS25WX064[ui32Index]);

    //
    // Disable MSPI defore re-configuring it
    //
    ui32Status = am_hal_mspi_disable(pMspiHandle);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_IS25WX064_STATUS_ERROR;
    }

    if (psConfig->eDeviceConfig == AM_HAL_MSPI_FLASH_OCTAL_DDR_CE0 ||
        psConfig->eDeviceConfig == AM_HAL_MSPI_FLASH_OCTAL_DDR_CE1)
    {
        ui32Status = am_hal_mspi_control(pMspiHandle, AM_HAL_MSPI_REQ_DDR_EN, NULL);
        if (AM_HAL_STATUS_SUCCESS != ui32Status)
        {
          return AM_DEVICES_MSPI_IS25WX064_STATUS_ERROR;
        }
    }

    //
    // Re-Configure the MSPI for the requested operation mode.
    //
    ui32Status = am_hal_mspi_device_configure(pMspiHandle, &gAmIS25WX064[ui32Index].stSetting);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_IS25WX064_STATUS_ERROR;
    }

#if defined(AM_PART_APOLLO4P) || defined(AM_PART_APOLLO4L)
    am_hal_mspi_dqs_t dqsCfg = gIS25WX064DqsCfg[ui32Module];
    ui32Status = am_hal_mspi_control(pMspiHandle, AM_HAL_MSPI_REQ_DQS, &dqsCfg);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
      return AM_DEVICES_MSPI_IS25WX064_STATUS_ERROR;
    }

    am_hal_mspi_rxcfg_t RxCfg = gIS25WX064MspiRxCfg;
    ui32Status = am_hal_mspi_control(pMspiHandle, AM_HAL_MSPI_REQ_RXCFG, &RxCfg);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
      return AM_DEVICES_MSPI_IS25WX064_STATUS_ERROR;
    }
#endif

    //
    // Re-Enable MSPI
    //
    ui32Status = am_hal_mspi_enable(pMspiHandle);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_IS25WX064_STATUS_ERROR;
    }

    //
    // Configure the MSPI pins.
    //
    am_bsp_mspi_pins_enable(ui32Module, psConfig->eDeviceConfig);

    //
    // Enable MSPI interrupts.
    //
    ui32Status = am_hal_mspi_interrupt_clear(pMspiHandle, AM_HAL_MSPI_INT_CQUPD | AM_HAL_MSPI_INT_ERR );
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_IS25WX064_STATUS_ERROR;
    }

    ui32Status = am_hal_mspi_interrupt_enable(pMspiHandle, AM_HAL_MSPI_INT_CQUPD | AM_HAL_MSPI_INT_ERR );
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_IS25WX064_STATUS_ERROR;
    }

    //
    // Return the handle.
    //
    gAmIS25WX064[ui32Index].bOccupied = true;
    *ppMspiHandle = pMspiHandle;
    *ppHandle = (void *)&gAmIS25WX064[ui32Index];


    //
    // Return the status.
    //
    return AM_DEVICES_MSPI_IS25WX064_STATUS_SUCCESS;
}

//*****************************************************************************
//
// DeInitialize the mspi_flash driver.
//
//*****************************************************************************
uint32_t
am_devices_mspi_is25wx064_deinit(void *pHandle)
{
    uint32_t      ui32Status;
    am_devices_mspi_is25wx064_t *pFlash = (am_devices_mspi_is25wx064_t *)pHandle;

    //
    // Device specific MSPI Flash de-initialization.
    //
    ui32Status = am_device_deinit_flash(pHandle);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_IS25WX064_STATUS_ERROR;
    }

    if (AM_HAL_STATUS_SUCCESS != am_devices_mspi_is25wx064_reset(pHandle, &pFlash->stSetting))
    {
        return AM_DEVICES_MSPI_IS25WX064_STATUS_ERROR;
    }

    //
    // Disable and clear the interrupts to start with.
    //
    ui32Status = am_hal_mspi_interrupt_disable(pFlash->pMspiHandle, 0xFFFFFFFF);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
      return AM_DEVICES_MSPI_IS25WX064_STATUS_ERROR;
    }
    ui32Status = am_hal_mspi_interrupt_clear(pFlash->pMspiHandle, 0xFFFFFFFF);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
      return AM_DEVICES_MSPI_IS25WX064_STATUS_ERROR;
    }

    //
    // Disable the MSPI instance.
    //
    ui32Status = am_hal_mspi_disable(pFlash->pMspiHandle);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_IS25WX064_STATUS_ERROR;
    }

    if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_power_control(pFlash->pMspiHandle, AM_HAL_SYSCTRL_DEEPSLEEP, false))
    {
        am_util_debug_printf("Error - Failed to power on MSPI.\n");
        return AM_DEVICES_MSPI_IS25WX064_STATUS_ERROR;
    }

    //
    // Deinitialize the MSPI instance.
    //
    ui32Status = am_hal_mspi_deinitialize(pFlash->pMspiHandle);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_IS25WX064_STATUS_ERROR;
    }

    //
    // Free this device handle
    //
    pFlash->bOccupied = false;

    //
    // Clear the Flash Caching.
    //
#if !defined(AM_PART_APOLLO4) && !defined(AM_PART_APOLLO4B) && !defined(AM_PART_APOLLO4P) && !defined(AM_PART_APOLLO4L)
#if AM_CMSIS_REGS
    CACHECTRL->CACHECFG = 0;
#else // AM_CMSIS_REGS
    AM_REG(CACHECTRL, CACHECFG) = 0;
#endif // AM_CMSIS_REGS
#endif // !AM_PART_APOLLO4
    //
    // Return the status.
    //
    return AM_DEVICES_MSPI_IS25WX064_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Reset the external flash.
//
//*****************************************************************************
uint32_t
am_devices_mspi_is25wx064_reset(void *pHandle, am_hal_mspi_dev_config_t *pDevCconfig)
{
    uint32_t      ui32Status;
    uint32_t      ui32PIOBuffer[32] = {0};

//    if (pDevCconfig->eDeviceConfig == AM_HAL_MSPI_FLASH_SERIAL_CE0 ||
//        pDevCconfig->eDeviceConfig == AM_HAL_MSPI_FLASH_SERIAL_CE1)
//    {
        ui32Status = command_write_serial(pHandle, AM_DEVICES_MSPI_IS25WX064_WRITE_ENABLE, false, 0, ui32PIOBuffer, 0);
        if (AM_HAL_STATUS_SUCCESS != ui32Status)
        {
           return AM_DEVICES_MSPI_IS25WX064_STATUS_ERROR;
        }

        ui32Status = command_write_serial(pHandle, AM_DEVICES_MSPI_IS25WX064_RESET_ENABLE, false, 0, ui32PIOBuffer, 0);
        if (AM_HAL_STATUS_SUCCESS != ui32Status)
        {
            return AM_DEVICES_MSPI_IS25WX064_STATUS_ERROR;
        }

        ui32Status = command_write_serial(pHandle, AM_DEVICES_MSPI_IS25WX064_WRITE_DISABLE, false, 0, ui32PIOBuffer, 0);
        if (AM_HAL_STATUS_SUCCESS != ui32Status)
        {
           return AM_DEVICES_MSPI_IS25WX064_STATUS_ERROR;
        }
//    }else if (pDevCconfig->eDeviceConfig == AM_HAL_MSPI_FLASH_OCTAL_DDR_CE0 ||
//        pDevCconfig->eDeviceConfig == AM_HAL_MSPI_FLASH_OCTAL_DDR_CE1)
//    {
        ui32Status = command_write_octal(pHandle, AM_DEVICES_MSPI_IS25WX064_OCTA_WRITE_ENABLE_CMD, false, 0, ui32PIOBuffer, 0);
        if (AM_HAL_STATUS_SUCCESS != ui32Status)
        {
           return AM_DEVICES_MSPI_IS25WX064_STATUS_ERROR;
        }

        ui32Status = command_write_octal(pHandle, AM_DEVICES_MSPI_IS25WX064_OCTA_RESET_ENABLE_CMD, false, 0, ui32PIOBuffer, 0);
        if (AM_HAL_STATUS_SUCCESS != ui32Status)
        {
            return AM_DEVICES_MSPI_IS25WX064_STATUS_ERROR;
        }

        ui32Status = command_write_octal(pHandle, AM_DEVICES_MSPI_IS25WX064_OCTA_WRITE_DISABLE_CMD, false, 0, ui32PIOBuffer, 0);
        if (AM_HAL_STATUS_SUCCESS != ui32Status)
        {
           return AM_DEVICES_MSPI_IS25WX064_STATUS_ERROR;
        }
//    }

    return AM_DEVICES_MSPI_IS25WX064_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Reads the ID of the external flash and returns the value.
//
//*****************************************************************************
uint32_t
am_devices_mspi_is25wx064_id(void *pHandle)
{
    uint32_t      ui32Status;
    uint32_t      ui32DeviceID;
    am_devices_mspi_is25wx064_t *pFlash = (am_devices_mspi_is25wx064_t *)pHandle;

    //
    // Send the command sequence to read the Device ID and return status.
    //
    uint8_t       ui8Response[20];

    if (pFlash->stSetting.eDeviceConfig == AM_HAL_MSPI_FLASH_SERIAL_CE0 ||
        pFlash->stSetting.eDeviceConfig == AM_HAL_MSPI_FLASH_SERIAL_CE1 ||
        pFlash->stSetting.eDeviceConfig == AM_HAL_MSPI_FLASH_OCTAL_CE0_1_1_8 ||
        pFlash->stSetting.eDeviceConfig == AM_HAL_MSPI_FLASH_OCTAL_CE1_1_1_8 ||
        pFlash->stSetting.eDeviceConfig == AM_HAL_MSPI_FLASH_OCTAL_CE0_1_8_8 ||
        pFlash->stSetting.eDeviceConfig == AM_HAL_MSPI_FLASH_OCTAL_CE1_1_8_8)
    {
        ui32Status = command_read_serial(pHandle, AM_DEVICES_MSPI_IS25WX064_READ_ID, false, 0, (uint32_t *)&ui8Response, 11);
    }
    else if (pFlash->stSetting.eDeviceConfig == AM_HAL_MSPI_FLASH_OCTAL_DDR_CE0 ||
             pFlash->stSetting.eDeviceConfig == AM_HAL_MSPI_FLASH_OCTAL_DDR_CE1)
    {
        ui32Status = command_read_octal(pHandle, AM_DEVICES_MSPI_IS25WX064_OCTA_READ_ID_CMD, false, 0, (uint32_t *)&ui8Response, 11);
    }

    ui32DeviceID = (ui8Response[0] << 16) | (ui8Response[1] << 8) | ui8Response[2];
    am_util_stdio_printf("Flash ID = 0x%06x\n", ui32DeviceID);

    if ( ((ui32DeviceID & AM_DEVICES_MSPI_IS25WX064_ID_MASK) == AM_DEVICES_MSPI_IS25WX064_ID) &&
       (AM_HAL_STATUS_SUCCESS == ui32Status) )
    {
        return AM_DEVICES_MSPI_IS25WX064_STATUS_SUCCESS;
    }
    else
    {
        return AM_DEVICES_MSPI_IS25WX064_STATUS_ERROR;
    }
}

//*****************************************************************************
//
// Reads the contents of the external flash into a buffer.
//
//*****************************************************************************
uint32_t
am_devices_mspi_is25wx064_read_adv(void *pHandle,
                                   uint8_t *pui8RxBuffer,
                                   uint32_t ui32ReadAddress,
                                   uint32_t ui32NumBytes,
                                   uint32_t ui32PauseCondition,
                                   uint32_t ui32StatusSetClr,
                                   am_hal_mspi_callback_t pfnCallback,
                                   void *pCallbackCtxt)
{
    am_hal_mspi_dma_transfer_t    Transaction;
    uint32_t                      ui32Status;
    am_devices_mspi_is25wx064_t *pFlash = (am_devices_mspi_is25wx064_t *)pHandle;
    //
    // Set the DMA priority
    //
    Transaction.ui8Priority = 1;
    //
    // Set the transfer direction to RX (Read)
    //
    Transaction.eDirection = AM_HAL_MSPI_RX;
    //
    // Set the transfer count in bytes.
    //
    Transaction.ui32TransferCount = ui32NumBytes;
    //
    // Set the address to read data from.
    //
    Transaction.ui32DeviceAddress = ui32ReadAddress;
    //
    // Set the target SRAM buffer address.
    //
    Transaction.ui32SRAMAddress = (uint32_t)pui8RxBuffer;
    //
    // Clear the CQ stimulus.
    //
    Transaction.ui32PauseCondition = ui32PauseCondition;
    //
    // Clear the post-processing
    //
    Transaction.ui32StatusSetClr = ui32StatusSetClr;

#if defined(AM_PART_APOLLO4)
    Transaction.eDeviceNum         = AM_HAL_MSPI_DEVICE0;
#endif
    //
    // Check the transaction status.
    //
    ui32Status = am_hal_mspi_nonblocking_transfer(pFlash->pMspiHandle, &Transaction,
                                                  AM_HAL_MSPI_TRANS_DMA, pfnCallback, pCallbackCtxt);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_IS25WX064_STATUS_ERROR;
    }

    //
    // Return the status.
    //
    return AM_DEVICES_MSPI_IS25WX064_STATUS_SUCCESS;
}


//*****************************************************************************
//
// Reads the contents of the external Flash into a buffer.
//
//*****************************************************************************
uint32_t
am_devices_mspi_is25wx064_read(void *pHandle,
                               uint8_t *pui8RxBuffer,
                               uint32_t ui32ReadAddress,
                               uint32_t ui32NumBytes,
                               bool bWaitForCompletion)
{
    am_hal_mspi_dma_transfer_t    Transaction;
    uint32_t                      ui32Status;
    am_devices_mspi_is25wx064_t *pFlash = (am_devices_mspi_is25wx064_t *)pHandle;

    am_hal_mspi_timing_scan_t timingCfg;
    if ( bTimingConfigSaved )
    {
        timingCfg.bTxNeg            = sTimingConfigStored.bTxNeg;
        timingCfg.bRxNeg            = sTimingConfigStored.bRxNeg;
        timingCfg.bRxCap            = sTimingConfigStored.bRxCap;
        timingCfg.ui8TxDQSDelay     = sTimingConfigStored.ui8TxDQSDelay;
        timingCfg.ui8RxDQSDelay     = sTimingConfigStored.ui8RxDQSDelay;
        timingCfg.ui8Turnaround     = sTimingConfigStored.ui8Turnaround;
        am_hal_mspi_control(pFlash->pMspiHandle, AM_HAL_MSPI_REQ_TIMING_SCAN, &timingCfg);
    }
    else
    {
        switch(pFlash->stSetting.eDeviceConfig)
        {
            case AM_HAL_MSPI_FLASH_SERIAL_CE0:
            case AM_HAL_MSPI_FLASH_SERIAL_CE1:
                //
                // for timing config for serial mode, already initialed in the beginning
                //
                break;
            case AM_HAL_MSPI_FLASH_OCTAL_DDR_CE0:
            case AM_HAL_MSPI_FLASH_OCTAL_DDR_CE1:
                timingCfg = sTimingCfgDefault_OCTAL_DDR;
                am_hal_mspi_control(pFlash->pMspiHandle, AM_HAL_MSPI_REQ_TIMING_SCAN, &timingCfg);
                break;
            case AM_HAL_MSPI_FLASH_OCTAL_CE0_1_1_8:
            case AM_HAL_MSPI_FLASH_OCTAL_CE1_1_1_8:
                timingCfg = sTimingCfgDefault_OCTAL_1_1_8;
                am_hal_mspi_control(pFlash->pMspiHandle, AM_HAL_MSPI_REQ_TIMING_SCAN, &timingCfg);
                break;
            case AM_HAL_MSPI_FLASH_OCTAL_CE0_1_8_8:
            case AM_HAL_MSPI_FLASH_OCTAL_CE1_1_8_8:
                timingCfg = sTimingCfgDefault_OCTAL_1_8_8;
                am_hal_mspi_control(pFlash->pMspiHandle, AM_HAL_MSPI_REQ_TIMING_SCAN, &timingCfg);
                break;
            default:
                break;
        }
    }

    //
    // Set the DMA priority
    //
    Transaction.ui8Priority = 1;
    //
    // Set the transfer direction to RX (Read)
    //
    Transaction.eDirection = AM_HAL_MSPI_RX;
    //
    // Set the transfer count in bytes.
    //
    Transaction.ui32TransferCount = ui32NumBytes;
    //
    // Set the address to read data from.
    //
    Transaction.ui32DeviceAddress = ui32ReadAddress;
    //
    // Set the target SRAM buffer address.
    //
    Transaction.ui32SRAMAddress = (uint32_t)pui8RxBuffer;
    //
    // Clear the CQ stimulus.
    //
    Transaction.ui32PauseCondition = 0;
    //
    // Clear the post-processing
    //
    Transaction.ui32StatusSetClr = 0;

#if defined(AM_PART_APOLLO4)
    Transaction.eDeviceNum         = AM_HAL_MSPI_DEVICE0;
#endif

    if (bWaitForCompletion)
    {
        //
        // Start the transaction.
        //
        volatile uint32_t ui32DMAStatus = 0xFFFFFFFF;
        ui32Status = am_hal_mspi_nonblocking_transfer(pFlash->pMspiHandle, &Transaction, AM_HAL_MSPI_TRANS_DMA, pfnMSPI_IS25WX064_Callback, (void *)&ui32DMAStatus);
        //
        // Check the transaction status.
        //
        if (AM_HAL_STATUS_SUCCESS != ui32Status)
        {
            return AM_DEVICES_MSPI_IS25WX064_STATUS_ERROR;
        }
        //
        // Wait for DMA Complete or Timeout
        //
        for (uint32_t i = 0; i < AM_DEVICES_MSPI_IS25WX064_TIMEOUT; i++)
        {
            if ( (AM_HAL_STATUS_SUCCESS == ui32DMAStatus) || (AM_HAL_MSPI_FIFO_FULL_CONDITION == ui32DMAStatus) )
            {
                break;
            }

            //
            // Call the BOOTROM cycle function to delay for about 1 microsecond.
            //
            am_util_delay_us(1);
        }

        if (AM_HAL_MSPI_FIFO_FULL_CONDITION == ui32DMAStatus)
        {
            am_util_stdio_printf("read FIFOFULL!\n");
            return AM_HAL_MSPI_FIFO_FULL_CONDITION;
        }
        else if (AM_HAL_STATUS_SUCCESS == ui32DMAStatus)
        {
            return AM_DEVICES_MSPI_IS25WX064_STATUS_SUCCESS;
        }
        else
        {
            return AM_DEVICES_MSPI_IS25WX064_STATUS_ERROR;
        }
    }
    else
    {
        //
        // Check the transaction status.
        //
        ui32Status = am_hal_mspi_nonblocking_transfer(pFlash->pMspiHandle, &Transaction,
                                                      AM_HAL_MSPI_TRANS_DMA, NULL, NULL);
        if (AM_HAL_STATUS_SUCCESS != ui32Status)
        {
            return AM_DEVICES_MSPI_IS25WX064_STATUS_ERROR;
        }
    }

    //
    // Return the status.
    //
    return AM_DEVICES_MSPI_IS25WX064_STATUS_SUCCESS;
}


//*****************************************************************************
//
//! @brief Reads the contents of the extern Flash using DMA mode
//!
//! @param pHandle
//! @param pui8RxBuffer
//! @param ui32ReadAddress
//! @param ui32NumBytes
//!
//! @return uint32_t
//
//*****************************************************************************
static uint32_t
mspi_is25wx064_dma_read(void *pHandle,
                        uint8_t *pui8RxBuffer,
                        uint32_t ui32ReadAddress,
                        uint32_t ui32NumBytes)
{
    am_hal_mspi_dma_transfer_t    Transaction;
    uint32_t                      ui32Status;
    am_devices_mspi_is25wx064_t *pFlash = (am_devices_mspi_is25wx064_t *)pHandle;
    //
    // Set the DMA priority
    //
    Transaction.ui8Priority = 1;
    //
    // Set the transfer direction to RX (Read)
    //
    Transaction.eDirection = AM_HAL_MSPI_RX;
    //
    // Set the transfer count in bytes.
    //
    Transaction.ui32TransferCount = ui32NumBytes;
    //
    // Set the address to read data from.
    //
    Transaction.ui32DeviceAddress = ui32ReadAddress;
    //
    // Set the target SRAM buffer address.
    //
    Transaction.ui32SRAMAddress = (uint32_t)pui8RxBuffer;
    //
    // Clear the CQ stimulus.
    //
    Transaction.ui32PauseCondition = 0;
    //
    // Clear the post-processing
    //
    Transaction.ui32StatusSetClr = 0;

#if defined(AM_PART_APOLLO4)
    Transaction.eDeviceNum         = AM_HAL_MSPI_DEVICE0;
#endif
    //
    // Start the transaction.
    //
    volatile uint32_t ui32DMAStatus = 0xFFFFFFFF;
    ui32Status = am_hal_mspi_nonblocking_transfer(pFlash->pMspiHandle, &Transaction, AM_HAL_MSPI_TRANS_DMA, pfnMSPI_IS25WX064_Callback, (void *)&ui32DMAStatus);
    //
    // Check the transaction status.
    //
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_IS25WX064_STATUS_ERROR;
    }
    //
    // Wait for DMA Complete or Timeout
    //
    for (uint32_t i = 0; i < AM_DEVICES_MSPI_IS25WX064_TIMEOUT; i++)
    {
        //
        // check DMA status without using ISR
        //
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
        return AM_DEVICES_MSPI_IS25WX064_STATUS_SUCCESS;
    }
#else
    if (AM_HAL_STATUS_SUCCESS == ui32DMAStatus)
    {
        return AM_DEVICES_MSPI_IS25WX064_STATUS_SUCCESS;
    }
#endif
    else
    {
        return AM_DEVICES_MSPI_IS25WX064_STATUS_ERROR;
    }
}


//*****************************************************************************
//
// Reads the contents of the external flash into a buffer.
//
//*****************************************************************************
uint32_t
am_devices_mspi_is25wx064_read_hiprio(void *pHandle,
                                      uint8_t *pui8RxBuffer,
                                      uint32_t ui32ReadAddress,
                                      uint32_t ui32NumBytes,
                                      bool bWaitForCompletion)
{
    am_hal_mspi_dma_transfer_t    Transaction;
    uint32_t                      ui32Status;
    am_devices_mspi_is25wx064_t *pFlash = (am_devices_mspi_is25wx064_t *)pHandle;
    //
    // Set the DMA priority
    //
    Transaction.ui8Priority = 1;
    //
    // Set the transfer direction to RX (Read)
    //
    Transaction.eDirection = AM_HAL_MSPI_RX;
    //
    // Set the transfer count in bytes.
    //
    Transaction.ui32TransferCount = ui32NumBytes;
    //
    // Set the address to read data from.
    //
    Transaction.ui32DeviceAddress = ui32ReadAddress;
    //
    // Set the target SRAM buffer address.
    //
    Transaction.ui32SRAMAddress = (uint32_t)pui8RxBuffer;
    //
    // Clear the CQ stimulus.
    //
    Transaction.ui32PauseCondition = 0;
    //
    // Clear the post-processing
    //
    Transaction.ui32StatusSetClr = 0;

#if defined(AM_PART_APOLLO4)
    Transaction.eDeviceNum         = AM_HAL_MSPI_DEVICE0;
#endif

    if (bWaitForCompletion)
    {
        //
        // Start the transaction.
        //
        volatile bool bDMAComplete = false;
        ui32Status = am_hal_mspi_highprio_transfer(pFlash->pMspiHandle, &Transaction, AM_HAL_MSPI_TRANS_DMA, pfnMSPI_IS25WX064_Callback, (void*)&bDMAComplete);
        //
        // Check the transaction status.
        //
        if (AM_HAL_STATUS_SUCCESS != ui32Status)
        {
            return AM_DEVICES_MSPI_IS25WX064_STATUS_ERROR;
        }
        //
        // Wait for DMA Complete or Timeout
        //
        for (uint32_t i = 0; i < AM_DEVICES_MSPI_IS25WX064_TIMEOUT; i++)
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
        //
        // Check the status.
        //
        if (!bDMAComplete)
        {
            return AM_DEVICES_MSPI_IS25WX064_STATUS_ERROR;
        }
    }
    else
    {
        //
        // Check the transaction status.
        //
        ui32Status = am_hal_mspi_highprio_transfer(pFlash->pMspiHandle, &Transaction,
                                                      AM_HAL_MSPI_TRANS_DMA, NULL, NULL);
        if (AM_HAL_STATUS_SUCCESS != ui32Status)
        {
            return AM_DEVICES_MSPI_IS25WX064_STATUS_ERROR;
        }
    }

    //
    // Return the status.
    //
    return AM_DEVICES_MSPI_IS25WX064_STATUS_SUCCESS;
}



//*****************************************************************************
//
//  Programs the given range of flash addresses.
//
//*****************************************************************************
uint32_t
am_devices_mspi_is25wx064_write(void *pHandle,
                                uint8_t *pui8TxBuffer,
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
    am_devices_mspi_is25wx064_t     *pFlash = (am_devices_mspi_is25wx064_t *)pHandle;
    uint32_t                      ui32PIOBuffer[32] = {0};

    while (ui32BytesLeft > 0)
    {
        //
        // Send the command sequence to enable writing.
        //
        ui32Status = command_write_combo(pHandle, AM_DEVICES_MSPI_IS25WX064_OCTA_WRITE_ENABLE_CMD, false, 0, ui32PIOBuffer, 0);
        if (AM_HAL_STATUS_SUCCESS != ui32Status)
        {
            return AM_DEVICES_MSPI_IS25WX064_STATUS_ERROR;
        }

        am_hal_mspi_timing_scan_t timingCfg;
        if ( bTimingConfigSaved )
        {
            timingCfg.bTxNeg            = sTimingConfigStored.bTxNeg;
            timingCfg.bRxNeg            = sTimingConfigStored.bRxNeg;
            timingCfg.bRxCap            = sTimingConfigStored.bRxCap;
            timingCfg.ui8TxDQSDelay     = sTimingConfigStored.ui8TxDQSDelay;
            timingCfg.ui8RxDQSDelay     = sTimingConfigStored.ui8RxDQSDelay;
            timingCfg.ui8Turnaround     = sTimingConfigStored.ui8Turnaround;
            am_hal_mspi_control(pFlash->pMspiHandle, AM_HAL_MSPI_REQ_TIMING_SCAN, &timingCfg);
        }
        else
        {
            switch(pFlash->stSetting.eDeviceConfig)
            {
                case AM_HAL_MSPI_FLASH_SERIAL_CE0:
                case AM_HAL_MSPI_FLASH_SERIAL_CE1:
                    //
                    // for timing config for serial mode, already initialed in the beginning
                    //
                    break;
                case AM_HAL_MSPI_FLASH_OCTAL_DDR_CE0:
                case AM_HAL_MSPI_FLASH_OCTAL_DDR_CE1:
                    timingCfg = sTimingCfgDefault_OCTAL_DDR;
                    am_hal_mspi_control(pFlash->pMspiHandle, AM_HAL_MSPI_REQ_TIMING_SCAN, &timingCfg);
                    break;
                case AM_HAL_MSPI_FLASH_OCTAL_CE0_1_1_8:
                case AM_HAL_MSPI_FLASH_OCTAL_CE1_1_1_8:
                    timingCfg = sTimingCfgDefault_OCTAL_1_1_8;
                    am_hal_mspi_control(pFlash->pMspiHandle, AM_HAL_MSPI_REQ_TIMING_SCAN, &timingCfg);
                    break;
                case AM_HAL_MSPI_FLASH_OCTAL_CE0_1_8_8:
                case AM_HAL_MSPI_FLASH_OCTAL_CE1_1_8_8:
                    timingCfg = sTimingCfgDefault_OCTAL_1_8_8;
                    am_hal_mspi_control(pFlash->pMspiHandle, AM_HAL_MSPI_REQ_TIMING_SCAN, &timingCfg);
                    break;
                default:
                    break;
            }
        }
        //
        // Set the DMA priority
        //
        Transaction.ui8Priority = 1;
        //
        // Set the transfer direction to TX (Write)
        //
        Transaction.eDirection = AM_HAL_MSPI_TX;

        if (ui32BytesLeft > AM_DEVICES_MSPI_IS25WX064_PAGE_SIZE)
        {
            //
            // Set the transfer count in bytes.
            //
            Transaction.ui32TransferCount = AM_DEVICES_MSPI_IS25WX064_PAGE_SIZE;
            ui32BytesLeft -= AM_DEVICES_MSPI_IS25WX064_PAGE_SIZE;
        }
        else
        {
            //
            // Set the transfer count in bytes.
            //
            Transaction.ui32TransferCount = ui32BytesLeft;
            ui32BytesLeft = 0;
        }
        //
        // Set the address to read data to.
        //
        Transaction.ui32DeviceAddress = ui32PageAddress;
        ui32PageAddress += AM_DEVICES_MSPI_IS25WX064_PAGE_SIZE;
        //
        // Set the source SRAM buffer address.
        //
        Transaction.ui32SRAMAddress = ui32BufferAddress;
        ui32BufferAddress += AM_DEVICES_MSPI_IS25WX064_PAGE_SIZE;
        //
        // Clear the CQ stimulus.
        //
        Transaction.ui32PauseCondition = 0;
        //
        // Clear the post-processing
        //
        Transaction.ui32StatusSetClr = 0;

#if defined(AM_PART_APOLLO4)
        Transaction.eDeviceNum         = AM_HAL_MSPI_DEVICE0;
#endif
        //
        // Start the transaction.
        //
        volatile uint32_t ui32DMAStatus = 0xFFFFFFFF;
        ui32Status = am_hal_mspi_nonblocking_transfer(pFlash->pMspiHandle, &Transaction, AM_HAL_MSPI_TRANS_DMA, pfnMSPI_IS25WX064_Callback, (void*)&ui32DMAStatus);
        //
        // Check the transaction status.
        //
        if (AM_HAL_STATUS_SUCCESS != ui32Status)
        {
            return AM_DEVICES_MSPI_IS25WX064_STATUS_ERROR;
        }
        //
        // Wait for DMA Complete or Timeout
        //
        for (uint32_t i = 0; i < AM_DEVICES_MSPI_IS25WX064_TIMEOUT; i++)
        {

            if ( (AM_HAL_STATUS_SUCCESS == ui32DMAStatus) )
            {
                break;
            }

            //
            // Call the BOOTROM cycle function to delay for about 1 microsecond.
            //
            am_util_delay_us(1);
        }
        //
        // Check the status.
        //
        if (AM_HAL_STATUS_SUCCESS != ui32DMAStatus)
        {
            return AM_DEVICES_MSPI_IS25WX064_STATUS_ERROR;
        }

        //
        // Wait for the Write In Progress to indicate the erase is complete.
        //
        for (uint32_t i = 0; i < AM_DEVICES_MSPI_IS25WX064_TIMEOUT; i++)
        {
            ui32Status = command_read_combo(pHandle, AM_DEVICES_MSPI_IS25WX064_OCTA_READ_STATUS_REG_CMD, false, 0, ui32PIOBuffer, 2);
            if (AM_HAL_STATUS_SUCCESS != ui32Status)
            {
                return AM_DEVICES_MSPI_IS25WX064_STATUS_ERROR;
            }
            bWriteComplete = (0 == (ui32PIOBuffer[0] & AM_DEVICES_IS25WX064_WIP));

            am_util_delay_us(100);
            if (bWriteComplete)
            {
                break;
            }
        }

        //
        // Send the command sequence to disable writing.
        //
        ui32Status = command_write_combo(pHandle, AM_DEVICES_MSPI_IS25WX064_OCTA_WRITE_DISABLE_CMD, false, 0, ui32PIOBuffer, 0);
        if (AM_HAL_STATUS_SUCCESS != ui32Status)
        {
            return AM_DEVICES_MSPI_IS25WX064_STATUS_ERROR;
        }
    }


  //
  // Return the status.
  //
  return AM_DEVICES_MSPI_IS25WX064_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Write the contents of the extern Flash using DMA mode
//!
//! @param pHandle
//! @param pui8TxBuffer
//! @param ui32WriteAddress
//! @param ui32NumBytes
//!
//! @return uint32_t
//
//*****************************************************************************
static uint32_t
mspi_is25wx064_dma_write(void *pHandle,
                         uint8_t *pui8TxBuffer,
                         uint32_t ui32WriteAddress,
                         uint32_t ui32NumBytes)
{
    am_hal_mspi_dma_transfer_t    Transaction;
    bool                          bWriteComplete = false;
    uint32_t                      ui32BytesLeft = ui32NumBytes;
    uint32_t                      ui32PageAddress = ui32WriteAddress;
    uint32_t                      ui32BufferAddress = (uint32_t)pui8TxBuffer;
    uint32_t                      ui32Status;
    am_devices_mspi_is25wx064_t   *pFlash = (am_devices_mspi_is25wx064_t *)pHandle;
    uint32_t                      ui32PIOBuffer[32] = {0};


    while (ui32BytesLeft > 0)
    {
        //
        // Send the command sequence to enable writing.
        //
        ui32Status = command_write_combo(pHandle, AM_DEVICES_MSPI_IS25WX064_OCTA_WRITE_ENABLE_CMD, false, 0, ui32PIOBuffer, 0);
        if (AM_HAL_STATUS_SUCCESS != ui32Status)
        {
            return AM_DEVICES_MSPI_IS25WX064_STATUS_ERROR;
        }
        //
        // Set the DMA priority
        //
        Transaction.ui8Priority = 1;
        //
        // Set the transfer direction to TX (Write)
        //
        Transaction.eDirection = AM_HAL_MSPI_TX;

        if (ui32BytesLeft > AM_DEVICES_MSPI_IS25WX064_PAGE_SIZE)
        {
            //
            // Set the transfer count in bytes.
            //
            Transaction.ui32TransferCount = AM_DEVICES_MSPI_IS25WX064_PAGE_SIZE;
            ui32BytesLeft -= AM_DEVICES_MSPI_IS25WX064_PAGE_SIZE;
        }
        else
        {
            //
            // Set the transfer count in bytes.
            //
            Transaction.ui32TransferCount = ui32BytesLeft;
            ui32BytesLeft = 0;
        }
        //
        // Set the address to read data to.
        //
        Transaction.ui32DeviceAddress = ui32PageAddress;
        ui32PageAddress += AM_DEVICES_MSPI_IS25WX064_PAGE_SIZE;
        //
        // Set the source SRAM buffer address.
        //
        Transaction.ui32SRAMAddress = ui32BufferAddress;
        ui32BufferAddress += AM_DEVICES_MSPI_IS25WX064_PAGE_SIZE;
        //
        // Clear the CQ stimulus.
        //
        Transaction.ui32PauseCondition = 0;
        //
        // Clear the post-processing
        //
        Transaction.ui32StatusSetClr = 0;

#if defined(AM_PART_APOLLO4)
    Transaction.eDeviceNum         = AM_HAL_MSPI_DEVICE0;
#endif
        //
        // Start the transaction.
        //
        volatile uint32_t ui32DMAStatus = 0xFFFFFFFF;
        ui32Status = am_hal_mspi_nonblocking_transfer(pFlash->pMspiHandle, &Transaction, AM_HAL_MSPI_TRANS_DMA, pfnMSPI_IS25WX064_Callback, (void*)&ui32DMAStatus);
        //
        // Check the transaction status.
        //
        if (AM_HAL_STATUS_SUCCESS != ui32Status)
        {
            return AM_DEVICES_MSPI_IS25WX064_STATUS_ERROR;
        }
        //
        // Wait for DMA Complete or Timeout
        //
        for (uint32_t i = 0; i < AM_DEVICES_MSPI_IS25WX064_TIMEOUT; i++)
        {
            //
            // check DMA status without using ISR
            //
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
        //
        // Check the status.
        //
        if (AM_HAL_STATUS_SUCCESS != ui32DMAStatus)
        {
            return AM_DEVICES_MSPI_IS25WX064_STATUS_ERROR;
        }

        if (is25wx064_busy_wait_until(pHandle, AM_DEVICES_MSPI_IS25WX064_ERASE_TIMEOUT*10) == AM_DEVICES_MSPI_IS25WX064_STATUS_READY)
        {
            bWriteComplete = 1;
        }

        //
        // Check the status.
        //
        if (!bWriteComplete)
        {
            return AM_DEVICES_MSPI_IS25WX064_STATUS_ERROR;
        }

        //
        // Send the command sequence to disable writing.
        //

        ui32Status = command_write_combo(pHandle, AM_DEVICES_MSPI_IS25WX064_OCTA_WRITE_DISABLE_CMD, false, 0, ui32PIOBuffer, 0);
        if (AM_HAL_STATUS_SUCCESS != ui32Status)
        {
            return AM_DEVICES_MSPI_IS25WX064_STATUS_ERROR;
        }
    }

  //
  // Return the status.
  //
  return AM_DEVICES_MSPI_IS25WX064_STATUS_SUCCESS;
}


//*****************************************************************************
//
//  Erases the entire contents of the external flash
//
//*****************************************************************************
uint32_t
am_devices_mspi_is25wx064_mass_erase(void *pHandle)
{
    bool          bEraseComplete = false;
    uint32_t      ui32Status;
    //am_devices_mspi_is25wx064_t *pFlash = (am_devices_mspi_is25wx064_t *)pHandle;
    uint32_t      ui32PIOBuffer[32] = {0};

    //
    // Send the command sequence to enable writing.
    //
    ui32Status = command_write_combo(pHandle, AM_DEVICES_MSPI_IS25WX064_OCTA_WRITE_ENABLE_CMD, false, 0, ui32PIOBuffer, 0);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_IS25WX064_STATUS_ERROR;
    }

    //
    // Send the command sequence to do the mass erase.
    //
    ui32Status = command_write_combo(pHandle, AM_DEVICES_MSPI_IS25WX064_OCTA_CHIP_ERASE_CMD, false, 0, ui32PIOBuffer, 0);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_IS25WX064_STATUS_ERROR;
    }
    //
    // Wait for the Write In Progress to indicate the erase is complete.
    //
    for (uint32_t i = 0; i < AM_DEVICES_MSPI_IS25WX064_ERASE_TIMEOUT; i++)
    {
        ui32PIOBuffer[0] = 0;
        command_read_combo(pHandle, AM_DEVICES_MSPI_IS25WX064_OCTA_READ_STATUS_REG_CMD, false, 0, ui32PIOBuffer, 1);
        bEraseComplete = (0 == (ui32PIOBuffer[0] & AM_DEVICES_MSPI_IS25WX064_WIP));
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
        return AM_DEVICES_MSPI_IS25WX064_STATUS_ERROR;
    }

    //
    // Send the command sequence to disable writing.
    //
    ui32Status = command_write_combo(pHandle, AM_DEVICES_MSPI_IS25WX064_OCTA_WRITE_DISABLE_CMD, false, 0, ui32PIOBuffer, 0);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_IS25WX064_STATUS_ERROR;
    }

    //
    // Return the status.
    //
    return AM_DEVICES_MSPI_IS25WX064_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! Erases the contents of a single sector of flash
//
//*****************************************************************************
uint32_t
am_devices_mspi_is25wx064_sector_erase(void *pHandle, uint32_t ui32SectorAddress)
{
    bool          bEraseComplete = false;
    uint32_t      ui32Status;
    uint32_t      ui32PIOBuffer[32] = {0};

    //
    // Send the command sequence to enable writing.
    //
    ui32Status = command_write_combo(pHandle, AM_DEVICES_MSPI_IS25WX064_OCTA_WRITE_ENABLE_CMD, false, 0, ui32PIOBuffer, 0);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_IS25WX064_STATUS_ERROR;
    }

    //
    // Send the command sequence to do the sector erase.
    //
    ui32Status = command_write_combo(pHandle, AM_DEVICES_MSPI_IS25WX064_OCTA_SECTOR_ERASE_128K_4BYTE, true, ui32SectorAddress, ui32PIOBuffer, 0);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_IS25WX064_STATUS_ERROR;
    }

    if (is25wx064_busy_wait_until(pHandle, AM_DEVICES_MSPI_IS25WX064_ERASE_TIMEOUT*10) == AM_DEVICES_MSPI_IS25WX064_STATUS_READY)
    {
        bEraseComplete = 1;
    }

    //
    // Check the status.
    //
    if (!bEraseComplete)
    {
        return AM_DEVICES_MSPI_IS25WX064_STATUS_ERROR;
    }

    //
    // Send the command sequence to disable writing.
    //

    ui32Status = command_write_combo(pHandle, AM_DEVICES_MSPI_IS25WX064_OCTA_WRITE_DISABLE_CMD, false, 0, ui32PIOBuffer, 0);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_IS25WX064_STATUS_ERROR;
    }

    //
    // Return the status.
    //
    return AM_DEVICES_MSPI_IS25WX064_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Sets up the MSPI and external flash into XIP mode.
//
//*****************************************************************************
uint32_t
am_devices_mspi_is25wx064_enable_xip(void *pHandle)
{
    uint32_t ui32Status;
    am_devices_mspi_is25wx064_t *pFlash = (am_devices_mspi_is25wx064_t *)pHandle;

#if defined(AM_PART_APOLLO4_API)
    //
    // Set Aperture XIP range
    //
    ui32Status = am_hal_mspi_control(pFlash->pMspiHandle, AM_HAL_MSPI_REQ_XIP_CONFIG, &gXipConfig[pFlash->ui32Module]);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_IS25WX064_STATUS_ERROR;
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
        return AM_DEVICES_MSPI_IS25WX064_STATUS_ERROR;
    }

#if !MSPI_USE_CQ
    // Disable the DMA interrupts.
    ui32Status = am_hal_mspi_interrupt_disable(pFlash->pMspiHandle,
                                               AM_HAL_MSPI_INT_DMAERR |
                                               AM_HAL_MSPI_INT_DMACMP );
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_IS25WX064_STATUS_ERROR;
    }
#endif

    return ui32Status;
}

//*****************************************************************************
//
//! Removes the MSPI and external flash from XIP mode.
//
//*****************************************************************************
uint32_t
am_devices_mspi_is25wx064_disable_xip(void *pHandle)
{
    uint32_t ui32Status;
    am_devices_mspi_is25wx064_t *pFlash = (am_devices_mspi_is25wx064_t *)pHandle;

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
        return AM_DEVICES_MSPI_IS25WX064_STATUS_ERROR;
    }

    return ui32Status;
}

//*****************************************************************************
//
// Sets up the MSPI and external FLASH into scrambling mode.
//
//*****************************************************************************
uint32_t
am_devices_mspi_is25wx064_enable_scrambling(void *pHandle)
{
    uint32_t ui32Status;
    am_devices_mspi_is25wx064_t *pFlash = (am_devices_mspi_is25wx064_t *)pHandle;

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
        return AM_DEVICES_MSPI_IS25WX064_STATUS_ERROR;
    }
    return AM_DEVICES_MSPI_IS25WX064_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Disable the MSPI and external flash from scrambling mode.
//
//*****************************************************************************
uint32_t
am_devices_mspi_is25wx064_disable_scrambling(void *pHandle)
{
    uint32_t ui32Status;
    am_devices_mspi_is25wx064_t *pFlash = (am_devices_mspi_is25wx064_t *)pHandle;
    uint32_t      ui32PIOBuffer[32] = {0};

    //
    // Send the command to enable writing.
    //
    ui32Status = command_write_serial(pHandle, AM_DEVICES_MSPI_IS25WX064_WRITE_ENABLE, false, 0, ui32PIOBuffer, 0);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_IS25WX064_STATUS_ERROR;
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
        return AM_DEVICES_MSPI_IS25WX064_STATUS_ERROR;
    }

    return AM_DEVICES_MSPI_IS25WX064_STATUS_SUCCESS;
}

// #if defined(AM_PART_APOLLO4) || defined(AM_PART_APOLLO4B) || defined(AM_PART_APOLLO4P)  || defined(AM_PART_APOLLO4L)
#if defined(AM_PART_APOLLO4_API)

//*****************************************************************************
//
//! @brief Reads the contents of the external flash into a buffer.
//!
//! @param pattern_index - Index of the patten
//! @param buff - Buffer to store the received data from the flash
//! @param len - Number of bytes to read from external flash
//!
//! @return 32-bit status
//
//*****************************************************************************
static int prepare_test_pattern(uint32_t pattern_index, uint8_t* buff, uint32_t len)
{
    uint32_t *pui32TxPtr = (uint32_t*)buff;
    uint8_t  *pui8TxPtr  = (uint8_t*)buff;
    //
    // length has to be multiple of 4 bytes
    //
    if ( len % 4 )
    {
        return -1;
    }

    switch ( pattern_index )
    {
        case 0:
            //
            // 0x5555AAAA
            //
            for (uint32_t i = 0; i < len / 4; i++)
            {
               pui32TxPtr[i] = (0x5555AAAA);
            }
            break;
        case 1:
            //
            // 0xFFFF0000
            //
            for (uint32_t i = 0; i < len / 4; i++)
            {
               pui32TxPtr[i] = (0xFFFF0000);
            }
            break;
        case 2:
            //
            // walking
            //
            for (uint32_t i = 0; i < len; i++)
            {
               pui8TxPtr[i] = 0x01 << (i % 8);
            }
            break;
        case 3:
            //
            // incremental from 1
            //
            for (uint32_t i = 0; i < len; i++)
            {
               pui8TxPtr[i] = ((i + 1) & 0xFF);
            }
            break;
        case 4:
            //
            // decremental from 0xff
            //
            for ( uint32_t i = 0; i < len; i++ )
            {
                //
                // decrement starting from 0xff
                //
                pui8TxPtr[i] = (0xff - i) & 0xFF;
            }
            break;
        default:
            //
            // incremental from 1
            //
            for (uint32_t i = 0; i < len; i++)
            {
               pui8TxPtr[i] = ((i + 1) & 0xFF);
            }
            break;

    }

    return 0;
}

AM_SHARED_RW uint8_t  ui8TxBuffer[FLASH_CHECK_DATA_SIZE_BYTES];
AM_SHARED_RW uint8_t  ui8RxBuffer[FLASH_CHECK_DATA_SIZE_BYTES];
//*****************************************************************************
//
//! @brief Reads the contents of the external flash into a buffer.
//!
//! @param flashHandle - MSPI Device handle.
//! @param length - Number of bytes to read from external flash
//!
//! @return 32-bit status
//
//*****************************************************************************
static bool
flash_write(void* flashHandle, uint32_t length)
{
    //
    // Try to use as less ram as possible in stack
    //
    uint32_t ui32NumberOfBytesLeft = length;
    uint32_t ui32TestBytes = 0;
    uint32_t ui32AddressOffset = 0;
    uint16_t ui8PatternCounter = 0;

    uint32_t ui32Status = AM_DEVICES_MSPI_IS25WX064_STATUS_SUCCESS;

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
        // write to target address
        //
        ui32Status = mspi_is25wx064_dma_write(flashHandle, ui8TxBuffer,
                                            (AM_DEVICES_MSPI_IS25WX064_SECTOR_FOR_TIMING_CHECK << AM_DEVICES_MSPI_IS25WX064_SECTOR_SHIFT) + ui32AddressOffset,
                                            ui32TestBytes);
        if ( ui32Status ==  AM_DEVICES_MSPI_IS25WX064_STATUS_ERROR)
        {
            return true;
        }

        ui32AddressOffset += ui32TestBytes;
    }

    return false;
}

//*****************************************************************************
//
//! @brief Reads the contents of the external flash into a buffer.
//!
//! @param flashHandle - MSPI Device handle.
//! @param length - Number of bytes to read from external flash
//!
//! @return 32-bit status
//
//*****************************************************************************
static bool
flash_check(void* flashHandle, uint32_t length)
{
    //
    // Try to use as less ram as possible in stack
    //
    uint32_t ui32NumberOfBytesLeft = length;
    uint32_t ui32TestBytes = 0;
    uint32_t ui32AddressOffset = 0;
    uint16_t ui8PatternCounter = 0;
    uint32_t ui32Status = AM_DEVICES_MSPI_IS25WX064_STATUS_SUCCESS;

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
        ui32Status = mspi_is25wx064_dma_read(flashHandle, ui8RxBuffer,
                                    (AM_DEVICES_MSPI_IS25WX064_SECTOR_FOR_TIMING_CHECK << AM_DEVICES_MSPI_IS25WX064_SECTOR_SHIFT) + ui32AddressOffset,
                                    ui32TestBytes);
#else
        am_devices_mspi_is25wx064_t *pFlash = (am_devices_mspi_is25wx064_t *)flashHandle;

        if ( pFlash->stSetting.eClockFreq == AM_HAL_MSPI_CLK_96MHZ )
        {
            //
            // Read the data back into the RX buffer using XIP
            //
            ui32Status = am_devices_mspi_is25wx064_enable_xip(flashHandle);
            if (AM_DEVICES_MSPI_IS25WX064_STATUS_SUCCESS != ui32Status)
            {
                am_util_debug_printf("Failed to put the MSPI into XIP mode!\n");
            }
            am_hal_sysctrl_bus_write_flush();
            uint8_t * xipPointer = (uint8_t *)(ui32MspiXipBaseAddress[pFlash->ui32Module] + (AM_DEVICES_MSPI_IS25WX064_SECTOR_FOR_TIMING_CHECK << AM_DEVICES_MSPI_IS25WX064_SECTOR_SHIFT) + ui32AddressOffset);
            memcpy((uint8_t*)ui8RxBuffer, xipPointer, ui32TestBytes);

            //
            // Quit XIP mode
            //
            ui32Status = mspi_is25wx064_disable_xip(flashHandle);
            if (AM_DEVICES_MSPI_IS25WX064_STATUS_SUCCESS != ui32Status)
            {
                am_util_debug_printf("Failed to disable XIP mode in the MSPI!\n");
            }
        }
        else
        {
            ui32Status = mspi_is25wx064_dma_read(flashHandle, ui8RxBuffer,
                                                (AM_DEVICES_MSPI_IS25WX064_SECTOR_FOR_TIMING_CHECK << AM_DEVICES_MSPI_IS25WX064_SECTOR_SHIFT) + ui32AddressOffset,
                                                ui32TestBytes);
        }
#endif

        if ( ui32Status ==  AM_DEVICES_MSPI_IS25WX064_STATUS_ERROR)
        {
            return true;
        }

        //
        // Verify the result
        //
        if ( memcmp(ui8RxBuffer, ui8TxBuffer, ui32TestBytes) )
        {
            //am_util_debug_printf("    Failed to verify at offset 0x%08x!\n", ui32AddressOffset);
            //
            // verify failed, return directly
            //
            return true;
        }

        ui32AddressOffset += ui32TestBytes;
    }

    return false;
}

//*****************************************************************************
//
//! @brief Count the longest consecutive 1s in a 32bit word
//!
//! @param pVal - Data will be count
//!
//! @return 32-bit count
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
//! @param pVal - Data will be count
//!
//! @return 32-bit count
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
        //
        // window is likely on low side
        //
        pick_point = pick_point - remainder;    // minus only when pattern length is odd
    }
    else if ( (pick_point > 15) && (val & 0x40000000) )
    {
        //
        // window is likely on high side
        //
        pick_point = pick_point + 1;
    }
    else
    {
        //
        // window is in the middle, no action
        //
    }

    return pick_point;
}
#endif



#if defined(AM_PART_APOLLO4) || defined(AM_PART_APOLLO4B)
//*****************************************************************************
//
//! @brief Checks flash timing and determine a delay setting.
//!
//! @param pDeviceID - Pointer to the return buffer for the Device ID.
//!
//! This function scans through the delay settings of MSPI DDR mode and selects
//! the best parameter to use by tuning TURNAROUND/RXNEG/RXDQSDELAY0 values.
//! This function is only valid in DDR mode and ENABLEDQS0 = 0.
//!
//! @return 32-bit status, scan result in structure type
//
//*****************************************************************************

//
//! @note If you uncomment out lines, the test will automatically include them.
//! No need to change any code below.
//
const am_devices_mspi_is25wx064_timing_config_t is25wx064_sConfigArray[] =
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
//! @brief
//!
//! @param module
//! @param pDevCfg
//! @param psDevTimingCfg
//! @return uint32_t
//*****************************************************************************
uint32_t
am_devices_mspi_is25wx064_init_timing_check(uint32_t module,
                                            am_devices_mspi_is25wx064_config_t *pDevCfg,
                                            am_devices_mspi_is25wx064_timing_config_t *psDevTimingCfg)
{
    uint32_t ui32Status;
    void *pDevHandle;
    void *pHandle;

    uint32_t ui32ResultArray[sizeof(is25wx064_sConfigArray) / sizeof(am_devices_mspi_is25wx064_timing_config_t)] = {0};
    const uint32_t ui32TestSize = sizeof(is25wx064_sConfigArray) / sizeof(am_devices_mspi_is25wx064_timing_config_t);

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
    //
    // clear previous saved config, rescan
    //
    if ( bTimingConfigSaved == true )
    {
        bTimingConfigSaved                   = false;
        sTimingConfigStored.ui32Rxdqsdelay    = sTimingConfigDefault.ui32Rxdqsdelay;
        sTimingConfigStored.ui32Rxneg         = sTimingConfigDefault.ui32Rxneg;
        sTimingConfigStored.ui32Turnaround    = sTimingConfigDefault.ui32Turnaround;
    }

    //
    // initialize interface
    //
    ui32Status = am_devices_mspi_is25wx064_init(module, pDevCfg, &pDevHandle, &pHandle);
    if (AM_DEVICES_MSPI_IS25WX064_STATUS_SUCCESS != ui32Status)
    {
        am_util_debug_printf("    Failed to configure the MSPI and Flash Device correctly!\n");
        return ui32Status;
    }

    //
    // erase target sector first (each "sector is 64Kbyte block")
    //
    if ( FLASH_TIMING_SCAN_SIZE_BYTES % AM_DEVICES_MSPI_IS25WX064_SECTOR_SIZE )
    {
        //
        // scan size shall be at block boundary
        //
        am_util_debug_printf("ERROR: Timing scan data size shall be at sector boundary!\n");
        return AM_DEVICES_MSPI_IS25WX064_STATUS_ERROR;
    }
    for ( uint8_t i = 0; i < (FLASH_TIMING_SCAN_SIZE_BYTES / AM_DEVICES_MSPI_IS25WX064_SECTOR_SIZE); i++ )
    {
        ui32Status = am_devices_mspi_is25wx064_sector_erase(pDevHandle,
                                                        (AM_DEVICES_MSPI_IS25WX064_SECTOR_FOR_TIMING_CHECK << AM_DEVICES_MSPI_IS25WX064_SECTOR_SHIFT) + i*AM_DEVICES_MSPI_IS25WX064_SECTOR_SIZE);

        if (AM_DEVICES_MSPI_IS25WX064_STATUS_SUCCESS != ui32Status)
        {
            am_util_debug_printf("Failed to erase Flash Device sector!\n");
            return AM_DEVICES_MSPI_IS25WX064_STATUS_ERROR;
        }
    }
    //
    // write test pattern into target sector
    //
    if ( flash_write(pDevHandle, FLASH_TIMING_SCAN_SIZE_BYTES) )
    {
        return AM_DEVICES_MSPI_IS25WX064_STATUS_ERROR;
    }

    //
    // Start scan loop
    //
    for ( uint8_t i = 0; i < ui32TestSize; i++ )
    {
        //
        // set Turnaround and RXNEG
        //
        scanCfg.ui8PioTurnaround    = scanCfg.ui8XipTurnaround = is25wx064_sConfigArray[i].ui32Turnaround;
        scanCfg.bRxNeg              = is25wx064_sConfigArray[i].ui32Rxneg;
        for ( uint8_t RxDqs_Index = 1; RxDqs_Index < 31; RxDqs_Index++ )
        {
            //
            // set RXDQSDELAY0 value
            //
            scanCfg.ui8RxDQSDelay   = RxDqs_Index;
            //
            // apply settings
            //
            ui32Status = am_hal_mspi_control(pHandle, AM_HAL_MSPI_REQ_DQS, &scanCfg);
            if (AM_HAL_STATUS_SUCCESS != ui32Status)
            {
                return AM_DEVICES_MSPI_IS25WX064_STATUS_ERROR;
            }
            //
            // run data check
            //
            if ( false == flash_check(pDevHandle, FLASH_TIMING_SCAN_SIZE_BYTES) )
            {
                //
                // data check pass
                //
                ui32ResultArray[i] |= 0x01 << RxDqs_Index;
            }
            else
            {
                //
                // data check failed
                //
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
        am_util_debug_printf(" Turnaround %d - RXNEG %d = 0x%08X\n", is25wx064_sConfigArray[i].ui32Turnaround, is25wx064_sConfigArray[i].ui32Rxneg, ui32ResultArray[i]);
    }

    am_util_debug_printf("Timing Scan found a window %d fine steps wide.\n", ui32MaxOnes);

    //
    // Find RXDQSDELAY Value
    //
    uint32_t dqsdelay = find_mid_point(&ui32ResultArray[ui32MaxOnesIndex]);

    //
    // Deinitialize the MSPI interface
    //
    am_devices_mspi_is25wx064_deinit(pDevHandle);
    NVIC_ClearPendingIRQ(mspi_interrupts[module]);

    //
    // Check consecutive passing settings
    //
    if ( ui32MaxOnes < IS25WX064_TIMING_SCAN_MIN_ACCEPTANCE_LENGTH )
    {
        //
        // too short is the passing settings, use default setting
        //
        psDevTimingCfg->ui32Rxdqsdelay = sTimingConfigDefault.ui32Rxdqsdelay;
        psDevTimingCfg->ui32Rxneg = sTimingConfigDefault.ui32Rxneg;
        psDevTimingCfg->ui32Turnaround = sTimingConfigDefault.ui32Turnaround;
        return AM_DEVICES_MSPI_IS25WX064_STATUS_ERROR;
    }
    else
    {
        //
        // Set output values
        //
        psDevTimingCfg->ui32Rxdqsdelay = dqsdelay;
        psDevTimingCfg->ui32Rxneg = is25wx064_sConfigArray[ui32MaxOnesIndex].ui32Rxneg;
        psDevTimingCfg->ui32Turnaround = is25wx064_sConfigArray[ui32MaxOnesIndex].ui32Turnaround;

        return AM_DEVICES_MSPI_IS25WX064_STATUS_SUCCESS;
    }
}

//*****************************************************************************
//
//! @brief Apply given SDR timing settings to target MSPI instance.
//!
//! @param pHandle - Handle to the flash.
//! @param psDevTimingCfg - Pointer to the ddr timing config structure
//!
//! This function applies the ddr timing settings to the selected mspi instance.
//! This function must be called after MSPI instance is initialized into
//! ENABLEFINEDELAY0 = 1 mode.
//!
//! @return 32-bit status
//
//*****************************************************************************
uint32_t
am_devices_mspi_is25wx064_apply_ddr_timing(void *pHandle,
                                           am_devices_mspi_is25wx064_timing_config_t *psDevTimingCfg)
{
    am_devices_mspi_is25wx064_t *pFlash = (am_devices_mspi_is25wx064_t *)pHandle;
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
    //
    // apply timing settings: Turnaround, RXNEG and RXDQSDELAY
    //
    applyCfg.ui8RxDQSDelay      = psDevTimingCfg->ui32Rxdqsdelay;
    applyCfg.ui8PioTurnaround   = psDevTimingCfg->ui32Turnaround;
    applyCfg.ui8XipTurnaround   = psDevTimingCfg->ui32Turnaround;
    applyCfg.bRxNeg             = psDevTimingCfg->ui32Rxneg;
    //
    // save a local copy of the timing settings
    //
    if ( bTimingConfigSaved == false )
    {
        bTimingConfigSaved                   = true;
        sTimingConfigStored.ui32Rxdqsdelay    = psDevTimingCfg->ui32Rxdqsdelay;
        sTimingConfigStored.ui32Rxneg         = psDevTimingCfg->ui32Rxneg;
        sTimingConfigStored.ui32Turnaround    = psDevTimingCfg->ui32Turnaround;
    }

    return am_hal_mspi_control(pFlash->pMspiHandle, AM_HAL_MSPI_REQ_DQS, &applyCfg);

}
#endif

#if defined(AM_PART_APOLLO4P) || defined(AM_PART_APOLLO4L)
//
//! @note If you uncomment out lines, the test will automatically include them.
//! No need to change any code below.
//
am_devices_mspi_is25wx064_timing_config_t is25wx064_sConfigArray[] =
{
//TXNEG RXNEG RXCAP TXDLY RXDLY TURNAROUND
    {1 ,  0,   0,    1,    1,      29},
    {1 ,  0,   0,    2,    1,      29},
    {1 ,  0,   0,    3,    1,      29},
    {1 ,  0,   0,    4,    1,      29},
    {1 ,  0,   0,    5,    1,      29},
    {1 ,  0,   0,    6,    1,      29},
    {1 ,  0,   0,    7,    1,      29},
    {1 ,  0,   0,    8,    1,      29},
    {1 ,  0,   0,    9,    1,      29},
    {1 ,  0,   0,    1,    1,      30},
    {1 ,  0,   0,    2,    1,      30},
    {1 ,  0,   0,    3,    1,      30},
    {1 ,  0,   0,    4,    1,      30},
    {1 ,  0,   0,    5,    1,      30},
    {1 ,  0,   0,    6,    1,      30},
    {1 ,  0,   0,    7,    1,      30},
    {1 ,  0,   0,    8,    1,      30},
    {1 ,  0,   0,    9,    1,      30},
    {1 ,  0,   0,    1,    1,      31},
    {1 ,  0,   0,    2,    1,      31},
    {1 ,  0,   0,    3,    1,      31},
    {1 ,  0,   0,    4,    1,      31},
    {1 ,  0,   0,    5,    1,      31},
    {1 ,  0,   0,    6,    1,      31},
    {1 ,  0,   0,    7,    1,      31},
    {1 ,  0,   0,    8,    1,      31},
    {1 ,  0,   0,    9,    1,      31},
    {1 ,  0,   0,    1,    1,      32},
    {1 ,  0,   0,    2,    1,      32},
    {1 ,  0,   0,    3,    1,      32},
    {1 ,  0,   0,    4,    1,      32},
    {1 ,  0,   0,    5,    1,      32},
    {1 ,  0,   0,    6,    1,      32},
    {1 ,  0,   0,    7,    1,      32},
    {1 ,  0,   0,    8,    1,      32},
    {1 ,  0,   0,    9,    1,      32},
    {1 ,  0,   0,    1,    1,      33},
    {1 ,  0,   0,    2,    1,      33},
    {1 ,  0,   0,    3,    1,      33},
    {1 ,  0,   0,    4,    1,      33},
    {1 ,  0,   0,    5,    1,      33},
    {1 ,  0,   0,    6,    1,      33},
    {1 ,  0,   0,    7,    1,      33},
    {1 ,  0,   0,    8,    1,      33},
    {1 ,  0,   0,    9,    1,      33},
    {1 ,  0,   0,    1,    1,      34},
    {1 ,  0,   0,    2,    1,      34},
    {1 ,  0,   0,    3,    1,      34},
    {1 ,  0,   0,    4,    1,      34},
    {1 ,  0,   0,    5,    1,      34},
    {1 ,  0,   0,    6,    1,      34},
    {1 ,  0,   0,    7,    1,      34},
    {1 ,  0,   0,    8,    1,      34},
    {1 ,  0,   0,    9,    1,      34},
    {1 ,  0,   1,    1,    1,      29},
    {1 ,  0,   1,    2,    1,      29},
    {1 ,  0,   1,    3,    1,      29},
    {1 ,  0,   1,    4,    1,      29},
    {1 ,  0,   1,    5,    1,      29},
    {1 ,  0,   1,    6,    1,      29},
    {1 ,  0,   1,    7,    1,      29},
    {1 ,  0,   1,    8,    1,      29},
    {1 ,  0,   1,    9,    1,      29},
    {1 ,  0,   1,    1,    1,      30},
    {1 ,  0,   1,    2,    1,      30},
    {1 ,  0,   1,    3,    1,      30},
    {1 ,  0,   1,    4,    1,      30},
    {1 ,  0,   1,    5,    1,      30},
    {1 ,  0,   1,    6,    1,      30},
    {1 ,  0,   1,    7,    1,      30},
    {1 ,  0,   1,    8,    1,      30},
    {1 ,  0,   1,    9,    1,      30},
    {1 ,  0,   1,    1,    1,      31},
    {1 ,  0,   1,    2,    1,      31},
    {1 ,  0,   1,    3,    1,      31},
    {1 ,  0,   1,    4,    1,      31},
    {1 ,  0,   1,    5,    1,      31},
    {1 ,  0,   1,    6,    1,      31},
    {1 ,  0,   1,    7,    1,      31},
    {1 ,  0,   1,    8,    1,      31},
    {1 ,  0,   1,    9,    1,      31},
    {1 ,  0,   1,    1,    1,      32},
    {1 ,  0,   1,    2,    1,      32},
    {1 ,  0,   1,    3,    1,      32},
    {1 ,  0,   1,    4,    1,      32},
    {1 ,  0,   1,    5,    1,      32},
    {1 ,  0,   1,    6,    1,      32},
    {1 ,  0,   1,    7,    1,      32},
    {1 ,  0,   1,    8,    1,      32},
    {1 ,  0,   1,    9,    1,      32},
    {1 ,  0,   1,    1,    1,      33},
    {1 ,  0,   1,    2,    1,      33},
    {1 ,  0,   1,    3,    1,      33},
    {1 ,  0,   1,    4,    1,      33},
    {1 ,  0,   1,    5,    1,      33},
    {1 ,  0,   1,    6,    1,      33},
    {1 ,  0,   1,    7,    1,      33},
    {1 ,  0,   1,    8,    1,      33},
    {1 ,  0,   1,    9,    1,      33},
    {1 ,  0,   1,    1,    1,      34},
    {1 ,  0,   1,    2,    1,      34},
    {1 ,  0,   1,    3,    1,      34},
    {1 ,  0,   1,    4,    1,      34},
    {1 ,  0,   1,    5,    1,      34},
    {1 ,  0,   1,    6,    1,      34},
    {1 ,  0,   1,    7,    1,      34},
    {1 ,  0,   1,    8,    1,      34},
    {1 ,  0,   1,    9,    1,      34},
    {1 ,  1,   0,    1,    1,      29},
    {1 ,  1,   0,    2,    1,      29},
    {1 ,  1,   0,    3,    1,      29},
    {1 ,  1,   0,    4,    1,      29},
    {1 ,  1,   0,    5,    1,      29},
    {1 ,  1,   0,    6,    1,      29},
    {1 ,  1,   0,    1,    1,      30},
    {1 ,  1,   0,    2,    1,      30},
    {1 ,  1,   0,    3,    1,      30},
    {1 ,  1,   0,    4,    1,      30},
    {1 ,  1,   0,    5,    1,      30},
    {1 ,  1,   0,    6,    1,      30},
    {1 ,  1,   0,    1,    1,      31},
    {1 ,  1,   0,    2,    1,      31},
    {1 ,  1,   0,    3,    1,      31},
    {1 ,  1,   0,    4,    1,      31},
    {1 ,  1,   0,    5,    1,      31},
    {1 ,  1,   0,    6,    1,      31},
    {1 ,  1,   0,    1,    1,      32},
    {1 ,  1,   0,    2,    1,      32},
    {1 ,  1,   0,    3,    1,      32},
    {1 ,  1,   0,    4,    1,      32},
    {1 ,  1,   0,    5,    1,      32},
    {1 ,  1,   0,    6,    1,      32},
    {1 ,  1,   0,    1,    1,      33},
    {1 ,  1,   0,    2,    1,      33},
    {1 ,  1,   0,    3,    1,      33},
    {1 ,  1,   0,    4,    1,      33},
    {1 ,  1,   0,    5,    1,      33},
    {1 ,  1,   0,    6,    1,      33},
    {1 ,  1,   0,    1,    1,      34},
    {1 ,  1,   0,    2,    1,      34},
    {1 ,  1,   0,    3,    1,      34},
    {1 ,  1,   0,    4,    1,      34},
    {1 ,  1,   0,    5,    1,      34},
    {1 ,  1,   0,    6,    1,      34},
    {1 ,  1,   1,    1,    1,      29},
    {1 ,  1,   1,    2,    1,      29},
    {1 ,  1,   1,    3,    1,      29},
    {1 ,  1,   1,    4,    1,      29},
    {1 ,  1,   1,    5,    1,      29},
    {1 ,  1,   1,    6,    1,      29},
    {1 ,  1,   1,    1,    1,      30},
    {1 ,  1,   1,    2,    1,      30},
    {1 ,  1,   1,    3,    1,      30},
    {1 ,  1,   1,    4,    1,      30},
    {1 ,  1,   1,    5,    1,      30},
    {1 ,  1,   1,    6,    1,      30},
    {1 ,  1,   1,    1,    1,      31},
    {1 ,  1,   1,    2,    1,      31},
    {1 ,  1,   1,    3,    1,      31},
    {1 ,  1,   1,    4,    1,      31},
    {1 ,  1,   1,    5,    1,      31},
    {1 ,  1,   1,    6,    1,      31},
    {1 ,  1,   1,    1,    1,      32},
    {1 ,  1,   1,    2,    1,      32},
    {1 ,  1,   1,    3,    1,      32},
    {1 ,  1,   1,    4,    1,      32},
    {1 ,  1,   1,    5,    1,      32},
    {1 ,  1,   1,    6,    1,      32},
    {1 ,  1,   1,    7,    1,      32},
    {1 ,  1,   1,    8,    1,      32},
    {1 ,  1,   1,    9,    1,      32},
    {1 ,  1,   1,    1,    1,      33},
    {1 ,  1,   1,    2,    1,      33},
    {1 ,  1,   1,    3,    1,      33},
    {1 ,  1,   1,    4,    1,      33},
    {1 ,  1,   1,    5,    1,      33},
    {1 ,  1,   1,    6,    1,      33},
    {1 ,  1,   1,    7,    1,      33},
    {1 ,  1,   1,    8,    1,      33},
    {1 ,  1,   1,    9,    1,      33},
    {1 ,  1,   1,    1,    1,      34},
    {1 ,  1,   1,    2,    1,      34},
    {1 ,  1,   1,    3,    1,      34},
    {1 ,  1,   1,    4,    1,      34},
    {1 ,  1,   1,    5,    1,      34},
    {1 ,  1,   1,    6,    1,      34},
    {1 ,  1,   1,    7,    1,      34},
    {1 ,  1,   1,    8,    1,      34},
    {1 ,  1,   1,    9,    1,      34},
};

//*****************************************************************************
//
// Checks flash timing and determine a delay setting.
//
//*****************************************************************************
uint32_t
am_devices_mspi_is25wx064_init_timing_check(uint32_t module,
                                            am_devices_mspi_is25wx064_config_t *pDevCfg,
                                            am_devices_mspi_is25wx064_timing_config_t *psDevTimingCfg)
{
    uint32_t ui32Status;
    void *pDevHandle;
    void *pHandle;

    uint32_t ui32ResultArray[sizeof(is25wx064_sConfigArray) / sizeof(am_devices_mspi_is25wx064_timing_config_t)] = {0};
    const uint32_t ui32TestSize = sizeof(is25wx064_sConfigArray) / sizeof(am_devices_mspi_is25wx064_timing_config_t);
#if defined(FAST_TIMING_SCAN)
    bool bConfigSaved = bTimingConfigSaved;
#endif

    am_hal_mspi_timing_scan_t scanCfg =
    {
        .bTxNeg            = 1,
        .bRxNeg            = 0,
        .bRxCap            = 0,
        .ui8TxDQSDelay     = 4,
        .ui8RxDQSDelay     = 16,
        .ui8Turnaround     = 31,
    };

    am_hal_mspi_dqs_t dqsCfg;
    //
    // clear previous saved config, rescan
    //
    if ( bTimingConfigSaved == true )
    {
        bTimingConfigSaved = false;
    }

    //
    // initialize interface
    //
    ui32Status = am_devices_mspi_is25wx064_init(module, pDevCfg, &pDevHandle, &pHandle);
    if (AM_DEVICES_MSPI_IS25WX064_STATUS_SUCCESS != ui32Status)
    {
        am_util_debug_printf("    Failed to configure the MSPI and Flash Device correctly!\n");
        return ui32Status;
    }
    //
    // do not use enable fine delay for command read
    //
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
    if ( FLASH_TIMING_SCAN_SIZE_BYTES % AM_DEVICES_MSPI_IS25WX064_SECTOR_SIZE )
    {
        //
        // scan size shall be at block boundary
        //
        am_util_debug_printf("ERROR: Timing scan data size shall be at sector boundary!\n");
        return AM_DEVICES_MSPI_IS25WX064_STATUS_ERROR;
    }

    for ( uint8_t i = 0; i < (FLASH_TIMING_SCAN_SIZE_BYTES / AM_DEVICES_MSPI_IS25WX064_SECTOR_SIZE); i++ )
    {
        ui32Status = am_devices_mspi_is25wx064_sector_erase(pDevHandle,
                     (AM_DEVICES_MSPI_IS25WX064_SECTOR_FOR_TIMING_CHECK << AM_DEVICES_MSPI_IS25WX064_SECTOR_SHIFT) + i*AM_DEVICES_MSPI_IS25WX064_SECTOR_SIZE);

        if (AM_DEVICES_MSPI_IS25WX064_STATUS_SUCCESS != ui32Status)
        {
            am_util_debug_printf("Failed to erase Flash Device sector!\n");
            return AM_DEVICES_MSPI_IS25WX064_STATUS_ERROR;
        }
    }
    //
    // write test pattern into target sector
    //
    if ( flash_write(pDevHandle, FLASH_TIMING_SCAN_SIZE_BYTES) )
    {
        return AM_DEVICES_MSPI_IS25WX064_STATUS_ERROR;
    }
    am_util_debug_printf("flash write complete, start MSPI Timing Scan!\n");

#if defined(FAST_TIMING_SCAN)
    am_hal_mspi_timing_scan_t timingCfg;
    if (bConfigSaved)
    {
        timingCfg.bTxNeg            = sTimingConfigStored.bTxNeg;
        timingCfg.bRxNeg            = sTimingConfigStored.bRxNeg;
        timingCfg.bRxCap            = sTimingConfigStored.bRxCap;
        timingCfg.ui8TxDQSDelay     = sTimingConfigStored.ui8TxDQSDelay;
        timingCfg.ui8RxDQSDelay     = sTimingConfigStored.ui8RxDQSDelay;
        timingCfg.ui8Turnaround     = sTimingConfigStored.ui8Turnaround;
        am_hal_mspi_control(pHandle, AM_HAL_MSPI_REQ_TIMING_SCAN, &timingCfg);
        if (AM_HAL_STATUS_SUCCESS != ui32Status)
        {
            return AM_DEVICES_MSPI_IS25WX064_STATUS_ERROR;
        }
        if ( false == flash_check(pDevHandle, FLASH_TIMING_SCAN_SIZE_BYTES) )
        {
            am_util_debug_printf("  Skipping MSPI timing scan!\n");
            *psDevTimingCfg = sTimingConfigStored;
            //
            // Deinitialize the MSPI interface
            //
            am_devices_mspi_is25wx064_deinit(pDevHandle);
            NVIC_ClearPendingIRQ(mspi_interrupts[module]);
            return AM_DEVICES_MSPI_IS25WX064_STATUS_SUCCESS;
        }
    }
#endif

    //
    // Start scan loop
    //
    am_util_debug_printf("timing scanning ");
    for ( uint8_t i = 0; i < ui32TestSize; i++ )
    {
        if (i % 4 == 0)
        {
            am_util_debug_printf(".");
        }
        //
        // set Timing Scan Parameters
        //
        scanCfg.bTxNeg                = is25wx064_sConfigArray[i].bTxNeg;
        scanCfg.bRxNeg                = is25wx064_sConfigArray[i].bRxNeg;
        scanCfg.bRxCap                = is25wx064_sConfigArray[i].bRxCap;
        scanCfg.ui8TxDQSDelay         = is25wx064_sConfigArray[i].ui8TxDQSDelay;
        if (pDevCfg->eDeviceConfig == AM_HAL_MSPI_FLASH_OCTAL_DDR_CE0 || pDevCfg->eDeviceConfig == AM_HAL_MSPI_FLASH_OCTAL_DDR_CE1)
        {
            scanCfg.ui8Turnaround     = is25wx064_sConfigArray[i].ui8Turnaround;
        }
        else
        {
            if (is25wx064_sConfigArray[i].ui8Turnaround % 2)
            {
                continue;
            }
            scanCfg.ui8Turnaround     = is25wx064_sConfigArray[i].ui8Turnaround / 2;
            if (pDevCfg->eDeviceConfig == AM_HAL_MSPI_FLASH_OCTAL_CE0_1_8_8 || pDevCfg->eDeviceConfig == AM_HAL_MSPI_FLASH_OCTAL_CE1_1_8_8)
            {
                //
                // Device configured to a fixed turnaround 16.
                //
                if (scanCfg.ui8Turnaround != 16)
                {
                    continue;
                }
            }
            else
            {
                scanCfg.ui8Turnaround /= 2;
                //
                // Device configured to a fixed turnaround 8.
                //
                if (scanCfg.ui8Turnaround != 8)
                {
                    continue;
                }
            }
        }


        for ( uint8_t RxDqs_Index = 0; RxDqs_Index <= 31; RxDqs_Index++ )
        {
            //
            // set RXDQSDELAY0 value
            //
            scanCfg.ui8RxDQSDelay   = RxDqs_Index;
            //
            // apply settings
            //
            ui32Status = am_hal_mspi_control(pHandle, AM_HAL_MSPI_REQ_TIMING_SCAN, &scanCfg);
            if (AM_HAL_STATUS_SUCCESS != ui32Status)
            {
                return AM_DEVICES_MSPI_IS25WX064_STATUS_ERROR;
            }
            //
            // run data check
            //
            if ( false == flash_check(pDevHandle, FLASH_TIMING_SCAN_SIZE_BYTES) )
            {
                //
                // data check pass
                //
                ui32ResultArray[i] |= 0x01 << RxDqs_Index;
            }
            else
            {
                //
                // data check failed
                //
            }
        }
    }

    am_util_debug_printf("\n");

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
                             is25wx064_sConfigArray[i].bTxNeg, is25wx064_sConfigArray[i].bRxNeg, is25wx064_sConfigArray[i].bRxCap,
                             is25wx064_sConfigArray[i].ui8TxDQSDelay,
                             is25wx064_sConfigArray[i].ui8Turnaround, ui32ResultArray[i]);
    }

    am_util_debug_printf("\nTiming Scan found a window %d fine steps wide in setting %d.\n", ui32MaxOnes, ui32MaxOnesIndex);
    am_util_debug_printf(" TxNeg %d, RxNeg %d, RxCap %d, TxDQSDelay %d, Turnaround %d == 0x%08X\n",
                             is25wx064_sConfigArray[ui32MaxOnesIndex].bTxNeg, is25wx064_sConfigArray[ui32MaxOnesIndex].bRxNeg, is25wx064_sConfigArray[ui32MaxOnesIndex].bRxCap,
                             is25wx064_sConfigArray[ui32MaxOnesIndex].ui8TxDQSDelay,
                             is25wx064_sConfigArray[ui32MaxOnesIndex].ui8Turnaround, ui32ResultArray[ui32MaxOnesIndex]);

    //
    // Find RXDQSDELAY Value
    //
    uint32_t dqsdelay = find_mid_point(&ui32ResultArray[ui32MaxOnesIndex]);
    am_util_debug_printf("RxDQSDelay is set to %d.\n\n", dqsdelay);

    //
    // Deinitialize the MSPI interface
    //
    am_devices_mspi_is25wx064_deinit(pDevHandle);
    NVIC_ClearPendingIRQ(mspi_interrupts[module]);

    //
    // Check consecutive passing settings
    //
    if ( ui32MaxOnes < IS25WX064_TIMING_SCAN_MIN_ACCEPTANCE_LENGTH )
    {
        //
        // too short is the passing settings, use default setting
        //
        psDevTimingCfg->bTxNeg                = sTimingConfigDefault.bTxNeg;
        psDevTimingCfg->bRxNeg                = sTimingConfigDefault.bRxNeg;
        psDevTimingCfg->bRxCap                = sTimingConfigDefault.bRxCap;
        psDevTimingCfg->ui8TxDQSDelay         = sTimingConfigDefault.ui8TxDQSDelay;
        psDevTimingCfg->ui8RxDQSDelay         = sTimingConfigDefault.ui8RxDQSDelay;
        if (pDevCfg->eDeviceConfig == AM_HAL_MSPI_FLASH_OCTAL_DDR_CE0 || pDevCfg->eDeviceConfig == AM_HAL_MSPI_FLASH_OCTAL_DDR_CE1)
        {
            psDevTimingCfg->ui8Turnaround     = sTimingConfigDefault.ui8Turnaround;
        }
        else if (pDevCfg->eDeviceConfig == AM_HAL_MSPI_FLASH_OCTAL_CE0_1_8_8 || pDevCfg->eDeviceConfig == AM_HAL_MSPI_FLASH_OCTAL_CE1_1_8_8)
        {
            psDevTimingCfg->ui8Turnaround     = sTimingConfigDefault.ui8Turnaround / 2;
        }
        else
        {
            psDevTimingCfg->ui8Turnaround     = sTimingConfigDefault.ui8Turnaround / 4;
        }

        bTimingConfigSaved                    = true;
        sTimingConfigStored.bTxNeg            = psDevTimingCfg->bTxNeg;
        sTimingConfigStored.bRxNeg            = psDevTimingCfg->bRxNeg;
        sTimingConfigStored.bRxCap            = psDevTimingCfg->bRxCap;
        sTimingConfigStored.ui8TxDQSDelay     = psDevTimingCfg->ui8TxDQSDelay;
        sTimingConfigStored.ui8RxDQSDelay     = psDevTimingCfg->ui8RxDQSDelay;
        sTimingConfigStored.ui8Turnaround     = psDevTimingCfg->ui8Turnaround;

        return AM_DEVICES_MSPI_IS25WX064_STATUS_ERROR;
    }
    else
    {
        //
        // Set output values
        //
        psDevTimingCfg->ui8RxDQSDelay         = dqsdelay;
        psDevTimingCfg->bTxNeg                = is25wx064_sConfigArray[ui32MaxOnesIndex].bTxNeg;
        psDevTimingCfg->bRxNeg                = is25wx064_sConfigArray[ui32MaxOnesIndex].bRxNeg;
        psDevTimingCfg->bRxCap                = is25wx064_sConfigArray[ui32MaxOnesIndex].bRxCap;
        psDevTimingCfg->ui8TxDQSDelay         = is25wx064_sConfigArray[ui32MaxOnesIndex].ui8TxDQSDelay;
        psDevTimingCfg->ui8Turnaround         = is25wx064_sConfigArray[ui32MaxOnesIndex].ui8Turnaround;
        if ( pDevCfg->eDeviceConfig == AM_HAL_MSPI_FLASH_OCTAL_DDR_CE0 || pDevCfg->eDeviceConfig == AM_HAL_MSPI_FLASH_OCTAL_DDR_CE1 )
        {
            psDevTimingCfg->ui8Turnaround     = is25wx064_sConfigArray[ui32MaxOnesIndex].ui8Turnaround;
        }
        else if (pDevCfg->eDeviceConfig == AM_HAL_MSPI_FLASH_OCTAL_CE0_1_8_8 || pDevCfg->eDeviceConfig == AM_HAL_MSPI_FLASH_OCTAL_CE1_1_8_8)
        {
            psDevTimingCfg->ui8Turnaround     = is25wx064_sConfigArray[ui32MaxOnesIndex].ui8Turnaround / 2;
        }
        else
        {
            psDevTimingCfg->ui8Turnaround     = is25wx064_sConfigArray[ui32MaxOnesIndex].ui8Turnaround / 4;
        }

        bTimingConfigSaved                    = true;
        sTimingConfigStored.bTxNeg            = psDevTimingCfg->bTxNeg;
        sTimingConfigStored.bRxNeg            = psDevTimingCfg->bRxNeg;
        sTimingConfigStored.bRxCap            = psDevTimingCfg->bRxCap;
        sTimingConfigStored.ui8TxDQSDelay     = psDevTimingCfg->ui8TxDQSDelay;
        sTimingConfigStored.ui8RxDQSDelay     = psDevTimingCfg->ui8RxDQSDelay;
        sTimingConfigStored.ui8Turnaround     = psDevTimingCfg->ui8Turnaround;

        return AM_DEVICES_MSPI_IS25WX064_STATUS_SUCCESS;
    }
}

//*****************************************************************************
//
// Apply given SDR timing settings to target MSPI instance.
//
//*****************************************************************************
uint32_t
am_devices_mspi_is25wx064_apply_ddr_timing(void *pHandle,
                                           am_devices_mspi_is25wx064_timing_config_t *psDevTimingCfg)
{
    am_devices_mspi_is25wx064_t *pFlash = (am_devices_mspi_is25wx064_t *)pHandle;
    am_hal_mspi_timing_scan_t applyCfg =
    {
        .bTxNeg            = 1,
        .bRxNeg            = 0,
        .bRxCap            = 1,
        .ui8Turnaround     = 31,
        .ui8TxDQSDelay     = 1,
        .ui8RxDQSDelay     = 14,
    };

    if ( pFlash->stSetting.eClockFreq == AM_HAL_MSPI_CLK_96MHZ )
    {
        applyCfg.bTxNeg        = true;
        applyCfg.bRxNeg        = true;
        applyCfg.bRxCap        = false;
        applyCfg.ui8Turnaround = 31;
        applyCfg.ui8TxDQSDelay = 5;
        applyCfg.ui8RxDQSDelay = 14;
    }
    else if ( pFlash->stSetting.eClockFreq == AM_HAL_MSPI_CLK_48MHZ )
    {
        applyCfg.bTxNeg        = true;
        applyCfg.bRxNeg        = false;
        applyCfg.bRxCap        = true;
        applyCfg.ui8Turnaround = 31;
        applyCfg.ui8TxDQSDelay = 1;
        applyCfg.ui8RxDQSDelay = 14;
    }
    else
    {
        applyCfg.bTxNeg        = true;
        applyCfg.bRxNeg        = false;
        applyCfg.bRxCap        = true;
        applyCfg.ui8Turnaround = 31;
        applyCfg.ui8TxDQSDelay = 1;
        applyCfg.ui8RxDQSDelay = 14;
    }
    //
    // save a local copy of the timing settings
    //
    sTimingConfigStored.bTxNeg            = psDevTimingCfg->bTxNeg;
    sTimingConfigStored.bRxNeg            = psDevTimingCfg->bRxNeg;
    sTimingConfigStored.bRxCap            = psDevTimingCfg->bRxCap;
    sTimingConfigStored.ui8TxDQSDelay     = psDevTimingCfg->ui8TxDQSDelay;
    sTimingConfigStored.ui8RxDQSDelay     = psDevTimingCfg->ui8RxDQSDelay;
    sTimingConfigStored.ui8Turnaround     = psDevTimingCfg->ui8Turnaround;
    bTimingConfigSaved = true;

    applyCfg.bTxNeg                         = psDevTimingCfg->bTxNeg;
    applyCfg.bRxNeg                         = psDevTimingCfg->bRxNeg;
    applyCfg.bRxCap                         = psDevTimingCfg->bRxCap;
    applyCfg.ui8TxDQSDelay                  = psDevTimingCfg->ui8TxDQSDelay;
    applyCfg.ui8RxDQSDelay                  = psDevTimingCfg->ui8RxDQSDelay;
    applyCfg.ui8Turnaround                  = psDevTimingCfg->ui8Turnaround;


    return am_hal_mspi_control(pFlash->pMspiHandle, AM_HAL_MSPI_REQ_TIMING_SCAN, &applyCfg);

}
#endif

#endif
//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
