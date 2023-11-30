//*****************************************************************************
//
//! @file am_devices_mspi_psram_aps6404l.c
//!
//! @brief Micron Serial SPI PSRAM driver.
//!
//! @addtogroup mspi_psram_aps6404l APS6404L MSPI PSRAM Driver
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
#include "am_devices_mspi_psram_aps6404l.h"
#include "am_util_stdio.h"
#include "am_bsp.h"
#include "am_util.h"

//*****************************************************************************
//
// Global variables.
//
//*****************************************************************************
//#define APS6404L_QUAD_CLKON4_MODE_EN

#define AM_DEVICES_MSPI_PSRAM_TIMEOUT             1000000
#define PSRAM_TIMING_SCAN_MIN_ACCEPTANCE_LENGTH   (8)     // there should be at least
                                                          // this amount of consecutive
                                                          // passing settings to be accepted.

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
#if defined(APS6404L_QUAD_CLKON4_MODE_EN)
  .bClkonD4             = 1
#else
  .bClkonD4             = 0
#endif
};
#endif

#if defined(AM_PART_APOLLO4) || defined(AM_PART_APOLLO4B)
am_hal_mspi_dqs_t gSDREnableFineDelayCfg =
{
    .bDQSEnable             = 0,
    .bEnableFineDelay       = 1,
    .bOverrideRXDQSDelay    = 1,
    .ui8RxDQSDelay          = 15,
    .bOverrideTXDQSDelay    = 0,
    .ui8TxDQSDelay          = 0,
    .bDQSSyncNeg            = 0,
    .ui8DQSDelay            = 0,
    .ui8PioTurnaround       = 7,
    .ui8XipTurnaround       = 7,
    .bRxNeg                 = 0,
};
#endif


#if defined(AM_PART_APOLLO4P) || defined(AM_PART_APOLLO4L)
am_hal_mspi_xip_misc_t gXipMiscCfg[] =
{
  {
    .ui32CEBreak        = 10,
    .bXIPBoundary       = true,
    .bXIPOdd            = false,
    .bAppndOdd          = false,
    .bBEOn              = false,
    .eBEPolarity        = AM_HAL_MSPI_BE_LOW_ENABLE,
  },
  {
    .ui32CEBreak        = 10,
    .bXIPBoundary       = true,
    .bXIPOdd            = false,
    .bAppndOdd          = false,
    .bBEOn              = false,
    .eBEPolarity        = AM_HAL_MSPI_BE_LOW_ENABLE,
  },
  {
    .ui32CEBreak        = 10,
    .bXIPBoundary       = true,
    .bXIPOdd            = false,
    .bAppndOdd          = false,
    .bBEOn              = false,
    .eBEPolarity        = AM_HAL_MSPI_BE_LOW_ENABLE,
  },
};
#endif

am_hal_mspi_dev_config_t  SerialCE0MSPIConfig =
{
  .ui8TurnAround        = 8,
  .eAddrCfg             = AM_HAL_MSPI_ADDR_3_BYTE,
  .eInstrCfg            = AM_HAL_MSPI_INSTR_1_BYTE,
#if defined(AM_PART_APOLLO4_API)
  .ui16ReadInstr         = AM_DEVICES_MSPI_PSRAM_FAST_READ,
  .ui16WriteInstr        = AM_DEVICES_MSPI_PSRAM_WRITE,
#else
  .ui8ReadInstr         = AM_DEVICES_MSPI_PSRAM_FAST_READ,
  .ui8WriteInstr        = AM_DEVICES_MSPI_PSRAM_WRITE,
#endif
  .eDeviceConfig        = AM_HAL_MSPI_FLASH_SERIAL_CE0,
  .eSpiMode             = AM_HAL_MSPI_SPI_MODE_0,
  .eClockFreq           = AM_HAL_MSPI_CLK_24MHZ,
  .bSendAddr            = true,
  .bSendInstr           = true,
  .bTurnaround          = true,
#if defined(AM_PART_APOLLO3P)
  .ui8WriteLatency      = 0,
  .bEnWriteLatency      = false,
  .bEmulateDDR          = false,
  .ui16DMATimeLimit     = 80,
  .eDMABoundary         = AM_HAL_MSPI_BOUNDARY_BREAK1K,
#elif defined(AM_PART_APOLLO4_API)
  .ui8WriteLatency      = 0,
  .bEnWriteLatency      = false,
  .bEmulateDDR          = false,
  .ui16DMATimeLimit     = 80,
  .eDMABoundary         = AM_HAL_MSPI_BOUNDARY_BREAK1K,
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

am_hal_mspi_dev_config_t  SerialCE1MSPIConfig =
{
  .ui8TurnAround        = 8,
  .eAddrCfg             = AM_HAL_MSPI_ADDR_3_BYTE,
  .eInstrCfg            = AM_HAL_MSPI_INSTR_1_BYTE,
#if defined(AM_PART_APOLLO4_API)
  .ui16ReadInstr         = AM_DEVICES_MSPI_PSRAM_FAST_READ,
  .ui16WriteInstr        = AM_DEVICES_MSPI_PSRAM_WRITE,
#else
  .ui8ReadInstr         = AM_DEVICES_MSPI_PSRAM_FAST_READ,
  .ui8WriteInstr        = AM_DEVICES_MSPI_PSRAM_WRITE,
#endif
  .eDeviceConfig        = AM_HAL_MSPI_FLASH_SERIAL_CE1,
  .eSpiMode             = AM_HAL_MSPI_SPI_MODE_0,
  .eClockFreq           = AM_HAL_MSPI_CLK_16MHZ,
  .bSendAddr            = true,
  .bSendInstr           = true,
  .bTurnaround          = true,
#if defined(AM_PART_APOLLO3P)
  .ui8WriteLatency      = 0,
  .bEnWriteLatency      = false,
  .bEmulateDDR          = false,
  .ui16DMATimeLimit     = 80,
  .eDMABoundary         = AM_HAL_MSPI_BOUNDARY_BREAK1K,
#elif defined(AM_PART_APOLLO4_API)
  .ui8WriteLatency      = 0,
  .bEnWriteLatency      = false,
  .bEmulateDDR          = false,
  .ui16DMATimeLimit     = 80,
  .eDMABoundary         = AM_HAL_MSPI_BOUNDARY_BREAK1K,
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

am_hal_mspi_dev_config_t  QuadCE0MSPIConfig =
{
  .ui8TurnAround        = 7,
  .eAddrCfg             = AM_HAL_MSPI_ADDR_3_BYTE,
  .eInstrCfg            = AM_HAL_MSPI_INSTR_1_BYTE,
#if defined(AM_PART_APOLLO4_API)
  .ui16ReadInstr         = AM_DEVICES_MSPI_PSRAM_QUAD_READ,
  .ui16WriteInstr        = AM_DEVICES_MSPI_PSRAM_QUAD_WRITE,
#else
  .ui8ReadInstr         = AM_DEVICES_MSPI_PSRAM_QUAD_READ,
  .ui8WriteInstr        = AM_DEVICES_MSPI_PSRAM_QUAD_WRITE,
#endif
  .eDeviceConfig        = AM_HAL_MSPI_FLASH_QUAD_CE0,
  .eSpiMode             = AM_HAL_MSPI_SPI_MODE_0,
  .eClockFreq           = AM_HAL_MSPI_CLK_24MHZ,
  .bSendAddr            = true,
  .bSendInstr           = true,
  .bTurnaround          = true,
#if defined(AM_PART_APOLLO3P)
  .ui8WriteLatency      = 0,
  .bEnWriteLatency      = false,
  .bEmulateDDR          = false,
  .ui16DMATimeLimit     = 30,
  .eDMABoundary         = AM_HAL_MSPI_BOUNDARY_BREAK1K,
#elif defined(AM_PART_APOLLO4_API)
  .ui8WriteLatency      = 0,
  .bEnWriteLatency      = false,
  .bEmulateDDR          = false,
  .ui16DMATimeLimit     = 70,
  .eDMABoundary         = AM_HAL_MSPI_BOUNDARY_BREAK1K,
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

am_hal_mspi_dev_config_t  QuadCE1MSPIConfig =
{
  .ui8TurnAround        = 7,
  .eAddrCfg             = AM_HAL_MSPI_ADDR_3_BYTE,
  .eInstrCfg            = AM_HAL_MSPI_INSTR_1_BYTE,
#if defined(AM_PART_APOLLO4_API)
  .ui16ReadInstr         = AM_DEVICES_MSPI_PSRAM_QUAD_READ,
  .ui16WriteInstr        = AM_DEVICES_MSPI_PSRAM_QUAD_WRITE,
#else
  .ui8ReadInstr         = AM_DEVICES_MSPI_PSRAM_QUAD_READ,
  .ui8WriteInstr        = AM_DEVICES_MSPI_PSRAM_QUAD_WRITE,
#endif
  .eDeviceConfig        = AM_HAL_MSPI_FLASH_QUAD_CE1,
  .eSpiMode             = AM_HAL_MSPI_SPI_MODE_0,
  .eClockFreq           = AM_HAL_MSPI_CLK_16MHZ,
  .bSendAddr            = true,
  .bSendInstr           = true,
  .bTurnaround          = true,
#if defined(AM_PART_APOLLO3P)
  .ui8WriteLatency      = 0,
  .bEnWriteLatency      = false,
  .bEmulateDDR          = false,
  .ui16DMATimeLimit     = 30,
  .eDMABoundary         = AM_HAL_MSPI_BOUNDARY_BREAK1K,
#elif defined(AM_PART_APOLLO4_API)
  .ui8WriteLatency      = 0,
  .bEnWriteLatency      = false,
  .bEmulateDDR          = false,
  .ui16DMATimeLimit     = 70,
  .eDMABoundary         = AM_HAL_MSPI_BOUNDARY_BREAK1K,
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

#if defined(AM_PART_APOLLO4) || defined(AM_PART_APOLLO4B)
//
// SDR timing default setting, scan starts from this setting
//
am_devices_mspi_psram_sdr_timing_config_t SDRTimingConfigDefault =
{
    .ui32Turnaround     = 7,
    .ui32Rxneg          = 1,
    .ui32Rxdqsdelay     = 12
};
#endif

typedef struct
{
  uint32_t                    ui32Module;
  void                        *pMspiHandle;
  am_hal_mspi_device_e        eDeviceConfig;
#if defined(AM_PART_APOLLO3)
  uint32_t                    maxTransSize;
#endif
  bool                        bOccupied;
} am_devices_mspi_psram_t;

am_devices_mspi_psram_t gAmPsram[AM_DEVICES_MSPI_PSRAM_MAX_DEVICE_NUM];

#if defined(AM_PART_APOLLO3)
// Hardware does not support transaction splitting - so need to take care in driver
const struct
{
  const uint32_t MHz;
  const uint32_t MaxSize;
} g_Mspi_SpeedMax[] =
{
  {AM_HAL_MSPI_CLK_48MHZ, AM_DEVICES_MSPI_PSRAM_48MHZ_MAX_BYTES},
  {AM_HAL_MSPI_CLK_24MHZ, AM_DEVICES_MSPI_PSRAM_24MHZ_MAX_BYTES},
  {AM_HAL_MSPI_CLK_16MHZ, AM_DEVICES_MSPI_PSRAM_16MHZ_MAX_BYTES},
  {AM_HAL_MSPI_CLK_12MHZ, AM_DEVICES_MSPI_PSRAM_12MHZ_MAX_BYTES},
  {AM_HAL_MSPI_CLK_8MHZ,  AM_DEVICES_MSPI_PSRAM_8MHZ_MAX_BYTES}  // Leave this in for PSRAM initialization at 8MHz.
};
#endif

void pfnMSPI_PSRAM_Callback(void *pCallbackCtxt, uint32_t status)
{
  // Set the DMA complete flag.
  *(volatile bool *)pCallbackCtxt = true;
}

//*****************************************************************************
//
// Generic Command Write function.
//
//*****************************************************************************
static uint32_t
am_device_command_write(void *pMspiHandle,
                        uint8_t ui8Instr,
                        bool bSendAddr,
                        uint32_t ui32Addr,
                        uint32_t *pData,
                        uint32_t ui32NumBytes)
{
  am_hal_mspi_pio_transfer_t  Transaction;

  // Create the individual write transaction.
  Transaction.ui32NumBytes            = ui32NumBytes;
  Transaction.bScrambling             = false;
  Transaction.eDirection              = AM_HAL_MSPI_TX;
  Transaction.bSendAddr               = bSendAddr;
  Transaction.ui32DeviceAddr          = ui32Addr;
  Transaction.bSendInstr              = true;
  Transaction.ui16DeviceInstr         = ui8Instr;
  Transaction.bTurnaround             = false;
#if defined(AM_PART_APOLLO3P)
  Transaction.bDCX                    = false;
  Transaction.bEnWRLatency            = false;
  Transaction.bContinue               = false;  // MSPI CONT is deprecated for Apollo3
#endif
#if !defined(AM_PART_APOLLO4) && !defined(AM_PART_APOLLO4B) && !defined(AM_PART_APOLLO4P) && !defined(AM_PART_APOLLO4L)
  Transaction.bQuadCmd                = false;
#else
  Transaction.bDCX                    = false;
  Transaction.bEnWRLatency            = false;
  Transaction.bContinue               = false;  // MSPI CONT is deprecated for Apollo4
#if defined(AM_PART_APOLLO4)
  Transaction.eDeviceNum              = AM_HAL_MSPI_DEVICE0;
#endif
#endif
  Transaction.pui32Buffer             = pData;

  // Execute the transction over MSPI.
  return am_hal_mspi_blocking_transfer(pMspiHandle,
                                       &Transaction,
                                       AM_DEVICES_MSPI_PSRAM_TIMEOUT);
}

//*****************************************************************************
//
// Generic Command Read function.
//
//*****************************************************************************
static uint32_t
am_device_command_read(void *pMspiHandle,
                       uint8_t ui8Instr,
                       bool bSendAddr,
                       uint32_t ui32Addr,
                       uint32_t *pData,
                       uint32_t ui32NumBytes)
{
  am_hal_mspi_pio_transfer_t  Transaction;

  // Create the individual write transaction.
  Transaction.ui32NumBytes            = ui32NumBytes;
  Transaction.bScrambling             = false;
  Transaction.eDirection              = AM_HAL_MSPI_RX;
  Transaction.bSendAddr               = bSendAddr;
  Transaction.ui32DeviceAddr          = ui32Addr;
  Transaction.bSendInstr              = true;
  Transaction.ui16DeviceInstr         = ui8Instr;
  Transaction.bTurnaround             = false;
#if defined(AM_PART_APOLLO3P)
  Transaction.bDCX                    = false;
  Transaction.bEnWRLatency            = false;
  Transaction.bContinue               = false;  // MSPI CONT is deprecated for Apollo3
#endif
#if !defined(AM_PART_APOLLO4) && !defined(AM_PART_APOLLO4B) && !defined(AM_PART_APOLLO4P) && !defined(AM_PART_APOLLO4L)
  Transaction.bQuadCmd                = false;
#else
  Transaction.bDCX                    = false;
  Transaction.bEnWRLatency            = false;
  Transaction.bContinue               = false;  // MSPI CONT is deprecated for Apollo4
#if defined(AM_PART_APOLLO4)
  Transaction.eDeviceNum              = AM_HAL_MSPI_DEVICE0;
#endif
#endif
  Transaction.pui32Buffer             = pData;

  // Execute the transction over MSPI.
  return am_hal_mspi_blocking_transfer(pMspiHandle,
                                       &Transaction,
                                       AM_DEVICES_MSPI_PSRAM_TIMEOUT);
}

//*****************************************************************************
//
// Reset the external psram
//
//*****************************************************************************
static uint32_t
am_devices_mspi_psram_aps6404l_reset(void *pMspiHandle)
{
  uint32_t      ui32PIOBuffer;
  //
  // Send the command sequence to reset the device and return status.
  //
  if (AM_HAL_STATUS_SUCCESS != am_device_command_write(pMspiHandle, AM_DEVICES_MSPI_PSRAM_RESET_ENABLE, false, 0, &ui32PIOBuffer, 0))
  {
    return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
  }
  if (AM_HAL_STATUS_SUCCESS != am_device_command_write(pMspiHandle, AM_DEVICES_MSPI_PSRAM_RESET_MEMORY, false, 0, &ui32PIOBuffer, 0))
  {
    return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
  }
  return AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Reads the ID of the external psram and returns the value.
//!
//! @param pDeviceID - Pointer to the return buffer for the Device ID.
//!
//! This function reads the device ID register of the external psram, and returns
//! the result as an 32-bit unsigned integer value.
//!
//! @return 32-bit status
//
//*****************************************************************************
static uint32_t
am_devices_mspi_psram_aps6404l_id(void *pMspiHandle)
{
  uint32_t      ui32Status;
  uint32_t aui32Rawdata[2] = {0};
  uint32_t      ui32DeviceID = 0;

  //
  // Send the command sequence to read the Device ID and return status.
  //
  ui32Status = am_device_command_read(pMspiHandle, AM_DEVICES_MSPI_PSRAM_READ_ID, false, 0, aui32Rawdata, 5);
  ui32DeviceID = ((aui32Rawdata[0] & 0xFF000000) >> 24) | ((aui32Rawdata[1] & 0xFF) << 8);
  am_util_stdio_printf("PSRAM ID is 0x%x\n", ui32DeviceID);
  if ((AM_DEVICES_MSPI_PSRAM_KGD_PASS == ui32DeviceID) &&
      (AM_HAL_STATUS_SUCCESS == ui32Status))
  {
    return AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS;
  }
  else
  {
    return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
  }
}

// This function takes care of splitting the transaction as needed, if the transaction crosses
// PSRAM page boundary or because of tCEM restrictions, if hardware does not support it
static uint32_t
psram_nonblocking_transfer(am_devices_mspi_psram_t *pPsram,
                           bool bHiPrio,
                           bool bWrite,
                           uint8_t *pui8Buffer,
                           uint32_t ui32Address,
                           uint32_t ui32NumBytes,
                           uint32_t ui32PauseCondition,
                           uint32_t ui32StatusSetClr,
                           am_hal_mspi_callback_t pfnCallback,
                           void *pCallbackCtxt)
{
  uint32_t ui32Status = AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS;
  am_hal_mspi_dma_transfer_t    Transaction;

  // Set the DMA priority
  Transaction.ui8Priority = 1;


  // Set the transfer direction to RX (Read)
  Transaction.eDirection = bWrite ? AM_HAL_MSPI_TX: AM_HAL_MSPI_RX;


  // Initialize the CQ stimulus.
  Transaction.ui32PauseCondition = ui32PauseCondition;
  // Initialize the post-processing
  Transaction.ui32StatusSetClr = 0;

  // Need to be aware of page size
  while (ui32NumBytes)
  {
#if defined(AM_PART_APOLLO3)
    uint32_t maxSize = AM_DEVICES_MSPI_PSRAM_PAGE_SIZE - (ui32Address & (AM_DEVICES_MSPI_PSRAM_PAGE_SIZE - 1));
    uint32_t limit = (maxSize > pPsram->maxTransSize) ? pPsram->maxTransSize : maxSize;
    uint32_t size = (ui32NumBytes > limit) ? limit : ui32NumBytes;
#else
    uint32_t size;
    if ((ui32Address & 0x3) &&
        ((AM_DEVICES_MSPI_PSRAM_PAGE_SIZE - (ui32Address & (AM_DEVICES_MSPI_PSRAM_PAGE_SIZE - 1))) < ui32NumBytes))
    {
      // Hardware does not support Page splitting if address is not word aligned
      // Need to split the transaction
      size = 4 - (ui32Address & 0x3);
    }
    else
    {
      size = ui32NumBytes;
    }
#endif
    bool bLast = (size == ui32NumBytes);
    // Set the transfer count in bytes.
    Transaction.ui32TransferCount = size;

    // Set the address to read data from.
    Transaction.ui32DeviceAddress = ui32Address;

    // Set the target SRAM buffer address.
    Transaction.ui32SRAMAddress = (uint32_t)pui8Buffer;

    if (bLast)
    {
      Transaction.ui32StatusSetClr = ui32StatusSetClr;
    }
#if defined(AM_PART_APOLLO4)
  Transaction.eDeviceNum              = AM_HAL_MSPI_DEVICE0;
#endif

    if (bHiPrio)
    {
      ui32Status = am_hal_mspi_highprio_transfer(pPsram->pMspiHandle, &Transaction, AM_HAL_MSPI_TRANS_DMA,
                                                 bLast ? pfnCallback : NULL,
                                                 bLast ? pCallbackCtxt : NULL);
    }
    else
    {
      ui32Status = am_hal_mspi_nonblocking_transfer(pPsram->pMspiHandle, &Transaction, AM_HAL_MSPI_TRANS_DMA,
                                                    bLast ? pfnCallback : NULL,
                                                    bLast ? pCallbackCtxt : NULL);
    }
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
      break;
    }
    ui32Address += size;
    ui32NumBytes -= size;
    pui8Buffer += size;

    Transaction.ui32PauseCondition = 0;
  }
  return ui32Status;
}


//*****************************************************************************
//
// Initialize the mspi_psram driver.
//
//*****************************************************************************
uint32_t
am_devices_mspi_psram_init(uint32_t ui32Module,
                           am_devices_mspi_psram_config_t *pDevCfg,
                           void **ppHandle,
                           void **ppMspiHandle)
{
    uint32_t                    ui32Status;
    uint32_t                    ui32PIOBuffer;
    am_hal_mspi_dev_config_t    *psMSPISettings1;
    am_hal_mspi_dev_config_t    *psMSPISettings0;
    am_hal_mspi_dev_config_t    mspiDevCfg;
    am_hal_mspi_dev_config_t    tempDevCfg;
    void                        *pMspiHandle;
    uint32_t                    ui32Index = 0;

    if ((ui32Module > AM_REG_MSPI_NUM_MODULES) || (pDevCfg == NULL))
    {
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
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

    // Allocate a vacant device handle
    for ( ui32Index = 0; ui32Index < AM_DEVICES_MSPI_PSRAM_MAX_DEVICE_NUM; ui32Index++ )
    {
        if ( gAmPsram[ui32Index].bOccupied == false )
        {
            break;
        }
    }
    if ( ui32Index == AM_DEVICES_MSPI_PSRAM_MAX_DEVICE_NUM)
    {
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

    //
    // Re-Configure the MSPI for the requested operation mode.
    //
    switch (pDevCfg->eDeviceConfig)
    {
        case AM_HAL_MSPI_FLASH_SERIAL_CE0:
            mspiDevCfg = SerialCE0MSPIConfig;
            break;
        case AM_HAL_MSPI_FLASH_SERIAL_CE1:
            mspiDevCfg = SerialCE1MSPIConfig;
            break;
        case AM_HAL_MSPI_FLASH_QUAD_CE0:
        case AM_HAL_MSPI_FLASH_QUAD_CE0_1_4_4:
            mspiDevCfg = QuadCE0MSPIConfig;
            break;
        case AM_HAL_MSPI_FLASH_QUAD_CE1:
        case AM_HAL_MSPI_FLASH_QUAD_CE1_1_4_4:
            mspiDevCfg = QuadCE1MSPIConfig;
            break;
        default:
            return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }
    mspiDevCfg.eDeviceConfig = pDevCfg->eDeviceConfig;
    mspiDevCfg.eClockFreq = pDevCfg->eClockFreq;
#if !defined(AM_PART_APOLLO4) && !defined(AM_PART_APOLLO4B) && !defined(AM_PART_APOLLO4P) && !defined(AM_PART_APOLLO4L)
    mspiDevCfg.ui32TCBSize = pDevCfg->ui32NBTxnBufLength;
    mspiDevCfg.pTCB = pDevCfg->pNBTxnBuf;
    mspiDevCfg.scramblingStartAddr = pDevCfg->ui32ScramblingStartAddr;
    mspiDevCfg.scramblingEndAddr = pDevCfg->ui32ScramblingEndAddr;
#endif

#if defined(AM_PART_APOLLO3)
    //
    // Look up the Max Transaction size to fit into 8usec for CE asserted
    //
    gAmPsram[ui32Index].maxTransSize = 0;
    for (uint32_t i = 0; i < (sizeof(g_Mspi_SpeedMax) / sizeof(g_Mspi_SpeedMax[0])); i++)
    {
        if (g_Mspi_SpeedMax[i].MHz == mspiDevCfg.eClockFreq)
        {
            gAmPsram[ui32Index].maxTransSize = g_Mspi_SpeedMax[i].MaxSize;
            break;
        }
    }
    if ( 0 == gAmPsram[ui32Index].maxTransSize ) // Return an error if Max Transaction size not found.
    {
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }
#endif

    //
    // Configure the MSPI for Serial or Quad operation during initialization.
    //
    switch (mspiDevCfg.eDeviceConfig)
    {
        case AM_HAL_MSPI_FLASH_SERIAL_CE0:
        case AM_HAL_MSPI_FLASH_QUAD_CE0:
        case AM_HAL_MSPI_FLASH_QUAD_CE0_1_4_4:
            psMSPISettings0 = &QuadCE0MSPIConfig;
            psMSPISettings1 = &SerialCE0MSPIConfig;
            break;
        case AM_HAL_MSPI_FLASH_SERIAL_CE1:
        case AM_HAL_MSPI_FLASH_QUAD_CE1:
        case AM_HAL_MSPI_FLASH_QUAD_CE1_1_4_4:
            psMSPISettings0 = &QuadCE1MSPIConfig;
            psMSPISettings1 = &SerialCE1MSPIConfig;
            break;
        default:
            return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
            //break;
    }
    psMSPISettings0->eDeviceConfig = pDevCfg->eDeviceConfig;

    // Adjust TURNAROUND for highest speed clock settings by 1.
    tempDevCfg = *psMSPISettings0;
#if defined(AM_PART_APOLLO4) || defined(AM_PART_APOLLO4B)
    if (AM_HAL_MSPI_CLK_96MHZ == psMSPISettings0->eClockFreq)
    {
      tempDevCfg.ui8TurnAround++;
    }
#endif

    // First configure in Quad mode and reset
    if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_initialize(ui32Module, &pMspiHandle))
    {
        am_util_stdio_printf("Error - Failed to initialize MSPI.\n");
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

    if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_power_control(pMspiHandle, AM_HAL_SYSCTRL_WAKE, false))
    {
        am_util_stdio_printf("Error - Failed to power on MSPI.\n");
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

#if defined(AM_PART_APOLLO4_API)
    am_hal_mspi_config_t    mspiCfg = gMspiCfg;
    mspiCfg.ui32TCBSize = pDevCfg->ui32NBTxnBufLength;
    mspiCfg.pTCB = pDevCfg->pNBTxnBuf;
    if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_configure(pMspiHandle, &mspiCfg))
    {
        am_util_stdio_printf("Error - Failed to configure MSPI device.\n");
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }
#endif

    if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_device_configure(pMspiHandle, &tempDevCfg))
    {
        am_util_stdio_printf("Error - Failed to configure MSPI device.\n");
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

#if defined(AM_PART_APOLLO4_API)
    am_hal_mspi_xip_config_t    xipCfg = gXipConfig[ui32Module];
#if defined(AM_PART_APOLLO4)
    xipCfg.eDeviceNum = psMSPISettings0->eDeviceNum;
#endif
    xipCfg.scramblingStartAddr = pDevCfg->ui32ScramblingStartAddr;
    xipCfg.scramblingEndAddr = pDevCfg->ui32ScramblingEndAddr;
    ui32Status = am_hal_mspi_control(pMspiHandle, AM_HAL_MSPI_REQ_XIP_CONFIG, &xipCfg);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
      return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }
#endif
#if defined(AM_PART_APOLLO4P) || defined(AM_PART_APOLLO4L)
    am_hal_mspi_xip_misc_t    xipMiscCfg = gXipMiscCfg[ui32Module];
    ui32Status = am_hal_mspi_control(pMspiHandle, AM_HAL_MSPI_REQ_XIP_MISC_CONFIG, &xipMiscCfg);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
      return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }
#endif

    if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_enable(pMspiHandle))
    {
        am_util_stdio_printf("Error - Failed to enable MSPI.\n");
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }
#if defined(APS6404L_QUAD_CLKON4_MODE_EN)
    am_bsp_mspi_clkond4_pins_enable(ui32Module, psMSPISettings0->eDeviceConfig);
#else
    am_bsp_mspi_pins_enable(ui32Module, psMSPISettings0->eDeviceConfig);
#endif
    am_util_delay_us(150);

    if (AM_HAL_STATUS_SUCCESS != am_devices_mspi_psram_aps6404l_reset(pMspiHandle))
    {
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

    // Revert to Serial mode
    switch (mspiDevCfg.eDeviceConfig)
    {
        case AM_HAL_MSPI_FLASH_SERIAL_CE0:
        case AM_HAL_MSPI_FLASH_SERIAL_CE1:
            // Nothing to do.  Device defaults to SPI mode.
            break;
        case AM_HAL_MSPI_FLASH_QUAD_CE0:
        case AM_HAL_MSPI_FLASH_QUAD_CE0_1_4_4:
        case AM_HAL_MSPI_FLASH_QUAD_CE1:
        case AM_HAL_MSPI_FLASH_QUAD_CE1_1_4_4:
            ui32Status = am_device_command_write(pMspiHandle, AM_DEVICES_MSPI_PSRAM_QUAD_MODE_EXIT, false, 0, &ui32PIOBuffer, 0);
            if (AM_HAL_STATUS_SUCCESS != ui32Status)
            {
                return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
            }
            break;
        default:
            return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
            //break;
    }

    // Disable MSPI defore re-configuring it
    ui32Status = am_hal_mspi_disable(pMspiHandle);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

    // Adjust TURNAROUND for highest speed clock settings by 1.
    tempDevCfg = *psMSPISettings1;
#if defined(AM_PART_APOLLO4) || defined(AM_PART_APOLLO4B)
    if (AM_HAL_MSPI_CLK_96MHZ == psMSPISettings1->eClockFreq)
    {
      tempDevCfg.ui8TurnAround++;
    }
#endif

    if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_device_configure(pMspiHandle, &tempDevCfg))
    {
        am_util_stdio_printf("Error - Failed to configure MSPI.\n");
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

    if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_enable(pMspiHandle))
    {
        am_util_stdio_printf("Error - Failed to enable MSPI.\n");
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }
//    am_bsp_mspi_pins_enable(ui32Module, psMSPISettings1->eDeviceConfig);

    am_util_delay_us(150);

    if (AM_HAL_STATUS_SUCCESS != am_devices_mspi_psram_aps6404l_reset(pMspiHandle))
    {
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

    //
    // Device specific MSPI psram initialization.
    //
    ui32Status = am_devices_mspi_psram_aps6404l_id(pMspiHandle);

    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

    //
    // Configure the APS6404L Device mode.
    //
    switch (mspiDevCfg.eDeviceConfig)
    {
        case AM_HAL_MSPI_FLASH_SERIAL_CE0:
        case AM_HAL_MSPI_FLASH_SERIAL_CE1:
            // Nothing to do.  Device defaults to SPI mode.
            break;
        case AM_HAL_MSPI_FLASH_QUAD_CE0:
        case AM_HAL_MSPI_FLASH_QUAD_CE0_1_4_4:
        case AM_HAL_MSPI_FLASH_QUAD_CE1:
        case AM_HAL_MSPI_FLASH_QUAD_CE1_1_4_4:
            ui32Status = am_device_command_write(pMspiHandle, AM_DEVICES_MSPI_PSRAM_QUAD_MODE_ENTER, false, 0, &ui32PIOBuffer, 0);
            if (AM_HAL_STATUS_SUCCESS != ui32Status)
            {
                return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
            }
            break;
        default:
            return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

    // Disable MSPI defore re-configuring it
    ui32Status = am_hal_mspi_disable(pMspiHandle);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

    // Adjust TURNAROUND for highest speed clock settings by 1.
    tempDevCfg = mspiDevCfg;
#if defined(AM_PART_APOLLO4) || defined(AM_PART_APOLLO4B)
    if (AM_HAL_MSPI_CLK_96MHZ == mspiDevCfg.eClockFreq)
    {
      tempDevCfg.ui8TurnAround++;
    }
#endif

    ui32Status = am_hal_mspi_device_configure(pMspiHandle, &tempDevCfg);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

#if defined(AM_PART_APOLLO4) || defined(AM_PART_APOLLO4B)
    //
    // Apply SDR timing config
    //
    ui32Status = am_hal_mspi_control(pMspiHandle, AM_HAL_MSPI_REQ_DQS, &gSDREnableFineDelayCfg);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }
#endif


    // Re-Enable MSPI
    ui32Status = am_hal_mspi_enable(pMspiHandle);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

    //
    // Configure the MSPI pins.
    //
#if defined(APS6404L_QUAD_CLKON4_MODE_EN)
    am_bsp_mspi_clkond4_pins_enable(ui32Module, mspiDevCfg.eDeviceConfig);
#else
    am_bsp_mspi_pins_enable(ui32Module, mspiDevCfg.eDeviceConfig);
#endif


    //
    // Enable MSPI interrupts.
    //

    ui32Status = am_hal_mspi_interrupt_clear(pMspiHandle, AM_HAL_MSPI_INT_CQUPD | AM_HAL_MSPI_INT_ERR );
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

    ui32Status = am_hal_mspi_interrupt_enable(pMspiHandle, AM_HAL_MSPI_INT_CQUPD | AM_HAL_MSPI_INT_ERR );
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

    //
    // Return the handle.
    //
    gAmPsram[ui32Index].bOccupied = true;
    *ppHandle = (void *)&gAmPsram[ui32Index];
    *ppMspiHandle = gAmPsram[ui32Index].pMspiHandle = pMspiHandle;
    gAmPsram[ui32Index].ui32Module = ui32Module;
    gAmPsram[ui32Index].eDeviceConfig = mspiDevCfg.eDeviceConfig;

    //
    // Return the status.
    //
    return AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS;
}

//*****************************************************************************
//
// DeInitialize the mspi_psram driver.
//
//*****************************************************************************
uint32_t
am_devices_mspi_psram_deinit(void *pHandle)
{
    uint32_t    ui32Status;
    uint32_t    ui32PIOBuffer;
    am_devices_mspi_psram_t *pPsram = (am_devices_mspi_psram_t *)pHandle;

    //
    // Device specific MSPI psram initialization.
    //
    // Revert to Serial mode
    switch (pPsram->eDeviceConfig)
    {
        case AM_HAL_MSPI_FLASH_SERIAL_CE0:
        case AM_HAL_MSPI_FLASH_SERIAL_CE1:
            // Nothing to do.  Device defaults to SPI mode.
            break;
        case AM_HAL_MSPI_FLASH_QUAD_CE0:
        case AM_HAL_MSPI_FLASH_QUAD_CE0_1_4_4:
        case AM_HAL_MSPI_FLASH_QUAD_CE1:
        case AM_HAL_MSPI_FLASH_QUAD_CE1_1_4_4:
            ui32Status = am_device_command_write(pPsram->pMspiHandle, AM_DEVICES_MSPI_PSRAM_QUAD_MODE_EXIT, false, 0, &ui32PIOBuffer, 0);
            if (AM_HAL_STATUS_SUCCESS != ui32Status)
            {
                return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
            }
            break;
        default:
            return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
        //break;
    }

    //
    // Disable and clear the interrupts to start with.
    //
    ui32Status = am_hal_mspi_interrupt_disable(pPsram->pMspiHandle, 0xFFFFFFFF);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }
    ui32Status = am_hal_mspi_interrupt_clear(pPsram->pMspiHandle, 0xFFFFFFFF);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

    //
    // Disable MSPI instance.
    //
    ui32Status = am_hal_mspi_disable(pPsram->pMspiHandle);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }
    //
    // Disable power to the MSPI instance.
    //
    if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_power_control(pPsram->pMspiHandle, AM_HAL_SYSCTRL_DEEPSLEEP, false))
    {
        am_util_stdio_printf("Error - Failed to power on MSPI.\n");
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }
    //
    // Deinitialize the MPSI instance.
    //
    ui32Status = am_hal_mspi_deinitialize(pPsram->pMspiHandle);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

    // Free this device handle
    pPsram->bOccupied = false;

    //
    // Return the status.
    //
    return AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Reads the contents of the external PSRAM into a buffer.
//
//*****************************************************************************
uint32_t
am_devices_mspi_psram_read(void *pHandle,
                           uint8_t *pui8RxBuffer,
                           uint32_t ui32ReadAddress,
                           uint32_t ui32NumBytes,
                           bool bWaitForCompletion)
{
  uint32_t                      ui32Status;
  am_devices_mspi_psram_t *pPsram = (am_devices_mspi_psram_t *)pHandle;

  if (bWaitForCompletion)
  {
    // Start the transaction.
    volatile bool bDMAComplete = false;
    ui32Status = psram_nonblocking_transfer(pPsram, false, false,
                                            pui8RxBuffer,
                                            ui32ReadAddress,
                                            ui32NumBytes,
                                            0,
                                            0,
                                            pfnMSPI_PSRAM_Callback,
                                            (void *)&bDMAComplete);

    // Check the transaction status.
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
      return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

    // Wait for DMA Complete or Timeout
    for (uint32_t i = 0; i < AM_DEVICES_MSPI_PSRAM_TIMEOUT; i++)
    {
      if (bDMAComplete)
      {
        break;
      }
      //
      // Call the BOOTROM cycle function to delay for about 1 microsecond.
      //
#if !defined(AM_PART_APOLLO4) && !defined(AM_PART_APOLLO4B) && !defined(AM_PART_APOLLO4P) && !defined(AM_PART_APOLLO4L)
      am_hal_flash_delay( FLASH_CYCLES_US(1) );
#else
      am_hal_delay_us(1);
#endif
    }

    // Check the status.
    if (!bDMAComplete)
    {
      return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }
  }
  else
  {
    // Check the transaction status.
    ui32Status = psram_nonblocking_transfer(pPsram, false, false,
                                            pui8RxBuffer,
                                            ui32ReadAddress,
                                            ui32NumBytes,
                                            0,
                                            0,
                                            NULL,
                                            NULL);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
      return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }
  }
  //
  // Return the status.
  //
  return AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Reads the contents of the external PSRAM into a buffer.
//
//*****************************************************************************
uint32_t
am_devices_mspi_psram_read_adv(void *pHandle,
                               uint8_t *pui8RxBuffer,
                               uint32_t ui32ReadAddress,
                               uint32_t ui32NumBytes,
                               uint32_t ui32PauseCondition,
                               uint32_t ui32StatusSetClr,
                               am_hal_mspi_callback_t pfnCallback,
                               void *pCallbackCtxt)
{
  uint32_t                      ui32Status;
  am_devices_mspi_psram_t *pPsram = (am_devices_mspi_psram_t *)pHandle;

  ui32Status = psram_nonblocking_transfer(pPsram, false, false,
                                          pui8RxBuffer,
                                          ui32ReadAddress,
                                          ui32NumBytes,
                                          ui32PauseCondition,
                                          ui32StatusSetClr,
                                          pfnCallback,
                                          pCallbackCtxt);

  // Check the transaction status.
  if (AM_HAL_STATUS_SUCCESS != ui32Status)
  {
    return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
  }

  //
  // Return the status.
  //
  return AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS;
}

//*****************************************************************************
//
//  Reads the contents of the external psram into a buffer.
//
//*****************************************************************************
uint32_t
am_devices_mspi_psram_read_hiprio(void *pHandle,
                                  uint8_t *pui8RxBuffer,
                                  uint32_t ui32ReadAddress,
                                  uint32_t ui32NumBytes,
                                  am_hal_mspi_callback_t pfnCallback,
                                  void *pCallbackCtxt)
{
  uint32_t                      ui32Status;
  am_devices_mspi_psram_t *pPsram = (am_devices_mspi_psram_t *)pHandle;

  ui32Status = psram_nonblocking_transfer(pPsram, true, false,
                                          pui8RxBuffer,
                                          ui32ReadAddress,
                                          ui32NumBytes,
                                          0,
                                          0,
                                          pfnCallback,
                                          pCallbackCtxt);

  // Check the transaction status.
  if (AM_HAL_STATUS_SUCCESS != ui32Status)
  {
    return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
  }

  //
  // Return the status.
  //
  return AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS;
}

//*****************************************************************************
//
// nonblocking read
//
//*****************************************************************************
uint32_t
am_devices_mspi_psram_nonblocking_read(void *pHandle,
                                       uint8_t *pui8RxBuffer,
                                       uint32_t ui32ReadAddress,
                                       uint32_t ui32NumBytes,
                                       am_hal_mspi_callback_t pfnCallback,
                                       void *pCallbackCtxt)
{
  uint32_t                      ui32Status;
  am_devices_mspi_psram_t *pPsram = (am_devices_mspi_psram_t *)pHandle;

  // Check the transaction status.
  ui32Status = psram_nonblocking_transfer(pPsram, false, false,
                                          pui8RxBuffer,
                                          ui32ReadAddress,
                                          ui32NumBytes,
                                          0,
                                          0,
                                          pfnCallback,
                                          pCallbackCtxt);
  if (AM_HAL_STATUS_SUCCESS != ui32Status)
  {
    return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
  }

  //
  // Return the status.
  //
  return AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS;
}


//*****************************************************************************
//
//  Programs the given range of psram addresses.
//
//*****************************************************************************
uint32_t
am_devices_mspi_psram_write(void *pHandle,
                            uint8_t *pui8TxBuffer,
                            uint32_t ui32WriteAddress,
                            uint32_t ui32NumBytes,
                            bool bWaitForCompletion)
{
  uint32_t                      ui32Status;
  am_devices_mspi_psram_t *pPsram = (am_devices_mspi_psram_t *)pHandle;

  if (bWaitForCompletion)
  {
    // Start the transaction.
    volatile bool bDMAComplete = false;
    ui32Status = psram_nonblocking_transfer(pPsram, false, true,
                                            pui8TxBuffer,
                                            ui32WriteAddress,
                                            ui32NumBytes,
                                            0,
                                            0,
                                            pfnMSPI_PSRAM_Callback,
                                            (void *)&bDMAComplete);

    // Check the transaction status.
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
      return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

    // Wait for DMA Complete or Timeout
    for (uint32_t i = 0; i < AM_DEVICES_MSPI_PSRAM_TIMEOUT; i++)
    {
      if (bDMAComplete)
      {
        break;
      }
      //
      // Call the BOOTROM cycle function to delay for about 1 microsecond.
      //
#if defined(AM_PART_APOLLO4_API)
      am_hal_delay_us(1);
#else
      am_hal_flash_delay( FLASH_CYCLES_US(1) );
#endif
    }

    // Check the status.
    if (!bDMAComplete)
    {
      return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }
  }
  else
  {
    // Check the transaction status.
    ui32Status = psram_nonblocking_transfer(pPsram, false, true,
                                            pui8TxBuffer,
                                            ui32WriteAddress,
                                            ui32NumBytes,
                                            0,
                                            0,
                                            NULL,
                                            NULL);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
      return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }
  }

  //
  // Return the status.
  //
  return AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS;
}

//*****************************************************************************
//
//  Programs the given range of psram addresses.
//
//*****************************************************************************
uint32_t
am_devices_mspi_psram_write_adv(void *pHandle,
                                uint8_t *puiTxBuffer,
                                uint32_t ui32WriteAddress,
                                uint32_t ui32NumBytes,
                                uint32_t ui32PauseCondition,
                                uint32_t ui32StatusSetClr,
                                am_hal_mspi_callback_t pfnCallback,
                                void *pCallbackCtxt)
{
  uint32_t                      ui32Status;
  am_devices_mspi_psram_t *pPsram = (am_devices_mspi_psram_t *)pHandle;

  ui32Status = psram_nonblocking_transfer(pPsram, false, true,
                                          puiTxBuffer,
                                          ui32WriteAddress,
                                          ui32NumBytes,
                                          ui32PauseCondition,
                                          ui32StatusSetClr,
                                          pfnCallback,
                                          pCallbackCtxt);

  // Check the transaction status.
  if (AM_HAL_STATUS_SUCCESS != ui32Status)
  {
    return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
  }

  //
  // Return the status.
  //
  return AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS;
}

//*****************************************************************************
//
//  Programs the given range of psram addresses.
//
//*****************************************************************************
uint32_t
am_devices_mspi_psram_write_hiprio(void *pHandle,
                                   uint8_t *pui8TxBuffer,
                                   uint32_t ui32WriteAddress,
                                   uint32_t ui32NumBytes,
                                   am_hal_mspi_callback_t pfnCallback,
                                   void *pCallbackCtxt)
{
  uint32_t                      ui32Status;
  am_devices_mspi_psram_t *pPsram = (am_devices_mspi_psram_t *)pHandle;

  // Check the transaction status.
  ui32Status = psram_nonblocking_transfer(pPsram, true, true,
                                          pui8TxBuffer,
                                          ui32WriteAddress,
                                          ui32NumBytes,
                                          0,
                                          0,
                                          pfnCallback,
                                          pCallbackCtxt);

  // Check the transaction status.
  if (AM_HAL_STATUS_SUCCESS != ui32Status)
  {
    return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
  }

  //
  // Return the status.
  //
  return AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS;
}

//*****************************************************************************
//
//Programs the given range of psram addresses.
//
//*****************************************************************************
uint32_t
am_devices_mspi_psram_nonblocking_write(void *pHandle,
                                        uint8_t *pui8TxBuffer,
                                        uint32_t ui32WriteAddress,
                                        uint32_t ui32NumBytes,
                                        am_hal_mspi_callback_t pfnCallback,
                                        void *pCallbackCtxt)
{
  uint32_t                      ui32Status;
  am_devices_mspi_psram_t *pPsram = (am_devices_mspi_psram_t *)pHandle;

  // Check the transaction status.
  ui32Status = psram_nonblocking_transfer(pPsram, false, true,
                                          pui8TxBuffer,
                                          ui32WriteAddress,
                                          ui32NumBytes,
                                          0,
                                          0,
                                          pfnCallback,
                                          pCallbackCtxt);
  if (AM_HAL_STATUS_SUCCESS != ui32Status)
  {
    return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
  }

  //
  // Return the status.
  //
  return AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS;
}


//*****************************************************************************
//
// Sets up the MSPI and external psram into XIP mode.
//
//*****************************************************************************
uint32_t
am_devices_mspi_psram_enable_xip(void *pHandle)
{
  uint32_t ui32Status;
  am_devices_mspi_psram_t *pPsram = (am_devices_mspi_psram_t *)pHandle;

#if defined(AM_PART_APOLLO4_API)
  //
  // Set Aperture XIP range
  //
  ui32Status = am_hal_mspi_control(pPsram->pMspiHandle, AM_HAL_MSPI_REQ_XIP_CONFIG, &gXipConfig[pPsram->ui32Module]);
  if (AM_HAL_STATUS_SUCCESS != ui32Status)
  {
    return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
  }
#endif

  //
  // Enable XIP on the MSPI.
  //
#if defined(AM_PART_APOLLO4_API)
  ui32Status = am_hal_mspi_control(pPsram->pMspiHandle, AM_HAL_MSPI_REQ_XIP_EN, &gXipConfig[pPsram->ui32Module]);
#else
  ui32Status = am_hal_mspi_control(pPsram->pMspiHandle, AM_HAL_MSPI_REQ_XIP_EN, NULL);
#endif

  if (AM_HAL_STATUS_SUCCESS != ui32Status)
  {
    return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
  }

  return AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Removes the MSPI and external psram from XIP mode.
//
//*****************************************************************************
uint32_t
am_devices_mspi_psram_disable_xip(void *pHandle)
{
  uint32_t ui32Status;
  am_devices_mspi_psram_t *pPsram = (am_devices_mspi_psram_t *)pHandle;

  //
  // Disable XIP on the MSPI.
  //
#if defined(AM_PART_APOLLO4_API)
  ui32Status = am_hal_mspi_control(pPsram->pMspiHandle, AM_HAL_MSPI_REQ_XIP_DIS, &gXipConfig[pPsram->ui32Module]);
#else
  ui32Status = am_hal_mspi_control(pPsram->pMspiHandle, AM_HAL_MSPI_REQ_XIP_DIS, NULL);
#endif
  if (AM_HAL_STATUS_SUCCESS != ui32Status)
  {
    return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
  }

  return AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Sets up the MSPI and external psram into scrambling mode.
//
//*****************************************************************************
uint32_t
am_devices_mspi_psram_enable_scrambling(void *pHandle)
{
  uint32_t ui32Status;
  am_devices_mspi_psram_t *pPsram = (am_devices_mspi_psram_t *)pHandle;

  //
  // Enable scrambling on the MSPI.
  //
#if defined(AM_PART_APOLLO4_API)
  ui32Status = am_hal_mspi_control(pPsram->pMspiHandle, AM_HAL_MSPI_REQ_SCRAMB_EN, &gXipConfig[pPsram->ui32Module]);
#else
  ui32Status = am_hal_mspi_control(pPsram->pMspiHandle, AM_HAL_MSPI_REQ_SCRAMB_EN, NULL);
#endif
  if (AM_HAL_STATUS_SUCCESS != ui32Status)
  {
    return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
  }
  return AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS;
}

//*****************************************************************************
//
//  Removes the MSPI and external psram from scrambling mode.
//
//*****************************************************************************
uint32_t
am_devices_mspi_psram_disable_scrambling(void *pHandle)
{
  uint32_t ui32Status;
  am_devices_mspi_psram_t *pPsram = (am_devices_mspi_psram_t *)pHandle;

  //
  // Disable Scrambling on the MSPI.
  //
#if defined(AM_PART_APOLLO4_API)
  ui32Status = am_hal_mspi_control(pPsram->pMspiHandle, AM_HAL_MSPI_REQ_SCRAMB_DIS, &gXipConfig[pPsram->ui32Module]);
#else
  ui32Status = am_hal_mspi_control(pPsram->pMspiHandle, AM_HAL_MSPI_REQ_SCRAMB_DIS, NULL);
#endif
  if (AM_HAL_STATUS_SUCCESS != ui32Status)
  {
    return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
  }

  return AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Reset the external psram
//
//*****************************************************************************
uint32_t
am_devices_mspi_psram_reset(void *pHandle)
{
  am_devices_mspi_psram_t *pPsram = (am_devices_mspi_psram_t *)pHandle;
  return am_devices_mspi_psram_aps6404l_reset(pPsram);
}

//*****************************************************************************
//
// Reads the ID of the external psram and returns the value.
//
//*****************************************************************************
uint32_t
am_devices_mspi_psram_id(void *pHandle)
{
  am_devices_mspi_psram_t *pPsram = (am_devices_mspi_psram_t *)pHandle;

  return am_devices_mspi_psram_aps6404l_id(pPsram);
}


#if defined(AM_PART_APOLLO4) || defined(AM_PART_APOLLO4B)
//*****************************************************************************
//
//! @brief write and read back check.
//!
//! @param pattern_index -
//! @param buff   -
//! @param len    -
//!
//! @note This function should be called before any other am_devices_mspi_psram
//! functions. It is used to set tell the other functions how to communicate
//! with the external psram hardware.
//!
//! @return status.
//
//*****************************************************************************
#define PSRAM_CHECK_DATA_SIZE_BYTES  256
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
//*****************************************************************************
//
//! @brief
//! @param length
//! @param address
//! @return
//
//*****************************************************************************
bool
psram_check(uint32_t length, uint32_t address)
{
    // Try to use as less ram as possible in stack
    uint32_t ui32NumberOfBytesLeft = length;
    uint32_t ui32TestBytes = 0;
    uint32_t ui32AddressOffset = 0;
    uint8_t ui8PatternCounter = 0;
    uint8_t ui8TxBuffer[PSRAM_CHECK_DATA_SIZE_BYTES];
    uint8_t ui8RxBuffer[PSRAM_CHECK_DATA_SIZE_BYTES];

    while ( ui32NumberOfBytesLeft )
    {
        if ( ui32NumberOfBytesLeft > PSRAM_CHECK_DATA_SIZE_BYTES )
        {
            ui32TestBytes = PSRAM_CHECK_DATA_SIZE_BYTES;
            ui32NumberOfBytesLeft -= PSRAM_CHECK_DATA_SIZE_BYTES;
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

        prepare_test_pattern((ui8PatternCounter) % 5, ui8TxBuffer, ui32TestBytes);
        ui8PatternCounter++;

        // write to target address
        am_hal_sysctrl_bus_write_flush();
        uint8_t * xipPointer = (uint8_t *)(address + ui32AddressOffset);
        memcpy(xipPointer, (uint8_t*)ui8TxBuffer, ui32TestBytes);

        //
        // Read back data
        //
        am_hal_sysctrl_bus_write_flush();
        xipPointer = (uint8_t *)(address + ui32AddressOffset);
        memcpy((uint8_t*)ui8RxBuffer, xipPointer, ui32TestBytes);

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

//*****************************************************************************
//
//! @brief  Count the longest consecutive 1s in a 32bit word
//! @details Static helper function:
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
//! @brief  Find and return the mid point of the longest continuous 1s in a 32bit word
//! @details Static helper function:
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
                if ( pattern_len > max_len )
                {
                    max_len = pattern_len;
                    pick_point = i - 1 - pattern_len / 2;
                    remainder = pattern_len % 2;
                }
                pattern_len = 0;
            }
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

#define PSRAM_TIMING_SCAN_SIZE_BYTES (128*1024)
static const uint32_t ui32MspiXipBaseAddress[3] =
{
    0x14000000, // mspi0
    0x18000000, // mspi1
    0x1C000000, // mspi2
};

const am_devices_mspi_psram_sdr_timing_config_t sConfigArray[] =
{
//    {1 , 0, 1}, // Turnaround=1 , RXNEG=0, RXDQSDELAY=Dummy
//    {1 , 1, 1}, // Turnaround=1 , RXNEG=1, RXDQSDELAY=Dummy
//    {2 , 0, 1}, // Turnaround=2 , RXNEG=0, RXDQSDELAY=Dummy
//    {2 , 1, 1}, // Turnaround=2 , RXNEG=1, RXDQSDELAY=Dummy
//    {3 , 0, 1}, // Turnaround=3 , RXNEG=0, RXDQSDELAY=Dummy
//    {3 , 1, 1}, // Turnaround=3 , RXNEG=1, RXDQSDELAY=Dummy
    {4 , 0, 1}, // Turnaround=4 , RXNEG=0, RXDQSDELAY=Dummy
    {4 , 1, 1}, // Turnaround=4 , RXNEG=1, RXDQSDELAY=Dummy
    {5 , 0, 1}, // Turnaround=5 , RXNEG=0, RXDQSDELAY=Dummy
    {5 , 1, 1}, // Turnaround=5 , RXNEG=1, RXDQSDELAY=Dummy
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
//    {11, 0, 1}, // Turnaround=11, RXNEG=0, RXDQSDELAY=Dummy
//    {11, 1, 1}, // Turnaround=11, RXNEG=1, RXDQSDELAY=Dummy
//    {12, 0, 1}, // Turnaround=12, RXNEG=0, RXDQSDELAY=Dummy
//    {12, 1, 1}, // Turnaround=12, RXNEG=1, RXDQSDELAY=Dummy
//    {13, 0, 1}, // Turnaround=13, RXNEG=0, RXDQSDELAY=Dummy
//    {13, 1, 1}, // Turnaround=13, RXNEG=1, RXDQSDELAY=Dummy
//    {14, 0, 1}, // Turnaround=14, RXNEG=0, RXDQSDELAY=Dummy
//    {14, 1, 1}, // Turnaround=14, RXNEG=1, RXDQSDELAY=Dummy
//    {15, 0, 1}, // Turnaround=15, RXNEG=0, RXDQSDELAY=Dummy
//    {15, 1, 1}, // Turnaround=15, RXNEG=1, RXDQSDELAY=Dummy
};
//*****************************************************************************
//
//  Checks PSRAM timing and determine a delay setting.
//
//*****************************************************************************
uint32_t
am_devices_mspi_psram_sdr_init_timing_check(uint32_t module,
                                            am_devices_mspi_psram_config_t *pDevCfg,
                                            am_devices_mspi_psram_sdr_timing_config_t *pDevSdrCfg)
{
    uint32_t ui32Status;
    void *pDevHandle;
    void *pHandle;

    uint32_t ui32ResultArray[sizeof(sConfigArray) / sizeof(am_devices_mspi_psram_sdr_timing_config_t)];
    const uint32_t ui32TestSize = sizeof(sConfigArray) / sizeof(am_devices_mspi_psram_sdr_timing_config_t);

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
        .ui8PioTurnaround       = 7,
        .ui8XipTurnaround       = 7,
        .bRxNeg                 = 0,
    };

    //
    // initialize interface
    //
    am_hal_mspi_dev_config_t    *psMSPISettings;
    switch (pDevCfg->eDeviceConfig)
    {
        case AM_HAL_MSPI_FLASH_SERIAL_CE0:
        case AM_HAL_MSPI_FLASH_QUAD_CE0:
        case AM_HAL_MSPI_FLASH_QUAD_CE0_1_4_4:
            psMSPISettings = &QuadCE0MSPIConfig;
            break;
        case AM_HAL_MSPI_FLASH_SERIAL_CE1:
        case AM_HAL_MSPI_FLASH_QUAD_CE1:
        case AM_HAL_MSPI_FLASH_QUAD_CE1_1_4_4:
            psMSPISettings = &QuadCE1MSPIConfig;
            break;
        default:
            return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
            //break;
    }

    uint16_t timeLimit = psMSPISettings->ui16DMATimeLimit;                  // save original setting here
    am_hal_mspi_dma_boundary_e dmaBound0 = psMSPISettings->eDMABoundary;    // save original setting here
    psMSPISettings->ui16DMATimeLimit    = 0;
    psMSPISettings->eDMABoundary        = AM_HAL_MSPI_BOUNDARY_NONE;
    ui32Status = am_devices_mspi_psram_init(module, pDevCfg, &pDevHandle, &pHandle);
    if (AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS != ui32Status)
    {
        am_util_debug_printf("    Failed to configure the MSPI and PSRAM Device correctly!\n");
        return ui32Status;
    }
    psMSPISettings->ui16DMATimeLimit = timeLimit;   // restore original setting here
    psMSPISettings->eDMABoundary = dmaBound0;       // restore original setting here

    //
    // Put the MSPI into XIP mode.
    //
    ui32Status = am_devices_mspi_psram_enable_xip(pDevHandle);
    if (AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS != ui32Status)
    {
        am_util_debug_printf("    Failed to disable XIP mode in the MSPI!\n");
        return ui32Status;
    }

    //
    // Start scan loop
    //
    for ( uint8_t i = 0; i < ui32TestSize; i++ )
    {
        // set Turnaround and RXNEG
        scanCfg.ui8PioTurnaround    = scanCfg.ui8XipTurnaround = sConfigArray[i].ui32Turnaround;
        scanCfg.bRxNeg              = sConfigArray[i].ui32Rxneg;
        for ( uint8_t RxDqs_Index = 1; RxDqs_Index < 31; RxDqs_Index++ )
        {
            // set RXDQSDELAY0 value
            scanCfg.ui8RxDQSDelay   = RxDqs_Index;
            // apply settings
            ui32Status = am_hal_mspi_control(pHandle, AM_HAL_MSPI_REQ_DQS, &scanCfg);
            if (AM_HAL_STATUS_SUCCESS != ui32Status)
            {
                return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
            }

            // run data check
            if ( false == psram_check(PSRAM_TIMING_SCAN_SIZE_BYTES, ui32MspiXipBaseAddress[module] + RxDqs_Index) )
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
        am_util_debug_printf(" Turnaround %d - RXNEG %d = 0x%08X\n", sConfigArray[i].ui32Turnaround, sConfigArray[i].ui32Rxneg, ui32ResultArray[i]);
    }
#if defined(AM_DEBUG_PRINTF)
    am_util_debug_printf("    Max length = %d \n", ui32MaxOnes);
#else
    am_util_stdio_printf("Timing Scan found a window %d fine steps wide.\n", ui32MaxOnes);
#endif

    //
    // Check consecutive passing settings
    //
    if ( ui32MaxOnes < PSRAM_TIMING_SCAN_MIN_ACCEPTANCE_LENGTH )
    {
        // too short is the passing settings
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

    //
    // Find RXDQSDELAY Value
    //
    uint32_t dqsdelay = find_mid_point(&ui32ResultArray[ui32MaxOnesIndex]);

#if !defined(AM_DEBUG_PRINTF)
    am_util_stdio_printf("Timing Scan set the fine delay to %d steps.\n", dqsdelay);
#endif


    //
    // Deinitialize the MSPI interface
    //
    am_devices_mspi_psram_deinit(pDevHandle);

    //
    // Set output values
    //
    pDevSdrCfg->ui32Rxdqsdelay = dqsdelay;
    pDevSdrCfg->ui32Rxneg = sConfigArray[ui32MaxOnesIndex].ui32Rxneg;
    pDevSdrCfg->ui32Turnaround = sConfigArray[ui32MaxOnesIndex].ui32Turnaround;

    return AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS;
}

//*****************************************************************************
//
//  Apply given SDR timing settings to target MSPI instance.
//
//*****************************************************************************
uint32_t
am_devices_mspi_psram_apply_sdr_timing(void *pHandle,
                                       am_devices_mspi_psram_sdr_timing_config_t *pDevSdrCfg)
{
    am_devices_mspi_psram_t *pPsram = (am_devices_mspi_psram_t *)pHandle;
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

    return am_hal_mspi_control(pPsram->pMspiHandle, AM_HAL_MSPI_REQ_DQS, &applyCfg);
}
#endif

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
