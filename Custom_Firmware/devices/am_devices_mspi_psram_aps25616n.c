//*****************************************************************************
//
//! @file am_devices_mspi_psram_aps25616n.c
//!
//! @brief APM DDR HEX and Octal SPI PSRAM driver.
//!
//! @addtogroup mspi_psram_aps25616n APS25616N MSPI PSRAM 1.8V Driver
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
#include "am_devices_mspi_psram_aps25616n.h"
#include "am_util_stdio.h"
#include "am_bsp.h"
#include "am_util.h"
#include "am_util_delay.h"

//*****************************************************************************
//
// Global variables.
//
//*****************************************************************************

//#define USE_NON_DQS_MODE

#define APS25616N_tHS_MIN_US        5   // with margin
#define APS25616N_tXHS_MIN_US       155 // with margin

#define AM_DEVICES_MSPI_PSRAM_TIMEOUT             1000000
#if defined(AM_PART_APOLLO4_API)
#define PSRAM_TIMING_SCAN_MIN_ACCEPTANCE_LENGTH   (8)     // there should be at least
                                                          // this amount of consecutive
                                                          // passing settings to be accepted.
#endif

am_hal_mspi_xip_config_t gAPMDDRXipConfig[] =
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

am_hal_mspi_dqs_t gAPMDDRDqsCfg[] =
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
#if defined(AM_PART_APOLLO4P)
    .ui8TxDQSDelay          = 12,
#else
    .ui8TxDQSDelay          = 0,
#endif
    .ui8RxDQSDelay          = 16,
    .ui8RxDQSDelayNeg       = 0,
    .bRxDQSDelayNegEN       = 0,
    .ui8RxDQSDelayHi        = 0,
    .ui8RxDQSDelayNegHi     = 0,
    .bRxDQSDelayHiEN        = 0,
  },
};

am_hal_mspi_xip_misc_t gAPMXipMiscCfg[] =
{
  {
    .ui32CEBreak        = 10,
    .bXIPBoundary       = true,
    .bXIPOdd            = true,
    .bAppndOdd          = false,
    .bBEOn              = false,
    .eBEPolarity        = AM_HAL_MSPI_BE_LOW_ENABLE,
  },
  {
    .ui32CEBreak        = 10,
    .bXIPBoundary       = true,
    .bXIPOdd            = true,
    .bAppndOdd          = false,
    .bBEOn              = false,
    .eBEPolarity        = AM_HAL_MSPI_BE_LOW_ENABLE,
  },
  {
    .ui32CEBreak        = 10,
    .bXIPBoundary       = true,
    .bXIPOdd            = true,
    .bAppndOdd          = false,
    .bBEOn              = false,
    .eBEPolarity        = AM_HAL_MSPI_BE_LOW_ENABLE,
  },
};

am_hal_mspi_config_t gAPMDDRMspiCfg =
{
  .ui32TCBSize          = 0,
  .pTCB                 = NULL,
  .bClkonD4             = 0
};

am_hal_mspi_rxcfg_t gAPMMspiRxCfg =
{
    .ui8DQSturn         = 2,
    .bRxHI              = 0,
    .bTaForth           = 1,
    .bHyperIO           = 0,
    .ui8RxSmp           = 1,
    .bRBX               = 0,
    .bWBX               = 0,
    .bSCLKRxHalt        = 0,
    .bRxCapEXT          = 0,
    .ui8Sfturn          = 10,
};

am_hal_mspi_dev_config_t  APMDDROctalCE0MSPIConfig =
{
  .eAddrCfg             = AM_HAL_MSPI_ADDR_4_BYTE,
  .eInstrCfg            = AM_HAL_MSPI_INSTR_2_BYTE,
  .ui16ReadInstr        = AM_DEVICES_MSPI_PSRAM_DDR_READ,
  .ui16WriteInstr       = AM_DEVICES_MSPI_PSRAM_DDR_WRITE,
  .eDeviceConfig        = AM_HAL_MSPI_FLASH_OCTAL_DDR_CE0,
  .eSpiMode             = AM_HAL_MSPI_SPI_MODE_0,
  .bSendAddr            = true,
  .bSendInstr           = true,
  .bTurnaround          = true,
  .eClockFreq           = AM_HAL_MSPI_CLK_96MHZ,
#ifdef USE_NON_DQS_MODE
  .ui8TurnAround        = 12,
  .ui8WriteLatency      = 6,
#else
  .ui8TurnAround        = 6,
  .ui8WriteLatency      = 6,
#endif
  .bEnWriteLatency      = true,
  .bEmulateDDR          = true,
  .ui16DMATimeLimit     = 20,
  .eDMABoundary         = AM_HAL_MSPI_BOUNDARY_BREAK1K,
};

am_hal_mspi_dev_config_t  APMDDROctalCE1MSPIConfig =
{
  .eAddrCfg             = AM_HAL_MSPI_ADDR_4_BYTE,
  .eInstrCfg            = AM_HAL_MSPI_INSTR_2_BYTE,
  .ui16ReadInstr        = AM_DEVICES_MSPI_PSRAM_DDR_READ,
  .ui16WriteInstr       = AM_DEVICES_MSPI_PSRAM_DDR_WRITE,
  .eDeviceConfig        = AM_HAL_MSPI_FLASH_OCTAL_DDR_CE1,
  .eSpiMode             = AM_HAL_MSPI_SPI_MODE_0,
  .bSendAddr            = true,
  .bSendInstr           = true,
  .bTurnaround          = true,
  .eClockFreq           = AM_HAL_MSPI_CLK_96MHZ,
#ifdef USE_NON_DQS_MODE
  .ui8TurnAround        = 12,
  .ui8WriteLatency      = 6,
#else
  .ui8TurnAround        = 6,
  .ui8WriteLatency      = 6,
#endif
  .bEnWriteLatency      = true,
  .bEmulateDDR          = true,
  .ui16DMATimeLimit     = 20,
  .eDMABoundary         = AM_HAL_MSPI_BOUNDARY_BREAK1K,
};

am_hal_mspi_dev_config_t  APMDDRHEXCE0MSPIConfig =
{
  .eAddrCfg             = AM_HAL_MSPI_ADDR_4_BYTE,
  .eInstrCfg            = AM_HAL_MSPI_INSTR_2_BYTE,
  .ui16ReadInstr        = AM_DEVICES_MSPI_PSRAM_DDR_READ,
  .ui16WriteInstr       = AM_DEVICES_MSPI_PSRAM_DDR_WRITE,
  .eDeviceConfig        = AM_HAL_MSPI_FLASH_HEX_DDR_CE0,
  .eSpiMode             = AM_HAL_MSPI_SPI_MODE_0,
  .bSendAddr            = true,
  .bSendInstr           = true,
  .bTurnaround          = true,
  .eClockFreq           = AM_HAL_MSPI_CLK_96MHZ,
#ifdef USE_NON_DQS_MODE
  .ui8TurnAround        = 12,
  .ui8WriteLatency      = 6,
#else
  .ui8TurnAround        = 6,
  .ui8WriteLatency      = 6,
#endif
  .bEnWriteLatency      = true,
  .bEmulateDDR          = true,
  .ui16DMATimeLimit     = 20,
  .eDMABoundary         = AM_HAL_MSPI_BOUNDARY_BREAK1K,
};

am_hal_mspi_dev_config_t  APMDDRHEXCE1MSPIConfig =
{
  .eAddrCfg             = AM_HAL_MSPI_ADDR_4_BYTE,
  .eInstrCfg            = AM_HAL_MSPI_INSTR_2_BYTE,
  .ui16ReadInstr        = AM_DEVICES_MSPI_PSRAM_DDR_READ,
  .ui16WriteInstr       = AM_DEVICES_MSPI_PSRAM_DDR_WRITE,
  .eDeviceConfig        = AM_HAL_MSPI_FLASH_HEX_DDR_CE1,
  .eSpiMode             = AM_HAL_MSPI_SPI_MODE_0,
  .bSendAddr            = true,
  .bSendInstr           = true,
  .bTurnaround          = true,
  .eClockFreq           = AM_HAL_MSPI_CLK_96MHZ,
#ifdef USE_NON_DQS_MODE
  .ui8TurnAround        = 12,
  .ui8WriteLatency      = 6,
#else
  .ui8TurnAround        = 6,
  .ui8WriteLatency      = 6,
#endif
  .bEnWriteLatency      = true,
  .bEmulateDDR          = true,
  .ui16DMATimeLimit     = 20,
  .eDMABoundary         = AM_HAL_MSPI_BOUNDARY_BREAK1K,
};

typedef struct
{
  uint32_t                      ui32Module;
  void                          *pMspiHandle;
  am_hal_mspi_device_e          eDeviceConfig;
  bool                          bOccupied;
  am_devices_mspi_psram_info_t  sDeviceInfo;
} am_devices_mspi_psram_t;

am_devices_mspi_psram_t gAPMDDRPsram[AM_DEVICES_MSPI_PSRAM_MAX_DEVICE_NUM];

void pfnMSPI_APMPSRAM_DDR_Callback(void *pCallbackCtxt, uint32_t status)
{
  // Set the DMA complete flag.
  *(volatile bool *)pCallbackCtxt = true;
}

//*****************************************************************************
//
//! @brief Generic Command Write function.
//!
//! @param pMspiHandle
//! @param ui16Instr
//! @param bSendAddr
//! @param ui32Addr
//! @param pData
//! @param ui32NumBytes
//!
//! @return
//
//*****************************************************************************
static uint32_t
am_device_command_write(void *pMspiHandle,
                        uint16_t ui16Instr,
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
  Transaction.ui16DeviceInstr         = ui16Instr;
  Transaction.bTurnaround             = false;
  Transaction.bDCX                    = false;
  Transaction.bEnWRLatency            = false;
  Transaction.bContinue               = false;  // MSPI CONT is deprecated for Apollo4
  Transaction.pui32Buffer             = pData;

  // Execute the transction over MSPI.
  return am_hal_mspi_blocking_transfer(pMspiHandle,
                                       &Transaction,
                                       AM_DEVICES_MSPI_PSRAM_TIMEOUT);
}

//*****************************************************************************
//
//! @brief Generic Command Read function.
//!
//! @param pMspiHandle
//! @param ui16Instr
//! @param bSendAddr
//! @param ui32Addr
//! @param pData
//! @param ui32NumBytes
//!
//! @return
//
//*****************************************************************************
static uint32_t
am_device_command_read(void *pMspiHandle,
                       uint16_t ui16Instr,
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
  Transaction.ui16DeviceInstr         = ui16Instr;
  Transaction.bTurnaround             = true;
  Transaction.bDCX                    = false;
  Transaction.bEnWRLatency            = true;
  Transaction.bContinue               = false;  // MSPI CONT is deprecated for Apollo4
  Transaction.pui32Buffer             = pData;

  // Execute the transction over MSPI.
  return am_hal_mspi_blocking_transfer(pMspiHandle,
                                       &Transaction,
                                       AM_DEVICES_MSPI_PSRAM_TIMEOUT);
}

//*****************************************************************************
//
//! @brief Reset the external psram
//!
//! @param pMspiHandle
//!
//! @return
//
//*****************************************************************************
static uint32_t
am_devices_mspi_psram_aps25616n_reset(void *pMspiHandle)
{
  uint32_t      ui32PIOBuffer = 0;
  //
  // Global Reset DDR PSRAM.
  //
  if (AM_HAL_STATUS_SUCCESS != am_device_command_write(pMspiHandle, AM_DEVICES_MSPI_PSRAM_DDR_GLOBAL_RESET, true, 0, &ui32PIOBuffer, 2))
  {
    return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
  }

  return AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Set PSRAM into HEX mode
//!
//! @param pMspiHandle
//!
//! @return
//
//*****************************************************************************
static uint32_t
am_devices_mspi_psram_aps25616n_enter_hex_mode(void *pMspiHandle)
{
  uint32_t     ui32Status;
  uint32_t     ui32Rawdata;
  uint8_t      ui8IOModeReg = 0;
  //
  // Read PSRAM MR8 register
  //
  am_util_debug_printf("Read PSRAM MR8 register\n");
  ui32Status = am_device_command_read(pMspiHandle, AM_DEVICES_MSPI_PSRAM_DDR_READ_REGISTER, true, 8, &ui32Rawdata, 4);
  if (AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS != ui32Status)
  {
      am_util_debug_printf("Failed to read PSRAM MR8 register!\n");
      return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
  }
  else
  {
      ui8IOModeReg = (uint8_t)ui32Rawdata;
      am_util_debug_printf("PSRAM Register MR8 = 0x%X\n", ui8IOModeReg);
      am_util_debug_printf("PSRAM IO mode = 0x%X\n\n", (ui8IOModeReg & 0x40)>>6 );
  }

  ui32Rawdata = ui8IOModeReg | 0x40;

  ui32Status = am_device_command_write(pMspiHandle, AM_DEVICES_MSPI_PSRAM_DDR_WRITE_REGISTER, true, 8, &ui32Rawdata, 4);
  if (AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS != ui32Status)
  {
      am_util_debug_printf("Failed to set PSRAM into HEX mode!\n");
      return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
  }
  else
  {
      am_util_debug_printf("Set PSRAM into HEX mode\n\n");
  }

  return AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Set PSRAM into Octal mode and exit HEX mode
//!
//! @param pMspiHandle
//!
//! @return
//
//*****************************************************************************
static uint32_t
am_devices_mspi_psram_aps25616n_exit_hex_mode(void *pMspiHandle)
{
  uint32_t     ui32Status;
  uint32_t     ui32Rawdata;
  uint8_t      ui8IOModeReg = 0;
  //
  // Read PSRAM MR8 register
  //
  am_util_debug_printf("Read PSRAM MR8 register\n");
  ui32Status = am_device_command_read(pMspiHandle, AM_DEVICES_MSPI_PSRAM_DDR_READ_REGISTER, true, 8, &ui32Rawdata, 4);
  if (AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS != ui32Status)
  {
      am_util_debug_printf("Failed to read PSRAM MR8 register!\n");
      return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
  }
  else
  {
      ui8IOModeReg = (uint8_t)ui32Rawdata;
      am_util_debug_printf("PSRAM Register MR8 = 0x%X\n", ui8IOModeReg);
      am_util_debug_printf("PSRAM IO mode = 0x%X\n\n", (ui8IOModeReg & 0x40) >> 6 );
  }

  ui32Rawdata = ui8IOModeReg & 0xBF;

  ui32Status = am_device_command_write(pMspiHandle, AM_DEVICES_MSPI_PSRAM_DDR_WRITE_REGISTER, true, 8, &ui32Rawdata, 4);
  if (AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS != ui32Status)
  {
      am_util_debug_printf("Failed to configure PSRAM to exit HEX mode!\n");
      return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
  }
  else
  {
      am_util_debug_printf("PSRAM is set to Octal mode\n\n");
  }

  return AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Configure the external psram and returns status.
//!
//! @param pMspiHandle - Pointer to MSPI instance handle.
//! @param pPsramInfo - Pointer to device info to be returned
//!
//! This function configure the write and read latency of the
//! psram and return status
//!
//! @return 32-bit status
//
//*****************************************************************************
static uint32_t
am_devices_mspi_psram_aps25616n_device_init(void *pMspiHandle, am_devices_mspi_psram_info_t *pPsramInfo)
{
  uint32_t     ui32Status;
  uint32_t     ui32Rawdata;
  uint8_t      ui8VendorIDReg = 0;
  uint8_t      ui8DeviceIDReg = 0;
  uint8_t      ui8RLCReg = 0;

  //
  // Read and set PSRAM Register MR0
  //
  am_util_debug_printf("Read PSRAM Register MR0\n");
  ui32Status = am_device_command_read(pMspiHandle, AM_DEVICES_MSPI_PSRAM_DDR_READ_REGISTER, true, 0, &ui32Rawdata, 4);
  if (AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS != ui32Status)
  {
      am_util_debug_printf("Failed to read PSRAM Register MR0!\n");
      return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
  }
  else
  {
      ui8RLCReg = (uint8_t)ui32Rawdata;
      am_util_debug_printf("PSRAM Register MR0 = 0x%02X\n", ui8RLCReg);
      am_util_debug_printf("PSRAM Read Latency Code = 0x%02X\n", ((ui8RLCReg & 0x1C)>>2) + 3 );
  }

  ui8RLCReg &= 0xC0;                        // set latency to 3 (0b000)
  ui8RLCReg |= 0x01;                        // set PSRAM drive strength (0:Full 1:Half(default) 2:Quarter 3: Eighth)
#ifdef USE_NON_DQS_MODE
  ui8RLCReg |= 0x20;                        // set LT to fixed
  am_util_debug_printf("Set read latency into fixed type in NON-DQS mode!\n");
#endif
  ui32Rawdata = ui8RLCReg;

  ui32Status = am_device_command_write(pMspiHandle, AM_DEVICES_MSPI_PSRAM_DDR_WRITE_REGISTER, true, 0, &ui32Rawdata, 4);
  if (AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS != ui32Status)
  {
      am_util_debug_printf("Failed to write PSRAM Register MR0!\n");
      return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
  }
  else
  {
      am_util_debug_printf("Set PSRAM Register MR0 into 0x%02X\n", ui8RLCReg);
  }

  am_util_debug_printf("Read PSRAM Read Latency Code\n");
  ui32Status = am_device_command_read(pMspiHandle, AM_DEVICES_MSPI_PSRAM_DDR_READ_REGISTER, true, 0, &ui32Rawdata, 4);
  if (AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS != ui32Status)
  {
      am_util_debug_printf("Failed to read PSRAM Read Latency Code!\n");
      return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
  }
  else
  {
      ui8RLCReg = (uint8_t)ui32Rawdata;
      am_util_debug_printf("PSRAM Register MR0 = 0x%02X\n", ui8RLCReg);
      am_util_debug_printf("PSRAM Read Latency Code = 0x%02X\n\n", ((ui8RLCReg & 0x1C)>>2) + 3 );
  }

  //
  // Read and set PSRAM Write Latency Code
  //
  am_util_debug_printf("Read PSRAM Write Latency Code\n");
  ui32Status = am_device_command_read(pMspiHandle, AM_DEVICES_MSPI_PSRAM_DDR_READ_REGISTER, true, 4, &ui32Rawdata, 4);
  if (AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS != ui32Status)
  {
      am_util_debug_printf("Failed to read PSRAM Write Latency Code!\n");
      return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
  }
  else
  {
      ui8RLCReg = (uint8_t)ui32Rawdata;
      am_util_debug_printf("PSRAM Register MR4 = 0x%02X\n", ui8RLCReg);
      am_util_debug_printf("PSRAM Write Latency Code = 0x%02X\n", ((ui8RLCReg & 0xE0)>>5) + 3 );
  }

  ui32Rawdata = ui8RLCReg & 0x1F;

  ui32Status = am_device_command_write(pMspiHandle, AM_DEVICES_MSPI_PSRAM_DDR_WRITE_REGISTER, true, 4, &ui32Rawdata, 4);
  if (AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS != ui32Status)
  {
      am_util_debug_printf("Failed to write PSRAM Write Latency Code!\n");
      return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
  }
  else
  {
      am_util_debug_printf("Set PSRAM Write Latency Code into 3\n");
  }

  am_util_debug_printf("Read PSRAM Write Latency Code\n");
  ui32Status = am_device_command_read(pMspiHandle, AM_DEVICES_MSPI_PSRAM_DDR_READ_REGISTER, true, 4, &ui32Rawdata, 4);
  if (AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS != ui32Status)
  {
      am_util_debug_printf("Failed to read PSRAM Write Latency Code!\n");
      return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
  }
  else
  {
      ui8RLCReg = (uint8_t)ui32Rawdata;
      am_util_debug_printf("PSRAM Register MR4 = 0x%02X\n", ui8RLCReg);
      am_util_debug_printf("PSRAM Write Latency Code = 0x%02X\n\n", ((ui8RLCReg & 0xE0)>>5) + 3 );
  }

  //
  // Read PSRAM Vendor ID and Device ID and return status.
  //
  am_util_debug_printf("Read PSRAM Vendor ID\n");
  ui32Status = am_device_command_read(pMspiHandle, AM_DEVICES_MSPI_PSRAM_DDR_READ_REGISTER, true, 1, &ui32Rawdata, 4);
  if (AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS != ui32Status)
  {
      am_util_debug_printf("Failed to read PSRAM Vendor ID!\n");
      return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
  }
  else
  {
      ui8VendorIDReg = (uint8_t)ui32Rawdata;
      am_util_debug_printf("PSRAM Register MR1 = 0x%X\n", ui8VendorIDReg);
      pPsramInfo->ui8VendorId = ui8VendorIDReg & 0x1F;
      if ( (ui8VendorIDReg & 0x1F) == 0xD )
      {
        am_util_debug_printf("PSRAM Vendor ID =  01101\n\n");
      }
      else
      {
        am_util_debug_printf("Fail to get correct PSRAM Vendor ID!\n\n");
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
      }
  }

  am_util_debug_printf("Read PSRAM Device ID\n");
  ui32Status = am_device_command_read(pMspiHandle, AM_DEVICES_MSPI_PSRAM_DDR_READ_REGISTER, true, 2, &ui32Rawdata, 4);
  if (AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS != ui32Status)
  {
      am_util_debug_printf("Failed to read PSRAM Device ID!\n");
      return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
  }
  else
  {
      ui8DeviceIDReg = (uint8_t)ui32Rawdata;
      pPsramInfo->ui8DeviceId = (ui8DeviceIDReg & 0x18) >> 3;
      am_util_debug_printf("PSRAM Register MR2 = 0x%X\n", ui8DeviceIDReg);
      am_util_debug_printf("PSRAM Device ID =  Generation %d\n", pPsramInfo->ui8DeviceId + 1);
      if ( (ui8DeviceIDReg & 0x7) == 0x1 )
      {
        pPsramInfo->ui32DeviceSizeKb = 32 / 8 * 1024U;
        am_util_debug_printf("PSRAM Density =  32Mb\n\n");
      }
      else if ( (ui8DeviceIDReg & 0x7) == 0x3 )
      {
        pPsramInfo->ui32DeviceSizeKb = 64 / 8 * 1024U;
        am_util_debug_printf("PSRAM Density =  64Mb\n\n");
      }
      else if ( (ui8DeviceIDReg & 0x7) == 0x5 )
      {
        pPsramInfo->ui32DeviceSizeKb = 128 / 8 * 1024U;
        am_util_debug_printf("PSRAM Density =  128Mb\n\n");
      }
      else if ( (ui8DeviceIDReg & 0x7) == 0x7 )
      {
        pPsramInfo->ui32DeviceSizeKb = 256 / 8 * 1024U;
        am_util_debug_printf("PSRAM Density =  256Mb\n\n");
      }
  }

  return AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS;

}


//*****************************************************************************
//
//! @brief This function takes care of splitting the transaction as needed, if the transaction crosses
//! PSRAM page boundary or because of tCEM restrictions, if hardware does not support it
//!
//! @param pPsram
//! @param bHiPrio
//! @param bWrite
//! @param pui8Buffer
//! @param ui32Address
//! @param ui32NumBytes
//! @param ui32PauseCondition
//! @param ui32StatusSetClr
//! @param pfnCallback
//! @param pCallbackCtxt
//!
//! @return
//
//*****************************************************************************
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

static inline uint8_t
am_devices_mspi_psram_aps25616n_get_wlc(am_devices_mspi_psram_aps25616n_wlc_e eWLC, uint8_t *pWLC)
{
    switch(eWLC)
    {
      case AM_DEVICES_MSPI_PSRAM_APS25616n_WLC_3:
          *pWLC = 3;
          break;
      case AM_DEVICES_MSPI_PSRAM_APS25616n_WLC_4:
          *pWLC = 4;
          break;
      case AM_DEVICES_MSPI_PSRAM_APS25616n_WLC_5:
          *pWLC = 5;
          break;
      case AM_DEVICES_MSPI_PSRAM_APS25616n_WLC_6:
          *pWLC = 6;
          break;
      case AM_DEVICES_MSPI_PSRAM_APS25616n_WLC_7:
          *pWLC = 7;
          break;
      default:
          return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }
    return AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Verify the External Chip Configuration matches Device settings
//
//*****************************************************************************
uint32_t
am_devices_mspi_psram_aps25616n_verify_config(void *pHandle, uint8_t ui8RLC, uint8_t ui8WLC)
{
    uint8_t ui8RegValue;
    uint32_t ui32Rawdata[2];
    uint32_t ui32Status;

    am_util_debug_printf("\nVerifying device config\n");
    am_util_debug_printf("Read PSRAM Read Latency Code\n");
    ui32Status = am_device_command_read(pHandle, AM_DEVICES_MSPI_PSRAM_DDR_READ_REGISTER, true, 0, (uint32_t *)&ui32Rawdata, 8);
    if (AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS != ui32Status)
    {
        am_util_debug_printf("Failed to read PSRAM Read Latency Code!\n");
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }
    else
    {
        ui8RegValue = (uint8_t)ui32Rawdata[1];
        am_util_debug_printf("PSRAM Register MR0 = 0x%02X\n", ui8RegValue);
        am_util_debug_printf("PSRAM Read Latency = %d\n\n", ((ui8RegValue & 0x1C)>>2) + 3 );
    }

    if ( ui8RLC != ((ui8RegValue & 0x1C) >> 2) )
    {
        am_util_debug_printf("Config PSRAM Read Latency Code failed!\n");
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

#ifdef USE_NON_DQS_MODE
    if ( !(ui8RegValue & 0x20) )
    {
        am_util_debug_printf("Config PSRAM Latency type failed!\n");
    }
#else
    if ( (ui8RegValue & 0x20) )
    {
        am_util_debug_printf("Config PSRAM Latency type failed!\n");
    }
#endif

    //
    // Read and set PSRAM Write Latency Code
    //
    am_util_debug_printf("Read PSRAM Write Latency Code\n");
    ui32Status = am_device_command_read(pHandle, AM_DEVICES_MSPI_PSRAM_DDR_READ_REGISTER, true, 4, (uint32_t *)&ui32Rawdata, 8);
    if (AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS != ui32Status)
    {
        am_util_debug_printf("Failed to read PSRAM Write Latency Code!\n");
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }
    else
    {
        ui8RegValue = (uint8_t)ui32Rawdata[1];
        am_util_debug_printf("PSRAM Register MR4 = 0x%02X\n", ui8RegValue);
        uint8_t ui8Temp;
        if (AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS != am_devices_mspi_psram_aps25616n_get_wlc((am_devices_mspi_psram_aps25616n_wlc_e)((ui8RegValue & 0xE0)>>5), &ui8Temp) )
        {
            am_util_debug_printf("Invalid Write Latency Code!\n");
            return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
        }
        am_util_debug_printf("PSRAM Write Latency = %d\n\n", ui8Temp);
    }

    if ( ui8WLC != ((ui8RegValue & 0xE0) >> 5) )
    {
        am_util_debug_printf("Config PSRAM Write Latency Code failed!\n");
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

    return AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS;
}


//*****************************************************************************
//
// Initialize the mspi_psram driver.
//
//*****************************************************************************
uint32_t
am_devices_mspi_psram_aps25616n_ddr_init(uint32_t ui32Module,
                                         am_devices_mspi_psram_config_t *pDevCfg,
                                         void **ppHandle,
                                         void **ppMspiHandle)
{
    uint32_t                    ui32Status;
    am_hal_mspi_dev_config_t    mspiRegDevCfg;
    am_hal_mspi_dev_config_t    *mspiMemDevCfg;
    void                        *pMspiHandle;
    uint32_t                    ui32Index = 0;

    if ((ui32Module > AM_REG_MSPI_NUM_MODULES) || (pDevCfg == NULL))
    {
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

    //
    // Enable fault detection.
    //
    am_hal_fault_capture_enable();

    // Allocate a vacant device handle
    for ( ui32Index = 0; ui32Index < AM_DEVICES_MSPI_PSRAM_MAX_DEVICE_NUM; ui32Index++ )
    {
        if ( gAPMDDRPsram[ui32Index].bOccupied == false )
        {
            break;
        }
    }
    if ( ui32Index == AM_DEVICES_MSPI_PSRAM_MAX_DEVICE_NUM)
    {
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

    am_util_debug_printf("\nStart PSRAM Initialization\n");

    //
    // Configure the MSPI into Octal mode for PSRAM register access and Hex mode for data access.
    //
    switch (pDevCfg->eDeviceConfig)
    {
        case AM_HAL_MSPI_FLASH_OCTAL_DDR_CE0:
            mspiMemDevCfg = &APMDDROctalCE0MSPIConfig;
            mspiRegDevCfg = APMDDROctalCE0MSPIConfig;
            break;
        case AM_HAL_MSPI_FLASH_OCTAL_DDR_CE1:
            mspiMemDevCfg = &APMDDROctalCE1MSPIConfig;
            mspiRegDevCfg = APMDDROctalCE1MSPIConfig;
            break;
        case AM_HAL_MSPI_FLASH_HEX_DDR_CE0:
            mspiMemDevCfg = &APMDDRHEXCE0MSPIConfig;
            mspiRegDevCfg = APMDDROctalCE0MSPIConfig;
            break;
        case AM_HAL_MSPI_FLASH_HEX_DDR_CE1:
            mspiMemDevCfg = &APMDDRHEXCE1MSPIConfig;
            mspiRegDevCfg = APMDDROctalCE1MSPIConfig;
            break;
        default:
            return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

    mspiMemDevCfg->eClockFreq = pDevCfg->eClockFreq;
    mspiRegDevCfg.eClockFreq = AM_HAL_MSPI_CLK_48MHZ;

    // First configure in HEX mode and reset
    if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_initialize(ui32Module, &pMspiHandle))
    {
        am_util_debug_printf("Error - Failed to initialize MSPI.\n");
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

    if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_power_control(pMspiHandle, AM_HAL_SYSCTRL_WAKE, false))
    {
        am_util_debug_printf("Error - Failed to power on MSPI.\n");
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

    am_hal_mspi_config_t    * mspiCfg = &gAPMDDRMspiCfg;
    mspiCfg->ui32TCBSize = pDevCfg->ui32NBTxnBufLength;
    mspiCfg->pTCB = pDevCfg->pNBTxnBuf;
    if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_configure(pMspiHandle, mspiCfg))
    {
        am_util_debug_printf("Error - Failed to configure MSPI device.\n");
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

    if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_device_configure(pMspiHandle, &mspiRegDevCfg))
    {
        am_util_debug_printf("Error - Failed to configure MSPI device.\n");
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

    am_hal_mspi_xip_config_t    xipCfg = gAPMDDRXipConfig[ui32Module];

    xipCfg.scramblingStartAddr = pDevCfg->ui32ScramblingStartAddr;
    xipCfg.scramblingEndAddr = pDevCfg->ui32ScramblingEndAddr;
    ui32Status = am_hal_mspi_control(pMspiHandle, AM_HAL_MSPI_REQ_XIP_CONFIG, &xipCfg);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
      return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }
    gAPMDDRPsram[ui32Index].sDeviceInfo.ui32BaseAddr = xipCfg.ui32APBaseAddr;

    am_hal_mspi_xip_misc_t    xipMiscCfg = gAPMXipMiscCfg[ui32Module];
    ui32Status = am_hal_mspi_control(pMspiHandle, AM_HAL_MSPI_REQ_XIP_MISC_CONFIG, &xipMiscCfg);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
      return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

    am_hal_mspi_dqs_t dqsCfg = gAPMDDRDqsCfg[ui32Module];
    ui32Status = am_hal_mspi_control(pMspiHandle, AM_HAL_MSPI_REQ_DQS, &dqsCfg);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
      return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

    if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_enable(pMspiHandle))
    {
        am_util_debug_printf("Error - Failed to enable MSPI.\n");
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }
    am_bsp_mspi_pins_enable(ui32Module, mspiRegDevCfg.eDeviceConfig);

    am_util_delay_us(150);

    //
    // Send reset command to PSRAM
    //
    if (AM_HAL_STATUS_SUCCESS != am_devices_mspi_psram_aps25616n_reset(pMspiHandle))
    {
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

    am_util_delay_us(2);

    //
    // initialize device
    //
    if (AM_HAL_STATUS_SUCCESS != am_devices_mspi_psram_aps25616n_device_init(pMspiHandle, &gAPMDDRPsram[ui32Index].sDeviceInfo))
    {
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

    if ((pDevCfg->eDeviceConfig == AM_HAL_MSPI_FLASH_HEX_DDR_CE0) || (pDevCfg->eDeviceConfig == AM_HAL_MSPI_FLASH_HEX_DDR_CE1))
    {
        //
        // Set PSRAM into HEX mode
        //
        if (AM_HAL_STATUS_SUCCESS != am_devices_mspi_psram_aps25616n_enter_hex_mode(pMspiHandle))
        {
            return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
        }
    }

    //
    // Disable MSPI defore re-configuring it
    //
    ui32Status = am_hal_mspi_disable(pMspiHandle);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

    //
    // Reconfig MSPI device settings
    //
    if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_device_configure(pMspiHandle, mspiMemDevCfg))
    {
        am_util_debug_printf("Error - Failed to reconfig MSPI device.\n");
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

    am_hal_mspi_rxcfg_t RxCfg = gAPMMspiRxCfg;
    if (AM_HAL_MSPI_CLK_96MHZ == pDevCfg->eClockFreq)
    {
      RxCfg.ui8RxSmp = 2;
    }
    ui32Status = am_hal_mspi_control(pMspiHandle, AM_HAL_MSPI_REQ_RXCFG, &RxCfg);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
      return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

    //
    // Re-Enable MSPI
    //
    ui32Status = am_hal_mspi_enable(pMspiHandle);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

    //
    // Re-config the MSPI pins.
    //
    am_bsp_mspi_pins_enable(ui32Module, mspiMemDevCfg->eDeviceConfig);

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
    gAPMDDRPsram[ui32Index].bOccupied = true;
    *ppHandle = (void *)&gAPMDDRPsram[ui32Index];
    *ppMspiHandle = gAPMDDRPsram[ui32Index].pMspiHandle = pMspiHandle;
    gAPMDDRPsram[ui32Index].ui32Module = ui32Module;
    gAPMDDRPsram[ui32Index].eDeviceConfig = mspiMemDevCfg->eDeviceConfig;

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
am_devices_mspi_psram_aps25616n_ddr_deinit(void *pHandle)
{
    uint32_t    ui32Status;
    am_devices_mspi_psram_t *pPsram = (am_devices_mspi_psram_t *)pHandle;

    am_util_debug_printf("\nStart PSRAM Deinitialization\n");

    if ((pPsram->eDeviceConfig == AM_HAL_MSPI_FLASH_HEX_DDR_CE0) || (pPsram->eDeviceConfig == AM_HAL_MSPI_FLASH_HEX_DDR_CE1))
    {
        //
        // Set PSRAM into Octal mode. Exit Hex mode.
        //
        ui32Status = am_devices_mspi_psram_aps25616n_exit_hex_mode(pPsram->pMspiHandle);
        if (AM_HAL_STATUS_SUCCESS != ui32Status)
        {
            return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
        }
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
        am_util_debug_printf("Error - Failed to power on MSPI.\n");
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
am_devices_mspi_psram_aps25616n_ddr_read(void *pHandle,
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
                                            pfnMSPI_APMPSRAM_DDR_Callback,
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
      am_hal_delay_us(1);
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
//! @brief
//!
//! @param pHandle
//! @param pui8RxBuffer
//! @param ui32ReadAddress
//! @param ui32NumBytes
//!
//! @return
//
//*****************************************************************************
static uint32_t
mspi_aps25616n_ddr_dma_read(void *pHandle, uint8_t *pui8RxBuffer,
                            uint32_t ui32ReadAddress,
                            uint32_t ui32NumBytes)
{
  uint32_t                      ui32Status;
  am_devices_mspi_psram_t *pPsram = (am_devices_mspi_psram_t *)pHandle;


    // Start the transaction.
    volatile bool bDMAComplete = false;
    ui32Status = psram_nonblocking_transfer(pPsram, false, false,
                                            pui8RxBuffer,
                                            ui32ReadAddress,
                                            ui32NumBytes,
                                            0,
                                            0,
                                            pfnMSPI_APMPSRAM_DDR_Callback,
                                            (void *)&bDMAComplete);

    // Check the transaction status.
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
      return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

    // Wait for DMA Complete or Timeout
    for (uint32_t i = 0; i < AM_DEVICES_MSPI_PSRAM_TIMEOUT; i++)
    {
        // check DMA status without using ISR
        am_hal_mspi_interrupt_status_get(pPsram->pMspiHandle, &ui32Status, false);
        am_hal_mspi_interrupt_clear(pPsram->pMspiHandle, ui32Status);
        am_hal_mspi_interrupt_service(pPsram->pMspiHandle, ui32Status);

      if (bDMAComplete)
      {
        break;
      }
      //
      // Call the BOOTROM cycle function to delay for about 1 microsecond.
      //
      am_hal_delay_us(1);
    }

    // Check the status.
    if (!bDMAComplete)
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
// Reads the contents of the external PSRAM into a buffer.
//
//*****************************************************************************
uint32_t
am_devices_mspi_psram_aps25616n_ddr_read_adv(void *pHandle,
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
// Reads the contents of the external psram into a buffer
// with high priority
//
//*****************************************************************************
uint32_t
am_devices_mspi_psram_aps25616n_ddr_read_hiprio(void *pHandle,
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

uint32_t
am_devices_mspi_psram_aps25616n_ddr_nonblocking_read(void *pHandle,
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
am_devices_mspi_psram_aps25616n_ddr_write(void *pHandle,
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
                                            pfnMSPI_APMPSRAM_DDR_Callback,
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
      am_hal_delay_us(1);
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
//! @brief
//!
//! @param pHandle
//! @param pui8TxBuffer
//! @param ui32WriteAddress
//! @param ui32NumBytes
//!
//! @return
//
//*****************************************************************************
static uint32_t
mspi_aps25616n_ddr_dma_write(void *pHandle, uint8_t *pui8TxBuffer,
                             uint32_t ui32WriteAddress,
                             uint32_t ui32NumBytes)
{
    uint32_t                      ui32Status;
    am_devices_mspi_psram_t *pPsram = (am_devices_mspi_psram_t *)pHandle;

    {
        // Start the transaction.
        volatile bool bDMAComplete = false;
        ui32Status = psram_nonblocking_transfer(pPsram, false, true,
                                    pui8TxBuffer,
                                    ui32WriteAddress,
                                    ui32NumBytes,
                                    0,
                                    0,
                                    pfnMSPI_APMPSRAM_DDR_Callback,
                                    (void *)&bDMAComplete);

        // Check the transaction status.
        if (AM_HAL_STATUS_SUCCESS != ui32Status)
        {
            return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
        }

        // Wait for DMA Complete or Timeout
        for (uint32_t i = 0; i < AM_DEVICES_MSPI_PSRAM_TIMEOUT; i++)
        {
            // check DMA status without using ISR
            am_hal_mspi_interrupt_status_get(pPsram->pMspiHandle, &ui32Status, false);
            am_hal_mspi_interrupt_clear(pPsram->pMspiHandle, ui32Status);
            am_hal_mspi_interrupt_service(pPsram->pMspiHandle, ui32Status);
            if (bDMAComplete)
            {
                break;
            }
            //
            // Call the BOOTROM cycle function to delay for about 1 microsecond.
            //
            am_hal_delay_us(1);
        }

        // Check the status.
        if ( !bDMAComplete )
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
// Programs the given range of psram addresses.
//
//*****************************************************************************
uint32_t
am_devices_mspi_psram_aps25616n_ddr_write_adv(void *pHandle,
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
// Programs the contents of the external psram into a buffer
// with high priority
//
//*****************************************************************************
uint32_t
am_devices_mspi_psram_aps25616n_ddr_write_hiprio(void *pHandle,
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
//! @brief
//!
//! @param pHandle
//! @param pui8TxBuffer
//! @param ui32WriteAddress
//! @param ui32NumBytes
//! @param pfnCallback
//! @param pCallbackCtxt
//!
//! @return
//
//*****************************************************************************
uint32_t
am_devices_mspi_psram_aps25616n_ddr_nonblocking_write(void *pHandle,
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
//  Sets up the MSPI and external psram into XIP mode.
//
//*****************************************************************************
uint32_t
am_devices_mspi_psram_aps25616n_ddr_enable_xip(void *pHandle)
{
  uint32_t ui32Status;
  am_devices_mspi_psram_t *pPsram = (am_devices_mspi_psram_t *)pHandle;

  //
  // Set Aperture XIP range
  //
  ui32Status = am_hal_mspi_control(pPsram->pMspiHandle, AM_HAL_MSPI_REQ_XIP_CONFIG, &gAPMDDRXipConfig[pPsram->ui32Module]);
  if (AM_HAL_STATUS_SUCCESS != ui32Status)
  {
    return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
  }

  //
  // Enable XIP on the MSPI.
  //
  ui32Status = am_hal_mspi_control(pPsram->pMspiHandle, AM_HAL_MSPI_REQ_XIP_EN, &gAPMDDRXipConfig[pPsram->ui32Module]);
  if (AM_HAL_STATUS_SUCCESS != ui32Status)
  {
    return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
  }

  return AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS;
}

//*****************************************************************************
//
//   Removes the MSPI and external psram from XIP mode.
//
//*****************************************************************************
uint32_t
am_devices_mspi_psram_aps25616n_ddr_disable_xip(void *pHandle)
{
  uint32_t ui32Status;
  am_devices_mspi_psram_t *pPsram = (am_devices_mspi_psram_t *)pHandle;

  //
  // Disable XIP on the MSPI.
  //
  ui32Status = am_hal_mspi_control(pPsram->pMspiHandle, AM_HAL_MSPI_REQ_XIP_DIS, &gAPMDDRXipConfig[pPsram->ui32Module]);
  if (AM_HAL_STATUS_SUCCESS != ui32Status)
  {
    return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
  }

  return AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS;
}

//*****************************************************************************
//
//   Sets up the MSPI and external psram into scrambling mode.
//
//*****************************************************************************
uint32_t
am_devices_mspi_psram_aps25616n_ddr_enable_scrambling(void *pHandle)
{
  uint32_t ui32Status;
  am_devices_mspi_psram_t *pPsram = (am_devices_mspi_psram_t *)pHandle;

  //
  // Enable scrambling on the MSPI.
  //
  ui32Status = am_hal_mspi_control(pPsram->pMspiHandle, AM_HAL_MSPI_REQ_SCRAMB_EN, &gAPMDDRXipConfig[pPsram->ui32Module]);
  if (AM_HAL_STATUS_SUCCESS != ui32Status)
  {
    return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
  }
  return AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS;
}

//*****************************************************************************
//
//   Removes the MSPI and external psram from scrambling mode.
//
//*****************************************************************************
uint32_t
am_devices_mspi_psram_aps25616n_ddr_disable_scrambling(void *pHandle)
{
  uint32_t ui32Status;
  am_devices_mspi_psram_t *pPsram = (am_devices_mspi_psram_t *)pHandle;

  //
  // Disable Scrambling on the MSPI.
  //
  ui32Status = am_hal_mspi_control(pPsram->pMspiHandle, AM_HAL_MSPI_REQ_SCRAMB_DIS, &gAPMDDRXipConfig[pPsram->ui32Module]);
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
am_devices_mspi_psram_aps25616n_ddr_reset(void *pHandle)
{
  am_devices_mspi_psram_t *pPsram = (am_devices_mspi_psram_t *)pHandle;
  return am_devices_mspi_psram_aps25616n_reset(pPsram);
}

//*****************************************************************************
//
//   Reads the ID of the external psram and returns the value.
//
//*****************************************************************************
uint32_t
am_devices_mspi_psram_aps25616n_ddr_id(void *pHandle)
{
  am_devices_mspi_psram_t *pPsram = (am_devices_mspi_psram_t *)pHandle;
  uint32_t ui32DeviceID;
  if ( pPsram->pMspiHandle == NULL || !pPsram->bOccupied )
  {
    return 0xFFFFFFFF;
  }

  ui32DeviceID = pPsram->sDeviceInfo.ui8DeviceId;
  ui32DeviceID |= (uint32_t)pPsram->sDeviceInfo.ui8VendorId << 8;

  return ui32DeviceID;
}

//*****************************************************************************
//
//   Reads the info of the external psram and returns the value.
//
//*****************************************************************************
uint32_t am_devices_mspi_psram_aps25616n_ddr_info(void *pHandle, am_devices_mspi_psram_info_t *pPsramInfo)
{
  am_devices_mspi_psram_t *pPsram = (am_devices_mspi_psram_t *)pHandle;
  if ( pPsram->pMspiHandle == NULL || !pPsram->bOccupied )
  {
    return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
  }

  *pPsramInfo = pPsram->sDeviceInfo;

  return AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS;
}


#if defined(AM_PART_APOLLO4P) || defined(AM_PART_APOLLO4L)
//*****************************************************************************
//
//! @brief write and read back check.
//!
//! @param psMSPISettings - MSPI device structure describing the target spi psram.
//! @param pHandle - MSPI handler which needs to be return
//!
//! This function should be called before any other am_devices_mspi_psram
//! functions. It is used to set tell the other functions how to communicate
//! with the external psram hardware.
//!
//! @return status.
//
//*****************************************************************************
#define PSRAM_CHECK_DATA_SIZE_BYTES  256
//*****************************************************************************
//
//! @brief
//!
//! @param pattern_index
//! @param buff
//! @param len
//!
//! @return
//
//*****************************************************************************
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
//!
//! @param flashHandle
//! @param length
//! @param address
//!
//! @return
//
//*****************************************************************************
static bool
psram_check_by_dma(void* flashHandle, uint32_t length, uint32_t address)
{
    // Try to use as less ram as possible in stack
    uint32_t ui32NumberOfBytesLeft = length;
    uint32_t ui32TestBytes = 0;
    uint32_t ui32AddressOffset = 0;
    uint8_t ui8PatternCounter = 0;
    uint8_t ui8TxBuffer[PSRAM_CHECK_DATA_SIZE_BYTES];
    uint8_t ui8RxBuffer[PSRAM_CHECK_DATA_SIZE_BYTES];
    uint32_t ui32Status = AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;

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
        ui32Status = mspi_aps25616n_ddr_dma_write(flashHandle, ui8TxBuffer,
                                            address,
                                            ui32TestBytes);
        if ( ui32Status ==  AM_DEVICES_MSPI_PSRAM_STATUS_ERROR)
        {
            return true;
        }
        ui32Status = mspi_aps25616n_ddr_dma_read(flashHandle, ui8RxBuffer,
                                                address,
                                                ui32TestBytes);
        if ( ui32Status ==  AM_DEVICES_MSPI_PSRAM_STATUS_ERROR)
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

//#define MEMORY_WORD_ACCESS
//#define MEMORY_SHORT_ACCESS
//#define MEMORY_BYTE_ACCESS
#define MEMORY_COPY_ACCESS
//*****************************************************************************
//
//! @brief
//!
//! @param flashHandle
//! @param length
//! @param address
//!
//! @return
//
//*****************************************************************************
static bool
psram_check_by_xip(void* flashHandle, uint32_t length, uint32_t address)
{
    // Try to use as less ram as possible in stack
    uint32_t ui32NumberOfBytesLeft = length;
    uint32_t ui32TestBytes = 0;
    uint32_t ui32AddressOffset = 0;
    uint8_t ui8PatternCounter = 0;
    uint32_t ix;

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

#if defined(MEMORY_WORD_ACCESS)
        uint8_t ui32TxBuffer[PSRAM_CHECK_DATA_SIZE_BYTES / 4];
        uint8_t ui32RxBuffer[PSRAM_CHECK_DATA_SIZE_BYTES / 4];
        //
        // Write to target address with test pattern with given length
        // Use 5 patterns: 0x5555AAAA, 0xFFFF0000, Walking, incremental and decremental
        //
        prepare_test_pattern((ui8PatternCounter) % 5, (uint8_t*)ui32TxBuffer, ui32TestBytes);
        ui8PatternCounter++;
        // write to target address
        am_hal_sysctrl_bus_write_flush();
        uint32_t * pu32Ptr = (uint32_t *)(address + ui32AddressOffset);
        for (ix = 0; ix < ui32TestBytes / 4; ix++)
        {
          *pu32Ptr++ = ui32TxBuffer[ix];
        }

        //
        // Read back data
        //
        am_hal_sysctrl_bus_write_flush();
        pu32Ptr = (uint32_t *)(address + ui32AddressOffset);
        for (ix = 0; ix < ui32TestBytes / 4; ix++)
        {
           ui32RxBuffer[ix] = *pu32Ptr++ ;
        }
        //
        // Verify the result
        //
        for (ix = 0; ix < ui32TestBytes / 4; ix++)
        {
            if (ui32RxBuffer[ix] != ui32TxBuffer[ix])
            {
                //am_util_debug_printf("    Failed to verify at offset 0x%08x, expect data : 0x%08x, read data : 0x%08x !\n", ui32AddressOffset, ui8RxBuffer[ix], ui8TxBuffer[ix]);
                // verify failed, return directly
                return true;

            }
        }

#elif defined(MEMORY_SHORT_ACCESS)
        uint8_t ui16TxBuffer[PSRAM_CHECK_DATA_SIZE_BYTES / 2];
        uint8_t ui16RxBuffer[PSRAM_CHECK_DATA_SIZE_BYTES / 2];

        //
        // Write to target address with test pattern with given length
        // Use 5 patterns: 0x5555AAAA, 0xFFFF0000, Walking, incremental and decremental
        //
        prepare_test_pattern((ui8PatternCounter) % 5, (uint8_t*)ui16TxBuffer, ui32TestBytes);
        ui8PatternCounter++;
        // write to target address
        am_hal_sysctrl_bus_write_flush();
        uint16_t * pu16Ptr = (uint16_t *)(address + ui32AddressOffset);
        for (ix = 0; ix < ui32TestBytes / 2; ix++)
        {
          *pu16Ptr++ = ui16TxBuffer[ix];
        }

        //
        // Read back data
        //
        am_hal_sysctrl_bus_write_flush();
        pu16Ptr = (uint16_t *)(address + ui32AddressOffset);
        for (ix = 0; ix < ui32TestBytes / 2; ix++)
        {
           ui16RxBuffer[ix] = *pu16Ptr++ ;
        }
        //
        // Verify the result
        //
        for (ix = 0; ix < ui32TestBytes / 2; ix++)
        {
            if (ui16RxBuffer[ix] != ui16TxBuffer[ix])
            {
                //am_util_debug_printf("    Failed to verify at offset 0x%08x, expect data : 0x%08x, read data : 0x%08x !\n", ui32AddressOffset, ui8RxBuffer[ix], ui8TxBuffer[ix]);
                // verify failed, return directly
                return true;

            }
        }
#elif defined(MEMORY_BYTE_ACCESS)
        uint8_t ui8TxBuffer[PSRAM_CHECK_DATA_SIZE_BYTES];
        uint8_t ui8RxBuffer[PSRAM_CHECK_DATA_SIZE_BYTES];
        //
        // Write to target address with test pattern with given length
        // Use 5 patterns: 0x5555AAAA, 0xFFFF0000, Walking, incremental and decremental
        //

        prepare_test_pattern((ui8PatternCounter) % 5, ui8TxBuffer, ui32TestBytes);
        ui8PatternCounter++;

        // write to target address
        am_hal_sysctrl_bus_write_flush();
        uint8_t * pu8Ptr = (uint8_t *)(address + ui32AddressOffset);
        for (ix = 0; ix < ui32TestBytes; ix++)
        {
          *pu8Ptr++ = ui8TxBuffer[ix];
        }

        //
        // Read back data
        //
        am_hal_sysctrl_bus_write_flush();
        pu8Ptr = (uint8_t *)(address + ui32AddressOffset);
        for (ix = 0; ix < ui32TestBytes; ix++)
        {
          ui8RxBuffer[ix] = *pu8Ptr++ ;
        }

        //
        // Verify the result
        //
        for (ix = 0; ix < ui32TestBytes; ix++)
        {
            if (ui8RxBuffer[ix] != ui8TxBuffer[ix])
            {
                //am_util_debug_printf("    Failed to verify at offset 0x%08x, expect data : 0x%08x, read data : 0x%08x !\n", ui32AddressOffset, ui8RxBuffer[ix], ui8TxBuffer[ix]);
                // verify failed, return directly
                return true;

            }

        }
#elif defined(MEMORY_COPY_ACCESS)
        uint8_t ui8TxBuffer[PSRAM_CHECK_DATA_SIZE_BYTES];
        uint8_t ui8RxBuffer[PSRAM_CHECK_DATA_SIZE_BYTES];

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
        for (ix = 0; ix < ui32TestBytes; ix++)
        {
            if (ui8RxBuffer[ix] != ui8TxBuffer[ix])
            {
                //am_util_debug_printf("    Failed to verify at offset 0x%08x, expect data : 0x%08x, read data : 0x%08x !\n", ui32AddressOffset, ui8RxBuffer[ix], ui8TxBuffer[ix]);
                // verify failed, return directly
                return true;

            }

        }
#endif

        ui32AddressOffset += ui32TestBytes;
    }

    return false;
}


//*****************************************************************************
//
//! @brief Count the longest consecutive 1s in a 32bit word
//! @details Static helper function:
//!
//! @param pVal
//!
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
//! @details Static helper function:
//!
//! @param pVal
//!
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

//*****************************************************************************
//
//  Checks PSRAM timing and determine a delay setting.
//
//*****************************************************************************
#define PSRAM_TIMING_SCAN_SIZE_BYTES (128*1024)
static const uint32_t ui32MspiXipBaseAddress[] =
{
    MSPI0_APERTURE_START_ADDR, // mspi0
    MSPI1_APERTURE_START_ADDR, // mspi1
    MSPI2_APERTURE_START_ADDR, // mspi2
};

//*****************************************************************************
//
// Checks PSRAM timing and determine a delay setting.
//
//*****************************************************************************
uint32_t
am_devices_mspi_psram_aps25616n_ddr_init_timing_check(uint32_t module,
                                                      am_devices_mspi_psram_config_t *pDevCfg,
                                                      am_devices_mspi_psram_ddr_timing_config_t *pDevDdrCfg)
{
    uint32_t ui32Status;
    void *pDevHandle;
    void *pHandle;
    uint32_t Txdqsdelay = 0;
    uint32_t Rxdqsdelay = 0;
    uint32_t ui32TxResult = 0;
    uint32_t ui32ResultArray[32] = {0};
    uint32_t ui32CheckAddress;
    uint32_t ui32Result = 0;

    am_hal_mspi_dqs_t scanCfg = gAPMDDRDqsCfg[module];
    //
    // initialize interface
    //
    am_hal_mspi_dev_config_t    *psMSPISettings;
    switch (pDevCfg->eDeviceConfig)
    {
        case AM_HAL_MSPI_FLASH_OCTAL_DDR_CE0:
            psMSPISettings = &APMDDROctalCE0MSPIConfig;
            break;
        case AM_HAL_MSPI_FLASH_OCTAL_DDR_CE1:
            psMSPISettings = &APMDDROctalCE1MSPIConfig;
            break;
        case AM_HAL_MSPI_FLASH_HEX_DDR_CE0:
            psMSPISettings = &APMDDRHEXCE0MSPIConfig;
            break;
        case AM_HAL_MSPI_FLASH_HEX_DDR_CE1:
            psMSPISettings = &APMDDRHEXCE1MSPIConfig;
            break;
        default:
            return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

    uint16_t timeLimit = psMSPISettings->ui16DMATimeLimit;                  // save original setting here
    am_hal_mspi_dma_boundary_e dmaBound0 = psMSPISettings->eDMABoundary;    // save original setting here
    psMSPISettings->ui16DMATimeLimit    = 0;
    psMSPISettings->eDMABoundary        = AM_HAL_MSPI_BOUNDARY_NONE;
    ui32Status = am_devices_mspi_psram_aps25616n_ddr_init(module, pDevCfg, &pDevHandle, &pHandle);
    if (AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS != ui32Status)
    {
        am_util_debug_printf("    Failed to configure the MSPI and PSRAM Device correctly!\n");
        return ui32Status;
    }
    psMSPISettings->ui16DMATimeLimit = timeLimit;   // restore original setting here
    psMSPISettings->eDMABoundary = dmaBound0;       // restore original setting here

    am_util_debug_printf("    Start Use XIP to Timing Scan!\n");
    //
    // Put the MSPI into XIP mode.
    //
    ui32Status = am_devices_mspi_psram_aps25616n_ddr_enable_xip(pDevHandle);
    if (AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS != ui32Status)
    {
        am_util_debug_printf("    Failed to disable XIP mode in the MSPI!\n");
        return ui32Status;
    }

#if defined(AM_PART_APOLLO4P)
    if ( module == 2 )
    {
        am_util_stdio_printf("\nStart MSPI2 timing scan. \nPlease wait for several minutes...\n");
        for ( uint8_t TxDqs_Index = 0; TxDqs_Index <= 31; TxDqs_Index++ )
        {
            for ( uint8_t RxDqs_Index = 0; RxDqs_Index <= 31; RxDqs_Index++ )
            {
                // set TXDQSDELAY0 value
                scanCfg.ui8TxDQSDelay   = TxDqs_Index;
                // set RXDQSDELAY0 value
                scanCfg.ui8RxDQSDelay   = RxDqs_Index;
                // apply settings
                ui32Status = am_hal_mspi_control(pHandle, AM_HAL_MSPI_REQ_DQS, &scanCfg);
                if (AM_HAL_STATUS_SUCCESS != ui32Status)
                {
                    return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
                }

                ui32CheckAddress = ui32MspiXipBaseAddress[module] + RxDqs_Index;

                // run data check
                if ( false == psram_check_by_xip(pDevHandle, PSRAM_TIMING_SCAN_SIZE_BYTES,  ui32CheckAddress ) )
                {
                    // data check pass
                    ui32ResultArray[TxDqs_Index] |= 0x01 << RxDqs_Index;
                }
                else
                {
                    // data check failed
                }
            }
        }

        //
        // Check TX scan result
        //
        for ( uint32_t i = 0; i < sizeof(ui32ResultArray) / sizeof(uint32_t); i++ )
        {
            ui32Result = count_consecutive_ones(&ui32ResultArray[i]);
            if ( ui32Result > PSRAM_TIMING_SCAN_MIN_ACCEPTANCE_LENGTH )
            {
                ui32TxResult |= 0x01 << i;
            }

            //
            // print result for test
            //
            am_util_debug_printf("    Setting %d = 0x%08X\n", i, ui32ResultArray[i]);
        }

        //
        // Check TxResult
        //
        if ( ui32TxResult == 0 )
        {
            // RX window is too small at all TX setting
            return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
        }

        //
        // Find TXDQSDELAY Value
        //
        Txdqsdelay = find_mid_point(&ui32TxResult);
        //
        // Find RXDQSDELAY Value
        //
        Rxdqsdelay = find_mid_point(&ui32ResultArray[Txdqsdelay]);
        am_util_debug_printf("Selected TxDqsDelay Setting = %d\n", Txdqsdelay);
        am_util_debug_printf("Selected RxDqsDelay Setting = %d\n", Rxdqsdelay);
    }
    else
#endif
    {

        //
        // Start scan loop
        //
        for ( uint8_t RxDqs_Index = 0; RxDqs_Index <= 31; RxDqs_Index++ )
        {
            // set RXDQSDELAY0 value
            scanCfg.ui8RxDQSDelay   = RxDqs_Index;
            // apply settings
            ui32Status = am_hal_mspi_control(pHandle, AM_HAL_MSPI_REQ_DQS, &scanCfg);
            if (AM_HAL_STATUS_SUCCESS != ui32Status)
            {
                return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
            }

            ui32CheckAddress = ui32MspiXipBaseAddress[module] + RxDqs_Index;

            // run data check
            if ( false == psram_check_by_xip(pDevHandle, PSRAM_TIMING_SCAN_SIZE_BYTES,  ui32CheckAddress ) )
            {
                // data check pass
                ui32ResultArray[0] |= 0x01 << RxDqs_Index;
            }
            else
            {
                // data check failed
            }
        }

        //
        // Check result
        //
        ui32Result = count_consecutive_ones(&ui32ResultArray[0]);

        //
        // print result for test
        //
        am_util_debug_printf("    Setting 0 = 0x%08X\n",  ui32ResultArray[0]);

#if defined(AM_DEBUG_PRINTF)
        am_util_stdio_printf("    Timing Scan found a window %d fine steps wide.\n", ui32Result);
#endif

        //
        // Find RXDQSDELAY Value
        //
        Rxdqsdelay = find_mid_point(&ui32ResultArray[0]);
        am_util_debug_printf("    Selected RxDqsDelay Setting = %d\n", Rxdqsdelay);

        //
        // Check consecutive passing settings
        //
        if ( ui32Result < PSRAM_TIMING_SCAN_MIN_ACCEPTANCE_LENGTH )
        {
            // too short is the passing settings
            return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
        }
    }

    //
    // Deinitialize the MSPI interface
    //
    am_devices_mspi_psram_aps25616n_ddr_deinit(pDevHandle);

    //
    // Set output values
    //
    pDevDdrCfg->ui32Rxdqsdelay = Rxdqsdelay;
    pDevDdrCfg->ui32Rxneg = 0;
#ifdef USE_NON_DQS_MODE
    pDevDdrCfg->ui32Turnaround = 12;
#else
    pDevDdrCfg->ui32Turnaround = 6;
#endif

#if defined(AM_PART_APOLLO4P)
    if ( module == 2 )
    {
        pDevDdrCfg->ui32Txdqsdelay = Txdqsdelay;
    }
    else
#endif
    {
        pDevDdrCfg->ui32Txdqsdelay = 0;
    }

    return AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS;
}

//*****************************************************************************
//
//  Apply given DDR timing settings to target MSPI instance.
//
//*****************************************************************************
uint32_t
am_devices_mspi_psram_aps25616n_apply_ddr_timing(void *pHandle,
                                                 am_devices_mspi_psram_ddr_timing_config_t *pDevDdrCfg)
{
    am_devices_mspi_psram_t *pPsram = (am_devices_mspi_psram_t *)pHandle;

    am_hal_mspi_dqs_t applyCfg = gAPMDDRDqsCfg[pPsram->ui32Module];
    // apply timing settings
    applyCfg.ui8RxDQSDelay      = pDevDdrCfg->ui32Rxdqsdelay;
    if ( pPsram->ui32Module == 2 )
    {
        applyCfg.ui8TxDQSDelay      = pDevDdrCfg->ui32Txdqsdelay;
#if defined(AM_DEBUG_PRINTF)
        am_util_stdio_printf("    Timing Scan set the TxDQSDelay = %d .\n", applyCfg.ui8TxDQSDelay);
#endif
    }
#if defined(AM_DEBUG_PRINTF)
    am_util_stdio_printf("    Timing Scan set the RxDQSDelay = %d .\n", applyCfg.ui8RxDQSDelay);
#endif
    return am_hal_mspi_control(pPsram->pMspiHandle, AM_HAL_MSPI_REQ_DQS, &applyCfg);

}
#endif

#if defined(AM_PART_APOLLO4P) || defined(AM_PART_APOLLO4L)
//*****************************************************************************
//
// Enter half sleep
//
// Send a command to Enter Half Sleep Mode. Will need to Be in OCTAL mode
// to access the register per the device driver.
//
//*****************************************************************************
uint32_t
am_devices_mspi_psram_aps25616n_enter_halfsleep(void *pHandle)
{
    uint32_t ui32PIOBuffer = 0xF0;

    am_devices_mspi_psram_t *pPsram = (am_devices_mspi_psram_t *)pHandle;

    if ((pPsram->eDeviceConfig == AM_HAL_MSPI_FLASH_HEX_DDR_CE0) || (pPsram->eDeviceConfig == AM_HAL_MSPI_FLASH_HEX_DDR_CE1))
    {
        am_hal_mspi_device_e lcl_devCfg = pPsram->eDeviceConfig;

        switch (pPsram->eDeviceConfig)
        {
            case AM_HAL_MSPI_FLASH_HEX_DDR_CE0:
                lcl_devCfg = AM_HAL_MSPI_FLASH_OCTAL_DDR_CE0;
                break;
            case AM_HAL_MSPI_FLASH_HEX_DDR_CE1:
                lcl_devCfg = AM_HAL_MSPI_FLASH_OCTAL_DDR_CE1;
                break;
            default:
                return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
        }

        //
        // Disable MSPI defore re-configuring it
        //
        if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_disable(pPsram->pMspiHandle))
        {
            am_util_debug_printf("Error - Failed to Disable MSPI.\n");
            return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
        }

        //
        // Reconfigure MSPI to OCTAL
        //
        am_hal_mspi_dev_config_t*    mspiRegDevCfg;
        switch (pPsram->eDeviceConfig)
        {
            case AM_HAL_MSPI_FLASH_HEX_DDR_CE0:
                mspiRegDevCfg = &APMDDROctalCE0MSPIConfig;
                break;
            case AM_HAL_MSPI_FLASH_HEX_DDR_CE1:
                mspiRegDevCfg = &APMDDROctalCE1MSPIConfig;
                break;
            default:
                return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
        }

        if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_device_configure(pPsram->pMspiHandle, mspiRegDevCfg))
        {
            am_util_debug_printf("Error - Failed to configure Device MSPI.\n");
            return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
        }

        //
        // Re-Enable MSPI
        //
        if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_enable(pPsram->pMspiHandle))
        {
            am_util_debug_printf("Error - Failed to Enable MSPI!\n");
            return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
        }

        //
        // Re-config the MSPI pins.
        //
        am_bsp_mspi_pins_enable(pPsram->ui32Module, lcl_devCfg);
    }

    //
    // Send command to Enter half sleep
    //
    if (AM_HAL_STATUS_SUCCESS != am_device_command_write(pPsram->pMspiHandle, AM_DEVICES_MSPI_PSRAM_DDR_WRITE_REGISTER, true, 6, &ui32PIOBuffer, 1))
    {
        am_util_debug_printf("Failed to write PSRAM MR6 register!\n");
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

    // tHS min = 4us
    am_util_delay_us(APS25616N_tHS_MIN_US);

    return AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS;
}

//*****************************************************************************
//
// This function resets the device to bring it out of halfsleep
//
//*****************************************************************************
uint32_t
am_devices_mspi_psram_aps25616n_exit_halfsleep(void *pHandle)
{
    am_devices_mspi_psram_t *pPsram = (am_devices_mspi_psram_t *)pHandle;
    uint32_t pin_num = 0;
    am_hal_gpio_pincfg_t gpio_pincfg = AM_HAL_GPIO_PINCFG_DEFAULT;

    if ( pHandle == NULL )
    {
        return AM_HAL_STATUS_FAIL;
    }

    am_bsp_mspi_ce_pincfg_get(pPsram->ui32Module, pPsram->eDeviceConfig, &pin_num, &gpio_pincfg);

    //
    // Configure CE pin to output and hold high
    //
    am_hal_gpio_output_set(pin_num);
    am_hal_gpio_pinconfig(pin_num, am_hal_gpio_pincfg_output);

    //
    // Start reset pulse on CE1
    //
    am_hal_gpio_output_clear(pin_num);

    //
    // hold reset pin for 60ns - 500ns
    //
    APS25616N_tXPHS_delay(1);

    //
    // Set pin to high to finish reset
    //
    am_hal_gpio_output_set(pin_num);

    //
    // Reconfigure pin for CE on PSRAM
    //
    am_hal_gpio_pinconfig(pin_num, gpio_pincfg);

    //
    // Delay after setting pin high to allow for device to accept command
    //  and go into half sleep mode
    //
    am_util_delay_us(APS25616N_tXHS_MIN_US);

    if ( (pPsram->eDeviceConfig == AM_HAL_MSPI_FLASH_HEX_DDR_CE0) || (pPsram->eDeviceConfig == AM_HAL_MSPI_FLASH_HEX_DDR_CE1) )
    {
        //
        // Disable MSPI defore re-configuring it
        //
        if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_disable(pPsram->pMspiHandle))
        {
            am_util_debug_printf("Error - Failed to Disable MSPI.\n");
            return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
        }

        //
        // Reconfigure MSPI to HEX
        //
        am_hal_mspi_dev_config_t*    mspiMemDevCfg;
        switch (pPsram->eDeviceConfig)
        {
            case AM_HAL_MSPI_FLASH_HEX_DDR_CE0:
                mspiMemDevCfg = &APMDDRHEXCE0MSPIConfig;
                break;
            case AM_HAL_MSPI_FLASH_HEX_DDR_CE1:
                mspiMemDevCfg = &APMDDRHEXCE1MSPIConfig;
                break;
            default:
                return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
        }

        if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_device_configure(pPsram->pMspiHandle, mspiMemDevCfg))
        {
            am_util_debug_printf("Error - Failed to configure Device MSPI.\n");
            return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
        }

        //
        // Re-Enable MSPI
        //
        if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_enable(pPsram->pMspiHandle))
        {
            am_util_debug_printf("Error - Failed to Enable MSPI!\n");
            return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
        }

        //
        // Re-config the MSPI pins.
        //
        am_bsp_mspi_pins_enable(pPsram->ui32Module, pPsram->eDeviceConfig);
    }

    return AM_HAL_STATUS_SUCCESS;
}
#endif

#if (defined (__ARMCC_VERSION)) && (__ARMCC_VERSION < 6000000)
__asm void
APS25616N_tXPHS_delay( uint32_t ui32Iterations )
{
    subs    r0, #1
    bne     APS25616N_tXPHS_delay
    bx      lr
}
#elif (defined (__ARMCC_VERSION)) && (__ARMCC_VERSION >= 6000000)
void
APS25616N_tXPHS_delay( uint32_t ui32Iterations )
{
  __asm
  (
    " subs  r0, #1\n"
    " bne   APS25616N_tXPHS_delay\n"
  );
}
#elif defined(__GNUC_STDC_INLINE__)
__attribute__((naked))
void
APS25616N_tXPHS_delay( uint32_t ui32Iterations )
{
    __asm
    (
        "   subs    r0, #1\n"
        "   bne     APS25616N_tXPHS_delay\n"
        "   bx      lr\n"
    );
}
#elif defined(__IAR_SYSTEMS_ICC__)
#pragma diag_suppress = Pe940   // Suppress IAR compiler warning about missing
                                // return statement on a non-void function
__stackless inline void
APS25616N_tXPHS_delay( uint32_t ui32Iterations )
{
    __asm(" subs    r0, #1 ");
    __asm(" bne     APS25616N_tXPHS_delay ");
}
#pragma diag_default = Pe940    // Restore IAR compiler warning
#else
#error Compiler is unknown, please contact Ambiq support team
#endif
#endif
//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
