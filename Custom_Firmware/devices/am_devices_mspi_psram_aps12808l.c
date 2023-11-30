//*****************************************************************************
//
//! @file am_devices_mspi_psram_aps12808l.c
//!
//! @brief Micron Serial SPI PSRAM driver.
//!
//! @addtogroup mspi_psram_12808l APS12808L MSPI PSRAM Driver
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
#include "am_devices_mspi_psram_aps12808l.h"
#include "am_util_stdio.h"
#include "am_bsp.h"
#include "am_util.h"
#include "am_util_delay.h"

//*****************************************************************************
//
//! @name Global variables.
//! @{
//
//*****************************************************************************
//#define USE_NON_DQS_MODE

#define AM_DEVICES_MSPI_PSRAM_TIMEOUT             1000000
#define PSRAM_MR_DRIVE_STRENGTH                   1
//! @note there should be at least this amount of consecutive passing settings
//! to be accepted.
#define PSRAM_TIMING_SCAN_MIN_ACCEPTANCE_LENGTH   (8)
//! @}

//
//!
//
am_hal_mspi_xip_config_t gDDRXipConfig[] =
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

#if defined(AM_PART_APOLLO4P) || defined(AM_PART_APOLLO4L)
//
//!
//
am_hal_mspi_dqs_t gDDRDqsCfg[] =
{
    {
#ifdef USE_NON_DQS_MODE
        .bDQSEnable             = 0,
#else
        .bDQSEnable             = 1,
#endif
        .bDQSSyncNeg            = 0,
        .bEnableFineDelay       = 0,
        //
        // a non-zero value of ui8TxDQSDelay 4 was mentioned by the Validation team
        //
        .ui8TxDQSDelay          = 4,
        .ui8RxDQSDelay          = 28,
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
        //
        // a non-zero value of ui8TxDQSDelay 4 was mentioned by the Validation team
        //
        .ui8TxDQSDelay          = 4,
        .ui8RxDQSDelay          = 28,
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
        //
        // a non-zero value of ui8TxDQSDelay 4 was mentioned by the Validation team
        //
        .ui8TxDQSDelay          = 4,
        .ui8RxDQSDelay          = 28,
        .ui8RxDQSDelayNeg       = 0,
        .bRxDQSDelayNegEN       = 0,
        .ui8RxDQSDelayHi        = 0,
        .ui8RxDQSDelayNegHi     = 0,
        .bRxDQSDelayHiEN        = 0,
    }
};
#else
am_hal_mspi_dqs_t gDDRDqsCfg[] =
{
    {
#ifdef USE_NON_DQS_MODE
        .bDQSEnable             = 0,
#else
        .bDQSEnable             = 1,
#endif
        .bOverrideRXDQSDelay    = 1,
        .bEnableFineDelay       = 0,
        .ui8RxDQSDelay          = 28,
        .bOverrideTXDQSDelay    = 0,
        .ui8TxDQSDelay          = 0,
        .bDQSSyncNeg            = 0,
        .ui8DQSDelay            = 0,
    },
    {
#ifdef USE_NON_DQS_MODE
        .bDQSEnable             = 0,
#else
        .bDQSEnable             = 1,
#endif
        .bOverrideRXDQSDelay    = 1,
        .bEnableFineDelay       = 0,
        .ui8RxDQSDelay          = 28,
        .bOverrideTXDQSDelay    = 0,
        .ui8TxDQSDelay          = 0,
        .bDQSSyncNeg            = 0,
        .ui8DQSDelay            = 0,
    },
    {
#ifdef USE_NON_DQS_MODE
        .bDQSEnable             = 0,
#else
        .bDQSEnable             = 1,
#endif
        .bOverrideRXDQSDelay    = 1,
        .bEnableFineDelay       = 0,
        .ui8RxDQSDelay          = 28,
        .bOverrideTXDQSDelay    = 0,
        .ui8TxDQSDelay          = 0,
        .bDQSSyncNeg            = 0,
        .ui8DQSDelay            = 0,
    },
};

am_hal_mspi_dqs_t gDDREnableFineDelayCfg =
{
    .bDQSEnable             = 0,
    .bEnableFineDelay       = 1,
    .bOverrideRXDQSDelay    = 1,
    .ui8RxDQSDelay          = 15,
    .bOverrideTXDQSDelay    = 0,
    .ui8TxDQSDelay          = 0,
    .bDQSSyncNeg            = 0,
    .ui8DQSDelay            = 0,
    .ui8PioTurnaround       = 12,
    .ui8XipTurnaround       = 12,
    .bRxNeg                 = 0,
};
#endif

#if defined(AM_PART_APOLLO4_API)
am_hal_mspi_xip_misc_t gXipMiscCfg[] =
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
#endif

//
//!
//
am_hal_mspi_config_t gDDRMspiCfg =
{
  .ui32TCBSize          = 0,
  .pTCB                 = NULL,
  .bClkonD4             = 0
};

//
//!
//
am_hal_mspi_dev_config_t  DDROctalCE0MSPIConfig =
{
  .ui8TurnAround        = 4,
  .eAddrCfg             = AM_HAL_MSPI_ADDR_4_BYTE,
  .eInstrCfg            = AM_HAL_MSPI_INSTR_2_BYTE,
  .ui16ReadInstr        = AM_DEVICES_MSPI_PSRAM_OCTAL_DDR_READ,
  .ui16WriteInstr       = AM_DEVICES_MSPI_PSRAM_OCTAL_DDR_WRITE,
  .eDeviceConfig        = AM_HAL_MSPI_FLASH_OCTAL_DDR_CE0,
  .eSpiMode             = AM_HAL_MSPI_SPI_MODE_0,
  .eClockFreq           = AM_HAL_MSPI_CLK_96MHZ,
  .bSendAddr            = true,
  .bSendInstr           = true,
  .bTurnaround          = true,
#if defined(AM_PART_APOLLO4_API)
  .ui8WriteLatency      = 4,
  .bEnWriteLatency      = true,
  .bEmulateDDR          = true,
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

//
//!
//
am_hal_mspi_dev_config_t  DDROctalCE1MSPIConfig =
{
  .ui8TurnAround        = 4,
  .eAddrCfg             = AM_HAL_MSPI_ADDR_4_BYTE,
  .eInstrCfg            = AM_HAL_MSPI_INSTR_2_BYTE,
  .ui16ReadInstr        = AM_DEVICES_MSPI_PSRAM_OCTAL_DDR_READ,
  .ui16WriteInstr       = AM_DEVICES_MSPI_PSRAM_OCTAL_DDR_WRITE,
  .eDeviceConfig        = AM_HAL_MSPI_FLASH_OCTAL_DDR_CE1,
  .eSpiMode             = AM_HAL_MSPI_SPI_MODE_0,
  .eClockFreq           = AM_HAL_MSPI_CLK_96MHZ,
  .bSendAddr            = true,
  .bSendInstr           = true,
  .bTurnaround          = true,
#if defined(AM_PART_APOLLO4_API)
  .ui8WriteLatency      = 4,
  .bEnWriteLatency      = true,
  .bEmulateDDR          = true,
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

//
//!
//
typedef struct
{
  uint32_t                    ui32Module;
  void                        *pMspiHandle;
  am_hal_mspi_device_e        eDeviceConfig;
  bool                        bOccupied;
} am_devices_mspi_psram_t;

//
//!
//
am_devices_mspi_psram_t gDDRAmPsram[AM_DEVICES_MSPI_PSRAM_MAX_DEVICE_NUM];

//*****************************************************************************
//
//! @brief
//!
//! @param pCallbackCtxt
//! @param status
//
//*****************************************************************************
void pfnMSPI_PSRAM_DDR_Callback(void *pCallbackCtxt, uint32_t status)
{
  //! Set the DMA complete flag.
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
#if !defined(AM_PART_APOLLO4_API)
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
#if !defined(AM_PART_APOLLO4_API)
  Transaction.bQuadCmd                = false;
#else
  Transaction.bDCX                    = false;
  Transaction.bEnWRLatency            = true;
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
//! @brief Reset the external psram
//!
//! @param pMspiHandle
//!
//! @return
//
//*****************************************************************************
static uint32_t
am_devices_mspi_psram_aps12808l_reset(void *pMspiHandle)
{
  uint32_t      ui32PIOBuffer = 0;
  //
  // Global Reset DDR PSRAM.
  //
  if (AM_HAL_STATUS_SUCCESS != am_device_command_write(pMspiHandle, AM_DEVICES_MSPI_PSRAM_OCTAL_DDR_GLOBAL_RESET, true, 0, &ui32PIOBuffer, 2))
  {
    return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
  }

  return AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Reads the ID of the external psram and returns the value.
//!
//! @param pMspiHandle - Pointer to the return buffer for the Device ID.
//!
//! This function reads the device ID register of the external psram, and returns
//! the result as an 32-bit unsigned integer value.
//!
//! @return 32-bit status
//
//*****************************************************************************
static uint32_t
am_devices_mspi_psram_aps12808l_id(void *pMspiHandle)
{
  uint32_t     ui32Status;
  uint32_t     ui32Rawdata;
  uint8_t      ui8VendorIDReg = 0;
  uint8_t      ui8DeviceIDReg = 0;
  uint8_t      ui8RLCReg = 0;

  //
  // Read and set PSRAM Read Latency Code
  //

  ui32Status = am_device_command_read(pMspiHandle, AM_DEVICES_MSPI_PSRAM_OCTAL_DDR_READ_REGISTER, true, 0, &ui32Rawdata, 4);
  if (AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS != ui32Status)
  {
      am_util_debug_printf("Failed to read PSRAM Read Latency Code!\n");
      return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
  }
  else
  {
      ui8RLCReg = (uint8_t)ui32Rawdata;
  }

  ui8RLCReg   = ui8RLCReg & (~0x0000001c);  // set latency to 3 (0b000)

  ui8RLCReg &= 0xFC;
  ui8RLCReg |= PSRAM_MR_DRIVE_STRENGTH;
  ui8RLCReg |= 0x20;                        // set LT to fixed
  ui32Rawdata = ui8RLCReg & 0x000000FF;

  ui32Status = am_device_command_write(pMspiHandle, AM_DEVICES_MSPI_PSRAM_OCTAL_DDR_WRITE_REGISTER, true, 0, &ui32Rawdata, 4);
  if (AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS != ui32Status)
  {
      am_util_debug_printf("Failed to write PSRAM Read Latency Code!\n");
      return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
  }

  ui32Status = am_device_command_read(pMspiHandle, AM_DEVICES_MSPI_PSRAM_OCTAL_DDR_READ_REGISTER, true, 0, &ui32Rawdata, 4);
  if (AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS != ui32Status)
  {
      am_util_debug_printf("Failed to read PSRAM Read Latency Code!\n");
      return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
  }
  else
  {
      ui8RLCReg = (uint8_t)ui32Rawdata;
      am_util_debug_printf("\n    PSRAM Read Latency Code = 0x%X\n", ((ui8RLCReg & 0x1C)>>2) + 3 );
  }

  //
  // Read and set PSRAM Write Latency Code
  //
  ui32Status = am_device_command_read(pMspiHandle, AM_DEVICES_MSPI_PSRAM_OCTAL_DDR_READ_REGISTER, true, 4, &ui32Rawdata, 4);
  if (AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS != ui32Status)
  {
      am_util_debug_printf("Failed to read PSRAM Write Latency Code!\n");
      return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
  }
  else
  {
      ui8RLCReg = (uint8_t)ui32Rawdata;
  }

  ui32Rawdata = ui8RLCReg & (~0x000000e0);

  ui32Status = am_device_command_write(pMspiHandle, AM_DEVICES_MSPI_PSRAM_OCTAL_DDR_WRITE_REGISTER, true, 4, &ui32Rawdata, 4);
  if (AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS != ui32Status)
  {
      am_util_debug_printf("Failed to write PSRAM Write Latency Code!\n");
      return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
  }

  ui32Status = am_device_command_read(pMspiHandle, AM_DEVICES_MSPI_PSRAM_OCTAL_DDR_READ_REGISTER, true, 4, &ui32Rawdata, 4);
  if (AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS != ui32Status)
  {
      am_util_debug_printf("Failed to read PSRAM Write Latency Code!\n");
      return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
  }
  else
  {
    ui8RLCReg = (uint8_t)ui32Rawdata;
    am_util_debug_printf("    PSRAM Write Latency Code = 0x%X\n", ((ui8RLCReg & 0xE0)>>5) + 3 );
  }

  //
  // Read PSRAM Vendor ID and Device ID and return status.
  //
  ui32Status = am_device_command_read(pMspiHandle, AM_DEVICES_MSPI_PSRAM_OCTAL_DDR_READ_REGISTER, true, 1, &ui32Rawdata, 4);
  if (AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS != ui32Status)
  {
      am_util_debug_printf("Failed to read PSRAM Vendor ID!\n");
      return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
  }
  else
  {
    ui8VendorIDReg = (uint8_t)ui32Rawdata;
    if ( (ui8VendorIDReg & 0x1F) == 0xD )
    {
      am_util_debug_printf("    PSRAM Vendor ID =  01101\n");
    }
  }

  ui32Status = am_device_command_read(pMspiHandle, AM_DEVICES_MSPI_PSRAM_OCTAL_DDR_READ_REGISTER, true, 2, &ui32Rawdata, 4);
  if (AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS != ui32Status)
  {
      am_util_debug_printf("Failed to read PSRAM Device ID!\n");
      return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
  }
  else
  {
    ui8DeviceIDReg = (uint8_t)ui32Rawdata;
    am_util_debug_printf("    PSRAM Device ID =  Generation %d\n", ((ui8DeviceIDReg & 0x18) >> 3) + 1);
    if ( (ui8DeviceIDReg & 0x7) == 0x1 )
    {
      am_util_debug_printf("    PSRAM Density =  32Mb\n");
    }
    else if ( (ui8DeviceIDReg & 0x7) == 0x3 )
    {
      am_util_debug_printf("    PSRAM Density =  64Mb\n");
    }
    else if ( (ui8DeviceIDReg & 0x7) == 0x5 )
    {
      am_util_debug_printf("    PSRAM Density =  128Mb\n");
    }
    else if ( (ui8DeviceIDReg & 0x7) == 0x7 )
    {
      am_util_debug_printf("    PSRAM Density =  256Mb\n");
    }
  }
    //
    // read and print all MR registers
    //
    // MR3
    ui32Rawdata = 0;
    ui32Status = am_device_command_read(pMspiHandle, AM_DEVICES_MSPI_PSRAM_OCTAL_DDR_READ_REGISTER, true, 3, &ui32Rawdata, 4);
    ui8RLCReg = (uint8_t)ui32Rawdata;
    am_util_debug_printf("    PSRAM Register MR3 = 0x%02X\n", ui8RLCReg);

    // MR6
    ui32Rawdata = 0;
    ui32Status = am_device_command_read(pMspiHandle, AM_DEVICES_MSPI_PSRAM_OCTAL_DDR_READ_REGISTER, true, 6, &ui32Rawdata, 4);
    ui8RLCReg = (uint8_t)ui32Rawdata;
    am_util_debug_printf("    PSRAM Register MR6 = 0x%02X\n", ui8RLCReg);

    // MR8
    ui32Rawdata = 0;
    ui32Status = am_device_command_read(pMspiHandle, AM_DEVICES_MSPI_PSRAM_OCTAL_DDR_READ_REGISTER, true, 8, &ui32Rawdata, 4);
    ui8RLCReg = (uint8_t)ui32Rawdata;
    am_util_debug_printf("    PSRAM Register MR8 = 0x%02X\n\n", ui8RLCReg);

    return AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief  This function takes care of splitting the transaction as needed
//! @details if the transaction crosses PSRAM page boundary or because of
//! tCEM restrictions, if hardware does not support it
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
#if defined(AM_PART_APOLLO4)
    Transaction.eDeviceNum         = AM_HAL_MSPI_DEVICE0;
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
am_devices_mspi_psram_aps12808l_ddr_init(uint32_t ui32Module,
                                         am_devices_mspi_psram_config_t *pDevCfg,
                                         void **ppHandle,
                                         void **ppMspiHandle)
{
    uint32_t                    ui32Status;
    am_hal_mspi_dev_config_t    mspiDevCfg;
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
        if ( gDDRAmPsram[ui32Index].bOccupied == false )
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
        case AM_HAL_MSPI_FLASH_OCTAL_DDR_CE0:
            mspiDevCfg = DDROctalCE0MSPIConfig;
            break;
        case AM_HAL_MSPI_FLASH_OCTAL_DDR_CE1:
            mspiDevCfg = DDROctalCE1MSPIConfig;
            break;
        default:
            return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }
    mspiDevCfg.eDeviceConfig = pDevCfg->eDeviceConfig;
    mspiDevCfg.eClockFreq = pDevCfg->eClockFreq;
#if !defined(AM_PART_APOLLO4_API)
    mspiDevCfg.ui32TCBSize = pDevCfg->ui32NBTxnBufLength;
    mspiDevCfg.pTCB = pDevCfg->pNBTxnBuf;
    mspiDevCfg.scramblingStartAddr = pDevCfg->ui32ScramblingStartAddr;
    mspiDevCfg.scramblingEndAddr = pDevCfg->ui32ScramblingEndAddr;
#endif

    // First configure in Octal mode and reset
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

    am_hal_mspi_config_t    mspiCfg = gDDRMspiCfg;
    mspiCfg.ui32TCBSize = pDevCfg->ui32NBTxnBufLength;
    mspiCfg.pTCB = pDevCfg->pNBTxnBuf;
    if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_configure(pMspiHandle, &mspiCfg))
    {
        am_util_debug_printf("Error - Failed to configure MSPI device.\n");
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

    if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_device_configure(pMspiHandle, &mspiDevCfg))
    {
        am_util_debug_printf("Error - Failed to configure MSPI device.\n");
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

    am_hal_mspi_xip_config_t    xipCfg = gDDRXipConfig[ui32Module];
#if defined(AM_PART_APOLLO4)
    xipCfg.eDeviceNum = AM_HAL_MSPI_DEVICE0;
#endif
    xipCfg.scramblingStartAddr = pDevCfg->ui32ScramblingStartAddr;
    xipCfg.scramblingEndAddr = pDevCfg->ui32ScramblingEndAddr;
    ui32Status = am_hal_mspi_control(pMspiHandle, AM_HAL_MSPI_REQ_XIP_CONFIG, &xipCfg);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
      return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

#if defined(AM_PART_APOLLO4B)
    am_hal_mspi_xip_misc_t    xipMiscCfg = gXipMiscCfg[ui32Module];
    ui32Status = am_hal_mspi_control(pMspiHandle, AM_HAL_MSPI_REQ_XIP_MISC_CONFIG, &xipMiscCfg);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
      return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }
#endif

    am_hal_mspi_dqs_t dqsCfg = gDDRDqsCfg[ui32Module];
    ui32Status = am_hal_mspi_control(pMspiHandle, AM_HAL_MSPI_REQ_DQS, &dqsCfg);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
      return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

    //
    // Enable DDR emulation in MSPI
    //
    ui32Status = am_hal_mspi_control(pMspiHandle, AM_HAL_MSPI_REQ_DDR_EN, NULL);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
      return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

    if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_enable(pMspiHandle))
    {
        am_util_debug_printf("Error - Failed to enable MSPI.\n");
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

    am_bsp_mspi_pins_enable(ui32Module, mspiDevCfg.eDeviceConfig);

    am_util_delay_us(150);

    if (AM_HAL_STATUS_SUCCESS != am_devices_mspi_psram_aps12808l_reset(pMspiHandle))
    {
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

    am_util_delay_us(2);

    if (AM_HAL_STATUS_SUCCESS != am_devices_mspi_psram_aps12808l_id(pMspiHandle))
    {
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

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
    gDDRAmPsram[ui32Index].bOccupied = true;
    *ppHandle = (void *)&gDDRAmPsram[ui32Index];
    *ppMspiHandle = gDDRAmPsram[ui32Index].pMspiHandle = pMspiHandle;
    gDDRAmPsram[ui32Index].ui32Module = ui32Module;
    gDDRAmPsram[ui32Index].eDeviceConfig = mspiDevCfg.eDeviceConfig;

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
am_devices_mspi_psram_aps12808l_ddr_deinit(void *pHandle)
{
    uint32_t    ui32Status;
    am_devices_mspi_psram_t *pPsram = (am_devices_mspi_psram_t *)pHandle;

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
am_devices_mspi_psram_aps12808l_ddr_read(void *pHandle,
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
                                              pfnMSPI_PSRAM_DDR_Callback,
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
//  Reads the contents of the external PSRAM into a buffer.
//
//*****************************************************************************
uint32_t
am_devices_mspi_psram_aps12808l_ddr_read_adv(void *pHandle,
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
// Reads the contents of the external psram into a buffer.
//
//*****************************************************************************
uint32_t
am_devices_mspi_psram_aps12808l_ddr_read_hiprio(void *pHandle,
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
//  nonblocking ddr read
//
//*****************************************************************************
uint32_t
am_devices_mspi_psram_aps12808l_ddr_nonblocking_read(void *pHandle,
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
// Programs the given range of psram addresses.
//
//*****************************************************************************
uint32_t
am_devices_mspi_psram_aps12808l_ddr_write(void *pHandle,
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
                                            pfnMSPI_PSRAM_DDR_Callback,
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
//  Programs the given range of psram addresses.
//
//*****************************************************************************
uint32_t
am_devices_mspi_psram_aps12808l_ddr_write_adv(void *pHandle,
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
// Programs the given range of psram addresses.
//
//*****************************************************************************
uint32_t
am_devices_mspi_psram_aps12808l_ddr_write_hiprio(void *pHandle,
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
// nonblocking write
//
//*****************************************************************************
uint32_t
am_devices_mspi_psram_aps12808l_ddr_nonblocking_write(void *pHandle,
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
am_devices_mspi_psram_aps12808l_ddr_enable_xip(void *pHandle)
{
  uint32_t ui32Status;
  am_devices_mspi_psram_t *pPsram = (am_devices_mspi_psram_t *)pHandle;

  //
  // Set Aperture XIP range
  //
  ui32Status = am_hal_mspi_control(pPsram->pMspiHandle, AM_HAL_MSPI_REQ_XIP_CONFIG, &gDDRXipConfig[pPsram->ui32Module]);
  if (AM_HAL_STATUS_SUCCESS != ui32Status)
  {
    return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
  }

  //
  // Enable XIP on the MSPI.
  //
  ui32Status = am_hal_mspi_control(pPsram->pMspiHandle, AM_HAL_MSPI_REQ_XIP_EN, &gDDRXipConfig[pPsram->ui32Module]);
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
am_devices_mspi_psram_aps12808l_ddr_disable_xip(void *pHandle)
{
  uint32_t ui32Status;
  am_devices_mspi_psram_t *pPsram = (am_devices_mspi_psram_t *)pHandle;

  //
  // Disable XIP on the MSPI.
  //
  ui32Status = am_hal_mspi_control(pPsram->pMspiHandle, AM_HAL_MSPI_REQ_XIP_DIS, &gDDRXipConfig[pPsram->ui32Module]);
  if (AM_HAL_STATUS_SUCCESS != ui32Status)
  {
    return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
  }

  return AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS;
}

//*****************************************************************************
//
//  Sets up the MSPI and external psram into scrambling mode.
//
//*****************************************************************************
uint32_t
am_devices_mspi_psram_aps12808l_ddr_enable_scrambling(void *pHandle)
{
  uint32_t ui32Status;
  am_devices_mspi_psram_t *pPsram = (am_devices_mspi_psram_t *)pHandle;

  //
  // Enable scrambling on the MSPI.
  //
  ui32Status = am_hal_mspi_control(pPsram->pMspiHandle, AM_HAL_MSPI_REQ_SCRAMB_EN, &gDDRXipConfig[pPsram->ui32Module]);
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
am_devices_mspi_psram_aps12808l_ddr_disable_scrambling(void *pHandle)
{
  uint32_t ui32Status;
  am_devices_mspi_psram_t *pPsram = (am_devices_mspi_psram_t *)pHandle;

  //
  // Disable Scrambling on the MSPI.
  //
  ui32Status = am_hal_mspi_control(pPsram->pMspiHandle, AM_HAL_MSPI_REQ_SCRAMB_DIS, &gDDRXipConfig[pPsram->ui32Module]);
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
am_devices_mspi_psram_aps12808l_ddr_reset(void *pHandle)
{
  am_devices_mspi_psram_t *pPsram = (am_devices_mspi_psram_t *)pHandle;
  return am_devices_mspi_psram_aps12808l_reset(pPsram);
}

//*****************************************************************************
//
// Reads the ID of the external psram and returns the value.
//
//*****************************************************************************
uint32_t
am_devices_mspi_psram_aps12808l_ddr_id(void *pHandle)
{
  am_devices_mspi_psram_t *pPsram = (am_devices_mspi_psram_t *)pHandle;

  return am_devices_mspi_psram_aps12808l_id(pPsram);
}

#if defined(AM_PART_APOLLO4) || defined(AM_PART_APOLLO4B)
#define PSRAM_CHECK_DATA_SIZE_BYTES  256
//*****************************************************************************
//
//! @brief write and read back check.
//!
//! @param pattern_index -
//! @param len           -
//!
//! This function should be called before any other am_devices_mspi_psram
//! functions. It is used to set tell the other functions how to communicate
//! with the external psram hardware.
//!
//! @return status.
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
//! @param length
//! @param address
//!
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
//! @brief   Count the longest consecutive 1s in a 32bit word
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
//! @brief   Count the longest consecutive 1s in a 32bit word
//! @details Static helper function:
//!
//!  Find and return the mid point of the longest continuous 1s in a 32bit word
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

const am_devices_mspi_psram_ddr_timing_config_t aps12808l_sConfigArray[] =
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
    {13, 0, 1}, // Turnaround=13, RXNEG=0, RXDQSDELAY=Dummy
    {13, 1, 1}, // Turnaround=13, RXNEG=1, RXDQSDELAY=Dummy
};

//*****************************************************************************
//
//  Checks PSRAM timing and determine a delay setting.
//
//
//*****************************************************************************
uint32_t
am_devices_mspi_psram_aps12808l_ddr_init_timing_check(uint32_t module,
                                                      am_devices_mspi_psram_config_t *pDevCfg,
                                                      am_devices_mspi_psram_ddr_timing_config_t *pDevDdrCfg)
{
    uint32_t ui32Status;
    void *pDevHandle;
    void *pHandle;

    uint32_t ui32ResultArray[sizeof(aps12808l_sConfigArray) / sizeof(am_devices_mspi_psram_ddr_timing_config_t)] = {0};
    const uint32_t ui32TestSize = sizeof(aps12808l_sConfigArray) / sizeof(am_devices_mspi_psram_ddr_timing_config_t);

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
        .ui8PioTurnaround       = 12,
        .ui8XipTurnaround       = 12,
        .bRxNeg                 = 0,
    };

    //
    // initialize interface
    //
    am_hal_mspi_dev_config_t    *psMSPISettings;
    switch (pDevCfg->eDeviceConfig)
    {
        case AM_HAL_MSPI_FLASH_OCTAL_DDR_CE0:
            psMSPISettings = &DDROctalCE0MSPIConfig;
            break;
        case AM_HAL_MSPI_FLASH_OCTAL_DDR_CE1:
            psMSPISettings = &DDROctalCE1MSPIConfig;
            break;
        default:
            return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

    uint16_t timeLimit = psMSPISettings->ui16DMATimeLimit;                  // save original setting here
    am_hal_mspi_dma_boundary_e dmaBound0 = psMSPISettings->eDMABoundary;    // save original setting here
    psMSPISettings->ui16DMATimeLimit    = 0;
    psMSPISettings->eDMABoundary        = AM_HAL_MSPI_BOUNDARY_NONE;
    ui32Status = am_devices_mspi_psram_aps12808l_ddr_init(module, pDevCfg, &pDevHandle, &pHandle);
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
    ui32Status = am_devices_mspi_psram_aps12808l_ddr_enable_xip(pDevHandle);
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
        scanCfg.ui8PioTurnaround    = scanCfg.ui8XipTurnaround = aps12808l_sConfigArray[i].ui32Turnaround;
        scanCfg.bRxNeg              = aps12808l_sConfigArray[i].ui32Rxneg;
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
        am_util_debug_printf(" Turnaround %d - RXNEG %d = 0x%08X\n", aps12808l_sConfigArray[i].ui32Turnaround, aps12808l_sConfigArray[i].ui32Rxneg, ui32ResultArray[i]);

    }

    am_util_debug_printf("Timing Scan found a window %d fine steps wide.\n", ui32MaxOnes);

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
    am_devices_mspi_psram_aps12808l_ddr_deinit(pDevHandle);

    //
    // Set output values
    //
    pDevDdrCfg->ui32Rxdqsdelay = dqsdelay;
    pDevDdrCfg->ui32Rxneg = aps12808l_sConfigArray[ui32MaxOnesIndex].ui32Rxneg;
    pDevDdrCfg->ui32Turnaround = aps12808l_sConfigArray[ui32MaxOnesIndex].ui32Turnaround;

    return AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS;
}

//*****************************************************************************
//
//  Apply given DDR timing settings to target MSPI instance.
//
//*****************************************************************************
uint32_t
am_devices_mspi_psram_aps12808l_apply_ddr_timing(void *pHandle,
                                                 am_devices_mspi_psram_ddr_timing_config_t *pDevDdrCfg)
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
    applyCfg.ui8RxDQSDelay      = pDevDdrCfg->ui32Rxdqsdelay;
    applyCfg.ui8PioTurnaround   = pDevDdrCfg->ui32Turnaround;
    applyCfg.ui8XipTurnaround   = pDevDdrCfg->ui32Turnaround;
    applyCfg.bRxNeg             = pDevDdrCfg->ui32Rxneg;

    return am_hal_mspi_control(pPsram->pMspiHandle, AM_HAL_MSPI_REQ_DQS, &applyCfg);

}
#elif defined(AM_PART_APOLLO4P) || defined(AM_PART_APOLLO4L)
#define PSRAM_CHECK_DATA_SIZE_BYTES  256
//*****************************************************************************
//
//! @brief write and read back check.
//!
//! @param pattern_index
//! @param buff
//! @param len
//!
//! This function should be called before any other am_devices_mspi_psram
//! functions. It is used to set tell the other functions how to communicate
//! with the external psram hardware.
//!
//! @return status.
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
        ui32Status = am_devices_mspi_psram_aps12808l_ddr_write(flashHandle, ui8TxBuffer,
                                            address,
                                            ui32TestBytes, true);
        if ( ui32Status ==  AM_DEVICES_MSPI_PSRAM_STATUS_ERROR)
        {
            return true;
        }
        ui32Status = am_devices_mspi_psram_aps12808l_ddr_read(flashHandle, ui8RxBuffer,
                                                address,
                                                ui32TestBytes, true);
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

// ****************************************************************************
//
//! @brief Count the longest consecutive 1s in a 32bit word
//!
//! @details Static helper function:
//! @param pVal
//!
//! @return
//
// ****************************************************************************
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

// ****************************************************************************
//
//! @brief Find and return the mid point of the longest continuous 1s in a 32bit word
//!
//! @details Static helper function:
//! @param pVal
//!
//! @return
//
// ****************************************************************************
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

#define PSRAM_TIMING_SCAN_SIZE_BYTES (128*1024)
static const uint32_t ui32MspiXipBaseAddress[3] =
{
    0x14000000, // mspi0
    0x18000000, // mspi1
    0x1C000000, // mspi2
};

//*****************************************************************************
//
// Checks PSRAM timing and determine a delay setting.
//
//*****************************************************************************
uint32_t
am_devices_mspi_psram_aps12808l_ddr_init_timing_check(uint32_t module,
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

    am_hal_mspi_dqs_t scanCfg = gDDRDqsCfg[module];
    //
    // initialize interface
    //
    am_hal_mspi_dev_config_t    *psMSPISettings;
    switch (pDevCfg->eDeviceConfig)
    {
        case AM_HAL_MSPI_FLASH_OCTAL_DDR_CE0:
            psMSPISettings = &DDROctalCE0MSPIConfig;
            break;
        case AM_HAL_MSPI_FLASH_OCTAL_DDR_CE1:
            psMSPISettings = &DDROctalCE1MSPIConfig;
            break;

        default:
            return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

    uint16_t timeLimit = psMSPISettings->ui16DMATimeLimit;                  // save original setting here
    am_hal_mspi_dma_boundary_e dmaBound0 = psMSPISettings->eDMABoundary;    // save original setting here
    psMSPISettings->ui16DMATimeLimit    = 0;
    psMSPISettings->eDMABoundary        = AM_HAL_MSPI_BOUNDARY_NONE;
    ui32Status = am_devices_mspi_psram_aps12808l_ddr_init(module, pDevCfg, &pDevHandle, &pHandle);
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
    ui32Status = am_devices_mspi_psram_aps12808l_ddr_enable_xip(pDevHandle);
    if (AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS != ui32Status)
    {
        am_util_debug_printf("    Failed to disable XIP mode in the MSPI!\n");
        return ui32Status;
    }

#if defined(AM_PART_APOLLO4P)
    if ( module == 2 )
    {
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
    am_devices_mspi_psram_aps12808l_ddr_deinit(pDevHandle);

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
// Apply given DDR timing settings to target MSPI instance.
//
//*****************************************************************************
uint32_t
am_devices_mspi_psram_aps12808l_apply_ddr_timing(void *pHandle,
                                                 am_devices_mspi_psram_ddr_timing_config_t *pDevDdrCfg)
{
    am_devices_mspi_psram_t *pPsram = (am_devices_mspi_psram_t *)pHandle;

    am_hal_mspi_dqs_t applyCfg = gDDRDqsCfg[pPsram->ui32Module];
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

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************

