//*****************************************************************************
//
//! @file am_devices_mspi_rm67162.c
//!
//! @brief Generic Raydium TFT display driver.
//!
//! @addtogroup mspi_rm67162 RM67162 MSPI Display Driver
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
#include "am_devices_mspi_rm67162.h"
#include "am_bsp.h"
#include "am_util_delay.h"
#include "am_util.h"

//*****************************************************************************
//
// Global variables.
//
//*****************************************************************************
#define BYTE_NUM_PER_WRITE  65535
//#define BYTE_NUM_PER_WRITE              AM_HAL_MSPI_MAX_TRANS_SIZE
#define AM_DEVICES_MSPI_TIMEOUT         1000000

static struct
{
    uint32_t row_start;
    uint32_t row_end;
    uint32_t col_start;
    uint32_t col_end;
} gs_display_info;

static am_devices_rm67162_graphic_conf_t g_sGraphic_conf =
{
    .bus_mode       = AM_DEVICES_RM67162_SPI_WRAM,
    .color_mode     = AM_DEVICES_RM67162_COLOR_MODE_8BIT,
    .scan_mode      = AM_DEVICES_RM67162_SCAN_MODE_0,
    .max_row        = 400, //390,
    .max_col        = 400, // 390,
    .row_offset     = 0,
    .col_offset     = 0 // 6
//    .col_offset     = 100
};

//! Display MSPI configuration
static am_hal_mspi_dev_config_t  SerialDisplayMSPICfg =
{
    .ui8TurnAround        = 1,
    .eAddrCfg             = AM_HAL_MSPI_ADDR_3_BYTE,
    .eInstrCfg            = AM_HAL_MSPI_INSTR_1_BYTE,
    .ui8ReadInstr         = AM_DEVICES_RM67162_MEMORY_READ,
    .ui8WriteInstr        = AM_DEVICES_RM67162_MEMORY_WRITE_CONTINUE, // AM_DEVICES_RM67162_MEMORY_WRITE_CONTINUE,
    .eDeviceConfig        = AM_HAL_MSPI_FLASH_SERIAL_CE0,
    .ui8WriteLatency      = 0,
    .eSpiMode             = AM_HAL_MSPI_SPI_MODE_0,
    .eClockFreq           = AM_HAL_MSPI_CLK_48MHZ,
    .bEnWriteLatency      = false,
    .bSendAddr            = false,
    .bSendInstr           = true,
    .bTurnaround          = false,
    .bEmulateDDR          = false,
    .ui16DMATimeLimit     = 0,
    .eDMABoundary         = AM_HAL_MSPI_BOUNDARY_NONE,
    .ui32TCBSize          = 0,
    .pTCB                 = 0,
    .scramblingStartAddr  = 0,
    .scramblingEndAddr    = 0,
};

typedef struct
{
    uint32_t                    ui32Module;
    am_hal_mspi_clock_e         eClockFreq;
    void                        *pMspiHandle;
    bool                        bOccupied;
} am_devices_mspi_rm67162_t;

am_devices_mspi_rm67162_t gAmRm67162[AM_DEVICES_MSPI_RM67162_MAX_DEVICE_NUM];

am_hal_mspi_clock_e g_MaxReadFreq = AM_HAL_MSPI_CLK_8MHZ;

//*****************************************************************************
//
//! @brief
//! @param pCallbackCtxt
//! @param status
//
//*****************************************************************************
void pfnMSPI_RM67162_Callback(void *pCallbackCtxt, uint32_t status)
{
    // Set the DMA complete flag.
    *(volatile bool *)pCallbackCtxt = true;
}

//*****************************************************************************
//
//
//*****************************************************************************
uint32_t
am_devices_rm67162_command_write(void *pHandle,
                                 uint32_t ui32Instr,
                                 uint8_t *pData,
                                 uint32_t ui32NumBytes)
{
  am_hal_mspi_pio_transfer_t  Transaction;
  am_devices_mspi_rm67162_t *pDisplay = (am_devices_mspi_rm67162_t *)pHandle;

  // Create the individual write transaction.
  Transaction.ui32NumBytes            = ui32NumBytes;
  Transaction.bScrambling             = false;
  Transaction.bDCX                    = true;
  Transaction.eDirection              = AM_HAL_MSPI_TX;
  Transaction.bSendAddr               = false;
  Transaction.ui32DeviceAddr          = 0;
  Transaction.bSendInstr              = true;
  Transaction.ui16DeviceInstr         = ui32Instr;
  Transaction.bTurnaround             = false;
  Transaction.bEnWRLatency            = false;
  Transaction.bQuadCmd                = false;
  Transaction.bContinue               = false;
  Transaction.pui32Buffer             = (uint32_t *)pData;

  //
  // Execute the transction over MSPI.
  //
  if (am_hal_mspi_blocking_transfer(pDisplay->pMspiHandle, &Transaction, AM_DEVICES_MSPI_TIMEOUT))
  {
    return AM_DEVICES_RM67162_STATUS_ERROR;
  }

  return AM_DEVICES_RM67162_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief
//! @param pHandle
//! @param ui32Instr
//! @param pData
//! @param ui32NumBytes
//! @param bTurnaround
//! @return
//
//*****************************************************************************
static uint32_t
am_devices_rm67162_command_read(void *pHandle,
                                uint32_t ui32Instr,
                                uint32_t *pData,
                                uint32_t ui32NumBytes,
                                bool bTurnaround)
{
  am_hal_mspi_pio_transfer_t  Transaction;
  am_devices_mspi_rm67162_t *pDisplay = (am_devices_mspi_rm67162_t *)pHandle;
  uint32_t        ui32Status = AM_DEVICES_RM67162_STATUS_SUCCESS;

  // Create the individual write transaction.
  Transaction.ui32NumBytes            = ui32NumBytes;
  Transaction.bScrambling             = false;
  Transaction.bDCX                    = true;
  Transaction.eDirection              = AM_HAL_MSPI_RX;
  Transaction.bSendAddr               = false;
  Transaction.ui32DeviceAddr          = 0;
  Transaction.bSendInstr              = true;
  Transaction.ui16DeviceInstr         = ui32Instr;
  Transaction.bTurnaround             = bTurnaround;
  Transaction.bEnWRLatency            = false;
  Transaction.bQuadCmd                = false;
  Transaction.bContinue               = false;
  Transaction.pui32Buffer             = pData;

  am_hal_mspi_control(pDisplay->pMspiHandle, AM_HAL_MSPI_REQ_CLOCK_CONFIG, &g_MaxReadFreq);

  //
  // Execute the transction over MSPI.
  //
  if (am_hal_mspi_blocking_transfer(pDisplay->pMspiHandle,
                                    &Transaction,
                                    AM_DEVICES_MSPI_TIMEOUT))
  {
    ui32Status = AM_DEVICES_RM67162_STATUS_ERROR;
  }
  am_hal_mspi_control(pDisplay->pMspiHandle, AM_HAL_MSPI_REQ_CLOCK_CONFIG, &pDisplay->eClockFreq);
  return ui32Status;
}

//*****************************************************************************
//
// Reads the current status of the external display
//
//*****************************************************************************
uint32_t
am_devices_rm67162_reset(void *pHandle)
{
    // Hardware Reset
#if 1
    am_hal_gpio_state_write(AM_BSP_GPIO_DISPLAY_RESET, AM_HAL_GPIO_OUTPUT_SET);
    am_util_delay_ms(20);
    am_hal_gpio_state_write(AM_BSP_GPIO_DISPLAY_RESET, AM_HAL_GPIO_OUTPUT_CLEAR);
    am_util_delay_ms(20);
    am_hal_gpio_state_write(AM_BSP_GPIO_DISPLAY_RESET, AM_HAL_GPIO_OUTPUT_SET);
    am_util_delay_ms(20);
#endif
    if ( am_devices_rm67162_command_write(pHandle, AM_DEVICES_RM67162_SWRESET, NULL, 0) )
    {
        return AM_DEVICES_RM67162_STATUS_ERROR;
    }
    am_util_delay_ms(300);

    return AM_DEVICES_RM67162_STATUS_SUCCESS;
}

//*****************************************************************************
//
//
//*****************************************************************************
uint32_t
am_devices_rm67162_display_off(void *pHandle)
{
    if ( am_devices_rm67162_command_write(pHandle, AM_DEVICES_RM67162_DISPLAY_OFF, NULL, 0) )
    {
        return AM_DEVICES_RM67162_STATUS_ERROR;
    }

    if ( am_devices_rm67162_command_write(pHandle, AM_DEVICES_RM67162_SLEEP_IN, NULL, 0) )
    {
        return AM_DEVICES_RM67162_STATUS_ERROR;
    }

    return AM_DEVICES_RM67162_STATUS_SUCCESS;
}

//*****************************************************************************
//
//
//*****************************************************************************
uint32_t
am_devices_rm67162_display_on(void *pHandle)
{
    if ( am_devices_rm67162_command_write(pHandle, AM_DEVICES_RM67162_DISPLAY_ON, NULL, 0) )
    {
        return AM_DEVICES_RM67162_STATUS_ERROR;
    }

    return AM_DEVICES_RM67162_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Programs the given range of flash addresses.
//
//*****************************************************************************
uint32_t
am_devices_rm67162_blocking_write(void *pHandle,
                                  uint8_t *pui8TxBuffer,
                                  uint32_t ui32NumBytes)
{
  am_hal_mspi_pio_transfer_t  Transaction;
  am_devices_mspi_rm67162_t *pDisplay = (am_devices_mspi_rm67162_t *)pHandle;

  // Create the individual write transaction.
  Transaction.ui32NumBytes              = 0;
  Transaction.bScrambling               = false;
  Transaction.bDCX                      = true;
  Transaction.eDirection                = AM_HAL_MSPI_TX;
  Transaction.bSendAddr                 = false;
  Transaction.ui32DeviceAddr            = 0;
  Transaction.bSendInstr                = true;
  Transaction.ui16DeviceInstr           = AM_DEVICES_RM67162_MEMORY_WRITE;
  Transaction.bTurnaround               = false;
  Transaction.bEnWRLatency              = false;
  Transaction.bQuadCmd                  = false;
  Transaction.bContinue                 = true;
  Transaction.pui32Buffer               = NULL;

  //
  // Execute the transction over MSPI.
  //
  if (am_hal_mspi_blocking_transfer(pDisplay->pMspiHandle, &Transaction, AM_DEVICES_MSPI_TIMEOUT))
  {
    return AM_DEVICES_RM67162_STATUS_ERROR;
  }

  while (ui32NumBytes)
  {
    // Create the individual write transaction.
    Transaction.ui32NumBytes            = (ui32NumBytes > BYTE_NUM_PER_WRITE) ? BYTE_NUM_PER_WRITE : ui32NumBytes;
    Transaction.bScrambling             = false;
    Transaction.bDCX                    = false;
    Transaction.eDirection              = AM_HAL_MSPI_TX;
    Transaction.bSendAddr               = false;
    Transaction.ui32DeviceAddr          = 0;
    Transaction.bSendInstr              = false;
    Transaction.ui16DeviceInstr         = 0;
    Transaction.bTurnaround             = false;
    Transaction.bEnWRLatency            = false;
    Transaction.bQuadCmd                = false;
    Transaction.bContinue               = (ui32NumBytes > BYTE_NUM_PER_WRITE) ? true : false;
    Transaction.pui32Buffer             = (uint32_t *)pui8TxBuffer;

    //
    // Execute the transction over MSPI.
    //
    if (am_hal_mspi_blocking_transfer(pDisplay->pMspiHandle, &Transaction, AM_DEVICES_MSPI_TIMEOUT))
    {
      return AM_DEVICES_RM67162_STATUS_ERROR;
    }

    ui32NumBytes -= Transaction.ui32NumBytes;
    pui8TxBuffer += Transaction.ui32NumBytes;
  }
  return AM_DEVICES_RM67162_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Reads the contents of the fram into a buffer.
//
//*****************************************************************************
uint32_t
am_devices_rm67162_blocking_read(void *pHandle,
                                 uint8_t *pui8RxBuffer,
                                 uint32_t ui32NumBytes)
{
  am_hal_mspi_pio_transfer_t  Transaction;
  am_devices_mspi_rm67162_t *pDisplay = (am_devices_mspi_rm67162_t *)pHandle;

  // Create the individual write transaction.
  Transaction.ui32NumBytes            = 0;
  Transaction.bScrambling             = false;
  Transaction.bDCX                    = true;
  Transaction.eDirection              = AM_HAL_MSPI_RX;
  Transaction.bSendAddr               = false;
  Transaction.ui32DeviceAddr          = 0;
  Transaction.bSendInstr              = true;
  Transaction.ui16DeviceInstr         = AM_DEVICES_RM67162_MEMORY_READ;
  Transaction.bTurnaround             = false;
  Transaction.bEnWRLatency            = false;
  Transaction.bQuadCmd                = false;
  Transaction.bContinue               = true;
  Transaction.pui32Buffer             = NULL;

  am_hal_mspi_control(pDisplay->pMspiHandle, AM_HAL_MSPI_REQ_CLOCK_CONFIG, &g_MaxReadFreq);

  //
  // Execute the transction over MSPI.
  //
  if (am_hal_mspi_blocking_transfer(pDisplay->pMspiHandle, &Transaction, AM_DEVICES_MSPI_TIMEOUT))
  {
    am_hal_mspi_control(pDisplay->pMspiHandle, AM_HAL_MSPI_REQ_CLOCK_CONFIG, &pDisplay->eClockFreq);
    return AM_DEVICES_RM67162_STATUS_ERROR;
  }

  while (ui32NumBytes)
  {
    // Create the individual write transaction.
    Transaction.ui32NumBytes            = (ui32NumBytes > BYTE_NUM_PER_WRITE) ? BYTE_NUM_PER_WRITE : ui32NumBytes;
    Transaction.bScrambling             = false;
    Transaction.bDCX                    = false;
    Transaction.eDirection              = AM_HAL_MSPI_RX;
    Transaction.bSendAddr               = false;
    Transaction.ui32DeviceAddr          = 0;
    Transaction.bSendInstr              = false;
    Transaction.ui16DeviceInstr         = 0;
    Transaction.bTurnaround             = false;
    Transaction.bEnWRLatency            = false;
    Transaction.bQuadCmd                = false;
    Transaction.bContinue               = (ui32NumBytes > BYTE_NUM_PER_WRITE) ? true : false;
    Transaction.pui32Buffer             = (uint32_t *)pui8RxBuffer;

    //
    // Execute the transction over MSPI.
    //
    if (am_hal_mspi_blocking_transfer(pDisplay->pMspiHandle,
                                      &Transaction,
                                      AM_DEVICES_MSPI_TIMEOUT))
    {
      am_hal_mspi_control(pDisplay->pMspiHandle, AM_HAL_MSPI_REQ_CLOCK_CONFIG, &pDisplay->eClockFreq);
      return AM_DEVICES_RM67162_STATUS_ERROR;
    }

    ui32NumBytes -= Transaction.ui32NumBytes;
    pui8RxBuffer += Transaction.ui32NumBytes;
  }
  am_hal_mspi_control(pDisplay->pMspiHandle, AM_HAL_MSPI_REQ_CLOCK_CONFIG, &pDisplay->eClockFreq);
  return AM_DEVICES_RM67162_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Programs the given range of display addresses.
//
//*****************************************************************************
uint32_t
am_devices_rm67162_nonblocking_write(void *pHandle,
                                     uint8_t *pui8TxBuffer,
                                     uint32_t ui32NumBytes,
                                     bool bWaitForCompletion)
{
  am_hal_mspi_dma_transfer_t    Transaction;
  volatile bool                 bDMAComplete = false;
  am_devices_mspi_rm67162_t *pDisplay = (am_devices_mspi_rm67162_t *)pHandle;

  //
  // Enable DCX for DMA Transactions.
  //
  if (am_hal_mspi_control(pDisplay->pMspiHandle, AM_HAL_MSPI_REQ_DCX_EN, 0))
  {
    return AM_DEVICES_RM67162_STATUS_ERROR;
  }

  //
  // Create the transaction.
  //
  Transaction.ui8Priority               = 1;
  Transaction.eDirection                = AM_HAL_MSPI_TX;
  Transaction.ui32TransferCount         = ui32NumBytes;
  Transaction.ui32DeviceAddress         = 0;
  Transaction.ui32SRAMAddress           = (uint32_t)pui8TxBuffer;
  Transaction.ui32PauseCondition        = 0;
  Transaction.ui32StatusSetClr          = 0;

  //
  // Execute the transction over MSPI.
  //
  if (am_hal_mspi_nonblocking_transfer(pDisplay->pMspiHandle,
                                       &Transaction,
                                       AM_HAL_MSPI_TRANS_DMA,
                                       pfnMSPI_RM67162_Callback,
                                       (void *)&bDMAComplete))
  {
    return AM_DEVICES_RM67162_STATUS_ERROR;
  }

  if (bWaitForCompletion)
  {

    // Wait for DMA Complete or Timeout
    for (uint32_t i = 0; i < AM_DEVICES_MSPI_TIMEOUT; i++)
    {
      if (bDMAComplete)
      {
        break;
      }
      //
      // Call the BOOTROM cycle function to delay for about 1 microsecond.
      //
      am_hal_flash_delay( FLASH_CYCLES_US(1) );
    }

    // Check the status.
    if (!bDMAComplete)
    {
      return AM_DEVICES_RM67162_STATUS_ERROR;
    }

  }
  return AM_DEVICES_RM67162_STATUS_SUCCESS;
}

//*****************************************************************************
//
//  Programs the given range of display addresses.
//
//*****************************************************************************
uint32_t
am_devices_rm67162_nonblocking_write_adv(void *pHandle,
                                         uint8_t *pui8TxBuffer,
                                         uint32_t ui32NumBytes,
                                         uint32_t ui32PauseCondition,
                                         uint32_t ui32StatusSetClr,
                                         am_hal_mspi_callback_t pfnCallback,
                                         void *pCallbackCtxt)
{
  am_hal_mspi_dma_transfer_t Transaction;
  am_devices_mspi_rm67162_t *pDisplay = (am_devices_mspi_rm67162_t *)pHandle;

  //
  // Create the transaction.
  //
  Transaction.ui8Priority               = 1;
  Transaction.eDirection                = AM_HAL_MSPI_TX;
  Transaction.ui32TransferCount         = ui32NumBytes;
  Transaction.ui32DeviceAddress         = 0;
  Transaction.ui32SRAMAddress           = (uint32_t)pui8TxBuffer;
  Transaction.ui32PauseCondition        = ui32PauseCondition;
  Transaction.ui32StatusSetClr          = ui32StatusSetClr;

  //    am_hal_gpio_state_write(AM_BSP_GPIO_IOM0_DCX, AM_HAL_GPIO_OUTPUT_CLEAR);
  //
  // Execute the transction over IOM.
  //
  if (am_hal_mspi_nonblocking_transfer(pDisplay->pMspiHandle,
                                       &Transaction,
                                       AM_HAL_MSPI_TRANS_DMA,
                                       pfnCallback,
                                       pCallbackCtxt))
  {
    return AM_DEVICES_RM67162_STATUS_ERROR;
  }

  return AM_DEVICES_RM67162_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Reads the contents of the display into a buffer.
//
//*****************************************************************************
uint32_t
am_devices_rm67162_nonblocking_read(void *pHandle,
                                    uint8_t *pui8RxBuffer,
                                    uint32_t ui32NumBytes,
                                    bool bWaitForCompletion)
{
  am_hal_mspi_dma_transfer_t    Transaction;
  volatile bool                 bDMAComplete = false;
  am_devices_mspi_rm67162_t *pDisplay = (am_devices_mspi_rm67162_t *)pHandle;

  //
  // Create the transaction.
  //
  Transaction.ui8Priority               = 1;
  Transaction.eDirection                = AM_HAL_MSPI_RX;
  Transaction.ui32TransferCount         = ui32NumBytes;
  Transaction.ui32DeviceAddress         = 0;
  Transaction.ui32SRAMAddress           = (uint32_t)pui8RxBuffer;
  Transaction.ui32PauseCondition        = 0;
  Transaction.ui32StatusSetClr          = 0;

  am_hal_mspi_control(pDisplay->pMspiHandle, AM_HAL_MSPI_REQ_CLOCK_CONFIG, &g_MaxReadFreq);

  //
  // Execute the transction over MSPI.
  //
  if (am_hal_mspi_nonblocking_transfer(pDisplay->pMspiHandle,
                                       &Transaction,
                                       AM_HAL_MSPI_TRANS_DMA,
                                       pfnMSPI_RM67162_Callback,
                                       (void *)&bDMAComplete))
  {
    am_hal_mspi_control(pDisplay->pMspiHandle, AM_HAL_MSPI_REQ_CLOCK_CONFIG, &pDisplay->eClockFreq);
    return AM_DEVICES_RM67162_STATUS_ERROR;
  }

  if (bWaitForCompletion)
  {
    // Wait for DMA Complete or Timeout
    for (uint32_t i = 0; i < AM_DEVICES_MSPI_TIMEOUT; i++)
    {
      if (bDMAComplete)
      {
        break;
      }
      //
      // Call the BOOTROM cycle function to delay for about 1 microsecond.
      //
      am_hal_flash_delay( FLASH_CYCLES_US(1) );
    }

    // Check the status.
    if (!bDMAComplete)
    {
      am_hal_mspi_control(pDisplay->pMspiHandle, AM_HAL_MSPI_REQ_CLOCK_CONFIG, &pDisplay->eClockFreq);
      return AM_DEVICES_RM67162_STATUS_ERROR;
    }
  }
  am_hal_mspi_control(pDisplay->pMspiHandle, AM_HAL_MSPI_REQ_CLOCK_CONFIG, &pDisplay->eClockFreq);
  return AM_DEVICES_RM67162_STATUS_SUCCESS;
}

static uint32_t
am_devices_set_row_col(void *pHandle, am_devices_rm67162_graphic_conf_t *psGraphic_conf)
{
    uint8_t data[10] = {0};

    gs_display_info.row_start = psGraphic_conf->row_offset;
    gs_display_info.row_end = psGraphic_conf->max_row + psGraphic_conf->row_offset - 1;
    gs_display_info.col_start = psGraphic_conf->col_offset;
    gs_display_info.col_end = psGraphic_conf->max_col + psGraphic_conf->col_offset - 1;

    /* set column start address */
    data[0] = (gs_display_info.col_start / 256);
    data[1] = (gs_display_info.col_start % 256);
    data[2] = (gs_display_info.col_end / 256);
    data[3] = (gs_display_info.col_end % 256);
    if (am_devices_rm67162_command_write(pHandle, AM_DEVICES_RM67162_COLUMN_ADDR_SETTING, data, 4))//Column
    {
        return AM_DEVICES_RM67162_STATUS_ERROR;
    }

    /* set row start address */
    data[0] = (gs_display_info.row_start / 256);
    data[1] = (gs_display_info.row_start % 256);
    data[2] = (gs_display_info.row_end / 256);
    data[3] = (gs_display_info.row_end % 256);
    if (am_devices_rm67162_command_write(pHandle, AM_DEVICES_RM67162_ROW_ADDR_SETTING, data, 4))//raw
    {
        return AM_DEVICES_RM67162_STATUS_ERROR;
    }

    return AM_DEVICES_RM67162_STATUS_SUCCESS;
}

static uint32_t
am_devices_lcm_init(void *pHandle, am_devices_rm67162_graphic_conf_t *psGraphic_conf)
{
    uint8_t data[10] = {0};

    /*     Tearing effect line ON */
    data[0] = 0x0;
    if (am_devices_rm67162_command_write(pHandle, AM_DEVICES_RM67162_TEARING_EFFECT_LINE_ON, data, 1))
    {
        return AM_DEVICES_RM67162_STATUS_ERROR;
    }

    data[0] = psGraphic_conf->bus_mode;
    if (am_devices_rm67162_command_write(pHandle, AM_DEVICES_RM67162_SET_DSPI_MODE, data, 1))
    {
        return AM_DEVICES_RM67162_STATUS_ERROR;
    }

    data[0] = psGraphic_conf->color_mode ;
    if (am_devices_rm67162_command_write(pHandle, AM_DEVICES_RM67162_DATA_FORMAT_SEL, data, 1))
    {
        return AM_DEVICES_RM67162_STATUS_ERROR;
    }

    data[0] = psGraphic_conf->scan_mode;
    if (am_devices_rm67162_command_write(pHandle, AM_DEVICES_RM67162_SCAN_MODE, data, 1))
    {
        return AM_DEVICES_RM67162_STATUS_ERROR;
    }

    data[0] = 0x20;
    if (am_devices_rm67162_command_write(pHandle, AM_DEVICES_RM67162_SET_WRITE_DISPLAY_CTRL, data, 1))
    {
        return AM_DEVICES_RM67162_STATUS_ERROR;
    }

    //
    // Set row/col start/end addresses.
    //
    am_devices_set_row_col(pHandle, psGraphic_conf);

    /* set tear scan-line */
    data[0] = 0x00;
    data[1] = 0x28; // 0xf0
    if ( am_devices_rm67162_command_write(pHandle, AM_DEVICES_RM67162_SET_TEAR_SCANLINE, data, 2) )
    {
        return AM_DEVICES_RM67162_STATUS_ERROR;
    }
    if ( am_devices_rm67162_command_write(pHandle, AM_DEVICES_RM67162_SLEEP_OUT, NULL, 0) )
    {
        return AM_DEVICES_RM67162_STATUS_ERROR;
    }

    am_util_delay_ms(130);

    if ( am_devices_rm67162_command_write(pHandle, AM_DEVICES_RM67162_DISPLAY_ON, NULL, 0) )
    {
        return AM_DEVICES_RM67162_STATUS_ERROR;
    }

    am_util_delay_ms(200);

    //    am_hal_gpio_state_write(AM_BSP_GPIO_DISPLAY_BL, AM_HAL_GPIO_OUTPUT_CLEAR);

    am_util_stdio_printf("AM_BSP_GPIO_DISPLAY_BL set on \n");

    return AM_DEVICES_RM67162_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Initialize the rm67162 driver.
//
//*****************************************************************************
uint32_t
am_devices_mspi_rm67162_init(uint32_t ui32Module,
                             am_devices_mspi_rm67162_config_t *psMSPISettings,
                             void **ppHandle,
                             void **ppMspiHandle)
{
  uint32_t      ui32Status;
  uint32_t      ui32DeviceID;
  uint32_t      ui32Index = 0;
  am_hal_mspi_dev_config_t    mspiDevCfg;
  void                       *pMspiHandle;

  if ((ui32Module > AM_REG_MSPI_NUM_MODULES) || (psMSPISettings == NULL))
  {
    return AM_DEVICES_RM67162_STATUS_ERROR;
  }

  // Allocate a vacant device handle
  for ( ui32Index = 0; ui32Index < AM_DEVICES_MSPI_RM67162_MAX_DEVICE_NUM; ui32Index++ )
  {
      if ( gAmRm67162[ui32Index].bOccupied == false )
      {
          break;
      }
  }
  if ( ui32Index == AM_DEVICES_MSPI_RM67162_MAX_DEVICE_NUM )
  {
      return AM_DEVICES_RM67162_STATUS_ERROR;
  }

  //
  // Re-Configure the MSPI for the requested operation mode.
  //
  mspiDevCfg = SerialDisplayMSPICfg;
  mspiDevCfg.eClockFreq = psMSPISettings->eClockFreq;
  //mspiDevCfg.eDeviceConfig = psMSPISettings->eDeviceConfig;
  mspiDevCfg.ui32TCBSize = psMSPISettings->ui32NBTxnBufLength;
  mspiDevCfg.pTCB = psMSPISettings->pNBTxnBuf;
  mspiDevCfg.scramblingStartAddr = psMSPISettings->ui32ScramblingStartAddr;
  mspiDevCfg.scramblingEndAddr = psMSPISettings->ui32ScramblingEndAddr;

  //
  // Configure the MSPI pins.
  //
  am_bsp_mspi_pins_enable(ui32Module, mspiDevCfg.eDeviceConfig);

  //
  // Initialize the MSPI instance.
  // Enable power to the MSPI instance.
  // Configure the MSPI for Serial operation during initialization.
  // Enable the MSPI.
  // HAL Success return is 0
  //
  if ( am_hal_mspi_initialize(ui32Module, &pMspiHandle)                      ||
       am_hal_mspi_power_control(pMspiHandle, AM_HAL_SYSCTRL_WAKE, false)    ||
       am_hal_mspi_device_configure(pMspiHandle, &mspiDevCfg)                ||
       am_hal_mspi_enable(pMspiHandle) )
  {
    return AM_DEVICES_RM67162_STATUS_ERROR;
  }
  else
  {
    //
    // Enable DCX for DMA Transactions.
    //
    if ( am_hal_mspi_control(pMspiHandle, AM_HAL_MSPI_REQ_DCX_EN, 0) )
    {
      return AM_DEVICES_RM67162_STATUS_ERROR;
    }
    //
    // Enable MSPI interrupts.
    //
    ui32Status = am_hal_mspi_interrupt_clear(pMspiHandle, AM_HAL_MSPI_INT_CQUPD | AM_HAL_MSPI_INT_ERR );
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_RM67162_STATUS_ERROR;
    }

    ui32Status = am_hal_mspi_interrupt_enable(pMspiHandle, AM_HAL_MSPI_INT_CQUPD | AM_HAL_MSPI_INT_ERR );
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_RM67162_STATUS_ERROR;
    }

    gAmRm67162[ui32Index].pMspiHandle = pMspiHandle;
    gAmRm67162[ui32Index].ui32Module = ui32Module;
    gAmRm67162[ui32Index].eClockFreq = psMSPISettings->eClockFreq;
    *ppMspiHandle = pMspiHandle;
    *ppHandle = (void *)&gAmRm67162[ui32Index];

    //
    // Read the Device ID.
    //
    am_devices_rm67162_read_id((void*)&gAmRm67162[ui32Index], &ui32DeviceID);
    am_util_stdio_printf("RM67167 Device ID = %6X\n", (ui32DeviceID & 0x00FFFFFF));

    //
    // Device specific TFT display initialization.
    //
    am_util_delay_ms(500);
    am_devices_rm67162_reset((void*)&gAmRm67162[ui32Index]);
    ui32Status = am_devices_lcm_init((void*)&gAmRm67162[ui32Index], &g_sGraphic_conf);
    if (AM_DEVICES_RM67162_STATUS_SUCCESS != ui32Status)
    {
      return AM_DEVICES_RM67162_STATUS_ERROR;
    }
    gAmRm67162[ui32Index].bOccupied = true;

    //am_hal_gpio_state_write(AM_BSP_GPIO_DISPLAY_BL, AM_HAL_GPIO_OUTPUT_SET);
    //
    // Return the status.
    //
    return AM_DEVICES_RM67162_STATUS_SUCCESS;
  }
}

uint32_t
am_devices_mspi_rm67162_row_col_reset(void *pHandle)
{
  uint32_t ui32Status = am_devices_set_row_col(pHandle, &g_sGraphic_conf);
  if (AM_DEVICES_RM67162_STATUS_SUCCESS != ui32Status)
  {
    return AM_DEVICES_RM67162_STATUS_ERROR;
  }
  return AM_DEVICES_RM67162_STATUS_SUCCESS;
}

//*****************************************************************************
//
// De-Initialize the rm67162 driver.
//
//*****************************************************************************
uint32_t
am_devices_rm67162_term(void *pHandle)
{
    am_devices_mspi_rm67162_t *pDisplay = (am_devices_mspi_rm67162_t *)pHandle;

    if ( pDisplay->ui32Module > AM_REG_IOM_NUM_MODULES )
    {
        return AM_DEVICES_RM67162_STATUS_ERROR;
    }

    //    am_hal_gpio_interrupt_clear(AM_HAL_GPIO_MASKBIT(pGpioIntMask, AM_BSP_GPIO_DISPLAY_TE));
    //    am_hal_gpio_interrupt_disable(AM_HAL_GPIO_MASKBIT(pGpioIntMask, AM_BSP_GPIO_DISPLAY_TE));
    //    NVIC_DisableIRQ(GPIO_IRQn);

    // Disable the pins
    //    am_bsp_iom_display_pins_disable(AM_BSP_4_WIRES_SPI_MODE);

    //
    // Disable the MSPI.
    //
    am_hal_mspi_disable(pDisplay->pMspiHandle);

    //
    // Disable power to and uninitialize the MSPI instance.
    //
    am_hal_mspi_power_control(pDisplay->pMspiHandle, AM_HAL_SYSCTRL_DEEPSLEEP, false);

    am_hal_mspi_deinitialize(pDisplay->pMspiHandle);

    // Free this device handle
    pDisplay->bOccupied = false;

    //
    // Return the status.
    //
    return AM_DEVICES_RM67162_STATUS_SUCCESS;
}
//*****************************************************************************
//
//
//*****************************************************************************
uint32_t
am_devices_rm67162_read_id(void *pHandle, uint32_t *pdata)
{
    if (am_devices_rm67162_command_read(pHandle, AM_DEVICES_RM67162_READ_ID, pdata, 3, true))
    {
        return AM_DEVICES_RM67162_STATUS_ERROR;
    }

    return AM_DEVICES_RM67162_STATUS_SUCCESS;
}

//*****************************************************************************
//
//
//*****************************************************************************
uint32_t
am_devices_mspi_rm67162_set_transfer_window(void *pHandle, uint32_t startRow, uint32_t startCol, uint32_t endRow, uint32_t endCol)
{
    uint8_t data[4] = {0};

    /* set column start address */
    data[0] = (startCol / 256);
    data[1] = (startCol % 256);
    data[2] = (endCol / 256);
    data[3] = (endCol % 256);
    if (am_devices_rm67162_command_write(pHandle, AM_DEVICES_RM67162_COLUMN_ADDR_SETTING, data, 4))//Column
    {
        return AM_DEVICES_RM67162_STATUS_ERROR;
    }

    /* set row start address */
    data[0] = (startRow / 256);
    data[1] = (startRow % 256);
    data[2] = (endRow / 256);
    data[3] = (endRow % 256);
    if (am_devices_rm67162_command_write(pHandle, AM_DEVICES_RM67162_ROW_ADDR_SETTING, data, 4))//raw
    {
        return AM_DEVICES_RM67162_STATUS_ERROR;
    }

    return AM_DEVICES_RM67162_STATUS_SUCCESS;
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************

