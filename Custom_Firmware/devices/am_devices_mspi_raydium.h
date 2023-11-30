//*****************************************************************************
//
//! @file am_devices_mspi_raydium.h
//!
//! @brief General Multibit SPI Display driver.
//!
//! @addtogroup mspi_rm69330 RM69330 MSPI Display Driver
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

#ifndef AM_DEVICES_MSPI_RM69330_H
#define AM_DEVICES_MSPI_RM69330_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "am_mcu_apollo.h"
#include "am_util_stdio.h"
#include "am_util_delay.h"
//*****************************************************************************
//
//! @name Display Dimensions
//! @{
//
//*****************************************************************************
#define AM_DEVICES_RM69330_NUM_ROWS                         454
#define AM_DEVICES_RM69330_NUM_COLUMNS                      454
#define AM_DEVICES_RM69330_SCAN_MODE_ORDER_RGB              0x00
#define AM_DEVICES_RM69330_SCAN_MODE_ORDER_BGR              0x08

//! @}

//*****************************************************************************
//
//! @name Global definitions for DISPLAY commands
//! @{
//
//*****************************************************************************
#define AM_DEVICES_MSPI_RM69330_CMD_WRITE                   0x02
#define AM_DEVICES_MSPI_RM69330_PIXEL_WRITE_ADDR1           0x32    // address on single line
#define AM_DEVICES_MSPI_RM69330_PIXEL_WRITE_ADDR4           0x12    // address on quad interface
#define AM_DEVICES_MSPI_RM69330_CMD_READ                    0x03
#define AM_DEVICES_MSPI_RM69330_READ_ID                     0x04

#define AM_DEVICES_MSPI_RM69330_SOFTWARE_RESET              0x01
#define AM_DEVICES_MSPI_RM69330_PAGE_PROGRAM                0x02
#define AM_DEVICES_MSPI_RM69330_READ                        0x03
#define AM_DEVICES_MSPI_RM69330_WRITE_DISABLE               0x04
#define AM_DEVICES_MSPI_RM69330_READ_STATUS                 0x05
#define AM_DEVICES_MSPI_RM69330_WRITE_ENABLE                0x06
#define AM_DEVICES_MSPI_RM69330_FAST_READ                   0x0B
#define AM_DEVICES_MSPI_RM69330_READ_PIXEL_FORMAT           0x0C
#define AM_DEVICES_MSPI_RM69330_SLEEP_IN                    0x10
#define AM_DEVICES_MSPI_RM69330_SLEEP_OUT                   0x11
#define AM_DEVICES_MSPI_RM69330_NORMAL_MODE_ON              0x13
#define AM_DEVICES_MSPI_RM69330_INVERSION_OFF               0x20
#define AM_DEVICES_MSPI_RM69330_DISPLAY_OFF                 0x28
#define AM_DEVICES_MSPI_RM69330_DISPLAY_ON                  0x29
#define AM_DEVICES_MSPI_RM69330_SET_COLUMN                  0x2A
#define AM_DEVICES_MSPI_RM69330_SET_ROW                     0x2B
#define AM_DEVICES_MSPI_RM69330_MEM_WRITE                   0x2C
#define AM_DEVICES_MSPI_RM69330_TE_LINE_OFF                 0x34
#define AM_DEVICES_MSPI_RM69330_TE_LINE_ON                  0x35
#define AM_DEVICES_MSPI_RM69330_SCAN_DIRECTION              0x36
#define AM_DEVICES_MSPI_RM69330_IDLE_MODE_OFF               0x38
#define AM_DEVICES_MSPI_RM69330_PIXEL_FORMAT                0x3A
#define AM_DEVICES_MSPI_RM69330_DUAL_READ                   0x3B
#define AM_DEVICES_MSPI_RM69330_MEM_WRITE_CONTINUE          0x3C
#define AM_DEVICES_MSPI_RM69330_SET_TEAR_SCANLINE           0x44
#define AM_DEVICES_MSPI_RM69330_WRITE_DISPLAY_BRIGHTNESS    0x51
#define AM_DEVICES_MSPI_RM69330_WRITE_ENHVOL_CFG            0x61
#define AM_DEVICES_MSPI_RM69330_RESET_ENABLE                0x66
#define AM_DEVICES_MSPI_RM69330_QUAD_READ                   0x6B
#define AM_DEVICES_MSPI_RM69330_WRITE_VOL_CFG               0x81
#define AM_DEVICES_MSPI_RM69330_RESET_MEMORY                0x99
#define AM_DEVICES_MSPI_RM69330_ENTER_4B                    0xB7
#define AM_DEVICES_MSPI_RM69330_SET_DSPI_MODE               0xC4
#define AM_DEVICES_MSPI_RM69330_BULK_ERASE                  0xC7
#define AM_DEVICES_MSPI_RM69330_SECTOR_ERASE                0xD8
#define AM_DEVICES_MSPI_RM69330_EXIT_4B                     0xE9
#define AM_DEVICES_MSPI_RM69330_QUAD_IO_READ                0xEB
#define AM_DEVICES_MSPI_RM69330_READ_QUAD_4B                0xEC
#define AM_DEVICES_MSPI_RM69330_CMD_MODE                    0xFE

#define AM_DEVICES_MSPI_RM69330_SPI_WRAM                    0x80
#define AM_DEVICES_MSPI_RM69330_DSPI_WRAM                   0x81

#define AM_DEVICES_MSPI_RM69330_COLOR_MODE_8BIT             0x72
#define AM_DEVICES_MSPI_RM69330_COLOR_MODE_3BIT             0x73
#define AM_DEVICES_MSPI_RM69330_COLOR_MODE_16BIT            0x75
#define AM_DEVICES_MSPI_RM69330_COLOR_MODE_18BIT            0x76
#define AM_DEVICES_MSPI_RM69330_COLOR_MODE_24BIT            0x77

#define AM_DEVICES_MSPI_RM69330_SCAN_MODE_0                 0x40
#define AM_DEVICES_MSPI_RM69330_SCAN_MODE_90                0x70
#define AM_DEVICES_MSPI_RM69330_SCAN_MODE_180               0x10
#define AM_DEVICES_MSPI_RM69330_SCAN_MODE_270               0x00

#define AM_DEVICES_MSPI_RM69330_MAX_DEVICE_NUM              1
//! @}

//*****************************************************************************
//
//! @name Global type definitions.
//! @{
//
//*****************************************************************************

typedef enum
{
    AM_DEVICES_MSPI_RM69330_STATUS_SUCCESS,
    AM_DEVICES_MSPI_RM69330_STATUS_ERROR
} am_devices_mspi_rm69330_status_t;
typedef struct
{
    uint8_t ui8BusMode;
    uint8_t ui8ColorMode;
    uint8_t ui8ScanMode;

    uint16_t ui16Height;
    uint16_t ui16Width;
    uint16_t ui16RowOffset;
    uint16_t ui16ColumnOffset;
} am_devices_mspi_rm69330_graphic_conf_t;

typedef struct
{
    am_hal_mspi_device_e eDeviceConfig;
    am_hal_mspi_clock_e eClockFreq;
    uint32_t *pNBTxnBuf;
    uint32_t ui32NBTxnBufLength;
} am_devices_mspi_rm69330_config_t;
//! @}

//*****************************************************************************
//
// External function definitions.
//
//*****************************************************************************
//*****************************************************************************
//
//! @brief Initialize the MSPI_RM69330 driver.
//!
//! @param ui32Module       - MSPI module.
//! @param pDevCfg          - device configuration.
//! @param ppHandle         - MSPI module handle
//! @param ppMspiHandle     - MSPI handle's handle.
//!
//! This function should be called before any other am_devices_MSPI_RM69330
//! functions. It is used to set tell the other functions how to communicate
//! with the external screen hardware.
//!
//! The \e pfnWriteFunc and \e pfnReadFunc variables may be used to provide
//! alternate implementations of SPI write and read functions respectively. If
//! they are left set to 0, the default functions am_hal_iom_spi_write() and
//! am_hal_iom_spi_read() will be used.
//!
//! @return None.
//
//*****************************************************************************
extern uint32_t am_devices_mspi_rm69330_init(uint32_t ui32Module,
                                             am_devices_mspi_rm69330_config_t *pDevCfg,
                                             void **ppHandle,
                                             void **ppMspiHandle);

//*****************************************************************************
//
//! @brief De-Initialize the mspi_rm69330 driver.
//!
//! @param pHandle              - mspi handle
//!
//! This function reverses the initialization
//!
//! @return Status.
//
//*****************************************************************************
extern uint32_t am_devices_mspi_rm69330_term(void *pHandle);

//*****************************************************************************
//
//! @brief
//!
//! @param pHandle
//!
//! @return
//
//*****************************************************************************
extern uint32_t am_devices_mspi_rm69330_display_off(void *pHandle);

//*****************************************************************************
//
//! @brief
//!
//! @param pHandle
//!
//! @return
//
//*****************************************************************************
extern uint32_t am_devices_mspi_rm69330_display_on(void *pHandle);

//*****************************************************************************
//
//! @brief set recommended scanline.
//!
//! @param pHandle                       - mspi handle
//! @param TETimesPerFrame            - how many TE intervals transfer one frame
//!
//! This function used to set recommended scanline,it's valid when TETimesPerFrame
//! equal to 1 or 2.
//!
//! @return Status.
//
//****************************************************************************
extern uint32_t am_devices_mspi_rm69330_set_scanline_recommended_parameter(void *pHandle,
                                                                           uint8_t TETimesPerFrame);

//*****************************************************************************
//
//! @brief set refresh scanline.
//!
//! @param pHandle                       - mspi handle
//! @param ui16ScanLine                  - scanline index
//!
//! This function used to set scanline.it will return error when scanline
//! over display maximum resolution.
//!
//! @return Status.
//
//****************************************************************************
extern uint32_t am_devices_mspi_rm69330_set_scanline(void *pHandle,
                                                     uint16_t ui16ScanLine);

//*****************************************************************************
//
//! @brief Programs the given range of display addresses.
//!
//! @param pHandle              - mspi handle
//! @param pui8TxBuffer         - Buffer to write the data from
//! @param ui32NumBytes         - Number of bytes to write to the display memory
//! @param bWaitForCompletion   - Waits for CQ/DMA to complete before return.
//! @param bContinue            - memories write or write continue.
//!
//! This function uses the data in the provided pui8TxBuffer and copies it to
//! the external flash at the address given by ui32WriteAddress. It will copy
//! exactly ui32NumBytes of data from the original pui8TxBuffer pointer. The
//! user is responsible for ensuring that they do not overflow the target flash
//! memory or underflow the pui8TxBuffer array
//!
//! @return 32-bit status
//
//*****************************************************************************
extern uint32_t am_devices_mspi_rm69330_nonblocking_write(void *pHandle,
                                                          const uint8_t *pui8TxBuffer,
                                                          uint32_t ui32NumBytes,
                                                          bool bWaitForCompletion,
                                                          bool bContinue);

//! @brief Programs the given range of display addresses.
//!
//! @param pHandle              - MSPI handle
//! @param pui8TxBuffer         - Buffer to write the data from
//! @param ui32NumBytes         - Number of bytes to write to the display memory
//! @param ui32PauseCondition   - CQ Pause condition before execution.
//! @param ui32StatusSetClr     - CQ Set/Clear condition after execution.
//! @param pfnCallback          - Callback function after execution.
//! @param pCallbackCtxt        - Callback context after execution.
//! @param bContinue            - memories write or write continue.
//!
//! This function uses the data in the provided pui8TxBuffer and copies it to
//! the external display the address given by ui32WriteAddress. It will copy
//! exactly ui32NumBytes of data from the original pui8TxBuffer pointer. The
//! user is responsible for ensuring that they do not overflow the target display
//! memory or underflow the pui8TxBuffer array
//
//! @return 32-bit status
//
//*****************************************************************************
extern uint32_t am_devices_mspi_rm69330_nonblocking_write_adv(void *pHandle,
                                                             uint8_t *pui8TxBuffer,
                                                             uint32_t ui32NumBytes,
                                                             uint32_t ui32PauseCondition,
                                                             uint32_t ui32StatusSetClr,
                                                             am_hal_mspi_callback_t pfnCallback,
                                                             void *pCallbackCtxt,
                                                             bool bContinue);

//*****************************************************************************
//
//! @brief Programs the given range of display addresses.
//!
//! @param pHandle              - MSPI handle
//! @param pui8TxBuffer         - Buffer to write the data from
//! @param ui32NumBytes         - Number of bytes to write to the display memory
//! @param ui32PauseCondition   - CQ Pause condition before execution.
//! @param ui32StatusSetClr     - CQ Set/Clear condition after execution.
//! @param pfnCallback          - Callback function after execution.
//! @param pCallbackCtxt        - Callback context after execution.
//! @param bContinue            - memories write or write continue.
//! @param bReverseBytes        - reverse high-byte with low-byte in half-word format
//!
//! This function uses the data in the provided pui8TxBuffer and copies it to
//! the external display the address given by ui32WriteAddress. It will copy
//! exactly ui32NumBytes of data from the original pui8TxBuffer pointer with
//! high-byte and low-byte reversed in halfword format. The user is responsible
//! for ensuring that they do not overflow the target display memory or underflow
//! the pui8TxBuffer array
//!
//! @return 32-bit status
//
//*****************************************************************************
extern uint32_t am_devices_mspi_rm69330_nonblocking_write_endian(void *pHandle,
                                                                uint8_t *pui8TxBuffer,
                                                                uint32_t ui32NumBytes,
                                                                uint32_t ui32PauseCondition,
                                                                uint32_t ui32StatusSetClr,
                                                                am_hal_mspi_callback_t pfnCallback,
                                                                void *pCallbackCtxt,
                                                                bool bContinue,
                                                                bool bReverseBytes);

//*****************************************************************************
//
//! @brief
//! @param pHandle
//! @return
//
//*****************************************************************************
extern uint32_t am_devices_mspi_rm69330_row_col_reset(void *pHandle);

//*****************************************************************************
//
//! @brief
//! @param pHandle
//! @param pdata
//! @return
//
//*****************************************************************************
extern uint32_t am_devices_mspi_rm69330_read_id(void *pHandle,
                                                uint32_t *pdata);

//*****************************************************************************
//
//! @brief
//! @param pHandle
//! @param ui16ColumnStart
//! @param ui16ColumnSize
//! @param ui16RowStart
//! @param ui16RowSize
//! @return
//
//*****************************************************************************
extern uint32_t am_devices_mspi_rm69330_set_transfer_window(void *pHandle,
                                                            uint16_t ui16ColumnStart,
                                                            uint16_t ui16ColumnSize,
                                                            uint16_t ui16RowStart,
                                                            uint16_t ui16RowSize);

//*****************************************************************************
//
//! @brief
//! @param ui16ColumnStart
//! @param ui16ColumnSize
//! @param ui16RowStart
//! @param ui16RowSize
//! @param ui8Format
//
//*****************************************************************************
extern void am_devices_rm69330_set_parameters(uint16_t ui16ColumnStart,
                                              uint16_t ui16ColumnSize,
                                              uint16_t ui16RowStart,
                                              uint16_t ui16RowSize,
                                              uint8_t ui8Format);

//*****************************************************************************
//
//! @brief
//! @param pHandle
//! @param pdata
//! @return
//
//*****************************************************************************
extern uint32_t am_devices_mspi_rm69330_read_format(void *pHandle,
                                                    uint32_t *pdata);

//*****************************************************************************
//
//! @brief
//! @param pHandle
//! @return
//
//*****************************************************************************
extern uint32_t am_devices_rm69330_get_parameters(void *pHandle);

#ifdef __cplusplus
}
#endif

#endif // AM_DEVICES_MSPI_RM69330_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************

